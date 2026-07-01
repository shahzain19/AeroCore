/**
 * @file SimulationEngine.cpp
 * @brief Core simulation world implementation.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Simulation/SimulationEngine.h"
#include "Core/ComplementaryEstimator.h"
#include "platforms/sim/SimIMU.h"
#include "platforms/sim/SimBarometer.h"
#include "Utilities/Logger.h"

#include <fstream>
#include <vector>

namespace AeroCore {
namespace Simulation {

std::string SimulationEngine::findConfigFile(const std::string& hint) {
    const std::vector<std::string> candidates = {
        hint,
        "config/simulation.toml",
        "../config/simulation.toml",
        "../../config/simulation.toml",
    };
    for (const auto& p : candidates) {
        if (std::ifstream(p).good()) return p;
    }
    throw std::runtime_error(
        "Cannot find config file. Tried: " + hint +
        ", config/simulation.toml, ../config/simulation.toml");
}

SimulationEngine::SimulationEngine(const std::string& config_hint)
    : config_path_(findConfigFile(config_hint))
    , config_(config_path_)
    , physics_(config_)
{
    auto& log = Utilities::Logger::getInstance();
    log.info("AeroCore starting — config: " + config_path_);

    drone_ = std::make_shared<Flight::Drone>(config_);
    const size_t motor_count = drone_->getRecommendedMotorCount();
    for (size_t i = 0; i < motor_count; ++i) {
        drone_->addMotor(std::make_unique<Flight::Motor>(config_));
    }
    log.info("Drone created — mass=" + std::to_string(drone_->getMass()) + " kg, " +
             std::to_string(drone_->getMotorCount()) + " motors");

    imu_            = std::make_shared<Sensors::IMU>(config_);
    altimeter_      = std::make_shared<Sensors::Altimeter>(50.0, 0.05, 0.001);
    battery_sensor_ = std::make_shared<Sensors::BatterySensor>(10.0, 0.02, 0.0);

    estimator_ = std::make_unique<Core::ComplementaryEstimator>(config_);
    sim_imu_   = std::make_unique<Platform::Sim::SimIMU>(imu_);
    sim_baro_  = std::make_unique<Platform::Sim::SimBarometer>(altimeter_);
    sim_rc_    = std::make_unique<Platform::Sim::SimRCInput>();

    try {
        const auto v = config_.get<std::string>("simulation", "perfect_state");
        perfect_state_ = (v == "true" || v == "1");
    } catch (...) {
        perfect_state_ = true;
    }

    flight_controller_ = std::make_unique<Flight::FlightController>(
        drone_, imu_, altimeter_, battery_sensor_, *estimator_, config_);
    flight_controller_->setRCInput(sim_rc_.get());

    physics_.setInertiaTensor(drone_->getInertiaTensor());
    log.info("Physics engine initialised (RK4, NED frame)");

    wind_ = Math::Vector3d::Zero();
    try {
        wind_.x() = config_.get<double>("simulation", "wind_x");
        wind_.y() = config_.get<double>("simulation", "wind_y");
    } catch (...) {}

    drone_area_ = drone_->getSize() * drone_->getSize() * 0.25;
    try {
        drag_coeff_ = config_.get<double>("physics", "drag_coefficient");
    } catch (...) {}
    try {
        physics_dt_ = config_.get<double>("simulation", "dt");
    } catch (...) {}
}

void SimulationEngine::stepPhysics() {
    auto& state = drone_->getState();
    const double altitude = std::max(0.0, -state.position.z());
    const double rho = physics_.airDensityAtAltitude(altitude);

    const Math::Vector3d accel_world = state.acceleration;
    const Math::Vector3d gravity_world(0.0, 0.0, physics_.getGravity());
    const Math::Vector3d specific_force_world = accel_world - gravity_world;
    const Math::Vector3d accel_body =
        Math::worldToBody(state.orientation, specific_force_world);
    imu_->update(physics_dt_, accel_body, state.angular_vel, state.orientation);
    altimeter_->update(physics_dt_, drone_->getAltitude());
    battery_sensor_->update(physics_dt_, drone_->getBatteryVoltage());

    estimator_->predict(physics_dt_, sim_imu_->read());
    if (perfect_state_) {
        estimator_->setAttitudeEuler(Math::quaternionToEuler(state.orientation));
        estimator_->injectPerfectNavigation(state.position, state.velocity);
    } else {
        estimator_->setAttitudeRollPitch(imu_->getEstimatedRoll(),
                                         imu_->getEstimatedPitch());
        estimator_->clearPerfectNavigation();
    }
    estimator_->correctBaro(sim_baro_->read());

    flight_controller_->update(physics_dt_);
    drone_->update(physics_dt_, rho);

    const Physics::ExternalForces forces =
        drone_->buildExternalForces(state.orientation, wind_, rho);

    physics_.step(state, forces,
                  drone_->getMass(), drag_coeff_, drone_area_,
                  physics_dt_);

    sim_time_ += physics_dt_;
}

void SimulationEngine::reset() {
    drone_->reset();
    estimator_->reset();
    flight_controller_->reset();
    physics_.setInertiaTensor(drone_->getInertiaTensor());
    wind_     = Math::Vector3d::Zero();
    sim_time_ = 0.0;
    Utilities::Logger::getInstance().info("Simulation reset");
}

void SimulationEngine::gatherTelemetry(TelemetryData& telemetry) const {
    const auto& state = drone_->getState();
    const auto  euler = flight_controller_->getVehicleState().euler_rpy;
    const auto  diag  = flight_controller_->getDiagnostics();
    const double alt  = drone_->getAltitude();
    const double rho  = physics_.airDensityAtAltitude(alt);

    telemetry.simulation_time  = sim_time_;
    telemetry.position         = state.position;
    telemetry.velocity         = state.velocity;
    telemetry.acceleration     = state.acceleration;
    telemetry.altitude         = alt;
    telemetry.euler_angles     = euler;
    telemetry.angular_velocity = state.angular_vel;
    telemetry.target_altitude  = flight_controller_->getTargetAltitude();
    telemetry.flight_mode      = flight_controller_->getMode();
    telemetry.air_density      = rho;
    telemetry.wind_world       = wind_;

    double total_T = 0.0;
    for (size_t i = 0; i < drone_->getMotorCount(); ++i) {
        const auto& m = drone_->getMotor(i);
        total_T += m.getCurrentThrust();
        if (i < 8) {
            telemetry.motor_throttle[i] = m.getThrottle();
            telemetry.motor_rpm[i]      = m.getRPM();
        }
    }
    telemetry.current_thrust = total_T;

    telemetry.pid_alt_error      = diag.alt_error;
    telemetry.pid_alt_integral   = diag.alt_integral;
    telemetry.pid_alt_derivative = diag.alt_derivative;
    telemetry.pid_alt_output     = diag.alt_output;
    telemetry.pid_roll_output    = diag.roll_rate_error;
    telemetry.pid_pitch_output   = diag.pitch_rate_error;
    telemetry.pid_yaw_output     = diag.yaw_rate_error;

    telemetry.accel_sensor    = imu_->getAccelerometer().getAcceleration();
    telemetry.gyro_sensor     = imu_->getGyroscope().getAngularVelocity();
    telemetry.alt_sensor        = altimeter_->getAltitude();
    telemetry.battery_voltage   = drone_->getBatteryVoltage();
    telemetry.battery_soc       = drone_->getBatteryPercentage();
    telemetry.total_current     = drone_->getTotalCurrentDraw();
}

Flight::FlightController& SimulationEngine::flightController() {
    return *flight_controller_;
}

const Flight::FlightController& SimulationEngine::flightController() const {
    return *flight_controller_;
}

Flight::Drone& SimulationEngine::drone() { return *drone_; }
const Flight::Drone& SimulationEngine::drone() const { return *drone_; }

Physics::PhysicsEngine& SimulationEngine::physics() { return physics_; }

Math::Vector3d& SimulationEngine::wind() { return wind_; }
const Math::Vector3d& SimulationEngine::wind() const { return wind_; }

double SimulationEngine::simTime()   const { return sim_time_; }
double SimulationEngine::physicsDt() const { return physics_dt_; }
const std::string& SimulationEngine::configPath() const { return config_path_; }

bool SimulationEngine::perfectState() const { return perfect_state_; }

Core::ComplementaryEstimator& SimulationEngine::estimator() {
    return *estimator_;
}

const Core::ComplementaryEstimator& SimulationEngine::estimator() const {
    return *estimator_;
}

Platform::Sim::SimRCInput& SimulationEngine::rcInput() {
    return *sim_rc_;
}

const Platform::Sim::SimRCInput& SimulationEngine::rcInput() const {
    return *sim_rc_;
}

void SimulationEngine::setPerfectState(bool enabled) {
    perfect_state_ = enabled;
}

} // namespace Simulation
} // namespace AeroCore
