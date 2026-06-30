/**
 * @file main.cpp
 * @brief AeroCore simulation entry point.
 *
 * ## Simulation Loop
 *
 * AeroCore uses a fixed-timestep physics loop decoupled from the render rate:
 *
 * ```
 * while (running) {
 *     frame_dt = real_time_since_last_frame   (capped to avoid spiral of death)
 *     accumulator += frame_dt
 *
 *     while (accumulator >= physics_dt) {
 *         // 1. Build external forces from current motor state
 *         // 2. RK4 physics step
 *         // 3. Update drone motors
 *         // 4. Update sensors
 *         // 5. Run flight controller (PID loops, motor commands)
 *         accumulator -= physics_dt
 *     }
 *
 *     // 6. Gather telemetry
 *     // 7. Render frame
 * }
 * ```
 *
 * This ensures physics is always stable regardless of frame rate variations.
 *
 * ## Command-Line Arguments
 *
 *   AeroCore [config_file] [--headless]
 *
 *   config_file  Path to simulation.toml (default: config/simulation.toml)
 *   --headless   Disable rendering, print telemetry to stdout
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Utilities/Config.h"
#include "Utilities/Logger.h"
#include "Physics/PhysicsEngine.h"
#include "Flight/Drone.h"
#include "Flight/Motor.h"
#include "Flight/FlightController.h"
#include "Sensors/IMU.h"
#include "Sensors/Altimeter.h"
#include "Sensors/BatterySensor.h"
#include "Rendering/Renderer.h"
#include "Simulation/TelemetryManager.h"
#include "Math/Vector.h"

#include <SFML/System.hpp>
#include <memory>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <cstdlib>

using namespace AeroCore;

// ---------------------------------------------------------------------------
//  Helper: locate config file
// ---------------------------------------------------------------------------
static std::string findConfigFile(const std::string& hint) {
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

// ---------------------------------------------------------------------------
//  Main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    bool        headless    = false;
    std::string config_hint = "config/simulation.toml";
    double      max_sim_time = 60.0;
    double      headless_status_rate_hz = 5.0;
    bool        headless_debug = false;

    // Parse CLI arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--headless" || arg == "-h") {
            headless = true;
        } else if (arg == "--duration") {
            if (i + 1 >= argc) {
                std::cerr << "[AeroCore] Error: --duration requires a value (seconds)\n";
                return 1;
            }
            max_sim_time = std::max(0.5, std::atof(argv[++i]));
        } else if (arg == "--status-rate") {
            if (i + 1 >= argc) {
                std::cerr << "[AeroCore] Error: --status-rate requires a value (Hz)\n";
                return 1;
            }
            headless_status_rate_hz = std::max(0.1, std::atof(argv[++i]));
        } else if (arg == "--debug-headless") {
            headless_debug = true;
        } else if (!arg.empty() && arg.front() == '-') {
            std::cerr << "[AeroCore] Warning: unknown option ignored: " << arg << "\n";
        } else {
            config_hint = arg;
        }
    }

    try {
        // ---- Configuration ----
        const std::string config_path = findConfigFile(config_hint);
        std::cout << "[AeroCore] Config: " << config_path << "\n";
        Utilities::Config config(config_path);

        // ---- Logger ----
        auto& log = Utilities::Logger::getInstance();
        log.info("AeroCore v2.0 starting — config: " + config_path);

        // ---- Drone ----
        auto drone = std::make_shared<Flight::Drone>(config);

        // Add propulsion units from airframe configuration.
        const size_t motor_count = drone->getRecommendedMotorCount();
        for (size_t i = 0; i < motor_count; ++i) {
            drone->addMotor(std::make_unique<Flight::Motor>(config));
        }
        log.info("Drone created — mass=" +
                 std::to_string(drone->getMass()) + " kg, " +
                 std::to_string(drone->getMotorCount()) + " motors");

        // ---- Sensors ----
        auto imu            = std::make_shared<Sensors::IMU>(config);
        auto altimeter      = std::make_shared<Sensors::Altimeter>(50.0, 0.05, 0.001);
        auto battery_sensor = std::make_shared<Sensors::BatterySensor>(10.0, 0.02, 0.0);

        // ---- Flight controller ----
        auto fc = std::make_unique<Flight::FlightController>(
            drone, imu, altimeter, battery_sensor, config);
        log.info("Flight controller initialised");

        // ---- Physics engine ----
        Physics::PhysicsEngine physics(config);
        // Inject the drone's computed inertia tensor
        physics.setInertiaTensor(drone->getInertiaTensor());
        log.info("Physics engine initialised (RK4, NED frame)");

        // ---- Renderer ----
        std::unique_ptr<Rendering::Renderer> renderer;
        if (!headless) {
            renderer = std::make_unique<Rendering::Renderer>(1440, 810, "AeroCore v2.0");
            log.info("Renderer created (1440×810)");
        }

        // ---- Telemetry ----
        Simulation::TelemetryData    telemetry{};
        Simulation::TelemetryManager telem_mgr;

        // ---- Wind (initial from config) ----
        Math::Vector3d wind = Math::Vector3d::Zero();
        try {
            wind.x() = config.get<double>("simulation", "wind_x");
            wind.y() = config.get<double>("simulation", "wind_y");
            wind.z() = 0.0;
        } catch (...) {}

        // ---- Drag params ----
        double drag_coeff  = 0.47;
        double drone_area  = drone->getSize() * drone->getSize() * 0.25; // m²
        try {
            drag_coeff = config.get<double>("physics", "drag_coefficient");
        } catch (...) {}

        // ---- Timing ----
        double physics_dt = 0.004;   // 250 Hz physics (stable for quadrotor)
        try {
            physics_dt = config.get<double>("simulation", "dt");
        } catch (...) {}

        double sim_time    = 0.0;
        double accumulator = 0.0;
        sf::Clock clock;
        bool running = true;
        double status_elapsed = 0.0;
        const double status_period = 1.0 / headless_status_rate_hz;

        // Cap frame delta to prevent spiral-of-death when window is moved
        const double MAX_FRAME_DT = 0.1;

        log.info("Entering main loop — physics dt=" + std::to_string(physics_dt) + "s");

        while (running) {
            // ---- Frame timing ----
            // On the very first frame clock gives ~0. Use physics_dt as minimum
            // so we always advance at least one physics step.
            const double raw_dt = static_cast<double>(clock.restart().asSeconds());
            const double frame_dt = std::max(physics_dt,
                                    std::min(raw_dt, MAX_FRAME_DT));

            // ---- Input handling ----
            if (!headless && renderer) {
                auto input = renderer->pollEvents();
                if (!renderer->isOpen() || input.quit) { running = false; break; }

                if (input.arm_toggle) {
                    if (fc->getMode() == Flight::FlightMode::DISARMED) fc->arm();
                    else fc->disarm();
                }
                if (input.takeoff)  fc->takeoff();
                if (input.land)     fc->land();
                if (input.reset) {
                    drone->reset();
                    fc->reset();
                    physics.setInertiaTensor(drone->getInertiaTensor());
                    sim_time    = 0.0;
                    accumulator = 0.0;
                    wind     = Math::Vector3d::Zero();
                    log.info("Simulation reset by user");
                }

                const double wind_step = 2.0 * frame_dt;
                if (input.wind_north) wind.x() += wind_step;
                if (input.wind_south) wind.x() -= wind_step;
                if (input.wind_east)  wind.y() += wind_step;
                if (input.wind_west)  wind.y() -= wind_step;

                const double alt_step = 5.0 * frame_dt;
                if (input.alt_increase) fc->setTargetAltitude(fc->getTargetAltitude() + alt_step);
                if (input.alt_decrease) fc->setTargetAltitude(fc->getTargetAltitude() - alt_step);

                if (input.mode_stabilize) fc->requestMode(Flight::FlightMode::STABILIZE);
                if (input.mode_alt_hold)  fc->requestMode(Flight::FlightMode::ALTITUDE_HOLD);
                if (input.mode_pos_hold)  fc->requestMode(Flight::FlightMode::POSITION_HOLD);
                if (input.mode_rth)       fc->requestMode(Flight::FlightMode::RETURN_HOME);
            } else {
                // Headless auto-flight sequence
                if (fc->getMode() == Flight::FlightMode::DISARMED) fc->arm();
                if (fc->getMode() == Flight::FlightMode::ARMED)    fc->takeoff();
                if (sim_time > max_sim_time) { running = false; break; }
            }

            // ---- Fixed-timestep physics loop ----
            accumulator += frame_dt;

            while (accumulator >= physics_dt) {
                auto& state = drone->getState();

                const double altitude = std::max(0.0, -state.position.z());
                const double rho = physics.airDensityAtAltitude(altitude);

                // 1. Update sensors from current state (so flight controller has fresh data)
                const Math::Vector3d accel_world = state.acceleration;
                const Math::Vector3d accel_body  = Math::worldToBody(state.orientation, accel_world);
                imu->update(physics_dt, accel_body, state.angular_vel, state.orientation);
                altimeter->update(physics_dt, drone->getAltitude());
                battery_sensor->update(physics_dt, drone->getBatteryVoltage());

                // 2. Flight controller reads sensors and writes motor commands
                fc->update(physics_dt);

                // 3. Update motor dynamics (lag toward commanded throttle)
                drone->update(physics_dt, rho);

                // 4. Build external forces from current motor state
                const Physics::ExternalForces forces =
                    drone->buildExternalForces(state.orientation, wind, rho);

                // Debug: print first few ticks
                if (headless_debug && sim_time < 3.0 && static_cast<int>(sim_time / physics_dt) % 125 == 0) {
                    double T = 0; for (size_t i=0;i<drone->getMotorCount();++i) T+=drone->getMotor(i).getCurrentThrust();
                    auto euler = drone->getEulerAngles();
                    auto omega = drone->getAngularVelocity();
                    std::ostringstream motor_ss;
                    for (size_t i = 0; i < drone->getMotorCount(); ++i) {
                        if (i != 0) motor_ss << "/";
                        motor_ss << drone->getMotor(i).getThrottle();
                    }
                    std::cout << "[DBG] t=" << std::fixed << std::setprecision(2) << sim_time
                              << " z=" << state.position.z()
                              << " roll=" << euler.x()*57.3
                              << " p=" << omega.x()
                              << " Fz=" << forces.thrust_world.z()
                              << " M:" << motor_ss.str()
                              << "\n";
                }

                // 5. RK4 physics step — integrates state forward
                physics.step(state, forces,
                             drone->getMass(), drag_coeff, drone_area,
                             physics_dt);

                sim_time += physics_dt;
                accumulator -= physics_dt;
            }

            // ---- Gather telemetry ----
            {
                const auto& state = drone->getState();
                const auto  euler = drone->getEulerAngles();
                const auto  diag  = fc->getDiagnostics();
                const double alt  = drone->getAltitude();
                const double rho  = physics.airDensityAtAltitude(alt);

                telemetry.simulation_time = sim_time;
                telemetry.position        = state.position;
                telemetry.velocity        = state.velocity;
                telemetry.acceleration    = state.acceleration;
                telemetry.altitude        = alt;
                telemetry.euler_angles    = euler;
                telemetry.angular_velocity = state.angular_vel;
                telemetry.target_altitude = fc->getTargetAltitude();
                telemetry.flight_mode     = fc->getMode();
                telemetry.air_density     = rho;
                telemetry.wind_world      = wind;

                // Thrust
                double total_T = 0.0;
                for (size_t i = 0; i < drone->getMotorCount(); ++i) {
                    const auto& m = drone->getMotor(i);
                    total_T += m.getCurrentThrust();
                    if (i < 8) {
                        telemetry.motor_throttle[i] = m.getThrottle();
                        telemetry.motor_rpm[i]      = m.getRPM();
                    }
                }
                telemetry.current_thrust = total_T;

                // PID diagnostics
                telemetry.pid_alt_error      = diag.alt_error;
                telemetry.pid_alt_integral   = diag.alt_integral;
                telemetry.pid_alt_derivative = diag.alt_derivative;
                telemetry.pid_alt_output     = diag.alt_output;
                telemetry.pid_roll_output    = diag.roll_rate_error;
                telemetry.pid_pitch_output   = diag.pitch_rate_error;
                telemetry.pid_yaw_output     = diag.yaw_rate_error;

                // Sensor readings
                telemetry.accel_sensor  = imu->getAccelerometer().getAcceleration();
                telemetry.gyro_sensor   = imu->getGyroscope().getAngularVelocity();
                telemetry.alt_sensor    = altimeter->getAltitude();
                telemetry.battery_voltage = drone->getBatteryVoltage();
                telemetry.battery_soc   = drone->getBatteryPercentage();
                telemetry.total_current = drone->getTotalCurrentDraw();
            }

            telem_mgr.update(frame_dt, telemetry);

            // ---- Render / output ----
            if (!headless && renderer) {
                renderer->clear();
                renderer->render(*drone, telem_mgr.getCurrentData(), wind);
                renderer->display();
                running = renderer->isOpen();
            } else {
                status_elapsed += frame_dt;
                if (status_elapsed >= status_period) {
                    std::cout << telem_mgr.getStatusLine() << "\n";
                    status_elapsed = 0.0;
                }
            }
        }

        log.info("AeroCore simulation ended — sim_time=" +
                 std::to_string(sim_time) + "s");

    } catch (const std::exception& e) {
        Utilities::Logger::getInstance().error(std::string("Fatal: ") + e.what());
        std::cerr << "[AeroCore] Fatal error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
