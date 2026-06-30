/**
 * @file FlightController.cpp
 * @brief Cascaded PID flight controller implementation.
 *
 * ## Control Loop Structure
 *
 * The update() method dispatches to a mode-specific handler.
 * Each handler builds a throttle + attitude setpoint, then calls
 * runAttitudeControl() which runs the inner rate loops and writes
 * per-motor commands through mixMotors().
 *
 * ### Altitude channel (ALTITUDE_HOLD / TAKEOFF / LANDING)
 *
 *   alt_error   = target_alt − measured_alt
 *   throttle_delta = pid_alt_.update(dt, target_alt, measured_alt)
 *   throttle_cmd   = hover_throttle_ + throttle_delta
 *
 * `hover_throttle_` is pre-computed from mass and motor max thrust so the
 * altitude PID only needs to correct deviations, not overcome gravity.
 *
 * ### Attitude cascade (ATTITUDE_HOLD)
 *
 *   roll_rate_sp  = pid_roll_.update(dt, target_roll,  measured_roll)
 *   pitch_rate_sp = pid_pitch_.update(dt, target_pitch, measured_pitch)
 *   yaw_rate_sp   = pid_yaw_.update(dt, target_yaw,   measured_yaw)
 *   (fed into the rate PIDs)
 *
 * ### Rate PIDs
 *
 *   roll_torque  = pid_roll_rate_.update(dt, roll_rate_sp,  measured_p)
 *   pitch_torque = pid_pitch_rate_.update(dt, pitch_rate_sp, measured_q)
 *   yaw_torque   = pid_yaw_rate_.update(dt, yaw_rate_sp,   measured_r)
 *   (fed to mixMotors())
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Flight/FlightController.h"
#include "Math/Vector.h"
#include <algorithm>
#include <cmath>

namespace AeroCore {
namespace Flight {

using Math::clamp;

// ============================================================
//  Construction
// ============================================================

FlightController::FlightController(
        std::shared_ptr<Drone>                 drone,
        std::shared_ptr<Sensors::IMU>          imu,
        std::shared_ptr<Sensors::Altimeter>    altimeter,
        std::shared_ptr<Sensors::BatterySensor> battery_sensor,
        const Utilities::Config& config)
    : drone_(std::move(drone))
    , imu_(std::move(imu))
    , altimeter_(std::move(altimeter))
    , battery_sensor_(std::move(battery_sensor))
    , mode_(FlightMode::DISARMED)
    , target_altitude_(0.0)
    , target_position_(Math::Vector3d::Zero())
    , target_heading_(0.0)
    , takeoff_alt_ramp_(0.0)
    , final_target_alt_(0.0)
    , home_position_(Math::Vector3d::Zero())
{
    // Safe config read with fallback
    auto tryGet = [&](const std::string& sect, const std::string& key, double fb) -> double {
        try { return config.get<double>(sect, key); }
        catch (...) { return fb; }
    };

    // ---- Altitude PID ----
    // Output is throttle delta around hover_throttle. Gains/limits come from config.
    pid_alt_ = std::make_unique<PIDController>(config, "pid_altitude");
    pid_alt_->setAntiWindupMode(AntiWindupMode::BACK_CALC);
    pid_alt_->setBackCalcGain(tryGet("pid_altitude", "back_calc_gain", 0.3));

    // ---- Roll angle PID ----
    pid_roll_ = std::make_unique<PIDController>(
        tryGet("pid_roll", "kp", 4.0),
        tryGet("pid_roll", "ki", 0.0),
        tryGet("pid_roll", "kd", 0.1));
    pid_roll_->setOutputLimits(-5.0, 5.0);  // [rad/s] rate command

    // ---- Pitch angle PID ----
    pid_pitch_ = std::make_unique<PIDController>(
        tryGet("pid_pitch", "kp", 4.0),
        tryGet("pid_pitch", "ki", 0.0),
        tryGet("pid_pitch", "kd", 0.1));
    pid_pitch_->setOutputLimits(-5.0, 5.0);

    // ---- Yaw angle PID ----
    pid_yaw_ = std::make_unique<PIDController>(
        tryGet("pid_yaw", "kp", 2.0),
        tryGet("pid_yaw", "ki", 0.0),
        tryGet("pid_yaw", "kd", 0.0));
    pid_yaw_->setOutputLimits(-3.0, 3.0);

    // ---- Roll rate PID ----
    // Output range: ±0.3 (fraction of mixer authority)
    // At scale=0.5 in mixer: max differential throttle = 0.15 → max roll torque ≈ 0.24 N·m
    pid_roll_rate_ = std::make_unique<PIDController>(
        tryGet("pid_roll_rate", "kp", 0.08),
        tryGet("pid_roll_rate", "ki", 0.01),
        tryGet("pid_roll_rate", "kd", 0.002));
    pid_roll_rate_->setOutputLimits(-0.4, 0.4);
    pid_roll_rate_->setIntegralMax(0.15);
    pid_roll_rate_->setDerivativeFilter(0.15);

    // ---- Pitch rate PID ----
    pid_pitch_rate_ = std::make_unique<PIDController>(
        tryGet("pid_pitch_rate", "kp", 0.08),
        tryGet("pid_pitch_rate", "ki", 0.01),
        tryGet("pid_pitch_rate", "kd", 0.002));
    pid_pitch_rate_->setOutputLimits(-0.4, 0.4);
    pid_pitch_rate_->setIntegralMax(0.15);
    pid_pitch_rate_->setDerivativeFilter(0.15);

    // ---- Yaw rate PID ----
    pid_yaw_rate_ = std::make_unique<PIDController>(
        tryGet("pid_yaw_rate", "kp", 0.10),
        tryGet("pid_yaw_rate", "ki", 0.01),
        tryGet("pid_yaw_rate", "kd", 0.0));
    pid_yaw_rate_->setOutputLimits(-0.3, 0.3);
    pid_yaw_rate_->setIntegralMax(0.1);

    // ---- Limits ----
    max_roll_angle_  = tryGet("flight", "max_roll_angle_deg",  35.0) * Math::DEG2RAD;
    max_pitch_angle_ = tryGet("flight", "max_pitch_angle_deg", 35.0) * Math::DEG2RAD;
    max_roll_rate_   = tryGet("flight", "max_roll_rate_deg_s", 220.0) * Math::DEG2RAD;
    max_pitch_rate_  = tryGet("flight", "max_pitch_rate_deg_s", 220.0) * Math::DEG2RAD;
    max_yaw_rate_    = tryGet("flight", "max_yaw_rate_deg_s", 180.0) * Math::DEG2RAD;
    max_tilt_angle_  = tryGet("flight", "max_tilt_angle_deg",  45.0) * Math::DEG2RAD;

    // Hover throttle: the throttle fraction needed to sustain 1g in hover.
    // T_hover = m·g.  At full throttle we get max_total_thrust.
    // NOTE: motors are added after construction so we defer this to first update.
    hover_throttle_ = 0.5;  // Will be recomputed in first update()

    // Read initial target altitude
    try {
        target_altitude_ = config.get<double>("simulation", "target_altitude");
    } catch (...) {
        target_altitude_ = 10.0;
    }
}

// ============================================================
//  Mode transitions
// ============================================================

void FlightController::requestMode(FlightMode mode) {
    // Validate transition
    if (mode == FlightMode::TAKEOFF && mode_ != FlightMode::ARMED) {
        Utilities::Logger::getInstance().warning("Rejected TAKEOFF: not ARMED");
        return;
    }
    if (mode != FlightMode::DISARMED && mode != FlightMode::ARMED &&
        mode_ == FlightMode::DISARMED) {
        Utilities::Logger::getInstance().warning("Rejected: vehicle is DISARMED");
        return;
    }
    if (mode == FlightMode::POSITION_HOLD) {
        target_position_ = drone_->getPosition();
    }
    if (mode == FlightMode::ALTITUDE_HOLD && mode_ == FlightMode::TAKEOFF) {
        transitionToAltitudeHold();
        return;
    }
    if (mode == FlightMode::ALTITUDE_HOLD) {
        pid_alt_->reset();
    }
    logModeTransition(mode_, mode);
    mode_ = mode;
}

void FlightController::arm() {
    if (mode_ == FlightMode::DISARMED) {
        home_position_   = drone_->getPosition();
        target_position_ = home_position_;
        // Reset all PID state to prevent integrator wind-up from a previous flight
        pid_alt_->reset();
        pid_roll_->reset();
        pid_pitch_->reset();
        pid_yaw_->reset();
        pid_roll_rate_->reset();
        pid_pitch_rate_->reset();
        pid_yaw_rate_->reset();
        takeoff_alt_ramp_ = 0.0;
        logModeTransition(mode_, FlightMode::ARMED);
        mode_ = FlightMode::ARMED;
    }
}

void FlightController::disarm() {
    logModeTransition(mode_, FlightMode::DISARMED);
    mode_ = FlightMode::DISARMED;
    for (size_t i = 0; i < drone_->getMotorCount(); ++i) {
        drone_->setMotorThrottle(i, 0.0);
    }
}

void FlightController::takeoff() {
    if (mode_ == FlightMode::ARMED) {
        final_target_alt_ = target_altitude_;
        takeoff_alt_ramp_ = 0.0;
        logModeTransition(mode_, FlightMode::TAKEOFF);
        mode_ = FlightMode::TAKEOFF;
    }
}

void FlightController::land() {
    if (isFlightMode(mode_)) {
        logModeTransition(mode_, FlightMode::LANDING);
        mode_ = FlightMode::LANDING;
    }
}

void FlightController::reset() {
    pid_alt_->reset();
    pid_roll_->reset();
    pid_pitch_->reset();
    pid_yaw_->reset();
    pid_roll_rate_->reset();
    pid_pitch_rate_->reset();
    pid_yaw_rate_->reset();
    mode_ = FlightMode::DISARMED;
    Utilities::Logger::getInstance().info("FlightController reset");
}

// ============================================================
//  Setpoints
// ============================================================

void FlightController::setTargetAltitude(double alt) {
    target_altitude_ = std::max(0.0, alt);
}

void FlightController::setTargetPosition(const Math::Vector3d& pos) {
    target_position_ = pos;
}

void FlightController::setTargetHeading(double yaw) {
    target_heading_ = Math::wrapAngle(yaw);
}

void FlightController::setPilotInput(const PilotInput& input) {
    pilot_input_ = input;
}

// ============================================================
//  Getters
// ============================================================

FlightMode FlightController::getMode()           const { return mode_; }
double     FlightController::getTargetAltitude() const { return target_altitude_; }
const PIDController& FlightController::getAltitudePID()   const { return *pid_alt_; }
const PIDController& FlightController::getRollRatePID()   const { return *pid_roll_rate_; }
const PIDController& FlightController::getPitchRatePID()  const { return *pid_pitch_rate_; }
const PIDController& FlightController::getYawRatePID()    const { return *pid_yaw_rate_; }

ControllerDiagnostics FlightController::getDiagnostics() const {
    ControllerDiagnostics d{};
    d.mode              = mode_;
    d.alt_error         = pid_alt_->getError();
    d.alt_integral      = pid_alt_->getIntegral();
    d.alt_derivative    = pid_alt_->getDerivative();
    d.alt_output        = pid_alt_->getOutput();
    d.roll_error        = pid_roll_->getError();
    d.pitch_error       = pid_pitch_->getError();
    d.yaw_error         = pid_yaw_->getError();
    d.roll_rate_error   = pid_roll_rate_->getError();
    d.pitch_rate_error  = pid_pitch_rate_->getError();
    d.yaw_rate_error    = pid_yaw_rate_->getError();
    for (size_t i = 0; i < std::min(drone_->getMotorCount(), (size_t)4); ++i) {
        d.motor_cmd[i] = drone_->getMotor(i).getThrottle();
    }
    return d;
}

// ============================================================
//  Main update dispatch
// ============================================================

void FlightController::update(double dt) {
    // Recompute hover throttle each update (accounts for mass changes / motor config)
    const double T_max = drone_->getMaxTotalThrust();
    if (T_max > 1.0) {
        hover_throttle_ = clamp((drone_->getMass() * Math::GRAVITY_MSL) / T_max, 0.05, 0.95);
    }

    if (checkFailsafe()) return;

    switch (mode_) {
        case FlightMode::DISARMED:      updateDisarmed(dt);     break;
        case FlightMode::ARMED:         updateArmed(dt);        break;
        case FlightMode::TAKEOFF:       updateTakeoff(dt);      break;
        case FlightMode::LANDING:       updateLanding(dt);      break;
        case FlightMode::STABILIZE:     updateStabilize(dt);    break;
        case FlightMode::ATTITUDE_HOLD: updateAttitudeHold(dt); break;
        case FlightMode::ALTITUDE_HOLD: updateAltitudeHold(dt); break;
        case FlightMode::POSITION_HOLD: updatePositionHold(dt); break;
        case FlightMode::RETURN_HOME:   updateReturnHome(dt);   break;
        case FlightMode::FAILSAFE:      updateFailsafe(dt);     break;
        default:
            updateDisarmed(dt);
            break;
    }
}

// ============================================================
//  Mode implementations
// ============================================================

void FlightController::updateDisarmed(double /*dt*/) {
    for (size_t i = 0; i < drone_->getMotorCount(); ++i) {
        drone_->setMotorThrottle(i, 0.0);
    }
}

void FlightController::updateArmed(double /*dt*/) {
    // Motors at minimum idle (e.g. 5%) so ESCs stay calibrated
    for (size_t i = 0; i < drone_->getMotorCount(); ++i) {
        drone_->setMotorThrottle(i, 0.05);
    }
}

void FlightController::updateTakeoff(double dt) {
    const double ramp_rate = 2.0;
    takeoff_alt_ramp_ += ramp_rate * dt;
    if (takeoff_alt_ramp_ > final_target_alt_)
        takeoff_alt_ramp_ = final_target_alt_;

    const double measured_alt = altimeter_->getAltitude();

    if (measured_alt >= final_target_alt_ - 1.0 &&
        takeoff_alt_ramp_ >= final_target_alt_ - 0.5) {
        transitionToAltitudeHold();
        return;
    }

    const double throttle_cmd =
        computeAltitudeThrottle(dt, takeoff_alt_ramp_, 0.05, false);
    runAttitudeControl(throttle_cmd, 0.0, 0.0, 0.0, dt);
}

void FlightController::updateLanding(double dt) {
    const double measured_alt = altimeter_->getAltitude();

    target_altitude_ = std::max(0.0, target_altitude_ - 1.0 * dt);

    const double vz = drone_->getVelocity().z();
    if (measured_alt < 0.1 && vz > -0.1) {
        disarm();
        return;
    }

    const double throttle_cmd =
        computeAltitudeThrottle(dt, target_altitude_, 0.0, false);
    runAttitudeControl(throttle_cmd, 0.0, 0.0, 0.0, dt);
}

void FlightController::updateStabilize(double dt) {
    // Rate mode: pilot sticks directly command angular rates
    const double roll_rate_sp  = pilot_input_.roll  * max_roll_rate_;
    const double pitch_rate_sp = pilot_input_.pitch * max_pitch_rate_;
    const double yaw_rate_sp   = pilot_input_.yaw   * max_yaw_rate_;

    const auto& omega = imu_->getGyroscope().getAngularVelocity();
    const double roll_cmd  = pid_roll_rate_->update(dt,  roll_rate_sp,  omega.x());
    const double pitch_cmd = pid_pitch_rate_->update(dt, pitch_rate_sp, omega.y());
    const double yaw_cmd   = pid_yaw_rate_->update(dt,   yaw_rate_sp,   omega.z());

    double throttle_cmd = clamp(pilot_input_.throttle, 0.0, 1.0);
    mixMotors(throttle_cmd, roll_cmd, pitch_cmd, yaw_cmd);
}

void FlightController::updateAttitudeHold(double dt) {
    // Outer: pilot sticks command angle setpoints
    const double roll_sp  = pilot_input_.roll  * max_roll_angle_;
    const double pitch_sp = pilot_input_.pitch * max_pitch_angle_;
    const double yaw_rate_sp = pilot_input_.yaw * max_yaw_rate_;

    runAttitudeControl(clamp(pilot_input_.throttle, 0.0, 1.0),
                       roll_sp, pitch_sp, yaw_rate_sp, dt);
}

void FlightController::updateAltitudeHold(double dt) {
    const double throttle_cmd =
        computeAltitudeThrottle(dt, target_altitude_, 0.05, true);

    const double roll_sp     = pilot_input_.roll  * max_roll_angle_;
    const double pitch_sp    = pilot_input_.pitch * max_pitch_angle_;
    const double yaw_rate_sp = pilot_input_.yaw   * max_yaw_rate_;

    runAttitudeControl(throttle_cmd, roll_sp, pitch_sp, yaw_rate_sp, dt);
}

void FlightController::updatePositionHold(double dt) {
    const auto& pos = drone_->getPosition();
    const Math::Vector3d pos_error = target_position_ - pos;

    // Basic position P → attitude setpoints (sim uses ground-truth position).
    constexpr double kp_pos = 0.35;
    const double roll_sp  = clamp(-pos_error.y() * kp_pos, -max_roll_angle_, max_roll_angle_);
    const double pitch_sp = clamp( pos_error.x() * kp_pos, -max_pitch_angle_, max_pitch_angle_);

    const double throttle_cmd =
        computeAltitudeThrottle(dt, target_altitude_, 0.05, true);

    runAttitudeControl(throttle_cmd, roll_sp, pitch_sp, 0.0, dt);
}

void FlightController::updateReturnHome(double dt) {
    target_position_ = home_position_;
    target_altitude_ = std::max(target_altitude_, -home_position_.z());

    const auto& pos = drone_->getPosition();
    const Math::Vector3d pos_error = home_position_ - pos;
    const double horiz_dist = pos_error.head<2>().norm();

    if (horiz_dist < 2.0) {
        pid_alt_->reset();
        logModeTransition(mode_, FlightMode::ALTITUDE_HOLD);
        mode_ = FlightMode::ALTITUDE_HOLD;
        return;
    }

    constexpr double kp_pos = 0.45;
    const double roll_sp  = clamp(-pos_error.y() * kp_pos, -max_roll_angle_, max_roll_angle_);
    const double pitch_sp = clamp( pos_error.x() * kp_pos, -max_pitch_angle_, max_pitch_angle_);

    const double throttle_cmd =
        computeAltitudeThrottle(dt, target_altitude_, 0.05, true);

    runAttitudeControl(throttle_cmd, roll_sp, pitch_sp, 0.0, dt);
}

void FlightController::updateFailsafe(double dt) {
    target_altitude_ = std::max(0.0, target_altitude_ - 1.0 * dt);
    const double measured_alt = altimeter_->getAltitude();
    const double throttle_cmd =
        computeAltitudeThrottle(dt, target_altitude_, 0.0, true);
    runAttitudeControl(throttle_cmd, 0.0, 0.0, 0.0, dt);

    if (measured_alt < 0.1) disarm();
}

// ============================================================
//  Attitude control (inner loops)
// ============================================================

void FlightController::runAttitudeControl(double throttle_cmd,
                                           double roll_angle_sp,
                                           double pitch_angle_sp,
                                           double yaw_rate_sp,
                                           double dt) {
    // Read IMU
    const auto euler = drone_->getEulerAngles();  // true angles (from physics)
    const auto& omega = imu_->getGyroscope().getAngularVelocity();

    // Outer loop: angle error → rate setpoint
    const double roll_rate_sp  = pid_roll_->update(dt,  roll_angle_sp,  euler.x());
    const double pitch_rate_sp = pid_pitch_->update(dt, pitch_angle_sp, euler.y());

    // For yaw: yaw angle SP is the integrated heading, but here we pass rate
    // directly from the outer call.  Wrap yaw error.
    const double yaw_error = Math::wrapAngle(target_heading_ - euler.z());
    double yaw_rate_from_heading = pid_yaw_->update(dt, 0.0, -yaw_error);
    // If pilot is commanding yaw, override heading hold
    double final_yaw_rate_sp = (std::abs(yaw_rate_sp) > 0.05)
                                ? yaw_rate_sp
                                : yaw_rate_from_heading;

    // Inner loop: rate error → torque command
    const double roll_cmd  = pid_roll_rate_->update(dt,  roll_rate_sp,  omega.x());
    const double pitch_cmd = pid_pitch_rate_->update(dt, pitch_rate_sp, omega.y());
    const double yaw_cmd   = pid_yaw_rate_->update(dt,   final_yaw_rate_sp, omega.z());

    mixMotors(throttle_cmd, roll_cmd, pitch_cmd, yaw_cmd);
}

// ============================================================
//  Motor mixer
// ============================================================

void FlightController::mixMotors(double throttle, double roll,
                                   double pitch,  double yaw) {
    ControlInput cmd;
    cmd.throttle = clamp(throttle, 0.0, 1.0);
    cmd.roll     = clamp(roll,    -1.0, 1.0);
    cmd.pitch    = clamp(pitch,   -1.0, 1.0);
    cmd.yaw      = clamp(yaw,     -1.0, 1.0);
    drone_->applyControl(cmd);
}

// ============================================================
//  Failsafe check
// ============================================================

bool FlightController::checkFailsafe() {
    // Low battery failsafe
    if (battery_sensor_->getVoltage() > 1.0 &&
        drone_->getBatteryPercentage() < 5.0 &&
        mode_ != FlightMode::FAILSAFE &&
        mode_ != FlightMode::DISARMED) {
        Utilities::Logger::getInstance().warning("Low battery — entering FAILSAFE");
        logModeTransition(mode_, FlightMode::FAILSAFE);
        mode_ = FlightMode::FAILSAFE;
        return false;  // Still execute one FAILSAFE update
    }
    return false;
}

// ============================================================
//  Logging
// ============================================================

void FlightController::logModeTransition(FlightMode from, FlightMode to) {
    Utilities::Logger::getInstance().info(
        "Mode: " + flightModeToString(from) + " → " + flightModeToString(to)
    );
}

void FlightController::transitionToAltitudeHold() {
    logModeTransition(mode_, FlightMode::ALTITUDE_HOLD);
    mode_ = FlightMode::ALTITUDE_HOLD;
    target_altitude_ = final_target_alt_;
    pid_alt_->reset();
}

double FlightController::computeAltitudeThrottle(double dt, double setpoint_alt,
                                                  double min_throttle,
                                                  bool hover_floor) {
    const double measured_alt = altimeter_->getAltitude();

    if (measured_alt < 0.05 && pid_alt_->getIntegral() < 0.0) {
        pid_alt_->reset();
    }

    const double throttle_delta = pid_alt_->update(dt, setpoint_alt, measured_alt);
    double throttle_cmd = hover_throttle_ + throttle_delta;

    double floor = min_throttle;
    if (hover_floor) {
        floor = std::max(min_throttle, hover_throttle_ * 0.70);
    }
    return clamp(throttle_cmd, floor, 0.98);
}

} // namespace Flight
} // namespace AeroCore
