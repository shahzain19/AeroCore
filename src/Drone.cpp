/**
 * @file Drone.cpp
 * @brief Multirotor vehicle model implementation.
 *
 * ## X-Frame Motor Layout (top view, NED body axes)
 *
 * ```
 *      North (+X)
 *        ↑
 *  M1 CCW  M0 CW          Body +Y = East (right)
 *    \      /             Body +Z = Down
 *     +----+
 *     |    |
 *     +----+
 *    /      \
 *  M3 CW  M2 CCW
 * ```
 *
 * Motor positions in body frame (arm_length from centre):
 *   M0: (+L/√2, +L/√2, 0)   front-right, CW  (+1)
 *   M1: (+L/√2, -L/√2, 0)   front-left,  CCW (-1)
 *   M2: (-L/√2, -L/√2, 0)   rear-left,   CW  (+1)
 *   M3: (-L/√2, +L/√2, 0)   rear-right,  CCW (-1)
 *
 * (L = arm_length)
 *
 * ## Mixing Matrix (X-frame)
 *   T0 = throttle + roll + pitch − yaw
 *   T1 = throttle − roll + pitch + yaw
 *   T2 = throttle − roll − pitch − yaw
 *   T3 = throttle + roll − pitch + yaw
 *
 * where roll, pitch, yaw are normalised torque commands ∈ [−1, +1].
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Flight/Drone.h"
#include "Math/Vector.h"
#include <stdexcept>
#include <cmath>

namespace AeroCore {
namespace Flight {

// ============================================================
//  Construction
// ============================================================

Drone::Drone(const Utilities::Config& config) {
    // Helper: safe config read with fallback
    auto tryGet = [&](const std::string& sect, const std::string& key, double fb) -> double {
        try { return config.get<double>(sect, key); }
        catch (...) { return fb; }
    };

    mass_       = tryGet("drone", "mass",        1.5);
    arm_length_ = tryGet("drone", "arm_length",  0.225);
    size_       = tryGet("drone", "size",         0.45);

    // Battery parameters
    battery_voltage_max_          = tryGet("battery", "voltage_max",           16.8); // 4S LiPo
    battery_capacity_ah_          = tryGet("battery", "capacity_ah",            1.5);
    battery_internal_resistance_  = tryGet("battery", "internal_resistance",    0.02);
    battery_charge_ah_            = battery_capacity_ah_;  // Full charge at start

    std::string airframe = "multirotor";
    try {
        airframe = config.get<std::string>("drone", "airframe");
    } catch (...) {}

    if (airframe == "fixed_wing" || airframe == "plane") {
        airframe_type_ = AirframeType::FIXED_WING;
    } else {
        airframe_type_ = AirframeType::MULTIROTOR;
    }

    if (airframe_type_ == AirframeType::FIXED_WING) {
        recommended_motor_count_ = static_cast<size_t>(
            std::max(1.0, tryGet("drone", "motor_count", 1.0)));
    } else {
        recommended_motor_count_ = static_cast<size_t>(
            std::max(1.0, tryGet("drone", "motor_count", 4.0)));
    }

    buildMotorGeometry();
}

// ============================================================
//  Motor geometry
// ============================================================

void Drone::buildMotorGeometry() {
    motor_positions_.clear();
    motor_spin_directions_.clear();

    if (recommended_motor_count_ == 0) {
        recommended_motor_count_ = 1;
    }

    if (airframe_type_ == AirframeType::FIXED_WING) {
        // Propulsion-only model: longitudinal thrust, no distributed rotor moments.
        motor_positions_.push_back(Math::Vector3d(0.0, 0.0, 0.0));
        motor_spin_directions_.push_back(1);
        return;
    }

    if (recommended_motor_count_ == 4) {
        const double L = arm_length_ / std::sqrt(2.0);
        motor_positions_ = {
            { L,  L, 0.0},
            { L, -L, 0.0},
            {-L, -L, 0.0},
            {-L,  L, 0.0},
        };
        motor_spin_directions_ = {+1, -1, +1, -1};
        return;
    }

    const double radius = std::max(0.05, arm_length_);
    motor_positions_.reserve(recommended_motor_count_);
    motor_spin_directions_.reserve(recommended_motor_count_);

    for (size_t i = 0; i < recommended_motor_count_; ++i) {
        const double a = 2.0 * Math::PI * static_cast<double>(i) /
                         static_cast<double>(recommended_motor_count_);
        motor_positions_.emplace_back(radius * std::cos(a), radius * std::sin(a), 0.0);
        motor_spin_directions_.push_back((i % 2 == 0) ? 1 : -1);
    }
}

// ============================================================
//  Motor management
// ============================================================

void Drone::addMotor(std::unique_ptr<Motor> motor) {
    motors_.push_back(std::move(motor));
}

void Drone::setMotorThrottle(size_t index, double throttle) {
    if (index < motors_.size()) {
        motors_[index]->setThrottle(throttle);
    }
}

void Drone::setMotorThrust(size_t index, double thrust) {
    if (index < motors_.size()) {
        motors_[index]->setTargetThrust(thrust);
    }
}

void Drone::applyControl(const ControlInput& input) {
    const double thr = Math::clamp(input.throttle, 0.0, 1.0);
    const double rol = Math::clamp(input.roll,    -1.0, 1.0);
    const double pit = Math::clamp(input.pitch,   -1.0, 1.0);
    const double yaw = Math::clamp(input.yaw,     -1.0, 1.0);

    if (airframe_type_ == AirframeType::FIXED_WING) {
        for (size_t i = 0; i < motors_.size(); ++i) {
            motors_[i]->setThrottle(i == 0 ? thr : 0.0);
        }
        return;
    }

    const size_t count = std::min(motors_.size(), motor_positions_.size());
    if (count == 0) return;

    double max_radius = 0.0;
    for (size_t i = 0; i < count; ++i) {
        max_radius = std::max(max_radius, std::hypot(motor_positions_[i].x(), motor_positions_[i].y()));
    }
    max_radius = std::max(max_radius, 1e-6);

    const double roll_gain = 0.45;
    const double pitch_gain = 0.45;
    const double yaw_gain = 0.25;

    std::vector<double> mix(count, thr);
    for (size_t i = 0; i < count; ++i) {
        const auto& r = motor_positions_[i];
        const double roll_coeff = -r.y() / max_radius;
        const double pitch_coeff = r.x() / max_radius;
        const double spin = (i < motor_spin_directions_.size() && motor_spin_directions_[i] >= 0) ? 1.0 : -1.0;
        mix[i] += roll_gain * roll_coeff * rol +
                  pitch_gain * pitch_coeff * pit +
                  yaw_gain * spin * yaw;
    }

    double min_cmd = mix[0], max_cmd = mix[0];
    for (double c : mix) {
        min_cmd = std::min(min_cmd, c);
        max_cmd = std::max(max_cmd, c);
    }

    if (min_cmd < 0.0 || max_cmd > 1.0) {
        const double center = 0.5 * (min_cmd + max_cmd);
        const double span = std::max(max_cmd - min_cmd, 1e-9);
        const double scale = std::min(1.0, 1.0 / span);
        for (double& c : mix) {
            c = (c - center) * scale + thr;
        }
    }

    for (size_t i = 0; i < count; ++i) {
        motors_[i]->setThrottle(Math::clamp(mix[i], 0.0, 1.0));
    }
}

// ============================================================
//  Physics interface
// ============================================================

Physics::ExternalForces Drone::buildExternalForces(
        const Math::Quaterniond& orientation,
        const Math::Vector3d& wind_world,
        double air_density) const {
    (void)air_density;

    Physics::ExternalForces forces;
    forces.wind_world = wind_world;

    Math::Vector3d torque_body = Math::Vector3d::Zero();
    Math::Vector3d thrust_body = Math::Vector3d::Zero();

    const size_t count = std::min(motors_.size(), motor_positions_.size());
    for (size_t i = 0; i < count; ++i) {
        const Motor& m = *motors_[i];
        const double T = m.getCurrentThrust();
        const double Q = m.getReactionTorque();

        // Thrust direction: −Z in body frame (upward in NED)
        const Math::Vector3d thrust_dir_body(0.0, 0.0, -1.0);
        thrust_body += thrust_dir_body * T;

        // Torque from motor position (moment arm cross product)
        // τ = r × F  where F = T * thrust_dir_body
        const Math::Vector3d& r = motor_positions_[i];
        torque_body += r.cross(thrust_dir_body * T);

        // Reaction torque from propeller drag (about body Z)
        // CCW motor (+1) produces +Z reaction torque (yaw left in NED)
        const double spin_dir = (i < motor_spin_directions_.size())
                                ? motor_spin_directions_[i] : 1;
        torque_body.z() += spin_dir * Q;
    }

    // Rotate net thrust from body frame to world (NED) frame
    forces.thrust_world = orientation * thrust_body;
    forces.torque_body  = torque_body;

    return forces;
}

// ============================================================
//  Update
// ============================================================

void Drone::update(double dt, double air_density) {
    // Update battery voltage for motor current estimation
    const double V = getBatteryVoltage();
    for (auto& m : motors_) {
        m->setVoltage(V);
        m->update(dt, air_density);
    }
    updateBattery(dt);
}

void Drone::reset() {
    state_ = Physics::RigidBodyState();
    battery_charge_ah_ = battery_capacity_ah_;
    for (auto& m : motors_) m->reset();
}

// ============================================================
//  State accessors
// ============================================================

const Physics::RigidBodyState& Drone::getState()    const { return state_; }
Physics::RigidBodyState&       Drone::getState()          { return state_; }

Math::Vector3d    Drone::getPosition()        const { return state_.position; }
Math::Vector3d    Drone::getVelocity()        const { return state_.velocity; }
Math::Vector3d    Drone::getAcceleration()    const { return state_.acceleration; }
Math::Quaterniond Drone::getOrientation()     const { return state_.orientation; }
Math::Vector3d    Drone::getAngularVelocity() const { return state_.angular_vel; }
double            Drone::getAltitude()        const { return -state_.position.z(); }

Math::Vector3d Drone::getEulerAngles() const {
    return Math::quaternionToEuler(state_.orientation);
}

void Drone::setState(const Physics::RigidBodyState& state) { state_ = state; }

// ============================================================
//  Vehicle parameters
// ============================================================

double Drone::getMass()           const { return mass_; }
double Drone::getArmLength()      const { return arm_length_; }
double Drone::getSize()           const { return size_; }
size_t Drone::getMotorCount()     const { return motors_.size(); }
AirframeType Drone::getAirframeType() const { return airframe_type_; }
size_t Drone::getRecommendedMotorCount() const { return recommended_motor_count_; }

const Motor& Drone::getMotor(size_t idx) const {
    if (idx >= motors_.size()) throw std::out_of_range("Motor index out of range");
    return *motors_[idx];
}

double Drone::getMaxTotalThrust() const {
    double total = 0.0;
    for (const auto& m : motors_) total += m->getMaxThrust();
    return total;
}

Math::Vector3d Drone::getThrustBody() const {
    double total_T = 0.0;
    for (const auto& m : motors_) total_T += m->getCurrentThrust();
    return Math::Vector3d(0.0, 0.0, -total_T);  // −Z = up in body NED
}

Math::Vector3d Drone::getThrustWorld(const Math::Quaterniond& orientation) const {
    return orientation * getThrustBody();
}

// ============================================================
//  Battery model
// ============================================================

double Drone::getBatteryVoltage() const {
    // Linear OCV curve for LiPo: V_OCV = V_min + (V_max − V_min) · SOC
    const double soc = battery_charge_ah_ / battery_capacity_ah_;
    const double V_min = battery_voltage_max_ * 0.75;  // ~3.5 V/cell for 4S = 14 V
    const double V_ocv = V_min + (battery_voltage_max_ - V_min) * soc;

    // Terminal voltage: V_t = V_ocv − I · R_int
    const double I = getTotalCurrentDraw();
    return std::max(V_min * 0.9, V_ocv - I * battery_internal_resistance_);
}

double Drone::getBatteryPercentage() const {
    return 100.0 * (battery_charge_ah_ / battery_capacity_ah_);
}

double Drone::getTotalCurrentDraw() const {
    double total = 0.0;
    for (const auto& m : motors_) total += m->getCurrentDraw();
    return total;
}

void Drone::updateBattery(double dt) {
    // Discharge: ΔQ = I · dt (Ah)
    const double I = getTotalCurrentDraw();
    battery_charge_ah_ -= I * dt / 3600.0;  // dt is in seconds
    battery_charge_ah_ = std::max(0.0, battery_charge_ah_);
}

// ============================================================
//  Inertia tensor
// ============================================================

Math::Matrix3d Drone::getInertiaTensor() const {
    // Model the drone as a central rigid body (cylinder) + 4 point masses at motors.
    //
    // Central body:
    //   Cylinder: Ixx = Iyy = m_body · (3·r² + h²) / 12
    //             Izz = m_body · r² / 2
    // We approximate the body as a flat disc:
    const double m_body   = mass_ * 0.4;    // 40% of mass in central body
    const double r_body   = size_ * 0.15;   // body radius ≈ 15% of frame size
    const double Ixy_body = 0.5 * m_body * r_body * r_body;  // disc
    const double Iz_body  = Ixy_body;

    // Point masses at each motor position
    double Ixx = Ixy_body;
    double Iyy = Ixy_body;
    double Izz = Iz_body;

    const double m_motor = mass_ * 0.6 / std::max((size_t)1, motors_.size());
    for (const auto& pos : motor_positions_) {
        // Parallel axis theorem: I += m · (r² − component²)
        const double r2 = pos.squaredNorm();
        Ixx += m_motor * (r2 - pos.x() * pos.x());
        Iyy += m_motor * (r2 - pos.y() * pos.y());
        Izz += m_motor * (r2 - pos.z() * pos.z());
    }

    Math::Matrix3d J = Math::Matrix3d::Zero();
    J(0,0) = std::max(1e-6, Ixx);
    J(1,1) = std::max(1e-6, Iyy);
    J(2,2) = std::max(1e-6, Izz);
    return J;
}

} // namespace Flight
} // namespace AeroCore
