/**
 * @file Motor.cpp
 * @brief Brushless motor + propeller model implementation.
 *
 * ## Motor Response Model
 *
 * The motor speed ω responds to the throttle command through a first-order lag:
 *
 *   ω̇ = (ω_target − ω) / τ
 *
 * Discretised as: ω[k+1] = ω[k] + dt / τ · (ω_target − ω[k])
 *                         = ω[k] · (1 − dt/τ) + ω_target · (dt/τ)
 *
 * This is a first-order IIR filter with coefficient α = exp(−dt/τ).
 * For small dt/τ we use the linear approximation for speed.
 *
 * ## Thrust Model
 *
 * Standard propeller thrust equation:
 *   T = Ct · ρ · n² · D⁴
 *
 * Simplified to: T = k_t · ρ · ω²
 * where k_t = Ct · D⁴ / (4π²) absorbs the conversion from ω [rad/s] to n [rev/s].
 *
 * ## Torque Model
 *
 *   Q = k_q · ρ · ω²
 * where k_q = Cq · D⁵ / (4π²).
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Flight/Motor.h"
#include "Math/Vector.h"
#include <cmath>
#include <stdexcept>

namespace AeroCore {
namespace Flight {

// ============================================================
//  Construction
// ============================================================

Motor::Motor(const Utilities::Config& config)
    : voltage_(12.6)  // Default: 3S LiPo nominal
{
    // Safe config reader with fallback
    auto tryGet = [&](const std::string& sect, const std::string& key, double fb) -> double {
        try { return config.get<double>(sect, key); }
        catch (...) { return fb; }
    };

    max_thrust_     = tryGet("motor",  "max_thrust",             8.0);
    time_constant_  = tryGet("motor",  "response_time_constant",  0.05);
    efficiency_     = tryGet("motor",  "efficiency",               0.85);
    prop_diameter_  = tryGet("motor",  "propeller_diameter",       0.127);   // 5 inch
    thrust_coeff_   = tryGet("motor",  "thrust_coefficient",       0.1);
    torque_coeff_   = tryGet("motor",  "torque_coefficient",       0.01);

    const double max_rpm = tryGet("motor", "max_rpm", 10000.0);
    max_omega_ = max_rpm * (2.0 * Math::PI / 60.0);  // RPM → rad/s

    omega_         = 0.0;
    omega_target_  = 0.0;
    throttle_      = 0.0;
    current_thrust_ = 0.0;
    reaction_torque_ = 0.0;
}

Motor::Motor(double max_thrust, double time_constant, double max_rpm, double efficiency)
    : max_thrust_(max_thrust)
    , time_constant_(time_constant)
    , max_omega_(max_rpm * (2.0 * Math::PI / 60.0))
    , prop_diameter_(0.127)
    , thrust_coeff_(0.1)
    , torque_coeff_(0.01)
    , efficiency_(efficiency)
    , omega_(0.0)
    , omega_target_(0.0)
    , throttle_(0.0)
    , current_thrust_(0.0)
    , reaction_torque_(0.0)
    , voltage_(12.6)
{}

// ============================================================
//  Thrust / torque from omega
// ============================================================

double Motor::omegaToThrust(double omega, double rho) const {
    // T = Ct · ρ · (ω/(2π))² · D⁴
    // Simplified: T = k_t · rho · omega²
    // k_t = Ct * D^4 / (4π²)
    const double n   = omega / (2.0 * Math::PI);     // rev/s
    const double D4  = std::pow(prop_diameter_, 4.0);
    const double T_raw = thrust_coeff_ * rho * n * n * D4;

    // Scale so that at full throttle we get max_thrust (calibration)
    // Compute T at max_omega with sea-level density
    const double n_max  = max_omega_ / (2.0 * Math::PI);
    const double T_max_isa = thrust_coeff_ * Math::ISA_AIR_DENSITY * n_max * n_max * D4;
    if (T_max_isa < 1e-9) return 0.0;

    return T_raw * (max_thrust_ / T_max_isa);
}

double Motor::omegaToTorque(double omega, double rho) const {
    // Q = Cq · ρ · (ω/(2π))² · D⁵
    const double n   = omega / (2.0 * Math::PI);
    const double D5  = std::pow(prop_diameter_, 5.0);
    const double Q_raw = torque_coeff_ * rho * n * n * D5;

    // Scale relative to thrust model
    const double n_max  = max_omega_ / (2.0 * Math::PI);
    const double T_max_isa = thrust_coeff_ * Math::ISA_AIR_DENSITY * n_max * n_max *
                             std::pow(prop_diameter_, 4.0);
    const double Q_max_isa = torque_coeff_ * Math::ISA_AIR_DENSITY * n_max * n_max * D5;
    if (T_max_isa < 1e-9 || Q_max_isa < 1e-9) return 0.0;

    // Torque coefficient ratio: Q/T = Cq·D / Ct
    const double ratio = (torque_coeff_ * prop_diameter_) / thrust_coeff_;
    return omegaToThrust(omega, rho) * ratio;
}

// ============================================================
//  Commands
// ============================================================

void Motor::setThrottle(double throttle) {
    throttle_     = Math::clamp(throttle, 0.0, 1.0);
    omega_target_ = throttle_ * max_omega_;
}

void Motor::setTargetThrust(double thrust) {
    thrust = Math::clamp(thrust, 0.0, max_thrust_);
    // Invert: throttle = sqrt(thrust / max_thrust)
    // (since T ∝ ω² and ω = throttle · max_omega)
    if (max_thrust_ < 1e-9) {
        setThrottle(0.0);
        return;
    }
    const double t = std::sqrt(thrust / max_thrust_);
    setThrottle(t);
}

// ============================================================
//  Update
// ============================================================

void Motor::update(double dt, double air_density) {
    if (dt <= 0.0) return;

    // First-order lag toward target omega
    // ω[k+1] = ω[k] + (ω_target − ω[k]) · (1 − exp(−dt/τ))
    // For small dt/τ: ≈ ω[k] + (ω_target − ω[k]) · dt/τ
    const double alpha = 1.0 - std::exp(-dt / time_constant_);
    omega_ += (omega_target_ - omega_) * alpha;
    omega_ = Math::clamp(omega_, 0.0, max_omega_);

    // Update thrust and torque at current omega
    current_thrust_   = omegaToThrust(omega_, air_density);
    reaction_torque_  = omegaToTorque(omega_, air_density);
}

// ============================================================
//  Accessors
// ============================================================

double Motor::getCurrentThrust()  const { return current_thrust_; }
double Motor::getReactionTorque() const { return reaction_torque_; }
double Motor::getMaxThrust()      const { return max_thrust_; }
double Motor::getRPM()            const { return omega_ * (60.0 / (2.0 * Math::PI)); }
double Motor::getOmega()          const { return omega_; }
double Motor::getThrottle()       const { return throttle_; }
double Motor::getEfficiency()     const { return efficiency_; }

void Motor::setVoltage(double v) { voltage_ = v; }

double Motor::getPowerDraw() const {
    // P = T · v_induced / η
    // v_induced ≈ sqrt(T / (2 · ρ · A)) — hover induced velocity
    // Simplified: P ≈ T^(3/2) · K / η  (typical quadrotor scaling)
    const double rho = Math::ISA_AIR_DENSITY;
    const double disk_area = Math::PI * std::pow(prop_diameter_ / 2.0, 2.0);
    if (disk_area < 1e-9 || efficiency_ < 1e-9) return 0.0;
    const double v_induced = std::sqrt(current_thrust_ / (2.0 * rho * disk_area));
    return (current_thrust_ * v_induced) / efficiency_;
}

double Motor::getCurrentDraw() const {
    if (voltage_ < 0.1) return 0.0;
    return getPowerDraw() / voltage_;
}

// ============================================================
//  Reset
// ============================================================

void Motor::reset() {
    omega_          = 0.0;
    omega_target_   = 0.0;
    throttle_       = 0.0;
    current_thrust_ = 0.0;
    reaction_torque_ = 0.0;
}

} // namespace Flight
} // namespace AeroCore
