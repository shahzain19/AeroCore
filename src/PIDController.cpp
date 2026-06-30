/**
 * @file PIDController.cpp
 * @brief Production-grade PID controller implementation.
 *
 * Key implementation notes:
 *
 * 1. **Derivative-on-measurement** — the derivative term is computed from
 *    the rate of change of the *measurement*, not the error.  This eliminates
 *    derivative kick when the setpoint steps.
 *
 * 2. **Anti-windup** — two modes:
 *    - CLAMP: integrator held constant when output is saturated
 *    - BACK_CALC: integrator decremented by Kb * (output_sat − output_unsat)
 *
 * 3. **First-update guard** — the derivative is zeroed on the first call so a
 *    large initial measurement doesn't produce a huge spike.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Flight/PIDController.h"
#include <stdexcept>

namespace AeroCore {
namespace Flight {

// ============================================================
//  Construction
// ============================================================

PIDController::PIDController(const Utilities::Config& config,
                             const std::string& section)
    : kp_(0.0), ki_(0.0), kd_(0.0), kff_(0.0)
    , output_min_(-1e9), output_max_(1e9)
    , integral_max_(1e9)
    , aw_mode_(AntiWindupMode::CLAMP)
    , back_calc_gain_(0.1)
    , deriv_alpha_(1.0)  // No filtering by default
    , error_(0.0), integral_(0.0)
    , prev_measurement_(0.0), filtered_deriv_(0.0)
    , output_(0.0), setpoint_(0.0), measurement_(0.0)
    , first_update_(true)
{
    // Helper lambda: safely read a double from config or use a default
    auto tryGet = [&](const std::string& key, double fallback) -> double {
        try { return config.get<double>(section, key); }
        catch (...) { return fallback; }
    };

    kp_             = tryGet("kp",             0.0);
    ki_             = tryGet("ki",             0.0);
    kd_             = tryGet("kd",             0.0);
    kff_            = tryGet("kff",            0.0);
    output_min_     = tryGet("output_min",    -1e9);
    output_max_     = tryGet("output_max",     1e9);
    integral_max_   = tryGet("integral_max",   1e9);
    deriv_alpha_    = tryGet("derivative_filter", 1.0);
    back_calc_gain_ = tryGet("back_calc_gain", 0.1);
}

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), kff_(0.0)
    , output_min_(-1e9), output_max_(1e9)
    , integral_max_(1e9)
    , aw_mode_(AntiWindupMode::CLAMP)
    , back_calc_gain_(0.1)
    , deriv_alpha_(1.0)
    , error_(0.0), integral_(0.0)
    , prev_measurement_(0.0), filtered_deriv_(0.0)
    , output_(0.0), setpoint_(0.0), measurement_(0.0)
    , first_update_(true)
{}

// ============================================================
//  Core update
// ============================================================

double PIDController::update(double dt, double setpoint, double measurement) {
    if (dt <= 0.0) return output_;

    setpoint_    = setpoint;
    measurement_ = measurement;

    // --- Proportional term ---
    error_ = setpoint - measurement;
    const double P = kp_ * error_;

    // --- Feed-forward term ---
    const double FF = kff_ * setpoint;

    // --- Derivative term (on measurement) ---
    // First update: skip derivative to avoid initial kick
    double raw_deriv = 0.0;
    if (!first_update_) {
        raw_deriv = -(measurement - prev_measurement_) / dt;
    }
    first_update_ = false;
    prev_measurement_ = measurement;

    // Low-pass IIR filter: y[k] = α·raw[k] + (1−α)·y[k−1]
    filtered_deriv_ = deriv_alpha_ * raw_deriv + (1.0 - deriv_alpha_) * filtered_deriv_;
    const double D = kd_ * filtered_deriv_;

    // --- Pre-saturation output (for anti-windup) ---
    double output_unsat = P + integral_ + D + FF;

    // --- Anti-windup: conditional integration ---
    bool saturated = false;
    if (aw_mode_ == AntiWindupMode::CLAMP) {
        // Only integrate if not saturated OR if error would bring us back
        const double projected = output_unsat + ki_ * error_ * dt;
        saturated = (projected > output_max_ && error_ > 0.0) ||
                    (projected < output_min_ && error_ < 0.0);
    }

    if (!saturated) {
        integral_ += ki_ * error_ * dt;
        // Clamp integrator
        integral_ = std::clamp(integral_, -integral_max_, integral_max_);
    }

    // Back-calculation anti-windup: bleed integrator on saturation
    if (aw_mode_ == AntiWindupMode::BACK_CALC) {
        double out_pre_clamp = P + integral_ + D + FF;
        double out_clamped   = std::clamp(out_pre_clamp, output_min_, output_max_);
        // Back-calc correction
        integral_ -= back_calc_gain_ * (out_pre_clamp - out_clamped);
    }

    // --- Final output ---
    output_ = std::clamp(P + integral_ + D + FF, output_min_, output_max_);

    return output_;
}

// ============================================================
//  Configuration
// ============================================================

void PIDController::setGains(double kp, double ki, double kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
}
void PIDController::setKp(double kp)   { kp_  = kp; }
void PIDController::setKi(double ki)   { ki_  = ki; }
void PIDController::setKd(double kd)   { kd_  = kd; }
void PIDController::setFeedForwardGain(double kff) { kff_ = kff; }

void PIDController::setOutputLimits(double min_out, double max_out) {
    if (min_out >= max_out) {
        throw std::invalid_argument("PIDController: min_out must be < max_out");
    }
    output_min_ = min_out;
    output_max_ = max_out;
}

void PIDController::setIntegralMax(double max) {
    if (max < 0.0) throw std::invalid_argument("PIDController: integral_max must be >= 0");
    integral_max_ = max;
}

void PIDController::setDerivativeFilter(double alpha) {
    deriv_alpha_ = std::clamp(alpha, 0.001, 1.0);
}

void PIDController::setAntiWindupMode(AntiWindupMode mode) { aw_mode_ = mode; }
void PIDController::setBackCalcGain(double kb) { back_calc_gain_ = kb; }

// ============================================================
//  State
// ============================================================

void PIDController::reset() {
    error_         = 0.0;
    integral_      = 0.0;
    prev_measurement_ = 0.0;
    filtered_deriv_   = 0.0;
    output_        = 0.0;
    setpoint_      = 0.0;
    measurement_   = 0.0;
    first_update_  = true;
}

// ============================================================
//  Diagnostics
// ============================================================

double PIDController::getKp()          const { return kp_;  }
double PIDController::getKi()          const { return ki_;  }
double PIDController::getKd()          const { return kd_;  }
double PIDController::getKff()         const { return kff_; }
double PIDController::getError()       const { return error_;  }
double PIDController::getIntegral()    const { return integral_; }
double PIDController::getDerivative()  const { return filtered_deriv_; }
double PIDController::getOutput()      const { return output_; }
double PIDController::getSetpoint()    const { return setpoint_; }
double PIDController::getMeasurement() const { return measurement_; }

} // namespace Flight
} // namespace AeroCore
