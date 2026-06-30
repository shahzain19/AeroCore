/**
 * @file PIDController.h
 * @brief Production-grade PID controller with anti-windup, derivative filtering,
 *        and feed-forward support.
 *
 * ## Algorithm
 * The discrete PID algorithm implemented here is the **derivative-on-measurement**
 * (D-o-M) variant, which avoids the "derivative kick" that occurs when the
 * setpoint changes abruptly in classic error-derivative forms.
 *
 * ```
 * e[k]       = setpoint − measurement
 *
 * P term:    P = Kp · e[k]
 *
 * I term:    I += Ki · e[k] · dt            (clamped by integral_max)
 *
 * D term:    raw_d = (measurement[k] − measurement[k−1]) / dt
 *            D  = −Kd · low_pass(raw_d)     (filtered, sign inverted)
 *
 * FF term:   FF = Kff · setpoint             (optional feed-forward)
 *
 * output = clamp(P + I + D + FF, out_min, out_max)
 * ```
 *
 * ### Anti-windup
 * Two strategies are selectable via `setAntiWindupMode()`:
 * - **CLAMP**  — integrator is clamped to ±integral_max and stops growing
 *               when the output is saturated.
 * - **BACK_CALC** — back-calculation: the integrator is decremented by the
 *               saturation error multiplied by a back-calculation gain `Kb`.
 *
 * ### Low-pass derivative filter
 * A first-order IIR filter is applied to the raw derivative:
 *   y[k] = α · raw[k] + (1−α) · y[k−1]
 * where α = `derivative_filter_alpha` ∈ (0, 1].  α = 1 means no filtering.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include "Utilities/Config.h"
#include <algorithm>

namespace AeroCore {
namespace Flight {

/**
 * @brief Anti-windup strategy selector.
 */
enum class AntiWindupMode {
    CLAMP,       ///< Simple clamping + conditional integration
    BACK_CALC    ///< Back-calculation with gain Kb
};

/**
 * @brief Single-axis PID controller.
 *
 * Designed to be instantiated once per control axis (e.g. altitude, roll,
 * pitch, yaw-rate).  Multiple instances are chained for cascaded control.
 *
 * ### Example: altitude hold
 * ```cpp
 * PIDController alt_pid(config);
 * alt_pid.setGains(10.0, 0.5, 5.0);
 * alt_pid.setOutputLimits(-20.0, 20.0);
 * double thrust_delta = alt_pid.update(dt, target_alt, measured_alt);
 * ```
 */
class PIDController {
public:
    /**
     * @brief Construct from config, reading [pid] section.
     *
     * Expected keys (all optional — sensible defaults used):
     *  - kp, ki, kd, kff
     *  - output_min, output_max
     *  - integral_max
     *  - derivative_filter   (alpha in (0,1])
     *  - back_calc_gain      (Kb for BACK_CALC mode)
     */
    explicit PIDController(const Utilities::Config& config,
                           const std::string& section = "pid");

    /**
     * @brief Minimal constructor for programmatic setup.
     * @param kp  Proportional gain.
     * @param ki  Integral gain.
     * @param kd  Derivative gain.
     */
    PIDController(double kp, double ki, double kd);

    // ----------------------------------------------------------
    //  Core update
    // ----------------------------------------------------------

    /**
     * @brief Compute PID output for one time step.
     *
     * Call once per control loop iteration with a fixed @p dt.
     *
     * @param dt           Time step [s].  Must be > 0.
     * @param setpoint     Desired value.
     * @param measurement  Measured (actual) value.
     * @return             Clamped PID output.
     */
    double update(double dt, double setpoint, double measurement);

    // ----------------------------------------------------------
    //  Configuration
    // ----------------------------------------------------------

    /// Set all three PID gains in one call.
    void setGains(double kp, double ki, double kd);

    /// Set individual gains.
    void setKp(double kp);
    void setKi(double ki);
    void setKd(double kd);

    /// Feed-forward gain (multiplied by the setpoint directly).
    void setFeedForwardGain(double kff);

    /// Clamp PID output to [min, max].
    void setOutputLimits(double min_out, double max_out);

    /// Maximum absolute value for the integral term.
    void setIntegralMax(double max);

    /**
     * @brief Set derivative low-pass filter coefficient α.
     * @param alpha  ∈ (0, 1].  1 = no filtering; 0.1 = heavy filtering.
     */
    void setDerivativeFilter(double alpha);

    /// Choose anti-windup strategy.
    void setAntiWindupMode(AntiWindupMode mode);

    /// Back-calculation gain for BACK_CALC anti-windup.
    void setBackCalcGain(double kb);

    // ----------------------------------------------------------
    //  State management
    // ----------------------------------------------------------

    /// Zero all internal state (integrator, previous measurement, …).
    void reset();

    // ----------------------------------------------------------
    //  Diagnostics / getters
    // ----------------------------------------------------------

    double getKp()         const;  ///< Proportional gain
    double getKi()         const;  ///< Integral gain
    double getKd()         const;  ///< Derivative gain
    double getKff()        const;  ///< Feed-forward gain
    double getError()      const;  ///< Last computed error (e[k])
    double getIntegral()   const;  ///< Current integrator state
    double getDerivative() const;  ///< Last filtered derivative
    double getOutput()     const;  ///< Last clamped output
    double getSetpoint()   const;  ///< Last setpoint used
    double getMeasurement() const; ///< Last measurement used

private:
    // Gains
    double kp_;    ///< Proportional gain
    double ki_;    ///< Integral gain
    double kd_;    ///< Derivative gain
    double kff_;   ///< Feed-forward gain

    // Output limits
    double output_min_;   ///< Lower output clamp
    double output_max_;   ///< Upper output clamp

    // Integral anti-windup
    double integral_max_;       ///< Maximum |integral|
    AntiWindupMode aw_mode_;    ///< Active anti-windup strategy
    double back_calc_gain_;     ///< Kb for back-calculation

    // Derivative filter
    double deriv_alpha_;        ///< IIR coefficient (0 < α ≤ 1)

    // State
    double error_;              ///< Current error e[k]
    double integral_;           ///< Integrator state
    double prev_measurement_;   ///< y[k−1] for derivative-on-measurement
    double filtered_deriv_;     ///< Low-pass filtered derivative
    double output_;             ///< Last output value
    double setpoint_;           ///< Last setpoint
    double measurement_;        ///< Last measurement
    bool   first_update_;       ///< True before first call to update()
};

} // namespace Flight
} // namespace AeroCore
