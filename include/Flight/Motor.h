/**
 * @file Motor.h
 * @brief Brushless DC motor + propeller model for AeroCore.
 *
 * ## Model
 * Motors are modelled as first-order systems with a configurable time constant τ.
 * The RPM is tracked directly and thrust / torque are derived from it:
 *
 *   ω̇   = (ω_target − ω) / τ                      [rad/s² — motor response]
 *   T    = Ct · ρ · n² · D⁴                         [N   — thrust coefficient]
 *   Q    = Cq · ρ · n² · D⁵                         [N·m — torque coefficient]
 *
 * where:
 *   - n   = ω / (2π)  [rev/s]
 *   - D   = propeller diameter [m]
 *   - ρ   = air density (injected from physics engine)
 *   - Ct  = non-dimensional thrust coefficient
 *   - Cq  = non-dimensional torque coefficient
 *
 * For a simplified model the coefficients are collapsed to:
 *   T = k_thrust · ω²
 *   Q = k_torque · ω²
 *
 * which is accurate for fixed-pitch propellers in hover.
 *
 * ## Battery Coupling
 * Motor current draw is estimated from:
 *   I = (T · v_eff) / (η · V_bat)
 * where η is motor+ESC efficiency, v_eff is induced velocity, and V_bat is
 * battery voltage.  This lets the battery model drain realistically.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include "Utilities/Config.h"
#include <algorithm>
#include <cmath>

namespace AeroCore {
namespace Flight {

/**
 * @brief Single brushless motor + propeller unit.
 *
 * One Motor instance corresponds to one ESC+motor+propeller assembly.
 * The vehicle model (Drone/FixedWing) owns a collection of Motor objects.
 */
class Motor {
public:
    /**
     * @brief Construct a motor from config.
     *
     * Reads from [motor] section:
     *  - response_time_constant [s]    (1st-order time constant, default 0.1)
     *  - max_rpm                [RPM]  (no-load max speed, default 10000)
     *  - propeller_diameter     [m]    (default 0.127 = 5 inch)
     *  - thrust_coefficient     Ct     (dimensionless, default 0.1)
     *  - torque_coefficient     Cq     (dimensionless, default 0.01)
     *  - efficiency                    (motor+ESC, 0–1, default 0.85)
     *
     * And from [drone]:
     *  - max_thrust             [N]    (per motor, default 20)
     */
    explicit Motor(const Utilities::Config& config);

    /**
     * @brief Construct with explicit parameters (useful for programmatic setup).
     *
     * @param max_thrust    Maximum thrust per motor [N].
     * @param time_constant First-order response time constant τ [s].
     * @param max_rpm       Maximum motor speed [RPM].
     * @param efficiency    Motor efficiency η ∈ (0, 1].
     */
    Motor(double max_thrust, double time_constant, double max_rpm, double efficiency);

    // ----------------------------------------------------------
    //  Update
    // ----------------------------------------------------------

    /**
     * @brief Advance motor state by @p dt.
     *
     * Updates ω toward the throttle command using a 1st-order lag.
     * Call once per physics tick.
     *
     * @param dt           Time step [s].
     * @param air_density  Current air density [kg/m³] (affects Ct, Cq scaling).
     */
    void update(double dt, double air_density = 1.225);

    // ----------------------------------------------------------
    //  Command
    // ----------------------------------------------------------

    /**
     * @brief Set the throttle command (normalised 0–1).
     *
     * 0 = motor stopped, 1 = full throttle.
     * Internally maps to a target RPM / thrust.
     *
     * @param throttle  Normalised throttle ∈ [0, 1].
     */
    void setThrottle(double throttle);

    /**
     * @brief Set a direct thrust setpoint [N].
     *
     * Convenience wrapper — converts thrust to throttle internally.
     *
     * @param thrust  Desired thrust [N].  Clamped to [0, max_thrust].
     */
    void setTargetThrust(double thrust);

    // ----------------------------------------------------------
    //  State accessors
    // ----------------------------------------------------------

    double getCurrentThrust()  const;  ///< Current thrust output [N]
    double getReactionTorque() const;  ///< Counter-torque on vehicle body [N·m]
    double getMaxThrust()      const;  ///< Maximum achievable thrust [N]
    double getRPM()            const;  ///< Current angular speed [RPM]
    double getOmega()          const;  ///< Current angular speed [rad/s]
    double getThrottle()       const;  ///< Current throttle ∈ [0, 1]
    double getEfficiency()     const;  ///< Motor efficiency η
    double getCurrentDraw()    const;  ///< Estimated current draw [A] (requires setVoltage())
    double getPowerDraw()      const;  ///< Estimated power draw [W]

    /**
     * @brief Provide battery voltage for current estimation.
     * @param voltage  Battery terminal voltage [V].
     */
    void setVoltage(double voltage);

    // ----------------------------------------------------------
    //  Reset
    // ----------------------------------------------------------

    /// Reset motor to zero RPM / thrust.
    void reset();

private:
    double max_thrust_;         ///< Maximum thrust [N]
    double time_constant_;      ///< 1st-order time constant τ [s]
    double max_omega_;          ///< Maximum angular velocity [rad/s]
    double prop_diameter_;      ///< Propeller diameter D [m]
    double thrust_coeff_;       ///< Thrust coefficient Ct (normalised)
    double torque_coeff_;       ///< Torque coefficient Cq (normalised)
    double efficiency_;         ///< Motor+ESC efficiency η

    double omega_;              ///< Current angular velocity [rad/s]
    double omega_target_;       ///< Target angular velocity [rad/s]
    double throttle_;           ///< Current throttle command [0–1]
    double current_thrust_;     ///< Current thrust [N]
    double reaction_torque_;    ///< Reaction torque on body [N·m]
    double voltage_;            ///< Battery voltage for I estimation [V]

    // --- Helper ---
    /// Convert omega [rad/s] to thrust [N] at given air density.
    double omegaToThrust(double omega, double rho) const;
    /// Convert omega [rad/s] to reaction torque [N·m].
    double omegaToTorque(double omega, double rho) const;
};

} // namespace Flight
} // namespace AeroCore
