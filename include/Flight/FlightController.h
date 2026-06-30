/**
 * @file FlightController.h
 * @brief Cascaded PID flight controller for AeroCore multirotors.
 *
 * ## Architecture — Cascaded Control Loops
 *
 * ```
 *  ┌─────────────┐      ┌──────────────┐      ┌──────────────┐      ┌─────────────┐
 *  │  Position   │──e──►│  Velocity    │──e──►│  Attitude    │──e──►│  Rate       │
 *  │  Controller │      │  Controller  │      │  Controller  │      │  Controller │
 *  │  (outer)    │      │  (mid)       │      │  (inner)     │      │  (inner-most│
 *  └─────────────┘      └──────────────┘      └──────────────┘      └─────────────┘
 *        │                    │                     │                      │
 *        ▼                    ▼                     ▼                      ▼
 *   pos error → vel_cmd  vel error → att_cmd  att error → rate_cmd  rate error → motor_cmd
 * ```
 *
 * The controller is mode-aware:
 *  - DISARMED / ARMED    → all outputs zeroed
 *  - TAKEOFF / LANDING   → altitude ramp + attitude stabilise
 *  - STABILIZE           → rate control only (acro)
 *  - ATTITUDE_HOLD       → outer attitude + inner rate
 *  - ALTITUDE_HOLD       → altitude → throttle + attitude control
 *  - POSITION_HOLD       → position → velocity → attitude + altitude
 *
 * ## Axis Conventions (Body Frame NED)
 *
 *  - Roll  φ : right wing down = positive roll
 *  - Pitch θ : nose up         = positive pitch
 *  - Yaw   ψ : nose right (CW from above) = positive yaw
 *
 * ## PID Cascade Details
 *
 * ### Altitude channel
 *   setpoint: target_altitude   measurement: altitude
 *   output  : throttle_cmd ∈ [0, 1] (maps to total thrust)
 *
 * ### Roll channel
 *   setpoint: target_roll   measurement: roll angle
 *   output  : roll_rate_cmd (fed into rate PID)
 *   rate PID setpoint: roll_rate_cmd   measurement: gyro_roll_rate
 *   output  : roll_torque_cmd
 *
 * ### Pitch / Yaw channels — analogous to Roll.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include "Math/Vector.h"
#include "Flight/FlightMode.h"
#include "Flight/Drone.h"
#include "Flight/PIDController.h"
#include "Sensors/IMU.h"
#include "Sensors/Altimeter.h"
#include "Sensors/BatterySensor.h"
#include "Utilities/Config.h"
#include "Utilities/Logger.h"
#include <memory>

namespace AeroCore {
namespace Flight {

/**
 * @brief Pilot / autopilot commands fed to the flight controller each tick.
 *
 * All values normalised to [−1, +1] unless noted.
 */
struct PilotInput {
    double throttle  = 0.0;  ///< Collective throttle [0, 1]
    double roll      = 0.0;  ///< Roll stick [-1, 1]
    double pitch     = 0.0;  ///< Pitch stick [-1, 1]
    double yaw       = 0.0;  ///< Yaw stick [-1, 1]
};

/**
 * @brief Diagnostic snapshot of all PID states, useful for telemetry.
 */
struct ControllerDiagnostics {
    // Altitude
    double alt_error;
    double alt_integral;
    double alt_derivative;
    double alt_output;

    // Attitude (angles)
    double roll_error;
    double pitch_error;
    double yaw_error;

    // Rates
    double roll_rate_error;
    double pitch_rate_error;
    double yaw_rate_error;

    // Motor commands (normalised 0–1)
    double motor_cmd[4];

    // Current mode
    FlightMode mode;
};

/**
 * @brief Cascaded PID flight controller for quadrotor vehicles.
 */
class FlightController {
public:
    /**
     * @brief Construct with shared sensors and config.
     *
     * @param drone          The vehicle to control.
     * @param imu            IMU for attitude and rate feedback.
     * @param altimeter      Altimeter for altitude feedback.
     * @param battery_sensor Battery sensor for voltage monitoring.
     * @param config         Simulation/controller configuration.
     */
    FlightController(std::shared_ptr<Drone>                  drone,
                     std::shared_ptr<Sensors::IMU>            imu,
                     std::shared_ptr<Sensors::Altimeter>      altimeter,
                     std::shared_ptr<Sensors::BatterySensor>  battery_sensor,
                     const Utilities::Config& config);

    // ----------------------------------------------------------
    //  Main update — call once per physics tick
    // ----------------------------------------------------------

    /**
     * @brief Run one control loop iteration.
     *
     * Reads sensor data, runs the active PID cascade, and writes motor
     * commands to the drone.
     *
     * @param dt  Time step [s].
     */
    void update(double dt);

    // ----------------------------------------------------------
    //  Mode transitions
    // ----------------------------------------------------------

    /**
     * @brief Request a transition to @p mode.
     *
     * Illegal transitions (e.g. POSITION_HOLD from DISARMED) are silently
     * rejected and a warning is logged.
     */
    void requestMode(FlightMode mode);

    void arm();      ///< DISARMED → ARMED
    void disarm();   ///< Any mode → DISARMED (emergency cut)
    void takeoff();  ///< ARMED → TAKEOFF
    void land();     ///< Any flight mode → LANDING
    void reset();    ///< Reset all PID state and return to DISARMED

    // ----------------------------------------------------------
    //  Setpoints
    // ----------------------------------------------------------

    void setTargetAltitude(double altitude_m);
    void setTargetPosition(const Math::Vector3d& pos_ned);
    void setTargetHeading(double yaw_rad);

    /**
     * @brief Inject pilot/autopilot stick inputs for STABILIZE/ATT_HOLD mode.
     */
    void setPilotInput(const PilotInput& input);

    // ----------------------------------------------------------
    //  Getters
    // ----------------------------------------------------------

    FlightMode getMode()           const;
    double     getTargetAltitude() const;

    /// Read-only access to the altitude PID for telemetry.
    const PIDController& getAltitudePID()   const;
    const PIDController& getRollRatePID()   const;
    const PIDController& getPitchRatePID()  const;
    const PIDController& getYawRatePID()    const;

    /// Full diagnostic snapshot.
    ControllerDiagnostics getDiagnostics() const;

private:
    // Shared resources
    std::shared_ptr<Drone>                 drone_;
    std::shared_ptr<Sensors::IMU>          imu_;
    std::shared_ptr<Sensors::Altimeter>    altimeter_;
    std::shared_ptr<Sensors::BatterySensor> battery_sensor_;

    // PIDs — altitude cascade
    std::unique_ptr<PIDController> pid_alt_;      ///< Altitude → throttle

    // PIDs — attitude outer loop (angle → rate command)
    std::unique_ptr<PIDController> pid_roll_;     ///< Roll angle → roll rate cmd
    std::unique_ptr<PIDController> pid_pitch_;    ///< Pitch angle → pitch rate cmd
    std::unique_ptr<PIDController> pid_yaw_;      ///< Yaw angle → yaw rate cmd

    // PIDs — rate inner loop (rate → torque command)
    std::unique_ptr<PIDController> pid_roll_rate_;   ///< Roll rate  → roll torque
    std::unique_ptr<PIDController> pid_pitch_rate_;  ///< Pitch rate → pitch torque
    std::unique_ptr<PIDController> pid_yaw_rate_;    ///< Yaw rate   → yaw torque

    // State
    FlightMode        mode_;               ///< Current flight mode
    double            target_altitude_;    ///< Target altitude [m]
    Math::Vector3d    target_position_;    ///< Target position, NED [m]
    double            target_heading_;     ///< Target heading [rad]
    PilotInput        pilot_input_;        ///< Current stick inputs

    // Takeoff/landing ramp state
    double            takeoff_alt_ramp_;   ///< Current altitude ramp target [m]
    double            final_target_alt_;   ///< Ultimate target altitude for takeoff
    Math::Vector3d    home_position_;    ///< Home position captured on arm [NED m]

    // Config-derived limits
    double max_roll_angle_;   ///< Max roll angle [rad] in ATT_HOLD
    double max_pitch_angle_;  ///< Max pitch angle [rad] in ATT_HOLD
    double max_roll_rate_;    ///< Max roll rate [rad/s] in STABILIZE
    double max_pitch_rate_;   ///< Max pitch rate [rad/s] in STABILIZE
    double max_yaw_rate_;     ///< Max yaw rate [rad/s]
    double max_tilt_angle_;   ///< Max combined tilt [rad] (safety)
    double hover_throttle_;   ///< Throttle needed to hover at 1g [0–1]

    // ----------------------------------------------------------
    //  Mode-specific update methods
    // ----------------------------------------------------------

    void updateDisarmed(double dt);
    void updateArmed(double dt);
    void updateTakeoff(double dt);
    void updateLanding(double dt);
    void updateStabilize(double dt);
    void updateAttitudeHold(double dt);
    void updateAltitudeHold(double dt);
    void updatePositionHold(double dt);
    void updateReturnHome(double dt);
    void updateFailsafe(double dt);

    // ----------------------------------------------------------
    //  Helpers
    // ----------------------------------------------------------

    /**
     * @brief Run the inner attitude+rate PID loops and write motor commands.
     *
     * @param throttle_cmd  Collective throttle [0, 1].
     * @param roll_angle_sp Roll setpoint [rad].
     * @param pitch_angle_sp Pitch setpoint [rad].
     * @param yaw_rate_sp   Yaw rate setpoint [rad/s].
     * @param dt            Time step [s].
     */
    void runAttitudeControl(double throttle_cmd,
                            double roll_angle_sp,
                            double pitch_angle_sp,
                            double yaw_rate_sp,
                            double dt);

    /// Mix throttle + roll/pitch/yaw torques into per-motor commands.
    void mixMotors(double throttle, double roll_cmd,
                   double pitch_cmd, double yaw_cmd);

    /// Check for failsafe conditions (low battery, sensor loss, …).
    bool checkFailsafe();

    /// Log a mode transition.
    void logModeTransition(FlightMode from, FlightMode to);

    /// TAKEOFF → ALTITUDE_HOLD with PID state cleared.
    void transitionToAltitudeHold();

    /**
     * @brief Altitude PID → collective throttle with hover-relative floor.
     * @param min_throttle  Absolute minimum throttle [0,1].
     * @param hover_floor   When true, floor is max(min_throttle, 70% of hover).
     */
    double computeAltitudeThrottle(double dt, double setpoint_alt,
                                   double min_throttle, bool hover_floor);
};

} // namespace Flight
} // namespace AeroCore
