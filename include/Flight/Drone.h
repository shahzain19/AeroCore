/**
 * @file Drone.h
 * @brief Multirotor (quadrotor) vehicle model for AeroCore.
 *
 * ## Architecture
 *
 * Drone is the primary **vehicle model** for multirotor aircraft.  It owns:
 *  - A collection of `Motor` objects (typically 4 for a quadrotor)
 *  - The vehicle's rigid-body state (`RigidBodyState` from PhysicsEngine)
 *  - Battery state
 *
 * It provides the bridge between the flight controller's motor commands
 * and the forces/torques consumed by the physics engine.
 *
 * ## Quadrotor Motor Layout (X-frame, default)
 *
 * ```
 *         [M0 CW]     [M1 CCW]
 *           \\           /
 *            +----+----+
 *            |  DRONE  |
 *            +----+----+
 *           /           \\
 *        [M3 CCW]    [M2 CW]
 * ```
 *
 * Motors are numbered 0–3.  CW/CCW refers to top-down view.  This is the
 * standard Betaflight / ArduPilot "X" layout.
 *
 * ## Coordinate System (Body Frame, NED)
 * - +X  = forward (nose)
 * - +Y  = right   (starboard)
 * - +Z  = down    (under-belly)
 *
 * Positive thrust is therefore in the −Z direction (upward in NED).
 *
 * ## Mixing Matrix
 * Motor throttle outputs are computed from the normalised control inputs
 * (roll_cmd, pitch_cmd, yaw_cmd, throttle_cmd) via a 4×4 mixing matrix M:
 *
 *   [T0, T1, T2, T3]ᵀ = M · [throttle, roll, pitch, yaw]ᵀ
 *
 * For the standard X-frame:
 *   T0 = throttle + roll + pitch − yaw
 *   T1 = throttle − roll + pitch + yaw
 *   T2 = throttle − roll − pitch − yaw
 *   T3 = throttle + roll − pitch + yaw
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include "Math/Vector.h"
#include "Flight/Motor.h"
#include "Physics/PhysicsEngine.h"
#include "Utilities/Config.h"
#include <vector>
#include <memory>
#include <string>

namespace AeroCore {
namespace Flight {

enum class AirframeType {
    MULTIROTOR = 0,
    FIXED_WING
};

/**
 * @brief Control input structure for the drone mixer.
 *
 * All values are normalised to [−1, +1] except throttle which is [0, 1].
 */
struct ControlInput {
    double throttle  = 0.0;  ///< Collective thrust command [0, 1]
    double roll      = 0.0;  ///< Roll rate / angle command [-1, 1] (+right)
    double pitch     = 0.0;  ///< Pitch rate / angle command [-1, 1] (+nose up)
    double yaw       = 0.0;  ///< Yaw rate command [-1, 1] (+CW from above)
};

/**
 * @brief Multirotor vehicle model.
 *
 * Manages motor state, battery, rotor geometry and the mixing matrix.
 * The flight controller drives the drone via `applyControl()`.
 */
class Drone {
public:
    /**
     * @brief Construct a drone from config.
     *
     * Reads from [drone]:
     *  - mass           [kg]
     *  - arm_length     [m]   (motor-to-centre distance)
     *  - size           [m]   (visual size hint for renderer)
     *  - battery_voltage_max [V]
     *  - battery_capacity    [Ah]
     *  - battery_internal_resistance [Ω]
     *
     * And from [motor] for each motor instance.
     */
    explicit Drone(const Utilities::Config& config);

    // ----------------------------------------------------------
    //  Motor management
    // ----------------------------------------------------------

    /**
     * @brief Add a motor to the vehicle.
     *
     * Motors are added in order: 0 = front-right CW, 1 = front-left CCW,
     * 2 = rear-right CW, 3 = rear-left CCW (standard X-frame).
     */
    void addMotor(std::unique_ptr<Motor> motor);

    /**
     * @brief Set throttle for a specific motor by index.
     * @param index     Motor index [0, num_motors).
     * @param throttle  Normalised throttle [0, 1].
     */
    void setMotorThrottle(size_t index, double throttle);

    /**
     * @brief Set thrust setpoint for a specific motor [N].
     * @param index   Motor index.
     * @param thrust  Desired thrust [N].
     */
    void setMotorThrust(size_t index, double thrust);

    /**
     * @brief Apply a normalised 4-channel control input through the mixer.
     *
     * The mixer maps the control inputs to per-motor throttle commands using
     * the standard X-frame mixing matrix.
     *
     * @param input  Normalised control input.
     */
    void applyControl(const ControlInput& input);

    // ----------------------------------------------------------
    //  Physics interface
    // ----------------------------------------------------------

    /**
     * @brief Build the ExternalForces structure for the current motor state.
     *
     * Computes net thrust in world frame and net torque in body frame
     * from all active motors.  Called by the simulation loop before
     * PhysicsEngine::step().
     *
     * @param orientation  Current vehicle orientation quaternion.
     * @param air_density  Air density [kg/m³] for motor aerodynamics.
     * @return ExternalForces ready to pass to PhysicsEngine.
     */
    Physics::ExternalForces buildExternalForces(
        const Math::Quaterniond& orientation,
        const Math::Vector3d& wind_world,
        double air_density) const;

    // ----------------------------------------------------------
    //  State management
    // ----------------------------------------------------------

    /**
     * @brief Update motor dynamics.  Call once per physics tick.
     * @param dt           Time step [s].
     * @param air_density  Current air density [kg/m³].
     */
    void update(double dt, double air_density = 1.225);

    /// Reset to initial state (zero velocity, ground position, full battery).
    void reset();

    // ----------------------------------------------------------
    //  Rigid-body state
    // ----------------------------------------------------------

    const Physics::RigidBodyState& getState()    const;
    Physics::RigidBodyState&       getState();

    // Convenience wrappers delegating to state
    Math::Vector3d   getPosition()    const;
    Math::Vector3d   getVelocity()    const;
    Math::Vector3d   getAcceleration() const;
    Math::Quaterniond getOrientation() const;
    Math::Vector3d   getAngularVelocity() const;
    Math::Vector3d   getEulerAngles() const;  ///< (roll, pitch, yaw) [rad]

    double getAltitude()  const;  ///< Altitude above ground [m] = −position.z()

    void setState(const Physics::RigidBodyState& state);

    // ----------------------------------------------------------
    //  Vehicle parameters
    // ----------------------------------------------------------

    double getMass()            const;
    double getArmLength()       const;  ///< Motor arm length [m]
    double getSize()            const;  ///< Visual size [m]
    double getMaxTotalThrust()  const;  ///< Sum of all motor max thrust [N]
    size_t getMotorCount()      const;
    const Motor& getMotor(size_t idx) const;

    /**
     * @brief Get the current net thrust vector in the body frame [N].
     * All motor thrusts summed; direction is −Z (up) in body frame.
     */
    Math::Vector3d getThrustBody() const;

    /**
     * @brief Get the current net thrust vector in the world (NED) frame [N].
     */
    Math::Vector3d getThrustWorld(const Math::Quaterniond& orientation) const;

    // ----------------------------------------------------------
    //  Battery
    // ----------------------------------------------------------

    double getBatteryVoltage()     const;  ///< Terminal voltage [V]
    double getBatteryPercentage()  const;  ///< State of charge [0–100 %]
    double getTotalCurrentDraw()   const;  ///< Sum of all motor currents [A]

    /**
     * @brief Update battery state based on motor current draw.
     * @param dt  Time step [s].
     */
    void updateBattery(double dt);

    // ----------------------------------------------------------
    //  Inertia (for physics engine)
    // ----------------------------------------------------------

    /**
     * @brief Return the inertia tensor for this drone configuration [kg·m²].
     *
     * Computed from mass, arm length and a point-mass approximation for each
     * motor.  Can be overridden by config.
     */
    Math::Matrix3d getInertiaTensor() const;
    AirframeType getAirframeType() const;
    size_t getRecommendedMotorCount() const;

private:
    AirframeType airframe_type_;
    size_t recommended_motor_count_;

    // Vehicle parameters
    double mass_;           ///< Total mass [kg]
    double arm_length_;     ///< Motor arm length (centre to motor) [m]
    double size_;           ///< Visual size hint [m]

    // Battery model
    double battery_voltage_max_;         ///< Fully charged voltage [V]
    double battery_capacity_ah_;         ///< Capacity [Ah]
    double battery_internal_resistance_; ///< Internal resistance [Ω]
    double battery_charge_ah_;           ///< Current remaining charge [Ah]

    // Motors
    std::vector<std::unique_ptr<Motor>> motors_;

    // Rigid-body state (position, velocity, orientation, angular velocity)
    Physics::RigidBodyState state_;

    // Motor body-frame positions (precomputed from arm_length_)
    std::vector<Math::Vector3d> motor_positions_;

    // Motor spin directions (+1 CCW, -1 CW), for torque calculation
    std::vector<int> motor_spin_directions_;

    /// Build motor positions for a standard X-frame quadrotor.
    void buildMotorGeometry();
};

} // namespace Flight
} // namespace AeroCore
