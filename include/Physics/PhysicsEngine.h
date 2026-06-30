/**
 * @file PhysicsEngine.h
 * @brief Full 6-DOF rigid-body physics engine for AeroCore.
 *
 * ## Overview
 * PhysicsEngine implements the equations of motion for a rigid body in 3-D
 * space using a **NED (North-East-Down)** inertial reference frame.
 *
 * ## Equations of Motion
 *
 * ### Translational (Newton's 2nd law in world frame)
 *   F_total = F_thrust + F_gravity + F_drag + F_wind
 *   a_world  = F_total / m
 *   v_world  = ∫ a_world dt
 *   p_world  = ∫ v_world dt
 *
 * ### Rotational (Euler's equations in body frame)
 *   τ_total  = τ_thrust + τ_gyro (gyroscopic precession)
 *   J · ω̇   = τ_total − ω × (J · ω)         (Euler's rotation eq.)
 *   q̇        = ½ q ⊗ [0, ω]                   (quaternion kinematics)
 *
 * ### Aerodynamics
 * Drag force uses the standard quadratic model:
 *   F_drag = −½ ρ(h) · Cd · A · |v_rel|² · v̂_rel
 * where v_rel = v_body − v_wind and ρ(h) is computed from the ISA model.
 *
 * ### Motor Forces & Torques (Quadrotor / generic multi-rotor)
 * Each rotor contributes:
 *   - Thrust along its spin axis (body +Z up for positive thrust → NED −Z)
 *   - Reaction torque about its spin axis proportional to thrust²
 *
 * ### Ground Model
 * A simple elastic ground plane at z = 0 (NED): if the vehicle penetrates
 * the ground a restitution force brings it back.  A landing gear damping
 * coefficient can be tuned via config.
 *
 * ## Integration
 * 4th-order Runge-Kutta (RK4) integration with a fixed sub-step size.
 * This ensures numerical stability for high motor response rates.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include "Math/Vector.h"
#include "Utilities/Config.h"
#include <vector>

namespace AeroCore {
namespace Physics {

// ============================================================
//  Data structures
// ============================================================

/**
 * @brief Describes a single rotor/motor in the vehicle's body frame.
 *
 * Rotors can be mounted anywhere (quadrotor X/+ frame, hex, octo, fixed-wing
 * pusher/puller prop, etc.).  Positive spin direction is CCW when viewed from
 * the thrust direction.
 */
struct RotorConfig {
    Math::Vector3d position;       ///< Position in body frame [m]
    Math::Vector3d spin_axis;      ///< Unit thrust axis in body frame (normalised)
    double torque_coefficient;     ///< Reaction torque per unit thrust [m] (propeller radius proxy)
    int    spin_direction;         ///< +1 = CCW (top view), -1 = CW (generates opposite yaw torque)
};

/**
 * @brief Complete rigid-body state vector passed in / out of the physics update.
 *
 * Using a flat structure makes it straightforward to pass to RK4 derivative
 * evaluation without scattering state across objects.
 */
struct RigidBodyState {
    Math::Vector3d   position;      ///< Position in NED world frame [m]
    Math::Vector3d   velocity;      ///< Velocity in NED world frame [m/s]
    Math::Quaterniond orientation;  ///< Body-to-world orientation quaternion
    Math::Vector3d   angular_vel;   ///< Angular velocity in body frame [rad/s] (p, q, r)
    Math::Vector3d   acceleration;  ///< Linear acceleration, NED world frame [m/s²] (derived)
    Math::Vector3d   angular_acc;   ///< Angular acceleration, body frame [rad/s²] (derived)

    RigidBodyState()
        : position(Math::Vector3d::Zero()),
          velocity(Math::Vector3d::Zero()),
          orientation(Math::Quaterniond::Identity()),
          angular_vel(Math::Vector3d::Zero()),
          acceleration(Math::Vector3d::Zero()),
          angular_acc(Math::Vector3d::Zero()) {}
};

/**
 * @brief External forces/torques applied to the vehicle for one physics tick.
 *
 * The physics engine accepts this structure so it remains independent of
 * vehicle type.  The FlightController or vehicle model fills it before
 * calling PhysicsEngine::step().
 */
struct ExternalForces {
    Math::Vector3d thrust_world;    ///< Net thrust force in world (NED) frame [N]
    Math::Vector3d torque_body;     ///< Net torque in body frame [N·m]
    Math::Vector3d wind_world;      ///< Ambient wind velocity in world frame [m/s]

    ExternalForces()
        : thrust_world(Math::Vector3d::Zero()),
          torque_body(Math::Vector3d::Zero()),
          wind_world(Math::Vector3d::Zero()) {}
};

// ============================================================
//  PhysicsEngine class
// ============================================================

/**
 * @brief 6-DOF rigid-body physics engine.
 *
 * ### Usage
 * ```cpp
 * PhysicsEngine engine(config);
 * engine.setInertiaTensor(diag(Ixx, Iyy, Izz));
 *
 * RigidBodyState state;     // zero-initialised
 * ExternalForces forces;
 * forces.thrust_world = ...;
 * forces.torque_body  = ...;
 * forces.wind_world   = wind;
 *
 * engine.step(state, forces, mass, drag_coeff, ref_area, dt);
 * // state now holds t+dt values
 * ```
 */
class PhysicsEngine {
public:
    /**
     * @brief Construct with simulation config.
     *
     * Reads from config sections:
     *   [physics]  gravity, air_density (baseline), drag_coefficient, ground_restitution
     */
    explicit PhysicsEngine(const Utilities::Config& config);

    // ----------------------------------------------------------
    //  Main integration step
    // ----------------------------------------------------------

    /**
     * @brief Advance the rigid-body state by @p dt seconds using RK4.
     *
     * Ground collision is enforced after integration.  If position.z() > 0
     * (below ground in NED) the vehicle is pushed back up and vertical
     * velocity is damped.
     *
     * @param[in,out] state      Rigid-body state to update.
     * @param[in]     forces     External forces/torques for this tick.
     * @param[in]     mass       Vehicle mass [kg].
     * @param[in]     drag_coeff Dimensionless drag coefficient Cd.
     * @param[in]     ref_area   Reference cross-section area A [m²].
     * @param[in]     dt         Time step [s].
     */
    void step(RigidBodyState& state,
              const ExternalForces& forces,
              double mass,
              double drag_coeff,
              double ref_area,
              double dt);

    // ----------------------------------------------------------
    //  Configuration setters / getters
    // ----------------------------------------------------------

    void setGravity(double gravity);          ///< Override gravity [m/s²]
    void setBaseDensity(double density);      ///< Override sea-level air density [kg/m³]
    void setInertiaTensor(const Math::Matrix3d& J); ///< Set inertia tensor [kg·m²]
    void setGroundRestitution(double e);      ///< Coefficient of restitution [0–1]
    void setGroundFriction(double mu);        ///< Ground friction coefficient

    double          getGravity()      const;
    double          getBaseDensity()  const;
    Math::Matrix3d  getInertiaTensor() const;
    double          getGroundRestitution() const;

    /**
     * @brief Compute air density at a given altitude using ISA model.
     * @param altitude_m  Altitude above MSL [m] (positive up).
     */
    double airDensityAtAltitude(double altitude_m) const;

private:
    double         gravity_;          ///< Gravitational acceleration [m/s²]
    double         base_density_;     ///< Sea-level air density [kg/m³]
    Math::Matrix3d inertia_;          ///< Inertia tensor J [kg·m²]
    Math::Matrix3d inertia_inv_;      ///< Pre-computed J⁻¹
    double         ground_restitution_; ///< Bounce coefficient [0=no bounce, 1=elastic]
    double         ground_friction_;  ///< Coulomb friction coefficient

    // ----------------------------------------------------------
    //  RK4 derivative evaluation
    // ----------------------------------------------------------

    /**
     * @brief Compute the time derivative of the state vector.
     *
     * Returns (ṗ, v̇, q̇, ω̇) given current state and applied forces.
     */
    struct StateDerivative {
        Math::Vector3d dp;     ///< ṗ = v
        Math::Vector3d dv;     ///< v̇ = F/m
        Math::Quaterniond dq;  ///< q̇ = ½ q ⊗ Ω
        Math::Vector3d dw;     ///< ω̇ = J⁻¹(τ − ω×Jω)
    };

    StateDerivative computeDerivative(const RigidBodyState& state,
                                      const ExternalForces& forces,
                                      double mass,
                                      double drag_coeff,
                                      double ref_area) const;

    /// Apply one RK4 sub-step of size @p h.
    RigidBodyState rk4Step(const RigidBodyState& s,
                           const ExternalForces& f,
                           double mass,
                           double drag_coeff,
                           double ref_area,
                           double h) const;

    // ----------------------------------------------------------
    //  Individual force/torque calculations
    // ----------------------------------------------------------

    /// Gravity force in NED world frame [N].
    Math::Vector3d gravityForce(double mass, double altitude_m) const;

    /**
     * @brief Aerodynamic drag force in world frame [N].
     *
     * Uses relative velocity w.r.t. wind to model effective drag.
     */
    Math::Vector3d dragForce(const Math::Vector3d& velocity_world,
                             const Math::Vector3d& wind_world,
                             double drag_coeff,
                             double ref_area,
                             double altitude_m) const;

    /**
     * @brief Gyroscopic cross-coupling torque: −ω × (J·ω).
     *
     * This is the term that makes quadrotors yaw when they pitch/roll.
     */
    Math::Vector3d gyroscopicTorque(const Math::Vector3d& angular_vel) const;

    /// Enforce ground collision / landing.
    void enforceGround(RigidBodyState& state) const;
};

} // namespace Physics
} // namespace AeroCore
