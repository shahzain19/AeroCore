/**
 * @file PhysicsEngine.cpp
 * @brief 6-DOF rigid-body physics engine implementation.
 *
 * ## Numerical Integration
 * Uses 4th-order Runge-Kutta (RK4).  The state derivative is computed from:
 *
 *   ṗ = v                              (position rate = velocity)
 *   v̇ = (F_gravity + F_drag + F_thrust) / m   (translational dynamics)
 *   q̇ = 0.5 · q ⊗ [0, ω]             (quaternion kinematics)
 *   ω̇ = J⁻¹ · (τ − ω × J·ω)         (Euler's rotation equation)
 *
 * The RK4 evaluates the derivative at k₁, k₂, k₃, k₄ and blends them:
 *   y_{n+1} = y_n + (h/6) · (k₁ + 2k₂ + 2k₃ + k₄)
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Physics/PhysicsEngine.h"
#include "Math/Vector.h"
#include <cmath>
#include <stdexcept>

namespace AeroCore {
namespace Physics {

// ============================================================
//  Construction
// ============================================================

PhysicsEngine::PhysicsEngine(const Utilities::Config& config)
    : gravity_(config.get<double>("physics", "gravity"))
    , base_density_(config.get<double>("physics", "air_density"))
    , ground_restitution_(0.1)   // Low bounce — soft landing behaviour
    , ground_friction_(0.5)
{
    // Default inertia for a 250-gram micro quad, 100 mm arm length
    // Ixx = Iyy = 2·m·l²/5 (point masses at motor tips)
    // Izz ≈ 2×Ixx (flat disc approximation)
    // These are overridden by Drone::getInertiaTensor().
    const double m  = 0.25;     // kg
    const double l  = 0.1;      // m
    const double Ixy = 2.0 * m * l * l;
    const double Iz  = 2.0 * Ixy;
    inertia_ = Math::Matrix3d::Zero();
    inertia_(0,0) = Ixy;
    inertia_(1,1) = Ixy;
    inertia_(2,2) = Iz;
    inertia_inv_  = inertia_.inverse();

    // Try to load optional config overrides
    try {
        double e = config.get<double>("physics", "ground_restitution");
        ground_restitution_ = e;
    } catch (...) {}
    try {
        double mu = config.get<double>("physics", "ground_friction");
        ground_friction_ = mu;
    } catch (...) {}
}

// ============================================================
//  Configuration setters
// ============================================================

void PhysicsEngine::setGravity(double gravity)    { gravity_      = gravity; }
void PhysicsEngine::setBaseDensity(double density) { base_density_ = density; }
void PhysicsEngine::setGroundRestitution(double e) { ground_restitution_ = e; }
void PhysicsEngine::setGroundFriction(double mu)   { ground_friction_ = mu; }

void PhysicsEngine::setInertiaTensor(const Math::Matrix3d& J) {
    if (J.determinant() < 1e-12) {
        throw std::runtime_error("PhysicsEngine: degenerate inertia tensor");
    }
    inertia_     = J;
    inertia_inv_ = J.inverse();
}

// ============================================================
//  Configuration getters
// ============================================================

double          PhysicsEngine::getGravity()           const { return gravity_; }
double          PhysicsEngine::getBaseDensity()       const { return base_density_; }
Math::Matrix3d  PhysicsEngine::getInertiaTensor()     const { return inertia_; }
double          PhysicsEngine::getGroundRestitution() const { return ground_restitution_; }

// ============================================================
//  ISA air density model
// ============================================================

double PhysicsEngine::airDensityAtAltitude(double altitude_m) const {
    // Scale base density by the ISA model ratio at this altitude.
    // ISA density ratio: ρ/ρ₀ = (T/T₀)^(g·M/(R·L) - 1)
    const double T0 = Math::ISA_TEMP_SEA_LEVEL;
    const double L  = Math::ISA_LAPSE_RATE;
    const double T  = T0 - L * std::max(0.0, altitude_m);
    const double exp = (gravity_ * Math::MOLAR_MASS_AIR) /
                       (Math::GAS_CONSTANT * L) - 1.0;
    const double ratio = std::pow(T / T0, exp);
    // Scale from our configured base density (not necessarily ISA standard)
    return base_density_ * ratio;
}

// ============================================================
//  Individual force calculations
// ============================================================

Math::Vector3d PhysicsEngine::gravityForce(double mass, double altitude_m) const {
    // In NED: gravity acts in the +Z direction (downward).
    // Use altitude-varying gravity for accuracy.
    const double g = Math::gravityAtAltitude(altitude_m);
    return Math::Vector3d(0.0, 0.0, mass * g);
}

Math::Vector3d PhysicsEngine::dragForce(const Math::Vector3d& velocity_world,
                                        const Math::Vector3d& wind_world,
                                        double drag_coeff,
                                        double ref_area,
                                        double altitude_m) const {
    // Relative velocity of vehicle w.r.t. air mass
    const Math::Vector3d v_rel = velocity_world - wind_world;
    const double speed = v_rel.norm();

    if (speed < 1e-9) {
        return Math::Vector3d::Zero();
    }

    // Dynamic pressure: q = 0.5 · ρ · v²
    const double rho = airDensityAtAltitude(altitude_m);
    const double dynamic_pressure = 0.5 * rho * speed * speed;

    // Drag force magnitude: Fd = Cd · A · q
    const double magnitude = drag_coeff * ref_area * dynamic_pressure;

    // Direction: opposes relative velocity
    return -v_rel.normalized() * magnitude;
}

Math::Vector3d PhysicsEngine::gyroscopicTorque(const Math::Vector3d& angular_vel) const {
    // τ_gyro = −ω × (J·ω)
    // This is the coupling term in Euler's rotation equation that causes
    // quadrotors to yaw when they pitch/roll.
    const Math::Vector3d J_omega = inertia_ * angular_vel;
    return -(angular_vel.cross(J_omega));
}

// ============================================================
//  State derivative computation (for RK4)
// ============================================================

PhysicsEngine::StateDerivative PhysicsEngine::computeDerivative(
        const RigidBodyState&  state,
        const ExternalForces&  forces,
        double mass,
        double drag_coeff,
        double ref_area) const {

    StateDerivative deriv;

    // --- Altitude above ground (NED: positive z is down) ---
    // altitude = -position.z()
    const double altitude_m = std::max(0.0, -state.position.z());

    // === Translational dynamics ===
    // F_total = F_thrust + F_gravity + F_drag
    const Math::Vector3d F_grav = gravityForce(mass, altitude_m);
    const Math::Vector3d F_drag = dragForce(state.velocity, forces.wind_world,
                                            drag_coeff, ref_area, altitude_m);
    const Math::Vector3d F_total = forces.thrust_world + F_grav + F_drag;

    // ṗ = v
    deriv.dp = state.velocity;

    // v̇ = F/m
    deriv.dv = F_total / mass;

    // === Rotational dynamics ===
    // Quaternion kinematics: q̇ = 0.5 · q ⊗ [0, ω]
    // Note: Eigen quaternion coefficient order is (w, x, y, z)
    const Math::Vector3d& omega = state.angular_vel;
    deriv.dq = Math::Quaterniond(
        (state.orientation * Math::Quaterniond(0.0, omega.x(), omega.y(), omega.z())).coeffs() * 0.5
    );

    // Euler's rotation equation: J·ω̇ = τ_total − ω × J·ω
    const Math::Vector3d tau_total = forces.torque_body + gyroscopicTorque(omega);
    deriv.dw = inertia_inv_ * tau_total;

    return deriv;
}

// ============================================================
//  RK4 sub-step
// ============================================================

RigidBodyState PhysicsEngine::rk4Step(const RigidBodyState& s,
                                       const ExternalForces& f,
                                       double mass,
                                       double drag_coeff,
                                       double ref_area,
                                       double h) const {
    // k1: derivative at start of step
    auto d1 = computeDerivative(s, f, mass, drag_coeff, ref_area);

    // k2: derivative at h/2 using k1
    RigidBodyState s2 = s;
    s2.position    = s.position    + d1.dp * (h * 0.5);
    s2.velocity    = s.velocity    + d1.dv * (h * 0.5);
    s2.orientation = Math::Quaterniond(
        (s.orientation.coeffs() + d1.dq.coeffs() * (h * 0.5)).normalized()
    );
    s2.angular_vel = s.angular_vel + d1.dw * (h * 0.5);
    auto d2 = computeDerivative(s2, f, mass, drag_coeff, ref_area);

    // k3: derivative at h/2 using k2
    RigidBodyState s3 = s;
    s3.position    = s.position    + d2.dp * (h * 0.5);
    s3.velocity    = s.velocity    + d2.dv * (h * 0.5);
    s3.orientation = Math::Quaterniond(
        (s.orientation.coeffs() + d2.dq.coeffs() * (h * 0.5)).normalized()
    );
    s3.angular_vel = s.angular_vel + d2.dw * (h * 0.5);
    auto d3 = computeDerivative(s3, f, mass, drag_coeff, ref_area);

    // k4: derivative at h using k3
    RigidBodyState s4 = s;
    s4.position    = s.position    + d3.dp * h;
    s4.velocity    = s.velocity    + d3.dv * h;
    s4.orientation = Math::Quaterniond(
        (s.orientation.coeffs() + d3.dq.coeffs() * h).normalized()
    );
    s4.angular_vel = s.angular_vel + d3.dw * h;
    auto d4 = computeDerivative(s4, f, mass, drag_coeff, ref_area);

    // Blend: y_{n+1} = y_n + (h/6)(k1 + 2k2 + 2k3 + k4)
    const double inv6 = h / 6.0;
    RigidBodyState result = s;
    result.position    = s.position    + (d1.dp + d2.dp * 2.0 + d3.dp * 2.0 + d4.dp) * inv6;
    result.velocity    = s.velocity    + (d1.dv + d2.dv * 2.0 + d3.dv * 2.0 + d4.dv) * inv6;
    result.angular_vel = s.angular_vel + (d1.dw + d2.dw * 2.0 + d3.dw * 2.0 + d4.dw) * inv6;

    // Quaternion blend and re-normalise
    result.orientation = Math::Quaterniond(
        (s.orientation.coeffs() +
         (d1.dq.coeffs() + d2.dq.coeffs() * 2.0 + d3.dq.coeffs() * 2.0 + d4.dq.coeffs()) * inv6
        ).normalized()
    );

    // Cache the acceleration (used by sensors/telemetry)
    result.acceleration = (d1.dv + d2.dv * 2.0 + d3.dv * 2.0 + d4.dv) * (1.0 / 6.0);
    result.angular_acc  = (d1.dw + d2.dw * 2.0 + d3.dw * 2.0 + d4.dw) * (1.0 / 6.0);

    return result;
}

// ============================================================
//  Ground collision enforcement
// ============================================================

void PhysicsEngine::enforceGround(RigidBodyState& state) const {
    // In NED, ground is at z = 0 (positive z = downward).
    // Vehicle is below ground when position.z() > 0.
    if (state.position.z() > 0.0) {
        state.position.z() = 0.0;

        // Vertical velocity component (NED +z = downward)
        if (state.velocity.z() > 0.0) {
            // Reflect with restitution coefficient
            state.velocity.z() = -state.velocity.z() * ground_restitution_;

            // Apply lateral friction: reduce horizontal velocity
            const double v_xy = std::sqrt(state.velocity.x() * state.velocity.x() +
                                          state.velocity.y() * state.velocity.y());
            if (v_xy > 1e-6) {
                const double friction_impulse = ground_friction_ * std::abs(state.velocity.z());
                const double new_vxy = std::max(0.0, v_xy - friction_impulse);
                const double scale = new_vxy / v_xy;
                state.velocity.x() *= scale;
                state.velocity.y() *= scale;
            }
        }

        // Damp angular velocity on ground contact (landing gear absorbs rotation)
        state.angular_vel *= 0.3;
    }
}

// ============================================================
//  Main integration step
// ============================================================

void PhysicsEngine::step(RigidBodyState& state,
                          const ExternalForces& forces,
                          double mass,
                          double drag_coeff,
                          double ref_area,
                          double dt) {
    if (dt <= 0.0) return;
    if (mass < 1e-6) return;

    // RK4 integration
    state = rk4Step(state, forces, mass, drag_coeff, ref_area, dt);

    // Re-normalise quaternion (floating-point drift accumulates without this)
    state.orientation.normalize();

    // Clamp tiny velocities to zero (prevents infinite drift at rest)
    if (state.velocity.norm() < 1e-6 && state.position.z() >= -0.001) {
        state.velocity.setZero();
    }
    if (state.angular_vel.norm() < 1e-6) {
        state.angular_vel.setZero();
    }

    // Ground collision
    enforceGround(state);
}

} // namespace Physics
} // namespace AeroCore
