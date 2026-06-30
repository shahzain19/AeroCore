/**
 * @file Vector.h
 * @brief Core mathematical types and utilities for AeroCore.
 *
 * This header provides the foundational math types used throughout AeroCore:
 *  - Eigen type aliases (Vector2d, Vector3d, Matrix3d, etc.)
 *  - Quaternion type and helpers
 *  - Common aerospace math utilities (clamp, lerp, wrap angle, etc.)
 *
 * AeroCore uses a **right-handed, NED (North-East-Down) coordinate system**:
 *   - +X = North (forward)
 *   - +Y = East  (right)
 *   - +Z = Down  (into ground)
 *
 * Altitude is therefore  alt = -position.z()
 *
 * All angles are in **radians** unless explicitly stated otherwise.
 * All angular rates are in **radians per second**.
 *
 * @author AeroCore Contributors
 * @license MIT
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>

namespace AeroCore {
namespace Math {

// ============================================================
//  Eigen type aliases
// ============================================================

/// 2-component double-precision vector (used for 2-D utilities).
using Vector2d = Eigen::Vector2d;

/// 3-component double-precision vector (position, velocity, force, torque, …).
using Vector3d = Eigen::Vector3d;

/// 4-component double-precision vector (quaternion raw storage / general).
using Vector4d = Eigen::Vector4d;

/// 3×3 double-precision matrix (inertia tensor, rotation matrix, …).
using Matrix3d = Eigen::Matrix3d;

/// 4×4 double-precision matrix.
using Matrix4d = Eigen::Matrix4d;

/// Double-precision unit quaternion for representing 3-D orientations.
using Quaterniond = Eigen::Quaterniond;

/// Angle-axis representation (convenient for constructing rotations).
using AngleAxisd = Eigen::AngleAxisd;

// ============================================================
//  Mathematical constants
// ============================================================

/// π
constexpr double PI = 3.14159265358979323846;

/// 2π
constexpr double TWO_PI = 2.0 * PI;

/// π/2
constexpr double HALF_PI = PI / 2.0;

/// Degrees → radians conversion factor.
constexpr double DEG2RAD = PI / 180.0;

/// Radians → degrees conversion factor.
constexpr double RAD2DEG = 180.0 / PI;

/// Standard acceleration due to gravity at sea level [m/s²].
constexpr double GRAVITY_MSL = 9.80665;

/// International Standard Atmosphere sea-level air density [kg/m³].
constexpr double ISA_AIR_DENSITY = 1.225;

/// International Standard Atmosphere sea-level temperature [K].
constexpr double ISA_TEMP_SEA_LEVEL = 288.15;

/// Temperature lapse rate (troposphere) [K/m].
constexpr double ISA_LAPSE_RATE = 0.0065;

/// Ideal gas constant [J/(mol·K)].
constexpr double GAS_CONSTANT = 8.31446;

/// Molar mass of dry air [kg/mol].
constexpr double MOLAR_MASS_AIR = 0.0289647;

// ============================================================
//  Scalar utilities
// ============================================================

/**
 * @brief Clamp a value to [lo, hi].
 * @tparam T  Numeric type.
 */
template<typename T>
inline T clamp(T value, T lo, T hi) {
    return std::max(lo, std::min(hi, value));
}

/**
 * @brief Linear interpolation between @p a and @p b.
 * @param t  Blend factor in [0, 1].
 */
template<typename T>
inline T lerp(T a, T b, double t) {
    return a + static_cast<T>((b - a) * t);
}

/**
 * @brief Wrap an angle in radians to (-π, π].
 */
inline double wrapAngle(double angle) {
    while (angle >  PI) angle -= TWO_PI;
    while (angle <= -PI) angle += TWO_PI;
    return angle;
}

/**
 * @brief Wrap an angle in degrees to (-180, 180].
 */
inline double wrapAngleDeg(double deg) {
    while (deg >  180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
}

/**
 * @brief Convert degrees to radians.
 */
inline constexpr double toRadians(double deg) { return deg * DEG2RAD; }

/**
 * @brief Convert radians to degrees.
 */
inline constexpr double toDegrees(double rad) { return rad * RAD2DEG; }

/**
 * @brief Sign function: returns -1, 0, or +1.
 */
template<typename T>
inline T sign(T v) {
    return (v > T(0)) ? T(1) : (v < T(0)) ? T(-1) : T(0);
}

// ============================================================
//  Aerospace atmosphere model
// ============================================================

/**
 * @brief Compute air density using the International Standard Atmosphere
 *        troposphere model.
 *
 * Valid up to ~11 000 m altitude.
 *
 * Formula:
 *   ρ(h) = ρ₀ · (T(h)/T₀)^((g·M)/(R·L) - 1)
 *   where T(h) = T₀ − L·h
 *
 * @param altitude_m  Altitude above mean sea level [m].
 * @return Air density [kg/m³].
 */
inline double isaAirDensity(double altitude_m) {
    if (altitude_m < 0.0) altitude_m = 0.0;
    const double T = ISA_TEMP_SEA_LEVEL - ISA_LAPSE_RATE * altitude_m;
    const double exponent = (GRAVITY_MSL * MOLAR_MASS_AIR) /
                            (GAS_CONSTANT * ISA_LAPSE_RATE) - 1.0;
    return ISA_AIR_DENSITY * std::pow(T / ISA_TEMP_SEA_LEVEL, exponent);
}

/**
 * @brief Compute the gravitational acceleration as a function of altitude.
 *
 * Uses the inverse-square law with Earth's mean radius r_e = 6 371 000 m.
 *
 * @param altitude_m  Altitude above mean sea level [m].
 * @return Gravitational acceleration [m/s²] (positive downward).
 */
inline double gravityAtAltitude(double altitude_m) {
    constexpr double R_EARTH = 6371000.0;
    if (altitude_m < 0.0) altitude_m = 0.0;
    const double r = R_EARTH + altitude_m;
    return GRAVITY_MSL * (R_EARTH / r) * (R_EARTH / r);
}

// ============================================================
//  Quaternion / rotation utilities
// ============================================================

/**
 * @brief Convert a unit quaternion to Euler angles (roll, pitch, yaw) in
 *        ZYX (aerospace) convention.
 *
 * Roll  = rotation about X  (φ)
 * Pitch = rotation about Y  (θ)
 * Yaw   = rotation about Z  (ψ)
 *
 * @param q  Unit quaternion (w, x, y, z).
 * @return   Vector3d(roll, pitch, yaw) in radians.
 */
inline Vector3d quaternionToEuler(const Quaterniond& q) {
    // ZYX order: ψ, θ, φ
    const double w = q.w(), x = q.x(), y = q.y(), z = q.z();

    // Roll (φ) — rotation about X
    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    const double roll  = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (θ) — rotation about Y, clamped for gimbal lock
    const double sinp = 2.0 * (w * y - z * x);
    const double pitch = (std::abs(sinp) >= 1.0)
                         ? std::copysign(HALF_PI, sinp)
                         : std::asin(sinp);

    // Yaw (ψ) — rotation about Z
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    const double yaw   = std::atan2(siny_cosp, cosy_cosp);

    return Vector3d(roll, pitch, yaw);
}

/**
 * @brief Build a unit quaternion from Euler ZYX angles.
 *
 * @param roll   φ [rad]
 * @param pitch  θ [rad]
 * @param yaw    ψ [rad]
 * @return Normalised quaternion.
 */
inline Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
    return Quaterniond(
        AngleAxisd(yaw,   Vector3d::UnitZ()) *
        AngleAxisd(pitch, Vector3d::UnitY()) *
        AngleAxisd(roll,  Vector3d::UnitX())
    ).normalized();
}

/**
 * @brief Integrate a quaternion one step forward using body-frame angular velocity.
 *
 * Uses the first-order (Euler) quaternion kinematic equation:
 *   q̇ = 0.5 · q ⊗ [0, ω]
 *
 * @param q      Current orientation quaternion.
 * @param omega  Angular velocity in body frame [rad/s] (p, q, r).
 * @param dt     Time step [s].
 * @return New normalised quaternion.
 */
inline Quaterniond integrateQuaternion(const Quaterniond& q,
                                       const Vector3d& omega,
                                       double dt) {
    // Quaternion rate: q_dot = 0.5 * q * [0, omega]
    const Quaterniond omega_q(0.0, omega.x(), omega.y(), omega.z());
    const Quaterniond q_dot = Quaterniond(
        (q * omega_q).coeffs() * 0.5
    );
    return Quaterniond(
        (q.coeffs() + q_dot.coeffs() * dt).normalized()
    );
}

/**
 * @brief Rotate a 3-D vector from body frame to world (NED) frame.
 *
 * @param q  Orientation quaternion (body→world).
 * @param v  Vector in body frame.
 * @return   Vector in world frame.
 */
inline Vector3d bodyToWorld(const Quaterniond& q, const Vector3d& v) {
    return q * v;
}

/**
 * @brief Rotate a 3-D vector from world (NED) frame to body frame.
 *
 * @param q  Orientation quaternion (body→world).
 * @param v  Vector in world frame.
 * @return   Vector in body frame.
 */
inline Vector3d worldToBody(const Quaterniond& q, const Vector3d& v) {
    return q.inverse() * v;
}

// ============================================================
//  Vector utilities
// ============================================================

/**
 * @brief Safe normalization — returns a zero vector if norm < epsilon.
 */
inline Vector3d safeNormalize(const Vector3d& v, double epsilon = 1e-9) {
    const double n = v.norm();
    if (n > epsilon) return Vector3d(v / n);
    return Vector3d::Zero();
}

/**
 * @brief Saturate a 3-D vector's magnitude to @p max_norm.
 */
inline Vector3d saturate(const Vector3d& v, double max_norm) {
    const double n = v.norm();
    return (n > max_norm) ? (v * (max_norm / n)) : v;
}

} // namespace Math
} // namespace AeroCore
