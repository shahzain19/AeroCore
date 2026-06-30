/**
 * @file IMU.cpp
 * @brief IMU (accelerometer + gyroscope + complementary filter) implementation.
 *
 * ## Complementary Filter Details
 *
 * The complementary filter fuses gyroscope integration (fast, no drift in short
 * term, but drifts over time) with accelerometer tilt estimation (noisy, but
 * long-term stable) to produce a better attitude estimate than either alone.
 *
 *   φ̂[k] = α · (φ̂[k−1] + ω_x·dt) + (1−α) · atan2(a_y, a_z)
 *   θ̂[k] = α · (θ̂[k−1] + ω_y·dt) + (1−α) · atan2(−a_x, √(a_y²+a_z²))
 *
 * The filter relies on the accelerometer measuring mostly gravity in steady
 * state (which is a good assumption in low-g flight conditions).
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Sensors/IMU.h"
#include <cmath>

namespace AeroCore {
namespace Sensors {

IMU::IMU(const Utilities::Config& config)
    : comp_alpha_(0.98)
    , estimated_roll_(0.0)
    , estimated_pitch_(0.0)
{
    // Read IMU parameters from config, or use sensible defaults for a
    // mid-grade MEMS IMU (e.g. ICM-42688 or MPU-6050 quality)
    auto tryGet = [&](const std::string& key, double fb) -> double {
        try { return config.get<double>("imu", key); }
        catch (...) { return fb; }
    };

    const double accel_rate    = tryGet("accel_rate_hz",     1000.0);
    const double accel_noise   = tryGet("accel_noise",          0.05);  // m/s²
    const double accel_drift   = tryGet("accel_drift",         0.001);  // m/s²/s

    const double gyro_rate     = tryGet("gyro_rate_hz",      1000.0);
    const double gyro_noise    = tryGet("gyro_noise",        0.0017);  // rad/s (~0.1 deg/s)
    const double gyro_drift    = tryGet("gyro_drift",        0.0001);  // rad/s/s

    comp_alpha_   = tryGet("comp_filter_alpha", 0.98);

    accelerometer_ = std::make_unique<Accelerometer>(accel_rate, accel_noise, accel_drift);
    gyroscope_     = std::make_unique<Gyroscope>(gyro_rate, gyro_noise, gyro_drift);
}

void IMU::update(double dt,
                 const Math::Vector3d& true_accel_body,
                 const Math::Vector3d& true_omega_body,
                 const Math::Quaterniond& /*orientation*/)  // reserved for future EKF
{
    accelerometer_->update(dt, true_accel_body);
    gyroscope_->update(dt, true_omega_body);

    // Update complementary filter with the latest sensor readings
    updateComplementaryFilter(dt,
                              accelerometer_->getAcceleration(),
                              gyroscope_->getAngularVelocity());
}

void IMU::updateComplementaryFilter(double dt,
                                    const Math::Vector3d& accel,
                                    const Math::Vector3d& omega) {
    // --- Gyroscope integration ---
    const double roll_gyro  = estimated_roll_  + omega.x() * dt;
    const double pitch_gyro = estimated_pitch_ + omega.y() * dt;

    // --- Accelerometer tilt estimate ---
    // The accelerometer measures specific force.  In hover, most of this is
    // gravity.  We can estimate tilt from the direction of the g vector.
    const double ax = accel.x();
    const double ay = accel.y();
    const double az = accel.z();

    // Magnitude of the accelerometer reading — use for confidence weighting
    const double a_mag = std::sqrt(ax*ax + ay*ay + az*az);

    // Only use accelerometer if it's measuring something close to 1g
    // (between 0.5g and 2g).  Outside this range we're in high-g manoeuvre.
    const double g_est = Math::GRAVITY_MSL;
    double roll_accel  = 0.0;
    double pitch_accel = 0.0;
    double trust       = 0.0;

    if (a_mag > 0.5 * g_est && a_mag < 2.0 * g_est) {
        // atan2-based tilt from gravity direction
        roll_accel  = std::atan2(ay, az);
        pitch_accel = std::atan2(-ax, std::sqrt(ay*ay + az*az));
        trust = comp_alpha_;  // Use configured trust level
    } else {
        trust = 1.0;  // High-g: trust gyro fully
    }

    // --- Complementary blend ---
    estimated_roll_  = trust * roll_gyro  + (1.0 - trust) * roll_accel;
    estimated_pitch_ = trust * pitch_gyro + (1.0 - trust) * pitch_accel;

    // Wrap angles to (−π, π]
    estimated_roll_  = Math::wrapAngle(estimated_roll_);
    estimated_pitch_ = Math::wrapAngle(estimated_pitch_);
}

// ============================================================
//  Accessors
// ============================================================

Accelerometer&       IMU::getAccelerometer()       { return *accelerometer_; }
Gyroscope&           IMU::getGyroscope()           { return *gyroscope_; }
const Accelerometer& IMU::getAccelerometer() const { return *accelerometer_; }
const Gyroscope&     IMU::getGyroscope()     const { return *gyroscope_; }

double IMU::getEstimatedRoll()  const { return estimated_roll_;  }
double IMU::getEstimatedPitch() const { return estimated_pitch_; }

void IMU::setComplementaryAlpha(double alpha) {
    comp_alpha_ = Math::clamp(alpha, 0.0, 1.0);
}

} // namespace Sensors
} // namespace AeroCore
