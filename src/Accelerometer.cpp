/**
 * @file Accelerometer.cpp
 * @brief 3-axis accelerometer implementation.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Sensors/Accelerometer.h"
#include <chrono>

namespace AeroCore {
namespace Sensors {

Accelerometer::Accelerometer(double update_rate, double noise_stddev, double drift_rate)
    : Sensor(update_rate, noise_stddev, drift_rate)
    , acceleration_(Math::Vector3d::Zero())
    // Seed each axis noise with a different bit mix to ensure independence
    , noise_x_(0.0, noise_stddev > 0.0 ? noise_stddev : 1e-12)
    , noise_y_(0.0, noise_stddev > 0.0 ? noise_stddev : 1e-12)
    , noise_z_(0.0, noise_stddev > 0.0 ? noise_stddev : 1e-12)
{}

void Accelerometer::update(double dt, const Math::Vector3d& true_accel_body) {
    time_since_update_ += dt;
    updateDrift(dt);

    const double interval = 1.0 / update_rate_;
    if (time_since_update_ >= interval) {
        // Apply independent noise and shared drift to each axis.
        // Drift is common-mode (same bias on all axes) in this simplified model.
        acceleration_.x() = true_accel_body.x() + drift_ + noise_x_(rng_);
        acceleration_.y() = true_accel_body.y() + drift_ + noise_y_(rng_);
        acceleration_.z() = true_accel_body.z() + drift_ + noise_z_(rng_);

        time_since_update_ = 0.0;
        ready_             = true;
    } else {
        ready_ = false;
    }
}

Math::Vector3d Accelerometer::getAcceleration() const {
    return acceleration_;
}

} // namespace Sensors
} // namespace AeroCore
