/**
 * @file Gyroscope.cpp
 * @brief 3-axis gyroscope implementation.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Sensors/Gyroscope.h"
#include <chrono>

namespace AeroCore {
namespace Sensors {

Gyroscope::Gyroscope(double update_rate, double noise_stddev, double drift_rate)
    : Sensor(update_rate, noise_stddev, drift_rate)
    , angular_velocity_(Math::Vector3d::Zero())
    , noise_x_(0.0, noise_stddev > 0.0 ? noise_stddev : 1e-12)
    , noise_y_(0.0, noise_stddev > 0.0 ? noise_stddev : 1e-12)
    , noise_z_(0.0, noise_stddev > 0.0 ? noise_stddev : 1e-12)
{}

void Gyroscope::update(double dt, const Math::Vector3d& true_omega_body) {
    time_since_update_ += dt;
    updateDrift(dt);

    const double interval = 1.0 / update_rate_;
    if (time_since_update_ >= interval) {
        angular_velocity_.x() = true_omega_body.x() + drift_ + noise_x_(rng_);
        angular_velocity_.y() = true_omega_body.y() + drift_ + noise_y_(rng_);
        angular_velocity_.z() = true_omega_body.z() + drift_ + noise_z_(rng_);

        time_since_update_ = 0.0;
        ready_             = true;
    } else {
        ready_ = false;
    }
}

Math::Vector3d Gyroscope::getAngularVelocity() const {
    return angular_velocity_;
}

} // namespace Sensors
} // namespace AeroCore
