/**
 * @file Sensor.cpp
 * @brief Base sensor class implementation.
 *
 * The sensor samples at the configured update_rate and applies Gaussian
 * noise plus a slowly drifting bias on each sample.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Sensors/Sensor.h"
#include <chrono>

namespace AeroCore {
namespace Sensors {

Sensor::Sensor(double update_rate, double noise_stddev, double drift_rate)
    : update_rate_(update_rate > 0.0 ? update_rate : 1.0)
    , noise_stddev_(noise_stddev)
    , drift_rate_(drift_rate)
    , current_value_(0.0)
    , drift_(0.0)
    , time_since_update_(0.0)
    , ready_(false)
    // Seed RNG from high-resolution clock to ensure each sensor has
    // independent noise (important when multiple sensors are created close
    // together in time).
    , rng_(static_cast<uint64_t>(
          std::chrono::steady_clock::now().time_since_epoch().count()))
    , noise_dist_(0.0, noise_stddev > 0.0 ? noise_stddev : 1e-12)
{}

void Sensor::update(double dt, double true_value) {
    time_since_update_ += dt;
    updateDrift(dt);

    const double interval = 1.0 / update_rate_;
    if (time_since_update_ >= interval) {
        // New sample available
        current_value_     = addNoise(true_value + drift_);
        time_since_update_ = 0.0;
        ready_             = true;
    } else {
        ready_ = false;
    }
}

double Sensor::getValue()      const { return current_value_; }
bool   Sensor::isReady()       const { return ready_; }
double Sensor::getUpdateRate() const { return update_rate_; }
double Sensor::getNoiseStddev() const { return noise_stddev_; }

void Sensor::setUpdateRate(double hz) {
    update_rate_ = (hz > 0.0) ? hz : 1.0;
}
void Sensor::setNoise(double stddev) {
    noise_stddev_ = stddev;
    noise_dist_   = std::normal_distribution<double>(0.0, stddev > 0.0 ? stddev : 1e-12);
}
void Sensor::setDrift(double rate)  { drift_rate_ = rate; }
void Sensor::resetDrift()           { drift_ = 0.0; }

double Sensor::addNoise(double value) {
    return value + noise_dist_(rng_);
}

void Sensor::updateDrift(double dt) {
    // Random-walk drift: incremented by a small Gaussian step each tick
    // (Wiener process model of gyro/accel bias)
    if (std::abs(drift_rate_) > 1e-15) {
        std::normal_distribution<double> drift_step(0.0, drift_rate_ * dt);
        drift_ += drift_step(rng_);
    }
}

} // namespace Sensors
} // namespace AeroCore
