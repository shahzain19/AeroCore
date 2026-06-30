/**
 * @file Altimeter.cpp
 * @brief Barometric altimeter implementation.
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Sensors/Altimeter.h"

namespace AeroCore {
namespace Sensors {

Altimeter::Altimeter(double update_rate, double noise_stddev, double drift_rate)
    : Sensor(update_rate, noise_stddev, drift_rate)
    , altitude_(0.0)
{}

void Altimeter::update(double dt, double true_altitude) {
    Sensor::update(dt, true_altitude);
    // The base class update() writes to current_value_
    altitude_ = current_value_;
    // Altitude cannot be negative
    if (altitude_ < 0.0) altitude_ = 0.0;
}

double Altimeter::getAltitude() const {
    return altitude_;
}

} // namespace Sensors
} // namespace AeroCore
