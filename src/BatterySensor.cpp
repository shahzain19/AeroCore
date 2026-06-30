/**
 * @file BatterySensor.cpp
 * @brief Battery voltage sensor implementation.
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Sensors/BatterySensor.h"

namespace AeroCore {
namespace Sensors {

BatterySensor::BatterySensor(double update_rate, double noise_stddev, double drift_rate)
    : Sensor(update_rate, noise_stddev, drift_rate)
    , voltage_(0.0)
{}

void BatterySensor::update(double dt, double true_voltage) {
    Sensor::update(dt, true_voltage);
    voltage_ = current_value_;
    // Voltage cannot be negative
    if (voltage_ < 0.0) voltage_ = 0.0;
}

double BatterySensor::getVoltage() const {
    return voltage_;
}

} // namespace Sensors
} // namespace AeroCore
