/**
 * @file SimBarometer.cpp
 */

#include "platforms/sim/SimBarometer.h"

namespace AeroCore {
namespace Platform {
namespace Sim {

SimBarometer::SimBarometer(std::shared_ptr<Sensors::Altimeter> altimeter)
    : altimeter_(std::move(altimeter)) {}

HAL::BaroSample SimBarometer::read() const {
    HAL::BaroSample s{};
    if (!altimeter_) return s;

    s.altitude_m = altimeter_->getAltitude();
    s.valid      = true;
    return s;
}

bool SimBarometer::healthy() const {
    return altimeter_ != nullptr;
}

} // namespace Sim
} // namespace Platform
} // namespace AeroCore
