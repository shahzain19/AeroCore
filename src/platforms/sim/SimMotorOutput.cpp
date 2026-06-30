/**
 * @file SimMotorOutput.cpp
 */

#include "platforms/sim/SimMotorOutput.h"

namespace AeroCore {
namespace Platform {
namespace Sim {

SimMotorOutput::SimMotorOutput(std::shared_ptr<Flight::Drone> drone)
    : drone_(std::move(drone)) {}

size_t SimMotorOutput::motorCount() const {
    return drone_ ? drone_->getMotorCount() : 0;
}

void SimMotorOutput::write(size_t motor_index, double throttle_0_1) {
    if (drone_) drone_->setMotorThrottle(motor_index, throttle_0_1);
}

void SimMotorOutput::disarmAll() {
    if (!drone_) return;
    for (size_t i = 0; i < drone_->getMotorCount(); ++i) {
        drone_->setMotorThrottle(i, 0.0);
    }
}

} // namespace Sim
} // namespace Platform
} // namespace AeroCore
