/**
 * @file SimMotorOutput.h
 * @brief HAL motor backend — forwards throttle to Flight::Drone (simulation).
 */

#pragma once

#include "HAL/IMotorOutput.h"
#include "Flight/Drone.h"
#include <memory>

namespace AeroCore {
namespace Platform {
namespace Sim {

class SimMotorOutput : public HAL::IMotorOutput {
public:
    explicit SimMotorOutput(std::shared_ptr<Flight::Drone> drone);

    size_t motorCount() const override;
    void write(size_t motor_index, double throttle_0_1) override;
    void disarmAll() override;

private:
    std::shared_ptr<Flight::Drone> drone_;
};

} // namespace Sim
} // namespace Platform
} // namespace AeroCore
