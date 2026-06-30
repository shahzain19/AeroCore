/**
 * @file SimIMU.h
 * @brief HAL IMU backend wrapping Sensors::IMU (simulation).
 */

#pragma once

#include "HAL/IIMU.h"
#include "Sensors/IMU.h"
#include <memory>

namespace AeroCore {
namespace Platform {
namespace Sim {

class SimIMU : public HAL::IIMU {
public:
    explicit SimIMU(std::shared_ptr<Sensors::IMU> imu);

    HAL::IMUSample read() const override;
    bool healthy() const override;

private:
    std::shared_ptr<Sensors::IMU> imu_;
};

} // namespace Sim
} // namespace Platform
} // namespace AeroCore
