/**
 * @file SimIMU.cpp
 */

#include "platforms/sim/SimIMU.h"

namespace AeroCore {
namespace Platform {
namespace Sim {

SimIMU::SimIMU(std::shared_ptr<Sensors::IMU> imu)
    : imu_(std::move(imu)) {}

HAL::IMUSample SimIMU::read() const {
    HAL::IMUSample s{};
    if (!imu_) return s;

    s.accel_body = imu_->getAccelerometer().getAcceleration();
    s.gyro_body  = imu_->getGyroscope().getAngularVelocity();
    s.valid      = true;
    return s;
}

bool SimIMU::healthy() const {
    return imu_ != nullptr;
}

} // namespace Sim
} // namespace Platform
} // namespace AeroCore
