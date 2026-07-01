#include "platforms/sim/SimGPS.h"

#include <cmath>

namespace AeroCore {
namespace Platform {
namespace Sim {

SimGPS::SimGPS(std::shared_ptr<Flight::Drone> drone)
    : drone_(std::move(drone)) {}

HAL::GPSSample SimGPS::read() const {
    HAL::GPSSample sample{};
    if (!healthy_ || !drone_) {
        sample.valid = false;
        sample.fix_type = HAL::GPSFixType::NoFix;
        return sample;
    }

    const auto pos = drone_->getPosition();
    const auto vel = drone_->getVelocity();
    sample.position_ned = Math::Vector3d(
        pos.x() + xy_noise_m_ * 0.5,
        pos.y() + xy_noise_m_ * 0.5,
        pos.z() + z_noise_m_ * 0.5);
    sample.velocity_ned = Math::Vector3d(
        vel.x() + xy_noise_m_ * 0.05,
        vel.y() + xy_noise_m_ * 0.05,
        vel.z() + z_noise_m_ * 0.05);
    sample.altitude_amsl_m = -sample.position_ned.z();
    sample.valid = true;
    sample.fix_type = HAL::GPSFixType::Fix3D;
    sample.satellites = 10;
    sample.hdop = 1.1;
    return sample;
}

bool SimGPS::healthy() const { return healthy_; }

bool SimGPS::fixAcceptableForNavigation() const {
    return healthy_ && drone_ != nullptr;
}

void SimGPS::setHealthy(bool healthy) { healthy_ = healthy; }

void SimGPS::setNoiseMeters(double xy_noise_m, double z_noise_m) {
    xy_noise_m_ = std::max(0.0, xy_noise_m);
    z_noise_m_  = std::max(0.0, z_noise_m);
}

} // namespace Sim
} // namespace Platform
} // namespace AeroCore
