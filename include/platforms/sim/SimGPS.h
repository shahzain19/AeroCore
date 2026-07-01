/**
 * @file SimGPS.h
 * @brief Simulation backend for GNSS position and velocity feedback.
 */

#pragma once

#include "HAL/IGPS.h"
#include "Flight/Drone.h"

#include <memory>

namespace AeroCore {
namespace Platform {
namespace Sim {

class SimGPS : public HAL::IGPS {
public:
    explicit SimGPS(std::shared_ptr<Flight::Drone> drone);

    HAL::GPSSample read() const override;
    bool healthy() const override;
    bool fixAcceptableForNavigation() const override;

    void setHealthy(bool healthy);
    void setNoiseMeters(double xy_noise_m, double z_noise_m);

private:
    std::shared_ptr<Flight::Drone> drone_;
    bool healthy_{true};
    double xy_noise_m_{0.2};
    double z_noise_m_{0.3};
};

} // namespace Sim
} // namespace Platform
} // namespace AeroCore
