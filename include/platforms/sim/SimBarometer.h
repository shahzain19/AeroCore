/**
 * @file SimBarometer.h
 * @brief HAL barometer backend wrapping Sensors::Altimeter (simulation).
 */

#pragma once

#include "HAL/IBarometer.h"
#include "Sensors/Altimeter.h"
#include <memory>

namespace AeroCore {
namespace Platform {
namespace Sim {

class SimBarometer : public HAL::IBarometer {
public:
    explicit SimBarometer(std::shared_ptr<Sensors::Altimeter> altimeter);

    HAL::BaroSample read() const override;
    bool healthy() const override;

private:
    std::shared_ptr<Sensors::Altimeter> altimeter_;
};

} // namespace Sim
} // namespace Platform
} // namespace AeroCore
