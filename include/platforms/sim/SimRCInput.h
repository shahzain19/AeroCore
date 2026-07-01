/**
 * @file SimRCInput.h
 * @brief HAL RC backend for keyboard / synthetic pilot input (simulation).
 */

#pragma once

#include "HAL/IRCInput.h"
#include "Flight/FlightController.h"

namespace AeroCore {
namespace Platform {
namespace Sim {

/**
 * @brief Synthetic RC receiver for desktop simulation.
 *
 * Channel layout (Betaflight-style):
 *   0 roll [-1,1], 1 pitch [-1,1], 2 throttle [0,1], 3 yaw [-1,1], 4 arm [0,1]
 */
class SimRCInput : public HAL::IRCInput {
public:
    SimRCInput();

    void setConnected(bool connected);
    void setThrottle(double throttle_0_1);
    void setRoll(double roll);
    void setPitch(double pitch);
    void setYaw(double yaw);
    void setArmSwitch(bool armed);
    void setFromPilot(const Flight::PilotInput& pilot, bool arm_switch_high);

    HAL::RCChannels read() const override;
    uint32_t msSinceLastFrame() const override;
    bool healthy() const override;

    Flight::PilotInput toPilotInput() const;

private:
    HAL::RCChannels channels_{};
    bool connected_{true};
};

} // namespace Sim
} // namespace Platform
} // namespace AeroCore
