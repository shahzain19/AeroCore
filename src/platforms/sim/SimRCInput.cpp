/**
 * @file SimRCInput.cpp
 */

#include "platforms/sim/SimRCInput.h"
#include "Math/Vector.h"

namespace AeroCore {
namespace Platform {
namespace Sim {

SimRCInput::SimRCInput() {
    channels_.channel_count = 5;
    channels_.link_status   = HAL::RCLinkStatus::Connected;
    channels_.valid         = true;
}

void SimRCInput::setConnected(bool connected) {
    connected_ = connected;
    channels_.link_status = connected ? HAL::RCLinkStatus::Connected
                                      : HAL::RCLinkStatus::Lost;
    channels_.valid = connected;
}

void SimRCInput::setThrottle(double t) {
    channels_.channels[2] = Math::clamp(t, 0.0, 1.0);
}

void SimRCInput::setRoll(double v)  { channels_.channels[0] = Math::clamp(v, -1.0, 1.0); }
void SimRCInput::setPitch(double v) { channels_.channels[1] = Math::clamp(v, -1.0, 1.0); }
void SimRCInput::setYaw(double v)   { channels_.channels[3] = Math::clamp(v, -1.0, 1.0); }

void SimRCInput::setArmSwitch(bool armed) {
    channels_.channels[4] = armed ? 1.0 : 0.0;
}

void SimRCInput::setFromPilot(const Flight::PilotInput& pilot, bool arm_switch_high) {
    setRoll(pilot.roll);
    setPitch(pilot.pitch);
    setYaw(pilot.yaw);
    setThrottle(pilot.throttle);
    setArmSwitch(arm_switch_high);
}

HAL::RCChannels SimRCInput::read() const {
    return channels_;
}

uint32_t SimRCInput::msSinceLastFrame() const {
    return connected_ ? 0 : 5000;
}

bool SimRCInput::healthy() const {
    return connected_;
}

Flight::PilotInput SimRCInput::toPilotInput() const {
    Flight::PilotInput p{};
    p.roll     = channels_.channels[0];
    p.pitch    = channels_.channels[1];
    p.throttle = channels_.channels[2];
    p.yaw      = channels_.channels[3];
    return p;
}

} // namespace Sim
} // namespace Platform
} // namespace AeroCore
