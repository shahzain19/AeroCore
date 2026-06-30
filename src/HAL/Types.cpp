/**
 * @file Types.cpp
 * @brief HAL type helpers.
 */

#include "HAL/Types.h"

namespace AeroCore {
namespace HAL {

bool RCChannels::armed_switch() const {
    // Convention: channel 4 (index 4) AUX1 high = armed. Platforms may remap.
    if (channel_count < 5) return false;
    return channels[4] > 0.5;
}

bool RCChannels::failsafe_active() const {
    return link_status == RCLinkStatus::Lost || !valid;
}

} // namespace HAL
} // namespace AeroCore
