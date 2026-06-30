/**
 * @file IRCInput.h
 * @brief Hardware abstraction for RC receiver input (CRSF, SBUS, PPM).
 */

#pragma once

#include "HAL/Types.h"

namespace AeroCore {
namespace HAL {

class IRCInput {
public:
    virtual ~IRCInput() = default;

    /// Latest decoded frame. Non-blocking.
    virtual RCChannels read() const = 0;

    /// Milliseconds since last valid frame.
    virtual uint32_t msSinceLastFrame() const = 0;

    virtual bool healthy() const = 0;
};

} // namespace HAL
} // namespace AeroCore
