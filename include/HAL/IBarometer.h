/**
 * @file IBarometer.h
 * @brief Hardware abstraction for barometric altitude sensors.
 */

#pragma once

#include "HAL/Types.h"

namespace AeroCore {
namespace HAL {

class IBarometer {
public:
    virtual ~IBarometer() = default;

    virtual BaroSample read() const = 0;
    virtual bool healthy() const = 0;

    /// Set ground reference for relative altitude (call when disarmed on pad).
    virtual void resetGroundReference() {}
};

} // namespace HAL
} // namespace AeroCore
