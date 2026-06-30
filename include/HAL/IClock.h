/**
 * @file IClock.h
 * @brief Monotonic clock for deterministic control-loop timing.
 */

#pragma once

#include <cstdint>

namespace AeroCore {
namespace HAL {

class IClock {
public:
    virtual ~IClock() = default;

    virtual uint64_t micros() const = 0;
    virtual uint64_t millis() const = 0;

    /// Block until @p target_us (monotonic). Used by firmware rate loops.
    virtual void sleepUntilMicros(uint64_t target_us) = 0;
};

} // namespace HAL
} // namespace AeroCore
