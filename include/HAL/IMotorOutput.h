/**
 * @file IMotorOutput.h
 * @brief Hardware abstraction for ESC / motor output (PWM, DShot).
 */

#pragma once

#include <cstddef>

namespace AeroCore {
namespace HAL {

class IMotorOutput {
public:
    virtual ~IMotorOutput() = default;

    virtual size_t motorCount() const = 0;

    /// Write normalized throttle [0, 1] for motor @p index.
    virtual void write(size_t motor_index, double throttle_0_1) = 0;

    /// Disarm: force all outputs to minimum / DShot stop.
    virtual void disarmAll() = 0;

    /// Arm ESCs (protocol-specific; no-op for PWM).
    virtual void arm() {}
};

} // namespace HAL
} // namespace AeroCore
