/**
 * @file IIMU.h
 * @brief Hardware abstraction for 6-DOF inertial sensors.
 */

#pragma once

#include "HAL/Types.h"

namespace AeroCore {
namespace HAL {

class IIMU {
public:
    virtual ~IIMU() = default;

    /// Poll or return latest DMA-buffered sample. Must be non-blocking.
    virtual IMUSample read() const = 0;

    /// True when sensor self-test / WHO_AM_I passed at init.
    virtual bool healthy() const = 0;

    /// Optional: start gyro bias calibration (vehicle must be stationary).
    virtual void startCalibration() {}
    virtual bool calibrationComplete() const { return true; }
};

} // namespace HAL
} // namespace AeroCore
