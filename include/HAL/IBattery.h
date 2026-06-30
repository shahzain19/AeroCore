/**
 * @file IBattery.h
 * @brief Hardware abstraction for battery voltage / current sensing.
 */

#pragma once

#include "HAL/Types.h"

namespace AeroCore {
namespace HAL {

class IBattery {
public:
    virtual ~IBattery() = default;

    virtual BatterySample read() const = 0;
    virtual bool healthy() const = 0;

    /// State of charge estimate [0, 1] if coulomb counting available.
    virtual double stateOfCharge() const = 0;
};

} // namespace HAL
} // namespace AeroCore
