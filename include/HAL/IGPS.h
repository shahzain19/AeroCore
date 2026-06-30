/**
 * @file IGPS.h
 * @brief Hardware abstraction for GNSS receivers.
 */

#pragma once

#include "HAL/Types.h"

namespace AeroCore {
namespace HAL {

class IGPS {
public:
    virtual ~IGPS() = default;

    virtual GPSSample read() const = 0;
    virtual bool healthy() const = 0;

    /// Minimum fix quality for navigation modes (default: 3D fix).
    virtual bool fixAcceptableForNavigation() const = 0;
};

} // namespace HAL
} // namespace AeroCore
