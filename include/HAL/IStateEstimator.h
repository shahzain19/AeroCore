/**
 * @file IStateEstimator.h
 * @brief State estimator interface (complementary filter → EKF).
 */

#pragma once

#include "HAL/Types.h"
#include "HAL/IIMU.h"
#include "HAL/IBarometer.h"
#include "HAL/IGPS.h"

namespace AeroCore {
namespace HAL {

class IStateEstimator {
public:
    virtual ~IStateEstimator() = default;

    virtual void reset() = 0;

    /// Propagate state with IMU at control rate.
    virtual void predict(double dt, const IMUSample& imu) = 0;

    virtual void correctBaro(const BaroSample& baro) = 0;
    virtual void correctGPS(const GPSSample& gps) = 0;

    virtual const VehicleState& state() const = 0;
};

} // namespace HAL
} // namespace AeroCore
