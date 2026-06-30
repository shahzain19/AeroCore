/**
 * @file Altimeter.h
 * @brief Barometric altimeter sensor model for AeroCore.
 *
 * ## Sensor Model
 * The altimeter measures altitude via static pressure using the ISA barometric
 * formula.  In this simulation we abstract that to a direct altitude measurement
 * with realistic noise and lag to represent a MEMS barometer (e.g. BMP280, MS5611).
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include "Sensors/Sensor.h"

namespace AeroCore {
namespace Sensors {

/**
 * @brief Barometric altimeter.
 *
 * Measures altitude above the reference ground level (takeoff point).
 * Noise parameters are tuned to represent a typical MEMS barometer.
 */
class Altimeter : public Sensor {
public:
    /**
     * @brief Construct.
     *
     * @param update_rate   Sample rate [Hz].  Typical barometer: 25–100 Hz.
     * @param noise_stddev  Altitude noise [m RMS].
     * @param drift_rate    Bias drift rate [m/s].
     */
    Altimeter(double update_rate, double noise_stddev, double drift_rate);

    // ----------------------------------------------------------
    //  Update
    // ----------------------------------------------------------

    /**
     * @brief Advance altimeter state.
     *
     * @param dt             Time step [s].
     * @param true_altitude  True altitude above ground [m].
     */
    void update(double dt, double true_altitude) override;

    // ----------------------------------------------------------
    //  Accessors
    // ----------------------------------------------------------

    /// Measured altitude above reference level [m].
    double getAltitude() const;

private:
    double altitude_;   ///< Measured altitude [m]
};

} // namespace Sensors
} // namespace AeroCore
