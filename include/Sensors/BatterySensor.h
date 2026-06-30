/**
 * @file BatterySensor.h
 * @brief Battery voltage / current / SOC sensor model for AeroCore.
 *
 * ## Battery Model
 * The sensor measures terminal voltage with noise.  The underlying battery
 * model (in Drone) tracks:
 *
 *   V_terminal = V_OCV(SOC) − I · R_internal
 *
 * where V_OCV is the open-circuit voltage curve (approximated as linear for
 * LiPo) and I is the total current draw from all motors.
 *
 * The sensor adds realistic measurement noise and a configurable sampling
 * delay to mimic a real ADC-based voltage monitor.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include "Sensors/Sensor.h"

namespace AeroCore {
namespace Sensors {

/**
 * @brief Battery voltage sensor simulation.
 */
class BatterySensor : public Sensor {
public:
    /**
     * @brief Construct.
     *
     * @param update_rate   Sample rate [Hz].
     * @param noise_stddev  Voltage measurement noise [V RMS].
     * @param drift_rate    Bias drift rate [V/s].
     */
    BatterySensor(double update_rate, double noise_stddev, double drift_rate);

    // ----------------------------------------------------------
    //  Update
    // ----------------------------------------------------------

    /**
     * @brief Advance sensor state.
     *
     * @param dt            Time step [s].
     * @param true_voltage  True terminal voltage [V].
     */
    void update(double dt, double true_voltage) override;

    // ----------------------------------------------------------
    //  Accessors
    // ----------------------------------------------------------

    /// Measured battery terminal voltage [V].
    double getVoltage() const;

private:
    double voltage_;   ///< Measured voltage [V]
};

} // namespace Sensors
} // namespace AeroCore
