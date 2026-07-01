/**
 * @file Accelerometer.h
 * @brief 3-axis accelerometer sensor model for AeroCore.
 *
 * ## Sensor Model
 * The accelerometer measures **specific force** (acceleration minus gravity)
 * in the **body frame**:
 *
 *   a_meas = a_body − g_body + noise + drift_bias
 *
 * where g_body = R^T · [0, 0, g] in NED, i.e. in body frame gravity is
 * rotated out.  This replicates real MEMS accelerometer behaviour.
 *
 * ## Output
 * The 3-D measured acceleration vector (in body frame) in m/s².
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include "Sensors/Sensor.h"
#include "Math/Vector.h"

namespace AeroCore {
namespace Sensors {

/**
 * @brief 3-axis MEMS accelerometer simulation.
 *
 * Noise is applied independently on each axis.  Drift is shared (common-mode
 * bias) across all axes in this simplified model.
 */
class Accelerometer : public Sensor {
public:
    /**
     * @brief Construct.
     *
     * @param update_rate   Sample rate [Hz].
     * @param noise_stddev  Per-axis RMS noise [m/s²].
     * @param drift_rate    Bias random-walk rate [m/s²/s].
     */
    Accelerometer(double update_rate, double noise_stddev, double drift_rate);

    // ----------------------------------------------------------
    //  3-D update
    // ----------------------------------------------------------

    /**
     * @brief Advance the accelerometer state.
     *
     * @param dt               Time step [s].
     * @param true_accel_body  True acceleration in body frame [m/s²].
     */
    void update(double dt, const Math::Vector3d& true_accel_body);

    // Override: unused for 3-D sensors — call the 3-D overload instead.
    void update(double dt, double /*true_value*/) override {
        (void)dt;
    }

    // ----------------------------------------------------------
    //  Accessors
    // ----------------------------------------------------------

    /// Most recent measured 3-D acceleration in body frame [m/s²].
    Math::Vector3d getAcceleration() const;

private:
    Math::Vector3d acceleration_;   ///< Measured acceleration [m/s²]

    // Independent per-axis noise generators
    std::normal_distribution<double> noise_x_;
    std::normal_distribution<double> noise_y_;
    std::normal_distribution<double> noise_z_;
};

} // namespace Sensors
} // namespace AeroCore
