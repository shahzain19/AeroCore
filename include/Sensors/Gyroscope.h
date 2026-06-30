/**
 * @file Gyroscope.h
 * @brief 3-axis gyroscope sensor model for AeroCore.
 *
 * ## Sensor Model
 * The gyroscope measures **angular velocity** in the body frame [rad/s]:
 *
 *   ω_meas = ω_true + noise + drift_bias
 *
 * Noise is applied independently per axis.  A slowly growing bias (drift)
 * simulates gyro drift over time.
 *
 * ## Axis Convention (body frame NED)
 *  - X = roll rate  p [rad/s]
 *  - Y = pitch rate q [rad/s]
 *  - Z = yaw rate   r [rad/s]
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
 * @brief 3-axis MEMS gyroscope simulation.
 */
class Gyroscope : public Sensor {
public:
    /**
     * @brief Construct.
     *
     * @param update_rate   Sample rate [Hz].
     * @param noise_stddev  Per-axis RMS noise [rad/s].
     * @param drift_rate    Bias random-walk rate [rad/s/s].
     */
    Gyroscope(double update_rate, double noise_stddev, double drift_rate);

    // ----------------------------------------------------------
    //  3-D update
    // ----------------------------------------------------------

    /**
     * @brief Advance gyroscope state.
     *
     * @param dt              Time step [s].
     * @param true_omega_body True angular velocity in body frame [rad/s].
     */
    void update(double dt, const Math::Vector3d& true_omega_body);

    // Override — not used for 3-D
    void update(double dt, double /*true_value*/) override {}

    // ----------------------------------------------------------
    //  Accessors
    // ----------------------------------------------------------

    /// Measured 3-D angular velocity in body frame [rad/s].
    Math::Vector3d getAngularVelocity() const;

private:
    Math::Vector3d angular_velocity_;   ///< Measured ω [rad/s]

    std::normal_distribution<double> noise_x_;
    std::normal_distribution<double> noise_y_;
    std::normal_distribution<double> noise_z_;
};

} // namespace Sensors
} // namespace AeroCore
