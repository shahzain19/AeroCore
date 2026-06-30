/**
 * @file IMU.h
 * @brief Inertial Measurement Unit (IMU) combining 3-axis accelerometer
 *        and 3-axis gyroscope for AeroCore.
 *
 * ## Overview
 * The IMU class aggregates the Accelerometer and Gyroscope into a single
 * sensor unit that mirrors a real 6-DOF MEMS IMU (e.g. MPU-6050, ICM-42688).
 *
 * It optionally applies a simple **complementary filter** to estimate the
 * vehicle's attitude (roll, pitch) from accel + gyro fusion, which is
 * useful when a full EKF/UKF is overkill for simulation.
 *
 * ## Complementary Filter
 *   φ̂[k] = α · (φ̂[k−1] + ω_x · dt) + (1−α) · atan2(ay, az)
 *   θ̂[k] = α · (θ̂[k−1] + ω_y · dt) + (1−α) · atan2(−ax, sqrt(ay²+az²))
 *
 * where α is the gyro trust coefficient (typically 0.98).
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include "Sensors/Accelerometer.h"
#include "Sensors/Gyroscope.h"
#include "Math/Vector.h"
#include "Utilities/Config.h"
#include <memory>

namespace AeroCore {
namespace Sensors {

/**
 * @brief Combined 6-DOF IMU (accelerometer + gyroscope).
 *
 * Provides the primary attitude and angular-rate feedback for the flight
 * controller's inner loops.
 */
class IMU {
public:
    /**
     * @brief Construct from config.
     *
     * Reads noise/update-rate parameters from [imu] section if present,
     * otherwise uses reasonable defaults for a mid-grade MEMS IMU.
     */
    explicit IMU(const Utilities::Config& config);

    // ----------------------------------------------------------
    //  Update
    // ----------------------------------------------------------

    /**
     * @brief Advance IMU simulation.
     *
     * @param dt               Time step [s].
     * @param true_accel_body  True acceleration in body frame [m/s²].
     * @param true_omega_body  True angular velocity in body frame [rad/s].
     * @param orientation      True vehicle orientation quaternion (for
     *                         gravity removal in accel model).
     */
    void update(double dt,
                const Math::Vector3d& true_accel_body,
                const Math::Vector3d& true_omega_body,
                const Math::Quaterniond& orientation);

    // ----------------------------------------------------------
    //  Sensor access
    // ----------------------------------------------------------

    Accelerometer&       getAccelerometer();
    Gyroscope&           getGyroscope();
    const Accelerometer& getAccelerometer() const;
    const Gyroscope&     getGyroscope() const;

    // ----------------------------------------------------------
    //  Fused attitude estimate (complementary filter)
    // ----------------------------------------------------------

    /**
     * @brief Estimated roll angle from complementary filter [rad].
     */
    double getEstimatedRoll()  const;

    /**
     * @brief Estimated pitch angle from complementary filter [rad].
     */
    double getEstimatedPitch() const;

    /**
     * @brief Set complementary filter gyro trust coefficient α.
     * @param alpha  Typically 0.98.  Higher = more gyro trust.
     */
    void setComplementaryAlpha(double alpha);

private:
    std::unique_ptr<Accelerometer> accelerometer_;
    std::unique_ptr<Gyroscope>     gyroscope_;

    // Complementary filter state
    double comp_alpha_;        ///< Gyro trust coefficient
    double estimated_roll_;    ///< Filtered roll estimate [rad]
    double estimated_pitch_;   ///< Filtered pitch estimate [rad]

    /// Run one iteration of the complementary filter.
    void updateComplementaryFilter(double dt,
                                   const Math::Vector3d& accel,
                                   const Math::Vector3d& omega);
};

} // namespace Sensors
} // namespace AeroCore
