/**
 * @file ComplementaryEstimator.h
 * @brief Attitude + altitude estimator using complementary filtering.
 *
 * Fuses gyro integration with accelerometer tilt for roll/pitch. Yaw is
 * gyro-integrated (drifts without magnetometer/GPS). Baro corrects altitude.
 * Optional perfect navigation injection for simulation GPS modes.
 */

#pragma once

#include "HAL/IStateEstimator.h"
#include "Utilities/Config.h"

namespace AeroCore {
namespace Core {

class ComplementaryEstimator : public HAL::IStateEstimator {
public:
    explicit ComplementaryEstimator(const Utilities::Config& config);

    void reset() override;
    void predict(double dt, const HAL::IMUSample& imu) override;
    void correctBaro(const HAL::BaroSample& baro) override;
    void correctGPS(const HAL::GPSSample& gps) override;

    const HAL::VehicleState& state() const override { return state_; }

    /// Apply fused roll/pitch from an external IMU (sim or driver).
    void setAttitudeRollPitch(double roll_rad, double pitch_rad);

    /// Set full attitude euler (roll, pitch, yaw) — sim ground-truth injection.
    void setAttitudeEuler(const Math::Vector3d& euler_rpy);

    /// Sim/dev: feed ground-truth navigation (position hold / RTH without GPS model).
    void injectPerfectNavigation(const Math::Vector3d& position_ned,
                                 const Math::Vector3d& velocity_ned);

    void clearPerfectNavigation();

private:
    HAL::VehicleState state_;
    double comp_alpha_{0.98};
    bool   perfect_nav_{false};

    void updateQuaternionFromEuler();
};

} // namespace Core
} // namespace AeroCore
