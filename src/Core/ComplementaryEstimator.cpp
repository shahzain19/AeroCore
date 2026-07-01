/**
 * @file ComplementaryEstimator.cpp
 */

#include "Core/ComplementaryEstimator.h"
#include "Math/Vector.h"
#include <cmath>

namespace AeroCore {
namespace Core {

ComplementaryEstimator::ComplementaryEstimator(const Utilities::Config& config) {
    try {
        comp_alpha_ = config.get<double>("imu", "comp_filter_alpha");
    } catch (...) {}
    reset();
}

void ComplementaryEstimator::reset() {
    state_ = HAL::VehicleState{};
    state_.attitude = Math::Quaterniond::Identity();
    perfect_nav_ = false;
}

void ComplementaryEstimator::updateQuaternionFromEuler() {
    const double roll  = state_.euler_rpy.x();
    const double pitch = state_.euler_rpy.y();
    const double yaw   = state_.euler_rpy.z();
    state_.attitude =
        Math::AngleAxisd(yaw,   Math::Vector3d::UnitZ()) *
        Math::AngleAxisd(pitch, Math::Vector3d::UnitY()) *
        Math::AngleAxisd(roll,  Math::Vector3d::UnitX());
    state_.attitude.normalize();
    state_.attitude_valid = true;
}

void ComplementaryEstimator::setAttitudeRollPitch(double roll_rad, double pitch_rad) {
    state_.euler_rpy.x() = Math::wrapAngle(roll_rad);
    state_.euler_rpy.y() = Math::wrapAngle(pitch_rad);
    updateQuaternionFromEuler();
}

void ComplementaryEstimator::setAttitudeEuler(const Math::Vector3d& euler_rpy) {
    state_.euler_rpy = Math::Vector3d(
        Math::wrapAngle(euler_rpy.x()),
        Math::wrapAngle(euler_rpy.y()),
        Math::wrapAngle(euler_rpy.z()));
    updateQuaternionFromEuler();
}

void ComplementaryEstimator::predict(double dt, const HAL::IMUSample& imu) {
    if (!imu.valid || dt <= 0.0) return;

    const auto& accel = imu.accel_body;
    const auto& omega = imu.gyro_body;

    double roll  = state_.euler_rpy.x();
    double pitch = state_.euler_rpy.y();
    double yaw   = state_.euler_rpy.z();

    // Standalone fusion (embedded): complementary filter on raw IMU.
    const double roll_gyro  = roll  + omega.x() * dt;
    const double pitch_gyro = pitch + omega.y() * dt;
    yaw += omega.z() * dt;

    const double ax = accel.x();
    const double ay = accel.y();
    const double az = accel.z();
    const double a_mag = std::sqrt(ax * ax + ay * ay + az * az);
    const double g_est = Math::GRAVITY_MSL;

    double trust = 1.0;
    double roll_accel  = roll;
    double pitch_accel = pitch;

    if (a_mag > 0.5 * g_est && a_mag < 2.0 * g_est) {
        roll_accel  = std::atan2(ay, -az);
        pitch_accel = std::atan2(-ax, std::sqrt(ay * ay + az * az));
        trust = comp_alpha_;
    }

    roll  = trust * roll_gyro  + (1.0 - trust) * roll_accel;
    pitch = trust * pitch_gyro + (1.0 - trust) * pitch_accel;

    state_.euler_rpy = Math::Vector3d(
        Math::wrapAngle(roll),
        Math::wrapAngle(pitch),
        Math::wrapAngle(yaw));
    state_.angular_rate_body = omega;
    updateQuaternionFromEuler();
}

void ComplementaryEstimator::correctBaro(const HAL::BaroSample& baro) {
    if (!baro.valid) return;
    state_.altitude_amsl = baro.altitude_m;
}

void ComplementaryEstimator::correctGPS(const HAL::GPSSample& gps) {
    if (!gps.valid || gps.fix_type == HAL::GPSFixType::NoFix) return;

    state_.position_ned     = gps.position_ned;
    state_.velocity_ned     = gps.velocity_ned;
    state_.altitude_amsl    = gps.altitude_amsl_m;
    state_.position_valid   = true;
    state_.velocity_valid   = true;
}

void ComplementaryEstimator::injectPerfectNavigation(
        const Math::Vector3d& position_ned,
        const Math::Vector3d& velocity_ned) {
    state_.position_ned   = position_ned;
    state_.velocity_ned   = velocity_ned;
    state_.position_valid = true;
    state_.velocity_valid = true;
    perfect_nav_ = true;
}

void ComplementaryEstimator::clearPerfectNavigation() {
    if (!perfect_nav_) return;
    perfect_nav_ = false;
    state_.position_valid = false;
    state_.velocity_valid = false;
    state_.position_ned   = Math::Vector3d::Zero();
    state_.velocity_ned   = Math::Vector3d::Zero();
}

} // namespace Core
} // namespace AeroCore
