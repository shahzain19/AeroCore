#include "Core/ComplementaryEstimator.h"
#include "Utilities/Config.h"
#include "test_common.h"

#include <cmath>

using AeroCore::Core::ComplementaryEstimator;
using AeroCore::HAL::BaroSample;
using AeroCore::HAL::IMUSample;

int main() {
  AeroCore::Utilities::Config config("config/simulation.toml");

  ComplementaryEstimator est(config);
  est.reset();

  // Level hover: specific force ≈ [0, 0, −g] in NED body frame.
  IMUSample imu{};
  imu.accel_body = AeroCore::Math::Vector3d(0.0, 0.0, -AeroCore::Math::GRAVITY_MSL);
  imu.gyro_body = AeroCore::Math::Vector3d::Zero();
  imu.valid = true;

  for (int i = 0; i < 500; ++i) {
    est.predict(0.004, imu);
  }

  const auto &euler = est.state().euler_rpy;
  AeroCore::Tests::expectTrue(est.state().attitude_valid, "attitude becomes valid");
  AeroCore::Tests::expectNear(euler.x(), 0.0, 0.05, "level roll estimate");
  AeroCore::Tests::expectNear(euler.y(), 0.0, 0.05, "level pitch estimate");

  // Roll right 0.3 rad: gravity projects into +Y.
  const double roll = 0.3;
  imu.accel_body = AeroCore::Math::Vector3d(
      0.0, AeroCore::Math::GRAVITY_MSL * std::sin(roll),
      -AeroCore::Math::GRAVITY_MSL * std::cos(roll));

  for (int i = 0; i < 800; ++i) {
    est.predict(0.004, imu);
  }

  AeroCore::Tests::expectNear(est.state().euler_rpy.x(), roll, 0.08,
                              "tracks roll tilt from accelerometer");

  BaroSample baro{};
  baro.altitude_m = 12.5;
  baro.valid = true;
  est.correctBaro(baro);
  AeroCore::Tests::expectNear(est.state().altitude_amsl, 12.5, 1e-6,
                              "baro updates altitude");

  est.injectPerfectNavigation(AeroCore::Math::Vector3d(1.0, 2.0, -3.0),
                              AeroCore::Math::Vector3d::Zero());
  AeroCore::Tests::expectTrue(est.state().position_valid, "perfect nav valid");
  AeroCore::Tests::expectNear(est.state().position_ned.x(), 1.0, 1e-9, "pos N");
  AeroCore::Tests::expectNear(est.state().position_ned.y(), 2.0, 1e-9, "pos E");

  est.clearPerfectNavigation();
  AeroCore::Tests::expectTrue(!est.state().position_valid,
                              "clear perfect nav invalidates position");

  return AeroCore::Tests::finish();
}
