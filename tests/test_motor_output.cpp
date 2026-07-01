#include "Flight/FlightController.h"
#include "Flight/Drone.h"
#include "Sensors/IMU.h"
#include "Sensors/Altimeter.h"
#include "Sensors/BatterySensor.h"
#include "Utilities/Config.h"
#include "test_common.h"

#include <vector>

using AeroCore::Flight::FlightController;
using AeroCore::Flight::Drone;
using AeroCore::Flight::FlightMode;
using AeroCore::Flight::ControlInput;
using AeroCore::Sensors::IMU;
using AeroCore::Sensors::Altimeter;
using AeroCore::Sensors::BatterySensor;
using AeroCore::Utilities::Config;

namespace {

class DummyStateEstimator : public AeroCore::HAL::IStateEstimator {
public:
    DummyStateEstimator() {
        state_.euler_rpy = AeroCore::Math::Vector3d::Zero();
        state_.attitude_valid = true;
        state_.position_valid = true;
        state_.velocity_valid = true;
    }

    void reset() override {}
    void predict(double /*dt*/, const AeroCore::HAL::IMUSample& /*imu*/) override {}
    void correctBaro(const AeroCore::HAL::BaroSample& /*baro*/) override {}
    void correctGPS(const AeroCore::HAL::GPSSample& /*gps*/) override {}
    const AeroCore::HAL::VehicleState& state() const override { return state_; }

    AeroCore::HAL::VehicleState state_;
};

class MockMotorOutput : public AeroCore::HAL::IMotorOutput {
public:
    explicit MockMotorOutput(size_t count) : count_(count) {}

    size_t motorCount() const override {
        return count_;
    }

    void write(size_t motor_index, double throttle_0_1) override {
        writes_.emplace_back(motor_index, throttle_0_1);
    }

    void disarmAll() override {
        ++disarm_calls_;
    }

    size_t count_;
    std::vector<std::pair<size_t, double>> writes_;
    int disarm_calls_{0};
};

} // namespace

int main() {
    const Config config("config/simulation.toml");
    auto drone = std::make_shared<Drone>(config);
    for (int i = 0; i < 4; ++i) {
        drone->addMotor(std::make_unique<AeroCore::Flight::Motor>(config));
    }
    auto imu = std::make_shared<IMU>(config);
    auto altimeter = std::make_shared<Altimeter>(100.0, 0.0, 0.0);
    auto battery = std::make_shared<BatterySensor>(10.0, 0.0, 0.0);
    DummyStateEstimator estimator;
    FlightController fc(drone, imu, altimeter, battery, estimator, config);

    MockMotorOutput motor_output(4);
    fc.setMotorOutput(&motor_output);

    fc.arm();
    AeroCore::Tests::expectTrue(fc.getMode() == FlightMode::ARMED,
                                "FlightController arms successfully with motor output bound");

    fc.requestMode(FlightMode::STABILIZE);
    AeroCore::Tests::expectTrue(fc.getMode() == FlightMode::STABILIZE,
                                "STABILIZE mode request accepted after arming");

    fc.update(0.02);
    AeroCore::Tests::expectTrue(!motor_output.writes_.empty(),
                                "Motor output receives at least one write during update");
    AeroCore::Tests::expectTrue(motor_output.disarm_calls_ == 0,
                                "Motor output disarm was not called before disarm()");

    fc.disarm();
    AeroCore::Tests::expectTrue(fc.getMode() == FlightMode::DISARMED,
                                "FlightController disarms cleanly");
    AeroCore::Tests::expectTrue(motor_output.disarm_calls_ == 1,
                                "disarm() forwards to HAL motor output disarmAll()");

    bool values_ok = true;
    for (auto& entry : motor_output.writes_) {
        const auto [index, throttle] = entry;
        if (index >= motor_output.motorCount() || throttle < -1e-9 || throttle > 1.0 + 1e-9) {
            values_ok = false;
            break;
        }
    }
    AeroCore::Tests::expectTrue(values_ok,
                                "Motor output writes are within valid index and throttle range");

    return AeroCore::Tests::finish();
}
