#include "Flight/FlightController.h"
#include "Core/ComplementaryEstimator.h"
#include "Sensors/IMU.h"
#include "Sensors/Altimeter.h"
#include "Sensors/BatterySensor.h"
#include "Utilities/Config.h"
#include "test_common.h"

#include <memory>

namespace {
class FakeRCInput : public AeroCore::HAL::IRCInput {
public:
    explicit FakeRCInput(bool healthy = true, uint32_t ms_since_last = 0)
        : healthy_(healthy), ms_since_last_(ms_since_last) {}

    AeroCore::HAL::RCChannels read() const override {
        AeroCore::HAL::RCChannels ch{};
        ch.channel_count = 4;
        ch.valid = healthy_;
        ch.link_status = healthy_ ? AeroCore::HAL::RCLinkStatus::Connected
                                 : AeroCore::HAL::RCLinkStatus::Lost;
        return ch;
    }

    uint32_t msSinceLastFrame() const override { return ms_since_last_; }
    bool healthy() const override { return healthy_; }

    void setHealthy(bool healthy) { healthy_ = healthy; }
    void setMsSinceLastFrame(uint32_t ms) { ms_since_last_ = ms; }

private:
    bool healthy_;
    uint32_t ms_since_last_;
};
} // namespace

int main() {
    AeroCore::Utilities::Config config("config/simulation.toml");
    auto drone = std::make_shared<AeroCore::Flight::Drone>(config);
    drone->addMotor(std::make_unique<AeroCore::Flight::Motor>(config));
    drone->addMotor(std::make_unique<AeroCore::Flight::Motor>(config));
    drone->addMotor(std::make_unique<AeroCore::Flight::Motor>(config));
    drone->addMotor(std::make_unique<AeroCore::Flight::Motor>(config));

    auto imu = std::make_shared<AeroCore::Sensors::IMU>(config);
    auto altimeter = std::make_shared<AeroCore::Sensors::Altimeter>(50.0, 0.05, 0.001);
    auto battery = std::make_shared<AeroCore::Sensors::BatterySensor>(10.0, 0.02, 0.0);
    AeroCore::Core::ComplementaryEstimator estimator(config);
    AeroCore::Flight::FlightController controller(drone, imu, altimeter, battery, estimator, config);

    FakeRCInput rc(true, 0);
    controller.setRCInput(&rc);

    estimator.setAttitudeRollPitch(0.0, 0.0);
    controller.arm();
    AeroCore::Tests::expectTrue(controller.getMode() == AeroCore::Flight::FlightMode::ARMED,
                                "controller arms when pre-arm checks pass");

    altimeter->update(0.0, 5.0);
    rc.setHealthy(false);
    rc.setMsSinceLastFrame(1000);
    controller.update(0.01);

    AeroCore::Tests::expectTrue(controller.getMode() != AeroCore::Flight::FlightMode::ARMED,
                                "RC loss removes the aircraft from armed control");

    return AeroCore::Tests::finish();
}
