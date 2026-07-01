#include "Simulation/SimulationEngine.h"
#include "Flight/FlightMode.h"
#include "test_common.h"

#include <fstream>

using AeroCore::Flight::FlightMode;
using AeroCore::Simulation::SimulationEngine;

int main() {
    SimulationEngine engine("config/simulation.toml");
    auto& fc = engine.flightController();
    auto& rc = engine.rcInput();

    // Settle estimator.
    for (int i = 0; i < 250; ++i) engine.stepPhysics();

    rc.setThrottle(0.0);
    AeroCore::Tests::expectTrue(fc.canArm(), "level + low throttle can arm");

    rc.setThrottle(0.5);
    AeroCore::Tests::expectTrue(!fc.canArm(), "high throttle blocks arm");
    AeroCore::Tests::expectTrue(!fc.preArmStatus().empty(), "pre-arm reason provided");

    rc.setThrottle(0.0);
    fc.arm();
    AeroCore::Tests::expectTrue(fc.getMode() == FlightMode::ARMED, "arm succeeds when checks pass");

    fc.disarm();
    rc.setThrottle(0.8);
    fc.arm();
    AeroCore::Tests::expectTrue(fc.getMode() == FlightMode::DISARMED,
                                "arm rejected with throttle high");

    return AeroCore::Tests::finish();
}
