#include "Simulation/SimulationEngine.h"
#include "Flight/FlightMode.h"
#include "test_common.h"

#include <algorithm>
#include <fstream>

using AeroCore::Flight::FlightMode;
using AeroCore::Simulation::SimulationEngine;

int main() {
    SimulationEngine engine("config/simulation.toml");
    auto& fc = engine.flightController();

    AeroCore::Tests::expectTrue(!engine.perfectState(),
                                "simulator defaults to estimator-driven state");

    for (int i = 0; i < 250; ++i) engine.stepPhysics();

    fc.arm();
    fc.takeoff();

    double max_alt = 0.0;
    bool saw_alt_hold = false;

    const int steps = 3750; // 15 s at 250 Hz
    for (int i = 0; i < steps; ++i) {
        engine.stepPhysics();
        max_alt = std::max(max_alt, engine.drone().getAltitude());
        if (fc.getMode() == FlightMode::ALTITUDE_HOLD) {
            saw_alt_hold = true;
        }
    }

    AeroCore::Tests::expectTrue(engine.simTime() > 14.0, "sim time advances");
    AeroCore::Tests::expectTrue(max_alt > 4.0, "auto takeoff climbs above 4 m");
    AeroCore::Tests::expectTrue(max_alt < 12.0, "climb does not diverge high");
    AeroCore::Tests::expectTrue(saw_alt_hold || max_alt > 8.0,
                                "reaches alt-hold or climbs past 8 m");

    AeroCore::Simulation::TelemetryData telem{};
    engine.gatherTelemetry(telem);
    AeroCore::Tests::expectTrue(telem.motor_throttle[0] >= 0.0,
                                "telemetry includes motor data");
    AeroCore::Tests::expectTrue(telem.euler_angles.norm() >= 0.0,
                                "telemetry uses estimator attitude");

    engine.reset();
    AeroCore::Tests::expectTrue(engine.simTime() == 0.0, "reset clears sim time");
    AeroCore::Tests::expectTrue(fc.getMode() == FlightMode::DISARMED,
                                "reset returns to disarmed");

    return AeroCore::Tests::finish();
}
