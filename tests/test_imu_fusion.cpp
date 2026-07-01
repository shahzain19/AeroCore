#include "Simulation/SimulationEngine.h"
#include "Flight/FlightMode.h"
#include "Utilities/Config.h"
#include "test_common.h"

#include <algorithm>
#include <fstream>

using AeroCore::Flight::FlightMode;
using AeroCore::Simulation::SimulationEngine;

int main() {
    const std::string path = "tmp_imu_fusion.toml";
    {
        std::ifstream in("config/simulation.toml");
        std::ofstream out(path);
        out << in.rdbuf();
    }
    {
        std::ofstream out(path, std::ios::app);
        out << "\n[simulation]\nperfect_state = false\n";
    }

    SimulationEngine engine(path);
    auto& fc = engine.flightController();
    engine.rcInput().setThrottle(0.0);

    for (int i = 0; i < 500; ++i) engine.stepPhysics();

    fc.arm();
    fc.takeoff();

    double max_alt = 0.0;
    bool saw_alt_hold = false;

    const int steps = 5000; // 20 s
    for (int i = 0; i < steps; ++i) {
        engine.stepPhysics();
        max_alt = std::max(max_alt, engine.drone().getAltitude());
        if (fc.getMode() == FlightMode::ALTITUDE_HOLD) saw_alt_hold = true;
    }

    AeroCore::Tests::expectTrue(max_alt > 2.5, "IMU-only fusion climbs above 2.5 m");
    AeroCore::Tests::expectTrue(!engine.estimator().state().position_valid,
                                "no GPS position without perfect_state");
    (void)saw_alt_hold;

    std::remove(path.c_str());
    return AeroCore::Tests::finish();
}
