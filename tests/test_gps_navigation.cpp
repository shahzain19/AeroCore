#include "Simulation/SimulationEngine.h"
#include "test_common.h"

using AeroCore::Simulation::SimulationEngine;

int main() {
    SimulationEngine engine("config/simulation.toml");

    for (int i = 0; i < 50; ++i) {
        engine.stepPhysics();
    }

    const auto& state = engine.estimator().state();
    AeroCore::Tests::expectTrue(state.position_valid,
                                "GPS correction makes position estimate valid");
    AeroCore::Tests::expectTrue(state.velocity_valid,
                                "GPS correction makes velocity estimate valid");

    return AeroCore::Tests::finish();
}
