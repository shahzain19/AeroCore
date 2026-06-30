#include "Physics/PhysicsEngine.h"
#include "Utilities/Config.h"
#include "test_common.h"

#include <fstream>

using AeroCore::Physics::ExternalForces;
using AeroCore::Physics::PhysicsEngine;
using AeroCore::Physics::RigidBodyState;
using AeroCore::Utilities::Config;

int main() {
    const std::string path = "tmp_test_physics.toml";
    {
        std::ofstream out(path);
        out << "[physics]\n";
        out << "gravity = 9.80665\n";
        out << "air_density = 1.225\n";
        out << "drag_coefficient = 0.47\n";
        out << "ground_restitution = 0.05\n";
        out << "ground_friction = 0.6\n";
    }

    Config cfg(path);
    PhysicsEngine engine(cfg);

    RigidBodyState state;
    state.position.z() = -5.0; // 5 m altitude in NED

    ExternalForces forces;
    const double mass = 1.5;
    const double drag = 0.47;
    const double area = 0.05;
    const double dt   = 0.004;

    const double z_before = state.position.z();
    engine.step(state, forces, mass, drag, area, dt);
    const double z_after = state.position.z();

    // With no thrust, gravity should pull the body downward (z increases in NED).
    AeroCore::Tests::expectTrue(z_after > z_before,
                                "free-fall increases NED z (downward motion)");

    const double rho_sea = engine.airDensityAtAltitude(0.0);
    const double rho_high = engine.airDensityAtAltitude(5000.0);
    AeroCore::Tests::expectTrue(rho_high < rho_sea,
                                "air density decreases with altitude");

    std::remove(path.c_str());
    return AeroCore::Tests::finish();
}
