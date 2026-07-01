#include "Physics/PhysicsEngine.h"
#include "Utilities/Config.h"
#include "test_common.h"

#include <fstream>

using AeroCore::Math::Vector3d;
using AeroCore::Physics::PhysicsEngine;
using AeroCore::Utilities::Config;

int main() {
    const std::string path = "tmp_test_physics_airdrag.toml";
    {
        std::ofstream out(path);
        out << "[physics]\n";
        out << "gravity = 9.80665\n";
        out << "air_density = 1.225\n";
    }

    Config cfg(path);
    PhysicsEngine engine(cfg);

    const Vector3d velocity(10.0, 0.0, 0.0);
    const Vector3d wind(0.0, 0.0, 0.0);
    const auto drag = engine.dragForce(velocity, wind, 0.47, 0.05, 0.0);

    AeroCore::Tests::expectTrue(drag.norm() > 0.0,
                                "drag force is non-zero for forward motion");
    AeroCore::Tests::expectTrue(drag.dot(velocity) < 0.0,
                                "drag opposes the relative velocity");

    const auto gravity = engine.gravityForce(1.5, 0.0);
    AeroCore::Tests::expectTrue(gravity.z() > 0.0,
                                "gravity points downward in NED coordinates");

    const double rho_sea = engine.airDensityAtAltitude(0.0);
    const double rho_high = engine.airDensityAtAltitude(1000.0);
    AeroCore::Tests::expectTrue(rho_high < rho_sea,
                                "air density decreases with altitude");

    std::remove(path.c_str());
    return AeroCore::Tests::finish();
}
