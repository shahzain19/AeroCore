#include "Simulation/TelemetryManager.h"
#include "test_common.h"

#include <string>

using AeroCore::Simulation::TelemetryData;
using AeroCore::Simulation::TelemetryManager;

int main() {
    TelemetryManager manager;
    TelemetryData data{};

    data.simulation_time = 12.5;
    data.altitude = 3.2;
    data.current_thrust = 11.0;
    data.battery_soc = 97.0;
    data.flight_mode = AeroCore::Flight::FlightMode::ALTITUDE_HOLD;
    data.air_density = 1.21;

    // Drive FPS estimator above the 1-second update threshold.
    for (int i = 0; i < 11; ++i) {
        data.simulation_time += 0.1;
        manager.update(0.1, data);
    }

    AeroCore::Tests::expectNear(manager.getFPS(), 10.0, 0.5, "fps estimate from fixed frame time");

    const std::string status = manager.getStatusLine();
    AeroCore::Tests::expectTrue(status.find("Mode=ALT_HOLD") != std::string::npos,
                                "status line prints flight mode");
    AeroCore::Tests::expectTrue(status.find("Alt=") != std::string::npos,
                                "status line includes altitude");

    const std::string hud = manager.getHUDString();
    AeroCore::Tests::expectTrue(hud.find("AEROCORE FLIGHT COMPUTER") != std::string::npos,
                                "hud has title");
    AeroCore::Tests::expectTrue(hud.find("PERFORMANCE") != std::string::npos,
                                "hud includes performance section");

    return AeroCore::Tests::finish();
}
