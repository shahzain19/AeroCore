#include "Flight/FlightMode.h"
#include "test_common.h"

using AeroCore::Flight::FlightMode;
using AeroCore::Flight::flightModeToString;
using AeroCore::Flight::isFlightMode;
using AeroCore::Flight::requiresGPS;

int main() {
    AeroCore::Tests::expectTrue(
        flightModeToString(FlightMode::ALTITUDE_HOLD) == "ALT_HOLD",
        "altitude hold string");
    AeroCore::Tests::expectTrue(
        flightModeToString(FlightMode::RETURN_HOME) == "RTH",
        "return home string");

    AeroCore::Tests::expectTrue(!isFlightMode(FlightMode::DISARMED),
                                "disarmed is not in-flight");
    AeroCore::Tests::expectTrue(isFlightMode(FlightMode::ALTITUDE_HOLD),
                                "alt hold is in-flight");

    AeroCore::Tests::expectTrue(requiresGPS(FlightMode::POSITION_HOLD),
                                "position hold needs GPS");
    AeroCore::Tests::expectTrue(requiresGPS(FlightMode::RETURN_HOME),
                                "RTH needs GPS");
    AeroCore::Tests::expectTrue(!requiresGPS(FlightMode::ALTITUDE_HOLD),
                                "alt hold does not need GPS");

    return AeroCore::Tests::finish();
}
