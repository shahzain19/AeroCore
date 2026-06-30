/**
 * @file HeadlessSimulation.h
 * @brief Console-only simulation runner with auto-flight sequence.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include "Simulation/SimulationEngine.h"
#include "Simulation/TelemetryManager.h"
#include "Utilities/CliArgs.h"

#include <string>

namespace AeroCore {
namespace Simulation {

/**
 * @brief Runs simulation without rendering; prints status lines to stdout.
 */
class HeadlessSimulation {
public:
    HeadlessSimulation(const std::string& config_hint,
                       const Utilities::CliArgs& args);

    /** Run until duration elapsed. Returns process exit code. */
    int run();

private:
    SimulationEngine engine_;
    TelemetryManager telemetry_;
    Utilities::CliArgs args_;

    double accumulator_ = 0.0;
    double status_elapsed_ = 0.0;

    void applyAutoFlight();
    void stepFixedTimestep(double frame_dt);
    void maybePrintStatus(double frame_dt);
    void maybePrintDebug();
};

} // namespace Simulation
} // namespace AeroCore
