/**
 * @file HeadedSimulation.h
 * @brief Interactive GUI simulation runner (SFML).
 *
 * Rebuilt as a dedicated runner on top of SimulationEngine so main.cpp stays
 * a thin entry point.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include "Simulation/SimulationEngine.h"
#include "Rendering/Renderer.h"
#include "Simulation/TelemetryManager.h"
#include "Utilities/CliArgs.h"

#include <memory>

namespace AeroCore {
namespace Simulation {

/**
 * @brief Runs the fixed-timestep simulation loop with SFML rendering.
 */
class HeadedSimulation {
public:
    HeadedSimulation(const std::string& config_hint,
                     const Utilities::CliArgs& args = Utilities::CliArgs());

    /** Run until the window is closed. Returns process exit code. */
    int run();

private:
    SimulationEngine engine_;
    std::unique_ptr<Rendering::Renderer> renderer_;
    TelemetryManager telemetry_;

    double accumulator_ = 0.0;

    void applyInput(const Rendering::InputState& input, double frame_dt);
    void stepFixedTimestep(double frame_dt);
};

} // namespace Simulation
} // namespace AeroCore
