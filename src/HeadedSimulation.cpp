/**
 * @file HeadedSimulation.cpp
 * @brief Interactive GUI simulation runner.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Simulation/HeadedSimulation.h"
#include "Utilities/Logger.h"

#include <SFML/System.hpp>
#include <algorithm>

namespace AeroCore {
namespace Simulation {

HeadedSimulation::HeadedSimulation(const std::string& config_hint)
    : engine_(config_hint)
    , renderer_(std::make_unique<Rendering::Renderer>(1440, 810, "AeroCore v2.0"))
{
    Utilities::Logger::getInstance().info("Headed simulation started (1440×810)");
}

void HeadedSimulation::applyInput(const Rendering::InputState& input, double frame_dt) {
    auto& fc = engine_.flightController();

    if (input.arm_toggle) {
        if (fc.getMode() == Flight::FlightMode::DISARMED) fc.arm();
        else fc.disarm();
    }
    if (input.takeoff) fc.takeoff();
    if (input.land)    fc.land();
    if (input.reset) {
        engine_.reset();
        accumulator_ = 0.0;
    }

    const double wind_step = 2.0 * frame_dt;
    auto& wind = engine_.wind();
    if (input.wind_north) wind.x() += wind_step;
    if (input.wind_south) wind.x() -= wind_step;
    if (input.wind_east)  wind.y() += wind_step;
    if (input.wind_west)  wind.y() -= wind_step;

    const double alt_step = 5.0 * frame_dt;
    if (input.alt_increase) fc.setTargetAltitude(fc.getTargetAltitude() + alt_step);
    if (input.alt_decrease) fc.setTargetAltitude(fc.getTargetAltitude() - alt_step);

    if (input.mode_stabilize) fc.requestMode(Flight::FlightMode::STABILIZE);
    if (input.mode_alt_hold)  fc.requestMode(Flight::FlightMode::ALTITUDE_HOLD);
    if (input.mode_pos_hold)  fc.requestMode(Flight::FlightMode::POSITION_HOLD);
    if (input.mode_rth)       fc.requestMode(Flight::FlightMode::RETURN_HOME);
}

void HeadedSimulation::stepFixedTimestep(double frame_dt) {
    const double physics_dt = engine_.physicsDt();
    const double MAX_FRAME_DT = 0.1;
    const double clamped_dt = std::max(physics_dt, std::min(frame_dt, MAX_FRAME_DT));

    accumulator_ += clamped_dt;
    while (accumulator_ >= physics_dt) {
        engine_.stepPhysics();
        accumulator_ -= physics_dt;
    }
}

int HeadedSimulation::run() {
    sf::Clock clock;
    bool running = true;

    while (running && renderer_->isOpen()) {
        const double raw_dt = static_cast<double>(clock.restart().asSeconds());
        const double frame_dt = std::max(engine_.physicsDt(), raw_dt);

        const auto input = renderer_->pollEvents();
        if (!renderer_->isOpen() || input.quit) break;

        applyInput(input, frame_dt);
        stepFixedTimestep(frame_dt);

        TelemetryData telemetry{};
        engine_.gatherTelemetry(telemetry);
        telemetry_.update(frame_dt, telemetry);

        renderer_->clear();
        renderer_->render(engine_.drone(), telemetry_.getCurrentData(),
                          engine_.wind());
        renderer_->display();
    }

    Utilities::Logger::getInstance().info(
        "Headed simulation ended — sim_time=" + std::to_string(engine_.simTime()) + "s");
    return 0;
}

} // namespace Simulation
} // namespace AeroCore
