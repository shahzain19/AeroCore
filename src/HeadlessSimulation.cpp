/**
 * @file HeadlessSimulation.cpp
 * @brief Console-only simulation runner.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Simulation/HeadlessSimulation.h"
#include "Utilities/Logger.h"

#include <SFML/System.hpp>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace AeroCore {
namespace Simulation {

HeadlessSimulation::HeadlessSimulation(const std::string& config_hint,
                                     const Utilities::CliArgs& args)
    : engine_(config_hint)
    , args_(args)
{
    engine_.setPerfectState(args.perfect_state);
    Utilities::Logger::getInstance().info("Headless simulation started");
}

void HeadlessSimulation::applyAutoFlight() {
    auto& fc = engine_.flightController();
    if (fc.getMode() == Flight::FlightMode::DISARMED) fc.arm();
    if (fc.getMode() == Flight::FlightMode::ARMED)    fc.takeoff();
}

void HeadlessSimulation::stepFixedTimestep(double frame_dt) {
    const double physics_dt = engine_.physicsDt();
    const double MAX_FRAME_DT = 0.1;
    const double clamped_dt = std::max(physics_dt, std::min(frame_dt, MAX_FRAME_DT));

    accumulator_ += clamped_dt;
    while (accumulator_ >= physics_dt) {
        engine_.stepPhysics();
        maybePrintDebug();
        accumulator_ -= physics_dt;
    }
}

void HeadlessSimulation::maybePrintDebug() {
    if (!args_.headless_debug) return;

    const double sim_time = engine_.simTime();
    const double physics_dt = engine_.physicsDt();
    if (sim_time >= 3.0) return;
    if (static_cast<int>(sim_time / physics_dt) % 125 != 0) return;

    const auto& state = engine_.drone().getState();
    const auto euler  = engine_.drone().getEulerAngles();
    const auto omega  = engine_.drone().getAngularVelocity();

    double total_thrust = 0.0;
    std::ostringstream motor_ss;
    for (size_t i = 0; i < engine_.drone().getMotorCount(); ++i) {
        if (i != 0) motor_ss << "/";
        motor_ss << engine_.drone().getMotor(i).getThrottle();
        total_thrust += engine_.drone().getMotor(i).getCurrentThrust();
    }

    const auto forces = engine_.drone().buildExternalForces(
        state.orientation, engine_.wind(),
        engine_.physics().airDensityAtAltitude(engine_.drone().getAltitude()));

    std::cout << "[DBG] t=" << std::fixed << std::setprecision(2) << sim_time
              << " z=" << state.position.z()
              << " roll=" << euler.x() * 57.3
              << " p=" << omega.x()
              << " Fz=" << forces.thrust_world.z()
              << " M:" << motor_ss.str()
              << "\n";
}

void HeadlessSimulation::maybePrintStatus(double frame_dt) {
    const double status_period = 1.0 / args_.headless_status_rate_hz;
    status_elapsed_ += frame_dt;
    if (status_elapsed_ < status_period) return;

    std::cout << telemetry_.getStatusLine() << "\n";
    status_elapsed_ = 0.0;
}

int HeadlessSimulation::run() {
    sf::Clock clock;
    bool running = true;

    while (running) {
        const double raw_dt = static_cast<double>(clock.restart().asSeconds());
        const double frame_dt = std::max(engine_.physicsDt(), raw_dt);

        applyAutoFlight();
        if (engine_.simTime() > args_.max_sim_time) break;

        stepFixedTimestep(frame_dt);

        TelemetryData telemetry{};
        engine_.gatherTelemetry(telemetry);
        telemetry_.update(frame_dt, telemetry);
        maybePrintStatus(frame_dt);
    }

    Utilities::Logger::getInstance().info(
        "Headless simulation ended — sim_time=" + std::to_string(engine_.simTime()) + "s");
    return 0;
}

} // namespace Simulation
} // namespace AeroCore
