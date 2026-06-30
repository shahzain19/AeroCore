/**
 * @file main.cpp
 * @brief AeroCore simulation entry point.
 *
 * Dispatches to headed or headless runners based on CLI arguments.
 * See Utilities::CliArgs::helpText() for usage.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Utilities/CliArgs.h"
#include "Simulation/HeadedSimulation.h"
#include "Simulation/HeadlessSimulation.h"
#include "Utilities/Logger.h"

#include <iostream>

using namespace AeroCore;

int main(int argc, char* argv[]) {
    std::string parse_error;
    const auto args = Utilities::CliArgs::parse(argc, argv, parse_error);

    if (args.show_help) {
        std::cout << Utilities::CliArgs::helpText();
        return 0;
    }
    if (!parse_error.empty()) {
        std::cerr << "[AeroCore] Error: " << parse_error << "\n"
                  << "Try 'AeroCore --help' for usage.\n";
        return 1;
    }

    try {
        if (args.headless) {
            Simulation::HeadlessSimulation sim(args.config_path, args);
            return sim.run();
        }
        Simulation::HeadedSimulation sim(args.config_path);
        return sim.run();
    } catch (const std::exception& e) {
        Utilities::Logger::getInstance().error(std::string("Fatal: ") + e.what());
        std::cerr << "[AeroCore] Fatal error: " << e.what() << "\n";
        return 1;
    }
}
