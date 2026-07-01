/**
 * @file CliArgs.cpp
 * @brief Command-line argument parsing for AeroCore.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Utilities/CliArgs.h"

#include <algorithm>
#include <cstdlib>
#include <sstream>

namespace AeroCore {
namespace Utilities {

std::string CliArgs::helpText() {
    std::ostringstream oss;
    oss << "AeroCore — open-source 6-DOF flight simulator\n"
        << "\n"
        << "Usage:\n"
        << "  AeroCore [options] [config_file]\n"
        << "\n"
        << "Options:\n"
        << "  -h, --help              Show this help message and exit\n"
        << "      --headless          Run without GUI (console telemetry only)\n"
        << "      --duration <sec>    Headless max simulation time (default: 60)\n"
        << "      --status-rate <hz>  Headless status-line refresh rate (default: 5)\n"
        << "      --debug-headless    Extra debug output during headless runs\n"
        << "      --no-perfect-state   Disable perfect-state simulator state injection\n"
        << "\n"
        << "Arguments:\n"
        << "  config_file             Path to simulation config (default: config/simulation.toml)\n"
        << "\n"
        << "Examples:\n"
        << "  AeroCore\n"
        << "  AeroCore config/fixed_wing.toml --headless --duration 30\n"
        << "  AeroCore --headless --status-rate 10 --debug-headless\n"
        << "\n"
        << "GUI controls (when not using --headless):\n"
        << "  Space     Arm / disarm          T  Takeoff\n"
        << "  L         Land                  R  Reset simulation\n"
        << "  Up/Down   Target altitude       W/A/S/D  Wind N/W/S/E\n"
        << "  1–4       Flight mode requests  Esc  Quit\n"
        << "\n"
        << "Documentation: docs/build-and-run.md, docs/capabilities-and-limitations.md\n";
    return oss.str();
}

CliArgs CliArgs::parse(int argc, char* argv[], std::string& error_out) {
    CliArgs args;
    error_out.clear();

    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);

        if (arg == "--help" || arg == "-h") {
            args.show_help = true;
            return args;
        }
        if (arg == "--headless") {
            args.headless = true;
            continue;
        }
        if (arg == "--debug-headless") {
            args.headless_debug = true;
            continue;
        }
        if (arg == "--duration") {
            if (i + 1 >= argc) {
                error_out = "--duration requires a value (seconds)";
                return args;
            }
            args.max_sim_time = std::max(0.5, std::atof(argv[++i]));
            continue;
        }
        if (arg == "--status-rate") {
            if (i + 1 >= argc) {
                error_out = "--status-rate requires a value (Hz)";
                return args;
            }
            args.headless_status_rate_hz = std::max(0.1, std::atof(argv[++i]));
            continue;
        }
        if (arg == "--no-perfect-state") {
            args.perfect_state = false;
            continue;
        }
        if (!arg.empty() && arg.front() == '-') {
            error_out = "unknown option: " + arg;
            return args;
        }

        args.config_path = arg;
    }

    return args;
}

} // namespace Utilities
} // namespace AeroCore
