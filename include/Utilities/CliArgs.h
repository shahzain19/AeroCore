/**
 * @file CliArgs.h
 * @brief Command-line argument parsing for the AeroCore executable.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include <string>

namespace AeroCore {
namespace Utilities {

/**
 * @brief Parsed runtime options for AeroCore.
 */
struct CliArgs {
    bool        show_help              = false;
    bool        headless               = false;
    bool        headless_debug         = false;
    std::string config_path            = "config/simulation.toml";
    double      max_sim_time           = 60.0;
    double      headless_status_rate_hz = 5.0;

    /**
     * @brief Parse argc/argv into a CliArgs structure.
     *
     * Unknown options starting with '-' set @p error_out and return a default
     * CliArgs.  Missing values for options that require arguments also set
     * @p error_out.
     */
    static CliArgs parse(int argc, char* argv[], std::string& error_out);

    /** @brief Full help text suitable for --help output. */
    static std::string helpText();
};

} // namespace Utilities
} // namespace AeroCore
