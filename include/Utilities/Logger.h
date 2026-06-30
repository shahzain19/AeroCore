/**
 * @file Logger.h
 * @brief Thread-safe singleton logger for AeroCore.
 *
 * ## Features
 *  - Thread-safe (std::mutex protected)
 *  - Singleton pattern — one logger per process
 *  - Timestamped entries to file (logs/flight_log.txt)
 *  - Echo to stdout/stderr for real-time console output
 *  - Four log levels: DEBUG, INFO, WARNING, ERROR
 *
 * ## Usage
 * ```cpp
 * auto& log = AeroCore::Utilities::Logger::getInstance();
 * log.info("Takeoff initiated");
 * log.warning("Low battery: " + std::to_string(v) + " V");
 * log.error("Sensor failure: IMU not responding");
 * ```
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include <string>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>

namespace AeroCore {
namespace Utilities {

/**
 * @brief Log severity levels.
 */
enum class LogLevel {
    DEBUG,    ///< Verbose developer info (disabled in release builds)
    INFO,     ///< Normal operational information
    WARNING,  ///< Non-fatal anomaly
    ERROR     ///< Serious error — written to stderr as well
};

/**
 * @brief Thread-safe singleton logger.
 */
class Logger {
public:
    // ----------------------------------------------------------
    //  Singleton access
    // ----------------------------------------------------------

    /// Returns the single global Logger instance.
    static Logger& getInstance();

    // Non-copyable, non-movable
    Logger(const Logger&)            = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&)                 = delete;
    Logger& operator=(Logger&&)      = delete;

    // ----------------------------------------------------------
    //  Logging API
    // ----------------------------------------------------------

    /// Log a message at the given level.
    void log(LogLevel level, const std::string& message);

    /// Convenience wrappers for each level.
    void debug(const std::string& message);
    void info(const std::string& message);
    void warning(const std::string& message);
    void error(const std::string& message);

private:
    Logger();
    ~Logger();

    std::ofstream    file_;   ///< Output file stream
    std::mutex       mutex_;  ///< Guards concurrent access

    /// Format current time as "YYYY-MM-DD HH:MM:SS.mmm".
    std::string getTimestamp() const;

    /// Convert LogLevel enum to a fixed-width string.
    std::string levelToString(LogLevel level) const;
};

} // namespace Utilities
} // namespace AeroCore
