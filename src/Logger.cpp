/**
 * @file Logger.cpp
 * @brief Thread-safe singleton logger implementation.
 *
 * Writes timestamped, levelled log messages to both a file and stdout
 * (INFO+WARNING → stdout, ERROR → stderr).
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Utilities/Logger.h"
#include <filesystem>
#include <iostream>

namespace AeroCore {
namespace Utilities {

// ============================================================
//  Singleton construction / destruction
// ============================================================

Logger::Logger() {
    // Ensure the logs/ directory exists
    std::filesystem::create_directories("logs");

    // Open the log file in append mode so successive runs accumulate
    file_.open("logs/flight_log.txt", std::ios::out | std::ios::app);
    if (!file_.is_open()) {
        // Don't throw — we still want the app to run without a log file
        std::cerr << "[Logger] WARNING: Could not open logs/flight_log.txt\n";
    }
}

Logger::~Logger() {
    if (file_.is_open()) {
        file_.close();
    }
}

Logger& Logger::getInstance() {
    static Logger instance;
    return instance;
}

// ============================================================
//  Internal helpers
// ============================================================

std::string Logger::getTimestamp() const {
    const auto now     = std::chrono::system_clock::now();
    const auto time_t  = std::chrono::system_clock::to_time_t(now);
    const auto ms      = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now.time_since_epoch()) % 1000;

    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    oss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

std::string Logger::levelToString(LogLevel level) const {
    switch (level) {
        case LogLevel::DEBUG:   return "DEBUG  ";
        case LogLevel::INFO:    return "INFO   ";
        case LogLevel::WARNING: return "WARNING";
        case LogLevel::ERROR:   return "ERROR  ";
        default:                return "UNKNOWN";
    }
}

// ============================================================
//  Core log method
// ============================================================

void Logger::log(LogLevel level, const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);

    const std::string entry = "[" + getTimestamp() + "] [" +
                              levelToString(level) + "] " + message;

    // Write to file
    if (file_.is_open()) {
        file_ << entry << '\n';
        file_.flush();  // Flush so logs are not lost on crash
    }

    // Echo to console
    if (level == LogLevel::ERROR) {
        std::cerr << entry << '\n';
    } else {
        std::cout << entry << '\n';
    }
}

// ============================================================
//  Convenience methods
// ============================================================

void Logger::debug(const std::string& message)   { log(LogLevel::DEBUG,   message); }
void Logger::info(const std::string& message)    { log(LogLevel::INFO,    message); }
void Logger::warning(const std::string& message) { log(LogLevel::WARNING, message); }
void Logger::error(const std::string& message)   { log(LogLevel::ERROR,   message); }

} // namespace Utilities
} // namespace AeroCore
