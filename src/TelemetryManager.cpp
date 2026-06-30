/**
 * @file TelemetryManager.cpp
 * @brief Telemetry aggregation and HUD string formatting.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Simulation/TelemetryManager.h"
#include "Flight/FlightMode.h"
#include <cmath>

namespace AeroCore {
namespace Simulation {

TelemetryManager::TelemetryManager()
    : fps_accumulator_(0.0)
    , fps_frame_count_(0)
    , current_fps_(0.0)
{}

// ============================================================
//  Update
// ============================================================

void TelemetryManager::update(double frame_dt, const TelemetryData& data) {
    current_data_ = data;

    // Rolling FPS average over FPS_WINDOW frames
    fps_accumulator_ += frame_dt;
    ++fps_frame_count_;

    if (fps_accumulator_ >= 1.0) {
        current_fps_       = fps_frame_count_ / fps_accumulator_;
        fps_accumulator_   = 0.0;
        fps_frame_count_   = 0;
    }

    current_data_.fps = current_fps_;
}

// ============================================================
//  Accessors
// ============================================================

double TelemetryManager::getFPS()     const { return current_fps_; }
double TelemetryManager::getSimTime() const { return current_data_.simulation_time; }
const TelemetryData& TelemetryManager::getCurrentData() const { return current_data_; }

// ============================================================
//  Formatting helpers
// ============================================================

std::string TelemetryManager::sectionHeader(const std::string& title) const {
    std::string line = "-- " + title + " ";
    while (line.size() < 44) line += '-';
    return line + "\n";
}

std::string TelemetryManager::hudLine(const std::string& key,
                                       double value,
                                       int precision,
                                       const std::string& unit) const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    // Left-align key in 18 chars, right-align value in 8 chars
    oss << "  " << std::left << std::setw(18) << key
        << std::right << std::setw(8) << value;
    if (!unit.empty()) oss << " " << unit;
    oss << "\n";
    return oss.str();
}

std::string TelemetryManager::hudLine(const std::string& key,
                                       const std::string& value) const {
    std::ostringstream oss;
    oss << "  " << std::left << std::setw(18) << key
        << std::right << std::setw(8) << value << "\n";
    return oss.str();
}

// ============================================================
//  Full HUD string
// ============================================================

std::string TelemetryManager::getHUDString() const {
    const auto& d = current_data_;
    std::ostringstream oss;

    // ── Header ──────────────────────────────────────
    oss << "+==========================================+\n";
    oss << "|        AEROCORE FLIGHT COMPUTER          |\n";
    oss << "+==========================================+\n";

    // ── Performance ─────────────────────────────────
    oss << sectionHeader("PERFORMANCE");
    oss << hudLine("FPS",        d.fps,             1, "hz");
    oss << hudLine("Sim Time",   d.simulation_time, 2, "s ");
    oss << hudLine("Air Density",d.air_density,     4, "kg/m³");

    // ── Flight State ────────────────────────────────
    oss << sectionHeader("FLIGHT STATE");
    oss << hudLine("Mode",      Flight::flightModeToString(d.flight_mode));
    oss << hudLine("Altitude",  d.altitude,                2, "m ");
    oss << hudLine("Target Alt",d.target_altitude,         2, "m ");

    // Velocity
    const double spd_xy = std::sqrt(d.velocity.x()*d.velocity.x() +
                                     d.velocity.y()*d.velocity.y());
    oss << hudLine("Speed (H)",  spd_xy,                    2, "m/s");
    oss << hudLine("Speed (V)",  -d.velocity.z(),           2, "m/s");  // NED

    // Position
    oss << hudLine("Pos North",  d.position.x(),            2, "m ");
    oss << hudLine("Pos East",   d.position.y(),            2, "m ");

    // ── Attitude ────────────────────────────────────
    oss << sectionHeader("ATTITUDE");
    const double roll_deg  = d.euler_angles.x() * Math::RAD2DEG;
    const double pitch_deg = d.euler_angles.y() * Math::RAD2DEG;
    const double yaw_deg   = d.euler_angles.z() * Math::RAD2DEG;
    oss << hudLine("Roll",       roll_deg,                   1, "deg");
    oss << hudLine("Pitch",      pitch_deg,                  1, "deg");
    oss << hudLine("Yaw",        yaw_deg,                    1, "deg");
    oss << hudLine("Roll Rate",  d.angular_velocity.x() * Math::RAD2DEG, 1, "°/s");
    oss << hudLine("Pitch Rate", d.angular_velocity.y() * Math::RAD2DEG, 1, "°/s");
    oss << hudLine("Yaw Rate",   d.angular_velocity.z() * Math::RAD2DEG, 1, "°/s");

    // ── Thrust ──────────────────────────────────────
    oss << sectionHeader("PROPULSION");
    oss << hudLine("Thrust",     d.current_thrust,           1, "N ");
    for (int i = 0; i < 4; ++i) {
        const std::string label = "M" + std::to_string(i) + " Throttle";
        oss << hudLine(label,    d.motor_throttle[i] * 100.0, 1, "% ");
        const std::string rlabel = "M" + std::to_string(i) + " RPM";
        oss << hudLine(rlabel,   d.motor_rpm[i],              0, "  ");
    }

    // ── Altitude PID ────────────────────────────────
    oss << sectionHeader("ALT PID");
    oss << hudLine("Error",      d.pid_alt_error,            3, "m ");
    oss << hudLine("Integral",   d.pid_alt_integral,         4, "  ");
    oss << hudLine("Derivative", d.pid_alt_derivative,       4, "  ");
    oss << hudLine("Output",     d.pid_alt_output,           4, "  ");

    // ── Rate PIDs ───────────────────────────────────
    oss << sectionHeader("RATE PIDs");
    oss << hudLine("Roll Out",   d.pid_roll_output,          4, "  ");
    oss << hudLine("Pitch Out",  d.pid_pitch_output,         4, "  ");
    oss << hudLine("Yaw Out",    d.pid_yaw_output,           4, "  ");

    // ── Sensors ─────────────────────────────────────
    oss << sectionHeader("SENSORS");
    oss << hudLine("Accel X",    d.accel_sensor.x(),         3, "m/s²");
    oss << hudLine("Accel Y",    d.accel_sensor.y(),         3, "m/s²");
    oss << hudLine("Accel Z",    d.accel_sensor.z(),         3, "m/s²");
    oss << hudLine("Gyro P",     d.gyro_sensor.x() * Math::RAD2DEG, 2, "°/s");
    oss << hudLine("Gyro Q",     d.gyro_sensor.y() * Math::RAD2DEG, 2, "°/s");
    oss << hudLine("Gyro R",     d.gyro_sensor.z() * Math::RAD2DEG, 2, "°/s");
    oss << hudLine("Altimeter",  d.alt_sensor,               2, "m ");

    // ── Battery ─────────────────────────────────────
    oss << sectionHeader("BATTERY");
    oss << hudLine("Voltage",    d.battery_voltage,          2, "V ");
    oss << hudLine("SOC",        d.battery_soc,              1, "% ");
    oss << hudLine("Current",    d.total_current,            2, "A ");

    // ── Wind ────────────────────────────────────────
    oss << sectionHeader("ENVIRONMENT");
    oss << hudLine("Wind N",     d.wind_world.x(),           2, "m/s");
    oss << hudLine("Wind E",     d.wind_world.y(),           2, "m/s");
    oss << hudLine("Wind D",     d.wind_world.z(),           2, "m/s");

    return oss.str();
}

// ============================================================
//  One-line status
// ============================================================

std::string TelemetryManager::getStatusLine() const {
    const auto& d = current_data_;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "T=" << d.simulation_time
        << "s | Alt=" << d.altitude
        << "m | Thr=" << d.current_thrust
        << "N | Mode=" << Flight::flightModeToString(d.flight_mode)
        << " | Batt=" << d.battery_soc
        << "% | FPS=" << static_cast<int>(d.fps);
    return oss.str();
}

} // namespace Simulation
} // namespace AeroCore
