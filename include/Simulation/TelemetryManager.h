/**
 * @file TelemetryManager.h
 * @brief Real-time telemetry aggregation and formatting for AeroCore.
 *
 * ## Overview
 * TelemetryManager collects data from the physics engine, sensors, and flight
 * controller each simulation tick, computes rolling statistics (FPS, update
 * rates), and formats everything into a display string for the HUD renderer.
 *
 * ## Data Flow
 * ```
 *  Drone / PhysicsEngine ──► TelemetryData struct ──► TelemetryManager::update()
 *                                                           │
 *                             ┌─────────────────────────────┘
 *                             ▼
 *                      formatted HUD text ──► Renderer::drawHUD()
 * ```
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include "Math/Vector.h"
#include "Flight/FlightMode.h"
#include <string>
#include <sstream>
#include <iomanip>
#include <deque>

namespace AeroCore {
namespace Simulation {

// ============================================================
//  TelemetryData — plain data structure for one telemetry snapshot
// ============================================================

/**
 * @brief Complete telemetry snapshot for one simulation tick.
 *
 * All fields are in SI units unless noted.
 */
struct TelemetryData {
    // Timing
    double simulation_time = 0.0;   ///< Elapsed sim time [s]
    double fps             = 0.0;   ///< Rendering frame rate [Hz]
    double physics_rate    = 0.0;   ///< Physics update rate [Hz]

    // Position / kinematics (NED world frame)
    Math::Vector3d position;        ///< Position [m]
    Math::Vector3d velocity;        ///< Velocity [m/s]
    Math::Vector3d acceleration;    ///< Acceleration [m/s²]
    double altitude = 0.0;          ///< Altitude above ground [m]

    // Attitude
    Math::Vector3d euler_angles;    ///< (roll, pitch, yaw) [rad]
    Math::Vector3d angular_velocity; ///< Body-frame ω [rad/s]

    // Flight controller
    double target_altitude  = 0.0;  ///< Current target altitude [m]
    double current_thrust   = 0.0;  ///< Total thrust magnitude [N]
    Flight::FlightMode flight_mode = Flight::FlightMode::DISARMED;

    // Altitude PID
    double pid_alt_output     = 0.0;
    double pid_alt_error      = 0.0;
    double pid_alt_integral   = 0.0;
    double pid_alt_derivative = 0.0;

    // Roll/Pitch/Yaw rate PIDs (for diagnostics)
    double pid_roll_output    = 0.0;
    double pid_pitch_output   = 0.0;
    double pid_yaw_output     = 0.0;

    // Sensor readings
    Math::Vector3d accel_sensor;    ///< IMU accelerometer [m/s²]
    Math::Vector3d gyro_sensor;     ///< IMU gyroscope [rad/s]
    double alt_sensor      = 0.0;   ///< Altimeter reading [m]
    double battery_voltage = 0.0;   ///< Battery terminal voltage [V]
    double battery_soc     = 0.0;   ///< Battery state of charge [%]
    double total_current   = 0.0;   ///< Total current draw [A]

    // Per-motor data (up to 8 motors)
    double motor_throttle[8] = {};   ///< Motor throttle commands [0–1]
    double motor_rpm[8]      = {};   ///< Motor RPM

    // Environment
    Math::Vector3d wind_world;      ///< Wind velocity in world frame [m/s]
    double air_density = 1.225;     ///< Current air density [kg/m³]
};

// ============================================================
//  TelemetryManager
// ============================================================

/**
 * @brief Aggregates telemetry and formats it for display.
 */
class TelemetryManager {
public:
    TelemetryManager();

    // ----------------------------------------------------------
    //  Update
    // ----------------------------------------------------------

    /**
     * @brief Record a new telemetry snapshot.
     *
     * Call once per render frame (not per physics tick).
     *
     * @param frame_dt  Real elapsed time since last frame [s].
     * @param data      Latest telemetry data.
     */
    void update(double frame_dt, const TelemetryData& data);

    // ----------------------------------------------------------
    //  Formatted output
    // ----------------------------------------------------------

    /**
     * @brief Generate the full HUD display string.
     *
     * Returns a multi-line string with sections:
     *   - Vehicle state (position, velocity, attitude)
     *   - Flight controller (mode, PID diagnostics)
     *   - Sensor readings
     *   - Battery
     *   - Performance (FPS, rates)
     */
    std::string getHUDString() const;

    /**
     * @brief Generate a compact one-line status string for console output.
     */
    std::string getStatusLine() const;

    // ----------------------------------------------------------
    //  Accessors
    // ----------------------------------------------------------

    double getFPS()          const;
    double getSimTime()      const;
    const TelemetryData& getCurrentData() const;

private:
    TelemetryData current_data_;

    // FPS tracking
    double fps_accumulator_;
    int    fps_frame_count_;
    double current_fps_;

    // Rolling window for physics rate measurement
    std::deque<double> recent_frame_times_;
    static constexpr int FPS_WINDOW = 60;

    /// Format a section header for the HUD string.
    std::string sectionHeader(const std::string& title) const;

    /// Format a key=value pair for the HUD.
    std::string hudLine(const std::string& key,
                        double value,
                        int precision = 2,
                        const std::string& unit = "") const;

    std::string hudLine(const std::string& key,
                        const std::string& value) const;
};

} // namespace Simulation
} // namespace AeroCore
