/**
 * @file Types.h
 * @brief Shared types for hardware abstraction layer (HAL).
 *
 * HAL interfaces decouple portable flight-control logic from sim backends
 * and MCU drivers. See docs/sim-to-production.md.
 */

#pragma once

#include "Math/Vector.h"
#include <cstddef>
#include <cstdint>

namespace AeroCore {
namespace HAL {

/// RC link quality for failsafe decisions.
enum class RCLinkStatus : uint8_t {
    Connected,
    Stale,      ///< Frames arriving but checksum errors / gaps
    Lost        ///< No valid frames within timeout
};

/// GPS fix quality (aligned with common u-blox NMEA/UBX categories).
enum class GPSFixType : uint8_t {
    NoFix,
    Fix2D,
    Fix3D,
    DGNSS,
    RTK
};

/// Aggregated vehicle state produced by an estimator (single FC input).
struct VehicleState {
    Math::Quaterniond attitude{Math::Quaterniond::Identity()};
    Math::Vector3d    euler_rpy{Math::Vector3d::Zero()};   ///< roll, pitch, yaw [rad]
    Math::Vector3d    angular_rate_body{Math::Vector3d::Zero()}; ///< [rad/s]
    Math::Vector3d    position_ned{Math::Vector3d::Zero()};    ///< [m]
    Math::Vector3d    velocity_ned{Math::Vector3d::Zero()};    ///< [m/s]
    double            altitude_amsl{0.0};                        ///< [m]
    bool              attitude_valid{false};
    bool              position_valid{false};
    bool              velocity_valid{false};
};

/// Raw IMU sample at control rate.
struct IMUSample {
    Math::Vector3d accel_body{Math::Vector3d::Zero()};  ///< [m/s²] specific force
    Math::Vector3d gyro_body{Math::Vector3d::Zero()};   ///< [rad/s]
    uint64_t       timestamp_us{0};
    bool           valid{false};
};

/// Barometer sample.
struct BaroSample {
    double   pressure_pa{0.0};
    double   altitude_m{0.0};
    double   temperature_c{0.0};
    uint64_t timestamp_us{0};
    bool     valid{false};
};

/// GPS sample in NED relative to home (set on first 3D fix).
struct GPSSample {
    double      latitude_deg{0.0};
    double      longitude_deg{0.0};
    double      altitude_amsl_m{0.0};
    Math::Vector3d position_ned{Math::Vector3d::Zero()};
    Math::Vector3d velocity_ned{Math::Vector3d::Zero()};
    double      hdop{99.0};
    uint8_t     satellites{0};
    GPSFixType  fix_type{GPSFixType::NoFix};
    uint64_t    timestamp_us{0};
    bool        valid{false};
};

/// Battery sample.
struct BatterySample {
    double   voltage_v{0.0};
    double   current_a{0.0};
    double   consumed_mah{0.0};
    uint64_t timestamp_us{0};
    bool     valid{false};
};

/// Normalized RC channels (stick positions).
struct RCChannels {
    static constexpr size_t kMaxChannels = 16;

    double       channels[kMaxChannels]{};  ///< [-1, 1] for roll/pitch/yaw; throttle [0, 1]
    size_t       channel_count{0};
    RCLinkStatus link_status{RCLinkStatus::Lost};
    uint64_t     timestamp_us{0};
    bool         valid{false};

    bool armed_switch() const;   ///< AUX arm channel high (platform maps channel index)
    bool failsafe_active() const;
};

} // namespace HAL
} // namespace AeroCore
