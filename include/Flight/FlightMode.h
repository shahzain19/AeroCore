/**
 * @file FlightMode.h
 * @brief Flight mode enumeration for AeroCore vehicles.
 *
 * FlightMode encodes the high-level state of the flight controller's
 * finite-state machine (FSM).  Transitions are enforced by FlightController.
 *
 * ### State Diagram
 * ```
 *  DISARMED ──(arm)──► ARMED ──(takeoff)──► TAKEOFF ──(at altitude)──► STABILIZE
 *      ▲                  │                                                   │
 *      └──(disarm/crash)──┘◄───────────────(disarm)───────────────────────────┘
 *                                            │
 *                         STABILIZE ──(mode change)──► ALTITUDE_HOLD
 *                                                     ──► POSITION_HOLD
 *                                                     ──► MISSION
 *                                                     ──► RETURN_HOME
 * ```
 *
 * Modes available to both **multirotor** and **fixed-wing** vehicles are
 * marked (both).  Modes only for multirotors are marked (MR); fixed-wing (FW).
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include <string>

namespace AeroCore {
namespace Flight {

/**
 * @enum FlightMode
 * @brief High-level flight controller operating modes.
 */
enum class FlightMode {
    // ------------------------------------------------------------------
    //  Ground / pre-flight states
    // ------------------------------------------------------------------

    /// @brief Motors off, all outputs zeroed. (both)
    DISARMED = 0,

    /// @brief Motors armed (spinning at idle), ready for takeoff. (both)
    ARMED,

    // ------------------------------------------------------------------
    //  Transition states
    // ------------------------------------------------------------------

    /// @brief Controlled ascent to the configured target altitude. (MR)
    TAKEOFF,

    /// @brief Controlled deceleration and descent to the ground. (MR)
    LANDING,

    // ------------------------------------------------------------------
    //  Normal flight modes — multirotor
    // ------------------------------------------------------------------

    /**
     * @brief Rate/acro mode.
     * Pilot commands map directly to body-axis angular rates.
     * No self-levelling.  Full manual authority. (MR)
     */
    STABILIZE,

    /**
     * @brief Attitude / angle mode.
     * Pilot commands map to roll/pitch angles; yaw rate controlled.
     * Self-levelling when sticks are centred. (both)
     */
    ATTITUDE_HOLD,

    /**
     * @brief Altitude hold.
     * Altitude PID loop active; lateral attitude still pilot-controlled. (MR)
     */
    ALTITUDE_HOLD,

    /**
     * @brief Full position hold.
     * GPS position + altitude controlled; vehicle holds position
     * when sticks released. (MR)
     */
    POSITION_HOLD,

    // ------------------------------------------------------------------
    //  Normal flight modes — fixed-wing
    // ------------------------------------------------------------------

    /**
     * @brief Fly-by-wire A — attitude-stabilised manual flight. (FW)
     * Roll/pitch angles bounded; pilot controls angle setpoints.
     */
    FBW_A,

    /**
     * @brief Fly-by-wire B — altitude+speed hold. (FW)
     * Altitude hold via pitch + throttle; coordinated turns.
     */
    FBW_B,

    // ------------------------------------------------------------------
    //  Autonomous modes
    // ------------------------------------------------------------------

    /**
     * @brief Follow a pre-loaded waypoint mission. (both)
     */
    MISSION,

    /**
     * @brief Return to home position and land/loiter. (both)
     */
    RETURN_HOME,

    // ------------------------------------------------------------------
    //  Safety / emergency modes
    // ------------------------------------------------------------------

    /**
     * @brief Failsafe — triggered on RC/comms loss. (both)
     * Vehicle attempts a safe descent and landing.
     */
    FAILSAFE,

    /// @brief Emergency stop — motors cut immediately. USE WITH EXTREME CAUTION.
    EMERGENCY_STOP,

    // ------------------------------------------------------------------
    //  Count (keep last)
    // ------------------------------------------------------------------
    COUNT ///< Number of flight modes (used for array sizing).
};

// ============================================================
//  Helper functions
// ============================================================

/**
 * @brief Convert a FlightMode to a human-readable string.
 *
 * @param mode  The flight mode.
 * @return  A short uppercase string such as "DISARMED", "HOVER", etc.
 */
inline std::string flightModeToString(FlightMode mode) {
    switch (mode) {
        case FlightMode::DISARMED:       return "DISARMED";
        case FlightMode::ARMED:          return "ARMED";
        case FlightMode::TAKEOFF:        return "TAKEOFF";
        case FlightMode::LANDING:        return "LANDING";
        case FlightMode::STABILIZE:      return "STABILIZE";
        case FlightMode::ATTITUDE_HOLD:  return "ATT_HOLD";
        case FlightMode::ALTITUDE_HOLD:  return "ALT_HOLD";
        case FlightMode::POSITION_HOLD:  return "POS_HOLD";
        case FlightMode::FBW_A:          return "FBW_A";
        case FlightMode::FBW_B:          return "FBW_B";
        case FlightMode::MISSION:        return "MISSION";
        case FlightMode::RETURN_HOME:    return "RTH";
        case FlightMode::FAILSAFE:       return "FAILSAFE";
        case FlightMode::EMERGENCY_STOP: return "E-STOP";
        default:                         return "UNKNOWN";
    }
}

/**
 * @brief Return true if the vehicle is allowed to be in flight in @p mode.
 */
inline bool isFlightMode(FlightMode mode) {
    switch (mode) {
        case FlightMode::DISARMED:
        case FlightMode::ARMED:
        case FlightMode::EMERGENCY_STOP:
            return false;
        default:
            return true;
    }
}

/**
 * @brief Return true if the mode requires active GPS / position feedback.
 */
inline bool requiresGPS(FlightMode mode) {
    return mode == FlightMode::POSITION_HOLD ||
           mode == FlightMode::MISSION       ||
           mode == FlightMode::RETURN_HOME;
}

} // namespace Flight
} // namespace AeroCore
