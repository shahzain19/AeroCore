/**
 * @file Renderer.h
 * @brief SFML-based 2-D renderer and HUD for AeroCore.
 *
 * ## Layout
 * ```
 * ┌──────────────────────────────────────────────────────┐
 * │ [Altitude bar]  [Velocity/attitude arrows]            │
 * │                                                        │
 * │          [Drone body, rotors animated]                 │
 * │          [Thrust vector]  [Wind vector]                │
 * │                                                        │
 * │▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ GROUND ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓│
 * │           [HUD panel — left side]                      │
 * │           [Motor bars — bottom]                        │
 * └──────────────────────────────────────────────────────┘
 * ```
 *
 * The viewport shows a **side-on 2-D projection** of the 3-D state:
 *  - Horizontal axis = North (X in NED)
 *  - Vertical axis   = Altitude (= −Z in NED)
 *
 * The HUD panel (bottom-left) shows all telemetry in a structured layout with
 * coloured sections, fixed-width columns, and unit labels.
 *
 * ## Key Controls
 * | Key        | Action                              |
 * |------------|-------------------------------------|
 * | Space      | Arm / disarm                        |
 * | T          | Takeoff                             |
 * | L          | Land                                |
 * | R          | Reset simulation                    |
 * | ↑ / ↓      | Increase / decrease target altitude |
 * | W/A/S/D    | Wind N/W/S/E                        |
 * | 1–4        | Select flight mode                  |
 * | Esc        | Quit                                |
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include <SFML/Graphics.hpp>
#include "Math/Vector.h"
#include "Flight/Drone.h"
#include "Simulation/TelemetryManager.h"
#include <memory>
#include <string>
#include <array>

namespace AeroCore {
namespace Rendering {

/**
 * @brief Input events produced by the renderer and consumed by main().
 *
 * This decouples keyboard handling from SFML specifics — the renderer
 * processes raw SFML events and exposes a clean boolean state per action.
 */
struct InputState {
    bool quit              = false;
    bool arm_toggle        = false;  ///< Space pressed
    bool takeoff           = false;  ///< T pressed
    bool land              = false;  ///< L pressed
    bool reset             = false;  ///< R pressed
    bool alt_increase      = false;  ///< Up arrow held
    bool alt_decrease      = false;  ///< Down arrow held
    bool wind_north        = false;  ///< W held
    bool wind_south        = false;  ///< S held
    bool wind_east         = false;  ///< D held
    bool wind_west         = false;  ///< A held
    bool mode_stabilize    = false;  ///< 1 pressed
    bool mode_alt_hold     = false;  ///< 2 pressed
    bool mode_pos_hold     = false;  ///< 3 pressed
    bool mode_rth          = false;  ///< 4 pressed
};

/**
 * @brief Main SFML renderer for AeroCore.
 */
class Renderer {
public:
    /**
     * @brief Construct the render window.
     *
     * @param width   Window width [pixels].
     * @param height  Window height [pixels].
     * @param title   Window title string.
     */
    Renderer(unsigned int width, unsigned int height, const std::string& title);

    // ----------------------------------------------------------
    //  Frame control
    // ----------------------------------------------------------

    bool isOpen() const;

    /**
     * @brief Poll OS events and update InputState.
     * @return Current frame's input state.
     */
    InputState pollEvents();

    void clear();
    void display();

    // ----------------------------------------------------------
    //  Render
    // ----------------------------------------------------------

    /**
     * @brief Render one frame.
     *
     * @param drone      The vehicle to draw.
     * @param telemetry  Latest telemetry snapshot.
     * @param wind       Current wind vector [m/s] (world NED).
     */
    void render(const Flight::Drone&             drone,
                const Simulation::TelemetryData& telemetry,
                const Math::Vector3d&            wind);

    // ----------------------------------------------------------
    //  Accessors
    // ----------------------------------------------------------

    sf::RenderWindow& getWindow();

    /// Scale factor: metres in world → pixels on screen.
    float getMetersToPixels() const;

    /// Set scale factor (default: auto-fitted to window height / 30 m range).
    void  setMetersToPixels(float scale);

private:
    sf::RenderWindow window_;
    sf::Font         font_;

    float meters_to_pixels_;  ///< World→screen scale [px/m]
    float ground_y_;          ///< Screen Y of the ground line [px]
    float center_x_;          ///< Screen X of world origin (drone at reset)

    // Track which keys were held last frame (for edge-detect)
    std::array<bool, 512> key_prev_;  ///< Previous key state (edge detection)

    // ----------------------------------------------------------
    //  Scene drawing helpers
    // ----------------------------------------------------------

    void drawBackground();
    void drawGrid();
    void drawGround();
    void drawAltitudeMarkers(double max_alt_m = 50.0);
    void drawTargetAltitudeLine(double target_alt_m);

    /**
     * @brief Draw the drone body.
     * Renders the frame arms, motor circles (colour-coded by thrust), and
     * attitude indicator (tilted box when rolled/pitched).
     */
    void drawDrone(const Flight::Drone& drone,
                   const Simulation::TelemetryData& telemetry);

    /**
     * @brief Draw a vector arrow in world space.
     * @param origin_world  Arrow start point [m, NED x-z plane].
     * @param vector        Arrow direction and magnitude.
     * @param color         Arrow colour.
     * @param scale         Length scale factor [px per unit].
     * @param label         Optional text label.
     */
    void drawVector(const Math::Vector2d& origin_world,
                    const Math::Vector2d& vector,
                    sf::Color color,
                    float scale = 1.0f,
                    const std::string& label = "");

    void drawThrustVector(const Flight::Drone& drone);
    void drawVelocityVector(const Flight::Drone& drone);
    void drawWindVector(const Math::Vector3d& wind);

    /**
     * @brief Draw the full telemetry HUD panel.
     *
     * Renders a semi-transparent panel with all telemetry fields
     * formatted in coloured, aligned columns.
     */
    void drawHUD(const Simulation::TelemetryData& telemetry);

    /**
     * @brief Draw per-motor throttle bars.
     *
     * 4 vertical bars at the bottom of the screen showing each motor's
     * current throttle and RPM.
     */
    void drawMotorBars(const Simulation::TelemetryData& telemetry);

    /**
     * @brief Draw an artificial horizon indicator.
     *
     * A small ADI (Attitude Direction Indicator) in the top-right corner
     * showing roll and pitch relative to the horizon.
     */
    void drawAttitudeIndicator(const Simulation::TelemetryData& telemetry);

    /**
     * @brief Draw a pitch ladder on the attitude indicator.
     */
    void drawPitchLadder(sf::Vector2f centre, float radius, double pitch_rad, double roll_rad);

    // ----------------------------------------------------------
    //  Coordinate helpers
    // ----------------------------------------------------------

    /**
     * @brief Convert a 2-D world point (NED x-z plane) to screen coordinates.
     *
     * @param world_xz  World point: x = North [m], y = −altitude (NED z).
     * @return Screen coordinates [px].
     */
    sf::Vector2f worldToScreen(const Math::Vector2d& world_xz) const;

    // ----------------------------------------------------------
    //  Text helpers
    // ----------------------------------------------------------

    /**
     * @brief Draw a text string at a given screen position.
     *
     * @param text      String to draw.
     * @param x, y      Screen position [px].
     * @param color     Text colour.
     * @param size      Character size [px].
     */
    void drawText(const std::string& text,
                  float x, float y,
                  sf::Color color = sf::Color::White,
                  unsigned int size = 13);

    /**
     * @brief Draw a filled rectangle (used for HUD backgrounds and bars).
     */
    void drawRect(float x, float y, float w, float h,
                  sf::Color fill,
                  sf::Color outline = sf::Color::Transparent,
                  float outline_thickness = 0.0f);
};

} // namespace Rendering
} // namespace AeroCore
