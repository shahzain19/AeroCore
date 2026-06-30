/**
 * @file Renderer.cpp
 * @brief SFML 2-D renderer and HUD for AeroCore.
 *
 * Coordinate system:
 *   World: NED x-z plane projected side-on.
 *     world_x (North) → screen_x (right is positive)
 *     altitude = -world_z → screen_y (up is positive, converted to screen-down)
 *
 *   worldToScreen(xz):
 *     screen_x = center_x + world_x  * scale
 *     screen_y = ground_y - altitude * scale
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#include "Rendering/Renderer.h"
#include "Math/Vector.h"
#include "Flight/FlightMode.h"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <array>

namespace AeroCore {
namespace Rendering {

// ============================================================
//  Construction
// ============================================================

Renderer::Renderer(unsigned int width, unsigned int height, const std::string& title)
    : window_(sf::VideoMode(sf::Vector2u(width, height)), title)
    , meters_to_pixels_(static_cast<float>(height - 150) / 50.0f)  // 50 m visible range
    , ground_y_(static_cast<float>(height) - 100.0f)
    , center_x_(static_cast<float>(width) / 2.0f)
{
    window_.setFramerateLimit(60);
    key_prev_.fill(false);

    // Try several common font paths
    const std::array<std::string, 5> font_paths = {
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
        "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
        "/Library/Fonts/Courier New.ttf",
        "assets/font.ttf"
    };
    for (const auto& p : font_paths) {
        if (font_.openFromFile(p)) break;
    }
}

// ============================================================
//  Frame control
// ============================================================

bool Renderer::isOpen() const { return window_.isOpen(); }

InputState Renderer::pollEvents() {
    InputState input;

    // Poll OS events (close, resize, etc.)
    std::optional<sf::Event> ev;
    while ((ev = window_.pollEvent())) {
        if (ev->is<sf::Event::Closed>()) {
            window_.close();
            input.quit = true;
        }
    }

    // Edge-detect helper (true only on the frame a key is pressed)
    auto edge = [&](sf::Keyboard::Key k) -> bool {
        const auto idx = static_cast<size_t>(k);
        if (idx >= key_prev_.size()) return false;
        const bool cur = sf::Keyboard::isKeyPressed(k);
        const bool was = key_prev_[idx];
        key_prev_[idx] = cur;
        return cur && !was;
    };

    // Hold helper (true while key is held)
    auto held = [&](sf::Keyboard::Key k) -> bool {
        return sf::Keyboard::isKeyPressed(k);
    };

    if (edge(sf::Keyboard::Key::Escape)) { window_.close(); input.quit = true; }
    input.arm_toggle    = edge(sf::Keyboard::Key::Space);
    input.takeoff       = edge(sf::Keyboard::Key::T);
    input.land          = edge(sf::Keyboard::Key::L);
    input.reset         = edge(sf::Keyboard::Key::R);
    input.alt_increase  = held(sf::Keyboard::Key::Up);
    input.alt_decrease  = held(sf::Keyboard::Key::Down);
    input.wind_north    = held(sf::Keyboard::Key::W);
    input.wind_south    = held(sf::Keyboard::Key::S);
    input.wind_east     = held(sf::Keyboard::Key::D);
    input.wind_west     = held(sf::Keyboard::Key::A);
    input.mode_stabilize = edge(sf::Keyboard::Key::Num1);
    input.mode_alt_hold  = edge(sf::Keyboard::Key::Num2);
    input.mode_pos_hold  = edge(sf::Keyboard::Key::Num3);
    input.mode_rth       = edge(sf::Keyboard::Key::Num4);

    return input;
}

void Renderer::clear()   { window_.clear(sf::Color(18, 22, 36)); }
void Renderer::display() { window_.display(); }
sf::RenderWindow& Renderer::getWindow() { return window_; }
float Renderer::getMetersToPixels() const { return meters_to_pixels_; }
void  Renderer::setMetersToPixels(float s) { meters_to_pixels_ = s; }

// ============================================================
//  Coordinate helpers
// ============================================================

sf::Vector2f Renderer::worldToScreen(const Math::Vector2d& world_xz) const {
    // world_xz.x = North (x), world_xz.y = altitude (= -NED_z)
    const float sx = center_x_ + static_cast<float>(world_xz.x()) * meters_to_pixels_;
    const float sy = ground_y_ - static_cast<float>(world_xz.y()) * meters_to_pixels_;
    return {sx, sy};
}

// ============================================================
//  Text / rect helpers
// ============================================================

void Renderer::drawText(const std::string& text, float x, float y,
                         sf::Color color, unsigned int size) {
    sf::Text t(font_);
    t.setCharacterSize(size);
    t.setFillColor(color);
    t.setString(text);
    t.setPosition({x, y});
    window_.draw(t);
}

void Renderer::drawRect(float x, float y, float w, float h,
                         sf::Color fill, sf::Color outline, float thick) {
    sf::RectangleShape r({w, h});
    r.setFillColor(fill);
    r.setOutlineColor(outline);
    r.setOutlineThickness(thick);
    r.setPosition({x, y});
    window_.draw(r);
}

// ============================================================
//  Scene elements
// ============================================================

void Renderer::drawBackground() {
    // Sky gradient: dark blue at top → lighter toward horizon
    const auto W = static_cast<float>(window_.getSize().x);
    sf::VertexArray sky(sf::PrimitiveType::TriangleFan, 4);
    sky[0] = sf::Vertex({0.f,     0.f},      sf::Color(10, 14, 30));
    sky[1] = sf::Vertex({W,       0.f},      sf::Color(10, 14, 30));
    sky[2] = sf::Vertex({W,       ground_y_}, sf::Color(30, 45, 80));
    sky[3] = sf::Vertex({0.f,     ground_y_}, sf::Color(30, 45, 80));
    window_.draw(sky);
}

void Renderer::drawGrid() {
    const sf::Color col(50, 60, 90, 80);
    const float W = static_cast<float>(window_.getSize().x);

    // Vertical lines every 5 m (world x)
    for (float wx = -100.f; wx <= 100.f; wx += 5.f) {
        const float sx = center_x_ + wx * meters_to_pixels_;
        sf::Vertex line[] = {
            sf::Vertex({sx, 0.f},      col),
            sf::Vertex({sx, ground_y_}, col)
        };
        window_.draw(line, 2, sf::PrimitiveType::Lines);
    }

    // Horizontal lines every 5 m altitude
    for (int alt = 0; alt <= 60; alt += 5) {
        const float sy = ground_y_ - alt * meters_to_pixels_;
        if (sy < 0) break;
        sf::Vertex line[] = {
            sf::Vertex({0.f, sy}, col),
            sf::Vertex({W,   sy}, col)
        };
        window_.draw(line, 2, sf::PrimitiveType::Lines);
    }
}

void Renderer::drawGround() {
    const float W = static_cast<float>(window_.getSize().x);
    // Ground fill
    drawRect(0, ground_y_, W, 100.f, sf::Color(45, 28, 10));
    // Grass strip
    drawRect(0, ground_y_, W, 6.f,   sf::Color(34, 110, 34));
}

void Renderer::drawAltitudeMarkers(double max_alt_m) {
    for (int alt = 0; alt <= static_cast<int>(max_alt_m); alt += 5) {
        const float sy = ground_y_ - alt * meters_to_pixels_;
        if (sy < 0) break;

        // Label
        const sf::Color lc = (alt % 10 == 0) ? sf::Color(200, 200, 220)
                                              : sf::Color(120, 120, 140);
        drawText(std::to_string(alt) + "m", 4.f, sy - 14.f, lc, 11);
    }
}

void Renderer::drawTargetAltitudeLine(double target_alt_m) {
    const float sy = ground_y_ - static_cast<float>(target_alt_m) * meters_to_pixels_;
    const float W  = static_cast<float>(window_.getSize().x);
    sf::Vertex line[] = {
        sf::Vertex({0.f, sy}, sf::Color(255, 200, 0, 160)),
        sf::Vertex({W,   sy}, sf::Color(255, 200, 0, 160))
    };
    window_.draw(line, 2, sf::PrimitiveType::Lines);
    drawText("TARGET", W - 70.f, sy - 15.f, sf::Color(255, 200, 0), 11);
}

// ============================================================
//  Drone drawing
// ============================================================

void Renderer::drawDrone(const Flight::Drone& drone,
                          const Simulation::TelemetryData& telemetry) {
    const double alt     = drone.getAltitude();
    const double north   = drone.getPosition().x();
    const auto euler     = drone.getEulerAngles();
    const float roll_deg = static_cast<float>(euler.x() * Math::RAD2DEG);

    // Project world NED x-z plane → screen
    const Math::Vector2d world_pos(north, alt);
    const sf::Vector2f   screen_pos = worldToScreen(world_pos);
    const float size_px = static_cast<float>(drone.getSize()) * meters_to_pixels_;
    const float arm_px  = static_cast<float>(drone.getArmLength()) * meters_to_pixels_;

    // Draw each motor arm + motor disc
    const float arm_angles_deg[4] = {45.f, 135.f, 225.f, 315.f};
    const sf::Color motor_colors[4] = {
        sf::Color(255, 80,  80),   // M0 CW  — red
        sf::Color(80,  200, 255),  // M1 CCW — cyan
        sf::Color(255, 80,  80),   // M2 CW  — red
        sf::Color(80,  200, 255),  // M3 CCW — cyan
    };

    for (int i = 0; i < 4 && i < static_cast<int>(drone.getMotorCount()); ++i) {
        const float ang_rad = (arm_angles_deg[i] + roll_deg) * static_cast<float>(Math::DEG2RAD);
        const float mx = screen_pos.x + arm_px * std::cos(ang_rad);
        const float my = screen_pos.y + arm_px * std::sin(ang_rad);

        // Arm line
        sf::Vertex arm[] = {
            sf::Vertex(screen_pos, sf::Color(130, 130, 160)),
            sf::Vertex({mx, my},   sf::Color(130, 130, 160))
        };
        window_.draw(arm, 2, sf::PrimitiveType::Lines);

        // Motor disc — brightness indicates throttle
        const float thr = static_cast<float>(telemetry.motor_throttle[i]);
        const float r   = size_px * 0.18f;
        sf::CircleShape disc(r);
        disc.setOrigin({r, r});
        disc.setPosition({mx, my});
        // Tint: dim at zero throttle, bright at full
        const sf::Color base = motor_colors[i];
        const sf::Color tinted(
            static_cast<uint8_t>(base.r * (0.3f + 0.7f * thr)),
            static_cast<uint8_t>(base.g * (0.3f + 0.7f * thr)),
            static_cast<uint8_t>(base.b * (0.3f + 0.7f * thr))
        );
        disc.setFillColor(tinted);
        disc.setOutlineColor(sf::Color(200, 200, 200, 100));
        disc.setOutlineThickness(1.0f);
        window_.draw(disc);
    }

    // Central body box (tilted by roll for visual feedback)
    sf::RectangleShape body({size_px, size_px * 0.28f});
    body.setFillColor(sf::Color(60, 130, 220));
    body.setOutlineColor(sf::Color(120, 180, 255));
    body.setOutlineThickness(1.5f);
    body.setOrigin({size_px / 2.f, size_px * 0.14f});
    body.setPosition(screen_pos);
    body.setRotation(sf::degrees(roll_deg));
    window_.draw(body);

    // Nose indicator dot (front = +North = right when looking side-on)
    sf::CircleShape nose(3.f);
    nose.setFillColor(sf::Color(255, 255, 100));
    nose.setOrigin({3.f, 3.f});
    nose.setPosition({screen_pos.x + size_px * 0.4f, screen_pos.y});
    window_.draw(nose);
}

void Renderer::drawThrustVector(const Flight::Drone& drone) {
    const double north = drone.getPosition().x();
    const double alt   = drone.getAltitude();
    const auto euler   = drone.getEulerAngles();

    // Total thrust magnitude
    double total_T = 0.0;
    for (size_t i = 0; i < drone.getMotorCount(); ++i)
        total_T += drone.getMotor(i).getCurrentThrust();

    if (total_T < 0.5) return;

    // Thrust direction in screen: rotated by roll
    const float roll = static_cast<float>(euler.x());
    const float scale = 0.015f * meters_to_pixels_;
    const Math::Vector2d origin(north, alt);
    // In side view, thrust is vertical (up) tilted by roll
    const Math::Vector2d vec(
        static_cast<double>(total_T * std::sin(roll) * scale / meters_to_pixels_),
        static_cast<double>(total_T * std::cos(roll) * scale / meters_to_pixels_)
    );
    drawVector(origin, vec, sf::Color(255, 80, 200), meters_to_pixels_, "T");
}

void Renderer::drawVelocityVector(const Flight::Drone& drone) {
    const double north = drone.getPosition().x();
    const double alt   = drone.getAltitude();
    const auto   vel   = drone.getVelocity();
    const double speed = vel.norm();
    if (speed < 0.1) return;

    // Velocity projected to x-z (north-altitude) plane
    const Math::Vector2d origin(north, alt);
    const Math::Vector2d vec(vel.x(), -vel.z());  // altitude = -NED_z
    drawVector(origin, vec, sf::Color(100, 255, 100), 2.0f * meters_to_pixels_, "V");
}

void Renderer::drawWindVector(const Math::Vector3d& wind) {
    if (wind.head<2>().norm() < 0.1) return;
    // Display wind arrow in upper-left of screen
    const float wx = 80.f;
    const float wy = 80.f;
    const float scale = 12.f;
    sf::Vertex line[] = {
        sf::Vertex({wx, wy}, sf::Color(0, 220, 220)),
        sf::Vertex({wx + static_cast<float>(wind.x()) * scale,
                    wy - static_cast<float>(wind.z()) * scale},
                   sf::Color(0, 220, 220))
    };
    window_.draw(line, 2, sf::PrimitiveType::Lines);
    drawText("WIND", wx - 30.f, wy - 18.f, sf::Color(0, 220, 220), 11);
}

void Renderer::drawVector(const Math::Vector2d& origin_world,
                           const Math::Vector2d& vector,
                           sf::Color color,
                           float scale,
                           const std::string& label) {
    const sf::Vector2f start = worldToScreen(origin_world);
    const sf::Vector2f end = {
        start.x + static_cast<float>(vector.x()) * scale,
        start.y - static_cast<float>(vector.y()) * scale
    };
    sf::Vertex line[] = {
        sf::Vertex(start, color),
        sf::Vertex(end,   color)
    };
    window_.draw(line, 2, sf::PrimitiveType::Lines);
    if (!label.empty()) {
        drawText(label, end.x + 4.f, end.y - 6.f, color, 11);
    }
}

// ============================================================
//  HUD panel
// ============================================================

void Renderer::drawHUD(const Simulation::TelemetryData& d) {
    // Semi-transparent background panel on the right side
    const float W = static_cast<float>(window_.getSize().x);
    const float H = static_cast<float>(window_.getSize().y);
    const float panel_w = 260.f;
    const float panel_x = W - panel_w - 4.f;
    const float panel_h = H - 8.f;

    drawRect(panel_x, 4.f, panel_w, panel_h,
             sf::Color(0, 0, 0, 170),
             sf::Color(60, 80, 120, 200), 1.0f);

    float y = 12.f;
    const float x = panel_x + 8.f;
    const float lh = 16.f;  // line height

    auto line = [&](const std::string& key, double val, int prec,
                    const std::string& unit, sf::Color vc = sf::Color(200, 220, 255)) {
        drawText(key, x, y, sf::Color(140, 160, 190), 11);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(prec) << val << " " << unit;
        drawText(oss.str(), x + 130.f, y, vc, 11);
        y += lh;
    };

    auto header = [&](const std::string& title) {
        y += 4.f;
        drawRect(x - 4.f, y, panel_w - 8.f, 14.f,
                 sf::Color(40, 60, 100, 180));
        drawText(title, x, y, sf::Color(180, 210, 255), 11);
        y += lh + 2.f;
    };

    // Determine mode colour
    sf::Color mode_col = sf::Color(100, 255, 100);
    if (d.flight_mode == Flight::FlightMode::FAILSAFE ||
        d.flight_mode == Flight::FlightMode::EMERGENCY_STOP)
        mode_col = sf::Color(255, 80, 80);
    else if (d.flight_mode == Flight::FlightMode::DISARMED ||
             d.flight_mode == Flight::FlightMode::ARMED)
        mode_col = sf::Color(255, 200, 80);

    // Title
    drawText("AEROCORE", x, y, sf::Color(100, 200, 255), 13);
    y += lh + 4.f;
    drawText(Flight::flightModeToString(d.flight_mode), x, y, mode_col, 13);
    y += lh + 4.f;

    // FPS
    drawText("FPS", x, y, sf::Color(140, 160, 190), 11);
    drawText(std::to_string(static_cast<int>(d.fps)), x + 130.f, y,
             sf::Color(200, 220, 255), 11);
    y += lh;

    header("── POSITION");
    line("Altitude",    d.altitude,         2, "m");
    line("Target Alt",  d.target_altitude,  2, "m",  sf::Color(255, 220, 80));
    line("North",       d.position.x(),     2, "m");
    line("East",        d.position.y(),     2, "m");

    const double spd = std::sqrt(d.velocity.x()*d.velocity.x() +
                                  d.velocity.y()*d.velocity.y());
    line("Horiz Speed", spd,                2, "m/s");
    line("Vert Speed",  -d.velocity.z(),    2, "m/s");

    header("── ATTITUDE");
    line("Roll",        d.euler_angles.x() * Math::RAD2DEG, 1, "°");
    line("Pitch",       d.euler_angles.y() * Math::RAD2DEG, 1, "°");
    line("Yaw",         d.euler_angles.z() * Math::RAD2DEG, 1, "°");

    header("── ALT PID");
    line("Error",       d.pid_alt_error,     3, "m");
    line("Output",      d.pid_alt_output,    4, "");

    header("── PROPULSION");
    line("Thrust",      d.current_thrust,   1, "N");
    for (int i = 0; i < 4; ++i) {
        const std::string lbl = "M" + std::to_string(i) + " thr";
        line(lbl,       d.motor_throttle[i] * 100.0, 1, "%");
    }

    header("── BATTERY");
    const sf::Color bat_col = (d.battery_soc < 20.0) ? sf::Color(255, 80, 80) :
                               (d.battery_soc < 40.0) ? sf::Color(255, 180, 0) :
                                                          sf::Color(100, 255, 100);
    line("Voltage",     d.battery_voltage, 2, "V",  bat_col);
    line("SOC",         d.battery_soc,     1, "%",  bat_col);
    line("Current",     d.total_current,   2, "A");

    header("── ENVIRONMENT");
    line("Air Density", d.air_density,     4, "kg/m³");
    line("Wind N",      d.wind_world.x(),  2, "m/s");
    line("Wind E",      d.wind_world.y(),  2, "m/s");

    // Key guide at bottom
    y = H - 90.f;
    drawText("SPACE=Arm  T=Takeoff  L=Land  R=Reset", x, y, sf::Color(100, 120, 160), 10);
    y += 13.f;
    drawText("↑↓=Alt  W/S=WindN/S  A/D=WindW/E", x, y, sf::Color(100, 120, 160), 10);
    y += 13.f;
    drawText("1=Stab  2=AltHold  3=PosHold  4=RTH", x, y, sf::Color(100, 120, 160), 10);
}

// ============================================================
//  Motor throttle bars (bottom strip)
// ============================================================

void Renderer::drawMotorBars(const Simulation::TelemetryData& d) {
    const float W    = static_cast<float>(window_.getSize().x);
    const float H    = static_cast<float>(window_.getSize().y);
    const float bw   = 50.f;
    const float bh   = 80.f;
    const float gap  = 10.f;
    const float by   = H - bh - 4.f;
    const float start_x = (W - 260.f - 4.f * (bw + gap)) * 0.5f;

    const sf::Color bar_colors[4] = {
        sf::Color(255, 80,  80),
        sf::Color(80,  200, 255),
        sf::Color(255, 80,  80),
        sf::Color(80,  200, 255),
    };

    for (int i = 0; i < 4; ++i) {
        const float bx = start_x + i * (bw + gap);
        // Background
        drawRect(bx, by, bw, bh, sf::Color(20, 30, 50, 200),
                 sf::Color(60, 70, 100), 1.f);
        // Filled bar
        const float fill = static_cast<float>(d.motor_throttle[i]) * bh;
        drawRect(bx, by + bh - fill, bw, fill,
                 bar_colors[i], sf::Color::Transparent);
        // Label
        std::ostringstream oss;
        oss << "M" << i << "\n"
            << std::fixed << std::setprecision(0)
            << d.motor_throttle[i] * 100.f << "%";
        drawText(oss.str(), bx + 4.f, by + bh + 2.f, sf::Color(180, 200, 220), 10);
    }
}

// ============================================================
//  Attitude indicator (artificial horizon)
// ============================================================

void Renderer::drawAttitudeIndicator(const Simulation::TelemetryData& d) {
    const float cx     = 90.f;
    const float cy     = static_cast<float>(window_.getSize().y) - 200.f;
    const float radius = 70.f;
    const double roll  = d.euler_angles.x();
    const double pitch = d.euler_angles.y();

    // Clip to circle using a scissor rect (approximate with stencil)
    // We'll draw the horizon split: sky above, earth below, rotated by roll+pitch

    // Sky half (blue)
    sf::CircleShape sky_circle(radius);
    sky_circle.setFillColor(sf::Color(30, 80, 180));
    sky_circle.setOrigin({radius, radius});
    sky_circle.setPosition({cx, cy});
    window_.draw(sky_circle);

    // Earth half (brown) — a rectangle rotated by roll, shifted up/down by pitch
    const float pitch_px = static_cast<float>(pitch) * radius * 1.5f;
    sf::RectangleShape earth({radius * 4.f, radius * 2.f});
    earth.setFillColor(sf::Color(100, 60, 20));
    earth.setOrigin({radius * 2.f, 0.f});
    earth.setPosition({cx, cy + pitch_px});
    earth.setRotation(sf::degrees(static_cast<float>(roll * Math::RAD2DEG)));
    window_.draw(earth);

    // Horizon line
    sf::RectangleShape horizon({radius * 4.f, 2.f});
    horizon.setFillColor(sf::Color(255, 255, 255, 180));
    horizon.setOrigin({radius * 2.f, 1.f});
    horizon.setPosition({cx, cy + pitch_px});
    horizon.setRotation(sf::degrees(static_cast<float>(roll * Math::RAD2DEG)));
    window_.draw(horizon);

    // Fixed aircraft reference marker
    drawRect(cx - 20.f, cy - 2.f, 15.f, 4.f, sf::Color(255, 220, 0));
    drawRect(cx +  5.f, cy - 2.f, 15.f, 4.f, sf::Color(255, 220, 0));
    drawRect(cx -  2.f, cy - 2.f,  4.f, 8.f, sf::Color(255, 220, 0));

    // Rim
    sf::CircleShape rim(radius);
    rim.setFillColor(sf::Color::Transparent);
    rim.setOutlineColor(sf::Color(140, 160, 200));
    rim.setOutlineThickness(2.f);
    rim.setOrigin({radius, radius});
    rim.setPosition({cx, cy});
    window_.draw(rim);

    // Label
    drawText("ADI", cx - 12.f, cy + radius + 4.f, sf::Color(120, 140, 180), 10);
}

// ============================================================
//  Main render call
// ============================================================

void Renderer::render(const Flight::Drone&             drone,
                       const Simulation::TelemetryData& telemetry,
                       const Math::Vector3d&            wind) {
    drawBackground();
    drawGrid();
    drawAltitudeMarkers();
    drawTargetAltitudeLine(telemetry.target_altitude);
    drawGround();
    drawWindVector(wind);
    drawVelocityVector(drone);
    drawThrustVector(drone);
    drawDrone(drone, telemetry);
    drawHUD(telemetry);
    drawMotorBars(telemetry);
    drawAttitudeIndicator(telemetry);
}

} // namespace Rendering
} // namespace AeroCore
