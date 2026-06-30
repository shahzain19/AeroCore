#include "Simulation/SimulationEngine.h"
#include "Flight/FlightMode.h"
#include "test_common.h"

#include <algorithm>
#include <fstream>

using AeroCore::Flight::FlightMode;
using AeroCore::Simulation::SimulationEngine;

int main() {
    const std::string path = "tmp_test_simulation.toml";
    {
        std::ofstream out(path);
        out << "[physics]\ngravity = 9.80665\nair_density = 1.225\n";
        out << "drag_coefficient = 0.47\n";
        out << "[drone]\nmass = 1.5\narm_length = 0.225\nsize = 0.45\n";
        out << "airframe = \"multirotor\"\nmotor_count = 4\n";
        out << "[battery]\nvoltage_max = 16.8\ncapacity_ah = 1.5\ninternal_resistance = 0.02\n";
        out << "[motor]\nresponse_time_constant = 0.05\nmax_rpm = 10000\n";
        out << "propeller_diameter = 0.127\nthrust_coefficient = 0.10\n";
        out << "torque_coefficient = 0.010\nefficiency = 0.85\nmax_thrust = 8.0\n";
        out << "[pid_altitude]\nkp = 0.12\nki = 0.015\nkd = 0.15\n";
        out << "output_min = -0.30\noutput_max = 0.30\nintegral_max = 0.15\n";
        out << "derivative_filter = 0.25\n";
        out << "[pid_roll]\nkp = 6.0\n";
        out << "[pid_pitch]\nkp = 6.0\n";
        out << "[pid_yaw]\nkp = 4.0\n";
        out << "[pid_roll_rate]\nkp = 0.15\n";
        out << "[pid_pitch_rate]\nkp = 0.15\n";
        out << "[pid_yaw_rate]\nkp = 0.20\n";
        out << "[flight]\nmax_roll_angle_deg = 35\nmax_pitch_angle_deg = 35\n";
        out << "max_roll_rate_deg_s = 220\nmax_pitch_rate_deg_s = 220\n";
        out << "max_yaw_rate_deg_s = 180\nmax_tilt_angle_deg = 45\n";
        out << "[imu]\naccel_rate_hz = 1000\naccel_noise = 0.05\naccel_drift = 0.001\n";
        out << "gyro_rate_hz = 1000\ngyro_noise = 0.0017\ngyro_drift = 0.0001\n";
        out << "comp_filter_alpha = 0.98\n";
        out << "[simulation]\ndt = 0.004\ntarget_altitude = 10.0\n";
    }

    SimulationEngine engine(path);
    auto& fc = engine.flightController();

    fc.arm();
    fc.takeoff();

    double max_alt = 0.0;
    double min_alt_after_hold = 1e9;
    bool saw_alt_hold = false;

    const int steps = 3750; // 15 s at 250 Hz
    for (int i = 0; i < steps; ++i) {
        engine.stepPhysics();
        if (fc.getMode() == FlightMode::ALTITUDE_HOLD) {
            saw_alt_hold = true;
            const double alt = engine.drone().getAltitude();
            max_alt = std::max(max_alt, alt);
            min_alt_after_hold = std::min(min_alt_after_hold, alt);
        }
    }

    AeroCore::Tests::expectTrue(engine.simTime() > 14.0, "sim time advances");
    AeroCore::Tests::expectTrue(saw_alt_hold, "reaches altitude hold mode");
    AeroCore::Tests::expectTrue(max_alt < 12.5, "alt hold does not diverge high");
    AeroCore::Tests::expectTrue(min_alt_after_hold > 7.5,
                                "alt hold does not diverge low");

    AeroCore::Simulation::TelemetryData telem{};
    engine.gatherTelemetry(telem);
    AeroCore::Tests::expectTrue(telem.motor_throttle[0] >= 0.0,
                                "telemetry includes motor data");

    engine.reset();
    AeroCore::Tests::expectTrue(engine.simTime() == 0.0, "reset clears sim time");
    AeroCore::Tests::expectTrue(fc.getMode() == FlightMode::DISARMED,
                                "reset returns to disarmed");

    std::remove(path.c_str());
    return AeroCore::Tests::finish();
}
