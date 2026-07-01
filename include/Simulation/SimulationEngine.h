/**
 * @file SimulationEngine.h
 * @brief Core simulation world: vehicle, sensors, physics, and flight control.
 *
 * Headed and headless runners share this engine.  Rendering and CLI I/O live
 * outside this class.
 *
 * @author  AeroCore Contributors
 * @license MIT
 */

#pragma once

#include "Utilities/Config.h"
#include "Physics/PhysicsEngine.h"
#include "Flight/Drone.h"
#include "Flight/FlightController.h"
#include "Sensors/IMU.h"
#include "Sensors/Altimeter.h"
#include "Sensors/BatterySensor.h"
#include "Simulation/TelemetryManager.h"
#include "Math/Vector.h"
#include "Core/ComplementaryEstimator.h"
#include "platforms/sim/SimIMU.h"
#include "platforms/sim/SimBarometer.h"
#include "platforms/sim/SimRCInput.h"

#include <memory>
#include <string>

namespace AeroCore {
namespace Simulation {

/**
 * @brief Owns the full simulation stack and advances physics one tick at a time.
 */
class SimulationEngine {
public:
    explicit SimulationEngine(const std::string& config_hint);

    /** Advance one fixed physics step (sensors → FC → motors → RK4). */
    void stepPhysics();

    /** Reset vehicle, controller, wind, and simulation clock. */
    void reset();

    /** Populate @p out from the current world state. */
    void gatherTelemetry(TelemetryData& out) const;

    // ---- Accessors used by runners ----
    Flight::FlightController&       flightController();
    const Flight::FlightController& flightController() const;
    Flight::Drone&                  drone();
    const Flight::Drone&            drone() const;
    Physics::PhysicsEngine&         physics();
    Math::Vector3d&                 wind();
    const Math::Vector3d&           wind() const;

    double simTime()    const;
    double physicsDt()  const;
    const std::string& configPath() const;
    Core::ComplementaryEstimator& estimator();
    const Core::ComplementaryEstimator& estimator() const;
    Platform::Sim::SimRCInput& rcInput();
    const Platform::Sim::SimRCInput& rcInput() const;
    bool perfectState() const;

    void setPerfectState(bool enabled);

private:
    std::string config_path_;
    Utilities::Config config_;

    std::shared_ptr<Flight::Drone>                 drone_;
    std::shared_ptr<Sensors::IMU>                  imu_;
    std::shared_ptr<Sensors::Altimeter>            altimeter_;
    std::shared_ptr<Sensors::BatterySensor>        battery_sensor_;
    std::unique_ptr<Flight::FlightController>      flight_controller_;
    std::unique_ptr<Core::ComplementaryEstimator>  estimator_;
    std::unique_ptr<Platform::Sim::SimIMU>       sim_imu_;
    std::unique_ptr<Platform::Sim::SimBarometer>   sim_baro_;
    std::unique_ptr<Platform::Sim::SimRCInput>     sim_rc_;
    Physics::PhysicsEngine                         physics_;

    bool perfect_state_ = false;

    Math::Vector3d wind_;
    double physics_dt_  = 0.004;
    double sim_time_    = 0.0;
    double drag_coeff_  = 0.47;
    double drone_area_  = 0.1;

    static std::string findConfigFile(const std::string& hint);
};

} // namespace Simulation
} // namespace AeroCore
