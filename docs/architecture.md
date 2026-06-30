# AeroCore Architecture

## Runtime Pipeline

`AeroCore` runs a fixed-step flight-control loop and separates simulation updates from rendering updates.

1. Parse CLI args and load config (`Utilities::Config`).
2. Create airframe (`Flight::Drone`) and add configured propulsion units (`Flight::Motor`).
3. Create sensors (`Sensors::IMU`, `Sensors::Altimeter`, `Sensors::BatterySensor`).
4. Create controller (`Flight::FlightController`) and physics (`Physics::PhysicsEngine`).
5. Enter main loop:
   - Collect inputs (GUI) or run automated sequence (headless).
   - Update sensors.
   - Run controller and update motor dynamics.
   - Build external forces and run RK4 step.
   - Aggregate telemetry and render/print output.

## Main Subsystems

- `Flight/`: control logic, vehicle model, motor model, PID.
- `Physics/`: environmental model and rigid-body integration.
- `Sensors/`: noisy virtual sensors used by the controller.
- `Simulation/`: telemetry data and formatting for HUD/status output.
- `Rendering/`: SFML visualization and keyboard input mapping.
- `Utilities/`: config parser and logger.

## Data Ownership

- `DroneState` is owned by `Flight::Drone`.
- `PhysicsEngine` advances the state in-place each tick.
- `TelemetryManager` keeps the latest snapshot (`TelemetryData`) for HUD/status output.

## Headless Mode

Headless mode runs the same simulation stack as the GUI path, but:

- skips SFML renderer creation,
- automatically arms and starts takeoff,
- prints a compact status line at a configurable rate.
