# AeroCore Architecture

AeroCore is evolving from a **desktop simulator** into a **portable flight-controller core** with platform-specific HAL backends. See [sim-to-production.md](sim-to-production.md) and [roadmap.md](roadmap.md).

## Layered view

```text
FlightController / PID / FlightMode     ← portable (all targets)
HAL (IIMU, IBarometer, IGPS, …)         ← interface in include/HAL/
platforms/sim | stm32 | linux-sbc       ← implementations
Physics / Renderer / SimulationEngine   ← sim target only
```

## Runtime Pipeline (sim target)

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
- `HAL/`: hardware abstraction interfaces for sensors, RC, motors, clock, estimator.
- `platforms/`: sim and embedded HAL implementations (`platforms/sim/SimMotorOutput`, etc.).
- `firmware/`: embedded entry points (scaffold; see `firmware/README.md`).
- `Physics/`: environmental model and rigid-body integration (**sim only**).
- `Sensors/`: noisy virtual sensors used by the controller (**sim backends**).
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

## Planned next features

The next major milestones are focused on closing the gap between the simulator and a real flight controller:

1. Estimator-driven control flow so the flight controller consumes fused sensor state rather than perfect-state shortcuts.
2. Hardware backend support for RC input and motor output on a concrete embedded target.
3. Safety and failsafe hardening for arming, RC loss, and low-battery behavior.
4. GPS-backed position hold and return-to-home primitives.

The public roadmap summary lives in [roadmap.md](roadmap.md) and the deeper implementation-oriented plan remains in [personal-docs/roadmap.md](personal-docs/roadmap.md).
