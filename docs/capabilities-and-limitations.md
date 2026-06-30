# Capabilities and Limitations

This document tracks what AeroCore currently implements, what is partially implemented, and what is not implemented yet.

## Implemented (Working Today)

- **Core simulation loop**
  - Fixed-timestep simulation with RK4 integration in `Physics::PhysicsEngine`.
  - Decoupled frame/update timing in `main.cpp`.
- **Vehicle and propulsion model**
  - Drone model with configurable mass/inertia/airframe parameters.
  - Motor dynamics and thrust/torque computation.
- **Flight controller stack**
  - FSM with arming, takeoff, and altitude-hold transitions.
  - PID-based control components (`Flight::PIDController`).
  - Mode request handling for multiple flight modes.
- **Sensor simulation**
  - IMU (accelerometer + gyroscope), altimeter, battery sensor.
  - Sensor values are propagated into telemetry each simulation step.
- **Telemetry and visualization**
  - Rich HUD string generation for GUI rendering.
  - Headless status-line telemetry output at configurable rate.
  - Runtime logging to console/log files.
- **Configuration system**
  - TOML-like parser with sections, key/value, inline comments.
  - Quoted string handling and typed getters.
- **CLI and runtime controls**
  - `--help` / `-h` for usage text.
  - `--headless`, `--duration <seconds>`, `--status-rate <hz>`, `--debug-headless`.
  - Unknown options produce explicit errors (no silent ignore).
- **Modular simulation runners**
  - `Simulation::SimulationEngine` — shared physics/sensor/FC stack.
  - `Simulation::HeadedSimulation` — SFML GUI runner.
  - `Simulation::HeadlessSimulation` — console auto-flight runner.
- **Test coverage (current)**
  - `test_pid_controller`
  - `test_config`
  - `test_telemetry_manager`
  - `test_cli_args`
  - `test_flight_mode`
  - `test_physics_engine`
  - `test_simulation_engine`

## Partially Implemented / Basic-Only

- **Fixed-wing support**
  - Config presets exist and baseline simulation runs.
  - Advanced fixed-wing-specific guidance/autopilot behavior is limited.
- **Flight mode breadth**
  - Several mode enums exist (`POSITION_HOLD`, `RETURN_HOME`, `MISSION`, etc.).
  - `POSITION_HOLD` and `RETURN_HOME` have basic ground-truth position guidance.
  - `MISSION` and full GPS-fused navigation are not complete end-to-end.
- **Validation/robustness**
  - CLI parsing and `--help` are implemented; config schema validation remains permissive.

## Not Implemented Yet (Known Gaps)

- **Full mission navigation stack**
  - Waypoint path management and mission execution logic are not complete.
- **High-fidelity environment/world model**
  - No terrain map, obstacle model, or advanced weather/turbulence model yet.
- **Estimator stack**
  - No full EKF/state-estimation pipeline (GPS fusion, bias estimation, etc.).
- **Comprehensive testing**
  - Integration test for `SimulationEngine` added; deeper FC/physics regression baselines still pending.
- **Production operator UX**
  - No packaged tuning UI yet.

## Current Practical Expectations

- Use AeroCore for **control-loop experimentation, mode-transition debugging, and simulation prototyping**.
- Do not treat current outputs as a **certified autopilot or high-fidelity aerodynamics benchmark**.
