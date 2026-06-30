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
  - `--headless`, `--duration <seconds>`, `--status-rate <hz>`, `--debug-headless`.
  - Missing values for `--duration` and `--status-rate` now produce explicit errors.
- **Test coverage (current)**
  - `test_pid_controller`
  - `test_config`
  - `test_telemetry_manager`

## Partially Implemented / Basic-Only

- **Fixed-wing support**
  - Config presets exist and baseline simulation runs.
  - Advanced fixed-wing-specific guidance/autopilot behavior is limited.
- **Flight mode breadth**
  - Several mode enums exist (`POSITION_HOLD`, `RETURN_HOME`, `MISSION`, etc.).
  - Not all modes have full mission-grade behavior implemented end-to-end.
- **Validation/robustness**
  - Core argument handling improved, but complete CLI validation/help text is still minimal.
  - Config schema validation is permissive and mostly runtime-driven.

## Not Implemented Yet (Known Gaps)

- **Full mission navigation stack**
  - Waypoint path management and mission execution logic are not complete.
- **High-fidelity environment/world model**
  - No terrain map, obstacle model, or advanced weather/turbulence model yet.
- **Estimator stack**
  - No full EKF/state-estimation pipeline (GPS fusion, bias estimation, etc.).
- **Comprehensive testing**
  - Missing deeper integration tests across `FlightController`, `Drone`, and `PhysicsEngine`.
  - No automated regression baselines for flight envelopes.
- **Production operator UX**
  - No built-in CLI help command (`--help`) and no packaged tuning UI yet.

## Current Practical Expectations

- Use AeroCore for **control-loop experimentation, mode-transition debugging, and simulation prototyping**.
- Do not treat current outputs as a **certified autopilot or high-fidelity aerodynamics benchmark**.
