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
- **Autonomy and guidance**
  - Basic autonomous behaviors such as altitude hold, return-home, and mission-mode scaffolding are implemented in simulation.
  - The system can make mode-based decisions and close the loop on state feedback in the simulator.
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
  - `--headless`, `--duration <seconds>`, `--status-rate <hz>`, `--debug-headless`, `--no-perfect-state`.
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

- **Embedded firmware targets**
  - HAL interfaces exist (`include/HAL/`); STM32/ESP32/Linux SBC drivers and firmware loops are not complete. See `docs/embedded-installation.md`.
- **Ground-truth shortcuts in flight controller**
  - With `simulation.perfect_state = true` (default), attitude and position come from the estimator fed by physics truth for POS_HOLD/RTH. Set `simulation.perfect_state = false` or run with `--no-perfect-state` to exercise noisy IMU-only fusion.
  - Full GPS sensor model not implemented; position modes still rely on perfect-state injection in sim.
- **Full autonomy stack**
  - The repo has autonomy primitives and basic autonomous scenarios, but not a complete production-grade autonomy stack.
  - Missing full EKF/state estimation, mission path following, and robust embedded hardware execution.
- **Full mission navigation stack**
  - Waypoint path management and mission execution logic are not complete.
- **High-fidelity environment/world model**
  - No terrain map, obstacle model, or advanced weather/turbulence model yet.
- **Estimator stack**
  - No full EKF/state-estimation pipeline (GPS fusion, bias estimation, etc.).
- **Hardware I/O**
  - No DShot/PWM ESC output, CRSF/SBUS RC, or MAVLink in firmware yet. The flight controller core now supports HAL motor output forwarding, but hardware-specific motor drivers are still pending.
- Test coverage has been expanded to include HAL motor output forwarding and disarm propagation in the flight controller.
- **Comprehensive testing**
  - Integration test for `SimulationEngine` added; deeper FC/physics regression baselines still pending.
- **Production operator UX**
  - No packaged tuning UI yet.

## Current Practical Expectations

- Use AeroCore for **control-loop experimentation, mode-transition debugging, and simulation prototyping**.
- The repo now supports a simulator target plus firmware scaffold targets for `stm32`, `linux-sbc`, and `esp32`.
- Do not treat current outputs as a **certified autopilot or high-fidelity aerodynamics benchmark**.
