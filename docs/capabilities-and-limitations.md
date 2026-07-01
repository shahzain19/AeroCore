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
  - **Estimator-driven control**: Flight controller now uses estimator state exclusively, no ground-truth shortcuts.
- **Autonomy and guidance**
  - Basic autonomous behaviors such as altitude hold, return-home, and mission-mode scaffolding are implemented in simulation.
  - The system can make mode-based decisions and close the loop on state feedback in the simulator.
- **Sensor simulation**
  - IMU (accelerometer + gyroscope), altimeter, battery sensor.
  - Sensor values are propagated into telemetry each simulation step.
- **State estimation**
  - Complementary filter attitude estimation (roll/pitch from IMU fusion).
  - **Default estimator-driven mode**: Simulation now defaults to noisy sensor data rather than perfect state injection.
  - Optional perfect-state injection for testing via `--no-perfect-state` CLI flag.
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
- **Hardware Abstraction Layer (HAL)**
  - Complete HAL interface definitions for IMU, barometer, GPS, battery, RC input, motor output, clock, and state estimator.
  - **STM32 HAL implementations**: Motor output (PWM/DShot) and RC input (SBUS/CRSF) drivers created as scaffolds.
  - STM32 platform configuration system with board-specific pin mappings.
  - CMake build target for STM32 firmware scaffold.
- **Test coverage (current)**
  - `test_pid_controller`
  - `test_config`
  - `test_telemetry_manager`
  - `test_cli_args`
  - `test_flight_mode`
  - `test_physics_engine`
  - `test_simulation_engine`
  - `test_motor_output` (HAL motor output forwarding)
  - `test_estimator`
  - `test_pre_arm`
  - `test_imu_fusion`

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

- **Complete STM32 firmware implementation**
  - HAL interfaces and STM32 driver scaffolds exist (`include/platforms/stm32/`), but full STM32 HAL integration is not complete.
  - STM32 drivers need actual STM32 HAL library integration, GPIO/Timer/UART peripheral initialization, and interrupt handlers.
  - Firmware build requires ARM toolchain setup (arm-none-eabi-gcc), STM32 HAL/LL drivers, and linker scripts.
- **Full GPS sensor model**
  - Position modes still rely on estimator state injection in sim (no GPS sensor model yet).
- **Full autonomy stack**
  - The repo has autonomy primitives and basic autonomous scenarios, but not a complete production-grade autonomy stack.
  - Missing full EKF/state estimation, mission path following, and robust embedded hardware execution.
- **Full mission navigation stack**
  - Waypoint path management and mission execution logic are not complete.
- **High-fidelity environment/world model**
  - No terrain map, obstacle model, or advanced weather/turbulence model yet.
- **Advanced estimator stack**
  - No full EKF/state-estimation pipeline (GPS fusion, bias estimation, etc.) - currently using complementary filter.
- **Complete hardware I/O integration**
  - STM32 motor output and RC input drivers are scaffolds with placeholder implementations.
  - Need actual DShot/PWM ESC output, CRSF/SBUS RC integration, and interrupt handlers.
  - No MAVLink implementation yet.
- **Comprehensive testing**
  - Integration test for `SimulationEngine` added; deeper FC/physics regression baselines still pending.
- **Production operator UX**
  - No packaged tuning UI yet.

## Current Practical Expectations

- Use AeroCore for **control-loop experimentation, mode-transition debugging, and simulation prototyping**.
- The simulator now runs in estimator-driven mode by default, providing realistic sensor-based control.
- The repo now supports a simulator target plus firmware scaffold targets for `stm32`, `linux-sbc`, and `esp32`.
- STM32 HAL scaffolds are in place but require STM32 HAL library integration for actual hardware builds.
- Do not treat current outputs as a **certified autopilot or high-fidelity aerodynamics benchmark**.
