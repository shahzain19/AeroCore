# AeroCore Roadmap: Next Big Features

AeroCore is already a capable simulation-first flight-controller research platform. The next step is to turn that foundation into a more realistic and portable control stack by tightening the gap between simulation and real hardware.

## Priority 1 — Replace simulation shortcuts with real estimator flow ✅ COMPLETED

Goal: make the controller consume estimator state instead of relying on perfect state from the simulator.

- ✅ Move the flight controller to a dedicated estimator-driven state pipeline.
- ✅ Keep the current simulator as a testbed, but make the default path use noisy sensor data and fused attitude/altitude estimates.
- ✅ Add regression tests around estimator behavior, bias handling, and mode transitions.

Why first: this is the biggest architectural gap between AeroCore as a simulator and AeroCore as a real flight controller.

**Status**: Completed - Flight controller now uses estimator state exclusively, simulation defaults to estimator-driven mode.

## Priority 2 — Add a real hardware I/O layer 🚧 IN PROGRESS

Goal: support RC input and motor-output backends that match what a real board would need.

- ✅ Add a stable HAL interface for RC input, motor outputs, and timing.
- ✅ Implement STM32 HAL scaffolds for motor output (PWM/DShot) and RC input (SBUS/CRSF).
- ✅ Add STM32 platform configuration system with board-specific pin mappings.
- ✅ Create CMake build target for STM32 firmware scaffold.
- ⏳ Complete STM32 HAL library integration (GPIO/Timer/UART peripheral initialization).
- ⏳ Implement interrupt handlers for RC input and control loop timing.
- ⏳ Set up ARM toolchain and STM32 HAL/LL drivers for actual firmware builds.

Why second: the controller core can be validated in simulation, but first-flight confidence requires real I/O loops.

**Status**: HAL scaffolds implemented, hardware integration pending STM32 HAL library setup.

## Priority 3 — Harden safety and failsafe behavior

Goal: make the system safe enough for real-world bench testing.

- Tighten pre-arm checks with explicit diagnostics.
- Add RC-loss, low-battery, and IMU health failsafe transitions.
- Improve logging and blackbox-style telemetry for tuning and debugging.

Why third: safety is the gating item before any bench or tethered flight work.

## Priority 4 — Add GPS and navigation primitives

Goal: move from manual altitude/attitude control to true position-aware modes.

- Introduce a simple GPS sensor model and a basic state estimator path.
- Implement position-hold and return-to-home behavior with real navigation state.
- Keep the scope narrow at first: single-airframe, low-speed, indoor/outdoor safe operation.

Why fourth: this unlocks meaningful autonomy, but it depends on the estimator work above.

## Priority 5 — Expand documentation and regression coverage

Goal: keep the project maintainable as the feature set grows.

- Add more subsystem-specific tests for physics, estimator, and flight-mode behavior.
- Keep the docs aligned with current capabilities and limitations.
- Publish a short contributor workflow so new features can be added without breaking the simulator path.

## Recommended execution order

1. Estimator-driven state flow
2. RC and motor backends
3. Failsafe and logging hardening
4. GPS and position-hold
5. Mission and fixed-wing extensions

## Success criteria

The roadmap can be considered successful when the project can:

- run the core control loop from estimator-driven sensor input,
- arm/disarm safely with explicit failsafe handling,
- output motor commands through a real hardware backend,
- demonstrate position-hold or return-to-home in simulation,
- and keep regression tests passing as new features are added.
