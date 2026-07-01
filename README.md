# AeroCore Flight Controller

AeroCore is a C++20 flight-controller project for multirotor and fixed-wing aircraft.  
It focuses on a simulation-first workflow: portable control logic, a desktop physics simulator, and a clear path toward firmware targets for real embedded hardware.

**Today:** desktop simulation with headless and GUI modes. **Next:** HAL-backed firmware scaffolding for STM32-class boards and other embedded targets — see [docs/sim-to-production.md](docs/sim-to-production.md) and [docs/personal-docs/roadmap.md](docs/personal-docs/roadmap.md).

## Core Features

- Fixed-step physics simulation with RK4 integration.
- Vehicle/propulsion simulation (`Flight::Drone`, `Flight::Motor`).
- Flight-control state machine with multiple modes.
- Virtual sensor stack (IMU, altimeter, battery).
- Config-driven behavior via TOML-like files.
- GUI renderer (SFML) and headless simulation mode.
- Telemetry/HUD formatting plus status-line output.
- Unit-style tests for key subsystems.

## Autonomy Status

- The codebase already includes autonomy-facing behavior in simulation, including mode transitions, altitude-hold, return-home guidance, and mission-mode scaffolding.
- This is best described as an autonomy prototype and research platform, not a finished production autopilot.
- Remaining autonomy gaps include full state estimation, GPS/mission navigation, robust failure handling, and embedded hardware I/O.

## Project Layout

```text
AeroCore/
├── include/        # Public headers by subsystem
├── src/            # Implementations + main entry point
├── config/         # Example simulation configs
├── tests/          # Unit-style executable tests
├── docs/           # Architecture, build, config, testing docs
├── assets/         # Rendering assets
├── logs/           # Runtime log output
└── CMakeLists.txt
```

## Dependencies

- CMake `3.22+`
- C++20 compiler
- Eigen3
- SFML (`graphics`, `window`, `system`) for simulator builds

## Build

AeroCore supports multiple build targets through CMake. The default desktop simulator target is the easiest way to get started:

```bash
mkdir -p build
cd build
cmake -DAEROCORE_TARGET=sim ..
cmake --build . -j"$(nproc)"
```

For a firmware-style scaffold without the simulator executable:

```bash
mkdir -p build-stm32
cd build-stm32
cmake -DAEROCORE_TARGET=stm32 -DAEROCORE_BUILD_TESTS=OFF ..
cmake --build . -j"$(nproc)"
```

Other supported targets include `linux-sbc` and `esp32`.

## Run

Show usage:

```bash
./AeroCore --help
```

GUI mode:

```bash
./AeroCore
```

Headless mode:

```bash
./AeroCore --headless
```

Headless options:

- `--duration <seconds>`: max simulation time (default `60`).
- `--status-rate <hz>`: console status refresh rate (default `5`).
- `--debug-headless`: enable additional early debug output.

Use a specific config:

```bash
./AeroCore config/fixed_wing.toml --headless --duration 20
```

## Controls (GUI)

- `Space`: arm/disarm
- `T`: takeoff
- `L`: land
- `R`: reset simulation
- `Up` / `Down`: target altitude
- `W/A/S/D`: adjust wind vector
- `1/2/3/4`: mode requests
- `Esc`: quit

## Testing

From `build/`:

```bash
cmake .. -DAEROCORE_BUILD_TESTS=ON
cmake --build . -j"$(nproc)"
ctest --output-on-failure
```

Current tests:

- `test_pid_controller`: PID behavior, saturation, and anti-windup.
- `test_config`: config parsing and key/value access.
- `test_telemetry_manager`: HUD/status output formatting.
- `test_cli_args`: command-line parsing and validation.
- `test_flight_mode`: mode state transitions and requirement checks.
- `test_physics_engine`: RK4 integration and environment model.
- `test_simulation_engine`: full integration of FC, physics, and telemetry.
- `test_estimator`: state estimation and sensor bias learning.
- `test_pre_arm`: arming safety checks (throttle, level, sensors).
- `test_imu_fusion`: IMU attitude filtering and fusion.

## Documentation

- `docs/roadmap.md` — real-world feature plan (phased)
- `docs/sim-to-production.md` — sim → firmware migration path and current status
- `docs/embedded-installation.md` — board support, build targets, and hardware porting notes
- `docs/architecture.md`
- `docs/build-and-run.md`
- `docs/capabilities-and-limitations.md`
- `docs/config-reference.md`
- `docs/testing.md`

## Repository Files

- `LICENSE.md` - MIT license
- `SECURITY.md` - vulnerability reporting policy
- `CONTRIBUTING.md` - contributor workflow
