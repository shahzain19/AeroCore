# AeroCore Flight Controller

AeroCore is a C++20 flight-controller project for multirotor and fixed-wing aircraft.  
It provides PID-based control loops, a mode state machine, and a desktop physics simulator for tuning before hardware deployment.

**Today:** desktop sim (SFML + headless). **Next:** HAL-backed firmware for STM32-class boards — see [roadmap](docs/roadmap.md).

## Core Features

- Fixed-step physics simulation with RK4 integration.
- Vehicle/propulsion simulation (`Flight::Drone`, `Flight::Motor`).
- Flight-control state machine with multiple modes.
- Virtual sensor stack (IMU, altimeter, battery).
- Config-driven behavior via TOML-like files.
- GUI renderer (SFML) and headless simulation mode.
- Telemetry/HUD formatting plus status-line output.
- Unit-style tests for key subsystems.

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
- SFML (`graphics`, `window`, `system`)

## Build

```bash
mkdir -p build
cd build
cmake ..
cmake --build . -j"$(nproc)"
```

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

- `test_pid_controller`
- `test_config`
- `test_telemetry_manager`
- `test_cli_args`
- `test_flight_mode`
- `test_physics_engine`
- `test_simulation_engine`

## Documentation

- `docs/roadmap.md` — real-world feature plan (phased)
- `docs/sim-to-production.md` — sim → firmware migration
- `docs/embedded-installation.md` — build & flash on STM32, ESP32, Pi, etc.
- `docs/architecture.md`
- `docs/build-and-run.md`
- `docs/capabilities-and-limitations.md`
- `docs/config-reference.md`
- `docs/testing.md`

## Repository Files

- `LICENSE.md` - MIT license
- `SECURITY.md` - vulnerability reporting policy
- `CONTRIBUTING.md` - contributor workflow
