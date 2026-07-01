# Testing

## Test Scope

The repository includes unit-style executable tests for critical utility/control blocks:

- `test_pid_controller`: PID behavior (saturation, anti-windup, derivative behavior, validation errors).
- `test_config`: config parsing and runtime key/value operations.
- `test_telemetry_manager`: status/HUD formatting and FPS estimator behavior.
- `test_cli_args`: `--help`, option parsing, and error handling.
- `test_flight_mode`: mode string helpers and GPS requirement flags.
- `test_physics_engine`: RK4 free-fall step and ISA air-density model.
- `test_simulation_engine`: arm/takeoff integration, telemetry, and reset.

## Build and Run Tests

From `build/`:

```bash
cmake .. -DAEROCORE_BUILD_TESTS=ON
cmake --build . -j"$(nproc)"
ctest --output-on-failure
```

## Notes

- Tests use a lightweight in-repo harness (`tests/test_common.h`) to avoid external test framework dependencies.
- These tests validate foundational behavior; integration and rendering-path tests can be added incrementally in future iterations.
- Recent additions include physics regression coverage for drag, gravity, and atmospheric density behavior in `test_physics_airdrag`.
- The next test expansion areas are estimator-driven state transitions, RC/motor HAL behavior, and fail-safe mode handling.
