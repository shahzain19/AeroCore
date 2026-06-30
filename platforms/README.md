# Platform ports

Hardware and simulation backends for the AeroCore HAL.

| Directory | Target | Status |
|-----------|--------|--------|
| `sim/` | Desktop simulation (wraps existing `Sensors/` + `Drone`) | Started |
| `stm32/` | STM32F4/F7/H7 flight-controller firmware | Planned — see `docs/embedded-installation.md` |
| `esp32/` | Wi-Fi MAVLink companion | Planned |
| `linux-sbc/` | Raspberry Pi / SBC with GPIO PWM | Planned |

## Sim backend

`SimMotorOutput` implements `HAL::IMotorOutput` by calling `Flight::Drone::setMotorThrottle()`.
Additional sim adapters (IMU, baro, GPS, RC from keyboard) will land in Phase 0 of the roadmap.

## Adding a new platform

1. Implement all HAL interfaces under `platforms/<name>/`.
2. Add `firmware/<name>/main.cpp` with the fixed-rate control loop.
3. Register a CMake target: `-DAEROCORE_TARGET=<name>`.
4. Document wiring in `docs/embedded-installation.md`.

See [sim-to-production.md](../docs/sim-to-production.md) for the full migration guide.
