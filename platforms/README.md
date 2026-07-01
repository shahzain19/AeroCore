# Platform ports

Hardware and simulation backends for the AeroCore HAL.

| Directory | Target | Status |
|-----------|--------|--------|
| `sim/` | Desktop simulation (wraps existing `Sensors/` + `Drone`) | Complete |
| `stm32/` | STM32F4/F7/H7 flight-controller firmware | 🚧 HAL scaffolds implemented, hardware integration pending |
| `esp32/` | Wi-Fi MAVLink companion | Planned |
| `linux-sbc/` | Raspberry Pi / SBC with GPIO PWM | Planned |

## Sim backend

`SimMotorOutput` implements `HAL::IMotorOutput` by calling `Flight::Drone::setMotorThrottle()`.
`SimIMU`, `SimBarometer`, and `SimRCInput` provide sensor simulation for testing.

## STM32 backend

STM32 HAL scaffolds are now implemented with the following components:

- **STM32MotorOutput**: Motor output driver supporting PWM and DShot protocols
- **STM32RCInput**: RC input driver supporting SBUS and CRSF protocols
- **STM32Config**: Board configuration system with pin mappings and peripheral assignments
- **firmware/stm32_main.cpp**: STM32 firmware entry point with control loop structure

**Current status**: HAL scaffolds are complete with placeholder implementations for STM32 HAL library calls. The structure is ready for STM32 HAL library integration when ARM toolchain is set up.

**Required for completion**:
- ARM toolchain setup (arm-none-eabi-gcc)
- STM32 HAL/LL library integration
- GPIO/Timer/UART peripheral initialization
- Interrupt handler implementation
- Linker scripts for memory layout

## Adding a new platform

1. Implement all HAL interfaces under `platforms/<name>/`.
2. Add `firmware/<name>/main.cpp` with the fixed-rate control loop.
3. Register a CMake target: `-DAEROCORE_TARGET=<name>`.
4. Document wiring in `docs/embedded-installation.md`.

See [sim-to-production.md](../docs/sim-to-production.md) for the full migration guide.
