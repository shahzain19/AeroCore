# Sim to Production Migration

This document explains the current AeroCore migration path from a desktop simulator to real embedded flight-controller firmware.

## Current state

- The simulator target is fully supported and builds as the primary user-facing path.
- A portable core library (`AeroCoreCore`) is now separated from simulator-specific code.
- Simulator-only code lives in `AeroCoreSim` and is gated behind `AEROCORE_TARGET=sim`.
- Firmware-style targets (`stm32`, `linux-sbc`, `esp32`) are scaffolded in the build system, but board-specific drivers and final firmware images are still under development.

## What this migration solves

The main risk with a sim-first flight controller is that the controller can cheat by reading ground-truth state directly from the physics engine.
AeroCore now separates:

- the portable control logic,
- the hardware abstraction layer,
- and the simulator backend.

That separation makes it possible to:

- test control loops in the simulator,
- then reuse the same FC logic on embedded targets,
- and write real hardware HAL drivers without rewriting core flight code.

## Current architecture

```
[ FlightController + PID + FlightMode ]  <-- portable core
                  ^
                  |
[ HAL interfaces (IMU, Baro, RC, MotorOutput, GPS, Clock) ]
                  |
      +-----------+-----------+
      |           |           |
    Sim backend  STM32 FW    Linux/ESP32
```

## Build target guidance

- `AEROCORE_TARGET=sim` builds the simulator executable and links SFML.
- `AEROCORE_TARGET=stm32`, `linux-sbc`, or `esp32` builds the portable core without the simulator GUI.

Example simulator build:

```bash
mkdir -p build
cd build
cmake -DAEROCORE_TARGET=sim ..
cmake --build . -j"$(nproc)"
```

Example firmware scaffold build:

```bash
mkdir -p build-stm32
cd build-stm32
cmake -DAEROCORE_TARGET=stm32 -DAEROCORE_BUILD_TESTS=OFF ..
cmake --build . -j"$(nproc)"
```

## What still needs to land for production

- concrete STM32/ESP32/Linux SBC HAL driver implementations
- RC receiver support (SBUS/CRSF)
- ESC output support (PWM/DShot)
- full hardware state estimation and sensor fusion
- board-specific pin maps and flash/boot instructions
- runtime CLI/telemetry on MCU targets

## Recommended next documentation updates

- keep `docs/embedded-installation.md` synchronized with the actual available firmware ports
- add board-specific notes once a concrete MCU path is implemented
- add a hardware acceptance criteria checklist for each target
