# AeroCore firmware targets

Firmware entry points for embedded and SBC deployments. The desktop **simulator** remains the default build (`AEROCORE_TARGET=sim`).

## Status

| Target | MCU / OS | Control rate | Status |
|--------|----------|--------------|--------|
| `sim` | Desktop | 250 Hz (configurable) | **Shipping** |
| `stm32` | STM32F405+ | 1–4 kHz | Scaffold only |
| `linux-sbc` | Linux + PREEMPT | 500 Hz–1 kHz | Scaffold only |
| `esp32` | ESP32-S3 | Bridge / companion | Scaffold only |

## Build (when implemented)

```bash
# STM32 (cross-compile)
cmake -B build-f4 -DAEROCORE_TARGET=stm32 -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f405.cmake
cmake --build build-f4

# Linux SBC (no SFML, no physics)
cmake -B build-sbc -DAEROCORE_TARGET=linux-sbc -DAEROCORE_BUILD_SIM=OFF
cmake --build build-sbc
```

## Documentation

- [Embedded installation](../docs/embedded-installation.md)
- [Sim → production](../docs/sim-to-production.md)
- [Roadmap](../docs/roadmap.md)

## Safety

Firmware in this directory is **experimental**. Complete Phase 1 safety items on the roadmap before prop-on testing.
