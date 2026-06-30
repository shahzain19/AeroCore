# Sim → Production: Transforming AeroCore for Real Hardware

This document describes how AeroCore moves from a **desktop physics simulator** to **deployable flight-controller firmware**, what changes in the codebase, and what stays the same.

---

## The problem today

AeroCore v2.0 runs a closed loop:

```
Physics (ground truth) → Virtual sensors → FlightController → Motor model → Physics
```

Three shortcuts make the sim fly better than real hardware ever would:

| Shortcut | Location | Real-world replacement |
|----------|----------|------------------------|
| Perfect roll/pitch/yaw | `FlightController::runAttitudeControl()` reads `drone_->getEulerAngles()` | IMU fusion (complementary → EKF) |
| Perfect position | `updatePositionHold()` / `updateReturnHome()` read `drone_->getPosition()` | GPS + velocity estimator |
| Abstract throttle | `Drone::applyControl()` sets 0–1 throttle | DShot/PWM to ESCs |

Until these are replaced, **position hold and RTH in the sim prove navigation logic only, not estimator quality**.

---

## Target architecture

```
                    ┌──────────────────┐
                    │  FlightController │
                    │  PIDController    │
                    │  FlightMode FSM   │
                    └────────┬─────────┘
                             │ reads
                    ┌────────▼─────────┐
                    │  VehicleState      │  ← single truth for FC
                    │  (att, rates, pos) │
                    └────────┬─────────┘
                             │ produced by
              ┌──────────────┼──────────────┐
              │              │              │
     ┌────────▼────┐  ┌──────▼──────┐  ┌───▼────────┐
     │  Estimator  │  │  Sim truth  │  │  EKF (HW)  │
     │  (sim: CF)  │  │  (dev only) │  │  (Phase 2) │
     └────────┬────┘  └─────────────┘  └─────┬──────┘
              │                              │
     ┌────────▼──────────────────────────────▼──────┐
     │              HAL sensor interfaces              │
     │  IIMU  IBarometer  IGPS  IBattery  IRCInput   │
     └────────┬───────────────────────┬─────────────┘
              │                       │
     ┌────────▼────────┐     ┌────────▼────────┐
     │  Sim backends   │     │  HW drivers     │
     │  (noise models) │     │  SPI/I2C/UART   │
     └─────────────────┘     └─────────────────┘

                    ┌──────────────────┐
                    │  IMotorOutput    │
                    └────────┬─────────┘
              ┌──────────────┼──────────────┐
     ┌────────▼────┐  ┌──────▼──────┐  ┌────▼─────┐
     │  Sim motors │  │  DShot STM32│  │  PWM Pi  │
     └─────────────┘  └─────────────┘  └──────────┘
```

---

## Repository layout (target)

```text
AeroCore/
├── include/
│   ├── Core/              # VehicleState, Estimator interface
│   ├── Flight/            # FlightController, PID, Drone mixer (existing)
│   ├── HAL/               # Platform-agnostic hardware interfaces
│   ├── Sensors/           # Sim sensor models (existing)
│   ├── Physics/           # Sim only
│   └── Simulation/        # Sim runners only
├── src/
├── platforms/
│   ├── sim/               # HAL backends wrapping Sensors/* + Drone
│   ├── stm32/             # Startup, drivers, linker scripts
│   ├── esp32/             # Wi-Fi bridge firmware
│   └── linux-sbc/         # gpiod, i2c-dev
├── firmware/
│   ├── stm32/main.cpp     # 1 kHz control loop, no SFML
│   └── linux-sbc/main.cpp
├── config/                # Shared airframe params
└── CMakeLists.txt         # AEROCORE_TARGET option
```

---

## HAL interfaces (added in `include/HAL/`)

| Interface | Responsibility |
|-----------|----------------|
| `IIMU` | Accelerometer + gyro at control rate; optional data-ready IRQ |
| `IBarometer` | Pressure altitude, temperature |
| `IGPS` | Lat/lon, alt, velocity NED, fix quality |
| `IBattery` | Voltage, current (optional) |
| `IRCInput` | Normalized channels + link status |
| `IMotorOutput` | `write(motor_index, throttle_0_1)` → protocol-specific |
| `IClock` | Monotonic microseconds; `sleep_until` for rate loop |

Sim implementations delegate to existing `Sensors::IMU`, etc. Hardware implementations talk to registers.

---

## Migration steps (ordered)

### Step 1 — Introduce `VehicleState`

```cpp
struct VehicleState {
    Math::Quaterniond attitude;
    Math::Vector3d euler_rpy;      // derived
    Math::Vector3d angular_rate;   // body frame
    Math::Vector3d position_ned;
    Math::Vector3d velocity_ned;
    double altitude_amsl;
    bool position_valid;
    bool attitude_valid;
};
```

FlightController reads `VehicleState&` instead of querying `Drone` for attitude/position.

### Step 2 — Add `IStateEstimator`

```cpp
class IStateEstimator {
public:
    virtual void predict(double dt) = 0;
    virtual void correctIMU(const IIMU& imu) = 0;
    virtual void correctBaro(const IBarometer& baro) = 0;
    virtual void correctGPS(const IGPS& gps) = 0;
    virtual const VehicleState& state() const = 0;
};
```

**Sim:** `ComplementaryEstimator` uses existing IMU filter + physics position (GPS modes only).  
**Hardware:** Start with `ComplementaryEstimator`; upgrade to `EkfEstimator` in Phase 2.

### Step 3 — Rewire `FlightController`

Change in `runAttitudeControl()`:

```cpp
// Before (sim cheat):
const auto euler = drone_->getEulerAngles();

// After (production):
const auto& euler = vehicle_state_.euler_rpy;  // from estimator
```

Change in `updatePositionHold()`:

```cpp
// Before:
const auto& pos = drone_->getPosition();

// After:
if (!vehicle_state_.position_valid) { requestMode(FAILSAFE); return; }
const auto& pos = vehicle_state_.position_ned;
```

### Step 4 — Split CMake targets

```cmake
option(AEROCORE_TARGET "Build target: sim, stm32, linux-sbc" "sim")

if(AEROCORE_TARGET STREQUAL "sim")
    # Current AeroCore executable + SFML + Physics
elseif(AEROCORE_TARGET STREQUAL "stm32")
    # firmware/stm32 only: no SFML, no PhysicsEngine
endif()
```

### Step 5 — Real-time control loop (firmware)

Desktop sim uses accumulator + `sf::Clock`. Firmware uses:

```cpp
void controlTask() {
    const auto period = std::chrono::microseconds(1000);  // 1 kHz
    auto next = IClock::now();
    while (running) {
        sensors.poll();
        estimator.predict(1e-3);
        estimator.correctIMU(imu);
        flight_controller.update(1e-3);
        motors.write(flight_controller.motorOutputs());
        next += period;
        IClock::sleep_until(next);
    }
}
```

Sensors run in SPI/I2C ISR or DMA completion callbacks; control task never blocks on I/O.

### Step 6 — Motor output layer

`Drone::applyControl()` becomes sim-only. Firmware path:

```cpp
class DShotMotorOutput : public HAL::IMotorOutput {
    void write(size_t motor, double throttle_0_1) override {
        const uint16_t dshot = throttleToDShot(throttle_0_1);
        dshot_dma_push(motor, dshot);
    }
};
```

Mixer math (`mixMotors`) stays in `FlightController` — it is already airframe-correct for X quads.

### Step 7 — Keep sim as regression harness

`SimulationEngine` implements HAL backends:

- `SimIMU` wraps `Sensors::IMU` (feeds physics truth into noise model)
- `SimMotorOutput` calls `Drone::applyControl`
- `SimEstimator` can optionally use perfect physics for "ideal" regression mode via config flag `sim.perfect_state = true`

This lets CI compare PID tuning before and after estimator changes.

---

## What stays simulation-only

| Module | Reason |
|--------|--------|
| `PhysicsEngine` | No physics on FC chip |
| `Renderer` | No display on FC |
| `HeadedSimulation` | Desktop UX |
| `Sensor` noise models | Real noise is on the silicon |
| Wind, ground collision | Environment model |

## What ports unchanged (mostly)

| Module | Notes |
|--------|-------|
| `PIDController` | Drop-in |
| `FlightMode` | Drop-in |
| `mixMotors` / X-frame mixer | Drop-in |
| `Config` | On MCU: compile-time defaults + flash params |
| `Math/Vector.h` | Replace Eigen on tiny MCUs with `platforms/stm32/math/` fixed-point optional |

---

## Eigen on embedded

Eigen works on STM32 F4+ with `-O2` and FPU enabled. For H7-class with EKF, keep Eigen.

For F1 / tight flash: extract `PIDController` math to scalar `float` without Eigen matrices. Do this only when linker size is a proven problem.

---

## Testing strategy

| Layer | Test |
|-------|------|
| PID | Existing `test_pid_controller` |
| FC + sim HAL | `test_simulation_engine` (extend) |
| Estimator | New `test_complementary_estimator` with recorded IMU CSV |
| Motor mixer | `test_mixer` — known inputs → expected motor outputs |
| HIL | Pi runs sim physics; STM32 runs real FC over UART injection |

---

## Timeline alignment

| Step | Roadmap phase | Effort |
|------|---------------|--------|
| HAL + VehicleState | Phase 0 | 1–2 weeks |
| FC rewire + sim estimator | Phase 0 | 1 week |
| STM32 IMU + DShot + RC | Phase 1 | 3–4 weeks |
| First tethered hover | Phase 1 | — |
| GPS + EKF + MAVLink | Phase 2 | 6–8 weeks |
| Missions | Phase 3 | 4+ weeks |

---

## Immediate next PRs (suggested)

1. **HAL headers** + `platforms/sim/` adapters (no behavior change)
2. **`VehicleState` + `ComplementaryEstimator`**; FC uses IMU euler in sim
3. **CMake `AEROCORE_TARGET`** stub for `firmware/stm32`
4. **`test_estimator`** with synthetic IMU data
5. **Remove ground-truth position** behind `sim.perfect_state` config flag

---

## Related docs

- [Roadmap](roadmap.md)
- [Embedded installation](embedded-installation.md)
- [Architecture](architecture.md)
