# AeroCore Roadmap — Real-World Flight Controller

This roadmap is grounded in what AeroCore **actually has today** and what a production multirotor flight controller **must** have before anyone straps it to a real aircraft. It is ordered by dependency and risk, not by feature glamour.

**Current state (v2.0):** A desktop simulator with solid PID cascades, mode FSM, motor mixing, and virtual sensors. The flight controller still reads **perfect attitude from physics** and **ground-truth GPS position** for position modes. There is no ESC output, no RC receiver, no state estimator, and no MAVLink.

---

## Guiding principles

1. **Fly safe first** — arming checks, failsafes, and motor cut logic matter more than mission planning.
2. **Estimator before autonomy** — position hold without a real position estimate is a sim cheat, not a feature.
3. **One loop, one clock** — the control task must run at a fixed rate (typically 1–4 kHz inner loop, 250–500 Hz outer) with deterministic timing on hardware.
4. **Sim stays useful** — every hardware feature gets a sim backend so you can test before flashing silicon.
5. **Incremental ports** — STM32 F4/H7 first (industry standard for FCs), then ESP32 companion, then Linux SBC for development.

---

## Phase 0 — Foundation (in progress)

**Goal:** Split portable control logic from simulation-only code.

| Task | Why | Status |
|------|-----|--------|
| HAL interfaces (`IMU`, `Baro`, `GPS`, `RC`, `MotorOutput`, `Clock`) | Same FC code on sim and MCU | Started — see `include/HAL/` |
| `VehicleState` struct fed by estimator, not physics | Removes ground-truth cheat | Planned |
| CMake target `AEROCORE_TARGET=sim\|firmware` | Separate sim binary from firmware | Planned |
| FlightController reads IMU attitude, not `drone_->getEulerAngles()` | Biggest sim-vs-reality gap | Planned |
| Remove SFML dependency from headless/firmware path | SFML is not for MCUs | Planned |

**Exit criteria:** Sim builds with HAL backends; FC uses complementary-filter roll/pitch from IMU; unit tests still pass.

---

## Phase 1 — Minimum viable real FC (MVP)

**Goal:** Hover a small quad indoors with manual RC. No GPS, no missions.

### 1.1 Hardware I/O

| Component | Requirement |
|-----------|-------------|
| **IMU** | MPU-6050 or ICM-42688-P over SPI (preferred) or I2C |
| **Barometer** | BMP280 or MS5611 for altitude hold |
| **RC input** | CRSF or SBUS UART parser |
| **Motor output** | DShot600 (preferred) or 400 Hz PWM to ESCs |
| **Buzzer/LED** | Arming and error indication |

### 1.2 State estimation (minimum)

| Feature | Detail |
|---------|--------|
| Gyro integration | Body rates → quaternion at 1 kHz+ |
| Accel tilt correction | Complementary or Mahony filter (already stubbed in `Sensors::IMU`) |
| Baro fusion | Low-pass + complementary for altitude rate |
| Gyro bias learning | Static bias estimate when disarmed on level surface |

**Not in MVP:** GPS, magnetometer fusion, EKF.

### 1.3 Safety

| Feature | Detail |
|---------|--------|
| Pre-arm checks | Level within ±15°, throttle low, IMU/baro healthy |
| Disarm on impact | Accel spike detection |
| Failsafe: RC loss | Hold last mode 2 s → descend → disarm |
| Failsafe: low battery | Already in sim; wire to real ADC |
| `EMERGENCY_STOP` mode | Implement handler (currently falls through to disarm) |

### 1.4 Flight modes (MVP subset)

- `DISARMED`, `ARMED`
- `STABILIZE` (acro/rate)
- `ATTITUDE_HOLD`
- `ALTITUDE_HOLD`
- `FAILSAFE`

**Defer:** `TAKEOFF`/`LANDING` auto sequences (pilot controls throttle in MVP), `POSITION_HOLD`, `RTH`, `MISSION`.

### 1.5 Tuning and debug

| Feature | Detail |
|---------|--------|
| Blackbox | 1 kHz log to flash or SD (attitude, rates, motor cmds, RC) |
| USB CLI | Parameter read/write over CDC (before MAVLink) |
| Buzzer codes | Arm/disarm/failsafe patterns |

**Exit criteria:** 250 g–2 kg quad hovers in alt-hold outdoors with light wind; RC loss triggers safe descent; no ground-truth reads in FC.

---

## Phase 2 — Navigation and GCS

**Goal:** Outdoor autonomous position hold, return-to-home, and ground-station visibility.

### 2.1 GPS and magnetometer

| Sensor | Use |
|--------|-----|
| u-blox M8/M10 | Position, velocity, heading (dual-antenna optional later) |
| QMC5883L / LIS3MDL | Yaw correction when moving |

### 2.2 Estimator upgrade

- **EKF2-style** loosely coupled INS/GPS (start with 15-state: pos, vel, att, gyro bias)
- Position/velocity outer loops (complete the cascade documented in `FlightController.h` but not fully implemented)
- `requiresGPS()` modes gated on GPS fix quality (HDOP, satellite count, speed)

### 2.3 MAVLink

| Message set | Purpose |
|-------------|---------|
| HEARTBEAT, SYS_STATUS | Link health |
| ATTITUDE, LOCAL_POSITION_NED | Telemetry |
| RC_CHANNELS, MANUAL_CONTROL | Override |
| PARAM_VALUE / PARAM_SET | Tuning |
| MISSION_ITEM, MISSION_ACK | Waypoints (Phase 3) |
| COMMAND_LONG | Arm, disarm, RTH |

Target GCS: **QGroundControl** compatibility for basic operations.

### 2.4 Flight modes

- `POSITION_HOLD` with real GPS
- `RETURN_HOME` with configurable RTL altitude and loiter
- `LANDING` with controlled descent rate

**Exit criteria:** QGC shows live attitude and position; POS_HOLD holds ±1 m in calm conditions; RTH returns within 3 m of home.

---

## Phase 3 — Missions and fixed-wing

**Goal:** Waypoint missions for multirotor; basic FBW for fixed-wing.

### 3.1 Mission manager

- Waypoint storage in flash (max 50–100 points)
- States: upload, validate, execute, pause, resume
- Commands: NAV_WAYPOINT, NAV_TAKEOFF, NAV_LAND, LOITER_TIME
- Geofence (cylindrical) — hard requirement before public mission release

### 3.2 Fixed-wing (separate control path)

Current `fixed_wing.toml` only changes mass/motor count. Real FW needs:

| Subsystem | Detail |
|-----------|--------|
| Aerodynamics | Lift/drag vs α, stall model (table or linearized) |
| Control surfaces | Aileron, elevator, rudder, throttle mixing |
| Airspeed sensor | Pitot or GPS groundspeed fallback |
| Modes `FBW_A`, `FBW_B` | Implement handlers (enum exists, logic does not) |
| Takeoff/landing logic | Hand launch vs runway; approach path |

**Exit criteria:** Multirotor completes 5-waypoint mission in sim and on bench; fixed-wing FBW_A holds attitude in sim with dedicated physics branch.

---

## Phase 4 — Production hardening

**Goal:** Field-ready firmware, not lab prototype.

| Area | Work |
|------|------|
| **Watchdog** | Independent IWDG reset on control-loop stall |
| **Redundant sensors** | Dual IMU voting (optional, competition-grade) |
| **ESC telemetry** | RPM, current, temperature (BLHeli_32 / AM32) |
| **OSD** | MSP or MAVLink → DJI / HDZero |
| **Parameter profiles** | Save/load tune sets per airframe |
| **CI on real HIL** | Sim loop + optional hardware-in-the-loop runner |
| **Conformal testing** | SITL regression baselines (position, attitude step responses) |

---

## What we are explicitly NOT building (yet)

These are common "dream features" that do not belong on the roadmap until Phase 2–3 are stable:

- Swarm coordination
- AI obstacle avoidance
- Full CFD/aero tables
- Certified DO-178C avionics
- VTOL tilt-rotor blending
- Optical-flow indoor nav without rangefinder
- Cloud fleet management

---

## Priority matrix

| Priority | Feature | Blocks |
|----------|---------|--------|
| P0 | HAL + remove ground-truth attitude | Everything on hardware |
| P0 | DShot/PWM motor output | First flight |
| P0 | RC parser (CRSF/SBUS) | First flight |
| P0 | Pre-arm + RC-loss failsafe | Safe first flight |
| P1 | Baro altitude hold on real hardware | Useful outdoor flight |
| P1 | Blackbox logging | Tuning without guessing |
| P2 | GPS + EKF | POS_HOLD, RTH |
| P2 | MAVLink | Field tuning |
| P3 | Mission manager | Autonomous ops |
| P3 | Fixed-wing FBW | FW airframes |

---

## Success metrics

| Milestone | Metric |
|-----------|--------|
| Bench spin | Motors respond to stick in STABILIZE; disarm cuts within 10 ms |
| Tethered hover | Alt-hold ±0.5 m for 60 s |
| Outdoor manual | Attitude-hold in 15 km/h wind without pilot correction |
| GPS nav | POS_HOLD ±1.5 m CEP |
| Mission | 1 km circuit, 5 waypoints, RTL on RC switch |

---

## Related docs

- [Sim → Production migration](sim-to-production.md)
- [Embedded installation](embedded-installation.md)
- [Architecture](architecture.md)
- [Capabilities and limitations](capabilities-and-limitations.md)
