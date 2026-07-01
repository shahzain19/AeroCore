# Installing AeroCore on Hardware

This guide covers how to build and deploy AeroCore for **desktop simulation** (works today) and **embedded targets** (HAL scaffolding in place; full firmware ports follow the [roadmap](roadmap.md)).

AeroCore is not a single binary for every board. The architecture is:

```
┌─────────────────────────────────────────┐
│  FlightController + PID + FlightMode    │  ← portable core
├─────────────────────────────────────────┤
│  HAL (IMU, Baro, GPS, RC, Motors, Clock)│  ← platform interface
├──────────────┬──────────────┬───────────┤
│  Sim backend │  STM32 FW    │  ESP32 /  │
│  (desktop)   │  (firmware)  │  Linux    │
│              │              │  SBC      │
└──────────────┴──────────────┴───────────┘
```

The current repo state is:

- Desktop simulator: fully supported and runnable.
- Firmware scaffolds: `AEROCORE_TARGET=stm32|linux-sbc|esp32` can configure and build the portable core library, but board-specific drivers and final firmware images are still under development.
- Flight controller HAL routing: `FlightController` now supports optional motor command forwarding through `HAL::IMotorOutput`, simplifying future ESC/hardware bindings.

---

## What works today

| Target | Status | Command |
|--------|--------|---------|
| Linux / macOS / Windows sim | **Supported** | `cmake -DAEROCORE_TARGET=sim .. && cmake --build .` |
| Headless sim (CI, tuning) | **Supported** | `./AeroCore --headless` |
| STM32 / ESP32 / RP2040 firmware | **Not yet** — use steps below when `firmware/` targets land | — |

---

## Desktop simulation (all platforms)

### Dependencies

| Package | Ubuntu/Debian | Fedora | macOS (Homebrew) |
|---------|---------------|--------|------------------|
| CMake ≥ 3.22 | `cmake` | `cmake` | `cmake` |
| C++20 compiler | `g++` / `clang++` | same | Xcode CLT |
| Eigen3 | `libeigen3-dev` | `eigen3-devel` | `eigen` |
| SFML 2.6 | `libsfml-dev` | `SFML-devel` | `sfml` |

### Ubuntu / Debian

```bash
sudo apt update
sudo apt install -y build-essential cmake libeigen3-dev libsfml-dev
git clone https://github.com/<your-org>/AeroCore.git
cd AeroCore
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j"$(nproc)"
./AeroCore --help
```

### Fedora

```bash
sudo dnf install -y gcc-c++ cmake eigen3-devel SFML-devel
# then same clone/build steps as above
```

### macOS

```bash
brew install cmake eigen sfml
# clone, mkdir build, cmake, build as above
```

### Windows (MSVC + vcpkg)

```powershell
vcpkg install eigen3 sfml
cmake -B build -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
build\Release\AeroCore.exe --headless
```

---

## STM32 (F4 / F7 / H7) — recommended primary FC target

Most commercial flight controllers (Matek, Holybro, custom designs) use **STM32F405/F722/H743**. This is the target AeroCore firmware will prioritize.

### Recommended hardware

| Board class | MCU | Notes |
|-------------|-----|-------|
| Dev breakout | STM32F405 (Black Pill) + external IMU | Cheapest bring-up |
| FC clone | Matek F405-SE, SpeedyBee F405 | Integrated IMU, baro, OSD pad |
| Performance | STM32H743 | 480 MHz, more RAM for EKF |

### Toolchain

**Option A — ARM GCC (recommended)**

```bash
# Ubuntu/Debian
sudo apt install -y gcc-arm-none-eabi binutils-arm-none-eabi
arm-none-eabi-gcc --version
```

**Option B — PlatformIO**

```bash
pip install platformio
# firmware/stm32/platformio.ini (when added)
cd firmware/stm32
pio run -e stm32f405
pio run -t upload
```

### Pin map (typical F405 FC — verify your board schematic)

| Function | Resource | Notes |
|----------|----------|-------|
| IMU (ICM-42688) | SPI1 (PA5/6/7, CS PA4) | 8 MHz SPI |
| Baro (BMP280) | I2C1 (PB6/PB7) | 400 kHz |
| RC (CRSF) | USART3 RX (PB11) | Inverted UART on some boards |
| Motors 1–4 | TIM1 CH1–4 | DShot via DMA bit-bang or timer |
| USB CLI | USB OTG FS | CDC for params |
| Buzzer | PA15 or PC15 | Active low |

### Flash procedure (ST-Link)

```bash
# After firmware build produces build/aerocore_fc.bin
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/aerocore_fc.bin 0x08000000 verify reset exit"
```

Or with `dfu-util` if the board boots in DFU mode (BOOT0 high).

### First-time checklist

1. **Remove props** until motor direction and arming are verified.
2. Connect USB — expect serial CLI or MAVLink (when enabled).
3. Place board level; run `status` — IMU should read ~0° roll/pitch.
4. Arm only with throttle at minimum and level check passing.
5. Spin motors one at a time at 10% throttle; verify order matches mixer (FR, RL, FL, RR for X quad).

### Expected timeline

Full STM32 port is **Phase 0–1** on the [roadmap](roadmap.md). Until `firmware/stm32/` is merged, use the desktop sim for control-loop development.

---

## ESP32 (S3 / C3) — companion / Wi-Fi telemetry

ESP32 is **not ideal** as the sole 1 kHz rate-loop MCU (Wi-Fi stack jitter), but it works well as:

- Wi-Fi MAVLink bridge to QGroundControl
- Blackbox Wi-Fi download
- Secondary board on a split FC design

### Toolchain (ESP-IDF)

```bash
# Ubuntu
sudo apt install -y git wget flex bison gperf python3 python3-pip \
  cmake ninja-build ccache libffi-dev libssl-dev dfu-util
mkdir -p ~/esp && cd ~/esp
git clone -b v5.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32s3
. ~/esp/esp-idf/export.sh
```

### Build (when `firmware/esp32/` exists)

```bash
cd firmware/esp32
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### Wiring to STM32 FC

| ESP32 | STM32 |
|-------|-------|
| UART TX | UART RX (MAVLink at 57600 or 921600) |
| GND | GND |
| Optional SPI | High-speed sensor bus (split design only) |

---

## Raspberry Pi / Linux SBC — development and HIL

A Pi 4/5 or Orange Pi can run AeroCore **without SFML** as a soft real-time FC for bench testing, or as a Software-in-the-Loop host talking to real ESCs via GPIO PWM (lab use only).

### Cross-compile on Pi (native build)

```bash
sudo apt install -y build-essential cmake libeigen3-dev
git clone <repo> && cd AeroCore
mkdir build && cd build
cmake -DAEROCORE_TARGET=linux-sbc -DAEROCORE_BUILD_SIM=OFF ..
cmake --build . -j4
```

### GPIO PWM (experimental — not for untethered flight)

| Interface | Library | Notes |
|-----------|---------|-------|
| 4× PWM outputs | `libgpiod` + DMA PWM kernel overlay | Jitter ~100 µs; OK for bench |
| RC input | UART on `/dev/ttyAMA0` | SBUS inverter required |
| IMU | I2C `/dev/i2c-1` | MPU-6050 at 0x68 |

```bash
sudo apt install -y libgpiod-dev i2c-tools
sudo usermod -aG gpio,i2c $USER
# reboot, then verify:
i2cdetect -y 1
```

### systemd service (headless FC on boot)

```ini
# /etc/systemd/system/aerocore.service
[Unit]
Description=AeroCore Flight Controller
After=network.target

[Service]
ExecStart=/usr/local/bin/aerocore-fc --config /etc/aerocore/airframe.toml
Restart=on-failure
Nice=-10

[Install]
WantedBy=multi-user.target
```

---

## RP2040 (Pico) — education / light quad

RP2040 can run ~500 Hz control loops with PIO-based DShot. Good for learning; tight RAM for full EKF.

```bash
sudo apt install -y gcc-arm-none-eabi libnewlib-arm-none-eabi
# Pico SDK:
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk && git submodule update --init
export PICO_SDK_PATH=$PWD
# Build when firmware/rp2040/ is available
```

---

## Teensy 4.1 — rapid prototyping

Teensyduino or bare-metal ARM GCC. Excellent DMA, FPU, low jitter. Pin assignments vary by carrier board.

```bash
# PlatformIO
pio run -e teensy41
pio run -t upload
```

---

## Sensor wiring reference

### MPU-6050 / ICM-42688 (SPI)

| IMU pin | MCU |
|---------|-----|
| VCC | 3.3 V |
| GND | GND |
| SCK | SPI SCK |
| MISO | SPI MISO |
| MOSI | SPI MOSI |
| CS | GPIO (active low) |
| INT | GPIO (data-ready, optional) |

### BMP280 (I2C)

| BMP280 | MCU |
|--------|-----|
| VCC | 3.3 V |
| GND | GND |
| SCL | I2C SCL + 2.2 kΩ pull-up |
| SDA | I2C SDA + 2.2 kΩ pull-up |

Address: `0x76` or `0x77` depending on SDO pin.

### CRSF receiver

| CRSF | FC UART |
|------|---------|
| TX | RX |
| RX | TX (optional) |
| GND | GND |
| VCC | 5 V (check receiver spec) |

Baud: **420000** 8N1. Some boards need hardware UART inversion.

---

## ESC protocol quick reference

| Protocol | Rate | Wiring |
|----------|------|--------|
| PWM | 50–490 Hz | Standard 1000–2000 µs |
| Oneshot125 | 1–2 kHz | Same wire, faster pulse |
| DShot150/300/600 | Digital | Single wire per ESC, bidirectional telemetry on DShot600+ |

AeroCore firmware will output **DShot600** by default on STM32; sim continues to use normalized throttle 0–1.

---

## Configuration on hardware

Desktop sim uses TOML files in `config/`. Embedded targets will use:

| Storage | Format | Tool |
|---------|--------|------|
| Flash params | Binary struct + CRC | USB CLI / MAVLink PARAM |
| SD card | `airframe.toml` | Copy from `config/simulation.toml` as template |

Key parameters to tune per airframe:

- `[pid_*]` — rate and angle gains
- `[flight]` — max angles, rates
- `[drone]` — mass, arm length (for feed-forward)
- `hover_throttle` — critical for alt-hold

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| Build fails: Eigen not found | Missing dev package | Install `libeigen3-dev` |
| IMU reads all zeros | Wrong SPI mode / CS | Mode 3, 8 MHz; check wiring |
| Motors twitch at arm | Gyro noise / no calibration | Disarm; leave still 5 s for bias cal |
| Alt-hold oscillates | Baro noise or P too high | Lower `pid_altitude.kp`, add D |
| RC not detected | Inverted UART | Hardware inverter or `UART_INVERT` flag |
| Control loop slow | Debug printf in hot path | Use blackbox, not `printf` in loop |

---

## Safety notice

**Do not fly untested firmware on a real aircraft without:**

- Prop-off motor verification
- Tether or net for first hover
- Working RC-loss failsafe
- Fire extinguisher nearby for LiPo tests

AeroCore firmware is experimental open-source software with **no airworthiness certification**.

---

## See also

- [Roadmap](roadmap.md) — when each target becomes supported
- [Sim → Production](sim-to-production.md) — code structure for ports
- [Build and run](build-and-run.md) — desktop sim details
- [Config reference](config-reference.md) — tuning parameters
