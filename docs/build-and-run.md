# Build and Run

## Dependencies

- CMake 3.22+
- C++20 compiler (GCC/Clang/MSVC)
- Eigen3
- SFML (Graphics, Window, System)

## Build

```bash
mkdir -p build
cd build
cmake -DAEROCORE_TARGET=sim ..
cmake --build . -j"$(nproc)"
```

`AEROCORE_TARGET=sim` is the default simulator build. Use firmware-style targets when you want to build the portable core library without the GUI runner:

```bash
mkdir -p build-stm32
cd build-stm32
cmake -DAEROCORE_TARGET=stm32 -DAEROCORE_BUILD_TESTS=OFF ..
cmake --build . -j"$(nproc)"
```

## Run (GUI)

```bash
./AeroCore
```

Show CLI usage:

```bash
./AeroCore --help
```

## Run (Headless)

```bash
./AeroCore --headless
```

Useful options:

- `--duration <seconds>`: stop after configured simulation time (default `60`).
- `--status-rate <hz>`: status line print rate in headless mode (default `5`).
- `--debug-headless`: enable early detailed debug prints.
- `--no-perfect-state`: disable perfect-state simulator state injection and exercise the estimator with noisy simulated sensors.

## Run tests

From the build directory:

```bash
cmake -DAEROCORE_BUILD_TESTS=ON ..
cmake --build . -j"$(nproc)"
ctest --output-on-failure
```

You can also run a specific unit test directly:

```bash
./test_motor_output
```

You can also pass a config file path:

```bash
./AeroCore config/fixed_wing.toml --headless --duration 20
```
