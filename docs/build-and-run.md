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
cmake ..
cmake --build . -j"$(nproc)"
```

## Run (GUI)

```bash
./AeroCore
```

## Run (Headless)

```bash
./AeroCore --headless
```

Useful options:

- `--duration <seconds>`: stop after configured simulation time (default `60`).
- `--status-rate <hz>`: status line print rate in headless mode (default `5`).
- `--debug-headless`: enable early detailed debug prints.

You can also pass a config file path:

```bash
./AeroCore config/fixed_wing.toml --headless --duration 20
```
