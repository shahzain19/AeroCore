# Config Reference

This project uses a TOML-like format parsed by `Utilities::Config`.

## Example Files

- `config/simulation.toml` (multirotor baseline)
- `config/fixed_wing.toml` (fixed-wing baseline)

## Common Sections

## `[simulation]`

- `dt`: physics step in seconds.
- `target_altitude`: initial altitude target.
- `wind_x`, `wind_y`: world-frame wind components.

## `[physics]`

- `drag_coefficient`: drag model coefficient.

## `[pid]` and controller sections

Typical keys used by PID controllers:

- `kp`, `ki`, `kd`, `kff`
- `output_min`, `output_max`
- `integral_max`
- `derivative_filter`
- `back_calc_gain`

## Parser Behavior

- Lines beginning with `#` or `;` are ignored.
- Inline comments after values are supported (`value = 1.0 # comment`).
- Double-quoted string values are unwrapped.
- Unknown keys are ignored by subsystems unless explicitly queried.
