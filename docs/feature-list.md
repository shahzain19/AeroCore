# AeroCore Feature List

## Currently implemented

- Estimator-driven flight control loop with attitude and altitude feedback from the estimator instead of perfect-state injection by default.
- Basic GPS-style navigation feedback in simulation through a dedicated simulated GNSS backend.
- RC-link loss fail-safe handling that transitions the controller into FAILSAFE when the link is lost or stale.
- Extended regression coverage for physics, estimator behavior, GPS feedback, and RC failsafe handling.

## Near-term roadmap

- STM32 hardware backend scaffolding for RC input and motor output.
- More explicit pre-arm diagnostics and fail-safe reason reporting.
- GPS-backed position hold and return-to-home behavior in simulation and on embedded targets.
- Mission and fixed-wing expansion beyond the current baseline.
