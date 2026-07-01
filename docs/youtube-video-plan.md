# AeroCore YouTube Video Plan (9-10 Minutes)

This is a practical production blueprint for a high-quality, curiosity-driven long-form video about AeroCore.  
Target style: clean, cinematic, educational, fast pacing, strong narrative, and minimal motion graphics — think Veritasium/Fern clarity without overproduction.

---

## 1) Video Goal

### Primary objective

Show how AeroCore works as a flight-controller simulator, why it matters, what you built, and what is still missing.

### Viewer promise

By the end, viewers should understand:

- what the simulator can do now,
- how the control loop behaves in practice,
- what engineering trade-offs you made,
- what next milestones are.

### Ideal audience

- engineering students
- robotics/drone hobbyists
- C++/simulation developers
- tech-curious viewers who enjoy seeing systems explained clearly

---

## 2) Core Story Arc

Use this simple narrative:

1. **Hook**: "Can we make a realistic flight controller simulator from scratch?"
2. **Conflict**: control systems are noisy, unstable, and hard to tune.
3. **Build journey**: physics + sensors + PID + telemetry + headless testing.
4. **Result**: demo of stable behavior and measurable outputs.
5. **Honest limits**: what it cannot do yet.
6. **Forward pull**: what comes next and invite collaboration.

---

## 3) Time-Coded Scene Plan (9:30 target)

## 0:00-0:30 - Cold Open Hook

### What to show

- Fast montage: console headless output, flight HUD, code snippets, takeoff transition.
- One strong line on screen: "A C++ flight controller simulator that actually flies."

### Voiceover direction

"I built AeroCore to test flight-control logic before touching real hardware."

---

## 0:30-1:15 - Problem Setup

### What to explain

- Real drones are expensive/risky to tune physically.
- Simulators reduce crash cost and speed iteration.

### Visuals

- Whiteboard sketch of control loop.
- Simple animation: desired altitude -> controller -> motors -> physics -> sensors -> back to controller.

---

## 1:15-2:30 - Architecture Walkthrough

### What to cover

- Subsystems:
  - `Flight` (controller, motor, drone)
  - `Physics` (RK4 integration)
  - `Sensors` (IMU/altimeter/battery)
  - `Simulation` (telemetry)
  - `Rendering` (GUI path)
  - `Utilities` (config/logging)

### Visuals

- Repo tree zoom.
- Diagram overlays with arrows between modules.

### Keep this concise

Do not deep-dive every class. Focus on flow.

---

## 2:30-3:45 - Headless Demo (Fast Feedback)

### What to show live

Run:

```bash
./AeroCore --headless --duration 60 --status-rate 60
```

Then explain:

- high-frequency status lines,
- mode transitions (`DISARMED -> ARMED -> TAKEOFF -> ALT_HOLD`),
- stable simulation cadence.

### Visual treatment

- Keep visuals simple and direct: zoom, callouts, and text highlights, not motion graphics.
- Speed-ramp longer terminal/code shots so the pace feels crisp without relying on animation.

---

## 3:45-5:10 - Control Logic Deep Dive (One Main Idea)

### Main teaching point

Why PID + anti-windup + derivative filtering matter.

### What to show

- Excerpt from `PIDController` behavior.
- Simple graph animation:
  - setpoint,
  - measured altitude,
  - controller output trend.

### Suggested wording

"Without anti-windup, the controller accumulates error during saturation and overshoots badly."

---

## 5:10-6:20 - Configurability and Repeatability

### What to show

- `config/simulation.toml` and `config/fixed_wing.toml`.
- Running with alternate config.
- Mention deterministic-ish repeatable runs for debugging.

### Message

Simulation is useful when changes are measurable and reproducible.

---

## 6:20-7:15 - Testing and Engineering Discipline

### What to show

- test files in `tests/`
- command:

```bash
ctest --output-on-failure
```

- all tests passing

### Why this scene matters

Signals credibility: this is not just a visual toy.

---

## 7:15-8:30 - What It Cannot Do Yet (Trust-Building Scene)

### Be explicit

- no full EKF/state estimator stack yet
- no complete waypoint mission navigation
- fixed-wing advanced autopilot behavior is partial
- limited integration/regression test depth

### Tone

Confident and honest, not apologetic.

---

## 8:30-9:20 - Roadmap + Call to Action

### Roadmap bullets

1. integration tests for controller-physics interactions
2. fuller mission modes
3. improved CLI/help and validation
4. scenario benchmarks

### CTA

- star/follow repo
- suggest scenarios in comments
- invite contributors for estimator/nav modules

---

## 9:20-9:40 - Ending Shot

- cinematic replay of GUI + telemetry overlay
- short sign-off line:
  - "Sim first. Fly safer. Build faster."

---

## 4) Shot List (Production Checklist)

## Screen capture shots
Use real screen captures and minimal overlays. Avoid full-motion animated sequences; instead, rely on tight edits, gradual reveals, and clear text callouts.
- build and run commands
- headless output with mode transitions
- GUI render moments
- test run summary
- docs view (`capabilities-and-limitations.md`)

## B-roll / overlays

- architecture diagram
- control loop animated arrows
- simple PID response graph
- lower-thirds for terms (RK4, anti-windup, telemetry)

## On-camera (optional but recommended)

- 2-3 short face-to-camera moments:
  - intro hook
  - limitations honesty segment
  - roadmap/CTA

---

## 5) Narration Style Guide

- Use short sentences.
- Explain one concept per beat.
- Avoid jargon stacking.
- Every 20-30 seconds: visual change or new question.
- Prioritize "why this matters" over raw implementation detail.

---

## 6) Editing Blueprint

## Pacing

- Average shot length: 3-6 seconds.
- Terminal/code shots: max 8-10 seconds before zoom/highlight cut.
- Use jump cuts aggressively on pauses.

## Audio

- Light background music (low in mix).
- Duck music under technical explanation.
- Use subtle risers for transitions only.

## Motion graphics

- Keep typography minimal and consistent.
- One accent color for highlights.
- No heavy particle/intense transitions; keep engineering clean.

---

## 7) Thumbnail + Title Options

## Thumbnail ideas

- Left: code/terminal, Right: drone HUD frame, center text: "I Built a Flight Controller Simulator"
- Or: "SIM FIRST, FLY LATER" over control-loop diagram

## Title options

- "I Built a C++ Flight Controller Simulator (And Tested It for Real Stability)"
- "Can You Simulate a Drone Flight Controller from Scratch?"
- "Inside AeroCore: Physics, PID, and a Real-Time Drone Sim"

---

## 8) What To Avoid

- Do not spend 2+ minutes on setup commands.
- Do not over-explain every class.
- Do not hide limitations; show them clearly.
- Do not use loud effects that distract from technical clarity.

---

## 9) Final 24-Hour Publish Checklist

- [ ] Confirm all commands shown still work exactly as recorded.
- [ ] Re-run headless and test scenes for fresh footage.
- [ ] Verify on-screen code paths match current repo.
- [ ] Add repo link + docs links in description.
- [ ] Add chapter timestamps.
- [ ] Pin comment with roadmap and contributor asks.

---

## 10) Optional Chapter Timestamps (for YouTube description)

```text
00:00 Hook
00:30 Why this simulator exists
01:15 AeroCore architecture
02:30 Headless run demo
03:45 PID/control deep dive
05:10 Config and repeatability
06:20 Tests and validation
07:15 Current limitations
08:30 Roadmap
09:20 Wrap up
```

Use this plan as your production script skeleton, then adapt wording to your speaking style.
