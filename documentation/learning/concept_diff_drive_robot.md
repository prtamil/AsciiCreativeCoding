# Pass 1 — diff_drive_robot.c: Two wheels, three numbers, one robot

## Core Idea

A robot with two wheels on a common axle, each wheel independently driven, no other freedom. State is a pose `(x, y, θ)` — two pixels and one angle. The whole simulation is two functions:

- `compute_wheels(v, ω)` → `(vL, vR)` — INVERSE kinematics. "I want the centre to move at v and turn at ω; how fast must each wheel spin?"
- `step_pose(pose, vL, vR, dt)` → new pose — FORWARD kinematics + Euler integration. "Given how fast each wheel is spinning, where am I after dt seconds?"

Plus a 600-slot ring buffer recording past positions for a fading trail. That's it. No solver, no obstacles, no obstacles, no lookups — pure 2D differential-drive kinematics.

## The Mental Model

A coffee cup on a tray with two parallel rollers underneath. Spin the rollers at the same speed → tray glides forward. Spin them at *opposite* speeds → tray pivots in place. Spin them at slightly different speeds → tray traces an arc whose radius is `R = v/ω` (the **Instantaneous Centre of Curvature**, ICC). Brake one wheel only → tray pivots about that wheel.

The constraint that makes a diff-drive robot a diff-drive robot — and not, say, a hovercraft — is that it cannot move sideways. This is called **nonholonomic**, but you don't need the word: it just means velocity is always along the heading, never perpendicular. Importantly, no penalty term, no Lagrange multiplier — the constraint is baked into the FK formula `(v cos θ, v sin θ)`. By construction.

## Data Structures

### Robot (§6.1)
```
float x, y;          — pose position (pixels)
float theta;         — heading (radians)
float v, omega;      — current linear speed (px/sec) and angular rate (rad/sec)
float v_cmd, w_cmd;  — commanded targets (smoothed toward, not snap)
float vL, vR;        — last-computed wheel speeds (display only)
Trail trail;         — 600-slot ring buffer
```

### Trail ring buffer (§1.5)
```
Vec2 buf[TRAIL_CAP];
int  head;           — write index (advances each TRAIL_STEP ticks)
int  count;          — entries valid (saturates at TRAIL_CAP)
```

The ring buffer is the canonical "fixed memory, append-only history" pattern: writes overwrite the oldest entry, reads walk backward from `head`. `count` saturates so we don't keep growing past the buffer capacity.

## Key Formulas

```
INVERSE KINEMATICS  (compute_wheels)
  vL = v - ω · L/2
  vR = v + ω · L/2
        where L = WHEEL_BASE (axle width).

FORWARD KINEMATICS  (step_pose, Euler)
  v_avg = (vL + vR) / 2
  ω     = (vR - vL) / L
  x     += v_avg · cos(θ) · dt
  y     += v_avg · sin(θ) · dt
  θ     += ω                · dt

ICC RADIUS  (HUD only)
  R     = v / ω         (∞ when |ω| < 0.001 — straight line)
```

The integrate-then-rotate order (`x,y` first, then `θ`) is **first-order Euler**. Curvature error per step ≈ `O(ω·dt)`, which is invisible at 60 FPS for the speeds the demo uses. A more accurate "exact" integrator would compute `Δθ` first and apply circular-arc displacement; not worth the code clarity loss here.

## The Main Loop (variable dt)

1. Resize check.
2. `dt` = wall-clock since last frame, capped at `DT_CAP_SEC`.
3. Drain input → `app_handle_key` (sets `v_cmd` / `w_cmd`).
4. `robot_tick(dt)`:
   - Smooth `v` toward `v_cmd`, `omega` toward `w_cmd` (rate-limited).
   - `compute_wheels(v, omega)` → updates `vL`, `vR`.
   - `step_pose(...)` integrates Euler.
   - Wrap pose at screen edges (toroidal world).
   - Append `(x, y)` to trail every `TRAIL_STEP` ticks.
5. `scene_draw` paints trail → robot → HUD.
6. Frame cap.

## Non-Obvious Decisions

**Why two named functions `compute_wheels` and `step_pose`?**
Because they're the two halves of the conceptual model. IK answers "how do I drive?" FK answers "where am I now?" Naming them separately (and putting `compute_wheels` BEFORE `step_pose` even though `step_pose` doesn't call it) keeps the math thinkable. A learner can read just §6.2 and understand IK, then §6.3 for FK, without skipping past helper code.

**Why `V_DECAY = 1.0` (no automatic slowdown)?**
A real wheeled robot on flat ground has near-zero rolling resistance. Setting decay < 1.0 per tick at 60 Hz creates an exponential slowdown that makes the robot stop within a second of releasing the key — counterintuitive ("why is it stopping when no brake is applied?") and physically wrong. The dedicated `S` key sets `v_cmd = w_cmd = 0` for explicit braking.

**Why is the trail a ring buffer instead of a list?**
Bounded memory. At 60 FPS over a 10-minute session you'd otherwise be appending to a list 36000 times, with no upper bound. The ring buffer caps storage at exactly `TRAIL_CAP · sizeof(Vec2)` = 600 · 8 = 4800 bytes regardless of how long the program runs.

**Why is the world toroidal (wrap at edges)?**
Bounded screen + unbounded simulation = wrap. Otherwise the robot disappears off the right and the demo is over. Wrap teaches the trick of "interpolation suppression": when wrap happens, the trail's last sample and current pose are on opposite sides of the screen, so naïve interpolation would draw a streak across the entire viewport. The fix is to detect `|Δx| > world_width/2` and clamp the interp delta to zero.

**Why `compute_wheels` is named INVERSE not FORWARD kinematics?**
"Forward" runs from inputs (wheel speeds) to outputs (pose change). "Inverse" runs the other way: given a desired pose change, find the wheel speeds. The user's input is `(v, ω)` — desired centre velocity and turn rate — so going to wheel speeds is INVERSE. Using `step_pose` to consume `(vL, vR)` and produce a new pose is FORWARD.

## Key Constants

| Constant | Default | Effect |
|---|---|---|
| WHEEL_BASE | 18 px | Axle width L. Larger = wider turn radius for same ω. |
| V_MAX | 90 px/s | Top linear speed. |
| W_MAX | 2.4 rad/s | Top angular speed. |
| V_RAMP | 240 px/s² | How quickly v approaches v_cmd. |
| W_RAMP | 8 rad/s² | How quickly omega approaches w_cmd. |
| TRAIL_CAP | 600 | Ring buffer slots. ~10 sec of history at TRAIL_STEP=2. |
| TRAIL_STEP | 2 | Sample one position every N ticks. Larger = sparser trail. |

## Open Questions

- A second-order integrator (mid-point or "exact" arc) would let the robot turn faster without visibly drifting.
- A heading-error PID (point and shoot at a target) would make a great teaching extension — the current demo only models the bottom layer (kinematics), not a controller.
- The trail is currently single-stream; per-wheel trails would visualise wheel slip (when the wheels would have followed different arcs in a real robot).

---

# Pass 2 — Pseudocode

## Module Map

| § | Purpose |
|---|---|
| §1 config | frame rate, cell sizes, robot geometry, dynamics, trail, dt cap, ncurses pairs, HUD |
| §2 clock | monotonic timer + sleep |
| §3 color | body, wheel, trail, HUD pairs |
| §4 coords | pixel ↔ cell conversion |
| §5 (omitted — diff_drive uses §6 directly for physics) |
| §6 robot | (6.1) types (6.2) compute_wheels IK (6.3) step_pose FK + Euler (6.4) wrap (6.5) init (6.6) tick |
| §7 screen | ncurses init / present |
| §8 render | (8.1) bounds (8.2) dotted line (8.3) tip char (8.4) wheel arrow (8.5) trail (8.6) robot (8.7) HUD (8.8) scene_draw |
| §9 app | signals, resize, variable-dt main loop |

## Data Flow

```
keys → app_handle_key → robot.{v_cmd, w_cmd, paused}
                            │
clock_ns → dt → robot_tick(r, dt)
                     ├── ramp v toward v_cmd, omega toward w_cmd
                     ├── compute_wheels(v, omega)         ← IK
                     │      vL = v - ω·L/2
                     │      vR = v + ω·L/2
                     ├── step_pose(...)                    ← FK + Euler
                     │      x += v_avg·cos(θ)·dt
                     │      y += v_avg·sin(θ)·dt
                     │      θ += ω·dt
                     ├── wrap_position (toroidal)
                     └── trail_push every TRAIL_STEP ticks
                            │
                            ▼
                       scene_draw
                          trail (faded) → robot (body+arrow+wheels) → HUD
```

## Pseudocode

```
setup:
  install signals
  screen_init, color_init
  robot_init(cols, rows)

while running:
  if need_resize: re-init geometry
  dt = clamp(now - last, 0, DT_CAP_SEC)
  drain input → app_handle_key
  robot_tick(r, dt, cols, rows)
  fps update
  erase
  scene_draw
  wnoutrefresh + doupdate
  sleep to TARGET_FPS
```

## Key Patterns to Internalize

**Two named functions = two named ideas.** Keep `compute_wheels` (IK) and `step_pose` (FK) as separate top-level functions. Don't fold them into a single `update()` — the names ARE the documentation.

**Constraint by construction, not by penalty.** The nonholonomic constraint "no sideways motion" is encoded by writing `(v cos θ, v sin θ)` as the velocity. No projection step needed.

**Ring-buffer trail.** Bounded storage with append semantics is the canonical recipe for ANY history-of-positions feature. Memorise the (head, count, mod) idiom.

**Wrap interpolation suppression.** When wrapping a toroidal world, interpolating between the previous and current frame draws a streak across the screen. Test `|Δ| > world/2` and clamp.

**Physical units beat tick units.** v in px/sec, ω in rad/sec — verifiable with a stopwatch ("at 90 px/s, the robot crosses a 200-cell screen in ~14 sec"). px/tick at SIM_FPS=60 forces mental conversion to even sanity check.
