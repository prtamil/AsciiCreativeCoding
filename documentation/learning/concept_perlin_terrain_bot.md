# Pass 1 — perlin_terrain_bot.c: Inverted-pendulum cart-pole on Perlin terrain, stabilised by PID

## Core Idea

A self-balancing single-wheel robot rolling across infinite Perlin-noise terrain, kept upright by a PID controller running on top of a Lagrangian cart-pole physics model. Three independent, swappable view modes let a learner watch what's happening from three angles:

- **Telemetry** — live numerical readouts and bar gauges (θ, ω, control u, errors).
- **Equations** — the math itself with current values plugged in, refreshing every frame.
- **Phase portrait** — a 2D plot of `(θ, ω)` showing the trajectory through state space.

The physics is the canonical inverted-pendulum-on-a-cart problem (Goldstein §1.4, Åström & Murray §3.2), but on a *sloped* surface — gravity decomposes into along-slope and perpendicular-to-slope components, and the perpendicular component leaks into the pendulum's tipping torque.

## The Mental Model

Imagine balancing a broomstick upright on your palm. To keep it from falling, you constantly slide your palm in the direction the broomstick is starting to lean. The faster it leans, the more you have to slide; if it leans far enough, no realistic palm motion can save it. That's the *control problem* the PID solves: read tilt error → output palm acceleration → trust the cart-pole physics to translate that into wheel torque → pendulum stays up.

Now tilt the floor. The broomstick now wants to fall *toward the downhill direction* even when you're standing still — gravity itself is helping push it over. You need to anticipate that: bias your palm acceleration into the slope as a feed-forward term, BEFORE the tilt error even develops. That's the **slope feed-forward** — adding a constant `K_SLOPE · slope_angle` directly to the controller output.

Putting both together: the PID corrects the tilt error after it shows up; the slope feed-forward prevents most of the error from ever developing on uneven terrain. The two together let the bot navigate Perlin-noise hills without falling over.

## Data Structures

### Bot (§6.2)
```
float x_pix, y_pix;       — wheel centre position (pixels)
float theta;              — body tilt from local-vertical (rad)
float omega;              — angular velocity (rad/sec)
float v;                  — horizontal velocity (px/sec)
PID   pid;                — { kp, ki, kd, integral, prev_err }
float drive_speed;        — commanded forward speed (cycle via 'd' key)
ViewMode view;            — TELEMETRY / EQUATIONS / PHASE
GainPreset preset_idx;    — current PID preset (0..5)
PhaseHistory history;     — ring buffer of (θ, ω) for phase plot
bool paused, single_step;
```

### PID controller (§6.3)
Standard textbook form with **anti-windup clamp** on the integral:
```
err     = setpoint - measured
P       = kp · err
I       = clamp(I + ki · err · dt,  -I_LIMIT,  +I_LIMIT)   ← anti-windup
D       = kd · (err - prev_err) / dt
output  = P + I + D + slope_feed_forward
```

`prev_err` and `integral` are the only state. The clamp is critical: without it, integral builds up indefinitely on long terrain slopes and the controller takes forever to recover after a disturbance.

### Phase history ring buffer (§6.5)
```
Vec2 buf[PHASE_HIST_CAP];   — (θ, ω) pairs
int  head, count;
```
Same canonical ring-buffer pattern as the trail in `diff_drive_robot.c`.

## Key Formulas

```
SLOPE FROM TERRAIN
  slope = atan2(h(x+dx) - h(x-dx),  2·dx)         ← finite difference

PID CONTROL  (pid_step)
  err     = θ_target - θ          (target = -slope · K_SLOPE_TARGET, plus 0)
  output  = kp·err + ki·∫err + kd·dérr - K_SLOPE_FF · slope

CART-POLE LAGRANGIAN  (cart_pole_step, simplified for unit masses)
  α       = (gravity·sin(θ) - control·cos(θ)) / pole_length
                                  ← angular accel of pole
  ω      += α · dt
  θ      += ω · dt
  v      += control · dt
  x_pix  += v · dt

SUB-STEPPING
  for n in 0..SUBSTEP_COUNT-1:
    bot_substep(dt / SUBSTEP_COUNT)
                                  ← cart-pole is stiff at large dt;
                                    sub-step protects against blow-up.
```

`SUBSTEP_COUNT = 4` (or higher) at 60 FPS gives an inner timestep of ~4 ms — enough to keep the cart-pole stable for `θ` up to ~1 rad before the linearisation breaks down. Without sub-stepping, large dt + large θ causes integration to diverge in one frame.

## The Main Loop (variable dt)

1. Resize check.
2. `dt` = wall-clock since last frame, capped at `DT_CAP_SEC`.
3. Drain input → `app_handle_key` (cycle preset, cycle drive_speed, change view, toggle pause).
4. For `n` in 0..SUBSTEP_COUNT-1: `bot_substep(dt / SUBSTEP_COUNT)`:
   - Sample slope at current x.
   - `pid_step(err, slope, dt_sub)` → control u.
   - `cart_pole_step(u, dt_sub)` → updates θ, ω, v, x.
   - Push `(θ, ω)` into phase history.
5. `scene_draw`:
   - `render_terrain` (sky → surface → rock fill below).
   - `render_bot` (clean wheels + tilted body line).
   - One of: `render_panel_telemetry / equations / phase`.
   - `render_hud` (yellow status row 0, cyan hint bottom row).
6. Frame cap.

## Non-Obvious Decisions

**Why TWO pure functions `pid_step` and `cart_pole_step`?**
They're independent layers. PID is a controller — pure math, no physics. Cart-pole is physics — pure math, no controller. Splitting them lets a learner mentally swap controllers (replace `pid_step` with bang-bang, LQR, etc.) without touching the physics, and vice versa. The orchestrator `bot_substep` is the only place they meet.

**Why sub-stepping inside one frame?**
The cart-pole's angular dynamics include a `1/length` factor and a `cos(θ)` factor that, at large `dt`, can produce per-step `Δθ` larger than π. Once that happens, the linearisation `sin(θ) ≈ θ` no longer holds and the integrator diverges in one step. Splitting the frame into `SUBSTEP_COUNT = 4` smaller dt's keeps the per-step `Δθ` small. This is **not** a clarity-vs-speed tradeoff — it's correctness; without sub-stepping the simulation literally explodes.

**Why anti-windup on the integral?**
Long sustained errors (e.g. a long slope) make `∫err` accumulate without bound. When the error finally clears, the controller keeps applying that giant integral correction for seconds afterwards, severely overshooting. Clamping `|I| ≤ I_LIMIT` keeps the integral memory finite. This is the difference between a textbook PID that destabilises in real systems and an industrial PID that doesn't.

**Why slope feed-forward?**
PID is *reactive* — it can only correct an error that has already developed. On a 30° slope, gravity tries to tip the bot constantly, so the integral term has to climb from zero every time the slope changes. Adding `-K_SLOPE_FF · slope` directly to the controller output ANTICIPATES the disturbance — the bot is already braced before tilt error even develops. This is the standard "feed-forward + feedback" cascade pattern from control theory.

**Why three view modes?**
Each is a different *language* for the same simulation:
- Telemetry — engineering language ("ω = 0.42 rad/s, control = -2.1 m/s²").
- Equations — physicist language (the math, with values substituted live).
- Phase portrait — applied-math language (a curve in `(θ, ω)` space tells you stability at a glance).
Switching views is the same simulation seen through three lenses; learners build intuition by watching the curves bend in phase space while the equations on the other tab tell them why.

**Why six gain presets?**
The `g` key cycles through preset PID gains chosen to demonstrate distinct failure modes:
- All zero — bot tips over immediately.
- High kp only — oscillates wildly.
- Adding kd — damps the oscillation.
- Adding ki — eliminates steady-state error but introduces overshoot.
- Tuned — well-behaved.
- Aggressive — fast but rings.
Watching the phase portrait reshape across these presets *teaches PID* faster than reading any chapter.

**Why is the bot rendered as a tilted body line + two wheels (not a sprite)?**
Visual clarity. The body angle θ is THE state variable being controlled — it must be visually obvious. A sprite would hide θ inside its silhouette; a single tilted line shows θ directly to the eye. Wheels are dots so they don't compete with the body for attention.

## Key Constants

| Constant | Default | Effect |
|---|---|---|
| SUBSTEP_COUNT | 4 | Physics steps per render frame. ↑ for higher stability. |
| GRAVITY | 9.81 m/s² | SI; converted to cells/s² at use. |
| POLE_LENGTH | 1.5 m | Body length in cart-pole model. |
| KP_DEFAULT | 38 | Proportional gain. |
| KD_DEFAULT | 8 | Derivative gain. |
| KI_DEFAULT | 4 | Integral gain. |
| I_LIMIT | 6 | Anti-windup clamp. |
| K_SLOPE_FF | 12 | Slope feed-forward strength. |
| K_SLOPE_TARGET | 0.55 | Target tilt = `-slope · this`; "lean into the hill." |
| PHASE_HIST_CAP | 256 | (θ,ω) ring buffer size for phase plot. |

## Open Questions

- A continuous slider for kp/kd/ki (instead of discrete presets) would let the learner sweep gains live.
- Modelling a non-zero wheel mass (currently the wheel is massless) adds inertia to the v dynamics — closer to a real Segway.
- Adding wind / impulse disturbances on a key press would test the controller more aggressively than just the slope.

---

# Pass 2 — Pseudocode

## Module Map

| § | Purpose |
|---|---|
| §1 config | frame rate, sub-step target, cell sizes, cart-pole consts (SI), PID defaults + anti-windup, slope FF, robot geometry, drive speed cycle, terrain, phase history, view modes, ncurses pairs, HUD |
| §2 clock | monotonic timer + sleep |
| §3 color | terrain bands, body, gauge tints, HUD pairs |
| §4 coords | pixel ↔ cell |
| §5 perlin | 1D Perlin + fBm for terrain |
| §6 bot | (6.1) GainPreset (6.2) Bot type (6.3) pid_step (6.4) cart_pole_step (6.5) phase_history_push (6.6) bot_substep (6.7) init/reset/preset |
| §7 render | (7.1) helpers (7.2) terrain (7.3) bot (7.4) telemetry panel (7.5) equations panel (7.6) phase panel (7.7) HUD (7.8) scene_draw |
| §8 screen | ncurses init / present |
| §9 app | signals, resize, variable-dt main loop |

## Data Flow

```
keys → app_handle_key → bot.{view, preset_idx, drive_speed, paused}
                            │
clock_ns → dt → for n in 0..SUBSTEP_COUNT-1:
                  bot_substep(dt / SUBSTEP_COUNT):
                    ├── slope = sample_terrain_slope(bot.x)
                    ├── pid_step(err, slope, dt_sub)        → control u
                    │      P = kp·err
                    │      I = clamp(I + ki·err·dt, ±I_LIMIT)
                    │      D = kd·(err - prev_err)/dt
                    │      u = P + I + D - K_SLOPE_FF·slope
                    ├── cart_pole_step(u, dt_sub)
                    │      α = (g·sin θ - u·cos θ) / L
                    │      ω += α·dt; θ += ω·dt
                    │      v += u·dt;  x += v·dt
                    └── phase_history_push(θ, ω)
                            │
                            ▼
                       scene_draw
                          terrain → bot → panel(view) → HUD
```

## Pseudocode

```
setup:
  install signals
  screen_init, color_init
  perlin_init (seed)
  bot_init(cols, rows)

while running:
  if need_resize: re-init geometry preserving preset/view
  dt = clamp(now - last, 0, DT_CAP_SEC)
  drain input → app_handle_key
  if not paused or single_step:
    for n in 0..SUBSTEP_COUNT-1:
      bot_substep(dt / SUBSTEP_COUNT)
  fps update
  erase
  scene_draw
  wnoutrefresh + doupdate
  sleep to TARGET_FPS
```

## Key Patterns to Internalize

**PID + cart-pole as two pure functions.** Each is a contract: `pid_step` consumes error, returns control output; `cart_pole_step` consumes control, returns new state. The orchestrator wires them. This is the canonical "controller + plant" decomposition from control theory.

**Sub-stepping for stiff systems.** When the dynamics include `1/L` or `cos(θ)` factors, large dt can blow up the integrator. Sub-step inside the frame to keep per-step delta small.

**Anti-windup is not optional.** A textbook PID without it works on an exam and fails in production. Always clamp the integral.

**Feed-forward + feedback cascade.** PID corrects errors that have already happened; feed-forward eliminates errors before they happen. Both layers are needed for disturbance-rich environments (slopes, wind).

**Phase portrait as diagnostic.** Plot `(θ, ω)` and read stability off the curve shape. Spirals → damped; ellipses → undamped; outward spirals → unstable. Worth more than ten paragraphs of text.

**Multiple views, one simulation.** Different audiences need different abstractions. Telemetry, equations, phase: three lenses, same physics underneath. Cycle them with one key.
