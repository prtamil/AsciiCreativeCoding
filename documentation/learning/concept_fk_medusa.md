# Pass 1 — fk_medusa: Bell oscillation + velocity-based tentacle trailing — stateless FK with analytic derivatives for physically plausible drag

## From the Source

**Algorithm:** Stateless FK with velocity-based trailing. The bell center is driven by a sine oscillator. Its instantaneous velocity (`bell_vy`, the analytic derivative of the sine) tilts the FK base heading for all tentacles, producing the trailing-drag illusion without any stored history. Each tentacle chain is derived algebraically each tick from `wave_time`; `prev_joint` is saved only for alpha lerp. Contrast with snake_forward_kinematics.c which uses a circular trail buffer (path-following FK, stateful).

**Math:** Bell position: `bell_cy = bell_base_cy + BELL_AMP_PX × sin(bell_time × ω)` where `ω = 2π / BELL_PERIOD`. Bell velocity (analytic derivative): `bell_vy = BELL_AMP_PX × ω × cos(bell_time × ω)`. Trailing tilt: `trailing_angle = clamp(bell_vy × TRAIL_FACTOR / 100, −MAX_TRAIL, MAX_TRAIL)`. FK base heading: `π/2 + trailing_angle`. FK cumulation: `cumulative_angle += amplitude × sin(frequency × wave_time + root_phase[k] + i × PHASE_PER_SEG)`. Bell pulse: `pulse = 0.85 + 0.15 × sin(pulse_phase × 3)`.

**Performance:** O(N_TENTACLES × N_SEGS) per tick for FK. O(rows) per frame for bell rendering. No trail buffer. At 60 Hz: 8 tentacles × 18 segments = 144 FK evaluations per tick. ncurses doupdate sends only changed cells.

**Data-structure:** Scene holds bell kinematics (`bell_cy`, `bell_vy`, `bell_time`, `pulse_phase`, `pulse`) and per-tentacle arrays `joint[N_TENTACLES][N_SEGS+1]` and `prev_joint[N_TENTACLES][N_SEGS+1]`, plus `root_phase[]` and `root_angle[]` pre-computed in `scene_init`. No trail buffer; the formula is the entire state.

## Core Idea

A jellyfish (medusa) pulses its bell and swims vertically through the terminal. The bell oscillates sinusoidally up and down. Eight tentacles hang from the bottom edge of the bell. When the bell moves upward, the tentacles stream backward behind it — when the bell moves downward they tilt forward, pushed by the water pressure. This trailing behaviour is stateless: the tilt angle is proportional to the bell's current instantaneous velocity, computed analytically from the same sine formula that drives the bell's position. No memory of previous positions is needed.

The bell itself is rendered as a dynamic half-ellipse: at each row within the dome, the ellipse equation gives the half-width, and the interior is filled with `-` / `(` / `)`. A `pulse` factor scales both axes uniformly, making the bell contract and expand rhythmically — the medusa's propulsive jet.

## The Mental Model

Think of a jellyfish at aquarium glass. When it thrusts upward, the tentacles stream downward behind it. When it drifts downward, they float forward. This is hydrodynamic drag: the faster it moves, the more the tentacles trail. In code, the instantaneous vertical velocity `bell_vy` is multiplied by `TRAIL_FACTOR/100` to get an angular tilt in radians. The base heading of every tentacle starts at π/2 (straight down) and is tilted by this angle. Moving up → negative `bell_vy` → negative tilt → tentacles lean backward (upward).

The key insight: real drag depends on velocity. The analytic sine derivative `BELL_AMP_PX × ω × cos(ω × t)` is the exact velocity at every tick — no subtraction of previous positions required. This is more accurate than finite-differencing (`cy[t] − cy[t−1]`) because it does not accumulate rounding errors and does not require storing past positions.

The bell's pulsation is a separate oscillator (`pulse_phase`) running at 2× the bell's vertical speed. The `× 3` in `sin(pulse_phase × 3)` gives approximately 0.95 pulses per vertical cycle — slightly less than one, creating a beat frequency that makes the contraction feel organic rather than perfectly mechanical.

## Data Structures

### Vec2
```
x, y   — pixel space position (float); x eastward, y downward
```

### Scene (complete simulation state)
```
BELL KINEMATICS:
  bell_cx        bell center x, pixel space (constant = screen_width/2)
  bell_cy        bell center y, pixel space; oscillates = bell_base_cy + BELL_AMP_PX × sin(ωt)
  bell_base_cy   oscillation equilibrium (vertical screen midpoint)
  bell_vy        instantaneous vertical velocity px/s = BELL_AMP_PX × ω × cos(ωt)
  bell_time      accumulator for bell sine (seconds)
  pulse_phase    accumulator for bell size pulse (advances at 2× real time)
  pulse          current size scale factor ∈ [0.70, 1.00]
                 = 0.85 + 0.15 × sin(pulse_phase × 3)

TENTACLE WAVE:
  wave_time      accumulator for FK sine (seconds); separate from bell_time
  amplitude      peak FK bend per segment (rad); keyboard-adjustable
  frequency      FK wave oscillation rate (rad/s); keyboard-adjustable
  seg_len_px     segment length (px); dynamic: rows × CELL_H × 0.55 / N_SEGS
  paused         physics frozen when true

THEME:
  theme_idx      index into THEMES[10]; 't' advances it

TENTACLE STATE:
  joint[8][19]      current pixel positions; [k][0]=root on bell edge, [k][18]=tip
  prev_joint[8][19] start-of-tick snapshot for alpha lerp
  root_phase[8]     per-tentacle constant phase offset; pre-computed in scene_init
  root_angle[8]     attachment angle around the bell ellipse; in [π, 2π] (bottom semicircle)
```

### Screen
```
cols, rows   terminal dimensions
```

### App
```
scene         simulation state
screen        terminal dimensions
sim_fps       physics tick rate Hz; keyboard-adjustable
running       volatile sig_atomic_t
need_resize   volatile sig_atomic_t
```

## The Main Loop

Each iteration:

1. **Resize check.** If `need_resize` is set, save `wave_time`, `bell_time`, `pulse_phase`, `amplitude`, `frequency`, `theme_idx`. Call `scene_init()` (re-centres bell for new dimensions, recomputes `seg_len_px`). Restore all saved values. Reset `frame_time` and `sim_accum`.

2. **Measure dt.** Read monotonic clock; subtract `frame_time`. Cap at 100 ms.

3. **Physics accumulator.** While `sim_accum >= tick_ns`: call `scene_tick(dt_sec, cols, rows)`, subtract `tick_ns`.

4. **Compute alpha.** `alpha = sim_accum / tick_ns ∈ [0, 1)`.

5. **FPS counter.** Sliding 500 ms window.

6. **Frame cap.** Sleep before render to target 60 fps.

7. **Draw and present.** `erase()` → `scene_draw()` (tentacles, then bell on top) → HUD bars → `wnoutrefresh()` + `doupdate()`.

8. **Drain input.** Loop `getch()` until `ERR`.

## Non-Obvious Decisions

**Why use the analytic derivative for bell velocity rather than finite differences?**
The finite-difference estimate `(bell_cy[t] − bell_cy[t-1]) / dt` accumulates floating-point errors, varies with `dt` (and therefore with `sim_fps`), and requires storing the previous position. The analytic derivative `BELL_AMP_PX × ω × cos(ω × bell_time)` is exact at every tick, requires no storage, and is independent of `sim_fps`. The trailing tilt angle is consistent regardless of whether the physics runs at 10 Hz or 120 Hz.

**Why divide by 100 in the trailing angle formula (`bell_vy × TRAIL_FACTOR / 100`)?**
`bell_vy` is in pixels per second. At maximum amplitude and frequency, `|bell_vy_max| = BELL_AMP_PX × ω ≈ 40 × (2π/4) ≈ 62.8 px/s`. Without the `/100`, `TRAIL_FACTOR × 62.8 = 0.6 × 62.8 ≈ 37.7 rad` — far beyond `MAX_TRAIL = 0.8 rad`. Dividing by 100 scales the raw velocity into an angular range where `62.8 × 0.6 / 100 ≈ 0.38 rad ≈ 22°`. This is noticeable but keeps the tentacles clearly pointing downward rather than sideways.

**Why base_heading = π/2 + trailing_angle (not −π/2)?**
In terminal pixel space, y increases downward. π/2 corresponds to `cos(π/2) = 0, sin(π/2) = 1` — a vector pointing in the +y direction, which is downward on screen. So π/2 is "straight down", the correct default heading for tentacles hanging below a jellyfish bell. (Compare fk_tentacle_forest which uses −π/2 = straight up because those tentacles grow upward from the sea floor.)

**Why place tentacle roots in [π, 2π] around the bell?**
The ellipse parameterization is `(bell_cx + BELL_RADIUS_X × cos(θ), bell_cy + BELL_RADIUS_Y × sin(θ))`. For θ ∈ [π, 2π], `sin(θ) ∈ [0, −1, 0]` — the y-component is non-negative in math convention, or non-negative in pixel convention. Wait: `sin(π) = 0`, `sin(3π/2) = −1`, `sin(2π) = 0`. In pixel space y-down, the bell center is at `bell_cy`. Adding `BELL_RADIUS_Y × sin(root_angle)` where `root_angle ∈ [π, 2π]` gives `sin ∈ [−1, 0]` — this adds a non-positive value, moving the root upward (toward the top of the bell), which is wrong. Actually the code uses `root.y = bell_cy + BELL_RADIUS_Y × sin(root_angle[i]) × pulse`. For θ ∈ [π, 2π]: `sin(π)=0`, `sin(3π/2) = −1`, `sin(2π)=0`. So `root.y = bell_cy + BELL_RADIUS_Y × sin × pulse`. Since `sin(θ)` for θ ∈ [π, 2π] spans [0, −1, 0] → `root.y` ranges from `bell_cy` down to `bell_cy − BELL_RADIUS_Y`. This places roots on or above the bell center, not below. But the tentacles point *downward* from these roots (base_heading ≈ π/2), so they trail below the bell correctly. The roots are on the equator and lower-hemisphere rim; they are not placed below the bell, but the tentacles extend downward from there.

**Why two separate time accumulators (`bell_time` and `wave_time`)?**
They drive different oscillators at potentially different scales. `bell_time` drives a 4-second period vertical oscillation. `wave_time` drives tentacle FK at a frequency set by the user (`FREQ_DEFAULT = 1.2 rad/s`). If they shared a single clock, changing the tentacle wave frequency (user keypress) would not affect the bell's motion — which is correct, the bell has a fixed BELL_PERIOD. Keeping them separate means the bell's oscillation speed is independent of the tentacle undulation frequency. They start together at 0 but diverge if the FK frequency is modified.

**Why `pulse_phase` advances at `dt × 2.0` rather than `dt`?**
The bell oscillates at `ω_bell = 2π / BELL_PERIOD = 2π/4 ≈ 1.57 rad/s`. The pulse uses `sin(pulse_phase × 3)` where `pulse_phase` advances at `2.0` rad/s of real time. So the pulse oscillates at `2.0 × 3 = 6.0 rad/s ≈ 0.95 cycles/second`. The bell oscillates at `1.57 / (2π) ≈ 0.25 cycles/second`. The pulse fires approximately `0.95 / 0.25 ≈ 3.8` times per bell oscillation cycle — nearly 4 contractions per up-down-up cycle. The ×2.0 and ×3 combination was tuned to give this near-integer ratio, creating an organic beat frequency rather than perfect synchrony.

**Why render tentacles before the bell (painter's algorithm)?**
The bell dome should appear in front of the tentacle roots — just as a real medusa's bell obscures the base of its tentacles. Drawing tentacles first, then the bell on top, implements this painter's-algorithm depth order. Without this, the bell's `(`, `-`, `)` characters would be overwritten by tentacle root glyphs.

**Why does `draw_bell` use `floor(hw + 0.5f)` for edge columns instead of `roundf`?**
`roundf` uses "round half to even" (banker's rounding), which can produce a different integer for identical `hw` values on different compiler implementations. `floorf(hw + 0.5f)` is "round half up" — deterministic. A one-cell variation in the left/right edge column would cause the bell outline to flicker between two widths on every frame when `hw` is exactly at a half-integer boundary.

**Why does the bell use `COLOR_PAIR(5) | A_BOLD` hardcoded rather than the theme gradient?**
The bell is a single structural object, not a gradient chain. Using a fixed pair (mid-gradient, typically cyan in Medusa theme) gives it a consistent appearance as a "solid shell" object. Applying a root-to-tip gradient across the bell rows would create a banded look that reads as depth (top of dome farther from the viewer), which may not match the desired aesthetic. The hardcoded pair 5 also means the bell color automatically changes when themes are switched, since `theme_apply()` redefines pair 5.

**Why does `scene_draw` use `sc->bell_cy` directly rather than lerping it?**
The bell position changes continuously every tick via the sine formula. `bell_cy` at tick t is `bell_base_cy + BELL_AMP_PX × sin(ω × bell_time_t)`. The alpha lerp for tentacle joints interpolates between the previous tick's FK positions and the current tick's FK positions — this is necessary because FK is only recomputed at tick boundaries, so between ticks the joints hold stale positions. The bell_cy, however, could in principle also be lerped between `prev_bell_cy` and `bell_cy`. But since the bell's motion is smooth (sinusoidal with low frequency), the visual difference between lerping and not lerping the bell center is imperceptible at 60 fps. The code takes the simpler path.

## State Machines

### App-level state
```
        ┌───────────────────────────────────────┐
        │              RUNNING                  │
        │                                       │
        │  [physics ticks] ←── accumulator     │
        │  [render + present] each frame        │
        │  [input dispatch]                     │
        │                                       │
        │  SIGWINCH ──────────────→ need_resize │
        │  need_resize ───────────→ app_do_resize│
        │  SIGINT/SIGTERM ────────→ running = 0 │
        └───────────────────────────────────────┘
                          │
               'q'/ESC/signal
                          │
                          v
                        EXIT
                      (endwin)
```

### Bell oscillation cycle (one complete period = BELL_PERIOD = 4.0 s)
```
bell_time=0:    bell_cy = bell_base_cy (equil.)  bell_vy = +max (about to move down)
                trailing_angle ≈ +0.38 rad → tentacles tilt slightly forward (down)

bell_time=1s:   bell_cy = bell_base_cy + BELL_AMP_PX (lowest pt)  bell_vy = 0
                trailing_angle = 0 → tentacles point straight down

bell_time=2s:   bell_cy = bell_base_cy (equil.)  bell_vy = -max (moving up fastest)
                trailing_angle ≈ -0.38 rad → tentacles stream backward (upward)

bell_time=3s:   bell_cy = bell_base_cy - BELL_AMP_PX (highest pt)  bell_vy = 0
                trailing_angle = 0 → tentacles point straight down

bell_time=4s:   back to bell_time=0

(BELL_AMP_PX = 40 px = 2.5 terminal rows of vertical travel)
```

### Bell pulse cycle
```
pulse_phase advances at 2.0 × dt per tick.
pulse = 0.85 + 0.15 × sin(pulse_phase × 3):
  min(pulse) = 0.85 − 0.15 = 0.70  (bell contracted, at 70% size)
  max(pulse) = 0.85 + 0.15 = 1.00  (bell fully expanded)
  pulse frequency = 2.0 × 3 / (2π) ≈ 0.95 cycles/second
  ≈ 3.8 contractions per bell up-down-up cycle
```

### Input actions
| Key | Effect |
|-----|--------|
| q / Q / ESC | running = 0 → exit |
| space | toggle scene.paused |
| w / ↑ | frequency + 0.15 rad/s (tentacle undulation, clamped to [0.1, 6.0]) |
| s / ↓ | frequency − 0.15 rad/s |
| d / → | amplitude + 0.10 rad (tentacle bend, clamped to [0.0, 1.2]) |
| a / ← | amplitude − 0.10 rad |
| t / T | cycle theme_idx forward (mod 10); call theme_apply() immediately |
| ] | sim_fps + 10 (clamped to [10, 120]) |
| [ | sim_fps − 10 |

Note: w/s control frequency and a/d control amplitude — the reverse of fk_tentacle_forest (where w/s is frequency and ←/→ is amplitude, but the letter assignments differ). The hint bar in the HUD always shows the correct mapping for this file.

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|----------|---------|-------------------|
| BELL_PERIOD | 4.0 s | Duration of one up-down-up cycle. Shorter → rapid swimming like a jellyfish in distress. Longer → meditative drift. Affects ω and therefore bell_vy magnitude. |
| BELL_AMP_PX | 40 px | Vertical travel per half-cycle (= 2.5 terminal rows). Larger → more vertical movement, higher max velocity, stronger trailing effect. |
| BELL_RADIUS_X | 18 px | Bell horizontal semi-axis (≈ 2.25 columns). Larger → wider bell; more tentacles spread apart at equator. |
| BELL_RADIUS_Y | 12 px | Bell vertical semi-axis (dome height ≈ 0.75 rows). Larger → taller dome, rounder bell; smaller → flat disc. |
| TRAIL_FACTOR | 0.6 | Velocity-to-tilt coefficient. At max bell_vy ≈ 62.8 px/s: tilt = 62.8 × 0.6 / 100 ≈ 0.38 rad ≈ 22°. Higher → more dramatic trailing at the cost of naturalness. |
| MAX_TRAIL | 0.8 rad | Maximum tentacle tilt from vertical (≈ 46°). Safety clamp; beyond this tentacles point sideways. |
| N_TENTACLES | 8 | Number of tentacles distributed over [π, 2π] around bell. More → denser, comb-like fringe. |
| N_SEGS | 18 | Segments per tentacle chain. More → smoother curvature, more FK evaluations per tick. |
| PHASE_PER_SEG | 0.3 rad | Wave propagation speed up tentacle. Total phase span = 17 × 0.3 = 5.1 rad ≈ 0.81 full cycles from root to tip. |
| AMP_DEFAULT | 0.2 rad | Default peak bend per segment. Lower than fk_tentacle_forest (0.28) — medusa tentacles are more trailing than swaying. |
| FREQ_DEFAULT | 1.2 rad/s | Default FK oscillation rate. Faster than fk_tentacle_forest (0.8) — medusa moves with more urgency. |
| seg_len_px | dynamic | rows × CELL_H × 0.55 / N_SEGS. Recalculated on resize. Tentacle tips reach approximately screen mid-height below the bell's travel range. |
| pulse range | [0.70, 1.00] | 0.85 ± 0.15. Narrowing the range (e.g. 0.95 ± 0.05) gives more subtle pulsation. Widening to ± 0.30 would make the bell shrink to 55% size and look dramatic. |
| N_THEMES | 10 | Selectable color palettes. Default 0 = "Medusa" (deep purple → bright cyan bioluminescent look). |

## Open Questions for Pass 3

- The bell velocity formula `bell_vy = BELL_AMP_PX × ω × cos(bell_time × ω)` gives the velocity at the START of each tick. At `bell_time` after advancing by `dt`, should the trailing angle use the velocity at the start or the end of the tick? Is there a visible difference at low sim Hz (e.g. 10 Hz)?
- `scene_draw` uses `sc->bell_cy` directly (not alpha-lerped). At `SIM_FPS = 10` and render rate 60 fps, alpha varies from 0.0 to ~0.9 between ticks. The bell moves `BELL_AMP_PX × ω × dt = 40 × 1.57 × 0.1 ≈ 6.3 px` per tick at 10 Hz. Without lerping the bell, there would be a ~6 px jump every 6 rendered frames. Is this stutter noticeable? Should bell_cy be lerped?
- `root_angle[i] = π + i × π / (N_TENTACLES − 1)`. For `i = N_TENTACLES − 1 = 7`: `root_angle = π + 7π/7 = 2π`. `cos(2π) = 1`, same as `cos(0)`. The rightmost tentacle root is at `(bell_cx + BELL_RADIUS_X × pulse, bell_cy + 0)`. Is this intentional — placing one tentacle directly to the right of centre rather than at the bottom? Does it look odd when the bell tilts?
- With `pulse_phase` advancing at `dt × 2.0`, after `wave_time ≈ 1e7` seconds of uninterrupted running, `pulse_phase ≈ 2 × 10⁷`. `sinf(2e7 × 3)` in single precision: `2e7 × 3 = 6e7`, which is far above the range where `sinf` is accurate (sinf accuracy degrades above ~10⁶). Would the pulse start flickering? How long would the demo need to run before this is observable?
- `draw_bell` loops `for dy = −ceil(bell_h_cells)` to `0`. `bell_h_cells = (BELL_RADIUS_Y / CELL_H) × pulse = (12/16) × pulse = 0.75 × pulse`. At `pulse = 0.70`: `bell_h_cells = 0.525`, `−ceil(0.525) = −1`. At `pulse = 1.00`: `bell_h_cells = 0.75`, `−ceil(0.75) = −1`. So `dy_min` is always −1, meaning the dome is always exactly 1 row tall regardless of pulse. Is the bell visibly changing height between rows, or does the change only show as width variation?
- The seabed '~' row from fk_tentacle_forest is absent from fk_medusa. The background is a plain black terminal. Could a dim water-column background (perhaps alternating dark blue columns) improve the atmosphere without cluttering the jellyfish? What would the rendering cost be?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app` | `App` | ~25 KB | top-level container |
| `g_app.scene.joint[8][19]` | `Vec2[152]` | 1.2 KB | current tentacle joint positions |
| `g_app.scene.prev_joint[8][19]` | `Vec2[152]` | 1.2 KB | previous tick snapshot for alpha lerp |
| `g_app.scene.root_phase[8]` | `float[8]` | 32 B | per-tentacle phase offsets |
| `g_app.scene.root_angle[8]` | `float[8]` | 32 B | attachment angles around bell ellipse |
| `THEMES[10]` | `Theme[10]` | ~800 B | named palettes; read-only |

# Pass 2 — fk_medusa: Pseudocode

## Module Map

| Section | Purpose |
|---------|---------|
| §1 config | All tunables: N_TENTACLES, N_SEGS, PHASE_PER_SEG, AMP/FREQ defaults, BELL_PERIOD, BELL_AMP_PX, BELL_RADIUS_X/Y, TRAIL_FACTOR, MAX_TRAIL, N_THEMES, timing, CELL_W/H |
| §2 clock | `clock_ns()` — monotonic timestamp; `clock_sleep_ns()` — sleep |
| §3 color | `Theme` struct; `THEMES[10]`; `theme_apply(idx)` — live pair re-init; `color_init(initial)` — 8-color fallback |
| §4 coords | `px_to_cell_x/y` — aspect-ratio conversion; `clampf()` — general clamp helper |
| §5a render helpers | `seg_pair(i)` — root-to-tip color; `seg_attr(i)` — brightness zones; `draw_segment_beads()` — 'o' bead fill |
| §5b FK chain | `compute_tentacle_fk()` — stateless FK with velocity-based trailing tilt |
| §5c bell rendering | `draw_bell()` — dynamic half-ellipse per row with pulse scaling |
| §6 scene | `Scene` struct; `scene_init()` — bell centring, root distribution; `scene_tick()` — bell + pulse + FK; `scene_draw()` — tentacles then bell, painter's order |
| §7 screen | ncurses setup, HUD composition, double-buffer flush |
| §8 app | Signal handlers, resize with multi-clock preservation, main loop |

---

## Data Flow Diagram

```
CLOCK_MONOTONIC
    │
    ▼
clock_ns() → dt (nanoseconds)
    │
    ▼
sim_accum += dt
    │
    while sim_accum >= tick_ns:
    │
    ▼
scene_tick(dt_sec, cols, rows)
    │
    ├── Step 1: save prev_joint for all N_TENTACLES   ← BEFORE any mutation
    │
    ├── Step 2: if paused: return (clean freeze)
    │
    ├── Step 3: bell_time += dt
    │           omega = 2π / BELL_PERIOD
    │           bell_cy = bell_base_cy + BELL_AMP_PX × sin(bell_time × omega)
    │           bell_vy = BELL_AMP_PX × omega × cos(bell_time × omega)
    │                                              ↑ analytic derivative, exact at any Hz
    │
    ├── Step 4: pulse_phase += dt × 2.0
    │           pulse = 0.85 + 0.15 × sin(pulse_phase × 3)
    │           range: [0.70, 1.00] — uniform bell scaling
    │
    ├── Step 5: wave_time += dt
    │
    └── Step 6: for each tentacle k:
                  root.x = bell_cx + BELL_RADIUS_X × cos(root_angle[k]) × pulse
                  root.y = bell_cy + BELL_RADIUS_Y × sin(root_angle[k]) × pulse
                  compute_tentacle_fk(joint[k], root, root_phase[k], ...)
                        │
                        ├── trailing_angle = clamp(bell_vy × 0.6/100, -0.8, 0.8)
                        ├── base_heading = π/2 + trailing_angle   (down + tilt)
                        ├── joint[0] = root
                        ├── cumulative_angle = base_heading
                        └── for i = 0..N_SEGS-1:
                              δθ = amplitude × sin(freq × wave_time
                                                   + root_phase + i × PHASE_PER_SEG)
                              cumulative_angle += δθ
                              joint[i+1] = joint[i] + seg_len × (cos/sin cumul_angle)

sim_accum -= tick_ns
    │
    ▼
alpha = sim_accum / tick_ns   [0.0 .. 1.0)
    │
    ▼
scene_draw(sc, w, cols, rows, alpha)
    │
    ├── Step 1 (all tentacles, painter's back-layer):
    │     for each tentacle k:
    │       rj[j] = lerp(prev_joint[k][j], joint[k][j], alpha)  for j=0..N_SEGS
    │       Pass 1: draw_segment_beads(rj[j] → rj[j+1]) root→tip
    │               each segment stamped with 'o' at DRAW_STEP_PX intervals
    │       Pass 2: joint markers at rj[j] positions:
    │               j==0 or j<=N_SEGS/3 → '0', j<=2N_SEGS/3 → 'o', else → '.'
    │
    └── Step 2 (bell, painter's front-layer):
          draw_bell(w, bell_cx, bell_cy, pulse, cols, rows)
                │
                for dy = -ceil(bell_h_cells) .. 0:
                  t = -dy / bell_h_cells; hw = bell_rx × sqrt(1 − t²)
                  left = bcx − round(hw); right = bcx + round(hw)
                  '(' at left, '-'/'=' interior, ')' at right
                  '=' used at dy=0 (equator, tentacle attachment row)

    ▼
screen_present() → doupdate()   [ONE atomic write to terminal]
```

---

## Function Breakdown

### compute_tentacle_fk(joint, root, root_phase, wave_time, amplitude, frequency, bell_vy, seg_len_px)
Purpose: stateless FK for one tentacle with velocity-based trailing
Steps:
1. `trailing_angle = clampf(bell_vy × TRAIL_FACTOR / 100.0f, −MAX_TRAIL, MAX_TRAIL)`
2. `base_heading = π/2 + trailing_angle`  (π/2 = straight down in pixel-y-down convention)
3. `joint[0] = root`
4. `cumulative_angle = base_heading`
5. For i = 0 .. N_SEGS - 1:
   - `delta = amplitude × sinf(frequency × wave_time + root_phase + i × PHASE_PER_SEG)`
   - `cumulative_angle += delta`
   - `joint[i+1].x = joint[i].x + seg_len_px × cosf(cumulative_angle)`
   - `joint[i+1].y = joint[i].y + seg_len_px × sinf(cumulative_angle)`
Note: The trailing angle shifts the entire chain as one rigid unit; the per-segment FK bends are applied on top of the trailing tilt.

---

### draw_bell(w, bell_cx, bell_cy, pulse, cols, rows)
Purpose: render dynamic half-ellipse for the bell dome
Steps:
1. `bcx = px_to_cell_x(bell_cx)`;  `bcy = px_to_cell_y(bell_cy)`
2. `bell_h_cells = (BELL_RADIUS_Y / CELL_H) × pulse`  (vertical semi-axis in cell units)
3. `bell_rx_cells = (BELL_RADIUS_X / CELL_W) × pulse`  (horizontal semi-axis in cell units)
4. `dy_min = −ceil(bell_h_cells)`
5. For dy = dy_min .. 0:
   - `t = (bell_h_cells > 0) ? float(−dy) / bell_h_cells : 0`  (clamp t to 1.0)
   - `hw = bell_rx_cells × sqrt(1 − t²)`  (ellipse half-width at this row)
   - `left = bcx − floor(hw + 0.5)`;  `right = bcx + floor(hw + 0.5)`
   - `row = bcy + dy`
   - Skip if `row < 1 || row >= rows − 1`
   - `fill_ch = (dy == 0) ? '=' : '-'`  (equator row uses '=')
   - For c = left .. right:
     - If c == left: ch = '('; elif c == right: ch = ')'; else ch = fill_ch
     - mvwaddch(row, c, ch) with COLOR_PAIR(5) | A_BOLD
Why pair 5 hardcoded: bell is a solid object; mid-gradient color gives consistent "glowing shell" look; it auto-changes with theme since pair 5 is re-bound by theme_apply().

---

### seg_pair(i) → int
Formula: `1 + (i × (N_PAIRS − 1)) / (N_SEGS − 1)`
For N_SEGS=18, N_PAIRS=7:
- i=0 → pair 1 (root, dark purple in Medusa theme)
- i=8 → pair 3 (mid-tentacle)
- i=17 → pair 7 (tip, bright cyan — bioluminescent)

### seg_attr(i) → attr_t
- i < N_SEGS/4 (root quarter) → A_DIM (thick, heavy root)
- i > 3×N_SEGS/4 (tip quarter) → A_BOLD (bioluminescent tip)
- otherwise → A_NORMAL

### draw_segment_beads(w, a, b, pair, attr, cols, rows, *prev_cx, *prev_cy)
Purpose: stamp 'o' beads along segment a→b at DRAW_STEP_PX intervals
Steps:
1. nsteps = ceil(len / DRAW_STEP_PX) + 1
2. For t = 0 .. nsteps: u = t/nsteps; cx,cy from px_to_cell
3. Dedup (shared cursor), clip ([1, rows-2]), stamp 'o'
Note: Tentacle uses 'o' beads uniformly (not direction glyphs); the bead chain is the visual style for a medusa tentacle.

---

### scene_init(sc, cols, rows)
Purpose: initialise medusa at screen centre
Steps:
1. `seg_len_px = rows × CELL_H × 0.55 / N_SEGS`
2. `bell_cx = cols × CELL_W × 0.5`; `bell_base_cy = rows × CELL_H × 0.5`
3. `bell_cy = bell_base_cy`; `bell_vy = 0` (sin(0)=0; cos(0)=1 → bell about to move down)
4. For i = 0 .. N_TENTACLES−1:
   - `root_angle[i] = π + i × π / (N_TENTACLES − 1)`  (π to 2π, bottom semicircle)
   - `root_phase[i] = i × 2π / N_TENTACLES`  (evenly over [0, 2π))
   - Seed all joints to bell centre (overwritten on first tick)
   - memcpy prev_joint = joint (no-op first-frame lerp)

### scene_tick(sc, dt, cols, rows)
Purpose: one physics step
Steps:
1. For all tentacles: memcpy prev_joint[k] = joint[k]   ← BEFORE physics
2. If paused: return
3. bell_time += dt; compute bell_cy (sin) and bell_vy (cos × ω)
4. pulse_phase += dt × 2.0; pulse = 0.85 + 0.15 × sin(pulse_phase × 3)
5. wave_time += dt
6. For each tentacle k: compute root on bell edge (scaled by pulse); compute_tentacle_fk()

### app_do_resize(app)
Purpose: handle SIGWINCH with full phase preservation
Steps:
1. screen_resize()
2. Save: wave_time, bell_time, pulse_phase, amplitude, frequency, theme_idx
3. scene_init(cols, rows)  — re-centres bell, recomputes seg_len_px, reseeds roots
4. Restore all 6 saved values
5. need_resize = 0
After: main() resets frame_time and sim_accum

---

## Pseudocode — Core Loop

```
setup:
  atexit(cleanup)
  signal(SIGINT/SIGTERM) → running = 0
  signal(SIGWINCH) → need_resize = 1
  screen_init()   ← calls color_init(0) → Medusa theme
  scene_init(cols, rows)
  frame_time = clock_ns()
  sim_accum = 0

main loop (while running):

  ① resize:
     if need_resize:
       app_do_resize(app)   ← saves 6 time/param values, re-inits, restores
       frame_time = clock_ns()
       sim_accum = 0

  ② dt:
     now = clock_ns()
     dt = now − frame_time
     frame_time = now
     cap dt at 100ms

  ③ fixed-step accumulator:
     tick_ns = NS_PER_SEC / sim_fps
     dt_sec = tick_ns / NS_PER_SEC (float)
     sim_accum += dt
     while sim_accum >= tick_ns:
       scene_tick(dt_sec, cols, rows):
         save prev_joint[]  ← ALL tentacles before any mutation
         if paused: return
         bell_time += dt; bell_cy = sin formula; bell_vy = cos formula
         pulse_phase += dt×2; pulse = 0.85 + 0.15×sin(pulse_phase×3)
         wave_time += dt
         for each tentacle: compute root on bell edge (×pulse); compute_tentacle_fk()
       sim_accum -= tick_ns

  ④ alpha:
     alpha = sim_accum / tick_ns   [0.0 .. 1.0)

  ⑤ fps counter:
     frame_count++; fps_accum += dt
     if fps_accum >= 500ms: fps_display = count/elapsed; reset

  ⑥ frame cap:
     elapsed = clock_ns() − frame_time + dt
     sleep(NS_PER_SEC/60 − elapsed)

  ⑦ draw + present:
     erase()
     scene_draw(alpha):
       for each tentacle: lerp joints, draw_segment_beads (pass1), node markers (pass2)
       draw_bell (on top of tentacle roots)
     HUD bar (right-aligned) + hint bar (bottom)
     wnoutrefresh() + doupdate()

  ⑧ drain input:
     while (ch = getch()) != ERR: dispatch key

cleanup:
  endwin()
```

---

## Interactions Between Modules

```
App
 ├── owns Screen (ncurses state, cols/rows)
 ├── owns Scene (bell kinematics + 8 tentacle chains)
 ├── reads sim_fps
 └── main loop drives everything

Scene — tick order per physics step:
  1. prev_joint saves (for ALL tentacles) — BEFORE any mutation
  2. bell kinematics: bell_time → bell_cy (sin), bell_vy (cos×ω)
  3. pulse: pulse_phase → pulse (sin)
  4. wave_time advance
  5. for each tentacle: root on bell rim (×pulse) → compute_tentacle_fk()

compute_tentacle_fk():
  reads: bell_vy → trailing_angle → base_heading
  reads: wave_time, root_phase, amplitude, frequency, seg_len_px
  writes: joint[k][0..N_SEGS] — all positions derived from formula, no history

scene_draw() — painter's algorithm back to front:
  reads: prev_joint, joint, alpha, bell_cy, bell_cx, pulse, root_phase, root_angle
  Step 1: tentacles (background)
    - alpha lerp for each tentacle
    - draw_segment_beads (pass 1: 'o' fill)
    - joint node markers (pass 2: '0'/'o'/'.' on top of fill)
  Step 2: bell (foreground)
    - draw_bell: ellipse equation per row, '('/'='/'−'/')' glyphs

§4 coords:
  called ONLY inside draw_segment_beads() and scene_draw() node stamp + draw_bell
  never during physics

§3 color:
  theme_apply(idx): called at startup (idx=0) and on 't' keypress
  re-binds all 8 pairs live; next frame picks up new palette automatically

Signal handlers:
  SIGINT/SIGTERM → running = 0
  SIGWINCH → need_resize = 1
  both volatile sig_atomic_t
```

---

## Key Patterns to Internalize

**Analytic derivatives for velocity — no finite differencing:**
When position is defined analytically as `f(t)`, velocity is `f'(t)`. Compute `f'(t)` symbolically and evaluate it directly. Do not subtract consecutive positions. The analytic formula is exact at any simulation frequency, requires no stored history, and does not amplify floating-point noise.

**Velocity-based trailing tilt — stateless drag illusion:**
Map the current velocity to a tilt angle via a proportional formula. The tilt is proportional to speed, just as hydrodynamic drag would produce. The entire illusion requires only: the instantaneous velocity (one float), a scale factor, and a clamp. No trajectory memory, no particle system, no integration.

**Uniform pulse scaling of an ellipse:**
Multiplying both semi-axes of an ellipse by the same scale factor `p` produces a uniformly scaled ellipse — the shape is preserved and the aspect ratio is unchanged. This makes the bell "breathe" (contract and expand proportionally) rather than stretch in one direction. The formula: `hw = (RADIUS_X × p / CELL_W) × sqrt(1 − t²)`.

**Multi-clock scene state and resize:**
When a scene has multiple independent time accumulators (`bell_time`, `pulse_phase`, `wave_time`), all of them must be saved and restored across any operation that calls `scene_init()`. Missing even one accumulator causes a discontinuity — the jellyfish would suddenly reset to t=0 for that oscillator on every terminal resize.

**Two-pass bead chain rendering:**
Pass 1 stamps 'o' beads along each segment at uniform spacing (fill). Pass 2 stamps distinct node markers ('0', 'o', '.') at joint positions (on top of fill). The varying marker size from root to tip (heavy '0' → delicate '.') reinforces the biological anatomy: a thick muscular base tapering to fine stinging-cell tips. This is the fk_medusa variant of the two-pass pattern used in all three FK demos.
