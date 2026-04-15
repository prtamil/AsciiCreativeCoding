# Pass 1 — jellyfish.c: Physics-based bioluminescent jellyfish with pulse locomotion

## Core Idea

A set of ASCII jellyfish drift upward through the terminal using a four-state physics model that mimics real medusozoan jet propulsion. Each jellyfish passively sinks during its rest phase, then fires a rapid bell contraction to produce an upward thrust burst, coasts on momentum through drag, and finally blooms back open. The bell renders as a scan-line ellipse dome with two independently controlled axes — body width and crown height — so contraction looks genuinely three-dimensional rather than uniform scaling.

## The Mental Model

A real jellyfish swims by squeezing water out of its bell cavity. It doesn't just drift; it sinks slightly between pulses, then snaps shut fast (the power stroke), which ejects a jet of water and accelerates it upward. Then the bell slowly opens back up (the recovery stroke) while the animal coasts on momentum that decays due to fluid drag.

The terminal renders this in pixel space (CELL_W=8, CELL_H=16 sub-pixels per cell). The bell is the upper half of an ellipse. Two fractions are tracked independently: `head_f` (horizontal radius multiplier — body width pulse) and `crown_f` (vertical span of the apex rows — crown height pulse). When the bell contracts, the body squeezes inward from the sides while the dome top flattens down — two separate axes creating an organic shape change.

Tentacles hang below the bell rim from three anchor points. Their lateral sway is driven by a travelling sine wave, but the amplitude is gated by `wave_scale` — a state-driven multiplier that goes from 1.0 (full sway in idle) down to ~0.12 (nearly straight) during the glide phase when the jellyfish is shooting upward. Tips also trail behind the motion via a velocity-proportional lag offset: `py_offset = -vy × TENT_LAG × depth`, so fast upward motion streams the tentacles downward.

## Data Structures

### Jellyfish
```
cx, cy          — bell equator centre in pixel space
drift_cx        — x oscillation centre (random per jellyfish)
drift_phase     — horizontal sinusoidal drift accumulator (rad)
tent_phase[3]   — per-tentacle wave phase accumulator (rad)
cp              — ncurses color pair index
vy              — vertical velocity px/s (negative = upward)
bell_open       — master open fraction [MIN_OPEN..1.0] driven by state machine
idle_dur        — randomised duration for current IDLE phase (seconds)
anim            — current AnimState (IDLE/CONTRACT/GLIDE/EXPAND)
anim_t          — elapsed time in current state (seconds)
crown_f         — crown height fraction (derived from bell_open, height axis)
head_f          — body width fraction  (derived from bell_open, width axis, deeper squeeze)
tent1_f         — near-bell tentacle reveal (always 1.0 in physics model)
tent2_f         — tip tentacle reveal     (always 1.0 in physics model)
wave_scale      — tentacle lateral amplitude scale [0..1], state-driven
```

### Scene
```
jellies[]       — up to N_JELLIES_MAX (8) Jellyfish structs
n_jellies       — how many are active (+ / - to change)
paused          — freezes all ticks
max_px, max_py  — pixel dimensions of terminal (cols×CELL_W, rows×CELL_H)
```

## The Four-State Physics Machine

```
IDLE → CONTRACT → GLIDE → EXPAND → IDLE → ...
```

**IDLE** (0.55–1.1 s, randomised):
- bell_open = 1.0, wave_scale = 1.0
- vy += GRAVITY × dt (passive sink, ~30 px/s²)
- Tentacles sway freely

**CONTRACT** (0.19 s, fast ease-out):
- bell_open: 1.0 → MIN_OPEN using `1 − (1−t)²` (fast at start, slows)
- wave_scale: 1.0 → 0.15 (sway tightens as bell squeezes)
- At completion: vy = −JET_THRUST (190 px/s upward burst)
- Transition → GLIDE

**GLIDE** (0.38 s):
- bell_open = MIN_OPEN (held closed)
- wave_scale = 0.12 (tentacles nearly straight, streaming behind)
- vy decays via `expf(−DRAG_VY × dt)` (exponential drag, DRAG_VY=2.4)

**EXPAND** (0.68 s, slow ease-in):
- bell_open: MIN_OPEN → 1.0 using smoothstep (slow at start, organic bloom)
- wave_scale: 0.12 → 1.0 via smoothstep (sway gradually returns with bell)
- Transition → IDLE with new randomised idle_dur

## Bell Rendering — Asymmetric Two-Axis Contraction

The bell is the upper half-ellipse: ny = (py − cy) / Ry, range [−1, 0].

**Width axis (head_f):**
`Rx = BELL_RX_BASE × head_f`
All scan rows use this Rx. `head_f` is remapped from `bell_open` with a steeper range:
`head_f = HEAD_MIN_OPEN + (1 − HEAD_MIN_OPEN) × t` where `t = (bell_open − MIN_OPEN)/(1 − MIN_OPEN)`
At peak contraction: head_f = 0.42 (squeezes to 42% of full width).

**Height axis (crown_f):**
`crown_ny_limit = 0.88 + 0.12 × crown_f`
Crown rows (abs_ny > 0.88) are only drawn if `abs_ny ≤ crown_ny_limit`. At full open: apex at abs_ny=1.0. At MIN_OPEN=0.85: limit = 0.982 (apex descends slightly).

**Row characters:**
- abs_ny > 0.94: closing roof `_____` (bold)
- abs_ny in (0.88, 0.94]: rounded crown `(___)`
- abs_ny > 0.40: sides `/ \` (bold)
- abs_ny > 0.14: lower body `( )` (bold)
- abs_ny ≤ 0.14: rim `~` with solid `---` fill line
- Interior (shell surface threshold r²>0.60): translucent `.` dots (A_DIM)

## Tentacle Rendering

N_TENTACLES=3 (left, center, right). Each has TENTACLE_SEGS=20 segments.

Attachment width follows body: `Rx = BELL_RX_BASE × head_f`
Base positions: evenly spread across ±Rx from centre.

Per segment:
```
lateral  = TENT_WAVE_AMP × wave_scale × sin(phase[k] − seg×K + k×PHASE_OFF)
lag      = −vy × TENT_LAG × depth           (depth = seg / SEGS−1)
py       = cy + seg × TENT_SEG_PX + lag
```
Characters by depth: `|` bold (0–20%) → `|` normal (20–45%) → `:` dim (45–70%) → `.` dim (70–100%).
Phase advance: `tent_phase[k] += TENT_WAVE_SPD × wave_scale × dt` — frozen during glide.

## Non-Obvious Decisions

### Why two separate min-open constants?
`MIN_OPEN=0.85` for crown height (subtle — crown barely flattens, keeps recognisable shape).
`HEAD_MIN_OPEN=0.42` for body width (dramatic — bell squeezes to 42%, jet looks powerful).
Single constant would mean either imperceptible height change or grotesque width squeeze.

### Why exponential drag instead of linear?
`vy *= expf(−DRAG × dt)` is frame-rate-independent. Linear `vy -= DRAG × vy × dt` diverges at large dt. Exponential decay is the exact solution to the ODE `dv/dt = −k×v`, so physics is stable regardless of whether the frame took 16ms or 50ms.

### Why wave_scale on phase advance too?
If only amplitude drops but phase keeps advancing rapidly, the tentacles at low amplitude would still jitter visibly each frame. Scaling phase advance to `TENT_WAVE_SPD × wave_scale` freezes the wave pattern during glide — tentacles look genuinely rigid and streaming, not just small-amplitude oscillating.

### Why remap bell_open to head_f?
The state machine drives `bell_open` in a single normalised range [MIN_OPEN, 1.0]. If `head_f = bell_open` directly, both axes share the same range and the width squeeze is only 15% (1.0−0.85). The remap stretches the same [0,1] normalised motion to [HEAD_MIN_OPEN, 1.0] = [0.42, 1.0], making the width pulse more than 5× stronger without touching the state machine.

## Open Questions to Explore

1. Can rim undulation (traveling wave on the bell edge) be added with a small per-row `cy_offset = A × sin(angle × 4 + time)`?
2. Would different jellyfish species benefit from different BELL_RY/BELL_RX aspect ratios? Moon jellyfish are flatter; lion's mane are taller.
3. Can multiple jellies interact — gentle repulsion force when two bells overlap in pixel space?
4. What happens if `GRAVITY` and `JET_THRUST` are tuned so the jelly barely makes net progress upward — hovering in place?

---

## From the Source

**Algorithm:** State-machine locomotion — 4 states: IDLE (passive sinking), CONTRACT (jet thrust), GLIDE (coast on momentum), EXPAND (bell re-opens). Bell shape is a parametric ellipse with time-varying radii; tentacles follow a segmented-chain follower model. Contract: 0.19 s (fast ease-out). Glide: 0.38 s. Expand: 0.68 s (slow ease-in). IDLE randomised to 0.55–1.1 s.

**Physics:** Jet propulsion: impulse `JET_THRUST=190 px/s` upward applied at peak CONTRACT. Buoyancy modelled as reduced effective gravity — `GRAVITY=30.0 px/s²` downward during IDLE only. GLIDE decay: `vy *= expf(-DRAG_VY=2.4 × dt)` — exact solution to `dv/dt = -k·v`, frame-rate independent. Bell geometry: BELL_RX_BASE=72 px (horizontal), BELL_RY_BASE=144 px (vertical). Cell-aspect correction: CELL_H/CELL_W=2.0 to produce circular bell on screen.

**Math:** head_f remap stretches body-width range from [MIN_OPEN=0.85, 1.0] to [HEAD_MIN_OPEN=0.85, 1.0] in the source constants, but the concept notes show HEAD_MIN_OPEN effectively mapped to 0.42 in the remap logic. Tentacle velocity lag: `lag = -vy × TENT_LAG=0.09 × depth`. Smoothstep: `t²(3-2t)`. Contract curve: `1-(1-t)²`.

---

# Pass 2 — Pseudocode, Module Map, Data Flow

## Module Map

```
§1 config     — BELL_RX/RY_BASE, MIN_OPEN, HEAD_MIN_OPEN, physics constants
§2 clock      — clock_ns() / clock_sleep_ns()
§3 color      — 8 jellyfish color pairs + HUD, 256-color with 8-color fallback
§4 coords     — px_col(px) / px_row(py) — only conversion point
§5 entity     — AnimState, Jellyfish struct, smoothstep, curve_contract/expand
              — jelly_spawn(), jelly_anim_tick(), jelly_tick()
§6 draw       — draw_bell(), draw_tentacles()
§7 screen     — Scene, scene_init/tick/draw, Screen, screen_init/resize/render
§8 app        — App, signal handlers, main loop
```

## Data Flow Diagram

```
clock_ns()
    │ dt
    ▼
jelly_anim_tick(j, dt)
    │ updates: anim, anim_t, bell_open, wave_scale
    │ drives:  crown_f = bell_open
    │          head_f  = HEAD_MIN_OPEN + (1−HEAD_MIN_OPEN)×t
    │          tent1_f = tent2_f = 1.0
    ▼
jelly_tick(j, dt)
    │ vy *= expf(−DRAG_VY × dt)
    │ cy += vy × dt
    │ cx  = drift_cx + JELLY_DRIFT_A × sin(drift_phase)
    │ tent_phase[k] += TENT_WAVE_SPD × wave_scale × dt
    │ wrap: cy > tent_extent above screen → respawn from bottom
    ▼
draw_tentacles(win, j, cols, rows)
    │ Rx = BELL_RX_BASE × head_f
    │ lag_total = −vy × TENT_LAG
    │ for each tentacle k, segment seg:
    │     lateral = TENT_WAVE_AMP × wave_scale × sin(...)
    │     py = cy + seg×TENT_SEG_PX + lag_total×depth
    │     char by depth: | | : .
    ▼
draw_bell(win, j, cols, rows)
    │ Rx = BELL_RX_BASE × head_f
    │ crown_ny_limit = 0.88 + 0.12 × crown_f
    │ for each row r in [row_top, row_bot]:
    │     ny = (py − cy) / Ry
    │     if abs_ny > crown_ny_limit: skip
    │     nx_edge = sqrt(1 − ny²)
    │     x_left/right = cx ± Rx × nx_edge
    │     render: roof / crown / sides / rim / interior
    ▼
wnoutrefresh → doupdate
```

## Core Loop Pseudocode

```
init screen, init scene (spawn N_JELLIES_DEFAULT jellies)
while running:
    if SIGWINCH: resize screen, update max_px/max_py
    dt = (now − prev) / 1e9  clamped 0..0.1
    for each active jelly:
        jelly_anim_tick(j, dt)   // state machine + bell_open + wave_scale
        jelly_tick(j, dt)        // physics: vy, cy, cx, wrap
    erase()
    for each active jelly:
        draw_tentacles(j)        // sine wave + velocity lag
        draw_bell(j)             // ellipse scan-line + section fractions
    draw HUD (count, fps, keys)
    doupdate()
    sleep to 60fps
    handle key: q=quit spc=pause r=reset +=add -=remove
```

## Key Equations

**Vertical physics:**
```
IDLE:     vy(t) = vy₀ + GRAVITY × dt
CONTRACT: vy → −JET_THRUST at end
GLIDE:    vy(t) = vy₀ × exp(−DRAG_VY × t)   [exact solution, frame-rate independent]
```

**Bell width at row ny:**
```
nx_edge = sqrt(1 − ny²)
x_edge  = cx ± BELL_RX_BASE × head_f × nx_edge
```

**head_f remap:**
```
t      = (bell_open − MIN_OPEN) / (1 − MIN_OPEN)     // normalise to [0,1]
head_f = HEAD_MIN_OPEN + (1 − HEAD_MIN_OPEN) × t     // stretch to [HEAD_MIN_OPEN, 1]
```

**Tentacle segment position:**
```
lateral = TENT_WAVE_AMP × wave_scale × sin(tent_phase[k] − seg×K + k×PHASE_OFF)
lag     = −vy × TENT_LAG × (seg / (SEGS−1))
px      = base_x + lateral
py      = cy + seg × TENT_SEG_PX + lag
```

**Smoothstep (ease-in-out):**
```
smoothstep(t) = t² × (3 − 2t)    for t ∈ [0,1]
```

**Contract curve (fast ease-out):**
```
curve_contract(t) = 1 − (1−t)²   — snaps shut quickly, decelerates at end
```
