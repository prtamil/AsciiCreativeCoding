# Pass 1 — pulsar_rain.c: Rotating Pulsar Neutron Star

## Core Idea

Models a rotating neutron star (pulsar): N evenly-spaced beams sweep continuously around a central `@` core like a lighthouse. Each beam leaves an angular wake of fading matrix characters behind it as it rotates. The number of beams is user-controlled (1–16), and render interpolation keeps the rotation smooth at 60 fps even though physics run at 20 Hz.

---

## The Mental Model

A lighthouse with N beams, all spinning together at the same rate. Each beam is not a solid line — it is a streak of shimmering matrix characters that fade from bright white at the leading edge to nearly invisible at the trailing edge over an arc of ~46°. The `@` at the centre is always on top of everything.

As N increases: N=1 is a single sweeping arm, N=2 is the classic pulsar pair (opposite beams, like a propeller), N=3 is a tri-blade, N=4 is a cross, N=12+ creates a dense corona where wakes overlap and merge into a continuous glow.

The key non-obvious detail: the simulation only computes 17 trigonometry values per beam per frame (one per wake slot), regardless of how many radial samples are drawn. Each of the 80 radial positions along the beam reuses those pre-computed cos/sin pairs.

---

## Data Structures

### `Pulsar` struct

| Field | Meaning |
|---|---|
| `angle` | Current beam angle in radians; advances by `spin` each tick |
| `spin` | Rotation rate in radians/tick (range SPIN_MIN=0.03 to SPIN_MAX=0.60) |
| `n_beams` | Number of beams (1–16); beams evenly spaced at 2π/n_beams |
| `cache[N_RADII][WAKE_LEN+1]` | Shimmer characters: [ri][k] is the char at radial sample ri, wake slot k |
| `max_r` | Distance from centre to farthest screen corner (isotropic cell units) |
| `r_step` | `max_r / N_RADII` — spacing between radial samples |
| `cx, cy` | Screen centre in terminal columns/rows |

### Wake slot indexing

`k = 0` is the beam head (leading edge, brightest).
`k = WAKE_LEN` is the tail (oldest, dimmest).
Each slot is `WAKE_STEP = 0.05 rad` behind the previous one.
Total wake arc = `WAKE_LEN × WAKE_STEP = 16 × 0.05 = 0.8 rad ≈ 46°`.

---

## Key Equations

### N-beam angular spacing
```
step = 2π / n_beams
beam_b_angle = draw_angle + b × step,  b ∈ {0, 1, …, n_beams-1}
```

### Render interpolation
```
draw_angle = beam_angle + spin × alpha
alpha = sim_accum / TICK_NS  ∈ [0, 1)
```
Projects the beam forward to its exact sub-tick position. No prev_angle storage needed because spin is constant within a tick — forward extrapolation is identical to lerp.

### Position of a wake character
```
wa = beam_angle − k × WAKE_STEP
col = cx + round(r × cos(wa))
row = cy + round(r × sin(wa) × ASPECT)
```
ASPECT = 0.45 is baked into `sin_a` so all position math uses simple `r × cos_a / r × sin_a`.

### Isotropic radius
`max_r = sqrt((cols/2)² + (rows/2 / ASPECT)²) × 1.05`

R is measured in x-cell units; y extends as R×ASPECT rows, making beams reach all four corners while appearing visually circular.

---

## The Draw Algorithm

### Per-beam (called N times per frame)

```
1. Pre-compute direction vectors for all WAKE_LEN+1 slots:
   for k in 0..WAKE_LEN:
     wa = base_angle - k * WAKE_STEP
     cw[k] = cos(wa)
     sw[k] = sin(wa) * ASPECT

2. For each radial sample ri in 0..N_RADII-1:
   r = (ri + 1) * r_step
   for k = WAKE_LEN down to 0:      ← dim-first so head wins at overlaps
     col = cx + round(r * cw[k])
     row = cy + round(r * sw[k])
     if in bounds: draw cache[ri][k] with wake_attr(k)

3. Draw '@' core last (always wins over all beams)
```

### Why descending k (dim→bright)?
Near the centre, small r means multiple k values round to the same terminal cell. Drawing dim entries first and the bright head (k=0) last ensures the head always overwrites — the correct result with no extra logic.

### Why pre-compute 17 trig values and reuse across 80 radii?
Each radial sample at the same angular position uses the same cos/sin. Pre-computing once per wake slot and reusing for all N_RADII samples cuts the trig calls from N_RADII×(WAKE_LEN+1) = 1360 to just WAKE_LEN+1 = 17 per beam (34 total for 2 beams).

---

## Non-Obvious Decisions

### `n_beams` stored in Pulsar, not App
Beam count is a simulation parameter, not just a display parameter. Keeping it in the Pulsar struct means it survives resize (passed into `pulsar_init`) and reset (same). The app-local `n_beams` variable mirrors it and is what the key handler modifies — both are updated together.

### Cache shared by all beams
All beams draw from the same `cache[ri][k]` array. Two beams at the same `(ri, k)` display the same character but at completely different screen positions (separated by 2π/N angle). The shimmer is still correct and independent-looking. Per-beam caches would quadruple memory usage with no visual benefit.

### Wake overlaps at high N
At N=12, beams are 30° apart and each wake spans 46°, so adjacent wakes overlap by 16°. The later-drawn beam simply overwrites the earlier one at shared cells. This creates a denser, merged-corona effect that looks intentional.

### `pulsar_init` preserves n_beams across resize and reset
The `r` key resets physics (angle, cache) but passes the current `n_beams` into `pulsar_init`, so the user-chosen beam count survives a reset. Same for terminal resize.

---

## From the Source

**Algorithm:** N-beam spacing: `step = 2π / n_beams; beam_b_angle = draw_angle + b × step` for b ∈ {0..n_beams-1}. Beams share one `cache[N_RADII][WAKE_LEN+1]` array — two beams at same `(ri,k)` show the same char but at completely different screen positions. Shimmer: ~75% of cache entries replaced each tick (`rand()%4 != 0`).

**Math:** `ASPECT=0.45` baked into `sw[k] = sin(wa) * ASPECT` so all row positions use `r × sw[k]` — no per-cell trig. Isotropic radius: `max_r = sqrt((cols/2)² + (rows/2/ASPECT)²) × 1.05` — beams reach all four corners and look visually circular. Wake arc = `WAKE_LEN × WAKE_STEP = 16 × 0.05 = 0.8 rad ≈ 46°` total angular trail.

**Performance:** Per beam per frame: only `WAKE_LEN+1 = 17` trig calls (one per wake slot direction vector). Each of `N_RADII=80` radial samples reuses those pre-computed `cw[k], sw[k]` pairs. Total: 34 trig calls for 2 beams vs 1360 without pre-computation. Draw order: `k = WAKE_LEN` down to `0` (dim→bright) so the bright head overwrites dim slots at cells where inner radii overlap.

**Rendering:** Core `@` drawn last — always on top of all beams. `pulsar_init` preserves `n_beams` across resize and reset (passed in, not re-defaulted). At high N (≥12), adjacent wakes overlap by 16° and the later-drawn beam overwrites earlier ones — creates a dense merged-corona effect.

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|---|---|---|
| `N_RADII` | 80 | Radial resolution; lower = gaps in the beam, higher = denser line |
| `WAKE_LEN` | 16 | Angular trail depth; longer = wider fade arc |
| `WAKE_STEP` | 0.05 rad | Angular gap between wake slots; smaller = tighter arc, more trig reuse opportunity |
| `SPIN_DEF` | 0.15 rad/tick | ~0.48 rot/s at 20 Hz; increase for faster spin |
| `BEAMS_DEF` | 2 | Classic pulsar pair; 1=single, 4=cross, 12=corona |
| `ASPECT` | 0.45 | Terminal cell height/width ratio; adjust if rings look elliptical |

---

## Wake Brightness Mapping

| Slot k | Attribute |
|---|---|
| 0 (head) | CP_HEAD = white \| A_BOLD |
| 1 | CP_HOT \| A_BOLD |
| 2 | CP_BRIGHT \| A_BOLD |
| 3–8 (mid) | CP_MID |
| 9–14 | CP_DARK |
| 15–16 (tail) | CP_FADE \| A_DIM |

---

## Themes

5 themes (green / amber / blue / plasma / fire) each define 5 foreground colors for the CP_FADE → CP_HOT gradient. CP_HEAD and CP_CORE are always white. Switching theme calls `theme_apply()` which re-registers all color pairs — takes effect on next `doupdate()`.

---

## State Machine

There is no explicit state machine. The Pulsar struct is a single continuous state: `angle` advances by `spin` every tick, wraps at 2π. There are no modes or transitions.

---

## Open Questions for Pass 3

1. What happens visually if WAKE_STEP is increased to π/(WAKE_LEN) so the full wake covers a half circle? Try it.
2. Can you give each beam its own independent spin rate (differential rotation)? What data structure change is needed?
3. The cache is shared by all beams. If you wanted per-beam character independence, how would you restructure it? What is the memory cost?
4. At N=16 with a wide WAKE_STEP the entire screen fills. Is there a mathematical relationship between N, WAKE_LEN, WAKE_STEP, and screen coverage?
5. What would the simulation look like if spin varied sinusoidally over time (pulsing fast-slow-fast like a real pulsar)?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_beam_angle` | `float` | scalar | current beam rotation angle (radians); advances by spin each tick |
| `g_spin` | `float` | scalar | angular velocity in rad/tick (default SPIN_DEF=0.15) |
| `g_n_beams` | `int` | scalar | active beam count (1–16, default 2) evenly spaced at 360°/N |
| `char_cache[N_RADII][WAKE_LEN+1]` | `char[]` | ~1.4 KB | matrix characters; ~75% randomised each tick for shimmer |
| `N_RADII` | `int` constant | N/A | radial samples along each beam (80) |
| `WAKE_LEN` | `int` constant | N/A | angular wake depth in slots behind head (16) |
| `WAKE_STEP` | `float` constant | N/A | angular gap between wake slots in radians (0.05) |
| `ASPECT` | `float` constant | N/A | row/column aspect correction for beam position (0.45) |

# Pass 2 — pulsar_rain.c: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | N_RADII=80, WAKE_LEN=16, WAKE_STEP=0.05, SIM_FPS=20, RENDER_FPS=60, SPIN_MIN/DEF/MAX, BEAMS_MIN=1/DEF=2/MAX=16, ASPECT=0.45 |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 theme/color | 5 named themes; 8 color pairs (CP_FADE..CP_HUD); `theme_apply()`, `wake_attr()` |
| §4 pulsar | `Pulsar` struct; `pulsar_init()`, `pulsar_tick()`, `draw_beam()`, `pulsar_draw()` |
| §5 screen | ncurses init/resize; `screen_hud()` |
| §6 app/main | Fixed-timestep loop, input, signal handling |

---

## Data Flow Diagram

```
clock_ns() → dt measurement
    │
    ▼
sim_accum += dt
    │
    ├─ [while sim_accum >= TICK_NS]──► pulsar_tick()
    │                                       │
    │                                       ├─ angle += spin; wrap at 2π
    │                                       └─ shimmer: ~75% of cache[][] refreshed
    │
    └─ alpha = sim_accum / TICK_NS  ∈ [0, 1)
           │
           ▼
    pulsar_draw(alpha)
           │
           ├─ draw_angle = angle + spin * alpha
           ├─ step = 2π / n_beams
           ├─ for b in 0..n_beams-1:
           │     draw_beam(draw_angle + b * step)
           │         │
           │         ├─ pre-compute cw[k], sw[k] for k=0..WAKE_LEN
           │         └─ for ri=0..N_RADII-1:
           │               r = (ri+1) * r_step
           │               for k=WAKE_LEN..0:
           │                 col = cx + round(r * cw[k])
           │                 row = cy + round(r * sw[k])
           │                 draw cache[ri][k] with wake_attr(k)
           │
           └─ draw '@' at (cx, cy) with CP_CORE | A_BOLD  ← always last
```

---

## Function Breakdown

### pulsar_init(p, cols, rows, spin, n_beams)
Purpose: Initialise or re-initialise a Pulsar for the given terminal size.
Steps:
1. `memset(p, 0)` — zero all fields.
2. `cx = cols / 2`, `cy = rows / 2`.
3. `spin = spin`, `angle = 0`, `n_beams = n_beams`.
4. `hx = cols * 0.5`, `hy = rows * 0.5 / ASPECT`.
5. `max_r = sqrt(hx² + hy²) × 1.05` — covers all four corners.
6. `r_step = max_r / N_RADII`.
7. Fill `cache[ri][k]` with random chars.

### pulsar_tick(p)
Purpose: Advance one simulation step.
Steps:
1. `angle += spin`.
2. If `angle > 2π`: `angle -= 2π` (wrap, not reset — preserves sub-revolution fraction).
3. For all `ri` and `k`: if `rand() % 4 != 0`, replace `cache[ri][k]` with a new random char (~75% refresh).

### draw_beam(p, base_angle, cols, rows)
Purpose: Draw one beam and its angular wake to stdscr.
Steps:
1. Pre-compute 17 direction vectors: `for k=0..WAKE_LEN: wa=base_angle - k*WAKE_STEP; cw[k]=cos(wa); sw[k]=sin(wa)*ASPECT`.
2. `for ri=0..N_RADII-1: r = (ri+1) * r_step`.
3. `for k=WAKE_LEN..0` (dim→bright):
   - `col = cx + round(r * cw[k])`, `row = cy + round(r * sw[k])`.
   - Skip if out of bounds.
   - `attron(wake_attr(k)); mvaddch(row, col, cache[ri][k]); attroff`.

### pulsar_draw(p, alpha, cols, rows)
Purpose: Draw all N beams then the core.
Steps:
1. `draw_angle = angle + spin * alpha`.
2. `step = 2π / n_beams`.
3. `for b=0..n_beams-1: draw_beam(p, draw_angle + b*step, cols, rows)`.
4. Draw `'@'` at `(cx, cy)` with `CP_CORE | A_BOLD`.

### wake_attr(k) → attr_t
Purpose: Map wake slot index to ncurses attribute.
```
k == 0           → CP_HEAD   | A_BOLD
k == 1           → CP_HOT    | A_BOLD
k == 2           → CP_BRIGHT | A_BOLD
k <= WAKE_LEN/2  → CP_MID
k <= WAKE_LEN-2  → CP_DARK
else             → CP_FADE   | A_DIM
```

---

## Pseudocode — Core Loop

```
setup:
  screen_init()
  g_has_256 = (COLORS >= 256)
  theme_apply(0)
  n_beams = BEAMS_DEF
  spin = SPIN_DEF
  pulsar_init(&psr, cols, rows, spin, n_beams)
  frame_time = clock_ns()

loop while g_running:

  if need_resize:
    screen_resize(&cols, &rows)
    pulsar_init(&psr, cols, rows, spin, n_beams)  ← preserves n_beams
    reset frame_time, sim_accum

  dt = clock_ns() - frame_time  (capped at 150ms)
  frame_time = now

  sim_accum += dt
  while sim_accum >= TICK_NS:
    pulsar_tick(&psr)
    sim_accum -= TICK_NS

  alpha = sim_accum / TICK_NS

  update FPS display every 500ms

  clock_sleep_ns(FRAME_NS - elapsed)  ← cap at 60fps

  erase()
  pulsar_draw(&psr, alpha, cols, rows)
  screen_hud(cols, fps, psr.spin, psr.n_beams, theme_idx)
  wnoutrefresh(); doupdate()

  switch getch():
    q/Q/ESC  → g_running = 0
    t        → theme_idx = (theme_idx+1) % THEME_COUNT; theme_apply
    + / =    → spin += SPIN_STEP; clamp to SPIN_MAX; psr.spin = spin
    -        → spin -= SPIN_STEP; clamp to SPIN_MIN; psr.spin = spin
    ]        → n_beams++; clamp to BEAMS_MAX; psr.n_beams = n_beams
    [        → n_beams--; clamp to BEAMS_MIN; psr.n_beams = n_beams
    r / R    → pulsar_init(&psr, cols, rows, spin, n_beams)
```

---

## Interactions Between Modules

```
main() [§6]
  ├─ pulsar_init()     [§4] — setup/resize/reset
  ├─ pulsar_tick()     [§4] — angle advance + shimmer
  ├─ pulsar_draw()     [§4]
  │     └─ draw_beam() [§4] × n_beams
  │           └─ wake_attr() [§3] — brightness per slot
  ├─ theme_apply()     [§3] — color pair re-registration on 't'
  └─ screen_hud()      [§5] — shows fps, rps, beam count, theme

Shared mutable state:
  - psr.angle    : only pulsar_tick writes, pulsar_draw reads
  - psr.n_beams  : only key handler writes, pulsar_draw reads
  - psr.cache    : pulsar_tick writes (~75%), draw_beam reads
```
