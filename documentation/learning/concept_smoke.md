# Pass 1 — smoke.c: Three-algorithm ASCII smoke with shared rendering pipeline

## Core Idea

A 2-D grid of floating-point density values is written by one of three swappable physics algorithms, then rendered through a shared Floyd-Steinberg + perceptual-LUT pipeline — exactly the same rendering architecture as `fire.c`. All three algorithms share helpers for warmup ramp, arch-shaped source seeding, and wind accumulation. Every tunable constant is a named `#define` in §1 so the algorithm functions contain only their own logic.

**Algorithm 0 — CA Diffusion:** Density rises from a bottom source row with ±2-column lateral jitter — wider than fire's ±1 — giving smoke's characteristic broad billow and lateral spread. Same Doom-style upward-copy-with-decay, but tuned for soft dispersal rather than sharp spires.

**Algorithm 1 — Particle Puffs:** A pool of 400 smoke puffs born at the arch source zone, each carrying position, velocity, and a lifetime. Rendered density = life² (quadratic fade: bright at birth, long gentle tail). Particles are bilinear-splatted onto the float density grid as a 2×2 tent filter for soft-edged puffs.

**Algorithm 2 — Vortex Advection:** Three slowly orbiting point vortices generate a 2-D velocity field via the 2D Biot-Savart law. Density is advected semi-Lagrangianly each tick — every cell looks up where the flow came from and samples there. Produces organic swirling, curling plumes.

Six color themes (gray, soot, steam, toxic, ember, arcane) switched manually with `t`.

## The Mental Model

Think of a smoke stack viewed from the side. The bottom row is the source — constantly emitting density shaped like an arch (hottest at the centre, zero at the edges). Each frame, the density field propagates upward while slowly dissipating.

The three algorithms differ in *how* density moves:
- **CA** copies one cell upward with a random sideways hop. Like billiard balls: simple, fast, chaotic.
- **Particles** are actual moving objects. Each puff has its own trajectory, drifts on wind turbulence, and fades over time. More physically intuitive.
- **Vortex** treats the air as a fluid with spinning whirlpools. The fluid carries density with it. The result looks the most natural and organic.

The display is identical to fire.c: 9 ASCII characters ordered by ink density (`" .,:coO0#"`) with Floyd-Steinberg dithering for smooth gradients. Soft rounded characters were chosen to reinforce the billowy nature of smoke vs. fire's sharp angular chars.

## Data Structures

### Scene (§8)
```
float *density;       — [rows * cols] current density, 0.0 to 1.0
float *prev_density;  — [rows * cols] density from the previous drawn frame
float *dither;        — [rows * cols] Floyd-Steinberg working buffer
float *work;          — [rows * cols] scratch buffer for vortex advection
int    cols, rows;
float  intensity;     — user-controlled smoke density [0.1, 1.0]
int    wind;          — wind velocity in cells/tick
int    wind_acc;      — accumulated wind offset (advanced once per tick in scene_tick)
int    theme;         — index into k_themes[] (0 = gray, 1 = soot, ...)
int    algo;          — 0=CA  1=Particle  2=Vortex
int    warmup;        — frame counter; warmup_scale() reads this

Particle parts[400];  — particle pool for algo 1
int     next_idx;     — round-robin spawn cursor for algo 1
Vortex  vorts[3];     — vortex state for algo 2
bool    paused;
bool    needs_clear;  — true after resize or theme change
```

`density` and `prev_density` are pointer-swapped after each draw (same diff-clear pattern as fire.c). `work` is a second full-size scratch buffer used only by the vortex advection algorithm to double-buffer the new density.

### SmokeTheme (§3)
```
const char *name;
int fg256[RAMP_N]; — 9 xterm-256 color indices, cold → dense
int fg8[RAMP_N];   — 9 standard-8 color fallbacks
attr_t attr8[RAMP_N]; — A_DIM / A_BOLD modifiers
```

### Particle (§6)
```
float x, y;    — position in grid cells
float vx, vy;  — velocity; vy negative = upward
float life;    — [1.0→0.0]; rendered density = life²
float decay;   — life lost per tick = 1/lifetime_ticks
bool  active;
```

### Vortex (§7)
```
float cx, cy;       — current centre in grid cells
float strength;     — Biot-Savart strength; positive = CCW, negative = CW
float orb_r;        — orbital radius in grid cells
float orb_a;        — current orbital angle (radians)
float orb_spd;      — angular speed in radians/tick
```

## The Main Loop

Each iteration:

1. **Resize check**: Rebuild the scene at new dimensions, preserving intensity and wind.

2. **Measure dt**: Nanosecond delta-time; clamp to 100ms.

3. **Drain sim accumulator**: Call `scene_tick()` as many times as needed to catch up at sim_fps.

4. **FPS counter update**: Every 500ms.

5. **Frame cap sleep**: Stay at ~60fps render rate.

6. **Draw**: `scene_draw()` → `density_draw()`. If `needs_clear` is set, call `erase()` once.

7. **HUD**: FPS, theme name, intensity, wind, algo name.

8. **Present**: `wnoutrefresh()` + `doupdate()`.

9. **Input**: pause, algo, theme, intensity, wind, speed.

## Non-Obvious Decisions

### Why wider jitter for smoke CA vs. fire CA?
Fire's CA uses ±1 lateral jitter to produce narrow rising spires — the visual signature of a flame. Smoke jitters ±2 columns (`CA_JITTER_RANGE=2`). The wider random walk makes each column of density spread sideways more aggressively, creating the broad, diffuse billow characteristic of smoke. This single constant change is what separates "fire behaviour" from "smoke behaviour" in the CA algorithm.

### Quadratic density fade for particles (life²)
Linear `life` would make a puff at half-life appear at half-density, which looks too uniform. Squaring it means a puff at half-life appears at one-quarter density. The puff is bright and opaque when freshly born, then spends most of its lifetime in a long gentle fade. This matches how real smoke puffs look: dense at emission, diffuse at the end.

### Bilinear splat (2×2 tent) vs. Gaussian splat (3×3)
Fire's particle system uses a 3×3 Gaussian splat (`splat3x3`) to build a dense filled flame body. Smoke particles use a 2×2 bilinear (tent filter) splat — simpler, covering only the 4 immediately surrounding cells. Smoke puffs are softer and more spread-out than fire embers, so the coarser filter is more appropriate. It also means neighbouring puffs blend together naturally as they overlap.

### Semi-Lagrangian advection for vortex
In Eulerian advection you'd push density forward from each cell. Semi-Lagrangian goes backwards: for each cell, trace back along the velocity field by `ADV_DT` steps, sample the density at that source position (bilinear interpolation), and write that as the new density. This is unconditionally stable regardless of `ADV_DT` — which is why `ADV_DT=0.8` is safe. Eulerian advection requires a CFL condition (dt × max_velocity ≤ 1 cell) that would restrict how fast the vortices can spin.

### Biot-Savart softening (VORT_EPS = 6.0)
The 2D Biot-Savart formula has `1/r²` which blows up as r→0. Adding `VORT_EPS` to the denominator gives `1/(r² + ε)` — a regularised vortex. Without it, cells near the vortex centre would receive enormous velocities, causing the advection to jump multiple cells and produce visual noise. `VORT_EPS=6.0` means the singularity is smoothed over a radius of ~2.4 cells, which is small enough to still feel like a point vortex but large enough to prevent runaway values.

### Wind accumulated once in scene_tick(), not in algo functions
The old code had `vortex_tick()` also advancing `wind_acc` internally, which meant wind was applied twice per tick when the vortex algo was active. Fixed: `scene_tick()` advances `wind_acc += wind` once before dispatching to any algo. The comment in `scene_tick()` makes this contract explicit so future algo functions know not to touch it.

### WARMUP_CAP = 200
The warmup counter in `warmup_scale(int *warmup)` is clamped to `WARMUP_CAP=200` after reaching it. Without this clamp, the counter would run forever and eventually overflow (all three algo functions call `warmup_scale` every tick). Clamping at `WARMUP_CAP` keeps the counter in a safe range while `warmup_scale()` returns exactly `1.0f` — the ramp reached full scale at `WARMUP_TICKS=80` and is merely held there.

### Vortex presets as file-scope static const arrays
The three vortex orbital parameters (radii, speeds, strengths, starting angles) live as `static const float VORT_ORB_FRACS[N_VORTS]` etc. in §1 presets alongside all other tunable constants. They used to be local arrays inside `vortex_init()`, which hid them from readers scanning §1 for "what can I tune". Hoisting them to file scope with comments makes the vortex personality immediately visible without needing to read the algo code.

## State Machines

The Scene has one explicit two-state lifecycle:

```
RUNNING ──── space key ────► PAUSED
   ▲                              │
   └───────── space key ──────────┘
```

Theme and algorithm are changed only by key presses — no auto-cycling:
- `t` → next theme (wraps at THEME_COUNT), resets warmup, sets needs_clear
- `a` → next algo (wraps at N_ALGOS=3), resets warmup, reinitialises vortices if switching to algo 2

## From the Source

**CA algorithm:** `ca_tick()` calls `warmup_scale(warmup)` first (returns ramp value AND increments counter), then `seed_source_row()`, then the upward-copy loop with `CA_JITTER_RANGE=2` jitter. The lateral offset is `rand() % (CA_JITTER_RANGE*2+1) - CA_JITTER_RANGE`, giving ±2 uniformly.

**Particle algorithm:** `particle_tick()` processes existing particles first (turbulence → move → decay → kill), then spawns `SPAWN_PER_TICK=5` new ones, then calls `warmup_scale(warmup)` just to advance the counter (return value discarded). Finally, it memsets density to 0 and rebuilds it entirely from the live particle list via bilinear splat. The grid is completely reconstructed each tick — there is no persistence in the density field itself.

**Vortex algorithm:** `vortex_tick()` calls `warmup_scale`, advances vortex orbital angles, computes Biot-Savart velocity per cell, then does the semi-Lagrangian back-trace into the `work` buffer, adds source injection at the bottom row, clamps to 1.0, then copies `work` → `density`.

**Math — Biot-Savart at point (px, py):**
```
for each vortex i:
    dx = px − vorts[i].cx
    dy = py − vorts[i].cy
    r2 = dx*dx + dy*dy + VORT_EPS
    vx_field += vorts[i].strength × (−dy) / r2
    vy_field += vorts[i].strength × ( dx) / r2
```
The `−dy` for vx and `+dx` for vy is the standard 2D curl: a CCW vortex creates upward flow to its right and downward flow to its left.

**Math — Semi-Lagrangian advection:**
```
for each cell (x, y):
    back_x = x − vx_field(x,y) × ADV_DT
    back_y = y − vy_field(x,y) × ADV_DT
    work[y*cols+x] = bilinear(density, back_x, back_y) × (1 − decay)
                   + source(x,y)
```
Source is added after the advection so freshly emitted density is never immediately swept away.

## Key Constants and What Tuning Them Does

| Constant | Default | Effect if changed |
|---|---|---|
| `CA_REACH_FRAC` | 0.60 | Fraction of rows the smoke column targets; raise = taller columns |
| `CA_JITTER_RANGE` | 2 | Lateral jitter in ±cols; raise = broader billowing; lower = narrow rising columns |
| `ARCH_MARGIN_FRAC` | 0.06 | Empty margin at each side edge; raise = narrower source |
| `WARMUP_TICKS` | 80 | Ticks to ramp 0→1 on startup or algo change |
| `WARMUP_CAP` | 200 | Counter ceiling; prevents overflow; scale stays 1.0 after warmup |
| `PART_LIFE_MIN` | 35 | Minimum particle lifetime; longer = taller particle smoke |
| `PART_LIFE_RANGE` | 35 | Random range on lifetime; higher = more height variation |
| `SPAWN_PER_TICK` | 5 | New particles per tick; raise = denser smoke, but eats pool faster |
| `ADV_DT` | 0.8 | Semi-Lagrangian time step; values < 1 are stable; raise = faster advection |
| `VORT_EPS` | 6.0 | Biot-Savart softening; raise = smoother but weaker vortex core |
| `VORT_REACH_FRAC` | 0.55 | Target smoke height for vortex decay; raise = taller vortex plume |
| `VORT_ORB_FRACS[i]` | 0.20, 0.30, 0.18 | Orbit radii as fraction of cols; raise = vortices farther apart |
| `VORT_STRENGTHS[i]` | 2.5, −1.8, 1.4 | Vortex power; negative = clockwise; raise magnitude = more curling |

## Open Questions for Pass 3

- The particle density grid is completely rebuilt from scratch each tick (`memset` to 0, then all active particles splat into it). Why not advect particle positions and add to the existing density? What visual difference would accumulation vs. full-rebuild produce?
- `bilinear_sample()` uses clamp-to-edge boundary conditions: `x0 = clamp(x0, 0, cols-1)`. What would happen at the edges if you used zero-padding instead? Would you see density "pile up" at the borders during vortex advection?
- The Biot-Savart velocity is clamped to `±ADV_VEL_CAP=2.0` cells per tick. What happens visually if you remove this clamp? Why is the combination of `ADV_VEL_CAP=2.0` and `ADV_DT=0.8` the right choice (the back-trace steps at most 1.6 cells)?
- `particle_spawn()` uses rejection sampling (up to 8 attempts) to bias spawn positions toward the arch centre. What distribution do you get if you skip rejection sampling and spawn uniformly across the arch span? Would the visual difference be noticeable?
- The vortex orbits are updated by `vortex_advance_orbits()` which advances each vortex angle by `orb_spd` radians per tick. The two vortices with negative `VORT_ORB_SPDS` spin clockwise. What happens to the smoke pattern when two adjacent vortices spin in opposite directions vs. the same direction?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `density[rows*cols]` | `float[]` | ~40 KB | current density values |
| `prev_density[rows*cols]` | `float[]` | ~40 KB | last-drawn density for diff clearing |
| `dither[rows*cols]` | `float[]` | ~40 KB | Floyd-Steinberg scratch buffer |
| `work[rows*cols]` | `float[]` | ~40 KB | vortex advection double-buffer |
| `parts[MAX_PARTS]` | `Particle[]` | ~10 KB | particle pool (algo 1) |
| `vorts[N_VORTS]` | `Vortex[]` | 72 bytes | vortex state (algo 2) |

# Pass 2 — smoke: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 presets | All named `#define` constants + file-scope vortex preset arrays |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 theme | 9-level ASCII ramp `" .,:coO0#"`, LUT breaks, SmokeTheme struct, 6 themes |
| §4 shared helpers | `warmup_scale(int *warmup)`, `arch_envelope()`, `seed_source_row()` |
| §5 algo 0 | `ca_tick()` — CA diffusion with wide jitter |
| §6 algo 1 | `particle_tick()`, `particle_spawn()` — pool + bilinear splat |
| §7 algo 2 | `vortex_tick()`, `vortex_advance_orbits()`, `bilinear_sample()` — Biot-Savart + semi-Lagrangian |
| §8 scene | Owns all state; `scene_tick()` dispatches to active algo; `density_draw()` renders |
| §9 screen | ncurses layer + HUD |
| §10 app | Signal handlers, main dt loop, key handling |

---

## Data Flow Diagram

```
scene_tick():
  wind_acc += wind   ← accumulated ONCE here before dispatch

  dispatch on scene.algo:
    0 → ca_tick(density, cols, rows, intensity, wind_acc, &warmup)
    1 → particle_tick(parts, &next_idx, density, cols, rows, intensity, wind_acc, &warmup)
    2 → vortex_tick(density, work, vorts, cols, rows, intensity, wind_acc, &warmup)

── shared helpers ────────────────────────────────────────────────────────

warmup_scale(int *warmup):
  s = (*warmup < WARMUP_TICKS) ? (*warmup / WARMUP_TICKS) : 1.0
  (*warmup)++
  if (*warmup > WARMUP_CAP) *warmup = WARMUP_CAP
  return s

arch_envelope(x, cols, wind_acc):
  margin = ARCH_MARGIN_FRAC * cols
  t = (x − margin − wind_acc) / (cols − 2*margin)
  if t < 0 or t > 1: return 0
  edge = min(t, 1−t) * 2
  return edge * edge

seed_source_row(density, cols, rows, intensity, wind_acc, wscale):
  fy = rows - 1
  for each x:
    arch = arch_envelope(x, cols, wind_acc)
    jitter = SRC_JITTER_BASE + rand * SRC_JITTER_RANGE
    density[fy*cols+x] = intensity * arch * jitter * wscale

── algo 0: ca_tick ───────────────────────────────────────────────────────

  wscale = warmup_scale(&warmup)
  seed_source_row(density, cols, rows, intensity, wind_acc, wscale)

  avg_decay = 1.0 / (rows * CA_REACH_FRAC)
  d_base = avg_decay * CA_DECAY_BASE_FRAC  (floored at CA_DECAY_BASE_MIN)
  d_rand = avg_decay * CA_DECAY_RAND_FRAC  (floored at CA_DECAY_RAND_MIN)

  for y = 0 to rows-2:
    for each x:
      rx = x + rand(-CA_JITTER_RANGE .. +CA_JITTER_RANGE)  [±2 cols]
      src = density[(y+1)*cols + clamp(rx)]
      decay = d_base + rand * d_rand
      density[y*cols+x] = max(0, src − decay)

── algo 1: particle_tick ─────────────────────────────────────────────────

  for each active particle:
    vx += rand(-PART_TURB_STEP/2, +PART_TURB_STEP/2)
    vx *= PART_VX_DAMP
    x += vx;  y += vy
    life -= decay
    if out-of-bounds or life <= 0: deactivate

  spawn SPAWN_PER_TICK new particles via particle_spawn()
  warmup_scale(&warmup)   ← advance counter only

  memset(density, 0)
  for each active particle:
    pd = life * life   [quadratic fade]
    bilinear splat pd onto 4 surrounding cells (tent filter)

── algo 2: vortex_tick ───────────────────────────────────────────────────

  wscale = warmup_scale(&warmup)
  vortex_advance_orbits(vorts, cols, rows)

  for each cell (x, y):
    compute Biot-Savart velocity from all N_VORTS vortices:
      for each vortex i:
        dx = x − vorts[i].cx,  dy = y − vorts[i].cy
        r2 = dx*dx + dy*dy + VORT_EPS
        vx_field += strength[i] * (−dy) / r2
        vy_field += strength[i] * ( dx) / r2
    clamp vx_field, vy_field to ±ADV_VEL_CAP

    back_x = x − vx_field * ADV_DT
    back_y = y − vy_field * ADV_DT
    advected = bilinear_sample(density, back_x, back_y)

    arch = arch_envelope(x, cols, wind_acc)
    jitter = SRC_JITTER_BASE + rand * SRC_JITTER_RANGE
    source = (y == rows-1) ? intensity * arch * jitter * wscale : 0

    work[y*cols+x] = clamp(advected * (1 − decay) + source, 0, 1)

  copy work → density

── density_draw (shared by all algos) ──────────────────────────────────

  gamma pass: for each cell v > 0: dither[i] = pow(v, 1/2.2)
              for each cell v ≤ 0: dither[i] = -1 (cold marker)

  Floyd-Steinberg + draw (left-to-right, top-to-bottom):
    if dither[i] < 0:
      if prev_density[i] > 0: mvaddch(y, x, ' ')   ← erase stale char
      else: skip
    else:
      idx = lut_index(dither[i])
      err = dither[i] − lut_midpoint(idx)
      spread err to warm neighbours: right 7/16, below-left 3/16,
                                     below 5/16, below-right 1/16
      mvaddch(y, x, k_ramp[idx]) with ramp_attr(idx, theme)

  swap density ↔ prev_density
  memcpy density from prev_density  (re-seed for next tick)
```

---

## Function Breakdown

### warmup_scale(int *warmup) → float
Purpose: return startup ramp multiplier [0..1] and increment counter
Steps:
1. If *warmup < WARMUP_TICKS: s = *warmup / WARMUP_TICKS; else s = 1.0
2. (*warmup)++
3. If *warmup > WARMUP_CAP: *warmup = WARMUP_CAP
4. return s
Note: takes `int *warmup` (pointer to Scene's counter), not a Grid struct. Smoke has no Grid wrapper; algo functions receive the counter pointer directly from Scene.

---

### arch_envelope(x, cols, wind_acc) → float
Purpose: squared arch weight [0..1] at column x, wind-shifted
Steps: (see fire.c concept for identical logic; margin is 6% not 4%)

---

### seed_source_row(density, cols, rows, intensity, wind_acc, wscale)
Purpose: fill bottom row with arch-shaped source density
Steps:
1. for each x: arch = arch_envelope; jitter = random in [SRC_JITTER_BASE, +RANGE]
2. density[bottom][x] = intensity × arch × jitter × wscale

---

### ca_tick(density, cols, rows, intensity, wind_acc, warmup)
Purpose: one CA diffusion step — source + upward propagation with wide jitter
Steps:
1. wscale = warmup_scale(warmup)
2. seed_source_row(...)
3. Compute adaptive decay from CA_REACH_FRAC and rows
4. For each cell top-to-bottom: sample jitter-offset neighbour below; subtract decay

---

### particle_spawn(p, cols, rows, intensity, wind_acc, warmup)
Purpose: initialise one new smoke puff at the arch source zone
Steps:
1. Compute wscale from warmup (read-only, does NOT increment)
2. Rejection-sample spawn column (up to 8 attempts, weighted by arch × intensity × wscale)
3. Set p->x, p->y at bottom row; random vx, vy; compute decay from random lifetime
Note: does NOT call warmup_scale(); reads warmup directly. Counter advances only in particle_tick().

---

### particle_tick(parts, next_idx, density, cols, rows, intensity, wind_acc, warmup)
Purpose: advance particles, spawn new ones, rebuild density grid by bilinear splat
Steps:
1. Advance all active particles (turbulence, velocity, life decay, OOB kill)
2. Spawn SPAWN_PER_TICK new particles round-robin from pool
3. warmup_scale(warmup) — advance counter, discard return value
4. memset(density, 0)
5. For each active particle: bilinear splat life² onto 4 surrounding cells
6. Clamp density to [0, 1]

---

### vortex_advance_orbits(vorts, cols, rows)
Purpose: step each vortex forward by its orbital speed
Steps:
1. For each vortex i: orb_a += orb_spd
2. Recompute cx = screen_cx + orb_r × cos(orb_a)
3. Recompute cy = screen_cy + orb_r × sin(orb_a)

---

### bilinear_sample(grid, sx, sy, cols, rows) → float
Purpose: interpolated grid lookup at non-integer position (sx, sy)
Steps:
1. Clamp floor/ceil indices to grid bounds (Neumann: zero gradient at edge)
2. Compute fractional offsets tx, ty
3. Return weighted sum of 4 surrounding cells: (1-tx)(1-ty)*v00 + tx(1-ty)*v10 + ...

---

### vortex_tick(density, work, vorts, cols, rows, intensity, wind_acc, warmup)
Purpose: one semi-Lagrangian advection step driven by Biot-Savart vortex field
Steps:
1. wscale = warmup_scale(warmup); advance orbits
2. Compute decay from VORT_REACH_FRAC
3. For each cell (x, y):
   a. Sum Biot-Savart velocity from all N_VORTS vortices; clamp to ±ADV_VEL_CAP
   b. back_x = x − vx * ADV_DT; back_y = y − vy * ADV_DT
   c. advected = bilinear_sample(density, back_x, back_y) × (1 − decay)
   d. source = seed value if y == rows−1 (bottom row only)
   e. work[y*cols+x] = clamp(advected + source, 0, 1)
4. memcpy work → density

---

## Interactions Between Modules

```
App
 └── main loop: sim_accum → scene_tick → [ca_tick | particle_tick | vortex_tick]
                render → density_draw → Floyd-Steinberg → mvaddch
                input → algo/theme/intensity/wind adjustments

§4 shared helpers (called by all three algos)
 ├── warmup_scale()      — ramp 0→1 over WARMUP_TICKS, increments scene.warmup
 ├── arch_envelope()     — squared arch, wind-shifted
 └── seed_source_row()   — bottom row arch-shaped source

Scene
 ├── density[]           — current frame physics (written by active algo)
 ├── prev_density[]      — previous drawn frame (for diff clearing)
 ├── dither[]            — Floyd-Steinberg scratch buffer
 ├── work[]              — vortex advection double-buffer (algo 2 only)
 ├── parts[MAX_PARTS]    — particle pool (algo 1 only)
 └── vorts[N_VORTS]      — vortex state (algo 2 only)

§3 theme
 ├── k_ramp[] = " .,:coO0#"  — 9 soft smoky chars
 ├── k_lut_breaks[]      — gamma-corrected density thresholds
 ├── k_themes[6]         — SmokeTheme structs
 ├── theme_apply()       — updates ncurses init_pair calls
 └── ramp_attr()         — builds attr_t for each ramp level
```

---

## Floyd-Steinberg Dithering (same as fire.c)

```
Goal: map continuous float [0,1] to discrete ramp indices 0..8
      while minimising visible banding.

For each cell (left→right, top→bottom):
  1. Quantize: idx = lut_index(v)   (snap to nearest ramp level)
  2. Error: err = v − lut_midpoint(idx)  (how far off we are)
  3. Spread error to neighbours:
        [  this  | +7/16 ]
        [+3/16 | +5/16 | +1/16]

  → Neighbours receive fractional correction.
  → Only warm cells (dither >= 0) receive error.
  → Cold cells are excluded to prevent error crossing the smoke boundary.
```
