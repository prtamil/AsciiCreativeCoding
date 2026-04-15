# Pass 1 — fern.c: Barnsley Fern IFS fractal, animated point-by-point

## Core Idea

Four affine transforms are applied repeatedly to a single floating-point point. Each transform is chosen randomly with a specific probability. The orbit of this iterated function system traces the Barnsley Fern attractor — the most famous IFS fractal. Points are computed N_PER_TICK=400 per tick and plotted one cell at a time, so you can watch the fern emerge: first the stem appears (low-probability transform), then the main leaflets fill in (high-probability transform), then the left and right side fronds (medium-probability transforms). After TOTAL_ITERS=80000 the fern is held briefly then resets. Color is assigned by the y coordinate (height): medium green at the base, bright lemon-green at the tips.

## The Mental Model

Imagine describing a fern leaf to a computer using only four rules:
1. **Stem** (1% of the time): collapse everything to near the base — this creates the main stem.
2. **Main body** (85% of the time): slightly shrink and push upward — this fills in all the main leaflets.
3. **Left frond** (7%): rotate and compress to create the smaller left branch.
4. **Right frond** (7%): mirror-rotate to create the smaller right branch.

The remarkable thing is that these four simple rules, applied randomly with the given probabilities, produce a near-perfect botanical fern — including the self-similar sub-fronds that look like miniature versions of the whole fern. This is because the four transforms collectively cover the entire space occupied by the fern, and nothing else.

Unlike the Sierpinski chaos game, you cannot see the individual transforms at work — they all paint on the same continuous fern shape. The probabilities are tuned so that parts of the fern with more "leaflet density" (the main body) get more points than sparse parts (the stem).

## Data Structures

### Grid (§4)
```
uint8_t cells[GRID_ROWS_MAX][GRID_COLS_MAX]   — color band per plotted cell (0=empty)
int     rows, cols
int     iters_done
int     done_ticks
```

### IFS state (§5)
```
float x, y    — current IFS coordinate (Barnsley fern space: x ∈ [−2.18, 2.66], y ∈ [0, 9.998])
```

The four Barnsley transforms:
```
f1: (x, y) → (0,         0.16·y)                     prob  1%  — stem
f2: (x, y) → (0.85·x + 0.04·y,  −0.04·x + 0.85·y + 1.6)  prob 85%  — main body
f3: (x, y) → (0.20·x − 0.26·y,   0.23·x + 0.22·y + 1.6)  prob  7%  — left frond
f4: (x, y) → (−0.15·x + 0.28·y,  0.26·x + 0.24·y + 0.44) prob  7%  — right frond
```

### Scene (§6)
```
Grid  grid
IFS state (x, y)
bool  paused
```

## The Main Loop

1. Resize: recompute fern-to-terminal mapping coefficients, reset grid.
2. Measure dt.
3. Ticks: compute N_PER_TICK=400 IFS iterations per tick; for each, pick transform by probability, apply, map IFS (x,y) to terminal (col,row), plot cell with height color.
4. After TOTAL_ITERS=80000: hold for DONE_PAUSE_TICKS=90, then reset.
5. Draw: for each non-zero cell, write `.` with height-based color.
6. HUD: fps, progress, speed.
7. Input: r=reset, p=pause, [/]=speed.

## Non-Obvious Decisions

### Why independent x and y scale factors?
The Barnsley fern has an intrinsic coordinate range: x ∈ [−2.18, 2.66] (width ≈ 4.84) and y ∈ [0, 9.998] (height ≈ 10). The aspect ratio is ≈ 2.07:1. A terminal cell is roughly 2× taller than wide. So mapping `fern_x_range` to `cols` and `fern_y_range / ASPECT_R` to `rows` produces a fern that looks proportionate. If you used the same scale for x and y the fern would be squashed.

### 1% stem probability
The stem (f1) simply multiplies y by 0.16 and zeroes x — it collapses the point to the base of the stem. Only 1% of iterations use this, which means the stem has very few points compared to the dense leaflets. This matches the visual appearance of a real fern: the stem is a thin line, not a thick mass.

### 85% main body probability
f2 is the dominant transform. It applies a slight rotation and scale (the 0.85 shrink) with a 1.6 vertical offset. This creates all the main leaflets and their recursive sub-leaflets. Most of the 80000 iterations go here.

### f3 and f4 are mirror transforms
f3 (left frond) and f4 (right frond) each have 7% probability and produce the characteristic curving side fronds. Together they add 14% of all iterations. The sign change in the x term of f4 mirrors f3 horizontally to produce the opposite frond.

### Coordinate mapping uses separate x_scale and y_scale
The mapping from fern space to terminal cell uses:
```
col = (int)((x - x_min) * x_scale + 0.5f)
row = (int)((y_max - y) * y_scale + 0.5f)    ← note: y_max - y flips vertical axis
```
y must be flipped because terminal rows increase downward but fern y increases upward.

## State Machines

```
DRAWING ──── iters_done >= TOTAL_ITERS ────► HOLDING
    ▲                                              │
    └──────── done_ticks >= DONE_PAUSE ────────────┘
```
Identical to sierpinski.c.

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `N_PER_TICK` | 400 | Higher = fern fills faster |
| `TOTAL_ITERS` | 80000 | More = denser leaf texture (diminishing returns above ~50k) |
| `DONE_PAUSE_TICKS` | 90 | ~3 s hold |
| `N_FERN_COLORS` | 5 | Number of height color bands (stem→tip gradient) |
| f2 probability | 85% | Lower = sparser main body; higher = less stem/fronds visible |

## Open Questions for Pass 3

- Are x_scale and y_scale computed once at resize time and stored, or recomputed per iteration?
- Does the code have a warm-up step to discard the first few iterations before plotting?
- How are the N_FERN_COLORS bands defined — equal y-height ranges or perceptually tuned thresholds?
- What happens if a computed (col, row) falls outside the grid bounds — is it clamped, skipped, or impossible given the IFS attractor bounds?

## From the Source

**Algorithm:** IFS chaos game — single-point iteration. Unlike barnsley.c which uses a density grid, fern.c plots each individual point as it is generated, so you can watch the attractor emerge point-by-point from noise.

**Math:** The four affine maps have a combined contractivity ≤ 0.85. Collage theorem: the IFS attractor is the unique compact set K satisfying `K = T₁(K) ∪ T₂(K) ∪ T₃(K) ∪ T₄(K)`. Probability assignment: `p_i ∝ |det(A_i)|` ensures the density of plotted points is proportional to the "area" each map covers. T₁ (stem): `det = 0 × 0.16 − 0 × 0 = 0`, so p₁=0.01 (flat map). T₂ (leaflets): `det ≈ 0.85 × 0.85 − 0.04² ≈ 0.72`, so p₂=0.85.

**Performance:** Color by height (y value in IFS space) maps naturally to botanical structure: dark roots, bright leaf tips. N_PER_TICK=400, TOTAL_ITERS=80,000. IFS coordinate range `x ∈ [−2.5, 2.8]`, `y ∈ [0.0, 10.0]` scaled to fill the terminal with aspect correction for CELL_W/CELL_H ratio.

---

# Pass 2 — fern: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | N_PER_TICK, TOTAL_ITERS, DONE_PAUSE_TICKS, N_FERN_COLORS, grid limits |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 5 green height bands, `color_init()` |
| §4 grid | 2-D cell array, `grid_plot()`, `grid_draw()`, `grid_reset()` |
| §5 ifs | (x, y) state, 4 transforms + probability table, `ifs_step()` |
| §6 scene | Owns grid + IFS, `scene_tick()`, iteration/done counters |
| §7 screen | ncurses init/draw/present/resize, HUD |
| §8 app | Signal handlers, main loop, key handling |

---

## Data Flow Diagram

```
scene_tick():
  for i in 0..N_PER_TICK-1:
    r = rand() % 100
    if r < 1:  (x, y) = f1(x, y)   ← stem
    elif r < 86: (x, y) = f2(x, y)  ← main body
    elif r < 93: (x, y) = f3(x, y)  ← left frond
    else:        (x, y) = f4(x, y)  ← right frond

    col = (int)((x - X_MIN) * x_scale + 0.5)
    row = (int)((Y_MAX - y) * y_scale + 0.5)
    if in bounds:
      grid.cells[row][col] = height_color(y)
    iters_done++

  completion check → done_ticks → reset
```

---

## Function Breakdown

### height_color(y) → color_id
Purpose: map IFS y coordinate to one of N_FERN_COLORS color bands.
Steps:
1. t = y / Y_MAX    (0=base, 1=tip)
2. return band index: 5 equal divisions from dark-green (0) to bright-lemon (4).

### ifs_step(x, y, which) → (x', y')
Purpose: apply one Barnsley transform.
Steps: matrix multiply `[a b; c d] · [x; y] + [e; f]` using the coefficients for `which` ∈ {0,1,2,3}.

---

## Pseudocode — Core Loop

```
setup:
  grid_reset()
  x = 0; y = 0         ← start at fern base
  compute x_scale = (cols - 1) / (X_MAX - X_MIN)
  compute y_scale = (rows - 1) / Y_MAX / ASPECT_R

main loop:
  1. resize → recompute scales, grid_reset

  2. dt, cap 100ms

  3. physics ticks:
     while sim_accum >= tick_ns:
       if not paused: scene_tick()

  4. frame cap sleep

  5. draw:
     erase()
     scene_draw()    ← non-zero cells: '.' with height color
     HUD: fps, pct%, speed
     wnoutrefresh + doupdate

  6. input:
     q/ESC → quit
     r     → reset
     ] [   → sim_fps
     p/spc → pause
```

---

## Interactions Between Modules

```
App
 └── scene_tick → ifs_step (×400) → grid_plot (height_color)
               → completion → reset

Grid
 └── cells[][] — height color per cell; read in scene_draw
```
