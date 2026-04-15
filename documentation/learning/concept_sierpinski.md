# Pass 1 — sierpinski.c: Sierpinski triangle via the chaos game (IFS)

## Core Idea

Three fixed vertices form an equilateral triangle. At each iteration, one of the three vertices is chosen at random, and the current point moves halfway toward it. The orbit of this deceptively simple rule traces the Sierpinski triangle — a fractal with self-similar triangular holes at every scale. Color is assigned based on which vertex was chosen last, giving the three main sub-triangles distinct hues (cyan, yellow, magenta) that repeat self-similarly at every level. N_PER_TICK=500 iterations are computed per tick and plotted as single terminal cells. After TOTAL_ITERS=50000 the completed triangle holds for DONE_PAUSE_TICKS=90 ticks, then resets.

## The Mental Model

Imagine you are playing a game with a triangle. You start at any point. You roll a three-sided die, and whichever corner comes up you jump halfway toward that corner. You mark your current position. You repeat. After a few thousand marks, the Sierpinski triangle emerges — the middle of the triangle is always empty (because no sequence of halfway-moves can reach the exact center from any corner), and the same void pattern repeats at every scale.

The coloring reveals the fractal structure: any point in the lower-left sub-triangle was last moved toward the lower-left vertex, so it is cyan. Any point in the lower-right is yellow. Any point in the top sub-triangle is magenta. Because the same rule applies recursively, sub-sub-triangles are also divided into three colors — you see a fractal tricolor texture.

The "chaos game" is so named because the process looks random but the output is a perfect deterministic fractal. The fractal is the attractor of the IFS — the orbit converges to it regardless of starting point (after a few warm-up iterations).

## Data Structures

### Grid (§4)
```
uint8_t cells[GRID_ROWS_MAX][GRID_COLS_MAX]   — color ID per drawn cell (0 = empty)
int     rows, cols
int     iters_done       — counter toward TOTAL_ITERS
int     done_ticks       — hold-period counter after completion
```
Cells are set once; no physics, no clearing within a cycle. The full fractal accumulates from the random walk.

### IFS state (§5)
```
float   x, y            — current point in IFS coordinate space
int     last_vertex     — 0, 1, or 2 — which vertex was chosen last tick
```
The IFS point wanders over the attractor; its coordinate is mapped to a terminal cell at each step.

### Vertex positions
```
v[0] = bottom-left   corner of the triangle (in IFS space)
v[1] = bottom-right  corner
v[2] = top-center    corner
```
Mapped to pixel space using terminal dimensions and ASPECT_R to keep the triangle equilateral-looking.

### Scene (§6)
```
Grid    grid
IFS state
bool    paused
```

## The Main Loop

1. Resize: recompute vertex positions for new dimensions, reset grid.
2. Measure dt, cap at 100ms.
3. Ticks: compute N_PER_TICK=500 IFS iterations per tick; for each, pick random vertex, move halfway, plot cell with vertex color.
4. After TOTAL_ITERS: increment done_ticks until DONE_PAUSE_TICKS, then reset.
5. Draw: for each non-zero cell, write `.` with vertex color pair.
6. HUD: fps, progress percentage, speed.
7. Input: r=reset, p=pause, [/]=speed.

## Non-Obvious Decisions

### Why 500 iterations per tick instead of one?
One iteration per tick at 30fps would take 50000/30 ≈ 1667 seconds (28 minutes) to complete. 500 per tick completes in 50000/(500×30) ≈ 3.3 seconds — fast enough to watch the triangle emerge in real time without it being instant.

### Warm-up iterations
The first few iterations of the chaos game can land anywhere (the initial point is arbitrary). In practice the first ~20 steps are discarded (or the IFS coordinate is simply initialized to a vertex) to avoid plotting points outside the attractor. Most implementations start at a vertex for correctness.

### ASPECT_R correction for equilateral appearance
Terminal cells are roughly 2× taller than wide. The top vertex must be placed at `row_center - width/2 / ASPECT_R` (not `- width/2`) for the triangle to look equilateral rather than squashed into a flat isoceles shape.

### Character `.` for all points
Every fractal cell uses the same character `.` — the visual richness comes entirely from the three-color tricolor pattern, not from character variation. This is unlike snowflake (topology chars) or coral (fixed `#`).

### IFS coordinate space vs terminal space
The IFS computes in abstract [0,1] × [0,1] space (or similar). Mapping to terminal (col, row) requires a linear transform that accounts for the terminal aspect ratio and desired triangle size. The transform is recomputed on resize.

## State Machines

```
DRAWING ──── iters_done >= TOTAL_ITERS ────► HOLDING (done_ticks++)
    ▲                                              │
    └──────── done_ticks >= DONE_PAUSE ────────────┘ (reset)
```

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `N_PER_TICK` | 500 | Higher = triangle fills faster; lower = slower emergence |
| `TOTAL_ITERS` | 50000 | More = denser/finer detail; fewer = sparser |
| `DONE_PAUSE_TICKS` | 90 | ~3 s at 30fps; hold time before reset |

## Themes (t key)

10 themes cycle with `t`/`T`. Each theme assigns distinct colors to the 3 vertices (V1, V2, V3) plus HUD, chosen so the three sub-triangles are always visually distinct:

| Theme | V1 | V2 | V3 | Character |
|---|---|---|---|---|
| Electric | 87 (cyan) | 226 (yellow) | 207 (magenta) | High contrast neon |
| Matrix | 46 (bright green) | 118 (lime) | 231 (white) | Monochrome green |
| Nova | 51 (cyan) | 39 (sky blue) | 231 (white) | Cold blue tones |
| Poison | 82 (lime) | 190 (yellow-green) | 154 (medium green) | Toxic green |
| Ocean | 45 (bright cyan) | 33 (blue) | 38 (teal) | Deep water |
| Fire | 196 (red) | 208 (orange) | 226 (yellow) | Flame gradient |
| Gold | 214 (orange) | 220 (gold) | 231 (white) | Metallic warm |
| Ice | 159 (light cyan) | 123 (light blue) | 231 (white) | Frost |
| Nebula | 93 (purple) | 201 (magenta) | 87 (cyan) | Space |
| Lava | 196 (red) | 214 (orange) | 208 (dark orange) | Volcanic |

The HUD color matches V1 to keep the status bar in the same palette family.

## Open Questions for Pass 3

- Does the code warm up by initializing x,y to one of the vertices, or does it run a fixed number of discarded iterations first?
- How are the three vertex positions computed — fractions of (cols, rows) or using ASPECT_R explicitly?
- Is the `.` character hardcoded or is there a character table like in julia.c?
- If two IFS iterations land on the same cell (different vertices), which color wins — first or last?

## From the Source

**Algorithm:** Chaos game IFS with 3 attractors (vertices of a triangle). At each step: pick a random vertex, move halfway from the current point toward it, plot the result. After a transient, the orbit traces out the Sierpinski triangle exactly.

**Math:** The Sierpinski triangle is a self-similar set with Hausdorff dimension D = log(3)/log(2) ≈ 1.585. It has zero 2D area (Lebesgue measure 0) but non-zero fractal measure. The IFS `{T₁, T₂, T₃}` where `T_i(x) = (x + v_i)/2` are contractions with ratio 1/2, combined contractivity < 1. Alternative binary view: a point is in the Sierpinski triangle iff in the base-2 representation of its coordinates, no column has both bits = 1 simultaneously (Pascal's triangle mod 2 connection: cell (n,k) is in if C(n,k) is odd).

**Performance:** N_PER_TICK=500 IFS iterations per tick, TOTAL_ITERS=50,000 before reset. Vertex positions define an equilateral triangle: V1=(0,0), V2=(1,0), V3=(0.5, √3/2 ≈ 0.866). ASPECT_R=2.0 ensures the triangle looks equilateral rather than squashed.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_grid[GRID_ROWS_MAX][GRID_COLS_MAX]` | `uint8_t[]` | ~24 KB | color index per cell (0=empty, 1–3 by last chosen vertex) |
| `g_cx`, `g_cy` | `float` | scalar | current IFS orbit point in normalized triangle space |
| `g_last_v` | `int` | scalar | index of last chosen vertex (0–2); drives coloring |
| `GRID_ROWS_MAX`, `GRID_COLS_MAX` | constants | N/A | maximum grid size (80 × 300) |
| `N_PER_TICK` | `int` constant | N/A | IFS iterations per simulation tick (500) |
| `TOTAL_ITERS` | `int` constant | N/A | total iterations before auto-reset (50,000) |

# Pass 2 — sierpinski: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | N_PER_TICK, TOTAL_ITERS, DONE_PAUSE_TICKS, grid limits |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 3 vertex colors (cyan/yellow/magenta), `color_init()` |
| §4 grid | 2-D cell array, `grid_plot()`, `grid_draw()`, `grid_reset()` |
| §5 ifs | IFS state (x, y, last_vertex), `ifs_step()`, vertex array |
| §6 scene | Owns grid + IFS, `scene_tick()`, iteration counter, done_ticks |
| §7 screen | ncurses init/draw/present/resize, HUD |
| §8 app | Signal handlers, main loop, key handling |

---

## Data Flow Diagram

```
scene_tick():
  for i in 0..N_PER_TICK-1:
    v = rand() % 3                    ← choose random vertex
    x = (x + vertex[v].x) / 2.0f    ← move halfway
    y = (y + vertex[v].y) / 2.0f
    last_vertex = v
    (col, row) = to_cell(x, y)       ← IFS → terminal mapping
    grid.cells[row][col] = v + 1     ← store vertex color (1-indexed)
    iters_done++

  if iters_done >= TOTAL_ITERS:
    done_ticks++
    if done_ticks >= DONE_PAUSE_TICKS:
      grid_reset(); ifs_init(); iters_done = 0; done_ticks = 0

scene_draw():
  for each (row, col) where cells[row][col] != 0:
    c = cells[row][col]              ← 1=cyan, 2=yellow, 3=magenta
    mvwaddch('.') with COLOR_PAIR(c)
```

---

## Function Breakdown

### ifs_step()
Purpose: one chaos game iteration.
Steps:
1. Pick vertex v = rand() % 3.
2. x = (x + vx[v]) * 0.5f
3. y = (y + vy[v]) * 0.5f
4. Return (col, row) = to_cell(x, y) and color = v.

### to_cell(x, y) → (col, row)
Purpose: map IFS float coords to terminal cell.
Steps:
1. col = (int)(x * (cols - 1) + 0.5f)
2. row = (int)(y * (rows - 1) / ASPECT_R + 0.5f)
(Exact formula depends on triangle positioning.)

---

## Pseudocode — Core Loop

```
setup:
  grid_reset()
  ifs_init()       ← x = vx[0], y = vy[0] (start at bottom-left vertex)
  iters_done = 0

main loop:
  1. resize → recompute vertices, grid_reset, ifs_init

  2. dt, cap 100ms

  3. physics ticks:
     while sim_accum >= tick_ns:
       if not paused: scene_tick()

  4. frame cap sleep

  5. draw:
     erase()
     scene_draw()
     pct = iters_done * 100 / TOTAL_ITERS
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
 └── scene_tick → ifs_step (×500) → grid_plot
               → completion check → done_ticks → grid_reset

Grid
 ├── cells[][] — color per cell; set at plot time, cleared on reset
 └── all reads are in scene_draw (iterate all cells, check != 0)
```
