# Pass 1 — mandelbrot.c: Mandelbrot set with random-pixel animated reveal

## Core Idea

Mandelbrot.c is structurally identical to julia.c with one key difference in the iteration formula. For each pixel mapped to complex c = re + im·i, the iteration starts from z₀ = 0 (not z₀ = c as in Julia) and runs `z → z² + c`. If the orbit escapes, the escape speed determines the color; if it never escapes the cell is inside the set. Six preset zoom windows cycle through different regions of the Mandelbrot set. The same Fisher-Yates random-order reveal as julia.c is used. The color palette is electric neon (magenta → light purple → cyan → lime → yellow) rather than fire.

## The Mental Model

The Mandelbrot set is a map of Julia sets. For any point c in the complex plane, the Julia set of c (the boundary of orbits that stay bounded) is connected if and only if c is inside the Mandelbrot set. So the Mandelbrot set is the set of c values for which starting at z=0 the orbit stays bounded.

The six presets explore different regions:
- **Full set**: the classic overview — main cardioid, period-2 bulb, antenna
- **Seahorse valley**: the spiral filaments along the neck between main cardioid and bulb
- **North antenna**: delicate tree-branch filaments above the main body
- **Deep spiral**: a zoomed-in spiral near the boundary
- **Mini Mandelbrot**: a tiny complete copy of the whole set deep in the boundary
- **Antenna tip**: the extreme fine filaments at the very top of the antenna

Each zoom window is defined by (re_center, re_half, im_half) — the center and half-extents of the viewport in the complex plane. Zooming into a region fills the same terminal with far less complex-plane area, revealing finer detail.

Unlike julia.c, all six Mandelbrot presets show the same mathematical object at different scales. The electric neon palette (magenta inside, yellow outermost) reads as "chemical" rather than "thermal," distinguishing the two programs visually.

## Data Structures

### Grid (§4)
```
uint8_t  color[GRID_ROWS_MAX][GRID_COLS_MAX]  — color band per cell
int      rows, cols
int      pixels_done
int      done_ticks
int      preset
```

### Compute (§5)
```
int  *order               — Fisher-Yates shuffle array (dynamically allocated)
int   order_pos
```

Preset table:
```
struct { float re_center, re_half, im_center, im_half; const char *name; } k_presets[6]
```
No Julia parameter c — the Mandelbrot iteration always uses z₀=0 and c from the pixel position.

## The Main Loop

Identical to julia.c. The only algorithmic difference is in the `mandelbrot_escape()` function:
- Julia: `z₀ = (re, im)` (use pixel as starting point), c = fixed preset constant
- Mandelbrot: `z₀ = (0, 0)`, c = (re, im) (pixel is the parameter, not the starting point)

All timing, shuffle logic, preset cycling, draw, HUD, and input are identical in structure.

## Non-Obvious Decisions

### z₀ = 0 vs z₀ = c
This is the fundamental difference between Julia and Mandelbrot:
- Julia asks: "for this starting point z, does the orbit under f(z)=z²+c (fixed c) escape?"
- Mandelbrot asks: "for this parameter c, does the orbit starting at z=0 escape?"

Both use the same function f(z) = z² + c, but they vary different things.

### MAX_ITER = 256 vs julia.c's 128
The Mandelbrot set has more intricate boundary structure, especially in zoomed-in regions like seahorse valley. Higher max_iter reveals more detail near the boundary at the cost of more computation per pixel.

### Viewport half-extents for zoom
Each preset stores `re_half` and `im_half` (half the width and height of the complex-plane viewport). For the full set `re_half ≈ 2.0`; for the deep spiral it might be `re_half ≈ 0.001`. The same pixel-to-complex mapping code `re = re_center + (col - cols/2) * re_step` works for all presets.

### Electric neon palette
Five levels: magenta (inside set), light purple (slow escape), cyan (medium), lime green (fast), yellow (fastest). This is the complement of julia.c's fire palette. The cold/chemical colors distinguish the two programs and suggest the Mandelbrot set as a map/diagram rather than a physical phenomenon.

### Cardioid / bulb skip optimization
Many pixels inside the main cardioid or period-2 bulb would iterate all 256 steps without escaping. A mathematical pre-check can skip these:
```c
float q = (cr - 0.25f)*(cr - 0.25f) + ci*ci;
if (q*(q + cr - 0.25f) < 0.25f*ci*ci) return INSIDE;        /* cardioid */
if ((cr + 1.0f)*(cr + 1.0f) + ci*ci < 0.0625f) return INSIDE; /* period-2 bulb */
```
This is a significant speedup for full-set views where the large interior dominates.

## State Machines

Identical to julia.c:
```
COMPUTING → HOLDING → (next preset, reset)
```

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `PIXELS_PER_TICK` | 60 | Speed of fill animation |
| `MAX_ITER` | 256 | Detail at boundaries; more CPU per pixel |
| `DONE_PAUSE_TICKS` | 90 | ~3 s hold |
| `N_PRESETS` | 6 | Six zoom windows |
| `N_COLORS` | 5 | Color bands (not counting "inside") |

## Themes (t key)

10 themes cycle with `t`/`T`. Each theme remaps 5 content color pairs (COL_INSIDE, COL_C2..C5) and 1 HUD pair using `theme_apply(int t)`. Themes are chosen to highlight different aspects of the set — cold themes read the boundary as geological, warm themes read it as thermal, dark themes emphasize depth:

| Theme | Character | Inside color | Outer ring |
|---|---|---|---|
| Electric | High-contrast neon | Purple | Yellow/cyan |
| Matrix | Green gradient | Dark green | Bright green |
| Nova | Blue-white cold | Dark blue | White |
| Poison | Green-yellow toxic | Olive | Lime |
| Ocean | Blue-cyan deep | Navy | Cyan |
| Fire | Red-orange thermal | Dark red | Yellow |
| Gold | Yellow-orange warm | Brown | White |
| Ice | Cyan-white cold | Steel | White |
| Nebula | Purple-magenta | Dark purple | Cyan |
| Lava | Red-orange volcanic | Dark red | Orange |

The Theme struct holds `c[5]` (256-color) and `c8[5]` (8-color fallback) for the 5 content pairs, plus `hud` and `hud8` for the status bar. `theme_apply()` calls `init_pair()` for all 6 pairs and takes effect on the next rendered frame.

## Open Questions for Pass 3

- Is the cardioid/bulb optimization implemented in mandelbrot.c? (It is used in buddhabrot.c.)
- Do the six zoom windows have fixed pixel-space boundaries or do they adapt to the terminal size?
- How are the color band thresholds chosen — fixed iteration counts or relative to MAX_ITER?
- When zoomed deep into a region (mini Mandelbrot preset), does the floating-point precision of `float` cause visible artifacts?

## From the Source

**Algorithm:** Pre-shuffled random pixel order via Fisher-Yates permutation — reveals the fractal uniformly across the screen rather than scan-line by scan-line. Cost per pixel: O(MAX_ITER) worst case, O(1) for fast-escape pixels far from the boundary.

**Math:** The escape condition `|z| > 2` is sufficient because if `|z| > 2` then the orbit diverges to ∞. Equivalent test: `Re(z)² + Im(z)² > 4`. The boundary of M has Hausdorff dimension = 2 (Brooks & Matelski, Mandelbrot 1980). M is connected (Douady & Hubbard, 1982). Period-1 bulb (main cardioid): converges to fixed point; period-2 bulb (left circle): oscillates between 2 values.

**Performance:** PIXELS_PER_TICK=60 pixels computed per simulation tick. The pre-shuffled index array costs O(W×H) memory but makes each tick O(PIXELS_PER_TICK) — deterministic cost.

**References:** Brooks & Matelski (Hausdorff dimension = 2, 1978 manuscript); Mandelbrot (1980); Douady & Hubbard (connectedness proof, 1982).

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_grid[GRID_ROWS_MAX][GRID_COLS_MAX]` | `uint8_t[]` | ~24 KB | computed escape-time level per cell (0=uncomputed, 1–5=gradient) |
| `g_order[GRID_ROWS_MAX * GRID_COLS_MAX]` | `int[]` | ~96 KB | shuffled pixel index permutation for random reveal order |
| `g_draw_pos` | `int` | scalar | current position in g_order; advances by PIXELS_PER_TICK per tick |
| `PIXELS_PER_TICK` | `int` constant | N/A | pixels computed per simulation tick (60) |
| `MAX_ITER` | `int` constant | N/A | Mandelbrot iteration cap (256) |
| `ASPECT_R` | `float` constant | N/A | cell height/width ratio (2.0) for correct complex-plane proportions |

# Pass 2 — mandelbrot: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | PIXELS_PER_TICK=60, MAX_ITER=256, DONE_PAUSE_TICKS=90, N_PRESETS=6 |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 5-level neon gradient (magenta→yellow), `color_init()` |
| §4 grid | color[][] array, tracking, `grid_draw()`, `grid_reset()` |
| §5 compute | Fisher-Yates shuffle, preset table, `mandelbrot_escape()` |
| §6 scene | Owns grid + shuffle, `scene_tick()`, `scene_draw()` |
| §7 screen | ncurses init/draw/present/resize, HUD |
| §8 app | Signal handlers, main loop, key handling |

---

## Data Flow Diagram

```
mandelbrot_escape(re, im):
  zr = 0; zi = 0             ← z₀ = 0  (not (re, im) like julia!)
  for iter = 0..MAX_ITER-1:
    zr2 = zr*zr - zi*zi + re  ← c = (re, im) from pixel
    zi2 = 2*zr*zi + im
    zr = zr2; zi = zi2
    if zr*zr + zi*zi > 4: return escape_band(iter)
  return COL_INSIDE           ← magenta

escape_band(iter):
  — thresholds scaled to MAX_ITER=256:
  if iter < 8:   return 0          (black background)
  if iter < 32:  return COL_YELLOW (fast escape = outermost)
  if iter < 96:  return COL_LIME
  if iter < 192: return COL_CYAN
  else:          return COL_PURPLE (slow escape = near boundary)
```
(Note: inside-set is magenta, outermost is yellow — opposite of typical coloring conventions. This is a deliberate artistic choice.)

---

## Function Breakdown

### scene_init(preset)
Purpose: allocate shuffle array, Fisher-Yates shuffle, reset grid.
Steps: identical to julia.c's scene_init; only preset table differs.

### scene_tick()
Purpose: compute PIXELS_PER_TICK pixels from shuffled order.
Steps: identical to julia.c; calls `mandelbrot_escape()` instead of `julia_escape()`.

---

## Pseudocode — Core Loop

```
Identical to julia.c; replace julia_escape with mandelbrot_escape.
```

---

## Differences from julia.c Summary

| Aspect | julia.c | mandelbrot.c |
|---|---|---|
| Iteration start | z₀ = pixel (re, im) | z₀ = (0, 0) |
| Parameter c | fixed preset constant | pixel (re, im) |
| MAX_ITER | 128 | 256 |
| Palette | fire (dark-red → white) | neon (magenta → yellow) |
| Presets | 6 Julia constants | 6 Mandelbrot zoom windows |
| Preset names | rabbit, spiral, dendrite... | full set, seahorse valley... |
