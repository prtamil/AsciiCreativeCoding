# Pass 1 — koch.c: Koch snowflake fractal, animated level-by-level

## Core Idea

The Koch snowflake is built by repeatedly applying one subdivision rule: every straight segment is replaced by four shorter segments forming an outward equilateral bump. The program starts with an equilateral triangle (level 1 = 12 segments after one application). Each level multiplies the segment count by 4. Five levels cycle automatically (12 → 48 → 192 → 768 → 3072 segments), each drawn segment-by-segment using Bresenham's algorithm so you watch the outline appear stroke-by-stroke. After level 5 the cycle restarts at level 1. Color shifts from deep blue to sky blue to white as drawing progresses across the segments (depth-position coloring). An adaptive segs_per_tick ensures each level takes approximately 2 seconds to draw.

## The Mental Model

Picture tracing the outline of a snowflake with a pen. You start with a triangle. Each time you reach a third-of-a-segment you instead draw an outward bump — up, right-up, back down, right-down. At level 5 the bumps are so tiny (1/243rd of the original edge length) that many are sub-pixel, but the overall silhouette is unmistakably the Koch snowflake.

The key insight is that the geometry is stored entirely in floating-point Euclidean coordinates (not terminal cells). Only at draw time are the coordinates converted to (col, row). This keeps the subdivision math clean — no rounding errors accumulate during the recursive subdivision.

The color progression is a gradient across the drawing order: the first segments drawn (the original triangle's base edges) get one color, and the final segments drawn (the fine tips) get another color. As you watch the snowflake being drawn, it starts in deep blue and finishes in white — like a frost forming from the inside out.

## Data Structures

### Segment array (§5)
```
typedef struct { float x0, y0, x1, y1; } Seg;
Seg  segs_a[MAX_SEGS]    — current level's segments
Seg  segs_b[MAX_SEGS]    — scratch buffer for subdivision
int  n_segs              — segment count for current level
```
Two buffers alternated: subdivide from segs_a into segs_b, then swap. No dynamic allocation.

Maximum segments: 3072 at level 5 (3 × 4⁵ / 4 because we start with 3 edges × 4 subdivisions per edge per level). Actually 3 × 4^(level-1) × 4 = 3 × 4^level edges... Let me think: triangle = 3 sides, level 1 = 3 × 4^1 = 12, level 5 = 3 × 4^5 = 3072.

### Scene (§6)
```
int  level           — current level (1–5)
int  seg_draw_index  — which segment is being drawn this tick
int  segs_per_tick   — adaptive: n_segs / 60 + 1
int  done_ticks      — hold counter after level complete
```

### Grid (§4)
```
— minimal or none; Koch draws directly via Bresenham, not through a cell buffer
```

## The Main Loop

1. Resize: recompute triangle vertices and circumradius, re-subdivide to current level.
2. Measure dt.
3. Ticks: draw segs_per_tick segments per tick from the current segment array; use Bresenham to rasterize each segment to terminal cells; color by seg_draw_index position.
4. After all segments drawn: hold HOLD_TICKS=45 (~1.5 s), then advance to next level (subdivide again from current segs or restart from triangle if level was 5).
5. HUD: fps, level number, speed.
6. Input: r=restart level 1, n=next level, p=pause, [/]=speed.

## Non-Obvious Decisions

### Why floating-point segment storage?
Rounding intermediate positions to terminal cells during subdivision would compound errors — each level's bump peak would be slightly off, and by level 4-5 the accumulated error would be visible as segment misalignments. Storing all geometry in float and converting only at the final Bresenham step keeps the snowflake geometrically exact.

### Koch subdivision rule in code
Given segment from P(px, py) to Q(qx, qy):
```c
float dpx = (qx - px) / 3.0f;
float dpy = (qy - py) / 3.0f;
/* A = P + (Q-P)/3 */
ax = px + dpx;  ay = py + dpy;
/* B = P + 2(Q-P)/3 */
bx = px + 2*dpx; by = py + 2*dpy;
/* M = A + R(+60°)(B-A) */
float ex = bx - ax, ey = by - ay;
mx = ax + COS60*ex - SIN60*ey;
my = ay + SIN60*ex + COS60*ey;
/* Four sub-segments: P→A, A→M, M→B, B→Q */
```
The rotation R(+60°) applied to (B−A) places the bump peak M outside the triangle for clockwise-wound edges. For the standard Koch snowflake the initial triangle must be wound clockwise for the bumps to point outward.

### Adaptive segs_per_tick
Level 1 has 12 segments and level 5 has 3072. If segs_per_tick were fixed at 1, level 5 would take 3072 ticks × (1/30 s) = 102 seconds. The adaptive formula `segs_per_tick = max(1, n_segs / 60)` targets ~60 ticks (2 seconds) per level regardless of level. Level 1: segs_per_tick = 1 (no speedup needed). Level 5: segs_per_tick ≈ 51.

### ASPECT_R in coordinate-to-cell conversion
The snowflake is generated in Euclidean coordinates with circumradius = 1 (normalized). When converting to terminal cells, the x-direction uses `col_scale = cols / 2` but the y-direction uses `row_scale = rows / 2 / ASPECT_R` (divided by ASPECT_R ≈ 2). Without this correction the snowflake would appear squashed into an ellipse.

### Color by draw order
`ci = (int)((float)seg_idx / n_segs * (N_KOCH_COLORS - 1))` maps draw position to color index. Segments drawn early (the original triangle edges) get index 0 (deep blue); segments drawn last (fine tips at level 5) get index 4 (white). This gives a radial gradient from the interior out.

## State Machines

```
DRAWING ──── seg_draw_index >= n_segs ────► HOLDING (done_ticks++)
    ▲                                             │
    └──── done_ticks >= HOLD_TICKS ───────────────┘
          level = (level % MAX_LEVEL) + 1
          subdivide → new segs, seg_draw_index = 0
```

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `MAX_LEVEL` | 5 | Higher = more levels (requires larger segment buffer) |
| `HOLD_TICKS` | 45 | ~1.5 s hold after each level completes |
| `N_KOCH_COLORS` | 5 | Color bands from start to end of drawing |
| `segs_per_tick` | n_segs/60+1 | Targets ~2 s per level |
| `COS60, SIN60` | 0.5, 0.866 | Fixed math constants |

## Open Questions for Pass 3

- Are the two segment buffers (segs_a, segs_b) statically allocated or does one dynamic allocation exist at MAX_SEGS?
- What is MAX_SEGS? 3072 (level 5) exactly, or with some headroom?
- Does the code reset to level 1 on resize, or continue from the current level?
- At level 5, many segments are sub-pixel. How does Bresenham handle a segment shorter than one cell — does it produce one cell mark or zero?

## From the Source

**Algorithm:** Iterative edge-replacement (segment subdivision). At each level, every existing segment is replaced by 4 shorter segments: A→P, P→M, M→Q, Q→B where M is the equilateral bump peak. No recursion stack needed: segments are stored in a flat array and a new array is generated each level.

**Math:** After n levels: segment count = `3 × 4ⁿ` (starts with triangle = 3 segments); segment length = `(1/3)ⁿ` of original edge; total perimeter = `(4/3)ⁿ × original → ∞` as n → ∞; area enclosed converges to `(2/5) × area of original triangle`. Fractal dimension: `D = log(4)/log(3) ≈ 1.26` (more than a curve, less than a surface). Precomputed constants: SIN60=0.8660254f, COS60=0.5f.

**Performance:** MAX_SEGS=4096 (level 5 → 3 × 4⁵ = 3072; 4096 gives safe margin). Segments drawn one per frame as an animated "drawing" effect. The bump peak M is computed by rotating the direction vector by ±60° (equilateral triangle geometry). ASPECT_R corrects for non-square terminal cells.

---

# Pass 2 — koch: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | MAX_LEVEL=5, HOLD_TICKS=45, N_KOCH_COLORS=5, ASPECT_R |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 5-band gradient cyan→teal→lime→yellow→white, `color_init()` |
| §4 grid | Direct cell writes via Bresenham (minimal struct) |
| §5 koch | Seg struct, seg arrays, `koch_subdivide()`, `bresenham_draw()` |
| §6 scene | Level management, draw cursor, hold counter, `scene_tick()` |
| §7 screen | ncurses init/draw/present/resize, HUD |
| §8 app | Signal handlers, main loop, key handling |

---

## Data Flow Diagram

```
scene_init(level):
  build_triangle()       ← 3 edges, circumradius=1, center=(0,0)
  for l in 1..level:
    koch_subdivide(segs_a → segs_b, n_segs → n_segs*4)
    swap segs_a, segs_b
  segs_per_tick = max(1, n_segs / 60)
  seg_draw_index = 0

scene_tick():
  for k in 0..segs_per_tick-1:
    if seg_draw_index >= n_segs: break
    seg = segs_a[seg_draw_index]
    ci = seg_draw_index * (N_KOCH_COLORS-1) / n_segs   ← color by position
    (col0, row0) = to_cell(seg.x0, seg.y0)
    (col1, row1) = to_cell(seg.x1, seg.y1)
    bresenham(col0, row0, col1, row1, color[ci])
    seg_draw_index++

  if seg_draw_index >= n_segs:
    done_ticks++
    if done_ticks >= HOLD_TICKS:
      level = (level % MAX_LEVEL) + 1
      scene_init(level)   ← re-subdivide from triangle

to_cell(x, y):
  col = (int)((x + 1.0) * cols/2)
  row = (int)((1.0 - y) * rows/2 / ASPECT_R)
```

---

## Function Breakdown

### koch_subdivide(in[], nin → out[], nout)
Purpose: one level of Koch subdivision.
Steps:
For each segment (px,py)→(qx,qy):
1. Compute A = P + (Q-P)/3
2. Compute B = P + 2(Q-P)/3
3. Compute M = A + R(+60°)·(B-A)
4. Output 4 segments: P→A, A→M, M→B, B→Q

### bresenham(col0, row0, col1, row1, color_id)
Purpose: rasterize a line segment to terminal cells.
Steps: standard integer Bresenham; for each cell on the line, call `mvwaddch` with the Koch character and color pair.

---

## Pseudocode — Core Loop

```
setup:
  level = 1
  scene_init(1)

main loop:
  1. resize → scene_init(level) with new center/scale

  2. dt, cap 100ms

  3. physics ticks:
     while sim_accum >= tick_ns:
       if not paused:
         if seg_draw_index < n_segs: scene_tick()
         else: done_ticks++; if done_ticks >= HOLD_TICKS: next level

  4. frame cap sleep

  5. draw:
     (Koch draws directly during scene_tick — no separate draw pass)
     erase() before each new level
     HUD: fps, level, speed
     wnoutrefresh + doupdate

  6. input:
     q/ESC → quit
     r     → level=1, scene_init(1)
     n     → next level immediately
     ] [   → sim_fps
     p/spc → pause
```

---

## Interactions Between Modules

```
App
 └── scene_tick → bresenham → mvwaddch  (direct draw during tick)
               → completion → done_ticks → scene_init(next level)

Segment buffers
 ├── segs_a[] — current level's geometry (source for drawing)
 └── segs_b[] — scratch for subdivision; becomes segs_a at next level
```
