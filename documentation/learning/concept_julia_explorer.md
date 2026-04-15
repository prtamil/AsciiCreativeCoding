# Pass 1 — julia_explorer.c: Interactive Julia Set Explorer

## Core Idea

The Julia set for a complex parameter `c` is the boundary of the set of points `z₀` whose orbit under `z → z² + c` remains bounded. The **Mandelbrot set** is the map of which values of `c` itself produce bounded orbits (starting from z₀ = 0). Every point *inside* the Mandelbrot set corresponds to a **connected** Julia set; every point *outside* corresponds to a **dust** (totally disconnected Cantor set).

This program shows both simultaneously: the Mandelbrot map on the left lets you navigate the parameter space, and the Julia set on the right instantly redraws for the selected `c`.

## The Mental Model

### The split-screen relationship

The left panel is a map of c-space. Moving the crosshair to a point c changes which Julia set is displayed on the right. Points deep inside the Mandelbrot set (solid black) produce Julia sets that look like full disks. Points near the boundary produce intricate, filigree Julia sets. Points outside the Mandelbrot set produce disconnected dust clouds.

This is the fundamental duality: **the Mandelbrot set is the index of Julia sets**.

### Why the Mandelbrot panel is precomputed

The Mandelbrot set never changes between keystrokes — only c changes, and c is shown by the crosshair position, not by recomputing the set. So `mandelbrot_draw()` writes iteration counts into `g_mbuf[80][150]` at startup and on resize. Every frame just reads this buffer and redraws it with the current theme and crosshair overlay.

The Julia panel cannot be precomputed: it changes every frame when auto-wander is active or when the user moves `c`. Every pixel must be computed fresh: for each `(col, row)` in the right panel, set `z₀ = x + i·y` (scaled to the Julia view), then iterate `z → z²+c` up to `MAX_ITER = 128` times.

### Escape-time iteration

```c
while (zr*zr + zi*zi < 4.0f && iter < MAX_ITER) {
    float tmp = zr*zr - zi*zi + cr;
    zi = 2.0f * zr * zi + ci;
    zr = tmp;
    iter++;
}
level = (iter == MAX_ITER) ? 0 : (iter % (N_LEVELS-1)) + 1;
```

`|z|² < 4` is the escape condition (`|z| > 2` guarantees divergence). `level = 0` means the point is interior (bounded orbit) — drawn as background. Levels 1–7 are colored by iteration count modulo 7, creating smooth bands.

### Terminal aspect ratio correction

Terminal cells are ≈2× taller than wide (ASPECT_R = 2.0). A pixel at `(col, row)` maps to complex coordinates:

```
x = re_min + col / g_jw * (re_max - re_min)
y = im_half - (row / (g_jh-1) * 2 * im_half) * (1/ASPECT_R)
```

The `* (1/ASPECT_R)` compresses the imaginary axis so that the Julia set appears circular rather than vertically stretched.

The Mandelbrot panel uses the same correction:
```c
im = im_half - (row / (g_rows-1)) * 2*M_IM_HALF / ASPECT_R
```

### Auto-wander

```c
cr = WANDER_R * cosf(g_wander_t);
ci = WANDER_R * WANDER_YSHRK * sinf(g_wander_t);
g_wander_t += WANDER_SPEED;   /* 0.006 rad/frame → ≈35 s orbit */
```

This traces an ellipse of radius 0.72 in c-space, compressed vertically by 0.65. The ellipse passes through the most interesting region near the Mandelbrot boundary at radius ~0.7. The squish keeps c off the real axis (which would produce symmetric "rabbit ear" Julia sets exclusively) and exposes more varied morphologies.

### Dynamic panel widths

```c
g_mw = g_cols / 2;    /* Mandelbrot panel: left half */
g_jw = g_cols - g_mw - 1;  /* Julia panel: right half minus divider */
```

Both panels resize to fill the terminal. The divider column is left blank, creating a visual gap.

### Crosshair rendering

After the Mandelbrot buffer is redrawn, the crosshair is drawn on top:
```c
/* vertical bar */
for (int r = 0; r < g_rows-HUD_ROWS; r++)
    mvaddch(r, g_cross_col, '|');
/* horizontal bar */
for (int c = 0; c < g_mw; c++)
    mvaddch(g_cross_row, c, '-');
/* intersection */
mvaddch(g_cross_row, g_cross_col, '+');
```

The crosshair position `(g_cross_col, g_cross_row)` tracks the current `c` value through the inverse of the Mandelbrot coordinate mapping.

## Data Structures

```c
uint8_t g_mbuf[GRID_ROWS_MAX][GRID_COLS_MAX/2]; /* precomputed Mandelbrot levels */
float   g_cr, g_ci;         /* current c = cr + ci·i */
float   g_j_im_half;        /* Julia view zoom (imaginary half-range) */
float   g_wander_t;         /* auto-wander angle (radians) */
int     g_cross_col, g_cross_row;  /* crosshair position in Mandelbrot panel */
int     g_mw, g_jw;         /* Mandelbrot and Julia panel widths */
```

## The Main Loop

```
startup:
    mandelbrot_compute()   ← fills g_mbuf, called once (or on resize)

per frame:
    if wander: cr = WANDER_R*cos(t), ci = WANDER_R*WANDER_YSHRK*sin(t); t+=0.006
    update g_cross_col, g_cross_row from (cr, ci) → Mandelbrot pixel coords
    erase()
    mandelbrot_draw()      ← reads g_mbuf → mvaddch with theme colors
    draw_crosshair()       ← overlays '|', '-', '+' at crosshair position
    julia_draw()           ← for each (col, row) in right panel: iterate z²+c
    draw_divider()         ← vertical bar between panels
    draw_hud()
    doupdate()
```

## Non-Obvious Decisions

### MAX_ITER = 128 rather than 256

128 iterations is enough to reveal fine structure in the Julia set while keeping each frame under budget. At 30 fps with ~150×(rows-2) Julia pixels per frame, 128 iterations × 24,000 pixels ≈ 3M iterations/frame — comfortably fast. Higher iterations would show more detail near the boundary but at proportional cost.

### Modulo coloring `iter % (N_LEVELS-1)`

Using `iter % 7` rather than a continuous gradient produces visible discrete bands. Bands cycle through the 7 theme colors, creating concentric "rings" of color that reveal the topology of the Julia set. Each ring boundary is an equipotential of the Green's function of the filled Julia set.

### The FINE_STEP / COARSE_STEP distinction

Arrow keys move c by `FINE_STEP = 0.015` — small enough to explore detail near a chosen feature. HJKL keys move by `COARSE_STEP = 0.08` — fast enough to jump between major regions of the Mandelbrot map. Both are additive so holding a key produces smooth traversal.

### Julia zoom (z/Z keys) changes only `g_j_im_half`

Zooming the Julia view rescales the imaginary half-range `g_j_im_half` by `J_ZOOM_IN = 0.85` per keypress. The real half-range follows via aspect correction. The Mandelbrot panel is unaffected — you always see the full Mandelbrot map, and zoom only affects the Julia display.

## From the Source

**Algorithm:** Split-screen interactive fractal explorer. Left panel (Mandelbrot): computed once at start/resize and cached — O(W/2 × H × MAX_ITER) up-front. Right panel (Julia): recomputed every frame from the current c position — O(W/2 × H × MAX_ITER) per frame. This trade-off makes Mandelbrot navigation smooth (no per-frame cost) while Julia responds instantly to c changes.

**Math:** Auto-wander: c orbits an ellipse in the complex plane (`WANDER_R=0.72`, `WANDER_YSHRK=0.65`, `WANDER_SPEED=0.006 rad/frame ≈ 35 s per orbit at 30 fps`), tracing a path through many different Julia topologies automatically. The squish keeps c off the real axis, exposing more varied morphologies.

**Performance:** Julia panel rendered per-frame — full W/2 × H pixel loop. At 30fps and 80×40 Julia panel: 3200 pixels × 128 max-iter = 409,600 iterations/frame — feasible in C without parallelism.

## Open Questions for Pass 3

- At what resolution does the Julia set become indistinguishable from noise near the boundary? This is related to the fact that the boundary of the Mandelbrot set is nowhere differentiable.
- Implement **smooth coloring** (fractional escape count via `iter + 1 - log(log|z|)/log(2)`) to eliminate the visible banding in the Julia panel.
- What does the Julia set look like for `c` on the real axis at various positions? The bulb boundaries (`c = -2, -0.75, 0.25`) should produce cusp-shaped Julia sets.
- Can you add a third panel showing the **parameter space derivative** `dJ/dc` at the current c — this would reveal how fast the Julia morphology changes with c movement.
