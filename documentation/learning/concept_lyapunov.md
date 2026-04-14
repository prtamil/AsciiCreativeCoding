# Pass 1 — lyapunov.c: Lyapunov Fractal

## Core Idea

The **Lyapunov fractal** maps the parameter plane of the **logistic map** `x → r·x·(1−x)` to a 2D image. Each pixel `(col, row)` corresponds to a pair of parameters `(a, b) ∈ [2.5, 4.0]²`. The logistic map is iterated with `r` alternating between `a` and `b` according to a fixed binary sequence (e.g. "AABAB"). The **Lyapunov exponent** λ of this orbit classifies the pixel:

- **λ < 0** — the orbit converges (stable / periodic) — rendered in blues
- **λ > 0** — the orbit diverges chaotically — rendered in reds
- **λ = 0** — boundary between order and chaos — rendered in black

## The Mental Model

### The logistic map

```
x_{n+1} = r · x_n · (1 − x_n)
```

For `r ∈ [0,4]` and `x₀ ∈ (0,1)`, x stays in `(0,1)`. At `r < 3` the orbit converges to a fixed point. As `r` increases past successive period-doubling bifurcations the attractor visits 2ⁿ points. Above `r ≈ 3.57` chaotic behaviour appears (the logistic map's **Feigenbaum cascade**).

### The Lyapunov exponent

For a 1D map `f(x)` the Lyapunov exponent measures exponential sensitivity to initial conditions:

```
λ = lim_{N→∞} (1/N) Σ_{n=0}^{N-1} ln|f'(x_n)|
```

For the logistic map `f'(x) = r·(1 − 2x)`, so:

```
λ = (1/N) Σ ln|r_n · (1 − 2x_n)|
```

where `r_n` follows the chosen sequence (A or B value for step n). The code uses `WARMUP_ITERS=100` to let the orbit settle, then accumulates over `LYAP_ITERS=200` steps.

### The sequence and 2D parameter space

The image axes are `a` (horizontal) and `b` (vertical), both in `[2.5, 4.0]`. The sequence (e.g. "AB", "AAB", "AABAB") determines which axis drives each step. Different sequences reveal different structures:

| Sequence | Structure |
|----------|-----------|
| "AB" | Classic Markus-Lyapunov image — clean boundary between stable/chaotic regions |
| "AAB" | Asymmetric — a-axis has more influence |
| "AABB" | Symmetric, checkerboard-like symmetry |
| "BBBAAAB" | Complex interlocked spirals |

### Progressive rendering

Each pixel requires 300 iterations of the logistic map — at 80×300 = 24,000 pixels total this is several hundred million float operations. Rendering all at once per frame would drop to ~1 fps.

The solution: compute `ROWS_PER_FRAME = 2` rows per frame, storing the encoded level in `g_lev[row][col]`. Previously computed rows are redrawn from the cache every frame — the image builds top to bottom. When complete, the simulation holds for `DONE_HOLD_FRAMES = 150` frames then auto-advances to the next sequence.

### Encoding scheme

`g_lev[row][col]` is an `int8_t`:
```
-127  → not yet computed (sentinel)
 0    → clipped (λ outside [−4, 2.5]) → draw nothing
-1..−4 → stable levels (|λ| increasing)
+1..+4 → chaotic levels (λ increasing)
```

This avoids storing full float λ values (which would require 4× more memory and a conversion every draw call).

### Color mapping

Two themes, each with 4 stable pairs (S1–S4, blues/greens) and 4 chaotic pairs (C1–C4, reds/yellows):

```
Classic:  stable = [51 33 21 17]  (bright→dark cyan→blue)
          chaotic = [226 208 196 124] (yellow→orange→red→dark)
Neon:     stable = greens, chaotic = magentas
```

Character selection reinforces depth:
```
stable:  S4='.', S3=':', S2='+', S1='#'   (deeper stable = dimmer char)
chaotic: C1='@', C2='#', C3='+', C4=':'   (deeper chaotic = bolder char at C1)
```

## Data Structures

```c
int8_t g_lev[GRID_ROWS_MAX][GRID_COLS_MAX]; /* encoded λ level */
int    g_cur_row;      /* next row to compute */
int    g_seq_idx;      /* index into k_sequences[] */
int    g_done_frames;  /* frames held since render complete */
```

## The Main Loop

```
reset_render():
    memset g_lev to -127  (sentinel)
    g_cur_row = 0
    g_done_frames = 0

compute_rows() per frame (when not done):
    for rr in 0 .. ROWS_PER_FRAME-1:
        b = B_MAX - row/(draw_rows-1) * (B_MAX - B_MIN)   ← b decreases top→bottom
        for col in 0 .. g_cols-1:
            a = A_MIN + col/(g_cols-1) * (A_MAX - A_MIN)
            lam = lyapunov_exponent(a, b, seq)
            g_lev[row][col] = level_encode(lam)
    g_cur_row += ROWS_PER_FRAME

when done:
    g_done_frames++
    if g_done_frames >= DONE_HOLD_FRAMES: advance sequence, reset_render()

render_grid():
    for each cell: look up g_lev → cp and ch → mvaddch
```

## Non-Obvious Decisions

### Why b maps top→bottom (inverted)

`b = B_MAX − row/draw_rows × (B_MAX − B_MIN)`

Screen rows go top (0) to bottom. The classic Markus-Lyapunov image has b increasing upward. The inversion aligns the image with the canonical orientation shown in textbooks — stable sea at bottom-left.

### Why warmup iterations are not counted in λ

The orbit may start far from the attractor. The first 100 iterations bring it to the neighbourhood of the eventual periodic orbit or chaotic attractor. Counting them would bias λ toward the transient behaviour, not the long-run dynamics.

### Divergence detection

```c
if (x <= 0.0 || x >= 1.0) return 10.0;
```

Once x leaves (0,1) it diverges to ±∞ in subsequent steps. Detecting this early and returning a large positive value marks the pixel as deeply chaotic without wasting further iterations.

### Why LEV_MIN = −4, LEV_MAX = 2.5

For most sequences in `[2.5, 4.0]²`, λ stays within this range. Points with |λ| > 4 are near degenerate fixed points or escaped orbits — marking them black (level 0) removes visual noise at the corners and lets the interesting structure fill the frame.

## Open Questions for Pass 3

- What is the **Hausdorff dimension** of the stable/chaotic boundary? It is conjectured to be fractal (between 1 and 2). Measure by counting boundary pixels at different resolutions.
- The sequence "BBBAAAB" produces nested spirals. Is the spiral structure related to the **winding number** of the period-doubling cascade in the logistic map?
- Can you add sequences longer than 7? Does the image converge to a fixed shape as sequence length → ∞ with a fixed a/b ratio?
- What happens on the diagonal `a = b`? Both axes use the same parameter, so the logistic map is iterated at constant `r = a = b` — the Lyapunov exponent should match the 1D logistic bifurcation diagram.
