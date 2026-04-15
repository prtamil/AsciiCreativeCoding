# Pass 1 — fourier_art.c: User-Drawn Path → Fourier Epicycle Reconstruction

## Core Idea

The user draws any shape on screen with cursor keys. When they press ENTER, the program:
1. Resamples the raw cursor path to 256 equally arc-length-spaced points
2. Computes the DFT of those points as complex numbers (x + iy)
3. Sorts the 256 resulting frequency components by amplitude
4. Animates a chain of rotating arms whose tip traces the drawn shape

The program has two phases: **DRAW** (user inputs a path) and **PLAY** (epicycle animation). Press `r` to draw again.

## The Mental Model

Every closed or open curve can be represented as a sum of rotating arms (epicycles). The DFT finds exactly which arms and at what speeds are needed. Large-amplitude arms reproduce the overall shape; small-amplitude arms fill in the fine detail. Adding arms one at a time (the auto-add feature) shows the Fourier series converging from a rough circle to the exact drawn shape.

### Why arc-length resampling matters

Suppose you draw a circle slowly on the left and quickly on the right. The raw cursor positions cluster densely on the left and sparsely on the right. If you feed these unequal-spacing samples directly to the DFT, the DFT "thinks" you spent more time on the left — the path is not uniformly parameterised. The DFT of a non-uniform parameterisation gives incorrect epicycle frequencies.

Arc-length resampling fixes this: you walk along the raw path and pick 256 equally-spaced-in-distance samples regardless of where the cursor paused. Now the DFT sees a uniformly parameterised path and gives the correct frequency decomposition.

### Why centering matters

The DC term Z[0] is the mean of all input samples. If the path is far off-centre, Z[0] is large and the "DC arm" dominates, just pointing to the centroid. By subtracting the centroid before the DFT, Z[0] = 0 and all arms describe shape, not position. The pivot (origin of the arm chain) is placed at screen centre.

## Data Structures

### Raw path (DRAW mode)
```
float  g_raw_px[RAW_MAX]   — pixel-space x of each cursor position
float  g_raw_py[RAW_MAX]
int    g_raw_n              — how many recorded so far
uint8_t g_drawn[ROWS_MAX][COLS_MAX]  — 1 if cell visited (for dot display)
int    g_cur_col, g_cur_row — current cursor cell position
```

### Epicycles (PLAY mode)
```
Epicycle g_epics[N_SAMPLES]:
  float amp    — |Z[n]| / N  (arm length in normalised units)
  float phase  — arg(Z[n])
  int   freq   — signed rotation rate (turns per animation cycle)

int  g_n_epics    — always N_SAMPLES after compute
int  g_n_active   — how many arms currently animated
```

### Trail (PLAY mode)
```
Trail g_trail:
  float px[TRAIL_LEN], py[TRAIL_LEN]   — ring buffer of tip positions
  int   head, count
```

### Arm chain tips
```
float g_tips_px[N_SAMPLES+1]   — pixel-space x at each arm junction
float g_tips_py[N_SAMPLES+1]   — index 0 = pivot; index g_n_active = tip
```

## The Main Loop

### DRAW mode
1. Input: arrow keys / WASD → `draw_move(dc, dr)` → `draw_record()` appends new pixel position to raw buffer
2. Tick: no simulation; just refresh screen
3. Draw: iterate `g_drawn[][]`, print `.` at each visited cell; print `@` at cursor; HUD shows point count
4. ENTER → `draw_compute()`: resample + center → DFT → sort epicycles → switch to PLAY

### PLAY mode  
1. Input: `+/-` add/remove epicycle, `p` pause, `c` circles, `t` theme, `r` → DRAW
2. `play_tick()`:
   - Auto-add one epicycle every AUTO_ADD_FRAMES
   - Advance `g_phi += 2π / CYCLE_FRAMES`
   - Recompute arm chain from pivot
   - Push tip to trail ring buffer
3. `play_draw()`: circles → trail → arm chain (Bresenham lines) → tip `@` → pivot `+` → HUD

## Non-Obvious Decisions

### Pixel space, not cell space
Cursor positions are recorded as `(col × CELL_W, row × CELL_H)` in pixel space, not as `(col, row)` in cell space. This is crucial: terminal cells are ~2× taller than wide (CELL_H=16, CELL_W=8). If we used cell coordinates, a "circle" drawn by moving equally in all four cell directions would actually be an ellipse in pixel space. Storing in pixel space preserves the true geometric shape.

The epicycle chain is also computed in pixel space and converted back to cell space only at draw time (`px_cx`, `px_cy`). This is the same pattern as `epicycles.c`.

### resample_and_center(): arc length walk
```c
arc[0] = 0;
for i in 1..n-1:
    arc[i] = arc[i-1] + |raw[i] - raw[i-1]|    // cumulative arc length
total = arc[n-1]

j = 0   // current segment pointer
for k in 0..N-1:
    s = k/N * total                              // target arc position
    while arc[j+1] < s: j++                     // advance to correct segment
    t = (s - arc[j]) / (arc[j+1] - arc[j])      // interpolation fraction
    out[k] = lerp(raw[j], raw[j+1], t)           // linearly interpolated point
```

### DFT twiddle-factor recurrence
Instead of calling `cos` and `sin` N times in the inner loop (N² trig calls total), the twiddle factor `W = exp(-2πi·n/N)` is advanced by complex multiplication:
```c
W_k = W_{k-1} × W     // multiply, no new trig
```
This reduces trig calls from N² to N (one per frequency bin), while the inner loop only does multiplications. For N=256 at O(N²), this is a significant constant-factor speedup.

### Signed frequencies
The DFT output index `n` gives unsigned frequency. For n > N/2, the "natural" frequency is negative: freq = n - N. This matters because negative-frequency arms rotate clockwise while positive-frequency arms rotate counter-clockwise. Without signed frequency, the reconstruction would be wrong — all arms would rotate in the same direction and the shape would not reproduce correctly.

### AUTO_ADD_FRAMES = 8
At 30fps, auto-adding one epicycle every 8 frames means 256/8 = 32 seconds to fully converge. That is long enough to watch the reconstruction improve visibly at each step, but fast enough that the program feels responsive. The `+/-` keys allow manual control to pause auto-add and examine any convergence state.

### Scale fitting
After centering, `max_r` is the max distance from the origin in any of the 256 resampled points. The scale is set so the reconstruction fits in 40% of the smaller screen dimension:
```c
screen_r = 0.40 × min(cols×CELL_W, rows×CELL_H)
g_scale  = screen_r / max_r
```
This ensures every drawn shape fills a reasonable portion of the screen regardless of actual drawing size.

## State Machines

```
DRAW ──── ENTER/g ────► PLAY
   ◄──── r ────────────
```

Inside PLAY:
```
each frame: phi += 2π/CYCLE_FRAMES
if phi ≥ 2π: phi -= 2π; trail_clear()    // one full cycle complete
```

## From the Source

**Algorithm:** Interactive path-recording → DFT epicycle reconstruction. User traces a free-form path; on ENTER the path is resampled to N_SAMPLES points using arc-length parameterisation (uniform spacing along the drawn curve), then an O(N²) DFT converts it to epicycle coefficients.

**Math:** Arc-length resampling: cumulative chord-length distances are computed, then uniform sample positions are mapped back to original points via linear interpolation. This ensures the DFT receives uniform-time samples regardless of how fast the user drew, eliminating velocity artefacts. DFT of complex path: z[k] = x[k] + i·y[k]; Z[n] gives the n-th epicycle amplitude and initial phase.

**Performance:** DFT is computed once (O(N²)) at draw-time, not per frame. Per-frame cost is O(N_active) arm chain evaluations. RAW_MAX=8192 points buffered during draw; resampled to N_SAMPLES=256 before DFT to bound computation.

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `N_SAMPLES` | 256 | DFT resolution; more = finer reconstruction but O(N²) slower |
| `RAW_MAX` | 8192 | Max raw cursor positions before buffer full |
| `CYCLE_FRAMES` | 300 | Frames per full reconstruction cycle (~10s at 30fps) |
| `AUTO_ADD_FRAMES` | 8 | Frames between auto-adding one epicycle |
| `TRAIL_LEN` | 600 | Ring buffer length for the tip trail |
| `N_CIRCLES` | 6 | How many of the largest arms show their orbit circles |

## Themes

5 themes cycle with `t`/`T`:

| # | Name | Arms | Trail |
|---|---|---|---|
| 0 | Classic | White/cyan | Yellow → orange → red |
| 1 | Fire | Orange/red | Yellow → gold → red |
| 2 | Neon | Magenta/violet | Pink gradient |
| 3 | Ocean | Teal/blue | Cyan gradient |
| 4 | Matrix | Green shades | Lime gradient |

---

# Pass 2 — fourier_art: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | N_SAMPLES=256, RAW_MAX=8192, TRAIL_LEN=600, CYCLE_FRAMES=300 |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 5 themes × 12 color roles, `theme_apply(t)` |
| §4 DFT | `Cplx`, `compute_dft()`, `Epicycle`, `build_epicycles()`, `resample_and_center()` |
| §5 scene | DRAW state (cursor, raw buffer, drawn grid), PLAY state (chain, trail, epicycles) |
| §6 screen | ncurses init |
| §7 app | main loop, signal handlers, resize |

## Data Flow

```
DRAW mode:
  arrow key → draw_move(dc, dr):
    new_col = clamp(cur_col + dc, 0, cols-1)
    new_row = clamp(cur_row + dr, 0, rows-2)
    record (new_col × CW, new_row × CH) into raw_px/py if moved
    g_drawn[new_row][new_col] = 1

  ENTER → draw_compute():
    samples[256] = resample_and_center(raw_px, raw_py, raw_n)
    max_r = max |samples[k]|
    build_epicycles(samples, 256)
    g_scale = 0.4 × min(screen_w, screen_h) / max_r
    g_state = STATE_PLAY

PLAY mode (each tick):
  if auto_add and n_active < n_epics and auto_cnt++ >= AUTO_ADD_FRAMES:
    n_active++; auto_cnt = 0
  phi += 2π / CYCLE_FRAMES
  if phi >= 2π: phi = 0; trail_clear()
  x = pivot_px; y = pivot_py
  tips[0] = (x, y)
  for i in 0..n_active-1:
    ang = epics[i].freq * phi + epics[i].phase
    r   = epics[i].amp * g_scale
    x += r*cos(ang); y += r*sin(ang)
    tips[i+1] = (x, y)
  trail_push(tips[n_active])
```

## resample_and_center(rx, ry, n, out, N)
```
arc[0] = 0
for i=1..n-1: arc[i] = arc[i-1] + sqrt((rx[i]-rx[i-1])^2 + (ry[i]-ry[i-1])^2)
total = arc[n-1]

j = 0
for k=0..N-1:
  s = k/N * total
  while j < n-2 and arc[j+1] < s: j++
  t = (s - arc[j]) / (arc[j+1] - arc[j])
  out[k] = { re: lerp(rx[j], rx[j+1], t),
             im: lerp(ry[j], ry[j+1], t) }

mean_re = sum(out[k].re)/N;  mean_im = sum(out[k].im)/N
for k: out[k].re -= mean_re; out[k].im -= mean_im
return max |out[k]|
```

## compute_dft(in, out, N)
```
for n=0..N-1:
  W = exp(-2πi·n/N)          // twiddle factor
  w = 1 + 0i                 // W^0
  acc = 0+0i
  for k=0..N-1:
    acc += in[k] * w          // complex multiply-accumulate
    w *= W                    // advance by multiplication (no new trig)
  out[n] = acc
```

## Open Questions for Pass 3

- At RAW_MAX=8192 raw points, the resample still only produces 256 DFT samples — does the extra density improve quality or is it redundant once arc-length is used?
- For open (non-closed) paths, the DFT treats the path as periodic — there is a "seam" discontinuity at start/end. Is this visible in the animation, and does it add unwanted high-frequency epicycles?
- How does the auto-add look for very simple paths (e.g., a straight line drawn left-right)? A line's DFT should have two dominant epicycles (DC + fundamental) and the rest near zero.
- Does `CYCLE_FRAMES=300` feel too fast or too slow for a full trace of complex paths? The user might want to control animation speed.
