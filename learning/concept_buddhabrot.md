# Pass 1 — buddhabrot.c: Buddhabrot density-accumulator fractal

## Core Idea

The Buddhabrot renders the Mandelbrot iteration as a density map of orbital trajectories rather than an escape-time map. For each randomly sampled complex number c that escapes the iteration `z → z² + c`, every intermediate z value visited by the orbit is projected onto a terminal grid and a hit counter is incremented. Regions traversed by many orbits accumulate high counts and glow brightest — producing a figure that resembles a luminous seated Buddha. The Anti-Buddhabrot variant traces orbits that do NOT escape (bounded orbits), revealing the Mandelbrot set's interior as a dense attractor. Five presets cycle automatically. Log normalization with a mode-aware invisible floor suppresses scattered dots from transient cells.

## The Mental Model

Imagine every point that escapes the Mandelbrot iteration as a ball rolling through a pinball machine. The pinball machine has wells at certain spots (where the orbit slows down near cycle boundaries) and fast-pass lanes (where the orbit quickly shoots off to infinity). After many balls have rolled through, some wells have deep scratches (high density) and some lanes have almost none (low density).

The Buddhabrot photograph shows the wear pattern — where orbits spend most of their time before escaping. The outer filaments of the traditional Mandelbrot set (the boundary regions) map to bright areas in the Buddhabrot because boundary-near points take many iterations to escape and their orbits wander extensively.

The Anti-Buddhabrot shows a completely different picture — the interior attractor. Bounded orbits cycle forever, concentrating density into smooth attractor curves inside the set. This looks like an alien landscape of nested rings and loops.

The key algorithmic insight is the two-pass design: compute the escape test cheaply (pass 1), then only trace the orbit (expensive) for c values that satisfy the mode condition.

## Data Structures

### Grid (§4)
```
uint32_t counts[80][300]     — hit count per terminal cell (static, ~96 KB)
uint32_t max_count           — tracked incrementally; used for normalization
int      rows, cols
int      samples_done        — counter toward TOTAL_SAMPLES
int      done_ticks
int      preset              — index into k_presets[N_PRESETS]
float    re_center, re_half
float    im_center, im_half
float    re_lo, re_range_inv — precomputed bounds for fast projection
float    im_hi, im_range_inv
```
The 96 KB grid is a single static global — no dynamic allocation.

### Preset table (§5)
```
struct { int max_iter; BuddhMode mode; const char *name; } k_presets[5] = {
    {  500, MODE_BUDDHA, "buddha  500" },
    { 2000, MODE_BUDDHA, "buddha 2000" },
    {  100, MODE_ANTI,   "anti    100" },
    {  500, MODE_ANTI,   "anti    500" },
    { 1000, MODE_ANTI,   "anti   1000" },
};
```

### Scene (§6)
```
Grid    grid
bool    paused
float   sim_fps
```

## The Main Loop

1. Resize: recompute grid viewport bounds (re_lo, re_range_inv, etc.), clear counts.
2. Measure dt.
3. Ticks: compute SAMPLES_PER_TICK random samples per tick; for each, run two-pass orbit test, accumulate hits.
4. After TOTAL_SAMPLES=150000: hold DONE_PAUSE_TICKS, then next preset.
5. Draw: for each non-zero cell, log-normalize count against max_count, apply mode-aware floor, write nebula character with color pair.
6. HUD: fps, preset name, progress %, speed.
7. Input: r/n=next preset, p=pause, [/]=speed.

## Non-Obvious Decisions

### Two-pass design (no orbit buffer)
The obvious approach allocates an orbit buffer, runs the iteration once, and if the orbit escapes, iterates through the buffer to accumulate hits. The two-pass approach re-runs the iteration from scratch for qualifying samples. This costs 2× computation for accepted samples but 0 bytes of orbit storage and no per-tick allocation.

Pass 1 (escape test) runs at most max_iter iterations and returns a boolean. Pass 2 runs at most max_iter iterations and increments grid cells. For buddha mode (escaping orbits) most samples are rejected in pass 1 (interior/cardioid points) so the two-pass cost is modest. For anti mode most samples are rejected (escaped points), which is also fast.

### Cardioid/bulb pre-rejection (Buddha mode only)
The main cardioid and period-2 bulb together contain roughly 20-30% of the sampling area. Points inside these regions never escape, so in Buddha mode they waste max_iter iterations in pass 1 only to be rejected. The pre-check uses:
```c
float q = (cr - 0.25f)*(cr - 0.25f) + ci*ci;
if (q*(q + cr - 0.25f) < 0.25f*ci*ci) return;   /* cardioid */
if ((cr + 1.0f)*(cr + 1.0f) + ci*ci < 0.0625f)  return;   /* period-2 bulb */
```
This is an analytical test that is much faster than running max_iter iterations.

### log normalization with mode-aware floor
Anti-mode attractor cells accumulate up to `max_iter × accepted_samples` hits (potentially millions). With sqrt normalization `sqrt(1/10⁶) ≈ 0.001` still falls in the first visible band, scattering dots across a blank screen.

Log normalization: `t = logf(1+count) / logf(1+max_count)`. For the same case: `log(2)/log(10⁶+1) ≈ 0.035` — below the anti-mode floor of 0.25 → invisible.

The invisible floor is mode-dependent:
- Anti mode: floor = 0.25 — aggressive suppression of transient noise dots
- Buddha mode: floor = 0.05 — gentle suppression; preserves fine orbital structure

### Incremental max_count tracking
Rather than scanning all 80×300=24000 cells each tick to find max_count, it is tracked incrementally: `if (++g->counts[row][col] > g->max_count) g->max_count = v`. This makes normalization O(1) per tick.

### Sampling bounds vs display viewport
The sampling region `[SAMPLE_RE_MIN, SAMPLE_RE_MAX] × [SAMPLE_IM_MIN, SAMPLE_IM_MAX]` is slightly larger than the display viewport. This ensures orbits whose trajectories pass through the display area are captured even if their starting point c is outside the display region.

## State Machines

```
ACCUMULATING ──── samples_done >= TOTAL_SAMPLES ────► HOLDING (done_ticks++)
    ▲                                                         │
    └──── done_ticks >= DONE_PAUSE_TICKS ─────────────────────┘
          preset = (preset + 1) % N_PRESETS
          grid_init()   ← clear counts, recompute bounds
```

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `TOTAL_SAMPLES` | 150000 | More = denser, more recognizable image; fewer = sparser |
| `SAMPLES_PER_TICK` | varies | Controls how many random samples per sim tick |
| `DONE_PAUSE_TICKS` | varies | Hold time before cycling to next preset |
| Buddha floor | 0.05 | Lower = more noise visible; higher = sparser image |
| Anti floor | 0.25 | Lower = noise dots visible; higher = fewer visible cells |

## Open Questions for Pass 3

- What is SAMPLES_PER_TICK — a fixed constant or adaptive?
- Is the sampling uniform over the bounding box, or does it use importance sampling to concentrate on boundary regions?
- How exactly are re_range_inv and im_range_inv precomputed — are they simply 1/(re_max - re_min)?
- Does TOTAL_SAMPLES count all random samples (including rejected ones in pass 1) or only accepted ones?

---

# Pass 2 — buddhabrot: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | TOTAL_SAMPLES, SAMPLES_PER_TICK, viewport bounds, palette |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 5-level nebula palette (dark purple → white), `color_init()` |
| §4 grid | counts[][], max_count, bounds, `grid_init()`, `grid_draw()` |
| §5 compute | `grid_sample()` — two-pass orbit test + accumulate |
| §6 scene | Owns grid, `scene_tick()`, `scene_draw()`, done tracking |
| §7 screen | ncurses init/draw/present/resize, HUD |
| §8 app | Signal handlers, main loop, key handling |

---

## Data Flow Diagram

```
grid_sample(g, cr, ci):
  /* cardioid/bulb check (buddha mode only) */
  if buddha: precheck → early return if inside cardioid or bulb

  /* Pass 1 — escape test */
  zr = 0; zi = 0
  for iter = 0..max_iter-1:
    zr2 = zr*zr - zi*zi + cr
    zi2 = 2*zr*zi + ci
    zr = zr2; zi = zi2
    if zr*zr + zi*zi > 4: break
  escaped = (iter < max_iter)
  if anti_mode == escaped: return   ← wrong mode, skip

  /* Pass 2 — orbit trace */
  zr = 0; zi = 0
  for i = 0..max_iter-1:
    col = (int)((zr - re_lo) * re_range_inv * cols + 0.5)
    row = (int)((im_hi - zi) * im_range_inv * rows + 0.5)
    if in bounds: v = ++counts[row][col]; if v > max_count: max_count = v
    zr2 = zr*zr - zi*zi + cr
    zi2 = 2*zr*zi + ci
    zr = zr2; zi = zi2
    if !anti_mode && zr*zr + zi*zi > 4: break

density_color(count, max_count, anti):
  t = logf(1 + count) / logf(1 + max_count)
  floor = anti ? 0.25 : 0.05
  if t < floor:  return 0         (invisible)
  if t < 0.45:   return COL_C1    (dark blue-purple)
  if t < 0.62:   return COL_C2    (violet)
  if t < 0.78:   return COL_C3    (light purple)
  if t < 0.90:   return COL_C4    (lavender-pink)
  return         COL_C5           (white, bold)
```

---

## Function Breakdown

### grid_init(g, cols, rows, preset)
Purpose: clear counts, set viewport bounds for given preset.
Steps:
1. memset counts to 0; max_count = 0; samples_done = 0; done_ticks = 0.
2. Set re_center = −0.5, im_center = 0, im_half = 1.3.
3. re_half = im_half × cols / rows / ASPECT_R.
4. Precompute: re_lo = re_center − re_half, re_range_inv = 1/(2×re_half×cols), etc.

### scene_tick()
Purpose: compute SAMPLES_PER_TICK samples.
Steps:
1. For each sample: cr = random in [SAMPLE_RE_MIN, SAMPLE_RE_MAX], ci = random in [SAMPLE_IM_MIN, SAMPLE_IM_MAX].
2. Call grid_sample(g, cr, ci).
3. samples_done++.
4. If samples_done >= TOTAL_SAMPLES: done_ticks++; if done_ticks >= DONE_PAUSE: next preset.

---

## Pseudocode — Core Loop

```
setup:
  grid_init(cols, rows, 0)

main loop:
  1. resize → grid_init(new cols, rows, preset)

  2. dt, cap 100ms

  3. physics ticks:
     while sim_accum >= tick_ns:
       if not paused: scene_tick()

  4. frame cap sleep

  5. draw:
     erase()
     grid_draw()     ← density_color → nebula chars
     HUD: fps, preset name, pct%, speed
     wnoutrefresh + doupdate

  6. input:
     q/ESC     → quit
     n / r     → (preset+1)%N_PRESETS, grid_init
     ] [       → sim_fps
     p / spc   → pause
```

---

## Interactions Between Modules

```
App
 └── scene_tick → grid_sample (×SAMPLES_PER_TICK)
                  → Pass 1 (escape test)
                  → Pass 2 (orbit accumulate → counts[][] + max_count)
               → done check → grid_init (next preset)

grid_draw()
 └── for each (row, col): density_color(counts[row][col], max_count, anti) → mvwaddch
```
