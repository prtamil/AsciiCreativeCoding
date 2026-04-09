# Pass 1 — julia.c: Julia set fractal with random-pixel animated reveal

## Core Idea

Every terminal cell is mapped to a complex number z = re + im·i. The Julia iteration `z → z² + c` is applied up to MAX_ITER=128 times. If |z| exceeds 2 the orbit has escaped; the number of iterations before escape determines the color band. If the orbit never escapes the cell is inside the Julia set and drawn white. Pixels are revealed in a Fisher-Yates shuffled random order so the fractal materialises from scattered dots rather than a boring scan line. Six preset Julia parameter values (c) cycle automatically, each producing a distinctly different fractal shape. The fire gradient palette (dark-red → red → orange → yellow → white) maps the escape speed to color.

## The Mental Model

The Julia set for a given complex constant c is the boundary between complex numbers whose orbits stay bounded forever and those that escape to infinity. The escape-time algorithm approximates this boundary by testing how quickly each point escapes. Points that escape quickly (in few iterations) are far outside the set and drawn dark. Points that take many iterations are close to the boundary and drawn bright. Points that never escape are inside the set and drawn white.

Each of the six presets sets c to a different complex constant, changing the shape of the boundary entirely:
- **Douady rabbit**: c ≈ −0.123 + 0.745i — three-lobed rabbit shape with self-similar ears
- **Spiral galaxy**: c ≈ −0.7269 + 0.1889i — dense spiral filaments
- **Dendrite**: c ≈ 0 + 1i — tree-branching structure
- **Flame**: c ≈ −0.8 + 0.156i — feathery flame-like filaments
- **Seahorse valley**: c ≈ −0.7 − 0.3i — seahorse spirals
- **Basilica**: c ≈ −0.4 + 0.6i — symmetrical cathedral-arch shape

The Fisher-Yates shuffle ensures every pixel is revealed exactly once in pseudo-random order. The fractal appears to "crystallize" from random static rather than sweeping across the screen.

## Data Structures

### Grid (§4)
```
uint8_t  color[GRID_ROWS_MAX][GRID_COLS_MAX]  — escape band per cell (0=not yet computed)
int      rows, cols
int      pixels_done    — count toward rows*cols
int      done_ticks     — hold period after all pixels computed
int      preset         — index into k_presets[N_PRESETS]
```

### Shuffle order (§5)
```
int *order              — dynamically allocated array of rows*cols indices
int  order_pos          — current read position in shuffled order
```
The shuffle array is allocated at scene init and freed on resize/reset. This is one of the few dynamic allocations in the codebase.

### Preset (§5)
```
float cr, ci            — real and imaginary parts of the Julia constant c
float re_center, re_half, im_half  — viewport window into the complex plane
const char *name
```

### Scene (§6)
```
Grid   grid
bool   paused
```

## The Main Loop

1. Resize: reallocate shuffle array, reset grid and shuffled order.
2. Measure dt.
3. Ticks: compute PIXELS_PER_TICK=60 pixels per tick from the shuffled order; for each index, compute (re, im) from (col, row), iterate, classify escape band, store in grid.color.
4. After all pixels done: hold DONE_PAUSE_TICKS=90, then advance to next preset and reset.
5. Draw: for each non-zero cell, write the band character with the band color pair.
6. HUD: fps, preset name, progress percentage, speed.
7. Input: r/n=next preset, p=pause, [/]=speed.

## Non-Obvious Decisions

### Why Fisher-Yates shuffle instead of scan order?
Scan order (left-to-right, top-to-bottom) reveals the fractal as a wipe. The Julia set has a very structured layout, so a wipe would reveal the interior white blob first, then the colored bands, making the animation look mechanical. Random order makes every pixel a surprise — the fractal crystallizes from apparent chaos, which is visually more satisfying and also a subtle reference to the "chaos" in chaotic dynamical systems.

### PIXELS_PER_TICK = 60 — why not compute all at once?
At 30fps that gives 60×30=1800 pixels per second, completing a 24000-cell grid (80×300) in 13.3 seconds. This is slow enough to watch the fractal emerge but fast enough to complete in a reasonable time. If all pixels were computed instantly, there would be no animation — the fractal would just appear.

### ASPECT_R correction in the complex plane mapping
Terminal cells are 2× taller than wide. If you map (col → re) and (row → im) with the same scale, the complex plane is stretched vertically by 2× and the Julia set appears squashed. The fix is `re_half = im_half * (cols / rows) / ASPECT_R` — this makes one im unit equal one re unit in screen distance.

### MAX_ITER = 128
Higher max_iter means more detail near the boundary (points near the boundary take more iterations to escape). 128 is a balance between detail and computation time. For mandelbrot.c the cap is 256 because the Mandelbrot set has more fine structure.

### Escape color bands
The escape iteration count is bucketed into 5 bands:
- inside set → white (brightest)
- slow escape (many iters) → yellow
- medium escape → orange
- fast escape → red
- very fast → dark red
- instant escape → black (cell not drawn)
The `color[row][col] = 0` for instant escape means those cells are skipped in `grid_draw`, leaving them as terminal background (black).

## State Machines

```
COMPUTING ──── all pixels done ────► HOLDING (done_ticks++)
    ▲                                       │
    └──── done_ticks >= DONE_PAUSE ─────────┘ (next preset, reset)
```

## Key Constants

| Constant | Default | Effect if changed |
|---|---|---|
| `PIXELS_PER_TICK` | 60 | Higher = faster fill; lower = slower emergence |
| `MAX_ITER` | 128 | Higher = more boundary detail; more CPU per pixel |
| `DONE_PAUSE_TICKS` | 90 | ~3 s hold after completion |
| `N_PRESETS` | 6 | Fixed set of Julia parameters |
| `ASPECT_R` | 2.0 | Must match terminal cell aspect; wrong value distorts shape |

## Open Questions for Pass 3

- How exactly is the Fisher-Yates shuffle implemented? Is it a full Knuth shuffle at init, or an online reservoir sample?
- What exactly happens at the boundary between escape bands — is there smooth interpolation or hard cutoffs?
- When the terminal is resized, is the shuffle array reallocated and re-shuffled, or is the existing permutation truncated?
- Why are the preset viewport windows different sizes? Is seahorse valley zoomed in more than the full set?

---

# Pass 2 — julia: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | PIXELS_PER_TICK, MAX_ITER, DONE_PAUSE_TICKS, N_PRESETS |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 5-level fire gradient, `color_init()` |
| §4 grid | color[][] array, done tracking, `grid_draw()`, `grid_reset()` |
| §5 compute | Fisher-Yates shuffle, preset table, `compute_pixel()`, `escape_band()` |
| §6 scene | Owns grid + shuffle, `scene_tick()`, `scene_draw()` |
| §7 screen | ncurses init/draw/present/resize, HUD |
| §8 app | Signal handlers, main loop, key handling |

---

## Data Flow Diagram

```
scene_init(preset):
  allocate order[rows*cols]  ← [0, 1, 2, ..., rows*cols-1]
  Fisher-Yates shuffle order[]
  grid_reset()
  order_pos = 0

scene_tick():
  for i in 0..PIXELS_PER_TICK-1:
    if order_pos >= rows*cols: break
    idx = order[order_pos++]
    col = idx % cols
    row = idx / cols
    re = re_center + (col - cols/2.0) * re_step
    im = im_center - (row - rows/2.0) * im_step
    band = julia_escape(re, im, cr, ci)
    grid.color[row][col] = band     (0 if instant escape → drawn as background)

  if order_pos >= rows*cols:
    done_ticks++
    if done_ticks >= DONE_PAUSE_TICKS:
      preset = (preset + 1) % N_PRESETS
      scene_init(preset)

julia_escape(re, im, cr, ci):
  zr = re, zi = im
  for iter = 0..MAX_ITER-1:
    zr2 = zr*zr - zi*zi + cr
    zi2 = 2*zr*zi + ci
    zr = zr2; zi = zi2
    if zr*zr + zi*zi > 4.0: return escape_band(iter)
  return COL_INSIDE     ← never escaped = inside set = white
```

---

## Function Breakdown

### escape_band(iter) → color_id
Purpose: map escape iteration to one of 5 color bands.
Steps:
1. if iter < 4: return 0  (black — too fast, treat as background)
2. if iter < 16: return COL_DARK_RED
3. if iter < 48: return COL_RED
4. if iter < 96: return COL_ORANGE
5. else: return COL_YELLOW

### Fisher-Yates shuffle (at scene_init)
```
for i from rows*cols-1 downto 1:
  j = rand() % (i+1)
  swap(order[i], order[j])
```
Produces a uniformly random permutation. Each cell is computed exactly once.

---

## Pseudocode — Core Loop

```
setup:
  preset = 0
  scene_init(0)

main loop:
  1. resize → free order, scene_init(preset) at new size

  2. dt, cap 100ms

  3. physics ticks:
     while sim_accum >= tick_ns:
       if not paused: scene_tick()

  4. frame cap sleep

  5. draw:
     erase()
     grid_draw()     ← non-zero cells with band char + color
     HUD: fps, preset name, pct%, speed
     wnoutrefresh + doupdate

  6. input:
     q/ESC     → quit
     r / n     → (preset+1)%N_PRESETS, scene_init
     ] [       → sim_fps
     p / spc   → pause
```

---

## Interactions Between Modules

```
App
 └── scene_tick → julia_escape (×60) → grid.color[][]
               → done_ticks → preset advance → scene_init

grid.color[][]
 └── scene_draw → for each non-zero → mvwaddch band char + color pair

order[]  (shuffle array)
 └── owned by scene; freed/reallocated on resize
```
