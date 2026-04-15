# Pass 1 — kaboom: Deterministic LCG Explosion

## Core Idea

A single explosion animation plays through 150 frames. The explosion is computed frame-by-frame using a deterministic Linear Congruential Generator (LCG) PRNG — not `rand()`. This means the same seed always produces the same explosion shape. The blast consists of three visual layers built each frame: initial disc, blast wave (shaped by `petal_n` lobes), and 800 3D debris blobs. The animation plays, then restarts with a new theme and shape combination.

---

## The Mental Model

The terminal is treated as a coordinate system centered at (0,0). Each frame, every cell is evaluated by mathematical formulas to decide what character and color to show.

**Three phases:**
1. **Frame 0**: single `*` at center (ignition flash)
2. **Frames 1-7**: expanding disc of `@` — radius = frame × disc_speed
3. **Frames 8-150**: blast wave expands outward; inner fireball characters shrink; 3D blob cloud flies outward

The blast wave shape is controlled by `petal_n` — an angular frequency that creates symmetric lobes (`petal_n=16` = spiky ring, `petal_n=6` = hex star, `petal_n=0` = smooth sphere).

The 800 debris blobs are precomputed at init using the LCG: each blob has a position on a unit sphere surface. Each frame, blobs are scaled by `(frame-6) × blob_speed` and projected with perspective onto the terminal.

---

## Data Structures

### `Blob` struct
| Field | Meaning |
|---|---|
| `x, y, z` | Unit-sphere surface position, scaled by (1.3 + 0.2*prng()) for irregular radii |

### `Cell` struct
| Fields | Meaning |
|---|---|
| `ch` | Character to draw (0 = empty cell) |
| `color` | ColorID (FLASH/INNER/WAVE/BLOB_F/BLOB_M/BLOB_N) |

### `Blast` struct
| Field | Meaning |
|---|---|
| `blobs[800]` | All 3D debris particles — fixed, computed once at init |
| `cells[cols*rows]` | Per-frame output: what to draw where |
| `frame` | Current frame index 0..NUM_FRAMES |
| `theme` | Index into k_themes (color set) |
| `shape` | Index into k_shapes (geometry parameters) |
| `done` | True when frame reaches NUM_FRAMES |

### `BlastShape` struct
Controls the geometry of the explosion:
- `petal_n` — number of lobes in angular modulation (0 = smooth)
- `ripple` — amplitude of lobe wobble
- `disc_speed` — how fast initial disc expands
- `y_squash` — vertical compression of blob cloud
- `persp` — perspective depth
- `blob_speed` — how fast blobs fly out
- `flash_chars` / `wave_chars` — character sequences for each phase

---

## The Main Loop

kaboom plays a pre-scripted animation:
1. `blast_tick()` — render frame N into `cells[]`, then advance `frame++`
2. `blast_draw()` — write non-empty cells to stdscr
3. When `done`, reset with next theme+shape and replay

---

## Non-Obvious Decisions

### LCG PRNG instead of `rand()`
```c
static double prng(void) {
    static long long s = 1;
    s = s * 1488248101LL + 981577151LL;
    return ((s % 65536) - 32768) / 32768.0;
}
```
The LCG state is global and deterministic from `s=1`. Every run produces the same blob positions. This means `'r'` to replay shows the exact same explosion — consistent, reproducible art. No need to store seeds or reset.

### Cell-based rendering (no erase + redraw per cell)
Each frame clears `cells[]` to zero, fills it with character+color for each non-empty cell, then `blast_draw()` writes to terminal. Empty cells (ch==0) are skipped. No `erase()` is called — instead the whole terminal is explicitly overwritten each frame by drawing space characters for cells that had content last frame but are now empty... actually looking at the code, `blast_draw()` skips cells with `ch==0`, but the main loop calls `erase()` before each draw.

### Blob on sphere surface
Each blob is positioned by:
1. Generate random (bx, by, bz) in [-1,1]
2. Normalize to unit sphere surface: `b.x = bx/|b|`
3. Add slight radial jitter: multiply by `1.3 + 0.2*prng()`
4. y_squash applied at render time, not at init

This gives uniformly distributed points on a sphere, not in a ball.

### Blast wave character by `v` index
```
v = frame - (int)r - 7
```
`r` is the distance from center (with perspective correction and lobe modulation). `v` indexes into `wave_chars` string — cells just inside the wave front show earlier chars, cells at the wave front show middle chars, cells outside show nothing. The wave is a moving "curtain" that sweeps outward.

### Color zones by z-depth for blobs
```
bz > persp*0.8  → COL_BLOB_F + '.'   (far blobs, dim)
bz > -persp*0.4 → COL_BLOB_M + 'o'  (mid blobs)
else            → COL_BLOB_N + '@'   (near blobs, bright)
```
Depth-based coloring without a z-buffer — just categorical zones.

---

## State Machine

```
PLAYING (frame 0..149):
  Each tick: blast_render_frame(), frame++
  frame == NUM_FRAMES:
    done = true

DONE:
  Next tick: blast_free(), blast_init() with next theme+shape
  done = false, frame = 0
  → PLAYING
```

---

## From the Source

**Algorithm:** Deterministic LCG PRNG: `s = s * 1488248101LL + 981577151LL; return (s%65536 - 32768) / 32768.0`. Static state `s` starts at 1 — same blob positions every run. The `prng()` is called both by `blob_init_pool()` and `blast_render_frame()` (wave noise uses the same LCG sequence), so the per-frame noise pattern is also deterministic.

**Math:** Blob placement on unit sphere: generate random `(bx,by,bz)` via `prng()`, normalize: `br = sqrt(bx²+by²+bz²); blob.x = (bx/br)*(1.3+0.2*prng())`. y pre-squashed by 0.5 at init time. Lobe formula: `lobe = 1 + ripple * cos(petal_n * angle)` where angle = `atan2(y*2+0.01, x+0.01)` — the `*2` and offset prevent atan2 singularity at origin. Radial wave: `r = sqrt(x²+4y²) * (0.5 + prng()/3 * lobe * 0.3)`; `4y²` compensates for cell aspect ratio.

**Physics:** Blob scale: `i0 = frame - 6; bx_screen = blob.x * i0 * blob_speed`. Perspective projection: `cx = cols/2 + bx*persp/(bz+persp)`. Three depth zones by `bz` vs `persp`: far (`.` dim) / mid (`o` normal) / near (`@` bright) — no z-buffer, just categorical buckets.

**Performance:** `cells[cols*rows]` array rebuilt from scratch each frame — all visible layers computed fresh. No incremental update. Fixed pool of 800 blobs computed once at init.

## Key Constants and What Tuning Them Does

| Constant | Default | Effect |
|---|---|---|
| `NUM_FRAMES` | 150 | Total animation length |
| `NUM_BLOBS` | 800 | More = denser debris cloud |
| `PERSPECTIVE` | 50.0 | Default; each shape has its own persp |
| `petal_n` (per shape) | 0–16 | Controls lobe count (0=circle, 16=spiky) |
| `ripple` | 0.0–0.6 | How jagged the lobes are |

---

## Open Questions for Pass 3

1. Try seeding the LCG with `time(NULL)` — how does it change the explosion?
2. What happens with `petal_n=1`? (`petal_n=1` produces an asymmetric teardrop)
3. Can you separate the disc phase from the wave phase visually by changing `disc_speed`?
4. The z-depth color zones use categorical buckets — how would continuous z-based brightness look?
5. `blast_render_frame()` rebuilds `cells[]` from scratch each frame — is there a way to make it incremental?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `blobs[NUM_BLOBS]` | `Blob[800]` | ~19 KB | 3-D debris particle pool (x,y,z doubles) |
| `cells[cols*rows]` | `Cell[]` | ~20 KB | per-cell framebuffer (char + color) |

# Pass 2 — kaboom: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | NUM_FRAMES=150, NUM_BLOBS=800, PERSPECTIVE=50, SIM_FPS |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 7 ColorIDs (FLASH/INNER/WAVE/BLOB_F/BLOB_M/BLOB_N/HUD); BlastTheme struct; 6 themes; `color_theme_apply()` |
| §4 blob | `Blob` struct (x,y,z); `prng()` LCG; `blob_init_pool()` — unit sphere distribution |
| §5 blast | `Blast` struct; `blast_init()`, `blast_render_frame()`, `blast_tick()`, `blast_draw()` |
| §6 screen | ncurses init/resize/present/HUD |
| §7 app | Main dt loop, replay logic, input, signals |

---

## Data Flow Diagram

```
At init:
  blob_init_pool():
    for i in 0..799:
      (bx, by, bz) = prng() × 3
      br = sqrt(bx² + by² + bz²)
      blob.x = bx/br × (1.3 + 0.2×prng())  ← unit sphere + jitter
      blob.y = 0.5 × by/br × (1.3 + 0.2×prng())  ← y squash at init
      blob.z = bz/br × (1.3 + 0.2×prng())

Each tick:
  blast_render_frame(blast):
    clear cells[] to {ch=0}
    origin = (cols/2, rows/2)

    for each cell (x, y) in [-cols/2..cols/2] × [-rows/2..rows/2]:
      if frame == 0:
        if x==0 and y==0: cell = ('*', COL_FLASH)

      else if frame < 8:
        r = sqrt(x² + 4y²)   ← 4y² compensates for cell aspect ratio
        if r < frame × disc_speed:
          cell = ('@', COL_FLASH)

      else:
        angle = atan2(y×2 + 0.01, x + 0.01)
        if petal_n > 0:
          lobe = 1 + ripple × cos(petal_n × angle)
        else:
          lobe = 1.0
        r = sqrt(x² + 4y²) × (0.5 + prng()/3 × lobe × 0.3)
        v = frame - (int)r - 7

        if v < 0 and frame < 8+flash_len:
          fi = frame - 8
          if fi >= 0: cell = (flash_chars[fi], COL_INNER)
        else if v < wave_len:
          cell = (wave_chars[v], v < wave_len/2 ? COL_INNER : COL_WAVE)

    if frame > 6:
      i0 = frame - 6
      for each blob j:
        bx = blob.x × i0 × blob_speed
        by = blob.y × i0 × blob_speed × y_squash
        bz = blob.z × i0 × blob_speed

        if bz out of visible depth range: skip

        perspective projection:
          cx = cols/2 + (int)(bx × persp / (bz + persp))
          cy = rows/2 + (int)(by × persp / (bz + persp))

        if out of bounds: skip

        color = (bz > persp×0.8) ? COL_BLOB_F : (bz > -persp×0.4) ? COL_BLOB_M : COL_BLOB_N
        char  = (bz > persp×0.8) ? '.' : (bz > -persp×0.4) ? 'o' : '@'
        cells[cy×cols+cx] = {char, color}   ← overwrites wave if blob lands on it

    frame++
    if frame >= NUM_FRAMES: done = true

  blast_draw():
    erase()
    for each cell:
      if ch != 0: mvaddch(y, x, ch) with COLOR_PAIR(color)
    HUD
    wnoutrefresh + doupdate
```

---

## Function Breakdown

### prng() → double in [-1, 1]
Purpose: deterministic LCG random number
Steps:
1. `s = s * 1488248101 + 981577151` (LCG step)
2. return `(s % 65536 - 32768) / 32768.0`
Note: `s` is a static variable — same sequence every run (s starts at 1)

---

### blob_init_pool(blobs)
Purpose: place 800 blobs on unit sphere surface with slight jitter
Steps:
1. For i in 0..799:
   a. Generate (bx, by, bz) via `prng()`
   b. `br = sqrt(bx² + by² + bz²)` — distance from origin
   c. Normalize and add radial jitter: `blob.x = (bx/br) × (1.3 + 0.2×prng())`
   d. y is pre-squashed by 0.5: `blob.y = 0.5 × (by/br) × ...`
   e. Same for z

---

### blast_render_frame(blast)
Purpose: compute what to display for frame N
Steps:
1. Clear cells to zero
2. Offset: minx = -cols/2, miny = -rows/2 (origin at center)
3. For each screen cell:
   - **Frame 0**: `*` at center with COL_FLASH
   - **Frames 1-7** (disc phase): if `sqrt(x²+4y²) < frame×disc_speed` → `@` COL_FLASH
   - **Frames 8+** (wave phase):
     - Compute angle via atan2
     - Compute lobe = 1 + ripple×cos(petal_n×angle) if petal_n > 0, else 1.0
     - r = radial distance × (0.5 + prng()/3×lobe×0.3) — shaped + noisy
     - v = frame - r - 7 — how far behind the wave front this cell is
     - v < 0: cell is inside wave front → show flash_chars[frame-8] if still flashing
     - v in [0, wave_len): show wave_chars[v]
4. **Blob layer** (frame > 6):
   - Scale blobs by i0 = frame-6
   - Perspective project: `cx = cols/2 + bx×persp/(bz+persp)`
   - Color by z-depth zone
   - Write over any wave character at that cell

---

### blast_tick(blast) → bool (still running?)
Purpose: render one frame and advance
Steps:
1. If done: return false
2. blast_render_frame()
3. frame++
4. If frame >= NUM_FRAMES: done = true; return false
5. Return true

---

## Pseudocode — Core Loop

```
setup:
  theme = 0, shape = 0
  screen_init(theme)
  blast_init(cols, rows, theme, shape)
  frame_time = clock_ns()
  sim_accum = 0

main loop (while running):

  1. resize:
     if need_resize:
       screen_resize()
       blast_free()
       blast_init(new cols, rows, theme, shape)
       reset sim_accum, frame_time

  2. dt (capped at 100ms)

  3. physics ticks:
     sim_accum += dt
     while sim_accum >= tick_ns:
       if not blast.done:
         blast_tick()     ← render frame into cells[]
       else:
         advance theme = (theme+1) % THEME_COUNT
         advance shape = (shape+1) % SHAPE_COUNT
         blast_free()
         blast_init(cols, rows, theme, shape)
       sim_accum -= tick_ns

  4. FPS counter

  5. frame cap: sleep to 60fps

  6. draw:
     erase()
     blast_draw(stdscr)
     HUD: frame counter, theme name, shape name
     wnoutrefresh + doupdate

  7. input:
     q/ESC → quit
     ] / [ → sim_fps ± 5 (playback speed)
     r     → blast_free + blast_init (replay same shape/theme)
```

---

## Interactions Between Modules

```
App
 └── owns Blast

Blast
 ├── owns blobs[800] — computed once by blob_init_pool (deterministic LCG)
 └── owns cells[cols*rows] — rebuilt each frame by blast_render_frame

§3 color
 └── 7 ColorIDs, 6 BlastThemes
     color_theme_apply() updates init_pair for all 6 roles

§4 PRNG
 └── static LCG state — called by blob_init_pool AND blast_render_frame
     (wave noise also uses prng — same LCG sequence across both)
```

---

## The Three Visual Layers per Frame

```
Layer 1: Disc / Wave (cell-by-cell formula)
  frame 0:     center '*'
  frame 1-7:   expanding disc of '@'
  frame 8-149: blast wave sweeping outward
               inner fireball chars fade
               wave chars scroll out
               empty cells stay blank

Layer 2: Blob cloud (3D projection)
  800 blobs fly outward from origin
  perspective projection flattens z → 2D screen position
  z-depth → character size (. o @) and color (F/M/N)
  overwrites Layer 1 cells when they land on the same position

Layer 3: HUD (always on top)
  frame counter, theme, shape
```
