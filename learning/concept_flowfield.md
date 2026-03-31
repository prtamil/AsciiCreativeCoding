# Pass 1 — flowfield: ASCII Flow Field Simulation

## Core Idea

A 2D vector field computed from layered Perlin noise drives hundreds of tracer particles that leave fading trails. The noise evolves slowly over time, so the field and the particle paths change continuously — producing motion that looks like wind, smoke, or ocean currents rendered in ASCII arrows and trail characters.

---

## The Mental Model

Imagine every terminal cell has a tiny arrow showing which direction the "wind" is blowing at that point. The arrows change slowly over time (the noise field evolves). Hundreds of tracer particles follow the wind, leaving fading trails behind them: `.,+~*` from dim-to-bright, with a directional arrow character at the head.

The result: a living, flowing, swirling pattern that looks organic because it is driven by multi-scale Perlin noise.

---

## Data Structures

### `Field` struct
| Field | Meaning |
|---|---|
| `angles[rows*cols]` | Flow angle in radians for each cell |
| `cols, rows` | Grid dimensions |
| `time` | Current noise time axis value |
| `speed` | How fast time advances per tick |

### `Particle` struct
| Field | Meaning |
|---|---|
| `x, y` | Continuous position (not snapped to cell) |
| `speed` | Per-particle speed in [0.5, 1.3] cells/tick |
| `angle` | Current movement direction (from field sample) |
| `trail_x[], trail_y[]` | Ring buffer of MAX_TRAIL=20 past positions |
| `head` | Ring buffer write index |
| `trail_fill` | How many trail slots are used (ramps up from 0) |
| `trail_len` | Active trail length (configurable, ≤ MAX_TRAIL) |
| `life` | Countdown to auto-respawn |
| `color_pair` | Color pair index 1..8 (updated each tick in rainbow mode) |
| `alive` | False = slot free for respawn |

### `Scene` struct
| Field | Meaning |
|---|---|
| `field` | The vector field |
| `pool[PARTICLES_MAX]` | All particle slots |
| `n_particles` | How many are active |
| `trail_len` | Current trail length setting |
| `theme` | 0=rainbow, 1=cyan, 2=green, 3=white |
| `show_field` | Toggle background arrow display |
| `paused` | Simulation paused flag |

---

## The Main Loop

Standard fixed-timestep accumulator:
1. `field_tick()` — advance noise time, recompute all cell angles from 3-octave Perlin
2. For each live particle: `particle_tick()` — sample field, move, wrap, age
3. Auto-respawn dead particles
4. `scene_draw()` — optionally draw background arrows, then all particle trails

---

## Non-Obvious Decisions

### 3-octave fBm instead of single Perlin
A single noise octave gives very smooth, large-scale flow — it looks bland. Three octaves (1.0 + 0.5 + 0.25 at frequencies 1×/2×/4×) add fine-detail swirls layered over the large-scale flow. This creates the "multi-scale wind" appearance.

### Two independent noise channels → `atan2`
The X and Y components of the flow vector come from two completely separate noise evaluations (spatially offset by 100.3/200.7). If you just used one noise value as an angle directly, the result would have a bias toward certain directions. Using `atan2(ny, nx)` ensures all directions are equally likely.

### Bilinear field sampling
Particles move at continuous floating-point positions. The field is on a discrete cell grid. Simple nearest-neighbor would cause particles to jump as they cross cell boundaries. Bilinear interpolation between the 4 surrounding cells gives smooth continuous direction.

### Ring buffer trail + proportional ramp
The trail uses a circular buffer (head index advances, wraps at trail_len). The visual ramp `.,+~*` is mapped **proportionally** across the actual filled length — a 3-slot trail and a 20-slot trail both show the full gradient. The old naive approach (raw index into ramp) made long trails look mostly dim, like a caterpillar.

### Per-particle speed jitter
All particles at the same speed would synchronize — they'd pile up and thin out in waves. Each particle gets a random speed in [0.5, 1.3]. This prevents lock-step behavior and makes the flow look natural.

### Toroidal wrapping
Particles wrap around screen edges (left↔right, top↔bottom) rather than bouncing or dying at the boundary. This makes the flow look infinite and avoids the density thinning you'd get near walls.

### Direction-aware head character
The head of each trail shows an arrow matching the particle's current movement direction (`- / | \`). This makes each particle visually read as "going somewhere" rather than being a generic dot.

---

## State Machines

No state machine. Continuous simulation. Only boolean: `paused`. Dead particles auto-respawn immediately at a random position.

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of increasing |
|---|---|---|
| `PARTICLES_DEFAULT` | 300 | More streaks, denser field coverage |
| `TRAIL_LEN_DEFAULT` | 14 | Longer streaks per particle |
| `FIELD_SPD_DEFAULT` | 0.008 | Faster field evolution → more turbulent motion |
| `PARTICLE_SPEED` | 0.9 | Faster particles → longer streaks per second |
| `NOISE_SCALE_X/Y` | 0.04/0.07 | Smaller = smoother large-scale flow |
| `LIFE_BASE` | 100 | Longer particle lifetime before respawn |

---

## Open Questions for Pass 3

1. Remove bilinear sampling — use nearest-neighbor — does it look choppy?
2. Change wrap to bounce — how does particle behavior change at edges?
3. What happens if you use only 1 octave of noise vs 3 octaves? (add octaves one at a time)
4. What if all particles had the same speed? Does lock-step actually appear?
5. Visualize the field arrows while particles move — does the flow direction match the particle paths?
6. Try making field speed proportional to local noise gradient — faster flow in high-gradient zones.

---

# Pass 2 — flowfield: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | PARTICLES_MIN/DEFAULT/MAX=50/300/800, TRAIL_LEN MIN/DEFAULT/MAX=3/14/20, THEME_COUNT=4, FIELD_SPD, PARTICLE_SPEED, NOISE_SCALE_X/Y, LIFE_BASE/JITTER |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 8 pairs; 4 themes (rainbow/cyan/green/white); `color_apply_theme()`; `angle_to_pair()` |
| §4 noise | `perm[512]`; `fade()`; `grad()`; `noise2()`; `noise_field_angle()` — 3-octave fBm |
| §5 field | `Field` struct; `field_alloc/free/resize/tick/sample`; `angle_to_char()` |
| §6 particle | `Particle` struct; `particle_spawn/tick/draw` |
| §7 scene | `Scene` owns Field + Particle pool; `scene_init/tick/draw` |
| §8 screen | ncurses init/resize/present/HUD |
| §9 app | Main dt loop, input, signals |

---

## Data Flow Diagram

```
noise_init():
  p[256] = {0,1,...,255}
  Fisher-Yates shuffle p[]
  perm[i] = p[i & 255] for i in 0..511

noise_field_angle(x, y, t):
  nx = ny = 0; amp = 1.0; freq = 1.0
  for oct in 0..2:
    nx += noise2(x*NOISE_SCALE_X*freq + t,
                 y*NOISE_SCALE_Y*freq + t*0.7) * amp
    ny += noise2(x*NOISE_SCALE_X*freq + ox + t*1.1,   ← ox=100.3
                 y*NOISE_SCALE_Y*freq + oy + t*0.5) * amp  ← oy=200.7
    amp *= 0.5; freq *= 2.0
  return atan2(ny, nx)   → angle in (-π, π)

field_tick(field):
  field.time += field.speed
  for y in 0..rows-1:
    for x in 0..cols-1:
      field.angles[y*cols+x] = noise_field_angle(x, y, field.time)

particle_tick(p, field, cols, rows, theme):
  if not alive: return
  trail_x[head] = (int)x; trail_y[head] = (int)y
  head = (head + 1) % trail_len
  trail_fill = min(trail_fill+1, trail_len)

  angle = field_sample(field, x, y)   ← bilinear
  p.angle = angle
  x += cos(angle) * speed
  y += sin(angle) * speed

  // wrap toroidally
  if x < 0: x += cols
  if x >= cols: x -= cols
  if y < 0: y += rows
  if y >= rows: y -= rows

  if theme == 0: color_pair = angle_to_pair(angle)

  life--
  if life <= 0: alive = false

particle_draw(p, w, cols, rows):
  ramp = ".,+~*" (5 chars: dim→bright)
  dir  = {'-','/',  '|', '\\', '-', '/', '|', '\\'} (8-direction arrows)

  for i in 0..trail_fill-1:
    ti = (head - trail_fill + i + trail_len*2) % trail_len
    tx = trail_x[ti]; ty = trail_y[ti]
    if out of bounds: continue

    if is_head (i == trail_fill-1):
      di = (int)(angle / 2π * 8 + 0.5) % 8
      ch = dir[di]
    else:
      ri = (i * (RAMP_N-1)) / (trail_fill-1)  ← proportional ramp
      ch = ramp[ri]

    // proportional color fade
    cp = color_pair - proportional_offset_by_i
    if cp < 1: cp = 1

    attr = COLOR_PAIR(cp) | (is_head ? A_BOLD : 0)
    mvwaddch(ty, tx, ch)
```

---

## Function Breakdown

### noise_init()
Purpose: build permutation table for Perlin noise
Steps:
1. Fill p[256] = {0..255}
2. Fisher-Yates shuffle p[]
3. Double it: `perm[i] = p[i & 255]` for i in 0..511 (avoids modulo in lookups)

---

### noise2(x, y) → float in [-1, 1]
Purpose: single-octave 2D Perlin noise
Steps:
1. Integer cell: xi = floor(x) & 255; yi = floor(y) & 255
2. Fractional offset: xf = x - floor(x); yf = y - floor(y)
3. Smoothstep: u = fade(xf); v = fade(yf)
4. Hash 4 corners: aa=perm[perm[xi]+yi], ab=..., ba=..., bb=...
5. Bilinear blend of grad() values with u, v

---

### noise_field_angle(x, y, t) → float angle
Purpose: 3-octave fBm → flow direction
Steps:
1. nx = ny = 0; amp = 1; freq = 1
2. For oct in 0..2:
   - nx += noise2(x * NOISE_SCALE_X * freq + t, ...) * amp
   - ny += noise2(different offset for independence) * amp
   - amp *= 0.5; freq *= 2
3. return atan2(ny, nx)

---

### field_sample(field, px, py) → float angle
Purpose: bilinear interpolation of field at continuous position
Steps:
1. x0 = floor(px); y0 = floor(py); x1 = x0+1; y1 = y0+1
2. Clamp all to [0, cols/rows - 1]
3. tx = px - floor(px); ty = py - floor(py)
4. Bilinear blend of 4 surrounding angles:
   lerp(lerp(a00, a10, tx), lerp(a01, a11, tx), ty)

---

### particle_spawn(p, cols, rows, trail_len)
Purpose: place particle at random position with random speed/life
Steps:
1. x = rand() % cols; y = rand() % rows
2. speed = PARTICLE_SPEED ± PARTICLE_SPD_JITTER/2 (uniform random)
3. head = trail_fill = 0; trail_len = given
4. life = LIFE_BASE + rand() % LIFE_JITTER
5. color_pair = 1 + rand() % N_PAIRS
6. alive = true
7. Zero-fill trail_x/trail_y to (int)x, (int)y (prevents garbage on first draw)

---

## Pseudocode — Core Loop

```
setup:
  srand(time(NULL))
  screen_init()
  noise_init()   ← must call before scene_init
  scene_init(cols, rows)
  frame_time = clock_ns()
  sim_accum = 0

main loop (while running):

  1. resize:
     if need_resize:
       screen_resize()
       scene_resize(new cols, rows)   ← field_resize + respawn particles
       reset sim_accum, frame_time

  2. dt (capped at 100ms)

  3. physics ticks:
     sim_accum += dt
     while sim_accum >= tick_ns:
       scene_tick(cols, rows)         ← field_tick + particle_tick all
       sim_accum -= tick_ns

  4. FPS counter

  5. frame cap: sleep to 60fps

  6. draw:
     erase()
     scene_draw(stdscr, cols, rows)   ← optional field arrows, then trails
     HUD: fps, n_particles, trail_len, theme, sim_fps
     wnoutrefresh + doupdate

  7. input:
     q/ESC   → quit
     space   → toggle paused
     r       → respawn all particles
     t       → theme = (theme+1) % THEME_COUNT; color_apply_theme()
     a       → toggle show_field (background arrows)
     ] / [   → sim_fps ± SIM_FPS_STEP
     f / F   → field.speed *= / /= FIELD_SPD_STEP (clamped)
     s / S   → trail_len ± 1 (clamped to TRAIL_LEN_MIN/MAX)
     + / -   → n_particles ± PARTICLES_STEP (clamped)
```

---

## Interactions Between Modules

```
App
 └── owns Scene (Field + particle pool)

Scene
 ├── field: recomputed each tick from noise
 │    └── field_sample() called per particle per tick
 └── pool[PARTICLES_MAX]:
      each particle calls field_sample, moves, wraps
      dead particles respawn immediately in scene_tick

§4 noise
 └── noise_init() randomizes perm[] at startup (srand seeded from time)
     noise_field_angle() called per cell per field_tick
     noise2() called 6 times per cell (3 oct × 2 channels)

§3 color
 └── angle_to_pair() called per particle per tick (rainbow mode only)
     color_apply_theme() called once at init and on 't' keypress
```

---

## Perlin Noise Implementation Detail

```
noise2(x, y):
  xi  = floor(x) & 255   ← integer grid cell
  xf  = x - floor(x)     ← fractional offset [0,1)
  u   = fade(xf) = xf²(3 - 2xf)   ← smoothstep

  aa = perm[perm[xi  ] + yi  ]     ← hash corner (xi,   yi)
  ba = perm[perm[xi+1] + yi  ]     ← hash corner (xi+1, yi)
  ab = perm[perm[xi  ] + yi+1]
  bb = perm[perm[xi+1] + yi+1]

  grad(h, x, y):
    H = h & 3
    u = (H < 2) ? x : y
    v = (H < 2) ? y : x
    return (H&1 ? -u : u) + (H&2 ? -v : v)
    → maps hash to one of {+x+y, -x+y, +x-y, -x-y} or rotated versions

  return lerp(
    lerp(grad(aa, xf,   yf  ), grad(ba, xf-1, yf  ), u),
    lerp(grad(ab, xf,   yf-1), grad(bb, xf-1, yf-1), u),
    v)
```

---

## Trail Ring Buffer Detail

```
Ring buffer: trail_x[MAX_TRAIL], trail_y[MAX_TRAIL]
head: points to NEXT write slot (not current)

Each tick:
  Write current position to trail_x[head], trail_y[head]
  head = (head + 1) % trail_len
  trail_fill = min(trail_fill + 1, trail_len)

To read in order (oldest → newest):
  for i in 0..trail_fill-1:
    ti = (head - trail_fill + i + trail_len*2) % trail_len
                                   ↑ +2*trail_len prevents negative modulo

  i=0 → oldest (dimmest)
  i=trail_fill-1 → newest (head, brightest, direction arrow)
```
