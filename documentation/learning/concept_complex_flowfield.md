# Pass 1 — complex_flowfield.c: Four-algorithm flow field visualiser with cosine palette

## Core Idea

A 2-D grid of flow angles is computed each tick by one of four swappable physics algorithms. Hundreds of tracer particles follow this field via bilinear interpolation, leaving short direction-glyph trails. The entire rendering path — particles and optional full-screen colormap — is colored through a cosine palette that maps flow angle to one of 16 pre-baked xterm-256 color pairs.

**What makes this "complex" compared to flowfield.c:**
- 4 field generators (1 in flowfield.c)
- 6 cosine-palette themes with 16 color pairs each (4 fixed themes, 8 pairs in flowfield.c)
- 3 background modes including a full-screen angle colormap
- Curl noise (divergence-free — no sources or sinks) vs. plain angle noise
- Biot-Savart vortex lattice as a second distinct physics mode
- Reset key `r` rewinds field time and respawns all particles

**Algorithm 0 — Curl noise:** Builds a scalar potential ψ from layered Perlin fBm, then takes the 2-D curl via central differences (`Vx=∂ψ/∂y`, `Vy=−∂ψ/∂x`). Divergence-free: particles orbit forever without clustering.

**Algorithm 1 — Vortex lattice:** 6 Biot-Savart point vortices on a slowly rotating ring, alternating CCW/CW. Each cell's velocity is the superposition of all 6 vortex contributions.

**Algorithm 2 — Sine lattice:** Superposition of two crossing sine/cosine wave pairs with time phase. Creates standing-wave interference patterns that evolve differently from organic noise.

**Algorithm 3 — Radial spiral:** Polar-coordinate field: pure CCW tangential rotation plus a time-pulsing radial component `W·sin(t)`. Particles spiral inward and outward like a breathing galaxy.

Six cosine-palette themes map the particle's movement angle to one of 16 complementary colors. Background mode `v=2` (colormap) paints every terminal cell by its local flow angle — a full-screen procedurally generated image that changes completely when you switch field type or theme.

## The Mental Model

Picture a grid of tiny arrows, each pointing in the direction the "wind" is blowing at that location. Particles are seeds dropped into the wind — they follow wherever the arrows point, one step per tick.

The four field types are four different ways of deciding what the arrows should say:
- **Curl noise:** Use calculus on a noise landscape to derive arrows that never point toward or away from any point. Like ocean currents — the water always goes around, never into a sink.
- **Vortex lattice:** Place 6 spinning tops on the screen. Each top pushes nearby particles sideways with strength proportional to 1/distance. Alternating clockwise and counterclockwise tops create complex interference.
- **Sine lattice:** Two perpendicular ripples crossing each other. Where the ripples reinforce, the arrows point strongly; where they cancel, they swirl in different directions.
- **Radial spiral:** Arrows that always point slightly counterclockwise — particles orbit the centre. Occasionally a pulse of outward force makes them spiral out, then the pulse reverses and they spiral back in.

The cosine palette: instead of picking 16 colors manually, the formula `a + b·cos(2π·(c·t+d))` traces a smooth curve through color space as `t` varies from 0 to 1. If you think of RGB as a 3-D space, the formula draws a Lissajous curve through it — a looping path that passes through many colors while maintaining perceptual smoothness and avoiding harsh jumps.

In colormap mode, every cell's color comes directly from its flow angle — the whole screen becomes a color picture of the invisible wind. Switching the field type changes the wind pattern and therefore changes the entire picture.

## Data Structures

### Field (§5)
```
float *angles;         — [rows * cols] flow angle per cell in radians
int    cols, rows;
float  time;           — animation time axis (all modes use this)
float  speed;          — time advance per tick (f/F keys)
int    field_type;     — 0=curl 1=vortex 2=sine 3=spiral

/* vortex lattice state (field_type=1 only) */
float  vort_cx[N_VORT]; — current vortex x-centres
float  vort_cy[N_VORT]; — current vortex y-centres
float  vort_ring_a;     — current ring orbital angle (radians)
```

### CosTheme (§3)
```
float a[3];   — bias      per R,G,B channel
float b[3];   — amplitude per R,G,B channel
float c[3];   — frequency per R,G,B channel
float d[3];   — phase     per R,G,B channel
const char *name;
```
16 ncurses pairs are pre-baked at theme-change time by sampling `t ∈ {0, 1/15, ..., 1}` and calling `cos_to_xterm256()` for each.

### Particle (§6)
```
float x, y;                — continuous position (cell units)
float speed;               — per-particle speed [0.65, 1.05] cells/tick
float angle;               — current movement direction (from field_sample)
int   trail_x[TRAIL_MAX];  — ring buffer of past integer positions
int   trail_y[TRAIL_MAX];
int   head;                — next-write index in ring buffer
int   trail_fill;          — how many slots are valid (ramps up to trail_len)
int   trail_len;           — active trail length (s/S keys)
int   life;                — countdown ticks; 0 → respawn
int   head_pair;           — ncurses pair from current angle → palette
bool  alive;
```

### Scene (§7)
```
Field    field;
Particle pool[PARTS_MAX];
int      n_particles;      — active pool size (+/- keys)
int      trail_len;        — shared trail length (s/S keys)
int      theme;            — 0=cosmic..5=mono
int      bg_mode;          — 0=blank 1=arrows 2=colormap
bool     paused;
```

## The Main Loop

1. **Resize check:** Rebuild angle grid and respawn particles at new dimensions.
2. **dt measurement:** Nanosecond delta clamped at 100ms.
3. **Sim accumulator:** `scene_tick()` called as many times as needed to stay at `sim_fps`.
4. **FPS counter:** Updated every 500ms.
5. **Frame cap:** Sleep to target ~60fps before drawing.
6. **Draw:** `erase()` then `scene_draw()` then HUD overlay.
7. **Present:** `wnoutrefresh + doupdate`.
8. **Input:** Keys modify scene state immediately.

## Non-Obvious Decisions

### Why divergence-free for curl noise?
Plain angle noise (`atan2(noise(x+offset,y), noise(x,y))`) has implicit sources and sinks where the two noise layers coincide. Particles drift toward low-noise regions over time, producing uneven density. The curl of a scalar potential is provably divergence-free: `div(curl(ψ)) = 0`. Particles stay uniformly distributed indefinitely — the visual density is consistent across the entire screen.

### Central differences for curl (not analytic derivatives)
Analytic derivatives of layered fBm would require differentiating `noise2()` with respect to both `x` and `y` while tracking the chain rule through all 3 octaves. Central differences (`(ψ(x+ε)−ψ(x−ε))/2ε`) approximate the same result with 4 noise evaluations per cell. `CURL_EPS=1.2` cells is large enough to avoid numerical noise from the discrete perm table but small enough to track fine spatial features.

### Cell aspect ratio correction in vortex lattice
Terminal characters are typically about twice as tall as wide. If you compute Biot-Savart distance in row/col units, a "circular" orbit appears elliptical on screen (tall and narrow). The fix: when computing the distance from particle to vortex, scale `dy` by `1/CELL_AR = 2` to convert to visual-pixel space. `CELL_AR=0.5` is the constant. The vortex centres themselves are stored in cell-space so they remain consistent with the angle grid.

### Why pre-bake palette vs. animate per-frame (like flocking.c)?
`flocking.c` calls `init_pair()` every N frames to continuously animate flock colors — the palette itself changes over time. For a flow field, continuously re-registering pairs would make the colormap flicker (every cell's color shifts even when particles aren't moving). Pre-baking once at theme-change time gives a stable palette: the color of a cell comes purely from the flow angle at that cell, not from the animation clock. This means the colormap is a faithful snapshot of the field topology.

### Proportional trail ramp vs. fixed characters
The trail char ramp `".,;+~*#"` is mapped proportionally across the trail length — `i * TRAIL_RAMP_N / fill` — so a 3-cell trail and an 18-cell trail both show the full gradient from `.` to `#`. If you used fixed character-per-age-step, short trails would only show the first 3 chars and look like dots rather than streamlines.

### `A_DIM`/`A_NORMAL`/`A_BOLD` instead of changing pairs for trail fade
The trail fade uses brightness modifiers (`A_DIM` for oldest quarter, `A_NORMAL` for middle, `A_BOLD` for head) rather than a fade to a different pair. All cells in the trail use the same `head_pair` (angle-based color). This means:
1. The full trail glows with the particle's current direction color, not a gradient toward an arbitrary "dim" color that may be a completely different hue.
2. When a particle changes direction and its `head_pair` updates, the entire trail shifts to the new color — a cohesive single-color streak.

## State Machines

```
RUNNING ──── space ────► PAUSED
   ▲                         │
   └──────── space ───────────┘

FIELD 0 ──── a ────► FIELD 1 ──── a ────► FIELD 2 ──── a ────► FIELD 3 ──── a ──► FIELD 0
(curl)               (vortex)             (sine)                (spiral)

THEME 0 ──── t ────► THEME 1 ──── t ────► ... ──── t ──► THEME 5 ──── t ──► THEME 0
(cosmic)             (ember)                            (mono)

BG MODE 0 ──── v ────► BG MODE 1 ──── v ────► BG MODE 2 ──── v ──► BG MODE 0
(blank)                (arrows)                (colormap)
```

Pressing `r` at any state respawns all particles and rewinds `field.time` to 0 — a complete visual reset while keeping the current field type, theme, and background mode.
Pressing `a` switches field type AND calls `scene_reset()` for a clean restart.

## From the Source

**Curl noise angle computation:**
```c
float dn = noise_fbm(x,     y + CURL_EPS, t, CURL_OCTAVES);
float ds = noise_fbm(x,     y - CURL_EPS, t, CURL_OCTAVES);
float de = noise_fbm(x+CURL_EPS, y,       t, CURL_OCTAVES);
float dw = noise_fbm(x-CURL_EPS, y,       t, CURL_OCTAVES);
float vx =  (dn - ds) / (2.f * CURL_EPS);    /* +∂ψ/∂y */
float vy = -(de - dw) / (2.f * CURL_EPS);    /* −∂ψ/∂x */
angle = atan2f(vy, vx);
```

**Vortex Biot-Savart with aspect correction:**
```c
for (int i = 0; i < N_VORT; i++) {
    float dx  = px - vort_cx[i];
    float dy  = (py - vort_cy[i]) / CELL_AR;    /* visual-pixel dy */
    float r2  = dx*dx + dy*dy + VORT_EPS;
    float str = (i % 2 == 0) ? VORT_STRENGTH : -VORT_STRENGTH;
    vx += str * (-dy) / r2;
    vy += str * ( dx) / r2;
}
angle = atan2f(vy, vx);
```

**Cosine palette → xterm-256:**
```c
float rf = a[0] + b[0] * cosf(2.f*M_PI*(c[0]*t + d[0]));
/* clamp to [0,1] */
int r5 = (int)(rf * 5.f + 0.5f);   /* nearest of 6 cube levels */
int g5 = (int)(gf * 5.f + 0.5f);
int b5 = (int)(bf * 5.f + 0.5f);
return 16 + 36*r5 + 6*g5 + b5;     /* xterm-256 cube index */
```

**Particle head pair update (per tick):**
```c
float a = p->angle;
if (a < 0.f) a += 2.f * (float)M_PI;
p->head_pair = CP_BASE + (int)(a / (2.f*M_PI) * N_PAIRS) % N_PAIRS;
```

## Key Constants and What Tuning Them Does

| Constant | Default | Effect if changed |
|---|---|---|
| `N_PAIRS` | 16 | More pairs → smoother angle-color gradient; fewer → coarser hue steps |
| `CURL_EPS` | 1.2 | Larger → coarser curl estimation, misses small features; smaller → noisy on discrete grid |
| `CURL_OCTAVES` | 3 | More → finer fractal detail, more compute per cell |
| `N_VORT` | 6 | More vortices → richer interference; fewer → simpler pattern |
| `VORT_RING_FRAC` | 0.28 | Fraction of screen used for vortex ring radius; larger = more spread out |
| `VORT_ORB_SPD` | 0.014 | Ring rotation speed; faster = faster evolution of vortex pattern |
| `VORT_STRENGTH` | 3.0 | Biot-Savart magnitude; larger = faster particle orbiting |
| `VORT_EPS` | 5.0 | Singularity softening radius; larger = smoother near vortex centres |
| `SINE_XFREQ/YFREQ` | 0.055/0.095 | Spatial frequency of sine waves; larger = tighter interference fringes |
| `SPIRAL_RADIAL_W` | 0.65 | Radial pulse weight; 0 = pure CCW vortex; 1 = strong in/out breathing |
| `CELL_AR` | 0.5 | Cell aspect ratio for vortex distance; adjust if orbits look non-circular |
| `PARTS_DEFAULT` | 400 | Starting particle count (+/- keys adjust at runtime) |
| `TRAIL_LEN_DEFAULT` | 18 | Starting trail length (s/S keys adjust at runtime) |
| `FIELD_SPD_DEFAULT` | 0.006 | Field evolution rate; f/F keys adjust at runtime |

## Open Questions for Pass 3

- The colormap draws every cell with the local field angle. If bg_mode=2 and there are 400 particles, every frame draws `cols*rows` background cells plus `400*trail_len` particle trail cells. For a 200×50 terminal that is 10000 + 7200 = 17200 `mvwaddch` calls per frame. At what particle count / terminal size does this become the framerate bottleneck vs. the field recomputation?
- The vortex lattice updates `vort_cx/cy[]` every tick (O(N_VORT)). The field recomputation visits every cell and sums N_VORT contributions (O(W×H×N_VORT)). For N_VORT=6 and a 200×50 grid that is 60000 operations. Could you precompute a velocity grid per vortex and sum them, rather than recomputing per-cell? What would the memory/compute tradeoff be?
- `field_sample()` uses bilinear interpolation — the same 4-cell average as `flowfield.c`. For the radial spiral field, the angles near the screen centre can change rapidly. Would bicubic (16-cell) interpolation make particle motion smoother near the centre? What would the performance cost be?
- The sine lattice uses `atan2(vy, vx)` to produce an angle. But `sin+sin` and `cos+cos` produce values in `[−2, 2]`, not unit vectors. Does this matter? What if you normalised `(vx, vy)` before `atan2`?
- The `r` key sets `field.time = 0` and respawns particles. After a reset, the field re-computes from the same noise at `t=0` as the initial startup. Does that mean two resets always give the same visual? What would you change to get a truly fresh random start each reset?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `field.angles[rows*cols]` | `float[]` | ~40 KB | flow angle per cell |
| `pool[PARTS_MAX]` | `Particle[]` | ~45 KB | particle pool |
| `perm[512]` | `uint8_t[]` | 512 B | Perlin permutation table |

# Pass 2 — complex_flowfield: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 presets | All named constants grouped by subsystem: loop, particles, trail, field speed, curl, vortex, sine, spiral |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 cosine pal | `CosTheme` structs; `cos_to_xterm256()`; `color_apply_theme()`; `angle_to_pair()` |
| §4 noise | `noise_init()`, `noise2()`, `noise_fbm()` — layered Perlin |
| §5 field | `field_alloc/free/resize/tick/sample`; per-mode compute functions; `angle_to_char()` |
| §6 particle | `particle_spawn()`, `particle_tick()`, `particle_draw()` |
| §7 scene | `Scene` struct; `scene_init/resize/reset/tick/draw()` |
| §8 screen | ncurses init/draw/HUD/present |
| §9 app | Signal handlers, main loop, key handler |

---

## Data Flow Diagram

```
scene_tick():
  field_tick(field):
    field.time += field.speed
    advance vortex ring: vort_ring_a += VORT_ORB_SPD → recompute vort_cx/cy[]

    for each cell (x, y):
      dispatch on field_type:
        0 (curl):   angle = field_curl_angle(x, y, time)
        1 (vortex): angle = field_vortex_angle(field, x, y)
        2 (sine):   angle = field_sine_angle(x, y, time)
        3 (spiral): angle = field_spiral_angle(field, x, y, time)
      field.angles[y*cols+x] = angle

  for each particle i in pool:
    if not alive: spawn at random position
    particle_tick(p, field):
      push (int)x, (int)y to ring buffer
      angle = field_sample(field, p->x, p->y)   ← bilinear interp
      p->x += cos(angle) * speed
      p->y += sin(angle) * speed
      wrap x, y toroidally
      p->head_pair = angle_to_pair(angle)        ← cosine palette pair
      p->life--

── field_sample bilinear ─────────────────────────────────────────────────

  x0 = floor(px), y0 = floor(py), x1=x0+1, y1=y0+1  (all clamped)
  tx = px - x0,  ty = py - y0
  return lerp(lerp(angles[y0,x0], angles[y0,x1], tx),
              lerp(angles[y1,x0], angles[y1,x1], tx), ty)

── scene_draw ────────────────────────────────────────────────────────────

  erase()  ← blank canvas for all bg_modes (bg is handled by drawing chars)

  if bg_mode > 0:
    for each cell (x, y):
      angle = field.angles[y*cols+x]
      cp    = angle_to_pair(angle)
      ch    = angle_to_char(angle)    ← >^<v/\
      if bg_mode == 1: draw ch with COLOR_PAIR(cp) | A_DIM
      if bg_mode == 2: draw ch with COLOR_PAIR(cp) | A_NORMAL

  for each particle i: particle_draw(p)
    for each trail cell i in ring (i=0 oldest → fill-1 newest):
      ch = (i == fill-1) ? direction_char(angle) : trail_ramp[proportional_index]
      brightness = A_DIM (oldest quarter) / A_NORMAL (middle) / A_BOLD (head)
      draw ch with COLOR_PAIR(p->head_pair) | brightness
```

---

## Function Breakdown

### cos_to_xterm256(theme, t) → int
Purpose: evaluate cosine palette at t ∈ [0,1] → xterm-256 cube index
Steps:
1. For each RGB channel i: `v[i] = a[i] + b[i] * cos(2π * (c[i]*t + d[i]))`
2. Clamp to [0,1]
3. Quantize to 0-5: `n[i] = (int)(v[i] * 5.f + 0.5f)`
4. Return `16 + 36*n[0] + 6*n[1] + n[2]`

---

### color_apply_theme(theme)
Purpose: register 16 ncurses color pairs from cosine palette
Steps:
1. For i in 0..15: t = i/15.0
2. fg = cos_to_xterm256(theme, t)
3. init_pair(CP_BASE + i, fg, COLOR_BLACK)
Fallback for 8-color: use k_fallback[theme][i] table

---

### angle_to_pair(angle) → int
Purpose: map flow angle ∈ (−π, π] to pair CP_BASE..CP_BASE+15
Steps:
1. Normalise: if angle < 0: angle += 2π
2. idx = (int)(angle / 2π × N_PAIRS) % N_PAIRS
3. Return CP_BASE + idx

---

### field_curl_angle(x, y, t) → float
Purpose: divergence-free flow angle from curl of scalar potential
Steps:
1. dn = noise_fbm(x, y+CURL_EPS, t, octaves)
2. ds = noise_fbm(x, y-CURL_EPS, t, octaves)
3. de = noise_fbm(x+CURL_EPS, y, t, octaves)
4. dw = noise_fbm(x-CURL_EPS, y, t, octaves)
5. vx = (dn-ds)/(2*eps);  vy = -(de-dw)/(2*eps)
6. Return atan2f(vy, vx)

---

### field_vortex_angle(field, x, y) → float
Purpose: Biot-Savart superposition from N_VORT vortices
Steps:
1. vx = 0, vy = 0
2. For each vortex i:
   - dx = x - vort_cx[i]
   - dy = (y - vort_cy[i]) / CELL_AR     ← visual-pixel correction
   - r2 = dx*dx + dy*dy + VORT_EPS
   - str = (i%2==0) ? +VORT_STRENGTH : -VORT_STRENGTH
   - vx += str * (-dy) / r2
   - vy += str * ( dx) / r2
3. Return atan2f(vy, vx)

---

### field_sine_angle(x, y, t) → float
Purpose: wave interference pattern
Steps:
1. vx = sin(x*SINE_XFREQ + t) + sin(y*SINE_YFREQ - t*0.7)
2. vy = cos(x*SINE_XFREQ - t*0.5) + cos(y*SINE_YFREQ + t*0.3)
3. Return atan2f(vy, vx)

---

### field_spiral_angle(field, x, y, t) → float
Purpose: breathing galaxy — tangential + pulsing radial
Steps:
1. dx = x - cols*0.5;  dy = (y - rows*0.5) / CELL_AR
2. theta = atan2f(dy, dx)
3. pulse = SPIRAL_RADIAL_W * sinf(t * 0.8f)
4. vx = -sin(theta) + pulse * cos(theta)
5. vy =  cos(theta) + pulse * sin(theta)
6. Return atan2f(vy, vx)

---

### particle_draw(p, window, cols, rows)
Purpose: draw trail ring buffer with brightness ramp and direction head
Steps:
1. For i = 0 to trail_fill-1 (oldest to newest):
   - ti = (head - fill + i + trail_len*4) % trail_len
   - tx, ty = trail_x[ti], trail_y[ti]
   - if out of bounds: skip
   - is_head = (i == fill-1)
   - ch = direction_char if is_head, else trail_ramp[i*RAMP_N/fill]
   - bright = A_BOLD if head, A_NORMAL if i >= fill*3/4, A_DIM otherwise
   - draw ch with COLOR_PAIR(head_pair) | bright

---

## Interactions Between Modules

```
App
 └── main loop: sim_accum → scene_tick → field_tick → [curl|vortex|sine|spiral]
                                       → particle_tick (bilinear sample + move)
                render → scene_draw → background (bg_mode) + particle trails
                input → algo/theme/bgmode/speed/count adjustments

§3 cosine palette
 ├── CosTheme k_themes[6]      — a,b,c,d parameter vectors per theme
 ├── cos_to_xterm256()         — palette at t → xterm-256 index
 ├── color_apply_theme()       — pre-bake 16 pairs at theme change
 └── angle_to_pair()           — flow angle → pair index (runtime)

Field
 ├── angles[rows*cols]          — computed each tick by active mode
 └── vort_cx/cy[], vort_ring_a  — vortex orbital state (mode 1 only)

Particle
 ├── trail_x/y[TRAIL_MAX]       — ring buffer of past positions
 └── head_pair                  — updated from angle each tick via angle_to_pair()
```
