# Pass 1 — fireworks_rain.c: Fireworks with Matrix-Rain Arc Trails

## Core Idea

Takes the fireworks rocket state machine from fireworks.c and replaces each simple spark with a `MatrixParticle`: a spark that carries a TRAIL_LEN=16 position history. Every tick, the particle's current position is pushed into the trail history before physics advance. At draw time, the trail history is rendered as a chain of shimmering matrix-rain characters that follows the exact arc the particle traces through the air — the familiar downward parabola of a fading firework ember, but made of matrix glyphs.

---

## The Mental Model

Think of each spark as leaving a glowing snake behind it as it flies. The snake is made of random ASCII characters that shimmer and change every tick (like matrix rain). The snake's head is white and bold; it fades through the spark's assigned color (bold → normal → dim) toward its tail. When the spark is dying (life < 0.25), the whole snake goes dim.

The rockets themselves are unchanged from fireworks.c: they rise, explode at the apex, and respawn after a fuse delay.

---

## The Physics Split: ROCKET_DRAG vs GRAVITY

The most important design decision: two separate constants control gravity at different stages.

### Why they must be different

A single GRAVITY constant causes a conflict:
- **High value (9.8):** rockets decelerate fast, explode at varied heights throughout the screen — good. But particles fall too fast, all arcs curve sharply downward → triangular burst shape — bad.
- **Low value (3.5):** particles spread in all directions, near-spherical burst — good. But rockets decelerate so slowly they always reach the top of the screen before exploding → all bursts are in one strip at the top — bad.

### The solution

```
ROCKET_DRAG = 9.8f   applied as: r->vy += ROCKET_DRAG * dt_sec * 0.5
GRAVITY     = 4.0f   applied as: g = GRAVITY * (0.8 + rand()*0.4)
                                  p->vy += g * dt_sec
```

`ROCKET_DRAG` controls apex height (stays at 9.8, rockets distribute across screen).
`GRAVITY` controls particle arcs (4.0 gives upward sparks enough momentum to visibly rise before curving down).

### Per-particle gravity variance

Each particle gets a slightly different effective gravity:
```
g = GRAVITY * (0.8 + rand()/RAND_MAX * 0.4)   →  g ∈ [3.2, 4.8]
```
Sparks launched at the same angle but with different `g` values diverge over time, giving the burst an organic, non-uniform look. Without this, all parabolas at the same angle are identical — the burst looks grid-like.

### No vertical squash

Original fireworks.c initialises `vy = sin(angle) * speed * 0.5`. The 0.5 squash was a visual correction for non-square terminal cells, but it also made the burst an ellipse compressed vertically — upward sparks had half the launch speed of horizontal ones. Removing it gives all sparks equal launch speed in all directions, producing a true circular burst.

---

## Data Structures

### `MatrixParticle` struct

| Field | Meaning |
|---|---|
| `cx, cy` | Live head position (float, updated by physics each tick) |
| `vx, vy` | Velocity; vy increased by per-particle gravity each tick |
| `trail_x[16], trail_y[16]` | Position history: [0] = one tick ago, [15] = oldest |
| `trail_fill` | How many trail slots are valid (ramps 0→16 as particle moves) |
| `life` | 0.6–1.0 at birth, decremented by decay each tick |
| `decay` | Life drain rate (randomised per particle) |
| `cache[16]` | Shimmer chars: ~75% replaced each tick |
| `color` | ColorPair (1–7); remapped by active theme |
| `active` | False when life <= 0 |

### Trail history update (each tick)

```
1. Shift history:  trail[15..1] = trail[14..0]
2. Store current:  trail[0] = (cx, cy)
3. trail_fill = min(trail_fill + 1, TRAIL_LEN)
4. Physics:        cx += vx*dt*8; cy += vy*dt*8; vy += g*dt
```

The shift happens BEFORE physics advance, so `trail[0]` always holds the previous tick's position (one tick older than the current head).

### `Rocket` struct

Identical to fireworks.c except `particles[]` is now `MatrixParticle[]` instead of `Particle[]`.

---

## Draw Algorithm

### Draw order: oldest first, head last

```
for i = trail_fill-1 down to 0:       ← oldest (dimmest) to newest (brightest)
  draw cache[i] at (trail_x[i], trail_y[i]) with trail_attr(i, color, fading)

draw cache[0] at (cx, cy) with CP_WHITE | A_BOLD   ← live head, always last
```

Drawing oldest first ensures the brightest elements overwrite the dimmer ones at any terminal cell where multiple trail positions round to the same (col, row). This is important near the burst origin where many sparks are tightly packed.

### Brightness mapping

| Position | Condition | Attribute |
|---|---|---|
| Live head (cx, cy) | life >= 0.25 | CP_WHITE \| A_BOLD |
| Live head (cx, cy) | life < 0.25 (dying) | particle color \| A_DIM |
| trail[0], [1], [2] | normal | particle color \| A_BOLD |
| trail[3..TRAIL_LEN/2-1] | normal | particle color |
| trail[TRAIL_LEN/2..] | normal | particle color \| A_DIM |
| any trail slot | fading (life < 0.25) | particle color \| A_DIM |

---

## Theme System

5 themes remap the 7 spark ColorPairs (1–7). CP_WHITE (pair 8, trail head) is always white regardless of theme.

| Theme | What it produces |
|---|---|
| vivid | Classic multi-color fireworks (red/orange/yellow/green/cyan/blue/magenta) |
| matrix | All sparks in green shades — trails look like Matrix rain arcs |
| fire | Reds, oranges, yellows — every burst is a flame arc |
| ice | Blues and cyans — cold crystalline trails |
| plasma | Purples and magentas — electric neon arcs |

`theme_apply(idx)` calls `init_pair(1..7, fg[i], COLOR_BLACK)` and re-registers CP_WHITE. The rest of the code (color_rand, trail_attr) is unchanged — only the pair definitions change.

---

## Non-Obvious Decisions

### Why trail history instead of drawing from velocity direction?

A direction-based trail (compute position backward along current velocity) would not follow the arc. Gravity continuously bends the trajectory, so the real trail is curved, not straight. Only a position history records the actual path the spark has traveled.

### trail_fill ramps up from 0

At burst time, `trail_fill = 0`. The first tick pushes one entry, the second pushes two, and so on until `trail_fill = TRAIL_LEN` after 16 ticks. This means the trail grows organically from the burst point outward — sparks start as dots and grow into full-length snakes as they travel.

### cache[i] for trail[i] — not assigned per position

The cache is a flat array of shimmer chars. `cache[i]` is used for `trail[i]`. There is no position-specific character assignment — any character at any slot is purely random and changes every tick. The visual identity of a trail position comes entirely from its brightness, not its character.

### Rockets memory in global BSS

`App` contains `Show` contains `rockets[MAX_ROCKETS]` each containing `MatrixParticle[72]`. Each MatrixParticle is ~180 bytes. Total: 16 × 72 × 180 ≈ 207 KB in the global `g_app`. This is in BSS (zero-initialized static storage), not the stack — no stack overflow risk.

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|---|---|---|
| `TRAIL_LEN` | 16 | Longer = more history, longer snake trails |
| `PARTICLES_PER_BURST` | 72 | More sparks per explosion; more memory and CPU |
| `ROCKET_DRAG` | 9.8 rows/s² | Higher = rockets explode lower and closer together |
| `GRAVITY` | 4.0 rows/s² | Higher = triangle-shaped burst; lower = all sparks float up |
| `GRAVITY` variance | ±20% | Wider range → more divergence between same-angle sparks |
| `life` range | 0.6–1.0 | Shorter max life → burst fades faster |
| `decay` range | 0.025–0.060 | Higher = faster fade per particle |

---

## Open Questions for Pass 3

1. The trail uses a simple shift-push. What if you used a circular buffer instead? What is the performance difference at TRAIL_LEN=64?
2. Can you add a "gravity = 0" mode where sparks travel in straight lines (trails become rays)?
3. What changes if you sort the particles by distance from origin and draw far ones first (painter's algorithm)? Does it matter for ASCII?
4. The per-particle gravity variance uses `rand()` every tick. Could you instead assign a fixed `g_scale` per particle at burst time? Would the result look different?
5. At PARTICLES_PER_BURST = 72, the App struct is ~207 KB. What is the practical stack limit on your platform, and what would happen if Show were stack-allocated instead of in BSS?

---

# Pass 2 — fireworks_rain.c: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | SIM_FPS, ROCKETS_MIN/DEFAULT/MAX=5/16, PARTICLES_PER_BURST=72, TRAIL_LEN=16, ROCKET_DRAG=9.8, GRAVITY=4.0, LAUNCH_SPEED_MIN/MAX |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | `ColorPair` enum (CP_RED..CP_WHITE=8); 5 `Theme` structs; `theme_apply()`, `color_rand()`, `trail_attr()` |
| §4 matrix particle | `MatrixParticle` struct; `mparticle_burst()`, `mparticle_tick()`, `mparticle_draw()` |
| §5 rocket | `Rocket` struct + state machine; `rocket_launch()`, `rocket_tick()`, `rocket_draw()` |
| §6 show | `Show` owns rocket pool; `show_init()`, `show_tick()`, `show_draw()` |
| §7 screen | ncurses init/resize/hud/present |
| §8 app | Main dt loop, input, signals |

---

## Data Flow Diagram

```
show_tick(dt_sec):
  for each active rocket:
    rocket_tick(rocket, dt_sec)
      │
      ├── IDLE:  fuse--; if 0 → rocket_launch()
      │
      ├── RISING:
      │     y += vy * dt * 6
      │     vy += ROCKET_DRAG * dt * 0.5   ← rocket deceleration only
      │     if apex: mparticle_burst(72 particles at x,y)
      │              state = EXPLODED
      │
      └── EXPLODED:
            for each particle: mparticle_tick(dt)
              shift trail history (trail[1..] = trail[0..])
              trail[0] = (cx, cy)
              trail_fill = min(trail_fill+1, 16)
              cx += vx*dt*8; cy += vy*dt*8
              g = GRAVITY * (0.8 + rand()*0.4)   ← variance
              vy += g * dt
              shimmer: ~75% of cache[] refreshed
              life -= decay; if life<=0: active=false
            if no particles alive: fuse = 0.5..2.5s; state=IDLE

show_draw(cols, rows):
  erase()
  for each active rocket:
    rocket_draw(rocket, cols, rows)
      │
      ├── RISING: draw '|' at (x,y); '\'' at (x,y+1)
      └── EXPLODED:
            for each particle: mparticle_draw(p, cols, rows)
              fading = (life < 0.25)
              for i = trail_fill-1 down to 0:   ← oldest first
                draw cache[i] at (trail_x[i], trail_y[i])
                attr = trail_attr(i, color, fading)
              draw cache[0] at (cx,cy):
                fading ? color|DIM : CP_WHITE|BOLD
  HUD: fps, sim_fps, rockets, theme
  doupdate()
```

---

## Function Breakdown

### mparticle_burst(p, count, x, y)
Purpose: Spawn a radial burst of MatrixParticles at (x, y).
Steps:
1. For i in 0..count-1:
   - `angle = (i/count)*2π + rand()*0.3` — evenly spaced + jitter
   - `speed = 1.5 + rand()*3.5`
   - `vx = cos(angle)*speed`
   - `vy = sin(angle)*speed` — no squash, full vertical speed
   - `trail_fill = 0`
   - `life = 0.6 + rand()*0.4`, `decay = 0.025 + rand()*0.035`
   - `color = color_rand()` — theme-dependent
   - `active = true`
   - Fill `cache[0..15]` with random chars

### mparticle_tick(p, dt_sec)
Purpose: Advance one simulation step.
Steps:
1. If not active: return.
2. Shift trail history: `trail[i] = trail[i-1]` for i=TRAIL_LEN-1..1.
3. `trail[0] = (cx, cy)`.
4. `trail_fill = min(trail_fill+1, TRAIL_LEN)`.
5. `g = GRAVITY * (0.8 + rand()/RAND_MAX * 0.4)`.
6. `cx += vx*dt*8`, `cy += vy*dt*8`, `vy += g*dt`.
7. Shimmer: for k in 0..TRAIL_LEN-1: if rand()%4 != 0, replace cache[k].
8. `life -= decay`. If `life <= 0`: `active = false`.

### mparticle_draw(p, cols, rows)
Purpose: Draw trail (oldest first) then live head (last).
Steps:
1. If not active: return.
2. `fading = (life < 0.25)`.
3. For `i = trail_fill-1` down to 0:
   - `x = round(trail_x[i])`, `y = round(trail_y[i])`. Skip if out of bounds.
   - `attr = trail_attr(i, color, fading)`. Draw `cache[i]`.
4. `hx = round(cx)`, `hy = round(cy)`. Skip if out of bounds.
5. `attr = fading ? (color|DIM) : (CP_WHITE|BOLD)`. Draw `cache[0]`.

### trail_attr(i, cp, fading) → attr_t
```
if fading:           return cp | A_DIM
if i <= 2:           return cp | A_BOLD
if i < TRAIL_LEN/2:  return cp
else:                return cp | A_DIM
```

### theme_apply(idx)
Purpose: Remap all 7 spark color pairs to theme palette.
Steps:
1. For i in 0..6: `init_pair(i+1, fg[i], COLOR_BLACK)` (256-color or fallback).
2. `init_pair(CP_WHITE, 255, COLOR_BLACK)` — always white.

---

## Pseudocode — Core Loop

```
setup:
  screen_init()
  g_has_256 = (COLORS >= 256)
  theme_apply(0)
  show_init(cols, rows, ROCKETS_DEFAULT)
  frame_time = clock_ns()

loop while running:

  if need_resize:
    show_free(); screen_resize(); show_init(new size, rockets)
    reset frame_time, sim_accum

  dt = clock_ns() - frame_time (capped 100ms)
  tick_ns = 1e9 / sim_fps
  dt_sec  = tick_ns / 1e9

  sim_accum += dt
  while sim_accum >= tick_ns:
    show_tick(dt_sec, cols, rows)
    sim_accum -= tick_ns

  update FPS every 500ms

  sleep to cap at 60fps

  erase()
  show_draw(cols, rows)
  screen_draw_hud(fps, sim_fps, rockets, theme_idx)
  doupdate()

  switch getch():
    q/ESC → quit
    ]     → sim_fps += 5, clamp
    [     → sim_fps -= 5, clamp
    = / + → add rocket (up to MAX)
    -     → remove rocket (down to MIN)
    t     → theme_idx = (theme_idx+1) % THEME_COUNT; theme_apply
```

---

## Interactions Between Modules

```
App [§8]
  └── owns Show [§6]
        └── owns rockets[16] [§5]
              └── each Rocket owns MatrixParticle[72] [§4]

At explosion:
  rocket_tick → mparticle_burst (spawn 72 particles)

Each EXPLODED tick:
  rocket_tick → mparticle_tick × 72
    └── trail shift + physics + shimmer

Each frame:
  rocket_draw → mparticle_draw × 72
    └── trail_attr [§3] → attron/mvaddch/attroff

Theme change (key 't'):
  theme_apply [§3] → init_pair(1..8) → affects all future COLOR_PAIR() draws
```
