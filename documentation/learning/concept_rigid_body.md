# Concept: 2D Rigid Body Simulation

## Pass 1 — Understanding

### Core Idea
Cubes and spheres fall under gravity, collide with each other and with the
floor, exchange momentum with friction, and eventually come to rest. Nothing
penetrates anything. No rotation — shapes are axis-aligned so collision
detection is fast and numerically stable.

## From the Source

**Algorithm:** Iterative impulse-based collision resolution (Baumgarte). SOLVER_ITERS constraint solver passes per step. Each pass: detect overlap, apply positional correction (Baumgarte stabilisation) then velocity impulse.

**Physics:** 2D rigid-body mechanics with restitution and friction. Restitution (REST_DEF=0.35): e=0 → perfectly inelastic (no bounce); e=1 → perfectly elastic (infinite bounce). Friction (FRICTION=0.35): Coulomb model |jt| ≤ μ·jn. REST_THRESH: micro-bounce suppression below 0.20 px/step.

**Math:** SLOP (allowed penetration depth before correction fires) = 0.05 px — absorbs floating-point noise while still resolving real overlap. BAUMGARTE (0.50): fraction of penetration corrected each iteration. 1.0 → sharp but can overshoot; 0.5 is stable.

**Performance:** Sleep system — bodies sleeping for SLEEP_FRAMES quiet frames are frozen. Saves CPU; woken by impulse or position correction > WAKE_IMP.

### Mental Model
A box of wooden blocks and rubber balls dropped onto a table. Each hit
transfers speed along the contact line (the normal). Friction slows sliding.
Objects that barely move for several frames "fall asleep" — they freeze until
something bumps them hard enough to wake them.

---

### The Terminal Aspect-Ratio Problem
Terminal character cells are approximately **2 × taller than wide** (one row
≈ two columns of height). This forces a coordinate system choice:

```
physics unit  =  1 column  =  0.5 rows
prow(y) = y / 2   (convert physics y to screen row)
pcol(x) = x       (1:1)
```

A sphere of visual radius `r` (meaning `r` columns wide, `r` rows tall) has:
- x-extent: `r` physics units each side
- y-extent: `2r` physics units each side (because `prow` halves it back)

So the sphere's **physics AABB** must be `hw = r, hh = 2r`.

Using `hw = hh = r` (the "obvious" choice) means collision detects when
centers are `2r` apart — but visually the drawn circles don't touch until
centers are `4r` apart. The bodies appear to overlap by `r` rows at the
moment physics fires. This was the bug in earlier versions.

---

### Collision Detection — Unified AABB

Every body (cube or sphere) has an AABB `(hw, hh)`. A single function
handles all pairs:

```
cube:   hw = CUBE_HW,   hh = CUBE_HH
sphere: hw = SPH_R,     hh = 2 * SPH_R   ← aspect-corrected
```

**AABB overlap test:**
```
ox = (hw_a + hw_b) - |cx_b - cx_a|
oy = (hh_a + hh_b) - |cy_b - cy_a|
if ox <= 0 or oy <= 0: no collision

minimum-penetration axis:
  if ox < oy:  normal = (±1, 0),  depth = ox
  else:        normal = (0, ±1),  depth = oy
```
Sign of normal is chosen so it points **from a toward b**.

This one test covers cube-cube, sphere-sphere, and cube-sphere correctly.
The sphere's AABB already encodes the visual shape — no special case needed.

**Floor and walls** use the same `hw/hh` extents:
- floor fires when `y + hh > WH()` → snaps body to `y = WH() - hh`
- left wall fires when `x - hw < 0` → snaps to `x = hw`
- right wall fires when `x + hw > WW()` → snaps to `x = WW() - hw`
- ceiling fires when `y - hh < 0` → snaps to `y = hh`

---

### Resolution — Two-Pass Per Iteration

This is the design decision that eliminates visible overlap. The old
approach put Baumgarte inside the velocity function, guarded by `vn > 0`.
When two bodies had the same velocity on the contact axis (`vn = 0`), they
fell together as a merged mass — nothing pushed them apart.

**Pass A — positional correction (always)**
```
corr = max(depth - SLOP, 0) * BAUMGARTE / (imA + imB)
pos_a -= n * corr * imA
pos_b += n * corr * imB
```
Runs even when bodies are separating. Wakes sleeping bodies if pushed
significantly (`corr * imass > WAKE_IMP`).

**Pass B — velocity impulse (only when approaching: vn > 0)**
```
vn = dot(vel_a - vel_b, n)
if vn <= 0: skip

e_eff = (vn > REST_THRESH) ? e : 0    ← adaptive restitution
j = (1 + e_eff) * vn / (imA + imB)

vel_a -= n * j * imA
vel_b += n * j * imB
```

**Adaptive restitution** (`e_eff`): at gravity-scale approach speeds (`vn`
small), `e_eff = 0`. The impulse exactly cancels the incoming velocity
instead of bouncing. Without this, gravity → tiny `vn` → tiny bounce → next
frame → repeat forever (the floor jitter bug).

**Coulomb friction** on the tangent `t = (-ny, nx)`:
```
vt = dot(vel_a - vel_b, t)
jt = clamp(-vt / (imA+imB),  -μ·j,  +μ·j)
```
Clamping to the Coulomb cone `|jt| ≤ μ·j` ensures friction can only slow
sliding, never accelerate it.

---

### Floor and Wall Resolution

Floor/walls use **full position snap** (not a fraction), then resolve
velocity directly — no Baumgarte fraction needed since position is already
exact:

```c
col_floor:
  b->y = WH() - b->hh         /* full snap */
  if vy > 0:
    e_eff = (vy > REST_THRESH) ? e : 0
    vy = -vy * e_eff           /* reflect or stop */
    vx *= (1 - friction)       /* floor friction */
```

Floor and wall checks run for **all bodies every step, including sleeping
ones**. Sleeping bodies have zero velocity so the velocity block returns
immediately; only the position clamp matters. This prevents Baumgarte
corrections from body-body pairs sinking sleeping bodies below the floor.

---

### Sleep System
```
every frame (non-sleeping body):
  if |vel| < SLEEP_VEL:
    sleep_cnt++
    if sleep_cnt >= SLEEP_FRAMES: freeze (vx=vy=0, sleeping=true)
  else:
    sleep_cnt = 0

woken when:
  velocity impulse j > WAKE_IMP, OR
  positional correction corr*imass > WAKE_IMP
```

Both-sleeping body pairs skip the solver entirely.

---

### Spawn Overlap Check
New bodies try up to 8 random x positions at the top of the screen. If all
positions overlap an existing body's AABB, the spawn is rejected. This
prevents bodies from starting fully nested (which the solver would need many
frames to resolve).

---

### Non-Obvious Decisions

- **Unified AABB for spheres**: Spheres could use circle-distance detection
  but that ignores the terminal y-scale. Giving spheres `hh = 2*hw` and
  using the same AABB test as cubes is simpler and more accurate on screen.

- **Separate positional correction from velocity impulse**: The key insight
  is that position overlap and velocity direction are independent problems.
  A body can be overlapping while moving away (`vn < 0`). Old code skipped
  correction in that case. Separating them means overlap is always resolved
  regardless of velocity.

- **`e_eff = 0` at low speed**: Without this, every frame gravity adds a
  tiny `vy`, floor fires, impulse bounces with `e>0`, body moves up, gravity
  pulls it down, repeat. This creates infinite micro-jitter at rest. Setting
  `e_eff = 0` when `vn < REST_THRESH` makes the impulse cancel `vy` exactly.

- **Floor runs for sleeping bodies**: Sleeping bodies skip gravity and
  integration — but Baumgarte in body-body collisions can still move them.
  If a sleeping cube is on the floor and an active body pushes it down via
  Baumgarte, the floor check corrects the position back without waking it.

- **SLOP = 0.05**: Small allowed penetration before correction fires. Removes
  jitter from floating-point noise at resting contacts (without it, every
  frame has a tiny depth > 0 and triggers correction in both directions).

- **10 solver iterations**: With N stacked bodies, resolving the bottom pair
  propagates up through the stack. More iterations → stack settles faster.
  6 iterations left residual drift visible over many frames.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_b[MAX_BODIES]` | `Body[32]` | ~1.5 KB | position, velocity, AABB half-extents, mass, color, sleep state |
| `g_fb[ROWS_MAX][COLS_MAX]` | `char[128][512]` | 64 KB | character framebuffer for body outlines |
| `g_fcp[ROWS_MAX][COLS_MAX]` | `int[128][512]` | 256 KB | color-pair framebuffer parallel to g_fb |
| `g_nb`, `g_ncubes`, `g_nsphs` | `int` | 12 B | live body count and per-type spawn counters |
| `g_rest`, `g_grav`, `g_paused` | `float` / `bool` × 2 | 12 B | restitution coefficient, gravity toggle, pause flag |
| `g_rows`, `g_cols` | `int` | 8 B | terminal dimensions |
| `g_rng` | `uint32_t` | 4 B | XorShift RNG state for body placement |

## Pass 2 — Implementation

### Module Map
```
§1  config     — GRAVITY, FRICTION, BAUMGARTE, SLOP, REST_THRESH, SPH_R, etc.
§2  clock      — monotonic ns timer for fixed-FPS loop
§3  color      — 5 themes × ncurses color-pair init
§4  body       — Body struct; body_init_mass(); body_wake(); rng_f()
§5  framebuf   — char+attr double buffer → ncurses batch render
§6  physics    — col_bodies(), col_floor(), col_walls(), scene_step()
§7  scene      — aabb_overlaps_any(), scene_add_cube/sphere(), scene_init()
§8  draw       — draw_body() → framebuf ('#' cube, 'O' sphere)
§9  screen     — screen_init(), screen_resize(), signal handlers
§10 app        — main(): input loop, fixed-timestep game loop
```

### Data Flow per Frame
```
1. Input → add/remove body, toggle settings
2. scene_step():
   a. Gravity to all non-sleeping bodies
   b. Integrate velocity → position; damping; speed cap
   c. SOLVER_ITERS × (col_bodies for all non-both-sleeping pairs)
   d. col_floor() + col_walls() for ALL bodies (sleeping too)
   e. Sleep counter update
3. draw_body() for each body → framebuf
4. fb_flush() + doupdate() → terminal
5. sleep_ns() until next frame
```

### Body Struct
```c
typedef enum { KIND_CUBE = 0, KIND_SPHERE } Kind;

typedef struct {
    Kind  kind;
    float x, y;        /* center; y increases downward (physics coords)  */
    float vx, vy;
    float hw, hh;      /* AABB half-extents (aspect-corrected)           */
                       /* cube: hw=CUBE_HW, hh=CUBE_HH                  */
                       /* sphere: hw=SPH_R, hh=2*SPH_R                  */
    float mass, imass;
    int   cp;          /* ncurses color pair                             */
    int   sleep_cnt;
    bool  sleeping;
} Body;
```

### Key Constants (actual values in code)
| Constant | Value | Role |
|---|---|---|
| `GRAVITY` | 0.05 | downward accel per step |
| `REST_DEF` | 0.35 | default coefficient of restitution |
| `FRICTION` | 0.35 | Coulomb μ |
| `DAMPING` | 0.991 | per-step velocity multiplier |
| `MAX_SPEED` | 22.0 | velocity cap (tunneling prevention) |
| `SOLVER_ITERS` | 10 | body-body iterations per step |
| `BAUMGARTE` | 0.50 | positional correction fraction |
| `SLOP` | 0.05 | penetration allowed before Baumgarte fires |
| `REST_THRESH` | 0.20 | approach speed below this → `e_eff = 0` |
| `SLEEP_VEL` | 0.07 | speed threshold for sleep counter |
| `SLEEP_FRAMES` | 30 | quiet frames before body sleeps |
| `WAKE_IMP` | 0.05 | min impulse/corr to wake sleeping body |
| `CUBE_HW / HH` | 7 / 5 | cube half-extents |
| `SPH_R` | 4.0 | sphere visual radius (cols and rows) |
| `MAX_BODIES` | 32 | scene capacity |

### Sphere Drawing vs Physics AABB
```
draw:   x offset = hw * cos(a)    → r columns from center
        y offset = hh * sin(a)    → 2r physics units = r rows via prow()
physics AABB: hw = r, hh = 2r

result: drawn circle exactly fits the physics bounding box on screen
```

### Build
```
gcc -std=c11 -O2 -Wall -Wextra physics/rigid_body.c -o rigid_body -lncurses -lm
```

### Keys
| Key | Action |
|---|---|
| `c` | add cube (spawn-checks for clear position) |
| `s` | add sphere (same) |
| `x` | remove last body |
| `r` | reset scene |
| `p` / `Space` | pause / resume |
| `g` | toggle gravity |
| `e` / `E` | increase / decrease restitution |
| `t` / `T` | cycle theme forward / backward |
| `q` / `Esc` | quit |
