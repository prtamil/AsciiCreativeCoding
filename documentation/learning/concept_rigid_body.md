# Concept: 2D Rigid Body Simulation

## Pass 1 — Understanding

### Core Idea
Cubes and spheres fall under gravity, collide with each other and with the floor, exchange momentum, and eventually come to rest. Nothing penetrates anything. The simulation uses axis-aligned shapes (no rotation) so collision detection stays cheap and numerically stable.

### Mental Model
Imagine a box of wooden blocks and rubber balls dropped onto a table. Each hit transfers speed along the contact line (the normal), while friction slows the sliding. Objects that barely move for several frames "fall asleep" — they freeze in place until something bumps them hard enough to wake them.

### Three Body Types
- **Floor** — implicit boundary at the bottom; infinite mass, never moves.
- **Cube** — AABB rectangle with half-extents `(hw, hh)`.
- **Sphere** — circle with radius `r` (stored as `hw = hh = r`).

### Collision Detection
| Pair | Method |
|---|---|
| cube ↔ cube | AABB overlap on each axis; smallest overlap gives penetration depth and normal |
| sphere ↔ sphere | distance between centers vs sum of radii |
| cube ↔ sphere | clamp sphere center to cube AABB → closest point; distance to sphere center vs radius |
| body ↔ floor/walls | compare AABB / circle bottom/top/sides to boundary |

AABB overlap formula for cube-cube:
```
overlap_x = (hw_a + hw_b) - |cx_b - cx_a|
overlap_y = (hh_a + hh_b) - |cy_b - cy_a|
if overlap_x <= 0 or overlap_y <= 0: no collision
normal = axis with smaller overlap (minimum-penetration axis)
```

Closest-point cube-sphere:
```
cx = clamp(sphere.x,  cube.x - cube.hw,  cube.x + cube.hw)
cy = clamp(sphere.y,  cube.y - cube.hh,  cube.y + cube.hh)
depth = sphere.r - distance(sphere.center, (cx, cy))
```

### Impulse Resolution
All collision types feed into one `impulse(a, b, nx, ny, depth, e)` function.
The normal `(nx, ny)` points **from a toward b**.

```
vn = dot(vel_a - vel_b, n)      ← relative velocity along normal
if vn <= 0: separating, skip

j = (1 + e) * vn / (1/m_a + 1/m_b)   ← normal impulse magnitude

vel_a -= n * j / m_a            ← a pushed away from b
vel_b += n * j / m_b            ← b pushed away from a
```

For a fixed surface (floor, wall) `b = NULL` and `1/m_b = 0`.

### Coulomb Friction
Applied on the tangent `t = (-ny, nx)` using the pre-impulse relative velocity:
```
vt = dot(vel_a - vel_b, t)
jt = -vt / (1/m_a + 1/m_b)
jt = clamp(jt, -mu*j, +mu*j)   ← Coulomb cone
```
This prevents friction from accelerating objects: it can only slow sliding.

### Baumgarte Positional Correction
Impulse alone doesn't fully fix overlapping bodies (floating point drift accumulates). Baumgarte adds a small position nudge each frame:
```
corr = max(depth - SLOP, 0) * BAUMGARTE / (1/m_a + 1/m_b)
pos_a -= n * corr / m_a
pos_b += n * corr / m_b
```
`SLOP` is a small allowed overlap (0.30 units) to prevent jitter from over-correction.

### Sleep System
Keeps the simulation stable when objects come to rest:
- Each body has a `sleep_cnt` integer and a `sleeping` flag.
- Every frame, if `|vel| < SLEEP_VEL`, increment `sleep_cnt`; otherwise reset to 0.
- When `sleep_cnt >= SLEEP_FRAMES`, mark the body as sleeping.
- Sleeping bodies skip gravity and integration.
- Any collision with impulse `> WAKE_IMP` wakes the body.
- Body-body pairs where **both** are sleeping skip collision entirely.

### Non-Obvious Decisions
- **No rotation**: AABB collision only works for axis-aligned shapes. Skipping rotation removes the need for SAT on arbitrary orientations, eliminates angular momentum state, and makes the math much cleaner. Most "stacking" demos don't visually need it.
- **Normal direction convention**: `impulse(a, b, n)` requires n to point from `a` toward `b`. This means `vn = dot(va - vb, n) > 0` is "approaching" — the impulse sign then works out without negation.
- **3 solver iterations**: Running the body-body collision loop 3 times per step reduces stacking penetration accumulation. More iterations → stiffer stack at the cost of CPU.
- **SLOP threshold**: Without it, every tiny overlap triggers a Baumgarte nudge, causing visible jitter on resting contacts. The slop absorbs numerical noise below 0.30 units.
- **Velocity cap**: Dense stacking can blow up impulses across multiple iterations. Capping `|vel| <= MAX_SPEED` prevents tunneling at the cost of energy conservation under stress.

---

## Pass 2 — Implementation

### Module Map
```
§1  config     — constants: GRAVITY, FRICTION, SLEEP_VEL, CUBE_HW, etc.
§2  clock      — monotonic timer for fixed-FPS loop
§3  color      — 5 themes × ncurses color-pair init
§4  body       — Body struct (kind, x, y, vx, vy, hw, hh, mass, imass, sleep)
§5  framebuf   — char + attr double buffer → ncurses batch render
§6  physics    — impulse(), col_cc(), col_ss(), col_cs(), col_floor(), col_walls()
§7  scene      — scene_add_cube(), scene_add_sphere(), scene_step()
§8  draw       — draw_body() fills framebuf with '#' (cube) or 'O' (sphere)
§9  screen     — flush_screen(), draw_hud()
§10 app        — main(), input handling, game loop
```

### Data Flow per Frame
```
1. Input → add/remove body or toggle settings
2. scene_step():
   a. Apply gravity to all non-sleeping bodies
   b. Integrate velocity → position, apply damping, cap speed
   c. 3 iterations of body-body collision (col_cc / col_ss / col_cs)
   d. col_floor() + col_walls() for each non-sleeping body
   e. Update sleep counters
3. draw_body() for each body → framebuf
4. flush_screen() → ncurses batch update
5. Sleep until next frame
```

### Body Struct
```c
typedef enum { KIND_CUBE = 0, KIND_SPHERE } Kind;

typedef struct {
    Kind  kind;
    float x, y;        /* center, physics coords (y increases downward) */
    float vx, vy;
    float hw, hh;      /* cube: half-extents;  sphere: hw == hh == radius */
    float mass, imass; /* imass = 1/mass, 0 for fixed bodies */
    int   cp;          /* ncurses color pair */
    int   sleep_cnt;
    bool  sleeping;
} Body;
```

### Key Constants
| Constant | Value | Role |
|---|---|---|
| `GRAVITY` | 0.04 | downward acceleration per step² |
| `REST_DEF` | 0.35 | default coefficient of restitution |
| `FRICTION` | 0.30 | Coulomb μ |
| `DAMPING` | 0.992 | per-step velocity multiplier |
| `MAX_SPEED` | 20.0 | velocity cap (tunneling prevention) |
| `BAUMGARTE` | 0.40 | positional correction fraction per step |
| `SLOP` | 0.30 | penetration before correction fires |
| `SLEEP_VEL` | 0.06 | speed threshold for sleep increment |
| `SLEEP_FRAMES` | 18 | quiet frames before body sleeps |
| `WAKE_IMP` | 0.04 | impulse needed to wake a sleeping body |
| `CUBE_HW / HH` | 7 / 5 | cube half-extents |
| `SPH_R` | 5.0 | sphere radius |
| `MAX_BODIES` | 30 | scene capacity |

### Physics Coordinate System
- Origin top-left, x increases right, y increases **downward** (ncurses convention).
- Gravity adds to `vy` each step (positive = downward).
- Floor is at `y = screen_height - HUD_HEIGHT`. Bodies rest with their bottom edge on this line.

### Build
```
gcc -std=c11 -O2 -Wall -Wextra physics/rigid_body.c -o rigid_body -lncurses -lm
```

### Keys
| Key | Action |
|---|---|
| `c` | add cube at random x position |
| `s` | add sphere at random x position |
| `x` | remove last body |
| `r` | reset scene |
| `p` | pause / resume |
| `g` | toggle gravity |
| `e` / `E` | decrease / increase restitution |
| `t` / `T` | cycle theme |
| `q` | quit |
