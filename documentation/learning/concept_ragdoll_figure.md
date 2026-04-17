# Pass 1 — ragdoll_figure: Verlet-integrated humanoid ragdoll with distance constraints

## From the Source

**Algorithm:** Verlet integration. Each particle stores its current and previous position; velocity is implicit in the difference `vel = (pos - old_pos) * DAMPING`. Gravity and wind accelerate particles each tick; 8 passes of distance-constraint projection then restore all bone lengths. Boundary collisions are implemented as position clamp + old_pos reflection, which naturally produces a physical bounce response without any explicit velocity manipulation.

**Math:** Verlet update rule: `new_pos = pos + (pos - old_pos)*DAMPING + accel * dt²`. Distance constraint: `error = (dist - rest_len) / dist`; correction `= 0.5 * error * vec` applied equally to both particles. Slanted-platform bounce decomposes velocity into normal and tangent components using the surface normal `n = (slope, -1) / sqrt(slope²+1)`, then reflects the normal component with `BOUNCE_COEFF` energy retention.

**Performance:** 8 constraint iterations × 17 bones = 136 scalar correction ops per tick. Platform re-enforcement runs inside the constraint loop (8 × 15 = 120 clamp checks per tick). All trivial. The ncurses doupdate diff engine is the bottleneck. Fixed-step accumulator decouples physics Hz from render Hz.

**Data-structure:** `Ragdoll` struct with parallel arrays `pos[]`, `old_pos[]`, `prev_pos[]` (Verlet pair + interpolation snapshot) and parallel constraint arrays `c_a[]`, `c_b[]`, `c_len[]`. 15 particles, 17 constraints. All positions in pixel space so distances and forces are isotropic regardless of terminal aspect ratio.

## Core Idea

A 15-particle humanoid stick figure falls under gravity, bounces off slanted platforms, and sways from random wind gusts. The central physics technique is Verlet integration: by storing current and previous positions instead of velocity, constraints can be enforced as pure geometry operations that do not introduce velocity error. The body stays together because 8 iterative distance-constraint passes each tick restore every bone to its rest length. The figure bounces because reflecting `old_pos` across a surface boundary reverses the implicit velocity perpendicular to that surface.

## The Mental Model

Imagine you have a marionette puppet whose joints are connected by rigid sticks. Now imagine those sticks are enforced not by real rigidity but by a careful correction step every frame: if any stick is too long, move both endpoints toward each other by half the excess. If too short, push them apart. Run this correction 8 times and the sticks are effectively rigid.

The figure does not have velocity stored anywhere. Its velocity is implicit: the distance and direction from last position to current position encodes exactly where it is heading. To make it bounce off a floor, you simply clamp its position to the floor and then move its "previous position" to the same side of the floor, which means next tick the implicit velocity points upward — the bounce.

Wind is applied by nudging `old_pos` rather than `pos`. Since `vel = pos - old_pos`, decreasing `old_pos.x` by `impulse * dt` is equivalent to adding `impulse` to the x-velocity. The constraint system then distributes that velocity through the connected skeleton naturally.

## Data Structures

### Vec2
```
x, y   — position in pixel space (float, square isotropic grid)
         x increases eastward; y increases downward (terminal convention)
```

### Ragdoll
```
pos[15]        — current particle positions in pixel space
old_pos[15]    — positions from the previous tick (Verlet pair)
                 velocity is implicit: vel = pos - old_pos
prev_pos[15]   — snapshot saved at tick start for alpha lerp
                 render_ragdoll() lerps prev_pos → pos using alpha ∈ [0,1)

c_a[17]        — first particle index for each constraint
c_b[17]        — second particle index for each constraint
c_len[17]      — rest length for each constraint (set from initial T-pose)

wind_timer     — seconds since last wind gust
wind_x         — current gust acceleration component (px/s²)
gravity        — downward acceleration (px/s², adjustable at runtime)
wind_force     — gust amplitude (px/s², adjustable at runtime)
paused         — if true, physics is frozen but prev_pos is still saved
```

### Platform
```
cx, y          — centre x and surface y at centre (pixel space)
half_w         — half-width along x
slope          — dy/dx (positive = right side lower, negative = right side higher)
```
Five platforms distributed across the screen in a staggered zig-zag pattern. Surface y at position x: `surf_y = y + (x - cx) * slope`.

### Scene
```
ragdoll        — the single Ragdoll simulation state
platforms[5]   — staggered slanted shelves the ragdoll falls through
```

## The Main Loop

Each iteration of the main loop:

1. **Check for resize.** Re-read terminal dimensions, clamp all 15 particle positions to the new pixel bounds (both `pos` and `old_pos`), rescale platforms to new dimensions, reset timing to prevent a physics avalanche.

2. **Measure dt.** Read the monotonic clock. Subtract from the previous frame timestamp. Cap at 100 ms to prevent physics runaway after process suspension.

3. **Physics accumulator.** Add dt to `sim_accum`. While `sim_accum >= tick_ns`: call `scene_tick(dt_sec)` and subtract `tick_ns`. This fires the ragdoll physics at exactly `sim_fps` Hz regardless of render rate.

4. **Compute alpha.** `alpha = sim_accum / tick_ns ∈ [0, 1)`. This is the sub-tick fraction for render interpolation.

5. **Update FPS counter.** 500 ms sliding window average.

6. **Sleep for frame cap.** Sleep before render to hold ~60 fps render rate. Terminal I/O cost does not bleed into the physics budget.

7. **Draw and present.** `erase()` clears newscr. `scene_draw(alpha)` draws bones and particle markers. HUD bars are drawn last (always win over any ragdoll glyph). `wnoutrefresh + doupdate` atomically sends the diff to the terminal.

8. **Handle input.** Non-blocking `getch()` loop. Arrow keys adjust gravity and wind force. `[/]` adjust sim Hz.

## Non-Obvious Decisions

**Why store velocity implicitly as `pos - old_pos`?**
Explicit Euler requires recomputing velocity after each constraint correction. With Verlet, the implicit velocity automatically reflects any position change made by the constraint solver — the corrected position becomes the "current" position, and the uncorrected "old" position encodes the velocity from before the constraint ran. You get position-based dynamics for free.

**Why apply constraints AFTER boundary collision?**
Boundary collision moves an ankle to the floor level. If constraints ran before collision, the constraint would try to restore the ankle-knee distance from the ankle's pre-collision (below-floor) position. Running constraints after collision means the ankle is already at the correct floor position, and the constraint then correctly pulls the knee to the appropriate height above it.

**Why re-enforce platform collisions INSIDE the constraint loop?**
The constraint solver can push a particle through a platform. After one constraint pass, a foot might be above the platform correctly, but the connected leg constraint might then push it back through. Re-enforcing after every pass prevents this compounding. The same "interleave boundary with solver" pattern appears in all position-based dynamics systems.

**Why reflect `old_pos` for bouncing instead of negating velocity?**
In Verlet integration there is no velocity variable to negate. Reflecting `old_pos` across the collision surface reverses the implicit velocity component perpendicular to that surface. For a floor bounce: `old_pos.y = pos.y + (pos.y - old_pos.y) * BOUNCE_COEFF`. The term `(pos.y - old_pos.y)` is the downward component of the implicit velocity at the moment of collision. Multiplying by `BOUNCE_COEFF` (0.55) removes 45% of that energy and reverses direction, placing `old_pos` above the floor so next tick the particle moves upward.

**Why `DAMPING = 0.995` specifically?**
`0.995^60 ≈ 0.741` — the ragdoll retains about 74% of its kinetic energy per second at 60 Hz. This is "barely perceptible air drag" — just enough to prevent the simulation from building up oscillation energy over many bounces and going unstable, without making the figure look like it is moving through honey.

**Why inject wind as a nudge to `old_pos` rather than as a force?**
The Verlet formulation means `vel = pos - old_pos`. Subtracting `impulse * dt` from `old_pos.x` is equivalent to adding `impulse` to the x-velocity with no dt² scaling artifact. It is a direct velocity kick in the Verlet framework. Applying it as an acceleration (`accel * dt²`) would be much weaker per frame and would require tuning different numbers.

**Why set `rest_len` from the initial T-pose positions?**
The constraint rest lengths are computed from the Euclidean distances in the initial T-pose: `add_constraint(r, &nc, a, b)` calls `sqrtf(dx*dx + dy*dy)` on the initial `pos[]`. This means there is no separate constant table of bone lengths. The T-pose IS the rest pose. If you change the initial joint offsets, all constraint lengths automatically update.

**Why save `prev_pos` before the pause check?**
The interpolation in `render_ragdoll` lerps from `prev_pos` to `pos`. If the simulation is paused and `prev_pos` equals `pos`, the lerp is a clean identity (no drift). If `prev_pos` were not saved on pause frames, `render_ragdoll` would lerp from the position before the previous tick to the frozen position — a visible jitter at pause time.

**Why use DRAW_STEP_PX = 2.0 for bone rendering?**
Bones are often near-vertical (arms hanging down, legs dangling). For a perfectly vertical bone of length L, samples must be spaced at less than `CELL_H = 16 px` to avoid skipping a terminal row. At `DRAW_STEP_PX = 2.0`, the step along y is at most 2 px — well below 16 px. The constraint `DRAW_STEP_PX < CELL_W (8 px)` ensures no column is skipped along near-horizontal bones either.

## State Machines

### Ragdoll physics order per tick
```
ragdoll_tick() — called once per physics step

  ┌────────────────────────────────────────────────────────┐
  │ 1. memcpy(prev_pos ← pos)   [interpolation anchor]    │
  │                                                        │
  │ 2. if paused → return                                  │
  │                                                        │
  │ 3. wind_timer += dt                                    │
  │    if wind_timer >= WIND_PERIOD:                       │
  │        random impulse → nudge old_pos.x                │
  │                                                        │
  │ 4. verlet_update() × 15 particles                      │
  │    (gravity + wind → new positions)                    │
  │                                                        │
  │ 5. apply_boundaries() × 15 particles                   │
  │    (floor bounce, wall bounce, ceiling clamp)          │
  │                                                        │
  │ 5b. apply_platform_collisions() × 15 particles         │
  │                                                        │
  │ 6. for iter in 0..7:                                   │
  │      satisfy_constraint() × 17 constraints             │
  │      apply_platform_collisions() × 15 particles        │
  └────────────────────────────────────────────────────────┘
```

### Particle boundary state
```
         pos.y > floor_y ?
         ┌──YES─→ clamp pos.y = floor_y
         │         reflect old_pos.y (BOUNCE_COEFF)
         │
pos.y ───┤
         │
         └──NO──→ no change

         pos.y < ceil_y ?
         ┌──YES─→ clamp pos.y = ceil_y  (hard stop, no bounce)
         │
         └──NO──→ no change
```

### Constraint satisfaction (per bone, per iteration)
```
dist = |p2.pos - p1.pos|
error = (dist - rest_len) / dist

if error > 0: bone is stretched  → push endpoints toward each other
if error < 0: bone is compressed → push endpoints apart
each endpoint moves 0.5 × error × vec  (equal mass assumption)
```

### Input actions
| Key | Effect |
|-----|--------|
| q / Q / ESC | exit |
| space | toggle pause |
| r / R | full reset (new T-pose, new platforms) |
| w / UP | gravity × 1.3 (max 3200 px/s²) |
| s / DOWN | gravity ÷ 1.3 (min 100 px/s²) |
| d / RIGHT | wind_force + 20 (max 600) |
| a / LEFT | wind_force − 20 (min 0) |
| ] / + / = | sim_fps + 10 |
| [ / - | sim_fps − 10 |

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|----------|---------|-------------------|
| GRAVITY | 800 px/s² | Increase → faster fall, more violent bounces. Decrease → floaty, slow collapse. Too low → constraints can't keep up, figure becomes elastic. |
| DAMPING | 0.995 | Decrease (e.g. 0.98) → heavier air drag, motion dies quickly. Increase toward 1.0 → bounces amplify, potential instability at high gravity. |
| BOUNCE_COEFF | 0.55 | 0 → dead stop at floor. 1.0 → perfectly elastic. > 1.0 → particle gains energy at each bounce (explosion). |
| WIND_PERIOD | 3.5 s | Decrease → more frequent gusts, chaotic. Increase → fewer gusts, figure comes to rest between them. |
| WIND_FORCE | 120 px/s | Increase → stronger lateral kicks, figure flies across screen. Decrease → minimal lateral motion. |
| N_CONSTRAINT_ITERS | 8 | Decrease → bones stretch visibly, figure sags. Increase → stiffer skeleton, more CPU. 4 iterations shows visible stretch at high gravity. |
| DRAW_STEP_PX | 2.0 | Increase beyond CELL_W (8) → gaps appear in near-horizontal bones. Decrease → more cell fills per bone (minimal visual benefit, slight CPU cost). |
| FLOOR_MARGIN | 8 px | Decrease → floor is lower on screen (more fall distance). Increase → floor occupies more visible area. |

## Open Questions for Pass 3

- The shoulder-width stabiliser (constraint 14) and hip-width stabiliser (constraint 15) are drawn with `A_DIM`. They are visible in the output. Should structural struts be invisible (not drawn)? How would you suppress specific constraints from rendering without renumbering the array?
- When the ragdoll is paused mid-bounce, the alpha lerp should produce a static image. But `alpha` is still non-zero and changing each frame. Does `prev_pos == pos` (because the physics tick returns early) make the lerp a no-op, or is there still drift?
- `apply_platform_collisions` re-runs inside the constraint loop. Why does the constraint loop not have its own anchor-enforcement for the constraint endpoints? Is this a gap that could cause subtle tunnelling?
- Why is only constraint `(0,2)` (head → left_shoulder strut) added and not `(0,3)` (head → right_shoulder)? What visual asymmetry does this create when the figure falls head-first?
- Platform slopes use `±0.35` to `±0.40`. What happens to the slanted-surface bounce math when `slope → ∞` (vertical surface)? Is there a singularity in the normal calculation?
- The wind gust applies the same impulse to all 15 particles equally. In a real ragdoll system, wind force would scale with exposed surface area per segment. What would change if you applied wind only to the torso particles (head, neck, hip) and let the constraints propagate it to the limbs?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app` | `App` | ~20 KB | top-level container: scene + screen + control flags |
| `g_app.scene.ragdoll.pos[15]` | `Vec2[15]` | 120 B | current particle positions (pixel space) |
| `g_app.scene.ragdoll.old_pos[15]` | `Vec2[15]` | 120 B | previous positions (implicit velocity) |
| `g_app.scene.ragdoll.prev_pos[15]` | `Vec2[15]` | 120 B | tick-start snapshot for alpha lerp |
| `g_app.scene.ragdoll.c_a/c_b/c_len[17]` | mixed | ~136 B | constraint topology and rest lengths |
| `g_app.scene.platforms[5]` | `Platform[5]` | 80 B | slanted shelf collision surfaces |

---

# Pass 2 — ragdoll_figure: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | All tunables: GRAVITY, DAMPING, BOUNCE_COEFF, WIND_*, particle/constraint counts, timing |
| §2 clock | `clock_ns()` — monotonic nanosecond timestamp; `clock_sleep_ns()` — precise sleep |
| §3 color | `color_init()` — body-part palette (white/grey/orange/blue) + HUD pair (yellow) |
| §4 coords | `px_to_cell_x/y` — the one aspect-ratio conversion; physics is pixel space, draw is cell space |
| §5a verlet_update | One Verlet integration step: `new_pos = pos + (pos-old_pos)*DAMPING + accel*dt²` |
| §5b apply_boundaries | Floor/ceiling/wall clamping + old_pos reflection for bounce |
| §5b-2 apply_platform_collisions | Slanted-surface detection + normal-component reflection |
| §5c satisfy_constraint | One distance constraint projection (half-correction to each endpoint) |
| §5d ragdoll_tick | Full physics step: prev_pos save → wind → Verlet → boundaries → constraints |
| §5e ragdoll_draw | Alpha lerp → bone rendering (draw_bone) → particle markers |
| §6 scene | `scene_init` (T-pose + platforms) / `scene_tick` / `scene_draw` wrappers |
| §7 screen | ncurses double-buffer layer (erase → draw → wnoutrefresh → doupdate) |
| §8 app | Signal handlers, resize handler, main fixed-step accumulator loop |

---

## Data Flow Diagram

```
CLOCK_MONOTONIC
    │
    ▼
clock_ns() → dt (nanoseconds)
    │
    ▼
sim_accum += dt
    │
    while sim_accum >= tick_ns:
    │
    ▼
ragdoll_tick(dt_sec, cols, rows)
    │
    ├── memcpy(prev_pos ← pos)          [save before physics for alpha lerp]
    │
    ├── wind: if timer >= WIND_PERIOD:
    │       old_pos[i].x -= impulse*dt  [velocity kick via Verlet old_pos]
    │
    ├── verlet_update(i, dt) × 15       [gravity + wind → new pos]
    │       vel = (pos - old_pos) * DAMPING
    │       new_pos = pos + vel + accel * dt²
    │       old_pos = pos; pos = new_pos
    │
    ├── apply_boundaries(i) × 15        [clamp + reflect old_pos for bounce]
    │
    ├── apply_platform_collisions × 15  [slanted surface bounce]
    │
    └── for 8 iterations:
            satisfy_constraint(ci) × 17 [push endpoints to rest_len]
            apply_platform_collisions   [re-enforce after each pass]

sim_accum -= tick_ns
    │
    ▼
alpha = sim_accum / tick_ns   [0.0 .. 1.0)
    │
    ▼
render_ragdoll(alpha)
    │
    ├── rp[i] = prev_pos[i] + (pos[i] - prev_pos[i]) * alpha  [lerp]
    │
    ├── draw platforms (bead fill + node markers)
    ├── draw ground line ('-' at floor row)
    ├── draw_bone(ci) × 17  [walk bone in DRAW_STEP_PX increments, stamp glyph]
    └── particle markers: 'O' head, '*' wrists, 'v' ankles, '.' others

    ▼
screen_present() → doupdate()   [ONE atomic diff write to terminal fd]
```

---

## Function Breakdown

### verlet_update(r, i, dt)
Purpose: advance particle i by one Verlet step with damping
Steps:
1. Save `old = old_pos[i]`, `cur = pos[i]`
2. `vel_x = (cur.x - old.x) * DAMPING`, same for y
3. `old_pos[i] = cur`
4. `pos[i].x = cur.x + vel_x + wind_x * dt²`
5. `pos[i].y = cur.y + vel_y + gravity * dt²`
Edge cases: none (gravity is always positive = downward, wind_x signed)

---

### apply_boundaries(r, i, cols, rows)
Purpose: keep particle i inside screen bounds; bounce or clamp
Steps:
1. Compute `floor_y`, `ceil_y`, `left_x`, `right_x` from pixel dimensions
2. Floor: `if pos.y > floor_y`: clamp pos.y = floor_y; set `old_pos.y = pos.y + (pos.y - old_pos.y) * BOUNCE_COEFF`
3. Ceiling: `if pos.y < ceil_y`: hard stop — set both `pos.y = old_pos.y = ceil_y`
4. Walls: same bounce pattern as floor but for x axis

---

### apply_platform_collisions(r, i, plats, n)
Purpose: detect and respond to slanted platform surface crossings
Steps:
1. For each platform: check if `|pos.x - pl.cx| <= pl.half_w`
2. Compute `surf_y = pl.y + (pos.x - pl.cx) * slope`
3. Detect crossing from above: `old_pos.y <= old_surf_y AND pos.y >= surf_y`
4. Snap `pos.y = surf_y`
5. Compute surface normal `n = (slope, -1) / sqrt(slope²+1)`
6. Decompose velocity into normal component `vn = dot(vel, n)`
7. If `vn < 0` (moving into surface): reflect `old_pos = pos - (v - (1+BOUNCE_COEFF)*vn*n)`

---

### satisfy_constraint(r, ci)
Purpose: project one bone to its rest length via equal-mass correction
Steps:
1. `dx = pos[c_b].x - pos[c_a].x`, `dy = pos[c_b].y - pos[c_a].y`
2. `dist = sqrt(dx² + dy²)`
3. Guard: `if dist < 1e-6`: return (coincident particles)
4. `error = (dist - c_len[ci]) / dist`
5. `cx = 0.5 * error * dx`, `cy = 0.5 * error * dy`
6. `pos[c_a] += (cx, cy)`, `pos[c_b] -= (cx, cy)`

---

### draw_bone(w, a, b, pair, attr, cols, rows, prev_cx, prev_cy)
Purpose: stamp direction glyphs along a bone from pixel point a to b
Steps:
1. Compute `(dx, dy) = b - a`, `len = ||(dx,dy)||`
2. `glyph = bone_glyph(dx, dy)` → one of `-`, `|`, `/`, `\`
3. Walk in `DRAW_STEP_PX` increments: `u = t/nsteps`, convert to `(cx, cy)`
4. Dedup: skip if `(cx,cy) == (prev_cx, prev_cy)`
5. Bounds check; stamp `glyph` with color pair and attribute

---

### bone_glyph(dx, dy)
Purpose: select ASCII character representing bone direction
Steps:
1. `ang = atan2f(-dy, dx)` — negate dy to convert terminal-down to math-up
2. Normalize to `[0°, 180°)` (bones have no orientation, only direction)
3. Map: `[0°,22.5°) or [157.5°,180°)` → `-`, `[22.5°,67.5°)` → `\`, `[67.5°,112.5°)` → `|`, `[112.5°,157.5°)` → `/`

---

### scene_init(sc, cols, rows)
Purpose: place ragdoll in T-pose at screen centre, register 17 constraints
Steps:
1. `memset(sc, 0)`, set gravity = GRAVITY, wind_force = WIND_FORCE
2. `init_platforms(sc->platforms, cols, rows)` — 5 staggered slanted shelves
3. Compute `cx = cols*CELL_W*0.5`, `cy = CEIL_MARGIN + 130` (head clears ceiling)
4. Place 15 particles at (cx + off[i][0], cy + off[i][1])
5. Set `old_pos[i] = pos[i]` (zero velocity), `prev_pos[i] = pos[i]`
6. Call `add_constraint` 17 times; `add_constraint` computes rest length from current pos[]

---

## Pseudocode — Core Loop

```
setup:
  srand(clock_ns() & 0xFFFFFFFF)
  atexit(cleanup)
  signal(SIGINT/SIGTERM) → running = 0
  signal(SIGWINCH) → need_resize = 1
  screen_init()
  scene_init(cols, rows)
  frame_time = clock_ns(); sim_accum = 0

main loop (while running):

  1. handle resize:
     if need_resize:
       screen_resize() → re-read LINES/COLS
       clamp all pos[i] and old_pos[i] to new pixel bounds
       init_platforms() for new dimensions
       reset frame_time, sim_accum = 0
       need_resize = 0

  2. compute dt:
     now = clock_ns()
     dt = now - frame_time
     frame_time = now
     if dt > 100 ms: dt = 100 ms   ← suspend guard

  3. physics accumulator:
     tick_ns = NS_PER_SEC / sim_fps
     dt_sec  = tick_ns / NS_PER_SEC
     sim_accum += dt
     while sim_accum >= tick_ns:
       ragdoll_tick(dt_sec, cols, rows)
       sim_accum -= tick_ns

  4. alpha = sim_accum / tick_ns   [0.0 .. 1.0)

  5. fps counter: 500 ms sliding window

  6. frame cap: sleep(NS_PER_SEC/60 - elapsed) before render

  7. draw + present:
     erase()
     render_ragdoll(alpha)   ← platforms + ground + bones + markers
     HUD bars (top + bottom)
     wnoutrefresh + doupdate

  8. input: getch() loop until ERR

cleanup: endwin()
```

---

## Interactions Between Modules

```
App
 ├── owns Screen (ncurses state, cols/rows)
 ├── owns Scene (Ragdoll + Platforms)
 ├── reads sim_fps (adjustable via [/] keys)
 └── main loop drives everything

Scene
 ├── scene_init()    → places T-pose, computes constraint rest lengths
 ├── scene_tick()    → delegates to ragdoll_tick()
 └── scene_draw()    → delegates to render_ragdoll()

Ragdoll physics (ragdoll_tick):
 ├── calls verlet_update()         — integrate gravity + wind
 ├── calls apply_boundaries()      — clamp + bounce off walls/floor/ceiling
 ├── calls apply_platform_collisions() — slanted surface bounce
 └── calls satisfy_constraint()    — restore all bone lengths (8 iterations)

Rendering (render_ragdoll):
 ├── lerps prev_pos → pos via alpha
 ├── calls draw_bone() × 17        — bone glyphs
 └── stamps particle markers last  — always win over bone glyphs

§4 coords (px_to_cell_x/y)
 └── called ONLY inside draw_bone() and render_ragdoll() — nowhere in physics

Signal handlers
 ├── write running = 0 (SIGINT/SIGTERM)
 └── write need_resize = 1 (SIGWINCH)
 └── both are volatile sig_atomic_t
```

---

## Key Patterns to Internalize

**Verlet integration — velocity is a difference, not a variable:**
Every time you see `vel = pos - old_pos`, you are seeing Verlet's core insight. Constraints, impulses, and bounces all operate by adjusting `pos` or `old_pos` — the velocity corrects itself on the next tick automatically.

**Constraint-based rigidity:**
Rigid bones are enforced by geometry (position correction), not by forces (spring constants). This makes the system numerically stable at any dt because there are no stiff differential equations — only linear algebra on position vectors.

**The interleaved bounce+constraint pipeline:**
`Verlet → boundary → platform → (constraint → platform) × N` is the canonical position-based dynamics pipeline. Separation of integration from constraint satisfaction, with boundary re-enforcement after every constraint pass, prevents tunnelling in all practical scenarios.

**prev_pos as an interpolation snapshot:**
Saved before any physics runs, `prev_pos` holds the state at the END of the previous tick. `render_ragdoll` lerps from `prev_pos` to `pos` using alpha. This produces smooth motion at any render rate with no extrapolation artifacts.

**Old_pos manipulation as velocity injection:**
The wind gust code `old_pos[i].x -= impulse * dt` is not a hack — it is the correct Verlet way to apply an instantaneous velocity kick. Understanding this pattern is essential for adding any kind of impulse-based interaction (kicks, explosions, picking up objects) to a Verlet simulation.
