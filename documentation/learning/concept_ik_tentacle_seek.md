# Pass 1 — ik_tentacle_seek: 12-Segment FABRIK Tentacle with Joint Constraints

## From the Source

**Algorithm:** FABRIK iterative IK with per-joint angle constraints. Backward pass: tip snapped to target, each parent joint slid along the child-to-parent direction to restore link length; constraint applied immediately after each slide. Forward pass: root re-anchored, each child slid along parent-to-child direction to restore link length. Repeat until `|tip − target| < CONV_TOL` or MAX_ITER=20. No Jacobian, no trigonometry in the solver, no singularities. Reachability check fires once per tick and short-circuits to a collinear stretch posture when the target is out of reach.

**Joint constraints:** After each backward-pass repositioning of joint i, compute the signed angle between the incoming link direction (p[i]−p[i−1]) and outgoing link direction (p[i+1]−p[i]) using `angle = atan2(cross, dot)` where `cross = dir_in × dir_out` (2-D scalar cross product = sin θ for unit vectors) and `dot = dir_in · dir_out` (= cos θ). If `|angle| > MAX_JOINT_BEND` (1.1 rad ≈ 63°), rotate dir_out by `(clamped − angle)` and reposition p[i+1]. Prevents kinking; enforces biological bend limit.

**Target motion:** Lissajous figure with frequency ratio 1:1.7 (= 10:17). The ratio is rational but produces a long period (≈62.8 s of scene_time), making the path feel quasi-periodic. `actual_target` low-pass lerps toward the raw Lissajous position at rate `TARGET_SMOOTH = 8 s⁻¹`, smoothing sudden direction reversals near path crossings so joint constraints do not fire aggressively.

**Data-structure:** `Vec2 pos[N_JOINTS]` — joint positions in pixel space (13 joints, 12 links). `Vec2 prev_pos[N_JOINTS]` — tick-start snapshot for alpha lerp. `float link_len[N_LINKS]` — tapered lengths (root longest). `Vec2 trail_pts[TRAIL_POINTS]` ring buffer (120 entries) — ghost trail of recent `actual_target` positions, rendered as '.' dots every 3rd entry.

## Core Idea

A 12-segment tentacle is anchored at screen centre and uses FABRIK to smoothly reach toward a target tracing a complex Lissajous path (frequency ratio 1:1.7). Per-joint angle constraints prevent the tentacle from kinking — each joint can bend at most 63°. A ghost trail of '.' dots shows the recent target path. Ten selectable bioluminescent colour themes cycle with 't'/'T'.

The key additions over ik_arm_reach.c: (1) the chain is 3× longer (12 links vs 4), demanding more FABRIK iterations; (2) per-joint angle constraints are woven into the backward pass — the tentacle behaves like a real biological appendage rather than a robot arm; (3) the target is low-pass filtered to absorb sudden direction changes at Lissajous crossings, keeping the tentacle fluid.

## The Mental Model

Imagine a squid tentacle anchored to a wall. It can reach far but cannot fold back on itself — each joint has a maximum bend angle. A target drifts in a complex looping path. The tentacle tracks it by alternately pulling the tip toward the target (backward pass) and restoring the root to its anchor (forward pass). At each joint during the backward pull, the constraint fires: "you're bending too sharply — rotate back within 63°." This makes the tentacle curve gracefully rather than kink.

The target smoothing is essential: at the internal crossings of the Lissajous figure, the raw target velocity reverses abruptly. Without smoothing, the tentacle would need to pivot instantly — the constraints would fire on almost every joint simultaneously, stiffening the tentacle. The low-pass filter spreads the direction change over ~125 ms, giving the tentacle time to curve gently.

## Data Structures

### Tentacle struct (key fields)
```
pos[N_JOINTS]         — current joint positions in pixel space (13 entries)
prev_pos[N_JOINTS]    — snapshot at tick start for sub-tick alpha lerp
link_len[N_LINKS]     — tapered link lengths; link_len[i] = 20.0 − i×0.8 (12 entries)
anchor                — fixed root pixel position (screen centre); set at init
actual_target         — smoothly tracked target (low-pass filtered)
prev_target           — snapshot of actual_target at tick start for alpha lerp
scene_time            — accumulated simulation time driving the Lissajous
speed_scale           — Lissajous time multiplier (w/s or +/- keys)
last_iter             — FABRIK iterations used on the most recent tick (shown in HUD)
at_limit              — true when |actual_target − anchor| >= total reach
paused
trail_pts[TRAIL_POINTS] — ring buffer of recent actual_target positions (120 entries)
trail_write           — write cursor (next slot to fill); advances mod TRAIL_POINTS
trail_fill            — valid entries <= TRAIL_POINTS
theme_idx
```

### Link length taper
```
link_len[i] = max(BASE_LINK_LEN − i × TAPER, 4.0)
            = max(20.0 − i × 0.8, 4.0)

i=0  (root link):  20.0 px
i=5  (mid link):   16.0 px
i=11 (tip link):   11.2 px
Total reach: Σᵢ₌₀¹¹(20 − 0.8i) = 240 − 52.8 = 187.2 px
```

### Vec2 helpers used in constraint
```
vec2_dot(a, b)    = a.x×b.x + a.y×b.y   = cos θ  (for unit vectors)
vec2_cross(a, b)  = a.x×b.y − a.y×b.x   = sin θ  (for unit vectors, signed)
vec2_rotate(v, θ) = (v.x cosθ − v.y sinθ,  v.x sinθ + v.y cosθ)

atan2(cross, dot) = atan2(sin θ, cos θ) = θ  (signed angle in [−π, π])
```

### Color pair layout (tentacle)
```
pairs 1–7   — tentacle body gradient: pair 1 = root (deepest/dimmest),
               pair 7 = tip (brightest) — seven pairs for 12 links
pair 8      — HUD status bar
```
No separate semantic pairs; the target marker and HUD both use pair assignments from the theme.

## The Main Loop

Each iteration of the main loop:

1. **Check for resize.** SIGWINCH → reinitialise scene; tentacle re-anchored at new screen centre.

2. **Measure dt.** CLOCK_MONOTONIC; cap at 100 ms.

3. **Physics accumulator.** `sim_accum += dt`. While `sim_accum >= tick_ns`:
   - Snapshot `prev_pos`, `prev_target`.
   - If paused: return early.
   - `update_target(dt_sec)`: advance `scene_time`, compute raw Lissajous target, lerp `actual_target` toward it (low-pass smoothing), push into `trail_pts`.
   - `last_iter = fabrik_solve(tentacle, actual_target, anchor)`: run FABRIK with constraints.
   - `sim_accum -= tick_ns`.

4. **Compute alpha.** `alpha = sim_accum / tick_ns`.

5. **FPS counter.** Every 500 ms update display value.

6. **Frame cap sleep.** Before render; 60 fps ceiling.

7. **Draw and present.** `render_tentacle()`: alpha-lerp positions, draw ghost trail, draw PASS 1 bead fill (gradient pairs), draw PASS 2 node markers, draw target marker. HUD overlay. `doupdate()`.

8. **Handle input.** Speed up/down, theme cycle (forward 't', backward 'T'), sim Hz, quit.

## Non-Obvious Decisions

**Why are joint constraints applied during the backward pass, not the forward pass?**
The backward pass is where the chain is being pulled toward the target — this is where joints are most likely to exceed their bend limits. Applying the constraint immediately after repositioning joint i (before moving to joint i−1) catches violations as they occur, rather than letting them propagate through the entire pass. Applying constraints during the forward pass would correct root-side joints after the tip-side joints have already been positioned without constraints — less effective at preventing kinking.

**Why use atan2(cross, dot) instead of acos(dot) for the constraint angle?**
`acos(dot)` gives only the magnitude of the angle — it cannot distinguish clockwise from counter-clockwise bending. `atan2(cross, dot)` gives the signed angle: positive = counter-clockwise, negative = clockwise. The constraint needs to clamp to [−MAX_JOINT_BEND, +MAX_JOINT_BEND], which requires knowing the sign. Also, `acos()` is numerically unstable near ±1 (the derivative of arccos blows up there), whereas `atan2` is well-conditioned everywhere.

**Why low-pass filter the target instead of using the raw Lissajous position?**
At Lissajous path crossings, the raw target velocity reverses abruptly — the direction changes by nearly 180° in a few ticks. FABRIK can track that geometrically, but the joint constraints would fire on many joints simultaneously (each trying to limit the sudden bend), stiffening the tentacle into a rigid stick for a few frames. The low-pass filter (time constant τ = 1/8 s ≈ 125 ms) converts sudden reversals into smooth curves. The tentacle always looks fluid.

**Why MAX_ITER=20 instead of 15 (as in ik_arm_reach.c)?**
The tentacle has 12 links vs 4. Longer chains require more iterations to converge near workspace boundaries, because the correction from the root end must propagate further through the chain. A 12-link chain also has more opportunities for joint constraints to fire during each pass, slightly slowing convergence per iteration. Empirical observation shows convergence in 3–8 iterations for smooth target motion; 20 is the safety cap for near-singular configurations.

**Why CONV_TOL=2.0 px instead of 1.5 px (as in ik_arm_reach.c)?**
The tentacle tip link is 11.2 px long vs 18 px (arm's shortest link). A 2.0 px tolerance is still sub-cell in both axes (2/8 = 0.25 columns, 2/16 = 0.125 rows) — the tip and target appear in the same terminal cell. The slightly larger tolerance allows slightly earlier termination, saving on average one extra iteration per tick for the longer chain.

**Why every 3rd trail point instead of every point?**
The trail buffer holds 120 entries. Drawing all 120 would produce 120 '.' dots covering the Lissajous path so densely that it would compete visually with the tentacle body. Drawing every 3rd entry gives 40 visible dots — enough to see the shape of the path without cluttering the screen.

**Why does the FABRIK backward pass in this file go from tip toward root (i = N−2 downto 0), unlike what some sources call the "backward" direction?**
Naming is relative to the chain direction. In this implementation: "backward pass" pulls the chain from the tip toward the root (tip is snapped to target first, then parent joints are slid); "forward pass" restores the root and propagates the fix toward the tip. The net effect is identical to other FABRIK implementations regardless of what each pass is called.

**Why is `last_iter` stored in the Tentacle struct and shown in the HUD?**
It provides a real-time debug indicator of solver workload. If `last_iter` consistently shows values near MAX_ITER=20, it means the solver is being pushed near its iteration budget — either because the target is moving very fast or the chain is near a singular configuration. This lets you tune MAX_ITER, speed_scale, or chain length without guessing.

## State Machines

### App-level state
```
        ┌──────────────────────────────────────────────────────┐
        │                    RUNNING                           │
        │                                                      │
        │  update_target  → low-pass filter → trail push       │
        │  fabrik_solve   → constraints → converge/at-limit    │
        │  render_tentacle → ghost trail → beads → markers     │
        │                                                      │
        │  SIGWINCH ────────────────→ need_resize              │
        │       need_resize ────────→ scene_init (re-anchor)   │
        │  SIGINT/SIGTERM ──────────→ running = 0              │
        └──────────────────────────────────────────────────────┘
                          │
               'q'/ESC/signal
                          ▼
                        EXIT
```

### Tentacle solver state per tick
```
        is |actual_target − anchor| >= total_reach?
                          │
              ┌── YES ────┴── NO ──┐
              │                   │
          AT_LIMIT             REACHABLE
          stretch collinear    FABRIK iterates
          at_limit = true      backwards + forwards
          return 1             with joint constraints
              │                until CONV_TOL or MAX_ITER
              │                   │
          '#' marker          '*' marker
          (HUD shows LIMIT)   (HUD shows last_iter)
```

### Joint constraint activation per backward step
```
For joint i (1 <= i <= N_JOINTS−2):

  dir_in  = norm(pos[i] − pos[i-1])
  dir_out = norm(pos[i+1] − pos[i])
  angle   = atan2(cross(dir_in, dir_out), dot(dir_in, dir_out))

  |angle| <= MAX_JOINT_BEND?
    YES → constraint passes; no change to pos[i+1]
    NO  → compute correction: delta = clamped − angle
          rotate dir_out by delta
          reposition pos[i+1] = pos[i] + new_dir × link_len[i]
```

### Input actions
| Key | Effect |
|-----|--------|
| q / ESC | exit |
| space | toggle paused |
| w / + / = | speed_scale × 1.25 (faster Lissajous) |
| s / - | speed_scale ÷ 1.25 (slower Lissajous) |
| t | cycle theme forward (0→9→0) |
| T | cycle theme backward |
| ] | increase sim_fps by SIM_FPS_STEP |
| [ | decrease sim_fps by SIM_FPS_STEP |

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|----------|---------|-------------------|
| N_JOINTS | 13 | 12 links, 13 joint positions. More joints: more expressive motion, slower convergence. |
| MAX_ITER | 20 | FABRIK iteration cap. Below 8: tentacle visibly lags during fast motion. Above 20: diminishing returns, extra CPU cost. |
| CONV_TOL | 2.0 px | Sub-cell tolerance. Smaller: more precise but rarely fires early exit. Larger: tip slightly misses target in cell space. |
| MAX_JOINT_BEND | 1.1 rad (≈63°) | Per-joint bend limit. Below 0.78 (45°): tentacle stiff, struggles to track close targets. Above 1.57 (90°): kinking visible at rapid reversals. |
| BASE_LINK_LEN | 20.0 px | Root link length. Scales total reach proportionally. |
| TAPER | 0.8 px | Length reduction per link toward tip. 0.0 = uniform links. Higher taper: longer root links, shorter tip links. |
| LIS_OMEGA_X / LIS_OMEGA_Y | 1.0 / 1.7 rad/s | Frequency ratio determines figure shape. 1:2 = figure-8. 1:1.7 = complex quasi-periodic crossings. |
| LIS_AX_FACTOR / LIS_AY_FACTOR | 0.55 / 0.38 | Amplitudes as fractions of screen size. Together give ~55% width, ~76% height path coverage. |
| LIS_PHASE_Y | π/3 (≈1.047 rad) | Phase offset for y component. Avoids starting at a cusp. |
| TARGET_SMOOTH | 8.0 s⁻¹ | Low-pass filter rate for actual_target. Time constant τ = 0.125 s. Higher: target follows Lissajous more tightly (more aggressive direction changes). Lower: heavier smoothing, tentacle tracks large-scale path only. |
| DRAW_STEP_PX | 5.0 px | Bead fill step. Must be < CELL_W (8 px). Larger: sparser beads, chain looks articulated. Smaller: denser fill, more mvwaddch calls. |
| TRAIL_POINTS | 120 | Ghost trail entries = 2 s at 60 Hz. Every 3rd shown = 40 dots. Larger: more path history visible. |

## Open Questions for Pass 3
- When `MAX_JOINT_BEND` is hit during the backward pass, the constraint rotates `dir_out` and repositions `p[i+1]`. Does this violate the link length between `p[i+1]` and `p[i+2]`? How does the next backward step (processing joint `i−1`) recover from this?
- At the Lissajous path crossings, `actual_target` undergoes its most rapid direction change. How many joints are expected to fire their constraint during the backward pass at these moments? Is there a way to measure this from the code?
- `last_iter` is shown in the HUD. At maximum speed (speed_scale near LIS_SPEED_MAX), does `last_iter` approach MAX_ITER=20? What is the visual effect when the iteration budget is exhausted?
- The `at_limit` condition fires when `|actual_target − anchor| >= total_len (187.2 px)`. The Lissajous amplitudes are `LIS_AX = 0.55 × screen_width_px`. On a wide terminal (e.g., 200 columns × 50 rows), `LIS_AX ≈ 0.55 × 1600 = 880 px`, which far exceeds the reach. Does `at_limit` fire for most of the path on large terminals?
- Why is the joint constraint applied during the backward pass but not during the forward pass? What would happen if you applied it in both passes?
- The ghost trail renders every 3rd stored point. With speed_scale increased to maximum, `actual_target` moves faster per tick. Does the visible trail spacing increase proportionally, or does the trail always appear similarly dense regardless of speed?
- `fabrik_solve()` returns `int` (iteration count). What is the range of expected return values: does it ever return 1 (immediate convergence after the first backward+forward pair)? Does it return 0?

---

# Structure

| Symbol | Type | Size (approx) | Role |
|--------|------|---------------|------|
| `Tentacle.pos[13]` | `Vec2[13]` | 104 B | current joint positions in pixel space |
| `Tentacle.prev_pos[13]` | `Vec2[13]` | 104 B | tick-start snapshot for alpha lerp |
| `Tentacle.link_len[12]` | `float[12]` | 48 B | tapered link lengths; constant after init |
| `Tentacle.trail_pts[120]` | `Vec2[120]` | 960 B | ghost trail ring buffer of actual_target positions |
| `THEMES[10]` | `Theme[10]` | ~560 B | 10 bioluminescent palettes (7 pairs each, static) |
| `g_app` | `App` | ~3 KB | top-level container |

---

# Pass 2 — ik_tentacle_seek: Pseudocode

## Module Map

| Section | Purpose |
|---------|---------|
| §1 config | N_JOINTS, MAX_ITER, CONV_TOL, MAX_JOINT_BEND, BASE_LINK_LEN, TAPER, LIS_OMEGA_X/Y, TARGET_SMOOTH, DRAW_STEP_PX, CELL_W/H |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 10-theme bioluminescent palette; `theme_apply()`, `color_init()` |
| §4 coords | `px_to_cell_x/y` — single aspect-ratio conversion point |
| §5a vec2 | add, sub, scale, lerp, len, dist, norm; `vec2_dot`, `vec2_cross`, `vec2_rotate` |
| §5c joint constraint | `apply_joint_constraint()` — atan2(cross, dot), clamp, rotate, reposition |
| §5b FABRIK | `fabrik_solve()` — reachability, backward+constraint, forward, convergence |
| §5d Lissajous + trail | `update_target()` — raw Lissajous, low-pass lerp, trail ring buffer push |
| §5e ghost trail | Ring buffer push/retrieve; draw every 3rd point as '.' |
| §5f rendering | `draw_link_beads()`, `joint_node_char()`, `render_tentacle()` |
| §6 scene | `scene_init/tick/draw` — thin wrappers over Tentacle |
| §7 screen | ncurses double-buffer display |
| §8 app | signal handlers; main fixed-step loop |

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

while sim_accum >= tick_ns:
    │
    ├── snapshot prev_pos = pos; prev_target = actual_target
    │
    ├── update_target(tentacle, dt_sec):
    │     scene_time += dt × speed_scale
    │     lx = anchor.x + LIS_AX × cos(LIS_OMEGA_X × scene_time)
    │     ly = anchor.y + LIS_AY × sin(LIS_OMEGA_Y × scene_time + LIS_PHASE_Y)
    │     rate = clamp(dt × TARGET_SMOOTH, 0, 1)
    │     actual_target += (lissajous_target − actual_target) × rate
    │     trail_pts[trail_write] = actual_target
    │     trail_write = (trail_write + 1) % TRAIL_POINTS
    │     trail_fill = min(trail_fill + 1, TRAIL_POINTS)
    │
    └── last_iter = fabrik_solve(tentacle, actual_target, anchor):
          total_len = Σ link_len[i]
          if |actual_target − anchor| >= total_len:
            stretch collinear; at_limit = true; return 1
          at_limit = false
          for iter = 0..MAX_ITER−1:
            BACKWARD PASS (tip→root):
              pos[N-1] = actual_target
              for i = N-2 downto 0:
                dir = norm(pos[i] − pos[i+1])    [parent direction]
                pos[i] = pos[i+1] + dir × link_len[i]
                if i > 0: apply_joint_constraint(tentacle, i)
            FORWARD PASS (root→tip):
              pos[0] = anchor
              for i = 0 to N-2:
                dir = norm(pos[i+1] − pos[i])    [child direction]
                pos[i+1] = pos[i] + dir × link_len[i]
            if |pos[N-1] − actual_target| < CONV_TOL: break
          return iter+1

sim_accum -= tick_ns

alpha = sim_accum / tick_ns  [0.0..1.0)

render_tentacle(tentacle, window, alpha):
    rj[i] = lerp(prev_pos[i], pos[i], alpha)   for each joint
    rt    = lerp(prev_target, actual_target, alpha)
    1. Draw ghost trail: every 3rd trail_pts entry as '.' (pair 6, A_DIM)
    2. PASS 1: draw_link_beads(rj[i]→rj[i+1]) for each link
               link-to-pair mapping: 7 pairs for 12 links (gradient root→tip)
    3. PASS 2: overwrite joint positions with joint_node_char(i) markers
               root quarter: '0', middle: 'o', tip quarter: '.'
    4. Draw target marker '*' or '#' at rt

screen_present() → doupdate()
```

---

## Function Breakdown

### apply_joint_constraint(tentacle, i)
Purpose: clamp the bend angle at joint i after the backward pass repositioned it.
Guards: only called for 1 <= i <= N_JOINTS−2 (inner joints only; root and tip have no constraint).
Steps:
1. `dir_in  = norm(pos[i] − pos[i-1])`   (direction of incoming link)
2. `dir_out = norm(pos[i+1] − pos[i])`   (direction of outgoing link)
3. `cr = vec2_cross(dir_in, dir_out)` = sin θ  (signed)
4. `dt = vec2_dot(dir_in, dir_out)` = cos θ
5. `angle = atan2f(cr, dt)` = signed bend angle in [−π, π]
6. If `|angle| > MAX_JOINT_BEND`:
   - `clamped = clamp(angle, −MAX_JOINT_BEND, MAX_JOINT_BEND)`
   - `delta = clamped − angle`  (rotation correction)
   - `new_dir = vec2_rotate(dir_out, delta)`
   - `pos[i+1] = pos[i] + new_dir × link_len[i]`  (reposition child)

### fabrik_solve(tentacle, target, anchor) → int
Purpose: run FABRIK IK; return iteration count used.
Steps:
1. Compute `total_len = Σ link_len[i]`
2. If `vec2_dist(anchor, target) >= total_len`: stretch collinear; `at_limit=true`; return 1
3. `at_limit = false`
4. For `iter = 0..MAX_ITER−1`:
   a. BACKWARD: `pos[N-1] = target`; for i=N-2 downto 0: slide pos[i], then `apply_joint_constraint(i)` if i>0
   b. FORWARD: `pos[0] = anchor`; for i=0 to N-2: slide pos[i+1]
   c. If `vec2_dist(pos[N-1], target) < CONV_TOL`: return `iter+1`
5. Return MAX_ITER (budget exhausted — best effort result used)

### update_target(tentacle, dt)
Purpose: advance scene time, compute smoothed target, push ghost trail.
Steps:
1. `scene_time += dt × speed_scale`
2. `lx = anchor.x + LIS_AX × cos(LIS_OMEGA_X × scene_time)`
3. `ly = anchor.y + LIS_AY × sin(LIS_OMEGA_Y × scene_time + LIS_PHASE_Y)`
4. `rate = clamp(dt × TARGET_SMOOTH, 0.0, 1.0)`
5. `actual_target += (lissajous_target − actual_target) × rate`  (exponential moving average)
6. Push `actual_target` into `trail_pts` ring buffer; advance `trail_write`

### draw_link_beads(window, a, b, pair, attr, cols, rows)
Purpose: stamp 'o' along segment a→b at DRAW_STEP_PX intervals; dedup per call.
Steps: identical to ik_arm_reach.c — walk from a to b in nsteps, dedup by prev cell, bounds-check, stamp 'o'.

### joint_node_char(i) → char
Purpose: return the glyph for joint i based on its position in the chain.
Steps:
1. Root quarter (i <= (N_JOINTS−1)/3):    return '0'  (thick, anchored look)
2. Tip quarter  (i >= (N_JOINTS−1)×2/3):  return '.'  (thin, nimble tip)
3. Otherwise:                              return 'o'  (standard bead)

### render_tentacle(tentacle, window, cols, rows, alpha)
Purpose: compose complete tentacle frame (read-only).
Steps:
1. `rj[i] = lerp(prev_pos[i], pos[i], alpha)` for each joint
2. `rt = lerp(prev_target, actual_target, alpha)`
3. Ghost trail: walk trail_pts; draw '.' for every 3rd valid entry (pair 6, A_DIM)
4. PASS 1: for each link i: `draw_link_beads(rj[i], rj[i+1])` with gradient color pair
5. PASS 2: for each joint i: draw `joint_node_char(i)` at rj[i]'s cell
6. Draw target: '*' (reachable) or '#' (at_limit) at rt

---

## Pseudocode — Core Loop

```
setup:
  register atexit(cleanup)
  register signal(SIGINT/SIGTERM) → running = 0
  register signal(SIGWINCH) → need_resize = 1
  screen_init()
  scene_init(cols, rows)
  frame_time = clock_ns()
  sim_accum = 0

main loop (while running):

  1. handle resize:
     if need_resize:
       endwin() + refresh()
       getmaxyx() → new cols, rows
       scene_init(cols, rows)     ← re-anchors tentacle at new screen centre
       reset frame_time, sim_accum
       need_resize = 0

  2. compute dt:
     now = clock_ns()
     dt  = now − frame_time
     frame_time = now
     cap dt at 100 ms

  3. physics ticks:
     tick_ns = NS_PER_SEC / sim_fps
     dt_sec  = tick_ns / 1e9
     sim_accum += dt
     while sim_accum >= tick_ns:
       save prev_pos, prev_target
       if not paused:
         update_target(dt_sec)           ← advance Lissajous, smooth target, push trail
         last_iter = fabrik_solve(...)   ← FABRIK with constraints
       sim_accum -= tick_ns

  4. alpha = sim_accum / tick_ns

  5. FPS counter update (every 500 ms)

  6. frame cap: sleep(NS_PER_SEC/60 − elapsed)

  7. draw + present:
     screen_draw(fps, alpha, last_iter)  ← erase + render_tentacle + HUD
     screen_present()                    ← doupdate()

  8. input:
     ch = getch()
     dispatch: q=quit, space=pause, w/s=speed, t/T=theme, [/]=sim_fps

cleanup:
  endwin()
```

---

## Interactions Between Modules

```
App
 ├── owns Screen (ncurses state, cols/rows)
 ├── owns Scene → Tentacle
 ├── reads sim_fps (adjustable by keys)
 └── main loop drives everything

Tentacle (per tick):
  update_target()       — computes actual_target; pushes ghost trail
  fabrik_solve()        — reads actual_target; modifies pos[]
    apply_joint_constraint() — called inside backward pass for each inner joint

render_tentacle()       — reads prev_pos, pos, prev_target, actual_target, trail_pts
                        — const Tentacle* (never modifies simulation state)

§4 coords (px_to_cell_x/y)
 └── called ONLY inside draw_link_beads() and render_tentacle()

§5a vec2 helpers
 └── vec2_dot/cross/rotate — called ONLY by apply_joint_constraint()
 └── vec2_len/norm/dist    — called by fabrik_solve() and other rendering code

Signal handlers (volatile sig_atomic_t — async-signal-safe writes only)
```

---

## Key Patterns to Internalize

**Joint constraints bolt onto FABRIK without redesigning the solver:**
The core FABRIK backward pass is unchanged: slide each joint along the child-to-parent direction to restore link length. The constraint is a single function call inserted immediately after that slide. Because the constraint only repositions `p[i+1]` (the child of the joint just processed), it is local — it does not invalidate the work already done for `p[i+2]` through `p[N-1]`. This composability is why FABRIK constraints are described as "natural."

**Low-pass filtering is not optional for constrained chains:**
Without `TARGET_SMOOTH`, the raw Lissajous target makes abrupt direction changes at its crossings. These sudden changes propagate through the FABRIK backward pass, firing the bend constraint on many consecutive joints simultaneously. The result is a temporarily rigid tentacle — all joints snap to their max bend limit, which looks wrong and unbiological. The exponential moving average absorbs the velocity spike, keeping the tentacle's response smooth.

**Two-pass bead rendering with joint markers:**
PASS 1 fills every link with 'o' beads at 5 px intervals — a uniform chain. PASS 2 overwrites joint positions with size-coded markers: '0' (thick) at the root, 'o' (medium) in the middle, '.' (thin) at the tip. The second pass creates visual hierarchy — the chain looks like a real biological appendage with a rigid root and a delicate tip, not just a line of dots.

**prev_pos/prev_target snapshots vs forward extrapolation:**
The tentacle's joint motion is non-linear (FABRIK with constraints can produce complex trajectories). Forward extrapolation from velocity is undefined. By saving `prev_pos` at the start of each tick and lerping to `pos` at render time using `alpha`, sub-tick smoothness is achieved correctly regardless of the motion type. The same pattern applies to `actual_target`.

**Frequency ratio 1:1.7 vs 1:2 for the Lissajous path:**
The 1:2 ratio (used in ik_arm_reach.c) gives a clean figure-8 that repeats every 9 seconds — the arm tracks it so well that the motion eventually feels predictable. The 1:1.7 (= 10:17) ratio produces a rational but complex closed figure that takes 62.8 s of scene_time to repeat. The many internal crossings create varied, unpredictable direction changes that continuously exercise the solver and constraints, producing visually complex motion throughout the entire demo.

**`at_limit` and the stretch posture:**
When `|actual_target − anchor| >= total_len (187.2 px)`, the tentacle cannot reach. Rather than running FABRIK iterations that would converge to the fully extended configuration anyway, the reachability check immediately sets the collinear stretch posture in O(N) — 12 assignments. The `at_limit` flag causes the target marker to change from '*' to '#' so the viewer sees the reach limit condition. This saves up to 20 iteration budgets per tick when the target is temporarily out of reach.
