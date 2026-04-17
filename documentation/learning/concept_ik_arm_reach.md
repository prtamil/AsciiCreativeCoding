# Pass 1 — ik_arm_reach: FABRIK Robotic Arm with Lissajous Figure-8 Target

## From the Source

**Algorithm:** FABRIK iterative IK. Two geometric passes per iteration: (1) FORWARD — tip snapped to target, link lengths restored in the tip→root direction; (2) BACKWARD — root re-anchored, link lengths restored root→tip. Each iteration strictly reduces tip error. Converges in 3–5 iterations for a 4-link chain; MAX_ITER=15 caps degenerate-configuration cost. No Jacobian matrix, no trigonometry, no singularities. Reachability check (`|target−root| vs Σ link_len`) fires once per tick and short-circuits to a straight-stretch posture when the target is out of reach.

**Math:** Lissajous figure-8: `x(t) = root_px + lis_ax × cos(LIS_FX × scene_time)`, `y(t) = root_py + lis_ay × sin(LIS_FY × scene_time + LIS_PHASE)`. Frequency ratio LIS_FX=1 : LIS_FY=2 creates a figure-8 because the y-axis completes two oscillations per x-cycle, crossing the centre from opposite vertical directions once — producing one self-intersection (a figure-8). Phase LIS_PHASE=π/4 converts the tangent cusp into a clean X crossing.

**Performance:** Fixed-step accumulator decouples physics (60 Hz default) from render (capped at 60 fps). FABRIK worst-case: MAX_ITER=15 × N_JOINTS=5 × 2 passes = 150 simple arithmetic ops per tick — trivial at 60 Hz. ncurses doupdate() sends only changed cells; typically fewer than 50 per frame.

**Data-structure:** `Vec2 pos[N_JOINTS]` — current joint positions (N=5 joints, 4 links). `Vec2 prev_pos[N_JOINTS]` — snapshot at tick start for sub-tick alpha lerp. Lissajous trail ring buffer (TRAIL_SIZE=60 entries) stores recent target positions for the faint figure-8 path display. All positions in square pixel space; converted to cell coordinates only at draw time.

## Core Idea

A 4-link robotic arm is anchored at screen centre. Its end effector tracks a target tracing a Lissajous figure-8 path autonomously. The FABRIK iterative IK solver bends the chain each tick to track it. When the target moves beyond total chain reach, the arm straightens toward it (maximum-extension configuration) and a yellow limit circle appears showing exactly how far the arm can extend. A faint red trail of recent target positions reveals the figure-8 shape.

The central insight: IK for chains longer than 2 links has infinitely many solutions. Rather than solving a Jacobian matrix (O(N³), fragile near singularities), FABRIK alternates between two O(N) geometric passes — pulling the tip to the target and restoring the root — and converges monotonically in a few iterations.

## The Mental Model

Imagine a chain of rigid rods connected by hinges. You hold the last rod's tip at a target point. This drags all the other rods along behind it (forward pass), but the root end wanders away from its socket. You then snap the root back into its socket, which propagates a correction forward (backward pass). The tip is now slightly displaced from the target — but closer than before. You repeat until the tip is within a pixel. That is FABRIK.

The Lissajous figure-8 is the motion path. Set x to oscillate at one frequency and y at twice that frequency: as x goes right-then-left once, y goes up-down-up-down twice. The two oscillations together trace the two lobes of a figure-8, crossing at the centre exactly once per cycle.

## Data Structures

### Arm struct (key fields)
```
pos[N_JOINTS]         — current joint positions in pixel space (Vec2, 5 entries)
prev_pos[N_JOINTS]    — snapshot at tick start; used for sub-tick alpha lerp
link_len[N_LINKS]     — tapered link lengths in pixels (4 entries, root longest)
total_len             — Σ link_len; reachability threshold
target                — current Lissajous target pixel position
prev_target           — snapshot for alpha lerp of target marker
scene_time            — accumulated simulation time driving the Lissajous
speed_scale           — Lissajous time multiplier (+ / - keys)
root_px, root_py      — root anchor pixel position (Lissajous centre)
lis_ax, lis_ay        — Lissajous amplitudes (set from terminal size at init)
at_limit              — true when target is outside reach sphere
trail[TRAIL_SIZE]     — ring buffer of recent target positions (60 entries)
trail_head            — write cursor (index of most recently pushed entry)
trail_count           — valid entries, <= TRAIL_SIZE
```

### Vec2
```
float x, y            — position in pixel space; x increases eastward,
                         y increases downward (terminal convention)
```
All positions, directions, and distances use pixel space. Cell coordinates appear only inside `px_to_cell_x/y()`.

### Color pair layout
```
pairs 1–5   — arm gradient: pair 1 = root (darkest), pair 5 = tip (brightest)
pair 6      — target marker (bright red — semantic; never changes with theme)
pair 7      — reach limit circle (yellow — semantic; never changes with theme)
pair 8      — HUD status bar (varies per theme)
```

## The Main Loop

Each iteration of the main loop:

1. **Check for resize.** If SIGWINCH fired, call `endwin` + `refresh`, re-read terminal size, reinitialise the scene with new dimensions and reset timing state.

2. **Measure elapsed time (dt).** Read CLOCK_MONOTONIC. Subtract previous frame timestamp. Cap dt at 100 ms to prevent spiral-of-death after OS suspend.

3. **Run the physics accumulator.** Add dt to `sim_accum`. While `sim_accum >= tick_ns`: save `prev_pos` snapshot, call `update_target()` to advance `scene_time` and compute new Lissajous target, call `fabrik_solve()` to move joints toward target, subtract one tick from `sim_accum`.

4. **Compute alpha.** `alpha = sim_accum / tick_ns` — fractional progress toward the next tick; used by renderer to interpolate between `prev_pos` and `pos`.

5. **Update FPS counter.** Every 500 ms, compute frames/elapsed for the HUD display.

6. **Frame cap sleep.** Sleep until 1/60 s has elapsed. Sleeping before rendering keeps terminal I/O latency off the physics budget.

7. **Draw and present.** `erase()` clears the back buffer. `render_arm()` draws the alpha-interpolated arm, trail, optional reach circle, and target marker. HUD drawn on top. `wnoutrefresh` + `doupdate` for one atomic write.

8. **Handle input.** `getch()` in nodelay mode. Dispatch keys for quit, pause, speed adjust, theme cycle, sim Hz change.

## Non-Obvious Decisions

**Why FABRIK instead of the Jacobian pseudo-inverse?**
The Jacobian approach requires computing and inverting a matrix of partial derivatives ∂tip/∂θᵢ — that is O(N³) per tick and becomes numerically unstable near singularities (arm nearly fully extended or folded). FABRIK needs only vector arithmetic, runs O(N) per pass, has no singularity issues (all ops are well-defined vector lengths and normalise), and converges in 3–5 passes for a 4-link chain.

**Why does the forward pass come first in this file's order (tip→root) but some FABRIK papers do it backward→forward?**
The naming in ik_arm_reach.c follows the operation direction: FORWARD pass moves from tip toward root (dragging joints along); BACKWARD pass restores the root and pushes joints forward. The result is the same regardless of naming convention — one pass fixes the tip, the other fixes the root.

**Why taper the link lengths (0.32, 0.27, 0.23, 0.18)?**
Constant-length links look mechanical and uniform — all joints contribute equally to bending. Tapering toward the tip mimics the biomechanical structure of real limbs (upper arm > forearm > hand > finger). Mathematically, the tapering also makes FABRIK converge faster: the final correction step uses the shortest link, so residual error after each forward pass is smaller, reaching CONV_TOL in fewer iterations.

**Why is CONV_TOL = 1.5 px chosen to be sub-cell?**
1.5 / CELL_W (8 px) = 0.19 of a column; 1.5 / CELL_H (16 px) = 0.09 of a row. Once the tip is within 1.5 px of target, both occupy the same terminal cell — further solver iterations produce no visible improvement and waste CPU.

**Why store prev_pos and use alpha lerp instead of forward extrapolation (as in bounce_ball)?**
In bounce_ball, forward extrapolation (`draw_px = px + vx*alpha*dt`) works because ball velocity is constant between ticks. FABRIK-solved joint positions are non-linear — the tip and interior joints can change direction abruptly as the solver converges. Forward extrapolation from velocity is undefined here; prev_pos gives an exact snapshot of the previous tick's position for a correct lerp.

**Why are the reach-circle dots spaced with 48 samples?**
Drawing every pixel on the circle would stamp hundreds of adjacent cells, many mapping to the same terminal cell (wasteful, noisy). 48 samples at 7.5° spacing gives arc spacing of R×2π/48 ≈ R/7.6 px. For a typical arm reach of ~300 px, that is ~39 px ≈ 5 cell-widths — a clearly dashed circle without dot overlap.

**Why are pairs 6 and 7 not part of the theme system?**
Pairs 6 (target, red) and 7 (reach circle, yellow) carry specific semantic meaning to the viewer. Cycling themes changes only pairs 1–5 (arm gradient) and pair 8 (HUD). Separating semantic pairs from theme pairs ensures theme cycling never accidentally changes the visual meaning of "target unreachable" or "here is your reach boundary".

**Why does update_target() push into the trail BEFORE fabrik_solve()?**
The trail records where the target has been. Pushing before solving guarantees the trail shows the exact positions FABRIK was asked to reach — even the out-of-reach positions. If the push happened after, a skipped solve (due to pausing) would still record the position, which is correct.

## State Machines

### App-level state
```
        ┌──────────────────────────────────────────┐
        │               RUNNING                    │
        │                                          │
        │  [update_target] → advance scene_time    │
        │  [fabrik_solve]  → bend chain to target  │
        │  [render_arm]    → alpha-lerp + draw      │
        │                                          │
        │  SIGWINCH ─────────────→ need_resize      │
        │       need_resize ─────→ do_resize()      │
        │                                          │
        │  SIGINT/SIGTERM ───────→ running = 0      │
        └──────────────────────────────────────────┘
                          │
               'q'/ESC/signal
                          │
                          v
                        EXIT
                      (endwin)
```

### Arm reachability state
```
          target within total_len?
                    │
          ┌── YES ──┴── NO ──┐
          │                  │
     REACHABLE           AT_LIMIT
     FABRIK iterates     arm straightens
     until CONV_TOL      toward target
     or MAX_ITER         at_limit = true
          │                  │
     '+' marker         'X' marker
     (red, bold)        (red, bold)
     no circle          reach circle drawn
                        (yellow dashed)
```

### Input actions
| Key | Effect |
|-----|--------|
| q / Q / ESC | running = 0 → exit |
| space | toggle paused |
| + / = | speed_scale × 1.25 (faster figure-8) |
| - | speed_scale ÷ 1.25 (slower figure-8) |
| t | cycle theme forward (0→9→0) |
| ] | increase sim_fps by SIM_FPS_STEP |
| [ | decrease sim_fps by SIM_FPS_STEP |

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|----------|---------|-------------------|
| N_JOINTS | 5 | 4 links, 5 joint positions. Increase: more lifelike arm, FABRIK converges slower (more joints to satisfy). |
| MAX_ITER | 15 | Safety cap on FABRIK iterations per tick. Decrease below 5: arm visibly lags fast targets; arm may not fully converge. |
| CONV_TOL | 1.5 px | Early-exit threshold. Smaller: more precise but wastes iterations on sub-pixel corrections. Larger: tip visibly misses target. |
| LIS_FX / LIS_FY | 1.0 / 2.0 | Frequency ratio determines path shape. 1:2 = figure-8. Try 1:3 for a clover shape. |
| LIS_PHASE | π/4 (0.785 rad) | Phase offset for y sinusoid. 0 = tangent cusp (looks like two touching ovals). π/4 = clean X crossing. |
| LIS_SPEED_DEFAULT | 0.7 | scene_time advancement rate. One full figure-8 = 2π/0.7 ≈ 9 s. Increase: faster arm motion, harder IK tracking. |
| DRAW_STEP_PX | 5 px | Bead fill step size. Must be < CELL_W (8 px). Larger: sparser beads, gaps visible. Smaller: denser, more mvwaddch calls per frame. |
| link_len weights | 0.32/0.27/0.23/0.18 | Taper ratio. Uniform weights (0.25 each) make all links identical — looks mechanical. |
| TRAIL_SIZE | 60 | Ring buffer capacity. At 60 Hz = 1 s of history ≈ 1/9 of the figure-8 cycle. |

## Open Questions for Pass 3
- When the arm is paused, `scene_time` stops advancing and `prev_pos` = `pos`. Alpha lerp still runs — what does the viewer see? Does the arm hold its exact position or drift?
- What happens when speed_scale is set very high (near LIS_SPEED_MAX=5.0)? Does FABRIK still converge within MAX_ITER=15? When does the arm start visibly lagging?
- The FABRIK forward pass in this file moves tip→root, while the backward pass moves root→tip. Some references call these the opposite. Why does the naming not matter for correctness?
- The `lis_ax` and `lis_ay` amplitudes are clipped at `total_len × 0.9` during scene_init. What would happen without this clipping — would the arm ever reach the target, or would it always be at_limit?
- During the degenerate-joint guard (`if (flen < 1e-6f) flen = 1e-6f`), the resulting `r = link_len / 1e-6` is enormous. This pushes the joint very far. Does the solver recover on the next iteration? What does the arm look like during this one-tick anomaly?
- `render_arm()` is declared `const Arm *a` — it never modifies simulation state. If it did accidentally write to `a`, what class of bug could result? Why is this const qualifier valuable at a project level?

---

# Structure

| Symbol | Type | Size (approx) | Role |
|--------|------|---------------|------|
| `Arm.pos[5]` | `Vec2[5]` | 40 B | current joint positions in pixel space |
| `Arm.prev_pos[5]` | `Vec2[5]` | 40 B | tick-start snapshot for sub-tick alpha lerp |
| `Arm.link_len[4]` | `float[4]` | 16 B | tapered link lengths; constant after init |
| `Arm.trail[60]` | `Vec2[60]` | 480 B | ring buffer of recent target positions |
| `THEMES[10]` | `Theme[10]` | ~400 B | selectable color palettes (static, read-only) |
| `g_app` | `App` | ~2 KB | top-level container: scene + screen + control flags |

---

# Pass 2 — ik_arm_reach: Pseudocode

## Module Map

| Section | Purpose |
|---------|---------|
| §1 config | All tunables: SIM_FPS, N_JOINTS, MAX_ITER, CONV_TOL, LIS_FX/FY, TRAIL_SIZE, DRAW_STEP_PX, CELL_W/H |
| §2 clock | `clock_ns()` — monotonic nanosecond timestamp; `clock_sleep_ns()` — precise sleep |
| §3 color | 10-theme palettes; `theme_apply()` — live pair rebind; fixed semantic pairs 6 (red) and 7 (yellow) |
| §4 coords | `px_to_cell_x/y` — the single aspect-ratio conversion point |
| §5a vec2 | `vec2_len`, `vec2_norm` — used heavily in the FABRIK inner loop |
| §5b FABRIK | `fabrik_solve()` — forward + backward passes, reachability check, convergence test |
| §5c target | `update_target()` — Lissajous formula + trail ring buffer push |
| §5d render helpers | `draw_link_beads()` — bead fill per link; `draw_reach_circle()` — dashed yellow circle |
| §5e render_arm | `render_arm()` — full frame: trail, circle, pass-1 beads, pass-2 joint markers, target marker |
| §6 scene | `scene_init/tick/draw` — thin wrappers over Arm |
| §7 screen | ncurses double-buffer display: `screen_init`, `screen_draw`, `screen_present` |
| §8 app | Signal handlers; main fixed-step loop |

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
    ├─── SNAPSHOT:  prev_pos = pos;  prev_target = target
    │
    ├─── update_target(arm, dt_sec):
    │       scene_time += dt × speed_scale
    │       target.x = root_px + lis_ax × cos(LIS_FX × scene_time)
    │       target.y = root_py + lis_ay × sin(LIS_FY × scene_time + LIS_PHASE)
    │       trail[trail_head] = target  (ring buffer push)
    │
    └─── fabrik_solve(arm):
            dist = |target − root|
            if dist > total_len:
              → stretch all links collinear toward target
              → at_limit = true; return
            else:
              for iter = 0..MAX_ITER:
                FORWARD PASS (tip→root):
                  pos[N-1] = target
                  for i = N-2 down to 0:
                    dir = pos[i] − pos[i+1]  (normalised)
                    pos[i] = pos[i+1] + dir × link_len[i]
                BACKWARD PASS (root→tip):
                  pos[0] = root
                  for i = 0 to N-2:
                    dir = pos[i+1] − pos[i]  (normalised)
                    pos[i+1] = pos[i] + dir × link_len[i]
                if |pos[N-1] − target| < CONV_TOL: break

sim_accum -= tick_ns
    │
    ▼
alpha = sim_accum / tick_ns    [0.0 .. 1.0)
    │
    ▼
render_arm(arm, window, alpha):
    rj[i] = lerp(prev_pos[i], pos[i], alpha)   for each joint
    rt    = lerp(prev_target, target, alpha)
    draw trail '.' dots (pair 6, A_DIM)
    if at_limit: draw_reach_circle(root, total_len, pair 7, A_DIM)
    PASS 1: draw_link_beads(rj[i]→rj[i+1]) for each link (gradient pairs 1–5)
    PASS 2: draw joint markers '0'/'o'/'.' at rj[i] (overwrite beads)
    draw target marker '+' or 'X' at rt (pair 6, A_BOLD)
    │
    ▼
screen_present() → doupdate()  [ONE atomic diff write]
```

---

## Function Breakdown

### vec2_len(v) → float
Purpose: Euclidean magnitude of v.
Steps: `return sqrtf(v.x*v.x + v.y*v.y)`
Note: inlined; called hundreds of times per tick in the FABRIK loop.

### vec2_norm(v) → Vec2
Purpose: unit-length vector in the direction of v.
Steps:
1. `l = vec2_len(v)`
2. If `l < 1e-6`: return `(1, 0)` — degenerate guard (coincident joints)
3. Return `(v.x/l, v.y/l)`

### update_target(arm, dt)
Purpose: advance Lissajous clock, compute new target, push into trail.
Steps:
1. `scene_time += dt × speed_scale`
2. `target.x = root_px + lis_ax × cos(LIS_FX × scene_time)`
3. `target.y = root_py + lis_ay × sin(LIS_FY × scene_time + LIS_PHASE)`
4. `trail_head = (trail_head + 1) % TRAIL_SIZE`; write target there
5. `trail_count = min(trail_count + 1, TRAIL_SIZE)`
Critical ordering: must be called before `fabrik_solve()` so the solver uses the freshly computed target.

### fabrik_solve(arm)
Purpose: move arm joints to reach arm->target; set at_limit flag.
Steps:
1. `dist = sqrt((target.x−root.x)² + (target.y−root.y)²)`
2. If `dist > total_len`: stretch all links collinear; `at_limit=true`; return
3. `at_limit = false`
4. For `iter = 0..MAX_ITER-1`:
   a. FORWARD: `pos[N-1] = target`; for i=N-2..0: slide pos[i] away from pos[i+1] by link_len[i]
   b. BACKWARD: `pos[0] = root`; for i=0..N-2: slide pos[i+1] away from pos[i] by link_len[i]
   c. If `|pos[N-1] − target| < CONV_TOL`: break
Edge cases: division by near-zero flen guarded with `max(flen, 1e-6)`.

### draw_link_beads(window, a, b, pair, attr, cols, rows)
Purpose: stamp 'o' along segment a→b at DRAW_STEP_PX intervals.
Steps:
1. `dx = b.x−a.x`, `dy = b.y−a.y`, `len = |(dx,dy)|`; return if len < 0.1
2. `nsteps = ceil(len / DRAW_STEP_PX) + 1`
3. For t=0..nsteps: `u=t/nsteps`; compute `(cx,cy)` from interpolated pixel position
4. Skip if same cell as previous (dedup); skip if out of bounds (clip)
5. `mvwaddch(cy, cx, 'o')` with COLOR_PAIR(pair)|attr

### draw_reach_circle(window, root, radius, cols, rows)
Purpose: draw 48 dashed dots on the reach-limit circle.
Steps:
1. For k=0..47: `angle = k × 2π / 48`
2. `px = root.x + radius × cos(angle)`, `py = root.y + radius × sin(angle)`
3. Convert to cell; skip if out of bounds
4. `mvwaddch(cy, cx, '.')` with pair 7, A_DIM

### render_arm(arm, window, cols, rows, alpha)
Purpose: compose complete arm frame (read-only — const Arm*).
Steps:
1. Compute `rj[i] = lerp(prev_pos[i], pos[i], alpha)` for each joint
2. Compute `rt = lerp(prev_target, target, alpha)`
3. Draw faint trail dots (pair 6, A_DIM) from oldest to newest trail entry
4. If `at_limit`: draw_reach_circle at root with radius=total_len
5. PASS 1: draw_link_beads for each link using gradient pairs
6. PASS 2: draw joint node markers ('0', 'o', '.') overwriting bead fill
7. Draw '+' or 'X' at rt (pair 6, A_BOLD)

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
       scene_init(cols, rows)        ← reinit from scratch with new screen size
       reset frame_time, sim_accum
       need_resize = 0

  2. compute dt:
     now = clock_ns()
     dt  = now − frame_time
     frame_time = now
     cap dt at 100 ms                ← prevents spiral-of-death

  3. physics ticks (fixed-step accumulator):
     tick_ns = NS_PER_SEC / sim_fps
     dt_sec  = tick_ns / 1e9
     sim_accum += dt
     while sim_accum >= tick_ns:
       save prev_pos = pos; prev_target = target
       update_target(arm, dt_sec)    ← advance Lissajous, push trail
       fabrik_solve(arm)             ← move joints toward target
       sim_accum -= tick_ns

  4. compute alpha:
     alpha = sim_accum / tick_ns     [0.0 .. 1.0)

  5. update FPS counter:
     frame_count++
     fps_accum += dt
     if fps_accum >= 500ms:
       fps_display = frame_count / (fps_accum in seconds)
       reset frame_count, fps_accum

  6. frame cap:
     elapsed = clock_ns() − frame_time + dt
     sleep(NS_PER_SEC/60 − elapsed)

  7. draw + present:
     screen_draw(fps_display, alpha)   ← erase + render_arm + HUD into newscr
     screen_present()                  ← doupdate() → atomic write to terminal

  8. input:
     ch = getch()    ← non-blocking
     if ch != ERR: app_handle_key(ch)

cleanup:
  endwin()
```

---

## Interactions Between Modules

```
App
 ├── owns Screen (ncurses state, cols/rows)
 ├── owns Scene (Arm struct + paused flag)
 ├── reads sim_fps (adjustable by [ and ] keys)
 └── main loop drives everything

Screen
 ├── calls color_init() at startup
 ├── calls scene_draw(alpha) inside screen_draw()
 └── calls screen_present() after screen_draw()

Scene / Arm
 ├── update_target() called first each tick (sets target, pushes trail)
 ├── fabrik_solve() called second (reads target, modifies pos[])
 └── render_arm() called at draw time (reads pos[], prev_pos[], const)

§4 coords (px_to_cell_x/y)
 └── called ONLY inside draw functions — never in physics code

§2 clock (clock_ns, clock_sleep_ns)
 └── called only by main loop — physics and render never touch the clock

Signal handlers
 ├── write running = 0 (SIGINT/SIGTERM)
 └── write need_resize = 1 (SIGWINCH)
 └── both flags are volatile sig_atomic_t — safe from signal context
```

---

## Key Patterns to Internalize

**FABRIK is geometry, not algebra:**
No matrix inversion, no angles stored anywhere. The solver works purely on Vec2 positions, sliding each joint along the vector between it and its neighbor to restore the link length. The resulting angles emerge implicitly from where the joints end up.

**Two-pass bead renderer:**
Pass 1 fills every link with uniform 'o' beads at 5 px intervals. Pass 2 overwrites joint positions with size-coded markers ('0', 'o', '.'). Drawing two passes in the same frame costs nothing at terminal rendering speed and gives the chain an articulated, hierarchical look — thick at the root, delicate at the tip.

**Semantic color pairs vs theme pairs:**
Pairs 1–5 are theme-controlled (they change when you press 't'). Pairs 6 and 7 are semantically fixed — bright red means "target" and yellow means "reach boundary" regardless of theme. Theme cycling must not accidentally break visual meaning.

**prev_pos enables smooth rendering at any render rate:**
Every physics tick snapshots prev_pos before modifying pos. The renderer lerps: `rj = lerp(prev_pos, pos, alpha)`. Whether rendering at 60 fps with 60 Hz physics, or 144 fps with 60 Hz physics, joints slide smoothly between tick positions rather than jumping.

**Reachability check short-circuits FABRIK:**
Computing `|target − root| > total_len` is one distance calculation. When true, it avoids all FABRIK iterations (which would simply converge to the fully-extended configuration anyway) and directly sets the collinear stretch posture in O(N) — five assignments.
