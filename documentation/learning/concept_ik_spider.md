# Pass 1 — ik_spider: 6-Legged Spider with Procedural IK Leg Placement

## From the Source

**Algorithm 1 — Trail-buffer FK (body):** The head position history is stored tick-by-tick in a circular trail buffer (TRAIL_CAP=1024). Body joints are placed by measuring cumulative arc length backward along the trail and linearly interpolating between trail entries. Any path the head carves propagates naturally to the body — curves, turns, all handled automatically. This is the same approach as snake_forward_kinematics.c.

**Algorithm 2 — 2-Joint Analytical IK (legs):** Given hip H and foot target T, the knee position is solved in closed form using the law of cosines. No iteration required. `cos(angle_at_hip) = (dist² + UPPER² − LOWER²) / (2 × dist × UPPER)`. Left vs right legs choose opposite knee-bend directions so knees splay outward from the body centre.

**Algorithm 3 — Alternating Tripod Gait:** Legs are grouped into two tripods: A={0,2,4} (left front, left mid, left rear) and B={1,3,5} (right front, right mid, right rear). Only one tripod steps at a time. A leg triggers a step when its foot drifts more than STEP_TRIGGER_DIST from ideal position, or hip-to-foot distance exceeds MAX_STRETCH. Foot animation uses a cubic smoothstep ease-in/ease-out curve over STEP_DURATION=0.22 s.

**Data-structure:** Circular trail buffer `Vec2 trail[TRAIL_CAP]` for body FK. Per-leg: `foot_pos` (planted), `foot_old` (step-start), `step_target` (destination), `step_t` (0→1 progress), `stepping` flag. Two snapshot arrays `prev_body/prev_hip/prev_knee/prev_foot` enable sub-tick alpha lerp for smooth rendering. All positions in pixel space; cell conversion only at draw time.

## Core Idea

A 6-legged spider crawls autonomously across the terminal. The body uses trail-buffer FK to follow a sinusoidal curved path — the head position is pushed into a ring buffer each tick, and each body segment is placed at the appropriate arc-length distance behind the head. Each of the 6 legs independently uses 2-joint analytical IK (law of cosines) to reach a computed step target. Legs step in an alternating tripod gait that automatically adapts to the spider's speed.

The key difference from ik_arm_reach.c: IK here is analytical (one formula, exact, no iteration) because each leg has exactly 2 joints. For 2-joint chains, the law of cosines gives a closed-form solution with two cases (elbow-up or elbow-down) chosen by the left/right body side.

## The Mental Model

Imagine a real spider. Its body snakes forward along whatever curved path it's walking. The legs are attached to the body and must reach the ground. Each leg has exactly two segments (thigh and shin), so you can solve exactly where the knee must be given where the hip is and where the foot needs to land — that is the law of cosines triangle.

The gait logic is automatic and emergent: no pre-programmed timing tells legs when to step. Instead, each leg watches how far its planted foot has drifted from where it ideally wants to be. When the drift exceeds a threshold, the leg wants to step. The alternating tripod rule — "only step if your opposite tripod is planted" — enforces the insect's natural 3-point stability: while one tripod of 3 legs is in the air, the other 3 are firmly on the ground.

## Data Structures

### Spider struct (key fields)
```
trail[TRAIL_CAP]       — circular buffer of head positions (1024 Vec2)
trail_head             — write cursor (index of most recently pushed position)
trail_count            — valid entries; <= TRAIL_CAP
body_joint[N_BODY_SEGS+1] — 7 joint positions (0=head, 1..6=body segments)
prev_body[]            — snapshot at tick start for alpha lerp
heading                — current body heading in radians (forward direction)
wave_time              — accumulated time for sinusoidal turn rate
move_speed             — head translation speed in pixels/s

hip[N_LEGS]            — world-space hip positions (attached to body)
knee[N_LEGS]           — knee positions from IK solve
foot_pos[N_LEGS]       — planted foot positions (IK targets)
foot_old[N_LEGS]       — foot position at step start (lerp source)
step_target[N_LEGS]    — destination foot position for current step
stepping[N_LEGS]       — true if leg is currently in swing phase
step_t[N_LEGS]         — step animation progress [0, 1]

prev_hip/knee/foot[]   — snapshots for alpha lerp
hip_dist               — hip lateral offset from body centerline (pixels)
paused
theme_idx
```

### Per-leg state machine fields
```
foot_pos  — the current planted foot position (IK target)
foot_old  — the foot position when this step started
step_target — where the foot is heading (ideal_foot from compute_ideal_foot)
step_t    — progress [0, 1]; advances by dt / STEP_DURATION each tick
stepping  — false = planted, true = swinging toward step_target
```

### Color pair layout (spider)
```
pairs 1–3  — body gradient (tail=1 → head=3)
pair 4     — upper leg segment
pair 5     — lower leg segment
pair 6     — planted foot marker ('*', bold)
pair 7     — stepping foot marker ('o', dim)
pair 8     — HUD status bar
```
Only pair 3 (main spider colour), pair 6 (planted foot), and pair 7 (stepping foot) are actively used in the renderer.

### LEG_ANGLE and HIP_BODY_T arrays
```
LEG_ANGLE[6] = {0.55, -0.55, 1.57, -1.57, 2.60, -2.60}  radians
  — angle added to body_forward for ideal step direction per leg
  — legs 0/1 = front pair (slight forward angle)
  — legs 2/3 = mid pair (straight sideways, π/2)
  — legs 4/5 = rear pair (backward-angled)

HIP_BODY_T[6] = {0.15, 0.15, 0.50, 0.50, 0.85, 0.85}
  — parametric position along body centerline (0=head, 1=tail)
  — front pair attaches 15% back from head
  — mid pair at 50%
  — rear pair at 85% (near the tail)
```

## The Main Loop

Each iteration of the main loop:

1. **Check for resize.** SIGWINCH → reinitialise scene with new terminal dimensions.

2. **Measure dt.** CLOCK_MONOTONIC; cap at 100 ms.

3. **Physics accumulator.** `sim_accum += dt`. While `sim_accum >= tick_ns`:
   - Snapshot `prev_body`, `prev_hip`, `prev_knee`, `prev_foot`.
   - If paused: return early (lerp freezes cleanly).
   - `move_body()`: advance `wave_time`, integrate sinusoidal turn rate into `heading`, translate body_joint[0] along heading, toroidal wrap at screen edges, push to trail.
   - `compute_body_joints()`: place body_joint[1..N_BODY_SEGS] via `trail_sample()` at arc-length offsets.
   - `compute_hips()`: attach each hip to its body point; offset laterally by `hip_dist` perpendicular to local body forward.
   - `update_steps()`: check step triggers; start or advance step animations; recompute IK for all legs.
   - `sim_accum -= tick_ns`.

4. **Compute alpha.** `alpha = sim_accum / tick_ns`.

5. **FPS counter.** Every 500 ms update display.

6. **Frame cap sleep.** Before render; 60 fps ceiling.

7. **Draw and present.** `render_spider()` draws at alpha-interpolated positions: legs (direction chars), body (bead chain), head (directional arrow). HUD overlay. `doupdate()`.

8. **Handle input.** Speed adjust, theme cycle, sim Hz, quit.

## Non-Obvious Decisions

**Why analytical IK instead of FABRIK for the spider legs?**
Each leg has exactly 2 joints (hip, knee, foot). For 2-joint chains, the law of cosines gives a closed-form exact solution with no iteration needed. FABRIK would converge to the same answer in 1–2 iterations but with more code. The analytical solution is always exact, always terminates in constant time, and is simpler to understand geometrically. The tradeoff is that the analytical approach hardcodes 2 joints per leg — FABRIK would trivially extend to 3 or more joints.

**Why does the distance get clamped before applying acos?**
`cos_hip = (dist² + UPPER² − LOWER²) / (2 × dist × UPPER)`. If the foot target is too close (dist < |UPPER−LOWER|) or too far (dist > UPPER+LOWER), this formula produces a value outside [−1, 1], causing `acosf()` to return NaN. The clamp `dist = clampf(dist, |UPPER−LOWER|+1, UPPER+LOWER−1)` keeps the IK in its valid geometric range. The foot is then snapped to the nearest reachable point on the IK range.

**Why left legs add `angle_at_hip` and right legs subtract it?**
The knee bends outward from the body centre. For a left leg, "outward" is the positive-normal (counter-clockwise) direction from the body centreline. Adding `angle_at_hip` to the base direction rotates the knee CCW (to the left). For right legs, subtracting rotates CW (to the right). This guarantees knees always splay laterally rather than folding inward.

**Why trail_sample() walks the trail rather than using a formula?**
The body follows an arbitrary curved path — there is no parametric formula that gives "position 100 pixels behind the head along the exact path taken." The trail buffer records every pixel position the head visited. `trail_sample(dist)` walks backward through the trail, accumulating segment lengths, until the cumulative distance equals `dist`, then linearly interpolates between the two bounding entries. This correctly places each body joint on the actual curve the head traced, not on a straight line or approximation.

**Why smoothstep for foot animation instead of linear lerp?**
`smoothstep(t) = t² × (3 − 2t)`. This cubic curve has zero first derivative at t=0 and t=1, meaning the foot starts and ends its step at zero velocity — no abrupt snap. With linear lerp, the foot would instantly be moving at full speed at the start of a step (velocity discontinuity), which looks mechanical and wrong. Smoothstep gives the natural deceleration of a real leg landing.

**Why is the screen-wrap snap needed?**
When the spider crosses a screen edge (toroidal wrap), the head teleports from one side of the screen to the other. The body joints and hips follow. But planted feet were at their old positions on the other side. After the wrap, `hip-to-foot` distance can exceed `UPPER+LOWER` — far beyond IK reach. The IK solver clamps this, producing a stretched unnatural leg. The snap detects this condition (`snap_stretch > UPPER+LOWER-2`) and immediately teleports the foot to the ideal position without animation.

**Why pre-fill the trail at scene_init?**
Without pre-filling, the trail starts empty. `trail_sample()` returns the head's starting position for any requested distance — all body joints pile up at the head position. On the first tick, every body joint would overlap at one cell. Pre-filling with straight-backward positions gives the spider a realistic initial pose with properly spread-out body joints from the first frame.

## State Machines

### App-level state
```
        ┌───────────────────────────────────────────────────┐
        │                  RUNNING                          │
        │                                                   │
        │  move_body → trail_push → compute_body_joints     │
        │  compute_hips → update_steps → solve_ik (×6)     │
        │  render_spider (alpha-lerp)                       │
        │                                                   │
        │  SIGWINCH ──────────────→ need_resize             │
        │       need_resize ──────→ scene_init              │
        │  SIGINT/SIGTERM ────────→ running = 0             │
        └───────────────────────────────────────────────────┘
                          │
               'q'/ESC/signal
                          v
                        EXIT
```

### Per-leg state machine
```
        ┌──────────────────────────────────────────────┐
        │                  PLANTED                     │
        │  foot_pos is fixed                           │
        │  IK reaches foot_pos each tick               │
        │                                              │
        │  drift > STEP_TRIGGER_DIST                   │
        │  OR stretch > MAX_STRETCH                    │
        │  AND opposite tripod is planted              │
        └──────────────────────────────────────────────┘
                          │
                    start_step()
                    foot_old = foot_pos
                    step_target = ideal_foot
                    step_t = 0
                          │
                          ▼
        ┌──────────────────────────────────────────────┐
        │               STEPPING                       │
        │  step_t advances by dt/STEP_DURATION         │
        │  foot_pos = lerp(foot_old, step_target,      │
        │                  smoothstep(step_t))         │
        │  IK tracks moving foot_pos                   │
        └──────────────────────────────────────────────┘
                          │
                  step_t >= 1.0
                          │
                          ▼
                  foot_pos = step_target
                  stepping = false
                  → PLANTED
```

### Tripod alternation
```
At any moment exactly one tripod may be stepping.

TRIPOD_A = legs {0, 2, 4}  (left front, left mid, left rear)
TRIPOD_B = legs {1, 3, 5}  (right front, right mid, right rear)

Rule: a leg in tripod X may start a step only if no leg in tripod Y
      is currently stepping.

Result: while A steps → B is planted (3-point ground contact)
        while B steps → A is planted (3-point ground contact)
        → statically stable at all times
```

### Input actions
| Key | Effect |
|-----|--------|
| q / ESC | exit |
| space | toggle paused |
| ↑ | increase move_speed (up to BODY_SPEED_MAX=200 px/s) |
| ↓ | decrease move_speed (down to BODY_SPEED_MIN=10 px/s) |
| t | cycle colour theme forward |
| ] | increase sim_fps by SIM_FPS_STEP |
| [ | decrease sim_fps by SIM_FPS_STEP |

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|----------|---------|-------------------|
| N_LEGS | 6 | 3 per side. Change to 4 (8 legs) for a spider, 2 for a biped — requires adjusting tripod groups. |
| UPPER_LEN / LOWER_LEN | 50 / 44 px | Leg segment lengths. UPPER > LOWER = knee above halfway. Equal = symmetric bend. If LOWER > UPPER+EPSILON the IK clamp range changes. |
| STEP_TRIGGER_DIST | 25 px | Foot drift before step triggers. Too small: legs step constantly, chaotic. Too large: legs stretch unnaturally before stepping. |
| MAX_STRETCH | 60 px | Absolute hip-to-foot limit. Should be < UPPER+LOWER (94 px). If larger, no snap needed. |
| STEP_DURATION | 0.22 s | One step takes this long. Shorter: faster, more mechanical. Longer: more graceful but lags behind body at high speed. |
| STEP_REACH_FACTOR | 0.68 | Ideal foot reach = (UPPER+LOWER) × factor. 0.68 × 94 ≈ 64 px. Larger: feet plant further away, wider stance. |
| BODY_SPEED | 45 px/s | Head translation speed. Increase: feet step more often, gait adapts. Beyond ~100 px/s step animations can't keep up. |
| TURN_AMP / TURN_FREQ | 0.40 / 0.50 rad | Sinusoidal turning: heading += TURN_AMP × sin(TURN_FREQ × wave_time) × dt. Higher amp: tighter curves. Lower freq: longer, lazier turns. |
| N_BODY_SEGS | 6 | Body length = N_BODY_SEGS × BODY_SEG_LEN (6×20=120 px). More segs: longer body, more trail history needed. |
| HIP_DIST_FACTOR | 0.06 | Hip offset = rows×CELL_H × factor. Increase: legs emanate further from body centre, wider spider. |
| DRAW_STEP_PX | 5 px | Body bead fill density. DRAW_LEG_STEP_PX=8 px (=CELL_W) for legs — one char per cell column. |

## Open Questions for Pass 3
- What happens to the gait if STEP_DURATION is longer than the time it takes the spider to walk one STEP_TRIGGER_DIST? Do two legs from the same tripod try to step simultaneously?
- The tripod check uses local variables `tripod_a` and `tripod_b` computed at the start of `update_steps()`. If multiple legs in the same tripod all have drift > STEP_TRIGGER_DIST simultaneously, can they all start stepping in the same tick? Trace through the code.
- After a toroidal wrap, the snap logic teleports the foot to ideal position. Does the opposite tripod still hold? Could a wrap cause both tripods to be stepping simultaneously for one tick?
- `trail_sample()` linearly interpolates between trail entries. The body moves at 45 px/s ≈ 0.75 px/tick at 60 Hz — the trail entries are very close together. Is linear interpolation sufficient, or could you see a staircase effect in the body curve?
- `rotate2d()` is used to compute ideal foot direction from body heading. What does the ideal foot direction look like for the mid-legs (LEG_ANGLE=π/2)? Draw it out: body heading east (0 rad), front leg angle 0.55 rad, mid-leg angle π/2 rad.
- What if `move_speed` is set to 0 (spider stands still)? The body_joint[0] never moves, trail entries are all identical, and `trail_sample()` would always return the head position. All body joints would pile up at the head position. Does the gait still work?

---

# Structure

| Symbol | Type | Size (approx) | Role |
|--------|------|---------------|------|
| `Spider.trail[1024]` | `Vec2[1024]` | 8 KB | circular buffer of head positions for body FK |
| `Spider.body_joint[7]` | `Vec2[7]` | 56 B | current body joint positions |
| `Spider.prev_body[7]` | `Vec2[7]` | 56 B | tick-start snapshot for alpha lerp |
| `Spider.hip/knee/foot[6]` | `Vec2[6]` × 3 | 144 B | current leg joint positions |
| `Spider.prev_hip/knee/foot[6]` | `Vec2[6]` × 3 | 144 B | alpha-lerp snapshots |
| `LEG_ANGLE[6]` | `float[6]` | 24 B | static; ideal leg direction offsets |
| `HIP_BODY_T[6]` | `float[6]` | 24 B | static; body attachment points |
| `THEMES[10]` | `Theme[10]` | ~400 B | selectable colour palettes (static) |
| `g_app` | `App` | ~10 KB | top-level container |

---

# Pass 2 — ik_spider: Pseudocode

## Module Map

| Section | Purpose |
|---------|---------|
| §1 config | N_LEGS, N_BODY_SEGS, TRAIL_CAP, leg geometry, gait params, CELL_W/H |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 10-theme arachnid palette; `theme_apply()` |
| §4 coords | `px_to_cell_x/y` — single aspect-ratio conversion |
| §5a vec2 | add, sub, scale, lerp, len, dist, norm; `smoothstep`, `rotate2d` |
| §5b trail | `trail_push`, `trail_at`, `trail_sample` — arc-length parameterised trail lookup |
| §5c body motion | `move_body()` — heading integration, translation, toroidal wrap, trail push |
| §5d body joints | `compute_body_joints()` — trail_sample for each body segment |
| §5e hip placement | `compute_hips()` — body-local lateral offset per leg |
| §5f 2-joint IK | `solve_ik()` — law of cosines, knee placement, left/right bend choice |
| §5g step logic | `compute_ideal_foot()`, `update_steps()` — tripod gait, smoothstep foot animation |
| §5h rendering | `head_glyph()`, `seg_glyph()`, `draw_leg_line()`, `draw_line_beads()`, `render_spider()` |
| §6 scene | `scene_init/tick/draw` — thin wrappers over Spider |
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
    ├── snapshot prev_body, prev_hip, prev_knee, prev_foot
    │
    ├── move_body(dt):
    │     wave_time += dt
    │     heading  += TURN_AMP × sin(TURN_FREQ × wave_time) × dt
    │     body_joint[0] += move_speed × (cos heading, sin heading) × dt
    │     toroidal wrap (body_joint[0] stays in [0, W) × [0, H))
    │     trail_push(body_joint[0])
    │
    ├── compute_body_joints():
    │     body_joint[i] = trail_sample(i × BODY_SEG_LEN)   for i=1..6
    │
    ├── compute_hips():
    │     for each leg i:
    │       attach = lerp(body_joint[seg], body_joint[seg+1], frac)
    │       fwd    = norm(body_joint[seg] − body_joint[seg+1])
    │       left_norm = (−fwd.y, fwd.x)
    │       side = +1 if even (left), −1 if odd (right)
    │       hip[i] = attach + left_norm × side × hip_dist
    │
    ├── update_steps(dt):
    │     for each leg i:
    │       if stretch > UPPER+LOWER−2: snap foot to ideal (screen-wrap guard)
    │       if NOT stepping:
    │         ideal = compute_ideal_foot(i)
    │         if drift > STEP_TRIGGER_DIST OR stretch > MAX_STRETCH:
    │           if opposite tripod NOT stepping:
    │             start step: foot_old=foot_pos, step_target=ideal, step_t=0
    │       else:
    │         step_t += dt / STEP_DURATION
    │         foot_pos = lerp(foot_old, step_target, smoothstep(step_t))
    │         if step_t >= 1.0: finalize (stepping=false)
    │
    └── solve_ik(hip[i], foot_pos[i], is_left, &knee[i])   for all 6 legs
          law of cosines → knee position

sim_accum -= tick_ns

alpha = sim_accum / tick_ns  [0.0..1.0)

render_spider(alpha):
    interpolate r_body, r_hip, r_knee, r_foot using alpha
    1. draw legs (\ | / - direction chars) for hip→knee and knee→foot
    2. draw knee 'o' and foot '*'(planted) or 'o'(stepping) markers
    3. draw body bead fill (tail→head, pair 3, A_BOLD)
    4. draw body '0' node markers
    5. draw head glyph (directional arrow)

screen_present() → doupdate()
```

---

## Function Breakdown

### trail_push(spider, pos)
Purpose: add head position to circular trail buffer.
Steps:
1. `trail_head = (trail_head + 1) % TRAIL_CAP`
2. `trail[trail_head] = pos`
3. `trail_count = min(trail_count+1, TRAIL_CAP)`

### trail_sample(spider, dist) → Vec2
Purpose: position at arc-length dist behind the most recent trail entry.
Steps:
1. `a = trail_at(sp, 0)` (most recent), `accum = 0`
2. For k=1..trail_count-1: `b = trail_at(sp, k)`; compute segment length
3. If `accum + seg >= dist`: linearly interpolate a→b at fraction `(dist−accum)/seg`; return
4. `accum += seg`; `a = b`; continue
5. If dist > total trail length: return oldest entry (tail end of trail)

### move_body(spider, dt, cols, rows)
Purpose: advance sinusoidal heading, translate body_joint[0], wrap, push trail.
Steps:
1. `wave_time += dt`
2. `heading += TURN_AMP × sin(TURN_FREQ × wave_time) × dt`
3. `body_joint[0] += move_speed × (cos(heading), sin(heading)) × dt`
4. Wrap x: if x < 0: x += W; if x >= W: x -= W (toroidal, not bounce)
5. `trail_push(body_joint[0])`

### compute_hips(spider)
Purpose: place each hip in world space, offset laterally from body centreline.
Steps: for each leg i:
1. Compute `t_body = HIP_BODY_T[i] × N_BODY_SEGS`; `seg_idx = floor(t_body)`, `frac = fractional part`
2. `attach = lerp(body_joint[seg_idx], body_joint[seg_idx+1], frac)`
3. `fwd = norm(body_joint[seg_idx] − body_joint[seg_idx+1])`
4. `left_norm = (−fwd.y, fwd.x)` (rotate forward 90° CCW)
5. `side = +1.0 if i even else −1.0`; `hip[i] = attach + left_norm × side × hip_dist`

### solve_ik(hip, target, is_left, knee_out)
Purpose: 2-joint analytical IK via law of cosines.
Steps:
1. `dx = target.x−hip.x`, `dy = target.y−hip.y`, `dist = sqrt(dx²+dy²)`
2. `dist = clamp(dist, |UPPER−LOWER|+1, UPPER+LOWER−1)`
3. `base_angle = atan2(dy, dx)`
4. `cos_hip = (dist² + UPPER² − LOWER²) / (2 × dist × UPPER)`, clamped to [−1,1]
5. `angle_hip = acos(cos_hip)`
6. `knee_angle = base_angle + angle_hip` (left leg) or `base_angle − angle_hip` (right leg)
7. `knee_out = hip + UPPER × (cos(knee_angle), sin(knee_angle))`
Note: foot always reaches target exactly — no residual error, no iteration.

### compute_ideal_foot(spider, i) → Vec2
Purpose: where leg i ideally wants its foot when not stepping.
Steps:
1. `fwd = (cos(heading), sin(heading))` (body forward direction)
2. `dir = rotate2d(fwd, LEG_ANGLE[i])` (rotate by per-leg angle offset)
3. `reach = (UPPER+LOWER) × STEP_REACH_FACTOR`
4. Return `hip[i] + dir × reach`

### update_steps(spider, dt)
Purpose: check step triggers, advance step animations, recompute IK.
Steps:
1. Compute `tripod_a` = any of {0,2,4} stepping; `tripod_b` = any of {1,3,5} stepping
2. For each leg i:
   a. Screen-wrap snap: if `|foot_pos−hip| > UPPER+LOWER−2`: snap foot to ideal, solve IK, continue
   b. If not stepping: compute drift and stretch; if should_step AND opposite tripod not stepping: start step
   c. If stepping: advance step_t; update foot_pos = lerp(foot_old, step_target, smoothstep(step_t)); finalise if step_t >= 1
3. After all legs: recompute IK for all 6 legs

---

## Pseudocode — Core Loop

```
setup:
  register atexit(cleanup)
  register signal(SIGINT/SIGTERM) → running = 0
  register signal(SIGWINCH)       → need_resize = 1
  screen_init()
  scene_init(cols, rows)
  frame_time = clock_ns()
  sim_accum = 0

main loop (while running):

  1. handle resize:
     if need_resize:
       endwin() + refresh()
       getmaxyx() → new cols, rows
       scene_init(cols, rows)
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
       scene_tick(dt_sec, cols, rows):
         snapshot prev_body, prev_hip, prev_knee, prev_foot
         if paused: return
         move_body(dt_sec, cols, rows)
         compute_body_joints()
         compute_hips()
         update_steps(dt_sec)         ← gait + IK solve
       sim_accum -= tick_ns

  4. alpha = sim_accum / tick_ns

  5. FPS counter update (every 500 ms)

  6. frame cap: sleep(NS_PER_SEC/60 − elapsed)

  7. draw + present:
     screen_draw(fps, alpha)           ← erase + render_spider + HUD
     screen_present()                  ← doupdate()

  8. input:
     ch = getch()
     dispatch: q=quit, space=pause, up/down=speed, t=theme, [/]=sim_fps

cleanup:
  endwin()
```

---

## Interactions Between Modules

```
App
 ├── owns Screen (ncurses state, cols/rows)
 ├── owns Scene → Spider
 ├── reads sim_fps (adjustable by keys)
 └── main loop drives everything

Spider (per tick):
  move_body()            — updates body_joint[0] + trail
  compute_body_joints()  — reads trail; writes body_joint[1..6]
  compute_hips()         — reads body_joint[]; writes hip[]
  update_steps()         — reads hip[]; reads/writes foot_pos[], stepping[], step_t[]
                         — calls solve_ik() for all 6 legs
  scene_draw(alpha)      — reads prev/current body,hip,knee,foot; renders

§5b trail helpers
 └── called ONLY by move_body() (push) and compute_body_joints() (sample)

§5f solve_ik()
 └── called ONLY by update_steps() (once per leg per tick) and scene_init()

§4 coords (px_to_cell_x/y)
 └── called ONLY inside rendering functions

Signal handlers (volatile sig_atomic_t flags — async-signal-safe)
```

---

## Key Patterns to Internalize

**Analytical IK is exact and free when the chain is exactly 2 joints:**
The law of cosines gives a closed-form formula. No iteration, no convergence criterion, no tolerance. The foot lands exactly on the target every tick. The cost is flexibility: this only works for exactly 2 joints; adding a 3rd requires switching to an iterative method like FABRIK.

**Trail-buffer FK decouples body shape from heading math:**
The body does not store joint angles or a kinematic chain. It stores the history of where the head has been. Any winding path (sharp turns, gentle curves, straight lines) propagates naturally to the body segments without changing the FK code. The trail is the state machine for body shape.

**Gait emergence from local rules:**
No global "gait clock" tells legs when to step. Each leg independently monitors its own foot drift. The single constraint — "don't step if the opposite tripod is also stepping" — produces the correct alternating tripod gait automatically, at any speed, without pre-programmed timing. This is an example of emergent behavior from simple local rules.

**Smoothstep creates biological motion from linear time:**
`step_t` advances linearly from 0 to 1. `smoothstep(step_t)` maps it to a cubic S-curve with zero velocity at both endpoints. The foot acceleration and deceleration are built into the interpolation function — no force model required.

**Screen wrap requires an explicit "snap" to avoid IK overstretch:**
Toroidal wrap is aesthetically clean (spider never hits a wall), but it creates a discontinuity: the hip teleports while the foot stays behind. Without the snap detection, the IK solver clamps the overextended leg in a visually wrong configuration. The snap treats the wrap as an instant "reset" for the affected leg, teleporting the foot to ideal position without animation.
