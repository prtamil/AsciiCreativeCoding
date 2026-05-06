# Pass 1 — fk_helloworld: 2-link Forward Kinematics (angles → positions)

## From the Source

**Algorithm:** Forward kinematics for a planar 2-link arm. Given a fixed shoulder S and two joint angles θ1 (shoulder) and θ2 (elbow, relative to upper arm), compute joint positions by composing two rotations: `E = S + L1·(cos θ1, sin θ1)`; `H = E + L2·(cos(θ1+θ2), sin(θ1+θ2))`. No equations to solve, no triangle to construct, no ambiguity. The hand position is a deterministic function of the joint angles. This is the FORWARD direction of the kinematic chain — the easy direction; IK is its non-trivial inverse.

**Math:** Each link is a polar offset: a length L at an absolute angle θ produces a tip at base + L·(cos θ, sin θ). The forearm's absolute angle is the cumulative sum θ1 + θ2 because θ2 is measured RELATIVE to the upper-arm direction. Stacking these is matrix-free 2-D forward kinematics: rotation in 2-D collapses to a single trig pair, and position is the running sum. `wrap_pi(a)` folds an angle into (−π, π] purely cosmetically — the trig calls work fine on raw values, but a stable HUD readout matters during sustained key holds.

**Performance:** O(1) per frame. Two `cosf` and two `sinf`, plus a handful of additions. Variable timestep at render rate; no fixed-step accumulator, no alpha lerp scaffolding, no `prev_pos` snapshot. The simulation is non-stiff — there are no springs or oscillators driving the joints, only direct user input — so a single dt per frame is unconditionally stable. ncurses redraw dominates the frame budget.

**Data-structure:** One `Vec2` for the fixed shoulder, two scalar angles (`theta1`, `theta2`) for user input, and two cached `Vec2` outputs (`elbow`, `hand`) in the scene. `compute_fk(S, θ1, θ2) → ArmPose` is a pure function returning `{Vec2 elbow, Vec2 hand}`; the main loop assigns its result into `scene.{elbow, hand}` so renderer and HUD read the positions without recomputing.

## Core Idea

Forward kinematics is the EASY half of robotics. You set the joint angles; the math tells you where every link ends up. There is no solver, no inverse, no ambiguity. Studying FK in isolation makes IK intelligible: every IK solver is essentially asking "what FK input produced this output?" — so understanding the forward direction is the natural first step.

The visual is identical to `ik_helloworld.c` — three points, two lines, an `'@'` shoulder, an `'O'` elbow, a `'*'` hand. What differs is the data flow direction. In IK, the user controls the hand and the elbow falls out of the math. In FK, the user controls the angles and BOTH the elbow and the hand fall out. The same `'*'` glyph means INPUT in IK and OUTPUT in FK — that single role-flip is the whole pedagogical point of having both files.

## The Mental Model

Sit at a desk and lay your right arm flat. Slowly rotate your right SHOULDER — the upper arm swings, and your elbow draws an arc of radius L1 around the shoulder. Now hold the shoulder fixed and bend your ELBOW — the forearm pivots, and your hand draws an arc of radius L2 around the elbow. Combine the two: any reachable hand position is some θ1 (shoulder) plus some θ2 (elbow). That is all FK is.

```
shoulder    @          ← S, fixed, never moves
             ╲ θ1
              ╲   L1   ← upper arm (constant length)
               ╲
        elbow   O      ← E = S + L1·(cos θ1, sin θ1)
                ╲ θ2   (relative to the upper arm direction)
                 ╲ L2
                  ╲
             hand  *   ← H = E + L2·(cos(θ1+θ2), sin(θ1+θ2))
```

The cumulative angle pattern is the key generalization: link N's absolute angle is the sum of all upstream joint angles. For a 2-link arm that is θ1 and θ1+θ2; for a 3-link arm it is θ1, θ1+θ2, θ1+θ2+θ3 (see `fk_ik_helloworld.c`). The chain in CHAIN comes from this accumulation.

## Data Structures

### Scene struct
```
shoulder    Vec2  — pinned anchor S
theta1      float — shoulder angle, radians (absolute, from +X axis)
theta2      float — elbow angle, radians (relative to upper arm direction)
elbow       Vec2  — cached FK output E, refreshed each frame
hand        Vec2  — cached FK output H, refreshed each frame
paused      bool  — when true, arrow keys ignored
rows, cols  int   — current terminal cell extents
```

### ArmPose
```
elbow, hand    Vec2 pair returned by compute_fk()
```
A small return-struct for the pure FK function. Keeps `compute_fk` free of pointer-out parameters, makes the call site read `pose = compute_fk(...)`.

### Vec2
```
x, y   float — pixel-space position; isotropic
```

### Color pair layout
```
PAIR_HUD          bright yellow (226) — status row
PAIR_HINT         bright cyan   ( 51) — hint row
PAIR_LINK_UPPER   sky cyan      ( 45) — upper arm S→E
PAIR_LINK_FORE    orange        (208) — forearm  E→H
PAIR_JOINT        white         (255) — elbow marker 'O'
PAIR_SHOULDER     lime          (118) — anchor marker '@'
PAIR_HAND         red           (196) — computed hand marker '*'
```

## The Main Loop

The loop mirrors the canonical FK frame skeleton:

1. **`(θ1, θ2) = read_input_angles()`.** Drain `getch`. ←/→ nudge θ1 by ±ANGLE_STEP_RAD; ↑/↓ nudge θ2. After the nudge, `wrap_pi` folds both angles into (−π, π].

2. **`(E, H) = compute_fk(S, θ1, θ2)`.** Pure function returning the cached pose. The call site is `ArmPose pose = compute_fk(...); scene.elbow = pose.elbow; scene.hand = pose.hand`.

3. **`erase()`.**

4. **`scene_draw`.** Two lines (S→E upper arm, E→H forearm) plus three markers (`*` hand bottom, `O` elbow, `@` shoulder top so it wins the cell contest).

5. **`screen_present()`.** `wnoutrefresh` + `doupdate`.

6. **Frame cap.** Sleep until 1/60 s has elapsed.

## Non-Obvious Decisions

**Why FK before IK?**
FK is the easy direction. Given angles, finding positions is one matrix multiplication (or, in 2-D, two trig calls). IK is the inverse problem: given a desired hand position, what angles produce it? That inverse is non-trivial — multiple solutions exist, the problem may be unsolvable, and the math branches into case analysis. Studying FK first makes IK intelligible: every IK solver is essentially asking "what FK input produced this output?"

**Why is θ2 RELATIVE to the upper arm and not absolute?**
Joint angles in robotics are conventionally relative to the parent link. Conceptually θ2 is "the elbow bend" — how far the forearm deviates from straight extension of the upper arm. With θ2 = 0 the arm is fully extended; with θ2 = π the forearm folds back along the upper arm. Making θ2 absolute would mean "the angle of the forearm in world space", which couples it to θ1 in a way that is mathematically equivalent (`abs_θ_fore = θ1 + θ2`) but pedagogically backwards.

**Why two arrow pairs (←→ for θ1, ↑↓ for θ2)?**
Each angle gets one axis of arrow keys. Symmetric, easy to learn, immediately maps to "sweeping the upper arm" vs "sweeping the forearm". Splitting one keymap across two semantic actions is what teaches the user that the two angles are independent inputs to one chain.

**Why the `wrap_pi` cosmetic normalisation?**
With key repeat at ~30 Hz and ANGLE_STEP_RAD = 2°, holding an arrow key for 30 seconds drifts θ to ±1800°. The trig calls handle that fine, but the HUD readout `theta1: +1800.0°` becomes meaningless. `wrap_pi` folds the value into (−π, π] every frame, costing a `while`-loop iteration or two per frame at most.

**Why store `scene.elbow` and `scene.hand` instead of recomputing in scene_draw?**
Same reason as in `ik_helloworld.c` — caching the solver output in the scene lets every consumer (renderer, HUD, future debug overlays) read the positions without re-running the solver and without taking them as parameters. The solver remains a pure function with no scene reference.

**Why no two-solution ambiguity here?**
FK is deterministic: each (θ1, θ2) pair produces exactly ONE (E, H) configuration. There is no `'f'` flip key because there is nothing to flip — the angles uniquely determine the positions. Compare with IK where any reachable target has TWO valid configurations (elbow up / elbow down). This contrast is the core teaching point of having both files.

**Why no reachability check?**
The hand can land anywhere within the reach disc; there is no "out of reach" because the user is not TARGETING anything — they are rotating joints. The arm just goes wherever the angles put it. That asymmetry — no reachability concept in FK, mandatory clamp_to_reach in IK — is one of the most striking ways FK is simpler than IK.

## State Machines

### Input actions
| Key | Effect (when not paused) | Effect when paused |
|-----|--------------------------|--------------------|
| ←   | θ1 -= ANGLE_STEP_RAD; wrap_pi | Ignored |
| →   | θ1 += ANGLE_STEP_RAD; wrap_pi | Ignored |
| ↑   | θ2 -= ANGLE_STEP_RAD; wrap_pi | Ignored |
| ↓   | θ2 += ANGLE_STEP_RAD; wrap_pi | Ignored |
| space | Toggle paused | Toggle paused |
| r   | Reset angles to (0, 0) | Reset angles to (0, 0) |
| q / ESC | Exit | Exit |

### App-level state
```
        ┌──────────────────────────────────────────┐
        │               RUNNING                    │
        │                                          │
        │  [scene_input]   → update angles         │
        │  [compute_fk]    → set scene.{elbow,hand}│
        │  [scene_draw]    → 5 draw calls          │
        │                                          │
        │  SIGWINCH ────────→ scene_resize          │
        │  SIGINT/SIGTERM ──→ running = 0           │
        └──────────────────────────────────────────┘
                          │
                  'q' / ESC / signal
                          │
                          ▼
                        EXIT
                      (endwin)
```

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|----------|---------|-------------------|
| TARGET_FPS | 60 | Render cap. FK runs in microseconds; this is purely for I/O smoothness. |
| L1_PX | 80 | Upper arm length, pixels. Increase for longer arm; the elbow's reach circle grows. |
| L2_PX | 80 | Forearm length, pixels. Same. With L1=L2 the hand can return to the shoulder when θ2 = π. |
| ANGLE_STEP_RAD | 2° (≈0.0349 rad) | Per-keystroke angle change. Smaller: finer control, slower sweep. Larger: faster sweep, less precision. |

## Open Questions for Pass 3
- The cumulative angle pattern (`a2 = θ1 + θ2`, `a3 = θ1 + θ2 + θ3` in the 3-link version) is the heart of FK. Is there a numerically smarter way to maintain absolute joint angles directly so each link updates in O(1) without summing the whole chain?
- The HUD's theta readout uses `wrap_pi` to fold into (−π, π]. Some users prefer (0, 2π] or (−180°, 180°]. Is the choice purely aesthetic or does it matter for any computation?
- With ANGLE_STEP_RAD = 2° and key repeat ~30 Hz, holding → for one second sweeps θ1 by 60°. Is this "right" for a hello-world (visible motion, easy targeting), or would smaller steps + faster repeat feel better?
- The arm has no joint limits: θ2 can wind around indefinitely, twisting the forearm into impossible-for-a-human poses. Adding `[θ2_min, θ2_max]` clamps would model real anatomy. Is that worth doing for the hello-world or does it muddy the pure-FK lesson?

---

# Structure

| Symbol | Type | Size (approx) | Role |
|--------|------|---------------|------|
| `Scene.shoulder` | `Vec2` | 8 B | pinned anchor, set once |
| `Scene.theta1`, `Scene.theta2` | `float` × 2 | 8 B | user-controlled joint angles, radians |
| `Scene.elbow`, `Scene.hand` | `Vec2` × 2 | 16 B | cached FK outputs, refreshed each frame |
| `Scene.paused` | `bool` | 1 B | input freeze |
| `ArmPose` | struct | 16 B | return value of compute_fk |
| `Scene` total | struct | ~48 B | full per-frame state of the demo |

---

# Pass 2 — fk_helloworld: Pseudocode

## Module Map

| Section | Purpose |
|---------|---------|
| §1 config | TARGET_FPS, L1/L2, ANGLE_STEP_RAD, DEG_TO_RAD/RAD_TO_DEG, color pair IDs, CELL_W/H |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` — monotonic timing |
| §3 color | `color_init()` — bind 7 semantic pairs |
| §4 coords | `px_to_cell_x/y` — pixel↔cell aspect bridge (one-way at draw time) |
| §5 fk | `Vec2` + helpers; `ArmPose`; `compute_fk(S, θ1, θ2) → ArmPose` |
| §6 scene | `Scene` state, `wrap_pi`, `scene_init/resize/input/draw` |
| §7 screen | `screen_init/cleanup/present/hud` — ncurses double-buffer |
| §8 app | Signal handlers, `main()` — three-step loop kernel |

## Data Flow Diagram

```
  arrow keys
      │
      ▼
  scene_input  →  scene.theta1, scene.theta2  ──► wrap_pi
                                                      │
  scene.shoulder ──┐                                  │
                   ├──► compute_fk(S, θ1, θ2) ◄───────┘
                   │
                   ▼
              ArmPose pose
                   │
                   ├──► scene.elbow
                   └──► scene.hand
                            │
                            ▼
                       scene_draw
                  ┌────────────────┐
                  │ draw_line(S,E) │
                  │ draw_line(E,H) │
                  │ draw_point('*')│
                  │ draw_point('O')│
                  │ draw_point('@')│
                  └────────────────┘
                            │
                            ▼
                     wnoutrefresh()
                      + doupdate()
```

## Function Breakdown

### `compute_fk(S, theta1, theta2) → ArmPose`
Purpose: forward kinematics for a 2-link planar arm.
Steps:
1. `E = (S.x + L1·cos θ1, S.y + L1·sin θ1)`
2. `H = (E.x + L2·cos(θ1+θ2), E.y + L2·sin(θ1+θ2))`
3. Return `(ArmPose){E, H}`

That's the entire algorithm. Pure function, no branches, four trig calls.

### `wrap_pi(a) → float`
Purpose: fold an angle into (−π, π].
Steps:
1. While `a > π`: `a -= 2π`
2. While `a < −π`: `a += 2π`
3. Return `a`

Cosmetic only — the trig calls don't care about the magnitude of `a`.

### `scene_input(s, ch)`
Purpose: translate one keypress.
Steps:
1. If not paused:
   a. Dispatch arrow keys → ±= ANGLE_STEP_RAD on `s->theta1` or `s->theta2`
   b. `s->theta1 = wrap_pi(s->theta1)`; same for theta2
2. Always-on: space toggles paused; 'r' calls scene_init

### `scene_draw(s)`
Purpose: paint the five elements.
Steps:
1. Read `S = s->shoulder`, `E = s->elbow`, `H = s->hand`
2. `draw_line(S, E, PAIR_LINK_UPPER, ...)`
3. `draw_line(E, H, PAIR_LINK_FORE, ...)`
4. `draw_point(H, '*', PAIR_HAND, ...)`
5. `draw_point(E, 'O', PAIR_JOINT, ...)`
6. `draw_point(S, '@', PAIR_SHOULDER, ...)`

## Pseudocode — Core Loop

```
setup:
  install_signals()
  atexit(screen_cleanup)
  screen_init()
  scene_init(scene, rows, cols)

main loop (while running):

  1. handle resize (if SIGWINCH fired):
       endwin() + refresh()
       getmaxyx() → new rows, cols
       scene_resize(scene, rows, cols)

  2. (θ1, θ2) = read_input_angles():
       drain getch():
         on 'q'/ESC: running = 0
         else: scene_input(scene, ch)

  3. (E, H) = compute_fk(S, θ1, θ2):
       pose = compute_fk(scene.shoulder, scene.theta1, scene.theta2)
       scene.elbow = pose.elbow
       scene.hand  = pose.hand

  4. clear / draw / present:
       erase()
       scene_draw(scene)
       screen_hud(scene, fps)
       wnoutrefresh + doupdate

  5. frame cap:
       elapsed = clock_ns() − t_now
       clock_sleep_ns(TICK_NS − elapsed)
```

## Key Patterns to Internalize

**Cumulative angles are the FK chain rule.** Each link inherits the SUM of all upstream joint angles for its absolute orientation in world space. That is what makes FK a CHAIN — the math literally chains the rotations together via summation. It generalises to N links by extending the sum: `a_n = Σ(θ_1..θ_n)`.

**Position is a deterministic function of angles.** No solving, no iteration, no branching. Six trig calls (or 2N for N links) and you have the full chain configuration. This O(1) determinism is what makes FK the foundation of every animation pipeline — pose first, render second.

**FK is the inverse of IK in problem direction, not difficulty.** Given the same chain, "angles → positions" is trivial; "positions → angles" is hard. The asymmetry comes from positions being unique per angles, but angles being NON-unique per position. Studying both files side-by-side puts that asymmetry on the screen.

**Pure function for the math, scene for the cache.** `compute_fk` takes three scalars and returns a struct. It has no globals, no side effects, no scene reference. The Scene caches the result so renderer and HUD do not have to know how it was computed. Same separation pattern as `solve_ik` in the IK file.

**The `'*'` role-flip is the pedagogical gold.** In IK '*' marks user input; in FK '*' marks a computed output. Same glyph, opposite meaning. That single inversion is what carries the conceptual point of having both files.
