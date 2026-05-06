# Pass 1 — ik_helloworld: Analytical 2-link Inverse Kinematics via Law of Cosines

## From the Source

**Algorithm:** Closed-form analytical inverse kinematics for a planar 2-link arm. Shoulder S, elbow E, and target T form a triangle with sides L1 (constant), L2 (constant), and d = |T − S|. All three sides are known, so the law of cosines delivers the angle α at the shoulder vertex in one expression. Direction φ from S to T is one `atan2`. Elbow sits L1 from S in direction (φ ± α), where the ± selects between the two valid solutions (elbow up / elbow down). No iteration, no convergence test, no Jacobian, no FABRIK passes — pure trigonometry. The hand position is the target itself, since `clamp_to_reach` guarantees the triangle is always reachable, and when reachable |E − T| comes out exactly L2 by construction.

**Math:** `cos α = (L1² + d² − L2²) / (2·L1·d)` is the law of cosines applied at the S vertex of triangle SET. The cosine is clamped to [−1, 1] before `acosf` to absorb floating-point drift on boundary configurations. `φ = atan2(T.y − S.y, T.x − S.x)` is the four-quadrant arctan giving the shoulder→target heading. `elbow_angle = φ ± α` then places E via `S + L1·(cos elbow_angle, sin elbow_angle)`. The two-solution ambiguity is the famous "elbow up vs elbow down" choice — pressing 'f' toggles the sign.

**Performance:** O(1) per frame. One `sqrtf`, one `acosf`, one `atan2f`, two `cosf`/`sinf`, a handful of multiplies. Below the noise floor of one ncurses redraw — frame budget is dominated entirely by terminal I/O. There is no fixed-step accumulator, no alpha lerp, no `prev_pos` snapshot: the solver runs once per render frame and the result is drawn directly.

**Data-structure:** Three `Vec2` points (shoulder, target, elbow) and two scalar link lengths, all in pixel space. `solve_ik(S, T, elbow_up) → E` is a pure function returning a `Vec2`. The main loop caches the result as `scene.elbow` so renderer and HUD read it without recomputing. There is no Arm struct, no chain array, no joint pool — just the three points the algorithm reasons about.

## Core Idea

This is the textbook hello-world of inverse kinematics. A two-link arm hangs off a fixed shoulder at screen centre. The user moves ONE target with the arrow keys, and every frame the solver computes where the elbow must bend so the hand (= the target) sits L2 from the elbow and L1 from the shoulder. Three points, two lines, one solved triangle.

The conceptual insight: with two links, IK is exact. There is no iteration loop, no convergence threshold, no fallback. The shoulder, elbow, and target are vertices of a triangle whose three side lengths you already know — and the law of cosines gives you every angle in that triangle in closed form. Past two links the math branches into multi-page case analyses and most projects switch to iterative solvers (FABRIK, Jacobian); for two links, the closed form IS the lesson.

## The Mental Model

Reach for a coffee cup with your right hand. Your shoulder didn't move — it can't, it's bolted to your torso. Your hand IS the cup — you commanded it there. So WHERE did your elbow go? It bent automatically to whatever angle was required for an upper arm of fixed length plus a forearm of fixed length to reach from your stationary shoulder to your chosen hand position. Your nervous system solved a triangle.

That is IK in one paragraph. The user inputs the hand. The algorithm outputs the elbow. The shoulder doesn't move because it's nailed down. The third leg of the conceptual stool — clamp_to_reach — ensures the triangle is always valid by pulling the target back onto the reach disc whenever the user pushes past the rim. The forearm |E − T| therefore stays pinned at exactly L2 forever; the visible arm length never grows.

## Data Structures

### Scene struct
```
shoulder    Vec2  — pinned anchor S, set once at init/resize
target      Vec2  — moveable T, the only point user controls
elbow       Vec2  — solver output E, refreshed each frame
elbow_up    bool  — selects between the two valid IK solutions
paused      bool  — when true, arrow keys are ignored
rows, cols  int   — current terminal cell extents
```

### Vec2
```
x, y   float — pixel-space position; x increases east, y increases south
```

All positions are in square pixel space. Cell coordinates appear only inside `px_to_cell_x/y()` at draw time, undoing the 8:16 cell aspect ratio.

### Color pair layout
```
PAIR_HUD          bright yellow (226) — status row
PAIR_HINT         bright cyan   ( 51) — hint row
PAIR_LINK_UPPER   sky cyan      ( 45) — upper arm S→E
PAIR_LINK_FORE    orange        (208) — forearm  E→T
PAIR_JOINT        white         (255) — elbow marker
PAIR_SHOULDER     lime          (118) — anchor marker '@'
PAIR_TARGET       red           (196) — target marker '*'
```

## The Main Loop

The loop is the canonical IK skeleton from the user-facing pseudocode, line by line:

1. **`T = read_input()`.** Drain `getch()` in nodelay mode. Each arrow key nudges `scene.target` by `KEY_STEP_PX = 4` pixels; `scene_input` then runs `clamp_to_screen` and `clamp_to_reach` so T stays both visible and inside the reach disc. `'f'` flips elbow side, `space` pauses, `'r'` resets, `'q'`/ESC exits.

2. **`E = solveIK(S, T)`.** A single call: `scene.elbow = solve_ik(scene.shoulder, scene.target, scene.elbow_up)`. The result is cached in the scene so renderer and HUD can read it without re-solving.

3. **`clear_screen()`.** `erase()`.

4. **`draw_line / draw_point ×5`.** `scene_draw` paints upper arm S→E, forearm E→T, then the three markers in stacking order — '*' target, 'O' elbow, '@' shoulder last so the pinned anchor always wins the cell contest.

5. **`present_screen()`.** `wnoutrefresh(stdscr)` + `doupdate()`.

6. **Frame cap.** Sleep until `1/60s` has elapsed since the start of this frame. Sleeping before the next frame's I/O keeps terminal latency off the timing budget.

## Non-Obvious Decisions

**Why analytical and not FABRIK?**
For two links the law of cosines is exact and one-shot — no iteration, no convergence test, no degenerate-pose loop. Iterative solvers become attractive only past two links, when the analytical solution branches into multi-page case analyses. For a hello-world, the closed form IS the lesson; FABRIK is studied in `ik_arm_reach.c` where the arm has four links and the analytical approach would explode.

**Why clamp the target to the reach disc instead of handling unreachable cases?**
Users objected that drawing the forearm as a line E→T when |T − S| > L1+L2 produced a visibly stretched forearm. The simplest fix is to refuse to let T leave the disc: `clamp_to_reach` pulls T back radially onto the rim whenever it would otherwise wander outside. The triangle SET is then always valid, the solver never has to handle "no solution exists", and |E − T| stays pinned at exactly L2 forever — the visible arm length never grows.

**Why the elbow-flip 'f' key?**
For any reachable target a 2-link arm has TWO valid configurations: elbow on one side of the S→T line, or elbow on the other (mirror image). The `±α` in `elbow_angle = φ ± α` selects which one. Pressing 'f' toggles `elbow_up` and the solver swaps in real time. That toggle is the conceptual heart of inverse kinematics — IK is harder than FK precisely because solutions are non-unique.

**Why store `scene.elbow` instead of taking E as a parameter to draw and HUD?**
Originally `Vec2 E = solve_ik(...)` lived as a local variable in `main()` and was passed explicitly to `scene_draw(&scene, E)`. Storing E as a Scene field lets every consumer (renderer, HUD, future debug overlays) read the elbow without re-running the solver and without taking E as a parameter. The solver remains a pure function with no scene reference; the scene is its cached output.

**Why `1e-6f` floor in the degenerate (d ≈ 0) branch?**
When the user drags the target onto the shoulder, d → 0 and `cos α = (L1² + d² − L2²) / (2·L1·d)` divides by zero. With L1 == L2 the forearm folds back onto the upper arm and the hand returns to the shoulder; we pick a default elbow direction (+X) so the chain stays visible instead of vanishing into NaNs. The floor's specific value is loose — anything below the per-frame motion (4 px) avoids triggering on normal input, and the fallback is benign.

**Why `clampf(cos_a, -1, 1)` before `acosf`?**
Floating-point error during the law-of-cosines computation can push `cos_a` to `1.0000001` on boundary configurations (target at exactly the rim of the reach disc). `acosf` of any value > 1 returns NaN, which propagates through the entire frame. The clamp is a one-line safety net.

## State Machines

### Input actions
| Key | Effect (when not paused) | Effect when paused |
|-----|--------------------------|--------------------|
| ↑↓←→ | Nudge target by ±KEY_STEP_PX | Ignored |
| f | Toggle elbow_up | Toggle elbow_up |
| space | Toggle paused | Toggle paused |
| r | Reset target to default | Reset target to default |
| q / ESC | Exit | Exit |

### App-level state
```
        ┌──────────────────────────────────────────┐
        │               RUNNING                    │
        │                                          │
        │  [scene_input]  → update target / flag   │
        │  [solve_ik]     → compute scene.elbow    │
        │  [scene_draw]   → 5 draw calls           │
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
| TARGET_FPS | 60 | Render cap. No physics implications — solver runs in microseconds. |
| L1_PX | 80 | Upper arm length, pixels. Increase: longer arm, more screen used. |
| L2_PX | 80 | Forearm length, pixels. Setting L1 ≠ L2 introduces the inner-reach degenerate (d < |L1−L2|) — keep equal for hello-world simplicity. |
| KEY_STEP_PX | 4 | Pixels per arrow keystroke. Half a cell on the wide axis at default 30 Hz key repeat → ~120 px/sec sweep. |
| INITIAL_TARGET_REACH_FRAC | 0.7 | Spawn target inside the disc. Setting to 1.0 spawns on the rim (still works), >1 would spawn outside but `clamp_to_reach` would pull it back on frame zero. |
| IK_EPSILON_PX | (deprecated) | The original FABRIK draft used this for early-exit. Removed once we settled on the closed-form solver. |

## Open Questions for Pass 3
- The `clampf(cos_a, -1, 1)` is a defensive guard against FP drift. With L1 == L2 and d clamped to L1+L2, can the cosine ever actually exceed 1, or is it always within 1 ulp of 1 at the boundary?
- Pressing 'f' currently flips elbow_up regardless of mode. If we wanted "stable elbow side across target moves", we'd need to track the sign of the cross product `(T−S) × (E−S)` and update elbow_up automatically when the user crosses S. Worth doing?
- The fallback unit vector in the degenerate `d < 1e-6` branch is hard-coded to `(1, 0)`. Could a smarter fallback (e.g., the previous frame's elbow direction) avoid the visible "snap" when target crosses the shoulder?
- The HUD shows `|E−T|`. With the law of cosines and clamp_to_reach, this should be EXACTLY L2 every frame. In practice it shows as `80.0` to one decimal — what's the actual floating-point error budget across long sessions?
- The screen clamp + reach clamp are applied separately. Are there target positions where the order matters (i.e., on-screen-but-outside-reach vs off-screen-but-inside-reach)?

---

# Structure

| Symbol | Type | Size (approx) | Role |
|--------|------|---------------|------|
| `Scene.shoulder` | `Vec2` | 8 B | pinned anchor, set once |
| `Scene.target` | `Vec2` | 8 B | user-controlled hand position |
| `Scene.elbow` | `Vec2` | 8 B | cached IK output, refreshed each frame |
| `Scene.elbow_up` | `bool` | 1 B | which of the two solutions |
| `Scene.paused` | `bool` | 1 B | input freeze |
| `Scene` total | struct | ~32 B | full per-frame state of the demo |

---

# Pass 2 — ik_helloworld: Pseudocode

## Module Map

| Section | Purpose |
|---------|---------|
| §1 config | All tunables: TARGET_FPS, L1/L2, KEY_STEP_PX, color pair IDs, CELL_W/H |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` — monotonic timing |
| §3 color | `color_init()` — bind 7 semantic pairs |
| §4 coords | `px_to_cell_x/y` — pixel↔cell aspect bridge (one-way at draw time) |
| §5 ik | `Vec2` + helpers; `solve_ik(S, T, elbow_up) → E` — the pure black box |
| §6 scene | `Scene` state, `clamp_to_screen` + `clamp_to_reach`, `scene_init/resize/input/draw` |
| §7 screen | `screen_init/cleanup/present/hud` — ncurses double-buffer |
| §8 app | Signal handlers, `main()` — the five-line render kernel |

## Data Flow Diagram

```
  arrow keys
      │
      ▼
  scene_input  →  scene.target  ──► clamp_to_screen ──► clamp_to_reach
                                                              │
                                                              ▼
  scene.shoulder ──┐
                   ├──► solve_ik(S, T, elbow_up) ──► scene.elbow
  scene.elbow_up ──┘                                      │
                                                          │
                                                          ▼
                                                    scene_draw
                                                  ┌────────────────┐
                                                  │ draw_line(S,E) │
                                                  │ draw_line(E,T) │
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

### `solve_ik(S, T, elbow_up) → Vec2`
Purpose: closed-form 2-link IK; return elbow position.
Steps:
1. `d_vec = T − S`; `d = |d_vec|`
2. If `d < 1e-6f`: return `(S.x + L1, S.y)` — degenerate fallback
3. `cos_a = (L1² + d² − L2²) / (2·L1·d)`; clamp to [−1, 1]
4. `α = acosf(cos_a)`
5. `φ = atan2f(d_vec.y, d_vec.x)`
6. `side = elbow_up ? −1 : +1`
7. `elbow_angle = φ + side · α`
8. Return `(S.x + L1·cos(elbow_angle), S.y + L1·sin(elbow_angle))`

### `clamp_to_reach(t, shoulder, reach) → Vec2`
Purpose: pull T radially onto the reach disc if it has wandered off.
Steps:
1. `d = t − shoulder`; `len = |d|`
2. If `len ≤ reach`: return t unchanged
3. Return `shoulder + d · (reach / len)` — same direction, snapped to rim

### `scene_input(s, ch)`
Purpose: translate one keypress.
Steps:
1. If not paused: dispatch arrow keys → nudge `s->target.x/y` by ±KEY_STEP_PX
2. `s->target = clamp_to_screen(...)`
3. `s->target = clamp_to_reach(...)`
4. Always-on: 'f' flips elbow_up; space toggles paused; 'r' calls scene_init

### `scene_draw(s)`
Purpose: paint the five elements per the pseudocode skeleton.
Steps:
1. Read `S = s->shoulder`, `E = s->elbow`, `T = s->target`
2. `draw_line(S, E, PAIR_LINK_UPPER, ...)`
3. `draw_line(E, T, PAIR_LINK_FORE, ...)`
4. `draw_point(T, '*', PAIR_TARGET, ...)`
5. `draw_point(E, 'O', PAIR_JOINT, ...)`
6. `draw_point(S, '@', PAIR_SHOULDER, ...)` — last so it always wins

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

  2. T = read_input():
       drain getch():
         on 'q'/ESC: running = 0
         else: scene_input(scene, ch)

  3. E = solveIK(S, T):
       scene.elbow = solve_ik(scene.shoulder, scene.target, scene.elbow_up)

  4. clear_screen():
       erase()

  5. draw_line / draw_point ×5:
       scene_draw(scene)
       screen_hud(scene, fps)

  6. present_screen():
       wnoutrefresh + doupdate

  7. frame cap:
       elapsed = clock_ns() − t_now
       clock_sleep_ns(TICK_NS − elapsed)

cleanup:
  endwin()
```

## Key Patterns to Internalize

**The triangle solves it all.** The three side lengths L1, L2, d uniquely determine every angle in triangle SET via the law of cosines. No part of the IK problem requires more than that one equation plus an `atan2` for orientation.

**Clamp at the input layer, not the solver.** Any well-behaved invariant (reachability, screen bounds) belongs at the input layer where the user's intent is captured. The solver assumes its preconditions and stays clean. Two clamps in `scene_input` keep the law of cosines from ever seeing a degenerate triangle.

**Cache the solver output in the scene.** The renderer and HUD both need the elbow position. Storing it in `scene.elbow` after the once-per-frame `solve_ik` call removes the parameter plumbing and lets new consumers (debug overlays, trace renderers) drop in without function-signature changes.

**The two-solution flip is THE conceptual point of IK.** Pressing 'f' instantly mirrors the elbow across the S→T line — same hand position, different arm shape. That ambiguity is what makes IK harder than FK and why iterative solvers exist for longer chains.

**Pure functions for the math, scene for the state.** `solve_ik` is a black box: three inputs, one output, no globals, no side effects. The Scene struct is the boundary between pure math and stateful UI. This is the pattern every demo in this codebase aims for.
