# Pass 1 — fk_ik_helloworld: 3-link arm with FK and IK modes (2+1 hybrid IK)

## From the Source

**Algorithm:** Two solvers in one file, toggled with `'m'`. **FK** is forward propagation of cumulative angles: `E = S + L1·(cos θ1, sin θ1)`; `W = E + L2·(cos(θ1+θ2), sin(θ1+θ2))`; `H = W + L3·(cos(θ1+θ2+θ3), sin(θ1+θ2+θ3))`. Six trig calls, no branching. **IK** is the 2+1 hybrid: (1) shrink the target by L3 along S→T to get a virtual 2-link target T2; (2) run plain 2-link analytical IK (law of cosines) from S to T2 to place E and W; (3) place H at L3 from W pointing at the real T. Hand always lands on T (when target is inside the reach disc, which `clamp_to_reach` guarantees), wrist always points at T, and the arm has a unique configuration up to the elbow_up flip.

**Math:** FK uses the cumulative-angle pattern: link N's absolute angle is the sum of all upstream joint angles. IK's hybrid is the simplest answer to 3-link IK underdetermination. Full 3-link IK is genuinely under-constrained — for any reachable target an infinite family of configurations satisfies the link-length constraints (a 1-parameter "redundancy"). Pinning the third link's direction (always `dir(W → T)`) collapses that redundancy to a finite set, then plain 2-link IK solves the remaining problem. Substituting back: when |T − S| ≤ L1+L2+L3, step 2 succeeds with W = T2 = T − L3·dir(T−S), and step 3 places H = W + L3·dir(T−W) = T2 + L3·dir(T−S) = T exactly.

**Performance:** O(1) per frame in either mode. FK does six trig calls. IK does one `acosf`, one `atan2f`, two `cosf`/`sinf`, plus two normalisations (each a `sqrtf` + reciprocal multiply). No iteration, no convergence loop in either branch. Variable timestep at render rate; no fixed-step accumulator.

**Data-structure:** One Scene struct holds the fixed shoulder, three FK angles (theta1/2/3), the IK target, the elbow_up flag, the current Mode (FK or IK), and three cached output positions (elbow, wrist, hand). `compute_fk` and `solve_ik3` are pure functions returning an `ArmPose` struct of three Vec2s. The internal helper `solve_ik2` returns a `TwoLinkPose` (elbow + tip) used as a building block by `solve_ik3`. The main loop calls `scene_recompute` once per frame, which dispatches on Mode and writes the result into the scene cache.

## Core Idea

This is the third file in a graduated series — `ik_helloworld.c` (2-link IK) → `fk_helloworld.c` (2-link FK) → this file (3-link arm with both modes). The third joint is where the math stops being "toy" and starts feeling like a real animation system. The arm has a wrist now, so the hand has its own orientation independent of the upper-arm direction. And the IK problem for 3+ links becomes underdetermined in a way the 2-link version was not — there is no longer a unique "elbow up vs elbow down" pair; there is a continuous family of valid poses.

The conceptual leap: **more joints = more ambiguity**. IK must CHOOSE one solution among many. The choice is a design decision, not a mathematical truth. This file picks the simplest possible: the wrist always points AT the target. That single design choice (the "wrist invariant") collapses the redundancy and lets the rest of the math fall out as plain 2-link IK on a virtual target one link's length back along the S→T direction. Real animation rigs use this exact split — IK for the gross "reach", FK for the fine "pose" (hand orientation, finger curl, eye gaze).

## The Mental Model

Same arm, two control modes.

**FK mode ("puppeteer mode"):** you rotate every joint by hand. The hand goes wherever the math says. Total control over pose, no goal. Press `a/d` for shoulder, `w/s` for elbow, `z/x` for wrist; the cumulative-angle chain places everything else.

**IK mode ("goal mode"):** you point at a spot in space. The arm contorts itself to put the hand there. Total control over destination, no direct say in the joint angles. Press the arrow keys to move the target; the 2+1 hybrid does the rest. Press `'f'` to flip the elbow side (the one residual ambiguity that survived the redundancy collapse).

```
shoulder    @        ← S, fixed
             ╲ θ1
              ╲  L1
               ╲
        elbow   O    ← E
                ╲ θ2 (relative to upper arm)
                 ╲ L2
                  ╲
        wrist      o ← W
                    ╲ θ3 (relative to forearm)
                     ╲ L3
                      ╲
             hand      *   ← H
                           ← + (only in IK mode: the target T)
```

These are the two halves of a real animation system. FK is what a 3-D modeller uses to pose a character frame by frame; IK is what a game uses to make the character's hand grab a doorknob. The toggle key `'m'` literally swaps which solver fires — the arm itself is identical between modes, only the input semantics change.

## Data Structures

### Mode enum
```
MODE_FK   user controls (θ1, θ2, θ3); compute_fk runs
MODE_IK   user controls T (and elbow_up); solve_ik3 runs
```

### Scene struct
```
shoulder         Vec2  — pinned anchor S, set once at init/resize
theta1, theta2,
theta3           float — FK joint angles, radians (cumulative for forearm/wrist)
target           Vec2  — IK target T, drawn as '+' only in IK mode
elbow_up         bool  — pick one of the two valid 2-link IK solutions
elbow, wrist,
hand             Vec2  — solver output, refreshed each frame by scene_recompute
mode             Mode  — MODE_FK or MODE_IK
paused           bool  — when true, value-changing keys are ignored
rows, cols       int   — current terminal cell extents
```

### ArmPose
```
elbow, wrist, hand   Vec2 triple returned by compute_fk and solve_ik3
```

### TwoLinkPose (internal helper return type)
```
elbow, tip           Vec2 pair returned by solve_ik2
```

### Color pair layout
```
PAIR_HUD          bright yellow (226) — status row
PAIR_HINT         bright cyan   ( 51) — hint row
PAIR_LINK_UPPER   sky cyan      ( 45) — upper arm S→E
PAIR_LINK_FORE    orange        (208) — forearm  E→W
PAIR_LINK_HAND    magenta       (201) — hand link W→H
PAIR_JOINT        white         (255) — elbow + wrist markers
PAIR_SHOULDER     lime          (118) — anchor marker '@'
PAIR_HAND         red           (196) — hand marker '*'
PAIR_TARGET       gold          (220) — target marker '+', IK only
```

## The Main Loop

The frame skeleton has three steps regardless of mode:

1. **Input.** Drain `getch()`. Always-on keys (space pause, 'r' reset, 'm' mode toggle, 'f' elbow flip, 'q'/ESC quit) work even when paused. Mode-specific keys are gated on `!paused`. In FK mode the keys `a/d`, `w/s`, `z/x` nudge θ1/θ2/θ3 by ±ANGLE_STEP_RAD; in IK mode the arrows nudge T by ±KEY_STEP_PX, then `clamp_to_screen` and `clamp_to_reach` keep T inside both the screen and the reach disc.

2. **Solve.** `scene_recompute(&scene)` calls either `compute_fk` or `solve_ik3` based on `scene.mode`, then writes the resulting `ArmPose` into `scene.{elbow, wrist, hand}`. The main loop never calls the solvers directly.

3. **Render.** `erase()` → `scene_draw` (3 link lines + 1 target marker if IK mode + 4 arm markers, with `'@'` painted last so it always wins the cell contest) → `screen_hud` (mode-aware: FK shows angles in degrees, IK shows target distance and `|H−T|` and elbow side) → `wnoutrefresh` + `doupdate`.

4. **Frame cap.** Sleep until 1/60 s has elapsed.

## Non-Obvious Decisions

**Why a 2+1 hybrid IK and not a full 3-link analytical solver?**
Full 3-link analytical IK is genuinely hard. The solution set is a 1-parameter family (the "redundancy"), not a discrete pair, so the algebra branches into multi-page case analyses depending on which constraints you add. Iterative solvers like FABRIK handle it cleanly but require a convergence loop. The 2+1 split pins the third link's direction (always points at the target) so the redundancy collapses and the rest reduces to the 2-link analytical IK we already understand. It is the simplest design that teaches "more joints = more ambiguity = pick a strategy".

**Why does the wrist always point AT the target?**
This is the design choice that kills the 1-parameter redundancy. Step 1 of the hybrid (`T2 = T − L3·dir(S→T)`) is what enforces the constraint: by placing the virtual 2-link target one link's length back along the S→T line, we ensure the wrist W lands at T2 and the hand link W→H is collinear with S→T. The hand then ends up exactly on T. Other choices were possible (e.g., wrist always horizontal, wrist FK-controlled while elbow is IK-controlled), but "point at target" is the most visually pleasing default and the simplest to teach.

**Why does pause not block 'm', 'r', 'f' but does block value-changing keys?**
`m`, `r`, `f` are STRUCTURAL changes — they switch modes, reset state, or flip a discrete boolean. None of them is a "continuous nudge" of a value. Blocking them during pause would punish the user for inspecting a state — they should be able to flip into IK mode while looking at a paused FK pose, or vice versa. Value keys (a/d, w/s, z/x, arrows) ARE continuous nudges and pause exists precisely to freeze them.

**Why preserve mode on 'r' reset?**
Pressing `'r'` resets values within the current mode rather than dragging the user out of the mode they are experimenting in. This is implemented by separating `scene_reset` (resets values, keeps mode) from `scene_init` (full init, sets mode to FK). The reset key calls `scene_reset`; the first init at startup calls `scene_init`. Resize calls neither — it just re-anchors the shoulder and re-clamps the target.

**Why `scene_recompute` and not inline solver dispatch in main?**
Three call sites need the same solver-dispatch logic: `scene_init`, `scene_resize`, and the main loop. Extracting `scene_recompute` keeps the dispatch in one place and lets the call sites be one-line invocations. The main loop's frame kernel becomes literally `scene_recompute(&scene); erase(); scene_draw(&scene); ...` — readable as plain English.

**Why is `solve_ik2` extracted as a helper, not inlined in `solve_ik3`?**
The same 2-link IK math is the building block; pulling it out makes `solve_ik3` read as a clean three-step composition (pull back → 2-link IK → aim final link) rather than ten lines of trig with the high-level structure obscured. It also makes `solve_ik2` independently testable and reusable (e.g., if the next file in the series adds a 4-link FABRIK that initialises from a 2-link warm start).

**Why the cumulative-angle pattern in compute_fk and not absolute angles per joint?**
Joint angles in robotics are conventionally relative to the parent link. Storing absolute angles would mean each FK update touches all downstream joints; storing relatives means a per-joint nudge updates one number and `compute_fk` does the summation. The cumulative summation is the FK chain rule and generalises to N links unchanged.

**Why a HUD that changes per mode?**
Different modes care about different numbers. In FK the user is twiddling angles, so the HUD shows θ1, θ2, θ3 in degrees and the hint strip lists the angle keys. In IK the user is dragging a target, so the HUD shows target distance, `|H−T|` (which should be ~0 thanks to clamp_to_reach), and the elbow side; the hint strip lists arrows + 'f'. Showing both sets at once would clutter the bar; showing only the active set keeps the relevant readout in front of the user.

**Why is `clamp_to_reach` applied in IK mode but not FK mode?**
In FK there is nothing to clamp — the user is rotating angles, not chasing a target. The hand goes wherever the angles say, including past the reach disc would be (impossible because reach IS L1+L2+L3, so hand is always within reach by definition). In IK the user can drag the target arbitrarily and we need to ensure it stays within solver-validity range. The asymmetry between modes is itself a teaching point: IK has REACHABILITY as a real constraint; FK does not.

## State Machines

### Input actions
| Key | FK mode | IK mode | Effect when paused |
|-----|---------|---------|--------------------|
| a / d | θ1 ∓ ANGLE_STEP_RAD | (no effect) | Ignored |
| w / s | θ2 ∓ ANGLE_STEP_RAD | (no effect) | Ignored |
| z / x | θ3 ∓ ANGLE_STEP_RAD | (no effect) | Ignored |
| ↑↓←→ | (no effect) | T nudged ±KEY_STEP_PX | Ignored |
| f | (no effect) | elbow_up flipped | elbow_up flipped |
| m | mode → IK | mode → FK | Toggles |
| space | Toggle paused | Toggle paused | Toggle paused |
| r | scene_reset | scene_reset | scene_reset |
| q / ESC | Exit | Exit | Exit |

### App-level state
```
        ┌──────────────────────────────────────────────┐
        │                  RUNNING                     │
        │                                              │
        │   ┌──────────┐         ┌──────────┐          │
        │   │   FK     │ ◄─'m'─► │    IK    │          │
        │   │ angles   │         │  target  │          │
        │   └──────────┘         └──────────┘          │
        │       │                     │                │
        │   compute_fk            solve_ik3            │
        │       │                     │                │
        │       └──────► scene.{E,W,H} ◄────           │
        │                     │                        │
        │                scene_draw                    │
        │                     │                        │
        │                screen_hud (mode-aware)       │
        │                                              │
        │   SIGWINCH ──────────────────► scene_resize  │
        │   SIGINT/SIGTERM ────────────► running = 0   │
        └──────────────────────────────────────────────┘
                              │
                      'q' / ESC / signal
                              │
                              ▼
                            EXIT
                          (endwin)
```

### IK hybrid pipeline
```
           T (clamped to reach disc)
              │
              │  Step 1: pull back by L3 along S→T
              ▼
           T2  ──────►  solve_ik2(S, T2)  ──►  (E, W)
                              │
                              │  Step 3: aim L3 from W toward T
                              ▼
                              H
```

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|----------|---------|-------------------|
| TARGET_FPS | 60 | Render cap. Both solvers run in microseconds. |
| L1_PX | 80 | Upper arm length, pixels. |
| L2_PX | 80 | Forearm length. |
| L3_PX | 80 | Hand-link length. With L1=L2=L3 the inner-reach degenerate is gone (any d ∈ [0, 3·L] is reachable). |
| ANGLE_STEP_RAD | 2° (≈0.0349 rad) | Per FK keystroke. |
| KEY_STEP_PX | 4 | Per IK arrow keystroke. |
| INITIAL_TARGET_REACH_FRAC | 0.7 | Spawn target at 70% of full reach. |

## Open Questions for Pass 3
- The wrist invariant pins the third link to point at T. Other invariants are possible: "wrist horizontal", "wrist parallel to ground", "wrist FK-controlled while shoulder/elbow IK-controlled". Which feels best for which application? When would each be the right pick?
- The 2+1 hybrid extends naturally to 4+ links: solve N-1 link IK to put the wrist at T2 = T − L_last · dir(T−S), then point the last link at T. With more than 2 IK links, the inner solver itself becomes underdetermined and we are back to needing FABRIK or analytical case-analysis. Is the right next step "FABRIK + last-link aim", and would that constitute a fourth file in this series?
- `'f'` flips elbow_up; the wrist keeps pointing at T regardless. Is there a meaningful "wrist flip" that would mirror W across the line E→T? Or is that direction already pinned by the design?
- In FK mode, is there a sensible way to display "where would the hand land if I switched to IK mode with the current target"? A faint grey ghost arm? Useful for cross-mode learning, or distracting?
- The HUD's `|H−T|` readout in IK mode should be exactly 0 thanks to clamp_to_reach. What's the actual floating-point error budget over long sessions? Does it ever drift to 0.1 or 1 px?
- Currently the elbow flip survives mode toggles (the boolean lives in Scene, not per-mode state). Is that the right call? In FK mode the flag does nothing, so it just sits there until you next switch to IK.

---

# Structure

| Symbol | Type | Size (approx) | Role |
|--------|------|---------------|------|
| `Scene.shoulder` | `Vec2` | 8 B | pinned anchor |
| `Scene.theta1/2/3` | `float` × 3 | 12 B | FK joint angles, radians |
| `Scene.target` | `Vec2` | 8 B | IK target |
| `Scene.elbow_up` | `bool` | 1 B | IK side selector |
| `Scene.elbow/wrist/hand` | `Vec2` × 3 | 24 B | cached solver output |
| `Scene.mode` | `Mode` enum | 4 B | FK / IK |
| `Scene` total | struct | ~64 B | full per-frame state |
| `ArmPose` | struct | 24 B | return value of compute_fk and solve_ik3 |
| `TwoLinkPose` | struct | 16 B | return value of solve_ik2 helper |

---

# Pass 2 — fk_ik_helloworld: Pseudocode

## Module Map

| Section | Purpose |
|---------|---------|
| §1 config | TARGET_FPS, L1/L2/L3, ANGLE_STEP_RAD, KEY_STEP_PX, color pair IDs, CELL_W/H |
| §2 clock | `clock_ns`, `clock_sleep_ns` — monotonic timing |
| §3 color | `color_init` — bind 9 semantic pairs |
| §4 coords | `px_to_cell_x/y` — pixel↔cell aspect bridge |
| §5 ik_fk | `Vec2` + helpers; `wrap_pi`; `compute_fk`; `solve_ik2` (helper); `solve_ik3` (hybrid) |
| §6 scene | `Mode` enum, `Scene` state, `clamp_to_reach`, `scene_recompute/init/reset/resize/input/draw` |
| §7 screen | `screen_init/cleanup/present/hud` — ncurses double-buffer; HUD is mode-aware |
| §8 app | Signal handlers, `main()` — three-step loop kernel |

## Data Flow Diagram

```
                    keypress
                       │
                       ▼
                  scene_input
                  ┌──────────┐
                  │ if FK:   │  → scene.theta1/2/3 (wrap_pi)
                  │ if IK:   │  → scene.target (clamp_to_screen, clamp_to_reach)
                  │ always:  │  → mode, paused, elbow_up, reset
                  └──────────┘
                       │
                       ▼
                scene_recompute
              ┌────────────────┐
              │ if FK:          │  → compute_fk(S, θ1, θ2, θ3)
              │ if IK:          │  → solve_ik3(S, T, elbow_up)
              └────────────────┘
                       │
                       ▼
              ArmPose {E, W, H}
                       │
                       ├──► scene.elbow
                       ├──► scene.wrist
                       └──► scene.hand
                                 │
                                 ▼
                            scene_draw
                       ┌────────────────────┐
                       │ 3 link lines       │
                       │ + '+' if IK mode   │
                       │ + 4 arm markers    │
                       └────────────────────┘
                                 │
                                 ▼
                            screen_hud (mode-aware)
                                 │
                                 ▼
                          wnoutrefresh + doupdate
```

## Function Breakdown

### `compute_fk(S, th1, th2, th3) → ArmPose`
Purpose: 3-link forward kinematics.
Steps:
1. `a1 = th1`; `a2 = th1 + th2`; `a3 = th1 + th2 + th3`
2. `E = (S.x + L1·cos a1, S.y + L1·sin a1)`
3. `W = (E.x + L2·cos a2, E.y + L2·sin a2)`
4. `H = (W.x + L3·cos a3, W.y + L3·sin a3)`
5. Return `(ArmPose){E, W, H}`

### `solve_ik2(S, T, elbow_up, L1, L2) → TwoLinkPose`
Purpose: analytical 2-link IK helper, used by solve_ik3.
Steps:
1. `d_vec = T − S`; `d = |d_vec|`
2. If `d < 1e-6`: return degenerate fallback
3. `cos α = (L1² + d² − L2²) / (2·L1·d)`, clamp to [−1, 1]
4. `α = acosf(cos α)`; `φ = atan2(d_vec.y, d_vec.x)`
5. `side = elbow_up ? −1 : +1`; `elbow_angle = φ + side·α`
6. `E = S + L1·(cos elbow_angle, sin elbow_angle)`
7. `dir = normalize(T − E)`; `tip = E + L2·dir`  (= T when reachable)
8. Return `(TwoLinkPose){E, tip}`

### `solve_ik3(S, T, elbow_up) → ArmPose`
Purpose: 2+1 hybrid 3-link IK.
Steps:
1. `dir_st = normalize(T − S)`; `T2 = T − L3·dir_st`  ← virtual 2-link target
2. `tl = solve_ik2(S, T2, elbow_up, L1, L2)`         ← places E, W
3. `dir_wt = normalize(T − tl.tip)`
4. `H = tl.tip + L3·dir_wt`                          ← aims last link at T
5. Return `(ArmPose){tl.elbow, tl.tip, H}`

### `scene_recompute(s)`
Purpose: dispatch on mode and cache solver output.
Steps:
1. If `s->mode == MODE_FK`: `pose = compute_fk(s->shoulder, s->theta1, s->theta2, s->theta3)`
2. Else: `pose = solve_ik3(s->shoulder, s->target, s->elbow_up)`
3. `s->elbow = pose.elbow; s->wrist = pose.wrist; s->hand = pose.hand`

### `scene_input(s, ch)`
Purpose: translate one keypress, mode-aware.
Steps:
1. Always-on switch on ch: handle space, 'r', 'm', 'f'
2. If paused: return
3. If FK mode: dispatch a/d/w/s/z/x → ±= ANGLE_STEP_RAD on theta1/2/3, then wrap_pi each
4. Else (IK mode): dispatch arrows → ±= KEY_STEP_PX on target.x/y, then clamp_to_screen + clamp_to_reach

### `scene_draw(s)`
Purpose: paint per the pseudocode skeleton; '+' target only in IK mode.
Steps:
1. 3 link lines: S→E (cyan), E→W (orange), W→H (magenta)
2. If `s->mode == MODE_IK`: draw target '+' (gold)
3. 4 arm markers: '*' hand, 'o' wrist, 'O' elbow, '@' shoulder (last)

## Pseudocode — Core Loop

```
setup:
  install_signals()
  atexit(screen_cleanup)
  screen_init()
  scene_init(scene, rows, cols)         ← mode = MODE_FK, then scene_reset

main loop (while running):

  1. handle resize:
     if SIGWINCH:
       endwin() + refresh()
       scene_resize(scene, rows, cols)

  2. drain input:
     while (ch = getch()) != ERR:
       if ch in {'q', ESC}: running = 0
       else: scene_input(scene, ch)

  3. solve and cache:
     scene_recompute(scene)             ← dispatches FK or IK by mode

  4. clear / draw / present:
     erase()
     scene_draw(scene)
     screen_hud(scene, fps)             ← mode-aware: angles vs target
     wnoutrefresh + doupdate

  5. frame cap:
     elapsed = clock_ns() − t_now
     clock_sleep_ns(TICK_NS − elapsed)
```

## Interactions Between Modules

```
App
 ├── owns Scene (mode, angles, target, cached output positions)
 ├── owns Screen (ncurses state, rows/cols)
 └── main loop: input → scene_recompute → draw → present

Scene
 ├── scene_input dispatches by mode
 ├── scene_recompute dispatches by mode → scene cache
 ├── scene_draw reads cache, paints '+' only in IK mode
 └── scene_reset preserves mode (UX choice)

§5 ik_fk solvers
 ├── compute_fk    — pure, FK
 ├── solve_ik2     — pure, helper
 └── solve_ik3     — pure, calls solve_ik2 + last-link aim

§7 HUD
 └── reads scene.mode and renders mode-specific row 1 + hint strip

Signal handlers
 ├── SIGINT/SIGTERM → running = 0
 └── SIGWINCH → need_resize = 1 (volatile sig_atomic_t)
```

## Key Patterns to Internalize

**More joints means more ambiguity, not more control.** 2-link IK had two valid solutions (elbow up/down). 3-link IK has infinitely many. With every additional joint, the SET of valid configurations grows from a discrete pair to a 1-parameter family to a 2-parameter family. IK must CHOOSE one solution among many. The chosen strategy is a design decision, not a mathematical truth.

**The wrist invariant collapses redundancy.** Pinning the third link's direction (`always points at T`) is one design choice that reduces the 1-parameter family to a single (elbow_up, elbow_down) pair. Other choices exist; this one is the simplest to teach and produces visually pleasing results.

**Hybrid IK is how real rigs work.** IK for the gross "reach" + FK for the fine "pose" is the standard split in production animation systems. The 2+1 hybrid in this file is the toy version; real systems extend it with FABRIK chains, joint limits, and pose blending. The structural pattern — IK does the gross task, FK finishes the details — generalises far past two links.

**Mode-aware HUDs are part of the UI.** Different modes care about different numbers. Showing both at once clutters; showing only the active set keeps the relevant readout in front of the user. The hint strip's key list also changes by mode, so the user always sees the right keys for what they are doing.

**Pure functions for the solvers, scene for the cache, mode for the dispatch.** The three solvers are pure functions of their inputs. The scene caches their output. The mode field decides which solver fires. This separation of concerns makes the code read top-to-bottom: input layer mutates scene state, dispatch picks a solver, solver writes cache, renderer reads cache. No hidden coupling.

**FK and IK side by side teaches the asymmetry.** Pressing `'m'` toggles between forward (angles → positions) and inverse (positions → angles via 2+1 hybrid). Same arm, same screen, opposite data flow. That toggle is the file's most concentrated teaching moment — five seconds of pressing 'm' clarifies what an entire chapter of robotics text would belabour.
