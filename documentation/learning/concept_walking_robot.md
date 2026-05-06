# Pass 1 — walking_robot.c: Procedural bipedal walk via FK swing + IK stance

## Core Idea

A stick-figure biped that walks left/right across the screen by mixing **two** kinds of kinematics on each leg, gated by phase:

- **Swing leg** (leg in the air) — *Forward Kinematics*. Pick joint angles from a pair of sinusoids; compute the foot's position from the joint chain.
- **Stance leg** (leg on the ground) — *Inverse Kinematics*. The foot is locked to where it was planted; the hip moves forward; solve the knee position from those two endpoints by the law of cosines.

A single oscillator drives both legs (offset by π) and the body bob/sway, so the whole gait is one float `phase` advancing at `2π · walk_freq · dt` per frame. The FK and IK halves of each cycle blend at the boundary because the touchdown FK pose is captured into `foot_lock` exactly when the IK takes over. No state machines, no spring controllers — just trig and one analytical IK call.

## The Mental Model

A pair of pendulums hanging from the same hub, swinging 180° out of phase. While one is mid-air sweeping forward, the other is pinned to the ground and the hub (the body) rolls over the planted foot. As the swing leg passes through forward and contacts the ground, ownership swaps — the new pinned foot is wherever the swing leg's FK landed it, and the old planted leg lifts off into a fresh swing. The illusion of "walking" is just the body advancing horizontally at the speed at which the swing leg can reach forward and the body can roll over the planted foot.

The trick that makes it READ as natural human walking is two separate corrections — both surprisingly easy to get wrong:

1. **Knee-fold direction** — a real knee folds so the heel rises toward the butt. In our angle convention that means `shin_angle = thigh_angle − knee_bend`, NOT `+ knee_bend`. The plus sign produces the famous "Smooth Criminal" lean (lower body slants forward while upper body stays vertical).
2. **Walk speed coupled to gait frequency** — the body must advance exactly one stride length per stride period. If `walk_speed < walk_freq · stride_length`, the body never crosses the planted foot, and the stance leg stays leaning forward through every step. So we don't expose `walk_speed` as a free parameter — it's derived: `walk_speed = walk_freq · stride_length` where `stride_length = 2 · (UPPER + LOWER) · sin(SWING_AMP)`.

## Data Structures

### Robot (§5.1)
```
float x, phase;                — body x in pixels, gait phase in rad
float walk_freq, walk_speed;   — Hz + derived px/sec (see robot_set_pace)
int   direction;               — +1 forward, -1 reverse

float ground_y, base_hip_y;    — screen-derived

Vec2  foot_lock[2];            — last-captured FK landing position
bool  on_ground[2];            — flagged by leg_swing_pose / leg_stance_pose

Vec2  hip_c, hip_j[2];         — body hip centre + per-leg hip joints
Vec2  torso_top, head_c;
Vec2  shoulder[2], hand[2];    — single-segment arms (no elbow)
Vec2  knee[2], foot[2];        — computed each frame

bool  paused, step_once, show_grid;
```

Two key invariants that keep the gait coherent:
- `phase` is the only timekeeper. Everything else (each leg's `phi_leg`, the body bob/sway, the arm swing) is derived from it.
- `foot_lock[i]` is updated exactly once per stride per leg — at the swing→stance transition. While the leg is in stance, that value is read but never written.

### Per-leg phase
```
phi_leg = phase + (i==1 ? π : 0)        ← right leg leads left by half a cycle

sin(phi_leg) > 0  → SWING   (leg in the air, FK)
sin(phi_leg) ≤ 0  → STANCE  (leg planted, IK)
```

Touchdown happens when `sin(phi_old) > 0` and `sin(phi_new) ≤ 0` for the same leg — that's the instant `foot_lock[i]` is captured.

## Key Formulas

```
thigh_angle  = -SWING_AMP · cos(φ_leg)         (0 = vertical, +tilts forward)
knee_bend    =  LIFT_AMP  · sin(φ_leg)         (peaks at π/2, swing only)
shin_angle   =  thigh_angle - knee_bend        ← MINUS, anatomical fold
knee_pos     = hip + UPPER_LEG · (sin θt, cos θt)
foot_pos     = knee + LOWER_LEG · (sin θs, cos θs)

stance_knee  = solve_ik2(hip, foot_lock, U, L)

bob          = BOB_AMP  · sin(2φ)              ← 2 peaks per stride
sway         = SWAY_AMP · cos(φ)               ← 1 peak per stride

stride_len   = 2 · (UPPER + LOWER) · sin(SWING_AMP)
walk_speed   = walk_freq · stride_len          ← coupled, never free
```

The `(sin θ, cos θ)` form (rather than the more common `(cos θ, sin θ)`) is because θ is measured FROM VERTICAL not from horizontal — `θ=0` should mean "leg straight down," so x uses sin and y uses cos.

## The Main Loop (variable dt)

1. Resize check.
2. `dt` = wall-clock since last frame, capped at `DT_CAP_SEC`.
3. Drain input → `app_handle_key`.
4. `robot_tick(dt)`:
   - advance `phase` and `x`
   - per leg: detect touchdown, update `foot_lock` if so
   - `compute_pose`: body → arms → legs (FK or IK depending on `sin(phi_leg)`)
   - wrap `x` at screen edges
5. `scene_draw`: ground → spine → arms → legs → head → feet → HUD.
6. Frame cap.

## Non-Obvious Decisions

**Why `shin = thigh − knee_bend`, not `+ knee_bend`?**
With `+`, the shin always tilts MORE forward than the thigh. At swing mid-cycle when the thigh is vertical, the shin slopes forward, which means the foot leads the knee by ~30° forward. Through stance, the body never catches up to that forward foot, so BOTH legs visibly lean forward all the time — the "MJ Smooth Criminal" pose. The minus sign makes the heel tuck UP toward the butt at mid-swing (the anatomical fold direction), and the foot then meets the ground at exactly the angle the IK expects.

**Why couple `walk_speed` to `walk_freq`?**
Because they're two views of the same physical thing. Stride length is set by the leg geometry (`2·(U+L)·sin(SWING_AMP)`); stride period is `1/walk_freq`; so speed is forced to `walk_freq · stride_length`. Letting the user set them independently means almost every choice is wrong: too slow → forward lean (foot stays ahead), too fast → backward lean (body outruns foot). The setter `robot_set_pace(r, freq)` enforces the relationship.

**Why FK for swing but IK for stance?**
Each phase has different *known* quantities:
- Swing — we know the joint *angles* we want (the artistic shape of the step). Compute foot from angles → FK.
- Stance — we know the foot is FIXED at the touchdown spot. Compute knee from hip and foot → IK.

Mixing them gives natural-looking walking from minimal authoring: only `SWING_AMP`, `LIFT_AMP`, and `walk_freq` need tuning; everything else falls out.

**Why is `solve_ik2` shared with `animation/ik_helloworld.c`?**
Same 2-link analytical IK — the law of cosines plus a sign choice for which side of the hip-foot line the knee bulges on. Studying ik_helloworld first lets a reader see the math in isolation before encountering it inside the gait machinery.

**Why does the swing-leg foot get clamped to the ground (`foot.y > ground_y → ground_y`)?**
At extreme `LIFT_AMP` or with weird user inputs the FK formulas could produce a foot that punches *through* the ground briefly. Clamping is a one-line guard rather than redesigning the gait. The stance leg never has this problem because `foot_lock.y` is set to `ground_y` at capture time.

**Why arms are single-segment (no elbow)?**
This file is about leg gait. Arms exist for visual balance only — counter-phase swing of a hand is enough to look right; an elbow joint adds noise without teaching anything new.

## Key Constants

| Constant | Default | Effect |
|---|---|---|
| UPPER_LEG_LEN | 56 px | Hip → knee. Bigger = taller robot. |
| LOWER_LEG_LEN | 48 px | Knee → foot. Affects stride too (`U+L` in formula). |
| SWING_AMP | 0.35 rad (20°) | Peak thigh angle from vertical. Bigger = longer stride, more lean. |
| LIFT_AMP | 0.45 rad (26°) | Peak knee bend at mid-swing. Bigger = "marching" gait. |
| ARM_SWING | 0.30 rad | Peak hand swing. Counter-phase to same-side leg. |
| BOB_AMP | 3 px | Vertical hip bob (2 peaks per stride — the "pelvic bob"). |
| SWAY_AMP | 4 px | Lateral hip sway (1 peak per stride — keeps COM over support). |
| WALK_FREQ_DEFAULT | 1.6 Hz | Strides per second. The ONLY pace knob. |

## Open Questions

- Could the arms grow elbows by reusing `solve_ik2` with shoulder/hand as endpoints? Easy extension.
- Foot lock currently snaps without slip; modeling slight foot slide on slick surfaces could be a next-level demo.
- BOB and SWAY amplitudes don't currently respond to walk_freq — at very fast pace, the body bobs the same as at slow pace. Coupling them (`BOB_AMP · sqrt(walk_freq)`?) might look more natural.

---

# Pass 2 — Pseudocode

## Module Map

| § | Purpose |
|---|---|
| §1 config | frame rate, geometry, gait amplitudes, dynamics, ground grid, ncurses pairs, HUD layout |
| §2 clock | monotonic timer + sleep |
| §3 color | bone/arm/leg/HUD pairs |
| §4 coords | pixel ↔ cell conversion |
| §5 robot | (5.1) types (5.2) IK (5.3) swing FK (5.4) stance IK (5.5) arms (5.6) compute_pose (5.7) init/reset/set_pace (5.8) tick |
| §6 render | (6.1) helpers (6.2) draw_bone (6.3) ground (6.4) robot (6.5) HUD (6.6) scene_draw |
| §7 screen | ncurses init / present |
| §8 app | signals, resize, variable-dt main loop |

## Data Flow

```
keys → app_handle_key → robot.{paused, direction, walk_freq via set_pace, show_grid}
                            │
clock_ns → dt → robot_tick(r, dt)
                     ├── phase += 2π · walk_freq · dir · dt
                     ├── x     += walk_speed       · dir · dt
                     ├── per leg: detect touchdown → foot_lock[i] = FK landing pos
                     ├── compute_pose:
                     │     ├── body: bob, sway, hip_c, hip_j[]
                     │     ├── arms: shoulder + ARM_LEN · (sin, cos)
                     │     └── legs: SWING → leg_swing_pose (FK)
                     │              STANCE → leg_stance_pose (IK via solve_ik2)
                     └── wrap x at screen edges
                           │
                           ▼
                    scene_draw
                       ground → spine → arms → legs → head → feet → HUD
```

## Pseudocode

```
setup:
  install signals (SIGINT/SIGTERM/SIGWINCH)
  screen_init, color_init
  robot_init(cols, rows)         ← seeds foot_lock; calls robot_set_pace

while running:
  if need_resize: robot_init preserving freq/dir/grid
  dt = clamp(now - last, 0, DT_CAP_SEC)
  drain input → app_handle_key
  robot_tick(r, dt, cols, rows)
  fps update
  erase
  scene_draw
  wnoutrefresh + doupdate
  sleep to TARGET_FPS
```

## Key Patterns to Internalize

**Phase as the universal clock.** One float drives gait, body bob, body sway, arm swing — all derived. No FSM, no per-component timers.

**FK/IK split per leg.** Same robot, different math depending on whether the foot is free (FK) or pinned (IK). The boundary is `sin(phi_leg) = 0`.

**Foot-lock capture on transition.** Touchdown is where FK ends and IK begins; capture the FK foot position at that exact instant so the IK starts coherent.

**Couple parameters that aren't independent.** `walk_speed` is not free — it's tied to `walk_freq` by leg geometry. Hide that derivation behind a setter so the user never desyncs them.

**Sign of knee-bend matters.** The same magnitude with the wrong sign produces a famous wrong gait. Anatomically valid is `shin = thigh - knee_bend`.

**One conversion point.** Pixel→cell rounding lives in `scene_draw` and nowhere else; the rest of the simulation is pure pixel-space float.
