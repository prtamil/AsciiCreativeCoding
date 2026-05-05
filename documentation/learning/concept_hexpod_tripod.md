# Pass 1 — hexpod_tripod: 6-Legged Walker with Alternating Tripod Gait + 2-Joint IK

## From the Source

**Algorithm 1 — Alternating Tripod Gait:** The six legs split into two interlocked tripods: A = {0 left-front, 3 right-mid, 4 left-rear} and B = {1 right-front, 2 left-mid, 5 right-rear}. At every instant exactly one tripod is PLANTED (anchored to the world, forming a stable support triangle under the body) while the other is SWINGING (lifting off, arcing forward, landing at a new target). Phase swap fires when `phase_timer ≥ PHASE_DURATION` AND every swinging foot has finished its arc — never mid-stride. This guarantees the support tripod is always complete and the body cannot tip.

**Algorithm 2 — 2-Joint Analytical IK:** For each leg, the femur U and tibia L plus the hip → foot vector form a triangle. The knee angle is closed-form via the law of cosines: `cos(ah) = (dist² + U² − L²) / (2 · dist · U)`. Left and right legs use opposite signs of `ah` so left knees break toward −y and right knees toward +y. The hip-to-target distance is clamped into `[|U − L| + 1, U + L − 1]` so `acosf` never sees a value outside `[−1, 1]`.

**Algorithm 3 — Heading Interpolation:** `heading` interpolates toward `target_heading` (set by arrow keys) at `TURN_RATE = 2.5 rad/s`, with the angular delta wrapped into `(−π, π]` so a 180° flip always takes the short arc. A 90° turn takes ≈ 0.6 s.

**Performance:** Variable timestep at render rate — no fixed-step accumulator, no alpha lerp, no prev/cur snapshots. Per frame: 6 IK solves (each ≈ 2 sqrtf + 1 atan2f + 1 acosf), one gait state-machine step, body integration, ~30 cell stamps. Microseconds total.

**Data-structure:** `Hexapod` owns body kinematics (`body_x`, `body_y`, `heading`, `target_heading`, `body_speed`), per-leg state (`foot_pos`, `foot_old`, `step_target`, `stepping` flag, `step_t` swing progress 0→1), per-leg derived values (`hip`, `knee`, recomputed each frame), and the gait state machine (`gait_phase`, `phase_timer`). Static tables `HIP_LOCAL_X/Y[6]`, `REST_X/Y[6]`, `TRIPOD_A/B[3]` encode the immutable per-leg geometry.

## Core Idea

A rigid body slides forward along its heading; six legs hang off it, three planted and three swinging at any moment. Every tick: re-derive hips from the body's pose, advance any swinging feet along their parabolic arcs, then solve law-of-cosines IK to place every knee. The support never drops below three planted feet, so the body cannot tip. Arrow keys steer the heading; the legs follow automatically.

## The Mental Model

**Body** — a brick on rails. You tell it which way to face; it slides forward at `body_speed`. Gravity is irrelevant because the legs aren't bearing weight; they're a visual.

**Legs** — six 2-bar linkages (femur + tibia). Given the hip's world position and the foot's target, the knee falls out of elementary trigonometry. No state stored on the leg itself beyond "is it stepping right now, and how far through the step?"

**Gait** — a metronome with two phases. Phase A: legs A swing, legs B plant. Phase B: legs B swing, legs A plant. Phase swap happens after `PHASE_DURATION` elapses AND all swinging feet have landed — whichever is later. This guarantees the support tripod is always complete.

The biological reference is direct: real insects use exactly this gait, called the *alternating tripod*. The triangle of three planted feet always contains the body's centre of mass, so the animal is statically stable even mid-stride.

## Data Structures

### Hexapod (complete simulation state)

```
body_x, body_y         body centre, pixel space
heading                current facing in radians
target_heading         user's desired facing (arrow keys)
body_speed             forward translation speed (px/s)

foot_pos[6]            current world-space foot positions (Vec2)
foot_old[6]            foot position at start of current swing
step_target[6]         destination for current swing
step_t[6]              swing progress in [0, 1]
stepping[6]            bool — true if foot is currently in the air

hip[6]                 derived each frame from body pose + HIP_LOCAL_*
knee[6]                derived each frame from IK solve

gait_phase             0 → tripod A swinging, 1 → tripod B swinging
phase_timer            seconds since last phase swap

paused, theme_idx, time_scale
```

### Static per-leg tables

```
HIP_LOCAL_X[6]    body-local hip X offset (front: +ve, rear: -ve)
HIP_LOCAL_Y[6]    body-local hip Y offset (left: +BODY_HALF_W, right: -)
REST_X[6]         body-local foot rest X offset (lookahead axis)
REST_Y[6]         body-local foot rest Y offset (out from body)
TRIPOD_A[3]       leg indices in tripod A: {0, 3, 4}
TRIPOD_B[3]       leg indices in tripod B: {1, 2, 5}
```

### Color pair layout

```
pair 1   body frame (rectangle, braces, hips '+', centre '@')
pair 2   femur (hip → knee)
pair 3   tibia (knee → foot)
pair 4   planted foot '*' (bright, A_BOLD)
pair 5   swinging foot 'o' (dim, in flight)
pair 6   knee joint 'o' (A_BOLD)
pair 7   reserved
pair 8   PAIR_HUD  (status bar — bright yellow, A_BOLD)
pair 9   PAIR_HINT (key hint — bright cyan, A_BOLD)
```

## The Main Loop

Each iteration:

1. **Resize check.** SIGWINCH → reinitialise scene with new terminal dimensions. Clamp body within new bounds.

2. **Measure dt.** `CLOCK_MONOTONIC` since previous frame; capped at 100 ms (suspend guard). Multiplied by `time_scale` (0.25× ↔ 4×) for the slow-mo / fast-forward keys.

3. **Steer.** `heading` lerps toward `target_heading` at `TURN_RATE` rad/s, with the delta wrapped to `(−π, π]` for short-arc.

4. **Translate body.** `body_x += cos(heading) · body_speed · dt; body_y += sin(heading) · body_speed · dt`. Toroidal wrap at edges.

5. **Recompute hips.** Each hip is the body centre plus `HIP_LOCAL_*[i]` rotated by the heading.

6. **Stretch-snap.** If any foot is now beyond IK reach (e.g. body just wrapped around the screen edge), snap it to its rest target.

7. **Gait tick.**
   a. For each swinging leg, advance `step_t`; place the foot at `lerp(foot_old, step_target, smoothstep(step_t))` plus a parabolic Y-arc `−STEP_HEIGHT · sin(π · step_t)`.
   b. If `phase_timer ≥ PHASE_DURATION` AND every swinging foot has `step_t ≥ 1`, swap tripods. The just-landed tripod plants; the other launches with new `step_target`s.

8. **Solve IK.** For every leg: 2-joint law-of-cosines → knee position.

9. **Render.** Painter's order: leg lines (femur pair 2, tibia pair 3) → foot markers ('*' planted bold, 'o' swinging dim) → knee markers → body rectangle (4 edges + 2 cross-braces) → hip attachment '+' markers → body centre '@'.

10. **Frame cap + present.** Sleep before write; `wnoutrefresh` + `doupdate`.

11. **Drain input.** Arrow keys → `target_heading`; w/s → `body_speed`; t → theme; [/] → `time_scale`; q/ESC → quit.

## Key Formulas

```
Heading lerp    : delta = wrap_pi(target_heading − heading)
                  heading += clamp(delta, ±TURN_RATE · dt)

2-joint IK      : dist  = clamp(|T − H|, |U − L| + 1, U + L − 1)
                  base  = atan2(Ty − Hy, Tx − Hx)
                  cos_h = (dist² + U² − L²) / (2 · dist · U)
                  ah    = acos(clamp(cos_h, −1, 1))
                  knee_angle = base ± ah   ( − for left, + for right )
                  knee  = H + U · (cos knee_angle, sin knee_angle)

Foot swing      : ease  = smoothstep(step_t)
                  hz    = lerp(foot_old, step_target, ease)
                  arc_y = −STEP_HEIGHT · sin(π · step_t)
                  foot  = (hz.x, hz.y + arc_y)

Step target     : T = hip_world + rotate(REST_offset
                                         + (body_speed · LOOKAHEAD, 0),
                                         heading)
```

`smoothstep(t)` is the cubic `3t² − 2t³` — decelerating into landing — applied to the lerp only, while `sin(π · t)` on the arc keeps the peak at `t = 0.5` symmetric.

## Non-Obvious Decisions

**Why an alternating tripod, not a wave gait or random?** A tripod is the smallest gait that keeps the centre of mass over the support polygon at every instant. With three feet planted forming a triangle, you are guaranteed the body never tips. Wave gaits (one leg swinging at a time) are slower and produce visible head-bob; tripod gives crisp insect-like motion. Random ordering breaks the support guarantee.

**Why phase swap requires both timer expiry AND all feet landed?** If the metronome alone fired the swap, a slow swing (large `STEP_DURATION` or low `time_scale`) could launch the next tripod before the current one had landed — instant tip. The conjunction is the safety interlock.

**Why analytical IK rather than FABRIK?** Each leg has exactly 2 joints. The law-of-cosines is closed-form, exact, terminates in constant time, and is simpler to read geometrically than an iterative solver. FABRIK would be the right choice for 3+ joints; here it would be over-engineering.

**Why snap stretched feet to rest instead of clamping in IK?** When the body wraps toroidally, planted feet remain at the OLD coordinates and become unreachable. The IK clamp keeps `acosf` valid but the chain still looks broken. Snapping the foot to its rest position triggers a fresh swing on the next phase, recovering the gait visibly within one tripod cycle.

**Why variable timestep with no alpha lerp?** The simulation is non-stiff — no springs, no constraints, no fast oscillators. Variable timestep with a 100 ms cap is stable for any reasonable frame rate, and skipping the alpha-lerp infrastructure trims the code by ~50 lines without visible motion artefact.

**Why is `STEP_LOOKAHEAD` multiplied by `body_speed`?** A foot landing at a fixed body-local offset would be left behind as the body accelerates. Multiplying by speed makes the foot land *where the hip will be* `LOOKAHEAD` seconds in the future — strides automatically lengthen at higher speeds, exactly as in real insect gaits.

## How to Verify

- Default config: robot walks rightward at 40 px/s. At any frozen frame, exactly 3 feet show `*` (planted) and 3 show `o` (swinging). The 3 planted feet form a triangle: a left/right pair at front-or-rear plus the opposite-side mid leg.
- Press arrow keys → heading interpolates; a 90° turn takes ~0.6 s. Body visibly spins while the gait keeps stepping.
- Press space → gait, body, knees freeze. Un-pause → motion resumes exactly where it was.
- Crank `w` (speed up) → step lookahead grows; feet land further ahead of the hips. Gait keeps up because targets are recomputed every phase swap.
- `[` slow-mo → motion stays smooth (variable timestep handles this); `]` fast-forward up to 4×.

## References

- Wikipedia, "Tripod gait" — biological reference for the alternating three-leg pattern used by insects and 6-legged robots.
- Wikipedia, "Inverse kinematics" — derivation of the 2-joint law-of-cosines solver.
- Hirose, *Biologically Inspired Robots: Snake-Like Locomotors and Manipulators*, MIT Press 1993 — foundational hexapod gait references.
- Reynolds, "Steering Behaviors for Autonomous Characters", 1999 — heading-toward-target interpolation pattern used here.
