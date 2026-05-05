# Pass 1 — ik_scorpin: Scorpion = Spider Body + Curving FK Tail

## From the Source

**Algorithm 1 — Trail-buffer FK (body):** Identical to ik_spider.c. The head pushes its position into a circular trail buffer each tick; body joints are placed at arc-length offsets `i × BODY_SEG_LEN` backward along the trail. The body curves naturally wherever the head went — no per-joint angle formula required.

**Algorithm 2 — 2-Joint Analytical IK (legs):** Six legs in three angular pairs (front, mid, rear) at 60° spacing. Each leg's knee is solved by law of cosines: `cos(θ_hip) = (d² + U² − L²) / (2 · d · U)` with hip-to-target distance `d` clamped to `[|U − L| + 1, U + L − 1]`. Left and right legs use opposite signs of `θ_hip`.

**Algorithm 3 — Per-Leg Autonomous Gait:** Each leg watches its own foot's drift from the ideal step target and triggers a swing when the drift exceeds `STEP_TRIGGER_DIST` or the foot is over-stretched. A global `n_air < N_LEGS/2` cap ensures at least three feet are always planted. Swing animation uses a smoothstep ease over `STEP_DURATION = 0.22 s`.

**Algorithm 4 — Cumulative-Angle FK Tail (the signature feature):** A 7-segment chain anchored at `body_joint[N_BODY_SEGS]` (the abdomen). The tail's first angle is `heading + π` (rearward); each subsequent segment's angle adds `TAIL_BASE_CURL = 0.34 rad ≈ 19.5°` plus a small sin perturbation. Cumulative over 7 segments: `7 × 0.34 = 2.4 rad ≈ 136°` — the iconic scorpion arch lifting up and over the body. A travelling-wave perturbation (`TAIL_SWAY_AMP · sin(wave_time · TAIL_FREQ + i · TAIL_PHASE_PER_SEG)`) makes the stinger sway visibly as `wave_time` advances. No iteration, no contact constraints — pure stateless FK.

**Performance:** Variable timestep at render rate. Per frame: 1 trail push, 4 trail samples, 6 IK solves, 7 tail FK iterations. Microseconds total.

**Data-structure:** `Scorpion` extends the spider's state (trail buffer, body joints, per-leg state, heading + steering) with a `tail[N_TAIL_SEGS + 1]` joint array and a single `wave_time` accumulator that drives the tail oscillation.

## Core Idea

A scorpion is the spider plus a tail. Body, legs, and gait are inherited verbatim from `ik_spider.c`. The tail is a chain of 7 stiff segments hinged end-to-end, anchored at the abdomen. Each joint angle accumulates: previous angle + base curl + small sine perturbation. Because every segment curls the same direction at the same rate, the whole chain naturally arcs over the body's back — exactly the silhouette of a real scorpion. The sine perturbation rolls a wave from base to tip so the stinger breathes.

## The Mental Model

**Body** — same trail-buffer chain as the spider — curves through space as the head walks.

**Legs** — six 2-bar linkages, IK-solved, autonomous gait. Same as the spider.

**Tail** — one extra chain hinged at the abdomen. Imagine attaching a snake to the back end of a flat lizard, but with each joint curling the same direction at the same rate — the whole snake naturally arcs over the lizard's back. Now add a slow sine wave to the angles and the snake breathes.

The tail uses *cumulative angle* FK, the simplest possible chain model: the angle at segment `i+1` is the angle at segment `i` plus a per-segment delta. Make every delta point the same way and the chain rolls into a circular arc. This is forward kinematics in its purest form — no inverse problem, no constraints, no targets.

## Data Structures

### Scorpion (complete simulation state)

```
trail[TRAIL_CAP]            circular head-position history (1024 Vec2)
trail_head, trail_count     ring buffer state

body_joint[N_BODY_SEGS+1]   5 body joints (0=head, 4=abdomen)

hip[N_LEGS], knee[N_LEGS]   per-leg derived (recomputed each frame)
foot_pos[N_LEGS]            current planted/swinging foot positions
foot_old[N_LEGS]            foot position at swing start
step_target[N_LEGS]         destination for current swing
step_t[N_LEGS]              swing progress in [0, 1]
stepping[N_LEGS]            true if leg is in air

tail[N_TAIL_SEGS+1]         8 tail joints (0=base at abdomen, 7=stinger)

heading                     current body facing in radians
target_heading              user's desired facing
body_speed                  forward translation speed (px/s)

wave_time                   single accumulator driving tail oscillation

paused, theme_idx, time_scale
```

### Tail constants (the new piece)

```
N_TAIL_SEGS         7      8 joints, 7 segments
TAIL_SEG_LEN       11.0    px per segment → 77 px total
TAIL_BASE_CURL      0.34   rad/seg → 7 × 0.34 = 2.38 rad ≈ 136° total arch
TAIL_SWAY_AMP       0.06   rad — small perturbation per segment
TAIL_FREQ           1.0    rad/s → 6.3 s sway period
TAIL_PHASE_PER_SEG  0.7    rad — phase offset between adjacent segments
```

`TAIL_SWAY_AMP << TAIL_BASE_CURL` (0.06 vs 0.34) is the design constraint: any segment's net delta must stay positive so the chain never folds back on itself.

### Color pair layout

```
pair 1   body fill (gradient)
pair 2   body markers / nodes
pair 3   leg femur
pair 4   leg tibia
pair 5   planted foot '*'
pair 6   swinging foot 'o'
pair 7   tail (matches body or contrasts; theme-driven)
pair 8   PAIR_HUD  (status — bright yellow)
pair 9   PAIR_HINT (key hint — bright cyan)
```

## The Main Loop

Each iteration:

1. **Resize check.** SIGWINCH → reinitialise scene; clamp head into new bounds.
2. **Measure dt.** Wall clock; cap at 100 ms; multiply by `time_scale`.
3. **Advance wave_time** (only when not paused) — drives tail sin wave.
4. **Steer.** `heading` lerps toward `target_heading` at `TURN_RATE` rad/s, short-arc through ±π.
5. **Translate body.** Translate `body_joint[0]` along heading; toroidal wrap; push into trail.
6. **Compute body joints.** `body_joint[i] = trail_sample(i × BODY_SEG_LEN)` for i = 1..N_BODY_SEGS.
7. **Recompute hips.** Each hip = its body anchor point offset perpendicular to the local body forward by `hip_dist`.
8. **Gait + IK.** Update step triggers, advance any swinging feet, then solve law-of-cosines IK for every leg.
9. **Compute tail.** Starting at `tail[0] = body_joint[N_BODY_SEGS]`, angle `ang_0 = heading + π`. For i in 0..N_TAIL_SEGS−1:
   ```
   δ_i      = TAIL_BASE_CURL + TAIL_SWAY_AMP · sin(wave_time · TAIL_FREQ + i · TAIL_PHASE_PER_SEG)
   ang_i+1  = ang_i + δ_i
   tail[i+1] = tail[i] + TAIL_SEG_LEN · (cos ang_i+1, sin ang_i+1)
   ```
10. **Render.** Painter's order — legs (lines) → leg joints → body fill → body markers → tail (with `#` stinger at the tip) → head cluster (eyes + arrow). Tail draws AFTER the body so when the curve passes over the abdomen, the tail wins. Head draws last so it always overlays.
11. **Frame cap + present.**
12. **Drain input.** Arrow keys steer; `w/s` speed; `t` theme; `[/]` time scale.

## Key Formulas

```
Tail FK (cumulative angle):
  ang_0     = heading + π                     (rearward from abdomen)
  δ_i       = TAIL_BASE_CURL
              + TAIL_SWAY_AMP · sin(wave_time · TAIL_FREQ
                                    + i · TAIL_PHASE_PER_SEG)
  ang_i+1   = ang_i + δ_i
  tail_i+1  = tail_i + TAIL_SEG_LEN · (cos ang_i+1, sin ang_i+1)

Total curl: N_TAIL_SEGS · TAIL_BASE_CURL = 2.38 rad ≈ 136°
            (stinger ends pointing forward-up over the body)
```

All other formulas — `trail_sample`, hip placement, 2-joint IK, foot swing — are unchanged from `ik_spider.c`.

## Non-Obvious Decisions

**Why cumulative-angle FK rather than IK on the tail?** The tail has no target. It is purely decorative — a sweeping arc that signals "scorpion." Cumulative-angle FK is the simplest possible chain model: one accumulator, one formula per segment, no iteration, no constraints. IK would require a target (where? the air?) and would need contact handling (so the tail doesn't intersect the body). Pure FK with a fixed base curl is the right fit for ornamental geometry.

**Why anchor the tail at `body_joint[N_BODY_SEGS]` and not some computed position?** The abdomen is the last body joint. Because `body_joint[N_BODY_SEGS]` already comes from `trail_sample`, it tracks the body's curve naturally. Anchoring there means the tail base inherits all the body's motion for free — no separate computation, no synchronisation bugs.

**Why phase offset per segment?** Without `i · TAIL_PHASE_PER_SEG` every segment would oscillate in lockstep — the entire tail would just wiggle as one rigid shape. With the offset, adjacent segments are slightly out of phase, so a wave rolls along the tail from base to tip. The stinger arrives at its peak ~`(N_TAIL_SEGS · TAIL_PHASE_PER_SEG) / (2π) ≈ 0.78` cycles after the base — a visible travelling wave.

**Why `TAIL_SWAY_AMP << TAIL_BASE_CURL`?** Each segment's effective delta is `TAIL_BASE_CURL + TAIL_SWAY_AMP · sin(...)`. If the sin term ever drove the delta negative, that segment would fold backward and the chain would collapse onto itself for one frame. With `0.06 << 0.34` (5× margin) the delta is bounded in `[0.28, 0.40]` — always positive, always curling the same direction.

**Why use `'#'` for the stinger glyph?** Distinct from the body's `'O'`/`'o'`/`'.'` and the legs' `'+'`. The `'#'` reads as a heavier, angular shape — visually denser than the rest of the tail beads. The eye picks out the stinger immediately even at a glance.

**Why painter's order tail-after-body?** Where the tail's curve passes over the abdomen, you want the tail to occlude the body — not the other way round. The scorpion's tail arches *above* its back in real life; the visual reads correctly only with that ordering.

## How to Verify

- Default config: scorpion crawls rightward at 45 px/s. The tail visibly arches up and over the body, ending in a `#` stinger. The arch is consistent — same shape every frame.
- Watch the stinger over a few seconds: it sways back-and-forth on a ~6 s period. Adjacent tail segments are slightly out of phase (the wave travels from base to tip).
- Press arrow keys → heading interpolates; the tail's base rotates with the abdomen, but the *curl shape* stays consistent in body-local frame — the silhouette always looks like a scorpion regardless of facing.
- Press space → tail freezes mid-sway. Un-pause → motion resumes from the same phase.
- Crank `w` (speed up) → body moves faster; tail sway period stays at 6 s (because `TAIL_FREQ` is independent of `body_speed`); but the tail shape itself doesn't change — only its position.
- `[` slow-mo to 0.25× → sway period stretches to 25 s. `]` 4× → 1.6 s — much more frantic stinger.

## References

- Reynolds, "Steering Behaviors for Autonomous Characters", 1999 — heading-toward-target interpolation pattern.
- Wikipedia, "Inverse kinematics" — derivation of the 2-joint law-of-cosines solver used in `solve_ik()`.
- Wikipedia, "Scorpion" — anatomy reference for the curving cauda raised over the back.
- Glenn Fiedler, "Fix Your Timestep!" (gafferongames.com) — case for fixed-step in stiff sims; this demo doesn't qualify, hence variable timestep.
