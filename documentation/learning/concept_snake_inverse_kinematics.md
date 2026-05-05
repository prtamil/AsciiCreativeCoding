# Pass 1 — snake_inverse_kinematics: IK goal-seeking head with multi-harmonic wandering target

## From the Source

**Algorithm:** IK goal-seeking head + path-following FK body, with a wall-bouncing wander target. Head steers toward `actual_target` (a lerp-smoothed wander position) at `move_speed` px/s. Body joints are placed by arc-length sampling of the head's recorded trail — no per-joint angle formula required. The wander target steers itself via a superposition of three incommensurable sine waves, producing terrain-like paths that never exactly repeat. When the wander target enters an `EDGE_MARGIN_PX = 64` band along any screen edge, its `tgt_dir` reflects through the edge normal (billiard-ball bounce) and `tgt_pos` is clamped — the chasing head naturally follows the bouncing target back inside the screen. There is no toroidal wrap; the snake stays on-screen forever.

**Math:** Multi-harmonic wander: `turn = A1*sin(f1*t) + A2*sin(f2*t+φ2) + A3*sin(f3*t+φ3)`. Target direction integrates: `tgt_dir += turn * dt`. Target position: `tgt_pos += tgt_speed * (cos(tgt_dir), sin(tgt_dir)) * dt`. Edge bounce: at left/right wall `tgt_dir → π − tgt_dir` (flip cos); at top/bottom wall `tgt_dir → −tgt_dir` (flip sin); `tgt_pos` hard-clamped to `[margin, wpx − margin] × [margin, hpx − margin]`. Smooth target: `actual_target += (tgt_pos - actual_target) * min(dt * TGT_SMOOTH_RATE, 1)`. Head steering: `heading = atan2(actual_target.y - head.y, actual_target.x - head.x)`. Movement: `head += (dx/dist) * min(move_speed*dt, dist)` (no overshoot). Hard clamp on `joint[0]` at the screen box is the high-speed safety net.

**Performance:** Same as FK snake. `trail_sample()` cost: O(dist / px_per_tick) per joint × 32 joints. One extra circular buffer for the ghost trail: `Vec2 tgt_trail[200]` = 1600 B. 10-theme colour system.

**Data-structure:** Same trail buffer + joint array as FK snake. Added IK fields: `actual_target`, `tgt_pos`, `tgt_time`, `tgt_speed`, `tgt_dir`, `heading`. Added ghost trail: `tgt_trail[200]`, `tgt_head`, `tgt_count`. All positions in pixel space.

## Core Idea

A 32-segment snake chases an organically wandering target. The "IK" is entirely in the head: instead of computing its own sinusoidal heading (as in the FK snake), the head uses `atan2` to look directly at the target and moves toward it at constant speed. The body follows via the same trail-buffer FK as the FK snake — the IK label applies only to how the head heading is determined.

The target is interesting: it steers itself using three sine waves at mutually irrational frequencies (0.29, 0.71, 1.13 rad/s). Since these frequencies are incommensurable (no rational ratio between any two), the combined turn-rate waveform never exactly repeats. The target carves terrain-like paths — wide sweeping hills (slow harmonic), medium wiggles (mid harmonic), and fine tremors (fast harmonic) — similar to river meanders or mountain ridge lines.

A second circular buffer (`tgt_trail[200]`) records the last 200 positions of `actual_target`. These are rendered as dim `'.'` dots trailing behind the target cursor, revealing the path the snake is chasing and making the terrain-like quality of the wander visible.

## The Mental Model

The FK snake navigates by a fixed compass bearing that swings autonomously. The IK snake navigates by always facing a moving landmark (the target). The two snakes look similar but differ in what drives the head.

The target behaves like a small autonomous boat following its own curved course. The snake is a larger boat that always steers directly toward the small boat. Because the target moves faster than the snake is wide, the snake traces the target's path with a time lag — the body curves the same way the target curved, but after a delay.

The three-frequency steering of the target is a way to make an irregular path without randomness. Random paths have sudden jumps; a sum of three smooth sinusoids is always smooth (C-infinity continuous). The irrational frequency ratios mean the pattern effectively never repeats — it is periodic on paper but the period is so long that for practical purposes the path looks random.

The `actual_target` smoothing (`lerp toward tgt_pos at 8×/s`) is a low-pass filter. Right after a wall bounce, `tgt_dir` flips and the new target velocity points back inside; `actual_target` lags the change by ~one tick, so the head's chase rounds the corner gradually rather than snapping. The result reads as the snake "noticing" the wall and curving away — exactly the behaviour the FK snake produces with edge-bias steering, but achieved indirectly by chasing a target that bounces.

## Data Structures

### Vec2
```
x, y   — position in pixel space (float, square isotropic grid)
```

### Snake
```
/* Trail buffer (same as FK snake) */
trail[TRAIL_CAP=4096]  — circular head position history
trail_head             — write pointer
trail_count            — valid entries, ≤ 4096

/* Body joints */
joint[N_SEGS+1=33]     — [0]=head ... [32]=tail tip
prev_joint[33]         — snapshot at tick start for alpha lerp

/* IK / wander target */
actual_target   — smoothed position (what head actually steers toward)
tgt_pos         — raw wander target position
tgt_time        — simulation time for the harmonic formulae
tgt_speed       — wander target translation speed (px/s)
tgt_dir         — heading of the wander target (radians, integrated)
heading         — head's current travel direction (for head arrow glyph)
move_speed      — head translation speed (px/s)

/* Ghost trail */
tgt_trail[200]  — circular buffer of recent actual_target positions
tgt_head        — write pointer
tgt_count       — valid entries, ≤ 200

theme_idx       — index into THEMES[]; t/T cycle it
paused          — physics freeze
```

## The Main Loop

Each iteration:

1. **Check resize.** Re-read terminal dimensions. Clamp `joint[0]` to new pixel bounds. No full scene reset (snake continues from its current position, same as FK snake on resize).

2. **Measure dt.** Monotonic clock, capped at 100 ms.

3. **Physics accumulator.** While `sim_accum >= tick_ns`: `scene_tick(dt_sec)` and drain.

4. **Compute alpha.** `sim_accum / tick_ns ∈ [0, 1)`.

5. **FPS counter.** 500 ms window.

6. **Sleep for frame cap.** Before render.

7. **Draw and present.** `erase()` → `render_chain(alpha)` (ghost trail dots → target cursor → bead fill → node markers → head arrow) → HUD → `wnoutrefresh + doupdate`.

8. **Handle input.** `getch()` loop. `UP/w` and `DOWN/s` scale `move_speed`. `+/-` scale `tgt_speed`. `t/T` cycle themes.

## Non-Obvious Decisions

**Why atan2 for IK head steering instead of integrating a turn rate?**
The FK snake computes its heading via integration of a turn-rate formula. The IK snake computes its heading via `atan2(target.y - head.y, target.x - head.x)`. The latter is "goal-directed" — the head always faces the target regardless of its current heading. This is the defining difference between IK (goal-driven) and FK (formula-driven). The trade-off: IK heading can change discontinuously if the target jumps; FK heading changes smoothly but cannot track an arbitrary goal.

**Why bounce the target off walls instead of wrapping toroidally?**
The earlier version of this demo wrapped `tgt_pos` toroidally; when the target crossed an edge, it reappeared on the opposite side and the snake had to traverse the full screen to catch up — visually it looked like the snake gave up its current chase mid-stride. With edge bounce, the target turns inward and the snake follows the corner; the chase is continuous from frame one. The bounce is implemented as a billiard reflection of `tgt_dir` through the edge normal: at left/right walls `tgt_dir → π − tgt_dir` (flips the cos component); at top/bottom walls `tgt_dir → −tgt_dir` (flips the sin component). The `tgt_pos` is also hard-clamped to inside the bounded region — without that, a single tick at high `tgt_speed` could push the target several pixels past the wall, and the next frame would reflect again, oscillating in place.

**Why bounce off an inner wall (`EDGE_MARGIN_PX = 64`) instead of the screen edge itself?**
Setting the bounce wall in by 64 px (≈ 8 columns / 4 rows) keeps the target visible at the corner — the bright `+` cursor and the snake's head both stay fully in view rather than half-disappearing under the HUD bars or against the very edge. The margin is purely cosmetic; physics-wise, bouncing at the edge would also work.

**Why three sine harmonics instead of one?**
One sine wave produces a perfectly regular sinusoidal path — the target oscillates between two fixed lateral extremes with a fixed period. This looks like a pendulum, not terrain. Two or three harmonics with different frequencies produce a path that looks more complex and organic. The specific choice of three frequencies (0.29, 0.71, 1.13 rad/s) with amplitudes (1.40, 0.80, 0.40) gives: wide sweeping hills from the slow harmonic, medium wiggles overlaid, and fine tremors from the fast harmonic. The sizes decrease with frequency, mirroring how real terrain has large mountains, smaller hills, and fine texture.

**Why incommensurable frequencies?**
Two frequencies f1 and f2 are commensurable if `f1/f2` is rational — in that case the combined waveform has period `LCM(2π/f1, 2π/f2)`. For `0.29` and `0.71`: their ratio is `0.29/0.71 ≈ 0.4084...`, which is irrational (cannot be expressed as p/q for integers p, q). The waveform `A1*sin(f1*t) + A2*sin(f2*t)` with irrational ratio is quasiperiodic — it never exactly repeats. Adding a third incommensurable frequency (`0.29/1.13 ≈ 0.257...`) reinforces the non-repetition further. The target path is practically aperiodic.

**Why use `actual_target` as a lerp-smoothed version of `tgt_pos` instead of steering toward `tgt_pos` directly?**
At a wall bounce, `tgt_dir` flips through the edge normal and the per-tick `tgt_pos` step suddenly reverses direction. If the head steered toward the raw `tgt_pos`, its heading would lurch the moment the bounce fired. `actual_target` lerps toward `tgt_pos` at `8×/s`: `k = dt * 8; actual_target += (tgt_pos - actual_target) * k`. At 60 Hz, `k ≈ 0.133` per tick — a smooth chase, not a snap. After a bounce, `actual_target` keeps moving toward the OLD `tgt_pos` for ~one tick before rounding into the new direction; the head's path traces a smooth corner rather than a kink. The same mechanism also masks the small position jumps from the bounce-clamp — even if `tgt_pos` is snapped a few pixels back inside the wall, the lerp absorbs the jump invisibly.

**Why clamp `step = min(move_speed * dt, dist)` when moving the head?**
Without the clamp, if `dist < move_speed * dt` (head is very close to target), the head would overshoot the target and oscillate around it. The clamp ensures the head always moves at most `dist` toward the target — it arrives at the target smoothly and stops (until the target moves further away). Without this, the head would jitter back and forth around the target when it gets close.

**Why record `actual_target` (not `tgt_pos`) in the ghost trail?**
`tgt_pos` is the raw bouncing position — at a wall bounce the per-tick step direction reverses sharply, and the bounce-clamp can also move `tgt_pos` by a few pixels in one frame. `actual_target` is the lerp-smoothed version, so its path is always C¹-continuous. Recording `actual_target` keeps the ghost trail as a smooth dotted line that traces the terrain the snake has been chasing — including the smoothly-rounded corners where the target bounced.

**Why draw ghost trail dots oldest-first (k = n-1 down to 1)?**
The older dots are drawn first so newer dots overwrite them at overlapping positions. The newest dots (most recent positions) should be most visible — they represent where the target is now. The oldest dots are background noise. Drawing oldest-first gives the newest dots priority.

**Why is `DRAW_STEP_PX = 5.0` for the IK snake instead of `3.0` as in the FK snake?**
Cosmetic choice: the IK snake uses a slightly sparser bead fill. At `5.0 px` step with `SEG_LEN_PX = 18 px`, a straight segment gets `ceil(18/5)+1 = 5` samples — enough to avoid visual gaps (a 18px segment spans at most `18/CELL_W = 2.25` cells horizontally, well within 5 samples' coverage). The sparser fill makes the IK snake body look slightly more wiry and distinct from the FK snake.

**Why start `tgt_dir = π/6` (30°) in `scene_init()`?**
A slight south-east heading for the target means it moves away from the head's initial position immediately. If both `tgt_dir = 0` (east) and the head were at the screen centre, the target would move directly away from the head's initial chase direction, producing a clean divergence-and-chase from frame one. `π/6 ≈ 30°` is a slight diagonal that also avoids the target immediately hitting a wall.

## State Machines

### move_head() — IK step-by-step
```
Step 1: advance wander target
  tgt_time += dt
  turn = TGT_TURN_AMP1 * sin(TGT_TURN_FREQ1 * tgt_time)
       + TGT_TURN_AMP2 * sin(TGT_TURN_FREQ2 * tgt_time + 1.10)
       + TGT_TURN_AMP3 * sin(TGT_TURN_FREQ3 * tgt_time + 2.40)
  tgt_dir += turn * dt
  tgt_pos += tgt_speed * (cos(tgt_dir), sin(tgt_dir)) * dt

Step 2: bounce target off bounded region
  bounded region = [margin, wpx-margin] × [margin, hpx-margin]
  if tgt_pos.x crossed left or right wall:
    clamp tgt_pos.x; tgt_dir ← π − tgt_dir   (flip cos)
  if tgt_pos.y crossed top or bottom wall:
    clamp tgt_pos.y; tgt_dir ← −tgt_dir       (flip sin)

Step 3: smooth actual_target
  k = min(dt * 8.0, 1.0)
  actual_target += (tgt_pos - actual_target) * k

Step 4: steer head toward actual_target
  dx, dy = actual_target - joint[0]
  dist = sqrt(dx² + dy²)
  if dist > 0.5:
    heading = atan2(dy, dx)
    step = min(move_speed * dt, dist)   ← no overshoot
    joint[0] += (dx/dist, dy/dist) * step

Step 5: hard clamp joint[0] into [0,wpx] × [0,hpx]   ← safety net

Step 6: trail_push(joint[0])
        tgt_push(actual_target)        ← record to ghost trail
```

### render_chain() — five-pass composition
```
Step 1: alpha lerp all 33 joints
  rj[i] = prev_joint[i] + (joint[i] - prev_joint[i]) * alpha

Step 2: ghost trail (oldest → newest)
  for k = tgt_count-1 down to 1:
    draw '.' with HUD_PAIR | A_DIM at tgt_trail[k]
    → fading dots show the path the head has been chasing

Step 3: target cursor
  draw '+' with HUD_PAIR | A_BOLD at actual_target
  → bright cross marks the current target

Step 4: bead fill (tail → head, pass 1)
  draw_segment_beads(rj[i+1], rj[i]) for i=31 down to 0
  → 'o' fill at 5px steps

Step 5: joint node markers + head arrow (pass 2)
  joint_node_char(i) = '0' | 'o' | '.'
  for i=32 down to 1; then head arrow at rj[0] last
```

### Multi-harmonic wander frequencies
| Harmonic | Amplitude | Frequency | Period | Role |
|----------|-----------|-----------|--------|------|
| 1 | 1.40 rad/s | 0.29 rad/s | ~21.7 s | Wide sweeping hills |
| 2 | 0.80 rad/s | 0.71 rad/s | ~8.9 s | Medium wiggles |
| 3 | 0.40 rad/s | 1.13 rad/s | ~5.6 s | Fine tremors |

All three frequencies are mutually irrational — no pair has a rational ratio. The path is quasiperiodic (effectively non-repeating).

### Ghost trail circular buffer
```
tgt_push(actual_target):
  tgt_head = (tgt_head + 1) % 200
  tgt_trail[tgt_head] = actual_target
  tgt_count = min(tgt_count + 1, 200)

Oldest-first render:
  for k = tgt_count-1 down to 1:
    idx = (tgt_head + 200 - k) % 200   ← same circular index trick as trail_at
```

### Input actions
| Key | Effect |
|-----|--------|
| q / Q / ESC | exit |
| space | toggle pause |
| UP / w / W | move_speed × 1.20 (max 600) |
| DOWN / s / S | move_speed ÷ 1.20 (min 20) |
| + / = | tgt_speed × 1.25 (max 500) |
| - | tgt_speed ÷ 1.25 (min 5) |
| t | next theme (10 total, wrapping) |
| T | previous theme |
| r / R | full reset (theme preserved) |
| ] | sim_fps + 10 |
| [ | sim_fps − 10 |

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|----------|---------|-------------------|
| MOVE_SPEED_DEFAULT | 150 px/s | Increase → snake overtakes target easily, target leads less. Decrease → target is always ahead, snake never catches up — perpetual chase. |
| TGT_WANDER_SPEED_DEFAULT | 80 px/s | Increase → target moves faster, harder to catch. Decrease → target nearly stationary; snake oscillates around it. |
| TGT_TURN_AMP1 | 1.40 rad/s | Increase → wider sweeping hills. Decrease → flatter, more linear path. |
| TGT_TURN_FREQ1 | 0.29 rad/s | Decrease → very long gentle bends. Increase toward TGT_TURN_FREQ2 → two harmonics compete for similar periods, complex interference. |
| TGT_SMOOTH_RATE | 8.0 /s | Decrease → actual_target lags further behind tgt_pos, snake steers more smoothly but reacts more slowly to direction changes. Increase toward ∞ → actual_target = tgt_pos (no smoothing). |
| TARGET_TRAIL_CAP | 200 entries | More entries → longer ghost trail visible on screen. Fewer → shorter trail, target path less legible. |
| DRAW_STEP_PX | 5.0 px | Must be < CELL_W (8). Increase → sparser beads, wiry look. Decrease → denser, more solid body. |
| SEG_LEN_PX | 18.0 px | Same as FK snake — see FK doc. |

## Open Questions for Pass 3

- When `move_speed > tgt_speed`, the head can overtake the target and oscillate around it. The `step = min(move_speed*dt, dist)` clamp prevents overshoot on any single tick, but over many ticks can the head still oscillate? What damping (if any) prevents this?
- The ghost trail renders 200 entries as identical `'.'` dots with the same `A_DIM` attribute. There is no visual gradient showing which dots are newer. How would you implement a fade from bright (new) to invisible (old) using only the available ncurses attributes (`A_BOLD`, `A_NORMAL`, `A_DIM`)?
- The wander target's `tgt_dir` integrates `turn * dt` without any normalisation or damping. Over a long run, `tgt_dir` can grow to very large values. Does this cause floating-point precision issues with `cosf(tgt_dir)` or `sinf(tgt_dir)` for very large arguments? At what `tgt_time` does this become a concern?
- `actual_target` lerps toward `tgt_pos` at `dt * 8.0`. At `sim_fps = 10 Hz`, `dt_sec = 0.1`, so `k = 0.8` per tick — 80% of the gap is closed each tick. At `sim_fps = 120 Hz`, `dt_sec = 1/120 ≈ 0.0083`, `k = 0.067` — only 6.7% per tick. Does this mean the smoothing is much less effective at higher sim Hz? What would be the correct framerate-independent lerp?
- The target trail buffer records `actual_target`, not `tgt_pos`. After a bounce, `tgt_dir` flips and `tgt_pos` is clamped — `actual_target` lags by one tick before turning. Does the ghost trail show a visible "elbow" at the bounce corner, or is the lerp tight enough that the corner reads smooth? Try slowing time scale (`[`) to inspect frame by frame.
- IK and FK produce different body curves for the same head path. The FK snake's body always reflects the exact mathematical path carved by the sinusoidal heading. The IK snake's body reflects the head's actual chase path, which changes with `move_speed` and `tgt_speed`. If you set `tgt_speed = 0` (stationary target) and `move_speed` very high, would the IK snake body converge to a straight line (all joints approaching the same point)?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app` | `App` | ~68 KB | top-level container |
| `g_app.scene.snake.trail[4096]` | `Vec2[4096]` | 32 KB | circular head position history |
| `g_app.scene.snake.tgt_trail[200]` | `Vec2[200]` | 1600 B | ghost trail of actual_target positions |
| `g_app.scene.snake.joint[33]` | `Vec2[33]` | 264 B | current body joint positions |
| `g_app.scene.snake.prev_joint[33]` | `Vec2[33]` | 264 B | tick-start snapshot for alpha lerp |
| `THEMES[10]` | `Theme[10]` (global const) | ~400 B | 10 named colour palettes |

---

# Pass 2 — snake_inverse_kinematics: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | N_SEGS, TRAIL_CAP, TARGET_TRAIL_CAP, SEG_LEN_PX, DRAW_STEP_PX, wander harmonic constants |
| §2 clock | `clock_ns()` / `clock_sleep_ns()` |
| §3 color | `Theme` struct, `THEMES[10]`, `theme_apply()`, `color_init()` — default theme is "Medusa" |
| §4 coords | `px_to_cell_x/y` — pixel to cell, only at draw time |
| §5a trail helpers | `trail_push`, `trail_at`, `trail_sample`, `tgt_push` — circular buffer operations |
| §5b move_head | Multi-harmonic wander → bounce target → actual_target lerp → atan2 steer → step → clamp → push |
| §5c compute_joints | Arc-length sampling of head trail, identical to FK snake |
| §5d bead rendering helpers | `seg_pair`, `seg_attr`, `joint_node_char`, `head_glyph` — same as FK snake |
| §5e draw_segment_beads | Dense 'o' fill at DRAW_STEP_PX=5.0 (sparser than FK's 3.0) |
| §5f render_chain | Alpha lerp → ghost trail → target cursor → bead fill → node markers → head arrow |
| §6 scene | `scene_init` (trail pre-pop + target at head) / `scene_tick` / `scene_draw` |
| §7 screen | ncurses layer (erase → draw → wnoutrefresh → doupdate) |
| §8 app | Signals, resize (clamp joint[0], no full reset), main loop |

---

## Data Flow Diagram

```
CLOCK_MONOTONIC
    │
    ▼
sim_accum += dt
    │
    while sim_accum >= tick_ns:
    │
    ▼
scene_tick(dt_sec, cols, rows)
    │
    ├── memcpy(prev_joint ← joint)         [snapshot before physics]
    │
    ├── move_head(dt, cols, rows):
    │
    │   WANDER TARGET (tgt_pos):
    │     tgt_time += dt
    │     turn = 1.40*sin(0.29*t) + 0.80*sin(0.71*t+1.10) + 0.40*sin(1.13*t+2.40)
    │     tgt_dir += turn * dt             [integrate curvature → heading]
    │     tgt_pos += tgt_speed*(cos,sin)(tgt_dir)*dt
    │     bounce_target():                  [reflect tgt_dir + clamp tgt_pos]
    │       L/R wall: tgt_dir ← π − tgt_dir (flip cos)
    │       T/B wall: tgt_dir ← −tgt_dir    (flip sin)
    │
    │   SMOOTH TARGET (actual_target):
    │     k = min(dt * 8.0, 1.0)
    │     actual_target += (tgt_pos - actual_target) * k   [low-pass filter]
    │
    │   IK HEAD STEERING:
    │     heading = atan2(actual_target.y - joint[0].y,
    │                     actual_target.x - joint[0].x)
    │     step = min(move_speed * dt, dist)   [no overshoot]
    │     joint[0] += (dx/dist, dy/dist) * step
    │     hard clamp joint[0] into [0,wpx] × [0,hpx]    [safety net]
    │
    │   RECORD:
    │     trail_push(joint[0])
    │     tgt_push(actual_target)
    │
    └── compute_joints():
          joint[i] = trail_sample(i * SEG_LEN_PX) for i=1..32

sim_accum -= tick_ns
    │
    ▼
alpha = sim_accum / tick_ns
    │
    ▼
render_chain(alpha)
    │
    ├── rj[i] = prev_joint[i] + (joint[i]-prev_joint[i])*alpha  [all 33]
    │
    ├── ghost trail: '.' HUD_PAIR A_DIM at tgt_trail[oldest → newest]
    │
    ├── target cursor: '+' HUD_PAIR A_BOLD at actual_target
    │
    ├── bead fill (tail → head):
    │     draw_segment_beads(rj[i+1], rj[i]) for i=31 down to 0
    │
    └── node markers + head arrow (tail → head):
          joint_node_char(i) at rj[i] for i=32..1
          head_glyph(heading) at rj[0] last

screen_present() → doupdate()
```

---

## Function Breakdown

### tgt_push(s, pos)
Purpose: record actual_target position into the ghost trail
Steps:
1. `tgt_head = (tgt_head + 1) % TARGET_TRAIL_CAP`
2. `tgt_trail[tgt_head] = pos`
3. `tgt_count = min(tgt_count + 1, TARGET_TRAIL_CAP)`

---

### bounce_target(s, wpx, hpx)
Purpose: reflect `tgt_dir` and clamp `tgt_pos` whenever the target crosses the bounded region's walls
Steps:
1. Compute bounded region `[m, wpx-m] × [m, hpx-m]` where `m = EDGE_MARGIN_PX = 64`.
2. If `tgt_pos.x < m` or `tgt_pos.x > wpx − m`: clamp `tgt_pos.x` and set `tgt_dir = π − tgt_dir` (flip cos).
3. If `tgt_pos.y < m` or `tgt_pos.y > hpx − m`: clamp `tgt_pos.y` and set `tgt_dir = −tgt_dir` (flip sin).

Why both reflect AND clamp: a single tick at high `tgt_speed` can push `tgt_pos` several pixels past the wall. Without the clamp, the next tick would still find `tgt_pos` outside, reflect again, and the target would oscillate at the wall.

### move_head(s, dt, cols, rows)
Purpose: advance wander target, bounce it, smooth it, steer head toward it, push trail
Steps:
1. Advance wander: `tgt_time += dt`
2. Compute multi-harmonic turn rate (3 terms)
3. `tgt_dir += turn * dt`
4. `tgt_pos += tgt_speed * (cos(tgt_dir), sin(tgt_dir)) * dt`
5. `bounce_target(s, wpx, hpx)` — reflect `tgt_dir` + clamp `tgt_pos` if it crossed any wall
6. `k = min(dt * TGT_SMOOTH_RATE, 1.0)`; `actual_target += (tgt_pos - actual_target) * k`
7. `dx = actual_target.x - joint[0].x`, `dy = actual_target.y - joint[0].y`
8. `dist = sqrt(dx² + dy²)`
9. If `dist > 0.5`: `heading = atan2(dy, dx)`; `step = min(move_speed*dt, dist)`; `joint[0] += (dx/dist, dy/dist) * step`
10. Hard clamp `joint[0]` into `[0, wpx] × [0, hpx]` (safety net at high speed)
11. `trail_push(joint[0])`, `tgt_push(actual_target)`

---

### render_chain(s, w, cols, rows, alpha)
Purpose: five-pass frame composition
Steps:
1. Alpha lerp all 33 joints
2. Ghost trail: for `k = tgt_count-1 down to 1`: compute `idx = (tgt_head + TARGET_TRAIL_CAP - k) % TARGET_TRAIL_CAP`; stamp `'.'` HUD_PAIR A_DIM
3. Target cursor: stamp `'+'` HUD_PAIR A_BOLD at `actual_target`
4. Bead fill: `draw_segment_beads(rj[i+1], rj[i])` for i=31..0 (tail to head)
5. Node markers: `joint_node_char(i)` for i=32..1; then head arrow at i=0

Note: the five-pass order matters — ghost trail → target → body fill → node markers → head arrow. Each later pass overwrites earlier ones on shared cells. Head arrow always wins.

---

### scene_init(sc, cols, rows)
Purpose: initialise snake with IK target at head position
Steps:
1. Save `theme_idx`; `memset(sc, 0)`; restore `theme_idx`
2. `move_speed=150, tgt_speed=80`
3. `tgt_dir = π/6` (start slightly south-east)
4. `joint[0] = (cols*CELL_W*0.5, rows*CELL_H*0.5)` (screen centre)
5. `tgt_pos = actual_target = joint[0]` (target starts at head, diverges naturally)
6. Pre-populate trail: `trail[k] = joint[0] + k * (-1, 0)` for k=0..4095 (extend west)
7. `trail_head = 0; trail_count = TRAIL_CAP`
8. `compute_joints()`, `memcpy(prev_joint ← joint)`
9. Seed `tgt_trail[0..199] = joint[0]`; `tgt_head = 0; tgt_count = TARGET_TRAIL_CAP`

---

## Pseudocode — Core Loop

```
setup:
  srand(clock_ns())
  atexit(cleanup)
  signals → running=0 / need_resize=1
  screen_init(initial_theme=0)   ← "Medusa" theme
  scene_init(cols, rows)
  frame_time = clock_ns(); sim_accum = 0

main loop (while running):

  1. if need_resize:
       screen_resize() → re-read LINES/COLS
       clamp joint[0] to new pixel bounds   ← no full reset
       need_resize = 0
       reset frame_time, sim_accum = 0

  2. dt = clock_ns() - frame_time; clamp 100ms

  3. sim_accum += dt
     while sim_accum >= tick_ns:
       scene_tick(dt_sec, cols, rows)   ← wander + steer + trail + joints
       sim_accum -= tick_ns

  4. alpha = sim_accum / tick_ns

  5. fps: 500ms window

  6. sleep(NS_PER_SEC/60 - elapsed) before render

  7. erase()
     render_chain(alpha):
       ghost trail dots → target '+' cursor → bead fill → node markers → head arrow
     HUD top: "fps Hz spd:X tgt:Y [theme] state"
     HUD bottom: key reference
     wnoutrefresh + doupdate

  8. getch() loop until ERR

cleanup: endwin()
```

---

## Interactions Between Modules

```
App
 ├── owns Screen (ncurses, cols/rows)
 ├── owns Scene (Snake state)
 └── main loop

Scene / scene_tick:
 ├── memcpy prev_joint          ← snapshot anchor
 ├── move_head()
 │     multi-harmonic wander    ← tgt_pos updated
 │     lerp smoothing           ← actual_target updated
 │     atan2 steering           ← heading + joint[0] updated
 │     trail_push + tgt_push
 └── compute_joints()           ← trail_sample(i*18) × 32

Rendering / render_chain:
 ├── alpha lerp 33 joints
 ├── ghost trail '.' × up to 200
 ├── target '+' cursor
 ├── draw_segment_beads() × 32  (pass 1: fill)
 ├── joint_node_char() × 32     (pass 2: articulation)
 └── head_glyph() × 1           (pass 3: head arrow)

§4 coords
 └── called ONLY inside draw_segment_beads() and render_chain()

§3 themes
 └── live theme_apply() on t/T keypress
```

---

## Key Patterns to Internalize

**IK = atan2 toward goal, FK = formula for heading:**
The single architectural difference between the IK and FK snakes is `move_head()`. In FK: `heading += amplitude * sin(frequency * t) * dt`. In IK: `heading = atan2(target.y - head.y, target.x - head.x)`. Everything else — trail buffer, arc-length sampling, two-pass bead rendering, alpha lerp — is identical. This shows how easily the FK body pattern (trail + arc-length sampling) can be reused with different head-steering strategies.

**Multi-harmonic wander for organic non-repeating paths:**
`sum(Ai * sin(fi * t + φi))` with mutually irrational frequencies produces a quasiperiodic signal that looks random but is fully deterministic. Amplitudes decreasing with frequency (1.40 → 0.80 → 0.40) give a natural "1/f" spectral shape — large slow variations dominate, fine fast variations add texture. This is the same spectral structure as pink noise and many natural phenomena.

**Lerp smoothing as a low-pass filter:**
`actual_target += (tgt_pos - actual_target) * min(dt * rate, 1)` is a first-order IIR filter (exponential moving average). It removes high-frequency content from the target position: wrap discontinuities, sudden direction changes, noise. Any time you have a goal-driven animation that needs smooth motion despite discontinuous input, this lerp pattern is the right tool.

**Ghost trail as debug / art overlay:**
The 200-entry `tgt_trail` buffer and its dim-dot rendering serves two purposes simultaneously: as a debug tool (you can see where the target has been, verifying the multi-harmonic path character) and as an art element (the faint dotted trail behind the target makes the terrain-like quality visible and gives the animation visual depth). Recording a second circular buffer of a computed quantity and rendering it as a background layer is a reusable pattern for any interesting animation state you want to make visible.

**Overshoot prevention with `min(step, dist)` clamp:**
Any goal-directed motion that uses `entity += direction * speed * dt` risks overshooting the goal when `speed * dt > dist`. The clamp `step = min(speed * dt, dist)` is the minimal fix — it costs one comparison and prevents perpetual oscillation around the goal. This pattern appears in any IK or steering system where the agent can move faster than the goal.
