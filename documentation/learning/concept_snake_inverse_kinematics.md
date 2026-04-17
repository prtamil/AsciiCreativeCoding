# Pass 1 — snake_inverse_kinematics: IK goal-seeking head with multi-harmonic wandering target

## From the Source

**Algorithm:** IK goal-seeking head + path-following FK body. Head steers toward `actual_target` (a lerp-smoothed wander position) at `move_speed` px/s. Body joints are placed by arc-length sampling of the head's recorded trail — no per-joint angle formula required. The wander target steers itself via a superposition of three incommensurable sine waves, producing terrain-like paths that never exactly repeat.

**Math:** Multi-harmonic wander: `turn = A1*sin(f1*t) + A2*sin(f2*t+φ2) + A3*sin(f3*t+φ3)`. Target direction integrates: `tgt_dir += turn * dt`. Target position: `tgt_pos += tgt_speed * (cos(tgt_dir), sin(tgt_dir)) * dt`. Smooth target: `actual_target += (tgt_pos - actual_target) * min(dt * TGT_SMOOTH_RATE, 1)`. Head steering: `heading = atan2(actual_target.y - head.y, actual_target.x - head.x)`. Movement: `head += (dx/dist) * min(move_speed*dt, dist)` (no overshoot clamp).

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

The `actual_target` smoothing (`lerp toward tgt_pos at 8×/s`) is a low-pass filter. Without it, the target could jump across the wrap boundary discontinuously (screen edge), causing the head to instantly snap to a new heading. The lerp makes the head steering oblivious to wrap jumps: the actual_target glides from one side of the screen to the other smoothly.

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

**Why three sine harmonics instead of one?**
One sine wave produces a perfectly regular sinusoidal path — the target oscillates between two fixed lateral extremes with a fixed period. This looks like a pendulum, not terrain. Two or three harmonics with different frequencies produce a path that looks more complex and organic. The specific choice of three frequencies (0.29, 0.71, 1.13 rad/s) with amplitudes (1.40, 0.80, 0.40) gives: wide sweeping hills from the slow harmonic, medium wiggles overlaid, and fine tremors from the fast harmonic. The sizes decrease with frequency, mirroring how real terrain has large mountains, smaller hills, and fine texture.

**Why incommensurable frequencies?**
Two frequencies f1 and f2 are commensurable if `f1/f2` is rational — in that case the combined waveform has period `LCM(2π/f1, 2π/f2)`. For `0.29` and `0.71`: their ratio is `0.29/0.71 ≈ 0.4084...`, which is irrational (cannot be expressed as p/q for integers p, q). The waveform `A1*sin(f1*t) + A2*sin(f2*t)` with irrational ratio is quasiperiodic — it never exactly repeats. Adding a third incommensurable frequency (`0.29/1.13 ≈ 0.257...`) reinforces the non-repetition further. The target path is practically aperiodic.

**Why use `actual_target` as a lerp-smoothed version of `tgt_pos` instead of steering toward `tgt_pos` directly?**
`tgt_pos` wraps toroidally when it crosses screen edges. At the wrap moment, `tgt_pos` jumps by `wpx` or `hpx` pixels instantly. If the head steered toward the raw `tgt_pos`, it would immediately snap to a completely different heading when the target wrapped. `actual_target` lerps toward `tgt_pos` at `8×/s`: `k = dt * 8; actual_target += (tgt_pos - actual_target) * k`. This means at 60 Hz, `k ≈ 0.133` per tick — a smooth chase, not a snap. The wrap discontinuity in `tgt_pos` is absorbed: `actual_target` glides from near the left edge to near the right edge over several ticks rather than jumping instantly.

**Why clamp `step = min(move_speed * dt, dist)` when moving the head?**
Without the clamp, if `dist < move_speed * dt` (head is very close to target), the head would overshoot the target and oscillate around it. The clamp ensures the head always moves at most `dist` toward the target — it arrives at the target smoothly and stops (until the target moves further away). Without this, the head would jitter back and forth around the target when it gets close.

**Why record `actual_target` (not `tgt_pos`) in the ghost trail?**
`tgt_pos` wraps discontinuously. `actual_target` is smooth — it has been low-pass filtered. The ghost trail is rendered as a path, and a path with sudden jumps in it would draw a line from one side of the screen to the other on every wrap, cluttering the display. Recording `actual_target` keeps the ghost trail as a continuous-looking dotted line tracing the terrain the snake has been chasing.

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
  toroidal wrap for tgt_pos

Step 2: smooth actual_target
  k = min(dt * 8.0, 1.0)
  actual_target += (tgt_pos - actual_target) * k

Step 3: steer head toward actual_target
  dx, dy = actual_target - joint[0]
  dist = sqrt(dx² + dy²)
  if dist > 0.5:
    heading = atan2(dy, dx)
    step = min(move_speed * dt, dist)   ← no overshoot
    joint[0] += (dx/dist, dy/dist) * step

Step 4: toroidal wrap for joint[0]

Step 5: trail_push(joint[0])
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
- The target trail buffer records `actual_target`, not `tgt_pos`. But `actual_target` is computed by lerping toward `tgt_pos`, which can include wrap-around jumps if `tgt_pos` wraps. At the exact frame where `tgt_pos` wraps from `wpx-1` to `1`, does `actual_target` receive a large discontinuity before the lerp smooths it out? Would this show as a visual glitch in the ghost trail?
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
| §5b move_head | Multi-harmonic wander → tgt_pos → actual_target lerp → atan2 steer → move → wrap → push |
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
    │     toroidal wrap for tgt_pos
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
    │     toroidal wrap for joint[0]
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

### move_head(s, dt, cols, rows)
Purpose: advance wander target, smooth it, steer head toward it, push trail
Steps:
1. Advance wander: `tgt_time += dt`
2. Compute multi-harmonic turn rate (3 terms)
3. `tgt_dir += turn * dt`
4. `tgt_pos += tgt_speed * (cos(tgt_dir), sin(tgt_dir)) * dt`
5. Toroidal wrap for `tgt_pos`
6. `k = min(dt * TGT_SMOOTH_RATE, 1.0)`; `actual_target += (tgt_pos - actual_target) * k`
7. `dx = actual_target.x - joint[0].x`, `dy = actual_target.y - joint[0].y`
8. `dist = sqrt(dx² + dy²)`
9. If `dist > 0.5`: `heading = atan2(dy, dx)`; `step = min(move_speed*dt, dist)`; `joint[0] += (dx/dist, dy/dist) * step`
10. Toroidal wrap for `joint[0]`
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
