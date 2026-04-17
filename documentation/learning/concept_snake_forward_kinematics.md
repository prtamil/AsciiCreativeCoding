# Pass 1 — snake_forward_kinematics: Circular trail buffer + arc-length path-following FK

## From the Source

**Algorithm:** Path-following FK. The head's past pixel positions are stored tick-by-tick in a circular trail buffer. Each body joint is placed by measuring cumulative arc length backward along the trail and linearly interpolating. No per-segment angle computation is needed; the trail encodes the full geometry of the path already taken. Sinusoidal auto-steering turns the heading continuously, producing smooth S-curve locomotion with no user input.

**Math:** Head steering: `dheading/dt = amplitude * sin(frequency * wave_time)`. Head translation: `joint[0] += move_speed * (cos(heading), sin(heading)) * dt`. Arc-length sampling: walk trail from newest, accumulate Euclidean segment lengths until `accum >= target_dist`, then `lerp(a, b, (target_dist - accum_before) / seg_len)`. Render: `rj[i] = prev_joint[i] + (joint[i] - prev_joint[i]) * alpha`.

**Performance:** Fixed-step accumulator decouples physics Hz from render Hz. At default speed (72 px/s at 60 Hz), each tick adds 1.2 px to trail. The farthest joint (joint[32]) is 576 px behind head → `trail_sample` walks ~480 entries. With 32 joints: ~15 360 iterations/tick — trivial. ncurses diff engine sends ~60–120 changed cells per frame.

**Data-structure:** Circular trail buffer `Vec2 trail[TRAIL_CAP]` (4096 entries), one position per simulation tick. `trail_head` is the write pointer; `trail_at(k)` retrieves k steps back from newest. Two joint arrays `joint[]` and `prev_joint[]` enable sub-tick alpha lerp. 10-theme colour system with live palette swap.

## Core Idea

A 32-segment snake swims autonomously across the screen in a smooth sinusoidal S-curve. The foundational insight is that the body does not compute its shape from a formula — it follows the actual recorded path of the head. Every pixel position the head visits is logged in a circular trail buffer. Each body joint is placed at a fixed arc-length distance behind the head by walking backward along that buffer. Any curve the head carves propagates down the body exactly as it would in a real snake, because the body is literally tracing the historical path.

This trail-buffer + arc-length pattern is the foundational FK technique in this codebase. The centipede, medusa, and spider demos all use it. Understanding it here unlocks all of them.

## The Mental Model

Imagine laying a piece of string on a table and dragging one end (the head) in a wavy path. The rest of the string follows, maintaining its length, tracing the exact same curves the head carved. Now imagine the head position is recorded every millisecond in a notebook. To find where joint 5 should be, you flip back through the notebook until you have read entries that together cover 5 segment lengths of distance, then mark that spot. The result is the exact point on the path that is 5 segment lengths behind the head.

The circular trail buffer IS that notebook. `trail_sample(dist)` is the "flip back" operation. The fact that it uses arc length (not time, not entry count) means the body spacing is constant in physical distance regardless of whether the head was moving fast or slow.

The autonomous steering (`dheading/dt = amplitude * sin(frequency * wave_time)`) produces an S-curve because it alternately swings the heading left and right. The heading is integrated (not snapped), so the transitions are smooth. The body follows the resulting curved path naturally via the trail.

## Data Structures

### Vec2
```
x, y   — position in pixel space (float, isotropic square grid)
```

### Snake
```
trail[TRAIL_CAP]    — circular buffer of head positions, newest at trail[trail_head]
trail_head          — write pointer (index of most recently pushed entry)
trail_count         — valid entries, clamped to TRAIL_CAP (= 4096)

joint[N_SEGS+1]     — [0]=head position ... [32]=tail tip
prev_joint[N_SEGS+1]— snapshot of joint[] at tick START (before physics)
                      render_chain() lerps prev_joint → joint using alpha

heading        — current travel direction (radians); 0=east, π/2=south
move_speed     — translation speed (px/s)
wave_time      — accumulated scaled time driving sin()
amplitude      — peak turn rate for auto-swim (rad/s)
frequency      — oscillation angular frequency (rad/s)
speed_scale    — multiplier on wave_time advancement (not move_speed)

theme_idx      — index into THEMES[]; t/T keys cycle it
paused         — when true, move_head() and compute_joints() are skipped
```

`trail[4096]` is the dominant memory consumer: `4096 × 8 bytes = 32 KB`. Because `Snake` is inside the global `g_app`, it lives in BSS (zero-initialized), not the stack.

### Scene
```
snake   — the single Snake simulation state
```

## The Main Loop

Each iteration:

1. **Check resize.** Re-read terminal dimensions. Clamp `joint[0]` to new pixel bounds (snake continues from wherever it is — no teleport). Reset timing.

2. **Measure dt.** Monotonic clock, capped at 100 ms.

3. **Physics accumulator.** While `sim_accum >= tick_ns`: call `scene_tick(dt_sec)` and drain. Physics always runs at exactly `sim_fps` Hz.

4. **Compute alpha.** `sim_accum / tick_ns ∈ [0, 1)`.

5. **FPS counter.** 500 ms sliding window.

6. **Sleep for frame cap.** Before render to hold ~60 fps. Terminal I/O latency does not bleed into next frame budget.

7. **Draw and present.** `erase()` → `render_chain(alpha)` (two-pass bead fill + node markers + head arrow) → HUD bars → `wnoutrefresh + doupdate`.

8. **Handle input.** Non-blocking `getch()` loop. Arrow keys adjust speed, frequency, amplitude. `+/-` adjust `speed_scale`. `t/T` cycle themes.

## Non-Obvious Decisions

**Why trail buffer + arc-length sampling instead of per-joint angle formula?**
The naive FK formula (`joint[i+1] = joint[i] - SEG_LEN * (cos θ_i, sin θ_i)`) is stateless — it recalculates the entire chain from the current heading each tick. This means the body does not actually follow the path the head carved; it is always computed from the current heading direction. A turning head produces a mathematically generated shape, not a physically correct chain. The trail buffer approach gives the body genuine memory of the head's actual path.

**Why TRAIL_CAP = 4096?**
At the minimum speed (20 px/s, 60 Hz) each tick adds 0.33 px to the trail. The full snake body (576 px) would need `576 / 0.33 ≈ 1745` entries. 4096 gives a 2.3× safety margin. At maximum speed (500 px/s), each tick adds 8.3 px and only ~70 entries cover the full body — 4096 is far more than enough. The 32 KB cost is acceptable for a global allocation.

**Why pre-populate the trail at `scene_init()`?**
At startup, `trail_count = 0`. If `trail_sample()` ran on an empty trail it would return the oldest entry (just the head position) for every body joint, making the snake appear as a single point. The fix: fill all 4096 trail entries with positions extending behind the head in the opposite-of-heading direction, 1 px apart. From frame one, `trail_sample(576)` correctly returns a point 576 px behind the head.

**Why initialise `wave_time = π/2` instead of 0?**
At `wave_time = 0`: `sin(frequency × 0) = 0` → turn rate = 0 → the snake swims straight for several seconds before the wave builds up visibly. At `wave_time = π/2`: `sin(frequency × π/2) ≈ sin(0.99 × 1.57) ≈ 1.0` → the turn rate starts at its peak. The snake is already carving a visible curve on the very first frame.

**Why integrate the turn rate into heading instead of assigning it?**
`heading += turn * dt` is integration. Assigning `heading = some_formula()` would produce a square-wave direction change (instant snap to each new angle every tick). Integration gives smooth continuous curvature because the heading accumulates fractional changes. This matches real muscle-driven locomotion where turning force builds and relaxes gradually.

**Why not normalise `heading` to `[0, 2π)`?**
`sinf()` and `cosf()` are periodic — they accept any floating-point angle without normalisation. Normalising `heading` to `[0, 2π)` would introduce a tiny discontinuity every time it crosses the `±π` boundary, which can appear as a single-frame direction jerk. By letting `heading` accumulate freely as a float, no discontinuity ever occurs.

**Why `(trail_head + TRAIL_CAP - k) % TRAIL_CAP` instead of `(trail_head - k) % TRAIL_CAP`?**
In C, the `%` operator on negative operands is implementation-defined before C99, and can return negative values in C99/C11 when the left operand is negative. `trail_head - k` can be negative when `k > trail_head`. Adding `TRAIL_CAP` before subtracting `k` guarantees the operand is always non-negative (as long as `k < TRAIL_CAP`), making the modulo operation safe and well-defined.

**Why draw tail-to-head in `render_chain()` (Step 2 and 3)?**
When two segments overlap (e.g. in a tight loop), the segment drawn last wins the cell. Drawing from tail to head means head-end segments always win over tail segments on overlaps. Since the head is the focus of attention, this ensures the head-end is always visible — the tail "disappears under" the head rather than vice versa.

**Why use `SEG_LEN_PX = 18.0` specifically?**
32 segments × 18 px = 576 px total body length. At CELL_W = 8, that is 72 terminal columns — long enough to show the full S-curve pattern on most terminals. Shorter segments (smaller SEG_LEN_PX) mean more joints per visible area, smoother curves, but more `trail_sample()` iterations (cost scales with `N_SEGS × SEG_LEN_PX / speed`). 18 px ≈ 2.25 columns per segment: direction transitions look gradual, not blocky.

**Why `speed_scale` separate from `move_speed`?**
`wave_time += dt * speed_scale` — the wave clock runs faster. `joint[0] += move_speed * direction * dt` — the head moves at the same physical speed. Increasing `speed_scale` produces tighter S-curves (the head turns faster relative to how far it has moved) without changing how fast the snake crosses the screen. If you just increased `frequency`, you would also need to rebalance `amplitude` to maintain the same spatial wavelength. `speed_scale` lets you compress/stretch the wave independently.

## State Machines

### scene_tick order
```
┌─────────────────────────────────────────────────────┐
│ 1. memcpy(prev_joint ← joint)  [snapshot before]   │
│                                                     │
│ 2. if paused → return           [clean freeze]      │
│                                                     │
│ 3. move_head(dt, cols, rows):                       │
│      wave_time += dt * speed_scale                  │
│      turn = amplitude * sin(frequency * wave_time)  │
│      heading += turn * dt                           │
│      joint[0] += move_speed * (cos,sin)(heading)*dt │
│      toroidal wrap: if joint[0].x < 0 → += wpx etc │
│      trail_push(joint[0])                           │
│                                                     │
│ 4. compute_joints():                                │
│      joint[i] = trail_sample(i * SEG_LEN_PX)        │
│      (i = 1..32)                                    │
└─────────────────────────────────────────────────────┘
```

### trail_sample() walkthrough
```
Input: dist = target arc-length from head (e.g. 3 × 18 = 54 px for joint 3)

accum = 0; a = trail[trail_head]   (newest = current head pos)

for k = 1, 2, 3, ...:
  b = trail[trail_head - k]        (one tick older each step)
  seg = |b - a|                    (Euclidean distance)

  if accum + seg >= dist:
    t = (dist - accum) / seg       ← fraction along this segment
    return lerp(a, b, t)           ← interpolated point

  accum += seg                     ← not there yet, keep walking
  a = b

return trail[oldest]               ← fallback (trail exhausted)
```

### Two-pass rendering (render_chain)
```
Step 1: alpha lerp all 33 joints:
          rj[i] = prev_joint[i] + (joint[i] - prev_joint[i]) * alpha

Step 2: bead fill, TAIL → HEAD (pass 1):
          for i = 31 down to 0:
            draw_segment_beads(rj[i+1], rj[i], seg_pair(i), seg_attr(i))
            → walk segment in 3px steps, stamp 'o' with dedup

Step 3: joint node markers, TAIL → HEAD (pass 2):
          for i = 32 down to 1:
            stamp joint_node_char(i) at rj[i]
            → '0' (head third), 'o' (middle), '.' (tail third)

Step 4: head arrow (drawn last, always wins):
          stamp head_glyph(heading) at rj[0]
          → >, v, <, ^ based on heading quadrant
```

### Colour gradient (N_SEGS = 32, N_PAIRS = 7)
```
seg_pair(i)  = 1 + (i * 6) / 31
seg_attr(i)  = A_BOLD  (i < 8)   head quarter — brightest
             = A_DIM   (i > 24)  tail quarter — dimmest
             = A_NORMAL           mid body
```

| Segment range | Pair | Default Solar theme colour |
|--------------|------|--------------------------|
| 0–4 | 1 | bright yellow (226) |
| 5–9 | 2 | yellow-orange (220) |
| 10–14 | 3 | orange (214) |
| 15–19 | 4 | orange-red (208) |
| 20–24 | 5 | red-orange (202) |
| 25–29 | 6 | red (196) |
| 30–31 | 7 | dark red (160) |

### Input actions
| Key | Effect |
|-----|--------|
| q / Q / ESC | exit |
| space | toggle pause |
| r / R | reset simulation (theme preserved) |
| UP | move_speed × 1.20 (max 500) |
| DOWN | move_speed ÷ 1.20 (min 20) |
| LEFT / a / A | frequency − 0.1 (min 0.10) |
| RIGHT / d / D | frequency + 0.1 (max 6.00) |
| w / W | amplitude + 0.1 (max 4.0) |
| s / S | amplitude − 0.1 (min 0.0) |
| + / = | speed_scale × 1.25 (max 8.0) |
| - | speed_scale ÷ 1.25 (min 0.05) |
| t | next theme (10 total, wrapping) |
| T | previous theme |
| ] | sim_fps + 10 |
| [ | sim_fps − 10 |

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|----------|---------|-------------------|
| N_SEGS | 32 | More segments → longer snake, more smooth; more `trail_sample()` iterations per tick. |
| SEG_LEN_PX | 18.0 px | Longer → fewer segments visible in any area, blockier; shorter → more joints, smoother curves. Total body = N_SEGS × SEG_LEN_PX px. |
| TRAIL_CAP | 4096 entries | Must be large enough to cover `N_SEGS × SEG_LEN_PX / (MOVE_SPEED_MIN / SIM_FPS_DEFAULT)` entries. |
| AMPLITUDE_DEFAULT | 0.52 rad/s | 0 → perfectly straight. Larger → wider lateral arcs. > 2.0 → tight spirals. |
| FREQUENCY_DEFAULT | 0.95 rad/s | Period = 2π/0.95 ≈ 6.6 s. Increase → faster wiggles, tighter spatial wavelength. |
| MOVE_SPEED_DEFAULT | 72 px/s | Increase → snake crosses screen faster; body fills with trail faster; spatial wavelength stretches. |
| DRAW_STEP_PX | 3.0 px | Must be < CELL_W (8). Larger → gaps in near-horizontal segments. Smaller → denser fill, minimal visual benefit. |
| SPEED_SCALE_DEFAULT | 1.0 | > 1 → wave clock runs faster → tighter curves at same physical speed. |

## Open Questions for Pass 3

- `trail_sample()` walks the trail from newest to oldest, O(dist / px_per_tick) per call. With 32 joints all called in sequence, joint 32 takes 32× longer to sample than joint 1. Is there a way to compute all 32 joints in a single O(trail_length) pass? (Hint: walk the trail once, sampling at each multiple of SEG_LEN_PX along the way.)
- At `MOVE_SPEED_MIN = 20 px/s`, `60 Hz`, each tick adds `0.33 px` to the trail. The trail fills at the rate of 0.33 entries per px of body. At very low speeds does `trail_sample()` ever fail to find enough arc-length and fall back to returning the oldest entry? What does this look like visually?
- The toroidal wrap sets `joint[0].x -= wpx` when it exceeds the pixel width. The trail then has a large discontinuity: one entry at `wpx - 1` and the next at `0 + something`. `trail_sample()` will compute a large `seg` distance for this entry pair and immediately overshoot. What visual artifact does this produce and how long does it last?
- `wave_time` is not normalised and accumulates indefinitely. After `2π / FREQUENCY_DEFAULT / dt = ~395` ticks at 60 Hz, the argument to `sinf()` has accumulated about `0.95 * 395 / 60 ≈ 6.25` radians. Over days of running, does floating-point precision degrade `sinf()`'s output for very large arguments?
- When `amplitude = 0`, the snake swims in a perfectly straight line. The `heading` still accumulates `turn * dt = 0 * dt = 0` per tick, so no change. But what happens to the trail? Does `trail_sample()` still work correctly when all 4096 trail entries are collinear at 1 px spacing?
- The 10 themes are hard-coded in `THEMES[10]`. If you wanted to add an 11th theme, you would need to change `N_THEMES`. Is there a way to detect the array length automatically in C without a count constant?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app` | `App` | ~67 KB | top-level container |
| `g_app.scene.snake.trail[4096]` | `Vec2[4096]` | 32 KB | circular head position history |
| `g_app.scene.snake.joint[33]` | `Vec2[33]` | 264 B | current body joint positions |
| `g_app.scene.snake.prev_joint[33]` | `Vec2[33]` | 264 B | tick-start snapshot for alpha lerp |
| `THEMES[10]` | `Theme[10]` (global const) | ~400 B | 10 named colour palettes |

---

# Pass 2 — snake_forward_kinematics: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | All tunables: N_SEGS, TRAIL_CAP, SEG_LEN_PX, DRAW_STEP_PX, speed/amplitude/frequency ranges |
| §2 clock | `clock_ns()` / `clock_sleep_ns()` — verbatim from framework |
| §3 color | `Theme` struct, `THEMES[10]`, `theme_apply()`, `color_init()` — 10-palette system |
| §4 coords | `px_to_cell_x/y` — aspect-ratio bridge, called only at draw time |
| §5a trail helpers | `trail_push()`, `trail_at()`, `trail_sample()` — the three trail operations |
| §5b move_head | Advance wave_time → integrate heading → translate head → wrap → trail_push |
| §5c compute_joints | `joint[i] = trail_sample(i * SEG_LEN_PX)` for i=1..32 |
| §5d rendering helpers | `seg_pair`, `seg_attr`, `joint_node_char`, `head_glyph` |
| §5e draw_segment_beads | Dense 'o' fill for one segment (pass 1 of two-pass render) |
| §5f render_chain | Alpha lerp → bead fill (tail→head) → node markers → head arrow |
| §6 scene | `scene_init` (trail pre-population + initial joints) / `scene_tick` / `scene_draw` |
| §7 screen | ncurses layer: erase → draw → wnoutrefresh → doupdate |
| §8 app | Signal handlers, resize (clamp joint[0], no scene reset), main loop |

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
    │     wave_time += dt * speed_scale
    │     turn = amplitude * sin(frequency * wave_time)
    │     heading += turn * dt              [integrate → smooth curves]
    │     joint[0] += move_speed * (cos(heading), sin(heading)) * dt
    │     toroidal wrap for joint[0]
    │     trail_push(joint[0])             [record new head position]
    │
    └── compute_joints():
          for i = 1..32:
            joint[i] = trail_sample(i * SEG_LEN_PX)
                ← walk trail from k=1 accumulating Euclidean seg distances
                ← when accum >= i*18: lerp(a, b, t) and return

sim_accum -= tick_ns
    │
    ▼
alpha = sim_accum / tick_ns
    │
    ▼
render_chain(alpha)
    │
    ├── rj[i] = lerp(prev_joint[i], joint[i], alpha)  [smooth motion]
    │
    ├── pass 1: bead fill (tail → head)
    │     draw_segment_beads(rj[i+1], rj[i], pair, attr)
    │       → walk in 3px steps, stamp 'o', dedup by (cx,cy)
    │
    ├── pass 2: node markers (tail → head)
    │     joint_node_char(i) = '0' | 'o' | '.'
    │     stamped at rj[i] for i=32 down to 1
    │
    └── pass 3: head arrow last (always wins overlaps)
          head_glyph(heading) = '>' | 'v' | '<' | '^'
          stamped at rj[0]

screen_present() → doupdate()
```

---

## Function Breakdown

### trail_push(s, pos)
Purpose: append pos to circular trail buffer
Steps:
1. `trail_head = (trail_head + 1) % TRAIL_CAP`
2. `trail[trail_head] = pos`
3. `trail_count = min(trail_count + 1, TRAIL_CAP)`

---

### trail_at(s, k) → Vec2
Purpose: retrieve trail entry k steps back from newest
Steps:
1. Return `trail[(trail_head + TRAIL_CAP - k) % TRAIL_CAP]`
Note: `+ TRAIL_CAP` prevents negative modulo in C

---

### trail_sample(s, dist) → Vec2
Purpose: interpolated position at arc-length dist behind head
Steps:
1. `accum = 0; a = trail_at(0)`
2. For `k = 1..trail_count-1`:
   a. `b = trail_at(k)`, `seg = |b - a|`
   b. If `accum + seg >= dist`: return `lerp(a, b, (dist - accum) / max(seg, 1e-4))`
   c. `accum += seg; a = b`
3. Return `trail_at(trail_count - 1)` (fallback: trail exhausted)
Cost: O(dist / px_per_tick) — proportional to how far back in the trail we need to look

---

### move_head(s, dt, cols, rows)
Purpose: advance wave, update heading, translate head, wrap, record trail
Steps:
1. `wave_time += dt * speed_scale`
2. `turn = amplitude * sinf(frequency * wave_time)`
3. `heading += turn * dt`
4. `joint[0].x += move_speed * cosf(heading) * dt`
5. `joint[0].y += move_speed * sinf(heading) * dt`
6. Toroidal wrap: `if joint[0].x < 0: += wpx`, etc.
7. `trail_push(joint[0])`

---

### compute_joints(s)
Purpose: place all body joints from trail
Steps:
1. For `i = 1..N_SEGS`: `joint[i] = trail_sample(i * SEG_LEN_PX)`

---

### draw_segment_beads(w, a, b, pair, attr, cols, rows)
Purpose: fill segment a→b with 'o' beads at DRAW_STEP_PX intervals
Steps:
1. `dx, dy = b - a`, `len = ||(dx,dy)||`; return if `len < 0.1`
2. `nsteps = ceil(len / DRAW_STEP_PX) + 1`
3. Walk `t = 0..nsteps`:
   - `u = t/nsteps`, `cx = px_to_cell_x(a.x + dx*u)`, `cy = ...`
   - Skip if same cell as previous (dedup)
   - Skip if out of bounds
   - Stamp `'o'` with `COLOR_PAIR(pair) | attr`

---

### render_chain(s, w, cols, rows, alpha)
Purpose: two-pass bead render with alpha interpolation
Steps:
1. `rj[i] = prev_joint[i] + (joint[i] - prev_joint[i]) * alpha` for i=0..32
2. Bead fill, i = 31 down to 0: `draw_segment_beads(rj[i+1], rj[i], seg_pair(i), seg_attr(i))`
3. Node markers, i = 32 down to 1: `mvwaddch(cy, cx, joint_node_char(i))`
4. Head arrow: `mvwaddch(hy, hx, head_glyph(heading))` with A_BOLD

---

### scene_init(sc, cols, rows)
Purpose: place snake at screen position with full pre-populated trail
Steps:
1. Save `theme_idx`; `memset(sc, 0)`; restore `theme_idx`
2. Set `move_speed=72, amplitude=0.52, frequency=0.95, speed_scale=1.0`
3. `wave_time = π/2` (start at peak of sine → immediate curves)
4. `heading = π/8` (slightly south-east → drifts toward visual centre)
5. `joint[0] = (cols*CELL_W*0.38, rows*CELL_H*0.50)`
6. Pre-populate: `trail[k] = joint[0] + k * (cos(heading+π), sin(heading+π))` for k=0..4095
   — 4096 entries at 1px spacing extending backward from head
7. `trail_head = 0; trail_count = TRAIL_CAP`
8. `compute_joints()` then `memcpy(prev_joint ← joint)`

---

## Pseudocode — Core Loop

```
setup:
  srand(clock_ns())
  atexit(cleanup)
  signals → running=0 / need_resize=1
  screen_init() ← color_init(theme 0)
  scene_init(cols, rows)
  frame_time = clock_ns(); sim_accum = 0

main loop (while running):

  1. if need_resize:
       screen_resize() → re-read LINES/COLS
       clamp joint[0] to new pixel bounds  ← no full reset, snake continues
       reset frame_time, sim_accum = 0

  2. dt = clock_ns() - frame_time; clamp 100ms; frame_time = now

  3. sim_accum += dt
     tick_ns = NS_PER_SEC / sim_fps; dt_sec = tick_ns / NS_PER_SEC
     while sim_accum >= tick_ns:
       scene_tick(dt_sec, cols, rows)   ← wave + head + trail + joints
       sim_accum -= tick_ns

  4. alpha = sim_accum / tick_ns   [0..1)

  5. fps: 500ms window

  6. sleep(NS_PER_SEC/60 - elapsed) before render

  7. erase()
     render_chain(alpha)     ← bead fill + node markers + head arrow
     HUD top:   "fps Hz spd:X amp:Y freq:Z x_scale [theme] state"
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
 └── main loop drives everything

Scene / scene_tick:
 ├── memcpy prev_joint  ← snapshot anchor
 ├── move_head()        ← wave → heading → translation → trail_push
 └── compute_joints()   ← trail_sample(i*SEG_LEN_PX) for all 32 joints

Rendering / render_chain:
 ├── alpha lerp all 33 joints
 ├── draw_segment_beads() × 32  (pass 1: fill)
 ├── joint_node_char() × 32     (pass 2: articulation)
 └── head_glyph() × 1           (pass 3: head arrow)

§4 coords (px_to_cell_x/y)
 └── called ONLY inside draw_segment_beads() — never in physics

§3 themes
 └── theme_apply() called at init and on t/T keypress
     live init_pair() — effective on next render frame
```

---

## Key Patterns to Internalize

**Trail buffer as simulation memory:**
The circular trail buffer is the only persistent state the body needs. No bone rotations, no joint constraints — just a record of where the head has been. Every downstream demo (centipede, medusa, spider) that uses multiple chains or branching bodies uses this same buffer for each chain.

**Arc-length sampling ensures constant body spacing:**
`trail_sample(i * SEG_LEN_PX)` measures distance physically, not by entry count. Whether the snake was fast or slow, the body joint lands at exactly `i * 18 px` behind the head in physical distance. This is what prevents the body from "bunching up" when the snake slows down.

**Sinusoidal heading integration produces S-curves:**
`dheading/dt = A * sin(f * t)` is a sinusoidal angular velocity. Integrating gives a heading that oscillates around its mean. The spatial path of a point moving at constant speed with sinusoidally varying heading is an approximate sinusoid — hence the S-curve. The key is that `heading` is integrated (not assigned), producing smooth continuous curvature.

**Two-pass bead rendering gives articulated structure:**
Pass 1 fills the visual volume of each segment with `'o'` beads. Pass 2 overwrites joint positions with size-graded markers (`'0'`/`'o'`/`'.'`). This separation means the segment fill and joint articulation are independently controllable — you can change segment density (DRAW_STEP_PX) without affecting joint appearance, and change joint glyphs without affecting segment fill.

**Tail-to-head draw order for natural depth:**
Drawing from tail to head (i = N_SEGS-1 down to 0) means the head always wins any cell conflict. Since the head is where the action is, this makes the most visually important part of the snake always fully visible.
