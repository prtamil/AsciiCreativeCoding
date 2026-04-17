# Pass 1 — ragdoll_ropes: Verlet rope simulation with sinusoidal wind and phase offsets

## From the Source

**Algorithm:** Position-based Verlet integration with iterative distance constraints. Unlike spring forces, distance constraints directly correct particle positions to maintain segment length, so the rope is numerically inextensible at any dt. Six constraint passes per tick converge to within floating-point precision for a 20-node chain. A sinusoidal wind force with per-rope phase offsets (`phase[r] = r × 2π / N_ROPES`) drives each rope independently, creating staggered rhythmic sway. Anchor particle `[r][0]` is pinned by resetting both `pos` and `old_pos` after every Verlet step and every constraint pass.

**Math:** Verlet update: `vel = (pos - old_pos) * ROPE_DAMPING`, `new_pos = pos + vel + accel * dt²`. Constraint: `error = (dist - rest_len) / dist`; correction `= 0.5 * error * (p2 - p1)` split equally. Wind formula: `accel_x = wind_amp * sin(wind_time * wind_freq + phase_offset[r])`. Rope length variation: `len[r] = min_len + r * (max_len - min_len) / (N_ROPES - 1)`, with `min_len = screen_height * 0.35` and `max_len = screen_height * 0.75`.

**Performance:** `N_ROPES × N_SEG = 7 × 20 = 140` particles. `N_ITERS = 6` constraint passes per tick × `(N_SEG - 1) = 19` segments × `N_ROPES = 7` ropes = 798 distance corrections per tick. Trivial. `ncurses doupdate()` transmits approximately 200–400 changed cells per frame.

**Data-structure:** `Scene` struct holds flat 2D arrays: `pos[N_ROPES][N_SEG]`, `old_pos[N_ROPES][N_SEG]`, `prev_pos[N_ROPES][N_SEG]`. `rest_len[N_ROPES]` stores per-segment rest length (= `rope_len_px[r] / (N_SEG - 1)`). `anchor[N_ROPES]` holds fixed ceiling pixel positions. `phase_offset[N_ROPES]` holds pre-computed 2π offsets. Single `wind_time` accumulator drives all sin() calls.

## Core Idea

Seven ropes hang from ceiling anchors evenly distributed across the terminal. Each rope has 20 particles connected by distance constraints. A sinusoidal wind force drives each rope at a different phase, creating a staggered "Mexican wave" sway where neighbouring ropes always oscillate in opposite directions. The key insight is that anchor enforcement requires resetting BOTH `pos` and `old_pos` — resetting only `pos` leaves a phantom velocity in `old_pos` that pulls the anchor away on the very next tick.

## The Mental Model

Imagine seven beaded necklaces pinned to the ceiling. Each bead is connected to the next by a rigid link. You shake them with a wave machine, but each necklace is shifted slightly in phase from its neighbour — one is at the peak of its swing while the next is at the trough, and so on around the full cycle.

The "rigid link" is not a real rigid link — it is an illusion produced by a correction step: every tick, if any two adjacent beads are too far apart, push them together. If too close, push them apart. Do this 6 times. After 6 passes the links look rigid because the error has been reduced to fractions of a pixel.

The anchor bead at the top is special. Every tick, regardless of what the physics computed, you forcibly set it back to the ceiling position. But you must also reset its "previous position" to the same ceiling position — otherwise the physics engine sees a huge velocity kick (from wherever the bead was computed to be, back to the ceiling) and propagates that shock down the rope.

## Data Structures

### Vec2
```
x, y   — position in pixel space (float, square isotropic grid)
```

### Scene
```
pos[7][20]         — current particle positions (pixel space)
old_pos[7][20]     — previous tick positions (Verlet implicit velocity)
prev_pos[7][20]    — tick-start snapshot for alpha lerp

rest_len[7]        — per-segment rest length for each rope (px)
                     = rope_len_px[r] / (N_SEG - 1)
rope_len_px[7]     — total visual length of each rope (35%–75% screen height)
anchor[7]          — fixed ceiling pixel position for pos[r][0]
phase_offset[7]    — wind phase offset per rope = r × 2π / N_ROPES

wind_time          — accumulated simulation seconds (drives sin() argument)
wind_amp           — peak lateral acceleration (px/s², adjustable at runtime)
wind_freq          — oscillation angular frequency (rad/s, adjustable)

paused             — when true, rope_tick() saves prev_pos then returns early
theme_idx          — index into THEMES[]; preserved across r/R reset
```

Rope r uses colour pair `(r % N_PAIRS) + 1`, cycling through all 7 pairs. Each rope therefore has a distinct colour in any 7-colour theme, giving immediate visual separation between ropes.

## The Main Loop

Each iteration:

1. **Check resize.** Re-read terminal dimensions, call `scene_init()` (ropes fully re-initialised for new size — simpler and more correct than relocating 140 scattered particles). Reset timing.

2. **Measure dt.** Monotonic clock, capped at 100 ms.

3. **Physics accumulator.** While `sim_accum >= tick_ns`: call `scene_tick(dt_sec)` and drain.

4. **Compute alpha.** `sim_accum / tick_ns ∈ [0, 1)`.

5. **FPS counter.** 500 ms window.

6. **Sleep for frame cap.** Before render, to hold ~60 fps.

7. **Draw and present.** `erase()` → `render_scene(alpha)` (ceiling line + all ropes) → HUD bars → `wnoutrefresh + doupdate`.

8. **Handle input.** `getch()` loop. `w/s` adjust wind amplitude; `a/d` adjust frequency; `t/T` cycle themes.

## Non-Obvious Decisions

**Why reset BOTH `pos[r][0]` and `old_pos[r][0]` for anchor enforcement?**
Verlet computes implicit velocity as `vel = pos - old_pos`. If you reset only `pos[r][0] = anchor[r]`, then `old_pos[r][0]` still holds the displaced position from the Verlet step. On the next tick, `vel = anchor - displaced_old_pos`, which is non-zero and points away from the anchor. This "phantom velocity" yanks the particle off its anchor position on the very next tick, and the drift accumulates across iterations. Resetting both guarantees `vel = anchor - anchor = 0`. The particle is truly immovable.

**Why re-enforce anchors INSIDE the constraint loop (after each pass), not just after Verlet?**
The constraint solver uses equal-mass correction, meaning it is allowed to displace the anchor particle when fixing the (0,1) segment. Over 6 passes, these tiny displacements compound — the anchor drifts by a few pixels from its ceiling position. Re-pinning after every pass (not just once after all 6 passes) prevents this compound drift completely. The cost is 6 extra anchor assignments per tick: negligible.

**Why sinusoidal wind instead of random impulses?**
Random impulses (as in ragdoll_figure.c) create jittery, unpredictable sway. Sinusoidal force is: (a) smooth — no sudden jumps that cause constraint explosions at high wind amplitudes; (b) deterministic — the same amplitude/frequency always produces the same visual rhythm; (c) controllable — `wind_amp` and `wind_freq` map directly to physical intuition. The periodic nature also creates the characteristic "Mexican wave" when combined with phase offsets.

**Why `ROPE_DAMPING = 0.992` instead of `0.995` (as in ragdoll_figure.c)?**
At 60 Hz: `0.992^60 ≈ 0.619` (lose 38% speed per second) vs `0.995^60 ≈ 0.741` (lose 26% speed). Rope segments are thin and whippy — physically they have more air resistance than rigid bones. The slightly stronger damping prevents ropes from building up oscillation energy at high wind amplitudes and going unstable, while still showing visible sway at lower amplitudes.

**Why pre-compute `phase_offset[r] = r × 2π / N_ROPES` instead of computing it in `apply_wind()`?**
The formula is `sin(wind_time * wind_freq + phase_offset[r])`. If you compute `r × 2π / N_ROPES` inside `apply_wind()` every call, you do 7 floating-point multiplications and a division per tick for no benefit. Pre-computing at `scene_init()` time reduces it to 7 additions. More importantly, the pre-computation makes the formula in `apply_wind()` identical for every rope — making it obvious that the only per-rope difference is the phase offset.

**Why distribute 7 rope lengths from 35% to 75% of screen height?**
A shorter rope has a higher natural frequency (shorter pendulum swings faster). With identical lengths, all 7 ropes would have the same natural frequency and would lock into synchronised oscillation under sinusoidal wind — boring. Mixing lengths means each rope's natural frequency slightly differs from its neighbours, producing beating patterns and richer visual complexity. This is exactly how real wind chime sets are designed.

**Why use evenly spaced anchors with `(r + 1) × width / (N_ROPES + 1)`?**
Dividing by `(N_ROPES + 1)` rather than `N_ROPES` places equal margins on both sides. At 80 columns (640 px): anchors at 80, 160, 240, 320, 400, 480, 560 px — 80 px spacing, 80 px margin each side. Using `N_ROPES` as divisor would pin the leftmost rope flush against the left wall, which would be immediately clipped by `LEFT_MARGIN`.

**Why use `DRAW_STEP_PX = 2.0` for rope rendering (not `3.0` as in the FK snake)?**
Rope segments are often near-vertical (hanging straight down under gravity). For a perfectly vertical segment of length 21 px (typical rest_len), samples must be spaced less than `CELL_H = 16 px` apart to avoid skipping a terminal row. At 2 px steps, a vertical segment gets `ceil(21/2)+1 = 12` samples — enough to cover `21/16 = 1.3` rows. At 3 px: `ceil(21/3)+1 = 8` samples — still enough, but at very steep angles the margin shrinks. 2 px is the conservative choice for near-vertical chains.

**Why is `BOUNCE_COEFF = 0.5` for ropes (vs 0.55 for ragdoll)?**
Slightly more energy loss at the floor: a dangling rope tip hitting the floor should bounce lightly and settle quickly. The ragdoll's whole body needs a more lively bounce to produce visually interesting behavior; a rope tip that bounces less energetically looks more physically natural for a rope hitting a hard surface.

## State Machines

### Per-tick physics order (scene_tick)
```
┌────────────────────────────────────────────────────────────────┐
│ 1. memcpy(prev_pos ← pos)   [snapshot before physics]         │
│                                                                │
│ 2. if paused → return  (prev == curr → clean alpha freeze)    │
│                                                                │
│ 3. wind_time += dt                                             │
│                                                                │
│ 4. for each rope r:                                            │
│      apply_wind(r, dt):                                        │
│        wind_acc = wind_amp * sin(wind_time*wind_freq+phase[r]) │
│        for s = 1..19: rope_verlet_step(r, s, wind_acc, dt)     │
│        boundary clamp + bounce for s = 1..19                   │
│      enforce_anchors()  ← reset pos[r][0]=old_pos[r][0]=anchor │
│                                                                │
│ 5. apply_rope_constraints():                                   │
│      for iter in 0..5:                                         │
│        for each rope r, each segment s:                        │
│          correct pos[r][s] and pos[r][s+1]                     │
│        enforce_anchors()  ← re-pin after every pass            │
└────────────────────────────────────────────────────────────────┘
```

### Anchor state machine
```
Verlet step fires → pos[r][0] displaced by gravity + wind
    │
    ▼
enforce_anchors() → pos[r][0] = old_pos[r][0] = anchor[r]
    │                              ^^^^^^^^^^^^
    │                              BOTH reset — phantom vel = 0
    ▼
constraint pass may displace pos[r][0] slightly
    │
    ▼
enforce_anchors() again → pin restored
    │
    ▼ (repeated 6 times)
Result: anchor never drifts regardless of wind_amp or N_ITERS
```

### Rendering pipeline (render_scene)
```
Step 1: alpha lerp → rp[r][s] for all 7×20 particles
Step 2: ceiling '#' line across full terminal width
Step 3: for each rope r:
          draw_rope_beads(r)
            Pass 1: segment fill — walk each of 19 segments in 2px steps, stamp 'o'
            Pass 2: node markers — '0' (top quarter A_BOLD), 'o' (middle A_NORMAL),
                                   '.' (bottom quarter A_DIM)
            Tip marker: bright 'o' A_BOLD at particle[19]
```

### Node marker zones (N_SEG = 20)
| Particle index | Glyph | Attribute | Physical meaning |
|---------------|-------|-----------|-----------------|
| 0–4 (s < N_SEG/4 = 5) | `0` | A_BOLD | High tension — supports all weight below |
| 5–14 (middle half) | `o` | A_NORMAL | Medium tension — mid-chain |
| 15–19 (s >= N_SEG×3/4 = 15) | `.` | A_DIM | Low tension — slack dangling tip |
| 19 (tip) | `o` | A_BOLD | Weight marker — bright to suggest hanging mass |

### Input actions
| Key | Effect |
|-----|--------|
| q / Q / ESC | exit |
| space | toggle pause |
| w / UP | wind_amp += 50 px/s² (max 1000) |
| s / DOWN | wind_amp -= 50 px/s² (min 0) |
| d / RIGHT | wind_freq += 0.05 rad/s (max 4.0) |
| a / LEFT | wind_freq -= 0.05 rad/s (min 0.05) |
| t | next theme (0..9 wrapping) |
| T | previous theme |
| r / R | full reset (ropes to straight, theme preserved) |
| ] / + / = | sim_fps + 10 |
| [ / - | sim_fps − 10 |

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|----------|---------|-------------------|
| N_ROPES | 7 | More ropes → denser visual, but pairs cycle and colours repeat. Less ropes → sparser, each more visible. |
| N_SEG | 20 | More segments → smoother rope curves, more constraint iterations needed. Fewer → blocky, stiffer appearance. |
| N_ITERS | 6 | Decrease to 2–3 → visible segment stretch under strong wind. Increase → stiffer rope, more CPU. 6 is the minimum for invisible stretch at default settings. |
| ROPE_DAMPING | 0.992 | Decrease → more damping, ropes hang straighter, less lively. Increase toward 1.0 → less damping, ropes oscillate more; above ~0.998 can become unstable at high wind_amp. |
| WIND_AMP_DEFAULT | 250 px/s² | Increase → wider lateral sway. At 1000 px/s² the ropes swing nearly horizontal. 0 → gravity only, ropes hang straight. |
| WIND_FREQ_DEFAULT | 0.4 rad/s | Period = 2π/0.4 ≈ 15.7 s. Increase → faster oscillation (4.0 rad/s → 1.6 s period, rapid rattling). |
| BOUNCE_COEFF | 0.5 | 0 → dead stop at floor/walls. 1.0 → elastic. Higher values cause rope tips to bounce wildly. |
| ANCHOR_ROW_CELLS | 2 | Move ceiling line and anchor points; must stay above HUD row (0). |
| DRAW_STEP_PX | 2.0 | Must remain < CELL_W (8). Increase → gaps in near-horizontal segments. Decrease → denser fill, slight CPU cost. |

## Open Questions for Pass 3

- The constraint loop structure is: outer loop over iterations, inner loop over ropes, inner-inner loop over segments. An alternative is: outer loop over ropes, inner loop over iterations, then segments. The second structure might converge faster per-rope because all 6 passes run consecutively on one rope before moving to the next. Would this change the visual? Is there a theoretical reason the current structure (all ropes, one pass at a time) is preferred?
- `scene_init()` performs a full re-initialisation on SIGWINCH (resize). A more sophisticated approach would relocate existing particles proportionally. Why is full re-init justified here, but ragdoll_figure.c instead clamps individual particle positions?
- At `wind_freq = 4.0 rad/s` (max), the period is `2π/4 ≈ 1.57 s`. At `SIM_FPS = 60 Hz`, that is 94 ticks per cycle. The Nyquist rate for this frequency would require sampling at `> 2 × 4 / (2π)` Hz... wait, that is not the right framing. What is the minimum sim Hz needed to accurately represent a 4 rad/s sinusoid? At what sim Hz would the ropes start aliasing?
- The weight marker at the tip is always drawn `A_BOLD`, overriding the `A_DIM` that `rope_node_attr(19)` would normally return (since 19 >= `N_SEG*3/4 = 15`). Is this intentional? What is the visual effect of this override?
- Phase offsets are `r × 2π / N_ROPES` — uniform spacing around the full 2π cycle. An alternative would be random phase offsets. What would the visual difference be? Is there a scenario where random phases could accidentally produce near-synchronous ropes?
- `rest_len[r] = rope_len_px[r] / (N_SEG - 1)`. If the terminal is very short (e.g. 10 rows), `min_len = rows * CELL_H * 0.35 = 10 * 16 * 0.35 = 56 px`. With N_SEG = 20, `rest_len = 56 / 19 ≈ 2.95 px`. This is less than `DRAW_STEP_PX = 2.0`. Would the bead fill still work correctly at this step size vs segment length?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app` | `App` | ~40 KB | top-level container |
| `g_app.scene.pos[7][20]` | `Vec2[140]` | 1120 B | current particle positions |
| `g_app.scene.old_pos[7][20]` | `Vec2[140]` | 1120 B | previous positions (implicit velocity) |
| `g_app.scene.prev_pos[7][20]` | `Vec2[140]` | 1120 B | tick-start snapshot for alpha lerp |
| `g_app.scene.anchor[7]` | `Vec2[7]` | 56 B | fixed ceiling attachment points |
| `g_app.scene.phase_offset[7]` | `float[7]` | 28 B | pre-computed 2π phase fractions |
| `g_app.scene.rest_len[7]` | `float[7]` | 28 B | per-segment rest lengths |
| `THEMES[10]` | `Theme[10]` (global const) | ~400 B | 10 named colour palettes |

---

# Pass 2 — ragdoll_ropes: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | All tunables: N_ROPES, N_SEG, N_ITERS, GRAVITY, ROPE_DAMPING, BOUNCE_COEFF, WIND_*, DRAW_STEP_PX |
| §2 clock | `clock_ns()` / `clock_sleep_ns()` — verbatim from framework |
| §3 color | `Theme` struct, `THEMES[10]`, `theme_apply()`, `color_init()` — 10-palette system |
| §4 coords | `px_to_cell_x/y` — aspect-ratio bridge, called only at draw time |
| §5a rope_verlet_step | Verlet integration for one rope particle with ROPE_DAMPING |
| §5b apply_rope_constraints | N_ITERS passes of distance correction + anchor re-enforcement inside loop |
| §5c enforce_anchors | Reset pos[r][0] = old_pos[r][0] = anchor[r] for all ropes |
| §5d apply_wind | Compute sin-based lateral acceleration, call Verlet for all non-anchors, boundary bounce |
| §5e rope_node_char/attr | Size and brightness gradient functions for two-pass bead rendering |
| §5f draw_rope_beads | Two-pass bead render per rope (fill + node markers + tip weight marker) |
| §5g render_scene | Alpha lerp → ceiling line → all ropes |
| §6 scene | `scene_init` (lengths, anchors, phases) / `scene_tick` / `scene_draw` wrappers |
| §7 screen | ncurses layer (erase → draw → wnoutrefresh → doupdate) |
| §8 app | Signal handlers, resize (full scene_init on resize), main loop |

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
    ├── memcpy(prev_pos ← pos)            [snapshot before physics]
    │
    ├── wind_time += dt
    │
    ├── for r in 0..6:
    │     apply_wind(r, dt, cols, rows):
    │       wind_acc = wind_amp * sin(wind_time * wind_freq + phase_offset[r])
    │       for s in 1..19:
    │         rope_verlet_step(r, s, wind_acc, dt)
    │           vel = (pos-old_pos) * ROPE_DAMPING
    │           old_pos = pos
    │           pos = pos + vel + (wind_acc, GRAVITY) * dt²
    │       boundary clamp/bounce for s in 1..19
    │
    ├── enforce_anchors()                  [pos[r][0] = old_pos[r][0] = anchor[r]]
    │
    └── apply_rope_constraints():
          for iter in 0..5:
            for r in 0..6:
              for s in 0..18:
                dx,dy = pos[r][s+1] - pos[r][s]
                err = (|dx,dy| - rest_len[r]) / |dx,dy|
                correction = 0.5 * err * (dx, dy)
                pos[r][s] += correction
                pos[r][s+1] -= correction
            enforce_anchors()              [re-pin after each pass]

sim_accum -= tick_ns
    │
    ▼
alpha = sim_accum / tick_ns
    │
    ▼
render_scene(alpha)
    │
    ├── rp[r][s] = prev_pos + (pos - prev_pos) * alpha  [for all 7×20]
    │
    ├── ceiling '#' line at ANCHOR_ROW_CELLS
    │
    └── for r in 0..6:
          draw_rope_beads(r):
            pass 1: fill segments with 'o' at 2px steps
            pass 2: node markers ('0'/'o'/'.') at particle positions
            tip:    bright 'o' A_BOLD at particle[19]

screen_present() → doupdate()
```

---

## Function Breakdown

### rope_verlet_step(sc, r, s, wind_x, dt)
Purpose: advance one rope particle using Verlet integration
Steps:
1. `old = old_pos[r][s]`, `cur = pos[r][s]`
2. `vel_x = (cur.x - old.x) * ROPE_DAMPING`, same for y
3. `old_pos[r][s] = cur` — slide the window
4. `pos[r][s].x = cur.x + vel_x + wind_x * dt²`
5. `pos[r][s].y = cur.y + vel_y + GRAVITY * dt²`
Note: particle 0 (anchor) is NEVER passed here — only s = 1..N_SEG-1

---

### enforce_anchors(sc)
Purpose: pin all rope anchor particles to their ceiling positions
Steps:
1. For each rope r: `pos[r][0] = old_pos[r][0] = anchor[r]`
Critical: resetting BOTH pos and old_pos zeros the implicit velocity

---

### apply_rope_constraints(sc)
Purpose: run N_ITERS passes of distance correction for all ropes
Steps:
1. For `iter = 0..N_ITERS-1`:
   a. For each rope r, each segment s in 0..N_SEG-2:
      - `dx = pos[r][s+1].x - pos[r][s].x`, same for y
      - `dist = sqrt(dx²+dy²)`; guard if `dist < 1e-6`
      - `err = (dist - rest_len[r]) / dist`
      - `cx = 0.5*err*dx`, `cy = 0.5*err*dy`
      - `pos[r][s] += (cx, cy)`, `pos[r][s+1] -= (cx, cy)`
   b. Call `enforce_anchors()` — prevent per-pass drift

---

### apply_wind(sc, r, dt, cols, rows)
Purpose: compute sinusoidal wind for rope r and integrate into particles
Steps:
1. `wind_acc = wind_amp * sinf(wind_time * wind_freq + phase_offset[r])`
2. For s = 1..N_SEG-1: `rope_verlet_step(sc, r, s, wind_acc, dt)`
3. For s = 1..N_SEG-1: boundary clamp + old_pos reflection for floor, left wall, right wall
Note: no ceiling enforcement here (ropes hang down; they rarely reach the ceiling)

---

### rope_node_char(s) and rope_node_attr(s)
Purpose: select bead glyph and brightness based on position in rope
```
s < N_SEG/4  (= 5):        '0'  A_BOLD   (top quarter — high tension)
s >= N_SEG*3/4 (= 15):     '.'  A_DIM    (bottom quarter — low tension)
otherwise:                 'o'  A_NORMAL  (middle half)
```

---

### draw_rope_beads(sc, rp, r, w, cols, rows)
Purpose: two-pass bead render for one rope
Steps:
1. `cpair = (r % N_PAIRS) + 1`
2. Pass 1: for each segment (s, s+1):
   - `dx, dy = rp[r][s+1] - rp[r][s]`
   - Walk in `DRAW_STEP_PX = 2.0` increments with dedup cursor
   - Stamp `'o'` with `COLOR_PAIR(cpair) | A_NORMAL`
3. Pass 2: for each particle s in 0..N_SEG-1:
   - Stamp `rope_node_char(s)` with `rope_node_attr(s)`
4. Tip: stamp `'o' A_BOLD` at particle `N_SEG-1`

---

### scene_init(sc, cols, rows)
Purpose: initialise all ropes to straight hanging state
Steps:
1. Save `theme_idx`; `memset(sc, 0)`; restore `theme_idx`
2. `wind_amp = 250`, `wind_freq = 0.4`, `wind_time = 0`
3. Compute `screen_px_h`, `screen_px_w`
4. For each rope r:
   - `anchor[r].x = (r+1) * screen_px_w / (N_ROPES+1)` (even spacing with margin)
   - `anchor[r].y = ANCHOR_ROW_CELLS * CELL_H`
   - `rope_len_px[r] = min_len + r * (max_len - min_len) / (N_ROPES-1)`
   - `rest_len[r] = rope_len_px[r] / (N_SEG-1)`
   - `phase_offset[r] = r * 2π / N_ROPES`
   - Place particles straight down: `pos[r][s].y = anchor[r].y + s * rest_len[r]`
   - Set `old_pos = pos`, `prev_pos = pos` (zero initial velocity, clean first frame)

---

## Pseudocode — Core Loop

```
setup:
  srand(clock_ns())
  atexit(cleanup)
  signal(SIGINT/SIGTERM) → running = 0
  signal(SIGWINCH) → need_resize = 1
  screen_init()        ← color_init(theme=0)
  scene_init(cols, rows)
  frame_time = clock_ns(); sim_accum = 0

main loop (while running):

  1. if need_resize:
       screen_resize() → re-read LINES/COLS
       scene_init()    ← full re-init (simpler than relocating 140 particles)
       reset frame_time, sim_accum = 0

  2. dt = clock_ns() - frame_time; clamp at 100ms

  3. sim_accum += dt
     while sim_accum >= tick_ns:
       scene_tick(dt_sec, cols, rows)   ← Verlet + constraints
       sim_accum -= tick_ns

  4. alpha = sim_accum / tick_ns

  5. fps counter (500ms window)

  6. sleep(NS_PER_SEC/60 - elapsed) before render

  7. erase()
     render_scene(alpha)   ← ceiling + 7 ropes
     HUD top:   "ROPES 7x20 wind:X freq:Y [theme] fps Hz state"
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
 ├── owns Scene (all rope state)
 ├── reads sim_fps (adjustable via [/])
 └── drives the main loop

Scene / scene_tick:
 ├── memcpy prev_pos  ← snapshot anchor
 ├── apply_wind()     ← Verlet + boundary per rope
 ├── enforce_anchors() ← pin after Verlet
 └── apply_rope_constraints() ← 6 passes, re-pin inside

Rendering / render_scene:
 ├── alpha lerp all 7×20 particles
 ├── draw ceiling '#' line
 └── draw_rope_beads() × 7  ← two-pass fill + node markers

§4 coords (px_to_cell_x/y)
 └── called ONLY inside draw_rope_beads() — never in physics

§3 color (theme_apply)
 └── called at init and on t/T keypress — live palette swap
```

---

## Key Patterns to Internalize

**Anchor enforcement: both pos and old_pos:**
This is the single most important lesson from this file. In Verlet, velocity is implicit. Any position manipulation that forgets to also update `old_pos` leaves a phantom velocity. For anchors you want zero velocity: set both positions to the same value.

**Constraint passes with anchor re-enforcement inside:**
The standard position-based dynamics pipeline for a chain with a fixed anchor is: integrate → pin → [solve constraints → pin] × N. The inner pin prevents N-pass constraint drift accumulating into a visible anchor wobble.

**Phase offsets for independent sway:**
Distributing 7 phases evenly over 2π (`phase[r] = r × 2π / N_ROPES`) ensures no two ropes are at the same point in their oscillation cycle at any time. The "Mexican wave" is a mathematical guarantee, not a visual coincidence.

**Per-rope length variation for frequency diversity:**
Shorter ropes have higher natural frequencies. By linearly spacing lengths from 35% to 75% of screen height, the 7 ropes have 7 different natural periods, producing the complex beating patterns characteristic of a real wind-chime set rather than synchronised marching.

**Two-pass bead rendering:**
Pass 1 fills segment lines with `'o'` beads. Pass 2 overwrites particle positions with size-graded markers (`'0'`/`'o'`/`'.'`). Drawing node markers last ensures they always win over segment fill on overlapping cells — the structure reads clearly even when ropes cross.
