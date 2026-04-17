# Pass 1 — fk_centipede: Two FK mechanisms in one demo — trail-buffer path-following for the body, stateless sinusoidal FK for coordinated legs

## From the Source

**Algorithm:** The body uses path-following FK — the head's trail buffer encodes positional history; each joint is placed at a fixed arc-length distance along that history via linear interpolation. Legs use stateless sinusoidal FK: all joint positions are computed analytically from `wave_time` and per-leg phase offsets, producing a coordinated gait without any explicit gait state machine.

**Math:** Leg gait phase: `phi_L = wave_time × GAIT_FREQ + i × (π / N_LEGS)` for leg pair i (left), `phi_R = phi_L + π` (right, contralateral antiphase). Upper leg angle: `body_dir ± LEG_SPLAY + SWING_AMP × sin(phi)`. Lower leg angle: `upper_angle + LEG_BEND + LOWER_SWING × sin(phi + π/4)`. Arc-length interpolation: `trail_sample(dist)` walks the circular trail accumulating Euclidean distances until the target arc-length is reached, then linearly interpolates between bounding entries.

**Performance:** Fixed-step accumulator decouples physics from render Hz. Trail buffer is pre-populated at init so the body is fully extended from frame one. `ncurses doupdate()` sends only changed cells to the terminal.

**Data-structure:** Circular trail buffer `Vec2 trail[TRAIL_CAP]` (4096 entries × 8 bytes = 32 KB). Joint arrays `joint[BODY_SEGS+1]` and `prev_joint[BODY_SEGS+1]` for body alpha-lerp. Leg positions `leg_left[N_LEGS][3]` and `leg_right[N_LEGS][3]` (hip=0, knee=1, foot=2) with `prev_` counterparts for alpha-lerp.

## Core Idea

A 16-segment centipede crawls across the terminal. The body uses a circular trail buffer: the head carves a path and each body joint follows it exactly, placed at a fixed arc-length step along the recorded path. The legs are a completely different mechanism — no history needed, just wave math: each leg's hip, knee, and foot are computed from a single formula involving the current time and a per-leg phase offset. The phase offset creates a travelling wave of leg steps rolling from head to tail, which is how real centipedes coordinate their many legs.

## The Mental Model

Imagine a train on a curved track. The engine (head) steers freely; each carriage (body joint) just follows the same track, always the same distance behind the one in front of it. That is the trail-buffer FK body: the "track" is the head's recorded path, and `trail_sample(i × SEG_LEN_PX)` picks the point on that track exactly `i` segment-lengths behind the head.

Now imagine the train has legs. But instead of using the track history to decide leg position, each leg just does a calculation: "what time is it, and which leg am I?" — and that is enough to place the hip, bend the knee, and extend the foot. If leg 0 is stepping forward, leg 3 is mid-stride, and leg 7 is pushing off. The `i × π/N_LEGS` phase offset is the key — it staggers each leg by a fixed fraction of the gait cycle, producing the rolling wave seen in real arthropods.

The contralateral antiphase (`phi_R = phi_L + π`) means the right leg of each pair is exactly half a cycle behind the left. When left is swinging forward, right is pushing backward. At any moment exactly half the legs are in ground contact, maintaining stable support while the other half swing.

## Data Structures

### Vec2
```
x, y   — position in pixel space (float, eastward / downward)
```
Used for every position in the simulation. No cell coordinates anywhere in the physics layer.

### Centipede (complete simulation state)
```
trail[TRAIL_CAP]          circular head-position history (Vec2, cap = 4096)
trail_head                write cursor (index of newest entry)
trail_count               valid entries (0 → TRAIL_CAP)

joint[BODY_SEGS + 1]      body joint positions, pixel space; [0] = head
prev_joint[BODY_SEGS + 1] snapshot before each tick, for alpha lerp

leg_left[N_LEGS][3]       left legs: [i][0]=hip [i][1]=knee [i][2]=foot
leg_right[N_LEGS][3]      right legs: same layout
prev_leg_left[N_LEGS][3]  snapshot for alpha lerp
prev_leg_right[N_LEGS][3]

heading                   current travel direction in radians
wave_time                 accumulated sim time (s), drives all oscillations
move_speed                head translation speed (px/s)
turn_amp                  peak sinusoidal turn rate (rad/s)
turn_freq                 body undulation frequency (rad/s)
theme_idx                 index into THEMES[10]
paused                    physics frozen when true
```

### Scene
```
centipede   — the single Centipede entity
```
Thin wrapper matching the framework convention; `scene_tick` and `scene_draw` delegate directly into the centipede.

### App
```
scene         — simulation state
screen        — terminal dimensions (cols, rows)
sim_fps       — physics tick rate (Hz), user-adjustable
running       — volatile sig_atomic_t: 0 = exit
need_resize   — volatile sig_atomic_t: 1 = SIGWINCH pending
```

## The Main Loop

Each iteration:

1. **Resize check.** If `need_resize` is set (SIGWINCH), call `screen_resize()`, clamp the head pixel position to the new bounds, clear `sim_accum` and reset `frame_time` so the large dt from the resize pause does not cause a physics avalanche.

2. **Measure dt.** Read monotonic clock; subtract `frame_time`. Cap at 100 ms (suspend guard). Update `frame_time`.

3. **Physics accumulator.** Compute `tick_ns = NS_PER_SEC / sim_fps`. Accumulate dt into `sim_accum`. While `sim_accum >= tick_ns`: call `scene_tick(dt_sec, cols, rows)`, subtract `tick_ns`. Physics fires at exactly `sim_fps` Hz regardless of render timing.

4. **Compute alpha.** `alpha = sim_accum / tick_ns ∈ [0, 1)` — the sub-tick interpolation factor.

5. **FPS counter.** Count frames over a 500 ms sliding window; compute smoothed fps every `FPS_UPDATE_MS` ms.

6. **Frame cap.** Sleep `(NS_PER_SEC/60 − elapsed)` before render. Caps render at ~60 Hz without burning CPU.

7. **Draw and present.** `erase()` → `scene_draw()` → HUD bars → `wnoutrefresh()` → `doupdate()`. One atomic diff write to the terminal.

8. **Drain input.** Loop `getch()` until `ERR`, dispatching every queued key.

## Non-Obvious Decisions

**Why trail-buffer FK for the body but stateless FK for the legs?**
The body needs to follow the actual path the head carved. If the centipede turns tightly, joints 10–15 must be inside the curve they went through two seconds ago — not at some analytically-computed position that might place them outside the turn. The trail buffer stores the exact path; `trail_sample` retrieves it. Legs, however, need no path history. They just need to know which direction the body is facing at their attachment point right now, and what phase of the gait cycle they are in. A pure math formula is sufficient, cheaper, and produces the correct biological behaviour.

**Why `i × (π / N_LEGS)` as the phase step between leg pairs?**
With 10 leg pairs, `π / 10 = 0.314 rad` per pair. Over 10 pairs, the total phase span is π radians — exactly half a wave cycle. This means the front-most and rear-most legs are always π out of phase, and the intermediate legs form a smooth stagger. The result is a travelling wave of leg activity that propagates from head to tail, matching the metachronal wave of real myriapod locomotion.

**Why contralateral antiphase (`phi_R = phi_L + π`)?**
If left and right legs were in phase, both legs of a pair would lift simultaneously, and the centipede would have no support from that pair during the swing phase. With `phi_R = phi_L + π`, when the left leg is mid-swing, the right leg is mid-stance, and vice versa. At every instant, at least half of all legs are in contact with the ground — the defining property of a stable alternating gait.

**Why TRAIL_CAP = 4096 when only ~480 px of trail is needed?**
The body is 24 × 20 px = 480 px long. At minimum speed (10 px/s) and 60 Hz, the head moves 10/60 ≈ 0.167 px per tick. To store 480 px of trail at that density requires 480 / 0.167 ≈ 2876 entries. 4096 > 2876, providing a safe margin with room to spare. If the trail buffer ran out, `trail_sample` would return the oldest entry — the tail would "freeze" at the last known position instead of following the path.

**Why pre-populate the trail with TRAIL_CAP entries at init?**
Without pre-fill, `trail_count = 0` initially. `trail_sample` at any distance returns the head position, so all body joints would start at the same pixel as the head — a single dot. The tail would grow out over the first ~7 seconds (480 px at 65 px/s). Pre-filling with 1 px steps along the starting heading gives a fully-extended body from frame one.

**Why save `prev_joint` before the paused check in `scene_tick`?**
If paused and `prev_joint` was not updated, the lerp on the next frame would use stale prev values from the moment the simulation was running. When unpaused, a single tick would snap the body from an old position to the new one — a visible jump. Saving before the paused check ensures prev == curr during pause; the lerp at any alpha produces a clean freeze with no stutter.

**Why DRAW_STEP_PX = 5 for the dense body renderer?**
The body is drawn by stepping along each segment every `DRAW_STEP_PX` pixels and stamping a direction glyph. 5 px is less than `CELL_W = 8 px`, so the step is guaranteed to visit every terminal column the segment passes through, leaving no visible gaps. Making it smaller (e.g. 2 px) would double-stamp many cells but produce no visible improvement; larger (e.g. 9 px) would skip some columns.

**Why two-pass rendering (lines then node markers)?**
Pass 1 walks each segment and stamps direction glyphs (`-`, `\`, `|`, `/`). Pass 2 stamps bold node markers (`@`, `O`, `o`, `.`, `*`) at each joint position. If these were merged, segment lines drawn after a joint would overwrite its marker. The two-pass order guarantees markers always appear on top of the segment fill, giving the body its "chain of beads connected by angled lines" appearance.

**Why does `seg_glyph` fold the angle to [0°, 180°)?**
A line glyph is rotationally symmetric under 180° rotation — `/` going up-right looks the same as `/` going down-left. By folding `atan2` result to `[0°, 180°)` using `if (deg >= 180°) deg -= 180°`, both directions map to the same glyph, halving the number of cases.

**Why `body_seg_pair` maps i=0 to pair N_PAIRS (brightest) and i=BODY_SEGS-1 to pair 1 (dimmest)?**
The head is index 0, the tail is index `BODY_SEGS-1`. The amber theme has pair 7 = bright red (most vivid) and pair 1 = dark brown (dimmest). Mapping head → brightest, tail → dimmest creates a natural gradient where the head draws the eye and the tail recedes visually — matching how real arthropods appear.

## State Machines

### App-level state
```
        ┌───────────────────────────────────────┐
        │              RUNNING                  │
        │                                       │
        │  [physics ticks] ←── accumulator     │
        │  [render + present] each frame        │
        │  [input dispatch]                     │
        │                                       │
        │  SIGWINCH ──────────────→ need_resize │
        │       need_resize ──────→ app_do_resize│
        │                                       │
        │  SIGINT/SIGTERM ────────→ running = 0 │
        └───────────────────────────────────────┘
                          │
               'q'/ESC/signal
                          │
                          v
                        EXIT
                      (endwin)
```

### Per-leg gait cycle
```
         phi = 0                  phi = π               phi = 2π
left:   SWING START ──────────→ STANCE START ──────────→ SWING START
right:  STANCE START ─────────→ SWING START ──────────→ STANCE START

At any moment: left.phi + π = right.phi
Half the legs (one from each pair) always in contact with ground.
```

### Input actions
| Key | Effect |
|-----|--------|
| q / Q / ESC | running = 0 → exit |
| space | toggle centipede.paused |
| ↑ / ↓ | move_speed × or ÷ 1.25 (clamped to [MOVE_SPEED_MIN, MOVE_SPEED_MAX]) |
| ← / → or a / d | turn_freq − or + 0.15 rad/s (undulation frequency) |
| w / s | turn_amp + or − 0.10 rad (curve width) |
| t / T | cycle theme_idx (mod N_THEMES = 10), call theme_apply() |
| [ / ] | sim_fps − or + SIM_FPS_STEP (clamped to [10, 120]) |

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|----------|---------|-------------------|
| BODY_SEGS | 24 | Number of body segments. More → longer centipede, larger trail buffer needed. 24 × 20 = 480 px total length. |
| N_LEGS | 10 | Number of leg pairs. More → denser legs, smaller per-pair phase step (π/N_LEGS). |
| SEG_LEN_PX | 20.0 px | Distance between body joints. Shorter → tighter body, sharper curves visible. Must keep TRAIL_CAP × speed × dt > BODY_SEGS × SEG_LEN_PX. |
| TRAIL_CAP | 4096 | Circular buffer capacity. Must be ≥ BODY_SEGS × SEG_LEN_PX / (MOVE_SPEED_MIN / SIM_FPS). |
| GAIT_FREQ | 2.0 rad/s | Leg oscillation frequency. Higher → legs cycle faster relative to body movement. |
| MOVE_SPEED | 65 px/s | Head translation speed. Higher → centipede crosses screen faster; gait cycle covers more ground per stride. |
| TURN_AMP | 0.52 rad/s | Peak sinusoidal turn rate. Higher → tighter curves, more coiling. 0 = straight line. |
| TURN_FREQ | 0.95 rad/s | Body undulation frequency. Higher → S-curve repeats more rapidly. |
| UPPER_LEN | 14 px | Hip-to-knee segment length. Longer → larger leg reach, more visible foot travel arc. |
| LOWER_LEN | 12 px | Knee-to-foot length. Shorter than UPPER_LEN → bent-knee appearance. |
| SWING_AMP | 0.4 rad | Upper leg fore-aft swing per half-cycle. > 1.0 → legs clip through body. |
| LEG_SPLAY | 1.2 rad | Base angle between body axis and upper leg. At π/2 legs point straight out; 1.2 tilts them slightly forward. |
| LEG_BEND | -0.5 rad | Static knee-bend offset. Negative = bent downward for insect-like resting posture. |
| BODY_OFFSET | 8 px | Lateral distance from body centerline to hip. = CELL_W exactly = 1 terminal column. |
| DRAW_STEP_PX | 5 px | Step size for dense segment renderer. Must be < CELL_W (8 px) to avoid skipping columns. |
| N_THEMES | 10 | Number of selectable color themes. |

## Open Questions for Pass 3

- If MOVE_SPEED is increased to 300 px/s (MOVE_SPEED_MAX), the head moves 5 px per tick at 60 Hz. How many trail entries does `trail_sample(480)` need to walk? Does the dedup cursor in `draw_line_dense` cause any visible gaps at that speed?
- `body_seg_pair` uses integer division `i * (N_PAIRS-1) / (BODY_SEGS-1)`. For BODY_SEGS=24, N_PAIRS=7: some color buckets cover 4 segments, others 3. Does this create a visible "step" in the gradient, or is the segment-to-segment color change smooth enough to be imperceptible?
- The phase offset per leg pair is `i × π / N_LEGS`. With N_LEGS=10 and BODY_SEGS=24, one leg pair attaches every ~2.4 body segments. Does the discrete attachment interval create a visual mismatch between the body undulation (driven by TURN_FREQ=0.95) and the leg gait wave (driven by GAIT_FREQ=2.0)?
- `compute_legs` uses `body_dir = atan2f(fwd_vec.y, fwd_vec.x)` derived from neighboring joints. At very low MOVE_SPEED, two adjacent joints may occupy the same pixel, making `fwd_vec` nearly zero and `body_dir` undefined. What does `vec2_norm` return for a near-zero vector, and how does it affect leg placement?
- When the centipede wraps toroidally (exits one edge, enters the other), a body segment near the wrap point has neighbors on opposite sides of the screen. What does `trail_sample` return for joints that straddle the wrap boundary? Is there a one-frame visual glitch as the body snaps across?
- The trail is pre-populated with entries 1 px apart along the initial heading. But `trail_head = 0` and the write cursor starts there. On the first push, does `trail_push` overwrite entry 0 with the actual head position, displacing the first pre-populated entry? Is this correct?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app` | `App` | ~260 KB | top-level container: scene + screen + control flags |
| `g_app.scene.centipede.trail[TRAIL_CAP]` | `Vec2[4096]` | 32 KB | circular head-position history for body FK |
| `g_app.scene.centipede.joint[25]` | `Vec2[25]` | 200 B | current body joint positions (head + 24 segments) |
| `g_app.scene.centipede.prev_joint[25]` | `Vec2[25]` | 200 B | previous tick snapshot for alpha lerp |
| `g_app.scene.centipede.leg_left[10][3]` | `Vec2[30]` | 240 B | left leg hip/knee/foot positions |
| `g_app.scene.centipede.leg_right[10][3]` | `Vec2[30]` | 240 B | right leg hip/knee/foot positions |
| `THEMES[10]` | `Theme[10]` | ~800 B | named color palettes, read-only |

# Pass 2 — fk_centipede: Pseudocode

## Module Map

| Section | Purpose |
|---------|---------|
| §1 config | All tunables: BODY_SEGS, N_LEGS, TRAIL_CAP, SEG_LEN_PX, GAIT_FREQ, MOVE_SPEED, TURN_AMP/FREQ, leg geometry, timing constants, CELL_W/H |
| §2 clock | `clock_ns()` — monotonic nanosecond timestamp; `clock_sleep_ns()` — precise sleep |
| §3 color | `Theme` struct; `THEMES[10]` palette array; `theme_apply(idx)` — live re-init of ncurses pairs; `color_init()` — one-time setup |
| §4 coords | `px_to_cell_x/y` — the one aspect-ratio conversion point |
| §5a trail | `trail_push()` — circular buffer write; `trail_at(k)` — read k steps back; `trail_sample(dist)` — arc-length interpolation |
| §5b move_head | Advance wave_time, integrate sinusoidal turn into heading, translate, toroidal wrap, push to trail |
| §5c compute_joints | Sample trail at i × SEG_LEN_PX for each body joint |
| §5d compute_legs | Stateless FK for all 10 leg pairs: attachment, body direction, normals, gait phase, upper/lower angles, joint positions |
| §5e render helpers | `body_seg_pair/attr()` — gradient + brightness; `seg_glyph()` — direction char; `head_glyph()` — directional arrow; `draw_line_dense()` — dense segment renderer with dedup |
| §5f render_centipede | Two-pass frame compositor: alpha lerp → legs → body lines → body nodes → head arrow |
| §6 scene | `Scene` wraps Centipede; `scene_init/tick/draw` wrappers |
| §7 screen | ncurses setup, HUD composition, double-buffer flush |
| §8 app | Signal handlers, resize, main game loop |

---

## Data Flow Diagram

```
CLOCK_MONOTONIC
    │
    ▼
clock_ns() → dt (nanoseconds)
    │
    ▼
sim_accum += dt
    │
    while sim_accum >= tick_ns:
    │
    ▼
scene_tick(dt_sec, cols, rows)
    │
    ├── save prev_joint[], prev_leg_*[]   ← interpolation anchors
    │
    ├── move_head(dt, cols, rows)
    │       ├── wave_time += dt
    │       ├── heading += turn_amp × sin(turn_freq × wave_time) × dt
    │       ├── joint[0] += move_speed × (cos heading, sin heading) × dt
    │       ├── toroidal wrap
    │       └── trail_push(joint[0])
    │
    ├── compute_joints()
    │       └── joint[i] = trail_sample(i × SEG_LEN_PX)  for i=1..BODY_SEGS
    │
    └── compute_legs()
            for each leg pair i:
              body_idx = 1 + i × (BODY_SEGS−2)/(N_LEGS−1)
              body_dir = atan2(joint[body_idx−1] − joint[body_idx+1])
              hip_L = joint[body_idx] + BODY_OFFSET × (cos(body_dir+π/2), sin(...))
              hip_R = joint[body_idx] + BODY_OFFSET × (cos(body_dir−π/2), sin(...))
              phi_L = wave_time × GAIT_FREQ + i × (π/N_LEGS)
              phi_R = phi_L + π
              upper_L/R: body_dir ± LEG_SPLAY + SWING_AMP × sin(phi)
              lower_L/R: upper + LEG_BEND + LOWER_SWING × sin(phi + π/4)
              knee, foot: hip + UPPER_LEN×(cos,sin), knee + LOWER_LEN×(cos,sin)

sim_accum -= tick_ns
    │
    ▼
alpha = sim_accum / tick_ns   [0.0 .. 1.0)
    │
    ▼
render_centipede(c, w, cols, rows, alpha)
    │
    ├── Step 1: rj[i] = lerp(prev_joint[i], joint[i], alpha)
    ├── Step 2: rl_left/right[i][j] = lerp(prev_leg_*[i][j], leg_*[i][j], alpha)
    ├── Step 3: draw legs (behind body)
    │       for each leg: draw_line_dense(hip→knee), draw_line_dense(knee→foot)
    │       foot tip markers: '*' (left), 'x' (right) in pair 5 A_BOLD
    ├── Step 4a: body segment lines tail→head (shared dedup cursor)
    │       draw_line_dense(rj[i+1], rj[i]) with body_seg_pair/attr(i)
    ├── Step 4b: body joint node markers tail→head
    │       '@'=head, 'O'=near-head, 'o'=mid, '.'=near-tail, '*'=tail-tip
    │       all A_BOLD so markers dominate fill chars
    └── Step 5: head arrow pair 7 A_BOLD (always on top)

    ▼
screen_present() → doupdate() [ONE atomic write to terminal]
```

---

## Function Breakdown

### trail_push(c, pos)
Purpose: append head position to circular buffer
Steps:
1. `trail_head = (trail_head + 1) % TRAIL_CAP`
2. `trail[trail_head] = pos`
3. Increment trail_count up to TRAIL_CAP

### trail_at(c, k) → Vec2
Purpose: retrieve k steps back from newest
Steps:
1. Return `trail[(trail_head + TRAIL_CAP − k) % TRAIL_CAP]`
Why add TRAIL_CAP before subtraction: avoids negative modulo result in C.

### trail_sample(c, dist) → Vec2
Purpose: arc-length interpolation at distance dist from head
Steps:
1. `a = trail_at(c, 0)` (newest = current head)
2. For k = 1 .. trail_count:
   - `b = trail_at(c, k)`
   - `seg = |b − a|`
   - If `accum + seg >= dist`: `t = (dist − accum) / seg`; return `a + t × (b − a)`
   - `accum += seg`; `a = b`
3. Trail exhausted: return oldest entry
Edge cases: if seg < 1e-4, use 1e-4 as divisor to avoid division by zero at slow speeds.

---

### move_head(c, dt, cols, rows)
Purpose: advance one physics step for the head
Steps:
1. `wave_time += dt`
2. `turn = turn_amp × sin(turn_freq × wave_time)` → `heading += turn × dt`
3. `joint[0] += (move_speed × cos(heading), move_speed × sin(heading)) × dt`
4. Toroidal wrap: if `x < 0`: `x += wpx`; if `x >= wpx`: `x -= wpx`; same for y
5. `trail_push(joint[0])`

---

### compute_joints(c)
Purpose: place all body joints from trail
Steps:
1. For i = 1 .. BODY_SEGS: `joint[i] = trail_sample(c, i × SEG_LEN_PX)`
Note: joint[0] is set by move_head; this only updates 1..BODY_SEGS.

---

### compute_legs(c)
Purpose: stateless FK for all N_LEGS=10 leg pairs
For each pair i (0=front, 9=rear):
1. body_idx = 1 + i × (BODY_SEGS−2) / (N_LEGS−1)  [clamped to 1..BODY_SEGS-1]
2. fwd_vec = normalize(joint[body_idx−1] − joint[body_idx+1])
3. body_dir = atan2(fwd_vec.y, fwd_vec.x)
4. left_normal = body_dir + π/2; right_normal = body_dir − π/2
5. hip_L = joint[body_idx] + BODY_OFFSET × (cos/sin left_normal)
   hip_R = joint[body_idx] + BODY_OFFSET × (cos/sin right_normal)
6. phi_L = wave_time × GAIT_FREQ + i × (π / N_LEGS)
   phi_R = phi_L + π
7. upper_L = body_dir + LEG_SPLAY + SWING_AMP × sin(phi_L)
   upper_R = body_dir − LEG_SPLAY + SWING_AMP × sin(phi_R)
   lower_L = upper_L + LEG_BEND + LOWER_SWING × sin(phi_L + π/4)
   lower_R = upper_R + LEG_BEND + LOWER_SWING × sin(phi_R + π/4)
8. knee_L = hip_L + UPPER_LEN × (cos upper_L, sin upper_L)
   foot_L = knee_L + LOWER_LEN × (cos lower_L, sin lower_L)
   knee_R, foot_R: same with right angles
9. Store leg_left[i][0..2] = (hip_L, knee_L, foot_L); same for right

---

### draw_line_dense(w, a, b, pair, attr, cols, rows, *prev_cx, *prev_cy)
Purpose: stamp direction glyph at every cell the line a→b passes through
Steps:
1. dx = b.x − a.x; dy = b.y − a.y; len = |ab|
2. glyph = seg_glyph(dx, dy)
3. nsteps = ceil(len / DRAW_STEP_PX) + 1
4. For t = 0 .. nsteps:
   - u = t / nsteps; sample = a + u × (dx, dy)
   - cx = px_to_cell_x(sample.x); cy = px_to_cell_y(sample.y)
   - If cx == *prev_cx && cy == *prev_cy: continue (dedup)
   - *prev_cx = cx; *prev_cy = cy
   - If out of bounds: continue
   - mvwaddch(cy, cx, glyph) with pair+attr

---

### render_centipede(c, w, cols, rows, alpha)
Purpose: compose complete centipede frame
Steps:
1. Compute rj[i] = lerp(prev_joint[i], joint[i], alpha) for 0..BODY_SEGS
2. Compute rl_left/right[i][j] = lerp(prev_leg_*[i][j], leg_*[i][j], alpha)
3. Draw legs (each gets independent dedup cursor, initialised to −9999):
   - Left: draw_line_dense hip→knee (pair 3, DIM); knee→foot (pair 3, DIM); '*' foot tip
   - Right: draw_line_dense hip→knee (pair 4, DIM); knee→foot (pair 4, DIM); 'x' foot tip
4a. Body segment lines tail→head (single shared dedup cursor):
   - For i = BODY_SEGS−1 down to 0: draw_line_dense(rj[i+1], rj[i], body_seg_pair(i), body_seg_attr(i))
4b. Body joint node markers tail→head (separate pass, no dedup needed):
   - For j = BODY_SEGS down to 0: stamp '@'/'O'/'o'/'.'/'*' in A_BOLD
5. Head arrow: head_glyph(heading) in pair 7 A_BOLD at rj[0]

---

## Pseudocode — Core Loop

```
setup:
  atexit(cleanup)
  signal(SIGINT/SIGTERM) → running = 0
  signal(SIGWINCH) → need_resize = 1
  screen_init()
  scene_init(cols, rows)
  frame_time = clock_ns()
  sim_accum = 0

main loop (while running):

  ① resize:
     if need_resize:
       screen_resize()
       clamp head pixel position to new bounds
       reset frame_time = clock_ns(), sim_accum = 0
       need_resize = 0

  ② dt:
     now = clock_ns()
     dt = now − frame_time
     frame_time = now
     cap dt at 100ms  ← suspend guard

  ③ physics accumulator:
     tick_ns = NS_PER_SEC / sim_fps
     dt_sec = tick_ns / NS_PER_SEC
     sim_accum += dt
     while sim_accum >= tick_ns:
       scene_tick(dt_sec, cols, rows):
         save prev_joint, prev_leg_left, prev_leg_right
         if not paused:
           move_head(dt_sec, cols, rows)
           compute_joints()
           compute_legs()
       sim_accum -= tick_ns

  ④ alpha:
     alpha = sim_accum / tick_ns   [0.0 .. 1.0)

  ⑤ fps counter:
     frame_count++; fps_accum += dt
     if fps_accum >= 500ms: update fps_display, reset counters

  ⑥ frame cap:
     elapsed = clock_ns() − frame_time + dt
     sleep(NS_PER_SEC/60 − elapsed)   ← sleep BEFORE render

  ⑦ draw + present:
     erase()
     render_centipede(alpha)   ← legs → body lines → body nodes → head arrow
     HUD + hint bar
     wnoutrefresh() + doupdate()   ← ONE atomic diff write

  ⑧ drain input:
     while (ch = getch()) != ERR:
       dispatch key → adjust speed/freq/amp/theme/sim_fps/pause/quit

cleanup:
  endwin()
```

---

## Interactions Between Modules

```
App
 ├── owns Screen (ncurses state, cols/rows)
 ├── owns Scene → Centipede (trail, joints, legs, motion params)
 ├── reads sim_fps (adjustable by '[' / ']')
 └── main loop drives everything

Scene / Centipede (tick order per physics step):
  1. save prev_joint[], prev_leg_*[]  ← BEFORE any mutation
  2. move_head()
       uses: wave_time, heading, move_speed, turn_amp, turn_freq
       writes: wave_time, heading, joint[0], trail[]
  3. compute_joints()
       reads: trail[], trail_head, trail_count
       writes: joint[1..BODY_SEGS]
  4. compute_legs()
       reads: joint[], wave_time, GAIT_FREQ, all leg geometry constants
       writes: leg_left[], leg_right[]

render_centipede() (draw order, painter's algorithm):
  reads: prev_joint[], joint[], prev_leg_*[], leg_*[], heading
  reads: alpha (from accumulator)
  writes: ncurses newscr only (no simulation state mutation)
  draw order: legs (back) → body lines → body node markers → head (front)

§4 coords (px_to_cell_x/y):
  called ONLY inside render_centipede() / draw_line_dense()
  never called during physics update

§2 clock:
  called only in main() loop
  physics and render code never read the clock directly

Signal handlers:
  write running = 0 (SIGINT/SIGTERM)
  write need_resize = 1 (SIGWINCH)
  both volatile sig_atomic_t — safe from signal context
```

---

## Key Patterns to Internalize

**Trail-buffer FK vs stateless FK — when to use each:**
Use trail-buffer FK for chains whose roots move freely through space (body, snake, rope, tail). The trail stores the actual path. Use stateless FK for chains with fixed roots or whose shape is fully determined by a formula (legs, fins, tentacles, antennae). No history needed; the formula is the state.

**Contralateral antiphase in multi-limb locomotion:**
`phi_R = phi_L + π` is the minimal constraint that ensures stability — at every moment, at least half the legs are on the ground. This generalizes: for n legs per side, spacing phases by `2π/n` gives maximum desynchronization with minimum support gaps.

**Two-pass rendering for line+node chains:**
Always draw segment fill lines first, then joint node markers. The second pass guarantees nodes are never overwritten by segment lines. This pattern appears in fk_centipede, fk_tentacle_forest, and fk_medusa — it is a codebase convention.

**Arc-length interpolation for path-following FK:**
`trail_sample(dist)` gives the correct body joint position regardless of head speed. If the head slowed down, trail entries are denser in that region, and the arc-length walk naturally finds the correct back-along-the-path position. Speed changes do not cause joint spacing to change.

**Pre-save of prev arrays before paused check:**
Saving `prev_joint` before the `if paused return` ensures a clean freeze. If you save after the paused check, unpausing would produce a one-frame jump from the stale prev position.
