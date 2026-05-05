# Pass 1 — fk_tentacle_forest: Stateless sinusoidal FK for fixed-root chains — the simplest FK that produces organic motion

## From the Source

**Algorithm:** Stateless Forward Kinematics for fixed-root chains. Every tick, all joint positions are derived from a closed-form sine formula that accumulates local bend angles up the chain: `cumulative_angle += amplitude × sin(ω×t + root_phase + i×PHASE_PER_SEG)`. No historical state is required because re-evaluating the formula at the same wave_time yields identical positions. `prev_joint[]` is saved solely for sub-tick alpha interpolation at render time — not for simulation correctness.

**Math:** Per-segment bend: `δθᵢ = amplitude × sin((frequency + freq_offset) × wave_time + root_phase + i × PHASE_PER_SEG)`. World-space angle of segment i: `cumulative_angle = -π/2 + Σ δθⱼ (j=0..i)`. Joint position: `joint[i+1] = joint[i] + seg_len_px × (cos cumulative_angle, sin cumulative_angle)`. Segment length is dynamic: `seg_len_px = rows × CELL_H × 0.55 / N_SEGS`.

**Performance:** No trail buffer → O(N_TENTACLES × N_SEGS) per tick = 8 × 16 = 128 sin/cos evaluations at 60 Hz ≈ 7,680 trig calls/second. Trivially fast on any modern FPU. The fixed-step accumulator decouples physics from render Hz. `ncurses doupdate()` sends only changed cells — typically 200–400 per frame for swaying tentacles.

**Data-structure:** `Tentacle`: fixed root position (`root_px`, `root_py`), per-strand `root_phase` (stagger) and `freq_offset` (slight detuning), plus `joint[]` and `prev_joint[]` arrays of Vec2 pixel positions. `Scene`: `N_TENTACLES=8` Tentacle entities, shared `wave_time`, `amplitude`, `frequency`, and dynamically computed `seg_len_px`.

## Core Idea

Eight seaweed strands grow up from the sea floor and sway left and right in a simulated underwater current. Every frame, each joint's position is computed fresh from a sine formula — nothing from the previous tick is used for simulation. The formula accumulates local bend angles up the chain: the root bends a little, then the next joint bends a little more relative to the root's bend, and so on to the tip. The per-segment phase offset makes each higher joint slightly "behind" the wave, creating the appearance of a wave travelling up the strand from root to tip, just as a real elastic medium transmits a travelling wave.

The chain is the minimal FK system: fixed root, pure math, no memory beyond the running clock. Understanding this demo fully is the prerequisite for understanding trail-buffer FK (snake, centipede body), inverse kinematics, and spring chains.

## The Mental Model

Imagine a flagpole in a light breeze. The pole bends at the base; the bend propagates up and the tip swings the most. That is the total accumulated bending. Now imagine the flagpole is segmented, and each segment's local bend oscillates with the wind — but each segment's wind phase is slightly delayed relative to the one below. The wave of bending travels upward from the base to the tip. That is `PHASE_PER_SEG`.

The key insight: the world-space angle of segment i is not just its own local bend — it is the sum of all local bends from segment 0 up through segment i. If the first five segments all bent 2° to the right, and your segment is segment 6, your direction is at least 10° to the right before your own local bend is added. This is the "accumulation property" of Forward Kinematics.

Each strand has a different `root_phase` (starts at a different position in the wave cycle) and a slightly different `freq_offset` (oscillates at a slightly different period). Without these, all 8 strands would sway in perfect lockstep — mechanical and clearly synthetic. With them, they drift in and out of sync organically, like real seaweed in a gentle current.

## Data Structures

### Vec2
```
x, y   — pixel space (float); x = eastward, y = downward (terminal convention)
```

### Tentacle (per-strand state)
```
root_px, root_py     fixed sea-floor anchor in pixel space
root_phase           phase offset: stagger over [0, 2π) for desync
freq_offset          tiny frequency variation (rad/s): prevents long-term re-sync
joint[N_SEGS + 1]   pixel positions; [0] = root (always = root_px,root_py)
                     [N_SEGS] = tip (free end, highest point)
prev_joint[N_SEGS+1] snapshot from end of previous tick; used for alpha lerp only
```

### Scene (full simulation state)
```
t[N_TENTACLES]   array of 8 Tentacle entities
wave_time        monotonic simulation clock (s); the sole input to FK formula
amplitude        peak bend per segment (rad); user-adjustable via → / ←
frequency        base oscillation rate (rad/s); user-adjustable via ↑ / ↓
seg_len_px       rigid segment length (px); computed from terminal height at init
                 formula: rows × CELL_H × 0.55 / N_SEGS
                 recalculated on every SIGWINCH resize
paused           when true, wave_time is frozen (FK recomputes same positions)
```

### Screen
```
cols, rows   terminal dimensions; updated after each resize
```

### App
```
scene         simulation state
screen        ncurses dimensions
sim_fps       physics tick rate Hz; user-adjustable via [ / ]
running       volatile sig_atomic_t: 0 = exit
need_resize   volatile sig_atomic_t: 1 = SIGWINCH pending
```

## The Main Loop

Each iteration:

1. **Resize check.** If `need_resize` is set, call `app_do_resize()`: re-read terminal dimensions, save `wave_time/amplitude/frequency`, call `scene_init()` (repositions roots for new width, recomputes `seg_len_px` for new height), restore saved wave state so animation continues without a jerk. Reset `frame_time` and `sim_accum`.

2. **Measure dt.** Read monotonic clock; subtract `frame_time`. Cap at 100 ms. Update `frame_time`.

3. **Physics accumulator.** Add dt to `sim_accum`. While `sim_accum >= tick_ns`: advance `wave_time += dt` (if not paused), call `compute_fk_chain()` for all 8 tentacles, subtract `tick_ns`.

4. **Compute alpha.** `alpha = sim_accum / tick_ns ∈ [0, 1)`.

5. **FPS counter.** Sliding 500 ms window average.

6. **Frame cap.** Sleep before render to target 60 fps.

7. **Draw and present.** `erase()` → draw all 8 tentacles (2 passes each) + seabed '~' row → HUD and hint bar → `wnoutrefresh()` + `doupdate()`.

8. **Drain input.** Loop `getch()` until `ERR`.

## Non-Obvious Decisions

**Why no trail buffer when roots are fixed?**
In snake_forward_kinematics.c the root (head) moves freely through space. Each body joint must follow the actual path the head carved — that path is encoded in the trail buffer. Here, `root_px` and `root_py` never change. Computing the same FK formula twice with the same `wave_time` is algebraically identical: the same output. There is no path history to store. The trail buffer would be redundant and consume 32 KB per tentacle.

**Why cumulative angle accumulation rather than absolute angles per joint?**
If each joint had an absolute world-space angle derived independently, there would be no "wave propagates through the chain" effect. The base could bend left while the tip points right with no continuous curve in between. By accumulating local bends into a running sum, each joint's direction inherits all the bending from below it. The result is a smooth, continuous curve — the chain acts as an elastic medium transmitting force from base to tip.

**Why `cumulative_angle = -π/2` as the starting angle?**
In terminal pixel space, y increases downward. The mathematical angle for "pointing straight up" is −π/2 (cos(−π/2) = 0, sin(−π/2) = −1, so the joint moves upward in pixel space, i.e. in the negative-y direction). Starting at 0 would make the tentacle initially point rightward (east). Starting at −π/2 makes an unperturbed tentacle (amplitude = 0) stand perfectly vertical — the correct rest posture.

**Why `PHASE_PER_SEG = 0.45 rad` specifically?**
At 0.0, all segments bend in phase → the tentacle tilts rigidly left and right like a rod, with no S-curve. At π/2 ≈ 1.57, adjacent segments are a quarter-wave apart → very tight zigzag. At 0.45 ≈ 26°, 16 segments span `16 × 0.45 = 7.2 rad ≈ 1.15 cycles`. The chain displays slightly more than one full wave from root to tip, producing a gentle, believable S-curve. This is the biologically realistic regime for seaweed in a mild current.

**Why `freq_offset = (i − N_TENTACLES/2) × 0.04 rad/s`?**
After one full oscillation period (≈7.9 s for FREQ_DEFAULT=0.8), adjacent strands drift apart by `0.04 × 7.9 ≈ 0.32 rad ≈ 18°`. This is enough to prevent visible re-synchronization over a multi-minute viewing period, while keeping individual strands close enough in frequency to appear in the "same current". Larger offsets would make strands look like they are in completely different environments.

**Why `root_py = rows × CELL_H − 4.0 px`?**
The seabed '~' row is at terminal row `rows − 2`. In pixel space that row spans `(rows−2)×CELL_H` to `(rows−1)×CELL_H − 1`. Placing the root 4 px above the absolute bottom (`rows × CELL_H − 4`) puts it inside this seabed row visually, so each tentacle appears to grow out of the seabed texture rather than from an invisible point above it.

**Why `seg_len_px = rows × CELL_H × 0.55 / N_SEGS` rather than 0.50?**
A swaying tentacle traces an arc, which is longer than the straight vertical line from root to tip. A straight-up tentacle of length `0.50 × screen_height` would reach exactly mid-screen only when perfectly vertical — with any sway amplitude, the tip would be visually lower than mid-screen. The 0.55 factor compensates: the geometric arc length is 10% longer than the chord at typical amplitude, so the tip reaches approximately mid-screen even when swaying.

**Why is `wave_time` preserved across resize in `app_do_resize`?**
On resize, `scene_init()` is called to reposition roots and recompute `seg_len_px`. Without saving and restoring `wave_time`, every resize would reset the animation to t=0 — tentacles would snap back to their straight-up rest posture and restart their cycle. Preserving `wave_time` makes the animation continue exactly where it was, with all 8 strands at the correct phase for that moment in time.

**Why does `compute_fk_chain` run even when paused?**
When paused, `wave_time` is not advanced. The FK formula, given the same `wave_time`, produces the same output as the previous tick. After the internal `memcpy(prev_joint, joint)`, the new `joint[]` is identical to `prev_joint[]`. The lerp in `render_tentacle` at any alpha value returns `prev + 0 × alpha = prev`. The animation freezes cleanly. If `compute_fk_chain` was skipped when paused, `prev_joint` would hold the last-computed positions but `joint` would hold whatever was there from before the pause — creating a potential mismatch on unpausing.

**Why does `draw_segment_dense` clip `cy < 1` and `cy >= rows-1` instead of `cy < 0` and `cy >= rows`?**
Rows 0 and `rows-1` are reserved for the HUD top bar and the hint bar. Drawing tentacle glyphs there would overwrite the status display. The exclusive range `[1, rows-2]` is the drawable content area. The tentacle tips can physically reach row 0 on tall terminals if the screen is narrow (many segments, large `seg_len_px`); silently skipping those cells is cleaner than clamping the FK computation.

**Why right-align the HUD bar?**
Tentacle tips can extend into any row including row 0 (they are clipped in the segment renderer, but the HUD is drawn after `erase()` and after `scene_draw()`). Right-aligning keeps the HUD in the top-right corner, away from the tallest tentacle tips which tend to cluster near their roots (left and right sides) rather than at the far right edge.

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
        │                                          │
        │  SIGINT/SIGTERM ────────→ running = 0 │
        └───────────────────────────────────────┘
                          │
               'q'/ESC/signal
                          │
                          v
                        EXIT
                      (endwin)
```

### Tentacle oscillation cycle (single strand)
```
wave_time = 0:       joint[0] at root, chain points straight up (cumul = -π/2)
wave_time = π/(2ω):  all segments at peak positive bend → tip displaced right
wave_time = π/ω:     all back near vertical (but phase offsets create S-curve)
wave_time = 3π/(2ω): all segments at peak negative bend → tip displaced left
wave_time = 2π/ω:    full cycle complete, back to start

(FREQ_DEFAULT = 0.8 rad/s → period = 2π/0.8 ≈ 7.85 s)
```

### Input actions
| Key | Effect |
|-----|--------|
| q / Q / ESC | running = 0 → exit |
| space | toggle scene.paused |
| ↑ / w / W | frequency + 0.15 rad/s (clamped to [FREQ_MIN, FREQ_MAX]) |
| ↓ / s / S | frequency − 0.15 rad/s |
| → / d / D | amplitude + 0.10 rad (clamped to [AMP_MIN, AMP_MAX]) |
| ← / a / A | amplitude − 0.10 rad |
| ] | sim_fps + 10 (clamped to [10, 120]) |
| [ | sim_fps − 10 |

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|----------|---------|-------------------|
| N_TENTACLES | 8 | Number of seaweed strands. More → denser forest; roots are at i/(N+1) fractions so spacing is automatic. |
| N_SEGS | 16 | Segments per strand. More → smoother curvature, more FK evaluations. 16 gives a smooth S-curve with one peak-trough visible. |
| seg_len_px | dynamic | Computed from screen height. Increasing 0.55 factor raises tips higher; decreasing leaves tips in lower half of screen. |
| PHASE_PER_SEG | 0.45 rad | Wave propagation speed up chain. 0 = all-rigid rod. π/2 = tight zigzag. 0.45 = one gentle S-curve over 16 segments. |
| AMP_DEFAULT | 0.28 rad | Peak bend per segment. Too high (>0.8): tentacle curls into spiral. Too low (<0.05): barely moves. |
| AMP_MAX | 1.2 rad | Maximum user-adjustable amplitude. At 1.2 the tentacle makes tight, dramatic coils. |
| FREQ_DEFAULT | 0.8 rad/s | Base sway frequency. Period ≈ 7.9 s — meditative, seaweed-like pace. |
| FREQ_MAX | 5.0 rad/s | At 5.0 (period ≈ 1.3 s) the strands flicker rapidly — turbulent current look. |
| DRAW_STEP_PX | 5 px | Glyph stamp stride. Must be < CELL_W (8 px). Smaller → denser fill, more iterations. |
| freq_offset | ±0.04/strand | Per-strand detuning in scene_init. Larger → faster desync; strands look more independent. Smaller → strands drift back into sync after many cycles. |
| root phase spacing | 2π/N | Evenly distributes strands around [0, 2π). Ensures maximum initial desync. |
| FPS_UPDATE_MS | 500 ms | HUD fps refresh interval. Shorter → flickery display; longer → stale number. |

## Open Questions for Pass 3

- `cumulative_angle` is a sum of floating-point sine values accumulated across 16 segments over potentially hours of simulation (wave_time grows monotonically). Does single-precision float accumulation drift? At what wave_time value would `sinf(0.8 × wave_time + phase)` start to lose precision and produce visible artifacts?
- With `PHASE_PER_SEG = 0.45` and `N_SEGS = 16`, the total phase span is `7.2 rad ≈ 1.15 cycles`. What does the tentacle look like at exactly `AMP_MAX = 1.2 rad/segment`? Would the accumulated angle cause the tip to point downward (re-entering the screen below the root)?
- The `freq_offset` is computed at `scene_init` as `(i − N_TENTACLES × 0.5) × 0.04`. If the user resizes the terminal and `app_do_resize` preserves `wave_time` but calls `scene_init` (which recomputes `freq_offset` identically since it depends only on index i), does the saved `wave_time` + fresh roots produce a seamless continuation or a phase discontinuity?
- `draw_segment_dense` uses a shared `prev_cx/prev_cy` cursor across all N_SEGS segments of one tentacle. The cursor is initialised to −9999. What happens if the tentacle's root is at cell (−9999/CELL_W, −9999/CELL_H)? Is this possible on any real terminal?
- When `amplitude = 0.0` (minimum), `delta = 0` for every segment, and `cumulative_angle` stays at exactly `−π/2`. Every tentacle points straight up. Does `draw_segment_dense` still produce a visible line, or does the zero-movement make it degenerate? What does `seg_glyph(0, −seg_len_px)` return for a perfectly vertical segment?
- `seg_attr` returns `A_DIM` for the root quarter, `A_BOLD` for the tip quarter. On an 8-color terminal where color pairs 1–5 are all `COLOR_BLUE`, do `A_DIM | COLOR_BLUE` and `A_BOLD | COLOR_BLUE` actually render differently, or are they identical?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app` | `App` | ~20 KB | top-level container |
| `g_app.scene.t[8]` | `Tentacle[8]` | ~4.4 KB | 8 strands; each has joint[17] + prev_joint[17] = 34 Vec2 = 272 B per strand |
| `g_app.scene` | `Scene` | ~4.5 KB | tentacle array + wave params + seg_len_px |
| `THEMES[]` | (none) | — | fk_tentacle_forest has no theme switching; single fixed color palette |

# Pass 2 — fk_tentacle_forest: Pseudocode

## Module Map

| Section | Purpose |
|---------|---------|
| §1 config | All tunables: N_TENTACLES, N_SEGS, PHASE_PER_SEG, AMP_DEFAULT/MIN/MAX, FREQ_DEFAULT/MIN/MAX, DRAW_STEP_PX, timing, CELL_W/H |
| §2 clock | `clock_ns()` — monotonic timestamp; `clock_sleep_ns()` — sleep |
| §3 color | Fixed 7-step deep-sea palette (deep blue root → bright yellow-green tip). 8-color fallback. `color_init()` — one-time setup; no theme switching |
| §4 coords | `px_to_cell_x/y` — single aspect-ratio conversion |
| §5a stateless FK | `compute_fk_chain(t, wave_time, amplitude, frequency, seg_len_px)` — the core algorithm |
| §5b render helpers | `seg_pair(i)` — linear root→tip color index; `seg_attr(i)` — DIM/NORMAL/BOLD by zone; `seg_glyph(dx,dy)` — direction char |
| §5c draw_segment_dense | Dense glyph stamper with dedup + clip |
| §5d render_tentacle | Two-pass: alpha lerp → seg lines → joint node markers |
| §6 scene | `Scene` struct; `scene_init()` — root distribution, phase assignment; `scene_tick()` — advance wave_time + FK; `scene_draw()` — all tentacles + seabed |
| §7 screen | ncurses setup, HUD composition, double-buffer flush |
| §8 app | Signal handlers, resize with wave-state preservation, main loop |

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
scene_tick(dt, cols, rows)
    │
    ├── if not paused: wave_time += dt   ← single shared clock for all strands
    │
    └── for each tentacle i:
          compute_fk_chain(t[i], wave_time, amplitude, frequency, seg_len_px)
                │
                ├── memcpy(prev_joint, joint)   ← alpha anchor FIRST
                ├── joint[0] = (root_px, root_py)
                ├── cumulative_angle = -π/2
                └── for seg i in [0, N_SEGS):
                      δθ = amplitude × sin((freq + freq_offset) × wave_time
                                           + root_phase + i × PHASE_PER_SEG)
                      cumulative_angle += δθ
                      joint[i+1] = joint[i] + seg_len_px × (cos/sin cumul_angle)

sim_accum -= tick_ns
    │
    ▼
alpha = sim_accum / tick_ns   [0.0 .. 1.0)
    │
    ▼
scene_draw(sc, w, cols, rows, alpha)
    │
    ├── for each tentacle i:
    │     render_tentacle(t[i], w, cols, rows, alpha)
    │           │
    │           ├── Step 1: rj[k] = lerp(prev_joint[k], joint[k], alpha)
    │           ├── Step 2 (pass 1): for seg 0..N_SEGS-1:
    │           │     draw_segment_dense(rj[k], rj[k+1], seg_pair(k), seg_attr(k))
    │           └── Step 3 (pass 2): for joint 0..N_SEGS:
    │                 stamp '#' / 'O' / 'o' / '.' / '*' marker at rj[k]
    │
    └── seabed row: '~' chars in dim pair 1 at row (rows-2)

    ▼
screen_present() → doupdate()   [ONE atomic write to terminal]
```

---

## Function Breakdown

### compute_fk_chain(t, wave_time, amplitude, frequency, seg_len_px)
Purpose: recompute all 17 joint positions for one tentacle from scratch
Steps:
1. `memcpy(prev_joint, joint, sizeof joint)` — snapshot before overwriting
2. `joint[0] = (root_px, root_py)` — anchor root (explicit, no drift)
3. `cumulative_angle = -π/2` — pointing straight up
4. For i = 0 .. N_SEGS - 1:
   - `delta = amplitude × sinf((frequency + freq_offset) × wave_time + root_phase + i × PHASE_PER_SEG)`
   - `cumulative_angle += delta`
   - `joint[i+1].x = joint[i].x + seg_len_px × cosf(cumulative_angle)`
   - `joint[i+1].y = joint[i].y + seg_len_px × sinf(cumulative_angle)`
Why cumulative: world-space direction of segment i = sum of all local bends from 0 to i. Each += propagates all upstream bends downstream.

---

### seg_pair(i) → int
Purpose: color pair for segment i, root (pair 1) to tip (pair N_PAIRS=7)
Formula: `1 + (i × (N_PAIRS - 1)) / (N_SEGS - 1)`
Example at N_SEGS=16: i=0→1, i=5→3, i=15→7
Guard: if N_SEGS <= 1, return 1 (avoid division by zero)

### seg_attr(i) → attr_t
Purpose: brightness attribute by chain position
Returns:
- `A_DIM` if i < N_SEGS/4 (root zone: dark, deep water)
- `A_BOLD` if i > 3×N_SEGS/4 (tip zone: bright, shallow water)
- `A_NORMAL` otherwise (mid-body: neutral)

### seg_glyph(dx, dy) → chtype
Purpose: best-matching ASCII direction char for vector (dx,dy)
Steps:
1. `ang = atan2f(-dy, dx)` — negate dy: terminal (y-down) → math (y-up) convention
2. `deg = ang × 180/π`; if < 0: `deg += 360`
3. Fold to [0°, 180°): if `deg >= 180°`: `deg -= 180°`
4. if < 22.5° or >= 157.5°: return `'-'`; if < 67.5°: return `'\\'`; if < 112.5°: return `'|'`; else return `'/'`

---

### draw_segment_dense(w, a, b, pair, attr, cols, rows, *prev_cx, *prev_cy)
Purpose: stamp direction glyph at every terminal cell line a→b crosses
Steps:
1. dx = b.x−a.x; dy = b.y−a.y; len = √(dx²+dy²); if len < 0.1 return
2. glyph = seg_glyph(dx, dy)
3. nsteps = ceil(len / DRAW_STEP_PX) + 1
4. For s = 0 .. nsteps:
   - u = s / nsteps
   - cx = px_to_cell_x(a.x + dx × u); cy = px_to_cell_y(a.y + dy × u)
   - Dedup: if cx == *prev_cx && cy == *prev_cy: continue
   - *prev_cx = cx; *prev_cy = cy
   - Clip: if cx < 0 || cx >= cols || cy < 1 || cy >= rows-1: continue (HUD rows excluded)
   - mvwaddch(cy, cx, glyph) with pair+attr
Note: prev_cx/prev_cy are shared across all segments of one tentacle to prevent double-stamping at joint boundaries.

---

### render_tentacle(t, w, cols, rows, alpha)
Purpose: draw one tentacle chain with two-pass compositor
Steps:
1. Build alpha-lerped render positions:
   `rj[k].x = prev_joint[k].x + (joint[k].x − prev_joint[k].x) × alpha` for k=0..N_SEGS
2. Pass 1 — segment fill (root to tip, bright tips win overlaps on curled tentacles):
   - `prev_cx = -9999; prev_cy = -9999` (shared dedup cursor)
   - For k = 0 .. N_SEGS−1: draw_segment_dense(w, rj[k], rj[k+1], seg_pair(k), seg_attr(k), ...)
3. Pass 2 — joint node markers (always on top of fill):
   - For k = 0 .. N_SEGS:
     - cx = px_to_cell_x(rj[k].x); cy = px_to_cell_y(rj[k].y)
     - Clip: same [1, rows-2] range
     - p = seg_pair(k < N_SEGS ? k : N_SEGS-1)
     - marker: k==0 → '#'; k <= N_SEGS/4 → 'O'; k <= 3×N_SEGS/4 → 'o'; k < N_SEGS → '.'; k==N_SEGS → '*'
     - mvwaddch(cy, cx, marker) with A_BOLD

---

### scene_init(sc, cols, rows)
Purpose: distribute roots, assign phases, seed joint positions
Steps:
1. seg_len_px = rows × CELL_H × 0.55 / N_SEGS
2. screen_wpx = cols × CELL_W; root_py = rows × CELL_H − 4.0
3. For i = 0 .. N_TENTACLES−1:
   - root_px = (i+1) × screen_wpx / (N_TENTACLES+1)   [fraction i/(N+1), no edge roots]
   - root_phase = i × 2π / N_TENTACLES   [evenly over [0, 2π)]
   - freq_offset = (i − N_TENTACLES×0.5) × 0.04   [detuning: −0.16..+0.12 rad/s for 8 strands]
   - Seed joints straight up: joint[k] = (root_px, root_py − k × seg_len_px)
   - memcpy(prev_joint, joint)   [first-frame lerp is no-op]

---

### scene_tick(sc, dt, cols, rows)
Purpose: one physics step for all tentacles
Steps:
1. If not paused: wave_time += dt
2. For i = 0 .. N_TENTACLES−1: compute_fk_chain(t[i], wave_time, amplitude, frequency, seg_len_px)
Note: wave_time advances BEFORE FK so all tentacles see this tick's time (not last tick's).
Note: when paused, wave_time unchanged → FK outputs same positions → prev_joint == joint → lerp freezes cleanly.

---

### scene_draw(sc, w, cols, rows, alpha)
Purpose: render all tentacles + seabed atmosphere (pure read; no simulation state changes)
Steps:
1. For i = 0 .. N_TENTACLES−1: render_tentacle(t[i], w, cols, rows, alpha)
2. Seabed row: if (rows−2) >= 1: fill row (rows−2) with '~' in pair 1 A_DIM

---

### app_do_resize(app)
Purpose: handle SIGWINCH; preserve wave phase
Steps:
1. screen_resize() — re-read terminal dimensions
2. Save: wave_time, amplitude, frequency
3. scene_init(cols, rows) — recomputes roots + seg_len_px, reseeds joints
4. Restore: wave_time, amplitude, frequency
5. need_resize = 0
After return: main() resets frame_time and sim_accum to prevent physics avalanche.

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
       app_do_resize(app)   ← saves wave state, re-roots, restores wave state
       frame_time = clock_ns()
       sim_accum = 0

  ② dt:
     now = clock_ns()
     dt = now − frame_time
     frame_time = now
     cap dt at 100ms

  ③ fixed-step accumulator:
     tick_ns = NS_PER_SEC / sim_fps
     dt_sec = tick_ns / NS_PER_SEC (float)
     sim_accum += dt
     while sim_accum >= tick_ns:
       scene_tick(dt_sec, cols, rows):
         if not paused: wave_time += dt_sec
         for each tentacle: compute_fk_chain(...)
       sim_accum -= tick_ns

  ④ alpha:
     alpha = sim_accum / tick_ns   [0.0 .. 1.0)

  ⑤ fps counter:
     frame_count++; fps_accum += dt
     if fps_accum >= 500ms: fps_display = count/elapsed; reset

  ⑥ frame cap:
     elapsed = clock_ns() − frame_time + dt
     sleep(NS_PER_SEC/60 − elapsed)   ← BEFORE render

  ⑦ draw + present:
     erase()
     scene_draw(alpha):
       for each tentacle: render_tentacle (2-pass: lines then nodes)
       seabed row of '~'
     HUD bar (right-aligned) + hint bar (left-aligned)
     wnoutrefresh() + doupdate()

  ⑧ drain input:
     while (ch = getch()) != ERR: dispatch key

cleanup:
  endwin()
```

---

## Interactions Between Modules

```
App
 ├── owns Screen (ncurses state, cols/rows)
 ├── owns Scene (8 tentacles, shared wave params)
 ├── reads sim_fps ([ / ] keys)
 └── main loop drives everything

Scene (tick order per physics step):
  1. wave_time += dt (if not paused)
  2. for each tentacle: compute_fk_chain()
       ├── saves prev_joint internally FIRST
       ├── pins joint[0] to fixed root
       └── accumulates cumulative_angle × N_SEGS iterations

render_tentacle() [called N_TENTACLES times per render frame]:
  reads: prev_joint[], joint[], alpha
  writes: ncurses newscr ONLY
  pass 1: draw_segment_dense() for each of N_SEGS segments
  pass 2: mvwaddch() at each of N_SEGS+1 joint positions

§4 coords (px_to_cell_x/y):
  called ONLY inside draw_segment_dense() and render_tentacle() pass 2
  never called during physics

§2 clock:
  called only in main() — physics and render never touch the clock

§3 color / color_init():
  called once at startup (no theme switching in this demo)
  pairs 1–7: deep blue → yellow-green gradient for tentacles
  pair 8: bright yellow for HUD

Signal handlers:
  SIGINT/SIGTERM → running = 0
  SIGWINCH → need_resize = 1
  both volatile sig_atomic_t

app_do_resize():
  called from main() when need_resize flag is set
  saves wave_time/amplitude/frequency
  calls scene_init() (full re-initialization of geometry)
  restores wave_time/amplitude/frequency
  frame_time and sim_accum reset by main() after this returns
```

---

## Key Patterns to Internalize

**Stateless FK — the formula IS the state:**
Every joint position is a function of `wave_time` alone (plus fixed per-strand constants). The simulation has no memory: throw away all joint arrays, reconstruct with the same `wave_time`, get identical results. `prev_joint` is only needed for the render interpolation trick — not for simulation correctness. This is the defining property of stateless FK.

**Cumulative angle accumulation — the FK inheritance property:**
World-space direction at segment i = sum of all local bends from segment 0 through i. Each `cumulative_angle += delta` is one application of this principle. If you see an FK loop that accumulates an angle, this is what it is doing: propagating the "inherited curvature" from root to tip.

**Phase-per-segment — making a wave travel through a chain:**
`i × PHASE_PER_SEG` in the sine argument means each segment is slightly later in the wave cycle than the one below it. The root leads, the tip follows. This is how physical waves travel through elastic media — and 0.45 rad/segment is the sweet spot for biologically plausible seaweed-like motion.

**Per-strand detuning — preventing re-synchronization:**
`freq_offset = (i − N/2) × 0.04` gives each strand a slightly different oscillation period. Without this, even with different initial `root_phase` values, all strands would gradually re-align into lockstep synchrony over time (because they are all driven by the same `wave_time`). The detuning ensures they permanently drift independently.

**Resize preserving wave state:**
When geometry changes (terminal resize), physics time must not reset. Save `wave_time`, reinitialize geometry, restore `wave_time`. It is the correct way to handle resize in any stateless-FK demo.
