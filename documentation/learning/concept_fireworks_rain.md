# Pass 1 — fireworks_rain.c: Fireworks with matrix-rain arc trails

## Core Idea

Takes the standard fireworks rocket state machine (rise → apex → burst) and replaces each simple spark with a `Spark` that carries a `TRAIL_LEN = 16` position history. Each frame the particle's current position is pushed into the trail before physics advance. At draw time, the trail is rendered as a chain of shimmering matrix-rain glyphs that follows the exact arc the particle traces through the air — the familiar downward parabola of a fading firework ember, made of jumping ASCII characters.

The simulation runs on **variable dt** (no fixed-step accumulator) and uses **physical units**: velocities in cells/sec, gravities in cells/sec², fuse times in seconds. A learner can predict and verify behaviour with a stopwatch ("slow rocket explodes ~1 sec after launch about 5 rows up; spark lifetimes 0.3–1.3 sec").

## The Mental Model

Three independent ideas stacked on top of each other.

**(A) ROCKET** — a 3-state machine. IDLE counts down a fuse (in seconds). RISING climbs under gravity until vy ≥ 0 (apex) or y < 2 (top edge). EXPLODED ticks all sparks until none remain, then returns to IDLE with a fresh random fuse.

**(B) SPARK** — one explosion particle with a head position + velocity, a 16-slot trail history of past head positions, and a parallel cache of random ASCII glyphs. Each frame: slide history, integrate physics, shimmer cache, decay life.

**(C) SHIMMER** — for each cache slot, with probability 1 − 1/SHIMMER_KEEP_ONE_IN, reroll the glyph. KEEP_ONE_IN = 4 → 75 % rerolled each frame, the classic Matrix-rain rate.

```
ASCII sketch of one spark in flight:

     head (white, bold)
      ↓
      A   ← cache[0]   (newest, BOLD, TRAIL_HOT band)
       q  ← cache[1]   BOLD
        W ← cache[2]   BOLD
         z              normal (TRAIL_WARM band)
          7             normal
           R            DIM   (TRAIL_COOL band)
            e           DIM
             %          DIM   (oldest)
```

## Data Structures

### Spark (§4.2)
```
Vec2  head;             — head position in cells (float)
Vec2  vel;              — velocity in cells/sec
Vec2  trail[16];        — history; [0] = newest, [N-1] = oldest
int   trail_fill;       — 0..16 (ramps up over first 16 frames)
char  cache[16];        — random ASCII glyphs, 75% reroll per frame
float life;             — 1.0 (fresh) → 0.0 (dead)
float decay_rps;        — life drop per second; per-spark variance
int   color;            — theme-dependent CP_xxx pair
bool  active;           — false once life ≤ 0
```

### Rocket (§5.1)
```
float        x, y;
float        vy;        — vertical velocity (cells/sec, negative = up)
int          color;     — body hue while RISING
RocketState  state;     — RS_IDLE / RS_RISING / RS_EXPLODED
float        fuse_sec;  — seconds remaining before launch (RS_IDLE only)
Spark        particles[72];
```

### Show (§6)
```
Rocket  rockets[16];
int     active_rockets;
float   speed_scale;    — global multiplier on dt ([ / ] keys)
bool    paused;
```

## Brightness bands (§1.5)

```
i in [0..TRAIL_HOT_END=2]      → BOLD   (hot, near-head)
i in [3..TRAIL_WARM_END=8-1]   → NORMAL (mid-fade)
else (deep tail)               → DIM
```

`life < FADING_LIFE_THRESHOLD = 0.25` overrides everything to DIM (death fade).

## State Machine

```
  ┌──────┐  fuse_sec ≤ 0    ┌────────┐  vy ≥ 0  OR    ┌──────────┐
  │ IDLE │ ────────────────►│ RISING │────────────────│ EXPLODED │
  └──────┘                  └────────┘  y < 2 row     └──────────┘
     ▲                                                      │
     │  all sparks dead → uniform(FUSE_MIN..MIN+VAR) sec    │
     └──────────────────────────────────────────────────────┘
```

The state machine is split into one named function per state — `rocket_tick_idle`, `rocket_tick_rising`, `rocket_tick_exploded` — plus a 5-line dispatcher. Each handler is its own readable paragraph.

## The Main Loop (variable dt)

1. Resize check.
2. `dt` = wall-clock seconds since last frame, capped at `DT_CAP_SEC`.
3. Drain input.
4. `show_tick(dt)`:
   - If paused, return.
   - `scaled = dt * speed_scale`.
   - For each active rocket: dispatch on state.
5. fps rolling average.
6. Draw: erase, `show_draw` (per-active-rocket), HUD, hint, present.
7. Frame cap.

## Per-spark step (`spark_tick`)

```
if (!active) return
spark_advance_trail(p)        # slide trail[i] = trail[i-1]; trail[0] = head
spark_integrate(p, dt)        # head += vel · dt; vel.y += g · dt (with jitter)
spark_shimmer(p)              # reroll cache (75% per frame)
life -= decay_rps · dt        # life decay
if life ≤ 0: active = false
```

## Non-Obvious Decisions

**Why drop fixed-step + alpha?**
Same reason as siblings: simulation is non-stiff, variable-dt with float positions is unconditionally stable, and removing the accumulator drops ~30 lines.

**Why physical units (cells/sec, cells/sec²)?**
The old code had `ROCKET_SPEED_SCALE = 6.0` and `SPARK_SPEED_SCALE = 8.0` magic multipliers on top of the velocity values. With physical units, `head += vel · dt` is just kinematics — no scale factor, no mental conversion. A learner can compute "rocket reaches apex in `|vy| / GRAVITY` seconds and that distance × 0.5 cells up" and verify.

**Why fuse in seconds?**
Old code used fuse in TICKS, decremented per frame. With variable dt, that meant fuse advances at different real-time rates depending on frame rate. Fuse in seconds with `fuse_sec -= dt` makes fuse progression frame-rate-independent.

**Why trail-shift in REVERSE order?**
`for (i = N-1; i > 0; i--) trail[i] = trail[i-1]` runs backward so each cell's old value is read into the next slot before it gets overwritten. Forward iteration would smear the newest position across the entire buffer.

**Why per-state functions instead of one big switch?**
Each `rocket_tick_idle`, `rocket_tick_rising`, `rocket_tick_exploded` is 4-7 lines, named after its state. A learner can study each state's logic in isolation. The dispatcher `rocket_tick` is then 5 lines of pure dispatch.

**Why dim-first painter's order in `spark_draw`?**
Near burst centres, multiple trail slots round to the same cell. Drawing dim slots first lets the bright head + near-head slots overwrite them at the contested cells. Reverse order = head hidden behind its own dim tail.

## Key Constants

| Constant | Default | Effect |
|---|---|---|
| TARGET_FPS | 60 | Render cap. |
| ROCKETS_DEFAULT | 5 | Active rockets at startup. `+`/`-` adjust 1..16. |
| PARTICLES_PER_BURST | 72 | Sparks per explosion. |
| TRAIL_LEN | 16 | Trail length per spark. |
| ROCKET_LAUNCH_VY_MIN/MAX_CPS | -18, -48 | Launch speed range (cells/sec, negative=up). |
| ROCKET_GRAVITY_CPS2 | 30 | Cells/sec² pull on rockets. |
| SPARK_SPEED_MIN/MAX_CPS | 12, 40 | Burst speed range. |
| SPARK_GRAVITY_CPS2 | 32 | Cells/sec² pull on sparks. |
| SPARK_GRAVITY_JITTER | 0.20 | ±20% gravity variance per spark. |
| SPARK_LIFE_MIN/VAR | 0.6, 0.4 | Initial life range. |
| SPARK_DECAY_MIN/VAR_RPS | 0.75, 1.05 | Life-drop rate per second. |
| BURST_ANGLE_JITTER | 0.30 rad | Random offset on evenly-spaced burst angles. |
| FADING_LIFE_THRESHOLD | 0.25 | Below this, everything is DIM. |
| SHIMMER_KEEP_ONE_IN | 4 | 75% reroll per frame. |
| FUSE_MIN/VAR_SEC | 0.5, 2.0 | Random fuse range after burst (seconds). |
| INITIAL_FUSE_STAGGER_SEC | 0.13 | Per-rocket startup stagger. |
| SPEED_SCALE_DEFAULT | 1.0 | `[/]` multiply by 0.8 / 1.25. |

## Open Questions
- Could the ROCKET trail be matrix-shimmery too (not just a fixed `|` `'` body)? Currently it's a static two-cell glyph while RISING.
- Per-burst colour palette would let "themed bursts" (e.g. all-red ones, all-blue ones) instead of every spark picking randomly.
- Acceleration on launch (ROCKET_THRUST > 0 for first 0.2 sec) would feel more like a real rocket — currently it's pure ballistic from t=0.

---

# Pass 2 — Pseudocode

## Module Map

| § | Purpose |
|---|---|
| §1 config | frame rate, rocket pool, rocket physics (CPS), spark physics (CPS), trail/shimmer, fade threshold, speed scale, color pair IDs |
| §2 clock | monotonic timer + sleep |
| §3 color | 5 themes (vivid/matrix/fire/ice/plasma), HUD pairs |
| §4 spark | Vec2 + Spark types, burst, advance_trail, integrate, shimmer, tick, trail_attr, draw |
| §5 rocket | RocketState + Rocket types, launch, per-state ticks, dispatch, draw |
| §6 show | Show type, init, tick, draw, input helpers |
| §7 screen | ncurses init / present / HUD |
| §8 app | signals, resize, variable-dt main loop |

## Data Flow

```
keys → app_handle_key → show.{paused, speed_scale, active_rockets, theme_idx}
                              │
clock_ns → dt → show_tick(s, dt)
                      └── per active rocket:
                            switch on state:
                              IDLE     → rocket_tick_idle    (fuse_sec)
                              RISING   → rocket_tick_rising  (physics)
                              EXPLODED → rocket_tick_exploded (sparks)
                              │
                              ▼
                          show_draw → per rocket:
                                      RISING:   draw '|' '\' body
                                      EXPLODED: per spark → spark_draw
```

## Pseudocode

```
setup:
  install signals
  screen_init, theme_apply, hud_pairs_init
  show_init(cols, rows, ROCKETS_DEFAULT)

while running:
  if need_resize: rebuild show preserving theme/speed/count
  dt = clamp(now - last, 0, DT_CAP_SEC)
  drain input → app_handle_key
  show_tick(show, dt, cols, rows)
  fps update
  erase
  show_draw(show, cols, rows)
  screen_draw_hud
  wnoutrefresh + doupdate
  sleep to TARGET_FPS
```

## Key Patterns to Internalize

**State machine = one function per state.** Don't put all the logic in one switch; extract per-state handlers and let the dispatcher be a 5-line switch.

**Per-spark trail = ring buffer that shifts.** Newest at [0], oldest at [N-1]. Shift in REVERSE order or you smear.

**Painter's order is load-bearing in particle systems.** Dim → bright. Always.

**Physical units > scale factors.** If `head += vel · dt` doesn't already work, your units are wrong.

**Fuse in seconds, not ticks.** Frame-rate-independent timers are always preferable.
