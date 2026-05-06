# Pass 1 — sun_rain.c: Radial matrix-rain solar corona

## Core Idea

A single `@` burns at the screen centre. 180 independent radial streams of matrix glyphs shoot outward from it in all directions — a continuous solar wind with no circle, no disc, no border. Each stream has its own random speed and a stagger offset so they appear at different distances from the core at any given moment, producing a stochastic field of beams rather than a single synchronised burst.

The simulation is variable-dt with `r_off += speed_cps · dt` per ray per frame. There is no fixed-step accumulator and no alpha interpolation — float `r_off` gives smooth sub-cell radial motion at any frame rate. Speed is in cells/sec so a learner can predict and verify ("at 80 cells/sec, a ray crosses a 30-cell screen in ~0.4 sec").

## The Mental Model

Imagine a clock face with 180 hour-marks. From each mark, a comet shoots outward through space. Each comet has a head, a 16-character glittering tail, a personal speed (30–80 cells/sec), and was launched at a different time so the field looks chaotic rather than synchronised. The trick that makes it look round (and not a tall ellipse) is the "tall cell" correction: terminal cells are about 2:1 in aspect, so raw `sin(θ)` would make vertical rays walk twice as fast as horizontal ones. We multiply `sin(θ)` by `ASPECT = 0.45` once per ray at init time and bake the result into `sin_a`, so the inner draw loop has no per-cell trig.

This is the same polar-coordinate machinery as `pulsar_rain.c`, with one big difference: rays here SLIDE OUTWARD along their fixed angle, while in `pulsar_rain` they ROTATE around their fixed radius.

## Data Structures

### Ray (§4.2)
```
float cos_a, sin_a;   — pre-baked direction; sin_a has ASPECT baked in
float r_off;          — head distance from centre (float, can be negative)
float speed;          — cells/sec; randomised on each (re)spawn
char  cache[16];      — random ASCII glyphs, 75% reroll per frame
```

### Sun (§4.6)
```
Ray   rays[180];      — N_RAYS rays
int   cx, cy;         — screen-cell centre
float max_r;          — recycle distance (cols + rows/ASPECT)
int   theme_idx;
bool  paused;
```

### Brightness ramp (§3 dist_attr)

```
i=0           HEAD     white     BOLD
i=1           HOT      theme[4]  BOLD
i=2           BRIGHT   theme[3]  BOLD
i=3..N/2      MID      theme[2]  NORMAL
i=N/2+1..N-2  DARK     theme[1]  NORMAL
i=N-1..       FADE     theme[0]  DIM
```

Identical scheme across all matrix_rain/ files.

## The Main Loop (variable dt)

1. Resize check.
2. `dt` = wall-clock seconds since last frame, capped at `DT_CAP_SEC`.
3. Drain input.
4. `sun_tick(dt)`: for each ray, `r_off += speed · dt`; reroll cache cells with prob 1−1/SHIMMER_KEEP_ONE_IN. If `r_off − RAY_TRAIL ≥ max_r`, the tail has cleared the screen → respawn the ray with fresh stagger / speed / cache (preserving angle).
5. fps rolling average.
6. Draw: erase, `sun_draw` (180 rays + core last), HUD, hint, present.
7. Frame cap.

## Per-ray render

```
for i in 0..RAY_TRAIL-1:
    ri = r_off - i
    if ri < CORE_RESERVED_RADIUS: break    ← preserve core cell
    col = cx + round(ri · cos_a)
    row = cy + round(ri · sin_a)
    paint cache[i] in dist_attr(i)
```

The break at `CORE_RESERVED_RADIUS = 1.0` is critical: without it, trails draw OVER the centre and the `@` flickers off whenever a tail crosses (cx, cy). Pass-2 `@` must paint AFTER all rays so it always sits on top.

## Ray lifecycle

```
ray_init(angle, max_r):     ← called once per ray at sun_init
  cos_a = cos(angle)
  sin_a = sin(angle) · ASPECT       ← ASPECT baked in
  ray_reset(max_r)

ray_reset(max_r):           ← called on respawn
  r_off  = uniform([-STAGGER_FRAC · max_r, 0])
                            ← negative = "pre-emerged" stagger
  speed  = uniform([SPEED_MIN_CPS, SPEED_MAX_CPS])
  cache  = random glyphs

ray_tick(dt, max_r):        ← called every frame
  r_off += speed · dt
  shimmer cache
  return r_off - RAY_TRAIL < max_r   ← false → respawn
```

## Non-Obvious Decisions

**Why drop fixed-step + alpha?**
Same as siblings: variable-dt with float r_off is unconditionally stable for non-stiff simulations. The whole accumulator + alpha apparatus is unnecessary.

**Why `ray_init` + `ray_reset` split?**
The original recomputed `cosf(angle)` and `sinf(angle)` on every reset — wasted compute and conceptually muddled (angle never changes after init). The split makes the constant-vs-per-life data clear: cos_a/sin_a are direction (constant), r_off/speed/cache are state (resets).

**Why bake ASPECT into sin_a?**
Terminal cells are ~2× tall as wide. Without aspect correction, vertical rays walk twice as fast as horizontal ones — the corona becomes a tall ellipse instead of round. Multiplying `sinf(θ)` by ASPECT once at ray_init time keeps every position formula uniform: `col = cx + ri·cos_a`, `row = cy + ri·sin_a`. No per-cell ASPECT correction.

**Why is `r_off` initial value negative?**
At spawn, `r_off = uniform(-STAGGER_FRAC · max_r, 0)`. Negative = "pre-emerged"; the tail is below the core for stagger/speed seconds before the head actually crosses the core. This staggered emergence is what makes the field look chaotic at startup instead of "all 180 rays appear at radius 0 at t=0".

**Why is the trail break condition `ri < CORE_RESERVED_RADIUS` (not `ri < 0`)?**
At `ri = 0`, the cell is exactly the core position. Drawing a trail glyph there would replace the `@`. We reserve a 1-cell guard so any rounding artefact still leaves the core untouched.

**Why is the `@` painted last?**
Painter's algorithm. With pass-1 ray draws preserving the core via the `ri < CORE_RESERVED_RADIUS` break, the core *should* never be overwritten — but defence in depth: drawing `@` last means the core can't be lost even if a future change accidentally allows trails to write at radius 0.

## Key Constants

| Constant | Default | Effect |
|---|---|---|
| N_RAYS | 180 | Angular density. Smaller (e.g. 60) leaves visible gaps. |
| RAY_TRAIL | 16 | Tail length. Larger = longer arc fade. |
| ASPECT | 0.45 | Cell-aspect correction (terminal cells are ~2:1 tall:wide). |
| SPEED_MIN/MAX_CPS | 30, 80 | Ray speed range (cells/sec). |
| STAGGER_FRAC | 0.55 | Initial spawn spread; 0 = synchronous, 1 = full screen-wide stagger. |
| CORE_RESERVED_RADIUS | 1.0 | Trail-draw cutoff so `@` is never overwritten. |
| SHIMMER_KEEP_ONE_IN | 4 | 75% reroll per frame. |
| TRAIL_HOT_END | 2 | Indices 0..2 render BOLD. |
| TRAIL_WARM_END | RAY_TRAIL/2 = 8 | Indices 3..7 render NORMAL. |

## Open Questions
- Could speed pulse over time (sin-wave per ray) to give the corona a "breathing" feel?
- Per-ray theme ratio (some rays use brighter palette, some dimmer) would mimic real solar wind layers — worth doing?
- At very small terminals, `max_r` is small and rays cycle quickly. Could a velocity floor keep slow-motion verifiable on small screens?

---

# Pass 2 — Pseudocode

## Module Map

| § | Purpose |
|---|---|
| §1 config | frame rate, ray geometry, ASPECT, speed range, stagger, shimmer, color pair IDs |
| §2 clock | monotonic timer + sleep |
| §3 color | 5 themes (solar/green/nova/plasma/fire), dist_attr, HUD pairs |
| §4 sun | Ray + Sun types, init/reset, tick, draw beam, draw core, dispatch, input helpers |
| §5 screen | ncurses init / present / HUD |
| §6 app | signals, resize, variable-dt main loop |

## Data Flow

```
keys → app_handle_key → sun.{paused, theme_idx}
                            │
clock_ns → dt → sun_tick(s, dt)
                      └── per ray:
                            ray_tick(r, dt, max_r)
                              ├── r_off += speed · dt
                              ├── shimmer cache
                              └── return false if tail off-screen → ray_reset
                                            │
                                            ▼
                                       sun_draw
                                          ├── per ray: ray_draw (head + trail)
                                          └── sun_draw_core ('@', LAST)
                                                  │
                                                  ▼
                                             ncurses paint + HUD + hint
```

## Pseudocode

```
setup:
  install signals
  screen_init, color (theme_apply + hud_pairs_init)
  sun_init(cols, rows)

while running:
  if need_resize: sun_init(new cols, rows) preserving theme/paused
  dt = clamp(now - last, 0, DT_CAP_SEC)
  drain input → app_handle_key
  sun_tick(sun, dt)
  fps update
  erase
  sun_draw(sun, cols, rows)         # 180 rays first, then '@' core
  screen_draw_hud
  wnoutrefresh + doupdate
  sleep to TARGET_FPS

cleanup: endwin
```

## Key Patterns to Internalize

**Polar coordinates, baked direction.** Pre-compute `(cos_a, sin_a · ASPECT)` once per ray. The inner loop is two muls + two rounds + one mvaddch.

**ASPECT belongs in `sin_a`.** Once. Never apply it again.

**Stagger via negative initial offset.** `r_off ∈ [-STAGGER_FRAC · max_r, 0]` is a clean way to spread "first appearance times" without an explicit timer.

**Reserve the core cell.** Any radial sim with a centre point needs a guard radius so trails don't paint over the centre.

**Init vs reset = constant vs state.** Anything that doesn't change after init (cos_a, sin_a) goes in init. Everything that resets per life cycle (r_off, speed, cache) goes in reset.

**Sibling-file consistency.** The brightness band ramp (HEAD/HOT/BRIGHT/MID/DARK/FADE), the SHIMMER_KEEP_ONE_IN trick, the per-stream cache, the HUD/HINT pair convention — all identical to `matrix_rain.c`, `pulsar_rain.c`, `fireworks_rain.c`, `matrix_snowflake.c`. Learn it once, recognise it everywhere.
