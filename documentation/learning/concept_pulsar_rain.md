# Pass 1 — pulsar_rain.c: Rotating pulsar neutron star with matrix-rain wake

## Core Idea

A neutron-star pulsar in your terminal. N evenly-spaced beams sweep around a single `@` core like a lighthouse, each leaving an angular wake of fading characters trailing behind it as it rotates. The wake glyphs are random ASCII letters/digits/punctuation that reroll every frame — the same matrix-rain shimmer trick from `matrix_rain.c`, applied to a rotating beam instead of a falling stream.

The simulation is variable-dt with `angle += spin_rps · 2π · dt` per frame. There is no fixed-step accumulator and no alpha interpolation — the float `angle` gives smooth sub-cell rotation at any frame rate because the simulation is non-stiff. Spin is in rotations-per-second so a learner can verify with a stopwatch ("at 0.5 rps, one beam returns to its starting cell every 2 seconds").

## The Mental Model

A lighthouse sweeping a foggy night. The beam itself is bright and narrow; behind it, glowing fog particles linger in the air for a moment before fading. Now spin the lighthouse fast enough that you see N beams at once (one or two for a real pulsar; more if you want stylised flowers). Each frame you snap a photo of the beams plus their fog wakes. The matrix-rain shimmer is just the foggy particles being random ASCII chars that re-pick themselves every frame — the **fog is the text**.

The pulsar is a SHEAF OF DIRECTIONS, not a pixel buffer. At any instant the simulation owns N angular sweep directions (`angle`, `angle + 2π/N`, ...). Each direction emits a beam consisting of `N_RADII` radial samples and a `WAKE_LEN`-slot angular tail. Rendering one beam is therefore TWO indices: `ri` (how far along the beam) and `k` (how far behind the head). The head k=0 is white-bold; growing k fades the colour through HOT → BRIGHT → MID → DARK → FADE while also rotating the cell back through the wake. No off-screen pixel buffer — the angle alone advances; everything else is recomputed every frame from cos/sin.

## Data Structures

### Pulsar (§4.2)
```
float angle;                      — current beam angle (radians, [0, 2π))
float spin_rps;                   — rotations per second; physical unit
int   n_beams;                    — 1..16, evenly spaced at 2π/n_beams
int   cx, cy;                     — screen-cell centre
float max_r;                      — farthest screen corner from centre
float r_step;                     — max_r / N_RADII; gap between radial samples
char  glyphs[N_RADII][WAKE_LEN+1];— shimmer cache, all beams share
int   theme_idx;
bool  paused;
```

### Wake-slot indexing
- `k = 0` is the beam head (leading edge, brightest).
- `k = WAKE_LEN` is the tail (oldest, dimmest).
- Each slot is `WAKE_STEP = 0.05 rad` behind the previous one.
- Total wake arc = `WAKE_LEN × WAKE_STEP = 16 × 0.05 = 0.8 rad ≈ 46°`.

### Brightness ramp (§3 wake_attr)
```
k=0           HEAD     white     BOLD
k=1           HOT      theme[4]  BOLD
k=2           BRIGHT   theme[3]  BOLD
k=3..N/2      MID      theme[2]  NORMAL
k=N/2+1..N-2  DARK     theme[1]  NORMAL
k=N-1..       FADE     theme[0]  DIM
```
Identical scheme to `matrix_rain.c`, `fireworks_rain.c`, `sun_rain.c`.

## The Main Loop (variable dt)

1. Resize check.
2. `dt` = wall-clock seconds since last frame, capped at `DT_CAP_SEC`.
3. Drain input.
4. `pulsar_tick(dt)`:
   - `omega = spin_rps · 2π`
   - `angle += omega · dt`; wrap into [0, 2π)
   - `pulsar_shimmer`: reroll most cache cells (1-in-`SHIMMER_KEEP_ONE_IN` survives)
5. fps rolling average.
6. Draw: `erase`, `pulsar_draw` (N beams + core last), HUD, `wnoutrefresh + doupdate`.
7. Frame cap.

## Beam render — key optimization

For each beam, pre-compute direction vectors **once**:
```
for k in 0..WAKE_LEN:
    cw[k] = cos(base_angle - k · WAKE_STEP)
    sw[k] = sin(base_angle - k · WAKE_STEP) · ASPECT
```
That is `WAKE_LEN+1 = 17` trig pairs per beam, regardless of `N_RADII`.

Then walk radial samples and angular slots:
```
for ri in 0..N_RADII-1:
    r = (ri + 1) · r_step
    for k in WAKE_LEN..0:        ← descending: dim-first so HEAD wins
        col = cx + round(r · cw[k])
        row = cy + round(r · sw[k])
        paint glyphs[ri][k] in wake_attr(k)
```
The descending k order is **load-bearing**. At small `r`, multiple slots round to the same cell; drawing dim slots first so the bright HEAD slot paints last lets the head always win the cell-paint contest.

## Multi-beam layout

```
beam b at base_angle = angle + b · (2π / n_beams)

n=1 :  single sweeping beam
n=2 :  classic pulsar pair (0°, 180°)        ← default
n=3 :  tri-blade            (0°, 120°, 240°)
n=4 :  cross                (0°, 90°, 180°, 270°)
n=8 :  star
n=16:  flower (wakes overlap at 22.5°)
```

## Non-Obvious Decisions

**Why drop fixed-step + alpha?**
Same reason as matrix_rain.c: simulation is non-stiff, float `angle` advancing by `omega · dt` per frame is unconditionally stable, and removing the accumulator drops ~30 lines of timing complexity.

**Why `spin_rps` (rotations/sec) instead of `spin_per_tick` (rad/tick)?**
Physical unit. A learner can predict and verify behaviour: "0.5 rps means one beam returns every 2 sec — I'll watch the clock." `rad/tick at SIM_FPS=20` requires mental conversion before any sanity check.

**Why bake `ASPECT` into `sin_a`?**
Terminal cells are ~2× tall as wide. Without aspect correction, vertical beams walk twice as fast as horizontal ones — the sun becomes an ellipse instead of round. Multiplying `sinf(angle)` by ASPECT once at direction-vector setup time keeps every position formula uniform: `col = cx + r·cw, row = cy + r·sw`.

**Why share one cache across all beams?**
N_BEAMS · N_RADII · (WAKE_LEN+1) cells would be wasteful. Identical glyphs across beams is acceptable because the eye can't track which beam shows which character — every beam shimmers in unison and that reads as one coherent rotating effect.

## Key Constants

| Constant | Default | Effect |
|---|---|---|
| N_RADII | 80 | Radial samples per beam. Smaller → gaps at the rim. |
| WAKE_LEN | 16 | Wake depth. Larger → longer fading tail (more arc). |
| WAKE_STEP | 0.05 rad | Angular gap between slots. Smaller → thinner wake. |
| ASPECT | 0.45 | Cell-aspect correction; baked into `sin_a`. |
| SPIN_DEFAULT_RPS | 0.5 | Rotations per second; `[`/`]` adjust by ×0.8 / ×1.25. |
| SHIMMER_KEEP_ONE_IN | 4 | 75% reroll per frame. |

## Open Questions
- Could `WAKE_STEP` adapt to current radius so the wake arc stays a constant cell-count instead of a constant angle? Would look better at small N_RADII.
- Per-beam glyph cache (instead of shared) would let beams shimmer at slightly different rates — visible as "twinkle" — at the cost of N× memory.

---

# Pass 2 — Pseudocode

## Module Map

| § | Purpose |
|---|---|
| §1 config | TARGET_FPS, beam geometry, spin range, beam count range, shimmer, color pairs |
| §2 clock | monotonic timer + sleep |
| §3 color | 5 themes (green/amber/blue/plasma/fire), wake_attr, HUD pairs |
| §4 pulsar | Pulsar state, init, tick, beam draw, core draw, input helpers |
| §5 screen | ncurses init / present / HUD |
| §6 app | signals, resize, variable-dt main loop |

## Data Flow

```
keys → app_handle_key → pulsar.{spin_rps, n_beams, theme_idx, paused}
                              │
clock_ns → dt → pulsar_tick(p, dt)
                       ├── angle += spin_rps · 2π · dt
                       ├── wrap into [0, 2π)
                       └── shimmer (reroll cache)
                              │
                              ▼
                      pulsar_draw
                       ├── for b in 0..n_beams: pulsar_draw_beam(p, base + b·step)
                       └── pulsar_draw_core (LAST)
                              │
                              ▼
                      ncurses paint + HUD + hint
```

## Pseudocode

```
setup:
  install signals
  screen_init, color_init (theme_apply + hud_pairs_init)
  pulsar_init(cols, rows)

while running:
  if need_resize: pulsar_init(new cols, rows) preserving spin/beams/theme/paused
  dt = clamp(now - last, 0, DT_CAP_SEC)
  drain input → app_handle_key
  pulsar_tick(p, dt)
  fps update
  erase
  pulsar_draw(p, cols, rows)
  screen_draw_hud
  wnoutrefresh + doupdate
  sleep to TARGET_FPS
```

## Key Patterns to Internalize

**Pre-compute direction vectors per beam.** Trig once per beam, reused at every radial sample. 17 trig per beam vs N_RADII × (WAKE_LEN+1) = 1360 if computed per cell.

**Dim-first rendering.** When multiple cells project to the same screen cell, paint dim ones first so the bright head wins the overlap contest. Universal pattern in radial / arc rendering.

**Physical units beat tick units.** `rotations per second` is verifiable; `rad/tick` is opaque.

**ASPECT belongs in the direction vector.** Bake it into `sin_a` once; never apply it again at draw time.

**Core LAST.** The `@` paints after every beam so it always sits on top, no special-cased z-order.
