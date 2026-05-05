# Pass 1 — murmuration.c: 800-Bird Reynolds Flock Rendered as a Density Field

## From the Source

**Algorithm:** Standard Reynolds boids (separation + alignment + cohesion) with toroidal wrapping, plus a flee force from one hawk predator. The three boid forces and the flee force are all computed in a single O(N²) pass per tick — `toroidal_delta` returns the shortest signed displacement on each axis so neighbours across the wrap stay clustered. Per-rule accumulators are converted to final forces by named helpers (`align_force`, `cohere_force`, `hawk_flee_force`); a sum-and-clamp produces the steering force capped at `MAX_STEER`.

**Math:** Per neighbour pair (toroidal Δ giving `dx, dy` and squared distance `d²`):
- `d² < SEP_R²` → `sep_force += unit(self − nb) · (SEP_R − d)/SEP_R · BOID_SPEED`
- `d² < ALIGN_R²` → accumulate `nb.vel`, count `ali_n`
- `d² < COH_R²` → accumulate toroidal *offset* (not absolute pos), count `coh_n`

Post-loop:
- `align = (ali_vsum/ali_n − vel)` if any neighbours
- `cohere = unit(coh_dsum/coh_n) · BOID_SPEED − vel` if any neighbours
- `hawk_flee = unit(self − hawk) · (HAWK_FLEE_R − d)/HAWK_FLEE_R · FLEE_SPEED` if hawk in range

**The visual trick (rendering):** Don't draw 800 individual bird glyphs — that would be a uniform soup at 800 birds in a 1920-cell terminal. Instead, bin agents into a `density[rows][cols]` grid each frame, then for each non-empty cell pick:
- glyph from the ASCII ramp `.,:;oO*#@` indexed by density (clamped at 9+)
- attribute `A_BOLD` for density ≥ 5, `A_NORMAL` otherwise (no `A_DIM` — the periphery would mute to invisible)
- color pair from `((cy * 7 + cx) % N_COLORS) + 1` (spatial-hash mottle within the active theme)

The flock now reads as a **2D density field**: dense core glows as `@`, peripheral tendrils fade through `*` `#` → `o` `O` → `:` `;` to scattered `.`. This is exactly the visual signature of real starling murmurations.

**Performance:** Single-pass O(N²) at 800 birds = 800·799 = 639 K pair tests per tick × 60 Hz ≈ 38 M/s. Each test is one `toroidal_delta` + dist² (with `sqrtf` only when inside cohesion radius). Sub-millisecond per tick at -O2. Density binning is O(N + cells) ≈ 2720 ops per render frame.

**Data-structure:** `Scene` owns a fixed `Boid pool[N_BOIDS_MAX = 1500]` (only first `n_birds` active), the `Hawk`, world dimensions, and `flock_centroid` (recomputed each tick toroidal-aware). A file-scope static `int g_density[MAX_ROWS][MAX_COLS]` (80 × 256 ints = 80 KB BSS) holds the per-frame density buffer — avoids per-frame malloc and per-tick stack pressure.

## Core Idea

Stop drawing birds. Draw the *local density* of birds. When 800 agents are packed into ~1900 terminal cells, each cell almost always has 0, 1, or 2 birds — except the heart of the flock, which has 5–15. Map count → ASCII glyph from a sparse-to-dense ramp, and the flock automatically reads as a moving black cloud with a bright core, internal density waves, and a clear silhouette — exactly how real starling murmurations look at human visual scale. The flocking forces just keep the cloud cohesive; the ramp is what makes it pretty.

## The Mental Model

Picture grayscale photography of a starling flock at dusk. At any pixel the brightness is *"how many birds happened to be along this line of sight?"* Shadows form where birds bunch up; the edges fade because density drops near the perimeter. That is exactly what we compute, in 2D, on a coarse grid: for each cell, count agents-in-cell, look up the corresponding glyph. The flocking simulation produces the spatial density field; the renderer projects it to characters. **No per-bird trail, no agent-level identity** — the field is the show.

Now add a hawk. The hawk is one bright `X` glyph that orbits the world far from the flock most of the time. When SPACE is pressed, the hawk picks a straight-line target (the flock centroid), accelerates to dive speed, and rips through the cloud as a bright `!`. Boids within `HAWK_FLEE_RADIUS` of the hawk pivot away; the centre splits open; the flock fragments into sheets and re-coheres seconds later. The whole "fragment and reform" is a famous murmuration motif — and it's visible in the density field as a **moving channel of low density** carved through the high-density core.

## Data Structures

### Boid
```
pos / prev_pos / vel  — kinematic state in pixel space
color_pair            — set once at spawn (cosmetic; renderer uses
                        spatial-hash cell tint, not per-bird color)
```
The renderer ignores `color_pair` because the cloud is drawn by density, not by individual identity. The field is kept for HUD identity and for any future per-bird drawing experiment.

### Hawk
```
pos / vel             — kinematic state (vel only used during DIVE)
mode                  — HAWK_PATROL | HAWK_DIVE
patrol_phase          — radians around world centre (PATROL only)
dive_timer            — seconds remaining in current dive
auto_dive             — bool toggled by 'h' key
auto_dive_timer       — seconds since last (auto-)fired dive
```

### Scene
```
pool[N_BOIDS_MAX]     — flat boid pool; first n_birds active
n_birds               — currently active count (100..1500)
hawk                  — singleton predator
theme_idx             — 0..N_THEMES-1; t/T cycles
world_w/h             — pixel-space dimensions, refreshed each tick
flock_centroid        — toroidal-aware mean position; cached for
                        HUD + dive targeting
paused                — physics frozen when true
```

### Theme palette (10 themes, t/T cycles)
Each theme is a **tight cluster** of 7 bright shades of one tint, not a dim-to-bright gradient. Reasoning: the renderer cycles through the 7 pairs by spatial hash, so adjacent cells get DIFFERENT pairs — if the palette spanned dim-to-bright, half the cells would render dim and the cloud would read as muddy. Keeping all 7 entries in the same bright band gives a clean tinted cloud with a subtle mottle.

```
Dusk     warm cream  (180-230)
Sky      sky-blue    (81-159)
Solar    warm yellow (214-228)
Aurora   mint→pink   (121-213)
Ember    fire ramp   (196-220)
Forest   leafy green (119-157)
Neon     hot magenta (165-225)
Sunset   warm orange (209-223)
Ghost    near-white  (231-255)
Matrix   matrix grn  (46-156)
```

All entries ≥ 80 (or full-saturation primaries 46/196/220/226). Hawk and HUD pairs are theme-independent.

### Density buffer
```
static int g_density[MAX_ROWS][MAX_COLS];   /* 80 × 256 ints = 80 KB BSS */
```
Sized for very large terminals so resize never overflows. `scene_draw` zeroes only the active region (`rows × cols`) per frame, not the full buffer.

## The Main Loop

Each iteration:

1. **Resize check.** SIGWINCH → re-read terminal size, rescale world dimensions, clamp all birds + the hawk into the new bounds.
2. **Measure dt.** `CLOCK_MONOTONIC` since last frame; capped at 100 ms (suspend guard).
3. **Fixed-step accumulator.** Drain `sim_accum` in `tick_ns`-sized steps; each fires `scene_tick(dt_sec)` which:
   - if auto-dive on, increments timer and fires a dive on rollover;
   - steps the hawk (PATROL orbit or DIVE integration with dive_timer countdown);
   - two-stage boid update: (A) compute `new_vel[i]` for every bird from OLD positions; (B) commit, integrate `pos`, wrap toroidally, clamp speed;
   - recomputes `flock_centroid` (toroidal-aware) for HUD + future dives.
4. **FPS counter.** 500 ms sliding window.
5. **Frame cap.** Sleep `(NS_PER_SEC/60 − elapsed)` before render. `elapsed = clock_ns() − frame_start`, NOT `+ dt`.
6. **Draw + present.** `erase()` → `scene_draw()` → HUD bars → `wnoutrefresh + doupdate`.
7. **Drain input.** `getch()` until `ERR`.

## Key Patterns to Internalize

**Density rendering as the visual primitive.** The flocking math produces a 2D density field; the renderer projects that to a glyph ramp + brightness tier. This is the single most important pattern in the file — it's what makes 800 birds look like a flock instead of a pixel cloud, and it's reusable for any high-count agent simulation (slime mold, dust storm, sand grains, particle effects).

**Toroidal-aware accumulators for cohesion.** Naïvely averaging absolute positions across a wrap-around world gives nonsense (birds at x=5 and x=635 average to x=320, in the middle of the screen). The fix: accumulate **toroidal offsets** (each computed by `toroidal_delta`) relative to *self*, then average. The mean offset is the direction to steer in.

**Single-pass O(N²) for multiple forces.** When several rules (sep, align, cohere) all need pairwise distances, compute `toroidal_delta` and `d²` ONCE per pair and reuse for every rule. ~3× faster than three independent loops.

**Two-stage update for "simultaneous" agents.** Stage A: every bird reads OLD positions, writes its own slot in `new_vel[]`. Stage B: every bird commits `vel = new_vel[i]`, integrates pos. Without two stages, bird 5 reacts to bird 0..4's NEW positions but bird 0..4 reacted to bird 5..N's OLD positions — index-order drift accumulates over many ticks.

**Predator state machine separated from prey forces.** The hawk has its own controller (`hawk_step`) and modes (`HAWK_PATROL` / `HAWK_DIVE`). Prey know nothing about the hawk's mode — they just see its position and apply a flee force if close. Same pattern as `shepherd.c`'s collie + sheep: keep predator and prey loosely coupled through position alone.

**Spatial-hash tint for "one tinted cloud".** Each cell picks pair `((cy*7 + cx) % 7) + 1` from the active theme. With 7 tightly-clustered shades per theme, the result is a soft mottle within the flock that reads as one colour — the eye doesn't notice individual cell tints unless looking closely.

## Non-Obvious Decisions

**Why no sub-tick alpha lerp on render?** In `flocking.c` and `crowd.c` the renderer interpolates `prev_pos → pos` by `alpha` for sub-tick smooth motion. Murmuration omits this on purpose: the visual is the **density field**, and density is itself stable across alpha values (a bird crossing a cell boundary momentarily increments two cells, smoothing the field naturally). Skipping the lerp also saves 2N float ops per render frame.

**Why no per-bird color cycle?** Earlier versions cycled 7 colors round-robin per agent index. With 800 agents in 1900 cells, every cell averages 7 agents from 7 different colors → unintelligible rainbow. The spatial hash (cell position, not bird identity) gives one colour per cell, producing a coherent tinted cloud.

**Why a Hawk not a Hawk-flock?** Single hawk gives a clean visual narrative — orbit, dive, retreat. Multiple hawks would each create their own scatter wave; the flock would never re-cohere because at least one hawk would always be inside `HAWK_FLEE_RADIUS` of part of the flock. The single-hawk system is the minimum viable predator that produces the murmuration "fragment and reform" motif.

**Why is `HAWK_DIVE_DURATION` 1.5 s and not "until target reached"?** A timed dive (rather than a goal-based one) gives the dive a predictable cadence regardless of how far the flock has moved. Also: at `HAWK_DIVE_SPEED = 250 px/s`, 1.5 s = 375 px, less than the 640-px width of an 80-col terminal. The hawk punches THROUGH the centre and stops mid-flock — visually more dramatic than a clean fly-by.

**Why a static `g_density` buffer instead of per-frame malloc or VLA?** 80 × 256 ints = 80 KB. On stack as a VLA, that's borderline (some terminals run with small thread stacks). On heap with malloc/free per frame, that's allocator pressure 60×/sec. As a static, it's allocated once in BSS and zeroed only over the active region each frame. Cleanest.

## How to Verify

- At startup: 800 random birds collapse into one roving cluster within ~3 seconds. HUD shows `n:800/1500 [Dusk] hawk:PATROL`. The cluster's core reads as `@` glyphs surrounded by `#`/`*` rings, fading through `o`/`O`/`:`/`;` to scattered `.` at the edges.
- Press SPACE: the red `X` becomes a bright `!` and accelerates toward the flock centroid. Within half a second a low-density channel forms behind the hawk's path. After `HAWK_DIVE_DURATION ≈ 1.5 s` the hawk slows and resumes patrol; ~5 s later the flock has reformed.
- Press `h`: SPACE-equivalent dive fires every 6 s automatically. Watch the flock cycle: cohere → split → reform → split → reform.
- Press `+`: bird count jumps by 100. The density ramp peaks higher more often, more cells read as `@`.
- Press `t`/`T`: tint cycles through 10 themes. Hawk and HUD stay theme-independent (red `!`, yellow status, cyan hint).

## References

- Reynolds, "Flocks, Herds, and Schools: A Distributed Behavioral Model," SIGGRAPH 1987 — the source of every boid force used here.
- Couzin, Krause, James, Ruxton, Franks, "Collective Memory and Spatial Sorting in Animal Groups," J. Theor. Biol. 218 (2002) — analyses the same sep/align/cohere parameter regimes that produce the murmuration density-wave behaviour.
- Hildenbrandt, Carere, Hemelrijk, "Self-organized aerial displays of thousands of starlings: a model," Behav. Ecol. 21 (2010) — the canonical quantitative model of starling murmurations.
- Wikipedia: "Murmuration" — context for the visual pattern this demo reproduces in ASCII.
