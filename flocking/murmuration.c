/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * murmuration.c — 800-bird starling flock rendered as a density field
 *
 * DEMO: A high-count Reynolds flock (separation + alignment + cohesion)
 *       drifts across a toroidal terminal world.  Instead of drawing
 *       one glyph per bird, the renderer bins agents into a per-cell
 *       density grid and stamps a glyph from the ramp ".,:;oO*#@" —
 *       sparse cells fade to dots, dense cells glow as `@`, the
 *       flock reads as a moving black blob with internal density
 *       waves.  A single hawk orbits the world centre; press SPACE
 *       to send it diving through the flock at 250 px/s for ~1.5 s
 *       — the flock fragments around the hawk and reforms behind
 *       it.  Press 'h' to enable auto-dive every 6 seconds.
 *
 * Study alongside: flocking/flocking.c (the boid forces this builds on)
 *                  flocking/shepherd.c (Strömbom hawk/sheep model)
 *
 * Section map:
 *   §1 config   — counts, speeds, radii, weights, glyph ramp
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 10 theme palettes + PAIR_HAWK + PAIR_HUD/PAIR_HINT
 *   §4 coords   — pixel↔cell aspect bridge + Vec2 + toroidal_delta
 *   §5 boid     — Boid struct, spawn, integrate, speed clamp, wrap
 *   §6 forces   — single-pass separation + alignment + cohesion + hawk flee
 *   §7 hawk     — Hawk struct + PATROL/DIVE controller
 *   §8 scene    — Scene state, tick, density binning + glyph rendering
 *   §9 app      — signals, resize, main game loop
 *
 * Keys:
 *   q / ESC    quit                       space    hawk dive (single)
 *   h          toggle auto-dive (every    p        pause / resume
 *              AUTO_DIVE_PERIOD seconds)
 *   r / R      reset                      t / T    next / prev theme
 *   + / -      add / remove 100 boids
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra flocking/murmuration.c \
 *       -o murmuration -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Reynolds boids (separation, alignment, cohesion) on a
 *                  toroidal grid, plus a flee force from a single hawk
 *                  predator.  All three boid forces and the flee force
 *                  are computed in ONE single-pass O(N²) loop per tick
 *                  — toroidal_delta gives the shortest signed
 *                  displacement on each axis so neighbours across the
 *                  screen edge are correctly seen as close.
 *
 *                  The renderer is what makes this visually distinct
 *                  from `flocking/flocking.c`.  Instead of one glyph
 *                  per bird (which at 800 birds in 80x24 = 1920 cells
 *                  would be a uniform soup), each frame:
 *                    1. zero a density[rows][cols] grid;
 *                    2. for each bird, increment density[cy][cx];
 *                    3. for each non-empty cell, pick a glyph from the
 *                       ASCII ramp ".,:;oO*#@" indexed by density and
 *                       a brightness from A_DIM/NORMAL/BOLD by tier.
 *                  The flock now reads as a 2D density field — exactly
 *                  the visual signature of real starling murmurations.
 *
 *                  HAWK has two states.  PATROL: orbit the world centre
 *                  at radius 0.40·min(w,h) with angular speed 0.4 rad/s.
 *                  DIVE (triggered by SPACE or auto-dive timer): set a
 *                  straight-line velocity toward the flock centroid,
 *                  step at 250 px/s for HAWK_DIVE_DURATION (~1.5 s),
 *                  then resume PATROL from the new position.
 *
 * Data-structure : Scene owns a fixed Boid pool[N_BOIDS_MAX], the
 *                  active count `n_birds`, the Hawk, the world
 *                  dimensions, and a pair of file-scope statics for
 *                  the per-frame density buffer (avoiding a per-frame
 *                  malloc and keeping the stack small).  Boids carry
 *                  pos / prev_pos / vel and a spawn-time colour pair
 *                  index used for HUD identity (the renderer ignores
 *                  per-bird colour because it draws the density field).
 *
 * Rendering      : Painter's order — density-mapped flock first, hawk
 *                  '!' last (always on top, A_BOLD red).  The density
 *                  ramp index is `min(density − 1, RAMP_LEN − 1)`; the
 *                  brightness tier is `A_DIM` for density 1-2,
 *                  `A_NORMAL` for 3-5, `A_BOLD` for 6+.  Sub-tick
 *                  alpha lerp is omitted on purpose — the visual is
 *                  the density gradient, which is itself stable across
 *                  alpha values, and skipping the per-bird interp
 *                  saves the 800 × 2 lerp ops per frame.
 *
 * Performance    : Single-pass O(N²) at 800 birds = 800·799 = 639 K
 *                  pair tests per tick × 60 Hz ≈ 38 M/s.  Each test is
 *                  one toroidal-delta + dist² (≤ one sqrt only when
 *                  inside cohesion radius).  Sub-millisecond per tick
 *                  on any modern CPU.  Density binning is O(N + cells)
 *                  ≈ 800 + 1920 = 2720 ops per render.
 *
 * References     : Reynolds, "Flocks, Herds, and Schools: A Distributed
 *                    Behavioral Model," SIGGRAPH 1987 — the source of
 *                    every boid force used here.
 *                  Couzin, Krause, James, Ruxton, Franks, "Collective
 *                    Memory and Spatial Sorting in Animal Groups,"
 *                    J. Theor. Biol. 218 (2002) — analyses the same
 *                    sep/align/cohere parameter regimes that produce
 *                    the murmuration density-wave behaviour.
 *                  Wikipedia, "Murmuration" — context for the visual
 *                    pattern this demo reproduces in ASCII.
 *                  Hildenbrandt, Carere, Hemelrijk, "Self-organized
 *                    aerial displays of thousands of starlings: a
 *                    model," Behav. Ecol. 21 (2010) — the canonical
 *                    quantitative model of starling murmurations.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Stop drawing birds.  Draw the LOCAL DENSITY of birds.  When 800
 * agents are clustered into ~1900 terminal cells, each cell almost
 * always has 0, 1, or 2 birds — except the heart of the flock,
 * which has 5-15.  Map count → ASCII glyph from a ramp going
 * sparse-to-dense, and the flock automatically reads as a moving
 * black cloud with a bright core, internal density waves, and
 * a clear silhouette — which is exactly how real starling
 * murmurations look at human visual scale.  The flocking forces
 * just keep the cloud cohesive; the ramp is what makes it pretty.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture grayscale photography of a starling flock at dusk.  At
 * any pixel, the brightness is "how many birds happened to be
 * along this line of sight?"  Shadows form where birds bunch up;
 * the edges fade because density drops off near the perimeter.
 * That's exactly what we compute, in 2D, on a coarse grid: for
 * each cell, count agents-in-cell, look up the corresponding
 * glyph.  The flocking simulation produces the spatial density
 * field; the renderer projects it to characters.  No per-bird
 * trail, no agent-level identity — the field is the show.
 *
 * Now add a hawk.  The hawk is one bright '!' that orbits the
 * world far from the flock most of the time.  When SPACE is
 * pressed, the hawk picks a straight-line target (the flock
 * centroid), accelerates to dive speed, and rips through the
 * cloud.  Boids within HAWK_FLEE_RADIUS of the hawk pivot away
 * from it, the centre splits open, the flock fragments into
 * sheets and re-coheres seconds later.  The whole "fragment and
 * reform" is a famous murmuration motif — and it's visible in
 * the density field as a moving channel of low density carved
 * through the high-density core.
 *
 * ALGORITHM IN STEPS  (per tick)
 * ──────────────────────────────
 *   1. Hawk: tick controller.  PATROL → advance phase along the
 *      orbit; DIVE → integrate forward at HAWK_DIVE_SPEED until
 *      dive_timer elapses, then swap back to PATROL at the
 *      hawk's current position (no teleport).
 *   2. Boids: read OLD positions in one O(N²) loop.  For each
 *      pair (i, j) compute toroidal Δ; classify into separation
 *      (very close), alignment (medium), cohesion (medium).
 *      Sum into per-bird accumulators.  After the inner loop,
 *      add a flee force from the hawk (read once per bird, not
 *      per pair).
 *   3. Boids: write phase.  Apply force·dt to vel, clamp |vel|
 *      to [MIN_SPEED, MAX_SPEED], integrate pos, wrap toroidally.
 *   4. Render: zero density[rows][cols]; bin every bird into the
 *      grid via px_to_cell; for each cell with density > 0 stamp
 *      the corresponding glyph + brightness; over-stamp the hawk.
 *
 * KEY FORMULAS
 * ────────────
 *   Toroidal delta (shortest signed displacement on a wrapped axis):
 *     d = b − a
 *     if d >  extent/2 : d −= extent
 *     if d < −extent/2 : d += extent
 *
 *   Boid forces (per neighbour with squared toroidal distance d²):
 *     if d² < SEP_R²  : sep_force  += unit(self−nb) · (SEP_R − d)/SEP_R · BOID_SPEED
 *     if d² < ALIGN_R²: ali_vsum   += nb.vel; ali_n++
 *     if d² < COH_R²  : coh_dsum   += toroidal_delta(self, nb)  (offset, NOT pos)
 *                       coh_n++
 *
 *   After loop:
 *     ali_force  = (ali_n>0) ? (ali_vsum/ali_n − vel)             : 0
 *     coh_force  = (coh_n>0) ? unit(coh_dsum/coh_n) · BOID_SPEED  : 0
 *     hawk_flee  = if dist(self, hawk) < HAWK_FLEE_RADIUS:
 *                    unit(self − hawk) · (HAWK_FLEE_R − d)/HAWK_FLEE_R · FLEE_SPEED
 *                  else 0
 *
 *   Steering sum:
 *     force = W_SEP·sep + W_ALIGN·ali + W_COHERE·coh + W_FLEE·hawk_flee
 *
 *   Density-to-glyph mapping (renderer):
 *     idx   = min(density − 1, RAMP_LEN − 1)
 *     glyph = RAMP[idx]                     ".,:;oO*#@"
 *     attr  = density ≥ 5 ? A_BOLD : A_NORMAL
 *
 *     (No A_DIM tier — sparse cells make up most of the flock area
 *     and A_DIM would mute them to near-invisible, defeating the
 *     "tinted cloud" look.  The fade-to-edge effect comes from the
 *     glyph ramp '.'→'@', not from brightness modulation.)
 *
 * WORKED EXAMPLE  (defaults: 800 birds, 80x24 terminal)
 * ─────────────────────────────────────────────────────
 *   World box       : 80 × 24 cells = 640 × 384 pixels.
 *   Cells available : 1920.  At even spread that's 0.42 birds/cell
 *                      — most cells empty, max ~1.  Cohesion makes
 *                      birds cluster: typical packed centre ~6-10
 *                      birds/cell, peak ~15+ in the densest core.
 *                      The glyph ramp uses density 1-9+ so the core
 *                      reads as `@` and the wings fade through
 *                      `*` `#` → `o` `O` → `:` `;` → `,` `.`.
 *   Per tick (1/60s):
 *     a bird at BOID_SPEED 90 px/s travels 1.5 px ≈ 1/5 column —
 *       slow enough to look graceful, fast enough to evolve.
 *     the hawk in DIVE mode at 250 px/s covers 4.2 px/tick —
 *       sweeps through the flock (~640 px wide) in ~2.5 seconds
 *       (longer than HAWK_DIVE_DURATION, so the dive ends BEFORE
 *       the hawk exits the screen — flock has time to react).
 *   Steering cost   : O(N²) inner loop at 800 birds = 800·799 =
 *                     639 K pair tests/tick = 38 M/s at 60 Hz.
 *                     Sub-millisecond per tick at -O2.  Density
 *                     binning is O(N + cells) ≈ 2720 ops/render.
 *   Cohesion        : COHESION_RADIUS = 80 px ≈ 10 cols × 5 rows —
 *                     a bird sees ~ π·80² / (640·384) · 800 ≈ 21
 *                     neighbours on average.  Plenty for a smooth
 *                     mean-position pull.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   - Naïve cohesion across the wrap: birds at x=5 and x=635 are
 *     close on the torus but the mean of their absolute positions
 *     is in the middle of the screen.  Cohesion accumulates summed
 *     toroidal_delta OFFSETS (relative to self), never absolute
 *     positions, then steers along the offset direction.
 *   - Density buffer size: a static `density[MAX_ROWS][MAX_COLS]`
 *     buffer (80 × 256 ints = 80 KB) sized for very large terminals.
 *     scene_draw zeroes only the active region (rows × cols) each
 *     frame, not the whole buffer.
 *   - Speed floor: when sep, align, cohere all roughly cancel, vel
 *     can drop to ~0 and a bird stalls.  boid_clamp_speed bumps it
 *     back to MIN_SPEED so the flock never freezes.
 *   - Hawk dive duration vs flock crossing: at HAWK_DIVE_SPEED=250
 *     the hawk traverses ~375 px in 1.5 s.  On 80-col terminals
 *     (640 px) the hawk doesn't cross fully — it punches through
 *     the centre and stops mid-flock, which produces a more dramatic
 *     scatter than a clean fly-by.
 *   - Auto-dive timer: re-armed each time it fires, NOT each frame
 *     (so the period is deterministic).  Disabling auto-dive
 *     freezes the timer; re-enabling resumes it.
 *   - Frame cap: never `elapsed = clock_ns() − frame_time + dt` —
 *     adding dt cancels the cap.  Use a `frame_start` snapshot.
 *
 * HOW TO VERIFY
 * ─────────────
 *   - Within ~3 seconds the random spawn of 800 birds collapses
 *     into a single roving cluster.  HUD shows "n:800/1500" and
 *     the core of the flock reads as `@` glyphs surrounded by
 *     `#`/`*` then `:`/`.` toward the periphery.
 *   - Press SPACE: the red `!` accelerates from its orbit toward
 *     the flock centroid; within half a second the flock starts
 *     splitting — a low-density channel forms behind the hawk's
 *     path.  After HAWK_DIVE_DURATION the hawk slows; ~5 s later
 *     the flock has reformed into a single cluster again.
 *   - Press `h`: SPACE-equivalent dive fires every 6 s
 *     automatically.  Watch the flock cycle: cohere → split →
 *     reform → split → reform.
 *   - Press `+` to add birds; the density ramp peak rises into
 *     `@` more often.  Press `-` to shrink the flock; the visual
 *     thins out and you can see individual `o` clusters.
 *   - Press `t` / `T`: flock tint cycles through 10 themes; the
 *     hawk and HUD stay theme-independent.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read flocking.c first; the boid forces are the
 *      same Reynolds rules. The NEW LESSONS here are: density-field
 *      rendering, large-N optimisation, and the hawk as a state
 *      machine.
 *   2. §6 forces — single-pass O(N²) boid + flee. THE HEART of
 *      the simulation. Read AFTER tutorials T1-T5 below.
 *   3. §7 hawk — PATROL / DIVE controller (T4).
 *   4. §8 scene — orchestrator + density-binning renderer (T1-T2).
 *   5. §1-§5, §9 — config / clock / colour / coords / boid /
 *      app loop. Skim if you've seen the framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   density[r][c]          per-cell count of birds, recomputed
 *                          every frame.
 *   RAMP                   glyph ramp ".,:;oO*#@" — 9 chars from
 *                          sparse to dense.
 *   N_BOIDS_MAX            cap (800 default; tunable with +/-).
 *   hawk.state             PATROL or DIVE.
 *   dive_timer             seconds remaining in DIVE.
 *   HAWK_FLEE_RADIUS       boids inside this distance of the hawk
 *                          feel a strong push away.
 *   AUTO_DIVE_PERIOD       seconds between auto-dives (h key
 *                          enables).
 *   MIN_SPEED, MAX_SPEED   speed clamp.
 *
 * Background you need
 * ───────────────────
 *   - flocking.c T1-T7 (boid + three rules + toroidal + 2-stage).
 *   - Histogram / binning — counting items into discrete buckets.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Spatial hashing (grids of buckets for O(N) neighbour search).
 *     We rely on raw O(N²) at 800 boids; on a modern CPU that's
 *     sub-millisecond.
 *   - Voxel splatting / volume rendering. The density grid is 2-D,
 *     ASCII output.
 *   - Per-bird identity. The renderer doesn't track WHICH boid is
 *     in which cell — only HOW MANY.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Five tutorials that build a 800-bird density-field flock from
 * first principles.
 *
 *   T1  Why DENSITY rendering — 800 glyphs is a soup
 *   T2  Density binning — count agents per cell, then map to glyph
 *   T3  Single-pass O(N²) — combine all three rules in one loop
 *   T4  Hawk state machine — PATROL ↔ DIVE without a teleport
 *   T5  Why this looks like REAL starlings — emergent density waves
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHY DENSITY RENDERING — 800 GLYPHS IS A SOUP
 * ────────────────────────────────────────────────
 * flocking.c renders ONE GLYPH PER BOID. With 36 boids in
 * 200×60 = 12000 cells, the glyphs are visibly distinct.
 *
 * Now scale to 800 boids in 80×24 = 1920 cells. That's 0.42
 * boids per cell on average. With cohesion bringing them
 * close, the central region has 5-15 boids per cell while
 * the edges have 0-1. If we drew one glyph per boid:
 *
 *   - 5 boids overlap on the same cell. Only the LAST one
 *     drawn shows; you can't tell the cell has 5 or 1.
 *   - The flock looks like a uniform soup of glyphs at the
 *     centre, dribbles of glyphs at the edges. Loses all the
 *     density structure that makes real murmurations
 *     beautiful.
 *
 * Solution: draw the DENSITY FIELD, not the boids.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │  per-boid           density-field                │
 *      │                                                  │
 *      │   ▓▓▓▓▓▓             .  ,  : oOO*#@*Oo: , .      │
 *      │  ▓▓▓▓▓▓▓▓▓          .,:o##@@@@@@##o:,.           │
 *      │  ▓▓▓▓▓▓▓▓▓           ,;oO*#@@@@@@*#Oo;,          │
 *      │   ▓▓▓▓▓▓             .,:o*#@@@*Oo:,.             │
 *      │                       . ,;:O*#OO;:, .            │
 *      │                                                  │
 *      │  uniform soup        moving cloud                │
 *      │  (no structure)      (visible density gradient)  │
 *      └──────────────────────────────────────────────────┘
 *
 * Same simulation; different rendering. The density field
 * makes 800 individual agents READ as one organism with
 * internal structure.
 *
 * T2  DENSITY BINNING — COUNT AGENTS PER CELL, THEN MAP TO GLYPH
 * ──────────────────────────────────────────────────────────────
 * Each render frame:
 *
 *   1. zero a 2-D density grid: density[rows][cols] = 0
 *   2. for each boid:
 *        cx = round(boid.px / CELL_W)
 *        cy = round(boid.py / CELL_H)
 *        density[cy][cx] += 1
 *   3. for each cell with density > 0:
 *        idx   = min(density - 1, RAMP_LEN - 1)
 *        glyph = RAMP[idx]                  // ".,:;oO*#@"
 *        attr  = (density ≥ 6) ? A_BOLD : A_NORMAL
 *        paint cell with (glyph, attr, theme_color)
 *
 * The RAMP is a 9-character ASCII gradient from sparse to
 * dense. density=1 → '.'; density=2 → ','; ...; density=9+
 * → '@'. The glyph itself encodes the count visually:
 *
 *     RAMP = ". , : ; o O * # @"
 *             1   2   3   4   5   6   7   8   9+
 *
 * Three glyph tiers + two attribute tiers (NORMAL, BOLD)
 * give the eye six visible brightness steps — enough for a
 * smooth gradient at terminal resolution.
 *
 * Why no A_DIM? Sparse cells dominate the area of the flock;
 * A_DIM would make them invisible, defeating the soft fade.
 * The RAMP itself provides the dim-end of the gradient.
 *
 * Cost: O(N + cells) per frame. For 800 boids + 1920 cells,
 * that's ~2700 operations per render — way cheaper than
 * 800 mvaddch operations one per bird.
 *
 * T3  SINGLE-PASS O(N²) — COMBINE ALL THREE RULES IN ONE LOOP
 * ───────────────────────────────────────────────────────────
 * Naive Reynolds: three separate O(N²) loops, one per rule:
 *
 *     for each pair (i, j): compute separation
 *     for each pair (i, j): compute alignment
 *     for each pair (i, j): compute cohesion
 *
 * That's 3 × 800² = 1.9M pair tests per tick.
 *
 * Better: ONE loop over pairs, with classify-by-distance
 * inside:
 *
 *     for each pair (i, j):
 *       d² = squared toroidal distance
 *       if d² < SEP_R²:    boid[i].sep_force += ... ; boid[j].sep_force += ...
 *       if d² < ALIGN_R²:  boid[i].ali_vsum  += boid[j].vel ; ali_n++
 *       if d² < COH_R²:    boid[i].coh_dsum  += toroidal_delta(j, i)
 *
 * One pair test per pair, three classifications per test.
 * Total: 800² = 640K pair tests = 1/3 the work.
 *
 * Crucial detail: keep ALL distance comparisons SQUARED until
 * the very end. sqrt() is more expensive than square. We only
 * sqrt at the moment we need a unit vector for the actual
 * force application.
 *
 * Same single-pass pattern is used in any large-N agent sim
 * (boids, n-body, swarm robotics).
 *
 * T4  HAWK STATE MACHINE — PATROL ↔ DIVE WITHOUT A TELEPORT
 * ─────────────────────────────────────────────────────────
 * The hawk has TWO states:
 *
 *   PATROL: orbit at radius 0.4 · min(w,h) around the world
 *           centre, angular speed 0.4 rad/s. Hawk position is
 *           a closed-form function of (centre, radius, phase).
 *
 *   DIVE:   set a straight-line velocity toward the flock
 *           CENTROID (computed at dive entry), step forward at
 *           250 px/s for HAWK_DIVE_DURATION (~1.5 s).
 *
 * Transition rule:
 *
 *     PATROL → DIVE       on SPACE press OR auto-dive timer.
 *                         Capture the flock centroid; capture
 *                         current hawk pos; dive_timer = duration.
 *
 *     DIVE → PATROL       when dive_timer ≤ 0. Resume PATROL
 *                         AT THE HAWK'S CURRENT POSITION (no
 *                         teleport back to the orbit). The
 *                         orbit phase is recomputed from the
 *                         hawk's actual position relative to
 *                         the centre, so the next patrol
 *                         starts smoothly from where the dive
 *                         left it.
 *
 * "Recompute orbit phase from current position" is the trick
 * that prevents teleport. Naïvely, PATROL has its own phase
 * variable that ticks regardless of DIVE; when you re-enter
 * PATROL, the phase is wherever it was paused, but the hawk
 * has moved miles away during DIVE. The hawk would visibly
 * jump back to the orbit.
 *
 * Solution: when re-entering PATROL, set
 *
 *     phase = atan2(hawk_pos.y - centre.y,
 *                   hawk_pos.x - centre.x)
 *
 * The orbit picks up exactly where the hawk currently is.
 * Continuous motion, no teleport.
 *
 * Same pattern works for any "predator suspends → resumes
 * passive behaviour" agent. shepherd.c uses an analogous
 * idea for the dog's relax state.
 *
 * T5  WHY THIS LOOKS LIKE REAL STARLINGS — EMERGENT DENSITY WAVES
 * ───────────────────────────────────────────────────────────────
 * Real starling murmurations exhibit several visual motifs:
 *
 *   - SHARP SILHOUETTES — the flock has a clear boundary; you
 *     can SEE its outline.
 *   - INTERNAL DENSITY WAVES — bands of higher density move
 *     through the flock at a different speed than the bulk.
 *   - SPLITTING + REJOINING — when a hawk dives through, the
 *     flock parts and re-coheres seconds later.
 *   - COLLECTIVE TURNS — the flock rotates as one organism.
 *
 * Our 800-boid simulation reproduces all four:
 *
 *   - Silhouette: density drops sharply at the edge of the
 *     cohesion radius. The renderer's RAMP makes the
 *     transition from `.` to nothing extremely visible.
 *
 *   - Internal waves: alignment + cohesion tug each boid
 *     simultaneously; the resulting forces are NOT spatially
 *     uniform. Local density variations propagate as waves —
 *     same as sound waves in fluids, except the medium is
 *     boids.
 *
 *   - Hawk dive: the FLEE force at HAWK_FLEE_RADIUS evicts
 *     boids; cohesion pulls neighbouring boids INTO the void
 *     once the hawk passes. Result: the flock fragments into
 *     sheets on dive entry, then reforms.
 *
 *   - Collective turn: alignment + cohesion both depend on
 *     the AVERAGE of neighbours. When some boids start
 *     turning (e.g. due to flee), the average direction
 *     shifts, propagating as a group turn.
 *
 * None of these motifs are CODED EXPLICITLY. They EMERGE from
 * three local rules. That's the lesson Reynolds (1987)
 * established and Couzin et al. (2002) quantified: complex
 * collective behaviour from simple local rules.
 *
 * Decision tree for "I want flock-like motion":
 *
 *   small N (≤50)   → individual glyphs, like flocking.c
 *   medium N (~150) → individual glyphs with brightness
 *                     coding, like crowd.c
 *   large N (≥500)  → density field rendering, like this file
 *
 * The simulation core is the same; the renderer changes with N.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_DEFAULT = 60,
    TARGET_FPS      = 60,
    HUD_COLS        = 96,
    FPS_UPDATE_MS   = 500,

    /*
     * Boid pool sizing.
     *
     * N_BOIDS_DEFAULT = 800: enough density that the binned grid
     *   shows clear `@`-core + falloff to `.` periphery on an 80x24
     *   terminal (1920 cells), without breaking sub-millisecond tick
     *   budget under O(N²) steering.
     *
     * N_BOIDS_MAX     = 1500: hard ceiling.  At 1500 the inner loop
     *   does 2.25 M pair tests/tick = 135 M/s — still well under the
     *   16 ms frame budget at 60 Hz on any modern CPU.
     *
     * BOID_STEP       = 100: chunk added/removed per `+`/`-` keypress.
     */
    N_BOIDS_DEFAULT = 800,
    N_BOIDS_MAX     = 1500,
    N_BOIDS_MIN     = 100,
    BOID_STEP       = 100,

    /*
     * Colour pair IDs (see §3 for actual colour values).
     *   Pairs 1-7 cycle the flock tint through the active theme.
     *   PAIR_HAWK is fixed (red, A_BOLD) regardless of theme so the
     *   predator always reads as the predator.
     *   PAIR_HUD/PAIR_HINT follow the project HUD spec.
     */
    N_COLORS        =   7,
    PAIR_HAWK       =   8,
    PAIR_HUD        =   9,
    PAIR_HINT       =  10,

    N_THEMES        =  10,

    /*
     * Density-grid hard cap — sized for very large terminals so
     * resize never overflows the buffer.  At MAX_ROWS=80, MAX_COLS=256
     * this is 80 × 256 × sizeof(int) = 80 KB of static BSS.
     */
    MAX_ROWS        =  80,
    MAX_COLS        = 256,
};

/* Cell dimensions — physics in pixel space, draw in cells. */
#define CELL_W   8
#define CELL_H  16

/* ── boid speeds (px/s) ─────────────────────────────────────────────── */
#define BOID_SPEED       90.0f   /* cruise speed used by force formulas    */
#define BOID_MAX_SPEED  150.0f   /* hard cap on |vel| each tick            */
#define BOID_MIN_SPEED   30.0f   /* floor — boids never stall completely   */
#define FLEE_SPEED      180.0f   /* effective speed scaling on hawk flee   */

/* ── boid radii (px) ────────────────────────────────────────────────── */
#define SEP_RADIUS       14.0f   /* personal-space bubble                  */
#define ALIGN_RADIUS     50.0f   /* heading-match neighbourhood            */
#define COHESION_RADIUS  80.0f   /* centre-of-mass neighbourhood           */
#define HAWK_FLEE_RADIUS 110.0f  /* boids flee within this disc of hawk    */

/* ── boid force weights ─────────────────────────────────────────────── *
 *  Order of magnitude reflects priority:
 *    FLEE   (4.5) overrides everything when hawk is close.
 *    SEP    (1.8) firm enough to stop overlap during panic.
 *    ALIGN  (1.0) nominal — produces the coherent direction of travel.
 *    COHERE (0.7) softer — flock stays loose, doesn't collapse to a point.
 *    MAX_STEER caps the per-tick acceleration so smooth curves emerge
 *    rather than instant pivots even when forces stack up.            */
#define W_SEP      1.8f
#define W_ALIGN    1.0f
#define W_COHERE   0.7f
#define W_FLEE     4.5f
#define MAX_STEER 130.0f

/* ── hawk parameters ────────────────────────────────────────────────── *
 *  HAWK_PATROL_FRAC   — orbit radius as fraction of min(world_w, world_h).
 *                       0.40 keeps the hawk visible but not constantly
 *                       inside the flock when at rest.
 *  HAWK_PATROL_OMEGA  — patrol angular speed (rad/s).  0.4 → 15.7 s/lap,
 *                       slow enough that the hawk reads as "watching".
 *  HAWK_DIVE_SPEED    — straight-line speed during dive (px/s).
 *                       250 vs BOID_MAX_SPEED 150 means the hawk
 *                       actually closes on the flock rather than chasing
 *                       it forever.
 *  HAWK_DIVE_DURATION — dive lasts this many seconds, regardless of
 *                       whether the hawk has reached the centroid.
 *  AUTO_DIVE_PERIOD   — when auto-dive is on, the next dive fires this
 *                       many seconds after the previous one ended.   */
#define HAWK_PATROL_FRAC      0.40f
#define HAWK_PATROL_OMEGA     0.4f
#define HAWK_DIVE_SPEED     250.0f
#define HAWK_DIVE_DURATION    1.5f
#define AUTO_DIVE_PERIOD      6.0f

/* Glyph ramp for density-to-character mapping.  Index 0 = density 1
 * (sparsest), index RAMP_LEN-1 = density ≥ RAMP_LEN (densest core).
 * Pure ASCII per CLAUDE.md "ASCII-Only Rendering" — no Unicode blocks. */
static const char DENSITY_RAMP[] = ".,:;oO*#@";
#define RAMP_LEN ((int)(sizeof DENSITY_RAMP - 1))

/* Timing primitives (verbatim from framework.c). */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/* clock_ns — monotonic wall-clock in ns; never goes backward. */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/* clock_sleep_ns — sleep ns nanoseconds; ns ≤ 0 returns immediately. */
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color — 10 themes for the flock + theme-independent hawk + HUD    */
/* ===================================================================== */

/*
 * Theme — one named flock palette.  body[0..N_COLORS-1] are xterm-256
 * indices that boids cycle through (bird i uses pair (i%N_COLORS)+1).
 *
 * Each theme is a TIGHT CLUSTER of 7 bright shades of one tint, not a
 * dim-to-bright gradient.  Reasoning: the renderer cycles through all
 * 7 pairs by spatial hash, so adjacent cells get DIFFERENT pairs.  If
 * the palette spans dim-to-bright, half the flock cells render dim
 * and the cloud reads as muddy.  Keeping all 7 entries in the same
 * bright band gives a clean tinted cloud with a subtle mottle from
 * the spatial hash, no dim spots.
 *
 * Brightness floor: all entries ≥ 81 (cube) or ≥ 119 (mint/peach) or
 * are full-saturation primaries (46, 196, 220, 226).  No values below
 * 80 — at A_DIM they'd vanish; we removed the A_DIM tier in
 * density_glyph but the floor remains a safety margin in case it's
 * ever re-enabled.
 */
typedef struct {
    const char *name;
    int         body[N_COLORS];
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name        bright cluster — all 7 entries are similar shades */
    {"Dusk",    { 180, 187, 188, 222, 223, 224, 230 }},  /* warm cream    */
    {"Sky",     {  81,  87, 111, 117, 123, 153, 159 }},  /* sky-blue      */
    {"Solar",   { 214, 220, 221, 222, 226, 227, 228 }},  /* warm yellow   */
    {"Aurora",  { 121, 122, 158, 159, 195, 207, 213 }},  /* mint → pink   */
    {"Ember",   { 196, 202, 208, 209, 214, 215, 220 }},  /* fire ramp     */
    {"Forest",  { 119, 120, 121, 154, 155, 156, 157 }},  /* leafy green   */
    {"Neon",    { 165, 171, 201, 207, 213, 219, 225 }},  /* hot magenta   */
    {"Sunset",  { 209, 210, 215, 216, 221, 222, 223 }},  /* warm orange   */
    {"Ghost",   { 247, 250, 252, 253, 254, 255, 231 }},  /* near-white    */
    {"Matrix",  {  46,  82, 118, 119, 120, 121, 156 }},  /* matrix green  */
};

/*
 * theme_apply — register the body palette with ncurses.  Background
 * = -1 (terminal default) per CLAUDE.md HUD spec.  Hawk and HUD pairs
 * are theme-independent and configured once in color_init().
 */
static void theme_apply(int idx)
{
    const Theme *th = &THEMES[idx];
    if (COLORS >= 256) {
        for (int p = 0; p < N_COLORS; p++)
            init_pair(p + 1, th->body[p], -1);
    } else {
        static const int fb8[N_COLORS] = {
            COLOR_WHITE, COLOR_WHITE, COLOR_CYAN,
            COLOR_CYAN,  COLOR_BLUE,  COLOR_BLUE,  COLOR_MAGENTA
        };
        for (int p = 0; p < N_COLORS; p++)
            init_pair(p + 1, fb8[p], -1);
    }
}

/*
 * color_init — one-time colour setup.
 *   start_color()        — initialise ncurses colour support.
 *   use_default_colors() — allow background = -1 (terminal default).
 *   theme_apply()        — register the initial flock palette.
 *   PAIR_HAWK            — bright red 196, A_BOLD on draw.
 *   PAIR_HUD             — bright yellow 226, A_BOLD on draw.
 *   PAIR_HINT            — bright cyan   51, A_BOLD on draw.
 */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();
    theme_apply(initial_theme);

    if (COLORS >= 256) {
        init_pair(PAIR_HAWK, 196, -1);
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_HAWK, COLOR_RED,    -1);
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell + Vec2 + toroidal_delta                       */
/* ===================================================================== */

/* Pixel-space world dimensions from terminal cell counts. */
static inline float pw(int cols) { return (float)(cols * CELL_W); }
static inline float ph(int rows) { return (float)(rows * CELL_H); }

/*
 * px_to_cell_x/y — round to nearest cell.  "Round half up" via
 * floor(p/dim + 0.5) avoids the half-pixel oscillation roundf()
 * would produce at exact boundaries.
 */
static inline int px_to_cell_x(float px) { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int px_to_cell_y(float py) { return (int)floorf(py / (float)CELL_H + 0.5f); }

/* Vec2 — 2-D float vector.  Tiny inline helpers; -O2 folds them away. */
typedef struct { float x, y; } Vec2;

static inline Vec2  v2(float x, float y)        { return (Vec2){x, y}; }
static inline Vec2  v2add(Vec2 a, Vec2 b)        { return v2(a.x+b.x, a.y+b.y); }
static inline Vec2  v2sub(Vec2 a, Vec2 b)        { return v2(a.x-b.x, a.y-b.y); }
static inline Vec2  v2scale(Vec2 v, float s)     { return v2(v.x*s, v.y*s); }
static inline float v2len(Vec2 v)                { return sqrtf(v.x*v.x + v.y*v.y); }
static inline float v2len2(Vec2 v)               { return v.x*v.x + v.y*v.y; }

/* v2norm — unit vector; returns zero if input is near-zero. */
static inline Vec2 v2norm(Vec2 v)
{
    float l = v2len(v);
    return (l > 0.001f) ? v2scale(v, 1.0f/l) : v2(0, 0);
}

/* v2clamp_len — cap |v| at max_len, preserve direction. */
static inline Vec2 v2clamp_len(Vec2 v, float max_len)
{
    float l = v2len(v);
    return (l > max_len) ? v2scale(v2norm(v), max_len) : v;
}

/*
 * toroidal_delta — shortest signed displacement from a to b on a
 * single axis of length `extent`.  On a wrap-around axis there are
 * two paths between any two points; we always return the shorter
 * one with sign indicating direction.
 *
 * Example (extent=100): a=5, b=95 → direct=+90, wrap=-10 → returns -10.
 * Used by every steering force so neighbours across the wrap are
 * correctly seen as close — without this, the flock would tear at edges.
 */
static inline float toroidal_delta(float a, float b, float extent)
{
    float d = b - a;
    if (d >  extent * 0.5f) d -= extent;
    if (d < -extent * 0.5f) d += extent;
    return d;
}

static inline float randf(void) { return (float)rand() / (float)RAND_MAX; }

/* ===================================================================== */
/* §5  boid                                                               */
/* ===================================================================== */

/*
 * Boid — one bird's complete state.
 *
 * Field groups:
 *   - kinematic state (pos / prev_pos / vel) is rewritten every tick.
 *   - identity (color_pair) is set once at spawn.  Note: the renderer
 *     uses the FLOCK's per-cell density tint (cycling agent color
 *     through the active theme), not the per-bird color_pair, so this
 *     field is mostly cosmetic for HUD identity.
 */
typedef struct {
    /* kinematic state — alpha-lerp candidate; renderer omits the lerp
     * because density rendering is itself stable across alpha values */
    Vec2 pos;
    Vec2 prev_pos;
    Vec2 vel;

    /* identity — set once at spawn */
    int  color_pair;
} Boid;

/*
 * boid_spawn — random pos in world, random direction at BOID_SPEED·0.5
 * so the flock isn't frozen on frame one.
 */
static void boid_spawn(Boid *b, int id, float ww, float wh)
{
    b->pos      = v2(randf() * ww, randf() * wh);
    b->prev_pos = b->pos;

    float ang = randf() * 2.0f * (float)M_PI;
    b->vel    = v2(cosf(ang) * BOID_SPEED * 0.5f,
                   sinf(ang) * BOID_SPEED * 0.5f);

    b->color_pair = (id % N_COLORS) + 1;
}

/*
 * boid_clamp_speed — enforce MIN_SPEED ≤ |vel| ≤ MAX_SPEED.
 *
 * Without a floor, boids stall when sep + ali + coh roughly cancel.
 * Without a ceiling, the flee force during a hawk dive can blow up
 * |vel| past anything visually useful.  Both clamps are needed.
 */
static void boid_clamp_speed(Boid *b)
{
    float l = v2len(b->vel);
    if (l < 0.001f) {
        /* Truly zero — nudge forward in a random direction. */
        float ang = randf() * 2.0f * (float)M_PI;
        b->vel = v2(cosf(ang) * BOID_MIN_SPEED,
                    sinf(ang) * BOID_MIN_SPEED);
        return;
    }
    if (l < BOID_MIN_SPEED) {
        b->vel = v2scale(v2norm(b->vel), BOID_MIN_SPEED);
    } else if (l > BOID_MAX_SPEED) {
        b->vel = v2scale(v2norm(b->vel), BOID_MAX_SPEED);
    }
}

/*
 * boid_wrap — toroidal boundary.  A bird leaving one edge re-enters
 * at the opposite edge.  Uniform flocking everywhere — no "edge of
 * the world" where birds bunch up.
 */
static void boid_wrap(Boid *b, float ww, float wh)
{
    if (b->pos.x <  0.0f) b->pos.x += ww;
    if (b->pos.x >= ww)   b->pos.x -= ww;
    if (b->pos.y <  0.0f) b->pos.y += wh;
    if (b->pos.y >= wh)   b->pos.y -= wh;
}

/* ===================================================================== */
/* §6  forces — single-pass O(N²) sep + alignment + cohesion             */
/* ===================================================================== */

/*
 * align_force — convert alignment accumulator (sum of neighbour
 * velocities + count) into a Reynolds-style "steer toward mean
 * heading" force.  Returns zero if no neighbours were in range.
 */
static Vec2 align_force(Vec2 ali_vsum, int ali_n, Vec2 my_vel)
{
    if (ali_n == 0) return v2(0, 0);
    Vec2 avg = v2scale(ali_vsum, 1.0f / (float)ali_n);
    return v2sub(avg, my_vel);
}

/*
 * cohere_force — convert cohesion accumulator (sum of toroidal
 * offsets toward neighbours + count) into a "seek mean position"
 * force.  We accumulate OFFSETS, not absolute positions, because
 * naïve averaging across the toroidal wrap would put the centroid
 * in the middle of the screen even when birds are clustered at
 * opposite edges.
 */
static Vec2 cohere_force(Vec2 coh_dsum, int coh_n, Vec2 my_vel)
{
    if (coh_n == 0) return v2(0, 0);
    Vec2 avg_off = v2scale(coh_dsum, 1.0f / (float)coh_n);
    Vec2 desired = v2scale(v2norm(avg_off), BOID_SPEED);
    return v2sub(desired, my_vel);
}

/*
 * hawk_flee_force — repulsion from the hawk with linear falloff
 * inside HAWK_FLEE_RADIUS, zero outside.  Magnitude scales toward
 * FLEE_SPEED at the wall (d → 0).  Uses toroidal_delta so a hawk
 * across the wrap still scares its neighbours.
 */
static Vec2 hawk_flee_force(Vec2 me_pos, Vec2 hawk_pos, float ww, float wh)
{
    float hx  = toroidal_delta(me_pos.x, hawk_pos.x, ww);
    float hy  = toroidal_delta(me_pos.y, hawk_pos.y, wh);
    float hd2 = hx*hx + hy*hy;
    if (hd2 >= HAWK_FLEE_RADIUS * HAWK_FLEE_RADIUS || hd2 <= 1e-6f)
        return v2(0, 0);
    float hd       = sqrtf(hd2);
    float strength = (HAWK_FLEE_RADIUS - hd) / HAWK_FLEE_RADIUS;
    return v2scale(v2(-hx / hd, -hy / hd), strength * FLEE_SPEED);
}

/*
 * boid_forces — compute the total steering force for one bird.
 *
 * One O(N²) inner loop visits every other bird and accumulates raw
 * contributions to separation / alignment / cohesion in a single
 * pass.  This is ~3× faster than three separate loops because
 * toroidal_delta + dist² are computed once per pair and reused.
 *
 * The per-rule accumulators are then converted to forces by the
 * helpers above; the hawk flee is computed once (per bird, not per
 * pair) and the four forces are summed with weights.
 *
 * Reads only — no writes to pool[].  scene_tick captures all
 * new_vel[] before writing back, so the update is "simultaneous"
 * across the whole flock (no order-of-iteration drift).
 */
static Vec2 boid_forces(const Boid *pool, int n, int self,
                        Vec2 hawk_pos, float ww, float wh)
{
    const Boid *me = &pool[self];

    /* Per-rule accumulators */
    Vec2 sep_force = v2(0, 0);
    Vec2 ali_vsum  = v2(0, 0); int ali_n = 0;
    Vec2 coh_dsum  = v2(0, 0); int coh_n = 0;   /* OFFSETS, not absolute */

    const float SEP_R2   = SEP_RADIUS      * SEP_RADIUS;
    const float ALIGN_R2 = ALIGN_RADIUS    * ALIGN_RADIUS;
    const float COH_R2   = COHESION_RADIUS * COHESION_RADIUS;

    for (int j = 0; j < n; j++) {
        if (j == self) continue;
        float dx = toroidal_delta(me->pos.x, pool[j].pos.x, ww);
        float dy = toroidal_delta(me->pos.y, pool[j].pos.y, wh);
        float d2 = dx*dx + dy*dy;
        if (d2 > COH_R2 || d2 < 1e-6f) continue;

        /* COHESION — every neighbour inside cohesion radius */
        coh_dsum.x += dx;
        coh_dsum.y += dy;
        coh_n++;

        /* ALIGNMENT — neighbours inside the (smaller) align radius */
        if (d2 < ALIGN_R2) {
            ali_vsum.x += pool[j].vel.x;
            ali_vsum.y += pool[j].vel.y;
            ali_n++;
        }

        /* SEPARATION — only very-close neighbours */
        if (d2 < SEP_R2) {
            float d        = sqrtf(d2);
            float strength = (SEP_RADIUS - d) / SEP_RADIUS;
            sep_force.x   -= (dx / d) * strength * BOID_SPEED;
            sep_force.y   -= (dy / d) * strength * BOID_SPEED;
        }
    }

    /* Accumulators → final per-rule forces */
    Vec2 ali  = align_force    (ali_vsum, ali_n, me->vel);
    Vec2 coh  = cohere_force   (coh_dsum, coh_n, me->vel);
    Vec2 flee = hawk_flee_force(me->pos, hawk_pos, ww, wh);

    /* Sum weighted forces, clamp to MAX_STEER for smooth curves */
    Vec2 total = v2add(
        v2add(v2scale(sep_force, W_SEP),
              v2scale(ali,       W_ALIGN)),
        v2add(v2scale(coh,       W_COHERE),
              v2scale(flee,      W_FLEE))
    );
    return v2clamp_len(total, MAX_STEER);
}

/* ===================================================================== */
/* §7  hawk — PATROL / DIVE controller                                    */
/* ===================================================================== */

/*
 * HawkMode — which behaviour the controller picked this tick.
 *   PATROL  : orbit the world centre at HAWK_PATROL_FRAC · min(w,h),
 *             angular speed HAWK_PATROL_OMEGA.  Stays out of the way
 *             except for occasional fly-throughs when the flock
 *             happens to drift across the orbit.
 *   DIVE    : straight-line sprint at HAWK_DIVE_SPEED toward the
 *             flock centroid for HAWK_DIVE_DURATION seconds.  After
 *             the dive expires, snap back to PATROL with patrol_phase
 *             updated so the orbit resumes from the hawk's current
 *             position (no teleport).
 */
typedef enum { HAWK_PATROL = 0, HAWK_DIVE } HawkMode;

typedef struct {
    /* kinematic state */
    Vec2 pos;
    Vec2 vel;        /* used only during DIVE; PATROL recomputes pos */

    /* controller state */
    HawkMode mode;
    float    patrol_phase;    /* radians around world centre        */
    float    dive_timer;      /* seconds remaining in current dive  */

    /* user-facing flag (HUD + auto-dive) */
    bool     auto_dive;
    float    auto_dive_timer; /* seconds since last (auto)-fired dive */
} Hawk;

/*
 * hawk_dive — kick off a dive toward `target` (typically the flock
 * centroid).  No-op if the hawk is already mid-dive.
 *
 *   pick the direction TARGET − POS;
 *   set vel = direction · HAWK_DIVE_SPEED;
 *   start dive_timer = HAWK_DIVE_DURATION;
 *   mode = DIVE.
 *
 * The dive moves in a straight line and IGNORES toroidal wrap on the
 * direction calculation — if the centroid wraps across an edge, the
 * hawk will fly the long way around.  This is fine because PATROL
 * resumes immediately after dive_timer expires, and the hawk's orbit
 * will re-centre.
 */
static void hawk_dive(Hawk *h, Vec2 target)
{
    if (h->mode == HAWK_DIVE) return;
    Vec2 dir = v2norm(v2sub(target, h->pos));
    if (v2len(dir) < 0.001f) return;       /* degenerate: hawk on target */

    h->vel        = v2scale(dir, HAWK_DIVE_SPEED);
    h->dive_timer = HAWK_DIVE_DURATION;
    h->mode       = HAWK_DIVE;
}

/*
 * hawk_step — advance the hawk one tick.
 *
 *   PATROL : patrol_phase += HAWK_PATROL_OMEGA · dt;
 *            pos = world_centre + r · (cos phase, sin phase)
 *   DIVE   : pos += vel · dt;  dive_timer -= dt
 *            on dive_timer ≤ 0:
 *              mode = PATROL
 *              patrol_phase = atan2(pos − world_centre)  (orbit resumes
 *                from the hawk's CURRENT position — no teleport)
 *
 * Toroidal wrap is applied at the end, so the hawk also reappears
 * from the opposite edge if it punches off-screen during a dive.
 */
static void hawk_step(Hawk *h, Vec2 world_centre, float patrol_r,
                      float ww, float wh, float dt)
{
    if (h->mode == HAWK_DIVE) {
        h->pos = v2add(h->pos, v2scale(h->vel, dt));
        h->dive_timer -= dt;
        if (h->dive_timer <= 0.0f) {
            h->mode = HAWK_PATROL;
            /* Re-derive patrol_phase from current position so orbit
             * resumes smoothly without teleport. */
            h->patrol_phase = atan2f(h->pos.y - world_centre.y,
                                      h->pos.x - world_centre.x);
        }
    } else {
        h->patrol_phase += HAWK_PATROL_OMEGA * dt;
        h->pos = v2add(world_centre,
                       v2scale(v2(cosf(h->patrol_phase),
                                  sinf(h->patrol_phase)), patrol_r));
    }

    /* Toroidal wrap — keeps the hawk on-screen during long dives. */
    if (h->pos.x <  0.0f) h->pos.x += ww;
    if (h->pos.x >= ww)   h->pos.x -= ww;
    if (h->pos.y <  0.0f) h->pos.y += wh;
    if (h->pos.y >= wh)   h->pos.y -= wh;
}

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

/*
 * Scene — flock + hawk + world dimensions + user mode flags.
 *
 * Field groups:
 *   - flock pool (touched every tick by every steering rule).
 *   - hawk (singleton predator).
 *   - active selection (theme).
 *   - world dimensions + flock centroid (refreshed each tick).
 *   - user mode flags (paused).
 */
typedef struct {
    /* flock pool — first n_birds slots active */
    Boid  pool[N_BOIDS_MAX];
    int   n_birds;

    /* hawk */
    Hawk  hawk;

    /* active theme */
    int   theme_idx;

    /* world dimensions in pixels — refreshed each tick */
    float world_w, world_h;

    /* HUD-only derived state */
    Vec2  flock_centroid;

    /* user mode flags */
    bool  paused;
} Scene;

/*
 * scene_init — fresh scene with random birds and a hawk parked at the
 * east edge of its patrol orbit.  Theme index is preserved across
 * reset (saved before memset, restored after).
 */
static void scene_init(Scene *s, int cols, int rows)
{
    int saved_theme = s->theme_idx;
    memset(s, 0, sizeof *s);
    s->theme_idx = saved_theme;

    s->world_w = pw(cols);
    s->world_h = ph(rows);
    s->n_birds = N_BOIDS_DEFAULT;

    for (int i = 0; i < N_BOIDS_MAX; i++)
        boid_spawn(&s->pool[i], i, s->world_w, s->world_h);

    /* Hawk patrol radius depends on world size; placed at east edge of orbit. */
    float min_dim = (s->world_w < s->world_h) ? s->world_w : s->world_h;
    float r       = min_dim * HAWK_PATROL_FRAC;
    Vec2  centre  = v2(s->world_w * 0.5f, s->world_h * 0.5f);

    s->hawk.pos             = v2add(centre, v2(r, 0));
    s->hawk.vel             = v2(0, 0);
    s->hawk.mode            = HAWK_PATROL;
    s->hawk.patrol_phase    = 0.0f;
    s->hawk.dive_timer      = 0.0f;
    s->hawk.auto_dive       = false;
    s->hawk.auto_dive_timer = 0.0f;

    s->flock_centroid = centre;
    s->paused         = false;
}

/*
 * scene_centroid — toroidal-aware flock centre of mass.  Used by the
 * hawk dive to choose a target.  For a small flock on a large world
 * this is just the average position; on small terminals (where the
 * flock can wrap around) we need to pick a reference and accumulate
 * toroidal offsets relative to it.
 */
static Vec2 scene_centroid(const Boid *pool, int n, float ww, float wh)
{
    if (n <= 0) return v2(ww * 0.5f, wh * 0.5f);
    Vec2 ref = pool[0].pos;
    Vec2 sum = v2(0, 0);
    for (int i = 0; i < n; i++) {
        sum.x += toroidal_delta(ref.x, pool[i].pos.x, ww);
        sum.y += toroidal_delta(ref.y, pool[i].pos.y, wh);
    }
    Vec2 mean_off = v2scale(sum, 1.0f / (float)n);
    Vec2 c        = v2add(ref, mean_off);
    /* Wrap result back into [0, w) × [0, h). */
    if (c.x <  0.0f) c.x += ww;
    if (c.x >= ww)   c.x -= ww;
    if (c.y <  0.0f) c.y += wh;
    if (c.y >= wh)   c.y -= wh;
    return c;
}

/*
 * scene_tick — one fixed-step physics update.
 *
 * Order matters:
 *   1. If auto-dive is on, increment its timer; fire a dive if due.
 *   2. Hawk: step controller (patrol orbit or dive integration).
 *   3. Boids: two-stage update so all read OLD positions.
 *      Stage A: compute new_vel[i] = clamp(vel + force·dt, MAX_SPEED).
 *      Stage B: write new_vel back, integrate pos, wrap toroidally.
 *   4. Recompute flock centroid for HUD + future dives.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    /* (1) auto-dive trigger */
    if (s->hawk.auto_dive && s->hawk.mode == HAWK_PATROL) {
        s->hawk.auto_dive_timer += dt;
        if (s->hawk.auto_dive_timer >= AUTO_DIVE_PERIOD) {
            s->hawk.auto_dive_timer = 0.0f;
            hawk_dive(&s->hawk, s->flock_centroid);
        }
    }

    /* (2) hawk step */
    Vec2  centre  = v2(s->world_w * 0.5f, s->world_h * 0.5f);
    float min_dim = (s->world_w < s->world_h) ? s->world_w : s->world_h;
    hawk_step(&s->hawk, centre, min_dim * HAWK_PATROL_FRAC,
              s->world_w, s->world_h, dt);

    /* (3a) read OLD positions, compute new velocities */
    static Vec2 new_vel[N_BOIDS_MAX];   /* file-scope: keep stack small */
    for (int i = 0; i < s->n_birds; i++) {
        Vec2 force = boid_forces(s->pool, s->n_birds, i,
                                 s->hawk.pos, s->world_w, s->world_h);
        Vec2 v     = v2add(s->pool[i].vel, v2scale(force, dt));
        new_vel[i] = v2clamp_len(v, BOID_MAX_SPEED);
    }

    /* (3b) commit velocities, integrate, wrap, clamp speed */
    for (int i = 0; i < s->n_birds; i++) {
        Boid *b = &s->pool[i];
        b->vel       = new_vel[i];
        b->prev_pos  = b->pos;
        b->pos       = v2add(b->pos, v2scale(b->vel, dt));
        boid_wrap(b, s->world_w, s->world_h);
        boid_clamp_speed(b);
    }

    /* (4) refresh centroid for next-frame HUD and any future dive */
    s->flock_centroid = scene_centroid(s->pool, s->n_birds,
                                       s->world_w, s->world_h);
}

/* ── render ──────────────────────────────────────────────────────────── */

/*
 * mark_cell — stamp one ASCII glyph at terminal cell (cx, cy).
 *
 * Centralises the (chtype)(unsigned char) cast plus bounds-check that
 * would otherwise be repeated at every mvwaddch site.  The double
 * cast prevents sign-extension on values > 127 (per CLAUDE.md
 * "Common ncurses Bugs").  Off-screen cells are silently dropped.
 */
static void mark_cell(WINDOW *w, int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/*
 * Per-frame density buffer.  Static so we don't re-allocate every
 * frame and don't put 80 KB on the stack.  Sized to MAX_ROWS × MAX_COLS
 * to handle very large terminals; scene_draw zeroes only the active
 * region (rows × cols), not the whole buffer.
 */
static int g_density[MAX_ROWS][MAX_COLS];

/*
 * density_glyph — pick the glyph + brightness tier for a given count.
 *
 *   density 0      : not drawn (caller skips)
 *   density 1-4    : A_NORMAL, glyphs '.' ',' ':' ';'
 *   density 5+     : A_BOLD,   glyphs 'o' 'O' '*' '#' '@' (saturating)
 *
 * No A_DIM tier — at A_DIM most terminals render colours at ~half
 * intensity, and since most cells in the flock have density 1 or 2
 * the periphery would fade to near-black.  The visual fade-to-edge
 * comes from the GLYPH ramp ('.' for sparse → '@' for dense), not
 * from brightness modulation; using only A_NORMAL and A_BOLD keeps
 * every drawn cell legible while still giving the bright core its
 * characteristic A_BOLD glow.
 */
static void density_glyph(int density, char *out_ch, attr_t *out_attr)
{
    int idx = density - 1;
    if (idx < 0)            idx = 0;
    if (idx >= RAMP_LEN)    idx = RAMP_LEN - 1;
    *out_ch = DENSITY_RAMP[idx];

    *out_attr = (density >= 5) ? A_BOLD : A_NORMAL;
}

/*
 * scene_draw — paint the frame in painter's order:
 *
 *   1. Bin every bird into the density grid (skipping out-of-bounds).
 *   2. For each non-empty cell, stamp the corresponding glyph from the
 *      ASCII ramp with its brightness tier; the colour pair cycles
 *      through pairs 1..N_COLORS keyed by cell position so the flock
 *      reads as a coherent tinted cloud rather than monochrome.
 *   3. Hawk on top — always A_BOLD red `!`.
 *
 * The flock tint per cell uses (cy * 7 + cx) % N_COLORS — a cheap
 * spatial hash that gives the cloud a soft mottled colour without
 * recomputing per-bird identity.
 */
static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows)
{
    /* Clip to buffer dimensions. */
    int cap_rows = (rows < MAX_ROWS) ? rows : MAX_ROWS;
    int cap_cols = (cols < MAX_COLS) ? cols : MAX_COLS;

    /* (1) zero active region of density grid */
    for (int r = 0; r < cap_rows; r++)
        for (int c = 0; c < cap_cols; c++)
            g_density[r][c] = 0;

    /* (1) bin birds into density grid */
    for (int i = 0; i < s->n_birds; i++) {
        int cx = px_to_cell_x(s->pool[i].pos.x);
        int cy = px_to_cell_y(s->pool[i].pos.y);
        if (cx < 0 || cx >= cap_cols || cy < 0 || cy >= cap_rows) continue;
        g_density[cy][cx]++;
    }

    /* (2) render: glyph + brightness tier per non-empty cell */
    for (int cy = 0; cy < cap_rows; cy++) {
        for (int cx = 0; cx < cap_cols; cx++) {
            int d = g_density[cy][cx];
            if (d == 0) continue;
            char   ch;
            attr_t attr;
            density_glyph(d, &ch, &attr);
            int pair = ((cy * 7 + cx) % N_COLORS) + 1;
            mark_cell(w, cx, cy, ch, pair, attr, cols, rows);
        }
    }

    /* (3) hawk on top */
    {
        int cx = px_to_cell_x(s->hawk.pos.x);
        int cy = px_to_cell_y(s->hawk.pos.y);
        char ch = (s->hawk.mode == HAWK_DIVE) ? '!' : 'X';
        mark_cell(w, cx, cy, ch, PAIR_HAWK, A_BOLD, cols, rows);
    }
}

/* ===================================================================== */
/* §9  app — screen + signals + main loop                                 */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s, int initial_theme)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(initial_theme);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw — compose one frame: density-rendered flock + hawk
 * + HUD bars.  HUD top-right (PAIR_HUD bright yellow A_BOLD) shows
 * fps + sim Hz + bird count + theme + hawk mode.  Bottom-left
 * (PAIR_HINT bright cyan A_BOLD) shows the key-binding strip.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    /* Top-right status — PAIR_HUD bright yellow, A_BOLD */
    char buf[HUD_COLS + 1];
    const char *hawk_mode = (sc->hawk.mode == HAWK_DIVE) ? "DIVE  " : "PATROL";
    const char *suffix    = sc->paused ? "  PAUSED"
                          : sc->hawk.auto_dive ? "  AUTO" : "";
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%3d Hz  n:%d/%d  [%s]  hawk:%s%s ",
             fps, sim_fps, sc->n_birds, N_BOIDS_MAX,
             THEMES[sc->theme_idx].name, hawk_mode, suffix);
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom-left key hint — PAIR_HINT bright cyan, A_BOLD per spec */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:dive  h:auto-dive  p:pause  r:reset  t/T:theme  +/-:birds ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── App + signal handlers ──────────────────────────────────────────── */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Scene *sc  = &app->scene;
    sc->world_w = pw(app->screen.cols);
    sc->world_h = ph(app->screen.rows);

    /* Clamp every bird and the hawk into the new world bounds. */
    for (int i = 0; i < sc->n_birds; i++) {
        Boid *b = &sc->pool[i];
        if (b->pos.x >= sc->world_w) b->pos.x = sc->world_w - 1.0f;
        if (b->pos.y >= sc->world_h) b->pos.y = sc->world_h - 1.0f;
        b->prev_pos = b->pos;
    }
    if (sc->hawk.pos.x >= sc->world_w) sc->hawk.pos.x = sc->world_w - 1.0f;
    if (sc->hawk.pos.y >= sc->world_h) sc->hawk.pos.y = sc->world_h - 1.0f;

    app->need_resize = 0;
}

/*
 * app_handle_key — dispatch one keypress; return false to quit.
 *
 *   q / Q / ESC    quit
 *   space          fire one hawk dive at the flock centroid
 *   h / H          toggle auto-dive (every AUTO_DIVE_PERIOD seconds)
 *   p / P          pause / resume
 *   r / R          reset (theme preserved)
 *   t              cycle to next theme; T cycles backward
 *   + / =          add BOID_STEP birds (cap N_BOIDS_MAX)
 *   -              remove BOID_STEP birds (floor N_BOIDS_MIN)
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':  hawk_dive(&sc->hawk, sc->flock_centroid);          break;
    case 'h': case 'H':
        sc->hawk.auto_dive       = !sc->hawk.auto_dive;
        sc->hawk.auto_dive_timer = 0.0f;
        break;
    case 'p': case 'P': sc->paused = !sc->paused;                 break;
    case 'r': case 'R':
        scene_init(sc, app->screen.cols, app->screen.rows);       break;
    case 't':
        sc->theme_idx = (sc->theme_idx + 1) % N_THEMES;
        theme_apply(sc->theme_idx);
        break;
    case 'T':
        sc->theme_idx = (sc->theme_idx + N_THEMES - 1) % N_THEMES;
        theme_apply(sc->theme_idx);
        break;
    case '+': case '=':
        sc->n_birds += BOID_STEP;
        if (sc->n_birds > N_BOIDS_MAX) sc->n_birds = N_BOIDS_MAX;
        break;
    case '-':
        sc->n_birds -= BOID_STEP;
        if (sc->n_birds < N_BOIDS_MIN) sc->n_birds = N_BOIDS_MIN;
        break;
    default: break;
    }
    return true;
}

/*
 * main — game loop.  Identical structure to the project framework:
 *   ① resize check → ② measure dt → ③ fixed-step accumulator →
 *   ④ fps counter  → ⑤ frame cap (sleep BEFORE render) →
 *   ⑥ draw + present → ⑦ drain input.
 *
 * The frame cap uses `clock_ns() − frame_start` (no `+ dt`) — adding
 * dt back into elapsed cancels the cap and pegs CPU at 100 %.
 */
int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen, 0 /* initial theme = Dusk */);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        int64_t frame_start = clock_ns();

        /* ① resize */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ② dt */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ③ fixed-step accumulator */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* ④ fps counter (500 ms window) */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ⑤ frame cap — sleep BEFORE render */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        /* ⑥ draw + present */
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* ⑦ drain input */
        int key = getch();
        if (key != ERR && !app_handle_key(app, key))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
