/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * tectonic.c
 *   — Procedural tectonic worldmap. Place N plate seeds; Voronoi-
 *     assign every cell to a plate; give each plate a velocity and
 *     a type (oceanic / continental); classify each plate boundary
 *     from the relative velocity (convergent → mountains, divergent
 *     → rifts, transform → faults); modulate elevation by distance
 *     to the nearest boundary; add Perlin fBm for organic detail;
 *     bin elevation into eight biomes from deep ocean to snowy peak.
 *     The result is a continent + ocean map that feels geological,
 *     not random.
 *
 * DEMO: A world appears: blue oceans, green plains, brown mountains,
 *       white peaks. The shape isn't accidental — mountain chains run
 *       along convergent plate boundaries (where two plates push into
 *       each other), trenches and rift seas trace divergent
 *       boundaries (where plates pull apart), and the coastlines
 *       follow the underlying Voronoi geometry. Cycle FIFTEEN map views
 *       of the same generated world with n / p:
 *
 *         WORLD      biome map — the finished world
 *         PLATES     coloured Voronoi cells, one tint per plate, seeds 'O'
 *         STRESS     boundary-type heatmap (convergent/divergent/transform)
 *         ELEVATION  pure height ramp — eight glyphs from '`' to '@'
 *         RELIEF     shaded relief — terrain lit from the north-west
 *         CONTOUR    topographic contour lines + coastline
 *         BATHYMETRY ocean-depth ramp — deep trenches darkest
 *         TEMP       climate by latitude + altitude (hot red, cold blue)
 *         MOISTURE   rainfall — a value-noise field with a rain shadow
 *         RUGGED     slope magnitude — mountain ranges & cliffs glow
 *         ASPECT     downhill-direction flow field (8 tinted headings)
 *         HEATFLOW   geothermal glow around plate boundaries
 *         NATIONS    plates as nations — borders + capital markers
 *         CONTINENT  land vs sea, continental vs oceanic crust
 *         DAY/NIGHT  an animated terminator sweeping the globe
 *
 *       After ~15 seconds the world is discarded and a new one
 *       builds from a fresh seed —
 *       different plate count, different layout, completely
 *       different geography but governed by the same physics.
 *
 * Study alongside:
 *   ../generational/voronoi_region_map.c
 *      — the same Voronoi machinery, used as a clean teaching
 *        example without the velocity / boundary / biome layer.
 *   ../worldgen/procedural_galaxy.c
 *      — also "world from a function", but in continuous polar
 *        coordinates instead of discrete tectonics.
 *
 * Section map (re-cut by CONCERN — see ARCHITECTURE block below):
 *   §1 CONFIG       — constants, enums, data tables, core world types
 *   §2 PERFORMANCE  — timing primitives (throttle policy lives in main)
 *   §3 LOGIC        — pure: hash, perlin/fbm, classify, cartographic math
 *   §4 SIMULATION   — world-gen pipeline + scene_tick (clock + regen timer)
 *   §5 EFFECTS      — cosmetic-only state (one-line note: none stored)
 *   §6 DELAYS       — pauses, holds, timers (one-line note: regen timer)
 *   §7 RENDER       — the 15 per-cell view renderers + dispatch + HUD
 *   §8 APP          — user events + per-tick combine + main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          regenerate world with a new seed
 *   n / N      next / prev pattern  (cycles all 15 map views)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      shorter regen interval (faster cycling)
 *   -          longer
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/worldgen/tectonic.c \
 *     -o tectonic -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Five stages, run in order whenever the world is
 *                  regenerated.
 *
 *                  (1) PLATE PLACEMENT — pick N ∈ [6, 14] plate seeds
 *                      via a jittered grid (same scheme as the
 *                      constellation file). One seed per grid cell at a
 *                      hash-driven offset; the result is organically
 *                      scattered without clusters. Each plate gets:
 *                        - position (x, y)
 *                        - random velocity (vx, vy), magnitude ~0.3-1
 *                        - random type ∈ {oceanic, continental}
 *                        - base elevation (oceanic = -0.5, continental
 *                          = +0.3) — the mean height that plate would
 *                          have with no boundary stress applied
 *
 *                  (2) VORONOI ASSIGNMENT — for each map cell, assign
 *                      it to the nearest plate (squared Euclidean,
 *                      with y multiplied by 2 to correct the cell-
 *                      aspect ratio so Voronoi cells look round).
 *                      The map is now partitioned into N regions,
 *                      each owned by exactly one plate.
 *
 *                  (3) BOUNDARY DETECTION & CLASSIFICATION — for
 *                      each cell, look at 4 neighbours; if any has
 *                      a different plate id, this cell is on a
 *                      plate boundary. Classify the boundary by
 *                      the RELATIVE VELOCITY of the two plates,
 *                      decomposed along their seed-to-seed normal:
 *                        approach = (v_a − v_b) · n_ab
 *                          where n_ab points from plate a to plate b
 *                          (and is aspect-corrected like the Voronoi)
 *                        - approach > +T  → CONVERGENT (plates close)
 *                        - approach < −T  → DIVERGENT  (plates pull apart)
 *                        - |approach| ≤ |perp|/k → TRANSFORM (parallel)
 *
 *                  (4) ELEVATION FIELD — for every cell, scan a small
 *                      aspect-corrected neighbourhood for the closest
 *                      boundary cell. The boundary type AND distance
 *                      drive a modifier:
 *                        CONVERGENT : +0.5 · (1 − d/R)   (mountains)
 *                        DIVERGENT  : −0.4 · (1 − d/R)   (rifts/trenches)
 *                        TRANSFORM  : 0                   (no vert. motion)
 *                      Final elevation = base_elev + modifier + Perlin
 *                      fBm noise (4 octaves), clamped to [-1, +1].
 *
 *                  (5) BIOME BINNING — eight elevation buckets:
 *                      DEEP_OCEAN < −0.55 < OCEAN < −0.20 < COAST < 0
 *                      < PLAINS < 0.15 < HILLS < 0.35 < MOUNTAINS
 *                      < 0.55 < HIGHLANDS < 0.75 < PEAKS.
 *
 *                  Once the world is built, fifteen PATTERNS render the
 *                  same data fifteen ways — the natural biome map, Voronoi
 *                  plate tinting, boundary heatmap, elevation ramp, plus
 *                  derived cartographic views (shaded relief, contours,
 *                  bathymetry, latitude climate, rainfall, slope, aspect,
 *                  geothermal heat, nations, continents, day/night).
 *
 * Data-structure : Cell { plate_id:1, boundary:1, elev:1, biome:1 } =
 *                  4 bytes. For a 240×80 map that's 76 KB. Plus a
 *                  small Plate[16] array. No allocation in steady
 *                  state — regen reuses the same buffer.
 *
 * Rendering      : ASCII only. Each pattern dispatches per cell:
 *                    WORLD     biome glyph (chosen by biome+hash) in
 *                              biome-ramp colour. Water shimmer cycles
 *                              the glyph by time; peaks twinkle; and
 *                              convergent mountain cells occasionally
 *                              flash '*' for volcanic activity.
 *                    PLATES    glyph by plate type ('~' / '#') in a
 *                              plate-id-cycled tint, plus 'O' markers
 *                              at each plate seed.
 *                    STRESS    boundary cells light up in their type's
 *                              accent (red / blue / accent); non-edge
 *                              cells render dimly as background.
 *                    ELEVATION 8-step density ramp ' .,:-^#@' coloured
 *                              by ramp index — pure height map.
 *
 * Performance    : Build is O(W·H · N_plates) for the Voronoi (the
 *                  dominant cost) — ~230 K ops for 240×80 × 12.
 *                  Boundary detection and elevation are O(W·H) and
 *                  O(W·H · R²) where R ≈ 6; under 2 M ops total.
 *                  Whole world builds in <10 ms on a current CPU.
 *                  Per-frame render is O(W·H) cell visits.
 *
 * References     :
 *
 *   World generation (the simulation) —
 *   • Wikipedia — "Plate tectonics"
 *     https://en.wikipedia.org/wiki/Plate_tectonics
 *   • Wikipedia — Convergent / Divergent / Transform boundaries
 *     https://en.wikipedia.org/wiki/Plate_boundaries
 *   • Wikipedia — "Voronoi diagram" (the plate-assignment partition)
 *     https://en.wikipedia.org/wiki/Voronoi_diagram
 *   • Lague, S. — "Tectonic Plate Simulation for Procedural Terrain"
 *     https://www.youtube.com/watch?v=x_Tn66PvTn4
 *   • Andy Gainey — "Procedural Worlds from Simple Tiles"
 *     https://experilous.com/1/blog/post/procedural-planet-generation
 *   • Perlin, K. (2002) — "Improving Noise" (the fBm scaffold + value_noise)
 *     https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *
 *   Cartographic rendering (the 15 map views) —
 *   • Horn, B.K.P. (1981) — "Hill Shading and the Reflectance Map", Proc. IEEE
 *     69(1):14-47. The 3×3 neighbour-gradient that powers RELIEF (NW hillshade),
 *     RUGGED (slope magnitude) and ASPECT (downhill heading) from a height grid.
 *   • Imhof, E. (1982) — "Cartographic Relief Presentation". The classic on
 *     depicting terrain: shaded relief, contour lines, and hypsometric /
 *     bathymetric tinting (RELIEF, CONTOUR, BATHYMETRY).
 *   • Whittaker, R.H. (1975) — biome classification by temperature × moisture;
 *     the idea behind the TEMP and MOISTURE views (here keyed to latitude +
 *     altitude + noise rather than measured climate).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To make a world look geological — not random — you don't draw the
 * mountains. You draw the FORCES that would make the mountains. Tile
 * the plane with plates, give each plate a velocity, look at where
 * the plates push or pull on each other, and let the elevation fall
 * out of those interactions. Add some noise for grit. The mountain
 * chains, ocean ridges, and coastlines emerge for free because you
 * drew the same physics nature uses.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a jigsaw puzzle floating on water. Each piece is a tectonic
 * plate. The pieces drift in different directions at different
 * speeds. Where two pieces are pushing INTO each other, the edges
 * crumple upward → mountains. Where two pieces are pulling APART,
 * a gap opens between them and water rushes in → rift / ocean ridge.
 * Where pieces SLIDE PAST each other, neither gap nor pile-up → a
 * fault zone. The interior of each piece is just whatever colour the
 * piece was — flat plains for continental plates, deep water for
 * oceanic plates. All the geography is in the EDGE INTERACTIONS.
 *
 * Now: for "jigsaw piece" read "Voronoi cell of a random seed", for
 * "drifting" read "random velocity vector", for "edges crumple" read
 * "+0.5 added to elevation within distance R of a convergent
 * boundary", and you have the algorithm. The whole world is just
 * Voronoi + per-plate constants + a 5-line classifier on the relative
 * velocity at every shared edge.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. PLACE N plate seeds in a jittered grid. For each:
 *       - position (x, y) in world cells
 *       - velocity (vx, vy), random direction, speed in [0.3, 1.0]
 *       - type ∈ {oceanic, continental}, 50/50
 *       - base_elev = oceanic ? −0.5 : +0.3
 *  2. ASSIGN each cell to its nearest plate seed (Voronoi). Use
 *     squared Euclidean distance with y · 2 for cell aspect.
 *  3. DETECT boundaries: for each cell, if any of its 4 neighbours
 *     belongs to a different plate, this is an edge cell.
 *  4. CLASSIFY each edge by the velocity of the two plates:
 *       n_ab    = (b.pos − a.pos) / |...|       boundary normal
 *       approach = (a.v − b.v) · n_ab
 *       if approach > +T     → CONVERGENT
 *       if approach < −T     → DIVERGENT
 *       else                 → TRANSFORM
 *  5. ELEVATION at each cell:
 *       for every cell within R of an edge, find its closest edge
 *       and apply the type-specific modifier (decayed by 1−d/R):
 *         CONVERGENT  : +0.5 · (1 − d/R)
 *         DIVERGENT   : −0.4 · (1 − d/R)
 *         TRANSFORM   : 0
 *       elev = plate.base_elev + modifier + 0.4 · (fbm − 0.5)
 *       clamp to [−1, +1].
 *  6. BIOME = bucket of elev. 8 buckets DEEP_OCEAN..PEAKS.
 *  7. RENDER per the active pattern. Repeat from step 1 every
 *     ~REGEN_SECONDS to bulldoze the world and rebuild from a new seed.
 *
 * KEY FORMULAS
 * ────────────
 *  Voronoi (per cell — choose closest plate):
 *      d²(plate_i)  = (x − pi.x)² + 4·(y − pi.y)²
 *      cell.plate   = argmin_i d²(plate_i)
 *
 *  Boundary classification:
 *      n  = ((b.x − a.x), 2·(b.y − a.y)) / |...|       // aspect normal
 *      approach = (a.vx − b.vx)·n.x + (a.vy − b.vy)·n.y
 *      perp     = √(|Δv|² − approach²)
 *      kind     = approach >  T   → CONVERGENT
 *               | approach < −T   → DIVERGENT
 *               | otherwise        → TRANSFORM
 *
 *  Elevation:
 *      d, t  = (distance, type) of nearest boundary cell within R
 *      mod   = match t with
 *                CONVERGENT → +0.5 · (1 − d/R)
 *                DIVERGENT  → −0.4 · (1 − d/R)
 *                TRANSFORM  →  0
 *      elev  = clamp(plate.base + mod + 0.4·(fbm(x·s, y·s)−0.5), −1, 1)
 *
 *  Biome buckets:
 *      e < −0.55 DEEP_OCEAN | < −0.20 OCEAN  | < 0    COAST
 *      | < +0.15 PLAINS     | < +0.35 HILLS  | < +0.55 MOUNTAINS
 *      | < +0.75 HIGHLANDS  | else PEAKS
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DEGENERATE NORMALS. If two plate seeds end up at identical (x, y)
 *    the boundary normal is undefined. Jittered-grid placement makes
 *    this practically impossible, but classify_boundary still guards
 *    against zero-length normals and falls back to TRANSFORM.
 *
 *  • ASPECT EVERYWHERE. Voronoi distance, boundary-normal direction,
 *    AND the elevation-modifier scan radius all need the y·2 aspect
 *    correction — otherwise plates look flat (squashed) and mountain
 *    chains form too easily in the y direction. Use it consistently.
 *
 *  • TRANSFORM MUST DOMINATE WHEN PERPENDICULAR. With the simple test
 *    "approach > T or approach < −T", every boundary becomes either
 *    convergent or divergent if T is tiny. We compare |approach| to
 *    the perpendicular magnitude — a boundary is only transform when
 *    the relative motion is mostly TANGENT to the boundary, not
 *    NORMAL to it. Otherwise small perpendicular velocities would
 *    make lots of lines look like faults.
 *
 *  • BOUNDARY-EFFECT RADIUS. Too small → mountains form only on the
 *    exact boundary cells, looking like a line drawing. Too large →
 *    mountains everywhere; the plate interior disappears. R ≈ 6 cells
 *    (with aspect) leaves a clear "coast plus inland" gradient.
 *
 *  • ELEVATION CLAMPING. Without clamp, divergent boundaries on top
 *    of an already-oceanic plate (base −0.5) plus the −0.2 noise
 *    floor can produce elev = −1.1, which falls outside the biome
 *    table. Always clamp post-modifier.
 *
 *  • PLATE COUNT VS MAP SIZE. Too many plates on a small map and the
 *    Voronoi cells become smaller than the boundary radius — every
 *    cell is "near a boundary", elevation washes out. Cap plates at
 *    14 and require ≥ 6 cells of plate radius on the smallest map.
 *
 *  • REGEN PERFORMANCE. The plate-classification loop's inner Voronoi
 *    is O(W·H · N). For 240·80·14 ≈ 270 K ops; fine. If you raise N
 *    past ~30 a spatial index becomes worth it.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • PAUSE during HOLD: the world freezes. Resume: animation
 *    continues exactly where it stopped (water shimmer, peak
 *    twinkle). Verifies the fixed-step accumulator.
 *
 *  • Press 'r': regenerate. The new world has a different
 *    plate count and layout but the SAME biome distribution
 *    statistics (e.g. roughly half ocean, half land for the default
 *    50/50 oceanic-continental split).
 *
 *  • PLATES pattern: every cell of the same plate has the same tint
 *    AND glyph; cells on the boundary are highlighted. Plate seeds
 *    are visible as bright 'O' markers — you should be able to find
 *    one inside every Voronoi cell.
 *
 *  • STRESS pattern: the boundary cells form a network of lines
 *    crossing the map. Convergent edges (red) should dominate where
 *    plates head into each other; divergent (blue) where they pull
 *    apart. Transform (yellow) is rarer but appears between plates
 *    moving roughly parallel.
 *
 *  • WORLD pattern: walk the eye along an EXTENDED CONVERGENT
 *    boundary in STRESS — the same line in WORLD should host a
 *    chain of MOUNTAINS / PEAKS. Walk a DIVERGENT line — same
 *    location should be DEEP_OCEAN / OCEAN. This is the visual
 *    proof the elevation modifier is wired up correctly.
 *
 *  • ELEVATION pattern: a smooth gradient with no plate-tint
 *    structure — just heights. Compare against WORLD: the same
 *    high-elevation cells should map to MOUNTAINS / PEAKS biomes.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * Re-cut from first principles into separated concern-layers (a SEPARATION
 * pass: RELOCATE + LABEL only — every function body is byte-identical, nothing
 * renamed). The world is GENERATED once per reset and then only viewed, so §4
 * SIMULATION is a one-shot pipeline plus a tiny per-tick timer, while §7 RENDER
 * is large (15 map views). Layer → section → what it mutates:
 *
 *   LAYER        §   MUTATES
 *   ─────────────────────────────────────────────────────────────────────
 *   CONFIG       §1  nothing — compile-time constants, the Biome / Boundary /
 *                    Plate / Pattern enums, const data tables (glyphs, ramp,
 *                    themes) and the core world types (Plate, Cell, World),
 *                    relocated up so §3/§4 can see them.
 *   PERFORMANCE  §2  nothing — clock_ns / clock_sleep_ns are pure timers; the
 *                    frame cap + fixed-timestep accumulator are POLICY in main.
 *   LOGIC        §3  nothing — the spatial hash + Perlin/fBm samplers, widx, the
 *                    boundary classifier, elev→biome, the pure generation
 *                    helpers (jitter_offset, nearest_boundary, boundary_uplift),
 *                    the cartographic math (clampi / elev_at / value_noise), the
 *                    biome census and pattern_name. All pure; perm[] reseeded
 *                    only from §4.
 *   SIMULATION   §4  World.{plates,cells,n_plates,seed} + the global perm[], and
 *                    Scene.{world,paused,speed,current_theme,current_pattern,
 *                    time_secs,regen_t}. world_build runs the generation pipeline
 *                    (plates→Voronoi→boundaries→elevation) on reset; scene_tick
 *                    advances the clock + regen countdown. The ONLY writers.
 *   EFFECTS      §5  (none) — no stored cosmetic buffer; shimmer / twinkle /
 *                    volcanic spark / city lights are render-derived from
 *                    time_secs. One-line section, not a real layer.
 *   DELAYS       §6  (none) — the auto-regen countdown (regen_t) + pause live in
 *                    scene_tick (§4); no standalone delay state.
 *   RENDER       §7  ncurses back buffer + colour-pair table only (theme_apply,
 *                    color_init, the 15 render_* views, scene_draw, screen_draw +
 *                    the HUD draw_* helpers). Reads §4 world state via the §3
 *                    deciders; never writes it.
 *   APP          §8  App.{running,need_resize,sim_fps}; drives Scene via the
 *                    combine + user events.
 *
 * PER-TICK COMBINE (the one place state advances — main(), §8):
 *
 *     while (sim_accum >= tick_ns)        // PERFORMANCE: fixed timestep
 *         scene_tick()                    //   SIMULATION + the regen timer
 *     screen_draw() ; screen_present()    // RENDER (reads only; views re-derived)
 *     getch() → app_handle_key()          // USER EVENTS — see below
 *
 * Nothing other than scene_tick() advances simulation state — and the only state
 * it advances per tick is the clock + regen countdown (a generation runs only
 * when that countdown expires, or on a user event). User events (app_handle_key
 * / app_do_resize) rebuild or mutate the world on a keypress or SIGWINCH, but
 * run once per frame OUTSIDE the accumulator loop, not as part of the tick.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ===================================================================== */
/* §1  CONFIG  -- constants, enums, data tables, core world types        */
/* ===================================================================== */

enum {
    /* Map cap. Anything larger is clipped. */
    MAP_W_MAX           = 240,
    MAP_H_MAX           =  80,
    CELLS_MAX           = MAP_W_MAX * MAP_H_MAX,

    /* Plate count range. Each regenerate picks N randomly in [MIN, MAX]. */
    N_PLATES_MIN        =   6,
    N_PLATES_MAX        =  14,
    N_PLATES_LIMIT      =  16,    /* hard upper bound for plates[] array */

    /* Boundary-effect neighbourhood radius — how far inland a boundary
     * pushes its elevation. R_X / R_Y are in CELLS; aspect-corrected
     * physical distance uses sqrt(dx² + 4·dy²). */
    BOUNDARY_RX         =   6,
    BOUNDARY_RY         =   3,

    SIM_FPS_MIN         =  10,
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,
    SIM_FPS_STEP        =  10,

    /* "speed" controls how often the world regenerates (smaller = faster).
     * SPEED_DEF is once every ~15 s. */
    SPEED_MIN           =   1,
    SPEED_DEF           =   8,
    SPEED_MAX           =  64,

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* Color pair indices.  PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* +0..+7 = 8 elevation/biome tints  */
    PAIR_HOT            =  11,    /* convergent / volcanic accent      */
    PAIR_COLD           =  12,    /* divergent / deep-water accent     */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Approximate seconds between full regenerations at SPEED_DEF.
 * speed > SPEED_DEF shortens this; speed < lengthens. */
#define REGEN_SECONDS    15.0f

/* Aspect — terminal cells are ~2× taller than wide.  Multiplies y in
 * Voronoi distance, boundary normals, and boundary scan radii. */
#define ASPECT_Y_F      2.0f

/* Boundary classification threshold. Approach magnitudes below this
 * fraction of the perpendicular magnitude become TRANSFORM rather
 * than CONVERGENT/DIVERGENT — see classify_boundary in §5. */
#define APPROACH_FRAC    0.50f

/* fBm scale — controls feature size of the noise added to elevation.
 * Lower = larger, smoother features. */
#define FBM_SCALE_X      0.060f
#define FBM_SCALE_Y      0.120f
#define FBM_OCTAVES      4
#define FBM_AMPLITUDE    0.40f       /* peak-to-peak elev contribution */

/* Plate base elevations and boundary modifiers. */
#define BASE_ELEV_OCEAN     (-0.50f)
#define BASE_ELEV_CONTINENT (+0.30f)
#define MOD_CONVERGENT      (+0.50f)
#define MOD_DIVERGENT       (-0.40f)

/* Plate velocity magnitude range (arbitrary units; only relative velocity
 * across a border matters). speed ∈ [MIN, MIN+SPAN]. */
#define PLATE_SPEED_MIN     0.30f
#define PLATE_SPEED_SPAN    0.70f

/* Elevation quantisation: the real [-1,+1] field stored as int8 [-100,+100]. */
#define ELEV_INT_SCALE     100.0f

/*
 * View parameters for the extended cartographic catalogue (cell elev is the
 * int8 [-100,+100]; SEA_LEVEL = 0 is the coastline).
 */
#define SEA_LEVEL            0          /* elev below this is underwater       */
#define RELIEF_LIGHT        12          /* hillshade slope→brightness divisor  */
#define CONTOUR_INTERVAL    18          /* elev units between contour lines     */
#define RUGGED_DIVISOR       8          /* slope magnitude → ruggedness divisor*/
#define HEAT_RADIUS          3          /* geothermal glow reach (cells)        */
#define MOISTURE_SCALE       9          /* rainfall value-noise cell size       */
#define DAYNIGHT_SECONDS    24.0f       /* seconds for one day/night rotation   */
#define TWILIGHT_WIDTH       0.18f      /* half-width of the dawn/dusk band     */

/* ── Biome ─────────────────────────────────────────────────────────────── *
 * Eight elevation bands, deepest ocean → highest peak, assigned purely from a
 * cell's elevation (elev_to_biome, §3). They are buckets, not an ecological
 * model — moisture/temperature don't enter (the TEMP/MOISTURE views add those
 * separately). Order IS the elevation order, so a Biome value doubles as a
 * brightness index into a theme's 8-step ramp (ramp[biome]). N_BIOMES = 8 also
 * sizes the ramp and the HUD census. REF: hypsometric tinting (Imhof). */
typedef enum {
    BIOME_DEEP_OCEAN = 0,
    BIOME_OCEAN      = 1,
    BIOME_COAST      = 2,
    BIOME_PLAINS     = 3,
    BIOME_HILLS      = 4,
    BIOME_MOUNTAINS  = 5,
    BIOME_HIGHLANDS  = 6,
    BIOME_PEAKS      = 7,
    N_BIOMES         = 8,
} Biome;

/*
 * BIOME_GLYPHS[biome][0..1] — two glyphs per biome; per-cell hash
 * picks one for textural variation. Kept ASCII-only so the map
 * renders identically on every terminal locale.
 */
static const char BIOME_GLYPHS[N_BIOMES][2] = {
    { '~', ',' },     /* DEEP_OCEAN */
    { '~', '_' },     /* OCEAN      */
    { '.', ',' },     /* COAST      */
    { '.', '_' },     /* PLAINS     */
    { '_', '^' },     /* HILLS      */
    { '^', 'A' },     /* MOUNTAINS  */
    { 'A', '#' },     /* HIGHLANDS  */
    { '#', '@' },     /* PEAKS      */
};

/* Elevation density ramp for ELEVATION pattern.
 * 0 = lowest (deepest), 7 = highest (peak). */
static const char ELEV_RAMP[N_BIOMES] = {
    '`', '.', ',', ':', '-', '^', '#', '@'
};

/* ── BoundaryKind ──────────────────────────────────────────────────────── *
 * What two plates are doing where they meet — classified from their RELATIVE
 * velocity projected onto the shared-border normal (classify_boundary, §3):
 *   NONE        interior cell, not on any plate border.
 *   CONVERGENT  plates push together → uplift (mountains, volcanic arcs, the
 *               +MOD_CONVERGENT elevation modifier).
 *   DIVERGENT   plates pull apart → rifts, trenches, new sea floor (the
 *               -MOD_DIVERGENT modifier).
 *   TRANSFORM   plates slide past (motion mostly tangent) → faults, no vertical
 *               effect. REF: plate boundaries — see the file-header References. */
typedef enum {
    BOUNDARY_NONE       = 0,
    BOUNDARY_CONVERGENT = 1,
    BOUNDARY_DIVERGENT  = 2,
    BOUNDARY_TRANSFORM  = 3,
} BoundaryKind;

/* ── PlateType ─────────────────────────────────────────────────────────── *
 * A plate's crust, ~50/50 by a hash bit (gen_plates, §4). It sets the plate's
 * resting elevation (base_elev): OCEANIC sits low (basins, BASE_ELEV_OCEAN),
 * CONTINENTAL high (landmasses, BASE_ELEV_CONTINENT) — so most land rides
 * continental plates and ocean rides oceanic ones, before boundary uplift. */
typedef enum {
    PLATE_OCEANIC     = 0,
    PLATE_CONTINENTAL = 1,
} PlateType;

/*
 * Pattern — fifteen ways to render the SAME simulated world. Each is a distinct
 * cartographic view of the per-cell data (elevation, plate id, boundary, biome)
 * or a quantity derived from it (slope, latitude-climate, geothermal heat). The
 * first four keep their original indices; the rest add standard map types
 * (hillshade relief, contour lines, bathymetry, …) plus an animated day/night.
 */
typedef enum {
    PATTERN_WORLD     = 0,    /* biome map (the default)                  */
    PATTERN_PLATES,           /* Voronoi plate tinting + seeds            */
    PATTERN_STRESS,           /* plate-boundary type heatmap              */
    PATTERN_ELEVATION,        /* grayscale height ramp                    */
    PATTERN_RELIEF,           /* shaded relief — terrain lit from the NW  */
    PATTERN_CONTOUR,          /* topographic contour lines                */
    PATTERN_BATHYMETRY,       /* ocean-depth ramp (sea floor focus)       */
    PATTERN_TEMPERATURE,      /* climate: latitude + altitude             */
    PATTERN_MOISTURE,         /* rainfall (value-noise field)             */
    PATTERN_RUGGEDNESS,       /* slope magnitude — mountains/cliffs glow  */
    PATTERN_ASPECT,           /* downhill direction (a flow field)        */
    PATTERN_HEATFLOW,         /* geothermal glow around plate boundaries  */
    PATTERN_NATIONS,          /* plates as nations: borders + capitals    */
    PATTERN_CONTINENTS,       /* land vs sea, continental vs oceanic      */
    PATTERN_DAYNIGHT,         /* animated terminator sweeping the globe   */
    N_PATTERNS,
} Pattern;

/*
 * Themes — same 10-name menu as the rest of the procedural showcases.
 * Each theme provides an 8-step ramp from "deepest / dimmest" to
 * "highest / brightest" plus two accent colours for hot (convergent /
 * volcanic) and cold (divergent / deep water) emphases. The ramp is
 * read by:
 *   - WORLD     — biome[i] uses ramp[i]
 *   - PLATES    — plate id i uses ramp[1 + (i mod 6)]
 *   - STRESS    — convergent uses HOT, divergent uses COLD,
 *                 transform uses ramp[6]
 *   - ELEVATION — index i in the 8-step ramp uses ramp[i]
 *
 * All entries chosen from the brighter half of the 256-colour cube so even
 * A_DIM cells stay legible against a default-black terminal. theme_apply()
 * loads these into the PAIR_RAMP_BASE+i / PAIR_HOT / PAIR_COLD pairs.
 * REF: hypsometric / relief colour schemes — see documentation/COLOR.md.
 */
typedef struct {
    const char *name;          /* HUD label (t/T cycles)                    */
    short       ramp[N_BIOMES];/* 8 tints, deep/dim → high/bright (by elev) */
    short       hot;           /* convergent / volcanic accent              */
    short       cold;          /* divergent / deep-water accent             */
} Theme;

#define N_THEMES 10

/*
 * All ramp entries sit in the bright half of the 256-colour space so
 * even A_DIM cells stay legible against a default-black terminal.
 * See "Theme Palette Brightness" in /CLAUDE.md.
 */
static const Theme themes[N_THEMES] = {
    /* name       deep   ----- land -----                peak  hot cold */
    /*            0    1    2    3    4    5    6    7                  */
    { "DEFAULT",{ 24,  31,  39,  70,  76, 137, 215, 230 }, 196,  39 },
    { "MATRIX", { 28,  34,  40,  46,  76, 118, 154, 192 }, 226,  39 },
    { "NOVA",   { 60,  91, 134, 165, 207, 213, 219, 231 }, 196,  39 },
    { "MONO",   {240, 243, 245, 247, 249, 251, 253, 255 }, 226,  39 },
    { "OCEAN",  { 24,  25,  31,  38,  45,  51, 117, 195 }, 196,  21 },
    { "FIRE",   { 88, 124, 130, 166, 202, 208, 214, 226 }, 226,  21 },
    { "EARTH",  { 94, 130, 137, 173, 179, 215, 222, 230 }, 196,  39 },
    { "FOREST", { 28,  34,  40,  70,  76, 112, 156, 192 }, 196,  39 },
    { "DESERT", {130, 137, 143, 173, 179, 215, 222, 229 }, 196,  21 },
    { "ARCTIC", { 24,  31,  67, 110, 117, 153, 195, 231 }, 196,  39 },
};

/* ----------------------------------------------------------------------- *
 * Core world types — the data the whole simulation reads and writes.       *
 * ----------------------------------------------------------------------- */

/* ── Plate ─────────────────────────────────────────────────────────────── *
 * One rigid tectonic plate — the moving unit of the simulation. A plate is a
 * Voronoi SEED (a point + a region of cells nearest it) carrying a constant
 * velocity; geology happens where neighbouring plates' velocities disagree at
 * their shared border (classify_boundary, §3). REF: plate tectonics — see the
 * file-header References block.
 *   x, y       seed position in map cells (the Voronoi site).
 *   vx, vy     velocity, magnitude ∈ [0.3,1.0]; only the RELATIVE velocity of
 *              two plates across their border matters, projected onto the border
 *              normal → convergent / divergent / transform.
 *   type       OCEANIC (low, dense) or CONTINENTAL (high) — sets base_elev and
 *              the land/ocean character of the plate's interior.
 *   base_elev  the plate's resting elevation in [-1,+1] before boundary uplift
 *              and fBm noise are added (compute_elevation, §4). */
typedef struct {
    int       x, y;
    float     vx, vy;
    PlateType type;
    float     base_elev;
} Plate;

/* ── Cell ──────────────────────────────────────────────────────────────── *
 * One map cell — the per-pixel result of the generation pipeline, packed to 4
 * bytes (a 240×80 map is ~76 KB). Every render view (§7) reads this; the §4
 * compute_* stages are its only writers. Stored compactly (uint8/int8) because
 * the field is large and the values are small, bounded enums/ranges.
 *   plate_id  which plate owns this cell (Voronoi nearest-seed). Index into
 *             World.plates[].
 *   boundary  BoundaryKind of the plate border this cell sits on (NONE for
 *             interior cells) — drives uplift and the STRESS / HEATFLOW views.
 *   elev      final elevation, the real-valued [-1,+1] field quantised to the
 *             int8 [-100,+100]. SEA_LEVEL is 0. The substrate for ELEVATION,
 *             RELIEF, CONTOUR, BATHYMETRY, RUGGED, ASPECT, …
 *   biome     Biome bucket derived from elev (elev_to_biome, §3) — the WORLD
 *             map's glyph + colour. */
typedef struct {
    uint8_t plate_id;
    uint8_t boundary;        /* BoundaryKind */
    int8_t  elev;            /* [-100, +100] */
    uint8_t biome;           /* Biome */
} Cell;

/* ── World ─────────────────────────────────────────────────────────────── *
 * The entire generated world: the plate set plus the dense per-cell grid, both
 * grown from one integer seed. Rebuilt wholesale by world_build (§4) on reset;
 * read-only to every render view. Fixed-capacity arrays (no malloc): a rebuild
 * overwrites in place. cells[] is row-major — index via widx(w,x,y) = y*w + x.
 *   w, h       active map dimensions (≤ MAP_W_MAX × MAP_H_MAX).
 *   plates[]   the plate seeds; n_plates of N_PLATES_LIMIT slots are live.
 *   cells[]    w·h cells, row-major; CELLS_MAX slots back the largest map.
 *   seed       what this world was generated from (also salts the per-cell
 *              hash so a given world always twinkles/textures the same way). */
typedef struct {
    int    w, h;
    Plate  plates[N_PLATES_LIMIT];
    int    n_plates;
    Cell   cells[CELLS_MAX];
    int    seed;
} World;

/* ===================================================================== */
/* §2  PERFORMANCE  -- timing primitives (throttle policy in main, §8)   */
/* ===================================================================== */

/* Timing primitives only. The frame cap and the fixed-timestep accumulator
 * that decide how many scene_tick()s run per frame are POLICY, in main (§8). */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

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
/* §3  LOGIC  -- pure decisions: hash, noise, classify, cartographic math*/
/* ===================================================================== */

/* Pure functions — given their arguments (and read-only world/noise data) each
 * returns a value with NO mutation and NO I/O: the spatial hash + Perlin/fBm
 * samplers, widx, the plate-pair boundary classifier, elev→biome, the pure
 * generation helpers (jitter_offset, nearest_boundary, boundary_uplift), the
 * cartographic math the views build on (clampi, elev_at, value_noise), the
 * biome census, and pattern_name. perm[] is a stable table reseeded ONLY from
 * §4 (on world build), never by RENDER/EFFECTS, so no frame ordering can
 * change a LOGIC result. These are decisions, not state. */

/* 9-char padded names so the HUD field width stays fixed. */
static const char *pattern_name(Pattern p)
{
    static const char *names[N_PATTERNS] = {
        "WORLD    ", "PLATES   ", "STRESS   ", "ELEVATION", "RELIEF   ",
        "CONTOUR  ", "BATHYMTRY", "TEMP     ", "MOISTURE ", "RUGGED   ",
        "ASPECT   ", "HEATFLOW ", "NATIONS  ", "CONTINENT", "DAY/NIGHT",
    };
    return ((int)p >= 0 && (int)p < N_PATTERNS) ? names[p] : "?        ";
}

/* hash3 — same as other showcases. */
static inline uint32_t hash3(int wx, int wy, int wz)
{
    uint32_t h = (uint32_t)wx * 73856093u
               ^ (uint32_t)wy * 19349663u
               ^ (uint32_t)wz * 83492791u;
    h ^= h >> 16;
    h *= 0x85ebca6bu;
    h ^= h >> 13;
    h *= 0xc2b2ae35u;
    h ^= h >> 16;
    return h;
}

/* Perlin scaffold — copied inline per the self-contained-file rule.
 * See ../fields/perin_noise_flow_showcase.c for derivation. */
static uint8_t perm[512];

static inline float fade_q(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}
static inline float lerp_f(float a, float b, float t) { return a + t * (b - a); }
static inline float grad2(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

static float perlin2d(float x, float y)
{
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x); y -= floorf(y);
    float u = fade_q(x), v = fade_q(y);
    int A = perm[X    ] + Y;
    int B = perm[X + 1] + Y;
    float n00 = grad2(perm[A    ], x,        y       );
    float n10 = grad2(perm[B    ], x - 1.0f, y       );
    float n01 = grad2(perm[A + 1], x,        y - 1.0f);
    float n11 = grad2(perm[B + 1], x - 1.0f, y - 1.0f);
    return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

static float fbm2(float x, float y)
{
    float total = 0.0f, amp = 1.0f, freq = 1.0f, max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;     /* → [0, 1] */
}

static inline int widx(const World *w, int x, int y) { return y * w->w + x; }

static uint8_t classify_boundary(const Plate *a, const Plate *b)
{
    /* Aspect-corrected normal from a to b. */
    float nx = (float)(b->x - a->x);
    float ny = (float)(b->y - a->y) * ASPECT_Y_F;
    float n_mag = sqrtf(nx * nx + ny * ny);
    if (n_mag < 0.001f) return BOUNDARY_TRANSFORM;
    nx /= n_mag; ny /= n_mag;

    float rel_vx = a->vx - b->vx;
    float rel_vy = a->vy - b->vy;
    float approach = rel_vx * nx + rel_vy * ny;
    float perp_x   = rel_vx - approach * nx;
    float perp_y   = rel_vy - approach * ny;
    float perp_mag = sqrtf(perp_x * perp_x + perp_y * perp_y);

    /* Classify TRANSFORM only when motion is mostly tangent — otherwise
     * any non-zero approach would override the perpendicular slide. */
    if (fabsf(approach) < perp_mag * APPROACH_FRAC) return BOUNDARY_TRANSFORM;
    return (approach > 0) ? BOUNDARY_CONVERGENT : BOUNDARY_DIVERGENT;
}

static int elev_to_biome(float e)
{
    if (e < -0.55f) return BIOME_DEEP_OCEAN;
    if (e < -0.20f) return BIOME_OCEAN;
    if (e <  0.00f) return BIOME_COAST;
    if (e <  0.15f) return BIOME_PLAINS;
    if (e <  0.35f) return BIOME_HILLS;
    if (e <  0.55f) return BIOME_MOUNTAINS;
    if (e <  0.75f) return BIOME_HIGHLANDS;
    return BIOME_PEAKS;
}

static inline int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Elevation at (x,y), clamped to the map edge — for neighbour gradients. */
static inline int elev_at(const World *w, int x, int y)
{
    if (x < 0) x = 0; else if (x >= w->w) x = w->w - 1;
    if (y < 0) y = 0; else if (y >= w->h) y = w->h - 1;
    return w->cells[widx(w, x, y)].elev;
}

/* value_noise — smooth [0,1] noise: bilinear blend of hashed lattice corners
 * spaced `scale` cells apart. Backs the rainfall map. */
static float value_noise(int x, int y, int seed, int scale)
{
    int   gx = x / scale, gy = y / scale;
    float fx = (float)(x - gx * scale) / (float)scale;
    float fy = (float)(y - gy * scale) / (float)scale;
    float n00 = (float)(hash3(gx,     gy,     seed) & 0xFFFFu) / 65535.0f;
    float n10 = (float)(hash3(gx + 1, gy,     seed) & 0xFFFFu) / 65535.0f;
    float n01 = (float)(hash3(gx,     gy + 1, seed) & 0xFFFFu) / 65535.0f;
    float n11 = (float)(hash3(gx + 1, gy + 1, seed) & 0xFFFFu) / 65535.0f;
    float a = n00 + (n10 - n00) * fx;
    float b = n01 + (n11 - n01) * fx;
    return a + (b - a) * fy;
}

/* jitter_offset — hash-driven scatter: maps the low 10 bits of `hbits`
 * (recentred to ±512) to an offset of about ±cell_size/4, used to nudge plate
 * seeds off their grid centres so the Voronoi cells look organic. */
static inline int jitter_offset(uint32_t hbits, int cell_size)
{
    return ((int)(hbits & 0x3FFu) - 512) * cell_size / 2048;
}

/* nearest_boundary — aspect-corrected distance from (x,y) to the closest
 * plate-boundary cell within the BOUNDARY_RX × BOUNDARY_RY window; *out_kind
 * receives that boundary's kind (NONE if none in range). The falloff input for
 * boundary uplift. Reads cells, mutates nothing. */
static float nearest_boundary(const World *w, int x, int y, uint8_t *out_kind)
{
    float best = (float)BOUNDARY_RX + 1.0f;
    *out_kind = BOUNDARY_NONE;
    for (int dy = -BOUNDARY_RY; dy <= BOUNDARY_RY; dy++) {
        int ny = y + dy;
        if (ny < 0 || ny >= w->h) continue;
        for (int dx = -BOUNDARY_RX; dx <= BOUNDARY_RX; dx++) {
            int nx = x + dx;
            if (nx < 0 || nx >= w->w) continue;
            uint8_t b = w->cells[widx(w, nx, ny)].boundary;
            if (b == BOUNDARY_NONE) continue;
            float d = sqrtf((float)(dx * dx)
                          + (float)(dy * dy) * (ASPECT_Y_F * ASPECT_Y_F));
            if (d < best) { best = d; *out_kind = b; }
        }
    }
    return best;
}

/* boundary_uplift — vertical elevation modifier from the nearest boundary:
 * convergent pushes terrain up, divergent pulls it down, both decaying linearly
 * to zero at BOUNDARY_RX. Transform / none contribute nothing. */
static float boundary_uplift(uint8_t kind, float dist)
{
    float r_max = (float)BOUNDARY_RX;
    if (dist >= r_max) return 0.0f;
    float weight = 1.0f - dist / r_max;             /* 1 on the line → 0 at r_max */
    if (kind == BOUNDARY_CONVERGENT) return MOD_CONVERGENT * weight;
    if (kind == BOUNDARY_DIVERGENT)  return MOD_DIVERGENT  * weight;
    return 0.0f;
}

/*
 * Biome census — count cells per biome for the HUD strip. O(W·H)
 * but only run when drawing the HUD; cost is trivial.
 */
static void biome_counts(const World *w, int counts[N_BIOMES])
{
    for (int i = 0; i < N_BIOMES; i++) counts[i] = 0;
    int total = w->w * w->h;
    for (int i = 0; i < total; i++)
        counts[w->cells[i].biome & 7]++;
}

/* ===================================================================== */
/* §4  SIMULATION  -- advances state (world generation + the tick)       */
/* ===================================================================== */

/* The ONLY writers of simulation state. Two kinds of work:
 *  • WORLD GENERATION (on reset) — world_build runs the pipeline perm_shuffle
 *    → gen_plates → compute_voronoi → compute_boundaries → compute_elevation,
 *    filling every Cell (plate_id, boundary, elev, biome) and the Plate[].
 *  • the per-tick advance — scene_tick steps the wall clock and the regen
 *    countdown, and triggers a rebuild when it expires.
 * Mutates: World.{plates,cells,n_plates,seed} + the global perm[] table, and
 * Scene.{world,paused,speed,current_theme,current_pattern,time_secs,regen_t}.
 * scene_tick() is the single per-tick entry (called only from main, §8); user
 * events (key/resize) also mutate Scene but are NOT part of the tick -- §8. */

static void perm_shuffle(int seed)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    /* Fisher-Yates with a hash-derived rng so regen is deterministic
     * given the same seed. */
    uint32_t st = (uint32_t)seed * 2654435761u;
    for (int i = 255; i > 0; i--) {
        st = st * 1664525u + 1013904223u;
        int j = (int)(st >> 16) % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
    for (int i = 0; i < 256; i++) {
        perm[i      ] = base[i];
        perm[i + 256] = base[i];
    }
}

/* ----------------------------------------------------------------------- *
 * Stage 1 — Plate placement (jittered grid).                              *
 * ----------------------------------------------------------------------- */

static void gen_plates(World *w, int seed)
{
    /* How many plates this world gets. */
    int n = N_PLATES_MIN
          + (int)(hash3(seed, 0, 1) % (uint32_t)(N_PLATES_MAX - N_PLATES_MIN + 1));
    if (n > N_PLATES_LIMIT) n = N_PLATES_LIMIT;

    /* Lay an aspect-corrected jittered grid of seed cells: cols ≈ √(n·aspect)
     * so the cells are roughly square once the 2:1 terminal aspect is undone. */
    float aspect = (float)w->w / ((float)w->h * ASPECT_Y_F);
    int   cols   = (int)ceilf(sqrtf((float)n * aspect));
    if (cols < 1) cols = 1;
    int rows = (n + cols - 1) / cols;
    int cell_w = w->w / cols; if (cell_w < 1) cell_w = 1;
    int cell_h = w->h / rows; if (cell_h < 1) cell_h = 1;

    /* One plate per grid cell: a jittered seed position, a hashed crust type,
     * and a random velocity. */
    int idx = 0;
    for (int r = 0; r < rows && idx < n; r++) {
        for (int c = 0; c < cols && idx < n; c++) {
            uint32_t h = hash3(c, r, seed);
            Plate *p = &w->plates[idx];

            p->x = clampi(c * cell_w + cell_w / 2 + jitter_offset(h,       cell_w), 1, w->w - 2);
            p->y = clampi(r * cell_h + cell_h / 2 + jitter_offset(h >> 10, cell_h), 1, w->h - 2);

            p->type = ((h >> 22) & 1u) ? PLATE_CONTINENTAL : PLATE_OCEANIC;
            p->base_elev = (p->type == PLATE_OCEANIC) ? BASE_ELEV_OCEAN : BASE_ELEV_CONTINENT;

            /* Random heading + magnitude, from a position-decorrelated hash. */
            uint32_t h2 = hash3(c, r, seed ^ 0x5A5A5A);
            float ang = ((float)(h2 & 0xFFFFu) / 65536.0f) * 2.0f * (float)M_PI;
            float spd = PLATE_SPEED_MIN
                      + ((float)((h2 >> 16) & 0xFFFFu) / 65536.0f) * PLATE_SPEED_SPAN;
            p->vx = cosf(ang) * spd;
            p->vy = sinf(ang) * spd;
            idx++;
        }
    }
    w->n_plates = idx;
}

/* ----------------------------------------------------------------------- *
 * Stage 2 — Voronoi assignment.                                           *
 * ----------------------------------------------------------------------- */

static void compute_voronoi(World *w)
{
    for (int y = 0; y < w->h; y++) {
        for (int x = 0; x < w->w; x++) {
            int  best = 0;
            long bd2  = (long)1 << 60;
            for (int i = 0; i < w->n_plates; i++) {
                long dx = x - w->plates[i].x;
                long dy = (y - w->plates[i].y) * 2;
                long d2 = dx * dx + dy * dy;
                if (d2 < bd2) { bd2 = d2; best = i; }
            }
            w->cells[widx(w, x, y)].plate_id = (uint8_t)best;
        }
    }
}

/* ----------------------------------------------------------------------- *
 * Stage 3 — boundary detection + classification.                          *
 * ----------------------------------------------------------------------- */

static void compute_boundaries(World *w)
{
    static const int dxs[4] = { 0,  0, -1,  1 };
    static const int dys[4] = {-1,  1,  0,  0 };

    for (int y = 0; y < w->h; y++) {
        for (int x = 0; x < w->w; x++) {
            int idx = widx(w, x, y);
            uint8_t my = w->cells[idx].plate_id;
            uint8_t kind = BOUNDARY_NONE;
            for (int d = 0; d < 4; d++) {
                int nx = x + dxs[d];
                int ny = y + dys[d];
                if (nx < 0 || nx >= w->w || ny < 0 || ny >= w->h) continue;
                uint8_t nb = w->cells[widx(w, nx, ny)].plate_id;
                if (nb == my) continue;
                kind = classify_boundary(&w->plates[my], &w->plates[nb]);
                break;
            }
            w->cells[idx].boundary = kind;
        }
    }
}

/* ----------------------------------------------------------------------- *
 * Stage 4 — elevation field.                                              *
 * ----------------------------------------------------------------------- */

static void compute_elevation(World *w)
{
    for (int y = 0; y < w->h; y++) {
        for (int x = 0; x < w->w; x++) {
            int idx = widx(w, x, y);

            /* Uplift from the nearest plate boundary, decaying with distance. */
            uint8_t kind;
            float dist     = nearest_boundary(w, x, y, &kind);
            float modifier = boundary_uplift(kind, dist);

            /* Resting plate elevation + boundary uplift + fBm detail, clamped. */
            const Plate *p = &w->plates[w->cells[idx].plate_id];
            float noise = fbm2((float)x * FBM_SCALE_X,
                               (float)y * FBM_SCALE_Y) - 0.5f;     /* [-0.5, 0.5] */
            float elev  = p->base_elev + modifier + noise * FBM_AMPLITUDE;
            if (elev < -1.0f) elev = -1.0f;
            if (elev >  1.0f) elev =  1.0f;

            w->cells[idx].elev  = (int8_t)(elev * ELEV_INT_SCALE);   /* [-1,1] → int8 */
            w->cells[idx].biome = (uint8_t)elev_to_biome(elev);
        }
    }
}

/* ----------------------------------------------------------------------- *
 * Top-level: build a world from a seed.                                   *
 * ----------------------------------------------------------------------- */

static void world_build(World *w, int seed)
{
    w->seed = seed;
    perm_shuffle(seed);
    gen_plates(w, seed);
    compute_voronoi(w);
    compute_boundaries(w);
    compute_elevation(w);
}

/* ── Scene ─────────────────────────────────────────────────────────────── *
 * The whole simulated-and-shown state. There is just ONE generated world here
 * (no entity pool): the views re-derive every pixel from it each frame.
 * scene_tick (§4) is the only per-tick writer; user events set the selections.
 *   world            the generated world (§4 rebuilds it; views read it).
 *   paused           freeze the regen countdown (rendering continues).
 *   speed            1..SPEED_MAX; scales how fast regen_t advances.
 *   current_theme    index into themes[] (t/T) — a RENDER selection.
 *   current_pattern  active map view (n/p) — a RENDER selection.
 *   time_secs        wall clock; drives the views' render-time animation
 *                    (twinkle, waves, day/night) and salts regen's next seed.
 *   regen_t          seconds the current world has been held; at REGEN_SECONDS
 *                    scene_rebuild() bulldozes it for a fresh one (the DELAYS
 *                    timer, advanced in scene_tick). */
typedef struct {
    World   world;
    bool    paused;
    int     speed;
    int     current_theme;
    Pattern current_pattern;
    float   time_secs;
    float   regen_t;          /* counts up; regen when > REGEN_SECONDS  */
} Scene;

static void scene_rebuild(Scene *s)
{
    int seed = (int)hash3((int)(s->time_secs * 1000.0f),
                          s->world.w, s->world.h);
    world_build(&s->world, seed);
    s->regen_t  = 0.0f;
}

static void scene_init(Scene *s, int map_w, int map_h)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_WORLD;
    s->world.w = map_w;
    s->world.h = map_h;
    scene_rebuild(s);
}

static void scene_resize_to(Scene *s, int map_w, int map_h)
{
    s->world.w = map_w;
    s->world.h = map_h;
    scene_rebuild(s);
}

/*
 * scene_tick — one continuous phase: hold the world, count up to
 * REGEN_SECONDS, then regenerate. Speed knob multiplies the rate so
 * users can speed through generations or slow down to inspect.
 */
static void scene_tick(Scene *s, float dt)
{
    s->time_secs += dt;
    if (s->paused) return;

    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->regen_t += dt * speed_mul;
    if (s->regen_t >= REGEN_SECONDS) scene_rebuild(s);
}

/* ===================================================================== */
/* §5  EFFECTS  -- cosmetic-only state                                   */
/* ===================================================================== */

/* No EFFECTS layer. Nothing cosmetic is stored: the ocean wave shimmer, the
 * peak twinkle, the convergent-mountain volcanic spark, the stress pulse and
 * the day/night city lights are all DERIVED at render time from time_secs in
 * the §7 view renderers — there is no glow/flash buffer to advance. */

/* ===================================================================== */
/* §6  DELAYS  -- pauses, holds, timers                                  */
/* ===================================================================== */

/* No separate layer. The one timer — the auto-regenerate countdown (regen_t
 * vs REGEN_SECONDS) — and the pause toggle both live in scene_tick (§4): the
 * world holds, the countdown advances at the speed knob's rate, and on expiry
 * scene_rebuild() is called. No standalone delay state. */

/* ===================================================================== */
/* §7  RENDER  -- state -> screen (reads only, never mutates sim)        */
/* ===================================================================== */

/* state -> screen. scene_draw dispatches the active Pattern to one of the 15
 * per-cell view renderers (biome map, plate Voronoi, stress, elevation, plus
 * the derived cartographic views); screen_draw lays the HUD + biome strip over
 * it. Reads World / Scene + the §3 deciders; writes ONLY the ncurses back
 * buffer and the colour-pair table (theme_apply / color_init). Never mutates
 * simulation state, so a frame can be dropped with no effect on the world. */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < N_BIOMES; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
        init_pair(PAIR_HOT,  t->hot,  -1);
        init_pair(PAIR_COLD, t->cold, -1);
    } else {
        static const short fb[N_BIOMES] = {
            COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN,   COLOR_GREEN,
            COLOR_GREEN, COLOR_YELLOW,COLOR_YELLOW, COLOR_WHITE,
        };
        for (int i = 0; i < N_BIOMES; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), fb[i], -1);
        init_pair(PAIR_HOT,  COLOR_RED,  -1);
        init_pair(PAIR_COLD, COLOR_CYAN, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ── Screen ────────────────────────────────────────────────────────────── *
 * The terminal viewport plus the map area carved out of it — pure presentation
 * geometry, holding NO simulation state. Recomputed by screen_layout() at
 * startup and on every SIGWINCH, and it also sizes the World (map_w/h feed
 * scene_resize_to). A world cell (x,y) is drawn at terminal (gy0+y, gx0+x).
 *   cols, rows   full terminal size (getmaxyx).
 *   map_w, map_h world dimensions that fit (≤ MAP_W_MAX × MAP_H_MAX, leaving
 *                rows 0-1 for the HUD and the last row for the hint).
 *   gx0, gy0     top-left origin where the map is drawn on screen. */
typedef struct {
    int cols, rows;
    int map_w, map_h;
    int gx0, gy0;
} Screen;

static void screen_layout(Screen *s)
{
    int top = 2, bottom = s->rows - 1;
    int avail_h = bottom - top;
    int avail_w = s->cols;

    int mw = avail_w;
    int mh = avail_h;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    if (mw < 16) mw = 16;
    if (mh < 8)  mh = 8;

    s->map_w = mw;
    s->map_h = mh;
    s->gx0 = (avail_w - mw) / 2;
    s->gy0 = top + (avail_h - mh) / 2;
    if (s->gx0 < 0) s->gx0 = 0;
    if (s->gy0 < top) s->gy0 = top;
}

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
    screen_layout(s);
}
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
    screen_layout(s);
}

/* ----------------------------------------------------------------------- *
 * Pattern renderers.                                                       *
 * ----------------------------------------------------------------------- */

/*
 * render_world — biome map.  Per cell pick a glyph from the biome's
 * 2-glyph table (chosen by hash for textural variation), in the
 * biome-ramp colour. Three subtle animations on top:
 *   - water cells (DEEP_OCEAN / OCEAN) rotate among '~' '_' ',' so the
 *     ocean "shimmers" with passing waves;
 *   - peak cells (PEAKS) twinkle to A_BOLD on a per-second hash;
 *   - convergent mountain cells occasionally flash '*' (volcano).
 */
static void render_world(const Screen *sc, const Scene *s)
{
    const World *w = &s->world;
    int twinkle_t  = (int)s->time_secs;
    int wave_phase = (int)(s->time_secs * 3.0f);

    for (int y = 0; y < w->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < 2 || sy >= sc->rows - 1) continue;
        for (int x = 0; x < w->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            const Cell *c = &w->cells[widx(w, x, y)];
            int biome = c->biome & 7;
            uint32_t hash = hash3(x, y, w->seed);

            char glyph;
            if (biome <= BIOME_OCEAN) {
                /* Water shimmer — rotate among 4 wave glyphs. */
                static const char waves[4] = { '~', '_', '~', ',' };
                int phase = (x + y * 2 + wave_phase) & 3;
                glyph = waves[phase];
            } else {
                glyph = BIOME_GLYPHS[biome][hash & 1];
            }

            int  attr = A_NORMAL;
            int  pair = PAIR_RAMP_BASE + biome;

            /* Peaks twinkle. */
            if (biome == BIOME_PEAKS) {
                attr = A_BOLD;
                if ((hash3(x, y, twinkle_t) % 50u) == 0u) attr |= A_BOLD;
            } else if (biome >= BIOME_HIGHLANDS) {
                attr = A_BOLD;
            } else if (biome == BIOME_DEEP_OCEAN) {
                attr = A_DIM;
            }

            /* Volcanic spark at convergent mountain cells. */
            if (c->boundary == BOUNDARY_CONVERGENT && biome >= BIOME_MOUNTAINS) {
                if ((hash3(x, y, (int)(s->time_secs * 0.6f)) % 80u) == 0u) {
                    glyph = '*';
                    pair  = PAIR_HOT;
                    attr  = A_BOLD;
                }
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/*
 * render_plates — Voronoi tinting. Each plate id picks a colour from
 * the ramp (cycled through ramp[1..6] so it doesn't collide with the
 * water/peak ends). Glyph by plate type; plate seeds drawn as bright 'O'.
 */
static void render_plates(const Screen *sc, const Scene *s)
{
    const World *w = &s->world;

    for (int y = 0; y < w->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < 2 || sy >= sc->rows - 1) continue;
        for (int x = 0; x < w->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            const Cell  *c = &w->cells[widx(w, x, y)];
            const Plate *p = &w->plates[c->plate_id];

            char glyph;
            int  pair;
            int  attr = A_NORMAL;

            if (c->boundary != BOUNDARY_NONE) {
                glyph = '+';
                pair  = PAIR_HOT;
                attr  = A_BOLD;
            } else {
                glyph = (p->type == PLATE_OCEANIC) ? '~' : '#';
                /* Cycle plate ids through 6 of the 8 ramp slots so all
                 * plates get distinct, non-edge tints. */
                pair  = PAIR_RAMP_BASE + 1 + (c->plate_id % 6);
                if (p->type == PLATE_CONTINENTAL) attr = A_BOLD;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }

    /* Plate seed markers — bright 'O' on top. */
    for (int i = 0; i < w->n_plates; i++) {
        int sy = sc->gy0 + w->plates[i].y;
        int sx = sc->gx0 + w->plates[i].x;
        if (sy < 2 || sy >= sc->rows - 1)  continue;
        if (sx < 0 || sx >= sc->cols)      continue;
        attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
        mvaddch(sy, sx, 'O');
        attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    }
}

/*
 * render_stress — boundary type heatmap. Boundary cells light up in
 * type-specific accent colour with directional glyphs; non-boundary
 * cells render as dim biome backdrop so the geology pops on top of
 * a recognisable map.
 */
static void render_stress(const Screen *sc, const Scene *s)
{
    const World *w = &s->world;
    int pulse_t = (int)(s->time_secs * 2.0f);

    for (int y = 0; y < w->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < 2 || sy >= sc->rows - 1) continue;
        for (int x = 0; x < w->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            const Cell *c = &w->cells[widx(w, x, y)];
            char glyph;
            int  pair;
            int  attr;

            if (c->boundary == BOUNDARY_CONVERGENT) {
                glyph = ((x + y) & 1) ? 'A' : '^';
                pair  = PAIR_HOT;
                /* Slow pulse — alternate BOLD/NORMAL each second. */
                attr  = ((pulse_t + x + y) & 1) ? A_BOLD : A_NORMAL;
            } else if (c->boundary == BOUNDARY_DIVERGENT) {
                glyph = ((x + y) & 1) ? 'v' : '_';
                pair  = PAIR_COLD;
                attr  = ((pulse_t + x + y) & 1) ? A_BOLD : A_NORMAL;
            } else if (c->boundary == BOUNDARY_TRANSFORM) {
                glyph = ((x + y) & 1) ? '/' : '\\';
                pair  = PAIR_RAMP_BASE + 6;
                attr  = A_BOLD;
            } else {
                /* Backdrop biome, dimmed. */
                int biome = c->biome & 7;
                glyph = BIOME_GLYPHS[biome][0];
                pair  = PAIR_RAMP_BASE + biome;
                attr  = A_DIM;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/*
 * render_elevation — pure height ramp. Map int8 elev [-100, +100] to
 * the 8-step ramp and render the corresponding glyph in the matching
 * tint. Same colour palette as WORLD; what differs is the GLYPH choice
 * — here we use a density ramp so the visual reads as a heatmap.
 */
static void render_elevation(const Screen *sc, const Scene *s)
{
    const World *w = &s->world;
    int twinkle_t = (int)s->time_secs;

    for (int y = 0; y < w->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < 2 || sy >= sc->rows - 1) continue;
        for (int x = 0; x < w->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            const Cell *c = &w->cells[widx(w, x, y)];
            /* Map [-100, +100] linearly onto [0, 7]. */
            int e = c->elev + 100;          /* [0, 200] */
            int level = e * N_BIOMES / 201; /* [0, 7]   */
            if (level < 0)         level = 0;
            if (level >= N_BIOMES) level = N_BIOMES - 1;

            char glyph = ELEV_RAMP[level];
            int  pair  = PAIR_RAMP_BASE + level;
            int  attr  = A_NORMAL;
            if      (level >= 6) attr = A_BOLD;
            else if (level <= 1) attr = A_DIM;

            if (level == 7 && (hash3(x, y, twinkle_t) % 40u) == 0u)
                attr |= A_BOLD;

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* ----------------------------------------------------------------------- *
 * Extended cartographic views — each is a different projection of the same
 * per-cell data (elev / plate_id / boundary / biome) or a quantity derived
 * from it. Shared helpers first.
 * ----------------------------------------------------------------------- */

/* The per-cell render loop these views share: clip to the map window, then run
 * `body` (a brace block referencing x, y, sy, sx, w, c). */
#define FOR_EACH_CELL(w, sc)                                                 \
    for (int y = 0; y < (w)->h; y++) {                                       \
        int sy = (sc)->gy0 + y;                                              \
        if (sy < 2 || sy >= (sc)->rows - 1) continue;                        \
        for (int x = 0; x < (w)->w; x++) {                                   \
            int sx = (sc)->gx0 + x;                                          \
            if (sx < 0 || sx >= (sc)->cols) continue;                        \
            const Cell *c = &(w)->cells[widx((w), x, y)];

#define END_EACH_CELL  } }

/* Draw one cell's glyph in (pair, attr) — every view ends each cell with this. */
static inline void put_cell(int sy, int sx, int pair, int attr, char glyph)
{
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

/* render_relief — shaded relief. Light the terrain from the NW: a slope facing
 * the light is bright, one facing away is in shadow (gx+gy is the NW dot). */
static void render_relief(const Screen *sc, const Scene *s)
{
    const World *w = &s->world;
    static const char shade[8] = { '.', '.', ':', '-', '+', '*', '#', '@' };
    FOR_EACH_CELL(w, sc) {
        if (c->elev < SEA_LEVEL) { put_cell(sy, sx, PAIR_RAMP_BASE + 0, A_DIM, '~'); continue; }
        int gx  = elev_at(w, x + 1, y) - elev_at(w, x - 1, y);
        int gy  = elev_at(w, x, y + 1) - elev_at(w, x, y - 1);
        int lit = clampi(4 + (gx + gy) / RELIEF_LIGHT, 0, 7);
        int attr = lit >= 6 ? A_BOLD : lit <= 1 ? A_DIM : A_NORMAL;
        put_cell(sy, sx, PAIR_RAMP_BASE + lit, attr, shade[lit]);
    } END_EACH_CELL
}

/* render_contour — topographic contour lines: a cell sits on a contour when its
 * elevation band differs from a neighbour's. The sea-level contour (coastline)
 * is drawn boldest. */
static void render_contour(const Screen *sc, const Scene *s)
{
    const World *w = &s->world;
    FOR_EACH_CELL(w, sc) {
        int e = c->elev, ee = elev_at(w, x + 1, y), es = elev_at(w, x, y + 1);
        bool coast = (e >= SEA_LEVEL) != (ee >= SEA_LEVEL)
                  || (e >= SEA_LEVEL) != (es >= SEA_LEVEL);
        int band = e / CONTOUR_INTERVAL;
        bool line = band != ee / CONTOUR_INTERVAL || band != es / CONTOUR_INTERVAL;
        int level = clampi((e + 100) * N_BIOMES / 201, 0, 7);
        if (coast)      put_cell(sy, sx, PAIR_COLD, A_BOLD, '~');
        else if (line)  put_cell(sy, sx, PAIR_RAMP_BASE + level, A_BOLD,
                                 (band != ee / CONTOUR_INTERVAL) ? '|' : '-');
        else            put_cell(sy, sx, PAIR_RAMP_BASE + level,
                                 e < SEA_LEVEL ? A_DIM : A_NORMAL, e < SEA_LEVEL ? ' ' : '.');
    } END_EACH_CELL
}

/* render_bathymetry — ocean-depth ramp: underwater cells tinted by depth
 * (deep trenches darkest), land flattened to a neutral backdrop. */
static void render_bathymetry(const Screen *sc, const Scene *s)
{
    const World *w = &s->world;
    static const char dg[4] = { '#', '*', ':', '.' };   /* shallow → deep */
    FOR_EACH_CELL(w, sc) {
        if (c->elev >= SEA_LEVEL) { put_cell(sy, sx, PAIR_RAMP_BASE + 5, A_DIM, '.'); continue; }
        int level = clampi((-c->elev) * 4 / 101, 0, 3);          /* 0 shallow .. 3 deep */
        put_cell(sy, sx, PAIR_RAMP_BASE + (3 - level), level == 3 ? A_DIM : A_NORMAL, dg[level]);
    } END_EACH_CELL
}

/* render_temperature — climate from latitude (cold at the poles) and altitude
 * (cold on the peaks). Hot cells glow red, cold cells blue, the rest the ramp. */
static void render_temperature(const Screen *sc, const Scene *s)
{
    const World *w = &s->world;
    int eq = w->h / 2;
    FOR_EACH_CELL(w, sc) {
        int lat  = abs(y - eq) * 100 / (eq > 0 ? eq : 1);        /* 0 equator .. 100 pole */
        int alt  = c->elev > 0 ? c->elev : 0;
        int temp = clampi(100 - lat - alt * 6 / 10, 0, 100);
        int level = temp * 8 / 101;                              /* 0 cold .. 7 hot */
        if (level >= 6)      put_cell(sy, sx, PAIR_HOT,  A_BOLD, '#');
        else if (level <= 1) put_cell(sy, sx, PAIR_COLD, A_BOLD, '*');
        else                 put_cell(sy, sx, PAIR_RAMP_BASE + level, A_NORMAL, '+');
    } END_EACH_CELL
}

/* render_moisture — rainfall: a value-noise field, dried out at altitude (a
 * crude rain shadow). Wet cells are bright, arid cells faint. */
static void render_moisture(const Screen *sc, const Scene *s)
{
    const World *w = &s->world;
    FOR_EACH_CELL(w, sc) {
        if (c->elev < SEA_LEVEL) { put_cell(sy, sx, PAIR_RAMP_BASE + 1, A_DIM, '~'); continue; }
        float m = value_noise(x, y, w->seed ^ 0x4D, MOISTURE_SCALE)
                - (float)(c->elev > 0 ? c->elev : 0) / 300.0f;
        int level = clampi((int)(m * 8.0f), 0, 7);
        char g = level >= 5 ? '#' : level >= 3 ? '*' : level >= 1 ? ':' : '.';
        int attr = level >= 6 ? A_BOLD : level <= 1 ? A_DIM : A_NORMAL;
        put_cell(sy, sx, PAIR_RAMP_BASE + level, attr, g);
    } END_EACH_CELL
}

/* render_ruggedness — terrain roughness = local slope magnitude; mountain
 * ranges and cliffs glow, flat plains fade. */
static void render_ruggedness(const Screen *sc, const Scene *s)
{
    const World *w = &s->world;
    FOR_EACH_CELL(w, sc) {
        if (c->elev < SEA_LEVEL) { put_cell(sy, sx, PAIR_RAMP_BASE + 0, A_DIM, ' '); continue; }
        int gx = elev_at(w, x + 1, y) - elev_at(w, x - 1, y);
        int gy = elev_at(w, x, y + 1) - elev_at(w, x, y - 1);
        int level = clampi((abs(gx) + abs(gy)) / RUGGED_DIVISOR, 0, 7);
        char g = level >= 5 ? '^' : level >= 3 ? '#' : level >= 1 ? ':' : '.';
        if (level >= 4) put_cell(sy, sx, PAIR_HOT, A_BOLD, g);
        else            put_cell(sy, sx, PAIR_RAMP_BASE + clampi(3 + level, 0, 7),
                                 level == 0 ? A_DIM : A_NORMAL, g);
    } END_EACH_CELL
}

/* render_aspect — downhill direction as a flow field: each slope cell is tinted
 * and arrowed by which of 8 compass directions it drains toward. */
static void render_aspect(const Screen *sc, const Scene *s)
{
    const World *w = &s->world;
    static const char arrow[8] = { '<', '\\', 'v', '/', '>', '/', '^', '\\' };
    FOR_EACH_CELL(w, sc) {
        if (c->elev < SEA_LEVEL) { put_cell(sy, sx, PAIR_RAMP_BASE + 0, A_DIM, '~'); continue; }
        int gx = elev_at(w, x + 1, y) - elev_at(w, x - 1, y);
        int gy = elev_at(w, x, y + 1) - elev_at(w, x, y - 1);
        if (abs(gx) + abs(gy) < 2) { put_cell(sy, sx, PAIR_RAMP_BASE + 3, A_DIM, '.'); continue; }
        float ang = atan2f((float)(-gy), (float)(-gx));         /* downhill heading */
        int sector = ((int)floorf((ang + (float)M_PI) / (2.0f * (float)M_PI) * 8.0f + 0.5f)) & 7;
        put_cell(sy, sx, PAIR_RAMP_BASE + sector, A_BOLD, arrow[sector]);
    } END_EACH_CELL
}

/* render_heatflow — geothermal map: cells glow with how close they sit to a
 * convergent/divergent plate boundary (volcanic & rift heat), pulsing. */
static void render_heatflow(const Screen *sc, const Scene *s)
{
    const World *w = &s->world;
    int pulse = (int)(s->time_secs * 2.0f);
    FOR_EACH_CELL(w, sc) {
        (void)c;                              /* this view scans neighbours, not c */
        int best = HEAT_RADIUS + 1;
        for (int dy = -HEAT_RADIUS; dy <= HEAT_RADIUS; dy++) {
            int ny = y + dy; if (ny < 0 || ny >= w->h) continue;
            for (int dx = -HEAT_RADIUS; dx <= HEAT_RADIUS; dx++) {
                int nx = x + dx; if (nx < 0 || nx >= w->w) continue;
                uint8_t b = w->cells[widx(w, nx, ny)].boundary;
                if (b == BOUNDARY_NONE || b == BOUNDARY_TRANSFORM) continue;
                int d = abs(dx) + abs(dy);
                if (d < best) best = d;
            }
        }
        if (best > HEAT_RADIUS) { put_cell(sy, sx, PAIR_RAMP_BASE + 0, A_DIM, '.'); continue; }
        int heat = HEAT_RADIUS + 1 - best;                      /* 1 (far) .. HEAT_RADIUS+1 (on it) */
        char g = heat >= HEAT_RADIUS ? '@' : heat >= HEAT_RADIUS - 1 ? '#' : '*';
        int attr = heat < HEAT_RADIUS - 1 ? A_DIM
                 : ((pulse + x + y) & 1)  ? A_BOLD : A_NORMAL;
        put_cell(sy, sx, PAIR_HOT, attr, g);
    } END_EACH_CELL
}

/* render_nations — plates as nations: solid territory tints, white borders
 * where plates meet, and a bright capital marker at each plate seed. */
static void render_nations(const Screen *sc, const Scene *s)
{
    const World *w = &s->world;
    FOR_EACH_CELL(w, sc) {
        uint8_t pid = c->plate_id;
        bool border = (x + 1 < w->w && w->cells[widx(w, x + 1, y)].plate_id != pid)
                   || (x - 1 >= 0   && w->cells[widx(w, x - 1, y)].plate_id != pid)
                   || (y + 1 < w->h && w->cells[widx(w, x, y + 1)].plate_id != pid)
                   || (y - 1 >= 0   && w->cells[widx(w, x, y - 1)].plate_id != pid);
        if (border) put_cell(sy, sx, PAIR_HUD, A_BOLD, '+');
        else        put_cell(sy, sx, PAIR_RAMP_BASE + 1 + (pid % 6),
                             c->elev < SEA_LEVEL ? A_DIM : A_NORMAL,
                             c->elev < SEA_LEVEL ? '~' : ':');
    } END_EACH_CELL

    for (int i = 0; i < w->n_plates; i++) {
        int sy = sc->gy0 + w->plates[i].y, sx = sc->gx0 + w->plates[i].x;
        if (sy < 2 || sy >= sc->rows - 1 || sx < 0 || sx >= sc->cols) continue;
        put_cell(sy, sx, PAIR_HUD, A_BOLD, '@');
    }
}

/* render_continents — tectonic geography: ocean basins vs land, and within
 * each, continental crust vs oceanic crust (volcanic islands). */
static void render_continents(const Screen *sc, const Scene *s)
{
    const World *w = &s->world;
    FOR_EACH_CELL(w, sc) {
        const Plate *p = &w->plates[c->plate_id];
        if (c->elev < SEA_LEVEL)
            put_cell(sy, sx, PAIR_RAMP_BASE + (p->type == PLATE_OCEANIC ? 0 : 1), A_DIM, '~');
        else if (p->type == PLATE_CONTINENTAL)
            put_cell(sy, sx, PAIR_RAMP_BASE + 4, A_BOLD, '#');
        else
            put_cell(sy, sx, PAIR_HOT, A_BOLD, '^');
    } END_EACH_CELL
}

/* render_daynight — an animated terminator sweeps the map: the day side shows
 * the biome map in full, a warm twilight band trails the edge, and the night
 * side darkens with twinkling "city lights" on inhabited land. */
static void render_daynight(const Screen *sc, const Scene *s)
{
    const World *w = &s->world;
    float sun = fmodf(s->time_secs / DAYNIGHT_SECONDS, 1.0f) * (float)w->w;
    int twinkle_t = (int)s->time_secs;
    FOR_EACH_CELL(w, sc) {
        int biome = c->biome & 7;
        float d = ((float)x - sun) / (float)w->w;
        d -= floorf(d + 0.5f);                                  /* wrap to [-0.5, 0.5] */
        float light = cosf(d * 2.0f * (float)M_PI);             /* 1 = noon, -1 = midnight */

        char g = (biome <= BIOME_OCEAN) ? '~' : BIOME_GLYPHS[biome][hash3(x, y, w->seed) & 1];
        if (light > TWILIGHT_WIDTH) {
            put_cell(sy, sx, PAIR_RAMP_BASE + biome, biome >= BIOME_HIGHLANDS ? A_BOLD : A_NORMAL, g);
        } else if (light > -TWILIGHT_WIDTH) {
            put_cell(sy, sx, PAIR_HOT, A_NORMAL, g);            /* dawn/dusk band */
        } else if (biome >= BIOME_PLAINS && (hash3(x, y, twinkle_t) % 60u) == 0u) {
            put_cell(sy, sx, PAIR_HUD, A_BOLD, '.');           /* city light */
        } else {
            put_cell(sy, sx, PAIR_RAMP_BASE + biome, A_DIM, biome <= BIOME_OCEAN ? ' ' : g);
        }
    } END_EACH_CELL
}

static void scene_draw(const Screen *sc, const Scene *s)
{
    switch (s->current_pattern) {
    case PATTERN_WORLD:       render_world(sc, s);       break;
    case PATTERN_PLATES:      render_plates(sc, s);      break;
    case PATTERN_STRESS:      render_stress(sc, s);      break;
    case PATTERN_ELEVATION:   render_elevation(sc, s);   break;
    case PATTERN_RELIEF:      render_relief(sc, s);      break;
    case PATTERN_CONTOUR:     render_contour(sc, s);     break;
    case PATTERN_BATHYMETRY:  render_bathymetry(sc, s);  break;
    case PATTERN_TEMPERATURE: render_temperature(sc, s); break;
    case PATTERN_MOISTURE:    render_moisture(sc, s);    break;
    case PATTERN_RUGGEDNESS:  render_ruggedness(sc, s);  break;
    case PATTERN_ASPECT:      render_aspect(sc, s);      break;
    case PATTERN_HEATFLOW:    render_heatflow(sc, s);    break;
    case PATTERN_NATIONS:     render_nations(sc, s);     break;
    case PATTERN_CONTINENTS:  render_continents(sc, s);  break;
    case PATTERN_DAYNIGHT:    render_daynight(sc, s);    break;
    case N_PATTERNS:          break;            /* unreachable sentinel */
    }
}

/* Row 0 right — primary status: fps, sim Hz, view/pause, speed. Right-aligned. */
static void draw_status_line(const Screen *sc, const Scene *s, double fps, int sim_fps)
{
    const char *state_str = s->paused ? "PAUSED   " : pattern_name(s->current_pattern);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Row 1 — view (n/N counter), theme, the elevation-ramp swatch, and world
 * stats: plate count, land/sea split, and regen-countdown progress. */
static void draw_param_line(const Scene *s)
{
    const World *w = &s->world;
    int x = 1;
    char pbuf[40];
    snprintf(pbuf, sizeof pbuf, " pattern:%s %d/%d ",
             pattern_name(s->current_pattern),
             (int)s->current_pattern + 1, (int)N_PATTERNS);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);  mvprintw(1, x, "%s", pbuf);  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += (int)strlen(pbuf);

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    attron(COLOR_PAIR(PAIR_HUD));  mvprintw(1, x, " ramp:");  attroff(COLOR_PAIR(PAIR_HUD));
    x += 6;
    for (int i = 0; i < N_BIOMES; i++) {                 /* elevation-ramp swatch */
        attron(COLOR_PAIR(PAIR_RAMP_BASE + i) | A_BOLD);
        mvaddch(1, x, (chtype)(unsigned char)ELEV_RAMP[i]);
        attroff(COLOR_PAIR(PAIR_RAMP_BASE + i) | A_BOLD);
        x++;
    }

    /* Land-vs-sea split (by biome) + regen-countdown progress. */
    int counts[N_BIOMES];
    biome_counts(w, counts);
    int sea  = counts[BIOME_DEEP_OCEAN] + counts[BIOME_OCEAN];
    int land = counts[BIOME_PLAINS]    + counts[BIOME_HILLS]
             + counts[BIOME_MOUNTAINS] + counts[BIOME_HIGHLANDS]
             + counts[BIOME_PEAKS];
    int total = w->w * w->h;
    int sea_pct   = total ? (sea  * 100) / total : 0;
    int land_pct  = total ? (land * 100) / total : 0;
    int regen_pct = clampi((int)((s->regen_t / REGEN_SECONDS) * 100.0f), 0, 100);

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  plates:%-2d  sea:%2d%%  land:%2d%%  regen:%3d%% ",
             w->n_plates, sea_pct, land_pct, regen_pct);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row — the key legend. Lists every interactive key (HUD standard). */
static void draw_hint(const Screen *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:speed  ]/[:tickHz  spc:pause  r:regen  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);

    draw_status_line(sc, s, fps, sim_fps);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " TECTONIC WORLD ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    draw_param_line(s);
    draw_hint(sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  APP  -- events + per-tick combine + main loop                     */
/* ===================================================================== */

/* Owns the App aggregate, signal flags, user-event handlers and the main loop.
 * main() is the ONE place that combines the layers per tick, in fixed order:
 * scene_tick (SIM + the regen timer) -> screen_draw (RENDER) -> screen_present
 * -> input. app_handle_key() / app_do_resize() mutate state on USER EVENTS (a
 * keypress or SIGWINCH; r rebuilds the world) and are OUTSIDE the tick. */

/* ── App ───────────────────────────────────────────────────────────────── *
 * Top-level harness binding the simulation (scene) to the terminal (screen),
 * plus the loop's PERFORMANCE knob and the async signal flags. A single static
 * instance (g_app) exists ONLY so the signal handlers — which may fire between
 * any two instructions — can reach the flags; everything else passes App
 * explicitly. The fixed-timestep rate (sim_fps) decouples the regen cadence
 * from frame rate (Fiedler, "Fix Your Timestep!"; main()).
 *   scene/screen  the two worlds it binds (what is simulated / where drawn).
 *   sim_fps       fixed-timestep rate, SIM_FPS_* Hz.
 *   running       cleared by SIGINT/SIGTERM → loop exits.
 *   need_resize   set by SIGWINCH, served next loop. Both are
 *                 volatile sig_atomic_t: written from handlers, so the compiler
 *                 must re-read them each loop and the write is atomic. */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_resize_to(&app->scene, app->screen.map_w, app->screen.map_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_rebuild(s);                              break;

    case '=': case '+':
        if (s->speed < SPEED_MAX) s->speed *= 2;
        if (s->speed > SPEED_MAX) s->speed  = SPEED_MAX;
        break;
    case '-':
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed  = SPEED_MIN;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case 'n': case 'N':
        s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        break;

    default: break;
    }
    return true;
}

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

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.map_w, app->screen.map_h);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
