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
 *       follow the underlying Voronoi geometry. Cycle four views of
 *       the same generated world with n / p:
 *
 *         WORLD      biome map — the finished world
 *         PLATES     coloured Voronoi cells, one tint per plate,
 *                    with plate seed centres marked as 'O'
 *         STRESS     boundary-type heatmap — convergent edges in red,
 *                    divergent in blue, transform faults in yellow
 *         ELEVATION  pure height ramp — eight glyphs from '`' to '@'
 *
 *       After ~15 seconds a flash sweeps the screen, the world is
 *       discarded, and a new one builds from a fresh seed —
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
 * Section map:
 *   §1 config     — geometry, biome thresholds, themes, patterns
 *   §2 clock      — monotonic timer + sleep
 *   §3 color      — HUD reserved + 10 themes (8-step ramp + accents)
 *   §5 tectonics  — hash, perlin/fbm, plate gen, Voronoi, boundaries,
 *                   classification, elevation, biome assignment
 *   §6 scene      — Scene state, regenerate cycle, animation
 *   §7 screen     — four pattern renderers (WORLD/PLATES/STRESS/ELEV)
 *   §8 app        — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          regenerate world with a new seed
 *   n / N      next pattern  (WORLD → PLATES → STRESS → ELEVATION)
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
 *                  Once the world is built, four PATTERNS render the
 *                  same data four ways — biomes (the natural map),
 *                  Voronoi tinting (the plates themselves), boundary
 *                  type heatmap (the geology), or pure elevation ramp.
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
 *   • Wikipedia — Plate tectonics
 *     https://en.wikipedia.org/wiki/Plate_tectonics
 *   • Wikipedia — Convergent / Divergent / Transform boundaries
 *     https://en.wikipedia.org/wiki/Plate_boundaries
 *   • Wikipedia — Voronoi diagram
 *     https://en.wikipedia.org/wiki/Voronoi_diagram
 *   • Lague, S. — "Tectonic Plate Simulation for Procedural Terrain"
 *     https://www.youtube.com/watch?v=x_Tn66PvTn4
 *   • Andy Gainey — "Procedural Worlds from Simple Tiles"
 *     https://experilous.com/1/blog/post/procedural-planet-generation
 *   • Perlin, K. (2002) — Improving Noise (the fBm scaffold)
 *     https://mrl.cs.nyu.edu/~perlin/paper445.pdf
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
 *  • Press 'r': flash + regenerate. The new world has a different
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

/* ===================================================================== */
/* §1  config                                                             */
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
    PAIR_FLASH          =  13,    /* regenerate flash                  */
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

/*
 * Biome buckets. Eight categories from deepest ocean to highest peaks.
 * Thresholds are on the elev value in [-1, +1].
 */
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

/* Boundary classification. */
typedef enum {
    BOUNDARY_NONE       = 0,
    BOUNDARY_CONVERGENT = 1,
    BOUNDARY_DIVERGENT  = 2,
    BOUNDARY_TRANSFORM  = 3,
} BoundaryKind;

/* Plate types. */
typedef enum {
    PLATE_OCEANIC     = 0,
    PLATE_CONTINENTAL = 1,
} PlateType;

/* Pattern — four ways to render the same world. */
typedef enum {
    PATTERN_WORLD     = 0,
    PATTERN_PLATES    = 1,
    PATTERN_STRESS    = 2,
    PATTERN_ELEVATION = 3,
    N_PATTERNS        = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_WORLD:     return "WORLD    ";
    case PATTERN_PLATES:    return "PLATES   ";
    case PATTERN_STRESS:    return "STRESS   ";
    case PATTERN_ELEVATION: return "ELEVATION";
    default:                return "?        ";
    }
}

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
 * All entries chosen from the brighter half of the 256-colour cube
 * so even A_DIM cells stay legible against a default-black terminal.
 */
typedef struct {
    const char *name;
    short       ramp[N_BIOMES];
    short       hot;
    short       cold;
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

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

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
/* §3  color                                                              */
/* ===================================================================== */

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
        init_pair(PAIR_FLASH, 226, -1);
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  tectonics                                                          */
/* ===================================================================== */

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

/* ----------------------------------------------------------------------- *
 * Plate + Cell + World                                                    *
 * ----------------------------------------------------------------------- */

typedef struct {
    int       x, y;
    float     vx, vy;
    PlateType type;
    float     base_elev;
} Plate;

typedef struct {
    uint8_t plate_id;
    uint8_t boundary;        /* BoundaryKind */
    int8_t  elev;            /* [-100, +100] */
    uint8_t biome;           /* Biome */
} Cell;

typedef struct {
    int    w, h;
    Plate  plates[N_PLATES_LIMIT];
    int    n_plates;
    Cell   cells[CELLS_MAX];
    int    seed;
} World;

static inline int widx(const World *w, int x, int y) { return y * w->w + x; }

/* ----------------------------------------------------------------------- *
 * Stage 1 — Plate placement (jittered grid).                              *
 * ----------------------------------------------------------------------- */

static void gen_plates(World *w, int seed)
{
    /* Pick N. */
    uint32_t h0 = hash3(seed, 0, 1);
    int n = N_PLATES_MIN + (int)(h0 % (uint32_t)(N_PLATES_MAX - N_PLATES_MIN + 1));
    if (n > N_PLATES_LIMIT) n = N_PLATES_LIMIT;

    /* Jittered grid. cols ≈ √(n · w/h) so cells are roughly square in
     * the aspect-corrected metric. */
    float aspect = (float)w->w / ((float)w->h * ASPECT_Y_F);
    int   cols   = (int)ceilf(sqrtf((float)n * aspect));
    if (cols < 1) cols = 1;
    int rows = (n + cols - 1) / cols;

    int cell_w = w->w / cols;
    int cell_h = w->h / rows;
    if (cell_w < 1) cell_w = 1;
    if (cell_h < 1) cell_h = 1;

    int idx = 0;
    for (int r = 0; r < rows && idx < n; r++) {
        for (int c = 0; c < cols && idx < n; c++) {
            uint32_t h = hash3(c, r, seed);
            int cx = c * cell_w + cell_w / 2;
            int cy = r * cell_h + cell_h / 2;
            int jx = (int)((int)(h        & 0x3FFu) - 512) * cell_w / 2048;
            int jy = (int)((int)((h >> 10) & 0x3FFu) - 512) * cell_h / 2048;
            Plate *p = &w->plates[idx];
            p->x = cx + jx;
            p->y = cy + jy;
            if (p->x < 1)         p->x = 1;
            if (p->x > w->w - 2)  p->x = w->w - 2;
            if (p->y < 1)         p->y = 1;
            if (p->y > w->h - 2)  p->y = w->h - 2;

            /* Type — biased ~50/50, with the hash bit selecting. */
            p->type = ((h >> 22) & 1u) ? PLATE_CONTINENTAL : PLATE_OCEANIC;
            p->base_elev = (p->type == PLATE_OCEANIC)
                         ? BASE_ELEV_OCEAN : BASE_ELEV_CONTINENT;

            /* Velocity — random direction, magnitude ∈ [0.3, 1.0]. */
            uint32_t h2 = hash3(c, r, seed ^ 0x5A5A5A);
            float ang  = ((float)(h2 & 0xFFFFu) / 65536.0f) * 2.0f * (float)M_PI;
            float spd  = 0.3f + ((float)((h2 >> 16) & 0xFFFFu) / 65536.0f) * 0.7f;
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

static void compute_elevation(World *w)
{
    float r_max = (float)BOUNDARY_RX;

    for (int y = 0; y < w->h; y++) {
        for (int x = 0; x < w->w; x++) {
            int idx = widx(w, x, y);

            /* Find the nearest boundary cell within the aspect-
             * corrected neighbourhood. */
            float   nearest_d   = r_max + 1.0f;
            uint8_t nearest_kind = BOUNDARY_NONE;
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
                    if (d < nearest_d) {
                        nearest_d    = d;
                        nearest_kind = b;
                    }
                }
            }

            /* Boundary modifier — decay with distance. */
            float modifier = 0.0f;
            if (nearest_d < r_max) {
                float weight = 1.0f - nearest_d / r_max;
                if      (nearest_kind == BOUNDARY_CONVERGENT) modifier = MOD_CONVERGENT * weight;
                else if (nearest_kind == BOUNDARY_DIVERGENT)  modifier = MOD_DIVERGENT  * weight;
                /* TRANSFORM and NONE: no vertical effect. */
            }

            const Plate *p = &w->plates[w->cells[idx].plate_id];
            float noise = fbm2((float)x * FBM_SCALE_X,
                               (float)y * FBM_SCALE_Y) - 0.5f;     /* [-0.5, 0.5] */
            float elev  = p->base_elev + modifier + noise * FBM_AMPLITUDE;
            if (elev < -1.0f) elev = -1.0f;
            if (elev >  1.0f) elev =  1.0f;

            w->cells[idx].elev  = (int8_t)(elev * 100.0f);
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

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    World   world;
    bool    paused;
    int     speed;
    int     current_theme;
    Pattern current_pattern;
    float   time_secs;
    float   regen_t;          /* counts up; regen when > REGEN_SECONDS  */
    float   flash_t;
} Scene;

static void scene_rebuild(Scene *s)
{
    int seed = (int)hash3((int)(s->time_secs * 1000.0f),
                          s->world.w, s->world.h);
    world_build(&s->world, seed);
    s->regen_t  = 0.0f;
    s->flash_t  = 1.0f;
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
    s->flash_t   *= expf(-4.0f * dt);
    if (s->paused) return;

    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->regen_t += dt * speed_mul;
    if (s->regen_t >= REGEN_SECONDS) scene_rebuild(s);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

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

static void scene_draw(const Screen *sc, const Scene *s)
{
    switch (s->current_pattern) {
    case PATTERN_WORLD:     render_world(sc, s);     break;
    case PATTERN_PLATES:    render_plates(sc, s);    break;
    case PATTERN_STRESS:    render_stress(sc, s);    break;
    case PATTERN_ELEVATION: render_elevation(sc, s); break;
    case N_PATTERNS:        break;            /* unreachable sentinel */
    }

    /* Regenerate flash. */
    if (s->flash_t > 0.05f) {
        int seed = (int)(s->time_secs * 1000.0f);
        attron(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
        for (int sy = 2; sy < sc->rows - 1; sy += 2) {
            for (int sx = 0; sx < sc->cols; sx += 2) {
                if (((sx ^ sy ^ seed) & 7) == 0)
                    mvaddch(sy, sx, '*');
            }
        }
        attroff(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
    }
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

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);

    const World *w = &s->world;
    const char *state_str = s->paused ? "PAUSED   " : pattern_name(s->current_pattern);

    /* Row 0 right — primary status. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " TECTONIC WORLD ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 — pattern + theme + ramp swatch + plate count + regen %. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-9s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 20;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " ramp:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 6;
    for (int i = 0; i < N_BIOMES; i++) {
        int p = PAIR_RAMP_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, (chtype)(unsigned char)ELEV_RAMP[i]);
        attroff(COLOR_PAIR(p) | A_BOLD);
        x++;
    }

    int regen_pct = (int)((s->regen_t / REGEN_SECONDS) * 100.0f);
    if (regen_pct > 100) regen_pct = 100;
    if (regen_pct < 0)   regen_pct = 0;

    /* Land-vs-sea ratio for the HUD. */
    int counts[N_BIOMES];
    biome_counts(w, counts);
    int sea  = counts[BIOME_DEEP_OCEAN] + counts[BIOME_OCEAN];
    int land = counts[BIOME_PLAINS]    + counts[BIOME_HILLS]
             + counts[BIOME_MOUNTAINS] + counts[BIOME_HIGHLANDS]
             + counts[BIOME_PEAKS];
    int total = w->w * w->h;
    int sea_pct  = total ? (sea  * 100) / total : 0;
    int land_pct = total ? (land * 100) / total : 0;

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             "  plates:%-2d  sea:%2d%%  land:%2d%%  regen:%3d%% ",
             w->n_plates, sea_pct, land_pct, regen_pct);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:speed  ]/[:tickHz  spc:pause  r:regen  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

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
