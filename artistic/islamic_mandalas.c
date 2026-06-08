/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * islamic_mandalas.c — 30 parametric Islamic geometric patterns
 *                       (20 simple + 10 complex)
 *
 *
 * Section map (cut by layer — see ARCHITECTURE):
 *   §1 config       — preset table + Ring/Preset types + constants + themes
 *   §2 performance  — monotonic clock + sleep
 *   §3 logic        — pure maps & queries (polar↔cell, line glyph, build math)
 *   §4 data         — Scene runtime state
 *   §5 simulation   — scene_tick: advance the build animation + rotation
 *   §6 render       — colour, primitives, draw_mandala, scene_draw + HUD
 *   §7 init/reset   — scene reset / restart / init
 *   §8 events       — keys, signals, screen setup
 *   §9 app          — the frame loop (the per-tick combine)
 *
 * Keys:
 *   q / Q / ESC     quit
 *   n / →           next preset (restarts build animation)
 *   p / ←           previous preset (restarts build animation)
 *   b / B           replay build animation for current preset
 *   t / T           cycle colour theme
 *   + / =           scale up
 *   -               scale down
 *   r / R           toggle slow rotation
 *   space           pause / resume (freezes build + rotation)
 *   0               reset to defaults
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/islamic_mandalas.c \
 *       -o islamic_mandalas -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Parametric Islamic-geometric synthesis.  Each
 *                 preset is a list of RINGS, where each ring is one
 *                 of six radial primitives:
 *                   CIRCLE      continuous dotted circle ('.')
 *                   POLYGON     N-vertex regular polygon (line-drawn)
 *                   STAR_POLY   N vertices connected by skip-density
 *                                lines (e.g. octagram = 8/3, dodecagram
 *                                = 12/5).  Same primitive as Hindu.
 *                   STAR_SHAPE  TRUE star outline: alternates OUTER
 *                                vertices (radius r) and INNER
 *                                vertices (radius r·inner_ratio),
 *                                drawing a sharp-pointed star
 *                                outline with 2n edges.  Distinctly
 *                                Islamic-style.
 *                   INTERLOCK   TWO regular n-gons at offset π/n
 *                                rotation (e.g. 4 + 4 rotated 45° =
 *                                Khatim 8-pointed star; 3 + 3 rotated
 *                                60° = hexagram Solomon's seal).
 *                   RAYS        N lines from inner gap to outer
 *                                radius (sun / compass patterns).
 *                 Optional centre marker '+' for medallion presets.
 *                 Thirty PRESETS pick different combinations to
 *                 produce thirty named geometric patterns.
 *
 * Data-structure: PRESETS[30] of MandalaPreset = (name, up to MAX_RINGS
 *                 Rings, centre flag).  Each Ring = (type, count,
 *                 density_or_inner_pct, relative_radius).  No heap
 *                 allocation.  Compact R(...) macro lets each
 *                 preset fit on one line.
 *
 * Rendering     : Polar→cell mapping with aspect correction
 *                 (ASPECT = 0.5 keeps circles round on screen).
 *                 Lines walked cell-by-cell with glyph picked from
 *                 (dx, dy) direction.  Per-ring-type colour pair,
 *                 themed by 4-theme palette (Iznik / Persian /
 *                 Andalusian / Mamluk).  Rings build progressively
 *                 over RING_BUILD_DUR seconds each; features fill in
 *                 sequentially within a ring.
 *
 * Performance   : O(rings · features · cells_per_feature) per frame.
 *                 At 4 rings × 32 features × ~12 cells = ~1500
 *                 paints per frame, microseconds.
 *
 * Strapwork simplification: real Islamic geometric art is built
 * from interlace strapwork — ribbons that pass over and under
 * each other at every crossing, creating a continuous 3-D woven
 * effect.  We render lines as plain crossings (no over-under).
 * The parametric structure is faithful to the underlying
 * geometry; the strapwork illusion is a lossy ASCII compromise.
 *
 * References    :
 *   Islamic geometric construction (the preset forms + their symmetry):
 *     Critchlow, "Islamic Patterns: An Analytical and Cosmological Approach"
 *       (1976) — the canonical introduction to the compass-and-straightedge
 *       construction behind these patterns.
 *     Bourgoin, "Arabic Geometrical Pattern and Design" (Dover reprint, 1973)
 *       — 200+ classical patterns analysed; a catalogue of the named forms.
 *     Broug, "Islamic Geometric Patterns" (Thames & Hudson, 2008) — a modern
 *       step-by-step guide to building each pattern from a circle subdivision.
 *     Lu & Steinhardt, "Decagonal and Quasi-Crystalline Tilings in Medieval
 *       Islamic Architecture" (Science 315, 2007) — the girih-tile / near-
 *       quasicrystal analysis behind the most intricate star patterns.
 *     Wikipedia, "Islamic geometric patterns", "Star polygon", "Girih tiles".
 *       https://en.wikipedia.org/wiki/Islamic_geometric_patterns
 *
 *   Geometry & rendering (star polygons, tilings, line raster, polar layout):
 *     Coxeter, "Regular Polytopes" (1973) — star polygon {n/d} notation and
 *       the gcd-based enumeration behind the STAR_POLY rings.
 *     Grünbaum & Shephard, "Tilings and Patterns" (1987) — the definitive
 *       treatment of star polygons, symmetry groups, and tilings.
 *     Bresenham, "Algorithm for computer control of a digital plotter"
 *       (IBM Syst. J. 1965) — the integer line walk behind the cell-by-cell
 *       segment drawing (POLYGON / STAR / INTERLOCK / RAYS rings).
 *     Foley, van Dam, Feiner & Hughes, "Computer Graphics: Principles and
 *       Practice" — polar / parametric primitives and 2-D raster line drawing
 *       behind §3's polar→cell mapping and the §6 ring draws.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * An Islamic geometric pattern is a SYMMETRIC DESIGN built from a
 * small alphabet of primitives — interlocking polygons (two
 * triangles = hexagram, two squares = 8-pointed star), star
 * polygons (octagram, dodecagram), outlined sharp stars, and
 * radial rays.  Thirty named patterns turn out to be thirty
 * parameter combinations of THE SAME draw function.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a tile-maker's stencil kit.  Stencils for an octagram
 * (8/3 star), a hexagram (two triangles), a sharp 12-point star
 * outline, a regular octagon frame, a circular border, a 16-spoke
 * sunburst.  An Andalusian tile is "octagram + outer circle"; a
 * Persian medallion is "two squares + outlined 8-star + octagonal
 * frame + circle".  Same kit, different recipe.
 *
 * VISUAL GRAMMAR
 * ──────────────
 *   STAR_POLY (line star)         STAR_SHAPE (outlined star)
 *
 *        \  |  /                       /\
 *         \ | /                       /  \
 *      ----X----                  ---/    \---
 *         / | \                       \  /
 *        /  |  \                       \/
 *
 *   "vertices connected by lines        "alternating outer-inner
 *    skipping d positions"                vertices, sharp points"
 *
 *   INTERLOCK (two polygons offset)
 *
 *           ▲         + 60°    ▽
 *          / \                 \-/
 *         /   \                /-\
 *         -----                ▲
 *
 *   "two regular n-gons rotated by π/n; together they form a
 *    2n-pointed star (4+4=Khatim, 3+3=hexagram)"
 *
 * ALGORITHM IN STEPS  (per frame)
 * ───────────────────────────────
 *   1. Find screen centre (cx, cy) and base radius from
 *      min(cols/2, rows/(2·ASPECT)) · scale.
 *   2. For each ring in the preset, gated by build_time:
 *      a. compute progress ∈ [0, 1] within the ring's time window
 *      b. dispatch by type to the matching draw primitive,
 *         passing the progress so partial features render
 *   3. Draw centre marker '+' if the preset has centre_dot set.
 *   4. HUD row + key hint strip.
 *
 * KEY FORMULAS
 * ────────────
 *   Polar→cell:
 *     col = cx + r·cos θ
 *     row = cy + r·sin θ · ASPECT      (ASPECT ≈ 0.5)
 *
 *   Star polygon (line star, n vertices, density d):
 *     edge i connects vertex[i] to vertex[(i+d) mod n]
 *     gcd(n, d) = 1 → single connected star
 *     gcd(n, d) > 1 → multi-graph (e.g. INTERLOCK 4 emulates
 *                                    star polygon 8/2)
 *
 *   Outlined star (n points, inner ratio ρ ∈ (0, 1)):
 *     2n vertices: outer at radius r, inner at radius r·ρ
 *     vertex k at angle θ_k = rot + π·k/n
 *     2n edges connecting consecutive vertices
 *
 *   Interlocking polygons:
 *     polygon A: rotation rot, n edges
 *     polygon B: rotation rot + π/n, n edges
 *     total: 2n edges; build draws A first, then B
 *
 *   Inner ratio encoding:
 *     density field in Ring stores integer 0..100 = percent
 *     inner_ratio = density / 100  (e.g. 50 = 0.50)
 *
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * The file is cut into LAYERS by concern. All runtime state lives on one Scene
 * (§4); each layer reads and/or mutates a named slice of it. Functions take the
 * NARROWEST type they need — const Scene* to read, Scene* to mutate — so the
 * layers never re-couple. The split is by SECTION; this table lists what each
 * mutates.
 *
 *   Layer        Section            Mutates
 *   ─────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2 performance     nothing (reads OS clock, sleeps)
 *   LOGIC        §3 logic           nothing (pure maps & queries)
 *   DATA         §4 data            — Scene declaration + the instance —
 *   SIMULATION   §5 simulation      scene.build_time, .build_complete, .rot
 *   RENDER       §6 render          the screen + the colour pairs; never Scene
 *   INIT/RESET   §7 init            ALL of Scene (defaults / build restart)
 *   EVENTS       §8 events          scene.{preset_idx, theme_idx, scale, ...}
 *   —            §9 app             the per-frame driver (combines layers)
 *
 * SIMULATION is THIN here: the pattern is a STATIC parametric drawing, so the
 * only state that advances per tick is the build-reveal timer (build_time) and
 * the rotation angle (rot) — cosmetic ANIMATION, not physics.
 *
 * No separate EFFECTS layer: that cosmetic animation state IS the per-tick state
 * (build_time / rot), advanced by SIMULATION and read by RENDER; there is no
 * extra stored effect (glow / trail / flash) on top of it.
 *
 * No separate DELAYS layer: pause is a single flag (scene.paused) tested once at
 * the top of scene_tick; the build reveal is a timer (build_time) owned by
 * SIMULATION; the fps window is a PERFORMANCE counter in main.
 *
 * LOGIC (polar_to_cell, line_glyph, progress_to_count, preset_ring_count,
 * preset_build_duration) does no mutation and no I/O — it maps inputs to a
 * value — so reordering or deleting RENDER cannot change a LOGIC result.
 *
 * PER-TICK COMBINE — main's loop (§9) advances state in ONE place, in order:
 *   1. scene_tick(dt)   — advance build animation + rotation        (SIMULATION)
 *   2. scene_draw()     — project the preset's rings to screen + HUD (RENDER)
 *
 * User events (quit, next/prev preset, theme, size, rotate, pause, replay,
 * reset, resize) DO mutate state but are NOT part of the tick — they run in
 * main's input/resize handling (scene_input, §8), before scene_tick. Reset (0)
 * and preset change re-invoke the INIT-style restarts in §7.
 *
 * ─────────────────────────────────────────────────────────────────────── */


#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <locale.h>

/* ===================================================================== */
/* §1  config                                                              */
/* ===================================================================== */

enum {
    TARGET_FPS    = 60,
    FPS_UPDATE_MS = 500,

    MAX_RINGS = 8,    /* enough for complex presets (some use 7) */
    N_PRESETS = 30,
    N_THEMES  = 4,

    /* Color pair IDs */
    PAIR_CENTRE     = 1,
    PAIR_CIRCLE     = 2,
    PAIR_POLYGON    = 3,
    PAIR_STAR_POLY  = 4,
    PAIR_STAR_SHAPE = 5,
    PAIR_INTERLOCK  = 6,
    PAIR_RAYS       = 7,
    PAIR_HUD        = 8,
    PAIR_HINT       = 9,
};

#define ASPECT        0.5f       /* terminal cells ~2:1 tall:wide */
#define ROT_RATE      0.18f      /* rad/sec when rotation enabled */
#define SCALE_MIN     0.40f
#define SCALE_MAX     1.00f
#define SCALE_DEFAULT 0.85f
#define SCALE_STEP    0.05f
#define NS_PER_SEC    1000000000LL

#define BINDU_AT       0.10f
#define RING_BUILD_DUR 0.55f

/* Primitive geometry */
#define CIRCLE_OVERSAMPLE  1.4f   /* '.' samples per cell of circle circumference */
#define CIRCLE_SAMPLES_MIN 24     /* clamp so small circles still read as a ring  */
#define CIRCLE_SAMPLES_MAX 360    /* and large ones don't oversample pointlessly  */
#define STAR_INNER_MIN     0.10f  /* outlined-star inner radius ratio: floor       */
#define STAR_INNER_MAX     0.95f  /*   (keeps points from collapsing / flattening) */
#define RAY_INNER_FRAC     0.10f  /* rays start this far out → a centre gap        */

/* Layout */
#define SCREEN_MARGIN      4      /* cells kept clear around the pattern (HUD rows) */
#define MANDALA_MIN_R      2.0f   /* never shrink the pattern below this radius     */

/* §1.1 Ring + Preset types ------------------------------------------- */

/* RingType — the six RADIAL PRIMITIVES every Islamic pattern in this file is
 * built from. A pattern is a concentric stack of rings, each one of these laid
 * out around a shared centre; combining a handful at different radii reproduces
 * the canonical girih / tile motifs (Critchlow; Bourgoin). RING_NONE is the
 * array SENTINEL — the first one ends a preset's ring list.
 *   RING_NONE       : end-of-list marker (value 0 so {0}-init ends a preset)
 *   RING_CIRCLE     : a continuous dotted ring ('.')
 *   RING_POLYGON    : a regular n-gon (n vertices joined edge-to-edge)
 *   RING_STAR_POLY  : a line star polygon {n/d} — vertices joined with skip d
 *                     (octagram {8/3}, dodecagram {12/5}); see Coxeter
 *   RING_STAR_SHAPE : a TRUE outlined star — 2n vertices alternating outer
 *                     radius r and inner radius r·ρ, drawing sharp points
 *   RING_INTERLOCK  : two n-gons offset by π/n (4+4 = Khatim 8-star, 3+3 =
 *                     hexagram / Solomon's seal)
 *   RING_RAYS       : N spokes from an inner gap to the rim (sun / compass) */
typedef enum {
    RING_NONE = 0, RING_CIRCLE, RING_POLYGON, RING_STAR_POLY,
    RING_STAR_SHAPE, RING_INTERLOCK, RING_RAYS,
} RingType;

/* Ring — one concentric layer of a pattern: a single RingType primitive drawn N
 * times around the centre at a relative radius. WHY four fields and no more: a
 * ring is fully specified by WHICH primitive, HOW MANY of it, one extra integer
 * the primitive interprets, and WHERE (radius). The `density` field is
 * OVERLOADED by type — a deliberate space saving so every preset fits one line:
 *   STAR_POLY  : the star-polygon stride d of {n/d} (Coxeter) — vertex i joins
 *                vertex (i+d) mod n; d=1 is a polygon, d≥2 a star.
 *   STAR_SHAPE : the inner radius as a PERCENT of the outer (0..100), so 50 →
 *                inner vertices at r·0.50 (points sharpen as the percent drops).
 *   others     : unused (0).
 *   type    : a RingType (stored as int; an array of these ends at RING_NONE).
 *   n       : feature count — vertices / points / rays (0 for CIRCLE, whose
 *             sample count is derived from its radius at draw time).
 *   density : the per-type extra integer described above.
 *   radius  : radius as a FRACTION of the pattern's base radius, in (0, 1]. */
typedef struct {
    int   type;     /* RingType */
    int   n;        /* feature count (or 0 for RING_CIRCLE) */
    int   density;  /* STAR_POLY: skip stride; STAR_SHAPE: inner_pct (0..100) */
    float radius;   /* relative to base_r ∈ (0, 1] */
} Ring;

/* MandalaPreset — one named Islamic pattern as a RECIPE: a centre flag plus an
 * ordered list of Rings. This is the file's central lesson — "thirty different
 * patterns" is really ONE parametric draw function fed thirty parameter sets.
 * The named forms (Khatim, Octagram, Alhambra Star, Quasi-Crystal 10, ...) are
 * canonical tilings / girih designs (Critchlow; Bourgoin; Lu & Steinhardt for
 * the near-quasicrystal stars); each is reproduced by choosing rings at chosen
 * radii. The rings[] order is innermost-to-outermost and also drives the
 * build-in ANIMATION — ring i reveals during its time window.
 *   name       : label shown in the HUD.
 *   rings      : up to MAX_RINGS layers; the list ends at the first RING_NONE
 *                entry, so short presets {0}-pad the remainder.
 *   centre_dot : draw a central '+' medallion marker (painted last so it always
 *                wins the centre cell); off for star-only patterns. */
typedef struct {
    const char *name;
    Ring rings[MAX_RINGS];
    bool centre_dot;
} MandalaPreset;

/* Compact preset constructor.  Macro params CC/DD/RR avoid colliding with
 * struct field names (`.n`, `.density`, `.radius` — the preprocessor
 * would substitute plain `n`/`d`/`r` inside those designators). */
#define R(t,CC,DD,RR) {.type = RING_##t, .n = (CC), .density = (DD), .radius = (RR)}

static const MandalaPreset PRESETS[N_PRESETS] = {
/*  ─────────── 20 named Islamic geometric patterns ──────────────────────── */
    {"Khatim",          {R(INTERLOCK,    4, 0,  0.65f), R(CIRCLE,        0,  0, 0.92f)},                                                                                  true},
    {"Hexagram",        {R(INTERLOCK,    3, 0,  0.65f), R(CIRCLE,        0,  0, 0.92f)},                                                                                  true},
    {"Octagram",        {R(STAR_POLY,    8, 3,  0.65f), R(CIRCLE,        0,  0, 0.92f)},                                                                                  true},
    {"Decagram",        {R(STAR_POLY,   10, 3,  0.65f), R(CIRCLE,        0,  0, 0.92f)},                                                                                  true},
    {"Dodecagram",      {R(STAR_POLY,   12, 5,  0.65f), R(CIRCLE,        0,  0, 0.92f)},                                                                                  true},
    {"Rub-el-hizb",     {R(INTERLOCK,    4, 0,  0.55f), R(POLYGON,       8,  0, 0.78f), R(CIRCLE,        0,  0, 0.92f)},                                                  true},
    {"Outlined 5-Star", {R(STAR_SHAPE,   5, 38, 0.70f), R(CIRCLE,        0,  0, 0.92f)},                                                                                  false},
    {"Outlined 8-Star", {R(STAR_SHAPE,   8, 50, 0.70f), R(CIRCLE,        0,  0, 0.92f)},                                                                                  false},
    {"Outlined 12-Star",{R(STAR_SHAPE,  12, 55, 0.70f), R(CIRCLE,        0,  0, 0.92f)},                                                                                  false},
    {"Persian 8-Fold",  {R(INTERLOCK,    4, 0,  0.45f), R(STAR_SHAPE,    8, 50, 0.70f), R(POLYGON,       8,  0, 0.85f), R(CIRCLE,        0,  0, 0.92f)},                  true},
    {"Iznik Medallion", {R(STAR_POLY,    8, 3,  0.45f), R(STAR_SHAPE,   16, 60, 0.72f), R(POLYGON,      16,  0, 0.88f)},                                                  false},
    {"Andalusian",      {R(INTERLOCK,    4, 0,  0.45f), R(STAR_POLY,     8,  3, 0.65f), R(CIRCLE,        0,  0, 0.92f)},                                                  false},
    {"Selcuk",          {R(STAR_SHAPE,  10, 40, 0.50f), R(STAR_SHAPE,   10, 60, 0.78f), R(CIRCLE,        0,  0, 0.92f)},                                                  false},
    {"Mamluk Burst",    {R(RAYS,        16, 0,  0.70f), R(STAR_POLY,    16,  5, 0.55f), R(CIRCLE,        0,  0, 0.85f)},                                                  false},
    {"Tabriz Compass",  {R(RAYS,         8, 0,  0.85f), R(RAYS,         16,  0, 0.85f), R(STAR_SHAPE,    8, 45, 0.55f), R(CIRCLE,        0,  0, 0.92f)},                  true},
    {"Samarkand Sun",   {R(RAYS,        24, 0,  0.85f), R(STAR_POLY,    12,  5, 0.55f), R(CIRCLE,        0,  0, 0.70f), R(CIRCLE,        0,  0, 0.92f)},                  false},
    {"Cairo Tile",      {R(STAR_POLY,   12, 5,  0.55f), R(INTERLOCK,     6,  0, 0.30f), R(CIRCLE,        0,  0, 0.85f)},                                                  false},
    {"Maghrebi Knot",   {R(STAR_POLY,    5, 2,  0.40f), R(STAR_SHAPE,   10, 50, 0.70f), R(CIRCLE,        0,  0, 0.92f)},                                                  false},
    {"Alhambra Star",   {R(STAR_SHAPE,  16, 55, 0.70f), R(POLYGON,      16,  0, 0.85f), R(CIRCLE,        0,  0, 0.92f)},                                                  false},
    {"Mosque Window",   {R(POLYGON,      6, 0,  0.85f), R(STAR_POLY,     6,  2, 0.55f), R(CIRCLE,        0,  0, 0.92f)},                                                  false},

/*  ─────────── 10 complex Islamic patterns — 5-7 rings each ─────────── */
    {"Topkapi Scroll",        {R(INTERLOCK,   5,  0, 0.30f), R(STAR_SHAPE,  10, 50, 0.50f), R(STAR_POLY,   10,  3, 0.65f), R(POLYGON,     10,  0, 0.85f), R(CIRCLE,        0,  0, 0.92f)},                                                                          true},
    {"Konya Rosette",         {R(STAR_POLY,  12,  5, 0.30f), R(STAR_SHAPE,  12, 55, 0.50f), R(INTERLOCK,    6,  0, 0.65f), R(POLYGON,     12,  0, 0.82f), R(RAYS,        24,  0, 0.92f), R(CIRCLE,        0,  0, 0.95f)},                                            false},
    {"Damascus Dome",         {R(STAR_POLY,  16,  7, 0.30f), R(STAR_SHAPE,  16, 60, 0.50f), R(INTERLOCK,    8,  0, 0.65f), R(POLYGON,     16,  0, 0.82f), R(CIRCLE,        0,  0, 0.88f), R(CIRCLE,        0,  0, 0.94f)},                                          false},
    {"Cordoba Mihrab",        {R(STAR_SHAPE,  8, 45, 0.30f), R(STAR_POLY,    8,  3, 0.50f), R(INTERLOCK,    4,  0, 0.65f), R(POLYGON,      8,  0, 0.80f), R(RAYS,        16,  0, 0.92f), R(CIRCLE,        0,  0, 0.95f)},                                            true},
    {"Isfahan Garden",        {R(STAR_POLY,  12,  5, 0.28f), R(STAR_SHAPE,  12, 55, 0.46f), R(INTERLOCK,    6,  0, 0.60f), R(POLYGON,     12,  0, 0.74f), R(STAR_SHAPE,  24, 65, 0.85f), R(RAYS,        24,  0, 0.92f), R(CIRCLE,        0,  0, 0.96f)},              false},
    {"Cairo Stellate",        {R(INTERLOCK,   6,  0, 0.30f), R(STAR_POLY,   12,  5, 0.50f), R(STAR_SHAPE,  12, 55, 0.65f), R(RAYS,        12,  0, 0.85f), R(POLYGON,     12,  0, 0.85f), R(CIRCLE,        0,  0, 0.92f)},                                            false},
    {"Granada Constellation", {R(STAR_SHAPE, 16, 60, 0.30f), R(STAR_POLY,   16,  7, 0.50f), R(INTERLOCK,    8,  0, 0.65f), R(POLYGON,     16,  0, 0.78f), R(RAYS,        32,  0, 0.88f), R(CIRCLE,        0,  0, 0.94f)},                                            false},
    {"Bursa Mosque",          {R(STAR_POLY,  10,  3, 0.30f), R(STAR_SHAPE,  10, 50, 0.50f), R(INTERLOCK,    5,  0, 0.65f), R(POLYGON,     10,  0, 0.78f), R(RAYS,        20,  0, 0.88f), R(CIRCLE,        0,  0, 0.94f)},                                            false},
    {"Marrakesh Tile",        {R(STAR_POLY,   8,  3, 0.25f), R(STAR_POLY,   16,  7, 0.42f), R(STAR_SHAPE,   8, 45, 0.55f), R(STAR_SHAPE, 16, 60, 0.72f), R(INTERLOCK,    4,  0, 0.85f), R(POLYGON,     16,  0, 0.92f), R(CIRCLE,        0,  0, 0.95f)},              false},
    {"Quasi-Crystal 10",      {R(STAR_POLY,  10,  3, 0.25f), R(STAR_POLY,   10,  4, 0.40f), R(STAR_SHAPE,  10, 45, 0.55f), R(STAR_SHAPE, 20, 65, 0.72f), R(INTERLOCK,    5,  0, 0.85f), R(RAYS,        20,  0, 0.92f), R(CIRCLE,        0,  0, 0.95f)},               false},
};

/* §1.2 Themes — six ring-type colours per theme, plus centre marker. */
static const int THEME_PALETTE[N_THEMES][6] = {
    /*           CIRCLE POLY  S_POLY S_SHAPE INTER  RAYS                  */
    /* IZNIK    */ {  39,   45,   51,   75,  117,   45 }, /* turquoise + cobalt */
    /* PERSIAN  */ {  27,   33,   75,  117,  159,   33 }, /* deep blue + ivory */
    /* ANDALUS  */ { 118,  154,   46,  220,  214,  154 }, /* green + gold */
    /* MAMLUK   */ { 220,  214,  208,  202,  166,  220 }, /* gold + red */
};
static const int   THEME_CENTRE[N_THEMES] = { 195, 230, 230, 230 };
static const char *THEME_NAME  [N_THEMES] = { "Iznik", "Persian", "Andalusian", "Mamluk" };

/* ===================================================================== */
/* §2  PERFORMANCE — clock                                                 */
/* ===================================================================== *
 * Monotonic clock + sleep. Mutates nothing. The frame cap, dt clamp and fps
 * window that use these live in main's loop (§9).                            */

static int64_t clock_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
    if (ns <= 0) return;
    struct timespec ts = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  LOGIC — pure maps & queries (no mutation, no I/O)                   */
/* ===================================================================== *
 * Each maps its inputs to a value (or out-params); no globals, no screen. So
 * reordering or deleting RENDER cannot change a result here.                 */

/* polar→cell with terminal aspect correction (sin scaled by ASPECT). */
static inline void polar_to_cell(int cx, int cy, float r, float theta,
                                 int *col, int *row) {
    *col = cx + (int)roundf(r * cosf(theta));
    *row = cy + (int)roundf(r * sinf(theta) * ASPECT);
}

/* angle of the i-th of n features evenly spaced around the circle, plus rot. */
static inline float feature_angle(int i, int n, float rot) {
    return rot + (float)i / (float)n * 2.0f * (float)M_PI;
}

/* valid star-polygon stride d: 1 (a plain polygon) unless 2 <= d < n (a star). */
static int star_stride(int density, int n) {
    int d = (density < 2) ? 1 : density;
    if (d >= n) d = 1;
    return d;
}

/* outlined-star inner radius as a fraction of the outer, clamped to a drawable
 * range so its points neither collapse to the centre nor flatten into the rim. */
static float star_inner_ratio(int inner_pct) {
    float ratio = (float)inner_pct / 100.0f;
    if (ratio < STAR_INNER_MIN) ratio = STAR_INNER_MIN;
    if (ratio > STAR_INNER_MAX) ratio = STAR_INNER_MAX;
    return ratio;
}

/* Pick an ASCII line glyph that matches a (dx, dy) direction in cells. */
static char line_glyph(int dx, int dy) {
    float adx = (float)abs(dx);
    float ady = (float)abs(dy) / ASPECT;
    if (adx < 0.5f) return '|';
    if (ady < 0.5f) return '-';
    float r = ady / adx;
    if (r < 0.5f) return '-';
    if (r > 2.0f) return '|';
    return ((dx > 0) == (dy > 0)) ? '\\' : '/';
}

/* progress_to_count — see hindu_mandalas.c for the float-roundoff
 * tolerance rationale.  Same fix here. */
static int progress_to_count(int n, float progress) {
    if (progress >= 0.999f) return n;
    if (progress <= 0.0f)   return 0;
    int k = (int)(progress * (float)n);
    if (k < 1) k = 1;
    if (k > n) k = n;
    return k;
}

/* ring count + total build duration for a preset */
static int preset_ring_count(const MandalaPreset *p) {
    int n = 0;
    for (int i = 0; i < MAX_RINGS; i++) {
        if (p->rings[i].type == RING_NONE) break;
        n++;
    }
    return n;
}

static float preset_build_duration(const MandalaPreset *p) {
    return BINDU_AT + (float)preset_ring_count(p) * RING_BUILD_DUR;
}

/* build-animation progress of ring i in [0,1]; <= 0 before its reveal window
 * opens. Ring i's window is [BINDU_AT + i·RING_BUILD_DUR, +RING_BUILD_DUR]. */
static float ring_build_progress(int i, float build_time) {
    float ring_start = BINDU_AT + (float)i * RING_BUILD_DUR;
    float prog = (build_time - ring_start) / RING_BUILD_DUR;
    if (prog > 1.0f) prog = 1.0f;
    return prog;
}

/* base pattern radius that fits the screen (minus the HUD margin), scaled by the
 * user size; the ×0.5 and /ASPECT make it round in 2:1 terminal cells. */
static float mandala_base_radius(int rows, int cols, float scale) {
    float max_r_x = (float)(cols - SCREEN_MARGIN) * 0.5f;
    float max_r_y = (float)(rows - SCREEN_MARGIN) * 0.5f / ASPECT;
    float base_r  = fminf(max_r_x, max_r_y) * scale;
    if (base_r < MANDALA_MIN_R) base_r = MANDALA_MIN_R;
    return base_r;
}

/* ===================================================================== */
/* §4  DATA — Scene runtime state                                          */
/* ===================================================================== *
 * Declaration + the single instance; no behaviour. Mutated by SIMULATION (§5)
 * / INIT (§7) / EVENTS (§8) and read by RENDER (§6).                         */

/* Scene — the whole runtime view in one aggregate, read like a table of
 * contents. Functions take the NARROWEST slice they need (const Scene* to read,
 * Scene* to mutate); only the orchestrators (scene_init / scene_reset /
 * scene_tick) and the event handler take Scene*, so the layers never re-couple.
 *   WHAT   — preset_idx: which of the PRESETS[] patterns is on screen.
 *   HOW    — the user-tunable knobs: scale (size), rotation_on (spin toggle).
 *   WHEN   — the cosmetic animation advanced per tick: build_time +
 *            build_complete (the ring-by-ring reveal) and rot (rotation angle);
 *            paused freezes both.
 *   RENDER — theme_idx: the selected colour-palette index.
 *   FPS    — a sliding-window frame-rate meter, shown in the HUD only. */
typedef struct {
    /* WHAT — the pattern on screen */
    int     preset_idx;       /* index into PRESETS[]  [0..N_PRESETS)          */
    /* HOW — user-tunable knobs */
    float   scale;            /* size multiplier  [SCALE_MIN..SCALE_MAX]       */
    bool    rotation_on;      /* slow rotation enabled?                        */
    /* WHEN — cosmetic animation + run state */
    float   build_time;       /* seconds into the ring-by-ring build reveal    */
    bool    build_complete;   /* true once build_time >= the preset's duration */
    float   rot;              /* current rotation angle (radians)              */
    bool    paused;           /* 1 freezes the build + rotation                */
    /* RENDER — palette selection */
    int     theme_idx;        /* index into THEME_*  [0..N_THEMES)             */
    /* FPS — sliding-window frame-rate meter (HUD only) */
    float   fps;              /* last computed frames/sec                      */
    int64_t fps_window_start; /* clock_ns at the window's start                */
    int     frames_in_window; /* frames counted since the window start         */
} Scene;

static Scene g_scene;

/* ===================================================================== */
/* §5  SIMULATION — scene_tick (advance the build animation + rotation)    */
/* ===================================================================== *
 * The ONLY per-tick state advance. The pattern is a static drawing, so the only
 * state that moves is cosmetic ANIMATION: the build-reveal timer (build_time /
 * build_complete) and the rotation angle (rot). Paused short-circuits both.    */

static void scene_tick(Scene *s, float dt) {
    if (s->paused) return;

    if (!s->build_complete) {
        const MandalaPreset *p = &PRESETS[s->preset_idx];
        float dur = preset_build_duration(p);
        s->build_time += dt;
        if (s->build_time >= dur) {
            s->build_time     = dur;
            s->build_complete = true;
        }
    }

    if (s->rotation_on) {
        s->rot += ROT_RATE * dt;
        const float TWO_PI = 2.0f * (float)M_PI;
        while (s->rot >  TWO_PI) s->rot -= TWO_PI;
        while (s->rot < -TWO_PI) s->rot += TWO_PI;
    }
}

/* ===================================================================== */
/* §6  RENDER — colour setup, primitives, draw_mandala, scene_draw         */
/* ===================================================================== *
 * State → screen (reads only, never mutates Scene). colour_init/theme_apply
 * load pairs; the draw_* primitives stamp cells via paint_cell; draw_mandala
 * dispatches a preset's rings; scene_draw composes the frame + HUD.           */

static void color_init(void) {
    use_default_colors();
    init_pair(PAIR_HUD,  226, -1);
    init_pair(PAIR_HINT,  51, -1);
}

static void theme_apply(int idx) {
    init_pair(PAIR_CENTRE,     THEME_CENTRE [idx],     -1);
    init_pair(PAIR_CIRCLE,     THEME_PALETTE[idx][0],  -1);
    init_pair(PAIR_POLYGON,    THEME_PALETTE[idx][1],  -1);
    init_pair(PAIR_STAR_POLY,  THEME_PALETTE[idx][2],  -1);
    init_pair(PAIR_STAR_SHAPE, THEME_PALETTE[idx][3],  -1);
    init_pair(PAIR_INTERLOCK,  THEME_PALETTE[idx][4],  -1);
    init_pair(PAIR_RAYS,       THEME_PALETTE[idx][5],  -1);
}

/* paint_cell: bounds-checked stamp.  Reserves rows 0 and rows-1 for HUD. */
static void paint_cell(int col, int row, char ch, int pair, int attr) {
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (col < 0 || col >= cols)        return;
    if (row < 1 || row >= rows - 1)    return;
    attron (COLOR_PAIR(pair) | attr);
    mvaddch(row, col, (chtype)(unsigned char)ch);
    attroff(COLOR_PAIR(pair) | attr);
}

static void draw_line(int x1, int y1, int x2, int y2, int pair, int attr) {
    int dx = x2 - x1, dy = y2 - y1;
    int adx = abs(dx), ady = abs(dy);
    int n_steps = (adx > ady ? adx : ady);
    if (n_steps < 1) n_steps = 1;
    char ch = line_glyph(dx, dy);
    for (int i = 0; i <= n_steps; i++) {
        float t  = (float)i / (float)n_steps;
        int   col = x1 + (int)roundf(t * (float)dx);
        int   row = y1 + (int)roundf(t * (float)dy);
        paint_cell(col, row, ch, pair, attr);
    }
}

/* circle — many '.' samples around a continuous ring */
static void draw_circle(int cx, int cy, float r, float progress) {
    if (progress <= 0.0f || r < 0.5f) return;
    int n = (int)(2.0f * (float)M_PI * r * CIRCLE_OVERSAMPLE);
    if (n < CIRCLE_SAMPLES_MIN) n = CIRCLE_SAMPLES_MIN;
    if (n > CIRCLE_SAMPLES_MAX) n = CIRCLE_SAMPLES_MAX;
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t = feature_angle(i, n, 0.0f);
        int col, row;
        polar_to_cell(cx, cy, r, t, &col, &row);
        paint_cell(col, row, '.', PAIR_CIRCLE, A_NORMAL);
    }
}

/* polygon — N corners connected by lines (regular n-gon) */
static void draw_polygon(int cx, int cy, float r, int n, float rot, float progress) {
    if (n < 3 || progress <= 0.0f) return;
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t1 = feature_angle(i,           n, rot);
        float t2 = feature_angle((i + 1) % n, n, rot);
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, r, t1, &x1, &y1);
        polar_to_cell(cx, cy, r, t2, &x2, &y2);
        draw_line(x1, y1, x2, y2, PAIR_POLYGON, A_BOLD);
    }
}

/* star_poly — N vertices, density-d skip lines (octagram, etc.) */
static void draw_star_poly(int cx, int cy, float r, int n, int density,
                           float rot, float progress) {
    if (n < 3 || progress <= 0.0f) return;
    int d = star_stride(density, n);
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t1 = feature_angle(i,           n, rot);
        float t2 = feature_angle((i + d) % n, n, rot);
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, r, t1, &x1, &y1);
        polar_to_cell(cx, cy, r, t2, &x2, &y2);
        draw_line(x1, y1, x2, y2, PAIR_STAR_POLY, A_BOLD);
    }
}

/* star_shape — TRUE outlined star: n outer + n inner vertices,
 *      2n edges total, alternating outer-inner.  inner_pct is the
 *      inner radius as a percentage of outer radius (0..100). */
static void draw_star_shape(int cx, int cy, float r, int n, int inner_pct,
                            float rot, float progress) {
    if (n < 3 || progress <= 0.0f) return;
    float r_in = r * star_inner_ratio(inner_pct);
    int total_edges = 2 * n;
    int n_done = progress_to_count(total_edges, progress);

    for (int e = 0; e < n_done; e++) {
        int v1 = e;
        int v2 = (e + 1) % total_edges;
        bool v1_outer = (v1 % 2 == 0);
        bool v2_outer = (v2 % 2 == 0);
        float t1 = feature_angle(v1, total_edges, rot);
        float t2 = feature_angle(v2, total_edges, rot);
        float radius1 = v1_outer ? r : r_in;
        float radius2 = v2_outer ? r : r_in;
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, radius1, t1, &x1, &y1);
        polar_to_cell(cx, cy, radius2, t2, &x2, &y2);
        draw_line(x1, y1, x2, y2, PAIR_STAR_SHAPE, A_BOLD);
    }
}

/* interlock — TWO regular n-gons rotated by π/n.  Total 2n
 *      edges; build draws polygon A first (n edges), then B.
 *      Khatim 8-pointed star = INTERLOCK 4 (two squares).
 *      Hexagram (Solomon's seal) = INTERLOCK 3 (two triangles). */
static void draw_interlock(int cx, int cy, float r, int n,
                           float rot, float progress) {
    if (n < 3 || progress <= 0.0f) return;
    int total_edges = 2 * n;
    int n_done = progress_to_count(total_edges, progress);

    for (int e = 0; e < n_done; e++) {
        bool first = (e < n);
        int  edge  = first ? e : (e - n);
        float poly_rot = rot + (first ? 0.0f : ((float)M_PI / (float)n));
        float t1 = feature_angle(edge,           n, poly_rot);
        float t2 = feature_angle((edge + 1) % n, n, poly_rot);
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, r, t1, &x1, &y1);
        polar_to_cell(cx, cy, r, t2, &x2, &y2);
        draw_line(x1, y1, x2, y2, PAIR_INTERLOCK, A_BOLD);
    }
}

/* rays — N lines from inner gap to outer radius */
static void draw_rays(int cx, int cy, float r, int n, float rot, float progress) {
    if (n < 1 || progress <= 0.0f) return;
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t = feature_angle(i, n, rot);
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, r * RAY_INNER_FRAC, t, &x1, &y1);
        polar_to_cell(cx, cy, r,                  t, &x2, &y2);
        draw_line(x1, y1, x2, y2, PAIR_RAYS, A_BOLD);
    }
}

/* dispatcher — render rings up to current build_time, centre last */
static void draw_mandala(const MandalaPreset *p,
                         int cx, int cy, float base_r, float rot,
                         float build_time) {
    for (int i = 0; i < MAX_RINGS; i++) {
        const Ring *ring = &p->rings[i];
        if (ring->type == RING_NONE) break;          /* end of the ring list  */
        float prog = ring_build_progress(i, build_time);
        if (prog <= 0.0f) continue;                  /* reveal window not open */
        float r = ring->radius * base_r;
        if (r < 0.5f) continue;                      /* sub-cell — nothing to draw */
        switch (ring->type) {
            case RING_CIRCLE:     draw_circle    (cx, cy, r,                       prog); break;
            case RING_POLYGON:    draw_polygon   (cx, cy, r, ring->n,         rot, prog); break;
            case RING_STAR_POLY:  draw_star_poly (cx, cy, r, ring->n, ring->density, rot, prog); break;
            case RING_STAR_SHAPE: draw_star_shape(cx, cy, r, ring->n, ring->density, rot, prog); break;
            case RING_INTERLOCK:  draw_interlock (cx, cy, r, ring->n,         rot, prog); break;
            case RING_RAYS:       draw_rays      (cx, cy, r, ring->n,         rot, prog); break;
            default: break;
        }
    }
    if (p->centre_dot && build_time >= BINDU_AT) {
        paint_cell(cx, cy, '+', PAIR_CENTRE, A_BOLD);
    }
}

/* draw_hud — top yellow data line (fps / preset / theme / size / build-status /
 * rotation) right-aligned, and the bottom cyan key legend. */
static void draw_hud(const Scene *s, int rows, int cols) {
    const MandalaPreset *p = &PRESETS[s->preset_idx];

    /* build-progress string: "build 47%" assembling, "complete", or "PAUSED". */
    char build_str[24];
    if (s->build_complete) {
        snprintf(build_str, sizeof build_str, "complete");
    } else if (s->paused) {
        snprintf(build_str, sizeof build_str, "PAUSED  ");
    } else {
        float dur = preset_build_duration(p);
        int   pct = (int)(100.0f * s->build_time / dur);
        if (pct < 0)   pct = 0;
        if (pct > 99)  pct = 99;
        snprintf(build_str, sizeof build_str, "build %2d%%", pct);
    }

    char buf[160];
    snprintf(buf, sizeof buf,
             " %5.1f fps  preset %2d/%d: %-16s  theme: %-10s  size: %.2f  %s  rot: %s ",
             (double)s->fps,
             s->preset_idx + 1, N_PRESETS,
             PRESETS[s->preset_idx].name,
             THEME_NAME[s->theme_idx],
             (double)s->scale,
             build_str,
             s->rotation_on ? "ON " : "OFF");
    int hud_len = (int)strlen(buf);
    int hud_x   = cols - hud_len;
    if (hud_x < 0) hud_x = 0;
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hud_x, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    const char *hint =
        " q:quit  n/p:cycle  t:theme  +/-:size  r:rotate  space:pause  b:replay  0:reset ";
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0, "%s", hint);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const Scene *s) {
    erase();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    int cx = cols / 2, cy = rows / 2;

    float base_r = mandala_base_radius(rows, cols, s->scale);
    draw_mandala(&PRESETS[s->preset_idx], cx, cy, base_r, s->rot, s->build_time);
    draw_hud(s, rows, cols);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §7  INIT/RESET — scene defaults & build restart (NOT part of the tick)  */
/* ===================================================================== */

static void scene_restart_build(Scene *s) {
    s->build_time     = 0.0f;
    s->build_complete = false;
}

static void scene_reset(Scene *s) {
    s->preset_idx       = 0;
    s->theme_idx        = 0;
    s->scale            = SCALE_DEFAULT;
    s->rotation_on      = false;
    s->paused           = false;
    s->rot              = 0.0f;
    scene_restart_build(s);
    theme_apply(s->theme_idx);
}

static void scene_init(Scene *s) {
    s->fps               = 0.0f;
    s->fps_window_start  = clock_ns();
    s->frames_in_window  = 0;
    scene_reset(s);
}

/* ===================================================================== */
/* §8  EVENTS — keys, signals, screen setup (mutate state, NOT the tick)   */
/* ===================================================================== */

static void scene_input(Scene *s, int ch) {
    switch (ch) {
        case 'n': case KEY_RIGHT:
            s->preset_idx = (s->preset_idx + 1) % N_PRESETS;
            scene_restart_build(s);
            break;
        case 'p': case KEY_LEFT:
            s->preset_idx = (s->preset_idx + N_PRESETS - 1) % N_PRESETS;
            scene_restart_build(s);
            break;
        case 't': case 'T':
            s->theme_idx = (s->theme_idx + 1) % N_THEMES;
            theme_apply(s->theme_idx);
            break;
        case '+': case '=':
            s->scale = fminf(SCALE_MAX, s->scale + SCALE_STEP); break;
        case '-':
            s->scale = fmaxf(SCALE_MIN, s->scale - SCALE_STEP); break;
        case 'r': case 'R':
            s->rotation_on = !s->rotation_on; break;
        case ' ':
            s->paused = !s->paused; break;
        case 'b': case 'B':
            scene_restart_build(s); break;
        case '0':
            scene_reset(s); break;
        default: break;
    }
}

static volatile sig_atomic_t g_running = 1;
static volatile sig_atomic_t g_resize  = 0;

static void on_signal(int sig) {
    if (sig == SIGWINCH) g_resize = 1;
    else                 g_running = 0;
}

static void screen_init(void) {
    setlocale(LC_ALL, "");
    initscr();
    noecho();
    cbreak();
    keypad(stdscr, true);
    nodelay(stdscr, true);
    curs_set(0);
    typeahead(-1);
    if (has_colors()) {
        start_color();
        color_init();
    }
}

static void screen_cleanup(void) {
    endwin();
}

/* ===================================================================== */
/* §9  app — the frame loop (the per-tick combine)                         */
/* ===================================================================== */

int main(void) {
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    screen_init();
    atexit(screen_cleanup);

    scene_init(&g_scene);

    int64_t prev_ns = clock_ns();
    const int64_t frame_ns = NS_PER_SEC / TARGET_FPS;

    while (g_running) {
        int64_t frame_start = clock_ns();
        float dt = (float)(frame_start - prev_ns) / 1e9f;
        if (dt > 0.1f) dt = 0.1f;
        prev_ns = frame_start;

        if (g_resize) {
            g_resize = 0;
            endwin();
            refresh();
        }

        int ch = getch();
        while (ch != ERR) {
            if (ch == 'q' || ch == 'Q' || ch == 27) {
                g_running = 0;
                break;
            }
            scene_input(&g_scene, ch);
            ch = getch();
        }

        scene_tick(&g_scene, dt);
        scene_draw(&g_scene);

        g_scene.frames_in_window++;
        int64_t since = frame_start - g_scene.fps_window_start;
        if (since > (int64_t)FPS_UPDATE_MS * 1000000LL) {
            g_scene.fps = (float)g_scene.frames_in_window * 1e9f / (float)since;
            g_scene.fps_window_start = frame_start;
            g_scene.frames_in_window = 0;
        }

        int64_t spent = clock_ns() - frame_start;
        if (spent < frame_ns) clock_sleep_ns(frame_ns - spent);
    }

    return 0;
}
