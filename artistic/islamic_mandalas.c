/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * islamic_mandalas.c — 30 parametric Islamic geometric patterns
 *                       (20 simple + 10 complex)
 *
 * DEMO: An Islamic geometric medallion fills the centre of the
 *       screen, built from radial primitives — outlined star shapes,
 *       interlocking polygons, line-density star polygons, regular
 *       polygons, rays, and circles.  Thirty PRESETS cycle through
 *       canonical Islamic forms in two tiers:
 *         • Simple (1-20)  — 1-4 rings: Khatim, Hexagram, Octagram,
 *           Decagram, Dodecagram, Rub-el-hizb, Persian / Iznik /
 *           Selçuk / Mamluk / Andalusian / Maghrebi / Alhambra
 *           medallions, and sun/compass patterns.
 *         • Complex (21-30) — 5-7 rings: Topkapi Scroll, Konya
 *           Rosette, Damascus Dome, Cordoba Mihrab, Isfahan Garden,
 *           Cairo Stellate, Granada Constellation, Bursa Mosque,
 *           Marrakesh Tile, Quasi-Crystal 10.
 *       Each preset selects different combinations of the same six
 *       primitive shapes at different radii.
 *
 *       Each preset starts blank; rings build in order over time
 *       with their features (edges / spokes) appearing one-by-one.
 *       Once complete, the pattern stays drawn until you cycle.
 *
 *       Press n / p (or arrow keys) to cycle presets; t cycles
 *       colour themes; +/- resizes; r toggles slow rotation; space
 *       pauses; b replays the build animation.
 *
 *       Companion file: artistic/hindu_mandalas.c — same parametric
 *       approach with a different geometric tradition (radial petals
 *       + bindu instead of interlocking stars).
 *
 * Phase-1 size note: this file is ~860 lines, above the 250-450
 * typical phase-1 budget.  The over-spend buys 30 named preset
 * entries (one line each), six primitive draw functions (including
 * two Islamic-specific ones — outlined star shapes with sharp
 * points, and interlocking double polygons), and a progressive
 * build animation.  No single section is doing too much.
 *
 * Section map:
 *   §1 config   — preset table + rendering constants + themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — themed palettes + HUD/HINT pairs
 *   §4 polar    — polar↔cell mapping with aspect correction
 *   §5 mandala  — Ring + Preset types, primitive draws, draw_mandala
 *   §6 scene    — Scene state, input, tick, draw
 *   §7 screen   — ncurses init / present
 *   §8 app      — signals, resize, main loop
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
 *                 Twenty PRESETS pick different combinations to
 *                 produce twenty named geometric patterns.
 *
 * Data-structure: PRESETS[20] of MandalaPreset = (name, up to 8
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
 *   Critchlow, "Islamic Patterns: An Analytical and Cosmological
 *     Approach" (1976) — the canonical introduction to Islamic
 *     geometric construction.
 *   Bourgoin, "Arabic Geometrical Pattern and Design" (Dover
 *     reprint, 1973) — 200+ classical patterns analysed.
 *   Wikipedia, "Islamic geometric patterns", "Star polygon",
 *     "Girih tiles".  https://en.wikipedia.org/wiki/Islamic_geometric_patterns
 *   Coxeter, "Regular Polytopes" (1973) — star polygon n/d
 *     notation, gcd-based enumeration.
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
 * radial rays.  Twenty named patterns turn out to be twenty
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
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Inner ratio bounds: ρ < 0.10 collapses inner vertices onto
 *     centre; ρ > 0.90 makes the star almost a regular polygon.
 *     Clamped to [0.10, 0.95] in draw_star_shape.
 *   • Float roundoff at build completion: progress can compute
 *     to 0.99999... instead of exactly 1.0, dropping the last
 *     feature.  progress_to_count tolerates ≥ 0.999 → return n.
 *   • Aspect ratio: ASPECT = 0.5 applied to sin keeps the
 *     pattern circular on screen (terminal cells ~2:1 tall).
 *   • Resize: SIGWINCH triggers re-init; geometry rebuilds.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Press n through all 20 presets — each visually distinct.
 *   • Khatim (#1): two squares at 45° offset = 8-pointed star.
 *   • Hexagram (#2): two triangles at 60° offset = Solomon's seal.
 *   • Outlined 8-Star (#8): sharp 8-pointed outline (NOT the same
 *     as Khatim — Khatim is two overlapping squares; this is one
 *     star shape).
 *   • Press 'r' — design rotates as a rigid unit; all primitives
 *     respect the global rot offset.
 *   • Sri Yantra-style multi-ring presets (Persian 8-Fold,
 *     Iznik Medallion) build over ~2 seconds, ring by ring.
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
    HUD_COLS      = 96,
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

/* §1.1 Ring + Preset types ------------------------------------------- */

typedef enum {
    RING_NONE = 0, RING_CIRCLE, RING_POLYGON, RING_STAR_POLY,
    RING_STAR_SHAPE, RING_INTERLOCK, RING_RAYS,
} RingType;

typedef struct {
    int   type;     /* RingType */
    int   n;        /* feature count (or 0 for RING_CIRCLE) */
    int   density;  /* STAR_POLY: skip stride; STAR_SHAPE: inner_pct (0..100) */
    float radius;   /* relative to base_r ∈ (0, 1] */
} Ring;

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
/* §2  clock                                                               */
/* ===================================================================== */

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
/* §3  color                                                               */
/* ===================================================================== */

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

/* ===================================================================== */
/* §4  polar — polar↔cell mapping with terminal aspect correction         */
/* ===================================================================== */

static inline void polar_to_cell(int cx, int cy, float r, float theta,
                                 int *col, int *row) {
    *col = cx + (int)roundf(r * cosf(theta));
    *row = cy + (int)roundf(r * sinf(theta) * ASPECT);
}

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

/* ===================================================================== */
/* §5  mandala — primitives + dispatch                                    */
/* ===================================================================== */

static void paint_cell(int col, int row, char ch, int pair, int attr) {
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (col < 0 || col >= cols)        return;
    if (row < 1 || row >= rows - 1)    return;
    attron (COLOR_PAIR(pair) | attr);
    mvaddch(row, col, (chtype)(unsigned char)ch);
    attroff(COLOR_PAIR(pair) | attr);
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

/* §5.1 circle — many '.' samples around a continuous ring */
static void draw_circle(int cx, int cy, float r, float progress) {
    if (progress <= 0.0f || r < 0.5f) return;
    int n = (int)(2.0f * (float)M_PI * r * 1.4f);
    if (n < 24)  n = 24;
    if (n > 360) n = 360;
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t = (float)i / (float)n * 2.0f * (float)M_PI;
        int col, row;
        polar_to_cell(cx, cy, r, t, &col, &row);
        paint_cell(col, row, '.', PAIR_CIRCLE, A_NORMAL);
    }
}

/* §5.2 polygon — N corners connected by lines (regular n-gon) */
static void draw_polygon(int cx, int cy, float r, int n, float rot, float progress) {
    if (n < 3 || progress <= 0.0f) return;
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t1 = rot + (float)i             / (float)n * 2.0f * (float)M_PI;
        float t2 = rot + (float)((i + 1) % n) / (float)n * 2.0f * (float)M_PI;
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, r, t1, &x1, &y1);
        polar_to_cell(cx, cy, r, t2, &x2, &y2);
        draw_line(x1, y1, x2, y2, PAIR_POLYGON, A_BOLD);
    }
}

/* §5.3 star_poly — N vertices, density-d skip lines (octagram, etc.) */
static void draw_star_poly(int cx, int cy, float r, int n, int density,
                           float rot, float progress) {
    if (n < 3 || progress <= 0.0f) return;
    int d = (density < 2) ? 1 : density;
    if (d >= n) d = 1;
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t1 = rot + (float)i             / (float)n * 2.0f * (float)M_PI;
        float t2 = rot + (float)((i + d) % n) / (float)n * 2.0f * (float)M_PI;
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, r, t1, &x1, &y1);
        polar_to_cell(cx, cy, r, t2, &x2, &y2);
        draw_line(x1, y1, x2, y2, PAIR_STAR_POLY, A_BOLD);
    }
}

/* §5.4 star_shape — TRUE outlined star: n outer + n inner vertices,
 *      2n edges total, alternating outer-inner.  inner_pct is the
 *      inner radius as a percentage of outer radius (0..100). */
static void draw_star_shape(int cx, int cy, float r, int n, int inner_pct,
                            float rot, float progress) {
    if (n < 3 || progress <= 0.0f) return;
    float ratio = (float)inner_pct / 100.0f;
    if (ratio < 0.10f) ratio = 0.10f;
    if (ratio > 0.95f) ratio = 0.95f;
    float r_in = r * ratio;
    int total_edges = 2 * n;
    int n_done = progress_to_count(total_edges, progress);

    for (int e = 0; e < n_done; e++) {
        int v1 = e;
        int v2 = (e + 1) % total_edges;
        bool v1_outer = (v1 % 2 == 0);
        bool v2_outer = (v2 % 2 == 0);
        float t1 = rot + (float)v1 / (float)total_edges * 2.0f * (float)M_PI;
        float t2 = rot + (float)v2 / (float)total_edges * 2.0f * (float)M_PI;
        float radius1 = v1_outer ? r : r_in;
        float radius2 = v2_outer ? r : r_in;
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, radius1, t1, &x1, &y1);
        polar_to_cell(cx, cy, radius2, t2, &x2, &y2);
        draw_line(x1, y1, x2, y2, PAIR_STAR_SHAPE, A_BOLD);
    }
}

/* §5.5 interlock — TWO regular n-gons rotated by π/n.  Total 2n
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
        float t1 = poly_rot + (float) edge              / (float)n * 2.0f * (float)M_PI;
        float t2 = poly_rot + (float)((edge + 1) % n)   / (float)n * 2.0f * (float)M_PI;
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, r, t1, &x1, &y1);
        polar_to_cell(cx, cy, r, t2, &x2, &y2);
        draw_line(x1, y1, x2, y2, PAIR_INTERLOCK, A_BOLD);
    }
}

/* §5.6 rays — N lines from inner gap to outer radius */
static void draw_rays(int cx, int cy, float r, int n, float rot, float progress) {
    if (n < 1 || progress <= 0.0f) return;
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t = rot + (float)i / (float)n * 2.0f * (float)M_PI;
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, r * 0.10f, t, &x1, &y1);
        polar_to_cell(cx, cy, r,         t, &x2, &y2);
        draw_line(x1, y1, x2, y2, PAIR_RAYS, A_BOLD);
    }
}

/* §5.7 ring count + total build duration for a preset */
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

/* §5.8 dispatcher — render rings up to current build_time, centre last */
static void draw_mandala(const MandalaPreset *p,
                         int cx, int cy, float base_r, float rot,
                         float build_time) {
    for (int i = 0; i < MAX_RINGS; i++) {
        const Ring *ring = &p->rings[i];
        if (ring->type == RING_NONE) break;
        float ring_start = BINDU_AT + (float)i * RING_BUILD_DUR;
        if (build_time <= ring_start) continue;
        float prog = (build_time - ring_start) / RING_BUILD_DUR;
        if (prog > 1.0f) prog = 1.0f;

        float r = ring->radius * base_r;
        if (r < 0.5f) continue;
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

/* ===================================================================== */
/* §6  scene                                                               */
/* ===================================================================== */

typedef struct {
    int     preset_idx;
    int     theme_idx;
    float   scale;
    bool    rotation_on;
    bool    paused;
    float   rot;
    float   build_time;
    bool    build_complete;
    float   fps;
    int64_t fps_window_start;
    int     frames_in_window;
} Scene;

static Scene g_scene;

static void scene_restart_build(void) {
    g_scene.build_time     = 0.0f;
    g_scene.build_complete = false;
}

static void scene_reset(void) {
    g_scene.preset_idx       = 0;
    g_scene.theme_idx        = 0;
    g_scene.scale            = SCALE_DEFAULT;
    g_scene.rotation_on      = false;
    g_scene.paused           = false;
    g_scene.rot              = 0.0f;
    scene_restart_build();
    theme_apply(g_scene.theme_idx);
}

static void scene_init(void) {
    g_scene.fps               = 0.0f;
    g_scene.fps_window_start  = clock_ns();
    g_scene.frames_in_window  = 0;
    scene_reset();
}

static void scene_input(int ch) {
    switch (ch) {
        case 'n': case KEY_RIGHT:
            g_scene.preset_idx = (g_scene.preset_idx + 1) % N_PRESETS;
            scene_restart_build();
            break;
        case 'p': case KEY_LEFT:
            g_scene.preset_idx = (g_scene.preset_idx + N_PRESETS - 1) % N_PRESETS;
            scene_restart_build();
            break;
        case 't': case 'T':
            g_scene.theme_idx = (g_scene.theme_idx + 1) % N_THEMES;
            theme_apply(g_scene.theme_idx);
            break;
        case '+': case '=':
            g_scene.scale = fminf(SCALE_MAX, g_scene.scale + SCALE_STEP); break;
        case '-':
            g_scene.scale = fmaxf(SCALE_MIN, g_scene.scale - SCALE_STEP); break;
        case 'r': case 'R':
            g_scene.rotation_on = !g_scene.rotation_on; break;
        case ' ':
            g_scene.paused = !g_scene.paused; break;
        case 'b': case 'B':
            scene_restart_build(); break;
        case '0':
            scene_reset(); break;
        default: break;
    }
}

static void scene_tick(float dt) {
    if (g_scene.paused) return;

    if (!g_scene.build_complete) {
        const MandalaPreset *p = &PRESETS[g_scene.preset_idx];
        float dur = preset_build_duration(p);
        g_scene.build_time += dt;
        if (g_scene.build_time >= dur) {
            g_scene.build_time     = dur;
            g_scene.build_complete = true;
        }
    }

    if (g_scene.rotation_on) {
        g_scene.rot += ROT_RATE * dt;
        const float TWO_PI = 2.0f * (float)M_PI;
        while (g_scene.rot >  TWO_PI) g_scene.rot -= TWO_PI;
        while (g_scene.rot < -TWO_PI) g_scene.rot += TWO_PI;
    }
}

static void scene_draw(void) {
    erase();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    int cx = cols / 2;
    int cy = rows / 2;

    float max_r_x = (float)(cols - 4) * 0.5f;
    float max_r_y = (float)(rows - 4) * 0.5f / ASPECT;
    float base_r  = fminf(max_r_x, max_r_y) * g_scene.scale;
    if (base_r < 2.0f) base_r = 2.0f;

    const MandalaPreset *p = &PRESETS[g_scene.preset_idx];
    draw_mandala(p, cx, cy, base_r, g_scene.rot, g_scene.build_time);

    char build_str[24];
    if (g_scene.build_complete) {
        snprintf(build_str, sizeof build_str, "complete");
    } else if (g_scene.paused) {
        snprintf(build_str, sizeof build_str, "PAUSED  ");
    } else {
        float dur = preset_build_duration(p);
        int   pct = (int)(100.0f * g_scene.build_time / dur);
        if (pct < 0)   pct = 0;
        if (pct > 99)  pct = 99;
        snprintf(build_str, sizeof build_str, "build %2d%%", pct);
    }

    char buf[160];
    snprintf(buf, sizeof buf,
             " %5.1f fps  preset %2d/%d: %-16s  theme: %-10s  size: %.2f  %s  rot: %s ",
             (double)g_scene.fps,
             g_scene.preset_idx + 1, N_PRESETS,
             PRESETS[g_scene.preset_idx].name,
             THEME_NAME[g_scene.theme_idx],
             (double)g_scene.scale,
             build_str,
             g_scene.rotation_on ? "ON " : "OFF");
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

    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §7  screen                                                              */
/* ===================================================================== */

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
/* §8  app                                                                 */
/* ===================================================================== */

int main(void) {
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    screen_init();
    atexit(screen_cleanup);

    scene_init();

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
            scene_input(ch);
            ch = getch();
        }

        scene_tick(dt);
        scene_draw();

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
