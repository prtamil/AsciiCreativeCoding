/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * hindu_mandalas.c — 30 parametric Hindu mandalas (20 simple + 10 complex)
 *
 * DEMO: A Hindu mandala fills the centre of the screen, built from
 *       radial primitives — petals, rings, star polygons, polygons,
 *       rays, dots, and a central bindu.  Thirty PRESETS cycle
 *       through canonical forms in two tiers:
 *         • Simple (1-20)  — 1-4 rings each: Bindu, Padma 8/12/16/32,
 *           Shatkona, Ashtakona, Sri Yantra, Lakshmi, Ganesh, Surya,
 *           Anahata, Rudra, ...
 *         • Complex (21-30) — 5-7 rings each: Sahasrara, Maha Yantra,
 *           Kalachakra, Mahakali, Bhairava, Sudarshana, Mahamrityunjaya,
 *           Mahalakshmi, Vajra, Mahaganesha.
 *       Each preset selects different combinations of the same six
 *       primitive shapes at different radii.
 *
 *       Press n / p (or arrow keys) to cycle presets; t cycles
 *       colour themes; +/- resizes; r toggles slow rotation; space
 *       pauses.  Press 0 to reset.
 *
 *       The lesson: "thirty different mandalas" is really "one
 *       parametric draw function with thirty parameter sets."
 *
 * Size note: this file is ~890 lines, well above the 250-450 typical
 * phase-1 budget.  The over-spend buys 30 named preset entries (one
 * line each), six primitive draw functions, a progressive build
 * animation, and the layered/documented structure of the later
 * refactor passes.  No single section is doing too much.
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
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/hindu_mandalas.c \
 *       -o hindu_mandalas -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Parametric mandala synthesis.  Each "mandala" is a
 *                 list of RINGS, where each ring is one of six radial
 *                 primitives:
 *                   CIRCLE   continuous dotted circle ('.')
 *                   DOTS     N evenly-spaced 'o' glyphs
 *                   PETALS   N petal-clusters (bold '*' + flank dots)
 *                   POLYGON  N vertices connected by line segments
 *                   STAR     N vertices connected by skip-density lines
 *                   RAYS     N lines from inner radius to outer radius
 *                 Plus an optional BINDU (centre '@' dot, painted last
 *                 so it always wins the centre cell).  Rendering a
 *                 mandala iterates the ring list and paints each
 *                 primitive at its specified relative radius around
 *                 a shared centre.  Thirty PRESETS pick different
 *                 combinations to produce thirty named mandalas.
 *
 * Data-structure: PRESETS[30] of MandalaPreset = (name, up to MAX_RINGS
 *                 Rings, bindu flag).  Each Ring = (type, count, density,
 *                 relative_radius).  No heap allocation.  The preset
 *                 table is a single C array literal — thirty lines,
 *                 one per preset, defined via a compact R(...) macro.
 *
 * Rendering     : Polar→cell mapping with terminal aspect correction
 *                 (sin component scaled by ASPECT = 0.5 so circles
 *                 render round, not vertically-stretched).  Lines
 *                 between two polar points are walked cell-by-cell;
 *                 the line glyph (- | / \) is chosen from the
 *                 direction vector.  Per-ring-type colour pair, all
 *                 themed by a 4-theme palette (Saffron / Ocean /
 *                 Forest / Cosmic).
 *
 * Performance   : O(rings · features · cells_per_feature) per frame.
 *                 At 6 rings × 32 features × ~12 cells per line =
 *                 ~2300 paints per frame, microseconds.
 *
 * References    :
 *   Mandala & yantra symbolism / geometry (the preset forms):
 *     Khanna, "Yantra: The Tantric Symbol of Cosmic Unity" (1979) —
 *       symbolic + geometric analysis of Hindu yantras and mandalas.
 *     Tucci, "The Theory and Practice of the Mandala" (1961) —
 *       scholarly treatment of mandala geometry.
 *     Kulaichev, "Sriyantra and its mathematical properties" (Indian J.
 *       Hist. Sci. 1984) — the precise interlocking-triangle construction
 *       behind the Sri Yantra preset.
 *     Wikipedia, "Yantra", "Sri Yantra", "Mandala" — accessible
 *       introductions to the geometric motifs simulated here.
 *       https://en.wikipedia.org/wiki/Yantra
 *
 *   Geometry & rendering (primitives, line rasterization, polar layout):
 *     Coxeter, "Regular Polytopes" (1973) — star polygons {n/d}, the
 *       gcd-based split of STAR vs POLYGON rings.
 *     Bresenham, "Algorithm for computer control of a digital plotter"
 *       (IBM Syst. J. 1965) — the integer line walk behind the cell-by-cell
 *       segment drawing (POLYGON / STAR / RAYS rings).
 *     Foley, van Dam, Feiner & Hughes, "Computer Graphics: Principles and
 *       Practice" — polar / parametric primitives and 2-D raster line
 *       drawing behind §3's polar→cell mapping and the §6 ring draws.
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
 * SIMULATION is THIN here: the mandala is a STATIC parametric drawing, so the
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

    MAX_RINGS = 8,    /* enough for complex presets (Maha Yantra uses 7) */
    N_PRESETS = 30,
    N_THEMES  = 4,

    /* Color pair IDs */
    PAIR_BINDU   = 1,
    PAIR_CIRCLE  = 2,
    PAIR_DOTS    = 3,
    PAIR_PETALS  = 4,
    PAIR_POLYGON = 5,
    PAIR_STAR    = 6,
    PAIR_RAYS    = 7,
    PAIR_HUD     = 8,
    PAIR_HINT    = 9,
};

#define ASPECT        0.5f       /* terminal cells ~2:1 tall:wide */
#define ROT_RATE      0.20f      /* rad/sec when rotation enabled */
#define SCALE_MIN     0.40f
#define SCALE_MAX     1.00f
#define SCALE_DEFAULT 0.85f
#define SCALE_STEP    0.05f
#define NS_PER_SEC    1000000000LL

/* Build animation: bindu appears at BINDU_AT seconds; each ring builds
 * over RING_BUILD_DUR seconds with features (petals / star edges / rays
 * / circle samples) appearing in order. After the last ring's window,
 * build_complete = true and the mandala stays drawn forever. */
#define BINDU_AT       0.10f
#define RING_BUILD_DUR 0.55f

/* Primitive geometry */
#define CIRCLE_OVERSAMPLE  1.4f   /* '.' samples per cell of circle circumference */
#define CIRCLE_SAMPLES_MIN 24     /* clamp so small circles still read as a ring  */
#define CIRCLE_SAMPLES_MAX 360    /* and large ones don't oversample pointlessly  */
#define PETAL_FLANK_LO     0.85f  /* inner flank dot of a petal cluster (× ring r) */
#define PETAL_FLANK_HI     1.15f  /* outer flank dot of a petal cluster (× ring r) */
#define RAY_INNER_FRAC     0.10f  /* rays start this far out → a centre gap        */

/* Layout */
#define SCREEN_MARGIN      4      /* cells kept clear around the mandala (HUD rows) */
#define MANDALA_MIN_R      2.0f   /* never shrink the mandala below this radius     */

/* §1.1 Ring + Preset types ------------------------------------------- */

/* RingType — the six RADIAL PRIMITIVES every mandala in this file is built from.
 * A mandala is a concentric stack of rings, each one of these laid out around a
 * shared centre; combining a handful at different radii reproduces the canonical
 * yantra/mandala motifs (Khanna; Tucci). RING_NONE is the array SENTINEL — the
 * first one ends a preset's ring list.
 *   RING_NONE    : end-of-list marker (value 0 so {0}-init terminates a preset)
 *   RING_CIRCLE  : a continuous dotted ring ('.')
 *   RING_DOTS    : N evenly-spaced 'o' beads
 *   RING_PETALS  : N petal clusters (lotus / padma)
 *   RING_POLYGON : N vertices joined edge-to-edge (a regular n-gon)
 *   RING_STAR    : N vertices joined with a skip stride (a star polygon {n/d})
 *   RING_RAYS    : N spokes from an inner gap to the rim (a chakra) */
typedef enum {
    RING_NONE = 0, RING_CIRCLE, RING_DOTS, RING_PETALS,
    RING_POLYGON, RING_STAR, RING_RAYS,
} RingType;

/* Ring — one concentric layer of a mandala: a single RingType primitive drawn N
 * times around the centre at a relative radius. WHY four fields and no more: a
 * ring is fully specified by WHICH primitive, HOW MANY of it, the star skip, and
 * WHERE (radius) — colour and glyph follow from the type. The (n, density) pair
 * is the star-polygon {n/d} notation (Coxeter, "Regular Polytopes"): density is
 * the stride joining vertex i to vertex (i+d) mod n, so d=1 is a plain polygon
 * and d>=2 is a star — e.g. {6/2} is a hexagram (two overlaid triangles).
 *   type    : a RingType (stored as int; an array of these ends at RING_NONE).
 *   n       : feature count — beads / petals / vertices / rays (0 for CIRCLE,
 *             whose sample count is derived from its radius at draw time).
 *   density : star-polygon stride d; <2 draws a polygon, >=2 draws a star.
 *   radius  : radius as a FRACTION of the mandala's base radius, in (0, 1]. */
typedef struct {
    int   type;     /* RingType */
    int   n;        /* feature count (or 0 for RING_CIRCLE) */
    int   density;  /* star polygon stride (>=2 = star, else polygon) */
    float radius;   /* relative to base_r ∈ (0, 1] */
} Ring;

/* MandalaPreset — one named mandala as a RECIPE: a centre flag plus an ordered
 * list of Rings. This is the file's central lesson — "thirty different mandalas"
 * is really ONE parametric draw function fed thirty parameter sets. The named
 * forms (Sri Yantra, Sahasrara, Padma, ...) are canonical yantras/mandalas
 * (Khanna; Tucci; Kulaichev for the Sri Yantra); each is reproduced by choosing
 * rings at chosen radii. The rings[] order is innermost-to-outermost and also
 * drives the build-in ANIMATION — ring i reveals during its time window.
 *   name   : label shown in the HUD.
 *   rings  : up to MAX_RINGS layers; the list ends at the first RING_NONE entry,
 *            so short presets {0}-pad the remainder.
 *   bindu  : draw the central '@' point — the BINDU, the seed-point of the
 *            mandala — painted last so it always wins the centre cell. */
typedef struct {
    const char *name;
    Ring rings[MAX_RINGS];
    bool bindu;
} MandalaPreset;

/* Compact preset constructor — one ring on one line.
 * Trailing rings default-initialise to {0,0,0,0} = RING_NONE sentinel.
 *
 * Note: the macro parameters are CC/DD/RR (not n/d/r) because plain
 * `n` would collide with the struct field name `.n` and the
 * preprocessor would substitute it inside `.n = (n)`. */
#define R(t,CC,DD,RR) {.type = RING_##t, .n = (CC), .density = (DD), .radius = (RR)}

static const MandalaPreset PRESETS[N_PRESETS] = {
/*  ─────────── 20 named mandalas as parameter recipes ─────────────────── */
    {"Bindu",       {{0}}, true},
    {"Padma 8",     {R(CIRCLE,  0,0, 0.18f), R(PETALS,  8,0, 0.65f)},                                                    true},
    {"Padma 12",    {R(CIRCLE,  0,0, 0.18f), R(PETALS, 12,0, 0.65f)},                                                    true},
    {"Padma 16",    {R(PETALS,  8,0, 0.30f), R(PETALS, 16,0, 0.75f)},                                                    true},
    {"Padma 32",    {R(PETALS, 16,0, 0.45f), R(PETALS, 32,0, 0.85f)},                                                    true},
    {"Trikon",      {R(POLYGON, 3,0, 0.65f), R(CIRCLE,  0,0, 0.88f)},                                                    true},
    {"Shatkona",    {R(STAR,    6,2, 0.65f), R(CIRCLE,  0,0, 0.88f)},                                                    true},
    {"Ashtakona",   {R(STAR,    8,3, 0.65f), R(CIRCLE,  0,0, 0.88f)},                                                    true},
    {"Dwadashara",  {R(STAR,   12,5, 0.65f), R(CIRCLE,  0,0, 0.88f)},                                                    true},
    {"Chakra 8",    {R(RAYS,    8,0, 0.85f), R(CIRCLE,  0,0, 0.88f), R(CIRCLE, 0,0, 0.30f)},                             true},
    {"Chakra 16",   {R(RAYS,   16,0, 0.85f), R(CIRCLE,  0,0, 0.88f), R(CIRCLE, 0,0, 0.40f)},                             true},
    {"Bhupura",     {R(PETALS,  8,0, 0.55f), R(POLYGON, 4,0, 0.90f)},                                                    true},
    {"Sri Yantra",  {R(STAR,    9,4, 0.40f), R(STAR,    9,2, 0.55f), R(PETALS, 8,0, 0.72f), R(POLYGON, 4,0, 0.92f)},     true},
    {"Kali",        {R(STAR,    5,2, 0.45f), R(PETALS,  8,0, 0.70f), R(POLYGON, 4,0, 0.92f)},                            true},
    {"Lakshmi",     {R(STAR,    8,3, 0.45f), R(PETALS, 16,0, 0.75f), R(POLYGON, 4,0, 0.92f)},                            true},
    {"Saraswati",   {R(CIRCLE,  0,0, 0.20f), R(PETALS, 16,0, 0.65f), R(CIRCLE, 0,0, 0.90f)},                             true},
    {"Ganesh",      {R(PETALS,  8,0, 0.50f), R(DOTS,   24,0, 0.72f), R(POLYGON, 4,0, 0.92f)},                            true},
    {"Surya",       {R(RAYS,   12,0, 0.85f), R(CIRCLE,  0,0, 0.65f), R(PETALS, 12,0, 0.30f)},                            true},
    {"Anahata",     {R(STAR,    6,2, 0.40f), R(PETALS, 12,0, 0.75f), R(CIRCLE, 0,0, 0.90f)},                             true},
    {"Rudra",       {R(STAR,   11,4, 0.55f), R(RAYS,   11,0, 0.85f), R(CIRCLE, 0,0, 0.90f)},                             true},

/*  ─────────── 10 complex mandalas — 5-7 rings each ────────────────────── */
    {"Sahasrara",       {R(PETALS,  8,0, 0.20f), R(PETALS, 16,0, 0.36f), R(PETALS, 24,0, 0.52f), R(PETALS, 32,0, 0.68f), R(PETALS, 48,0, 0.83f), R(CIRCLE, 0,0, 0.92f)}, true},
    {"Maha Yantra",     {R(STAR,    9,4, 0.30f), R(STAR,    9,2, 0.42f), R(STAR,    8,3, 0.52f), R(STAR,    6,2, 0.62f), R(PETALS,  8,0, 0.74f), R(PETALS, 16,0, 0.84f), R(POLYGON, 4,0, 0.94f)}, true},
    {"Kalachakra",      {R(CIRCLE,  0,0, 0.18f), R(RAYS,   12,0, 0.50f), R(PETALS, 12,0, 0.55f), R(RAYS,   24,0, 0.78f), R(PETALS, 24,0, 0.83f), R(POLYGON, 4,0, 0.92f), R(CIRCLE,  0,0, 0.95f)}, true},
    {"Mahakali",        {R(STAR,    5,2, 0.30f), R(STAR,    6,2, 0.46f), R(PETALS,  8,0, 0.60f), R(PETALS, 12,0, 0.74f), R(RAYS,   16,0, 0.86f), R(CIRCLE,  0,0, 0.92f)}, true},
    {"Bhairava",        {R(STAR,    8,3, 0.30f), R(STAR,   16,7, 0.48f), R(RAYS,    8,0, 0.65f), R(PETALS, 24,0, 0.78f), R(CIRCLE,  0,0, 0.85f), R(POLYGON, 4,0, 0.92f)}, true},
    {"Sudarshana",      {R(RAYS,    8,0, 0.34f), R(PETALS,  8,0, 0.40f), R(RAYS,   16,0, 0.55f), R(PETALS, 16,0, 0.60f), R(RAYS,   24,0, 0.78f), R(CIRCLE,  0,0, 0.85f), R(CIRCLE,  0,0, 0.92f)}, true},
    {"Mahamrityunjaya", {R(STAR,    5,2, 0.28f), R(STAR,    6,2, 0.42f), R(PETALS,  8,0, 0.56f), R(PETALS, 12,0, 0.70f), R(PETALS, 16,0, 0.82f), R(POLYGON, 4,0, 0.92f)}, true},
    {"Mahalakshmi",     {R(STAR,    9,2, 0.26f), R(STAR,    8,3, 0.40f), R(PETALS,  8,0, 0.54f), R(PETALS, 16,0, 0.66f), R(PETALS, 24,0, 0.80f), R(POLYGON, 4,0, 0.90f), R(CIRCLE,  0,0, 0.94f)}, true},
    {"Vajra",           {R(RAYS,    4,0, 0.30f), R(RAYS,    8,0, 0.50f), R(RAYS,   16,0, 0.80f), R(PETALS, 12,0, 0.42f), R(PETALS, 24,0, 0.65f), R(POLYGON, 4,0, 0.92f), R(CIRCLE,  0,0, 0.95f)}, true},
    {"Mahaganesha",     {R(STAR,    6,2, 0.28f), R(PETALS,  8,0, 0.42f), R(PETALS, 16,0, 0.58f), R(PETALS, 32,0, 0.74f), R(RAYS,   24,0, 0.86f), R(POLYGON, 4,0, 0.92f), R(CIRCLE,  0,0, 0.95f)}, true},
};

/* §1.2 Themes — six ring-type colours per theme, plus bindu colour.
 * Indices into the 256-colour cube (or basic 8 if cube unavailable).  All
 * tier-bottom values stay above 24 to remain legible under A_DIM. */
static const int THEME_PALETTE[N_THEMES][6] = {
    /*           CIRCLE DOTS  PETALS POLY  STAR  RAYS                  */
    /* SAFFRON */ { 220,  214,  208,  202,  226,  220 }, /* warm gold/red */
    /* OCEAN   */ {  39,   45,   51,   33,  117,   75 }, /* cyans + blues */
    /* FOREST  */ {  34,   40,   46,  118,  154,   28 }, /* greens */
    /* COSMIC  */ { 165,  171,  201,  207,  213,  219 }, /* magenta + violet */
};
static const int   THEME_BINDU[N_THEMES] = { 230, 195, 195, 230 };
static const char *THEME_NAME [N_THEMES] = { "Saffron", "Ocean", "Forest", "Cosmic" };

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

/* Pick an ASCII line glyph that matches a (dx, dy) direction in cells. */
static char line_glyph(int dx, int dy) {
    /* Aspect-correct dy so '|' vs '-' classification matches visual angle. */
    float adx = (float)abs(dx);
    float ady = (float)abs(dy) / ASPECT;
    if (adx < 0.5f) return '|';
    if (ady < 0.5f) return '-';
    float r = ady / adx;
    if (r < 0.5f) return '-';
    if (r > 2.0f) return '|';
    /* Diagonals: in screen coords y points DOWN.  Same-sign dx,dy = '\'. */
    return ((dx > 0) == (dy > 0)) ? '\\' : '/';
}

/* progress_to_count — how many of `n` features to draw at this build
 * progress.  At any positive progress at least one feature is shown
 * (so each ring announces its arrival immediately).
 *
 * Tolerance: 0.999 instead of 1.0 because float arithmetic in the
 * dispatcher can produce prog = 0.999998... at build completion
 * (e.g. (4*0.55f - 3*0.55f) / 0.55f doesn't always yield exactly
 * 1.0).  Without the tolerance, the LAST RING's last feature was
 * silently dropped — most visibly the 4th edge of a POLYGON 4
 * frame, leaving the square open on one side. */
static int progress_to_count(int n, float progress) {
    if (progress >= 0.999f) return n;
    if (progress <= 0.0f)   return 0;
    int k = (int)(progress * (float)n);
    if (k < 1) k = 1;
    if (k > n) k = n;
    return k;
}

/* build-animation progress of ring i in [0,1]; <= 0 before its reveal window
 * opens. Ring i's window is [BINDU_AT + i·RING_BUILD_DUR, +RING_BUILD_DUR]. */
static float ring_build_progress(int i, float build_time) {
    float ring_start = BINDU_AT + (float)i * RING_BUILD_DUR;
    float prog = (build_time - ring_start) / RING_BUILD_DUR;
    if (prog > 1.0f) prog = 1.0f;
    return prog;
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

/* base mandala radius that fits the screen (minus the HUD margin), scaled by the
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
 *   WHAT   — preset_idx: which of the PRESETS[] mandalas is on screen.
 *   HOW    — the user-tunable knobs: scale (size), rotation_on (spin toggle).
 *   WHEN   — the cosmetic animation advanced per tick: build_time +
 *            build_complete (the ring-by-ring reveal) and rot (rotation angle);
 *            paused freezes both.
 *   RENDER — theme_idx: the selected colour-palette index.
 *   FPS    — a sliding-window frame-rate meter, shown in the HUD only. */
typedef struct {
    /* WHAT — the mandala on screen */
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
 * The ONLY per-tick state advance. The mandala is a static drawing, so the only
 * state that moves is cosmetic ANIMATION: the build-reveal timer (build_time /
 * build_complete) and the rotation angle (rot). Paused short-circuits both.    */

static void scene_tick(Scene *s, float dt) {
    if (s->paused) return;

    /* Advance build_time until the preset is complete; clamp on completion. */
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
    init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
    init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
}

static void theme_apply(int idx) {
    init_pair(PAIR_BINDU,   THEME_BINDU  [idx],     -1);
    init_pair(PAIR_CIRCLE,  THEME_PALETTE[idx][0],  -1);
    init_pair(PAIR_DOTS,    THEME_PALETTE[idx][1],  -1);
    init_pair(PAIR_PETALS,  THEME_PALETTE[idx][2],  -1);
    init_pair(PAIR_POLYGON, THEME_PALETTE[idx][3],  -1);
    init_pair(PAIR_STAR,    THEME_PALETTE[idx][4],  -1);
    init_pair(PAIR_RAYS,    THEME_PALETTE[idx][5],  -1);
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

/* dots — N evenly-spaced 'o' glyphs */
static void draw_dots(int cx, int cy, float r, int n, float rot, float progress) {
    if (n < 1 || progress <= 0.0f) return;
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t = feature_angle(i, n, rot);
        int col, row;
        polar_to_cell(cx, cy, r, t, &col, &row);
        paint_cell(col, row, 'o', PAIR_DOTS, A_BOLD);
    }
}

/* petals — N petal-clusters: bold '*' centre + dim '.' flanks */
static void draw_petals(int cx, int cy, float r, int n, float rot, float progress) {
    if (n < 1 || progress <= 0.0f) return;
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t = feature_angle(i, n, rot);
        int col, row;
        /* radial flanks read as a small petal cluster */
        polar_to_cell(cx, cy, r * PETAL_FLANK_LO, t, &col, &row);
        paint_cell(col, row, '.', PAIR_PETALS, A_DIM);
        polar_to_cell(cx, cy, r * PETAL_FLANK_HI, t, &col, &row);
        paint_cell(col, row, '.', PAIR_PETALS, A_DIM);
        /* central glyph painted last so it dominates the cluster */
        polar_to_cell(cx, cy, r, t, &col, &row);
        paint_cell(col, row, '*', PAIR_PETALS, A_BOLD);
    }
}

/* line walker — paint cells along a straight line in cell coords */
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

/* star_polygon — handles RING_POLYGON (d<=1) and RING_STAR (d>=2).
 * Connects vertex[i] to vertex[(i + d) mod n] for every i.  When
 * gcd(n, d) > 1, the result is a MULTI-GRAPH (e.g. 6/2 = two triangles
 * = hexagram) — visually correct for that case. */
static void draw_star_polygon(int cx, int cy, float r, int n, int density,
                              float rot, int pair, float progress) {
    if (n < 3 || progress <= 0.0f) return;
    int d = star_stride(density, n);
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t1 = feature_angle(i,           n, rot);
        float t2 = feature_angle((i + d) % n, n, rot);
        int x1, y1, x2, y2;
        polar_to_cell(cx, cy, r, t1, &x1, &y1);
        polar_to_cell(cx, cy, r, t2, &x2, &y2);
        draw_line(x1, y1, x2, y2, pair, A_BOLD);
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

/* dispatcher — render rings up to current build_time, bindu on top.
 * Each ring has a time window [BINDU_AT + i·RING_BUILD_DUR,
 * BINDU_AT + (i+1)·RING_BUILD_DUR]; within that window features fill in. */
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
            case RING_CIRCLE:  draw_circle      (cx, cy, r,                         prog); break;
            case RING_DOTS:    draw_dots        (cx, cy, r, ring->n,           rot, prog); break;
            case RING_PETALS:  draw_petals      (cx, cy, r, ring->n,           rot, prog); break;
            case RING_POLYGON: draw_star_polygon(cx, cy, r, ring->n, 1,        rot, PAIR_POLYGON, prog); break;
            case RING_STAR:    draw_star_polygon(cx, cy, r, ring->n, ring->density, rot, PAIR_STAR,    prog); break;
            case RING_RAYS:    draw_rays        (cx, cy, r, ring->n,           rot, prog); break;
            default: break;
        }
    }
    if (p->bindu && build_time >= BINDU_AT) {
        paint_cell(cx, cy, '@', PAIR_BINDU, A_BOLD);
    }
}

/* draw_hud — the two HUD bars: a top-right yellow data line (fps / preset /
 * theme / size / build-status / rotation) and a bottom cyan key legend. */
static void draw_hud(const Scene *s, int rows, int cols) {
    const MandalaPreset *p = &PRESETS[s->preset_idx];

    /* Build-progress string: "build  47%" while assembling, "complete"
     * once finished, "paused" if held mid-build. */
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

    /* HUD top-right (yellow A_BOLD) */
    char buf[160];
    snprintf(buf, sizeof buf,
             " %5.1f fps  preset %2d/%d: %-12s  theme: %s  size: %.2f  %s  rot: %s ",
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

    /* HUD bottom (cyan A_BOLD) */
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

/* Restart the build animation for the current preset. */
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
            scene_restart_build(s); break;     /* replay build animation */
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

        /* fps update ~every FPS_UPDATE_MS */
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
