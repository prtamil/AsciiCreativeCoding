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
 *       The lesson: "twenty different mandalas" is really "one
 *       parametric draw function with twenty parameter sets."
 *
 * Phase-1 size note: this file is ~780 lines, above the 250-450
 * typical phase-1 budget.  The over-spend buys 30 named preset
 * entries (one line each), six primitive draw functions, and a
 * progressive build animation (each ring + its features fill in
 * over time).  No single section is doing too much.
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
 *                 a shared centre.  Twenty PRESETS pick different
 *                 combinations to produce twenty named mandalas.
 *
 * Data-structure: PRESETS[20] of MandalaPreset = (name, up to 6 Rings,
 *                 bindu flag).  Each Ring = (type, count, density,
 *                 relative_radius).  No heap allocation.  The preset
 *                 table is a single C array literal — twenty lines,
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
 *   Khanna, "Yantra: The Tantric Symbol of Cosmic Unity" (1979) —
 *     symbolic + geometric analysis of Hindu yantras and mandalas.
 *   Tucci, "The Theory and Practice of the Mandala" (1961) —
 *     scholarly treatment of mandala geometry.
 *   Wikipedia, "Yantra", "Sri Yantra", "Mandala" — accessible
 *     introductions to the geometric motifs simulated here.
 *     https://en.wikipedia.org/wiki/Yantra
 *   Coxeter, "Regular Polytopes" (1973) — star polygons (n/d
 *     notation), gcd-based enumeration of stars vs polygons.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A mandala is a SYMMETRIC RADIAL DESIGN built from a small alphabet
 * of primitives — petals, rings, star polygons, dotted circles, rays.
 * Twenty named Hindu mandalas turn out to be twenty parameter
 * combinations of THE SAME draw function.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a cake decorator's wheel.  They have a fixed kit of stamps
 * (8-petal lotus, 16-petal lotus, hexagram, 8-pointed star, ray
 * spokes, dotted ring, square frame) and they LAYER 2-5 stamps at
 * different radii to produce a specific named mandala.  Two stamps
 * + bindu = simple Padma 8.  Five stamps = Sri Yantra.  Same kit;
 * different recipe.
 *
 * ALGORITHM IN STEPS  (per frame)
 * ───────────────────────────────
 *   1. Find screen centre (cx, cy) and base radius from
 *      min(cols/2, rows/(2·ASPECT)) · scale.
 *   2. For each ring in the preset (until RING_NONE sentinel):
 *      a. absolute radius r = ring.radius · base_r
 *      b. dispatch on ring.type to draw_circle / draw_dots /
 *         draw_petals / draw_star_polygon (handles polygon &
 *         star) / draw_rays.
 *   3. Paint the bindu '@' at (cx, cy) on top.
 *   4. HUD row + hint strip.
 *
 * KEY FORMULAS
 * ────────────
 *   Polar→cell:
 *     col = cx + r·cos θ
 *     row = cy + r·sin θ · ASPECT      (ASPECT ≈ 0.5)
 *
 *   Star polygon (n vertices, density d):
 *     for i in 0..n-1: line from vertex[i] to vertex[(i+d) mod n]
 *     gcd(n, d) = 1 → single connected star
 *     gcd(n, d) > 1 → multi-graph (e.g. 6/2 hexagram = 2 triangles)
 *
 *   Petals: angular position θ_k = rot + 2π·k/N
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Star density d=1 → polygon; d=2 with even n → multi-graph
 *     (hexagram = 6/2 → 2 disjoint triangles).  Both visually
 *     useful; we accept either as valid mandala geometry.
 *   • Aspect: terminal cells are roughly 2:1 tall.  ASPECT = 0.5
 *     applied to sin keeps circles ROUND on screen, not egg-
 *     shaped.
 *   • Small radii: at very small r (zoomed out, or innermost
 *     rings), petals collapse onto each other.  We clip rings
 *     with absolute radius < 0.5 cells to avoid centre noise.
 *   • Resize: SIGWINCH triggers a re-init.  All geometry re-
 *     derives from new (cols, rows) automatically.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Press n through all 20 presets — each visually distinct
 *     from its neighbours; names match the displayed geometry.
 *   • Press 'r' to start rotation — every primitive rotates as
 *     one rigid unit (the bindu and circles, both rotation-
 *     symmetric, look unchanged).
 *   • Press '+' or '-' — design grows / shrinks while staying
 *     centred; no clipping at small scale.
 *   • Press 't' through all 4 themes — palette changes; every
 *     ring type stays legible against default-black even with
 *     A_DIM (per project theme-brightness rule).
 *   • Sri Yantra preset (#13): visible interlocking star pattern
 *     at centre, 8-petal lotus around it, square frame outside.
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

/* §1.1 Ring + Preset types ------------------------------------------- */

typedef enum {
    RING_NONE = 0, RING_CIRCLE, RING_DOTS, RING_PETALS,
    RING_POLYGON, RING_STAR, RING_RAYS,
} RingType;

typedef struct {
    int   type;     /* RingType */
    int   n;        /* feature count (or 0 for RING_CIRCLE) */
    int   density;  /* star polygon stride (>=2 = star, else polygon) */
    float radius;   /* relative to base_r ∈ (0, 1] */
} Ring;

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

/* ===================================================================== */
/* §4  polar — polar↔cell mapping with terminal aspect correction         */
/* ===================================================================== */

static inline void polar_to_cell(int cx, int cy, float r, float theta,
                                 int *col, int *row) {
    *col = cx + (int)roundf(r * cosf(theta));
    *row = cy + (int)roundf(r * sinf(theta) * ASPECT);
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

/* ===================================================================== */
/* §5  mandala — primitives + dispatch                                    */
/* ===================================================================== */

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

/* §5.2 dots — N evenly-spaced 'o' glyphs */
static void draw_dots(int cx, int cy, float r, int n, float rot, float progress) {
    if (n < 1 || progress <= 0.0f) return;
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t = rot + (float)i / (float)n * 2.0f * (float)M_PI;
        int col, row;
        polar_to_cell(cx, cy, r, t, &col, &row);
        paint_cell(col, row, 'o', PAIR_DOTS, A_BOLD);
    }
}

/* §5.3 petals — N petal-clusters: bold '*' centre + dim '.' flanks */
static void draw_petals(int cx, int cy, float r, int n, float rot, float progress) {
    if (n < 1 || progress <= 0.0f) return;
    int n_done = progress_to_count(n, progress);
    for (int i = 0; i < n_done; i++) {
        float t = rot + (float)i / (float)n * 2.0f * (float)M_PI;
        int col, row;
        /* radial flanks read as a small petal cluster */
        polar_to_cell(cx, cy, r * 0.85f, t, &col, &row);
        paint_cell(col, row, '.', PAIR_PETALS, A_DIM);
        polar_to_cell(cx, cy, r * 1.15f, t, &col, &row);
        paint_cell(col, row, '.', PAIR_PETALS, A_DIM);
        /* central glyph painted last so it dominates the cluster */
        polar_to_cell(cx, cy, r, t, &col, &row);
        paint_cell(col, row, '*', PAIR_PETALS, A_BOLD);
    }
}

/* §5.4 line walker — paint cells along a straight line in cell coords */
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

/* §5.5 star_polygon — handles RING_POLYGON (d<=1) and RING_STAR (d>=2).
 * Connects vertex[i] to vertex[(i + d) mod n] for every i.  When
 * gcd(n, d) > 1, the result is a MULTI-GRAPH (e.g. 6/2 = two triangles
 * = hexagram) — visually correct for that case. */
static void draw_star_polygon(int cx, int cy, float r, int n, int density,
                              float rot, int pair, float progress) {
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
        draw_line(x1, y1, x2, y2, pair, A_BOLD);
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

/* §5.8 dispatcher — render rings up to current build_time, bindu on top.
 * Each ring has a time window [BINDU_AT + i·RING_BUILD_DUR,
 * BINDU_AT + (i+1)·RING_BUILD_DUR]; within that window features fill in. */
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
    float   build_time;       /* seconds since current preset started */
    bool    build_complete;   /* true once build_time >= preset's duration */
    float   fps;
    int64_t fps_window_start;
    int     frames_in_window;
} Scene;

static Scene g_scene;

/* Restart the build animation for the current preset. */
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
            scene_restart_build(); break;     /* replay build animation */
        case '0':
            scene_reset(); break;
        default: break;
    }
}

static void scene_tick(float dt) {
    if (g_scene.paused) return;

    /* Advance build_time until the preset is complete; clamp on completion. */
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

    /* base radius bounded by both axes (with HUD reservation). */
    float max_r_x = (float)(cols - 4) * 0.5f;
    float max_r_y = (float)(rows - 4) * 0.5f / ASPECT;
    float base_r  = fminf(max_r_x, max_r_y) * g_scene.scale;
    if (base_r < 2.0f) base_r = 2.0f;

    const MandalaPreset *p = &PRESETS[g_scene.preset_idx];
    draw_mandala(p, cx, cy, base_r, g_scene.rot, g_scene.build_time);

    /* Build-progress string: "build  47%" while assembling, "complete"
     * once finished, "paused" if held mid-build. */
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

    /* HUD top-right (yellow A_BOLD) */
    char buf[160];
    snprintf(buf, sizeof buf,
             " %5.1f fps  preset %2d/%d: %-12s  theme: %s  size: %.2f  %s  rot: %s ",
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

    /* HUD bottom (cyan A_BOLD) */
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
