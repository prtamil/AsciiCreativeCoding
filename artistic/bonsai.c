/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bonsai.c — bonsai tree gallery: 5 classical styles, 6 themes,
 *            static skeleton + subtle wind rustle on foliage.
 *
 * DEMO: A single bonsai tree fills the screen, drawn as a hand-tuned
 *       composition: pot at the bottom, trunk rising in a style-
 *       specific path (straight, S-curved, slanted, cascading, or
 *       sparse), recursive branches growing outward, foliage clouds
 *       at the branch tips, all framed by empty negative space in
 *       the Japanese aesthetic.  Foliage cells subtly re-randomise
 *       their glyph each frame to simulate leaves rustling in a
 *       breeze — the only animation; the skeleton itself is static.
 *
 *       Distinct from a "growing tree" demo (the previous bonsai.c
 *       grows live like cbonsai): this file shows the FINISHED tree
 *       as a hand-tuned still image you can study, with the wind
 *       rustle keeping it visually alive.  Better for "looks great
 *       to leave on screen" use; less educational about L-systems.
 *
 *       Styles (cycle with n / N):
 *         CHOKKAN  formal upright — straight vertical trunk
 *         MOYOGI   informal upright — S-curved trunk (most popular)
 *         SHAKAN   slanting — trunk leans 15-30° to one side
 *         KENGAI   cascade — trunk arcs over the pot rim
 *         BUNJIN   literati — tall thin sparse, lots of negative space
 *
 *       Themes (cycle with t / T):
 *         SPRING   fresh greens, light bark
 *         SUMMER   deep saturated greens
 *         AUTUMN   orange/red/copper leaves, dark bark
 *         WINTER   bare branches, snow tips, no foliage
 *         CHERRY   pink-blossom-heavy
 *         SUMI_E   black ink on white "paper" (inverted, Japanese
 *                  ink-painting aesthetic)
 *
 * Section map:
 *   §1 config    — sizes, theme + style tables
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-tier theme apply (with inverted-bg support)
 *   §4 random    — LCG + cheap hashes
 *   §5 tree      — recursive skeleton generation per style
 *   §6 raster    — char-grid line drawing + foliage cloud fill
 *   §7 pot       — pot frame rendering
 *   §8 scene     — generate + raster + render with wind rustle
 *   §9 screen + app
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume rustle
 *   n / N        next / previous style
 *   t / T        next / previous theme
 *   r            reseed (new tree variant of same style)
 *   ] / [        sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/bonsai.c \
 *       -o bonsai -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two-pass procedural composition.
 *
 *                  PASS 1 — SKELETON GENERATION:
 *                     Generate trunk path according to style:
 *                       CHOKKAN  : vertical with tiny noise wobble
 *                       MOYOGI   : sin-S-curve, 1-2 wavelengths over height
 *                       SHAKAN   : straight line at chosen lean angle
 *                       KENGAI   : up-then-arc-over-rim 2-phase path
 *                       BUNJIN   : nearly vertical, very tall, very thin
 *                     Recursively subdivide: at each trunk node, with
 *                     style-dependent probability, branch outward.
 *                     Each branch is itself recursively subdivided.
 *                     Stop when length < MIN_LEN or depth ≥ MAX_DEPTH;
 *                     terminal points become FOLIAGE CLOUDS (centre,
 *                     radius, density).
 *
 *                  PASS 2 — RASTERISE TO CHAR GRID:
 *                     For each trunk/branch segment, walk a Bresenham
 *                     line and stamp a glyph at each cell, choosing
 *                     trunk-thick (`M`, `H`) vs medium (`|`, `[`) vs
 *                     thin (`/`, `\`, `|`) based on segment thickness.
 *                     For each foliage cloud, fill an aspect-corrected
 *                     ELLIPSE (round in physical pixels) with random
 *                     leaf glyphs sampled from the theme's leaf set.
 *                     Pot drawn last over the bottom rows.
 *
 *                  PER FRAME:
 *                     Re-emit the char grid; for ~5% of FOLIAGE cells
 *                     (gated by per-cell rustle hash + global time),
 *                     re-pick a random leaf glyph.  This is the only
 *                     per-frame work — the skeleton never regenerates
 *                     unless the user reseeds (`r`) or switches style
 *                     (`n`).
 *
 * Data-structure : `Tree` struct holds segments[] (trunk + branches as
 *                  line segments with thickness) and foliage[] (clouds
 *                  with centre/radius/density).  Sized for the gnarliest
 *                  recursion: 256 segments + 64 clouds, ~3 KB total.
 *                  Plus per-render `char_grid[H][W]` and `pair_grid[H][W]`
 *                  for the rasterised image.
 *
 * Rendering      : ASCII only.  Trunk uses heavy bracket-style chars
 *                  (`M`, `H`, `(`, `)`, `[`, `]`); branches use slashes
 *                  (`/`, `\`); foliage uses density chars (`&`, `%`,
 *                  `#`, `*`, `@`, `^`, `o`).  Pot uses `_`, `|`, `:`,
 *                  `.`, `'` for box-drawing-like effect.
 *
 * Performance    : Skeleton generation: 256 segments × ~5 ops + 64
 *                  clouds × ~50 cells each = ~5 K ops, one-time.
 *                  Per-frame: cols·rows mvaddch + rustle hash check.
 *                  Trivial at any terminal size; the limit is ncurses
 *                  output throughput, not the algorithm.
 *
 * References     :
 *   • Naka, J. Y. — *Bonsai Techniques I & II* (1973, 1982).  The
 *     classical-style taxonomy (chokkan / moyogi / shakan / kengai /
 *     bunjin) we use comes from these foundational books.
 *   • Iwata, Y. — *Bonsai: A Patient Art* (2009).  Photographic
 *     reference for how trunks curve, branches space, foliage clumps.
 *   • Lindenmayer, A. — "Mathematical models for cellular interactions
 *     in development", *J. Theor. Biol.* 18 (1968).  L-system origin;
 *     we use a simplified non-L-system recursive descent here, but the
 *     branching pattern follows L-system tradition.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Build a TREE SKELETON once (line segments + foliage clouds), draw it
 * to a character grid once, then render that grid every frame with
 * tiny per-cell wind randomness on the foliage.  The "tree" is just a
 * static still life; the rustle is what keeps it from looking like
 * a screenshot.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine you've painted a sumi-e bonsai on rice paper.  The tree is
 * fixed forever; there's nothing to recompute when you display it.
 * Now imagine you tape that paper outdoors and watch it from a
 * distance: every few seconds a leaf flutters.  That's all the
 * animation we need — the tree's structure is its identity, the
 * flutter is just life.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. SKELETON GENERATION (once per seed/style change):
 *
 *       Init RNG from seed.
 *       Pick pot dimensions, compute pot_top y-coordinate.
 *       Generate trunk path according to style:
 *         - CHOKKAN: walk vertically up, tiny per-segment x-wobble.
 *         - MOYOGI:  x = mid + amp · sin(2π · (y / height) · waves).
 *         - SHAKAN:  x = mid + slope · (y_pot − y), straight line.
 *         - KENGAI:  phase 1 (vertical short rise) then phase 2 (arc
 *                    over pot rim outward and downward).
 *         - BUNJIN:  vertical nearly-straight, EXTRA tall, sparse.
 *       At each trunk node, with style-dependent probability, recurse
 *       into a branch of shorter length and rotated direction.
 *       Branches recursively subdivide; terminal branches drop a
 *       FOLIAGE CLOUD at their tip.
 *
 *  2. RASTERISE TO CHAR GRID (once after skeleton built):
 *
 *       For each segment:
 *         Walk a Bresenham line from (x0, y0) to (x1, y1).
 *         At each cell, decide a glyph by segment thickness:
 *           thick  → 'M', 'H', or thick-bracket
 *           medium → '|', '[', ']'
 *           thin   → '|', '/', '\\'
 *         Write to char_grid + pair_grid.
 *
 *       For each foliage cloud:
 *         Fill an ELLIPSE around (cx, cy) with semi-axes (rx, ry·CELL_ASPECT)
 *           — corrected so the cloud looks round in physical pixels.
 *         For each cell in the ellipse, write a random leaf glyph
 *           from the theme's leaf set.
 *
 *       Pot rasterised over the bottom rows last.
 *
 *  3. RENDER PER FRAME:
 *
 *       For each char_grid cell:
 *         If FOLIAGE cell AND (rustle_hash(cell, time) < RUSTLE_RATE):
 *             re-pick a random leaf glyph for this cell
 *         Emit (glyph, pair) to ncurses with batched attron/attroff.
 *
 *  4. CYCLE:
 *
 *       n / N — next / previous style: regenerate skeleton + raster.
 *       t / T — next / previous theme: re-init colours (~10 microsec),
 *                 char_grid stays the same, pair_grid recoloured by
 *                 theme region table.
 *       r     — new seed: regenerate skeleton + raster (same style).
 *
 * KEY FORMULAS
 * ────────────
 *  Trunk path (MOYOGI, S-curve):
 *    x(y) = mid + amp · sin(2π · waves · (y_pot − y) / trunk_height)
 *
 *  Trunk path (SHAKAN, slanted):
 *    x(y) = mid + slope · (y_pot − y)   ,   slope ∈ [0.15, 0.40]
 *
 *  Trunk path (KENGAI, cascade two-phase):
 *    if y in [y_apex, y_pot]:    vertical rise to y_apex
 *    else (y > y_apex):          arc out & down past pot edge
 *
 *  Branch direction at trunk node n:
 *    base_angle = ±π/2 (90° from trunk axis), ± random jitter
 *    length     = parent_length · BRANCH_LEN_FACTOR^depth
 *
 *  Foliage ellipse (cell-aspect-corrected so it appears round):
 *    inside iff  ((dx)/rx)² + ((dy · CELL_ASPECT)/ry)² ≤ 1
 *
 *  Wind rustle gate (per cell, per frame):
 *    rustle iff  hash(col, row, ⌊time · RUSTLE_FREQ⌋)  <  RUSTLE_RATE
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • TREE TOO TALL FOR SCREEN.  Trunk height is computed as a fraction
 *    of `rows_eff`; if the user resizes very small, foliage clouds
 *    clamp to the visible area.  KENGAI specifically can extend BELOW
 *    its pot — we draw the cascading branches directly on top of the
 *    pot frame in that style.
 *
 *  • CELL ASPECT.  Terminal cells are ~2× taller than wide.  Foliage
 *    ellipses use CELL_ASPECT to correct, otherwise circular clouds
 *    render as vertically-squashed rugby-ball shapes.
 *
 *  • RNG DETERMINISM.  Each (style, seed) produces an identical tree.
 *    Useful for reproducibility — if you find a beautiful seed, you
 *    can keep coming back to it.  `r` takes a fresh wall-clock seed.
 *
 *  • RUSTLE LOAD.  A small percentage (RUSTLE_RATE ~ 5 %) of foliage
 *    cells re-randomise per frame; combined with the per-cell hash
 *    you get a steady twinkle without wholesale repainting.  At
 *    higher rates the foliage looks noisy / boiling.
 *
 *  • INVERTED THEME (SUMI_E).  Pre-fill the visible region with the
 *    white "paper" bg pair before rendering tree cells, so that the
 *    tree's dark ink stands out against bright paper.  Same recipe
 *    used in nuke.c, mandelbulb.c, etc.
 *
 *  • WINTER THEME — set foliage density to zero in colour-init so no
 *    leaves get drawn; instead, branch tips paint snow-tip glyphs.
 *
 *  • PAUSE.  Freezes the rustle clock; the static skeleton is still
 *    visible.  Resume — clock continues from where it stopped.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press space.  Foliage rustle stops.  Tree holds still.  Resume —
 *    rustle picks up smoothly.
 *
 *  • Press `n`.  Style cycles; tree regenerates.  Each style has a
 *    distinct silhouette: CHOKKAN straight, MOYOGI S-curve obvious,
 *    SHAKAN visibly leaning, KENGAI cascading down past pot, BUNJIN
 *    thin and tall.
 *
 *  • Press `r`.  New seed: same style, different specific tree.
 *    Branch positions and foliage clouds shift; skeleton regenerates.
 *
 *  • Press `t`.  Theme cycles.  Tree silhouette identical across
 *    themes; only colours change.  WINTER drops all foliage; SUMI_E
 *    inverts to white-paper bg.
 *
 *  • At 60 × 20, the tree should still be recognisable as a bonsai
 *    with a pot.  Trunk + at least one branch + at least one foliage
 *    cloud should fit even at small terminal sizes.
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
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  30,    /* rustle is gentle; 30 fps is plenty   */
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    FPS_UPDATE_MS    = 500,

    PAIR_HUD          =  1,
    PAIR_HINT         =  2,
    PAIR_TRUNK_BASE   =  3,    /* +0..+2 — trunk dark/mid/light bark   */
    PAIR_FOLIAGE_BASE =  6,    /* +0..+5 — leaf colour variants        */
    PAIR_POT          = 12,
    PAIR_PAPER        = 13,    /* SUMI_E white-paper bg                */

    /* Skeleton pool sizes. */
    SEG_MAX           = 384,
    FOL_MAX           = 96,

    /* Char grid maximum (for static buffers). */
    GRID_MAX_W        = 280,
    GRID_MAX_H        = 90,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define CELL_ASPECT      2.0f      /* terminal cell h/w                  */

/* Tree generation — global tunables. */
#define MAX_DEPTH        7         /* max recursion depth                */
#define MIN_LEN          1.5f      /* segments shorter than this stop    */
#define BRANCH_LEN_F     0.66f     /* child length = parent · this       */

/* Wind rustle. */
#define RUSTLE_FREQ      6.0f      /* rustle hash key advances at this Hz */
#define RUSTLE_RATE      0.06f     /* fraction of foliage cells per tick  */

/* ── Style enum + per-style params ── */
typedef enum {
    STYLE_CHOKKAN = 0,    /* formal upright */
    STYLE_MOYOGI  = 1,    /* informal upright (S-curve) */
    STYLE_SHAKAN  = 2,    /* slanting */
    STYLE_KENGAI  = 3,    /* cascade */
    STYLE_BUNJIN  = 4,    /* literati */
    N_STYLES      = 5,
} TreeStyle;

typedef struct {
    const char *name;
    float       trunk_height_frac;  /* of (rows_eff − pot_rows)         */
    float       trunk_curve_amp;    /* MOYOGI sin amplitude (cells)     */
    float       trunk_curve_waves;  /* MOYOGI wavelengths over height   */
    float       trunk_lean;         /* SHAKAN slope (cells per row)     */
    float       branch_density;     /* probability per trunk node       */
    float       branch_length;      /* base branch length (cells)       */
    int         max_depth;
    float       foliage_size;       /* base foliage cloud radius        */
} StyleParams;

static const StyleParams styles[N_STYLES] = {
    /*  name           trunk_h  curv_amp  waves  lean  br_dens br_len  depth  fol_size */
    /* CHOKKAN */   { "CHOKKAN ", 0.65f,  0.5f,   0.0f, 0.00f,  0.45f,  6.5f,  6,    3.0f },
    /* MOYOGI  */   { "MOYOGI  ", 0.65f,  4.0f,   1.7f, 0.00f,  0.50f,  6.0f,  6,    2.8f },
    /* SHAKAN  */   { "SHAKAN  ", 0.62f,  0.0f,   0.0f, 0.30f,  0.55f,  5.5f,  6,    2.7f },
    /* KENGAI  */   { "KENGAI  ", 0.50f,  0.0f,   0.0f, 0.00f,  0.55f,  6.5f,  6,    2.5f },
    /* BUNJIN  */   { "BUNJIN  ", 0.78f,  1.0f,   0.5f, 0.05f,  0.18f,  4.5f,  5,    2.0f },
};

/* ── Theme ── */
typedef struct {
    const char *name;
    short       trunk[3];        /* dark / mid / light bark              */
    short       foliage[6];      /* 6 leaf colour variants for variety   */
    short       pot;
    bool        inverted;        /* white-paper bg + dark ink            */
    bool        bare;            /* WINTER: skip foliage clouds entirely */
    const char *leaf_set;        /* glyphs for leaves                    */
    const char *snow_tip;        /* WINTER tip char (else NULL)          */
} Theme;

#define N_THEMES 6

static const Theme themes[N_THEMES] = {
    /* name        trunk[]              foliage[]                              pot  inv   bare    leaves            snow */
    { "SPRING ",  { 130, 137, 173 },   {  28,  34,  64,  70, 112, 154 },      136, false, false, "&%#*o^",          NULL  },
    { "SUMMER ",  { 130, 137, 173 },   {  22,  28,  34,  64,  70, 112 },      136, false, false, "&%#@*o",          NULL  },
    { "AUTUMN ",  {  88, 130, 166 },   { 124, 130, 166, 202, 208, 220 },      130, false, false, "&%#*@^",          NULL  },
    { "WINTER ",  {  60,  66, 103 },   { 240, 244, 247, 250, 252, 255 },      103, false, true,  ".,oO",            "*"   },
    { "CHERRY ",  {  88, 130, 166 },   { 175, 211, 213, 217, 219, 225 },      130, false, false, "&%#*o^",          NULL  },
    { "SUMI_E ",  {  16, 232, 234 },   { 234, 236, 238, 241, 244, 247 },      234, true,  false, "&%#*o^",          NULL  },
};

/* Glyph palettes. */
static const char TRUNK_THICK[]    = "MHHM[]()";   /* depth 0-1, thick bark   */
static const char TRUNK_MEDIUM[]   = "|[]/\\";       /* depth 2-3, medium       */
static const char TRUNK_THIN[]     = "|/\\";         /* depth 4+, thin twigs    */

#define POT_TOP_CH      '_'
#define POT_SIDE_CH     '|'
#define POT_LIP_LEFT    '\\'
#define POT_LIP_RIGHT   '/'
#define POT_BOTTOM_CH   ':'

/* Per-cell role (drives glyph + colour selection at render time). */
typedef enum {
    ROLE_EMPTY = 0,
    ROLE_TRUNK_DARK,
    ROLE_TRUNK_MID,
    ROLE_TRUNK_LIGHT,
    ROLE_FOLIAGE,
    ROLE_SNOW_TIP,        /* WINTER only */
    ROLE_POT,
} CellRole;

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
    const Theme *t = &themes[idx];
    short bg = t->inverted ? 231 : -1;

    if (COLORS >= 256) {
        for (int i = 0; i < 3; i++)
            init_pair((short)(PAIR_TRUNK_BASE + i), t->trunk[i], bg);
        for (int i = 0; i < 6; i++)
            init_pair((short)(PAIR_FOLIAGE_BASE + i), t->foliage[i], bg);
        init_pair(PAIR_POT,   t->pot, bg);
        init_pair(PAIR_PAPER, 16,     t->inverted ? 231 : -1);  /* unused if not inverted */
    } else {
        for (int i = 0; i < 3; i++)
            init_pair((short)(PAIR_TRUNK_BASE + i),
                      t->inverted ? COLOR_BLACK : COLOR_YELLOW,
                      t->inverted ? COLOR_WHITE : (short)-1);
        for (int i = 0; i < 6; i++)
            init_pair((short)(PAIR_FOLIAGE_BASE + i),
                      t->inverted ? COLOR_BLACK : COLOR_GREEN,
                      t->inverted ? COLOR_WHITE : (short)-1);
        init_pair(PAIR_POT,
                  t->inverted ? COLOR_BLACK : COLOR_YELLOW,
                  t->inverted ? COLOR_WHITE : (short)-1);
        init_pair(PAIR_PAPER, COLOR_BLACK, COLOR_WHITE);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §4  random — LCG + cheap hashes                                        */
/* ===================================================================== */

static inline uint32_t lcg_next(uint32_t *st)
{
    *st = *st * 1664525u + 1013904223u;
    return *st;
}

static inline float lcg_unit(uint32_t *st)
{
    return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

static inline float lcg_range(uint32_t *st, float lo, float hi)
{
    return lo + (hi - lo) * lcg_unit(st);
}

/* Stateless 3-input hash → uint32 for per-cell rustle gating. */
static inline uint32_t hash3(int x, int y, int t)
{
    uint32_t h = (uint32_t)x * 374761393u
               + (uint32_t)y * 668265263u
               + (uint32_t)t * 2147483647u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}

/* ===================================================================== */
/* §5  tree — recursive skeleton generation                               */
/* ===================================================================== */

typedef struct {
    float    x0, y0, x1, y1;
    int      depth;        /* 0 = trunk, 1 = primary branch, ...   */
} Segment;

typedef struct {
    float    cx, cy;       /* centre in cell coords                */
    float    radius;       /* base radius (x); y radius applies aspect */
    int      depth_at_tip; /* used for snow-tip flag in WINTER     */
} FoliageCloud;

typedef struct {
    Segment      segs[SEG_MAX];
    int          n_segs;
    FoliageCloud fols[FOL_MAX];
    int          n_fols;
} Tree;

static Tree g_tree;

static void tree_clear(Tree *t)
{
    t->n_segs = 0;
    t->n_fols = 0;
}

static void seg_add(Tree *t, float x0, float y0, float x1, float y1, int depth)
{
    if (t->n_segs >= SEG_MAX) return;
    Segment *s = &t->segs[t->n_segs++];
    s->x0 = x0; s->y0 = y0; s->x1 = x1; s->y1 = y1;
    s->depth = depth;
}

static void fol_add(Tree *t, float cx, float cy, float radius, int depth)
{
    if (t->n_fols >= FOL_MAX) return;
    FoliageCloud *f = &t->fols[t->n_fols++];
    f->cx = cx; f->cy = cy; f->radius = radius;
    f->depth_at_tip = depth;
}

/*
 * grow_branch — recursive descent.  Walks (start → end) along `dir` for
 * `length` cells, emits a Segment, then either spawns sub-branches or
 * places a foliage cloud at the tip.
 *
 * `dir` is a 2-D unit-ish vector in screen-cell space; we scale y by
 * 1/CELL_ASPECT so that "1 unit of length" produces equal physical
 * pixels regardless of direction.
 */
static void grow_branch(Tree *t, uint32_t *rng,
                        float x, float y, float dir_x, float dir_y,
                        float length, int depth,
                        const StyleParams *sp, bool no_foliage)
{
    if (length < MIN_LEN || depth > sp->max_depth) {
        if (!no_foliage) {
            float r = sp->foliage_size * (0.6f + lcg_unit(rng) * 0.6f);
            fol_add(t, x, y, r, depth);
        }
        return;
    }

    /* Apply cell-aspect to y component so the branch is the same
     * physical length regardless of angle. */
    float ex = x + dir_x * length;
    float ey = y + dir_y * length / CELL_ASPECT;
    seg_add(t, x, y, ex, ey, depth);

    /* Decide how to continue: extend (with bend) and/or branch. */
    int n_subs = 1;
    if (lcg_unit(rng) < sp->branch_density && depth < sp->max_depth - 1) {
        n_subs = 2;                    /* fork into 2 */
    }
    if (depth == 0 && lcg_unit(rng) < 0.6f) n_subs = 2;

    for (int i = 0; i < n_subs; i++) {
        float bend = lcg_range(rng, -0.40f, 0.40f);
        if (n_subs > 1) {
            /* Two children fork outward at moderate angles. */
            bend = (i == 0 ? -1 : +1) * lcg_range(rng, 0.5f, 0.95f);
        }
        float c = cosf(bend), s = sinf(bend);
        float ndx = dir_x * c - dir_y * s;
        float ndy = dir_x * s + dir_y * c;
        /* Slight upward bias (anti-gravity for branches that aren't
         * cascading) so non-trunk branches reach upward. */
        if (depth > 0 && ndy > -0.1f) ndy -= 0.2f;
        float nlen = length * BRANCH_LEN_F;
        grow_branch(t, rng, ex, ey, ndx, ndy, nlen, depth + 1, sp, no_foliage);
    }
}

/*
 * tree_generate — entry point.  Picks pot dimensions, lays out the
 * trunk according to style, and recurses for branches.
 *
 * Returns the y-coordinate of the pot's TOP edge so the renderer can
 * draw the pot frame.
 */
static int tree_generate(Tree *t, uint32_t seed, int cols, int rows_eff,
                         TreeStyle style, bool no_foliage)
{
    uint32_t rng = seed;
    tree_clear(t);

    const StyleParams *sp = &styles[style];

    /* Pot occupies bottom POT_ROWS rows. */
    int pot_rows = (rows_eff >= 18) ? 4 : 3;
    int pot_top  = rows_eff - pot_rows - 1;
    if (pot_top < 5) pot_top = rows_eff - 1;

    /* Trunk rooted at (mid, pot_top). */
    float trunk_height = (float)(pot_top) * sp->trunk_height_frac;
    if (trunk_height > (float)pot_top - 1.0f) trunk_height = (float)pot_top - 1.0f;
    if (trunk_height < 4.0f) trunk_height = 4.0f;
    float mid = (float)cols * 0.5f;

    /* Build trunk as a poly-line of small segments — gives the trunk
     * its style-specific shape.  At each segment endpoint we have a
     * branching opportunity. */
    int trunk_segs = (int)(trunk_height * 1.8f);
    if (trunk_segs < 6) trunk_segs = 6;
    if (trunk_segs > 30) trunk_segs = 30;

    float prev_x = mid, prev_y = (float)pot_top;
    for (int i = 1; i <= trunk_segs; i++) {
        float frac = (float)i / (float)trunk_segs;
        float y    = (float)pot_top - frac * trunk_height;
        float x;
        switch (style) {
        case STYLE_CHOKKAN:
            x = mid + lcg_range(&rng, -0.3f, 0.3f);
            break;
        case STYLE_MOYOGI:
            x = mid + sp->trunk_curve_amp
                    * sinf(2.0f * (float)M_PI * sp->trunk_curve_waves * frac);
            break;
        case STYLE_SHAKAN:
            x = mid + sp->trunk_lean * frac * trunk_height;
            break;
        case STYLE_KENGAI: {
            /* Trunk rises a short way, then arcs over the pot rim
             * outward and downward.  We model it by putting trunk
             * apex at frac = 0.30, then path drops/right.    */
            float apex = 0.30f;
            if (frac < apex) {
                /* short straight rise */
                x = mid;
            } else {
                float t2 = (frac - apex) / (1.0f - apex);
                /* Arc outward right and slightly downward */
                x = mid + 6.0f * sinf(t2 * (float)M_PI * 0.6f) * 1.5f;
                /* Cascade down: reverse y to grow downward past pot */
                y = (float)pot_top - apex * trunk_height + t2 * 6.0f;
            }
            break;
        }
        case STYLE_BUNJIN:
        default:
            x = mid + sp->trunk_curve_amp
                    * sinf(2.0f * (float)M_PI * sp->trunk_curve_waves * frac
                           + 0.5f);
            break;
        }

        /* Add the trunk segment. */
        seg_add(t, prev_x, prev_y, x, y, 0);

        /* Branching opportunity (skip first 30 % so trunk has a clean
         * base, also skip very last 10 % so trunk apex is clean). */
        if (frac > 0.30f && frac < 0.92f
            && lcg_unit(&rng) < sp->branch_density) {
            float angle = (lcg_unit(&rng) < 0.5f ? -1.0f : 1.0f)
                        * lcg_range(&rng, 0.7f, 1.2f);
            float dx = sinf(angle);
            float dy = -fabsf(cosf(angle));   /* always upward          */
            float blen = sp->branch_length * (1.0f - frac * 0.5f);
            grow_branch(t, &rng, x, y, dx, dy, blen, 1, sp, no_foliage);
        }

        prev_x = x;
        prev_y = y;
    }

    /* Trunk tip foliage (skipped for KENGAI which trails off). */
    if (style != STYLE_KENGAI && !no_foliage) {
        float r = sp->foliage_size * 1.4f;     /* big crown at the top */
        fol_add(t, prev_x, prev_y - 0.5f, r, sp->max_depth);
    }
    if (style == STYLE_KENGAI && !no_foliage) {
        /* Foliage at the cascading tip */
        fol_add(t, prev_x, prev_y, sp->foliage_size * 1.2f, sp->max_depth);
    }

    return pot_top;
}

/* ===================================================================== */
/* §6  raster — char grid + Bresenham + ellipse fill                      */
/* ===================================================================== */

static char     g_char[GRID_MAX_H][GRID_MAX_W];
static uint8_t  g_role[GRID_MAX_H][GRID_MAX_W];
static uint8_t  g_pair_idx[GRID_MAX_H][GRID_MAX_W];

static void grid_clear(int rows_eff, int cols)
{
    for (int r = 0; r < rows_eff && r < GRID_MAX_H; r++)
        for (int c = 0; c < cols && c < GRID_MAX_W; c++) {
            g_char[r][c]     = ' ';
            g_role[r][c]     = ROLE_EMPTY;
            g_pair_idx[r][c] = 0;
        }
}

static inline void grid_put(int r, int c, char ch, uint8_t role,
                            uint8_t pair_idx, int rows_eff, int cols)
{
    if (r < 0 || r >= rows_eff || r >= GRID_MAX_H) return;
    if (c < 0 || c >= cols     || c >= GRID_MAX_W) return;
    /* Don't overwrite trunk/branch cells with foliage edges; trunk wins. */
    uint8_t existing = g_role[r][c];
    if (existing >= ROLE_TRUNK_DARK && existing <= ROLE_TRUNK_LIGHT
        && role == ROLE_FOLIAGE) return;
    g_char[r][c]     = ch;
    g_role[r][c]     = role;
    g_pair_idx[r][c] = pair_idx;
}

/*
 * draw_segment — Bresenham line in cell coords.  Picks trunk-thick /
 * medium / thin glyph by depth and writes to the grid.
 */
static void draw_segment(const Segment *s, int rows_eff, int cols)
{
    int x0 = (int)(s->x0 + 0.5f);
    int y0 = (int)(s->y0 + 0.5f);
    int x1 = (int)(s->x1 + 0.5f);
    int y1 = (int)(s->y1 + 0.5f);

    /* Pick glyph + role by depth. */
    char glyph;
    uint8_t role;
    uint8_t pair_idx;     /* 0..2 within trunk palette */
    if (s->depth <= 1) {
        glyph    = TRUNK_THICK[((unsigned)abs(x0 + y0)) % (sizeof TRUNK_THICK - 1)];
        role     = ROLE_TRUNK_DARK;
        pair_idx = 0;
    } else if (s->depth <= 3) {
        glyph    = TRUNK_MEDIUM[((unsigned)abs(x0 + y0)) % (sizeof TRUNK_MEDIUM - 1)];
        role     = ROLE_TRUNK_MID;
        pair_idx = 1;
    } else {
        glyph    = TRUNK_THIN[((unsigned)abs(x0 + y0)) % (sizeof TRUNK_THIN - 1)];
        role     = ROLE_TRUNK_LIGHT;
        pair_idx = 2;
    }

    /* Bresenham. */
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    while (1) {
        /* Choose a glyph that hints at direction for diagonal cells. */
        char g = glyph;
        if (dx > dy * 2)        g = (s->depth <= 1) ? '=' : '-';
        else if (dy > dx * 2)   g = (s->depth <= 1) ? 'H' : '|';
        else if (sx == sy)      g = (s->depth <= 1) ? 'M' : '\\';
        else if (sx != sy)      g = (s->depth <= 1) ? 'M' : '/';
        grid_put(y0, x0, g, role, pair_idx, rows_eff, cols);

        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/*
 * draw_foliage_cloud — fill an aspect-corrected ellipse with random
 * leaf glyphs from the active theme's leaf set.
 */
static void draw_foliage_cloud(const FoliageCloud *f, const Theme *th,
                                uint32_t *rng,
                                int rows_eff, int cols)
{
    int cxi = (int)(f->cx + 0.5f);
    int cyi = (int)(f->cy + 0.5f);
    float rx = f->radius;
    float ry = f->radius / CELL_ASPECT;     /* corrected → round on screen */

    int yr_lo = cyi - (int)(ry + 1);
    int yr_hi = cyi + (int)(ry + 1);
    int xr_lo = cxi - (int)(rx + 1);
    int xr_hi = cxi + (int)(rx + 1);

    int leaf_set_len = (int)strlen(th->leaf_set);

    for (int y = yr_lo; y <= yr_hi; y++) {
        for (int x = xr_lo; x <= xr_hi; x++) {
            float dx = (float)(x - cxi);
            float dy = (float)(y - cyi) * CELL_ASPECT;
            float r2 = (dx*dx) / (rx*rx) + (dy*dy) / (rx*rx);
            if (r2 > 1.0f) continue;

            /* Density falloff at the edge — softer cloud. */
            if (r2 > 0.6f && lcg_unit(rng) > 0.7f) continue;

            char g = th->leaf_set[lcg_next(rng) % (uint32_t)leaf_set_len];
            uint8_t pair_idx = (uint8_t)(lcg_next(rng) % 6u);
            grid_put(y, x, g, ROLE_FOLIAGE, pair_idx, rows_eff, cols);
        }
    }

    /* WINTER snow tip on the topmost cell of each cloud (depth at tip
     * means this is a terminal foliage point). */
    if (th->snow_tip != NULL) {
        int top = yr_lo + 1;
        if (top >= 0 && top < rows_eff && cxi >= 0 && cxi < cols) {
            grid_put(top, cxi, th->snow_tip[0], ROLE_SNOW_TIP, 5,
                     rows_eff, cols);
        }
    }
}

/* ===================================================================== */
/* §7  pot — pot frame rendering                                          */
/* ===================================================================== */

/*
 * draw_pot — a small trapezoidal pot centred under the trunk.  Width
 * is half the screen, capped at PRECEDED constants.  The pot frame is
 * drawn last so it sits on top of any trunk segments that dipped into
 * its cells (e.g. KENGAI cascades).
 */
static void draw_pot(int pot_top, int rows_eff, int cols)
{
    int w = cols / 3;
    if (w < 12) w = 12;
    if (w > cols - 4) w = cols - 4;
    int x0 = cols / 2 - w / 2;
    int x1 = x0 + w - 1;

    /* Top rim row (just above the pot proper).  Slight overhang. */
    int rim_y = pot_top + 1;
    if (rim_y >= 0 && rim_y < rows_eff) {
        for (int x = x0 - 1; x <= x1 + 1; x++) {
            if (x < 0 || x >= cols) continue;
            char c = POT_TOP_CH;
            if (x == x0 - 1)      c = POT_LIP_LEFT;
            else if (x == x1 + 1) c = POT_LIP_RIGHT;
            grid_put(rim_y, x, c, ROLE_POT, 0, rows_eff, cols);
        }
    }

    /* Side walls + bottom row(s). */
    int pot_rows = rows_eff - pot_top - 1;
    if (pot_rows < 1) pot_rows = 1;
    if (pot_rows > 4) pot_rows = 4;

    for (int i = 0; i < pot_rows; i++) {
        int y = rim_y + 1 + i;
        if (y >= rows_eff) break;

        if (i == pot_rows - 1) {
            /* Bottom row — solid. */
            for (int x = x0; x <= x1; x++) {
                if (x < 0 || x >= cols) continue;
                char c = POT_BOTTOM_CH;
                if (x == x0)      c = '\\';
                else if (x == x1) c = '/';
                grid_put(y, x, c, ROLE_POT, 0, rows_eff, cols);
            }
        } else {
            /* Side walls. */
            if (x0 >= 0 && x0 < cols)
                grid_put(y, x0, POT_SIDE_CH, ROLE_POT, 0, rows_eff, cols);
            if (x1 >= 0 && x1 < cols)
                grid_put(y, x1, POT_SIDE_CH, ROLE_POT, 0, rows_eff, cols);
            /* Faint interior fill (just a few dots). */
            for (int x = x0 + 1; x <= x1 - 1; x++) {
                if (x < 0 || x >= cols) continue;
                if (((unsigned)(x + y)) & 3u) continue;   /* 25 % density */
                grid_put(y, x, '.', ROLE_POT, 0, rows_eff, cols);
            }
        }
    }
}

/* ===================================================================== */
/* §8  scene — generate + raster + render with rustle                     */
/* ===================================================================== */

typedef struct {
    bool       paused;
    int        cols, rows;
    uint32_t   seed;
    int        current_style;
    int        current_theme;
    float      time;
    int        pot_top;     /* y row of pot's top edge                  */
} Scene;

static void scene_rebuild(Scene *s)
{
    int rows_eff = s->rows - 1;
    bool no_foliage = themes[s->current_theme].bare;

    s->pot_top = tree_generate(&g_tree, s->seed, s->cols, rows_eff,
                                (TreeStyle)s->current_style, no_foliage);

    grid_clear(rows_eff, s->cols);

    /* Draw segments (trunk + branches). */
    for (int i = 0; i < g_tree.n_segs; i++)
        draw_segment(&g_tree.segs[i], rows_eff, s->cols);

    /* Draw foliage. */
    const Theme *th = &themes[s->current_theme];
    uint32_t fol_rng = s->seed ^ 0xDEC0DEu;
    for (int i = 0; i < g_tree.n_fols; i++)
        draw_foliage_cloud(&g_tree.fols[i], th, &fol_rng, rows_eff, s->cols);

    /* Draw pot. */
    draw_pot(s->pot_top, rows_eff, s->cols);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->cols          = cols;
    s->rows          = rows;
    s->seed          = (uint32_t)clock_ns();
    s->current_style = STYLE_MOYOGI;
    s->current_theme = 0;
    s->time          = 0.0f;
    scene_rebuild(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
    scene_rebuild(s);
}

static void scene_reseed(Scene *s)
{
    s->seed = (uint32_t)clock_ns() ^ 0xA5A5A5A5u;
    scene_rebuild(s);
}

static void scene_cycle_style(Scene *s, int dir)
{
    int idx = s->current_style + dir;
    while (idx < 0) idx += N_STYLES;
    s->current_style = idx % N_STYLES;
    scene_rebuild(s);
}

static void scene_cycle_theme(Scene *s, int dir)
{
    int idx = s->current_theme + dir;
    while (idx < 0) idx += N_THEMES;
    s->current_theme = idx % N_THEMES;
    theme_apply(s->current_theme);
    scene_rebuild(s);     /* WINTER toggles foliage on/off → rebuild */
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->time += dt;
}

/*
 * scene_render — emit the char grid to ncurses.  For foliage cells,
 * gate a per-cell rustle: hash(col, row, ⌊time · RUSTLE_FREQ⌋) gives
 * a stable random per-cell seed; if the high bits fall under a
 * threshold, re-pick the leaf glyph.
 *
 * Inverted theme (SUMI_E): pre-fill white background before drawing
 * the tree, and disable A_BOLD/A_DIM (light fg over white bg would
 * invert the brightness intent).
 */
static void scene_render(const Scene *s)
{
    int rows_eff = s->rows - 1;
    if (rows_eff < 1) return;
    int cols = s->cols;
    if (rows_eff > GRID_MAX_H) rows_eff = GRID_MAX_H;
    if (cols     > GRID_MAX_W) cols     = GRID_MAX_W;

    const Theme *th       = &themes[s->current_theme];
    bool         inverted = th->inverted;
    int          time_key = (int)(s->time * RUSTLE_FREQ);
    int          leaf_len = (int)strlen(th->leaf_set);

    /* Pre-fill white "paper" if SUMI_E. */
    if (inverted) {
        attron(COLOR_PAIR(PAIR_PAPER));
        for (int r = 0; r < rows_eff; r++)
            for (int c = 0; c < cols; c++)
                mvaddch(r, c, ' ');
        attroff(COLOR_PAIR(PAIR_PAPER));
    }

    int    last_pair = -1;
    attr_t last_attr = 0;

    for (int r = 0; r < rows_eff; r++) {
        for (int c = 0; c < cols; c++) {
            uint8_t role = g_role[r][c];
            if (role == ROLE_EMPTY) {
                if (!inverted && last_pair >= 0) {
                    attroff(COLOR_PAIR(last_pair) | last_attr);
                    last_pair = -1;
                }
                continue;
            }

            char glyph = g_char[r][c];
            int  pair;
            attr_t attr;

            switch (role) {
            case ROLE_TRUNK_DARK:
                pair = PAIR_TRUNK_BASE + 0;
                attr = inverted ? A_NORMAL : A_BOLD;
                break;
            case ROLE_TRUNK_MID:
                pair = PAIR_TRUNK_BASE + 1;
                attr = A_NORMAL;
                break;
            case ROLE_TRUNK_LIGHT:
                pair = PAIR_TRUNK_BASE + 2;
                attr = inverted ? A_NORMAL : A_DIM;
                break;
            case ROLE_FOLIAGE: {
                uint8_t pair_idx = g_pair_idx[r][c];
                pair = PAIR_FOLIAGE_BASE + (pair_idx % 6);
                attr = (pair_idx >= 4 && !inverted) ? A_BOLD : A_NORMAL;
                /* Per-cell rustle: stable bytes per (cell, time_key);
                 * if random byte under threshold, re-pick glyph. */
                uint32_t h = hash3(c, r, time_key);
                if ((h & 0xFFu) < (uint32_t)(255.0f * RUSTLE_RATE)) {
                    glyph = th->leaf_set[(h >> 8) % (uint32_t)leaf_len];
                }
                break;
            }
            case ROLE_SNOW_TIP:
                pair = PAIR_FOLIAGE_BASE + 5;       /* whitest */
                attr = inverted ? A_NORMAL : A_BOLD;
                break;
            case ROLE_POT:
            default:
                pair = PAIR_POT;
                attr = A_NORMAL;
                break;
            }

            if (pair != last_pair || attr != last_attr) {
                if (last_pair >= 0)
                    attroff(COLOR_PAIR(last_pair) | last_attr);
                attron(COLOR_PAIR(pair) | attr);
                last_pair = pair;
                last_attr = attr;
            }
            mvaddch(r, c, (chtype)(unsigned char)glyph);
        }
    }
    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

/* ===================================================================== */
/* §9  screen + app                                                       */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *sc)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) { (void)sc; endwin(); }
static void screen_resize_curses(Screen *sc)
{
    endwin();
    refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_render(s);

    char buf[200];
    snprintf(buf, sizeof buf,
             " BONSAI   %s   style:%s   theme:%s   "
             "%4.1f fps  %3d Hz   "
             "n/N:style  t/T:theme  r:reseed  spc:pause  q:quit ",
             s->paused ? "PAUSED" : "RUSTLE",
             styles[s->current_style].name,
             themes[s->current_theme].name,
             fps, sim_fps);

    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

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
    screen_resize_curses(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reseed(s);                              break;

    case 'n':           scene_cycle_style(s, +1);                     break;
    case 'N':           scene_cycle_style(s, -1);                     break;

    case 't':           scene_cycle_theme(s, +1);                     break;
    case 'T':           scene_cycle_theme(s, -1);                     break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
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
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        float dt_sec = (float)dt / (float)NS_PER_SEC;
        scene_tick(&app->scene, dt_sec);

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t target_ns = TICK_NS(app->sim_fps);
        int64_t elapsed   = clock_ns() - frame_time + dt;
        clock_sleep_ns(target_ns - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
