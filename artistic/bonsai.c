/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bonsai.c — a gallery of ASCII bonsai trees: 5 classical styles, 6 colour
 * themes, a tree that's grown once and then gently rustles in the wind.
 *
 * The trunk shapes follow the classic bonsai style names (Naka, Bonsai
 * Techniques I & II); the branching is a stochastic cousin of an L-system
 * (Prusinkiewicz & Lindenmayer, The Algorithmic Beauty of Plants, 1990).
 */

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

/* ── §1 config — constants, style + theme tables, glyph palettes ── */

enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  30,    /* rustle is gentle, so 30 fps is plenty */
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    FPS_UPDATE_MS    = 500,

    PAIR_HUD          =  1,
    PAIR_HINT         =  2,
    PAIR_TRUNK_BASE   =  3,    /* trunk bark: +0 dark, +1 mid, +2 light  */
    PAIR_FOLIAGE_BASE =  6,    /* leaf colours: +0..+5, six hues         */
    PAIR_POT          = 12,

    /* How many segments / foliage clouds a tree can have at most. */
    SEG_MAX           = 384,
    FOL_MAX           = 96,

    /* Backing size of the baked character grid. */
    GRID_MAX_W        = 280,
    GRID_MAX_H        = 90,

    /* Pot placement: a tall screen gets a 4-row pot, a short one gets 3. */
    POT_TALL_MIN_ROWS = 18,
    POT_ROWS_TALL     =  4,
    POT_ROWS_SHORT    =  3,

    /* Limits on how many nodes the trunk poly-line is split into. */
    TRUNK_SEGS_MIN    =  6,
    TRUNK_SEGS_MAX    = 30,

    FOLIAGE_BOLD_FROM =  4,     /* leaf colour index at or above this draws bold */
    MAX_FRAME_MS      = 100,    /* cap a long frame so a stall can't snowball     */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* A terminal cell is about twice as tall as it is wide; we use this to
 * keep circles looking round instead of squashed. */
#define CELL_ASPECT      2.0f

/* How branches grow.  (Each style sets its own recursion depth cap.) */
#define MIN_LEN          1.5f      /* a branch shorter than this stops growing */
#define BRANCH_LEN_F     0.66f     /* each child branch is this fraction of its parent */

/* Trunk shape. */
#define MIN_TRUNK_HEIGHT     4.0f  /* trunk is never shorter than this (cells)  */
#define TRUNK_SEGS_PER_CELL  1.8f  /* poly-line nodes per cell of trunk height  */
#define BRANCH_BAND_LO       0.30f /* only sprout branches between these heights */
#define BRANCH_BAND_HI       0.92f /*   so the bare base and tip stay clean      */
#define KENGAI_APEX_FRAC     0.30f /* cascade style: trunk rises to here, then bends down */

/* Wind rustle. */
#define RUSTLE_FREQ      6.0f      /* how fast the rustle pattern changes, in Hz */
#define RUSTLE_RATE      0.06f     /* fraction of leaves flickering at any moment */

/*
 * TreeStyle — the five classic bonsai shapes a grower would name (Naka,
 * Bonsai Techniques).  Each value indexes the styles[] table below; the
 * only thing that really changes between styles is the trunk shape (in
 * trunk_x_at, §5) and that row of knobs.
 * N_STYLES is both the array length and the wrap-around count for n/N
 * cycling, so it MUST stay last.
 */
typedef enum {
    STYLE_CHOKKAN = 0,    /* formal upright — straight vertical trunk         */
    STYLE_MOYOGI  = 1,    /* informal upright — S-curved trunk (most popular) */
    STYLE_SHAKAN  = 2,    /* slanting — trunk leans at a fixed angle          */
    STYLE_KENGAI  = 3,    /* cascade — trunk arcs down past the pot rim       */
    STYLE_BUNJIN  = 4,    /* literati — tall, thin, sparse, much empty space  */
    N_STYLES      = 5,    /* style count + cycle modulus (keep last)          */
} TreeStyle;

/*
 * StyleParams — one row of dials per style.  The same tree-growing code
 * (§5) reads these numbers and produces a different silhouette for each
 * style.  Some dials are only used by one style (noted in the comments).
 */
typedef struct {
    const char *name;               /* HUD label, space-padded to fixed width    */
    float       trunk_height_frac;  /* trunk height as a fraction of usable rows  */
    float       trunk_curve_amp;    /* MOYOGI/BUNJIN: S-curve amplitude, cells    */
    float       trunk_curve_waves;  /* MOYOGI/BUNJIN: S-curve wavelengths / height */
    float       trunk_lean;         /* SHAKAN: slope, cells of x per row of y      */
    float       branch_density;     /* P(spawn branch) at each node, 0..1         */
    float       branch_length;      /* base branch length at depth 1, cells       */
    int         max_depth;          /* recursion-depth cap for this style         */
    float       foliage_size;       /* base foliage-cloud radius, cells           */
} StyleParams;

static const StyleParams styles[N_STYLES] = {
    /*  name           trunk_h  curv_amp  waves  lean  br_dens br_len  depth  fol_size */
    /* CHOKKAN */   { "CHOKKAN ", 0.65f,  0.5f,   0.0f, 0.00f,  0.45f,  6.5f,  6,    3.0f },
    /* MOYOGI  */   { "MOYOGI  ", 0.65f,  4.0f,   1.7f, 0.00f,  0.50f,  6.0f,  6,    2.8f },
    /* SHAKAN  */   { "SHAKAN  ", 0.62f,  0.0f,   0.0f, 0.30f,  0.55f,  5.5f,  6,    2.7f },
    /* KENGAI  */   { "KENGAI  ", 0.50f,  0.0f,   0.0f, 0.00f,  0.55f,  6.5f,  6,    2.5f },
    /* BUNJIN  */   { "BUNJIN  ", 0.78f,  1.0f,   0.5f, 0.05f,  0.18f,  4.5f,  5,    2.0f },
};

/*
 * Theme — a season or mood: the colours and the set of leaf characters a
 * tree is painted with.  Switching theme re-binds the colour pairs (§3)
 * and rebuilds the tree, because the WINTER theme strips all the leaves,
 * which changes the shape and not just the colour.  The colour numbers
 * are xterm-256 indices, kept bright so they stay readable on a dark
 * background.  MONO is the grey-only theme.
 */
typedef struct {
    const char *name;            /* HUD label, padded to a fixed width          */
    short       trunk[3];        /* bark colours: dark / mid / light            */
    short       foliage[6];      /* six leaf colours, so a clump isn't flat     */
    short       pot;             /* pot colour                                  */
    bool        bare;            /* true only for WINTER: branches, no leaves   */
    const char *leaf_set;        /* the characters a leaf cell can be drawn as  */
    const char *snow_tip;        /* WINTER: snow character on top of a clump; NULL otherwise */
} Theme;

#define N_THEMES 6

static const Theme themes[N_THEMES] = {
    /* name        trunk[]              foliage[]                              pot  bare    leaves            snow */
    { "SPRING ",  { 130, 137, 173 },   {  28,  34,  64,  70, 112, 154 },      136, false, "&%#*o^",          NULL  },
    { "SUMMER ",  { 130, 137, 173 },   {  22,  28,  34,  64,  70, 112 },      136, false, "&%#@*o",          NULL  },
    { "AUTUMN ",  {  88, 130, 166 },   { 124, 130, 166, 202, 208, 220 },      130, false, "&%#*@^",          NULL  },
    { "WINTER ",  {  60,  66, 103 },   { 240, 244, 247, 250, 252, 255 },      103, true,  ".,oO",            "*"   },
    { "CHERRY ",  {  88, 130, 166 },   { 175, 211, 213, 217, 219, 225 },      130, false, "&%#*o^",          NULL  },
    { "MONO   ",  { 245, 250, 255 },   { 246, 248, 250, 252, 253, 255 },      250, false, "&%#*o^",          NULL  },
};

/* Characters used to draw bark, picked by how thick the branch is. */
static const char TRUNK_THICK[]    = "MHHM[]()";   /* the trunk and big limbs */
static const char TRUNK_MEDIUM[]   = "|[]/\\";       /* medium branches         */
static const char TRUNK_THIN[]     = "|/\\";         /* thin twigs              */

#define POT_TOP_CH      '_'
#define POT_SIDE_CH     '|'
#define POT_LIP_LEFT    '\\'
#define POT_LIP_RIGHT   '/'
#define POT_BOTTOM_CH   ':'

/*
 * CellRole — what a cell in the baked grid is: empty, bark, leaf, snow,
 * or pot.  Set when the tree is built (§6/§7), read at draw time (§11) to
 * pick the right colour.  Two ordering rules the code leans on:
 *   - ROLE_EMPTY = 0, so a freshly zeroed grid means "nothing here".
 *   - the three trunk roles are listed back-to-back, so the code can ask
 *     "is this a trunk cell?" with a single range check.
 */
typedef enum {
    ROLE_EMPTY = 0,       /* background — nothing drawn                     */
    ROLE_TRUNK_DARK,      /* thick bark (trunk and big limbs)              */
    ROLE_TRUNK_MID,       /* medium branches                               */
    ROLE_TRUNK_LIGHT,     /* thin twigs                                    */
    ROLE_FOLIAGE,         /* a leaf — one of six colours, may flicker      */
    ROLE_SNOW_TIP,        /* WINTER only: snow on top of a leaf clump      */
    ROLE_POT,             /* pot frame                                     */
} CellRole;

/* ── §2 clock — monotonic time + sleep ── */

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

/* ── §3 color — bind a theme's colours into ncurses pairs ── */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *t = &themes[idx];

    /* 256-colour terminals get the real theme; 8-colour ones fall back. */
    if (COLORS >= 256) {
        for (int i = 0; i < 3; i++)
            init_pair((short)(PAIR_TRUNK_BASE + i), t->trunk[i], -1);
        for (int i = 0; i < 6; i++)
            init_pair((short)(PAIR_FOLIAGE_BASE + i), t->foliage[i], -1);
        init_pair(PAIR_POT, t->pot, -1);
    } else {
        for (int i = 0; i < 3; i++)
            init_pair((short)(PAIR_TRUNK_BASE + i), COLOR_YELLOW, -1);
        for (int i = 0; i < 6; i++)
            init_pair((short)(PAIR_FOLIAGE_BASE + i), COLOR_GREEN, -1);
        init_pair(PAIR_POT, COLOR_YELLOW, -1);
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

/* ── §4 random — a tiny random-number generator and a hash ── */
/*
 * These keep no hidden state.  The caller hands in a counter (st) that the
 * generator scrambles; the same seed always produces the same tree, which
 * is what lets reseeding be repeatable.  hash3 just mixes three numbers
 * into one and is used to decide which leaves flicker.
 */

/* Standard "linear congruential" generator: multiply-and-add to get the
 * next pseudo-random number (constants from Numerical Recipes). */
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

/* Mix three numbers into one scrambled number; same inputs, same output. */
static inline uint32_t hash3(int x, int y, int t)
{
    uint32_t h = (uint32_t)x * 374761393u
               + (uint32_t)y * 668265263u
               + (uint32_t)t * 2147483647u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}

/* ── §5 generation — grow the tree skeleton (runs once, not per frame) ── */

/*
 * Segment — one straight piece of trunk or branch.  Endpoints are stored
 * as floats so the generator can work at finer-than-a-cell precision and
 * round only when it draws.  `depth` says how far from the trunk this
 * piece is (0 = trunk, 1 = main branch, ...) and also picks its thickness:
 * low depth = thick bark, high depth = thin twig.
 */
typedef struct {
    float    x0, y0, x1, y1;   /* start and end point, in cell coordinates  */
    int      depth;            /* 0 = trunk, higher = thinner branch        */
} Segment;

/*
 * FoliageCloud — a round clump of leaves dropped at the tip of a branch.
 * Stored as a centre and a horizontal radius; the drawing code (§6) makes
 * the clump look round on screen by squashing it vertically to undo the
 * tall-cell distortion.
 */
typedef struct {
    float    cx, cy;       /* centre, in cell coordinates                        */
    float    radius;       /* horizontal radius in cells (vertical is smaller)   */
    int      depth_at_tip; /* how deep the tree was here; used for the snow tip   */
} FoliageCloud;

/*
 * Tree — the whole skeleton: every trunk/branch segment plus every leaf
 * clump.  Built once by §5, then drawn by §6.  The arrays are fixed-size
 * (no malloc); if the tree somehow grows past the limit, seg_add/fol_add
 * just stop adding instead of overflowing, so it fails gracefully.
 */
typedef struct {
    Segment      segs[SEG_MAX];   /* all trunk and branch pieces           */
    int          n_segs;          /* how many of segs[] are in use         */
    FoliageCloud fols[FOL_MAX];   /* all the leaf clumps                    */
    int          n_fols;          /* how many of fols[] are in use         */
} Tree;

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

/* Turn a direction by `bend` radians, used to angle a child branch away
 * from the one it grew from. */
static void rotate_dir(float dx, float dy, float bend, float *out_x, float *out_y)
{
    float c = cosf(bend), s = sinf(bend);
    *out_x = dx * c - dy * s;
    *out_y = dx * s + dy * c;
}

/*
 * grow_branch — grow one branch and then its children.  It draws a single
 * straight piece in the given direction, then either splits into smaller
 * branches or, when it's run out of length or depth, drops a leaf clump at
 * the end.  It calls itself for each child, so the whole tree comes from
 * this one function.
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

    /* Squash the vertical step so the branch covers the same on-screen
     * distance no matter which way it points. */
    float ex = x + dir_x * length;
    float ey = y + dir_y * length / CELL_ASPECT;
    seg_add(t, x, y, ex, ey, depth);

    /* Either keep going as one branch or fork into two. */
    int n_subs = 1;
    if (lcg_unit(rng) < sp->branch_density && depth < sp->max_depth - 1) {
        n_subs = 2;
    }
    if (depth == 0 && lcg_unit(rng) < 0.6f) n_subs = 2;

    for (int i = 0; i < n_subs; i++) {
        float bend = lcg_range(rng, -0.40f, 0.40f);
        if (n_subs > 1) {
            /* When forking, send the two children off to opposite sides. */
            bend = (i == 0 ? -1 : +1) * lcg_range(rng, 0.5f, 0.95f);
        }
        float ndx, ndy;
        rotate_dir(dir_x, dir_y, bend, &ndx, &ndy);
        /* Nudge non-trunk branches upward so they reach for the light. */
        if (depth > 0 && ndy > -0.1f) ndy -= 0.2f;
        float nlen = length * BRANCH_LEN_F;
        grow_branch(t, rng, ex, ey, ndx, ndy, nlen, depth + 1, sp, no_foliage);
    }
}

/* Which row the top of the pot sits on.  Normally we leave a few rows at
 * the bottom for the pot, but on a very short screen we shove it to the
 * last row so the tree still has somewhere to grow. */
static int pot_top_row(int rows_eff)
{
    int pot_rows = (rows_eff >= POT_TALL_MIN_ROWS) ? POT_ROWS_TALL : POT_ROWS_SHORT;
    int pot_top  = rows_eff - pot_rows - 1;
    if (pot_top < 5) pot_top = rows_eff - 1;     /* no room above: pot at the floor */
    return pot_top;
}

/*
 * trunk_x_at — where the trunk sits side-to-side at a given height (frac
 * runs 0 at the base to 1 at the top).  This is the one place each style's
 * trunk shape lives: upright, S-curved, leaning, or cascading.  KENGAI also
 * pushes the trunk down past the pot rim, so it can change *y too.  CHOKKAN
 * uses the random generator for its tiny wobble, so call this once per node
 * in order to keep a given seed repeatable.
 */
static float trunk_x_at(uint32_t *rng, TreeStyle style, const StyleParams *sp,
                        float mid, float frac, float trunk_height,
                        int pot_top, float *y)
{
    switch (style) {
    case STYLE_CHOKKAN:                          /* straight up, tiny wobble */
        return mid + lcg_range(rng, -0.3f, 0.3f);
    case STYLE_MOYOGI:                           /* gentle S-curve */
        return mid + sp->trunk_curve_amp
                   * sinf(2.0f * (float)M_PI * sp->trunk_curve_waves * frac);
    case STYLE_SHAKAN:                           /* leans at a fixed angle */
        return mid + sp->trunk_lean * frac * trunk_height;
    case STYLE_KENGAI: {                          /* rises, then cascades down */
        float apex = KENGAI_APEX_FRAC;
        if (frac < apex) return mid;             /* short straight rise first */
        float t2 = (frac - apex) / (1.0f - apex);
        *y = (float)pot_top - apex * trunk_height + t2 * 6.0f;   /* arc down past the rim */
        return mid + 6.0f * sinf(t2 * (float)M_PI * 0.6f) * 1.5f;/* and out to the side */
    }
    case STYLE_BUNJIN:                            /* sparse, slightly shifted S */
    default:
        return mid + sp->trunk_curve_amp
                   * sinf(2.0f * (float)M_PI * sp->trunk_curve_waves * frac + 0.5f);
    }
}

/*
 * maybe_sprout_branch — at one point up the trunk, sometimes start a side
 * branch.  It only happens in the middle stretch of the trunk (so the base
 * and the very top stay bare) and then only by chance, so denser styles get
 * more branches.  The branch always angles upward.
 */
static void maybe_sprout_branch(Tree *t, uint32_t *rng, float x, float y,
                                float frac, const StyleParams *sp, bool no_foliage)
{
    if (frac <= BRANCH_BAND_LO || frac >= BRANCH_BAND_HI) return;
    if (lcg_unit(rng) >= sp->branch_density)              return;

    float angle = (lcg_unit(rng) < 0.5f ? -1.0f : 1.0f) * lcg_range(rng, 0.7f, 1.2f);
    float dx = sinf(angle);
    float dy = -fabsf(cosf(angle));              /* the minus keeps it pointing up */
    float blen = sp->branch_length * (1.0f - frac * 0.5f);   /* shorter near the top */
    grow_branch(t, rng, x, y, dx, dy, blen, 1, sp, no_foliage);
}

/* Put a leaf clump at the very top of the trunk: a big one for upright
 * trees, a smaller one for the cascading KENGAI tip, none if the theme is
 * bare (WINTER). */
static void add_crown(Tree *t, TreeStyle style, const StyleParams *sp,
                      float tip_x, float tip_y, bool no_foliage)
{
    if (no_foliage) return;
    if (style == STYLE_KENGAI)
        fol_add(t, tip_x, tip_y,        sp->foliage_size * 1.2f, sp->max_depth);
    else
        fol_add(t, tip_x, tip_y - 0.5f, sp->foliage_size * 1.4f, sp->max_depth);
}

/*
 * tree_generate — build a whole tree from a seed: work out the pot and
 * trunk size, walk up the trunk dropping side branches, and cap it with a
 * crown.  Returns the row the pot's top edge sits on so the pot can be
 * drawn afterward.
 */
static int tree_generate(Tree *t, uint32_t seed, int cols, int rows_eff,
                         TreeStyle style, bool no_foliage)
{
    uint32_t rng = seed;
    tree_clear(t);
    const StyleParams *sp = &styles[style];

    /* Work out how tall the trunk is, keeping it within sane bounds. */
    int   pot_top      = pot_top_row(rows_eff);
    float trunk_height = (float)pot_top * sp->trunk_height_frac;
    if (trunk_height > (float)pot_top - 1.0f) trunk_height = (float)pot_top - 1.0f;
    if (trunk_height < MIN_TRUNK_HEIGHT)      trunk_height = MIN_TRUNK_HEIGHT;
    float mid = (float)cols * 0.5f;

    /* Taller trunks are drawn from more nodes (smoother), within limits. */
    int trunk_segs = (int)(trunk_height * TRUNK_SEGS_PER_CELL);
    if (trunk_segs < TRUNK_SEGS_MIN) trunk_segs = TRUNK_SEGS_MIN;
    if (trunk_segs > TRUNK_SEGS_MAX) trunk_segs = TRUNK_SEGS_MAX;

    /* Step up the trunk node by node: place the node where the style wants
     * it, draw the segment to it, and maybe start a branch there. */
    float prev_x = mid, prev_y = (float)pot_top;
    for (int i = 1; i <= trunk_segs; i++) {
        float frac = (float)i / (float)trunk_segs;
        float y    = (float)pot_top - frac * trunk_height;
        float x    = trunk_x_at(&rng, style, sp, mid, frac, trunk_height,
                                pot_top, &y);

        seg_add(t, prev_x, prev_y, x, y, 0);
        maybe_sprout_branch(t, &rng, x, y, frac, sp, no_foliage);

        prev_x = x;
        prev_y = y;
    }

    add_crown(t, style, sp, prev_x, prev_y, no_foliage);
    return pot_top;
}

/* ── §6 raster — turn the skeleton into a grid of characters ── */

/*
 * CharGrid — the finished picture, baked once into a grid of characters
 * that the renderer just replays every frame.  §6/§7 write it; §11 reads
 * it.  It's stored as three separate planes (one big array each) rather
 * than one array of structs, so each attribute sits contiguously in memory:
 *   glyph — the character to print in the cell
 *   role  — what the cell is (bark / leaf / pot / ...); picks the colour
 *   pair  — which of a role's colours to use (0..5 for the six leaf hues)
 * The arrays are sized for the biggest terminal we support; only the part
 * matching the real window is ever touched.
 */
typedef struct {
    char    glyph[GRID_MAX_H][GRID_MAX_W];
    uint8_t role [GRID_MAX_H][GRID_MAX_W];
    uint8_t pair [GRID_MAX_H][GRID_MAX_W];
} CharGrid;

static void grid_clear(CharGrid *grid, int rows_eff, int cols)
{
    for (int r = 0; r < rows_eff && r < GRID_MAX_H; r++)
        for (int c = 0; c < cols && c < GRID_MAX_W; c++) {
            grid->glyph[r][c] = ' ';
            grid->role [r][c] = ROLE_EMPTY;
            grid->pair [r][c] = 0;
        }
}

static inline void grid_put(CharGrid *grid, int r, int c, char ch, uint8_t role,
                            uint8_t pair_idx, int rows_eff, int cols)
{
    if (r < 0 || r >= rows_eff || r >= GRID_MAX_H) return;
    if (c < 0 || c >= cols     || c >= GRID_MAX_W) return;
    /* If bark is already here, don't let a leaf paint over it. */
    uint8_t existing = grid->role[r][c];
    if (existing >= ROLE_TRUNK_DARK && existing <= ROLE_TRUNK_LIGHT
        && role == ROLE_FOLIAGE) return;
    grid->glyph[r][c] = ch;
    grid->role [r][c] = role;
    grid->pair [r][c] = pair_idx;
}

/*
 * bark_tier — given how deep a branch is, pick its bark thickness: which
 * colour to use and a character from the matching set (thick / medium /
 * thin).  The `sample` number just picks one character so a given cell
 * always looks the same.
 */
static void bark_tier(int depth, int sample, char *glyph,
                      uint8_t *role, uint8_t *pair_idx)
{
    if (depth <= 1) {
        *glyph    = TRUNK_THICK[((unsigned)sample) % (sizeof TRUNK_THICK - 1)];
        *role     = ROLE_TRUNK_DARK;
        *pair_idx = 0;
    } else if (depth <= 3) {
        *glyph    = TRUNK_MEDIUM[((unsigned)sample) % (sizeof TRUNK_MEDIUM - 1)];
        *role     = ROLE_TRUNK_MID;
        *pair_idx = 1;
    } else {
        *glyph    = TRUNK_THIN[((unsigned)sample) % (sizeof TRUNK_THIN - 1)];
        *role     = ROLE_TRUNK_LIGHT;
        *pair_idx = 2;
    }
}

/*
 * draw_segment — stamp one trunk/branch piece onto the grid, cell by cell,
 * using Bresenham's line algorithm (a classic integer way to draw a
 * straight line between two grid points).
 */
static void draw_segment(const Segment *s, CharGrid *grid, int rows_eff, int cols)
{
    int x0 = (int)(s->x0 + 0.5f);
    int y0 = (int)(s->y0 + 0.5f);
    int x1 = (int)(s->x1 + 0.5f);
    int y1 = (int)(s->y1 + 0.5f);

    char    glyph;
    uint8_t role;
    uint8_t pair_idx;
    bark_tier(s->depth, abs(x0 + y0), &glyph, &role, &pair_idx);

    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    while (1) {
        /* Pick a character that leans the way the line is going, so a
         * diagonal branch reads as a slash rather than a stack of bars. */
        char g = glyph;
        if (dx > dy * 2)        g = (s->depth <= 1) ? '=' : '-';
        else if (dy > dx * 2)   g = (s->depth <= 1) ? 'H' : '|';
        else if (sx == sy)      g = (s->depth <= 1) ? 'M' : '\\';
        else if (sx != sy)      g = (s->depth <= 1) ? 'M' : '/';
        grid_put(grid, y0, x0, g, role, pair_idx, rows_eff, cols);

        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/*
 * draw_foliage_cloud — fill in one leaf clump.  It walks the cells inside
 * a circle (squashed vertically so it looks round on screen) and drops a
 * random leaf character from the theme in each one.
 */
static void draw_foliage_cloud(const FoliageCloud *f, const Theme *th,
                                uint32_t *rng, CharGrid *grid,
                                int rows_eff, int cols)
{
    int cxi = (int)(f->cx + 0.5f);
    int cyi = (int)(f->cy + 0.5f);
    float rx = f->radius;
    float ry = f->radius / CELL_ASPECT;     /* smaller vertically so it looks round */

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
            if (r2 > 1.0f) continue;           /* outside the circle */

            /* Thin the leaves out toward the edge so the clump looks soft. */
            if (r2 > 0.6f && lcg_unit(rng) > 0.7f) continue;

            char g = th->leaf_set[lcg_next(rng) % (uint32_t)leaf_set_len];
            uint8_t pair_idx = (uint8_t)(lcg_next(rng) % 6u);
            grid_put(grid, y, x, g, ROLE_FOLIAGE, pair_idx, rows_eff, cols);
        }
    }

    /* In WINTER, dab a bit of snow on top of each clump. */
    if (th->snow_tip != NULL) {
        int top = yr_lo + 1;
        if (top >= 0 && top < rows_eff && cxi >= 0 && cxi < cols) {
            grid_put(grid, top, cxi, th->snow_tip[0], ROLE_SNOW_TIP, 5,
                     rows_eff, cols);
        }
    }
}

/* ── §7 pot — draw the pot under the tree ── */

/*
 * draw_pot — a small pot centred under the trunk, about a third of the
 * screen wide.  Drawn last so it covers any trunk that dipped down into its
 * cells (the cascading KENGAI style does this).
 */
static void draw_pot(CharGrid *grid, int pot_top, int rows_eff, int cols)
{
    int w = cols / 3;
    if (w < 12) w = 12;
    if (w > cols - 4) w = cols - 4;
    int x0 = cols / 2 - w / 2;
    int x1 = x0 + w - 1;

    /* The rim, which sticks out a little past the walls on each side. */
    int rim_y = pot_top + 1;
    if (rim_y >= 0 && rim_y < rows_eff) {
        for (int x = x0 - 1; x <= x1 + 1; x++) {
            if (x < 0 || x >= cols) continue;
            char c = POT_TOP_CH;
            if (x == x0 - 1)      c = POT_LIP_LEFT;
            else if (x == x1 + 1) c = POT_LIP_RIGHT;
            grid_put(grid, rim_y, x, c, ROLE_POT, 0, rows_eff, cols);
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
            /* The solid bottom row. */
            for (int x = x0; x <= x1; x++) {
                if (x < 0 || x >= cols) continue;
                char c = POT_BOTTOM_CH;
                if (x == x0)      c = '\\';
                else if (x == x1) c = '/';
                grid_put(grid, y, x, c, ROLE_POT, 0, rows_eff, cols);
            }
        } else {
            /* The two side walls. */
            if (x0 >= 0 && x0 < cols)
                grid_put(grid, y, x0, POT_SIDE_CH, ROLE_POT, 0, rows_eff, cols);
            if (x1 >= 0 && x1 < cols)
                grid_put(grid, y, x1, POT_SIDE_CH, ROLE_POT, 0, rows_eff, cols);
            /* Scatter a few dots inside for a hint of soil texture. */
            for (int x = x0 + 1; x <= x1 - 1; x++) {
                if (x < 0 || x >= cols) continue;
                if (((unsigned)(x + y)) & 3u) continue;   /* keep about one in four */
                grid_put(grid, y, x, '.', ROLE_POT, 0, rows_eff, cols);
            }
        }
    }
}

/* ── §8 build — assemble one tree and hold all its state ── */

/*
 * Scene — everything about the one bonsai on screen.  The first group is
 * the finished tree (rebuilt whenever something changes); the second group
 * is the choices that decide which tree to grow (a seed plus a style plus a
 * theme fully determines it); the last group is the window size and the
 * rustle clock.  scene_rebuild regrows the tree from the choices; the
 * per-frame tick only nudges `time` forward.  Scene keeps its own cols/rows
 * so the tree-building code never has to ask the terminal directly.
 */
typedef struct {
    /* The finished tree (rebuilt by scene_rebuild on any change). */
    Tree       tree;          /* the skeleton (§5)                            */
    CharGrid   canvas;        /* the baked picture the renderer replays       */
    int        pot_top;       /* row the pot's top edge sits on               */

    /* The choices that decide which tree to grow. */
    uint32_t   seed;          /* picks one specific tree of the chosen style  */
    int        current_style; /* which trunk shape (a TreeStyle)              */
    int        current_theme; /* which colours + leaves (a Theme index)       */

    /* Window size and timing. */
    int        cols, rows;    /* window size in cells (own copy)              */
    float      time;          /* seconds elapsed, drives the rustle           */
    bool       paused;        /* when true, the rustle freezes                */
} Scene;

static void scene_rebuild(Scene *s)
{
    int rows_eff = s->rows - 1;
    bool no_foliage = themes[s->current_theme].bare;

    s->pot_top = tree_generate(&s->tree, s->seed, s->cols, rows_eff,
                                (TreeStyle)s->current_style, no_foliage);

    grid_clear(&s->canvas, rows_eff, s->cols);

    /* Draw segments (trunk + branches). */
    for (int i = 0; i < s->tree.n_segs; i++)
        draw_segment(&s->tree.segs[i], &s->canvas, rows_eff, s->cols);

    /* Draw foliage. */
    const Theme *th = &themes[s->current_theme];
    uint32_t fol_rng = s->seed ^ 0xDEC0DEu;
    for (int i = 0; i < s->tree.n_fols; i++)
        draw_foliage_cloud(&s->tree.fols[i], th, &fol_rng, &s->canvas,
                           rows_eff, s->cols);

    /* Draw pot. */
    draw_pot(&s->canvas, s->pot_top, rows_eff, s->cols);
}

/* ── §9 events — keypress and resize handlers that rebuild the tree ── */
/*
 * These run only when the user presses a key or resizes the window, never
 * as part of the normal frame loop.  Each changes the Scene and rebuilds
 * the tree; switching theme also re-binds the colours (§3).
 */

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
    scene_rebuild(s);     /* rebuild because WINTER adds/removes the leaves */
}

/* ── §10 simulation — advance the rustle clock ── */
/*
 * The tree itself never moves; the only thing that changes over time is the
 * clock that drives the leaf flicker.  When paused, even that stops.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->time += dt;
}

/* ── §11 render — draw the baked grid to the screen ── */
/*
 * This only reads the baked picture and writes to the screen; it never
 * changes the tree.  The wind flicker is worked out here on the fly (not
 * stored anywhere) by re-picking a leaf character for some cells.
 */

/* cell_style — pick the colour and emphasis for a cell from what it is.
 * Bark gets brighter the closer to the trunk; the lighter leaf colours are
 * drawn bold so the canopy has some sparkle. */
static void cell_style(uint8_t role, uint8_t pair_idx, int *pair, attr_t *attr)
{
    switch (role) {
    case ROLE_TRUNK_DARK:
        *pair = PAIR_TRUNK_BASE + 0; *attr = A_BOLD;   break;
    case ROLE_TRUNK_MID:
        *pair = PAIR_TRUNK_BASE + 1; *attr = A_NORMAL; break;
    case ROLE_TRUNK_LIGHT:
        *pair = PAIR_TRUNK_BASE + 2; *attr = A_DIM;    break;
    case ROLE_FOLIAGE:
        *pair = PAIR_FOLIAGE_BASE + (pair_idx % 6);
        *attr = (pair_idx >= FOLIAGE_BOLD_FROM) ? A_BOLD : A_NORMAL;
        break;
    case ROLE_SNOW_TIP:
        *pair = PAIR_FOLIAGE_BASE + 5; *attr = A_BOLD; break;
    case ROLE_POT:
    default:
        *pair = PAIR_POT; *attr = A_NORMAL; break;
    }
}

/* rustle_glyph — the wind flicker.  A hash of the cell and the current time
 * window decides whether this leaf "flutters" right now: a small fraction do,
 * and those swap to a different random leaf; the rest keep their baked one. */
static char rustle_glyph(char baked, const Theme *th, int c, int r,
                         int time_key, int leaf_len)
{
    uint32_t h = hash3(c, r, time_key);
    if ((h & 0xFFu) < (uint32_t)(255.0f * RUSTLE_RATE))
        return th->leaf_set[(h >> 8) % (uint32_t)leaf_len];
    return baked;
}

/* scene_render — print every non-empty cell of the baked grid in its
 * colour, letting the leaves flicker as it goes. */
static void scene_render(const Scene *s)
{
    int rows_eff = s->rows - 1;
    if (rows_eff < 1) return;
    int cols = s->cols;
    if (rows_eff > GRID_MAX_H) rows_eff = GRID_MAX_H;
    if (cols     > GRID_MAX_W) cols     = GRID_MAX_W;

    const Theme *th       = &themes[s->current_theme];
    int          time_key = (int)(s->time * RUSTLE_FREQ);
    int          leaf_len = (int)strlen(th->leaf_set);

    /* Only switch ncurses colour when it actually changes, so a long run of
     * the same colour costs one switch instead of one per cell. */
    int    last_pair = -1;
    attr_t last_attr = 0;
    for (int r = 0; r < rows_eff; r++) {
        for (int c = 0; c < cols; c++) {
            uint8_t role = s->canvas.role[r][c];
            if (role == ROLE_EMPTY) {
                if (last_pair >= 0) {
                    attroff(COLOR_PAIR(last_pair) | last_attr);
                    last_pair = -1;
                }
                continue;
            }

            char   glyph = s->canvas.glyph[r][c];
            int    pair;
            attr_t attr;
            cell_style(role, s->canvas.pair[r][c], &pair, &attr);
            if (role == ROLE_FOLIAGE)
                glyph = rustle_glyph(glyph, th, c, r, time_key, leaf_len);

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

/* ── §12 screen — terminal setup/teardown and the HUD ── */
/*
 * Owns the ncurses lifecycle (start, resize, stop) and draws the two HUD
 * bars on top of the tree.
 */

/*
 * Screen — the current terminal size in cells, re-read at startup and on
 * every resize.  Scene keeps its own copy of the size; the two are synced
 * when the window resizes.
 */
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

/*
 * screen_draw — draw the tree, then put the HUD on top: a status line along
 * the top (title, state, style, theme, and fps/Hz on the right) and the key
 * legend along the bottom.  Both bars are filled with spaces first so the
 * tree doesn't peek through, and every piece of text is clipped to the
 * window width so a narrow terminal can't overflow or wrap it.
 */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_render(s);

    int cols = sc->cols;
    int rows = sc->rows;
    if (cols < 1 || rows < 1) return;

    /* Top line: status on the left, fps/Hz pinned to the right. */
    char data[128], stats[40];
    snprintf(data, sizeof data, " BONSAI   %s   style:%s  theme:%s ",
             s->paused ? "PAUSED" : "RUSTLE",
             styles[s->current_style].name,
             themes[s->current_theme].name);
    snprintf(stats, sizeof stats, " %5.1f fps  %3d Hz ", fps, sim_fps);
    int sx = cols - (int)strlen(stats);          /* column where the stats begin */

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < cols; x++) mvaddch(0, x, ' ');
    if (sx >= 0) {
        mvprintw(0, 0, "%.*s", sx, data);        /* stop the status short of the stats */
        mvprintw(0, sx, "%s", stats);
    } else {
        mvprintw(0, 0, "%.*s", cols, data);      /* window too narrow for stats */
    }
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom line: the list of keys you can press. */
    if (rows >= 2) {
        int brow = rows - 1;
        attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
        for (int x = 0; x < cols; x++) mvaddch(brow, x, ' ');
        mvprintw(brow, 0, "%.*s", cols,
                 " n/N:style  t/T:theme  r:reseed  [/]:Hz  spc:pause  q:quit ");
        attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    }
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §13 app + main — wire it together and run the frame loop ── */

/*
 * App — the running program: the tree, the terminal, and the loop flags.
 * sim_fps is just a frame-rate cap (the [ and ] keys); it doesn't change how
 * the rustle advances, since that uses real elapsed time.  running and
 * need_resize are set inside signal handlers, so they're volatile
 * sig_atomic_t — the one type C promises is safe to poke from a handler and
 * read back in the loop without surprises.
 */
typedef struct {
    Scene                 scene;       /* the bonsai (§8)                      */
    Screen                screen;      /* terminal size (§12)                  */
    int                   sim_fps;     /* frame-rate cap, SIM_FPS_MIN..MAX     */
    volatile sig_atomic_t running;     /* set to 0 to quit (Ctrl-C or 'q')     */
    volatile sig_atomic_t need_resize; /* set to 1 by a window resize          */
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
        /* Handle a pending resize before timing this frame. */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
        }

        /* How long since the last frame, capped so a hiccup can't make the
         * rustle jump. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > MAX_FRAME_MS * NS_PER_MS) dt = MAX_FRAME_MS * NS_PER_MS;

        /* Move the rustle clock forward. */
        float dt_sec = (float)dt / (float)NS_PER_SEC;
        scene_tick(&app->scene, dt_sec);

        /* Update the fps reading a couple of times a second. */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* Sleep to hold the frame rate, then draw. */
        int64_t target_ns = TICK_NS(app->sim_fps);
        int64_t elapsed   = clock_ns() - frame_time + dt;
        clock_sleep_ns(target_ns - elapsed);
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* Read one keypress, if any. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
