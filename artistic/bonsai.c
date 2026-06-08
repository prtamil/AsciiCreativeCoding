/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bonsai.c — bonsai tree gallery: 5 classical styles, 6 themes,
 *            static skeleton + subtle wind rustle on foliage.
 *
 * Section map  (re-cut into single-concern layers — see ARCHITECTURE below):
 *   §1  config       — constants, style + theme tables, glyph palettes
 *   §2  clock        — PERFORMANCE: monotonic timer + sleep
 *   §3  color        — RENDER setup: theme → colour-pair binding
 *   §4  random       — LOGIC: pure RNG + hashes (no global mutation, no I/O)
 *   §5  generation   — BUILD: recursive tree skeleton
 *   §6  raster       — BUILD/bake: skeleton → char grid (Bresenham + ellipse)
 *   §7  pot          — BUILD/bake: pot frame
 *   §8  build        — BUILD orchestration: scene_rebuild + Scene state
 *   §9  events       — USER EVENTS: init/resize/reseed/cycle (NOT the tick)
 *   §10 simulation   — SIMULATION: scene_tick (the only per-tick state advance)
 *   §11 render       — RENDER: scene_render (grid → screen, reads only)
 *   §12 screen       — RENDER: terminal I/O + HUD
 *   §13 app + main   — USER-EVENT glue + PERFORMANCE per-frame combine
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
 *                     Stop when length < MIN_LEN or depth > the style's max_depth;
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
 * Data-structure : Both live on the Scene aggregate.  `Tree` holds segs[]
 *                  (trunk + branches as line segments with depth) and fols[]
 *                  (foliage clouds with centre/radius), sized for the gnarliest
 *                  recursion (SEG_MAX=384, FOL_MAX=96).  `CharGrid` is the baked
 *                  image: parallel glyph / role / pair planes (struct-of-arrays)
 *                  the renderer replays each frame.
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
 *   Bonsai form & botanical structure (the CONCEPTS) —
 *   • Naka, J. Y. — *Bonsai Techniques I & II* (1973, 1982).  The
 *     classical-style taxonomy (chokkan / moyogi / shakan / kengai /
 *     bunjin) the StyleParams table encodes.
 *   • Iwata, Y. — *Bonsai: A Patient Art* (2009).  Photographic
 *     reference for how trunks curve, branches space, foliage clumps.
 *   • Prusinkiewicz, P. & Lindenmayer, A. — *The Algorithmic Beauty of
 *     Plants* (Springer, 1990; free PDF at algorithmicbotany.org).  THE
 *     reference for procedural trees; our recursive grow_branch() is a
 *     stochastic, non-rewriting cousin of its bracketed L-systems.
 *   • Lindenmayer, A. — "Mathematical models for cellular interactions
 *     in development", *J. Theor. Biol.* 18 (1968).  The L-system origin
 *     paper; the branch-then-subdivide pattern descends from it.
 *
 *   Rasterisation & rendering (the RENDERING part) —
 *   • Bresenham, J. E. — "Algorithm for computer control of a digital
 *     plotter", *IBM Systems Journal* 4(1), 1965.  The integer line walk
 *     in draw_segment() that stamps trunk/branch glyphs cell by cell.
 *   • Foley, van Dam, Feiner & Hughes — *Computer Graphics: Principles
 *     and Practice* (2nd ed., 1990), ch. 3.  Scan-conversion of lines
 *     and the midpoint ELLIPSE — the model behind the aspect-corrected
 *     foliage fill in draw_foliage_cloud().
 *   • Bourke, P. — "Character representation of grey-scale images"
 *     (paulbourke.net, 1997).  The ASCII-ramp idea: map structure and
 *     density to a glyph set (thick→thin trunk ramps, foliage density
 *     chars) — the basis of all text-mode rendering here.
 *   • Press, Teukolsky, Vetterling & Flannery — *Numerical Recipes*
 *     (3rd ed., 2007), ch. 7.  The LCG in lcg_next() uses this book's
 *     multiplier / increment (1664525 / 1013904223) for the
 *     deterministic, reproducible per-seed tree.
 *   • Jarzynski, M. & Olano, M. — "Hash Functions for GPU Rendering",
 *     *JCGT* 9(3), 2020.  Integer bit-mixing hashes of the kind hash3()
 *     uses to gate the stateless, per-cell wind rustle.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (separated layers) ──────────────────────────────────── *
 *
 * This program has TWO time axes, not one:
 *
 *   A. BUILD pipeline — runs ONLY on a user event (init / reseed / style /
 *      theme / resize), never per frame.  GENERATION lays out the skeleton,
 *      RASTER bakes it into the static char grid.  The result is a frozen
 *      still-life the render layer then replays every frame.
 *   B. Per-frame TICK — advance the rustle clock (SIMULATION), then RENDER the
 *      baked grid with a derived wind flutter.
 *
 * The six canonical concerns, as they actually appear here:
 *
 *   LAYER        SECTION                 MUTATES
 *   ─────        ───────                 ───────
 *   LOGIC        §4  random              nothing global — pure RNG (transforms
 *                                        the caller's *st) + hash3 (pure).  No I/O.
 *   (BUILD)      §5  generation          Scene.tree   (segments + foliage clouds)
 *   (BUILD)      §6  raster, §7 pot      Scene.canvas (CharGrid: glyph/role/pair)
 *   (BUILD)      §8  build               Scene.tree + Scene.canvas + Scene.pot_top
 *   SIMULATION   §10 simulation          Scene.time   (the ONLY per-tick advance)
 *   RENDER       §3  color (setup),      the ncurses screen + colour pairs ONLY;
 *                §11 render, §12 screen  reads Scene.canvas + Scene.time, writes none
 *   PERFORMANCE  §2  clock, §13 main     frame timing / fps / sleep — no sim state
 *
 *   EFFECTS — NONE STORED.  The wind rustle is re-derived every frame inside
 *             scene_render from hash3(col,row,⌊time·RUSTLE_FREQ⌋); the chosen
 *             leaf glyph is a LOCAL, never written back to the grid.  There is
 *             no cosmetic state to advance or corrupt, so no EFFECTS layer.
 *   DELAYS  — TRIVIAL.  The only "timer" is Scene.paused, a boolean gate at the
 *             top of scene_tick; no holds, cooldowns, or staged delays exist.
 *
 * PER-TICK COMBINE — main() (§13) is the ONE place the per-frame layers meet,
 * in this fixed order; nothing else advances Scene.time:
 *     (resize?) → measure dt (capped) → scene_tick (SIM) → fps sample
 *               → sleep to frame cap (PERF) → screen_draw + present (RENDER)
 *               → drain one input event (USER EVENT)
 *
 * USER EVENTS (NOT part of the tick) mutate state and may trigger a full BUILD,
 * but fire only on input / resize — see §9 and app_handle_key (§13):
 *     space        Scene.paused
 *     r            reseed  → BUILD
 *     n/N, t/T     style / theme → BUILD (theme also re-binds colours)
 *     [ / ]        App.sim_fps  (the frame-cap target)
 *     SIGWINCH     resize → BUILD
 *
 * (The const-pointer read/mutate signature convention is deferred to step 5,
 *  where the types exist to express it; here the roles are documented only.)
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

    /* Skeleton pool sizes. */
    SEG_MAX           = 384,
    FOL_MAX           = 96,

    /* Char grid maximum (for static buffers). */
    GRID_MAX_W        = 280,
    GRID_MAX_H        = 90,

    /* Pot placement (pot_top_row). */
    POT_TALL_MIN_ROWS = 18,    /* rows_eff ≥ this → a 4-row pot, else 3       */
    POT_ROWS_TALL     =  4,
    POT_ROWS_SHORT    =  3,

    /* Trunk poly-line resolution clamp (tree_generate). */
    TRUNK_SEGS_MIN    =  6,
    TRUNK_SEGS_MAX    = 30,

    FOLIAGE_BOLD_FROM =  4,     /* foliage pair index ≥ this → drawn A_BOLD   */
    MAX_FRAME_MS      = 100,    /* clamp a long dt so a stall can't spiral    */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define CELL_ASPECT      2.0f      /* terminal cell h/w                  */

/* Tree generation — global tunables.  (Recursion depth is per-style:
 * StyleParams.max_depth, not a global cap.) */
#define MIN_LEN          1.5f      /* segments shorter than this stop    */
#define BRANCH_LEN_F     0.66f     /* child length = parent · this       */

/* Trunk shape (tree_generate / trunk_x_at). */
#define MIN_TRUNK_HEIGHT     4.0f  /* never shorter than this (cells)        */
#define TRUNK_SEGS_PER_CELL  1.8f  /* poly-line nodes per cell of height     */
#define BRANCH_BAND_LO       0.30f /* sprout branches only within this band  */
#define BRANCH_BAND_HI       0.92f /*   of trunk height (clean base + apex)  */
#define KENGAI_APEX_FRAC     0.30f /* cascade: trunk rises to here, then arcs */

/* Wind rustle. */
#define RUSTLE_FREQ      6.0f      /* rustle hash key advances at this Hz */
#define RUSTLE_RATE      0.06f     /* fraction of foliage cells per tick  */

/* ── Style enum + per-style params ── */

/*
 * TreeStyle — the five classical bonsai styles, the taxonomy a grower would
 * name (Naka, *Bonsai Techniques I & II*).  Each value indexes the styles[]
 * table of StyleParams; the visible difference between styles is entirely the
 * trunk-path branch of tree_generate() (§5) plus that row of knobs.
 * N_STYLES doubles as the array length and the modulus for n/N cycling — it
 * MUST stay last so it equals the real style count.
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
 * StyleParams — the per-style generation recipe: the numbers that bend ONE
 * recursive-descent generator (grow_branch / tree_generate, §5) into each
 * TreeStyle.  styles[] holds one row per style, indexed by TreeStyle.  This is
 * "parametric" in the sense of Prusinkiewicz & Lindenmayer, *The Algorithmic
 * Beauty of Plants* — identical production rules, different parameters yield
 * different silhouettes.  Several fields are read only by one style (noted).
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

/* ── Theme ── */

/*
 * Theme — a season/mood: the colours and the leaf alphabet a tree is painted
 * in.  Indexed by Scene.current_theme into themes[]; a theme change re-binds
 * the colour pairs (theme_apply, §3) AND forces a rebuild, because `bare`
 * changes the skeleton, not just the palette.  Colour fields are xterm-256
 * indices, all kept in the bright half of the cube so the tree stays legible on
 * the dark background (project palette rule).  Every theme is foreground-on-dark
 * — MONO is the achromatic one (white/grey bark + foliage).
 */
typedef struct {
    const char *name;            /* HUD label, space-padded                     */
    short       trunk[3];        /* bark ramp by depth: dark / mid / light      */
    short       foliage[6];      /* 6 leaf hues — variety so a cloud isn't flat */
    short       pot;             /* pot colour                                  */
    bool        bare;            /* WINTER: no foliage clouds (branches only)   */
    const char *leaf_set;        /* glyph alphabet each foliage cell draws from */
    const char *snow_tip;        /* WINTER: snow glyph atop clouds, else NULL   */
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

/* Glyph palettes. */
static const char TRUNK_THICK[]    = "MHHM[]()";   /* depth 0-1, thick bark   */
static const char TRUNK_MEDIUM[]   = "|[]/\\";       /* depth 2-3, medium       */
static const char TRUNK_THIN[]     = "|/\\";         /* depth 4+, thin twigs    */

#define POT_TOP_CH      '_'
#define POT_SIDE_CH     '|'
#define POT_LIP_LEFT    '\\'
#define POT_LIP_RIGHT   '/'
#define POT_BOTTOM_CH   ':'

/*
 * CellRole — what a baked CharGrid cell represents.  Written at build time
 * (§6/§7), read at render (§11) to pick the colour pair + attribute.  Two
 * ordering invariants the code relies on:
 *   - ROLE_EMPTY = 0, so a zeroed grid reads as "nothing here".
 *   - ROLE_TRUNK_DARK..ROLE_TRUNK_LIGHT are contiguous, so grid_put() can test
 *     "is this a trunk cell?" with one range check (trunk wins over foliage).
 */
typedef enum {
    ROLE_EMPTY = 0,       /* background — nothing drawn                     */
    ROLE_TRUNK_DARK,      /* depth 0-1 bark (thick)  — trunk ramp index 0   */
    ROLE_TRUNK_MID,       /* depth 2-3 bark (medium) — index 1              */
    ROLE_TRUNK_LIGHT,     /* depth 4+ twigs (thin)   — index 2              */
    ROLE_FOLIAGE,         /* leaf cell — one of 6 theme hues, may rustle    */
    ROLE_SNOW_TIP,        /* WINTER only: snow glyph atop a foliage cloud   */
    ROLE_POT,             /* pot frame                                      */
} CellRole;

/* ===================================================================== */
/* §2  clock                       PERFORMANCE — monotonic time + sleep   */
/* ===================================================================== */
/* Timing primitives only.  Mutate nothing; read by the §13 main loop.    */

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
/* §3  color                       RENDER setup — theme → colour pairs    */
/* ===================================================================== */
/* Mutates ncurses colour pairs only (theme change / boot).  No sim state. */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *t = &themes[idx];

    /* Every theme is foreground-on-default-background (dark). */
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

/* ===================================================================== */
/* §4  random                      LOGIC — pure RNG + cheap hashes        */
/* ===================================================================== */
/*
 * Pure: no global state, no I/O.  lcg_* transform the caller-owned RNG word
 * passed by pointer; hash3 is a stateless function of its inputs.  Because
 * they touch nothing else, reordering or deleting any RENDER/BUILD code cannot
 * change their results.  Used by BUILD (§5/§6) and by the render-time rustle.
 */

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
/* §5  generation                  BUILD — recursive tree skeleton        */
/* ===================================================================== */
/*
 * Mutates the Scene's Tree (segments + foliage clouds), via a Tree* — no
 * globals.  BUILD-time only: runs from scene_rebuild (§8) on a user event,
 * NEVER per frame.  Produces the abstract skeleton that §6 bakes into a CharGrid.
 */

/*
 * Segment — one straight trunk-or-branch piece in cell coordinates, the unit
 * draw_segment() walks with a Bresenham line (§6; Bresenham 1965).  Endpoints
 * are floats — the generator works at sub-cell precision and rounds at raster
 * time.  `depth` is both the recursion level AND the bark-thickness tier:
 * 0-1 thick, 2-3 medium, 4+ thin (see TRUNK_THICK/MEDIUM/THIN and CellRole).
 */
typedef struct {
    float    x0, y0, x1, y1;   /* start → end, cell coords (sub-cell precision)   */
    int      depth;            /* 0 = trunk, 1 = primary branch, … ; sets thickness */
} Segment;

/*
 * FoliageCloud — a clump of leaves dropped at a branch tip (Iwata, on how real
 * foliage clumps).  draw_foliage_cloud() (§6) fills it as an aspect-corrected
 * ellipse so it reads as round on screen despite the ~2:1 cell aspect (ellipse
 * scan-conversion, Foley/van Dam).  Stored as centre + base (x) radius; the
 * y-radius is derived as radius / CELL_ASPECT at raster time.
 */
typedef struct {
    float    cx, cy;       /* centre, cell coords                                 */
    float    radius;       /* x radius, cells; y radius = radius / CELL_ASPECT    */
    int      depth_at_tip; /* recursion depth here — gates the WINTER snow-tip    */
} FoliageCloud;

/*
 * Tree — the abstract skeleton: every trunk/branch Segment plus every
 * FoliageCloud.  Produced by the recursive descent in §5 (a stochastic,
 * non-rewriting cousin of an L-system — Lindenmayer 1968; Prusinkiewicz &
 * Lindenmayer 1990) and consumed once by the rasteriser (§6).  Fixed-capacity,
 * append-only pools — NO malloc (project memory rule); seg_add/fol_add silently
 * no-op when full, so runaway recursion degrades gracefully instead of
 * overflowing.  Lives on Scene; rebuilt on every seed/style change.
 */
typedef struct {
    Segment      segs[SEG_MAX];   /* trunk + branches, in emission order   */
    int          n_segs;          /* count in use, 0..SEG_MAX              */
    FoliageCloud fols[FOL_MAX];   /* leaf clumps at terminal tips          */
    int          n_fols;          /* count in use, 0..FOL_MAX              */
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

/* rotate_dir — rotate a 2-D direction vector by `bend` radians (the standard
 * 2×2 rotation matrix).  Used to splay child branches off the parent heading. */
static void rotate_dir(float dx, float dy, float bend, float *out_x, float *out_y)
{
    float c = cosf(bend), s = sinf(bend);
    *out_x = dx * c - dy * s;
    *out_y = dx * s + dy * c;
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
        float ndx, ndy;
        rotate_dir(dir_x, dir_y, bend, &ndx, &ndy);
        /* Slight upward bias (anti-gravity for branches that aren't
         * cascading) so non-trunk branches reach upward. */
        if (depth > 0 && ndy > -0.1f) ndy -= 0.2f;
        float nlen = length * BRANCH_LEN_F;
        grow_branch(t, rng, ex, ey, ndx, ndy, nlen, depth + 1, sp, no_foliage);
    }
}

/* pot_top_row — y of the pot's top edge: reserve POT_ROWS_* rows at the bottom,
 * but on a tiny screen push the pot to the last row so the tree still gets room. */
static int pot_top_row(int rows_eff)
{
    int pot_rows = (rows_eff >= POT_TALL_MIN_ROWS) ? POT_ROWS_TALL : POT_ROWS_SHORT;
    int pot_top  = rows_eff - pot_rows - 1;
    if (pot_top < 5) pot_top = rows_eff - 1;     /* no room above: pot at floor */
    return pot_top;
}

/*
 * trunk_x_at — the trunk centre-line: x of the trunk at height-fraction `frac`.
 * This parametric path is what GIVES each style its silhouette (see KEY FORMULAS).
 * Most styles set only x; KENGAI (cascade) also overrides *y to arc downward past
 * the pot rim.  CHOKKAN draws on *rng for its wobble, so this must be called once
 * per node in ascending order to keep the tree deterministic for a seed.
 */
static float trunk_x_at(uint32_t *rng, TreeStyle style, const StyleParams *sp,
                        float mid, float frac, float trunk_height,
                        int pot_top, float *y)
{
    switch (style) {
    case STYLE_CHOKKAN:                          /* upright: tiny x wobble */
        return mid + lcg_range(rng, -0.3f, 0.3f);
    case STYLE_MOYOGI:                           /* S-curve */
        return mid + sp->trunk_curve_amp
                   * sinf(2.0f * (float)M_PI * sp->trunk_curve_waves * frac);
    case STYLE_SHAKAN:                           /* straight lean */
        return mid + sp->trunk_lean * frac * trunk_height;
    case STYLE_KENGAI: {                          /* rise to apex, then cascade */
        float apex = KENGAI_APEX_FRAC;
        if (frac < apex) return mid;             /* short straight rise */
        float t2 = (frac - apex) / (1.0f - apex);
        *y = (float)pot_top - apex * trunk_height + t2 * 6.0f;   /* down past rim */
        return mid + 6.0f * sinf(t2 * (float)M_PI * 0.6f) * 1.5f;/* arc out right */
    }
    case STYLE_BUNJIN:                            /* sparse, phase-shifted S */
    default:
        return mid + sp->trunk_curve_amp
                   * sinf(2.0f * (float)M_PI * sp->trunk_curve_waves * frac + 0.5f);
    }
}

/*
 * maybe_sprout_branch — at a trunk node (height-fraction `frac`) possibly grow a
 * side branch: only within the clean band BRANCH_BAND_LO..HI (so the base and
 * apex stay bare), then with probability branch_density.  Picks a side + angle
 * (always reaching upward) and recurses via grow_branch.
 */
static void maybe_sprout_branch(Tree *t, uint32_t *rng, float x, float y,
                                float frac, const StyleParams *sp, bool no_foliage)
{
    if (frac <= BRANCH_BAND_LO || frac >= BRANCH_BAND_HI) return;
    if (lcg_unit(rng) >= sp->branch_density)              return;

    float angle = (lcg_unit(rng) < 0.5f ? -1.0f : 1.0f) * lcg_range(rng, 0.7f, 1.2f);
    float dx = sinf(angle);
    float dy = -fabsf(cosf(angle));              /* always upward */
    float blen = sp->branch_length * (1.0f - frac * 0.5f);   /* shorter up high */
    grow_branch(t, rng, x, y, dx, dy, blen, 1, sp, no_foliage);
}

/*
 * add_crown — the foliage cloud at the very top of the trunk: one big crown just
 * above an upright tip, a smaller one at the KENGAI cascading tip, none if bare.
 */
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

    /* Pot + trunk geometry. */
    int   pot_top      = pot_top_row(rows_eff);
    float trunk_height = (float)pot_top * sp->trunk_height_frac;
    if (trunk_height > (float)pot_top - 1.0f) trunk_height = (float)pot_top - 1.0f;
    if (trunk_height < MIN_TRUNK_HEIGHT)      trunk_height = MIN_TRUNK_HEIGHT;
    float mid = (float)cols * 0.5f;

    /* Trunk drawn as a poly-line: more nodes for taller trunks, clamped. */
    int trunk_segs = (int)(trunk_height * TRUNK_SEGS_PER_CELL);
    if (trunk_segs < TRUNK_SEGS_MIN) trunk_segs = TRUNK_SEGS_MIN;
    if (trunk_segs > TRUNK_SEGS_MAX) trunk_segs = TRUNK_SEGS_MAX;

    /* Walk up the trunk: place each node on the style's centre-line, emit the
     * segment to it, and maybe sprout a branch there. */
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

/* ===================================================================== */
/* §6  raster                      BUILD/bake — skeleton → char grid      */
/* ===================================================================== */
/*
 * Mutates a CharGrid (the baked still-life the renderer replays), via a
 * CharGrid* — no globals.  BUILD-time only, from scene_rebuild (§8).  Bresenham
 * line walk for segments + aspect-corrected ellipse fill for foliage clouds.
 */

/*
 * CharGrid — the baked still-life: a 2-D grid of character cells the renderer
 * replays every frame.  §6/§7 write it once per user event; §11 reads it.  The
 * glyph chosen per cell encodes structure/thickness as ASCII (heavy bark vs
 * thin twig vs leaf density) — the text-mode analogue of an intensity ramp
 * (Bourke, ASCII grey-scale).  Fixed [GRID_MAX_H][GRID_MAX_W] backing; only the
 * live rows_eff×cols sub-rectangle is cleared and read, the rest is ignored.
 * Structure-of-arrays (one plane per attribute, so each streams contiguously):
 *   glyph — the ASCII character to stamp in the cell
 *   role  — CellRole: what the cell represents (selects colour + bold at render)
 *   pair  — variant within the role's colour ramp (0..5 for the six leaf hues)
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
    /* Don't overwrite trunk/branch cells with foliage edges; trunk wins. */
    uint8_t existing = grid->role[r][c];
    if (existing >= ROLE_TRUNK_DARK && existing <= ROLE_TRUNK_LIGHT
        && role == ROLE_FOLIAGE) return;
    grid->glyph[r][c] = ch;
    grid->role [r][c] = role;
    grid->pair [r][c] = pair_idx;
}

/*
 * bark_tier — map recursion depth to a bark thickness tier: the role + colour-
 * ramp index, and a base glyph sampled from that tier's palette (deterministic
 * in the cell so the same segment looks stable).  Tiers: 0-1 thick, 2-3 medium,
 * 4+ thin twig (see TRUNK_THICK/MEDIUM/THIN and CellRole).
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
 * draw_segment — Bresenham line in cell coords.  Picks trunk-thick /
 * medium / thin glyph by depth and writes to the grid.
 */
static void draw_segment(const Segment *s, CharGrid *grid, int rows_eff, int cols)
{
    int x0 = (int)(s->x0 + 0.5f);
    int y0 = (int)(s->y0 + 0.5f);
    int x1 = (int)(s->x1 + 0.5f);
    int y1 = (int)(s->y1 + 0.5f);

    char    glyph;
    uint8_t role;
    uint8_t pair_idx;     /* 0..2 within trunk palette */
    bark_tier(s->depth, abs(x0 + y0), &glyph, &role, &pair_idx);

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
        grid_put(grid, y0, x0, g, role, pair_idx, rows_eff, cols);

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
                                uint32_t *rng, CharGrid *grid,
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
            grid_put(grid, y, x, g, ROLE_FOLIAGE, pair_idx, rows_eff, cols);
        }
    }

    /* WINTER snow tip on the topmost cell of each cloud (depth at tip
     * means this is a terminal foliage point). */
    if (th->snow_tip != NULL) {
        int top = yr_lo + 1;
        if (top >= 0 && top < rows_eff && cxi >= 0 && cxi < cols) {
            grid_put(grid, top, cxi, th->snow_tip[0], ROLE_SNOW_TIP, 5,
                     rows_eff, cols);
        }
    }
}

/* ===================================================================== */
/* §7  pot                         BUILD/bake — pot frame → char grid     */
/* ===================================================================== */
/* Mutates the same grids as §6; drawn last so it sits over any trunk that  */
/* dipped into its cells.  BUILD-time only, from scene_rebuild (§8).        */

/*
 * draw_pot — a small trapezoidal pot centred under the trunk.  Width is a third
 * of the screen, clamped to [12, cols-4].  Drawn last so it sits on top of any
 * trunk segments that dipped into its cells (e.g. KENGAI cascades).
 */
static void draw_pot(CharGrid *grid, int pot_top, int rows_eff, int cols)
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
            /* Bottom row — solid. */
            for (int x = x0; x <= x1; x++) {
                if (x < 0 || x >= cols) continue;
                char c = POT_BOTTOM_CH;
                if (x == x0)      c = '\\';
                else if (x == x1) c = '/';
                grid_put(grid, y, x, c, ROLE_POT, 0, rows_eff, cols);
            }
        } else {
            /* Side walls. */
            if (x0 >= 0 && x0 < cols)
                grid_put(grid, y, x0, POT_SIDE_CH, ROLE_POT, 0, rows_eff, cols);
            if (x1 >= 0 && x1 < cols)
                grid_put(grid, y, x1, POT_SIDE_CH, ROLE_POT, 0, rows_eff, cols);
            /* Faint interior fill (just a few dots). */
            for (int x = x0 + 1; x <= x1 - 1; x++) {
                if (x < 0 || x >= cols) continue;
                if (((unsigned)(x + y)) & 3u) continue;   /* 25 % density */
                grid_put(grid, y, x, '.', ROLE_POT, 0, rows_eff, cols);
            }
        }
    }
}

/* ===================================================================== */
/* §8  build                       BUILD orchestration + Scene state      */
/* ===================================================================== */
/*
 * Scene holds all per-instance state.  scene_rebuild is the BUILD combine
 * point: GENERATION (§5) → RASTER (§6/§7).  It mutates Scene.tree, Scene.canvas
 * and Scene.pot_top.  Runs on user events (§9), NEVER per frame.
 */

/*
 * Scene — the whole bonsai instance, read top-to-bottom as a table of contents.
 *   WHAT is built — the tree skeleton, its baked image, and the pot geometry.
 *   HOW it is chosen — the generation knobs the user cycles (which tree, which
 *     palette); a (seed, style, theme) triple fully determines the specimen.
 *   WHERE / WHEN — viewport size, the rustle clock, and the run-state.
 * scene_rebuild owns the WHAT (regenerates it from the HOW); the per-frame tick
 * only advances `time`.  Holds its own cols/rows so the build never reaches
 * into the I/O layer (Screen) for bounds.
 */
typedef struct {
    /* WHAT — the built specimen (regenerated by scene_rebuild on any change) */
    Tree       tree;          /* skeleton: segments + foliage clouds (§5)     */
    CharGrid   canvas;        /* baked still-life the renderer replays (§6/§7) */
    int        pot_top;       /* y of the pot's top edge — a build output     */

    /* HOW — which tree to build (the generation knobs the user drives) */
    uint32_t   seed;          /* RNG seed → one specific tree of the style    */
    int        current_style; /* TreeStyle: trunk shape + branching recipe    */
    int        current_theme; /* palette + leaf set + bare flag               */

    /* WHERE / WHEN */
    int        cols, rows;    /* viewport in cells (own copy; cf. Screen)     */
    float      time;          /* rustle clock, seconds — the sim state        */
    bool       paused;        /* run-state: freeze the rustle clock           */
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

/* ===================================================================== */
/* §9  events                      USER EVENTS — mutate + trigger BUILD   */
/* ===================================================================== */
/*
 * Each runs ONLY on input or resize, NOT as part of the per-frame tick.
 * They mutate Scene and call scene_rebuild (§8) to re-bake; scene_cycle_theme
 * additionally re-binds colours (§3).  None of these advance Scene.time.
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
    scene_rebuild(s);     /* WINTER toggles foliage on/off → rebuild */
}

/* ===================================================================== */
/* §10  simulation                 SIMULATION — the only per-tick advance */
/* ===================================================================== */
/*
 * Advances Scene.time (the rustle clock) and nothing else.  Gated by the
 * Scene.paused boolean — the program's only DELAY.  Called once per frame
 * from main (§13); this is the entire per-tick state advance.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->time += dt;
}

/* ===================================================================== */
/* §11  render                     RENDER — char grid → screen (read-only)*/
/* ===================================================================== */
/*
 * Reads the baked Scene.canvas + Scene.time; writes the ncurses screen ONLY,
 * never Scene or the canvas.  The wind flutter is the lone EFFECT, and it is
 * DERIVED here (not stored): hash3 gates an on-the-fly leaf re-pick into a local.
 */

/* cell_style — the colour pair + attribute a cell is drawn with, from its role.
 * Bark steps dark→bold / mid→normal / light→dim; bright foliage hues
 * (≥ FOLIAGE_BOLD_FROM) go bold so the canopy has highlights. */
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

/* rustle_glyph — the wind flutter (the lone EFFECT, derived not stored): a
 * stable per-cell hash (cell + time window) fires for ~RUSTLE_RATE of cells; a
 * firing cell swaps in a different random leaf, else the baked glyph holds. */
static char rustle_glyph(char baked, const Theme *th, int c, int r,
                         int time_key, int leaf_len)
{
    uint32_t h = hash3(c, r, time_key);
    if ((h & 0xFFu) < (uint32_t)(255.0f * RUSTLE_RATE))
        return th->leaf_set[(h >> 8) % (uint32_t)leaf_len];
    return baked;
}

/*
 * scene_render — emit the baked grid to ncurses: each non-empty cell coloured
 * by its role, foliage flickering with the wind.
 */
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

    /* Emit each non-empty cell, batching attron/attroff so a run of one colour
     * costs a single ncurses attr change rather than one per cell. */
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

/* ===================================================================== */
/* §12  screen                     RENDER — terminal I/O + HUD            */
/* ===================================================================== */
/*
 * Terminal lifecycle (init / resize / teardown) plus the two-line HUD overlay.
 * Writes the ncurses screen only.  screen_resize_curses re-reads the terminal
 * size on SIGWINCH — a USER EVENT, driven from app_do_resize (§13).
 */

/*
 * Screen — cached terminal size in character cells, owned by the I/O layer.
 * Refreshed by getmaxyx at init and on every SIGWINCH (§12).  Scene keeps its
 * OWN cols/rows copy so the build/sim never reaches into the I/O layer for
 * bounds; the two are synced at resize (app_do_resize, §13).
 */
typedef struct { int cols, rows; } Screen;   /* cells, ≥1; refreshed on resize */

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
 * screen_draw — render the tree, then overlay the two-line HUD:
 *   top    row 0      DATA    — title, rustle/paused, style, theme (left)
 *                              + fps / Hz (right-aligned).
 *   bottom row rows-1 ACTIONS — the key legend.
 * Both bars are filled first so the tree never shows through.  Every field is
 * clipped to the terminal width with "%.*s", and the left data block is kept
 * clear of the right-aligned stats, so a narrow terminal can neither overflow
 * nor wrap the HUD.
 */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_render(s);

    int cols = sc->cols;
    int rows = sc->rows;
    if (cols < 1 || rows < 1) return;

    /* ── Top row 0: DATA — left block + right-aligned fps/Hz. ── */
    char data[128], stats[40];
    snprintf(data, sizeof data, " BONSAI   %s   style:%s  theme:%s ",
             s->paused ? "PAUSED" : "RUSTLE",
             styles[s->current_style].name,
             themes[s->current_theme].name);
    snprintf(stats, sizeof stats, " %5.1f fps  %3d Hz ", fps, sim_fps);
    int sx = cols - (int)strlen(stats);          /* right-aligned stats column */

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < cols; x++) mvaddch(0, x, ' ');
    if (sx >= 0) {
        mvprintw(0, 0, "%.*s", sx, data);        /* data clipped clear of stats */
        mvprintw(0, sx, "%s", stats);
    } else {
        mvprintw(0, 0, "%.*s", cols, data);      /* too narrow for stats: data only */
    }
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* ── Bottom row: ACTIONS — every interactive key. ── */
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

/* ===================================================================== */
/* §13  app + main                 USER-EVENT glue + PERFORMANCE combine  */
/* ===================================================================== */
/*
 * App owns the sub-systems + run flags.  The signal handlers and app_handle_key
 * / app_do_resize are USER EVENTS (mutate state, may trigger a BUILD), NOT part
 * of the tick.  main() is the ONE per-frame combine point (see ARCHITECTURE):
 *   resize? → dt(cap) → scene_tick (SIM) → fps → sleep (PERF) → render → input.
 */

/*
 * App — the running program: the two sub-systems plus the loop's run-state.
 * A single instance (g_app), driven by main (§13).  sim_fps is the render
 * frame-cap target in Hz (the [ / ] keys), NOT a simulation-accuracy knob — the
 * rustle clock advances by real dt regardless of it.  running / need_resize are
 * written from signal handlers, hence volatile sig_atomic_t: the only type the
 * C standard guarantees safe to set in a handler and re-read in the loop without
 * tearing or being optimised away.
 */
typedef struct {
    Scene                 scene;       /* the bonsai instance (§8)             */
    Screen                screen;      /* terminal geometry cache (§12)        */
    int                   sim_fps;     /* frame-cap target, SIM_FPS_MIN..MAX   */
    volatile sig_atomic_t running;     /* 0 ⇒ exit the main loop (SIGINT / q)  */
    volatile sig_atomic_t need_resize; /* 1 ⇒ rebuild at top of next frame     */
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
        /* 1. apply a pending resize before this frame is measured */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
        }

        /* 2. measure elapsed time, clamped so a stall can't spiral the rustle */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > MAX_FRAME_MS * NS_PER_MS) dt = MAX_FRAME_MS * NS_PER_MS;

        /* 3. advance the only sim state: the rustle clock */
        float dt_sec = (float)dt / (float)NS_PER_SEC;
        scene_tick(&app->scene, dt_sec);

        /* 4. refresh the fps readout a couple of times a second */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* 5. cap the frame rate, then draw the tree + HUD */
        int64_t target_ns = TICK_NS(app->sim_fps);
        int64_t elapsed   = clock_ns() - frame_time + dt;
        clock_sleep_ns(target_ns - elapsed);
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* 6. drain one input event (may flip running or trigger a rebuild) */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
