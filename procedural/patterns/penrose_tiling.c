/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * penrose_tiling.c — a Penrose tiling that draws itself, in ASCII.
 *
 * We start with a few triangles, then repeatedly replace every triangle
 * with a smaller arrangement of the same two triangle shapes. Do that a
 * handful of times and you get a patch of the famous Penrose pattern: a
 * fill of the plane that never repeats, yet has perfect 5-fold symmetry.
 *
 * Sister files: quasicrystal.c builds the same 5-fold family a different
 * way (overlapping waves); wang_tiles.c is another rule-constrained tiling.
 * References live in the CONCEPTS block below (Gardner 1977 is the easiest
 * first read).
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * THE IDEA
 *   The whole pattern is built from two triangle shapes — the two halves you
 *   get by slicing a rhombus down the middle. We call them ACUTE (a long thin
 *   sliver) and OBTUSE (a squat one). The trick that makes Penrose work: there
 *   is a fixed rule for cutting each shape into a few smaller copies of the
 *   same two shapes. Apply that rule to every triangle, over and over, and the
 *   triangles get finer and finer while the overall picture never starts
 *   repeating. That's "deflation". The cuts always land at the golden-ratio
 *   point of an edge (φ ≈ 1.618), which is what keeps everything fitting.
 *
 *   We start from a seed (a 10-triangle star, or a single rhombus), then apply
 *   the cut rule a handful of times (5 or 6). Each round multiplies the
 *   triangle count by roughly φ, so a 10-triangle star reaches ~900 triangles
 *   after 5 rounds, ~2,300 after 6.
 *
 * HOW IT'S DRAWN
 *   Each triangle is three corner points. We draw its three edges as straight
 *   lines of ASCII characters, picking the character that best matches the
 *   line's slope (- | / \). Acute and obtuse triangles get different colours
 *   so you can tell the two shapes apart. The whole thing animates: it shows
 *   the seed, applies one cut-round at a time so you watch it subdivide, holds
 *   at full detail, then loops back to the seed.
 *
 * STORAGE
 *   All triangles live in two plain arrays, capped at 4,096 each (room to
 *   spare for 6 rounds). We use two arrays so a cut-round can read every parent
 *   from one while writing all the children into the other.
 *
 * References — the maths and the line-drawing trick the code can't show you:
 *   • Gardner, M. (1977) — Scientific American 236(1):110-121. The famous
 *     popular article on Penrose tiles; easiest first read.
 *   • Penrose, R. (1974) — Bulletin of the IMA 10:266-271. The original tiling.
 *   • Grünbaum & Shephard (1987) — "Tilings and Patterns", ch. 10. The
 *     definitive treatment of the half-rhombus (Robinson) triangles and the
 *     exact cut rules used here.
 *   • Senechal, M. (1995) — "Quasicrystals and Geometry". Why the limit is
 *     non-repeating yet ordered.
 *   • De Bruijn, N.G. (1981) — Indagationes Math 43:39-66. The other way to
 *     build the same tiling (cf. penrose_pentagrid.c).
 *   • Preshing, J. (2011) — preshing.com/20110831/penrose-tiling-explained/.
 *     Hands-on walkthrough; closest match to this code.
 *   • Bresenham, J. (1965) — IBM Systems Journal 4(1):25-30. The integer-only
 *     line-drawing method used for every edge (see draw_line_styled).
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * THE PICTURE TO HOLD IN YOUR HEAD
 * Think of a triangle of putty. There's one fixed way to cut it into a couple
 * of smaller putty triangles. Cut those the same way, and again, and again.
 * After a few rounds your one triangle has become a fine mosaic of tiny ones.
 * Penrose's discovery: with the right two triangle shapes and the right cuts,
 * that mosaic never repeats itself, yet stays perfectly 5-fold symmetric.
 *
 * THE LOOP
 *   1. SEED — drop a few triangles in a known starting layout (a 10-triangle
 *      star, or a single rhombus split into two triangles).
 *   2. CUT one round — replace every triangle with its smaller children. An
 *      acute splits into 2 (one acute, one obtuse); an obtuse into 3.
 *   3. Repeat the cut a few times. Count grows by roughly φ each round.
 *   4. DRAW each triangle's three edges, coloured by its shape.
 *   5. ANIMATE — show the seed, then cut one round at a time so you watch it
 *      subdivide; hold at full detail, then loop back to the seed.
 *
 * The exact cut formulas live right above deflate_step (where they're used).
 *
 * THINGS THAT TRIP YOU UP (and why the code looks the way it does)
 *  • The star seed alternates the orientation of every other triangle (the
 *    "(i & 1)" flip in seed_star). Skip that and the children from neighbouring
 *    triangles won't line up edge-to-edge — you'd get gaps and overlaps.
 *  • Terminal cells are about twice as tall as they are wide, so we squash the
 *    y axis (divide by ASPECT_Y_F). Without it the round star looks like a
 *    flattened ellipse.
 *  • Neighbouring triangles share an edge, so each shared edge gets drawn
 *    twice — same colour, so it looks identical. Not worth the bookkeeping to
 *    avoid.
 *  • Cutting can't run forever: the triangle arrays are capped (MAX_TRIANGLES),
 *    so we stop at depth 6 for the star. Push deeper and triangles silently go
 *    missing rather than overflowing.
 *
 * QUICK WAYS TO CHECK IT'S RIGHT
 *  • STAR: 10 triangles should meet cleanly at the centre — perfect 5-fold
 *    symmetry. THIN: mostly thin (acute) triangles. THICK: mostly squat
 *    (obtuse) ones. Press 'r' to spin the whole thing to a new orientation;
 *    'space' freezes the build; 'g' switches between edges, centre-dots, both.
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

/* ── HOW THE FILE IS ORGANISED ─────────────────────────────────────────── *
 *
 * The tiling itself is fixed — the only thing that "runs" is the animation
 * that builds it up and loops. The code is split into layers, each with one
 * job, and they only meet in main's per-frame loop:
 *
 *   §1 CONFIG       constants, colour themes, the Pattern/Glyph menus (data)
 *   §2 PERFORMANCE  a clock and a sleep
 *   §3 LOGIC        small pure helpers (name lookups, a hash, edge→glyph)
 *   §4 SIMULATION   the triangles, the cut rule, the seeds, the animation tick
 *   §5 RENDER       colours, screen layout, drawing triangles, the HUD
 *   §6 APP          signals, resize, keys, and the main loop that ties it
 *                   together: advance the animation, sleep to cap the frame
 *                   rate, then draw. Keys and signals change things outside
 *                   that loop.
 * ─────────────────────────────────────────────────────────────────────── */

/* ── §1  CONFIG — constants, themes, enums (data only) ─────────────────── */

enum {
    SIM_FPS_MIN         =  10,
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,
    SIM_FPS_STEP        =  10,

    /* How often we redraw, capped separately from the sim so a fast machine
     * doesn't burn cycles redrawing the same frame. */
    RENDER_FPS_CAP      =  60,    /* draw at most this many frames per second */
    MAX_FRAME_DT_MS     = 100,    /* if a frame stalls, pretend no more than this passed,
                                   * so the sim can't try to "catch up" forever */

    SPEED_MIN           =   1,
    SPEED_DEF           =   8,
    SPEED_MAX           =  64,

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* How many triangles each array can hold — comfortably more than the
     * ~2,300 a 6-round star reaches. */
    MAX_TRIANGLES       = 4096,

    /* Colour-pair slots. HUD/HINT are the shared ones every demo reserves. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* the 8 theme tints sit at +0..+7 */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define ASPECT_Y_F           2.0f   /* terminal cells are ~2x taller than wide */

/* Animation pacing (see scene_tick). We show one cut-round, wait, show the
 * next, and so on, so you can watch the tiling subdivide. These are measured
 * in "animation seconds"; the +/- keys speed that clock up or down. */
#define STEP_INTERVAL_SEC    0.60f   /* how long each cut-round stays on screen */
#define HOLD_SEC             2.50f   /* how long to pause on the finished tiling */

/* The golden-ratio point: every cut lands this fraction of the way along an
 * edge. This is 1/φ. (See phi_split / deflate_step.) */
#define INV_PHI              0.6180339887498949f

/* The two triangle shapes: thin sliver vs squat. */
#define TRI_ACUTE            0
#define TRI_OBTUSE           1

/*
 * Pattern — which picture to build; the n/p keys cycle through these.
 *
 * Each choice is really two settings rolled into one: which seed to start from,
 * and how many cut-rounds to apply. scene_begin_cycle picks the seed builder,
 * pattern_depth picks the round count the animation climbs to.
 *   • STAR and DEEP use the same 10-triangle star seed; DEEP just runs one
 *     extra round for a finer figure.
 *   • THIN and THICK start from a single rhombus instead, so there's no 5-fold
 *     symmetry — you just watch one rhombus fan out, the clearest look at the
 *     bare cut rule.
 * The round count can't go past 6 or the triangle arrays overflow.
 */
typedef enum {
    PATTERN_STAR  = 0,    /* star seed, 5 rounds — the default 5-fold figure   */
    PATTERN_DEEP  = 1,    /* star seed, 6 rounds — same seed, one round denser */
    PATTERN_THIN  = 2,    /* a single thin rhombus, 5 rounds                   */
    PATTERN_THICK = 3,    /* a single thick rhombus, 5 rounds                  */
    N_PATTERNS    = 4,    /* how many there are — not itself a pattern         */
} Pattern;

/*
 * GlyphSet — how to draw each triangle; the g/G keys cycle these. This only
 * changes the look, never the tiling. EDGES draws the three sides (the outline
 * you'd picture); CENTERS drops a single dot at each triangle's middle (a dot
 * field that reveals the underlying point pattern); BOTH does both. Edges and
 * dots use different colour slots so they don't blur together.
 */
typedef enum {
    GLYPH_EDGES   = 0,    /* outlines only        */
    GLYPH_CENTERS = 1,    /* centre dots only     */
    GLYPH_BOTH    = 2,    /* both                 */
    N_GLYPH_SETS  = 3,    /* how many there are   */
} GlyphSet;

/* Which colour slot each shape draws in. Acute and obtuse use different slots
 * so the two shapes are always tellable apart; the centre dots use yet other
 * slots so they don't merge with the edges. */
#define ACUTE_EDGE_RAMP      4
#define OBTUSE_EDGE_RAMP     6
#define ACUTE_CENTER_RAMP    5
#define OBTUSE_CENTER_RAMP   7

/*
 * Theme — one named colour palette; the t/T keys cycle through them.
 *
 * Keeping a whole palette in one table row means adding or tweaking a theme is
 * a one-line edit, and theme_apply just copies the row into the terminal's
 * colour slots. Each number is an index into the 256-colour palette; they're
 * all kept in the bright half (per the project's palette rule) so nothing
 * vanishes against a black background.
 *
 *   name    — the label shown in the HUD.
 *   ramp[8] — eight shades, dark to bright. The draw code only actually uses
 *             slots 4–7 (acute/obtuse, edge/dot), so the theme's character
 *             comes from how those few shades relate, not any single colour.
 *             Slots 0–3 are spare, kept for possible future shading.
 */
typedef struct {
    const char *name;      /* HUD label                       */
    short       ramp[8];   /* eight shades, dark to bright     */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name       0    1    2    3    4    5    6    7  */
    { "DEFAULT",{ 24,  31,  39,  70,  76, 137, 215, 230 } },
    { "MATRIX", { 28,  34,  40,  46,  76, 118, 154, 192 } },
    { "NOVA",   { 60,  91, 134, 165, 207, 213, 219, 231 } },
    { "MONO",   {240, 243, 245, 247, 249, 251, 253, 255 } },
    { "OCEAN",  { 24,  25,  31,  38,  45,  51, 117, 195 } },
    { "FIRE",   { 88, 124, 130, 166, 202, 208, 214, 226 } },
    { "EARTH",  { 94, 130, 137, 173, 179, 215, 222, 230 } },
    { "FOREST", { 28,  34,  40,  70,  76, 112, 156, 192 } },
    { "DESERT", {130, 137, 143, 173, 179, 215, 222, 229 } },
    { "ARCTIC", { 24,  31,  67, 110, 117, 153, 195, 231 } },
};

/* ── §2  PERFORMANCE — a clock and a sleep ─────────────────────────────── */
/* The frame-rate bookkeeping that uses these lives in main (§6).           */

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

/* ── §3  LOGIC — small pure helpers (no state, no drawing) ─────────────── */
/* Each reads only its arguments and returns an answer. (world_to_screen is */
/* the same kind of thing but lives in §5, next to the type it needs.)      */

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_STAR:  return "STAR ";
    case PATTERN_DEEP:  return "DEEP ";
    case PATTERN_THIN:  return "THIN ";
    case PATTERN_THICK: return "THICK";
    default:            return "?    ";
    }
}

static int pattern_depth(Pattern p)
{
    switch (p) {
    case PATTERN_STAR:  return 5;
    case PATTERN_DEEP:  return 6;
    case PATTERN_THIN:  return 5;
    case PATTERN_THICK: return 5;
    default:            return 5;
    }
}

static const char *glyph_set_name(GlyphSet g)
{
    switch (g) {
    case GLYPH_EDGES:   return "edges";
    case GLYPH_CENTERS: return "ctrs ";
    case GLYPH_BOTH:    return "both ";
    default:            return "?    ";
    }
}

/* Next / previous in a wrap-around list — used to cycle pattern, theme, glyph.
 * wrap_dec adds n before the modulo so it never goes negative. */
static inline int wrap_inc(int v, int n) { return (v + 1) % n; }
static inline int wrap_dec(int v, int n) { return (v + n - 1) % n; }

/* Scrambles three numbers into one. We use it on 'r' to pick a fresh random
 * orientation for the tiling. */
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

/* Picks the ASCII character whose slant best matches a line going (dx, dy):
 * flat -> '-', steep -> '|', and the two diagonals -> '\' or '/'. */
static inline char line_glyph_for(int dx, int dy)
{
    int adx = abs(dx), ady = abs(dy);
    if (adx >= 2 * ady) return '-';
    if (ady >= 2 * adx) return '|';
    if ((dx > 0) == (dy > 0)) return '\\';
    return '/';
}

/* ── §4  SIMULATION — the triangles, the cut rule, the seeds, the tick ─── */
/* The only part that changes over time. The seeds and deflate_step rewrite  */
/* the triangle arrays; scene_tick paces the build and loops it.             */

/*
 * Triangle — one of the two basic shapes, the building block of everything.
 * Two of them glued along their long edge make a Penrose rhombus; we work in
 * these half-rhombi because the cut rule then maps triangles to triangles
 * cleanly. (Grünbaum & Shephard ch. 10; file header.)
 *
 * The corners are kept in a fixed order, not just three loose points, because
 * the cut rule is stated relative to one special corner — the apex — and you
 * can't recover which corner that was from the geometry alone.
 *   kind  — which shape: acute (thin) or obtuse (squat). Decides both how the
 *           triangle is cut and what colour it draws in.
 *   ax,ay — the apex: the sharp corner every cut is measured from. Must stay
 *           in this slot.
 *   bx,by / cx,cy — the other two corners (the long edge neighbours share).
 *
 * Coordinates are in the maths "world" (the seed sits on a unit circle); the
 * conversion to screen cells happens once, in world_to_screen (§5).
 */
typedef struct {
    int   kind;           /* acute or obtuse — sets the cut rule and the colour */
    float ax, ay;         /* apex   — the sharp corner cuts are measured from    */
    float bx, by;         /* corner — one end of the shared long edge            */
    float cx, cy;         /* corner — the other end                              */
} Triangle;

/*
 * Tiling — the whole tiling, just a flat list of triangles (we never track who
 * touches whom; the cut rule looks at one triangle at a time, so a plain array
 * is enough).
 *
 * There are TWO arrays because one cut-round needs to read every parent while
 * writing all the children — writing children back into the same array would
 * stomp on parents we hadn't gotten to yet. So we read from one array, fill the
 * other with the finer level, then just flip which one is "live". Both arrays
 * are allocated up front so drawing never has to allocate memory.
 *   buf[2] — the two arrays (one holds the current triangles, one the next set)
 *   n[2]   — how many triangles are really in each array
 *   active — which array is the live one right now (the other is scratch)
 */
typedef struct {
    Triangle buf[2][MAX_TRIANGLES];   /* the two triangle arrays         */
    int      n[2];                    /* how many are in buf[0] / buf[1]  */
    int      active;                  /* 0 or 1 — which one is live now    */
} Tiling;

/* The cut point on the line from A to B, at the golden-ratio fraction along it.
 * Every cut lands on one of these. Giving it a name lets deflate_step read like
 * the rule instead of a wall of arithmetic. */
static inline void phi_split(float ax, float ay, float bx, float by,
                             float *px, float *py)
{
    *px = ax + (bx - ax) * INV_PHI;
    *py = ay + (by - ay) * INV_PHI;
}

/* Adds one child triangle to the array being filled, unless it's already full
 * (in which case the triangle is quietly dropped, so we never overflow). */
static inline void emit_child(Tiling *tl, int to, int *n_out, int kind,
                              float ax, float ay,
                              float bx, float by,
                              float cx, float cy)
{
    if (*n_out >= MAX_TRIANGLES) return;
    Triangle *o = &tl->buf[to][(*n_out)++];
    o->kind = kind;
    o->ax = ax; o->ay = ay;
    o->bx = bx; o->by = by;
    o->cx = cx; o->cy = cy;
}

/*
 * One cut-round: replace every triangle with its smaller children. Reads the
 * live array, writes the next finer level into the other, then makes that the
 * live one. The exact cuts (which child gets which corners) are the Penrose
 * rule from the references — an acute becomes 1 acute + 1 obtuse, an obtuse
 * becomes 2 obtuse + 1 acute. Children past the array cap are dropped.
 */
static void deflate_step(Tiling *tl)
{
    int from  = tl->active;
    int to    = 1 - tl->active;   /* the other array — we fill it with the finer level */
    int n_in  = tl->n[from];
    int n_out = 0;

    for (int i = 0; i < n_in; i++) {
        const Triangle *t = &tl->buf[from][i];

        if (t->kind == TRI_ACUTE) {
            /* cut once, on the apex-to-B edge → one acute + one obtuse */
            float px, py;
            phi_split(t->ax, t->ay, t->bx, t->by, &px, &py);
            emit_child(tl, to, &n_out, TRI_ACUTE,
                       t->cx, t->cy,  px,    py,    t->bx, t->by);
            emit_child(tl, to, &n_out, TRI_OBTUSE,
                       px,    py,     t->cx, t->cy, t->ax, t->ay);
        } else {
            /* cut twice, both from corner B → two obtuse + one acute */
            float qx, qy, rx, ry;
            phi_split(t->bx, t->by, t->ax, t->ay, &qx, &qy);
            phi_split(t->bx, t->by, t->cx, t->cy, &rx, &ry);
            emit_child(tl, to, &n_out, TRI_OBTUSE,
                       rx,    ry,     t->cx, t->cy, t->ax, t->ay);
            emit_child(tl, to, &n_out, TRI_OBTUSE,
                       qx,    qy,     rx,    ry,    t->bx, t->by);
            emit_child(tl, to, &n_out, TRI_ACUTE,
                       rx,    ry,     qx,    qy,    t->ax, t->ay);
        }
    }

    tl->n[to]  = n_out;
    tl->active = to;
}

/* ── Seeds — the starting layout, one per pattern ──────────────────────── */

/* Spins a point around the centre by a given angle (passed as its cos/sin,
 * computed once and shared by all the corners). Used to give each fresh build
 * a random orientation. */
static inline void rotate2d(float x, float y, float cr, float sr,
                            float *ox, float *oy)
{
    *ox = x * cr - y * sr;
    *oy = x * sr + y * cr;
}

/*
 * The star seed: 10 thin triangles arranged like spokes, all sharp tips
 * meeting at the centre. Every other one is mirrored (the "(i & 1)" flip) so
 * that after cutting, the children from neighbours line up edge-to-edge instead
 * of leaving gaps.
 */
static void seed_star(Tiling *tl, float rot)
{
    int n = 0;
    Triangle *out = tl->buf[0];
    for (int i = 0; i < 10; i++) {
        float a0 = (float)i * 2.0f * (float)M_PI / 10.0f + rot;
        float a1 = (float)(i + 1) * 2.0f * (float)M_PI / 10.0f + rot;
        Triangle *t = &out[n++];
        t->kind = TRI_ACUTE;
        t->ax = 0.0f; t->ay = 0.0f;
        if ((i & 1) == 0) {
            t->bx = cosf(a0); t->by = sinf(a0);
            t->cx = cosf(a1); t->cy = sinf(a1);
        } else {
            t->bx = cosf(a1); t->by = sinf(a1);
            t->cx = cosf(a0); t->cy = sinf(a0);
        }
    }
    tl->n[0] = n;
    tl->active = 0;
}

/*
 * A single thin rhombus, sliced into its two thin triangles. Lets you watch
 * the thin-triangle cut rule on its own, with no surrounding symmetry.
 */
static void seed_thin(Tiling *tl, float rot)
{
    /* Thin rhombus: sharp 36° tips top and bottom, blunt 144° corners L/R. */
    float c18 = cosf(18.0f * (float)M_PI / 180.0f);
    float s18 = sinf(18.0f * (float)M_PI / 180.0f);

    float v_top_x = 0.0f,         v_top_y = c18;       /* 36° */
    float v_bot_x = 0.0f,         v_bot_y = -c18;      /* 36° */
    float v_rt_x  =  s18,         v_rt_y  = 0.0f;      /* 144° */
    float v_lf_x  = -s18,         v_lf_y  = 0.0f;      /* 144° */

    /* Spin every corner to this build's orientation. */
    float cr = cosf(rot), sr = sinf(rot);
    float tx, ty, bx, by, rx, ry, lx, ly;
    rotate2d(v_top_x, v_top_y, cr, sr, &tx, &ty);
    rotate2d(v_bot_x, v_bot_y, cr, sr, &bx, &by);
    rotate2d(v_rt_x,  v_rt_y,  cr, sr, &rx, &ry);
    rotate2d(v_lf_x,  v_lf_y,  cr, sr, &lx, &ly);

    /* Cut down the short diagonal into top and bottom halves. */
    Triangle *out = tl->buf[0];
    out[0] = (Triangle){ TRI_ACUTE, tx, ty, rx, ry, lx, ly };  /* top half */
    out[1] = (Triangle){ TRI_ACUTE, bx, by, lx, ly, rx, ry };  /* bottom — mirrored so they fit */
    tl->n[0]  = 2;
    tl->active = 0;
}

/*
 * A single thick rhombus, sliced into its two squat triangles — the
 * thick-rhombus counterpart of seed_thin.
 */
static void seed_thick(Tiling *tl, float rot)
{
    /* Thick rhombus: 72° corners L/R, blunt 108° corners top/bottom. */
    float c36 = cosf(36.0f * (float)M_PI / 180.0f);
    float s36 = sinf(36.0f * (float)M_PI / 180.0f);

    float v_lf_x = -c36,         v_lf_y =  0.0f;       /* 72° */
    float v_rt_x =  c36,         v_rt_y =  0.0f;       /* 72° */
    float v_tp_x =  0.0f,        v_tp_y =  s36;        /* 108° */
    float v_bt_x =  0.0f,        v_bt_y = -s36;        /* 108° */

    float cr = cosf(rot), sr = sinf(rot);
    float lx, ly, rx, ry, tx, ty, bx, by;
    rotate2d(v_lf_x, v_lf_y, cr, sr, &lx, &ly);
    rotate2d(v_rt_x, v_rt_y, cr, sr, &rx, &ry);
    rotate2d(v_tp_x, v_tp_y, cr, sr, &tx, &ty);
    rotate2d(v_bt_x, v_bt_y, cr, sr, &bx, &by);

    /* Cut down the long diagonal; each half's apex is its blunt 108° corner. */
    Triangle *out = tl->buf[0];
    out[0] = (Triangle){ TRI_OBTUSE, tx, ty, lx, ly, rx, ry };  /* top half */
    out[1] = (Triangle){ TRI_OBTUSE, bx, by, rx, ry, lx, ly };  /* bottom half */
    tl->n[0]  = 2;
    tl->active = 0;
}

/* ── Scene — everything the animation needs, plus how to advance it ─────── */

/*
 * Scene — the tiling plus every dial and bit of progress that drives it. The
 * drawing code only reads it; just a few orchestrator functions write to it.
 */
typedef struct {
    /* The tiling being built. */
    Tiling   tiling;

    /* What the viewer chose. */
    Pattern  current_pattern;   /* which picture (seed + how many rounds) */
    int      speed;             /* how fast the build plays (+/- keys)    */

    /* How far along the build we are, and this build's random orientation. */
    int      defl_level;        /* how many rounds have played so far     */
    float    step_timer;        /* time saved up toward the next round     */
    float    time_secs;         /* seconds running, used to vary 'r'       */
    float    rot;               /* this build's rotation                   */
    int      seed;              /* this build's random number              */
    bool     paused;            /* true = animation frozen                 */

    /* Display choices, just toggled by keys. */
    int      current_theme;     /* which colour palette */
    GlyphSet current_glyph;     /* edges, dots, or both  */
} Scene;

/*
 * Lay down the seed for the chosen pattern and rewind the animation to the
 * start. It does NOT cut anything yet — scene_tick does that one round at a
 * time. Called when starting, reseeding, switching pattern, or looping.
 */
static void scene_begin_cycle(Scene *s)
{
    switch (s->current_pattern) {
    case PATTERN_STAR:
    case PATTERN_DEEP:  seed_star (&s->tiling, s->rot); break;
    case PATTERN_THIN:  seed_thin (&s->tiling, s->rot); break;
    case PATTERN_THICK: seed_thick(&s->tiling, s->rot); break;
    case N_PATTERNS:    break;
    }
    s->defl_level = 0;
    s->step_timer = 0.0f;
}

static void scene_reseed(Scene *s)
{
    /* Roll a new random orientation and restart the build. */
    s->seed = (int)hash3((int)(s->time_secs * 1000.0f), s->defl_level, 0xC0FFEE);
    uint32_t h = hash3(s->seed, 0, 1);
    s->rot = ((float)(h & 0xFFFFu) / 65536.0f) * 2.0f * (float)M_PI;
    scene_begin_cycle(s);
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_STAR;
    s->current_glyph   = GLYPH_EDGES;
    s->seed            = 0xCAFE;
    s->rot             = 0.0f;
    scene_begin_cycle(s);
}

/*
 * Drives the animation. Below full depth, it cuts one more round every so
 * often so you watch it subdivide; once finished, it waits a beat, then loops
 * back to the seed. The +/- speed setting stretches or shrinks the clock.
 * Nothing moves while paused.
 */
static void scene_tick(Scene *s, float dt)
{
    s->time_secs += dt;
    if (s->paused) return;

    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->step_timer += dt * speed_mul;

    int target = pattern_depth(s->current_pattern);
    if (s->defl_level < target) {
        if (s->step_timer >= STEP_INTERVAL_SEC) {
            s->step_timer -= STEP_INTERVAL_SEC;
            deflate_step(&s->tiling);
            s->defl_level++;
        }
    } else {
        if (s->step_timer >= HOLD_SEC) {
            s->step_timer = 0.0f;
            scene_begin_cycle(s);   /* loop back to the seed and replay */
        }
    }
}

/* ── §5  RENDER — turn the tiling into characters on screen ────────────── */
/* Colours, screen layout, drawing triangles, the HUD. Only reads the state; */
/* a triangle's colour comes straight from its shape, nothing fancier.       */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
    } else {
        static const short fb[8] = {
            COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN,   COLOR_CYAN,
            COLOR_GREEN, COLOR_YELLOW,COLOR_YELLOW, COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), fb[i], -1);
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

/*
 * Screen — the terminal window and how to place the tiling in it. The centre
 * cell and zoom factor depend only on the window size, so we work them out once
 * (in screen_layout, and again whenever the window is resized) and then
 * world_to_screen can map thousands of corners with cheap arithmetic.
 */
typedef struct {
    int cols, rows;         /* window size in cells, refreshed on resize     */
    int cx, cy;             /* the centre cell the tiling is built around    */
    float scale;            /* how many cells one maths-unit is worth         */
} Screen;

static void screen_layout(Screen *s)
{
    int top = 2, bottom = s->rows - 1;
    int avail_h = bottom - top;
    int avail_w = s->cols;

    s->cx = avail_w / 2;
    s->cy = top + avail_h / 2;

    /* Zoom so the seed (about one unit wide) nearly fills the smaller side of
     * the window, squashing the y axis so the round star stays round. */
    float max_h = (float)(avail_w - 4) * 0.5f;
    float max_v = (float)(avail_h - 2) * 0.5f * ASPECT_Y_F;
    s->scale = (max_h < max_v) ? max_h : max_v;
    if (s->scale < 4.0f) s->scale = 4.0f;
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

/* Turns a maths (x, y) into a screen cell: zoom, squash the y axis, and shift
 * to the centre. */
static inline void world_to_screen(const Screen *sc, float wx, float wy,
                                    int *sx, int *sy)
{
    *sx = (int)(wx * sc->scale)              + sc->cx;
    *sy = (int)(wy * sc->scale / ASPECT_Y_F) + sc->cy;
}

/*
 * Draws a straight line of one repeated character from (x0,y0) to (x1,y1),
 * one cell at a time (Bresenham's classic integer line walk). The caller picks
 * the character and colour. Cells off-screen, or in the top two / bottom HUD
 * rows, are skipped so the line art never paints over the HUD.
 */
static void draw_line_styled(int x0, int y0, int x1, int y1,
                             int rows, int cols,
                             int pair, int attr, char glyph)
{
    int dx_abs = abs(x1 - x0);
    int dy_abs = abs(y1 - y0);
    int sx     = (x0 < x1) ?  1 : -1;
    int sy     = (y0 < y1) ?  1 : -1;
    int dx     =  dx_abs;
    int dy     = -dy_abs;
    int err    = dx + dy;
    int x = x0, y = y0;

    while (1) {
        if (y >= 2 && y < rows - 1 && x >= 0 && x < cols) {
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(y, x, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
}

/* Draws one triangle's three sides, coloured by its shape so acute and obtuse
 * stay tellable apart. */
static void draw_triangle_edges(const Screen *sc, const Triangle *t)
{
    int ax, ay, bx, by, cx, cy;
    world_to_screen(sc, t->ax, t->ay, &ax, &ay);
    world_to_screen(sc, t->bx, t->by, &bx, &by);
    world_to_screen(sc, t->cx, t->cy, &cx, &cy);

    int ramp = (t->kind == TRI_ACUTE) ? ACUTE_EDGE_RAMP : OBTUSE_EDGE_RAMP;
    int pair = PAIR_RAMP_BASE + ramp;
    int attr = A_BOLD;

    /* each side gets the character that matches its slant */
    draw_line_styled(ax, ay, bx, by, sc->rows, sc->cols, pair, attr,
                     line_glyph_for(bx - ax, by - ay));
    draw_line_styled(bx, by, cx, cy, sc->rows, sc->cols, pair, attr,
                     line_glyph_for(cx - bx, cy - by));
    draw_line_styled(cx, cy, ax, ay, sc->rows, sc->cols, pair, attr,
                     line_glyph_for(ax - cx, ay - cy));
}

/* Drops a single dot at the triangle's middle (the average of its corners),
 * coloured by shape. */
static void draw_triangle_centroid(const Screen *sc, const Triangle *t)
{
    float wcx = (t->ax + t->bx + t->cx) / 3.0f;
    float wcy = (t->ay + t->by + t->cy) / 3.0f;
    int sx, sy;
    world_to_screen(sc, wcx, wcy, &sx, &sy);
    if (sy < 2 || sy >= sc->rows - 1) return;
    if (sx < 0 || sx >= sc->cols)     return;

    int ramp = (t->kind == TRI_ACUTE)
             ? ACUTE_CENTER_RAMP
             : OBTUSE_CENTER_RAMP;
    int pair = PAIR_RAMP_BASE + ramp;
    char glyph = (t->kind == TRI_ACUTE) ? '.' : 'o';
    int attr = (t->kind == TRI_OBTUSE) ? A_BOLD : A_NORMAL;

    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

static void scene_draw(const Screen *sc, const Tiling *tl, GlyphSet glyph)
{
    int n = tl->n[tl->active];
    const Triangle *T = tl->buf[tl->active];

    bool draw_edges   = (glyph == GLYPH_EDGES
                      || glyph == GLYPH_BOTH);
    bool draw_centers = (glyph == GLYPH_CENTERS
                      || glyph == GLYPH_BOTH);

    if (draw_edges) {
        for (int i = 0; i < n; i++)
            draw_triangle_edges(sc, &T[i]);
    }
    if (draw_centers) {
        for (int i = 0; i < n; i++)
            draw_triangle_centroid(sc, &T[i]);
    }
}

/* Top row: title on the left, live status on the right. The status word says
 * where the animation is — PAUSED, GROW (still subdividing), or HOLD (finished,
 * about to loop). */
static void hud_draw_status_line(const Screen *sc, const Scene *s,
                                 double fps, int sim_fps)
{
    bool growing = s->defl_level < pattern_depth(s->current_pattern);
    const char *state_str = s->paused ? "PAUSED"
                          : growing    ? "GROW  "
                                       : "HOLD  ";

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    mvprintw(0, 1, " PENROSE TILING ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Second row: the things the keys change — pattern, glyph mode, theme, a strip
 * showing the theme's colours, and the round/triangle counts. Each field is
 * printed, then x steps past it to the next column. */
static void hud_draw_param_line(const Screen *sc, const Scene *s)
{
    (void)sc;
    int n = s->tiling.n[s->tiling.active];
    int x = 1;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-5s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 16;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " glyph:%-5s ", glyph_set_name(s->current_glyph));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 14;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    /* Colour strip — one character in each of the theme's eight shades. */
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " ramp:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 6;
    static const char ramp_glyphs[8] = { '`', '.', ',', ':', '-', 'o', '#', '@' };
    for (int i = 0; i < 8; i++) {
        int p = PAIR_RAMP_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, (chtype)(unsigned char)ramp_glyphs[i]);
        attroff(COLOR_PAIR(p) | A_BOLD);
        x++;
    }

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             "  K=%d/%d  N=%d ",
             s->defl_level, pattern_depth(s->current_pattern), n);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row: the full list of keys you can press. */
static void hud_draw_key_hints(const Screen *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  g/G:glyph  t/T:theme  +/-:speed  spc:pause  r:reseed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, &s->tiling, s->current_glyph);   /* the tiling */
    hud_draw_status_line(sc, s, fps, sim_fps);      /* top row    */
    hud_draw_param_line(sc, s);                     /* middle row */
    hud_draw_key_hints(sc);                          /* bottom row */
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §6  APP — keys, signals, and the main loop ────────────────────────── */
/* main is where everything comes together each frame. Keys and signals       */
/* change things between frames, outside the loop's tick.                      */

/*
 * App — the whole running program: the scene, the screen, and a couple of
 * flags. running and need_resize are flipped from inside signal handlers, so
 * they're volatile sig_atomic_t (the only thing a handler may safely touch).
 */
typedef struct {
    Scene                 scene;        /* the tiling being shown          */
    Screen                screen;       /* where it's drawn                */
    int                   sim_fps;      /* how often the sim ticks ([ / ])  */
    volatile sig_atomic_t running;      /* 0 = time to quit                */
    volatile sig_atomic_t need_resize;  /* 1 = window was resized           */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reseed(s);                               break;

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
        s->current_theme = wrap_inc(s->current_theme, N_THEMES);
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = wrap_dec(s->current_theme, N_THEMES);
        theme_apply(s->current_theme);
        break;

    case 'n': case 'N':
        s->current_pattern = (Pattern)wrap_inc((int)s->current_pattern, N_PATTERNS);
        scene_begin_cycle(s);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)wrap_dec((int)s->current_pattern, N_PATTERNS);
        scene_begin_cycle(s);
        break;

    case 'g':
        s->current_glyph = (GlyphSet)wrap_inc((int)s->current_glyph, N_GLYPH_SETS);
        break;
    case 'G':
        s->current_glyph = (GlyphSet)wrap_dec((int)s->current_glyph, N_GLYPH_SETS);
        break;

    default: break;
    }
    return true;
}

/* Starts everything up once before the loop: random seed, signal handlers, the
 * cleanup-on-exit hook, then opens the screen and lays down the first tiling. */
static void app_init(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    scene_init(&app->scene);
}

int main(void)
{
    App *app = &g_app;
    app_init(app);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;   /* time that's passed but the sim hasn't used yet */
    int64_t fps_accum   = 0;   /* time counted toward the next fps reading        */
    int     frame_count = 0;
    double  fps_display = 0.0;

    const int64_t max_dt_ns      = (int64_t)MAX_FRAME_DT_MS * NS_PER_MS;
    const int64_t frame_cap_ns   = NS_PER_SEC / RENDER_FPS_CAP;

    while (app->running) {

        /* Service a pending resize before timing this frame. */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* How long since last frame, capped so a hiccup can't snowball. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > max_dt_ns) dt = max_dt_ns;

        /* Step the animation forward in fixed-size ticks. */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* Refresh the displayed fps once per FPS_UPDATE_MS window. */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* Sleep to hold the render frame cap, then draw and present. */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(frame_cap_ns - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* Drain one key event. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
