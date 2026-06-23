/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * apollonian.c — an Apollonian gasket drawn in the terminal.
 *
 * Start with a big circle and a few smaller circles snug inside it,
 * all touching.  Wherever three circles touch they leave a little
 * curved gap; there's exactly one circle that fits perfectly into
 * that gap, touching all three.  Drop it in, and now you have new,
 * smaller gaps to fill.  Keep going and you get this lacy fractal.
 * The math that finds each gap-filler is Descartes' Circle Theorem.
 *
 * Keys:
 *   q  quit         p  pause            r  reset (rewinds the sweep)
 *   +/- depth       t  cycle palette    n  cycle seed pack
 *   ,/. hue speed   f  toggle fill
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/fractals/apollonian.c \
 *       -o apollonian -lncurses -lm
 *
 * References (things the code can't tell you):
 *   ── Mumford, Series & Wright, "Indra's Pearls", Cambridge, 2002 —
 *      the best book for SEEING what this draws.
 *   ── Lagarias, Mallows & Wilks, "Beyond the Descartes Circle
 *      Theorem", AMM 109 (2002), 338–361 — the curvature and centre
 *      formulas used in §3.  https://www.jstor.org/stable/2695498
 *   ── Graham et al., "Apollonian Circle Packings: Number Theory",
 *      JNT 100 (2003), 1–45 — why the integer seed packs work.
 *   ── Soddy, "The Kiss Precise", Nature 137 (1936), 1021 — Descartes'
 *      theorem stated as a poem; the charming original.
 *
 * §1 types & data   §2 math   §3 gasket
 * §4 time & sweep   §5 view   §6 render & HUD   §7 scene & main
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>           /* snprintf                                    */
#include <stdlib.h>
#include <time.h>

/* ===================================================================== */
/* §1  types & data                                                       */
/* ===================================================================== */

/* ---- sizing & depth ----------------------------------------------- */
#define MAX_CIRCLES         8000
#define DEPTH_MAX              7
#define DEPTH_PAIRS            7        /* matches CP_D1..CP_D7         */
#define HUD_ROWS               1        /* top HUD line                 */

/* ---- frame pacing ------------------------------------------------- */
#define RENDER_NS           (1000000000LL / 30)   /* ~30 fps budget     */
#define NS_PER_SECOND       1.0e9f

/* ---- when to accept a candidate circle ---------------------------- */
#define MIN_VALID_CURVATURE   1.0f     /* below this, the circle is bigger than
                                          the outer disc — not a real child   */
#define EPS_NONZERO_CURV      1e-6f    /* a near-zero curvature means a line,
                                          not a circle — throw it out         */
#define GEOM_TANGENT_TOL      1e-3f    /* wiggle room for "just touching" vs
                                          "overlapping" (in gasket units)     */
#define DEDUP_REL_TOL         2e-3f    /* how close two circles must be to
                                          count as the same one               */
#define DEDUP_SCALE_BIAS      2.0f     /* keeps that tolerance sane for the
                                          big outer circle (curvature near 0) */

/* ---- terminal cell shape ------------------------------------------ *
 * Terminal characters are about twice as tall as they are wide.  So a
 * step of one row covers twice the distance of a step of one column.
 * We do all our math in "column widths" and convert when we touch rows. */
#define ASPECT_CELL_HEIGHT    2.0f     /* rows -> column-widths              */
#define ASPECT_INV            0.5f     /* column-widths -> rows             */

/* ---- drawing thresholds & glyphs ---------------------------------- */
#define SUBPIXEL_RADIUS       0.5f     /* smaller than this, just draw a dot */
#define BOUNDARY_BAND_CELLS   1.0f     /* how thick the bold disc edge is    */
#define DISC_MATH_CLIP        0.0f     /* hard edge, no soft fade            */
#define ROUNDING_BIAS         0.5f     /* add then truncate = round to nearest*/
#define SLOPE_SQRT3           1.73f    /* picks where / \ flip to | or -     */

#define GLYPH_SUBPIXEL        '.'
#define GLYPH_DISC_INTERIOR   '#'
#define GLYPH_DISC_BOUNDARY   '@'

/* ---- how finely to trace a circle outline ------------------------- */
#define RING_BASE_STEPS       8        /* fewest dots, even on a tiny ring   */
#define RING_STEPS_PER_RAD    2.5f     /* more dots as the ring gets bigger  */
#define RING_MAX_STEPS        1024     /* but cap it so huge rings stay cheap */

/* ---- the grow-in animation ---------------------------------------- */
#define SWEEP_PER_DEPTH_S     1.5f     /* seconds for one layer to grow in   */
#define SWEEP_HOLD_S          3.0f     /* pause on the full picture, then redo*/

/* ---- HUD & input ------------------------------------------------- */
#define HUD_BUFFER_BYTES      220
#define KEY_ESCAPE            27
#define HUE_SPEED_MAX         2.0f
#define HUE_SPEED_STEP        0.1f
#define HUE_SPEED_DEFAULT     0.2f

/* ---- defaults ----------------------------------------------------- */
#define SEED_DEFAULT          0
#define PALETTE_DEFAULT       0

/* ---- Circle — one circle in the gasket ---------------------------- *
 *
 * One circle.  We describe it by curvature (k = 1 / radius) instead
 * of radius, because that's the form Descartes' theorem works in.
 * The sign of the curvature is meaningful:
 *   • k > 0  — a normal circle sitting inside the gasket
 *   • k < 0  — the big outer circle that holds everything else
 *              (the standard convention: a circle you live inside
 *               gets a negative curvature)
 *
 * Storage quirk: instead of the centre point itself, we store the
 * centre already multiplied by the curvature (the "kz" values).
 * Descartes' centre formula is written in terms of those products,
 * so keeping them pre-multiplied saves a divide every time we run
 * the formula.  We undo it once at draw time to get the real centre.
 *
 * Fields:
 *   k       — curvature, 1/radius.  Radius is 1/|k|; negative means
 *             this is the outer circle.
 *   zx, zy  — the centre times the curvature.  Real centre is
 *             (zx/k, zy/k).
 *   depth   — which round of the build produced this circle:
 *               0  the outer circle
 *               1  the first few circles from the seed pack
 *               2+ everything found by filling gaps
 *             The grow-in animation uses this so each layer fades
 *             in one after another.
 *
 * Ref: Lagarias, Mallows & Wilks, AMM 109 (2002), eqs. (1.4), (4.1). */
typedef struct {
    float k;
    float zx, zy;
    int   depth;
} Circle;

/* ---- DescartesTriple — three touching circles, one gap to fill --- *
 *
 * A reminder to ourselves: "these three circles all touch, go find
 * the circle that fits in the gap between them."  It's one item of
 * work on a to-do list.  We pull one off, fill its gap, and the new
 * circle creates more gaps that go back on the list.
 *
 * We point at the three circles by their array slot number, not by
 * a pointer, because the circles array keeps growing as we build —
 * a stored pointer could go stale, an index won't.
 *
 * Fields:
 *   i1, i2, i3 — slot numbers of the three touching circles.  Order
 *                doesn't matter.
 *   depth      — the layer number to stamp on whatever circle this
 *                gap produces (set by whoever added this to-do item).
 *
 * Important assumption: the three circles really do touch each other.
 * The seed packs hand-pick valid starting triples, and every gap we
 * fill yields a circle that touches its three neighbours, so this
 * stays true automatically as the build goes. */
typedef struct { int i1, i2, i3, depth; } DescartesTriple;

/* ---- Gasket — the whole packing, plus its build state ------------ *
 *
 * Everything we've built so far: the circles we've placed, the
 * gaps still waiting to be filled, and how deep we're willing to go.
 * Keeping it all in one struct means every operation is just a
 * function that takes a Gasket* — no globals to chase.
 *
 * The build, in plain steps:
 *     while there's a gap to fill:
 *         take one off the list
 *         work out the circle that fits it
 *         if it's a real new circle, add it and put its new gaps
 *         on the list
 * (Descartes' formula spits out four candidate circles per gap;
 *  usually one is the new gap-filler, one is a circle we already
 *  have, and two are garbage.  The checks in §3 keep the good one.)
 *
 * The build runs all at once — at startup or whenever you change a
 * setting — so we just use fixed-size arrays and never malloc.
 *
 * Fields:
 *   circles[]  — every circle so far.  Slot 0 is always the outer
 *                circle; the rest are in the order we found them,
 *                which is also the order we draw them.
 *   n          — how many slots in circles[] are used.
 *   pending[]  — the to-do list of gaps.  We always take the most
 *                recently added one (it's a stack), which keeps the
 *                list short.  Sized 4x MAX_CIRCLES: each gap can add
 *                up to three more, so there's plenty of headroom.
 *   n_pending  — how many slots in pending[] are used.
 *   depth_max  — how many layers deep to build (1..DEPTH_MAX).  The
 *                +/- keys change it; lower means a faster, sparser
 *                picture.
 *
 * Ref: Graham et al., JNT 100 (2003), §2. */
typedef struct {
    Circle           circles[MAX_CIRCLES];
    int              n;
    DescartesTriple  pending[MAX_CIRCLES * 4];
    int              n_pending;
    int              depth_max;
} Gasket;

/* ---- Palette — a colour theme, one colour per depth layer -------- *
 *
 * A named list of seven colours, one for each depth layer (1..7).
 * Keeping colour choices here, away from the drawing code, means a
 * new theme is just one more entry in g_palettes[] and the 't' key
 * cycling through them.
 *
 * Colour choice rule (project CLAUDE.md "Theme Palette Brightness"):
 * every colour has to stay readable on a black background.  The very
 * darkest xterm colours (16-23 and 232-239) disappear, so we never
 * use them; the next step up (24-29, 240-243) is only safe for the
 * biggest, lowest layer; anything 30+ / 244+ is fine anywhere.
 *
 * Seven colours because we build seven layers deep; anything finer
 * than that is too small to see and just reuses the last colour.
 *
 * Fields:
 *   name      — short label shown in the HUD.
 *   colors[]  — the seven xterm colour codes, outer layer first.
 *               Most themes fade one tone across the seven;
 *               "rainbow" jumps between distinct hues. */
typedef struct {
    const char *name;
    int         colors[DEPTH_PAIRS];
} Palette;

/* ---- SeedPack — a named starting arrangement of circles ---------- *
 *
 * A function that drops in the first handful of touching circles and
 * lists their gaps.  That starting arrangement decides the whole
 * fractal, and different starts make different-looking ones, so we
 * keep a few and let the 'n' key switch between them.  Each is a
 * function (not just data) because laying out the circles takes a
 * bit of arithmetic — easier to write as code than as a table.
 *
 * Fields:
 *   name   — short label shown in the HUD.
 *   seed   — fills in the starting circles and gaps from scratch
 *            (gasket_build resets the counters before calling it).
 *
 * The three shipped here:
 *   classic — the textbook one, symmetric left-right and top-bottom.
 *   trefoil — three equal circles with three-fold pinwheel symmetry.
 *   coxeter — a lopsided one; the build fills in the missing mirror
 *             circles as it goes.
 *
 * Ref: Graham et al., JNT 100 (2003), Table 1 — the integer starting
 * sets these are drawn from. */
typedef struct {
    const char *name;
    void (*seed)(Gasket *);
} SeedPack;

/* ---- Viewport — how to place the gasket on the screen ------------ *
 *
 * The recipe for turning a point in the gasket's own coordinates into
 * a row and column on screen.  Keeping it here means the drawing code
 * never has to think about terminal size; on a window resize we just
 * refit this one struct and everything follows.
 *
 * Two things it sorts out:
 *   1) Characters are taller than wide, so a circle drawn naively
 *      would look like a tall egg.  We squash the vertical direction
 *      to compensate.
 *   2) The gasket should fill the window without spilling off either
 *      edge, whatever the window's shape.  We take whichever fits
 *      tighter, width or height, and use that for both.
 *
 * Both come down to one number, `scale`: how many screen columns wide
 * one gasket unit is.  Bigger window, bigger scale, bigger picture.
 *
 * Fields:
 *   rows, cols  — the terminal's current size, in characters.
 *   cx_center,
 *   cy_center   — the screen cell where the gasket's centre (0,0)
 *                 lands, sitting between the HUD line and the hint
 *                 line.
 *   scale       — columns per gasket unit (see above).
 *
 * Ref: Foley et al., "Computer Graphics", 3rd ed., §6 — fitting a
 * picture into a window while keeping its shape. */
typedef struct {
    int   rows, cols;
    int   cx_center, cy_center;
    float scale;
} Viewport;

/* ---- Scene — everything the running program needs ---------------- *
 *
 * One struct with the whole program's state: the gasket, how to put
 * it on screen, the animation clock, and the user's current choices.
 * It lets the main loop read as a short to-do list: handle a key,
 * advance time, draw a frame.
 *
 * Fields, grouped by what they're for:
 *
 *   The shape:
 *     gasket  — the circles and their build state.
 *     seed    — which starting arrangement (index into g_seeds[]).
 *               'n' cycles it.
 *
 *   How it looks:
 *     view    — the screen-placement recipe; refit on resize.
 *     palette — which colour theme (index into g_palettes[]).
 *               't' cycles it and reloads the colours.
 *     filled  — true draws solid discs, false draws hollow rings.
 *               The outer circle is always a ring either way.
 *               'f' toggles it.
 *
 *   Animation:
 *     t         — seconds since the last reset.  Drives both the
 *                 grow-in and the colour rotation.  Frozen while
 *                 paused.
 *     paused    — 'p' toggles; HUD shows "PAUSED".
 *     hue_speed — how fast colours rotate, in steps per second.
 *                 Negative rotates the other way.  ',' and '.'
 *                 nudge it, kept within +/-HUE_SPEED_MAX. */
typedef struct {
    /* what we're packing */
    Gasket   gasket;
    int      seed;

    /* how we view it */
    Viewport view;
    int      palette;
    bool     filled;

    /* animation */
    float    t;
    bool     paused;
    float    hue_speed;
} Scene;

/* ncurses colour-pair slot numbers: seven for the depth layers,
 * two for the HUD text. */
enum { CP_D1 = 1, CP_D2, CP_D3, CP_D4, CP_D5, CP_D6, CP_D7, CP_HUD, CP_HINT };

/* ---- the colour themes -------------------------------------------- */
static const Palette g_palettes[] = {
    { "rainbow", { 226, 118,  51,  33,  93, 201, 196 } },   /* yellow→red       */
    { "ocean",   {  31,  38,  45,  51, 117, 159, 195 } },   /* teal→pale cyan   */
    { "fire",    {  88, 124, 160, 202, 208, 220, 226 } },   /* deep red→yellow  */
    { "forest",  {  28,  34,  40,  46,  82, 154, 226 } },   /* dark green→yellow*/
    { "magma",   {  53,  90, 125, 162, 199, 213, 225 } },   /* purple→pink      */
    { "mono",    { 240, 244, 248, 250, 252, 254, 255 } },   /* grayscale        */
};
#define N_PALETTES ((int)(sizeof g_palettes / sizeof g_palettes[0]))

/* ---- seed packs (forward decls — bodies in §3) -------------------- */
static void seed_classic(Gasket *g);
static void seed_trefoil(Gasket *g);
static void seed_coxeter(Gasket *g);

static const SeedPack g_seeds[] = {
    { "classic", seed_classic },
    { "trefoil", seed_trefoil },
    { "coxeter", seed_coxeter },
};
#define N_SEEDS ((int)(sizeof g_seeds / sizeof g_seeds[0]))

/* ===================================================================== */
/* §2  math — complex arithmetic primitives                               */
/* ===================================================================== */

/* Square root of a complex number.  A number can have two square
 * roots; we always return the same one (the one pointing "rightward"),
 * which is what Descartes' formula below expects. */
static void complex_sqrt(float ax, float ay, float *rx, float *ry)
{
    float magnitude_sqrt = sqrtf(sqrtf(ax*ax + ay*ay));
    float half_angle     = atan2f(ay, ax) * 0.5f;
    *rx = magnitude_sqrt * cosf(half_angle);
    *ry = magnitude_sqrt * sinf(half_angle);
}

static void complex_mul(float ax, float ay, float bx, float by,
                        float *rx, float *ry)
{
    *rx = ax * bx - ay * by;
    *ry = ax * by + ay * bx;
}

/* ===================================================================== */
/* §3  gasket — Descartes recursion and seed packs                        */
/* ===================================================================== */

/* Do we already have this circle?  Descartes hands back circles we've
 * seen before, so we check before adding.  The "close enough" margin
 * grows with the circle's curvature, because the math gets fuzzier for
 * tightly-curved circles — a fixed margin would miss some duplicates. */
static bool gasket_has(const Gasket *g, float k, float kzx, float kzy)
{
    for (int i = 0; i < g->n; i++) {
        const Circle *o = &g->circles[i];
        float scale = fabsf(o->k) + fabsf(k) + DEDUP_SCALE_BIAS;
        float eps   = DEDUP_REL_TOL * scale;
        if (fabsf(o->k  - k  ) < eps &&
            fabsf(o->zx - kzx) < eps &&
            fabsf(o->zy - kzy) < eps)
            return true;
    }
    return false;
}

/* Sanity-check a candidate circle: does it actually fit?  Descartes'
 * formula is just algebra and sometimes returns circles that don't
 * really belong, so we check it against every circle we already have.
 * It must sit fully inside the outer circle, and it must not overlap
 * any of the inner ones (touching is fine). */
static bool gasket_admits(const Gasket *g, float k4, float c4x, float c4y)
{
    float r4 = 1.f / k4;

    for (int i = 0; i < g->n; i++) {
        const Circle *e = &g->circles[i];
        float re  = 1.f / fabsf(e->k);
        float cex = e->zx / e->k;
        float cey = e->zy / e->k;
        float dx  = c4x - cex;
        float dy  = c4y - cey;
        float d   = sqrtf(dx*dx + dy*dy);

        if (e->k < 0.f) {
            if (d + r4 > re + GEOM_TANGENT_TOL) return false;   /* pokes out of the outer */
        } else {
            if (d < r4 + re - GEOM_TANGENT_TOL) return false;   /* overlaps an inner one  */
        }
    }
    return true;
}

/* ---- Descartes' Circle Theorem ------------------------------------ *
 * Given three circles that touch, this is the part of the theorem that
 * gives the curvature (1/radius) of the fourth circle in the gap.  The
 * sign `sk` picks one of the two answers it offers.  Returns false if
 * the formula can't give a real answer for this triple. */
static bool descartes_curvature_root(float k1, float k2, float k3,
                                     int sk, float *k4_out)
{
    float radicand = k1*k2 + k2*k3 + k3*k1;
    if (radicand < 0.f) return false;
    *k4_out = k1 + k2 + k3 + sk * 2.f * sqrtf(radicand);
    return true;
}

/* The bit that goes under the square root in the centre formula below.
 * It works directly on the pre-multiplied centre values we store. */
static void descartes_centre_radicand(const Circle *c1, const Circle *c2,
                                      const Circle *c3,
                                      float *rx, float *ry)
{
    float p12x, p12y, p23x, p23y, p31x, p31y;
    complex_mul(c1->zx, c1->zy, c2->zx, c2->zy, &p12x, &p12y);
    complex_mul(c2->zx, c2->zy, c3->zx, c3->zy, &p23x, &p23y);
    complex_mul(c3->zx, c3->zy, c1->zx, c1->zy, &p31x, &p31y);
    *rx = p12x + p23x + p31x;
    *ry = p12y + p23y + p31y;
}

/* The other half of the theorem: where the new circle's centre is.
 * Like the curvature, it has two answers and `sz` picks one. */
static void descartes_centre_root(const Circle *c1, const Circle *c2,
                                  const Circle *c3,
                                  int sz, float *kz4x_out, float *kz4y_out)
{
    float sum_kzx = c1->zx + c2->zx + c3->zx;
    float sum_kzy = c1->zy + c2->zy + c3->zy;

    float rad_x, rad_y;
    descartes_centre_radicand(c1, c2, c3, &rad_x, &rad_y);

    float root_x, root_y;
    complex_sqrt(rad_x, rad_y, &root_x, &root_y);

    *kz4x_out = sum_kzx + sz * 2.f * root_x;
    *kz4y_out = sum_kzy + sz * 2.f * root_y;
}

/* ---- gasket mutation helpers -------------------------------------- */

static void gasket_append_circle(Gasket *g, float k, float kzx, float kzy, int depth)
{
    Circle *c = &g->circles[g->n++];
    c->k     = k;
    c->zx    = kzx;
    c->zy    = kzy;
    c->depth = depth;
}

static DescartesTriple gasket_pop_triple(Gasket *g)
{
    return g->pending[--g->n_pending];
}

/* A freshly added circle makes three new gaps — one with each pair of
 * the three circles it just landed between.  Add each to the to-do
 * list, unless we've hit the depth limit or run out of room. */
static void gasket_push_subgaps(Gasket *g, const DescartesTriple *parent, int child_idx)
{
    int child_depth = parent->depth + 1;
    if (child_depth > g->depth_max) return;

    int queue_capacity = (int)(sizeof g->pending / sizeof g->pending[0]);
    if (g->n_pending + 3 > queue_capacity) return;

    g->pending[g->n_pending++] =
        (DescartesTriple){parent->i1, parent->i2, child_idx, child_depth};
    g->pending[g->n_pending++] =
        (DescartesTriple){parent->i1, parent->i3, child_idx, child_depth};
    g->pending[g->n_pending++] =
        (DescartesTriple){parent->i2, parent->i3, child_idx, child_depth};
}

/* Try one of the four answers Descartes gives for a gap, and add the
 * circle if it checks out.  Returns true if a circle was added.  Reads
 * top to bottom as: work out the circle, then a string of "is it real?
 * is it new? does it fit?" checks before we keep it. */
static bool gasket_try_descartes(Gasket *g,
                                 int i1, int i2, int i3,
                                 int sk, int sz, int depth)
{
    if (g->n >= MAX_CIRCLES) return false;

    const Circle *c1 = &g->circles[i1];
    const Circle *c2 = &g->circles[i2];
    const Circle *c3 = &g->circles[i3];

    float k4;
    if (!descartes_curvature_root(c1->k, c2->k, c3->k, sk, &k4))  return false;
    if (k4 < MIN_VALID_CURVATURE)                                  return false;
    if (depth > g->depth_max)                                      return false;

    float kz4x, kz4y;
    descartes_centre_root(c1, c2, c3, sz, &kz4x, &kz4y);

    if (fabsf(k4) < EPS_NONZERO_CURV)                  return false;
    if (gasket_has(g, k4, kz4x, kz4y))                 return false;
    if (!gasket_admits(g, k4, kz4x / k4, kz4y / k4))   return false;

    gasket_append_circle(g, k4, kz4x, kz4y, depth);
    return true;
}

/* Try all four of Descartes' answers for one gap, and for every real
 * new circle, queue up the gaps it just created.  Returns true if any
 * of the four panned out. */
static bool gasket_try_all_descartes_signs(Gasket *g, const DescartesTriple *t)
{
    bool any = false;
    for (int sk = -1; sk <= 1; sk += 2)
        for (int sz = -1; sz <= 1; sz += 2)
            if (gasket_try_descartes(g, t->i1, t->i2, t->i3, sk, sz, t->depth)) {
                gasket_push_subgaps(g, t, g->n - 1);
                any = true;
            }
    return any;
}

/* Do one unit of work: take the next gap off the list and fill it.
 * Returns true while there's still work; call it in a loop until it
 * returns false, meaning the gasket is finished. */
static bool gasket_step(Gasket *g)
{
    while (g->n_pending > 0) {
        DescartesTriple t = gasket_pop_triple(g);
        if (t.depth > g->depth_max) continue;
        if (gasket_try_all_descartes_signs(g, &t)) return true;
    }
    return false;
}

/* ---- seed packs --------------------------------------------------- */

/* CLASSIC — the textbook gasket: one outer circle, a left and right
 * pair, and a top and bottom pair.  We skip the gaps that would just
 * rediscover circles we've already placed by hand (top and bottom
 * don't touch each other, and the left-right-outer gap is exactly
 * where top and bottom already sit). */
static void seed_classic(Gasket *g)
{
    g->circles[0] = (Circle){ -1.f, 0.f,  0.f, 0 };
    g->circles[1] = (Circle){  2.f, 1.f,  0.f, 1 };   /* right  */
    g->circles[2] = (Circle){  2.f,-1.f,  0.f, 1 };   /* left   */
    g->circles[3] = (Circle){  3.f, 0.f,  2.f, 1 };   /* top    */
    g->circles[4] = (Circle){  3.f, 0.f, -2.f, 1 };   /* bottom */
    g->n          = 5;

    g->pending[g->n_pending++] = (DescartesTriple){0, 1, 3, 2};
    g->pending[g->n_pending++] = (DescartesTriple){0, 2, 3, 2};
    g->pending[g->n_pending++] = (DescartesTriple){0, 1, 4, 2};
    g->pending[g->n_pending++] = (DescartesTriple){0, 2, 4, 2};
    g->pending[g->n_pending++] = (DescartesTriple){1, 2, 3, 2};
    g->pending[g->n_pending++] = (DescartesTriple){1, 2, 4, 2};
}

/* TREFOIL — three identical circles spaced evenly inside the outer
 * one, 120 degrees apart like a three-bladed pinwheel: one up top,
 * two along the bottom.  The numbers below place those three centres;
 * `h` is just a shorthand the placement uses. */
static void seed_trefoil(Gasket *g)
{
    float r              = 2.f * sqrtf(3.f) - 3.f;
    float k              = 1.f / r;
    float h              = k - 1.f;
    float lower_kz_x     = h * sqrtf(3.f) * 0.5f;
    float lower_kz_y     = -h * 0.5f;

    g->circles[0] = (Circle){ -1.f, 0.f,          0.f,        0 };
    g->circles[1] = (Circle){  k,   0.f,          h,          1 };  /* top         */
    g->circles[2] = (Circle){  k,  -lower_kz_x,   lower_kz_y, 1 };  /* lower-left  */
    g->circles[3] = (Circle){  k,   lower_kz_x,   lower_kz_y, 1 };  /* lower-right */
    g->n          = 4;

    g->pending[g->n_pending++] = (DescartesTriple){0, 1, 2, 2};
    g->pending[g->n_pending++] = (DescartesTriple){0, 1, 3, 2};
    g->pending[g->n_pending++] = (DescartesTriple){0, 2, 3, 2};
    g->pending[g->n_pending++] = (DescartesTriple){1, 2, 3, 2};
}

/* COXETER — a lopsided starting set of three differently-sized
 * circles.  The build fills in the missing mirror circles as it runs. */
static void seed_coxeter(Gasket *g)
{
    g->circles[0] = (Circle){ -1.f, 0.f, 0.f, 0 };
    g->circles[1] = (Circle){  2.f, 1.f, 0.f, 1 };
    g->circles[2] = (Circle){  3.f, 0.f, 2.f, 1 };
    g->circles[3] = (Circle){  6.f, 3.f, 4.f, 1 };
    g->n          = 4;

    g->pending[g->n_pending++] = (DescartesTriple){0, 1, 2, 2};
    g->pending[g->n_pending++] = (DescartesTriple){0, 1, 3, 2};
    g->pending[g->n_pending++] = (DescartesTriple){0, 2, 3, 2};
    g->pending[g->n_pending++] = (DescartesTriple){1, 2, 3, 2};
}

/* Build the entire gasket synchronously under the given seed pack. */
static void gasket_build(Gasket *g, const SeedPack *pack)
{
    g->n         = 0;
    g->n_pending = 0;
    pack->seed(g);
    while (gasket_step(g)) { }
}

/* ===================================================================== */
/* §4  time & sweep                                                       */
/* ===================================================================== */

static long long clock_ns_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* How many layers deep the grow-in has reached at time t.  Climbs
 * steadily from 0 up to the full depth, sits there a moment, then
 * jumps back to 0 and replays — over and over. */
static float sweep_visible_depth(float t, int depth_max)
{
    float dmax           = (float)depth_max;
    float ramp_duration  = SWEEP_PER_DEPTH_S * dmax;
    float cycle_duration = ramp_duration + SWEEP_HOLD_S;
    float t_in_cycle     = fmodf(t, cycle_duration);
    if (t_in_cycle < ramp_duration)
        return t_in_cycle / SWEEP_PER_DEPTH_S;
    return dmax;
}

/* How big to draw one circle right now, as a fraction of full size
 * (0 = invisible, 1 = full).  A circle starts as a speck when the
 * grow-in reaches its layer and swells to full size over the next
 * layer's worth of time. */
static float blossom_alpha(float visible_depth, int circle_depth)
{
    float a = visible_depth - (float)circle_depth + 1.0f;
    if (a < 0.f) return 0.f;
    if (a > 1.f) return 1.f;
    return a;
}

/* Pick the colour for a circle at this depth, shifted over time so the
 * colours rotate through the layers.  Returns a colour-pair slot
 * (CP_D1..CP_D7).  The doubled "+ N then % N" guards against a negative
 * result when hue_speed runs the rotation backwards. */
static int hue_shift(int depth, float t, float hue_speed)
{
    int clamped_depth = depth < 1            ? 1
                       : depth > DEPTH_PAIRS ? DEPTH_PAIRS
                       :                       depth;
    int rotation_offset = (int)floorf(t * hue_speed);
    return ((clamped_depth - 1 + rotation_offset) % DEPTH_PAIRS + DEPTH_PAIRS)
           % DEPTH_PAIRS + 1;
}

/* ===================================================================== */
/* §5  view — palette + viewport                                          */
/* ===================================================================== */

/* Load the current theme's seven colours into ncurses.  Called at
 * startup and each time the 't' key switches themes. */
static void palette_apply(int idx)
{
    if (COLORS >= 256) {
        const Palette *p = &g_palettes[idx];
        for (int i = 0; i < DEPTH_PAIRS; i++)
            init_pair(CP_D1 + i, p->colors[i], -1);
    } else {
        /* On an old 8-colour terminal, ignore themes and use basics. */
        init_pair(CP_D1, COLOR_YELLOW,  -1);
        init_pair(CP_D2, COLOR_GREEN,   -1);
        init_pair(CP_D3, COLOR_CYAN,    -1);
        init_pair(CP_D4, COLOR_BLUE,    -1);
        init_pair(CP_D5, COLOR_MAGENTA, -1);
        init_pair(CP_D6, COLOR_RED,     -1);
        init_pair(CP_D7, COLOR_RED,     -1);
    }
}

/* The HUD's own colours, set once — theme cycling doesn't touch them. */
static void palette_init_static_pairs(void)
{
    if (COLORS >= 256) {
        init_pair(CP_HUD,  226, -1);
        init_pair(CP_HINT,  51, -1);
    } else {
        init_pair(CP_HUD,  COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN,   -1);
    }
}

/* Work out the screen placement for the current window size: the
 * biggest the gasket can be drawn while still fitting inside, both
 * across and down. */
static void viewport_fit(Viewport *v, int rows, int cols)
{
    v->rows = rows;
    v->cols = cols;

    /* Both measured in column-widths so they're comparable. */
    float horizontal_budget = (float)(cols - 2) * 0.5f;
    float vertical_budget   = (float)(rows - HUD_ROWS - 2);

    v->scale     = horizontal_budget < vertical_budget
                     ? horizontal_budget : vertical_budget;
    v->cx_center = cols / 2;
    v->cy_center = (HUD_ROWS + rows - 1) / 2;
}

static int viewport_to_col(const Viewport *v, float x)
{
    return v->cx_center + (int)(x * v->scale + ROUNDING_BIAS);
}

static int viewport_to_row(const Viewport *v, float y)
{
    /* y axis points up; cells are 2× taller than wide so vertical
     * stride is half the horizontal scale. */
    return v->cy_center - (int)(y * v->scale * ASPECT_INV + ROUNDING_BIAS);
}

/* ===================================================================== */
/* §6  render — drawing primitives + HUD                                  */
/* ===================================================================== */

/* ---- common helpers ----------------------------------------------- */

/* Is this cell inside the play area (between HUD and hint row)? */
static bool in_play_area(const Viewport *v, int row, int col)
{
    return row >= HUD_ROWS && row < v->rows - 1
        && col >= 0        && col < v->cols;
}

/* Project a Circle into screen-space drawing parameters.
 * Recovers the gasket-space centre c = kz/k and radius r = 1/|k|
 * from the stored representation, then applies the viewport's
 * projection to produce:
 *   cx_col, cy_row — circle centre in terminal cells
 *   r_col          — horizontal radius in cell-WIDTH units
 *   r_row          — vertical radius (= r_col · ASPECT_INV) */
static void circle_screen_geometry(const Circle *c, const Viewport *v, float alpha,
                                   int *cx_col, int *cy_row,
                                   float *r_col, float *r_row)
{
    float r_gasket  = 1.f / fabsf(c->k);
    float cx_gasket = c->zx / c->k;
    float cy_gasket = c->zy / c->k;
    *cx_col = viewport_to_col(v, cx_gasket);
    *cy_row = viewport_to_row(v, cy_gasket);
    *r_col  = r_gasket * v->scale * alpha;
    *r_row  = *r_col * ASPECT_INV;
}

/* Slope-based outline glyph at angle θ on an aspect-1:2 ellipse —
 * picks `|`, `-`, `/`, or `\` for a smooth visual curve in 2:1 cells. */
static char ring_char(float theta)
{
    float tangent_x = -sinf(theta);
    float tangent_y =  ASPECT_INV * cosf(theta);
    float abs_x     = fabsf(tangent_x);
    float abs_y     = fabsf(tangent_y);
    if (abs_y > abs_x * SLOPE_SQRT3) return '|';
    if (abs_x > abs_y * SLOPE_SQRT3) return '-';
    return (tangent_x * tangent_y > 0.f) ? '\\' : '/';
}

/* Render a single dot at (cx, cy) — used for sub-pixel circles. */
static void plot_subpixel_dot(const Viewport *v, int cx_col, int cy_row, int pair)
{
    if (!in_play_area(v, cy_row, cx_col)) return;
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddch(cy_row, cx_col, GLYPH_SUBPIXEL);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* ---- ring (outline) renderer -------------------------------------- */

/* Choose how many parametric samples to take for a ring of given
 * radius — proportional to arc length, with a floor for very small
 * circles and a cap to keep huge ones bounded. */
static int ring_sample_count(float r_col)
{
    int n = (int)(2.f * (float)M_PI * r_col * RING_STEPS_PER_RAD) + RING_BASE_STEPS;
    if (n > RING_MAX_STEPS) n = RING_MAX_STEPS;
    return n;
}

/* Parametric outline ring around an aspect-1:2 ellipse. */
static void draw_ring(const Viewport *v,
                      int cx_col, int cy_row,
                      float r_col, float r_row,
                      int pair)
{
    if (r_col < SUBPIXEL_RADIUS) {
        plot_subpixel_dot(v, cx_col, cy_row, pair);
        return;
    }

    int n_samples = ring_sample_count(r_col);

    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int s = 0; s < n_samples; s++) {
        float theta = (2.f * (float)M_PI * (float)s) / (float)n_samples;
        int col = cx_col + (int)(r_col * cosf(theta) + ROUNDING_BIAS);
        int row = cy_row + (int)(r_row * sinf(theta) + ROUNDING_BIAS);
        if (in_play_area(v, row, col))
            mvaddch(row, col, (chtype)(unsigned char)ring_char(theta));
    }
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* ---- disc (flat fill) renderer ------------------------------------ */

/* Row range that the disc's bounding box covers, clipped to the play
 * area.  `+1` margin allows per-cell distance test to do the precise
 * clipping. */
static void disc_row_range(const Viewport *v, int cy_row, float r_row,
                           int *row_min, int *row_max)
{
    *row_min = (int)floorf((float)cy_row - r_row - 1.f);
    *row_max = (int)ceilf ((float)cy_row + r_row + 1.f);
    if (*row_min < HUD_ROWS)       *row_min = HUD_ROWS;
    if (*row_max >= v->rows - 1)   *row_max = v->rows - 2;
}

/* Column range a single scanline covers within the disc.  For a row
 * with dy_width offset from centre, the horizontal half-width is
 * √(r_col² − dy_width²) — standard scanline circle rasterisation. */
static void disc_col_range(const Viewport *v, int cx_col,
                           float dy_width, float r_col,
                           int *col_min, int *col_max)
{
    float half_width_sq = r_col * r_col - dy_width * dy_width;
    float half_width    = half_width_sq > 0.f ? sqrtf(half_width_sq) : 0.f;
    *col_min = (int)floorf((float)cx_col - half_width - 1.f);
    *col_max = (int)ceilf ((float)cx_col + half_width + 1.f);
    if (*col_min < 0)         *col_min = 0;
    if (*col_max >= v->cols)  *col_max = v->cols - 1;
}

/* Signed distance from (col, dy_width) to the disc boundary.
 *   < 0  inside the disc
 *   = 0  on the boundary
 *   > 0  outside the disc */
static float disc_signed_distance(int col, int cx_col, float dy_width, float r_col)
{
    float dx_width       = (float)(col - cx_col);
    float dist_to_center = sqrtf(dx_width * dx_width + dy_width * dy_width);
    return dist_to_center - r_col;
}

/* Place one cell of a filled disc — boundary band uses bold '@',
 * deeper interior uses plain '#'. */
static void plot_disc_cell(int row, int col, float signed_distance, int pair)
{
    bool   at_boundary = signed_distance > -BOUNDARY_BAND_CELLS;
    char   glyph       = at_boundary ? GLYPH_DISC_BOUNDARY : GLYPH_DISC_INTERIOR;
    attr_t weight      = at_boundary ? A_BOLD : 0;

    attron(COLOR_PAIR(pair) | weight);
    mvaddch(row, col, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | weight);
}

/* Flat-filled disc, strict-clipped at the math boundary.  Reads as
 * pseudocode: bound a scanline rectangle, walk each row, walk each
 * cell, plot the ones inside the math radius. */
static void draw_disc(const Viewport *v,
                      int cx_col, int cy_row,
                      float r_col, float r_row,
                      int pair)
{
    if (r_col < SUBPIXEL_RADIUS) {
        plot_subpixel_dot(v, cx_col, cy_row, pair);
        return;
    }

    int row_min, row_max;
    disc_row_range(v, cy_row, r_row, &row_min, &row_max);

    for (int row = row_min; row <= row_max; row++) {
        /* dy in cell-WIDTH units (disc is isotropic in visual space) */
        float dy_width = (float)(row - cy_row) * ASPECT_CELL_HEIGHT;
        if (fabsf(dy_width) > r_col + ASPECT_CELL_HEIGHT) continue;

        int col_min, col_max;
        disc_col_range(v, cx_col, dy_width, r_col, &col_min, &col_max);

        for (int col = col_min; col <= col_max; col++) {
            float d_signed = disc_signed_distance(col, cx_col, dy_width, r_col);
            if (d_signed > DISC_MATH_CLIP) continue;
            plot_disc_cell(row, col, d_signed, pair);
        }
    }
}

/* ---- per-circle dispatcher ---------------------------------------- */

/* Choose ring or disc renderer for one circle.  Outer is always a
 * ring (we never tint the play-area background); inner follows the
 * scene's `filled` toggle. */
static void draw_circle(const Circle *c, float alpha,
                        const Viewport *v, bool filled,
                        float t, float hue_speed)
{
    int   cx_col, cy_row;
    float r_col,  r_row;
    circle_screen_geometry(c, v, alpha, &cx_col, &cy_row, &r_col, &r_row);

    int pair = hue_shift(c->depth, t, hue_speed);

    bool render_as_ring = (c->k < 0.f) || !filled;
    if (render_as_ring)
        draw_ring(v, cx_col, cy_row, r_col, r_row, pair);
    else
        draw_disc(v, cx_col, cy_row, r_col, r_row, pair);
}

/* ---- scene-level render pass -------------------------------------- */

/* Walk every circle currently visible under the construction sweep
 * and dispatch it to the renderer. */
static void scene_render(const Scene *s)
{
    float visible_depth = sweep_visible_depth(s->t, s->gasket.depth_max);
    for (int i = 0; i < s->gasket.n; i++) {
        const Circle *c = &s->gasket.circles[i];
        float alpha = blossom_alpha(visible_depth, c->depth);
        if (alpha <= 0.f) continue;
        draw_circle(c, alpha, &s->view, s->filled, s->t, s->hue_speed);
    }
}

/* ---- HUD ---------------------------------------------------------- */

static void hud_draw_data_line(const Scene *s)
{
    char top[HUD_BUFFER_BYTES];
    int  len = snprintf(top, sizeof top,
        " seed:%s   palette:%s   fill:%s   depth:%d   circles:%d   hue:%+.1f   %s ",
        g_seeds[s->seed].name,
        g_palettes[s->palette].name,
        s->filled ? "on " : "off",
        s->gasket.depth_max, s->gasket.n,
        s->hue_speed,
        s->paused ? "PAUSED" : "      ");

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, s->view.cols - len, "%s", top);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

static void hud_draw_hint_line(const Scene *s)
{
    const char *hints =
        " q:quit  p:pause  r:reset  +/-:depth  ,/.:hue  t:palette  n:seed  f:fill ";
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(s->view.rows - 1, 0, "%s", hints);
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void hud_draw(const Scene *s)
{
    hud_draw_data_line(s);
    hud_draw_hint_line(s);
}

/* ===================================================================== */
/* §7  scene & main                                                       */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void on_exit_cleanup(void) { endwin(); }

/* ---- ncurses + signal setup --------------------------------------- */

static void app_init(void)
{
    atexit(on_exit_cleanup);
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGWINCH,on_signal);

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    start_color(); use_default_colors();
    palette_init_static_pairs();
}

/* ---- scene lifecycle ---------------------------------------------- */

/* Set the scene to its starting state and build the gasket. */
static void scene_init(Scene *s)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    viewport_fit(&s->view, rows, cols);

    s->seed             = SEED_DEFAULT;
    s->palette          = PALETTE_DEFAULT;
    s->filled           = true;
    s->t                = 0.f;
    s->paused           = false;
    s->hue_speed        = HUE_SPEED_DEFAULT;
    s->gasket.depth_max = DEPTH_MAX;

    gasket_build(&s->gasket, &g_seeds[s->seed]);
}

/* Re-fit the projection after a SIGWINCH and rebuild the gasket so
 * any cached screen-space sizing is consistent. */
static void scene_resize(Scene *s)
{
    endwin(); refresh();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    viewport_fit(&s->view, rows, cols);
    gasket_build(&s->gasket, &g_seeds[s->seed]);
}

static void scene_tick(Scene *s, float dt_seconds)
{
    if (!s->paused) s->t += dt_seconds;
}

/* ---- scene action helpers (named for what the user sees) ---------- */

/* Rebuild from the current seed pack and rewind the sweep clock. */
static void scene_rewind(Scene *s)
{
    gasket_build(&s->gasket, &g_seeds[s->seed]);
    s->t = 0.f;
}

static void scene_change_depth(Scene *s, int delta)
{
    int requested = s->gasket.depth_max + delta;
    if (requested < 1 || requested > DEPTH_MAX) return;
    s->gasket.depth_max = requested;
    scene_rewind(s);
}

static void scene_adjust_hue_speed(Scene *s, float delta)
{
    float v = s->hue_speed + delta;
    if (v < -HUE_SPEED_MAX) v = -HUE_SPEED_MAX;
    if (v >  HUE_SPEED_MAX) v =  HUE_SPEED_MAX;
    s->hue_speed = v;
}

static void scene_cycle_palette(Scene *s)
{
    s->palette = (s->palette + 1) % N_PALETTES;
    palette_apply(s->palette);
}

static void scene_cycle_seed(Scene *s)
{
    s->seed = (s->seed + 1) % N_SEEDS;
    scene_rewind(s);
}

/* React to a single key.  Each case is a named scene operation, so
 * the dispatch table reads as a mapping from input → intent. */
static void scene_handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case KEY_ESCAPE:  g_quit = 1;                                  break;
    case 'p': case 'P':                   s->paused = !s->paused;                      break;
    case 'r': case 'R':                   scene_rewind(s);                             break;
    case '+': case '=':                   scene_change_depth(s, +1);                   break;
    case '-': case '_':                   scene_change_depth(s, -1);                   break;
    case ',': case '<':                   scene_adjust_hue_speed(s, -HUE_SPEED_STEP);  break;
    case '.': case '>':                   scene_adjust_hue_speed(s, +HUE_SPEED_STEP);  break;
    case 't': case 'T':                   scene_cycle_palette(s);                      break;
    case 'n': case 'N':                   scene_cycle_seed(s);                         break;
    case 'f': case 'F':                   s->filled = !s->filled;                      break;
    default: break;
    }
}

/* ---- one frame ---------------------------------------------------- */

/* Composite the scene + HUD and flush to the terminal. */
static void frame_render(const Scene *s)
{
    erase();
    scene_render(s);
    hud_draw(s);
    wnoutrefresh(stdscr);
    doupdate();
}

/* Sleep what's left of the frame budget so we hold a stable rate. */
static void frame_pace_to_target(long long t_frame_start)
{
    long long elapsed = clock_ns_now() - t_frame_start;
    clock_sleep_ns(RENDER_NS - elapsed);
}

/* ---- main loop ---------------------------------------------------- */

int main(void)
{
    app_init();

    static Scene scene;
    scene_init(&scene);
    palette_apply(scene.palette);

    long long t_prev = clock_ns_now();
    while (!g_quit) {

        if (g_resize) { g_resize = 0; scene_resize(&scene); }

        scene_handle_key(&scene, getch());

        long long t_now    = clock_ns_now();
        float     dt       = (float)(t_now - t_prev) / NS_PER_SECOND;
        t_prev             = t_now;
        scene_tick(&scene, dt);

        frame_render(&scene);
        frame_pace_to_target(t_now);
    }
    return 0;
}
