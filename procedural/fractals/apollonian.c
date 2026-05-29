/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * apollonian.c — Apollonian Gasket: a recursive 2D circle packing.
 *
 * An Apollonian gasket starts with four mutually-tangent circles: an
 * outer disc and three inner discs tangent to it and to each other.
 * In every curvilinear gap formed by three mutually-tangent circles,
 * inscribe the unique fourth circle that touches all three.  Recurse
 * to fill the gasket up to a chosen depth.
 *
 * Descartes' Circle Theorem gives both the curvature and the centre
 * of that fourth circle in closed form:
 *
 *      k₄    = k₁+k₂+k₃ ± 2√(k₁k₂ + k₂k₃ + k₃k₁)
 *      k₄·z₄ = k₁z₁+k₂z₂+k₃z₃ ± 2√(k₁k₂z₁z₂+k₂k₃z₂z₃+k₃k₁z₃z₁)
 *
 * The two ± signs are independent, so the formula has FOUR algebraic
 * (k₄, z₄) solutions per triple.  Two correspond to real tangent
 * circles (the new gap-filler and an ancestor already in the set);
 * the other two are spurious — they satisfy the algebra but don't
 * actually touch all three inputs.  Every candidate is gated by two
 * geometric invariants:
 *
 *   • fits inside the outer disc:  |c₄ − c_outer| + r₄ ≤ r_outer
 *   • disjoint or tangent to every existing inner disc:
 *                                  |c₄ − c_E| ≥ r₄ + r_E
 *
 * Real Apollonian children satisfy both by construction; spurious
 * algebraic solutions get rejected.
 *
 * Rendering: every inner circle is either a flat-filled disc or an
 * outline ring (`f` toggles).  The outer disc is always an outline so
 * it doesn't tint the play area.
 *
 * Animation: a single clock t drives a construction sweep.
 * sweep_visible_depth(t) ramps 0 → depth_max over SWEEP_PER_DEPTH_S ×
 * depth_max seconds, holds at the ceiling for SWEEP_HOLD_S, then
 * snaps back to zero.  Each circle's drawn radius is multiplied by
 * alpha = clamp(visible − depth + 1, 0, 1), so it blossoms from a
 * point to full size as the cutoff crosses its depth layer.  Hue
 * rotates independently through the active palette's 7-colour ramp.
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
 * REFERENCES — for the concepts behind this program.
 *
 *   Mathematics & Algorithm
 *   ── Mumford, Series & Wright, "Indra's Pearls: The Vision of Felix
 *      Klein", Cambridge University Press, 2002.  The definitive
 *      pictorial introduction to circle packings, Möbius groups and
 *      Apollonian limit sets.  The single most useful book for
 *      SEEING what this program computes.
 *   ── Lagarias, Mallows & Wilks, "Beyond the Descartes Circle
 *      Theorem", American Mathematical Monthly 109 (2002), 338–361.
 *      Modern derivation of BOTH the curvature and the complex-
 *      centre forms of Descartes used in gasket_try_descartes (§3).
 *      JSTOR: https://www.jstor.org/stable/2695498
 *   ── Graham, Lagarias, Mallows, Wilks & Yan, "Apollonian Circle
 *      Packings: Number Theory", Journal of Number Theory 100
 *      (2003), 1–45.  Why the integer pack (−1, 2, 2, 3) and
 *      (−1, 2, 3, 6) work, with the algebraic structure of all
 *      integer Apollonian gaskets.
 *   ── Coxeter, H. S. M., "Introduction to Geometry", 2nd ed.,
 *      Wiley, 1969.  Inversive geometry foundations and the
 *      classical statement of Descartes' Circle Theorem.
 *   ── Soddy, F., "The Kiss Precise", Nature 137 (1936), 1021.
 *      A one-page verse statement of Descartes' theorem; charming
 *      historical primary source.
 *
 *   Rendering
 *   ── Foley, van Dam, Feiner & Hughes, "Computer Graphics:
 *      Principles and Practice", 3rd ed., Addison-Wesley, 2013.
 *      Standard reference for the circle outline (midpoint /
 *      Bresenham) and scanline disc-fill primitives draw_ring and
 *      draw_disc are direct expressions of.
 *   ── Bourke, P., "Character representation of greyscale images",
 *      1997.  https://paulbourke.net/dataformats/asciiart/
 *      Origin of the density-ramp idea behind ASCII intensity
 *      glyphs ('@', '#', '*', '+', ':', '.').
 *   ── Padala, P., "NCURSES Programming HOWTO", 2005.
 *      https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/
 *      Practical reference for `init_pair`, `mvaddch`, frame pacing
 *      and the resize/cleanup conventions used in §5 and §7.
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

/* ---- Descartes gates ---------------------------------------------- */
#define MIN_VALID_CURVATURE   1.0f     /* k₄ < 1 ⇒ radius > outer disc        */
#define EPS_NONZERO_CURV      1e-6f    /* |k₄| ≈ 0 means degenerate (line)     */
#define GEOM_TANGENT_TOL      1e-3f    /* tangent vs overlap (gasket units)    */
#define DEDUP_REL_TOL         2e-3f    /* relative (k, kz) match window        */
#define DEDUP_SCALE_BIAS      2.0f     /* keeps DEDUP tolerance positive at k≈0*/

/* ---- terminal cell aspect (cells are 2× taller than wide) --------- */
#define ASPECT_CELL_HEIGHT    2.0f     /* multiply row-stride by this to get */
                                       /* the equivalent in cell-WIDTH units */
#define ASPECT_INV            0.5f     /* 1 / ASPECT_CELL_HEIGHT              */

/* ---- rendering thresholds & glyphs -------------------------------- */
#define SUBPIXEL_RADIUS       0.5f     /* r_col below ⇒ render as single dot */
#define BOUNDARY_BAND_CELLS   1.0f     /* width of bold-edge disc band       */
#define DISC_MATH_CLIP        0.0f     /* strict — no antialias fringe       */
#define ROUNDING_BIAS         0.5f     /* (int)(x + 0.5f) rounds to nearest  */
#define SLOPE_SQRT3           1.73f    /* √3 — slope threshold in ring_char  */

#define GLYPH_SUBPIXEL        '.'
#define GLYPH_DISC_INTERIOR   '#'
#define GLYPH_DISC_BOUNDARY   '@'

/* ---- ring sampling (parametric outline) --------------------------- */
#define RING_BASE_STEPS       8        /* minimum samples on any ring         */
#define RING_STEPS_PER_RAD    2.5f     /* samples per cell-arc length        */
#define RING_MAX_STEPS        1024     /* cap so very big radii don't bloat  */

/* ---- construction sweep timing ------------------------------------ */
#define SWEEP_PER_DEPTH_S     1.5f     /* seconds per layer of blossom       */
#define SWEEP_HOLD_S          3.0f     /* hold at full depth before re-ramp  */

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
 * What it is: a single circle.  Has a CURVATURE k = 1/r and a
 * complex CENTRE z = x + iy.  Curvature carries a useful sign:
 *   • k > 0  — a normal inner disc
 *   • k < 0  — the outer "containing" disc; negative curvature is
 *              the standard convention for circles that hold
 *              everything else inside
 *   • k = 0  — a straight line (not used here)
 *
 * Why it exists: this is the basic thing the whole program moves
 * around.  Descartes takes three Circles and produces a fourth; the
 * renderer takes one Circle and draws it; the animation takes one
 * Circle and a clock and decides how big to draw it.  Giving the
 * bundle (curvature, centre, depth) a name lets us pass it by
 * pointer instead of by four loose floats.
 *
 * One quirk worth knowing: we store kz = k·z PRE-MULTIPLIED rather
 * than z itself.  The complex Descartes formula
 *
 *      k₄·z₄ = k₁·z₁ + k₂·z₂ + k₃·z₃ ± 2·√(k₁·k₂·z₁·z₂ + …)
 *
 * works directly on k·z values, so storing them avoids a division
 * inside the inner loop.  We recover z = kz/k once at render time.
 *
 * Fields:
 *   k       — curvature.  r = 1/|k|; k < 0 means this is the outer.
 *   zx, zy  — real and imaginary parts of k·z.  Centre is:
 *                 cx = zx / k,   cy = zy / k
 *   depth   — which recursion layer this circle came from:
 *               0    — outer seed
 *               1    — initial inner discs from the seed pack
 *               2+   — circles found by Descartes recursion
 *             Drives the blossom animation: a circle starts
 *             appearing when the sweep reaches its layer and is
 *             at full size one layer later.
 *
 * References:
 *   ── Mumford, Series & Wright, "Indra's Pearls", §3.
 *   ── Lagarias, Mallows & Wilks, "Beyond the Descartes Circle
 *      Theorem", AMM 109 (2002), eqs. (1.4) and (4.1). */
typedef struct {
    float k;
    float zx, zy;
    int   depth;
} Circle;

/* ---- DescartesTriple — three tangent circles + a depth tag ------- *
 *
 * What it is: three circles in the gasket (by index) that all touch
 * each other.  Together they bound one curvilinear gap — the kind
 * of gap Descartes' theorem fills with a unique fourth circle.
 *
 * Why it exists: the build is iterative.  We pop one of these,
 * compute the gap-filler, push the three new triples it spawns
 * with its parents — and repeat.  Giving the work-item a name
 * makes the recursion loop read like English:
 *
 *     while there is a pending triple:
 *         pop it, expand it, push the new triples it spawns
 *
 * Why indices instead of pointers: the circles[] array grows
 * during the build.  Pointers would risk going stale; indices
 * stay valid no matter what.
 *
 * Why a queue, not actual recursion: an explicit LIFO stack
 * bounds the working set (n_pending ≤ 4·MAX_CIRCLES — at most
 * three sub-triples pushed per pop) and would let us pause and
 * resume the build in chunks if we ever wanted to.
 *
 * Fields:
 *   i1, i2, i3 — indices of the three parent circles.  Order
 *                doesn't matter to the math.
 *   depth      — depth label to give whichever new circle this
 *                triple yields.  Pre-set by whoever pushed it.
 *
 * Invariant: the three circles are pairwise tangent.  Seed packs
 * pick valid starting triples by hand; each new circle is tangent
 * to all three of its parents, so the sub-triples we push from it
 * are tangent too.  The invariant is preserved by construction. */
typedef struct { int i1, i2, i3, depth; } DescartesTriple;

/* ---- Gasket — the recursive packing ------------------------------ *
 *
 * What it is: everything we've built so far.  All the circles we've
 * placed, the triples still waiting to be expanded, and the
 * recursion depth ceiling.
 *
 * Why it exists: the algorithm is just "state + operations".  Put
 * the state in one struct and every operation becomes a function
 * that takes `Gasket *`.  No globals, no surprise dependencies —
 * same gasket plus same operations always gives the same result.
 *
 * How the recursion works:
 *
 *     while there is a pending triple:
 *         pop it
 *         try all four Descartes sign combinations
 *         for each one that produces a new valid circle:
 *             add the circle
 *             push three new triples for its three sub-gaps
 *
 * Of the four sign combinations, usually one yields the new
 * gap-filler, one is the ancestor we already have (rejected as a
 * duplicate), and two are spurious roots (rejected as geometrically
 * invalid).  See §3 for the gate functions.
 *
 * Why fixed-size arrays: the build is offline (one synchronous pass
 * at startup or on parameter change), and at depth 7 the integer
 * Apollonian gasket has only a few thousand circles — well under
 * MAX_CIRCLES.  Static storage, no malloc.
 *
 * Why pending[] is 4× MAX_CIRCLES: each pop pushes up to three
 * triples.  Most reject immediately; 4× is generous headroom.
 *
 * Fields:
 *   circles[]  — every circle found so far.  Index 0 is always the
 *                outer (k < 0); the rest are inner discs in
 *                insertion order, which is also the render order.
 *   n          — how many entries in circles[] are valid.
 *   pending[]  — LIFO stack of pending triples.  LIFO means we go
 *                depth-first, which keeps the queue small.
 *   n_pending  — how many entries in pending[] are valid.
 *   depth_max  — recursion ceiling (1..DEPTH_MAX).  Adjusted by
 *                '+' / '-' at runtime.  Smaller = faster build
 *                and faster render.
 *
 * References:
 *   ── Graham, Lagarias, Mallows, Wilks & Yan, "Apollonian Circle
 *      Packings: Number Theory", JNT 100 (2003), §2. */
typedef struct {
    Circle           circles[MAX_CIRCLES];
    int              n;
    DescartesTriple  pending[MAX_CIRCLES * 4];
    int              n_pending;
    int              depth_max;
} Gasket;

/* ---- Palette — a depth-to-colour theme --------------------------- *
 *
 * What it is: a named 7-entry colour table.  Index by a circle's
 * depth (1..7); get back an xterm 256-colour code.
 *
 * Why it exists: keeps theming separate from drawing.  The renderer
 * never sees raw colour codes — it asks for the ncurses colour pair
 * matching a depth and draws with that.  Adding a new theme is one
 * entry in g_palettes[]; the rest of the program doesn't notice.
 * The `t` key just bumps an index and reloads pairs.
 *
 * Brightness rules (from project CLAUDE.md "Theme Palette
 * Brightness"):
 *   • Every entry must sit in the bright half of the 256-colour
 *     cube — must remain legible on a default-black background.
 *   • Ranges 16-23 (cube) and 232-239 (gray) vanish under A_DIM —
 *     never use them.
 *   • Ranges 24-29 / 240-243 are dim enough that we only allow
 *     them for the lowest tier (depth 1, biggest visual mass).
 *   • Ranges 30+ / 244+ are safe in every slot.
 *
 * Why exactly 7 colours: depth_max is 7 here, and anything deeper
 * is sub-pixel — it gets clamped to the deepest slot by
 * hue_shift().
 *
 * Fields:
 *   name      — short string shown in the HUD.
 *   colors[]  — 256-colour codes for depths 1..7, outermost to
 *               innermost.  Gradient palettes walk one tone from
 *               one end to the other; "rainbow" cycles through
 *               distinct hues.
 *
 * References:
 *   ── XTerm 256-colour palette:
 *      https://en.wikipedia.org/wiki/ANSI_escape_code#8-bit */
typedef struct {
    const char *name;
    int         colors[DEPTH_PAIRS];
} Palette;

/* ---- SeedPack — a starting configuration ------------------------- *
 *
 * What it is: a named function that lays down the first few
 * mutually-tangent circles and pushes the initial recursion tasks.
 *
 * Why it exists: an Apollonian gasket is determined by its first
 * few mutually-tangent circles.  Different starting picks give
 * visually different fractals.  By making each pack a function
 * pointer behind a name, we can offer several variants and cycle
 * between them at runtime ('n' key) with no branching inside the
 * recursion — the Gasket primitives don't know which seed they're
 * running.
 *
 * Why a function pointer instead of static data: each seed needs
 * its own arithmetic — Descartes math, geometry-specific kz
 * coordinates that often involve √3 and the like.  Inline code
 * expresses that clearly; static initialisers can't.
 *
 * Fields:
 *   name   — short string shown in the HUD.
 *   seed   — populates g->circles[] and g->pending[] from scratch.
 *            Caller (gasket_build) zeroes the counters first.
 *
 * Seeds shipped here:
 *   classic  (−1, 2, 2, 3, 3) — the canonical integer gasket;
 *                                left-right + top-bottom symmetric.
 *   trefoil  (−1, κ, κ, κ)   — three equal inner circles with
 *                                3-fold rotational symmetry;
 *                                κ = 1/(2√3 − 3) ≈ 2.155.
 *   coxeter  (−1, 2, 3, 6)   — asymmetric integer gasket;
 *                                recursion fills the missing
 *                                mirror circles.
 *
 * References:
 *   ── Graham et al., "Apollonian Circle Packings: Number Theory",
 *      JNT 100 (2003), Table 1 — admissible integer quadruples.
 *   ── Indra's Pearls, §7 — geometric intuition behind 3-fold
 *      symmetric packings. */
typedef struct {
    const char *name;
    void (*seed)(Gasket *);
} SeedPack;

/* ---- Viewport — gasket coords → terminal cells ------------------ *
 *
 * What it is: the recipe for turning a point in the (mathematical)
 * gasket into a (visual) terminal cell.
 *
 * Why it exists: keeps the projection in one place.  The renderer
 * doesn't need to know about terminal dimensions or cell aspect —
 * it just asks the Viewport to project for it.  When the terminal
 * resizes, viewport_fit() updates one struct and everything else
 * does the right thing on the next frame.
 *
 * Two things have to happen on the way to the screen:
 *
 *   1) ASPECT CORRECTION.  Terminal cells are roughly 2× taller
 *      than wide.  Distances in rows and columns aren't directly
 *      comparable.  We work in "cell-WIDTH units" everywhere and
 *      multiply vertical distances by 0.5 when converting to rows.
 *
 *   2) ASPECT FIT.  The gasket is a unit disc; we want it to fit
 *      inside the terminal no matter the shape.  Pick the smaller
 *      of the horizontal and vertical budgets and use that for
 *      both axes — the disc stays inside the play area.
 *
 * One value, `scale`, captures both: visual cells (width-units)
 * per gasket unit.  Bigger terminal → bigger scale → bigger draw.
 *
 * Aspect-fit budget (in cell-WIDTH units):
 *     horizontal  =  (cols − 2) / 2
 *     vertical    =  (rows − HUD_ROWS − 2)        ← already in
 *                                                   width-units
 *                                                   because cells
 *                                                   are 2× taller
 *     scale       =  min(horizontal, vertical)
 *
 * Projection (gasket → cell):
 *     col  =  cx_center  +  round( x · scale         )
 *     row  =  cy_center  −  round( y · scale · 0.5   )
 *
 * The −0.5× on rows is the aspect correction; the − sign flips
 * gasket-y (up positive) to terminal-y (down positive).
 *
 * Fields:
 *   rows, cols    — current terminal dimensions in cells.
 *   cx_center     — column where gasket origin (0, 0) lands.
 *                   Equals cols/2.
 *   cy_center     — row where gasket origin lands.  Vertically
 *                   centred between the HUD row and the hint row.
 *   scale         — visual cells (width-units) per gasket unit.
 *                   The unit disc is 2·scale cells wide and
 *                   scale cells tall.
 *
 * References:
 *   ── Foley, van Dam, Feiner & Hughes, "Computer Graphics:
 *      Principles and Practice", 3rd ed., §6 — window-to-viewport
 *      mappings with aspect preservation. */
typedef struct {
    int   rows, cols;
    int   cx_center, cy_center;
    float scale;
} Viewport;

/* ---- Scene — the whole program's state -------------------------- *
 *
 * What it is: one struct holding everything the running program
 * needs — the gasket, the projection, the animation clock, and the
 * user's render/theme/seed choices.
 *
 * Why it exists: makes the main loop read as a short sequence of
 * verbs on a single noun:
 *
 *     scene_handle_key(s, ch);
 *     scene_tick(s, dt);
 *     frame_render(s);
 *
 * Every helper takes only the piece it actually needs — `Gasket *`
 * for recursion, `Viewport *` for projection, `Scene *` only for
 * orchestration — so each function signature spells out what that
 * function depends on.  No mystery globals to chase.
 *
 * Fields, grouped by responsibility:
 *
 *   What we're packing:
 *     gasket     — the circles plus the pending recursion.
 *     seed       — index into g_seeds[].  Cycled by 'n'.
 *
 *   How we view it:
 *     view       — the projection.  Re-fit on every resize.
 *     palette    — index into g_palettes[].  Cycled by 't';
 *                  reloads the CP_D1..CP_D7 pairs each time.
 *     filled     — true:  inner discs are flat-filled.
 *                  false: inner discs are outline rings.
 *                  The outer is always an outline either way.
 *                  Toggled by 'f'.
 *
 *   Animation:
 *     t          — seconds since the last reset.  Drives both
 *                  the construction sweep and the hue rotation.
 *                  Advances when !paused.
 *     paused     — animation freeze.  HUD shows "PAUSED".
 *                  Toggled by 'p'.
 *     hue_speed  — palette-rotation rate in steps per second.
 *                  Signed: negative reverses direction.
 *                  Adjusted by ',' / '.' in HUE_SPEED_STEP
 *                  increments, clamped to ±HUE_SPEED_MAX by
 *                  scene_adjust_hue_speed. */
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

/* ---- colour-pair slots -------------------------------------------- */
enum { CP_D1 = 1, CP_D2, CP_D3, CP_D4, CP_D5, CP_D6, CP_D7, CP_HUD, CP_HINT };

/* ---- palettes (constant config) ----------------------------------- */
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

/* Principal complex square root of (ax + i·ay).
 *
 * Strategy: convert to polar, halve the angle, take real √r.  This
 * always returns the root with positive real part (or zero), which
 * is what Descartes' formula expects when combined with our (sk, sz)
 * sign convention. */
static void complex_sqrt(float ax, float ay, float *rx, float *ry)
{
    float magnitude_sqrt = sqrtf(sqrtf(ax*ax + ay*ay));
    float half_angle     = atan2f(ay, ax) * 0.5f;
    *rx = magnitude_sqrt * cosf(half_angle);
    *ry = magnitude_sqrt * sinf(half_angle);
}

/* Complex multiplication (ax + i·ay)·(bx + i·by). */
static void complex_mul(float ax, float ay, float bx, float by,
                        float *rx, float *ry)
{
    *rx = ax * bx - ay * by;
    *ry = ax * by + ay * bx;
}

/* ===================================================================== */
/* §3  gasket — Descartes recursion and seed packs                        */
/* ===================================================================== */

/* ---- duplicate detection ------------------------------------------- */

/* Does the gasket already contain this (k, kz)?
 *
 * Tolerance is relative because float error in Descartes compounds
 * roughly linearly with k — a fixed absolute eps lets deep duplicates
 * slip through; a scaled eps catches them without falsely merging
 * genuinely-distinct nearby circles. */
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

/* ---- geometric admissibility -------------------------------------- */

/* Does a candidate (k4, c4, r4=1/k4) satisfy the Apollonian invariants
 * relative to every circle already in the gasket?
 *
 *   vs OUTER (k<0):  candidate must fit inside the outer disc.
 *                    reject if  d + r4  >  r_outer + TOL
 *
 *   vs INNER (k>0):  candidate must be externally disjoint or tangent.
 *                    reject if  d       <  r4 + r_E - TOL
 *                    (one inequality catches overlap, internal tangent
 *                     and full containment all at once) */
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
            if (d + r4 > re + GEOM_TANGENT_TOL) return false;   /* breaches outer */
        } else {
            if (d < r4 + re - GEOM_TANGENT_TOL) return false;   /* overlaps inner */
        }
    }
    return true;
}

/* ---- Descartes' theorem primitives -------------------------------- */

/* Curvature root k₄ = k₁+k₂+k₃ ± 2·√(k₁k₂+k₂k₃+k₃k₁).
 * Returns false if the radicand is negative (no real root). */
static bool descartes_curvature_root(float k1, float k2, float k3,
                                     int sk, float *k4_out)
{
    float radicand = k1*k2 + k2*k3 + k3*k1;
    if (radicand < 0.f) return false;
    *k4_out = k1 + k2 + k3 + sk * 2.f * sqrtf(radicand);
    return true;
}

/* The complex radicand for the centre formula:
 *   (k₁z₁)(k₂z₂) + (k₂z₂)(k₃z₃) + (k₃z₃)(k₁z₁)
 * computed directly from the stored kz values via complex products. */
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

/* Complex centre root:  k₄z₄ = Σ(k_i z_i) ± 2·√(radicand). */
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

/* When a new circle joins the gasket, three new gaps appear — one
 * between the new circle and each pair of its parents.  Push a
 * recursion task for each, unless we're at the depth ceiling or the
 * queue is full. */
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

/* ---- the Descartes attempt ---------------------------------------- *
 *
 * Try one (sk, sz) sign combination for the given triple.  Returns
 * true iff a geometrically-valid new circle was appended to the
 * gasket.  Reads top-to-bottom as a sequence of named gates:
 *
 *     compute curvature root → bail if no root, too big, too deep
 *     compute centre root
 *     bail if degenerate
 *     bail if duplicate
 *     bail if geometrically inadmissible
 *     append */
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

/* Exhaust all four (sk, sz) sign combinations for one triple.  For
 * each new child found, push its three sub-gaps onto the pending
 * stack.  Returns true if any of the four attempts succeeded. */
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

/* Pop one pending triple, exhaust its Descartes children, push their
 * sub-gaps.  Returns true if progress was made — call repeatedly
 * until it returns false (which means the gasket is complete). */
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

/* CLASSIC — the integer Apollonian gasket k = (−1, 2, 2, 3, 3).  Top
 * and bottom (both k=3) are NOT tangent to each other, so triples
 * mentioning both are omitted.  The triple (outer, right, left) is
 * also omitted because its two Descartes children are exactly the
 * hard-coded top and bottom. */
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

/* TREFOIL — three equal inner circles k = 1/(2√3 − 3) ≈ 2.155
 * arranged with 3-fold rotational symmetry.  Each inner centre sits
 * at distance (1 − r) from the origin; the three centres are at
 * angles 90°, 210°, 330° (i.e. 120° apart).  In stored (kz) form
 * with h = k − 1:
 *     top         (0,         h    )
 *     lower-left  (−h·√3/2,  −h/2  )
 *     lower-right ( h·√3/2,  −h/2  ) */
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

/* COXETER — the asymmetric integer gasket (−1, 2, 3, 6).  Descartes
 * recursion fills the missing mirror circles. */
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

/* Sawtooth visibility cutoff for the construction sweep.  Ramps
 * 0 → depth_max over T_sweep seconds, holds for SWEEP_HOLD_S, then
 * snaps back to 0 and re-ramps. */
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

/* Blossom factor for a circle at the given depth.  Grows linearly
 * from 0 to 1 across one depth layer of the sweep, then stays at 1.
 * A circle "starts to appear" when visible_depth crosses depth−1
 * and "reaches full size" when visible_depth reaches depth. */
static float blossom_alpha(float visible_depth, int circle_depth)
{
    float a = visible_depth - (float)circle_depth + 1.0f;
    if (a < 0.f) return 0.f;
    if (a > 1.f) return 1.f;
    return a;
}

/* Integer rotation through the 7-pair depth ramp at rate hue_speed.
 * Returns a colour-pair index in [CP_D1 .. CP_D7].  The (+N)%N
 * sandwich is the standard positive-modulo trick so negative
 * hue_speed (rotating the other direction) still yields a valid
 * non-negative index. */
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

/* Install the seven depth-colour pairs from the active palette.
 * Called at startup and whenever the palette is cycled. */
static void palette_apply(int idx)
{
    if (COLORS >= 256) {
        const Palette *p = &g_palettes[idx];
        for (int i = 0; i < DEPTH_PAIRS; i++)
            init_pair(CP_D1 + i, p->colors[i], -1);
    } else {
        /* 8-colour fallback — palettes degenerate to rainbow defaults */
        init_pair(CP_D1, COLOR_YELLOW,  -1);
        init_pair(CP_D2, COLOR_GREEN,   -1);
        init_pair(CP_D3, COLOR_CYAN,    -1);
        init_pair(CP_D4, COLOR_BLUE,    -1);
        init_pair(CP_D5, COLOR_MAGENTA, -1);
        init_pair(CP_D6, COLOR_RED,     -1);
        init_pair(CP_D7, COLOR_RED,     -1);
    }
}

/* HUD colour pairs are static — they don't change with theme cycling. */
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

/* Aspect-fit the unit gasket disc into the current terminal.  Scale
 * is the largest value that keeps the disc fully inside the play
 * area both horizontally AND vertically. */
static void viewport_fit(Viewport *v, int rows, int cols)
{
    v->rows = rows;
    v->cols = cols;

    /* Both budgets in cell-WIDTH units */
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
