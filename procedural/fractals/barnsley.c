/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * barnsley.c — draws Barnsley-style fractals (ferns, trees, dragons...)
 * by playing the "chaos game": start at a point, then over and over pick
 * one of a few simple move-and-shrink rules at random and jump there.
 * Plot where the point lands millions of times and a fractal appears.
 * 30 built-in fractals, switchable live.
 *
 * References (the math the code can't explain on its own):
 *   ── Barnsley, "Fractals Everywhere", 2nd ed., 1993 — the chaos game
 *      and the fern coefficients used here.
 *   ── Hutchinson, "Fractals and Self-Similarity", 1981 — the proof that
 *      these rule-sets have a unique fractal they always converge to.
 *   ── Draves & Reckase, "The Fractal Flame Algorithm", 2003,
 *      https://flam3.com/flame_draves.pdf — the trick of counting hits
 *      per cell and using a log scale so faint and bright areas both show.
 *   ── Bourke, "Character representation of greyscale images", 1997 — the
 *      '.' ':' '+' '@' brightness ramp.
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>          /* memset                                  */
#include <time.h>

/* ===================================================================== */
/* §1  types & data                                                       */
/* ===================================================================== */

/* ---- grid + HUD sizing -------------------------------------------- */
#define GRID_ROWS_MAX        80
#define GRID_COLS_MAX       300
#define HUD_TOP_ROWS          1     /* top row shows the stats            */
#define HUD_BOTTOM_ROWS       1     /* bottom row shows the key hints     */

/* ---- iteration limits -------------------------------------------- */
#define ITERS_MIN          1000
#define ITERS_MAX         30000
#define ITERS_DEFAULT      8000
#define ITERS_STEP         1000

/* ---- highest hit count we'll store in one cell (stops overflow) -- */
#define HITS_CAP          60000u

/* ---- timing ------------------------------------------------------ */
#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define FRAME_NS         (NS_PER_SEC / 30)        /* 30 fps target      */
#define FPS_WINDOW_MS         500                 /* rolling FPS window */

/* ---- how many of each thing ------------------------------------- */
#define MAX_IFS_MAPS          8     /* most rules any fractal here uses (Carpet) */
#define N_DENSITY_TIERS       4     /* brightness levels: faint to bright */
#define N_PRESETS            30
#define N_PALETTES            5

/* ---- random-number generator dials (a known-good classic pair) -- */
#define LCG_MULTIPLIER  1664525u
#define LCG_INCREMENT   1013904223u

/* ---- brightness cutoffs: how full a cell must be to reach each
 *      level.  The hit count is squeezed into a 0..1 brightness first. */
#define DENSITY_BG_MAX     0.15    /* below this: leave the cell blank   */
#define DENSITY_L1_MAX     0.35
#define DENSITY_L2_MAX     0.55
#define DENSITY_L3_MAX     0.75

/* ---- the glyphs for each brightness level, faintest to brightest - */
#define GLYPH_L1    '.'
#define GLYPH_L2    ':'
#define GLYPH_L3    '+'
#define GLYPH_L4    '@'

/* ---- fitting the picture to the screen --------------------------- */
#define ASPECT_CELL_HEIGHT      2.0f      /* terminal cells are ~2x taller than wide */
#define ASPECT_INV              0.5f      /* 1 / ASPECT_CELL_HEIGHT          */
#define VIEW_MIDPOINT_FRACTION  0.5f      /* halfway between min and max     */
#define EDGE_MARGIN_CELLS       1         /* leave a 1-cell gap at the edges */

/* ---- turning raw RNG output into a number between 0 and 1 -------- */
#define LCG_HIGH_BITS_SHIFT     8u        /* drop the low 8 bits; they're the wobbly ones */
#define LCG_UNIT_DENOM          (1u << 24)/* so the result lands in [0, 1)   */
#define LCG_SEED_TIME_MIX       123456789LL  /* stir the seconds into the seed */

/* ---- defaults ---------------------------------------------------- */
#define DEFAULT_PRESET_IDX      0
#define DEFAULT_PALETTE_IDX     0

/* ---- misc -------------------------------------------------------- */
#define KEY_ESCAPE          27
#define EPS_LOG_MAX     1e-12         /* tiny floor so we never divide by zero */
#define EPS_RANGE       1e-6f         /* tiny floor so a zero-size view can't blow up */
#define ORBIT_SEED_X    0.1f          /* where the wandering point starts */
#define ORBIT_SEED_Y    0.0f


/* ---- AffineMap — one move-and-shrink rule ------------------------ *
 *
 * One rule of the chaos game.  It takes the current point (x, y) and
 * spits out a new point:
 *
 *     new_x = a*x + b*y + e
 *     new_y = c*x + d*y + f
 *
 * In plain terms: a, b, c, d rotate/shrink/squash the point, and e, f
 * slide it sideways and up.  Each fractal is just a short list of these
 * rules.  Picking and applying one is the whole inner loop, so it's
 * deliberately tiny.
 *
 * Fields:
 *   a, b, c, d  — the four numbers that rotate, shrink, and squash the
 *                 point.  (For a Barnsley fern's main leaflet rule:
 *                 a=0.85, b=0.04, c=-0.04, d=0.85 — a slight turn plus
 *                 an 85% shrink.)
 *   e, f        — how far to slide the point left/right and up/down
 *                 after the shrink.
 *   cum         — a "running total" of how likely the rules up to and
 *                 including this one are.  We list the rules so cum only
 *                 climbs, and the last one is exactly 1.0.  To pick a
 *                 rule we roll a number in [0,1) and grab the first rule
 *                 whose cum is at least the roll — one comparison, done.
 *                 (Fern example with odds 1%/85%/7%/7%: the cums come out
 *                 0.01, 0.86, 0.93, 1.00.)
 *
 * Adding your own fractal?  A rule that covers more area should be
 * picked more often, or that part comes out too sparse to see.
 *
 * Reference: Barnsley, "Fractals Everywhere", ch. 3. */
typedef struct {
    float a, b, c, d, e, f;
    float cum;
} AffineMap;

/* ---- IFSPreset — one ready-to-draw fractal ----------------------- *
 *
 * Everything that makes a fractal what it is: a name, its list of rules,
 * and the rectangle of fractal-space it lives in.  One engine draws all
 * 30 fractals; switching fractal just means pointing at a different
 * preset, clearing the grid, and re-fitting it to the screen.  No new
 * code per fractal.
 *
 * Fields:
 *   name          — short name for the on-screen stats (kept short to
 *                   fit the line).
 *   n_maps        — how many rules in maps[] are real (1..MAX_IFS_MAPS).
 *   maps[]        — the move-and-shrink rules, ordered so their `cum`
 *                   running total climbs to 1.0 at the last one.  The
 *                   array is a fixed 8 slots; unused slots stay zero and
 *                   n_maps says where to stop, so no allocation is needed.
 *   x_min, x_max,
 *   y_min, y_max  — the box in fractal coordinates the shape fits in.
 *                   The view code (§4) uses this to size and center the
 *                   drawing.  A tighter box draws bigger; too tight and
 *                   the tips get clipped off.
 *
 * Reference: Barnsley, "Fractals Everywhere", §3.6 (designing a rule-set
 * to match a target shape). */
typedef struct {
    const char *name;
    int         n_maps;
    AffineMap   maps[MAX_IFS_MAPS];
    float       x_min, x_max;
    float       y_min, y_max;
} IFSPreset;

/* ---- Palette — one colour theme --------------------------------- *
 *
 * Four colours, one for each brightness level (faint dots up to the
 * brightest cores).  Keeping colour choices here in a table means the
 * drawing code never deals with raw colours — it just asks for "the
 * colour of level 2".  Adding a theme is one new row; the `t` key cycles
 * through them.  The colours go from dark to light as cells get busier,
 * which is what makes a "Fire" theme glow red-to-yellow toward the dense
 * spots.
 *
 * Fields:
 *   name      — short name for the on-screen stats.
 *   fg256[]   — the four colours on a modern 256-colour terminal.
 *   fg8[]     — fallback colours for old 8-colour terminals; used only
 *               when the terminal can't do 256. */
typedef struct {
    const char *name;
    int  fg256[N_DENSITY_TIERS];
    int  fg8[N_DENSITY_TIERS];
} Palette;

/* ---- RandLCG — a tiny, fast random-number generator -------------- *
 *
 * Holds one 32-bit number and rolls it forward with a single multiply
 * and add to get the next "random" value.  We use our own instead of
 * the library's rand() purely for speed: the chaos game asks for a
 * random number hundreds of thousands of times a second, and this is a
 * couple of instructions per call.  The randomness is plenty good for
 * just picking which rule to apply.
 *
 * Fields:
 *   state — the current number.  Must never be zero: a zero would stay
 *           zero forever, so the seeding code below makes sure it isn't.
 *
 * Reference: Knuth, TAOCP Vol. 2, §3.2.1 (the multiplier/increment we
 * use are the well-studied "minimal standard" pair). */
typedef struct {
    uint32_t state;
} RandLCG;

/* ---- Orbit — the wandering point ---------------------------------- *
 *
 * The single point the chaos game is currently bouncing around, plus the
 * random generator that decides where it jumps next.  Just one point,
 * not a trail.  Each step replaces it with wherever the chosen rule
 * sends it.
 *
 * It doesn't matter where we start: the rules all pull inward, so within
 * a dozen or so jumps the point is sitting on the fractal, and from then
 * on every spot it lands is a real part of the shape.  (Those first few
 * stray points are drowned out by the millions that follow, so we don't
 * bother throwing them away.)
 *
 * Fields:
 *   cx, cy — where the point is now, in fractal coordinates (not screen
 *            pixels).  Changes every step.
 *   rng    — the random source, kept right next to the position so the
 *            tight loop only juggles one struct.
 *
 * Reference: Barnsley, "Fractals Everywhere", §3.4. */
typedef struct {
    float    cx, cy;
    RandLCG  rng;
} Orbit;

/* ---- DensityGrid — a tally of where the point landed ------------- *
 *
 * One counter per screen cell, recording how many times the wandering
 * point landed there.  The picture we want isn't single dots — it's how
 * crowded each spot is.  Busy spots get hit far, far more often than
 * faint edges, so when we draw we squeeze the counts through a log scale
 * (in grid_render) so the faint parts still show up instead of being
 * washed out by the bright cores.
 *
 * Counters are 16-bit and we stop them at HITS_CAP so they can't
 * overflow; it doesn't change the look since we always scale relative to
 * the busiest cell each frame.  The grid is a fixed size set at compile
 * time, so it just sits in static memory — no allocating or resizing.
 *
 * Fields:
 *   cells[][] — the hit counts, indexed [row][col] in screen position
 *               (row 0 is the top of the screen).
 *
 * References:
 *   ── Draves & Reckase, "The Fractal Flame Algorithm", 2003 — counting
 *      hits and using a log scale to draw them.
 *   ── Bourke, "Character representation of greyscale images", 1997 —
 *      the ASCII brightness glyphs. */
typedef struct {
    uint16_t cells[GRID_ROWS_MAX][GRID_COLS_MAX];
} DensityGrid;

/* ---- Viewport — how to place the fractal on screen --------------- *
 *
 * A small bundle of numbers that say where each fractal point should land
 * on the terminal.  Keeping it in one place means the busy inner loop can
 * just say "where does this point go?" without caring about screen size
 * or the rows reserved for the stats.  It gets recomputed whenever the
 * window resizes or the fractal changes, then stays put for many frames.
 *
 * Two things have to be handled when fitting the shape on screen:
 *
 *   - Terminal cells are about twice as tall as they are wide, so a row
 *     of the shape covers more height than a column covers width.  We do
 *     all our sizing in width-units and halve the vertical part, or every
 *     circle would look squashed.
 *
 *   - Each fractal has its own natural size.  We pick one zoom level that
 *     makes it fit the screen both ways (whichever direction is tighter
 *     wins), keeping the shape's proportions and leaving a margin on the
 *     looser side.
 *
 * Fields:
 *   rows, cols   — current terminal size in cells.
 *   play_rows    — rows actually free for the picture (total minus the
 *                  two stats rows).  Cached so it isn't recomputed.
 *   center_col   — the column the picture is centered on.
 *   center_row   — the row the picture is centered on (between the stats
 *                  rows).
 *   scale        — the single zoom knob: how many screen cells one unit
 *                  of fractal space takes up.  Bigger window, bigger scale,
 *                  bigger drawing.
 *   view_mid_x,
 *   view_mid_y   — the point of the fractal that sits at screen center
 *                  (the middle of its bounding box).
 *
 * Reference: Foley et al., "Computer Graphics", 3rd ed., §6 (fitting a
 * drawing into a window while keeping its proportions). */
typedef struct {
    int    rows, cols;
    int    play_rows;
    int    center_col, center_row;
    float  scale;
    float  view_mid_x, view_mid_y;
} Viewport;

/* ---- FpsCounter — a steady frames-per-second readout ------------- *
 *
 * Counts frames and elapsed time over a short window (about half a
 * second), then divides to get one frames-per-second number for the
 * stats line.  We average instead of reporting each frame because
 * individual frame times jump around wildly and would make the number
 * flicker uselessly.
 *
 * Fields:
 *   accum_ns — total time spent so far in the current window.
 *   frames   — how many frames so far in the current window.
 *   display  — the last figure we worked out; this is what's shown.
 *              It refreshes only when a window finishes, so it stays
 *              readable between updates. */
typedef struct {
    long long accum_ns;
    int       frames;
    double    display;
} FpsCounter;

/* ---- Scene — all the program's state in one place ---------------- *
 *
 * One struct holding everything the running program needs, instead of a
 * pile of scattered globals.  Each helper takes just the part of it that
 * it touches, so you can tell from a function's arguments what it reads
 * and changes.
 *
 * Fields, grouped by job:
 *
 *   the fractal:
 *     grid    — the hit tally.  Wiped clean on reset.
 *     orbit   — the wandering point and its randomness.  Restarted on
 *               reset; its random seed is set once at startup.
 *     preset  — which of the 30 fractals is showing.  Changed by n/N.
 *     iters   — how many jumps to make per frame.  Changed by +/-,
 *               kept within sensible bounds.  More = denser and faster
 *               to fill in, but more work.
 *
 *   how it's shown:
 *     view    — the screen-placement recipe.  Recomputed on resize and
 *               when the fractal changes.
 *     palette — which colour theme is active.  Changed by t/T; only
 *               affects colours, not the picture itself.
 *
 *   control:
 *     paused  — when true, the point stops moving (stats keep updating).
 *               Toggled by 'p' or space.
 *
 *   timing:
 *     fps     — the frames-per-second readout. */
typedef struct {
    /* the fractal */
    DensityGrid  grid;
    Orbit        orbit;
    int          preset;
    int          iters;

    /* how it's shown */
    Viewport     view;
    int          palette;

    /* control */
    bool         paused;

    /* timing */
    FpsCounter   fps;
} Scene;

/* ---- colour-pair slots (ncurses numbers each colour combo) ------- */
enum {
    CP_HUD  = 1,    /* the stats line (bright yellow)                    */
    CP_HINT = 2,    /* the key-hint line (bright cyan)                   */
    CP_L1   = 3,    /* faintest cells                                    */
    CP_L2   = 4,
    CP_L3   = 5,
    CP_L4   = 6,    /* brightest cells                                   */
};

/* ---- the colour themes ------------------------------------------- */
static const Palette g_palettes[N_PALETTES] = {
    { "Fern",   { 22, 34, 46, 154 },
                { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN } },
    { "Fire",   { 124, 196, 208, 226 },
                { COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW } },
    { "Ice",    { 17, 27, 51, 123 },
                { COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN } },
    { "Plasma", { 54, 129, 201, 231 },
                { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN, COLOR_WHITE } },
    { "Mono",   { 240, 245, 250, 255 },
                { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE } },
};

/* ---- the 30 fractals, grouped by family -------------------------- *
 * Each row is the rule-set + bounding box for one fractal.  The numbers
 * inside the rules are the move-and-shrink coefficients (see AffineMap). */
static const IFSPreset g_presets[N_PRESETS] = {

    /* ────── ferns & leaves ────── */

    /*  1. Barnsley Fern — the famous one.  Rule 1 (1% of the time) draws
     *  the stem, rule 2 (85%) the big fronds, rules 3-4 the side leaves. */
    { "Barnsley Fern", 4,
      {
        { 0.00f,  0.00f,  0.00f, 0.16f, 0.00f, 0.00f, 0.01f },
        { 0.85f,  0.04f, -0.04f, 0.85f, 0.00f, 1.60f, 0.86f },
        { 0.20f, -0.26f,  0.23f, 0.22f, 0.00f, 1.60f, 0.93f },
        {-0.15f,  0.28f,  0.26f, 0.24f, 0.00f, 0.44f, 1.00f },
      },
      -2.6f, 2.6f,  -0.2f, 10.2f,
    },

    /*  2. Mirror Fern — Barnsley fern reflected across the y-axis. */
    { "Mirror Fern", 4,
      {
        { 0.00f,  0.00f,  0.00f, 0.16f, 0.00f, 0.00f, 0.01f },
        { 0.85f, -0.04f,  0.04f, 0.85f, 0.00f, 1.60f, 0.86f },
        { 0.20f,  0.26f, -0.23f, 0.22f, 0.00f, 1.60f, 0.93f },
        {-0.15f, -0.28f, -0.26f, 0.24f, 0.00f, 0.44f, 1.00f },
      },
      -2.6f, 2.6f,  -0.2f, 10.2f,
    },

    /*  3. Cyclosorus Fern — a more spiral-stemmed fern variant. */
    { "Cyclosorus", 4,
      {
        {  0.000f,  0.000f,  0.000f, 0.250f,  0.000f,-0.400f, 0.02f },
        {  0.950f,  0.005f, -0.005f, 0.930f, -0.002f, 0.500f, 0.86f },
        {  0.035f, -0.200f,  0.160f, 0.040f, -0.090f, 0.020f, 0.93f },
        { -0.040f,  0.200f,  0.160f, 0.040f,  0.083f, 0.120f, 1.00f },
      },
      -2.6f, 2.6f,  -0.5f, 6.5f,
    },

    /*  4. Slim Fern — tall narrow fern with longer stem. */
    { "Slim Fern", 4,
      {
        { 0.00f,  0.00f,  0.00f, 0.30f, 0.00f, 0.00f, 0.02f },
        { 0.85f,  0.02f, -0.02f, 0.90f, 0.00f, 1.80f, 0.85f },
        { 0.10f, -0.15f,  0.13f, 0.15f, 0.00f, 1.60f, 0.93f },
        {-0.10f,  0.15f,  0.13f, 0.15f, 0.00f, 0.44f, 1.00f },
      },
      -1.5f, 1.5f,  -0.2f, 12.0f,
    },

    /*  5. Maple Leaf — Barnsley's published maple-leaf coefficients. */
    { "Maple Leaf", 4,
      {
        { 0.14f,  0.01f,  0.00f, 0.51f, -0.08f, -1.31f, 0.10f },
        { 0.43f,  0.52f, -0.45f, 0.50f,  1.49f, -0.75f, 0.45f },
        { 0.45f, -0.49f,  0.47f, 0.47f, -1.62f, -0.74f, 0.80f },
        { 0.49f,  0.00f,  0.00f, 0.51f,  0.02f,  1.62f, 1.00f },
      },
      -4.0f, 4.0f,  -2.0f, 4.0f,
    },

    /* ────── trees ────── */

    /*  6. Fractal Tree — symmetric binary tree, 45° branches. */
    { "Fractal Tree", 4,
      {
        { 0.00f,  0.00f,  0.00f, 0.50f, 0.00f, 0.00f, 0.05f },
        { 0.42f, -0.42f,  0.42f, 0.42f, 0.00f, 0.20f, 0.45f },
        { 0.42f,  0.42f, -0.42f, 0.42f, 0.00f, 0.20f, 0.85f },
        { 0.00f,  0.00f,  0.00f, 0.75f, 0.00f, 0.20f, 1.00f },
      },
      -1.1f, 1.1f,   0.0f, 2.0f,
    },

    /*  7. Pythagoras Tree — the classic one: two branches splitting off
     *     at 45 degrees, each a bit smaller than the trunk. */
    { "Pythagoras", 3,
      {
        { 0.05f,  0.00f,  0.00f, 0.50f, 0.00f, 0.00f, 0.10f },   /* trunk */
        { 0.50f, -0.50f,  0.50f, 0.50f, 0.00f, 1.00f, 0.55f },   /* left  */
        { 0.50f,  0.50f, -0.50f, 0.50f, 0.50f, 1.50f, 1.00f },   /* right */
      },
      -2.0f, 2.5f,   0.0f, 4.0f,
    },

    /*  8. Pine Tree — tall narrow conifer (small branch angle). */
    { "Pine Tree", 4,
      {
        { 0.00f,  0.00f,  0.00f, 0.60f, 0.00f, 0.00f, 0.10f },
        { 0.35f, -0.10f,  0.10f, 0.35f, 0.00f, 0.40f, 0.50f },
        { 0.35f,  0.10f, -0.10f, 0.35f, 0.00f, 0.40f, 0.90f },
        { 0.00f,  0.00f,  0.00f, 0.80f, 0.00f, 0.20f, 1.00f },
      },
      -0.6f, 0.6f,   0.0f, 3.5f,
    },

    /*  9. Wide Tree — broad 60° branch angle. */
    { "Wide Tree", 4,
      {
        { 0.00f,  0.00f,  0.00f, 0.50f, 0.00f, 0.00f, 0.05f },
        { 0.35f, -0.70f,  0.70f, 0.35f, 0.00f, 0.20f, 0.45f },
        { 0.35f,  0.70f, -0.70f, 0.35f, 0.00f, 0.20f, 0.85f },
        { 0.00f,  0.00f,  0.00f, 0.75f, 0.00f, 0.20f, 1.00f },
      },
      -2.0f, 2.0f,   0.0f, 2.5f,
    },

    /* 10. Narrow Tree — tight 15° branch angle. */
    { "Narrow Tree", 4,
      {
        { 0.00f,  0.00f,  0.00f, 0.50f, 0.00f, 0.00f, 0.05f },
        { 0.50f, -0.15f,  0.15f, 0.50f, 0.00f, 0.20f, 0.45f },
        { 0.50f,  0.15f, -0.15f, 0.50f, 0.00f, 0.20f, 0.85f },
        { 0.00f,  0.00f,  0.00f, 0.75f, 0.00f, 0.20f, 1.00f },
      },
      -0.8f, 0.8f,   0.0f, 2.5f,
    },

    /* 11. Asym Tree — unequal branch sizes / angles. */
    { "Asym Tree", 4,
      {
        { 0.00f,  0.00f,  0.00f, 0.50f, 0.00f, 0.00f, 0.05f },
        { 0.50f, -0.30f,  0.30f, 0.50f, 0.00f, 0.20f, 0.50f },
        { 0.30f,  0.40f, -0.40f, 0.30f, 0.00f, 0.20f, 0.80f },
        { 0.00f,  0.00f,  0.00f, 0.75f, 0.00f, 0.20f, 1.00f },
      },
      -1.5f, 1.5f,   0.0f, 2.5f,
    },

    /* 12. Bushy Tree — extra branch maps for a fuller crown. */
    { "Bushy Tree", 5,
      {
        { 0.00f,  0.00f,  0.00f, 0.50f, 0.00f, 0.00f, 0.10f },
        { 0.40f, -0.30f,  0.30f, 0.40f, 0.00f, 0.30f, 0.35f },
        { 0.40f,  0.30f, -0.30f, 0.40f, 0.00f, 0.30f, 0.60f },
        { 0.50f, -0.50f,  0.50f, 0.50f, 0.00f, 0.30f, 0.80f },
        { 0.50f,  0.50f, -0.50f, 0.50f, 0.00f, 0.30f, 1.00f },
      },
      -1.5f, 1.5f,   0.0f, 2.5f,
    },

    /* 13. Tilted Tree — trunk slightly leaning. */
    { "Tilted Tree", 4,
      {
        { 0.05f,  0.00f,  0.00f, 0.45f, 0.10f, 0.05f, 0.05f },
        { 0.40f, -0.50f,  0.50f, 0.40f, 0.20f, 0.20f, 0.45f },
        { 0.45f,  0.40f, -0.40f, 0.45f,-0.20f, 0.20f, 0.85f },
        { 0.05f,  0.00f,  0.00f, 0.70f, 0.10f, 0.20f, 1.00f },
      },
      -1.5f, 1.5f,   0.0f, 2.5f,
    },

    /* ────── Sierpinski family ────── */

    /* 14. Sierpinski Triangle — three half-size copies, one per corner. */
    { "Sierpinski", 3,
      {
        { 0.5f, 0.f, 0.f, 0.5f, 0.0f, 0.00f, 0.333f },
        { 0.5f, 0.f, 0.f, 0.5f, 1.0f, 0.00f, 0.667f },
        { 0.5f, 0.f, 0.f, 0.5f, 0.5f, 0.87f, 1.000f },
      },
      -0.1f, 1.1f,  -0.1f, 1.1f,
    },

    /* 15. Sierpinski Pentagon — five shrunken copies, one per pentagon corner. */
    { "Pentagon", 5,
      {
        { 0.382f, 0.f, 0.f, 0.382f,  0.000f,  0.618f, 0.20f },
        { 0.382f, 0.f, 0.f, 0.382f, -0.588f,  0.191f, 0.40f },
        { 0.382f, 0.f, 0.f, 0.382f, -0.363f, -0.500f, 0.60f },
        { 0.382f, 0.f, 0.f, 0.382f,  0.363f, -0.500f, 0.80f },
        { 0.382f, 0.f, 0.f, 0.382f,  0.588f,  0.191f, 1.00f },
      },
      -1.1f, 1.1f,  -1.1f, 1.1f,
    },

    /* 16. Sierpinski Hexagon — six third-size copies, one per hexagon corner. */
    { "Hexagon", 6,
      {
        { 0.333f, 0.f, 0.f, 0.333f,  0.000f,  0.667f, 1.f/6.f },
        { 0.333f, 0.f, 0.f, 0.333f, -0.577f,  0.333f, 2.f/6.f },
        { 0.333f, 0.f, 0.f, 0.333f, -0.577f, -0.333f, 3.f/6.f },
        { 0.333f, 0.f, 0.f, 0.333f,  0.000f, -0.667f, 4.f/6.f },
        { 0.333f, 0.f, 0.f, 0.333f,  0.577f, -0.333f, 5.f/6.f },
        { 0.333f, 0.f, 0.f, 0.333f,  0.577f,  0.333f, 1.000f  },
      },
      -1.1f, 1.1f,  -1.1f, 1.1f,
    },

    /* 17. Sierpinski Carpet — split a square into a 3x3 grid, keep all
     *     but the middle cell, repeat. */
    { "Carpet", 8,
      {
        { 0.333f, 0.f, 0.f, 0.333f, 0.000f, 0.000f, 0.125f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.333f, 0.000f, 0.250f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.667f, 0.000f, 0.375f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.000f, 0.333f, 0.500f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.667f, 0.333f, 0.625f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.000f, 0.667f, 0.750f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.333f, 0.667f, 0.875f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.667f, 0.667f, 1.000f },
      },
      -0.1f, 1.1f,  -0.1f, 1.1f,
    },

    /* 18. T-Square — four half-size copies, one per corner of a square. */
    { "T-Square", 4,
      {
        { 0.5f, 0.f, 0.f, 0.5f, 0.0f, 0.0f, 0.25f },
        { 0.5f, 0.f, 0.f, 0.5f, 0.5f, 0.0f, 0.50f },
        { 0.5f, 0.f, 0.f, 0.5f, 0.0f, 0.5f, 0.75f },
        { 0.5f, 0.f, 0.f, 0.5f, 0.5f, 0.5f, 1.00f },
      },
      -0.1f, 1.1f,  -0.1f, 1.1f,
    },

    /* 19. Vicsek — five third-size copies arranged in a plus shape. */
    { "Vicsek", 5,
      {
        { 0.333f, 0.f, 0.f, 0.333f, 0.333f, 0.333f, 0.20f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.000f, 0.333f, 0.40f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.667f, 0.333f, 0.60f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.333f, 0.000f, 0.80f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.333f, 0.667f, 1.00f },
      },
      -0.1f, 1.1f,  -0.1f, 1.1f,
    },

    /* 20. Cantor Dust — third-size copies at the four corners only. */
    { "Cantor Dust", 4,
      {
        { 0.333f, 0.f, 0.f, 0.333f, 0.000f, 0.000f, 0.25f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.667f, 0.000f, 0.50f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.000f, 0.667f, 0.75f },
        { 0.333f, 0.f, 0.f, 0.333f, 0.667f, 0.667f, 1.00f },
      },
      -0.1f, 1.1f,  -0.1f, 1.1f,
    },

    /* 21. Inverted Sierpinski — apex points down. */
    { "Inv Sierp", 3,
      {
        { 0.5f, 0.f, 0.f, 0.5f, 0.0f,  0.00f, 0.333f },
        { 0.5f, 0.f, 0.f, 0.5f, 1.0f,  0.00f, 0.667f },
        { 0.5f, 0.f, 0.f, 0.5f, 0.5f, -0.87f, 1.000f },
      },
      -0.1f, 1.1f, -1.1f, 0.1f,
    },

    /* ────── dragons ────── */

    /* 22. Heighway Dragon — two shrink-and-turn rules, one of them flipped. */
    { "Dragon", 2,
      {
        {  0.5f, -0.5f,  0.5f,  0.5f, 0.0f, 0.0f, 0.5f },
        { -0.5f,  0.5f, -0.5f, -0.5f, 1.0f, 0.0f, 1.0f },
      },
      -0.2f, 1.3f, -0.7f, 0.7f,
    },

    /* 23. Twindragon — two Heighway dragons fitted edge-to-edge. */
    { "Twindragon", 2,
      {
        { 0.5f, -0.5f,  0.5f, 0.5f, 0.0f,  0.0f, 0.5f },
        { 0.5f,  0.5f, -0.5f, 0.5f, 0.5f, -0.5f, 1.0f },
      },
      -0.5f, 1.5f, -1.2f, 0.7f,
    },

    /* 24. Terdragon — three shrink-and-turn rules at different angles. */
    { "Terdragon", 3,
      {
        { 0.5f, -0.289f,  0.289f, 0.5f, 0.000f, 0.000f, 0.333f },
        { 0.0f, -0.577f,  0.577f, 0.0f, 0.500f, 0.289f, 0.667f },
        { 0.5f, -0.289f,  0.289f, 0.5f, 0.500f,-0.289f, 1.000f },
      },
      -0.2f, 1.2f, -0.5f, 0.7f,
    },

    /* 25. Lévy C Curve — two shrink-and-turn rules that fold into a C shape. */
    { "Levy C", 2,
      {
        { 0.5f, -0.5f,  0.5f, 0.5f, 0.0f, 0.0f, 0.5f },
        { 0.5f,  0.5f, -0.5f, 0.5f, 0.5f, 0.5f, 1.0f },
      },
      -0.5f, 1.5f, -0.5f, 1.5f,
    },

    /* ────── curves & forms ────── */

    /* 26. Koch Curve — four third-size segments; the angled middle pair
     *     makes the familiar snowflake-edge bump. */
    { "Koch Curve", 4,
      {
        { 0.333f,  0.000f,  0.000f, 0.333f, 0.000f, 0.000f, 0.25f },
        { 0.167f, -0.289f,  0.289f, 0.167f, 0.333f, 0.000f, 0.50f },
        { 0.167f,  0.289f, -0.289f, 0.167f, 0.500f, 0.289f, 0.75f },
        { 0.333f,  0.000f,  0.000f, 0.333f, 0.667f, 0.000f, 1.00f },
      },
      -0.1f, 1.1f, -0.1f, 0.5f,
    },

    /* 27. Lévy Tapestry — Lévy C generalised to 4 maps, fills a square. */
    { "Levy Tap", 4,
      {
        { 0.50f, -0.20f,  0.20f, 0.50f, 0.0f, 0.0f, 0.25f },
        { 0.50f, -0.20f,  0.20f, 0.50f, 0.5f, 0.5f, 0.50f },
        { 0.50f,  0.20f, -0.20f, 0.50f, 0.0f, 0.5f, 0.75f },
        { 0.50f,  0.20f, -0.20f, 0.50f, 0.5f, 0.0f, 1.00f },
      },
      -0.5f, 1.5f, -0.5f, 1.5f,
    },

    /* 28. Cesàro Curve — two contractive rotations sweep a Cesàro shape. */
    { "Cesaro", 2,
      {
        { 0.4f, -0.3f,  0.3f, 0.4f, 0.0f, 0.0f, 0.5f },
        { 0.4f,  0.3f, -0.3f, 0.4f, 0.6f, 0.0f, 1.0f },
      },
      -0.2f, 1.2f, -0.3f, 0.5f,
    },

    /* 29. Spiral — Barnsley's 3-rule spiral; the first rule, picked most
     *     of the time, does most of the inward winding. */
    { "Spiral", 3,
      {
        { 0.787879f, -0.424242f, 0.242424f, 0.859848f,  1.758647f, 1.408065f, 0.895f },
        {-0.121212f,  0.257576f, 0.151515f, 0.053030f, -6.721654f, 1.377236f, 0.950f },
        { 0.181818f, -0.136364f, 0.090909f, 0.181818f,  6.086107f, 1.568035f, 1.000f },
      },
      -8.0f, 8.0f, -1.0f, 8.0f,
    },

    /* 30. Crystal — four shrunken copies pushed out up/down/left/right,
     *     giving a four-pointed star. */
    { "Crystal", 4,
      {
        { 0.4f, 0.f, 0.f, 0.4f,  0.0f,  0.6f, 0.25f },
        { 0.4f, 0.f, 0.f, 0.4f,  0.6f,  0.0f, 0.50f },
        { 0.4f, 0.f, 0.f, 0.4f,  0.0f, -0.6f, 0.75f },
        { 0.4f, 0.f, 0.f, 0.4f, -0.6f,  0.0f, 1.00f },
      },
      -1.1f, 1.1f, -1.1f, 1.1f,
    },
};

/* ===================================================================== */
/* §2  random — the fast generator that picks which rule to apply         */
/* ===================================================================== */

/* Start the generator from a different point each run by seeding it off
 * the clock.  Any non-zero start works; we just make sure it isn't zero. */
static void lcg_seed_from_clock(RandLCG *r)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    r->state = (uint32_t)(ts.tv_nsec ^ (ts.tv_sec * LCG_SEED_TIME_MIX));
    if (r->state == 0) r->state = 1u;
}

/* Roll forward one step and hand back the new value. */
static inline uint32_t lcg_next_u32(RandLCG *r)
{
    r->state = r->state * LCG_MULTIPLIER + LCG_INCREMENT;
    return r->state;
}

/* Give back a random number between 0 and 1.  We use the top bits of the
 * raw value because the bottom bits of this kind of generator are weaker. */
static inline float lcg_unit_float(RandLCG *r)
{
    uint32_t high_bits = lcg_next_u32(r) >> LCG_HIGH_BITS_SHIFT;
    return (float)high_bits / (float)LCG_UNIT_DENOM;
}

/* ===================================================================== */
/* §3  time — monotonic clock + rolling FPS                               */
/* ===================================================================== */

static long long clock_ns_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&ts, NULL);
}

/* Add this frame to the running tally, and once about half a second has
 * passed work out a fresh frames-per-second figure to show. */
static void fps_tick(FpsCounter *fps, long long frame_ns)
{
    fps->accum_ns += frame_ns;
    fps->frames   += 1;
    if (fps->accum_ns >= FPS_WINDOW_MS * NS_PER_MS) {
        fps->display = (double)fps->frames
                     / ((double)fps->accum_ns / (double)NS_PER_SEC);
        fps->accum_ns = 0;
        fps->frames   = 0;
    }
}

/* ===================================================================== */
/* §4  view & palette                                                     */
/* ===================================================================== */

/* ---- preset geometry queries ------------------------------------ */

static float preset_x_range(const IFSPreset *p) {
    float r = p->x_max - p->x_min;
    return r < EPS_RANGE ? EPS_RANGE : r;
}
static float preset_y_range(const IFSPreset *p) {
    float r = p->y_max - p->y_min;
    return r < EPS_RANGE ? EPS_RANGE : r;
}
static float preset_mid_x(const IFSPreset *p) {
    return (p->x_min + p->x_max) * VIEW_MIDPOINT_FRACTION;
}
static float preset_mid_y(const IFSPreset *p) {
    return (p->y_min + p->y_max) * VIEW_MIDPOINT_FRACTION;
}

/* ---- viewport_fit decomposition --------------------------------- */

/* How many rows of play area remain after reserving HUD rows. */
static int viewport_play_rows(int rows) {
    int n = rows - HUD_TOP_ROWS - HUD_BOTTOM_ROWS;
    return n < 1 ? 1 : n;
}

/* Aspect-correct scale: pick the tighter of (horizontal cell budget)
 * vs (vertical cell budget × 2 for cell aspect), so the view
 * rectangle fits both ways while preserving shape. */
static float viewport_aspect_fit_scale(const Viewport *v, const IFSPreset *p)
{
    float horizontal_budget =  (float)(v->cols      - EDGE_MARGIN_CELLS)
                                 / preset_x_range(p);
    float vertical_budget   =  (float)(v->play_rows - EDGE_MARGIN_CELLS)
                                 * ASPECT_CELL_HEIGHT / preset_y_range(p);
    return horizontal_budget < vertical_budget ? horizontal_budget : vertical_budget;
}

/* Place the gasket origin at the centre of the available play area. */
static void viewport_place_center(Viewport *v, const IFSPreset *p)
{
    v->center_col = v->cols / 2;
    v->center_row = HUD_TOP_ROWS + v->play_rows / 2;
    v->view_mid_x = preset_mid_x(p);
    v->view_mid_y = preset_mid_y(p);
}

/* Recompute the viewport for the current terminal size and active
 * preset.  Called on startup, resize, and any preset change. */
static void viewport_fit(Viewport *v, int rows, int cols, const IFSPreset *p)
{
    /* Step 1 — store the terminal dimensions and play-area height. */
    v->rows      = rows;
    v->cols      = cols;
    v->play_rows = viewport_play_rows(rows);

    /* Step 2 — choose the single scale that fits both axes. */
    v->scale = viewport_aspect_fit_scale(v, p);

    /* Step 3 — anchor the gasket origin at the screen centre. */
    viewport_place_center(v, p);
}

/* ---- viewport_project decomposition ----------------------------- */

/* Convert a gasket-x to a terminal column. */
static inline int viewport_project_col(const Viewport *v, float x)
{
    return v->center_col + (int)((x - v->view_mid_x) * v->scale);
}

/* Convert a gasket-y to a terminal row.  The 0.5× factor is the
 * cell-aspect correction; the minus sign flips gasket-y (points up)
 * to terminal row (points down). */
static inline int viewport_project_row(const Viewport *v, float y)
{
    return v->center_row - (int)((y - v->view_mid_y) * v->scale * ASPECT_INV);
}

/* Is a cell inside the play area (between the two HUD rows)? */
static inline bool viewport_cell_in_play_area(const Viewport *v, int col, int row)
{
    if (col < 0 || col >= v->cols) return false;
    if (row < HUD_TOP_ROWS || row >= v->rows - HUD_BOTTOM_ROWS) return false;
    return true;
}

/* Project a gasket-space point to a terminal cell.  Returns true if
 * the cell is inside the play area; false if it falls outside. */
static inline bool viewport_project(const Viewport *v,
                                    float x, float y,
                                    int *col_out, int *row_out)
{
    int col = viewport_project_col(v, x);
    int row = viewport_project_row(v, y);
    if (!viewport_cell_in_play_area(v, col, row)) return false;
    *col_out = col;
    *row_out = row;
    return true;
}

/* Map from density tier index (0..3) to the ncurses colour-pair slot. */
static const int k_tier_pair_slots[N_DENSITY_TIERS] = {
    CP_L1, CP_L2, CP_L3, CP_L4,
};

/* Pick the foreground colour for one tier, with 8-colour fallback. */
static int palette_tier_fg(const Palette *p, int tier)
{
    return (COLORS >= 256) ? p->fg256[tier] : p->fg8[tier];
}

/* Install the four density-tier pairs from the active palette.
 * Called at startup and on every palette cycle. */
static void palette_apply(int idx)
{
    const Palette *p = &g_palettes[idx];
    for (int tier = 0; tier < N_DENSITY_TIERS; tier++)
        init_pair(k_tier_pair_slots[tier], palette_tier_fg(p, tier), -1);
}

/* HUD pairs are theme-independent — bright yellow data + bright cyan
 * hints so the bars remain legible against any palette. */
static void palette_init_static(void)
{
    if (COLORS >= 256) {
        init_pair(CP_HUD,  226, -1);
        init_pair(CP_HINT,  51, -1);
    } else {
        init_pair(CP_HUD,  COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN,   -1);
    }
}

/* ===================================================================== */
/* §5  density grid                                                       */
/* ===================================================================== */

static void grid_clear(DensityGrid *g)
{
    memset(g->cells, 0, sizeof g->cells);
}

/* Bump one cell, saturating at HITS_CAP.  Out-of-range coordinates
 * are ignored (the caller is the viewport; clipping is a defence in
 * depth). */
static inline void grid_hit(DensityGrid *g, int row, int col)
{
    if (row < 0 || row >= GRID_ROWS_MAX) return;
    if (col < 0 || col >= GRID_COLS_MAX) return;
    uint16_t h = g->cells[row][col];
    if (h < (uint16_t)HITS_CAP)
        g->cells[row][col] = h + 1u;
}

/* Find the highest hit count in the visible region — used to
 * normalise log density into [0, 1] each frame. */
static uint32_t grid_max_hits(const DensityGrid *g,
                              int row_top, int row_bot, int cols)
{
    uint32_t max_hits = 1;
    int row_lim = row_bot < GRID_ROWS_MAX ? row_bot : GRID_ROWS_MAX;
    int col_lim = cols    < GRID_COLS_MAX ? cols    : GRID_COLS_MAX;
    for (int r = row_top; r < row_lim; r++)
        for (int c = 0; c < col_lim; c++)
            if (g->cells[r][c] > max_hits)
                max_hits = g->cells[r][c];
    return max_hits;
}

/* Map a normalised log-density value in [0, 1] to a tier index 0..3
 * (or -1 for background, signalling "don't draw"). */
static int density_tier(double t)
{
    if (t < DENSITY_BG_MAX) return -1;
    if (t < DENSITY_L1_MAX) return 0;
    if (t < DENSITY_L2_MAX) return 1;
    if (t < DENSITY_L3_MAX) return 2;
    return 3;
}

/* ===================================================================== */
/* §6  IFS & chaos game                                                   */
/* ===================================================================== */

/* Pick which affine map to apply next.  Iterates the preset's maps
 * in cumulative-probability order and returns the first whose cum is
 * ≥ rnd.  Linear scan is fine because n_maps ≤ 8. */
static const AffineMap *ifs_pick_map(const IFSPreset *p, float rnd)
{
    for (int k = 0; k < p->n_maps; k++)
        if (rnd <= p->maps[k].cum) return &p->maps[k];
    /* Fallback for float-rounding edge cases at rnd ≈ 1.0 */
    return &p->maps[p->n_maps - 1];
}

/* Apply one affine map: (x, y) → (a·x + b·y + e, c·x + d·y + f). */
static inline void affine_apply(const AffineMap *m,
                                float x, float y,
                                float *nx, float *ny)
{
    *nx = m->a * x + m->b * y + m->e;
    *ny = m->c * x + m->d * y + m->f;
}

/* Reset the orbit to a neutral starting point inside the basin of
 * attraction.  Burn-in of the first few iterations brings it onto
 * the actual attractor. */
static void orbit_reset(Orbit *o)
{
    o->cx = ORBIT_SEED_X;
    o->cy = ORBIT_SEED_Y;
}

/* One chaos-game step: draw a random map and apply it in place to
 * the orbit position.  Updates (*x, *y) with the new orbit point. */
static inline void chaos_step_one(const IFSPreset *preset, RandLCG *rng,
                                  float *x, float *y)
{
    const AffineMap *map = ifs_pick_map(preset, lcg_unit_float(rng));
    float nx, ny;
    affine_apply(map, *x, *y, &nx, &ny);
    *x = nx;
    *y = ny;
}

/* Project the orbit's current point to a grid cell and bump its
 * hit counter, if the point lands inside the play area. */
static inline void chaos_plot_orbit_hit(DensityGrid *grid, const Viewport *view,
                                        float x, float y)
{
    int col, row;
    if (viewport_project(view, x, y, &col, &row))
        grid_hit(grid, row, col);
}

/* Run iters chaos-game steps on the orbit, plotting each into the
 * density grid.  This is the program's hot loop — every iteration
 * is one map application + one viewport projection + one grid bump.
 *
 *     for each iteration:
 *         orbit ← T_i(orbit) for a randomly-chosen map i
 *         plot the new orbit point to the grid
 */
static void chaos_iterate(Scene *s)
{
    const IFSPreset *preset = &g_presets[s->preset];
    Orbit           *orbit  = &s->orbit;
    float x = orbit->cx;
    float y = orbit->cy;

    for (int i = 0; i < s->iters; i++) {
        chaos_step_one(preset, &orbit->rng, &x, &y);
        chaos_plot_orbit_hit(&s->grid, &s->view, x, y);
    }

    orbit->cx = x;
    orbit->cy = y;
}

/* ===================================================================== */
/* §7  render & HUD                                                       */
/* ===================================================================== */

/* Density tier → glyph lookup.  (The tier → colour-pair lookup
 * `k_tier_pair_slots[]` lives in §4 with palette_apply.) */
static const chtype k_tier_glyphs[N_DENSITY_TIERS] = {
    GLYPH_L1, GLYPH_L2, GLYPH_L3, GLYPH_L4,
};

/* Plot a single density tier into a screen cell.  The densest tier
 * gets A_BOLD so the peaks of the fractal stand out. */
static void plot_density_cell(int row, int col, int tier)
{
    chtype glyph = k_tier_glyphs[tier];
    int    pair  = k_tier_pair_slots[tier];
    attr_t bold  = (tier == N_DENSITY_TIERS - 1) ? A_BOLD : 0;

    attron(COLOR_PAIR(pair) | bold);
    mvaddch(row, col, glyph);
    attroff(COLOR_PAIR(pair) | bold);
}

/* Clamp log1p output so the denominator can never be ~0. */
static inline double log1p_safe(double x)
{
    double v = log1p(x);
    return v < EPS_LOG_MAX ? EPS_LOG_MAX : v;
}

/* Log-tone map: hits → normalised brightness in [0, 1].
 * See Draves & Reckase, "The Fractal Flame Algorithm" (2003) §4. */
static inline double log_tone_map(uint16_t hits, double log_max)
{
    return log1p((double)hits) / log_max;
}

/* Compute the rectangle of grid cells we should walk this frame.
 * Clipped to both the play area (between HUD rows) and the static
 * grid bounds. */
static void grid_render_walk_bounds(const Viewport *v,
                                    int *row_top, int *row_lim, int *col_lim)
{
    int row_bot = v->rows - HUD_BOTTOM_ROWS;
    *row_top    = HUD_TOP_ROWS;
    *row_lim    = row_bot < GRID_ROWS_MAX ? row_bot : GRID_ROWS_MAX;
    *col_lim    = v->cols < GRID_COLS_MAX ? v->cols : GRID_COLS_MAX;
}

/* Convert one cell's hit count into a density tier + plot.  Returns
 * silently if the cell is empty or below the background threshold. */
static void grid_render_one_cell(uint16_t hits, double log_max, int row, int col)
{
    if (hits == 0) return;
    int tier = density_tier(log_tone_map(hits, log_max));
    if (tier < 0) return;
    plot_density_cell(row, col, tier);
}

/* Render the density grid as glyph-tier ASCII.
 *
 *     1. Find this frame's peak density (for normalisation).
 *     2. Walk every cell in the play area.
 *     3. Map each cell's hit count → tier via log-tone, plot. */
static void grid_render(const DensityGrid *g, const Viewport *v)
{
    int row_top, row_lim, col_lim;
    grid_render_walk_bounds(v, &row_top, &row_lim, &col_lim);

    uint32_t peak    = grid_max_hits(g, row_top, row_lim, v->cols);
    double   log_max = log1p_safe((double)peak);

    for (int row = row_top; row < row_lim; row++)
        for (int col = 0; col < col_lim; col++)
            grid_render_one_cell(g->cells[row][col], log_max, row, col);
}

/* Top HUD row — bright yellow data line. */
static void hud_draw_data(const Scene *s)
{
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0,
        " [%d/%d] %-14s   theme:%-7s   iters:%5d   %5.1f fps   %s",
        s->preset + 1, N_PRESETS,
        g_presets[s->preset].name,
        g_palettes[s->palette].name,
        s->iters,
        s->fps.display,
        s->paused ? "PAUSED" : "      ");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Bottom HUD row — bright cyan key-hint line. */
static void hud_draw_hint(const Scene *s)
{
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(s->view.rows - 1, 0,
        " q:quit  p:pause  r:reset  n/N:preset  t/T:theme  +/-:iters ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void hud_draw(const Scene *s)
{
    hud_draw_data(s);
    hud_draw_hint(s);
}

/* ===================================================================== */
/* §8  scene & main                                                       */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void on_exit_cleanup(void) { endwin(); }

/* ---- scene action helpers (named for what the user sees) --------- */

/* Clear the density grid and reseed the orbit — restart the chaos
 * game from scratch without changing preset or palette. */
static void scene_rebuild(Scene *s)
{
    grid_clear(&s->grid);
    orbit_reset(&s->orbit);
}

/* Cycle preset by ±1 (wraps); refit viewport; restart the chaos game. */
static void scene_cycle_preset(Scene *s, int dir)
{
    s->preset = (s->preset + dir + N_PRESETS) % N_PRESETS;
    viewport_fit(&s->view, s->view.rows, s->view.cols, &g_presets[s->preset]);
    scene_rebuild(s);
}

/* Cycle palette by ±1 (wraps).  Doesn't touch the grid or orbit. */
static void scene_cycle_palette(Scene *s, int dir)
{
    s->palette = (s->palette + dir + N_PALETTES) % N_PALETTES;
    palette_apply(s->palette);
}

/* Step iters by `delta`, clamped to [ITERS_MIN, ITERS_MAX]. */
static void scene_change_iters(Scene *s, int delta)
{
    int n = s->iters + delta;
    if (n < ITERS_MIN) n = ITERS_MIN;
    if (n > ITERS_MAX) n = ITERS_MAX;
    s->iters = n;
}

/* ---- scene lifecycle --------------------------------------------- */

/* Read terminal dimensions, clamp to grid limits. */
static void scene_read_term_size(int *rows_out, int *cols_out)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    *rows_out = rows;
    *cols_out = cols;
}

/* Defaults for a fresh scene — first preset, first palette, default
 * iteration count, not paused, FPS counter zeroed. */
static void scene_init_options(Scene *s)
{
    s->preset  = DEFAULT_PRESET_IDX;
    s->palette = DEFAULT_PALETTE_IDX;
    s->iters   = ITERS_DEFAULT;
    s->paused  = false;
    s->fps     = (FpsCounter){ .accum_ns = 0, .frames = 0, .display = 0.0 };
}

/* Seed the orbit's RNG from the wall clock and position the orbit
 * at its burn-in starting point. */
static void scene_init_orbit(Scene *s)
{
    lcg_seed_from_clock(&s->orbit.rng);
    orbit_reset(&s->orbit);
}

/* Read the terminal size, fit the viewport to the active preset,
 * and clear the density grid for a fresh draw. */
static void scene_init_view_and_grid(Scene *s)
{
    int rows, cols;
    scene_read_term_size(&rows, &cols);
    viewport_fit(&s->view, rows, cols, &g_presets[s->preset]);
    grid_clear(&s->grid);
}

/* Set the scene to its starting state.
 *
 *     1. options    — preset, palette, iters, pause, FPS
 *     2. orbit      — RNG seed + start position
 *     3. view/grid  — viewport from terminal + preset; clear grid */
static void scene_init(Scene *s)
{
    scene_init_options(s);
    scene_init_orbit(s);
    scene_init_view_and_grid(s);
}

/* Refit the viewport after SIGWINCH and restart the chaos game. */
static void scene_resize(Scene *s)
{
    endwin();
    refresh();
    int rows, cols;
    scene_read_term_size(&rows, &cols);
    viewport_fit(&s->view, rows, cols, &g_presets[s->preset]);
    scene_rebuild(s);
}

/* React to a single key.  Each case maps to a named scene operation,
 * so the dispatch table reads as a mapping from input to intent. */
static void scene_handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case KEY_ESCAPE:  g_quit = 1;                          break;
    case 'p': case ' ':                   s->paused = !s->paused;              break;
    case 'r': case 'R':                   scene_rebuild(s);                    break;
    case 'n':                             scene_cycle_preset(s,  +1);          break;
    case 'N':                             scene_cycle_preset(s,  -1);          break;
    case 't':                             scene_cycle_palette(s, +1);          break;
    case 'T':                             scene_cycle_palette(s, -1);          break;
    case '+': case '=':                   scene_change_iters(s, +ITERS_STEP);  break;
    case '-':                             scene_change_iters(s, -ITERS_STEP);  break;
    default: break;
    }
}

/* Drain all pending keystrokes (nodelay returns ERR when none). */
static void scene_process_input(Scene *s)
{
    int ch;
    while ((ch = getch()) != ERR)
        scene_handle_key(s, ch);
}

/* Advance the chaos game by one frame's worth of iterations.  Skips
 * the work entirely while paused. */
static void scene_tick(Scene *s)
{
    if (!s->paused) chaos_iterate(s);
}

/* ---- frame ------------------------------------------------------- */

/* Composite the grid + HUD and flush to the terminal. */
static void frame_render(const Scene *s)
{
    erase();
    grid_render(&s->grid, &s->view);
    hud_draw(s);
    wnoutrefresh(stdscr);
    doupdate();
}

/* Pace the frame: update the rolling FPS counter with the elapsed
 * duration, then sleep what's left of the frame budget so we hold
 * a stable rate. */
static void frame_pace_to_target(long long frame_start, FpsCounter *fps)
{
    long long frame_dur = clock_ns_now() - frame_start;
    fps_tick(fps, frame_dur);
    clock_sleep_ns(FRAME_NS - frame_dur);
}

/* ---- ncurses + signal setup -------------------------------------- */

static void app_init(void)
{
    atexit(on_exit_cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    initscr();
    cbreak();
    noecho();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);

    start_color();
    use_default_colors();
    palette_init_static();
}

/* ---- main loop --------------------------------------------------- */

int main(void)
{
    app_init();

    static Scene scene;
    scene_init(&scene);
    palette_apply(scene.palette);

    while (!g_quit) {
        if (g_resize) { g_resize = 0; scene_resize(&scene); }

        long long frame_start = clock_ns_now();
        scene_process_input(&scene);
        scene_tick(&scene);
        frame_render(&scene);
        frame_pace_to_target(frame_start, &scene.fps);
    }
    return 0;
}
