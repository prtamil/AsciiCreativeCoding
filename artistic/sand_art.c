/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sand_art.c — sands of time: curved hourglass with falling-sand
 *              simulation, decorative frame, time tracking + auto-flip.
 *
 * DEMO: A proper iconic hourglass with quarter-elliptical curved bulbs
 *       (not the linear-tapered triangles of the previous version)
 *       sits centred in a wooden frame.  Eight coloured sand layers
 *       fill the top bulb in geological strata.  A falling-sand
 *       cellular automaton with per-cell momentum accumulation runs
 *       the physics: grains move down (or up after flip), diagonals
 *       in random order if blocked, and free-falling grains in the
 *       neck stream gain momentum so they advance multiple cells
 *       per tick — giving the visible streaming acceleration through
 *       the narrow waist that a pure 1-cell-per-tick CA can't produce.
 *
 *       Press space to flip gravity → the sand avalanches the other
 *       way, layers visibly mix as they fall, and a new landscape
 *       forms in the formerly empty bulb.  HUD tracks `Time: NN%`
 *       (grains transferred / total) and `Flips: N`.
 *
 *       Patterns (cycle with n / N):
 *         NORMAL       8 horizontal layers, classic hourglass timer
 *         RAINBOW      16 thinner layers, vivid colour spectrum
 *         GEOLOGICAL   8 layers with random thickness asymmetry
 *         DUAL_FLOW    sand in BOTH bulbs, perpetual flip drama
 *
 *       Themes (cycle with t / T):
 *         BEACH        golden sand, brown frame
 *         VOLCANIC     black sand, dark frame, glowing red veins
 *         ROYAL        purple + gold sand, gold frame
 *         ICE          blue/white sand, silver frame
 *         ALIEN        multicolour neon, glowing
 *         NEGATIVE     white-paper bg + dark sand (inverted)
 *
 * Section map:
 *   §1 config    — themes, patterns, hourglass + frame geometry
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-tier sand palette per theme
 *   §4 utilities — RNG, hashes
 *   §5 hourglass — curved bulb mask, frame decoration coords
 *   §6 sand      — grid + CA + momentum tick + flip + transfer count
 *   §7 scene     — compose render passes
 *   §8 screen + app
 *
 * Keys:
 *   q / ESC      quit
 *   space / f    flip gravity (DOWN ↔ UP)
 *   a            toggle auto-flip (default ON: flips when fully drained)
 *   r            refill sand layers (reset transfer count)
 *   n / N        next / previous pattern
 *   t / T        next / previous theme
 *   p            pause / resume simulation
 *   ] / [        sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/sand_art.c \
 *       -o sand_art -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Falling-sand cellular automaton with per-cell
 *                  MOMENTUM accumulation, on a curved-bulb hourglass
 *                  mask.
 *
 *                  GRID:
 *                     `mask[row][col]` is 1 inside the hourglass cavity
 *                     (sand can occupy), 0 outside (impassable).
 *                     `sand[row][col]` is the colour layer index of any
 *                     grain at that cell, or 0 if empty.
 *                     `momentum[row][col]` is per-cell vertical velocity
 *                     (0 = at rest, >0 = falling fast).
 *
 *                  TICK (when gravity is DOWN):
 *                     Scan rows BOTTOM → TOP (so a grain doesn't move
 *                     twice in one tick).  For each filled cell:
 *                       - Try direct movement direction (down).  If
 *                         destination empty AND inside mask: move.
 *                         Increment momentum.
 *                       - Else try diagonals (down-left, down-right)
 *                         in random order.  Reset momentum on diagonal.
 *                       - Else: at rest.  Reset momentum.
 *                     Free-falling cells (advanced multiple cells via
 *                     momentum) speed up the visible neck stream.
 *
 *                  FLIP:
 *                     Toggle gravity direction.  Scan order reverses.
 *                     Sand piles re-form with new dynamics.
 *
 *                  HOURGLASS SHAPE:
 *                     Two bulbs joined at a narrow NECK row range.
 *                     Each bulb's half-width as a function of vertical
 *                     position uses a QUARTER-ELLIPSE curve:
 *                       half_w(p) = NECK_HALF
 *                                 + (FULL_HALF − NECK_HALF) · √(1 − (1−p)²)
 *                     where p ∈ [0, 1] is the bulb's vertical fraction
 *                     (0 = at the neck, 1 = at the bulb's far end).
 *                     This produces the iconic "rounded hourglass"
 *                     silhouette instead of straight triangular taper.
 *
 *                  TIME TRACKING:
 *                     Count grains that have crossed the neck row.
 *                     `transferred / total` is the elapsed-time fraction.
 *                     Reaches 1.0 when all sand has flowed through;
 *                     resets on flip or refill.
 *
 * Data-structure : Three parallel grids: mask (uint8), sand (uint8 colour
 *                  index 0..LAYER_MAX), momentum (uint8 0..MOMENTUM_MAX).
 *                  Static buffers sized for SCREEN_MAX_W × SCREEN_MAX_H.
 *                  Frame decoration: a few hard-coded glyph patterns
 *                  drawn at the cap rows.
 *
 * Rendering      : ASCII only.  Sand glyphs vary with momentum: at-rest
 *                  cells render dense ('@', '#'), falling cells render
 *                  lighter ('*', '+', '.') so the streaming neck looks
 *                  visibly faster than the static piles.  Hourglass
 *                  walls are '|', '/', '\\' depending on slope; frame
 *                  uses '=' caps and '|' rails.
 *
 * Performance    : Per tick: cols·rows mask check + sand moves.  At
 *                  80×24 ≈ 2 K cell visits.  CA is O(N) in occupied
 *                  cells, not O(N²); even with 1500 grains it's
 *                  trivial at 60 Hz.
 *
 * References     :
 *   • Toffoli, T. & Margolus, N. (1987) — *Cellular Automata Machines*
 *     §7 (sand pile / falling sand CA).
 *   • Bak, P., Tang, C. & Wiesenfeld, K. (1988) — "Self-Organized
 *     Criticality", *Phys. Rev. A* 38.  The angle-of-repose physics
 *     emerges from the same diagonal-fall rule we use.
 *   • Niksic, P. — "Powder Toy" (2008+).  Modern interactive falling-
 *     sand sandbox; the per-cell momentum accumulator we use is from
 *     this lineage of CA games.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * An hourglass is a closed shape with sand inside.  Each grain has one
 * job per tick: try to move with gravity.  If the cell directly with
 * gravity is empty AND inside the hourglass, go there.  Otherwise try
 * the two diagonal-with-gravity cells.  Otherwise rest.  Doing this for
 * every grain, every tick, gives proper sand-pile behaviour: cones
 * form, slopes settle at the angle of repose, narrow openings (the
 * neck) bottleneck flow.  Adding momentum lets free-falling grains
 * advance multiple cells per tick so the neck stream visibly accelerates.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine you're a sand grain.  Each tick the universe asks "can you
 * fall straight down?"  If yes, you fall.  If not, "can you slide down
 * to your left or right?"  If yes, you slide (random choice between
 * left/right).  If everything below is blocked, you stay put.  When
 * gravity flips, "down" becomes "up" — same questions, opposite
 * direction.  The hourglass shape constrains where you can go.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. STARTUP — build hourglass mask:
 *
 *        For each (row, col):
 *            decide which bulb (top/bottom) this row belongs to
 *            compute that bulb's half_w via quarter-ellipse:
 *                p = vertical_progress_in_bulb(row)
 *                half_w = NECK_HALF + (FULL_HALF − NECK_HALF) · √(1 − (1−p)²)
 *            mask[row][col] = (|col − cx| ≤ half_w) ? 1 : 0
 *
 *     Neck rows: half_w = NECK_HALF (constant width).
 *
 *  2. STARTUP — fill sand layers:
 *
 *        Determine TOP bulb cells.  Divide vertically into N_LAYERS
 *        bands.  Each band gets one colour index (1..N_LAYERS).
 *        sand[row][col] = layer for cells in that band; 0 elsewhere.
 *
 *  3. PER TICK (when gravity is DOWN):
 *
 *        Scan rows from BOTTOM to TOP, cols left-to-right.
 *        For each filled cell (row, col):
 *
 *            // Try straight-down
 *            if mask[row+1][col] AND sand[row+1][col] == 0:
 *                sand[row+1][col] = sand[row][col]
 *                sand[row][col] = 0
 *                momentum[row+1][col] = min(momentum[row][col]+1, MAX_MOM)
 *                momentum[row][col] = 0
 *                // High-momentum cells get an additional drop
 *                if momentum[row+1][col] >= 2 AND
 *                   mask[row+2][col] AND sand[row+2][col] == 0:
 *                    sand[row+2][col] = sand[row+1][col]
 *                    sand[row+1][col] = 0
 *                    momentum[row+2][col] = momentum[row+1][col]
 *                    momentum[row+1][col] = 0
 *                continue
 *
 *            // Try diagonals in random order
 *            (left_first = random) → try those two
 *            on success: move, reset momentum (diagonals reset speed)
 *
 *            // No move: at rest, momentum decays to 0
 *
 *  4. FLIP — toggle gravity_dir, reverse scan direction.  Reset all
 *     momentums (avalanche starts from rest).
 *
 *  5. TIME COUNTING — when a grain crosses the neck row(s) downward
 *     (or upward), increment `transferred`.  HUD shows
 *     `transferred / total · 100` as %.
 *
 * KEY FORMULAS
 * ────────────
 *  Bulb half-width via quarter-ellipse profile:
 *    half_w(p) = NECK_HALF + (FULL_HALF − NECK_HALF) · √(1 − (1−p)²)
 *  where p ∈ [0, 1] is fractional distance from neck toward bulb's far
 *  end.  At p=0 (neck) half_w = NECK_HALF; at p=1 (bulb tip)
 *  half_w = FULL_HALF.  Curve is concave-out — the bulb curves
 *  outward from the neck before flattening at the cap.
 *
 *  Diagonal random order (deterministic CA needs random tie-break):
 *    if hash3(row, col, frame) & 1:  try left_first
 *    else:                            try right_first
 *
 *  Cell-aspect handling:
 *    Hourglass is taller than wide in cell space because terminal
 *    cells are 2× taller than wide.  We don't aspect-correct the
 *    bulb shape — a hourglass IS naturally taller than wide.  But
 *    a "circular bulb top" would need correction: not used here.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • SCAN ORDER vs MOVEMENT DIRECTION.  If we scanned top-to-bottom
 *    while gravity points down, a grain that just moved down would
 *    be processed AGAIN at its new row → it could fall multiple
 *    cells per tick uncontrollably.  Bottom-to-top scan ensures
 *    each grain moves at most once per tick (except via momentum,
 *    which is explicit).  Same logic in reverse for flipped gravity.
 *
 *  • NECK BOTTLENECK is a feature not a bug.  Only NECK_HALF·2 cells
 *    wide; many grains compete; angle-of-repose dynamics naturally
 *    queue them up.  Time-to-empty depends on NECK_HALF + total
 *    sand volume + momentum cap.
 *
 *  • DIAGONAL BIAS.  Always trying left-first creates pile asymmetry.
 *    Random order per-cell-per-tick fixes it.
 *
 *  • MOMENTUM RESET.  When a grain hits a wall or piles up, reset to
 *    0 immediately — otherwise it'd "phantom-jump" through the pile.
 *
 *  • REFILL WITH FLIPPED GRAVITY.  `r` always refills the bulb that's
 *    "up" relative to current gravity.  That's the user-intuitive
 *    behaviour ("more sand!") regardless of orientation.
 *
 *  • DUAL_FLOW pattern starts with sand in BOTH bulbs.  Flipping
 *    means the larger bulb avalanches; the smaller bulb fills more.
 *    No "drained empty" state.
 *
 *  • PAUSE freezes simulation but not input — flip / theme / pattern
 *    cycle still respond.  Resume picks up cleanly.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • At startup, top bulb is full of stratified colour bands; a thin
 *    stream begins flowing through the neck into the bottom bulb.
 *
 *  • Bottom bulb fills as a CONE (sand piles centred under the neck,
 *    spreading at the angle of repose).  The cone's apex sits
 *    directly under the neck.
 *
 *  • Press space → gravity flips.  Sand currently in the top bulb
 *    avalanches downward (which is now upward in flipped frame); the
 *    lower bulb's pile starts emptying upward through the neck.
 *
 *  • HUD: `Time: NN%` rises monotonically until 100, then plateaus
 *    until you flip.  `Flips: N` increments on every space press.
 *
 *  • Press `r` → bulb refills (whichever is "up" per current gravity)
 *    with fresh stratified layers; transfer count resets to 0.
 *
 *  • Press `n` to RAINBOW → 16 thinner layers visible in the top bulb.
 *
 *  • Press `n` to DUAL_FLOW → both bulbs start half-full; flipping
 *    moves sand the OTHER way without ever fully draining either.
 *
 *  • Press `t` cycles theme.  Hourglass shape unchanged; only colours.
 *
 *  • Watch the NECK during heavy flow.  You should see a visible
 *    streaming column with lighter (faster-moving) glyphs in the
 *    middle and denser (resting) glyphs at the pile boundaries.
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
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    FPS_UPDATE_MS    = 500,

    PAIR_HUD          =  1,
    PAIR_HINT         =  2,
    PAIR_FRAME        =  3,
    PAIR_WALL         =  4,
    PAIR_LAYER_BASE   =  5,    /* +0..+15 — sand colour layers       */
    PAIR_PAPER        = 22,    /* NEGATIVE bg                        */

    /* Static-buffer caps. */
    SCREEN_MAX_W      = 200,
    SCREEN_MAX_H      = 80,

    /* Sand layers: max LAYER_MAX colours.  RAINBOW pattern uses up
     * to 16; NORMAL/GEOLOGICAL use 8.                              */
    LAYER_MAX         = 16,

    /* Per-cell momentum: 0 (rest) .. MOMENTUM_MAX (free-fall).
     * Stored as uint8 to keep grids small. */
    MOMENTUM_MAX      = 4,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/*
 * Hourglass geometry — tunables.  Narrower than v1's wider container
 * to look like a real hourglass: bulb-to-neck ratio ~8:1 vs the
 * old 27:1 (which made the bulbs feel like wide funnels).
 */
#define HG_FULL_FRAC     0.16f     /* full half-width as frac of cols    */
#define HG_NECK_HALF     1.0f      /* neck half-width (cells); 1 → neck is 3 cells wide */
#define HG_NECK_ROWS     2         /* number of rows where neck is at MIN width */
#define HG_HEIGHT_FRAC   0.86f     /* height as frac of rows_eff         */
#define HG_FRAME_PAD     1         /* extra rows at top + bottom for frame */

/* Sand layer fill — fraction of TOP bulb height to fill initially. */
#define HG_FILL_FRAC     0.92f

/*
 * Auto-flip — fires when the source bulb has FULLY drained (every
 * grain has crossed the neck) AND the simulation has settled.  Models
 * real hourglass behaviour: watch it complete, THEN flip it.
 *
 * AUTO_FLIP_SETTLE is the post-completion pause (frames of no movement
 * before flipping) so the user sees the drained state for half a sec.
 *
 * AUTO_FLIP_STALL is a fallback: if the sim has been settled for this
 * many consecutive frames but the bulb hasn't fully drained (grains
 * stuck in repose pockets above the neck), flip anyway.  The flip
 * itself unsticks them.  Without this, a stuck final 1-2 % could
 * lock the demo at "almost done" forever.
 */
#define AUTO_FLIP_SETTLE     30     /* settle frames after completion    */
#define AUTO_FLIP_STALL     180     /* hard stall fallback (~3 sec)      */

/* Pattern enum + per-pattern params. */
typedef enum {
    PAT_NORMAL     = 0,
    PAT_RAINBOW    = 1,
    PAT_GEOLOGICAL = 2,
    PAT_DUAL_FLOW  = 3,
    N_PATTERNS     = 4,
} Pattern;

typedef struct {
    const char *name;
    int         layer_count;       /* 1..LAYER_MAX                       */
    bool        irregular;         /* layer thicknesses random?           */
    bool        dual_fill;         /* both bulbs filled at start?         */
} PatternParams;

static const PatternParams patterns[N_PATTERNS] = {
    /* name           layers  irregular  dual */
    { "NORMAL    ",    8,      false,    false },
    { "RAINBOW   ",   16,      false,    false },
    { "GEOLOGICAL",    8,      true,     false },
    { "DUAL_FLOW ",    8,      false,    true  },
};

/* Theme. */
typedef struct {
    const char *name;
    short       sand[16];         /* up to LAYER_MAX colours              */
    short       frame;            /* wooden frame                         */
    short       wall;             /* hourglass glass walls                */
    bool        inverted;
} Theme;

#define N_THEMES 6

static const Theme themes[N_THEMES] = {
    /* BEACH: golden sands, brown frame, sandy walls.                   */
    { "BEACH    ",
      { 220, 222, 214, 215, 208, 209, 173, 137,
        130, 138, 144, 180, 187, 222, 226, 229 },
      130, 137, false },

    /* VOLCANIC: deep crimson + black sand, dark frame.                 */
    { "VOLCANIC ",
      { 196, 202, 208, 130, 124,  88,  52, 235,
        237, 239, 241, 124, 160, 196, 208, 220 },
       60,  66, false },

    /* ROYAL: purple + gold sands, gold frame.                          */
    { "ROYAL    ",
      { 220, 226, 178, 173, 134, 135, 141, 105,
         99,  93, 165, 207, 213, 219, 220, 226 },
      136, 137, false },

    /* ICE: blue/white sands, silver frame.                             */
    { "ICE      ",
      {  24,  31,  38,  45,  87, 117, 153, 195,
        159, 117,  87,  45,  39,  33,  27,  24 },
      247, 195, false },

    /* ALIEN: vivid neon multicolour.                                    */
    { "ALIEN    ",
      {  46,  82, 118, 154, 190, 226, 220, 208,
       196, 162, 165, 129,  93,  56,  21,  27 },
       51, 159, false },

    /* NEGATIVE: white-paper bg, dark sand layers (greyscale).           */
    { "NEGATIVE ",
      { 232, 234, 236, 238, 240, 243, 245, 247,
        249, 251, 252, 253, 254,  16, 234, 240 },
       16,  60, true  },
};

/* Glyph for sand depending on momentum: at rest = dense, falling = light. */
static const char SAND_AT_REST[]   = "@#";
static const char SAND_MOVING[]    = "*+";
static const char SAND_FAST[]      = ".,";

#define WALL_GLYPH_V       '|'
#define WALL_GLYPH_DR      '\\'    /* slope going down-right */
#define WALL_GLYPH_DL      '/'     /* slope going down-left  */

#define FRAME_CAP_GLYPH    '='
#define FRAME_RAIL_GLYPH   '|'
#define FRAME_CORNER       '+'

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
        for (int i = 0; i < 16; i++)
            init_pair((short)(PAIR_LAYER_BASE + i), t->sand[i], bg);
        init_pair(PAIR_FRAME, t->frame, bg);
        init_pair(PAIR_WALL,  t->wall,  bg);
        init_pair(PAIR_PAPER, 16, t->inverted ? 231 : -1);
    } else {
        static const short fb[16] = {
            COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_RED,
            COLOR_MAGENTA, COLOR_MAGENTA, COLOR_BLUE, COLOR_BLUE,
            COLOR_CYAN, COLOR_CYAN, COLOR_GREEN, COLOR_GREEN,
            COLOR_WHITE, COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW
        };
        for (int i = 0; i < 16; i++)
            init_pair((short)(PAIR_LAYER_BASE + i),
                      t->inverted ? COLOR_BLACK : fb[i],
                      t->inverted ? COLOR_WHITE : (short)-1);
        init_pair(PAIR_FRAME,
                  t->inverted ? COLOR_BLACK : COLOR_YELLOW,
                  t->inverted ? COLOR_WHITE : (short)-1);
        init_pair(PAIR_WALL,
                  t->inverted ? COLOR_BLACK : COLOR_WHITE,
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
/* §4  utilities                                                          */
/* ===================================================================== */

static uint32_t g_rng = 0x12345678u;

static inline uint32_t lcg_next(uint32_t *st)
{
    *st = *st * 1664525u + 1013904223u;
    return *st;
}

static inline float lcg_unit(uint32_t *st)
{
    return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

static inline int lcg_pick(uint32_t *st)
{
    return (int)(lcg_next(st) & 1u);
}

/* Hash three ints to uint32 — used for per-cell-per-frame random
 * tie-break in diagonal-direction selection (deterministic given
 * frame number, so it's reproducible for a given simulation). */
static inline uint32_t hash3(int a, int b, int c)
{
    uint32_t h = (uint32_t)a * 374761393u
               + (uint32_t)b * 668265263u
               + (uint32_t)c * 2147483647u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}

/* ===================================================================== */
/* §5  hourglass — curved bulb mask + frame decoration                    */
/* ===================================================================== */

typedef struct {
    /* cavity[r][c] = 1 if inside the hourglass cavity (sand-able);
     *               0 if outside (impassable / wall / outside-frame).
     *
     * Built once per resize/reseed; never mutated during simulation. */
    uint8_t cavity[SCREEN_MAX_H][SCREEN_MAX_W];

    /* Hourglass bounding rows/cols within the screen. */
    int hg_top;        /* top bulb cap row                              */
    int hg_bot;        /* bottom bulb cap row (inclusive)               */
    int hg_neck_top;   /* first row of neck (constant width)            */
    int hg_neck_bot;   /* last row of neck (inclusive)                  */
    int hg_cx;         /* horizontal centre column                      */
    int hg_full_half;  /* full half-width at bulb caps                   */
    int hg_neck_half;  /* neck half-width                                */

    /* Frame decoration rows: top/bottom caps. */
    int frame_top_row;
    int frame_bot_row;
    int frame_left_col;
    int frame_right_col;
} Hourglass;

static Hourglass g_hg;

/*
 * hourglass_build — generate the cavity mask.
 *
 * Two bulbs joined at a HG_NECK_ROWS-row neck of constant width.
 * Each bulb's half-width along its vertical axis follows a quarter-
 * ellipse profile so the silhouette is properly rounded (not a
 * straight triangular taper).
 *
 *   half_w(p) = NECK_HALF + (FULL_HALF − NECK_HALF) · √(1 − (1 − p)²)
 *
 * where p ∈ [0, 1] is the fractional distance from neck toward the
 * bulb's far end (p=0 at neck, p=1 at bulb cap).  Setting q = 1 − p:
 *   half_w(q) = NECK_HALF + (FULL_HALF − NECK_HALF) · √(1 − q²)
 * which is just a quarter-ellipse with semi-axes
 * `(FULL_HALF − NECK_HALF, bulb_height)`.
 */
static void hourglass_build(Hourglass *hg, int cols, int rows_eff)
{
    /* Clear mask. */
    for (int r = 0; r < rows_eff && r < SCREEN_MAX_H; r++)
        for (int c = 0; c < cols && c < SCREEN_MAX_W; c++)
            hg->cavity[r][c] = 0;

    /* Hourglass dimensions. */
    int hg_height = (int)((float)rows_eff * HG_HEIGHT_FRAC);
    if (hg_height < 12) hg_height = (rows_eff > 12) ? 12 : rows_eff - 4;
    if (hg_height < 8)  hg_height = 8;
    int half_w = (int)((float)cols * HG_FULL_FRAC);
    if (half_w < 6)  half_w = 6;
    if (half_w > cols / 2 - 3) half_w = cols / 2 - 3;
    if (half_w < 3)  half_w = 3;
    int neck_half = (int)HG_NECK_HALF;

    int top    = (rows_eff - hg_height) / 2;
    int bot    = top + hg_height - 1;
    if (top < HG_FRAME_PAD)      top = HG_FRAME_PAD;
    if (bot > rows_eff - 1 - HG_FRAME_PAD) bot = rows_eff - 1 - HG_FRAME_PAD;
    int neck_top = (top + bot) / 2 - HG_NECK_ROWS / 2;
    int neck_bot = neck_top + HG_NECK_ROWS - 1;
    if (neck_top <= top + 1)       neck_top = top + 2;
    if (neck_bot >= bot - 1)       neck_bot = bot - 2;
    if (neck_bot < neck_top)       neck_bot = neck_top;

    int cx = cols / 2;

    hg->hg_top       = top;
    hg->hg_bot       = bot;
    hg->hg_neck_top  = neck_top;
    hg->hg_neck_bot  = neck_bot;
    hg->hg_cx        = cx;
    hg->hg_full_half = half_w;
    hg->hg_neck_half = neck_half;

    /* Frame coords. */
    hg->frame_top_row   = top - 1;
    hg->frame_bot_row   = bot + 1;
    hg->frame_left_col  = cx - half_w - 1;
    hg->frame_right_col = cx + half_w + 1;
    if (hg->frame_top_row < 0)              hg->frame_top_row = 0;
    if (hg->frame_bot_row >= rows_eff)      hg->frame_bot_row = rows_eff - 1;
    if (hg->frame_left_col < 0)             hg->frame_left_col = 0;
    if (hg->frame_right_col >= cols)        hg->frame_right_col = cols - 1;

    /* Carve top bulb (rows top..neck_top-1) — quarter-ellipse profile. */
    int top_h = neck_top - top;
    if (top_h < 1) top_h = 1;
    for (int r = top; r < neck_top; r++) {
        /* p = 0 at the neck (r = neck_top - 1), p = 1 at the bulb top
         * (r = top).  So p = (neck_top - 1 - r) / (top_h - 1). */
        float p = (top_h > 1) ? (float)(neck_top - 1 - r) / (float)(top_h - 1)
                              : 0.0f;
        float q = 1.0f - p;
        float hw_f = (float)neck_half
                   + (float)(half_w - neck_half) * sqrtf(1.0f - q * q);
        int hw = (int)(hw_f + 0.5f);
        if (hw < neck_half) hw = neck_half;
        for (int dx = -hw; dx <= hw; dx++) {
            int c = cx + dx;
            if (c < 0 || c >= cols) continue;
            hg->cavity[r][c] = 1;
        }
    }

    /* Carve neck rows (constant width). */
    for (int r = neck_top; r <= neck_bot; r++) {
        for (int dx = -neck_half; dx <= neck_half; dx++) {
            int c = cx + dx;
            if (c < 0 || c >= cols) continue;
            hg->cavity[r][c] = 1;
        }
    }

    /* Carve bottom bulb (rows neck_bot+1..bot) — mirror profile. */
    int bot_h = bot - neck_bot;
    if (bot_h < 1) bot_h = 1;
    for (int r = neck_bot + 1; r <= bot; r++) {
        float p = (bot_h > 1) ? (float)(r - neck_bot - 1) / (float)(bot_h - 1)
                              : 0.0f;
        float q = 1.0f - p;
        float hw_f = (float)neck_half
                   + (float)(half_w - neck_half) * sqrtf(1.0f - q * q);
        int hw = (int)(hw_f + 0.5f);
        if (hw < neck_half) hw = neck_half;
        for (int dx = -hw; dx <= hw; dx++) {
            int c = cx + dx;
            if (c < 0 || c >= cols) continue;
            hg->cavity[r][c] = 1;
        }
    }
}

/* ===================================================================== */
/* §6  sand — grid + CA tick + flip + transfer counter                    */
/* ===================================================================== */

/*
 * sand[row][col] = 0 (empty) or 1..LAYER_MAX (colour layer index).
 * mom[row][col]  = 0..MOMENTUM_MAX (vertical speed accumulator).
 */
typedef struct {
    uint8_t sand[SCREEN_MAX_H][SCREEN_MAX_W];
    uint8_t mom [SCREEN_MAX_H][SCREEN_MAX_W];
    int     gravity_dir;       /* +1 = down, -1 = up                     */
    int     transferred;       /* grains crossed the neck in current dir */
    int     total_grains;      /* total filled cells (for % progress)    */
    int     flip_count;        /* how many times the user has flipped    */
    int     frame;             /* monotonic frame counter for hash       */

    /* Settling detection: count grains that moved this tick (for the
     * idle_frames counter that drives auto-flip).  Reset at the start
     * of each tick. */
    int     moves_this_tick;
    int     idle_frames;       /* consecutive frames with zero moves     */
} Sand;

static Sand g_sand;

static void sand_clear(Sand *s)
{
    memset(s->sand, 0, sizeof s->sand);
    memset(s->mom,  0, sizeof s->mom);
    s->gravity_dir  = +1;
    s->transferred  = 0;
    s->total_grains = 0;
    s->flip_count   = 0;
    s->frame        = 0;
}

/*
 * sand_fill_layers — fill the BULB-currently-on-top with stratified
 * colour bands, dividing its vertical extent into `layers` bands.
 *
 * Pattern flag `dual_fill` makes us also fill the BULB-currently-on-
 * bottom (with reversed colour order) — the DUAL_FLOW pattern.
 *
 * `irregular` jitters each layer's vertical thickness with a hash
 * for organic geological strata.
 */
static void sand_fill_layers(Sand *s, const Hourglass *hg,
                              int cols, int rows_eff,
                              const PatternParams *pp, uint32_t seed)
{
    int layers = pp->layer_count;
    if (layers > LAYER_MAX) layers = LAYER_MAX;
    if (layers < 1)         layers = 1;

    /* Determine which bulb is "up" for refill (gravity-direction-aware:
     * +1 gravity → top bulb is "up"; −1 → bottom bulb is "up"). */
    int up_top, up_bot;     /* row range of the bulb to fill            */
    if (s->gravity_dir > 0) {
        up_top = hg->hg_top;
        up_bot = hg->hg_neck_top - 1;
    } else {
        up_top = hg->hg_neck_bot + 1;
        up_bot = hg->hg_bot;
    }
    int up_h = up_bot - up_top + 1;
    if (up_h < 1) return;

    /* Fill the bulb to HG_FILL_FRAC of its height, starting from the
     * far end (away from neck). */
    int fill_h = (int)((float)up_h * HG_FILL_FRAC);
    if (fill_h < 1)  fill_h = 1;
    if (fill_h > up_h) fill_h = up_h;

    /* For each layer band, compute its row range. */
    int filled_count = 0;
    for (int L = 0; L < layers; L++) {
        /* Layer L's vertical extent within the fill region. */
        float t0 = (float)L       / (float)layers;
        float t1 = (float)(L + 1) / (float)layers;
        if (pp->irregular) {
            /* Jitter the boundaries with a hash; keep monotonic. */
            float j0 = ((hash3(L,    0, seed) & 0xFFu) / 255.0f - 0.5f) * 0.10f;
            float j1 = ((hash3(L+1,  0, seed) & 0xFFu) / 255.0f - 0.5f) * 0.10f;
            t0 += j0;
            t1 += j1;
            if (t0 < 0) t0 = 0;
            if (t1 > 1) t1 = 1;
            if (t1 <= t0) t1 = t0 + 0.01f;
        }

        /* Map [t0, t1] into rows.  When gravity is +1 (down), layer 0
         * (oldest sand) is at the TOP (far from neck); when -1, layer
         * 0 is at the BOTTOM. */
        int row_a, row_b;
        if (s->gravity_dir > 0) {
            row_a = up_top + (int)(t0 * (float)fill_h);
            row_b = up_top + (int)(t1 * (float)fill_h);
        } else {
            row_a = up_bot - (int)(t1 * (float)fill_h);
            row_b = up_bot - (int)(t0 * (float)fill_h);
        }
        if (row_a > row_b) { int t = row_a; row_a = row_b; row_b = t; }

        for (int r = row_a; r <= row_b; r++) {
            if (r < 0 || r >= rows_eff) continue;
            for (int c = 0; c < cols; c++) {
                if (!hg->cavity[r][c]) continue;
                if (s->sand[r][c] != 0) continue;     /* don't overwrite */
                s->sand[r][c] = (uint8_t)(L + 1);     /* 1-based         */
                s->mom [r][c] = 0;
                filled_count++;
            }
        }
    }

    /* DUAL_FLOW: also fill the OTHER bulb to half its height, with
     * reversed layer order. */
    if (pp->dual_fill) {
        int dn_top, dn_bot;
        if (s->gravity_dir > 0) {
            dn_top = hg->hg_neck_bot + 1;
            dn_bot = hg->hg_bot;
        } else {
            dn_top = hg->hg_top;
            dn_bot = hg->hg_neck_top - 1;
        }
        int dn_h = dn_bot - dn_top + 1;
        if (dn_h > 0) {
            int dn_fill = dn_h / 2;
            if (dn_fill < 1) dn_fill = 1;
            for (int L = 0; L < layers; L++) {
                float t0 = (float)L       / (float)layers;
                float t1 = (float)(L + 1) / (float)layers;
                int row_a, row_b;
                if (s->gravity_dir > 0) {
                    row_a = dn_top + (int)(t0 * (float)dn_fill);
                    row_b = dn_top + (int)(t1 * (float)dn_fill);
                } else {
                    row_a = dn_bot - (int)(t1 * (float)dn_fill);
                    row_b = dn_bot - (int)(t0 * (float)dn_fill);
                }
                if (row_a > row_b) { int t = row_a; row_a = row_b; row_b = t; }
                for (int r = row_a; r <= row_b; r++) {
                    if (r < 0 || r >= rows_eff) continue;
                    for (int c = 0; c < cols; c++) {
                        if (!hg->cavity[r][c]) continue;
                        if (s->sand[r][c] != 0) continue;
                        /* Reverse layer order in the secondary bulb. */
                        s->sand[r][c] = (uint8_t)(layers - L);
                        s->mom [r][c] = 0;
                        filled_count++;
                    }
                }
            }
        }
    }

    s->total_grains = filled_count;
    s->transferred  = 0;
    s->frame        = 0;
}

/*
 * sand_tick — one CA step.  Scan order depends on gravity direction:
 *
 *   gravity_dir = +1 (down): scan rows BOTTOM → TOP
 *   gravity_dir = -1 (up):   scan rows TOP → BOTTOM
 *
 * For each filled cell, try to move WITH gravity (1 cell, plus
 * momentum-bonus cells if accumulated).  If blocked, try diagonals
 * (random L/R order via hash3).  If still blocked, settle (momentum
 * = 0).
 */
static void sand_tick(Sand *s, const Hourglass *hg, int cols, int rows_eff)
{
    int dir = s->gravity_dir;          /* +1 down, -1 up */
    int neck_mid_row = (hg->hg_neck_top + hg->hg_neck_bot) / 2;

    int r_start, r_end, r_step;
    if (dir > 0) {
        /* scan bottom up */
        r_start = rows_eff - 1; r_end = -1; r_step = -1;
    } else {
        r_start = 0; r_end = rows_eff; r_step = +1;
    }

    s->frame++;
    s->moves_this_tick = 0;

    for (int r = r_start; r != r_end; r += r_step) {
        if (r < 0 || r >= SCREEN_MAX_H) continue;
        for (int c = 0; c < cols && c < SCREEN_MAX_W; c++) {
            uint8_t cell = s->sand[r][c];
            if (cell == 0) continue;

            int dest_r = r + dir;
            if (dest_r < 0 || dest_r >= rows_eff) {
                s->mom[r][c] = 0;       /* would fall off — clamp */
                continue;
            }

            uint8_t mom_now = s->mom[r][c];

            /* 1. Try straight movement. */
            if (hg->cavity[dest_r][c] && s->sand[dest_r][c] == 0) {
                s->sand[dest_r][c] = cell;
                s->sand[r][c]      = 0;
                uint8_t new_mom = (mom_now < MOMENTUM_MAX) ? mom_now + 1
                                                            : MOMENTUM_MAX;
                s->mom[dest_r][c] = new_mom;
                s->mom[r][c]      = 0;
                s->moves_this_tick++;

                /* Track neck crossings for time-progress HUD. */
                if (dir > 0 && r <= neck_mid_row && dest_r > neck_mid_row)
                    s->transferred++;
                else if (dir < 0 && r >= neck_mid_row && dest_r < neck_mid_row)
                    s->transferred++;

                /* Momentum bonus: high-mom cells fall multiple cells. */
                if (new_mom >= 2) {
                    int extra = (new_mom >= 3) ? 2 : 1;
                    int rr = dest_r;
                    for (int k = 0; k < extra; k++) {
                        int nr = rr + dir;
                        if (nr < 0 || nr >= rows_eff)         break;
                        if (!hg->cavity[nr][c])                break;
                        if (s->sand[nr][c] != 0)               break;
                        s->sand[nr][c] = s->sand[rr][c];
                        s->sand[rr][c] = 0;
                        s->mom [nr][c] = s->mom[rr][c];
                        s->mom [rr][c] = 0;
                        s->moves_this_tick++;
                        if (dir > 0 && rr <= neck_mid_row && nr > neck_mid_row)
                            s->transferred++;
                        else if (dir < 0 && rr >= neck_mid_row && nr < neck_mid_row)
                            s->transferred++;
                        rr = nr;
                    }
                }
                continue;
            }

            /* 2. Try diagonals.  Random order per cell via hash. */
            int prefer_left = (hash3(r, c, s->frame) & 1u) ? 1 : 0;
            int dx_a = prefer_left ? -1 : +1;
            int dx_b = prefer_left ? +1 : -1;

            int moved = 0;
            for (int try = 0; try < 2 && !moved; try++) {
                int dx = (try == 0) ? dx_a : dx_b;
                int nc = c + dx;
                if (nc < 0 || nc >= cols)                continue;
                if (!hg->cavity[dest_r][nc])             continue;
                if (s->sand[dest_r][nc] != 0)            continue;
                /* Also require the same-row neighbour to be
                 * non-blocking (i.e. the grain can "see" the diagonal
                 * cell); otherwise sand would tunnel through walls. */
                if (!hg->cavity[r][nc] && s->sand[r][nc] != 0) continue;

                s->sand[dest_r][nc] = cell;
                s->sand[r][c]       = 0;
                s->mom[dest_r][nc]  = 0;     /* diagonals reset speed   */
                s->mom[r][c]        = 0;
                s->moves_this_tick++;
                /* Track neck crossings on diagonals too. */
                if (dir > 0 && r <= neck_mid_row && dest_r > neck_mid_row)
                    s->transferred++;
                else if (dir < 0 && r >= neck_mid_row && dest_r < neck_mid_row)
                    s->transferred++;
                moved = 1;
            }

            if (!moved) {
                /* Settled: momentum decays. */
                s->mom[r][c] = 0;
            }
        }
    }

    /* End-of-tick: update idle-frames counter for auto-flip detection. */
    if (s->moves_this_tick == 0) {
        s->idle_frames++;
    } else {
        s->idle_frames = 0;
    }
}

static void sand_flip(Sand *s)
{
    s->gravity_dir = -s->gravity_dir;
    s->transferred = 0;
    s->flip_count++;
    s->idle_frames = 0;
    /* Reset momentum after flip — avalanche starts from rest. */
    memset(s->mom, 0, sizeof s->mom);
}

/* ===================================================================== */
/* §7  scene — compose render passes                                      */
/* ===================================================================== */

typedef struct {
    bool       paused;
    int        cols, rows;
    uint32_t   seed;
    int        current_pattern;
    int        current_theme;
    bool       auto_flip;          /* auto-flip when fully drained?      */
} Scene;

static void scene_rebuild(Scene *s)
{
    int rows_eff = s->rows - 1;
    if (rows_eff < 8) rows_eff = 8;
    hourglass_build(&g_hg, s->cols, rows_eff);
    sand_clear(&g_sand);
    sand_fill_layers(&g_sand, &g_hg, s->cols, rows_eff,
                     &patterns[s->current_pattern], s->seed);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->cols            = cols;
    s->rows            = rows;
    s->seed            = (uint32_t)clock_ns();
    s->current_pattern = PAT_NORMAL;
    s->current_theme   = 0;
    s->auto_flip       = true;
    g_rng = s->seed ^ 0xBEEFu;
    scene_rebuild(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
    scene_rebuild(s);
}

static void scene_refill(Scene *s)
{
    s->seed = (uint32_t)clock_ns() ^ 0xA5A5A5A5u;
    g_rng   = s->seed ^ 0xBEEFu;
    /* Keep current gravity direction; just refill the up-bulb. */
    sand_clear(&g_sand);
    int rows_eff = s->rows - 1;
    sand_fill_layers(&g_sand, &g_hg, s->cols, rows_eff,
                     &patterns[s->current_pattern], s->seed);
}

static void scene_cycle_pattern(Scene *s, int dir)
{
    int idx = s->current_pattern + dir;
    while (idx < 0) idx += N_PATTERNS;
    s->current_pattern = idx % N_PATTERNS;
    scene_rebuild(s);
}

static void scene_cycle_theme(Scene *s, int dir)
{
    int idx = s->current_theme + dir;
    while (idx < 0) idx += N_THEMES;
    s->current_theme = idx % N_THEMES;
    theme_apply(s->current_theme);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    int rows_eff = s->rows - 1;
    sand_tick(&g_sand, &g_hg, s->cols, rows_eff);

    /*
     * Auto-flip — two trigger conditions:
     *
     *   (a) FULLY DRAINED: transferred reaches total_grains AND the sim
     *       has been settled for AUTO_FLIP_SETTLE frames.  This is the
     *       canonical "the hourglass has finished, flip it" trigger.
     *
     *   (b) STALL FALLBACK: settled for AUTO_FLIP_STALL frames (~3 sec)
     *       even though some grains haven't crossed.  Catches the rare
     *       case where grains are stuck in repose pockets above the
     *       neck and would never drain on their own.  Flip anyway —
     *       the avalanche unsticks them.
     */
    if (s->auto_flip && g_sand.total_grains > 0) {
        bool fully_drained = (g_sand.transferred >= g_sand.total_grains);
        bool short_settled = fully_drained
                          && g_sand.idle_frames >= AUTO_FLIP_SETTLE;
        bool long_stalled  = (g_sand.idle_frames >= AUTO_FLIP_STALL)
                          && (g_sand.transferred > 0);
        if (short_settled || long_stalled) {
            sand_flip(&g_sand);
        }
    }
    (void)dt;
}

/*
 * draw_frame — wooden frame around the hourglass.  Top + bottom caps
 * with `=` chars and corners; vertical rails along the side columns.
 */
static void draw_frame(const Scene *s, bool inverted)
{
    (void)inverted;
    const Hourglass *hg = &g_hg;
    attron(COLOR_PAIR(PAIR_FRAME) | A_BOLD);

    int top_row = hg->frame_top_row;
    int bot_row = hg->frame_bot_row;
    int lc      = hg->frame_left_col;
    int rc      = hg->frame_right_col;

    /* Top + bottom cap rows. */
    for (int c = lc; c <= rc; c++) {
        if (top_row >= 0)
            mvaddch(top_row, c, (chtype)(unsigned char)FRAME_CAP_GLYPH);
        if (bot_row < s->rows - 1)
            mvaddch(bot_row, c, (chtype)(unsigned char)FRAME_CAP_GLYPH);
    }
    /* Corners. */
    if (top_row >= 0) {
        mvaddch(top_row, lc, (chtype)(unsigned char)FRAME_CORNER);
        mvaddch(top_row, rc, (chtype)(unsigned char)FRAME_CORNER);
    }
    if (bot_row < s->rows - 1) {
        mvaddch(bot_row, lc, (chtype)(unsigned char)FRAME_CORNER);
        mvaddch(bot_row, rc, (chtype)(unsigned char)FRAME_CORNER);
    }

    /* Side rails — the columns directly outside the bulbs at top/bottom
     * caps' Y range.  Just two vertical bars on the outside corners. */
    for (int r = top_row + 1; r < bot_row; r++) {
        if (r >= 0 && r < s->rows - 1) {
            mvaddch(r, lc, (chtype)(unsigned char)FRAME_RAIL_GLYPH);
            mvaddch(r, rc, (chtype)(unsigned char)FRAME_RAIL_GLYPH);
        }
    }

    attroff(COLOR_PAIR(PAIR_FRAME) | A_BOLD);
}

/*
 * draw_walls — hourglass GLASS walls.  Walk the cavity boundary;
 * any cell that's NOT inside the cavity but IS adjacent to one
 * gets a wall glyph based on slope direction.
 */
static void draw_walls(const Scene *s, bool inverted)
{
    (void)inverted;
    const Hourglass *hg = &g_hg;
    int rows_eff = s->rows - 1;
    int cols     = s->cols;

    attron(COLOR_PAIR(PAIR_WALL));
    for (int r = hg->hg_top; r <= hg->hg_bot; r++) {
        if (r < 0 || r >= rows_eff) continue;
        /* Find leftmost and rightmost cavity cells in this row. */
        int left = -1, right = -1;
        for (int c = 0; c < cols; c++) {
            if (hg->cavity[r][c]) {
                if (left < 0) left = c;
                right = c;
            }
        }
        if (left < 0) continue;

        /* Wall just outside the cavity boundary. */
        int wl = left  - 1;
        int wr = right + 1;
        if (wl >= 0) {
            /* Choose slope glyph by neighbouring rows' boundaries. */
            char g = WALL_GLYPH_V;
            if (r > 0 && r < rows_eff - 1) {
                int left_above = -1, left_below = -1;
                for (int c = 0; c < cols; c++) {
                    if (r-1 >= 0          && hg->cavity[r-1][c] && left_above < 0) left_above = c;
                    if (r+1 < rows_eff    && hg->cavity[r+1][c] && left_below < 0) left_below = c;
                }
                if (left_above < left)       g = WALL_GLYPH_DR;   /* widening down */
                else if (left_above > left)  g = WALL_GLYPH_DL;
            }
            mvaddch(r, wl, (chtype)(unsigned char)g);
        }
        if (wr < cols) {
            char g = WALL_GLYPH_V;
            if (r > 0 && r < rows_eff - 1) {
                int right_above = -1, right_below = -1;
                for (int c = cols - 1; c >= 0; c--) {
                    if (r-1 >= 0          && hg->cavity[r-1][c] && right_above < 0) right_above = c;
                    if (r+1 < rows_eff    && hg->cavity[r+1][c] && right_below < 0) right_below = c;
                }
                if (right_above > right)       g = WALL_GLYPH_DL;  /* widening down */
                else if (right_above < right)  g = WALL_GLYPH_DR;
            }
            mvaddch(r, wr, (chtype)(unsigned char)g);
        }
    }
    attroff(COLOR_PAIR(PAIR_WALL));
}

/*
 * draw_sand — render every filled cell.  Glyph chosen by momentum:
 *   momentum 0 (resting): dense glyph from SAND_AT_REST
 *   momentum 1-2 (slow):  SAND_MOVING
 *   momentum 3+ (fast):   SAND_FAST
 * Colour pair chosen by layer index.
 */
static void draw_sand(const Scene *s, bool inverted)
{
    int rows_eff = s->rows - 1;
    int cols     = s->cols;
    int last_pair = -1;
    attr_t last_attr = 0;

    for (int r = 0; r < rows_eff && r < SCREEN_MAX_H; r++) {
        for (int c = 0; c < cols && c < SCREEN_MAX_W; c++) {
            uint8_t layer = g_sand.sand[r][c];
            if (layer == 0) continue;
            uint8_t mom = g_sand.mom[r][c];

            char g;
            if (mom == 0)
                g = SAND_AT_REST[(unsigned)(r + c) % (sizeof SAND_AT_REST - 1)];
            else if (mom <= 2)
                g = SAND_MOVING[(unsigned)(r + c) % (sizeof SAND_MOVING - 1)];
            else
                g = SAND_FAST[(unsigned)(r + c) % (sizeof SAND_FAST - 1)];

            int pair = PAIR_LAYER_BASE + ((layer - 1) % 16);
            attr_t attr;
            if (inverted) {
                attr = A_NORMAL;
            } else {
                attr = (mom == 0) ? A_BOLD
                     : (mom >= 3) ? A_DIM
                     :              A_NORMAL;
            }

            if (pair != last_pair || attr != last_attr) {
                if (last_pair >= 0)
                    attroff(COLOR_PAIR(last_pair) | last_attr);
                attron(COLOR_PAIR(pair) | attr);
                last_pair = pair;
                last_attr = attr;
            }
            mvaddch(r, c, (chtype)(unsigned char)g);
        }
    }
    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

static void scene_render(const Scene *s)
{
    bool inverted = themes[s->current_theme].inverted;
    int rows_eff = s->rows - 1;

    if (inverted) {
        attron(COLOR_PAIR(PAIR_PAPER));
        for (int r = 0; r < rows_eff; r++)
            for (int c = 0; c < s->cols; c++)
                mvaddch(r, c, ' ');
        attroff(COLOR_PAIR(PAIR_PAPER));
    }

    draw_frame(s, inverted);
    draw_walls(s, inverted);
    draw_sand (s, inverted);
}

/* ===================================================================== */
/* §8  screen + app                                                       */
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

    int total = g_sand.total_grains;
    if (total < 1) total = 1;
    int pct = 100 * g_sand.transferred / total;
    if (pct > 100) pct = 100;

    char buf[220];
    snprintf(buf, sizeof buf,
             " SANDS OF TIME   %s   pat:%s   theme:%s   "
             "Time:%3d%%   Flips:%2d   grav:%s   auto:%s   "
             "%5.1f fps  %3d Hz   "
             "spc:flip  a:auto  r:refill  n/N:pat  t/T:theme  p:pause  q:quit ",
             s->paused ? "PAUSED " : "FLOWING",
             patterns[s->current_pattern].name,
             themes[s->current_theme].name,
             pct, g_sand.flip_count,
             g_sand.gravity_dir > 0 ? "DOWN" : "UP  ",
             s->auto_flip ? "ON " : "OFF",
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
    case 'p': case 'P': s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_refill(s);                              break;
    case ' ': case 'f': case 'F':
        sand_flip(&g_sand);
        break;
    case 'a': case 'A':
        s->auto_flip = !s->auto_flip;
        break;

    case 'n':           scene_cycle_pattern(s, +1);                   break;
    case 'N':           scene_cycle_pattern(s, -1);                   break;

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
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
