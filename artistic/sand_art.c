/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sand_art.c — an hourglass you can watch and flip: coloured sand falls
 * through a curved-bulb glass, driven by a falling-sand cellular automaton.
 * Sand-pile physics: Toffoli & Margolus (1987); Powder Toy (2008+) for the
 * momentum trick; Beverloo et al. (1961) for why the narrow neck meters flow.
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

/* ── §1 config — tunables: themes, patterns, hourglass + frame geometry ── */

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

    /* Biggest terminal we size our fixed grids for. */
    SCREEN_MAX_W      = 200,
    SCREEN_MAX_H      = 80,

    /* Most distinct sand colours one fill can use (RAINBOW uses all 16). */
    LAYER_MAX         = 16,

    /* Top fall speed a grain can build up (0 = resting, 4 = free-fall). */
    MOMENTUM_MAX      = 4,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Default hourglass shape. The bulbs are wide, the neck is tiny — that
 * narrow waist is what makes sand trickle through slowly, like a real timer. */
#define HG_FULL_FRAC     0.16f     /* bulb half-width, as a fraction of screen width */
#define HG_NECK_HALF     1.0f      /* neck half-width in cells; 1 means a 3-cell-wide waist */
#define HG_NECK_ROWS     2         /* how many rows stay at the narrow neck width */
#define HG_HEIGHT_FRAC   0.86f     /* glass height, as a fraction of usable rows */
#define HG_FRAME_PAD     1         /* spare rows above + below for the wooden frame */

/* How full the upper bulb starts — 0.92 leaves a little headroom. */
#define HG_FILL_FRAC     0.92f

/* When the demo flips itself, like turning a real hourglass over once it
 * finishes. SETTLE is the little pause after the last grain falls, so you
 * see the empty bulb before it turns. STALL is a safety net: if a few grains
 * get stuck near the neck and never drain, flip anyway after a few seconds —
 * the avalanche shakes them loose. Both counted in frames. */
#define AUTO_FLIP_SETTLE     30     /* pause after the bulb empties */
#define AUTO_FLIP_STALL     180     /* give up waiting and flip (~3 sec) */

/* The four sand-art presets you cycle with n/N. Each one changes both the
 * colours/layering of the sand AND the glass shape, so they look clearly
 * different, not just recoloured. Used as an index into patterns[] below. */
typedef enum {
    PAT_NORMAL     = 0,
    PAT_RAINBOW    = 1,
    PAT_GEOLOGICAL = 2,
    PAT_DUAL_FLOW  = 3,
    N_PATTERNS     = 4,
} Pattern;

/* The full recipe for one preset: how the sand is laid out, plus how the
 * glass is shaped. One row of patterns[] per Pattern value. */
typedef struct {
    const char *name;
    int         layer_count;       /* number of colour bands, 1..LAYER_MAX (8 = coarse, 16 = fine) */
    bool        irregular;         /* wobble the band thicknesses for a geological look? */
    bool        dual_fill;         /* start sand in BOTH bulbs (so it flows forever)? */
    /* The glass shape for this preset — what makes each silhouette distinct. */
    float       height_frac;       /* glass height, as a fraction of usable rows */
    float       full_frac;         /* bulb half-width, as a fraction of screen width */
    float       neck_half;         /* neck half-width in cells */
} PatternParams;

static const PatternParams patterns[N_PATTERNS] = {
    /* name           layers irreg  dual    height          full          neck */
    /* classic slim, tall glass */
    { "NORMAL    ",    8,    false, false, HG_HEIGHT_FRAC, HG_FULL_FRAC, HG_NECK_HALF },
    /* short & wide — shows off all 16 colour bands */
    { "RAINBOW   ",   16,    false, false, 0.70f,          0.24f,        1.0f         },
    /* tall, narrow tower — deep geological strata */
    { "GEOLOGICAL",    8,    true,  false, 0.94f,          0.12f,        1.0f         },
    /* medium with a fat neck — both bulbs drain fast */
    { "DUAL_FLOW ",    8,    false, true,  0.80f,          0.20f,        2.0f         },
};

/* One colour scheme for the whole scene. Each layer gets its own sand colour
 * so neighbouring bands contrast and you can actually see the stripes; the
 * frame and glass walls get their own accent colours too. `inverted` is the
 * odd one out (the NEGATIVE theme): dark sand on a white-paper background.
 * Colours are all from the bright half of the palette so they stay readable. */
typedef struct {
    const char *name;
    short       sand[16];         /* colour per layer, indexed by (layer - 1) */
    short       frame;            /* the wooden frame */
    short       wall;             /* the glass walls of the hourglass */
    bool        inverted;         /* dark sand on white paper instead of light-on-dark? */
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

/* The character drawn for a grain depends on how fast it's moving: settled
 * sand looks solid, falling sand looks wispy. The pairs add a little variety. */
static const char SAND_AT_REST[]   = "@#";
static const char SAND_MOVING[]    = "*+";
static const char SAND_FAST[]      = ".,";

#define WALL_GLYPH_V       '|'
#define WALL_GLYPH_DR      '\\'    /* slope going down-right */
#define WALL_GLYPH_DL      '/'     /* slope going down-left  */

#define FRAME_CAP_GLYPH    '='
#define FRAME_RAIL_GLYPH   '|'
#define FRAME_CORNER       '+'

/* ── §2 performance — monotonic clock + sleep (frame timing lives in §9) ── */

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

/* ── §3 color — load a theme's palette into ncurses colour pairs ── */

/* Push one theme's colours into the ncurses pairs the renderer reads.
 * Called at startup and again each time t/T cycles the theme. */
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

/* ── §4 logic — small random-number helpers (no shared state, no I/O) ── */

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

/* Scramble three numbers into one. Used to pick left-or-right for a sliding
 * grain: feeding it (row, col, frame) gives a per-grain coin flip that's the
 * same every time, so a run replays identically. */
static inline uint32_t hash3(int a, int b, int c)
{
    uint32_t h = (uint32_t)a * 374761393u
               + (uint32_t)b * 668265263u
               + (uint32_t)c * 2147483647u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}

/* ── §5 simulation — build the hourglass shape (the empty space sand fills) ── */

/* The fixed shape of the glass: a map of which cells are inside the cavity,
 * plus the key coordinates (where the bulbs and neck sit). The glass never
 * moves during a run — only the sand does — so this is built once and then
 * just read. The rounded bulbs and pinched neck are the whole point: that
 * narrow waist is what meters the flow and makes it act like a timer. */
typedef struct {
    /* For each cell: 1 = sand can live here (inside the glass), 0 = wall or
     * outside. Rebuilt on resize or reset, never touched while sand moves. */
    uint8_t cavity[SCREEN_MAX_H][SCREEN_MAX_W];

    /* Where the glass sits on screen. */
    int hg_top;        /* top edge of the upper bulb */
    int hg_bot;        /* bottom edge of the lower bulb (inclusive) */
    int hg_neck_top;   /* first row of the narrow neck */
    int hg_neck_bot;   /* last row of the narrow neck (inclusive) */
    int hg_cx;         /* centre column the glass is symmetric about */
    int hg_full_half;  /* half-width at the widest part of a bulb, in cells */
    int hg_neck_half;  /* half-width at the neck, in cells */

    /* Where the wooden frame's top/bottom bars and side rails go. */
    int frame_top_row;
    int frame_bot_row;
    int frame_left_col;
    int frame_right_col;
} Hourglass;

/* How wide the bulb is `d` rows away from the neck. It starts pinched at the
 * neck and swells out toward the cap along a curve (a quarter of an ellipse),
 * which gives the rounded hourglass look instead of straight funnel walls. */
static int bulb_half_width(int d, int bulb_h, int neck_half, int full_half)
{
    float p = (bulb_h > 1) ? (float)d / (float)(bulb_h - 1) : 0.0f;
    float q = 1.0f - p;
    float hw_f = (float)neck_half + (float)(full_half - neck_half) * sqrtf(1.0f - q * q);
    int hw = (int)(hw_f + 0.5f);
    if (hw < neck_half) hw = neck_half;
    return hw;
}

/* Open up one row of the glass: mark cells within hw of the centre as inside. */
static void carve_row(Hourglass *hg, int r, int cx, int hw, int cols)
{
    for (int dx = -hw; dx <= hw; dx++) {
        int c = cx + dx;
        if (c < 0 || c >= cols) continue;
        hg->cavity[r][c] = 1;
    }
}

/* Work out where every part of the glass and frame goes, given the window size
 * and the preset's shape fractions. Lots of clamping so it always fits onscreen. */
static void compute_glass_dims(Hourglass *hg, int cols, int rows_eff,
                               const PatternParams *pp)
{
    int hg_height = (int)((float)rows_eff * pp->height_frac);
    if (hg_height < 12) hg_height = (rows_eff > 12) ? 12 : rows_eff - 4;
    if (hg_height < 8)  hg_height = 8;
    int half_w = (int)((float)cols * pp->full_frac);
    if (half_w < 6)  half_w = 6;
    if (half_w > cols / 2 - 3) half_w = cols / 2 - 3;
    if (half_w < 3)  half_w = 3;
    int neck_half = (int)pp->neck_half;

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

    /* The frame sits just outside the glass on all four sides. */
    hg->frame_top_row   = top - 1;
    hg->frame_bot_row   = bot + 1;
    hg->frame_left_col  = cx - half_w - 1;
    hg->frame_right_col = cx + half_w + 1;
    if (hg->frame_top_row < 0)              hg->frame_top_row = 0;
    if (hg->frame_bot_row >= rows_eff)      hg->frame_bot_row = rows_eff - 1;
    if (hg->frame_left_col < 0)             hg->frame_left_col = 0;
    if (hg->frame_right_col >= cols)        hg->frame_right_col = cols - 1;
}

static void hourglass_build(Hourglass *hg, int cols, int rows_eff,
                            const PatternParams *pp)
{
    for (int r = 0; r < rows_eff && r < SCREEN_MAX_H; r++)
        for (int c = 0; c < cols && c < SCREEN_MAX_W; c++)
            hg->cavity[r][c] = 0;

    compute_glass_dims(hg, cols, rows_eff, pp);

    int top = hg->hg_top, bot = hg->hg_bot;
    int neck_top = hg->hg_neck_top, neck_bot = hg->hg_neck_bot;
    int cx = hg->hg_cx, half_w = hg->hg_full_half, neck_half = hg->hg_neck_half;

    /* Carve the upper bulb, then the straight neck, then the mirrored lower
     * bulb — one row at a time, each row's width from bulb_half_width. */
    int top_h = neck_top - top;     if (top_h < 1) top_h = 1;
    int bot_h = bot - neck_bot;     if (bot_h < 1) bot_h = 1;

    for (int r = top; r < neck_top; r++)
        carve_row(hg, r, cx, bulb_half_width(neck_top - 1 - r, top_h, neck_half, half_w), cols);

    for (int r = neck_top; r <= neck_bot; r++)
        carve_row(hg, r, cx, neck_half, cols);

    for (int r = neck_bot + 1; r <= bot; r++)
        carve_row(hg, r, cx, bulb_half_width(r - neck_bot - 1, bot_h, neck_half, half_w), cols);
}

/* ── §6 simulation — move the sand each tick, flip gravity, count progress ── */

/* All the sand and the bookkeeping that goes with it. Two grids run in
 * lockstep: `sand` says what colour grain is in each cell, `mom` says how fast
 * it's falling. The speed grid is the trick that makes the neck stream look
 * like it's accelerating — a fast grain can drop several cells in one tick
 * instead of just one. Letting grains slide sideways gives sand piles their
 * natural slope for free. */
typedef struct {
    uint8_t sand[SCREEN_MAX_H][SCREEN_MAX_W];   /* 0 = empty, 1..LAYER_MAX = colour */
    uint8_t mom [SCREEN_MAX_H][SCREEN_MAX_W];   /* fall speed, 0..MOMENTUM_MAX */
    int     gravity_dir;       /* which way is "down": +1 = down, -1 = up */
    int     transferred;       /* grains that have fallen through the neck so far */
    int     total_grains;      /* how many grains there are (the 100% mark) */
    int     flip_count;        /* how many times the glass has been turned over */
    int     frame;             /* tick counter, fed to hash3 for the L/R coin flip */

    /* Used to notice when everything has stopped, which triggers an auto-flip. */
    int     moves_this_tick;   /* grains that moved this tick (reset each tick) */
    int     idle_frames;       /* how many ticks in a row nothing has moved */
} Sand;

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

/* Paint a horizontal band of rows with one colour, only touching empty
 * in-glass cells. Returns how many grains it placed. */
static int fill_layer_band(Sand *s, const Hourglass *hg, int row_a, int row_b,
                           uint8_t color, int cols, int rows_eff)
{
    int n = 0;
    for (int r = row_a; r <= row_b; r++) {
        if (r < 0 || r >= rows_eff) continue;
        for (int c = 0; c < cols; c++) {
            if (!hg->cavity[r][c]) continue;
            if (s->sand[r][c] != 0) continue;     /* leave already-filled cells alone */
            s->sand[r][c] = color;
            s->mom [r][c] = 0;
            n++;
        }
    }
    return n;
}

/* Fill the upper bulb with coloured stripes, splitting its height into `layers`
 * bands. The DUAL_FLOW preset also seeds the lower bulb (colours reversed) so
 * sand is always flowing; the geological preset jitters each stripe's thickness
 * so the bands look natural rather than ruler-straight. */
static void sand_fill_layers(Sand *s, const Hourglass *hg,
                              int cols, int rows_eff,
                              const PatternParams *pp, uint32_t seed)
{
    int layers = pp->layer_count;
    if (layers > LAYER_MAX) layers = LAYER_MAX;
    if (layers < 1)         layers = 1;

    /* Which bulb is currently the upper one depends on gravity direction. */
    int up_top, up_bot;     /* the row range we'll fill */
    if (s->gravity_dir > 0) {
        up_top = hg->hg_top;
        up_bot = hg->hg_neck_top - 1;
    } else {
        up_top = hg->hg_neck_bot + 1;
        up_bot = hg->hg_bot;
    }
    int up_h = up_bot - up_top + 1;
    if (up_h < 1) return;

    /* Only fill most of the bulb (HG_FILL_FRAC), starting at the far end. */
    int fill_h = (int)((float)up_h * HG_FILL_FRAC);
    if (fill_h < 1)  fill_h = 1;
    if (fill_h > up_h) fill_h = up_h;

    int filled_count = 0;
    for (int L = 0; L < layers; L++) {
        /* This stripe occupies the slice [t0, t1] of the fill region. */
        float t0 = (float)L       / (float)layers;
        float t1 = (float)(L + 1) / (float)layers;
        if (pp->irregular) {
            /* Nudge the stripe edges a little, but keep them in order. */
            float j0 = ((hash3(L,    0, seed) & 0xFFu) / 255.0f - 0.5f) * 0.10f;
            float j1 = ((hash3(L+1,  0, seed) & 0xFFu) / 255.0f - 0.5f) * 0.10f;
            t0 += j0;
            t1 += j1;
            if (t0 < 0) t0 = 0;
            if (t1 > 1) t1 = 1;
            if (t1 <= t0) t1 = t0 + 0.01f;
        }

        /* Turn that slice into actual rows. Stripe 0 (the oldest sand) sits
         * at the far end of the bulb, away from the neck. */
        int row_a, row_b;
        if (s->gravity_dir > 0) {
            row_a = up_top + (int)(t0 * (float)fill_h);
            row_b = up_top + (int)(t1 * (float)fill_h);
        } else {
            row_a = up_bot - (int)(t1 * (float)fill_h);
            row_b = up_bot - (int)(t0 * (float)fill_h);
        }
        if (row_a > row_b) { int t = row_a; row_a = row_b; row_b = t; }

        /* Colours are 1-based (0 means empty), so stripe L gets colour L+1. */
        filled_count += fill_layer_band(s, hg, row_a, row_b,
                                        (uint8_t)(L + 1), cols, rows_eff);
    }

    /* DUAL_FLOW preset: seed the other bulb too, half-full, colours reversed. */
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
                /* Colours run the opposite way here. */
                filled_count += fill_layer_band(s, hg, row_a, row_b,
                                                (uint8_t)(layers - L), cols, rows_eff);
            }
        }
    }

    s->total_grains = filled_count;
    s->transferred  = 0;
    s->frame        = 0;
}

/* If this move carried a grain across the middle of the neck, count it — that
 * tally is what the Time% readout is based on. */
static void count_neck_crossing(Sand *s, int from_r, int to_r, int dir,
                                int neck_mid)
{
    bool crossed = (dir > 0) ? (from_r <= neck_mid && to_r > neck_mid)
                             : (from_r >= neck_mid && to_r < neck_mid);
    if (crossed) s->transferred++;
}

/* Let a fast grain fall a couple of extra cells in the same tick. This is what
 * makes the stream through the neck look like it's speeding up, instead of every
 * grain crawling exactly one cell per tick. Stops as soon as it hits a wall or
 * another grain. */
static void grain_bonus_drops(Sand *s, const Hourglass *hg, int from_r, int c,
                              uint8_t mom, int dir, int neck_mid, int rows_eff)
{
    if (mom < 2) return;
    int extra = (mom >= 3) ? 2 : 1;
    int rr = from_r;
    for (int k = 0; k < extra; k++) {
        int nr = rr + dir;
        if (nr < 0 || nr >= rows_eff)  break;
        if (!hg->cavity[nr][c])         break;
        if (s->sand[nr][c] != 0)        break;
        s->sand[nr][c] = s->sand[rr][c];
        s->sand[rr][c] = 0;
        s->mom [nr][c] = s->mom[rr][c];
        s->mom [rr][c] = 0;
        s->moves_this_tick++;
        count_neck_crossing(s, rr, nr, dir, neck_mid);
        rr = nr;
    }
}

/* Try to drop the grain straight down by one cell. If it works it picks up a
 * little speed, and a fast grain then takes its bonus drops. Returns whether
 * it moved. */
static bool grain_fall_straight(Sand *s, const Hourglass *hg, int r, int c,
                                int dir, int neck_mid, int rows_eff)
{
    int dest_r = r + dir;
    if (!hg->cavity[dest_r][c] || s->sand[dest_r][c] != 0) return false;

    uint8_t mom_now = s->mom[r][c];
    s->sand[dest_r][c] = s->sand[r][c];
    s->sand[r][c]      = 0;
    uint8_t new_mom = (mom_now < MOMENTUM_MAX) ? mom_now + 1 : MOMENTUM_MAX;
    s->mom[dest_r][c] = new_mom;
    s->mom[r][c]      = 0;
    s->moves_this_tick++;
    count_neck_crossing(s, r, dest_r, dir, neck_mid);

    grain_bonus_drops(s, hg, dest_r, c, new_mom, dir, neck_mid, rows_eff);
    return true;
}

/* If it can't fall straight, try sliding down to one side. Which side it checks
 * first is a coin flip per grain, so piles spread evenly and settle at a natural
 * slope. Sliding kills its speed. Returns whether it moved. */
static bool grain_slide_diagonal(Sand *s, const Hourglass *hg, int r, int c,
                                 int dir, int neck_mid, int cols)
{
    int dest_r = r + dir;
    int prefer_left = (hash3(r, c, s->frame) & 1u) ? 1 : 0;
    int dx_a = prefer_left ? -1 : +1;
    int dx_b = prefer_left ? +1 : -1;

    for (int try = 0; try < 2; try++) {
        int dx = (try == 0) ? dx_a : dx_b;
        int nc = c + dx;
        if (nc < 0 || nc >= cols)                continue;
        if (!hg->cavity[dest_r][nc])             continue;
        if (s->sand[dest_r][nc] != 0)            continue;
        /* The grain has to be able to "reach" the diagonal cell — the cell
         * beside it must be open. Otherwise sand would leak through walls. */
        if (!hg->cavity[r][nc] && s->sand[r][nc] != 0) continue;

        s->sand[dest_r][nc] = s->sand[r][c];
        s->sand[r][c]       = 0;
        s->mom[dest_r][nc]  = 0;     /* a sideways slide resets the grain's speed */
        s->mom[r][c]        = 0;
        s->moves_this_tick++;
        count_neck_crossing(s, r, dest_r, dir, neck_mid);
        return true;
    }
    return false;
}

/* Move one grain for one tick: fall straight if you can, else slide to a side,
 * else just sit there. */
static void sand_move_grain(Sand *s, const Hourglass *hg, int r, int c,
                            int dir, int neck_mid, int cols, int rows_eff)
{
    if (s->sand[r][c] == 0) return;            /* nothing here to move */

    int dest_r = r + dir;
    if (dest_r < 0 || dest_r >= rows_eff) {    /* off the edge of the screen */
        s->mom[r][c] = 0;
        return;
    }

    if (grain_fall_straight(s, hg, r, c, dir, neck_mid, rows_eff)) return;
    if (grain_slide_diagonal(s, hg, r, c, dir, neck_mid, cols))    return;

    s->mom[r][c] = 0;                           /* stuck, so it comes to rest */
}

/* One step of the whole simulation: move every grain once. We walk the rows
 * against gravity (bottom-up when falling down) so a grain that just dropped
 * isn't picked up and moved again in the same tick. Afterwards we note whether
 * anything moved, which is how the auto-flip knows the glass has emptied. */
static void sand_tick(Sand *s, const Hourglass *hg, int cols, int rows_eff)
{
    int dir = s->gravity_dir;
    int neck_mid_row = (hg->hg_neck_top + hg->hg_neck_bot) / 2;

    int r_start, r_end, r_step;
    if (dir > 0) { r_start = rows_eff - 1; r_end = -1;       r_step = -1; }
    else         { r_start = 0;            r_end = rows_eff; r_step = +1; }

    s->frame++;
    s->moves_this_tick = 0;

    for (int r = r_start; r != r_end; r += r_step) {
        if (r < 0 || r >= SCREEN_MAX_H) continue;
        for (int c = 0; c < cols && c < SCREEN_MAX_W; c++)
            sand_move_grain(s, hg, r, c, dir, neck_mid_row, cols, rows_eff);
    }

    /* No grain moved this tick → the sand has settled. */
    if (s->moves_this_tick == 0) s->idle_frames++;
    else                         s->idle_frames = 0;
}

/* Turn the glass over: swap which way is down and let the sand avalanche back. */
static void sand_flip(Sand *s)
{
    s->gravity_dir = -s->gravity_dir;
    s->transferred = 0;
    s->flip_count++;
    s->idle_frames = 0;
    memset(s->mom, 0, sizeof s->mom);   /* everything starts from rest again */
}

/* ── §7 simulation — scene control: build, refill, cycle presets, step ── */

/* Everything that makes up one running hourglass, bundled together: the glass
 * shape, the sand, the current preset/theme, the terminal size, and run state.
 * The low-level helpers take just the piece they need; the scene_* functions
 * here are the ones that coordinate the whole thing. */
typedef struct {
    Hourglass  hg;                 /* the glass shape */
    Sand       sand;               /* the grains and their state */
    int        current_pattern;    /* which preset (cycled with n/N) */
    int        current_theme;      /* which colour scheme (cycled with t/T) */
    bool       auto_flip;          /* turn the glass over by itself when it empties? */
    int        cols, rows;         /* terminal size in cells */
    uint32_t   seed;               /* random seed for this run's fills */
    bool       paused;             /* is the simulation frozen? */
} Scene;

static void scene_rebuild(Scene *s)
{
    int rows_eff = s->rows - 1;
    if (rows_eff < 8) rows_eff = 8;
    hourglass_build(&s->hg, s->cols, rows_eff, &patterns[s->current_pattern]);
    sand_clear(&s->sand);
    sand_fill_layers(&s->sand, &s->hg, s->cols, rows_eff,
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
    /* Keep gravity pointing the same way; just top the upper bulb back up. */
    sand_clear(&s->sand);
    int rows_eff = s->rows - 1;
    sand_fill_layers(&s->sand, &s->hg, s->cols, rows_eff,
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
    sand_tick(&s->sand, &s->hg, s->cols, rows_eff);

    /* Decide whether to turn the glass over on its own. Two ways that happens:
     * either every grain has fallen through and the sand has sat still for a
     * moment (the normal "it's done" case), or a few grains got stuck near the
     * neck and it's been frozen for a few seconds — flip anyway, the tumble
     * frees them. */
    if (s->auto_flip && s->sand.total_grains > 0) {
        bool fully_drained = (s->sand.transferred >= s->sand.total_grains);
        bool short_settled = fully_drained
                          && s->sand.idle_frames >= AUTO_FLIP_SETTLE;
        bool long_stalled  = (s->sand.idle_frames >= AUTO_FLIP_STALL)
                          && (s->sand.transferred > 0);
        if (short_settled || long_stalled) {
            sand_flip(&s->sand);
        }
    }
    (void)dt;
}

/* ── §8 render — draw the frame, glass, sand, and HUD (reads, never writes) ── */

/* Draw the wooden frame: a bar across the top and bottom with corner pieces,
 * and two vertical rails down the sides. */
static void draw_frame(const Scene *s, bool inverted)
{
    (void)inverted;
    const Hourglass *hg = &s->hg;
    attron(COLOR_PAIR(PAIR_FRAME) | A_BOLD);

    int top_row = hg->frame_top_row;
    int bot_row = hg->frame_bot_row;
    int lc      = hg->frame_left_col;
    int rc      = hg->frame_right_col;

    /* The top and bottom bars. */
    for (int c = lc; c <= rc; c++) {
        if (top_row >= 0)
            mvaddch(top_row, c, (chtype)(unsigned char)FRAME_CAP_GLYPH);
        if (bot_row < s->rows - 1)
            mvaddch(bot_row, c, (chtype)(unsigned char)FRAME_CAP_GLYPH);
    }
    /* The four corner pieces. */
    if (top_row >= 0) {
        mvaddch(top_row, lc, (chtype)(unsigned char)FRAME_CORNER);
        mvaddch(top_row, rc, (chtype)(unsigned char)FRAME_CORNER);
    }
    if (bot_row < s->rows - 1) {
        mvaddch(bot_row, lc, (chtype)(unsigned char)FRAME_CORNER);
        mvaddch(bot_row, rc, (chtype)(unsigned char)FRAME_CORNER);
    }

    /* The two vertical rails down the left and right sides. */
    for (int r = top_row + 1; r < bot_row; r++) {
        if (r >= 0 && r < s->rows - 1) {
            mvaddch(r, lc, (chtype)(unsigned char)FRAME_RAIL_GLYPH);
            mvaddch(r, rc, (chtype)(unsigned char)FRAME_RAIL_GLYPH);
        }
    }

    attroff(COLOR_PAIR(PAIR_FRAME) | A_BOLD);
}

/* Draw the glass walls. For each row we find the leftmost and rightmost cells
 * the sand can reach, then draw a wall just outside them. The character is
 * picked to follow the curve: a slash where the wall slopes, a bar where it's
 * straight. */
static void draw_walls(const Scene *s, bool inverted)
{
    (void)inverted;
    const Hourglass *hg = &s->hg;
    int rows_eff = s->rows - 1;
    int cols     = s->cols;

    attron(COLOR_PAIR(PAIR_WALL));
    for (int r = hg->hg_top; r <= hg->hg_bot; r++) {
        if (r < 0 || r >= rows_eff) continue;
        /* Leftmost and rightmost open cells in this row. */
        int left = -1, right = -1;
        for (int c = 0; c < cols; c++) {
            if (hg->cavity[r][c]) {
                if (left < 0) left = c;
                right = c;
            }
        }
        if (left < 0) continue;

        /* The wall sits one cell outside the open span on each side. */
        int wl = left  - 1;
        int wr = right + 1;
        if (wl >= 0) {
            /* Compare with the rows above/below to see if the wall is sloping. */
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

/* Draw every grain. The character shows how fast it's moving (solid at rest,
 * wispier the faster it falls) and the colour comes from which stripe it's in.
 * Colour/attribute changes are batched so we don't reset ncurses state per cell. */
static void draw_sand(const Scene *s, bool inverted)
{
    int rows_eff = s->rows - 1;
    int cols     = s->cols;
    int last_pair = -1;
    attr_t last_attr = 0;

    for (int r = 0; r < rows_eff && r < SCREEN_MAX_H; r++) {
        for (int c = 0; c < cols && c < SCREEN_MAX_W; c++) {
            uint8_t layer = s->sand.sand[r][c];
            if (layer == 0) continue;
            uint8_t mom = s->sand.mom[r][c];

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

/* ── §8 render (cont'd) — the ncurses surface and the HUD ── */

/* The terminal as a drawing surface: just its size in cells, re-read at startup
 * and after every resize so the layout keeps up with the window. */
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

static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_render(s);

    int total = s->sand.total_grains;
    if (total < 1) total = 1;
    int pct = 100 * s->sand.transferred / total;
    if (pct > 100) pct = 100;

    /* Top row: status readout, clipped to the window width so it never wraps. */
    char buf[200];
    snprintf(buf, sizeof buf,
             " SANDS OF TIME   %s   pat:%s   theme:%s   "
             "Time:%3d%%   Flips:%2d   grav:%s   auto:%s   %5.1f fps  %3d Hz ",
             s->paused ? "PAUSED " : "FLOWING",
             patterns[s->current_pattern].name,
             themes[s->current_theme].name,
             pct, s->sand.flip_count,
             s->sand.gravity_dir > 0 ? "DOWN" : "UP  ",
             s->auto_flip ? "ON " : "OFF",
             fps, sim_fps);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(0, x, ' ');
    mvprintw(0, 0, "%.*s", sc->cols, buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom row: the key hints. */
    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%.*s", sc->cols,
             " spc:flip  a:auto  r:refill  n/N:pat  t/T:theme  p:pause  [/]:Hz  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §9 app — the main loop, keys, signals, and frame timing ── */

/* The top-level bundle: the scene, its display, the sim-speed setting, and two
 * flags the signal handlers flip. It's a single global because POSIX signal
 * handlers can't be passed any data — they can only reach these fields through
 * a file-scope object. The flags use sig_atomic_t because that's the one type
 * the C standard promises is safe to set in a handler and read in the loop. */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;       /* simulation ticks per second */
    volatile sig_atomic_t running;       /* cleared by Ctrl-C / kill to exit */
    volatile sig_atomic_t need_resize;   /* set on window resize */
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
        sand_flip(&s->sand);
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
