/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * magnetic_fields.c — iron-filings-style magnetic field flow.
 *
 * Hundreds of tiny particles drift along magnetic field lines from N
 * poles toward S poles, tracing out the patterns you'd see if you
 * sprinkled iron filings around a magnet. There are 30 built-in pole
 * arrangements (a single pole, a bar magnet, rings, grids, chaos);
 * cycle them with n/p. The math is the same for every one — only the
 * list of poles changes.
 *
 * The field at any point is just the sum of each pole's pull, where a
 * pole's strength falls off with distance (the classic inverse-square
 * law from any intro electromagnetism text — see Griffiths,
 * "Introduction to Electrodynamics", ch. 5). Real magnets have no
 * lone "monopoles", so a single signed pole is a convenient 2-D
 * fiction; the flow everywhere except right at a pole looks physically
 * right. The density-to-character idea (faint dot up to solid block)
 * comes from Paul Bourke's ASCII grey-scale ramp:
 *   https://paulbourke.net/dataformats/asciiart/
 *
 * Sister file: ./flow_field_particles.c uses the same particle engine
 * but with made-up math fields instead of physical magnets.
 *
 * Keys:
 *   q / ESC  quit          space  pause       r      reset
 *   n / N    next pattern   p / P  prev pattern
 *   t / T    theme          g / G  glyph set
 *   + / =    faster          -     slower      ] / [  tick rate up/down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra magnetic_fields.c -o magnetic -lncurses -lm
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    CELLS_MAX         = MAP_W_MAX * MAP_H_MAX,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    MAX_PARTICLES     = 1024,
    N_PARTICLES_DEF   =  256,

    AGE_MIN_TICKS     =  60,
    AGE_MAX_TICKS     = 360,

    SPEED_MIN         =   1,
    SPEED_DEF         =   8,
    SPEED_MAX         =  64,

    MAX_POLES         =  32,

    /* Color slots. HUD/HINT are reserved by the project convention;
     * the trail palette uses four consecutive slots starting at BASE. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_TRAIL_BASE   =   3,    /* four trail colours: BASE .. BASE+3 */
    PAIR_NORTH        =   7,    /* red 'N' marker, same on every theme  */
    PAIR_SOUTH        =   8,    /* blue 'S' marker, same on every theme */
};

#define TRAIL_GLOW_DECAY    0.6f
#define GLOW_THRESHOLD      0.05f
#define VELOCITY_EPSILON    1e-6f   /* if the field is weaker than this, don't bother steering */
#define TRAIL_HIT_INTENSITY 1.0f    /* brightness left behind where a particle lands */

/* How fast the poles slowly wander, in radians per second. Keeps the
 * picture gently alive instead of frozen. */
#define DRIFT_RATE          0.2f

/* How many colours each theme uses for the trails. */
#define N_BANDS             4

/* Field strengths, one per pattern. Tuned by eye so the particles move
 * at a nice pace. Patterns with more poles use weaker ones so the total
 * pull stays in a similar range no matter how many poles there are. */

/* Tier 1 — simple (1–2 poles) */
#define MONOPOLE_STRENGTH   40.0f
#define DIPOLE_STRENGTH     50.0f
#define DIPOLE_V_STRENGTH   50.0f
#define REPELLER_STRENGTH   40.0f
#define ATTRACT_STRENGTH    40.0f

/* Tier 2 — few poles (3–4) */
#define TRIPOLE_STRENGTH    40.0f
#define TRIANGLE_STRENGTH   35.0f
#define HORSHOE_STRENGTH    45.0f
#define QUAD_STRENGTH       40.0f
#define CROSS_STRENGTH      35.0f
#define PINWHEEL_STRENGTH   25.0f

/* Tier 3 — medium (5–8 poles) */
#define CHAIN_STRENGTH      35.0f
#define CHAIN_V_STRENGTH    35.0f
#define HEXAPOLE_STRENGTH   30.0f
#define TWIN_DIP_STRENGTH   35.0f
#define STAR_5_STRENGTH     30.0f
#define OCTUPOLE_STRENGTH   25.0f
#define MIRROR_STRENGTH     40.0f

/* Tier 4 — structured complex (8+ poles) */
#define NESTED_STRENGTH     25.0f
#define SUNSPOT_STRENGTH    20.0f
#define GRID3_STRENGTH      25.0f
#define RING_8_STRENGTH     25.0f
#define DBL_RING_STRENGTH   20.0f
#define COIL_STRENGTH       18.0f
#define HELMHLTZ_STRENGTH   20.0f
#define SOLENOID_STRENGTH   20.0f

/* Tier 5 — chaotic / dense */
#define PLASMA_STRENGTH     25.0f
#define MAZE_STRENGTH       20.0f
#define AURORA_STRENGTH     18.0f
#define CHAOS_STRENGTH      18.0f

#define EPS_R2              4.0f    /* a fudge added to distance so we never divide by zero right on a pole */

#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/* On-screen layout. The top three rows show information (title, current
 * settings, a small legend); the field fills the middle; the bottom row
 * lists the keys you can press. */
#define HUD_TOP_ROWS             3
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1
#define HUD_PATTERN_FIELD_W      20
#define HUD_THEME_FIELD_W        17
#define HUD_PALETTE_LABEL_W       9

/*
 * Pattern — which pole arrangement is on screen, one of 30.
 * The simulation is the same for all of them; only the poles differ.
 * Cycle through them with n/p. They run roughly simple to complex.
 */
typedef enum {
    /* Tier 1 — simple (1–2 poles) */
    PATTERN_MONOPOLE = 0,
    PATTERN_DIPOLE   = 1,
    PATTERN_DIPOLE_V = 2,
    PATTERN_REPELLER = 3,
    PATTERN_ATTRACT  = 4,

    /* Tier 2 — few poles (3–4) */
    PATTERN_TRIPOLE  = 5,
    PATTERN_TRIANGLE = 6,
    PATTERN_HORSHOE  = 7,
    PATTERN_QUAD     = 8,
    PATTERN_CROSS    = 9,
    PATTERN_PINWHEEL = 10,

    /* Tier 3 — medium (5–8 poles) */
    PATTERN_CHAIN    = 11,
    PATTERN_CHAIN_V  = 12,
    PATTERN_HEXAPOLE = 13,
    PATTERN_TWIN_DIP = 14,
    PATTERN_STAR_5   = 15,
    PATTERN_OCTUPOLE = 16,
    PATTERN_MIRROR   = 17,

    /* Tier 4 — structured complex (8+ poles) */
    PATTERN_NESTED   = 18,
    PATTERN_SUNSPOT  = 19,
    PATTERN_GRID3    = 20,
    PATTERN_RING_8   = 21,
    PATTERN_DBL_RING = 22,
    PATTERN_COIL     = 23,
    PATTERN_HELMHLTZ = 24,
    PATTERN_SOLENOID = 25,

    /* Tier 5 — chaotic / dense */
    PATTERN_PLASMA   = 26,
    PATTERN_MAZE     = 27,
    PATTERN_AURORA   = 28,
    PATTERN_CHAOS    = 29,

    N_PATTERNS       = 30,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_MONOPOLE: return "MONOPOLE";
    case PATTERN_DIPOLE:   return "DIPOLE  ";
    case PATTERN_DIPOLE_V: return "DIPOLE_V";
    case PATTERN_REPELLER: return "REPELLER";
    case PATTERN_ATTRACT:  return "ATTRACT ";
    case PATTERN_TRIPOLE:  return "TRIPOLE ";
    case PATTERN_TRIANGLE: return "TRIANGLE";
    case PATTERN_HORSHOE:  return "HORSHOE ";
    case PATTERN_QUAD:     return "QUAD    ";
    case PATTERN_CROSS:    return "CROSS   ";
    case PATTERN_PINWHEEL: return "PINWHEEL";
    case PATTERN_CHAIN:    return "CHAIN   ";
    case PATTERN_CHAIN_V:  return "CHAIN_V ";
    case PATTERN_HEXAPOLE: return "HEXAPOLE";
    case PATTERN_TWIN_DIP: return "TWIN_DIP";
    case PATTERN_STAR_5:   return "STAR_5  ";
    case PATTERN_OCTUPOLE: return "OCTUPOLE";
    case PATTERN_MIRROR:   return "MIRROR  ";
    case PATTERN_NESTED:   return "NESTED  ";
    case PATTERN_SUNSPOT:  return "SUNSPOT ";
    case PATTERN_GRID3:    return "GRID3   ";
    case PATTERN_RING_8:   return "RING_8  ";
    case PATTERN_DBL_RING: return "DBL_RING";
    case PATTERN_COIL:     return "COIL    ";
    case PATTERN_HELMHLTZ: return "HELMHLTZ";
    case PATTERN_SOLENOID: return "SOLENOID";
    case PATTERN_PLASMA:   return "PLASMA  ";
    case PATTERN_MAZE:     return "MAZE    ";
    case PATTERN_AURORA:   return "AURORA  ";
    case PATTERN_CHAOS:    return "CHAOS   ";
    default:               return "?       ";
    }
}

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* If a frame ever takes longer than this (a stall, a debugger pause),
 * pretend only this much time passed so the sim doesn't try to catch up
 * all at once. Idea from Glenn Fiedler's "Fix Your Timestep". */
#define DT_MAX_NS       (100 * NS_PER_MS)
#define FRAME_CAP_FPS   60

/*
 * Theme — one colour scheme for the particle trails. There are ten;
 * press t/T to flip between them. Each is just four trail colours
 * running dim to bright, plus a leftover "flash" colour this file
 * doesn't use (kept only so sibling files share the same struct shape).
 * Each particle picks one of the four colours when it spawns and paints
 * its trail in that colour, so a coherent dim-to-bright set looks best.
 * The N/S pole markers ignore the theme entirely — they're always
 * red/blue (the usual physics convention) so polarity stays obvious.
 * The numbers are xterm-256 colour codes, not RGB; on a terminal with
 * fewer colours, theme_apply() falls back to a basic 8-colour set.
 */
typedef struct {
    const char *name;             /* short label shown in the HUD       */
    short       trail[N_BANDS];   /* the four trail colours, dim to bright */
    short       flash;            /* unused here; kept to match sister files */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      trail0 trail1 trail2 trail3 flash */
    { "DEFAULT", {  33,  117,  220,  220 }, 226 },
    { "MATRIX",  {  22,   34,   46,  118 }, 226 },
    { "NOVA",    {  53,  129,  201,  219 }, 226 },
    { "MONO",    { 240,  244,  250,  254 }, 226 },
    { "OCEAN",   {  17,   33,   39,   51 }, 226 },
    { "FIRE",    {  88,  124,  208,  226 }, 196 },
    { "EARTH",   {  58,  100,  173,  230 }, 226 },
    { "FOREST",  {  22,   28,   64,  144 }, 226 },
    { "DESERT",  {  94,  130,  173,  222 }, 226 },
    { "ARCTIC",  {  18,   39,  159,  231 }, 226 },
};

/*
 * GlyphSet — three characters for faint, medium, and dense trail spots.
 * There are five sets, from thin marks to fat ones; press g/G to switch.
 * Keeping the character choice separate from the colour theme lets you
 * mix and match — a faint theme can use bold glyphs, and so on. The
 * brighter a cell's trail, the heavier the character it gets.
 */
typedef struct {
    const char *name;       /* short label shown in the HUD       */
    char        low;        /* faintest visible trail             */
    char        mid;        /* medium trail                       */
    char        high;       /* densest trail                      */
} GlyphSet;

#define N_GLYPH_SETS 5

static const GlyphSet glyph_sets[N_GLYPH_SETS] = {
    { "SLIM",    '.', '\'', ':' },
    { "LIGHT",   '.', '*',  '+' },
    { "MEDIUM",  '.', '*',  '#' },
    { "HEAVY",   'o', 'O',  '@' },
    { "FAT",     '+', '#',  'M' },
};

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
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < N_BANDS; i++)
            init_pair(PAIR_TRAIL_BASE + i, t->trail[i], -1);
    } else {
        static const short fallback[N_BANDS] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < N_BANDS; i++)
            init_pair(PAIR_TRAIL_BASE + i, fallback[i], -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
        /* Red for north, blue for south — the usual textbook colours,
         * fixed so the poles read the same no matter the theme. */
        init_pair(PAIR_NORTH,      196, -1);   /* hot red    */
        init_pair(PAIR_SOUTH,       33, -1);   /* clear blue */
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,   -1);
        init_pair(PAIR_NORTH,     COLOR_RED,    -1);
        init_pair(PAIR_SOUTH,     COLOR_BLUE,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  fields — magnetic field math                                       */
/* ===================================================================== */

/*
 * Pole — a single magnet point that pushes or pulls on the field. This
 * is the fixed description of one pole; it never changes while running.
 * Think of a north pole as a fountain spraying the field outward and a
 * south pole as a drain sucking it in. Its pull gets weaker the farther
 * away you are (the inverse-square law from Griffiths, ch. 5).
 *
 * Real magnets never come as a single lone pole, so this is a handy
 * 2-D fiction — a +1 or -1 point standing in for the tip of a long bar
 * magnet. A whole pattern is just several of these added together.
 *
 * Each pole also wanders a little: bx/by is its home spot, and ox/oy
 * plus phase make it trace a small slow ellipse around that home so the
 * picture stays alive. Different phases keep poles from moving in lock-step.
 */
typedef struct {
    float bx, by;       /* home position, the centre of the slow wander    */
    float ox, oy;       /* how far it wanders, sideways and up/down         */
    float phase;        /* head start in its wander, so poles move out of sync */
    float strength;     /* how hard this pole pushes/pulls                 */
    int   polarity;     /* +1 = north (pushes out), -1 = south (pulls in)  */
} Pole;

/*
 * ActivePole — where a pole actually is right now, this frame, after its
 * wander is worked out. We compute the wandering position once per frame
 * for each pole and store it here, so the inner particle loop can read a
 * plain position instead of redoing the slow sin/cos math thousands of
 * times. It only needs position, strength, and which way it pushes.
 */
typedef struct {
    float x, y;
    float strength;
    int   polarity;
} ActivePole;

/*
 * field_at — which way and how hard the field pulls at one spot.
 * Adds up the pull from every pole: each pole pulls along the line
 * between it and the spot, more strongly the closer it is.
 */
static void field_at(const ActivePole *poles, int n_poles,
                     float x, float y,
                     float *out_bx, float *out_by)
{
    float bx = 0.0f, by = 0.0f;
    for (int i = 0; i < n_poles; i++) {
        float dx = x - poles[i].x;
        float dy = y - poles[i].y;
        float r2 = dx * dx + dy * dy + EPS_R2;
        float r3 = r2 * sqrtf(r2);
        float w  = (float)poles[i].polarity * poles[i].strength / r3;
        bx += dx * w;
        by += dy * w;
    }
    *out_bx = bx;
    *out_by = by;
}

/* ===================================================================== */
/* §6  patterns — 30 pole configurations                                  */
/* ===================================================================== */

/* ── pole-placement builders ─────────────────────────────────────── *
 * Reusable ways to drop poles: one pole, a row along a line, a ring,
 * a grid, a random scatter. The patterns below are built from these.
 * Each one appends to out[] and bumps the running count *n, so you can
 * stack several together to make a single pattern. */

static Pole make_pole(float bx, float by, float ox, float oy,
                       float phase, float strength, int polarity)
{
    return (Pole){
        .bx = bx, .by = by, .ox = ox, .oy = oy,
        .phase = phase, .strength = strength, .polarity = polarity,
    };
}

/* A row of evenly spaced poles from one point to another, flipping
 * north/south as it goes. The first one's sign is start_sign. */
static void place_chain_along(Pole out[], int max_n, int *n,
                               float x0, float y0, float x1, float y1,
                               int count, int start_sign,
                               float strength, float ox, float oy)
{
    for (int i = 0; i < count && *n < max_n; i++) {
        float t  = (count > 1) ? (float)i / (float)(count - 1) : 0.5f;
        float bx = x0 + (x1 - x0) * t;
        float by = y0 + (y1 - y0) * t;
        int   pol = (i & 1) ? -start_sign : start_sign;
        out[(*n)++] = make_pole(bx, by, ox, oy,
                                 (float)i * 0.4f, strength, pol);
    }
}

/* Poles spread evenly around an oval. If `alternate` is set they flip
 * north/south around the ring; otherwise they're all the same sign. */
static void place_ring(Pole out[], int max_n, int *n,
                        float cx, float cy, float rx, float ry,
                        int count, int alternate, int start_sign,
                        float strength, float ox, float oy)
{
    for (int i = 0; i < count && *n < max_n; i++) {
        float ang = (float)i * 2.0f * (float)M_PI / (float)count;
        int   pol = alternate ? ((i & 1) ? -start_sign : start_sign)
                              : start_sign;
        out[(*n)++] = make_pole(cx + rx * cosf(ang),
                                  cy + ry * sinf(ang),
                                  ox, oy, ang, strength, pol);
    }
}

/* A checkerboard of poles, north and south alternating like the squares
 * on a chessboard. */
static void place_grid(Pole out[], int max_n, int *n,
                        float cx, float cy, float dx, float dy,
                        int rows, int cols, float strength)
{
    int k = 0;
    for (int gy = 0; gy < rows; gy++) {
        for (int gx = 0; gx < cols; gx++) {
            if (*n >= max_n) return;
            float bx = cx + ((float)gx - (float)(cols - 1) * 0.5f) * dx;
            float by = cy + ((float)gy - (float)(rows - 1) * 0.5f) * dy;
            int   pol = ((gx + gy) & 1) ? -1 : +1;
            out[(*n)++] = make_pole(bx, by, 1.0f, 1.0f,
                                      (float)k * 0.3f, strength, pol);
            k++;
        }
    }
}

/* Scatter poles at random spots (kept a little away from the edges),
 * each randomly north or south. strength_jitter randomly varies how
 * strong each one is, if you pass more than zero. */
static void place_random(Pole out[], int max_n, int *n,
                          int w, int h, int count, float strength,
                          float strength_jitter)
{
    int margin_x = w / 10;
    int margin_y = h / 8;
    for (int i = 0; i < count && *n < max_n; i++) {
        float px = (float)(margin_x + rand() % (w - 2 * margin_x));
        float py = (float)(margin_y + rand() % (h - 2 * margin_y));
        float s  = strength;
        if (strength_jitter > 0.0f) {
            float r = (float)(rand() % 200 - 100) / 100.0f;   /* −1..+1 */
            s *= (1.0f + r * strength_jitter);
        }
        out[(*n)++] = make_pole(px, py, 1.0f, 1.0f,
                                  (float)i * 0.31f, s,
                                  (rand() & 1) ? -1 : +1);
    }
}

/* ── per-pattern pole placers ────────────────────────────────────── *
 * One function per pattern. Each lays out the poles for that one
 * pattern and returns how many it placed. pattern_init_poles further
 * down just picks the right one. The names follow the physics. */

/* ── Tier 1 — simple (1–2 poles) ─────────────────────────────── */

/* MONOPOLE — a single north pole in the middle. The field just sprays
 * straight out in every direction. */
static int place_monopole_centre(Pole out[], int max_n, float cx, float cy)
{
    if (max_n < 1) return 0;
    out[0] = make_pole(cx, cy, 1.0f, 1.0f, 0.0f,
                        MONOPOLE_STRENGTH, +1);
    return 1;
}

/* DIPOLE — a plain bar magnet lying flat: north on the left, south on
 * the right. */
static int place_dipole_horizontal(Pole out[], int max_n,
                                    float cx, float cy, int w)
{
    if (max_n < 2) return 0;
    float dx = (float)w * 0.20f;
    out[0] = make_pole(cx - dx, cy, 3.0f, 2.0f, 0.0f,
                        DIPOLE_STRENGTH, +1);
    out[1] = make_pole(cx + dx, cy, 3.0f, 2.0f, (float)M_PI,
                        DIPOLE_STRENGTH, -1);
    return 2;
}

/* DIPOLE_V — the bar magnet stood on end: north up top, south below. */
static int place_dipole_vertical(Pole out[], int max_n,
                                  float cx, float cy, int h)
{
    if (max_n < 2) return 0;
    float dy = (float)h * 0.25f;
    out[0] = make_pole(cx, cy - dy, 2.0f, 3.0f, 0.0f,
                        DIPOLE_V_STRENGTH, +1);
    out[1] = make_pole(cx, cy + dy, 2.0f, 3.0f, (float)M_PI,
                        DIPOLE_V_STRENGTH, -1);
    return 2;
}

/* REPELLER — two north poles side by side. Both push the field away,
 * so it piles up between them and shoots out sideways. */
static int place_repeller_pair(Pole out[], int max_n,
                                float cx, float cy, int w)
{
    if (max_n < 2) return 0;
    float dx = (float)w * 0.12f;
    out[0] = make_pole(cx - dx, cy, 2.0f, 2.0f, 0.0f,
                        REPELLER_STRENGTH, +1);
    out[1] = make_pole(cx + dx, cy, 2.0f, 2.0f, (float)M_PI,
                        REPELLER_STRENGTH, +1);
    return 2;
}

/* ATTRACT — two south poles side by side. The mirror image of REPELLER:
 * the field flows inward toward both and splits between them. */
static int place_attract_pair(Pole out[], int max_n,
                               float cx, float cy, int w)
{
    if (max_n < 2) return 0;
    float dx = (float)w * 0.12f;
    out[0] = make_pole(cx - dx, cy, 2.0f, 2.0f, 0.0f,
                        ATTRACT_STRENGTH, -1);
    out[1] = make_pole(cx + dx, cy, 2.0f, 2.0f, (float)M_PI,
                        ATTRACT_STRENGTH, -1);
    return 2;
}

/* ── Tier 2 — few poles (3–4) ────────────────────────────────── */

/* TRIPOLE — north, south, north in a row. Both outer norths feed the
 * south in the middle, making two loops that share the centre. */
static int place_tripole_line(Pole out[], int max_n,
                               float cx, float cy, int w)
{
    if (max_n < 3) return 0;
    float sp = (float)w * 0.18f;
    for (int i = 0; i < 3; i++) {
        out[i] = make_pole(cx + ((float)i - 1.0f) * sp, cy,
                            2.0f, 1.5f, (float)i * 0.5f,
                            TRIPOLE_STRENGTH,
                            (i & 1) ? -1 : +1);
    }
    return 3;
}

/* TRIANGLE — one north at the top, two souths at the bottom corners.
 * The field fans down from the north into each south. */
static int place_triangle_1n2s(Pole out[], int max_n,
                                float cx, float cy, int h)
{
    if (max_n < 3) return 0;
    float r = (float)h * 0.30f;
    for (int i = 0; i < 3; i++) {
        float ang = (float)i * 2.0f * (float)M_PI / 3.0f
                  - (float)M_PI * 0.5f;
        out[i] = make_pole(cx + r * cosf(ang),
                            cy + r * sinf(ang),
                            1.5f, 1.5f, ang,
                            TRIANGLE_STRENGTH,
                            (i == 0) ? +1 : -1);
    }
    return 3;
}

/* HORSESHOE — north and south tips close together near the top, like
 * the two ends of a horseshoe magnet. The tight arc of field between
 * them is the giveaway. */
static int place_horseshoe_tips(Pole out[], int max_n,
                                 float cx, float cy, int w, int h)
{
    if (max_n < 2) return 0;
    float gap = (float)w * 0.06f;
    float yp  = cy - (float)h * 0.15f;
    out[0] = make_pole(cx - gap, yp, 1.0f, 1.0f, 0.0f,
                        HORSHOE_STRENGTH, +1);
    out[1] = make_pole(cx + gap, yp, 1.0f, 1.0f, (float)M_PI,
                        HORSHOE_STRENGTH, -1);
    return 2;
}

/* QUAD — four poles at the corners of a small square, signs alternating.
 * The textbook "quadrupole" (Jackson, ch. 4). */
static int place_quadrupole_corners(Pole out[], int max_n,
                                     float cx, float cy, int w, int h)
{
    if (max_n < 4) return 0;
    float ax = (float)w * 0.18f;
    float ay = (float)h * 0.22f;
    const struct { float dx, dy; int sign; } cfg[4] = {
        {-ax, -ay, +1}, {+ax, -ay, -1},
        {-ax, +ay, -1}, {+ax, +ay, +1},
    };
    for (int i = 0; i < 4; i++) {
        out[i] = make_pole(cx + cfg[i].dx, cy + cfg[i].dy,
                            2.0f, 2.0f, (float)i * 0.5f,
                            QUAD_STRENGTH, cfg[i].sign);
    }
    return 4;
}

/* CROSS — four alternating poles in a plus shape (top, right, bottom,
 * left). Same idea as QUAD, just turned 45 degrees. */
static int place_quadrupole_cross(Pole out[], int max_n,
                                   float cx, float cy, int w, int h)
{
    if (max_n < 4) return 0;
    float r = (float)h * 0.28f;
    float aspect = (float)h / (float)w;
    const float xs[4] = { cx, cx + r / aspect, cx, cx - r / aspect };
    const float ys[4] = { cy - r, cy, cy + r, cy };
    const int   sg[4] = { +1, -1, +1, -1 };
    for (int i = 0; i < 4; i++) {
        out[i] = make_pole(xs[i], ys[i], 1.5f, 1.5f,
                            (float)i * 0.4f,
                            CROSS_STRENGTH, sg[i]);
    }
    return 4;
}

/* PINWHEEL — four little bar magnets set sideways around the centre, so
 * the whole field swirls like a windmill seen from above. Eight poles. */
static int place_pinwheel_dipoles(Pole out[], int max_n,
                                   float cx, float cy, int h)
{
    if (max_n < 8) return 0;
    float r   = (float)h * 0.22f;
    float gap = (float)h * 0.06f;
    int n = 0;
    for (int i = 0; i < 4; i++) {
        float ang = (float)i * (float)M_PI * 0.5f;
        float tx  = -sinf(ang);                   /* sideways direction */
        float ty  =  cosf(ang);
        float ix  = cx + r * cosf(ang);           /* centre of this pair */
        float iy  = cy + r * sinf(ang);
        /* Put the north and south to either side, crosswise to the spoke,
         * so each little magnet pushes the field around in a circle. */
        out[n++] = make_pole(ix + gap * tx, iy + gap * ty,
                              1.0f, 1.0f, ang,
                              PINWHEEL_STRENGTH, +1);
        out[n++] = make_pole(ix - gap * tx, iy - gap * ty,
                              1.0f, 1.0f, ang + (float)M_PI,
                              PINWHEEL_STRENGTH, -1);
    }
    return n;
}

/* ── Tier 3 — medium (5–8 poles) ─────────────────────────────── */

/* CHAIN — six poles in a row across the screen, signs alternating. */
static int place_chain_horizontal(Pole out[], int max_n,
                                   float cx, float cy, int w)
{
    int n = 0;
    place_chain_along(out, max_n, &n,
                      cx - (float)w * 0.35f, cy,
                      cx + (float)w * 0.35f, cy,
                      6, +1, CHAIN_STRENGTH, 2.0f, 1.0f);
    return n;
}

/* CHAIN_V — the same six-pole chain, stacked in a vertical column. */
static int place_chain_vertical(Pole out[], int max_n,
                                 float cx, float cy, int h)
{
    int n = 0;
    place_chain_along(out, max_n, &n,
                      cx, cy - (float)h * 0.40f,
                      cx, cy + (float)h * 0.40f,
                      6, +1, CHAIN_V_STRENGTH, 1.0f, 2.0f);
    return n;
}

/* HEXAPOLE — six poles around a ring, signs alternating. */
static int place_hexapole_ring(Pole out[], int max_n,
                                float cx, float cy, int w, int h)
{
    int n = 0;
    place_ring(out, max_n, &n, cx, cy,
               (float)w * 0.25f, (float)h * 0.28f,
               6, /*alternate*/1, +1, HEXAPOLE_STRENGTH, 1.5f, 1.5f);
    return n;
}

/* TWIN_DIP — two flat bar magnets, one stacked above the other. */
static int place_twin_dipole(Pole out[], int max_n,
                              float cx, float cy, int w, int h)
{
    if (max_n < 4) return 0;
    float dx = (float)w * 0.20f;
    float dy = (float)h * 0.18f;
    out[0] = make_pole(cx - dx, cy - dy, 2.0f, 1.0f,
                        0.0f, TWIN_DIP_STRENGTH, +1);
    out[1] = make_pole(cx + dx, cy - dy, 2.0f, 1.0f,
                        (float)M_PI, TWIN_DIP_STRENGTH, -1);
    out[2] = make_pole(cx - dx, cy + dy, 2.0f, 1.0f,
                        0.5f, TWIN_DIP_STRENGTH, +1);
    out[3] = make_pole(cx + dx, cy + dy, 2.0f, 1.0f,
                        (float)M_PI + 0.5f, TWIN_DIP_STRENGTH, -1);
    return 4;
}

/* STAR_5 — five poles on a ring. With an odd number the alternating
 * signs don't come out even, and that lopsidedness makes for an
 * interesting flow. */
static int place_pentapole_ring(Pole out[], int max_n,
                                 float cx, float cy, int w, int h)
{
    int n = 0;
    place_ring(out, max_n, &n, cx, cy,
               (float)w * 0.27f, (float)h * 0.30f,
               5, /*alternate*/1, +1, STAR_5_STRENGTH, 1.5f, 1.5f);
    return n;
}

/* OCTUPOLE — eight poles around a ring, signs alternating. One step up
 * from QUAD; the more poles you ring up, the faster the field fades away
 * with distance (Jackson, ch. 4). */
static int place_octupole_ring(Pole out[], int max_n,
                                float cx, float cy, int w, int h)
{
    int n = 0;
    place_ring(out, max_n, &n, cx, cy,
               (float)w * 0.28f, (float)h * 0.32f,
               8, /*alternate*/1, +1, OCTUPOLE_STRENGTH, 1.5f, 1.5f);
    return n;
}

/* MIRROR — two north poles stacked vertically. The field bulges out from
 * each, meets in the middle, and turns back — the "magnetic mirror" shape
 * used to trap plasma (Chen, ch. 2). */
static int place_magnetic_mirror(Pole out[], int max_n,
                                  float cx, float cy, int h)
{
    if (max_n < 2) return 0;
    float dy = (float)h * 0.32f;
    out[0] = make_pole(cx, cy - dy, 1.0f, 2.0f, 0.0f,
                        MIRROR_STRENGTH, +1);
    out[1] = make_pole(cx, cy + dy, 1.0f, 2.0f, (float)M_PI,
                        MIRROR_STRENGTH, +1);
    return 2;
}

/* ── Tier 4 — structured complex (8+ poles) ──────────────────── */

/* NESTED — a small square of four poles inside a bigger square of four,
 * with the outer signs flipped. Each inner/outer corner pair acts like a
 * little magnet pointing outward. Eight poles. */
static int place_nested_squares(Pole out[], int max_n,
                                 float cx, float cy, int w, int h)
{
    if (max_n < 8) return 0;
    float ix = (float)w * 0.10f, iy = (float)h * 0.12f;
    float ox = (float)w * 0.25f, oy = (float)h * 0.30f;
    const struct { float dx, dy; int sign_in, sign_out; } cfg[4] = {
        {-1, -1, +1, -1}, {+1, -1, -1, +1},
        {-1, +1, -1, +1}, {+1, +1, +1, -1},
    };
    int n = 0;
    for (int i = 0; i < 4; i++) {
        out[n++] = make_pole(cx + cfg[i].dx * ix, cy + cfg[i].dy * iy,
                              1.0f, 1.0f, (float)i * 0.4f,
                              NESTED_STRENGTH, cfg[i].sign_in);
        out[n++] = make_pole(cx + cfg[i].dx * ox, cy + cfg[i].dy * oy,
                              1.0f, 1.0f, (float)i * 0.4f + 0.3f,
                              NESTED_STRENGTH, cfg[i].sign_out);
    }
    return n;
}

/* SUNSPOT — two clusters, each three norths beside three souths, like a
 * pair of sunspots where magnetism pokes out of the sun and loops back
 * (Chen, solar magnetism chapter). Twelve poles. */
static int place_sunspot_clusters(Pole out[], int max_n,
                                   float cx, float cy, int w)
{
    if (max_n < 12) return 0;
    float dxc = (float)w * 0.22f;
    float sp  = 2.5f;
    const float xs[2] = { cx - dxc, cx + dxc };
    int n = 0;
    for (int s = 0; s < 2; s++) {
        int sign_lr = (s == 0) ? +1 : -1;
        for (int i = 0; i < 3; i++) {
            float dy_i = ((float)i - 1.0f) * sp;
            out[n++] = make_pole(xs[s] - sp, cy + dy_i,
                                   1.0f, 1.0f, (float)(s * 3 + i) * 0.4f,
                                   SUNSPOT_STRENGTH,  sign_lr);
            out[n++] = make_pole(xs[s] + sp, cy + dy_i,
                                   1.0f, 1.0f, (float)(s * 3 + i) * 0.4f + 1.0f,
                                   SUNSPOT_STRENGTH, -sign_lr);
        }
    }
    return n;
}

/* GRID3 — a 3x3 checkerboard of poles. Nine in all. */
static int place_grid_3x3(Pole out[], int max_n,
                           float cx, float cy, int w, int h)
{
    int n = 0;
    place_grid(out, max_n, &n, cx, cy,
               (float)w * 0.22f, (float)h * 0.24f, 3, 3, GRID3_STRENGTH);
    return n;
}

/* RING_8 — eight north poles on a circle, all the same sign. Makes a
 * sunburst, with curved arms where neighbouring poles tug on each other. */
static int place_ring8_samesign(Pole out[], int max_n,
                                 float cx, float cy, int w, int h)
{
    int n = 0;
    place_ring(out, max_n, &n, cx, cy,
               (float)w * 0.28f, (float)h * 0.32f,
               8, /*alternate*/0, +1, RING_8_STRENGTH, 1.5f, 1.5f);
    return n;
}

/* DBL_RING — a ring of norths inside a ring of souths. The field flows
 * straight out from the inner ring into the outer one. */
static int place_concentric_rings(Pole out[], int max_n,
                                   float cx, float cy, int w, int h)
{
    int n = 0;
    place_ring(out, max_n, &n, cx, cy,
               (float)w * 0.15f, (float)h * 0.18f,
               6, /*alternate*/0, +1, DBL_RING_STRENGTH, 1.0f, 1.0f);
    place_ring(out, max_n, &n, cx, cy,
               (float)w * 0.32f, (float)h * 0.36f,
               8, /*alternate*/0, -1, DBL_RING_STRENGTH, 1.0f, 1.0f);
    return n;
}

/* COIL — twelve same-sign poles on a circle, standing in for a single
 * loop of current. The field is fairly even inside the loop and looks
 * like a bar magnet from outside (Griffiths, ch. 5). */
static int place_current_loop(Pole out[], int max_n,
                                float cx, float cy, int w, int h)
{
    int n = 0;
    place_ring(out, max_n, &n, cx, cy,
               (float)w * 0.30f, (float)h * 0.36f,
               12, /*alternate*/0, +1, COIL_STRENGTH, 1.0f, 1.0f);
    return n;
}

/* HELMHLTZ — two matching rings of norths, one above the other. The
 * field between them comes out nice and even, which is exactly why
 * physics labs use this setup. */
static int place_helmholtz_coils(Pole out[], int max_n,
                                  float cx, float cy, int w, int h)
{
    float ring_rx = (float)w * 0.18f;
    float ring_ry = (float)h * 0.10f;
    float sep     = (float)h * 0.22f;
    int n = 0;
    place_ring(out, max_n, &n, cx, cy - sep, ring_rx, ring_ry,
               6, /*alternate*/0, +1, HELMHLTZ_STRENGTH, 0.5f, 0.5f);
    place_ring(out, max_n, &n, cx, cy + sep, ring_rx, ring_ry,
               6, /*alternate*/0, +1, HELMHLTZ_STRENGTH, 0.5f, 0.5f);
    return n;
}

/* SOLENOID — a row of norths along the top and a row of souths along the
 * bottom. The field in the gap runs fairly straight, top to bottom. */
static int place_parallel_NS_plates(Pole out[], int max_n,
                                     float cx, float cy, int w, int h)
{
    int n = 0;
    place_chain_along(out, max_n, &n,
                      cx - (float)w * 0.30f, cy - (float)h * 0.30f,
                      cx + (float)w * 0.30f, cy - (float)h * 0.30f,
                      8, +1, SOLENOID_STRENGTH, 1.0f, 0.5f);
    place_chain_along(out, max_n, &n,
                      cx - (float)w * 0.30f, cy + (float)h * 0.30f,
                      cx + (float)w * 0.30f, cy + (float)h * 0.30f,
                      8, -1, SOLENOID_STRENGTH, 1.0f, 0.5f);
    return n;
}

/* ── Tier 5 — chaotic / dense ────────────────────────────────── */

/* PLASMA — 14 poles scattered at random, each randomly north or south. */
static int place_plasma_random(Pole out[], int max_n, int w, int h)
{
    int n = 0;
    place_random(out, max_n, &n, w, h, 14, PLASMA_STRENGTH, 0.0f);
    return n;
}

/* MAZE — 16 alternating poles strung along a wavy line that snakes
 * across the screen. */
static int place_maze_chain(Pole out[], int max_n,
                             float cy, int w, int h)
{
    const int count = 16;
    int n = 0;
    for (int i = 0; i < count && n < max_n; i++) {
        float t  = (float)i / (float)(count - 1);
        float bx = (float)w * 0.10f + t * (float)w * 0.80f;
        float by = cy + sinf(t * 4.0f * (float)M_PI) * (float)h * 0.20f;
        out[n++] = make_pole(bx, by, 1.0f, 1.0f,
                              (float)i * 0.3f, MAZE_STRENGTH,
                              (i & 1) ? -1 : +1);
    }
    return n;
}

/* AURORA — 18 alternating poles along a broad arc, like a curtain of
 * northern lights hanging across the sky. */
static int place_aurora_arc(Pole out[], int max_n,
                             float cx, float cy, int h)
{
    const int count = 18;
    int n = 0;
    for (int i = 0; i < count && n < max_n; i++) {
        float t   = (float)i / (float)(count - 1);
        float ang = -(float)M_PI * 0.55f + t * (float)M_PI * 1.10f;
        float r   = (float)h * 0.55f;
        float bx  = cx + r * sinf(ang) * 1.4f;
        float by  = cy - r * cosf(ang) + (float)h * 0.30f;
        out[n++] = make_pole(bx, by, 0.5f, 0.5f,
                              (float)i * 0.25f, AURORA_STRENGTH,
                              (i & 1) ? -1 : +1);
    }
    return n;
}

/* CHAOS — 24 random poles, random signs, and random strengths too.
 * The busiest pattern of the lot. */
static int place_chaos_random(Pole out[], int max_n, int w, int h)
{
    int n = 0;
    place_random(out, max_n, &n, w, h, 24, CHAOS_STRENGTH, 0.30f);
    return n;
}

/* Picks the right pole-layout function for the chosen pattern and runs
 * it. One line per pattern. */
static int pattern_init_poles(Pole out[], int max_n, Pattern p, int w, int h)
{
    float cx = (float)w * 0.5f;
    float cy = (float)h * 0.5f;

    switch (p) {

    /* Tier 1 — simple (1–2 poles) */
    case PATTERN_MONOPOLE: return place_monopole_centre   (out, max_n, cx, cy);
    case PATTERN_DIPOLE:   return place_dipole_horizontal (out, max_n, cx, cy, w);
    case PATTERN_DIPOLE_V: return place_dipole_vertical   (out, max_n, cx, cy, h);
    case PATTERN_REPELLER: return place_repeller_pair     (out, max_n, cx, cy, w);
    case PATTERN_ATTRACT:  return place_attract_pair      (out, max_n, cx, cy, w);

    /* Tier 2 — few poles (3–4) */
    case PATTERN_TRIPOLE:  return place_tripole_line      (out, max_n, cx, cy, w);
    case PATTERN_TRIANGLE: return place_triangle_1n2s     (out, max_n, cx, cy, h);
    case PATTERN_HORSHOE:  return place_horseshoe_tips    (out, max_n, cx, cy, w, h);
    case PATTERN_QUAD:     return place_quadrupole_corners(out, max_n, cx, cy, w, h);
    case PATTERN_CROSS:    return place_quadrupole_cross  (out, max_n, cx, cy, w, h);
    case PATTERN_PINWHEEL: return place_pinwheel_dipoles  (out, max_n, cx, cy, h);

    /* Tier 3 — medium (5–8 poles) */
    case PATTERN_CHAIN:    return place_chain_horizontal  (out, max_n, cx, cy, w);
    case PATTERN_CHAIN_V:  return place_chain_vertical    (out, max_n, cx, cy, h);
    case PATTERN_HEXAPOLE: return place_hexapole_ring     (out, max_n, cx, cy, w, h);
    case PATTERN_TWIN_DIP: return place_twin_dipole       (out, max_n, cx, cy, w, h);
    case PATTERN_STAR_5:   return place_pentapole_ring    (out, max_n, cx, cy, w, h);
    case PATTERN_OCTUPOLE: return place_octupole_ring     (out, max_n, cx, cy, w, h);
    case PATTERN_MIRROR:   return place_magnetic_mirror   (out, max_n, cx, cy, h);

    /* Tier 4 — structured complex (8+ poles) */
    case PATTERN_NESTED:   return place_nested_squares    (out, max_n, cx, cy, w, h);
    case PATTERN_SUNSPOT:  return place_sunspot_clusters  (out, max_n, cx, cy, w);
    case PATTERN_GRID3:    return place_grid_3x3          (out, max_n, cx, cy, w, h);
    case PATTERN_RING_8:   return place_ring8_samesign    (out, max_n, cx, cy, w, h);
    case PATTERN_DBL_RING: return place_concentric_rings  (out, max_n, cx, cy, w, h);
    case PATTERN_COIL:     return place_current_loop      (out, max_n, cx, cy, w, h);
    case PATTERN_HELMHLTZ: return place_helmholtz_coils   (out, max_n, cx, cy, w, h);
    case PATTERN_SOLENOID: return place_parallel_NS_plates(out, max_n, cx, cy, w, h);

    /* Tier 5 — chaotic / dense */
    case PATTERN_PLASMA:   return place_plasma_random     (out, max_n, w, h);
    case PATTERN_MAZE:     return place_maze_chain        (out, max_n, cy, w, h);
    case PATTERN_AURORA:   return place_aurora_arc        (out, max_n, cx, cy, h);
    case PATTERN_CHAOS:    return place_chaos_random      (out, max_n, w, h);

    default:               return 0;
    }
}

/* Works out where every pole is right now by nudging each one a little
 * way around its slow wander. Run once per frame to keep the field from
 * sitting still. */
static int compute_active_poles(const Pole src[], int n_src,
                                ActivePole out[], float t)
{
    for (int i = 0; i < n_src; i++) {
        float ang = t * DRIFT_RATE + src[i].phase;
        out[i].x        = src[i].bx + src[i].ox * cosf(ang);
        out[i].y        = src[i].by + src[i].oy * sinf(ang);
        out[i].strength = src[i].strength;
        out[i].polarity = src[i].polarity;
    }
    return n_src;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * The whole simulation is split into six little structs, each handling
 * one job, all gathered into Scene below. Splitting it this way also
 * makes the function signatures honest: pass a `const Grid *` and it
 * plainly can't scribble on the buffers, and so on.
 */

/*
 * Particle — one of the dots drifting along the field. It has no weight
 * or momentum: every frame it just looks at which way the field points
 * where it stands and steps that way, like a single iron filing lining
 * up. ctrl.speed sets the pace. The field's strength swings wildly (huge
 * next to a pole, tiny far off), so we only use its direction; that
 * trimming happens in sample_unit_field_direction.
 *
 * Each particle is given a random lifespan so they don't all pile up
 * and get stuck near one pole — they keep dying and respawning at fresh
 * spots, which keeps the overall picture even.
 */
typedef struct {
    float x, y;       /* position on the grid (fractional, not whole cells) */
    int   color_idx;  /* which trail colour this particle paints, 0..3      */
    int   age;        /* how many ticks it's been alive                     */
    int   max_age;    /* respawn once it gets this old (or wanders off-grid) */
} Particle;

/*
 * Grid — how big the playing area is, in cells. Just the dimensions,
 * nothing else; lots of code needs them so they sit up front. Cells are
 * numbered row by row, so cell (x, y) is at index y*w + x — the same
 * order the render buffers use, which keeps the drawing loop fast.
 * width times height never exceeds CELLS_MAX; app_pick_map_size() caps
 * it (at most 200 x 56), and everything downstream trusts that cap.
 */
typedef struct {
    int w, h;         /* current width and height, in cells                */
    int total_cells;  /* width times height, saved so loops skip the multiply */
} Grid;

static inline int  grid_idx       (const Grid *g, int x, int y) { return y * g->w + x; }
static inline bool grid_in_bounds (const Grid *g, int x, int y)
{
    return x >= 0 && x < g->w && y >= 0 && y < g->h;
}

/*
 * RenderBuffers — what to draw at each cell, the bridge between the
 * moving particles and what shows on screen. Two arrays, one value per
 * cell: glow is how bright the trail is there (0 to 1), color is which
 * trail colour to use. The particles write into these; the drawing code
 * reads them and nothing else. The N/S pole markers are drawn straight
 * from the poles, not from here. Keeping the two as separate arrays
 * (rather than one array of pairs) is just faster: the draw loop reads
 * glow for every cell but colour only where there's actually a trail,
 * and the fade step only touches glow.
 */
typedef struct {
    /* Trail brightness per cell, 0 to 1. Set to full where a particle
     * lands, then faded a bit every tick to make trails trail off. */
    float   glow [CELLS_MAX];

    /* Trail colour per cell, taken from whichever particle last landed
     * here. Re-masked at draw time just to be safe. */
    uint8_t color[CELLS_MAX];
} RenderBuffers;

static void buffers_clear(RenderBuffers *b, int n)
{
    for (int i = 0; i < n; i++) {
        b->glow [i] = 0.0f;
        b->color[i] = 0;
    }
}

/*
 * Particles — the whole flock of dots. It's a fixed array big enough for
 * the most we'd ever use, plus a count of how many are actually live, so
 * spawning never has to ask for memory. Only the first `n` entries
 * count. The array holds up to 1024; we normally run 256, which fills
 * out the field lines nicely without clogging the screen.
 */
typedef struct {
    Particle pool[MAX_PARTICLES];
    int      n;
} Particles;

/*
 * PoleField — all the magnets making the field. It keeps two lists: the
 * fixed descriptions (base, with each pole's home and wander), and a
 * worked-out "where they are right now" copy (active), refreshed once a
 * frame. We split them so the slow sin/cos wander math runs about 30
 * times a frame instead of inside the inner particle loop, which runs
 * thousands of times. Same setup as AttractorPool in
 * flow_field_particles.c.
 */
typedef struct {
    Pole       base [MAX_POLES];   /* the fixed pole descriptions        */
    int        n_base;
    ActivePole active[MAX_POLES];  /* current positions, redone each tick */
    int        n_active;
} PoleField;

/*
 * SimState — the one value the simulation itself changes each tick. Kept
 * apart from the user's keyboard settings so it's clear what the sim
 * touches versus what you touch. This single number feeds the poles'
 * slow wander, so bumping it up each tick is what makes the whole field
 * gently move.
 */
typedef struct {
    /* Seconds elapsed since the last reset. Grows a little each tick,
     * goes back to zero on reset. */
    float field_time;
} SimState;

/*
 * Controls — everything the keyboard can change. The key handler writes
 * these; the simulation and the drawing code read them. Keeping the
 * user's settings separate from the sim's own state means the changes
 * only flow one way and the code stays simple to follow.
 */
typedef struct {
    /* When true the field stops moving, but the screen keeps redrawing
     * so the display stays responsive. */
    bool    paused;

    /* How fast the particles move, in cells per second. '+' doubles it,
     * '-' halves it, kept between 1 and 64. Doubling (rather than +1)
     * makes the steps feel even across the whole range. */
    int     speed;

    int     current_theme;       /* which colour theme is showing  */

    /* Which glyph set is showing. Changing this (g/G) only affects how
     * trails look, so it doesn't restart the simulation. */
    int     current_glyph_set;

    /* Which pole pattern is showing. Switching it (n/p) restarts the
     * scene so the new pattern's poles get laid out fresh. */
    Pattern current_pattern;
} Controls;

/*
 * Scene — everything, bundled together. Reading the fields top to bottom
 * is the quickest tour of the program: where things live (grid), what
 * gets drawn (buf), the moving dots (particles), the magnets (poles),
 * the bit of state the sim updates (sim), and the user's settings (ctrl).
 * They're listed so each one only leans on the ones above it. Only
 * scene_tick() needs to see all of them at once.
 */
typedef struct {
    Grid          grid;       /* the playing area's size                   */
    RenderBuffers buf;        /* what to draw; particles write, drawer reads */
    Particles     particles;  /* the moving dots                           */
    PoleField     poles;      /* the magnets making the field              */
    SimState      sim;        /* changed only by the simulation            */
    Controls      ctrl;       /* changed only by the keyboard              */
} Scene;

/* ── particle pipeline ───────────────────────────────────────────── *
 * What happens to one dot each tick: see which way the field points,
 * step that way, leave a mark, and respawn if it's too old or off-grid.
 * Each line of particle_step (below) is one of these helpers. */

static void particle_spawn(Particle *p, const Grid *g)
{
    p->x         = (float)(rand() % g->w);
    p->y         = (float)(rand() % g->h);
    p->color_idx = rand() & (N_BANDS - 1);
    p->age       = 0;
    p->max_age   = AGE_MIN_TICKS + rand() % (AGE_MAX_TICKS - AGE_MIN_TICKS);
}

/* Get just the direction the field points at a spot, throwing away how
 * strong it is. We drop the strength because it's huge right by a pole
 * and almost nothing far away — without this, dots near poles would
 * shoot off the screen while distant ones barely crawled. */
static void sample_unit_field_direction(const Scene *s, float px, float py,
                                          float *out_bx, float *out_by)
{
    field_at(s->poles.active, s->poles.n_active, px, py, out_bx, out_by);
    float m = sqrtf((*out_bx) * (*out_bx) + (*out_by) * (*out_by));
    if (m > VELOCITY_EPSILON) {
        *out_bx /= m;
        *out_by /= m;
    }
}

/* Nudge the dot one small step in the field's direction. No momentum,
 * it just goes where the field points. */
static void advect_particle_euler(Particle *p, float bx, float by,
                                   float dt, int speed)
{
    p->x += bx * (float)speed * dt;
    p->y += by * (float)speed * dt;
    p->age++;
}

/* Light up the cell the dot is sitting on, full brightness. It just
 * overwrites; the gradual fade is handled elsewhere, between hits. */
static void deposit_trail_hit(Scene *s, int cx, int cy, int color_idx)
{
    if (!grid_in_bounds(&s->grid, cx, cy)) return;
    int idx = grid_idx(&s->grid, cx, cy);
    s->buf.glow [idx] = TRAIL_HIT_INTENSITY;
    s->buf.color[idx] = (uint8_t)color_idx;
}

/* Is this dot done? True if it's too old or has drifted off the grid;
 * either way it gets respawned. */
static bool particle_is_expired(const Particle *p, const Grid *g)
{
    return p->age >= p->max_age
        || p->x < 0.0f || p->x >= (float)g->w
        || p->y < 0.0f || p->y >= (float)g->h;
}

static void particle_step(Scene *s, Particle *p, float dt, int speed)
{
    float bx, by;
    sample_unit_field_direction(s, p->x, p->y, &bx, &by);
    advect_particle_euler      (p, bx, by, dt, speed);
    deposit_trail_hit          (s, (int)p->x, (int)p->y, p->color_idx);
    if (particle_is_expired(p, &s->grid))
        particle_spawn         (p, &s->grid);
}

/* ── tick pipeline ───────────────────────────────────────────────── *
 * One simulation tick: if paused, do nothing; otherwise fade the trails
 * a bit, move time forward, recompute where the poles are, and step
 * every dot. Each line of scene_tick (below) is one of these helpers. */

static void decay_trail_glow(RenderBuffers *b, int n, float dt)
{
    float decay = expf(-TRAIL_GLOW_DECAY * dt);
    for (int i = 0; i < n; i++) b->glow[i] *= decay;
}

static void advance_field_time(SimState *sim, float dt)
{
    sim->field_time += dt;
}

static void refresh_active_poles(PoleField *pf, float sim_time)
{
    pf->n_active = compute_active_poles(pf->base, pf->n_base,
                                          pf->active, sim_time);
}

static void step_all_particles(Scene *s, float dt)
{
    int spd = s->ctrl.speed;
    for (int i = 0; i < s->particles.n; i++)
        particle_step(s, &s->particles.pool[i], dt, spd);
}

/* ── reset / init pipeline ───────────────────────────────────────── */

static void apply_grid_dimensions(Grid *g, int w, int h)
{
    g->w           = w;
    g->h           = h;
    g->total_cells = w * h;
}

static void reset_sim_state(SimState *sim)
{
    sim->field_time = 0.0f;
}

static void spawn_all_particles(Particles *ps, const Grid *g)
{
    ps->n = N_PARTICLES_DEF;
    for (int i = 0; i < ps->n; i++)
        particle_spawn(&ps->pool[i], g);
}

static void install_pattern_poles(PoleField *pf, Pattern p, const Grid *g)
{
    pf->n_base   = pattern_init_poles(pf->base, MAX_POLES, p, g->w, g->h);
    pf->n_active = 0;
}

static void scene_reset(Scene *s, int w, int h)
{
    apply_grid_dimensions    (&s->grid, w, h);
    reset_sim_state          (&s->sim);
    buffers_clear            (&s->buf, s->grid.total_cells);
    install_pattern_poles    (&s->poles, s->ctrl.current_pattern, &s->grid);
    spawn_all_particles      (&s->particles, &s->grid);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->ctrl.paused            = false;
    s->ctrl.speed             = SPEED_DEF;
    s->ctrl.current_theme     = 0;
    s->ctrl.current_glyph_set = 2;        /* MEDIUM */
    s->ctrl.current_pattern   = PATTERN_DIPOLE;
    scene_reset(s, w, h);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;

    decay_trail_glow      (&s->buf, s->grid.total_cells, dt);
    advance_field_time    (&s->sim, dt);
    refresh_active_poles  (&s->poles, s->sim.field_time);
    step_all_particles    (s, dt);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

/*
 * Screen — just the terminal's current size, looked up fresh whenever
 * the window is resized. That's all we track here; ncurses keeps the
 * rest of its own state. We need the size only to centre the field and
 * place the text rows.
 */
typedef struct {
    int cols;   /* terminal width,  in characters */
    int rows;   /* terminal height, in characters */
} Screen;

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
}
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* ── scene_draw pipeline ─────────────────────────────────────────── *
 * Drawn in two passes: first the trails (a character per cell, picked by
 * how bright the trail is and coloured by the theme), then the N/S pole
 * markers on top in their fixed red/blue. */

/*
 * CellDraw — a little note saying "here's what to put in this cell":
 * which colour, any bold, which character, or skip it entirely. Working
 * out the note is kept separate from actually drawing it, so the choice
 * logic stays clean and there's just one spot that talks to ncurses.
 */
typedef struct {
    int  pair;
    int  attr;
    char glyph;
    bool skip;
} CellDraw;

/* Find the top-left corner to start drawing the field so it sits
 * centred, leaving room for the text rows at top and bottom. */
static void compute_centred_origin(const Grid *g, int cols, int rows,
                                    int *out_gx0, int *out_gy0)
{
    int gx0 = (cols - g->w) / 2;
    int gy0 = ((rows - HUD_BAND_RESERVED_ROWS) - g->h) / 2 + HUD_TOP_ROWS;
    if (gx0 < 0)            gx0 = 0;
    if (gy0 < HUD_TOP_ROWS) gy0 = HUD_TOP_ROWS;
    *out_gx0 = gx0;
    *out_gy0 = gy0;
}

/* Decide what to draw in one trail cell: brighter trails get heavier
 * characters; very faint ones get skipped. */
static CellDraw cell_density_band(uint8_t band, float glow, const GlyphSet *gs)
{
    int pair = PAIR_TRAIL_BASE + (band & (N_BANDS - 1));
    if (glow > GLYPH_HIGH_THRESH) return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = gs->high };
    if (glow > GLYPH_MID_THRESH ) return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = gs->mid  };
    if (glow > GLOW_THRESHOLD   ) return (CellDraw){ .pair = pair, .attr = A_NORMAL, .glyph = gs->low  };
    return (CellDraw){ .skip = true };
}

/* Actually put one trail character on screen. The only place the trail
 * layer talks to ncurses. */
static void paint_cell(int sy, int sx, CellDraw c)
{
    if (c.skip) return;
    attron (COLOR_PAIR(c.pair) | c.attr);
    mvaddch(sy, sx, (chtype)(unsigned char)c.glyph);
    attroff(COLOR_PAIR(c.pair) | c.attr);
}

/* First pass: draw all the trails. */
static void draw_trail_layer(const Scene *s, int gx0, int gy0, int cols, int rows,
                              const GlyphSet *gs)
{
    const Grid          *g = &s->grid;
    const RenderBuffers *b = &s->buf;
    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            int idx = grid_idx(g, x, y);
            paint_cell(sy, sx, cell_density_band(b->color[idx], b->glow[idx], gs));
        }
    }
}

/* Second pass: stamp a red 'N' or blue 'S' on top of the trails at each
 * pole, so you can always see where the magnets are. */
static void draw_pole_layer(const PoleField *pf, int gx0, int gy0,
                              int cols, int rows)
{
    for (int i = 0; i < pf->n_active; i++) {
        int sx = gx0 + (int)pf->active[i].x;
        int sy = gy0 + (int)pf->active[i].y;
        if (sx < 0 || sx >= cols) continue;
        if (sy < 0 || sy >= rows) continue;
        int   pair  = (pf->active[i].polarity > 0) ? PAIR_NORTH : PAIR_SOUTH;
        char  glyph = (pf->active[i].polarity > 0) ? 'N' : 'S';
        attron (COLOR_PAIR(pair) | A_BOLD);
        mvaddch(sy, sx, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

static const GlyphSet *active_glyph_set(const Scene *s)
{
    int idx = s->ctrl.current_glyph_set;
    if (idx < 0 || idx >= N_GLYPH_SETS) idx = 0;
    return &glyph_sets[idx];
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    int gx0, gy0;
    compute_centred_origin(&s->grid, cols, rows, &gx0, &gy0);
    const GlyphSet *gs = active_glyph_set(s);
    draw_trail_layer(s, gx0, gy0, cols, rows, gs);
    draw_pole_layer (&s->poles, gx0, gy0, cols, rows);
}

/* ── HUD draw pipeline ───────────────────────────────────────────── *
 * The text around the field. The top rows show what's going on — current
 * pattern and where you are in the list, settings, and a small legend.
 * The bottom row just lists the keys. draw_hud_status_line builds row 1
 * out of several small pieces, one per labelled field. */

static void draw_hud_state_bar(const Screen *sc, const Scene *s,
                                double fps, int sim_fps)
{
    const Controls *c = &s->ctrl;
    const char *state_str = c->paused ? "PAUSED  "
                                      : pattern_name(c->current_pattern);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  speed:%-3d ",
             fps, sim_fps, state_str,
             (int)c->current_pattern + 1, N_PATTERNS,
             c->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void draw_hud_title(void)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " MAGNETIC FIELDS ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* ── row 1 segment drawers ───────────────────────────────────────── *
 * Row 1 is laid out piece by piece, left to right. Each piece draws its
 * bit of text and hands back where the next piece should start. */

static int draw_status_pattern_field(int row, int x, Pattern p)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " pattern:%-9s ", pattern_name(p));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_PATTERN_FIELD_W;
}

static int draw_status_theme_field(int row, int x, int theme_idx)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " theme:%-8s ", themes[theme_idx].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_THEME_FIELD_W;
}

static int draw_status_palette_label(int row, int x)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    return x + HUD_PALETTE_LABEL_W;
}

static int draw_palette_swatch(int row, int x)
{
    for (int i = 0; i < N_BANDS; i++) {
        int pair = PAIR_TRAIL_BASE + i;
        attron (COLOR_PAIR(pair) | A_BOLD);
        mvaddch(row, x, '#');
        attroff(COLOR_PAIR(pair) | A_BOLD);
        x++;
    }
    return x;
}

/* Show how many north and south poles the current pattern has, in the
 * matching red/blue, so you can check the count at a glance. */
static int draw_status_pole_counts(int row, int x, const PoleField *pf)
{
    int n_north = 0, n_south = 0;
    for (int i = 0; i < pf->n_base; i++) {
        if (pf->base[i].polarity > 0) n_north++; else n_south++;
    }
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  ");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 2;
    attron (COLOR_PAIR(PAIR_NORTH) | A_BOLD);
    mvprintw(row, x, "N:%d", n_north);
    attroff(COLOR_PAIR(PAIR_NORTH) | A_BOLD);
    x += (n_north < 10 ? 3 : 4);
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, " ");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 1;
    attron (COLOR_PAIR(PAIR_SOUTH) | A_BOLD);
    mvprintw(row, x, "S:%d", n_south);
    attroff(COLOR_PAIR(PAIR_SOUTH) | A_BOLD);
    x += (n_south < 10 ? 3 : 4);
    return x;
}

/* Last piece of the row: how many particles are running and how big the
 * field is. */
static void draw_status_sim_counts(int row, int x, const Scene *s)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  parts:%d  map:%dx%d ",
             s->particles.n, s->grid.w, s->grid.h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

static void draw_hud_status_line(const Scene *s)
{
    const Controls *c = &s->ctrl;
    int x = HUD_LEFT_MARGIN;
    x = draw_status_pattern_field (1, x, c->current_pattern);
    x = draw_status_theme_field   (1, x, c->current_theme);
    x = draw_status_palette_label (1, x);
    x = draw_palette_swatch       (1, x);
    x = draw_status_pole_counts   (1, x, &s->poles);
        draw_status_sim_counts    (1, x, s);
}

/* On the right end of row 1, show the three characters of the current
 * glyph set so you can preview them before switching with g/G. Kept on
 * the right so it doesn't bump into the counts on the left. */
static void draw_hud_glyph_indicator(const Scene *s, const Screen *sc)
{
    const GlyphSet *gs = active_glyph_set(s);
    char buf[32];
    snprintf(buf, sizeof buf, " glyph:%-7s [%c%c%c] ",
             gs->name, gs->low, gs->mid, gs->high);
    int gx = sc->cols - (int)strlen(buf);
    if (gx < 0) gx = 0;
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, gx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* ── row 2 legend segment drawers ────────────────────────────────── *
 * Row 2 explains what you're looking at: on the left, what the trails
 * mean; on the right, what N and S stand for, drawn in the same red/blue
 * as the markers on the field. */

/* Left half of the legend: a note about the trails. */
static void draw_legend_trail_tiers(int row, int x)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, " trails: field-lines (low -> mid -> high)   ");
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Draw one legend entry: the coloured letter, then its label. Returns
 * where the next entry should start. */
static int draw_one_pole_marker(int row, int x,
                                  char glyph, int marker_pair,
                                  const char *description)
{
    attron (COLOR_PAIR(marker_pair) | A_BOLD);
    mvaddch(row, x, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(marker_pair) | A_BOLD);
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x + 1, "%s", description);
    attroff(COLOR_PAIR(PAIR_HUD));
    return x + 1 + (int)strlen(description);
}

/* Right half of the legend: the N and S key. */
static void draw_legend_pole_markers(int row, int x)
{
    x = draw_one_pole_marker(row, x, 'N', PAIR_NORTH, ": north pole   ");
        draw_one_pole_marker(row, x, 'S', PAIR_SOUTH, ": south pole ");
}

/* Row 2: the full legend, both halves together. */
static void draw_hud_legend(void)
{
    draw_legend_trail_tiers (2, HUD_LEFT_MARGIN);
    draw_legend_pole_markers(2, HUD_LEFT_MARGIN + 45);
}

/* Bottom row: the list of keys you can press. */
static void draw_bottom_hint(const Screen *sc)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  g/G:glyph  t/T:theme  r:reset  spc:pause  +/-:speed  ]/[:Hz  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw              (s, sc->cols, sc->rows);
    draw_hud_state_bar      (sc, s, fps, sim_fps);
    draw_hud_title          ();
    draw_hud_status_line    (s);
    draw_hud_glyph_indicator(s, sc);
    draw_hud_legend         ();
    draw_bottom_hint        (sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

/*
 * App — the whole program, one global so the signal handlers can reach
 * it. It holds the scene and the screen, plus the tick rate and chosen
 * field size, and two flags the signal handlers set. The handlers do
 * nothing but flip a flag; the main loop notices and does the real work
 * (quit, resize) on its own time, which is the safe way to handle
 * signals (Stevens & Rago, "Advanced Programming in the UNIX
 * Environment", ch. 10).
 */
typedef struct {
    Scene                 scene;   /* the simulation                    */
    Screen                screen;  /* current terminal size             */

    int                   sim_fps; /* ticks per second; '[' and ']' change it */
    int                   map_w;   /* chosen field width                */
    int                   map_h;   /* chosen field height               */

    /* These are written from signal handlers, so they need both
     * qualifiers: sig_atomic_t so the write can't be seen half-done,
     * and volatile so the loop actually re-reads them each time instead
     * of caching a stale value. Drop either one and it can break. */
    volatile sig_atomic_t running;       /* 0 means quit the main loop  */
    volatile sig_atomic_t need_resize;   /* 1 means the window changed size */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS;
    if (mw < 16) mw = 16;
    if (mh < 8)  mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

static void app_do_resize(App *app)
{
    screen_resize    (&app->screen);
    app_pick_map_size(app);
    scene_reset      (&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* ── keyboard handlers ──────────────────────────────────────────────── *
 * One small function per key action; app_handle_key just routes to them. */

/* '+' doubles the speed, '-' halves it, kept within bounds. */
static void bump_speed_geometric(Controls *c, int dir)
{
    if (dir > 0) {
        if (c->speed < SPEED_MAX) c->speed *= 2;
        if (c->speed > SPEED_MAX) c->speed = SPEED_MAX;
    } else {
        c->speed /= 2;
        if (c->speed < SPEED_MIN) c->speed = SPEED_MIN;
    }
}

/* '[' and ']' step the tick rate down and up, kept within bounds. */
static void bump_sim_fps(App *app, int delta)
{
    app->sim_fps += delta;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

static void cycle_theme(Controls *c, int dir)
{
    c->current_theme = (c->current_theme + dir + N_THEMES) % N_THEMES;
    theme_apply(c->current_theme);
}

static void cycle_glyph_set(Controls *c, int dir)
{
    c->current_glyph_set =
        (c->current_glyph_set + dir + N_GLYPH_SETS) % N_GLYPH_SETS;
}

/* Move to the next/previous pattern and restart the scene, because each
 * pattern needs its poles laid out fresh. */
static void cycle_pattern(App *app, int dir)
{
    Controls *c = &app->scene.ctrl;
    c->current_pattern = (Pattern)(
        ((int)c->current_pattern + dir + N_PATTERNS) % N_PATTERNS);
    scene_reset(&app->scene, app->map_w, app->map_h);
}

static bool app_handle_key(App *app, int ch)
{
    Controls *c = &app->scene.ctrl;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           c->paused = !c->paused;                            break;
    case 'r': case 'R': scene_reset(&app->scene, app->map_w, app->map_h);  break;
    case '=': case '+': bump_speed_geometric(c, +1);                       break;
    case '-':           bump_speed_geometric(c, -1);                       break;
    case ']':           bump_sim_fps(app, +SIM_FPS_STEP);                  break;
    case '[':           bump_sim_fps(app, -SIM_FPS_STEP);                  break;
    case 't':           cycle_theme    (c,   +1);                          break;
    case 'T':           cycle_theme    (c,   -1);                          break;
    case 'g':           cycle_glyph_set(c,   +1);                          break;
    case 'G':           cycle_glyph_set(c,   -1);                          break;
    case 'n': case 'N': cycle_pattern  (app, +1);                          break;
    case 'p': case 'P': cycle_pattern  (app, -1);                          break;
    default: break;
    }
    return true;
}

/* ── main-loop helpers ──────────────────────────────────────────────── */

static void install_signal_handlers(void)
{
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* How much real time has passed since last frame, capped so a long
 * stall doesn't make the sim lurch. */
static int64_t advance_frame_clock(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > DT_MAX_NS) dt = DT_MAX_NS;
    return dt;
}

/* Run as many fixed-size sim steps as the elapsed time has earned, so
 * the sim runs at a steady rate no matter the frame rate (Glenn
 * Fiedler, "Fix Your Timestep"). */
static void simulate_pending_ticks(App *app, int64_t *sim_accum,
                                    int64_t tick_ns, float dt_sec)
{
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* Every half second or so, work out the frames-per-second figure to
 * show; otherwise leave it as it was. */
static double maybe_update_fps_counter(int64_t *fps_accum,
                                        int *frame_count,
                                        double previous)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return previous;
    double fps = (double)(*frame_count) /
                  ((double)(*fps_accum) / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
    return fps;
}

/* Sleep off whatever time is left so we don't draw faster than the
 * target frame rate. */
static void cap_frame_rate(int64_t work_done_ns, int target_fps)
{
    int64_t budget = NS_PER_SEC / target_fps;
    clock_sleep_ns(budget - work_done_ns);
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    install_signal_handlers();

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init      (&app->screen);
    app_pick_map_size(app);
    scene_init       (&app->scene, app->map_w, app->map_h);

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

        int64_t dt      = advance_frame_clock(&frame_time);
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        simulate_pending_ticks(app, &sim_accum, tick_ns, dt_sec);

        frame_count++;
        fps_accum  += dt;
        fps_display = maybe_update_fps_counter(&fps_accum, &frame_count, fps_display);

        cap_frame_rate((clock_ns() - frame_time) + dt, FRAME_CAP_FPS);

        screen_draw   (&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
