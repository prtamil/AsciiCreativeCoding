/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * flow_field_particles.c
 *
 * Drop a few hundred dust specks onto an invisible "wind" and watch
 * them trace it. Each of the 30 wind patterns is written out as a
 * plain formula (no random noise) — swirls, magnets, waves, and the
 * famous phase-plane pictures from dynamical-systems textbooks.
 * Bright '@' marks the special points each wind spins around.
 *
 * Sister files (same particle engine, different wind source):
 *   ./perin_noise_flow_showcase.c — wind comes from Perlin noise.
 *   ./curl_noise_vector_field.c   — swirl-only noise; nothing converges.
 *
 * Where the patterns come from:
 *   Strogatz, "Nonlinear Dynamics and Chaos" — the textbook tour of
 *     saddles, spirals, limit cycles, and bifurcations seen here.
 *   Guckenheimer & Holmes — the rigorous normal forms for Duffing,
 *     Van der Pol, Hopf, Hamilton.
 *   Hénon (1976), Comm. Math. Phys. 50:69 — the (a,b)=(1.4,0.3) map.
 *   von Kármán (1911) — the alternating vortex street (CHAIN, WAKE).
 *   Bourke ASCII grey-scale ramp — the low/mid/high glyph idea.
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

    /* How many dust specks. The pool is sized big; we only switch on 256. */
    MAX_PARTICLES     = 1024,
    N_PARTICLES_DEF   =  256,

    /* How many frames a speck lives before it's recycled. Each speck
     * gets a random lifetime in this range so they don't all vanish
     * and respawn together in one ugly blink. */
    AGE_MIN_TICKS     =  60,
    AGE_MAX_TICKS     = 360,

    /* How fast specks move, in cells per second. */
    SPEED_MIN         =   1,
    SPEED_DEF         =   8,
    SPEED_MAX         =  64,

    /* Room for the most crowded pattern (TURBULENT uses 20). */
    MAX_ATTRACTORS    =  32,

    /* Colour-pair slots. HUD/HINT slots are reserved project-wide. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_TRAIL_BASE   =   3,    /* the 4 trail colours start here */
    PAIR_ATTRACT      =   7,    /* the bright '@' marker colour   */
};

#define TRAIL_GLOW_DECAY    0.6f    /* how fast trails fade out */
#define GLOW_THRESHOLD      0.05f   /* dimmer than this and a cell shows nothing */
#define VELOCITY_EPSILON    1e-6f   /* basically-zero wind: don't try to normalise it */
#define TRAIL_HIT_INTENSITY 1.0f    /* brightness a speck stamps onto its cell */

#define DRIFT_RATE          0.2f    /* how fast the '@' points slowly wander (rad/s) */

/* How strong each kind of wind pushes. The speck's step is normalised
 * to one unit anyway, so for single-source winds these numbers barely
 * matter — they mostly decide which source wins when several add up. */
#define VORTEX_STRENGTH     80.0f
#define MAGNET_STRENGTH     50.0f
#define TURBULENT_STRENGTH   6.0f
#define WAVE_FREQ           0.15f
#define SADDLE_RATE         0.05f
#define EPS_R2              4.0f    /* tiny cushion so we never divide by zero at a source */

/* SPIRAL — a swirl that also pulls inward, so specks corkscrew toward
 * the middle. */
#define SPIRAL_STRENGTH     80.0f
#define SPIRAL_INFLOW       20.0f

/* RADIAL — everything pushed straight out from the centre. */
#define RADIAL_RATE          0.5f

/* SHEAR — a flat horizontal wind that's faster the further up or down
 * you go (like layers sliding past each other). */
#define SHEAR_RATE           0.3f

/* RIPPLE — rings of wind spreading out from the centre. */
#define RIPPLE_WAVENUMBER    0.3f   /* how tightly packed the rings are */
#define RIPPLE_FREQUENCY     1.5f   /* how fast they pulse              */
#define RIPPLE_AMPLITUDE     5.0f

/* CIRCULAR — gentle even spin around the centre. */
#define CIRCULAR_RATE        0.3f

/* CHAIN — a row of swirls that spin in alternating directions. */
#define CHAIN_STRENGTH      50.0f
#define CHAIN_COUNT          6
#define CHAIN_ZIGZAG_Y       4.0f   /* nudge each swirl up/down so the row isn't a straight line */

/* GRID — a 3x3 checkerboard of swirls, neighbours spinning opposite ways. */
#define GRID_STRENGTH       30.0f
#define GRID_COUNT           3

/* GALAXY — a central swirl with brighter spiral arms. */
#define GALAXY_STRENGTH     80.0f
#define GALAXY_ARM_AMP       0.4f   /* how much the arms stand out (0 = no arms) */
#define GALAXY_ARM_PITCH     0.15f  /* how tightly the arms wind up            */
#define GALAXY_INFLOW       20.0f

/* DUFFING — oscillator with two resting spots and a divide between them. */
#define DUFFING_DAMPING      0.2f
#define DUFFING_SCALE        0.05f   /* shrinks screen coords into the math's range */

/* VANDERPOL — oscillator that settles onto one steady loop. */
#define VANDERPOL_MU         1.5f
#define VANDERPOL_SCALE      0.07f

/* PENDULUM — the classic swing picture: small wiggles vs. full spins. */
#define PENDULUM_SCALE       0.08f

/* LOTKA — predator-prey cycles (foxes and rabbits chasing each other). */
#define LOTKA_ALPHA          1.0f
#define LOTKA_BETA           1.0f
#define LOTKA_GAMMA          1.0f
#define LOTKA_DELTA          1.0f
#define LOTKA_SCALE          0.012f
#define LOTKA_OFFSET         1.0f

/* HOPF — a steady loop that appears as the wind strengthens. */
#define HOPF_MU              1.0f
#define HOPF_SCALE           0.06f

/* NEWTON — the root-finding flow for z^3 = 1; three basins meet at a
 * jagged border. */
#define NEWTON_SCALE         0.04f

/* JET — pipe flow: fast down the middle, still at the walls. */
#define JET_STRENGTH         5.0f

/* HURRICANE — a swirl with inflow and a calm eye in the centre. */
#define HURRICANE_RADIUS    15.0f
#define HURRICANE_STRENGTH  10.0f
#define HURRICANE_INFLOW     3.0f

/* HAMILTON — double-well flow shaped like a figure eight. */
#define HAMILTON_SCALE       0.07f

/* WAKE — steady wind blowing past an obstacle, leaving swirls behind it. */
#define WAKE_FREESTREAM      3.0f
#define WAKE_OBSTACLE_REL    0.30f   /* obstacle sits 30% across the map */
#define WAKE_OBSTACLE_GAP    8.0f    /* gap before the first trailing swirl */
#define WAKE_ZIGZAG_AMP      3.0f    /* swirls staggered above/below the line */
#define WAKE_VORTEX_COUNT    5
#define WAKE_VORTEX_STR     30.0f

/* STANDING — a wave that stays put and pulses in place. */
#define STANDING_K           0.3f
#define STANDING_OMEGA       1.0f
#define STANDING_AMP         5.0f

/* PLANE — a wave marching steadily in one direction. */
#define PLANE_KX             0.2f
#define PLANE_KY             0.1f
#define PLANE_OMEGA          0.7f
#define PLANE_AMP            5.0f

/* QUADPOLE — four poles at the corners of a square, signs alternating. */
#define QUADPOLE_STRENGTH   30.0f
#define QUADPOLE_SEPARATION  0.18f

/* GRADIENT — everything rolls downhill into one central dip. */
#define GRADIENT_SIGMA      10.0f
#define GRADIENT_STRENGTH   80.0f

/* EDDY — one soft swirl that fades smoothly with distance, no sharp centre. */
#define EDDY_SIGMA          12.0f
#define EDDY_STRENGTH        2.0f

/* PITCHFORK — one resting spot splits into two as the wind strengthens. */
#define PITCHFORK_MU         1.0f
#define PITCHFORK_SCALE      0.05f

/* HENON — flow version of the famous strange-attractor map. */
#define HENON_A              1.4f
#define HENON_B              0.3f
#define HENON_SCALE          0.03f

/* Trails come in 4 colour shades, dim to bright. */
#define N_BANDS              4

#define GLYPH_HIGH_THRESH   0.65f   /* brightest trails use the "high" glyph */
#define GLYPH_MID_THRESH    0.30f   /* medium trails use the "mid" glyph     */

/* Where the on-screen info sits. Top three rows show status; the
 * bottom row lists the keys; the map fills the middle. */
#define HUD_TOP_ROWS             3
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1
#define HUD_PATTERN_FIELD_W      20
#define HUD_THEME_FIELD_W        17
#define HUD_PALETTE_LABEL_W       9

/*
 * Pattern — which of the 30 winds is showing. The user cycles through
 * these with n/p. Roughly: swirls, magnets/poles, travelling waves,
 * and the textbook oscillator pictures.
 */
typedef enum {
    PATTERN_VORTICES  =  0,
    PATTERN_WAVE      =  1,
    PATTERN_SADDLE    =  2,
    PATTERN_MAGNET    =  3,
    PATTERN_TURBULENT =  4,
    PATTERN_SPIRAL    =  5,
    PATTERN_RADIAL    =  6,
    PATTERN_SHEAR     =  7,
    PATTERN_RIPPLE    =  8,
    PATTERN_CIRCULAR  =  9,
    PATTERN_CHAIN     = 10,
    PATTERN_GRID      = 11,
    PATTERN_GALAXY    = 12,
    PATTERN_DUFFING   = 13,
    PATTERN_VANDERPOL = 14,
    PATTERN_PENDULUM  = 15,
    PATTERN_LOTKA     = 16,
    PATTERN_HOPF      = 17,
    PATTERN_NEWTON    = 18,
    PATTERN_JET       = 19,
    PATTERN_HURRICANE = 20,
    PATTERN_HAMILTON  = 21,
    PATTERN_WAKE      = 22,
    PATTERN_STANDING  = 23,
    PATTERN_PLANE     = 24,
    PATTERN_QUADPOLE  = 25,
    PATTERN_GRADIENT  = 26,
    PATTERN_EDDY      = 27,
    PATTERN_PITCHFORK = 28,
    PATTERN_HENON     = 29,
    N_PATTERNS        = 30,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_VORTICES:  return "VORTICES ";
    case PATTERN_WAVE:      return "WAVE     ";
    case PATTERN_SADDLE:    return "SADDLE   ";
    case PATTERN_MAGNET:    return "MAGNET   ";
    case PATTERN_TURBULENT: return "TURBULENT";
    case PATTERN_SPIRAL:    return "SPIRAL   ";
    case PATTERN_RADIAL:    return "RADIAL   ";
    case PATTERN_SHEAR:     return "SHEAR    ";
    case PATTERN_RIPPLE:    return "RIPPLE   ";
    case PATTERN_CIRCULAR:  return "CIRCULAR ";
    case PATTERN_CHAIN:     return "CHAIN    ";
    case PATTERN_GRID:      return "GRID     ";
    case PATTERN_GALAXY:    return "GALAXY   ";
    case PATTERN_DUFFING:   return "DUFFING  ";
    case PATTERN_VANDERPOL: return "VANDERPOL";
    case PATTERN_PENDULUM:  return "PENDULUM ";
    case PATTERN_LOTKA:     return "LOTKA    ";
    case PATTERN_HOPF:      return "HOPF     ";
    case PATTERN_NEWTON:    return "NEWTON   ";
    case PATTERN_JET:       return "JET      ";
    case PATTERN_HURRICANE: return "HURRICANE";
    case PATTERN_HAMILTON:  return "HAMILTON ";
    case PATTERN_WAKE:      return "WAKE     ";
    case PATTERN_STANDING:  return "STANDING ";
    case PATTERN_PLANE:     return "PLANE    ";
    case PATTERN_QUADPOLE:  return "QUADPOLE ";
    case PATTERN_GRADIENT:  return "GRADIENT ";
    case PATTERN_EDDY:      return "EDDY     ";
    case PATTERN_PITCHFORK: return "PITCHFORK";
    case PATTERN_HENON:     return "HENON    ";
    default:                return "?        ";
    }
}

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* If the program stalls (e.g. you drag the window), one frame's dt
 * could be huge. Cap it so we don't try to simulate a thousand steps
 * at once and lock up. See Glenn Fiedler, "Fix Your Timestep". */
#define DT_MAX_NS       (100 * NS_PER_MS)
#define FRAME_CAP_FPS   60

/*
 * Theme — one colour scheme. Four trail shades (dim to bright) plus
 * one bright "flash" colour reserved for the '@' markers, so the
 * markers stay readable no matter which theme is on. Ten of these
 * live in themes[]; the user flips through them with t/T.
 *
 * Each speck is assigned one of the four shades when it's born and
 * keeps it for life, so a trail reads as one consistent colour even
 * where it crosses cells another speck painted.
 *
 * The numbers are xterm-256 colour codes, not red/green/blue values.
 * On old terminals with fewer colours, theme_apply() swaps in a
 * basic 8-colour set instead.
 */
typedef struct {
    const char *name;             /* short label shown in the HUD              */
    short       trail[N_BANDS];   /* the 4 trail shades, dim (0) to bright (3) */
    short       flash;            /* colour of the '@' markers                 */
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
 * GlyphSet — the three characters used for faint, medium, and strong
 * trails. Five sets go from thin to chunky; the user picks one with
 * g/G. Glyph choice is kept separate from theme so any look can pair
 * with any colour scheme. The three steps line up with the
 * GLYPH_*_THRESH brightness cutoffs in §1.
 */
typedef struct {
    const char *name;       /* short label shown in the HUD          */
    char        low;        /* used for the faintest visible trails  */
    char        mid;        /* used for medium trails                */
    char        high;       /* used for the brightest trails         */
} GlyphSet;

#define N_GLYPH_SETS 5

static const GlyphSet glyph_sets[N_GLYPH_SETS] = {
    /*  name      low  mid  high */
    { "SLIM",    '.', '\'', ':' },
    { "LIGHT",   '.', '*',  '+' },
    { "MEDIUM",  '.', '*',  '#' },   /* the default */
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
        init_pair(PAIR_ATTRACT, t->flash, -1);
    } else {
        static const short fallback[N_BANDS] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < N_BANDS; i++)
            init_pair(PAIR_TRAIL_BASE + i, fallback[i], -1);
        init_pair(PAIR_ATTRACT, COLOR_YELLOW, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  fields — 30 closed-form vector fields                              */
/* ===================================================================== */

/*
 * Attractor — one special point in a wind: a swirl centre, a magnet
 * pole, an oscillator's resting spot, or just a place to draw an '@'.
 * Set up when the pattern loads and never changed during play.
 *
 * Besides where it sits, it also carries a slow wandering orbit. Each
 * frame the point drifts a little around its home spot, tracing a
 * small ellipse (radii ox, oy). Giving each point its own ellipse and
 * its own starting phase keeps busy multi-swirl patterns from looking
 * frozen — they breathe instead. The actual wandered position is
 * worked out each frame into an ActiveAttractor (below).
 */
typedef struct {
    float bx, by;     /* home position (centre of the wander ellipse)        */
    float ox, oy;     /* how far it wanders left/right and up/down           */
    float phase;      /* its own starting point in the wander, so points     */
                      /* don't all sway together                             */
    float strength;   /* how hard it pushes; 0 = just a marker, no push      */
    int   sign;       /* +1 = pull in / spin one way, -1 = push out / other  */
} Attractor;

/*
 * ActiveAttractor — an Attractor's actual spot for this one frame,
 * after the wander is applied. The wind formulas and the renderer
 * both read this. We compute it once per frame for the handful of
 * points we have, instead of redoing the sin/cos wander math inside
 * the speck loop (which runs thousands of times a frame).
 */
typedef struct {
    float x, y;       /* this frame's position                          */
    float strength;   /* copied from the Attractor                      */
    int   sign;       /* copied from the Attractor                      */
} ActiveAttractor;

/*
 * field_vortices — add up the push from every swirl. Shared by
 * VORTICES, TURBULENT, CHAIN, and GRID; those only differ in where
 * the swirls sit. Each swirl pushes sideways (round it) and weaker
 * the further away you are.
 */
static void field_vortices(const ActiveAttractor *att, int n_att,
                           float x, float y,
                           float *out_vx, float *out_vy)
{
    float vx = 0.0f, vy = 0.0f;
    for (int i = 0; i < n_att; i++) {
        float dx = x - att[i].x;
        float dy = y - att[i].y;
        float r2 = dx * dx + dy * dy + EPS_R2;
        float w  = att[i].strength / r2 * (float)att[i].sign;
        vx += -dy * w;
        vy +=  dx * w;
    }
    *out_vx = vx;
    *out_vy = vy;
}

/* field_wave — wind that rolls in smooth crisscrossing waves. */
static void field_wave(float x, float y, float t,
                       float *out_vx, float *out_vy)
{
    *out_vx = sinf(y * WAVE_FREQ + t       ) * 5.0f;
    *out_vy = cosf(x * WAVE_FREQ - t * 0.7f) * 5.0f;
}

/* field_saddle — pushes out sideways and pulls in vertically, so the
 * centre is a "pass" specks rush through. */
static void field_saddle(float x, float y, float cx, float cy,
                         float *out_vx, float *out_vy)
{
    *out_vx =  (x - cx) * SADDLE_RATE;
    *out_vy = -(y - cy) * SADDLE_RATE;
}

/* field_magnet — each pole pulls or pushes straight toward/away,
 * stronger up close, like gravity or a magnet. */
static void field_magnet(const ActiveAttractor *att, int n_att,
                         float x, float y,
                         float *out_vx, float *out_vy)
{
    float vx = 0.0f, vy = 0.0f;
    for (int i = 0; i < n_att; i++) {
        float dx = x - att[i].x;
        float dy = y - att[i].y;
        float r2 = dx * dx + dy * dy + EPS_R2;
        float w  = (float)(-att[i].sign) * att[i].strength / r2;
        vx += dx * w;
        vy += dy * w;
    }
    *out_vx = vx;
    *out_vy = vy;
}

/* field_spiral — a swirl plus a pull inward, so specks corkscrew
 * steadily toward the centre. */
static void field_spiral(const ActiveAttractor *att, int n_att,
                          float x, float y,
                          float *out_vx, float *out_vy)
{
    float vx = 0.0f, vy = 0.0f;
    for (int i = 0; i < n_att; i++) {
        float dx = x - att[i].x;
        float dy = y - att[i].y;
        float r2 = dx * dx + dy * dy + EPS_R2;
        float w  = att[i].strength / r2 * (float)att[i].sign;
        vx += -dy * w;
        vy +=  dx * w;
        vx += -dx * SPIRAL_INFLOW / r2;
        vy += -dy * SPIRAL_INFLOW / r2;
    }
    *out_vx = vx;
    *out_vy = vy;
}

/* field_radial — everything sprays straight out from the centre. */
static void field_radial(float x, float y, float cx, float cy,
                          float *out_vx, float *out_vy)
{
    *out_vx = (x - cx) * RADIAL_RATE;
    *out_vy = (y - cy) * RADIAL_RATE;
}

/* field_shear — flat horizontal wind, faster the further from the
 * middle line; top and bottom slide opposite ways. */
static void field_shear(float x, float y, float cy,
                         float *out_vx, float *out_vy)
{
    (void)x;
    *out_vx = (y - cy) * SHEAR_RATE;
    *out_vy = 0.0f;
}

/* field_ripple — rings of wind spreading out from the centre, like
 * dropping a stone in a pond. */
static void field_ripple(float x, float y, float t, float cx, float cy,
                          float *out_vx, float *out_vy)
{
    float dx = x - cx;
    float dy = y - cy;
    float r  = sqrtf(dx * dx + dy * dy) + 0.001f;
    float wave = sinf(r * RIPPLE_WAVENUMBER - t * RIPPLE_FREQUENCY);
    *out_vx = (dx / r) * wave * RIPPLE_AMPLITUDE;
    *out_vy = (dy / r) * wave * RIPPLE_AMPLITUDE;
}

/* field_circular — even spin around the centre, like a record player. */
static void field_circular(float x, float y, float cx, float cy,
                            float *out_vx, float *out_vy)
{
    *out_vx = -(y - cy) * CIRCULAR_RATE;
    *out_vy =  (x - cx) * CIRCULAR_RATE;
}

/* field_galaxy — a central swirl whose strength ripples around the
 * circle, carving out brighter spiral arms. */
static void field_galaxy(const ActiveAttractor *att, int n_att,
                          float x, float y,
                          float *out_vx, float *out_vy)
{
    float vx = 0.0f, vy = 0.0f;
    for (int i = 0; i < n_att; i++) {
        float dx = x - att[i].x;
        float dy = y - att[i].y;
        float r2 = dx * dx + dy * dy + EPS_R2;
        float r  = sqrtf(r2);
        float theta = atan2f(dy, dx);
        float arm = 1.0f + GALAXY_ARM_AMP * cosf(theta * 2.0f - r * GALAXY_ARM_PITCH);
        float w  = att[i].strength / r2 * (float)att[i].sign * arm;
        vx += -dy * w;
        vy +=  dx * w;
        vx += -dx * GALAXY_INFLOW / r2;
        vy += -dy * GALAXY_INFLOW / r2;
    }
    *out_vx = vx;
    *out_vy = vy;
}

/* field_duffing — the swing-pattern of a spring with two resting
 * spots; specks fall toward one or the other. */
static void field_duffing(float x, float y, float cx, float cy,
                           float *out_vx, float *out_vy)
{
    float xp = (x - cx) * DUFFING_SCALE;
    float yp = (y - cy) * DUFFING_SCALE;
    *out_vx = yp;
    *out_vy = -xp - DUFFING_DAMPING * yp - xp * xp * xp;
}

/* field_van_der_pol — an oscillator that, from anywhere, settles onto
 * one steady repeating loop. */
static void field_van_der_pol(float x, float y, float cx, float cy,
                               float *out_vx, float *out_vy)
{
    float xp = (x - cx) * VANDERPOL_SCALE;
    float yp = (y - cy) * VANDERPOL_SCALE;
    *out_vx = yp;
    *out_vy = VANDERPOL_MU * (1.0f - xp * xp) * yp - xp;
}

/* field_pendulum — a swing: gentle pushes make it rock back and forth,
 * a hard push sends it spinning all the way over. */
static void field_pendulum(float x, float y, float cx, float cy,
                            float *out_vx, float *out_vy)
{
    float xp = (x - cx) * PENDULUM_SCALE;
    float yp = (y - cy) * PENDULUM_SCALE;
    *out_vx = yp;
    *out_vy = -sinf(xp);
}

/* field_lotka — predators and prey rising and falling forever in
 * closed loops (more rabbits, then more foxes, then fewer rabbits...). */
static void field_lotka(float x, float y, float cx, float cy,
                         float *out_vx, float *out_vy)
{
    float xp = (x - cx) * LOTKA_SCALE + LOTKA_OFFSET;
    float yp = (y - cy) * LOTKA_SCALE + LOTKA_OFFSET;
    if (xp < 0.01f) xp = 0.01f;
    if (yp < 0.01f) yp = 0.01f;
    *out_vx = LOTKA_ALPHA * xp - LOTKA_BETA  * xp * yp;
    *out_vy = LOTKA_DELTA * xp * yp - LOTKA_GAMMA * yp;
}

/* field_hopf — specks spiral onto a single steady ring from both
 * inside and outside. */
static void field_hopf(float x, float y, float cx, float cy,
                        float *out_vx, float *out_vy)
{
    float xp = (x - cx) * HOPF_SCALE;
    float yp = (y - cy) * HOPF_SCALE;
    float r2 = xp * xp + yp * yp;
    *out_vx = HOPF_MU * xp - yp - xp * r2;
    *out_vy = xp + HOPF_MU * yp - yp * r2;
}

/* field_newton — the root-finding flow for z^3 = 1: three pull-points
 * whose territories meet along a jagged, fractal border. */
static void field_newton(float x, float y, float cx, float cy,
                          float *out_vx, float *out_vy)
{
    float zx = (x - cx) * NEWTON_SCALE;
    float zy = (y - cy) * NEWTON_SCALE;
    float fz_re = zx * zx * zx - 3.0f * zx * zy * zy - 1.0f;
    float fz_im = 3.0f * zx * zx * zy - zy * zy * zy;
    float fp_re = 3.0f * (zx * zx - zy * zy);
    float fp_im = 6.0f * zx * zy;
    float denom = fp_re * fp_re + fp_im * fp_im + EPS_R2;
    *out_vx = -(fz_re * fp_re + fz_im * fp_im) / denom;
    *out_vy = -(fz_im * fp_re - fz_re * fp_im) / denom;
}

/* field_jet — flow through a pipe: fastest down the middle, dead still
 * at the top and bottom walls. */
static void field_jet(float x, float y, float cy, float h_half,
                       float *out_vx, float *out_vy)
{
    (void)x;
    float dy = (y - cy) / h_half;
    if (dy < -1.0f) dy = -1.0f;
    if (dy >  1.0f) dy =  1.0f;
    *out_vx = JET_STRENGTH * (1.0f - dy * dy);
    *out_vy = 0.0f;
}

/* field_hurricane — a swirl that also draws inward, with the wind
 * peaking in a ring (the eyewall) and dying down in the calm eye. */
static void field_hurricane(float x, float y, float cx, float cy,
                             float *out_vx, float *out_vy)
{
    float dx = x - cx;
    float dy = y - cy;
    float r  = sqrtf(dx * dx + dy * dy);
    float t_norm = r / HURRICANE_RADIUS;
    float envelope = t_norm * expf(-t_norm * t_norm);
    float invr = 1.0f / (r + 1.0f);
    *out_vx = (-dy * HURRICANE_STRENGTH + -dx * HURRICANE_INFLOW) * envelope * invr;
    *out_vy = ( dx * HURRICANE_STRENGTH + -dy * HURRICANE_INFLOW) * envelope * invr;
}

/* field_hamilton — two side-by-side loops that meet in the middle,
 * tracing a figure eight. */
static void field_hamilton(float x, float y, float cx, float cy,
                            float *out_vx, float *out_vy)
{
    float xp = (x - cx) * HAMILTON_SCALE;
    float yp = (y - cy) * HAMILTON_SCALE;
    *out_vx = yp;
    *out_vy = xp - 2.0f * xp * xp * xp;
}

/* field_wake — a steady wind blowing right, plus the alternating
 * swirls that peel off behind the obstacle. */
static void field_wake(const ActiveAttractor *att, int n_att,
                        float x, float y,
                        float *out_vx, float *out_vy)
{
    float vx = WAKE_FREESTREAM, vy = 0.0f;
    for (int i = 0; i < n_att; i++) {
        float dx = x - att[i].x;
        float dy = y - att[i].y;
        float r2 = dx * dx + dy * dy + EPS_R2;
        float w  = att[i].strength / r2 * (float)att[i].sign;
        vx += -dy * w;
        vy +=  dx * w;
    }
    *out_vx = vx;
    *out_vy = vy;
}

/* field_standing — a wave that doesn't travel; the whole pattern just
 * grows and shrinks in place. */
static void field_standing(float x, float y, float t,
                            float *out_vx, float *out_vy)
{
    float temporal = cosf(t * STANDING_OMEGA);
    *out_vx = sinf(x * STANDING_K) * temporal * STANDING_AMP;
    *out_vy = sinf(y * STANDING_K) * temporal * STANDING_AMP;
}

/* field_plane — a wave that marches steadily across the screen in one
 * fixed direction. */
static void field_plane(float x, float y, float t,
                         float *out_vx, float *out_vy)
{
    float phase = PLANE_KX * x + PLANE_KY * y - PLANE_OMEGA * t;
    float kmag = sqrtf(PLANE_KX * PLANE_KX + PLANE_KY * PLANE_KY);
    float wave = sinf(phase) * PLANE_AMP;
    *out_vx = (PLANE_KX / kmag) * wave;
    *out_vy = (PLANE_KY / kmag) * wave;
}

/* field_gradient — there's a dip in the ground at the centre and
 * everything rolls downhill straight into it. */
static void field_gradient(float x, float y, float cx, float cy,
                            float *out_vx, float *out_vy)
{
    float dx = x - cx;
    float dy = y - cy;
    float r2 = dx * dx + dy * dy;
    float sigma2 = GRADIENT_SIGMA * GRADIENT_SIGMA;
    float falloff = expf(-r2 / (2.0f * sigma2));
    *out_vx = -dx / sigma2 * falloff * GRADIENT_STRENGTH;
    *out_vy = -dy / sigma2 * falloff * GRADIENT_STRENGTH;
}

/* field_eddy — one soft swirl that fades out smoothly with distance,
 * with no sharp spike at the middle. */
static void field_eddy(float x, float y, float cx, float cy,
                        float *out_vx, float *out_vy)
{
    float dx = x - cx;
    float dy = y - cy;
    float r2 = dx * dx + dy * dy;
    float falloff = expf(-r2 / (2.0f * EDDY_SIGMA * EDDY_SIGMA));
    *out_vx = -dy * EDDY_STRENGTH * falloff;
    *out_vy =  dx * EDDY_STRENGTH * falloff;
}

/* field_pitchfork — the middle is unstable, so specks fall off to one
 * of two resting spots on either side. */
static void field_pitchfork(float x, float y, float cx, float cy,
                             float *out_vx, float *out_vy)
{
    float xp = (x - cx) * PITCHFORK_SCALE;
    float yp = (y - cy) * PITCHFORK_SCALE;
    *out_vx = PITCHFORK_MU * xp - xp * xp * xp;
    *out_vy = -yp;
}

/* field_henon — a flowing version of Hénon's famous map, which folds
 * specks into a wispy strange attractor. */
static void field_henon(float x, float y, float cx, float cy,
                         float *out_vx, float *out_vy)
{
    float xp = (x - cx) * HENON_SCALE;
    float yp = (y - cy) * HENON_SCALE;
    *out_vx = 1.0f - HENON_A * xp * xp + yp - xp;
    *out_vy = HENON_B * xp - yp;
}

/* ===================================================================== */
/* §6  attractors — pattern-specific source / sink configuration         */
/* ===================================================================== */

/* ── attractor builders ──────────────────────────────────────────── *
 * Tiny helpers so each pattern's setup below stays short and readable. */

/* make_marker — a point that only gets an '@' drawn on it and pushes
 * nothing (strength 0). For patterns whose wind comes from a formula,
 * not from these points. */
static Attractor make_marker(float x, float y)
{
    return (Attractor){
        .bx = x, .by = y,
        .ox = 0.0f, .oy = 0.0f, .phase = 0.0f,
        .strength = 0.0f, .sign = +1,
    };
}

/* make_drifting_vortex — a swirl that slowly wanders around its home
 * spot, so busy patterns keep moving instead of freezing. */
static Attractor make_drifting_vortex(float bx, float by, float ox, float oy,
                                       float phase, float strength, int sign)
{
    return (Attractor){
        .bx = bx, .by = by,
        .ox = ox, .oy = oy,
        .phase = phase,
        .strength = strength, .sign = sign,
    };
}

/* place_centre_marker — just one '@' in the middle. Returns how many
 * points it wrote (1, or 0 if there's no room). */
static int place_centre_marker(Attractor out[], int max_n, float cx, float cy)
{
    if (max_n < 1) return 0;
    out[0] = make_marker(cx, cy);
    return 1;
}

/* ── per-pattern placers ─────────────────────────────────────────── *
 * One per pattern: each lays out that pattern's special points. They
 * all write into out[] (up to max_n) and return how many they wrote.
 * pattern_init_attractors below just picks the right one. */

/* VORTICES — four swirls spinning in alternating directions, set in a
 * square around the centre. */
static int place_vortices_quarters(Attractor out[], int max_n,
                                    float cx, float cy, int w, int h)
{
    float rx = (float)w * 0.25f;
    float ry = (float)h * 0.25f;
    int n = 0;
    for (int i = 0; i < 4 && n < max_n; i++) {
        float ang = (float)i * (float)M_PI * 0.5f;
        out[n++] = make_drifting_vortex(
            cx + rx * cosf(ang), cy + ry * sinf(ang),
            4.0f, 4.0f,                  /* how far it wanders        */
            (float)i * 0.7f,             /* each starts wandering off-beat */
            VORTEX_STRENGTH,
            (i & 1) ? -1 : +1            /* every other one spins back */
        );
    }
    return n;
}

/* MAGNET — two opposite poles set apart left and right, wandering in
 * and out so they breathe toward and away from each other. */
static int place_magnet_dipole(Attractor out[], int max_n,
                                float cx, float cy, int w)
{
    if (max_n < 2) return 0;
    float dx = (float)w * 0.20f;
    out[0] = make_drifting_vortex(cx - dx, cy, 3.0f, 2.0f,
                                    0.0f,         MAGNET_STRENGTH, +1);
    out[1] = make_drifting_vortex(cx + dx, cy, 3.0f, 2.0f,
                                    (float)M_PI,  MAGNET_STRENGTH, -1);
    return 2;
}

/* TURBULENT — 20 small swirls scattered at random, spinning random
 * ways, so the flow looks chaotic. */
static int place_turbulent_field(Attractor out[], int max_n, int w, int h)
{
    int n = 0;
    for (int i = 0; i < 20 && n < max_n; i++) {
        out[n++] = make_drifting_vortex(
            (float)(rand() % w), (float)(rand() % h),
            2.0f, 2.0f,
            (float)i * 0.31f,
            TURBULENT_STRENGTH,
            (rand() & 1) ? +1 : -1
        );
    }
    return n;
}

/* SPIRAL — one strong swirl parked at the centre. */
static int place_spiral_centre(Attractor out[], int max_n, float cx, float cy)
{
    if (max_n < 1) return 0;
    out[0] = make_drifting_vortex(cx, cy, 0.0f, 0.0f, 0.0f,
                                    SPIRAL_STRENGTH, +1);
    return 1;
}

/* GALAXY — one central swirl; the spiral arms are added by the wind
 * formula itself, not by extra points. */
static int place_galaxy_centre(Attractor out[], int max_n, float cx, float cy)
{
    if (max_n < 1) return 0;
    out[0] = make_drifting_vortex(cx, cy, 0.0f, 0.0f, 0.0f,
                                    GALAXY_STRENGTH, +1);
    return 1;
}

/* CHAIN — a row of swirls across the map spinning in alternating
 * directions, staggered up and down a touch so the line isn't dead
 * straight. */
static int place_chain_alternating(Attractor out[], int max_n,
                                    float cy, int w)
{
    int n = 0;
    float spacing = (float)w / (float)(CHAIN_COUNT + 1);
    for (int i = 0; i < CHAIN_COUNT && n < max_n; i++) {
        out[n++] = make_drifting_vortex(
            spacing * (float)(i + 1),
            cy + ((i & 1) ? CHAIN_ZIGZAG_Y : -CHAIN_ZIGZAG_Y),
            2.0f, 2.0f,
            (float)i * 0.5f,
            CHAIN_STRENGTH,
            (i & 1) ? -1 : +1
        );
    }
    return n;
}

/* GRID — a 3x3 checkerboard of swirls; like a checkerboard, neighbours
 * spin opposite ways. */
static int place_grid_checkerboard(Attractor out[], int max_n, int w, int h)
{
    float dx = (float)w / (float)(GRID_COUNT + 1);
    float dy = (float)h / (float)(GRID_COUNT + 1);
    int n = 0, i = 0;
    for (int gy = 0; gy < GRID_COUNT && n < max_n; gy++) {
        for (int gx = 0; gx < GRID_COUNT && n < max_n; gx++) {
            out[n++] = make_drifting_vortex(
                dx * (float)(gx + 1),
                dy * (float)(gy + 1),
                1.5f, 1.5f,
                (float)i * 0.4f,
                GRID_STRENGTH,
                ((gx + gy) & 1) ? -1 : +1
            );
            i++;
        }
    }
    return n;
}

/* WAKE — a marker for the obstacle, plus a line of alternating swirls
 * trailing off behind it. */
static int place_wake_obstacle(Attractor out[], int max_n, float cy, int w)
{
    if (max_n < 1) return 0;
    float obs_x    = (float)w * WAKE_OBSTACLE_REL;
    float dn_start = obs_x + WAKE_OBSTACLE_GAP;
    float spacing  = ((float)w - dn_start) / (float)(WAKE_VORTEX_COUNT + 1);

    int n = 0;
    out[n++] = make_marker(obs_x, cy);
    for (int i = 0; i < WAKE_VORTEX_COUNT && n < max_n; i++) {
        out[n++] = make_drifting_vortex(
            dn_start + spacing * (float)i,
            cy + ((i & 1) ? WAKE_ZIGZAG_AMP : -WAKE_ZIGZAG_AMP),
            1.5f, 1.5f,
            (float)i * 0.4f,
            WAKE_VORTEX_STR,
            (i & 1) ? -1 : +1
        );
    }
    return n;
}

/* QUADPOLE — four poles at the corners of a square, alternating
 * push/pull as you go around. */
static int place_quadpole_corners(Attractor out[], int max_n,
                                   float cx, float cy, int w, int h)
{
    float dx = (float)w * QUADPOLE_SEPARATION;
    float dy = (float)h * QUADPOLE_SEPARATION;
    float xs[4] = { cx - dx, cx + dx, cx + dx, cx - dx };
    float ys[4] = { cy - dy, cy - dy, cy + dy, cy + dy };
    int   sg[4] = { +1, -1, +1, -1 };
    int n = 0;
    for (int i = 0; i < 4 && n < max_n; i++) {
        out[n++] = make_drifting_vortex(xs[i], ys[i], 0.0f, 0.0f,
                                          0.0f, QUADPOLE_STRENGTH, sg[i]);
    }
    return n;
}

/* DUFFING — '@' markers on the two resting spots. The wind comes from
 * the formula; these are just labels. */
static int place_duffing_equilibria(Attractor out[], int max_n,
                                     float cx, float cy)
{
    if (max_n < 2) return 0;
    float d = 1.0f / DUFFING_SCALE;
    out[0] = make_marker(cx - d, cy);
    out[1] = make_marker(cx + d, cy);
    return 2;
}

/* NEWTON — '@' markers on the three pull-points (the three roots). */
static int place_newton_roots(Attractor out[], int max_n, float cx, float cy)
{
    if (max_n < 3) return 0;
    float d      = 1.0f / NEWTON_SCALE;
    float root_y = (float)(sin(M_PI / 3.0)) / NEWTON_SCALE;
    out[0] = make_marker(cx + d,        cy);
    out[1] = make_marker(cx - d * 0.5f, cy + root_y);
    out[2] = make_marker(cx - d * 0.5f, cy - root_y);
    return 3;
}

/* HAMILTON — '@' markers on the pinch point in the middle and the two
 * loop centres on either side. */
static int place_hamilton_equilibria(Attractor out[], int max_n,
                                      float cx, float cy)
{
    if (max_n < 3) return 0;
    float d = 1.0f / ((float)sqrt(2.0) * HAMILTON_SCALE);
    out[0] = make_marker(cx,     cy);   /* the pinch point  */
    out[1] = make_marker(cx + d, cy);   /* right loop centre */
    out[2] = make_marker(cx - d, cy);   /* left loop centre  */
    return 3;
}

/* PITCHFORK — '@' markers on the unstable middle and the two resting
 * spots it splits into. */
static int place_pitchfork_equilibria(Attractor out[], int max_n,
                                       float cx, float cy)
{
    if (max_n < 3) return 0;
    float d = sqrtf(PITCHFORK_MU) / PITCHFORK_SCALE;
    out[0] = make_marker(cx,     cy);
    out[1] = make_marker(cx + d, cy);
    out[2] = make_marker(cx - d, cy);
    return 3;
}

/*
 * pattern_init_attractors — set up the right special points for the
 * chosen pattern by calling its placer. Some patterns have a full
 * layout, some just one '@' in the middle, and some have none at all.
 */
static int pattern_init_attractors(Attractor out[], int max_n,
                                   Pattern p, int w, int h)
{
    float cx = (float)w * 0.5f;
    float cy = (float)h * 0.5f;

    switch (p) {

    /* Patterns with a full arrangement of points. */
    case PATTERN_VORTICES:   return place_vortices_quarters    (out, max_n, cx, cy, w, h);
    case PATTERN_MAGNET:     return place_magnet_dipole        (out, max_n, cx, cy, w);
    case PATTERN_TURBULENT:  return place_turbulent_field      (out, max_n, w, h);
    case PATTERN_SPIRAL:     return place_spiral_centre        (out, max_n, cx, cy);
    case PATTERN_GALAXY:     return place_galaxy_centre        (out, max_n, cx, cy);
    case PATTERN_CHAIN:      return place_chain_alternating    (out, max_n, cy, w);
    case PATTERN_GRID:       return place_grid_checkerboard    (out, max_n, w, h);
    case PATTERN_WAKE:       return place_wake_obstacle        (out, max_n, cy, w);
    case PATTERN_QUADPOLE:   return place_quadpole_corners     (out, max_n, cx, cy, w, h);

    /* Patterns that just get '@' labels on their resting spots. */
    case PATTERN_DUFFING:    return place_duffing_equilibria   (out, max_n, cx, cy);
    case PATTERN_NEWTON:     return place_newton_roots         (out, max_n, cx, cy);
    case PATTERN_HAMILTON:   return place_hamilton_equilibria  (out, max_n, cx, cy);
    case PATTERN_PITCHFORK:  return place_pitchfork_equilibria (out, max_n, cx, cy);

    /* Patterns with one important point: just mark the centre. */
    case PATTERN_RADIAL:
    case PATTERN_RIPPLE:
    case PATTERN_CIRCULAR:
    case PATTERN_VANDERPOL:
    case PATTERN_PENDULUM:
    case PATTERN_LOTKA:
    case PATTERN_HOPF:
    case PATTERN_HURRICANE:
    case PATTERN_GRADIENT:
    case PATTERN_EDDY:
        return place_centre_marker(out, max_n, cx, cy);

    /* Patterns with no single point worth marking. */
    case PATTERN_WAVE:
    case PATTERN_SADDLE:
    case PATTERN_SHEAR:
    case PATTERN_JET:
    case PATTERN_STANDING:
    case PATTERN_PLANE:
    case PATTERN_HENON:
    default:
        return 0;
    }
}

/*
 * compute_active_attractors — work out where each point actually sits
 * this frame, applying its slow wander to its home spot.
 */
static int compute_active_attractors(const Attractor src[], int n_src,
                                     ActiveAttractor out[], float t)
{
    for (int i = 0; i < n_src; i++) {
        float ang = t * DRIFT_RATE + src[i].phase;
        out[i].x         = src[i].bx + src[i].ox * cosf(ang);
        out[i].y         = src[i].by + src[i].oy * sinf(ang);
        out[i].strength  = src[i].strength;
        out[i].sign      = src[i].sign;
    }
    return n_src;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * The scene is built from six small structs, each handling one job,
 * with Scene bundling them together. Splitting them up also makes
 * function signatures honest: anything handed a `const Grid *` plainly
 * can't change the drawing buffers, and so on.
 */

/*
 * Particle — one dust speck riding the wind. It paints a coloured
 * trail as it goes, and is recycled (respawned somewhere new) when it
 * runs off the edge or its lifetime runs out.
 *
 * The speck has no weight or momentum: it just goes wherever the wind
 * points right now. That's exactly what you want to *see* the wind's
 * shape, and it means the user's speed knob controls pace cleanly no
 * matter which pattern is on.
 *
 * Each speck also gets a random lifetime, so they don't all die and
 * respawn at the same instant — otherwise the screen would empty and
 * refill in distracting pulses.
 */
typedef struct {
    float x, y;       /* position, in cells (fractional)                    */
    int   color_idx;  /* which of the 4 trail shades; fixed for life        */
    int   age;        /* frames lived so far                                */
    int   max_age;    /* recycle once age reaches this (or it leaves the map) */
} Particle;

/*
 * Grid — just the map's size. A flat cell at (x, y) lives at index
 * y*w + x in the drawing buffers. The size is always kept within
 * MAP_W_MAX x MAP_H_MAX (clamped on every resize), so the rest of the
 * code can trust w*h never overflows the buffers.
 */
typedef struct {
    int w, h;
    int total_cells;
} Grid;

static inline int  grid_idx       (const Grid *g, int x, int y) { return y * g->w + x; }
static inline bool grid_in_bounds (const Grid *g, int x, int y)
{
    return x >= 0 && x < g->w && y >= 0 && y < g->h;
}

/*
 * RenderBuffers — what to draw at each cell, in two arrays:
 *   glow  : how bright the trail is, 0..1 (picks the glyph)
 *   color : which of the 4 trail shades
 * Specks write these as they move; the drawing code reads only these
 * to paint the trails. The '@' markers are handled separately.
 *
 * They're kept as two separate arrays (not one array of pairs) because
 * glow is touched for every cell every frame while color is read only
 * where something was drawn — separating them is friendlier to the CPU
 * cache.
 */
typedef struct {
    float   glow [CELLS_MAX];
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
 * Particles — the speck pool. A big fixed array plus a count of how
 * many are currently in use, so spawning and recycling never touch the
 * heap (only pool[0..n-1] are alive). We allocate room for 1024 but
 * normally run 256, which fills a full-size grid nicely without turning
 * it into a solid blob.
 */
typedef struct {
    Particle pool[MAX_PARTICLES];   /* storage; only [0..n-1] are alive */
    int      n;                     /* how many are in use              */
} Particles;

/*
 * AttractorPool — the pattern's special points in two forms: the home
 * list (set at load time, with each point's wander) and the live list
 * (where each point actually is this frame). The wind formulas and the
 * renderer read the live list, which is recomputed once per frame so we
 * do the wander math once, not for every speck.
 *
 * Some patterns get their wind from a formula, not from these points;
 * those still put strength-0 entries here just so the '@' marks show up.
 * A strength of 0 means they add nothing to any wind.
 */
typedef struct {
    Attractor       base [MAX_ATTRACTORS];   /* home positions + wander */
    int             n_base;
    ActiveAttractor active[MAX_ATTRACTORS];  /* this frame's positions  */
    int             n_active;
} AttractorPool;

/*
 * SimState — the simulation's own clock, kept apart from the user's
 * knobs (Controls) so it's clear what the simulation changes versus
 * what the user changes. field_time is the seconds the wind has been
 * running. It drives both the time-varying patterns (WAVE, RIPPLE,
 * STANDING, PLANE) and the slow wander of the '@' points, so the two
 * stay in step. (It's a struct of one field to match the other files
 * and leave room to grow.)
 */
typedef struct {
    /* Seconds since the last reset; starts at 0, only grows. */
    float field_time;
} SimState;

/*
 * Controls — everything the user can change from the keyboard. The
 * keyboard handler writes these; the simulation and the renderer only
 * read them. Keeping them separate from SimState makes it clear which
 * changes come from the user and which from the simulation itself.
 */
typedef struct {
    /* When true, the wind freezes but the screen keeps refreshing, so
     * the HUD stays live and you can still change theme or glyphs. */
    bool    paused;

    /* Speck speed, cells per second. '+' doubles it, '-' halves it
     * (kept between 1 and 64). Doubling rather than stepping by one
     * makes each press feel like the same-sized change. */
    int     speed;

    int     current_theme;       /* which colour scheme is on */

    /* Which glyph set is on. Changing it only affects drawing, so it
     * doesn't restart the simulation. */
    int     current_glyph_set;

    /* Which wind is showing. Switching it does a full reset, because
     * each pattern needs its own set of special points. */
    Pattern current_pattern;
} Controls;

/*
 * Scene — everything, in one place. Reading the fields top to bottom
 * is a quick tour of the program:
 *   grid       — the map's size
 *   buf        — what to draw at each cell
 *   particles  — the dust specks
 *   attractors — the wind's special points
 *   sim        — the simulation clock
 *   ctrl       — the user's settings
 */
typedef struct {
    Grid          grid;
    RenderBuffers buf;
    Particles     particles;
    AttractorPool attractors;
    SimState      sim;
    Controls      ctrl;
} Scene;

/* ── particle pipeline ────────────────────────────────────────────── *
 * What happens to one speck each frame: read the wind where it is,
 * nudge it that way, stamp its trail, and recycle it if it's done.
 * Each step is a small helper below. */

static void particle_spawn(Particle *p, const Grid *g)
{
    p->x         = (float)(rand() % g->w);
    p->y         = (float)(rand() % g->h);
    p->color_idx = rand() & (N_BANDS - 1);
    p->age       = 0;
    p->max_age   = AGE_MIN_TICKS + rand() % (AGE_MAX_TICKS - AGE_MIN_TICKS);
}

/*
 * field_velocity_at — the one place that picks the active wind and
 * asks it for the push at a point. Some winds read the special points;
 * the rest just compute from a formula.
 */
static void field_velocity_at(const Scene *s, Pattern pat,
                              float x, float y,
                              float *out_vx, float *out_vy)
{
    const AttractorPool *ap = &s->attractors;
    float cx = (float)s->grid.w * 0.5f;
    float cy = (float)s->grid.h * 0.5f;

    switch (pat) {
    case PATTERN_VORTICES:
    case PATTERN_CHAIN:
    case PATTERN_GRID:
    case PATTERN_TURBULENT:
        field_vortices(ap->active, ap->n_active, x, y, out_vx, out_vy);
        break;

    case PATTERN_WAVE:
        field_wave(x, y, s->sim.field_time, out_vx, out_vy);
        break;

    case PATTERN_SADDLE:
        field_saddle(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_MAGNET:
    case PATTERN_QUADPOLE:
        /* QUADPOLE uses the same magnet math; only where the poles sit
         * is different. */
        field_magnet(ap->active, ap->n_active, x, y, out_vx, out_vy);
        break;

    case PATTERN_SPIRAL:
        field_spiral(ap->active, ap->n_active, x, y, out_vx, out_vy);
        break;

    case PATTERN_RADIAL:
        field_radial(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_SHEAR:
        field_shear(x, y, cy, out_vx, out_vy);
        break;

    case PATTERN_RIPPLE:
        field_ripple(x, y, s->sim.field_time, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_CIRCULAR:
        field_circular(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_GALAXY:
        field_galaxy(ap->active, ap->n_active, x, y, out_vx, out_vy);
        break;

    case PATTERN_DUFFING:
        field_duffing(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_VANDERPOL:
        field_van_der_pol(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_PENDULUM:
        field_pendulum(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_LOTKA:
        field_lotka(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_HOPF:
        field_hopf(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_NEWTON:
        field_newton(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_JET:
        field_jet(x, y, cy, (float)s->grid.h * 0.5f, out_vx, out_vy);
        break;

    case PATTERN_HURRICANE:
        field_hurricane(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_HAMILTON:
        field_hamilton(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_WAKE:
        field_wake(ap->active, ap->n_active, x, y, out_vx, out_vy);
        break;

    case PATTERN_STANDING:
        field_standing(x, y, s->sim.field_time, out_vx, out_vy);
        break;

    case PATTERN_PLANE:
        field_plane(x, y, s->sim.field_time, out_vx, out_vy);
        break;

    case PATTERN_GRADIENT:
        field_gradient(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_EDDY:
        field_eddy(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_PITCHFORK:
        field_pitchfork(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_HENON:
        field_henon(x, y, cx, cy, out_vx, out_vy);
        break;

    default:
        *out_vx = 0.0f;
        *out_vy = 0.0f;
        break;
    }
}

/* sample_unit_velocity — get the wind's direction at a point, dropping
 * its strength (length 1). That way the user's speed knob sets the
 * pace, not the raw numbers, which differ wildly between patterns. */
static void sample_unit_velocity(const Scene *s, Pattern pat,
                                  float px, float py,
                                  float *out_vx, float *out_vy)
{
    field_velocity_at(s, pat, px, py, out_vx, out_vy);
    float m = sqrtf((*out_vx) * (*out_vx) + (*out_vy) * (*out_vy));
    if (m > VELOCITY_EPSILON) {
        *out_vx /= m;
        *out_vy /= m;
    }
}

/* advect_particle_euler — slide the speck along the wind by one step.
 * No momentum: it goes purely where the wind points. */
static void advect_particle_euler(Particle *p, float vx, float vy,
                                   float dt, int speed)
{
    p->x += vx * (float)speed * dt;
    p->y += vy * (float)speed * dt;
    p->age++;
}

/* deposit_trail_hit — light up the cell the speck is on. The fading is
 * handled elsewhere; here we just stamp it bright. */
static void deposit_trail_hit(Scene *s, int cx, int cy, int color_idx)
{
    if (!grid_in_bounds(&s->grid, cx, cy)) return;
    int idx = grid_idx(&s->grid, cx, cy);
    s->buf.glow [idx] = TRAIL_HIT_INTENSITY;
    s->buf.color[idx] = (uint8_t)color_idx;
}

/* particle_is_expired — has the speck run out of life or wandered off
 * the map? Either way it's time to recycle it. */
static bool particle_is_expired(const Particle *p, const Grid *g)
{
    return p->age >= p->max_age
        || p->x < 0.0f || p->x >= (float)g->w
        || p->y < 0.0f || p->y >= (float)g->h;
}

static void particle_step(Scene *s, Particle *p, Pattern pat,
                          float dt, int speed)
{
    float vx, vy;
    sample_unit_velocity (s, pat, p->x, p->y, &vx, &vy);
    advect_particle_euler(p, vx, vy, dt, speed);
    deposit_trail_hit    (s, (int)p->x, (int)p->y, p->color_idx);
    if (particle_is_expired(p, &s->grid))
        particle_spawn   (p, &s->grid);
}

/* ── tick pipeline ────────────────────────────────────────────────── *
 * One simulation step: fade the old trails a bit, advance the clock,
 * move the special points along their wander, then move every speck.
 * Each line below is a small helper. */

static void decay_trail_glow(RenderBuffers *b, int n, float dt)
{
    float decay = expf(-TRAIL_GLOW_DECAY * dt);
    for (int i = 0; i < n; i++) b->glow[i] *= decay;
}

static void advance_field_time(SimState *sim, float dt)
{
    sim->field_time += dt;
}

static void refresh_active_attractors(AttractorPool *ap, float sim_time)
{
    ap->n_active = compute_active_attractors(ap->base, ap->n_base,
                                              ap->active, sim_time);
}

static void step_all_particles(Scene *s, float dt)
{
    Pattern pat = s->ctrl.current_pattern;
    int     spd = s->ctrl.speed;
    for (int i = 0; i < s->particles.n; i++)
        particle_step(s, &s->particles.pool[i], pat, dt, spd);
}

/* ── reset / init pipeline ────────────────────────────────────────── */

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

static void init_pattern_attractors(AttractorPool *ap, Pattern p,
                                     const Grid *g)
{
    ap->n_base = pattern_init_attractors(ap->base, MAX_ATTRACTORS,
                                          p, g->w, g->h);
    ap->n_active = 0;
}

static void scene_reset(Scene *s, int w, int h)
{
    apply_grid_dimensions    (&s->grid, w, h);
    reset_sim_state          (&s->sim);
    buffers_clear            (&s->buf, s->grid.total_cells);
    init_pattern_attractors  (&s->attractors, s->ctrl.current_pattern, &s->grid);
    spawn_all_particles      (&s->particles, &s->grid);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->ctrl.paused            = false;
    s->ctrl.speed             = SPEED_DEF;
    s->ctrl.current_theme     = 0;
    s->ctrl.current_glyph_set = 2;        /* MEDIUM */
    s->ctrl.current_pattern   = PATTERN_VORTICES;
    scene_reset(s, w, h);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;

    decay_trail_glow           (&s->buf, s->grid.total_cells, dt);
    advance_field_time         (&s->sim, dt);
    refresh_active_attractors  (&s->attractors, s->sim.field_time);
    step_all_particles         (s, dt);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the terminal's current size. That's all we need to remember
 * to centre the map and place the HUD; ncurses tracks everything else.
 * Refreshed whenever the window is resized.
 */
typedef struct {
    int cols;   /* width in characters  */
    int rows;   /* height in characters */
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

/* ── scene_draw pipeline ──────────────────────────────────────────── *
 * Drawn in two passes: first the trails, then the bright '@' markers
 * on top so they're never hidden. */

/*
 * CellDraw — a little "here's what to put in this cell" note: which
 * colour, which character, or skip it entirely. We work out this note
 * first (a plain decision, no drawing) and let one function turn it
 * into actual output. Skipping leaves whatever was there, which is most
 * cells, since the map is mostly empty between trails.
 */
typedef struct {
    int  pair;   /* which colour to use     */
    int  attr;   /* bold for brighter trails, normal otherwise */
    char glyph;  /* which character to draw  */
    bool skip;   /* true = draw nothing here */
} CellDraw;

/* compute_centred_origin — where to start drawing the map so it sits
 * centred, leaving room for the HUD rows above and below. */
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

/* cell_density_band — pick the character and brightness for a cell
 * based on how strong its trail is. */
static CellDraw cell_density_band(uint8_t band, float glow, const GlyphSet *gs)
{
    int pair = PAIR_TRAIL_BASE + (band & (N_BANDS - 1));
    if (glow > GLYPH_HIGH_THRESH) return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = gs->high };
    if (glow > GLYPH_MID_THRESH ) return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = gs->mid  };
    if (glow > GLOW_THRESHOLD   ) return (CellDraw){ .pair = pair, .attr = A_NORMAL, .glyph = gs->low  };
    return (CellDraw){ .skip = true };
}

/* paint_cell — actually draw one cell's CellDraw note to the screen. */
static void paint_cell(int sy, int sx, CellDraw c)
{
    if (c.skip) return;
    attron (COLOR_PAIR(c.pair) | c.attr);
    mvaddch(sy, sx, (chtype)(unsigned char)c.glyph);
    attroff(COLOR_PAIR(c.pair) | c.attr);
}

/* draw_trail_layer — pass 1: the speck trails. */
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

/* draw_attractor_layer — pass 2: the bright '@' markers, drawn on top. */
static void draw_attractor_layer(const AttractorPool *ap,
                                  int gx0, int gy0, int cols, int rows)
{
    attron(COLOR_PAIR(PAIR_ATTRACT) | A_BOLD);
    for (int i = 0; i < ap->n_active; i++) {
        int sx = gx0 + (int)ap->active[i].x;
        int sy = gy0 + (int)ap->active[i].y;
        if (sx < 0 || sx >= cols) continue;
        if (sy < 0 || sy >= rows) continue;
        mvaddch(sy, sx, (chtype)(unsigned char)'@');
    }
    attroff(COLOR_PAIR(PAIR_ATTRACT) | A_BOLD);
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
    draw_trail_layer    (s, gx0, gy0, cols, rows, gs);
    draw_attractor_layer(&s->attractors, gx0, gy0, cols, rows);
}

/* ── HUD draw pipeline ────────────────────────────────────────────── *
 * The top three rows show what's going on (pattern, theme, counts,
 * legend); the bottom row lists the keys. Each piece has its own small
 * drawer below. */

static void draw_hud_state_bar(const Screen *sc, const Scene *s,
                                double fps, int sim_fps)
{
    const Controls *c = &s->ctrl;
    const char *state_str = c->paused ? "PAUSED   "
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
    mvprintw(0, HUD_LEFT_MARGIN, " FLOW FIELD PARTICLES ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* draw_palette_swatch — show one '#' in each of the four trail colours
 * so you can see the theme. Returns where to draw next. */
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

/* ── row 1 segment drawers ──────────────────────────────────────────── *
 * Row 1 is built left to right from fixed-width pieces. Each piece
 * draws its part and returns where the next one should start. */

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

/* draw_status_sim_counts — the last piece: live counts (specks, marker
 * points, map size). */
static void draw_status_sim_counts(int row, int x, const Scene *s)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  parts:%d  attr:%d  map:%dx%d ",
             s->particles.n, s->attractors.n_active,
             s->grid.w, s->grid.h);
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
        draw_status_sim_counts    (1, x, s);
}

/* draw_hud_glyph_indicator — shows the current set's three characters
 * so you can preview a look before switching with g/G. Pinned to the
 * right so it won't bump into the counts on the left. */
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

/* draw_hud_legend — row 2: a reminder of what the glyphs mean. */
static void draw_hud_legend(void)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(2, HUD_LEFT_MARGIN,
             " trails: density-graded (low -> mid -> high)   @: attractor / equilibrium ");
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* draw_bottom_hint — bottom row: the list of keys you can press. */
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
    draw_hud_state_bar      (sc, s, fps, sim_fps);   /* row 0  */
    draw_hud_title          ();                       /* row 0  */
    draw_hud_status_line    (s);                      /* row 1  */
    draw_hud_glyph_indicator(s, sc);                  /* row 1  */
    draw_hud_legend         ();                       /* row 2  */
    draw_bottom_hint        (sc);                     /* bottom */
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

/*
 * App — the whole program in one bundle: the scene, the screen, the
 * tick rate, the map size, and a couple of flags. There's a single
 * global copy (g_app) so signal handlers can reach it.
 *
 * The running and need_resize flags are the only thing the signal
 * handlers touch — they just flip a flag and the main loop does the
 * real work. That's the safe way to handle signals.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   map_w, map_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
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
 * One small function per key; app_handle_key just routes to them. */

/* bump_speed_geometric — '+' doubles speed, '-' halves it. Doubling
 * makes each press feel like the same-sized change. */
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

/* bump_sim_fps — '[' / ']' change how many times a second the
 * simulation steps, kept within limits. */
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

/* cycle_pattern — switch winds with n/p, then fully reset. We reset
 * because each wind has its own special points; keeping the old ones
 * would make the first frame look wrong. */
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
    case 't':           cycle_theme   (c,   +1);                           break;
    case 'T':           cycle_theme   (c,   -1);                           break;
    case 'g':           cycle_glyph_set(c,  +1);                           break;
    case 'G':           cycle_glyph_set(c,  -1);                           break;
    case 'n': case 'N': cycle_pattern (app, +1);                           break;
    case 'p': case 'P': cycle_pattern (app, -1);                           break;
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

/* advance_frame_clock — how long since the last frame, capped so a big
 * stall doesn't make us try to catch up all at once. */
static int64_t advance_frame_clock(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > DT_MAX_NS) dt = DT_MAX_NS;
    return dt;
}

/* simulate_pending_ticks — run as many fixed-size simulation steps as
 * the elapsed time has earned, so the sim runs at a steady rate no
 * matter the frame rate. See Glenn Fiedler, "Fix Your Timestep". */
static void simulate_pending_ticks(App *app, int64_t *sim_accum,
                                    int64_t tick_ns, float dt_sec)
{
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* maybe_update_fps_counter — every half second, work out the frame
 * rate from the frames counted so far (otherwise leave it unchanged). */
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

/* cap_frame_rate — nap for whatever's left in this frame's time budget
 * so we don't run faster than the target frame rate. */
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
