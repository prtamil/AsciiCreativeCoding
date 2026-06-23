/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * vector_field_arrows_showcase.c
 *   Draws a 2-D vector field as a grid of ASCII arrows: at every cell we
 *   work out which way the field points and how strong it is, then pick an
 *   arrow glyph for the direction and a colour for the strength.  30 fields
 *   to flip through, from simple radial sprays to fluid swirls and physics
 *   classics.
 *
 * Sister demos in this folder, for the curious:
 *   ./curl_noise_vector_field.c   — one swirl technique used here, gone deeper
 *   ./flow_field_particles.c      — particles riding a field; this shows the field itself
 *   ./magnetic_fields.c           — magnetic field drawn as lines, not per-cell arrows
 *
 * A few references the code alone can't give you:
 *   Helman & Hesselink (1989), "Vector Field Topology in Fluid Flow Data" —
 *     names the field shapes (sources, sinks, saddles, spirals) Tiers 2-5 show.
 *   Bridson et al. (2007), "Curl-Noise for Procedural Fluid Flow" — Tier 4 swirls.
 *   Strogatz, "Nonlinear Dynamics and Chaos" — Tier 5 phase portraits + the spiral
 *     the MOTION feature uses.
 *   Griffiths, "Introduction to Electrodynamics" — Tier 3 charge/magnet fields.
 *   Eric Treacy, "material-vector-field" p5.js sketch — the MOTION idea, but with a
 *     noise-driven roaming point instead of the mouse.
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

    HUD_COLS          =  80,
    FPS_UPDATE_MS     = 500,

    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,
};

/* HUD layout — the top two rows show status, the bottom row shows keys. */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* Column widths for the status row.  These MUST match the field widths the
 * hud_field_* helpers print, or the fields overlap or leave gaps. */
#define HUD_PATTERN_FIELD_W     21
#define HUD_TIER_FIELD_W        15
#define HUD_THEME_FIELD_W       17
#define HUD_PALETTE_LABEL_W      9
#define HUD_N_PALETTE_BANDS      4

/* Smallest grid we'll draw — anything tinier is unreadable. */
#define MAP_W_MIN               16
#define MAP_H_MIN                8

/* How long each drawn frame should last, so we don't peg the CPU. */
#define RENDER_FPS_TARGET       60
#define RENDER_FRAME_BUDGET_NS  (NS_PER_SEC / RENDER_FPS_TARGET)

/* If one frame takes too long (slow terminal), pretend no more than this
 * much time passed.  Otherwise the sim tries to "catch up" forever and
 * the program locks up.  See Fiedler, "Fix Your Timestep!". */
#define SIM_MAX_FRAME_DT_MS    100

/* Animation speed multiplier, stepped by +/-. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      4
#define DRIFT_MULT_MAX      16

/* How fast the animation clock advances (per second) for the moving fields. */
#define FIELD_DRIFT         1.0f

/* If a cell's field is weaker than this (after scaling to 0..1), draw a dot
 * instead of an arrow — the direction is too weak to mean anything. */
#define ARROW_DEAD_ZONE     0.06f

/* We round each direction to one of 8 compass headings; this is the width
 * of one heading slice (a full turn split into 8). */
#define ARROW_BIN_WIDTH     ((float)(M_PI / 4.0))

/* The random-but-smooth noise used by the GRAD_NOISE and CURL_NOISE fields.
 * The small frequency keeps the noise gentle across neighbouring cells. */
#define NOISE_FREQ          0.10f

/* How tight the wave patterns are for two of the gradient fields. */
#define GRAD_PERIODIC_FREQ  0.25f
#define GRAD_RIPPLE_FREQ    0.30f

/* Physics fields: how far apart the two charges sit, and a small fudge that
 * stops us dividing by zero right on top of a charge. */
#define DIPOLE_HALF_SEP     8.0f
#define QUADRUPOLE_HALF     6.0f
#define COULOMB_SOFT_EPS    1.5f

/* How tightly the grid of swirls is packed for the STREAM_GRID field. */
#define STREAM_FREQ_X       0.10f
#define STREAM_FREQ_Y       0.20f

/* The Tier-5 fields think in their own coordinate range, not screen pixels.
 * These say how much of that range the screen maps onto. */
#define PHASE_HALF_EXTENT       3.0f      /* most fields: -3 to +3            */
#define PENDULUM_X_HALF_EXTENT  3.14159f  /* pendulum angle: -pi to +pi       */
#define VDP_MU                  1.0f      /* Van der Pol "wobbliness" knob    */
#define HOPF_MU                 1.0f      /* sets the Hopf cycle's radius      */

/* How long one full cycle of each animated field takes, in seconds. */
#define ROT_DIPOLE_PERIOD   6.0f
#define TRAVEL_WAVE_PERIOD  4.0f
#define BREATHE_PERIOD      4.0f
#define ORBIT_PERIOD        8.0f
#define ORBIT_RADIUS_FRAC   0.30f         /* orbit size, as a fraction of the screen */

/* ---------- MOTION (the 'm' key) ------------------------------------ *
 *
 * Press 'm' and a roaming "swirl" is blended into whatever field is showing.
 * The idea comes from Eric Treacy's material-vector-field sketch, except the
 * swirl follows an invisible point that wanders on its own (driven by noise)
 * instead of following the mouse.
 *
 * Each cell's arrow becomes the static field's direction PLUS the swirl's
 * direction, both shrunk to the same length first so neither overpowers the
 * other.  We add rather than replace so each preset still looks like itself
 * under MOTION — a tilted field plus swirl looks nothing like a quadrupole
 * plus swirl.  The roaming point uses two separate noise streams for x and y
 * so it drifts naturally instead of tracing a neat circle. */
#define MOTION_SPIRAL_WEIGHT  1.0f         /* swirl vs field mix (1.0 = equal)      */
#define PROBE_EXTENT_FRAC     0.40f        /* point roams the central 80% of screen */
#define PROBE_DRIFT_RATE      0.12f        /* how fast the point wanders            */

/* Each field has its own idea of "how strong is strong".  These numbers tune
 * the strength-to-brightness mapping per field: bigger number means the field
 * must be stronger before it reaches the brightest colour. */
#define SCALE_RADIAL_HALFDIAG  1.0f       /* radial fields, scaled by screen size */
#define SCALE_GRADIENT         0.5f       /* slopes of simple scalar fields       */
#define SCALE_NOISE_GRAD       2.0f       /* noise slopes are gentle              */
#define SCALE_INV_SQ           0.4f       /* fields that fall off like 1/r^2      */
#define SCALE_INV_R            1.5f       /* fields that fall off like 1/r        */
#define SCALE_BOUNDED          0.8f       /* sin/cos fields (already small)       */
#define SCALE_PHASE            2.0f       /* the dynamical-system fields          */

/*
 * Pattern — the 30 fields you can flip through, grouped into 6 tiers of
 * increasing richness.  The order here MUST match the vector_patterns[] table
 * down in §7; the table is keyed by these names so the compiler catches any
 * mismatch.  The names are spelled out in plain words in that table.
 */
typedef enum {
    /* Tier 1 — slopes of simple landscapes (arrows point uphill) */
    PATTERN_GRAD_PARABOLOID = 0,
    PATTERN_GRAD_SADDLE,
    PATTERN_GRAD_PERIODIC,
    PATTERN_GRAD_RIPPLE,
    PATTERN_GRAD_NOISE,
    /* Tier 2 — textbook flows: sprays, sinks, spins, shears */
    PATTERN_RADIAL_OUT,
    PATTERN_RADIAL_IN,
    PATTERN_ROTATION,
    PATTERN_SHEAR_X,
    PATTERN_UNIFORM_TILTED,
    /* Tier 3 — real physics: charges, magnets, gravity */
    PATTERN_POINT_CHARGE,
    PATTERN_DIPOLE,
    PATTERN_WIRE_MAGNETIC,
    PATTERN_GRAVITY,
    PATTERN_QUADRUPOLE,
    /* Tier 4 — fluid-like swirls (nothing piles up or drains away) */
    PATTERN_CURL_NOISE,
    PATTERN_STREAM_GRID,
    PATTERN_VORTEX_PAIR,
    PATTERN_CHANNEL_FLOW,
    PATTERN_NOISY_UNIFORM,
    /* Tier 5 — phase portraits: how systems settle, spin, or oscillate */
    PATTERN_STABLE_NODE,
    PATTERN_STABLE_SPIRAL,
    PATTERN_HOPF_CYCLE,
    PATTERN_VAN_DER_POL,
    PATTERN_PENDULUM,
    /* Tier 6 — fields that move on their own over time */
    PATTERN_ROTATING_DIPOLE,
    PATTERN_TRAVELLING_WAVE,
    PATTERN_BREATHING_RADIAL,
    PATTERN_ORBITING_VORTEX,
    PATTERN_DRIFT_CURL,
    N_PATTERNS,
} Pattern;

/* Defined later in §7, next to the table they read from. */
static const char *pattern_name(Pattern p);
static const char *pattern_tier(Pattern p);

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Theme — one colour scheme: four colours running from dim to bright.
 *
 * Why it exists: fields don't pick colours.  Each cell just reports a
 * brightness level 0..3 (0 = weak field, 3 = strong), and the theme decides
 * what actual colour each level looks like.  Swapping themes with t/T just
 * rebinds those four colours, so no field code ever mentions colour.
 *
 * The four colours are a dim-to-bright ramp, not random picks:
 *   band[0] dim    — weakest cells (and the '.' dots at dead spots)
 *   band[1] low    — weak cells
 *   band[2] mid    — the everyday on-screen brightness
 *   band[3] bright — strongest cells
 * Every colour stays in the bright half of the palette so even band 0 is
 * clearly visible on a black background.  That matters here because MOTION
 * animates by spinning arrows (it doesn't change their brightness), so even
 * the weak cells need to be bright enough to watch their glyphs turn.
 *
 * Members:
 *   name    — short label shown in the HUD (just a pointer to a literal).
 *   band[4] — the four colour codes, from dim to bright.  A theme's
 *             character comes from the shape of the ramp, not the exact hues.
 *
 * Colour codes are xterm-256 indices (the 256-colour terminal palette).
 */
typedef struct {
    const char *name;
    short       band[4];        /* colour codes, dim -> bright */
} Theme;

#define N_THEMES 10

/* Every colour sits in the bright half of the palette so even the weakest
 * cells stay visible — see the note on the Theme struct for why. */
static const Theme themes[N_THEMES] = {
    { "DEFAULT", {  75,  123,  220,  231 } },   /* sky-blue → cyan → yellow → white */
    { "MATRIX",  {  77,  118,  156,  194 } },   /* bright green ramp                */
    { "NOVA",    { 135,  171,  207,  219 } },   /* magenta → pink → pale pink       */
    { "MONO",    { 247,  250,  253,  255 } },   /* light-gray ramp                  */
    { "OCEAN",   {  81,  117,  159,  195 } },   /* cyan → pale cyan                 */
    { "FIRE",    { 208,  214,  220,  227 } },   /* orange → yellow → pale yellow    */
    { "EARTH",   { 143,  179,  215,  222 } },   /* tan → sand → cream               */
    { "FOREST",  { 114,  150,  157,  194 } },   /* sage → light sage → very light   */
    { "DESERT",  { 179,  215,  222,  229 } },   /* sand → cream → pale              */
    { "ARCTIC",  { 117,  159,  195,  231 } },   /* pale blue → near white           */
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
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_BAND_BASE + i, t->band[i], -1);
    } else {
        static const short fallback[4] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_BAND_BASE + i, fallback[i], -1);
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
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  vector primitives — noise, scalar gradients, 2-D curl              */
/* ===================================================================== */

/*
 * NoiseField — the random seed behind our smooth random noise.
 *
 * The noise gives us a wiggly random landscape that's the same every time
 * for a given seed.  Three fields lean on it: the two noise-based fields and
 * the roaming point used by MOTION.  Pressing 'r' picks a new seed, which
 * freshens all of them at once.
 *
 * Why a struct for one number: it gives the noise a clear home on the Scene,
 * and re-seeding it here leaves the program's general random number stream
 * (rand()) alone — so re-seeding doesn't accidentally shift colours or
 * anything else that also calls rand().
 *
 * Member:
 *   seed — the 32-bit number that shapes the random landscape.  We make it
 *          by xor-ing two rand() draws together, which gives better-mixed
 *          bits than a single draw.
 *
 * The noise math is value noise (Perlin, 1985, "An Image Synthesizer") — a
 * simpler cousin of classic Perlin noise.  hash32() is a Wang-style integer
 * scrambler used so we don't need a lookup table.
 */
typedef struct {
    uint32_t seed;
} NoiseField;

/* The noise lookups happen for every cell, so rather than pass the seed
 * down through every call we keep a copy here.  Only noise_field_reseed
 * ever writes it. */
static uint32_t g_lattice_seed = 0;

static inline uint32_t hash32(uint32_t x)
{
    x = (x ^ (x >> 16)) * 0x7feb352du;
    x = (x ^ (x >> 15)) * 0x846ca68bu;
    x = (x ^ (x >> 16));
    return x;
}

static void noise_field_reseed(NoiseField *nf)
{
    nf->seed       = (uint32_t)rand() ^ ((uint32_t)rand() << 16);
    g_lattice_seed = nf->seed;
}

/* A repeatable random value in 0..1 for one integer grid point. */
static inline float lattice_scalar(int xi, int yi)
{
    uint32_t h = (uint32_t)xi * 374761393u
               + (uint32_t)yi * 668265263u
               + g_lattice_seed;
    return (float)(hash32(h) >> 8) * (1.0f / 16777215.0f);
}

static inline float smoothstep01(float t) { return t * t * (3.0f - 2.0f * t); }
static inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }

/* Smooth random value at any point — blends the four nearest grid corners
 * so the noise glides smoothly instead of jumping cell to cell. */
static float noise_sample(float x, float y)
{
    int   xi = (int)floorf(x);
    int   yi = (int)floorf(y);
    float ux = smoothstep01(x - (float)xi);
    float uy = smoothstep01(y - (float)yi);
    float v00 = lattice_scalar(xi,     yi);
    float v10 = lattice_scalar(xi + 1, yi);
    float v01 = lattice_scalar(xi,     yi + 1);
    float v11 = lattice_scalar(xi + 1, yi + 1);
    return lerpf(lerpf(v00, v10, ux), lerpf(v01, v11, ux), uy);
}

/* A scalar function: give it a point, it returns one number — think of it as
 * a height for that spot, so the whole thing is a landscape. */
typedef float (*ScalarFn2D)(float x, float y);

/* Find the uphill direction of a landscape at one point — the "slope".
 * We just compare the height a step right vs left, and up vs down. */
static inline void scalar_gradient(ScalarFn2D f, float x, float y,
                                   float *vx, float *vy)
{
    *vx = 0.5f * (f(x + 1.0f, y) - f(x - 1.0f, y));
    *vy = 0.5f * (f(x, y + 1.0f) - f(x, y - 1.0f));
}

/* Turn a landscape into a swirling flow: instead of pointing uphill, point
 * sideways to the slope.  Flows built this way never have a spot where stuff
 * piles up or drains out — exactly how an ideal fluid behaves.  Used by the
 * curl-noise fields in Tiers 4 and 6. */
static inline void scalar_curl_2d(ScalarFn2D phi, float x, float y,
                                  float *vx, float *vy)
{
    *vx =  0.5f * (phi(x, y + 1.0f) - phi(x, y - 1.0f));
    *vy = -0.5f * (phi(x + 1.0f, y) - phi(x - 1.0f, y));
}

/* The simple landscapes the Tier-1 gradient fields take the slope of. */
static float scalar_paraboloid (float x, float y) { return x * x + y * y; }
static float scalar_saddle     (float x, float y) { return x * x - y * y; }
static float scalar_periodic   (float x, float y)
{
    return sinf(x * GRAD_PERIODIC_FREQ) * cosf(y * GRAD_PERIODIC_FREQ);
}
static float scalar_ripple     (float x, float y)
{
    return cosf(GRAD_RIPPLE_FREQ * sqrtf(x * x + y * y));
}
static float scalar_noise      (float x, float y)
{
    return noise_sample(x * NOISE_FREQ, y * NOISE_FREQ);
}

/* ===================================================================== */
/* §6  arrow rendering — direction → glyph, magnitude → palette band      */
/* ===================================================================== */

/*
 * ARROW_GLYPHS — the eight arrow characters, one per compass heading.
 * We round each cell's direction to the nearest of 8 headings (E, SE, S, SW,
 * W, NW, N, NE on the screen) and look up the matching glyph here:
 *
 *   right '>'   down-right '\'   down 'v'   down-left '/'
 *   left  '<'   up-left   '\'    up   '^'   up-right   '/'
 *
 * Remember screen "down" is positive y, so the headings are listed the way
 * they look on screen.  The slashes do double duty: '/' is both up-right and
 * down-left, '\' is both up-left and down-right.  You can't tell which from
 * the glyph alone, but the arrows around it make the flow direction obvious.
 */
static const char ARROW_GLYPHS[8] = {
    '>', '\\', 'v', '/', '<', '\\', '^', '/',
};

/* Turn an angle into one of the 8 headings (0..7).  The +8 and the mask just
 * keep the result in range when the angle comes out negative. */
static inline int arrow_bin_from_angle(float angle)
{
    return ((int)floorf(angle / ARROW_BIN_WIDTH + 0.5f) + 8) & 7;
}

/* Squash a field strength into the 0..1 range for colouring.  No matter how
 * huge it gets (some physics fields blow up near their centre), this stays
 * below 1 — so a blow-up just maps to "brightest" instead of breaking. */
static inline float mag_saturate(float mag, float scale)
{
    return mag / (mag + scale);
}

/* Pick a brightness level 0..3 from a 0..1 strength. */
static inline uint8_t mag_to_band(float mag_norm)
{
    int b = (int)(mag_norm * 3.999f);
    if (b < 0) b = 0;
    if (b > 3) b = 3;
    return (uint8_t)b;
}

/* Draw a dot at a dead spot — where the field is too weak to point anywhere.
 * These dots mark the calm centres and balance points of a field. */
static inline void cell_emit_dot(float *gl, uint8_t *bn, char *gy)
{
    *gl = 1.0f; *bn = 0; *gy = '.';
}

/* Fill in one cell's arrow: glyph from the direction, colour from the
 * strength.  If the field is too weak there, draw a dot instead. */
static inline void cell_emit_arrow(float vx, float vy, float mag_norm,
                                   float *gl, uint8_t *bn, char *gy)
{
    if (mag_norm < ARROW_DEAD_ZONE) {
        cell_emit_dot(gl, bn, gy);
        return;
    }
    int bin = arrow_bin_from_angle(atan2f(vy, vx));
    *gl = 1.0f;
    *bn = mag_to_band(mag_norm);
    *gy = ARROW_GLYPHS[bin];
}

/* ===================================================================== */
/* §7  patterns — 30 vector-field visualisations + dispatch               */
/* ===================================================================== */

/*
 * Every field is one function with this shape, called once per cell.  It gets
 * the cell's position, the grid size (so it can find the centre), and the
 * animation clock (only the moving fields care).  It fills in the three
 * things the renderer needs for that cell:
 *   out_glow  — 1.0 = draw something here, 0.0 = leave blank
 *   out_band  — brightness level 0..3
 *   out_glyph — which character to draw (0 = nothing)
 *
 * Names you'll see reused inside the field functions:
 *   cx, cy — the screen centre
 *   dx, dy — how far this cell is from the centre
 *   r, r2  — distance from centre, and that distance squared
 */
typedef void (*VectorPatternFn)(int x, int y, int w, int h, float field_time,
                                float *out_glow, uint8_t *out_band, char *out_glyph);

/* Distance from the centre to a corner — a handy "size of the screen" number
 * to scale fields against. */
static inline float grid_half_diag(int w, int h)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    return sqrtf(cx * cx + cy * cy);
}

/* Is this one of the Tier-6 fields that move on their own?  Those always run
 * their animation; the others only move when MOTION is switched on. */
static inline bool pattern_is_animated_tier(Pattern p)
{
    return p >= PATTERN_ROTATING_DIPOLE;
}

/* ---------- MOTION — the roaming swirl ('m' key) ------------------- *
 *
 * Three small helpers make up the MOTION feature: find where the invisible
 * roaming point is right now, work out the swirl's direction at one cell, and
 * blend that swirl into a cell's arrow.  See the MOTION note in §1 for the
 * idea behind it. */

/* Where the roaming point is at time t.  Two separate noise streams steer x
 * and y so it drifts around naturally instead of circling.  Same t always
 * gives the same spot. */
static inline void motion_probe_position(float t, int w, int h,
                                         float *px, float *py)
{
    /* The big offset on the y stream pulls from a different part of the noise
     * so x and y wander independently. */
    float nx = noise_sample(t * PROBE_DRIFT_RATE,        0.0f);
    float ny = noise_sample(0.0f, t * PROBE_DRIFT_RATE + 13.7f);
    *px = 0.5f * (float)w + PROBE_EXTENT_FRAC * (float)w * (2.0f * nx - 1.0f);
    *py = 0.5f * (float)h + PROBE_EXTENT_FRAC * (float)h * (2.0f * ny - 1.0f);
}

/* Which way the inward swirl points at one cell, given the swirl's centre.
 * Returned as a unit-length arrow so it blends fairly with the field's own
 * direction.  (This is Eric Treacy's calcVec.) */
static inline void motion_spiral_dir(int x, int y, float px, float py,
                                     float *spx, float *spy)
{
    float dx = (float)x - px;
    float dy = (float)y - py;
    *spx =  dy - dx;
    *spy = -dx - dy;
    float m = sqrtf((*spx) * (*spx) + (*spy) * (*spy));
    if (m > 1e-6f) { *spx /= m; *spy /= m; }
}

/* The standard way the 30 fields hand off a cell.  With MOTION off (t == 0)
 * it's just the plain arrow.  With MOTION on (t > 0) it blends the field's
 * direction with the roaming swirl — both shrunk to the same length first so
 * neither wins by sheer size.  Where the two point opposite ways they cancel
 * and you get a dot, marking the seams between field and swirl.  The colour
 * still comes from the field's own strength. */
static inline void pattern_emit_arrow(int x, int y, int w, int h, float t,
                                      float vx, float vy, float mag_norm,
                                      float *gl, uint8_t *bn, char *gy)
{
    if (t > 0.0f) {
        float px, py;   motion_probe_position(t, w, h, &px, &py);
        float spx, spy; motion_spiral_dir(x, y, px, py, &spx, &spy);

        float pm = sqrtf(vx * vx + vy * vy);
        if (pm > 1e-6f) { vx /= pm; vy /= pm; }

        vx += MOTION_SPIRAL_WEIGHT * spx;
        vy += MOTION_SPIRAL_WEIGHT * spy;
    }
    cell_emit_arrow(vx, vy, mag_norm, gl, bn, gy);
}

/* ---------- Tier 1 — uphill arrows on simple landscapes ------------- */

/* GRAD_PARABOLOID — the slope of a bowl.  Arrows point straight out from the
 * centre, longer the farther out you go. */
static void pattern_grad_paraboloid(int x, int y, int w, int h, float t,
                                    float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float vx, vy;
    scalar_gradient(scalar_paraboloid, (float)x - cx, (float)y - cy, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, grid_half_diag(w, h) * SCALE_GRADIENT),
        gl, bn, gy);
}

/* GRAD_SADDLE — the slope of a Pringle/saddle shape.  Arrows push out
 * sideways but in from top and bottom; the centre is a balance point. */
static void pattern_grad_saddle(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float vx, vy;
    scalar_gradient(scalar_saddle, (float)x - cx, (float)y - cy, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, grid_half_diag(w, h) * SCALE_GRADIENT),
        gl, bn, gy);
}

/* GRAD_PERIODIC — slope of an egg-carton landscape.  A repeating grid of
 * hills and dips, with little swirls where they meet. */
static void pattern_grad_periodic(int x, int y, int w, int h, float t,
                                  float *gl, uint8_t *bn, char *gy)
{
    float vx, vy;
    scalar_gradient(scalar_periodic, (float)x, (float)y, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_BOUNDED), gl, bn, gy);
}

/* GRAD_RIPPLE — slope of a pond after a stone drops in.  Rings of arrows
 * that flip between pointing in and pointing out as you move outward. */
static void pattern_grad_ripple(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float vx, vy;
    scalar_gradient(scalar_ripple, (float)x - cx, (float)y - cy, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_BOUNDED), gl, bn, gy);
}

/* GRAD_NOISE — slope of a random rolling-hills landscape.  Arrows always
 * point uphill, like the steepest path up wherever you stand. */
static void pattern_grad_noise(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    float vx, vy;
    scalar_gradient(scalar_noise, (float)x, (float)y, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_NOISE_GRAD * NOISE_FREQ),
        gl, bn, gy);
}

/* ---------- Tier 2 — textbook flows --------------------------------- */

/* RADIAL_OUT — everything sprays straight out from the centre, like a sprinkler. */
static void pattern_radial_out(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float vx = (float)x - cx, vy = (float)y - cy;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, grid_half_diag(w, h) * SCALE_RADIAL_HALFDIAG),
        gl, bn, gy);
}

/* RADIAL_IN — everything points toward the centre, like water down a drain. */
static void pattern_radial_in(int x, int y, int w, int h, float t,
                              float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float vx = -((float)x - cx), vy = -((float)y - cy);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, grid_half_diag(w, h) * SCALE_RADIAL_HALFDIAG),
        gl, bn, gy);
}

/* ROTATION — everything spins around the centre, like a merry-go-round.
 * On screen it looks clockwise (the top edge slides right). */
static void pattern_rotation(int x, int y, int w, int h, float t,
                             float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float dx = (float)x - cx, dy = (float)y - cy;
    float vx = -dy, vy = dx;
    float mag = sqrtf(dx * dx + dy * dy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, grid_half_diag(w, h) * SCALE_RADIAL_HALFDIAG),
        gl, bn, gy);
}

/* SHEAR_X — horizontal sliding, like a deck of cards pushed sideways: the top
 * drifts one way, the bottom the other, faster the farther from the middle. */
static void pattern_shear_x(int x, int y, int w, int h, float t,
                            float *gl, uint8_t *bn, char *gy)
{
    (void)x; (void)w;
    float cy = 0.5f * (float)h;
    float vx = (float)y - cy, vy = 0.0f;
    float mag = fabsf(vx);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, 0.5f * (float)h), gl, bn, gy);
}

/* UNIFORM_TILTED — the whole field drifts the same way, tilted 30 degrees
 * down-right.  Handy as a sanity check: every cell shows the same arrow
 * (until MOTION's swirl stirs it up). */
static void pattern_uniform_tilted(int x, int y, int w, int h, float t,
                                   float *gl, uint8_t *bn, char *gy)
{
    float vx = cosf((float)M_PI / 6.0f);
    float vy = sinf((float)M_PI / 6.0f);
    pattern_emit_arrow(x, y, w, h, t, vx, vy, 0.7f, gl, bn, gy);
}

/* ---------- Tier 3 — real physics fields ---------------------------- */

/* POINT_CHARGE — the electric field of a single positive charge at the
 * centre: arrows point outward and fade fast with distance. */
static void pattern_point_charge(int x, int y, int w, int h, float t,
                                 float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float dx = (float)x - cx, dy = (float)y - cy;
    float r2 = dx * dx + dy * dy + COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;
    float vx = dx / r2, vy = dy / r2;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_INV_SQ), gl, bn, gy);
}

/* DIPOLE — a plus charge on the left and a minus on the right.  Their fields
 * add up so arrows stream out of the plus, arc across, and dive into the
 * minus — the classic bar-magnet look. */
static void pattern_dipole(int x, int y, int w, int h, float t,
                           float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float d  = DIPOLE_HALF_SEP;
    float eps2 = COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;

    /* plus charge on the left */
    float ax = (float)x - (cx - d), ay = (float)y - cy;
    float ar2 = ax * ax + ay * ay + eps2;
    /* minus charge on the right — pulls the field toward it */
    float bx = (float)x - (cx + d), by = (float)y - cy;
    float br2 = bx * bx + by * by + eps2;

    float vx = ax / ar2 - bx / br2;
    float vy = ay / ar2 - by / br2;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_INV_SQ), gl, bn, gy);
}

/* WIRE_MAGNETIC — the magnetic field around a wire poking straight out of the
 * screen with current running through it.  The field wraps in circles around
 * the wire and weakens with distance. */
static void pattern_wire_magnetic(int x, int y, int w, int h, float t,
                                  float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float dx = (float)x - cx, dy = (float)y - cy;
    float r2 = dx * dx + dy * dy + COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;
    float vx = -dy / r2, vy = dx / r2;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_INV_R), gl, bn, gy);
}

/* GRAVITY — the pull of a planet sitting at the centre: arrows point inward
 * and get much stronger the closer you are. */
static void pattern_gravity(int x, int y, int w, int h, float t,
                            float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float dx = (float)x - cx, dy = (float)y - cy;
    float r2 = dx * dx + dy * dy + COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;
    float r  = sqrtf(r2);
    float vx = -dx / (r2 * r), vy = -dy / (r2 * r);
    float mag = 1.0f / r2;
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_INV_SQ), gl, bn, gy);
}

/* QUADRUPOLE — four charges in a square, alternating plus/minus like a
 * checkerboard.  Their fields combine into a four-petal pattern with sharp
 * calm lines running through the centre. */
static void pattern_quadrupole(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float d  = QUADRUPOLE_HALF;
    float eps2 = COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;
    float vx = 0.0f, vy = 0.0f;

    /* the four charges: their corner (ox, oy) and their sign (sg) */
    static const float ox[4] = { +1.0f, +1.0f, -1.0f, -1.0f };
    static const float oy[4] = { +1.0f, -1.0f, +1.0f, -1.0f };
    static const float sg[4] = { +1.0f, -1.0f, -1.0f, +1.0f };

    for (int i = 0; i < 4; i++) {
        float ax = (float)x - (cx + d * ox[i]);
        float ay = (float)y - (cy + d * oy[i]);
        float ar2 = ax * ax + ay * ay + eps2;
        vx += sg[i] * ax / ar2;
        vy += sg[i] * ay / ar2;
    }
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_INV_SQ), gl, bn, gy);
}

/* ---------- Tier 4 — fluid-like swirls ------------------------------ */

/* CURL_NOISE — a random landscape turned into swirling flow.  It looks like
 * gently churning fluid, with no spot where things pile up or drain away. */
static void pattern_curl_noise(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    float vx, vy;
    scalar_curl_2d(scalar_noise, (float)x, (float)y, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_NOISE_GRAD * NOISE_FREQ), gl, bn, gy);
}

/* The landscape behind STREAM_GRID — a bumpy egg-carton shape that, once
 * turned into flow, gives a grid of swirls spinning opposite ways. */
static float scalar_stream_grid(float x, float y)
{
    return sinf(x * STREAM_FREQ_X) * sinf(y * STREAM_FREQ_Y);
}

/* STREAM_GRID — a tiled grid of little whirlpools, each spinning the opposite
 * way to its neighbours. */
static void pattern_stream_grid(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    float vx, vy;
    scalar_curl_2d(scalar_stream_grid, (float)x, (float)y, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_BOUNDED * 0.3f), gl, bn, gy);
}

/* VORTEX_PAIR — two whirlpools side by side spinning opposite ways.  Between
 * them the flow lines up into a strong jet; farther out it fades. */
static void pattern_vortex_pair(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float d  = DIPOLE_HALF_SEP;
    float eps2 = COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;

    /* whirlpool on the left, spinning one way */
    float ax = (float)x - (cx - d), ay = (float)y - cy;
    float ar2 = ax * ax + ay * ay + eps2;
    /* whirlpool on the right, spinning the other way */
    float bx = (float)x - (cx + d), by = (float)y - cy;
    float br2 = bx * bx + by * by + eps2;

    float vx = (-ay / ar2) + ( by / br2);
    float vy = ( ax / ar2) + (-bx / br2);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_INV_R), gl, bn, gy);
}

/* CHANNEL_FLOW — water flowing through a pipe: everything moves right, fastest
 * down the middle and slowing to a stop at the top and bottom walls. */
static void pattern_channel_flow(int x, int y, int w, int h, float t,
                                 float *gl, uint8_t *bn, char *gy)
{
    (void)w;
    float u = 2.0f * (float)y / (float)(h - 1) - 1.0f;     /* -1 at one wall, +1 at the other */
    float vx = 1.0f - u * u;                               /* full speed at the middle, 0 at walls */
    float vy = 0.0f;
    pattern_emit_arrow(x, y, w, h, t, vx, vy, vx * 0.9f, gl, bn, gy);
}

/* NOISY_UNIFORM — a steady rightward flow with a little random jitter mixed
 * in, so it reads like a breeze with light turbulence. */
static void pattern_noisy_uniform(int x, int y, int w, int h, float t,
                                  float *gl, uint8_t *bn, char *gy)
{
    float nx, ny;
    scalar_gradient(scalar_noise, (float)x, (float)y, &nx, &ny);
    float vx = 1.0f + nx * 4.0f;
    float vy =        ny * 4.0f;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, 1.5f), gl, bn, gy);
}

/* ---------- Tier 5 — phase portraits -------------------------------- *
 *
 * A phase portrait is a map of "what happens next" for a system.  Each spot
 * on screen stands for a possible state, and the arrow there shows which way
 * the state would move from there.  Trace the arrows and you see whether the
 * system settles down, spins forever, or oscillates.
 */

/* Map a screen cell to the spot it represents in the system's own coordinate
 * range.  half_extent says how far that range reaches from the centre. */
static inline void cell_to_phase(int x, int y, int w, int h, float half_extent,
                                 float *xp, float *yp)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    *xp = ((float)x - cx) * (half_extent / cx);
    *yp = ((float)y - cy) * (half_extent / cy);
}

/* STABLE_NODE — a system that always settles to rest: every arrow points
 * straight back to the centre, so wherever you start you slide home. */
static void pattern_stable_node(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    float xp, yp;
    cell_to_phase(x, y, w, h, PHASE_HALF_EXTENT, &xp, &yp);
    float vx = -xp, vy = -yp;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_PHASE), gl, bn, gy);
}

/* STABLE_SPIRAL — a system that settles down while circling, like a marble
 * spiralling into the bottom of a bowl. */
static void pattern_stable_spiral(int x, int y, int w, int h, float t,
                                  float *gl, uint8_t *bn, char *gy)
{
    float xp, yp;
    cell_to_phase(x, y, w, h, PHASE_HALF_EXTENT, &xp, &yp);
    float vx = -xp - yp;
    float vy =  xp - yp;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_PHASE), gl, bn, gy);
}

/* HOPF_CYCLE — a system that settles into a steady loop.  There's a ring it
 * wants to be on: start inside and you spiral out to it, start outside and you
 * spiral in to it, and on the ring you just circle around. */
static void pattern_hopf_cycle(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    float xp, yp;
    cell_to_phase(x, y, w, h, PHASE_HALF_EXTENT, &xp, &yp);
    float r2  = xp * xp + yp * yp;       /* squared distance from centre */
    float mu_minus_r2 = HOPF_MU - r2;    /* positive inside the ring, negative outside */
    float vx = mu_minus_r2 * xp - yp;
    float vy = xp + mu_minus_r2 * yp;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_PHASE), gl, bn, gy);
}

/* VAN_DER_POL — an oscillator that always falls into the same loop, but a
 * lopsided one rather than a clean circle (van der Pol, 1926).  Wherever you
 * start, you end up on that one loop; sitting still at the centre is unstable. */
static void pattern_van_der_pol(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    float xp, yp;
    cell_to_phase(x, y, w, h, PHASE_HALF_EXTENT, &xp, &yp);
    float vx = yp;
    float vy = VDP_MU * (1.0f - xp * xp) * yp - xp;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_PHASE * 2.0f), gl, bn, gy);
}

/* PENDULUM — a frictionless swing.  Across the screen is the angle, up/down is
 * how fast it's swinging.  Near the centre you see small back-and-forth swings
 * as closed loops; near the top and bottom the pendulum has enough speed to go
 * all the way over, shown as bands that run off the edges. */
static void pattern_pendulum(int x, int y, int w, int h, float t,
                             float *gl, uint8_t *bn, char *gy)
{
    /* across = angle (-pi to +pi); up/down = swing speed */
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float xp = ((float)x - cx) * (PENDULUM_X_HALF_EXTENT / cx);
    float yp = ((float)y - cy) * (PHASE_HALF_EXTENT / cy);
    float vx = yp;
    float vy = -sinf(xp);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_PHASE), gl, bn, gy);
}

/* ---------- Tier 6 — fields that move on their own ------------------ */

/* ROTATING_DIPOLE — the DIPOLE field, but the two charges slowly orbit the
 * centre so the whole pattern turns like the hands of a clock. */
static void pattern_rotating_dipole(int x, int y, int w, int h, float t,
                                    float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float omega = 2.0f * (float)M_PI / ROT_DIPOLE_PERIOD;
    float ang = omega * t;
    float dx_pole = DIPOLE_HALF_SEP * cosf(ang);
    float dy_pole = DIPOLE_HALF_SEP * sinf(ang);
    float eps2 = COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;

    /* plus charge on one side of the centre, minus directly opposite */
    float ax = (float)x - (cx + dx_pole), ay = (float)y - (cy + dy_pole);
    float ar2 = ax * ax + ay * ay + eps2;
    float bx = (float)x - (cx - dx_pole), by = (float)y - (cy - dy_pole);
    float br2 = bx * bx + by * by + eps2;

    float vx = ax / ar2 - bx / br2;
    float vy = ay / ar2 - by / br2;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_INV_SQ), gl, bn, gy);
}

/* TRAVELLING_WAVE — a wave rolling to the right; watch the bright and dark
 * stripes march across the screen. */
static void pattern_travelling_wave(int x, int y, int w, int h, float t,
                                    float *gl, uint8_t *bn, char *gy)
{
    (void)y; (void)w; (void)h;
    float k     = 0.25f;
    float omega = 2.0f * (float)M_PI / TRAVEL_WAVE_PERIOD;
    float vx    = sinf(k * (float)x - omega * t);
    float vy    = 0.0f;
    float mag   = fabsf(vx);
    cell_emit_arrow(vx, vy, mag * 0.95f, gl, bn, gy);
}

/* BREATHING_RADIAL — the radial field, pulsing in and out like a lung: it
 * blows outward, fades to a screen of dots, then sucks inward, over and over. */
static void pattern_breathing_radial(int x, int y, int w, int h, float t,
                                     float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float dx = (float)x - cx, dy = (float)y - cy;
    float omega = 2.0f * (float)M_PI / BREATHE_PERIOD;
    float s = sinf(omega * t);
    float vx = dx * s, vy = dy * s;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, grid_half_diag(w, h) * SCALE_RADIAL_HALFDIAG),
                    gl, bn, gy);
}

/* ORBITING_VORTEX — one whirlpool whose centre circles around the middle of
 * the screen, dragging its swirl along with it. */
static void pattern_orbiting_vortex(int x, int y, int w, int h, float t,
                                    float *gl, uint8_t *bn, char *gy)
{
    float cx     = 0.5f * (float)w, cy = 0.5f * (float)h;
    float half_d = grid_half_diag(w, h);
    float omega  = 2.0f * (float)M_PI / ORBIT_PERIOD;
    float r_orb  = half_d * ORBIT_RADIUS_FRAC;
    float vcx    = cx + r_orb * cosf(omega * t);
    float vcy    = cy + r_orb * sinf(omega * t);
    float eps2   = COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;

    float dx = (float)x - vcx, dy = (float)y - vcy;
    float r2 = dx * dx + dy * dy + eps2;
    float vx = -dy / r2, vy = dx / r2;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_INV_R), gl, bn, gy);
}

/* DRIFT_CURL — the curl-noise swirls of Tier 4, but the underlying noise
 * slides upward over time, so the whole thing churns and evolves like smoke. */
static void pattern_drift_curl(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    (void)w; (void)h;
    /* read the noise at four nearby points, shifted by how far it has drifted */
    float drift = t * 0.7f;
    float n_yp = noise_sample((float)x * NOISE_FREQ, ((float)y + 1.0f + drift) * NOISE_FREQ);
    float n_ym = noise_sample((float)x * NOISE_FREQ, ((float)y - 1.0f + drift) * NOISE_FREQ);
    float n_xp = noise_sample(((float)x + 1.0f) * NOISE_FREQ, ((float)y + drift) * NOISE_FREQ);
    float n_xm = noise_sample(((float)x - 1.0f) * NOISE_FREQ, ((float)y + drift) * NOISE_FREQ);
    /* turn the slope sideways to get swirling flow */
    float vx =  0.5f * (n_yp - n_ym);
    float vy = -0.5f * (n_xp - n_xm);
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_NOISE_GRAD * NOISE_FREQ),
                    gl, bn, gy);
}

/* ---------- The table that ties it all together --------------------- */

/*
 * VectorPattern — one row of the lookup table, one per field.
 *
 * Why a table: instead of a giant switch on which field is active, we just
 * look the field up here and call its function.  Adding a field is three
 * lines: write the function, add a name to the Pattern enum, add a row here.
 * Because each row is keyed by its enum name, the compiler complains if the
 * table is missing one or has them out of order.
 *
 * Members:
 *   name   — label shown in the HUD, padded to 10 characters so the columns
 *            line up neatly as you flip through fields.
 *   tier   — short "tier number + group" tag, also for the HUD.
 *   sample — the field's function (called once per cell, every tick).
 */
typedef struct {
    const char      *name;        /* padded to 10 chars for HUD alignment */
    const char      *tier;        /* padded to 7 chars  for HUD alignment */
    VectorPatternFn  sample;
} VectorPattern;

static const VectorPattern vector_patterns[N_PATTERNS] = {
    /* Tier 1 — GRADIENT */
    [PATTERN_GRAD_PARABOLOID] = { "GRAD-PARAB", "1-GRAD ", pattern_grad_paraboloid },
    [PATTERN_GRAD_SADDLE]     = { "GRAD-SADDL", "1-GRAD ", pattern_grad_saddle     },
    [PATTERN_GRAD_PERIODIC]   = { "GRAD-PERIO", "1-GRAD ", pattern_grad_periodic   },
    [PATTERN_GRAD_RIPPLE]     = { "GRAD-RIPPL", "1-GRAD ", pattern_grad_ripple     },
    [PATTERN_GRAD_NOISE]      = { "GRAD-NOISE", "1-GRAD ", pattern_grad_noise      },
    /* Tier 2 — ANALYTIC */
    [PATTERN_RADIAL_OUT]      = { "RADIAL-OUT", "2-ANLY ", pattern_radial_out      },
    [PATTERN_RADIAL_IN]       = { "RADIAL-IN ", "2-ANLY ", pattern_radial_in       },
    [PATTERN_ROTATION]        = { "ROTATION  ", "2-ANLY ", pattern_rotation        },
    [PATTERN_SHEAR_X]         = { "SHEAR-X   ", "2-ANLY ", pattern_shear_x         },
    [PATTERN_UNIFORM_TILTED]  = { "UNIF-TILT ", "2-ANLY ", pattern_uniform_tilted  },
    /* Tier 3 — PHYSICS */
    [PATTERN_POINT_CHARGE]    = { "POINT-CHG ", "3-PHYS ", pattern_point_charge    },
    [PATTERN_DIPOLE]          = { "DIPOLE    ", "3-PHYS ", pattern_dipole          },
    [PATTERN_WIRE_MAGNETIC]   = { "WIRE-MAG  ", "3-PHYS ", pattern_wire_magnetic   },
    [PATTERN_GRAVITY]         = { "GRAVITY   ", "3-PHYS ", pattern_gravity         },
    [PATTERN_QUADRUPOLE]      = { "QUADRUPOLE", "3-PHYS ", pattern_quadrupole      },
    /* Tier 4 — SOLENOID */
    [PATTERN_CURL_NOISE]      = { "CURL-NOISE", "4-SOLN ", pattern_curl_noise      },
    [PATTERN_STREAM_GRID]     = { "STREAM-GRD", "4-SOLN ", pattern_stream_grid     },
    [PATTERN_VORTEX_PAIR]     = { "VORTX-PAIR", "4-SOLN ", pattern_vortex_pair     },
    [PATTERN_CHANNEL_FLOW]    = { "CHAN-FLOW ", "4-SOLN ", pattern_channel_flow    },
    [PATTERN_NOISY_UNIFORM]   = { "NOISY-UNI ", "4-SOLN ", pattern_noisy_uniform   },
    /* Tier 5 — DYNAMICS */
    [PATTERN_STABLE_NODE]     = { "STBL-NODE ", "5-DYNS ", pattern_stable_node     },
    [PATTERN_STABLE_SPIRAL]   = { "STBL-SPRL ", "5-DYNS ", pattern_stable_spiral   },
    [PATTERN_HOPF_CYCLE]      = { "HOPF-CYCLE", "5-DYNS ", pattern_hopf_cycle      },
    [PATTERN_VAN_DER_POL]     = { "VAN-DR-POL", "5-DYNS ", pattern_van_der_pol     },
    [PATTERN_PENDULUM]        = { "PENDULUM  ", "5-DYNS ", pattern_pendulum        },
    /* Tier 6 — ANIMATED */
    [PATTERN_ROTATING_DIPOLE] = { "ROT-DIPOLE", "6-ANIM ", pattern_rotating_dipole },
    [PATTERN_TRAVELLING_WAVE] = { "TRAV-WAVE ", "6-ANIM ", pattern_travelling_wave },
    [PATTERN_BREATHING_RADIAL]= { "BREATHE   ", "6-ANIM ", pattern_breathing_radial},
    [PATTERN_ORBITING_VORTEX] = { "ORBT-VORTX", "6-ANIM ", pattern_orbiting_vortex },
    [PATTERN_DRIFT_CURL]      = { "DRIFT-CURL", "6-ANIM ", pattern_drift_curl      },
};

static const char *pattern_name(Pattern p)
{
    if ((unsigned)p >= (unsigned)N_PATTERNS) return "?         ";
    return vector_patterns[p].name;
}

static const char *pattern_tier(Pattern p)
{
    if ((unsigned)p >= (unsigned)N_PATTERNS) return "?      ";
    return vector_patterns[p].tier;
}

/* ===================================================================== */
/* §8  scene — ArrowGrid + PatternState + PaletteState + Scene            */
/* ===================================================================== */

/*
 * ArrowGrid — the worked-out "what to draw" for every cell.
 *
 * It's the hand-off between the fields and the renderer.  The active field
 * fills this in (one entry per cell: visible?, brightness, which character),
 * then the renderer just reads it and paints — the renderer never does any
 * field math.  Think of it as a tiny screen buffer, one slot per character
 * cell instead of one per pixel.
 *
 * Filled in once per tick by arrow_grid_evaluate, read once per frame by
 * arrow_grid_paint.  All in fixed BSS storage, no allocation.
 *
 * Members:
 *   w, h    — grid size in cells; set when the screen size changes.
 *   count   — w * h, kept around so the clear loop doesn't recompute it.
 *   glow[]  — is there anything to draw here? (> 0 yes, else blank).  It's a
 *             float, not a flag, leaving room for a future "how bright" use.
 *   band[]  — brightness level 0..3; the "& 3" when read is just a safety net.
 *   glyph[] — the character to draw; 0 means "draw nothing", a second safety
 *             net in case a field marked the cell visible but set no glyph.
 */
typedef struct {
    int      w, h;
    int      count;
    float    glow [CELLS_MAX];
    uint8_t  band [CELLS_MAX];
    char     glyph[CELLS_MAX];
} ArrowGrid;

static inline int arrow_grid_idx(const ArrowGrid *g, int x, int y)
{
    return y * g->w + x;
}

static void arrow_grid_reset(ArrowGrid *g, int w, int h)
{
    g->w     = w;
    g->h     = h;
    g->count = w * h;
    for (int i = 0; i < g->count; i++) {
        g->glow [i] = 0.0f;
        g->band [i] = 0;
        g->glyph[i] = 0;
    }
}

/* Run the active field over every cell, filling in the grid.  The caller
 * decides what animation time t to pass (see scene_evaluate). */
static void arrow_grid_evaluate(ArrowGrid *g, Pattern p, float t)
{
    if ((unsigned)p >= (unsigned)N_PATTERNS) return;
    VectorPatternFn sample = vector_patterns[p].sample;
    for (int y = 0; y < g->h; y++) {
        for (int x = 0; x < g->w; x++) {
            int idx = arrow_grid_idx(g, x, y);
            sample(x, y, g->w, g->h, t,
                   &g->glow [idx],
                   &g->band [idx],
                   &g->glyph[idx]);
        }
    }
}

/*
 * PatternState — which field is showing and how its animation is running.
 *
 * These three change and get read together, so they live in one place.
 *
 * Members:
 *   current    — which field is active.  Starts on GRAD_PARABOLOID.
 *   field_time — the animation clock, in seconds.  It drives the moving
 *                fields and the roaming MOTION point.  Counts up while the
 *                program runs; 'r' resets it to zero.
 *   drift_mult — animation speed, doubled/halved by +/- (kept between
 *                DRIFT_MULT_MIN and DRIFT_MULT_MAX).
 */
typedef struct {
    Pattern  current;
    float    field_time;
    int      drift_mult;
} PatternState;

static void pattern_state_init(PatternState *ps)
{
    ps->current     = PATTERN_GRAD_PARABOLOID;
    ps->field_time  = 0.0f;
    ps->drift_mult  = DRIFT_MULT_DEF;
}

/* Move the animation clock forward by one frame, scaled by the speed knob. */
static void pattern_state_advance_clock(PatternState *ps, float dt)
{
    ps->field_time += FIELD_DRIFT * (float)ps->drift_mult * dt;
}

/* Step to the next/previous field, wrapping around the ends. */
static void pattern_state_cycle_next(PatternState *ps)
{
    ps->current = (Pattern)(((int)ps->current + 1) % N_PATTERNS);
}
static void pattern_state_cycle_prev(PatternState *ps)
{
    ps->current = (Pattern)(((int)ps->current + N_PATTERNS - 1) % N_PATTERNS);
}

/* Double / halve the animation speed, kept within its limits. */
static void pattern_state_drift_faster(PatternState *ps)
{
    if (ps->drift_mult < DRIFT_MULT_MAX) ps->drift_mult *= 2;
    if (ps->drift_mult > DRIFT_MULT_MAX) ps->drift_mult  = DRIFT_MULT_MAX;
}
static void pattern_state_drift_slower(PatternState *ps)
{
    ps->drift_mult /= 2;
    if (ps->drift_mult < DRIFT_MULT_MIN) ps->drift_mult = DRIFT_MULT_MIN;
}

/*
 * PaletteState — which colour theme is showing (an index into themes[]).
 *
 * It's just one number, but giving it a name keeps the t/T key handler
 * readable.  Important: whenever this changes you must also call theme_apply()
 * so ncurses actually switches to the new colours — this only remembers the
 * choice, ncurses holds the live colours.
 *
 * Member:
 *   current — which theme, from 0 to N_THEMES-1.
 */
typedef struct {
    int current;
} PaletteState;

static void palette_state_init(PaletteState *p) { p->current = 0; }

static void palette_state_cycle_next(PaletteState *p)
{
    p->current = (p->current + 1) % N_THEMES;
}
static void palette_state_cycle_prev(PaletteState *p)
{
    p->current = (p->current + N_THEMES - 1) % N_THEMES;
}

/*
 * Scene — everything that can change while the program runs, in one place.
 *
 * The main loop, the per-tick update, and the renderer all share this.  The
 * pieces flow into each other in order:
 *
 *   noise   — the random seed              -> feeds the noise-based fields
 *   pattern — which field + its clock       -> drives the per-cell work
 *   grid    — the worked-out per-cell draw  -> read by the renderer
 *   palette — which colour theme            -> turns brightness into colour
 *   plus two on/off switches: paused and motion.
 *
 * Lives in fixed storage on App; never freed (the OS cleans up at exit).
 *   scene_init  sets defaults then calls scene_reset.
 *   scene_reset re-seeds the noise, clears the grid and clock, draws once.
 *               Runs on 'r' and when the window is resized.
 *   scene_tick  advances the clock and recomputes the grid.
 *
 * Members:
 *   noise, grid, pattern, palette — the four pieces above.
 *   paused — space bar.  Freezes the clock so the picture holds still (the
 *            renderer keeps showing the last computed frame).
 *   motion — the 'm' key.  Turns on the roaming-swirl blend for the
 *            non-moving fields.
 */
typedef struct {
    NoiseField   noise;
    ArrowGrid    grid;
    PatternState pattern;
    PaletteState palette;
    bool         paused;     /* space — hold the picture still */
    bool         motion;     /* m     — blend in the roaming swirl */
} Scene;

/* Run the active field over the grid.  The moving fields always get the live
 * clock; the others get 0 (frozen) unless MOTION is on, since the roaming
 * swirl only kicks in when the clock is running. */
static void scene_evaluate(Scene *s)
{
    Pattern p = s->pattern.current;
    float   t = (s->motion || pattern_is_animated_tier(p))
              ? s->pattern.field_time : 0.0f;
    arrow_grid_evaluate(&s->grid, p, t);
}

static void scene_reset(Scene *s, int mw, int mh)
{
    noise_field_reseed(&s->noise);
    arrow_grid_reset  (&s->grid, mw, mh);
    s->pattern.field_time = 0.0f;
    scene_evaluate(s);                          /* fill in the first frame */
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    pattern_state_init (&s->pattern);
    palette_state_init (&s->palette);
    s->paused = false;
    s->motion = false;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    pattern_state_advance_clock(&s->pattern, dt);
    scene_evaluate(s);
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

/*
 * Screen — how big the terminal is right now (width and height in characters).
 *
 * We remember it so the renderer can centre the grid and right-align the HUD
 * without asking ncurses every time we draw.  Refreshed at startup and on
 * resize.
 *
 * Members:
 *   cols — width  in characters.
 *   rows — height in characters (includes the HUD rows; subtract
 *          HUD_BAND_RESERVED_ROWS to get the area the field can use).
 */
typedef struct { int cols, rows; } Screen;

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

/* ---------- drawing the field --------------------------------------- */

/* Work out the top-left corner so the grid sits centred, leaving room for the
 * HUD rows top and bottom.  If the grid is bigger than the space, it's nudged
 * back to the HUD edge rather than overlapping it. */
static void viewport_centre_grid_origin(int cols, int rows,
                                        int grid_w, int grid_h,
                                        int *gx0, int *gy0)
{
    int interior_h = rows - HUD_BAND_RESERVED_ROWS;
    *gx0 = (cols       - grid_w) / 2;
    *gy0 = (interior_h - grid_h) / 2 + HUD_TOP_ROWS;
    if (*gx0 < 0)            *gx0 = 0;
    if (*gy0 < HUD_TOP_ROWS) *gy0 = HUD_TOP_ROWS;
}

/* Draw one cell on screen, skipping it if it's blank or has no glyph. */
static void arrow_cell_paint(const ArrowGrid *g, int gx, int gy, int sx, int sy)
{
    int  idx = arrow_grid_idx(g, gx, gy);
    if (g->glow[idx] <= 0.0f) return;
    char glyph = g->glyph[idx];
    if (glyph == 0 || glyph == ' ') return;

    int pair = PAIR_BAND_BASE + (g->band[idx] & 3);
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Draw the whole grid: centre it, then paint each cell that's on screen.
 * The fields already decided what each cell looks like; this just paints it. */
static void arrow_grid_paint(const ArrowGrid *g, int cols, int rows)
{
    int gx0, gy0;
    viewport_centre_grid_origin(cols, rows, g->w, g->h, &gx0, &gy0);

    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            arrow_cell_paint(g, x, y, sx, sy);
        }
    }
}

/* ---------- the HUD ------------------------------------------------- */

/* The title chip in the top-left corner. */
static void hud_draw_top_left_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " VECTOR FIELD ARROWS ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* The status readout in the top-right corner: speed, which field, etc. */
static void hud_draw_top_right_status(int cols, double fps, int sim_fps,
                                      const char *state_str,
                                      int current_idx_zero_based,
                                      int drift_mult, bool motion)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  drift:x%-2d  motion:%s ",
             fps, sim_fps, state_str,
             current_idx_zero_based + 1, N_PATTERNS,
             drift_mult, motion ? "ON " : "off");
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Draw one labelled field on the status row and return where the next one
 * should start. */
static int hud_field_bold_label(int x, const char *fmt,
                                const char *val, int width)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, fmt, val);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + width;
}

/* Draw "palette:####" — a label, then one '#' in each of the four theme
 * colours so you can preview the current theme.  Returns the next x. */
static int hud_field_palette_swatch(int x)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += HUD_PALETTE_LABEL_W;
    for (int i = 0; i < HUD_N_PALETTE_BANDS; i++) {
        int p = PAIR_BAND_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, '#');
        attroff(COLOR_PAIR(p) | A_BOLD);
        x += 1;
    }
    return x;
}

/* The tail of the status row: the clock and the grid size. */
static void hud_field_meta(int x, float field_time, int grid_w, int grid_h)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  t:%.1fs  map:%dx%d ",
             (double)field_time, grid_w, grid_h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* The full status row, laid out left to right: field name, tier, theme,
 * colour preview, then the clock and grid size. */
static void hud_draw_param_row(const Scene *s)
{
    const PatternState *ps = &s->pattern;
    int x = HUD_LEFT_MARGIN;
    x = hud_field_bold_label(x, " pattern:%-10s ", pattern_name(ps->current),
                             HUD_PATTERN_FIELD_W);
    x = hud_field_bold_label(x, " tier:%-7s ",     pattern_tier(ps->current),
                             HUD_TIER_FIELD_W);
    x = hud_field_bold_label(x, " theme:%-8s ",    themes[s->palette.current].name,
                             HUD_THEME_FIELD_W);
    x = hud_field_palette_swatch(x);
    hud_field_meta(x, ps->field_time, s->grid.w, s->grid.h);
}

/* The list of keys along the bottom edge. */
static void hud_draw_bottom_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:pattern  t/T:theme  m:motion  +/-:drift  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* Draw one whole frame: clear, paint the field, then lay the HUD on top. */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    arrow_grid_paint(&s->grid, sc->cols, sc->rows);

    const PatternState *ps        = &s->pattern;
    const char         *state_str = s->paused ? "PAUSED    "
                                              : pattern_name(ps->current);
    hud_draw_top_left_title();
    hud_draw_top_right_status(sc->cols, fps, sim_fps, state_str,
                              (int)ps->current, ps->drift_mult, s->motion);
    hud_draw_param_row(s);
    hud_draw_bottom_hint(sc->rows);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §10 app                                                                */
/* ===================================================================== */

/*
 * App — the whole program in one struct.
 *
 * It lives as a single global (g_app) for one specific reason: the signal
 * handlers below need to flip a couple of flags, and a signal handler can only
 * safely touch global state.  Everything else here is used only by the main
 * loop.
 *
 * Members:
 *   scene        — all the simulation state (see Scene above).
 *   screen       — the terminal size.
 *   sim_fps      — how many times a second the sim updates; changed by ]/[.
 *                  Separate from how fast we draw.
 *   map_w, map_h — the grid size, worked out from the terminal size and
 *                  clamped to sane limits.
 *   running      — set to 0 to quit (by Ctrl-C, the window manager, or q/ESC).
 *   need_resize  — set when the terminal is resized; the main loop notices and
 *                  rebuilds at the new size.
 * The last two are the flags the signal handlers touch.
 *
 * The main loop's timing follows Glenn Fiedler's "Fix Your Timestep!".
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

/* Pick the grid size from the terminal size, leaving room for the HUD rows
 * and staying within the limits the storage allows. */
static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS;
    if (mw < MAP_W_MIN) mw = MAP_W_MIN;
    if (mh < MAP_H_MIN) mh = MAP_H_MIN;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

/* React to a window resize: re-read the size, repick the grid, rebuild. */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* Speed up / slow down how often the sim updates (the ]/[ keys). */
static void app_sim_rate_faster(App *app)
{
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
}
static void app_sim_rate_slower(App *app)
{
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

/* Handle one keypress.  Returns false only when the user asks to quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */:
        return false;

    case ' ':
        s->paused = !s->paused;
        break;
    case 'r': case 'R':
        scene_reset(s, app->map_w, app->map_h);
        break;
    case 'm': case 'M':
        s->motion = !s->motion;     /* toggle the roaming swirl */
        break;

    case '=': case '+':  pattern_state_drift_faster(&s->pattern); break;
    case '-':            pattern_state_drift_slower(&s->pattern); break;
    case ']':            app_sim_rate_faster       ( app);        break;
    case '[':            app_sim_rate_slower       ( app);        break;
    case 'n': case 'N':  pattern_state_cycle_next  (&s->pattern); break;
    case 'p': case 'P':  pattern_state_cycle_prev  (&s->pattern); break;

    case 't':
        palette_state_cycle_next(&s->palette);
        theme_apply(s->palette.current);
        break;
    case 'T':
        palette_state_cycle_prev(&s->palette);
        theme_apply(s->palette.current);
        break;

    default: break;
    }
    return true;
}

/* ---------- main loop ----------------------------------------------- */

/* Set up clean shutdown on Ctrl-C and react-to-resize.  The handlers only
 * flip flags; the main loop does the actual work. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* Start everything up: ncurses, the screen size, and the initial scene. */
static void app_bootstrap(App *app)
{
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);
}

/* If a resize is pending, handle it and restart the frame clock, so the time
 * the resize took isn't counted as one giant slow frame. */
static void app_handle_pending_resize(App *app,
                                      int64_t *frame_time,
                                      int64_t *sim_accum)
{
    if (!app->need_resize) return;
    app_do_resize(app);
    *frame_time = clock_ns();
    *sim_accum  = 0;
}

/* How much time passed since the last frame, capped so one slow frame can't
 * make the sim try to catch up forever. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    int64_t dt_cap = (int64_t)SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    if (dt > dt_cap) dt = dt_cap;
    return dt;
}

/* Run as many fixed-size sim steps as fit into the time that passed, so the
 * sim keeps a steady rate no matter how the drawing speed jitters. */
static void app_drain_fixed_timestep(App *app, int64_t dt, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    *sim_accum += dt;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* Keep the fps number fresh by averaging over the last half-second or so. */
static void app_update_fps_meter(int64_t dt,
                                 int *frame_count,
                                 int64_t *fps_accum,
                                 double *fps_display)
{
    (*frame_count)++;
    *fps_accum += dt;
    if (*fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
        *fps_display = (double)(*frame_count)
                     / ((double)(*fps_accum) / (double)NS_PER_SEC);
        *frame_count = 0;
        *fps_accum   = 0;
    }
}

/* Sleep off whatever's left of this frame's time budget so we don't peg the CPU. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* Draw the frame and push it to the terminal. */
static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* Check for a keypress (without blocking) and act on it.  Returns false only
 * if the user asked to quit. */
static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* Start up, then loop every frame: handle resizes, advance the sim, draw, and
 * check for input — until the user quits. */
int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    main_install_signal_handlers();
    App *app = &g_app;
    app_bootstrap(app);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {
        app_handle_pending_resize    (app, &frame_time, &sim_accum);
        int64_t dt = app_compute_frame_dt(&frame_time);
        app_drain_fixed_timestep     (app, dt, &sim_accum);
        app_update_fps_meter         (dt, &frame_count, &fps_accum, &fps_display);
        app_throttle_to_render_target(frame_time, dt);
        app_present_frame            (app, fps_display);
        if (!app_poll_keyboard(app)) app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
