/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * complex_flowfield.c — ncurses ASCII complex flow-field visualiser
 *
 * Four swappable physics algorithms drive tracer particles through a
 * 2-D vector field.  Six cosine-palette color themes make each field
 * mode look visually distinct.  Three background modes range from a
 * blank canvas to a full-screen angle colormap.
 *
 * FIELD TYPES (cycle 'a'):
 *   0  Curl noise     — divergence-free turbulence from ∂noise/∂y, −∂noise/∂x
 *   1  Vortex lattice — N_VORT Biot-Savart point vortices orbiting screen centre
 *   2  Sine lattice   — crossing sine/cosine waves forming interference patterns
 *   3  Radial spiral  — polar mix of tangential (galaxy) + pulsing radial force
 *
 * COLOR THEMES (cycle 't'): cosine palette  a + b·cos(2π·(c·t + d))
 *   0 Cosmic  1 Ember  2 Ocean  3 Neon  4 Sunset  5 Mono
 *
 * BACKGROUND MODES (cycle 'v'):
 *   0  blank      — dark canvas, particle trails only
 *   1  arrows     — dim directional glyphs tinted by angle
 *   2  colormap   — every cell painted by flow angle → full palette
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   r         reset  — respawn all particles, rewind field time
 *   a         next field type
 *   t         next color theme
 *   v         next background mode
 *   ]  [      sim fps up / down
 *   +  -      more / fewer particles
 *   f  F      field evolution speed up / down
 *   s  S      trail length up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra complex_flowfield.c -o complex_flowfield -lncurses -lm
 *
 * Sections
 * --------
 *   §1  presets      — all named constants, grouped by subsystem
 *   §2  clock        — monotonic ns clock + sleep
 *   §3  cosine pal   — cosine palette; 6 themes; 16 ncurses color pairs
 *   §4  noise        — layered Perlin noise (2-D + time)
 *   §5  field        — 4 field modes; vortex state; tick; sample
 *   §6  particle     — tracer pool; spawn; tick; trail draw
 *   §7  scene        — owns field + particle pool; tick; draw
 *   §8  screen       — ncurses layer; HUD
 *   §9  app          — signals; resize; main loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm:
 *   Four distinct vector field generators each produce an angle per cell.
 *   Particles follow this angle field (Lagrangian advection: dx/dt = V(x,t)).
 *   The shared rendering path colors every particle by its movement angle
 *   via a cosine palette so adjacent particles with different directions
 *   show complementary hues rather than arbitrary color steps.
 *
 * Cosine palette:
 *   color(t) = a + b · cos(2π · (c·t + d))
 *   where a (bias), b (amplitude), c (frequency), d (phase) are 3-vectors.
 *   Choosing c,d carefully creates perceptually smooth, complementary gradients
 *   — the hue rotates continuously rather than jumping between fixed colors.
 *   16 evenly-spaced samples of t ∈ [0,1] are registered as ncurses pairs
 *   so the palette is pre-baked once per theme change.
 *
 * Curl noise (field 0):
 *   Build a scalar potential ψ(x,y,t) from layered Perlin noise.
 *   The curl of a 2-D scalar field gives a divergence-free vector field:
 *     Vx = ∂ψ/∂y ≈ (ψ(x, y+ε) − ψ(x, y−ε)) / (2ε)
 *     Vy = −∂ψ/∂x ≈ −(ψ(x+ε, y, t) − ψ(x−ε, y, t)) / (2ε)
 *   Divergence-free means no sources or sinks — particles orbit without
 *   accumulating, producing infinite looping motion.
 *
 * Vortex lattice (field 1):
 *   N_VORT point vortices evenly distributed on a ring.  Alternating
 *   CCW/CW strengths.  Each tick the ring rotates by VORT_ORB_SPD radians.
 *   Velocity at (px,py) from Biot-Savart regularised by VORT_EPS:
 *     Vx += S·(−dy) / (r² + ε),   Vy += S·dx / (r² + ε)
 *   Terminal cell aspect ratio (CELL_AR = 0.5) corrects dy so circles look
 *   round rather than squashed.
 *
 * Sine lattice (field 2):
 *   Superposition of two crossing wave pairs, each advancing in time:
 *     Vx = sin(x·Fx + t) + sin(y·Fy − t·0.7)
 *     Vy = cos(x·Fx − t·0.5) + cos(y·Fy + t·0.3)
 *   Creates standing-wave interference patterns that evolve over time —
 *   ripples that fold and cross, very different from the organic curl.
 *
 * Radial spiral (field 3):
 *   In polar coords relative to screen centre, pure tangential flow is
 *   a CCW vortex.  Adding a time-pulsing radial component makes particles
 *   spiral alternately outward and inward — a breathing galaxy effect.
 *     Vx = −sin(θ) + W·sin(t)·cos(θ)
 *     Vy = cos(θ) + W·sin(t)·sin(θ)   (W = SPIRAL_RADIAL_W)
 *
 * Background colormap (bg_mode 2):
 *   The flow angle at each cell (clamped to [0,1] after normalisation)
 *   directly indexes the cosine palette, painting the entire screen with
 *   complementary hues that reveal the field topology.  Particle trails
 *   drawn on top at A_BOLD remain clearly visible over the colormap.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  presets                                                            */
/* ===================================================================== */

/* ── loop / display ─────────────────────────────────────────────────── */
enum {
    SIM_FPS_MIN      =  5,
    SIM_FPS_DEFAULT  = 30,
    SIM_FPS_MAX      = 60,
    SIM_FPS_STEP     =  5,

    HUD_COLS         = 60,
    FPS_UPDATE_MS    = 500,

    N_FIELD_TYPES    =  4,    /* curl, vortex, sine, spiral                 */
    N_THEMES         =  6,    /* cosmic, ember, ocean, neon, sunset, mono   */
    N_BG_MODES       =  3,    /* blank, arrows, colormap                    */
    N_PAIRS          = 16,    /* cosine palette samples; pairs CP_BASE..+15 */
    CP_BASE          =  1,    /* first ncurses color pair used              */
};

/* ── particles ──────────────────────────────────────────────────────── *
 * Pool size, defaults, and physics constants.
 * ─────────────────────────────────────────────────────────────────────*/
#define PARTS_MIN          50
#define PARTS_DEFAULT     400
#define PARTS_MAX         800
#define PARTS_STEP         50

#define PART_SPEED         0.85f   /* base movement per tick in cells       */
#define PART_SPD_JITTER    0.40f   /* ± random offset to desync particles   */
#define PART_LIFE_BASE    120      /* base lifetime in ticks                */
#define PART_LIFE_JITTER   80      /* random range added to lifetime        */

/* ── trail ──────────────────────────────────────────────────────────── */
#define TRAIL_LEN_MIN      3
#define TRAIL_LEN_DEFAULT 18
#define TRAIL_LEN_MAX     24
#define TRAIL_MAX         24      /* ring-buffer hard limit                 */

/* ── field evolution speed ──────────────────────────────────────────── *
 * The field time axis advances by this amount per sim tick.
 * Low = slow drift.  High = rapid churning.
 * ─────────────────────────────────────────────────────────────────────*/
#define FIELD_SPD_DEFAULT  0.006f
#define FIELD_SPD_MIN      0.001f
#define FIELD_SPD_MAX      0.08f
#define FIELD_SPD_STEP     1.5f    /* multiplicative step for f/F keys      */

/* ── field 0: curl noise ─────────────────────────────────────────────
 *   CURL_SX/SY     : spatial scale for noise sampling
 *   CURL_EPS       : central-difference step for curl computation
 *   CURL_OCTAVES   : layered octaves (more = more fractal detail)
 * ─────────────────────────────────────────────────────────────────────*/
#define CURL_SX        0.030f
#define CURL_SY        0.055f
#define CURL_EPS       1.2f
#define CURL_OCTAVES   3

/* ── field 1: vortex lattice ─────────────────────────────────────────
 *   N_VORT         : number of point vortices in the ring
 *   VORT_RING_FRAC : ring radius as fraction of min(cols,rows)/2
 *   VORT_ORB_SPD   : angular speed of the ring in radians/tick
 *   VORT_STRENGTH  : |Biot-Savart strength|; sign alternates CCW/CW
 *   VORT_EPS       : denominator softening to prevent singularity
 *   CELL_AR        : approximate terminal char width/height ratio
 *                    (corrects Biot-Savart so orbits look circular)
 * ─────────────────────────────────────────────────────────────────────*/
#define N_VORT         6
#define VORT_RING_FRAC 0.28f
#define VORT_ORB_SPD   0.014f
#define VORT_STRENGTH  3.0f
#define VORT_EPS       5.0f
#define CELL_AR        0.5f

/* ── field 2: sine lattice ───────────────────────────────────────────
 *   SINE_XFREQ/YFREQ : spatial frequencies for the wave components
 * ─────────────────────────────────────────────────────────────────────*/
#define SINE_XFREQ     0.055f
#define SINE_YFREQ     0.095f

/* ── field 3: radial spiral ──────────────────────────────────────────
 *   SPIRAL_RADIAL_W  : weight of the pulsing radial component (0=pure vortex)
 * ─────────────────────────────────────────────────────────────────────*/
#define SPIRAL_RADIAL_W 0.65f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

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
    struct timespec r = { (time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  cosine palette                                                     */
/* ===================================================================== */

/*
 * Cosine palette formula (Inigo Quilez):
 *   color(t) = a + b · cos(2π · (c·t + d))
 *
 * where a (bias), b (amplitude), c (frequency), d (phase) are RGB 3-vectors.
 * t ∈ [0, 1] is the palette parameter.
 *
 * At t = 0 the result is a + b·cos(2π·d).
 * As t increases the cosine oscillates, creating smooth complementary hue
 * transitions.  Choosing c near 1.0 gives one full hue cycle across [0,1];
 * fractional c values give partial rotations with a clear dominant hue.
 *
 * 16 evenly-spaced t values are pre-baked into ncurses color pairs.
 * angle_to_pair() maps a flow angle ∈ [−π, π) to the matching pair.
 */
typedef struct {
    float a[3];        /* bias      per R,G,B channel */
    float b[3];        /* amplitude per R,G,B channel */
    float c[3];        /* frequency per R,G,B channel */
    float d[3];        /* phase     per R,G,B channel */
    const char *name;
} CosTheme;

/*
 * Six themes — parameter comments show the dominant color journey.
 *
 *   Cosmic : electric violet → bright cyan → hot magenta (nebula)
 *   Ember  : deep red → orange → pale yellow (fire embers)
 *   Ocean  : navy → teal → ice-cyan (deep water)
 *   Neon   : electric green → hot pink → violet (club lighting)
 *   Sunset : dark purple → crimson → amber → gold (evening sky)
 *   Mono   : silver-blue → white → silver-blue (clean grayscale)
 */
static const CosTheme k_themes[N_THEMES] = {
    /* 0  Cosmic */
    { {0.50f, 0.50f, 0.50f},
      {0.50f, 0.50f, 0.50f},
      {1.00f, 1.00f, 0.50f},
      {0.80f, 0.90f, 0.30f},
      "cosmic" },

    /* 1  Ember */
    { {0.55f, 0.30f, 0.05f},
      {0.45f, 0.30f, 0.05f},
      {1.00f, 0.80f, 0.30f},
      {0.00f, 0.10f, 0.25f},
      "ember" },

    /* 2  Ocean */
    { {0.15f, 0.40f, 0.60f},
      {0.20f, 0.35f, 0.40f},
      {0.50f, 0.70f, 1.00f},
      {0.00f, 0.10f, 0.30f},
      "ocean" },

    /* 3  Neon */
    { {0.50f, 0.50f, 0.50f},
      {0.50f, 0.50f, 0.50f},
      {1.00f, 0.50f, 1.00f},
      {0.00f, 0.50f, 0.33f},
      "neon" },

    /* 4  Sunset */
    { {0.50f, 0.38f, 0.30f},
      {0.50f, 0.38f, 0.30f},
      {1.00f, 0.85f, 0.60f},
      {0.00f, 0.18f, 0.40f},
      "sunset" },

    /* 5  Mono */
    { {0.45f, 0.48f, 0.55f},
      {0.40f, 0.42f, 0.45f},
      {0.50f, 0.50f, 0.50f},
      {0.00f, 0.02f, 0.05f},
      "mono" },
};

/*
 * cos_to_xterm256() — map cosine palette at parameter t to xterm-256 index.
 *
 * The xterm-256 6×6×6 color cube occupies indices 16–231:
 *   index = 16 + 36·r + 6·g + b   (r,g,b ∈ [0,5])
 * We map each float channel to the nearest of the 6 cube levels.
 */
static int cos_to_xterm256(const CosTheme *th, float t)
{
    float rf = th->a[0] + th->b[0] * cosf(2.f*(float)M_PI*(th->c[0]*t + th->d[0]));
    float gf = th->a[1] + th->b[1] * cosf(2.f*(float)M_PI*(th->c[1]*t + th->d[1]));
    float bf = th->a[2] + th->b[2] * cosf(2.f*(float)M_PI*(th->c[2]*t + th->d[2]));

    /* clamp [0, 1] */
    if (rf < 0.f) rf = 0.f; else if (rf > 1.f) rf = 1.f;
    if (gf < 0.f) gf = 0.f; else if (gf > 1.f) gf = 1.f;
    if (bf < 0.f) bf = 0.f; else if (bf > 1.f) bf = 1.f;

    int r5 = (int)(rf * 5.f + 0.5f);
    int g5 = (int)(gf * 5.f + 0.5f);
    int b5 = (int)(bf * 5.f + 0.5f);
    if (r5 > 5) r5 = 5;
    if (g5 > 5) g5 = 5;
    if (b5 > 5) b5 = 5;
    return 16 + 36*r5 + 6*g5 + b5;
}

/*
 * Fallback 8-color pairs (one row per theme, N_PAIRS entries each).
 * Each 4-color block repeats with a brightness variation to fill 16 slots.
 */
static const int k_fallback[N_THEMES][N_PAIRS] = {
    /* 0 cosmic  */ {COLOR_MAGENTA,COLOR_CYAN,COLOR_BLUE,COLOR_WHITE,
                     COLOR_MAGENTA,COLOR_CYAN,COLOR_BLUE,COLOR_WHITE,
                     COLOR_MAGENTA,COLOR_CYAN,COLOR_BLUE,COLOR_WHITE,
                     COLOR_MAGENTA,COLOR_CYAN,COLOR_BLUE,COLOR_WHITE},
    /* 1 ember   */ {COLOR_RED,COLOR_RED,COLOR_YELLOW,COLOR_WHITE,
                     COLOR_RED,COLOR_RED,COLOR_YELLOW,COLOR_WHITE,
                     COLOR_RED,COLOR_RED,COLOR_YELLOW,COLOR_WHITE,
                     COLOR_RED,COLOR_RED,COLOR_YELLOW,COLOR_WHITE},
    /* 2 ocean   */ {COLOR_BLUE,COLOR_CYAN,COLOR_CYAN,COLOR_WHITE,
                     COLOR_BLUE,COLOR_CYAN,COLOR_CYAN,COLOR_WHITE,
                     COLOR_BLUE,COLOR_CYAN,COLOR_CYAN,COLOR_WHITE,
                     COLOR_BLUE,COLOR_CYAN,COLOR_CYAN,COLOR_WHITE},
    /* 3 neon    */ {COLOR_GREEN,COLOR_MAGENTA,COLOR_BLUE,COLOR_WHITE,
                     COLOR_GREEN,COLOR_MAGENTA,COLOR_BLUE,COLOR_WHITE,
                     COLOR_GREEN,COLOR_MAGENTA,COLOR_BLUE,COLOR_WHITE,
                     COLOR_GREEN,COLOR_MAGENTA,COLOR_BLUE,COLOR_WHITE},
    /* 4 sunset  */ {COLOR_MAGENTA,COLOR_RED,COLOR_YELLOW,COLOR_WHITE,
                     COLOR_MAGENTA,COLOR_RED,COLOR_YELLOW,COLOR_WHITE,
                     COLOR_MAGENTA,COLOR_RED,COLOR_YELLOW,COLOR_WHITE,
                     COLOR_MAGENTA,COLOR_RED,COLOR_YELLOW,COLOR_WHITE},
    /* 5 mono    */ {COLOR_WHITE,COLOR_WHITE,COLOR_WHITE,COLOR_WHITE,
                     COLOR_WHITE,COLOR_WHITE,COLOR_WHITE,COLOR_WHITE,
                     COLOR_WHITE,COLOR_WHITE,COLOR_WHITE,COLOR_WHITE,
                     COLOR_WHITE,COLOR_WHITE,COLOR_WHITE,COLOR_WHITE},
};

static void color_apply_theme(int theme)
{
    const CosTheme *th = &k_themes[theme];
    for (int i = 0; i < N_PAIRS; i++) {
        float t = (float)i / (float)(N_PAIRS - 1);  /* evenly spaced in [0,1] */
        if (COLORS >= 256) {
            int fg = cos_to_xterm256(th, t);
            init_pair(CP_BASE + i, fg, COLOR_BLACK);
        } else {
            init_pair(CP_BASE + i, k_fallback[theme][i], COLOR_BLACK);
        }
    }
}

static void color_init(int theme)
{
    start_color();
    color_apply_theme(theme);
}

/*
 * angle_to_pair() — map flow angle ∈ (−π, π] to color pair CP_BASE..CP_BASE+N_PAIRS-1.
 *
 * Normalises angle to [0,1] then scales to pair index.  This means angle
 * and the cosine palette parameter t use the same [0,1] space — particles
 * flowing in opposite directions get complementary colors (half-cycle apart).
 */
static int angle_to_pair(float angle)
{
    float a = angle;
    if (a < 0.f) a += 2.f * (float)M_PI;
    int idx = (int)(a / (2.f * (float)M_PI) * N_PAIRS) % N_PAIRS;
    return CP_BASE + idx;
}

/* ===================================================================== */
/* §4  noise                                                              */
/* ===================================================================== */

static uint8_t perm[512];

static void noise_init(void)
{
    uint8_t p[256];
    for (int i = 0; i < 256; i++) p[i] = (uint8_t)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t tmp = p[i]; p[i] = p[j]; p[j] = tmp;
    }
    for (int i = 0; i < 512; i++) perm[i] = p[i & 255];
}

static inline float fade(float t)  { return t * t * (3.f - 2.f * t); }
static inline float lerpf(float a, float b, float t) { return a + t * (b - a); }

static inline float grad2(int h, float x, float y)
{
    int H = h & 3;
    float u = (H < 2) ? x : y;
    float v = (H < 2) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -v : v);
}

static float noise2(float x, float y)
{
    int   xi = (int)floorf(x) & 255;
    int   yi = (int)floorf(y) & 255;
    float xf = x - floorf(x);
    float yf = y - floorf(y);
    float u  = fade(xf);
    float v  = fade(yf);

    int aa = perm[perm[xi  ] + yi  ];
    int ab = perm[perm[xi  ] + yi+1];
    int ba = perm[perm[xi+1] + yi  ];
    int bb = perm[perm[xi+1] + yi+1];

    return lerpf(
        lerpf(grad2(aa,xf,  yf  ), grad2(ba,xf-1,yf  ), u),
        lerpf(grad2(ab,xf,  yf-1), grad2(bb,xf-1,yf-1), u),
        v
    );
}

/* multi-octave noise used for the curl potential */
static float noise_fbm(float x, float y, float t, int octaves)
{
    float val = 0.f, amp = 1.f, freq = 1.f;
    for (int o = 0; o < octaves; o++) {
        val  += noise2(x * CURL_SX * freq + t,
                       y * CURL_SY * freq + t * 0.7f) * amp;
        amp  *= 0.5f;
        freq *= 2.0f;
    }
    return val;
}

/* ===================================================================== */
/* §5  field                                                              */
/* ===================================================================== */

/*
 * Field — the 2-D vector field.
 *
 * angles[y*cols+x] holds the flow direction at cell (x,y) in radians.
 * Recomputed each tick by the active field mode.
 *
 * Vortex state is embedded here: vort_cx/cy[] hold current vortex centres
 * that orbit the screen centre each tick.
 */
typedef struct {
    float *angles;                  /* [rows * cols] flow angle per cell     */
    int    cols;
    int    rows;
    float  time;                    /* time axis (all modes)                 */
    float  speed;                   /* time advance per tick                 */
    int    field_type;              /* 0=curl 1=vortex 2=sine 3=spiral       */

    /* vortex lattice state (field type 1) */
    float  vort_cx[N_VORT];
    float  vort_cy[N_VORT];
    float  vort_ring_a;             /* current ring angle (radians)          */
} Field;

static void field_vortex_update_centres(Field *f)
{
    float cx = (float)f->cols * 0.5f;
    float cy = (float)f->rows * 0.5f;
    float r  = VORT_RING_FRAC * (float)(f->cols < f->rows ? f->cols : f->rows) * 0.5f;

    for (int i = 0; i < N_VORT; i++) {
        float a   = f->vort_ring_a + (float)i * (2.f * (float)M_PI / (float)N_VORT);
        f->vort_cx[i] = cx + r * cosf(a);
        f->vort_cy[i] = cy + r * sinf(a) * CELL_AR;
    }
}

static void field_alloc(Field *f, int cols, int rows, int field_type)
{
    f->cols       = cols;
    f->rows       = rows;
    f->angles     = calloc((size_t)(cols * rows), sizeof(float));
    f->time       = 0.f;
    f->speed      = FIELD_SPD_DEFAULT;
    f->field_type = field_type;
    f->vort_ring_a= 0.f;
    field_vortex_update_centres(f);
}

static void field_free(Field *f)
{
    free(f->angles);
    *f = (Field){0};
}

static void field_resize(Field *f, int cols, int rows)
{
    free(f->angles);
    f->cols   = cols;
    f->rows   = rows;
    f->angles = calloc((size_t)(cols * rows), sizeof(float));
    field_vortex_update_centres(f);
}

/*
 * field_compute_curl() — divergence-free field via curl of scalar noise.
 *
 * Central differences estimate the spatial gradient of the potential ψ.
 * The sign convention for curl in 2-D (z-component of ∇ × (0,0,ψ)):
 *   Vx = +∂ψ/∂y,   Vy = −∂ψ/∂x
 */
static float field_curl_angle(float x, float y, float t)
{
    float eps = CURL_EPS;
    float dn = noise_fbm(x,     y + eps, t, CURL_OCTAVES);
    float ds = noise_fbm(x,     y - eps, t, CURL_OCTAVES);
    float de = noise_fbm(x+eps, y,       t, CURL_OCTAVES);
    float dw = noise_fbm(x-eps, y,       t, CURL_OCTAVES);
    float vx =  (dn - ds) / (2.f * eps);
    float vy = -(de - dw) / (2.f * eps);
    return atan2f(vy, vx);
}

/*
 * field_compute_vortex() — Biot-Savart superposition from N_VORT vortices.
 *
 * CELL_AR corrects dy so that distance is computed in visual-pixel space,
 * making the velocity field appear rotationally symmetric on screen despite
 * terminal characters being taller than wide.
 */
static float field_vortex_angle(const Field *f, float x, float y)
{
    float vx = 0.f, vy = 0.f;
    for (int i = 0; i < N_VORT; i++) {
        float dx  = x - f->vort_cx[i];
        float dy  = (y - f->vort_cy[i]) / CELL_AR;   /* visual-pixel dy */
        float r2  = dx*dx + dy*dy + VORT_EPS;
        float str = (i % 2 == 0) ? VORT_STRENGTH : -VORT_STRENGTH;
        vx += str * (-dy) / r2;
        vy += str * ( dx) / r2;
    }
    return atan2f(vy, vx);
}

/* field_compute_sine() — interference of two crossing wave pairs */
static float field_sine_angle(float x, float y, float t)
{
    float vx = sinf(x * SINE_XFREQ + t)
             + sinf(y * SINE_YFREQ - t * 0.7f);
    float vy = cosf(x * SINE_XFREQ - t * 0.5f)
             + cosf(y * SINE_YFREQ + t * 0.3f);
    return atan2f(vy, vx);
}

/*
 * field_compute_spiral() — galaxy spiral with pulsing radial component.
 *
 * Pure tangential: (−sin θ, cos θ) — infinite CCW rotation.
 * Radial term oscillates with time, making particles breathe in/out.
 * The aspect ratio (CELL_AR) is applied to dy so the spiral looks round.
 */
static float field_spiral_angle(const Field *f, float x, float y, float t)
{
    float cx = (float)f->cols * 0.5f;
    float cy = (float)f->rows * 0.5f;
    float dx = x - cx;
    float dy = (y - cy) / CELL_AR;
    float r  = sqrtf(dx*dx + dy*dy);
    if (r < 1e-4f) return 0.f;
    float theta = atan2f(dy, dx);
    float pulse = SPIRAL_RADIAL_W * sinf(t * 0.8f);
    float vx = -sinf(theta) + pulse * cosf(theta);
    float vy =  cosf(theta) + pulse * sinf(theta);
    return atan2f(vy, vx);
}

static void field_tick(Field *f)
{
    f->time += f->speed;

    /* advance vortex ring */
    f->vort_ring_a += VORT_ORB_SPD;
    field_vortex_update_centres(f);

    for (int y = 0; y < f->rows; y++) {
        for (int x = 0; x < f->cols; x++) {
            float angle;
            switch (f->field_type) {
            case 0: angle = field_curl_angle((float)x, (float)y, f->time);       break;
            case 1: angle = field_vortex_angle(f, (float)x, (float)y);           break;
            case 2: angle = field_sine_angle((float)x, (float)y, f->time);       break;
            default:angle = field_spiral_angle(f, (float)x, (float)y, f->time);  break;
            }
            f->angles[y * f->cols + x] = angle;
        }
    }
}

static float field_sample(const Field *f, float px, float py)
{
    int x0 = (int)floorf(px);
    int y0 = (int)floorf(py);
    int x1 = x0 + 1;
    int y1 = y0 + 1;

    if (x0 < 0) x0 = 0; else if (x0 >= f->cols) x0 = f->cols - 1;
    if (y0 < 0) y0 = 0; else if (y0 >= f->rows) y0 = f->rows - 1;
    if (x1 < 0) x1 = 0; else if (x1 >= f->cols) x1 = f->cols - 1;
    if (y1 < 0) y1 = 0; else if (y1 >= f->rows) y1 = f->rows - 1;

    float tx = px - floorf(px);
    float ty = py - floorf(py);

    float a00 = f->angles[y0 * f->cols + x0];
    float a10 = f->angles[y0 * f->cols + x1];
    float a01 = f->angles[y1 * f->cols + x0];
    float a11 = f->angles[y1 * f->cols + x1];

    return lerpf(lerpf(a00, a10, tx), lerpf(a01, a11, tx), ty);
}

/*
 * angle_to_char() — 8-directional glyph for a flow angle.
 *
 *   E→>  NE→/  N→^  NW→\  W→<  SW→/  S→v  SE→\
 */
static char angle_to_char(float angle)
{
    static const char k_dir[8] = {'>', '/', '^', '\\', '<', '/', 'v', '\\'};
    float a = angle;
    if (a < 0.f) a += 2.f * (float)M_PI;
    int idx = (int)(a / (2.f * (float)M_PI) * 8.f + 0.5f) % 8;
    return k_dir[idx];
}

/* ===================================================================== */
/* §6  particle                                                           */
/* ===================================================================== */

/*
 * Particle — one flow tracer.
 *
 * head_pair : ncurses color pair for the particle head, updated every tick
 *             from the current movement angle via angle_to_pair().
 * trail_x/y : ring buffer of past cell-grid positions, length = trail_len.
 * head      : index of the *next* write slot in the ring buffer.
 *
 * Color scheme per trail cell (i=0 oldest, fill-1=newest):
 *   Newest quarter   : A_BOLD   — bright, stands out over background
 *   Middle half      : A_NORMAL — standard brightness
 *   Oldest quarter   : A_DIM    — fades into background
 * All cells use head_pair so the whole trail shares one coherent hue.
 *
 * Trail ramp chars (oldest→newest):
 *   '.' ',' ';' '+' '~' '*' '#'   then direction char at head
 */
#define TRAIL_RAMP      ".,;+~*#"
#define TRAIL_RAMP_N    7          /* excludes direction char at head         */

typedef struct {
    float x, y;
    float speed;
    float angle;
    int   trail_x[TRAIL_MAX];
    int   trail_y[TRAIL_MAX];
    int   head;
    int   trail_fill;
    int   trail_len;
    int   life;
    int   head_pair;
    bool  alive;
} Particle;

static void particle_spawn(Particle *p, int cols, int rows, int trail_len)
{
    p->x          = (float)(rand() % cols);
    p->y          = (float)(rand() % rows);
    p->speed      = PART_SPEED - PART_SPD_JITTER * 0.5f
                  + PART_SPD_JITTER * ((float)rand() / RAND_MAX);
    p->angle      = 0.f;
    p->head       = 0;
    p->trail_fill = 0;
    p->trail_len  = trail_len < TRAIL_MAX ? trail_len : TRAIL_MAX;
    p->life       = PART_LIFE_BASE + rand() % PART_LIFE_JITTER;
    p->head_pair  = CP_BASE + rand() % N_PAIRS;
    p->alive      = true;
    for (int i = 0; i < TRAIL_MAX; i++) {
        p->trail_x[i] = (int)p->x;
        p->trail_y[i] = (int)p->y;
    }
}

static void particle_tick(Particle *p, const Field *f, int cols, int rows)
{
    if (!p->alive) return;

    /* push current grid position into ring buffer */
    p->trail_x[p->head] = (int)p->x;
    p->trail_y[p->head] = (int)p->y;
    p->head = (p->head + 1) % p->trail_len;
    if (p->trail_fill < p->trail_len) p->trail_fill++;

    float angle = field_sample(f, p->x, p->y);
    p->angle = angle;
    p->x += cosf(angle) * p->speed;
    p->y += sinf(angle) * p->speed;

    /* wrap toroidally */
    if (p->x < 0.f)            p->x += (float)cols;
    if (p->x >= (float)cols)   p->x -= (float)cols;
    if (p->y < 0.f)            p->y += (float)rows;
    if (p->y >= (float)rows)   p->y -= (float)rows;

    /* update color from current direction */
    p->head_pair = angle_to_pair(angle);

    p->life--;
    if (p->life <= 0) p->alive = false;
}

static void particle_draw(const Particle *p, WINDOW *w, int cols, int rows)
{
    if (!p->alive || p->trail_fill == 0) return;

    static const char k_dir[8] = {'>', '/', '^', '\\', '<', '/', 'v', '\\'};
    int fill = p->trail_fill;

    for (int i = 0; i < fill; i++) {
        /* i=0 oldest, i=fill-1 newest (head) */
        int ti = (p->head - fill + i + p->trail_len * 4) % p->trail_len;
        int tx = p->trail_x[ti];
        int ty = p->trail_y[ti];

        if (tx < 0 || tx >= cols || ty < 0 || ty >= rows) continue;

        bool is_head = (i == fill - 1);

        char ch;
        if (is_head) {
            float a = p->angle;
            if (a < 0.f) a += 2.f * (float)M_PI;
            int di = (int)(a / (2.f * (float)M_PI) * 8.f + 0.5f) % 8;
            ch = k_dir[di];
        } else {
            /* proportional ramp across trail body */
            int ri = (i * TRAIL_RAMP_N) / (fill > 1 ? fill : 1);
            if (ri >= TRAIL_RAMP_N) ri = TRAIL_RAMP_N - 1;
            ch = TRAIL_RAMP[ri];
        }

        /* brightness by relative position in trail */
        attr_t bright;
        if (is_head)
            bright = A_BOLD;
        else if (i >= fill * 3 / 4)
            bright = A_NORMAL;
        else if (i >= fill / 4)
            bright = A_NORMAL;
        else
            bright = A_DIM;

        attr_t attr = COLOR_PAIR(p->head_pair) | bright;
        wattron(w, attr);
        mvwaddch(w, ty, tx, (chtype)(unsigned char)ch);
        wattroff(w, attr);
    }
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

typedef struct {
    Field    field;
    Particle pool[PARTS_MAX];
    int      n_particles;
    int      trail_len;
    int      theme;
    int      bg_mode;    /* 0=blank 1=arrows 2=colormap */
    bool     paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->n_particles = PARTS_DEFAULT;
    s->trail_len   = TRAIL_LEN_DEFAULT;
    s->theme       = 0;
    s->bg_mode     = 0;
    s->paused      = false;
    field_alloc(&s->field, cols, rows, 0);
    field_tick(&s->field);
    for (int i = 0; i < s->n_particles; i++)
        particle_spawn(&s->pool[i], cols, rows, s->trail_len);
}

static void scene_free(Scene *s) { field_free(&s->field); }

static void scene_resize(Scene *s, int cols, int rows)
{
    field_resize(&s->field, cols, rows);
    field_tick(&s->field);
    for (int i = 0; i < PARTS_MAX; i++)
        if (s->pool[i].alive)
            particle_spawn(&s->pool[i], cols, rows, s->trail_len);
}

static void scene_reset(Scene *s, int cols, int rows)
{
    s->field.time      = 0.f;
    s->field.vort_ring_a = 0.f;
    field_vortex_update_centres(&s->field);
    field_tick(&s->field);
    for (int i = 0; i < s->n_particles; i++)
        particle_spawn(&s->pool[i], cols, rows, s->trail_len);
    for (int i = s->n_particles; i < PARTS_MAX; i++)
        s->pool[i].alive = false;
}

static void scene_tick(Scene *s, int cols, int rows)
{
    if (s->paused) return;
    field_tick(&s->field);
    for (int i = 0; i < s->n_particles; i++) {
        if (!s->pool[i].alive)
            particle_spawn(&s->pool[i], cols, rows, s->trail_len);
        particle_tick(&s->pool[i], &s->field, cols, rows);
    }
}

/*
 * scene_draw() — background then particle trails.
 *
 * bg_mode 0 : blank canvas — erase() clears to terminal background
 * bg_mode 1 : dim directional arrows tinted by cosine palette
 * bg_mode 2 : full colormap — every cell painted by angle→palette
 *             particles drawn on top at A_BOLD stand out clearly
 */
static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows)
{
    for (int y = 0; y < rows && s->bg_mode > 0; y++) {
        for (int x = 0; x < cols; x++) {
            float angle = s->field.angles[y * cols + x];
            int   cp    = angle_to_pair(angle);
            char  ch    = angle_to_char(angle);

            if (s->bg_mode == 1) {
                wattron(w, COLOR_PAIR(cp) | A_DIM);
                mvwaddch(w, y, x, (chtype)(unsigned char)ch);
                wattroff(w, COLOR_PAIR(cp) | A_DIM);
            } else {
                /* full colormap: bold direction char fills the screen */
                wattron(w, COLOR_PAIR(cp) | A_NORMAL);
                mvwaddch(w, y, x, (chtype)(unsigned char)ch);
                wattroff(w, COLOR_PAIR(cp) | A_NORMAL);
            }
        }
    }

    for (int i = 0; i < s->n_particles; i++)
        particle_draw(&s->pool[i], w, cols, rows);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

static const char * const k_field_names[N_FIELD_TYPES] = {
    "curl-noise", "vortex-lattice", "sine-lattice", "radial-spiral"
};
static const char * const k_bg_names[N_BG_MODES] = {
    "blank", "arrows", "colormap"
};

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s, int theme)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(theme);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(const Screen *s, const Scene *sc,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    /* HUD — top-right corner */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%4.1ffps  n:%d  %s  %s  %s  %dhz",
             fps,
             sc->n_particles,
             k_themes[sc->theme].name,
             k_field_names[sc->field.field_type],
             k_bg_names[sc->bg_mode],
             sim_fps);
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    int cp = angle_to_pair(sc->field.time * 0.3f);
    attron(COLOR_PAIR(cp) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(cp) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize  = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s    = &app->scene;
    int    cols = app->screen.cols;
    int    rows = app->screen.rows;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        s->paused = !s->paused;
        break;

    case 'r': case 'R':
        scene_reset(s, cols, rows);
        break;

    case 'a': case 'A':
        s->field.field_type = (s->field.field_type + 1) % N_FIELD_TYPES;
        field_tick(&s->field);
        scene_reset(s, cols, rows);
        break;

    case 't': case 'T':
        s->theme = (s->theme + 1) % N_THEMES;
        color_apply_theme(s->theme);
        break;

    case 'v': case 'V':
        s->bg_mode = (s->bg_mode + 1) % N_BG_MODES;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case '=': case '+':
        s->n_particles += PARTS_STEP;
        if (s->n_particles > PARTS_MAX) s->n_particles = PARTS_MAX;
        for (int i = s->n_particles - PARTS_STEP; i < s->n_particles; i++)
            particle_spawn(&s->pool[i], cols, rows, s->trail_len);
        break;
    case '-':
        s->n_particles -= PARTS_STEP;
        if (s->n_particles < PARTS_MIN) s->n_particles = PARTS_MIN;
        for (int i = s->n_particles; i < s->n_particles + PARTS_STEP; i++)
            s->pool[i].alive = false;
        break;

    case 'f':
        s->field.speed *= FIELD_SPD_STEP;
        if (s->field.speed > FIELD_SPD_MAX) s->field.speed = FIELD_SPD_MAX;
        break;
    case 'F':
        s->field.speed /= FIELD_SPD_STEP;
        if (s->field.speed < FIELD_SPD_MIN) s->field.speed = FIELD_SPD_MIN;
        break;

    case 's':
        s->trail_len++;
        if (s->trail_len > TRAIL_LEN_MAX) s->trail_len = TRAIL_LEN_MAX;
        for (int i = 0; i < PARTS_MAX; i++)
            s->pool[i].trail_len = s->trail_len < TRAIL_MAX
                                 ? s->trail_len : TRAIL_MAX;
        break;
    case 'S':
        s->trail_len--;
        if (s->trail_len < TRAIL_LEN_MIN) s->trail_len = TRAIL_LEN_MIN;
        for (int i = 0; i < PARTS_MAX; i++)
            s->pool[i].trail_len = s->trail_len;
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFFu));
    noise_init();

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen, 0);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── dt ──────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── sim accumulator ─────────────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── FPS counter (every 500 ms) ──────────────────────────── */
        fps_accum += dt;
        frame_count++;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count * 1e9 / (double)fps_accum;
            fps_accum   = 0;
            frame_count = 0;
        }

        /* ── frame cap: sleep to ~60 fps before rendering ────────── */
        int64_t target_ns = NS_PER_SEC / 60;
        int64_t elapsed   = clock_ns() - now;
        clock_sleep_ns(target_ns - elapsed);

        /* ── draw ────────────────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int key;
        while ((key = getch()) != ERR)
            if (!app_handle_key(app, key)) { app->running = 0; break; }
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
