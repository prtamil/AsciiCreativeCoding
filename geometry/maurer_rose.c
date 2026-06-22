/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * maurer_rose.c — the Maurer rose: sample the flower curve r = cos(n·θ)
 * at fixed angle steps and join the dots with straight lines.  Big steps
 * make the lines cut across the flower, weaving a lacework that drifts
 * slowly through named (n, d) landmarks.
 *
 * Maurer's original paper, where the construction and the famous (6, 71)
 * figure come from: P. M. Maurer, "A Rose is a Rose...",
 * American Mathematical Monthly 94(7), 1987, pp. 631-645.
 * Palette idea: Iñigo Quilez, "Palettes" (2015),
 * https://iquilezles.org/articles/palettes/
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* §1 config — sizes, tunings, named constants, colour-pair numbers */

#define NS_PER_SEC        1000000000LL
#define NS_PER_MS         1000000LL

#define TICK_NS           33333333LL              /* ~30 fps             */
#define DT_CAP_NS         (100 * NS_PER_MS)       /* spiral-of-death cap */
#define FRAME_PERIOD_NS   (NS_PER_SEC / 60)       /* render cadence      */
#define FPS_UPDATE_MS     500

#define CELL_W            8
#define CELL_H            16

#define MAX_ROWS          128
#define MAX_COLS          320

/* The two numbers that define the figure: n sets how many petals the
 * flower has, d is the angle (degrees) between one sample and the next.
 * Both slide around on their own, slowing down near whole-number pairs
 * so the famous shapes linger on screen (see dwell_envelope in §6). */
#define N_DEFAULT         6.0f
#define N_MIN             2.0f
#define N_MAX             9.0f
#define N_SPEED_DEFAULT   0.002f     /* how fast n drifts, per tick      */

#define D_DEFAULT         71.0f
#define D_MIN             2.0f
#define D_MAX             178.0f
#define D_SPEED_DEFAULT   0.07f      /* how fast d drifts, per tick      */

#define DRIFT_SPEED_STEP  1.5f       /* +/- keys multiply speed by this  */
#define DRIFT_SPEED_MIN_F 0.10f      /* slowest: 10% of default          */
#define DRIFT_SPEED_MAX_F 5.00f      /* fastest: 500% of default         */

/* How hard the drift brakes near a whole-number (n, d). */
#define DWELL_FLOOR       0.12f      /* speed right at a landmark        */
#define DWELL_RAMP        1.88f      /* how quickly it speeds back up    */

#define N_SAMPLES         360        /* lines drawn per frame            */
#define ROSE_AMP_FRAC     0.48f      /* flower size as fraction of screen*/

/* 8 brightness levels, paired one-to-one with 8 palette colours, so
 * faint cells get the cool end and crowded ones get the hot end. */
#define N_TIERS           8
#define BRIGHT_TIER       (N_TIERS - 1)

/* Rows reserved top and bottom for the HUD. */
enum { HUD_TOP_ROWS = 1, HUD_BOTTOM_ROWS = 1 };

/* Which colour pair number means what.  Pairs 1..N_TIERS are the
 * brightness colours and change with the theme; the rest are fixed
 * chrome (rim, centre dot, HUD text). */
#define CP_TIER_0         1
#define CP_RIM            (N_TIERS + 1)
#define CP_CENTRE         (N_TIERS + 2)
#define HUD_PAIR_TITLE    (N_TIERS + 3)
#define HUD_PAIR_DATA     (N_TIERS + 4)
#define HUD_PAIR_HINT     HUD_PAIR_TITLE

#define KEY_ESC           27

/* TAU is one full turn (2π); DEG_TO_RAD turns degrees into radians. */
#define TAU               (2.0f * (float)M_PI)
#define DEG_TO_RAD        ((float)M_PI / 180.0f)

/* The 256-colour terminal palette has a 6×6×6 colour cube starting at
 * index 16.  To find a colour: pick a 0..5 level for red, green, blue
 * and combine them with these strides.  (See rgb_to_ansi256.) */
#define ANSI_CUBE_BASE        16
#define ANSI_CUBE_R_STRIDE    36
#define ANSI_CUBE_G_STRIDE     6
#define ANSI_CUBE_MAX_STEP     5

/* On old 8-colour terminals each colour channel is just on or off; a
 * channel counts as "on" past the halfway mark.  (See rgb_to_ansi8.) */
#define RGB_BIT_THRESHOLD     0.5f

/* Column where the middle of the HUD text starts (after the title). */
#define HUD_DATA_COL          15

/* Which preset we boot into: slot 4 is "classic-6" (n=6, d=71),
 * Maurer's published figure, so the first thing you see is the famous
 * one. */
#define DEFAULT_PRESET_IDX    4

/* The grey/cyan/yellow colour codes for the HUD, kept in one place.
 * Greys are dim so they don't fight the rose; cyan and yellow are
 * bright so the text stays readable over any animation. */
#define HUD_RIM_GREY_256       240
#define HUD_CENTRE_GREY_256    251
#define HUD_TITLE_CYAN_256      51
#define HUD_DATA_YELLOW_256    226

/* §2 clock — monotonic nanosecond clock + sleep */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec ts = { .tv_sec  = (time_t)(ns / NS_PER_SEC),
                           .tv_nsec = (long)(ns % NS_PER_SEC) };
    nanosleep(&ts, NULL);
}

/* §3 color — RGB, cosine palette, ANSI colour pickers, themes */

/*
 * RGB -- one colour, each of red/green/blue held as a float in [0, 1].
 *
 * We work in floats (not 0..255 ints) because the palette maths runs in
 * floats; we only round to an actual terminal colour at the very end,
 * in rgb_to_ansi256 / rgb_to_ansi8.  Once cosine_palette hands one back,
 * nothing changes it.
 *
 * Fields:
 *   r, g, b  the red, green and blue amounts, each 0 (none) to 1 (full).
 */
typedef struct {
    float r;    /* red   amount, 0..1 */
    float g;    /* green amount, 0..1 */
    float b;    /* blue  amount, 0..1 */
} RGB;

/* Pull one channel back inside 0..1. */
static inline float clamp01(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

/*
 * Pick the colour at position t (0..1) along a smooth gradient.  Each
 * channel follows a cosine wave, color = a + b·cos(TAU·(c·t + d)), so the
 * four control values are: a, the middle brightness; b, how far it swings;
 * c, how many waves fit across the gradient; d, where the wave starts.
 * Running the three channels with different c and d is what makes a
 * rainbow.  Idea from Quilez's "Palettes".
 */
static RGB cosine_palette(const RGB *a, const RGB *b,
                          const RGB *c, const RGB *d, float t)
{
    RGB out;
    out.r = a->r + b->r * cosf(TAU * (c->r * t + d->r));
    out.g = a->g + b->g * cosf(TAU * (c->g * t + d->g));
    out.b = a->b + b->b * cosf(TAU * (c->b * t + d->b));

    /* The cosine can overshoot past 0 or 1; clip rather than rescale so
     * one stray channel doesn't shift the colours next to it. */
    out.r = clamp01(out.r);
    out.g = clamp01(out.g);
    out.b = clamp01(out.b);
    return out;
}

/* Snap one 0..1 channel to its nearest of the cube's 6 levels (0..5). */
static int rgb_channel_to_cube_step(float v)
{
    int idx = (int)(v * (float)ANSI_CUBE_MAX_STEP + 0.5f);
    if (idx < 0)                   idx = 0;
    if (idx > ANSI_CUBE_MAX_STEP)  idx = ANSI_CUBE_MAX_STEP;
    return idx;
}

/* Find the closest colour in the terminal's 6×6×6 colour cube. */
static int rgb_to_ansi256(RGB c)
{
    int ri = rgb_channel_to_cube_step(c.r);
    int gi = rgb_channel_to_cube_step(c.g);
    int bi = rgb_channel_to_cube_step(c.b);
    return ANSI_CUBE_BASE
         + ANSI_CUBE_R_STRIDE * ri
         + ANSI_CUBE_G_STRIDE * gi
         + bi;
}

/* Fallback for old 8-colour terminals: treat each channel as just
 * on or off, which gives the 8 standard colours. */
static int rgb_to_ansi8(RGB c)
{
    int r = (c.r > RGB_BIT_THRESHOLD);
    int g = (c.g > RGB_BIT_THRESHOLD);
    int b = (c.b > RGB_BIT_THRESHOLD);
    if (!r && !g && !b) return COLOR_BLACK;
    if ( r && !g && !b) return COLOR_RED;
    if (!r &&  g && !b) return COLOR_GREEN;
    if (!r && !g &&  b) return COLOR_BLUE;
    if ( r &&  g && !b) return COLOR_YELLOW;
    if ( r && !g &&  b) return COLOR_MAGENTA;
    if (!r &&  g &&  b) return COLOR_CYAN;
    return COLOR_WHITE;
}

/*
 * Theme -- one named gradient, stored as the four control values that
 * cosine_palette needs rather than as a list of finished colours.  Keeping
 * the recipe (12 floats) instead of the result means the same theme works
 * no matter how many brightness levels we ask for, and a new theme is just
 * a few numbers to nudge.  Switching theme (t/T) recomputes the brightness
 * colours and leaves the fixed HUD colours alone.
 *
 * Fields (each is an RGB so the three channels can differ):
 *   name  short label shown in the HUD ("Matrix", "Sunset", ...).
 *   a     middle brightness each channel sits around.
 *   b     how far each channel swings up and down from a.  Keep a+b ≤ 1
 *         and a−b ≥ 0 to avoid clipping (cosine_palette clamps anyway).
 *   c     how many waves fit across the gradient; 0.5 = a single smooth
 *         sweep, 1.0 = a full loop back to the start colour.
 *   d     where each channel's wave begins; offsetting the three is what
 *         spreads them into different hues.
 */
typedef struct {
    const char *name;
    RGB         a;
    RGB         b;
    RGB         c;
    RGB         d;
} Theme;

#define N_THEMES 10

static const Theme k_themes[N_THEMES] = {
    /* Tuned so the faintest cells stay just visible and the busiest get
     * the theme's hottest colour. */
    /*  name            a (offset)              b (amplitude)            c (frequency)            d (phase)              */
    { "Matrix",      {0.00f, 0.55f, 0.08f}, {0.00f, 0.45f, 0.08f}, {0.00f, 0.50f, 0.50f}, {0.00f, 0.50f, 0.50f} },
    { "Sunset",      {0.70f, 0.50f, 0.30f}, {0.20f, 0.50f, 0.30f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.10f, 0.20f} },
    { "Ocean",       {0.20f, 0.45f, 0.65f}, {0.20f, 0.40f, 0.35f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.30f, 0.50f} },
    { "Plasma",      {0.55f, 0.40f, 0.55f}, {0.45f, 0.40f, 0.45f}, {1.00f, 1.00f, 1.00f}, {0.00f, 0.33f, 0.67f} },
    { "Fire",        {0.55f, 0.30f, 0.15f}, {0.45f, 0.50f, 0.35f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.10f, 0.30f} },
    { "Mint",        {0.20f, 0.55f, 0.50f}, {0.20f, 0.40f, 0.40f}, {0.50f, 0.50f, 0.50f}, {0.30f, 0.50f, 0.50f} },
    { "Lavender",    {0.55f, 0.45f, 0.65f}, {0.40f, 0.35f, 0.30f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.10f, 0.20f} },
    { "Aurora",      {0.45f, 0.55f, 0.55f}, {0.40f, 0.40f, 0.40f}, {1.00f, 1.00f, 0.50f}, {0.00f, 0.50f, 0.75f} },
    { "Sepia",       {0.45f, 0.35f, 0.25f}, {0.40f, 0.35f, 0.20f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.05f, 0.10f} },
    { "Rainbow",     {0.50f, 0.50f, 0.50f}, {0.50f, 0.50f, 0.50f}, {1.00f, 1.00f, 1.00f}, {0.00f, 0.33f, 0.67f} },
};

static inline int theme_next(int cur) { return (cur + 1) % N_THEMES; }
static inline int theme_prev(int cur) { return (cur + N_THEMES - 1) % N_THEMES; }

/* Where tier i sits along the gradient, 0 at the cool end, 1 at the hot. */
static inline float tier_to_gradient_t(int i)
{
    return (float)i / (float)(N_TIERS - 1);
}

/* Build this theme's brightness colours and load them into the terminal. */
static void theme_apply(int theme_idx)
{
    const Theme *th = &k_themes[theme_idx];
    bool have_256   = (COLORS >= 256);
    for (int i = 0; i < N_TIERS; i++) {
        float t   = tier_to_gradient_t(i);
        RGB   rgb = cosine_palette(&th->a, &th->b, &th->c, &th->d, t);
        int   pair_code = have_256 ? rgb_to_ansi256(rgb) : rgb_to_ansi8(rgb);
        init_pair(CP_TIER_0 + i, pair_code, -1);
    }
}

/* Set up colours once: the fixed HUD colours, then the first theme. */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_RIM,          HUD_RIM_GREY_256,    -1);
        init_pair(CP_CENTRE,       HUD_CENTRE_GREY_256, -1);
        init_pair(HUD_PAIR_TITLE,  HUD_TITLE_CYAN_256,  -1);
        init_pair(HUD_PAIR_DATA,   HUD_DATA_YELLOW_256, -1);
    } else {
        init_pair(CP_RIM,          COLOR_WHITE,  -1);
        init_pair(CP_CENTRE,       COLOR_WHITE,  -1);
        init_pair(HUD_PAIR_TITLE,  COLOR_CYAN,   -1);
        init_pair(HUD_PAIR_DATA,   COLOR_YELLOW, -1);
    }
    theme_apply(0);
}

/* §4 coords — turn the rose's maths coordinates into screen cells */

/*
 * RenderFrame -- how to place the rose on this size of screen.
 *
 * The rose maths works in a tidy square from -1 to 1.  This struct holds
 * where the centre of that square lands on screen and how much to blow it
 * up, so a point can be moved out to its pixel and then rounded to a
 * character cell (scene_sample_cell does that).  The rose is centred in
 * the drawing band only, leaving the HUD rows clear.
 *
 * It's rebuilt fresh every frame rather than stored, because it depends
 * only on the current size and a stale copy after a resize would put the
 * rose in the wrong place; recomputing it is a handful of multiplies.
 *
 * Fields (all in sub-cell pixels, where CELL_W pixels make one column
 * and CELL_H pixels make one row):
 *   cx_px, cy_px  screen position of the rose's centre.
 *   scale_px      how many pixels one unit of the rose maths becomes.
 */
typedef struct {
    float cx_px;
    float cy_px;
    float scale_px;
} RenderFrame;

static RenderFrame render_frame_make(int rows, int cols)
{
    int   draw_rows = rows - HUD_TOP_ROWS - HUD_BOTTOM_ROWS;
    float draw_h_px = (float)draw_rows * CELL_H;
    float draw_w_px = (float)cols      * CELL_W;
    return (RenderFrame){
        .cx_px    = (float)cols * CELL_W * 0.5f,
        .cy_px    = ((float)HUD_TOP_ROWS + (float)draw_rows * 0.5f) * CELL_H,
        .scale_px = fminf(draw_w_px, draw_h_px) * ROSE_AMP_FRAC,
    };
}

/* Round-half-up to integer cell. */
static inline int cell_round(float v) { return (int)(v + 0.5f); }

/* §5 entity — the rose maths, the density buffer, the brightness ramp */

/*
 * RoseParams -- the live (n, d) pair plus how fast each is sliding.
 *
 * These two numbers fully describe the figure (n = petals, d = angle step
 * in degrees); both are floats so the picture can morph smoothly through
 * the whole-number landmarks instead of jumping between them.
 *
 * Speed sits next to value because the two drift at different natural
 * rates: n slowly, so the petal count holds still for a few seconds at a
 * time, and d faster, since the woven interior needs to keep moving to
 * feel alive.  scene_tick then advances each with one line.
 *
 * Fields (units / who changes them):
 *   n         petal count, in [N_MIN, N_MAX).  Changed by the drift,
 *             the preset keys, and reset.
 *   d         sample step in degrees, in [D_MIN, D_MAX).  Same changers.
 *   n_speed   how far n moves per tick at full speed.  The +/- keys
 *             scale it (via speed_scale).
 *   d_speed   the same for d.
 */
typedef struct {
    float n;
    float d;
    float n_speed;
    float d_speed;
} RoseParams;

/*
 * RosePreset -- a named, fixed (n, d) landmark worth jumping to.
 *
 * Kept separate from the live RoseParams for two reasons: it's const so
 * the compiler stops anyone writing over a landmark, and it carries a
 * name.  The HUD shows the name of whichever landmark is nearest right
 * now, and the [ and ] keys hop straight to the previous or next one, so
 * the list doubles as a guided tour.  Several names point at published
 * figures (e.g. "classic-6" is Maurer's paper figure).
 *
 * Fields:
 *   n, d  the landmark's petal count and angle step (whole numbers).
 *   name  short label shown in the HUD ("classic-6", "fine-7", ...).
 */
typedef struct {
    float       n;
    float       d;
    const char *name;
} RosePreset;

/* Hand-picked landmarks, ordered simple-to-busy so [ and ] make a tour. */
static const RosePreset k_presets[] = {
    { 2.0f,  39.0f, "soft-2"     },   /* gentle 2-petal weave           */
    { 3.0f,  47.0f, "trefoil"    },   /* 3-petal with crisscross        */
    { 4.0f,  89.0f, "quad-89"    },   /* 4-petal star envelope          */
    { 5.0f,  97.0f, "pentagon-5" },   /* 5-fold woven star              */
    { 6.0f,  71.0f, "classic-6"  },   /* Maurer's published landmark    */
    { 7.0f,  11.0f, "fine-7"     },   /* 7-fold dense lacework          */
    { 8.0f, 121.0f, "octa-121"   },   /* 8-fold envelope                */
    { 5.0f,  37.0f, "star-37"    },   /* 5-pointed star (gcd interest)  */
    { 3.0f,  53.0f, "triangle-3" },   /* near-triangle weave            */
    { 2.0f, 179.0f, "deg-179"    },   /* near-period mirror             */
};
#define N_PRESETS ((int)(sizeof(k_presets) / sizeof(k_presets[0])))

/*
 * Density -- a tally of how many lines crossed each screen cell.
 *
 * The rose is hundreds of lines piled on top of each other.  Instead of
 * just drawing them (where the last line wins and you lose the overlap
 * info), we count: every line bumps the counter in each cell it passes
 * through, then the picture is painted from those counts.  Busy cells
 * (the outer edge of the flower) end up with high counts and the hot
 * colour; lightly-crossed cells in the middle stay cool.  It's basically
 * a heat map.
 *
 * Two things stay true: every cell holds the number of lines that crossed
 * it since the last clear, and peak holds the largest of those counts
 * (kept up to date as we go, so the painter doesn't have to rescan for
 * it).  Nothing ever writes into the HUD rows.
 *
 * It's a fixed MAX_ROWS×MAX_COLS array (~160 KB) living inside Scene, so
 * there's no allocation and no per-frame zeroing of heap memory.
 *
 * Fields:
 *   cells  the per-cell counts; 0 means no line touched it.
 *   peak   the biggest count right now, used as the "fully lit" mark.
 */
typedef struct {
    int cells[MAX_ROWS][MAX_COLS];
    int peak;
} Density;

static void density_clear(Density *d)
{
    memset(d->cells, 0, sizeof d->cells);
    d->peak = 0;
}

/* Walk a straight line cell by cell (Bresenham's method), bumping the
 * count in each cell it crosses and keeping peak current.  Cells outside
 * the drawing band are skipped so the HUD stays clean. */
static void density_stamp_line(Density *d,
                               int x0, int y0, int x1, int y1,
                               int rows, int cols)
{
    int dx =  abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    for (;;) {
        if ((unsigned)x0 < (unsigned)cols && x0 < MAX_COLS &&
            y0 >= HUD_TOP_ROWS && y0 < rows - HUD_BOTTOM_ROWS && y0 < MAX_ROWS) {
            int v = ++d->cells[y0][x0];
            if (v > d->peak) d->peak = v;
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* Turn a crossing count into a brightness level 0..N_TIERS-1, or -1 for
 * an empty cell.  A square-root curve lifts the lightly-crossed cells so
 * they stay visible without the busiest cells washing everything out. */
static inline int density_to_tier(int d, int peak)
{
    if (d <= 0 || peak <= 0) return -1;
    float norm = (float)d / (float)peak;
    int   tier = (int)(sqrtf(norm) * (float)(N_TIERS - 1) + 0.5f);
    if (tier < 0)         tier = 0;
    if (tier >= N_TIERS)  tier = N_TIERS - 1;
    return tier;
}

/* The character for each brightness level, lightest to heaviest, so the
 * shape reads even without colour. */
static inline char tier_glyph(int tier)
{
    static const char ramp[N_TIERS] = {
        '.', ',', ':', ';', '+', '*', '#', '@'
    };
    return ramp[tier];
}

/* Make the brightest level bold so the flower's edge appears to glow. */
static inline chtype tier_attr(int tier)
{
    return (tier == BRIGHT_TIER) ? A_BOLD : 0;
}

/* The point on the smooth rose at angle theta (radians), in maths coords. */
static void rose_point(float n, float theta_rad, float *x, float *y)
{
    float r = cosf(n * theta_rad);
    *x = r * cosf(theta_rad);
    *y = r * sinf(theta_rad);
}

/* The angle of the k-th sample: just k steps of d degrees, in radians. */
static inline float maurer_step_index_to_theta(int k, float d_deg)
{
    return (float)k * d_deg * DEG_TO_RAD;
}

/* §6 scene — all the state, the per-tick update, and the painting */

/*
 * Scene -- everything the program needs to know to draw the next frame.
 * Anything else (the HUD text, the screen cells) is worked out from these.
 *
 * Fields (units / who changes them):
 *   params       the drifting (n, d) and their speeds.  Changed by the
 *                tick, reset, and the preset keys.
 *   preset_idx   which landmark we last jumped to, in [0, N_PRESETS).
 *                Changed by the [ and ] keys.
 *   speed_scale  overall drift multiplier the +/- keys set, 1.0 = normal,
 *                clamped to [DRIFT_SPEED_MIN_F, DRIFT_SPEED_MAX_F].
 *   theme_idx    which colour theme is on, in [0, N_THEMES).  Changed by
 *                t/T (which also reload the palette).
 *   paused       SPACE freezes the drift; the rose still gets drawn.
 *   density      the crossing-count buffer, rebuilt every frame.
 *   frame        where/how big to draw the rose, rebuilt on resize.
 */
typedef struct {
    RoseParams  params;
    int         preset_idx;
    float       speed_scale;
    int         theme_idx;
    bool        paused;
    Density     density;
    RenderFrame frame;
} Scene;

/* Jump to a landmark's (n, d), keeping the current speeds. */
static void scene_apply_preset(Scene *s, int idx)
{
    s->preset_idx = idx;
    s->params.n   = k_presets[idx].n;
    s->params.d   = k_presets[idx].d;
}

static void scene_init(Scene *s, int rows, int cols)
{
    memset(s, 0, sizeof *s);
    s->params.n_speed  = N_SPEED_DEFAULT;
    s->params.d_speed  = D_SPEED_DEFAULT;
    s->speed_scale     = 1.0f;
    s->theme_idx       = 0;
    s->paused          = false;
    s->frame           = render_frame_make(rows, cols);
    scene_apply_preset(s, DEFAULT_PRESET_IDX);
}

/* Keep x inside [lo, hi) by wrapping it round if it runs off either end. */
static float wrap_into(float x, float lo, float hi)
{
    float range = hi - lo;
    while (x >= hi) x -= range;
    while (x <  lo) x += range;
    return x;
}

/* Brake factor for one parameter: near a whole number it returns about
 * DWELL_FLOOR (slow, so the landmark lingers) and rises smoothly to 1
 * (full speed) halfway between whole numbers. */
static float dwell_envelope(float x)
{
    float dist = fabsf(x - roundf(x));
    float mult = DWELL_FLOOR + DWELL_RAMP * (dist * dist * 4.0f);
    return (mult > 1.0f) ? 1.0f : mult;
}

/* Combined brake: we only want to slow down at a real landmark, which is
 * when both n and d are near whole numbers, so take the looser (faster)
 * of the two brakes. */
static float pair_dwell(float n, float d)
{
    float mn = dwell_envelope(n);
    float md = dwell_envelope(d);
    return (mn > md) ? mn : md;
}

/* Advance the drift one step (unless paused), braking near landmarks. */
static void scene_tick(Scene *s)
{
    if (s->paused) return;
    float mult = pair_dwell(s->params.n, s->params.d);
    float gain = mult * s->speed_scale;
    s->params.n = wrap_into(s->params.n + s->params.n_speed * gain, N_MIN, N_MAX);
    s->params.d = wrap_into(s->params.d + s->params.d_speed * gain, D_MIN, D_MAX);
}

/* The screen cell (column, row) the k-th sample lands on. */
static void scene_sample_cell(const Scene *s, int k, int *out_col, int *out_row)
{
    float theta = maurer_step_index_to_theta(k, s->params.d);
    float x_u, y_u;
    rose_point(s->params.n, theta, &x_u, &y_u);
    *out_col = cell_round(s->frame.cx_px + x_u * s->frame.scale_px) / CELL_W;
    *out_row = cell_round(s->frame.cy_px + y_u * s->frame.scale_px) / CELL_H;
}

/* Clear the buffer, then draw every line from one sample to the next. */
static void scene_build_density(Scene *s, int rows, int cols)
{
    density_clear(&s->density);

    int prev_col, prev_row;
    scene_sample_cell(s, 0, &prev_col, &prev_row);

    for (int k = 1; k <= N_SAMPLES; k++) {
        int col, row;
        scene_sample_cell(s, k, &col, &row);
        density_stamp_line(&s->density, prev_col, prev_row, col, row,
                           rows, cols);
        prev_col = col;
        prev_row = row;
    }
}

/* A small '+' at the rose's centre, just so the origin is visible. */
static void render_centre(const RenderFrame *rf, int rows, int cols)
{
    int cx = cell_round(rf->cx_px) / CELL_W;
    int cy = cell_round(rf->cy_px) / CELL_H;
    if ((unsigned)cx >= (unsigned)cols || cx >= MAX_COLS) return;
    if (cy < HUD_TOP_ROWS || cy >= rows - HUD_BOTTOM_ROWS) return;
    if (cy >= MAX_ROWS) return;
    attron(COLOR_PAIR(CP_CENTRE) | A_DIM);
    mvaddch(cy, cx, '+');
    attroff(COLOR_PAIR(CP_CENTRE) | A_DIM);
}

/* Draw one cell: pick its brightness level, then its character and
 * colour; empty cells are left blank. */
static void paint_density_cell(int row, int col, int density, int peak)
{
    int tier = density_to_tier(density, peak);
    if (tier < 0) return;
    chtype attr = COLOR_PAIR(CP_TIER_0 + tier) | tier_attr(tier);
    attron(attr);
    mvaddch(row, col, (chtype)(unsigned char)tier_glyph(tier));
    attroff(attr);
}

/* Paint the whole drawing area from the counts. */
static void render_density(const Density *d, int rows, int cols)
{
    int row_end = rows - HUD_BOTTOM_ROWS;
    if (row_end > MAX_ROWS) row_end = MAX_ROWS;
    int col_end = (cols < MAX_COLS) ? cols : MAX_COLS;

    for (int y = HUD_TOP_ROWS; y < row_end; y++)
        for (int x = 0; x < col_end; x++)
            paint_density_cell(y, x, d->cells[y][x], d->peak);
}

static void scene_draw(Scene *s, int rows, int cols)
{
    scene_build_density(s, rows, cols);
    render_centre  (&s->frame, rows, cols);
    render_density (&s->density, rows, cols);
}

/* Name of the landmark nearest the current (n, d), for the HUD.  n and d
 * cover very different ranges, so n is scaled up before measuring distance
 * to keep the comparison fair. */
static const char *scene_nearest_preset_name(const Scene *s)
{
    float best_dist  = 1e9f;
    int   best_idx   = 0;
    for (int i = 0; i < N_PRESETS; i++) {
        float dn = (s->params.n - k_presets[i].n) * (D_MAX / N_MAX);
        float dd =  s->params.d - k_presets[i].d;
        float d2 = dn * dn + dd * dd;
        if (d2 < best_dist) { best_dist = d2; best_idx = i; }
    }
    return k_presets[best_idx].name;
}

/* §7 screen — terminal setup, the HUD, and the per-frame draw */

/*
 * Screen -- the current terminal size, remembered so the drawing code can
 * take plain numbers and never has to talk to ncurses itself.  The size is
 * capped at MAX_ROWS/MAX_COLS so the fixed density buffer is never indexed
 * out of bounds; on a huge terminal you get the rose centred with empty
 * margin rather than a mess.  Only screen_init and screen_resize update it.
 *
 * Fields:
 *   cols  width  in character cells.
 *   rows  height in character cells.
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_clamp(Screen *s)
{
    getmaxyx(stdscr, s->rows, s->cols);
    if (s->rows > MAX_ROWS) s->rows = MAX_ROWS;
    if (s->cols > MAX_COLS) s->cols = MAX_COLS;
}

static void screen_init(Screen *s)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
    screen_clamp(s);
}

static void screen_resize(Screen *s, Scene *sc)
{
    endwin(); refresh();
    screen_clamp(s);
    sc->frame = render_frame_make(s->rows, s->cols);
    erase();
}

static void hud_draw_title(void)
{
    attron(COLOR_PAIR(HUD_PAIR_TITLE) | A_BOLD);
    mvprintw(0, 0, " [MAURER ROSE] ");
    attroff(COLOR_PAIR(HUD_PAIR_TITLE) | A_BOLD);
}

static void hud_draw_state(const Scene *s)
{
    attron(COLOR_PAIR(HUD_PAIR_DATA) | A_BOLD);
    mvprintw(0, HUD_DATA_COL,
             "  t:%-9s  n=%4.2f  d=%5.1f\xC2\xB0  [%-11s]  spd=%.2f\xC3\x97 ",
             k_themes[s->theme_idx].name,
             (double)s->params.n,
             (double)s->params.d,
             scene_nearest_preset_name(s),
             (double)s->speed_scale);
    attroff(COLOR_PAIR(HUD_PAIR_DATA) | A_BOLD);
}
/*  The odd-looking \xC2\xB0 and \xC3\x97 are the degree and times signs in
 *  UTF-8.  They're fine here: this is HUD text, not the drawing area that
 *  the project keeps to plain ASCII, and nearly all terminals show them. */

static void hud_draw_engine_stats(const Screen *s, double fps, bool paused)
{
    char buf[64];
    snprintf(buf, sizeof buf, " %5.1f fps  %s ",
             fps, paused ? "PAUSED " : "running");
    int len = (int)strlen(buf);
    if (len >= s->cols) return;
    attron(COLOR_PAIR(HUD_PAIR_DATA) | A_BOLD);
    mvprintw(0, s->cols - len, "%s", buf);
    attroff(COLOR_PAIR(HUD_PAIR_DATA) | A_BOLD);
}

static void hud_draw_action_bar(const Screen *s)
{
    attron(COLOR_PAIR(HUD_PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit   SPACE:pause   r:reset   t:theme   [/]:preset   +/-:speed ");
    attroff(COLOR_PAIR(HUD_PAIR_HINT) | A_BOLD);
}

static void screen_draw_hud(const Screen *s, const Scene *sc, double fps)
{
    hud_draw_title();
    hud_draw_state(sc);
    hud_draw_engine_stats(s, fps, sc->paused);
    hud_draw_action_bar(s);
}

static void screen_draw(Screen *s, Scene *sc, double fps)
{
    erase();
    scene_draw(sc, s->rows, s->cols);
    screen_draw_hud(s, sc, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* §8 app — signal handlers, key actions, and the main loop */

/*
 * App -- the whole program in one struct, kept as a single global (g_app)
 * so the signal handlers can flip a flag on it without a pile of loose
 * globals.  The two flags live here rather than on Scene because quitting
 * and resizing are run-loop business, not rose maths.
 *
 * running and need_resize are volatile sig_atomic_t (not bool) because a
 * signal can change them at any moment: sig_atomic_t is the type the
 * standard promises is safe to touch from a signal handler, and volatile
 * forces the loop to re-read them from memory each pass instead of
 * caching a stale copy in a register and never noticing the signal.
 *
 * Fields:
 *   scene        the simulation state (§6); the heavy part, kept in g_app.
 *   screen       the cached terminal size (§7).
 *   running      cleared to 0 by Ctrl-C / kill to ask the loop to stop.
 *   need_resize  set to 1 by a terminal resize so the loop rebuilds layout.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}

static void cleanup(void) { endwin(); }

static void install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);
}

static void app_init(App *app)
{
    app->running     = 1;
    app->need_resize = 0;
    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.rows, app->screen.cols);
}

/* After a resize: pick up the new size and rebuild the layout. */
static void apply_resize(App *app)
{
    screen_resize(&app->screen, &app->scene);
    app->need_resize = 0;
}

static void action_pause          (Scene *s) { s->paused = !s->paused; }
static void action_reset          (Scene *s)
{
    s->params.n = N_DEFAULT;
    s->params.d = D_DEFAULT;
}
static void action_theme_next     (Scene *s)
{
    s->theme_idx = theme_next(s->theme_idx);
    theme_apply(s->theme_idx);
}
static void action_theme_prev     (Scene *s)
{
    s->theme_idx = theme_prev(s->theme_idx);
    theme_apply(s->theme_idx);
}
static void action_preset_next    (Scene *s)
{
    scene_apply_preset(s, (s->preset_idx + 1) % N_PRESETS);
}
static void action_preset_prev    (Scene *s)
{
    scene_apply_preset(s, (s->preset_idx + N_PRESETS - 1) % N_PRESETS);
}
static void action_speed_faster   (Scene *s)
{
    s->speed_scale *= DRIFT_SPEED_STEP;
    if (s->speed_scale > DRIFT_SPEED_MAX_F) s->speed_scale = DRIFT_SPEED_MAX_F;
}
static void action_speed_slower   (Scene *s)
{
    s->speed_scale /= DRIFT_SPEED_STEP;
    if (s->speed_scale < DRIFT_SPEED_MIN_F) s->speed_scale = DRIFT_SPEED_MIN_F;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case KEY_ESC: return false;
    case ' ':                         action_pause(s);        break;
    case 'r': case 'R':               action_reset(s);        break;
    case 't':                         action_theme_next(s);   break;
    case 'T':                         action_theme_prev(s);   break;
    case ']':                         action_preset_next(s);  break;
    case '[':                         action_preset_prev(s);  break;
    case '+': case '=':               action_speed_faster(s); break;
    case '-': case '_':               action_speed_slower(s); break;
    default: break;
    }
    return true;
}

static bool drain_input(App *app)
{
    int ch;
    while ((ch = getch()) != ERR) {
        if (!app_handle_key(app, ch)) return false;
    }
    return true;
}

/* Update the frames-per-second figure once every FPS_UPDATE_MS. */
static void fps_counter_update(int64_t now, int64_t *window_start,
                               int *frame_count, double *fps_out)
{
    (*frame_count)++;
    int64_t elapsed = now - *window_start;
    if (elapsed < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_out      = (double)(*frame_count) / ((double)elapsed / (double)NS_PER_SEC);
    *frame_count  = 0;
    *window_start = now;
}

/* Sleep until the next frame is due; hand back when that will be. */
static int64_t pace_to_deadline(int64_t deadline)
{
    deadline += TICK_NS;
    clock_sleep_ns(deadline - clock_ns());
    return deadline;
}

/* One frame: advance the drift, then draw. */
static void frame_render(App *app, double fps)
{
    scene_tick(&app->scene);
    screen_draw(&app->screen, &app->scene, fps);
}

int main(void)
{
    install_signal_handlers();

    App *app = &g_app;
    app_init(app);

    int64_t next_deadline = clock_ns();
    int64_t fps_window    = clock_ns();
    int     frame_count   = 0;
    double  fps           = 0.0;

    while (app->running) {
        if (app->need_resize)  apply_resize(app);
        if (!drain_input(app)) { app->running = 0; break; }

        frame_render(app, fps);
        fps_counter_update(clock_ns(), &fps_window, &frame_count, &fps);
        next_deadline = pace_to_deadline(next_deadline);
    }
    return 0;
}
