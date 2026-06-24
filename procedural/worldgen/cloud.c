/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cloud.c — a drifting ASCII sky.
 *
 * For every cell we ask "how thick is the cloud here?" The answer is a
 * noise value; the thicker it is, the heavier the glyph and brighter the
 * tint we draw. Wind slowly shifts where we sample the noise, so the
 * clouds slide across the screen forever. Fifteen presets (cumulus,
 * cirrus, storm...) are just different settings fed to one sampler and
 * one renderer; n/p cycles them.
 *
 * The noise is fractional Brownian motion (fBm): plain Perlin noise
 * stacked at finer and finer detail so clouds have shape at every scale.
 *
 * Related files:
 *   ../fields/perin_noise_flow_showcase.c — the Perlin/fBm reference.
 *   ../worldgen/hydraulic.c — same fBm field used as a height map, not a
 *                             density map. Two uses, one noise scaffold.
 *
 * References the code can't give you:
 *   Perlin (2002) "Improving Noise" — the fade curve + gradient scheme.
 *     https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *   Quílez — "fBm" and "Domain warping": https://iquilezles.org/articles/
 *   Schneider & Vos (2015), "Volumetric Cloudscapes of Horizon: Zero Dawn"
 *     — how cloud type maps to noise shape, the idea behind the presets.
 *   WMO International Cloud Atlas — the genus names: https://cloudatlas.wmo.int/
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

/* ── §1 config — const tables + tunable knobs ───────────────────────────── */

/* Every named number lives here so nothing magic hides in the logic. */
enum {
    /* tick rate — how often the wind advances; ]/[ change it */
    SIM_FPS_MIN         =  10,    /* below this the motion looks jerky          */
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,    /* above this is past the display refresh      */
    SIM_FPS_STEP        =  10,

    /* wind speed knob — +/- double or halve it */
    SPEED_MIN           =   1,
    SPEED_DEF           =   8,    /* the "1.0x" reference speed                  */
    SPEED_MAX           =  64,

    HUD_COLS            =  80,    /* width of the status-line text buffer        */
    FPS_UPDATE_MS       = 500,    /* how often the fps number on screen refreshes */

    /* ncurses colour-pair slots. HUD/HINT are fixed per CLAUDE.md;
     * RAMP_BASE..+7 hold the 8 cloud-density tints of the active theme. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* +0..+7 = the 8 density tints                */
    PAIR_HOT            =  11,    /* the lightning-bolt colour                   */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))
#define RENDER_FPS_CAP       60        /* how many frames we draw per second        */
#define MAX_FRAME_DT_MS     100        /* if a frame stalls longer than this, pretend
                                        * it was only this long, so the sim doesn't
                                        * try to catch up in a runaway burst        */

/* Wind speed at the default knob setting; the user's knob scales these. */
#define WIND_X_BASE          6.0f      /* cells per second, sideways  */
#define WIND_Y_BASE          0.4f      /* a slow vertical drift too    */
#define WARP_DRIFT_BASE      0.05f     /* how fast the swirl pattern churns */

/* The swirl ("domain warp") pre-pass uses its own, coarser noise. The clouds'
 * own detail level lives per-preset in the table below. */
#define OCT_WARP             2
#define WARP_SCALE           0.012f

/* Storm lightning: a few cells flash '/' at the brightest cloud cores.
 * Only 1 in MOD cells strikes, so bolts stay sparse. */
#define LIGHTNING_HASH_MOD   1500u
#define LIGHTNING_HZ         30.0f    /* how often the bolt pattern reshuffles (Hz) */

/* Seed for the noise, also the starting point for the 'r' reseed. */
#define BASE_SEED            0xC0FFEE

/*
 * CloudPreset — one cloud "look" stored as plain numbers, not code.
 *
 * The whole point: a cumulus puff and a cirrus streak differ only in HOW you
 * sample and shade the same noise, not in separate code paths. So every preset
 * is one row here, and all 15 share one sampler (cloud_density) and one
 * renderer (render_clouds). Adding a new sky costs one line. The inner loops
 * never ask "which cloud is this?" — they just read the row they're handed.
 *
 * That "cloud type = the shape of the noise sampling" idea comes from
 * Schneider & Vos (2015) and Quílez's fBm / domain-warping articles.
 */
typedef struct {
    const char *name;        /* shown in the HUD; the real cloud genus it mimics */

    /* --- shape: how to sample the noise (used by cloud_density) ---------- */
    float scale_x, scale_y;  /* how zoomed-in the noise is on each axis. Making
                              * scale_y bigger squashes clouds flat, so they read
                              * as horizontal streaks or bands; roughly 2x bigger
                              * just cancels the fact that terminal cells are
                              * twice as tall as wide, giving round puffs.        */
    int   octaves;           /* how many noise layers to stack (2..5). More =
                              * finer, wispier detail.                            */
    float warp_amt;          /* how much to swirl the sampling. 0 = clean blobs;
                              * higher = churning, turbulent silhouettes.         */
    float wind_mult;         /* this preset's slice of the wind, so different
                              * skies drift at slightly different speeds.         */

    /* --- shading: how to ink the result (used by density_to_level / render) */
    float thresholds[4];     /* the four density cutoffs, rising. Below the first
                              * is clear sky; the rest split cloud into 4 tiers.
                              * Lowering the first cutoff covers more of the sky.  */
    char  glyphs[4];         /* the glyph for each of the four cloud tiers, light
                              * to dense (ASCII only — weight reads as density).   */
    short ramp[4];           /* which theme tint (slot 0..7) each tier uses;
                              * higher slot = brighter, for denser cloud.         */
    bool  lightning;         /* if true, flash sparse '/' bolts at the densest
                              * cores (storm, anvil).                             */
} CloudPreset;

static const CloudPreset presets[] = {
/*   name           sx      sy     oct warp  wind  thresholds                glyphs              ramp        light */
{ "CUMULUS",    0.025f, 0.050f, 4, 3.0f, 1.0f, {0.50f,0.62f,0.74f,0.85f}, {'.','o','O','@'}, {3,5,6,7}, false },
{ "CIRRUS",     0.060f, 0.180f, 3, 0.0f, 1.4f, {0.48f,0.58f,0.66f,0.76f}, {'`','~','~','-'}, {3,4,5,6}, false },
{ "STRATUS",    0.018f, 0.060f, 3, 0.0f, 0.7f, {0.42f,0.52f,0.62f,0.74f}, {',','_','~','#'}, {3,4,5,7}, false },
{ "STORM",      0.022f, 0.045f, 5, 4.0f, 0.9f, {0.40f,0.52f,0.64f,0.74f}, {':','%','#','@'}, {4,5,7,7}, true  },
{ "MACKEREL",   0.055f, 0.110f, 4, 1.0f, 1.1f, {0.50f,0.60f,0.70f,0.80f}, {'.',':','o','O'}, {3,4,6,7}, false },
{ "FOG",        0.012f, 0.024f, 2, 0.0f, 0.3f, {0.30f,0.44f,0.58f,0.72f}, {'.',',',':','#'}, {3,4,5,6}, false },
{ "WISPS",      0.070f, 0.200f, 3, 0.5f, 1.5f, {0.60f,0.70f,0.78f,0.86f}, {'`','`','~','-'}, {3,4,5,6}, false },
{ "MARESTAILS", 0.050f, 0.150f, 3, 2.0f, 1.3f, {0.52f,0.62f,0.72f,0.82f}, {'`','~','-','='}, {3,4,5,6}, false },
{ "STREETS",    0.014f, 0.085f, 3, 0.6f, 1.0f, {0.46f,0.56f,0.66f,0.78f}, {',','-','~','#'}, {3,4,5,7}, false },
{ "POPCORN",    0.040f, 0.080f, 3, 2.0f, 1.0f, {0.56f,0.67f,0.77f,0.87f}, {'.','o','O','@'}, {3,5,6,7}, false },
{ "NIMBUS",     0.022f, 0.044f, 4, 2.5f, 0.6f, {0.34f,0.48f,0.61f,0.75f}, {':','o','#','@'}, {3,4,6,7}, false },
{ "TURBULENT",  0.028f, 0.056f, 5, 6.0f, 1.2f, {0.46f,0.58f,0.70f,0.82f}, {'~','o','%','@'}, {4,5,6,7}, false },
{ "VEIL",       0.030f, 0.060f, 2, 0.0f, 0.8f, {0.40f,0.52f,0.64f,0.78f}, {'`','.',',',':'}, {3,3,4,5}, false },
{ "BILLOWS",    0.022f, 0.075f, 3, 1.5f, 1.0f, {0.46f,0.56f,0.66f,0.78f}, {',','~','o','O'}, {3,4,6,7}, false },
{ "ANVIL",      0.032f, 0.075f, 4, 3.5f, 1.1f, {0.44f,0.56f,0.66f,0.76f}, {'.','%','#','@'}, {3,5,7,7}, true  },
};
#define N_PRESETS ((int)(sizeof presets / sizeof presets[0]))

/*
 * Theme — one named colour palette: an 8-step dim-to-bright gradient plus a
 * colour for lightning, written as xterm-256 colour numbers.
 *
 * Keeping colour separate from cloud thickness means any preset looks right
 * under any palette: a preset says "tier 3," the theme says what tint that is.
 * t/T swap the whole palette live without touching the clouds. Every colour
 * sits in the bright half of the range so even dim cells stay readable on a
 * black terminal (see "Theme Palette Brightness" in CLAUDE.md).
 */
typedef struct {
    const char *name;    /* shown in the HUD                                  */
    short       ramp[8]; /* the dim-to-bright gradient. Clouds only ever use
                          * slots 3..7, so the drawn tiers stay in the bright,
                          * well-spread upper half.                            */
    short       hot;     /* the colour of the '/' lightning bolts             */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name       0    1    2    3    4    5    6    7   hot */
    { "DEFAULT",{ 24,  31,  67, 110, 153, 195, 231, 255 }, 226 },
    { "MATRIX", { 28,  34,  40,  76, 118, 154, 192, 230 }, 226 },
    { "NOVA",   { 60,  91, 134, 165, 207, 219, 225, 231 }, 226 },
    { "MONO",   {240, 243, 245, 247, 249, 251, 253, 255 }, 226 },
    { "OCEAN",  { 24,  25,  31,  38,  45,  51, 117, 195 }, 226 },
    { "FIRE",   { 88, 124, 130, 166, 202, 208, 214, 226 }, 226 },
    { "EARTH",  { 94, 130, 137, 173, 179, 215, 222, 230 }, 226 },
    { "FOREST", { 28,  34,  40,  70,  76, 112, 156, 192 }, 226 },
    { "DESERT", {130, 137, 143, 173, 179, 215, 222, 229 }, 226 },
    { "ARCTIC", { 24,  31,  67, 110, 117, 153, 195, 231 }, 226 },
};

/* ── §2 clock — read the time, sleep ────────────────────────────────────── */
/* Just the two timing helpers; the loop that uses them lives in main(). */

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

/* ── §3 logic — the field math: hash, perlin, fBm, density, level ───────── */
/* These functions only read; they never change anything, so the renderer
 * can't corrupt them. The one thing they share is perm[] (the noise table),
 * which they only read — perm_shuffle in §4 is what fills it. */

/* hash3 — turns three ints into a scrambled number; used by the lightning gate. */
static inline uint32_t hash3(int wx, int wy, int wz)
{
    uint32_t h = (uint32_t)wx * 73856093u
               ^ (uint32_t)wy * 19349663u
               ^ (uint32_t)wz * 83492791u;
    h ^= h >> 16;
    h *= 0x85ebca6bu;
    h ^= h >> 13;
    h *= 0xc2b2ae35u;
    h ^= h >> 16;
    return h;
}

/* perm[] is the shuffled lookup table that gives the noise its randomness.
 * Read here; only perm_shuffle (§4) writes it, and only on startup or 'r'. */
static uint8_t perm[512];

static inline float fade_q(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}
static inline float lerp_f(float a, float b, float t) { return a + t * (b - a); }
static inline float grad2(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

/* perlin2d — smooth random value at a point. Find the grid square the point
 * sits in, take a random direction at each of its four corners, and blend them
 * with a soft S-curve so neighbouring squares join seamlessly.
 * (floorf, not a plain int cast, so negative coords round the right way.) */
static float perlin2d(float x, float y)
{
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x); y -= floorf(y);          /* position inside the square */

    float u = fade_q(x), v = fade_q(y);       /* the soft S-curve blend weights */

    int A = perm[X    ] + Y;
    int B = perm[X + 1] + Y;

    float n00 = grad2(perm[A    ], x,        y       );   /* the four corners */
    float n10 = grad2(perm[B    ], x - 1.0f, y       );
    float n01 = grad2(perm[A + 1], x,        y - 1.0f);
    float n11 = grad2(perm[B + 1], x - 1.0f, y - 1.0f);

    return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

/* fbm2_n — stack several noise layers, each finer and fainter than the last,
 * to get detail at every scale. Returns roughly 0..1, densest around 0.5. */
static float fbm2_n(float x, float y, int octaves)
{
    float total = 0.0f, amp = 1.0f, freq = 1.0f, max_amp = 0.0f;
    for (int o = 0; o < octaves; o++) {
        total   += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;
}

/*
 * cloud_density — how thick is this preset's cloud at cell (x, y)?
 *
 * First, if the preset wants swirl, nudge the sample point around using a
 * second, coarse noise — this turns clean blobs into churning shapes. Then
 * stretch the point by the preset's per-axis scales (which flattens clouds
 * into streaks or bands) and read the main noise there.
 */
static float cloud_density(const CloudPreset *p, float x, float y,
                           float wind_x, float wind_y, float warp_t)
{
    float wx = x + wind_x * p->wind_mult;
    float wy = y + wind_y;
    if (p->warp_amt > 0.0f) {
        float qx = fbm2_n(x * WARP_SCALE, y * WARP_SCALE + warp_t, OCT_WARP);
        float qy = fbm2_n((x + 5.2f) * WARP_SCALE,
                          (y + 1.3f) * WARP_SCALE + warp_t, OCT_WARP);
        wx += qx * p->warp_amt;
        wy += qy * p->warp_amt;
    }
    return fbm2_n(wx * p->scale_x, wy * p->scale_y, p->octaves);
}

/* density_to_level — turn a thickness into a tier 0..4 using the preset's
 * cutoffs. 0 is clear sky (draw nothing); 4 is the bright core. */
static inline int density_to_level(float d, const CloudPreset *p)
{
    if (d < p->thresholds[0]) return 0;
    if (d < p->thresholds[1]) return 1;
    if (d < p->thresholds[2]) return 2;
    if (d < p->thresholds[3]) return 3;
    return 4;
}

/* ── §4 simulation — the state that changes, and what changes it ────────── */
/* Two things move: perm[] (the noise table, only on startup or 'r') and the
 * Scene's drift (wind and time, once per tick). 'paused' freezes the tick. */

/* perm_shuffle — refill the noise table with a fresh random shuffle. The only
 * place perm[] gets written. Called on startup and on 'r', never in the tick. */
static void perm_shuffle(int seed)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;

    /* shuffle the 0..255 list into random order, driven by the seed */
    uint32_t st = (uint32_t)seed * 2654435761u;
    for (int i = 255; i > 0; i--) {
        st = st * 1664525u + 1013904223u;
        int j = (int)(st >> 16) % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }

    /* keep a second copy right after the first so perlin2d can index past the
     * end (perm[x]+y) without ever wrapping around */
    for (int i = 0; i < 256; i++) {
        perm[i      ] = base[i];
        perm[i + 256] = base[i];
    }
}

/*
 * Drift — how far the sky has slid so far.
 *
 * The noise itself never moves; we make it look like it does by sampling a
 * little further along each frame. So these are running totals of "how far,"
 * not speeds (the speed knob is Scene.speed). They only grow, advanced together
 * each tick; the renderer just reads them. They're floats, so after a very long
 * run the numbers get coarse — fine for a demo that runs minutes.
 */
typedef struct {
    float time_secs;   /* seconds since start; also drives the lightning flicker */
    float wind_x;      /* how far the sky has drifted sideways, in cells          */
    float wind_y;      /* how far it has drifted vertically, in cells             */
    float warp_t;      /* a separate clock for the swirl, so it churns on its own */
} Drift;

/*
 * Scene — everything about the running sky in one place: the drift, the knobs
 * the user turns, and the chosen palette. The theme is grouped apart because it
 * only affects colour, not the clouds themselves. Most functions take a smaller
 * piece (just the Drift, just a CloudPreset) so the layers stay independent.
 */
typedef struct {
    Drift drift;               /* the moving sky; advanced once per tick         */
    bool  paused;              /* true freezes the drift                         */
    int   speed;               /* wind speed knob, 1..64 (default 8)             */
    int   current_preset;      /* which sky is showing (index into presets[])    */
    int   current_theme;       /* which palette is active (index into themes[])  */
} Scene;

/* scene_reseed — pick a new random sky from the current moment, so each 'r'
 * gives a different layout. Only touches the noise table, not the scene. */
static void scene_reseed(const Scene *s)
{
    int seed = (int)hash3((int)(s->drift.time_secs * 1000.0f),
                          (int)(s->drift.wind_x * 100.0f), BASE_SEED);
    perm_shuffle(seed);
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_preset  = 0;
    perm_shuffle(BASE_SEED);
}

/* scene_tick — nudge the sky forward by one step. Clouds drift forever; the
 * only way to get a new layout is 'r'. */
static void scene_tick(Scene *s, float dt)
{
    s->drift.time_secs += dt;
    if (s->paused) return;

    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->drift.wind_x += WIND_X_BASE * speed_mul * dt;
    s->drift.wind_y += WIND_Y_BASE * speed_mul * dt;
    s->drift.warp_t += WARP_DRIFT_BASE * speed_mul * dt;
}

/* ── §5 render — paint the scene onto the screen ────────────────────────── */
/* All the ncurses drawing lives here. It reads the scene and paints; it never
 * changes the simulation. Lightning isn't stored anywhere — it's decided here,
 * at draw time, for cells that land on a bright core. */

/* set_palette_pairs — load one palette into ncurses: 8 tints + the bolt colour. */
static void set_palette_pairs(const short ramp[8], short hot)
{
    for (int i = 0; i < 8; i++)
        init_pair((short)(PAIR_RAMP_BASE + i), ramp[i], -1);
    init_pair(PAIR_HOT, hot, -1);
}

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        set_palette_pairs(t->ramp, t->hot);
    } else {
        /* old 8-colour terminals can't do the palette, so fall back to a
         * fixed cool-to-bright ramp */
        static const short fb[8] = {
            COLOR_BLUE, COLOR_BLUE,  COLOR_CYAN,   COLOR_CYAN,
            COLOR_WHITE,COLOR_WHITE, COLOR_YELLOW, COLOR_WHITE,
        };
        set_palette_pairs(fb, COLOR_YELLOW);
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
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* Screen — the current terminal size. Passing this around (instead of a global)
 * keeps the width/height correct after a resize. Refreshed from getmaxyx. */
typedef struct {
    int cols, rows;   /* width / height in character cells */
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

/* draw_cell — put one glyph on screen, brightening it for denser tiers. */
static inline void draw_cell(int sy, int sx, char glyph, int pair, int level)
{
    int attr;
    if      (level >= 4) attr = A_BOLD;
    else if (level >= 3) attr = A_BOLD;
    else if (level >= 2) attr = A_NORMAL;
    else                 attr = A_DIM;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

/* lightning_strikes — does a bolt flash at this cell right now? Only at the
 * brightest cores of a storm preset, and only 1 cell in MOD, so it stays rare. */
static inline bool lightning_strikes(const CloudPreset *p, int level,
                                     int sx, int sy, int t)
{
    return p->lightning && level == 4 &&
           (hash3(sx, sy, t) % LIGHTNING_HASH_MOD) == 0u;
}

/* render_clouds — for every cell of sky, work out how thick the cloud is,
 * pick the matching glyph and tint, and draw it; storms add the odd bolt. */
static void render_clouds(const Screen *sc, const Drift *drift,
                          const CloudPreset *p)
{
    int top = 2, bottom = sc->rows - 1;          /* leave the HUD rows alone */
    int strike_bucket = (int)(drift->time_secs * LIGHTNING_HZ);

    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < sc->cols; sx++) {
            float d = cloud_density(p, (float)sx, (float)sy,
                                    drift->wind_x, drift->wind_y, drift->warp_t);
            int level = density_to_level(d, p);
            if (level == 0) continue;            /* clear sky — draw nothing */

            char glyph = p->glyphs[level - 1];
            int  pair  = PAIR_RAMP_BASE + p->ramp[level - 1];

            if (lightning_strikes(p, level, sx, sy, strike_bucket)) {
                glyph = '/';
                pair  = PAIR_HOT;
            }
            draw_cell(sy, sx, glyph, pair, level);
        }
    }
}

static void scene_draw(const Screen *sc, const Scene *s)
{
    render_clouds(sc, &s->drift, &presets[s->current_preset]);
}

/* top-right status: fps, tick rate, paused/drifting, wind speed. */
static void hud_status_line(const Screen *sc, const Scene *s,
                            double fps, int sim_fps)
{
    const char *state_str = s->paused ? "PAUSED" : "DRIFT ";
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* top-left: the title plus which preset is showing (e.g. 3/15 STRATUS). */
static void hud_title(const Scene *s)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " CLOUD (fBm)  %2d/%d %-10s ",
             s->current_preset + 1, N_PRESETS,
             presets[s->current_preset].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* second row: theme name, a little swatch of the palette, and the drift so far. */
static void hud_param_line(const Scene *s)
{
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " ramp:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 6;
    static const char ramp_glyphs[8] = { '`', '.', ',', ':', '-', 'o', '#', '@' };
    for (int i = 0; i < 8; i++) {
        int p = PAIR_RAMP_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, (chtype)(unsigned char)ramp_glyphs[i]);
        attroff(COLOR_PAIR(p) | A_BOLD);
        x++;
    }

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  wind:%6.1f,%5.1f  warp:%5.2f ",
             (double)s->drift.wind_x, (double)s->drift.wind_y,
             (double)s->drift.warp_t);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* bottom row: the list of keys you can press. */
static void hud_key_hints(const Screen *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:wind  ]/[:tickHz  spc:pause  r:reseed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);
    hud_status_line(sc, s, fps, sim_fps);
    hud_title(s);
    hud_param_line(s);
    hud_key_hints(sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §6 app — keys, resize, and the main loop ───────────────────────────── */
/* main() is the one place the sky actually advances. Keys and resizes also
 * change state, but they happen on input, outside the timed loop. */

/*
 * App — the whole running program: the sky plus the bookkeeping that drives it.
 * There's exactly one (g_app) so the signal handlers can reach the run flags;
 * everything else is reached through it.
 */
typedef struct {
    Scene                 scene;       /* the simulated sky                    */
    Screen                screen;      /* the terminal it's drawn into         */
    int                   sim_fps;     /* tick rate, 10..240                   */
    volatile sig_atomic_t running;     /* loop runs while non-zero; 'q' or a
                                        * kill signal clears it                */
    volatile sig_atomic_t need_resize; /* a signal handler sets this; volatile +
                                        * sig_atomic_t makes that safe          */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app->need_resize = 0;
}

/* step an index forward / backward, wrapping around the ends */
static inline int wrap_inc(int i, int n) { return (i + 1)     % n; }
static inline int wrap_dec(int i, int n) { return (i + n - 1) % n; }

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reseed(s);                               break;

    case '=': case '+':                       /* double wind speed, capped */
        if (s->speed < SPEED_MAX) s->speed *= 2;
        if (s->speed > SPEED_MAX) s->speed  = SPEED_MAX;
        break;
    case '-':                                 /* halve wind speed, floored */
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed  = SPEED_MIN;
        break;

    case ']':                                 /* faster tick, capped */
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':                                 /* slower tick, floored */
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->current_theme = wrap_inc(s->current_theme, N_THEMES);
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = wrap_dec(s->current_theme, N_THEMES);
        theme_apply(s->current_theme);
        break;

    case 'n': case 'N': s->current_preset = wrap_inc(s->current_preset, N_PRESETS); break;
    case 'p': case 'P': s->current_preset = wrap_dec(s->current_preset, N_PRESETS); break;

    default: break;
    }
    return true;
}

/* app_init — set everything up before the first frame: signals, the terminal,
 * and a fresh scene. */
static void app_init(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene);
}

int main(void)
{
    App *app = &g_app;
    app_init(app);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {       /* EVENT (not a tick): resize */
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ---- per-frame combine — the ONLY place sim state advances ----
         * PERFORMANCE: measure + clamp dt
         * SIMULATION : scene_tick × N   (fixed timestep; skipped if paused)
         * PERFORMANCE: fps tally + frame-cap sleep
         * RENDER     : screen_draw (→ scene_draw → LOGIC per cell) + present
         * EVENTS     : getch → app_handle_key   (NOT part of the tick)     */

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > MAX_FRAME_DT_MS * NS_PER_MS) dt = MAX_FRAME_DT_MS * NS_PER_MS;

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
        clock_sleep_ns(NS_PER_SEC / RENDER_FPS_CAP - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
