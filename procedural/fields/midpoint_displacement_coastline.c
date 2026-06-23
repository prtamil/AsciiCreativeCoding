/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * midpoint_displacement_coastline.c
 *
 * Draws jagged fractal lines that look like coastlines, mountains, or
 * lightning. The trick: take a straight line, push its midpoint up or
 * down at random, then do the same to each new half with a smaller
 * nudge each time. Thirty named presets dress the same idea up as
 * different scenes, and every line slowly morphs between two random
 * shapes so the picture is always moving.
 *
 * Algorithm: Fournier, Fussell & Carpenter, "Computer rendering of
 * stochastic models", CACM 25(6) 1982 — the paper that introduced this.
 * Glyph ramps: Paul Bourke, https://paulbourke.net/dataformats/asciiart/
 *
 * Sister files (same scene structure, different algorithm):
 *   ../generational/diamond_square_heightmap_showcase.c — the 2-D version
 *   ./magnetic_fields.c, ./flow_field_particles.c, ./curl_noise_vector_field.c
 *
 * Keys: q/ESC quit · space pause · r reset · n/p pattern · t/T theme
 *       g/G glyph set · +/- morph speed · ]/[ tick rate
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra midpoint_displacement_coastline.c \
 *       -o md_coast -lncurses -lm
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    CELLS_MAX         = MAP_W_MAX * MAP_H_MAX,

    /* How many points make up each line. Must be a power of two plus
     * one; 257 means we halve 8 times, which is plenty of detail. */
    LINE_POINTS       = 257,

    /* Most lines any one preset draws (CHAOS uses all 8). */
    MAX_LINES         =   8,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* Colour-pair slots. The HUD slots are reserved project-wide. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,    /* first of 4 line colours: base..base+3 */
    PAIR_FLASH        =   7,    /* unused here; kept so themes match sister files */
};

/* How fast a line morphs from its current shape to the next, as a
 * fraction of the trip per second. The default finishes in 10 seconds. */
#define MORPH_RATE_DEF      0.10f
#define MORPH_RATE_MIN      0.01f
#define MORPH_RATE_MAX      2.00f

/* How many colours a theme offers. Each line picks one of them. */
#define N_BANDS             4

/* CHAOS preset — the random ranges every line is rolled from. Pulling
 * these out here lets you re-tune CHAOS without editing the placer. */
#define CHAOS_LINES_TARGET   8       /* how many lines to try for (capped)  */
#define CHAOS_Y_LO           0.10f   /* vertical position: low end          */
#define CHAOS_Y_RANGE        0.80f   /* vertical position: spread           */
#define CHAOS_AMP_LO         0.05f   /* jaggedness height: low end          */
#define CHAOS_AMP_RANGE      0.20f   /* jaggedness height: spread           */
#define CHAOS_ROUGH_LO       0.30f   /* roughness: low end                  */
#define CHAOS_ROUGH_RANGE    0.55f   /* roughness: spread                   */
#define CHAOS_OUTLINE_DENOM  3       /* 1-in-3 chance a line gets an outline */
#define CHAOS_STYLE_COUNT    3       /* number of render styles to pick from */

/* The two fixed numbers in the fractal step itself. */
#define MIDPOINT_AVG_WEIGHT  0.5f   /* take the halfway point between two ends */
#define JITTER_HALFRANGE     0.5f   /* endpoints wobble by up to half the amplitude */

/* HUD layout. Row 0 and row 1 at the top show info, the bottom row
 * shows the key list, and the scene fills everything between. The
 * row-1 fields sit at fixed column widths so each knows where the next
 * begins — change one width and the whole row still lines up. */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1
#define HUD_PATTERN_FIELD_W     20    /* width of the " pattern:XXX " field */
#define HUD_THEME_FIELD_W       17    /* width of the " theme:XXX " field   */
#define HUD_PALETTE_LABEL_W      9    /* width of the " palette:" label     */

/*
 * Pattern — the 30 named scenes, from simplest to most elaborate.
 *
 * The engine (build a fractal line, morph it) is the same for all of
 * them; a preset is just a list of lines with different settings. So
 * adding one is three small edits: a placer function, an enum value
 * here, and a line in the dispatcher.
 *
 * They're grouped in five tiers, simplest first, and the values run in
 * order so pressing n/p walks from easy to hard. The tiers are: one
 * line, two lines, a few layered lines, many shallow lines, then the
 * exotic ones with extreme settings.
 */
typedef enum {
    /* Tier 1 — single-line silhouettes */
    PATTERN_COASTLINE =  0,
    PATTERN_SKYLINE   =  1,
    PATTERN_HORIZON   =  2,
    PATTERN_DUNES     =  3,
    PATTERN_CITY      =  4,

    /* Tier 2 — paired lines */
    PATTERN_VALLEY    =  5,
    PATTERN_CAVE      =  6,
    PATTERN_CLIFF     =  7,
    PATTERN_PLATEAU   =  8,
    PATTERN_REEF      =  9,

    /* Tier 3 — small layered stacks */
    PATTERN_MOUNTAINS = 10,
    PATTERN_HILLS     = 11,
    PATTERN_ALPS      = 12,
    PATTERN_RIDGES    = 13,
    PATTERN_TERRACES  = 14,

    /* Tier 4 — multi-layer stacks */
    PATTERN_WAVES     = 15,
    PATTERN_STRATA    = 16,
    PATTERN_FOREST    = 17,
    PATTERN_RAPIDS    = 18,
    PATTERN_SEDIMENT  = 19,

    /* Tier 5 — exotic / max-complex */
    PATTERN_ATOLLS    = 20,
    PATTERN_CRYSTAL   = 21,
    PATTERN_SAWTOOTH  = 22,
    PATTERN_NEBULA    = 23,
    PATTERN_STORM     = 24,
    PATTERN_AURORA    = 25,
    PATTERN_CEILING   = 26,
    PATTERN_ISLANDS   = 27,
    PATTERN_PETRA     = 28,
    PATTERN_CHAOS     = 29,

    N_PATTERNS        = 30,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_COASTLINE: return "COASTLINE";
    case PATTERN_SKYLINE:   return "SKYLINE  ";
    case PATTERN_HORIZON:   return "HORIZON  ";
    case PATTERN_DUNES:     return "DUNES    ";
    case PATTERN_CITY:      return "CITY     ";
    case PATTERN_VALLEY:    return "VALLEY   ";
    case PATTERN_CAVE:      return "CAVE     ";
    case PATTERN_CLIFF:     return "CLIFF    ";
    case PATTERN_PLATEAU:   return "PLATEAU  ";
    case PATTERN_REEF:      return "REEF     ";
    case PATTERN_MOUNTAINS: return "MOUNTAINS";
    case PATTERN_HILLS:     return "HILLS    ";
    case PATTERN_ALPS:      return "ALPS     ";
    case PATTERN_RIDGES:    return "RIDGES   ";
    case PATTERN_TERRACES:  return "TERRACES ";
    case PATTERN_WAVES:     return "WAVES    ";
    case PATTERN_STRATA:    return "STRATA   ";
    case PATTERN_FOREST:    return "FOREST   ";
    case PATTERN_RAPIDS:    return "RAPIDS   ";
    case PATTERN_SEDIMENT:  return "SEDIMENT ";
    case PATTERN_ATOLLS:    return "ATOLLS   ";
    case PATTERN_CRYSTAL:   return "CRYSTAL  ";
    case PATTERN_SAWTOOTH:  return "SAWTOOTH ";
    case PATTERN_NEBULA:    return "NEBULA   ";
    case PATTERN_STORM:     return "STORM    ";
    case PATTERN_AURORA:    return "AURORA   ";
    case PATTERN_CEILING:   return "CEILING  ";
    case PATTERN_ISLANDS:   return "ISLANDS  ";
    case PATTERN_PETRA:     return "PETRA    ";
    case PATTERN_CHAOS:     return "CHAOS    ";
    default:                return "?        ";
    }
}

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Loop safety valves. If a frame takes too long (laptop slept, etc.)
 * we pretend no more than 100 ms passed so the sim doesn't try to catch
 * up forever. And we never redraw faster than this to spare the terminal.
 * Both ideas: Glenn Fiedler, "Fix Your Timestep". */
#define DT_MAX_NS       (100 * NS_PER_MS)
#define FRAME_CAP_FPS   60

/*
 * Theme — one colour scheme for the lines. Ten of them, cycled with t/T.
 *
 * Each has four colours ordered dim to bright. A line names which of
 * the four it wants, and stacked patterns hand the brightest to the
 * back and dimmest to the front so colour helps sell the sense of depth.
 *
 * The numbers are xterm-256 colour codes, not RGB. On an old terminal
 * with fewer colours, theme_apply() falls back to a plain 8-colour set
 * so the demo still runs.
 */
typedef struct {
    const char *name;                /* short label shown in the HUD            */
    short       band[N_BANDS];       /* four colours, dim (0) to bright (3)     */
    short       flash;               /* unused here; kept to match sister files */
} Theme;

#define N_THEMES 10

/*
 * Every colour here is deliberately on the bright side. These patterns
 * paint solid blocks below their lines, and a dark colour would just
 * vanish into a dark terminal background and leave the shape invisible.
 * Keeping all of them light means the scene reads on dark, light, or
 * see-through terminals alike. (Sister files that only draw thin lines
 * don't need this and use the full range of colours.)
 */
static const Theme themes[N_THEMES] = {
    /*           name      band0 band1 band2 band3 flash */
    { "DEFAULT", {  39,  117,  220,  231 }, 226 },   /* sky → cyan → gold → white */
    { "MATRIX",  {  46,   82,  118,  154 }, 226 },   /* bright greens             */
    { "NOVA",    {  99,  165,  213,  219 }, 226 },   /* purple → pink (light)     */
    { "MONO",    { 245,  248,  251,  254 }, 226 },   /* light greyscale           */
    { "OCEAN",   {  39,   51,   81,  159 }, 226 },   /* sky → cyan (all light)    */
    { "FIRE",    { 166,  208,  220,  226 }, 196 },   /* orange → yellow           */
    { "EARTH",   { 137,  173,  215,  230 }, 226 },   /* light tans + cream        */
    { "FOREST",  {  64,  107,  144,  187 }, 226 },   /* mid-green → tan           */
    { "DESERT",  { 179,  215,  222,  230 }, 226 },   /* light sand                */
    { "ARCTIC",  { 117,  153,  195,  231 }, 226 },   /* light blue → white        */
};

/*
 * GlyphSet — three characters going from thin to thick. Five sets, each
 * a different look (dots, stars, blocks...), cycled with g/G.
 *
 * Some themes read better with heavier characters and some with lighter
 * ones, so glyph choice is separate from colour choice and you can mix
 * them however you like. A line says whether it wants the thin, middle,
 * or thick character; the renderer looks it up here at draw time.
 *
 * The idea of using a few characters of different "weight" to stand for
 * light and dark comes from Paul Bourke's ASCII art ramp:
 *   https://paulbourke.net/dataformats/asciiart/
 * Three steps is enough here — more would just add visual noise.
 */
typedef struct {
    const char *name;             /* short label shown in the HUD       */
    char        low;              /* thinnest — used for far-back layers */
    char        mid;              /* middle weight                      */
    char        high;            /* thickest — used for front layers and outlines */
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
            init_pair(PAIR_BAND_BASE + i, t->band[i], -1);
        init_pair(PAIR_FLASH, t->flash, -1);
    } else {
        static const short fallback[N_BANDS] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < N_BANDS; i++)
            init_pair(PAIR_BAND_BASE + i, fallback[i], -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
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
/* §5  md — midpoint displacement core                                    */
/* ===================================================================== */

/* A random number between -1 and +1. Every bit of randomness here
 * starts from this. */
static inline float rand_signed(void)
{
    return ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
}

/* The point halfway between two heights. */
static inline float midpoint_y(float left, float right)
{
    return (left + right) * MIDPOINT_AVG_WEIGHT;
}

/* A random nudge up or down, no bigger than amp. */
static inline float jitter(float amp)
{
    return rand_signed() * amp;
}

/*
 * Builds one fractal line into line[0..n-1], running from height left_y
 * on the left to right_y on the right.
 *
 * The shape: set the two ends, find the midpoint and nudge it randomly,
 * then do the same to each new half with a smaller nudge, and keep
 * halving until every slot is filled. amp_init is how big the first
 * nudge is; roughness is how much of the nudge survives to the next,
 * finer level. n must be a power of two plus one (like 257).
 *
 * Written as a loop rather than recursion — same result, no deep call
 * stack to worry about.
 */
static void md_generate(float *line, int n,
                        float left_y, float right_y,
                        float amp_init, float roughness)
{
    line[0]     = left_y;
    line[n - 1] = right_y;

    int   step = n - 1;
    float amp  = amp_init;
    while (step > 1) {
        int half = step / 2;
        for (int i = half; i < n - 1; i += step) {
            line[i] = midpoint_y(line[i - half], line[i + half])
                    + jitter(amp);
        }
        step  = half;
        amp  *= roughness;
    }
}

/*
 * Blends two lines into out[]: t=0 gives line a, t=1 gives line b, and
 * values in between give a mix. This is what makes the shapes morph.
 */
static void line_lerp(const float *a, const float *b, float *out, int n, float t)
{
    float u = 1.0f - t;
    for (int i = 0; i < n; i++) {
        out[i] = u * a[i] + t * b[i];
    }
}

/*
 * Reads the line's height at screen column x. The line has a fixed
 * number of points but the terminal can be any width, so this stretches
 * it to fit, smoothing between the two nearest points.
 */
static float line_sample(const float *line, int n_points, int w, int x)
{
    if (w <= 1) return line[0];
    float fx = (float)x * (float)(n_points - 1) / (float)(w - 1);
    int   xi = (int)fx;
    if (xi < 0) xi = 0;
    if (xi >= n_points - 1) return line[n_points - 1];
    float frac = fx - (float)xi;
    return line[xi] * (1.0f - frac) + line[xi + 1] * frac;
}

/* ===================================================================== */
/* §6  patterns — 30 line configurations + render-style dispatcher        */
/* ===================================================================== */

/*
 * RenderStyle — how a line gets painted.
 *   FILL_BELOW — colour everything from the line down to the floor
 *                (the ground in COASTLINE, the mass of a mountain).
 *   FILL_ABOVE — colour everything from the ceiling down to the line
 *                (sky, cave roofs, stalactites).
 *   THIN_LINE  — just the line itself, one character wide.
 */
typedef enum {
    STYLE_FILL_BELOW = 0,
    STYLE_FILL_ABOVE = 1,
    STYLE_THIN_LINE  = 2,
} RenderStyle;

/*
 * GlyphDensity — which of the glyph set's three characters a line uses.
 * Stacked patterns give far layers the thin one and near layers the
 * thick one, which makes them look like they're at different distances.
 */
typedef enum {
    GLYPH_LOW  = 0,
    GLYPH_MID  = 1,
    GLYPH_HIGH = 2,
} GlyphDensity;

/*
 * LineSpec — the full recipe for one line: how to build its shape and
 * how to paint it. A preset is just a list of these. The first half
 * feeds the fractal builder; the second half tells the renderer what to
 * draw. These four shape knobs are all you need to get everything from
 * a gentle coast to a spiky crystal.
 *
 * The roughness knob is the interesting one: it's how much of the random
 * nudge carries over to each finer level. Low values smooth out fast
 * (dunes, terraces); high values keep the small wobbles all the way down
 * (cities, crystals). Voss 1985 and Peitgen & Saupe 1988 ch.2 cover the
 * theory and how to tune it.
 */
typedef struct {
    /* --- shape: how to build the line --- */

    /* Where the line sits vertically, in screen rows. The line wanders
     * around this height. */
    float        y_centre;

    /* How big the first random nudge is, in rows. Bigger means taller
     * peaks and deeper dips. */
    float        amp_init;

    /* How jagged the line is: how much of each nudge survives to the
     * next, finer level. Below 0.5 looks smooth, above 0.5 looks rough.
     * We use the range 0.30 to 0.85. */
    float        roughness;

    /* Shift the two ends up or down from y_centre. Use these to tilt the
     * whole line (a cliff, a sawtooth, tilted rock layers). Zero means
     * both ends sit level. */
    float        left_dy, right_dy;

    /* --- look: how to paint the line --- */

    /* Which of the theme's four colours to fill with (0..3). */
    int          pair_idx;

    /* Optional second colour for a thin outline drawn on top of the
     * fill, or -1 for none. This is how COASTLINE gets a bright shore
     * line over its water and CITY a crisp skyline over its buildings. */
    int          outline_idx;

    /* Fill below, fill above, or just the line — see RenderStyle. */
    RenderStyle  style;

    /* Which of the three glyph weights to use; far layers go thin, near
     * layers go thick to fake depth. */
    GlyphDensity glyph_density;
} LineSpec;

/* ── tier 1 — one line each ──────────────────────────────────────── *
 * Just one fractal line. Changing its height, roughness, and fill
 * direction is enough to make five different scenes. */

/* COASTLINE — water filled below a gentle shore. The classic example. */
static int place_coastline(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.60f, .amp_init = fh * 0.20f, .roughness = 0.55f,
        .pair_idx = 0, .outline_idx = 2,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* SKYLINE — COASTLINE flipped: fill above the line, so it reads as a
 * coloured ceiling over open ground. */
static int place_skyline(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.30f, .amp_init = fh * 0.18f, .roughness = 0.55f,
        .pair_idx = 3, .outline_idx = 1,
        .style = STYLE_FILL_ABOVE, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* HORIZON — just the bare line, nothing filled. Shows what the
 * algorithm produces on its own. */
static int place_horizon(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.50f, .amp_init = fh * 0.10f, .roughness = 0.50f,
        .pair_idx = 2, .outline_idx = -1,
        .style = STYLE_THIN_LINE, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* DUNES — low roughness so the small wobbles fade fast, leaving only
 * gentle rolling waves like sand dunes. */
static int place_dunes(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.65f, .amp_init = fh * 0.15f, .roughness = 0.40f,
        .pair_idx = 1, .outline_idx = -1,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_MID,
    };
    return 1;
}

/* CITY — the opposite of DUNES. High roughness keeps the small wobbles,
 * so sharp peaks read as skyscrapers and the dips as streets. */
static int place_city(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.70f, .amp_init = fh * 0.30f, .roughness = 0.75f,
        .pair_idx = 1, .outline_idx = 3,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* ── tier 2 — two lines each ─────────────────────────────────────── *
 * A fill-above line and a fill-below line together can frame an open
 * space between them. */

/* VALLEY — a canyon: rock filled down from the top, rock filled up from
 * the bottom, open air in the gap between. */
static int place_valley(LineSpec out[], int max_n, int h)
{
    if (max_n < 2) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.25f, .amp_init = fh * 0.15f, .roughness = 0.55f,
        .pair_idx = 2, .outline_idx = -1,
        .style = STYLE_FILL_ABOVE, .glyph_density = GLYPH_MID,
    };
    out[1] = (LineSpec){
        .y_centre = fh * 0.75f, .amp_init = fh * 0.15f, .roughness = 0.55f,
        .pair_idx = 0, .outline_idx = -1,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_MID,
    };
    return 2;
}

/* CAVE — a tighter VALLEY with a brighter outline on the roof and floor
 * for a glowing-edge look. */
static int place_cave(LineSpec out[], int max_n, int h)
{
    if (max_n < 2) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.40f, .amp_init = fh * 0.08f, .roughness = 0.55f,
        .pair_idx = 2, .outline_idx = 3,
        .style = STYLE_FILL_ABOVE, .glyph_density = GLYPH_HIGH,
    };
    out[1] = (LineSpec){
        .y_centre = fh * 0.60f, .amp_init = fh * 0.08f, .roughness = 0.55f,
        .pair_idx = 2, .outline_idx = 3,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 2;
}

/* CLIFF — one end low, the other high, so the line slopes; the random
 * wobble on top makes the slope look like real rock. */
static int place_cliff(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre  = fh * 0.50f,
        .amp_init  = fh * 0.10f, .roughness = 0.55f,
        .left_dy   = -fh * 0.25f, .right_dy = +fh * 0.25f,
        .pair_idx = 1, .outline_idx = 3,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* PLATEAU — very low roughness flattens the line into one big step:
 * a broad flat top with steep sides, like a mesa. */
static int place_plateau(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.50f, .amp_init = fh * 0.25f, .roughness = 0.30f,
        .pair_idx = 1, .outline_idx = 3,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* REEF — two filled layers: a dim sea floor behind, a bright outlined
 * coral reef in front that paints over it. */
static int place_reef(LineSpec out[], int max_n, int h)
{
    if (max_n < 2) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.85f, .amp_init = fh * 0.06f, .roughness = 0.55f,
        .pair_idx = 0, .outline_idx = -1,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_LOW,
    };
    out[1] = (LineSpec){
        .y_centre = fh * 0.60f, .amp_init = fh * 0.18f, .roughness = 0.65f,
        .pair_idx = 2, .outline_idx = 3,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 2;
}

/* ── tier 3 — a few layered lines ────────────────────────────────── *
 * Three or four lines stacked. Far layers use the thinnest character
 * and near layers the thickest, which gives a sense of depth. */

/* MOUNTAINS — four ranges, the farther ones sitting higher and dimmer.
 * (Fournier, Fussell & Carpenter 1982, fig. 3.) */
static int place_mountains(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[4]    = { 0.45f, 0.55f, 0.65f, 0.75f };
    const float amps[4]  = { 0.18f, 0.20f, 0.22f, 0.25f };
    const int   pairs[4] = { 3, 2, 1, 0 };
    const GlyphDensity gd[4] = { GLYPH_LOW, GLYPH_LOW, GLYPH_MID, GLYPH_HIGH };
    for (int i = 0; i < 4 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * amps[i], .roughness = 0.58f,
            .pair_idx = pairs[i], .outline_idx = -1,
            .style = STYLE_FILL_BELOW, .glyph_density = gd[i],
        };
    }
    return n;
}

/* HILLS — MOUNTAINS but smoother, for gentle rolling shapes. */
static int place_hills(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[3]    = { 0.55f, 0.65f, 0.75f };
    const float amps[3]  = { 0.18f, 0.20f, 0.22f };
    const int   pairs[3] = { 3, 2, 0 };
    const GlyphDensity gd[3] = { GLYPH_LOW, GLYPH_MID, GLYPH_HIGH };
    for (int i = 0; i < 3 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * amps[i], .roughness = 0.40f,
            .pair_idx = pairs[i], .outline_idx = -1,
            .style = STYLE_FILL_BELOW, .glyph_density = gd[i],
        };
    }
    return n;
}

/* ALPS — MOUNTAINS but much rougher, for sharp jagged peaks; the front
 * range is the spikiest. */
static int place_alps(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[3]     = { 0.40f, 0.55f, 0.70f };
    const float amps[3]   = { 0.25f, 0.28f, 0.30f };
    const float roughs[3] = { 0.70f, 0.72f, 0.75f };
    const int   pairs[3]  = { 3, 2, 0 };
    const GlyphDensity gd[3] = { GLYPH_LOW, GLYPH_MID, GLYPH_HIGH };
    for (int i = 0; i < 3 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * amps[i], .roughness = roughs[i],
            .pair_idx = pairs[i], .outline_idx = -1,
            .style = STYLE_FILL_BELOW, .glyph_density = gd[i],
        };
    }
    return n;
}

/* RIDGES — four thin lines, no fill, so all four stay visible at once. */
static int place_ridges(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[4]    = { 0.30f, 0.45f, 0.60f, 0.75f };
    const float amps[4]  = { 0.10f, 0.12f, 0.15f, 0.18f };
    const int   pairs[4] = { 3, 2, 1, 0 };
    for (int i = 0; i < 4 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * amps[i], .roughness = 0.65f,
            .pair_idx = pairs[i], .outline_idx = -1,
            .style = STYLE_THIN_LINE, .glyph_density = GLYPH_HIGH,
        };
    }
    return n;
}

/* TERRACES — four nearly-flat filled lines at rising heights, reading
 * as stepped farming terraces. */
static int place_terraces(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[4]    = { 0.30f, 0.45f, 0.60f, 0.75f };
    const int   pairs[4] = { 3, 2, 1, 0 };
    const GlyphDensity gd[4] = { GLYPH_LOW, GLYPH_LOW, GLYPH_MID, GLYPH_HIGH };
    for (int i = 0; i < 4 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * 0.08f, .roughness = 0.30f,
            .pair_idx = pairs[i], .outline_idx = -1,
            .style = STYLE_FILL_BELOW, .glyph_density = gd[i],
        };
    }
    return n;
}

/* ── tier 4 — many shallow lines ─────────────────────────────────── *
 * Five to seven low, gentle lines. The overall texture is the point,
 * not any one line. */

/* WAVES — six thin, shallow lines stacked up the screen, like layers of
 * an ocean seen edge-on. */
static int place_waves(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    for (int i = 0; i < 6 && n < max_n; i++) {
        float t = (float)(i + 1) / 7.0f;
        out[n++] = (LineSpec){
            .y_centre = fh * t, .amp_init = fh * 0.04f, .roughness = 0.55f,
            .pair_idx = i & 3, .outline_idx = -1,
            .style = STYLE_THIN_LINE, .glyph_density = GLYPH_HIGH,
        };
    }
    return n;
}

/* STRATA — seven barely-wavy lines, like flat rock layers seen from
 * the side. */
static int place_strata(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    for (int i = 0; i < 7 && n < max_n; i++) {
        float t = (float)(i + 1) / 8.0f;
        out[n++] = (LineSpec){
            .y_centre = fh * t, .amp_init = fh * 0.02f, .roughness = 0.35f,
            .pair_idx = i & 3, .outline_idx = -1,
            .style = STYLE_THIN_LINE, .glyph_density = GLYPH_HIGH,
        };
    }
    return n;
}

/* FOREST — five filled layers with bumpy tops that read as treetops,
 * stacked to look like a receding tree line. */
static int place_forest(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[5]    = { 0.40f, 0.50f, 0.60f, 0.70f, 0.80f };
    const int   pairs[5] = { 3, 2, 1, 2, 0 };
    const GlyphDensity gd[5] = { GLYPH_LOW, GLYPH_LOW, GLYPH_MID, GLYPH_MID, GLYPH_HIGH };
    for (int i = 0; i < 5 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * 0.10f, .roughness = 0.65f,
            .pair_idx = pairs[i], .outline_idx = -1,
            .style = STYLE_FILL_BELOW, .glyph_density = gd[i],
        };
    }
    return n;
}

/* RAPIDS — five thin lines bunched near the middle, low and rough, for
 * a fast-rippling river surface. */
static int place_rapids(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[5] = { 0.45f, 0.50f, 0.55f, 0.60f, 0.65f };
    for (int i = 0; i < 5 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * 0.02f, .roughness = 0.70f,
            .pair_idx = i & 3, .outline_idx = -1,
            .style = STYLE_THIN_LINE, .glyph_density = GLYPH_HIGH,
        };
    }
    return n;
}

/* SEDIMENT — five smooth filled layers, each painting over the last so
 * only a thin shelf of each shows, like a cutaway through rock layers. */
static int place_sediment(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[5]    = { 0.45f, 0.55f, 0.65f, 0.75f, 0.85f };
    const int   pairs[5] = { 3, 2, 1, 0, 0 };
    const GlyphDensity gd[5] = { GLYPH_LOW, GLYPH_LOW, GLYPH_MID, GLYPH_HIGH, GLYPH_HIGH };
    for (int i = 0; i < 5 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * 0.10f, .roughness = 0.40f,
            .pair_idx = pairs[i], .outline_idx = -1,
            .style = STYLE_FILL_BELOW, .glyph_density = gd[i],
        };
    }
    return n;
}

/* ── tier 5 — the exotic ones ────────────────────────────────────── *
 * Settings pushed to their limits: very rough, tilted, upside-down, or
 * fully random. */

/* ATOLLS — a rough shore set low, so only small peaks poke out of the
 * "water", each reading as a little island. */
static int place_atolls(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.78f, .amp_init = fh * 0.10f, .roughness = 0.80f,
        .pair_idx = 0, .outline_idx = 3,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* CRYSTAL — roughness near its max, so the wobbles barely shrink and the
 * line ends up sharply spiked, like a cluster of crystals. */
static int place_crystal(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.55f, .amp_init = fh * 0.30f, .roughness = 0.85f,
        .pair_idx = 2, .outline_idx = 3,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* SAWTOOTH — CLIFF turned up: a steeper tilt plus heavy roughness give
 * a dramatic saw-toothed slope. */
static int place_sawtooth(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre  = fh * 0.55f,
        .amp_init  = fh * 0.25f, .roughness = 0.70f,
        .left_dy   = -fh * 0.20f, .right_dy = +fh * 0.20f,
        .pair_idx = 1, .outline_idx = 3,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* NEBULA — six thin lines spread unevenly, tall but smooth, so they
 * drift like overlapping wisps of gas. */
static int place_nebula(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[6] = { 0.20f, 0.30f, 0.45f, 0.55f, 0.70f, 0.85f };
    for (int i = 0; i < 6 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * 0.12f, .roughness = 0.40f,
            .pair_idx = i & 3, .outline_idx = -1,
            .style = STYLE_THIN_LINE, .glyph_density = GLYPH_HIGH,
        };
    }
    return n;
}

/* STORM — three rough cloud layers filled down from the top plus one
 * ground line filled up from the bottom, mixing both fill styles. */
static int place_storm(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float cloud_ys[3]   = { 0.15f, 0.25f, 0.35f };
    const float cloud_amps[3] = { 0.15f, 0.18f, 0.20f };
    const int   cloud_p[3]    = { 2, 1, 0 };
    const GlyphDensity cloud_gd[3] = { GLYPH_LOW, GLYPH_MID, GLYPH_HIGH };
    for (int i = 0; i < 3 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * cloud_ys[i], .amp_init = fh * cloud_amps[i],
            .roughness = 0.70f,
            .pair_idx = cloud_p[i], .outline_idx = -1,
            .style = STYLE_FILL_ABOVE, .glyph_density = cloud_gd[i],
        };
    }
    if (n < max_n) {
        out[n++] = (LineSpec){
            .y_centre = fh * 0.85f, .amp_init = fh * 0.10f, .roughness = 0.55f,
            .pair_idx = 0, .outline_idx = -1,
            .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
        };
    }
    return n;
}

/* AURORA — four smooth thin lines clustered near the top, flowing like
 * ribbons of northern lights. */
static int place_aurora(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[4]    = { 0.20f, 0.25f, 0.30f, 0.35f };
    const float amps[4]  = { 0.15f, 0.12f, 0.10f, 0.08f };
    const int   pairs[4] = { 3, 2, 1, 0 };
    for (int i = 0; i < 4 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * amps[i], .roughness = 0.35f,
            .pair_idx = pairs[i], .outline_idx = -1,
            .style = STYLE_THIN_LINE, .glyph_density = GLYPH_HIGH,
        };
    }
    return n;
}

/* CEILING — a rough line filled from the top down, so the spikes hang
 * downward like stalactites from a cave roof. */
static int place_ceiling(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.30f, .amp_init = fh * 0.15f, .roughness = 0.80f,
        .pair_idx = 1, .outline_idx = 3,
        .style = STYLE_FILL_ABOVE, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* ISLANDS — ATOLLS but smoother, giving fewer, larger islands with
 * softer shores. */
static int place_islands(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.75f, .amp_init = fh * 0.10f, .roughness = 0.65f,
        .pair_idx = 0, .outline_idx = 2,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* PETRA — three sandstone layers, each tilted a different way so they
 * don't stack flat, evoking the carved rock of Petra. Each row of the
 * table below is one layer. */
static int place_petra(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const struct {
        float        y_frac, left_tilt, right_tilt;
        int          pair;
        GlyphDensity density;
    } layers[3] = {
        { 0.45f, -0.10f, +0.05f, 3, GLYPH_LOW  },
        { 0.60f, +0.05f, -0.10f, 2, GLYPH_MID  },
        { 0.75f, -0.05f, +0.10f, 1, GLYPH_HIGH },
    };
    for (int i = 0; i < 3 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre      = fh * layers[i].y_frac,
            .amp_init      = fh * 0.10f, .roughness = 0.50f,
            .left_dy       = fh * layers[i].left_tilt,
            .right_dy      = fh * layers[i].right_tilt,
            .pair_idx      = layers[i].pair, .outline_idx = -1,
            .style         = STYLE_FILL_BELOW,
            .glyph_density = layers[i].density,
        };
    }
    return n;
}

/* A random number from 0 up to (not including) 1, in coarse steps.
 * Only CHAOS uses it, where fine precision doesn't matter. */
static inline float random_unit_uniform(void)
{
    return (float)(rand() % 1000) / 1000.0f;
}

/* A random number somewhere in [lo, lo + range). */
static inline float random_in_range(float lo, float range)
{
    return lo + random_unit_uniform() * range;
}

/* Pick one of the three fill styles at random. */
static RenderStyle random_render_style(void)
{
    switch (rand() % CHAOS_STYLE_COUNT) {
    case 0:  return STYLE_FILL_BELOW;
    case 1:  return STYLE_FILL_ABOVE;
    default: return STYLE_THIN_LINE;
    }
}

/* Maybe give a line an outline colour (about 1 in 3), or -1 for none,
 * so outlines stay an accent rather than the main thing. */
static int random_outline_band(void)
{
    return (rand() % CHAOS_OUTLINE_DENOM == 0) ? (rand() % N_BANDS) : -1;
}

/* CHAOS — eight lines, every setting rolled at random, so no two resets
 * look the same. */
static int place_chaos(LineSpec out[], int max_n, int h)
{
    int target = CHAOS_LINES_TARGET;
    if (target > max_n) target = max_n;
    int n = 0;
    float fh = (float)h;
    for (int i = 0; i < target; i++) {
        out[n++] = (LineSpec){
            .y_centre      = fh * random_in_range(CHAOS_Y_LO,     CHAOS_Y_RANGE),
            .amp_init      = fh * random_in_range(CHAOS_AMP_LO,   CHAOS_AMP_RANGE),
            .roughness     =      random_in_range(CHAOS_ROUGH_LO, CHAOS_ROUGH_RANGE),
            .pair_idx      = rand() % N_BANDS,
            .outline_idx   = random_outline_band(),
            .style         = random_render_style(),
            .glyph_density = GLYPH_HIGH,
        };
    }
    return n;
}

/* Look up a preset and run its placer; returns how many lines it made. */
static int pattern_setup(LineSpec out[], int max_n, Pattern p, int h)
{
    switch (p) {

    /* Tier 1 — single-line silhouettes */
    case PATTERN_COASTLINE: return place_coastline(out, max_n, h);
    case PATTERN_SKYLINE:   return place_skyline  (out, max_n, h);
    case PATTERN_HORIZON:   return place_horizon  (out, max_n, h);
    case PATTERN_DUNES:     return place_dunes    (out, max_n, h);
    case PATTERN_CITY:      return place_city     (out, max_n, h);

    /* Tier 2 — paired lines */
    case PATTERN_VALLEY:    return place_valley   (out, max_n, h);
    case PATTERN_CAVE:      return place_cave     (out, max_n, h);
    case PATTERN_CLIFF:     return place_cliff    (out, max_n, h);
    case PATTERN_PLATEAU:   return place_plateau  (out, max_n, h);
    case PATTERN_REEF:      return place_reef     (out, max_n, h);

    /* Tier 3 — small layered stacks */
    case PATTERN_MOUNTAINS: return place_mountains(out, max_n, h);
    case PATTERN_HILLS:     return place_hills    (out, max_n, h);
    case PATTERN_ALPS:      return place_alps     (out, max_n, h);
    case PATTERN_RIDGES:    return place_ridges   (out, max_n, h);
    case PATTERN_TERRACES:  return place_terraces (out, max_n, h);

    /* Tier 4 — multi-layer stacks */
    case PATTERN_WAVES:     return place_waves    (out, max_n, h);
    case PATTERN_STRATA:    return place_strata   (out, max_n, h);
    case PATTERN_FOREST:    return place_forest   (out, max_n, h);
    case PATTERN_RAPIDS:    return place_rapids   (out, max_n, h);
    case PATTERN_SEDIMENT:  return place_sediment (out, max_n, h);

    /* Tier 5 — exotic / max-complex */
    case PATTERN_ATOLLS:    return place_atolls   (out, max_n, h);
    case PATTERN_CRYSTAL:   return place_crystal  (out, max_n, h);
    case PATTERN_SAWTOOTH:  return place_sawtooth (out, max_n, h);
    case PATTERN_NEBULA:    return place_nebula   (out, max_n, h);
    case PATTERN_STORM:     return place_storm    (out, max_n, h);
    case PATTERN_AURORA:    return place_aurora   (out, max_n, h);
    case PATTERN_CEILING:   return place_ceiling  (out, max_n, h);
    case PATTERN_ISLANDS:   return place_islands  (out, max_n, h);
    case PATTERN_PETRA:     return place_petra    (out, max_n, h);
    case PATTERN_CHAOS:     return place_chaos    (out, max_n, h);

    default:                return 0;
    }
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * The scene splits into four small pieces, each handling one thing, and
 * Scene bundles them together. Splitting them this way means a function's
 * arguments tell you what it can touch: a Grid argument can't change the
 * lines, and so on.
 */

/*
 * Grid — the size of the drawing area, in character cells. Sits first
 * because everything else needs to know the dimensions. The width and
 * height never exceed MAP_W_MAX / MAP_H_MAX; app_pick_map_size() makes
 * sure of that, so the rest of the code can assume it.
 */
typedef struct {
    int w, h;     /* drawing area width and height, in cells */
} Grid;

static inline bool grid_in_bounds(const Grid *g, int x, int y)
{
    return x >= 0 && x < g->w && y >= 0 && y < g->h;
}

/*
 * FractalField — all the line data the renderer draws from.
 *
 * For each line we keep two fully-built shapes (keyframe_a and
 * keyframe_b) and blend between them every frame into live[]. The
 * renderer only ever reads live[]. We build the shapes only on reset
 * and once per morph (roughly every 10 seconds), so the per-frame cost
 * is just the cheap blend; building a shape from scratch is the
 * expensive part. This two-shapes-and-blend trick is the standard way
 * to animate between key poses.
 *
 *   spec[]        — each line's recipe (its LineSpec).
 *   keyframe_a[]  — the shape the morph is coming from.
 *   keyframe_b[]  — the shape the morph is heading toward. When it
 *                   arrives, b becomes the new a and a fresh b is built.
 *   live[]        — the blended shape for this frame; the only thing
 *                   drawn.
 */
typedef struct {
    int      n_lines;
    LineSpec spec       [MAX_LINES];
    float    keyframe_a [MAX_LINES][LINE_POINTS];
    float    keyframe_b [MAX_LINES][LINE_POINTS];
    float    live       [MAX_LINES][LINE_POINTS];
} FractalField;

/*
 * SimState — how far along the current morph we are. Kept apart from the
 * user's controls so it's clear this is the one thing the sim changes on
 * its own each tick. When it passes 1.0 the morph is done: the target
 * shape becomes the new start, a fresh target is built, and this resets
 * to 0.
 */
typedef struct {
    float morph_t;     /* 0 = at the start shape, 1 = at the target shape */
} SimState;

/*
 * Controls — everything the user sets from the keyboard. Only the key
 * handler writes these; the sim and the renderer only read them. That
 * keeps the flow one-way and easy to follow.
 */
typedef struct {
    /* When true the morph freezes, but drawing keeps going so the HUD
     * still responds to keys. */
    bool    paused;

    int     current_theme;       /* which theme is active     */
    int     current_glyph_set;   /* which glyph set is active */
    Pattern current_pattern;     /* which preset is active    */

    /* How fast lines morph, as a fraction of the trip per second. '+'
     * doubles it, '-' halves it; the default finishes in 10 seconds. */
    float   morph_rate;
} Controls;

/*
 * Scene — the whole simulation in one place. Reading it top to bottom
 * is the quickest way to see how the program fits together: the drawing
 * area, then the lines, then the morph progress, then the user's
 * settings. Each piece only needs the ones above it.
 */
typedef struct {
    Grid          grid;       /* drawing-area size           */
    FractalField  fractal;    /* the lines and their shapes  */
    SimState      sim;        /* morph progress              */
    Controls      ctrl;       /* the user's settings         */
} Scene;

/* ── building the line shapes ─────────────────────────────────────── */

/*
 * Builds one fresh shape for a line. The two ends start near the line's
 * centre height (shifted by its tilt) but get a small random wobble too,
 * so each new shape is a little different and the morphs never repeat.
 */
static void generate_keyframe(const LineSpec *s, float *out)
{
    float endpoint_jitter = s->amp_init * JITTER_HALFRANGE;
    float left            = s->y_centre + s->left_dy  + jitter(endpoint_jitter);
    float right           = s->y_centre + s->right_dy + jitter(endpoint_jitter);
    md_generate(out, LINE_POINTS, left, right, s->amp_init, s->roughness);
}

static void generate_all_keyframes_a(FractalField *ff)
{
    for (int i = 0; i < ff->n_lines; i++)
        generate_keyframe(&ff->spec[i], ff->keyframe_a[i]);
}

static void generate_all_keyframes_b(FractalField *ff)
{
    for (int i = 0; i < ff->n_lines; i++)
        generate_keyframe(&ff->spec[i], ff->keyframe_b[i]);
}

/*
 * Called when a morph finishes: the shape we just reached becomes the
 * new starting shape, and a fresh target is built. This is what lets the
 * lines keep evolving without ever jumping.
 */
static void rotate_keyframes(FractalField *ff)
{
    memcpy(ff->keyframe_a, ff->keyframe_b, sizeof(ff->keyframe_a));
    generate_all_keyframes_b(ff);
}

static void copy_keyframes_a_to_live(FractalField *ff)
{
    for (int i = 0; i < ff->n_lines; i++)
        memcpy(ff->live[i], ff->keyframe_a[i], sizeof(ff->live[i]));
}

static void interpolate_live_lines(FractalField *ff, float morph_t)
{
    for (int i = 0; i < ff->n_lines; i++)
        line_lerp(ff->keyframe_a[i], ff->keyframe_b[i],
                  ff->live[i], LINE_POINTS, morph_t);
}

/* ── morph progress ──────────────────────────────────────────────── *
 * Three tiny helpers, named so the tick loop reads like plain steps. */

static void advance_morph(SimState *sim, float morph_rate, float dt)
{
    sim->morph_t += morph_rate * dt;
}

static bool morph_completed(const SimState *sim)
{
    return sim->morph_t >= 1.0f;
}

static void reset_morph(SimState *sim)
{
    sim->morph_t = 0.0f;
}

/* ── setup and reset ─────────────────────────────────────────────── */

static void apply_grid_dimensions(Grid *g, int w, int h)
{
    g->w = w;
    g->h = h;
}

static void install_pattern(FractalField *ff, Pattern p, const Grid *g)
{
    ff->n_lines = pattern_setup(ff->spec, MAX_LINES, p, g->h);
}

static void scene_reset(Scene *s, int w, int h)
{
    apply_grid_dimensions     (&s->grid, w, h);
    reset_morph               (&s->sim);
    install_pattern           (&s->fractal, s->ctrl.current_pattern, &s->grid);
    generate_all_keyframes_a  (&s->fractal);
    generate_all_keyframes_b  (&s->fractal);
    copy_keyframes_a_to_live  (&s->fractal);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->ctrl.paused             = false;
    s->ctrl.current_theme      = 0;
    s->ctrl.current_glyph_set  = 2;        /* MEDIUM */
    s->ctrl.current_pattern    = PATTERN_COASTLINE;
    s->ctrl.morph_rate         = MORPH_RATE_DEF;
    scene_reset(s, w, h);
}

/*
 * One simulation step: unless paused, push the morph forward a bit, swap
 * in a new target if it finished, then blend every line's two shapes
 * into the live shape that gets drawn.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;

    advance_morph             (&s->sim, s->ctrl.morph_rate, dt);
    if (morph_completed(&s->sim)) {
        rotate_keyframes      (&s->fractal);
        reset_morph           (&s->sim);
    }
    interpolate_live_lines    (&s->fractal, s->sim.morph_t);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the terminal's current size in characters. We only track
 * this much because ncurses keeps the rest of its state itself; we just
 * need the size to centre the scene and place the HUD. Re-read on resize.
 */
typedef struct {
    int cols;   /* terminal width  in characters */
    int rows;   /* terminal height in characters */
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

/* Fills every column from the line down to the floor. */
static void draw_fill_below_line(const float *line, int n_points,
                                 int gx0, int gy0, int map_w, int map_h,
                                 int cols, int rows, int pair, char glyph)
{
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < map_w; x++) {
        float ly = line_sample(line, n_points, map_w, x);
        int y_start = (int)ly;
        if (y_start < 0) y_start = 0;
        if (y_start >= map_h) continue;
        for (int y = y_start; y < map_h; y++) {
            int sx = gx0 + x;
            int sy = gy0 + y;
            if (sx < 0 || sx >= cols) continue;
            if (sy < 0 || sy >= rows) continue;
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
        }
    }
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Fills every column from the ceiling down to the line. */
static void draw_fill_above_line(const float *line, int n_points,
                                  int gx0, int gy0, int map_w, int map_h,
                                  int cols, int rows, int pair, char glyph)
{
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < map_w; x++) {
        float ly = line_sample(line, n_points, map_w, x);
        int y_end = (int)ly;
        if (y_end < 0) continue;
        if (y_end >= map_h) y_end = map_h - 1;
        for (int y = 0; y <= y_end; y++) {
            int sx = gx0 + x;
            int sy = gy0 + y;
            if (sx < 0 || sx >= cols) continue;
            if (sy < 0 || sy >= rows) continue;
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
        }
    }
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Draws just the line, one character per column. Also used to add an
 * outline on top of a fill. */
static void draw_line_only(const float *line, int n_points,
                           int gx0, int gy0, int map_w, int map_h,
                           int cols, int rows, int pair, char glyph)
{
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < map_w; x++) {
        float ly = line_sample(line, n_points, map_w, x);
        int y = (int)ly;
        if (y < 0 || y >= map_h) continue;
        int sx = gx0 + x;
        int sy = gy0 + y;
        if (sx < 0 || sx >= cols) continue;
        if (sy < 0 || sy >= rows) continue;
        mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    }
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Picks the thin, middle, or thick character for a line's chosen weight. */
static char pick_glyph(GlyphDensity density, const GlyphSet *gs)
{
    switch (density) {
    case GLYPH_LOW:  return gs->low;
    case GLYPH_MID:  return gs->mid;
    case GLYPH_HIGH: return gs->high;
    }
    return gs->high;
}

/* If the line wants an outline, draw it once more on top in the outline
 * colour. Shared by both fill styles. */
static void maybe_paint_outline(const LineSpec *spec, const float *line, int n_points,
                                 int gx0, int gy0, int map_w, int map_h,
                                 int cols, int rows, const GlyphSet *gs)
{
    if (spec->outline_idx < 0) return;
    draw_line_only(line, n_points, gx0, gy0, map_w, map_h, cols, rows,
                    PAIR_BAND_BASE + spec->outline_idx, gs->high);
}

/*
 * Draws one line: pick its character and colour, paint it in the chosen
 * style, and add an outline if it has one. Every visual choice comes
 * from the line's own recipe, so this works for any preset.
 */
static void render_line(const LineSpec *spec, const float *line, int n_points,
                         int gx0, int gy0, int map_w, int map_h,
                         int cols, int rows, const GlyphSet *gs)
{
    char glyph = pick_glyph(spec->glyph_density, gs);
    int  pair  = PAIR_BAND_BASE + spec->pair_idx;

    switch (spec->style) {
    case STYLE_FILL_BELOW:
        draw_fill_below_line(line, n_points, gx0, gy0, map_w, map_h,
                              cols, rows, pair, glyph);
        maybe_paint_outline (spec, line, n_points, gx0, gy0, map_w, map_h,
                              cols, rows, gs);
        break;
    case STYLE_FILL_ABOVE:
        draw_fill_above_line(line, n_points, gx0, gy0, map_w, map_h,
                              cols, rows, pair, glyph);
        maybe_paint_outline (spec, line, n_points, gx0, gy0, map_w, map_h,
                              cols, rows, gs);
        break;
    case STYLE_THIN_LINE:
        draw_line_only      (line, n_points, gx0, gy0, map_w, map_h,
                              cols, rows, pair, glyph);
        break;
    }
}

/* Finds the top-left corner to start drawing at, centring the scene and
 * leaving room for the HUD rows top and bottom. */
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

static const GlyphSet *active_glyph_set(const Controls *c)
{
    int idx = c->current_glyph_set;
    if (idx < 0 || idx >= N_GLYPH_SETS) idx = 0;
    return &glyph_sets[idx];
}

/*
 * Draws all the lines in list order. Later lines paint over earlier
 * ones, so listing them back-to-front makes front layers cover the ones
 * behind — that's how the stacked scenes get their sense of depth.
 */
static void scene_draw(const Scene *s, int cols, int rows)
{
    int gx0, gy0;
    compute_centred_origin(&s->grid, cols, rows, &gx0, &gy0);
    const GlyphSet *gs = active_glyph_set(&s->ctrl);

    for (int i = 0; i < s->fractal.n_lines; i++) {
        render_line(&s->fractal.spec[i], s->fractal.live[i], LINE_POINTS,
                     gx0, gy0, s->grid.w, s->grid.h, cols, rows, gs);
    }
}

/* ── the HUD ──────────────────────────────────────────────────────── *
 * The top two rows show what's going on (state, settings, colours,
 * glyphs); the bottom row lists the keys. */

static void draw_hud_state_bar(const Screen *sc, const Scene *s,
                                double fps, int sim_fps)
{
    const Controls *c = &s->ctrl;
    const char *state_str = c->paused ? "PAUSED   "
                                      : pattern_name(c->current_pattern);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  morph:%4.2f  rate:%4.2f ",
             fps, sim_fps, state_str,
             (int)c->current_pattern + 1, N_PATTERNS,
             (double)s->sim.morph_t, (double)c->morph_rate);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void draw_hud_title(void)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " MIDPOINT-DISPLACEMENT FRACTAL ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* ── row 1 fields ─────────────────────────────────────────────────── *
 * These paint row 1 left to right. Each draws its bit and returns where
 * the next one should start. */

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
        int pair = PAIR_BAND_BASE + i;
        attron (COLOR_PAIR(pair) | A_BOLD);
        mvaddch(row, x, '#');
        attroff(COLOR_PAIR(pair) | A_BOLD);
        x++;
    }
    return x;
}

/* Last field: line count, points per line, and scene size. */
static void draw_status_sim_counts(int row, int x, const Scene *s)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  lines:%d  pts:%d  map:%dx%d ",
             s->fractal.n_lines, LINE_POINTS, s->grid.w, s->grid.h);
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

/* Right side of row 1: shows the current set's three characters so you
 * can preview a set before switching with g/G. */
static void draw_hud_glyph_indicator(const Scene *s, const Screen *sc)
{
    const GlyphSet *gs = active_glyph_set(&s->ctrl);
    char buf[32];
    snprintf(buf, sizeof buf, " glyph:%-7s [%c%c%c] ",
             gs->name, gs->low, gs->mid, gs->high);
    int gx = sc->cols - (int)strlen(buf);
    if (gx < 0) gx = 0;
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, gx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Bottom row: the list of keys. */
static void draw_bottom_hint(const Screen *sc)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  g/G:glyph  t/T:theme  r:reset  spc:pause  +/-:morph-rate  ]/[:Hz  q:quit ");
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
    draw_bottom_hint        (sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

/*
 * App — the whole running program. There's one of these, g_app, kept as
 * a global so the signal handlers can reach it. It holds the scene, the
 * terminal size, a few settings, and the flags the handlers set.
 *
 * The running and need_resize flags are touched by signal handlers, so
 * they're marked volatile sig_atomic_t: that makes a handler's write
 * land safely and stops the main loop from caching a stale value. The
 * handlers themselves do nothing but flip a flag; the real work happens
 * back in the main loop, since it isn't safe to call things like ncurses
 * from inside a handler. (Stevens & Rago, "Advanced Programming in the
 * UNIX Environment", ch. 10, covers why.)
 */
typedef struct {
    Scene                 scene;     /* the simulation              */
    Screen                screen;    /* current terminal size       */

    int                   sim_fps;   /* tick rate, set with [ and ] */
    int                   map_w;     /* scene width  (capped)       */
    int                   map_h;     /* scene height (capped)       */

    volatile sig_atomic_t running;       /* goes to 0 to quit             */
    volatile sig_atomic_t need_resize;   /* set to 1 when the window resizes */
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
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* ── keyboard ─────────────────────────────────────────────────────── *
 * One named action per key; app_handle_key just routes to them. */

/* '+' doubles the morph speed, '-' halves it, kept within limits. */
static void bump_morph_rate(Controls *c, int dir)
{
    if (dir > 0) c->morph_rate *= 2.0f;
    else         c->morph_rate *= 0.5f;
    if (c->morph_rate > MORPH_RATE_MAX) c->morph_rate = MORPH_RATE_MAX;
    if (c->morph_rate < MORPH_RATE_MIN) c->morph_rate = MORPH_RATE_MIN;
}

/* '[' and ']' step the tick rate up or down, kept within limits. */
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

/* Move to the next or previous preset. We rebuild the whole scene because
 * each preset has its own set of lines; reusing the old ones would flash
 * a wrong first frame. */
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
    case ' ':           c->paused = !c->paused;                              break;
    case 'r': case 'R': scene_reset(&app->scene, app->map_w, app->map_h);    break;
    case '=': case '+': bump_morph_rate (c,   +1);                           break;
    case '-':           bump_morph_rate (c,   -1);                           break;
    case ']':           bump_sim_fps    (app, +SIM_FPS_STEP);                break;
    case '[':           bump_sim_fps    (app, -SIM_FPS_STEP);                break;
    case 't':           cycle_theme     (c,   +1);                           break;
    case 'T':           cycle_theme     (c,   -1);                           break;
    case 'g':           cycle_glyph_set (c,   +1);                           break;
    case 'G':           cycle_glyph_set (c,   -1);                           break;
    case 'n': case 'N': cycle_pattern   (app, +1);                           break;
    case 'p': case 'P': cycle_pattern   (app, -1);                           break;
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

/* How long since the last frame, with an upper limit so a long pause
 * (like the laptop sleeping) doesn't make the sim try to catch up forever. */
static int64_t advance_frame_clock(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > DT_MAX_NS) dt = DT_MAX_NS;
    return dt;
}

/* Run as many fixed-size sim steps as the elapsed time has earned, so
 * the sim advances at a steady rate no matter the frame rate.
 * (Glenn Fiedler, "Fix Your Timestep".) */
static void simulate_pending_ticks(App *app, int64_t *sim_accum,
                                    int64_t tick_ns, float dt_sec)
{
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* Every half second or so, turn the frames-since-last-check into an fps
 * number for the HUD; otherwise leave it alone. */
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

/* Sleep off the rest of the frame's time budget so we don't redraw
 * faster than the target rate. */
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
