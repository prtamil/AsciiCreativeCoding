/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * perlin_landscape.c — a scrolling cartoon landscape made of three terrain
 * layers (far mountains, rolling hills, foreground ground). The layers slide
 * past at different speeds, which is the old trick for faking depth: nearer
 * things move faster than far ones. Each layer's silhouette is procedural
 * terrain from Perlin noise, recomputed every frame — nothing is stored.
 *
 * Keys: q quit  p pause  r reset  <-/-> speed/dir  +/- zoom  t theme
 *
 * References (things the code can't tell you):
 *   Perlin, "An Image Synthesizer" (1985) and "Improving Noise" (2002) —
 *     the gradient noise and the smooth-fade curve used below.
 *   Ebert/Musgrave et al., "Texturing & Modeling" (2003) — the canonical
 *     treatment of stacking noise octaves into fBm terrain (fbm()).
 *   Imhof, "Cartographic Relief Presentation" (1982) — colouring height
 *     bands, and using haze to make distant ranges read as far away.
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* How the file is laid out: all the state lives in three named types, defined
 * in §1 — PerlinNoise (the noise generator), Landscape (the world: that noise
 * plus the camera and its knobs), and Scene (the Landscape plus what's needed
 * to draw it: the colour theme and the terminal size). main() owns one Scene
 * and passes it (or just its Landscape) to each function. The only globals are
 * the two signal flags, because the signal handler can't be handed anything.
 *
 * Each frame main() does the same four things in order: read keys, advance the
 * camera by one step (unless paused), draw, then sleep to hold ~30 fps. The
 * camera position only ever moves in that one spot. */

/* ── §1  config: tunables, theme data, core types, signal flags ── */

#define RENDER_NS  (1000000000LL / 30)   /* one frame's worth of time, ~30 fps */
#define HUD_TOP     2    /* top two rows reserved for the HUD text */
#define HUD_BOTTOM  1    /* bottom row reserved for the key legend */

/* How fast each layer scrolls, as a fraction of the camera's speed. The far
 * layer barely moves; the near layer keeps pace. That speed difference is what
 * sells the depth — near things slide by faster than distant ones. */
#define SCROLL_FAR    0.12f
#define SCROLL_MID    0.38f
#define SCROLL_NEAR   1.00f

/* How stretched-out each layer's hills are. Smaller = broader, gentler humps;
 * the far range is the smoothest, the near ground the most jagged. */
#define BASE_FREQ_FAR   0.009f
#define BASE_FREQ_MID   0.018f
#define BASE_FREQ_NEAR  0.040f

/* The shape of the terrain. fbm() builds it by adding up several layers of
 * noise: each next layer is twice as wiggly (LACUNARITY) but half as tall
 * (GAIN), so you get big shapes with finer detail riding on top. More octaves
 * means finer detail, with rapidly diminishing returns. */
#define FBM_OCTAVES     5
#define FBM_LACUNARITY  2.0f
#define FBM_GAIN        0.5f

/* Where each layer sits and how tall it gets, measured as fractions of the
 * drawing height from the top. BASE is the layer's resting line; ridges rise
 * up to AMP above it. */
#define BASE_FAR   0.60f
#define AMP_FAR    0.28f
#define BASE_MID   0.70f
#define AMP_MID    0.22f
#define BASE_NEAR  0.80f
#define AMP_NEAR   0.16f

/* Each layer reads a different slice of the noise so their outlines don't end
 * up looking like copies of each other. */
#define NOISE_YOFF_FAR   0.0f
#define NOISE_YOFF_MID   8.3f
#define NOISE_YOFF_NEAR  16.7f

/* Sky shading, measured down from the top toward the far ridge. Stars only
 * appear in the top stretch, and the sky switches from bright to dim halfway. */
#define SKY_STAR_MAX     0.55f   /* below this height fraction: no stars */
#define SKY_SPLIT        0.50f   /* above it bright sky, below it dim    */

/* How many rows of bright foreground ground before it darkens into the base. */
#define NEAR_FILL_ROWS   3

#define SPEED_MAX  4.f   /* fastest the camera can scroll, either direction */

#define PERM 256   /* size of the noise lookup table (must be a power of two) */

/* One colour slot per piece of the scene, listed from the back of the picture
 * (sky) to the front (near ground) — the same order they get painted in. The
 * first ten line up one-for-one with a theme's ten colours below, so a theme
 * is just "here are my ten colours, back to front". Slot 0 is off-limits in
 * ncurses (it's the terminal default), so these start at 1. */
enum {
    CP_SKY_HI = 1,   /* top of the sky      */
    CP_SKY_LO,       /* sky near the horizon */
    CP_STAR,          /* stars               */
    CP_FAR_E,         /* far ridge line      */
    CP_FAR_F,         /* far slope fill      */
    CP_MID_E,         /* mid ridge line      */
    CP_MID_F,         /* mid slope fill      */
    CP_NEAR_E,        /* near ground edge    */
    CP_NEAR_F,        /* near ground fill    */
    CP_NEAR_B,        /* near ground base    */
    CP_HUD,           /* HUD text            */
    CP_HINT           /* bottom key legend   */
};

#define N_THEMES 10

/* One named colour scheme (the 't' key cycles through them). A theme is just a
 * list of ten colours — one for each scene slot above, in back-to-front order —
 * plus a HUD colour. The background is always the terminal default so the sky
 * shows through; the parallax look needs one flat, uniform sky.
 *
 * The depth trick lives in how you pick the colours: a good theme makes the
 * far slots pale and low-contrast (like a hazy mountain you can barely make
 * out) and the near ones bold and vivid. That's the same thing painters do to
 * fake distance. c8[] is the same ten slots again in the basic 8 colours, used
 * when the terminal can't do 256. (Idea: Imhof, "Cartographic Relief
 * Presentation", 1982. Palette notes: documentation/COLOR.md.) */
typedef struct {
    const char *name;   /* label shown in the HUD                  */
    int c[10];          /* the ten slot colours (256-colour mode)  */
    int c8[10];         /* same ten slots, basic 8-colour fallback */
    int hud;            /* HUD text colour, 256-colour mode        */
    int hud8;           /* HUD text colour, 8-colour fallback      */
} Theme;

static const Theme k_themes[N_THEMES] = {
    { "Classic",
      { 17,  18, 250,  24,  17,  22,  22,  34,  28,  22 },
      { COLOR_BLUE, COLOR_CYAN,  COLOR_WHITE, COLOR_BLUE,  COLOR_BLUE,
        COLOR_GREEN,COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN },
      226, COLOR_YELLOW },

    { "Matrix",
      { 16,  22,  46,  28,  22,  34,  28,  46,  34,  22 },
      { COLOR_BLACK,COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
        COLOR_GREEN, COLOR_GREEN,COLOR_GREEN, COLOR_GREEN, COLOR_GREEN },
       46, COLOR_GREEN },

    { "Nova",
      { 17,  18, 231,  39,  17,  21,  17,  51,  39,  21 },
      { COLOR_BLUE, COLOR_BLUE,  COLOR_WHITE, COLOR_CYAN,  COLOR_BLUE,
        COLOR_CYAN,  COLOR_BLUE, COLOR_CYAN,  COLOR_CYAN,  COLOR_BLUE },
       51, COLOR_CYAN },

    { "Poison",
      { 16,  22, 190, 100,  22, 148, 100, 190, 148, 100 },
      { COLOR_BLACK,COLOR_GREEN, COLOR_YELLOW,COLOR_GREEN, COLOR_GREEN,
        COLOR_YELLOW,COLOR_GREEN,COLOR_YELLOW,COLOR_YELLOW,COLOR_GREEN },
      154, COLOR_YELLOW },

    { "Ocean",
      { 17,  24, 231,  38,  18,  30,  24,  45,  38,  30 },
      { COLOR_BLUE, COLOR_CYAN,  COLOR_WHITE, COLOR_CYAN,  COLOR_BLUE,
        COLOR_CYAN,  COLOR_CYAN, COLOR_CYAN,  COLOR_CYAN,  COLOR_CYAN },
       39, COLOR_CYAN },

    { "Fire",
      { 52,  88, 226, 196,  88, 208, 196, 226, 214, 208 },
      { COLOR_RED,  COLOR_RED,   COLOR_YELLOW,COLOR_RED,   COLOR_RED,
        COLOR_YELLOW,COLOR_RED,  COLOR_YELLOW,COLOR_YELLOW,COLOR_RED },
      208, COLOR_YELLOW },

    { "Gold",
      { 17,  52, 231,  94,  52, 136,  94, 220, 136,  94 },
      { COLOR_BLUE, COLOR_RED,   COLOR_WHITE, COLOR_YELLOW,COLOR_RED,
        COLOR_YELLOW,COLOR_YELLOW,COLOR_YELLOW,COLOR_YELLOW,COLOR_YELLOW },
      226, COLOR_YELLOW },

    { "Ice",
      { 16,  17, 231,  30,  17,  23,  17, 159,  30,  23 },
      { COLOR_BLACK,COLOR_BLUE,  COLOR_WHITE, COLOR_CYAN,  COLOR_BLUE,
        COLOR_BLUE,  COLOR_BLUE, COLOR_CYAN,  COLOR_CYAN,  COLOR_BLUE },
      123, COLOR_CYAN },

    { "Nebula",
      { 16,  54, 183,  93,  54,  55,  54, 141,  93,  55 },
      { COLOR_BLACK,COLOR_MAGENTA,COLOR_WHITE,COLOR_MAGENTA,COLOR_MAGENTA,
        COLOR_MAGENTA,COLOR_MAGENTA,COLOR_CYAN,COLOR_MAGENTA,COLOR_MAGENTA },
       87, COLOR_CYAN },

    { "Lava",
      { 52,  88, 226, 124,  52, 196, 124, 214, 196, 124 },
      { COLOR_RED,  COLOR_RED,   COLOR_YELLOW,COLOR_RED,   COLOR_RED,
        COLOR_RED,   COLOR_RED,  COLOR_YELLOW,COLOR_RED,   COLOR_RED },
      214, COLOR_YELLOW },
};

/* ── data types ── */

/* The Perlin noise generator: it turns any point (x,y) into a smoothly-varying
 * number, so the landscape looks natural instead of random static. The idea:
 * lay down a grid, give every grid corner a random "which way is downhill"
 * arrow, and for any point blend the four arrows around it. Points right on a
 * corner come out 0, and the value drifts gently between corners — no jagged
 * jumps, no visible grid lines. fbm() then stacks several of these to build
 * terrain. (Perlin, 1985 & 2002 — see the file-header references.)
 *
 * The three arrays below are kept in lockstep — shuffling or copying one means
 * doing the same to all three — so that one table lookup picks both a slot and
 * its matching arrow.
 *   perm  A shuffled list of 0..255, used as a hash to turn grid coordinates
 *         into a slot number. The whole list is stored twice, back to back.
 *         That doubling is a deliberate trick: when perlin() looks up a corner
 *         and its neighbour, the index can run off the end of the first copy
 *         and just land in the second one, which holds the same values — so no
 *         wrap-around math or bounds check is needed in the hot path.
 *   gx,gy The actual arrow for each slot: a random direction, length 1.
 *         (Perlin's later version uses a fixed handful of directions to avoid
 *         clumping; this file keeps it simple with a fresh random angle each.) */
typedef struct {
    unsigned char perm[PERM * 2];   /* shuffled 0..255 hash, stored twice so
                                       lookups never need to wrap by hand */
    float         gx[PERM * 2];     /* arrow x-component for each slot */
    float         gy[PERM * 2];     /* arrow y-component for each slot */
} PerlinNoise;

/* The world: a camera gliding over procedural terrain, plus the few knobs the
 * user can turn. There is no stored heightmap — the terrain is recomputed from
 * the noise every frame, so "moving the world" just means nudging one number,
 * `scroll`. The depth illusion falls out of each layer reading the noise at a
 * different fraction of that one number.
 *   noise   the noise generator above; stays fixed until 'r' makes a new one.
 *   scroll  how far the camera has travelled, in columns. Just keeps counting
 *           up; huge values are fine because the noise repeats.
 *   speed   columns moved per frame, from -4 to +4. Sign is the direction,
 *           size is the pace. Arrow keys nudge it by 0.1.
 *   zoom    stretches all the hills horizontally, from 0.125x to 8x. Below 1x
 *           the hills get wider and gentler. +/- multiply it.
 *   paused  stops the camera moving; drawing keeps going.
 * The colour theme isn't kept here on purpose — it's a how-we-draw-it choice,
 * not a fact about the land, so it lives up in the Scene. */
typedef struct {
    PerlinNoise noise;     /* terrain source (sampled fresh, never stored) */
    float       scroll;    /* camera position in columns (only counts up)  */
    float       speed;     /* columns per frame, -SPEED_MAX..SPEED_MAX      */
    float       zoom;      /* horizontal stretch, 0.125..8                  */
    bool        paused;    /* stop the camera (drawing still runs)          */
} Landscape;

/* Everything in one bundle: the world (Landscape) plus the two things you need
 * to draw it — which colour theme is active and how big the terminal is. The
 * theme and size live here, not in the Landscape, because a keypress changes
 * the theme and a window resize changes the size; neither is part of the land. */
typedef struct {
    Landscape landscape;   /* the world being drawn        */
    int       theme;       /* which theme, index into k_themes */
    int       rows, cols;  /* terminal size (set at start and on resize) */
} Scene;

/* The only globals. They have to be globals because the signal handler can't be
 * passed anything — everything else travels inside the Scene that main() owns. */
static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

/* ── §2  performance: timing helpers (the frame cap lives in main) ── */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3  logic: pure read-only helpers — no changing state, no drawing ── */

/* Everything here just computes an answer from its inputs and returns it. None
 * of it stores anything or touches the screen, which is why a column can be
 * recomputed from scratch every frame without anything going stale. */

static float fade(float t) { return t * t * t * (t * (t * 6.f - 15.f) + 10.f); }
static float lerp(float a, float b, float t) { return a + t * (b - a); }

/* Which grid cell a coordinate falls in, folded back into the table's 0..255
 * range. The `& (PERM-1)` is just a fast way to take "remainder by 256". */
static int lattice_index(float v) { return (int)floorf(v) & (PERM - 1); }

/* How strongly one corner's arrow points toward our sample point: 0 if we're
 * sitting on the corner, growing as we move along the arrow's direction. This
 * is the bit that makes Perlin noise roll smoothly instead of looking blocky. */
static float grad_dot(const PerlinNoise *n, int hash, float dx, float dy)
{
    return n->gx[hash] * dx + n->gy[hash] * dy;
}

/* The noise value at one point: find the grid cell, look up its four corners'
 * arrows, and blend them, weighting by how close we are to each corner. */
static float perlin(const PerlinNoise *n, float x, float y)
{
    /* which cell we're in, and how far into it we are (0..1 each way) */
    int   xi = lattice_index(x), yi = lattice_index(y);
    float xf = x - floorf(x),    yf = y - floorf(y);
    float u  = fade(xf),         v  = fade(yf);   /* eased blend weights */

    /* turn the four corner coordinates into table slots */
    int aa = n->perm[n->perm[xi  ] + yi  ];
    int ab = n->perm[n->perm[xi  ] + yi+1];
    int ba = n->perm[n->perm[xi+1] + yi  ];
    int bb = n->perm[n->perm[xi+1] + yi+1];

    /* blend the four corners' contributions, left-right then top-bottom */
    return lerp(lerp(grad_dot(n, aa, xf,       yf      ),
                     grad_dot(n, ba, xf - 1.f, yf      ), u),
                lerp(grad_dot(n, ab, xf,       yf - 1.f),
                     grad_dot(n, bb, xf - 1.f, yf - 1.f), u), v);
}

/* Build terrain by piling up several copies of the noise — each next one finer
 * and fainter than the last (see the FBM_* knobs). Dividing by the running
 * total `m` rescales the result to roughly -0.5..0.5; because of that rescale,
 * the starting amplitude (a's initial 0.6) doesn't matter — only the ratios do. */
static float fbm(const PerlinNoise *n, float x, float y)
{
    float v = 0.f, a = 0.6f, f = 1.f, m = 0.f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        v += a * perlin(n, x * f, y * f);
        m += a; a *= FBM_GAIN; f *= FBM_LACUNARITY;
    }
    return v / m;   /* roughly -0.5 .. 0.5 */
}

/* Is there a star at this cell? Decided straight from the coordinates, so the
 * same cells are stars every frame — no list to keep, and they never flicker. */
static bool is_star(int col, int row)
{
    unsigned h = (unsigned)col * 2654435761u ^ (unsigned)row * 2246822519u;
    return (h & 0x1FF) < 3;   /* roughly 0.6% of cells */
}

/* The screen row of one layer's ridge at one column. It takes the whole
 * Landscape (not loose numbers) because it needs the noise, the camera
 * position and the zoom together — they're really one thing, "the moving
 * terrain". scroll_frac is this layer's parallax speed; yoffset keeps it
 * reading a different slice of noise than the other layers. */
static int layer_row(const Landscape *ld, float scroll_frac, float base_freq,
                     float base, float amp,
                     float col, float yoffset, int draw_rows)
{
    /* where to read the noise: the camera shifts the layer by its parallax
     * fraction, and zoom stretches how fast we step across columns */
    float sample_x = ld->scroll * scroll_frac + col * base_freq * ld->zoom;
    float n        = fbm(&ld->noise, sample_x, yoffset);

    /* turn that into a row: start at the layer's resting line, then lift the
     * ridge up toward the top by an amount set by the noise */
    float top_frac = base - (n + 0.5f) * amp;
    int   row      = (int)(top_frac * (float)draw_rows);
    if (row < 0)          row = 0;
    if (row >= draw_rows) row = draw_rows - 1;
    return row;
}

/* The three ridge rows (far, mid, near) for one column, then nudged so each
 * nearer one sits at least a row below the one behind it. That keeps the layers
 * from ever crossing over and looking inside-out. */
static void column_horizons(const Landscape *ld, int col, int draw_rows,
                            int *rf, int *rm, int *rn)
{
    *rf = layer_row(ld, SCROLL_FAR,  BASE_FREQ_FAR,  BASE_FAR,  AMP_FAR,
                    (float)col, NOISE_YOFF_FAR,  draw_rows);
    *rm = layer_row(ld, SCROLL_MID,  BASE_FREQ_MID,  BASE_MID,  AMP_MID,
                    (float)col, NOISE_YOFF_MID,  draw_rows);
    *rn = layer_row(ld, SCROLL_NEAR, BASE_FREQ_NEAR, BASE_NEAR, AMP_NEAR,
                    (float)col, NOISE_YOFF_NEAR, draw_rows);

    if (*rm < *rf + 1) *rm = *rf + 1;
    if (*rn < *rm + 1) *rn = *rm + 1;
    if (*rf >= draw_rows) *rf = draw_rows - 1;
    if (*rm >= draw_rows) *rm = draw_rows - 1;
    if (*rn >= draw_rows) *rn = draw_rows - 1;
}

/* Picks the character and colour for one cell, based only on where its row
 * falls among the three ridges: sky above the far ridge (with stars and a
 * lighter-to-darker gradient), then for each layer a lit ridge line and the
 * fill below it. Returns the character; hands back the colour through *cp. */
static chtype band_cell(int r, int rf, int rm, int rn, int col, int *cp)
{
    if (r < rf) {                                    /* sky */
        float sky_t = (float)r / (float)(rf > 0 ? rf : 1);
        if (sky_t < SKY_STAR_MAX && is_star(col, r)) { *cp = CP_STAR; return '.'; }
        *cp = (sky_t < SKY_SPLIT) ? CP_SKY_HI : CP_SKY_LO;
        return ' ';
    }
    if (r == rf) { *cp = CP_FAR_E;  return '^'; }    /* far ridge line   */
    if (r < rm)  { *cp = CP_FAR_F;  return (r == rf + 1) ? ':' : '.'; }
    if (r == rm) { *cp = CP_MID_E;  return '^'; }    /* mid ridge line   */
    if (r < rn)  { *cp = CP_MID_F;  return (r == rm + 1) ? ':' : '#'; }
    if (r == rn) { *cp = CP_NEAR_E; return '~'; }    /* near ground edge */
    *cp = (r < rn + NEAR_FILL_ROWS) ? CP_NEAR_F : CP_NEAR_B;   /* fades to base */
    return '#';
}

/* ── §4  simulation: the only code that changes the world's state ── */

/* The per-frame move is just one line in main() ("scroll += speed"), too small
 * to be its own function. The functions here are the setup: make a fresh noise
 * field, and reset the camera and knobs (at startup, or when 'r' is pressed). */

/* Fill in a brand-new random noise field: give every grid corner an arrow
 * pointing a random way, shuffle the table, then copy the whole thing once more
 * onto the end (the doubling perlin() relies on to avoid wrap-around math). */
static void noise_init(PerlinNoise *n)
{
    /* start with slots 0,1,2,... in order, each with a random arrow */
    for (int i = 0; i < PERM; i++) {
        n->perm[i] = (unsigned char)i;
        float a = (float)rand() / RAND_MAX * 2.f * (float)M_PI;
        n->gx[i] = cosf(a);
        n->gy[i] = sinf(a);
    }
    /* shuffle them — keeping each slot's arrow with its slot */
    for (int i = PERM - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        unsigned char t = n->perm[i]; n->perm[i] = n->perm[j]; n->perm[j] = t;
        float tx = n->gx[i]; n->gx[i] = n->gx[j]; n->gx[j] = tx;
        float ty = n->gy[i]; n->gy[i] = n->gy[j]; n->gy[j] = ty;
    }
    /* copy the whole table onto the back of itself */
    for (int i = 0; i < PERM; i++) {
        n->perm[i + PERM] = n->perm[i];
        n->gx[i + PERM] = n->gx[i];
        n->gy[i + PERM] = n->gy[i];
    }
}

/* Back to a fresh start with new terrain. Deliberately leaves `paused` alone,
 * so pressing 'r' while paused resets the world without un-pausing it. */
static void landscape_reset(Landscape *ld)
{
    ld->scroll = 0.f;
    ld->speed  = 0.6f;
    ld->zoom   = 1.0f;
    noise_init(&ld->noise);
}

static void landscape_init(Landscape *ld)
{
    ld->paused = false;
    landscape_reset(ld);
}

/* ── §5  effects: none — nothing cosmetic is stored between frames ── */
/* ── §6  delays: none — the only timing state is the pause toggle ── */

/* ── §7  render: turn the world into screen output (never changes it) ── */

/* These read the Scene and draw it; they only ever write to ncurses and the
 * colour table, never back to the world. */

static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    /* every slot gets the terminal-default background so the sky shows through */
    if (COLORS >= 256) {
        init_pair(CP_SKY_HI, th->c[0], -1);
        init_pair(CP_SKY_LO, th->c[1], -1);
        init_pair(CP_STAR,   th->c[2], -1);
        init_pair(CP_FAR_E,  th->c[3], -1);
        init_pair(CP_FAR_F,  th->c[4], -1);
        init_pair(CP_MID_E,  th->c[5], -1);
        init_pair(CP_MID_F,  th->c[6], -1);
        init_pair(CP_NEAR_E, th->c[7], -1);
        init_pair(CP_NEAR_F, th->c[8], -1);
        init_pair(CP_NEAR_B, th->c[9], -1);
        init_pair(CP_HUD,    th->hud,  -1);
    } else {
        init_pair(CP_SKY_HI, th->c8[0], -1);
        init_pair(CP_SKY_LO, th->c8[1], -1);
        init_pair(CP_STAR,   th->c8[2], -1);
        init_pair(CP_FAR_E,  th->c8[3], -1);
        init_pair(CP_FAR_F,  th->c8[4], -1);
        init_pair(CP_MID_E,  th->c8[5], -1);
        init_pair(CP_MID_F,  th->c8[6], -1);
        init_pair(CP_NEAR_E, th->c8[7], -1);
        init_pair(CP_NEAR_F, th->c8[8], -1);
        init_pair(CP_NEAR_B, th->c8[9], -1);
        init_pair(CP_HUD,    th->hud8,  -1);
    }
}

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    theme_apply(theme);
    /* the key legend is always cyan whatever the theme, so set it once here
     * instead of in theme_apply() (which never touches it) */
    init_pair(CP_HINT, (COLORS >= 256) ? 51 : COLOR_CYAN, -1);
}

/* Draw a string, but cut it off at the right edge so a long line can't spill
 * onto the next row and mess up the display. */
static void hud_draw(int cols, int row, int col, int pair, int attr, const char *s)
{
    if (col < 0) col = 0;
    int avail = cols - col;
    if (avail <= 0) return;
    char buf[256];
    snprintf(buf, sizeof buf, "%s", s);
    if ((int)strlen(buf) > avail) buf[avail] = '\0';   /* cut, don't wrap */
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(row, col, "%s", buf);
    attroff(COLOR_PAIR(pair) | attr);
}

/* The status overlay: title and run-state on top, current settings underneath,
 * and the key legend on the bottom row. */
static void draw_hud(const Scene *sc)
{
    int cols = sc->cols;

    /* top-left: title (bold so it stands out) */
    hud_draw(cols, 0, 1, CP_HUD, A_BOLD, " PARALLAX LANDSCAPE ");

    /* top-right: running or paused */
    const char *state = sc->landscape.paused ? " PAUSED " : " scrolling ";
    hud_draw(cols, 0, cols - (int)strlen(state), CP_HUD, A_BOLD, state);

    /* second row: the live settings */
    char buf[256];
    snprintf(buf, sizeof buf,
             " scroll:%.0f  speed:%+.2f  zoom:%.1fx  theme:%s ",
             (double)sc->landscape.scroll, (double)sc->landscape.speed,
             (double)sc->landscape.zoom, k_themes[sc->theme].name);
    hud_draw(cols, 1, 0, CP_HUD, A_NORMAL, buf);

    /* bottom row: the key legend */
    hud_draw(cols, sc->rows - 1, 0, CP_HINT, A_BOLD,
             " q:quit  p:pause  r:reset  <-/->:speed/dir  +/-:zoom  t:theme ");
}

/* Repaint the whole picture: for each column work out its three ridges, fill
 * the column top to bottom (sky first, then far/mid/near in order so each
 * paints over the one behind), and finally lay the HUD on top. */
static void scene_draw(const Scene *sc)
{
    int draw_rows = sc->rows - HUD_TOP - HUD_BOTTOM;
    if (draw_rows < 1) draw_rows = 1;

    for (int col = 0; col < sc->cols; col++) {
        int rf, rm, rn;
        column_horizons(&sc->landscape, col, draw_rows, &rf, &rm, &rn);

        for (int r = 0; r < draw_rows; r++) {
            int cp;
            chtype ch = band_cell(r, rf, rm, rn, col, &cp);
            attron(COLOR_PAIR(cp));
            mvaddch(r + HUD_TOP, col, ch);
            attroff(COLOR_PAIR(cp));
        }
    }

    draw_hud(sc);
}

/* ── §8  app: signals, startup/shutdown, the main loop ── */

/* main() sets up the terminal, then runs the one loop that ties it all
 * together each frame. Keypresses and window resizes do change things, but
 * they're separate from the steady camera advance. */

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

int main(void)
{
    srand((unsigned)time(NULL));
    atexit(cleanup);
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    Scene sc;
    sc.theme = 0;
    color_init(sc.theme);
    getmaxyx(stdscr, sc.rows, sc.cols);
    landscape_init(&sc.landscape);

    Landscape *ld = &sc.landscape;   /* shorthand for the loop below */

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, sc.rows, sc.cols);
        }

        /* read whatever key was pressed and act on it */
        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case 'p': case 'P': ld->paused = !ld->paused; break;
        case 'r': case 'R': landscape_reset(ld); break;
        case KEY_RIGHT:
            ld->speed += 0.1f; if (ld->speed > SPEED_MAX) ld->speed = SPEED_MAX; break;
        case KEY_LEFT:
            ld->speed -= 0.1f; if (ld->speed < -SPEED_MAX) ld->speed = -SPEED_MAX; break;
        case '+': case '=':
            ld->zoom *= 1.25f; if (ld->zoom > 8.f) ld->zoom = 8.f; break;
        case '-':
            ld->zoom *= 0.8f; if (ld->zoom < 0.125f) ld->zoom = 0.125f; break;
        case 't': case 'T':
            sc.theme = (sc.theme + 1) % N_THEMES;
            theme_apply(sc.theme);
            break;
        default: break;
        }

        long long now = clock_ns();

        if (!ld->paused) ld->scroll += ld->speed;   /* move the camera one step */

        erase();
        scene_draw(&sc);
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));   /* hold ~30 fps */
    }
    return 0;
}
