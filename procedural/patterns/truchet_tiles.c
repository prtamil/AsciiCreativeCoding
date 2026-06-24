/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * truchet_tiles.c — fill the screen with one tiny tile drawn at a random
 * rotation in every cell, and watch swirls, mazes, and ribbons appear out of
 * pure local randomness. That surprise is the "Truchet effect". The picture is
 * frozen: it only changes when you reseed or switch pattern / glyph set / theme.
 *
 * Sébastien Truchet first noticed this in 1704. Good overview: C.S. Smith,
 * "The Tiling Patterns of Sébastien Truchet...", Leonardo 20(4), 1987.
 * The NOISE / VORONOI distributions use Perlin (1985) and Worley (1996) noise.
 * Sister files: ../worldgen/cloud.c (same noise, animated) and
 * ../fractal_random/automaton_2d.c (also complexity from a simple per-cell rule).
 *
 * Keys: q/ESC quit · r reseed · n/p pattern · g/G glyph set · t/T theme · ]/[ Hz
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

/* ── §1 CONFIG — constants and the pattern/glyph/theme tables ─────────── */

enum {
    SIM_FPS_MIN         =  10,
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,
    SIM_FPS_STEP        =  10,

    /* Nothing animates, but the loop still spins for keys and resizes, so we
     * still cap the redraw rate and guard against one stalled frame. */
    RENDER_FPS_CAP      =  60,    /* don't redraw faster than this (Hz)              */
    MAX_FRAME_DT_MS     = 100,    /* treat a long pause as at most this, so we don't lurch */

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* Colour-pair slots. PAIR_HUD/PAIR_HINT are reserved project-wide (CLAUDE.md). */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* this plus 0..7 = the 8 theme shades */

    /* Each cell is tinted by which way its tile leans, using these two ends of
     * the colour ramp. So /-regions look light and \-regions look dark, which
     * is what makes the pattern's shape pop out. */
    TIER_HI             =   7,
    TIER_LO             =   4,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* How many layers of noise the NOISE pattern stacks (more = finer detail). */
#define FBM_OCTAVES          3

/*
 * Pattern — the four ways we can spread the tile rotations across the grid,
 * cycled with n/p. Every tile needs to pick a rotation; a Pattern decides the
 * *style* of that choosing. It turns a tile's (x, y) plus the seed into a single
 * number from 0 to 1 (see pattern_value); the glyph set later turns that number
 * into one of its rotations. Pattern and glyph set are independent: Pattern sets
 * the overall texture, the glyph set picks the alphabet of marks.
 *
 *   RANDOM  — every tile flips its own coin, no relation to its neighbours; the
 *             classic speckled Truchet look (Truchet 1704; Smith 1987).
 *   NOISE   — smooth, cloud-like regions where nearby tiles agree (Perlin 1985).
 *             The only pattern that uses Scene.perm.
 *   BANDS   — a wave across the grid, giving clean diagonal stripes.
 *   VORONOI — scattered seed points carve the grid into irregular blocks, each
 *             block sharing one rotation (Worley 1996).
 */
typedef enum {
    PATTERN_RANDOM  = 0,    /* every tile independent — speckled       */
    PATTERN_NOISE   = 1,    /* smooth cloud-like regions               */
    PATTERN_BANDS   = 2,    /* diagonal stripes                        */
    PATTERN_VORONOI = 3,    /* irregular blocks                        */
    N_PATTERNS      = 4,    /* how many patterns there are (not one)   */
} Pattern;

/* Knobs that shape each pattern. */
#define NOISE_SCALE_X        0.045f     /* how zoomed-in the NOISE clouds are */
#define NOISE_SCALE_Y        0.090f
#define BANDS_FREQ_X         0.25f      /* how tight the BANDS stripes are    */
#define BANDS_FREQ_Y         0.45f
#define VORONOI_GRID_X       6          /* VORONOI block size, in columns     */
#define VORONOI_GRID_Y       3          /* VORONOI block size, in rows        */

/*
 * GlyphSet — the actual marks a tile can show, plus how to lay them out. A
 * Pattern hands over a number from 0 to 1; this turns it into one of n_orient
 * rotations and looks up the character(s) to print (see truchet_glyph).
 *
 * There are 12 sets, in three families:
 *   2 rotations, 1 cell wide : diag, lens, brkt, wave
 *   4 rotations, 1 cell wide : axis, cross, arrow, dots
 *   2 rotations, 2 cells wide: slope, tri, wcurv, wbrkt
 */
typedef struct {
    const char *name;   /* short label shown in the HUD (g/G cycles it)        */
    int  n_orient;      /* how many rotations this set has — 2 or 4             */
    int  tile_w;        /* how many screen cells wide one tile is — 1 or 2      */
    char glyphs[8];     /* the marks, row by row per rotation: [orient*tile_w + sub_x].
                           Spare slots are left 0. */
} GlyphSet;

static const GlyphSet GLYPH_SETS[] = {
    /*  name     n_orient tile_w  glyphs */
    {  "diag ",    2,    1,   { '/', '\\',                  0, 0, 0, 0, 0, 0 } },
    {  "lens ",    2,    1,   { '(', ')',                   0, 0, 0, 0, 0, 0 } },
    {  "brkt ",    2,    1,   { '[', ']',                   0, 0, 0, 0, 0, 0 } },
    {  "wave ",    2,    1,   { '~', '-',                   0, 0, 0, 0, 0, 0 } },
    {  "axis ",    4,    1,   { '/', '\\', '_', '|',        0, 0, 0, 0 } },
    {  "cross",    4,    1,   { '/', '\\', '+', 'X',        0, 0, 0, 0 } },
    {  "arrow",    4,    1,   { '<', '>',  '^', 'v',        0, 0, 0, 0 } },
    {  "dots ",    4,    1,   { 'o', 'O',  '#', '@',        0, 0, 0, 0 } },
    {  "slope",    2,    2,   { ',', '\'', '\'', ',',       0, 0, 0, 0 } },
    {  "tri  ",    2,    2,   { '<', '>',  '>',  '<',       0, 0, 0, 0 } },
    {  "wcurv",    2,    2,   { '(', ')',  ')',  '(',       0, 0, 0, 0 } },
    {  "wbrkt",    2,    2,   { '[', ']',  ']',  '[',       0, 0, 0, 0 } },
};
#define N_GLYPH_SETS ((int)(sizeof GLYPH_SETS / sizeof GLYPH_SETS[0]))

/*
 * Theme — one named colour palette, cycled with t/T. Keeping each palette as a
 * single row of the themes[] table means adding or tweaking one touches just one
 * line, and theme_apply simply copies the row into ncurses colour pairs. All
 * colours sit in the bright half of the 256-colour space (CLAUDE.md rule) so
 * even faint cells stay readable on a black terminal.
 *
 *   name — the label shown in the HUD.
 *   ramp — 8 colours running dark to bright. The picture mostly uses the four
 *          brightest (tiers 4..7); all 8 show up in the HUD's little swatch.
 */
typedef struct {
    const char *name;
    short       ramp[8];
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name       0    1    2    3    4    5    6    7  */
    { "DEFAULT",{ 24,  31,  39,  70,  76, 137, 215, 230 } },
    { "MATRIX", { 28,  34,  40,  46,  76, 118, 154, 192 } },
    { "NOVA",   { 60,  91, 134, 165, 207, 213, 219, 231 } },
    { "MONO",   {240, 243, 245, 247, 249, 251, 253, 255 } },
    { "OCEAN",  { 24,  25,  31,  38,  45,  51, 117, 195 } },
    { "FIRE",   { 88, 124, 130, 166, 202, 208, 214, 226 } },
    { "EARTH",  { 94, 130, 137, 173, 179, 215, 222, 230 } },
    { "FOREST", { 28,  34,  40,  70,  76, 112, 156, 192 } },
    { "DESERT", {130, 137, 143, 173, 179, 215, 222, 229 } },
    { "ARCTIC", { 24,  31,  67, 110, 117, 153, 195, 231 } },
};

/* ── §2 PERFORMANCE — clock + sleep ───────────────────────────────────── */
/* Just the two timing primitives; the frame pacing that uses them is in main. */

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

/* ── §3 LOGIC — small pure helpers (answer depends only on the arguments) ─ */

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_RANDOM:  return "RANDOM ";
    case PATTERN_NOISE:   return "NOISE  ";
    case PATTERN_BANDS:   return "BANDS  ";
    case PATTERN_VORONOI: return "VORONOI";
    default:              return "?      ";
    }
}

/* Scramble three integers into one well-mixed pseudo-random number. */
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

/* Turn a hash into a float from 0 up to (not including) 1. We use 24 bits
 * because that's all a float can hold exactly. */
static inline float hash_unit(uint32_t h)
{
    return (float)(h & 0xFFFFFFu) / 16777216.0f;
}

/* Next / previous item in a wrap-around list — used to cycle pattern, glyph set
 * and theme. wrap_dec adds n first so the result never goes negative. */
static inline int wrap_inc(int v, int n) { return (v + 1) % n; }
static inline int wrap_dec(int v, int n) { return (v + n - 1) % n; }

/* ── §4 SIMULATION — the noise field, the Truchet pattern, and Scene ───── */
/* Nothing here animates. The pattern is a frozen picture; a tick only nudges a */
/* clock that feeds fresh randomness into the next reseed.                      */

/*
 * Permutation — a shuffled bag of the numbers 0..255 that Perlin noise uses as
 * its source of randomness (Perlin 1985). It's stored twice in a row (512 slots,
 * the second half a copy of the first) so noise lookups can add an offset and
 * read past the end without bothering to wrap around. Rebuilt whenever the seed
 * or pattern changes; only the NOISE pattern reads it. Copied inline to keep
 * this one file self-contained.
 */
typedef struct {
    uint8_t table[512];   /* the shuffle, then the same shuffle again */
} Permutation;

static void perm_shuffle(Permutation *p, int seed)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    uint32_t st = (uint32_t)seed * 2654435761u;
    for (int i = 255; i > 0; i--) {
        st = st * 1664525u + 1013904223u;
        int j = (int)(st >> 16) % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
    for (int i = 0; i < 256; i++) {
        p->table[i      ] = base[i];
        p->table[i + 256] = base[i];
    }
}

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

static float perlin2d(const Permutation *pm, float x, float y)
{
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x); y -= floorf(y);
    float u = fade_q(x), v = fade_q(y);
    int A = pm->table[X    ] + Y;
    int B = pm->table[X + 1] + Y;
    float n00 = grad2(pm->table[A    ], x,        y       );
    float n10 = grad2(pm->table[B    ], x - 1.0f, y       );
    float n01 = grad2(pm->table[A + 1], x,        y - 1.0f);
    float n11 = grad2(pm->table[B + 1], x - 1.0f, y - 1.0f);
    return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

static float fbm2(const Permutation *pm, float x, float y)
{
    float total = 0.0f, amp = 1.0f, freq = 1.0f, max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * perlin2d(pm, x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;     /* rescale to land in 0..1 */
}

/*
 * nearest_seed_cell — the heart of the VORONOI pattern. Picture the grid carved
 * into coarse blocks, each holding one scattered seed point. For tile (tx,ty) we
 * check the 3x3 ring of blocks around it and report which seed is closest, so a
 * whole block of tiles ends up "belonging" to the same seed. Vertical distance
 * counts double because terminal cells are about twice as tall as they are wide.
 * The winning block is returned in (*best_cx, *best_cy).
 */
static void nearest_seed_cell(int tx, int ty, int seed,
                              int *best_cx, int *best_cy)
{
    int cx = tx / VORONOI_GRID_X;
    int cy = ty / VORONOI_GRID_Y;
    *best_cx = cx; *best_cy = cy;
    long best_d = (long)1 << 60;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int gcx = cx + dx, gcy = cy + dy;
            uint32_t h = hash3(gcx, gcy, seed);
            int sx = gcx * VORONOI_GRID_X + (int)(h        % (uint32_t)VORONOI_GRID_X);
            int sy = gcy * VORONOI_GRID_Y + (int)((h >> 8) % (uint32_t)VORONOI_GRID_Y);
            long ddx = (long)(sx - tx);
            long ddy = (long)(sy - ty) * 2;     /* cells are ~2x taller than wide */
            long d   = ddx * ddx + ddy * ddy;
            if (d < best_d) { best_d = d; *best_cx = gcx; *best_cy = gcy; }
        }
    }
}

/*
 * pattern_value — give one tile a number from 0 to 1. That number is what a
 * glyph set later turns into a rotation. Each pattern computes it a different
 * way, which is what gives each its distinct look.
 */
static float pattern_value(const Permutation *pm, Pattern p,
                           int tx, int ty, int seed)
{
    switch (p) {
    case PATTERN_RANDOM:                          /* just a per-tile coin flip */
        return hash_unit(hash3(tx, ty, seed));

    case PATTERN_NOISE: {                          /* smooth cloud-like noise */
        float ox = (float)((seed >> 16) & 0xFFFF) * 0.001f;   /* seed slides the clouds around */
        float oy = (float)( seed        & 0xFFFF) * 0.001f;
        return fbm2(pm, (float)tx * NOISE_SCALE_X + ox,
                    (float)ty * NOISE_SCALE_Y + oy);
    }
    case PATTERN_BANDS: {                          /* a wave, giving stripes */
        float phase = (float)(seed & 0xFFFF) * (1.0f / 65536.0f) * 6.2832f;
        float a     = (float)tx * BANDS_FREQ_X + (float)ty * BANDS_FREQ_Y + phase;
        return 0.5f + 0.5f * sinf(a);
    }
    case PATTERN_VORONOI: {                        /* whichever seed point is nearest */
        int best_cx, best_cy;
        nearest_seed_cell(tx, ty, seed, &best_cx, &best_cy);
        return hash_unit(hash3(best_cx, best_cy, seed ^ 0x9E3779B9));
    }
    case N_PATTERNS: break;
    }
    return 0.5f;
}

/*
 * truchet_glyph — work out which character to print at one screen cell. Find the
 * tile that cell belongs to, ask the pattern for that tile's 0..1 number, turn it
 * into a rotation, and look up the matching mark. Same inputs always give the
 * same answer, which is why the picture is frozen. The chosen rotation is handed
 * back in *out_orient so the renderer can tint by it. Only the NOISE pattern uses
 * pm; the rest ignore it.
 */
static char truchet_glyph(const Permutation *pm, int sx, int sy, int seed,
                          Pattern p, int set_idx, int *out_orient)
{
    if (set_idx < 0 || set_idx >= N_GLYPH_SETS) set_idx = 0;
    const GlyphSet *gs = &GLYPH_SETS[set_idx];
    int tile_w = gs->tile_w;
    int tile_x = (sx >= 0) ? (sx / tile_w) : -((-sx + tile_w - 1) / tile_w);
    int sub_x  = sx - tile_x * tile_w;

    float v = pattern_value(pm, p, tile_x, sy, seed);
    int orient = (int)(v * (float)gs->n_orient);
    if (orient < 0)              orient = 0;
    if (orient >= gs->n_orient)  orient = gs->n_orient - 1;

    *out_orient = orient;
    return gs->glyphs[orient * tile_w + sub_x];
}

/*
 * Scene — everything that defines the current picture, in one place. Only the
 * top-level routines (init / reseed / tick) take a whole Scene*; everything else
 * takes just the narrow piece it needs.
 */
typedef struct {
    /* What the pattern is built from. */
    Permutation perm;            /* noise table, only the NOISE pattern uses it */
    int      seed;               /* the one number every tile's rotation comes from */

    /* What the user has dialled in. */
    Pattern  current_pattern;    /* which distribution (n/p) */
    int      current_glyph_set;  /* which marks (g/G)        */

    /* A running clock, kept only to make each reseed land differently. */
    float    time_secs;

    /* Just a colour choice (t/T). */
    int      current_theme;
} Scene;

/* Re-shuffle the noise table for the current seed and pattern. Folding the
 * pattern into the seed means each pattern gets its own distinct noise, while
 * still being repeatable for a given seed+pattern. */
static void apply_perm(Scene *s)
{
    perm_shuffle(&s->perm, s->seed ^ ((int)s->current_pattern * 0xA5A5A5));
}

static void scene_reseed(Scene *s)
{
    /* Stir the clock into the old seed so every 'r' gives a fresh picture. */
    s->seed = (int)hash3((int)(s->time_secs * 1000.0f), s->seed, 0xC0FFEE);
    apply_perm(s);
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->current_theme     = 0;
    s->current_pattern   = PATTERN_NOISE;   /* start on NOISE — its flowing regions */
    s->current_glyph_set = 0;               /* look better than RANDOM's plain speckle */
    s->seed              = 0xC0FFEE;
    apply_perm(s);
}

/* The picture is frozen, so a tick does nothing but advance the clock that
 * scene_reseed reads. Nothing on screen moves between frames. */
static void scene_tick(Scene *s, float dt)
{
    s->time_secs += dt;
}

/* ── §5 RENDER — draw the picture (reads the Scene, never changes it) ──── */
/* Sets up colours, then paints each cell and the HUD on top. */

/* Load a theme's 8 colours into the ncurses pairs (or a coarse fallback when
 * the terminal only has 8 colours). */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
    } else {
        static const short fb[8] = {
            COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN,   COLOR_CYAN,
            COLOR_GREEN, COLOR_YELLOW,COLOR_YELLOW, COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), fb[i], -1);
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

/*
 * Screen — just the terminal's current size. It's a type of its own so the
 * drawing code can take a small read-only handle instead of the whole program.
 * Filled in at startup and refreshed on every resize. The pattern fills rows
 * 2..bottom; rows 0-1 are the HUD and the last row is the key hint.
 */
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

/* The endwin/refresh pair is the ncurses way to pick up the new terminal size
 * after a resize, before we re-read it. */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* Map a rotation (0..n_orient-1) to one of the bright colour shades, with
 * rotation 0 the brightest. That light-vs-dark tint is what makes the pattern's
 * shape visible. */
static inline int orient_to_tier(int orient, int n_orient)
{
    int span = (n_orient > 1) ? n_orient - 1 : 1;
    return TIER_HI - (orient * (TIER_HI - TIER_LO)) / span;
}

/* Paint the whole pattern: for each cell, fetch its mark and tint it by the
 * tile's lean. Identical every frame until a key reseeds or changes a knob. */
static void scene_draw(const Screen *sc, const Scene *s)
{
    int top = 2, bottom = sc->rows - 1;
    int n_orient = GLYPH_SETS[s->current_glyph_set].n_orient;
    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < sc->cols; sx++) {
            int  orient;
            char glyph = truchet_glyph(&s->perm, sx, sy, s->seed,
                                       s->current_pattern,
                                       s->current_glyph_set, &orient);

            int pair = PAIR_RAMP_BASE + orient_to_tier(orient, n_orient);
            attron(COLOR_PAIR(pair) | A_BOLD);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | A_BOLD);
        }
    }
}

/* Top row: title on the left, fps and tick rate on the right. (No pause state —
 * a frozen picture has nothing to pause.) */
static void hud_draw_status_line(const Screen *sc, double fps, int sim_fps)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz ", fps, sim_fps);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    mvprintw(0, 1, " TRUCHET TILES ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Second row: the current knob settings — pattern, glyph set, theme, a live
 * swatch of the 8 ramp colours, and the seed. Each field prints, then x steps
 * past its fixed-width column. */
static void hud_draw_param_line(const Screen *sc, const Scene *s)
{
    (void)sc;
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-7s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 18;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " glyph:%-5s ", GLYPH_SETS[s->current_glyph_set].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 14;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    /* Little swatch showing all 8 theme colours, each in its own pair. */
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
    mvprintw(1, x, "  seed:%08x ", (unsigned)s->seed);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row: the full list of keys. */
static void hud_draw_key_hints(const Screen *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  g/G:glyph  t/T:theme  r:reseed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);
    hud_draw_status_line(sc, fps, sim_fps);
    hud_draw_param_line(sc, s);
    hud_draw_key_hints(sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §6 APP — signals, resize, keys, and the main loop ────────────────── */

/*
 * App — the whole running program in one bundle: the Scene, the Screen, and the
 * few values the loop and the signal handlers share.
 *   scene / screen — what is drawn, and how big the terminal is.
 *   sim_fps        — how often we tick the clock (Hz), nudged by [ and ].
 *   running        — set to 0 to quit; a Ctrl-C / kill signal clears it.
 *   need_resize    — set to 1 when the terminal was resized; handled next loop.
 * running and need_resize are written from signal handlers, so they're the
 * special volatile sig_atomic_t type — the only kind a handler may safely touch,
 * and what stops the compiler from caching the value across loop turns.
 */
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
    screen_resize(&app->screen);
    app->need_resize = 0;
}

/* Apply one keypress. Returns false only when the user asked to quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case 'r': case 'R': scene_reseed(s);                               break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
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

    case 'n': case 'N':
        s->current_pattern = (Pattern)wrap_inc((int)s->current_pattern, N_PATTERNS);
        apply_perm(s);                 /* re-shuffle noise for the new pattern */
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)wrap_dec((int)s->current_pattern, N_PATTERNS);
        apply_perm(s);
        break;

    case 'g':
        s->current_glyph_set = wrap_inc(s->current_glyph_set, N_GLYPH_SETS);
        break;
    case 'G':
        s->current_glyph_set = wrap_dec(s->current_glyph_set, N_GLYPH_SETS);
        break;

    default: break;
    }
    return true;
}

/* One-time startup: seed randomness, wire up the signal handlers and the
 * terminal-restore-on-exit hook, then open the screen and build the scene. */
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

    const int64_t max_dt_ns    = (int64_t)MAX_FRAME_DT_MS * NS_PER_MS;
    const int64_t frame_cap_ns = NS_PER_SEC / RENDER_FPS_CAP;

    while (app->running) {

        /* Handle a pending resize before we start timing this frame. */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* How much real time passed, capped so a long stall can't snowball. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > max_dt_ns) dt = max_dt_ns;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        /* Tick the clock in fixed steps, however many fit the elapsed time. */
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* Update the fps reading every so often. */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* Sleep the rest of the frame so we don't redraw faster than the cap. */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(frame_cap_ns - elapsed);

        /* Draw the picture and the HUD. */
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* Grab at most one key this frame. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
