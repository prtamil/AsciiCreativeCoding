/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wang_tiles.c — Wang tiles: square tiles with a colour painted on each of
 * their four edges, laid on a grid so touching edges always match in colour.
 * That one local rule makes long ribbons of colour run across the whole grid,
 * even though every tile was chosen on its own. Hao Wang's 1961 idea; here the
 * full set of 16 tiles you get from 2 colours on 4 edges.
 *
 * The picture never moves — it only redraws when you press a key:
 *   r          re-roll the whole arrangement
 *   n/p        cycle the PATTERN (what nudges each tile's colour choice):
 *              RANDOM = anything goes, NOISE = soft blobs, STRIPES =
 *              horizontal bands, SWIRL = colour split around the centre
 *   g/G        cycle the GLYPH look: EDGES (thin borders),
 *              BLOCKS (heavy borders), WIRES (a wire mesh, the default)
 *   t/T        cycle the colour theme
 *   ] / [      raise / lower tick Hz
 *   q / ESC    quit
 *
 * Related files:
 *   ../patterns/truchet_tiles.c       — same per-cell tile idea, but no
 *                                        edge-matching rule.
 *   ../generational/wfc_showcase.c    — wave-function collapse: the same
 *                                        "tiles must agree on shared edges"
 *                                        idea generalised, with backtracking.
 *
 * Section map:
 *   §1 config        constants + the colour ramp, pattern/glyph/theme tables
 *   §2 performance   the clock and sleep helpers
 *   §3 logic         small pure helpers (names, hashing, index wrap)
 *   §4 simulation    noise + the tile set + the rule-solver that fills the grid
 *   §5 render        colours, screen geometry, per-cell drawing + HUD
 *   §6 app           signals, resize, input, the main loop
 *
 * References (the parts the code can't tell you):
 *   Wang, H. (1961) — "Proving Theorems by Pattern Recognition II", Bell
 *     System Technical Journal 40:1-41. The original edge-matching tiles.
 *   Cohen, Shade, Hiller & Deussen (2003) — "Wang Tiles for Image and Texture
 *     Generation", SIGGRAPH '03:287-294. The "pick a tile that fits its
 *     neighbours" placement this file uses.
 *   Perlin, K. (2002) — "Improving Noise", SIGGRAPH '02. The gradient noise
 *     the NOISE pattern leans on.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/patterns/wang_tiles.c \
 *     -o wang -lncurses -lm
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

/* ── §1 config — constants, colour ramp, pattern/glyph/theme tables ── */

enum {
    SIM_FPS_MIN         =  10,
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,
    SIM_FPS_STEP        =  10,

    /* The picture is still, but the loop still spins for input and resize. */
    RENDER_FPS_CAP      =  60,    /* don't redraw faster than this           */
    MAX_FRAME_DT_MS     = 100,    /* if a frame stalls, pretend it was 100ms */

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* One tile is this many screen cells. 6x3 looks square once you account
     * for terminal cells being about twice as tall as they are wide. */
    TILE_W              =   6,
    TILE_H              =   3,

    /* Two colours on four edges gives every combination: 2^4 = 16 tiles. */
    N_EDGE_COLORS       =   2,
    N_TILES             =  16,

    /* Biggest grid we'll ever need, used to size the grid array once. */
    MAX_SCREEN_W        = 256,
    MAX_SCREEN_H        =  96,
    MAX_GRID_W          = MAX_SCREEN_W / TILE_W,
    MAX_GRID_H          = MAX_SCREEN_H / TILE_H,
    MAX_GRID_CELLS      = MAX_GRID_W * MAX_GRID_H,

    /* The two HUD pairs are fixed by the project; the ramp gets the next 8. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* +0..+7 = the 8 theme shades */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Terminal cells are about twice as tall as wide; the SWIRL pattern stretches
 * its vertical distances by this so its circle of colour looks round. */
#define ASPECT_Y_F           2.0f

/* How many layers of noise the NOISE pattern stacks (more = finer detail). */
#define FBM_OCTAVES          3

/* "There is no neighbour here, so any colour fits" — used for the grid's
 * top row and left column, where there's nothing to match against. */
#define EDGE_ANY (-1)

/* The two edge colours pick two well-spaced shades from the 8-shade ramp,
 * so the two colours stay easy to tell apart. */
static const int EDGE_RAMP[N_EDGE_COLORS] = { 3, 6 };

/*
 * Pattern — the four "moods" you cycle with n/p. The matching rule (which tiles
 * are even allowed next to their neighbours) is always the same; the pattern
 * only leans the choice among the allowed tiles toward one colour or the other,
 * which changes the big-picture look:
 *   RANDOM  — no lean; pick freely. Most varied, no large colour regions.
 *   NOISE   — lean toward the colour a soft noise field likes here -> blobs.
 *   STRIPES — lean the bottom-edge colour by a wave down the page -> bands.
 *   SWIRL   — lean by the angle around the centre -> the grid splits in colour.
 */
typedef enum {
    PATTERN_RANDOM  = 0,
    PATTERN_NOISE   = 1,
    PATTERN_STRIPES = 2,
    PATTERN_SWIRL   = 3,
    N_PATTERNS      = 4,    /* how many there are — not a real pattern */
} Pattern;

/*
 * GlyphSet — three ways to draw the same tiles, cycled with g/G. This is a
 * looks-only choice; it never changes which tile sits where, just the shapes:
 *   EDGES  — thin borders only ('-' '_' '|'), hollow inside.
 *   BLOCKS — heavy '#' borders with a faintly tinted '.' interior.
 *   WIRES  — a wire mesh: matching edges join up into a circuit (the default).
 */
typedef enum {
    GLYPH_EDGES  = 0,
    GLYPH_BLOCKS = 1,
    GLYPH_WIRES  = 2,
    N_GLYPH_SETS = 3,      /* how many there are */
} GlyphSet;

/* How tightly the patterns vary across the grid (smaller = broader features). */
#define NOISE_SCALE_X        0.06f
#define NOISE_SCALE_Y        0.12f
#define STRIPES_FREQ_Y       0.45f

/*
 * Theme — one named colour palette. Each theme is a whole look packed into one
 * row of the themes[] table, so adding or tweaking a palette is a one-line edit
 * and theme_apply just copies the row into ncurses colour pairs. Every number
 * is an index into the 256-colour palette, kept in the bright half so the tiles
 * stay readable on a black terminal.
 *
 *   name — the label shown in the HUD.
 *   ramp — 8 shades from dark to bright. The tiling only uses two of them (the
 *          two EDGE_RAMP slots); all 8 show up in the little HUD swatch.
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

/* ── §2 performance — a clock and a sleep ── */
/* The frame pacing that uses these lives in main (§6). */

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

/* ── §3 logic — small pure helpers (answer depends only on the arguments) ── */

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_RANDOM:  return "RANDOM ";
    case PATTERN_NOISE:   return "NOISE  ";
    case PATTERN_STRIPES: return "STRIPES";
    case PATTERN_SWIRL:   return "SWIRL  ";
    default:              return "?      ";
    }
}

static const char *glyph_set_name(GlyphSet g)
{
    switch (g) {
    case GLYPH_EDGES:  return "edge ";
    case GLYPH_BLOCKS: return "block";
    case GLYPH_WIRES:  return "wire ";
    default:           return "?    ";
    }
}

/* hash3 — turn three integers into one well-scrambled number. Used for "pick
 * one at random but always the same one for this spot" tiebreaks. */
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

/* next/previous in a ring of n items, wrapping past the ends. Used to cycle
 * pattern, glyph set and theme. wrap_dec adds n first so it never goes negative. */
static inline int wrap_inc(int v, int n) { return (v + 1) % n; }
static inline int wrap_dec(int v, int n) { return (v + n - 1) % n; }

/* ── §4 simulation — noise, the tile set, and the rule-solver that fills the grid ── */
/* This is all the real state. None of it runs every frame — only when you    */
/* reseed, change pattern, or resize. (scene_tick just ticks a wall clock.)    */

/*
 * Permutation — the lookup table Perlin noise needs: the numbers 0..255 in a
 * random order, used to turn grid corners into noise values. It's stored twice
 * back-to-back (512 entries) so the noise code can read a little past the end
 * without checking for wraparound. Re-shuffled on every reseed/pattern change;
 * only the NOISE pattern actually reads it. Copied in so the file stands alone.
 */
typedef struct {
    uint8_t table[512];   /* 0..255 shuffled, then repeated into [256,512) */
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
    return (total / max_amp) * 0.5f + 0.5f;     /* shift from -1..1 into 0..1 */
}

/* ---- the tile set and the grid ---- */

/*
 * WangTile — one tile: a square with a colour (0 or 1) painted on each of its
 * four edges, north / east / south / west. The matching rule is simply that
 * touching edges must share a colour, so these four numbers say everything.
 * We use the full set of all 16 colour combinations (tile_set), which is what
 * guarantees that whatever the left and top neighbours demand, some tile fits —
 * the grid can never paint itself into a corner. (Wang 1961.)
 */
typedef struct {
    uint8_t n, e, s, w;   /* edge colour on each side, each 0 or 1 */
} WangTile;

static WangTile tile_set[N_TILES];

/* Build all 16 tiles by reading each tile number's four bits as its four edge
 * colours (top bit = north, then east, south, west). */
static void tile_set_init(void)
{
    for (int i = 0; i < N_TILES; i++) {
        tile_set[i].n = (uint8_t)((i >> 3) & 1);
        tile_set[i].e = (uint8_t)((i >> 2) & 1);
        tile_set[i].s = (uint8_t)((i >> 1) & 1);
        tile_set[i].w = (uint8_t)((i >> 0) & 1);
    }
}

/*
 * WangGrid — the finished tiling: which tile sits in each grid spot. It's just
 * tile numbers, one per spot, laid out row by row (spot (tx,ty) lives at
 * cells[ty*w + tx]). grid_generate fills it so every shared edge matches; the
 * drawing code reads it back. Sized once to the biggest grid we allow, so
 * generating a new tiling never needs to allocate memory.
 *   w, h  — size in TILES, not in screen cells
 *   cells — tile numbers (0..15); only the first w*h entries are in use
 */
typedef struct {
    int     w, h;
    uint8_t cells[MAX_GRID_CELLS];
} WangGrid;

/*
 * pick_preferred — out of the tiles that fit, pick one whose east/south edges
 * best match the colours we'd like here (pass -1 for "no preference" on an
 * edge). When several tie for best, a hash picks one — same spot, same choice. */
static int pick_preferred(int prefer_e, int prefer_s,
                          const uint8_t *valid, int n_valid,
                          int tx, int ty, int seed)
{
    uint8_t best[N_TILES];
    int best_score = -1;
    int n_best     =  0;
    for (int i = 0; i < n_valid; i++) {
        const WangTile *t = &tile_set[valid[i]];
        int score = 0;
        if (prefer_e >= 0 && (int)t->e == prefer_e) score++;
        if (prefer_s >= 0 && (int)t->s == prefer_s) score++;
        if (score > best_score) {
            best_score = score;
            best[0]    = valid[i];
            n_best     = 1;
        } else if (score == best_score) {
            best[n_best++] = valid[i];
        }
    }
    return best[hash3(tx, ty, seed ^ 0x9E3779B9) % (uint32_t)n_best];
}

/*
 * grid_pick — out of the tiles that already fit here (`valid`), choose one the
 * way the current pattern wants. RANDOM just picks one; the others first decide
 * a preferred colour for this spot (from noise / a wave / the angle to centre)
 * and then lean toward a tile carrying it. Either way the matching rule holds,
 * because `valid` was already filtered down to legal tiles.
 */
static int grid_pick(const Permutation *pm, int tx, int ty, int seed, Pattern p,
                     const uint8_t *valid, int n_valid,
                     int grid_w, int grid_h)
{
    if (n_valid <= 0) return 0;

    int prefer_e = -1, prefer_s = -1;   /* -1 means "no preference on this edge" */

    switch (p) {
    case PATTERN_RANDOM:
        return valid[hash3(tx, ty, seed) % (uint32_t)n_valid];
    case PATTERN_NOISE: {                /* soft noise -> blobby colour regions */
        float ox = (float)((seed >> 16) & 0xFFFF) * 0.001f;
        float oy = (float)( seed        & 0xFFFF) * 0.001f;
        float v  = fbm2(pm, (float)tx * NOISE_SCALE_X + ox,
                        (float)ty * NOISE_SCALE_Y + oy);
        int prefer = (v > 0.5f) ? 1 : 0;
        prefer_e = prefer; prefer_s = prefer;
        break;
    }
    case PATTERN_STRIPES: {              /* a wave down the page -> bands */
        float phase = (float)(seed & 0xFFFF) * (1.0f / 65536.0f) * 6.2832f;
        float v     = 0.5f + 0.5f * sinf((float)ty * STRIPES_FREQ_Y + phase);
        prefer_s = (v > 0.5f) ? 1 : 0;
        break;
    }
    case PATTERN_SWIRL: {                /* angle around the centre -> colour split */
        float cx = (float)grid_w * 0.5f;
        float cy = (float)grid_h * 0.5f;
        float dx = (float)tx - cx;
        float dy = ((float)ty - cy) * ASPECT_Y_F;   /* stretch y so the split looks round */
        float a  = atan2f(dy, dx);
        int prefer = (a > 0) ? 1 : 0;
        prefer_e = prefer; prefer_s = prefer;
        break;
    }
    case N_PATTERNS: break;
    }

    return pick_preferred(prefer_e, prefer_s, valid, n_valid, tx, ty, seed);
}

/* collect_valid_tiles — list every tile whose west and north edges match what
 * the left and top neighbours require (EDGE_ANY = no requirement). These are
 * the tiles allowed in this spot. Returns how many there are. */
static int collect_valid_tiles(int expected_w, int expected_n, uint8_t *valid)
{
    int n_valid = 0;
    for (int i = 0; i < N_TILES; i++) {
        if (expected_w != EDGE_ANY && (int)tile_set[i].w != expected_w) continue;
        if (expected_n != EDGE_ANY && (int)tile_set[i].n != expected_n) continue;
        valid[n_valid++] = (uint8_t)i;
    }
    return n_valid;
}

/* Fill the whole grid, one spot at a time, left to right and top to bottom.
 * Each spot must agree with the neighbour to its left and the one above, so by
 * the time we reach it those two edges are already decided. */
static void grid_generate(WangGrid *g, const Permutation *pm, int seed,
                          Pattern p, int grid_w, int grid_h)
{
    if (grid_w > MAX_GRID_W) grid_w = MAX_GRID_W;
    if (grid_h > MAX_GRID_H) grid_h = MAX_GRID_H;
    g->w = grid_w;
    g->h = grid_h;

    /* The top row and left column have no neighbour to match, so they're free. */
    for (int ty = 0; ty < grid_h; ty++) {
        for (int tx = 0; tx < grid_w; tx++) {
            int expected_w = (tx > 0) ? tile_set[g->cells[ty * grid_w + (tx - 1)]].e
                                      : EDGE_ANY;
            int expected_n = (ty > 0) ? tile_set[g->cells[(ty - 1) * grid_w + tx]].s
                                      : EDGE_ANY;

            uint8_t valid[N_TILES];
            int n_valid = collect_valid_tiles(expected_w, expected_n, valid);

            g->cells[ty * grid_w + tx] = (uint8_t)grid_pick(pm, tx, ty, seed, p,
                                            valid, n_valid, grid_w, grid_h);
        }
    }
}

/*
 * Scene — everything about the current picture in one place. The drawing code
 * only reads it; only the handful of "rebuild" functions below change it.
 */
typedef struct {
    /* The tiling itself, plus the noise and seed it was built from. */
    WangGrid    grid;
    Permutation perm;            /* noise table, only the NOISE pattern uses it */
    int         seed;            /* one number that decides the whole layout    */

    /* Choices that, when changed, force a rebuild. */
    Pattern     current_pattern; /* which pattern (n/p)                         */
    int         grid_w_cap;      /* how big the grid may be, in tiles —         */
    int         grid_h_cap;      /*   set from the screen size                  */

    /* A running clock, used only to flavour the next reseed differently. */
    float       time_secs;

    /* Pure looks — changing these just redraws, no rebuild. */
    GlyphSet    current_glyph;   /* which glyph style (g/G)                     */
    int         current_theme;   /* which colour theme (t/T)                    */
} Scene;

/* Re-shuffle the noise table, folding in both the seed and the pattern, so the
 * NOISE look comes out different on every reseed and every pattern. */
static void apply_perm(Scene *s)
{
    perm_shuffle(&s->perm, s->seed ^ ((int)s->current_pattern * 0xA5A5A5));
}

static void scene_regenerate(Scene *s)
{
    apply_perm(s);
    grid_generate(&s->grid, &s->perm, s->seed, s->current_pattern,
                  s->grid_w_cap, s->grid_h_cap);
}

static void scene_reseed(Scene *s)
{
    /* Stir the clock into the old seed so every 'r' press lands somewhere new. */
    s->seed = (int)hash3((int)(s->time_secs * 1000.0f), s->seed, 0xABCDEF);
    scene_regenerate(s);
}

static void scene_init(Scene *s, int grid_w, int grid_h)
{
    memset(s, 0, sizeof *s);
    s->current_theme   = 0;
    s->current_pattern = PATTERN_RANDOM;
    s->current_glyph   = GLYPH_WIRES;   /* the wire mesh looks best on first sight */
    s->seed            = 0xDEADBEEF;
    s->grid_w_cap      = grid_w;
    s->grid_h_cap      = grid_h;
    tile_set_init();
    scene_regenerate(s);
}

static void scene_resize_to(Scene *s, int grid_w, int grid_h)
{
    s->grid_w_cap = grid_w;
    s->grid_h_cap = grid_h;
    scene_regenerate(s);
}

/* The picture never moves, so a tick does nothing but nudge the clock forward —
 * which is only there to make the next 'r' reseed land somewhere fresh. */
static void scene_tick(Scene *s, float dt)
{
    s->time_secs += dt;
}

/* ── §5 render — draw the scene to the screen (reads the scene, never changes it) ── */

/* ---- colour: copy a theme's shades into ncurses colour pairs ---- */

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

/* ---- deciding what to draw in each cell of a tile ---- */

/*
 * cell_glyph_wires — draws one cell of a tile in the WIRES style. An edge gets
 * "wired" only when its colour shows up on at least two of the tile's edges, so
 * it has a same-colour partner to connect to. Each wired edge runs a coloured
 * spoke ('|' or '-') in to the centre, where they meet as a straight, a bend, a
 * T or a cross ('+'). The upshot: same-colour edges link up across tiles and
 * the whole grid reads like a circuit of coloured wires.
 */
static int cell_glyph_wires(const WangTile *t, int dx, int dy,
                            char *out_glyph, bool *visible)
{
    *visible = true;
    int  ramp_idx = 0;
    char glyph    = ' ';

    int cx = TILE_W / 2, cy = TILE_H / 2;
    int cnt[N_EDGE_COLORS] = { 0 };
    cnt[t->n]++; cnt[t->e]++; cnt[t->s]++; cnt[t->w]++;
    bool route_n = cnt[t->n] >= 2;
    bool route_e = cnt[t->e] >= 2;
    bool route_s = cnt[t->s] >= 2;
    bool route_w = cnt[t->w] >= 2;

    if (dx == cx && dy == cy) {                  /* the centre, where spokes meet */
        if      (!route_n && !route_e && !route_s && !route_w) *visible = false;
        else if (route_n && route_s && !route_e && !route_w)   glyph = '|';
        else if (route_e && route_w && !route_n && !route_s)   glyph = '-';
        else                                                   glyph = '+';
        ramp_idx = EDGE_RAMP[(cnt[1] > cnt[0]) ? 1 : 0];   /* whichever colour wins */
    } else if (dx == cx && dy < cy) {            /* spoke heading up    */
        if (route_n) { glyph = '|'; ramp_idx = EDGE_RAMP[t->n]; }
        else         *visible = false;
    } else if (dx == cx && dy > cy) {            /* spoke heading down  */
        if (route_s) { glyph = '|'; ramp_idx = EDGE_RAMP[t->s]; }
        else         *visible = false;
    } else if (dy == cy && dx < cx) {            /* spoke heading left  */
        if (route_w) { glyph = '-'; ramp_idx = EDGE_RAMP[t->w]; }
        else         *visible = false;
    } else if (dy == cy && dx > cx) {            /* spoke heading right */
        if (route_e) { glyph = '-'; ramp_idx = EDGE_RAMP[t->e]; }
        else         *visible = false;
    } else {
        *visible = false;
    }

    *out_glyph = glyph;
    return ramp_idx;
}

/*
 * tile_cell_render — for one cell of a tile (dx, dy are its spot inside the
 * tile), hand back the character to draw, its colour, and whether to draw it at
 * all. EDGES and BLOCKS only paint the four border rows/columns; WIRES draws
 * the mesh above. Cells that should stay blank come back with *visible = false
 * and the caller skips them.
 */
static int tile_cell_render(const WangTile *t, int dx, int dy,
                            GlyphSet g, char *out_glyph, bool *visible)
{
    *visible = true;
    int ramp_idx = 0;
    char glyph = ' ';

    bool is_top    = (dy == 0);
    bool is_bot    = (dy == TILE_H - 1);
    bool is_left   = (dx == 0);
    bool is_right  = (dx == TILE_W - 1);

    if (g == GLYPH_EDGES) {
        if (is_top) {
            glyph = '-'; ramp_idx = EDGE_RAMP[t->n];
        } else if (is_bot) {
            glyph = '_'; ramp_idx = EDGE_RAMP[t->s];
        } else if (is_left) {
            glyph = '|'; ramp_idx = EDGE_RAMP[t->w];
        } else if (is_right) {
            glyph = '|'; ramp_idx = EDGE_RAMP[t->e];
        } else {
            *visible = false;
        }
    }
    else if (g == GLYPH_BLOCKS) {
        if (is_top || is_bot || is_left || is_right) {
            glyph = '#';
            ramp_idx = is_top  ? EDGE_RAMP[t->n]
                     : is_bot  ? EDGE_RAMP[t->s]
                     : is_left ? EDGE_RAMP[t->w]
                     :           EDGE_RAMP[t->e];
        } else {
            /* Inside: pick one of two faint shades from whether the tile's
             * four edge colours add up to an even or odd count, so neighbouring
             * tiles often differ and each tile is recognisable. */
            int parity = (t->n ^ t->e ^ t->s ^ t->w) & 1;
            glyph = '.';
            ramp_idx = parity ? 5 : 4;
        }
    }
    else if (g == GLYPH_WIRES) {
        return cell_glyph_wires(t, dx, dy, out_glyph, visible);
    }

    *out_glyph = glyph;
    return ramp_idx;
}

/* ---- screen: the terminal size and where the grid sits in it ---- */

/*
 * Screen — how big the terminal is and where the tile grid is parked inside it.
 * Kept apart from the app so the drawing code can take just this and nothing
 * else. screen_layout works out the centred placement once per resize; the
 * drawing then maps tile (tx,ty) to the screen cell (gx0 + tx*TILE_W, ...).
 *   cols, rows     — terminal size in characters
 *   gx0, gy0       — screen position of the grid's top-left corner (centred)
 *   grid_w, grid_h — grid size in tiles
 */
typedef struct {
    int cols, rows;
    int gx0, gy0;
    int grid_w, grid_h;
} Screen;

static void screen_layout(Screen *s)
{
    int top = 2, bottom = s->rows - 1;
    int avail_h = bottom - top;
    int avail_w = s->cols;

    int gw = avail_w / TILE_W;
    int gh = avail_h / TILE_H;
    if (gw > MAX_GRID_W) gw = MAX_GRID_W;
    if (gh > MAX_GRID_H) gh = MAX_GRID_H;
    if (gw < 4) gw = 4;
    if (gh < 4) gh = 4;

    s->grid_w = gw;
    s->grid_h = gh;
    /* Centre the grid, with leftover space split as margins, so no half-tiles
     * spill off the edges. */
    s->gx0 = (avail_w - gw * TILE_W) / 2;
    s->gy0 = top + (avail_h - gh * TILE_H) / 2;
    if (s->gx0 < 0)   s->gx0 = 0;
    if (s->gy0 < top) s->gy0 = top;
}

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
    screen_layout(s);
}
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
    screen_layout(s);
}

/* Draw one tile: walk its TILE_W x TILE_H cells, asking tile_cell_render what
 * each one looks like in the current glyph style. Cells off-screen or under the
 * two HUD rows are skipped. */
static void draw_one_tile(const Screen *sc, const WangTile *t,
                          int tx, int ty, GlyphSet g)
{
    for (int dy = 0; dy < TILE_H; dy++) {
        int sy = sc->gy0 + ty * TILE_H + dy;
        if (sy < 2 || sy >= sc->rows - 1) continue;
        for (int dx = 0; dx < TILE_W; dx++) {
            int sx = sc->gx0 + tx * TILE_W + dx;
            if (sx < 0 || sx >= sc->cols) continue;

            char glyph;
            bool visible;
            int ramp_idx = tile_cell_render(t, dx, dy, g, &glyph, &visible);
            if (!visible) continue;

            int pair = PAIR_RAMP_BASE + ramp_idx;
            attron(COLOR_PAIR(pair) | A_BOLD);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | A_BOLD);
        }
    }
}

/* Draw the whole tiling — just every tile in the grid. There are no overlays
 * or effects; the matching edge colours running from tile to tile are the
 * whole show. */
static void scene_draw(const Screen *sc, const Scene *s)
{
    const WangGrid *g = &s->grid;
    for (int ty = 0; ty < g->h; ty++)
        for (int tx = 0; tx < g->w; tx++)
            draw_one_tile(sc, &tile_set[g->cells[ty * g->w + tx]],
                          tx, ty, s->current_glyph);
}

/* Top row of the HUD: the title on the left, fps and tick rate on the right.
 * There's no paused/running flag because nothing is moving to pause. */
static void hud_draw_status_line(const Screen *sc, double fps, int sim_fps)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz ", fps, sim_fps);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);            /* pushed to the right edge */
    mvprintw(0, 1, " WANG TILES ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Second HUD row: the current pattern, glyph style, theme, a little swatch of
 * the 8 theme shades, and the grid size. Each field prints, then `x` steps past
 * its fixed width to where the next one starts. */
static void hud_draw_param_line(const Screen *sc, const Scene *s)
{
    (void)sc;
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-7s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 18;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " glyph:%-5s ", glyph_set_name(s->current_glyph));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 14;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    /* Swatch — show all 8 theme shades, each in its own colour. */
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
    mvprintw(1, x, "  grid:%dx%d  tiles:%d ",
             s->grid.w, s->grid.h, s->grid.w * s->grid.h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row: the list of keys you can press. */
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
    scene_draw(sc, s);                      /* the tiling itself */
    hud_draw_status_line(sc, fps, sim_fps); /* top row    */
    hud_draw_param_line(sc, s);             /* second row */
    hud_draw_key_hints(sc);                 /* bottom row */
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §6 app — signals, resize, keys, and the main loop ── */

/*
 * App — the whole running program: the scene, the screen, and the few loop
 * variables the main loop and the signal handlers share.
 *   scene / screen — the picture and where it's drawn (see those types).
 *   sim_fps        — tick rate, raised/lowered with [ and ].
 *   running        — set to 0 to quit (also by Ctrl-C / kill).
 *   need_resize    — set when the terminal was resized; handled before the
 *                    next frame, which rebuilds the grid for the new size.
 * running and need_resize are written from signal handlers, which may only
 * safely touch a `volatile sig_atomic_t`.
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
    scene_resize_to(&app->scene, app->screen.grid_w, app->screen.grid_h);
    app->need_resize = 0;
}

/* Handle one keypress. Returns false only for quit. Some keys rebuild the grid
 * right here (reseed, change pattern); others just flip a display setting. */
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
        scene_regenerate(s);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)wrap_dec((int)s->current_pattern, N_PATTERNS);
        scene_regenerate(s);
        break;

    /* Glyph style is looks-only, so no rebuild — just redraw next frame. */
    case 'g':
        s->current_glyph = (GlyphSet)wrap_inc((int)s->current_glyph, N_GLYPH_SETS);
        break;
    case 'G':
        s->current_glyph = (GlyphSet)wrap_dec((int)s->current_glyph, N_GLYPH_SETS);
        break;

    default: break;
    }
    return true;
}

/* One-time startup: seed the RNG, wire up the signal handlers and the cleanup
 * hook, set the loop's starting values, open the screen and build the first
 * tiling. Everything that happens before the loop. */
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
    scene_init(&app->scene, app->screen.grid_w, app->screen.grid_h);
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

        /* If the terminal was resized, rebuild for the new size first. */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* How long since last frame? Clamp a big stall so we don't try to
         * catch up with a flood of ticks. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > max_dt_ns) dt = max_dt_ns;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        /* Run the fixed-rate ticks that fit in the time elapsed. (They only
         * move the clock here — nothing in the picture changes.) */
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* Update the fps reading every so often, then sleep to the frame cap. */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(frame_cap_ns - elapsed);

        /* Draw the tiling and the HUD. */
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* Take one keypress, if any. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
