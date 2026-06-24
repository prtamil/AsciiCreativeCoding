/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * voronoi_region_map.c — a Voronoi map, drawn as a growing animation.
 *
 * Scatter a few seed points, then colour every cell by whichever seed is
 * closest. That carve-up is a Voronoi diagram. Here the cells reveal
 * closest-first, so coloured waves seem to ripple out from every seed at
 * once and meet at the borders. Then it holds, flashes white, and restarts.
 *
 * Companion file: ./poission_disk_sampling_showcase.c — its evenly-spaced
 * points make a good input for this. References for the math live in
 * documentation/Reference.md (Aurenhammer 1991; de Berg et al. ch. 7).
 *
 * Keys: q/ESC quit · space pause · r reset · t/T theme · +/- reveal speed ·
 *       ]/[ tick rate.  Build: see CLAUDE.md (needs -lncurses -lm).
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

/* ── §1  config ──────────────────────────────────────────────────────── */

enum {
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    CELLS_MAX         = MAP_W_MAX * MAP_H_MAX,

    /* Below this the map is too cramped to read; the terminal size is
     * clamped into [MIN, MAX] before we build a map. */
    MAP_W_MIN         =  16,
    MAP_H_MIN         =   8,

    /* One seed makes one region, and each region gets one palette colour,
     * so this is also the palette size. 8 looks like a real map without
     * neighbouring regions blurring together. */
    N_SEEDS           =   8,

    /* When dropping a seed, keep it at least this many cells from every
     * seed already placed, so two seeds don't land almost on top of each
     * other and pinch out a sliver region. */
    MIN_SEED_DIST     =  10,
    SEED_PLACE_TRIES  = 100,

    PLACE_HOLD_TICKS  =  18,        /* gap between seed drops, so you see them appear one by one */

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    OPS_PER_TICK_MIN  =   1,        /* cells revealed each tick — the reveal speed */
    OPS_PER_TICK_DEF  =  64,
    OPS_PER_TICK_MAX  = 4096,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* The HUD eats two rows up top (title + params) and one hint row at the
     * bottom; the map must leave all three clear. */
    HUD_TOP_ROWS      =   2,
    HUD_ROWS          =   3,

    /* Colour-pair slots. HUD/HINT are the standard reserved pair (see
     * CLAUDE.md). Then 8 region colours, plus three accents. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_REGION_BASE  =   3,        /* PAIR_REGION_BASE..+7 = the 8 regions */
    PAIR_SEED         =  11,        /* the '@' seed marker */
    PAIR_FLASH        =  12,        /* the '*' just-revealed flash */
    PAIR_SUPERNOVA    =  13,        /* the white reset flash */
};

/* How fast the per-cell flashes fade. Bigger = quicker. */
#define WAVE_GLOW_DECAY     2.5f    /* the just-revealed flash, lasts ~0.7 s */
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f   /* below this a flash counts as gone */

#define HOLD_SECONDS        2.5f    /* how long the finished map sits still */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Never draw faster than this, no matter how high the sim tick rate goes. */
#define RENDER_CAP_FPS  60
/* If one frame stalls badly (debugger, swap), pretend no more than this much
 * time passed — otherwise the sim tries to "catch up" forever and locks up. */
#define MAX_FRAME_NS  (100 * NS_PER_MS)

/*
 * Theme — one named colour scheme for the whole map.
 *
 * A cell only knows which seed won; the colour is the only thing that makes
 * the regions visible. So a theme bundles a name and every colour it uses,
 * which lets t/T swap the entire look in one go. All values are xterm-256
 * colour indices.
 *   name     — what t/T shows in the HUD.
 *   region[] — one colour per region. Exactly N_SEEDS of them: one colour
 *              per seed. (With more seeds than colours the renderer would
 *              wrap around and two regions would share a colour, which is
 *              why N_SEEDS is tied to this size.) DEFAULT picks far-apart
 *              hues so touching regions stand out; the rest are gradients
 *              (OCEAN runs navy to cyan) that look nice but contrast less.
 *   seed_fg  — colour of the '@' seed marker. Usually bright white so the
 *              seed shows up over any region colour beneath it.
 *   flash_fg — colour of the '*' that pops when a cell first reveals,
 *              before it settles into its region colour.
 * The HUD/hint/reset-flash colours are NOT here — they're fixed in
 * color_init() so the status line looks the same in every theme.
 */
typedef struct {
    const char *name;
    short       region[N_SEEDS];    /* one colour per region */
    short       seed_fg;            /* the '@' marker */
    short       flash_fg;           /* the '*' just-revealed flash */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* DEFAULT: rainbow mix that maximises adjacent-region distinction */
    { "DEFAULT", {  33,  67, 132, 165, 220,  34, 178, 244 }, 231, 226 },
    /* MATRIX: shades of green */
    { "MATRIX",  {  22,  28,  34,  40,  46,  82, 118, 154 }, 231, 226 },
    /* NOVA: purple to magenta to pink */
    { "NOVA",    {  53,  92, 129, 165, 201, 213, 219, 225 }, 231, 226 },
    /* MONO: greyscale gradient */
    { "MONO",    { 236, 240, 244, 247, 250, 252, 254, 255 }, 226, 226 },
    /* OCEAN: navy to bright cyan */
    { "OCEAN",   {  17,  18,  20,  27,  33,  39,  51, 117 }, 231, 226 },
    /* FIRE: dark red to yellow */
    { "FIRE",    {  52,  88, 124, 160, 196, 208, 220, 226 }, 231, 196 },
    /* EARTH: dark brown to cream */
    { "EARTH",   {  58,  94, 100, 137, 173, 215, 222, 229 }, 231, 226 },
    /* FOREST: dark green to tan */
    { "FOREST",  {  22,  28,  64, 100, 130, 144, 178, 187 }, 231, 226 },
    /* DESERT: brown to sand */
    { "DESERT",  {  94, 130, 137, 173, 215, 222, 229, 230 }, 231, 226 },
    /* ARCTIC: navy to white */
    { "ARCTIC",  {  17,  18,  24,  39,  51, 159, 195, 231 }, 226, 226 },
};

/* ── §2  performance ─────────────────────────────────────────────────── *
 * Just a clock and a sleep. The frame pacing that uses them is in main. */

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

/* ── §3  simulation — data structures ────────────────────────────────── *
 * The types the simulation reads and changes. The behaviour is in §5. */

/*
 * Seed — one of the scattered points the map is built around. (The Voronoi
 * literature calls these "sites" or "generators".) The whole input is just
 * this set of points: N_SEEDS of them carve the map into N_SEEDS regions.
 *   x, y — position in cells. Whole cells only: the output is one character
 *          per cell, so a fractional position couldn't move a border anyway.
 */
typedef struct {
    int x, y;
} Seed;

/*
 * CellOrd — one cell paired with how far it is from its owner, used only to
 * decide the reveal order.
 *
 * The map is computed all at once, but we reveal it closest-first so the
 * colour looks like it's rippling out from each seed. To do that we list
 * every cell, sort by distance, and unveil in that order. This struct is the
 * sortable record. It's purely about the animation — the finished map is the
 * same whatever order cells appear in.
 *   idx   — flat index (y*w + x) into owner[] / dist2[] / revealed[].
 *   dist2 — squared distance to the owner seed. Squared instead of the real
 *           distance because that keeps the same ordering while skipping a
 *           square root per cell and staying an exact integer (ties are fine,
 *           the reveal doesn't care which equal cell comes first).
 */
typedef struct {
    int idx;
    int dist2;
} CellOrd;

/*
 * Voronoi — the whole map: the seeds, the per-cell "who's closest" answer,
 * and the bookkeeping the growing reveal needs.
 *
 * The answer is found the simplest possible way: for every cell, check every
 * seed and keep the nearest. That's W*H*N work — slower-on-paper than the
 * clever sweep algorithms, but at this size (~11K cells, 8 seeds) it's plenty
 * fast and far easier to read. We store the result per cell (not as polygon
 * outlines), which is exactly what lets the reveal light up cells one by one.
 *
 *   THE ANSWER (who owns each cell)
 *   owner[]         : which seed won this cell, 0..N_SEEDS-1
 *   dist2[]         : squared distance from this cell to its owner seed
 *
 *   THE INPUTS (the seeds)
 *   seeds[]         : seed positions, filled in one at a time during PLACING
 *   n_seeds         : how many are down so far (0 up to N_SEEDS)
 *
 *   THE REVEAL (unveiling the answer closest-first)
 *   cell_order[]    : every cell sorted by distance, nearest first
 *   reveal_progress : how far down that list we've revealed
 *   revealed[]      : has this cell been shown yet?
 *   computed        : are owner[]/dist2[]/cell_order[] filled in yet?
 *   place_cooldown  : ticks left before the next seed drops, during PLACING
 *
 *   THE EYE CANDY (flashes — purely cosmetic, never affect the map)
 *   wave_glow[]      : brief flash when a cell first reveals, fades to 0
 *   supernova_glow[] : the white flash on reset
 *   They live here because there's one per cell and they index just like
 *   owner[], so they ride along with the grid instead of in their own type.
 *
 *   w, h, total_cells : grid size — what every array above is sized to.
 */
typedef struct {
    int     w, h;
    int     total_cells;
    int8_t  owner[CELLS_MAX];
    int     dist2[CELLS_MAX];
    float   wave_glow    [CELLS_MAX];
    float   supernova_glow[CELLS_MAX];
    bool    revealed[CELLS_MAX];

    Seed    seeds[N_SEEDS];
    int     n_seeds;

    CellOrd cell_order[CELLS_MAX];
    int     reveal_progress;

    bool    computed;
    int     place_cooldown;
} Voronoi;

/*
 * SceneState — which of the three repeating beats the demo is in. Listed in
 * the order they happen, then it loops:
 *   SCENE_PLACING   — seeds drop in one at a time (the inputs appear).
 *   SCENE_REVEALING — the finished map unveils, closest cells first.
 *   SCENE_HOLD      — the map sits still for a moment, then resets.
 */
typedef enum {
    SCENE_PLACING   = 0,
    SCENE_REVEALING = 1,
    SCENE_HOLD      = 2,
} SceneState;

/*
 * Scene — everything the running demo holds: the map, which beat it's on,
 * and the few things the user can tweak.
 */
typedef struct {
    Voronoi     v;              /* the map being built */
    SceneState  state;          /* which beat: PLACING / REVEALING / HOLD */
    float       hold_timer;     /* seconds left before HOLD resets */
    bool        paused;         /* space-bar freeze */
    int         ops_per_tick;   /* cells revealed per tick (reveal speed) */
    int         current_theme;  /* which palette in themes[] */
} Scene;

/* ── §4  logic ───────────────────────────────────────────────────────── *
 * Pure look-ups: they only read their arguments, so nothing else in the
 * frame can change what they return. */

static inline int v_idx(const Voronoi *v, int x, int y) { return y * v->w + x; }

/* How far apart two cells are, squared. We square instead of taking the real
 * distance because we only ever compare these — same ordering, no sqrt. */
static inline int dist2_cells(int ax, int ay, int bx, int by)
{
    int dx = ax - bx, dy = ay - by;
    return dx * dx + dy * dy;
}

/* Would a seed at (x,y) crowd one already placed? (Compares squared
 * distances so there's no sqrt.) */
static bool seed_too_close(const Voronoi *v, int x, int y)
{
    for (int i = 0; i < v->n_seeds; i++)
        if (dist2_cells(x, y, v->seeds[i].x, v->seeds[i].y)
                < MIN_SEED_DIST * MIN_SEED_DIST)
            return true;
    return false;
}

/* The whole Voronoi rule in one function: find the seed nearest to (x,y).
 * Returns that seed's index and hands back its squared distance via *out_d2.
 * Doesn't touch the map — the caller stores the result. */
static int nearest_seed(const Voronoi *v, int x, int y, int *out_d2)
{
    int best_d2 = -1, best_i = 0;
    for (int i = 0; i < v->n_seeds; i++) {
        int d2 = dist2_cells(x, y, v->seeds[i].x, v->seeds[i].y);
        if (best_d2 < 0 || d2 < best_d2) {
            best_d2 = d2;
            best_i  = i;
        }
    }
    *out_d2 = best_d2;
    return best_i;
}

/* Sort order for qsort: nearer cells (smaller dist2) come first. */
static int cmp_cellord(const void *a, const void *b)
{
    const CellOrd *ca = (const CellOrd *)a;
    const CellOrd *cb = (const CellOrd *)b;
    if (ca->dist2 < cb->dist2) return -1;
    if (ca->dist2 > cb->dist2) return  1;
    return 0;
}

/* ── §5  simulation ──────────────────────────────────────────────────── *
 * Where the state actually moves: dropping seeds, computing the map, the
 * reveal, and scene_tick — the one place a tick advances everything. */

/* Drop one seed at a random spot, trying to keep its distance from the
 * others. Returns false if the map is already full of seeds. On a tiny map
 * where no roomy spot turns up in SEED_PLACE_TRIES tries, we just take the
 * last try anyway — slightly crowded beats not placing one. */
static bool v_place_one_seed(Voronoi *v)
{
    if (v->n_seeds >= N_SEEDS) return false;

    int best_x = 0, best_y = 0;
    for (int attempt = 0; attempt < SEED_PLACE_TRIES; attempt++) {
        best_x = rand() % v->w;
        best_y = rand() % v->h;
        if (!seed_too_close(v, best_x, best_y)) break;
    }

    v->seeds[v->n_seeds].x = best_x;
    v->seeds[v->n_seeds].y = best_y;
    v->n_seeds++;
    return true;
}

/* The actual map: for every cell, record its nearest seed and how far it is.
 * Run once, when the last seed lands and we move into REVEALING. */
static void v_compute_distances(Voronoi *v)
{
    for (int y = 0; y < v->h; y++) {
        for (int x = 0; x < v->w; x++) {
            int idx = v_idx(v, x, y);
            int d2;
            v->owner[idx] = (int8_t)nearest_seed(v, x, y, &d2);
            v->dist2[idx] = d2;
        }
    }
}

/* List every cell with its distance, then sort nearest-first — that sorted
 * order is what makes the colour ripple outward from each seed. */
static void build_reveal_order(Voronoi *v)
{
    int n = v->total_cells;
    for (int i = 0; i < n; i++) {
        v->cell_order[i].idx   = i;
        v->cell_order[i].dist2 = v->dist2[i];
    }
    qsort(v->cell_order, n, sizeof(CellOrd), cmp_cellord);
}

/* Build the map and its reveal order in one shot, then arm the reveal. */
static void v_compute_full(Voronoi *v)
{
    v_compute_distances(v);
    build_reveal_order(v);
    v->reveal_progress = 0;
    v->computed = true;
}

/* Reveal the next cell in line. Returns false once every cell is shown. */
static bool v_reveal_step(Voronoi *v)
{
    if (v->reveal_progress >= v->total_cells) return false;
    int idx = v->cell_order[v->reveal_progress++].idx;
    v->revealed[idx]  = true;
    v->wave_glow[idx] = 1.0f;
    return true;
}

/* Wipe the map clean and size it to w*h, ready to start placing seeds again. */
static void v_reset(Voronoi *v, int w, int h)
{
    v->w = w;
    v->h = h;
    v->total_cells = w * h;
    v->n_seeds = 0;
    v->reveal_progress = 0;
    v->computed = false;
    v->place_cooldown = 0;

    for (int i = 0; i < v->total_cells; i++) {
        v->owner[i]          = -1;
        v->dist2[i]          = 0;
        v->wave_glow[i]      = 0.0f;
        v->supernova_glow[i] = 1.0f;
        v->revealed[i]       = false;
    }
}

static void scene_reset(Scene *s, int mw, int mh)
{
    v_reset(&s->v, mw, mh);
    s->state      = SCENE_PLACING;
    s->hold_timer = 0.0f;
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused        = false;
    s->ops_per_tick  = OPS_PER_TICK_DEF;
    s->current_theme = 0;
    scene_reset(s, mw, mh);
}

/* Fade every per-cell flash a little this tick. */
static void decay_glows(Voronoi *v, float dt)
{
    float wave_d = expf(-WAVE_GLOW_DECAY * dt);
    float nova_d = expf(-SUPERNOVA_DECAY * dt);
    for (int i = 0; i < v->total_cells; i++) {
        v->wave_glow[i]      *= wave_d;
        v->supernova_glow[i] *= nova_d;
    }
}

/* PLACING beat: drop a seed each time the cooldown runs out; once the last
 * one's down, build the map and hand off to REVEALING. */
static void advance_placing(Scene *s)
{
    if (s->v.place_cooldown > 0) {
        s->v.place_cooldown--;
    } else {
        v_place_one_seed(&s->v);
        s->v.place_cooldown = PLACE_HOLD_TICKS;
        if (s->v.n_seeds >= N_SEEDS) {
            v_compute_full(&s->v);
            s->state = SCENE_REVEALING;
        }
    }
}

/* REVEALING beat: show up to ops_per_tick more cells; when none are left,
 * start the hold timer and hand off to HOLD. */
static void advance_revealing(Scene *s)
{
    for (int i = 0; i < s->ops_per_tick; i++) {
        if (!v_reveal_step(&s->v)) {
            s->state      = SCENE_HOLD;
            s->hold_timer = HOLD_SECONDS;
            break;
        }
    }
}

/* HOLD beat: let the finished map sit, then start over. */
static void advance_hold(Scene *s, float dt)
{
    s->hold_timer -= dt;
    if (s->hold_timer <= 0.0f)
        scene_reset(s, s->v.w, s->v.h);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    decay_glows(&s->v, dt);                 /* fade the flashes */

    switch (s->state) {                     /* then move the current beat along */
    case SCENE_PLACING:   advance_placing(s);    break;
    case SCENE_REVEALING: advance_revealing(s);  break;
    case SCENE_HOLD:      advance_hold(s, dt);   break;
    }
}

/* ── §6  render ──────────────────────────────────────────────────────── *
 * Turns the state into pixels. Only reads the sim; the only things it
 * writes are the terminal and the Screen size. */

/* Switch to a named palette: reload the 8 region colours plus the seed and
 * flash colours. The HUD and reset-flash colours are fixed elsewhere. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < N_SEEDS; i++)
            init_pair(PAIR_REGION_BASE + i, t->region[i], -1);
        init_pair(PAIR_SEED,  t->seed_fg,  -1);
        init_pair(PAIR_FLASH, t->flash_fg, -1);
    } else {
        /* 8-colour fallback. */
        static const short fallback[N_SEEDS] = {
            COLOR_BLUE,  COLOR_CYAN,    COLOR_GREEN,  COLOR_YELLOW,
            COLOR_MAGENTA, COLOR_RED,   COLOR_WHITE,  COLOR_BLUE,
        };
        for (int i = 0; i < N_SEEDS; i++)
            init_pair(PAIR_REGION_BASE + i, fallback[i], -1);
        init_pair(PAIR_SEED,  COLOR_WHITE,  -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
        init_pair(PAIR_SUPERNOVA,  226, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
        init_pair(PAIR_SUPERNOVA, COLOR_YELLOW,  -1);
    }
    theme_apply(0);
}

/*
 * Screen — just the terminal's current size in cells. Kept as its own tiny
 * type so the draw functions can take a Screen* and never need the whole App.
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

/* Where the map's top-left corner lands: centred in the terminal, but left of
 * the HUD rows (two on top, one hint at the bottom). */
static void map_screen_origin(const Voronoi *v, int cols, int rows,
                              int *gx0, int *gy0)
{
    int x0 = (cols - v->w) / 2;
    int y0 = ((rows - HUD_ROWS) - v->h) / 2 + HUD_TOP_ROWS;
    if (x0 < 0)            x0 = 0;
    if (y0 < HUD_TOP_ROWS) y0 = HUD_TOP_ROWS;
    *gx0 = x0;
    *gy0 = y0;
}

/* Decide how one cell looks right now. A live flash wins over the plain
 * region colour. Returns false if the cell should stay blank. */
static bool cell_appearance(const Voronoi *v, int idx,
                            int *pair, int *attr, char *glyph)
{
    if (v->supernova_glow[idx] > GLOW_THRESHOLD) {        /* reset flash */
        *pair = PAIR_SUPERNOVA; *attr = A_BOLD; *glyph = '*';
    } else if (v->wave_glow[idx] > GLOW_THRESHOLD) {      /* just-revealed flash */
        *pair = PAIR_FLASH;     *attr = A_BOLD; *glyph = '*';
    } else if (v->revealed[idx]) {                        /* settled region colour */
        int8_t owner = v->owner[idx];
        if (owner < 0) return false;
        *pair = PAIR_REGION_BASE + (owner % N_SEEDS); *attr = A_NORMAL; *glyph = '#';
    } else {
        return false;                                     /* not revealed yet */
    }
    return true;
}

/* Draw every visible region cell in whatever look cell_appearance gives it. */
static void draw_region_cells(const Voronoi *v, int gx0, int gy0,
                              int cols, int rows)
{
    for (int y = 0; y < v->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < v->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            int pair, attr; char glyph;
            if (!cell_appearance(v, v_idx(v, x, y), &pair, &attr, &glyph))
                continue;

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Stamp the seed markers on top: a bold '@' in each seed's own region colour. */
static void draw_seeds(const Voronoi *v, int gx0, int gy0, int cols, int rows)
{
    for (int i = 0; i < v->n_seeds; i++) {
        int sx = gx0 + v->seeds[i].x;
        int sy = gy0 + v->seeds[i].y;
        if (sx < 0 || sx >= cols) continue;
        if (sy < 0 || sy >= rows) continue;
        int pair = PAIR_REGION_BASE + (i % N_SEEDS);
        attron(COLOR_PAIR(pair) | A_BOLD);
        mvaddch(sy, sx, (chtype)(unsigned char)'@');
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    const Voronoi *v = &s->v;
    int gx0, gy0;
    map_screen_origin(v, cols, rows, &gx0, &gy0);
    draw_region_cells(v, gx0, gy0, cols, rows);   /* regions and flashes */
    draw_seeds(v, gx0, gy0, cols, rows);          /* seed markers on top */
}

/* The current beat as a same-width label for the HUD (kept padded so the line
 * doesn't jiggle as the word changes). */
static const char *state_label(const Scene *s)
{
    if (s->paused)                   return "PAUSED   ";
    if (s->state == SCENE_PLACING)   return "PLACING  ";
    if (s->state == SCENE_REVEALING) return "REVEALING";
    return "HOLD     ";
}

/* The little colour key in the HUD: one bold '#' per region, left to right. */
static void draw_palette_swatches(int row, int x)
{
    for (int i = 0; i < N_SEEDS; i++) {
        int p = PAIR_REGION_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(row, x + i, '#');
        attroff(COLOR_PAIR(p) | A_BOLD);
    }
}

static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Voronoi *v = &s->v;
    const char *state_str = state_label(s);

    int reveal_pct = (v->total_cells > 0)
                   ? (100 * v->reveal_progress / v->total_cells)
                   : 0;

    /* top-right: the live status line */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  ops:%-3d  %s  seeds:%d/%d  %3d%% ",
             fps, sim_fps, s->ops_per_tick, state_str,
             v->n_seeds, N_SEEDS, reveal_pct);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* top-left: the title */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " VORONOI REGION MAP ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* second row: theme name, the colour key, and a couple of read-outs */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 9;
    draw_palette_swatches(1, x);
    x += N_SEEDS;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  metric:euclid  map:%dx%d ", v->w, v->h);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* bottom row: the key reminder */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " #:region  @:seed  *:flash | t/T:theme  r:reset  spc:pause  +/-:speed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §7  app ─────────────────────────────────────────────────────────── *
 * Glue: holds the run settings and signal flags, handles resize and keys
 * (which happen outside a tick), and runs the main loop. */

/*
 * App — the whole running program in one place: the animation, the terminal
 * it draws to, and the loop state. Separate from Scene so the draw code only
 * needs a Screen, and so the signal handler can reach the run flags through
 * the one global g_app.
 *   scene, screen      : the simulation and where it draws.
 *   sim_fps            : how many ticks per second (paces the loop; not the
 *                        same as ops_per_tick, which paces work inside a tick).
 *   map_w, map_h       : the chosen map size, re-applied on reset and resize.
 *   running/need_resize: set by signal handlers, so they're sig_atomic_t.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   map_w, map_h;
    volatile sig_atomic_t running;      /* cleared to quit */
    volatile sig_atomic_t need_resize;  /* set by SIGWINCH */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_ROWS;   /* keep room for the HUD */
    if (mw < MAP_W_MIN) mw = MAP_W_MIN;
    if (mh < MAP_H_MIN) mh = MAP_H_MIN;
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

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s, app->map_w, app->map_h);
        break;
    case '=': case '+':
        if (s->ops_per_tick < OPS_PER_TICK_MAX) s->ops_per_tick *= 2;
        if (s->ops_per_tick > OPS_PER_TICK_MAX) s->ops_per_tick = OPS_PER_TICK_MAX;
        break;
    case '-':
        s->ops_per_tick /= 2;
        if (s->ops_per_tick < OPS_PER_TICK_MIN) s->ops_per_tick = OPS_PER_TICK_MIN;
        break;
    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

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

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > MAX_FRAME_NS) dt = MAX_FRAME_NS;   /* don't try to catch up on a long stall */

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* fps read-out, averaged over each half-second */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* wait out the rest of the frame so we don't draw too fast */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(TICK_NS(RENDER_CAP_FPS) - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
