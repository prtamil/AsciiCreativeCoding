/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * matrix_snowflake.c -- Matrix rain + live DLA snowflake crystal growth
 *
 * Two real simulations sharing one screen:
 *
 *   BACKGROUND  Matrix rain   -- streams of random ASCII chars fall
 *               through the entire terminal (classic digital-rain look).
 *               Characters flicker and change as the head passes.
 *
 *   FOREGROUND  DLA crystal   -- a fractal snowflake grows from the
 *               center using Diffusion-Limited Aggregation with full
 *               D6 hexagonal symmetry (6 rotations x 2 reflections = 12
 *               positions frozen simultaneously per walker stick event).
 *               Frozen cells render on top of the rain behind them.
 *
 * When the crystal fills the screen it flashes bright white, dissolves
 * back into pure rain, and a new crystal begins growing from zero.
 * The cycle repeats indefinitely.
 *
 * Keys:
 *   q / ESC   quit
 *   r         reset crystal now
 *   + =       more walkers  (grow faster)
 *   -         fewer walkers (grow slower)
 *   ] [       rain faster / slower
 *   t         cycle theme (5 themes)
 *   p / spc   pause / resume
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra matrix_rain/matrix_snowflake.c \
 *       -o matrix_snowflake -lncurses -lm
 *
 * Sections:
 *   [1] config  [2] clock  [3] color  [4] crystal
 *   [5] rain    [6] walkers [7] scene  [8] app
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

/* ===================================================================== */
/* [1] config                                                             */
/* ===================================================================== */

#define RENDER_FPS      30
#define RENDER_NS       (1000000000LL / RENDER_FPS)

#define ROWS_MAX        80
#define COLS_MAX        300

#define WALKER_MAX      300
#define WALKER_DEFAULT  12

/*
 * STICK_PROB: probability a walker sticks on contact.
 * 0.35 = thicker, rounder arms (walkers bounce many times before freezing).
 * 0.90 = sparse spiky DLA fractal (sticks almost immediately on contact).
 */
#define STICK_PROB      0.35f

/*
 * ASPECT_R: terminal cell height / width (~2.0 for typical monospace fonts).
 * Applied when converting terminal (col, row) to Euclidean (x, y) space
 * for D6 rotation and distance calculations.
 */
#define ASPECT_R        2.0f

/* Frames the flash state lasts before crystal resets */
#define FLASH_FRAMES    28

/* Rain character set */
static const char k_rain_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
    "0123456789!#$%&*+-<>=?@^~|";
#define N_RAIN_CHARS ((int)(sizeof k_rain_chars - 1))

/* Color pair IDs */
enum {
    CP_RAIN_0 = 1,   /* rain head — brightest                */
    CP_RAIN_1,       /* rain trail near                      */
    CP_RAIN_2,       /* rain trail far                       */
    CP_RAIN_3,       /* rain fade                            */
    CP_XTAL_1,       /* crystal tip  (outermost, brightest)  */
    CP_XTAL_2,
    CP_XTAL_3,
    CP_XTAL_4,
    CP_XTAL_5,
    CP_XTAL_6,       /* crystal core (innermost, darkest)    */
    CP_HUD,
    N_CP
};

#define N_XTAL_COLORS   6   /* must match CP_XTAL_1 .. CP_XTAL_6 span */
#define N_THEMES        5

typedef struct {
    const char *name;
    /* rain: [head, near, far, fade] — 256-color then 8-color fallback */
    int rain[4];
    int rain8[4];
    /* crystal: [tip, ..., core] x6 — 256-color then 8-color */
    int xtal[6];
    int xtal8[6];
} Theme;

/*
 * Each theme pairs a cool rain hue with a contrasting crystal palette so
 * the growing snowflake is always visually distinct from the rain.
 */
static const Theme k_themes[N_THEMES] = {
    { "Classic",
      /* rain: green */
      {  46,  40,  34,  28 },
      { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN },
      /* crystal: white -> icy teal -> ocean blue */
      { 231, 195,  51,  44,  38, 117 },
      { COLOR_WHITE, COLOR_WHITE, COLOR_CYAN, COLOR_CYAN,
        COLOR_BLUE, COLOR_BLUE } },

    { "Inferno",
      /* rain: red */
      { 196, 160, 124,  88 },
      { COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED },
      /* crystal: white -> yellow -> gold -> orange */
      { 231, 226, 220, 214, 208, 202 },
      { COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
        COLOR_RED, COLOR_RED } },

    { "Nebula",
      /* rain: purple */
      { 201, 165, 129,  93 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA },
      /* crystal: white -> pale cyan -> cyan -> teal */
      { 231, 159, 123,  87,  51,  45 },
      { COLOR_WHITE, COLOR_WHITE, COLOR_CYAN, COLOR_CYAN,
        COLOR_CYAN, COLOR_BLUE } },

    { "Toxic",
      /* rain: cyan */
      {  51,  45,  39,  30 },
      { COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE },
      /* crystal: white -> pink -> magenta -> deep violet */
      { 231, 219, 213, 207, 201, 165 },
      { COLOR_WHITE, COLOR_WHITE, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_MAGENTA, COLOR_MAGENTA } },

    { "Gold",
      /* rain: yellow */
      { 226, 220, 214, 178 },
      { COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW },
      /* crystal: white -> lavender -> violet -> deep purple */
      { 231, 183, 141,  99,  57,  55 },
      { COLOR_WHITE, COLOR_WHITE, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_MAGENTA, COLOR_MAGENTA } },
};

static int g_theme = 0;

/* ===================================================================== */
/* [2] clock                                                              */
/* ===================================================================== */

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

/* ===================================================================== */
/* [3] color                                                              */
/* ===================================================================== */

static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    if (COLORS >= 256) {
        init_pair(CP_RAIN_0, th->rain[0], -1);
        init_pair(CP_RAIN_1, th->rain[1], -1);
        init_pair(CP_RAIN_2, th->rain[2], -1);
        init_pair(CP_RAIN_3, th->rain[3], -1);
        for (int i = 0; i < N_XTAL_COLORS; i++)
            init_pair(CP_XTAL_1 + i, th->xtal[i], COLOR_BLACK);
        init_pair(CP_HUD, th->rain[0], -1);
    } else {
        init_pair(CP_RAIN_0, th->rain8[0], -1);
        init_pair(CP_RAIN_1, th->rain8[1], -1);
        init_pair(CP_RAIN_2, th->rain8[2], -1);
        init_pair(CP_RAIN_3, th->rain8[3], -1);
        for (int i = 0; i < N_XTAL_COLORS; i++)
            init_pair(CP_XTAL_1 + i, th->xtal8[i], COLOR_BLACK);
        init_pair(CP_HUD, th->rain8[0], -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    theme_apply(g_theme);
}

/* ===================================================================== */
/* [4] crystal -- DLA with D6 (hexagonal) symmetry                       */
/* ===================================================================== */

/*
 * D6 rotation tables.
 *
 *   CA6[k] = cos(k * 60 deg)
 *   SA6[k] = sin(k * 60 deg)
 *
 * For each frozen cell, xtal_freeze_symmetric() applies all 12 elements of
 * the dihedral group D6 = { R^k, S*R^k : k=0..5 } where R is 60-degree
 * rotation and S is reflection.  Euclidean (x, y) coordinates are used for
 * the rotation; terminal (col, row) coords are scaled by ASPECT_R.
 */
static const float CA6[6] = {
     1.0f,  0.5f, -0.5f,
    -1.0f, -0.5f,  0.5f,
};
static const float SA6[6] = {
     0.0f,  0.8660254f,  0.8660254f,
     0.0f, -0.8660254f, -0.8660254f,
};

typedef struct {
    uint8_t cells[ROWS_MAX][COLS_MAX];  /* 0 = empty, n = color pair id   */
    int     frozen_count;
    int     cols, rows;                 /* grid dimensions (= terminal)    */
    int     cx0,  cy0;                  /* terminal center column/row      */
    float   max_dist;                   /* normalisation radius            */
    float   trigger_dist;               /* reset threshold                 */
    float   max_frozen_dist;            /* farthest frozen cell so far     */
} Crystal;

static bool xtal_in(const Crystal *x, int c, int r)
{
    return c >= 0 && c < x->cols && r >= 0 && r < x->rows;
}

static bool xtal_frozen(const Crystal *x, int c, int r)
{
    return xtal_in(x, c, r) && x->cells[r][c] != 0;
}

static bool xtal_adj_frozen(const Crystal *x, int c, int r)
{
    return xtal_frozen(x, c-1, r) || xtal_frozen(x, c+1, r)
        || xtal_frozen(x, c, r-1) || xtal_frozen(x, c, r+1);
}

static void xtal_freeze_one(Crystal *x, int c, int r)
{
    if (!xtal_in(x, c, r) || x->cells[r][c] != 0) return;

    float dx_e = (float)(c - x->cx0);
    float dy_e = (float)(r - x->cy0) / ASPECT_R;
    float dist = sqrtf(dx_e * dx_e + dy_e * dy_e);

    /* Map distance to color band: near tips = CP_XTAL_1 (bright),
     * near core = CP_XTAL_6 (dim).                                    */
    int band = (int)(dist / x->max_dist * (float)N_XTAL_COLORS);
    if (band >= N_XTAL_COLORS) band = N_XTAL_COLORS - 1;
    int cp = CP_XTAL_1 + band;

    x->cells[r][c] = (uint8_t)cp;
    x->frozen_count++;
    if (dist > x->max_frozen_dist) x->max_frozen_dist = dist;
}

static void xtal_freeze_symmetric(Crystal *x, int c, int r)
{
    float dx = (float)(c - x->cx0);
    float dy = (float)(r - x->cy0);

    for (int refl = 0; refl < 2; refl++) {
        float dx_e =  dx;
        float dy_e = (refl == 0 ? dy : -dy) / ASPECT_R;  /* Euclidean y */

        for (int k = 0; k < 6; k++) {
            float rx_e = dx_e * CA6[k] - dy_e * SA6[k];
            float ry_e = dx_e * SA6[k] + dy_e * CA6[k];

            int nc = x->cx0 + (int)roundf(rx_e);
            int nr = x->cy0 + (int)roundf(ry_e * ASPECT_R);
            xtal_freeze_one(x, nc, nr);
        }
    }
}

/*
 * Choose a display character from the frozen-neighbor topology.
 * Cardinal + diagonal neighbors determine arm direction.
 */
static chtype xtal_char(const Crystal *x, int c, int r)
{
    bool N  = xtal_frozen(x, c,   r-1);
    bool S  = xtal_frozen(x, c,   r+1);
    bool E  = xtal_frozen(x, c+1, r  );
    bool W  = xtal_frozen(x, c-1, r  );
    bool NE = xtal_frozen(x, c+1, r-1);
    bool NW = xtal_frozen(x, c-1, r-1);
    bool SE = xtal_frozen(x, c+1, r+1);
    bool SW = xtal_frozen(x, c-1, r+1);

    int card = (int)N + (int)S + (int)E + (int)W;

    if (card == 0 && !(NE||NW||SE||SW)) return (chtype)'*';
    if (card >= 3)                       return (chtype)'#';
    if (N  && S  && !E && !W)           return (chtype)'|';
    if (E  && W  && !N && !S)           return (chtype)'=';
    if (card == 1) return (N||S) ? (chtype)'|' : (chtype)'=';
    if (card == 2)                       return (chtype)'#';
    if ((NE||SW) && !(NW||SE))          return (chtype)'/';
    if ((NW||SE) && !(NE||SW))          return (chtype)'\\';
    return (chtype)'#';
}

static void xtal_init(Crystal *x, int cols, int rows)
{
    if (cols > COLS_MAX) cols = COLS_MAX;
    if (rows > ROWS_MAX) rows = ROWS_MAX;
    memset(x->cells, 0, sizeof x->cells);
    x->frozen_count    = 0;
    x->cols = cols;  x->rows = rows;
    x->cx0  = cols / 2;
    x->cy0  = (rows - 1) / 2;  /* centre in usable area (excl. HUD) */

    /* Normalise to 90% of the half-diagonal so colour spans full range */
    x->max_dist       = hypotf((float)x->cx0,
                               (float)x->cy0 / ASPECT_R) * 0.90f;
    /* Reset when the outermost arm reaches 88% of max_dist */
    x->trigger_dist   = x->max_dist * 0.88f;
    x->max_frozen_dist = 0.0f;

    /* Seed: center + short horizontal stub to bootstrap D6 symmetry */
    xtal_freeze_one(x, x->cx0, x->cy0);
    xtal_freeze_symmetric(x, x->cx0 + 2, x->cy0);
}

/*
 * Draw the crystal in two passes.
 *
 * Pass 1: glow halo -- paint dim ':' on each empty 8-neighbor of every
 *         frozen cell.  Uses the frozen cell's own color so the glow
 *         matches the arm it surrounds.  This makes 1-cell-wide arms
 *         appear visually thicker without changing the DLA aggregate.
 *
 * Pass 2: frozen cells on top, with directional chars and bold for tips.
 *         In flash state every cell renders as bright '*'.
 */
static void xtal_draw(const Crystal *x, bool flashing)
{
    /* Pass 1: glow halo */
    for (int r = 0; r < x->rows - 1; r++) {
        for (int c = 0; c < x->cols; c++) {
            int cp = (int)x->cells[r][c];
            if (cp == 0) continue;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    int nr = r+dy, nc = c+dx;
                    if (!xtal_in(x, nc, nr) || nr >= x->rows-1) continue;
                    if (x->cells[nr][nc] != 0) continue;
                    attron(COLOR_PAIR(cp) | A_DIM);
                    mvaddch(nr, nc, ':');
                    attroff(COLOR_PAIR(cp) | A_DIM);
                }
            }
        }
    }

    /* Pass 2: frozen cells */
    for (int r = 0; r < x->rows - 1; r++) {
        for (int c = 0; c < x->cols; c++) {
            int cp = (int)x->cells[r][c];
            if (cp == 0) continue;

            if (flashing) {
                attron(COLOR_PAIR(CP_XTAL_1) | A_BOLD);
                mvaddch(r, c, '*');
                attroff(COLOR_PAIR(CP_XTAL_1) | A_BOLD);
            } else {
                /* Tips (band 0-1) and core (band 5) rendered bold */
                attr_t attr = COLOR_PAIR(cp);
                if (cp <= CP_XTAL_2 || cp == CP_XTAL_6) attr |= A_BOLD;
                attron(attr);
                mvaddch(r, c, xtal_char(x, c, r));
                attroff(attr);
            }
        }
    }
}

/* ===================================================================== */
/* [5] rain                                                               */
/* ===================================================================== */

/*
 * One stream per column.  Each stream has a floating-point head position
 * and a fixed-length trail.  Characters at each (row, col) are stored in
 * g_rain_ch and randomised near the head every tick.
 */
typedef struct {
    float head;   /* current head row (float for smooth sub-row speed) */
    int   trail;  /* trail length in rows                              */
    float speed;  /* rows / frame                                      */
    bool  active;
} RainStream;

static RainStream g_streams[COLS_MAX];
static char       g_rain_ch[ROWS_MAX][COLS_MAX];
static int        g_n_streams;
static float      g_speed_mult = 1.0f;

static int g_rows, g_cols;

static void rain_ch_randomize_all(void)
{
    for (int r = 0; r < g_rows; r++)
        for (int c = 0; c < g_cols; c++)
            g_rain_ch[r][c] = k_rain_chars[rand() % N_RAIN_CHARS];
}

static void rain_stream_spawn(int col)
{
    g_streams[col].head   = -(float)(rand() % 6);
    g_streams[col].trail  = 6 + rand() % 16;
    g_streams[col].speed  = 0.35f + (float)(rand() % 90) / 100.0f;
    g_streams[col].active = true;
}

static void rain_init(void)
{
    memset(g_streams, 0, sizeof g_streams);
    rain_ch_randomize_all();
    /* Pre-scatter: about 35% of columns active on first frame */
    for (int c = 0; c < g_cols; c++) {
        if (rand() % 3 == 0) {
            g_streams[c].head   = (float)(rand() % g_rows);
            g_streams[c].trail  = 6 + rand() % 16;
            g_streams[c].speed  = 0.35f + (float)(rand() % 90) / 100.0f;
            g_streams[c].active = true;
        }
    }
    g_n_streams = g_cols / 2;
}

static void rain_tick(void)
{
    int live = 0;

    for (int c = 0; c < g_cols; c++) {
        RainStream *s = &g_streams[c];
        if (!s->active) continue;
        s->head += s->speed * g_speed_mult;
        if ((int)s->head - s->trail >= g_rows) {
            s->active = false;
        } else {
            live++;
            /* Randomise chars at head and one row above: rapid flicker */
            int h = (int)s->head;
            for (int d = 0; d <= 2; d++) {
                int r = h - d;
                if (r >= 0 && r < g_rows) {
                    if (rand() % 3 != 0)  /* ~67% chance per cell */
                        g_rain_ch[r][c] = k_rain_chars[rand() % N_RAIN_CHARS];
                }
            }
        }
    }

    /* Ambient flicker: randomise ~4% of cells each tick */
    int n = (g_rows * g_cols * 4) / 100;
    for (int i = 0; i < n; i++)
        g_rain_ch[rand() % g_rows][rand() % g_cols] =
            k_rain_chars[rand() % N_RAIN_CHARS];

    /* Spawn new streams to maintain target density.
     * Pick columns randomly so streams are distributed across the full
     * width — a sequential scan would always fill left columns first. */
    int deficit = g_n_streams - live;
    for (int attempt = 0; attempt < deficit * 4 && live < g_n_streams; attempt++) {
        int c = rand() % g_cols;
        if (!g_streams[c].active) {
            rain_stream_spawn(c);
            live++;
        }
    }
}

static void rain_draw(const Crystal *x)
{
    for (int c = 0; c < g_cols; c++) {
        const RainStream *s = &g_streams[c];
        if (!s->active) continue;

        int h = (int)s->head;
        for (int d = 0; d < s->trail; d++) {
            int r = h - d;
            if (r < 0 || r >= g_rows - 1) continue;

            /* Crystal cells are drawn on top in xtal_draw; skip them here */
            if (x->cells[r][c] != 0) continue;

            int    cp;
            attr_t extra = 0;
            if      (d == 0)     { cp = CP_RAIN_0; extra = A_BOLD; }
            else if (d <= 3)     { cp = CP_RAIN_1; }
            else if (d <= 8)     { cp = CP_RAIN_2; }
            else                 { cp = CP_RAIN_3; extra = A_DIM; }

            attron(COLOR_PAIR(cp) | extra);
            mvaddch(r, c, (chtype)(unsigned char)g_rain_ch[r][c]);
            attroff(COLOR_PAIR(cp) | extra);
        }
    }
}

/* ===================================================================== */
/* [6] walkers                                                            */
/* ===================================================================== */

typedef struct {
    int  col, row;
    bool active;
} Walker;

static Walker g_walkers[WALKER_MAX];
static int    g_n_walkers;

/*
 * Spawn a walker on a circle just outside the current crystal boundary.
 * This is the standard DLA optimisation: walkers spawned at screen edges
 * take O(cols^2) steps to reach the tiny central seed -- spawning near the
 * aggregate makes them find it in O(1) steps and the crystal grows fast.
 */
static void walker_spawn(Walker *w, const Crystal *x)
{
    /* Spawn radius = current max arm + small buffer, capped inside grid */
    float spawn_r = x->max_frozen_dist + 8.0f;
    float max_r   = fminf((float)x->cx0,
                          (float)(x->rows - 2) * 0.5f * ASPECT_R) * 0.94f;
    if (spawn_r > max_r) spawn_r = max_r;
    if (spawn_r < 4.0f)  spawn_r = 4.0f;

    float angle = (float)rand() / (float)RAND_MAX * 6.2831853f;
    int nc = x->cx0 + (int)roundf(cosf(angle) * spawn_r);
    int nr = x->cy0 + (int)roundf(sinf(angle) * spawn_r / ASPECT_R);

    if (nc < 0)              nc = 0;
    if (nc >= x->cols)       nc = x->cols - 1;
    if (nr < 0)              nr = 0;
    if (nr >= x->rows - 1)  nr = x->rows - 2;

    w->col = nc;
    w->row = nr;
    w->active = true;
}

static void walkers_init(Crystal *x)
{
    for (int i = 0; i < WALKER_MAX; i++) g_walkers[i].active = false;
    for (int i = 0; i < g_n_walkers; i++) walker_spawn(&g_walkers[i], x);
}

static void walkers_tick(Crystal *x)
{
    static const int ddx[4] = {  0,  0, -1,  1 };
    static const int ddy[4] = { -1,  1,  0,  0 };

    for (int i = 0; i < g_n_walkers; i++) {
        Walker *w = &g_walkers[i];
        if (!w->active) continue;

        int dir = rand() % 4;
        int nc  = w->col + ddx[dir];
        int nr  = w->row + ddy[dir];

        /* Out of bounds -> respawn near crystal */
        if (nc < 0 || nc >= g_cols || nr < 0 || nr >= g_rows - 1) {
            walker_spawn(w, x);
            continue;
        }

        /* Bumped into frozen cell -> try sticking from current position */
        if (xtal_frozen(x, nc, nr)) {
            if (!xtal_frozen(x, w->col, w->row)) {
                if ((float)rand() / (float)RAND_MAX < STICK_PROB) {
                    xtal_freeze_symmetric(x, w->col, w->row);
                    walker_spawn(w, x);
                }
            }
            continue;
        }

        w->col = nc;
        w->row = nr;

        /* Moved to empty cell adjacent to crystal -> try sticking */
        if (xtal_adj_frozen(x, w->col, w->row)) {
            if ((float)rand() / (float)RAND_MAX < STICK_PROB) {
                xtal_freeze_symmetric(x, w->col, w->row);
                walker_spawn(w, x);
            }
        }
    }
}

/* ===================================================================== */
/* [7] scene                                                              */
/* ===================================================================== */

typedef enum { STATE_GROW, STATE_FLASH } AppState;

static Crystal  g_xtal;
static AppState g_state      = STATE_GROW;
static int      g_flash_tick = 0;
static bool     g_paused     = false;

static void scene_reset(void)
{
    xtal_init(&g_xtal, g_cols, g_rows);
    walkers_init(&g_xtal);
    g_state = STATE_GROW;
}

static void scene_init(void)
{
    rain_init();
    g_n_walkers = WALKER_DEFAULT;
    scene_reset();
}

static void scene_tick(void)
{
    if (g_paused) return;

    if (g_state == STATE_FLASH) {
        rain_tick();  /* rain keeps falling during flash */
        if (--g_flash_tick <= 0) scene_reset();
        return;
    }

    rain_tick();
    walkers_tick(&g_xtal);

    /* Trigger flash when crystal reaches the threshold radius */
    if (g_xtal.max_frozen_dist >= g_xtal.trigger_dist) {
        g_state      = STATE_FLASH;
        g_flash_tick = FLASH_FRAMES;
    }
}

static void scene_draw(void)
{
    bool flashing = (g_state == STATE_FLASH);

    /* Rain first (background), crystal drawn on top */
    rain_draw(&g_xtal);
    xtal_draw(&g_xtal, flashing);

    /* HUD */
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    for (int c = 0; c < g_cols; c++) mvaddch(g_rows - 1, c, ' ');
    mvprintw(g_rows - 1, 1,
        "MatrixFlake  q:quit  r:reset  +/-:walkers:%d  [/]:rain:%.1fx"
        "  t:%-8s  frozen:%d",
        g_n_walkers, (double)g_speed_mult,
        k_themes[g_theme].name, g_xtal.frozen_count);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* ===================================================================== */
/* [8] app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

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
    color_init();

    getmaxyx(stdscr, g_rows, g_cols);
    scene_init();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            scene_init();
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27:
            g_quit = 1; break;
        case 'r': case 'R':
            scene_reset(); break;
        case 'p': case 'P': case ' ':
            g_paused = !g_paused; break;
        case '+': case '=':
            g_n_walkers += 5;
            if (g_n_walkers > WALKER_MAX) g_n_walkers = WALKER_MAX;
            walkers_init(&g_xtal);
            break;
        case '-':
            g_n_walkers -= 5;
            if (g_n_walkers < 1) g_n_walkers = 1;
            break;
        case ']':
            g_speed_mult += 0.2f;
            if (g_speed_mult > 4.0f) g_speed_mult = 4.0f;
            break;
        case '[':
            g_speed_mult -= 0.2f;
            if (g_speed_mult < 0.1f) g_speed_mult = 0.1f;
            break;
        case 't': case 'T':
            g_theme = (g_theme + 1) % N_THEMES;
            theme_apply(g_theme);
            break;
        default: break;
        }

        long long now = clock_ns();
        scene_tick();
        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
