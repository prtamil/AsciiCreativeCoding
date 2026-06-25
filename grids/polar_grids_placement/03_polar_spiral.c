/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_polar_spiral.c — draw a spiral by walking along it and dropping dots.
 *
 * Move the cursor to pick where the spiral starts, watch a live preview, and
 * press space to stamp it down for keeps. Two spiral shapes: Archimedean (arms
 * evenly spaced) and logarithmic (arms spread wider as they go out, like a
 * nautilus shell).
 *
 * The trick here: instead of checking every screen cell "are you near a
 * spiral?", we step the angle forward, compute the radius for that angle, and
 * place a dot right there. Walking the curve instead of scanning the screen.
 *
 * Sister files: 02_polar_arc.c (arc/spoke from two anchors),
 *               grids/polar_grids/03_archimedean_spiral.c (the scan-the-screen
 *               version of the same spiral, for comparison).
 */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ── §1 config ── */

#define TARGET_FPS    30
#define CELL_W         2
#define CELL_H         4

/* How fast the on-screen fps number settles. Small = smooth but laggy. */
#define FPS_EWMA_ALPHA  0.05

/* Archimedean shape: how far the arm steps outward over one full turn. */
#define PITCH_DEFAULT  32.0
#define PITCH_MIN       8.0
#define PITCH_MAX      80.0
#define PITCH_STEP      4.0

/* Log-spiral shape: bigger growth = arms fly outward faster each turn. */
#define GROWTH_DEFAULT  0.18
#define GROWTH_MIN      0.05
#define GROWTH_MAX      0.70
#define GROWTH_STEP     0.02
#define PHI             1.61803398874989484820
#define GROWTH_GOLDEN  (2.0 * log(PHI) / M_PI)   /* the nautilus-shell rate, ≈ 0.31 */

/* How many full turns the spiral makes. */
#define N_TURNS_DEFAULT  3
#define N_TURNS_MIN      1
#define N_TURNS_MAX     10

/* Angle to step between dots. Bigger = sparser dots. */
#define DENSITY_DEFAULT  0.08    /* about one dot every 4.6 degrees */
#define DENSITY_MIN      0.01
#define DENSITY_MAX      0.40
#define DENSITY_STEP     0.01

/* Object pool */
#define MAX_OBJ        4096
#define OBJ_GLYPH      '.'

#define GOLDEN_ANGLE   (2.0 * M_PI / (PHI * PHI))
#define N_BG_SEEDS      600

#define PAIR_GRID    1
#define PAIR_ACTIVE  2
#define PAIR_ANCHOR  3
#define PAIR_HUD     4   /* top status bar, yellow */
#define PAIR_HINT    5   /* bottom key hints, cyan */

static const char *const BG_NAMES[] = {
    "rings+spokes", "log-polar",  "archimedean",
    "log-spiral",   "sunflower",  "equal-area",  "elliptic",
};
#define N_BG_TYPES  7

static const short THEME_FG[][2] = {
    {75,  COLOR_CYAN},
    {82,  COLOR_GREEN},
    {69,  COLOR_BLUE},
    {201, COLOR_MAGENTA},
    {226, COLOR_YELLOW},
};
#define N_THEMES  5

/* ── §2 clock ── */

static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec=(time_t)(ns/1000000000LL),
                          .tv_nsec=(long)(ns%1000000000LL) };
    nanosleep(&r, NULL);
}

/* ── §3 color ── */

static void color_init(int theme)
{
    start_color(); use_default_colors();
    short fg = COLORS >= 256 ? THEME_FG[theme][0] : THEME_FG[theme][1];
    init_pair(PAIR_GRID,   fg,                               -1);
    init_pair(PAIR_ACTIVE, COLORS>=256 ? 255 : COLOR_WHITE,  -1);
    init_pair(PAIR_ANCHOR, COLORS>=256 ? 220 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS>=256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS>=256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 polar mapping ── */

/*
 * Everything we need to map between screen cells and polar (radius, angle)
 * coordinates, all anchored to one center point. Set up once at startup and
 * again on resize; nothing here changes per frame.
 */
typedef struct {
    int mode;             /* which background pattern (0..6), see BG_NAMES */
    int rows, cols;       /* terminal size in cells right now */
    int cw, ch;           /* cell size in pixels, so a cell isn't square */
    int ox, oy;           /* center of the polar grid, in cells */
    int max_ring, max_spoke;  /* how many rings/spokes fit on screen */
} GridCtx;

static void ctx_init(GridCtx *g, int mode, int rows, int cols)
{
    memset(g, 0, sizeof *g);
    g->mode = mode; g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->ox = cols / 2; g->oy = rows / 2;
    g->max_spoke = 12;
    int half_diag_px = (int)(sqrt((double)(g->ox*g->cw)*(g->ox*g->cw) +
                                   (double)(g->oy*g->ch)*(g->oy*g->ch)));
    g->max_ring = (int)(half_diag_px / 20.0);
}

/* a screen cell -> its distance and angle from the centre */
static void screen_to_polar(int col, int row, int ox, int oy,
                           double *r_px, double *theta)
{
    double dx = (double)(col - ox) * CELL_W;
    double dy = (double)(row - oy) * CELL_H;
    *r_px  = sqrt(dx*dx + dy*dy);
    *theta = atan2(dy, dx);
}

/* the reverse — a distance and angle -> the screen cell that lands there */
static void polar_to_screen(double r, double theta, int ox, int oy,
                              int *col, int *row)
{
    *col = ox + (int)round(r * cos(theta) / CELL_W);
    *row = oy + (int)round(r * sin(theta) / CELL_H);
}

static char line_glyph(double theta)
{
    double a = fmod(theta + 2.0*M_PI, M_PI);
    if (a < M_PI/8.0 || a >= 7.0*M_PI/8.0) return '-';
    if (a < 3.0*M_PI/8.0)                   return '\\';
    if (a < 5.0*M_PI/8.0)                   return '|';
    return '/';
}

/* ── §5 pool ── */

/* One stamped dot: where it sits and what character to draw. */
typedef struct {
    int  row, col;   /* cell position on screen */
    char glyph;      /* the character to draw there */
    bool alive;      /* false means skip it when drawing */
} Obj;

/*
 * The permanent collection of stamped dots. A plain fixed array used like a
 * stack: stamping pushes onto the end, clearing just resets the count to 0
 * (no need to wipe the old entries). Full at MAX_OBJ; extra dots are dropped.
 */
typedef struct {
    Obj items[MAX_OBJ];
    int count;
} Pool;

static void pool_place(Pool *p, int row, int col,
                       int rows, int cols, char glyph)
{
    if (row < 0 || row >= rows-1 || col < 0 || col >= cols) return;
    if (p->count < MAX_OBJ)
        p->items[p->count++] = (Obj){ row, col, glyph, true };
}

static void pool_draw(const Pool *p)
{
    attron(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        mvaddch(p->items[i].row, p->items[i].col,
                (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);
}

static void pool_clear(Pool *p) { p->count = 0; }

/* ── §6 cursor ── */

/*
 * The movable spot that marks where the next spiral starts. We keep it both
 * ways: as a screen cell (for drawing and arrow-key moves) and as polar
 * radius/angle (what the spiral math actually needs). The two are kept in
 * sync — move the cell, then recompute the polar pair.
 */
typedef struct {
    int    row, col;   /* where it is on screen, in cells */
    double r, theta;   /* same spot as distance-from-center and angle */
} Cursor;

static void cursor_sync_polar(Cursor *c, const GridCtx *g)
{
    screen_to_polar(c->col, c->row, g->ox, g->oy, &c->r, &c->theta);
}

static void cursor_reset(Cursor *c, const GridCtx *g)
{
    c->r = 20.0; c->theta = 0.0;
    polar_to_screen(c->r, c->theta, g->ox, g->oy, &c->col, &c->row);
    if (c->row < 0)          c->row = 0;
    if (c->row >= g->rows-1) c->row = g->rows-2;
    if (c->col < 0)          c->col = 0;
    if (c->col >= g->cols)   c->col = g->cols-1;
}

static void cursor_move(Cursor *c, const GridCtx *g, int dr, int dc)
{
    int nr = c->row + dr, nc = c->col + dc;
    if (nr >= 0 && nr < g->rows-1) c->row = nr;
    if (nc >= 0 && nc < g->cols)   c->col = nc;
    cursor_sync_polar(c, g);
}

static void cursor_draw(const Cursor *c, const GridCtx *g)
{
    if (c->row < 0 || c->row >= g->rows-1 || c->col < 0 || c->col >= g->cols)
        return;
    attron(COLOR_PAIR(PAIR_ANCHOR) | A_REVERSE | A_BOLD);
    mvaddch(c->row, c->col, (chtype)'+');
    attroff(COLOR_PAIR(PAIR_ANCHOR) | A_REVERSE | A_BOLD);
}

/* ── §7 mode ── */

/* These draw the faint backdrop the spiral sits on. Each is a self-contained
 * pattern lifted from a sister file in grids/polar_grids/; the cited file is
 * where to go for how that one works. Cycle them with a/e. */

/* Mode 0 — evenly spaced rings crossed by straight spokes, like a dartboard. */
static void bg_rings_spokes_draw(const GridCtx *g)
{
    const double two_pi = 2.0 * M_PI;
    const double sp = 20.0, rw = 1.6, sw = 0.10;
    const double sa = two_pi / 12.0;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double r, th;
            screen_to_polar(col, row, g->ox, g->oy, &r, &th);
            double rp   = fmod(r, sp);
            bool   on_r = rp < rw || rp > sp - rw;
            double tn   = fmod(th + two_pi, two_pi);
            double sp2  = fmod(tn, sa);
            if (on_r || (r > 3.0 && (sp2 < sw || sp2 > sa - sw)))
                mvaddch(row, col, (chtype)(unsigned char)line_glyph(th));
        }
    }
}

/* Mode 1 — rings that get farther apart as they go out (polar_grids/02_log_polar.c). */
static void bg_log_polar_draw(const GridCtx *g)
{
    const double two_pi = 2.0 * M_PI;
    const double rmin = 4.0, ls = 0.25, rwu = 0.08, sw = 0.10;
    const double sa = two_pi / 12.0;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double r, th;
            screen_to_polar(col, row, g->ox, g->oy, &r, &th);
            bool on_r = false;
            if (r > rmin) {
                double u  = log(r / rmin) / ls;
                double fr = u - floor(u);
                on_r = fr < rwu || fr > 1.0 - rwu;
            }
            double tn  = fmod(th + two_pi, two_pi);
            double sp2 = fmod(tn, sa);
            if (on_r || (r > 3.0 && (sp2 < sw || sp2 > sa - sw)))
                mvaddch(row, col, (chtype)(unsigned char)line_glyph(th));
        }
    }
}

/* Mode 2 — two evenly-spaced spiral arms (polar_grids/03_archimedean_spiral.c). */
static void bg_archimedean_draw(const GridCtx *g)
{
    const double two_pi = 2.0 * M_PI;
    const double pitch = 32.0, sw = 0.20, rmin = 3.0;
    double a = pitch / two_pi;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double r, th;
            screen_to_polar(col, row, g->ox, g->oy, &r, &th);
            if (r < rmin) continue;
            double tn = fmod(th + two_pi, two_pi);
            double ph = fmod(2.0 * (tn - r / a) + 2.0 * two_pi, two_pi);
            if (ph < sw || ph > two_pi - sw)
                mvaddch(row, col, (chtype)(unsigned char)line_glyph(th));
        }
    }
}

/* Mode 3 — two nautilus-shell spiral arms at the golden growth rate
 * (polar_grids/04_log_spiral.c). */
static void bg_log_spiral_draw(const GridCtx *g)
{
    const double two_pi = 2.0 * M_PI;
    const double growth = 2.0 * log(PHI) / M_PI;
    const double sw = 0.22, rmin = 4.0;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double r, th;
            screen_to_polar(col, row, g->ox, g->oy, &r, &th);
            if (r < rmin) continue;
            double tn = fmod(th + two_pi, two_pi);
            double tp = log(r / rmin) / growth;
            double ph = fmod(2.0 * (tn - tp) + 2.0 * two_pi, two_pi);
            if (ph < sw || ph > two_pi - sw)
                mvaddch(row, col, (chtype)(unsigned char)line_glyph(th));
        }
    }
}

/* Mode 4 — seeds packed in a sunflower-head spiral, golden-angle apart
 * (polar_grids/05_sunflower.c). */
static void bg_sunflower_draw(const GridCtx *g)
{
    const double sp = 3.5;
    /* Scratch "already drawn here?" grid so two seeds don't fight over one
     * cell. Freed before we return — owned entirely by this call. */
    bool *vis = calloc((size_t)(g->rows * g->cols), 1);
    if (!vis) return;

    for (int i = 0; i < N_BG_SEEDS; i++) {
        double r  = sqrt((double)i) * sp;
        double th = (double)i * GOLDEN_ANGLE;
        int    c  = g->ox + (int)round(r * cos(th) / CELL_W);
        int    rw = g->oy + (int)round(r * sin(th) / CELL_H);
        if (rw < 0 || rw >= g->rows - 1 || c < 0 || c >= g->cols) continue;
        if (vis[rw * g->cols + c]) continue;
        vis[rw * g->cols + c] = true;
        mvaddch(rw, c, (chtype)(unsigned char)'o');
    }
    free(vis);
}

/* Mode 5 — rings spaced so every ring band has the same area, plus spokes
 * (polar_grids/06_sector.c). */
static void bg_equal_area_draw(const GridCtx *g)
{
    const double two_pi = 2.0 * M_PI;
    const double ru = 18.0, rwf = 0.06, sw = 0.10;
    const double sa = two_pi / 12.0;
    double rusq = ru * ru;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double r, th;
            screen_to_polar(col, row, g->ox, g->oy, &r, &th);
            if (r < 3.0) continue;
            double kf  = (r * r) / rusq;
            double fr  = kf - floor(kf);
            double tn  = fmod(th + two_pi, two_pi);
            double sp2 = fmod(tn, sa);
            if (fr < rwf || fr > 1.0 - rwf || sp2 < sw || sp2 > sa - sw)
                mvaddch(row, col, (chtype)(unsigned char)line_glyph(th));
        }
    }
}

/* Mode 6 — rings squashed into ovals instead of circles (polar_grids/07_elliptic.c). */
static void bg_elliptic_draw(const GridCtx *g)
{
    const double A = 1.6, B = 1.0, sp = 20.0, rwu = 0.07;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double dx = (double)(col - g->ox) * CELL_W;
            double dy = (double)(row - g->oy) * CELL_H;
            double er = sqrt((dx/A)*(dx/A) + (dy/B)*(dy/B));
            if (er < 0.5) continue;
            double et = atan2(dy/B, dx/A);
            double fr = (er / sp) - floor(er / sp);
            if (fr < rwu || fr > 1.0 - rwu)
                mvaddch(row, col, (chtype)(unsigned char)line_glyph(et));
        }
    }
}

/* Picks the right backdrop for the current mode and draws it. */
static void draw_polar_bg(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_GRID));
    switch (g->mode) {
    case 0: bg_rings_spokes_draw(g); break;
    case 1: bg_log_polar_draw   (g); break;
    case 2: bg_archimedean_draw (g); break;
    case 3: bg_log_spiral_draw  (g); break;
    case 4: bg_sunflower_draw   (g); break;
    case 5: bg_equal_area_draw  (g); break;
    case 6: bg_elliptic_draw    (g); break;
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ── §8 spiral ── */

/* PLACEMENT STRATEGY: spiral trail from the cursor. Step radius and angle
 * together along the spiral's law — one curls the angle forward, the other
 * grows the radius to match — and stamp a dot at each step. Walking the curve,
 * never scanning the screen. */

/* Archimedean: radius grows by a fixed amount per step, so arms stay evenly
 * spaced. (en.wikipedia.org/wiki/Archimedean_spiral) */
static void spiral_place_archim(Pool *pool, double r0, double theta0,
                                 double pitch, int n_turns, double density,
                                 const GridCtx *g)
{
    double a       = pitch / (2.0 * M_PI);
    double theta_max = (double)n_turns * 2.0 * M_PI;
    for (double t = 0.0; t <= theta_max; t += density) {
        double r  = r0 + a * t;
        double th = theta0 + t;
        int c, row;
        polar_to_screen(r, th, g->ox, g->oy, &c, &row);
        pool_place(pool, row, c, g->rows, g->cols, OBJ_GLYPH);
    }
}

/* Logarithmic: radius grows by a percentage of itself per step, so arms spread
 * wider and wider — the nautilus look. (en.wikipedia.org/wiki/Logarithmic_spiral) */
static void spiral_place_log(Pool *pool, double r0, double theta0,
                               double growth, int n_turns, double density,
                               const GridCtx *g)
{
    /* A start radius of 0 would stay stuck at 0 forever (growing a percentage
     * of nothing is still nothing), so nudge it up to at least 1. */
    if (r0 < 1.0) r0 = 1.0;
    double theta_max = (double)n_turns * 2.0 * M_PI;
    for (double t = 0.0; t <= theta_max; t += density) {
        double r  = r0 * exp(growth * t);
        double th = theta0 + t;
        int c, row;
        polar_to_screen(r, th, g->ox, g->oy, &c, &row);
        pool_place(pool, row, c, g->rows, g->cols, OBJ_GLYPH);
    }
}

/* ── §9 scene ── */

/* Draws the on-screen readouts: spiral stats top-right, key hints along the
 * bottom. Kept bright and bold so they stay readable over the animation. */
static void hud_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                     bool log_mode, double pitch, double growth,
                     int n_turns, double density,
                     int theme, double fps, bool paused)
{
    const char *mode_str = log_mode ? "log-spiral" : "archimedean";
    double param = log_mode ? growth : pitch;
    const char *pname = log_mode ? "g" : "p";
    char buf[96];
    snprintf(buf, sizeof buf,
             " %5.1f fps  r:%.0f  θ:%.0f°  %s:%s=%.2f  n:%d  d:%.2f  objs:%d ",
             fps, cur->r, cur->theta*180.0/M_PI,
             mode_str, pname, param, n_turns, density, p->count);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);
    mvprintw(0, 0, " %-13s %s ", BG_NAMES[g->mode], paused ? "PAUSED" : "");
    attroff(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " l:archi  o:log  +/-:param  [/]:turns  ,/.:density"
        "  spc:stamp  C:clear  a/e:bg  t:theme(%d)  q:quit ", theme + 1);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void preview_draw(const GridCtx *g, const Cursor *cur,
                         bool log_mode, double pitch, double growth,
                         int n_turns, double density)
{
    /* Ghost of the spiral you'd stamp right now — recomputed from the cursor
     * every frame so it follows you, but never saved into the pool. */
    double r0 = (cur->r < 1.0) ? 1.0 : cur->r;
    double a  = pitch / (2.0 * M_PI);
    double theta_max = (double)n_turns * 2.0 * M_PI;
    attron(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
    for (double t = 0.0; t <= theta_max; t += density) {
        double r  = log_mode ? r0 * exp(growth * t) : r0 + a * t;
        double th = cur->theta + t;
        int c, row;
        polar_to_screen(r, th, g->ox, g->oy, &c, &row);
        if (row >= 0 && row < g->rows-1 && c >= 0 && c < g->cols)
            mvaddch(row, c, (chtype)(unsigned char)OBJ_GLYPH);
    }
    attroff(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *pool, const Cursor *cur,
                       bool log_mode, double pitch, double growth,
                       int n_turns, double density,
                       int theme, double fps, bool paused)
{
    erase();
    draw_polar_bg(g);
    pool_draw(pool);
    preview_draw(g, cur, log_mode, pitch, growth, n_turns, density);
    cursor_draw(cur, g);
    hud_draw(g, pool, cur, log_mode, pitch, growth, n_turns, density,
             theme, fps, paused);
    wnoutrefresh(stdscr); doupdate();
}

/* ── §10 screen ── */

static void screen_cleanup(void) { endwin(); }
static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme); atexit(screen_cleanup);
}

/* ── §11 app ── */

static volatile sig_atomic_t g_running = 1, g_need_resize = 0;
static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);

    int theme = 0;
    screen_init(theme);

    int rows = LINES, cols = COLS;
    GridCtx ctx; ctx_init(&ctx, 0, rows, cols);
    Pool    pool; pool_clear(&pool);
    Cursor  cur; cursor_reset(&cur, &ctx);

    bool   log_mode = false;
    double pitch    = PITCH_DEFAULT;
    double growth   = GROWTH_DEFAULT;
    int    n_turns  = N_TURNS_DEFAULT;
    double density  = DENSITY_DEFAULT;

    bool    paused = false;
    double  fps    = TARGET_FPS;
    int64_t t0     = clock_ns();
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            rows = LINES; cols = COLS;
            ctx_init(&ctx, ctx.mode, rows, cols);
            /* Center moved, so keep the cursor at the same radius/angle and
             * work out its new cell. */
            polar_to_screen(cur.r, cur.theta, ctx.ox, ctx.oy, &cur.col, &cur.row);
            if (cur.row < 0)          cur.row = 0;
            if (cur.row >= ctx.rows-1) cur.row = ctx.rows-2;
            if (cur.col < 0)          cur.col = 0;
            if (cur.col >= ctx.cols)   cur.col = ctx.cols-1;
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'P': paused = !paused; break;
        case 't': theme = (theme+1) % N_THEMES; color_init(theme); break;
        case 'a':
        case 'e': {
            int m = (ch == 'a') ? (ctx.mode - 1 + N_BG_TYPES) % N_BG_TYPES
                                : (ctx.mode + 1) % N_BG_TYPES;
            ctx_init(&ctx, m, rows, cols);
            /*
             * When you switch to a spiral backdrop, match your spiral to it so
             * the preview lines up: the Archimedean grid (mode 2) and the
             * log-spiral grid (mode 3) each set their own shape. Other
             * backdrops leave your current spiral settings untouched.
             */
            if (ctx.mode == 2) { log_mode = false; pitch  = 32.0; }
            if (ctx.mode == 3) { log_mode = true;  growth = GROWTH_GOLDEN; }
            break;
        }
        case 'l': log_mode = false; break;
        case 'o': log_mode = true;  break;
        case ' ':
            if (log_mode)
                spiral_place_log(&pool, cur.r, cur.theta, growth,
                                  n_turns, density, &ctx);
            else
                spiral_place_archim(&pool, cur.r, cur.theta, pitch,
                                     n_turns, density, &ctx);
            break;
        case '+': case '=':
            if (log_mode) { if (growth < GROWTH_MAX) growth += GROWTH_STEP; }
            else          { if (pitch  < PITCH_MAX)  pitch  += PITCH_STEP; }
            break;
        case '-':
            if (log_mode) { if (growth > GROWTH_MIN) growth -= GROWTH_STEP; }
            else          { if (pitch  > PITCH_MIN)  pitch  -= PITCH_STEP; }
            break;
        case '[':
            if (n_turns > N_TURNS_MIN) n_turns--;
            break;
        case ']':
            if (n_turns < N_TURNS_MAX) n_turns++;
            break;
        case ',':
            if (density < DENSITY_MAX) density += DENSITY_STEP;
            break;
        case '.':
            if (density > DENSITY_MIN) density -= DENSITY_STEP;
            break;
        case 'C': pool_clear(&pool); break;
        case 'r': cursor_reset(&cur, &ctx); break;
        case KEY_UP:    cursor_move(&cur, &ctx, -1,  0); break;
        case KEY_DOWN:  cursor_move(&cur, &ctx, +1,  0); break;
        case KEY_LEFT:  cursor_move(&cur, &ctx,  0, -1); break;
        case KEY_RIGHT: cursor_move(&cur, &ctx,  0, +1); break;
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9/(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;
        if (!paused)
            scene_draw(&ctx, &pool, &cur, log_mode, pitch, growth,
                       n_turns, density, theme, fps, paused);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
