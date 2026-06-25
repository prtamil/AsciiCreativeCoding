/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 06_sector.c — a polar grid where every cell covers the same area.
 *
 * Rings get packed tighter as you go out (spacing follows √k) so each band
 * between two rings has the same area — like a dartboard where every ring is
 * equally hard to hit.  Mix that with even pie-slice sectors and every cell is
 * the same size.  An '@' marker sits on one cell; arrows move it, +/- changes
 * the ring spacing, [/] changes the number of sectors.
 *
 * Sister files: 01_rings_spokes.c (evenly-spaced rings — outer cells bigger),
 *               02_log_polar.c (log-spaced rings), ../rect_grids/01_uniform_rect.c.
 *
 * The "same area per cell" idea shows up in real tools: HEALPix maps the sky
 * into equal-area pixels (Górski et al. 2005, ApJ 622:759), and camera sensors
 * bin pixels this way so each bin catches the same amount of light.
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

#define TARGET_FPS      30
#define CELL_W          2
#define CELL_H          4

/* Size of the innermost ring.  The first ring sits here, and each ring out is
 * spaced by √k of this so the bands all come out equal-area. */
#define R_UNIT_DEFAULT  18.0
#define R_UNIT_MIN       6.0
#define R_UNIT_MAX      40.0
#define R_UNIT_STEP      2.0

/* How wide a ring is allowed to look when we draw it.  Bigger = thicker rings. */
#define RING_W_F        0.06

/* Sectors */
#define N_SECTORS_DEFAULT  12
#define N_SECTORS_MIN       4
#define N_SECTORS_MAX      36
#define SECTOR_W           0.10   /* how thick each sector line is, in radians */
#define SECTOR_MIN_R       3.0

/* Ignore anything closer than this to the centre — keeps it from smearing. */
#define R_MIN            3.0

/* How fast the on-screen fps number reacts; small = smooth, slow to change. */
#define FPS_EWMA_ALPHA   0.05

#define PAIR_GRID    1
#define PAIR_CURSOR  2
#define PAIR_HUD     3
#define PAIR_HINT    4

static const short THEME_FG[][2] = {
    {75,  COLOR_CYAN},
    {82,  COLOR_GREEN},
    {69,  COLOR_BLUE},
    {201, COLOR_MAGENTA},
    {226, COLOR_YELLOW},
};
#define N_THEMES 5

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
    init_pair(PAIR_GRID,   fg,                              -1);
    init_pair(PAIR_CURSOR, COLORS>=256 ? 226 : COLOR_YELLOW,-1);
    init_pair(PAIR_HUD,    COLORS>=256 ? 226 : COLOR_YELLOW,-1);
    init_pair(PAIR_HINT,   COLORS>=256 ?  51 : COLOR_CYAN,  -1);
}

/* ── §4 polar mapping & lattice ── */

/*
 * GridCtx — everything we need to know to draw the grid and place the cursor:
 * the terminal size, how the rings are spaced, how many sectors, where the
 * centre is, and how far the cursor is allowed to roam.
 */
typedef struct {
    int rows, cols;

    double r_unit;         /* size of the innermost ring, in pixels         */
    int    n_spokes;       /* how many pie-slice sectors                     */
    int    cell_w, cell_h; /* pixels packed into one character cell          */

    int    ox, oy;         /* centre of the grid, in character cells         */

    int    max_ring, max_spoke; /* furthest ring/sector the cursor can reach */
} GridCtx;

/* Recompute the centre and how far out the rings fit, from the terminal size.
 * Called again on resize and whenever the ring spacing changes. */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows   = rows;
    g->cols   = cols;
    g->cell_w = CELL_W;
    g->cell_h = CELL_H;
    g->ox     = cols / 2;
    g->oy     = rows / 2;
    if (g->r_unit   <= 0.0) g->r_unit   = R_UNIT_DEFAULT;
    if (g->n_spokes <= 0)   g->n_spokes = N_SECTORS_DEFAULT;

    double rx = (double)cols * 0.5 * CELL_W;
    double ry = (double)rows * 0.5 * CELL_H;
    double r_visible = (rx < ry ? rx : ry);
    int mr = 0;
    if (r_visible > g->r_unit) {
        double ratio = r_visible / g->r_unit;
        mr = (int)floor(ratio * ratio - 0.5);
        if (mr < 0) mr = 0;
    }
    g->max_ring  = mr;
    g->max_spoke = g->n_spokes - 1;
}

/* recipe step 1 — a (ring, sector) cell -> the screen cell at its middle. The
 * equal-area radius is √(ring+0.5)·r_unit (NOT (ring+0.5)·spacing): squaring
 * the radius makes the bands evenly spaced, so √k packs the rings tighter as
 * they go out and every band ends up the same area. */
static void polar_to_screen(const GridCtx *g, int ring, int spoke,
                            int *sr, int *sc)
{
    double mid_radius = sqrt((double)ring + 0.5) * g->r_unit;
    double theta_mid  = ((double)spoke + 0.5) * (2.0 * M_PI / (double)g->n_spokes);
    double cx = mid_radius * cos(theta_mid);
    double cy = mid_radius * sin(theta_mid);
    *sc = g->ox + (int)round(cx / (double)g->cell_w);
    *sr = g->oy + (int)round(cy / (double)g->cell_h);
}

/* the reverse — a screen cell -> its distance and angle from the grid centre */
static void screen_to_polar(const GridCtx *g, int col, int row,
                            double *r_px, double *theta)
{
    double dx = (double)(col - g->ox) * g->cell_w;
    double dy = (double)(row - g->oy) * g->cell_h;
    *r_px  = sqrt(dx * dx + dy * dy);
    *theta = atan2(dy, dx);
}

/* the line char matching a direction: '-' horizontal, '|' vertical, '/' '\' the
 * diagonals. A line and its 180° flip look the same, so we fold into a half-turn. */
static char line_glyph(double theta)
{
    double a = fmod(theta + 2.0*M_PI, M_PI);
    if (a < M_PI/8.0 || a >= 7.0*M_PI/8.0) return '-';
    if (a < 3.0*M_PI/8.0)                   return '\\';
    if (a < 5.0*M_PI/8.0)                   return '|';
    return '/';
}

/* recipe step 2 — draw the grid: each cell knows it's on *some* equal-area ring
 * from its distance and on *some* sector from its angle, with no loop over
 * rings/sectors. Ring + sector crossing -> '+', one of them -> a line char. */
static void draw_lattice(const GridCtx *g)
{
    double sector_angle = 2.0 * M_PI / (double)g->n_spokes;
    double r_unit_sq    = g->r_unit * g->r_unit;

    attron(COLOR_PAIR(PAIR_GRID));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double r_px, theta;
            screen_to_polar(g, col, row, &r_px, &theta);
            if (r_px < R_MIN) continue;

            /* Squaring the distance turns the √k ring spacing into evenly-spaced
             * numbers, so a cell sits on a ring when k_float lands near a whole. */
            double k_float = (r_px * r_px) / r_unit_sq;
            double frac    = k_float - floor(k_float);
            bool on_ring = (frac < RING_W_F || frac > 1.0 - RING_W_F);

            double theta_norm   = fmod(theta + 2.0*M_PI, 2.0*M_PI);
            double sector_phase = fmod(theta_norm, sector_angle);
            bool on_sector = (r_px > SECTOR_MIN_R) &&
                             (sector_phase < SECTOR_W ||
                              sector_phase > sector_angle - SECTOR_W);

            if (!on_ring && !on_sector) continue;

            char c = (on_ring && on_sector) ? '+' : line_glyph(theta);
            mvaddch(row, col, (chtype)(unsigned char)c);
        }
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ── §5 cursor ── */

/*
 * Cursor — which cell the '@' marker is on, given as a ring number (0 = inner)
 * and a sector number (0 = first pie slice).
 */
typedef struct { int ring, spoke; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->ring  = g->max_ring / 2;
    cur->spoke = 0;
}

/* recipe step 3 — move the cursor: in/out clamps at the edge, round wraps */
static void cursor_move(Cursor *cur, const GridCtx *g, int d_ring, int d_spoke)
{
    int nr = cur->ring + d_ring;
    if (nr < 0)            nr = 0;
    if (nr > g->max_ring)  nr = g->max_ring;
    cur->ring = nr;

    int n = g->n_spokes > 0 ? g->n_spokes : 1;
    int ns = (cur->spoke + d_spoke) % n;
    if (ns < 0) ns += n;
    cur->spoke = ns;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    polar_to_screen(g, cur->ring, cur->spoke, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, (chtype)'@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     bool paused, double fps)
{
    char buf[112];
    snprintf(buf, sizeof buf,
             " ring:%d sector:%d  R_unit:%.0fpx  sectors:%d  th:%d  %5.1f fps  %s ",
             cur->ring, cur->spoke, g->r_unit, g->n_spokes,
             theme + 1, fps, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:R-unit  [/]:sectors ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       bool paused, double fps)
{
    erase();
    draw_lattice(g);
    cursor_draw(cur, g);
    hud_draw(g, cur, theme, paused, fps);
    wnoutrefresh(stdscr); doupdate();
}

/* ── §7 screen ── */

static void screen_cleanup(void) { endwin(); }
static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    atexit(screen_cleanup);
}

/* ── §8 app ── */

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
    screen_init();
    color_init(theme);

    GridCtx g = {0};
    g.r_unit   = R_UNIT_DEFAULT;
    g.n_spokes = N_SECTORS_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    bool   paused = false;
    double fps    = TARGET_FPS;
    int64_t t0    = clock_ns();
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            ctx_init(&g, LINES, COLS);
            cursor_reset(&cur, &g);
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'p': paused = !paused; break;
        case 'r': cursor_reset(&cur, &g); break;
        case 't': theme = (theme + 1) % N_THEMES; color_init(theme); break;
        case KEY_UP:    cursor_move(&cur, &g, -1,  0); break;
        case KEY_DOWN:  cursor_move(&cur, &g, +1,  0); break;
        case KEY_LEFT:  cursor_move(&cur, &g,  0, -1); break;
        case KEY_RIGHT: cursor_move(&cur, &g,  0, +1); break;
        case '+': case '=':
            if (g.r_unit < R_UNIT_MAX) {
                g.r_unit += R_UNIT_STEP;
                ctx_init(&g, LINES, COLS);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        case '-':
            if (g.r_unit > R_UNIT_MIN) {
                g.r_unit -= R_UNIT_STEP;
                ctx_init(&g, LINES, COLS);
            }
            break;
        case '[':
            if (g.n_spokes > N_SECTORS_MIN) {
                g.n_spokes -= (g.n_spokes > 8 ? 4 : 2);
                ctx_init(&g, LINES, COLS);
                if (cur.spoke > g.max_spoke) cur.spoke = g.max_spoke;
            }
            break;
        case ']':
            if (g.n_spokes < N_SECTORS_MAX) {
                g.n_spokes += (g.n_spokes >= 8 ? 4 : 2);
                ctx_init(&g, LINES, COLS);
            }
            break;
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) +
              (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;
        if (!paused)
            scene_draw(&g, &cur, theme, paused, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
