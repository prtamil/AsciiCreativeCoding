/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 07_elliptic.c — a polar grid drawn with stretched rings (ellipses).
 *
 * Take the round rings of 01_rings_spokes and squash them sideways: instead
 * of circles you get ellipses, and you can change how stretched they are
 * live. Press 'h' to also draw the family of curves that cross every ellipse
 * at a right angle (hyperbolae) — the same pattern you see around an electric
 * charge shaped like an oval rod. An '@' marker rides one cell of the grid.
 *
 * Sister files: 01_rings_spokes.c is the round version (this with A = B);
 *               06_sector.c is the equal-area variant.
 * Background, if you want the names and the physics:
 *   Elliptic coordinates / confocal conics — Wikipedia;
 *   the electrostatics tie-in — Griffiths, Introduction to Electrodynamics §3.3.
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

/* How much to stretch the rings. A widens, B heightens; A = B gives circles. */
#define AXIS_A_DEFAULT  1.6
#define AXIS_B_DEFAULT  1.0
#define AXIS_MIN        0.5
#define AXIS_MAX        4.0
#define AXIS_STEP       0.1

/* Gap between one ring and the next, and how far +/- nudge it. */
#define RING_SPACING_DEFAULT  20.0
#define RING_SPACING_MIN       8.0
#define RING_SPACING_MAX      48.0
#define RING_SPACING_STEP      4.0

/* How thick a ring is drawn (a fraction of the gap between rings). */
#define RING_W_U        0.07

/* How many slots the @ marker snaps to as it goes around. The rings are
 * smooth ellipses and ignore this; only the marker needs evenly spaced stops. */
#define N_SPOKES_DEFAULT  12

/* The 'h' overlay: how far apart the hyperbola curves are, and how thick. */
#define HYPER_STEP      0.12
#define HYPER_W         0.02

/* Don't draw rings right at the centre — everything bunches up and smears there. */
#define E_R_MIN         0.3

/* How quickly the on-screen fps number settles (a running average). */
#define FPS_EWMA_ALPHA  0.05

/* Color pair IDs */
#define PAIR_GRID    1
#define PAIR_HUD     2   /* status bar (yellow)      */
#define PAIR_HINT    3   /* key-hint footer (cyan)   */
#define PAIR_CURSOR  4   /* the bright '@' marker    */
#define PAIR_HYPER   5   /* the hyperbola overlay    */

static const short THEME_FG[][2] = {
    {75,  COLOR_CYAN},
    {82,  COLOR_GREEN},
    {69,  COLOR_BLUE},
    {201, COLOR_MAGENTA},
    {226, COLOR_YELLOW},
};
/* Matching highlight color for the hyperbola overlay, one per theme above. */
static const short THEME_HFG[][2] = {
    {214, COLOR_YELLOW},
    {220, COLOR_YELLOW},
    {214, COLOR_YELLOW},
    {82,  COLOR_GREEN},
    {75,  COLOR_CYAN},
};
#define N_THEMES 5

/* ── §2 clock ── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ── §3 color ── */

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg  = COLORS >= 256 ? THEME_FG[theme][0]  : THEME_FG[theme][1];
    short hfg = COLORS >= 256 ? THEME_HFG[theme][0] : THEME_HFG[theme][1];
    init_pair(PAIR_GRID,   fg,                                -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HYPER,  hfg,                                -1);
}

/* ── §4 formula ── */

/*
 * GridCtx — everything we need to know to draw the grid and to know where the
 * @ marker can go.
 *
 * The stretch factors and the ring gap live here as fields (not constants) on
 * purpose: when you press a key to stretch or space out the rings, we just edit
 * this struct and the very next frame draws the new shape.
 *
 *   rows, cols       size of the terminal, in characters
 *   cell_w, cell_h   how many pixels wide/tall one character cell counts as
 *   ox, oy           the centre of the grid (in cell coordinates)
 *   axis_a           sideways stretch; bigger = wider ellipses
 *   axis_b           up/down stretch; bigger = taller ellipses
 *   ring_spacing     gap from one ring to the next
 *   show_hyper       whether the 'h' overlay is on
 *   n_spokes         how many evenly spaced stops the @ marker has going around
 *   max_ring         outermost ring the marker can reach before it runs off-screen
 *   max_spoke        last marker stop going around (always n_spokes - 1)
 */
typedef struct {
    int rows, cols;
    int cell_w, cell_h;
    int ox, oy;

    double axis_a;
    double axis_b;
    double ring_spacing;
    bool   show_hyper;

    int    n_spokes;
    int    max_ring, max_spoke;
} GridCtx;

/* Recompute centre and how many rings fit, given the current terminal size.
 * Call it on startup and again after a resize or any stretch/spacing change. */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows   = rows;
    g->cols   = cols;
    g->cell_w = CELL_W;
    g->cell_h = CELL_H;
    g->ox     = cols / 2;
    g->oy     = rows / 2;

    if (g->axis_a       <= 0.0) g->axis_a       = AXIS_A_DEFAULT;
    if (g->axis_b       <= 0.0) g->axis_b       = AXIS_B_DEFAULT;
    if (g->ring_spacing <= 0.0) g->ring_spacing = RING_SPACING_DEFAULT;
    if (g->n_spokes     <= 0)   g->n_spokes     = N_SPOKES_DEFAULT;

    /* How far it is from centre to edge, in pixels, across and down. */
    double hx_px = (double)cols * 0.5 * CELL_W;
    double hy_px = (double)rows * 0.5 * CELL_H;
    /* Biggest ring that still fits each way; take the smaller so nothing
     * spills off the narrow side of the screen. */
    double max_u_x = hx_px / g->axis_a;
    double max_u_y = hy_px / g->axis_b;
    double max_u   = (max_u_x < max_u_y ? max_u_x : max_u_y);
    int mr = (int)floor(max_u / g->ring_spacing - 0.5);
    if (mr < 0) mr = 0;
    g->max_ring  = mr;
    g->max_spoke = g->n_spokes - 1;
}

/* Turn a (which ring, which way around) pair into an actual screen cell.
 * We aim for the middle of the ring's band, not its edge, so the @ sits
 * comfortably between two rings rather than on a line. */
static void ctx_to_screen(const GridCtx *g, int ring, int spoke,
                          int *sr, int *sc)
{
    double theta_mid = ((double)spoke + 0.5) *
                       (2.0 * M_PI / (double)g->n_spokes);
    double u_mid     = ((double)ring + 0.5) * g->ring_spacing;
    double cx = u_mid * g->axis_a * cos(theta_mid);
    double cy = u_mid * g->axis_b * sin(theta_mid);
    *sc = g->ox + (int)round(cx / (double)g->cell_w);
    *sr = g->oy + (int)round(cy / (double)g->cell_h);
}

/* Pick the ASCII character ( - \ | / ) that best looks like a line tilted
 * at this angle, so the rings read as smooth curves instead of dots. */
static char angle_char(double theta)
{
    double a = fmod(theta + 2.0*M_PI, M_PI);
    if (a < M_PI/8.0 || a >= 7.0*M_PI/8.0) return '-';
    if (a < 3.0*M_PI/8.0)                   return '\\';
    if (a < 5.0*M_PI/8.0)                   return '|';
    return '/';
}

/* Draw the whole grid: walk every screen cell, ask "is this on a ring? on a
 * hyperbola?", and stamp a character if so. The trick that makes the rings
 * elliptical is dividing the distance from centre by the stretch factors
 * before measuring how far out we are. */
static void ctx_draw_bg(const GridCtx *g)
{
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double dx = (double)(col - g->ox) * g->cell_w;
            double dy = (double)(row - g->oy) * g->cell_h;
            double u  = dx / g->axis_a;
            double v  = dy / g->axis_b;
            double e_r       = sqrt(u*u + v*v);
            if (e_r < E_R_MIN) continue;
            double ell_theta = atan2(v, u);

            /* On a ring? Count how many ring-gaps out we are; if we're close
             * to a whole number we're sitting on a ring line. */
            double u_ring = e_r / g->ring_spacing;
            double frac   = u_ring - floor(u_ring);
            bool on_ring  = (frac < RING_W_U || frac > 1.0 - RING_W_U);

            /* On a hyperbola? Step the angle evenly and check the same way. */
            bool on_hyper = false;
            if (g->show_hyper) {
                double cv    = fabs(cos(ell_theta));
                double hfrac = fmod(cv, HYPER_STEP);
                on_hyper = (hfrac < HYPER_W || hfrac > HYPER_STEP - HYPER_W);
            }

            if (!on_ring && !on_hyper) continue;

            char c = angle_char(ell_theta);
            if (on_ring && on_hyper) {
                attron(COLOR_PAIR(PAIR_HYPER));
                mvaddch(row, col, (chtype)(unsigned char)'+');
                attroff(COLOR_PAIR(PAIR_HYPER));
            } else if (on_hyper) {
                attron(COLOR_PAIR(PAIR_HYPER));
                mvaddch(row, col, (chtype)(unsigned char)c);
                attroff(COLOR_PAIR(PAIR_HYPER));
            } else {
                attron(COLOR_PAIR(PAIR_GRID));
                mvaddch(row, col, (chtype)(unsigned char)c);
                attroff(COLOR_PAIR(PAIR_GRID));
            }
        }
    }
}

/* ── §5 cursor ── */

/*
 * Cursor — where the @ marker sits, named by grid position rather than pixels.
 *   ring   how many rings out from the centre (0 = innermost)
 *   spoke  which way around, as one of the evenly spaced slots
 * Arrow keys just bump these two numbers; ctx_to_screen turns them into a cell.
 */
typedef struct { int ring, spoke; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->ring  = g->max_ring / 2;
    cur->spoke = 0;
}

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
    ctx_to_screen(g, cur->ring, cur->spoke, &sr, &sc);
    if (sr < 0 || sr >= g->rows - 1 || sc < 0 || sc >= g->cols) return;
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr, sc, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     double fps, bool paused)
{
    char buf[96];
    snprintf(buf, sizeof buf,
             " %.1f fps  A:%.1f B:%.1f  sp:%.0f  ring:%d/%d  spoke:%d  %s%s ",
             fps, g->axis_a, g->axis_b, g->ring_spacing,
             cur->ring, g->max_ring, cur->spoke,
             g->show_hyper ? "+hyper  " : "",
             paused ? "PAUSED" : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme(%d)  r:reset  h:hyper  arrows:move "
             " +/-:spacing  a/z:A  s/x:B ",
             theme + 1);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       double fps, bool paused)
{
    erase();
    ctx_draw_bg(g);
    cursor_draw(cur, g);
    hud_draw(g, cur, theme, fps, paused);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §7 screen ── */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ── §8 app ── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);

    int theme = 0;
    screen_init(theme);

    GridCtx g = {0};
    ctx_init(&g, LINES, COLS);
    Cursor cur;
    cursor_reset(&cur, &g);

    bool paused = false;
    double fps = TARGET_FPS;
    int64_t t0 = clock_ns();
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS);
            if (cur.ring  > g.max_ring)  cur.ring  = g.max_ring;
            if (cur.spoke > g.max_spoke) cur.spoke = g.max_spoke;
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'p': paused = !paused; break;
        case 't': theme = (theme + 1) % N_THEMES; color_init(theme); break;
        case 'r': cursor_reset(&cur, &g); break;
        case 'h': g.show_hyper = !g.show_hyper; break;
        case KEY_UP:    cursor_move(&cur, &g, -1,  0); break;
        case KEY_DOWN:  cursor_move(&cur, &g, +1,  0); break;
        case KEY_LEFT:  cursor_move(&cur, &g,  0, -1); break;
        case KEY_RIGHT: cursor_move(&cur, &g,  0, +1); break;
        case '+': case '=':
            if (g.ring_spacing < RING_SPACING_MAX) {
                g.ring_spacing += RING_SPACING_STEP;
                ctx_init(&g, g.rows, g.cols);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        case '-':
            if (g.ring_spacing > RING_SPACING_MIN) {
                g.ring_spacing -= RING_SPACING_STEP;
                ctx_init(&g, g.rows, g.cols);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        case 'a':
            if (g.axis_a < AXIS_MAX) {
                g.axis_a += AXIS_STEP;
                ctx_init(&g, g.rows, g.cols);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        case 'z':
            if (g.axis_a > AXIS_MIN) {
                g.axis_a -= AXIS_STEP;
                ctx_init(&g, g.rows, g.cols);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        case 's':
            if (g.axis_b < AXIS_MAX) {
                g.axis_b += AXIS_STEP;
                ctx_init(&g, g.rows, g.cols);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        case 'x':
            if (g.axis_b > AXIS_MIN) {
                g.axis_b -= AXIS_STEP;
                ctx_init(&g, g.rows, g.cols);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) +
              (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;
        if (!paused)
            scene_draw(&g, &cur, theme, fps, paused);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
