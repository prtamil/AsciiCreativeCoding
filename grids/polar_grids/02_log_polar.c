/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_log_polar.c — a polar grid whose rings grow apart as you move outward.
 *
 * Normal polar grids put rings at even gaps. Here each ring sits a fixed
 * multiple further out than the one before it, so inner rings crowd together
 * and outer rings spread wide — the same way our eyes pack detail near the
 * centre of vision. An '@' marker lives in one (ring, spoke) cell; arrows
 * move it. The sister file 01_rings_spokes.c uses plain even spacing instead.
 *
 * Background reading:
 *   Log-polar coordinates — en.wikipedia.org/wiki/Log-polar_coordinates
 *   SIFT scale-invariant features — Lowe 2004, IJCV 60(2):91–110
 *   Human retinal sampling — Schwartz 1980, Biol. Cybernetics 37(4):199–208
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

#define TARGET_FPS   30
#define CELL_W       2
#define CELL_H       4

/* Innermost ring radius, in pixels. Anything closer to the centre is left blank. */
#define R_MIN               4.0

/* How much bigger each ring is than the one inside it.
 * It's stored as a log-step, so the actual size multiple is e^LOG_STEP:
 * 0.25 means every ring sits about 1.28x further out than the last.
 * The +/- keys nudge it within [0.10, 0.60] — small packs rings tight,
 * large spreads them apart. */
#define LOG_STEP_DEFAULT    0.25
#define LOG_STEP_MIN        0.10
#define LOG_STEP_MAX        0.60
#define LOG_STEP_DELTA      0.05

/* How thick to draw a ring, measured as a fraction of the gap to the next ring
 * (0..1). Using a fraction instead of fixed pixels keeps thin inner rings from
 * vanishing — every ring stays equally visible. */
#define RING_W_U            0.08

#define N_SPOKES_DEFAULT    12
#define N_SPOKES_MIN         4
#define N_SPOKES_MAX        36
#define SPOKE_W             0.10
#define SPOKE_MIN_R         3.0

/* How quickly the FPS readout reacts to change. Small = smooth and steady. */
#define FPS_EWMA_ALPHA      0.05

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

/* ── §4 formula — turn a (ring, spoke) cell into a screen spot, and back ── */

/*
 * GridCtx — everything we need to know to draw the grid and place the cursor.
 * One copy lives for the whole run and gets rebuilt whenever the terminal
 * resizes or the user changes the ring spacing or spoke count.
 */
typedef struct {
    int rows, cols;     /* terminal size, in character cells               */

    double r_min;       /* radius of the innermost ring, in pixels         */
    double log_step;    /* spacing knob: bigger = rings further apart       */
    double ring_w_u;    /* ring thickness, as a fraction of the ring gap   */

    int    n_spokes;    /* how many evenly-spaced spokes radiate out        */
    int    cell_w, cell_h; /* pixels per character cell (chars aren't square) */

    int    ox, oy;      /* centre of the grid, in cell coordinates          */

    int    max_ring;    /* highest ring whose centre still fits on screen   */
    int    max_spoke;   /* highest spoke index (n_spokes - 1)               */
} GridCtx;

/*
 * Recompute the grid layout for the current terminal size. Mostly it works out
 * the centre and the outermost ring that still fits, so the cursor can't wander
 * off-screen. Call it again after any resize or spacing change.
 */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows   = rows;
    g->cols   = cols;
    g->cell_w = CELL_W;
    g->cell_h = CELL_H;
    g->ox     = cols / 2;
    g->oy     = rows / 2;
    if (g->r_min    <= 0.0) g->r_min    = R_MIN;
    if (g->log_step <= 0.0) g->log_step = LOG_STEP_DEFAULT;
    if (g->ring_w_u <= 0.0) g->ring_w_u = RING_W_U;
    if (g->n_spokes <= 0)   g->n_spokes = N_SPOKES_DEFAULT;

    double rx = (double)cols * 0.5 * CELL_W;
    double ry = (double)rows * 0.5 * CELL_H;
    double r_visible = (rx < ry ? rx : ry);
    int mr = 0;
    if (r_visible > g->r_min) {
        double k = log(r_visible / g->r_min) / g->log_step - 0.5;
        mr = (int)floor(k);
        if (mr < 0) mr = 0;
    }
    g->max_ring  = mr;
    g->max_spoke = g->n_spokes - 1;
}

/*
 * Find the screen cell sitting in the middle of one (ring, spoke) cell — that's
 * where the '@' cursor gets drawn. Because rings grow by multiplying, the
 * "middle" radius is half a log-step out, which lands it visually centred in
 * the widening gap rather than hugging the inner edge.
 */
static void ctx_to_screen(const GridCtx *g, int ring, int spoke,
                          int *sr, int *sc)
{
    double mid_radius = g->r_min * exp(((double)ring + 0.5) * g->log_step);
    double theta_mid  = ((double)spoke + 0.5) * (2.0 * M_PI / (double)g->n_spokes);
    double cx = mid_radius * cos(theta_mid);
    double cy = mid_radius * sin(theta_mid);
    *sc = g->ox + (int)round(cx / (double)g->cell_w);
    *sr = g->oy + (int)round(cy / (double)g->cell_h);
}

/*
 * Pick the ASCII character that best mimics a line pointing in direction theta:
 * a dash for near-horizontal, a bar for near-vertical, a slash either way for
 * the diagonals. A line looks the same flipped end-to-end, so we ignore which
 * way it points and only care about its tilt.
 */
static char angle_char(double theta)
{
    double a = fmod(theta + 2.0*M_PI, M_PI);
    if (a < M_PI/8.0 || a >= 7.0*M_PI/8.0) return '-';
    if (a < 3.0*M_PI/8.0)                   return '\\';
    if (a < 5.0*M_PI/8.0)                   return '|';
    return '/';
}

/*
 * Draw the grid by walking every cell on screen and asking two questions:
 * is this cell on a ring, and is it on a spoke? For the ring test we measure
 * how far out the cell is, convert that distance into a ring count (a few rings
 * close in, then sparser as we go out), and check whether it's right on top of
 * a whole ring. Cells on both a ring and a spoke get a '+'; otherwise they get
 * a line character; everything else stays blank.
 */
static void ctx_draw_bg(const GridCtx *g)
{
    double spoke_angle = 2.0 * M_PI / (double)g->n_spokes;

    attron(COLOR_PAIR(PAIR_GRID));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double dx = (double)(col - g->ox) * g->cell_w;
            double dy = (double)(row - g->oy) * g->cell_h;
            double r_px = sqrt(dx*dx + dy*dy);
            double theta = atan2(dy, dx);

            bool on_ring = false;
            if (r_px > g->r_min) {
                double u = log(r_px / g->r_min) / g->log_step;
                double frac = u - floor(u);
                on_ring = (frac < g->ring_w_u || frac > 1.0 - g->ring_w_u);
            }

            double theta_norm  = fmod(theta + 2.0*M_PI, 2.0*M_PI);
            double spoke_phase = fmod(theta_norm, spoke_angle);
            bool on_spoke = (r_px > SPOKE_MIN_R) &&
                            (spoke_phase < SPOKE_W ||
                             spoke_phase > spoke_angle - SPOKE_W);

            if (!on_ring && !on_spoke) continue;

            char c = (on_ring && on_spoke) ? '+' : angle_char(theta);
            mvaddch(row, col, (chtype)(unsigned char)c);
        }
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ── §5 cursor ── */

/*
 * Where the '@' marker currently sits, as a grid address rather than a pixel
 * spot: which ring (counting out from the centre) and which spoke (counting
 * around). ctx_to_screen turns this pair into an actual screen cell.
 */
typedef struct {
    int ring;   /* 0 = innermost ring, up to g->max_ring                  */
    int spoke;  /* 0..n_spokes-1, going around; wraps past the last       */
} Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->ring  = g->max_ring / 2;
    cur->spoke = 0;
}

/*
 * Step the marker by some rings and spokes. Rings stop at the edges so it can't
 * leave the grid; spokes wrap around, since going past the last spoke just
 * brings you back to the first.
 */
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
    char buf[96];
    double ratio = exp(g->log_step);
    snprintf(buf, sizeof buf,
             " ring:%d spoke:%d  ratio:%.2f  spokes:%d  th:%d  %5.1f fps  %s ",
             cur->ring, cur->spoke, ratio, g->n_spokes,
             theme + 1, fps, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:ring-ratio  [/]:spokes ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       bool paused, double fps)
{
    erase();
    ctx_draw_bg(g);
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
    g.r_min    = R_MIN;
    g.log_step = LOG_STEP_DEFAULT;
    g.ring_w_u = RING_W_U;
    g.n_spokes = N_SPOKES_DEFAULT;
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
            if (g.log_step < LOG_STEP_MAX) {
                g.log_step += LOG_STEP_DELTA;
                ctx_init(&g, LINES, COLS);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        case '-':
            if (g.log_step > LOG_STEP_MIN) {
                g.log_step -= LOG_STEP_DELTA;
                ctx_init(&g, LINES, COLS);
            }
            break;
        case '[':
            if (g.n_spokes > N_SPOKES_MIN) {
                g.n_spokes -= (g.n_spokes > 8 ? 4 : 2);
                ctx_init(&g, LINES, COLS);
                if (cur.spoke > g.max_spoke) cur.spoke = g.max_spoke;
            }
            break;
        case ']':
            if (g.n_spokes < N_SPOKES_MAX) {
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
