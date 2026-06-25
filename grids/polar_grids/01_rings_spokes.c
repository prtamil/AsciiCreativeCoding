/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_rings_spokes.c — a polar grid: concentric rings crossed by radial spokes,
 * with an '@' cursor you move by ring and spoke. The grid is never stored; each
 * cell decides what to draw from its distance and angle to the centre.
 *
 * Sister files: 02_log_polar.c (rings that grow with distance),
 *               ../rect_grids/01_uniform_rect.c (same GridCtx template).
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

#define CELL_W        2     /* pixels per char cell (2 wide x 4 tall) — */
#define CELL_H        4     /* makes circles come out round, not oval */

#define RING_SPACING_DEFAULT  20.0f  /* pixels between one ring and the next */
#define RING_SPACING_MIN       8.0f
#define RING_SPACING_MAX      48.0f
#define RING_SPACING_STEP      4.0f
#define RING_W                 1.6f  /* a cell this close to a ring counts as on it */

#define N_SPOKES_DEFAULT  12
#define N_SPOKES_MIN       4
#define N_SPOKES_MAX      36
#define SPOKE_W            0.10      /* angle (radians) within which a cell is on a spoke */
#define SPOKE_MIN_R        3.0f      /* skip spokes near the centre so they don't blob */

#define FPS_EWMA_ALPHA     0.05      /* small = steadier on-screen fps number */

#define PAIR_GRID    1
#define PAIR_CURSOR  2
#define PAIR_HUD     3
#define PAIR_HINT    4

/* one colour per theme: the 256-colour pick, plus a basic-8 fallback */
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
    init_pair(PAIR_GRID,   fg,                              -1);
    init_pair(PAIR_CURSOR, COLORS>=256 ? 226 : COLOR_YELLOW,-1);
    init_pair(PAIR_HUD,    COLORS>=256 ? 226 : COLOR_YELLOW,-1);
    init_pair(PAIR_HINT,   COLORS>=256 ?  51 : COLOR_CYAN,  -1);
}

/* ── §4 polar mapping & lattice ── */

/* GridCtx — the polar grid for one frame: screen size, where the centre sits,
 * and how rings/spokes are spaced. The grid is computed cell-by-cell from these
 * numbers, so this is the single source of truth for the picture and the cursor
 * limits. The grid is always centred on (ox, oy). */
typedef struct {
    int    rows, cols;     /* terminal size in character cells */
    float  ring_spacing;   /* gap between rings, pixels; +/- changes it */
    int    n_spokes;       /* how many spokes radiate out; [/] changes it */
    int    cell_w, cell_h; /* pixels per cell (CELL_W / CELL_H) */
    int    ox, oy;         /* grid centre, as a cell column and row */
    int    max_ring;       /* furthest ring whose cursor cell stays on screen */
    int    max_spoke;      /* n_spokes - 1; the spoke index wraps past it */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows   = rows;
    g->cols   = cols;
    g->cell_w = CELL_W;
    g->cell_h = CELL_H;
    g->ox     = cols / 2;
    g->oy     = rows / 2;
    if (g->ring_spacing <= 0.0f) g->ring_spacing = RING_SPACING_DEFAULT;
    if (g->n_spokes     <= 0)    g->n_spokes     = N_SPOKES_DEFAULT;

    /* biggest ring still on screen, limited by whichever of width/height runs out first */
    double rx = (double)cols * 0.5 * CELL_W;
    double ry = (double)rows * 0.5 * CELL_H;
    double r_visible = (rx < ry ? rx : ry);
    int mr = (int)(r_visible / g->ring_spacing) - 1;
    g->max_ring  = mr < 0 ? 0 : mr;
    g->max_spoke = g->n_spokes - 1;
}

/* recipe step 1 — a (ring, spoke) cell -> the screen cell at its middle (half a
 * ring out, half a wedge round), so the cursor lands centred, not on an edge. */
static void polar_to_screen(const GridCtx *g, int ring, int spoke,
                            int *sr, int *sc)
{
    double mid_radius = ((double)ring + 0.5) * (double)g->ring_spacing;
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
    double a = fmod(theta + 2.0 * M_PI, M_PI);
    if (a < M_PI / 8.0 || a >= 7.0 * M_PI / 8.0) return '-';
    if (a < 3.0 * M_PI / 8.0)                    return '\\';
    if (a < 5.0 * M_PI / 8.0)                    return '|';
    return '/';
}

/* recipe step 2 — draw the grid: each cell knows it's on *some* ring from its
 * distance and on *some* spoke from its angle, with no loop over rings/spokes.
 * Ring + spoke crossing -> '+', one of them -> a line char, neither -> blank. */
static void draw_lattice(const GridCtx *g)
{
    double spoke_angle = 2.0 * M_PI / (double)g->n_spokes;

    attron(COLOR_PAIR(PAIR_GRID));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double r_px, theta;
            screen_to_polar(g, col, row, &r_px, &theta);
            float rf = (float)r_px;   /* ring test is float math, kept exact */

            float ring_phase = fmodf(rf, g->ring_spacing);
            bool on_ring = (ring_phase < RING_W ||
                            ring_phase > g->ring_spacing - RING_W);

            double theta_norm  = fmod(theta + 2.0 * M_PI, 2.0 * M_PI);
            double spoke_phase = fmod(theta_norm, spoke_angle);
            bool on_spoke = (rf > SPOKE_MIN_R) &&
                            (spoke_phase < SPOKE_W ||
                             spoke_phase > spoke_angle - SPOKE_W);

            if (!on_ring && !on_spoke) continue;

            char c = (on_ring && on_spoke) ? '+' : line_glyph(theta);
            mvaddch(row, col, (chtype)(unsigned char)c);
        }
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ── §5 cursor ── */

/* Cursor — the grid cell the user points at: a ring (band out from the centre)
 * and a spoke (wedge round). ring is 0..max_ring; spoke wraps, since a circle
 * has no first or last wedge. */
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
    if (nr < 0)              nr = 0;
    if (nr > g->max_ring)    nr = g->max_ring;
    cur->ring = nr;

    int n = g->n_spokes > 0 ? g->n_spokes : 1;
    int ns = (cur->spoke + d_spoke) % n;
    if (ns < 0) ns += n;
    cur->spoke = ns;
}

/* draw the '@' on the cursor's cell; after the grid so it sits on top */
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
    char buf[96];
    snprintf(buf, sizeof buf,
             " ring:%d spoke:%d  rings:%.0fpx  spokes:%d  th:%d  %5.1f fps  %s ",
             cur->ring, cur->spoke, (double)g->ring_spacing, g->n_spokes,
             theme + 1, fps, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:rings  [/]:spokes ");
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
    g.ring_spacing = RING_SPACING_DEFAULT;
    g.n_spokes     = N_SPOKES_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    bool   paused   = false;
    double fps      = TARGET_FPS;
    int64_t t0      = clock_ns();
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
        case 't':
            theme = (theme + 1) % N_THEMES;
            color_init(theme);
            break;
        case KEY_UP:    cursor_move(&cur, &g, -1,  0); break;
        case KEY_DOWN:  cursor_move(&cur, &g, +1,  0); break;
        case KEY_LEFT:  cursor_move(&cur, &g,  0, -1); break;
        case KEY_RIGHT: cursor_move(&cur, &g,  0, +1); break;
        case '+': case '=':
            if (g.ring_spacing < RING_SPACING_MAX) {
                g.ring_spacing += RING_SPACING_STEP;
                ctx_init(&g, LINES, COLS);
                if (cur.ring > g.max_ring) cur.ring = g.max_ring;
            }
            break;
        case '-':
            if (g.ring_spacing > RING_SPACING_MIN) {
                g.ring_spacing -= RING_SPACING_STEP;
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
