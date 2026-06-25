/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_archimedean_spiral.c — an Archimedean spiral grid: one or more arms wind
 * outward with a constant gap between turns (r = a*theta). The grid is never
 * stored; each cell decides if it lands on an arm from its distance and angle.
 *
 * Sister files: 04_log_spiral.c (gap grows with distance),
 *               01_rings_spokes.c (same GridCtx + cursor template).
 * Reference: en.wikipedia.org/wiki/Archimedean_spiral
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

#define CELL_W          2     /* pixels per char cell (2 wide x 4 tall) — */
#define CELL_H          4     /* makes the spiral come out round, not oval */

#define PITCH_DEFAULT   32.0  /* pixel gap between one turn and the next */
#define PITCH_MIN        8.0
#define PITCH_MAX       80.0
#define PITCH_STEP       4.0

#define SPIRAL_W        0.20  /* phase width within which a cell is on an arm */

#define N_ARMS_DEFAULT   1
#define N_ARMS_MIN       1
#define N_ARMS_MAX       8

#define MIN_R            3.0  /* skip cells nearer the centre so it doesn't smear */

#define CURSOR_SPOKES    12   /* cursor stops per full turn of the spiral */

#define FPS_EWMA_ALPHA   0.05 /* small = steadier on-screen fps number */

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

/* GridCtx — the spiral grid for one frame: screen size, where the centre sits,
 * and how tight the coil is. The grid is computed cell-by-cell from these
 * numbers, the single source of truth for the picture and the cursor limits.
 * The grid is always centred on (ox, oy). */
typedef struct {
    int    rows, cols;     /* terminal size in character cells */
    double pitch;          /* gap between turns, pixels; +/- changes it */
    double a;              /* radius grown per radian; pitch / 2π */
    int    n_arms;         /* how many arms wind out; [/] changes it */
    int    cell_w, cell_h; /* pixels per cell (CELL_W / CELL_H) */
    int    ox, oy;         /* grid centre, as a cell column and row */
    int    max_turn;       /* furthest turn whose cursor cell stays on screen */
    int    max_spoke;      /* CURSOR_SPOKES - 1; the spoke index wraps past it */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows   = rows;
    g->cols   = cols;
    g->cell_w = CELL_W;
    g->cell_h = CELL_H;
    g->ox     = cols / 2;
    g->oy     = rows / 2;
    if (g->pitch  <= 0.0) g->pitch  = PITCH_DEFAULT;
    if (g->n_arms <= 0)   g->n_arms = N_ARMS_DEFAULT;
    g->a = g->pitch / (2.0 * M_PI);

    /* biggest turn still on screen, limited by whichever of width/height runs out first */
    double rx = (double)cols * 0.5 * CELL_W;
    double ry = (double)rows * 0.5 * CELL_H;
    double r_visible = (rx < ry ? rx : ry);
    int mt = (int)(r_visible / g->pitch - 0.5);
    g->max_turn  = mt < 0 ? 0 : mt;
    g->max_spoke = CURSOR_SPOKES - 1;
}

/* DISTINCT MATH — the Archimedean spiral: radius grows linearly with angle. */
static double spiral_radius(double a, double theta)
{
    return a * theta;
}

/* recipe step 1 — a (turn, spoke) cell -> the screen cell on the arm. We pick
 * the matching angle, walk out to the spiral's radius there, then convert to a
 * row/col. The half-step (spoke + 0.5) centres the cursor in its slice. */
static void polar_to_screen(const GridCtx *g, int turn, int spoke,
                            int *sr, int *sc)
{
    double theta_sample = ((double)turn +
                           ((double)spoke + 0.5) / (double)CURSOR_SPOKES)
                          * 2.0 * M_PI;
    double r_sample = spiral_radius(g->a, theta_sample);
    double cx = r_sample * cos(theta_sample);
    double cy = r_sample * sin(theta_sample);
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

/* DISTINCT MATH — is this cell on a spiral arm? An arm sits where the cell's
 * angle matches r/a (radius back-solved for angle). The gap between the two is
 * the "phase". Multiplying the gap by n_arms folds all N arms onto one target,
 * so a single wrap-around test near zero catches every arm with no loop. */
static bool on_spiral(const GridCtx *g, double r_px, double theta)
{
    double two_pi = 2.0 * M_PI;
    double theta_norm = fmod(theta + two_pi, two_pi);
    double raw   = (double)g->n_arms * (theta_norm - r_px / g->a);
    double phase = fmod(raw + (double)g->n_arms * two_pi, two_pi);
    return phase < SPIRAL_W || phase > two_pi - SPIRAL_W;
}

/* recipe step 2 — draw the grid: each cell knows if it lands on *some* arm from
 * its distance and angle, with no loop over arms. On an arm -> a line char,
 * off it -> blank. Cells nearer than MIN_R are skipped (angle swings wildly). */
static void draw_lattice(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_GRID));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double r_px, theta;
            screen_to_polar(g, col, row, &r_px, &theta);
            if (r_px < MIN_R) continue;

            if (on_spiral(g, r_px, theta)) {
                char c = line_glyph(theta);
                mvaddch(row, col, (chtype)(unsigned char)c);
            }
        }
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ── §5 cursor ── */

/* Cursor — the spiral cell the user points at, named by how far along the
 * spiral it is: a turn (loop out from the centre) and a spoke (stop within that
 * loop). turn is 0..max_turn; spoke wraps, since a loop has no first or last. */
typedef struct { int turn, spoke; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->turn  = g->max_turn / 2;
    cur->spoke = 0;
}

/* recipe step 3 — move the cursor: in/out clamps at the edge, round wraps */
static void cursor_move(Cursor *cur, const GridCtx *g, int d_turn, int d_spoke)
{
    int nt = cur->turn + d_turn;
    if (nt < 0)            nt = 0;
    if (nt > g->max_turn)  nt = g->max_turn;
    cur->turn = nt;

    int n = CURSOR_SPOKES;
    int ns = (cur->spoke + d_spoke) % n;
    if (ns < 0) ns += n;
    cur->spoke = ns;
}

/* draw the '@' on the cursor's cell; after the grid so it sits on top */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    polar_to_screen(g, cur->turn, cur->spoke, &sr, &sc);
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
             " turn:%d spoke:%d  pitch:%.0fpx  arms:%d  th:%d  %5.1f fps  %s ",
             cur->turn, cur->spoke, g->pitch, g->n_arms,
             theme + 1, fps, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:pitch  [/]:arms ");
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
    g.pitch  = PITCH_DEFAULT;
    g.n_arms = N_ARMS_DEFAULT;
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
            if (g.pitch < PITCH_MAX) {
                g.pitch += PITCH_STEP;
                ctx_init(&g, LINES, COLS);
                if (cur.turn > g.max_turn) cur.turn = g.max_turn;
            }
            break;
        case '-':
            if (g.pitch > PITCH_MIN) {
                g.pitch -= PITCH_STEP;
                ctx_init(&g, LINES, COLS);
            }
            break;
        case '[':
            if (g.n_arms > N_ARMS_MIN) g.n_arms--;
            break;
        case ']':
            if (g.n_arms < N_ARMS_MAX) g.n_arms++;
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
