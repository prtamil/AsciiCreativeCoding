/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_archimedean_spiral.c — an Archimedean spiral grid.
 *
 * One or more spiral arms wind outward, and the gap between turns stays the
 * same no matter how far out you go (that's what "Archimedean" means). A '@'
 * cursor rides the spiral; arrows move it, +/- tighten or loosen the coil,
 * [/] add or remove arms.
 *
 * Study alongside: 04_log_spiral.c (gap grows as you go out),
 *                  01_rings_spokes.c (evenly spaced rings, same idea),
 *                  ../rect_grids/01_uniform_rect.c (the GridCtx template).
 *
 * Reference: en.wikipedia.org/wiki/Archimedean_spiral
 */

/* ── HOW THE SPIRAL IS DRAWN ──
 *
 * A single Archimedean spiral arm follows r = a·θ: the radius grows in step
 * with the angle. So a point sits on the arm when its angle θ matches r/a.
 *
 * We test every cell: how far is its actual angle from r/a? Call that gap the
 * "phase". Phase near zero means the cell lands on the arm — draw it.
 *
 * The arm-count trick: for N evenly spaced arms, multiply the phase by N
 * before testing. That lines up all N arms onto the same target at once, so a
 * single test catches every arm with no loop. (The reason: arm k sits exactly
 * k·2π/N off the base arm, and multiplying by N turns that offset into a whole
 * number of full turns, which the wrap-around test reads as "on the arm".)
 *
 * The pitch is the pixel gap between successive turns. We store it directly in
 * pixels; a = pitch / (2π) is just how much the radius grows per radian.
 *
 * Watch out for:
 *   • Very near the centre the angle-vs-r/a math swings wildly, so we skip
 *     cells closer than MIN_R to avoid a smeary blob at the origin.
 *   • With many arms each arm gets thin (its width is shared N ways). Bump
 *     SPIRAL_W if arms start vanishing.
 *   • The cursor's reach (max_turn) depends on screen size and pitch, so it's
 *     recomputed on resize and whenever pitch changes.
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

/* Pitch: the pixel gap between one turn of the spiral and the next. */
#define PITCH_DEFAULT   32.0
#define PITCH_MIN        8.0
#define PITCH_MAX       80.0
#define PITCH_STEP       4.0

/* How thick the spiral line is — bigger means a fatter, easier-to-see arm. */
#define SPIRAL_W        0.20

/* Number of spiral arms */
#define N_ARMS_DEFAULT   1
#define N_ARMS_MIN       1
#define N_ARMS_MAX       8

/* Skip cells this close to the centre — keeps the origin from smearing. */
#define MIN_R            3.0

/* How many stops the cursor can land on per full turn of the spiral. */
#define CURSOR_SPOKES    12

/* Smooths the FPS number so it doesn't jitter every frame. */
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

/* ── §4 formula — GridCtx and turning a spiral spot into a screen cell ── */

/*
 * GridCtx — everything we need to draw the spiral and to know where the
 * cursor is allowed to go. Built once from the terminal size, then reused.
 */
typedef struct {
    int rows, cols;        /* terminal size, in character cells              */

    double pitch;          /* pixel gap between one turn and the next        */
    double a;              /* how fast the radius grows per radian; pitch/2π */
    int    n_arms;         /* how many arms wind out (1..N_ARMS_MAX)         */
    int    cell_w, cell_h; /* pixels per cell, since cells aren't square     */

    int    ox, oy;         /* the centre cell the spiral winds out from      */

    int    max_turn;       /* farthest turn the cursor can reach on-screen   */
    int    max_spoke;      /* highest spoke index per turn (SPOKES − 1)      */
} GridCtx;

/*
 * Sets up the spiral for the current terminal size. The interesting part is
 * max_turn: we figure out how far the spiral can reach before it runs off the
 * shorter edge of the screen, then cap the cursor there.
 */
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

    double rx = (double)cols * 0.5 * CELL_W;
    double ry = (double)rows * 0.5 * CELL_H;
    double r_visible = (rx < ry ? rx : ry);
    int mt = (int)(r_visible / g->pitch - 0.5);
    g->max_turn  = mt < 0 ? 0 : mt;
    g->max_spoke = CURSOR_SPOKES - 1;
}

/*
 * Turns a cursor address (turn, spoke) into the screen cell it sits on. We
 * pick the matching angle on the spiral, walk out to the radius the spiral
 * has there, then convert that point to a row/col. The half-step nudge
 * (spoke + 0.5) centres the cursor in its slice so neighbouring spokes don't
 * land on the same cell.
 */
static void ctx_to_screen(const GridCtx *g, int turn, int spoke,
                          int *sr, int *sc)
{
    double theta_sample = ((double)turn +
                           ((double)spoke + 0.5) / (double)CURSOR_SPOKES)
                          * 2.0 * M_PI;
    double r_sample = g->a * theta_sample;
    double cx = r_sample * cos(theta_sample);
    double cy = r_sample * sin(theta_sample);
    *sc = g->ox + (int)round(cx / (double)g->cell_w);
    *sr = g->oy + (int)round(cy / (double)g->cell_h);
}

/*
 * Picks the ASCII character that best matches the slope at this point, so the
 * spiral looks like a line and not a field of dots: nearly flat → '-', steep
 * → '|', and '/' or '\' for the diagonals in between.
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
 * Walks every cell and draws the ones that land on a spiral arm. For each
 * cell we find its distance and angle from the centre, ask "does this angle
 * match where an arm should be at this radius?", and if so draw a line
 * character there. The N-arm trick (see the top of the file) lets one test
 * catch all arms at once.
 */
static void ctx_draw_bg(const GridCtx *g)
{
    double two_pi = 2.0 * M_PI;
    double a = g->a;

    attron(COLOR_PAIR(PAIR_GRID));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double dx = (double)(col - g->ox) * g->cell_w;
            double dy = (double)(row - g->oy) * g->cell_h;
            double r_px = sqrt(dx*dx + dy*dy);
            if (r_px < MIN_R) continue;

            double theta = atan2(dy, dx);
            double theta_norm = fmod(theta + two_pi, two_pi);

            /* Multiply the angle gap by the arm count, then wrap it. Cells on
             * any arm land near zero here; everything else is far from it. */
            double raw = (double)g->n_arms * (theta_norm - r_px / a);
            double phase = fmod(raw + (double)g->n_arms * two_pi, two_pi);

            if (phase < SPIRAL_W || phase > two_pi - SPIRAL_W) {
                char c = angle_char(theta);
                mvaddch(row, col, (chtype)(unsigned char)c);
            }
        }
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ── §5 cursor ── */

/*
 * Cursor — where the '@' marker sits, named by how far along the spiral it is
 * rather than by row/col. Pair it with a GridCtx and ctx_to_screen() turns it
 * into an actual screen cell.
 *
 * turn  : which loop of the spiral, counting out from the centre.
 * spoke : which stop within that loop (0 .. CURSOR_SPOKES − 1).
 */
typedef struct { int turn, spoke; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->turn  = g->max_turn / 2;
    cur->spoke = 0;
}

/*
 * Nudges the cursor. Turn is clamped to the visible range (so it can't fly
 * off-screen); spoke wraps around, so stepping past the last stop in a loop
 * lands back at the first.
 */
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

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->turn, cur->spoke, &sr, &sc);
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
