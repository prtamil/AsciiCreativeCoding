/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 04_log_spiral.c — a spiral grid whose coils spread wider the farther out
 * they go.  This is the shape you see in nautilus shells, galaxy arms, and
 * sunflower seed heads.  Press 'g' for the "golden spiral" preset that nature
 * tends to land on; the '@' cursor walks along one arm so you can probe it.
 *
 * Sister files: 03_archimedean_spiral.c is the cousin whose coils stay evenly
 * spaced; 05_sunflower.c shows the dot-pattern version of the same idea.
 *
 * References: en.wikipedia.org/wiki/Logarithmic_spiral and
 * /wiki/Golden_spiral; Prusinkiewicz & Lindenmayer, "The Algorithmic Beauty
 * of Plants" (1990), ch. 4, on why plants pick this spiral.
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

/* How fast the spiral fans out: bigger = coils race apart, smaller = tight
 * wind.  The golden-spiral value below is the one nature favours. */
#define GROWTH_DEFAULT  0.18
#define GROWTH_MIN      0.05
#define GROWTH_MAX      0.80
#define GROWTH_STEP     0.02

/* The golden ratio, and the growth rate that makes the golden spiral. */
#define PHI             1.61803398874989484820
#define GROWTH_GOLDEN   (2.0 * log(PHI) / M_PI)   /* ≈ 0.3065 */

/* We don't draw the very centre — the coils pile up too tightly to read. */
#define R_MIN           4.0

/* How thick to draw each arm.  Wider = fatter spiral lines. */
#define SPIRAL_W        0.22

/* How many spiral arms to draw. */
#define N_ARMS_DEFAULT   2
#define N_ARMS_MIN       1
#define N_ARMS_MAX       8

/* How many stops the cursor can make as it walks around one coil. */
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

/* ── §4 formula — where the spiral lives on screen ── */

/*
 * GridCtx — everything we need to know to draw the spiral and to keep the
 * cursor inside it.  Built once from the terminal size, rebuilt on resize or
 * when the growth rate changes.
 */
typedef struct {
    int rows, cols;        /* terminal size, in character cells             */

    double a;              /* growth rate: how fast coils fan out           */
    double r_min;          /* radius of the blank centre we don't draw      */
    int    n_arms;         /* how many spiral arms                          */
    int    cell_w, cell_h; /* width/height of one cell, in our pixel units  */

    int    ox, oy;         /* screen cell the spiral centre sits on         */

    int    max_turn;       /* furthest coil the cursor can reach before     */
                           /*   it would fall off the visible area          */
    int    max_spoke;      /* last stop around one coil (CURSOR_SPOKES − 1) */
} GridCtx;

/* Recompute the centre and how far out the cursor may roam for the current
 * terminal size and growth rate.  Call it on startup, on resize, and any time
 * the growth changes. */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows   = rows;
    g->cols   = cols;
    g->cell_w = CELL_W;
    g->cell_h = CELL_H;
    g->ox     = cols / 2;
    g->oy     = rows / 2;
    if (g->a      <= 0.0) g->a      = GROWTH_DEFAULT;
    if (g->r_min  <= 0.0) g->r_min  = R_MIN;
    if (g->n_arms <= 0)   g->n_arms = N_ARMS_DEFAULT;

    double rx = (double)cols * 0.5 * CELL_W;
    double ry = (double)rows * 0.5 * CELL_H;
    double r_visible = (rx < ry ? rx : ry);
    int mt = 0;
    if (r_visible > g->r_min) {
        double k = log(r_visible / g->r_min) / (g->a * 2.0 * M_PI) - 0.5;
        mt = (int)floor(k);
        if (mt < 0) mt = 0;
    }
    g->max_turn  = mt;
    g->max_spoke = CURSOR_SPOKES - 1;
}

/* Given a (turn, spoke) spot on the first arm, find which screen cell it lands
 * on.  Turn says which coil, spoke says how far around that coil; from those we
 * get an angle, the spiral law gives the radius, and we round to a cell. */
static void ctx_to_screen(const GridCtx *g, int turn, int spoke,
                          int *sr, int *sc)
{
    double theta_sample = ((double)turn +
                           ((double)spoke + 0.5) / (double)CURSOR_SPOKES)
                          * 2.0 * M_PI;
    double r_sample = g->r_min * exp(g->a * theta_sample);
    double cx = r_sample * cos(theta_sample);
    double cy = r_sample * sin(theta_sample);
    *sc = g->ox + (int)round(cx / (double)g->cell_w);
    *sr = g->oy + (int)round(cy / (double)g->cell_h);
}

/* Pick the slash, pipe, or dash that best matches the direction the spiral is
 * heading at this point, so the curve reads as a smooth line instead of dots. */
static char angle_char(double theta)
{
    double a = fmod(theta + 2.0*M_PI, M_PI);
    if (a < M_PI/8.0 || a >= 7.0*M_PI/8.0) return '-';
    if (a < 3.0*M_PI/8.0)                   return '\\';
    if (a < 5.0*M_PI/8.0)                   return '|';
    return '/';
}

/* Draw the spiral by asking every cell one question: "are you sitting on an
 * arm?"  For each cell we work out its distance and angle from the centre,
 * then check how far its angle is from where an arm should be at that distance.
 * Close enough, and we draw it. */
static void ctx_draw_bg(const GridCtx *g)
{
    double two_pi = 2.0 * M_PI;

    attron(COLOR_PAIR(PAIR_GRID));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double dx = (double)(col - g->ox) * g->cell_w;
            double dy = (double)(row - g->oy) * g->cell_h;
            double r_px = sqrt(dx*dx + dy*dy);
            if (r_px < g->r_min) continue;

            double theta = atan2(dy, dx);
            double theta_norm = fmod(theta + two_pi, two_pi);

            /* Where should an arm be, angle-wise, at this distance?  "phase" is
             * how far this cell's angle strays from that; near zero means it's
             * right on an arm.  The wrap keeps the answer in a clean range. */
            double theta_pred = log(r_px / g->r_min) / g->a;
            double raw   = (double)g->n_arms * (theta_norm - theta_pred);
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
 * Cursor — where the '@' probe sits on the first arm.
 *   turn  — which coil, counting outward from the centre.
 *   spoke — how far around that coil, in CURSOR_SPOKES even steps.
 * The arrow keys nudge these two numbers; ctx_to_screen turns them into a cell.
 */
typedef struct { int turn, spoke; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->turn  = g->max_turn / 2;
    cur->spoke = 0;
}

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

static void hud_draw(const GridCtx *g, const Cursor *cur, bool golden,
                     int theme, bool paused, double fps)
{
    char buf[112];
    snprintf(buf, sizeof buf,
             " turn:%d spoke:%d  a:%.4f%s  arms:%d  th:%d  %5.1f fps  %s ",
             cur->turn, cur->spoke, g->a, golden ? "(g)" : "",
             g->n_arms, theme + 1, fps, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  g:golden  arrows:move  +/-:growth  [/]:arms ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, bool golden,
                       int theme, bool paused, double fps)
{
    erase();
    ctx_draw_bg(g);
    cursor_draw(cur, g);
    hud_draw(g, cur, golden, theme, paused, fps);
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
    g.a      = GROWTH_DEFAULT;
    g.r_min  = R_MIN;
    g.n_arms = N_ARMS_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    bool   golden = false;
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
        case 'g':
            golden = !golden;
            g.a = golden ? GROWTH_GOLDEN : GROWTH_DEFAULT;
            ctx_init(&g, LINES, COLS);
            if (cur.turn > g.max_turn) cur.turn = g.max_turn;
            break;
        case '+': case '=':
            if (g.a < GROWTH_MAX) {
                g.a += GROWTH_STEP;
                golden = false;
                ctx_init(&g, LINES, COLS);
                if (cur.turn > g.max_turn) cur.turn = g.max_turn;
            }
            break;
        case '-':
            if (g.a > GROWTH_MIN) {
                g.a -= GROWTH_STEP;
                golden = false;
                ctx_init(&g, LINES, COLS);
            }
            break;
        case '[':
            if (g.n_arms > N_ARMS_MIN) g.n_arms--;
            break;
        case ']':
            if (g.n_arms < N_ARMS_MAX) g.n_arms++;
            break;
        case KEY_UP:    cursor_move(&cur, &g, -1,  0); break;
        case KEY_DOWN:  cursor_move(&cur, &g, +1,  0); break;
        case KEY_LEFT:  cursor_move(&cur, &g,  0, -1); break;
        case KEY_RIGHT: cursor_move(&cur, &g,  0, +1); break;
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) +
              (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;
        if (!paused)
            scene_draw(&g, &cur, golden, theme, paused, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
