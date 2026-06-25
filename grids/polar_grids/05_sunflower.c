/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 05_sunflower.c — the spiral dot pattern at the centre of a sunflower.
 *
 * Drops seeds one by one around a centre, each a bit farther out and turned
 * by the "golden angle." That single turn is what makes the dots settle into
 * the interlocking spirals you see in sunflowers, pinecones, and daisies. An
 * '@' marker sits on one chosen seed; arrows move it, and 'g' swaps in other
 * turn angles so you can watch the nice pattern fall apart.
 *
 * Sister files: 04_log_spiral.c, 03_archimedean_spiral.c (other spiral shapes),
 * ../rect_grids/01_uniform_rect.c (where the GridCtx layout came from).
 * Background: Vogel H (1979), "A better way to construct the sunflower head,"
 * Mathematical Biosciences 44(3–4):179–189; en.wikipedia.org/wiki/Golden_angle.
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

#define TARGET_FPS         30
#define CELL_W             2
#define CELL_H             4

/* The golden ratio, and the turn angle that gives every seed a fresh
 * direction (about 137.5°). This special turn is the whole trick. */
#define PHI                1.61803398874989484820
#define GOLDEN_ANGLE       (2.0 * M_PI / (PHI * PHI))

/* How far apart the seeds sit — bigger means a bigger flower head. */
#define SPACING_DEFAULT    3.5
#define SPACING_MIN        1.5
#define SPACING_MAX        8.0
#define SPACING_STEP       0.5

/* How many seeds to draw. */
#define N_SEEDS_DEFAULT    800
#define N_SEEDS_MIN        100
#define N_SEEDS_MAX       4096
#define N_SEEDS_STEP       100

/* Turn angles the 'g' key cycles through, to compare against the golden one.
 * Most of these are "tidy" fractions of a full turn, which is exactly why
 * they make ugly spokes instead of the even spiral fill. */
static const double ANGLE_TABLE[] = {
    2.0 * M_PI / (PHI * PHI),    /* the golden angle — fills evenly      */
    2.0 * M_PI / 5.0,            /* 72° — seeds line up into 5 spokes     */
    2.0 * M_PI / 8.0,            /* 45° — 8 spokes                        */
    2.0 * M_PI * (1.0 - 1.0/PHI),/* same as golden, just measured the other way */
    2.0 * M_PI * 0.382,          /* close to golden but not quite — gaps appear */
};
#define N_ANGLES  5

/* The character drawn for each seed. */
#define SEED_CHAR  'o'

/* How heavily we smooth the FPS number so it doesn't jitter every frame. */
#define FPS_EWMA_ALPHA     0.05

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

/* ── §4 formula — turning a seed number into a screen cell ── */

/*
 * GridCtx — everything needed to lay out the flower in the current terminal.
 *
 * The three values that shape the pattern are spacing, angle, and n_seeds;
 * the rest is bookkeeping so we always know where the centre is and which
 * seed numbers are valid.
 */
typedef struct {
    int rows, cols;        /* terminal size in character cells              */

    double spacing;        /* how far apart seeds sit (bigger = larger head)*/
    double angle;          /* turn between seeds, in radians                */
    int    n_seeds;        /* how many seeds to draw                        */
    int    cell_w, cell_h; /* a cell is taller than wide; used to keep circles round */

    int    ox, oy;         /* centre of the flower, in cells                */

    int    max_n;          /* highest valid seed number (n_seeds − 1)       */
} GridCtx;

/*
 * Recompute the layout for the current terminal size. We don't track an
 * outer edge — seeds that land off-screen are simply skipped while drawing.
 */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows   = rows;
    g->cols   = cols;
    g->cell_w = CELL_W;
    g->cell_h = CELL_H;
    g->ox     = cols / 2;
    g->oy     = rows / 2;
    if (g->spacing <= 0.0) g->spacing = SPACING_DEFAULT;
    if (g->angle   <= 0.0) g->angle   = GOLDEN_ANGLE;
    if (g->n_seeds <= 0)   g->n_seeds = N_SEEDS_DEFAULT;
    g->max_n = g->n_seeds - 1;
}

/*
 * Where on screen does seed n land? Seed n sits at distance √n × spacing from
 * the centre, turned n times by the angle. We turn that distance-and-direction
 * into a row and column. Dividing by the cell width and height makes up for
 * cells being taller than they are wide, so the flower looks round, not squashed.
 */
static void ctx_to_screen(const GridCtx *g, int n, int *sr, int *sc)
{
    double r_n     = sqrt((double)n) * g->spacing;
    double theta_n = (double)n * g->angle;
    *sc = g->ox + (int)round(r_n * cos(theta_n) / (double)g->cell_w);
    *sr = g->oy + (int)round(r_n * sin(theta_n) / (double)g->cell_h);
}

/*
 * Draw the whole flower: walk through every seed number, work out its cell,
 * and place a character there. Two seeds can round to the same cell; we let
 * the first one win and quietly skip the second so the dot doesn't get
 * overwritten. The √i spacing is what keeps the dots evenly packed all the
 * way out, instead of crowding near the edge.
 */
static void ctx_draw_bg(const GridCtx *g)
{
    int rows = g->rows, cols = g->cols;
    /* Tracks which cells already have a seed, so we don't draw one twice. */
    bool visited[rows][cols];
    memset(visited, 0, sizeof(bool) * (size_t)(rows * cols));

    attron(COLOR_PAIR(PAIR_GRID));
    for (int i = 0; i < g->n_seeds; i++) {
        double r   = sqrt((double)i) * g->spacing;
        double th  = (double)i * g->angle;
        int sc = g->ox + (int)round(r * cos(th) / (double)g->cell_w);
        int sr = g->oy + (int)round(r * sin(th) / (double)g->cell_h);

        if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) continue;
        if (visited[sr][sc]) continue;
        visited[sr][sc] = true;
        mvaddch(sr, sc, (chtype)(unsigned char)SEED_CHAR);
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ── §5 cursor ── */

/*
 * Cursor — just the number of the seed we're pointing at.
 *
 * That single number is the whole address: ask GridCtx where seed n is and
 * you get its spot on screen. The valid range and all the geometry live in
 * GridCtx, so the cursor itself stays this simple.
 */
typedef struct { int n; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->n = g->max_n / 4;     /* start partway out — not the centre, not the edge */
}

/*
 * Move the marker and keep it on a real seed.
 *
 * Left/right step to the neighbouring seed number. Up/down jump by several at
 * once: neighbouring seed numbers land on opposite sides of the flower, so a
 * single step looks like a teleport — a bigger jump instead traces along one
 * of the visible spiral arms.
 */
#define CURSOR_RING_STEP 8
static void cursor_move(Cursor *cur, const GridCtx *g, int d_n)
{
    int nn = cur->n + d_n;
    if (nn < 0)         nn = 0;
    if (nn > g->max_n)  nn = g->max_n;
    cur->n = nn;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->n, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, (chtype)'@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ── */

static void hud_draw(const GridCtx *g, const Cursor *cur, int angle_idx,
                     int theme, bool paused, double fps)
{
    const char *angle_name = (angle_idx == 0) ? "golden" :
                             (angle_idx == 1) ? "72deg"  :
                             (angle_idx == 2) ? "45deg"  :
                             (angle_idx == 3) ? "222.5d" : "near-g";
    char buf[112];
    snprintf(buf, sizeof buf,
             " seed:%d/%d  sp:%.1f  ang:%s  th:%d  %5.1f fps  %s ",
             cur->n, g->n_seeds, g->spacing, angle_name,
             theme + 1, fps, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  g:cycle-angle  arrows:move  +/-:spacing  [/]:seeds ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int angle_idx,
                       int theme, bool paused, double fps)
{
    erase();
    ctx_draw_bg(g);
    cursor_draw(cur, g);
    hud_draw(g, cur, angle_idx, theme, paused, fps);
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
    int angle_idx = 0;
    screen_init();
    color_init(theme);

    GridCtx g = {0};
    g.spacing = SPACING_DEFAULT;
    g.angle   = ANGLE_TABLE[angle_idx];
    g.n_seeds = N_SEEDS_DEFAULT;
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
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'p': paused = !paused; break;
        case 'r': cursor_reset(&cur, &g); break;
        case 't': theme = (theme + 1) % N_THEMES; color_init(theme); break;
        case 'g':
            angle_idx = (angle_idx + 1) % N_ANGLES;
            g.angle = ANGLE_TABLE[angle_idx];
            break;
        case KEY_UP:    cursor_move(&cur, &g, -CURSOR_RING_STEP); break;
        case KEY_DOWN:  cursor_move(&cur, &g, +CURSOR_RING_STEP); break;
        case KEY_LEFT:  cursor_move(&cur, &g, -1);                break;
        case KEY_RIGHT: cursor_move(&cur, &g, +1);                break;
        case '+': case '=':
            if (g.spacing < SPACING_MAX) g.spacing += SPACING_STEP;
            break;
        case '-':
            if (g.spacing > SPACING_MIN) g.spacing -= SPACING_STEP;
            break;
        case '[':
            if (g.n_seeds > N_SEEDS_MIN) {
                g.n_seeds -= N_SEEDS_STEP;
                ctx_init(&g, LINES, COLS);
                if (cur.n > g.max_n) cur.n = g.max_n;
            }
            break;
        case ']':
            if (g.n_seeds < N_SEEDS_MAX) {
                g.n_seeds += N_SEEDS_STEP;
                ctx_init(&g, LINES, COLS);
            }
            break;
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) +
              (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;
        if (!paused)
            scene_draw(&g, &cur, angle_idx, theme, paused, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
