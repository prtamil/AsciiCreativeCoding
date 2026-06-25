/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 05_sunflower.c — phyllotaxis: seed n sits at r = spacing·√n, turned n·θ
 * round, where θ is the golden angle (~137.5°). That one turn per seed packs
 * the dots into the interlocking spirals of a sunflower head. An '@' marks one
 * seed (arrows move it); 'g' swaps in other turn angles to break the pattern.
 *
 * Sister files: 04_log_spiral.c, 03_archimedean_spiral.c (other spirals),
 *               01_rings_spokes.c (shared GridCtx / cursor template).
 * Ref: Vogel H (1979), Math. Biosciences 44:179–189; the golden angle.
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

#define CELL_W             2   /* pixels per char cell (2 wide x 4 tall) — */
#define CELL_H             4   /* keeps the flower head round, not oval */

#define PHI                1.61803398874989484820   /* golden ratio */
#define GOLDEN_ANGLE       (2.0 * M_PI / (PHI * PHI))  /* ~137.5°, the magic turn */

#define SPACING_DEFAULT    3.5   /* pixels between successive seeds */
#define SPACING_MIN        1.5
#define SPACING_MAX        8.0
#define SPACING_STEP       0.5

#define N_SEEDS_DEFAULT    800
#define N_SEEDS_MIN        100
#define N_SEEDS_MAX       4096
#define N_SEEDS_STEP       100

/* Turn angles the 'g' key cycles, to compare against the golden one. The tidy
 * fractions of a full turn make spokes instead of an even spiral fill. */
static const double ANGLE_TABLE[] = {
    2.0 * M_PI / (PHI * PHI),    /* golden — fills evenly                      */
    2.0 * M_PI / 5.0,            /* 72° — seeds line up into 5 spokes          */
    2.0 * M_PI / 8.0,            /* 45° — 8 spokes                             */
    2.0 * M_PI * (1.0 - 1.0/PHI),/* golden, measured the other way             */
    2.0 * M_PI * 0.382,          /* near golden — gaps appear                  */
};
#define N_ANGLES  5

#define SEED_CHAR  'o'

#define FPS_EWMA_ALPHA     0.05   /* small = steadier on-screen fps number */

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

/* ── §4 phyllotaxis mapping & lattice ── */

/* GridCtx — the flower head for one frame: screen size, where the centre sits,
 * and the three knobs that shape the spiral (spacing, angle, n_seeds). Seeds
 * are computed one number at a time from these, so this is the single source of
 * truth for the picture and the cursor limits. Always centred on (ox, oy). */
typedef struct {
    int    rows, cols;     /* terminal size in character cells */
    double spacing;        /* pixels between seeds; +/- changes it (bigger head) */
    double angle;          /* turn between seeds, radians; g cycles it */
    int    n_seeds;        /* how many seeds to draw; [/] changes it */
    int    cell_w, cell_h; /* pixels per cell (CELL_W / CELL_H) */
    int    ox, oy;         /* flower centre, as a cell column and row */
    int    max_n;          /* highest valid seed number; n_seeds - 1 */
} GridCtx;

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
    /* no outer ring: seeds that land off-screen are skipped while drawing */
}

/* recipe step 1 — seed n -> the screen cell it lands on. Seed n sits √n
 * spacings out, turned n times by the angle; dividing by cell_w/cell_h undoes
 * the tall-cell aspect so the head reads round. */
static void seed_position(const GridCtx *g, int n, int *sr, int *sc)
{
    double r_n     = sqrt((double)n) * g->spacing;
    double theta_n = (double)n * g->angle;
    *sc = g->ox + (int)round(r_n * cos(theta_n) / (double)g->cell_w);
    *sr = g->oy + (int)round(r_n * sin(theta_n) / (double)g->cell_h);
}

/* recipe step 2 — draw the flower: walk seed 0..n_seeds, place SEED_CHAR at
 * each. Two seeds can round to one cell; the first wins (visited[] skips the
 * rest) so a dot is never overdrawn. The √n radius is what keeps the packing
 * even all the way out instead of crowding at the edge. */
static void draw_lattice(const GridCtx *g)
{
    int rows = g->rows, cols = g->cols;
    bool visited[rows][cols];
    memset(visited, 0, sizeof(bool) * (size_t)(rows * cols));

    attron(COLOR_PAIR(PAIR_GRID));
    for (int n = 0; n < g->n_seeds; n++) {
        int sr, sc;
        seed_position(g, n, &sr, &sc);
        if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) continue;
        if (visited[sr][sc]) continue;
        visited[sr][sc] = true;
        mvaddch(sr, sc, (chtype)(unsigned char)SEED_CHAR);
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ── §5 cursor ── */

/* Cursor — the seed number we point at. That one number is the whole address;
 * ask seed_position where it is and you get its screen cell. Range 0..max_n. */
typedef struct { int n; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->n = g->max_n / 4;     /* partway out — not the centre, not the edge */
}

/* recipe step 3 — move the marker, clamped to a real seed. Up/down jump by a
 * whole arm: adjacent seed numbers land on opposite sides of the head, so a
 * single step looks like a teleport; a bigger jump traces one spiral arm. */
#define CURSOR_RING_STEP 8
static void cursor_move(Cursor *cur, const GridCtx *g, int d_n)
{
    int nn = cur->n + d_n;
    if (nn < 0)         nn = 0;
    if (nn > g->max_n)  nn = g->max_n;
    cur->n = nn;
}

/* draw the '@' on the cursor's seed; after the lattice so it sits on top */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    seed_position(g, cur->n, &sr, &sc);
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
    draw_lattice(g);
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
