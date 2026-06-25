/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_rings_spokes.c — a polar grid drawn on the terminal: a bullseye of
 * concentric rings crossed by spokes radiating from the centre.  Move an '@'
 * cursor between cells with the arrows; tweak ring spacing and spoke count live.
 *
 * Sister files: 02_log_polar.c (rings that grow with distance),
 *               ../rect_grids/01_uniform_rect.c (the same GridCtx template).
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

/* How big one terminal cell is in our pretend "pixels".  A row is about twice
 * as tall as a column is wide, so we make the vertical pixel twice the size —
 * that way circles come out round instead of squashed into ovals. */
#define CELL_W        2
#define CELL_H        4

/* Rings: how far apart they sit and how thick each ring line draws. */
#define RING_SPACING_DEFAULT  20.0f  /* pixels between one ring and the next */
#define RING_SPACING_MIN       8.0f
#define RING_SPACING_MAX      48.0f
#define RING_SPACING_STEP      4.0f
#define RING_W                 1.6f  /* a cell this close to a ring counts as on it */

/* Spokes: how many radiate out, and how wide each one draws. */
#define N_SPOKES_DEFAULT  12
#define N_SPOKES_MIN       4
#define N_SPOKES_MAX      36
#define SPOKE_W            0.10      /* angle (radians) within which a cell is on a spoke */
#define SPOKE_MIN_R        3.0f      /* skip spokes near the centre so they don't smear into a blob */

/* The shown FPS is smoothed a little so the number doesn't jitter every frame. */
#define FPS_EWMA_ALPHA     0.05

#define PAIR_GRID    1
#define PAIR_CURSOR  2   /* the '@' cursor */
#define PAIR_HUD     3   /* status bar */
#define PAIR_HINT    4   /* key-hint footer */

/* Each theme is one colour: the nice 256-colour choice, plus a basic-8 fallback
 * for terminals that can't do 256 colours. */
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

/* ── §4 formula ── */

/*
 * GridCtx — everything that describes the polar grid as it's drawn right now:
 * how big the screen is, where its centre sits, how the rings and spokes are
 * spaced, and how far the cursor is allowed to roam.  The grid itself is never
 * stored as an array — it's worked out cell by cell from these numbers — so
 * this is the single source of truth for the picture and the cursor's limits.
 *
 * The grid is always centred on screen.  For a terminal cell (col, row), its
 * distance from the centre in "pixels" is found by subtracting the centre
 * (ox, oy) and scaling by the cell size.
 */
typedef struct {
    int rows, cols;        /* size of the terminal in character cells */

    float  ring_spacing;   /* gap between rings, in pixels; the user nudges this with +/- */
    int    n_spokes;       /* how many spokes radiate out; the user changes it with [/]   */
    int    cell_w, cell_h; /* size of one cell in pixels (copies of CELL_W / CELL_H)      */

    int    ox, oy;         /* the centre of the grid, as a cell column and row */

    /* how far the cursor may go: rings 0..max_ring, spokes 0..max_spoke.
     * Recomputed whenever the screen or grid changes so the cursor can't
     * wander off the visible area. */
    int    max_ring, max_spoke;
} GridCtx;

/* Recompute everything that depends on the screen size — call it at startup
 * and again after every resize or spacing change. */
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

    /* Biggest ring whose centre still lands on screen, using whichever way
     * (width or height) runs out of room first. */
    double rx = (double)cols * 0.5 * CELL_W;
    double ry = (double)rows * 0.5 * CELL_H;
    double r_visible = (rx < ry ? rx : ry);
    int mr = (int)(r_visible / g->ring_spacing) - 1;
    g->max_ring  = mr < 0 ? 0 : mr;
    g->max_spoke = g->n_spokes - 1;
}

/* Given a (ring, spoke) cell, find which screen column and row sit at the
 * middle of that wedge — half a ring out, half a wedge round — so the cursor
 * lands in the centre of its cell rather than on an edge. */
static void ctx_to_screen(const GridCtx *g, int ring, int spoke,
                          int *sr, int *sc)
{
    double mid_radius = ((double)ring + 0.5) * (double)g->ring_spacing;
    double theta_mid  = ((double)spoke + 0.5) * (2.0 * M_PI / (double)g->n_spokes);
    double cx = mid_radius * cos(theta_mid);
    double cy = mid_radius * sin(theta_mid);
    *sc = g->ox + (int)round(cx / (double)g->cell_w);
    *sr = g->oy + (int)round(cy / (double)g->cell_h);
}

/* Pick the line character that best matches a direction: '-' for roughly
 * horizontal, '|' for vertical, '/' and '\\' for the diagonals.  This is what
 * makes the spokes and rings look like real lines instead of dots.
 *
 * A line pointing one way looks the same as a line pointing the opposite way,
 * so we fold the angle into a half-turn first and only ever pick from four
 * orientations. */
static char angle_char(double theta)
{
    double a = fmod(theta + 2.0*M_PI, M_PI); /* collapse opposite directions onto one */
    if (a < M_PI/8.0 || a >= 7.0*M_PI/8.0) return '-';
    if (a < 3.0*M_PI/8.0)                   return '\\';
    if (a < 5.0*M_PI/8.0)                   return '|';
    return '/';
}

/* Walk every cell on screen and decide what, if anything, to draw there: a ring
 * line, a spoke line, a '+' where they cross, or nothing.  The trick is that
 * one cell can tell whether it's on *any* ring just from its distance, and on
 * *any* spoke just from its angle — no need to loop over each ring or spoke. */
static void ctx_draw_bg(const GridCtx *g)
{
    double spoke_angle = 2.0 * M_PI / (double)g->n_spokes;

    attron(COLOR_PAIR(PAIR_GRID));
    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double dx = (double)(col - g->ox) * g->cell_w;
            double dy = (double)(row - g->oy) * g->cell_h;
            float  r_px = (float)sqrt(dx*dx + dy*dy);
            double theta = atan2(dy, dx);

            /* on a ring if the distance is close to a whole number of ring gaps */
            float ring_phase = fmodf(r_px, g->ring_spacing);
            bool on_ring = (ring_phase < RING_W ||
                            ring_phase > g->ring_spacing - RING_W);

            /* on a spoke if the angle is close to a whole number of spoke steps */
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
 * Cursor — where on the grid the user is pointing, given as a ring number
 * (which band, counting out from the centre) and a spoke number (which wedge,
 * counting round).  That's the whole address; the limits and the screen
 * position come from GridCtx, so the cursor stays a tiny pair of indices.
 *   ring  : 0 at the innermost band, up to GridCtx.max_ring.
 *   spoke : 0..GridCtx.max_spoke, and it wraps around since angles are a circle.
 */
typedef struct { int ring, spoke; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->ring  = g->max_ring / 2;
    cur->spoke = 0;
}

/* Nudge the cursor by some rings and spokes.  Moving in/out stops at the edge,
 * but going round simply wraps — a circle has no first or last spoke. */
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

/* Draw the '@' on the cursor's cell.  Call this after the grid so the marker
 * sits on top instead of being painted over. */
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

/* The on-screen readouts: a status line top-right and the key hints along the
 * bottom, kept bright and bold so they stay readable over the grid. */
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
