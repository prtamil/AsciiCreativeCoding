/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_double_diagonal.c — tetrakis square tiling (4 triangles per cell)
 *
 * DEMO: Each square is split by BOTH diagonals into 4 right-isosceles
 *       triangles meeting at the centre. Triangles are labelled by the
 *       direction their apex points: N, E, S, W. Arrow keys move the
 *       cursor toward that compass direction — within the current square
 *       if possible, jumping to the next square otherwise.
 *
 * Study alongside: 02_right_isosceles.c — 2 triangles per square.
 *                  04_30_60_90.c — kisrhombille of equilaterals (analogous
 *                  6-triangle decomposition for the triangular grid).
 *
 * Section map:
 *   §1 config   — CELL_W, CELL_H, TRI_SIZE, BORDER_W
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 4 pairs: border / cursor / HUD / hint
 *   §4 formula  — wedge classifier + barycentric → edge character
 *   §5 cursor   — TETRA_DIR table + cursor_step + cursor_draw
 *   §6 scene    — grid_draw + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   t theme   p pause
 *        +/- size        [/] border thickness   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/tri_grids/03_double_diagonal.c \
 *       -o 03_double_diagonal -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Tetrakis square tiling — every square split into 4
 *                  right-isosceles triangles by both diagonals (vertex
 *                  config 8.8.8.8: eight triangles meet at every original
 *                  square corner). Pixel→lattice axis-aligned, then a
 *                  WEDGE classifier picks one of {N, E, S, W} based on
 *                  which axis-distance from the centre dominates.
 *
 * Formula        : Wedge classifier:
 *                    dx = fa − ½, dy = fb − ½
 *                    |dx| > |dy|, dx > 0  →  E
 *                    |dx| > |dy|, dx < 0  →  W
 *                    |dy| ≥ |dx|, dy > 0  →  S
 *                    |dy| ≥ |dx|, dy < 0  →  N
 *
 * Edge chars     : Each triangle has one straight edge + two half-diagonal
 *                  edges sharing the square centre. Barycentric weights
 *                  pick which is closest:
 *                    N → '/' '\\' '_'   E → '\\' '/' '|'
 *                    S → '\\' '/' '_'   W → '\\' '/' '|'
 *
 * Movement       : (col, row, dir) walked by lookup table TETRA_DIR[4][4].
 *                  Each arrow key moves the cursor toward that compass
 *                  direction within the current square; if already at the
 *                  matching apex direction, jumps to the next square.
 *
 * References     :
 *   Tetrakis square tiling — https://en.wikipedia.org/wiki/Tetrakis_square_tiling
 *   Conway, Burgiel, Goodman-Strauss, "The Symmetries of Things" (2008) §22
 *   Barycentric coords    — https://en.wikipedia.org/wiki/Barycentric_coordinate_system
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Take 02's half-rect grid and add the SECOND diagonal. The square is now
 * split into 4 wedges meeting at the centre, like cutting a slice of pie
 * along both diagonals. Each wedge is a right-isosceles triangle whose
 * apex points to one cardinal direction (North, East, South, West). The
 * "which wedge owns this pixel?" question reduces to a single comparison
 * of |Δx| versus |Δy| from the cell centre.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Stand at the centre of a square. The two diagonals (running through
 * you) divide the square into 4 wedges:
 *   N — top wedge, apex points up at the centre
 *   E — right wedge, apex points right
 *   S — bottom wedge, apex points down
 *   W — left wedge, apex points left
 * For any point inside the square, the wedge it lies in is determined by
 * the GREATER of "horizontal distance from centre" and "vertical distance
 * from centre". |dx| wins → it's E or W (sign of dx tells which); |dy|
 * wins → it's N or S.
 *
 * DRAWING METHOD  (raster scan, the approach used here)
 * ──────────────
 *  1. Pick TRI_SIZE — square side length in pixels.
 *  2. Loop every screen cell; convert to centred pixel.
 *  3. Lattice inverse: a = px/size, b = py/size.
 *  4. Floor + frac: tC=⌊a⌋, tR=⌊b⌋, fa=a−tC, fb=b−tR.
 *  5. Wedge classifier:  dx = fa−½,  dy = fb−½.  Pick N/E/S/W from
 *     |dx| vs |dy| and the signs.
 *  6. Compute the wedge's barycentric weights (closed form per direction).
 *  7. m = min(l₁, l₂, l₃). If m ≥ BORDER_W → interior, skip.
 *     Otherwise pick the matching edge character.
 *  8. Draw in cursor or border color.
 *
 * KEY FORMULAS
 * ────────────
 *  Wedge classifier:
 *    adx = |fa − ½|,  ady = |fb − ½|
 *    if adx > ady:  E if fa > ½ else W
 *    else:          S if fb > ½ else N
 *
 *  Barycentric weights (apex at C = (½, ½) in every wedge):
 *    N (A=(0,0), B=(1,0), C=(½,½)):
 *      l_A = 1−fa−fb,  l_B = fa−fb,  l_C = 2·fb
 *    E (A=(1,0), B=(1,1), C=(½,½)):
 *      l_A = fa−fb,    l_B = fa+fb−1, l_C = 2·(1−fa)
 *    S (A=(0,1), B=(1,1), C=(½,½)):
 *      l_A = fb−fa,    l_B = fa+fb−1, l_C = 2·(1−fb)
 *    W (A=(0,0), B=(0,1), C=(½,½)):
 *      l_A = 1−fa−fb,  l_B = fb−fa,  l_C = 2·fa
 *
 *  Centroids in lattice units (relative to square's upper-left corner):
 *    N: (½, 1/6)    E: (5/6, ½)    S: (½, 5/6)    W: (1/6, ½)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • At the diagonals (|dx| = |dy|), the wedge classifier ties; we resolve
 *    arbitrarily by giving |dy| precedence in the second branch. Either
 *    choice works because diagonal pixels are always border anyway.
 *  • At the centre (fa = fb = ½), all four wedges meet. The classifier
 *    picks N by tie-breaking; the centre pixel renders as a border
 *    character regardless.
 *  • TRI_SIZE = 8 minimum: smaller and the centre wedges collapse to
 *    fewer than 1 pixel, making the diagonals invisible.
 *
 * HOW TO VERIFY
 * ─────────────
 *  At cursor (cC, cR, cD) = (0, 0, N):
 *    centroid lattice = (½, 1/6) → pixel (size/2, size/6).
 *    For TRI_SIZE = 18: centroid ≈ (9, 3) px →
 *      col ≈ 9/CELL_W = 4, row ≈ 3/CELL_H = 0.
 *
 *  Wedge sanity at (fa, fb) = (0.7, 0.3):
 *    dx = 0.2, dy = -0.2. |dx| = |dy| = 0.2; tie → second branch picks N.
 *    Hmm: with my classifier "adx > ady" (strict), tie goes to else →
 *    dy < 0 → N. That's the small-angle border between E and N — right
 *    on the upper-right diagonal. Correct edge: the diagonal '\\'.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS 60

#define CELL_W 2
#define CELL_H 4

#define TRI_SIZE_DEFAULT 18.0
#define TRI_SIZE_MIN      8.0
#define TRI_SIZE_MAX     48.0
#define TRI_SIZE_STEP     2.0

#define BORDER_W_DEFAULT 0.10
#define BORDER_W_MIN     0.03
#define BORDER_W_MAX     0.35
#define BORDER_W_STEP    0.02

/* Apex direction indices */
#define DIR_N 0
#define DIR_E 1
#define DIR_S 2
#define DIR_W 3

#define N_THEMES 4

#define PAIR_BORDER 1
#define PAIR_CURSOR 2
#define PAIR_HUD    3
#define PAIR_HINT   4

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static const short THEME_FG[N_THEMES]   = {  82, 207, 214,  15 };
static const short THEME_FG_8[N_THEMES] = {
    COLOR_GREEN, COLOR_MAGENTA, COLOR_YELLOW, COLOR_WHITE,
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    short fg = (COLORS >= 256) ? THEME_FG[theme] : THEME_FG_8[theme];
    init_pair(PAIR_BORDER, fg, -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 15 : COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  0 : COLOR_BLACK, COLOR_CYAN);
    init_pair(PAIR_HINT,   COLORS >= 256 ? 75 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — wedge classifier + barycentric → edge char                */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * pixel_to_tri — square cell + wedge classifier.
 *
 * THE FORMULA (axis-aligned lattice + 4-wedge split):
 *
 *   a = px / size,   b = py / size
 *   col = ⌊a⌋, row = ⌊b⌋, fa = a−col, fb = b−row
 *   dx = fa − ½, dy = fb − ½
 *   |dx| > |dy|, dx > 0  →  E
 *   |dx| > |dy|, dx < 0  →  W
 *   |dy| ≥ |dx|, dy > 0  →  S
 *   |dy| ≥ |dx|, dy < 0  →  N
 */
static void pixel_to_tri(double px, double py, double size,
                         int *col, int *row, int *dir,
                         double *fa, double *fb)
{
    double inv = 1.0 / size;
    double a   = px * inv;
    double b   = py * inv;
    int    c   = (int)floor(a);
    int    r   = (int)floor(b);
    *col = c; *row = r;
    *fa = a - (double)c;
    *fb = b - (double)r;

    double dx = *fa - 0.5, dy = *fb - 0.5;
    double adx = fabs(dx), ady = fabs(dy);
    if (adx > ady) *dir = (dx > 0.0) ? DIR_E : DIR_W;
    else           *dir = (dy > 0.0) ? DIR_S : DIR_N;
}

/*
 * tri_centroid_pixel — forward map for cursor mark.
 *
 *   N: (½, 1/6)    E: (5/6, ½)    S: (½, 5/6)    W: (1/6, ½)
 */
static void tri_centroid_pixel(int col, int row, int dir, double size,
                               double *cx_pix, double *cy_pix)
{
    double a, b;
    switch (dir) {
        case DIR_N: a = 0.5;       b = 1.0/6.0;  break;
        case DIR_E: a = 5.0/6.0;   b = 0.5;      break;
        case DIR_S: a = 0.5;       b = 5.0/6.0;  break;
        default:    a = 1.0/6.0;   b = 0.5;      break;  /* W */
    }
    *cx_pix = ((double)col + a) * size;
    *cy_pix = ((double)row + b) * size;
}

/*
 * tri_edge_char — barycentric weights → edge character per wedge.
 *
 * Weights derivation (each wedge has C=(½,½) as apex):
 *   N (A=(0,0), B=(1,0), C=(½,½)):
 *     l_A = 1−fa−fb,    l_B = fa−fb,        l_C = 2·fb
 *     l_A → '/'   l_B → '\\'  l_C → '_'
 *   E (A=(1,0), B=(1,1), C=(½,½)):
 *     l_A = fa−fb,      l_B = fa+fb−1,      l_C = 2·(1−fa)
 *     l_A → '\\'  l_B → '/'   l_C → '|'
 *   S (A=(0,1), B=(1,1), C=(½,½)):
 *     l_A = fb−fa,      l_B = fa+fb−1,      l_C = 2·(1−fb)
 *     l_A → '\\'  l_B → '/'   l_C → '_'
 *   W (A=(0,0), B=(0,1), C=(½,½)):
 *     l_A = 1−fa−fb,    l_B = fb−fa,        l_C = 2·fa
 *     l_A → '\\'  l_B → '/'   l_C → '|'
 */
static char tri_edge_char(int dir, double fa, double fb, double *out_min)
{
    double l1, l2, l3;
    char   ch1, ch2, ch3;
    switch (dir) {
        case DIR_N:
            l1 = 1.0 - fa - fb; ch1 = '/';
            l2 = fa - fb;       ch2 = '\\';
            l3 = 2.0 * fb;      ch3 = '_';
            break;
        case DIR_E:
            l1 = fa - fb;       ch1 = '\\';
            l2 = fa + fb - 1.0; ch2 = '/';
            l3 = 2.0 * (1.0 - fa); ch3 = '|';
            break;
        case DIR_S:
            l1 = fb - fa;       ch1 = '\\';
            l2 = fa + fb - 1.0; ch2 = '/';
            l3 = 2.0 * (1.0 - fb); ch3 = '_';
            break;
        default: /* DIR_W */
            l1 = 1.0 - fa - fb; ch1 = '\\';
            l2 = fb - fa;       ch2 = '/';
            l3 = 2.0 * fa;      ch3 = '|';
            break;
    }
    char   ch = ch1;
    double m  = l1;
    if (l2 < m) { m = l2; ch = ch2; }
    if (l3 < m) { m = l3; ch = ch3; }
    *out_min = m;
    return ch;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    col, row, dir;       /* dir ∈ {N, E, S, W} */
    double tri_size;
    double border_w;
    int    theme;
    int    paused;
} Cursor;

/*
 * TETRA_DIR — arrow-key transitions (Δcol, Δrow, target_dir).
 *   index 0:LEFT  1:RIGHT  2:UP  3:DOWN
 *   row   0:N     1:E      2:S   3:W
 *
 * Arrow press moves the cursor "toward" the compass direction. If the
 * current triangle's apex already points that way and its base edge is
 * the boundary, jump to the matching triangle in the adjacent square
 * (apex flipped). Otherwise toggle to the matching triangle in the same
 * square.
 *   W + LEFT  → E in (col-1, row)        N + LEFT → W in same square
 *   N + UP    → S in (col, row-1)        E + UP   → N in same square
 */
static const int TETRA_DIR[4][4][3] = {
    /* LEFT  */ { {  0,  0, DIR_W }, {  0,  0, DIR_W }, {  0,  0, DIR_W }, { -1,  0, DIR_E } },
    /* RIGHT */ { {  0,  0, DIR_E }, { +1,  0, DIR_W }, {  0,  0, DIR_E }, {  0,  0, DIR_E } },
    /* UP    */ { {  0, -1, DIR_S }, {  0,  0, DIR_N }, {  0,  0, DIR_N }, {  0,  0, DIR_N } },
    /* DOWN  */ { {  0,  0, DIR_S }, {  0,  0, DIR_S }, {  0, +1, DIR_N }, {  0,  0, DIR_S } },
};

static void cursor_reset(Cursor *cur)
{
    cur->col = 0; cur->row = 0; cur->dir = DIR_N;
    cur->tri_size = TRI_SIZE_DEFAULT;
    cur->border_w = BORDER_W_DEFAULT;
    cur->theme    = 0;
    cur->paused   = 0;
}

static void cursor_step(Cursor *cur, int arrow)
{
    const int *t = TETRA_DIR[arrow][cur->dir];
    cur->col += t[0];
    cur->row += t[1];
    cur->dir  = t[2];
}

static void cursor_draw(const Cursor *cur, int rows, int cols, int ox, int oy)
{
    double cx_pix, cy_pix;
    tri_centroid_pixel(cur->col, cur->row, cur->dir, cur->tri_size,
                       &cx_pix, &cy_pix);
    int col = ox + (int)(cx_pix / CELL_W);
    int row = oy + (int)(cy_pix / CELL_H);
    if (col >= 0 && col < cols && row >= 0 && row < rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(row, col, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void grid_draw(int rows, int cols, const Cursor *cur, int ox, int oy)
{
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;

            int    tC, tR, tD;
            double fa, fb, m;
            pixel_to_tri(px, py, cur->tri_size, &tC, &tR, &tD, &fa, &fb);
            char ch = tri_edge_char(tD, fa, fb, &m);
            if (m >= cur->border_w) continue;

            int on_cur = (tC == cur->col && tR == cur->row && tD == cur->dir);
            int attr   = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

static const char *DIR_NAME[4] = { "N", "E", "S", "W" };

static void scene_draw(int rows, int cols, const Cursor *cur, double fps)
{
    erase();
    int ox = cols / 2;
    int oy = (rows - 1) / 2;
    grid_draw(rows, cols, cur, ox, oy);
    cursor_draw(cur, rows, cols, ox, oy);

    char buf[112];
    snprintf(buf, sizeof buf,
             " C:%+d R:%+d %s  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->col, cur->row, DIR_NAME[cur->dir],
             cur->tri_size, cur->border_w, cur->theme, fps,
             cur->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " arrows:move  +/-:size  [/]:border  t:theme  r:reset  p:pause  q:quit  [03 double diagonal] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

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

    Cursor cur;
    cursor_reset(&cur);
    screen_init(cur.theme);

    int rows = LINES, cols = COLS;
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps = TARGET_FPS;
    int64_t t0  = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27:  g_running = 0; break;
                case 'p':           cur.paused ^= 1; break;
                case 'r':           cursor_reset(&cur); color_init(cur.theme); break;
                case 't':
                    cur.theme = (cur.theme + 1) % N_THEMES;
                    color_init(cur.theme);
                    break;
                case KEY_LEFT:  cursor_step(&cur, 0); break;
                case KEY_RIGHT: cursor_step(&cur, 1); break;
                case KEY_UP:    cursor_step(&cur, 2); break;
                case KEY_DOWN:  cursor_step(&cur, 3); break;
                case '+': case '=':
                    if (cur.tri_size < TRI_SIZE_MAX) { cur.tri_size += TRI_SIZE_STEP; } break;
                case '-':
                    if (cur.tri_size > TRI_SIZE_MIN) { cur.tri_size -= TRI_SIZE_STEP; } break;
                case '[':
                    if (cur.border_w > BORDER_W_MIN) { cur.border_w -= BORDER_W_STEP; } break;
                case ']':
                    if (cur.border_w < BORDER_W_MAX) { cur.border_w += BORDER_W_STEP; } break;
            }
        }

        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0  = now;

        scene_draw(rows, cols, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
