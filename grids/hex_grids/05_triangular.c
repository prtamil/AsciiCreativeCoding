/*
 * 05_triangular.c — triangular grid with stripe-index cursor movement
 *
 * DEMO: Equilateral triangular tiling. A '@' cursor moves between adjacent
 *       triangles using arrow keys. The cursor triangle is highlighted.
 *       The cursor position is stored as two stripe indices (si, sj) —
 *       a different, non-cube coordinate system specific to triangular grids.
 *
 * Study alongside: grids/hex_grids/01_flat_top.c (hex = dual of this tiling)
 *
 * Section map:
 *   §1 config   — tunable constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — ncurses color pairs
 *   §4 coords   — pixel↔cell, stripe coordinate system
 *   §5 draw     — three-family stripe classifier
 *   §5b cursor  — TRI_DIR movement, triangle center formula, cursor_draw
 *   §6 scene    — state struct + scene_draw
 *   §7 screen   — ncurses display layer
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit  p pause  t theme  r reset  arrows move  +/-:size  [/]:border
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra 05_triangular.c -o 05_triangular -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Three-family stripe classifier. For triangle side s,
 *                  h = s·√3/2:
 *                    n1 = py / h             (horizontal family)
 *                    n2 = (√3·px + py) / h   (−60° family)
 *                    n3 = (−√3·px + py) / h  (+60° family)
 *                  edge_frac(nk) < border_w → draw '-', '/', or '\'.
 *
 * Cursor storage : (si, sj) = (floor(n1), floor(n2)) — two stripe indices
 *                  uniquely identify the triangle. This works because each
 *                  triangle occupies exactly one integer cell in the (n1,n2)
 *                  stripe-pair space.
 *
 *                  Triangle center in pixel space (for centered grid):
 *                    py_c = (si + 0.5) × h
 *                    px_c = (sj − si) × s/2
 *                  Derived by solving n1_center = si+0.5 and n2_center = sj+0.5.
 *
 * Cursor movement : TRI_DIR[4] moves by ±1 in si or sj:
 *                    UP/DOWN change si → move vertically by one stripe row
 *                    LEFT/RIGHT change sj → move horizontally by half a side
 *                  Adjacent triangles share an edge, so each step moves to
 *                  exactly one neighboring triangle.
 *
 * References     :
 *   Triangular tiling        — https://mathworld.wolfram.com/TriangularGrid.html
 *   Hex-triangle duality     — https://www.redblobgames.com/grids/hexagons/#map-storage
 *   Archimedean tilings list — https://en.wikipedia.org/wiki/Archimedean_tiling
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The equilateral triangular tiling is the dual of the hexagonal tiling:
 * place a point at every hex center and connect adjacent centers → you get
 * a triangular grid. The triangular grid has THREE families of parallel
 * lines (not two, like a rectangular grid). These families run at 0°, 60°,
 * and 120° to the horizontal. A pixel is "on a border" if it falls close to
 * any of the three families.
 *
 * The three-family classifier is fundamentally different from the cube-
 * coordinate hex classifier. There is no "cube rounding" — instead you
 * measure fractional distance to the nearest line in each family, and use
 * the MINIMUM as the border-distance metric.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Each line family divides the plane into parallel stripes. Family 1 is
 * horizontal: stripes are rows of height h = s·√3/2. Family 2 tilts at
 * 60°. Family 3 tilts at 120°. Together, these three families slice the
 * plane into small equilateral triangles.
 *
 * For each pixel, compute its fractional position WITHIN each stripe
 * (where 0 = on a line, 0.5 = maximum distance from any line). The
 * minimum over all three families tells you how close the pixel is to the
 * nearest triangular edge. If that minimum is below border_w → border.
 *
 * The cursor uses a STRIPE INDEX coordinate (si, sj), not cube coordinates.
 * si = floor(n1) identifies which row, sj = floor(n2) identifies which
 * diagonal stripe. Together (si, sj) uniquely labels a triangle.
 *
 * DRAWING METHOD  (three-family stripe rasterizer)
 * ─────────────────────────────────────────────────
 *  1. Center the grid: px = (col−ox)×CELL_W, py = (row−oy)×CELL_H.
 *
 *  2. Compute three stripe coordinates:
 *       n1 = py / h                    (family 1: horizontal lines at y = k·h)
 *       n2 = (√3·px + py) / h         (family 2: lines tilted −60°)
 *       n3 = (−√3·px + py) / h        (family 3: lines tilted +60°)
 *
 *  3. Compute edge fraction for each:
 *       d1 = edge_frac(n1)   d2 = edge_frac(n2)   d3 = edge_frac(n3)
 *       edge_frac(v) = min(frac(v), 1−frac(v))  ∈ [0, 0.5]
 *       0 = on a line, 0.5 = center of stripe.
 *
 *  4. dmin = min(d1, d2, d3). The character matches whichever family wins:
 *       d1 wins → '-'   (horizontal line)
 *       d2 wins → '/'   (−60° line)
 *       d3 wins → '\'   (+60° line)
 *
 *  5. Stripe indices for cursor detection:
 *       cell_si = floor(n1),  cell_sj = floor(n2)
 *       on_cur = (cell_si == si && cell_sj == sj)
 *
 *  6. Draw:
 *       dmin < border_w  → border char (cursor or border color)
 *       dmin ≥ border_w && on_cur → fill interior with PAIR_CURSOR background
 *
 * KEY FORMULAS
 * ────────────
 *  Stripe period: h = tri_size × √3 / 2
 *  (The perpendicular distance between parallel lines in each family)
 *
 *  Three stripe coordinates:
 *    n1 = py / h
 *    n2 = (√3·px + py) / h
 *    n3 = (−√3·px + py) / h
 *
 *  edge_frac(v) = frac(v) if frac(v) < 0.5 else 1 − frac(v)
 *    where frac(v) = v − floor(v) ∈ [0,1)
 *
 *  Triangle center from stripe indices:
 *    py_c = (si + 0.5) × h
 *    px_c = (sj − si) × s/2
 *    Derivation: set n1 = si+0.5 → py_c = (si+0.5)·h
 *                set n2 = sj+0.5 → √3·px_c + py_c = (sj+0.5)·h
 *                → px_c = [(sj+0.5)·h − (si+0.5)·h] / √3
 *                       = (sj−si)·h/√3 = (sj−si)·s/2  (since h = s·√3/2)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • edge_frac must handle NEGATIVE n values. fmod can return negative
 *    results in C for negative inputs. Always add 1.0 if fmod result < 0:
 *    t = fmod(v, 1.0); if (t < 0) t += 1.0;
 *
 *  • "Which character wins" is determined by argmin(d1,d2,d3), not by a
 *    fixed threshold. If border_w is large, many pixels show border chars;
 *    if small, only the line-nearest pixels do.
 *
 *  • Cursor detection uses floor(n1) and floor(n2) only — NOT n3. Two
 *    families suffice to uniquely identify a triangle (the third is
 *    redundant). Using all three would over-constrain and miss some triangles.
 *
 *  • The triangular grid has NO concept of cube coordinates. Do not try to
 *    apply HEX_DIR here — the movement table TRI_DIR works in (si, sj) space.
 *
 * HOW TO VERIFY  (80×24 terminal, TRI_SIZE=20, cursor at si=0, sj=0)
 * ─────────────
 *  h = 20 × √3/2 ≈ 17.32 pixels.
 *
 *  Cursor center: py_c = 0.5 × 17.32 ≈ 8.66 px → row = oy + 2 = 13.
 *                 px_c = (0−0) × 10 = 0 → col = ox = 40.
 *  '@' at (13, 40). ✓
 *
 *  Pixel at center (col=40, row=13): px=0, py=8.66.
 *    n1 = 8.66/17.32 = 0.5 → edge_frac = 0.5 (far from line). Interior ✓
 *    n2 = (0 + 8.66)/17.32 = 0.5 → edge_frac = 0.5. Interior ✓
 *    Both far from borders → dmin = 0.5 > border_w → interior fill drawn. ✓
 *
 *  Pixel at the n1=0 line (py=0, any px):
 *    n1 = 0 → edge_frac(0) = 0 → d1=0 → dmin=0 < border_w → border '-'. ✓
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ncurses.h>

/* ── §1 config ────────────────────────────────────────────────────────── */

#define CELL_W              2
#define CELL_H              4

#define TRI_SIZE_DEFAULT   20.0
#define TRI_SIZE_MIN        8.0
#define TRI_SIZE_MAX       60.0
#define TRI_SIZE_STEP       4.0

#define BORDER_W_DEFAULT    0.10
#define BORDER_W_MIN        0.02
#define BORDER_W_MAX        0.45
#define BORDER_W_STEP       0.02

#define N_THEMES            4
#define TICK_NS            16666667LL

/* ── §2 clock ─────────────────────────────────────────────────────────── */

static int64_t clock_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void clock_sleep_ns(int64_t ns) {
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3 color ─────────────────────────────────────────────────────────── */

#define PAIR_BORDER   1
#define PAIR_CURSOR   2
#define PAIR_HUD      3
#define PAIR_HINT     4

static const short THEMES[N_THEMES][2] = {
    { COLOR_CYAN,   COLOR_BLACK },
    { COLOR_GREEN,  COLOR_BLACK },
    { COLOR_YELLOW, COLOR_BLACK },
    { COLOR_WHITE,  COLOR_BLACK },
};
static void color_init(int theme) {
    start_color();
    use_default_colors();
    init_pair(PAIR_BORDER, THEMES[theme][0], THEMES[theme][1]);
    init_pair(PAIR_CURSOR, COLOR_WHITE,      COLOR_BLUE);
    init_pair(PAIR_HUD,    COLOR_BLACK,      COLOR_CYAN);
    init_pair(PAIR_HINT,   COLOR_CYAN,       COLOR_BLACK);
}

/* ── §4 coords ────────────────────────────────────────────────────────── */
/*
 * Centered: px = (col - ox)×CELL_W, py = (row - oy)×CELL_H.
 *
 * Stripe coordinate system (non-cube, triangular-specific):
 *   n1 = py / h             → floor(n1) = si  (horizontal stripe row)
 *   n2 = (√3·px + py) / h  → floor(n2) = sj  (diagonal stripe column)
 *
 * Triangle (si, sj) center in pixel space:
 *   py_c = (si + 0.5) × h                — midpoint of row si
 *   px_c = (sj − si) × s/2               — solved from n2_center = sj + 0.5
 *
 * Derivation: n2_center = (√3·px_c + py_c)/h = sj+0.5
 *             → √3·px_c = (sj+0.5)·h − py_c = (sj+0.5)·h − (si+0.5)·h = (sj−si)·h
 *             → px_c = (sj−si)·h/√3 = (sj−si)·s/2  (since h/√3 = s/2)
 */

/* ── §5 draw ──────────────────────────────────────────────────────────── */

/*
 * edge_frac — distance [0, 0.5] to the nearest integer in val.
 *
 * THE FORMULA:
 *   t = fmod(v, 1.0)       → fractional part in [0,1)
 *   if t < 0: t += 1.0     → handle negative v (C fmod can return negative)
 *   return t < 0.5 ? t : 1.0-t
 *
 *   Result: 0 when v is an integer (on a grid line),
 *           0.5 when v = k+0.5 (farthest from any grid line).
 *
 * This measures the "closeness to a stripe boundary" in normalized units.
 * Comparing this to border_w gives the border/interior classification.
 */
static double edge_frac(double v) {
    double t = fmod(v, 1.0);
    if (t < 0.0) t += 1.0;
    return t < 0.5 ? t : 1.0 - t;
}

/*
 * grid_draw — three-family stripe rasterizer with cursor triangle highlight.
 *
 * THE PIPELINE (per screen cell):
 *
 *   (col, row)
 *      │
 *      ▼  Center the grid
 *   px = (col−ox)×CELL_W,  py = (row−oy)×CELL_H
 *      │
 *      ▼  Compute three stripe coordinates
 *   n1 = py / h
 *   n2 = (√3·px + py) / h
 *   n3 = (−√3·px + py) / h
 *      │
 *      ▼  Stripe indices for cursor detection
 *   cell_si = floor(n1),  cell_sj = floor(n2)
 *   on_cur  = (cell_si == si && cell_sj == sj)
 *      │
 *      ▼  Edge fractions — distance to nearest line in each family
 *   d1 = edge_frac(n1)   character '-'
 *   d2 = edge_frac(n2)   character '/'
 *   d3 = edge_frac(n3)   character '\'
 *   dmin = min(d1, d2, d3);  ch = character of the winning family
 *      │
 *      ├── dmin < border_w:  draw ch in cursor or border color
 *      │
 *      └── dmin ≥ border_w && on_cur: fill with PAIR_CURSOR background
 *
 * CHARACTER-TO-FAMILY MAPPING:
 *   Family 1 (n1, horizontal lines) → '-'
 *   Family 2 (n2, −60° lines)       → '/'
 *   Family 3 (n3, +60° lines)       → '\'
 * The character direction matches the actual line slope in screen space.
 */
static void grid_draw(int rows, int cols, double tri_size, double border_w,
                       int si, int sj, int ox, int oy) {
    double h   = tri_size * sqrt(3.0) * 0.5;
    double sq3 = sqrt(3.0);

    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;

            double n1 = py / h;
            double n2 = (sq3 * px + py) / h;
            double n3 = (-sq3 * px + py) / h;

            /* Stripe indices identify which triangle this cell is in */
            int cell_si = (int)floor(n1);
            int cell_sj = (int)floor(n2);
            int on_cur  = (cell_si == si && cell_sj == sj);

            double d1 = edge_frac(n1);
            double d2 = edge_frac(n2);
            double d3 = edge_frac(n3);

            double dmin = d1;
            char   ch   = '-';
            if (d2 < dmin) { dmin = d2; ch = '/';  }
            if (d3 < dmin) { dmin = d3; ch = '\\'; }

            if (dmin < border_w) {
                int attr = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                  : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)ch);
                attroff(attr);
            } else if (on_cur) {
                /* Fill interior of cursor triangle with dim highlight */
                attron(COLOR_PAIR(PAIR_CURSOR));
                mvaddch(row, col, ' ');
                attroff(COLOR_PAIR(PAIR_CURSOR));
            }
        }
    }
}

/* ── §5b cursor ───────────────────────────────────────────────────────── */

/*
 * TRI_DIR — 4-direction movement in (si, sj) stripe-index space.
 *
 *                UP: si-1  (one stripe row up — moves py_c by -h)
 *                   ↑
 *   LEFT: sj-1 ← ● → RIGHT: sj+1  (moves px_c by ±s/2)
 *                   ↓
 *               DOWN: si+1
 *
 * WHY THIS WORKS:
 *   si indexes horizontal stripes. si±1 moves to the adjacent stripe row.
 *   This crosses a family-1 line (horizontal edge), moving to the triangle
 *   that shares the horizontal edge.
 *
 *   sj indexes diagonal stripes. sj±1 moves along the n2-family diagonal.
 *   This crosses a family-2 line (60° edge), moving to the triangle that
 *   shares that diagonal edge.
 *
 *   Both directions produce strictly edge-adjacent triangles.
 *
 * WHY NOT USE HEX_DIR:
 *   HEX_DIR applies to axial (Q,R) cube space. Triangular stripe space
 *   (si, sj) is a completely different coordinate system — using HEX_DIR
 *   here would move to the wrong triangles.
 */
static const int TRI_DIR[4][2] = {
    {-1,  0 },   /* UP    — si decreases → row moves up */
    {+1,  0 },   /* DOWN  — si increases */
    { 0, -1 },   /* LEFT  — sj decreases → px_c decreases by s/2 */
    { 0, +1 },   /* RIGHT — sj increases */
};

/*
 * cursor_draw — place '@' at the center cell of triangle (si, sj).
 *
 * THE FORMULA (triangle center from stripe indices):
 *   h    = tri_size × √3 / 2
 *   py_c = (si + 0.5) × h
 *   px_c = (sj − si) × tri_size / 2
 *
 *   col = ox + (int)(px_c / CELL_W)
 *   row = oy + (int)(py_c / CELL_H)
 *
 * Derivation: the center of triangle (si,sj) is where n1=si+0.5 and
 * n2=sj+0.5. Solving both equations simultaneously gives the formulas above.
 * See §4 for the full derivation.
 */
static void cursor_draw(double tri_size, int si, int sj,
                         int ox, int oy, int rows, int cols) {
    double h     = tri_size * sqrt(3.0) * 0.5;
    double py_c  = (si + 0.5) * h;
    double px_c  = (double)(sj - si) * tri_size * 0.5;
    int col = ox + (int)(px_c / CELL_W);
    int row = oy + (int)(py_c / CELL_H);
    if (col >= 0 && col < cols && row >= 0 && row < rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(row, col, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ── §6 scene ─────────────────────────────────────────────────────────── */

typedef struct {
    double tri_size, border_w;
    int    si, sj;    /* cursor triangle — stripe indices (si=row, sj=diagonal) */
    int    theme, paused;
} Scene;

static void scene_init(Scene *s) {
    s->tri_size = TRI_SIZE_DEFAULT;
    s->border_w = BORDER_W_DEFAULT;
    s->si = 0; s->sj = 0;
    s->theme = 0; s->paused = 0;
}
static void scene_draw(const Scene *s, int rows, int cols) {
    int ox = cols / 2;
    int oy = (rows - 1) / 2;
    grid_draw(rows, cols, s->tri_size, s->border_w, s->si, s->sj, ox, oy);
    cursor_draw(s->tri_size, s->si, s->sj, ox, oy, rows, cols);
}

/* ── §7 screen ────────────────────────────────────────────────────────── */

static void screen_init(void) {
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); curs_set(0);
    nodelay(stdscr, TRUE); typeahead(-1);
}
static void screen_draw_hud(const Scene *s, int rows, int cols, double fps) {
    char buf[96];
    snprintf(buf, sizeof buf,
             " stripe si:%+d sj:%+d  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             s->si, s->sj, s->tri_size, s->border_w, s->theme, fps,
             s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:size  [/]:border ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);
}
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }
static void cleanup(void) { endwin(); }

/* ── §8 app ───────────────────────────────────────────────────────────── */

static volatile sig_atomic_t running = 1, need_resize = 0;
static void sig_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) running = 0;
    if (sig == SIGWINCH) need_resize = 1;
}

int main(void) {
    atexit(cleanup);
    signal(SIGINT, sig_handler); signal(SIGTERM, sig_handler);
    signal(SIGWINCH, sig_handler);
    screen_init();
    int rows, cols; getmaxyx(stdscr, rows, cols);
    Scene sc; scene_init(&sc); color_init(sc.theme);
    int64_t prev = clock_ns(); double fps = 60.0;

    while (running) {
        if (need_resize) {
            need_resize = 0; endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: running = 0; break;
            case 'p': sc.paused ^= 1; break;
            case 'r': sc.si = 0; sc.sj = 0; break;
            case 't': sc.theme = (sc.theme + 1) % N_THEMES; color_init(sc.theme); break;
            /* Arrow keys apply TRI_DIR deltas to stripe cursor */
            case KEY_UP:    sc.si += TRI_DIR[0][0]; sc.sj += TRI_DIR[0][1]; break;
            case KEY_DOWN:  sc.si += TRI_DIR[1][0]; sc.sj += TRI_DIR[1][1]; break;
            case KEY_LEFT:  sc.si += TRI_DIR[2][0]; sc.sj += TRI_DIR[2][1]; break;
            case KEY_RIGHT: sc.si += TRI_DIR[3][0]; sc.sj += TRI_DIR[3][1]; break;
            case '+': case '=':
                if (sc.tri_size < TRI_SIZE_MAX) { sc.tri_size += TRI_SIZE_STEP; } break;
            case '-':
                if (sc.tri_size > TRI_SIZE_MIN) { sc.tri_size -= TRI_SIZE_STEP; } break;
            case '[':
                if (sc.border_w > BORDER_W_MIN) { sc.border_w -= BORDER_W_STEP; } break;
            case ']':
                if (sc.border_w < BORDER_W_MAX) { sc.border_w += BORDER_W_STEP; } break;
            }
        }
        int64_t now = clock_ns(), dt = now - prev; prev = now;
        if (dt > 0) fps = fps * 0.9 + 1e9 / (double)dt * 0.1;
        erase();
        scene_draw(&sc, rows, cols);
        screen_draw_hud(&sc, rows, cols, fps);
        screen_present();
        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }
    return 0;
}
