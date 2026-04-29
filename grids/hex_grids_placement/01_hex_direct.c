/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_hex_direct.c — cursor-based direct object placement on a flat-top hex grid
 *
 * DEMO: A flat-top hexagonal grid fills the screen. Navigate '@' between
 *       hexagons with arrow keys. Press SPACE to toggle a '*' object at the
 *       cursor hex. Objects are stored by axial address (Q, R) and survive
 *       resizes — they follow their hex when the terminal changes size.
 *
 * Study alongside: grids/hex_grids/01_flat_top.c (background rasterizer),
 *                  grids/rect_grids_placement/01_direct.c (same idea on rect)
 *
 * Section map:
 *   §1  config  — all tunable constants
 *   §2  clock   — monotonic timer + sleep
 *   §3  color   — color pairs: grid, cursor, object, HUD, hint
 *   §4  coords  — cube_round, hex_to_screen, hex_dist, angle_char
 *   §5  pool    — HPool: object array, toggle, clear, draw
 *   §6  bgrid   — flat-top hex rasterizer (grid_draw)
 *   §7  cursor  — HEX_DIR movement table, cursor_draw
 *   §8  scene   — Scene struct, scene_draw
 *   §9  screen  — ncurses init, HUD, cleanup
 *  §10  app     — signals, main loop
 *
 * Keys:  arrows:move  spc:toggle  C:clear  r:reset  +/-:size  q/ESC:quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/hex_grids_placement/01_hex_direct.c \
 *       -o 01_hex_direct -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Flat-top hex cursor placement.  A cursor (cQ, cR) in axial
 *                  coordinates moves with arrow keys.  SPACE toggles an object
 *                  record (Q, R, glyph) in a flat HPool array.  Each frame:
 *                  grid_draw rasterizes the background per-pixel using the
 *                  inverse matrix; pool_draw projects each stored (Q, R) back
 *                  to screen via the forward matrix.
 *
 * Data-structure : HPool — flat array of HObj{Q, R, glyph}.  Removal swaps
 *                  the dead slot with the last item (O(1)).  Capacity MAX_OBJ
 *                  greatly exceeds the visible hex count on any terminal.
 *
 * Rendering      : Three-pass: (1) grid_draw rasterizes hex borders per pixel,
 *                  (2) pool_draw renders each placed object at its hex centre,
 *                  (3) cursor_draw places '@' at cursor hex centre.  The cursor
 *                  draws over objects so the player always sees their position.
 *
 * Performance    : grid_draw is O(rows × cols) per frame — the dominant cost.
 *                  pool_draw is O(MAX_OBJ).  No dynamic allocation after init.
 *
 * References     :
 *   Red Blob Games hex guide  — https://www.redblobgames.com/grids/hexagons/
 *   Cube coordinates          — https://www.redblobgames.com/grids/hexagons/#coordinates-cube
 *   Object pool pattern       — gameprogrammingpatterns.com/object-pool.html
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two independent coordinate systems are always in play:
 *
 *   HEX space   — axial (Q, R) integers.  The cursor and placed objects live
 *                 here.  Hex (0, 0) is the screen centre.  Distances between
 *                 hexes are hex_dist — independent of terminal size.
 *
 *   SCREEN space — (row, col) characters.  ncurses lives here.  The background
 *                  rasterizer visits every (row, col), converts to pixel,
 *                  applies the inverse matrix → (Q, R), and draws a border
 *                  character if that pixel is near a hex edge.
 *
 * Placed objects know ONLY their (Q, R) address.  To render them, apply the
 * forward matrix (hex centre → pixel) then divide by (CELL_W, CELL_H) to get
 * the screen cell.  This is the same formula cursor_draw uses for '@'.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Think of each hex as a named city.  Its name is (Q, R).  Its pixel location
 * (cx, cy) is computed on demand from the forward matrix — it is never stored.
 * The background map (grid_draw) redraws each frame by asking every pixel
 * "which city owns you?" via the inverse matrix.  Placed objects are sticky
 * notes attached to a city name; they rerender at that city's pixel location.
 *
 * DRAWING METHOD  (forward matrix — hex → screen)
 * ──────────────────────────────────────────────
 *  Given hex (Q, R) and radius parameter size:
 *
 *  1. Flat-top forward matrix (hex → pixel centre):
 *       cx = size × 3/2 × Q
 *       cy = size × (√3/2 × Q  +  √3 × R)
 *
 *  2. Terminal cell (centering offset ox = cols/2, oy = (rows-1)/2):
 *       col = ox + round(cx / CELL_W)
 *       row = oy + round(cy / CELL_H)
 *     cursor_draw uses truncation (int)(cx/CELL_W) to stay in interior.
 *
 *  3. Draw glyph at (row, col) if within visible bounds.
 *
 * KEY FORMULAS
 * ────────────
 *  Forward (hex → pixel centre):
 *    cx = size × 3/2 × Q
 *    cy = size × (√3/2 × Q  +  √3 × R)
 *
 *  Inverse (pixel → fractional cube):
 *    fq = (2/3 × px) / size
 *    fr = (−1/3 × px  +  √3/3 × py) / size
 *    fs = −fq − fr
 *
 *  cube_round (fractional cube → nearest integer hex):
 *    Round all three; fix the component with the LARGEST rounding error:
 *      if dq = max: Q = −rr − rs
 *      if dr = max: R = −rq − rs
 *      otherwise:   Q = rq, R = rr  (S = −Q − R implicit)
 *
 *  hex_dist (axial distance between two hexes A and B):
 *    d = (|dQ| + |dR| + |dQ + dR|) / 2
 *    where dQ = Q_B − Q_A, dR = R_B − R_A
 *
 *  Cube distance (border detection in grid_draw):
 *    dist = max(|fq − Q|, |fr − R|, |fs − S|)
 *    border if dist ≥ 0.5 − border_w
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • cube_round: never skip the "fix largest error" step. Rounding fq, fr, fs
 *    independently breaks Q+R+S=0. The fix step restores the constraint.
 *
 *  • pool_draw uses round() (not truncation). Objects must centre in their hex,
 *    not drift to the interior like '@'. One pixel of difference matters.
 *
 *  • CELL_H / CELL_W = 4/2 = 2. Terminal chars are ~2× taller than wide.
 *    Without this ratio, hexes appear squashed vertically on screen.
 *
 *  • Resize: ox/oy recompute from current rows/cols each frame. cQ/cR and
 *    HPool (Q, R) values stay valid — they live in hex space, not screen space.
 *
 * HOW TO VERIFY  (80×24 terminal, HEX_SIZE=14, CELL_W=2, CELL_H=4)
 * ─────────────
 *  Screen centre: ox=40, oy=11.
 *
 *  Cursor at (Q=0, R=0):
 *    cx = 0, cy = 0 → col=40, row=11 → '@' at (11,40). ✓
 *
 *  Place object at (Q=1, R=0):
 *    cx = 14×1.5×1 = 21,  cy = 14×(√3/2×1 + 0) ≈ 12.12
 *    col = 40 + round(21/2) = 50,  row = 11 + round(12.12/4) = 14
 *    → '*' at (14, 50). ✓
 *
 *  Move cursor RIGHT: cQ += +1 → (Q=1, R=0) → '@' at (14, 50).
 *  Draws over '*' — cursor always visible on top. ✓
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Aspect correction: terminal chars are ~2× taller than wide.
 * CELL_W=2, CELL_H=4 → 2:1 ratio so hexes appear circular, not oval. */
#define CELL_W              2
#define CELL_H              4

#define HEX_SIZE_DEFAULT   14.0   /* hex radius in pixels (sub-cell units) */
#define HEX_SIZE_MIN        6.0
#define HEX_SIZE_MAX       40.0
#define HEX_SIZE_STEP       2.0

/* Border width as a fraction of cube distance [0..0.5].
 * 0.10 ≈ 1–2 terminal cells of border visible inside each hex. */
#define BORDER_W_DEFAULT    0.10
#define BORDER_W_MIN        0.03
#define BORDER_W_MAX        0.35

#define MAX_OBJ            256   /* max placed objects; far more than visible */
#define FRAME_NS    16666667LL   /* ~60 fps                                   */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

#define PAIR_GRID    1   /* dim hex border lines        */
#define PAIR_CURSOR  2   /* cursor hex + '@' character  */
#define PAIR_OBJ     3   /* placed object glyphs        */
#define PAIR_HUD     4   /* status bar                  */
#define PAIR_HINT    5   /* key hint strip              */

static void color_init(void) {
    start_color();
    use_default_colors();
    init_pair(PAIR_GRID,   COLORS >= 256 ?  75 : COLOR_CYAN,    -1);
    init_pair(PAIR_CURSOR, COLOR_WHITE,                COLOR_BLUE);
    init_pair(PAIR_OBJ,    COLORS >= 256 ? 226 : COLOR_YELLOW,  -1);
    init_pair(PAIR_HUD,    COLOR_BLACK,                COLOR_CYAN);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  75 : COLOR_CYAN,    -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  coords — cube_round, hex_to_screen, hex_dist, angle_char           */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * cube_round — nearest integer hex to fractional cube position (fq, fr, fs).
 *
 * THE FORMULA:
 *   Round all three axes independently: rq=round(fq), rr=round(fr), rs=round(fs).
 *   Rounding can break Q+R+S=0, so fix the component with the LARGEST error:
 *     dq=|rq-fq|, dr=|rr-fr|, ds=|rs-fs|
 *     if dq is max: Q = -rr - rs   (recomputed to restore constraint)
 *     if dr is max: R = -rq - rs
 *     else:         Q = rq, R = rr  (S = -Q-R, implicit)
 *
 * WHY fix the largest-error component: it was rounded the most aggressively,
 * so reassigning it from the constraint loses the least accuracy in the other
 * two axes (which were rounded more conservatively).
 */
static void cube_round(double fq, double fr, double fs, int *Q, int *R) {
    int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
    double dq = fabs((double)rq - fq);
    double dr = fabs((double)rr - fr);
    double ds = fabs((double)rs - fs);
    if      (dq > dr && dq > ds) { *Q = -rr - rs; *R = rr; }
    else if (dr > ds)             { *Q = rq; *R = -rq - rs; }
    else                          { *Q = rq; *R = rr; }
}

/*
 * hex_to_screen — project hex (Q, R) to terminal (col, row).
 *
 * THE FORMULA (flat-top forward matrix + aspect correction):
 *
 *   Pixel centre of hex (Q, R):
 *     cx = size × 3/2 × Q
 *     cy = size × (√3/2 × Q  +  √3 × R)
 *
 *   Terminal cell (centering offset ox, oy):
 *     col = ox + round(cx / CELL_W)
 *     row = oy + round(cy / CELL_H)
 *
 * round() (not truncation) keeps object glyphs centred in their hex interior.
 * cursor_draw uses truncation to guarantee '@' stays inside the border ring.
 */
static void hex_to_screen(double size, int Q, int R, int ox, int oy,
                           int *col, int *row) {
    double sq3   = sqrt(3.0);
    double cx    = size * 1.5 * (double)Q;
    double cy    = size * (sq3 * 0.5 * (double)Q + sq3 * (double)R);
    *col = ox + (int)round(cx / CELL_W);
    *row = oy + (int)round(cy / CELL_H);
}

/*
 * angle_char — map a tangent angle to the best-fit ASCII line character.
 *
 * THE FORMULA:
 *   Input theta = atan2(py-cy, px-cx) + π/2  (radial → tangent direction).
 *   Fold into [0, π) — ASCII chars are symmetric under 180° rotation.
 *   Map: [0, π/8) → '-'  [π/8, 3π/8) → '\'  [3π/8, 5π/8) → '|'
 *        [5π/8, 7π/8) → '/'  [7π/8, π) → '-'
 */
static char angle_char(double theta) {
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if      (t < M_PI / 8.0)        return '-';
    else if (t < 3.0 * M_PI / 8.0)  return '\\';
    else if (t < 5.0 * M_PI / 8.0)  return '|';
    else if (t < 7.0 * M_PI / 8.0)  return '/';
    else                              return '-';
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  pool — flat object array with O(1) removal                         */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int Q, R; char glyph; } HObj;
typedef struct { HObj items[MAX_OBJ]; int count; } HPool;

/*
 * pool_toggle — add '*' at (Q,R) if absent; remove it if present.
 *
 * Removal swaps the target with the last item and decrements count — O(1).
 * This avoids shifting the array but makes the pool unordered.  Draw order
 * doesn't matter for placement (each hex is unique), so this is fine.
 */
static void pool_toggle(HPool *p, int Q, int R) {
    for (int i = 0; i < p->count; i++) {
        if (p->items[i].Q == Q && p->items[i].R == R) {
            p->items[i] = p->items[--p->count]; /* swap-last remove */
            return;
        }
    }
    if (p->count < MAX_OBJ) {
        p->items[p->count++] = (HObj){ Q, R, '*' };
    }
}

static void pool_clear(HPool *p) { p->count = 0; }

/*
 * pool_draw — render all objects using the flat-top forward matrix.
 *
 * THE FORMULA: for each (Q, R) in pool → hex_to_screen → bounds check → draw.
 * Drawing over the grid background is intentional: objects sit ON the grid.
 */
static void pool_draw(const HPool *p, double size, int ox, int oy,
                       int rows, int cols) {
    attron(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        int col, row;
        hex_to_screen(size, p->items[i].Q, p->items[i].R, ox, oy, &col, &row);
        if (col >= 0 && col < cols && row >= 0 && row < rows - 1)
            mvaddch(row, col, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
}

/* ── end §5 — to understand the background hex rendering, read §6 ─────── */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  bgrid — flat-top hex rasterizer                                    */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * grid_draw — rasterize the flat-top hex background with cursor highlighted.
 *
 * THE PIPELINE (per screen cell):
 *
 *   (col, row) → pixel:  px = (col-ox)×CELL_W,  py = (row-oy)×CELL_H
 *      ↓
 *   Inverse matrix → fractional cube:
 *     fq = (2/3 × px) / size
 *     fr = (-px/3 + √3·py/3) / size
 *     fs = -fq - fr
 *      ↓
 *   cube_round → integer hex (Q, R)
 *      ↓
 *   Cube distance: dist = max(|fq-Q|, |fr-R|, |fs-(-Q-R)|)
 *      ├── dist < 0.5-border_w → interior → skip
 *      └── dist ≥ 0.5-border_w → border:
 *            cx = size×1.5×Q,  cy = size×(√3/2×Q + √3×R)
 *            theta = atan2(py-cy, px-cx)
 *            ch = angle_char(theta + π/2)
 *            color = PAIR_CURSOR if Q==cQ && R==cR, else PAIR_GRID
 */
static void grid_draw(int rows, int cols, double size, double border_w,
                       int cQ, int cR, int ox, int oy) {
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double limit = 0.5 - border_w;

    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            double px = (double)(col - ox) * CELL_W;
            double py = (double)(row - oy) * CELL_H;

            double fq = (2.0/3.0 * px) / size;
            double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
            double fs = -fq - fr;

            int Q, R;
            cube_round(fq, fr, fs, &Q, &R);
            double fS = (double)(-Q - R);

            double dist = fabs(fq - (double)Q);
            double d2   = fabs(fr - (double)R);
            double d3   = fabs(fs - fS);
            if (d2 > dist) dist = d2;
            if (d3 > dist) dist = d3;
            if (dist < limit) continue;

            double cx    = size * 1.5 * (double)Q;
            double cy    = size * (sq3_2 * (double)Q + sq3 * (double)R);
            double theta = atan2(py - cy, px - cx);
            char ch = angle_char(theta + M_PI / 2.0);

            int on_cur = (Q == cQ && R == cR);
            int attr   = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                : (COLOR_PAIR(PAIR_GRID)   | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  cursor — movement vectors and '@' placement                        */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * HEX_DIR — axial movement deltas for the 4 arrow keys.
 *
 * Flat-top hex: UP/DOWN move along R, LEFT/RIGHT move along Q.
 * The 2 diagonal faces (NE = +Q−R, SW = −Q+R) are not mapped to keys.
 *
 *             UP: (0, -1)
 *  LEFT: (-1, 0)  ●  RIGHT: (+1, 0)
 *            DOWN: (0, +1)
 */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    — R decreases */
    { 0, +1 },   /* DOWN  — R increases */
    {-1,  0 },   /* LEFT  — Q decreases */
    {+1,  0 },   /* RIGHT — Q increases */
};

/*
 * cursor_draw — place '@' at the interior centre of hex (cQ, cR).
 *
 * THE FORMULA:
 *   cx_pix = size × 3/2 × cQ
 *   cy_pix = size × (√3/2 × cQ  +  √3 × cR)
 *   col = ox + (int)(cx_pix / CELL_W)   ← truncation stays in interior
 *   row = oy + (int)(cy_pix / CELL_H)
 */
static void cursor_draw(double size, int cQ, int cR,
                         int ox, int oy, int rows, int cols) {
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx    = size * 1.5    * (double)cQ;
    double cy    = size * (sq3_2 * (double)cQ + sq3 * (double)cR);
    int col = ox + (int)(cx / CELL_W);
    int row = oy + (int)(cy / CELL_H);
    if (col >= 0 && col < cols && row >= 0 && row < rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(row, col, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    double hex_size;
    double border_w;
    int    cQ, cR;     /* cursor in axial hex coordinates */
    HPool  pool;
} Scene;

static void scene_init(Scene *s) {
    s->hex_size = HEX_SIZE_DEFAULT;
    s->border_w = BORDER_W_DEFAULT;
    s->cQ = 0; s->cR = 0;
    pool_clear(&s->pool);
}

static void scene_draw(const Scene *s, int rows, int cols) {
    int ox = cols / 2;
    int oy = (rows - 1) / 2;
    grid_draw(rows, cols, s->hex_size, s->border_w, s->cQ, s->cR, ox, oy);
    pool_draw(&s->pool, s->hex_size, ox, oy, rows, cols);
    cursor_draw(s->hex_size, s->cQ, s->cR, ox, oy, rows, cols);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void cleanup(void) { endwin(); }

static void screen_init(void) {
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); curs_set(0);
    nodelay(stdscr, TRUE); typeahead(-1);
    color_init();
    atexit(cleanup);
}

static void screen_hud(const Scene *s, int rows, int cols, double fps) {
    char buf[80];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d  obj:%d  size:%.0f  %5.1f fps ",
             s->cQ, s->cR, s->pool.count, s->hex_size, fps);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    attron(COLOR_PAIR(PAIR_HINT) | A_DIM);
    mvprintw(rows - 1, 0,
             " q:quit  arrows:move  spc:toggle  C:clear  r:reset  +/-:size ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_DIM);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ═══════════════════════════════════════════════════════════════════════ */
/* §10 app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t running = 1, need_resize = 0;
static void sig_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) running = 0;
    if (sig == SIGWINCH)                 need_resize = 1;
}

int main(void) {
    signal(SIGINT, sig_handler); signal(SIGTERM, sig_handler);
    signal(SIGWINCH, sig_handler);
    screen_init();

    int rows, cols; getmaxyx(stdscr, rows, cols);
    Scene sc; scene_init(&sc);
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
            case 'r': sc.cQ = 0; sc.cR = 0; break;
            case 'C': pool_clear(&sc.pool); break;
            case ' ': pool_toggle(&sc.pool, sc.cQ, sc.cR); break;
            case KEY_UP:
                sc.cQ += HEX_DIR[0][0]; sc.cR += HEX_DIR[0][1]; break;
            case KEY_DOWN:
                sc.cQ += HEX_DIR[1][0]; sc.cR += HEX_DIR[1][1]; break;
            case KEY_LEFT:
                sc.cQ += HEX_DIR[2][0]; sc.cR += HEX_DIR[2][1]; break;
            case KEY_RIGHT:
                sc.cQ += HEX_DIR[3][0]; sc.cR += HEX_DIR[3][1]; break;
            case '+': case '=':
                if (sc.hex_size < HEX_SIZE_MAX) sc.hex_size += HEX_SIZE_STEP;
                break;
            case '-':
                if (sc.hex_size > HEX_SIZE_MIN) sc.hex_size -= HEX_SIZE_STEP;
                break;
            }
        }
        int64_t now = clock_ns();
        int64_t dt  = now - prev; prev = now;
        if (dt > 0) fps = fps * 0.9 + 1e9 / (double)dt * 0.1;

        erase();
        scene_draw(&sc, rows, cols);
        screen_hud(&sc, rows, cols, fps);
        screen_present();
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
