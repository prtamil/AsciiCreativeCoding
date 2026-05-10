/*
 * 02_pointy_top.c — pointy-top hexagonal grid with keyboard-controlled cursor
 *
 * DEMO: Same cube-coordinate + cursor logic as 01_flat_top.c but with the
 *       hex rotated 30°. Pointy-top flat sides face left/right → '|'.
 *       '@' cursor moves between hexes with arrow keys.
 *
 * Study alongside: grids/hex_grids/01_flat_top.c
 *                  (identical Cursor + HEX_DIR; only matrix entries differ)
 *
 * Section map:
 *   §1 config   — tunable constants, EWMA alpha
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — ncurses color pairs (BORDER, CURSOR, HUD, HINT)
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_pixel_to_axial /
 *                 ctx_draw_bg + cube_round + angle_char (pointy-top matrix)
 *   §5 cursor   — Cursor + cursor_reset / cursor_move / cursor_draw, HEX_DIR
 *   §6 scene    — hud_draw + scene_draw
 *   §7 screen   — ncurses display layer
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit  p pause  t theme  r reset  arrows move  +/-:size  [/]:border
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/hex_grids/02_pointy_top.c \
 *       -o 02_pointy_top -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Pointy-top hex = flat-top rotated 30°. Only the 2×2
 *                  inverse matrix changes:
 *                    fq = (√3/3·px − 1/3·py) / s
 *                    fr = (2/3·py) / s
 *                  Cube→pixel: cx = s(√3Q + √3/2·R), cy = s·3/2·R.
 *
 * Data-structure : Same GridCtx + Cursor as 01_flat_top — no grid array,
 *                  axial (cur->q, cur->r) cursor only. The only
 *                  orientation-dependent state is the inverse matrix in §4;
 *                  cube space is unchanged.
 *
 * Cursor movement : Same HEX_DIR[4][2] deltas as 01_flat_top — the axial
 *                  system is orientation-independent. RIGHT still adds Q+1,
 *                  UP subtracts R by 1, etc. cursor_draw uses the pointy-top
 *                  forward matrix for hex→pixel conversion.
 *
 * References     :
 *   Pointy-top matrix — https://www.redblobgames.com/grids/hexagons/#hex-to-pixel-pointy
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Pointy-top is flat-top rotated 30° around the screen center. The cube
 * coordinate math (Q+R+S=0, cube_round, cube distance, border threshold) is
 * IDENTICAL. Only the 2×2 transform matrix that converts between pixel space
 * and cube space changes. Compare the two matrices side-by-side:
 *
 *   Flat-top inverse:               Pointy-top inverse:
 *     fq = (2/3·px) / s               fq = (√3/3·px − 1/3·py) / s
 *     fr = (−1/3·px + √3/3·py) / s   fr = (2/3·py) / s
 *
 * The flat-top matrix has a pure-x term for fq; the pointy-top matrix has a
 * pure-y term for fr. This is exactly a 30° rotation of the basis vectors.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * In flat-top orientation the hex has a flat edge at the top. Move the whole
 * hex grid 30° counterclockwise: now a vertex points up, and the flat edge
 * faces left/right. The tiling still covers the plane the same way — just
 * rotated. Every formula that worked on cube coordinates still works; only the
 * front-door (pixel→cube) and back-door (cube→pixel) change to reflect the
 * new orientation.
 *
 * The cursor movement table (HEX_DIR) is in AXIAL space (Q,R), which is
 * independent of orientation. So the same 4 arrow keys work for both files.
 *
 * DRAWING METHOD  (identical to 01, with different matrix entries)
 * ──────────────────────────────────────────────────────────────────
 *  1. Center the grid: px = (col−ox)×CELL_W, py = (row−oy)×CELL_H.
 *
 *  2. Pointy-top inverse matrix:
 *       fq = (√3/3·px − 1/3·py) / size
 *       fr = (2/3·py) / size
 *       fs = −fq − fr
 *
 *  3. cube_round (same as 01_flat_top — orientation-independent).
 *
 *  4. Cube distance (same as 01_flat_top).
 *
 *  5. Skip interior pixels (dist < 0.5 − border_w).
 *
 *  6. Pointy-top forward matrix for hex center:
 *       cx = size × (√3·Q + √3/2·R)
 *       cy = size × 3/2 × R
 *
 *  7. angle_char(theta + π/2) — same function as 01_flat_top.
 *
 *  8. Draw character in cursor or border color.
 *
 * KEY FORMULAS
 * ────────────
 *  Pointy-top inverse matrix (pixel → fractional cube):
 *    fq = (√3/3·px − 1/3·py) / size
 *    fr = (2/3·py) / size
 *
 *  Pointy-top forward matrix (hex → pixel center):
 *    cx = size × (√3·Q + √3/2·R)
 *    cy = size × 3/2 × R
 *
 *  Derivation: pointy-top forward = flat-top forward × R(−30°), where
 *    R(−30°) is a 30° clockwise rotation matrix. The inverse is R(+30°).
 *    Working out the product gives the formulas above.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • The flat top/bottom edges that show '-' in flat-top now become the
 *    left/right edges and show '|'. The diagonal edges (\ and /) remain but
 *    swap roles. angle_char is the same function — it adapts automatically
 *    because theta is computed from the actual pixel→center geometry.
 *
 *  • cursor_draw MUST use the pointy-top forward matrix, not flat-top.
 *    Using the wrong formula shifts '@' to the wrong hex center visually.
 *
 *  • HEX_DIR deltas are the same in both files because they operate in
 *    axial (Q,R) space, not screen space.
 *
 * HOW TO VERIFY  (80×24 terminal, HEX_SIZE=14)
 * ─────────────
 *  Origin hex (Q=0,R=0) center: cx=0, cy=0 → col=ox=40, row=oy=11.
 *  '@' at (11,40). ✓ Same screen position as 01_flat_top.
 *
 *  Check that top vertex is vertical:
 *  Pixel directly above center: py=−16 (row=7), px=0.
 *    fq = (0 − (−16)/3) / 14 = (16/3)/14 ≈ 0.381
 *    fr = (2/3·(−16)) / 14 = −32/42 ≈ −0.762
 *    cube_round → Q=0, R=−1 (the hex directly above).
 *    Pointy-top: hex (0,−1) center: cx=0, cy=14·(−1.5)=−21 pixels.
 *    That hex's top vertex is at cy=−21−14 = screen up. ✓ Vertex points up.
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

#define CELL_W             2
#define CELL_H             4

#define HEX_SIZE_DEFAULT  14.0
#define HEX_SIZE_MIN       6.0
#define HEX_SIZE_MAX      40.0
#define HEX_SIZE_STEP      2.0

#define BORDER_W_DEFAULT   0.10
#define BORDER_W_MIN       0.03
#define BORDER_W_MAX       0.35
#define BORDER_W_STEP      0.02

#define N_THEMES           4
#define TICK_NS           16666667LL

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA     0.05

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

#define PAIR_BORDER   1
#define PAIR_CURSOR   2
#define PAIR_HUD      3   /* yellow status bar */
#define PAIR_HINT     4   /* cyan key-hint footer */

static const short THEMES[N_THEMES][2] = {
    { COLOR_CYAN,   COLOR_BLACK },
    { COLOR_GREEN,  COLOR_BLACK },
    { COLOR_YELLOW, COLOR_BLACK },
    { COLOR_WHITE,  COLOR_BLACK },
};

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_BORDER, THEMES[theme][0], THEMES[theme][1]);
    init_pair(PAIR_CURSOR, COLOR_WHITE, COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the hex ↔ screen mapping (pointy-top)         */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — geometry of the active pointy-top hex grid.
 *
 * Centering: px = (col - ox) × CELL_W, py = (row - oy) × CELL_H.
 *
 * Pointy-top pixel↔cube:
 *   fq = (√3/3·px − 1/3·py) / size   (inverse matrix row 1)
 *   fr = (2/3·py) / size               (inverse matrix row 2)
 *   cx = size × (√3·Q + √3/2·R)       (forward matrix column 1)
 *   cy = size × 3/2 × R               (forward matrix column 2)
 *
 * Compare flat-top: only the matrix entries differ; cube_round, cube dist,
 * angle_char, and HEX_DIR are identical.
 */
typedef struct {
    /* terminal extent */
    int rows, cols;

    /* hex geometry */
    double hex_size;
    double border_w;
    int    cell_w, cell_h;

    /* screen origin = pixel (0,0) */
    int    ox, oy;

    /* cursor bounds in axial space (advisory; hex plane is infinite) */
    int    max_q, max_r;
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows   = rows;
    g->cols   = cols;
    g->cell_w = CELL_W;
    g->cell_h = CELL_H;
    g->ox     = cols / 2;
    g->oy     = (rows - 1) / 2;
    if (g->hex_size <= 0.0) g->hex_size = HEX_SIZE_DEFAULT;
    if (g->border_w <= 0.0) g->border_w = BORDER_W_DEFAULT;
    g->max_q = (int)((double)cols * CELL_W / (sqrt(3.0) * g->hex_size)) + 1;
    g->max_r = (int)((double)rows * CELL_H / (3.0      * g->hex_size)) + 1;
}

/*
 * ctx_to_screen — center cell of pointy-top hex (Q, R) in screen coordinates.
 *
 *   cx_pix = size × (√3 × Q  +  √3/2 × R)
 *   cy_pix = size × 3/2 × R
 *   sc = ox + (int)(cx_pix / CELL_W)
 *   sr = oy + (int)(cy_pix / CELL_H)
 *
 * Compare to flat-top:
 *   flat-top:   cx = 3/2·Q·s,                  cy = (√3/2·Q + √3·R)·s
 *   pointy-top: cx = (√3·Q + √3/2·R)·s,        cy = 3/2·R·s
 * The Q and R roles are "swapped" between x and y — the 30° rotation.
 */
static void ctx_to_screen(const GridCtx *g, int Q, int R, int *sr, int *sc)
{
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx_pix = g->hex_size * (sq3   * (double)Q + sq3_2 * (double)R);
    double cy_pix = g->hex_size * 1.5    * (double)R;
    *sc = g->ox + (int)(cx_pix / g->cell_w);
    *sr = g->oy + (int)(cy_pix / g->cell_h);
}

/*
 * cube_round — round fractional cube to integer (Q,R) restoring Q+R+S=0.
 * Identical to flat-top — orientation-independent.
 */
static void cube_round(double fq, double fr, double fs, int *Q, int *R)
{
    int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
    double dq = fabs((double)rq - fq);
    double dr = fabs((double)rr - fr);
    double ds = fabs((double)rs - fs);
    if      (dq > dr && dq > ds) rq = -rr - rs;
    else if (dr > ds)             rr = -rq - rs;
    *Q = rq; *R = rr;
}

/*
 * ctx_pixel_to_axial — terminal cell (sr, sc) → axial (Q, R) via the
 * pointy-top inverse matrix and cube_round.
 */
__attribute__((unused))
static void ctx_pixel_to_axial(const GridCtx *g, int sr, int sc, int *Q, int *R)
{
    double sq3_3 = sqrt(3.0) / 3.0;
    double px = (double)(sc - g->ox) * g->cell_w;
    double py = (double)(sr - g->oy) * g->cell_h;
    double fq = (sq3_3 * px - 1.0/3.0 * py) / g->hex_size;
    double fr = (2.0/3.0 * py) / g->hex_size;
    double fs = -fq - fr;
    cube_round(fq, fr, fs, Q, R);
}

/*
 * angle_char — same as 01_flat_top. See that file for full documentation.
 *
 * Works unchanged for pointy-top because theta is computed from the actual
 * geometry (atan2 of pixel relative to hex center), not from orientation
 * assumptions. The pointy-top edges happen to produce different theta values
 * that naturally map to '|' on the left/right faces.
 */
static char angle_char(double theta)
{
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if      (t < M_PI / 8.0)         return '-';
    else if (t < 3.0 * M_PI / 8.0)   return '\\';
    else if (t < 5.0 * M_PI / 8.0)   return '|';
    else if (t < 7.0 * M_PI / 8.0)   return '/';
    else                              return '-';
}

/*
 * ctx_draw_bg — pointy-top hex rasterizer.
 *
 * THE PIPELINE (differences from 01_flat_top marked with ★):
 *
 *   px = (col−ox)×CELL_W,  py = (row−oy)×CELL_H
 *      │
 *      ▼  ★ Pointy-top inverse matrix (note the row-major swap):
 *   fq = (√3/3·px − 1/3·py) / size     ← mixes x and y
 *   fr = (2/3·py) / size                ← pure y
 *   fs = −fq − fr
 *      │
 *      ▼  cube_round (identical to flat-top)
 *      │
 *      ▼  cube distance (identical to flat-top)
 *      │
 *      ├── interior: skip
 *      │
 *      └── border:
 *            ★ cx = size × (√3·Q + √3/2·R)   ← pointy-top forward x
 *            ★ cy = size × 3/2 × R            ← pointy-top forward y
 *            theta = atan2(py−cy, px−cx)
 *            ch = angle_char(theta + π/2)
 *
 * WHY THE MATRIX SWAP WORKS:
 *   Pointy-top = flat-top rotated 30° CCW. The rotation matrix is:
 *     [cos30   −sin30] = [√3/2  −1/2]
 *     [sin30    cos30]   [1/2    √3/2]
 *   Applying this to the flat-top inverse gives the pointy-top inverse
 *   above. The cube_round and distance logic are rotation-invariant.
 */
static void ctx_draw_bg(const GridCtx *g, int cQ, int cR)
{
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double size  = g->hex_size;
    double limit = 0.5 - g->border_w;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;

            /* Pointy-top inverse matrix */
            double fq = (sq3_3 * px - 1.0/3.0 * py) / size;
            double fr = (2.0/3.0 * py) / size;
            double fs = -fq - fr;

            int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
            double dq = fabs((double)rq - fq);
            double dr = fabs((double)rr - fr);
            double ds = fabs((double)rs - fs);
            if      (dq > dr && dq > ds) rq = -rr - rs;
            else if (dr > ds)             rr = -rq - rs;
            int Q = rq, R = rr;

            double fQ = (double)Q, fR = (double)R, fS = (double)(-Q - R);
            double dist = fabs(fq - fQ);
            double d2   = fabs(fr - fR);
            double d3   = fabs(fs - fS);
            if (d2 > dist) dist = d2;
            if (d3 > dist) dist = d3;
            if (dist < limit) continue;

            /* Pointy-top forward matrix */
            double cx = size * (sq3 * fQ + sq3_2 * fR);
            double cy = size * 1.5 * fR;
            double theta = atan2(py - cy, px - cx);
            char ch = angle_char(theta + M_PI / 2.0);

            int on_cur = (Q == cQ && R == cR);
            int attr   = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — just (q, r) in axial hex space. Same struct as 01_flat_top.
 */
typedef struct { int q, r; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->q = 0;
    cur->r = 0;
}

/*
 * HEX_DIR — 4-direction movement in axial (Q, R).
 *
 * Pointy-top screen layout (same deltas as flat-top):
 *
 *               UP: (0,-1)  ← cy = 3/2·R·s, so R-1 moves up
 *                    ↑
 *   LEFT: (-1,0) ← ● → RIGHT: (+1,0)   ← cx = √3·Q·s, Q+1 moves right
 *                    ↓
 *              DOWN: (0,+1)
 *
 * WHY SAME DELTAS AS FLAT-TOP:
 *   HEX_DIR operates in axial (Q,R) space which has no notion of screen
 *   orientation. Both the flat-top and pointy-top grids use the same axial
 *   coordinate labels — only the pixel positions of those hexes differ.
 *   A step of Δ(Q=+1, R=0) moves one hex to the right in both orientations.
 *
 * Unmapped faces for pointy-top: (Q+1,R-1) upper-right and (Q-1,R+1) lower-left.
 */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    */
    { 0, +1 },   /* DOWN  */
    {-1,  0 },   /* LEFT  */
    {+1,  0 },   /* RIGHT */
};

static void cursor_move(Cursor *cur, const GridCtx *g, int dq, int dr)
{
    (void)g;
    cur->q += dq;
    cur->r += dr;
}

/*
 * cursor_draw — '@' at the center cell of the pointy-top cursor hex.
 *
 * Uses ctx_to_screen, which embeds the pointy-top forward matrix.
 */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->q, cur->r, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, int theme,
                     int paused, double fps)
{
    char buf[96];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d  size:%.0f  border:%.2f  theme:%d  %5.1f fps  %s ",
             cur->q, cur->r, g->hex_size, g->border_w, theme, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:size  [/]:border ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int theme,
                       int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->q, cur->r);
    cursor_draw(cur, g);
    hud_draw(g, cur, theme, paused, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }

static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
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

    screen_init();
    int theme = 0, paused = 0;
    color_init(theme);

    GridCtx g = {0};
    g.hex_size = HEX_SIZE_DEFAULT;
    g.border_w = BORDER_W_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    double fps = 60.0;
    int64_t prev = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS);
        }
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_running = 0; break;
            case 'p': paused ^= 1; break;
            case 'r': cursor_reset(&cur, &g); break;
            case 't':
                theme = (theme + 1) % N_THEMES;
                color_init(theme);
                break;
            case KEY_UP:    cursor_move(&cur, &g, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, &g, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, &g, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, &g, HEX_DIR[3][0], HEX_DIR[3][1]); break;
            case '+': case '=':
                if (g.hex_size < HEX_SIZE_MAX) { g.hex_size += HEX_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
            case '-':
                if (g.hex_size > HEX_SIZE_MIN) { g.hex_size -= HEX_SIZE_STEP; ctx_init(&g, LINES, COLS); } break;
            case '[':
                if (g.border_w > BORDER_W_MIN) { g.border_w -= BORDER_W_STEP; } break;
            case ']':
                if (g.border_w < BORDER_W_MAX) { g.border_w += BORDER_W_STEP; } break;
            }
        }

        int64_t now = clock_ns(), dt = now - prev; prev = now;
        if (dt > 0)
            fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (double)dt) * FPS_EWMA_ALPHA;

        scene_draw(&g, &cur, theme, paused, fps);
        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }
    return 0;
}
