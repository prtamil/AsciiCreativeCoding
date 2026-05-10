/*
 * 03_axial.c — flat-top hex grid with axial labels and keyboard cursor
 *
 * DEMO: Each hexagon shows its "Q,R" axial coordinates. The cursor hex is
 *       highlighted white-on-blue with '@' at its center. Q=0/R=0/S=0 axes
 *       glow in cyan/green/yellow so you can trace them across the plane.
 *
 * Study alongside: grids/hex_grids/01_flat_top.c (same Cursor + HEX_DIR;
 *                  this file adds a label pass)
 *
 * Section map:
 *   §1 config   — tunable constants, EWMA alpha
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — axis color pairs (Q/R/S/origin/cursor) + HUD/HINT
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_pixel_to_axial /
 *                 ctx_draw_bg (Pass 1) / ctx_draw_labels (Pass 2)
 *                 + cube_round + angle_char + hex_color
 *   §5 cursor   — Cursor + cursor_reset / cursor_move / cursor_draw, HEX_DIR
 *   §6 scene    — hud_draw + two-pass scene_draw
 *   §7 screen   — ncurses display layer
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit  p pause  r reset  arrows move  +/-:size  [/]:border
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/hex_grids/03_axial.c \
 *       -o 03_axial -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two-pass render. Pass 1 (ctx_draw_bg): hex borders
 *                  colored by axis membership. Pass 2 (ctx_draw_labels):
 *                  enumerate visible hexes, print label at each center.
 *                  Labels land on interior cells (dist≈0) skipped in pass 1.
 *
 * Data-structure : Same GridCtx + Cursor as 01_flat_top. Pass 2 enumerates
 *                  the visible (Q,R) range derived from screen bounds +
 *                  hex_size to print labels. Axis membership (Q=0, R=0, S=0)
 *                  is a runtime test on the current (Q,R), not stored.
 *
 * Cursor movement : Same HEX_DIR[4][2] as 01_flat_top. '@' drawn in a third
 *                  pass so it always sits on top of the label. Cursor hex
 *                  border is drawn in PAIR_CURSOR in pass 1.
 *
 * References     :
 *   Axial vs cube — https://www.redblobgames.com/grids/hexagons/#coordinates-axial
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Axial coordinates (Q, R) are a 2-component alias for cube coordinates.
 * Since S = −Q−R is always derivable, you only need to store Q and R.
 * This file makes the coordinate structure visible: each hex displays its
 * own (Q,R) label, and the three coordinate axes are colored differently
 * so you can see how the hex plane is partitioned.
 *
 * The three axis planes of cube space slice the hex plane into six sectors:
 *   Q=0 axis  (cyan):   hexes where Q=0  — a diagonal band of hexes
 *   R=0 axis  (green):  hexes where R=0  — another diagonal band
 *   S=0 axis  (yellow): hexes where S=0, i.e., Q+R=0 — the third band
 *   Origin    (white):  the single hex where Q=R=0 (S=0 automatically)
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the hexagonal plane as the diagonal cross-section of a 3-D cubic
 * lattice. Each hex corresponds to a unit cube whose (x,y,z) coordinates
 * satisfy x+y+z=0. The Q=0 axis is the set of cubes where x=0 — a plane
 * in 3-D that slices the hex plane as a line of hexes.
 *
 * Moving RIGHT (+Q): you climb along the Q-axis; the label's Q number
 *   increases by 1, R stays the same.
 * Moving DOWN (+R): R number increases by 1, Q stays the same.
 * Moving diagonally (would be NE or SW): both Q and R change.
 *
 * The label in each hex lets you directly read off the coordinate system
 * rather than counting from the origin — very useful when learning hex math.
 *
 * DRAWING METHOD  (two-pass rendering)
 * ─────────────────────────────────────
 *  PASS 1 — ctx_draw_bg (border rasterizer, same as 01_flat_top):
 *   For each screen cell, compute cube coords (Q,R) and cube distance.
 *   If border pixel: draw angle_char in the color given by hex_color(Q,R).
 *   hex_color priority: cursor > origin > Q-axis > R-axis > S-axis > default.
 *
 *  PASS 2 — ctx_draw_labels (label printer):
 *   Loop over all (Q,R) whose hex center would be on screen.
 *   Compute screen cell via ctx_to_screen. Print "Q,R" centered on that
 *   cell in the appropriate axis color. This pass only writes interior
 *   cells (never border cells), so pass 1 characters are not overwritten
 *   by labels.
 *
 *  PASS 3 — cursor_draw ('@'):
 *   Draw '@' on top of the label in the cursor hex. Must be last so '@'
 *   overwrites the label character at the cursor hex center.
 *
 * KEY FORMULAS
 * ────────────
 *  hex_color priority (highest wins):
 *    cursor      if Q==cur->q && R==cur->r
 *    origin      if Q==0  && R==0
 *    Q-axis      if Q==0
 *    R-axis      if R==0
 *    S-axis      if −Q−R==0  (i.e., Q+R=0)
 *    default     otherwise
 *
 *  Visible hex count (for label pass iteration bounds):
 *    Qmax ≈ cols × CELL_W / (1.5 × size)
 *    Rmax ≈ rows × CELL_H / (√3  × size)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Label centering: "Q,R" has variable length (e.g., "-10,-3" is 7 chars).
 *    Use lx = sc − len/2 and guard lx >= 0 && lx+len < cols.
 *
 *  • At large sizes the label may be wider than the hex interior. Guard with
 *    sc > 2 && sc < cols-2 to avoid partial labels at screen edges.
 *
 *  • hex_color: cursor check must come FIRST — if cur->q=0, the cursor hex
 *    is also on the Q-axis; we want cursor color to win over axis color.
 *
 * HOW TO VERIFY  (80×24 terminal, HEX_SIZE=20)
 * ─────────────
 *  Origin hex (Q=0, R=0): label "0,0" drawn in white (PAIR_ORIGIN), '@' on top.
 *  Hex (1, 0): cx = 20×1.5=30 pixels → col = 40+15=55.  label "1,0" in green (R=0).
 *  Hex (0, 1): cy = 20×√3=34.6 px → row = 11+8=19.     label "0,1" in cyan (Q=0).
 *  Hex (1,-1): Q+R = 0 → S-axis color (yellow).         label "1,-1".
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

#define HEX_SIZE_DEFAULT  20.0
#define HEX_SIZE_MIN      10.0
#define HEX_SIZE_MAX      50.0
#define HEX_SIZE_STEP      2.0

#define BORDER_W_DEFAULT   0.08
#define BORDER_W_MIN       0.03
#define BORDER_W_MAX       0.25
#define BORDER_W_STEP      0.02

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

#define PAIR_DEFAULT  1
#define PAIR_Q_AXIS   2   /* cyan   — hexes where Q=0 */
#define PAIR_R_AXIS   3   /* green  — hexes where R=0 */
#define PAIR_S_AXIS   4   /* yellow — hexes where S=Q+R=0 */
#define PAIR_ORIGIN   5   /* white  — the single hex (0,0,0) */
#define PAIR_CURSOR   6   /* cursor hex: white-on-blue */
#define PAIR_HUD      7   /* yellow status bar */
#define PAIR_HINT     8   /* cyan key-hint footer */

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_DEFAULT, COLOR_WHITE,  COLOR_BLACK);
    init_pair(PAIR_Q_AXIS,  COLOR_CYAN,   COLOR_BLACK);
    init_pair(PAIR_R_AXIS,  COLOR_GREEN,  COLOR_BLACK);
    init_pair(PAIR_S_AXIS,  COLOR_YELLOW, COLOR_BLACK);
    init_pair(PAIR_ORIGIN,  COLOR_WHITE,  COLOR_BLACK);
    init_pair(PAIR_CURSOR,  COLOR_WHITE,  COLOR_BLUE);
    init_pair(PAIR_HUD,     COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,    COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/*
 * hex_color — pick color pair for hex (Q,R) based on axis membership.
 *
 * Pure helper — keeps its domain name. THE PRIORITY ORDER (first match wins):
 *   1. cursor   — must be first so moving onto an axis hex shows cursor color
 *   2. origin   — (0,0) gets special white display
 *   3. Q=0 axis — cyan diagonal band
 *   4. R=0 axis — green band
 *   5. S=0 axis — yellow band (S = −Q−R, so S=0 means Q+R=0)
 *   6. default  — all other hexes
 */
static int hex_color(int Q, int R, int cQ, int cR)
{
    if (Q == cQ && R == cR)    return PAIR_CURSOR;
    if (Q == 0 && R == 0)      return PAIR_ORIGIN;
    if (Q == 0)                return PAIR_Q_AXIS;
    if (R == 0)                return PAIR_R_AXIS;
    if (-Q - R == 0)           return PAIR_S_AXIS;
    return PAIR_DEFAULT;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the hex ↔ screen mapping (flat-top)           */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — geometry of the active flat-top hex grid (with axis labels).
 *
 * Same shape as 01_flat_top. The only file-specific extra would be label
 * formatting state, but that's stack-local in ctx_draw_labels.
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

    /* cursor bounds in axial space (advisory) */
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
    g->max_q = (int)((double)cols * CELL_W / (3.0 * g->hex_size)) + 1;
    g->max_r = (int)((double)rows * CELL_H / (sqrt(3.0) * g->hex_size)) + 1;
}

/*
 * ctx_to_screen — center cell of flat-top hex (Q, R) in screen coordinates.
 *   cx_pix = size × 3/2 × Q
 *   cy_pix = size × (√3/2 × Q + √3 × R)
 */
static void ctx_to_screen(const GridCtx *g, int Q, int R, int *sr, int *sc)
{
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx_pix = g->hex_size * 1.5    * (double)Q;
    double cy_pix = g->hex_size * (sq3_2 * (double)Q + sq3 * (double)R);
    *sc = g->ox + (int)(cx_pix / g->cell_w);
    *sr = g->oy + (int)(cy_pix / g->cell_h);
}

/* cube_round — see 01_flat_top for full documentation. */
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

__attribute__((unused))
static void ctx_pixel_to_axial(const GridCtx *g, int sr, int sc, int *Q, int *R)
{
    double sq3_3 = sqrt(3.0) / 3.0;
    double px = (double)(sc - g->ox) * g->cell_w;
    double py = (double)(sr - g->oy) * g->cell_h;
    double fq = (2.0/3.0 * px) / g->hex_size;
    double fr = (-1.0/3.0 * px + sq3_3 * py) / g->hex_size;
    double fs = -fq - fr;
    cube_round(fq, fr, fs, Q, R);
}

/* angle_char — same as 01_flat_top. */
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
 * ctx_draw_bg — Pass 1: hex borders colored by axis membership.
 *
 * Same per-pixel pipeline as 01_flat_top ctx_draw_bg. The only difference:
 * instead of PAIR_BORDER/PAIR_CURSOR, the color is chosen by hex_color(),
 * which returns an axis-specific pair.
 *
 * Border cells are visited in raster order. Interior cells are skipped
 * (dist < limit). This means label pass 2 can safely write to interior
 * cells without colliding with border characters.
 *
 * Attribute selection:
 *   PAIR_ORIGIN or PAIR_CURSOR: A_BOLD for prominence
 *   Axis colors (Q/R/S):        no bold — dimmer than axis origin
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

            double fq = (2.0/3.0 * px) / size;
            double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
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

            double cx = size * 1.5 * fQ;
            double cy = size * (sq3_2 * fQ + sq3 * fR);
            double theta = atan2(py - cy, px - cx);
            char ch = angle_char(theta + M_PI / 2.0);

            int pair = hex_color(Q, R, cQ, cR);
            int attr = (pair == PAIR_ORIGIN || pair == PAIR_CURSOR)
                       ? (COLOR_PAIR(pair) | A_BOLD) : COLOR_PAIR(pair);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/*
 * ctx_draw_labels — Pass 2: print "Q,R" at each visible hex center.
 *
 * THE FORMULA (hex enumeration + forward projection via ctx_to_screen):
 *
 *   Iteration bounds (conservative — screens outside are clipped below):
 *     Qmax = cols×CELL_W / (1.5×size) + 3
 *     Rmax = rows×CELL_H / (√3×size)  + 3
 *
 *   For each (Q,R) in [−Qmax..Qmax] × [−Rmax..Rmax]:
 *     ctx_to_screen → (sr, sc)
 *     If on screen: format label, print centered.
 *     Label text: snprintf(buf, sizeof buf, "%d,%d", Q, R)
 *     Centering:  lx = sc - len/2
 *
 * Attribute selection:
 *   cursor hex: A_BOLD | A_REVERSE — very visible (inverted colors)
 *   origin:     A_BOLD
 *   axis hexes: A_DIM  — labels recede; borders dominate visually
 */
static void ctx_draw_labels(const GridCtx *g, int cQ, int cR)
{
    double sq3 = sqrt(3.0);
    int Qmax = (int)(g->cols * g->cell_w / (1.5 * g->hex_size)) + 3;
    int Rmax = (int)(g->rows * g->cell_h / (sq3 * g->hex_size)) + 3;

    for (int Q = -Qmax; Q <= Qmax; Q++) {
        for (int R = -Rmax; R <= Rmax; R++) {
            int sr, sc;
            ctx_to_screen(g, Q, R, &sr, &sc);
            if (sr < 1 || sr >= g->rows - 1) continue;
            if (sc < 2 || sc >= g->cols - 2) continue;

            char buf[12];
            int len = snprintf(buf, sizeof buf, "%d,%d", Q, R);
            int lx  = sc - len / 2;
            if (lx < 0 || lx + len >= g->cols) continue;

            int pair = hex_color(Q, R, cQ, cR);
            int attr = (pair == PAIR_CURSOR) ? (COLOR_PAIR(pair) | A_BOLD | A_REVERSE)
                     : (pair == PAIR_ORIGIN) ? (COLOR_PAIR(pair) | A_BOLD)
                     :                          (COLOR_PAIR(pair) | A_DIM);
            attron(attr);
            mvprintw(sr, lx, "%s", buf);
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
 * HEX_DIR — 4-direction axial movement (same as 01_flat_top §5).
 *
 * Navigating axial labels:
 *   RIGHT (+Q): Q value on label increases by 1, R unchanged.
 *   DOWN  (+R): R value on label increases by 1, Q unchanged.
 * The axis highlighting follows the cursor — moving onto a Q=0 hex while
 * the cursor is there shows PAIR_CURSOR, not PAIR_Q_AXIS.
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
 * cursor_draw — '@' at cursor hex center, drawn last (on top of label).
 *
 * Drawn AFTER ctx_draw_labels so '@' overwrites the label character at
 * the center cell. The label is still partially visible in adjacent cells.
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

static void hud_draw(const GridCtx *g, const Cursor *cur, int paused, double fps)
{
    int S = -cur->q - cur->r;
    char buf[96];
    snprintf(buf, sizeof buf,
             " cursor Q:%+d R:%+d S:%+d  size:%.0f  %5.1f fps  %s ",
             cur->q, cur->r, S, g->hex_size, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  r:reset  arrows:move  +/-:size  [/]:border "
             " cyan=Q  green=R  yellow=S  white=origin ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int paused, double fps)
{
    erase();
    ctx_draw_bg    (g, cur->q, cur->r);   /* Pass 1: borders */
    ctx_draw_labels(g, cur->q, cur->r);   /* Pass 2: labels  */
    cursor_draw    (cur, g);              /* Pass 3: '@'     */
    hud_draw       (g, cur, paused, fps);
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
    color_init();
    int paused = 0;

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

        scene_draw(&g, &cur, paused, fps);
        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }
    return 0;
}
