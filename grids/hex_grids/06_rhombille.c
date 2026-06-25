/*
 * 06_rhombille.c — rhombille tiling: each hexagon split into 3 diamonds
 *
 * We draw a flat-top hex grid, then inside every hex draw three short lines
 * from the center out to every other corner. That splits each hex into three
 * diamonds, and the diamonds from neighboring hexes line up to look like a
 * stack of 3D cubes seen from the corner — the classic "cube illusion".
 * An '@' marker moves between hexes with the arrow keys.
 *
 * Study alongside: ../README.md (the GridCtx + Cursor split used everywhere),
 *                  hex_grids/01_flat_top.c (the hex-border drawing this builds on),
 *                  hex_grids/05_triangular.c (the triangular cousin of this tiling)
 *
 * Keys:  q/ESC quit  p pause  t theme  r reset  arrows move  +/-:size  [/]:spoke
 *
 * References:
 *   Rhombille tiling     https://en.wikipedia.org/wiki/Rhombille_tiling
 *   Isometric projection https://en.wikipedia.org/wiki/Isometric_projection
 *   Red Blob hex guide   https://www.redblobgames.com/grids/hexagons/
 */

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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config ── */

/* One terminal cell is this many "pixels" wide and tall. All the hex math
 * works in these finer pixels so the shapes don't look chunky. */
#define CELL_W              2
#define CELL_H              4

/* How big each hex is, in pixels (its side length). This is the main dial
 * for how zoomed-in the grid looks. */
#define HEX_SIZE_DEFAULT   14.0
#define HEX_SIZE_MIN        6.0
#define HEX_SIZE_MAX       40.0
#define HEX_SIZE_STEP       2.0

/* How thick the hex outlines are. Bigger = fatter borders, smaller interiors. */
#define BORDER_W_DEFAULT    0.10
#define BORDER_W_MIN        0.03
#define BORDER_W_MAX        0.30
#define BORDER_W_STEP       0.02

/* Half the thickness of the inner spoke lines, in pixels. */
#define SPOKE_W_DEFAULT     2.5
#define SPOKE_W_MIN         0.5
#define SPOKE_W_MAX         6.0
#define SPOKE_W_STEP        0.5

#define N_THEMES            4
#define TICK_NS            16666667LL    /* one frame at ~60 fps */

/* How fast the on-screen fps number reacts. Small = smooth and slow. */
#define FPS_EWMA_ALPHA      0.05

/* Color pair IDs */
#define PAIR_BORDER  1
#define PAIR_SPOKE   2   /* the diamond lines inside each hex */
#define PAIR_CURSOR  3   /* the highlighted hex and its '@' */
#define PAIR_HUD     4
#define PAIR_HINT    5

/* ── §2 clock ── */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec ts = { .tv_sec  = (time_t)(ns / 1000000000LL),
                           .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&ts, NULL);
}

/* ── §3 color ── */

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
    init_pair(PAIR_SPOKE,  THEMES[theme][0], THEMES[theme][1]);
    init_pair(PAIR_CURSOR, COLOR_WHITE,      COLOR_BLUE);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 formula ── */

/*
 * GridCtx — everything we need to know to draw the grid and find where a hex
 * lands on screen. One of these is filled in once at startup (and again on
 * resize) and then passed, read-only, to every drawing routine.
 *
 * Hex addresses use (q, r) "axial" coordinates — think of them as a slanted
 * row/column system for hexagons, the same one 01_flat_top uses.
 *
 * The two look-tuning knobs (border_w, spoke_w_px) live here, not in a global,
 * so a drawing call always uses the settings handed to it.
 */
typedef struct {
    /* size of the terminal, in cells */
    int rows, cols;

    /* hex side length in pixels, plus how many pixels make one cell */
    double hex_size;
    int    cell_w, cell_h;

    /* where (0,0) sits on screen, so the grid is centered */
    int ox, oy;

    /* how far the cursor may roam in each axis (generous on purpose) */
    int max_q, max_r;

    /* look-tuning knobs read while drawing */
    double border_w;       /* how thick the hex outlines are */
    double spoke_w_px;     /* half-thickness of the inner spoke lines, in pixels */
} GridCtx;

/* Fills in a GridCtx from the current terminal size and chosen settings.
 * The roam limits are deliberately generous — they just stop the cursor from
 * wandering miles off screen; anything off the edge is hidden by cursor_draw. */
static void ctx_init(GridCtx *g, int rows, int cols,
                     double hex_size, double border_w, double spoke_w_px)
{
    g->rows = rows; g->cols = cols;
    g->hex_size = hex_size;
    g->cell_w = CELL_W; g->cell_h = CELL_H;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    g->border_w   = border_w;
    g->spoke_w_px = spoke_w_px;

    /* roughly how many hexes fit across and down, with a little slack */
    g->max_q = (int)((cols * CELL_W) / (1.5 * hex_size)) + 2;
    g->max_r = (int)((rows * CELL_H) / (sqrt(3.0) * hex_size)) + 2;
}

/* Given a hex address (q, r), works out which screen cell its center lands on.
 * Stepping q moves you right; stepping r moves you down-and-right. Same math
 * every hex file in this series uses; only the reverse lookup differs. */
static void ctx_to_screen(const GridCtx *g, int q, int r, int *sr, int *sc)
{
    double sq3   = sqrt(3.0);
    double sq3_2 = sq3 * 0.5;
    double cx_pix = g->hex_size * 1.5      * (double)q;
    double cy_pix = g->hex_size * (sq3_2   * (double)q + sq3 * (double)r);
    *sc = g->ox + (int)(cx_pix / g->cell_w);
    *sr = g->oy + (int)(cy_pix / g->cell_h);
}

/* Picks the ASCII character that best matches a line at the given angle, so a
 * curve drawn cell-by-cell reads as a smooth edge. (Same as 01_flat_top.) */
static char angle_char(double theta)
{
    double t = fmod(theta, M_PI);
    if (t < 0.0) t += M_PI;
    if      (t < M_PI / 8.0)         return '-';
    else if (t < 3.0 * M_PI / 8.0)  return '\\';
    else if (t < 5.0 * M_PI / 8.0)  return '|';
    else if (t < 7.0 * M_PI / 8.0)  return '/';
    else                              return '-';
}

/*
 * The three "spokes" we draw inside every hex: lines from the center out to
 * every other corner, 120° apart. SPOKE_D holds which way each one points on
 * screen (right, up-left, down-left); SPOKE_C is the character that looks like
 * a line going that way. Splitting the hex this way is what makes the diamonds.
 * (Y points down on screen, which is why these aren't the textbook angles.)
 */
static const double SPOKE_D[3][2] = {
    {  1.0,        0.0       },  /* points right      → '-'  */
    { -0.5,       -0.8660254 },  /* points up-left    → '\\' */
    { -0.5,        0.8660254 },  /* points down-left  → '/'  */
};
static const char SPOKE_C[3] = { '-', '\\', '/' };

/*
 * Draws the whole grid, one terminal cell at a time. For each cell we figure
 * out which hex it falls in and how far it is from that hex's center. If it's
 * near the edge, we draw an outline character. If it's inside, we check the
 * three spokes and draw one if the cell sits on it — that's what carves each
 * hex into diamonds. The cursor's hex gets the highlight color.
 */
static void ctx_draw_bg(const GridCtx *g, int cur_q, int cur_r)
{
    double sq3   = sqrt(3.0);
    double sq3_3 = sq3 / 3.0;
    double sq3_2 = sq3 * 0.5;
    double limit = 0.5 - g->border_w;
    double size  = g->hex_size;

    for (int row = 0; row < g->rows - 1; row++) {
        for (int col = 0; col < g->cols; col++) {
            double px = (double)(col - g->ox) * g->cell_w;
            double py = (double)(row - g->oy) * g->cell_h;

            /* Which hex is this cell in? Convert to hex coordinates... */
            double fq = (2.0/3.0 * px) / size;
            double fr = (-1.0/3.0 * px + sq3_3 * py) / size;
            double fs = -fq - fr;

            /* ...then snap to the nearest whole hex. */
            int rq = (int)round(fq), rr = (int)round(fr), rs = (int)round(fs);
            double dq = fabs((double)rq - fq);
            double dr = fabs((double)rr - fr);
            double ds = fabs((double)rs - fs);
            if      (dq > dr && dq > ds) rq = -rr - rs;
            else if (dr > ds)             rr = -rq - rs;
            int Q = rq, R = rr;

            /* How far (0..0.5) this cell is from the hex center; near 0.5 means
             * it's out at the rim, so it's part of the outline. */
            double fQ = (double)Q, fR = (double)R, fS = (double)(-Q - R);
            double dist = fabs(fq - fQ);
            double d2   = fabs(fr - fR);
            double d3   = fabs(fs - fS);
            if (d2 > dist) dist = d2;
            if (d3 > dist) dist = d3;

            /* where this hex's center sits in pixels */
            double cx = size * 1.5 * fQ;
            double cy = size * (sq3_2 * fQ + sq3 * fR);

            int on_cur = (Q == cur_q && R == cur_r);

            if (dist >= limit) {
                /* Out at the rim: draw an outline character. */
                double theta = atan2(py - cy, px - cx);
                char ch = angle_char(theta + M_PI / 2.0);
                int attr = on_cur ? (COLOR_PAIR(PAIR_CURSOR) | A_BOLD)
                                  : (COLOR_PAIR(PAIR_BORDER) | A_BOLD);
                attron(attr);
                mvaddch(row, col, (chtype)(unsigned char)ch);
                attroff(attr);
            } else {
                /* Inside the hex: see if this cell lands on one of the three
                 * spokes. t is how far along the spoke we are (must be between
                 * the center and the corner); perp is how far off to the side. */
                double vx = px - cx, vy = py - cy;
                for (int k = 0; k < 3; k++) {
                    double dx = SPOKE_D[k][0], dy = SPOKE_D[k][1];
                    double t    = vx * dx + vy * dy;
                    double perp = fabs(vx * dy - vy * dx);
                    if (t >= 0.0 && t <= size && perp <= g->spoke_w_px) {
                        int attr = on_cur ? COLOR_PAIR(PAIR_CURSOR)
                                          : COLOR_PAIR(PAIR_SPOKE);
                        attron(attr);
                        mvaddch(row, col, (chtype)(unsigned char)SPOKE_C[k]);
                        attroff(attr);
                        break;
                    }
                }
            }
        }
    }
}

/* ── §5 cursor ── */

/*
 * Cursor — which hex the user is pointing at, as a (q, r) address. That's all
 * it holds. It doesn't know the grid's size (that's the GridCtx's job) or
 * where it lands on screen (ctx_to_screen works that out). Keeping it this
 * tiny is why the same Cursor type is reused across every grid in the series.
 */
typedef struct { int q, r; } Cursor;

/*
 * The four arrow-key moves, written as a change in hex address. Up/down nudge
 * r, left/right nudge q. Moving the cursor just shifts which hex is
 * highlighted — the diamond pattern itself looks the same in every hex.
 *
 *                 UP: (q=0, r=-1)
 *   LEFT: (q=-1, r=0)  *  RIGHT: (q=+1, r=0)
 *                 DOWN: (q=0, r=+1)
 */
static const int HEX_DIR[4][2] = {
    { 0, -1 },   /* UP    */
    { 0, +1 },   /* DOWN  */
    {-1,  0 },   /* LEFT  */
    {+1,  0 },   /* RIGHT */
};

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->q = 0;
    cur->r = 0;
}

/* Nudges the cursor by one step, refusing to step past the roam limits. */
static void cursor_move(Cursor *cur, const GridCtx *g, int dq, int dr)
{
    int nq = cur->q + dq;
    int nr = cur->r + dr;
    if (nq >= -g->max_q && nq <= g->max_q) cur->q = nq;
    if (nr >= -g->max_r && nr <= g->max_r) cur->r = nr;
}

/* Draws the '@' marker at the cursor's hex, skipping it if that hex is
 * currently off screen. */
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

/* ── §6 scene ── */

/* Draws the on-screen readouts: the status line top-right, the key list along
 * the bottom. Bright and bold so it stays readable over the busy grid. */
static void hud_draw(const GridCtx *g, const Cursor *cur,
                     int theme, int paused, double fps)
{
    char buf[96];
    snprintf(buf, sizeof buf,
             " Q:%+d R:%+d  size:%.0f  spoke:%.1f  theme:%d  %5.1f fps  %s ",
             cur->q, cur->r, g->hex_size, g->spoke_w_px, theme, fps,
             paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  arrows:move  +/-:size  [/]:spoke ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur,
                       int theme, int paused, double fps)
{
    erase();
    ctx_draw_bg(g, cur->q, cur->r);
    cursor_draw(cur, g);
    hud_draw(g, cur, theme, paused, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §7 screen ── */

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

/* ── §8 app ── */

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

    double hex_size = HEX_SIZE_DEFAULT;
    double border_w = BORDER_W_DEFAULT;
    double spoke_w  = SPOKE_W_DEFAULT;
    int    theme    = 0;
    int    paused   = 0;

    GridCtx g;     ctx_init(&g, LINES, COLS, hex_size, border_w, spoke_w);
    Cursor  cur;   cursor_reset(&cur, &g);
    color_init(theme);

    double  fps = 60.0;
    int64_t prev = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            ctx_init(&g, LINES, COLS, hex_size, border_w, spoke_w);
            cursor_reset(&cur, &g);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_running = 0; break;
            case 'p': paused ^= 1; break;
            case 't':
                theme = (theme + 1) % N_THEMES;
                color_init(theme);
                break;
            case 'r': cursor_reset(&cur, &g); break;
            case KEY_UP:    cursor_move(&cur, &g, HEX_DIR[0][0], HEX_DIR[0][1]); break;
            case KEY_DOWN:  cursor_move(&cur, &g, HEX_DIR[1][0], HEX_DIR[1][1]); break;
            case KEY_LEFT:  cursor_move(&cur, &g, HEX_DIR[2][0], HEX_DIR[2][1]); break;
            case KEY_RIGHT: cursor_move(&cur, &g, HEX_DIR[3][0], HEX_DIR[3][1]); break;
            case '+': case '=':
                if (hex_size < HEX_SIZE_MAX) {
                    hex_size += HEX_SIZE_STEP;
                    ctx_init(&g, LINES, COLS, hex_size, border_w, spoke_w);
                }
                break;
            case '-':
                if (hex_size > HEX_SIZE_MIN) {
                    hex_size -= HEX_SIZE_STEP;
                    ctx_init(&g, LINES, COLS, hex_size, border_w, spoke_w);
                }
                break;
            case '[':
                if (spoke_w > SPOKE_W_MIN) {
                    spoke_w -= SPOKE_W_STEP;
                    g.spoke_w_px = spoke_w;
                }
                break;
            case ']':
                if (spoke_w < SPOKE_W_MAX) {
                    spoke_w += SPOKE_W_STEP;
                    g.spoke_w_px = spoke_w;
                }
                break;
            }
        }

        int64_t now = clock_ns(), dt = now - prev; prev = now;
        if (dt > 0) {
            fps = fps * (1.0 - FPS_EWMA_ALPHA)
                + (1e9 / (double)dt) * FPS_EWMA_ALPHA;
        }

        scene_draw(&g, &cur, theme, paused, fps);
        clock_sleep_ns(TICK_NS - (clock_ns() - now));
    }
    return 0;
}
