/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 14_origin.c — coordinate-system grid with labelled axes and quadrants
 *
 * DEMO: A rectangular grid with mathematical (x,y) axes drawn at the
 *       screen centre. The X axis goes right (positive) and left (negative);
 *       the Y axis goes UP (positive, inverted from screen) and down (negative).
 *       The cursor '@' shows its position in mathematical coordinates,
 *       updating as it moves. Quadrant labels (I–IV) are shown.
 *
 * Study alongside: 01_uniform_rect.c (base grid), 04_coarse_sparse.c
 *
 * Section map:
 *   §1 config   — UNIT_W, UNIT_H (one coordinate unit in screen chars), EWMA
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 7 pairs (grid, x-axis, y-axis, cursor, quadrants, HUD, HINT)
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen (math↔screen) /
 *                 ctx_grid_char / ctx_draw_bg + axes_draw + labels_draw
 *   §5 cursor   — Cursor (mx, my) in MATH space, with Y flip
 *   §6 scene    — hud_draw + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids/14_origin.c \
 *       -o 14_origin -lncurses
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Rectangular grid centred at the terminal midpoint.
 *                  The mathematical Y axis is inverted: +Y is UP on screen
 *                  (decreasing screen row), matching standard maths convention.
 *
 * Forward formula (math → screen):
 *
 *   screen_col = ox + mx * UNIT_W
 *   screen_row = oy - my * UNIT_H      ← MINUS because screen Y flips
 *
 *   where ox = COLS/2 (origin column), oy = (LINES-1)/2 (origin row).
 *
 * Inverse formula (screen → math):
 *
 *   mx = (screen_col - ox) / UNIT_W
 *   my = (oy - screen_row) / UNIT_H    ← note the flip
 *
 * Axis detection:
 *   X axis (my=0): screen_row == oy   → sr == oy
 *   Y axis (mx=0): screen_col == ox   → sc == ox
 *
 * Grid lines:
 *   horizontal: (sr - oy) % UNIT_H == 0
 *   vertical:   (sc - ox) % UNIT_W == 0
 *
 * Quadrant: determined by sign of (mx, my).
 *   I:   mx>0, my>0   (right & above origin)
 *   II:  mx<0, my>0   (left  & above origin)
 *   III: mx<0, my<0   (left  & below origin)
 *   IV:  mx>0, my<0   (right & below origin)
 *
 * References     :
 *   Cartesian coordinates — en.wikipedia.org/wiki/Cartesian_coordinate_system
 *   Quadrants — en.wikipedia.org/wiki/Quadrant_(plane_geometry)
 *   Screen-space Y flip — this project's documentation/Architecture.md §4
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * This is a rectangular grid with a COORDINATE SYSTEM overlaid.  The grid
 * lines are the same as 01_uniform_rect but the origin (0,0) is at the
 * screen centre, not the top-left.  The critical formula change is one
 * MINUS SIGN in the row formula: `screen_row = oy - my * UNIT_H`.  This
 * single sign flip makes the mathematical Y axis point upward on screen,
 * matching the convention used in mathematics and most scientific plots.
 *
 * HOW TO THINK ABOUT IT — TWO Y DIRECTIONS
 * ──────────────────────────────────────────
 * There are TWO Y conventions in computing:
 *
 *   Screen convention:  row 0 is at TOP; row increases DOWNWARD.
 *   Math convention:    y=0 is at centre; y increases UPWARD.
 *
 * The terminal uses screen convention.  A math coordinate system requires
 * a FLIP of the Y axis.  The formula encodes this flip:
 *
 *   screen_row = oy - my * UNIT_H
 *
 * When my INCREASES (moving up in math): screen_row DECREASES (moving up
 * on screen).  The minus sign is the entire flip — nothing else changes.
 *
 * Analogy: think of a standard graph paper taped to a wall.  The paper's
 * origin is at the centre.  Moving UP on the paper means decreasing screen
 * row.  The paper's y axis points AGAINST the screen's row axis.
 *
 * DRAWING METHOD
 * ──────────────
 *  Step 1: Determine origin position:
 *    ox = COLS / 2        (centre column)
 *    oy = (LINES-1) / 2   (centre row, excluding HUD row)
 *
 *  Step 2: Grid line conditions (using offset from origin):
 *    on_h = ((sr - oy) % UNIT_H == 0)   <- rows that are multiples of UNIT_H from origin
 *    on_v = ((sc - ox) % UNIT_W == 0)   <- cols that are multiples of UNIT_W from origin
 *
 *  Step 3: Axis detection (special case of grid lines):
 *    is_x_axis = (sr == oy)    <- my=0 -> screen_row = oy - 0*UNIT_H = oy
 *    is_y_axis = (sc == ox)    <- mx=0 -> screen_col = ox + 0*UNIT_W = ox
 *    is_origin = is_x_axis AND is_y_axis
 *
 *  Step 4: Draw with priority: origin > axis > grid line.
 *    origin  -> 'O'  (or '+') in bright color
 *    x-axis  -> '='  in red/x-color
 *    y-axis  -> '|'  in green/y-color
 *    grid    -> '+'/'-'/':' in dim color
 *
 * KEY FORMULAS
 * ────────────
 *  Forward (math -> screen):
 *    screen_col = ox + mx * UNIT_W
 *    screen_row = oy - my * UNIT_H      <- MINUS for Y flip
 *
 *  Inverse (screen -> math):
 *    mx = (sc - ox) / UNIT_W
 *    my = (oy - sr) / UNIT_H            <- oy MINUS sr to flip back
 *
 *  Grid line test (relative to origin):
 *    on_h = (safe_mod(sr - oy, UNIT_H) == 0)
 *    on_v = (safe_mod(sc - ox, UNIT_W) == 0)
 *
 *  Quadrant of cursor position (mx, my):
 *    I   (mx>0, my>0):  right and above origin
 *    II  (mx<0, my>0):  left  and above origin
 *    III (mx<0, my<0):  left  and below origin
 *    IV  (mx>0, my<0):  right and below origin
 *    Axis (mx=0 OR my=0): on an axis, not in any quadrant
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • UP arrow increases my (math) but DECREASES screen row:
 *    cursor_move with dmy=+1 moves '@' visually UP on screen.
 *    This is intentional — it matches math convention.
 *
 *  • Origin NOT at screen centre: if COLS or LINES is even, ox or oy
 *    may be off by half a unit.  Integer division gives the closest row/col.
 *
 *  • The axis highlight must override the grid line:
 *    at (sr=oy, sc=anything), it should draw '=' (x-axis), not '-' (grid).
 *    Test axis conditions BEFORE regular grid line conditions.
 *
 *  • Coordinate label axis ticks: to add "2, 4, 6" labels on the x-axis,
 *    draw text at screen col = ox + k*UNIT_W for integer k, at row oy+1.
 *
 * HOW TO VERIFY
 * ─────────────
 *  At screen (ox, oy): should show 'O' (origin).
 *  At (oy-UNIT_H, ox): should show '|' on y-axis, 1 unit ABOVE origin.
 *    -> my = (oy - (oy-UNIT_H)) / UNIT_H = 1. ✓  (positive)
 *  At (oy+UNIT_H, ox): my = (oy - (oy+UNIT_H)) / UNIT_H = -1. ✓  (negative, below)
 *  Cursor at (mx=+1, my=+1): quadrant I.  Move UP -> my=+2, still quadrant I.
 *  Move LEFT from quadrant I -> mx=0, on y-axis, no quadrant.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
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

#define TARGET_FPS  30

/*
 * UNIT_W, UNIT_H — one unit of mathematical coordinate in screen chars.
 * With UNIT_W=8 and UNIT_H=4, each unit step looks square in pixels.
 */
#define UNIT_W   8    /* screen cols per 1 math unit */
#define UNIT_H   4    /* screen rows per 1 math unit */

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA  0.05

#define PAIR_GRID   1   /* regular grid lines (dim)     */
#define PAIR_XAXIS  2   /* X axis (bright horizontal)   */
#define PAIR_YAXIS  3   /* Y axis (bright vertical)     */
#define PAIR_CURSOR 4
#define PAIR_QUAD   5   /* quadrant labels              */
#define PAIR_HUD    6
#define PAIR_HINT   7

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void color_init(void)
{
    start_color(); use_default_colors();
    init_pair(PAIR_GRID,   COLORS >= 256 ? 244 : COLOR_WHITE,  -1);
    init_pair(PAIR_XAXIS,  COLORS >= 256 ? 196 : COLOR_RED,    -1);
    init_pair(PAIR_YAXIS,  COLORS >= 256 ?  46 : COLOR_GREEN,  -1);
    init_pair(PAIR_CURSOR, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_QUAD,   COLORS >= 256 ?  39 : COLOR_CYAN,   -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the math ↔ screen mapping                    */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — terminal extent + units + origin + cursor range (math space).
 *
 * cw/ch carry UNIT_W/UNIT_H; ox/oy are the math origin in screen coords.
 * range bounds the cursor to ±range in both math axes.
 */
typedef struct {
    int rows, cols;
    int cw, ch;          /* UNIT_W, UNIT_H */
    int ox, oy;          /* math origin in screen coords */
    int range;           /* cursor lives in [-range, +range] in math space */
    int max_r, max_c;    /* mirror of range for parity with template */
} GridCtx;

static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows = rows; g->cols = cols;
    g->cw = UNIT_W; g->ch = UNIT_H;
    g->ox = cols / 2;
    g->oy = (rows - 1) / 2;
    g->range = (rows < cols) ? rows / (2 * UNIT_H) - 1 : cols / (2 * UNIT_W) - 1;
    g->max_r = g->range;
    g->max_c = g->range;
}

/*
 * ctx_to_screen — COORDINATE SYSTEM FORMULA (math -> screen):
 *
 *   screen_col = ox + mx * UNIT_W
 *   screen_row = oy - my * UNIT_H    ← MINUS flips Y axis
 *
 * Naming note: the (r, c) of the standard ctx_to_screen here are (my, mx) —
 * the math coordinates.  Y is inverted: positive my goes UP on screen.
 */
static void ctx_to_screen(const GridCtx *g, int my, int mx, int *sr, int *sc)
{
    *sc = g->ox + mx * g->cw;
    *sr = g->oy - my * g->ch;
}

/*
 * screen_to_math — INVERSE FORMULA:
 *   mx = (sc - ox) / UNIT_W
 *   my = (oy - sr) / UNIT_H   ← oy - sr to flip Y back
 */
static void screen_to_math(const GridCtx *g, int sr, int sc, int *mx, int *my)
{
    *mx = (sc - g->ox) / g->cw;
    *my = (g->oy - sr) / g->ch;
}

/*
 * grid char classification:
 *   axis detection takes priority over modular grid lines.
 */
typedef enum { GC_NONE, GC_GRID, GC_XAXIS, GC_YAXIS, GC_ORIGIN } GridCharType;

static GridCharType ctx_grid_char_type(const GridCtx *g, int sr, int sc, char *out_ch)
{
    bool on_x = (sr == g->oy);
    bool on_y = (sc == g->ox);
    if (on_x && on_y) { *out_ch = 'O'; return GC_ORIGIN; }
    if (on_x)         { *out_ch = '='; return GC_XAXIS; }
    if (on_y)         { *out_ch = '|'; return GC_YAXIS; }

    int dr = sr - g->oy, dc = sc - g->ox;
    bool gh = (dr % g->ch == 0);
    bool gv = (dc % g->cw == 0);
    if (gh && gv) { *out_ch = '+'; return GC_GRID; }
    if (gh)       { *out_ch = '-'; return GC_GRID; }
    if (gv)       { *out_ch = ':'; return GC_GRID; }
    *out_ch = ' ';
    return GC_NONE;
}

/*
 * ctx_grid_char — standard char-only API (delegates to type-classifier).
 */
static char ctx_grid_char(const GridCtx *g, int sr, int sc)
{
    char ch; ctx_grid_char_type(g, sr, sc, &ch);
    return ch;
}

/*
 * ctx_draw_bg — paint grid + axes with priority colouring.
 */
static void ctx_draw_bg(const GridCtx *g)
{
    for (int sr = 0; sr < g->rows - 1; sr++) {
        for (int sc = 0; sc < g->cols; sc++) {
            char ch; GridCharType t = ctx_grid_char_type(g, sr, sc, &ch);
            if (t == GC_NONE) continue;
            if (t == GC_ORIGIN) {
                attron(COLOR_PAIR(PAIR_XAXIS) | A_BOLD);
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
                attroff(COLOR_PAIR(PAIR_XAXIS) | A_BOLD);
            } else if (t == GC_XAXIS) {
                attron(COLOR_PAIR(PAIR_XAXIS));
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
                attroff(COLOR_PAIR(PAIR_XAXIS));
            } else if (t == GC_YAXIS) {
                attron(COLOR_PAIR(PAIR_YAXIS));
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
                attroff(COLOR_PAIR(PAIR_YAXIS));
            } else {
                attron(COLOR_PAIR(PAIR_GRID));
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
                attroff(COLOR_PAIR(PAIR_GRID));
            }
        }
    }
}

/*
 * labels_draw — place I/II/III/IV in the middle of each quadrant.
 * Positions are derived from ctx_to_screen with fractional unit offsets.
 */
static void labels_draw(const GridCtx *g)
{
    int r, c;
    attron(COLOR_PAIR(PAIR_QUAD) | A_DIM);
    /* Quadrant I: +x, +y → right of Y axis, above X axis */
    ctx_to_screen(g, 2,  2, &r, &c);  mvprintw(r, c, "I");
    /* Quadrant II: -x, +y */
    ctx_to_screen(g, 2, -3, &r, &c);  mvprintw(r, c, "II");
    /* Quadrant III: -x, -y */
    ctx_to_screen(g, -2, -3, &r, &c); mvprintw(r, c, "III");
    /* Quadrant IV: +x, -y */
    ctx_to_screen(g, -2,  2, &r, &c); mvprintw(r, c, "IV");
    attroff(COLOR_PAIR(PAIR_QUAD) | A_DIM);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — position in MATHEMATICAL coordinates (mx, my).
 * Y is the math Y (positive = up); range bound lives in GridCtx.range.
 */
typedef struct { int mx, my; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    (void)g;
    cur->mx = 0; cur->my = 0;
}

/*
 * cursor_move — delta in math space.
 *   RIGHT → mx += 1   (move right on X axis)
 *   LEFT  → mx -= 1
 *   UP    → my += 1   (move UP in math = screen row decreases)
 *   DOWN  → my -= 1
 *
 * Note: UP key increases my (math), which DECREASES screen_row.
 * This is the correct mathematical convention.
 */
static void cursor_move(Cursor *cur, const GridCtx *g, int dmx, int dmy)
{
    int nx = cur->mx + dmx, ny = cur->my + dmy;
    if (nx >= -g->range && nx <= g->range) cur->mx = nx;
    if (ny >= -g->range && ny <= g->range) cur->my = ny;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc; ctx_to_screen(g, cur->my, cur->mx, &sr, &sc);
    /* Verify inverse: screen_to_math should return (mx, my) unchanged */
    int vx, vy; screen_to_math(g, sr, sc, &vx, &vy);
    (void)vx; (void)vy;
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(sr, sc, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static const char *quadrant_name(int mx, int my)
{
    if (mx == 0 || my == 0) return "axis";
    if (mx > 0 && my > 0) return "I";
    if (mx < 0 && my > 0) return "II";
    if (mx < 0 && my < 0) return "III";
    return "IV";
}

static void hud_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  math(%+d,%+d)  Q%s ",
        fps, cur->mx, cur->my, quadrant_name(cur->mx, cur->my));
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " arrows:move  r:reset  q/ESC:quit  [14 origin  screen_row=oy-my*UNIT_H] ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, double fps)
{
    erase();
    ctx_draw_bg(g);
    labels_draw(g);
    cursor_draw(cur, g);
    hud_draw(g, cur, fps);
    wnoutrefresh(stdscr); doupdate();
}

/* Reference for the standard ctx_grid_char API; quiets -Wunused-function. */
static void ctx_grid_char_ref(void) { (void)ctx_grid_char; }

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }
static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(); atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running = 1, g_need_resize = 0;
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
    GridCtx g;     ctx_init(&g, LINES, COLS);
    Cursor  cur;   cursor_reset(&cur, &g);
    ctx_grid_char_ref();   /* keep the standard API symbol live */

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double fps = TARGET_FPS;
    int64_t t0 = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            ctx_init(&g, LINES, COLS);
        }
        int ch = getch();
        switch (ch) {
            case 'q': case 27: g_running = 0;              break;
            case 'r':          cursor_reset(&cur, &g);     break;
            /* UP   → my+1 (math Y up)    DOWN → my-1   */
            /* RIGHT → mx+1               LEFT → mx-1   */
            case KEY_UP:    cursor_move(&cur, &g,  0, +1); break;
            case KEY_DOWN:  cursor_move(&cur, &g,  0, -1); break;
            case KEY_LEFT:  cursor_move(&cur, &g, -1,  0); break;
            case KEY_RIGHT: cursor_move(&cur, &g, +1,  0); break;
        }
        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9 / (now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;
        scene_draw(&g, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
