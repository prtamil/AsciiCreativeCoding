/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 14_origin.c — coordinate-system grid with labelled axes and quadrants
 *
 * DEMO: A rectangular grid with mathematical (x,y) axes drawn at the
 *       screen centre. The X axis goes right (positive) and left (negative);
 *       the Y axis goes UP (positive, inverted from screen) and down (negative).
 *       The player '@' shows its position in mathematical coordinates,
 *       updating as it moves. Quadrant labels (I–IV) are shown.
 *
 * Study alongside: 01_uniform_rect.c (base grid), 04_coarse_sparse.c
 *
 * Section map:
 *   §1 config   — UNIT_W, UNIT_H (one coordinate unit in screen chars)
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 6 pairs: grid, x-axis, y-axis, player, quadrants, HUD
 *   §4 formula  — math↔screen conversion; axis detection
 *   §5 player   — position in math coords (mx, my), screen movement
 *   §6 scene    — draw_grid(), draw_axes(), draw_labels(), draw_player()
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
 *    Note: in C, (sr - oy) can be negative.  Use safe_mod for correct results:
 *      on_h = (safe_mod(sr - oy, UNIT_H) == 0)
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
 *  Quadrant of player position (mx, my):
 *    I   (mx>0, my>0):  right and above origin
 *    II  (mx<0, my>0):  left  and above origin
 *    III (mx<0, my<0):  left  and below origin
 *    IV  (mx>0, my<0):  right and below origin
 *    Axis (mx=0 OR my=0): on an axis, not in any quadrant
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • safe_mod REQUIRED for grid line test: (sr - oy) is negative for
 *    rows ABOVE the origin (sr < oy).  C's % returns negative there.
 *    Without safe_mod, the upper half of the grid has no horizontal lines.
 *
 *  • Origin NOT at screen centre: if COLS or LINES is even, ox or oy
 *    may be off by half a unit.  Integer division gives the closest row/col.
 *    This is correct — just be aware the grid extends asymmetrically.
 *
 *  • UP arrow increases my (math) but DECREASES screen row:
 *    player_move with dmy=+1 moves '@' visually UP on screen.
 *    This is intentional — it matches math convention.  It may feel
 *    inverted if you're used to screen-Y games.
 *
 *  • The axis highlight must override the grid line:
 *    at (sr=oy, sc=anything), it should draw '=' (x-axis), not '-' (grid).
 *    Test axis conditions BEFORE regular grid line conditions.
 *
 *  • Coordinate label axis ticks: to add "2, 4, 6" labels on the x-axis,
 *    draw text at screen col = ox + k*UNIT_W for integer k, at row oy+1
 *    (one row below the x-axis).
 *
 * HOW TO VERIFY
 * ─────────────
 *  At screen (ox, oy): should show 'O' (origin).
 *  At (oy-UNIT_H, ox): should show '|' on y-axis, 1 unit ABOVE origin.
 *    -> my = (oy - (oy-UNIT_H)) / UNIT_H = UNIT_H/UNIT_H = 1. ✓  (positive)
 *  At (oy+UNIT_H, ox): my = (oy - (oy+UNIT_H)) / UNIT_H = -1. ✓  (negative, below)
 *  Player at (mx=+1, my=+1): quadrant I.  Move UP -> my=+2, still quadrant I.
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

#define PAIR_GRID   1   /* regular grid lines (dim)     */
#define PAIR_XAXIS  2   /* X axis (bright horizontal)   */
#define PAIR_YAXIS  3   /* Y axis (bright vertical)     */
#define PAIR_PLAYER 4
#define PAIR_QUAD   5   /* quadrant labels              */
#define PAIR_HUD    6

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
    init_pair(PAIR_PLAYER, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_QUAD,   COLORS >= 256 ?  39 : COLOR_CYAN,   -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  15 : COLOR_WHITE,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * math_to_screen — COORDINATE SYSTEM FORMULA:
 *
 *   screen_col = ox + mx * UNIT_W
 *   screen_row = oy - my * UNIT_H    ← MINUS flips Y axis
 *
 * The minus sign on UNIT_H is the key difference from 01_uniform_rect.
 * Without it, increasing my would go DOWN on screen (screen convention).
 * With it, increasing my goes UP (math convention).
 */
static void math_to_screen(int mx, int my, int ox, int oy, int *sr, int *sc)
{
    *sc = ox + mx * UNIT_W;
    *sr = oy - my * UNIT_H;    /* ← Y flip: -my because screen Y grows downward */
}

/*
 * screen_to_math — INVERSE FORMULA:
 *
 *   mx = (sc - ox) / UNIT_W
 *   my = (oy - sr) / UNIT_H   ← oy - sr to flip Y back
 */
static void screen_to_math(int sr, int sc, int ox, int oy, int *mx, int *my)
{
    *mx = (sc - ox) / UNIT_W;
    *my = (oy - sr) / UNIT_H;
}

/*
 * grid_char_at — what to draw at screen (sr, sc), given origin (ox, oy).
 *
 * Axis detection:
 *   X axis: sr == oy          → draw '=' (bold horizontal)
 *   Y axis: sc == ox          → draw '|' (bold vertical)
 *   Origin: sr==oy && sc==ox  → draw 'O'
 *
 * Grid lines (non-axis):
 *   sr offset from oy is a multiple of UNIT_H → horizontal '-'
 *   sc offset from ox is a multiple of UNIT_W → vertical ':'
 *   Both → '+'
 */
typedef enum { GC_NONE, GC_GRID, GC_XAXIS, GC_YAXIS, GC_ORIGIN } GridCharType;

static GridCharType grid_char_type(int sr, int sc, int ox, int oy, char *out_ch)
{
    bool on_x = (sr == oy);
    bool on_y = (sc == ox);
    if (on_x && on_y) { *out_ch = 'O'; return GC_ORIGIN; }
    if (on_x)         { *out_ch = '='; return GC_XAXIS; }
    if (on_y)         { *out_ch = '|'; return GC_YAXIS; }

    int dr = sr - oy, dc = sc - ox;
    bool gh = (dr % UNIT_H == 0);
    bool gv = (dc % UNIT_W == 0);
    if (gh && gv) { *out_ch = '+'; return GC_GRID; }
    if (gh)       { *out_ch = '-'; return GC_GRID; }
    if (gv)       { *out_ch = ':'; return GC_GRID; }
    *out_ch = ' ';
    return GC_NONE;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  player                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int mx, my;       /* position in MATHEMATICAL coordinates  */
    int range;        /* cells ±range in each direction        */
} Player;

static void player_reset(Player *p, int rows, int cols)
{
    p->range = (rows < cols) ? rows / (2 * UNIT_H) - 1 : cols / (2 * UNIT_W) - 1;
    p->mx = 0; p->my = 0;
}

/*
 * player_move — delta in math space.
 *   RIGHT → mx += 1   (move right on X axis)
 *   LEFT  → mx -= 1
 *   UP    → my += 1   (move UP in math = screen row decreases)
 *   DOWN  → my -= 1
 *
 * Note: UP key increases my (math), which DECREASES screen_row.
 * This is the correct mathematical convention.
 */
static void player_move(Player *p, int dmx, int dmy)
{
    int nx = p->mx + dmx, ny = p->my + dmy;
    if (nx >= -p->range && nx <= p->range) p->mx = nx;
    if (ny >= -p->range && ny <= p->range) p->my = ny;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void draw_grid(int rows, int cols, int ox, int oy)
{
    for (int sr = 0; sr < rows - 1; sr++) {
        for (int sc = 0; sc < cols; sc++) {
            char ch; GridCharType t = grid_char_type(sr, sc, ox, oy, &ch);
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
 * draw_quadrant_labels — place I/II/III/IV in the middle of each quadrant.
 * Positions are derived from math_to_screen with fractional unit offsets.
 */
static void draw_quadrant_labels(int ox, int oy)
{
    int r, c;
    attron(COLOR_PAIR(PAIR_QUAD) | A_DIM);
    /* Quadrant I: +x, +y → right of Y axis, above X axis */
    math_to_screen(2, 2, ox, oy, &r, &c); mvprintw(r, c, "I");
    /* Quadrant II: -x, +y */
    math_to_screen(-3, 2, ox, oy, &r, &c); mvprintw(r, c, "II");
    /* Quadrant III: -x, -y */
    math_to_screen(-3, -2, ox, oy, &r, &c); mvprintw(r, c, "III");
    /* Quadrant IV: +x, -y */
    math_to_screen(2, -2, ox, oy, &r, &c); mvprintw(r, c, "IV");
    attroff(COLOR_PAIR(PAIR_QUAD) | A_DIM);
}

static void draw_player(const Player *p, int ox, int oy)
{
    int sr, sc; math_to_screen(p->mx, p->my, ox, oy, &sr, &sc);
    /* Verify inverse: screen_to_math should return (mx, my) unchanged */
    int vx, vy; screen_to_math(sr, sc, ox, oy, &vx, &vy);
    (void)vx; (void)vy;  /* available for debugging: assert vx==p->mx, vy==p->my */
    attron(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
    mvaddch(sr, sc, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
}

static const char *quadrant_name(int mx, int my)
{
    if (mx == 0 || my == 0) return "axis";
    if (mx > 0 && my > 0) return "I";
    if (mx < 0 && my > 0) return "II";
    if (mx < 0 && my < 0) return "III";
    return "IV";
}

static void scene_draw(int rows, int cols, const Player *p, double fps)
{
    int ox = cols / 2, oy = (rows - 1) / 2;
    erase();
    draw_grid(rows, cols, ox, oy);
    draw_quadrant_labels(ox, oy);
    draw_player(p, ox, oy);

    /* HUD: show math coordinates and quadrant */
    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  math(%+d,%+d)  Q%s ",
        fps, p->mx, p->my, quadrant_name(p->mx, p->my));
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD) | A_DIM);
    mvprintw(rows - 1, 0,
        " arrows:move  r:reset  q:quit  [14 origin  screen_row=oy-my*UNIT_H] ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_DIM);

    wnoutrefresh(stdscr); doupdate();
}

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
    int rows = LINES, cols = COLS;
    Player player; player_reset(&player, rows, cols);
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double fps = TARGET_FPS; int64_t t0 = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            rows = LINES; cols = COLS;
            player_reset(&player, rows, cols);
        }
        int ch = getch();
        switch (ch) {
            case 'q': case 27: g_running = 0;                    break;
            case 'r':          player_reset(&player, rows, cols); break;
            /* UP   → my+1 (math Y up)    DOWN → my-1 */
            /* RIGHT → mx+1               LEFT → mx-1  */
            case KEY_UP:    player_move(&player,  0, +1); break;
            case KEY_DOWN:  player_move(&player,  0, -1); break;
            case KEY_LEFT:  player_move(&player, -1,  0); break;
            case KEY_RIGHT: player_move(&player, +1,  0); break;
        }
        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (now - t0 + 1)) * 0.05; t0 = now;
        scene_draw(rows, cols, &player, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
