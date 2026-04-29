/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_square.c — rectangular grid with visually square cells
 *
 * DEMO: Same formula as 01_uniform_rect.c but CELL_W is derived from
 *       CELL_H using the terminal's character aspect ratio (~2:1 h:w).
 *       The HUD shows both character dimensions and pixel estimates so
 *       you can see why CELL_W must be 2× CELL_H for square pixels.
 *
 * Study alongside: 01_uniform_rect.c (base formula), 03_fine_dense.c
 *
 * Section map:
 *   §1 config   — CELL_H, ASPECT_RATIO, CELL_W (derived)
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 4 pairs
 *   §4 formula  — same cell_to_screen as 01; aspect derivation explained
 *   §5 player   — struct, move, reset
 *   §6 scene    — draw_grid(), draw_player(), HUD with dimension info
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids/02_square.c \
 *       -o 02_square -lncurses
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Same Cartesian grid as 01_uniform_rect. The only
 *                  change is how CELL_W is chosen.
 *
 * Aspect ratio   : Terminal character cells are NOT square in pixels.
 *                  A typical font has:  char_w ≈ 8 px,  char_h ≈ 16 px
 *                  → aspect ratio = char_h / char_w ≈ 2.0
 *
 *                  For a cell to appear square in pixels:
 *                    pixel_cell_w  = pixel_cell_h
 *                    CELL_W * char_w = CELL_H * char_h
 *                    CELL_W = CELL_H * (char_h / char_w)
 *                    CELL_W = CELL_H * ASPECT_RATIO
 *
 *                  With CELL_H=4, ASPECT_RATIO=2.0:
 *                    CELL_W = 4 * 2 = 8  → cells are 8 chars wide, 4 chars tall
 *                    pixel size ≈ 8*8=64px × 4*16=64px  ✓  square
 *
 * Formula        : Identical to 01_uniform_rect:
 *                    screen_row = r * CELL_H
 *                    screen_col = c * CELL_W          (but CELL_W = CELL_H * ASPECT)
 *
 * References     :
 *   Terminal font metrics — en.wikipedia.org/wiki/Monospace_font
 *   Pixel aspect ratio    — en.wikipedia.org/wiki/Pixel_aspect_ratio
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Terminal characters are NOT square pixels — they are taller than they are
 * wide.  A typical monospace font cell is ~8 px wide × ~16 px tall.  If you
 * use equal CELL_W and CELL_H in character counts, your grid cells appear
 * tall and thin on the physical display.  To make them look square you must
 * compensate: set CELL_W = CELL_H * (char_height / char_width).
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Think in TWO coordinate systems simultaneously:
 *
 *   Character space:  positions measured in terminal characters (cols/rows).
 *   Pixel space:      positions measured in physical screen pixels.
 *
 * The grid formula lives in character space.  The visual result lives in
 * pixel space.  They are related by the character aspect ratio:
 *
 *   pixel_x = char_x * char_width_px      (typically * 8)
 *   pixel_y = char_y * char_height_px     (typically * 16)
 *
 * For a cell to appear square in pixels:
 *   pixel_cell_width = pixel_cell_height
 *   CELL_W * char_w  = CELL_H * char_h
 *   CELL_W           = CELL_H * (char_h / char_w)
 *   CELL_W           = CELL_H * ASPECT_RATIO
 *
 * DRAWING METHOD
 * ──────────────
 *  1. Pick CELL_H (how many rows per cell).  This is the "base" dimension.
 *  2. DERIVE CELL_W: CELL_W = CELL_H * ASPECT_RATIO (do NOT pick independently).
 *  3. Draw the grid exactly as in 01_uniform_rect using these derived values.
 *  4. Verify visually: the cells should look square on your monitor.
 *
 *  If cells look too tall:  ASPECT_RATIO is too low — increase it.
 *  If cells look too wide:  ASPECT_RATIO is too high — decrease it.
 *  Common values: 2.0 (most terminals), 1.6 (some HD setups), 2.1 (retro).
 *
 * KEY FORMULAS
 * ────────────
 *  Aspect ratio derivation:
 *    ASPECT_RATIO = char_height_px / char_width_px  (≈ 2.0 typical)
 *
 *  Derived cell width (the only formula that differs from 01):
 *    CELL_W = CELL_H * ASPECT_RATIO
 *
 *  Pixel size of each cell (for verification):
 *    pixel_w = CELL_W * char_w  =  CELL_H * ASPECT_RATIO * char_w
 *            =  CELL_H * (char_h/char_w) * char_w
 *            =  CELL_H * char_h    <- equals pixel_h. Square! ✓
 *
 *  Everything else (line test, forward/inverse) is identical to 01.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Integer ASPECT_RATIO: using ASPECT_RATIO=2 (integer) avoids floating
 *    point and keeps CELL_W exact.  If your font is not exactly 2:1, cells
 *    will be slightly non-square but close enough for most purposes.
 *
 *  • Font changes: switching fonts changes the aspect ratio.  The grid
 *    cells will look non-square until ASPECT_RATIO is tuned.  There is no
 *    way to query the true pixel dimensions from within ncurses.
 *
 *  • CELL_H=1 and ASPECT_RATIO=2 gives CELL_W=2: a valid grid but very
 *    dense (every other row is a line, lines 2 chars wide).
 *
 *  • Do NOT pick CELL_W and CELL_H independently for a "square" grid —
 *    their ratio must equal the aspect ratio.  Getting this wrong is the
 *    most common mistake when first drawing grids.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Take a screenshot at 1:1 pixel zoom.  Measure one cell in pixels.
 *  width_px / height_px should be ~1.0 for a square grid.
 *  In the terminal: a cell with CELL_W=8, CELL_H=4 at a 8×16 font is
 *  exactly 64 × 64 pixels — perfectly square.
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

#define TARGET_FPS      30

/*
 * ASPECT_RATIO = char_height / char_width ≈ 2.0 for most terminal fonts.
 *
 * Derivation:
 *   We want: CELL_W * char_w  ==  CELL_H * char_h   (square in pixels)
 *   →  CELL_W = CELL_H * (char_h / char_w) = CELL_H * ASPECT_RATIO
 *
 * If your cells look taller than wide: increase ASPECT_RATIO.
 * If your cells look wider than tall:  decrease ASPECT_RATIO.
 */
#define CELL_H          4               /* rows per cell (you tune this)        */
#define ASPECT_RATIO    2               /* char_h / char_w; integer for clarity */
#define CELL_W          (CELL_H * ASPECT_RATIO)  /* = 8: derived, not a guess  */

#define PAIR_GRID    1
#define PAIR_ACTIVE  2
#define PAIR_PLAYER  3
#define PAIR_HUD     4

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

static void color_init(void)
{
    start_color(); use_default_colors();
    init_pair(PAIR_GRID,   COLORS >= 256 ?  75 : COLOR_CYAN,   -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ? 148 : COLOR_GREEN,  -1);
    init_pair(PAIR_PLAYER, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  15 : COLOR_WHITE,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * cell_to_screen — identical formula to 01_uniform_rect.
 *
 *   screen_row = r * CELL_H
 *   screen_col = c * CELL_W          (CELL_W = CELL_H * ASPECT_RATIO)
 *
 * The ONLY difference from 01 is that CELL_W is derived from CELL_H.
 * That single constraint makes the cells appear square in pixels.
 */
static void cell_to_screen(int r, int c, int *sr, int *sc)
{
    *sr = r * CELL_H;
    *sc = c * CELL_W;
}

static char grid_char(int sr, int sc)
{
    bool h = (sr % CELL_H == 0);
    bool v = (sc % CELL_W == 0);
    if (h && v) return '+';
    if (h)      return '-';
    if (v)      return '|';
    return ' ';
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  player                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int r, c, max_r, max_c; } Player;

static void player_reset(Player *p, int rows, int cols)
{
    p->max_r = (rows - 1) / CELL_H - 1;
    p->max_c = cols / CELL_W - 1;
    p->r = p->max_r / 2;
    p->c = p->max_c / 2;
}

static void player_move(Player *p, int dr, int dc)
{
    int nr = p->r + dr, nc = p->c + dc;
    if (nr >= 0 && nr <= p->max_r) p->r = nr;
    if (nc >= 0 && nc <= p->max_c) p->c = nc;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void draw_grid(int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_GRID));
    for (int sr = 0; sr < rows - 1; sr++)
        for (int sc = 0; sc < cols; sc++) {
            char ch = grid_char(sr, sc);
            if (ch != ' ')
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
        }
    attroff(COLOR_PAIR(PAIR_GRID));
}

static void draw_player(const Player *p)
{
    int sr, sc;
    cell_to_screen(p->r, p->c, &sr, &sc);

    attron(COLOR_PAIR(PAIR_ACTIVE));
    for (int dr = 1; dr < CELL_H; dr++)
        for (int dc = 1; dc < CELL_W; dc++)
            mvaddch(sr + dr, sc + dc, (chtype)'.');
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    attron(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
    mvaddch(sr + CELL_H / 2, sc + CELL_W / 2, (chtype)'@');
    attroff(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Player *p, double fps)
{
    erase();
    draw_grid(rows, cols);
    draw_player(p);

    /* Show the aspect ratio reasoning in the HUD */
    char buf[80];
    snprintf(buf, sizeof buf,
        " %.1f fps  cell(%d,%d)  size:%dx%d chars ~%dpx sq ",
        fps, p->r, p->c, CELL_W, CELL_H, CELL_H * 16);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD) | A_DIM);
    mvprintw(rows - 1, 0,
        " arrows:move  r:reset  q:quit  [02 square  CELL_W=CELL_H*ASPECT(%d)] ",
        ASPECT_RATIO);
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
            case KEY_UP:    player_move(&player, -1,  0); break;
            case KEY_DOWN:  player_move(&player, +1,  0); break;
            case KEY_LEFT:  player_move(&player,  0, -1); break;
            case KEY_RIGHT: player_move(&player,  0, +1); break;
        }
        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (now - t0 + 1)) * 0.05; t0 = now;
        scene_draw(rows, cols, &player, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
