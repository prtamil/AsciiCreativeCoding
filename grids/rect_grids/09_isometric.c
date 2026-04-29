/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 09_isometric.c — flat-parallelogram isometric grid
 *
 * DEMO: Same rotation formula as 08_diamond but with IW=8, IH=2 instead
 *       of DW=4, DH=2. The 4:1 char ratio gives the classic 2:1-pixel
 *       isometric look (26.6°, as used in SimCity/Age of Empires).
 *       Each cell is 16 chars wide × 4 rows tall.  Arrow keys navigate
 *       one grid cell at a time along the grid axes (not screen axes).
 *
 * Study alongside: 08_diamond.c (same formula, 45° square cells)
 *
 * Section map:
 *   §1 config   — IW=8, IH=2 (classic 2:1 isometric aspect)
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 4 pairs
 *   §4 formula  — IDENTICAL formula to 08_diamond; only IW/IH differ
 *   §5 player   — grid-axis movement (RIGHT=c+1, LEFT=c-1, UP=r-1, DOWN=r+1)
 *   §6 scene    — draw_grid(), draw_player()
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  right/left: c±1   up/down: r∓1   r reset   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids/09_isometric.c \
 *       -o 09_isometric -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Identical rotation formula to 08_diamond.
 *                  Only the half-cell extents IW and IH change.
 *
 * Formula        :
 *   screen_col = ox + (c - r) * IW
 *   screen_row = oy + (c + r) * IH
 *
 * Comparison with 08_diamond:
 *   08_diamond:   DW=4, DH=2 → 45° cells (square diamonds)
 *   09_isometric: IW=8, IH=2 → 26.6° cells (classic 2:1 game isometric)
 *                               Cell is 16 chars wide × 4 rows tall.
 *                               In pixel space (8×16 px chars): 128px × 64px = 2:1.
 *
 * Grid lines (from inverse formula with IW=8, IH=2, MODULUS=32):
 *   c-line: (u*IH + v*IW) ≡ 0 (mod 32) → (2u + 8v) ≡ 0 (mod 32) → (u + 4v) ≡ 0 (mod 16)
 *   r-line: (v*IW - u*IH) ≡ 0 (mod 32) → (8v - 2u) ≡ 0 (mod 32) → (4v - u) ≡ 0 (mod 16)
 *
 * Visual: c-lines and r-lines now land at DIFFERENT screen columns per row,
 *         so isolated '/' and '\' characters appear between the '+' corners.
 *         With IH=1 (MODULUS=16) the conditions were identical — only '+' drawn.
 *
 * References     :
 *   Isometric projection — en.wikipedia.org/wiki/Isometric_projection
 *   Same formula as 08_diamond — compare §4 in both files
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The formula is IDENTICAL to 08_diamond.  The ONLY change is the ratio
 * IW:IH.  In 08, DW=4 and DH=2 (2:1 char ratio) makes cells look square.
 * Here, IW=8 and IH=2 (4:1 char ratio) makes cells wide and flat —
 * the classic look of 2D game isometric grids (SimCity, Age of Empires).
 *
 * Understanding how IW:IH controls the "tilt angle":
 *   The slope of a grid line in PIXELS is IH*char_h / IW*char_w.
 *   With IW=8, IH=2, char 8px wide, char 16px tall:
 *     slope = 2*16 / 8*8 = 32/64 = 0.5  →  atan(0.5) ≈ 26.6°  ← classic 2:1 iso
 *   With DW=4, DH=2:
 *     slope = 2*16 / 4*8 = 32/32 = 1.0  →  atan(1.0) = 45°    ← square diamond
 *
 * WHY IH=1 BROKE THE GRID
 * ────────────────────────
 * With IH=1, MODULUS=16.  The c-line condition (u+8v)%16=0 and r-line
 * condition (8v-u)%16=0 are mathematically equivalent: any point satisfying
 * one automatically satisfies the other.  Result: only '+' corners, no '/'
 * or '\' — the grid looked like a sparse dot pattern, not isometric.
 * With IH=2, MODULUS=32: c-line hits u=12 and r-line hits u=4 on odd rows
 * — different positions — so both '/' and '\' characters appear. ✓
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Think of IW and IH as "how far does one cell step stretch on screen?"
 *   Large IW, small IH → cells are wide and shallow (isometric).
 *   IW=IH (after aspect correction) → cells are square (=45° diamond).
 *
 * The "perspective angle" θ from horizontal satisfies:
 *   tan(θ) = (IH * char_height_px) / (IW * char_width_px)
 *
 * Common game choices:
 *   θ ≈ 26.6° (2:1 pixel ratio) — classic 2D isometric (e.g. SimCity)
 *   θ = 30°   (true isometric projection)
 *   θ = 45°   (diamond, as in 08_diamond)
 *
 * IW=8, IH=2 with 8×16 px chars: slope = 0.5 → 26.6°. ✓
 *
 * MOVEMENT — GRID-AXIS vs SCREEN-AXIS
 * ─────────────────────────────────────
 * 08_diamond uses screen-axis movement: RIGHT=(dr=-1,dc=+1) moves purely
 * horizontal on screen (Δsc=2*DW, Δsr=0).  This works for 45° because
 * Δsc = (dc-dr)*DW = 2*4 = 8 chars — a small jump.
 *
 * For isometric (IW=8), screen-axis RIGHT would require Δsc=2*IW=16 chars
 * per keypress — a very large visual jump, and RANGE=6 would place the
 * player 96 chars off-center (off-screen on any normal terminal).
 *
 * Instead, this file uses GRID-AXIS movement: each key changes exactly ONE
 * cell coordinate, just like navigating a normal (non-rotated) grid:
 *   RIGHT  → c += 1  (screen: 8 cols right, 2 rows down)
 *   LEFT   → c -= 1  (screen: 8 cols left,  2 rows up)
 *   UP     → r -= 1  (screen: 8 cols right, 2 rows up)
 *   DOWN   → r += 1  (screen: 8 cols left,  2 rows down)
 * Each step is IW=8 cols — half the jump of screen-axis movement.
 * The movement is "atomic": both r and c must stay in bounds, or neither moves.
 *
 * DRAWING METHOD
 * ──────────────
 *  Exactly as in 08_diamond.  Read 08's mental model first.
 *  The only change in the code: substitute IW for DW, IH for DH.
 *
 *  Grid line conditions with IW=8, IH=2, MODULUS=32:
 *    c-line: (u + 4v) % 16 == 0
 *    r-line: (4v - u) % 16 == 0
 *
 *  At v=0: both → '+' at u=0,16,32...
 *  At v=1: c-line u=12,28; r-line u=4,20 → isolated '/' and '\'
 *  At v=2: both → '+' at u=8,24... (offset corners)
 *  At v=3: c-line u=4; r-line u=12 → isolated '/' and '\'
 *
 * KEY FORMULAS
 * ────────────
 *  Forward:   sc = ox + (c-r)*IW,    sr = oy + (c+r)*IH
 *  c-line:    (u + 4v) % 16 == 0         (IW=8, IH=2)
 *  r-line:    (4v - u) % 16 == 0
 *  Tilt angle: θ = atan(IH*char_h / IW*char_w)  in radians
 *
 *  Parametric comparison:
 *    09_isometric (IW=8, IH=2): θ_px ≈ 26.6° — classic 2:1 game isometric
 *    08_diamond   (DW=4, DH=2): θ_px = 45°   — square diamonds
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Cell dimensions: 2*IW=16 cols wide × 2*IH=4 rows tall.  RANGE=4 limits
 *    the player to ±4 cells per axis.  Worst-case screen offset (diagonal
 *    corner r=-4, c=4): sc = ox + (4-(-4))*IW = ox + 64 cols.  The '@' guard
 *    in draw_player skips drawing if the centre is off-screen — no crash.
 *
 *  • The safe_mod() trick is critical: u and v can be negative near the origin
 *    and C's % operator returns negative results for negative dividends.
 *    Without safe_mod, the modular conditions would never trigger for negative
 *    (u,v), leaving a blank upper-left quadrant.
 *
 *  • '@' centre formula: top corner of cell (r,c) is at (oy+(c+r)*IH, ox+(c-r)*IW).
 *    Centre = average of all 4 corners:
 *      sc_centre = ox + (c-r)*IW          (same column as top/bottom corners)
 *      sr_centre = oy + (c+r+1)*IH        (halfway between top and bottom rows)
 *
 * HOW TO VERIFY
 * ─────────────
 *  Origin '+': at screen (oy, ox).
 *  Cell (0,0) right corner '+': at (oy+IH, ox+IW) = (oy+2, ox+8).
 *  Cell (0,0) bottom corner '+': at (oy+2*IH, ox) = (oy+4, ox).
 *  Cell width = 2*IW = 16 cols.  Cell height = 2*IH = 4 rows.
 *  The '@' for cell (0,0) sits at (oy+IH, ox) = (oy+2, ox) — 2 rows below
 *  the top corner, at the same column.
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
 * IW, IH — isometric half-cell extents in screen characters.
 *   Cell dimensions: 2*IW cols wide × 2*IH rows tall  →  16 wide × 4 tall.
 *   Tilt angle in pixel space: atan(IH*char_h / IW*char_w)
 *                            = atan(2*16 / 8*8) = atan(0.5) ≈ 26.6°  ← classic 2:1 iso.
 *   To adjust the angle: decrease IW (steeper) or decrease IH (flatter).
 *   IW=4, IH=2 gives 45° (same as 08_diamond).
 */
#define IW     8   /* half-cell column extent — cell is 2*IW=16 chars wide */
#define IH     2   /* half-cell row extent    — cell is 2*IH= 4 rows  tall */

/* 2*IW*IH = modulus for grid-line inverse formula */
#define MODULUS  (2 * IW * IH)   /* = 32 */

/*
 * RANGE — how many cells the player can navigate from the origin per axis.
 * With grid-axis movement the worst-case screen position is the diagonal
 * corner (r=-RANGE, c=RANGE) or (r=RANGE, c=-RANGE):
 *   sc = ox ± (RANGE + RANGE) * IW = ox ± 2*RANGE*IW
 * RANGE=4: worst-case sc = ox ± 64.  Safe on terminals wider than 128 cols
 * (ox ≈ cols/2, so 64 px room each side).  The '@' guard prevents crashes
 * if the player reaches an off-screen position on a narrower terminal.
 */
#define RANGE  4   /* player cell range: -RANGE .. +RANGE per axis */

#define PAIR_GRID    1
#define PAIR_ACTIVE  2
#define PAIR_PLAYER  3
#define PAIR_HUD     4

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
    init_pair(PAIR_GRID,   COLORS >= 256 ?  48 : COLOR_GREEN,  -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ?  82 : COLOR_GREEN,  -1);
    init_pair(PAIR_PLAYER, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  15 : COLOR_WHITE,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * cell_to_screen — ISOMETRIC FORMULA (same structure as 08_diamond):
 *
 *   screen_col = ox + (c - r) * IW
 *   screen_row = oy + (c + r) * IH
 *
 * The formula is IDENTICAL to 08_diamond. Only IW=8, IH=2 instead of DW=4, DH=2.
 * Change IW/IH to get any slope between 0° (horizontal grid) and 90° (vertical grid).
 */
static void cell_to_screen(int r, int c, int ox, int oy, int *sr, int *sc)
{
    *sc = ox + (c - r) * IW;
    *sr = oy + (c + r) * IH;
}

static int safe_mod(int a, int b) { return ((a % b) + b) % b; }

/*
 * grid_char — inverse formula with IW=8, IH=2, MODULUS=32:
 *
 *   c-line: (u*IH + v*IW) ≡ 0 (mod 32) → (u + 4v) ≡ 0 (mod 16)  → '/'
 *   r-line: (v*IW - u*IH) ≡ 0 (mod 32) → (4v - u) ≡ 0 (mod 16)  → '\'
 *
 * The conditions hit DIFFERENT positions on odd rows (c-line u=12, r-line u=4),
 * so isolated '/' and '\' characters appear between '+' corners. ✓
 */
static char grid_char(int sr, int sc, int ox, int oy)
{
    int u = sc - ox;
    int v = sr - oy;
    bool c_line = (safe_mod(u * IH + v * IW, MODULUS) == 0);
    bool r_line = (safe_mod(v * IW - u * IH, MODULUS) == 0);
    if (c_line && r_line) return '+';
    if (c_line)           return '/';
    if (r_line)           return '\\';
    return ' ';
}

/*
 * in_player_cell — test whether screen position (sr,sc) lies inside cell (pr,pc).
 *
 * The forward formula sc=ox+(c-r)*IW, sr=oy+(c+r)*IH can be inverted:
 *   cn = u*IH + v*IW  where  cn = c * MODULUS  (u=sc-ox, v=sr-oy)
 *   rn = v*IW - u*IH  where  rn = r * MODULUS
 *
 * Cell (pr,pc) occupies the rectangle:
 *   cn in (pc*MODULUS, (pc+1)*MODULUS]
 *   rn in (pr*MODULUS, (pr+1)*MODULUS]
 *
 * The strict left bound (>) excludes the grid line shared with the previous
 * cell; the inclusive right bound (<=) claims the grid line on the far edge.
 */
static bool in_player_cell(int sr, int sc, int ox, int oy, int pr, int pc)
{
    int u = sc - ox, v = sr - oy;
    int cn = u * IH + v * IW;          /* = MODULUS * c  (scaled c-coordinate) */
    int rn = v * IW - u * IH;          /* = MODULUS * r  (scaled r-coordinate) */
    return (cn > pc * MODULUS && cn <= (pc + 1) * MODULUS &&
            rn > pr * MODULUS && rn <= (pr + 1) * MODULUS);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  player                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int r, c; } Player;

static Player player_reset(void) { return (Player){0, 0}; }

/*
 * player_move — GRID-AXIS movement (one coordinate changes per keypress):
 *   RIGHT/LEFT → dc = ±1  (c changes; screen moves right-down / left-up)
 *   UP/DOWN    → dr = ∓1  (r changes; screen moves right-up  / left-down)
 *
 * Movement is ATOMIC: both r and c must stay within [-RANGE, RANGE],
 * otherwise neither changes.  This prevents partial moves at boundaries
 * where one coordinate is clamped independently (which would produce
 * unintended diagonal motion on screen).
 */
static void player_move(Player *p, int dr, int dc)
{
    int nr = p->r + dr, nc = p->c + dc;
    if (nr >= -RANGE && nr <= RANGE && nc >= -RANGE && nc <= RANGE) {
        p->r = nr;
        p->c = nc;
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * draw_grid — raster-scan every screen cell and draw the isometric grid.
 * grid_char() returns '+', '/', '\', or ' ' for each position.
 * Skipping ' ' avoids erasing the background on every cell.
 */
static void draw_grid(int rows, int cols, int ox, int oy)
{
    attron(COLOR_PAIR(PAIR_GRID));
    for (int sr = 0; sr < rows - 1; sr++)
        for (int sc = 0; sc < cols; sc++) {
            char ch = grid_char(sr, sc, ox, oy);
            if (ch != ' ')
                mvaddch(sr, sc, (chtype)(unsigned char)ch);
        }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/*
 * draw_player — highlight the active cell interior and draw '@' at its centre.
 *
 * Two-pass approach:
 *   1. Fill interior: search a bounding box around the cell's top corner,
 *      test each position with in_player_cell(), draw '.' on empty space.
 *      span_r = 2*IH+1  covers the full 4-row cell height plus one guard row.
 *      span_c = 2*IW    covers the full 16-col cell width.
 *
 *   2. Draw '@': placed at the cell centre (sc=ox+(c-r)*IW, sr=oy+(c+r+1)*IH).
 *      Guarded on the CENTRE position (not the top corner) so '@' stays visible
 *      even when the cell's top corner scrolls above the screen edge.
 */
static void draw_player(const Player *p, int ox, int oy, int rows, int cols)
{
    int csr, csc; cell_to_screen(p->r, p->c, ox, oy, &csr, &csc);
    int span_r = IH * 2 + 1;   /* rows to search: covers 2*IH=4 row cell height */
    int span_c = IW * 2;        /* cols to search: covers 2*IW=16 col cell width  */

    attron(COLOR_PAIR(PAIR_ACTIVE));
    for (int dr = -span_r; dr <= span_r; dr++) {
        for (int dc = -span_c; dc <= span_c; dc++) {
            int sr = csr + dr, sc = csc + dc;
            if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) continue;
            if (!in_player_cell(sr, sc, ox, oy, p->r, p->c)) continue;
            if (grid_char(sr, sc, ox, oy) == ' ')
                mvaddch(sr, sc, (chtype)'.');
        }
    }
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    /* Centre of cell (r,c): sc = ox+(c-r)*IW,  sr = oy+(c+r+1)*IH.
     * Guard on the CENTRE (not the top corner csr/csc) so @ stays visible
     * even when the cell's top corner is above the screen edge. */
    int centre_sc = ox + (p->c - p->r) * IW;
    int centre_sr = oy + (p->c + p->r + 1) * IH;
    if (centre_sr >= 0 && centre_sr < rows - 1 &&
        centre_sc >= 0 && centre_sc < cols) {
        attron(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
        mvaddch(centre_sr, centre_sc, (chtype)'@');
        attroff(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
    }
}

static void scene_draw(int rows, int cols, const Player *p, double fps)
{
    int ox = cols / 2, oy = (rows - 1) / 2;
    erase();
    draw_grid(rows, cols, ox, oy);
    draw_player(p, ox, oy, rows, cols);

    char buf[80];
    snprintf(buf, sizeof buf, " %.1f fps  cell(%d,%d)  IW=%d IH=%d ", fps, p->r, p->c, IW, IH);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD) | A_DIM);
    mvprintw(rows - 1, 0,
        " right/left:c±1  up/down:r∓1 (grid-axis)  r:reset  q:quit  [09 iso] ");
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
    Player player = player_reset();
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double fps = TARGET_FPS; int64_t t0 = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            rows = LINES; cols = COLS;
        }
        int ch = getch();
        switch (ch) {
            case 'q': case 27: g_running = 0;           break;
            case 'r':          player = player_reset();  break;
            case KEY_RIGHT: player_move(&player,  0, +1); break;
            case KEY_LEFT:  player_move(&player,  0, -1); break;
            case KEY_UP:    player_move(&player, -1,  0); break;
            case KEY_DOWN:  player_move(&player, +1,  0); break;
        }
        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (now - t0 + 1)) * 0.05; t0 = now;
        scene_draw(rows, cols, &player, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
