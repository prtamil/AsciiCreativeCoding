/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 08_diamond.c — 45°-rotated rectangular grid (diamond / isometric cells)
 *
 * DEMO: The same square grid of 01_uniform_rect, rotated 45 degrees.
 *       Each cell appears as a diamond (rhombus). The screen origin is at
 *       the terminal centre. Arrow keys move '@' in screen-aligned
 *       directions: RIGHT moves visually right, UP moves visually up.
 *
 * Study alongside: 01_uniform_rect.c (pre-rotation), 09_isometric.c
 *
 * Section map:
 *   §1 config   — DW, DH (half-cell screen extents), RANGE
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 4 pairs
 *   §4 formula  — 45° rotation formula + inverse (grid line rasterisation)
 *   §5 player   — struct, screen-direction movement mapping
 *   §6 scene    — draw_grid(), draw_player()
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, main loop
 *
 * Keys:  arrows move @   r reset   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids/08_diamond.c \
 *       -o 08_diamond -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : 45° rotation of the Cartesian grid.
 *                  The standard grid has axes along (1,0) and (0,1).
 *                  The diamond grid rotates them to (1,1) and (1,-1).
 *
 * Forward formula (cell → screen):
 *
 *   screen_col = ox + (c - r) * DW
 *   screen_row = oy + (c + r) * DH
 *
 *   where ox,oy = screen centre, DW = half-cell column step, DH = half-cell row step.
 *
 *   Derivation: rotate grid axes by 45°.
 *     In a standard grid: x = c, y = r.
 *     Rotate 45° CW: x' = (c + r) / √2,  y' = (c - r) / √2
 *     Then scale to terminal: col = x' * DW*√2,  row = y' * DH*√2
 *     → col = (c + r) * DW,   row = ... but which axis maps to col vs row?
 *     Convention used here: (c-r) → col, (c+r) → row.
 *     This means: moving c right shifts the cell right AND down by equal parts;
 *                 moving r up   shifts the cell right AND up  by equal parts.
 *
 * Inverse formula (screen → fractional cell, for grid line rasterisation):
 *
 *   Let u = sc - ox,  v = sr - oy
 *   c = (u/DW + v/DH) / 2
 *   r = (v/DH - u/DW) / 2
 *
 *   A grid line exists where c or r is an INTEGER.
 *   Multiply through by 2*DW*DH to avoid floating point:
 *     c integer  →  (u*DH + v*DW) % (2*DW*DH) == 0   → draw '/'
 *     r integer  →  (v*DW - u*DH) % (2*DW*DH) == 0   → draw '\'
 *
 *   With DW=4, DH=2, 2*DW*DH=16:
 *     c-line: (2u + 4v) ≡ 0 (mod 16)  →  (u + 2v) ≡ 0 (mod 8)
 *     r-line: (4v - 2u) ≡ 0 (mod 16)  →  (2v - u) ≡ 0 (mod 8)
 *
 * Movement       : Arrow keys are mapped to screen-space directions.
 *   RIGHT (screen right, no vertical change):
 *     d_col = (dc - dr)*DW > 0, d_row = (dc + dr)*DH = 0
 *     → dc - dr = +k, dc + dr = 0  → dc = +1, dr = -1
 *   LEFT:   dc = -1, dr = +1
 *   UP (no horizontal change, screen row decreases):
 *     dc - dr = 0, dc + dr = -k → dc = -1, dr = -1
 *   DOWN:   dc = +1, dr = +1
 *
 * References     :
 *   Isometric projection — en.wikipedia.org/wiki/Isometric_projection
 *   Diamond / isometric grids in games — redblobgames.com/grids/hexagons
 *   Rotation matrix — en.wikipedia.org/wiki/Rotation_matrix
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Take the standard rectangular grid and rotate it 45 degrees.  The cells
 * become diamond shapes.  The coordinate formula uses the SUM and DIFFERENCE
 * of (c, r) — a trick that encodes both axes of the rotated grid in two
 * simple linear expressions:
 *
 *   (c - r)  drives screen column  (the "east-west" axis of the diamond grid)
 *   (c + r)  drives screen row     (the "north-south" axis of the diamond grid)
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * In the standard grid, moving right (+c) shifts the cell right on screen.
 * In the diamond grid, moving right (+c) shifts right AND down by equal
 * amounts.  Moving up (+r negated = -r) shifts right AND up.  The net
 * effect is that the axes point diagonally — northeast and southeast.
 *
 * The "sum and difference" trick:
 *   Standard grid axes: (1,0) and (0,1)  — horizontal and vertical.
 *   Diamond  grid axes: (1,1) and (1,-1) — diagonals.
 *   (c-r) = c projected onto the (1,-1) axis  -> horizontal screen displacement
 *   (c+r) = c projected onto the (1,+1) axis  -> vertical screen displacement
 *
 * Think of it as a 45° rotation matrix applied to (c, r):
 *   [ sc ]   [ DW   -DW ] [ c ]   + [ox]
 *   [ sr ] = [ DH    DH ] [ r ]   + [oy]
 *
 * DRAWING METHOD — the INVERSE FORMULA approach
 * ──────────────────────────────────────────────
 * You cannot use the simple modular test of 01_uniform_rect because the
 * grid lines are diagonal.  Instead, for every screen position (sr, sc),
 * compute the fractional cell coordinates (c, r) using the inverse formula,
 * and check whether c or r is near an integer:
 *
 *  Step 1: Offset from origin.
 *    u = sc - ox     v = sr - oy
 *
 *  Step 2: Solve the forward equations for c and r:
 *    sc = ox + (c-r)*DW  ->  u = (c-r)*DW
 *    sr = oy + (c+r)*DH  ->  v = (c+r)*DH
 *    c = (u/DW + v/DH) / 2       r = (v/DH - u/DW) / 2
 *
 *  Step 3: Multiply both sides by 2*DW*DH to clear fractions:
 *    2*DW*DH*c = u*DH + v*DW    (call this c_num)
 *    2*DW*DH*r = v*DW - u*DH    (call this r_num)
 *
 *  Step 4: c is an integer when c_num % (2*DW*DH) == 0  -> c-line ('/').
 *          r is an integer when r_num % (2*DW*DH) == 0  -> r-line ('\').
 *
 *  With DW=4, DH=2, 2*DW*DH=16:
 *    c-line: (u*2 + v*4) % 16 == 0  ->  (u + 2v) % 8 == 0
 *    r-line: (v*4 - u*2) % 16 == 0  ->  (2v - u) % 8 == 0
 *
 * WHY '/' FOR c-LINES AND '\' FOR r-LINES
 * ────────────────────────────────────────
 *  On a c-line (constant c), r varies.  As r increases by 1:
 *    sc decreases by DW (goes left), sr increases by DH (goes down).
 *  => Going left-and-down as r increases = the '/' direction.
 *
 *  On an r-line (constant r), c varies.  As c increases by 1:
 *    sc increases by DW (goes right), sr increases by DH (goes down).
 *  => Going right-and-down as c increases = the '\' direction.
 *
 * SCREEN-DIRECTION MOVEMENT
 * ─────────────────────────
 *  Arrow keys should move '@' in the expected screen direction.
 *  Derive the delta (dr, dc) from the desired screen delta (Dsr, Dsc):
 *
 *    Dsc = (dc - dr)*DW     Dsr = (dc + dr)*DH
 *
 *  For RIGHT (Dsc > 0, Dsr = 0):
 *    dc + dr = 0  AND  dc - dr > 0  =>  dc=+1, dr=-1
 *  For LEFT:   dc=-1, dr=+1
 *  For UP   (Dsc = 0, Dsr < 0):
 *    dc - dr = 0  AND  dc + dr < 0  =>  dc=-1, dr=-1
 *  For DOWN:   dc=+1, dr=+1
 *
 * KEY FORMULAS SUMMARY
 * ────────────────────
 *  Forward:   sc = ox + (c-r)*DW,    sr = oy + (c+r)*DH
 *  c-line:    (u + 2v) % 8 == 0      (u=sc-ox, v=sr-oy, with DW=4,DH=2)
 *  r-line:    (2v - u) % 8 == 0
 *  In-cell:   c_num in [pc*16, (pc+1)*16)  AND  r_num in [pr*16, (pr+1)*16)
 *  Cell centre of (pr,pc):  sc=ox+(pc-pr)*DW,  sr=oy+(pc+pr+1)*DH
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Negative modulo: C's % can return negative for negative u or v.
 *    Always use safe_mod: ((a % N) + N) % N.  Forgetting this gives
 *    gaps or phantom lines in the lower-left quadrant.
 *
 *  • Origin placement: cells extend in all ± directions from (ox, oy).
 *    If ox or oy is not at the screen centre, many cells will be clipped.
 *
 *  • Cell highlight bounding box: the diamond cell is not axis-aligned.
 *    You must scan a bounding box of ±DW*2 cols and ±DH*2 rows around
 *    the cell centre, then use in_player_cell() to reject non-interior pts.
 *
 *  • DW and DH must satisfy 2*DW*DH divides into nice per-pixel increments.
 *    With DW=4, DH=2, each screen position changes c_num by 2 (per col step)
 *    and 4 (per row step), so c_num cycles through 0..15 cleanly.
 *
 * HOW TO VERIFY
 * ─────────────
 *  Cell (0,0): origin (ox, oy) should show '+'.
 *  Cell (0,1): screen at (ox+DW, oy+DH) = (ox+4, oy+2) should show '+'.
 *  Cell (1,0): screen at (ox-DW, oy+DH) = (ox-4, oy+2) should show '+'.
 *  The '+' at the origin should appear at the exact screen centre.
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
 * DW, DH — half-cell extents in screen space.
 *   Moving one cell in the c direction: Δcol = DW, Δrow = DH
 *   Moving one cell in the r direction: Δcol = DW, Δrow = DH (but sign differs)
 *
 * For cells to appear square in pixel space:
 *   DW * char_w == DH * char_h  →  DW/DH == char_h/char_w ≈ 2
 *   DW=4, DH=2 → pixel width = 4*8=32, pixel height = 2*16=32 ✓
 */
#define DW     4   /* half-cell column extent in screen chars */
#define DH     2   /* half-cell row extent in screen chars    */

/* RANGE: player cells ∈ [-RANGE, +RANGE] in both r and c */
#define RANGE  8

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
    init_pair(PAIR_GRID,   COLORS >= 256 ?  87 : COLOR_CYAN,   -1);
    init_pair(PAIR_ACTIVE, COLORS >= 256 ?  82 : COLOR_GREEN,  -1);
    init_pair(PAIR_PLAYER, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS >= 256 ?  15 : COLOR_WHITE,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * cell_to_screen — DIAMOND FORMULA:
 *
 *   screen_col = ox + (c - r) * DW
 *   screen_row = oy + (c + r) * DH
 *
 * Memorise as: (c-r) drives horizontal, (c+r) drives vertical.
 * The origin cell (0,0) maps to screen centre (ox,oy).
 */
static void cell_to_screen(int r, int c, int ox, int oy, int *sr, int *sc)
{
    *sc = ox + (c - r) * DW;
    *sr = oy + (c + r) * DH;
}

/*
 * safe_mod — C's % operator returns negative for negative dividend.
 * We need non-negative remainder for the grid line tests below.
 */
static int safe_mod(int a, int b) { return ((a % b) + b) % b; }

/*
 * grid_char — INVERSE FORMULA for grid line rasterisation.
 *
 * Given screen position (sr, sc):
 *   u = sc - ox,  v = sr - oy
 *
 * c-line (appears as '/'): (u*DH + v*DW) ≡ 0 (mod 2*DW*DH)
 * r-line (appears as '\'): (v*DW - u*DH) ≡ 0 (mod 2*DW*DH)
 *
 * With DW=4, DH=2, 2*DW*DH=16:
 *   c-line: (u + 2v) ≡ 0 (mod 8)   (simplified by dividing all by DH=2)
 *   r-line: (2v - u) ≡ 0 (mod 8)
 *
 * Why '/' for c-lines and '\' for r-lines?
 *   A c-line has constant c, varying r. As r increases by 1:
 *     sc changes by -DW (left), sr changes by +DH (down) → going down-LEFT = '/'
 *   A r-line has constant r, varying c. As c increases by 1:
 *     sc changes by +DW (right), sr changes by +DH (down) → going down-RIGHT = '\'
 */
static char grid_char(int sr, int sc, int ox, int oy)
{
    int u = sc - ox;
    int v = sr - oy;
    bool c_line = (safe_mod(u + 2 * v, 8) == 0);
    bool r_line = (safe_mod(2 * v - u, 8) == 0);
    if (c_line && r_line) return '+';
    if (c_line)           return '/';
    if (r_line)           return '\\';
    return ' ';
}

/*
 * in_player_cell — test whether screen (sr,sc) is in the interior of cell (pr,pc).
 *
 * Cell (pr,pc) occupies: pr ≤ r < pr+1  AND  pc ≤ c < pc+1 in cell space.
 * In terms of u = sc-ox, v = sr-oy (scaled by 2*DW*DH = 16):
 *   c * 16 = u*DH + v*DW  → for pc ≤ c < pc+1:
 *              16*pc ≤ u*2 + v*4 < 16*(pc+1)
 *   r * 16 = v*DW - u*DH  → for pr ≤ r < pr+1:
 *              16*pr ≤ v*4 - u*2 < 16*(pr+1)
 */
static bool in_player_cell(int sr, int sc, int ox, int oy, int pr, int pc)
{
    int u = sc - ox, v = sr - oy;
    int cn = u * DH + v * DW;          /* = 16 * c */
    int rn = v * DW - u * DH;          /* = 16 * r */
    int denom = 2 * DW * DH;           /* = 16     */
    return (cn > pc * denom && cn <= (pc + 1) * denom &&
            rn > pr * denom && rn <= (pr + 1) * denom);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  player                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int r, c; } Player;

static Player player_reset(void) { return (Player){0, 0}; }

/*
 * player_move — SCREEN-DIRECTION MOVEMENT for diamond grid.
 *
 * Arrow key → screen direction → (dr, dc) delta:
 *
 *   For RIGHT (Δscreen_col > 0, Δscreen_row = 0):
 *     Δcol = (dc - dr)*DW  must be > 0
 *     Δrow = (dc + dr)*DH  must be = 0
 *     → dc + dr = 0, dc - dr > 0  → dc=+1, dr=-1
 *
 *   For LEFT:   dc=-1, dr=+1
 *   For UP (Δscreen_row < 0, Δscreen_col = 0):
 *     dc + dr < 0, dc - dr = 0  → dc=-1, dr=-1
 *   For DOWN:   dc=+1, dr=+1
 */
static void player_move(Player *p, int dr, int dc)
{
    int nr = p->r + dr, nc = p->c + dc;
    if (nr >= -RANGE && nr <= RANGE) p->r = nr;
    if (nc >= -RANGE && nc <= RANGE) p->c = nc;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

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

static void draw_player(const Player *p, int ox, int oy, int rows, int cols)
{
    /* Highlight all screen positions inside the player's diamond cell */
    attron(COLOR_PAIR(PAIR_ACTIVE));
    /* Scan a bounding box: ±(DW+DH) around the cell centre */
    int csr, csc; cell_to_screen(p->r, p->c, ox, oy, &csr, &csc);
    int span_r = DH * 2, span_c = DW * 2;
    for (int dr = -span_r; dr <= span_r; dr++) {
        for (int dc = -span_c; dc <= span_c; dc++) {
            int sr = csr + dr, sc = csc + dc;
            if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) continue;
            if (!in_player_cell(sr, sc, ox, oy, p->r, p->c)) continue;
            char ch = grid_char(sr, sc, ox, oy);
            if (ch == ' ')
                mvaddch(sr, sc, (chtype)'.');
        }
    }
    attroff(COLOR_PAIR(PAIR_ACTIVE));

    /* '@' at cell centre */
    if (csr >= 0 && csr < rows - 1 && csc >= 0 && csc < cols) {
        attron(COLOR_PAIR(PAIR_PLAYER) | A_BOLD);
        mvaddch(csr + DH, csc, (chtype)'@');   /* centre = (ox, oy+DH) for (0,0) */
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
    snprintf(buf, sizeof buf,
        " %.1f fps  cell(%d,%d)  screen(%d,%d) ",
        fps, p->r, p->c,
        ox + (p->c - p->r) * DW, oy + (p->c + p->r) * DH);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD) | A_DIM);
    mvprintw(rows - 1, 0,
        " arrows:move (screen-dir)  r:reset  q:quit  [08 diamond  DW=%d DH=%d] ",
        DW, DH);
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
            case 'q': case 27: g_running = 0;              break;
            case 'r':          player = player_reset();    break;
            /* Screen-direction movement: see §4 derivation */
            case KEY_RIGHT: player_move(&player, -1, +1);  break;
            case KEY_LEFT:  player_move(&player, +1, -1);  break;
            case KEY_UP:    player_move(&player, -1, -1);  break;
            case KEY_DOWN:  player_move(&player, +1, +1);  break;
        }
        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (now - t0 + 1)) * 0.05; t0 = now;
        scene_draw(rows, cols, &player, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
