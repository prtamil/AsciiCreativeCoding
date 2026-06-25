/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_direct.c — drop objects onto any of 14 grid styles with a movable cursor.
 *
 * SPACE toggles an object on the cursor's cell; a/e cycle the 14 backgrounds.
 * Each grid supplies one (row,col)->screen formula (cell_to_screen) and one
 * background drawer; the object list and cursor work the same on all of them.
 *
 * Sister files: rect_grids/01_uniform_rect.c (the grid formulas), 02_patterns.c.
 */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ── */

#define TARGET_FPS  30
#define MAX_OBJ    256       /* most objects we can hold at once */

/* How much the fps number leans on the latest frame vs. the running average.
   Small value = a steady reading that doesn't jitter every frame. */
#define FPS_EWMA_ALPHA  0.05

/* Cell sizes for each grid style, all in one place so they're easy to tweak. */
#define U_CW  8              /* GM_UNIFORM cell width  */
#define U_CH  4              /* GM_UNIFORM cell height */
#define SQ_CS 3              /* GM_SQUARE  cell step   */
#define FN_CW 4              /* GM_FINE    cell width  */
#define FN_CH 2              /* GM_FINE    cell height */
#define CO_CW 12             /* GM_COARSE  cell width  */
#define CO_CH 4              /* GM_COARSE  cell height */
#define HI_CW 6              /* GM_HIER    major cell  */
#define HI_CH 3
#define BH_CW 10             /* GM_BRICK_H brick width */
#define BH_CH 3              /* GM_BRICK_H brick height*/
#define BV_CW 4              /* GM_BRICK_V brick width */
#define BV_CH 6              /* GM_BRICK_V brick height*/
#define DM_IW 4              /* GM_DIAMOND half-cell col */
#define DM_IH 2              /* GM_DIAMOND half-cell row */
#define DM_RNG 5             /* GM_DIAMOND grid range  */
#define IS_IW 8              /* GM_ISO half-cell col   */
#define IS_IH 2              /* GM_ISO half-cell row   */
#define IS_RNG 4             /* GM_ISO grid range      */
#define CR_CW 8              /* GM_CROSS cell width    */
#define CR_CH 4              /* GM_CROSS cell height   */
#define CK_CW 6              /* GM_CHECK cell width    */
#define CK_CH 3              /* GM_CHECK cell height   */
#define RL_LS 3              /* GM_RULED line step     */
#define RL_CS 4              /* GM_RULED col step      */
#define DT_CW 6              /* GM_DOT cell width      */
#define DT_CH 3              /* GM_DOT cell height     */
#define OR_CW 10             /* GM_ORIGIN cell width   */
#define OR_CH 4              /* GM_ORIGIN cell height  */

/* color slots */
#define PAIR_GRID    1
#define PAIR_ACTIVE  2
#define PAIR_CURSOR  3
#define PAIR_OBJ     4
#define PAIR_HUD     5   /* status bar (yellow)  */
#define PAIR_HINT    6   /* key-hint footer (cyan) */

/* ── §2 clock ── */

static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec=(time_t)(ns/1000000000LL),
                          .tv_nsec=(long)(ns%1000000000LL) };
    nanosleep(&r, NULL);
}

/* ── §3 color ── */

static void color_init(void)
{
    start_color(); use_default_colors();
    init_pair(PAIR_GRID,   COLORS>=256 ?  75 : COLOR_CYAN,    -1);
    init_pair(PAIR_ACTIVE, COLORS>=256 ?  82 : COLOR_GREEN,   -1);
    init_pair(PAIR_CURSOR, COLORS>=256 ? 226 : COLOR_YELLOW,  -1);
    init_pair(PAIR_OBJ,    COLORS>=256 ? 214 : COLOR_RED,     -1);
    init_pair(PAIR_HUD,    COLORS>=256 ? 226 : COLOR_YELLOW,  -1);
    init_pair(PAIR_HINT,   COLORS>=256 ?  51 : COLOR_CYAN,    -1);
}

/* ── §4 grid mapping & backgrounds ── */

/* GridMode — which of the 14 grid styles we're showing right now. The order
   here matches the 14 sibling source files, and GM_COUNT (last) doubles as the
   count, so a/e can wrap around with a simple modulo. */
typedef enum {
    GM_UNIFORM=0, GM_SQUARE, GM_FINE, GM_COARSE,
    GM_HIER, GM_BRICK_H, GM_BRICK_V, GM_DIAMOND,
    GM_ISO, GM_CROSS, GM_CHECK, GM_RULED,
    GM_DOT, GM_ORIGIN,
    GM_COUNT
} GridMode;

static const char *const gm_name[GM_COUNT] = {
    "01 uniform","02 square","03 fine","04 coarse",
    "05 hier","06 brick-h","07 brick-v","08 diamond",
    "09 iso","10 cross","11 check","12 ruled",
    "13 dot","14 origin"
};

/* GridCtx — the grid on screen now, rebuilt on every switch/resize. */
typedef struct {
    GridMode mode;
    int rows, cols;                  /* terminal size in characters */
    int cw, ch;                      /* cell width/height in characters */
    int ox, oy;                      /* where cell (0,0) lands; centre for diamond/iso */
    int range;                       /* diamond/iso run -range..+range per axis */
    int min_r, max_r, min_c, max_c;  /* cursor roam limits, in cells */
} GridCtx;

/* One tiny setter per grid style, just filling in that style's cell size.
   Split out so each style is easy to find and tweak on its own. */

static void ctx_geom_uniform (GridCtx *g) { g->cw = U_CW;    g->ch = U_CH;  }
static void ctx_geom_square  (GridCtx *g) { g->cw = SQ_CS*2; g->ch = SQ_CS; }
static void ctx_geom_fine    (GridCtx *g) { g->cw = FN_CW;   g->ch = FN_CH; }
static void ctx_geom_coarse  (GridCtx *g) { g->cw = CO_CW;   g->ch = CO_CH; }
static void ctx_geom_hier    (GridCtx *g) { g->cw = HI_CW;   g->ch = HI_CH; }
static void ctx_geom_brick_h (GridCtx *g) { g->cw = BH_CW;   g->ch = BH_CH; }
static void ctx_geom_brick_v (GridCtx *g) { g->cw = BV_CW;   g->ch = BV_CH; }
/* the diamond/iso grids also set how far out their centred cells go */
static void ctx_geom_diamond (GridCtx *g) { g->cw = DM_IW; g->ch = DM_IH; g->range = DM_RNG; }
static void ctx_geom_iso     (GridCtx *g) { g->cw = IS_IW; g->ch = IS_IH; g->range = IS_RNG; }
static void ctx_geom_cross   (GridCtx *g) { g->cw = CR_CW;   g->ch = CR_CH; }
static void ctx_geom_check   (GridCtx *g) { g->cw = CK_CW;   g->ch = CK_CH; }
/* ruled is just horizontal lines, so there's no column width to set */
static void ctx_geom_ruled   (GridCtx *g) { g->ch = RL_LS; }
static void ctx_geom_dot     (GridCtx *g) { g->cw = DT_CW;   g->ch = DT_CH; }
static void ctx_geom_origin  (GridCtx *g) { g->cw = OR_CW;   g->ch = OR_CH; }

/* Work out how far the cursor may travel. It's not just "screen size / cell
   size": the diamond/iso grids count outward from a centre, and the ruled grid
   counts whole lines instead of cells, so each gets its own rule. */
static void ctx_set_bounds(GridCtx *g, GridMode m, int rows, int cols)
{
    if (m == GM_DIAMOND || m == GM_ISO) {
        g->min_r = -g->range; g->max_r = g->range;
        g->min_c = -g->range; g->max_c = g->range;
    } else if (m == GM_RULED) {
        g->min_r = 0; g->max_r = (rows-1)/g->ch - 1;
        g->min_c = 0; g->max_c = cols - 1;
    } else {
        g->min_r = 0; g->max_r = (rows-2)/g->ch;
        g->min_c = 0; g->max_c = (cols-1)/g->cw;
    }
}

/* Set up a fresh grid: pick the matching size setter, then nail down the
   origin and the cursor's roaming limits. Called on startup, on resize, and
   every time you switch grids. */
static void ctx_init(GridCtx *g, GridMode m, int rows, int cols)
{
    memset(g, 0, sizeof *g);
    g->mode = m; g->rows = rows; g->cols = cols;
    g->ox = cols / 2; g->oy = rows / 2;

    switch (m) {
        case GM_UNIFORM:  ctx_geom_uniform (g); break;
        case GM_SQUARE:   ctx_geom_square  (g); break;
        case GM_FINE:     ctx_geom_fine    (g); break;
        case GM_COARSE:   ctx_geom_coarse  (g); break;
        case GM_HIER:     ctx_geom_hier    (g); break;
        case GM_BRICK_H:  ctx_geom_brick_h (g); break;
        case GM_BRICK_V:  ctx_geom_brick_v (g); break;
        case GM_DIAMOND:  ctx_geom_diamond (g); break;
        case GM_ISO:      ctx_geom_iso     (g); break;
        case GM_CROSS:    ctx_geom_cross   (g); break;
        case GM_CHECK:    ctx_geom_check   (g); break;
        case GM_RULED:    ctx_geom_ruled   (g); break;
        case GM_DOT:      ctx_geom_dot     (g); break;
        case GM_ORIGIN:   ctx_geom_origin  (g); break;
        default:          g->cw = 8; g->ch = 4; break;
    }
    ctx_set_bounds(g, m, rows, cols);
}

/* The one place a cell (r,c) becomes a screen spot (sr,sc). Every grid style
   has its own way of laying cells out, so this is where they differ. For the
   rectangular grids it returns the cell's top-left corner; for the slanted
   diamond/iso grids it returns the cell's centre. Keeping all the layout math
   here means objects and the cursor never have to know which grid they're on —
   they just ask this. */
static void cell_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    switch (g->mode) {
    case GM_DIAMOND:
        *sc = g->ox + (c - r) * DM_IW;
        *sr = g->oy + (c + r) * DM_IH;
        break;
    case GM_ISO:
        *sc = g->ox + (c - r) * IS_IW;
        *sr = g->oy + (c + r) * IS_IH;
        break;
    case GM_RULED:
        *sr = r * RL_LS;
        *sc = c;
        break;
    case GM_BRICK_H:
        *sr = r * g->ch;
        *sc = c * g->cw + (r % 2) * (g->cw / 2);
        break;
    case GM_BRICK_V:
        *sr = r * g->ch + (c % 2) * (g->ch / 2);
        *sc = c * g->cw;
        break;
    default:
        *sr = r * g->ch;
        *sc = c * g->cw;
        break;
    }
}

/* Remainder that's never negative. Plain % can come back negative for negative
   inputs (which the centred grids have), and that would break the line tests. */
static int safe_mod(int a, int b) { return ((a % b) + b) % b; }

/* Background drawers — one per grid style. Each paints the grid lines for its
   style. Split into separate functions so you can read one style at a time. */

/* Plain rectangular grid. The "origin" style also paints a bold cross through
   the centre to mark where (0,0) sits. */
static void bg_draw_rect_family(const GridCtx *g)
{
    int rows = g->rows, cols = g->cols;
    int cw = g->cw, ch = g->ch, ox = g->ox, oy = g->oy;

    for (int sr = 0; sr < rows-1; sr++) {
        for (int sc = 0; sc < cols; sc++) {
            bool hl = (sr % ch == 0), vl = (sc % cw == 0);
            if (!hl && !vl) continue;
            char c = (hl && vl) ? '+' : (hl ? '-' : '|');
            if (g->mode == GM_ORIGIN && sr == oy) c = (vl ? '+' : '=');
            if (g->mode == GM_ORIGIN && sc == ox) c = (hl ? '+' : 'I');
            mvaddch(sr, sc, (chtype)(unsigned char)c);
        }
    }
}

/* A grid with three line weights — heavy, medium, light — so it reads like
   graph paper. The character drawn tells you which weight a line is. */
static void bg_draw_hier(const GridCtx *g)
{
    int rows = g->rows, cols = g->cols;
    int cw = g->cw, ch = g->ch;
    int major = cw * 2, semi = cw;  /* major=12, semi=6, minor=3 */

    for (int sr = 0; sr < rows-1; sr++) {
        for (int sc = 0; sc < cols; sc++) {
            bool hm = (sr % major == 0), hs = (!hm && sr % semi == 0),
                 hmi= (!hm && !hs && sr % ch == 0);
            bool vm = (sc % major == 0), vs = (!vm && sc % semi == 0),
                 vmi= (!vm && !vs && sc % cw == 0);
            if (!hm && !hs && !hmi && !vm && !vs && !vmi) continue;
            bool hl = hm || hs || hmi, vl = vm || vs || vmi;
            char c;
            if (hl && vl) c = '+';
            else if (hl)  c = (hm ? '=' : (hs ? '-' : '.'));
            else          c = (vm ? '#' : (vs ? '|' : ':'));
            mvaddch(sr, sc, (chtype)(unsigned char)c);
        }
    }
}

/* Brick wall: every other row is nudged sideways by half a brick so the seams
   don't line up, like a real wall. */
static void bg_draw_brick_h(const GridCtx *g)
{
    int rows = g->rows, cols = g->cols, cw = g->cw, ch = g->ch;
    int half = cw / 2;

    for (int sr = 0; sr < rows-1; sr++) {
        bool hl = (sr % ch == 0);
        int rb = sr / ch;
        for (int sc = 0; sc < cols; sc++) {
            bool vl = ((sc + (rb % 2) * half) % cw == 0);
            if (!hl && !vl) continue;
            mvaddch(sr, sc, (chtype)(hl && vl ? '+' : (hl ? '-' : '|')));
        }
    }
}

/* The same brick idea turned on its side: every other column is nudged up or
   down by half a brick. */
static void bg_draw_brick_v(const GridCtx *g)
{
    int rows = g->rows, cols = g->cols, cw = g->cw, ch = g->ch;
    int half = ch / 2;

    for (int sr = 0; sr < rows-1; sr++) {
        for (int sc = 0; sc < cols; sc++) {
            int cb = sc / cw;
            bool hl = ((sr + (cb % 2) * half) % ch == 0);
            bool vl = (sc % cw == 0);
            if (!hl && !vl) continue;
            mvaddch(sr, sc, (chtype)(hl && vl ? '+' : (hl ? '-' : '|')));
        }
    }
}

/* Diamond grid: two sets of slanted lines crossing to make diamonds, the same
   square grid tilted 45 degrees. */
static void bg_draw_diamond(const GridCtx *g)
{
    int rows = g->rows, cols = g->cols, ox = g->ox, oy = g->oy;
    int mod = 2 * DM_IW * DM_IH;  /* = 16 */

    for (int sr = 0; sr < rows-1; sr++) {
        for (int sc = 0; sc < cols; sc++) {
            int u = sc - ox, v = sr - oy;
            bool cl = (safe_mod(u*DM_IH + v*DM_IW, mod) == 0);
            bool rl = (safe_mod(v*DM_IW - u*DM_IH, mod) == 0);
            if (!cl && !rl) continue;
            mvaddch(sr, sc, (chtype)(cl && rl ? '+' : (cl ? '/' : '\\')));
        }
    }
}

/* Isometric grid: same diamond trick, but the cells are stretched wide (2:1)
   to give that game-map "viewed from an angle" look. */
static void bg_draw_iso(const GridCtx *g)
{
    int rows = g->rows, cols = g->cols, ox = g->ox, oy = g->oy;
    int mod = 2 * IS_IW * IS_IH;  /* = 32 */

    for (int sr = 0; sr < rows-1; sr++) {
        for (int sc = 0; sc < cols; sc++) {
            int u = sc - ox, v = sr - oy;
            bool cl = (safe_mod(u*IS_IH + v*IS_IW, mod) == 0);
            bool rl = (safe_mod(v*IS_IW - u*IS_IH, mod) == 0);
            if (!cl && !rl) continue;
            mvaddch(sr, sc, (chtype)(cl && rl ? '+' : (cl ? '/' : '\\')));
        }
    }
}

/* A plain grid with diagonals laid over it too, so every cell has an X in it. */
static void bg_draw_cross(const GridCtx *g)
{
    int rows = g->rows, cols = g->cols, cw = g->cw, ch = g->ch;
    int step_a = cw, step_b = ch;  /* diagonal family spacings */

    for (int sr = 0; sr < rows-1; sr++) {
        for (int sc = 0; sc < cols; sc++) {
            bool hl = (sr % ch == 0), vl = (sc % cw == 0);
            bool sl = ((sc + sr) % step_a == 0);
            bool bl = (safe_mod(sc - sr, step_b) == 0);
            if (!hl && !vl && !sl && !bl) continue;
            char c;
            if      (hl && vl) c = '+';
            else if (hl)       c = '-';
            else if (vl)       c = '|';
            else if (sl && bl) c = 'X';
            else if (sl)       c = '/';
            else               c = '\\';
            mvaddch(sr, sc, (chtype)(unsigned char)c);
        }
    }
}

/* Grid lines plus filled-in squares on every other cell, like a chessboard. */
static void bg_draw_check(const GridCtx *g)
{
    int rows = g->rows, cols = g->cols, cw = g->cw, ch = g->ch;

    for (int sr = 0; sr < rows-1; sr++) {
        for (int sc = 0; sc < cols; sc++) {
            bool hl = (sr % ch == 0), vl = (sc % cw == 0);
            if (hl || vl) {
                mvaddch(sr, sc, (chtype)(hl && vl ? '+' : (hl ? '-' : '|')));
            } else {
                int r = sr/ch, c_ = sc/cw;
                if ((r + c_) % 2 == 1)
                    mvaddch(sr, sc, (chtype)'#');
            }
        }
    }
}

/* Just horizontal lines, like ruled notebook paper — no columns. */
static void bg_draw_ruled(const GridCtx *g)
{
    int rows = g->rows, cols = g->cols, ch = g->ch;

    for (int sr = 0; sr < rows-1; sr++) {
        if (sr % ch != 0) continue;
        for (int sc = 0; sc < cols; sc++)
            mvaddch(sr, sc, (chtype)'-');
    }
}

/* Just a dot where each grid corner would be — the lines are left to the eye. */
static void bg_draw_dot(const GridCtx *g)
{
    int rows = g->rows, cols = g->cols, cw = g->cw, ch = g->ch;

    for (int sr = 0; sr < rows-1; sr++) {
        if (sr % ch != 0) continue;
        for (int sc = 0; sc < cols; sc++) {
            if (sc % cw == 0)
                mvaddch(sr, sc, (chtype)'*');
        }
    }
}

/* Draw whichever grid is active by handing off to its drawer. The five plain
   rectangular styles share one drawer since they only differ in cell size. */
static void draw_grid(const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_GRID));
    switch (g->mode) {
        case GM_UNIFORM: case GM_SQUARE: case GM_FINE:
        case GM_COARSE:  case GM_ORIGIN:  bg_draw_rect_family(g); break;
        case GM_HIER:                     bg_draw_hier       (g); break;
        case GM_BRICK_H:                  bg_draw_brick_h    (g); break;
        case GM_BRICK_V:                  bg_draw_brick_v    (g); break;
        case GM_DIAMOND:                  bg_draw_diamond    (g); break;
        case GM_ISO:                      bg_draw_iso        (g); break;
        case GM_CROSS:                    bg_draw_cross      (g); break;
        case GM_CHECK:                    bg_draw_check      (g); break;
        case GM_RULED:                    bg_draw_ruled      (g); break;
        case GM_DOT:                      bg_draw_dot        (g); break;
        default: break;
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ── §5 pool ── */

/* Obj — one placed object. r,c is its cell on the grid; glyph is the character
   we draw for it; alive says whether this slot is in use. */
typedef struct { int r, c; char glyph; bool alive; } Obj;

/* Pool — every object we've placed, kept in one fixed array so we never
   allocate while running. items[0..count-1] are the live ones; the rest is
   spare room. Removing an object fills its gap with the last one, so the live
   entries always stay packed at the front. */
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

static int pool_find(const Pool *p, int r, int c)
{
    for (int i = 0; i < p->count; i++)
        if (p->items[i].alive && p->items[i].r == r && p->items[i].c == c)
            return i;
    return -1;
}

static void pool_place(Pool *p, int r, int c, char glyph)
{
    if (pool_find(p, r, c) >= 0) return;
    if (p->count >= MAX_OBJ) return;
    p->items[p->count++] = (Obj){ r, c, glyph, true };
}

static void pool_remove(Pool *p, int r, int c)
{
    int i = pool_find(p, r, c);
    if (i < 0) return;
    p->items[i] = p->items[--p->count];
}

static void pool_toggle(Pool *p, int r, int c, char glyph)
{
    if (pool_find(p, r, c) >= 0) pool_remove(p, r, c);
    else                          pool_place(p, r, c, glyph);
}

static void pool_clear(Pool *p) { p->count = 0; }

static void pool_draw(const Pool *p, const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        int sr, sc;
        cell_to_screen(g, p->items[i].r, p->items[i].c, &sr, &sc);
        /* nudge inside the cell so the object sits in the open space, not on
           top of the grid lines. The slanted grids draw at the centre already,
           so they skip this. */
        if (g->mode != GM_DIAMOND && g->mode != GM_ISO && g->mode != GM_RULED) {
            sr += (g->ch > 1 ? 1 : 0);
            sc += (g->cw > 1 ? 1 : 0);
        }
        if (sr >= 0 && sr < g->rows-1 && sc >= 0 && sc < g->cols)
            mvaddch(sr, sc, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
}

/* ── §6 cursor ── */

/* Cursor — where you're pointing right now, as a cell (row, col). SPACE drops
   or removes an object at this spot. */
typedef struct { int r, c; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->r = (g->min_r + g->max_r) / 2;
    cur->c = (g->min_c + g->max_c) / 2;
}

static void cursor_move(Cursor *cur, const GridCtx *g, int dr, int dc)
{
    int nr = cur->r + dr, nc = cur->c + dc;
    if (nr >= g->min_r && nr <= g->max_r) cur->r = nr;
    if (nc >= g->min_c && nc <= g->max_c) cur->c = nc;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    cell_to_screen(g, cur->r, cur->c, &sr, &sc);
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD | A_REVERSE);

    if (g->mode == GM_DIAMOND || g->mode == GM_ISO) {
        /* slanted grids have no boxy cell to fill, so just mark the centre */
        if (sr >= 0 && sr < g->rows-1 && sc >= 0 && sc < g->cols)
            mvaddch(sr, sc, (chtype)'@');
    } else if (g->mode == GM_RULED) {
        /* one line, no columns, so mark a single spot on it */
        if (sr >= 0 && sr < g->rows-1 && sc >= 0 && sc < g->cols)
            mvaddch(sr, sc, (chtype)'@');
    } else {
        /* light up the whole cell so it's obvious which one you're on */
        for (int dr2 = 1; dr2 < g->ch && sr+dr2 < g->rows-1; dr2++)
            for (int dc2 = 1; dc2 < g->cw && sc+dc2 < g->cols; dc2++)
                mvaddch(sr+dr2, sc+dc2, (chtype)' ');
    }

    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD | A_REVERSE);
}

/* ── §7 scene ── */

/* The overlay text: fps and grid info up top, the key reminders along the
   bottom. Kept bright and bold so it stays readable over any grid. */
static void hud_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                     double fps)
{
    char buf[96];
    snprintf(buf, sizeof buf, " %.1f fps  %s  r=%d c=%d  objs=%d ",
             fps, gm_name[g->mode], cur->r, cur->c, p->count);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " arrows:move  spc:toggle  C:clear  r:reset  q:quit"
        "  a:prev-grid  e:next-grid  [%s] ", gm_name[g->mode]);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                       double fps)
{
    erase();
    draw_grid(g);
    pool_draw(p, g);
    cursor_draw(cur, g);
    hud_draw(g, p, cur, fps);
    wnoutrefresh(stdscr); doupdate();
}

/* ── §8 screen ── */

static void screen_cleanup(void) { endwin(); }
static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(); atexit(screen_cleanup);
}

/* ── §9 app ── */

static volatile sig_atomic_t g_running=1, g_need_resize=0;
static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running=0;
    if (s == SIGWINCH)               g_need_resize=1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    screen_init();

    int rows = LINES, cols = COLS;
    GridCtx ctx;  ctx_init(&ctx, GM_UNIFORM, rows, cols);
    Cursor cur;   cursor_reset(&cur, &ctx);
    Pool pool;    pool_clear(&pool);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double fps = TARGET_FPS; int64_t t0 = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            rows = LINES; cols = COLS;
            ctx_init(&ctx, ctx.mode, rows, cols);
            cursor_reset(&cur, &ctx);
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'r': cursor_reset(&cur, &ctx); break;
        case 'C': pool_clear(&pool); break;
        case 'a': { GridMode m=(GridMode)((ctx.mode-1+GM_COUNT)%GM_COUNT);
                    ctx_init(&ctx,m,rows,cols); cursor_reset(&cur,&ctx); } break;
        case 'e': { GridMode m=(GridMode)((ctx.mode+1)%GM_COUNT);
                    ctx_init(&ctx,m,rows,cols); cursor_reset(&cur,&ctx); } break;
        case ' ': pool_toggle(&pool, cur.r, cur.c, 'O'); break;
        case KEY_UP:    cursor_move(&cur, &ctx, -1,  0); break;
        case KEY_DOWN:  cursor_move(&cur, &ctx, +1,  0); break;
        case KEY_LEFT:  cursor_move(&cur, &ctx,  0, -1); break;
        case KEY_RIGHT: cursor_move(&cur, &ctx,  0, +1); break;
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9/(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;
        scene_draw(&ctx, &pool, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
