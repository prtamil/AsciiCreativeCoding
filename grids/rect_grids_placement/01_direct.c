/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 01_direct.c — cursor-based direct object placement on all 14 grid types
 *
 * DEMO: Navigate a cursor with arrow keys; press SPACE to toggle an object
 *       at the current cell. Switch between all 14 grid backgrounds with
 *       keys 1-9 and a-e. Shows how a single ObjectPool works across every
 *       grid type — rect, staggered, diamond, isometric, ruled, dot, origin.
 *
 * Study alongside: grids/rect_grids/01_uniform_rect.c (grid formulas),
 *                  02_patterns.c (pattern-fill placement)
 *
 * Section map:
 *   §1 config   — per-mode geometry constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 6 pairs: grid, active cell, cursor, object, HUD, label
 *   §4 gridctx  — GridCtx struct, init, cursor↔screen, draw background
 *   §5 pool     — ObjectPool: place, remove, toggle, query, clear
 *   §6 cursor   — Cursor struct, move, reset
 *   §7 scene    — scene_draw: background + objects + cursor + HUD
 *   §8 screen   — ncurses init/cleanup
 *   §9 app      — signals, main loop
 *
 * Keys:  arrows:move  spc:toggle  C:clear  r:reset  q/ESC:quit
 *        a:prev-grid  e:next-grid
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids_placement/01_direct.c \
 *       -o 01_direct -lncurses
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Direct placement using a movable cursor.  The cursor
 *                  holds (row, col) in GRID space.  SPACE toggles an object
 *                  at that grid address.  Objects are rendered by converting
 *                  grid (r,c) back to screen (sr,sc) using the same formula
 *                  that draws the grid background.
 *
 * Data-structure : ObjectPool — flat array of (r,c,glyph,alive) records.
 *                  Removal swaps the dead slot with the last item (O(1)).
 *                  Capacity MAX_OBJ=256 is far more than fits on a screen.
 *
 * GridContext    : A single struct carries the geometry of whichever of the
 *                  14 grid modes is active.  Switching grids (keys 1-9, a-f)
 *                  re-initialises GridCtx and resets the cursor.  The pool
 *                  is NOT cleared on switch — objects persist across modes
 *                  (they may appear off-screen until you switch back).
 *
 * Rendering      : Two-pass: (1) draw grid background, (2) draw objects,
 *                  (3) draw cursor highlight.  The cursor draws over any
 *                  object at that cell so the player can see where they are.
 *
 * References     :
 *   Object pool pattern — gameprogrammingpatterns.com/object-pool.html
 *   Grid coordinate systems — this project documentation/Architecture.md §4
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Every one of the 14 grid types provides TWO things to the placement layer:
 *   (a) a drawing function that fills the terminal with its background lines
 *   (b) a formula that converts grid (r,c) → screen (sr,sc) — the TOP-LEFT
 *       corner of the cell (for rect grids) or the cell's centre (rotated).
 *
 * The placement layer sits ON TOP and knows nothing about the grid formula
 * internals: it only calls ctx_to_screen() to position glyphs.  This is the
 * same separation as a graphics pipeline: geometry lives in one layer, the
 * raster lives in another.
 *
 * HOW TO THINK ABOUT COORDINATES
 * ────────────────────────────────
 * Two coordinate spaces are in play simultaneously:
 *
 *   GRID space   — (r,c) integers.  The cursor and objects live here.
 *                  Rect grids:    r∈[0,max_r], c∈[0,max_c]
 *                  Rotated grids: r,c ∈ [-RANGE, RANGE] (centred origin)
 *
 *   SCREEN space — (sr,sc) characters.  ncurses lives here.
 *                  Rect:    sr=r*CH, sc=c*CW  (top-left of cell)
 *                  Rotated: sr=oy+(c+r)*IH, sc=ox+(c-r)*IW  (cell centre)
 *
 * The formula ctx_to_screen() is the ONLY place where grid→screen conversion
 * happens.  It is the single seam between the two coordinate systems.
 *
 * OBJECT POOL MECHANICS
 * ──────────────────────
 * Objects are stored in a flat array.  The "alive" flag marks live entries.
 * toggle(r,c): search for existing object at (r,c); if found, kill it (swap
 * with last, decrement count); if not found, append a new entry.
 * This gives O(n) toggle (scanning the pool), O(1) removal.  With n≤256
 * and the pool scanned once per frame anyway, O(n) is acceptable.
 *
 * SWITCHING GRIDS
 * ───────────────
 * Pressing 1-9 / a-f re-initialises GridCtx (new bounds, new formula) and
 * clamps the cursor to the new bounds.  The pool is NOT cleared: objects
 * placed in one grid mode persist when you switch.  They remain at their
 * stored (r,c) values; they may appear off-screen if the new mode uses a
 * smaller grid or different origin.  Press C to clear the pool.
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
#define MAX_OBJ    256       /* object pool capacity */

/* Geometry for each grid mode (kept here so CONCEPTS block can reference) */
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

/* color pair IDs */
#define PAIR_GRID    1
#define PAIR_ACTIVE  2
#define PAIR_CURSOR  3
#define PAIR_OBJ     4
#define PAIR_HUD     5
#define PAIR_LABEL   6

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
    struct timespec r = { .tv_sec=(time_t)(ns/1000000000LL),
                          .tv_nsec=(long)(ns%1000000000LL) };
    nanosleep(&r, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void color_init(void)
{
    start_color(); use_default_colors();
    init_pair(PAIR_GRID,   COLORS>=256 ?  75 : COLOR_CYAN,    -1);
    init_pair(PAIR_ACTIVE, COLORS>=256 ?  82 : COLOR_GREEN,   -1);
    init_pair(PAIR_CURSOR, COLORS>=256 ? 226 : COLOR_YELLOW,  -1);
    init_pair(PAIR_OBJ,    COLORS>=256 ? 214 : COLOR_RED,     -1);
    init_pair(PAIR_HUD,    COLORS>=256 ? 226 : COLOR_YELLOW,  -1);
    init_pair(PAIR_LABEL,  COLORS>=256 ? 252 : COLOR_WHITE,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  gridctx                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridMode — one value per grid type, matching the 14 source files.
 */
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

/*
 * GridCtx — geometry for the current grid mode.
 * cw/ch: cell dimensions (rect grids).
 * ox/oy: screen-space origin (rotated grids, set to centre of terminal).
 * range: ±range is the grid coordinate range for rotated grids.
 * min/max r,c: cursor bounds in grid space.
 */
typedef struct {
    GridMode mode;
    int rows, cols;
    int cw, ch;
    int ox, oy;
    int range;
    int min_r, max_r, min_c, max_c;
} GridCtx;

static void ctx_init(GridCtx *g, GridMode m, int rows, int cols)
{
    memset(g, 0, sizeof *g);
    g->mode = m; g->rows = rows; g->cols = cols;
    g->ox = cols / 2; g->oy = rows / 2;
    switch (m) {
    case GM_UNIFORM:  g->cw=U_CW;  g->ch=U_CH;  break;
    case GM_SQUARE:   g->cw=SQ_CS*2; g->ch=SQ_CS; break;
    case GM_FINE:     g->cw=FN_CW; g->ch=FN_CH; break;
    case GM_COARSE:   g->cw=CO_CW; g->ch=CO_CH; break;
    case GM_HIER:     g->cw=HI_CW; g->ch=HI_CH; break;
    case GM_BRICK_H:  g->cw=BH_CW; g->ch=BH_CH; break;
    case GM_BRICK_V:  g->cw=BV_CW; g->ch=BV_CH; break;
    case GM_DIAMOND:  g->cw=DM_IW; g->ch=DM_IH; g->range=DM_RNG; break;
    case GM_ISO:      g->cw=IS_IW; g->ch=IS_IH; g->range=IS_RNG; break;
    case GM_CROSS:    g->cw=CR_CW; g->ch=CR_CH; break;
    case GM_CHECK:    g->cw=CK_CW; g->ch=CK_CH; break;
    case GM_RULED:    g->ch=RL_LS; break;
    case GM_DOT:      g->cw=DT_CW; g->ch=DT_CH; break;
    case GM_ORIGIN:   g->cw=OR_CW; g->ch=OR_CH; break;
    default: g->cw=8; g->ch=4; break;
    }
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

/*
 * ctx_to_screen — convert grid (r,c) to screen (sr,sc).
 * Returns the top-left character of the cell for rect grids,
 * or the cell centre for rotated grids.
 */
static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
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

/* safe_mod — always non-negative, needed for diagonal line conditions */
static int safe_mod(int a, int b) { return ((a % b) + b) % b; }

/*
 * ctx_draw_bg — draw the grid background for the current mode.
 * All 14 types are dispatched here.  Each case is the drawing core
 * from the corresponding grids/rect_grids/NN_*.c file.
 */
static void ctx_draw_bg(const GridCtx *g)
{
    int rows = g->rows, cols = g->cols;
    int cw = g->cw, ch = g->ch;
    int ox = g->ox, oy = g->oy;

    attron(COLOR_PAIR(PAIR_GRID));

    switch (g->mode) {

    case GM_UNIFORM: case GM_SQUARE: case GM_FINE:
    case GM_COARSE:  case GM_ORIGIN:
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
        break;

    case GM_HIER: {
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
        break;
    }

    case GM_BRICK_H: {
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
        break;
    }

    case GM_BRICK_V: {
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
        break;
    }

    case GM_DIAMOND: {
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
        break;
    }

    case GM_ISO: {
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
        break;
    }

    case GM_CROSS: {
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
        break;
    }

    case GM_CHECK:
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
        break;

    case GM_RULED:
        for (int sr = 0; sr < rows-1; sr++) {
            if (sr % ch != 0) continue;
            for (int sc = 0; sc < cols; sc++)
                mvaddch(sr, sc, (chtype)'-');
        }
        break;

    case GM_DOT:
        for (int sr = 0; sr < rows-1; sr++) {
            if (sr % ch != 0) continue;
            for (int sc = 0; sc < cols; sc++) {
                if (sc % cw == 0)
                    mvaddch(sr, sc, (chtype)'*');
            }
        }
        break;

    default: break;
    }

    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int r, c; char glyph; bool alive; } Obj;
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
        ctx_to_screen(g, p->items[i].r, p->items[i].c, &sr, &sc);
        /* place object one row/col inside the cell for rect grids */
        if (g->mode != GM_DIAMOND && g->mode != GM_ISO && g->mode != GM_RULED) {
            sr += (g->ch > 1 ? 1 : 0);
            sc += (g->cw > 1 ? 1 : 0);
        }
        if (sr >= 0 && sr < g->rows-1 && sc >= 0 && sc < g->cols)
            mvaddch(sr, sc, (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJ) | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

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
    ctx_to_screen(g, cur->r, cur->c, &sr, &sc);
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD | A_REVERSE);

    if (g->mode == GM_DIAMOND || g->mode == GM_ISO) {
        /* highlight the single centre character */
        if (sr >= 0 && sr < g->rows-1 && sc >= 0 && sc < g->cols)
            mvaddch(sr, sc, (chtype)'@');
    } else if (g->mode == GM_RULED) {
        /* highlight one character wide on the ruled line */
        if (sr >= 0 && sr < g->rows-1 && sc >= 0 && sc < g->cols)
            mvaddch(sr, sc, (chtype)'@');
    } else {
        /* fill interior of cell with reverse-video spaces */
        for (int dr2 = 1; dr2 < g->ch && sr+dr2 < g->rows-1; dr2++)
            for (int dc2 = 1; dc2 < g->cw && sc+dc2 < g->cols; dc2++)
                mvaddch(sr+dr2, sc+dc2, (chtype)' ');
    }

    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD | A_REVERSE);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                       double fps)
{
    int rows = g->rows, cols = g->cols;
    erase();
    ctx_draw_bg(g);
    pool_draw(p, g);
    cursor_draw(cur, g);

    char buf[96];
    snprintf(buf, sizeof buf, " %.1f fps  %s  r=%d c=%d  objs=%d ",
             fps, gm_name[g->mode], cur->r, cur->c, p->count);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);
    mvprintw(rows-1, 0, " %-12s", gm_name[g->mode]);
    attroff(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);
    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(rows-1, 13,
        " arrows:move  spc:toggle  C:clear  r:reset  q:quit"
        "  a:prev-grid  e:next-grid ");
    attroff(COLOR_PAIR(PAIR_LABEL));

    wnoutrefresh(stdscr); doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  screen                                                              */
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
/* §9  app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

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
        fps = fps*0.95 + (1e9/(now - t0 + 1))*0.05; t0 = now;
        scene_draw(&ctx, &pool, &cur, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
