/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_patterns.c — pattern-fill placement on all 14 grid types
 *
 * DEMO: Move a cursor and press a key to stamp a pattern centred on the
 *       cursor cell.  Patterns: B=border rect, F=solid fill, H=hollow frame,
 *       R=row strip, V=col strip.  Pattern size is tunable with +/-.
 *       Works on all 14 grid backgrounds (keys 1-9, a-f).
 *
 * Study alongside: 01_direct.c (single-cell toggle), 03_path.c (two-point)
 *
 * Section map:
 *   §1 config   — per-mode geometry, pattern-size range
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 6 pairs
 *   §4 gridctx  — GridCtx, ctx_init, ctx_to_screen, ctx_draw_bg (same as 01)
 *   §5 pool     — ObjectPool (same as 01)
 *   §6 patterns — pattern generators: border, fill, hollow, row, col
 *   §7 cursor   — Cursor struct, move, reset, draw
 *   §8 scene    — scene_draw
 *   §9 screen   — ncurses init/cleanup
 *   §10 app     — signals, main loop
 *
 * Keys:  arrows:move  B:border  F:fill  H:hollow  R:row  V:col
 *        spc:stamp  +:grow  -:shrink  C:clear  r:reset  q/ESC:quit
 *        a:prev-grid  e:next-grid
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids_placement/02_patterns.c \
 *       -o 02_patterns -lncurses
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Pattern-fill placement.  Each pattern is a PREDICATE over
 *                  relative offsets (dr,dc) from the cursor: the pattern
 *                  places an object at every (cursor.r+dr, cursor.c+dc) for
 *                  which the predicate returns true.  Five predicates:
 *
 *   border(dr,dc,N): |dr|==N || |dc|==N  — the N-ring perimeter
 *   fill  (dr,dc,N): |dr|<=N && |dc|<=N  — the (2N+1)²  square
 *   hollow(dr,dc,N): fill(N) && !fill(N-1) — only the outermost ring
 *   row   (dr,dc,N): dr==0 && |dc|<=N    — horizontal strip
 *   col   (dr,dc,N): dc==0 && |dr|<=N    — vertical strip
 *
 * Data-structure : Same ObjectPool as 01_direct.c — flat array, swap-last
 *                  removal, O(n) membership test.  Stamping a pattern calls
 *                  pool_place() for each cell in the shape.
 *
 * Rendering      : Background → objects → cursor → HUD.  Cursor shows the
 *                  PREVIEW outline of the pattern before stamping.
 *
 * References     :
 *   Flood-fill and region-fill — en.wikipedia.org/wiki/Flood_fill
 *   Stamp/brush tools in raster editors — Inkscape source, GIMP paintbrush
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA — PATTERNS AS PREDICATES
 * ─────────────────────────────────────
 * A "pattern" is not a fixed bitmap.  It is a RULE that answers the question:
 * "Should a cell at offset (dr,dc) from the anchor be filled?"
 *
 * This representation has three advantages:
 *   1. Patterns scale with a single integer N.
 *   2. Patterns compose: hollow = fill(N) AND NOT fill(N-1).
 *   3. The same predicate works on any grid type — no special-casing.
 *
 * HOW STAMPING WORKS
 * ───────────────────
 * When you press B/F/H/R/V, the program iterates a bounding box of offsets
 * (-N..N, -N..N), evaluates the predicate for each (dr,dc), and calls
 * pool_place() for every cell that passes.  The pool silently ignores
 * duplicates (cells that already have an object).
 *
 * PREVIEW OVERLAY
 * ────────────────
 * Before stamping, the cursor_draw() function shows a preview: it evaluates
 * the SAME predicate and highlights matching cells in a different colour.
 * The user sees exactly what will be stamped before pressing the key.
 *
 * PATTERN SIZE
 * ─────────────
 * N is the "half-size" of the pattern.  N=1 gives a 3×3 region; N=3 gives
 * 7×7.  Keys + and - increment/decrement N in [1, MAX_PAT_N].
 *
 * SWITCHING GRIDS
 * ────────────────
 * The pool persists across grid switches.  Objects placed in one mode remain
 * stored at (r,c) values; they render via ctx_to_screen() using the new mode's
 * formula, so they appear at different screen positions.  Press C to clear.
 *
 * KEY FORMULAS
 * ────────────
 * pat_test — five predicate rules (ar=|dr|, ac=|dc|):
 *
 *   border(dr,dc,N): (ar==N || ac==N) && ar<=N && ac<=N
 *     → cells on the outermost row or column of an N-ring square perimeter
 *     Count = 8N cells
 *
 *   fill(dr,dc,N): ar<=N && ac<=N
 *     → all cells in the (2N+1)×(2N+1) square
 *     Count = (2N+1)²
 *
 *   hollow(dr,dc,N): fill(N) && NOT fill(N−1)
 *     ≡ cells with max(ar, ac) == N  (L∞ distance = N)
 *     Count = (2N+1)² − (2N−1)² = 8N  (same as border)
 *
 *   row(dr,dc,N): dr==0 && ac<=N
 *     → horizontal strip of 2N+1 cells
 *
 *   col(dr,dc,N): dc==0 && ar<=N
 *     → vertical strip of 2N+1 cells
 *
 * Note: border and hollow are numerically equal because for a square both
 * definitions select the same cells (the outermost shell).
 *
 * HOW TO VERIFY
 * ─────────────
 * Uniform grid (U_CW=8, U_CH=4), terminal 80×24.
 * Cursor at (r=2, c=3), N=2.
 *
 * border (N=2): outer ring of a 5×5 box.
 *   Count = 8×2 = 16 cells placed.
 *   Top-left offset (dr=−2, dc=−2) → grid cell (0,1)
 *   screen: sr=0×4+1=1, sc=1×8+1=9  ✓
 *
 * fill (N=1): 3×3 = 9 cells.
 *   Centre (dr=0, dc=0) → grid (2,3); screen: sr=9, sc=25  ✓
 *
 * hollow (N=2): 5×5 minus 3×3 = 25−9 = 16 = 8×2 cells.
 *   Identical to border count — the outermost shell of a square.  ✓
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
#define MAX_OBJ    512
#define MAX_PAT_N   8    /* maximum pattern half-size */
#define MIN_PAT_N   1

/* Per-mode geometry (same values as 01_direct.c) */
#define U_CW  8
#define U_CH  4
#define SQ_CS 3
#define FN_CW 4
#define FN_CH 2
#define CO_CW 12
#define CO_CH 4
#define HI_CW 6
#define HI_CH 3
#define BH_CW 10
#define BH_CH 3
#define BV_CW 4
#define BV_CH 6
#define DM_IW 4
#define DM_IH 2
#define DM_RNG 5
#define IS_IW 8
#define IS_IH 2
#define IS_RNG 4
#define CR_CW 8
#define CR_CH 4
#define CK_CW 6
#define CK_CH 3
#define RL_LS 3
#define DT_CW 6
#define DT_CH 3
#define OR_CW 10
#define OR_CH 4

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
    init_pair(PAIR_GRID,   COLORS>=256 ?  75 : COLOR_CYAN,   -1);
    init_pair(PAIR_ACTIVE, COLORS>=256 ?  82 : COLOR_GREEN,  -1);
    init_pair(PAIR_CURSOR, COLORS>=256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_OBJ,    COLORS>=256 ? 214 : COLOR_RED,    -1);
    init_pair(PAIR_HUD,    COLORS>=256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_LABEL,  COLORS>=256 ? 252 : COLOR_WHITE,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  gridctx                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

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

typedef struct {
    GridMode mode;
    int rows, cols, cw, ch, ox, oy, range;
    int min_r, max_r, min_c, max_c;
} GridCtx;

static void ctx_init(GridCtx *g, GridMode m, int rows, int cols)
{
    memset(g, 0, sizeof *g);
    g->mode=m; g->rows=rows; g->cols=cols;
    g->ox=cols/2; g->oy=rows/2;
    switch (m) {
    case GM_UNIFORM:  g->cw=U_CW;    g->ch=U_CH;  break;
    case GM_SQUARE:   g->cw=SQ_CS*2; g->ch=SQ_CS; break;
    case GM_FINE:     g->cw=FN_CW;   g->ch=FN_CH; break;
    case GM_COARSE:   g->cw=CO_CW;   g->ch=CO_CH; break;
    case GM_HIER:     g->cw=HI_CW;   g->ch=HI_CH; break;
    case GM_BRICK_H:  g->cw=BH_CW;   g->ch=BH_CH; break;
    case GM_BRICK_V:  g->cw=BV_CW;   g->ch=BV_CH; break;
    case GM_DIAMOND:  g->cw=DM_IW;   g->ch=DM_IH; g->range=DM_RNG; break;
    case GM_ISO:      g->cw=IS_IW;   g->ch=IS_IH; g->range=IS_RNG; break;
    case GM_CROSS:    g->cw=CR_CW;   g->ch=CR_CH; break;
    case GM_CHECK:    g->cw=CK_CW;   g->ch=CK_CH; break;
    case GM_RULED:    g->ch=RL_LS;   break;
    case GM_DOT:      g->cw=DT_CW;   g->ch=DT_CH; break;
    case GM_ORIGIN:   g->cw=OR_CW;   g->ch=OR_CH; break;
    default: g->cw=8; g->ch=4; break;
    }
    if (m == GM_DIAMOND || m == GM_ISO) {
        g->min_r=-g->range; g->max_r=g->range;
        g->min_c=-g->range; g->max_c=g->range;
    } else if (m == GM_RULED) {
        g->min_r=0; g->max_r=(rows-1)/g->ch-1;
        g->min_c=0; g->max_c=cols-1;
    } else {
        g->min_r=0; g->max_r=(rows-2)/g->ch;
        g->min_c=0; g->max_c=(cols-1)/g->cw;
    }
}

static void ctx_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
{
    switch (g->mode) {
    case GM_DIAMOND: *sc=g->ox+(c-r)*DM_IW; *sr=g->oy+(c+r)*DM_IH; break;
    case GM_ISO:     *sc=g->ox+(c-r)*IS_IW; *sr=g->oy+(c+r)*IS_IH; break;
    case GM_RULED:   *sr=r*RL_LS; *sc=c; break;
    case GM_BRICK_H: *sr=r*g->ch; *sc=c*g->cw+(r%2)*(g->cw/2); break;
    case GM_BRICK_V: *sr=r*g->ch+(c%2)*(g->ch/2); *sc=c*g->cw; break;
    default:         *sr=r*g->ch; *sc=c*g->cw; break;
    }
}

static int safe_mod(int a, int b) { return ((a%b)+b)%b; }

static void ctx_draw_bg(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    int ox=g->ox, oy=g->oy;
    attron(COLOR_PAIR(PAIR_GRID));
    switch (g->mode) {
    case GM_UNIFORM: case GM_SQUARE: case GM_FINE:
    case GM_COARSE:  case GM_ORIGIN:
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            bool hl=(sr%ch==0), vl=(sc%cw==0);
            if (!hl && !vl) continue;
            char c=(hl&&vl)?'+': (hl?'-':'|');
            if (g->mode==GM_ORIGIN && sr==oy) c=(vl?'+':'=');
            if (g->mode==GM_ORIGIN && sc==ox) c=(hl?'+':'I');
            mvaddch(sr, sc, (chtype)(unsigned char)c);
        } break;
    case GM_HIER: {
        int major=cw*2, semi=cw;
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            bool hm=(sr%major==0), hs=(!hm&&sr%semi==0), hmi=(!hm&&!hs&&sr%ch==0);
            bool vm=(sc%major==0), vs=(!vm&&sc%semi==0), vmi=(!vm&&!vs&&sc%cw==0);
            if (!hm&&!hs&&!hmi&&!vm&&!vs&&!vmi) continue;
            bool hl=hm||hs||hmi, vl=vm||vs||vmi;
            char c; if(hl&&vl) c='+'; else if(hl) c=(hm?'=':(hs?'-':'.')); else c=(vm?'#':(vs?'|':':'));
            mvaddch(sr, sc, (chtype)(unsigned char)c);
        } break;
    }
    case GM_BRICK_H: {
        int half=cw/2;
        for (int sr=0; sr<rows-1; sr++) { bool hl=(sr%ch==0); int rb=sr/ch;
            for (int sc=0; sc<cols; sc++) { bool vl=((sc+(rb%2)*half)%cw==0);
                if (!hl&&!vl) continue;
                mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|'))); } } break;
    }
    case GM_BRICK_V: {
        int half=ch/2;
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            int cb=sc/cw; bool hl=((sr+(cb%2)*half)%ch==0), vl=(sc%cw==0);
            if (!hl&&!vl) continue;
            mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|'))); } break;
    }
    case GM_DIAMOND: {
        int mod=2*DM_IW*DM_IH;
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            int u=sc-ox, v=sr-oy;
            bool cl=(safe_mod(u*DM_IH+v*DM_IW,mod)==0);
            bool rl=(safe_mod(v*DM_IW-u*DM_IH,mod)==0);
            if (!cl&&!rl) continue;
            mvaddch(sr,sc,(chtype)(cl&&rl?'+': (cl?'/':'\\'))); } break;
    }
    case GM_ISO: {
        int mod=2*IS_IW*IS_IH;
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            int u=sc-ox, v=sr-oy;
            bool cl=(safe_mod(u*IS_IH+v*IS_IW,mod)==0);
            bool rl=(safe_mod(v*IS_IW-u*IS_IH,mod)==0);
            if (!cl&&!rl) continue;
            mvaddch(sr,sc,(chtype)(cl&&rl?'+': (cl?'/':'\\'))); } break;
    }
    case GM_CROSS: {
        int sa=cw, sb=ch;
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            bool hl=(sr%ch==0), vl=(sc%cw==0);
            bool sl=((sc+sr)%sa==0), bl=(safe_mod(sc-sr,sb)==0);
            if (!hl&&!vl&&!sl&&!bl) continue;
            char c; if(hl&&vl) c='+'; else if(hl) c='-'; else if(vl) c='|';
                    else if(sl&&bl) c='X'; else if(sl) c='/'; else c='\\';
            mvaddch(sr,sc,(chtype)(unsigned char)c); } break;
    }
    case GM_CHECK:
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            bool hl=(sr%ch==0), vl=(sc%cw==0);
            if (hl||vl) mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|')));
            else if (((sr/ch)+(sc/cw))%2==1) mvaddch(sr,sc,(chtype)'#');
        } break;
    case GM_RULED:
        for (int sr=0; sr<rows-1; sr++) { if (sr%ch!=0) continue;
            for (int sc=0; sc<cols; sc++) mvaddch(sr,sc,(chtype)'-'); } break;
    case GM_DOT:
        for (int sr=0; sr<rows-1; sr++) { if (sr%ch!=0) continue;
            for (int sc=0; sc<cols; sc++) if (sc%cw==0) mvaddch(sr,sc,(chtype)'*'); } break;
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
    for (int i=0; i<p->count; i++)
        if (p->items[i].alive && p->items[i].r==r && p->items[i].c==c) return i;
    return -1;
}
static void pool_place(Pool *p, int r, int c, char glyph)
{
    if (pool_find(p,r,c)>=0 || p->count>=MAX_OBJ) return;
    p->items[p->count++] = (Obj){r,c,glyph,true};
}
static void pool_clear(Pool *p) { p->count=0; }

static void pool_draw(const Pool *p, const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_OBJ)|A_BOLD);
    for (int i=0; i<p->count; i++) {
        if (!p->items[i].alive) continue;
        int sr,sc; ctx_to_screen(g,p->items[i].r,p->items[i].c,&sr,&sc);
        if (g->mode!=GM_DIAMOND && g->mode!=GM_ISO && g->mode!=GM_RULED) {
            sr+=(g->ch>1?1:0); sc+=(g->cw>1?1:0);
        }
        if (sr>=0&&sr<g->rows-1&&sc>=0&&sc<g->cols)
            mvaddch(sr,sc,(chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJ)|A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  patterns                                                            */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef enum { PAT_BORDER=0, PAT_FILL, PAT_HOLLOW, PAT_ROW, PAT_COL } PatMode;

static const char *const pat_name[] = {
    "border","fill","hollow","row","col"
};

/*
 * pat_test — predicate: should offset (dr,dc) be included in pattern pat of
 * half-size N?  Returns true if the cell belongs to the pattern.
 */
static bool pat_test(PatMode pat, int dr, int dc, int N)
{
    int ar = dr<0?-dr:dr, ac = dc<0?-dc:dc;
    switch (pat) {
    case PAT_BORDER:  return (ar==N || ac==N) && ar<=N && ac<=N;
    case PAT_FILL:    return ar<=N && ac<=N;
    case PAT_HOLLOW:  return (ar<=N && ac<=N) && !(ar<N && ac<N);
    case PAT_ROW:     return dr==0 && ac<=N;
    case PAT_COL:     return dc==0 && ar<=N;
    }
    return false;
}

/*
 * pattern_stamp — place objects for all offsets where pat_test is true.
 * Clamps to grid bounds so out-of-range cells are silently skipped.
 */
static void pattern_stamp(Pool *p, const GridCtx *g, int cr, int cc,
                           PatMode pat, int N, char glyph)
{
    for (int dr=-N; dr<=N; dr++) for (int dc=-N; dc<=N; dc++) {
        if (!pat_test(pat,dr,dc,N)) continue;
        int r=cr+dr, c=cc+dc;
        if (r<g->min_r||r>g->max_r||c<g->min_c||c>g->max_c) continue;
        pool_place(p,r,c,glyph);
    }
}

/*
 * preview_draw — highlight cells that WOULD be placed if stamped now.
 * Uses the cursor colour with reverse video so the user sees the preview.
 */
static void preview_draw(const GridCtx *g, int cr, int cc, PatMode pat, int N)
{
    attron(COLOR_PAIR(PAIR_CURSOR)|A_REVERSE);
    for (int dr=-N; dr<=N; dr++) for (int dc=-N; dc<=N; dc++) {
        if (!pat_test(pat,dr,dc,N)) continue;
        int r=cr+dr, c=cc+dc;
        if (r<g->min_r||r>g->max_r||c<g->min_c||c>g->max_c) continue;
        int sr,sc; ctx_to_screen(g,r,c,&sr,&sc);
        if (g->mode!=GM_DIAMOND && g->mode!=GM_ISO && g->mode!=GM_RULED)
            { sr+=(g->ch>1?1:0); sc+=(g->cw>1?1:0); }
        if (sr>=0&&sr<g->rows-1&&sc>=0&&sc<g->cols)
            mvaddch(sr,sc,(chtype)'.');
    }
    attroff(COLOR_PAIR(PAIR_CURSOR)|A_REVERSE);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int r, c; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->r=(g->min_r+g->max_r)/2;
    cur->c=(g->min_c+g->max_c)/2;
}
static void cursor_move(Cursor *cur, const GridCtx *g, int dr, int dc)
{
    int nr=cur->r+dr, nc=cur->c+dc;
    if (nr>=g->min_r&&nr<=g->max_r) cur->r=nr;
    if (nc>=g->min_c&&nc<=g->max_c) cur->c=nc;
}
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr,sc; ctx_to_screen(g,cur->r,cur->c,&sr,&sc);
    attron(COLOR_PAIR(PAIR_CURSOR)|A_BOLD);
    if (sr>=0&&sr<g->rows-1&&sc>=0&&sc<g->cols)
        mvaddch(sr,sc,(chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR)|A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                       PatMode pat, int N, double fps)
{
    int rows=g->rows, cols=g->cols;
    erase();
    ctx_draw_bg(g);
    pool_draw(p,g);
    preview_draw(g,cur->r,cur->c,pat,N);
    cursor_draw(cur,g);

    char buf[96];
    snprintf(buf,sizeof buf," %.1f fps  %s  pat=%s N=%d  objs=%d ",
             fps,gm_name[g->mode],pat_name[pat],N,p->count);
    attron(COLOR_PAIR(PAIR_HUD)|A_BOLD);
    mvprintw(0,cols-(int)strlen(buf),"%s",buf);
    attroff(COLOR_PAIR(PAIR_HUD)|A_BOLD);

    attron(COLOR_PAIR(PAIR_ACTIVE)|A_BOLD);
    mvprintw(rows-1, 0, " %-12s", gm_name[g->mode]);
    attroff(COLOR_PAIR(PAIR_ACTIVE)|A_BOLD);
    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(rows-1,13,
        " arrows:move  B:border F:fill H:hollow R:row V:col"
        "  spc:stamp  +/-:size  C:clear  r:reset  q:quit"
        "  a:prev-grid  e:next-grid ");
    attroff(COLOR_PAIR(PAIR_LABEL));

    wnoutrefresh(stdscr); doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }
static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr,TRUE); nodelay(stdscr,TRUE);
    curs_set(0); typeahead(-1);
    color_init(); atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §10 app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running=1, g_need_resize=0;
static void on_signal(int s)
{
    if (s==SIGINT||s==SIGTERM) g_running=0;
    if (s==SIGWINCH)           g_need_resize=1;
}

int main(void)
{
    signal(SIGINT,on_signal); signal(SIGTERM,on_signal);
    signal(SIGWINCH,on_signal);
    screen_init();

    int rows=LINES, cols=COLS;
    GridCtx ctx; ctx_init(&ctx,GM_UNIFORM,rows,cols);
    Cursor cur;  cursor_reset(&cur,&ctx);
    Pool pool;   pool_clear(&pool);
    PatMode pat=PAT_BORDER;
    int N=2;

    const int64_t FRAME_NS=1000000000LL/TARGET_FPS;
    double fps=TARGET_FPS; int64_t t0=clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize=0; endwin(); refresh();
            rows=LINES; cols=COLS;
            ctx_init(&ctx,ctx.mode,rows,cols);
            cursor_reset(&cur,&ctx);
        }

        int ch=getch();
        switch (ch) {
        case 'q': case 27: g_running=0; break;
        case 'r': cursor_reset(&cur,&ctx); break;
        case 'C': pool_clear(&pool); break;
        case 'a': { GridMode m=(GridMode)((ctx.mode-1+GM_COUNT)%GM_COUNT);
                    ctx_init(&ctx,m,rows,cols); cursor_reset(&cur,&ctx); } break;
        case 'e': { GridMode m=(GridMode)((ctx.mode+1)%GM_COUNT);
                    ctx_init(&ctx,m,rows,cols); cursor_reset(&cur,&ctx); } break;
        case '+': case '=': if (N<MAX_PAT_N) N++; break;
        case '-':           if (N>MIN_PAT_N) N--; break;
        case 'B': pat=PAT_BORDER; break;
        case 'F': pat=PAT_FILL;   break;
        case 'H': pat=PAT_HOLLOW; break;
        case 'R': pat=PAT_ROW;    break;
        case 'V': pat=PAT_COL;    break;
        case ' ': pattern_stamp(&pool,&ctx,cur.r,cur.c,pat,N,'O'); break;
        case KEY_UP:    cursor_move(&cur,&ctx,-1, 0); break;
        case KEY_DOWN:  cursor_move(&cur,&ctx,+1, 0); break;
        case KEY_LEFT:  cursor_move(&cur,&ctx, 0,-1); break;
        case KEY_RIGHT: cursor_move(&cur,&ctx, 0,+1); break;
        }

        int64_t now=clock_ns();
        fps=fps*0.95+(1e9/(now-t0+1))*0.05; t0=now;
        scene_draw(&ctx,&pool,&cur,pat,N,fps);
        clock_sleep_ns(FRAME_NS-(clock_ns()-now));
    }
    return 0;
}
