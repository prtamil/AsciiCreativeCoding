/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_path.c — two-point path drawing on all 14 grid types
 *
 * DEMO: Press ENTER to set a start point, move the cursor, press ENTER again
 *       to set the end point.  Then press L=line, P=L-path, O=ring to draw
 *       a path between the two points.  Works on all 14 grid backgrounds.
 *
 * Study alongside: 02_patterns.c (single-point patterns), 01_direct.c
 *
 * Section map:
 *   §1 config   — per-mode geometry
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 6 pairs
 *   §4 gridctx  — GridCtx, ctx_init, ctx_to_screen, ctx_draw_bg
 *   §5 pool     — ObjectPool
 *   §6 paths    — Bresenham line, L-path, ring, staircase
 *   §7 cursor   — Cursor + selection state machine
 *   §8 scene    — scene_draw
 *   §9 screen   — ncurses init/cleanup
 *   §10 app     — signals, main loop
 *
 * Keys:  arrows:move  p:set-point  l:line  j:L-path  o:ring  x:diagonal
 *        C:clear  r:reset  q/ESC:quit
 *        a:prev-grid  e:next-grid
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids_placement/03_path.c \
 *       -o 03_path -lncurses
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two-point path generation on a discrete grid.
 *
 *   Bresenham line (L key):
 *     Iterates from (r0,c0) to (r1,c1) placing cells along the
 *     closest integer approximation of the straight line.  Uses the
 *     classic error-accumulation technique: advance the major axis
 *     every step; advance the minor axis when accumulated error ≥ 0.5.
 *     References: Bresenham 1965 (IBM Systems Journal 4(1):25-30).
 *
 *   L-path (P key):
 *     A rectilinear path: move all the way in one axis first, then
 *     in the other.  Two variants exist (horizontal-first vs vertical-
 *     first); the shorter total Manhattan distance variant is chosen.
 *     L-paths are used in PCB routing and maze solvers.
 *
 *   Ring (O key):
 *     The hollow square (axis-aligned rectangle border) with corners
 *     at (r0,c0) and (r1,c1).  Four sides: top, bottom, left, right.
 *     Same as PAT_HOLLOW from 02_patterns.c but with user-defined corners.
 *
 *   Diagonal staircase (D key):
 *     Alternates row and column steps to produce a 45° diagonal.
 *     Each step moves by (sign(dr), sign(dc)) until the target is reached.
 *
 * Data-structure : Same ObjectPool as 01_direct.c.  All path generators
 *                  call pool_place() for each cell in the path.
 *
 * References     :
 *   Bresenham's line algorithm — en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
 *   Rectilinear routing — en.wikipedia.org/wiki/Rectilinear_polygon
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA — TWO-PHASE INTERACTION
 * ──────────────────────────────────
 * The user interaction has a THREE-STATE machine:
 *
 *   IDLE:       No points selected.  Press ENTER to set point A.
 *   ONE_POINT:  Point A is set.  Move cursor to point B, press ENTER.
 *   TWO_POINTS: Both points set.  Press L/P/O/D to draw a path between them.
 *               After drawing, state resets to IDLE.
 *
 * The selected points are shown with distinct markers ('A' and 'B').
 * The cursor shows '@' at all times.
 *
 * BRESENHAM'S ALGORITHM (the core of this file)
 * ──────────────────────────────────────────────
 * To draw a line from (r0,c0) to (r1,c1):
 *   1. Compute deltas: dr=|r1-r0|, dc=|c1-c0|.
 *   2. The axis with the LARGER delta is the "major" axis — it steps every
 *      iteration.  The axis with the smaller delta is "minor" — it steps
 *      only when an accumulated error term reaches a threshold.
 *   3. Error starts at 2*minor_delta - major_delta.  Each major step adds
 *      2*minor_delta.  When error > 0, take a minor step and subtract
 *      2*major_delta.
 *
 * This produces exactly one pixel per major-axis step — no gaps, no doubles.
 * The result is the closest approximation of a straight line on a grid.
 *
 * RING (RECTANGLE BORDER)
 * ────────────────────────
 * Given corners A=(r0,c0) and B=(r1,c1), the ring consists of:
 *   Top and bottom rows: c from min_c to max_c at r0 and r1.
 *   Left and right cols: r from min_r to max_r at c0 and c1.
 * No interior cells are placed (unlike PAT_FILL).
 *
 * KEY FORMULAS
 * ────────────
 * path_line — Bresenham's error accumulation:
 *   dr = |r1−r0|,  dc = |c1−c0|,  sr = sign(r1−r0),  sc = sign(c1−c0)
 *   Column-major (dc >= dr):
 *     err = 2×dr − dc
 *     for i in [0, dc]: place(r,c); c+=sc
 *       if err > 0: r+=sr; err −= 2×dc
 *       err += 2×dr
 *   Row-major (dr > dc): symmetric with dr/dc swapped.
 *   Invariant: err tracks 2×(minor_steps×major_delta − major_steps×minor_delta).
 *   When err > 0 the path overshoots; step minor axis and subtract 2×major.
 *
 * path_lpath — L-shaped rectilinear path:
 *   Leg 1 (horizontal): c from c0 to c1, r fixed at r0
 *   Leg 2 (vertical):   r from r0 to r1, c fixed at c1
 *   Total unique cells = |dc|+1 + |dr|+1 − 1 = |dc|+|dr|+1
 *   (minus 1 avoids double-counting the corner at (r0, c1))
 *
 * path_ring — hollow rectangle:
 *   rmin=min(r0,r1), rmax=max(r0,r1), cmin=min(c0,c1), cmax=max(c0,c1)
 *   Top row:    r=rmin, c ∈ [cmin, cmax]
 *   Bottom row: r=rmax, c ∈ [cmin, cmax]
 *   Left col:   c=cmin, r ∈ [rmin+1, rmax−1]
 *   Right col:  c=cmax, r ∈ [rmin+1, rmax−1]
 *   Total cells = 2×(|c1−c0|+1) + 2×(|r1−r0|−1) = 2×|dc| + 2×|dr|
 *
 * HOW TO VERIFY
 * ─────────────
 * Uniform grid (U_CW=8, U_CH=4), terminal 80×24.
 *
 * path_line from A=(1,1) to B=(3,5):
 *   dr=2, dc=4 → column-major; err_init=2×2−4=0; sc=+1, sr=+1
 *   i=0: place(1,1); err=0 (not>0); c=2, err=0+4=4
 *   i=1: err=4>0: r=2, err=4−8=−4; c=3, err=−4+4=0
 *   i=2: err=0 (not>0); c=4, err=0+4=4
 *   i=3: err=4>0: r=3, err=4−8=−4; c=5, err=0
 *   Placed: (1,1),(1,2),(2,3),(2,4),(3,5) — 5 cells  ✓
 *
 * path_ring from A=(1,1) to B=(3,4):
 *   rmin=1,rmax=3,cmin=1,cmax=4
 *   Top (r=1): c=1,2,3,4 — 4 cells
 *   Bottom (r=3): c=1,2,3,4 — 4 cells
 *   Left (c=1): r=2 — 1 cell
 *   Right (c=4): r=2 — 1 cell
 *   Total = 10 = 2×3 + 2×2 ✓
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
    GM_DOT, GM_ORIGIN, GM_COUNT
} GridMode;

static const char *const gm_name[GM_COUNT] = {
    "01 uniform","02 square","03 fine","04 coarse",
    "05 hier","06 brick-h","07 brick-v","08 diamond",
    "09 iso","10 cross","11 check","12 ruled","13 dot","14 origin"
};

typedef struct {
    GridMode mode;
    int rows, cols, cw, ch, ox, oy, range;
    int min_r, max_r, min_c, max_c;
} GridCtx;

static void ctx_init(GridCtx *g, GridMode m, int rows, int cols)
{
    memset(g,0,sizeof *g);
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
    if (m==GM_DIAMOND||m==GM_ISO) {
        g->min_r=-g->range; g->max_r=g->range;
        g->min_c=-g->range; g->max_c=g->range;
    } else if (m==GM_RULED) {
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
            bool hl=(sr%ch==0),vl=(sc%cw==0); if(!hl&&!vl) continue;
            char c=(hl&&vl)?'+': (hl?'-':'|');
            if(g->mode==GM_ORIGIN&&sr==oy) c=(vl?'+':'=');
            if(g->mode==GM_ORIGIN&&sc==ox) c=(hl?'+':'I');
            mvaddch(sr,sc,(chtype)(unsigned char)c); } break;
    case GM_HIER: {
        int major=cw*2, semi=cw;
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            bool hm=(sr%major==0),hs=(!hm&&sr%semi==0),hmi=(!hm&&!hs&&sr%ch==0);
            bool vm=(sc%major==0),vs=(!vm&&sc%semi==0),vmi=(!vm&&!vs&&sc%cw==0);
            if(!hm&&!hs&&!hmi&&!vm&&!vs&&!vmi) continue;
            bool hl=hm||hs||hmi, vl=vm||vs||vmi;
            char c; if(hl&&vl) c='+'; else if(hl) c=(hm?'=':(hs?'-':'.')); else c=(vm?'#':(vs?'|':':'));
            mvaddch(sr,sc,(chtype)(unsigned char)c); } break; }
    case GM_BRICK_H: { int half=cw/2;
        for (int sr=0; sr<rows-1; sr++) { bool hl=(sr%ch==0); int rb=sr/ch;
            for (int sc=0; sc<cols; sc++) { bool vl=((sc+(rb%2)*half)%cw==0);
                if(!hl&&!vl) continue;
                mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|'))); } } break; }
    case GM_BRICK_V: { int half=ch/2;
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            int cb=sc/cw; bool hl=((sr+(cb%2)*half)%ch==0),vl=(sc%cw==0);
            if(!hl&&!vl) continue;
            mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|'))); } break; }
    case GM_DIAMOND: { int mod=2*DM_IW*DM_IH;
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            int u=sc-ox,v=sr-oy;
            bool cl=(safe_mod(u*DM_IH+v*DM_IW,mod)==0),rl=(safe_mod(v*DM_IW-u*DM_IH,mod)==0);
            if(!cl&&!rl) continue;
            mvaddch(sr,sc,(chtype)(cl&&rl?'+': (cl?'/':'\\'))); } break; }
    case GM_ISO: { int mod=2*IS_IW*IS_IH;
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            int u=sc-ox,v=sr-oy;
            bool cl=(safe_mod(u*IS_IH+v*IS_IW,mod)==0),rl=(safe_mod(v*IS_IW-u*IS_IH,mod)==0);
            if(!cl&&!rl) continue;
            mvaddch(sr,sc,(chtype)(cl&&rl?'+': (cl?'/':'\\'))); } break; }
    case GM_CROSS: { int sa=cw,sb=ch;
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            bool hl=(sr%ch==0),vl=(sc%cw==0),sl=((sc+sr)%sa==0),bl=(safe_mod(sc-sr,sb)==0);
            if(!hl&&!vl&&!sl&&!bl) continue;
            char c; if(hl&&vl) c='+'; else if(hl) c='-'; else if(vl) c='|';
                    else if(sl&&bl) c='X'; else if(sl) c='/'; else c='\\';
            mvaddch(sr,sc,(chtype)(unsigned char)c); } break; }
    case GM_CHECK:
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            bool hl=(sr%ch==0),vl=(sc%cw==0);
            if(hl||vl) mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|')));
            else if(((sr/ch)+(sc/cw))%2==1) mvaddch(sr,sc,(chtype)'#'); } break;
    case GM_RULED:
        for (int sr=0; sr<rows-1; sr++) { if(sr%ch!=0) continue;
            for (int sc=0; sc<cols; sc++) mvaddch(sr,sc,(chtype)'-'); } break;
    case GM_DOT:
        for (int sr=0; sr<rows-1; sr++) { if(sr%ch!=0) continue;
            for (int sc=0; sc<cols; sc++) if(sc%cw==0) mvaddch(sr,sc,(chtype)'*'); } break;
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
        if (p->items[i].alive&&p->items[i].r==r&&p->items[i].c==c) return i;
    return -1;
}
static void pool_place(Pool *p, int r, int c, char glyph)
{
    if (pool_find(p,r,c)>=0||p->count>=MAX_OBJ) return;
    p->items[p->count++]=(Obj){r,c,glyph,true};
}
static void pool_clear(Pool *p) { p->count=0; }

static void pool_draw(const Pool *p, const GridCtx *g)
{
    attron(COLOR_PAIR(PAIR_OBJ)|A_BOLD);
    for (int i=0; i<p->count; i++) {
        if (!p->items[i].alive) continue;
        int sr,sc; ctx_to_screen(g,p->items[i].r,p->items[i].c,&sr,&sc);
        if (g->mode!=GM_DIAMOND&&g->mode!=GM_ISO&&g->mode!=GM_RULED)
            { sr+=(g->ch>1?1:0); sc+=(g->cw>1?1:0); }
        if (sr>=0&&sr<g->rows-1&&sc>=0&&sc<g->cols)
            mvaddch(sr,sc,(chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJ)|A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  paths                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int iabs(int x) { return x<0?-x:x; }
static int isign(int x) { return x>0?1:x<0?-1:0; }

/*
 * path_line — Bresenham's line from (r0,c0) to (r1,c1).
 * The major axis advances every step; the minor axis advances when the
 * accumulated error crosses zero.
 */
static void path_line(Pool *p, const GridCtx *g,
                      int r0, int c0, int r1, int c1, char glyph)
{
    int dr=iabs(r1-r0), dc=iabs(c1-c0);
    int sr=isign(r1-r0), sc_=isign(c1-c0);
    int r=r0, c=c0;

    if (dc >= dr) {                      /* column-major */
        int err = 2*dr - dc;
        for (int i=0; i<=dc; i++) {
            if (r>=g->min_r&&r<=g->max_r&&c>=g->min_c&&c<=g->max_c)
                pool_place(p,r,c,glyph);
            if (err > 0) { r+=sr; err-=2*dc; }
            err += 2*dr; c+=sc_;
        }
    } else {                             /* row-major */
        int err = 2*dc - dr;
        for (int i=0; i<=dr; i++) {
            if (r>=g->min_r&&r<=g->max_r&&c>=g->min_c&&c<=g->max_c)
                pool_place(p,r,c,glyph);
            if (err > 0) { c+=sc_; err-=2*dr; }
            err += 2*dc; r+=sr;
        }
    }
}

/*
 * path_lpath — L-shaped rectilinear path.
 * Moves horizontally to (r0,c1), then vertically to (r1,c1).
 */
static void path_lpath(Pool *p, const GridCtx *g,
                       int r0, int c0, int r1, int c1, char glyph)
{
    /* horizontal leg */
    int cs=isign(c1-c0);
    for (int c=c0; c!=c1+cs; c+=cs)
        if (r0>=g->min_r&&r0<=g->max_r&&c>=g->min_c&&c<=g->max_c)
            pool_place(p,r0,c,glyph);
    /* vertical leg */
    int rs=isign(r1-r0);
    for (int r=r0; r!=r1+rs; r+=rs)
        if (r>=g->min_r&&r<=g->max_r&&c1>=g->min_c&&c1<=g->max_c)
            pool_place(p,r,c1,glyph);
}

/*
 * path_ring — hollow rectangle border with corners at (r0,c0) and (r1,c1).
 */
static void path_ring(Pool *p, const GridCtx *g,
                      int r0, int c0, int r1, int c1, char glyph)
{
    int rmin=r0<r1?r0:r1, rmax=r0>r1?r0:r1;
    int cmin=c0<c1?c0:c1, cmax=c0>c1?c0:c1;
    for (int c=cmin; c<=cmax; c++) {      /* top and bottom rows */
        if (rmin>=g->min_r&&c>=g->min_c&&c<=g->max_c) pool_place(p,rmin,c,glyph);
        if (rmax<=g->max_r&&c>=g->min_c&&c<=g->max_c) pool_place(p,rmax,c,glyph);
    }
    for (int r=rmin+1; r<rmax; r++) {     /* left and right cols (avoid corners) */
        if (r>=g->min_r&&r<=g->max_r&&cmin>=g->min_c) pool_place(p,r,cmin,glyph);
        if (r>=g->min_r&&r<=g->max_r&&cmax<=g->max_c) pool_place(p,r,cmax,glyph);
    }
}

/*
 * path_diagonal — staircase diagonal: each step moves (sign_r, sign_c).
 * Produces a 45° staircase of cells.
 */
static void path_diagonal(Pool *p, const GridCtx *g,
                           int r0, int c0, int r1, int c1, char glyph)
{
    int rs=isign(r1-r0), cs=isign(c1-c0);
    int r=r0, c=c0;
    while (r!=r1 || c!=c1) {
        if (r>=g->min_r&&r<=g->max_r&&c>=g->min_c&&c<=g->max_c)
            pool_place(p,r,c,glyph);
        if (r!=r1) r+=rs;
        if (c!=c1) c+=cs;
    }
    if (r>=g->min_r&&r<=g->max_r&&c>=g->min_c&&c<=g->max_c)
        pool_place(p,r,c,glyph);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef enum { SEL_IDLE=0, SEL_ONE, SEL_TWO } SelState;

typedef struct {
    int r, c;          /* current cursor position */
    int ar, ac;        /* point A (first ENTER) */
    int br, bc;        /* point B (second ENTER) */
    SelState state;
} Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->r=(g->min_r+g->max_r)/2;
    cur->c=(g->min_c+g->max_c)/2;
    cur->state=SEL_IDLE;
}
static void cursor_move(Cursor *cur, const GridCtx *g, int dr, int dc)
{
    int nr=cur->r+dr, nc=cur->c+dc;
    if (nr>=g->min_r&&nr<=g->max_r) cur->r=nr;
    if (nc>=g->min_c&&nc<=g->max_c) cur->c=nc;
}

static void mark_at(const GridCtx *g, int r, int c, chtype ch)
{
    int sr,sc; ctx_to_screen(g,r,c,&sr,&sc);
    if (g->mode!=GM_DIAMOND&&g->mode!=GM_ISO&&g->mode!=GM_RULED)
        { sr+=(g->ch>1?1:0); sc+=(g->cw>1?1:0); }
    if (sr>=0&&sr<g->rows-1&&sc>=0&&sc<g->cols) mvaddch(sr,sc,ch);
}

/* Draw '@' cursor, 'A' and 'B' markers when selected */
static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    if (cur->state>=SEL_ONE) {
        attron(COLOR_PAIR(PAIR_ACTIVE)|A_BOLD);
        mark_at(g,cur->ar,cur->ac,(chtype)'A');
        attroff(COLOR_PAIR(PAIR_ACTIVE)|A_BOLD);
    }
    if (cur->state==SEL_TWO) {
        attron(COLOR_PAIR(PAIR_ACTIVE)|A_BOLD);
        mark_at(g,cur->br,cur->bc,(chtype)'B');
        attroff(COLOR_PAIR(PAIR_ACTIVE)|A_BOLD);
    }
    attron(COLOR_PAIR(PAIR_CURSOR)|A_BOLD);
    mark_at(g,cur->r,cur->c,(chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR)|A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static const char *sel_hint(SelState s)
{
    if (s==SEL_IDLE) return "p:set-A";
    if (s==SEL_ONE)  return "A set — p:set-B";
    return              "A+B set — l/j/o/x:draw  p:cancel";
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                       double fps)
{
    int rows=g->rows, cols=g->cols;
    erase();
    ctx_draw_bg(g);
    pool_draw(p,g);
    cursor_draw(cur,g);

    char buf[96];
    snprintf(buf,sizeof buf," %.1f fps  %s  objs=%d  [%s] ",
             fps,gm_name[g->mode],p->count,sel_hint(cur->state));
    attron(COLOR_PAIR(PAIR_HUD)|A_BOLD);
    mvprintw(0,cols-(int)strlen(buf),"%s",buf);
    attroff(COLOR_PAIR(PAIR_HUD)|A_BOLD);

    attron(COLOR_PAIR(PAIR_ACTIVE)|A_BOLD);
    mvprintw(rows-1, 0, " %-12s", gm_name[g->mode]);
    attroff(COLOR_PAIR(PAIR_ACTIVE)|A_BOLD);
    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(rows-1,13,
        " arrows:move  p:set-pt  l:line j:L-path o:ring x:diag"
        "  C:clear  r:reset  q:quit"
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
        case 'C': pool_clear(&pool); cur.state=SEL_IDLE; break;
        case 'a': { GridMode m=(GridMode)((ctx.mode-1+GM_COUNT)%GM_COUNT);
                    ctx_init(&ctx,m,rows,cols); cursor_reset(&cur,&ctx); } break;
        case 'e': { GridMode m=(GridMode)((ctx.mode+1)%GM_COUNT);
                    ctx_init(&ctx,m,rows,cols); cursor_reset(&cur,&ctx); } break;
        case 'p':
            if (cur.state==SEL_IDLE) {
                cur.ar=cur.r; cur.ac=cur.c; cur.state=SEL_ONE;
            } else if (cur.state==SEL_ONE) {
                cur.br=cur.r; cur.bc=cur.c; cur.state=SEL_TWO;
            } else {
                cur.state=SEL_IDLE;
            }
            break;
        case 'l':
            if (cur.state==SEL_TWO)
                { path_line(&pool,&ctx,cur.ar,cur.ac,cur.br,cur.bc,'*'); cur.state=SEL_IDLE; }
            break;
        case 'j':
            if (cur.state==SEL_TWO)
                { path_lpath(&pool,&ctx,cur.ar,cur.ac,cur.br,cur.bc,'*'); cur.state=SEL_IDLE; }
            break;
        case 'o':
            if (cur.state==SEL_TWO)
                { path_ring(&pool,&ctx,cur.ar,cur.ac,cur.br,cur.bc,'*'); cur.state=SEL_IDLE; }
            break;
        case 'x':
            if (cur.state==SEL_TWO)
                { path_diagonal(&pool,&ctx,cur.ar,cur.ac,cur.br,cur.bc,'*'); cur.state=SEL_IDLE; }
            break;
        case KEY_UP:    cursor_move(&cur,&ctx,-1, 0); break;
        case KEY_DOWN:  cursor_move(&cur,&ctx,+1, 0); break;
        case KEY_LEFT:  cursor_move(&cur,&ctx, 0,-1); break;
        case KEY_RIGHT: cursor_move(&cur,&ctx, 0,+1); break;
        }

        int64_t now=clock_ns();
        fps=fps*0.95+(1e9/(now-t0+1))*0.05; t0=now;
        scene_draw(&ctx,&pool,&cur,fps);
        clock_sleep_ns(FRAME_NS-(clock_ns()-now));
    }
    return 0;
}
