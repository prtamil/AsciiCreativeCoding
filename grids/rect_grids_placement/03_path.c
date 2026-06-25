/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_path.c — draw a path between two points on any of 14 grid styles.
 *
 * Pick a start point with 'p', move the cursor, press 'p' again for the end
 * point, then choose a path shape: straight line, L-bend, rectangle outline,
 * or a 45-degree staircase. Switch the background grid with a/e.
 *
 * Sister files: 01_direct.c, 02_patterns.c (single-point placement).
 * The straight-line shape is Bresenham's line (Bresenham 1965, IBM Systems
 * Journal 4(1):25-30).
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
#define MAX_OBJ    512

/* How quickly the on-screen fps number settles. Smaller = steadier reading. */
#define FPS_EWMA_ALPHA  0.05

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
    init_pair(PAIR_GRID,   COLORS>=256 ?  75 : COLOR_CYAN,   -1);
    init_pair(PAIR_ACTIVE, COLORS>=256 ?  82 : COLOR_GREEN,  -1);
    init_pair(PAIR_CURSOR, COLORS>=256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_OBJ,    COLORS>=256 ? 214 : COLOR_RED,    -1);
    init_pair(PAIR_HUD,    COLORS>=256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS>=256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 grid mapping & backgrounds ── */

/* The 14 background grid styles you can cycle through with a/e. GM_COUNT is
 * the tally, used for the array sizes and for wrapping around when cycling. */
typedef enum {
    GM_UNIFORM=0, GM_SQUARE, GM_FINE, GM_COARSE,
    GM_HIER, GM_BRICK_H, GM_BRICK_V, GM_DIAMOND,
    GM_ISO, GM_CROSS, GM_CHECK, GM_RULED,
    GM_DOT, GM_ORIGIN, GM_COUNT
} GridMode;

/* Human-readable label per style, shown in the HUD. Indexed by GridMode. */
static const char *const gm_name[GM_COUNT] = {
    "01 uniform","02 square","03 fine","04 coarse",
    "05 hier","06 brick-h","07 brick-v","08 diamond",
    "09 iso","10 cross","11 check","12 ruled","13 dot","14 origin"
};

/* Everything we need to know about the current grid: which style it is, how
 * big a cell is, where the center sits, and which grid coordinates are legal.
 *   mode             which of the 14 styles
 *   rows, cols       terminal size in characters
 *   cw, ch           one cell's width and height in characters
 *   ox, oy           screen center, used by the rotated (diamond/iso) styles
 *   range            half-size of the rotated styles (they run -range..+range)
 *   min/max_r/c      the box of valid grid coordinates the cursor stays inside
 */
typedef struct {
    GridMode mode;
    int rows, cols, cw, ch, ox, oy, range;
    int min_r, max_r, min_c, max_c;
} GridCtx;

/* One tiny setter per style, each just stamping in that style's cell size. */

static void ctx_geom_uniform (GridCtx *g) { g->cw = U_CW;    g->ch = U_CH;  }
static void ctx_geom_square  (GridCtx *g) { g->cw = SQ_CS*2; g->ch = SQ_CS; }
static void ctx_geom_fine    (GridCtx *g) { g->cw = FN_CW;   g->ch = FN_CH; }
static void ctx_geom_coarse  (GridCtx *g) { g->cw = CO_CW;   g->ch = CO_CH; }
static void ctx_geom_hier    (GridCtx *g) { g->cw = HI_CW;   g->ch = HI_CH; }
static void ctx_geom_brick_h (GridCtx *g) { g->cw = BH_CW;   g->ch = BH_CH; }
static void ctx_geom_brick_v (GridCtx *g) { g->cw = BV_CW;   g->ch = BV_CH; }
/* the rotated styles also need range, since they run from -range to +range */
static void ctx_geom_diamond (GridCtx *g) { g->cw = DM_IW; g->ch = DM_IH; g->range = DM_RNG; }
static void ctx_geom_iso     (GridCtx *g) { g->cw = IS_IW; g->ch = IS_IH; g->range = IS_RNG; }
static void ctx_geom_cross   (GridCtx *g) { g->cw = CR_CW;   g->ch = CR_CH; }
static void ctx_geom_check   (GridCtx *g) { g->cw = CK_CW;   g->ch = CK_CH; }
/* ruled is just horizontal lines, so it never sets a column width */
static void ctx_geom_ruled   (GridCtx *g) { g->ch = RL_LS; }
static void ctx_geom_dot     (GridCtx *g) { g->cw = DT_CW;   g->ch = DT_CH; }
static void ctx_geom_origin  (GridCtx *g) { g->cw = OR_CW;   g->ch = OR_CH; }

/* Works out how far the cursor may roam, which differs by style: rotated
 * grids run -range..+range, ruled grids count whole lines, the rest count
 * whole cells that fit on screen. */
static void ctx_set_bounds(GridCtx *g, GridMode m, int rows, int cols)
{
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

/* Sets up a fresh grid: pick the style's cell size, then its valid bounds. */
static void ctx_init(GridCtx *g, GridMode m, int rows, int cols)
{
    memset(g,0,sizeof *g);
    g->mode=m; g->rows=rows; g->cols=cols;
    g->ox=cols/2; g->oy=rows/2;

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
        default:          g->cw=8; g->ch=4; break;
    }
    ctx_set_bounds(g, m, rows, cols);
}

static void cell_to_screen(const GridCtx *g, int r, int c, int *sr, int *sc)
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

/* Like %, but the answer is never negative — handy for the rotated styles
 * where coordinates go negative and plain % would give a negative remainder. */
static int safe_mod(int a, int b) { return ((a%b)+b)%b; }

/* Background drawers: one per grid style, so each can be read on its own. */

/* Plain grid of lines; the origin style also paints a bold central cross. */
static void bg_draw_rect_family(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch, ox=g->ox, oy=g->oy;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        bool hl=(sr%ch==0),vl=(sc%cw==0); if(!hl&&!vl) continue;
        char c=(hl&&vl)?'+': (hl?'-':'|');
        if(g->mode==GM_ORIGIN&&sr==oy) c=(vl?'+':'=');
        if(g->mode==GM_ORIGIN&&sc==ox) c=(hl?'+':'I');
        mvaddch(sr,sc,(chtype)(unsigned char)c);
    }
}

/* Grid with three thicknesses of line; the character tells you which tier. */
static void bg_draw_hier(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    int major=cw*2, semi=cw;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        bool hm=(sr%major==0),hs=(!hm&&sr%semi==0),hmi=(!hm&&!hs&&sr%ch==0);
        bool vm=(sc%major==0),vs=(!vm&&sc%semi==0),vmi=(!vm&&!vs&&sc%cw==0);
        if(!hm&&!hs&&!hmi&&!vm&&!vs&&!vmi) continue;
        bool hl=hm||hs||hmi, vl=vm||vs||vmi;
        char c; if(hl&&vl) c='+'; else if(hl) c=(hm?'=':(hs?'-':'.')); else c=(vm?'#':(vs?'|':':'));
        mvaddch(sr,sc,(chtype)(unsigned char)c);
    }
}

/* Brick wall: every other row of bricks is shifted half a brick sideways. */
static void bg_draw_brick_h(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    int half=cw/2;
    for (int sr=0; sr<rows-1; sr++) {
        bool hl=(sr%ch==0); int rb=sr/ch;
        for (int sc=0; sc<cols; sc++) {
            bool vl=((sc+(rb%2)*half)%cw==0);
            if(!hl&&!vl) continue;
            mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|')));
        }
    }
}

/* Brick wall turned on its side: every other column is shifted half a brick. */
static void bg_draw_brick_v(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    int half=ch/2;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        int cb=sc/cw; bool hl=((sr+(cb%2)*half)%ch==0),vl=(sc%cw==0);
        if(!hl&&!vl) continue;
        mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|')));
    }
}

/* Diamond grid: two sets of slanted lines crossing like a rotated checkerboard. */
static void bg_draw_diamond(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, ox=g->ox, oy=g->oy;
    int mod=2*DM_IW*DM_IH;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        int u=sc-ox,v=sr-oy;
        bool cl=(safe_mod(u*DM_IH+v*DM_IW,mod)==0),rl=(safe_mod(v*DM_IW-u*DM_IH,mod)==0);
        if(!cl&&!rl) continue;
        mvaddch(sr,sc,(chtype)(cl&&rl?'+': (cl?'/':'\\')));
    }
}

/* Same idea as the diamond grid, but the cells are stretched wide (2:1) to
 * give that flattened, video-game isometric look. */
static void bg_draw_iso(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, ox=g->ox, oy=g->oy;
    int mod=2*IS_IW*IS_IH;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        int u=sc-ox,v=sr-oy;
        bool cl=(safe_mod(u*IS_IH+v*IS_IW,mod)==0),rl=(safe_mod(v*IS_IW-u*IS_IH,mod)==0);
        if(!cl&&!rl) continue;
        mvaddch(sr,sc,(chtype)(cl&&rl?'+': (cl?'/':'\\')));
    }
}

/* Plain grid with two extra sets of diagonals laid on top, forming little Xs. */
static void bg_draw_cross(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    int sa=cw,sb=ch;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        bool hl=(sr%ch==0),vl=(sc%cw==0),sl=((sc+sr)%sa==0),bl=(safe_mod(sc-sr,sb)==0);
        if(!hl&&!vl&&!sl&&!bl) continue;
        char c; if(hl&&vl) c='+'; else if(hl) c='-'; else if(vl) c='|';
                else if(sl&&bl) c='X'; else if(sl) c='/'; else c='\\';
        mvaddch(sr,sc,(chtype)(unsigned char)c);
    }
}

/* Grid lines plus filled-in squares on alternating cells, like a chessboard. */
static void bg_draw_check(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        bool hl=(sr%ch==0),vl=(sc%cw==0);
        if(hl||vl) mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|')));
        else if(((sr/ch)+(sc/cw))%2==1) mvaddch(sr,sc,(chtype)'#');
    }
}

/* Just horizontal lines, like ruled notebook paper — no columns at all. */
static void bg_draw_ruled(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, ch=g->ch;
    for (int sr=0; sr<rows-1; sr++) {
        if(sr%ch!=0) continue;
        for (int sc=0; sc<cols; sc++) mvaddch(sr,sc,(chtype)'-');
    }
}

/* Just a dot at each crossing point — the grid implied, not drawn out. */
static void bg_draw_dot(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    for (int sr=0; sr<rows-1; sr++) {
        if(sr%ch!=0) continue;
        for (int sc=0; sc<cols; sc++) if(sc%cw==0) mvaddch(sr,sc,(chtype)'*');
    }
}

/* Paints the background for whatever style is active. The five plain-grid
 * styles all look the same, so they share one drawer. */
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

/* One placed cell: where it sits and what character shows there.
 *   r, c     its grid coordinates
 *   glyph    the character drawn for it
 *   alive    false once removed; kept in the array so indices stay stable */
typedef struct { int r, c; char glyph; bool alive; } Obj;

/* A fixed-size bag of placed cells. No growing or freeing — we allocate the
 * whole array up front and just track how many slots are used.
 *   items    storage for up to MAX_OBJ cells
 *   count    how many slots are in use (0..MAX_OBJ) */
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
        int sr,sc; cell_to_screen(g,p->items[i].r,p->items[i].c,&sr,&sc);
        if (g->mode!=GM_DIAMOND&&g->mode!=GM_ISO&&g->mode!=GM_RULED)
            { sr+=(g->ch>1?1:0); sc+=(g->cw>1?1:0); }
        if (sr>=0&&sr<g->rows-1&&sc>=0&&sc<g->cols)
            mvaddch(sr,sc,(chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJ)|A_BOLD);
}

/* ── §6 paths ── */

static int iabs(int x) { return x<0?-x:x; }
static int isign(int x) { return x>0?1:x<0?-1:0; }

/* THE DISTINCT STEP — lay objects along a straight line of cells between A and
 * B (Bresenham's line walk). Step one cell along the longer axis each iteration;
 * step the shorter axis only when a running integer error says we've drifted too
 * far off the true line. Gapless, no fractions or floating point. */
static void cell_line(Pool *p, const GridCtx *g,
                      int r0, int c0, int r1, int c1, char glyph)
{
    int dr=iabs(r1-r0), dc=iabs(c1-c0);
    int sr=isign(r1-r0), sc_=isign(c1-c0);
    int r=r0, c=c0;

    if (dc >= dr) {                      /* mostly sideways: step columns */
        int err = 2*dr - dc;
        for (int i=0; i<=dc; i++) {
            if (r>=g->min_r&&r<=g->max_r&&c>=g->min_c&&c<=g->max_c)
                pool_place(p,r,c,glyph);
            if (err > 0) { r+=sr; err-=2*dc; }
            err += 2*dr; c+=sc_;
        }
    } else {                             /* mostly up/down: step rows */
        int err = 2*dc - dr;
        for (int i=0; i<=dr; i++) {
            if (r>=g->min_r&&r<=g->max_r&&c>=g->min_c&&c<=g->max_c)
                pool_place(p,r,c,glyph);
            if (err > 0) { c+=sc_; err-=2*dr; }
            err += 2*dc; r+=sr;
        }
    }
}

/* Draws an L-bend: go all the way across first, then all the way up/down,
 * so the path turns exactly one corner. */
static void path_lpath(Pool *p, const GridCtx *g,
                       int r0, int c0, int r1, int c1, char glyph)
{
    /* across */
    int cs=isign(c1-c0);
    for (int c=c0; c!=c1+cs; c+=cs)
        if (r0>=g->min_r&&r0<=g->max_r&&c>=g->min_c&&c<=g->max_c)
            pool_place(p,r0,c,glyph);
    /* up/down */
    int rs=isign(r1-r0);
    for (int r=r0; r!=r1+rs; r+=rs)
        if (r>=g->min_r&&r<=g->max_r&&c1>=g->min_c&&c1<=g->max_c)
            pool_place(p,r,c1,glyph);
}

/* Draws the outline of a rectangle whose opposite corners are the two points
 * — just the border, nothing filled in. */
static void path_ring(Pool *p, const GridCtx *g,
                      int r0, int c0, int r1, int c1, char glyph)
{
    int rmin=r0<r1?r0:r1, rmax=r0>r1?r0:r1;
    int cmin=c0<c1?c0:c1, cmax=c0>c1?c0:c1;
    for (int c=cmin; c<=cmax; c++) {      /* top and bottom edges */
        if (rmin>=g->min_r&&c>=g->min_c&&c<=g->max_c) pool_place(p,rmin,c,glyph);
        if (rmax<=g->max_r&&c>=g->min_c&&c<=g->max_c) pool_place(p,rmax,c,glyph);
    }
    for (int r=rmin+1; r<rmax; r++) {     /* left and right edges; corners already done */
        if (r>=g->min_r&&r<=g->max_r&&cmin>=g->min_c) pool_place(p,r,cmin,glyph);
        if (r>=g->min_r&&r<=g->max_r&&cmax<=g->max_c) pool_place(p,r,cmax,glyph);
    }
}

/* Draws a 45-degree staircase: each step nudges one row and one column toward
 * the target, until it can't move in a direction anymore. */
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

/* ── §7 cursor ── */

/* How far along picking the two endpoints we are: none yet, one picked, both
 * picked. Each press of 'p' moves to the next stage. */
typedef enum { SEL_IDLE=0, SEL_ONE, SEL_TWO } SelState;

/* The moving cursor plus the two endpoints the user has locked in.
 *   r, c       where the cursor is right now (grid coordinates)
 *   ar, ac     point A, the first endpoint picked
 *   br, bc     point B, the second endpoint picked
 *   state      which picking stage we're in (see SelState) */
typedef struct {
    int r, c;
    int ar, ac;
    int br, bc;
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
    int sr,sc; cell_to_screen(g,r,c,&sr,&sc);
    if (g->mode!=GM_DIAMOND&&g->mode!=GM_ISO&&g->mode!=GM_RULED)
        { sr+=(g->ch>1?1:0); sc+=(g->cw>1?1:0); }
    if (sr>=0&&sr<g->rows-1&&sc>=0&&sc<g->cols) mvaddch(sr,sc,ch);
}

/* Draws the '@' cursor, plus 'A' and 'B' once those endpoints are picked. */
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

/* ── §8 scene ── */

/* The little prompt in the HUD telling the user what to do next. */
static const char *sel_hint(SelState s)
{
    if (s==SEL_IDLE) return "p:set-A";
    if (s==SEL_ONE)  return "A set — p:set-B";
    return              "A+B set — l/j/o/x:draw  p:cancel";
}

/* Top-right status line (fps, grid, what's next) and the key-list along the
 * bottom. Both bold and bright so they stay readable over the grid. */
static void hud_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                     double fps)
{
    char buf[96];
    snprintf(buf,sizeof buf," %.1f fps  %s  objs=%d  [%s] ",
             fps,gm_name[g->mode],p->count,sel_hint(cur->state));
    attron(COLOR_PAIR(PAIR_HUD)|A_BOLD);
    mvprintw(0,g->cols-(int)strlen(buf),"%s",buf);
    attroff(COLOR_PAIR(PAIR_HUD)|A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT)|A_BOLD);
    mvprintw(g->rows-1, 0,
        " arrows:move  p:set-pt  l:line j:L-path o:ring x:diag"
        "  C:clear  r:reset  q:quit"
        "  a:prev-grid  e:next-grid  [%s] ", gm_name[g->mode]);
    attroff(COLOR_PAIR(PAIR_HINT)|A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                       double fps)
{
    erase();
    draw_grid(g);
    pool_draw(p,g);
    cursor_draw(cur,g);
    hud_draw(g, p, cur, fps);
    wnoutrefresh(stdscr); doupdate();
}

/* ── §9 screen ── */

static void screen_cleanup(void) { endwin(); }
static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr,TRUE); nodelay(stdscr,TRUE);
    curs_set(0); typeahead(-1);
    color_init(); atexit(screen_cleanup);
}

/* ── §10 app ── */

static volatile sig_atomic_t g_running=1, g_need_resize=0;
static void on_signal(int s)
{
    if (s==SIGINT||s==SIGTERM) g_running=0;
    if (s==SIGWINCH)           g_need_resize=1;
}

/* Switches to the next or previous grid style and recenters the cursor, since
 * the new style may have different bounds. */
static void switch_grid(GridCtx *ctx, Cursor *cur, int rows, int cols, int delta)
{
    GridMode m = (GridMode)((ctx->mode + delta + GM_COUNT) % GM_COUNT);
    ctx_init(ctx, m, rows, cols);
    cursor_reset(cur, ctx);
}

/* Handles one press of 'p': lock in point A, then point B, then (if both are
 * already set) start over. */
static void cycle_selection(Cursor *cur)
{
    if (cur->state == SEL_IDLE) {
        cur->ar = cur->r; cur->ac = cur->c; cur->state = SEL_ONE;
    } else if (cur->state == SEL_ONE) {
        cur->br = cur->r; cur->bc = cur->c; cur->state = SEL_TWO;
    } else {
        cur->state = SEL_IDLE;
    }
}

/* Draws the chosen path shape, but only once both endpoints are set; the
 * draw keys do nothing before that. Afterwards it clears the selection so the
 * next 'p' begins a fresh pair. PathFn is just "any of the path_* drawers". */
typedef void (*PathFn)(Pool *, const GridCtx *, int, int, int, int, char);
static void try_draw_path(Pool *pool, const GridCtx *ctx, Cursor *cur, PathFn fn)
{
    if (cur->state != SEL_TWO) return;
    fn(pool, ctx, cur->ar, cur->ac, cur->br, cur->bc, '*');
    cur->state = SEL_IDLE;
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
            case 'a': switch_grid(&ctx, &cur, rows, cols, -1); break;
            case 'e': switch_grid(&ctx, &cur, rows, cols, +1); break;
            case 'p': cycle_selection(&cur); break;
            case 'l': try_draw_path(&pool, &ctx, &cur, cell_line);     break;
            case 'j': try_draw_path(&pool, &ctx, &cur, path_lpath);    break;
            case 'o': try_draw_path(&pool, &ctx, &cur, path_ring);     break;
            case 'x': try_draw_path(&pool, &ctx, &cur, path_diagonal); break;
            case KEY_UP:    cursor_move(&cur,&ctx,-1, 0); break;
            case KEY_DOWN:  cursor_move(&cur,&ctx,+1, 0); break;
            case KEY_LEFT:  cursor_move(&cur,&ctx, 0,-1); break;
            case KEY_RIGHT: cursor_move(&cur,&ctx, 0,+1); break;
        }

        int64_t now=clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9/(now-t0+1)) * FPS_EWMA_ALPHA;
        t0 = now;
        scene_draw(&ctx,&pool,&cur,fps);
        clock_sleep_ns(FRAME_NS-(clock_ns()-now));
    }
    return 0;
}
