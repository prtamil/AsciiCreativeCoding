/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 04_scatter.c — scatter objects across a grid four different ways.
 *
 * Move the @ cursor and press a key to drop a crowd of 'o' objects onto any
 * of 14 grid backgrounds. The four scatter styles differ in how they pick
 * which cells get an object: pure random, evenly-spaced, a spreading flood
 * from the cursor, or a cloud that's densest at the middle.
 *
 * Study alongside: 01_direct.c (one object at a time), 02_patterns.c (stamps).
 *
 * The four styles, in a sentence each:
 *   Random   — toss N objects at random cells; you'll get clumps and gaps.
 *   Min-dist — same toss, but throw away any pick too close to one already
 *              down. Gives the even, "blue-noise" look of Poisson-disk
 *              sampling (Bridson 2007). We brute-force the distance check,
 *              which is fine for a few hundred objects.
 *   Flood    — spread outward from the cursor one ring at a time, like a
 *              breadth-first search, filling cells as we reach them.
 *   Gradient — walk every cell and roll the dice; cells near the grid centre
 *              are more likely to win, so the result is dense in the middle.
 *
 * "Distance" here means Chebyshev distance: how many king-moves apart two
 * cells are, i.e. max(row gap, col gap). It's cheap (no square roots) and its
 * idea of a "circle" is a square, which suits a character grid.
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

#define TARGET_FPS   30
#define MAX_OBJ     256

/* How fast the on-screen fps number reacts: small = smooth, slow to change. */
#define FPS_EWMA_ALPHA  0.05

#define RAND_N       40     /* how many objects the random scatter drops */
#define MAX_TRIES   400     /* how many picks min-dist tries before giving up */
#define MIN_DIST      3     /* min-dist: keep objects this many cells apart */
#define FLOOD_MAX   120     /* flood stops after filling this many cells */
#define GRAD_SCALE    6     /* gradient: bigger = the dense middle spreads wider */

/* Cell size in characters for each grid style (same numbers as 01_direct.c). */
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

/* Which of the 14 background grid styles is on screen. GM_COUNT is the total,
 * handy for wrapping when you cycle styles. */
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

/* Everything we need to know to draw and address one grid style:
 * its type, the terminal size, cell width/height, and the valid cell range.
 *
 *   mode               which style this is
 *   rows, cols         terminal size in characters
 *   cw, ch             one cell's width and height in characters
 *   ox, oy             screen centre (origin for the rotated grids)
 *   range              for diamond/iso: cells run from -range to +range
 *   min_r/max_r,
 *   min_c/max_c        the legal cell coordinates; everything else clamps
 *                      cursor moves and scatter picks to these bounds. */
typedef struct {
    GridMode mode;
    int rows, cols, cw, ch, ox, oy, range;
    int min_r, max_r, min_c, max_c;
} GridCtx;

/* One tiny setter per style fills in that style's cell size. */

static void ctx_geom_uniform (GridCtx *g) { g->cw = U_CW;    g->ch = U_CH;  }
static void ctx_geom_square  (GridCtx *g) { g->cw = SQ_CS*2; g->ch = SQ_CS; }
static void ctx_geom_fine    (GridCtx *g) { g->cw = FN_CW;   g->ch = FN_CH; }
static void ctx_geom_coarse  (GridCtx *g) { g->cw = CO_CW;   g->ch = CO_CH; }
static void ctx_geom_hier    (GridCtx *g) { g->cw = HI_CW;   g->ch = HI_CH; }
static void ctx_geom_brick_h (GridCtx *g) { g->cw = BH_CW;   g->ch = BH_CH; }
static void ctx_geom_brick_v (GridCtx *g) { g->cw = BV_CW;   g->ch = BV_CH; }
/* The rotated grids also set `range` so cells run from -range to +range. */
static void ctx_geom_diamond (GridCtx *g) { g->cw = DM_IW; g->ch = DM_IH; g->range = DM_RNG; }
static void ctx_geom_iso     (GridCtx *g) { g->cw = IS_IW; g->ch = IS_IH; g->range = IS_RNG; }
static void ctx_geom_cross   (GridCtx *g) { g->cw = CR_CW;   g->ch = CR_CH; }
static void ctx_geom_check   (GridCtx *g) { g->cw = CK_CW;   g->ch = CK_CH; }
/* Ruled is just horizontal lines, so it has a row height but no column width. */
static void ctx_geom_ruled   (GridCtx *g) { g->ch = RL_LS; }
static void ctx_geom_dot     (GridCtx *g) { g->cw = DT_CW;   g->ch = DT_CH; }
static void ctx_geom_origin  (GridCtx *g) { g->cw = OR_CW;   g->ch = OR_CH; }

/* Work out the legal cell coordinates. Rotated grids count out from the
 * centre (±range); ruled grids count horizontal lines; the rest count whole
 * cells that fit on screen. */
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

/* Set up the whole context for a style: clear it, pick the cell size for the
 * style, then work out the legal coordinate range once. */
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

/* Turn a grid cell (r,c) into a screen spot (sr,sc). Each style lays its cells
 * out differently, so this is the one place that knows the geometry. */
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

/* Remainder that's never negative — handy when coordinates go below zero. */
static int safe_mod(int a, int b) { return ((a%b)+b)%b; }

/* Background drawers: one function per grid style so each can be read on its
 * own. They only paint the faint grid lines behind the scattered objects. */

/* Plain box grid; the "origin" style also draws a bright cross at the centre. */
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

/* Grid with three line weights; the character shows which weight a line is. */
static void bg_draw_hier(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    int major=cw*2, semi=cw;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        bool hm=(sr%major==0),hs=(!hm&&sr%semi==0),hmi=(!hm&&!hs&&sr%ch==0);
        bool vm=(sc%major==0),vs=(!vm&&sc%semi==0),vmi=(!vm&&!vs&&sc%cw==0);
        if(!hm&&!hs&&!hmi&&!vm&&!vs&&!vmi) continue;
        bool hl=hm||hs||hmi,vl=vm||vs||vmi;
        char c; if(hl&&vl) c='+'; else if(hl) c=(hm?'=':(hs?'-':'.')); else c=(vm?'#':(vs?'|':':'));
        mvaddch(sr,sc,(chtype)(unsigned char)c);
    }
}

/* Brick wall: shift every other row sideways by half a brick. */
static void bg_draw_brick_h(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    int half=cw/2;
    for (int sr=0; sr<rows-1; sr++) {
        bool hl=(sr%ch==0); int rb=sr/ch;
        for (int sc=0; sc<cols; sc++) {
            bool vl=((sc+(rb%2)*half)%cw==0);
            if(!hl&&!vl) { continue; }
            mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|')));
        }
    }
}

/* Sideways brick wall: shift every other column down by half a brick. */
static void bg_draw_brick_v(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    int half=ch/2;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        int cb=sc/cw; bool hl=((sr+(cb%2)*half)%ch==0),vl=(sc%cw==0);
        if(!hl&&!vl) { continue; }
        mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|')));
    }
}

/* A grid turned 45 degrees, so the lines run as two sets of diagonals. */
static void bg_draw_diamond(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, ox=g->ox, oy=g->oy;
    int mod=2*DM_IW*DM_IH;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        int u=sc-ox,v=sr-oy;
        bool cl=(safe_mod(u*DM_IH+v*DM_IW,mod)==0),rl=(safe_mod(v*DM_IW-u*DM_IH,mod)==0);
        if(!cl&&!rl) { continue; }
        mvaddch(sr,sc,(chtype)(cl&&rl?'+': (cl?'/':'\\')));
    }
}

/* Like diamond, but the cells are twice as wide for that flatter iso look. */
static void bg_draw_iso(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, ox=g->ox, oy=g->oy;
    int mod=2*IS_IW*IS_IH;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        int u=sc-ox,v=sr-oy;
        bool cl=(safe_mod(u*IS_IH+v*IS_IW,mod)==0),rl=(safe_mod(v*IS_IW-u*IS_IH,mod)==0);
        if(!cl&&!rl) { continue; }
        mvaddch(sr,sc,(chtype)(cl&&rl?'+': (cl?'/':'\\')));
    }
}

/* Box grid with both diagonals drawn on top, so every cell gets an X. */
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

/* Grid lines plus filled squares on alternating cells, like a chessboard. */
static void bg_draw_check(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        bool hl=(sr%ch==0),vl=(sc%cw==0);
        if(hl||vl) mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|')));
        else if(((sr/ch)+(sc/cw))%2==1) mvaddch(sr,sc,(chtype)'#');
    }
}

/* Just evenly spaced horizontal lines, like ruled notebook paper. */
static void bg_draw_ruled(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, ch=g->ch;
    for (int sr=0; sr<rows-1; sr++) {
        if(sr%ch!=0) continue;
        for (int sc=0; sc<cols; sc++) mvaddch(sr,sc,(chtype)'-');
    }
}

/* Just a dot where each grid crossing would be, with no lines between them. */
static void bg_draw_dot(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    for (int sr=0; sr<rows-1; sr++) {
        if(sr%ch!=0) continue;
        for (int sc=0; sc<cols; sc++) if(sc%cw==0) mvaddch(sr,sc,(chtype)'*');
    }
}

/* Draw the background for whichever style is active. The five plain box
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

/* One scattered object sitting on a grid cell.
 *   r, c    which cell it's on
 *   glyph   the character drawn for it
 *   alive   false means this slot is empty / removed */
typedef struct { int r, c; char glyph; bool alive; } Obj;

/* All the objects on screen, kept in a plain fixed-size array. `count` is how
 * many slots are in use; we never grow past MAX_OBJ, so no allocation needed. */
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
        /* nudge into the cell's interior so the object doesn't sit on a line;
         * the rotated and ruled grids have no interior, so skip them */
        if (g->mode!=GM_DIAMOND&&g->mode!=GM_ISO&&g->mode!=GM_RULED)
            { sr+=(g->ch>1?1:0); sc+=(g->cw>1?1:0); }
        if (sr>=0&&sr<g->rows-1&&sc>=0&&sc<g->cols)
            mvaddch(sr,sc,(chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_OBJ)|A_BOLD);
}

/* ── §6 scatter (this file's distinct step) ── */

/* Placement = pick random cells inside the grid bounds and drop objects on
 * them. Four strategies differ only in how they choose / accept a pick:
 * scatter_random (no rule), scatter_mindist (spacing + tries rule),
 * scatter_flood (BFS rings), scatter_gradient (per-cell dice). */

/* King-move distance between two cells: the bigger of the row gap and column
 * gap. Cheap, and it treats a "circle" as a square — fitting for a grid. */
static int cheb(int r0, int c0, int r1, int c1)
{
    int dr=r0-r1; int dc=c0-c1;
    return (dr<0?-dr:dr) > (dc<0?-dc:dc) ? (dr<0?-dr:dr) : (dc<0?-dc:dc);
}

static int grid_w(const GridCtx *g) { return g->max_c - g->min_c + 1; }
static int grid_h(const GridCtx *g) { return g->max_r - g->min_r + 1; }

static int rand_range(int lo, int hi)
{
    if (lo >= hi) return lo;
    return lo + (int)((unsigned)rand() % (unsigned)(hi - lo + 1));
}

/* Toss RAND_N objects at random cells. Expect clumps and bare patches. */
static void scatter_random(Pool *p, const GridCtx *g, char glyph)
{
    for (int i=0; i<RAND_N; i++) {
        int r=rand_range(g->min_r,g->max_r);
        int c=rand_range(g->min_c,g->max_c);
        pool_place(p,r,c,glyph);
    }
}

/* Same random toss, but throw away any pick closer than MIN_DIST to an object
 * already down. The keep-or-toss rule gives the even, no-clumps look. */
static void scatter_mindist(Pool *p, const GridCtx *g, char glyph)
{
    for (int attempt=0; attempt<MAX_TRIES && p->count<MAX_OBJ; attempt++) {
        int r=rand_range(g->min_r,g->max_r);
        int c=rand_range(g->min_c,g->max_c);
        bool ok=true;
        for (int i=0; i<p->count && ok; i++)
            if (cheb(r,c,p->items[i].r,p->items[i].c) < MIN_DIST) ok=false;
        if (ok) pool_place(p,r,c,glyph);
    }
}

/* Spread out from the cursor one ring at a time (a breadth-first search),
 * dropping an object on each new cell, until FLOOD_MAX cells are filled.
 * We only step up/down/left/right, so it spreads as a growing diamond. */
static void scatter_flood(Pool *p, const GridCtx *g, int cr, int cc, char glyph)
{
    static const int dr4[]={-1,+1,0,0}, dc4[]={0,0,-1,+1};
    int gw=grid_w(g), gh=grid_h(g);
    int area=gw*gh;
    if (area<=0) return;

    /* vis marks cells we've already reached; qr/qc are the to-visit queue.
     * Freed at the end of this call — they don't outlive the flood. */
    bool *vis=(bool*)calloc((size_t)area, sizeof(bool));
    int  *qr =(int*) malloc((size_t)area * sizeof(int));
    int  *qc =(int*) malloc((size_t)area * sizeof(int));
    if (!vis||!qr||!qc) { free(vis); free(qr); free(qc); return; }

    int head=0, tail=0, placed=0;

    /* Add a cell to the queue if it's on the grid and we haven't seen it yet.
     * tail wraps around the array (a ring buffer); each cell enters at most
     * once, so the array is always big enough. */
#define ENQUEUE(R,C) do { \
    int _r=(R), _c=(C); \
    if (_r>=g->min_r&&_r<=g->max_r&&_c>=g->min_c&&_c<=g->max_c) { \
        int _idx=(_r-g->min_r)*gw+(_c-g->min_c); \
        if (!vis[_idx]) { vis[_idx]=true; qr[tail]=_r; qc[tail]=_c; \
                          tail=(tail+1)%area; } } } while(0)

    ENQUEUE(cr,cc);

    while (head!=tail && placed<FLOOD_MAX && p->count<MAX_OBJ) {
        int r=qr[head], c=qc[head]; head=(head+1)%area;
        pool_place(p,r,c,glyph); placed++;
        for (int d=0; d<4; d++) ENQUEUE(r+dr4[d], c+dc4[d]);
    }
#undef ENQUEUE

    free(vis); free(qr); free(qc);
}

/* Walk every cell and roll the dice for each one. Cells near the grid centre
 * win more often, so the scatter comes out densest in the middle and thins
 * toward the edges. */
static void scatter_gradient(Pool *p, const GridCtx *g, char glyph)
{
    int cr=(g->min_r+g->max_r)/2, cc=(g->min_c+g->max_c)/2;
    for (int r=g->min_r; r<=g->max_r && p->count<MAX_OBJ; r++) {
        for (int c=g->min_c; c<=g->max_c && p->count<MAX_OBJ; c++) {
            int dist=cheb(r,c,cr,cc);
            /* chance of placing here: full odds at the centre, fading with
             * distance. We scale that chance up to RAND_MAX so a single
             * rand() roll decides it. The long cast keeps the big
             * multiplication from overflowing an int. */
            int threshold = (int)((long)GRAD_SCALE * RAND_MAX / (dist + GRAD_SCALE));
            if (rand() < threshold)
                pool_place(p,r,c,glyph);
        }
    }
}

/* ── §7 cursor ── */

/* Where the @ marker sits, as a grid cell. Flood scatter starts from here. */
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
    int sr,sc; cell_to_screen(g,cur->r,cur->c,&sr,&sc);
    if (g->mode!=GM_DIAMOND&&g->mode!=GM_ISO&&g->mode!=GM_RULED)
        { sr+=(g->ch>1?1:0); sc+=(g->cw>1?1:0); }
    attron(COLOR_PAIR(PAIR_CURSOR)|A_BOLD);
    if (sr>=0&&sr<g->rows-1&&sc>=0&&sc<g->cols) mvaddch(sr,sc,(chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR)|A_BOLD);
}

/* ── §8 scene ── */

/* Draw the on-screen labels: fps and object count in the top-right, the list
 * of keys along the bottom. */
static void hud_draw(const GridCtx *g, const Pool *p, double fps)
{
    char buf[96];
    snprintf(buf,sizeof buf," %.1f fps  %s  objs=%d/%d ",
             fps,gm_name[g->mode],p->count,MAX_OBJ);
    attron(COLOR_PAIR(PAIR_HUD)|A_BOLD);
    mvprintw(0,g->cols-(int)strlen(buf),"%s",buf);
    attroff(COLOR_PAIR(PAIR_HUD)|A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT)|A_BOLD);
    mvprintw(g->rows-1, 0,
        " arrows:move  R:random M:min-dist F:flood G:gradient"
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
    hud_draw(g, p, fps);
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

int main(void)
{
    signal(SIGINT,on_signal); signal(SIGTERM,on_signal);
    signal(SIGWINCH,on_signal);
    screen_init();
    srand((unsigned)time(NULL));

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
        case 'C': pool_clear(&pool); break;
        case 'a': { GridMode m=(GridMode)((ctx.mode-1+GM_COUNT)%GM_COUNT);
                    ctx_init(&ctx,m,rows,cols); cursor_reset(&cur,&ctx); } break;
        case 'e': { GridMode m=(GridMode)((ctx.mode+1)%GM_COUNT);
                    ctx_init(&ctx,m,rows,cols); cursor_reset(&cur,&ctx); } break;
        case 'R': scatter_random(&pool,&ctx,'o'); break;
        case 'M': scatter_mindist(&pool,&ctx,'o'); break;
        case 'F': scatter_flood(&pool,&ctx,cur.r,cur.c,'o'); break;
        case 'G': scatter_gradient(&pool,&ctx,'o'); break;
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
