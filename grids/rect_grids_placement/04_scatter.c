/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 04_scatter.c — procedural object scattering on all 14 grid types
 *
 * DEMO: Press keys to scatter objects procedurally across the grid.
 *       R=random scatter, M=min-distance (Poisson-ish), F=BFS flood fill
 *       from cursor, G=gradient density (denser near grid centre).
 *       Each scatter uses the cursor as the seed/anchor point.
 *       Works on all 14 grid backgrounds (keys 1-9, a-f).
 *
 * Study alongside: 02_patterns.c (stamp-based placement), 01_direct.c
 *
 * Section map:
 *   §1 config   — per-mode geometry, scatter parameters
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 6 pairs
 *   §4 gridctx  — GridCtx, ctx_init, ctx_to_screen, ctx_draw_bg
 *   §5 pool     — ObjectPool
 *   §6 scatter  — random, min-distance, BFS flood fill, gradient
 *   §7 cursor   — Cursor struct, move, reset, draw
 *   §8 scene    — scene_draw
 *   §9 screen   — ncurses init/cleanup
 *   §10 app     — signals, main loop
 *
 * Keys:  arrows:move  R:random  M:min-dist  F:flood  G:gradient
 *        C:clear  r:reset  q/ESC:quit
 *        a:prev-grid  e:next-grid
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/rect_grids_placement/04_scatter.c \
 *       -o 04_scatter -lncurses
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Four procedural placement strategies.
 *
 *   Random scatter (R):
 *     Uniformly pick N random (r,c) pairs from the valid grid range.
 *     Simple but may produce clusters and large voids.
 *
 *   Min-distance / Poisson-ish (M):
 *     Repeat up to MAX_TRIES times: pick a random candidate, accept it
 *     only if it is at least MIN_DIST cells (Chebyshev distance) from
 *     every existing object.  Produces more even spacing than pure random.
 *     This is a simplified Bridson Poisson-disk sample — O(n²) per attempt
 *     rather than O(1) with spatial hashing, but correct for n≤256.
 *
 *   BFS flood fill (F):
 *     Starting from the cursor cell, expand outward in breadth-first order.
 *     Place an object at each visited cell.  Stop after FLOOD_MAX cells.
 *     Shows how BFS naturally produces an even radial expansion pattern.
 *
 *   Gradient density (G):
 *     Sweep all cells; place an object with probability P(r,c) that depends
 *     on the Chebyshev distance from the grid centre.  Cells near the centre
 *     have higher probability — produces a cloud denser at the middle.
 *
 * Data-structure : Same ObjectPool as 01_direct.c.  BFS uses a small
 *                  circular queue (ring buffer) sized to the max grid area.
 *
 * References     :
 *   Poisson disk sampling — Bridson 2007 "Fast Poisson Disk Sampling"
 *                           ACM SIGGRAPH Sketches
 *   BFS — en.wikipedia.org/wiki/Breadth-first_search
 *   Chebyshev distance — en.wikipedia.org/wiki/Chebyshev_distance
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA — PLACEMENT AS SAMPLING
 * ───────────────────────────────────
 * All four algorithms are SAMPLING strategies: they select a subset of grid
 * cells to receive objects.  The difference is in how they define "which
 * cells" — the sampling distribution:
 *
 *   Random:    uniform distribution over all valid cells.
 *   Min-dist:  uniform with rejection: reject any sample within MIN_DIST of
 *              an existing object.  Produces blue-noise distribution.
 *   Flood:     BFS-order from a seed: the "distribution" is determined by
 *              graph distance from the cursor, not randomness.
 *   Gradient:  Bernoulli trial per cell with P proportional to 1/(dist+1).
 *              Produces higher density near the centre.
 *
 * WHY CHEBYSHEV DISTANCE
 * ───────────────────────
 * Chebyshev distance = max(|Δr|, |Δc|).  It measures the minimum number of
 * king-moves (8-connected steps) between two cells.  It is used because:
 *   (a) It is fast to compute — no sqrt, no multiplication.
 *   (b) It defines the natural "ball" shape for grid movement (a square,
 *       not a circle), which matches the grid aesthetics of this project.
 *   (c) The Chebyshev ball of radius k = all cells within a k×k square,
 *       making the MIN_DIST check visually intuitive.
 *
 * BFS FLOOD FILL ON A GRID
 * ─────────────────────────
 * A BFS from cell (cr,cc) visits cells in order of their shortest graph
 * distance from (cr,cc).  On a rectangular grid with 4-connectivity, this
 * produces concentric "diamonds"; with 8-connectivity, concentric squares.
 * This file uses 4-connectivity for clarity.
 * The visited[] array prevents re-queuing cells.  It is zeroed before each
 * flood and acts as both the "seen" and "placed" marker.
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

#define TARGET_FPS   30
#define MAX_OBJ     256

#define RAND_N       40     /* objects placed by R (random scatter) */
#define MAX_TRIES   400     /* rejection-sampling attempts for M */
#define MIN_DIST      3     /* Chebyshev min-distance for M */
#define FLOOD_MAX   120     /* max cells filled by F (BFS flood) */
#define GRAD_SCALE    6     /* gradient: probability denominator scale */

/* Per-mode geometry (same as 01_direct.c) */
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
    int rows=g->rows,cols=g->cols,cw=g->cw,ch=g->ch,ox=g->ox,oy=g->oy;
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
    case GM_HIER: { int major=cw*2,semi=cw;
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            bool hm=(sr%major==0),hs=(!hm&&sr%semi==0),hmi=(!hm&&!hs&&sr%ch==0);
            bool vm=(sc%major==0),vs=(!vm&&sc%semi==0),vmi=(!vm&&!vs&&sc%cw==0);
            if(!hm&&!hs&&!hmi&&!vm&&!vs&&!vmi) continue;
            bool hl=hm||hs||hmi,vl=vm||vs||vmi;
            char c; if(hl&&vl) c='+'; else if(hl) c=(hm?'=':(hs?'-':'.')); else c=(vm?'#':(vs?'|':':'));
            mvaddch(sr,sc,(chtype)(unsigned char)c); } break; }
    case GM_BRICK_H: { int half=cw/2;
        for (int sr=0; sr<rows-1; sr++) { bool hl=(sr%ch==0); int rb=sr/ch;
            for (int sc=0; sc<cols; sc++) { bool vl=((sc+(rb%2)*half)%cw==0);
                if(!hl&&!vl) { continue; }
                mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|'))); } } break; }
    case GM_BRICK_V: { int half=ch/2;
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            int cb=sc/cw; bool hl=((sr+(cb%2)*half)%ch==0),vl=(sc%cw==0);
            if(!hl&&!vl) { continue; }
            mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|'))); } break; }
    case GM_DIAMOND: { int mod=2*DM_IW*DM_IH;
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            int u=sc-ox,v=sr-oy;
            bool cl=(safe_mod(u*DM_IH+v*DM_IW,mod)==0),rl=(safe_mod(v*DM_IW-u*DM_IH,mod)==0);
            if(!cl&&!rl) { continue; }
            mvaddch(sr,sc,(chtype)(cl&&rl?'+': (cl?'/':'\\'))); } break; }
    case GM_ISO: { int mod=2*IS_IW*IS_IH;
        for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
            int u=sc-ox,v=sr-oy;
            bool cl=(safe_mod(u*IS_IH+v*IS_IW,mod)==0),rl=(safe_mod(v*IS_IW-u*IS_IH,mod)==0);
            if(!cl&&!rl) { continue; }
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
/* §6  scatter                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

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

/*
 * scatter_random — place RAND_N objects at uniformly random grid cells.
 */
static void scatter_random(Pool *p, const GridCtx *g, char glyph)
{
    for (int i=0; i<RAND_N; i++) {
        int r=rand_range(g->min_r,g->max_r);
        int c=rand_range(g->min_c,g->max_c);
        pool_place(p,r,c,glyph);
    }
}

/*
 * scatter_mindist — Poisson-ish: reject candidates within MIN_DIST
 * Chebyshev distance of any existing object.
 */
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

/*
 * scatter_flood — BFS from (cr,cc); place objects at visited cells.
 * Uses 4-connectivity.  Stops after FLOOD_MAX placements.
 */
static void scatter_flood(Pool *p, const GridCtx *g, int cr, int cc, char glyph)
{
    static const int dr4[]={-1,+1,0,0}, dc4[]={0,0,-1,+1};
    int gw=grid_w(g), gh=grid_h(g);
    int area=gw*gh;
    if (area<=0) return;

    bool *vis=(bool*)calloc((size_t)area, sizeof(bool));
    int  *qr =(int*) malloc((size_t)area * sizeof(int));
    int  *qc =(int*) malloc((size_t)area * sizeof(int));
    if (!vis||!qr||!qc) { free(vis); free(qr); free(qc); return; }

    int head=0, tail=0, placed=0;

    /* inline enqueue: bounds-check then push to ring buffer */
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

/*
 * scatter_gradient — Bernoulli trial per cell: P = GRAD_SCALE/(dist+GRAD_SCALE).
 * Cells closer to grid centre have higher probability of receiving an object.
 */
static void scatter_gradient(Pool *p, const GridCtx *g, char glyph)
{
    int cr=(g->min_r+g->max_r)/2, cc=(g->min_c+g->max_c)/2;
    for (int r=g->min_r; r<=g->max_r && p->count<MAX_OBJ; r++) {
        for (int c=g->min_c; c<=g->max_c && p->count<MAX_OBJ; c++) {
            int dist=cheb(r,c,cr,cc);
            /* probability: GRAD_SCALE/(dist+GRAD_SCALE) in [0,1] */
            /* use long to avoid overflow: GRAD_SCALE*RAND_MAX exceeds INT_MAX */
            int threshold = (int)((long)GRAD_SCALE * RAND_MAX / (dist + GRAD_SCALE));
            if (rand() < threshold)
                pool_place(p,r,c,glyph);
        }
    }
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
    if (g->mode!=GM_DIAMOND&&g->mode!=GM_ISO&&g->mode!=GM_RULED)
        { sr+=(g->ch>1?1:0); sc+=(g->cw>1?1:0); }
    attron(COLOR_PAIR(PAIR_CURSOR)|A_BOLD);
    if (sr>=0&&sr<g->rows-1&&sc>=0&&sc<g->cols) mvaddch(sr,sc,(chtype)'@');
    attroff(COLOR_PAIR(PAIR_CURSOR)|A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                       double fps)
{
    int rows=g->rows, cols=g->cols;
    erase();
    ctx_draw_bg(g);
    pool_draw(p,g);
    cursor_draw(cur,g);

    char buf[96];
    snprintf(buf,sizeof buf," %.1f fps  %s  objs=%d/%d ",
             fps,gm_name[g->mode],p->count,MAX_OBJ);
    attron(COLOR_PAIR(PAIR_HUD)|A_BOLD);
    mvprintw(0,cols-(int)strlen(buf),"%s",buf);
    attroff(COLOR_PAIR(PAIR_HUD)|A_BOLD);

    attron(COLOR_PAIR(PAIR_ACTIVE)|A_BOLD);
    mvprintw(rows-1, 0, " %-12s", gm_name[g->mode]);
    attroff(COLOR_PAIR(PAIR_ACTIVE)|A_BOLD);
    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(rows-1,13,
        " arrows:move  R:random M:min-dist F:flood G:gradient"
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
        fps=fps*0.95+(1e9/(now-t0+1))*0.05; t0=now;
        scene_draw(&ctx,&pool,&cur,fps);
        clock_sleep_ns(FRAME_NS-(clock_ns()-now));
    }
    return 0;
}
