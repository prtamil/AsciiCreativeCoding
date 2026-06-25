/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_patterns.c — stamp shapes onto any of 14 grid styles.
 *
 * Move the @ cursor, pick a shape (border / fill / hollow box / row / column),
 * and stamp it. A shape is just a rule that says "fill the cell at this offset
 * from the cursor" — so the same shape works on every grid and grows or shrinks
 * with one number. Before you stamp, the cursor shows a dotted preview of what
 * will land.
 *
 * Sister files: 01_direct.c (toggle one cell), 03_path.c (draw between two points).
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
#define MAX_PAT_N   8    /* biggest a shape can grow (half-size) */
#define MIN_PAT_N   1

/* How much to trust the newest frame when smoothing the fps number. Small =
 * steady reading that doesn't jump around. */
#define FPS_EWMA_ALPHA  0.05

/* Cell width/height for each grid style, in characters (same as 01_direct.c). */
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

/* ── §4 gridctx ── */

/* The 14 grid styles you can stamp onto. GM_COUNT is the total, used for the
 * a/e cycle and for sizing name arrays — keep it last. */
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

/* Everything we need to know about the currently chosen grid: its size on
 * screen, how big one cell is, and which (row,col) coordinates are valid. One
 * of these is rebuilt whenever you switch grids or the terminal resizes. */
typedef struct {
    GridMode mode;              /* which of the 14 styles is active */
    int rows, cols;             /* terminal size in characters */
    int cw, ch;                 /* one cell's width and height in characters */
    int ox, oy;                 /* screen centre — the origin for rotated grids */
    int range;                  /* rotated grids run from -range..+range cells */
    int min_r, max_r;           /* valid row range for the cursor and stamps */
    int min_c, max_c;           /* valid column range */
} GridCtx;

/* Each grid style just needs its own cell size; one tiny setter per style. */

static void ctx_geom_uniform (GridCtx *g) { g->cw = U_CW;    g->ch = U_CH;  }
static void ctx_geom_square  (GridCtx *g) { g->cw = SQ_CS*2; g->ch = SQ_CS; }
static void ctx_geom_fine    (GridCtx *g) { g->cw = FN_CW;   g->ch = FN_CH; }
static void ctx_geom_coarse  (GridCtx *g) { g->cw = CO_CW;   g->ch = CO_CH; }
static void ctx_geom_hier    (GridCtx *g) { g->cw = HI_CW;   g->ch = HI_CH; }
static void ctx_geom_brick_h (GridCtx *g) { g->cw = BH_CW;   g->ch = BH_CH; }
static void ctx_geom_brick_v (GridCtx *g) { g->cw = BV_CW;   g->ch = BV_CH; }
/* The 45°-rotated grids count cells outward from the centre, so they need range. */
static void ctx_geom_diamond (GridCtx *g) { g->cw = DM_IW; g->ch = DM_IH; g->range = DM_RNG; }
static void ctx_geom_iso     (GridCtx *g) { g->cw = IS_IW; g->ch = IS_IH; g->range = IS_RNG; }
static void ctx_geom_cross   (GridCtx *g) { g->cw = CR_CW;   g->ch = CR_CH; }
static void ctx_geom_check   (GridCtx *g) { g->cw = CK_CW;   g->ch = CK_CH; }
/* Ruled is just horizontal lines, so there's no column width to set. */
static void ctx_geom_ruled   (GridCtx *g) { g->ch = RL_LS; }
static void ctx_geom_dot     (GridCtx *g) { g->cw = DT_CW;   g->ch = DT_CH; }
static void ctx_geom_origin  (GridCtx *g) { g->cw = OR_CW;   g->ch = OR_CH; }

/* Work out which (row,col) coordinates actually fit on screen. Each family
 * counts differently: rotated grids spread out from the centre, ruled grids
 * count whole horizontal lines, and the rest count whole rectangular cells. */
static void ctx_set_bounds(GridCtx *g, GridMode m, int rows, int cols)
{
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

/* Build the context for one grid style: pick its cell size, then figure out
 * its valid coordinate range. Call this on startup and on every grid switch. */
static void ctx_init(GridCtx *g, GridMode m, int rows, int cols)
{
    memset(g, 0, sizeof *g);
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

/* Turn a grid coordinate (row,col) into the screen spot to draw it. This is
 * the one place that knows each style's layout, so objects placed in one grid
 * reappear in the right spot when you switch to another. */
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

/* Remainder that's never negative, so a%b wraps cleanly for negative coords. */
static int safe_mod(int a, int b) { return ((a%b)+b)%b; }

/* Background drawers — one per grid style, so you can read each in isolation. */

/* Plain rectangular grid. The origin style also paints a bright cross at the
 * centre row and column so you can see where (0,0) is. */
static void bg_draw_rect_family(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch, ox=g->ox, oy=g->oy;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        bool hl=(sr%ch==0), vl=(sc%cw==0);
        if (!hl && !vl) continue;
        char c=(hl&&vl)?'+': (hl?'-':'|');
        if (g->mode==GM_ORIGIN && sr==oy) c=(vl?'+':'=');
        if (g->mode==GM_ORIGIN && sc==ox) c=(hl?'+':'I');
        mvaddch(sr, sc, (chtype)(unsigned char)c);
    }
}

/* Grid with three line weights — major, semi, minor — like graph paper. The
 * glyph tells you which weight each line is. */
static void bg_draw_hier(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    int major=cw*2, semi=cw;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        bool hm=(sr%major==0), hs=(!hm&&sr%semi==0), hmi=(!hm&&!hs&&sr%ch==0);
        bool vm=(sc%major==0), vs=(!vm&&sc%semi==0), vmi=(!vm&&!vs&&sc%cw==0);
        if (!hm&&!hs&&!hmi&&!vm&&!vs&&!vmi) continue;
        bool hl=hm||hs||hmi, vl=vm||vs||vmi;
        char c; if(hl&&vl) c='+'; else if(hl) c=(hm?'=':(hs?'-':'.')); else c=(vm?'#':(vs?'|':':'));
        mvaddch(sr, sc, (chtype)(unsigned char)c);
    }
}

/* Brick wall: every other row is shoved sideways by half a brick. */
static void bg_draw_brick_h(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    int half=cw/2;
    for (int sr=0; sr<rows-1; sr++) {
        bool hl=(sr%ch==0); int rb=sr/ch;
        for (int sc=0; sc<cols; sc++) {
            bool vl=((sc+(rb%2)*half)%cw==0);
            if (!hl&&!vl) continue;
            mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|')));
        }
    }
}

/* Brick wall turned on its side: every other column is shoved up by half a brick. */
static void bg_draw_brick_v(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    int half=ch/2;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        int cb=sc/cw; bool hl=((sr+(cb%2)*half)%ch==0), vl=(sc%cw==0);
        if (!hl&&!vl) continue;
        mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|')));
    }
}

/* Diamond grid: two sets of diagonal lines crossing, like a grid tilted 45°. */
static void bg_draw_diamond(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, ox=g->ox, oy=g->oy;
    int mod=2*DM_IW*DM_IH;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        int u=sc-ox, v=sr-oy;
        bool cl=(safe_mod(u*DM_IH+v*DM_IW,mod)==0);
        bool rl=(safe_mod(v*DM_IW-u*DM_IH,mod)==0);
        if (!cl&&!rl) continue;
        mvaddch(sr,sc,(chtype)(cl&&rl?'+': (cl?'/':'\\')));
    }
}

/* Like diamond, but the cells are twice as wide as tall — the isometric look. */
static void bg_draw_iso(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, ox=g->ox, oy=g->oy;
    int mod=2*IS_IW*IS_IH;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        int u=sc-ox, v=sr-oy;
        bool cl=(safe_mod(u*IS_IH+v*IS_IW,mod)==0);
        bool rl=(safe_mod(v*IS_IW-u*IS_IH,mod)==0);
        if (!cl&&!rl) continue;
        mvaddch(sr,sc,(chtype)(cl&&rl?'+': (cl?'/':'\\')));
    }
}

/* Rectangular grid with diagonals laid over it too, so each cell gets an X. */
static void bg_draw_cross(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    int sa=cw, sb=ch;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        bool hl=(sr%ch==0), vl=(sc%cw==0);
        bool sl=((sc+sr)%sa==0), bl=(safe_mod(sc-sr,sb)==0);
        if (!hl&&!vl&&!sl&&!bl) continue;
        char c; if(hl&&vl) c='+'; else if(hl) c='-'; else if(vl) c='|';
                else if(sl&&bl) c='X'; else if(sl) c='/'; else c='\\';
        mvaddch(sr,sc,(chtype)(unsigned char)c);
    }
}

/* Grid lines plus filled-in squares on alternating cells — a chessboard. */
static void bg_draw_check(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    for (int sr=0; sr<rows-1; sr++) for (int sc=0; sc<cols; sc++) {
        bool hl=(sr%ch==0), vl=(sc%cw==0);
        if (hl||vl) mvaddch(sr,sc,(chtype)(hl&&vl?'+': (hl?'-':'|')));
        else if (((sr/ch)+(sc/cw))%2==1) mvaddch(sr,sc,(chtype)'#');
    }
}

/* Just horizontal lines, like ruled notebook paper — no columns. */
static void bg_draw_ruled(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, ch=g->ch;
    for (int sr=0; sr<rows-1; sr++) {
        if (sr%ch!=0) continue;
        for (int sc=0; sc<cols; sc++) mvaddch(sr,sc,(chtype)'-');
    }
}

/* Just a dot at each crossing point, with no lines drawn between them. */
static void bg_draw_dot(const GridCtx *g)
{
    int rows=g->rows, cols=g->cols, cw=g->cw, ch=g->ch;
    for (int sr=0; sr<rows-1; sr++) {
        if (sr%ch!=0) continue;
        for (int sc=0; sc<cols; sc++) if (sc%cw==0) mvaddch(sr,sc,(chtype)'*');
    }
}

/* Draw whichever grid is active. The five rectangular styles share one drawer;
 * the rest get their own. */
static void ctx_draw_bg(const GridCtx *g)
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

/* One stamped object: where it sits on the grid and what character to draw. */
typedef struct {
    int  r, c;      /* grid coordinate, not screen coordinate */
    char glyph;     /* the character drawn for this object */
    bool alive;     /* false means this slot is unused */
} Obj;

/* The bag of all stamped objects — a plain fixed array, no allocation. We never
 * remove objects here (only Clear wipes them), so count just grows up to MAX_OBJ. */
typedef struct {
    Obj items[MAX_OBJ];
    int count;      /* how many slots are in use */
} Pool;

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

/* ── §6 patterns ── */

/* The five shapes you can stamp. */
typedef enum { PAT_BORDER=0, PAT_FILL, PAT_HOLLOW, PAT_ROW, PAT_COL } PatMode;

static const char *const pat_name[] = {
    "border","fill","hollow","row","col"
};

/* The heart of the file: given an offset from the cursor, does this shape fill
 * that spot? N is the shape's half-size. Each shape is just a yes/no rule, which
 * is why one function covers all of them and they all scale with N. */
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

/* Walk every spot the shape covers and drop an object there. Anything that falls
 * off the grid is quietly ignored. */
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

/* Show a dotted outline of where the shape would land if you stamped right now,
 * using the same rule as the real stamp so the preview is always honest. */
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

/* ── §7 cursor ── */

/* Where the @ sits, in grid coordinates. Shapes are stamped centred here. */
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

/* ── §8 scene ── */

/* The status line (top-right) and the key reminder (bottom). */
static void hud_draw(const GridCtx *g, const Pool *p, PatMode pat, int N,
                     double fps)
{
    char buf[96];
    snprintf(buf,sizeof buf," %.1f fps  %s  pat=%s N=%d  objs=%d ",
             fps,gm_name[g->mode],pat_name[pat],N,p->count);
    attron(COLOR_PAIR(PAIR_HUD)|A_BOLD);
    mvprintw(0,g->cols-(int)strlen(buf),"%s",buf);
    attroff(COLOR_PAIR(PAIR_HUD)|A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT)|A_BOLD);
    mvprintw(g->rows-1, 0,
        " arrows:move  B:border F:fill H:hollow R:row V:col"
        "  spc:stamp  +/-:size  C:clear  r:reset  q:quit"
        "  a:prev-grid  e:next-grid  [%s] ", gm_name[g->mode]);
    attroff(COLOR_PAIR(PAIR_HINT)|A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                       PatMode pat, int N, double fps)
{
    erase();
    ctx_draw_bg(g);
    pool_draw(p,g);
    preview_draw(g,cur->r,cur->c,pat,N);
    cursor_draw(cur,g);
    hud_draw(g, p, pat, N, fps);
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

/* Set from signal handlers, so they must be the safe-to-touch-from-a-signal type.
 * The main loop reads them and reacts: stop, or rebuild after a terminal resize. */
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
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9/(now-t0+1)) * FPS_EWMA_ALPHA;
        t0 = now;
        scene_draw(&ctx,&pool,&cur,pat,N,fps);
        clock_sleep_ns(FRAME_NS-(clock_ns()-now));
    }
    return 0;
}
