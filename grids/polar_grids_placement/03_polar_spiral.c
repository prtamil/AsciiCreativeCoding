/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 03_polar_spiral.c — parametric spiral path placement on a polar grid
 *
 * DEMO: Move the cursor and watch the spiral preview update in real time.
 *       Toggle 'l'/'o' for Archimedean or log-spiral mode.  +/- adjusts
 *       pitch or growth rate; [/] adjusts the number of turns; ,/. adjusts
 *       density (dots per radian).  Press space to stamp the current spiral
 *       into the permanent pool; C clears the pool.  Because the path is
 *       parametric (θ → r → cell) rather than a screen sweep, dots follow
 *       the spiral equation exactly.
 *
 * Study alongside: 02_polar_arc.c (arc/spoke from two anchors),
 *                  grids/polar_grids/03_archimedean_spiral.c (sweep rendering)
 *
 * Section map:
 *   §1 config   — spiral parameters, density, turn range
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 5 pairs: grid, active, anchor, HUD (yellow), hint (cyan)
 *   §4 gridctx  — GridCtx (mode, origin), cell_to_polar, polar_to_screen
 *   §5 pool     — Pool: place, clear, draw
 *   §6 cursor   — Cursor (row, col, r, theta) + screen-space arrow move
 *   §7 mode     — draw_polar_bg: 7 inline polar background types in one switch
 *   §8 spiral   — spiral_place_archim, spiral_place_log
 *   §9 scene    — hud_draw + scene_draw (+ live preview)
 *   §10 screen  — ncurses init / cleanup
 *   §11 app     — signals, resize, main loop
 *
 * Keys:  q/ESC quit   P pause   t theme   a/e prev/next background
 *        l Archimedean mode    o log-spiral mode
 *        +/- pitch or growth rate    [/] turns    ,/. density
 *        space stamp spiral to pool    C clear pool    r reset cursor
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/polar_grids_placement/03_polar_spiral.c \
 *       -o 03_polar_spiral -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Parametric spiral rasterisation.  Instead of testing
 *                  every screen cell (sweep approach used in polar_grids/),
 *                  this file evaluates the spiral equation forward:
 *
 *                  Archimedean: r(θ) = r_0 + a × θ,    a = PITCH/(2π)
 *                  Log-spiral:  r(θ) = r_0 × e^(GROWTH × θ)
 *
 *                  θ is incremented by DENSITY_STEP (radians) up to
 *                  N_TURNS × 2π.  Each (r, θ) is converted to a terminal
 *                  cell; objects accumulate in the pool.
 *
 *                  Parametric vs sweep: the sweep in polar_grids/ tests
 *                  whether a cell is NEAR a spiral; the parametric approach
 *                  here WALKS along the spiral and places objects directly.
 *                  Parametric placement gives sparser dots but exact
 *                  positions — ideal for seeing the curve's structure.
 *
 * Math           : At radius r, one terminal column subtends CELL_W/r radians.
 *                  Setting DENSITY_STEP ≈ CELL_W/r_start ensures ≈ 1 object
 *                  per column near the inner end.  The user can coarsen
 *                  (fewer dots) or refine (more) with ,/. keys.
 *
 * References     :
 *   Archimedean spiral — en.wikipedia.org/wiki/Archimedean_spiral
 *   Logarithmic spiral — en.wikipedia.org/wiki/Logarithmic_spiral
 *   Sweep approach — grids/polar_grids/03_archimedean_spiral.c
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ──────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Instead of sweeping every screen cell and testing whether it lies near a
 * spiral (the approach in polar_grids/03_archimedean_spiral.c), this file
 * walks the spiral PARAMETRICALLY: t increments from 0 to N_TURNS×2π, r is
 * computed from t, and each (r, θ0+t) is placed as an object.  This gives
 * exact positions at adjustable dot density.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Parametric vs. sweep:
 *   Sweep:      for each screen cell → is this cell near the spiral?
 *   Parametric: for each t step    → where does the spiral go?
 * Parametric gives sparser, exact dots; sweep gives a filled line.
 * Use sweep for a thick background; use parametric to study the curve.
 *
 * t is the angular offset from the start (the "clock hand" rotation).
 * Archimedean: hand grows by a constant amount per radian.
 * Log-spiral:  hand grows by a fraction of its current length per radian.
 *
 * DRAWING METHOD
 * ──────────────
 * 1. Move cursor to the spiral's starting point (r0, θ0).
 * 2. The live preview redraws every frame in scene_draw():
 *    walk t from 0 to N_TURNS×2π by DENSITY_STEP, compute r(t),
 *    call polar_to_screen(r(t), θ0+t), draw dot directly.
 * 3. SPACE stamps: spiral_place_archim or spiral_place_log does the same
 *    walk and appends each cell to the permanent pool.
 * 4. +/-: pitch/growth; [/]: turns; ,/.: density.
 *
 * KEY FORMULAS
 * ────────────
 * Archimedean (spiral_place_archim):
 *   a = pitch / (2π)                  [px of radial advance per radian]
 *   r(t) = r0 + a × t                 [linear radial growth]
 *   θ(t) = θ0 + t,  t ∈ [0, N_TURNS × 2π],  step = DENSITY_STEP
 *   Dots ≈ N_TURNS × 2π / DENSITY_STEP
 *   Example: N=3, d=0.08 → 3×6.283/0.08≈236 dots
 *
 * Log-spiral (spiral_place_log):
 *   r(t) = r0 × e^(GROWTH × t)       [exponential radial growth]
 *   r0 clamped to ≥ 1.0 (r0=0 would freeze r=0 for all t since 0×eˣ=0)
 *   Radius after 1 turn: r0 × e^(GROWTH × 2π)
 *   Example: GROWTH=0.18, 1 turn: r × e^1.13 ≈ r × 3.1
 *
 * Dot density at radius r:
 *   One terminal column subtends CELL_W/r radians.
 *   DENSITY ≈ CELL_W/r_start ≈ 1 dot per column at the inner end.
 *   Outer end is sparser (more arc length per radian).
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 * - r0 < 1.0 clamp: spiral_place_log enforces r0≥1 to prevent the
 *   r=0 freeze case.
 * - Very small DENSITY_STEP (0.01): up to N_MAX×2π/0.01≈6280 dots per
 *   spiral arm; MAX_OBJ=4096 may be reached with 2+ turns.
 * - Live preview loops up to N_MAX×2π/DENSITY_MIN≈6280 times per frame;
 *   no pool writes so this is fast (just mvaddch calls).
 *
 * HOW TO VERIFY
 * ─────────────
 * Terminal 80×24 → ox=40, oy=12.
 * Defaults: r0=20, θ0=0, PITCH=32, N_TURNS=3, DENSITY=0.08.
 *
 * Archimedean:
 *   a=32/(2π)≈5.09
 *   t=0:  r=20, θ=0   → col=40+round(20/2)=50, row=12         ✓
 *   t=2π: r=52, θ=2π  → col=40+round(52/2)=66, row=12         ✓
 *   t=4π: r=84, θ=4π  → col=40+round(84/2)=82 (off-screen)    ✓
 *
 * Log-spiral (GROWTH=0.18):
 *   t=π/2: r=20×e^(0.18×π/2)≈20×e^0.283≈26.6, θ=90°
 *          → col=40, row=12+round(26.6/4)≈19                   ✓
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS    30
#define CELL_W         2
#define CELL_H         4

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA  0.05

/* Archimedean pitch: pixel advance per full turn */
#define PITCH_DEFAULT  32.0
#define PITCH_MIN       8.0
#define PITCH_MAX      80.0
#define PITCH_STEP      4.0

/* Log-spiral growth rate a: r multiplies by e^(a×2π) per turn */
#define GROWTH_DEFAULT  0.18
#define GROWTH_MIN      0.05
#define GROWTH_MAX      0.70
#define GROWTH_STEP     0.02
#define PHI             1.61803398874989484820
#define GROWTH_GOLDEN  (2.0 * log(PHI) / M_PI)   /* ≈ 0.3065 */

/* Number of spiral turns to draw */
#define N_TURNS_DEFAULT  3
#define N_TURNS_MIN      1
#define N_TURNS_MAX     10

/* Angular step between placed objects (radians) */
#define DENSITY_DEFAULT  0.08    /* ~4.6° per dot */
#define DENSITY_MIN      0.01
#define DENSITY_MAX      0.40
#define DENSITY_STEP     0.01

/* Object pool */
#define MAX_OBJ        4096
#define OBJ_GLYPH      '.'

#define GOLDEN_ANGLE   (2.0 * M_PI / (PHI * PHI))
#define N_BG_SEEDS      600

#define PAIR_GRID    1
#define PAIR_ACTIVE  2
#define PAIR_ANCHOR  3
#define PAIR_HUD     4   /* status bar (yellow)  */
#define PAIR_HINT    5   /* key-hint footer (cyan) */

static const char *const BG_NAMES[] = {
    "rings+spokes", "log-polar",  "archimedean",
    "log-spiral",   "sunflower",  "equal-area",  "elliptic",
};
#define N_BG_TYPES  7

static const short THEME_FG[][2] = {
    {75,  COLOR_CYAN},
    {82,  COLOR_GREEN},
    {69,  COLOR_BLUE},
    {201, COLOR_MAGENTA},
    {226, COLOR_YELLOW},
};
#define N_THEMES  5

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

static void color_init(int theme)
{
    start_color(); use_default_colors();
    short fg = COLORS >= 256 ? THEME_FG[theme][0] : THEME_FG[theme][1];
    init_pair(PAIR_GRID,   fg,                               -1);
    init_pair(PAIR_ACTIVE, COLORS>=256 ? 255 : COLOR_WHITE,  -1);
    init_pair(PAIR_ANCHOR, COLORS>=256 ? 220 : COLOR_YELLOW, -1);
    init_pair(PAIR_HUD,    COLORS>=256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,   COLORS>=256 ?  51 : COLOR_CYAN,   -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  gridctx                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int mode;
    int rows, cols;
    int cw, ch;
    int ox, oy;
    int max_ring, max_spoke;
} GridCtx;

static void ctx_init(GridCtx *g, int mode, int rows, int cols)
{
    memset(g, 0, sizeof *g);
    g->mode = mode; g->rows = rows; g->cols = cols;
    g->cw = CELL_W; g->ch = CELL_H;
    g->ox = cols / 2; g->oy = rows / 2;
    g->max_spoke = 12;
    int half_diag_px = (int)(sqrt((double)(g->ox*g->cw)*(g->ox*g->cw) +
                                   (double)(g->oy*g->ch)*(g->oy*g->ch)));
    g->max_ring = (int)(half_diag_px / 20.0);
}

static void cell_to_polar(int col, int row, int ox, int oy,
                           double *r_px, double *theta)
{
    double dx = (double)(col - ox) * CELL_W;
    double dy = (double)(row - oy) * CELL_H;
    *r_px  = sqrt(dx*dx + dy*dy);
    *theta = atan2(dy, dx);
}

static void polar_to_screen(double r, double theta, int ox, int oy,
                              int *col, int *row)
{
    *col = ox + (int)round(r * cos(theta) / CELL_W);
    *row = oy + (int)round(r * sin(theta) / CELL_H);
}

static char angle_char(double theta)
{
    double a = fmod(theta + 2.0*M_PI, M_PI);
    if (a < M_PI/8.0 || a >= 7.0*M_PI/8.0) return '-';
    if (a < 3.0*M_PI/8.0)                   return '\\';
    if (a < 5.0*M_PI/8.0)                   return '|';
    return '/';
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int row, col; char glyph; bool alive; } Obj;
typedef struct { Obj items[MAX_OBJ]; int count; } Pool;

static void pool_place(Pool *p, int row, int col,
                       int rows, int cols, char glyph)
{
    if (row < 0 || row >= rows-1 || col < 0 || col >= cols) return;
    if (p->count < MAX_OBJ)
        p->items[p->count++] = (Obj){ row, col, glyph, true };
}

static void pool_draw(const Pool *p)
{
    attron(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);
    for (int i = 0; i < p->count; i++) {
        if (!p->items[i].alive) continue;
        mvaddch(p->items[i].row, p->items[i].col,
                (chtype)(unsigned char)p->items[i].glyph);
    }
    attroff(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);
}

static void pool_clear(Pool *p) { p->count = 0; }

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct {
    int    row, col;
    double r, theta;
} Cursor;

static void cursor_sync_polar(Cursor *c, const GridCtx *g)
{
    cell_to_polar(c->col, c->row, g->ox, g->oy, &c->r, &c->theta);
}

static void cursor_reset(Cursor *c, const GridCtx *g)
{
    c->r = 20.0; c->theta = 0.0;
    polar_to_screen(c->r, c->theta, g->ox, g->oy, &c->col, &c->row);
    if (c->row < 0)          c->row = 0;
    if (c->row >= g->rows-1) c->row = g->rows-2;
    if (c->col < 0)          c->col = 0;
    if (c->col >= g->cols)   c->col = g->cols-1;
}

static void cursor_move(Cursor *c, const GridCtx *g, int dr, int dc)
{
    int nr = c->row + dr, nc = c->col + dc;
    if (nr >= 0 && nr < g->rows-1) c->row = nr;
    if (nc >= 0 && nc < g->cols)   c->col = nc;
    cursor_sync_polar(c, g);
}

static void cursor_draw(const Cursor *c, const GridCtx *g)
{
    if (c->row < 0 || c->row >= g->rows-1 || c->col < 0 || c->col >= g->cols)
        return;
    attron(COLOR_PAIR(PAIR_ANCHOR) | A_REVERSE | A_BOLD);
    mvaddch(c->row, c->col, (chtype)'+');
    attroff(COLOR_PAIR(PAIR_ANCHOR) | A_REVERSE | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  mode  (polar background dispatcher)                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static void draw_polar_bg(const GridCtx *g)
{
    int rows = g->rows, cols = g->cols;
    int ox = g->ox, oy = g->oy;
    const double two_pi = 2.0 * M_PI;
    attron(COLOR_PAIR(PAIR_GRID));
    switch (g->mode) {
    case 0: {
        const double sp=20.0,rw=1.6,sw=0.10,sa=two_pi/12.0;
        for(int row=0;row<rows-1;row++) for(int col=0;col<cols;col++){
            double r,th;cell_to_polar(col,row,ox,oy,&r,&th);
            double rp=fmod(r,sp);bool on_r=rp<rw||rp>sp-rw;
            double tn=fmod(th+two_pi,two_pi),sp2=fmod(tn,sa);
            if(on_r||(r>3.0&&(sp2<sw||sp2>sa-sw)))
                mvaddch(row,col,(chtype)(unsigned char)angle_char(th));}
        break;}
    case 1: {
        const double rmin=4.0,ls=0.25,rwu=0.08,sw=0.10,sa=two_pi/12.0;
        for(int row=0;row<rows-1;row++) for(int col=0;col<cols;col++){
            double r,th;cell_to_polar(col,row,ox,oy,&r,&th);
            bool on_r=false;if(r>rmin){double u=log(r/rmin)/ls,fr=u-floor(u);on_r=fr<rwu||fr>1.0-rwu;}
            double tn=fmod(th+two_pi,two_pi),sp2=fmod(tn,sa);
            if(on_r||(r>3.0&&(sp2<sw||sp2>sa-sw)))
                mvaddch(row,col,(chtype)(unsigned char)angle_char(th));}
        break;}
    case 2: {
        const double pitch=32.0,sw=0.20,rmin=3.0;double a=pitch/two_pi;
        for(int row=0;row<rows-1;row++) for(int col=0;col<cols;col++){
            double r,th;cell_to_polar(col,row,ox,oy,&r,&th);if(r<rmin)continue;
            double ph=fmod(2.0*(fmod(th+two_pi,two_pi)-r/a)+2.0*two_pi,two_pi);
            if(ph<sw||ph>two_pi-sw) mvaddch(row,col,(chtype)(unsigned char)angle_char(th));}
        break;}
    case 3: {
        const double growth=2.0*log(PHI)/M_PI,sw=0.22,rmin=4.0;
        for(int row=0;row<rows-1;row++) for(int col=0;col<cols;col++){
            double r,th;cell_to_polar(col,row,ox,oy,&r,&th);if(r<rmin)continue;
            double ph=fmod(2.0*(fmod(th+two_pi,two_pi)-log(r/rmin)/growth)+2.0*two_pi,two_pi);
            if(ph<sw||ph>two_pi-sw) mvaddch(row,col,(chtype)(unsigned char)angle_char(th));}
        break;}
    case 4: {
        const double sp=3.5;bool *vis=calloc((size_t)(rows*cols),1);if(!vis)break;
        for(int i=0;i<N_BG_SEEDS;i++){
            double r=sqrt((double)i)*sp,th=(double)i*GOLDEN_ANGLE;
            int c=ox+(int)round(r*cos(th)/CELL_W),rw=oy+(int)round(r*sin(th)/CELL_H);
            if(rw<0||rw>=rows-1||c<0||c>=cols||vis[rw*cols+c])continue;
            vis[rw*cols+c]=true;mvaddch(rw,c,(chtype)(unsigned char)'o');}
        free(vis);break;}
    case 5: {
        const double ru=18.0,rwf=0.06,sw=0.10,sa=two_pi/12.0;double rusq=ru*ru;
        for(int row=0;row<rows-1;row++) for(int col=0;col<cols;col++){
            double r,th;cell_to_polar(col,row,ox,oy,&r,&th);if(r<3.0)continue;
            double kf=(r*r)/rusq,fr=kf-floor(kf),tn=fmod(th+two_pi,two_pi),sp2=fmod(tn,sa);
            if(fr<rwf||fr>1.0-rwf||sp2<sw||sp2>sa-sw)
                mvaddch(row,col,(chtype)(unsigned char)angle_char(th));}
        break;}
    case 6: {
        const double A=1.6,B=1.0,sp=20.0,rwu=0.07;
        for(int row=0;row<rows-1;row++) for(int col=0;col<cols;col++){
            double dx=(double)(col-ox)*CELL_W,dy=(double)(row-oy)*CELL_H;
            double er=sqrt((dx/A)*(dx/A)+(dy/B)*(dy/B));if(er<0.5)continue;
            double et=atan2(dy/B,dx/A),fr=(er/sp)-floor(er/sp);
            if(fr<rwu||fr>1.0-rwu) mvaddch(row,col,(chtype)(unsigned char)angle_char(et));}
        break;}
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  spiral                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * spiral_place_archim — walk Archimedean spiral r=r0+a×t, placing one dot
 * per DENSITY_STEP radians up to n_turns full turns.
 *
 * THE FORMULA:
 *   a = pitch / (2π)        [radial advance per radian]
 *   r(t) = r0 + a × t      [linear growth]
 *   θ(t) = theta0 + t,  t ∈ [0, n_turns × 2π]
 */
static void spiral_place_archim(Pool *pool, double r0, double theta0,
                                 double pitch, int n_turns, double density,
                                 const GridCtx *g)
{
    double a       = pitch / (2.0 * M_PI);
    double theta_max = (double)n_turns * 2.0 * M_PI;
    for (double t = 0.0; t <= theta_max; t += density) {
        double r  = r0 + a * t;
        double th = theta0 + t;
        int c, row;
        polar_to_screen(r, th, g->ox, g->oy, &c, &row);
        pool_place(pool, row, c, g->rows, g->cols, OBJ_GLYPH);
    }
}

/*
 * spiral_place_log — walk log-spiral r=r0×e^(growth×t), placing one dot
 * per DENSITY_STEP radians up to n_turns full turns.
 *
 * THE FORMULA:
 *   r(t) = r0 × e^(growth × t)   [exponential radial growth]
 *   r0 clamped to ≥1.0: r0=0 would give r=0 for all t (0×eˣ=0 always)
 *   Radius after 1 full turn: r0 × e^(growth × 2π)
 */
static void spiral_place_log(Pool *pool, double r0, double theta0,
                               double growth, int n_turns, double density,
                               const GridCtx *g)
{
    if (r0 < 1.0) r0 = 1.0;
    double theta_max = (double)n_turns * 2.0 * M_PI;
    for (double t = 0.0; t <= theta_max; t += density) {
        double r  = r0 * exp(growth * t);
        double th = theta0 + t;
        int c, row;
        polar_to_screen(r, th, g->ox, g->oy, &c, &row);
        pool_place(pool, row, c, g->rows, g->cols, OBJ_GLYPH);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Bright bold yellow status (top-right) + bold cyan key hints (bottom). */
static void hud_draw(const GridCtx *g, const Pool *p, const Cursor *cur,
                     bool log_mode, double pitch, double growth,
                     int n_turns, double density,
                     int theme, double fps, bool paused)
{
    const char *mode_str = log_mode ? "log-spiral" : "archimedean";
    double param = log_mode ? growth : pitch;
    const char *pname = log_mode ? "g" : "p";
    char buf[96];
    snprintf(buf, sizeof buf,
             " %5.1f fps  r:%.0f  θ:%.0f°  %s:%s=%.2f  n:%d  d:%.2f  objs:%d ",
             fps, cur->r, cur->theta*180.0/M_PI,
             mode_str, pname, param, n_turns, density, p->count);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);
    mvprintw(0, 0, " %-13s %s ", BG_NAMES[g->mode], paused ? "PAUSED" : "");
    attroff(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
        " l:archi  o:log  +/-:param  [/]:turns  ,/.:density"
        "  spc:stamp  C:clear  a/e:bg  t:theme(%d)  q:quit ", theme + 1);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void preview_draw(const GridCtx *g, const Cursor *cur,
                         bool log_mode, double pitch, double growth,
                         int n_turns, double density)
{
    /* live spiral preview — redrawn every frame from current cursor position */
    double r0 = (cur->r < 1.0) ? 1.0 : cur->r;
    double a  = pitch / (2.0 * M_PI);
    double theta_max = (double)n_turns * 2.0 * M_PI;
    attron(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
    for (double t = 0.0; t <= theta_max; t += density) {
        double r  = log_mode ? r0 * exp(growth * t) : r0 + a * t;
        double th = cur->theta + t;
        int c, row;
        polar_to_screen(r, th, g->ox, g->oy, &c, &row);
        if (row >= 0 && row < g->rows-1 && c >= 0 && c < g->cols)
            mvaddch(row, c, (chtype)(unsigned char)OBJ_GLYPH);
    }
    attroff(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Pool *pool, const Cursor *cur,
                       bool log_mode, double pitch, double growth,
                       int n_turns, double density,
                       int theme, double fps, bool paused)
{
    erase();
    draw_polar_bg(g);
    pool_draw(pool);
    preview_draw(g, cur, log_mode, pitch, growth, n_turns, density);
    cursor_draw(cur, g);
    hud_draw(g, pool, cur, log_mode, pitch, growth, n_turns, density,
             theme, fps, paused);
    wnoutrefresh(stdscr); doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §10  screen                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }
static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init(theme); atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §11  app                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running = 1, g_need_resize = 0;
static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);

    int theme = 0;
    screen_init(theme);

    int rows = LINES, cols = COLS;
    GridCtx ctx; ctx_init(&ctx, 0, rows, cols);
    Pool    pool; pool_clear(&pool);
    Cursor  cur; cursor_reset(&cur, &ctx);

    bool   log_mode = false;
    double pitch    = PITCH_DEFAULT;
    double growth   = GROWTH_DEFAULT;
    int    n_turns  = N_TURNS_DEFAULT;
    double density  = DENSITY_DEFAULT;

    bool    paused = false;
    double  fps    = TARGET_FPS;
    int64_t t0     = clock_ns();
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            rows = LINES; cols = COLS;
            ctx_init(&ctx, ctx.mode, rows, cols);
            /* re-anchor cursor: keep (cur.r, cur.theta), recompute cell */
            polar_to_screen(cur.r, cur.theta, ctx.ox, ctx.oy, &cur.col, &cur.row);
            if (cur.row < 0)          cur.row = 0;
            if (cur.row >= ctx.rows-1) cur.row = ctx.rows-2;
            if (cur.col < 0)          cur.col = 0;
            if (cur.col >= ctx.cols)   cur.col = ctx.cols-1;
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'P': paused = !paused; break;
        case 't': theme = (theme+1) % N_THEMES; color_init(theme); break;
        case 'a': ctx_init(&ctx, (ctx.mode-1+N_BG_TYPES) % N_BG_TYPES, rows, cols); break;
        case 'e': ctx_init(&ctx, (ctx.mode+1) % N_BG_TYPES, rows, cols); break;
        case 'l': log_mode = false; break;
        case 'o': log_mode = true;  break;
        case ' ':
            if (log_mode)
                spiral_place_log(&pool, cur.r, cur.theta, growth,
                                  n_turns, density, &ctx);
            else
                spiral_place_archim(&pool, cur.r, cur.theta, pitch,
                                     n_turns, density, &ctx);
            break;
        case '+': case '=':
            if (log_mode) { if (growth < GROWTH_MAX) growth += GROWTH_STEP; }
            else          { if (pitch  < PITCH_MAX)  pitch  += PITCH_STEP; }
            break;
        case '-':
            if (log_mode) { if (growth > GROWTH_MIN) growth -= GROWTH_STEP; }
            else          { if (pitch  > PITCH_MIN)  pitch  -= PITCH_STEP; }
            break;
        case '[':
            if (n_turns > N_TURNS_MIN) n_turns--;
            break;
        case ']':
            if (n_turns < N_TURNS_MAX) n_turns++;
            break;
        case ',':
            if (density < DENSITY_MAX) density += DENSITY_STEP;
            break;
        case '.':
            if (density > DENSITY_MIN) density -= DENSITY_STEP;
            break;
        case 'C': pool_clear(&pool); break;
        case 'r': cursor_reset(&cur, &ctx); break;
        case KEY_UP:    cursor_move(&cur, &ctx, -1,  0); break;
        case KEY_DOWN:  cursor_move(&cur, &ctx, +1,  0); break;
        case KEY_LEFT:  cursor_move(&cur, &ctx,  0, -1); break;
        case KEY_RIGHT: cursor_move(&cur, &ctx,  0, +1); break;
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) + (1e9/(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0 = now;
        if (!paused)
            scene_draw(&ctx, &pool, &cur, log_mode, pitch, growth,
                       n_turns, density, theme, fps, paused);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
