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
 *   §3 color    — theme-switchable color pairs
 *   §4 coords   — cell_to_polar, polar_to_screen, angle_char
 *   §5 bgrid    — draw_polar_bg: 7 inline backgrounds
 *   §6 pool     — ObjectPool: pool_place, pool_draw, pool_clear
 *   §7 spiral   — spiral_place_archim, spiral_place_log
 *   §8 cursor   — cursor_draw
 *   §9 scene    — scene_draw
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
#define PAIR_HUD     3
#define PAIR_LABEL   4
#define PAIR_ANCHOR  5

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
    init_pair(PAIR_HUD,    COLORS>=256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_LABEL,  COLORS>=256 ? 252 : COLOR_WHITE,  -1);
    init_pair(PAIR_ANCHOR, COLORS>=256 ? 220 : COLOR_YELLOW, -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  coords                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

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
/* §5  bgrid                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void draw_polar_bg(int type, int rows, int cols, int ox, int oy)
{
    const double two_pi = 2.0 * M_PI;
    attron(COLOR_PAIR(PAIR_GRID));
    switch (type) {
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
/* §6  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int row, col; char glyph; } PObj;
typedef struct { PObj items[MAX_OBJ]; int count; } ObjPool;

static void pool_place(ObjPool *p, int row, int col,
                       int rows, int cols, char glyph)
{
    if (row < 0 || row >= rows-1 || col < 0 || col >= cols) return;
    if (p->count < MAX_OBJ)
        p->items[p->count++] = (PObj){row, col, glyph};
}

static void pool_draw(const ObjPool *p)
{
    attron(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);
    for (int i = 0; i < p->count; i++)
        mvaddch(p->items[i].row, p->items[i].col,
                (chtype)(unsigned char)p->items[i].glyph);
    attroff(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);
}

static void pool_clear(ObjPool *p) { p->count = 0; }

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  spiral                                                              */
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
static void spiral_place_archim(ObjPool *pool, double r0, double theta0,
                                 double pitch, int n_turns, double density,
                                 int rows, int cols, int ox, int oy)
{
    double a       = pitch / (2.0 * M_PI);
    double theta_max = (double)n_turns * 2.0 * M_PI;
    for (double t = 0.0; t <= theta_max; t += density) {
        double r  = r0 + a * t;
        double th = theta0 + t;
        int c, row;
        polar_to_screen(r, th, ox, oy, &c, &row);
        pool_place(pool, row, c, rows, cols, OBJ_GLYPH);
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
static void spiral_place_log(ObjPool *pool, double r0, double theta0,
                               double growth, int n_turns, double density,
                               int rows, int cols, int ox, int oy)
{
    if (r0 < 1.0) r0 = 1.0;
    double theta_max = (double)n_turns * 2.0 * M_PI;
    for (double t = 0.0; t <= theta_max; t += density) {
        double r  = r0 * exp(growth * t);
        double th = theta0 + t;
        int c, row;
        polar_to_screen(r, th, ox, oy, &c, &row);
        pool_place(pool, row, c, rows, cols, OBJ_GLYPH);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void cursor_draw(int row, int col, int rows, int cols)
{
    if (row < 0 || row >= rows-1 || col < 0 || col >= cols) return;
    attron(COLOR_PAIR(PAIR_ANCHOR) | A_REVERSE | A_BOLD);
    mvaddch(row, col, (chtype)'+');
    attroff(COLOR_PAIR(PAIR_ANCHOR) | A_REVERSE | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void scene_draw(int rows, int cols,
                       const ObjPool *pool, int cur_row, int cur_col,
                       double cur_r, double cur_theta,
                       bool log_mode, double pitch, double growth,
                       int n_turns, double density,
                       int bg_type, int theme, double fps, bool paused)
{
    int ox = cols/2, oy = rows/2;
    erase();
    draw_polar_bg(bg_type, rows, cols, ox, oy);
    pool_draw(pool);

    /* live spiral preview — redrawn every frame from current cursor position */
    {
        double r0 = (cur_r < 1.0) ? 1.0 : cur_r;
        double a  = pitch / (2.0 * M_PI);
        double theta_max = (double)n_turns * 2.0 * M_PI;
        attron(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
        for (double t = 0.0; t <= theta_max; t += density) {
            double r  = log_mode ? r0 * exp(growth * t) : r0 + a * t;
            double th = cur_theta + t;
            int c, row;
            polar_to_screen(r, th, ox, oy, &c, &row);
            if (row >= 0 && row < rows-1 && c >= 0 && c < cols)
                mvaddch(row, c, (chtype)(unsigned char)OBJ_GLYPH);
        }
        attroff(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
    }

    cursor_draw(cur_row, cur_col, rows, cols);

    const char *mode_str = log_mode ? "log-spiral" : "archimedean";
    double param = log_mode ? growth : pitch;
    const char *pname = log_mode ? "g" : "p";
    char buf[80];
    snprintf(buf, sizeof buf, " %.1f fps  r:%.0f  θ:%.0f°  %s:%s=%.2f  n:%d  d:%.2f ",
             fps, cur_r, cur_theta*180.0/M_PI,
             mode_str, pname, param, n_turns, density);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols-(int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);
    mvprintw(0, 0, " %-13s", BG_NAMES[bg_type]);
    attroff(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);

    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(rows-1, 0,
        " l:archi  o:log  +/-:param  [/]:turns  ,/.:density"
        "  spc:stamp  C:clear  a/e:bg  t:theme(%d)  %s  q:quit ",
        theme+1, paused ? "PAUSED" : "running");
    attroff(COLOR_PAIR(PAIR_LABEL));

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
    int ox = cols/2, oy = rows/2;

    int     bg_type  = 0;
    ObjPool pool     = {.count = 0};
    bool    log_mode = false;
    double  pitch    = PITCH_DEFAULT;
    double  growth   = GROWTH_DEFAULT;
    int     n_turns  = N_TURNS_DEFAULT;
    double  density  = DENSITY_DEFAULT;

    double cur_r = 20.0, cur_theta = 0.0;
    int    cur_col, cur_row;
    polar_to_screen(cur_r, cur_theta, ox, oy, &cur_col, &cur_row);

    bool    paused = false;
    double  fps    = TARGET_FPS;
    int64_t t0     = clock_ns();
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            rows = LINES; cols = COLS; ox = cols/2; oy = rows/2;
            /* re-anchor cursor: keep (cur_r, cur_theta), recompute cell */
            polar_to_screen(cur_r, cur_theta, ox, oy, &cur_col, &cur_row);
            if (cur_row < 0)       cur_row = 0;
            if (cur_row >= rows-1) cur_row = rows-2;
            if (cur_col < 0)       cur_col = 0;
            if (cur_col >= cols)   cur_col = cols-1;
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'P': paused = !paused; break;
        case 't': theme = (theme+1) % N_THEMES; color_init(theme); break;
        case 'a': bg_type = (bg_type-1+N_BG_TYPES) % N_BG_TYPES; break;
        case 'e': bg_type = (bg_type+1) % N_BG_TYPES; break;
        case 'l': log_mode = false; break;
        case 'o': log_mode = true;  break;
        case ' ':
            if (log_mode)
                spiral_place_log(&pool, cur_r, cur_theta, growth,
                                  n_turns, density, rows, cols, ox, oy);
            else
                spiral_place_archim(&pool, cur_r, cur_theta, pitch,
                                     n_turns, density, rows, cols, ox, oy);
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
        case 'r':
            cur_r = 20.0; cur_theta = 0.0;
            polar_to_screen(cur_r, cur_theta, ox, oy, &cur_col, &cur_row);
            break;
        case KEY_UP:
            if (cur_row > 0) cur_row--;
            cell_to_polar(cur_col, cur_row, ox, oy, &cur_r, &cur_theta);
            break;
        case KEY_DOWN:
            if (cur_row < rows-2) cur_row++;
            cell_to_polar(cur_col, cur_row, ox, oy, &cur_r, &cur_theta);
            break;
        case KEY_LEFT:
            if (cur_col > 0) cur_col--;
            cell_to_polar(cur_col, cur_row, ox, oy, &cur_r, &cur_theta);
            break;
        case KEY_RIGHT:
            if (cur_col < cols-1) cur_col++;
            cell_to_polar(cur_col, cur_row, ox, oy, &cur_r, &cur_theta);
            break;
        }

        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0  = now;
        if (!paused)
            scene_draw(rows, cols, &pool, cur_row, cur_col, cur_r, cur_theta,
                       log_mode, pitch, growth, n_turns, density,
                       bg_type, theme, fps, paused);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
