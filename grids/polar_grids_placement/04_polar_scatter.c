/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 04_polar_scatter.c — four polar scatter strategies on a polar grid
 *
 * DEMO: Move the cursor and watch a live scatter preview update in real time.
 *       U/G/W/D select the distribution: U uniform-area, G radial Gaussian
 *       (clusters around the cursor ring), W angular wedge (fills the cursor
 *       sector), D ring-snap (objects gravitate to ring boundaries).  The
 *       preview regenerates whenever the cursor moves or a parameter changes.
 *       Press space to stamp the current preview into the permanent pool.
 *
 * Study alongside: 03_polar_spiral.c (parametric path placement),
 *                  grids/rect_grids_placement/04_scatter.c (rectangular version)
 *
 * Section map:
 *   §1 config   — scatter parameters: N, radii, sigma, wedge half-angle
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — theme-switchable color pairs
 *   §4 coords   — cell_to_polar, polar_to_screen, angle_char
 *   §5 bgrid    — draw_polar_bg: 7 inline backgrounds
 *   §6 pool     — ObjectPool: pool_place, pool_draw, pool_clear
 *   §7 scatter  — gauss, scatter_uniform, scatter_gauss, scatter_wedge,
 *                  scatter_ringsnap
 *   §8 cursor   — cursor_draw
 *   §9 scene    — scene_draw
 *   §10 screen  — ncurses init / cleanup
 *   §11 app     — signals, resize, main loop
 *
 * Keys:  q/ESC quit   P pause   t theme   a/e prev/next background
 *        U/G/W/D select scatter mode (preview updates live)
 *        +/- N    [/] sigma (G) or wedge (W)
 *        space stamp preview to pool    C clear pool    r reset cursor
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/polar_grids_placement/04_polar_scatter.c \
 *       -o 04_polar_scatter -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Four scatter strategies that differ only in how (r, θ)
 *                  is sampled.  Each converts the sample to a terminal cell
 *                  via polar_to_screen() and appends to the pool.
 *
 *                  U — Uniform area:
 *                    r  ~ sqrt(rand(r_min², r_max²))  — corrects radial bias
 *                    θ  ~ Uniform[0, 2π)
 *                    Without the sqrt correction, sampling r ~ Uniform
 *                    would over-weight the centre (small r has less area
 *                    per unit r but the same sampling probability).
 *
 *                  G — Radial Gaussian:
 *                    r  ~ |Normal(r_cursor, SIGMA_R)|
 *                    θ  ~ Uniform[0, 2π)
 *                    Objects cluster around the ring at radius r_cursor.
 *                    Box-Muller transform for the Normal deviate.
 *
 *                  W — Angular wedge:
 *                    r  ~ Uniform[R_INNER, R_OUTER]
 *                    θ  ~ Uniform[θ_cursor − WEDGE_HALF, θ_cursor + WEDGE_HALF]
 *                    Objects fill a pie-slice centred on the cursor angle.
 *
 *                  D — Ring-snap:
 *                    r  ~ nearest ring position k × RING_SNAP_SP + jitter
 *                    θ  ~ Uniform[0, 2π)
 *                    Objects appear to "snap" to ring boundaries.
 *
 * Math           : Uniform area correction: the disc of radius R has area
 *                  π × R².  Uniform sampling of r² (not r) gives uniform
 *                  areal density.  CDF inversion: r = sqrt(r_min² + rand × (r_max² − r_min²)).
 *
 *                  Box-Muller: U1, U2 ~ Uniform[0,1] →
 *                    Z = sqrt(−2 ln U1) × cos(2π U2)  ~ Normal(0,1)
 *
 * References     :
 *   Uniform disc sampling — en.wikipedia.org/wiki/Disk_sampling
 *   Box-Muller transform — en.wikipedia.org/wiki/Box-Muller_transform
 *   Rectangular analogue — grids/rect_grids_placement/04_scatter.c
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ──────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Four polar scatter strategies that differ only in how (r, θ) is sampled.
 * Every strategy converts its sample to a terminal cell via polar_to_screen()
 * and appends to the preview pool.  The dirty flag triggers regeneration
 * whenever cursor or parameters change; SPACE stamps preview → permanent pool.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine throwing darts at the polar grid with different aiming strategies:
 *
 *   U: blindfolded — completely random, but corrected so every ring band
 *      gets proportional coverage (outer bands are larger, so they get
 *      more darts when sampling uniformly over area, not over radius).
 *   G: aiming for a ring — most darts land near r_cursor with Gaussian
 *      falloff; the further from that ring the rarer the dart.
 *   W: aiming at a clock sector — darts land in a ±WEDGE_HALF wedge
 *      around the cursor angle.
 *   D: magnetised darts — they snap to ring boundaries regardless of aim.
 *
 * DRAWING METHOD
 * ──────────────
 * 1. Move cursor to set the scatter centre (r_cursor, θ_cursor).
 * 2. Select mode U/G/W/D; adjust N with +/−, param with [/].
 * 3. dirty=true triggers pool_clear(preview) + scatter_*(preview, ...).
 *    Preview drawn in amber (PAIR_ANCHOR) each frame.
 * 4. SPACE: pool_stamp copies preview → permanent pool (no regen needed).
 *
 * KEY FORMULAS
 * ────────────
 * gauss — Box-Muller transform:
 *   U1 = (rand+1)/(RAND_MAX+2) ∈ (0,1)   [+1 shift avoids log(0)]
 *   U2 = rand/(RAND_MAX+1)    ∈ [0,1)
 *   Z  = sqrt(−2 ln U1) × cos(2π U2)     ∼ Normal(0,1)
 *   return mean + sigma × Z
 *
 * scatter_uniform (U) — CDF-inversion for uniform areal density:
 *   Without correction, r~Uniform gives 4× more dots near r_min than r_max.
 *   Area of disc = π r², so sampling r² uniformly → uniform areal density.
 *   r = sqrt(r_min² + rand × (r_max² − r_min²))
 *   θ ∈ [0, 2π) uniformly
 *
 * scatter_gauss (G) — Gaussian ring cluster:
 *   r = |Normal(r_cursor, SIGMA_R)|   [fabs keeps r > 0]
 *   r clamped to R_INNER
 *   θ ∈ [0, 2π) uniformly
 *   Slight pile-up at R_INNER when r_cursor is small and sigma is large.
 *
 * scatter_wedge (W) — angular wedge:
 *   r ∈ [R_INNER, r_outer] uniformly (no area correction — fills wedge)
 *   θ ∈ [θ_cursor − WEDGE_HALF, θ_cursor + WEDGE_HALF] uniformly
 *
 * scatter_ringsnap (D) — ring-snap with jitter:
 *   k ∈ {1, …, floor(r_outer / RING_SNAP_SP)} uniformly (integer)
 *   r = k × RING_SNAP_SP + jitter,  jitter ∈ [−RING_SNAP_JITTER, +JITTER]
 *   θ ∈ [0, 2π) uniformly
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 * - dirty flag: preview regenerates on cursor move, key, param change.
 *   pool_stamp (SPACE) does NOT set dirty, so repeated SPACE stamps the
 *   same pattern without regeneration (preserves expected behaviour).
 * - R_OUTER_FACTOR=0.42: on a large terminal (200×50) r_outer can exceed
 *   200px; with U mode the pool fills in one press.
 * - scatter_gauss: fabs(gauss(...)) causes pile-up at R_INNER when
 *   r_cursor is small.  This is visible as a bright spot at the inner radius.
 *
 * HOW TO VERIFY
 * ─────────────
 * Terminal 80×24 → ox=40, oy=12.
 * r_outer = 0.42 × sqrt((40×2)²+(12×4)²) = 0.42×sqrt(6400+2304) ≈ 39px
 *
 * scatter_uniform (U, N=1):
 *   r_min=R_INNER=8, r_max≈39, r_min²=64, r_max²≈1521, range≈1457
 *   rand=0.25: r=sqrt(64+0.25×1457)=sqrt(428)≈20.7; θ random  ✓
 *
 * scatter_ringsnap (D, r_outer≈39):
 *   n_rings=floor(39/20)=1 → only ring 1 (r≈20) available.
 *   All dots cluster near r=20±RING_SNAP_JITTER=±2px.
 *   On larger terminal (r_outer=80): n_rings=4; rings at 20,40,60,80px. ✓
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

/* Objects placed per key press */
#define N_SCATTER_DEFAULT  60
#define N_SCATTER_MIN      10
#define N_SCATTER_MAX     400
#define N_SCATTER_STEP     10

/* Radial bounds for U and W scatter */
#define R_INNER           8.0    /* pixels, minimum scatter radius */
#define R_OUTER_FACTOR    0.42   /* fraction of half-screen diagonal */

/* Radial Gaussian sigma (adjustable) */
#define SIGMA_R_DEFAULT   20.0
#define SIGMA_R_MIN        5.0
#define SIGMA_R_MAX       60.0
#define SIGMA_R_STEP       5.0

/* Angular wedge half-angle (adjustable, same variable as sigma in context) */
#define WEDGE_DEFAULT     (M_PI / 6.0)   /* 30° either side = 60° total  */
#define WEDGE_MIN         (M_PI / 18.0)  /* 10° */
#define WEDGE_MAX         (M_PI)         /* 180° */
#define WEDGE_STEP        (M_PI / 18.0)  /* 10° per step */

/* Ring-snap: spacing between snap targets (pixels) */
#define RING_SNAP_SP      20.0
#define RING_SNAP_JITTER   2.0   /* ± pixel jitter around each ring */

/* Object pool */
#define MAX_OBJ        2048
#define OBJ_GLYPH      'o'

#define PHI            1.61803398874989484820
#define GOLDEN_ANGLE  (2.0 * M_PI / (PHI * PHI))
#define N_BG_SEEDS     600

#define PAIR_GRID    1
#define PAIR_ACTIVE  2
#define PAIR_HUD     3
#define PAIR_LABEL   4
#define PAIR_ANCHOR  5   /* amber — live scatter preview */

static const char *const SCATTER_NAMES[] = {
    "uniform", "radial-gauss", "wedge", "ring-snap",
};

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
            double rp=fmod(r,sp),tn=fmod(th+two_pi,two_pi),sp2=fmod(tn,sa);
            if(rp<rw||rp>sp-rw||(r>3.0&&(sp2<sw||sp2>sa-sw)))
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

static void pool_draw_preview(const ObjPool *p)
{
    attron(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
    for (int i = 0; i < p->count; i++)
        mvaddch(p->items[i].row, p->items[i].col,
                (chtype)(unsigned char)p->items[i].glyph);
    attroff(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
}

/* pool_stamp — append all preview objects into the permanent pool */
static void pool_stamp(ObjPool *dst, const ObjPool *src)
{
    for (int i = 0; i < src->count && dst->count < MAX_OBJ; i++)
        dst->items[dst->count++] = src->items[i];
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  scatter                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * gauss — Box-Muller transform: Normal(mean, sigma) from two uniforms.
 *
 * THE FORMULA:
 *   U1 = (rand+1)/(RAND_MAX+2) ∈ (0,1)   [+1 avoids log(0)]
 *   U2 = rand/(RAND_MAX+1)    ∈ [0,1)
 *   Z  = sqrt(−2 ln U1) × cos(2π U2)     ~ Normal(0,1)
 *   return mean + sigma × Z
 */
static double gauss(double mean, double sigma)
{
    double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = (double)rand() / ((double)RAND_MAX + 1.0);
    double z  = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
    return mean + sigma * z;
}

/*
 * scatter_uniform (U) — CDF-inversion for uniform areal density.
 *
 * THE FORMULA:
 *   Area ∝ r², so sampling r² uniformly → uniform dots per unit area.
 *   r = sqrt(R_INNER² + rand × (r_outer² − R_INNER²))
 *   θ ∈ [0, 2π) uniformly
 */
static void scatter_uniform(ObjPool *pool, int n,
                             double r_outer,
                             int rows, int cols, int ox, int oy)
{
    double r0sq = R_INNER  * R_INNER;
    double r1sq = r_outer  * r_outer;
    for (int i = 0; i < n; i++) {
        double r  = sqrt(r0sq + ((double)rand()/(double)RAND_MAX) * (r1sq - r0sq));
        double th = ((double)rand()/(double)RAND_MAX) * 2.0 * M_PI;
        int c, row;
        polar_to_screen(r, th, ox, oy, &c, &row);
        pool_place(pool, row, c, rows, cols, OBJ_GLYPH);
    }
}

/*
 * scatter_gauss (G) — Gaussian ring cluster.
 *
 * THE FORMULA:
 *   r = |Normal(r_cursor, sigma)|   [fabs keeps r positive]
 *   r clamped to R_INNER minimum
 *   θ ∈ [0, 2π) uniformly
 */
static void scatter_gauss(ObjPool *pool, int n,
                           double r_cursor, double sigma,
                           int rows, int cols, int ox, int oy)
{
    for (int i = 0; i < n; i++) {
        double r  = fabs(gauss(r_cursor, sigma));
        if (r < R_INNER) r = R_INNER;
        double th = ((double)rand()/(double)RAND_MAX) * 2.0 * M_PI;
        int c, row;
        polar_to_screen(r, th, ox, oy, &c, &row);
        pool_place(pool, row, c, rows, cols, OBJ_GLYPH);
    }
}

/*
 * scatter_wedge (W) — angular pie-slice filled uniformly.
 *
 * THE FORMULA:
 *   r ∈ [R_INNER, r_outer] uniformly (no area correction — fills the wedge)
 *   θ ∈ [theta_cursor − wedge_half, theta_cursor + wedge_half] uniformly
 */
static void scatter_wedge(ObjPool *pool, int n,
                           double r_outer, double theta_cursor,
                           double wedge_half,
                           int rows, int cols, int ox, int oy)
{
    for (int i = 0; i < n; i++) {
        double r  = R_INNER + ((double)rand()/(double)RAND_MAX) * (r_outer - R_INNER);
        double th = theta_cursor - wedge_half
                    + ((double)rand()/(double)RAND_MAX) * 2.0 * wedge_half;
        int c, row;
        polar_to_screen(r, th, ox, oy, &c, &row);
        pool_place(pool, row, c, rows, cols, OBJ_GLYPH);
    }
}

/*
 * scatter_ringsnap (D) — objects snap to ring boundaries with jitter.
 *
 * THE FORMULA:
 *   k ∈ {1, …, floor(r_outer / RING_SNAP_SP)} uniformly (integer)
 *   r = k × RING_SNAP_SP + jitter,  jitter ∈ [−RING_SNAP_JITTER, +JITTER]
 *   θ ∈ [0, 2π) uniformly
 */
static void scatter_ringsnap(ObjPool *pool, int n,
                              double r_outer,
                              int rows, int cols, int ox, int oy)
{
    int n_rings = (int)(r_outer / RING_SNAP_SP);
    if (n_rings < 1) n_rings = 1;
    for (int i = 0; i < n; i++) {
        int   k   = 1 + rand() % n_rings;
        double r  = k * RING_SNAP_SP
                    + ((double)rand()/(double)RAND_MAX - 0.5) * 2.0 * RING_SNAP_JITTER;
        if (r < R_INNER) r = R_INNER;
        double th = ((double)rand()/(double)RAND_MAX) * 2.0 * M_PI;
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
    attron(COLOR_PAIR(PAIR_ACTIVE) | A_REVERSE | A_BOLD);
    mvaddch(row, col, (chtype)'+');
    attroff(COLOR_PAIR(PAIR_ACTIVE) | A_REVERSE | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void scene_draw(int rows, int cols,
                       const ObjPool *pool, const ObjPool *preview,
                       int cur_row, int cur_col,
                       double cur_r, double cur_theta,
                       int scatter_type, int n, double param,
                       int bg_type, int theme, double fps, bool paused)
{
    int ox = cols/2, oy = rows/2;
    erase();
    draw_polar_bg(bg_type, rows, cols, ox, oy);
    pool_draw(pool);
    pool_draw_preview(preview);
    cursor_draw(cur_row, cur_col, rows, cols);

    char buf[80];
    snprintf(buf, sizeof buf, " %.1f fps  r:%.0f  θ:%.0f°  [%s]  n:%d  p:%.1f  %s ",
             fps, cur_r, cur_theta*180.0/M_PI, SCATTER_NAMES[scatter_type],
             n, param, paused ? "PAUSED" : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols-(int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);
    mvprintw(0, 0, " %-13s", BG_NAMES[bg_type]);
    attroff(COLOR_PAIR(PAIR_ACTIVE) | A_BOLD);

    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(rows-1, 0,
        " U/G/W/D:mode  +/-:N  [/]:param  spc:stamp  C:clear"
        "  a/e:bg  t:theme(%d)  q:quit ", theme+1);
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
    srand((unsigned)time(NULL));
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

    int     bg_type      = 0;
    int     scatter_type = 0;   /* 0=U 1=G 2=W 3=D */
    ObjPool pool         = {.count = 0};
    ObjPool preview      = {.count = 0};
    bool    dirty        = true;   /* regenerate preview before first draw */
    int     n_scat       = N_SCATTER_DEFAULT;
    double  sigma        = SIGMA_R_DEFAULT;
    double  wedge        = WEDGE_DEFAULT;

    int    cur_col = ox + (int)round(20.0 / CELL_W);
    int    cur_row = oy;
    double cur_r, cur_theta;
    cell_to_polar(cur_col, cur_row, ox, oy, &cur_r, &cur_theta);

    bool    paused = false;
    double  fps    = TARGET_FPS;
    int64_t t0     = clock_ns();
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            rows = LINES; cols = COLS; ox = cols/2; oy = rows/2;
            cell_to_polar(cur_col, cur_row, ox, oy, &cur_r, &cur_theta);
            dirty = true;
        }

        double r_outer = sqrt((double)(ox*CELL_W)*(double)(ox*CELL_W) +
                              (double)(oy*CELL_H)*(double)(oy*CELL_H))
                         * R_OUTER_FACTOR;

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'P': paused = !paused; break;
        case 't': theme = (theme+1) % N_THEMES; color_init(theme); break;
        case 'a': bg_type = (bg_type-1+N_BG_TYPES) % N_BG_TYPES; break;
        case 'e': bg_type = (bg_type+1) % N_BG_TYPES; break;
        case 'U': scatter_type = 0; dirty = true; break;
        case 'G': scatter_type = 1; dirty = true; break;
        case 'W': scatter_type = 2; dirty = true; break;
        case 'D': scatter_type = 3; dirty = true; break;
        case ' ': pool_stamp(&pool, &preview); break;
        case '+': case '=':
            if (n_scat < N_SCATTER_MAX) { n_scat += N_SCATTER_STEP; dirty = true; }
            break;
        case '-':
            if (n_scat > N_SCATTER_MIN) { n_scat -= N_SCATTER_STEP; dirty = true; }
            break;
        case '[':
            if (scatter_type == 1 && sigma > SIGMA_R_MIN) { sigma -= SIGMA_R_STEP; dirty = true; }
            if (scatter_type == 2 && wedge > WEDGE_MIN)   { wedge -= WEDGE_STEP;   dirty = true; }
            break;
        case ']':
            if (scatter_type == 1 && sigma < SIGMA_R_MAX) { sigma += SIGMA_R_STEP; dirty = true; }
            if (scatter_type == 2 && wedge < WEDGE_MAX)   { wedge += WEDGE_STEP;   dirty = true; }
            break;
        case 'C': pool_clear(&pool); break;
        case 'r':
            cur_col = ox + (int)round(20.0 / CELL_W); cur_row = oy;
            cell_to_polar(cur_col, cur_row, ox, oy, &cur_r, &cur_theta);
            dirty = true;
            break;
        case KEY_UP:
            if (cur_row > 0) { cur_row--; cell_to_polar(cur_col, cur_row, ox, oy, &cur_r, &cur_theta); dirty = true; }
            break;
        case KEY_DOWN:
            if (cur_row < rows-2) { cur_row++; cell_to_polar(cur_col, cur_row, ox, oy, &cur_r, &cur_theta); dirty = true; }
            break;
        case KEY_LEFT:
            if (cur_col > 0) { cur_col--; cell_to_polar(cur_col, cur_row, ox, oy, &cur_r, &cur_theta); dirty = true; }
            break;
        case KEY_RIGHT:
            if (cur_col < cols-1) { cur_col++; cell_to_polar(cur_col, cur_row, ox, oy, &cur_r, &cur_theta); dirty = true; }
            break;
        }

        /* regenerate preview only when cursor or params changed */
        if (dirty) {
            pool_clear(&preview);
            switch (scatter_type) {
            case 0: scatter_uniform(&preview, n_scat, r_outer, rows, cols, ox, oy); break;
            case 1: scatter_gauss  (&preview, n_scat, cur_r, sigma, rows, cols, ox, oy); break;
            case 2: scatter_wedge  (&preview, n_scat, r_outer, cur_theta, wedge, rows, cols, ox, oy); break;
            case 3: scatter_ringsnap(&preview, n_scat, r_outer, rows, cols, ox, oy); break;
            }
            dirty = false;
        }

        double param = (scatter_type == 1) ? sigma
                     : (scatter_type == 2) ? wedge * 180.0 / M_PI
                     : 0.0;

        int64_t now = clock_ns();
        fps = fps * 0.95 + (1e9 / (double)(now - t0 + 1)) * 0.05;
        t0  = now;
        if (!paused)
            scene_draw(rows, cols, &pool, &preview, cur_row, cur_col,
                       cur_r, cur_theta, scatter_type, n_scat, param,
                       bg_type, theme, fps, paused);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
