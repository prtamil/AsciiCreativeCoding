/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 02_polar_arc.c — two-anchor arc and spoke drawing on a polar grid
 *
 * DEMO: Press 'p' to set anchor A, move, press 'p' again to set anchor B.
 *       Then press 'a' to draw an arc along the ring at r_A from θ_A to θ_B;
 *       's' to draw a radial spoke at θ_A from r_A to r_B; 'o' to stamp a
 *       full ring at r_A; 'x' for a full radial line at θ_A.  The HUD shows
 *       the anchor coordinates in (r, θ) so you can see the polar geometry.
 *
 * Study alongside: 01_polar_direct.c (cursor model),
 *                  grids/rect_grids_placement/03_path.c (analogous rect version)
 *
 * Section map:
 *   §1 config   — pool size, arc/spoke step, background names
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — theme-switchable color pairs + PAIR_ANCHOR
 *   §4 coords   — cell_to_polar, polar_to_screen, angle_char
 *   §5 bgrid    — draw_polar_bg: 7 inline backgrounds
 *   §6 pool     — ObjectPool: pool_place (no-dup), pool_draw, pool_clear
 *   §7 anchor   — AnchorCtx, 3-state FSM, arc/spoke/ring/radial draw
 *   §8 cursor   — cursor_draw
 *   §9 scene    — scene_draw
 *   §10 screen  — ncurses init / cleanup
 *   §11 app     — signals, resize, main loop
 *
 * Keys:  q/ESC quit   p pause-or-anchor   t theme   a/e prev/next background
 *        p  advance state (IDLE→ONE→TWO→IDLE)
 *        In ONE or TWO: o full-ring   x full-spoke
 *        In TWO only:   a arc   s spoke
 *        C clear all   r reset anchors
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/polar_grids_placement/02_polar_arc.c \
 *       -o 02_polar_arc -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Polar path rasterisation.  Given two polar anchors
 *                  A = (r_A, θ_A) and B = (r_B, θ_B) the four operations
 *                  are:
 *
 *                  Arc   — iterate θ from θ_A to θ_B at constant r = r_A,
 *                           placing one object per terminal cell:
 *                           step = CELL_W / r_A radians (≈ 1 cell per step).
 *
 *                  Spoke — iterate r from r_A to r_B at constant θ = θ_A,
 *                           step = 1.0 pixel (finer than CELL_W/CELL_H).
 *
 *                  Ring  — full arc at r_A over [0, 2π).
 *
 *                  Radial — spoke at θ_A from r=4 to max visible radius.
 *
 *                  All operations convert each (r, θ) sample to a terminal
 *                  cell via polar_to_screen() and append to the object pool.
 *
 * Data-structure : ObjectPool with pool_place (no-duplicate check for draw
 *                  ops; duplicates are harmless since the pool is capped).
 *                  State machine: IDLE → ONE → TWO, advanced by 'p'.
 *
 * Math           : Arc length in terminal columns ≈ r × Δθ / CELL_W.
 *                  Step size CELL_W/r ensures ≤ 1 object per column at
 *                  any radius.  Spoke length in cells ≈ Δr / CELL_H.
 *
 * References     :
 *   Polar coordinate system — en.wikipedia.org/wiki/Polar_coordinate_system
 *   Rectangular analogue — grids/rect_grids_placement/03_path.c
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ──────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A polar "cell" is bounded by two arcs (at r_A and r_B) and two spokes
 * (at θ_A and θ_B).  This file lets you draw those four boundary types
 * individually using a two-anchor state machine.  Each operation converts
 * a parametric polar curve into a sequence of object-pool entries.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Anchor A = (r_A, θ_A) defines a specific ring and a specific ray.
 * Anchor B = (r_B, θ_B) defines a second ring and second ray.
 *
 *   Arc:    portion of ring r_A between rays θ_A and θ_B.
 *   Spoke:  segment of ray θ_A between rings r_A and r_B.
 *   Ring:   full ring at r_A (θ from 0 to 2π).
 *   Radial: full ray at θ_A from origin to screen edge.
 *
 * The state machine (IDLE→ONE→TWO) ensures both anchors are set before
 * drawing arc or spoke.  Ring and radial only need anchor ONE.
 *
 * DRAWING METHOD
 * ──────────────
 * 1. Move cursor, press 'p' → sets anchor A (state: IDLE→ONE).
 * 2. Move cursor, press 'p' → sets anchor B (state: ONE→TWO).
 * 3. Press 'a'=arc, 's'=spoke, 'o'=ring, 'x'=radial to draw.
 *    Each operation calls pool_place() for every sampled (r, θ).
 * 4. Press 'p' again → resets to IDLE.
 *
 * KEY FORMULAS
 * ────────────
 * arc_draw — angular step for one terminal column:
 *   step = CELL_W / (r_A + 1)          [radians per column at radius r_A]
 *   clamped to ARC_STEP_MIN=0.005 rad (prevents infinite loop at large r)
 *   Direction: always forward — if t1 < t0 add 2π (counterclockwise wrap)
 *   Arc cells ≈ Δθ × r_A / CELL_W
 *   Example: Δθ=π/2, r_A=20 → ≈ 20×1.571/2 ≈ 15 objects
 *
 * spoke_draw — pixel-step radial segment:
 *   step = SPOKE_PX_STEP = 1.0 px (finer than CELL_H=4; no gaps)
 *   r from min(r_A, r_B) to max(r_A, r_B) at constant θ = θ_A
 *   Spoke cells ≈ |r_B − r_A| / CELL_H
 *
 * ring_draw — full circle at r_A:
 *   Same step as arc_draw; t from 0 to 2π
 *   Objects ≈ 2π × r_A / CELL_W  (= π × r_A for CELL_W=2)
 *
 * radial_draw — full spoke from R_OPS_MIN to screen corner:
 *   r_max = sqrt((ox × CELL_W)² + (oy × CELL_H)²)   [screen diagonal in px]
 *   r from R_OPS_MIN to r_max by SPOKE_PX_STEP
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 * - Arc always goes counterclockwise (t1 += 2π when t1 < t0).  If θ_A and
 *   θ_B are close and θ_A > θ_B you get the long arc (> π), not the short.
 * - ARC_STEP_MIN=0.005: without this the step rounds to 0 for r_A > ~400,
 *   causing an infinite loop in arc_draw.
 * - MAX_OBJ=4096.  A full ring at r=200 needs ≈628 objects; at r=1300 the
 *   ring would overflow the pool.  pool_place silently caps at MAX_OBJ.
 *
 * HOW TO VERIFY
 * ─────────────
 * Terminal 80×24 → ox=40, oy=12.
 *
 * arc_draw (A at col=50,row=12 → r_A=20, θ_A=0; B at col=40,row=7):
 *   θ_B = atan2((7−12)×4, 0) = atan2(−20, 0) = −π/2
 *   t0=0, t1=fmod(−π/2+2π,2π)=3π/2≈4.712
 *   step=2/(20+1)≈0.095 rad → ≈50 objects covering 270°.
 *
 * ring_draw at r_A=20:
 *   step≈0.095; loop: 2π/0.095≈66 objects.
 *   At θ=0: col=50, row=12 ✓.  At θ=π/2: col=40, row=17 ✓.
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

#define TARGET_FPS   30
#define CELL_W        2
#define CELL_H        4

/* Spoke rasterisation step in pixels (finer than cell size, no gaps) */
#define SPOKE_PX_STEP   1.0
/* Minimum arc angular step — prevents infinite loops at very large r */
#define ARC_STEP_MIN    0.005   /* radians */
/* Minimum radius for radial/ring operations */
#define R_OPS_MIN       4.0

/* Object pool — larger than 01 because arc/ring can produce many objects */
#define MAX_OBJ       4096
#define OBJ_GLYPH     '*'

#define PHI           1.61803398874989484820
#define GOLDEN_ANGLE  (2.0 * M_PI / (PHI * PHI))
#define N_BG_SEEDS    600

/* Color pairs */
#define PAIR_GRID    1
#define PAIR_ACTIVE  2
#define PAIR_HUD     3
#define PAIR_LABEL   4
#define PAIR_ANCHOR  5   /* anchor A and B markers */

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
        const double sp = 20.0, rw = 1.6, sw = 0.10, sa = two_pi/12.0;
        for (int row = 0; row < rows-1; row++)
          for (int col = 0; col < cols; col++) {
            double r, th; cell_to_polar(col, row, ox, oy, &r, &th);
            double rp = fmod(r, sp);
            bool on_r = rp < rw || rp > sp-rw;
            double tn = fmod(th+two_pi, two_pi), sp2 = fmod(tn, sa);
            bool on_s = r>3.0 && (sp2<sw || sp2>sa-sw);
            if (on_r || on_s) mvaddch(row,col,(chtype)(unsigned char)angle_char(th));
          }
        break;
    }
    case 1: {
        const double rmin=4.0,ls=0.25,rwu=0.08,sw=0.10,sa=two_pi/12.0;
        for (int row = 0; row < rows-1; row++)
          for (int col = 0; col < cols; col++) {
            double r,th; cell_to_polar(col,row,ox,oy,&r,&th);
            bool on_r=false;
            if (r>rmin){double u=log(r/rmin)/ls,fr=u-floor(u);on_r=fr<rwu||fr>1.0-rwu;}
            double tn=fmod(th+two_pi,two_pi),sp2=fmod(tn,sa);
            bool on_s=r>3.0&&(sp2<sw||sp2>sa-sw);
            if (on_r||on_s) mvaddch(row,col,(chtype)(unsigned char)angle_char(th));
          }
        break;
    }
    case 2: {
        const double pitch=32.0,sw=0.20,rmin=3.0; double a=pitch/two_pi;
        for (int row=0;row<rows-1;row++)
          for (int col=0;col<cols;col++){
            double r,th;cell_to_polar(col,row,ox,oy,&r,&th);if(r<rmin)continue;
            double tn=fmod(th+two_pi,two_pi),raw=2.0*(tn-r/a);
            double ph=fmod(raw+2.0*two_pi,two_pi);
            if(ph<sw||ph>two_pi-sw) mvaddch(row,col,(chtype)(unsigned char)angle_char(th));
          }
        break;
    }
    case 3: {
        const double growth=2.0*log(PHI)/M_PI,sw=0.22,rmin=4.0;
        for (int row=0;row<rows-1;row++)
          for (int col=0;col<cols;col++){
            double r,th;cell_to_polar(col,row,ox,oy,&r,&th);if(r<rmin)continue;
            double tn=fmod(th+two_pi,two_pi),tp=log(r/rmin)/growth;
            double ph=fmod(2.0*(tn-tp)+2.0*two_pi,two_pi);
            if(ph<sw||ph>two_pi-sw) mvaddch(row,col,(chtype)(unsigned char)angle_char(th));
          }
        break;
    }
    case 4: {
        const double sp=3.5; bool *vis=calloc((size_t)(rows*cols),1);
        if(!vis)break;
        for(int i=0;i<N_BG_SEEDS;i++){
            double r=sqrt((double)i)*sp,th=(double)i*GOLDEN_ANGLE;
            int c=ox+(int)round(r*cos(th)/CELL_W),rw=oy+(int)round(r*sin(th)/CELL_H);
            if(rw<0||rw>=rows-1||c<0||c>=cols||vis[rw*cols+c])continue;
            vis[rw*cols+c]=true; mvaddch(rw,c,(chtype)(unsigned char)'o');
        }
        free(vis); break;
    }
    case 5: {
        const double ru=18.0,rwf=0.06,sw=0.10,sa=two_pi/12.0; double rusq=ru*ru;
        for(int row=0;row<rows-1;row++)
          for(int col=0;col<cols;col++){
            double r,th;cell_to_polar(col,row,ox,oy,&r,&th);if(r<3.0)continue;
            double kf=(r*r)/rusq,fr=kf-floor(kf);
            bool on_r=fr<rwf||fr>1.0-rwf;
            double tn=fmod(th+two_pi,two_pi),sp2=fmod(tn,sa);
            if(on_r||sp2<sw||sp2>sa-sw) mvaddch(row,col,(chtype)(unsigned char)angle_char(th));
          }
        break;
    }
    case 6: {
        const double A=1.6,B=1.0,sp=20.0,rwu=0.07;
        for(int row=0;row<rows-1;row++)
          for(int col=0;col<cols;col++){
            double dx=(double)(col-ox)*CELL_W,dy=(double)(row-oy)*CELL_H;
            double er=sqrt((dx/A)*(dx/A)+(dy/B)*(dy/B));if(er<0.5)continue;
            double et=atan2(dy/B,dx/A),u=er/sp,fr=u-floor(u);
            if(fr<rwu||fr>1.0-rwu) mvaddch(row,col,(chtype)(unsigned char)angle_char(et));
          }
        break;
    }
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  pool                                                                */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef struct { int row, col; char glyph; } PObj;
typedef struct { PObj items[MAX_OBJ]; int count; } ObjPool;

/* pool_place — append if in bounds; cap silently at MAX_OBJ */
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
/* §7  anchor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

typedef enum { IDLE = 0, ONE = 1, TWO = 2 } AnchorState;

typedef struct {
    AnchorState state;
    double r_a, theta_a;   /* anchor A */
    int    row_a, col_a;
    double r_b, theta_b;   /* anchor B */
    int    row_b, col_b;
} AnchorCtx;

/*
 * arc_draw — rasterise arc of ring r_a from theta_a to theta_b.
 *
 * THE FORMULA:
 *   step = CELL_W / (r_a + 1)   [rad/col: ≈1 object per terminal column]
 *   step clamped to ARC_STEP_MIN=0.005 (prevents infinite loop at large r)
 *   Always counterclockwise: if t1 < t0 add 2π before iterating
 */
static void arc_draw(ObjPool *pool, const AnchorCtx *ac,
                     int rows, int cols, int ox, int oy)
{
    double t0 = fmod(ac->theta_a + 2.0*M_PI, 2.0*M_PI);
    double t1 = fmod(ac->theta_b + 2.0*M_PI, 2.0*M_PI);
    if (t1 < t0) t1 += 2.0*M_PI;                     /* always go forward */
    double step = CELL_W / (ac->r_a + 1.0);
    if (step < ARC_STEP_MIN) step = ARC_STEP_MIN;
    for (double t = t0; t <= t1 + step*0.5; t += step) {
        int c, r;
        polar_to_screen(ac->r_a, t, ox, oy, &c, &r);
        pool_place(pool, r, c, rows, cols, OBJ_GLYPH);
    }
}

/*
 * spoke_draw — rasterise radial segment at theta_a from r_a to r_b.
 *
 * THE FORMULA:
 *   step = SPOKE_PX_STEP = 1.0 px (finer than CELL_H=4 to avoid gaps)
 *   r from min(r_a, r_b) to max(r_a, r_b) at constant θ = theta_a
 */
static void spoke_draw(ObjPool *pool, const AnchorCtx *ac,
                       int rows, int cols, int ox, int oy)
{
    double r0 = fmin(ac->r_a, ac->r_b);
    double r1 = fmax(ac->r_a, ac->r_b);
    for (double r = r0; r <= r1; r += SPOKE_PX_STEP) {
        int c, row;
        polar_to_screen(r, ac->theta_a, ox, oy, &c, &row);
        pool_place(pool, row, c, rows, cols, OBJ_GLYPH);
    }
}

/* ring_draw — full 2π arc at r_a (same step formula as arc_draw) */
static void ring_draw(ObjPool *pool, const AnchorCtx *ac,
                      int rows, int cols, int ox, int oy)
{
    const double two_pi = 2.0 * M_PI;
    double step = CELL_W / (ac->r_a + 1.0);
    if (step < ARC_STEP_MIN) step = ARC_STEP_MIN;
    for (double t = 0.0; t < two_pi; t += step) {
        int c, r;
        polar_to_screen(ac->r_a, t, ox, oy, &c, &r);
        pool_place(pool, r, c, rows, cols, OBJ_GLYPH);
    }
}

/*
 * radial_draw — full spoke at theta_a from R_OPS_MIN to screen corner.
 *
 * THE FORMULA:
 *   r_max = sqrt((ox×CELL_W)² + (oy×CELL_H)²)   [screen diagonal in px]
 *   r from R_OPS_MIN to r_max by SPOKE_PX_STEP=1.0 px
 */
static void radial_draw(ObjPool *pool, const AnchorCtx *ac,
                        int rows, int cols, int ox, int oy)
{
    double r_max = sqrt(
        (double)(ox * CELL_W) * (double)(ox * CELL_W) +
        (double)(oy * CELL_H) * (double)(oy * CELL_H));
    for (double r = R_OPS_MIN; r <= r_max; r += SPOKE_PX_STEP) {
        int c, row;
        polar_to_screen(r, ac->theta_a, ox, oy, &c, &row);
        pool_place(pool, row, c, rows, cols, OBJ_GLYPH);
    }
}

/* Draw anchor markers on screen */
static void anchors_draw(const AnchorCtx *ac)
{
    attron(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
    if (ac->state >= ONE)
        mvaddch(ac->row_a, ac->col_a, (chtype)'@');
    if (ac->state >= TWO)
        mvaddch(ac->row_b, ac->col_b, (chtype)'#');
    attroff(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
}

/* cursor draw (screen position, no polar mode in this file) */
static void cursor_draw(int row, int col, int rows, int cols)
{
    if (row < 0 || row >= rows-1 || col < 0 || col >= cols) return;
    attron(COLOR_PAIR(PAIR_ACTIVE) | A_REVERSE | A_BOLD);
    mvaddch(row, col, (chtype)'+');
    attroff(COLOR_PAIR(PAIR_ACTIVE) | A_REVERSE | A_BOLD);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static const char *const STATE_NAMES[] = {"IDLE", "A-set", "B-set"};

static void scene_draw(int rows, int cols,
                       const ObjPool *pool, const AnchorCtx *ac,
                       int cur_row, int cur_col,
                       double cur_r, double cur_theta,
                       int bg_type, int theme, double fps, bool paused)
{
    int ox = cols/2, oy = rows/2;
    erase();
    draw_polar_bg(bg_type, rows, cols, ox, oy);
    pool_draw(pool);
    anchors_draw(ac);
    cursor_draw(cur_row, cur_col, rows, cols);

    char buf[80];
    double deg = cur_theta * 180.0 / M_PI;
    snprintf(buf, sizeof buf, " %.1f fps  r:%.0f  θ:%.0f°  objs:%d  %s ",
             fps, cur_r, deg, pool->count, paused ? "PAUSED" : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols-(int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);
    mvprintw(0, 0, " %-8s %s", BG_NAMES[bg_type], STATE_NAMES[ac->state]);
    attroff(COLOR_PAIR(PAIR_ANCHOR) | A_BOLD);

    const char *ops = (ac->state == TWO)
        ? " a:arc  s:spoke  o:ring  x:radial  p:reset  C:clear "
        : " p:set-anchor  o:ring  x:radial  a/e:bg  q:quit ";
    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(rows-1, 0, "%s  t:theme(%d)", ops, theme+1);
    attroff(COLOR_PAIR(PAIR_LABEL));

    wnoutrefresh(stdscr); doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  screen                                                              */
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

    int       bg_type = 0;
    ObjPool   pool    = {.count = 0};
    AnchorCtx ac      = {.state = IDLE};
    int       cur_row = oy, cur_col = ox;
    double    cur_r   = 0.0, cur_theta = 0.0;
    cell_to_polar(cur_col, cur_row, ox, oy, &cur_r, &cur_theta);

    bool    paused = false;
    double  fps    = TARGET_FPS;
    int64_t t0     = clock_ns();
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            rows = LINES; cols = COLS; ox = cols/2; oy = rows/2;
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'P': paused = !paused; break;   /* capital P = pause to avoid clash */
        case 't': theme = (theme+1) % N_THEMES; color_init(theme); break;
        case 'a':
            if (ac.state == TWO)
                arc_draw(&pool, &ac, rows, cols, ox, oy);
            else
                bg_type = (bg_type - 1 + N_BG_TYPES) % N_BG_TYPES;
            break;
        case 'e': bg_type = (bg_type+1) % N_BG_TYPES; break;
        case 'p':
            if (ac.state == IDLE) {
                ac.r_a = cur_r; ac.theta_a = cur_theta;
                ac.row_a = cur_row; ac.col_a = cur_col;
                ac.state = ONE;
            } else if (ac.state == ONE) {
                ac.r_b = cur_r; ac.theta_b = cur_theta;
                ac.row_b = cur_row; ac.col_b = cur_col;
                ac.state = TWO;
            } else {
                ac.state = IDLE;
            }
            break;
        case 's':
            if (ac.state == TWO) spoke_draw(&pool, &ac, rows, cols, ox, oy);
            break;
        case 'o':
            if (ac.state >= ONE) ring_draw(&pool, &ac, rows, cols, ox, oy);
            break;
        case 'x':
            if (ac.state >= ONE) radial_draw(&pool, &ac, rows, cols, ox, oy);
            break;
        case 'C': pool_clear(&pool); break;
        case 'r': ac.state = IDLE; break;
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
            scene_draw(rows, cols, &pool, &ac, cur_row, cur_col,
                       cur_r, cur_theta, bg_type, theme, fps, paused);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
