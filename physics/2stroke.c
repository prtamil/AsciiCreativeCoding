/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 2stroke.c  —  2-stroke engine cross-section animation
 *
 * Visualises a 2-stroke internal-combustion engine in cross-section.
 * The crank angle θ advances each tick via slider-crank kinematics.
 * Piston, connecting rod, and crankshaft are drawn each frame.  Exhaust
 * and transfer ports open / close as the piston passes them.  A spark
 * fires at TDC.  A phase label updates throughout the cycle.
 *
 * 2-stroke cycle  (θ = 0 at TDC, positive direction = CW looking right):
 *
 *   θ =   0°  TDC  — spark fires → IGNITION
 *   θ = 0-75° POWER stroke  (ports sealed)
 *   θ ≈  75°  exhaust port uncovers    → EXHAUST
 *   θ ≈  90°  transfer port uncovers  → SCAVENGING
 *   θ = 180°  BDC  — both ports fully open
 *   θ ≈ 270°  transfer port covers
 *   θ ≈ 285°  exhaust port covers     → COMPRESSION
 *   θ = 360°  TDC  — cycle repeats
 *
 * Engine geometry (cell units, y increases downward):
 *   Crank radius    CRANK_R  = 4   (stroke = 8 cells)
 *   Connecting rod  CONROD_L = 9
 *   Bore half-width CYL_IHW  = 6
 *   Piston height   PISTON_H = 3
 *
 * Keys:
 *   q / ESC   quit
 *   space/p   pause / resume
 *   r         reset to BDC
 *   ] [       RPM up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra Artistic/2stroke.c -o 2stroke -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  engine  (kinematics, phase, tick)
 *   §5  draw    (primitives, scene_draw)
 *   §6  scene   (owns engine)
 *   §7  screen  (ncurses init, HUD)
 *   §8  app     (main loop, input, resize)
 */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_DEFAULT = 120,
    SIM_FPS_MIN     =  20,
    SIM_FPS_MAX     = 300,

    RPM_DEFAULT     = 120,
    RPM_MIN         =  30,
    RPM_MAX         = 600,
    RPM_STEP        =  30,

    HUD_COLS        =  60,
    FPS_UPDATE_MS   = 500,
};

/*
 * Engine geometry (all in cell units).
 *
 * Slider-crank at TDC (θ=0):
 *   crank_pin_row = crank_centre_row - CRANK_R
 *   wrist_pin_row = crank_pin_row    - CONROD_L
 *   crown_row     = wrist_pin_row    - (PISTON_H - 1)
 *
 * Setting crown@TDC = engine_top + HEAD_H:
 *   CRANK_CENTER_OFF = HEAD_H + (PISTON_H-1) + CONROD_L + CRANK_R
 *                    = 1      +  2            +  9       +  4      = 16
 *
 * Stroke = 2 * CRANK_R = 8 cells.
 * Crown ranges: engine_top+1 (TDC) … engine_top+9 (BDC).
 */
#define CYL_IHW         6    /* cylinder bore inner half-width           */
#define CYL_WALL        1    /* cylinder wall thickness                  */
#define HEAD_H          1    /* cylinder head height (rows)              */
#define PISTON_H        3    /* piston height (rows)                     */
#define CRANK_R         4    /* crank throw radius (cells)               */
#define CONROD_L        9    /* connecting rod length (cells)            */
#define CRANK_CENTER_OFF (HEAD_H + (PISTON_H-1) + CONROD_L + CRANK_R)

/*
 * Port positions — row offset from engine_top.
 * A port is open when the piston crown has moved below it:
 *   ex_open = crown_row > engine_top + EX_PORT_OFF
 *
 * Crown@TDC=1, Crown@BDC=9 → exhaust opens at (6-1)/(9-1) = 62% stroke,
 * transfer opens at ~75%.
 */
#define EX_PORT_OFF      6
#define TR_PORT_OFF      7

/* Crankcase geometry */
#define CASE_TOP_OFF    12   /* crankcase top = engine_top + 12          */
#define CASE_BOT_OFF    21   /* crankcase bottom = engine_top + 21       */
#define CASE_HW          9   /* crankcase half-width                     */

#define ENGINE_H        (CASE_BOT_OFF + 2)   /* total rows = 23          */

/*
 * IGNITE_WINDOW — half-angle (rad) around TDC where spark shows and
 * phase is labelled IGNITION.  ~17° either side of TDC.
 */
#define IGNITE_WINDOW   0.30f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS     1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

typedef enum {
    CP_WALL    = 1,   /* cylinder walls, head, crankcase  */
    CP_PISTON  = 2,   /* piston body                      */
    CP_CONROD  = 3,   /* connecting rod                   */
    CP_CRANK   = 4,   /* crankshaft                       */
    CP_FIRE    = 5,   /* combustion / hot gas             */
    CP_EXHAUST = 6,   /* exhaust gas                      */
    CP_INTAKE  = 7,   /* fresh charge                     */
    CP_SPARK   = 8,   /* spark flash                      */
    CP_HUD     = 9,   /* HUD text                         */
    CP_PHASE   = 10,  /* phase name                       */
} ColorPair;

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(CP_WALL,    252, COLOR_BLACK);   /* light grey   */
        init_pair(CP_PISTON,   68, COLOR_BLACK);   /* steel blue   */
        init_pair(CP_CONROD,  248, COLOR_BLACK);   /* silver       */
        init_pair(CP_CRANK,   220, COLOR_BLACK);   /* gold         */
        init_pair(CP_FIRE,    196, COLOR_BLACK);   /* bright red   */
        init_pair(CP_EXHAUST, 244, COLOR_BLACK);   /* smoke grey   */
        init_pair(CP_INTAKE,   51, COLOR_BLACK);   /* cyan         */
        init_pair(CP_SPARK,   231, COLOR_BLACK);   /* white        */
        init_pair(CP_HUD,     231, COLOR_BLACK);
        init_pair(CP_PHASE,   214, COLOR_BLACK);   /* orange       */
    } else {
        init_pair(CP_WALL,    COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_PISTON,  COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_CONROD,  COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_CRANK,   COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_FIRE,    COLOR_RED,     COLOR_BLACK);
        init_pair(CP_EXHAUST, COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_INTAKE,  COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_SPARK,   COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_HUD,     COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_PHASE,   COLOR_YELLOW,  COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  engine — kinematics, phase, tick                                  */
/* ===================================================================== */

typedef enum {
    PHASE_COMPRESS,
    PHASE_IGNITE,
    PHASE_POWER,
    PHASE_EXHAUST,
    PHASE_SCAVENGE,
} Phase;

static const char *phase_name(Phase p)
{
    switch (p) {
    case PHASE_COMPRESS: return "COMPRESSION";
    case PHASE_IGNITE:   return "IGNITION   ";
    case PHASE_POWER:    return "POWER      ";
    case PHASE_EXHAUST:  return "EXHAUST    ";
    case PHASE_SCAVENGE: return "SCAVENGING ";
    default:             return "           ";
    }
}

typedef struct {
    float theta;   /* crank angle (rad), 0 = TDC, increases CW */
    int   rpm;
    bool  paused;
} Engine;

static void engine_reset(Engine *e)
{
    e->theta  = (float)M_PI;   /* start at BDC so first revolution is visible */
    e->rpm    = RPM_DEFAULT;
    e->paused = false;
}

/*
 * Slider-crank kinematics.
 *
 * cc_row, cc_col — crank centre (floating-point cell coords).
 *
 * Outputs (all floating-point cell coords):
 *   *crown — piston crown row
 *   *wp    — wrist pin row (= crown + PISTON_H - 1)
 *   *cp_r  — crank pin row
 *   *cp_c  — crank pin col
 */
static void engine_kinematics(float theta, float cc_row, float cc_col,
                               float *crown, float *wp,
                               float *cp_r, float *cp_c)
{
    float cr  = (float)CRANK_R;
    float rod = (float)CONROD_L;

    float cp_dy    = -cr * cosf(theta);
    float cp_dx    =  cr * sinf(theta);
    float rod_vert = sqrtf(rod * rod - cp_dx * cp_dx);
    float wp_row   = (cc_row + cp_dy) - rod_vert;

    *cp_r  = cc_row + cp_dy;
    *cp_c  = cc_col + cp_dx;
    *wp    = wp_row;
    *crown = wp_row - (float)(PISTON_H - 1);
}

/*
 * Determine the current cycle phase.
 * ex_open / tr_open are derived from the piston position (see scene_draw).
 * spark is true when theta is within IGNITE_WINDOW of TDC.
 */
static Phase compute_phase(float theta, bool ex_open, bool tr_open)
{
    if (tr_open) return PHASE_SCAVENGE;
    if (ex_open) return PHASE_EXHAUST;
    /* theta is already in [0, 2π) coming from engine_tick */
    if (theta < IGNITE_WINDOW ||
        theta > 2.0f * (float)M_PI - IGNITE_WINDOW) return PHASE_IGNITE;
    if (theta < (float)M_PI) return PHASE_POWER;
    return PHASE_COMPRESS;
}

static void engine_tick(Engine *e, float dt)
{
    if (e->paused) return;
    float omega = 2.0f * (float)M_PI * (float)e->rpm / 60.0f;
    e->theta += omega * dt;
    if (e->theta >= 2.0f * (float)M_PI)
        e->theta -= 2.0f * (float)M_PI;
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static void safeaddch(WINDOW *w, int r, int c, chtype ch)
{
    int rows, cols;
    getmaxyx(w, rows, cols);
    if (r >= 0 && r < rows && c >= 0 && c < cols)
        mvwaddch(w, r, c, ch);
}

static void safeaddstr(WINDOW *w, int r, int c, const char *s)
{
    int rows, cols;
    getmaxyx(w, rows, cols);
    if (r < 0 || r >= rows) return;
    for (int i = 0; s[i] && c + i < cols; i++)
        if (c + i >= 0)
            mvwaddch(w, r, c + i, (unsigned char)s[i]);
}

/* Bresenham line — draws character ch along the line (r0,c0)→(r1,c1). */
static void draw_line_ch(WINDOW *w, int r0, int c0, int r1, int c1, chtype ch)
{
    int dr = abs(r1 - r0), dc = abs(c1 - c0);
    int sr = (r0 < r1) ? 1 : -1, sc = (c0 < c1) ? 1 : -1;
    int err = dr - dc;
    int rows, cols;
    getmaxyx(w, rows, cols);
    for (int i = 0; i < 400; i++) {
        if (r0 >= 0 && r0 < rows && c0 >= 0 && c0 < cols)
            mvwaddch(w, r0, c0, ch);
        if (r0 == r1 && c0 == c1) break;
        int e2 = 2 * err;
        if (e2 > -dc) { err -= dc; r0 += sr; }
        if (e2 <  dr) { err += dr; c0 += sc; }
    }
}

/*
 * scene_draw — render one frame of the engine.
 *
 * Drawing order (later layers overwrite earlier ones):
 *   1. Cylinder head and walls
 *   2. Gas effects above piston (combustion / exhaust / fresh charge)
 *   3. Exhaust pipe (left) and transfer duct (right)
 *   4. Piston
 *   5. Connecting rod
 *   6. Crankshaft (ellipse orbit indicator, arm, bearing, pin)
 *   7. Crankcase
 *   8. Phase label and angle readout
 */
static void scene_draw(WINDOW *w, const Engine *e, int engine_top, int center_col)
{
    float cc_row_f = (float)(engine_top + CRANK_CENTER_OFF);
    float cc_col_f = (float)center_col;
    float crown_f, wp_f, cp_r_f, cp_c_f;

    engine_kinematics(e->theta, cc_row_f, cc_col_f,
                      &crown_f, &wp_f, &cp_r_f, &cp_c_f);

    int crown_row = (int)roundf(crown_f);
    int wp_row    = (int)roundf(wp_f);
    int cp_row    = (int)roundf(cp_r_f);
    int cp_col    = (int)roundf(cp_c_f);
    int cc_r      = engine_top + CRANK_CENTER_OFF;

    /* Port open when piston crown has descended below the port row */
    bool ex_open = (crown_row > engine_top + EX_PORT_OFF);
    bool tr_open = (crown_row > engine_top + TR_PORT_OFF);
    Phase phase  = compute_phase(e->theta, ex_open, tr_open);
    bool spark   = (phase == PHASE_IGNITE);

    int li = center_col - CYL_IHW;        /* inner left wall col  */
    int ri = center_col + CYL_IHW;        /* inner right wall col */
    int lo = li - CYL_WALL;               /* outer left wall col  */
    int ro = ri + CYL_WALL;               /* outer right wall col */
    int cyl_top  = engine_top + HEAD_H;   /* first cylinder row   */
    int case_top = engine_top + CASE_TOP_OFF;
    int case_bot = engine_top + CASE_BOT_OFF;
    int case_lw  = center_col - CASE_HW;
    int case_rw  = center_col + CASE_HW;

    /* ── 1. Cylinder head ──────────────────────────────────────── */
    wattron(w, COLOR_PAIR(CP_WALL) | A_BOLD);
    for (int c = lo; c <= ro; c++)
        safeaddch(w, engine_top, c, (c == lo || c == ro) ? '+' : '-');
    wattroff(w, COLOR_PAIR(CP_WALL) | A_BOLD);

    if (spark) {
        wattron(w, COLOR_PAIR(CP_SPARK) | A_BOLD);
        safeaddch(w, engine_top, center_col - 2, '-');
        safeaddch(w, engine_top, center_col - 1, '[');
        safeaddch(w, engine_top, center_col,     '*');
        safeaddch(w, engine_top, center_col + 1, ']');
        safeaddch(w, engine_top, center_col + 2, '-');
        wattroff(w, COLOR_PAIR(CP_SPARK) | A_BOLD);
    } else {
        wattron(w, COLOR_PAIR(CP_WALL));
        safeaddch(w, engine_top, center_col - 1, '[');
        safeaddch(w, engine_top, center_col,     'i');
        safeaddch(w, engine_top, center_col + 1, ']');
        wattroff(w, COLOR_PAIR(CP_WALL));
    }

    /* ── 1b. Cylinder walls ────────────────────────────────────── */
    for (int r = cyl_top; r < case_top; r++) {
        bool at_ex = (r == engine_top + EX_PORT_OFF);
        bool at_tr = (r == engine_top + TR_PORT_OFF);

        wattron(w, COLOR_PAIR(CP_WALL));
        safeaddch(w, r, lo, '|');
        safeaddch(w, r, ro, '|');
        if (!(at_ex && ex_open)) safeaddch(w, r, li, '|');
        if (!(at_tr && tr_open)) safeaddch(w, r, ri, '|');
        wattroff(w, COLOR_PAIR(CP_WALL));
    }

    /* ── 2. Gas above piston ───────────────────────────────────── */
    if (phase == PHASE_IGNITE) {
        wattron(w, COLOR_PAIR(CP_SPARK) | A_BOLD);
        for (int r = cyl_top; r < crown_row; r++)
            for (int c = li + 1; c < ri; c++)
                safeaddch(w, r, c, ((r + c) & 1) ? '*' : '^');
        wattroff(w, COLOR_PAIR(CP_SPARK) | A_BOLD);
    } else if (phase == PHASE_POWER) {
        static const char pch[] = "^~`";
        wattron(w, COLOR_PAIR(CP_FIRE));
        for (int r = cyl_top; r < crown_row; r++)
            for (int c = li + 1; c < ri; c++)
                safeaddch(w, r, c, pch[(r + c) % 3]);
        wattroff(w, COLOR_PAIR(CP_FIRE));
    } else if (phase == PHASE_EXHAUST) {
        wattron(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
        for (int r = cyl_top; r < crown_row; r++)
            for (int c = li + 1; c < ri; c++)
                safeaddch(w, r, c, '~');
        wattroff(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
    } else if (phase == PHASE_SCAVENGE) {
        int mid = (li + ri) / 2;
        for (int r = cyl_top; r < crown_row; r++) {
            wattron(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
            for (int c = li + 1; c <= mid; c++)
                safeaddch(w, r, c, '~');
            wattroff(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
            wattron(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
            for (int c = mid + 1; c < ri; c++)
                safeaddch(w, r, c, '+');
            wattroff(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
        }
    }
    /* COMPRESS: leave space empty (invisible compressed charge) */

    /* ── 3a. Exhaust pipe (left side) ─────────────────────────── */
    if (ex_open) {
        int er = engine_top + EX_PORT_OFF;
        wattron(w, COLOR_PAIR(CP_EXHAUST));
        safeaddch(w, er - 1, lo - 1, '/');
        safeaddch(w, er + 1, lo - 1, '\\');
        for (int c = lo - 5; c < lo; c++)
            safeaddch(w, er, c, '~');
        wattroff(w, COLOR_PAIR(CP_EXHAUST));
        wattron(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
        for (int c = lo - 9; c < lo - 5; c++)
            safeaddch(w, er, c, '.');
        wattroff(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
    }

    /* ── 3b. Transfer duct (right side) ───────────────────────── */
    if (tr_open) {
        int tr = engine_top + TR_PORT_OFF;
        wattron(w, COLOR_PAIR(CP_INTAKE));
        safeaddch(w, tr - 1, ro + 1, '\\');
        safeaddch(w, tr + 1, ro + 1, '/');
        for (int c = ro + 1; c <= ro + 5; c++)
            safeaddch(w, tr, c, '>');
        wattroff(w, COLOR_PAIR(CP_INTAKE));
        wattron(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
        for (int c = ro + 6; c <= ro + 9; c++)
            safeaddch(w, tr, c, '+');
        wattroff(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
    }

    /* ── 4. Piston ─────────────────────────────────────────────── */
    wattron(w, COLOR_PAIR(CP_PISTON) | A_BOLD);
    for (int r = crown_row; r < crown_row + PISTON_H; r++) {
        safeaddch(w, r, li, '[');
        for (int c = li + 1; c < ri; c++)
            safeaddch(w, r, c, (r == crown_row) ? '=' : '#');
        safeaddch(w, r, ri, ']');
    }
    wattroff(w, COLOR_PAIR(CP_PISTON) | A_BOLD);

    /* ── 5. Connecting rod ─────────────────────────────────────── */
    wattron(w, COLOR_PAIR(CP_CONROD));
    draw_line_ch(w, wp_row, center_col, cp_row, cp_col, ':');
    safeaddch(w, wp_row, center_col, 'o');   /* wrist pin */
    wattroff(w, COLOR_PAIR(CP_CONROD));

    /* ── 6. Crankshaft ─────────────────────────────────────────── */
    /* Ellipse orbit indicator (aspect-corrected so it looks circular) */
    wattron(w, COLOR_PAIR(CP_CRANK) | A_DIM);
    for (int deg = 0; deg < 360; deg += 12) {
        float a  = (float)deg * (float)M_PI / 180.0f;
        int   er = cc_r + (int)roundf((float)CRANK_R * 0.5f * sinf(a));
        int   ec = center_col + (int)roundf((float)CRANK_R * cosf(a));
        safeaddch(w, er, ec, '.');
    }
    wattroff(w, COLOR_PAIR(CP_CRANK) | A_DIM);
    /* Crank arm */
    wattron(w, COLOR_PAIR(CP_CRANK));
    draw_line_ch(w, cc_r, center_col, cp_row, cp_col, '-');
    safeaddch(w, cc_r, center_col, 'O');     /* main bearing        */
    wattron(w, A_BOLD);
    safeaddch(w, cp_row, cp_col, 'o');       /* crank pin           */
    wattroff(w, A_BOLD);
    wattroff(w, COLOR_PAIR(CP_CRANK));

    /* ── 7. Crankcase ──────────────────────────────────────────── */
    wattron(w, COLOR_PAIR(CP_WALL));
    /* Top bar — gap where cylinder bore passes through */
    for (int c = case_lw; c <= case_rw; c++) {
        chtype ch;
        if (c == case_lw || c == case_rw) ch = '+';
        else if (c >= lo && c <= ro)      ch = ' ';   /* bore opening */
        else                               ch = '-';
        safeaddch(w, case_top, c, ch);
    }
    /* Side walls */
    for (int r = case_top + 1; r < case_bot; r++) {
        safeaddch(w, r, case_lw, '|');
        safeaddch(w, r, case_rw, '|');
    }
    /* Bottom */
    for (int c = case_lw; c <= case_rw; c++)
        safeaddch(w, case_bot, c, (c == case_lw || c == case_rw) ? '+' : '-');
    /* Power-takeoff stub on right */
    safeaddch(w, cc_r, case_rw,     '>');
    safeaddch(w, cc_r, case_rw + 1, '=');
    safeaddch(w, cc_r, case_rw + 2, '>');
    wattroff(w, COLOR_PAIR(CP_WALL));

    /* ── 8. Phase label and crank angle (right of cylinder) ────── */
    wattron(w, COLOR_PAIR(CP_PHASE) | A_BOLD);
    safeaddstr(w, engine_top + 3, ro + 3, phase_name(phase));
    wattroff(w, COLOR_PAIR(CP_PHASE) | A_BOLD);

    wattron(w, COLOR_PAIR(CP_HUD));
    {
        char buf[20];
        int deg = (int)(e->theta * 180.0f / (float)M_PI) % 360;
        snprintf(buf, sizeof(buf), "%3d deg", deg);
        safeaddstr(w, engine_top + 5, ro + 3, buf);
    }

    /* Port state indicators */
    safeaddstr(w, engine_top + EX_PORT_OFF, lo - 3,
               ex_open ? "EX" : "  ");
    safeaddstr(w, engine_top + TR_PORT_OFF, ro + 3,
               tr_open ? "TR" : "  ");
    wattroff(w, COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    Engine engine;
    int    sim_fps;
} Scene;

static void scene_init(Scene *s)
{
    engine_reset(&s->engine);
    s->sim_fps = SIM_FPS_DEFAULT;
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit   = 0;

static void on_sigwinch(int s) { (void)s; g_resize = 1; }
static void on_sigterm(int s)  { (void)s; g_quit   = 1; }

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    color_init();
}

static void screen_cleanup(void)
{
    curs_set(1);
    endwin();
}

static void draw_hud(WINDOW *w, const Scene *s, float fps, int engine_top)
{
    int rows, cols;
    getmaxyx(w, rows, cols);

    int r = engine_top + ENGINE_H;
    if (r >= rows) r = rows - 1;
    if (r < 0) return;

    wattron(w, COLOR_PAIR(CP_HUD));
    char buf[HUD_COLS + 4];
    snprintf(buf, sizeof(buf),
             "%5.1f fps   RPM: %3d   ] faster   [ slower",
             (double)fps, s->engine.rpm);
    int c = (cols - (int)strlen(buf)) / 2;
    if (c < 0) c = 0;
    safeaddstr(w, r, c, buf);
    wattroff(w, COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

int main(void)
{
    signal(SIGWINCH, on_sigwinch);
    signal(SIGTERM,  on_sigterm);
    signal(SIGINT,   on_sigterm);

    screen_init();

    Scene   scene;
    scene_init(&scene);

    int64_t last_time  = clock_ns();
    int64_t sim_accum  = 0;
    int64_t fps_accum  = 0;
    int     fps_frames = 0;
    float   fps_disp   = 0.0f;
    int64_t frame_ns   = TICK_NS(60);   /* render cap ~60 fps */

    int scr_rows, scr_cols;
    getmaxyx(stdscr, scr_rows, scr_cols);

    for (;;) {
        /* ── resize ── */
        if (g_resize) {
            g_resize = 0;
            endwin();
            refresh();
            getmaxyx(stdscr, scr_rows, scr_cols);
        }
        if (g_quit) break;

        /* ── dt ── */
        int64_t now   = clock_ns();
        int64_t dt_ns = now - last_time;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;
        last_time = now;
        sim_accum += dt_ns;
        fps_accum += dt_ns;
        fps_frames++;

        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_disp   = (float)fps_frames * 1e9f / (float)fps_accum;
            fps_accum  = 0;
            fps_frames = 0;
        }

        /* ── sim ticks ── */
        int64_t tick = TICK_NS(scene.sim_fps);
        while (sim_accum >= tick) {
            float dt = (float)tick / (float)NS_PER_SEC;
            engine_tick(&scene.engine, dt);
            sim_accum -= tick;
        }

        /* ── layout ── */
        int engine_top = (scr_rows - ENGINE_H - 2) / 2;
        if (engine_top < 1) engine_top = 1;
        int center_col = scr_cols / 2;

        /* ── draw ── */
        erase();
        scene_draw(stdscr, &scene.engine, engine_top, center_col);
        draw_hud(stdscr, &scene, fps_disp, engine_top);
        wnoutrefresh(stdscr);
        doupdate();

        /* ── input ── */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q':
            case 27:          g_quit = 1; break;
            case ' ':
            case 'p':         scene.engine.paused = !scene.engine.paused; break;
            case 'r':         engine_reset(&scene.engine); break;
            case ']':
                if (scene.engine.rpm < RPM_MAX) scene.engine.rpm += RPM_STEP;
                break;
            case '[':
                if (scene.engine.rpm > RPM_MIN) scene.engine.rpm -= RPM_STEP;
                break;
            case KEY_RESIZE:  g_resize = 1; break;
            }
        }

        /* ── frame cap ── */
        int64_t elapsed = clock_ns() - now;
        clock_sleep_ns(frame_ns - elapsed);
    }

    screen_cleanup();
    return 0;
}
