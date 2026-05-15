/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * stroke_engine.c  —  Internal combustion engine gallery: 2/4/6-stroke
 *
 * Cross-section animation of three reciprocating engine cycles in a
 * single program. Switch between them at runtime with n / p.
 *
 *   2-stroke  Schnurle scavenging — piston covers/uncovers wall ports.
 *             1 power stroke per crank revolution.
 *   4-stroke  Otto cycle — poppet valves in the head, camshaft at half
 *             crank speed. 1 power stroke per 2 crank revolutions.
 *   6-stroke  Crower water-injection — Otto plus water sprayed onto
 *             the hot piston after exhaust; it flashes to steam and
 *             drives a second power stroke from waste heat.
 *             1 fuel + 1 steam stroke per 3 crank revolutions.
 *
 * Slider-crank kinematics drive the piston identically in all three;
 * what differs is the cycle state machine and the gas-exchange geometry
 * (wall ports vs poppet valves vs valves + injector).
 *
 * Cycle table (theta_c = cycle angle, advances continuously):
 *
 *   2-stroke (cycle_total = 360 deg = 1 crank revolution):
 *     theta_c =   0   TDC, spark, IGNITION
 *     theta_c ~  75   exhaust port uncovers       -> EXHAUST
 *     theta_c ~  90   transfer port uncovers      -> SCAVENGING
 *     theta_c = 180   BDC
 *     theta_c ~ 285   exhaust port re-covered     -> COMPRESSION
 *     theta_c = 360   TDC, repeat
 *
 *   4-stroke (cycle_total = 720 deg = 2 crank revolutions):
 *     [  0, 180)  INTAKE        — intake valve open
 *     [180, 360)  COMPRESSION
 *     theta_c = 360   IGNITION (spark fires)
 *     [360, 540)  POWER
 *     [540, 720)  EXHAUST       — exhaust valve open
 *
 *   6-stroke Crower (cycle_total = 1080 deg = 3 crank revolutions):
 *     [  0, 180)  INTAKE        — intake valve open (air-fuel)
 *     [180, 360)  COMPRESSION
 *     theta_c = 360   IGNITION
 *     [360, 540)  POWER         — combustion expansion
 *     [540, 720)  EXHAUST       — exhaust valve open
 *     theta_c = 720   WATER INJECTION (water sprays onto hot piston)
 *     [720, 900)  STEAM POWER   — water flashes to steam, expands
 *     [900,1080)  STEAM EXHAUST — exhaust valve open
 *
 * Engine geometry (cell units, y increases downward):
 *   Crank radius    CRANK_R  = 4   (stroke = 8 cells)
 *   Connecting rod  CONROD_L = 9
 *   Bore half-width CYL_IHW  = 6
 *   Piston height   PISTON_H = 3
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   r         reset cycle to start
 *   ] [       RPM up / down
 *   n / p     next / previous engine (2S -> 4S -> 6S -> 2S)
 *   t / T     next / previous theme
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/stroke_engine.c -o stroke_engine -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  themes & color
 *   §4  kinematics  (slider-crank, type-agnostic)
 *   §5  engine      (type, cycle state machine, tick)
 *   §6  draw        (walls / head / piston / conrod / crank / case + adornments)
 *   §7  scene
 *   §8  screen      (ncurses init, HUD)
 *   §9  app         (main loop, input)
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Engineering    : Slider-crank mechanism, identical for every engine.
 *                  Given crank radius R, rod length L, crank centre row
 *                  y_cc, crank angle theta (0 = TDC, +ve = CW):
 *                    y_wrist = y_cc - R*cos(theta) - sqrt(L^2 - R^2*sin^2(theta))
 *                  At TDC (theta = 0) the wrist pin sits R+L cells above
 *                  y_cc; at BDC it sits L-R cells above. The cycle
 *                  differs only in the surrounding state machine and
 *                  the head geometry.
 *
 * Thermodynamics : Three cycles, increasing complexity.
 *                  2-stroke : Schnurle scavenging — no valves; ports in
 *                             the cylinder wall are uncovered by the
 *                             descending piston. Intake and exhaust
 *                             overlap near BDC.
 *                  4-stroke : Otto — discrete intake / compression /
 *                             power / exhaust strokes. Poppet valves
 *                             in the head, camshaft at half crank speed
 *                             (one open-close per 2 revolutions).
 *                  6-stroke : Crower water injection. 4-stroke plus two
 *                             extra strokes. Water injected onto the hot
 *                             piston after exhaust flashes to steam;
 *                             the steam expansion drives a second power
 *                             stroke from otherwise-wasted heat.
 *
 * State machine  : cycle_angle in [0, cycle_total).
 *                  cycle_total = strokes * pi    (1 rev per 2 strokes)
 *                  Phase is chosen by which segment of cycle_angle we
 *                  are in. Crank angle (for piston kinematics) is
 *                  cycle_angle mod 2*pi. For 2-stroke, port states also
 *                  depend on the piston row (uncovered when the crown
 *                  descends past the port).
 *
 * Rendering      : ASCII cross-section. Common skeleton (walls, piston,
 *                  conrod, crank, case) plus per-type adornments:
 *                    2S — wall ports + manifold stubs
 *                    4S — poppet valves drawn in the head row
 *                    6S — valves + water injector marker above the head
 *                  Gas above piston rendered with a per-phase glyph and
 *                  colour:
 *                    INTAKE '+' (cyan, dim), COMPRESS empty,
 *                    POWER '^~`' (red), EXHAUST '~' (grey, dim),
 *                    SCAVENGE half-and-half, WATER_INJ '.,' (cyan),
 *                    STEAM_POWER '%"' (white), STEAM_EXHAUST '"' (cyan).
 *
 * References     : 1. Heywood, J. B. — "Internal Combustion Engine
 *                     Fundamentals", 2nd ed., McGraw-Hill (2018). The
 *                     standard textbook on engine cycles: covers Otto
 *                     (4-stroke), 2-stroke scavenging, and the valve /
 *                     port timing this file's state machines model.
 *                  2. Reuleaux, F. — "The Kinematics of Machinery",
 *                     Macmillan (1876). The historic foundation for
 *                     drawing mechanisms in cross-section. Reuleaux's
 *                     plates of the slider-crank are the template every
 *                     engine cutaway since has followed; §6's head /
 *                     piston / conrod / crank layout inherits from his
 *                     conventions.
 *                  3. Foley, J. D.; van Dam, A.; Feiner, S. K.; Hughes,
 *                     J. F. — "Computer Graphics: Principles and
 *                     Practice", 3rd ed., Addison-Wesley (2013). Ch. 2
 *                     covers Bresenham's line algorithm, used by
 *                     draw_line_ch() to rasterise the connecting rod
 *                     and crank arm onto the character grid.
 * ─────────────────────────────────────────────────────────────────────── */

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

    FPS_UPDATE_MS   = 500,
};

/*
 * Engine geometry — shared by every engine type.
 *
 * Slider-crank at TDC (theta=0):
 *   crank_pin_row = crank_centre_row - CRANK_R
 *   wrist_pin_row = crank_pin_row    - CONROD_L
 *   crown_row     = wrist_pin_row    - (PISTON_H - 1)
 *
 * Setting crown@TDC = engine_top + HEAD_H:
 *   CRANK_CENTER_OFF = HEAD_H + (PISTON_H-1) + CONROD_L + CRANK_R = 16
 *
 * Stroke = 2 * CRANK_R = 8 cells.
 * Crown ranges engine_top+1 (TDC) ... engine_top+9 (BDC).
 */
#define CYL_IHW          6   /* cylinder bore inner half-width            */
#define CYL_WALL         1   /* cylinder wall thickness                   */
#define HEAD_H           1   /* cylinder head height (rows)               */
#define PISTON_H         3   /* piston height (rows)                      */
#define CRANK_R          4   /* crank throw radius (cells)                */
#define CONROD_L         9   /* connecting rod length (cells)             */
#define CRANK_CENTER_OFF (HEAD_H + (PISTON_H-1) + CONROD_L + CRANK_R)

/* 2-stroke wall ports — row offset from engine_top.
 * A port is open when the piston crown has descended below it. */
#define EX_PORT_OFF      6
#define TR_PORT_OFF      7

/* Crankcase */
#define CASE_TOP_OFF    12
#define CASE_BOT_OFF    21
#define CASE_HW          9
#define ENGINE_H        (CASE_BOT_OFF + 2)   /* total = 23 rows */

/* 4/6-stroke valve column offsets from cylinder centre. */
#define VALVE_OFF        3

/* IGNITE_WINDOW: half-angle (rad) around the spark crank position where
 * the spark glyph shows and the phase is labelled IGNITION. */
#define IGNITE_WINDOW    0.30f

/* WATER_WINDOW: half-angle (rad) around the water-injector trigger.
 * Water injection is brief but visually distinct — give it some room. */
#define WATER_WINDOW     0.45f

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
/* §3  themes & color                                                     */
/* ===================================================================== */

typedef enum {
    CP_WALL    = 1,   /* cylinder walls, head, crankcase   */
    CP_PISTON  = 2,   /* piston body                       */
    CP_CONROD  = 3,   /* connecting rod                    */
    CP_CRANK   = 4,   /* crankshaft                        */
    CP_FIRE    = 5,   /* combustion / hot gas              */
    CP_EXHAUST = 6,   /* exhaust gas, exhaust valve        */
    CP_INTAKE  = 7,   /* fresh charge, intake valve, water */
    CP_SPARK   = 8,   /* spark flash, steam                */
    CP_HUD     = 9,   /* top HUD (bright yellow)           */
    CP_HINT    = 10,  /* bottom hint (bright cyan)         */
    CP_PHASE   = 11,  /* phase name                        */
} ColorPair;

/*
 * Theme palette — 8 indexed colours per theme, one per mechanical /
 * effect pair (CP_WALL ... CP_SPARK).  All values sit in the bright
 * half of the 256-colour space so nothing turns invisible against the
 * default-black background (CLAUDE.md "Theme Palette Brightness").
 * HUD, HINT, and PHASE pairs are theme-independent for legibility.
 */
typedef struct {
    const char *name;
    short wall, piston, conrod, crank;
    short fire, exhaust, intake, spark;
} Theme;

static const Theme THEMES[] = {
    /* name       wall pist  rod crank  fire exha  int  spark */
    { "MATRIX",   244,  46,  40,  82,   46,  28, 118, 231 },
    { "FIRE",     244, 208, 214, 220,  196, 240, 226, 231 },
    { "OCEANIC",  251,  68, 110,  87,   45, 244, 159, 231 },
    { "NEON",      51, 201, 219,  82,  199, 141,  51, 231 },
    { "MONO",     251, 248, 246, 255,  244, 242, 250, 255 },
    { "ICE",      255, 153, 159,  51,  159, 251,  87, 231 },
    { "NOVA",     255, 177, 213, 231,  201, 141, 219, 231 },
    { "FOREST",   144,  70, 108, 178,  130, 240, 148, 231 },
    { "DESERT",   180, 215, 222, 178,  166, 180, 228, 231 },
    { "ECLIPSE",  250, 240, 244, 208,  160, 240, 214, 231 },
};
#define N_THEMES ((int)(sizeof(THEMES) / sizeof(THEMES[0])))

static void color_apply_theme(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &THEMES[idx];
        init_pair(CP_WALL,    t->wall,    COLOR_BLACK);
        init_pair(CP_PISTON,  t->piston,  COLOR_BLACK);
        init_pair(CP_CONROD,  t->conrod,  COLOR_BLACK);
        init_pair(CP_CRANK,   t->crank,   COLOR_BLACK);
        init_pair(CP_FIRE,    t->fire,    COLOR_BLACK);
        init_pair(CP_EXHAUST, t->exhaust, COLOR_BLACK);
        init_pair(CP_INTAKE,  t->intake,  COLOR_BLACK);
        init_pair(CP_SPARK,   t->spark,   COLOR_BLACK);
    } else {
        init_pair(CP_WALL,    COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_PISTON,  COLOR_CYAN,   COLOR_BLACK);
        init_pair(CP_CONROD,  COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_CRANK,   COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_FIRE,    COLOR_RED,    COLOR_BLACK);
        init_pair(CP_EXHAUST, COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_INTAKE,  COLOR_CYAN,   COLOR_BLACK);
        init_pair(CP_SPARK,   COLOR_WHITE,  COLOR_BLACK);
    }
}

static void color_init(int theme_idx)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(CP_HUD,   226, COLOR_BLACK);
        init_pair(CP_HINT,   51, COLOR_BLACK);
        init_pair(CP_PHASE, 214, COLOR_BLACK);
    } else {
        init_pair(CP_HUD,   COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_HINT,  COLOR_CYAN,   COLOR_BLACK);
        init_pair(CP_PHASE, COLOR_YELLOW, COLOR_BLACK);
    }
    color_apply_theme(theme_idx);
}

/* ===================================================================== */
/* §4  kinematics — slider-crank                                          */
/* ===================================================================== */

/*
 * Kinematics — slider-crank snapshot in cell coordinates.
 *
 * Why this exists:
 *   solve_slider_crank() returns four points that all derive from the
 *   same theta. Bundling them into one struct keeps drawing consistent:
 *   if any two drifted out of sync (separate return slots, separate
 *   rounds), the conrod would stop touching the wrist pin and the
 *   piston would appear to detach from the rod.
 *
 * What the four points represent (cells, y increases downward):
 *
 *      crown_row    ┌───── top of piston body (combustion surface)
 *                   │ piston: PISTON_H rows tall
 *      wrist_row    └─── wrist pin: bottom of piston / small end of rod
 *                   │
 *                   │  conrod (length L, drawn by Bresenham)
 *                   │
 *      crank_pin_r  ●── crank pin: big end of rod / orbit endpoint
 *      crank_pin_c     orbits the crank centre at radius CRANK_R
 *
 * Geometric relations:
 *   wrist_row - crown_row = PISTON_H - 1
 *   |crank_pin - crank_centre| = CRANK_R         (always — that's the orbit)
 *   |crank_pin - wrist_pin|    = CONROD_L        (always — rigid rod)
 *
 * Algorithm:
 *   Closed-form slider-crank (no iteration). One end of the rod orbits
 *   the crank centre at radius R; the other end is constrained to the
 *   cylinder axis. The triangle has all three sides determined by
 *   theta alone:
 *     y_wrist = y_cc - R·cos(theta) - sqrt(L² - R²·sin²(theta))
 *   See Norton "Design of Machinery" §4.5 for the derivation; the same
 *   construction is in Reuleaux's plates from 1876 (header ref [2]).
 *
 *   Values are rounded to int once here so every downstream draw call
 *   targets the same cells — no half-cell jitter between piston body
 *   and the rod that's supposed to be touching it.
 */
typedef struct {
    int crown_row;     /* top of piston body                       */
    int wrist_row;     /* wrist pin: piston-to-rod joint           */
    int crank_pin_r;   /* crank pin (big end), row component       */
    int crank_pin_c;   /* crank pin (big end), col component       */
} Kinematics;

/*
 * Slider-crank: given crank angle theta (0 = TDC, +ve = CW) and the
 * crank centre position (cell coords), return the four cell-rounded
 * reference points the renderer needs. Pure function — identical for
 * every engine type.
 */
static Kinematics solve_slider_crank(float theta, float cc_row, float cc_col)
{
    float cr     = (float)CRANK_R;
    float rod    = (float)CONROD_L;
    float cp_dy  = -cr * cosf(theta);
    float cp_dx  =  cr * sinf(theta);
    float rod_v  = sqrtf(rod * rod - cp_dx * cp_dx);
    float wp_row = (cc_row + cp_dy) - rod_v;

    Kinematics k;
    k.crank_pin_r = (int)roundf(cc_row + cp_dy);
    k.crank_pin_c = (int)roundf(cc_col + cp_dx);
    k.wrist_row   = (int)roundf(wp_row);
    k.crown_row   = (int)roundf(wp_row - (float)(PISTON_H - 1));
    return k;
}

/* ===================================================================== */
/* §5  engine — type, cycle state machine, tick                          */
/* ===================================================================== */

typedef enum {
    ENG_2STROKE = 0,
    ENG_4STROKE = 1,
    ENG_6STROKE = 2,
} EngineType;
#define N_ENGINES 3

static const char *engine_name(EngineType t)
{
    switch (t) {
    case ENG_2STROKE: return "2-STROKE";
    case ENG_4STROKE: return "4-STROKE";
    case ENG_6STROKE: return "6-STROKE";
    default:          return "?";
    }
}

static int engine_strokes(EngineType t)
{
    return (t == ENG_2STROKE) ? 2 : (t == ENG_4STROKE) ? 4 : 6;
}

/* cycle_total = strokes * pi   (one revolution per two strokes). */
static float engine_cycle_total(EngineType t)
{
    return (float)engine_strokes(t) * (float)M_PI;
}

/* One-line cycle name shown beside the HUD so users see at a glance
 * which thermodynamic cycle is on screen and how many crank revolutions
 * one full cycle takes. */
static const char *engine_subtitle(EngineType t)
{
    switch (t) {
    case ENG_2STROKE: return "Schnurle scavenging (1 rev/cycle)";
    case ENG_4STROKE: return "Otto cycle (2 rev/cycle)";
    case ENG_6STROKE: return "Crower water-injection (3 rev/cycle)";
    default:          return "";
    }
}

typedef enum {
    PHASE_INTAKE,
    PHASE_COMPRESS,
    PHASE_IGNITE,
    PHASE_POWER,
    PHASE_EXHAUST,
    PHASE_SCAVENGE,
    PHASE_WATER_INJ,
    PHASE_STEAM_POWER,
    PHASE_STEAM_EXHAUST,
} Phase;

static const char *phase_name(Phase p)
{
    switch (p) {
    case PHASE_INTAKE:        return "INTAKE       ";
    case PHASE_COMPRESS:      return "COMPRESSION  ";
    case PHASE_IGNITE:        return "IGNITION     ";
    case PHASE_POWER:         return "POWER        ";
    case PHASE_EXHAUST:       return "EXHAUST      ";
    case PHASE_SCAVENGE:      return "SCAVENGING   ";
    case PHASE_WATER_INJ:     return "WATER INJECT ";
    case PHASE_STEAM_POWER:   return "STEAM POWER  ";
    case PHASE_STEAM_EXHAUST: return "STEAM EXHAUST";
    default:                  return "             ";
    }
}

/*
 * Engine — combined simulation state for one cylinder.
 *
 * Two kinds of state live here deliberately together:
 *
 *   (a) Persistent state — advanced by engine_tick() each sim step,
 *       mutated by user input. Survives across frames.
 *
 *   (b) Derived state — pure function of (a). Recomputed once per
 *       frame by engine_derive_state() at the top of scene_draw().
 *       Every drawing helper reads these flags rather than re-running
 *       the threshold comparisons — the gas-above-piston, head-valve,
 *       and port-stub routines all need the same answers, so we
 *       compute them once.
 *
 * Why cache the derived flags instead of recomputing per draw call:
 *   the 4-stroke and 6-stroke state machines have non-trivial
 *   branching (sextant lookup, IGNITE window check around 2·pi).
 *   Doing that inside every draw_* function would duplicate logic
 *   across five callsites and risk inconsistent answers if the
 *   thresholds were ever tuned in one place but not another.
 *
 * Algorithm refs:
 *   Phase boundaries (Otto)            — Heywood §6   (header ref [1])
 *   2-stroke port timing by geometry   — Heywood §7
 *   Slider-crank link (cycle -> piston row)
 *                                       — Norton §4.5 (header ref [2])
 *
 * Invariants (maintained by engine_tick and engine_reset):
 *   0 <= cycle_angle < engine_cycle_total(type)
 *   RPM_MIN <= rpm <= RPM_MAX
 *   Derived flags reflect cycle_angle as of the most recent
 *   engine_derive_state() call (i.e. current frame).
 */
typedef struct {
    /* ── (a) Persistent state ──────────────────────────────────── */
    EngineType type;            /* which thermodynamic cycle to run            */
    float      cycle_angle;     /* rad, [0, strokes·pi); wraps at cycle_total  */
    int        rpm;             /* crank speed setpoint; drives omega in tick  */
    bool       paused;          /* if true, engine_tick() is a no-op           */

    /* ── (b) Per-frame derived state ───────────────────────────── *
     * Cache. Filled by engine_derive_state() — do not write from
     * anywhere else. Reading these from draw helpers is fine; they
     * are valid for the duration of one scene_draw() call. */
    bool   intake_valve_open;   /* 4/6-stroke: cycle_angle in [0, pi)          */
    bool   exhaust_valve_open;  /* 4/6-stroke: combustion or steam exhaust     */
    bool   water_inj_open;      /* 6-stroke: WATER_WINDOW around cycle = 4·pi  */
    bool   ex_port_open;        /* 2-stroke: crown row > engine_top+EX_PORT_OFF*/
    bool   tr_port_open;        /* 2-stroke: crown row > engine_top+TR_PORT_OFF*/
    Phase  phase;               /* state-machine label; drives gas chemistry   */
} Engine;

static float wrap_to_2pi(float a)
{
    float two_pi = 2.0f * (float)M_PI;
    while (a < 0.0f)     a += two_pi;
    while (a >= two_pi)  a -= two_pi;
    return a;
}

static float engine_crank_angle(const Engine *e)
{
    return wrap_to_2pi(e->cycle_angle);
}

static void engine_reset(Engine *e, EngineType type)
{
    e->type        = type;
    e->cycle_angle = (float)M_PI;   /* start near BDC of first stroke */
    e->rpm         = RPM_DEFAULT;
    e->paused      = false;
    e->intake_valve_open  = false;
    e->exhaust_valve_open = false;
    e->water_inj_open     = false;
    e->ex_port_open       = false;
    e->tr_port_open       = false;
    e->phase              = PHASE_COMPRESS;
}

/* Switch engine type but preserve RPM and pause state. */
static void engine_set_type(Engine *e, EngineType type)
{
    e->type        = type;
    e->cycle_angle = (float)M_PI;
}

static void engine_tick(Engine *e, float dt)
{
    if (e->paused) return;
    float omega = 2.0f * (float)M_PI * (float)e->rpm / 60.0f;
    e->cycle_angle += omega * dt;
    float total = engine_cycle_total(e->type);
    if (e->cycle_angle >= total) e->cycle_angle -= total;
    if (e->cycle_angle <  0.0f)  e->cycle_angle += total;
}

/*
 * Derive valve/port/phase state for the current cycle_angle.
 * For 2-stroke, port states also depend on the piston crown row, since
 * wall ports are uncovered by geometry, not by a camshaft.
 */
static void engine_derive_state(Engine *e, int crown_row, int engine_top)
{
    const float TWO_PI   = 2.0f * (float)M_PI;
    const float THREE_PI = 3.0f * (float)M_PI;
    const float FOUR_PI  = 4.0f * (float)M_PI;
    const float FIVE_PI  = 5.0f * (float)M_PI;
    float c = e->cycle_angle;

    e->intake_valve_open  = false;
    e->exhaust_valve_open = false;
    e->water_inj_open     = false;
    e->ex_port_open       = false;
    e->tr_port_open       = false;

    switch (e->type) {
    case ENG_2STROKE: {
        bool exo = (crown_row > engine_top + EX_PORT_OFF);
        bool tro = (crown_row > engine_top + TR_PORT_OFF);
        e->ex_port_open = exo;
        e->tr_port_open = tro;
        if      (tro)                                          e->phase = PHASE_SCAVENGE;
        else if (exo)                                          e->phase = PHASE_EXHAUST;
        else if (c < IGNITE_WINDOW || c > TWO_PI - IGNITE_WINDOW)
                                                               e->phase = PHASE_IGNITE;
        else if (c < (float)M_PI)                              e->phase = PHASE_POWER;
        else                                                   e->phase = PHASE_COMPRESS;
        break;
    }
    case ENG_4STROKE: {
        if (c < (float)M_PI) {
            e->intake_valve_open = true;
            e->phase = PHASE_INTAKE;
        } else if (c < TWO_PI) {
            e->phase = (c > TWO_PI - IGNITE_WINDOW) ? PHASE_IGNITE : PHASE_COMPRESS;
        } else if (c < TWO_PI + IGNITE_WINDOW) {
            e->phase = PHASE_IGNITE;
        } else if (c < THREE_PI) {
            e->phase = PHASE_POWER;
        } else {
            e->exhaust_valve_open = true;
            e->phase = PHASE_EXHAUST;
        }
        break;
    }
    case ENG_6STROKE: {
        if (c < (float)M_PI) {
            e->intake_valve_open = true;
            e->phase = PHASE_INTAKE;
        } else if (c < TWO_PI) {
            e->phase = (c > TWO_PI - IGNITE_WINDOW) ? PHASE_IGNITE : PHASE_COMPRESS;
        } else if (c < TWO_PI + IGNITE_WINDOW) {
            e->phase = PHASE_IGNITE;
        } else if (c < THREE_PI) {
            e->phase = PHASE_POWER;
        } else if (c < FOUR_PI) {
            e->exhaust_valve_open = true;
            e->phase = PHASE_EXHAUST;
        } else if (c < FOUR_PI + WATER_WINDOW) {
            e->water_inj_open = true;
            e->phase = PHASE_WATER_INJ;
        } else if (c < FIVE_PI) {
            e->phase = PHASE_STEAM_POWER;
        } else {
            e->exhaust_valve_open = true;
            e->phase = PHASE_STEAM_EXHAUST;
        }
        break;
    }
    }
}

/* ===================================================================== */
/* §6  draw                                                               */
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

/* Bresenham line — draws character ch along the line (r0,c0)->(r1,c1). */
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

/* ── 6.1  walls ───────────────────────────────────────────────────── */
/* 2-stroke walls have ports cut into the inner wall at fixed rows. */
static void draw_walls_2stroke(WINDOW *w, int engine_top, int center_col,
                               const Engine *e)
{
    int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
    int lo = li - CYL_WALL,        ro = ri + CYL_WALL;
    int cyl_top  = engine_top + HEAD_H;
    int case_top = engine_top + CASE_TOP_OFF;

    wattron(w, COLOR_PAIR(CP_WALL));
    for (int r = cyl_top; r < case_top; r++) {
        bool at_ex = (r == engine_top + EX_PORT_OFF);
        bool at_tr = (r == engine_top + TR_PORT_OFF);
        safeaddch(w, r, lo, '|');
        safeaddch(w, r, ro, '|');
        if (!(at_ex && e->ex_port_open)) safeaddch(w, r, li, '|');
        if (!(at_tr && e->tr_port_open)) safeaddch(w, r, ri, '|');
    }
    wattroff(w, COLOR_PAIR(CP_WALL));
}

/* 4/6-stroke walls are solid — gas exchange happens through head valves. */
static void draw_walls_solid(WINDOW *w, int engine_top, int center_col)
{
    int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
    int lo = li - CYL_WALL,        ro = ri + CYL_WALL;
    int cyl_top  = engine_top + HEAD_H;
    int case_top = engine_top + CASE_TOP_OFF;

    wattron(w, COLOR_PAIR(CP_WALL));
    for (int r = cyl_top; r < case_top; r++) {
        safeaddch(w, r, lo, '|');
        safeaddch(w, r, ro, '|');
        safeaddch(w, r, li, '|');
        safeaddch(w, r, ri, '|');
    }
    wattroff(w, COLOR_PAIR(CP_WALL));
}

/* ── 6.2  head ────────────────────────────────────────────────────── */
static void draw_head_2stroke(WINDOW *w, int engine_top, int center_col, bool spark)
{
    int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
    int lo = li - CYL_WALL,        ro = ri + CYL_WALL;

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
}

/*
 * 4-stroke head: spark plug in centre, exhaust valve at -VALVE_OFF,
 * intake valve at +VALVE_OFF. Valve glyphs:
 *   '#' = seated (closed),  'v' = lifted (open).
 */
static void draw_head_4stroke(WINDOW *w, int engine_top, int center_col,
                              bool spark, bool intake_open, bool exhaust_open)
{
    int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
    int lo = li - CYL_WALL,        ro = ri + CYL_WALL;
    int v_left  = center_col - VALVE_OFF;
    int v_right = center_col + VALVE_OFF;

    /* Head fill — leave the valve and spark-plug slots for later. */
    wattron(w, COLOR_PAIR(CP_WALL) | A_BOLD);
    for (int c = lo; c <= ro; c++) {
        chtype ch = '-';
        if (c == lo || c == ro) {
            ch = '+';
        } else if (c == v_left || c == v_right) {
            continue;
        } else if (c >= center_col - 1 && c <= center_col + 1) {
            continue;
        }
        safeaddch(w, engine_top, c, ch);
    }
    wattroff(w, COLOR_PAIR(CP_WALL) | A_BOLD);

    /* Spark plug. */
    if (spark) {
        wattron(w, COLOR_PAIR(CP_SPARK) | A_BOLD);
        safeaddch(w, engine_top, center_col - 1, '[');
        safeaddch(w, engine_top, center_col,     '*');
        safeaddch(w, engine_top, center_col + 1, ']');
        wattroff(w, COLOR_PAIR(CP_SPARK) | A_BOLD);
    } else {
        wattron(w, COLOR_PAIR(CP_WALL));
        safeaddch(w, engine_top, center_col - 1, '[');
        safeaddch(w, engine_top, center_col,     'i');
        safeaddch(w, engine_top, center_col + 1, ']');
        wattroff(w, COLOR_PAIR(CP_WALL));
    }

    /* Exhaust valve (left). */
    wattron(w, COLOR_PAIR(CP_EXHAUST) | A_BOLD);
    safeaddch(w, engine_top, v_left, exhaust_open ? 'v' : '#');
    wattroff(w, COLOR_PAIR(CP_EXHAUST) | A_BOLD);

    /* Intake valve (right). */
    wattron(w, COLOR_PAIR(CP_INTAKE) | A_BOLD);
    safeaddch(w, engine_top, v_right, intake_open ? 'v' : '#');
    wattroff(w, COLOR_PAIR(CP_INTAKE) | A_BOLD);
}

/* 6-stroke head: 4-stroke layout plus a water injector above the head. */
static void draw_head_6stroke(WINDOW *w, int engine_top, int center_col,
                              bool spark, bool intake_open, bool exhaust_open,
                              bool water_open)
{
    draw_head_4stroke(w, engine_top, center_col, spark, intake_open, exhaust_open);

    int wr = engine_top - 1;
    if (wr < 0) return;
    wattron(w, COLOR_PAIR(CP_INTAKE) | A_BOLD);
    safeaddch(w, wr, center_col, water_open ? 'w' : 'W');
    wattroff(w, COLOR_PAIR(CP_INTAKE) | A_BOLD);
}

/* ── 6.3  2-stroke port stubs ────────────────────────────────────── */
/*
 * Always visible so the wall ports stay recognisable even when the
 * piston has them covered. Port closed: dim outline of the pipe.
 * Port open: bright flow chars (exhaust '~', transfer '>') plus a
 * trailing plume.
 */
static void draw_ports_2stroke(WINDOW *w, int engine_top, int center_col,
                               const Engine *e)
{
    int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
    int lo = li - CYL_WALL,        ro = ri + CYL_WALL;
    int er = engine_top + EX_PORT_OFF;
    int tr = engine_top + TR_PORT_OFF;

    /* Exhaust manifold (left). */
    if (e->ex_port_open) {
        wattron(w, COLOR_PAIR(CP_EXHAUST) | A_BOLD);
        safeaddch(w, er - 1, lo - 1, '/');
        safeaddch(w, er + 1, lo - 1, '\\');
        for (int c = lo - 5; c < lo; c++) safeaddch(w, er, c, '~');
        wattroff(w, COLOR_PAIR(CP_EXHAUST) | A_BOLD);
        wattron(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
        for (int c = lo - 9; c < lo - 5; c++) safeaddch(w, er, c, '.');
        wattroff(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
    } else {
        wattron(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
        safeaddch(w, er - 1, lo - 1, '/');
        safeaddch(w, er + 1, lo - 1, '\\');
        for (int c = lo - 5; c < lo; c++) safeaddch(w, er, c, '-');
        wattroff(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
    }

    /* Transfer duct (right). */
    if (e->tr_port_open) {
        wattron(w, COLOR_PAIR(CP_INTAKE) | A_BOLD);
        safeaddch(w, tr - 1, ro + 1, '\\');
        safeaddch(w, tr + 1, ro + 1, '/');
        for (int c = ro + 1; c <= ro + 5; c++) safeaddch(w, tr, c, '>');
        wattroff(w, COLOR_PAIR(CP_INTAKE) | A_BOLD);
        wattron(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
        for (int c = ro + 6; c <= ro + 9; c++) safeaddch(w, tr, c, '+');
        wattroff(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
    } else {
        wattron(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
        safeaddch(w, tr - 1, ro + 1, '\\');
        safeaddch(w, tr + 1, ro + 1, '/');
        for (int c = ro + 1; c <= ro + 5; c++) safeaddch(w, tr, c, '-');
        wattroff(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
    }
}

/* ── 6.3b  4/6-stroke head manifolds ──────────────────────────────── */
/*
 * Pipes extending horizontally from the sides of the head row. Always
 * visible — dim '-' when the valve is seated, bright flow chars when
 * the valve is lifted. Distinguishes valve-head engines from the
 * port-wall geometry of the 2-stroke.
 */
static void draw_manifolds_valves(WINDOW *w, int engine_top, int center_col,
                                  const Engine *e)
{
    int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
    int lo = li - CYL_WALL,        ro = ri + CYL_WALL;
    int row = engine_top;

    /* Exhaust manifold extending left. */
    chtype ex_attr = e->exhaust_valve_open ? (COLOR_PAIR(CP_EXHAUST) | A_BOLD)
                                            : (COLOR_PAIR(CP_EXHAUST) | A_DIM);
    char   ex_ch   = e->exhaust_valve_open ? '~' : '-';
    wattron(w, ex_attr);
    for (int c = lo - 5; c < lo; c++) safeaddch(w, row, c, ex_ch);
    wattroff(w, ex_attr);

    /* Intake manifold extending right. */
    chtype in_attr = e->intake_valve_open ? (COLOR_PAIR(CP_INTAKE) | A_BOLD)
                                           : (COLOR_PAIR(CP_INTAKE) | A_DIM);
    char   in_ch   = e->intake_valve_open ? '>' : '-';
    wattron(w, in_attr);
    for (int c = ro + 1; c <= ro + 5; c++) safeaddch(w, row, c, in_ch);
    wattroff(w, in_attr);
}

/* ── 6.4  gas above piston — per-phase chemistry visualisation ───── */
static void draw_gas_above_piston(WINDOW *w, int engine_top, int center_col,
                                   int crown_row, Phase phase)
{
    int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
    int cyl_top = engine_top + HEAD_H;

    switch (phase) {
    case PHASE_IGNITE:
        wattron(w, COLOR_PAIR(CP_SPARK) | A_BOLD);
        for (int r = cyl_top; r < crown_row; r++)
            for (int c = li + 1; c < ri; c++)
                safeaddch(w, r, c, ((r + c) & 1) ? '*' : '^');
        wattroff(w, COLOR_PAIR(CP_SPARK) | A_BOLD);
        break;

    case PHASE_POWER: {
        static const char pch[] = "^~`";
        wattron(w, COLOR_PAIR(CP_FIRE));
        for (int r = cyl_top; r < crown_row; r++)
            for (int c = li + 1; c < ri; c++)
                safeaddch(w, r, c, pch[(r + c) % 3]);
        wattroff(w, COLOR_PAIR(CP_FIRE));
        break;
    }

    case PHASE_EXHAUST:
        wattron(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
        for (int r = cyl_top; r < crown_row; r++)
            for (int c = li + 1; c < ri; c++)
                safeaddch(w, r, c, '~');
        wattroff(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
        break;

    case PHASE_INTAKE:
        wattron(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
        for (int r = cyl_top; r < crown_row; r++)
            for (int c = li + 1; c < ri; c++)
                safeaddch(w, r, c, '+');
        wattroff(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
        break;

    case PHASE_SCAVENGE: {
        int mid = (li + ri) / 2;
        for (int r = cyl_top; r < crown_row; r++) {
            wattron(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
            for (int c = li + 1; c <= mid; c++) safeaddch(w, r, c, '~');
            wattroff(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
            wattron(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
            for (int c = mid + 1; c < ri; c++) safeaddch(w, r, c, '+');
            wattroff(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
        }
        break;
    }

    case PHASE_WATER_INJ:
        wattron(w, COLOR_PAIR(CP_INTAKE) | A_BOLD);
        for (int r = cyl_top; r < crown_row; r++)
            for (int c = li + 1; c < ri; c++)
                safeaddch(w, r, c, ((r + c) & 1) ? '.' : ',');
        wattroff(w, COLOR_PAIR(CP_INTAKE) | A_BOLD);
        break;

    case PHASE_STEAM_POWER:
        wattron(w, COLOR_PAIR(CP_SPARK));
        for (int r = cyl_top; r < crown_row; r++)
            for (int c = li + 1; c < ri; c++)
                safeaddch(w, r, c, ((r + c) % 3) == 0 ? '%' : '"');
        wattroff(w, COLOR_PAIR(CP_SPARK));
        break;

    case PHASE_STEAM_EXHAUST:
        wattron(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
        for (int r = cyl_top; r < crown_row; r++)
            for (int c = li + 1; c < ri; c++)
                safeaddch(w, r, c, '"');
        wattroff(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
        break;

    case PHASE_COMPRESS:
    default:
        /* Compressed charge intentionally invisible. */
        break;
    }
}

/* ── 6.5  piston / conrod / crank / case ──────────────────────────── */
static void draw_piston(WINDOW *w, int center_col, int crown_row)
{
    int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
    wattron(w, COLOR_PAIR(CP_PISTON) | A_BOLD);
    for (int r = crown_row; r < crown_row + PISTON_H; r++) {
        safeaddch(w, r, li, '[');
        for (int c = li + 1; c < ri; c++)
            safeaddch(w, r, c, (r == crown_row) ? '=' : '#');
        safeaddch(w, r, ri, ']');
    }
    wattroff(w, COLOR_PAIR(CP_PISTON) | A_BOLD);
}

static void draw_conrod(WINDOW *w, int center_col, int wp_row, int cp_row, int cp_col)
{
    wattron(w, COLOR_PAIR(CP_CONROD));
    draw_line_ch(w, wp_row, center_col, cp_row, cp_col, ':');
    safeaddch(w, wp_row, center_col, 'o');
    wattroff(w, COLOR_PAIR(CP_CONROD));
}

static void draw_crank(WINDOW *w, int cc_row, int center_col, int cp_row, int cp_col)
{
    /* Aspect-corrected orbit indicator (looks circular in cells). */
    wattron(w, COLOR_PAIR(CP_CRANK) | A_DIM);
    for (int deg = 0; deg < 360; deg += 12) {
        float a = (float)deg * (float)M_PI / 180.0f;
        int er = cc_row + (int)roundf((float)CRANK_R * 0.5f * sinf(a));
        int ec = center_col + (int)roundf((float)CRANK_R * cosf(a));
        safeaddch(w, er, ec, '.');
    }
    wattroff(w, COLOR_PAIR(CP_CRANK) | A_DIM);

    wattron(w, COLOR_PAIR(CP_CRANK));
    draw_line_ch(w, cc_row, center_col, cp_row, cp_col, '-');
    safeaddch(w, cc_row, center_col, 'O');
    wattron(w, A_BOLD);
    safeaddch(w, cp_row, cp_col, 'o');
    wattroff(w, A_BOLD);
    wattroff(w, COLOR_PAIR(CP_CRANK));
}

static void draw_case(WINDOW *w, int engine_top, int center_col, int cc_r)
{
    int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
    int lo = li - CYL_WALL, ro = ri + CYL_WALL;
    int case_top = engine_top + CASE_TOP_OFF;
    int case_bot = engine_top + CASE_BOT_OFF;
    int case_lw  = center_col - CASE_HW;
    int case_rw  = center_col + CASE_HW;

    wattron(w, COLOR_PAIR(CP_WALL));
    /* Top bar — leave the cylinder bore open. */
    for (int c = case_lw; c <= case_rw; c++) {
        chtype ch;
        if (c == case_lw || c == case_rw) ch = '+';
        else if (c >= lo && c <= ro)      ch = ' ';
        else                              ch = '-';
        safeaddch(w, case_top, c, ch);
    }
    for (int r = case_top + 1; r < case_bot; r++) {
        safeaddch(w, r, case_lw, '|');
        safeaddch(w, r, case_rw, '|');
    }
    for (int c = case_lw; c <= case_rw; c++)
        safeaddch(w, case_bot, c, (c == case_lw || c == case_rw) ? '+' : '-');
    /* Power-takeoff stub on the right. */
    safeaddch(w, cc_r, case_rw,     '>');
    safeaddch(w, cc_r, case_rw + 1, '=');
    safeaddch(w, cc_r, case_rw + 2, '>');
    wattroff(w, COLOR_PAIR(CP_WALL));
}

/* ── 6.6  stroke timeline ─────────────────────────────────────────── */
/*
 * The timeline is a horizontal "phase ribbon" beneath the engine that
 * shows the cycle as N labelled stroke blocks with a caret pointing
 * at the current cycle position. N is the engine's stroke count
 * (2/4/6), so the ribbon's structure encodes the cycle complexity at
 * a glance.
 *
 * Drawing it once meant inlining several concerns in one function.
 * They are split here into named helpers (layout / stroke index /
 * label lookup / opener bracket / block render / position caret) so
 * the top-level draw routine reads as pseudocode.
 */

/* Pre-computed ribbon geometry. Block widths shrink as the cycle gets
 * longer (more strokes packed into roughly the same total width),
 * keeping the ribbon visually anchored to the cylinder centerline. */
typedef struct {
    int  row;          /* row to draw the labelled blocks on   */
    int  start_col;    /* leftmost column of the ribbon        */
    int  block_w;      /* one block's full width incl. sep     */
    int  total_w;      /* strokes * block_w + 1 closing ']'    */
    bool fits;         /* false if it does not fit on screen   */
} TimelineLayout;

static TimelineLayout timeline_layout_compute(WINDOW *w, int engine_top,
                                              int center_col, int strokes)
{
    int rows, cols;
    getmaxyx(w, rows, cols);

    TimelineLayout L;
    L.row       = engine_top + ENGINE_H + 1;
    L.block_w   = (strokes == 2) ? 22 : (strokes == 4) ? 11 : 9;
    L.total_w   = strokes * L.block_w + 1;
    L.start_col = center_col - L.total_w / 2;
    L.fits      = (L.row + 1 < rows - 1) &&
                  (L.start_col >= 0) &&
                  (L.start_col + L.total_w <= cols);
    return L;
}

/* Map cycle angle to stroke index.  Each stroke spans pi rad of
 * cycle_angle (half a crank revolution), so the index is simply
 * floor(cycle_angle / pi), clamped for safety. */
static int current_stroke_from_cycle_angle(float cycle_angle, int strokes)
{
    int s = (int)(cycle_angle / (float)M_PI);
    if (s < 0)        s = 0;
    if (s >= strokes) s = strokes - 1;
    return s;
}

/* Lookup: stroke labels for each engine type. */
static const char **stroke_labels_for_type(EngineType t)
{
    static const char *lbl_2s[] = { "POWER + EXH/SCAV", "COMPRESSION" };
    static const char *lbl_4s[] = { "INTAKE", "COMPRESS", "POWER", "EXHAUST" };
    static const char *lbl_6s[] = { "INTAKE", "COMPRES", "POWER", "EXHAUST",
                                    "STM PWR", "STM EXH" };
    return (t == ENG_2STROKE) ? lbl_2s
         : (t == ENG_4STROKE) ? lbl_4s : lbl_6s;
}

/* Render one labelled stroke block plus its trailing separator
 * ('|' between blocks, ']' after the last). Active block is
 * highlighted in CP_PHASE/A_BOLD; inactive blocks dim. */
static void render_stroke_block(WINDOW *w, int row, int block_start,
                                int inner_w, const char *label,
                                bool is_active, bool is_last)
{
    int lbl_len = (int)strlen(label);
    if (lbl_len > inner_w) lbl_len = inner_w;
    int padding = (inner_w - lbl_len) / 2;

    chtype attr = is_active ? (COLOR_PAIR(CP_PHASE) | A_BOLD)
                            : (COLOR_PAIR(CP_WALL)  | A_DIM);
    wattron(w, attr);
    for (int k = 0; k < inner_w; k++) {
        int lp = k - padding;
        char ch = (lp >= 0 && lp < lbl_len) ? label[lp] : ' ';
        safeaddch(w, row, block_start + k, ch);
    }
    wattroff(w, attr);

    wattron(w, COLOR_PAIR(CP_WALL) | A_BOLD);
    safeaddch(w, row, block_start + inner_w, is_last ? ']' : '|');
    wattroff(w, COLOR_PAIR(CP_WALL) | A_BOLD);
}

/* Render the '[' that opens the ribbon. */
static void render_ribbon_opener(WINDOW *w, const TimelineLayout *L)
{
    wattron(w, COLOR_PAIR(CP_WALL) | A_BOLD);
    safeaddch(w, L->row, L->start_col, '[');
    wattroff(w, COLOR_PAIR(CP_WALL) | A_BOLD);
}

/* Render the cycle-position caret '^' under the ribbon, mapped from
 * cycle_fraction in [0,1] to a column proportional inside the ribbon
 * (clamped so it never lands on a separator). */
static void render_cycle_position_caret(WINDOW *w, int row,
                                         const TimelineLayout *L,
                                         float cycle_fraction)
{
    int c = L->start_col + 1 + (int)(cycle_fraction * (L->total_w - 2));
    if (c < L->start_col + 1)              c = L->start_col + 1;
    if (c > L->start_col + L->total_w - 2) c = L->start_col + L->total_w - 2;
    wattron(w, COLOR_PAIR(CP_PHASE) | A_BOLD);
    safeaddch(w, row, c, '^');
    wattroff(w, COLOR_PAIR(CP_PHASE) | A_BOLD);
}

/* Top-level: assemble the ribbon from its parts. */
static void draw_stroke_timeline(WINDOW *w, const Engine *e,
                                 int engine_top, int center_col)
{
    int strokes = engine_strokes(e->type);
    TimelineLayout L = timeline_layout_compute(w, engine_top, center_col, strokes);
    if (!L.fits) return;

    int cur_stroke = current_stroke_from_cycle_angle(e->cycle_angle, strokes);
    const char **labels = stroke_labels_for_type(e->type);

    /* "[ block_0 | block_1 | ... | block_{N-1} ]" */
    render_ribbon_opener(w, &L);
    for (int s = 0; s < strokes; s++) {
        int block_start = L.start_col + 1 + s * L.block_w;
        render_stroke_block(w, L.row, block_start, L.block_w - 1,
                            labels[s], s == cur_stroke, s == strokes - 1);
    }

    float cycle_fraction = e->cycle_angle / engine_cycle_total(e->type);
    render_cycle_position_caret(w, L.row + 1, &L, cycle_fraction);
}

/* ── 6.7  scene_draw — top-level orchestrator ─────────────────────── */
/*
 * One frame, built as five named passes that mirror the physics
 * pipeline and the back-to-front layering of the cross-section:
 *
 *   1. Mechanics       — solve slider-crank for piston/wrist/crank pin.
 *   2. State machine   — derive phase + valve + port flags.
 *   3. Structural pass — walls (background), gas chamber chemistry,
 *                        head (foreground, layered on top of gas so
 *                        valves and the injector stay visible).
 *   4. Moving parts    — piston -> conrod -> crank, then the crankcase
 *                        enclosure layered on top of overhangs.
 *   5. Overlays        — phase telemetry, port state, stroke timeline.
 *
 * Each step is one helper call so the orchestrator reads as
 * pseudocode. The helpers below carry the per-step intent.
 */

/* Step 1: pure mechanics. Wrap the crank-angle / centre-position
 * unpack + the closed-form slider-crank solve into one frame-level
 * call. Cell coordinates of all four reference points are returned
 * atomically (see Kinematics doc and Norton §4.5). */
static Kinematics solve_frame_kinematics(const Engine *e,
                                         int engine_top, int center_col)
{
    float crank_th = engine_crank_angle(e);
    float cc_row_f = (float)(engine_top + CRANK_CENTER_OFF);
    float cc_col_f = (float)center_col;
    return solve_slider_crank(crank_th, cc_row_f, cc_col_f);
}

/* Step 3 (background): cylinder walls — geometry differs by gas-
 * exchange mechanism. 2-stroke walls have ports cut at fixed rows;
 * 4/6-stroke walls are solid (gas exchange goes through head valves). */
static void render_cylinder_walls(WINDOW *w, int engine_top, int center_col,
                                  const Engine *e)
{
    if (e->type == ENG_2STROKE) draw_walls_2stroke(w, engine_top, center_col, e);
    else                        draw_walls_solid(w, engine_top, center_col);
}

/* Step 3 (foreground): cylinder head and per-type adornments. Drawn
 * AFTER the gas chamber so valves and the injector overlay the gas.
 * The head is where the three engines differ most:
 *   2-stroke -> spark plug only;       wall-port stubs sit at port rows.
 *   4-stroke -> spark + 2 poppet valves; side manifolds at head row.
 *   6-stroke -> 4-stroke layout       + water injector above the head.
 */
static void render_cylinder_head(WINDOW *w, int engine_top, int center_col,
                                 const Engine *e)
{
    bool spark = (e->phase == PHASE_IGNITE);
    switch (e->type) {
    case ENG_2STROKE:
        draw_head_2stroke (w, engine_top, center_col, spark);
        draw_ports_2stroke(w, engine_top, center_col, e);
        break;
    case ENG_4STROKE:
        draw_head_4stroke    (w, engine_top, center_col, spark,
                              e->intake_valve_open, e->exhaust_valve_open);
        draw_manifolds_valves(w, engine_top, center_col, e);
        break;
    case ENG_6STROKE:
        draw_head_6stroke    (w, engine_top, center_col, spark,
                              e->intake_valve_open, e->exhaust_valve_open,
                              e->water_inj_open);
        draw_manifolds_valves(w, engine_top, center_col, e);
        break;
    }
}

/* Step 4: reciprocating train — piston -> wrist pin -> conrod ->
 * crank pin -> crankshaft. The moving column that converts gas
 * pressure into shaft rotation. Drawn in this order so the conrod
 * sits on top of the crank arm at the pin, matching real cutaways. */
static void render_reciprocating_train(WINDOW *w, int center_col, int cc_r,
                                        const Kinematics *k)
{
    draw_piston(w, center_col,  k->crown_row);
    draw_conrod(w, center_col,  k->wrist_row, k->crank_pin_r, k->crank_pin_c);
    draw_crank (w, cc_r, center_col, k->crank_pin_r, k->crank_pin_c);
}

/* Step 5 (overlay): phase telemetry — three labelled rows to the right
 * of the cylinder showing the state-machine label and the two angle
 * readouts. Diagnostic overlay; does not interact with engine state. */
static void render_phase_telemetry(WINDOW *w, int engine_top, int center_col,
                                    const Engine *e)
{
    int ri = center_col + CYL_IHW;
    int ro = ri + CYL_WALL;

    wattron(w, COLOR_PAIR(CP_PHASE) | A_BOLD);
    safeaddstr(w, engine_top + 3, ro + 3, phase_name(e->phase));
    wattroff(w, COLOR_PAIR(CP_PHASE) | A_BOLD);

    char buf[40];
    float crank_th = engine_crank_angle(e);
    int deg_crank = (int)(crank_th       * 180.0f / (float)M_PI) % 360;
    int deg_cycle = (int)(e->cycle_angle * 180.0f / (float)M_PI);
    wattron(w, COLOR_PAIR(CP_HUD));
    snprintf(buf, sizeof(buf), "crank %3d deg", deg_crank);
    safeaddstr(w, engine_top + 5, ro + 3, buf);
    snprintf(buf, sizeof(buf), "cycle %4d deg", deg_cycle);
    safeaddstr(w, engine_top + 6, ro + 3, buf);
    wattroff(w, COLOR_PAIR(CP_HUD));
}

/* Step 5 (overlay): 2-stroke port-state — "EX" / "TR" labels next to
 * each wall port that confirm whether the port is currently open. */
static void render_port_state_overlay(WINDOW *w, int engine_top, int center_col,
                                       const Engine *e)
{
    int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
    int lo = li - CYL_WALL,        ro = ri + CYL_WALL;
    wattron(w, COLOR_PAIR(CP_HUD));
    safeaddstr(w, engine_top + EX_PORT_OFF, lo - 3,
               e->ex_port_open ? "EX" : "  ");
    safeaddstr(w, engine_top + TR_PORT_OFF, ro + 3,
               e->tr_port_open ? "TR" : "  ");
    wattroff(w, COLOR_PAIR(CP_HUD));
}

/* Top-level: one frame as a sequence of named passes. */
static void scene_draw(WINDOW *w, Engine *e, int engine_top, int center_col)
{
    /* 1. Mechanics: cell positions of crown, wrist pin, crank pin. */
    Kinematics k = solve_frame_kinematics(e, engine_top, center_col);
    int cc_r = engine_top + CRANK_CENTER_OFF;

    /* 2. State machine: phase + valve + port flags for this frame. */
    engine_derive_state(e, k.crown_row, engine_top);

    /* 3. Structural pass, back-to-front: walls -> chamber gas -> head.
     *    Head goes last so its valves and injector sit on top of gas. */
    render_cylinder_walls(w, engine_top, center_col, e);
    draw_gas_above_piston(w, engine_top, center_col, k.crown_row, e->phase);
    render_cylinder_head(w, engine_top, center_col, e);

    /* 4. Moving parts inside, then the crankcase that contains them. */
    render_reciprocating_train(w, center_col, cc_r, &k);
    draw_case(w, engine_top, center_col, cc_r);

    /* 5. Overlays: diagnostic text and cycle-structure timeline. */
    render_phase_telemetry(w, engine_top, center_col, e);
    if (e->type == ENG_2STROKE)
        render_port_state_overlay(w, engine_top, center_col, e);
    draw_stroke_timeline(w, e, engine_top, center_col);
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * Scene — top-level application state for the main loop.
 *
 * Composition, not inheritance: the simulation owns the engine, the
 * presentation owns the theme and the sim-step rate. Bundling both
 * into one struct means the main loop passes a single pointer to
 * every helper and input handlers mutate either side without
 * threading separate pointers through the call chain.
 *
 * Two distinct groups of parameters, separated below for clarity.
 *
 *   Simulation params — affect what the engine *does* each tick.
 *     Persist across resizes, theme changes, and engine-type switches.
 *     Mutated by 'r' (reset), '['/']' (RPM), space (pause),
 *     'n'/'p' (engine type).
 *
 *   Rendering params — affect what the screen *looks like*.
 *     Do not alter the simulation in any way. Mutated by 't'/'T'.
 *     A different theme drawn over the same engine state must
 *     produce identical physics (cycle_angle, valve states, phase).
 *
 * Locality rationale:
 *   The split exists for the *reader*, not the CPU. Scene is small
 *   enough (~70 bytes) that the whole thing lives in one cache line;
 *   no field ordering trick will measurably move the needle. What
 *   matters is that future changes — adding a parameter, swapping a
 *   theme — know exactly which side to land on. A field added on the
 *   wrong side of the split is the kind of bug that causes pause to
 *   silently change the theme or theme cycling to bump the RPM.
 *
 * Why not split into SceneSim + SceneRender structs?
 *   At two and one field respectively, two separate structs would be
 *   more ceremony than the locality buys. If this scene grew
 *   substantially (multi-cylinder engine, camera state, recording
 *   buffer) the split would earn its keep — for now, a comment
 *   boundary is enough.
 */
typedef struct {
    /* ── Simulation parameters ─────────────────────────────────── */
    Engine engine;          /* full ICE state (type, cycle_angle, RPM, ...)  */
    int    sim_fps;         /* fixed-timestep rate, Hz.
                             *   TICK_NS(sim_fps) is the simulation period
                             *   driving the inner while-accumulator loop.
                             *   Independent of render fps (capped at 60).  */

    /* ── Rendering parameters ──────────────────────────────────── */
    int    theme_idx;       /* index into THEMES[] — cycled with t / T.
                             *   Read by draw_hud() for the readout and by
                             *   color_apply_theme() to remap CP_WALL..
                             *   CP_SPARK. Never read inside §5 (engine).  */
} Scene;

static void scene_init(Scene *s)
{
    engine_reset(&s->engine, ENG_2STROKE);
    s->sim_fps   = SIM_FPS_DEFAULT;
    s->theme_idx = 0;
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit   = 0;

static void on_sigwinch(int s) { (void)s; g_resize = 1; }
static void on_sigterm(int s)  { (void)s; g_quit   = 1; }

static void screen_init(int theme_idx)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    color_init(theme_idx);
}

static void screen_cleanup(void)
{
    curs_set(1);
    endwin();
}

/*
 * Two-bar HUD per project convention:
 *   row 0       — engine type / fps / sim rate / RPM / paused state,
 *                 bright yellow + bold
 *   row 1       — theme readout, bright yellow (no bold)
 *   row rows-1  — interactive key hints, bright cyan + bold
 */
static void draw_hud(WINDOW *w, const Scene *s, float fps)
{
    int rows, cols;
    getmaxyx(w, rows, cols);

    char top[100];
    snprintf(top, sizeof(top),
             " %s  %5.1f fps  sim:%3d Hz  RPM:%3d  %s ",
             engine_name(s->engine.type),
             (double)fps, s->sim_fps, s->engine.rpm,
             s->engine.paused ? "PAUSED " : "running");
    int top_len = (int)strlen(top);
    wattron(w, COLOR_PAIR(CP_HUD) | A_BOLD);
    safeaddstr(w, 0, cols - top_len, top);
    wattroff(w, COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Row 1 left: cycle subtitle naming the thermodynamic cycle. */
    char sub_left[80];
    snprintf(sub_left, sizeof(sub_left), " %s ",
             engine_subtitle(s->engine.type));
    wattron(w, COLOR_PAIR(CP_HUD));
    safeaddstr(w, 1, 0, sub_left);
    wattroff(w, COLOR_PAIR(CP_HUD));

    /* Row 1 right: theme readout. */
    char sub[40];
    snprintf(sub, sizeof(sub), " theme: %-7s ", THEMES[s->theme_idx].name);
    int sub_len = (int)strlen(sub);
    wattron(w, COLOR_PAIR(CP_HUD));
    safeaddstr(w, 1, cols - sub_len, sub);
    wattroff(w, COLOR_PAIR(CP_HUD));

    const char *hint =
        " q:quit  spc:pause  r:reset  ]/[:RPM  n/p:engine  t/T:theme ";
    wattron(w, COLOR_PAIR(CP_HINT) | A_BOLD);
    safeaddstr(w, rows - 1, 0, hint);
    wattroff(w, COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

int main(void)
{
    signal(SIGWINCH, on_sigwinch);
    signal(SIGTERM,  on_sigterm);
    signal(SIGINT,   on_sigterm);

    Scene scene;
    scene_init(&scene);

    screen_init(scene.theme_idx);

    int64_t last_time  = clock_ns();
    int64_t sim_accum  = 0;
    int64_t fps_accum  = 0;
    int     fps_frames = 0;
    float   fps_disp   = 0.0f;
    int64_t frame_ns   = TICK_NS(60);   /* render cap ~60 fps */

    int scr_rows, scr_cols;
    getmaxyx(stdscr, scr_rows, scr_cols);

    for (;;) {
        if (g_resize) {
            g_resize = 0;
            endwin();
            refresh();
            getmaxyx(stdscr, scr_rows, scr_cols);
        }
        if (g_quit) break;

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

        int64_t tick = TICK_NS(scene.sim_fps);
        while (sim_accum >= tick) {
            float dt = (float)tick / (float)NS_PER_SEC;
            engine_tick(&scene.engine, dt);
            sim_accum -= tick;
        }

        /* Reserve rows 0-1 for HUD; row 2 holds the 6-stroke water
         * injector marker above the head without overlapping HUD. */
        int engine_top = (scr_rows - ENGINE_H) / 2;
        if (engine_top < 3) engine_top = 3;
        int center_col = scr_cols / 2;

        erase();
        scene_draw(stdscr, &scene.engine, engine_top, center_col);
        draw_hud(stdscr, &scene, fps_disp);
        wnoutrefresh(stdscr);
        doupdate();

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q':
            case 27:
                g_quit = 1;
                break;
            case ' ':
                scene.engine.paused = !scene.engine.paused;
                break;
            case 'r':
                engine_reset(&scene.engine, scene.engine.type);
                break;
            case ']':
                if (scene.engine.rpm < RPM_MAX) scene.engine.rpm += RPM_STEP;
                break;
            case '[':
                if (scene.engine.rpm > RPM_MIN) scene.engine.rpm -= RPM_STEP;
                break;
            case 'n':
                engine_set_type(&scene.engine,
                    (EngineType)((scene.engine.type + 1) % N_ENGINES));
                break;
            case 'p':
                engine_set_type(&scene.engine,
                    (EngineType)((scene.engine.type + N_ENGINES - 1) % N_ENGINES));
                break;
            case 't':
                scene.theme_idx = (scene.theme_idx + 1) % N_THEMES;
                color_apply_theme(scene.theme_idx);
                break;
            case 'T':
                scene.theme_idx = (scene.theme_idx + N_THEMES - 1) % N_THEMES;
                color_apply_theme(scene.theme_idx);
                break;
            case KEY_RESIZE:
                g_resize = 1;
                break;
            }
        }

        int64_t elapsed = clock_ns() - now;
        clock_sleep_ns(frame_ns - elapsed);
    }

    screen_cleanup();
    return 0;
}
