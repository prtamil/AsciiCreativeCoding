/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cobweb_diagram.c
 *   — Visualise a 1-D iterated map x_{n+1} = f(x_n) by the classic
 *     cobweb (staircase) construction.  Companion to ./bifurcation.c:
 *     bifurcation shows the long-term attractor across ALL r in one
 *     picture; this file shows the geometric MECHANISM by which a
 *     SINGLE r produces that attractor.
 *
 * DEMO: Draw f(x) = r·x(1-x) as a parabola plus the diagonal y = x.
 *       Iterate x_n: each step draws (x_n, x_n) → (x_n, f(x_n)) →
 *       (f(x_n), f(x_n)) → ... building a "staircase" that visibly
 *       converges to a fixed point, settles on a periodic cycle, or
 *       wanders chaotically.  30 presets stepping simple → complex
 *       across the r ∈ [1, 4] range:
 *         Class A  fixed-point regime    (5 presets, r ∈ [1.5, 2.95])
 *         Class B  period-doubling       (8 presets, r ∈ [3.05, 3.57])
 *         Class C  chaos + windows       (17 presets, r ∈ [3.575, 4.0])
 *
 * Study alongside:
 *   ./bifurcation.c       — the COMPLEMENTARY view: all r at once.
 *   ./sensitive_dependence.c — the temporal divergence side.
 *
 * Section map:
 *   §1 config   — constants, presets, themes, sim/render budgets
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD pairs + 10 themes
 *   §5 map      — LogisticMap (the function) + MapIterator (the orbit)
 *   §6 cobweb   — CobwebRing: age-banded segment ring buffer
 *   §7 viewport — UnitViewport: unit square ↔ cell mapping
 *   §8 scene    — PresetState + PaletteState + Scene composite
 *   §9 screen   — Screen + paint_* + HUD helpers + screen_draw
 *   §10 app     — App + signals + main game loop helpers
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (re-apply current preset, fresh x₀)
 *   n / N      next preset    (p / P previous)
 *   t / T      next / previous theme
 *   ] / [      raise / lower simulation tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/chaos/cobweb_diagram.c \
 *       -o cobweb_diagram -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Logistic map f(x) = r·x(1 − x) on the unit square.
 *                 Each tick advances the iteration index by one and
 *                 records TWO cobweb segments per step (vertical
 *                 (x, x) → (x, f(x)) and horizontal (x, f(x)) →
 *                 (f(x), f(x))).  Segments live in a ring buffer
 *                 with age-based fade so the most recent steps are
 *                 bright and old ones fade.
 *
 * Data-struct   : LogisticMap (parameter r) + MapIterator (current x
 *                 + step count) — splits "the function" from "the
 *                 orbit through it".  CobwebRing — fixed-capacity
 *                 ring of (x0, y0, x1, y1, kind) quartets.
 *
 * Rendering     : Unit square [0, 1]² maps to a UnitViewport inside
 *                 the HUD-aware drawable area.  y=0 at BOTTOM; y=1
 *                 at TOP.  Parabola: '.' across cols.  Diagonal: '/'.
 *                 Cobweb: '|'/'-' coloured by recency band.
 *
 * References    : LOGISTIC MAP & PERIOD-DOUBLING
 *                 • May, R. M. (1976) — "Simple mathematical models
 *                   with very complicated dynamics", Nature 261.
 *                   Established the logistic map as the canonical
 *                   1-D chaotic system; survey of the bifurcation
 *                   regimes our 30 presets enumerate.
 *                 • Feigenbaum, M. J. (1978) — "Quantitative
 *                   universality for a class of nonlinear
 *                   transformations", J. Stat. Phys. 19.  Discovered
 *                   the universal period-doubling constant
 *                   δ ≈ 4.6692 — applies to ANY unimodal map, not
 *                   just logistic.  See preset FEIGNBM (r ≈ 3.5699).
 *                 • Sharkovskii, A. N. (1964) — "Coexistence of
 *                   cycles of a continuous map of a line into
 *                   itself", Ukrainian Math. Journal 16.  Period
 *                   ordering 3 ▻ 5 ▻ 7 ▻ … ▻ 2·3 ▻ 2·5 ▻ … ▻ 2ⁿ ▻ …
 *                   ▻ 4 ▻ 2 ▻ 1: presence of an earlier period in
 *                   this list IMPLIES presence of every later one.
 *                   Preset P3-WIND (r=3.835) sits on the leftmost
 *                   period — hence chaos is unavoidable beyond it.
 *                 • Li, T.-Y. & Yorke, J. A. (1975) — "Period three
 *                   implies chaos", Amer. Math. Monthly 82.  Their
 *                   famous Western-rediscovery of Sharkovskii's
 *                   corollary; coined the term "chaos" itself.
 *
 *                 BOOKS
 *                 • Strogatz, S. H. (1994) — "Nonlinear Dynamics
 *                   and Chaos".  §10 covers cobweb construction
 *                   with the logistic map as the running example.
 *                 • Devaney, R. L. (2003) — "An Introduction to
 *                   Chaotic Dynamical Systems" (2nd ed.).
 *                   Definitive textbook treatment of 1-D iterated
 *                   maps; chapters 1-3 use cobweb diagrams as the
 *                   primary visual tool for proving fixed-point
 *                   stability, periodicity, and chaos.
 *
 *                 PRACTICAL CATALOGUE
 *                 • "Logistic map" — Wikipedia:
 *                   https://en.wikipedia.org/wiki/Logistic_map
 *                   Catalogues the period-doubling cascade, the
 *                   periodic windows (p-3, p-5, p-6, p-7 etc.),
 *                   and the interior-crisis transitions — the
 *                   r-value source for our 30-preset tour.
 *
 *                 COMPARE IN PROJECT
 *                 • ./bifurcation.c          — same map, all-r view.
 *                 • ./sensitive_dependence.c — exponential divergence
 *                                              of two trajectories.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To find x_{n+1} from x_n:  go UP from (x_n, x_n) to the curve
 * y = f(x_n), then ACROSS to the diagonal y = x to "read off" the
 * new x-coordinate.  Repeating draws a staircase that visibly
 * converges (fixed point), cycles (periodic), or wanders (chaos).
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Fixed points = intersections of f(x) with the diagonal y = x.
 * Stability = slope of f at the fixed point.  |f'(x*)| < 1 →
 * stable (cobweb spirals/staircases INTO the point).  |f'(x*)| > 1
 * → unstable (cobweb pushes AWAY).  At |f'| = 1 you get bifurcations.
 *
 * KEY FORMULAS
 * ────────────
 *   f(x)        = r · x · (1 − x)
 *   Fixed pts   = {0} ∪ {1 − 1/r}
 *   Stability   = |f'(x*)| = |r(1 − 2x*)| < 1
 *   At r = 3 the non-trivial fixed point loses stability → period-2.
 *   Feigenbaum r∞ ≈ 3.5699 = accumulation point of bifurcations.
 *
 * HOW TO VERIFY  (try in sequence)
 * ─────────────
 *  • FP-SPIRL (r=2.80): staircase spirals tightly into x* ≈ 0.643.
 *  • P2-MID (r=3.20):   staircase settles into a 2-rectangle dance.
 *  • P4-BORN (r=3.46):  4-rectangle dance.
 *  • FEIGNBM (r=3.5699): the dance has infinitely many sub-rectangles.
 *  • P3-WIND (r=3.835): suddenly a clean 3-cycle in the middle of chaos.
 *  • FULL-MAX (r=4.00): staircase fills the unit square; no convergence.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN          =   1,
    SIM_FPS_DEFAULT      =   8,    /* slow default — cobweb is hand-paced */
    SIM_FPS_MAX          =  60,
    SIM_FPS_STEP         =   2,

    HUD_COLS             =  80,
    FPS_UPDATE_MS        = 500,

    PAIR_HUD             =   1,
    PAIR_HINT            =   2,
    PAIR_CURVE           =   3,    /* parabola f(x)                  */
    PAIR_DIAG            =   4,    /* diagonal y = x                  */
    PAIR_COBWEB_BASE     =   5,    /* 4 age bands: +0 oldest..+3 new  */
    PAIR_LIVE            =  10,    /* current (x, f(x)) live marker   */
};

#define HUD_TOP_ROWS              2
#define HUD_BOTTOM_ROWS           1
#define HUD_BAND_RESERVED_ROWS    (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN           1

#define NS_PER_SEC                1000000000LL
#define NS_PER_MS                    1000000LL
#define TICK_NS(f)                (NS_PER_SEC / (f))
#define RENDER_FPS_TARGET         60
#define RENDER_FRAME_BUDGET_NS    (NS_PER_SEC / RENDER_FPS_TARGET)
#define SIM_MAX_FRAME_DT_MS       100

#define COBWEB_AGE_BANDS           4
#define COBWEB_MAX              1024   /* total stored segments (2 per iterate) */
#define ITER_STEPS_PER_TICK        1   /* iterations advanced per tick */

/*
 * CobwebPreset — one entry in the simple-to-complex r-axis tour.
 *
 * INTENT
 *   The single bifurcation parameter r is the WHOLE dynamics of the
 *   logistic map.  We pre-pick 30 visually-distinct r values so the
 *   user can step through the canonical "route to chaos" without
 *   having to know which r values are interesting.  Same x0 across
 *   the entire table — the one knob changing is r.
 *
 * CONTEXT
 *   `presets[N_PRESETS]` immediately below this struct is the static
 *   .rodata table.  Indexed by the `Preset` enum (also below).
 *   `preset_state_active()` reads one row out for scene_load_preset.
 *
 * MEMBER LOGIC
 *   name : ≤ 8-char ALL-CAPS label shown in the HUD; padded with
 *          trailing spaces to exactly 8 chars so the HUD format
 *          string produces aligned columns when cycling presets.
 *   r    : bifurcation parameter ∈ [1, 4].  Above r∞ ≈ 3.5699
 *          (Feigenbaum point) chaos sets in; periodic windows
 *          (p-3, p-5, p-6, p-7) interrupt it at specific r values.
 *   x0   : initial iterate (kept at 0.10 for the whole table — the
 *          ONE thing varying is r).  Logistic map's basin of
 *          attraction covers (0, 1) for r ∈ [1, 4], so any non-zero
 *          x0 reaches the same long-term attractor.
 *
 * REFERENCES
 *   • May, R. M. (1976) — survey of the r-regimes our table tours.
 *   • Wikipedia "Logistic map" — practical catalogue of the periodic
 *     windows our table targets (P3-WIND, P5-WIND, etc.).
 */
typedef struct { const char *name; float r; float x0; } CobwebPreset;

/*
 * Preset — enum index into the 30-entry presets[] table.
 *
 * INTENT
 *   Named indices give every interesting r-value its own typed
 *   identifier.  Cycling forward through these names IS the
 *   "route to chaos" tour: the user sees fixed points → period
 *   doubling → Feigenbaum point → chaos → periodic windows →
 *   full chaos in order, exactly as Strogatz and Devaney present it.
 *
 * CONTEXT
 *   Used solely as an index into the `presets[]` table below.  Order
 *   matches the table strictly: A-class first (5), B-class (8),
 *   C-class (17), all monotonically increasing in r.  The compiler
 *   enforces table completeness via `[N_PRESETS]` sizing on the
 *   `presets[]` initialiser.
 *
 * CLASS BOUNDARIES (in r-space)
 *   Class A : r ∈ (1, 3)          — single stable fixed point
 *   Class B : r ∈ (3, r∞)         — period-doubling cascade
 *                                   r∞ ≈ 3.5699 = Feigenbaum point
 *   Class C : r ∈ (r∞, 4]         — chaos + periodic-window mix
 *
 * REFERENCES
 *   • Feigenbaum (1978) — explains the Class A → B → C transitions
 *     and the universal constant δ ≈ 4.6692 governing the cascade.
 *   • Sharkovskii (1964) — explains WHY PRESET_P3_WIND (r=3.835)
 *     forces the existence of every other period: 3 is leftmost in
 *     his ordering 3 ▻ 5 ▻ 7 ▻ … ▻ 2·3 ▻ 2·5 ▻ … ▻ 4 ▻ 2 ▻ 1.
 */
typedef enum {
    /* Class A — fixed-point regime (r ∈ (1, 3)): spirals INTO x* = 1 − 1/r */
    PRESET_FP_SLOW = 0,    /* r=1.500 — slow approach to x* ≈ 0.333          */
    PRESET_FP_MID,         /* r=2.000 — fast approach to x* = 0.500          */
    PRESET_FP_FAST,        /* r=2.500 — fastest approach (slope ≈ 0)         */
    PRESET_FP_SPIRL,       /* r=2.800 — oscillatory spiral to x* ≈ 0.643     */
    PRESET_FP_EDGE,        /* r=2.950 — near loss of stability               */

    /* Class B — period-doubling cascade (Feigenbaum) */
    PRESET_P2_BORN,        /* r=3.050 — 2-cycle just born                    */
    PRESET_P2_MID,         /* r=3.200 — established 2-cycle                  */
    PRESET_P2_WIDE,        /* r=3.400 — wide 2-cycle, near next bifurcation  */
    PRESET_P4_BORN,        /* r=3.460 — 4-cycle                              */
    PRESET_P4_MID,         /* r=3.500 — established 4-cycle                  */
    PRESET_P8_BORN,        /* r=3.550 — 8-cycle                              */
    PRESET_P16_BORN,       /* r=3.566 — 16-cycle (barely distinguishable)    */
    PRESET_FEIGNBM,        /* r=3.5699 — Feigenbaum accumulation point r∞    */

    /* Class C — chaos, interleaved with periodic windows */
    PRESET_CHAOS_A,        /* r=3.575 — narrow chaotic bands                 */
    PRESET_CHAOS_B,        /* r=3.600 — band-merged chaos                    */
    PRESET_P6_WIND,        /* r=3.626 — period-6 window inside chaos         */
    PRESET_CHAOS_C,        /* r=3.650 — typical chaos                        */
    PRESET_P7_WIND,        /* r=3.701 — period-7 window                      */
    PRESET_P5_WIND,        /* r=3.739 — period-5 window (the widest)         */
    PRESET_INTERMTT,       /* r=3.752 — intermittency past the p-5 edge      */
    PRESET_CHAOS_D,        /* r=3.780 — post-p-5 chaos                       */
    PRESET_TANGENT,        /* r=3.828 — tangent bifurcation, just before p-3 */
    PRESET_P3_WIND,        /* r=3.835 — famous period-3 (Li-Yorke 1975)      */
    PRESET_P6_IN_P3,       /* r=3.844 — period-doubled inside the p-3 window */
    PRESET_CRISIS,         /* r=3.857 — interior crisis ending the p-3 window*/
    PRESET_CHAOS_E,        /* r=3.880 — post-p-3 chaos                       */
    PRESET_CHAOS_F,        /* r=3.920 — strong chaos                         */
    PRESET_CHAOS_G,        /* r=3.950 — wide chaos                           */
    PRESET_EDGE_MAX,       /* r=3.990 — near the edge                        */
    PRESET_FULL_MAX,       /* r=4.000 — full ergodic chaos on [0, 1]         */

    N_PRESETS,
} Preset;

/* 30 r-values ordered simple → complex.  Same x0 throughout so the
 * one variable changing across cycle-through-with-n is r. */
static const CobwebPreset presets[N_PRESETS] = {
    /* Class A — fixed-point regime ─────────────────────────────────── */
    { "FP-SLOW ", 1.500f,  0.10f },
    { "FP-MID  ", 2.000f,  0.10f },
    { "FP-FAST ", 2.500f,  0.10f },
    { "FP-SPIRL", 2.800f,  0.10f },
    { "FP-EDGE ", 2.950f,  0.10f },

    /* Class B — period-doubling cascade ────────────────────────────── */
    { "P2-BORN ", 3.050f,  0.10f },
    { "P2-MID  ", 3.200f,  0.10f },
    { "P2-WIDE ", 3.400f,  0.10f },
    { "P4-BORN ", 3.460f,  0.10f },
    { "P4-MID  ", 3.500f,  0.10f },
    { "P8-BORN ", 3.550f,  0.10f },
    { "P16-BORN", 3.566f,  0.10f },
    { "FEIGNBM ", 3.5699f, 0.10f },

    /* Class C — chaos with periodic windows ────────────────────────── */
    { "CHAOS-A ", 3.575f,  0.10f },
    { "CHAOS-B ", 3.600f,  0.10f },
    { "P6-WIND ", 3.626f,  0.10f },
    { "CHAOS-C ", 3.650f,  0.10f },
    { "P7-WIND ", 3.701f,  0.10f },
    { "P5-WIND ", 3.739f,  0.10f },
    { "INTERMTT", 3.752f,  0.10f },
    { "CHAOS-D ", 3.780f,  0.10f },
    { "TANGENT ", 3.828f,  0.10f },
    { "P3-WIND ", 3.835f,  0.10f },
    { "P6-IN-P3", 3.844f,  0.10f },
    { "CRISIS  ", 3.857f,  0.10f },
    { "CHAOS-E ", 3.880f,  0.10f },
    { "CHAOS-F ", 3.920f,  0.10f },
    { "CHAOS-G ", 3.950f,  0.10f },
    { "EDGE-MAX", 3.990f,  0.10f },
    { "FULL-MAX", 4.000f,  0.10f },
};

/*
 * Theme — colour pairs for one named visual style.
 *
 * INTENT
 *   Decouple "which colours" from "where they're used".  Painters in
 *   §9 emit only via the named PAIR_* constants (PAIR_CURVE,
 *   PAIR_DIAG, PAIR_COBWEB_BASE+age, PAIR_LIVE) — theme_apply() in
 *   §3 binds those pairs to the xterm-256 indices stored here.
 *   Cycling themes (t/T) re-binds the ncurses pairs in place; no
 *   §5/§6/§7/§8 code has to know about colour at all.
 *
 * CONTEXT
 *   themes[N_THEMES] below is the static .rodata table.  The active
 *   index lives in PaletteState.current on Scene.  theme_apply()
 *   pushes this row's colours into the corresponding ncurses pairs;
 *   the renderer then reads via COLOR_PAIR(PAIR_*).
 *
 * PALETTE LOGIC
 *   curve : parabola y = r·x(1-x) colour — the static reference shape.
 *   diag  : y = x diagonal colour — the "read off" reflector.
 *   age[] : 4-step cobweb age ramp (oldest → newest).  Monotone-
 *           brightness; all entries ≥ 39 in the xterm-256 cube so
 *           even band 0 stays legible against default-black with
 *           A_BOLD.
 *   live  : '@' marker on current (x, f(x)) — attention accent;
 *           usually a saturated red/yellow against the rest.
 *
 * REFERENCES
 *   • xterm-256 colour cube — indices 16..231 = 6×6×6 RGB cube,
 *     232..255 = 24-step grayscale ramp.
 *   • ncurses(3X) — start_color, init_pair, COLOR_PAIR.
 */
typedef struct {
    const char *name;
    short       curve;
    short       diag;
    short       age[COBWEB_AGE_BANDS];
    short       live;
} Theme;

#define N_THEMES 10
static const Theme themes[N_THEMES] = {
    { "DEFAULT", 220, 244, {  75, 117, 220, 231 }, 196 },
    { "MATRIX",  118, 244, {  77, 118, 156, 194 }, 226 },
    { "NOVA",    207, 244, { 135, 171, 207, 219 }, 226 },
    { "MONO",    250, 240, { 247, 250, 253, 255 }, 226 },
    { "OCEAN",   159, 244, {  81, 117, 159, 195 }, 226 },
    { "FIRE",    220, 244, { 208, 214, 220, 227 }, 231 },
    { "EARTH",   222, 244, { 143, 179, 215, 222 }, 196 },
    { "FOREST",  150, 244, { 114, 150, 157, 194 }, 226 },
    { "DESERT",  222, 244, { 179, 215, 222, 229 }, 196 },
    { "ARCTIC",  195, 244, { 117, 159, 195, 231 }, 196 },
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *t = &themes[idx];
    if (COLORS >= 256) {
        init_pair(PAIR_CURVE, t->curve, -1);
        init_pair(PAIR_DIAG,  t->diag,  -1);
        init_pair(PAIR_LIVE,  t->live,  -1);
        for (int i = 0; i < COBWEB_AGE_BANDS; i++)
            init_pair(PAIR_COBWEB_BASE + i, t->age[i], -1);
    } else {
        init_pair(PAIR_CURVE, COLOR_YELLOW, -1);
        init_pair(PAIR_DIAG,  COLOR_WHITE,  -1);
        init_pair(PAIR_LIVE,  COLOR_RED,    -1);
        for (int i = 0; i < COBWEB_AGE_BANDS; i++)
            init_pair(PAIR_COBWEB_BASE + i, COLOR_CYAN, -1);
    }
}
static void color_init(void)
{
    start_color(); use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  map — LogisticMap (the function) + MapIterator (the orbit)        */
/* ===================================================================== */

/*
 * LogisticMap — the function f(x) = r · x · (1 − x).
 *
 * INTENT
 *   The LOGISTIC MAP itself.  Just a parameter r — the entire
 *   dynamical-systems story (fixed points, period-doubling cascade,
 *   chaos, periodic windows) flows from changing this ONE number.
 *   Kept as a struct (rather than a free float) so the function and
 *   its parameter travel together and any future per-map state
 *   (e.g. caching f'(x), tracking superstable cycle locations) has
 *   a home.
 *
 * CONTEXT
 *   Lives inside MapIterator (below) as a by-value member, since
 *   "the function we're iterating" is conceptually part of "where
 *   we are in iterating it".  Cycling presets via n/p mutates this
 *   r through preset_state_active().  paint_logistic_curve() reads
 *   r at render time to sample the parabola.
 *
 * MEMBER LOGIC
 *   r : bifurcation parameter ∈ [1, 4].
 *       Fixed points: x = 0 (always unstable for r > 1) and
 *       x* = 1 − 1/r (stable for r ∈ (1, 3]).
 *       Stability boundary at r = 3 → first period-doubling.
 *       Chaos onset at r∞ ≈ 3.5699 (the Feigenbaum point).
 *
 * REFERENCES
 *   • May, R. M. (1976) — "Simple mathematical models with very
 *     complicated dynamics", Nature 261.  Established this map as
 *     the canonical 1-D chaotic system.
 *   • Feigenbaum, M. J. (1978) — discovered that the period-doubling
 *     ratio r_{n+1} − r_n  /  r_n − r_{n-1}  → δ ≈ 4.6692 is
 *     UNIVERSAL across unimodal maps, not specific to the logistic
 *     form r·x(1-x).
 *   • Devaney (2003) §1.5 — gives the analytical derivation of the
 *     non-trivial fixed point's stability transition.
 */
typedef struct {
    float r;
} LogisticMap;

/* logistic_map_eval — evaluate f(x) = r · x · (1 − x).  Pure. */
static inline float logistic_map_eval(const LogisticMap *m, float x)
{
    return m->r * x * (1.0f - x);
}

/*
 * MapIterator — the ORBIT of the iterated map.
 *
 * INTENT
 *   The function (LogisticMap) is one thing; WHERE WE ARE in
 *   iterating it is another.  Splitting them out separates "what
 *   the dynamics is" from "what the current state is" — a small
 *   pedagogical win that matches Strogatz §10's and Devaney §1's
 *   vocabulary (the map vs the orbit).  An "orbit" in this
 *   discrete-dynamics sense is the sequence x, f(x), f²(x), … —
 *   exactly what map_iterator_step() generates one element of.
 *
 * CONTEXT
 *   Lives on Scene.  Reset by scene_reset() at boot / r-press / on
 *   preset change.  Advanced by map_iterator_step() inside
 *   scene_tick() once per sim tick.  Read by paint_iterator_point
 *   to draw the live '@' marker.
 *
 * MEMBER LOGIC
 *   map   : the underlying LogisticMap (by value — cheap to copy;
 *           ScalarFn2D-style indirection would obscure the fact
 *           that orbit and map are tightly coupled).
 *   x     : the current iterate ∈ [0, 1].  For r ∈ [1, 4] the
 *           logistic map's invariant interval is [0, 1] — no need
 *           to clamp.
 *   steps : how many times step() has been called since reset
 *           (HUD readout, also useful for "transient cutoff" if a
 *           future feature wants to skip the first N iterations).
 *
 * REFERENCES
 *   • Strogatz §10.1 — defines "orbit" for a discrete map.
 *   • Devaney (2003) §1.2 — same definition + the topological
 *     setup for periodic, eventually-periodic, and aperiodic orbits.
 */
typedef struct {
    LogisticMap map;
    float       x;
    int         steps;
} MapIterator;

static void map_iterator_init(MapIterator *it, float r, float x0)
{
    it->map.r = r;
    it->x     = x0;
    it->steps = 0;
}

/* map_iterator_step — advance one iteration.  Returns x_new = f(x). */
static float map_iterator_step(MapIterator *it)
{
    float x_new = logistic_map_eval(&it->map, it->x);
    it->x = x_new;
    it->steps++;
    return x_new;
}

/* ===================================================================== */
/* §6  cobweb — CobwebRing of (x0, y0, x1, y1, kind) quartets             */
/* ===================================================================== */

/*
 * CobwebSegKind — which of the two cobweb-step segments this is.
 *
 *   SEG_VERT : the "lift" stroke — (x_n, x_n) → (x_n, f(x_n)).
 *              Climbs from the diagonal up to the parabola.
 *   SEG_HORZ : the "reflect" stroke — (x_n, f(x_n)) → (f(x_n), f(x_n)).
 *              Returns from the parabola to the diagonal at the new x.
 *
 * Stored explicitly (rather than inferred from coords) because the
 * painter picks '|' for verticals and '-' for horizontals; explicit
 * kind avoids float-equality comparisons at render time.
 */
typedef enum { SEG_VERT = 0, SEG_HORZ } CobwebSegKind;

/*
 * CobwebSeg — one segment in the cobweb staircase.
 *
 * INTENT
 *   The cobweb construction is built from line segments alternating
 *   vertical / horizontal.  We store them as (x0, y0)→(x1, y1) pairs
 *   in unit coordinates ∈ [0, 1]², plus a `kind` tag so the painter
 *   picks the right glyph.  Unit coords mean the data is renderer-
 *   independent (works the same regardless of viewport size).
 *
 * MEMBER LOGIC
 *   x0, y0 : start point in unit square coordinates.
 *   x1, y1 : end point in unit square coordinates.
 *   kind   : SEG_VERT or SEG_HORZ — sets the painter's glyph choice.
 */
typedef struct {
    float          x0, y0, x1, y1;
    CobwebSegKind  kind;
} CobwebSeg;

/*
 * CobwebRing — bounded ring buffer of CobwebSeg quartets.
 *
 * INTENT
 *   Each iteration adds TWO segments (one vertical, one horizontal).
 *   We retain the most recent COBWEB_MAX segments so the cobweb
 *   visibly fades oldest → newest in COBWEB_AGE_BANDS colour tiers.
 *   Bounded capacity → no allocation, predictable BSS footprint,
 *   no malloc/free in the hot path.
 *
 * CONTEXT
 *   Lives on Scene.  Written by scene_tick (one push per direction
 *   change in the cobweb staircase).  Read by paint_cobweb in §9 to
 *   render the trail; ages are mapped through cobweb_ring_age_band
 *   to pick a colour pair.
 *
 * MEMORY
 *   COBWEB_MAX × sizeof(CobwebSeg) ≈ 1024 × 20 ≈ 20 KB in BSS.
 *
 * MEMBER LOGIC
 *   seg   : segment storage.  Oldest valid sample lives at
 *           (head - count + 1) mod COBWEB_MAX; newest at head.
 *   head  : index of MOST RECENT segment (advances modulo MAX).
 *   count : ≤ COBWEB_MAX — number of valid segments since reset.
 *           Once count saturates at COBWEB_MAX, push overwrites
 *           the oldest entry (classic ring-buffer semantics).
 *
 * REFERENCES
 *   • Ring (circular) buffer — Knuth, "The Art of Computer
 *     Programming" Vol 1 §2.2.2.  Fixed-capacity FIFO with modular
 *     head index; the canonical lock-free single-producer pattern
 *     when you don't need a true queue.
 */
typedef struct {
    CobwebSeg seg[COBWEB_MAX];
    int       head;
    int       count;
} CobwebRing;

static void cobweb_ring_reset(CobwebRing *c) { c->head = 0; c->count = 0; }

static void cobweb_ring_push(CobwebRing *c, float x0, float y0,
                             float x1, float y1, CobwebSegKind k)
{
    c->head = (c->head + 1) % COBWEB_MAX;
    c->seg[c->head] = (CobwebSeg){ x0, y0, x1, y1, k };
    if (c->count < COBWEB_MAX) c->count++;
}

/* cobweb_ring_age_band — return band ∈ [0, COBWEB_AGE_BANDS) for the
 * segment at age `age` out of `n` total (0 = newest). */
static inline int cobweb_ring_age_band(int age, int n)
{
    int band = (COBWEB_AGE_BANDS - 1) - (age * COBWEB_AGE_BANDS) / n;
    if (band < 0)                        band = 0;
    if (band > COBWEB_AGE_BANDS - 1)     band = COBWEB_AGE_BANDS - 1;
    return band;
}

/* ===================================================================== */
/* §7  viewport — UnitViewport: unit square [0,1]² ↔ cell grid           */
/* ===================================================================== */

/*
 * UnitViewport — affine map between the abstract unit square the
 * dynamics live in and the concrete cell rectangle on screen.
 *
 * INTENT
 *   The dynamics produces values in [0, 1] × [0, 1] (the logistic
 *   map's natural domain).  ncurses draws to integer cells.  The
 *   viewport encapsulates the one-time scale/origin computation so
 *   no painter has to remember it.  y is FLIPPED so that x = 0 lives
 *   at the BOTTOM of the visible region — the standard maths
 *   convention students expect from cobweb plots in any textbook.
 *
 * CONTEXT
 *   Recomputed once per frame inside paint_scene() from the current
 *   Screen dims (cheap: 4 int assignments).  Read-only for every
 *   painter; passed by const pointer.
 *
 * MEMBER LOGIC
 *   gx0, gy0 : top-left cell of the drawable interior — the origin
 *              of the viewport in screen-cell coordinates.
 *   w, h     : interior width / height in cells.  Excludes the
 *              HUD_BAND_RESERVED_ROWS reservation so painters never
 *              overwrite the dashboard.
 *
 * REFERENCES
 *   • Affine transform / coordinate frames — Foley, van Dam et al.,
 *     "Computer Graphics: Principles and Practice" §5.  The
 *     world→screen mapping is the simplest possible affine: a
 *     uniform scale + translate, no rotation, no shear.
 */
typedef struct {
    int gx0, gy0;
    int w, h;
} UnitViewport;

static void viewport_compute(UnitViewport *v, int cols, int rows)
{
    v->gx0 = 0;
    v->gy0 = HUD_TOP_ROWS;
    v->w   = cols;
    v->h   = rows - HUD_BAND_RESERVED_ROWS;
}

/* viewport_u_to_cx — unit x ∈ [0, 1] → cell column. */
static inline int viewport_u_to_cx(const UnitViewport *v, float u)
{ return v->gx0 + (int)(u * (float)(v->w - 1)); }

/* viewport_u_to_cy — unit y ∈ [0, 1] → cell row.  y FLIPPED so 0 = bottom. */
static inline int viewport_u_to_cy(const UnitViewport *v, float u)
{ return v->gy0 + (v->h - 1) - (int)(u * (float)(v->h - 1)); }

/* ===================================================================== */
/* §8  scene — PresetState + PaletteState + Scene composite               */
/* ===================================================================== */

/*
 * PresetState — index into the 30-preset table + named mutators.
 *
 * INTENT
 *   A one-int wrapper is overkill data-wise but valuable structurally:
 *   gives "which r value is showing" a typed home on Scene, matches
 *   the sub-struct convention used by sibling showcases, and makes
 *   the n/p key handlers read as
 *      `preset_state_cycle_next(&s->preset); scene_load_preset(s);`
 *   instead of inline `s->preset_idx = (s->preset_idx + 1) % N_PRESETS;`
 *
 * CONTEXT
 *   Lives on Scene (§8).  Mutated by app_handle_key on n/N/p/P;
 *   read by scene_load_preset (applies r, x0 to the MapIterator)
 *   and by hud_draw_param_row (HUD readout).
 *
 * MEMBER LOGIC
 *   current : index into presets[] ∈ [0, N_PRESETS).  Anything
 *             outside is invalid; cycle_next/prev preserve the
 *             invariant via modular arithmetic.
 *
 * REFERENCES
 *   • Modular cycling — the (i ± 1 + N) % N idiom is the standard
 *     way to do branchless circular index walks without underflow.
 */
typedef struct { int current; } PresetState;

static void preset_state_init(PresetState *p)        { p->current = PRESET_FP_SPIRL; }
static void preset_state_cycle_next(PresetState *p)  { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)  { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const CobwebPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }

/*
 * PaletteState — index of the currently-active Theme.
 *
 * INTENT
 *   Same shape as PresetState — a typed one-int wrapper that makes
 *   the t/T key handler read as `palette_state_cycle_next(&p);
 *   theme_apply(p.current);` rather than touching free integers.
 *
 * CONTEXT
 *   Lives on Scene (§8).  Mutated by app_handle_key on t/T (with
 *   modular wraparound through N_THEMES).  Whenever it changes,
 *   theme_apply() (§3) MUST be called to push the new Theme's
 *   colour indices into the ncurses colour pairs — this struct
 *   holds the index, ncurses holds the actual pair bindings.
 *   Read by hud_draw_param_row for the HUD label.
 *
 * MEMBER LOGIC
 *   current : index into themes[] ∈ [0, N_THEMES).  Anything
 *             outside that range is invalid; theme_apply() clamps
 *             to 0 defensively.
 */
typedef struct { int current; } PaletteState;

static void palette_state_init(PaletteState *p)       { p->current = 0; }
static void palette_state_cycle_next(PaletteState *p) { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p) { p->current = (p->current + N_THEMES - 1) % N_THEMES; }

/*
 * Scene — composite owner of ALL mutable state.
 *
 * THE PIPELINE  (left → right reflects the dataflow per tick)
 *
 *   preset   — which r value is selected               (the CONTROL)
 *      ↓ (preset_state_active gives r, x0)
 *   iter     — LogisticMap + current x + step count    (the DYNAMICS)
 *      ↓ (map_iterator_step advances x; we record the two new segments)
 *   cobweb   — ring of (x0,y0)→(x1,y1) staircase segs   (the OUTPUT)
 *      ↓ (read by paint_cobweb in §9)
 *   palette  — active theme index                       (the COLOUR)
 *      + paused                                         (the GATE)
 *
 * CONTEXT
 *   One instance, owned by App in §10.  Touched only from the main
 *   thread (the signal handlers do NOT reach into Scene; they flip
 *   volatile flags on App and let the main loop respond at the next
 *   frame boundary).
 *
 * LIFETIME
 *   scene_init        : zero-init, defaults, scene_load_preset.
 *   scene_load_preset : apply current preset's r, x0 to iter; clear ring.
 *   scene_reset       : alias for scene_load_preset.
 *   scene_tick        : advance iter ITER_STEPS_PER_TICK times, push
 *                       the two new cobweb segments per iteration.
 *   No teardown — Scene lives in BSS via App; OS reclaims at exit.
 *
 * MEMBER LOGIC
 *   iter    : MapIterator   — the orbit being drawn.
 *   cobweb  : CobwebRing    — fading segments behind the orbit.
 *   preset  : PresetState   — which r value is active.
 *   palette : PaletteState  — which theme is active.
 *   paused  : when true, scene_tick early-returns; the renderer keeps
 *             drawing the last computed cobweb → user sees a still frame.
 *             Toggled by space.
 */
typedef struct {
    MapIterator  iter;
    CobwebRing   cobweb;
    PresetState  preset;
    PaletteState palette;
    bool         paused;
} Scene;

static void scene_load_preset(Scene *s)
{
    const CobwebPreset *p = preset_state_active(&s->preset);
    map_iterator_init(&s->iter, p->r, p->x0);
    cobweb_ring_reset(&s->cobweb);
}

static void scene_reset(Scene *s) { scene_load_preset(s); }

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    preset_state_init(&s->preset);
    palette_state_init(&s->palette);
    s->paused = false;
    scene_load_preset(s);
}

static void scene_tick(Scene *s, float dt)
{
    (void)dt;
    if (s->paused) return;
    for (int k = 0; k < ITER_STEPS_PER_TICK; k++) {
        float x_old = s->iter.x;
        float x_new = map_iterator_step(&s->iter);
        /* Vertical lift: (x_old, x_old) → (x_old, f(x_old) = x_new) */
        cobweb_ring_push(&s->cobweb, x_old, x_old, x_old, x_new, SEG_VERT);
        /* Horizontal reflect: (x_old, x_new) → (x_new, x_new) */
        cobweb_ring_push(&s->cobweb, x_old, x_new, x_new, x_new, SEG_HORZ);
    }
}

/* ===================================================================== */
/* §9  screen — Screen + painters + HUD                                  */
/* ===================================================================== */

/*
 * Screen — ncurses viewport dimensions cache (just cols × rows).
 *
 * INTENT
 *   ncurses owns the terminal state.  We just need to know "how wide
 *   / tall is the drawable area right now" so the painters can
 *   compute the UnitViewport without re-querying ncurses on every
 *   paint.  Updated lazily on SIGWINCH.
 *
 * CONTEXT
 *   One instance lives on App (§10).  Refreshed by screen_init() at
 *   startup and screen_resize() on SIGWINCH.  Read by every render
 *   fn that needs to know where edges or centres are.
 *
 * MEMBER LOGIC
 *   cols : Viewport width in character cells.  Source of truth is
 *          getmaxyx(stdscr, ..., cols) inside screen_init /
 *          screen_resize.
 *   rows : Viewport height in character cells.  Includes the HUD
 *          band — render fns subtract HUD_BAND_RESERVED_ROWS as
 *          needed to get the drawable interior.
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s)   { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh();
                                       getmaxyx(stdscr, s->rows, s->cols); }
static void screen_present(void)     { wnoutrefresh(stdscr); doupdate(); }

/* ---------- painters ------------------------------------------------- */

/* in_drawable — clip predicate so painters can never overwrite the HUD. */
static inline bool in_drawable(int sx, int sy, int cols, int rows)
{
    return sx >= 0 && sx < cols && sy >= HUD_TOP_ROWS && sy < rows - HUD_BOTTOM_ROWS;
}

/* paint_logistic_curve — sample f(x) = r·x(1-x) at every column and
 * paint a '.' at the corresponding row.  Static reference geometry
 * the cobweb climbs against. */
static void paint_logistic_curve(const UnitViewport *v, const LogisticMap *m,
                                 int cols, int rows)
{
    attron(COLOR_PAIR(PAIR_CURVE) | A_BOLD);
    for (int x = 0; x < v->w; x++) {
        float u = (float)x / (float)(v->w - 1);
        float y = logistic_map_eval(m, u);
        int sx = v->gx0 + x;
        int sy = viewport_u_to_cy(v, y);
        if (in_drawable(sx, sy, cols, rows)) mvaddch(sy, sx, '.');
    }
    attroff(COLOR_PAIR(PAIR_CURVE) | A_BOLD);
}

/* paint_diagonal — draw y = x as '/' across the unit square. */
static void paint_diagonal(const UnitViewport *v, int cols, int rows)
{
    attron(COLOR_PAIR(PAIR_DIAG));
    for (int x = 0; x < v->w; x++) {
        float u  = (float)x / (float)(v->w - 1);
        int   sx = v->gx0 + x;
        int   sy = viewport_u_to_cy(v, u);
        if (in_drawable(sx, sy, cols, rows)) mvaddch(sy, sx, '/');
    }
    attroff(COLOR_PAIR(PAIR_DIAG));
}

/* paint_cobweb_segment — render one segment, choosing '|' for vertical
 * and '-' for horizontal, all in the given band's colour pair. */
static void paint_cobweb_segment(const CobwebSeg *s, int band,
                                 const UnitViewport *v, int cols, int rows)
{
    int pair = PAIR_COBWEB_BASE + band;
    attron(COLOR_PAIR(pair) | A_BOLD);
    if (s->kind == SEG_VERT) {
        int sx = viewport_u_to_cx(v, s->x0);
        int ya = viewport_u_to_cy(v, s->y0);
        int yb = viewport_u_to_cy(v, s->y1);
        int lo = ya < yb ? ya : yb;
        int hi = ya < yb ? yb : ya;
        for (int y = lo; y <= hi; y++)
            if (in_drawable(sx, y, cols, rows)) mvaddch(y, sx, '|');
    } else {
        int sy = viewport_u_to_cy(v, s->y0);
        int xa = viewport_u_to_cx(v, s->x0);
        int xb = viewport_u_to_cx(v, s->x1);
        int lo = xa < xb ? xa : xb;
        int hi = xa < xb ? xb : xa;
        for (int x = lo; x <= hi; x++)
            if (in_drawable(x, sy, cols, rows)) mvaddch(sy, x, '-');
    }
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* paint_cobweb — walk the ring oldest → newest, paint each segment in
 * the band corresponding to its age (0 = oldest = dimmest). */
static void paint_cobweb(const CobwebRing *c, const UnitViewport *v,
                         int cols, int rows)
{
    if (c->count == 0) return;
    int n      = c->count;
    int oldest = (c->head - n + 1 + COBWEB_MAX) % COBWEB_MAX;
    for (int i = 0; i < n; i++) {
        int idx  = (oldest + i) % COBWEB_MAX;
        int age  = n - 1 - i;
        int band = cobweb_ring_age_band(age, n);
        paint_cobweb_segment(&c->seg[idx], band, v, cols, rows);
    }
}

/* paint_iterator_point — '@' marker at the CURRENT (x, f(x)) so the
 * eye finds where the staircase is going next. */
static void paint_iterator_point(const MapIterator *it, const UnitViewport *v,
                                 int cols, int rows)
{
    float fx = logistic_map_eval(&it->map, it->x);
    int sx = viewport_u_to_cx(v, it->x);
    int sy = viewport_u_to_cy(v, fx);
    if (!in_drawable(sx, sy, cols, rows)) return;
    attron(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
    mvaddch(sy, sx, '@');
    attroff(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
}

/* paint_scene — the full FIELD draw: curve + diagonal + cobweb + live. */
static void paint_scene(const Scene *s, int cols, int rows)
{
    UnitViewport v; viewport_compute(&v, cols, rows);
    paint_logistic_curve(&v, &s->iter.map, cols, rows);
    paint_diagonal(&v, cols, rows);
    paint_cobweb(&s->cobweb, &v, cols, rows);
    paint_iterator_point(&s->iter, &v, cols, rows);
}

/* ---------- HUD helpers --------------------------------------------- */

static void hud_draw_top_left_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " COBWEB DIAGRAM (logistic) ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_draw_top_right_status(int cols, double fps, int sim_fps,
                                      const Scene *s)
{
    const CobwebPreset *p = preset_state_active(&s->preset);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  r:%.4f  step:%d ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : p->name,
             s->preset.current + 1, N_PRESETS,
             (double)p->r, s->iter.steps);
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static int hud_field_bold_label(int x, const char *fmt, const char *val, int width)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, fmt, val);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + width;
}

static void hud_draw_param_row(const Scene *s)
{
    const CobwebPreset *p = preset_state_active(&s->preset);
    int x = HUD_LEFT_MARGIN;
    x = hud_field_bold_label(x, " preset:%-8s ", p->name,                    19);
    x = hud_field_bold_label(x, " theme:%-8s ",  themes[s->palette.current].name, 17);
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " x:%.4f  f(x):%.4f  segs:%d/%d ",
             (double)s->iter.x,
             (double)logistic_map_eval(&s->iter.map, s->iter.x),
             s->cobweb.count, COBWEB_MAX);
    attroff(COLOR_PAIR(PAIR_HUD));
}

static void hud_draw_bottom_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:preset  t/T:theme  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* screen_draw — full-frame composer.
 *
 *   STEP 1 — CLEAR the back buffer
 *   STEP 2 — PAINT the logistic field (curve + diagonal + cobweb + live)
 *   STEP 3 — OVERLAY the top HUD row (title + right-aligned status)
 *   STEP 4 — OVERLAY the row-1 parameter dashboard
 *   STEP 5 — OVERLAY the bottom key-hint bar
 */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    paint_scene(s, sc->cols, sc->rows);
    hud_draw_top_left_title();
    hud_draw_top_right_status(sc->cols, fps, sim_fps, s);
    hud_draw_param_row(s);
    hud_draw_bottom_hint(sc->rows);
}

/* ===================================================================== */
/* §10 app                                                                */
/* ===================================================================== */

/*
 * App — top-level program state; the BSS root.
 *
 * INTENT
 *   Single owner for Scene, Screen, the sim-rate target, and the two
 *   volatile signal flags.  Lives in BSS as g_app (file-scope) for
 *   one specific reason: POSIX signal handlers can ONLY safely touch
 *   global async-signal-safe state, so the two sig_atomic_t flags
 *   must be reachable without arguments.  Everything else on App is
 *   touched from the main thread only.
 *
 * CONTEXT
 *   Exactly one instance: g_app at file scope.  Built in main(),
 *   driven by the main game loop, torn down by atexit(cleanup).
 *   on_exit_signal / on_resize_signal flip the flags; the main loop
 *   polls them at frame boundaries and reacts (clean exit / lazy
 *   resize) — keeping all the heavy work off the signal handler.
 *
 * MEMBER LOGIC
 *   scene       : all mutable simulation state (§8).  See Scene doc
 *                 above for the dataflow pipeline.
 *   screen      : viewport dimensions cache (§9).
 *   sim_fps     : target simulation Hz ∈ [SIM_FPS_MIN, SIM_FPS_MAX].
 *                 Default 8 Hz so the cobweb's per-iteration jumps
 *                 are visible to the eye; stepped by ]/[ keys.
 *                 Independent of the render FPS (capped at 60).
 *   running     : 0 = exit the main loop next iteration.  Cleared
 *                 by the SIGINT/SIGTERM handlers and by q/ESC in
 *                 app_handle_key().  sig_atomic_t for handler safety.
 *   need_resize : Set by the SIGWINCH handler; main loop polls it
 *                 before the next frame and runs app_do_resize().
 *                 sig_atomic_t for handler safety.
 *
 * REFERENCES
 *   • Fix Your Timestep! — Glenn Fiedler:
 *     https://gafferongames.com/post/fix_your_timestep/
 *     The fixed-timestep accumulator pattern main() uses.
 *   • POSIX.1-2017 § 2.4.3 — async-signal-safe functions and signal
 *     handler discipline.  Why running/need_resize are sig_atomic_t.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* main_install_signal_handlers — clean exit on SIGINT/SIGTERM, lazy
 * resize on SIGWINCH.  POSIX async-signal-safe path: handlers touch
 * only the sig_atomic_t flags on g_app. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

static void app_bootstrap(App *app)
{
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    scene_init(&app->scene);
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_reset(&app->scene);
    app->need_resize = 0;
}

static void app_handle_pending_resize(App *app, int64_t *frame_time, int64_t *sim_accum)
{
    if (!app->need_resize) return;
    app_do_resize(app);
    *frame_time = clock_ns();
    *sim_accum  = 0;
}

static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    int64_t dt_cap = (int64_t)SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    if (dt > dt_cap) dt = dt_cap;
    return dt;
}

static void app_drain_fixed_timestep(App *app, int64_t dt, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    *sim_accum += dt;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

static void app_update_fps_meter(int64_t dt, int *frame_count,
                                 int64_t *fps_accum, double *fps_display)
{
    (*frame_count)++;
    *fps_accum += dt;
    if (*fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
        *fps_display = (double)(*frame_count)
                     / ((double)(*fps_accum) / (double)NS_PER_SEC);
        *frame_count = 0; *fps_accum = 0;
    }
}

static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* Tiny sim-rate mutators so app_handle_key reads as a binding table. */
static void app_sim_rate_faster(App *app)
{
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
}
static void app_sim_rate_slower(App *app)
{
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

/* app_handle_key — keyboard dispatch.  Each case forwards to a tiny
 * named mutator on the appropriate Scene sub-state.  Returns false
 * on exit request (q/Q/ESC); true otherwise. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */:
        return false;

    case ' ':
        s->paused = !s->paused;
        break;
    case 'r': case 'R':
        scene_reset(s);
        break;

    case ']':            app_sim_rate_faster(app); break;
    case '[':            app_sim_rate_slower(app); break;

    case 't':
        palette_state_cycle_next(&s->palette);
        theme_apply(s->palette.current);
        break;
    case 'T':
        palette_state_cycle_prev(&s->palette);
        theme_apply(s->palette.current);
        break;

    case 'n': case 'N':
        preset_state_cycle_next(&s->preset);
        scene_load_preset(s);
        break;
    case 'p': case 'P':
        preset_state_cycle_prev(&s->preset);
        scene_load_preset(s);
        break;

    default: break;
    }
    return true;
}

static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* main — game-loop driver.
 *
 *   STEP 1 — SEED rng + INSTALL signal handlers + BOOTSTRAP app
 *   STEP 2 — LOOP:
 *              a. honour any pending resize
 *              b. compute dt (clamped)
 *              c. drain fixed-timestep sim ticks
 *              d. update fps meter
 *              e. throttle to render budget
 *              f. present the frame
 *              g. poll input → maybe exit
 *   STEP 3 — TEARDOWN screen
 */
int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    main_install_signal_handlers();
    App *app = &g_app;
    app_bootstrap(app);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {
        app_handle_pending_resize    (app, &frame_time, &sim_accum);
        int64_t dt = app_compute_frame_dt(&frame_time);
        app_drain_fixed_timestep     (app, dt, &sim_accum);
        app_update_fps_meter         (dt, &frame_count, &fps_accum, &fps_display);
        app_throttle_to_render_target(frame_time, dt);
        app_present_frame            (app, fps_display);
        if (!app_poll_keyboard(app)) app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
