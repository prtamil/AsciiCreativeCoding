/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lissajous.c — Harmonograph / Lissajous Figures
 *
 * Two perpendicular damped oscillators:
 *   x(t) = sin(fx·t + phase)·exp(−decay·t)
 *   y(t) = sin(fy·t)       ·exp(−decay·t)
 *
 * The phase offset drifts slowly, morphing the Lissajous figure
 * through figure-8s, stars, petals, and spirals.  Drift slows near
 * symmetric phases so each named shape dwells on screen before the
 * figure transitions.  After a full 2π phase sweep the program
 * auto-advances to the next rational frequency ratio.
 *
 * Ratios:  1:2 Figure-8 · 2:3 Trefoil · 3:4 Star   · 1:3 Clover
 *          3:5 Pentagram · 2:5 Five-petal · 4:5 Crown · 1:4 Eye
 *
 * Each ratio's T_MAX scales to N_LOOPS=4 cycles of the slower
 * oscillator, and DECAY adjusts so amplitude reaches ~1% at T_MAX —
 * giving an identical spiral depth regardless of frequency ratio.
 *
 * Rendering: draw the full decaying spiral each frame (oldest innermost
 * first, newest outermost last).  Four brightness levels map to the four
 * decay loops; the outermost loop is always brightest.
 *
 * Keys:
 *   space      pause / resume
 *   n / p      next / prev ratio (resets phase to 0)
 *   + / =      faster phase drift
 *   - / _      slower phase drift
 *   c          cycle color themes (Golden / Ice / Ember / Neon)
 *   q / Q      quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra geometry/lissajous.c -o lissajous -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 sim  §5 scene  §6 screen  §7 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Parametric curve tracing with decaying amplitude.
 *                  The Lissajous figure is traced analytically (no ODE) by
 *                  evaluating the closed-form parametric equations at each t.
 *                  A decaying exponential envelope ensures the spiral converges
 *                  to the centre, giving finite total length.
 *
 * Math           : Lissajous figures x(t) = A sin(fx·t + φ), y(t) = B sin(fy·t).
 *                  Ratio fx:fy rational → closed curve (repeats exactly).
 *                  The figure has fx·fy/gcd(fx,fy) lobes and is symmetric about
 *                  both axes when φ=0.  Phase offset φ = π/2 → rotated figure.
 *                  With damping: x = sin(fx·t + φ) · e^(-λt)
 *                  The decay λ is set so amplitude ≈ 1% at T_MAX:
 *                    λ = ln(100) / T_MAX ≈ 4.6 / T_MAX
 *
 * Rendering      : The full curve is redrawn each frame (not accumulated).
 *                  Four brightness levels map to the four decay loops:
 *                  newest (largest amplitude) = brightest, innermost = dimmest.
 *                  Phase drifts slowly to cycle through all Lissajous shapes
 *                  for the current frequency ratio.
 * ─────────────────────────────────────────────────────────────────────── */

/* -- REFERENCES ---------------------------------------------------------  *
 *
 * History & original mathematics:
 *   [1] Lissajous, J. A. (1857). "Mémoire sur l'étude optique des mouvements
 *       vibratoires." Annales de Chimie et de Physique, 51, 147-231.
 *       -- the namesake paper: figures traced by two perpendicular tuning
 *          forks observed through a mirror.  The shape catalogue that gives
 *          k_ratios[] its names (figure-8, trefoil, star, ...) traces back
 *          to this work.
 *
 *   [2] Bowditch, N. (1815). "On the motion of a pendulum suspended from
 *       two points." Memoirs of the American Academy of Arts & Sciences,
 *       3(2), 413-436.
 *       -- predates Lissajous by ~40 years; the curves are also called
 *          "Bowditch curves" in some texts.  Different derivation (compound
 *          pendulum), same family of (x(t), y(t)) = (sin fx t, sin fy t).
 *
 *   [3] Lawrence, J. D. (1972). "A Catalog of Special Plane Curves."
 *       Dover Publications.  Reprint of the 1955 edition.
 *       -- chapter on Lissajous curves: closed-curve criterion, lobe
 *          count, symmetries by phase.  The reference catalogue.
 *
 * Physics of damped oscillators (the decay envelope):
 *   [4] French, A. P. (1971). "Vibrations and Waves." MIT Introductory
 *       Physics Series, W. W. Norton.
 *       -- the canonical undergrad treatment of damped harmonic motion;
 *          source of the e^(-λt) envelope evaluated by §5 envelope_amp().
 *
 *   [5] Whitaker, R. J. (2001). "Harmonographs. I. Pendulum design" and
 *       "II. Circular design." American Journal of Physics 69(2),
 *       162-167 & 174-183.
 *       -- the harmonograph is the 19th-century mechanical instrument
 *          that DRAWS damped Lissajous spirals -- exactly what this file
 *          simulates.  These two papers derive the equations and the
 *          aesthetic effect of detuning fx:fy slightly off-rational.
 *
 * Dynamical systems context:
 *   [6] Strogatz, S. H. (2015). "Nonlinear Dynamics and Chaos: With
 *       Applications to Physics, Biology, Chemistry, and Engineering"
 *       (2nd ed.). Westview Press.
 *       -- chapters on coupled oscillators explain WHY rational fx:fy
 *          gives closed curves and irrational fx:fy gives space-filling
 *          ergodic orbits.  Same machinery, broader frame.
 *
 * Rendering:
 *   [7] Foley, van Dam, Feiner, Hughes (1995). "Computer Graphics:
 *       Principles and Practice" (2nd ed. in C), chapter 11.
 *       Addison-Wesley.
 *       -- parametric-curve discretization, uniform vs. adaptive
 *          t-sampling.  The N_CURVE_PTS uniform sampling we use is the
 *          textbook trade-off (simple, good enough at this scale; for
 *          high-curvature regions adaptive sampling would win).
 *
 * ----------------------------------------------------------------------- */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define TICK_NS       33333333LL   /* ~30 fps                              */
#define N_CURVE_PTS   2500         /* parametric samples per frame          */
#define N_LOOPS       4            /* complete cycles of slower oscillator  */
#define DECAY_TOTAL   4.5f         /* total exponent → amp ≈ 1% at T_MAX   */
#define CELL_ASPECT   2.0f         /* physical char height / width ratio    */
#define AMP_FRAC      0.92f        /* fraction of screen half-size used     */

/* Color pairs: N_THEMES × N_LEVELS + HUD + HINT */
#define N_LEVELS      4            /* one brightness level per decay loop   */
#define N_THEMES      4
#define CP_IDX(t,l)   ((t)*N_LEVELS + (l) + 1)
#define CP_HUD        (N_THEMES * N_LEVELS + 1)   /* data row (top)         */
#define CP_HINT       (N_THEMES * N_LEVELS + 2)   /* action row (bottom)    */

/* HUD layout — one row reserved at top for data, one at bottom for keys.
 * The drawing area is rows [HUD_TOP_ROWS, rows - HUD_BOTTOM_ROWS). */
#define HUD_TOP_ROWS     1
#define HUD_BOTTOM_ROWS  1

/* Math literals named for their geometric meaning. */
#define TAU              (2.0f * (float)M_PI)   /* one full turn, radians */

/* Rendering thresholds and tunings. */
#define AMP_VISIBLE_MIN  0.01f   /* skip samples whose envelope amplitude
                                    is below this -- they are below the
                                    "1% of full" target (DECAY_TOTAL was
                                    calibrated to bottom out exactly here). */
#define BOLD_LEVELS      2       /* brightness tiers [0, BOLD_LEVELS) get
                                    A_BOLD; the two dimmer tiers do not.    */
#define EDGE_MARGIN_X    2       /* total cells reserved horizontally
                                    (1 on each side) so the widest point
                                    of the figure never lands on column 0
                                    or column cols-1.                       */

/* Phase drift (rad/tick; 2π ÷ (DRIFT × 30fps) = seconds per ratio) */
#define DRIFT_DEFAULT 0.004f       /* 2π in ~1047 ticks ≈ 35 s             */
#define DRIFT_MIN     0.0005f
#define DRIFT_MAX     0.08f
#define DRIFT_STEP    1.6f

/*
 * Drift slow-down near symmetric phases.
 *
 * Tuning knobs for the V-shaped dwell envelope built in §4 by
 * ratio_key_period + dwell_envelope.  DWELL_WIDTH sets the V's width
 * (as a fraction of the key-phase period), DWELL_SPEED sets its floor
 * (the speed multiplier at the bottom of the V).  See dwell_envelope
 * for the full formula.
 */
#define DWELL_WIDTH   0.25f        /* width of the dwell well, fraction of kp */
#define DWELL_SPEED   0.25f        /* speed multiplier at the well floor      */

#define N_RATIOS      8

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Theme -- an N_LEVELS-stop color ramp paired with its display name.
 *
 * WHY this exists:
 *   The spiral has N_LOOPS = 4 visible decay loops; each loop occupies
 *   a distinct slice of t and gets its own brightness tier so the eye
 *   reads the loops as a stack from "freshest" (outer, brightest) to
 *   "stalest" (inner, dimmest).  Two orthogonal visual channels carry
 *   the two signals: COLOR encodes the theme; BRIGHTNESS encodes the
 *   age tier.  Each (theme, tier) pair therefore needs its own ANSI
 *   color code -- the colors[] array.
 *
 * WHY one struct (not two parallel arrays as the first draft had):
 *   colors[] and name were always indexed by the same theme_idx;
 *   pairing them as a record lets the type system enforce "no orphaned
 *   name".  k_themes[i].name is the HUD label; k_themes[i].colors[l]
 *   is the 256-color ANSI code for brightness tier l.
 *
 * WHY N_LEVELS = 4 (and not, say, 8):
 *   It must equal N_LOOPS so each decay loop maps to exactly one
 *   brightness tier.  More tiers would split a single loop across
 *   colors (visually confusing); fewer would merge loops (the depth
 *   stack would collapse).
 *
 * Brightness convention (every theme obeys it):
 *   l = 0                outermost loop -- newest, full amp, brightest hue
 *   l = N_LEVELS - 1     innermost loop -- oldest, tiny amp, dimmest hue
 *   Same ramp shape across all themes; only the hue FAMILY differs
 *   (Golden / Ice / Ember / Neon).  Cycling themes with the c key
 *   therefore preserves the brightness gradient -- the eye reads the
 *   same depth signal regardless of palette.
 *
 * Reference: Foley/van Dam [7] §13 on perceptual brightness ramps;
 *   we sit at the trivial end of that machinery (discrete 4-stop, no
 *   gamma correction, no LUT interpolation).
 */
typedef struct {
    const char *name;             /* HUD label ("Golden", "Ice", ...)       */
    int         colors[N_LEVELS]; /* 256-color ANSI codes; brightest first  */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* Golden: bright-yellow → gold → dark-orange → brown    */
    { "Golden", { 226, 220, 136,  94 } },
    /* Ice:    bright-cyan  → teal → dark-teal  → navy       */
    { "Ice",    {  51,  38,  23,  17 } },
    /* Ember:  white → orange → red → dark-red               */
    { "Ember",  { 231, 208, 196,  88 } },
    /* Neon:   bright-green → green → dark-green → very-dark */
    { "Neon",   { 118,  82,  28,  22 } },
};

/* Glyph for each brightness level, brightest first.
 * Orthogonal to Theme -- the same four characters render across every
 * hue family, so character shape carries the brightness signal and
 * color carries the theme signal. */
static const char k_lev_ch[N_LEVELS] = { '#', '*', '+', '.' };

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        for (int t = 0; t < N_THEMES; t++)
            for (int l = 0; l < N_LEVELS; l++)
                init_pair(CP_IDX(t, l), k_themes[t].colors[l], -1);
        init_pair(CP_HUD,  252, -1);   /* bright gray for data         */
        init_pair(CP_HINT,  51, -1);   /* bright cyan for action keys  */
    } else {
        for (int i = 1; i <= N_THEMES * N_LEVELS; i++)
            init_pair(i, COLOR_YELLOW, -1);
        init_pair(CP_HUD,  COLOR_WHITE, -1);
        init_pair(CP_HINT, COLOR_CYAN,  -1);
    }
}

/* ===================================================================== */
/* §4  sim                                                                */
/* ===================================================================== */

/*
 * Ratio -- one (fx:fy) integer frequency pair plus its shape-family name.
 *
 * WHY exact integers (and not floats):
 *   The integrality of fx and fy is the WHOLE REASON the Lissajous
 *   figure is a closed curve rather than a never-repeating space-filler.
 *   With fx, fy ∈ Z the parametric x(t), y(t) share a common period
 *
 *      T = 2π · lcm(fx, fy) / (fx · fy)
 *
 *   so the trajectory returns to (x(0), y(0)) after time T.  Irrational
 *   ratios make the curve ergodic -- it densely fills the rectangle
 *   and never closes.  See Strogatz [6] ch. 8 for the formal coupled-
 *   oscillator argument; Lawrence [3] for the classical closed-curve
 *   catalogue.
 *
 *   fx, fy are stored as int so the code reflects that exactness;
 *   callers promote to float at the math boundary (sinf / fminf).
 *
 * Lobe count (for gcd(fx, fy) = 1, phase φ = π/2):
 *   - 'fx' lobes along the X axis
 *   - 'fy' lobes along the Y axis
 *   When φ varies, the figure morphs continuously between a rotated
 *   bounding rectangle (φ = 0) and the fully-lobed figure (φ = π/2);
 *   eff_drift in §4 is what walks φ through this morph at a viewer-
 *   friendly speed.
 *
 * Convention:
 *   No required ordering between fx and fy.  Swapping them rotates the
 *   figure 90° but the shape family is unchanged (a 1:2 figure-8 lying
 *   horizontally is the same family as a 2:1 figure-8 lying vertically).
 *   k_ratios[] picks one canonical orientation per family.
 *
 * History:
 *   Lissajous 1857 [1] introduced the optical observation (perpendicular
 *   tuning forks viewed through a mirror).  Bowditch 1815 [2] derived
 *   the same family ~40 years earlier as compound-pendulum trajectories;
 *   the curves are sometimes called "Bowditch curves" in older texts.
 */
typedef struct {
    int         fx, fy;       /* exact integer frequencies (Z, not Q)    */
    const char *name;         /* visual family ("Trefoil", "Pentagram")  */
} Ratio;

static const Ratio k_ratios[N_RATIOS] = {
    { 1, 2, "1:2 Figure-8"   },
    { 2, 3, "2:3 Trefoil"    },
    { 3, 4, "3:4 Star"       },
    { 1, 3, "1:3 Clover"     },
    { 3, 5, "3:5 Pentagram"  },
    { 2, 5, "2:5 Five-petal" },
    { 4, 5, "4:5 Crown"      },
    { 1, 4, "1:4 Eye"        },
};

/*
 * Scene -- the live simulation state vector.
 *
 * In dynamical-systems language (Strogatz [6]), this is the state
 * vector of the visualiser: the minimum set of values you need to
 * render the next frame.  Everything else (CurveFrame, the trail
 * itself, HUD strings) is derived.
 *
 * WHY flat (no sub-structs):
 *   Five fields, each mutated independently by exactly one of {SPACE
 *   key, c key, n/p keys, +/- keys, scene_tick}.  Grouping into sub-
 *   structs would add indirection without grouping any reads/writes
 *   that actually co-vary.
 *
 * WHY indices (ratio_idx, theme_idx), not pointers:
 *   k_ratios[] and k_themes[] are const-static §3/§4 tables.  Indices
 *   are integers that cycle modulo N_RATIOS / N_THEMES, so n/p/c keys
 *   are a single arithmetic bump -- no pointer swizzling, no risk of
 *   dangling references after a future table refactor.  Resolved to
 *   pointers at the use site by scene_current_ratio / scene_current_theme.
 *
 * Fields (range / unit / mutator):
 *   ratio_idx   [0, N_RATIOS)    --   active row of k_ratios[]
 *                                     mutator: n/p keys, scene_tick wrap
 *   theme_idx   [0, N_THEMES)    --   active row of k_themes[]
 *                                     mutator: c key
 *   phase_x     [0, 2π)          rad  X-oscillator phase offset φ;
 *                                     drifts via eff_drift; on wrap,
 *                                     scene_tick advances ratio_idx
 *                                     mutator: scene_tick
 *   drift       [DRIFT_MIN,       rad/tick   base advance per tick
 *                DRIFT_MAX]              eff_drift scales it via the
 *                                        dwell envelope
 *                                     mutator: +/- keys (geometric ×÷
 *                                              DRIFT_STEP)
 *   paused      bool             --   freezes scene_tick; rendering
 *                                     continues so the user can still
 *                                     change theme / ratio while frozen
 *                                     mutator: SPACE key
 */
typedef struct {
    int   ratio_idx;   /* [0, N_RATIOS)   index into k_ratios[]            */
    int   theme_idx;   /* [0, N_THEMES)   index into k_themes[]            */
    float phase_x;     /* [0, 2π)         X-oscillator phase φ, radians   */
    float drift;       /* [MIN, MAX]      base phase advance, rad/tick     */
    bool  paused;      /* SPACE toggles; rendering still runs              */
} Scene;

/* Const-table accessors -- one line of indirection, but every call site
 * reads as the concept ("the current ratio", "the current theme")
 * rather than as the underlying table-lookup arithmetic. */
static inline const Ratio *scene_current_ratio(const Scene *s)
{
    return &k_ratios[s->ratio_idx];
}

static inline const Theme *scene_current_theme(const Scene *s)
{
    return &k_themes[s->theme_idx];
}

static void scene_init(Scene *s)
{
    s->ratio_idx = 0;
    s->theme_idx = 0;
    s->phase_x   = 0.0f;
    s->drift     = DRIFT_DEFAULT;
    s->paused    = false;
}

/* ---- phase-drift envelope ------------------------------------------ */

/*
 * The phase distance between successive symmetric Lissajous figures.
 *
 * Symmetric (mirror-axis-aligned) figures recur every π / max(fx, fy)
 * radians of phase as φ sweeps from 0 to 2π.  These are the "named"
 * shapes the demo wants to dwell on; everything in between is a morph.
 */
static float ratio_key_period(const Ratio *r)
{
    return (float)M_PI / fmaxf((float)r->fx, (float)r->fy);
}

/*
 * Dwell envelope: speed multiplier as a function of how close φ is to
 * the nearest key (symmetric-figure) phase.
 *
 *   distance 0           -> DWELL_SPEED       (slow, sits on the figure)
 *   distance kp·DWELL_WIDTH or more -> 1.0    (full speed, between figures)
 *   linear ramp in between
 *
 * Reading the curve: speed dips into a V-shaped well centred on each
 * key phase; the V's width is DWELL_WIDTH · kp and its floor is
 * DWELL_SPEED · base_drift.
 */
static float dwell_envelope(float phase, float key_period)
{
    float phase_mod    = fmodf(phase, key_period);
    float dist_to_key  = fminf(phase_mod, key_period - phase_mod);
    float dwell_frac   = dist_to_key / (key_period * DWELL_WIDTH);
    float ramp         = fminf(dwell_frac, 1.0f);
    return DWELL_SPEED + (1.0f - DWELL_SPEED) * ramp;
}

/*
 * eff_drift -- effective phase advance for this tick.
 * Base drift, multiplied by the dwell envelope so the figure "lingers"
 * on every symmetric shape before morphing.
 */
static float eff_drift(const Scene *s)
{
    float key_period = ratio_key_period(scene_current_ratio(s));
    float envelope   = dwell_envelope(s->phase_x, key_period);
    return s->drift * envelope;
}

/* ---- phase / index wrap helpers ------------------------------------- */

static inline bool phase_completed_cycle(float phase) { return phase >= TAU; }
static inline int  ratio_idx_next(int cur) { return (cur + 1) % N_RATIOS; }

static void scene_tick(Scene *s)
{
    if (s->paused) return;
    s->phase_x += eff_drift(s);
    if (phase_completed_cycle(s->phase_x)) {
        s->phase_x  -= TAU;
        s->ratio_idx = ratio_idx_next(s->ratio_idx);
    }
}

/* ===================================================================== */
/* §5  scene                                                              */
/* ===================================================================== */

/*
 * CurveFrame -- per-frame derived geometry for rendering the spiral.
 *
 * WHY a struct (not six locals at the top of scene_draw):
 *   curve_frame_make() does the algebra once; scene_draw just reads
 *   the fields and rasterises.  Separating "figure out the frame
 *   parameters" from "paint the points" makes each function single-
 *   purpose -- and the algebra is testable in isolation (it's pure
 *   arithmetic, no ncurses calls).
 *
 * WHY computed fresh each frame and NOT cached on Scene:
 *   Every field is a pure function of (Scene, rows, cols).  Caching on
 *   Scene would create a stale-value risk: the terminal could resize
 *   (SIGWINCH) between a cache-write and the next draw-read, leaving
 *   CurveFrame pointing at the wrong viewport.  Pure recomputation
 *   costs ~10 FLOPs/frame -- imperceptible.
 *
 * Fields & their derivations:
 *
 *   t_max  = N_LOOPS · 2π / min(fx, fy)              [rad of param t]
 *     Total parametric length: exactly N_LOOPS cycles of the SLOWER
 *     oscillator.  The slower oscillator sets the visible loop count;
 *     using min() instead of max() means a 1:5 ratio still shows
 *     N_LOOPS = 4 trips around the slow axis, just decorated with five
 *     times as much fast-axis wiggle.
 *
 *   decay  = DECAY_TOTAL / t_max                      [1/rad]
 *     Exponent rate for the e^(-decay · t) envelope.  Calibrated so
 *     amp(t_max) = exp(-DECAY_TOTAL) ≈ 1%, giving the same visible
 *     spiral depth regardless of frequency ratio.  Damped-oscillator
 *     framing in French [4]; harmonograph instrument in Whitaker [5].
 *
 *   cx, cy                                            [cell coords]
 *     Center of the figure in cell space; midpoint of the band left
 *     after HUD_TOP_ROWS + HUD_BOTTOM_ROWS reservations.
 *
 *   rx, ry                                            [cells]
 *     Aspect-corrected semi-axes.  Cells are CELL_ASPECT (~2x) taller
 *     than wide, so a geometrically round figure needs rx = ry ·
 *     CELL_ASPECT.  Horizontal budget shrinks by EDGE_MARGIN_X (1 cell
 *     on each side); vertical budget uses (draw_rows - 1) to convert a
 *     1-based row count into the max-index distance from cy.  Both
 *     scaled by AMP_FRAC so the figure leaves visible breathing room.
 *
 * Reference: Foley/van Dam [7] §11 on parametric-curve discretisation;
 *   we use uniform t-sampling (N_CURVE_PTS points), the textbook
 *   trade-off (simple, good enough at terminal resolutions).
 */
typedef struct {
    float t_max;     /* total parametric length (rad of t)               */
    float decay;     /* envelope rate; amp(t) = exp(-decay · t)          */
    float cx, cy;    /* figure center, cell space                        */
    float rx, ry;    /* aspect-corrected semi-axes, cell space           */
} CurveFrame;

/* ---- CurveFrame builders -------------------------------------------- */

/* Total parametric length: N_LOOPS cycles of the SLOWER oscillator,
 * so visible loop count is constant across frequency ratios. */
static float curve_t_max(const Ratio *r)
{
    return (float)N_LOOPS * TAU / fminf((float)r->fx, (float)r->fy);
}

/* Envelope rate calibrated so amp(t_max) = exp(-DECAY_TOTAL) ≈ 1%. */
static float curve_decay_rate(float t_max)
{
    return DECAY_TOTAL / t_max;
}

/* Figure center: horizontally on the screen midline, vertically in the
 * middle of the band that survives HUD_TOP_ROWS + HUD_BOTTOM_ROWS. */
static void figure_center(int rows, int cols, float *cx, float *cy)
{
    int draw_rows = rows - HUD_TOP_ROWS - HUD_BOTTOM_ROWS;
    *cx = cols * 0.5f;
    *cy = HUD_TOP_ROWS + draw_rows * 0.5f;
}

/* Largest aspect-corrected semi-axes (rx, ry) that fit AMP_FRAC of the
 * drawable band.  EDGE_MARGIN_X reserves 1 cell on each side; the
 * "draw_rows - 1" converts a 1-based count to the max-index distance
 * from center. */
static void fit_radii(int rows, int cols, float *rx, float *ry)
{
    int   draw_rows = rows - HUD_TOP_ROWS - HUD_BOTTOM_ROWS;
    float max_rx   = (cols - EDGE_MARGIN_X) * 0.5f;
    float max_ry   = (draw_rows - 1)        * 0.5f;
    *ry = fminf(max_rx / CELL_ASPECT, max_ry) * AMP_FRAC;
    *rx = *ry * CELL_ASPECT;
}

static CurveFrame curve_frame_make(const Scene *s, int rows, int cols)
{
    const Ratio *r = scene_current_ratio(s);
    CurveFrame cf;
    cf.t_max = curve_t_max(r);
    cf.decay = curve_decay_rate(cf.t_max);
    figure_center(rows, cols, &cf.cx, &cf.cy);
    fit_radii    (rows, cols, &cf.rx, &cf.ry);
    return cf;
}

/* ---- sample-level helpers ------------------------------------------- */

/* Parametric time for sample i in [0, N_CURVE_PTS).  i=0 -> t=0
 * (newest, outer), i=N-1 -> t=t_max (oldest, inner). */
static inline float sample_time(int i, float t_max)
{
    return (float)i / (float)(N_CURVE_PTS - 1) * t_max;
}

/* Damped-oscillator amplitude envelope at parametric time t. */
static inline float envelope_amp(float t, float decay)
{
    return expf(-decay * t);
}

/* The Lissajous parametric trajectory in NORMALISED [-1,1]^2 space.
 *   x(t) = amp · sin(fx · t + φ)
 *   y(t) = amp · sin(fy · t)
 * The amplitude is pre-multiplied so the spiral collapses to (0,0). */
static inline void lissajous_point(float t, float fx, float fy,
                                   float phase_x, float amp,
                                   float *nx, float *ny)
{
    *nx = amp * sinf(fx * t + phase_x);
    *ny = amp * sinf(fy * t);
}

/* Round-half-up to integer cell coordinate. */
static inline int cell_round(float v) { return (int)(v + 0.5f); }

/* Project a normalised (-1..1) point through CurveFrame to a cell.
 * Returns false if the cell falls in a HUD band or off-screen. */
static bool project_to_cell(const CurveFrame *cf, float nx, float ny,
                            int rows, int cols, int *out_col, int *out_row)
{
    int col = cell_round(cf->cx + nx * cf->rx);
    int row = cell_round(cf->cy + ny * cf->ry);
    if (row < HUD_TOP_ROWS || row >= rows - HUD_BOTTOM_ROWS) return false;
    if (col < 0 || col >= cols) return false;
    *out_col = col;
    *out_row = row;
    return true;
}

/* Map sample age in [0, 1] to brightness tier in [0, N_LEVELS-1]. */
static inline int age_to_level(float age)
{
    int lev = (int)(age * (float)N_LEVELS);
    if (lev >= N_LEVELS) lev = N_LEVELS - 1;
    return lev;
}

/* ncurses attribute for (theme, brightness tier).  The first
 * BOLD_LEVELS tiers get A_BOLD so they read as the bright loops. */
static attr_t level_attr(int theme_idx, int level)
{
    attr_t attr = COLOR_PAIR(CP_IDX(theme_idx, level));
    if (level < BOLD_LEVELS) attr |= A_BOLD;
    return attr;
}

/* Plot one sample at (row, col) in the theme/level style. */
static void plot_sample(int row, int col, int theme_idx, int level)
{
    char   ch   = k_lev_ch[level];
    attr_t attr = level_attr(theme_idx, level);
    attron(attr);
    mvaddch(row, col, (chtype)(unsigned char)ch);
    attroff(attr);
}

/*
 * scene_draw -- paint the full harmonograph spiral for the current phase.
 *
 * Reads as pseudocode:
 *   for each sample i from oldest (N-1) to newest (0):
 *       compute parametric time, amplitude, normalised (x,y)
 *       project to cell coordinates; skip if off-band or off-screen
 *       map age to brightness tier; plot
 *
 * Back-to-front iteration order: newer samples overwrite older on
 * shared cells, so the outer (brightest) loop wins overlap conflicts.
 */
static void scene_draw(const Scene *s, int rows, int cols)
{
    CurveFrame   cf = curve_frame_make(s, rows, cols);
    const Ratio *r  = scene_current_ratio(s);
    float fx = (float)r->fx, fy = (float)r->fy;
    float inv_n_minus_1 = 1.0f / (float)(N_CURVE_PTS - 1);

    for (int i = N_CURVE_PTS - 1; i >= 0; i--) {
        float t   = sample_time  (i, cf.t_max);
        float amp = envelope_amp (t, cf.decay);
        if (amp < AMP_VISIBLE_MIN) continue;

        float nx, ny;
        lissajous_point(t, fx, fy, s->phase_x, amp, &nx, &ny);

        int row, col;
        if (!project_to_cell(&cf, nx, ny, rows, cols, &col, &row)) continue;

        float age   = (float)i * inv_n_minus_1;
        int   level = age_to_level(age);
        plot_sample(row, col, s->theme_idx, level);
    }
}

/* Top HUD (row 0) — live data: ratio, phase, drift, theme, paused flag. */
static void scene_hud_top(const Scene *s, int cols)
{
    (void)cols;
    float phase_pi = s->phase_x / (float)M_PI;   /* display as multiples of π */
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0,
             " [Lissajous]  ratio: %-14s  phase: %.2fπ  drift: %.4f  theme: %-6s  %s ",
             scene_current_ratio(s)->name,
             phase_pi,
             s->drift,
             scene_current_theme(s)->name,
             s->paused ? "PAUSED " : "running");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Bottom HUD (row rows-1) — action keys. */
static void scene_hud_bottom(int rows, int cols)
{
    (void)cols;
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit   spc:pause   n/p:ratio   +/-:drift   c:theme ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §6  screen                                                             */
/* ===================================================================== */

/*
 * Screen -- cached terminal geometry.
 *
 * WHY cache cols/rows on a struct instead of calling getmaxyx() everywhere:
 *   getmaxyx is cheap but not free.  More importantly, many functions
 *   in the render layer take rows/cols as plain ints; passing one
 *   Screen* would conflate "I need the size" with "I need to talk to
 *   ncurses".  Holding size as a value lets the math layer stay
 *   terminal-agnostic; only screen_init / screen_resize / the SIGWINCH
 *   path in main() ever write into Screen.
 *
 * Update points:
 *   screen_init     -- once at startup
 *   screen_resize   -- after a SIGWINCH (terminal resized)
 *
 * Fields are in ncurses convention: cols = X-extent, rows = Y-extent.
 * Both are 1-based counts, so valid coordinates are
 *    [0, cols-1] × [0, rows-1].
 * The HUD reserves the first HUD_TOP_ROWS and the last HUD_BOTTOM_ROWS
 * of rows; the curve renderer must stay within the middle band.
 */
typedef struct {
    int cols;   /* terminal width  in character cells  */
    int rows;   /* terminal height in character cells  */
} Screen;

static void screen_init(Screen *sc)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_resize(Screen *sc)
{
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

/*
 * App -- the top-level container.
 *
 * Lives as a single static instance (g_app) so the signal handlers can
 * flip flags on it without scattering globals across the file.
 *
 * WHY signal handler flags are on App, not Scene:
 *   Signals interrupt the main loop, not the simulation.  "Should I
 *   quit?" and "did the terminal resize?" are run-loop concerns, not
 *   Lissajous-math concerns.  Keeping them off Scene means Scene
 *   stays PURE simulation state -- snapshottable, replayable from a
 *   seed, testable in isolation.
 *
 * WHY volatile sig_atomic_t (and not bool):
 *   Per POSIX, sig_atomic_t is the only integer type guaranteed atomic
 *   with respect to async signal delivery.  `volatile` forces the
 *   main loop to re-read from memory each iteration rather than
 *   caching the flag in a register; without it the compiler may
 *   legally hoist the read out of the loop and the program would
 *   never see the signal-set value.
 *
 * Fields:
 *   scene        the live simulation state           (§4 Scene)
 *   screen       cached terminal dimensions          (§6 Screen)
 *   running      0 ⇒ main loop should exit          (SIGINT/SIGTERM)
 *   need_resize  1 ⇒ main loop should re-init screen (SIGWINCH)
 */
typedef struct {
    Scene                  scene;        /* simulation state              */
    Screen                 screen;       /* terminal size cache           */
    volatile sig_atomic_t  running;      /* 0 = exit main loop  (SIGINT)  */
    volatile sig_atomic_t  need_resize;  /* 1 = re-init screen  (SIGWINCH)*/
} App;

static App g_app;

static void sig_handler(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}
static void cleanup(void) { endwin(); }

/* Returns false only when the user pressed a quit key. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': return false;
    case ' ':           s->paused ^= 1;                                  break;
    case 'c': case 'C': s->theme_idx = (s->theme_idx + 1) % N_THEMES;    break;
    case 'n': case 'N':
        s->ratio_idx = (s->ratio_idx + 1) % N_RATIOS;
        s->phase_x   = 0.0f;
        break;
    case 'p': case 'P':
        s->ratio_idx = (s->ratio_idx + N_RATIOS - 1) % N_RATIOS;
        s->phase_x   = 0.0f;
        break;
    case '+': case '=':
        s->drift *= DRIFT_STEP;
        if (s->drift > DRIFT_MAX) s->drift = DRIFT_MAX;
        break;
    case '-': case '_':
        s->drift /= DRIFT_STEP;
        if (s->drift < DRIFT_MIN) s->drift = DRIFT_MIN;
        break;
    }
    return true;
}

static void install_signal_handlers(void)
{
    signal(SIGINT,   sig_handler);
    signal(SIGTERM,  sig_handler);
    signal(SIGWINCH, sig_handler);
    atexit(cleanup);
}

static void app_init(App *app)
{
    app->running     = 1;
    app->need_resize = 0;
    screen_init(&app->screen);
    scene_init(&app->scene);
}

/* ---- main-loop step helpers ----------------------------------------- */

/* Respond to SIGWINCH: re-query the terminal size and clear the canvas
 * so the next frame redraws from scratch. */
static void apply_resize(App *app)
{
    app->need_resize = 0;
    screen_resize(&app->screen);
    erase();
}

/* Drain every pending keystroke through app_handle_key.
 * Returns false if a quit key was pressed (caller exits). */
static bool drain_input(App *app)
{
    int ch;
    while ((ch = getch()) != ERR) {
        if (!app_handle_key(app, ch)) return false;
    }
    return true;
}

/* Tick the simulation and paint one complete frame (curve + HUDs). */
static void frame_render(App *app)
{
    scene_tick(&app->scene);
    erase();
    scene_draw      (&app->scene, app->screen.rows, app->screen.cols);
    scene_hud_top   (&app->scene, app->screen.cols);
    scene_hud_bottom(             app->screen.rows, app->screen.cols);
    wnoutrefresh(stdscr);
    doupdate();
}

/* Sleep until wall-clock 'deadline', then return the NEXT deadline
 * (one TICK_NS later).  Steady-cadence pacing -- frame drift is
 * absorbed by the sleep, not by the simulation. */
static long long pace_to_deadline(long long deadline)
{
    deadline += TICK_NS;
    clock_sleep_ns(deadline - clock_ns());
    return deadline;
}

/*
 * main() -- the orchestrator.  Each line names one phase of a frame.
 */
int main(void)
{
    install_signal_handlers();

    App *app = &g_app;
    app_init(app);

    long long next_frame_deadline = clock_ns();

    while (app->running) {
        if (app->need_resize)  apply_resize(app);
        if (!drain_input(app)) { app->running = 0; break; }

        frame_render(app);
        next_frame_deadline = pace_to_deadline(next_frame_deadline);
    }

    return 0;
}
