/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lorenz.c — Lorenz Strange Attractor [1, 2]
 *
 * Integrates the three Lorenz ODEs with RK4 [6].  Multiple trajectories
 * run in parallel: a main trajectory plus 5 "ghost" ε-offset companions
 * that diverge on the Lyapunov timescale [7], making chaos visible.
 *
 * Framework: follows framework.c §1–§8 skeleton.
 *
 * ═════════════════════════════════════════════════════════════════════
 *  WHAT YOU SEE ON SCREEN
 * ═════════════════════════════════════════════════════════════════════
 *
 *  A "butterfly" — the famous Lorenz strange attractor [1, 4] — drawn
 *  as a glowing 3-D trail rotating slowly through space.  The trajectory
 *  weaves between two unstable spiral fixed points (marked '+'), tracing
 *  a bounded but never-repeating orbit.  A sparse starfield drifts past
 *  in the background for parallax; a fan of fainter ghost trajectories
 *  spreads alongside the main one — Lyapunov chaos rendered visceral.
 *
 *  Each frame paints back-to-front:
 *
 *    1. STARFIELD   — 60 ambient '.' stars in deep space, rotating with
 *                     the attractor for parallax (cheap volumetric cue
 *                     [8]).
 *    2. EQUILIBRIA  — three '+' crosshairs at the fixed points: the
 *                     origin (saddle), and the two unstable spirals
 *                     C± = (±√72, ±√72, 27) that each lobe orbits [2].
 *    3. GHOSTS      — 5 ε-offset trajectories, ε ∈ {0.001 .. 0.1}.
 *                     They start clustered, then fan out exponentially
 *                     as Lyapunov divergence kicks in (λ ≈ 0.9 → ε
 *                     doubles every 0.77 Lorenz-time, reaching attractor
 *                     scale in ~10 real seconds [7]).  Toggle with g.
 *    4. MAIN TRAIL  — newest 2 points = 'O' BOLD + 4-cross '+' bloom
 *                     halo (comet head); oldest 80% = fading '.' in
 *                     theme tail colour.  DEPTH CUEING [8, 9] — points
 *                     close to the camera paint BOLD, ones behind fade
 *                     A_DIM, so the trail visibly pulses in and out of
 *                     the page as the view rotates.
 *    5. ε-SEP TEXT  — live distance from main trajectory to smallest-ε
 *                     ghost.  Watch it climb from 0.001 to ~30 over
 *                     ~10 real seconds — the Lyapunov timescale.
 *
 *  Two alternate views (single-key toggles):
 *
 *    LOBE MODE (l)    — colour by attractor wing (sign of x): right
 *                       wing in theme head, left wing in theme ghost.
 *                       Each chaotic wing-switch becomes a vivid hue
 *                       flip — Lorenz bistability [3] made visceral.
 *    DENSITY MODE (h) — replace the trail with a 2-D screen-space
 *                       heatmap of trail visits.  Reveals the attractor's
 *                       invariant measure — where the trajectory actually
 *                       spends its time [3, 4].  Resets on rotation so
 *                       each angle re-accumulates from scratch.
 *
 *  THEMES (t / T): 10 four-anchor palettes (MATRIX / FIRE / OCEANIC /
 *  NEON / MONO / ICE / NOVA / FOREST / DESERT / ECLIPSE) re-skin every
 *  coloured element coherently.  Themes are pure rendering — the
 *  physics is identical across all 10.
 *
 * PHYSICS SUMMARY
 * ─────────────────────────────────────────────────────────────────────
 * Lorenz ODEs [1] (σ=10, ρ=28, β=8/3 — the classical chaotic regime):
 *   dx/dt = σ(y − x)            σ = Prandtl number      [5]
 *   dy/dt = x(ρ − z) − y        ρ = Rayleigh ratio       [5]
 *   dz/dt = xy − βz             β = convection geometry  [5]
 *
 * Integrated with classical four-stage Runge-Kutta (RK4) [6], step
 * h = 0.005 Lorenz-time units.  SUB_STEPS = 8 sub-steps per sim tick
 * → 0.04 Lorenz-time / tick.  At 60 ticks/s: 2.4 Lorenz-time/s advance.
 *
 * Lyapunov exponent λ ≈ 0.9 [7] → e-folding time ≈ 1.11 Lorenz-time.
 * Smallest ghost ε = 0.001 reaches attractor scale (~30) after ~37
 * doublings ≈ 13 Lorenz-time ≈ 5.4 s real-time.  Visible divergence
 * within 4 s.
 *
 * PROJECTION  (implemented as the 5-step pipeline in §4)
 * ─────────────────────────────────────────────────────────────────────
 * Orthographic with azimuth φ and elevation θ.  Attractor centred at
 * (0,0,25) before projection; the rotated z-component sz is kept as
 * camera-axis depth and used by the renderer for depth cueing [8].
 *   pz  = z − 25                            (centre on origin)
 *   rx  =  x·cosφ + y·sinφ                  (rotate around z by φ)
 *   ry  = −x·sinφ + y·cosφ
 *   sx  = rx                                (tilt around x by θ)
 *   sy  =  ry·cosθ + pz·sinθ
 *   sz  = −ry·sinθ + pz·cosθ                ← depth (+ = behind, − = in front)
 *   col = cx + sx·scale
 *   row = cy − sy·scale·ASPECT
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   r         restart trajectories (also resets density heatmap)
 *   g         toggle multi-ghost shower (5 ε-offset trajectories)
 *   l         toggle LOBE coloring (colour by attractor wing)
 *   h         toggle DENSITY HEATMAP view (replaces trail)
 *   a         toggle auto-rotation
 *   ← →       azimuth (manual)
 *   ↑ ↓       elevation
 *   ] / [     sim Hz up / down
 *   t / T     next / previous theme (10 four-anchor palettes)
 *
 * ═════════════════════════════════════════════════════════════════════
 *  REFERENCES  (cite inline as [n])
 * ═════════════════════════════════════════════════════════════════════
 *
 *  ── Lorenz system & chaos theory ────────────────────────────────────
 *
 *   [1] Lorenz, E. N. (1963) — "Deterministic Nonperiodic Flow",
 *       *Journal of the Atmospheric Sciences* 20 (2), 130-141.  THE
 *       original paper.  Derives the three-equation simplification from
 *       atmospheric convection and first demonstrates "sensitive
 *       dependence on initial conditions" — what every ghost-shower
 *       frame in this demo re-proves.
 *
 *   [2] Strogatz, S. H. (2014) — *Nonlinear Dynamics and Chaos*, 2nd ed.,
 *       Westview Press.  The canonical chaos textbook; Ch.9 covers the
 *       Lorenz attractor in 30 readable pages — strange attractor,
 *       Lyapunov exponents, bifurcations.  Best single entry point if
 *       you want to understand WHY the butterfly is shaped this way.
 *
 *   [3] Sparrow, C. (1982) — *The Lorenz Equations: Bifurcations, Chaos,
 *       and Strange Attractors*, Springer Applied Mathematical Sciences
 *       41.  Deep monograph: bifurcation diagrams, return maps, hetero
 *       and homoclinic orbits, the invariant measure on the attractor.
 *       The reference for what density mode is showing.
 *
 *   [4] Tucker, W. (1999) — "The Lorenz attractor exists", *Comptes
 *       Rendus de l'Académie des Sciences Paris* 328 (12), 1197-1202.
 *       Computer-assisted proof that the observed object is a genuine
 *       chaotic attractor (not a numerical artefact).  Makes the
 *       "strange attractor" terminology mathematically rigorous.
 *
 *   [5] Saltzman, B. (1962) — "Finite Amplitude Free Convection as an
 *       Initial Value Problem—I", *J. Atmos. Sci.* 19 (4), 329-341.
 *       The 7-mode convection model that Lorenz [1] truncated to 3
 *       modes.  Source of the physical interpretation of σ (Prandtl),
 *       ρ (Rayleigh-ratio), β (convection-cell geometry).
 *
 *  ── Numerical integration & Lyapunov analysis ──────────────────────
 *
 *   [6] Press, W. H.; Teukolsky, S. A.; Vetterling, W. T.; Flannery,
 *       B. P. (2007) — *Numerical Recipes*, 3rd ed., Cambridge.
 *       Ch.17 covers classical RK4 derivation, local-error analysis,
 *       and the step-size / accuracy trade-off.  Backs the L_H = 0.005
 *       choice (RK4 global error ≈ h⁴ ≈ 6×10⁻¹⁰ per step).
 *
 *   [7] Wolf, A.; Swift, J. B.; Swinney, H. L.; Vastano, J. A. (1985) —
 *       "Determining Lyapunov exponents from a time series", *Physica D*
 *       16 (3), 285-317.  Foundational paper on numerical Lyapunov
 *       computation.  λ ≈ 0.9056 for σ=10, ρ=28, β=8/3 comes from here;
 *       it sets the timescale on which the ε-sep indicator climbs.
 *
 *  ── Visualisation ──────────────────────────────────────────────────
 *
 *   [8] Ware, C. (2020) — *Information Visualization: Perception for
 *       Design*, 4th ed., Morgan Kaufmann.  Ch.4 on perceptual contrast
 *       backs the depth × age brightness coding; Ch.5 on motion and
 *       depth backs the auto-rotation + parallax-starfield strategy.
 *
 *   [9] Foley, J. D.; van Dam, A.; Feiner, S. K.; Hughes, J. F. (1995) —
 *       *Computer Graphics: Principles and Practice*, 2nd ed., Addison-
 *       Wesley.  Ch.6 / 12 cover orthographic projection math — the
 *       azimuth-elevation rotation in §4 project() comes straight from
 *       this textbook.
 *
 * ═════════════════════════════════════════════════════════════════════
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra lorenz.c -o lorenz -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Classical four-stage Runge-Kutta (RK4) [6] applied to
 *                  the autonomous 3-variable Lorenz ODE.  Autonomous =
 *                  no explicit t in dx/dt = f(x,y,z), so each RK4 step
 *                  uses only the current state.  Step size L_H is fixed
 *                  in Lorenz-time units regardless of display fps —
 *                  decouples physics from rendering speed.
 *
 * Physics/Math   : Deterministic chaos on a strange attractor [1, 4].
 *                  Lorenz (1963) derived these equations from Saltzman's
 *                  truncated convection model [5]:
 *                    σ (sigma) = Prandtl number  (ν/κ)
 *                    ρ (rho)   = Rayleigh-ratio  (Ra / Ra_critical)
 *                    β (beta)  = convection-cell geometric factor
 *                  At σ=10, ρ=28, β=8/3 the system has a STRANGE
 *                  ATTRACTOR: bounded but never repeating, Lyapunov
 *                  exponent λ ≈ 0.9 [7], Hausdorff dimension ≈ 2.06.
 *                  Tucker [4] proved the geometric existence of this
 *                  attractor in 1999 (computer-assisted), settling the
 *                  question whether the picture was a numerical artefact.
 *
 * Rendering      : Orthographic projection [9] with azimuth φ and
 *                  elevation θ.  The 3-D point (x,y,z) is rotated around
 *                  the z-axis by φ, then the (ry, z) plane is tilted by θ
 *                  to produce screen (sx, sy) plus a camera-axis depth sz
 *                  (the latter drives the depth-cue brightness [8]).
 *                  ASPECT compensates for non-square terminal cell ratio.
 *                  Multi-layer pipeline (back-to-front): starfield →
 *                  equilibrium markers → ghosts → main trail (with
 *                  bloom + depth) OR density heatmap.
 *
 * Data-structure : Ring-buffer trail (TRAIL_LEN=2500 points) per
 *                  trajectory; 1 main + 5 ghost = 6 trails total.
 *                  At 60 fps × 8 sub-steps = 480 points/s, 2500 points
 *                  ≈ 5.2 s of trajectory history — long enough for the
 *                  full butterfly shape to be visible at any moment.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#define M_PI 3.14159265358979323846
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
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 60,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    = 10,
    FPS_UPDATE_MS   = 500,
    N_COLORS        = 7,
    TRAIL_LEN       = 2500,   /* ring-buffer length per trajectory          */
    SUB_STEPS       = 8,      /* RK4 sub-steps per sim tick                 */
};

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC / (f))

/* Lorenz system parameters — classic "chaotic" values:
 * σ=10 (Prandtl ≈ air), ρ=28 (Rayleigh just above onset of turbulence),
 * β=8/3 (geometry factor for a square convection cell).
 * Changing any of these shifts the system between periodic, quasi-periodic,
 * and chaotic regimes.  ρ < 1 → all trajectories converge to origin.
 * ρ ≈ 24.74 → onset of chaos.  ρ > 28 → fully developed butterfly attractor. */
#define L_SIGMA   10.0f
#define L_RHO     28.0f
#define L_BETA    (8.0f / 3.0f)

/* L_H: fixed RK4 step in Lorenz time units.
 * At h=0.005 the RK4 global error is O(h⁴) ≈ 6×10⁻¹⁰ per step — far
 * below float precision over the first 10 s.  Larger h (e.g. 0.02) causes
 * the trajectory to visibly drift off the true attractor.                 */
#define L_H       0.005f

/* N_GHOSTS: number of ε-offset ghost trajectories shown alongside the
 * main trajectory.  Each ghost gets a different initial offset from
 * GHOST_EPS_TABLE below.  The fan-out from initially-close ghosts to
 * fully-spread ghosts IS the visual signature of Lyapunov chaos. */
#define N_GHOSTS 5

/* GHOST_EPS_TABLE: per-ghost initial x-offset (Lorenz units).
 * Spans four orders of magnitude (0.001 → 0.1) so the divergence
 * timescale is visible across one viewing session: the smallest ε
 * stays close to the main trail for ~5 Lyapunov times before fully
 * spreading; the largest ε reaches attractor scale almost immediately.
 * Lyapunov exponent λ ≈ 0.9 → ε doubles every ~0.77 Lorenz-time. */
static const float GHOST_EPS_TABLE[N_GHOSTS] = {
    0.001f, 0.005f, 0.01f, 0.05f, 0.1f
};

/* View */
#define CELL_W  8
#define CELL_H  16
#define ASPECT  ((float)CELL_W / (float)CELL_H)   /* ≈ 0.5 */

#define VIEW_PHI_DEFAULT    0.5f    /* initial azimuth (rad)               */
#define VIEW_THETA_DEFAULT  0.55f   /* initial elevation (rad)             */
#define VIEW_PHI_SPEED      0.08f   /* auto-rotation speed (rad/s)         */

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
/* §3  color / theme                                                      */
/* ===================================================================== */

enum {
    CP_TRAIL_HEAD = 1,   /* newest trail tier — theme head colour       */
    CP_TRAIL_MID  = 2,   /* mid-age trail tier — theme mid colour       */
    CP_TRAIL_TAIL = 3,   /* oldest trail tier — theme tail colour       */
    CP_GHOST      = 4,   /* ghost trajectory + ε-sep indicator          */
    CP_HUD        = 5,   /* status bar — canonical bright yellow (226)  */
    CP_HINT       = 6,   /* action keys — canonical bright cyan  (51)   */
};

/*
 * Theme — 4 colour anchors for the Lorenz trail visualisation.
 *
 * The trail is rendered with an AGE-BASED COLOUR RAMP: newest points
 * paint with `head` + A_BOLD, mid-age points use `mid`, oldest points
 * use `tail` (with A_DIM for the very oldest tier).  The `ghost`
 * colour paints both the ε-offset trajectory and the inline
 * separation indicator.
 *
 * Picking themes is just choosing 4 coordinated hues — no ramp logic,
 * no interpolation.  Each theme tells a different visual story while
 * preserving the same age-to-brightness signal (newest = brightest).
 *
 * Fixed pairs across all themes:
 *   CP_HUD   bright yellow 226 (status bar, CLAUDE.md canonical)
 *   CP_HINT  bright cyan   51  (action keys, CLAUDE.md canonical)
 */
typedef struct {
    const char *name;
    short head;    /* newest trail tier — painted with A_BOLD           */
    short mid;     /* mid-age trail tier                                */
    short tail;    /* oldest trail tier — A_DIM for ancient subtier     */
    short ghost;   /* ghost trajectory + ε-sep indicator                */
} Theme;

/*
 * 10 themes — hand-picked 4-anchor palettes in the xterm-256 palette.
 *
 *   MATRIX  — phosphor green CRT
 *   FIRE    — red / orange / yellow flame (original default)
 *   OCEANIC — deep blue → bright cyan
 *   NEON    — hot pink electric on purple
 *   MONO    — pure greyscale
 *   ICE     — pale blue / white frost
 *   NOVA    — white star burst with purple shadow
 *   FOREST  — lime / olive / dark green
 *   DESERT  — gold / orange / brown
 *   ECLIPSE — bright red fading into purple shadow
 */
static const Theme THEMES[] = {
    /*  name         head  mid  tail  ghost */
    { "MATRIX",        46,  40,   22,   246 },
    { "FIRE",         196, 208,  226,   201 },
    { "OCEANIC",       51,  45,   24,   153 },
    { "NEON",         199, 213,   57,   226 },
    { "MONO",         255, 250,  244,   238 },
    { "ICE",          195, 159,   75,   153 },
    { "NOVA",         231, 213,   57,   117 },
    { "FOREST",       154, 142,   22,   130 },
    { "DESERT",       220, 215,  130,   137 },
    { "ECLIPSE",      196,  88,   53,    67 },
};
#define N_THEMES ((int)(sizeof THEMES / sizeof THEMES[0]))

/* Active theme index — cycled by t/T keys.  File-scope so theme_apply
 * is callable from both startup (color_init) and the runtime key
 * handler. */
static int g_theme_idx = 0;

/*
 * theme_apply — rebuild the THEME-DEPENDENT colour pairs for theme
 * `idx`.  Fixed pairs (CP_HUD, CP_HINT) are NOT touched — they live
 * in color_init() and stay constant across themes.
 */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) return;
    const Theme *t = &THEMES[idx];

    if (COLORS >= 256) {
        init_pair(CP_TRAIL_HEAD, t->head,  -1);
        init_pair(CP_TRAIL_MID,  t->mid,   -1);
        init_pair(CP_TRAIL_TAIL, t->tail,  -1);
        init_pair(CP_GHOST,      t->ghost, -1);
    } else {
        /* 8-colour ANSI fallback — themes collapse to one palette */
        init_pair(CP_TRAIL_HEAD, COLOR_RED,     -1);
        init_pair(CP_TRAIL_MID,  COLOR_YELLOW,  -1);
        init_pair(CP_TRAIL_TAIL, COLOR_GREEN,   -1);
        init_pair(CP_GHOST,      COLOR_MAGENTA, -1);
    }
}

/*
 * color_init — one-shot at startup.  Binds the two FIXED HUD pairs
 * (yellow status, cyan hints) and then applies the initial theme.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(CP_HUD,  226, -1);   /* canonical bright yellow */
        init_pair(CP_HINT,  51, -1);   /* canonical bright cyan   */
    } else {
        init_pair(CP_HUD,  COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN,   -1);
    }

    theme_apply(g_theme_idx);
}

/* ===================================================================== */
/* §4  coords — orthographic 3-D → 2-D projection                        */
/* ===================================================================== */

/* ── project() math primitives ──────────────────────────────────────── *
 *
 * The full 3-D → 2-D pipeline is four named transformations applied in
 * order — translate the world so the attractor is centred at the origin,
 * rotate around z by azimuth φ, tilt around x by elevation θ, then map
 * (sx, sy) to a terminal cell.  Each step gets a tiny helper so the
 * orchestrator project() reads as the textbook recipe.                */

/* (1) TRANSLATE — recentre Lorenz space so the attractor (which sits
 *     mostly in z ∈ [0, 50]) is centred on the origin (z = 0 after
 *     translation).  Pure shift; x and y unchanged. */
static inline void world_center_at_attractor(float lx, float ly, float lz,
                                              float *px, float *py, float *pz)
{
    *px = lx;
    *py = ly;
    *pz = lz - 25.0f;
}

/* (2) ROTATE AROUND Z by azimuth φ.  Stand 2-D rotation in the (x, y)
 *     plane; the convention here puts the camera implicitly along +y
 *     after rotation, so ry becomes the "into-screen" axis. [9] */
static inline void rotate_around_z_axis(float px, float py, float phi,
                                         float *rx, float *ry)
{
    *rx =  px * cosf(phi) + py * sinf(phi);
    *ry = -px * sinf(phi) + py * cosf(phi);
}

/* (3) TILT AROUND X by elevation θ.  Rotates the (ry, pz) plane.  After
 *     tilt, sy is the vertical screen coord and sz is the camera-axis
 *     depth (positive = behind centre / farther; negative = in front /
 *     closer).  Used downstream for depth-cued shading. [8, 9] */
static inline void tilt_around_x_axis(float ry, float pz, float theta,
                                       float *sy, float *sz)
{
    *sy =  ry * cosf(theta) + pz * sinf(theta);
    *sz = -ry * sinf(theta) + pz * cosf(theta);
}

/*
 * project() — map a Lorenz-space point to terminal (col, row, depth).
 *
 * Pseudocode pipeline (matches the helpers above):
 *
 *   (1) translate world so attractor centres on origin
 *   (2) rotate around z by azimuth φ        → (rx, ry, pz)
 *   (3) tilt   around x by elevation θ      → screen (sx=rx, sy) + depth sz
 *   (4) scale  + offset to cell coords      → (col, row)
 *   (5) clip   to the renderable band       → return false if off-screen
 *
 * out_depth: pass NULL when the caller doesn't need depth shading
 * (e.g. equilibrium markers, density binning).  Trail rendering and
 * starfield pass &depth so closer points get a brightness boost.
 *
 * Returns false if (col, row) falls outside the renderable band
 * (rows [1, rows-2], cols [0, cols-1]).
 */
static bool project(float lx, float ly, float lz,
                    float phi, float theta, float scale,
                    int cx, int cy, int cols, int rows,
                    int *out_col, int *out_row, float *out_depth)
{
    float px, py, pz;
    world_center_at_attractor(lx, ly, lz, &px, &py, &pz);  /* (1) */

    float rx, ry;
    rotate_around_z_axis(px, py, phi, &rx, &ry);           /* (2) */

    float sx = rx;
    float sy, sz;
    tilt_around_x_axis(ry, pz, theta, &sy, &sz);           /* (3) */

    int col = cx + (int)(sx * scale);                      /* (4) */
    int row = cy - (int)(sy * scale * ASPECT);

    *out_col = col;
    *out_row = row;
    if (out_depth) *out_depth = sz;
    return (col >= 0 && col < cols && row >= 1 && row < rows - 1); /* (5) */
}

/* ===================================================================== */
/* §5  entity — Lorenz                                                    */
/* ===================================================================== */

/*
 * Trail — bounded history of integration points for one trajectory.
 *
 * WHY a ring buffer (and not a linear append-only array):
 *   the integration runs forever; a linear buffer would either grow
 *   unbounded (memory leak) or stop recording after some cap (info
 *   loss).  A ring of TRAIL_LEN slots overwrites the oldest sample
 *   on each push — bounded memory, O(1) push, and the last
 *   TRAIL_LEN samples are always available.  TRAIL_LEN = 2500 covers
 *   ≈ 5.2 s of trajectory at our 60-fps × 8-substep cadence — long
 *   enough for the full butterfly outline to be visible at any time
 *   without storing minutes of history.
 *
 * WHY {head, count} (and not {head, tail}):
 *   the buffer fills monotonically (no element is ever removed
 *   except by overwrite), so a tail pointer would just be
 *   head − count + 1 mod TRAIL_LEN — redundant.  count also gives
 *   the renderer a free progress signal: count < TRAIL_LEN means the
 *   ring isn't full yet, so the oldest-end shows what's actually
 *   there rather than uninitialised noise.
 *
 * WHY parallel x[]/y[]/z[] arrays (not an array of {x,y,z} structs):
 *   the renderer walks the ring projecting one point at a time and
 *   reads all three coords for each point — so SoA vs AoS makes no
 *   cache-locality difference here.  Parallel arrays match the
 *   trail_push / trail_clear interface (three scalars in) and avoid
 *   needing a Vec3 type that doesn't exist elsewhere in the file.
 */
typedef struct {
    float x[TRAIL_LEN];   /* x coordinates of the last TRAIL_LEN samples  */
    float y[TRAIL_LEN];   /* y coordinates                                */
    float z[TRAIL_LEN];   /* z coordinates                                */
    int   head;
        /* Index of the NEWEST sample.  trail_push advances head before
         * writing.  Renderer iterates k = 0..count-1 with index
         * (head - k + TRAIL_LEN) % TRAIL_LEN → newest first. */
    int   count;
        /* Number of valid samples, ≤ TRAIL_LEN.  Saturates at
         * TRAIL_LEN once the ring is full.  Drives the renderer's
         * age normalisation: age = k / (count - 1).                 */
} Trail;

static void trail_push(Trail *t, float x, float y, float z)
{
    t->head = (t->head + 1) % TRAIL_LEN;
    t->x[t->head] = x;
    t->y[t->head] = y;
    t->z[t->head] = z;
    if (t->count < TRAIL_LEN) t->count++;
}

static void trail_clear(Trail *t)
{
    t->head  = 0;
    t->count = 0;
}

/* ── Lorenz ODE ────────────────────────────────────────────────────── */

static void lorenz_deriv(float x, float y, float z,
                          float *dx, float *dy, float *dz)
{
    *dx = L_SIGMA * (y - x);
    *dy = x * (L_RHO - z) - y;
    *dz = x * y - L_BETA * z;
}

static void lorenz_rk4(float *x, float *y, float *z, float h)
{
    float k1x, k1y, k1z;
    float k2x, k2y, k2z;
    float k3x, k3y, k3z;
    float k4x, k4y, k4z;

    lorenz_deriv(*x, *y, *z, &k1x, &k1y, &k1z);
    lorenz_deriv(*x + h*0.5f*k1x, *y + h*0.5f*k1y, *z + h*0.5f*k1z,
                 &k2x, &k2y, &k2z);
    lorenz_deriv(*x + h*0.5f*k2x, *y + h*0.5f*k2y, *z + h*0.5f*k2z,
                 &k3x, &k3y, &k3z);
    lorenz_deriv(*x + h*k3x, *y + h*k3y, *z + h*k3z,
                 &k4x, &k4y, &k4z);

    *x += (h / 6.0f) * (k1x + 2.0f*k2x + 2.0f*k3x + k4x);
    *y += (h / 6.0f) * (k1y + 2.0f*k2y + 2.0f*k3y + k4y);
    *z += (h / 6.0f) * (k1z + 2.0f*k2z + 2.0f*k3z + k4z);
}

/* ── Lorenz entity ─────────────────────────────────────────────────── *
 *
 * The entire visible world in one struct: 6 trajectories (1 main +
 * 5 ghosts), their per-trail ring buffers, the camera, and the
 * render-toggle latches.
 *
 * WHY a struct (and not loose globals): every field shares the same
 *   lifetime — born at scene_init, dies at scene_free (none today,
 *   but the API supports it).  Bundling lets `Lorenz *l` carry the
 *   whole world through lorenz_tick / lorenz_draw / handle_key
 *   without 15-argument function signatures.
 *
 * WHY ONE struct (not separate SimState + RenderState):
 *   the renderer reads simulation fields (trajectory positions) every
 *   frame; the simulation never reads render state.  Splitting would
 *   force the renderer to take two arguments and would buy nothing —
 *   the lifetime is identical.  We use comment dividers below to mark
 *   the sim/render boundary INSIDE the struct.
 *
 * LOCALITY INTENT (fields grouped, top-to-bottom):
 *
 *   SIMULATION HALF  — physics state, advanced by lorenz_tick:
 *     mx, my, mz, mt              main trajectory + trail
 *     gx[], gy[], gz[], gt[]      N_GHOSTS ghost trajectories + trails
 *
 *   VIEW (shared)    — written by handle_key (manual), read by every
 *                      renderer.  auto_rotate also lets lorenz_tick
 *                      drift phi for hands-free demos:
 *     phi, theta                  azimuth + elevation (radians)
 *     auto_rotate                 hands-free rotation on/off
 *
 *   RENDERING / UI latches — written by handle_key only, read by
 *                            lorenz_draw and the HUD overlay:
 *     show_ghost                  g — multi-ghost shower
 *     show_lobe                   l — colour by attractor wing
 *     show_density                h — density heatmap (replaces trail)
 *     paused                      space — lorenz_tick early-returns
 *
 * References for the field semantics: [1] for the trajectory ODE,
 * [7] for the Lyapunov-driven ghost divergence timescale, [3] for
 * the attractor's invariant measure (what density mode reveals),
 * [9] for the (phi, theta) projection conventions.
 */
typedef struct {

    /* ── SIMULATION: main trajectory ─────────────────────────────── */
    float  mx, my, mz;
        /* Current 3-D state of the MAIN trajectory in Lorenz space.
         * Advanced by lorenz_rk4 once per sub-step.  Initial state
         * (1, 1, 1) — approximately on the attractor.            [1] */

    Trail  mt;
        /* Ring-buffer history of the main trajectory — see Trail
         * doc.  Pushed once per sub-step; rendered with age + depth
         * + (optional) lobe colouring + bloom on the head.        */

    /* ── SIMULATION: N_GHOSTS ε-offset shadow trajectories ────────── */
    float  gx[N_GHOSTS], gy[N_GHOSTS], gz[N_GHOSTS];
        /* Each ghost g starts at (1 + ε_g, 1, 1) where ε_g comes
         * from GHOST_EPS_TABLE (0.001..0.1 spanning four decades).
         * Same Lorenz ODE as the main — by construction they
         * differ ONLY in initial conditions, so any divergence is
         * pure deterministic chaos (sensitive dependence on i.c.).
         * Diverge exponentially as exp(λ t), λ ≈ 0.9 [7].         */

    Trail  gt[N_GHOSTS];
        /* Per-ghost ring buffer.  Rendered in CP_GHOST + A_DIM with
         * ',' glyph (no lobe colouring, no bloom).  The widest-ε
         * ghost paints first so the close ghosts overlay it —
         * visual order matches the divergence order.             */

    /* ── VIEW (written by handle_key, read by every renderer) ─────── */
    float  phi;
        /* Azimuth in radians — rotation around the z-axis.  Auto-
         * incremented by VIEW_PHI_SPEED·dt when auto_rotate is on;
         * adjusted by ←/→ keys (which also disable auto_rotate).
         * Sets the rx/ry plane in project(). [9]                  */

    float  theta;
        /* Elevation in radians — tilt of the (ry, z) plane toward
         * the camera.  Adjusted by ↑/↓; clamped to [0.1, 1.4] so
         * the view stays away from gimbal singularities.          */

    bool   auto_rotate;
        /* If true, lorenz_tick drifts phi for a hands-free demo.
         * Toggled by 'a'; also disabled implicitly by any manual
         * ←/→ keypress so the user never has to fight the drift. */

    /* ── RENDERING / UI latches (never affect physics) ───────────── */
    bool   show_ghost;
        /* g — draw the 5-ghost Lyapunov shower under the main
         * trail.  Off → only the main trail and the ε-sep number
         * indicator are visible.                                  */

    bool   show_lobe;
        /* l — LOBE MODE.  When on, main trail colour is keyed by
         * sign(x): right wing → CP_TRAIL_HEAD, left wing →
         * CP_GHOST.  Reveals the chaotic wing-switches as vivid
         * hue flips — the Lorenz bistability [3] made visceral.
         * Age still controls A_BOLD / NORMAL / DIM intensity.    */

    bool   show_density;
        /* h — DENSITY MODE.  Replaces the trail with a 2-D
         * screen-space heatmap accumulating trail-cell visits.
         * Reveals the attractor's invariant measure [3] — where
         * the trajectory actually spends its time.  Auto-resets
         * on rotation (each angle re-accumulates).               */

    bool   paused;
        /* space — physics freeze.  lorenz_tick early-returns; the
         * renderer keeps painting (so the user can study a frozen
         * state).  View rotation also halts (no auto_rotate while
         * paused, since rotation lives in lorenz_tick).          */
} Lorenz;

/* ── lorenz_init step-helpers ───────────────────────────────────────── *
 *
 * Initialisation is four named steps: seed the main trajectory, seed
 * the ghost shower, reset the view, reset the render toggles.        */

/* (1) SEED MAIN TRAJECTORY at (1,1,1) — approximately on the attractor,
 *     so the trail joins the butterfly almost immediately rather than
 *     having a long transient flying-into-frame. */
static void seed_main_trajectory(Lorenz *l)
{
    l->mx = 1.0f;  l->my = 1.0f;  l->mz = 1.0f;
    trail_clear(&l->mt);
    trail_push(&l->mt, l->mx, l->my, l->mz);
}

/* (2) SEED GHOST SHOWER — N_GHOSTS trajectories at (1 + ε_g, 1, 1)
 *     where ε_g comes from GHOST_EPS_TABLE.  All ghosts share an
 *     initial y/z; only x differs by ε.  After integration each
 *     ghost diverges exponentially at rate λ ≈ 0.9 [7]. */
static void seed_ghost_shower(Lorenz *l)
{
    for (int g = 0; g < N_GHOSTS; g++) {
        l->gx[g] = 1.0f + GHOST_EPS_TABLE[g];
        l->gy[g] = 1.0f;
        l->gz[g] = 1.0f;
        trail_clear(&l->gt[g]);
        trail_push(&l->gt[g], l->gx[g], l->gy[g], l->gz[g]);
    }
}

/* (3) RESET VIEW DEFAULTS — camera angles + auto-rotate. */
static void reset_view_defaults(Lorenz *l)
{
    l->phi         = VIEW_PHI_DEFAULT;
    l->theta       = VIEW_THETA_DEFAULT;
    l->auto_rotate = true;
}

/* (4) RESET RENDER TOGGLES — show_ghost on (the multi-ghost shower
 *     is the default visual signature); lobe + density off (those are
 *     alternate views user must opt into); paused off. */
static void reset_render_toggles(Lorenz *l)
{
    l->show_ghost   = true;
    l->show_lobe    = false;
    l->show_density = false;
    l->paused       = false;
}

/*
 * lorenz_init — four-step recipe; body reads as the recipe.
 */
static void lorenz_init(Lorenz *l)
{
    seed_main_trajectory(l);    /* (1) main at (1,1,1)                 */
    seed_ghost_shower(l);       /* (2) N_GHOSTS at (1 + ε_g, 1, 1)     */
    reset_view_defaults(l);     /* (3) phi / theta / auto_rotate       */
    reset_render_toggles(l);    /* (4) show_ghost / lobe / density …   */
}

static void lorenz_tick(Lorenz *l, float dt)
{
    if (l->paused) return;

    if (l->auto_rotate)
        l->phi += VIEW_PHI_SPEED * dt;

    for (int s = 0; s < SUB_STEPS; s++) {
        lorenz_rk4(&l->mx, &l->my, &l->mz, L_H);
        trail_push(&l->mt, l->mx, l->my, l->mz);
        for (int g = 0; g < N_GHOSTS; g++) {
            lorenz_rk4(&l->gx[g], &l->gy[g], &l->gz[g], L_H);
            trail_push(&l->gt[g], l->gx[g], l->gy[g], l->gz[g]);
        }
    }
}

/* ── BACKGROUND STARFIELD ────────────────────────────────────────────── *
 *
 * ~60 fixed 3-D star positions scattered in a box around the attractor.
 * They project + paint each frame BEFORE the trail, so the attractor
 * appears in front of an ambient star backdrop.  Because the stars
 * rotate with the same azimuth as the lattice, you get parallax — the
 * stars sweep behind the attractor as the view spins (a strong depth
 * cue, even without proper z-buffering).                                */

#define N_STARS 60

static float g_star_x[N_STARS];
static float g_star_y[N_STARS];
static float g_star_z[N_STARS];
static bool  g_stars_initialised = false;

/* Tiny LCG for star positions — avoids touching the main srand() state
 * so star placement stays reproducible regardless of when init runs. */
static unsigned int stars_lcg_next(unsigned int *s)
{
    *s = (*s) * 1103515245u + 12345u;
    return (*s) >> 16;
}

static void stars_init(void)
{
    if (g_stars_initialised) return;
    unsigned int s = 987654321u;
    for (int i = 0; i < N_STARS; i++) {
        g_star_x[i] = ((float)(stars_lcg_next(&s) & 0xFFFF) / 65535.0f - 0.5f) * 80.0f;
        g_star_y[i] = ((float)(stars_lcg_next(&s) & 0xFFFF) / 65535.0f - 0.5f) * 80.0f;
        g_star_z[i] = ((float)(stars_lcg_next(&s) & 0xFFFF) / 65535.0f)        * 70.0f - 10.0f;
    }
    g_stars_initialised = true;
}

static void stars_draw(float phi, float theta, float scale,
                       int cx, int cy, int cols, int rows, WINDOW *w)
{
    if (!g_stars_initialised) stars_init();
    for (int i = 0; i < N_STARS; i++) {
        int col, row;
        float depth;
        if (!project(g_star_x[i], g_star_y[i], g_star_z[i],
                     phi, theta, scale, cx, cy, cols, rows,
                     &col, &row, &depth))
            continue;
        /* Closer stars get A_NORMAL, far stars A_DIM — even the
         * background has a faint depth signal. */
        attr_t at = (depth < -10.0f) ? A_NORMAL : A_DIM;
        wattron(w, COLOR_PAIR(CP_TRAIL_TAIL) | at);
        mvwaddch(w, row, col, '.');
        wattroff(w, COLOR_PAIR(CP_TRAIL_TAIL) | at);
    }
}

/* ── EQUILIBRIUM (FIXED-POINT) MARKERS ───────────────────────────────── *
 *
 * The Lorenz system has 3 fixed points where d/dt = 0:
 *
 *   origin (0, 0, 0)        — unstable saddle for ρ > 1
 *   C+ = (+√(β(ρ−1)), +√(β(ρ−1)), ρ−1) = (+√72, +√72, 27)
 *   C- = (−√72, −√72, 27)
 *
 * For ρ ≈ 28 (our default) C± are unstable spirals — the two lobes of
 * the strange attractor each orbit AROUND one of them.  Marking the
 * fixed points shows the "skeleton" the chaotic trajectory wraps.    */

#define FP_C_XY 8.485281f   /* √72 */
#define FP_C_Z  27.0f

static const float FIXED_POINTS[3][3] = {
    {   0.0f,      0.0f,      0.0f  },   /* origin */
    {  FP_C_XY,   FP_C_XY,   FP_C_Z },   /* C+     */
    { -FP_C_XY,  -FP_C_XY,   FP_C_Z },   /* C-     */
};

static void equilibria_draw(float phi, float theta, float scale,
                            int cx, int cy, int cols, int rows, WINDOW *w)
{
    for (int i = 0; i < 3; i++) {
        int col, row;
        if (!project(FIXED_POINTS[i][0], FIXED_POINTS[i][1], FIXED_POINTS[i][2],
                     phi, theta, scale, cx, cy, cols, rows,
                     &col, &row, NULL))
            continue;
        wattron(w, COLOR_PAIR(CP_TRAIL_MID) | A_BOLD);
        mvwaddch(w, row, col, '+');
        wattroff(w, COLOR_PAIR(CP_TRAIL_MID) | A_BOLD);
    }
}

/* ── DENSITY HEATMAP (alternate view) ────────────────────────────────── *
 *
 * 2-D screen-space accumulator: for each rendered frame (when the
 * density toggle is on) we project every trail point and increment a
 * per-cell counter.  Cells coloured by tier:
 *
 *   1..3       → CP_TRAIL_TAIL  A_DIM      '.'
 *   4..15      → CP_TRAIL_MID   A_NORMAL   '*'
 *   16+        → CP_TRAIL_HEAD  A_BOLD     '#'
 *
 * Density is in SCREEN coordinates, so it's valid only for a single
 * view.  When the view changes (rotation or elevation tilt) we reset
 * — gives the user a fresh accumulation at the new angle.            */

#define DENSITY_MAX_COLS 600
#define DENSITY_MAX_ROWS 200

static uint8_t g_density[DENSITY_MAX_ROWS][DENSITY_MAX_COLS];
static float   g_density_view_phi   = -999.0f;
static float   g_density_view_theta = -999.0f;

static void density_reset(void)
{
    memset(g_density, 0, sizeof g_density);
    g_density_view_phi   = -999.0f;
    g_density_view_theta = -999.0f;
}

/* (a) View-change INVALIDATION — density is in SCREEN coords, so the
 *     accumulator is only valid for one (phi, theta).  If the camera
 *     has rotated past a small threshold (~3°), reset the buffer so
 *     the user sees a fresh accumulation at the new angle.        */
static void density_invalidate_on_view_change(float phi, float theta)
{
    if (fabsf(phi   - g_density_view_phi)   > 0.05f ||
        fabsf(theta - g_density_view_theta) > 0.05f) {
        density_reset();
        g_density_view_phi   = phi;
        g_density_view_theta = theta;
    }
}

/* (b) Saturating BIN INCREMENT for one (col, row).  Bounds-checks the
 *     buffer extents and stops counting at 255 (uint8 saturation) —
 *     long sessions don't wrap to zero. */
static void density_bin_sample(int col, int row)
{
    if (col < 0 || col >= DENSITY_MAX_COLS) return;
    if (row < 0 || row >= DENSITY_MAX_ROWS) return;
    if (g_density[row][col] < 255) g_density[row][col]++;
}

/*
 * density_accumulate — invalidate-on-view-change + walk the trail,
 * project each sample, bin it.  Body reads as two steps + a project loop.
 */
static void density_accumulate(const Trail *t, float phi, float theta,
                               float scale, int cx, int cy,
                               int cols, int rows)
{
    density_invalidate_on_view_change(phi, theta);

    for (int k = 0; k < t->count; k++) {
        int idx = (t->head - k + TRAIL_LEN) % TRAIL_LEN;
        int col, row;
        if (!project(t->x[idx], t->y[idx], t->z[idx],
                     phi, theta, scale, cx, cy, cols, rows,
                     &col, &row, NULL))
            continue;
        density_bin_sample(col, row);
    }
}

static void density_draw(WINDOW *w, int cols, int rows)
{
    int rmax = rows < DENSITY_MAX_ROWS ? rows - 1 : DENSITY_MAX_ROWS;
    int cmax = cols < DENSITY_MAX_COLS ? cols     : DENSITY_MAX_COLS;
    for (int r = 1; r < rmax; r++) {
        for (int c = 0; c < cmax; c++) {
            uint8_t n = g_density[r][c];
            if (n == 0) continue;
            short  cp; attr_t at; char ch;
            if      (n >= 16) { cp = CP_TRAIL_HEAD; at = A_BOLD;   ch = '#'; }
            else if (n >=  4) { cp = CP_TRAIL_MID;  at = A_NORMAL; ch = '*'; }
            else              { cp = CP_TRAIL_TAIL; at = A_DIM;    ch = '.'; }
            wattron(w, COLOR_PAIR(cp) | at);
            mvwaddch(w, r, c, ch);
            wattroff(w, COLOR_PAIR(cp) | at);
        }
    }
}

/* ── lorenz_draw_trail step-helpers ─────────────────────────────────── *
 *
 * Painting one trail point is four orthogonal decisions:
 *
 *   COLOUR tier (which hue from the active theme):
 *     lobe-mode OFF → age tier:  head (newest) / mid / tail (oldest)
 *     lobe-mode ON  → wing sign: sign(x) > 0 head, else ghost colour
 *
 *   ATTRIBUTE tier (DIM / NORMAL / BOLD intensity):
 *     depth tier:  closer to camera → BOLD, mid → NORMAL, far → DIM
 *     OVERRIDE 1:  newest 2 points always A_BOLD (comet head)
 *     OVERRIDE 2:  oldest 20% always A_DIM (fading-memory tail)
 *
 *   GLYPH:
 *     head marker on k=0 ('O' main / 'x' ghost); else '.' or ','
 *
 *   BLOOM halo (main trail only, newest 2):
 *     4-cross '+' painted in theme head colour AROUND the head cell;
 *     visible perpendicular to motion (parallel cells get overdrawn
 *     by subsequent tail samples) → soft comet-head glow.
 *
 * Each gets a named helper so the inner loop reads as the four steps. */

/* COLOUR tier — pick a theme colour pair for this sample. */
static inline short trail_sample_color_pair(bool show_lobe, float lx,
                                              float age)
{
    if (show_lobe)         return (lx > 0.0f) ? CP_TRAIL_HEAD : CP_GHOST;
    if (age < 0.25f)       return CP_TRAIL_HEAD;
    if (age < 0.60f)       return CP_TRAIL_MID;
    return                        CP_TRAIL_TAIL;
}

/* ATTRIBUTE tier — pick A_DIM / A_NORMAL / A_BOLD from depth, with the
 * two age-extreme overrides for the comet head and fading tail. */
static inline attr_t trail_sample_depth_attr(float depth, int k, float age)
{
    attr_t at;
    if      (depth < -10.0f) at = A_BOLD;    /* close to camera */
    else if (depth >  10.0f) at = A_DIM;     /* far behind      */
    else                     at = A_NORMAL;
    if      (k   < 2)        at = A_BOLD;    /* comet head      */
    else if (age > 0.80f)    at = A_DIM;     /* fading tail     */
    return at;
}

/* GLYPH — head marker (k=0) or trail dot.  Ghosts use ',' / 'x'. */
static inline char trail_sample_glyph(bool is_ghost, int k)
{
    if (k == 0) return is_ghost ? 'x' : 'O';
    return       is_ghost ? ',' : '.';
}

/* 4-cross BLOOM HALO around the comet head — perpendicular '+' marks
 * give the head a wide, glowing footprint without obscuring the trail. */
static inline void paint_bloom_halo(int row, int col, int cols, int rows,
                                     WINDOW *w)
{
    static const int hdr[4] = { -1,  1,  0,  0 };
    static const int hdc[4] = {  0,  0, -1,  1 };
    wattron(w, COLOR_PAIR(CP_TRAIL_HEAD));
    for (int hi = 0; hi < 4; hi++) {
        int br = row + hdr[hi], bc = col + hdc[hi];
        if (br >= 1 && br < rows - 1 && bc >= 0 && bc < cols)
            mvwaddch(w, br, bc, '+');
    }
    wattroff(w, COLOR_PAIR(CP_TRAIL_HEAD));
}

/*
 * lorenz_draw_trail — walk one trail newest → oldest, project + paint
 * each sample with the four named encodings above, plus bloom halo on
 * the newest two main-trail samples.  Ghost trails skip lobe mode and
 * bloom (always CP_GHOST + A_DIM with ',' glyph).
 */
static void lorenz_draw_trail(const Trail *t, bool is_ghost,
                              bool show_lobe,
                              float phi, float theta, float scale,
                              int cx, int cy, int cols, int rows,
                              WINDOW *w)
{
    int last_col = -999, last_row = -999;

    for (int k = 0; k < t->count; k++) {
        int idx = (t->head - k + TRAIL_LEN) % TRAIL_LEN;
        float age = (float)k / (float)(t->count > 1 ? t->count - 1 : 1);

        int col, row;
        float depth;
        if (!project(t->x[idx], t->y[idx], t->z[idx],
                     phi, theta, scale, cx, cy, cols, rows,
                     &col, &row, &depth))
            continue;

        /* skip duplicate cell to bound draw calls along the dense curve */
        if (col == last_col && row == last_row) continue;
        last_col = col;
        last_row = row;

        chtype attr;
        if (is_ghost) {
            attr = COLOR_PAIR(CP_GHOST) | A_DIM;
        } else {
            short  cp = trail_sample_color_pair(show_lobe, t->x[idx], age);
            attr_t at = trail_sample_depth_attr(depth, k, age);
            attr = COLOR_PAIR(cp) | at;
        }

        char ch = trail_sample_glyph(is_ghost, k);

        wattron(w, attr);
        mvwaddch(w, row, col, ch);
        wattroff(w, attr);

        if (!is_ghost && k < 2)
            paint_bloom_halo(row, col, cols, rows, w);
    }
}

/* ── lorenz_draw layer-orchestration helpers ────────────────────────── *
 *
 * lorenz_draw is a back-to-front layer stack: starfield → equilibrium
 * markers → trajectory (trails or density) → ε-sep text.  Each layer
 * gets a tiny named helper so the orchestrator reads top-to-bottom as
 * the visual recipe.                                                  */

/* Pick the lattice scale factor for the current terminal: fill ~80%
 * of the usable vertical band, accounting for the ASPECT non-square-
 * cell correction.  Max projected |sy| ≈ 39 at default elevation. */
static inline float compute_view_scale(int rows)
{
    float usable = (float)(rows - 4);
    return usable * 0.80f / (39.0f * ASPECT);
}

/* Layer 3a — TRAJECTORY view.  Draw all ghosts (widest-ε first so the
 * close ghosts overlay them), then the main trail with full
 * depth/lobe/bloom encoding on top. */
static void draw_trajectory_layer(const Lorenz *l, float scale,
                                   int cx, int cy, int cols, int rows,
                                   WINDOW *w)
{
    if (l->show_ghost) {
        for (int g = N_GHOSTS - 1; g >= 0; g--)
            lorenz_draw_trail(&l->gt[g], true, false,
                              l->phi, l->theta, scale,
                              cx, cy, cols, rows, w);
    }
    lorenz_draw_trail(&l->mt, false, l->show_lobe,
                      l->phi, l->theta, scale,
                      cx, cy, cols, rows, w);
}

/* Layer 3b — DENSITY view (alternate mode 'h').  Accumulate this
 * frame's main-trail samples into the screen-space heatmap, then
 * paint the buffer.  Trails are NOT drawn in this mode. */
static void draw_density_layer(const Lorenz *l, float scale,
                                int cx, int cy, int cols, int rows,
                                WINDOW *w)
{
    density_accumulate(&l->mt, l->phi, l->theta, scale,
                       cx, cy, cols, rows);
    density_draw(w, cols, rows);
}

/* Layer 4 — ε-SEP INDICATOR.  Inline text reporting the live distance
 * from the main trajectory to the smallest-ε ghost (g0).  g0 is the
 * most meaningful "shadow" — the wider-ε ghosts saturate at attractor
 * scale (~30) within a few seconds and no longer change. */
static void draw_sep_indicator(const Lorenz *l, WINDOW *w, int rows)
{
    float dx = l->gx[0] - l->mx;
    float dy = l->gy[0] - l->my;
    float dz = l->gz[0] - l->mz;
    float sep = sqrtf(dx*dx + dy*dy + dz*dz);

    wattron(w, COLOR_PAIR(CP_GHOST) | A_DIM);
    mvwprintw(w, rows - 3, 1, " ε-sep[g0=%.3f]: %.4f ",
              GHOST_EPS_TABLE[0], sep);
    wattroff(w, COLOR_PAIR(CP_GHOST) | A_DIM);
}

/*
 * lorenz_draw — back-to-front layer stack, body reads as the recipe:
 *
 *   1. starfield               (ambient background with parallax)
 *   2. equilibrium markers     (fixed-point '+' crosshairs)
 *   3. trajectory layer        (ghosts + main trail with depth/bloom)
 *      OR density layer        (replaces trail entirely)
 *   4. ε-sep inline indicator  (distance from main to smallest-ε ghost)
 */
static void lorenz_draw(const Lorenz *l, WINDOW *w, int cols, int rows)
{
    int   cx    = cols / 2;
    int   cy    = rows / 2;
    float scale = compute_view_scale(rows);

    stars_draw(l->phi, l->theta, scale, cx, cy, cols, rows, w);       /* 1 */
    equilibria_draw(l->phi, l->theta, scale, cx, cy, cols, rows, w);  /* 2 */

    if (l->show_density)
        draw_density_layer(l, scale, cx, cy, cols, rows, w);          /* 3b */
    else
        draw_trajectory_layer(l, scale, cx, cy, cols, rows, w);       /* 3a */

    draw_sep_indicator(l, w, rows);                                   /* 4 */
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene — thin wrapper around the Lorenz entity.
 *
 * The Scene currently holds a single Lorenz, which IS the visible
 * world: the Lorenz struct already contains the simulation half
 * (trajectories + trails), the view (azimuth + elevation), and the
 * render/UI latches (show_ghost / show_lobe / show_density / paused).
 *
 * WHY wrap it in a Scene then?  Forward cover.  If we ever need
 *   per-scene state that DOESN'T belong inside Lorenz — a particle
 *   tracer overlay, an annotation layer, a saved-state buffer for
 *   replay, a second attractor for comparison — it goes here as a
 *   new field without changing every function signature that
 *   already takes Scene*.  The scene_init / scene_tick / scene_draw
 *   contract is stable; adding to Scene leaves the call sites in
 *   §8 (app loop) untouched.
 *
 * LOCALITY: the simulation-vs-rendering separation lives INSIDE the
 * Lorenz struct (see its "SIMULATION HALF / VIEW / RENDERING/UI
 * latches" field groups in §5), not at the Scene level.  Scene is
 * currently a placeholder ready for future per-scene additions.
 *
 * The full per-frame pipeline:
 *   scene_init   — zero the wrapper + lorenz_init (place trajectory
 *                  at the seed point, build trails, set defaults).
 *   scene_tick   — advance physics by dt: lorenz_tick → SUB_STEPS
 *                  RK4 steps for main + every ghost, plus an auto-
 *                  rotation drift on phi.
 *   scene_draw   — paint the back-to-front layer stack (starfield →
 *                  equilibria → ghosts → main trail or density →
 *                  ε-sep text).  No-op when nothing visible has
 *                  changed (renderer still paints — caller compares).
 */
typedef struct {
    Lorenz lorenz;    /* the entire visible world — see §5 Lorenz struct */
} Scene;

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    lorenz_init(&s->lorenz);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)cols; (void)rows;
    lorenz_tick(&s->lorenz, dt);
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    lorenz_draw(&s->lorenz, w, cols, rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — terminal-geometry container.
 *
 * Distinct from anything in §5 Lorenz so terminal-size changes don't
 * leak into simulation state.  Recomputed by screen_init at startup
 * and by screen_resize on SIGWINCH; nothing else writes it.  The
 * value drives:
 *
 *   - the render-layer call sites (project takes cx = cols/2, cy =
 *     rows/2 and clips to [0, cols) × [1, rows-1])
 *   - the HUD layout in screen_draw (top row 0, bottom row rows-1,
 *     status string right-justified at cols - strlen(buf))
 *   - the trail's automatic scale: usable = rows-4, scale fills ~80%
 *
 * Holds no allocation — just a (cols, rows) pair.
 */
typedef struct {
    int cols, rows;    /* current terminal width × height in cells */
} Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw — canonical CLAUDE.md two-row HUD.
 *
 *   Row 0          : title (theme-coordinated CP_TRAIL_HEAD + A_BOLD)
 *                    left-justified; status block (CP_HUD + A_BOLD,
 *                    canonical bright yellow) right-justified with
 *                    fps / sim Hz / ghost / rotation / theme / paused.
 *   Row rows-1     : action keys, CP_HINT + A_BOLD (canonical cyan,
 *                    NEVER A_DIM — must stay legible against the
 *                    animated trail).
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Lorenz *l = &sc->lorenz;

    /* ── Top row 0 LEFT: title in active theme's head colour ───────── */
    attron(COLOR_PAIR(CP_TRAIL_HEAD) | A_BOLD);
    mvprintw(0, 1, " LORENZ ATTRACTOR ");
    attroff(COLOR_PAIR(CP_TRAIL_HEAD) | A_BOLD);

    /* ── Top row 0 RIGHT: status (canonical yellow + A_BOLD) ───────── */
    char buf[200];
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%3d Hz  ghost:%s  lobe:%s  density:%s  rot:%s"
             "  theme:%s  %s ",
             fps, sim_fps,
             l->show_ghost   ? "on " : "off",
             l->show_lobe    ? "on " : "off",
             l->show_density ? "on " : "off",
             l->auto_rotate  ? "auto"   : "manual",
             THEMES[g_theme_idx].name,
             l->paused ? "PAUSED " : "running");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* ── Bottom row rows-1: ACTION keys (canonical cyan + A_BOLD) ──── */
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset  g:ghost  l:lobe  h:density"
             "  a:rot  arrows:view  [/]:Hz  t/T:theme ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level lifecycle container; everything in one place.
 *
 * WHY a struct (and not loose globals): the App lifecycle is well-
 *   defined — born at main() init, dies at main() return.  Bundling
 *   lets a single App* propagate through the loop helpers
 *   (app_do_resize, app_handle_key) without a forest of unrelated
 *   globals.
 *
 * WHY sim_fps lives in App (not in Lorenz): it's a LOOP-LEVEL knob,
 *   not a physics constant.  The integrator step L_H is fixed in
 *   Lorenz-time; sim_fps controls how many sim ticks happen per
 *   real second of wall clock, decoupling the apparent animation
 *   speed from the integrator accuracy.  Adjusting with [/] changes
 *   how fast you watch the same exact trajectory unroll.
 *
 * WHY signal flags live inside App (with the volatile sig_atomic_t
 * dance):
 *
 *   The C signal-handler signature is `void handler(int)` — no place
 *   to pass an App* into it.  So we keep a SINGLE static App g_app
 *   instance at file scope; on_exit_signal / on_resize_signal write
 *   to its flags.
 *
 *   The flags themselves MUST be `volatile sig_atomic_t` because:
 *     - volatile      — the main-loop read can't be optimised away
 *                       (the compiler doesn't know a signal will
 *                       fire and mutate the variable from outside).
 *     - sig_atomic_t  — the C standard guarantees these reads/writes
 *                       are atomic w.r.t. signal interruption; other
 *                       integer types may not be.
 *
 *   Signal handlers ONLY SET FLAGS; the main loop reads them and
 *   does the real work.  Doing the work IN the handler is unsafe —
 *   endwin / ncurses calls / malloc / free are not signal-safe.
 *
 *     running       — clear (= 0) to break the main loop and exit
 *                     cleanly through atexit(cleanup).
 *     need_resize   — set on SIGWINCH; main loop services it by
 *                     re-fitting Screen to the new terminal size on
 *                     the next iteration.
 */
typedef struct {
    Scene                 scene;        /* §6 — the world container         */
    Screen                screen;       /* §7 — terminal geometry           */
    int                   sim_fps;      /* sim Hz (10..120, +/- via [/])    */
    volatile sig_atomic_t running;      /* SIGINT/SIGTERM → 0 → exit        */
    volatile sig_atomic_t need_resize;  /* SIGWINCH → 1 → resize next iter  */
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Lorenz *l = &app->scene.lorenz;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        l->paused = !l->paused;
        break;

    case 'r': case 'R':
        lorenz_init(l);
        density_reset();   /* fresh density accumulation after restart */
        break;

    case 'g': case 'G':
        l->show_ghost = !l->show_ghost;
        break;

    case 'a': case 'A':
        l->auto_rotate = !l->auto_rotate;
        break;

    case 'l': case 'L':
        l->show_lobe = !l->show_lobe;
        break;

    case 'h': case 'H':
        l->show_density = !l->show_density;
        if (l->show_density) density_reset();
        break;

    case KEY_LEFT:
        l->auto_rotate = false;
        l->phi -= 0.1f;
        break;
    case KEY_RIGHT:
        l->auto_rotate = false;
        l->phi += 0.1f;
        break;

    case KEY_UP:
        l->theta += 0.05f;
        if (l->theta > 1.4f) l->theta = 1.4f;
        break;
    case KEY_DOWN:
        l->theta -= 0.05f;
        if (l->theta < 0.1f) l->theta = 0.1f;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        g_theme_idx = (g_theme_idx + 1) % N_THEMES;
        theme_apply(g_theme_idx);
        break;
    case 'T':
        g_theme_idx = (g_theme_idx + N_THEMES - 1) % N_THEMES;
        theme_apply(g_theme_idx);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        float alpha = (float)sim_accum / (float)tick_ns;

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha, dt_sec);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
