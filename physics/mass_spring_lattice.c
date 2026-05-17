/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * physics/mass_spring_lattice.c — 2D Mass-Spring Lattice as a Stress Field
 *
 * Classical mass-spring lattice (Hooke's law springs + symplectic Euler
 * integration) rendered as a STRESS FIELD instead of a wireframe mesh:
 * springs at rest are INVISIBLE — only stressed springs glow.  When the
 * lattice is undisturbed the screen shows a faint dot grid (the
 * rest-state nodes); when an impulse strikes, waves of bright `-` `=`
 * `#` edges radiate outward, reflect off boundaries, interfere, and
 * fade as damping does its work.
 *
 * The demo is SELF-RUNNING: a "struck membrane" showcase cycles through
 * four impulse patterns (CENTER, CORNER, TWO-POINT, LINE) on a ~12-second
 * loop.  No menu, no cursor — sit and watch the wave physics.
 *
 * ═════════════════════════════════════════════════════════════════════
 *  WHAT YOU ARE SEEING ON SCREEN
 * ═════════════════════════════════════════════════════════════════════
 *
 *  ┌───────────────────────────────────────────────────────────────┐
 *  │ MassSpring  scenario:Center  theme:Classic  k=60  E=…  60fps  │ ← row 0  HUD STATUS (bright yellow + bold)
 *  │                                                               │
 *  │   . . . . . . . . . . . . . . . . . . . . . .                 │ ← faint '.' = rest-state NODES.  Every dot is a
 *  │   . . . . . . . . . . . . . . . . . . . . . .                 │   point mass at its anchor position; they form
 *  │   . . . . #=#=*           . . . . . . . . . .                 │   the LATTICE you'd see if everything were quiet.
 *  │   . . #=#=O O=#-          . . . . . . . . . .                 │
 *  │   . . *=#=*=#-            . . . . . . . . . .                 │ ← bright glyphs = STRESSED SPRINGS.
 *  │   . . . . . . . . . . . . . . . . . . . . . .                 │     -  |   mild stress   (dim colour)
 *  │   . . . . . . . . . . . . . . . . . . . . . .                 │     =  H   medium stress (bright colour)
 *  │   . . . . . . . . . . . . . . . . . . . . . .                 │     #      EXTREME stress (bright + bold)
 *  │                                                               │   COLOUR encodes sign:
 *  │ q:quit  spc:pause  r:reset  n:next  t/T:theme  k/K:stiff …    │ ← row n-1  CYAN ↘ compressed (springs pushed in)
 *  └───────────────────────────────────────────────────────────────┘   YELLOW/RED ↗ stretched (springs pulled out)
 *                                                                     bright cyan + bold
 *
 *   MOVING NODES — overlay the rest-grid dots:
 *     .    rest    (speed < 0.6 cell/s)   — almost still
 *     o    slow    (0.6 .. 5 cell/s)      — oscillating
 *     O    fast    (> 5 cell/s)           — just struck, peak energy (bold)
 *
 *   READING THE PATTERNS:
 *     • A wave front travels at the lattice's natural speed
 *           c ≈ √(k/m) · spacing      (here ~ √60 · 4 ≈ 31 cell/s)
 *       so on a ~30-cell-wide lattice the first front crosses in ~1 s.
 *     • CENTER strike    → an expanding RING; the bright halo is the
 *                          wave front, the dim trail behind is residual
 *                          oscillation that damping is still draining.
 *     • CORNER strike    → a QUARTER-CIRCLE that reflects off the two
 *                          near edges, producing herringbone interference.
 *     • TWO-POINT strike → two rings collide near the midline; bright
 *                          bands appear where they ADD (constructive
 *                          interference), gaps where they CANCEL.
 *     • LINE strike      → a PLANAR wave marching across — the closest
 *                          thing to a 1-D textbook wave on this 2-D grid.
 *     • Watch the HUD's  E  (total kinetic + potential energy) — it
 *       falls monotonically as damping converts motion to heat (which
 *       this simulation does not model, only the loss).
 *
 * ═════════════════════════════════════════════════════════════════════
 *  SECTION MAP
 * ═════════════════════════════════════════════════════════════════════
 *   §1  config       — tunable constants (lattice size, k, damping…)
 *   §2  clock        — monotonic ns clock + sleep
 *   §3  color/theme  — stress + node palette (per CLAUDE.md HUD standard)
 *   §4  lattice      — Node / Spring structs + BSS arrays
 *   §5  physics      — Hooke's law forces + symplectic Euler
 *   §6  scenarios    — strike patterns + auto-advance
 *   §7  render       — stress-glow paint pipeline
 *   §8  main         — fixed-timestep sim loop + HUD + signals
 *
 * Keys:
 *   q / ESC      quit                  space / p    pause / resume
 *   r            re-trigger scenario   n            next scenario
 *   t / T        next / previous theme
 *   k / K        softer / stiffer springs
 *   d / D        less / more damping
 *
 * ═════════════════════════════════════════════════════════════════════
 *  REFERENCES   (cite inline as [n])
 * ═════════════════════════════════════════════════════════════════════
 *
 *  ── Physics: elasticity, mass-spring & cloth ───────────────────────
 *
 *   [1] Hooke, R. (1678) — *De Potentia Restitutiva, or of Spring*.
 *       Royal Society of London.  The HISTORICAL primary publication
 *       of "ut tensio, sic vis" (F = −k·x) — the single equation that
 *       every spring in this lattice obeys.  Cited for provenance.
 *
 *   [2] Witkin, A.; Baraff, D. (2001) — "Physically Based Modeling:
 *       Principles and Practice", SIGGRAPH Course Notes, ACM.  THE
 *       standard practitioner reference for mass-spring systems,
 *       constrained dynamics, and the cloth simulation pipeline.
 *       Read this first if you want to build off the §5 solver.
 *
 *   [3] Provot, X. (1995) — "Deformation Constraints in a Mass-Spring
 *       Model to Describe Rigid Cloth Behaviour", *Graphics Interface
 *       '95*, 147-154.  Introduced the H/V structural + diagonal shear
 *       + bend spring taxonomy that nearly all subsequent cloth /
 *       lattice work has used.  This demo deliberately drops shear
 *       springs for visual clarity — Provot explains what you lose.
 *
 *  ── Numerical integration ──────────────────────────────────────────
 *
 *   [4] Hairer, E.; Lubich, C.; Wanner, G. (2006) — *Geometric
 *       Numerical Integration*, 2nd ed., Springer.  Chapters I-VI
 *       cover SYMPLECTIC INTEGRATORS — why our "velocity-first" Euler
 *       (§5 integrate_step) conserves a shadow Hamiltonian and
 *       therefore doesn't blow up over thousands of steps the way
 *       explicit Euler does.  The deeper read after a numerics 101.
 *
 *  ── Wave physics & vibrating membranes ─────────────────────────────
 *
 *   [5] Rayleigh, J. W. S. (1877) — *The Theory of Sound*, Vol. I,
 *       Macmillan, London.  Chapters IX-X derive the modes of
 *       vibrating membranes — the analytical counterpart to what
 *       you see in the CENTER (Bessel-like radial modes) and
 *       TWO-POINT (interference of two sources) scenarios.
 *
 *   [6] Crawford, F. S. (1968) — *Waves*, Berkeley Physics Course
 *       Vol. 3, McGraw-Hill.  The most accessible undergraduate
 *       treatment of waves on lattices, dispersion, reflection at
 *       boundaries, and standing-wave / travelling-wave duality —
 *       all of which appear plainly in this demo.
 *
 *  ── Visualisation & perception ─────────────────────────────────────
 *
 *   [7] Ware, C. (2020) — *Information Visualization: Perception for
 *       Design*, 4th ed., Morgan Kaufmann.  Ch.4 (colour, sequential
 *       maps) backs the LO→HI brightness ramp per stress sign; Ch.5
 *       (pre-attentive features) backs the "rest = invisible" choice
 *       that makes wave fronts pop instead of competing with a mesh.
 *
 *   [8] Tufte, E. R. (2001) — *The Visual Display of Quantitative
 *       Information*, 2nd ed., Graphics Press.  The "data-ink ratio"
 *       principle (minimise non-data ink) is the design philosophy
 *       behind dropping the old wireframe mesh entirely — every
 *       glowing character on screen now encodes a stress value.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/mass_spring_lattice.c \
 *       -o mass_spring -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────── *
 *
 * Algorithm     : Mass-spring lattice on a rectangular grid (Witkin &
 *                 Baraff [ref 2]).  Each node obeys Newton's 2nd law
 *                 (F = m·a); each spring contributes a Hooke restoring
 *                 force [ref 1] to its two endpoints.  Time integration
 *                 is SYMPLECTIC EULER [ref 4] — update velocity first
 *                 using current force, then update position using the
 *                 NEW velocity.  Costs the same as explicit Euler but
 *                 conserves a shadow Hamiltonian, so the lattice does
 *                 not drift in energy over thousands of steps.
 *
 * Physics       : Hooke's law per spring [ref 1]:
 *                     F = -k · (|d| - L₀) · d̂
 *                 acting equal-and-opposite on the two endpoints
 *                 (Newton's 3rd law).  Plus a global velocity damping
 *                 F_damp = -c·v that drains kinetic energy over time —
 *                 the reason a struck membrane eventually goes quiet.
 *                 This demo uses H+V structural springs only; Provot
 *                 [ref 3] adds diagonal shear + bend springs for cloth
 *                 rigidity, deliberately omitted here for visual clarity.
 *
 * Numerics      : Stability requires dt < 2/ω_max where ω_max ≈
 *                 √(N_neighbours·k/m).  With 4-neighbour (H+V)
 *                 connectivity, k=60, m=1: dt_crit ≈ 2·√(1/240) ≈
 *                 0.13 s — our 1/120 s sub-step is well inside.
 *                 Wave propagation speed on the lattice is c ≈
 *                 √(k/m)·spacing (Crawford [ref 6], waves on lattices).
 *
 * Visualisation : STRESS-FIELD GLOW — rest-length springs (|σ| < 0.04)
 *                 are not drawn at all (Tufte data-ink minimisation
 *                 [ref 8]).  Strain σ = (|d|-L₀)/L₀ maps to a glyph +
 *                 colour tier:
 *
 *                     |σ| tier     glyph          colour
 *                     ──────────   ───────────    ──────────────────
 *                     [.04, .15)   - or |         dim   (cyan/yellow)
 *                     [.15, .35)   = or H         bright (cyan/red)
 *                     ≥ .35        # (any axis)   bright + A_BOLD
 *
 *                     σ < 0 → COOL ramp (compressed) — cyan family
 *                     σ > 0 → HOT  ramp (stretched)  — yellow → red
 *
 *                 The signed sequential colour mapping (Ware [ref 7],
 *                 Ch.4) makes compression vs tension instantly readable;
 *                 the dropout for low stress exploits pre-attentive
 *                 luminance contrast (Ware [ref 7], Ch.5) so wave fronts
 *                 POP against an empty background instead of competing
 *                 with a static mesh.
 *
 * Showcase      : "Struck membrane" — four impulse patterns cycle on a
 *                 ~12-s timer (CENTER strike, CORNER strike, TWO-POINT
 *                 interference, LINE strike).  Each one produces a
 *                 classical vibrating-membrane mode (Rayleigh [ref 5]):
 *                 radial Bessel-like waves, edge reflections, two-source
 *                 interference fringes, and planar travelling waves.
 *
 * ─────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ════════════════════════════════════════════════════════════════════
 * §1  CONFIG
 * ════════════════════════════════════════════════════════════════════ */

enum {
    TARGET_FPS      = 60,
    SUBSTEPS        = 2,        /* physics sub-steps per render frame */

    NODE_DX         = 4,        /* cell columns between adjacent nodes */
    NODE_DY         = 2,        /* cell rows between adjacent nodes */
    MAX_NX          = 40,
    MAX_NY          = 18,
    MAX_NODES       = MAX_NX * MAX_NY,
    MAX_SPRINGS     = MAX_NX * MAX_NY * 2,

    SCENARIO_COUNT  = 4,
    SCENARIO_TICKS  = 12 * TARGET_FPS,   /* ~12 s per scenario */

    N_THEMES        = 3,
};

#define MASS_DEF        1.0f
#define K_DEF           60.0f
#define DAMPING_DEF     1.5f
#define IMPULSE_VEL     14.0f
#define HAMMER_VEL      30.0f

#define K_STEP          5.0f
#define K_MIN           10.0f
#define K_MAX           300.0f
#define DAMPING_STEP    0.25f
#define DAMPING_MIN     0.0f
#define DAMPING_MAX     8.0f

/* Strain magnitude thresholds — boundaries between visual tiers */
#define STRAIN_NULL     0.04f   /* below → spring not drawn (THE GLOW) */
#define STRAIN_MID      0.15f   /* low → mid tier                       */
#define STRAIN_HIGH     0.35f   /* mid → high tier                      */

/* Speed thresholds — boundaries between node visual tiers */
#define SPEED_REST      0.6f
#define SPEED_FAST      5.0f

#define HUD_TOP_ROWS    1
#define HUD_BOT_ROWS    1

/* ════════════════════════════════════════════════════════════════════
 * §2  CLOCK
 * ════════════════════════════════════════════════════════════════════ */

#define NS_PER_SEC      1000000000LL
#define TICK_NS(fps)    (NS_PER_SEC / (fps))

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

/* ════════════════════════════════════════════════════════════════════
 * §3  COLOR / THEME
 * ════════════════════════════════════════════════════════════════════ *
 *
 * Two CANONICAL pairs (CLAUDE.md HUD Standard): CP_HUD bright yellow,
 * CP_HINT bright cyan, both A_BOLD.  The rest is theme-dependent:
 *   COMPRESS_LO/HI — cool ramp for σ < 0
 *   TENSION_LO/HI  — hot  ramp for σ > 0
 *   NODE_REST      — dim grey, the rest-state lattice dots
 *   NODE_SLOW/FAST — moving-node tiers
 *   NODE_PIN       — bold anchor (unused in default scenarios)
 */

enum {
    CP_HUD         = 1,
    CP_HINT        = 2,
    CP_COMPRESS_LO = 3,
    CP_COMPRESS_HI = 4,
    CP_TENSION_LO  = 5,
    CP_TENSION_HI  = 6,
    CP_NODE_REST   = 7,
    CP_NODE_SLOW   = 8,
    CP_NODE_FAST   = 9,
    CP_NODE_PIN    = 10,
};

/*
 * Theme — one named visual identity bundling every theme-dependent
 *         colour-pair value as a tightly correlated set.
 *
 * WHY a struct (not loose const arrays):
 *   The four stress colours (compress LO, compress HI, tension LO,
 *   tension HI) must form TWO perceptually monotone sequential ramps
 *   — one cool, one hot — so that brightness encodes |strain|
 *   consistently within each sign (Ware [ref 7], Ch.4 sequential
 *   colour maps).  Bundling them together prevents accidentally
 *   pairing compress LO of theme A with compress HI of theme B at
 *   edit time.  Same argument for the three node tiers — rest/slow/
 *   fast must share a luminance ordering, not just an arbitrary
 *   palette.
 *
 * WHY DUAL palettes (256 + 8):
 *   Modern terminals expose 256 indexed colours; legacy TTYs and
 *   minimal $TERM values ("linux", "dumb") expose only 8.
 *   theme_apply() inspects ncurses COLORS at runtime and binds the
 *   matching set — no #ifdef ladder.  The 8-colour fallback is a
 *   coarser approximation of the same theme INTENT (cool / hot ramps,
 *   bright accent for nodes), preserving sign discrimination even when
 *   per-tier luminance can't be expressed.
 *
 * WHY CP_HUD / CP_HINT live OUTSIDE this struct:
 *   The CLAUDE.md HUD Standard fixes them at bright yellow (226) +
 *   bright cyan (51) GLOBALLY — they must stay legible against any
 *   stress palette.  Putting them in a per-theme struct would invite
 *   drift toward dim or coloured HUDs that disappear against active
 *   animation.  color_init() binds them once at startup; theme_apply()
 *   never touches them.
 *
 * WHY CP_NODE_PIN is also outside:
 *   The pinned-node marker is bold WHITE-ON-BLUE everywhere — it's a
 *   "structural anchor" indicator that must read identically across
 *   every theme so a viewer can spot anchors at a glance.  Hard-coded
 *   in theme_apply() rather than per-theme.
 *
 * Brightness floor:  Every 256-colour entry sits in the bright half
 *   of the cube (CLAUDE.md Theme Palette Brightness rule — colours
 *   16-23 / 232-239 become invisible against default-black + A_DIM).
 */
typedef struct {
    /* ── 256-colour palette (preferred when COLORS >= 256) ────────── *
     * Pairs are listed in visual order: the COOL ramp encodes
     * compression (σ<0), the HOT ramp encodes tension (σ>0).  Each
     * ramp goes LO → HI as |σ| crosses STRAIN_MID into STRAIN_HIGH. */
    short compress_lo;   /* CP_COMPRESS_LO — mild compression, |σ|∈[.04,.15) */
    short compress_hi;   /* CP_COMPRESS_HI — strong/extreme compression      */
    short tension_lo;    /* CP_TENSION_LO  — mild tension                    */
    short tension_hi;    /* CP_TENSION_HI  — strong/extreme tension          */

    /* Node luminance tiers — must form a monotone ramp REST → FAST so
     * the viewer reads "this node is moving" as "this node is brighter
     * than its neighbours" without consulting a legend. */
    short node_rest;     /* CP_NODE_REST — speed < SPEED_REST, the dot grid */
    short node_slow;     /* CP_NODE_SLOW — SPEED_REST ≤ speed < SPEED_FAST  */
    short node_fast;     /* CP_NODE_FAST — speed ≥ SPEED_FAST (impact peak) */

    /* ── 8-colour ANSI fallback ──────────────────────────────────── *
     * Same semantic roles, coarser granularity.  Within each ramp the
     * fallback typically collapses LO and HI onto a single colour
     * (sign discrimination preserved, intensity discrimination not). */
    short compress_lo8, compress_hi8;
    short tension_lo8,  tension_hi8;
    short node_rest8, node_slow8, node_fast8;

    const char *name;    /* shown in the HUD's `theme:` field         */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* Classic — cyan compression, red tension */
    { 51, 195,   220, 196,   245, 46, 207,
      COLOR_CYAN, COLOR_WHITE, COLOR_YELLOW, COLOR_RED,
      COLOR_WHITE, COLOR_GREEN, COLOR_MAGENTA,
      "Classic" },
    /* Cold — blue/cyan compression, white-hot tension */
    { 39, 87,    255, 226,   244, 51, 213,
      COLOR_BLUE, COLOR_CYAN, COLOR_WHITE, COLOR_YELLOW,
      COLOR_WHITE, COLOR_CYAN, COLOR_MAGENTA,
      "Cold" },
    /* Plasma — magenta/violet compression, gold tension */
    { 99, 207,   214, 196,   246, 156, 213,
      COLOR_MAGENTA, COLOR_WHITE, COLOR_YELLOW, COLOR_RED,
      COLOR_WHITE, COLOR_GREEN, COLOR_MAGENTA,
      "Plasma" },
};

/* Bind the canonical HUD pairs once at startup — they NEVER change
 * across theme cycling (CLAUDE.md: dim/coloured HUDs disappear). */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(CP_HUD,  226, -1);
        init_pair(CP_HINT,  51, -1);
    } else {
        init_pair(CP_HUD,  COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN,   -1);
    }
}

static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    if (COLORS >= 256) {
        init_pair(CP_COMPRESS_LO, th->compress_lo, -1);
        init_pair(CP_COMPRESS_HI, th->compress_hi, -1);
        init_pair(CP_TENSION_LO,  th->tension_lo,  -1);
        init_pair(CP_TENSION_HI,  th->tension_hi,  -1);
        init_pair(CP_NODE_REST,   th->node_rest,   -1);
        init_pair(CP_NODE_SLOW,   th->node_slow,   -1);
        init_pair(CP_NODE_FAST,   th->node_fast,   -1);
    } else {
        init_pair(CP_COMPRESS_LO, th->compress_lo8, -1);
        init_pair(CP_COMPRESS_HI, th->compress_hi8, -1);
        init_pair(CP_TENSION_LO,  th->tension_lo8,  -1);
        init_pair(CP_TENSION_HI,  th->tension_hi8,  -1);
        init_pair(CP_NODE_REST,   th->node_rest8,   -1);
        init_pair(CP_NODE_SLOW,   th->node_slow8,   -1);
        init_pair(CP_NODE_FAST,   th->node_fast8,   -1);
    }
    init_pair(CP_NODE_PIN, COLOR_WHITE, COLOR_BLUE);
}

/* ════════════════════════════════════════════════════════════════════
 * §4  LATTICE — Node, Spring, and the Scene that owns them
 * ════════════════════════════════════════════════════════════════════ *
 *
 * The mass-spring system is captured in two primary record types
 * (Node, Spring) plus an axis enum (SpringKind), all gathered inside
 * the Scene struct further down.  The data model follows the standard
 * Witkin & Baraff [ref 2] particle-system layout: positions, velocities
 * and per-step force accumulators on each particle; sparse spring
 * records pointing at particle indices.
 */

/*
 * SpringKind — axis tag identifying whether a spring lies along the
 * horizontal or vertical lattice direction.
 *
 * WHY tag the axis at all (the endpoints already encode it):
 *   The RENDERER needs the axis at draw time to pick the right glyph
 *   ('-' for H, '|' for V) WITHOUT recomputing the angle of every
 *   spring every frame.  The PHYSICS doesn't care — Hooke's law treats
 *   every spring uniformly — but rendering's hot loop reads `kind` per
 *   spring per frame, so storing it once is cheaper than reading both
 *   endpoint positions to infer it.
 *
 * WHY only H and V (no diagonals):
 *   Provot [ref 3] introduces three spring families — STRUCTURAL (H/V),
 *   SHEAR (diagonal), BEND (skip-one).  Shear/bend springs are what
 *   make cloth feel stiff and what gives a lattice rigid-body modes.
 *   This demo intentionally drops them: with only H/V structurals the
 *   wave fronts are SHARPER and SIMPLER to read, which is the whole
 *   point of the stress-glow visualisation.  Restoring shear is a
 *   one-line addition to springs_build().
 */
typedef enum {
    SPRING_H = 0,    /* horizontal: dx = NODE_DX, dy = 0  →  glyph '-' */
    SPRING_V = 1,    /* vertical:   dx = 0, dy = NODE_DY  →  glyph '|' */
} SpringKind;

/*
 * Node — one point mass in the lattice.
 *
 * ALGORITHM context:
 *   Each node obeys Newton's 2nd law (F = m·a) with force accumulated
 *   from every spring touching it (Hooke [ref 1]) plus a linear damping
 *   term.  Time integration is symplectic Euler [ref 4]:
 *
 *       v_new = v + (f / m) · dt          ← uses CURRENT force
 *       x_new = x + v_new · dt            ← uses NEW velocity
 *
 *   Cheap (two scalar updates per axis) and energy-preserving on
 *   average — explicit Euler would inject phantom energy and the
 *   lattice would drift apart over time.
 *
 * WHY both rest (rx, ry) AND current (x, y) positions:
 *   • (x, y)   is the SIMULATION state — what the integrator updates,
 *               what every render pass reads to place the node glyph.
 *   • (rx, ry) is the LATTICE-ANCHOR position — the cell where this
 *               node would sit if everything were quiet.  Two uses:
 *                 - scenario_quiet() snaps (x, y) back to (rx, ry) when
 *                   re-triggering a scenario, so prior energy doesn't
 *                   bleed in to the next strike.
 *                 - rebuilding the lattice on resize: (rx, ry) tells
 *                   the renderer where the "rest dot" should be even
 *                   when the node is currently displaced mid-wave.
 *   Storing both costs 8 bytes per node; in exchange the integrator
 *   never recomputes rest positions and resets are O(N).
 *
 * WHY force accumulators (fx, fy) live in the struct, not on the stack:
 *   compute_forces() runs TWO passes over the data — pass 1 (springs)
 *   adds spring forces to each endpoint; pass 2 (damping) reads
 *   velocity, adds -c·v, and stores into the same accumulator.  Keeping
 *   fx/fy in the struct lets these two passes write to the same memory
 *   without an intermediate buffer.  Zeroed at the start of every
 *   compute_forces() call.
 *
 * Memory layout note:
 *   Position pair, velocity pair, force pair, rest pair — listed in
 *   the order the inner loops touch them, so a node fits in roughly
 *   one cache line and per-pass scans are prefetcher-friendly.
 */
typedef struct {
    float x,  y;     /* CURRENT position (cell coords).  Floats because the
                      * integrator advances in sub-cell steps. */
    float rx, ry;    /* REST / lattice-anchor position.  Set once in
                      * lattice_init(); never modified after.  Used by
                      * scenario_quiet() and rest-grid drawing. */
    float vx, vy;    /* velocity [cells/s].  Symplectic Euler updates this
                      * FIRST inside integrate_step() using f/m·dt. */
    float fx, fy;    /* force accumulator [N].  Cleared at the top of every
                      * compute_forces() call; receives spring contributions
                      * (Hooke [ref 1]) and damping (-c·v) in two passes. */
    bool  pinned;    /* if true: forces are still computed but integrate_step
                      * skips this node, so it acts as a fixed anchor.
                      * Not used in the default showcase scenarios. */
} Node;

/*
 * Spring — one Hooke connector linking two nodes (a, b).
 *
 * ALGORITHM context:
 *   Each spring contributes a force pair to compute_forces() per step:
 *
 *       d   = pos_b - pos_a              (displacement)
 *       L   = |d|                        (current length)
 *       F   = k · (L − L₀) · (d / L)     (Hooke [ref 1])
 *
 *       a.f +=  F                        (pulled toward b if extended)
 *       b.f += -F                        (Newton's 3rd law)
 *
 *   Pinned endpoints skip the accumulation but still participate in
 *   the strain calculation so the spring still glows visually.
 *
 * WHY store strain on the spring (not recompute at render time):
 *   The strain ratio σ = (L − L₀)/L₀ is the value the renderer needs
 *   to pick a colour tier + glyph.  compute_forces() already computes
 *   it for the force expression, so caching it on the spring costs
 *   nothing per step and saves the renderer a sqrt() per spring per
 *   frame.  The renderer is the ONLY reader; one writer, one reader,
 *   same struct = clear ownership.
 *
 * WHY store rest_len per spring (not as a global derived from kind):
 *   H springs have rest = NODE_DX, V springs have rest = NODE_DY —
 *   that's true today.  But pinning/breaking/anisotropy mods would
 *   want per-spring rest lengths, so the field is here from day one.
 *   Cost is 4 bytes per spring; future-proofing for free.
 *
 * Connectivity (a, b are INDICES into Scene.nodes[]):
 *   Indices, not pointers — the array is contiguous and re-allocated
 *   on resize; storing pointers would invalidate them on rebuild.
 *   Index order matters: a is the "lower" index (smaller row*nx+col)
 *   by springs_build() construction, so a < b always.  No code
 *   currently depends on this, but it's a convenient invariant.
 */
typedef struct {
    int        a, b;        /* node indices into Scene.nodes[] (a < b) */
    float      rest_len;    /* relaxed length L₀ [cells].  H: NODE_DX,
                             * V: NODE_DY (set in springs_build).      */
    float      strain;      /* σ = (|d| - L₀) / L₀ — signed strain.
                             * WRITTEN by compute_forces() each step;
                             * READ by render_spring() each frame.
                             * Sign convention: <0 = compressed, >0 = stretched. */
    SpringKind kind;        /* axis tag (H/V) — picks renderer glyph    */
} Spring;

/*
 * Scene — every piece of mutable state for one running session.
 *
 * Replaces what used to be a scattered set of file-scope globals
 * (g_nodes / g_springs / g_nn / g_ns / g_nx / g_ny / g_x0 / g_y0)
 * plus a pile of locals inside main() (k, damping, scenario, theme…).
 * Gathering them into ONE struct does three things:
 *
 *   1. Makes the data flow explicit — every function that needs scene
 *      state takes Scene* / const Scene*; no implicit "this function
 *      reads the lattice from globals" surprises.
 *   2. Documents ownership — each field belongs to exactly one of the
 *      four locality regions below, and the regions name who writes
 *      and who reads them.
 *   3. Makes the lifetime obvious — the WHOLE struct is rebuilt
 *      atomically on terminal resize or full reset; no partial state
 *      can leak between rebuilds.
 *
 * LAYOUT — fields are grouped into FOUR locality regions ordered the
 * way each pass touches them:
 *
 *   ┌─────────────────────────────────────────────────────────────────┐
 *   │ (A) PHYSICS DATA     — nodes[], springs[], counts                │
 *   │     read by  : compute_forces, integrate_step (HOT, per substep) │
 *   │     written  : compute_forces (forces), integrate_step (v, x),   │
 *   │                lattice_init / springs_build (init)               │
 *   │                                                                  │
 *   │ (B) GEOMETRY         — nx, ny, x0, y0, term_cols, term_rows      │
 *   │     read by  : render passes, scenario_apply                     │
 *   │     written  : lattice_init / fit_lattice — ONCE per (re)build   │
 *   │                                                                  │
 *   │ (C) SIM PARAMS       — k, damping, mass, dt, sim_fps, substeps   │
 *   │     read by  : compute_forces, integrate_step                    │
 *   │     written  : main()'s input handler (user knobs)               │
 *   │                                                                  │
 *   │ (D) ANIMATION + UI   — scenario, sc_tick, theme, paused,         │
 *   │                        fps_disp                                  │
 *   │     read by  : scene_tick, scene_draw, render_hud                │
 *   │     written  : main()'s input handler + per-frame counters       │
 *   └─────────────────────────────────────────────────────────────────┘
 *
 * WHY this grouping (locality + clarity):
 *   • Each per-frame pass touches a CONTIGUOUS region — prefetcher
 *     stays happy, and a reader scanning top-to-bottom sees the
 *     order in which the loop body reads each region.
 *   • PHYSICS hot loops (A+C) never touch UI (D); UI never touches
 *     force accumulators.  A bug in one region can't silently
 *     corrupt another.
 *   • GEOMETRY (B) is FROZEN between resizes, so any function in
 *     the per-frame critical path can hold (B) values in registers
 *     without re-reading.
 *
 * WHY ONE big struct (not split PhysicsScene + RenderScene + UIScene):
 *   On preset / theme / resize change the WHOLE scene rebuilds
 *   atomically.  Keeping it together makes the rebuild = "memset +
 *   refill" and makes every field's lifetime obvious (= lifetime of
 *   Scene itself).  Splitting would introduce a pointer hop, a second
 *   allocation site, and risk drift between the splits.
 *
 * Storage:  ~80 KB (dominated by nodes[] + springs[]).  Lives as a
 *   single file-scope BSS variable `g_scene` — zero malloc on the hot
 *   path (CLAUDE.md Memory rule).
 */
typedef struct {
    /* ── (A) PHYSICS DATA — input + state for compute_forces / integrate_step ─
     * Flat arrays.  nn / ns track how many entries are live;
     * indices past those are uninitialised garbage. */
    Node    nodes  [MAX_NODES];     /* point-mass states (pos, vel, force)    */
    Spring  springs[MAX_SPRINGS];   /* Hooke connectors with cached strain    */
    int     nn;                     /* live node   count (= nx * ny)          */
    int     ns;                     /* live spring count (set by springs_build) */

    /* ── (B) GEOMETRY — lattice topology + terminal anchor ─────── *
     * Set ONCE per (re)build by lattice_init() / fit_lattice() and
     * frozen until the next resize.  The renderer reads these every
     * frame to clip drawing to the visible band. */
    int     nx, ny;                 /* nodes per row, per column           */
    int     x0, y0;                 /* cell coords of the top-left REST node
                                     * (the renderer's anchor for the
                                     *  faint dot grid)                    */
    int     term_cols, term_rows;   /* current ncurses terminal dims        */

    /* ── (C) SIM PARAMS — physics knobs, user-tunable at runtime ─ *
     * compute_forces() + integrate_step() read these per substep.
     * Mutated by the keyboard handler (k/K, d/D) in main(); also
     * touched by scenario_apply() if a preset wants a different
     * damping/k for its visual.  Independent of region (A): changing
     * `k` doesn't rewrite the lattice, only the forces it generates. */
    float   k;                      /* Hooke spring constant [N/m, scaled] */
    float   damping;                /* linear velocity damping coefficient */
    float   mass;                   /* per-node mass (uniform lattice)     */
    float   dt;                     /* one physics substep duration [s]    */
    int     sim_fps;                /* render-frame cap (Hz)                */
    int     substeps;               /* physics substeps per render frame    */

    /* ── (D) ANIMATION + UI — showcase + HUD state ───────────────── *
     * Mutated by main()'s input handler and the per-frame counters;
     * read by scenario_apply, render_hud, and the auto-cycle logic.
     * No physics function reads these — they're presentation only. */
    int     scenario;               /* 0..SCENARIO_COUNT-1, active strike */
    int     sc_tick;                /* frames since this scenario started */
    int     theme;                  /* 0..N_THEMES-1, active palette       */
    bool    paused;                 /* freeze tick() but keep rendering    */
    int     fps_disp;               /* live FPS shown in the HUD (smoothed) */
} Scene;

/* THE single scene instance — file-scope BSS, no malloc.  Treated as
 * a parameter (Scene *) by every helper below; the symbol exists at
 * file scope only because signal handlers + main() need somewhere to
 * anchor it.                                                          */
static Scene g_scene;

/* ════════════════════════════════════════════════════════════════════
 * §5  PHYSICS
 * ════════════════════════════════════════════════════════════════════ */

/* CENTRE THE LATTICE — solve for the top-left cell (x0, y0) so the
 * (nx-1)·DX by (ny-1)·DY bounding box of the rest grid sits in the
 * middle of the renderable band (HUD rows excluded).  Clamps to a
 * minimum margin so a tiny terminal can't push the grid off-screen. */
static void compute_lattice_anchor(Scene *s, int nx, int ny)
{
    int lat_w = (nx - 1) * NODE_DX;
    int lat_h = (ny - 1) * NODE_DY;
    s->x0 = (s->term_cols - lat_w) / 2;
    s->y0 = HUD_TOP_ROWS
          + ((s->term_rows - HUD_TOP_ROWS - HUD_BOT_ROWS - lat_h) / 2);
    if (s->x0 < 0)             s->x0 = 0;
    if (s->y0 < HUD_TOP_ROWS)  s->y0 = HUD_TOP_ROWS;
}

/* STAMP ONE NODE at its lattice-anchor cell (x0 + c·DX, y0 + r·DY).
 * Current position equals rest position; velocity / force / pin all
 * zero — the standard "quiescent lattice" initial condition. */
static void stamp_rest_node(Node *n, int x0, int y0, int c, int r)
{
    n->rx = (float)(x0 + c * NODE_DX);
    n->ry = (float)(y0 + r * NODE_DY);
    n->x  = n->rx; n->y  = n->ry;
    n->vx = 0.0f;  n->vy = 0.0f;
    n->fx = 0.0f;  n->fy = 0.0f;
    n->pinned = false;
}

/* Initialise the lattice: pick the anchor, then stamp every node at
 * its rest position.  Spring topology is built separately
 * (springs_build) so a future variant can swap H+V for H+V+shear
 * without re-touching this function. */
static void lattice_init(Scene *s, int nx, int ny)
{
    s->nx = nx;
    s->ny = ny;
    s->nn = nx * ny;
    s->ns = 0;

    compute_lattice_anchor(s, nx, ny);

    for (int r = 0; r < ny; r++) {
        for (int c = 0; c < nx; c++) {
            stamp_rest_node(&s->nodes[r * nx + c], s->x0, s->y0, c, r);
        }
    }
}

/* Wire up the H+V structural-spring topology over the current node
 * grid.  Each interior cell contributes a right neighbour and a down
 * neighbour; boundaries contribute the ones that fit. */
static void springs_build(Scene *s)
{
    s->ns = 0;
    float h_rest = (float)NODE_DX;
    float v_rest = (float)NODE_DY;
    for (int r = 0; r < s->ny; r++) {
        for (int c = 0; c < s->nx; c++) {
            int i = r * s->nx + c;
            if (c + 1 < s->nx) {
                s->springs[s->ns++] =
                    (Spring){ i, i + 1, h_rest, 0.0f, SPRING_H };
            }
            if (r + 1 < s->ny) {
                s->springs[s->ns++] =
                    (Spring){ i, i + s->nx, v_rest, 0.0f, SPRING_V };
            }
        }
    }
}

/* CLEAR ACCUMULATORS — every force pass starts from zero so the two
 * physical contributions (spring + damping) can simply += in. */
static void clear_force_accumulators(Scene *s)
{
    for (int i = 0; i < s->nn; i++) {
        s->nodes[i].fx = 0.0f;
        s->nodes[i].fy = 0.0f;
    }
}

/* HOOKE'S LAW PAIR [ref 1] — for one spring, compute the restoring
 * force from its current extension and accumulate it on both endpoints
 * with opposite signs (Newton's 3rd law).  Also caches the signed
 * strain σ = (L − L₀)/L₀ on the spring so the renderer can read it.
 *
 *     d   = pos_b - pos_a
 *     L   = |d|
 *     F   = k · (L − L₀) · (d / L)
 *     a.f +=  F                          (pulled toward b if extended)
 *     b.f += -F
 *
 * Degenerate-length guard: if the two nodes have coincided exactly
 * (numerical artefact at blow-up), the direction is undefined — we
 * skip the force and just record zero strain. */
static void accumulate_hooke_pair(Scene *s, Spring *sp)
{
    Node *a = &s->nodes[sp->a];
    Node *b = &s->nodes[sp->b];
    float dx = b->x - a->x;
    float dy = b->y - a->y;
    float dist = sqrtf(dx*dx + dy*dy);
    if (dist < 1e-4f) { sp->strain = 0.0f; return; }

    sp->strain = (dist - sp->rest_len) / sp->rest_len;

    float inv_d   = 1.0f / dist;
    float stretch = dist - sp->rest_len;
    float fx = s->k * stretch * dx * inv_d;
    float fy = s->k * stretch * dy * inv_d;

    if (!a->pinned) { a->fx += fx; a->fy += fy; }
    if (!b->pinned) { b->fx -= fx; b->fy -= fy; }
}

/* SPRING-FORCE PASS — apply Hooke to every spring.  This is the
 * dominant cost of compute_forces (O(N_springs · constant)). */
static void apply_hooke_spring_forces(Scene *s)
{
    for (int i = 0; i < s->ns; i++) {
        accumulate_hooke_pair(s, &s->springs[i]);
    }
}

/* VELOCITY DAMPING PASS — Stokes-style linear drag: F_damp = -c·v.
 * Drains kinetic energy each step; the reason a struck membrane
 * eventually goes quiet.  No gravity in this showcase — free
 * membrane, isotropic damping is the only sink. */
static void apply_velocity_damping(Scene *s)
{
    for (int i = 0; i < s->nn; i++) {
        if (s->nodes[i].pinned) continue;
        s->nodes[i].fx -= s->damping * s->nodes[i].vx;
        s->nodes[i].fy -= s->damping * s->nodes[i].vy;
    }
}

/* Per-step force assembly.  Reads as the physical decomposition:
 *     start from zero      → add Hooke restoring forces
 *                          → add linear velocity damping.
 * The integrator then consumes node.f in the next half of the step. */
static void compute_forces(Scene *s)
{
    clear_force_accumulators(s);
    apply_hooke_spring_forces(s);
    apply_velocity_damping(s);
}

/* Symplectic Euler: velocity updates first using the CURRENT force,
 * then position uses the NEW velocity.  Cheap and energy-conserving. */
static void integrate_step(Scene *s)
{
    float inv_m = 1.0f / s->mass;
    float dt    = s->dt;
    for (int i = 0; i < s->nn; i++) {
        if (s->nodes[i].pinned) continue;
        s->nodes[i].vx += s->nodes[i].fx * inv_m * dt;
        s->nodes[i].vy += s->nodes[i].fy * inv_m * dt;
        s->nodes[i].x  += s->nodes[i].vx * dt;
        s->nodes[i].y  += s->nodes[i].vy * dt;
    }
}

/* NaN/Inf detector — if a stiffness change pushed dt past the stability
 * limit and the lattice exploded, scenarios reset it; this just reports. */
static bool lattice_blown_up(const Scene *s)
{
    for (int i = 0; i < s->nn; i++) {
        if (!isfinite(s->nodes[i].x) || !isfinite(s->nodes[i].y) ||
            !isfinite(s->nodes[i].vx)|| !isfinite(s->nodes[i].vy))
            return true;
    }
    return false;
}

/* ════════════════════════════════════════════════════════════════════
 * §6  SCENARIOS — the auto-cycling impulse showcase
 * ════════════════════════════════════════════════════════════════════ *
 *
 * Four strike patterns rotate every SCENARIO_TICKS frames.  Each
 * scenario_apply() call resets the lattice to rest (scenario_quiet)
 * then dispatches to the named strike helper for that id:
 *
 *   0 CENTER     — strike_center_pulse           : clean expanding ring
 *   1 CORNER     — strike_corner_pulse           : edge-reflected wave
 *   2 TWO-POINT  — strike_twopoint_interference  : interference fringes
 *   3 LINE       — strike_line_planar            : 1-D travelling wave
 *
 * scenario_strike_radial is the shared primitive — a circular impulse
 * with linear falloff that the first three strikes compose into their
 * patterns.  LINE bypasses it for a sinusoidal-velocity column.
 */

static const char * const k_scenario_names[SCENARIO_COUNT] = {
    "Center", "Corner", "TwoPoint", "Line"
};

/* Re-snap every node to its rest position and zero everything else.
 * Called at the start of each scenario_apply so prior energy doesn't
 * bleed in to the next strike. */
static void scenario_quiet(Scene *s)
{
    for (int i = 0; i < s->nn; i++) {
        s->nodes[i].x  = s->nodes[i].rx;
        s->nodes[i].y  = s->nodes[i].ry;
        s->nodes[i].vx = 0.0f;
        s->nodes[i].vy = 0.0f;
        s->nodes[i].fx = 0.0f;
        s->nodes[i].fy = 0.0f;
        s->nodes[i].pinned = false;
    }
    for (int idx = 0; idx < s->ns; idx++) s->springs[idx].strain = 0.0f;
}

/* Stamp a radial impulse on every node inside `radius` of (cr, cc).
 * Velocity magnitude falls off linearly with distance from centre,
 * pointing outward — produces the classic expanding-ring stress wave. */
static void scenario_strike_radial(Scene *s, int cr, int cc,
                                   float speed, int radius)
{
    if (cr < 0 || cr >= s->ny || cc < 0 || cc >= s->nx) return;
    for (int r = cr - radius; r <= cr + radius; r++) {
        for (int c = cc - radius; c <= cc + radius; c++) {
            if (r < 0 || r >= s->ny || c < 0 || c >= s->nx) continue;
            float dr = (float)(r - cr), dc = (float)(c - cc);
            float d2 = dr*dr + dc*dc;
            if (d2 > (float)(radius*radius)) continue;
            float fall  = 1.0f - sqrtf(d2) / (float)(radius + 1);
            float angle = (d2 < 1e-4f) ? 0.0f : atan2f(dr, dc);
            s->nodes[r * s->nx + c].vx = speed * fall * cosf(angle);
            s->nodes[r * s->nx + c].vy = speed * fall * sinf(angle);
        }
    }
}

/* The "minor axis" cell count — used to scale strike radius so each
 * scenario looks the same on any aspect ratio. */
static int lattice_minor_axis(const Scene *s)
{
    return (s->nx < s->ny ? s->nx : s->ny);
}

/* STRIKE 0 — CENTER PULSE.  One radial kick at the lattice midpoint.
 * Produces a clean expanding stress ring (the closest 2-D analogue
 * to Rayleigh's circular-membrane fundamental mode [ref 5]). */
static void strike_center_pulse(Scene *s)
{
    int radius = lattice_minor_axis(s) / 5;
    scenario_strike_radial(s, s->ny / 2, s->nx / 2, HAMMER_VEL, radius);
}

/* STRIKE 1 — CORNER PULSE.  Off-axis impulse near (1,1) — the wave
 * front is a quarter-circle that reflects off the two near edges
 * almost immediately, generating herringbone interference. */
static void strike_corner_pulse(Scene *s)
{
    int radius = lattice_minor_axis(s) / 6;
    scenario_strike_radial(s, 1, 1, HAMMER_VEL * 1.2f, radius);
}

/* STRIKE 2 — TWO-POINT INTERFERENCE.  Two equal pulses at the left
 * and right quarter columns.  The two expanding rings collide along
 * the midline: constructive interference fringes where they ADD,
 * cancellation gaps where they SUBTRACT (Crawford [ref 6]). */
static void strike_twopoint_interference(Scene *s)
{
    int radius = lattice_minor_axis(s) / 7;
    int mid_r  = s->ny / 2;
    scenario_strike_radial(s, mid_r, s->nx / 4,
                           HAMMER_VEL * 0.9f, radius);
    scenario_strike_radial(s, mid_r, s->nx - s->nx / 4,
                           HAMMER_VEL * 0.9f, radius);
}

/* STRIKE 3 — LINE / PLANAR WAVE.  Sinusoidal vertical velocity along
 * a single column.  The whole column oscillates IN PHASE along its
 * length but with one full spatial period — closest thing to a
 * textbook 1-D travelling wave on this 2-D grid. */
static void strike_line_planar(Scene *s)
{
    int col = s->nx / 3;
    for (int r = 0; r < s->ny; r++) {
        float phase = (float)r / (float)(s->ny - 1) * 2.0f * (float)M_PI;
        s->nodes[r * s->nx + col].vy = IMPULSE_VEL * sinf(phase);
    }
}

/* Dispatch the scenario id to the matching strike helper.  Body reads
 * as pure SELECTION; every velocity-stamping detail lives in the
 * named helpers above.  scenario_quiet resets state so prior energy
 * doesn't bleed between scenarios. */
static void scenario_apply(Scene *s, int id)
{
    scenario_quiet(s);
    s->scenario = id;
    s->sc_tick  = 0;

    switch (id) {
    case 0: strike_center_pulse           (s); break;
    case 1: strike_corner_pulse           (s); break;
    case 2: strike_twopoint_interference  (s); break;
    case 3: strike_line_planar            (s); break;
    }
}

/* ════════════════════════════════════════════════════════════════════
 * §7  RENDER — the stress-glow paint pipeline
 * ════════════════════════════════════════════════════════════════════ */

/* Strain → (color pair, glyph, bold).
 * Returns false when the spring is at rest and should NOT be drawn —
 * the dropout that makes wave fronts pop against an invisible mesh. */
static bool strain_visual(float strain, SpringKind kind,
                          int *out_cp, char *out_glyph, bool *out_bold)
{
    float a = fabsf(strain);
    if (a < STRAIN_NULL) return false;

    bool compress = (strain < 0.0f);
    if (a < STRAIN_MID) {
        *out_cp    = compress ? CP_COMPRESS_LO : CP_TENSION_LO;
        *out_glyph = (kind == SPRING_H) ? '-' : '|';
        *out_bold  = false;
    } else if (a < STRAIN_HIGH) {
        *out_cp    = compress ? CP_COMPRESS_HI : CP_TENSION_HI;
        *out_glyph = (kind == SPRING_H) ? '=' : 'H';
        *out_bold  = false;
    } else {
        *out_cp    = compress ? CP_COMPRESS_HI : CP_TENSION_HI;
        *out_glyph = '#';
        *out_bold  = true;
    }
    return true;
}

/* CLIPPED PAINT — single mvaddch with bounds-check against the
 * scene's renderable band (HUD rows excluded).  Every render helper
 * funnels through this so clipping logic lives in exactly one place. */
static void paint_cell_clipped(const Scene *s, int row, int col,
                               char glyph, int attr)
{
    if (col < 0 || col >= s->term_cols) return;
    if (row < HUD_TOP_ROWS || row >= s->term_rows - HUD_BOT_ROWS) return;
    mvaddch(row, col, (chtype)(unsigned char)glyph | attr);
}

/* PARAMETRIC LINE WALK — visit the INTERIOR cells of segment A→B
 * (k = 1 .. steps-1) and paint each with the same glyph + attribute.
 * Endpoints belong to the nodes, so this never overlaps the node
 * markers.  Using a parametric walk instead of Bresenham keeps each
 * cell at its "fair share" of the line — important for short H/V
 * spans (NODE_DX = 4 or NODE_DY = 2) where Bresenham would clump. */
static void paint_spring_interior(const Scene *s,
                                  int x0, int y0, int x1, int y1,
                                  int steps, char glyph, int attr)
{
    for (int k = 1; k < steps; k++) {
        float t = (float)k / (float)steps;
        int cx = (int)(x0 + (x1 - x0) * t + 0.5f);
        int cy = (int)(y0 + (y1 - y0) * t + 0.5f);
        paint_cell_clipped(s, cy, cx, glyph, attr);
    }
}

/*
 * render_spring — paint one spring as a glowing line, if it's
 * stressed enough to be visible.
 *
 * Pseudocode:
 *     classify strain → (glyph, colour, bold) or "at rest, skip"
 *     fetch the two endpoint nodes
 *     guard against NaN/Inf positions (post-blow-up frames)
 *     walk the interior cells, painting each with the chosen glyph
 */
static void render_spring(const Scene *s, const Spring *sp)
{
    int cp; char glyph; bool bold;
    if (!strain_visual(sp->strain, sp->kind, &cp, &glyph, &bold)) return;

    const Node *a = &s->nodes[sp->a];
    const Node *b = &s->nodes[sp->b];
    if (!isfinite(a->x) || !isfinite(b->x)) return;

    int x0 = (int)(a->x + 0.5f), y0 = (int)(a->y + 0.5f);
    int x1 = (int)(b->x + 0.5f), y1 = (int)(b->y + 0.5f);

    int steps = (sp->kind == SPRING_H) ? NODE_DX : NODE_DY;
    int attr  = COLOR_PAIR(cp) | (bold ? A_BOLD : 0);

    paint_spring_interior(s, x0, y0, x1, y1, steps, glyph, attr);
}

/* NODE VISUAL FROM SPEED — classify a node into one of four visual
 * states by speed.  The thresholds (SPEED_REST, SPEED_FAST) are
 * calibrated to make "moving" pop against the rest grid without
 * flicker (Ware [ref 7], pre-attentive luminance contrast).
 *
 *   pinned (any speed)         → '@' bold WHITE/BLUE   structural anchor
 *   speed < SPEED_REST         → '.' dim   NODE_REST   the lattice baseline
 *   speed < SPEED_FAST         → 'o'       NODE_SLOW   oscillating
 *   speed ≥ SPEED_FAST         → 'O' bold  NODE_FAST   peak energy
 */
static void node_visual_from_speed(const Node *n,
                                   char *out_glyph, int *out_cp,
                                   int *out_attr)
{
    if (n->pinned) {
        *out_glyph = '@'; *out_cp = CP_NODE_PIN;  *out_attr = A_BOLD;
        return;
    }
    float speed = sqrtf(n->vx * n->vx + n->vy * n->vy);
    if (speed < SPEED_REST) {
        *out_glyph = '.'; *out_cp = CP_NODE_REST; *out_attr = 0;
    } else if (speed < SPEED_FAST) {
        *out_glyph = 'o'; *out_cp = CP_NODE_SLOW; *out_attr = 0;
    } else {
        *out_glyph = 'O'; *out_cp = CP_NODE_FAST; *out_attr = A_BOLD;
    }
}

/*
 * render_node — paint one node glyph at its CURRENT position.
 *
 * Pseudocode:
 *     guard NaN/Inf positions
 *     classify visual state from speed (pinned / rest / slow / fast)
 *     paint clipped cell with chosen glyph + colour + attribute
 */
static void render_node(const Scene *s, const Node *n)
{
    if (!isfinite(n->x) || !isfinite(n->y)) return;
    int cx = (int)(n->x + 0.5f);
    int cy = (int)(n->y + 0.5f);

    char glyph; int cp; int attr;
    node_visual_from_speed(n, &glyph, &cp, &attr);

    paint_cell_clipped(s, cy, cx, glyph, COLOR_PAIR(cp) | attr);
}

/* Compose: springs (background, only the stressed ones glow) then
 * nodes (foreground, always visible — the lattice's skeleton). */
static void render_lattice(const Scene *s)
{
    for (int i = 0; i < s->ns; i++) render_spring(s, &s->springs[i]);
    for (int i = 0; i < s->nn; i++) render_node  (s, &s->nodes  [i]);
}

/* Total kinetic + potential energy — a global "is the lattice ringing"
 * gauge surfaced in the HUD so the viewer can see decay numerically. */
static float total_energy(const Scene *s)
{
    float ke = 0.0f, pe = 0.0f;
    for (int i = 0; i < s->nn; i++) {
        float v2 = s->nodes[i].vx * s->nodes[i].vx
                 + s->nodes[i].vy * s->nodes[i].vy;
        ke += 0.5f * s->mass * v2;
    }
    for (int i = 0; i < s->ns; i++) {
        float stretch = s->springs[i].strain * s->springs[i].rest_len;
        pe += 0.5f * s->k * stretch * stretch;
    }
    return ke + pe;
}

static void render_hud(const Scene *s)
{
    /* Top row 0 — canonical bright-yellow status, A_BOLD. */
    char buf[160];
    snprintf(buf, sizeof buf,
             " MassSpring  scenario:%-8s  theme:%-7s  k=%-4.0f  damp=%-3.1f"
             "  E=%-7.1f  %2dfps  %s",
             k_scenario_names[s->scenario], k_themes[s->theme].name,
             s->k, s->damping, total_energy(s), s->fps_disp,
             s->paused ? "PAUSED " : "running");
    move(0, 0); clrtoeol();
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Bottom row — canonical bright-cyan keys, A_BOLD. */
    int bot = s->term_rows - 1;
    move(bot, 0); clrtoeol();
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(bot, 0,
             " q:quit  spc:pause  r:reset  n:next  t/T:theme"
             "  k/K:stiff  d/D:damp ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ════════════════════════════════════════════════════════════════════
 * §8  MAIN — bootstrap helpers, per-frame stage helpers, and the loop
 * ════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit   = 0;

static void on_sigwinch(int s) { (void)s; g_resize = 1; }
static void on_sigterm (int s) { (void)s; g_quit   = 1; }

/* Fit a lattice to current terminal dims (capped to MAX_NX/NY).
 * Writes nx/ny back into Scene region (B) GEOMETRY. */
static void fit_lattice(Scene *s)
{
    int avail_w = s->term_cols - 4;
    int avail_h = s->term_rows - HUD_TOP_ROWS - HUD_BOT_ROWS - 2;
    int nx = avail_w / NODE_DX + 1;
    int ny = avail_h / NODE_DY + 1;
    if (nx > MAX_NX) nx = MAX_NX;
    if (ny > MAX_NY) ny = MAX_NY;
    if (nx < 4) nx = 4;
    if (ny < 3) ny = 3;
    s->nx = nx;
    s->ny = ny;
}

/* Full scene rebuild — call after a resize or a destructive reset.
 * Re-fits the lattice to terminal dims, re-stamps node positions,
 * re-builds spring topology, and re-applies the current scenario. */
static void rebuild_for_terminal(Scene *s)
{
    fit_lattice(s);
    lattice_init(s, s->nx, s->ny);
    springs_build(s);
    scenario_apply(s, s->scenario);
}

/* Populate Scene's SIM PARAMS (C) and ANIMATION+UI (D) regions with
 * defaults.  Region A (physics data) + B (geometry) are filled by
 * rebuild_for_terminal afterwards. */
static void scene_defaults(Scene *s)
{
    s->k         = K_DEF;
    s->damping   = DAMPING_DEF;
    s->mass      = MASS_DEF;
    s->sim_fps   = TARGET_FPS;
    s->substeps  = SUBSTEPS;
    s->dt        = 1.0f / (float)(s->sim_fps * s->substeps);

    s->scenario  = 0;
    s->sc_tick   = 0;
    s->theme     = 0;
    s->paused    = false;
    s->fps_disp  = s->sim_fps;
}

/* ── Bootstrap helpers ──────────────────────────────────────────────── */

/* SIGNAL HANDLERS — SIGINT / SIGTERM → graceful quit; SIGWINCH →
 * deferred resize.  The handlers only flip volatile flags; all real
 * work happens synchronously from the main loop. */
static void install_signal_handlers(void)
{
    signal(SIGWINCH, on_sigwinch);
    signal(SIGTERM,  on_sigterm);
    signal(SIGINT,   on_sigterm);
}

/* TERMINAL SETUP — ncurses raw-keys mode + no echo + no cursor +
 * non-blocking input + typeahead off so output isn't interrupted
 * (CLAUDE.md ncurses Bug Table). */
static void terminal_setup(void)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
}

/* Modular cyclic step (+1 / −1 with wrap-around).  Makes the keybind
 * cases read as "advance by +1, wrap mod N" instead of the cryptic
 * `(x + N - 1) % N` reverse-wrap trick. */
static void cycle_index(int *value, int step, int n)
{
    *value = (*value + n + step) % n;
}

/* Clamp a float to [lo, hi] in place — keeps the k / damping keybind
 * arithmetic short and reads as a single intent line. */
static void clamp_float(float *v, float lo, float hi)
{
    if (*v < lo) *v = lo;
    if (*v > hi) *v = hi;
}

/* ── Per-frame stage helpers — one phase of the sim loop each ────── */

/* RESIZE — SIGWINCH set the flag asynchronously; we drain it here,
 * ask ncurses for fresh terminal dimensions, and rebuild the entire
 * scene (lattice topology depends on terminal size). */
static void consume_resize_event(Scene *s)
{
    if (!g_resize) return;
    g_resize = 0;
    endwin();
    refresh();
    getmaxyx(stdscr, s->term_rows, s->term_cols);
    rebuild_for_terminal(s);
}

/* INPUT — drain every queued key and dispatch.  Mutates the
 * ANIMATION+UI region (paused, scenario, theme) and the SIM PARAMS
 * region (k, damping); sets g_quit on q/ESC. */
static void process_input(Scene *s)
{
    int ch;
    while ((ch = getch()) != ERR) {
        switch (ch) {
        case 'q': case 27:                /* 27 = ESC */
            g_quit = 1;
            break;
        case ' ': case 'p':                /* pause toggle */
            s->paused = !s->paused;
            break;
        case 'r':                          /* re-trigger current scenario */
            scenario_apply(s, s->scenario);
            break;
        case 'n':                          /* next scenario */
            cycle_index(&s->scenario, +1, SCENARIO_COUNT);
            scenario_apply(s, s->scenario);
            break;
        case 't':                          /* next theme */
            cycle_index(&s->theme, +1, N_THEMES);
            theme_apply(s->theme);
            break;
        case 'T':                          /* previous theme */
            cycle_index(&s->theme, -1, N_THEMES);
            theme_apply(s->theme);
            break;
        case 'k':                          /* softer springs */
            s->k -= K_STEP;
            clamp_float(&s->k, K_MIN, K_MAX);
            break;
        case 'K':                          /* stiffer springs */
            s->k += K_STEP;
            clamp_float(&s->k, K_MIN, K_MAX);
            break;
        case 'd':                          /* less damping (rings longer) */
            s->damping -= DAMPING_STEP;
            clamp_float(&s->damping, DAMPING_MIN, DAMPING_MAX);
            break;
        case 'D':                          /* more damping (decays faster) */
            s->damping += DAMPING_STEP;
            clamp_float(&s->damping, DAMPING_MIN, DAMPING_MAX);
            break;
        }
    }
}

/* AUTO-ADVANCE — when a scenario's hold time expires, switch to the
 * next one.  Showcase auto-cycle so a passive viewer sees variety. */
static void auto_advance_scenario(Scene *s)
{
    s->sc_tick++;
    if (s->sc_tick < SCENARIO_TICKS) return;
    cycle_index(&s->scenario, +1, SCENARIO_COUNT);
    scenario_apply(s, s->scenario);
}

/* STEP PHYSICS — run substeps of (compute_forces → integrate_step),
 * detect numerical blow-up, and let the scenario auto-cycle tick.
 * Skipped entirely while paused. */
static void step_physics(Scene *s)
{
    if (s->paused) return;

    for (int sub = 0; sub < s->substeps; sub++) {
        compute_forces(s);
        integrate_step(s);
    }
    if (lattice_blown_up(s)) {
        /* k pushed past stability — rebuild current scenario */
        rebuild_for_terminal(s);
    }
    auto_advance_scenario(s);
}

/* RENDER FRAME — painter's-order composite: clear → lattice → HUD →
 * flush.  `wnoutrefresh + doupdate` (single diff write) avoids
 * flicker on slow terminals (CLAUDE.md ncurses Bug Table). */
static void render_frame(const Scene *s)
{
    erase();
    render_lattice(s);
    render_hud(s);
    wnoutrefresh(stdscr);
    doupdate();
}

/* FRAME PACING — fixed-timestep cap to sim_fps.  Sleeps the leftover
 * of (tick budget − work already done), then re-reads the clock so
 * the returned value is the start of the NEXT iteration's budget.
 * Reports the measured frame-work duration via out-param so the FPS
 * counter doesn't have to re-read the clock. */
static int64_t pace_frame_to_fps(int64_t t_last, int sim_fps,
                                 int64_t *out_work_ns)
{
    int64_t t_now  = clock_ns();
    int64_t t_work = t_now - t_last;
    int64_t t_tick = TICK_NS(sim_fps);
    clock_sleep_ns(t_tick - t_work);     /* clamped to 0 inside sleep */
    *out_work_ns = t_work;
    return clock_ns();
}

/* FPS COUNTER — accumulate per-frame elapsed time; every half second,
 * convert (frames in window × 2) into a displayed fps value.  Uses
 * `work + max(0, tick − work) = max(work, tick)` to count both
 * budgeted and busted frames uniformly. */
static void update_fps_counter(Scene *s, int64_t work_ns,
                               int64_t *fps_acc, int *fps_cnt)
{
    int64_t t_tick = TICK_NS(s->sim_fps);
    int64_t slack  = t_tick - work_ns;
    *fps_acc += work_ns + (slack > 0 ? slack : 0);
    (*fps_cnt)++;
    if (*fps_acc >= NS_PER_SEC / 2) {
        s->fps_disp = *fps_cnt * 2;      /* half-second window → ×2 */
        *fps_acc    = 0;
        *fps_cnt    = 0;
    }
}

/*
 * main — fixed-timestep sim loop.
 *
 * Pseudocode:
 *     install signals; ncurses + colours; scene defaults; theme
 *     fit lattice to current terminal; trigger initial scenario
 *     loop until quit:
 *         drain SIGWINCH    (rebuild scene if resized)
 *         drain keyboard    (mutate sim / UI knobs)
 *         step physics      (substeps + blow-up check + auto-cycle)
 *         render frame      (lattice + HUD)
 *         pace to sim_fps
 *         update fps counter
 *     teardown ncurses
 */
int main(void)
{
    install_signal_handlers();
    terminal_setup();
    color_init();

    Scene *s = &g_scene;
    memset(s, 0, sizeof *s);
    scene_defaults(s);
    theme_apply(s->theme);

    getmaxyx(stdscr, s->term_rows, s->term_cols);
    rebuild_for_terminal(s);

    int64_t t_last  = clock_ns();
    int64_t fps_acc = 0;
    int     fps_cnt = 0;

    while (!g_quit) {
        consume_resize_event(s);
        process_input(s);
        step_physics(s);
        render_frame(s);

        int64_t work_ns;
        t_last = pace_frame_to_fps(t_last, s->sim_fps, &work_ns);
        update_fps_counter(s, work_ns, &fps_acc, &fps_cnt);
    }

    endwin();
    return 0;
}
