/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * acoustic_wavesolver.c
 *
 * 2D Acoustic Wave Simulator — FDTD scalar pressure field
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  WHAT YOU SEE
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  A rectangular room filled with air.  Up to four point sources emit
 *  sinusoidal pressure waves.  Waves radiate outward as expanding rings,
 *  reflect from hard walls, and interfere — forming standing-wave patterns
 *  and Chladni-like nodal structures.
 *
 *  Three display modes  (v / i / w):
 *    PRESSURE   — signed heatmap: blue=rarefaction, red=compression
 *    INTENSITY  — |p| magnitude: dark=quiet, bright=loud
 *    WAVEFRONT  — banded |p| contours: concentric ring propagation
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — PRESSURE WAVE PROPAGATION
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  The scalar acoustic wave equation (linearized Euler / continuity):
 *
 *       ∂²p/∂t²  =  c² · ∇²p
 *
 *  p  = pressure perturbation above ambient
 *  c  = speed of sound (343 m/s in air, dimensionless here)
 *  ∇² = ∂²/∂x² + ∂²/∂y²  (2D Laplacian)
 *
 *  PHYSICAL INTERPRETATION:
 *    A region of overpressure pushes its neighbours outward.  The
 *    pushed neighbours compress their own neighbours, while the
 *    original region rebounds into underpressure.  This alternating
 *    compression / rarefaction cycle propagates outward at speed c.
 *
 *    Compression half-cycle  (p > 0): air molecules pushed together.
 *    Rarefaction half-cycle  (p < 0): molecules pulled apart.
 *    These alternate at half-wavelength spacing along the wavefront.
 *
 *  CIRCULAR WAVEFRONTS:
 *    A point source in a uniform medium radiates equally in all
 *    directions.  At time t after emission, pressure peaks lie on
 *    a circle of radius r = c·t — the expanding wavefront ring.
 *    Energy spreads over 2πr, so amplitude decays as 1/√r in 2D.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — REFLECTION BEHAVIOR
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  REFLECTING WALLS  (Dirichlet BC: p = 0 at wall):
 *    Called a "hard" or "rigid" wall in acoustics.  The wall is a
 *    pressure node — the incoming wave cannot push through it, so it
 *    reflects with a 180° phase reversal: compression → rarefaction.
 *
 *    Equivalent to an image source on the other side of the wall
 *    with inverted sign.  After many reflections the room fills with
 *    standing waves — a superposition of room modes f_mn.
 *
 *    Room mode frequencies:  f_mn = (c/2)·√[(m/Lx)²+(n/Ly)²]
 *    At resonance, energy builds up dramatically (visible as a stable
 *    bright standing pattern instead of propagating rings).
 *
 *  ABSORBING BOUNDARY  (sponge layer):
 *    A strip near each wall where pressure is multiplied by (1−d) per
 *    step.  d grows quadratically from 0 (inner edge) to SPONGE_DAMP
 *    (at the wall).  Simulates an anechoic chamber — waves dissipate
 *    without reflection and propagate as if in open free space.
 *
 *  Toggle with  b.  Watch how the field changes from complex standing
 *  patterns (reflecting) to clean expanding rings (absorbing).
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  THEORY — FREQUENCY RELATION
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  For a sinusoidal point source at frequency f:
 *    Wavelength   λ = c / f
 *    Period       T = 1 / f
 *    Wave number  k = 2π / λ
 *
 *  The displayed WAVEFRONT mode shows |p| banded at fractional
 *  amplitudes.  Each visible ring = one half-wavelength.  You can
 *  estimate wavelength visually by counting ring spacings.
 *
 *  FDTD STABILITY (CFL condition in 2D):
 *    c·dt·√(1/dx² + 1/dy²) ≤ 1
 *  Violation causes exponential blow-up.  The code enforces it via
 *  CFL=0.90 safety margin.  Changing wave speed c recalculates dt.
 *
 *  FREQUENCY ESTIMATION:
 *    Count zero-crossings at the center probe over a 0.5-second window.
 *    Dominant frequency ≈ zero_crossings / (2 · window_time).
 *    This tracks the first source well; multi-source fields show
 *    a beating average.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  Keys:
 *    q/ESC  quit         space   pause/resume
 *    v      pressure mode       i   intensity mode    w  wavefront mode
 *    1-4    toggle source 1–4
 *    b      cycle boundary (reflecting / absorbing)
 *    +/-    increase/decrease wave speed c
 *    r      reset simulation
 *    p      inject Gaussian impulse at centre
 *
 *  Build:
 *    gcc -std=c11 -O2 -Wall -Wextra \
 *        physics/acoustic_wavesolver.c \
 *        -o acoustic_wave -lncurses -lm
 *
 * ─────────────────────────────────────────────────────────────────────
 *  §1 config     §2 clock      §3 color      §4 grid
 *  §5 update_wave               §6 apply_boundary
 *  §7 render_field              §8 render_overlay
 *  §9 scene      §10 screen    §11 app
 * ─────────────────────────────────────────────────────────────────────
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/*
 * ── Wave speed (dimensionless, normalized to grid units) ─────────────
 *
 * WAVE_C_DEFAULT = 0.35:
 *   The wave travels 0.35 grid cells per simulation timestep.
 *   Why not 1.0?  In 2D FDTD, the CFL stability condition requires
 *     c·dt·√(1/dx² + 1/dy²) ≤ 1
 *   With dx=dy=1, that upper bound is c·dt ≤ 1/√2 ≈ 0.707.
 *   c=0.35 sits comfortably in the stable regime and gives rings that
 *   expand at a speed comfortable to watch — not too fast, not too slow.
 *
 * WAVE_C_MIN = 0.10:
 *   Lower bound on user-adjustable speed.  Below this, rings expand so
 *   slowly that the simulation feels stalled.  Still physically stable,
 *   just visually uninteresting.
 *
 * WAVE_C_MAX = 0.70:
 *   Upper bound on user-adjustable speed.  Near 0.707 the simulation
 *   approaches the 2D CFL limit and small rounding errors can cause
 *   visible artefacts (the grid "rings" start to become diamond-shaped).
 *   Keeping the cap at 0.70 leaves a small headroom before instability.
 *
 * WAVE_C_STEP = 0.05:
 *   Each +/- keypress changes c by this amount.  Fine enough to see the
 *   effect on ring spacing and speed without jumping past the stable range.
 */
#define WAVE_C_DEFAULT  0.35f
#define WAVE_C_MIN      0.10f
#define WAVE_C_MAX      0.70f
#define WAVE_C_STEP     0.05f

/*
 * ── Terminal cell aspect ratio correction ────────────────────────────
 *
 * ASPECT_Y = 2.0:
 *   A standard terminal character cell is roughly 8 pixels wide and
 *   16 pixels tall — i.e., twice as tall as wide (CELL_H/CELL_W = 16/8).
 *   If we treated every cell as a square in physical space the wavefront
 *   would appear as an ellipse (squashed vertically) rather than a circle.
 *
 *   Fix: define the physical y-spacing as dy_phys = ASPECT_Y * dx_phys.
 *   When computing the FDTD y-coupling coefficient ry = c·dt/dy_phys,
 *   the larger dy_phys shrinks ry relative to rx, exactly compensating
 *   for the fact that two screen rows cover the same physical distance
 *   as one screen column.
 *
 *   Result: wavefronts appear as circles on the terminal regardless of
 *   font size, as long as the font has the standard 1:2 column:row ratio.
 */
#define ASPECT_Y        2.0f

/*
 * ── CFL safety factor ─────────────────────────────────────────────────
 *
 * CFL = 0.90 (dimensionless, must be < 1.0):
 *   The Courant-Friedrichs-Lewy (CFL) condition for the 2D wave equation
 *   on a rectangular grid is:
 *     c · dt · √(1/dx² + 1/dy²) ≤ 1
 *
 *   The code sets dt = CFL / (c · √(1/dx² + 1/dy²)), so the left-hand
 *   side is exactly CFL = 0.90 — giving a 10% margin below the limit.
 *
 *   What happens at the boundary cases:
 *     CFL = 1.0  (borderline): mathematically the scheme is neutral, but
 *                floating-point rounding errors accumulate over thousands
 *                of steps and can trigger divergence.
 *     CFL > 1.0  (violation): the numerical scheme is unconditionally
 *                unstable.  Pressure values double every few steps,
 *                quickly overflowing to ±infinity (visible as the screen
 *                suddenly going all-red or all-blue).
 *     CFL = 0.90 (this code): stable with visible margin.  If you
 *                experiment by raising WAVE_C_MAX above 0.70, keep
 *                watching for the blow-up artefact.
 */
#define CFL             0.90f

/*
 * ── Target wavelength in grid cells ──────────────────────────────────
 *
 * LAMBDA_CELLS = 10:
 *   Source frequencies are derived so that the wavelength λ equals
 *   exactly 10 grid cells:  f = c / (LAMBDA_CELLS · dx).
 *   This guarantees the rings are visually clear at any terminal size
 *   or wave speed — you always see several ring spacings on screen.
 *
 *   Too small (e.g., 3–4 cells): rings are so tight they blur into a
 *   grey wash; individual wavefronts are not distinguishable.
 *   Too large (e.g., 40+ cells): the wavelength spans most of the screen;
 *   only one ring is visible and the sweep looks very slow.
 *   10 cells is the sweet spot for a typical 80–160 column terminal.
 */
#define LAMBDA_CELLS    10.0f

/*
 * ── Physics sub-steps per rendered frame ─────────────────────────────
 *
 * STEPS_PER_FRAME = 4:
 *   Each display frame advances the simulation by 4 physics timesteps.
 *   This makes the wave propagate at 4× the speed it would if only one
 *   step were taken per frame — useful because dt is very small (set by
 *   the CFL condition) while we only draw at ~30 fps.
 *
 *   Increasing this number makes waves expand faster on screen (more
 *   simulation time per real-time second).  Decreasing it to 1 makes
 *   the simulation play in slow motion.  Values above ~8 may cause the
 *   30-fps budget to be exceeded on slow machines.
 */
#define STEPS_PER_FRAME  4

/*
 * ── Grid size caps ────────────────────────────────────────────────────
 *
 * GRID_MAX_X = 160, GRID_MAX_Y = 48:
 *   The physics grid is capped at these dimensions even if the terminal
 *   is wider/taller.  The FDTD inner loop is O(nx·ny) per step, so a
 *   very large terminal (e.g., 300×80) would push the per-frame physics
 *   cost above the 33 ms frame budget at 30 fps.
 *   With these caps the worst case is 160×48 = 7 680 cells × 4 steps =
 *   30 720 stencil evaluations per frame — fast enough on any modern CPU.
 */
#define GRID_MAX_X  160
#define GRID_MAX_Y   48

/*
 * ── Maximum simultaneous point sources ───────────────────────────────
 *
 * MAX_SOURCES = 4:
 *   The simulator supports up to four independently toggled point sources.
 *   Each source injects p_new[src] += amp·sin(2π·f·t) at its grid cell.
 *   With two or more sources active, waves from different sources overlap
 *   and interfere — producing constructive bright bands where crests meet,
 *   and destructive dark bands (nodal lines) where crest meets trough.
 *   Four sources produce rich, Chladni-figure-like interference patterns.
 */
#define MAX_SOURCES  4

/*
 * ── Sponge (absorbing boundary) parameters ───────────────────────────
 *
 * SPONGE_WIDTH = 8  (cells from each wall):
 *   The absorbing boundary is not a single wall cell — it is a gradual
 *   absorption zone 8 cells wide along all four edges.  Abrupt absorption
 *   at a single cell would itself cause a reflection (impedance mismatch).
 *   The wider the zone, the gentler the impedance transition and the less
 *   the residual reflection.  8 cells is a practical minimum; wider zones
 *   consume more of the visible grid.
 *
 * SPONGE_DAMP = 0.10  (maximum damping coefficient, 10%):
 *   At the outermost cell of the sponge, pressure is multiplied by
 *     (1 − SPONGE_DAMP) = 0.90  each timestep.
 *   The damping coefficient d grows quadratically from 0 at the inner
 *   edge to SPONGE_DAMP at the wall face:
 *     d(e) = SPONGE_DAMP · (e / SPONGE_WIDTH)²
 *   where e is the cell's distance from the inner sponge boundary.
 *   Quadratic ramp avoids the abrupt step-change that a linear ramp
 *   would create at the inner edge.
 *
 *   Compare: a true Perfectly Matched Layer (PML) modifies the wave
 *   equation itself with complex stretching coordinates and achieves
 *   near-zero reflection for any angle.  The sponge used here is
 *   simpler to implement and sufficient for visual purposes; it leaves
 *   a small residual reflection for waves hitting at grazing angles.
 */
#define SPONGE_WIDTH    8     /* cells from each wall */
#define SPONGE_DAMP     0.10f /* max damping coefficient at wall */

/*
 * ── Source positions and frequency multipliers ───────────────────────
 *
 * SRC_NX / SRC_NY:
 *   Normalized [0, 1] × [0, 1] positions.  Multiplied by (nx−1) or
 *   (ny−1) at runtime to get grid cell indices.  Using normalized
 *   coordinates means source positions scale automatically with any
 *   terminal size — the same four sources always occupy the same
 *   relative positions in the room.
 *
 *   Source layout (approximate):
 *     1 → left-centre     2 → right-centre
 *     3 → top-centre      4 → bottom-left
 *   This asymmetric arrangement avoids mirror symmetry so interference
 *   patterns are richer and less regular.
 *
 * SRC_FMUL:
 *   Frequency multipliers relative to the base frequency derived from
 *   LAMBDA_CELLS.  The four values {1.00, 1.33, 0.75, 1.60} are chosen
 *   to produce interesting beating and interference:
 *     1.00 and 1.33 (ratio 3:4) → slow beats; periodic constructive /
 *                                  destructive reinforcement.
 *     1.00 and 0.75 (ratio 4:3) → similar beating in the other direction.
 *     1.00 and 1.60 (ratio 5:8) → faster modulation, quasiperiodic.
 *   The irrational-ish ratios prevent the field from locking into a
 *   perfectly repeating pattern, keeping the display lively.
 */
static const float SRC_NX[MAX_SOURCES] = { 0.25f, 0.75f, 0.50f, 0.25f };
static const float SRC_NY[MAX_SOURCES] = { 0.50f, 0.50f, 0.25f, 0.75f };
/* Frequency multipliers relative to base_freq (set in wave_init) */
static const float SRC_FMUL[MAX_SOURCES] = { 1.00f, 1.33f, 0.75f, 1.60f };

/* Boundary mode */
#define BC_REFLECT  0
#define BC_ABSORB   1

/* Visualization modes */
#define VIS_PRESSURE   0
#define VIS_INTENSITY  1
#define VIS_WAVEFRONT  2
#define VIS_COUNT      3

/* Color pair IDs */
#define CP_PN2   1   /* pressure: strong negative (deep blue)   */
#define CP_PN1   2
#define CP_PN0   3
#define CP_PZERO 4   /* near zero (dim grey)                    */
#define CP_PP0   5
#define CP_PP1   6
#define CP_PP2   7   /* pressure: strong positive (deep red)    */
#define CP_INT0  8   /* intensity: darkest                      */
#define CP_INT1  9
#define CP_INT2  10
#define CP_INT3  11
#define CP_INT4  12
#define CP_INT5  13
#define CP_INT6  14
#define CP_INT7  15  /* intensity: brightest                    */
#define CP_WFA   16  /* wavefront low band (dim)                */
#define CP_WFB   17  /* wavefront mid band                      */
#define CP_WFC   18  /* wavefront high band (bright)            */
#define CP_SRC   19  /* source marker (yellow)                  */
#define CP_HUD   20  /* HUD overlay (cyan)                      */

static const char DENS_CHARS[8] = { ' ', '.', ':', '+', 'x', 'X', '#', '@' };
static const char *VIS_NAMES[VIS_COUNT] = { "PRESSURE", "INTENSITY", "WAVEFRONT" };
static const char *BC_NAMES[2] = { "REFLECTING", "ABSORBING " };

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

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
    struct timespec r = { (time_t)(ns/NS_PER_SEC), (long)(ns%NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        /* Pressure: diverging blue → grey → red */
        init_pair(CP_PN2,   17, COLOR_BLACK);   /* deep blue    */
        init_pair(CP_PN1,   27, COLOR_BLACK);   /* mid blue     */
        init_pair(CP_PN0,  153, COLOR_BLACK);   /* light blue   */
        init_pair(CP_PZERO,238, COLOR_BLACK);   /* dim grey     */
        init_pair(CP_PP0,  217, COLOR_BLACK);   /* light red    */
        init_pair(CP_PP1,  160, COLOR_BLACK);   /* mid red      */
        init_pair(CP_PP2,   52, COLOR_BLACK);   /* deep red     */
        /* Intensity: black → bright green → white */
        init_pair(CP_INT0,  16, COLOR_BLACK);
        init_pair(CP_INT1,  22, COLOR_BLACK);
        init_pair(CP_INT2,  28, COLOR_BLACK);
        init_pair(CP_INT3,  34, COLOR_BLACK);
        init_pair(CP_INT4,  82, COLOR_BLACK);
        init_pair(CP_INT5, 118, COLOR_BLACK);
        init_pair(CP_INT6, 154, COLOR_BLACK);
        init_pair(CP_INT7, 231, COLOR_BLACK);   /* white        */
        /* Wavefront bands */
        init_pair(CP_WFA,   17, COLOR_BLACK);   /* dark blue    */
        init_pair(CP_WFB,   27, COLOR_BLACK);   /* mid cyan     */
        init_pair(CP_WFC,   51, COLOR_BLACK);   /* bright cyan  */
        /* Misc */
        init_pair(CP_SRC,  226, COLOR_BLACK);   /* yellow       */
        init_pair(CP_HUD,   51, COLOR_BLACK);   /* cyan HUD     */
    } else {
        init_pair(CP_PN2,  COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_PN1,  COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_PN0,  COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_PZERO,COLOR_BLACK,   COLOR_BLACK);
        init_pair(CP_PP0,  COLOR_MAGENTA, COLOR_BLACK);
        init_pair(CP_PP1,  COLOR_RED,     COLOR_BLACK);
        init_pair(CP_PP2,  COLOR_RED,     COLOR_BLACK);
        for (int i = 0; i < 8; i++)
            init_pair(CP_INT0+i, COLOR_GREEN, COLOR_BLACK);
        init_pair(CP_WFA, COLOR_BLUE,   COLOR_BLACK);
        init_pair(CP_WFB, COLOR_CYAN,   COLOR_BLACK);
        init_pair(CP_WFC, COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_SRC, COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_HUD, COLOR_YELLOW, COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  grid — state, allocation, initialization                           */
/* ===================================================================== */

/*
 * Source — one independent sinusoidal point source.
 *
 *   nx_f, ny_f : normalized [0,1] position.  Converted to integer grid
 *                cell indices at each step: sx = round(nx_f * (nx-1)).
 *                Normalized storage means source positions are grid-size
 *                independent — sources move to the correct relative spot
 *                when the terminal is resized.
 *
 *   freq       : driving frequency in simulation time units (cycles per
 *                unit of wave->time).  Derived from LAMBDA_CELLS and
 *                multiplied by SRC_FMUL[i] so different sources emit at
 *                harmonically related but distinct frequencies.
 *
 *   amp        : pressure amplitude added to p_new at the source cell
 *                each step (monopole injection strength).
 *
 *   active     : whether this source is currently driving the field.
 *                Toggled by keys 1–4.  Inactive sources are skipped
 *                entirely during injection so they contribute no energy.
 */
typedef struct {
    float nx_f, ny_f;   /* normalized position of source [0,1] */
    float freq;         /* driving frequency (simulation Hz)   */
    float amp;          /* pressure amplitude injected per step */
    bool  active;
} Source;

/*
 * Wave — the entire simulation state.
 *
 * ── Pressure field buffers (triple-buffer leapfrog scheme) ────────────
 *
 *   p, p_old, p_new  are three equally-sized flat arrays of floats,
 *   each holding nx*ny pressure values.  They represent three
 *   consecutive time levels in the leapfrog time-integration:
 *
 *     p_old  →  pressure at time t − dt   (one step in the past)
 *     p      →  pressure at time t        (current)
 *     p_new  →  pressure at time t + dt   (being computed)
 *
 *   Why three buffers?
 *   The second-order-in-time central-difference formula
 *     p_new = 2·p − p_old + (spatial stencil)
 *   requires two past levels (t and t−dt) to advance to the next level
 *   (t+dt).  A first-order Euler scheme would need only one past level
 *   but is only first-order accurate in time; the leapfrog scheme is
 *   second-order accurate with the same memory cost as two arrays (the
 *   third array is reused in a rolling fashion).
 *
 *   After each step the pointers are rotated:
 *     new p_old ← old p       (what was "current" becomes "past")
 *     new p     ← old p_new   (the freshly computed level becomes "current")
 *     new p_new ← old p_old   (the old "past" buffer is recycled for scratch)
 *   This is an O(1) pointer swap — no data is copied (contrast: O(N) memcpy).
 *
 * ── Spatial discretization parameters ────────────────────────────────
 *
 *   dx   : physical cell spacing.  Derived as 1 / (min(nx,ny) − 1) so
 *          that the shorter grid dimension spans exactly [0, 1] in
 *          physical units.  The y-spacing is dy = ASPECT_Y * dx, but
 *          we store only dx because dy is always ASPECT_Y times it.
 *
 *   dt   : simulation timestep, recomputed whenever wave speed c changes.
 *          Formula: dt = CFL / (c · √(1/dx² + 1/dy²)).
 *          Smaller c → larger dt (waves need more time to cross a cell).
 *          Never set manually; always derived from c via wave_recompute_dt.
 *
 * ── Precomputed FDTD coupling coefficients ────────────────────────────
 *
 *   rx2  = (c·dt/dx)²    — x-direction FDTD coupling strength
 *   ry2  = (c·dt/dy)²    — y-direction FDTD coupling strength
 *          where dy = ASPECT_Y · dx
 *
 *   These are the squared Courant numbers for each axis.  Their sum
 *   rx2 + ry2 = CFL² / (1 + 1/ASPECT_Y²) · something ≤ 1 by construction.
 *   They appear in every cell of the inner FDTD loop:
 *     p_new = 2p − p_old + rx2·(p_E + p_W − 2p) + ry2·(p_N + p_S − 2p)
 *   Precomputing rx2 and ry2 saves two multiplications and two divisions
 *   per inner-loop iteration — a meaningful saving when iterating over
 *   tens of thousands of cells per frame.
 *
 * ── Sponge coefficient array ──────────────────────────────────────────
 *
 *   damp[]  : per-cell damping coefficient, same layout as p[].
 *             0.0 everywhere in the interior.
 *             Grows quadratically to SPONGE_DAMP at each wall face.
 *             Applied as p[i] *= (1 − damp[i]) in absorbing BC mode.
 *             Precomputed once in build_sponge(); never changes unless
 *             the grid is resized.
 *
 * ── Diagnostics ───────────────────────────────────────────────────────
 *
 *   p_max   : smoothed running maximum of |p| across all cells.
 *             Updated each boundary step:  p_max = 0.97·p_max + 0.03·max|p|.
 *             The slow update rate (3% weight on the new sample) prevents
 *             frame-to-frame flicker in the color scale when a brief
 *             transient creates a large |p| spike.  All three render modes
 *             divide by p_max to normalize pressure to [−1, 1] or [0, 1].
 *
 *   energy  : Σ p[i]² over all cells — proportional to total acoustic
 *             energy in the field.  In BC_REFLECT mode with a driven
 *             source this grows until it reaches a steady oscillation.
 *             In BC_ABSORB mode (no source) it decays toward zero as the
 *             sponge absorbs the field.  Displayed in the HUD as "E=".
 *
 *   freq_est : dominant frequency estimated by zero-crossing counting at
 *             the grid center.  Updated every 0.5 simulation-time units.
 *
 *   probe_prev : pressure value at the center probe on the previous step.
 *             Used to detect sign changes (zero crossings).
 *
 *   zc_count : number of zero crossings detected since the last frequency
 *             estimate update.  Each full oscillation produces exactly
 *             2 zero crossings (one positive→negative, one negative→positive),
 *             so  freq_est = zc_count / (2 · zc_timer).
 *
 *   zc_timer : accumulated simulation time since the last frequency update.
 *             Reset to 0 each time freq_est is recomputed (every 0.5 s).
 */
typedef struct {
    int    nx, ny;      /* grid dimensions                                 */
    float  dx;          /* physical cell spacing (same for x; dy=ASPECT*dx)*/
    float  dt;          /* adaptive timestep                               */
    float  c;           /* wave speed                                      */
    float  time;        /* accumulated simulation time                     */

    float *p;           /* current pressure  [ny*nx]                       */
    float *p_old;       /* previous pressure [ny*nx]                       */
    float *p_new;       /* scratch buffer    [ny*nx]                       */
    float *damp;        /* per-cell sponge coefficient [ny*nx]             */

    /* precomputed FDTD coupling coefficients */
    float  rx2;         /* (c·dt/dx)²                                      */
    float  ry2;         /* (c·dt/(ASPECT_Y·dx))²                           */

    Source srcs[MAX_SOURCES];

    /* diagnostics */
    float  p_max;       /* smoothed max |p| for display normalization      */
    float  energy;      /* Σ p² (total acoustic energy, unnormalized)      */
    float  freq_est;    /* dominant frequency estimate from zero-crossings  */
    float  probe_prev;  /* previous pressure at centre probe               */
    int    zc_count;    /* zero-crossing counter in current window         */
    float  zc_timer;    /* elapsed sim time in current zero-cross window   */

    int    vis_mode;
    int    bc_mode;
    bool   paused;
} Wave;

#define IDX(w, x, y) ((y)*(w)->nx + (x))

static int wave_alloc(Wave *w, int nx, int ny)
{
    w->nx = nx; w->ny = ny;
    int n = nx * ny;
    w->p     = calloc(n, sizeof(float));
    w->p_old = calloc(n, sizeof(float));
    w->p_new = calloc(n, sizeof(float));
    w->damp  = calloc(n, sizeof(float));
    return (w->p && w->p_old && w->p_new && w->damp) ? 0 : -1;
}

static void wave_free(Wave *w)
{
    free(w->p); free(w->p_old); free(w->p_new); free(w->damp);
}

static void build_sponge(Wave *w)
{
    /* Quadratic ramp: d=0 at inner boundary, SPONGE_DAMP at wall */
    for (int y = 0; y < w->ny; y++) {
        for (int x = 0; x < w->nx; x++) {
            int ex = (x < SPONGE_WIDTH) ? (SPONGE_WIDTH - x)
                   : (x > w->nx-1-SPONGE_WIDTH ? (x - (w->nx-1-SPONGE_WIDTH)) : 0);
            int ey = (y < SPONGE_WIDTH) ? (SPONGE_WIDTH - y)
                   : (y > w->ny-1-SPONGE_WIDTH ? (y - (w->ny-1-SPONGE_WIDTH)) : 0);
            int e  = ex > ey ? ex : ey;
            float d = 0.0f;
            if (e > 0) {
                float t = (float)e / (float)SPONGE_WIDTH;
                d = SPONGE_DAMP * t * t;
            }
            w->damp[IDX(w, x, y)] = d;
        }
    }
}

static void wave_recompute_dt(Wave *w)
{
    /* CFL: c·dt/dx · √(1 + 1/ASPECT_Y²) ≤ CFL */
    float r = CFL / (w->c * sqrtf(1.0f + 1.0f/(ASPECT_Y*ASPECT_Y)));
    w->dt  = r * w->dx;
    float rx = w->c * w->dt / w->dx;
    float ry = w->c * w->dt / (ASPECT_Y * w->dx);
    w->rx2 = rx * rx;
    w->ry2 = ry * ry;
}

static void wave_init(Wave *w, int nx, int ny)
{
    w->c    = WAVE_C_DEFAULT;
    /* dx = smallest physical spacing; y-spacing = ASPECT_Y*dx */
    w->dx   = 1.0f / (float)((nx < ny ? nx : ny) - 1);
    w->time = 0.0f;
    w->p_max = 1e-6f;
    w->energy = 0.0f;
    w->freq_est  = 0.0f;
    w->probe_prev = 0.0f;
    w->zc_count  = 0;
    w->zc_timer  = 0.0f;
    w->vis_mode  = VIS_PRESSURE;
    w->bc_mode   = BC_REFLECT;
    w->paused    = false;

    wave_recompute_dt(w);

    memset(w->p,     0, nx*ny*sizeof(float));
    memset(w->p_old, 0, nx*ny*sizeof(float));
    memset(w->p_new, 0, nx*ny*sizeof(float));
    build_sponge(w);

    /*
     * Compute source frequency so wavelength ≈ LAMBDA_CELLS in x-cells.
     *   λ_cells = c / (f · dx)  →  f = c / (LAMBDA_CELLS · dx)
     * This ensures ring spacing is visually comparable to ~10 cells
     * regardless of terminal size or wave speed.
     */
    float base_freq = w->c / (LAMBDA_CELLS * w->dx);
    for (int i = 0; i < MAX_SOURCES; i++) {
        w->srcs[i].nx_f   = SRC_NX[i];
        w->srcs[i].ny_f   = SRC_NY[i];
        w->srcs[i].freq   = base_freq * SRC_FMUL[i];
        w->srcs[i].amp    = 1.0f;
        w->srcs[i].active = (i == 0);   /* only source 1 active at start */
    }
}

/* ===================================================================== */
/* §5  update_wave() — FDTD leapfrog timestep                            */
/* ===================================================================== */

/*
 * ── Deriving the FDTD stencil from the wave equation ─────────────────
 *
 * Starting from ∂²p/∂t² = c²·(∂²p/∂x² + ∂²p/∂y²), replace each
 * second derivative with a second-order central difference:
 *
 *   ∂²p/∂t²  ≈  [p(t+dt) − 2p(t) + p(t−dt)] / dt²
 *   ∂²p/∂x²  ≈  [p(x+dx) − 2p(x) + p(x−dx)] / dx²
 *   ∂²p/∂y²  ≈  [p(y+dy) − 2p(y) + p(y−dy)] / dy²
 *
 * Substitute and solve for p(t+dt):
 *
 *   p_new = 2·p − p_old
 *         + rx²·(p_E + p_W − 2·p)    x-Laplacian term, spacing dx
 *         + ry²·(p_N + p_S − 2·p)    y-Laplacian term, spacing dy
 *
 *   where rx = c·dt/dx,  ry = c·dt/dy = c·dt/(ASPECT_Y·dx)
 *
 * ── Why "leapfrog" and why is it second-order? ────────────────────────
 *
 *   The name "leapfrog" comes from the time indexing: to advance from
 *   time t to t+dt we need the field at both t (= p) and t−dt (= p_old).
 *   The two time levels "leap over" each other at each step.
 *
 *   A first-order Euler scheme would approximate ∂²p/∂t² ≈ (p_new−p)/dt²
 *   using only one past level; its truncation error is O(dt).
 *   The leapfrog central-difference approximation has truncation error
 *   O(dt²) — one order higher — because all odd-order error terms cancel
 *   by symmetry when both p_old and p are used.  This means halving dt
 *   reduces the time-integration error by a factor of 4 rather than 2.
 *
 * ── Source injection: additive (monopole) vs Dirichlet (hard) ─────────
 *
 *   Two ways to drive a source cell:
 *     Dirichlet:   p_new[src] = amp · sin(2π·f·t)   (override)
 *     Additive:    p_new[src] += amp · sin(2π·f·t)  (add to FDTD result)
 *
 *   Dirichlet injection forces an exact pressure value at the cell,
 *   creating an impedance discontinuity between the source cell and its
 *   neighbours — effectively a hard wall that partially reflects the
 *   outgoing wave back inward.  This distorts the circular wavefront
 *   near the source and causes non-physical artefacts.
 *
 *   Additive injection adds a small pressure excitation on top of
 *   whatever the FDTD computed — it is the numerical equivalent of a
 *   monopole (omnidirectional point source).  The source drives the
 *   field without interrupting wave propagation at the injection cell,
 *   producing clean, symmetric circular wavefronts.
 *
 * ── O(1) buffer rotation ──────────────────────────────────────────────
 *
 *   After computing p_new, we rotate the three pointers:
 *     p_old ← p        (current becomes past)
 *     p     ← p_new    (newly computed becomes current)
 *     p_new ← old p_old  (past buffer is recycled for the next scratch)
 *
 *   This is three pointer assignments — O(1) cost regardless of grid size.
 *   The alternative (memcpy) would copy 2·nx·ny floats = O(N) every step,
 *   which would dominate the runtime for large grids.
 */
static void update_wave(Wave *w)
{
    int   nx  = w->nx, ny = w->ny;
    float rx2 = w->rx2,  ry2 = w->ry2;

    /* Interior FDTD */
    for (int y = 1; y < ny-1; y++) {
        for (int x = 1; x < nx-1; x++) {
            float pc  = w->p    [IDX(w, x,   y  )];
            float pe  = w->p    [IDX(w, x+1, y  )];
            float pw_ = w->p    [IDX(w, x-1, y  )];
            float pn  = w->p    [IDX(w, x,   y+1)];
            float ps  = w->p    [IDX(w, x,   y-1)];
            float po  = w->p_old[IDX(w, x,   y  )];
            w->p_new[IDX(w, x, y)] =
                2.0f*pc - po
                + rx2*(pe + pw_ - 2.0f*pc)
                + ry2*(pn + ps  - 2.0f*pc);
        }
    }

    /* Source injection into p_new */
    for (int i = 0; i < MAX_SOURCES; i++) {
        if (!w->srcs[i].active) continue;
        int sx = (int)(w->srcs[i].nx_f * (float)(nx-1) + 0.5f);
        int sy = (int)(w->srcs[i].ny_f * (float)(ny-1) + 0.5f);
        if (sx < 1) sx = 1;
        if (sx > nx-2) sx = nx-2;
        if (sy < 1) sy = 1;
        if (sy > ny-2) sy = ny-2;
        w->p_new[IDX(w, sx, sy)] +=
            w->srcs[i].amp * sinf(2.0f*(float)M_PI * w->srcs[i].freq * w->time);
    }

    /* Rotate buffers: p_old ← p, p ← p_new, p_new ← old p_old */
    float *tmp = w->p_old;
    w->p_old   = w->p;
    w->p       = w->p_new;
    w->p_new   = tmp;

    w->time += w->dt;
}

/* ===================================================================== */
/* §6  apply_boundary() — wall BC + diagnostics                          */
/* ===================================================================== */

/*
 * ── BC_REFLECT: Dirichlet (hard acoustic wall) ────────────────────────
 *
 *   Setting p = 0 on the four boundary rows/columns is the Dirichlet
 *   boundary condition.  In acoustic terms this is a "rigid" or "hard"
 *   wall — it cannot move, so the net particle velocity at the surface
 *   is zero.  By the acoustic impedance relation, a rigid wall is a
 *   pressure antinode for the particle velocity but a pressure node for
 *   the wave itself, meaning:
 *
 *     - The wall forces p = 0 (pressure node at the boundary).
 *     - An incoming compression peak (p > 0) reflects as a rarefaction
 *       trough (p < 0) — a 180° phase reversal.
 *
 *   This is mathematically equivalent to placing an "image source" of
 *   opposite sign on the other side of each wall.  After many back-and-
 *   forth reflections the field builds into a standing-wave pattern
 *   (room modes) rather than outwardly propagating rings.
 *
 *   Why zero BOTH p and p_old?
 *   Both buffers are "live" — the leapfrog stencil reads p and p_old
 *   simultaneously in the next update_wave() call.  Zeroing only p
 *   would leave a non-zero p_old at the boundary that would immediately
 *   re-inject energy into the interior via the "−p_old" term.
 *
 * ── BC_ABSORB: sponge layer (anechoic chamber) ────────────────────────
 *
 *   Instead of a sharp wall, the sponge multiplies pressure by (1−d)
 *   each step in a SPONGE_WIDTH-cell border zone.  The coefficient d
 *   is stored in w->damp[] (precomputed, zero in the interior).
 *
 *   The quadratic ramp (d ∝ distance²) creates a smooth impedance
 *   gradient from the interior to the wall, minimising the reflection
 *   that a step-change would cause.
 *
 *   Both p and p_old are damped for the same reason as in BC_REFLECT:
 *   both buffers are read by the next FDTD step.
 *
 *   Comparison with a true Perfectly Matched Layer (PML):
 *     PML analytically modifies the wave equation inside the absorbing
 *     zone using complex-coordinate stretching so that outgoing waves of
 *     any frequency and any angle of incidence are absorbed with zero
 *     reflection.  It requires storing additional auxiliary field
 *     components and is more complex to implement.  The sponge used here
 *     is a simple multiplicative attenuation — effective for normal
 *     incidence, leaves a small residual reflection for grazing angles,
 *     but sufficient for a visual demonstration.
 *
 * ── Diagnostics ───────────────────────────────────────────────────────
 *
 *   p_max smoothing:
 *     new_p_max = 0.97·p_max + 0.03·max|p|
 *     A slow exponential moving average.  Without smoothing, a single
 *     high-amplitude transient (e.g., a Gaussian impulse) would rescale
 *     the entire color map to that peak, making everything else appear
 *     nearly zero until the transient decays.  The 3% weight means the
 *     scale adapts slowly — changes in field strength are visible over
 *     ~30 frames rather than instantly.
 *
 *   Energy (Σ p²):
 *     In BC_REFLECT with an active source, energy grows as the room
 *     fills with reflections, then oscillates at steady state.
 *     In BC_ABSORB with no source, energy decays monotonically toward
 *     zero as the sponge absorbs all remaining pressure.  A sudden jump
 *     in energy indicates a new source was activated or an impulse was
 *     injected.
 *
 *   Zero-crossing frequency estimator:
 *     A zero crossing occurs whenever the pressure at the center probe
 *     changes sign (probe_prev and pc have opposite signs).  Each
 *     complete oscillation at frequency f produces exactly 2 crossings
 *     per period T = 1/f — once from positive to negative, once from
 *     negative to positive.  So:
 *
 *       freq_est = zc_count / (2 · zc_timer)
 *
 *     The window is reset every 0.5 simulation-time units to track
 *     frequency changes when wave speed c is adjusted.  With multiple
 *     active sources at different frequencies, the estimator measures
 *     the dominant frequency at the probe location (usually the first
 *     source, which is placed closest to the centre).
 */
static void apply_boundary(Wave *w)
{
    int nx = w->nx, ny = w->ny;

    if (w->bc_mode == BC_REFLECT) {
        for (int x = 0; x < nx; x++) {
            w->p[IDX(w,x,0)]     = 0.0f;  w->p_old[IDX(w,x,0)]     = 0.0f;
            w->p[IDX(w,x,ny-1)]  = 0.0f;  w->p_old[IDX(w,x,ny-1)]  = 0.0f;
        }
        for (int y = 0; y < ny; y++) {
            w->p[IDX(w,0,y)]     = 0.0f;  w->p_old[IDX(w,0,y)]     = 0.0f;
            w->p[IDX(w,nx-1,y)]  = 0.0f;  w->p_old[IDX(w,nx-1,y)]  = 0.0f;
        }
    } else {
        /* Sponge: apply damping in both p and p_old (both buffers are live) */
        for (int y = 0; y < ny; y++) {
            for (int x = 0; x < nx; x++) {
                float d = w->damp[IDX(w, x, y)];
                if (d > 0.0f) {
                    float s = 1.0f - d;
                    w->p    [IDX(w,x,y)] *= s;
                    w->p_old[IDX(w,x,y)] *= s;
                }
            }
        }
    }

    /* Diagnostics: p_max, energy */
    float pmax = 1e-6f, energy = 0.0f;
    for (int i = 0; i < nx*ny; i++) {
        float ap = fabsf(w->p[i]);
        if (ap > pmax) pmax = ap;
        energy += w->p[i] * w->p[i];
    }
    /* Smooth p_max so color scale doesn't flicker */
    w->p_max  = w->p_max * 0.97f + pmax * 0.03f;
    if (w->p_max < 1e-6f) w->p_max = 1e-6f;
    w->energy = energy;

    /* Frequency estimation via zero-crossings at centre probe */
    int px = nx/2, py = ny/2;
    float pc = w->p[IDX(w, px, py)];
    if ((w->probe_prev < 0.0f && pc >= 0.0f) ||
        (w->probe_prev >= 0.0f && pc < 0.0f))
        w->zc_count++;
    w->probe_prev = pc;
    w->zc_timer  += w->dt;
    if (w->zc_timer >= 0.5f) {
        w->freq_est  = (float)w->zc_count / (2.0f * w->zc_timer);
        w->zc_count  = 0;
        w->zc_timer  = 0.0f;
    }
}

/* ===================================================================== */
/* §7  render_field() — three visualization modes                         */
/* ===================================================================== */

/*
 * ── attrset change-detection optimization ────────────────────────────
 *
 *   ncurses' attrset() and COLOR_PAIR() calls carry overhead: they touch
 *   internal terminal state and eventually generate escape sequences.
 *   For a full-screen wave field most adjacent cells share the same color
 *   (uniform-pressure regions, nodes, or bands).
 *
 *   Optimization: track the last-set attribute in cur_attr and only call
 *   attrset() when the new cell's attribute differs.  In a typical pressure
 *   field with large uniform-color regions, 70–80% of cells skip the call,
 *   reducing ncurses overhead by roughly 3×.
 *
 * ── Screen vs physics y-axis ─────────────────────────────────────────
 *
 *   Terminal row 0 is at the top of the screen; the physics grid uses
 *   row 0 at the bottom (standard mathematical convention).
 *   Mapping:  screen row sy  →  grid row gy = (ny − 1) − sy
 *   Row sy=0 maps to gy=ny-1 (top of physics grid = top of screen).
 *   Row sy=ny-1 maps to gy=0 (bottom of physics grid = bottom of screen).
 *   Without this flip the wave field would appear vertically mirrored.
 *
 * ── VIS_PRESSURE: signed diverging heatmap ───────────────────────────
 *
 *   Pressure p is normalized to [−1, +1] by dividing by p_max.
 *   Seven bands map the normalized value to color + character:
 *
 *     p < −0.65 → deep blue  '#' bold      (strong rarefaction)
 *     p < −0.30 → mid  blue  'x'           (moderate rarefaction)
 *     p < −0.07 → light blue ':'           (weak rarefaction)
 *     |p| < 0.07 → grey     ' ' dim        (near-zero / node)
 *     p >  0.07 → light red  ':'           (weak compression)
 *     p >  0.30 → mid  red   'x'           (moderate compression)
 *     p >  0.65 → deep red   '#' bold      (strong compression)
 *
 *   Blue = rarefaction (molecules spread apart, below-ambient pressure).
 *   Red  = compression (molecules squeezed together, above-ambient).
 *   This is the standard diverging colormap used in acoustics textbooks.
 *   Best mode for understanding the instantaneous wave structure and
 *   the phase relationship between wavefronts.
 *
 * ── VIS_INTENSITY: |p| magnitude ramp ────────────────────────────────
 *
 *   |p| is scaled to [0, 7] and mapped to 8 intensity levels (CP_INT0
 *   through CP_INT7) using the DENS_CHARS character density ramp.
 *   Dark = quiet (nodes where waves cancel), bright = loud (antinodes).
 *   Standing-wave patterns are most visible in this mode because the
 *   nodal lines (perpetually dark) are clearly distinct from the
 *   antinodal regions (perpetually bright).
 *
 * ── VIS_WAVEFRONT: banded contour rings ──────────────────────────────
 *
 *   |p| is divided into N_WF_BANDS=6 bands.  Adjacent bands use
 *   different characters (space → '.' → 'o' → 'O' → '#' → '@') and
 *   alternate between dim and bold brightness.  The resulting high-
 *   contrast boundaries between bands appear as visible rings that
 *   travel outward with the wavefront.
 *
 *   With multiple sources active, the bands from each source cross and
 *   interfere: where two wavefronts add constructively the ring boundary
 *   appears brighter; where they cancel destructively a dark nodal line
 *   forms between rings.  This mode makes interference fringes the most
 *   visually striking of the three modes.
 */

/* PRESSURE mode: signed diverging heatmap.
 * Shows compression (red) and rarefaction (blue) zones.
 * Best for understanding the instantaneous wave structure. */
static void render_pressure(const Wave *w, int cols, int rows)
{
    int   nx  = w->nx, ny = w->ny;
    float inv = 1.0f / w->p_max;
    chtype cur_attr = A_NORMAL;
    attrset(cur_attr);

    for (int sy = 0; sy < ny && sy < rows; sy++) {
        int gy = ny - 1 - sy;
        for (int x = 0; x < nx && x < cols; x++) {
            float pn = w->p[IDX(w, x, gy)] * inv;
            if (pn >  1.0f) pn =  1.0f;
            if (pn < -1.0f) pn = -1.0f;

            int cp; char ch; attr_t at;
            if      (pn < -0.65f) { cp=CP_PN2;   ch='#'; at=A_BOLD;   }
            else if (pn < -0.30f) { cp=CP_PN1;   ch='x'; at=A_NORMAL; }
            else if (pn < -0.07f) { cp=CP_PN0;   ch=':'; at=A_NORMAL; }
            else if (pn <  0.07f) { cp=CP_PZERO; ch=' '; at=A_DIM;    }
            else if (pn <  0.30f) { cp=CP_PP0;   ch=':'; at=A_NORMAL; }
            else if (pn <  0.60f) { cp=CP_PP1;   ch='x'; at=A_NORMAL; }
            else                  { cp=CP_PP2;   ch='#'; at=A_BOLD;   }

            chtype a = (chtype)COLOR_PAIR(cp) | at;
            if (a != cur_attr) { attrset(a); cur_attr = a; }
            mvaddch(sy + 1, x, (chtype)ch);
        }
    }
    attrset(A_NORMAL);
}

/* INTENSITY mode: |p| magnitude ramp.
 * Dark = quiet nodes (where waves cancel), bright = antinodes (loud).
 * Standing-wave patterns are most clearly visible here. */
static void render_intensity(const Wave *w, int cols, int rows)
{
    int   nx  = w->nx, ny = w->ny;
    float inv = 7.0f / w->p_max;
    chtype cur_attr = A_NORMAL;
    attrset(cur_attr);

    for (int sy = 0; sy < ny && sy < rows; sy++) {
        int gy = ny - 1 - sy;
        for (int x = 0; x < nx && x < cols; x++) {
            float ap = fabsf(w->p[IDX(w, x, gy)]) * inv;
            int   lv = (int)ap;
            if (lv < 0) lv = 0;
            if (lv > 7) lv = 7;
            attr_t at = (lv >= 5) ? A_BOLD : A_NORMAL;
            chtype a  = (chtype)COLOR_PAIR(CP_INT0 + lv) | at;
            if (a != cur_attr) { attrset(a); cur_attr = a; }
            mvaddch(sy + 1, x, (chtype)DENS_CHARS[lv]);
        }
    }
    attrset(A_NORMAL);
}

/*
 * WAVEFRONT mode: |p| mapped to alternating-contrast bands.
 *
 * Each band uses a distinct char and alternates between dim/bold.
 * Adjacent bands with different brightness create a high-contrast
 * boundary at each pressure level — visually these are the wavefront
 * rings.  As the wave expands outward, the bands scroll with it,
 * making propagation clearly visible.
 *
 * With multiple sources, bands from different sources intersect,
 * producing visible interference fringes where waves constructively
 * or destructively add.
 */
#define N_WF_BANDS  6

static void render_wavefront(const Wave *w, int cols, int rows)
{
    int   nx  = w->nx, ny = w->ny;
    float inv = (float)N_WF_BANDS / w->p_max;

    /* Per-band: char, color-pair, attr — alternating dim/bright */
    static const char   WF_CH[N_WF_BANDS] = { ' ', '.', 'o', 'O', '#', '@' };
    static const int    WF_CP[N_WF_BANDS] = {
        CP_PZERO, CP_WFA, CP_WFB, CP_WFB, CP_WFC, CP_WFC };
    static const attr_t WF_AT[N_WF_BANDS] = {
        A_DIM, A_DIM, A_NORMAL, A_BOLD, A_NORMAL, A_BOLD };

    chtype cur_attr = A_NORMAL;
    attrset(cur_attr);

    for (int sy = 0; sy < ny && sy < rows; sy++) {
        int gy = ny - 1 - sy;
        for (int x = 0; x < nx && x < cols; x++) {
            float ap = fabsf(w->p[IDX(w, x, gy)]) * inv;
            int   b  = (int)ap;
            if (b < 0) b = 0;
            if (b >= N_WF_BANDS) b = N_WF_BANDS - 1;
            chtype a = (chtype)COLOR_PAIR(WF_CP[b]) | WF_AT[b];
            if (a != cur_attr) { attrset(a); cur_attr = a; }
            mvaddch(sy + 1, x, (chtype)WF_CH[b]);
        }
    }
    attrset(A_NORMAL);
}

static void render_field(const Wave *w, int cols, int rows)
{
    switch (w->vis_mode) {
    case VIS_PRESSURE:  render_pressure(w, cols, rows);  break;
    case VIS_INTENSITY: render_intensity(w, cols, rows); break;
    case VIS_WAVEFRONT: render_wavefront(w, cols, rows); break;
    }

    /* Source markers drawn on top of the field */
    for (int i = 0; i < MAX_SOURCES; i++) {
        if (!w->srcs[i].active) continue;
        int sx = (int)(w->srcs[i].nx_f * (float)(w->nx-1) + 0.5f);
        int sg = (int)(w->srcs[i].ny_f * (float)(w->ny-1) + 0.5f);
        int ss = (w->ny - 1 - sg) + 1;  /* screen row (offset by HUD row) */
        if (sx >= 0 && sx < cols && ss >= 1 && ss <= rows) {
            attron(COLOR_PAIR(CP_SRC) | A_BOLD);
            mvaddch(ss, sx, (chtype)('1' + i));
            attroff(COLOR_PAIR(CP_SRC) | A_BOLD);
        }
    }
}

/* ===================================================================== */
/* §8  render_overlay() — HUD                                             */
/* ===================================================================== */

/*
 * The HUD (heads-up display) occupies row 0 (top status bar) and the
 * last row (bottom key reference).  The main status bar shows:
 *
 *   [VIS_MODE]   current visualization name (PRESSURE / INTENSITY / WAVEFRONT)
 *   c=           current wave speed (dimensionless)
 *   dt=          current timestep (simulation time units per physics step)
 *   E=           total acoustic energy Σp² — watch this to see energy
 *                build up (reflecting mode) or decay (absorbing, no source)
 *   f_est=       estimated dominant frequency at the center probe (Hz in
 *                simulation units); derived from zero-crossing count
 *   srcs:[]      which sources are currently active (1-indexed)
 *   BC mode      REFLECTING or ABSORBING
 *   t=           accumulated simulation time
 *   fps          measured display frame rate
 *
 * Resolution estimates displayed implicitly via dt and c:
 *   Temporal resolution: the simulation resolves features down to
 *     Δt = dt per step.  The highest frequency representable is the
 *     Nyquist limit f_max = 1/(2·dt).
 *   Spatial resolution: the grid resolves features down to Δx = dx.
 *     The shortest resolvable wavelength is λ_min = 2·dx (Nyquist in
 *     space).  Ring spacings below 2 grid cells will alias.
 *   Together: the highest stable frequency satisfies
 *     f_max · λ_min = c  →  f_max = c / (2·dx).
 *   LAMBDA_CELLS=10 keeps source frequencies well below this limit.
 */
static void render_overlay(const Wave *w, int cols, int rows, double fps)
{
    /* Build compact source-list string e.g. "1 3" */
    char src_str[16] = { 0 };
    int  si = 0;
    for (int i = 0; i < MAX_SOURCES; i++) {
        if (w->srcs[i].active) {
            src_str[si++] = '1' + i;
            src_str[si++] = ' ';
        }
    }
    if (si == 0) { src_str[0]='-'; src_str[1]='\0'; }
    else          { src_str[si > 0 ? si-1 : 0] = '\0'; }

    char buf[320];
    snprintf(buf, sizeof buf,
        " ACOUSTIC [%s] | c=%.2f  dt=%.4f"
        " | E=%.2e | f_est=%.3f Hz | srcs:[%s] | %s | t=%.1f | %.0ffps ",
        VIS_NAMES[w->vis_mode],
        w->c, w->dt,
        (double)w->energy, (double)w->freq_est,
        src_str, BC_NAMES[w->bc_mode],
        (double)w->time, fps);

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvaddnstr(0, 0, buf, cols);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    attron(COLOR_PAIR(CP_HUD) | A_DIM);
    mvprintw(rows-1, 0,
        " q:quit  spc:pause  v:pressure  i:intensity  w:wavefront"
        "  1-4:src  b:BC  +/-:speed  p:impulse  r:reset ");
    attroff(COLOR_PAIR(CP_HUD) | A_DIM);

    if (w->paused) {
        attron(COLOR_PAIR(CP_PP2) | A_BOLD);
        mvprintw(rows/2, cols/2-4, " PAUSED ");
        attroff(COLOR_PAIR(CP_PP2) | A_BOLD);
    }
}

/* ===================================================================== */
/* §9  scene                                                              */
/* ===================================================================== */

typedef struct { Wave wave; } Scene;

static void scene_init(Scene *sc, int cols, int rows)
{
    Wave *w = &sc->wave;
    int nx  = cols   < GRID_MAX_X ? cols   : GRID_MAX_X;
    int ny  = rows-2 < GRID_MAX_Y ? rows-2 : GRID_MAX_Y;
    if (ny < 4) ny = 4;
    if (wave_alloc(w, nx, ny) != 0) return;
    wave_init(w, nx, ny);
}

static void scene_free(Scene *sc) { wave_free(&sc->wave); }

static void scene_resize(Scene *sc, int cols, int rows)
{
    Wave *w = &sc->wave;
    int vm = w->vis_mode, bc = w->bc_mode;
    Source saved[MAX_SOURCES];
    memcpy(saved, w->srcs, sizeof saved);

    wave_free(w);
    memset(w, 0, sizeof *w);

    int nx = cols   < GRID_MAX_X ? cols   : GRID_MAX_X;
    int ny = rows-2 < GRID_MAX_Y ? rows-2 : GRID_MAX_Y;
    if (ny < 4) ny = 4;
    if (wave_alloc(w, nx, ny) != 0) return;
    wave_init(w, nx, ny);
    w->vis_mode = vm;
    w->bc_mode  = bc;
    /* Restore active flags (positions/freqs recomputed for new grid size) */
    for (int i = 0; i < MAX_SOURCES; i++)
        w->srcs[i].active = saved[i].active;
}

static void scene_tick(Scene *sc)
{
    Wave *w = &sc->wave;
    if (w->paused || !w->p) return;
    update_wave(w);
    apply_boundary(w);
}

static void scene_draw(const Scene *sc, int cols, int rows, double fps)
{
    erase();
    render_field(&sc->wave, cols, rows - 2);
    render_overlay(&sc->wave, cols, rows, fps);
}

/* ===================================================================== */
/* §10  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak();
    curs_set(0); nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s)  { (void)s; endwin(); }
static void screen_resize(Screen *s)
{
    endwin(); refresh(); getmaxyx(stdscr, s->rows, s->cols);
}

/* ===================================================================== */
/* §11  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;
static void on_exit(int s)   { (void)s; g_app.running    = 0; }
static void on_resize(int s) { (void)s; g_app.need_resize = 1; }
static void cleanup(void)    { endwin(); }

static bool handle_key(App *app, int ch)
{
    Wave *w = &app->scene.wave;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ':  w->paused = !w->paused; break;
    case 'v': case 'V': w->vis_mode = VIS_PRESSURE;  break;
    case 'i': case 'I': w->vis_mode = VIS_INTENSITY; break;
    case 'w': case 'W': w->vis_mode = VIS_WAVEFRONT; break;
    case '1': w->srcs[0].active = !w->srcs[0].active; break;
    case '2': w->srcs[1].active = !w->srcs[1].active; break;
    case '3': w->srcs[2].active = !w->srcs[2].active; break;
    case '4': w->srcs[3].active = !w->srcs[3].active; break;
    case 'b': case 'B':
        w->bc_mode = (w->bc_mode + 1) % 2;
        break;
    case '+': case '=':
        w->c += WAVE_C_STEP;
        if (w->c > WAVE_C_MAX) w->c = WAVE_C_MAX;
        wave_recompute_dt(w);
        /* Recompute source frequencies for new c */
        { float bf = w->c / (LAMBDA_CELLS * w->dx);
          for (int i = 0; i < MAX_SOURCES; i++)
              w->srcs[i].freq = bf * SRC_FMUL[i]; }
        break;
    case '-':
        w->c -= WAVE_C_STEP;
        if (w->c < WAVE_C_MIN) w->c = WAVE_C_MIN;
        wave_recompute_dt(w);
        { float bf = w->c / (LAMBDA_CELLS * w->dx);
          for (int i = 0; i < MAX_SOURCES; i++)
              w->srcs[i].freq = bf * SRC_FMUL[i]; }
        break;
    case 'p': case 'P': {
        /* Gaussian pressure impulse at centre — excites all room modes */
        int cx = w->nx/2, cy = w->ny/2;
        int R  = (int)(LAMBDA_CELLS * 0.5f);
        if (R < 2) R = 2;
        for (int y = cy-R; y <= cy+R; y++) {
            if (y < 1 || y >= w->ny-1) continue;
            for (int x = cx-R; x <= cx+R; x++) {
                if (x < 1 || x >= w->nx-1) continue;
                float ddx = (float)(x-cx), ddy = (float)(y-cy);
                float r2  = (ddx*ddx + ddy*ddy) / (float)(R*R);
                w->p[IDX(w,x,y)] += 2.5f * expf(-r2 * 4.0f);
            }
        }
        break;
    }
    case 'r': case 'R':
        scene_resize(&app->scene, app->screen.cols, app->screen.rows);
        break;
    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit);
    signal(SIGTERM,  on_exit);
    signal(SIGWINCH, on_resize);

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time = clock_ns();
    int64_t fps_accum  = 0;
    int     fps_count  = 0;
    double  fps_disp   = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────────── */
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_resize(&app->scene, app->screen.cols, app->screen.rows);
            app->need_resize = 0;
            frame_time = clock_ns();
        }

        /* ── wall-clock dt ───────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 200 * NS_PER_MS) dt = 200 * NS_PER_MS;

        /* ── physics steps ───────────────────────────── */
        for (int s = 0; s < STEPS_PER_FRAME; s++)
            scene_tick(&app->scene);

        /* ── fps tracking ────────────────────────────── */
        fps_count++;
        fps_accum += dt;
        if (fps_accum >= 500 * NS_PER_MS) {
            fps_disp  = (double)fps_count
                      / ((double)fps_accum / (double)NS_PER_SEC);
            fps_count = 0; fps_accum = 0;
        }

        /* ── sleep to target 30 fps ──────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 30 - elapsed);

        /* ── render ──────────────────────────────────── */
        scene_draw(&app->scene, app->screen.cols, app->screen.rows, fps_disp);
        wnoutrefresh(stdscr);
        doupdate();

        /* ── input ───────────────────────────────────── */
        int key;
        while ((key = getch()) != ERR)
            if (!handle_key(app, key)) { app->running = 0; break; }
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
