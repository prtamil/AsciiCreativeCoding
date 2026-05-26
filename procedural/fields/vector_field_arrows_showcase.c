/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * vector_field_arrows_showcase.c
 *   — Visualise 2-D vector fields f(x, y) → (u, v) as ASCII arrows.
 *     30 patterns spanning gradient fields, classical analytic fields,
 *     physics fields, divergence-free flows, dynamical-system phase
 *     portraits, and time-varying fields.
 *
 * DEMO: Every cell carries a 2-D vector v = (u, v).  The active
 *       pattern decides how to compute it; the renderer maps:
 *         direction → 8-way ASCII arrow glyph via atan2(vy, vx)
 *         magnitude → palette band brightness (0..3)
 *       Result: a field of arrows showing what a particle at each
 *       cell would feel.  Watch radial sources spray, vortices spin,
 *       saddle points push two ways at once, and ∇f always point
 *       uphill on a scalar field.
 *
 *       30 patterns in 6 tiers (cycle with n / p):
 *         Tier 1 GRADIENT  — ∇f for chosen scalars (paraboloid,
 *                            saddle, periodic, ripple, noise) — the
 *                            BRIDGE: scalar field → vector field
 *         Tier 2 ANALYTIC  — classical 2-D fields (radial source/
 *                            sink, rotation, shear, uniform diagonal)
 *         Tier 3 PHYSICS   — Coulomb point charge, electric dipole,
 *                            current-wire magnetic, gravity (1/r²),
 *                            quadrupole
 *         Tier 4 SOLENOID  — divergence-free / incompressible flows
 *                            (curl noise, stream function vortex
 *                            grid, vortex pair, Poiseuille channel,
 *                            noisy uniform)
 *         Tier 5 DYNAMICS  — 2-D ODE phase portraits (stable node,
 *                            stable spiral, Hopf limit cycle, Van der
 *                            Pol, nonlinear pendulum)
 *         Tier 6 ANIMATED  — time-varying (rotating dipole,
 *                            travelling wave, breathing radial,
 *                            orbiting vortex, drifting curl noise)
 *
 * Study alongside:
 *   ./curl_noise_vector_field.c     — specific curl-noise demo.  This
 *       file includes CURL_NOISE as one of 30; the dedicated demo
 *       goes deeper on that single technique.
 *   ./flow_field_particles.c        — particles riding a vector field.
 *       This file draws the FIELD itself; that one draws what flows on it.
 *   ./magnetic_fields.c             — physics magnetic field with a
 *       different visualisation strategy (lines, not per-cell arrows).
 *   ./perin_noise_flow_showcase.c   — Perlin-gradient flow.
 *
 * Section map:
 *   §1 config   — grid bounds, T constants, themes, HUD widths,
 *                 sim/render budgets, MOTION-probe parameters
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD pairs + 10 themes (PAIR_BAND_BASE+0..3)
 *   §5 vector   — NoiseField + scalar / gradient / curl primitives
 *   §6 arrow    — 8-direction ASCII arrow picker + magnitude band
 *   §7 patterns — 30 vector-field visualisations + dispatch table +
 *                 MOTION layer (motion_probe_position, motion_spiral_dir,
 *                 pattern_emit_arrow)
 *   §8 scene    — ArrowGrid + PatternState + PaletteState + Scene;
 *                 per-frame tick + evaluate
 *   §9 screen   — viewport + ArrowGrid renderer + HUD layout helpers
 *   §10 app     — signals, resize, key dispatch, main game loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume animation
 *   r          reset (new noise seed)
 *   n / N      next pattern   (p / P previous)
 *   t / T      next / previous theme
 *   m          toggle MOTION — compose a global stable-spiral
 *              attractor centred on a noise-driven wandering probe
 *              with each T1-T5 pattern's static direction; T6
 *              unaffected (already animated)
 *   + / =      faster animation drift (× 2)
 *   -          slower animation drift (/ 2)
 *   ] / [      raise / lower simulation tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra vector_field_arrows_showcase.c \
 *       -o vector_field_arrows -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Per-cell direct vector field sampling.  Each cell
 *                  computes a (vx, vy) from the active pattern's
 *                  formula — analytic closed-form, scalar gradient
 *                  via finite difference, curl of a potential, or
 *                  ODE phase-space velocity.  Direction is binned to
 *                  one of 8 ASCII glyphs via atan2(vy, vx); magnitude
 *                  is quantised to 4 palette bands.
 *
 * Data-structure : Per-cell render output buffers (glow, band, glyph)
 *                  on a Field struct.  No persistent vector buffer —
 *                  patterns compute (vx, vy) on the fly and emit the
 *                  arrow glyph + band directly.  A small NoiseField
 *                  holds the seed used by Tier 1 GRAD_NOISE and Tier
 *                  4 CURL_NOISE (re-rolled on r).
 *
 * Rendering      : ASCII-only.  8 directional glyphs:
 *                      east  → '>'    west  → '<'
 *                      north → '^'    south → 'v'
 *                      NE / SW → '/'  (slope-ambiguous: sign carried
 *                                       by flow context)
 *                      NW / SE → '\'  (same caveat)
 *                  Magnitude below ARROW_DEAD_ZONE renders as '.'
 *                  in band 0 — marks field zeros and equilibrium
 *                  points where direction is meaningless.  The bin
 *                  formula is exact: bin = ((round(atan2/(π/4))) + 8) & 7.
 *
 * Performance    : O(W · H) per frame.  atan2 + sqrt per cell ≈ 50 ns
 *                  on modern hardware → 0.6 ms for an 11K-cell grid;
 *                  comfortable at 60 Hz.  Tier 1 GRAD_NOISE and Tier 4
 *                  CURL_NOISE add 4-6 noise samples per cell for the
 *                  finite-difference derivative, still well under budget.
 *
 * References     : VECTOR FIELD VISUALISATION
 *                  • Helman, J. & Hesselink, L. (1989) — "Representation
 *                    and Display of Vector Field Topology in Fluid Flow
 *                    Data Sets", IEEE Computer 22(8).  Foundational —
 *                    classifies the kinds of critical points (sources,
 *                    sinks, saddles, centres, spirals) that Tiers 2-5
 *                    of this file enumerate by example.
 *                  • Cabral, B. & Leedom, L. C. (1993) — "Imaging
 *                    Vector Fields Using Line Integral Convolution",
 *                    SIGGRAPH'93.  The other major vector-field
 *                    visualisation school (texture advection along
 *                    streamlines) — useful contrast to the per-cell
 *                    arrow-glyph approach here.
 *                  • Bridson, R., Hourihan, J., Nordenstam, M. (2007) —
 *                    "Curl-Noise for Procedural Fluid Flow", SIGGRAPH'07.
 *                    Source for Tier 4 CURL_NOISE — divergence-free
 *                    flows from a noise potential via the curl operator.
 *
 *                  PROCEDURAL NOISE (Tier 1 GRAD_NOISE / Tier 4 CURL_NOISE)
 *                  • Perlin, K. (1985) — "An Image Synthesizer",
 *                    SIGGRAPH'85.  Foundational noise paper; value
 *                    noise (used in §5) is its simpler cousin —
 *                    hash lattice corners, bilerp with smoothstep.
 *                  • Quilez, I. — "Curl noise" and "2D distance
 *                    functions":
 *                    https://iquilezles.org/articles/curlnoise/
 *
 *                  DYNAMICAL SYSTEMS (Tier 5)
 *                  • Strogatz, S. H. (1994) — "Nonlinear Dynamics
 *                    and Chaos".  Standard reference for the phase
 *                    portraits drawn in Tier 5 (stable node, spiral,
 *                    Hopf bifurcation, Van der Pol, pendulum); also
 *                    the source of the spiral the MOTION layer
 *                    composes on top of every pattern.
 *                  • Van der Pol, B. (1926) — "On 'relaxation
 *                    oscillations'", Philosophical Magazine.  Original
 *                    paper for the limit-cycle equation in Tier 5.
 *
 *                  CLASSICAL PHYSICS FIELDS (Tier 3)
 *                  • Griffiths, D. J. — "Introduction to Electro-
 *                    dynamics" (4th ed.).  Chapter on multipole
 *                    expansion is the textbook treatment of the
 *                    POINT_CHARGE, DIPOLE, QUADRUPOLE, and
 *                    WIRE_MAGNETIC fields enumerated in Tier 3.
 *
 *                  ASCII RENDERING (§6 arrow encoder)
 *                  • Bourke, P. — "Character representation of grey
 *                    scale images":
 *                    http://paulbourke.net/dataformats/asciiart/
 *
 *                  INTERACTIVE INSPIRATION (MOTION layer)
 *                  • Treacy, E. — "material-vector-field" p5.js sketch:
 *                    http://winkervsbecks.github.io/material-vector-field/
 *                    Direct inspiration for the MOTION spiral compose —
 *                    same stable-spiral linear field, but parameterised
 *                    by a noise-driven probe instead of the mouse.
 *
 *                  COMPARE IN PROJECT
 *                  • ./curl_noise_vector_field.c — dedicated curl-noise
 *                    demo; one Tier 4 pattern here is its 30-pattern
 *                    cousin.
 *                  • ./flow_field_particles.c — particles ON a vector
 *                    field; this file shows the field ITSELF.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A vector field assigns a 2-D arrow (direction + magnitude) to every
 * point in the plane.  We sample on a grid and draw each arrow as a
 * single ASCII character whose orientation matches the direction.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine releasing a fleet of tiny tokens, one per cell.  The field
 * tells each token which way to drift and how hard to push.  Freeze
 * the moment of release and ink an arrow showing each token's
 * instantaneous velocity.  Where the field has a SINK, all arrows
 * point inward; a SOURCE, outward; a SADDLE, half push one way and
 * half the other; a CENTRE, arrows curl around it.  The arrows ARE
 * the field — they are not particles in motion.
 *
 * COORDINATE CONVENTION
 * ─────────────────────
 * Screen Y grows DOWN (ncurses convention).  All patterns compute
 * vectors in SCREEN coordinates: (vx > 0, vy > 0) means rightward +
 * DOWNWARD on screen.  Textbook physics/math formulas usually assume
 * Y-up; when porting, NEGATE vy.  Tier 5 phase-portrait patterns
 * document this explicitly per-pattern.
 *
 * ALGORITHM IN STEPS  (per cell, per frame)
 * ──────────────────
 *  1. Pattern computes (vx, vy) for cell (x, y), optionally using
 *     time t (Tier 6) and the NoiseField seed (Tier 1.5 / 4.1).
 *  2. Compute magnitude |v| = √(vx² + vy²).
 *  3. Normalise |v| to [0, 1] via a pattern-specific scale
 *     (typically  |v|/(|v|+s)  or  tanh(|v|/s) ).
 *  4. If |v|_norm < ARROW_DEAD_ZONE: draw '.' in band 0 (this is a
 *     field zero — direction is meaningless).
 *  5. Else: bin = (round(atan2(vy, vx) / (π/4)) + 8) mod 8;
 *           glyph = ARROW_GLYPHS[bin];
 *           band  = quartile of |v|_norm.
 *  6. Renderer paints (glyph, band) at the cell.
 *
 * KEY FORMULAS
 * ────────────
 *  Bin index           : bin = (round(atan2(vy, vx) / (π/4)) + 8) & 7
 *  Gradient (central
 *  finite difference,
 *  h = 1)              : ∂f/∂x ≈ (f(x+1, y) - f(x-1, y)) / 2
 *                        ∂f/∂y ≈ (f(x, y+1) - f(x, y-1)) / 2
 *  2-D curl of φ       : v = (∂φ/∂y, -∂φ/∂x)
 *                        (divergence-free by construction)
 *  Inverse-square      : v = -k · (dx, dy) / (dx² + dy²)^1.5
 *  Magnitude saturator : |v|_norm = |v| / (|v| + s_scale)
 *                        — bounded in [0, 1), avoids ∞ near singularities
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Singularities (point charges exactly at the cell location): add
 *    ε to denominator squared so r² + ε > 0.
 *  • Tiny |v|: atan2 angle is meaningless — DEAD_ZONE gives '.'.
 *  • Slope glyphs '/' and '\' don't carry sign — NE and SW share '/',
 *    NW and SE share '\'.  Surrounding flow disambiguates.
 *  • Screen Y-down: a "northward" math vector has negative vy here.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • RADIAL_OUT: top-centre cell is '^' (up); bottom-right is '\' (SE).
 *  • ROTATION (textbook CCW): visually CW on screen because of Y-flip;
 *                             top moves right.
 *  • POINT_CHARGE: arrows radiate from screen centre, dimming with 1/r².
 *  • CURL_NOISE: arrows curve smoothly, no obvious sources / sinks.
 *  • STABLE_NODE: every arrow points toward screen centre.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    CELLS_MAX         = MAP_W_MAX * MAP_H_MAX,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    HUD_COLS          =  80,
    FPS_UPDATE_MS     = 500,

    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,
};

/* HUD layout — top carries data, bottom carries actions. */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* HUD row-1 column widths — MUST match the printf format-string widths
 * the corresponding hud_field_* helpers emit, otherwise consecutive
 * fields overlap or leave gaps. */
#define HUD_PATTERN_FIELD_W     21
#define HUD_TIER_FIELD_W        15
#define HUD_THEME_FIELD_W       17
#define HUD_PALETTE_LABEL_W      9
#define HUD_N_PALETTE_BANDS      4

/* Viewport floor — the algorithm needs at least one MS cell, but
 * anything tinier than this is unreadable. */
#define MAP_W_MIN               16
#define MAP_H_MIN                8

/* Render-loop budget — sleep target for the per-frame throttle. */
#define RENDER_FPS_TARGET       60
#define RENDER_FRAME_BUDGET_NS  (NS_PER_SEC / RENDER_FPS_TARGET)

/* Spiral-of-death guard for the fixed-timestep accumulator: cap any
 * single frame's dt so a slow terminal can't trigger unbounded
 * sim-tick catch-up.  See Fiedler "Fix Your Timestep!". */
#define SIM_MAX_FRAME_DT_MS    100

/* Drift multiplier — cranked by +/-. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      4
#define DRIFT_MULT_MAX      16

/* Per-cell field-time drift increment per second (radians/sec
 * conceptually — used by Tier 6 animated patterns). */
#define FIELD_DRIFT         1.0f

/* Magnitude below this normalised threshold renders as '.' instead
 * of a directional arrow.  Marks field zeros / equilibrium points. */
#define ARROW_DEAD_ZONE     0.06f

/* Bin width for arrow direction binning (8 cardinal+diagonal bins). */
#define ARROW_BIN_WIDTH     ((float)(M_PI / 4.0))

/* Noise — value-noise field used by GRAD_NOISE (Tier 1) and CURL_NOISE
 * (Tier 4).  Frequency scale is small so the noise is smooth across
 * several cells; the finite-difference gradient stays well-behaved. */
#define NOISE_FREQ          0.10f

/* Tier 1 GRADIENT — knobs for the periodic and ripple scalars. */
#define GRAD_PERIODIC_FREQ  0.25f
#define GRAD_RIPPLE_FREQ    0.30f

/* Tier 3 PHYSICS — dipole half-separation (cells) and singularity
 * softening ε (avoids divide-by-zero at charge centres). */
#define DIPOLE_HALF_SEP     8.0f
#define QUADRUPOLE_HALF     6.0f
#define COULOMB_SOFT_EPS    1.5f

/* Tier 4 SOLENOID — stream-function vortex grid frequency. */
#define STREAM_FREQ_X       0.10f
#define STREAM_FREQ_Y       0.20f

/* Tier 5 DYNAMICS — phase-space half-extent each pattern uses to map
 * screen coords to its native phase-space domain. */
#define PHASE_HALF_EXTENT       3.0f      /* most patterns: x_p, y_p ∈ [-3, 3] */
#define PENDULUM_X_HALF_EXTENT  3.14159f  /* pendulum: x_p ∈ [-π, π]            */
#define VDP_MU                  1.0f      /* Van der Pol non-linearity         */
#define HOPF_MU                 1.0f      /* Hopf limit-cycle radius² parameter */

/* Tier 6 ANIMATED — period constants (seconds). */
#define ROT_DIPOLE_PERIOD   6.0f
#define TRAVEL_WAVE_PERIOD  4.0f
#define BREATHE_PERIOD      4.0f
#define ORBIT_PERIOD        8.0f
#define ORBIT_RADIUS_FRAC   0.30f         /* fraction of half-diagonal */

/* ---------- MOTION (m key) — pattern + stable-spiral composition ---- *
 *
 * Inspired by Eric Treacy's p5.js material-vector-field sketch
 *   http://winkervsbecks.github.io/material-vector-field/
 * but with a NOISE-DRIVEN probe in place of the mouse — hands-free,
 * organic motion across the entire viewport.
 *
 * When MOTION is on, every cell's vector is the (weighted) SUM of two
 * unit-length contributions:
 *
 *   1. pattern_dir  — the static field's direction at this cell
 *                     (the currently-selected preset's character)
 *   2. spiral_dir   — direction of a stable-spiral attractor centred
 *                     on the noise-driven probe
 *
 *     out = pattern_dir + MOTION_SPIRAL_WEIGHT · spiral_dir
 *
 * Composition (not replacement) is what keeps the active preset
 * visually distinct under MOTION — a uniform-tilted field plus
 * spiral looks completely different from a quadrupole plus spiral.
 *
 * Spiral math (Eric Treacy's calcVec, eigenvalues -1 ± i):
 *
 *     v(dx, dy) = (dy - dx,  -dx - dy)   where (dx, dy) = cell - probe
 *
 * Probe position: noise-driven smooth wander across most of the
 * screen.  Two decorrelated 1D noise channels feed (px, py) so the
 * probe roams organically, not on a predictable orbit. */
#define MOTION_SPIRAL_WEIGHT  1.0f         /* spiral vs pattern weight (1.0 = equal mix) */
#define PROBE_EXTENT_FRAC     0.40f        /* wander within central 80% of screen        */
#define PROBE_DRIFT_RATE      0.12f        /* noise sample-step per second of t          */

/* Magnitude saturator scales — per pattern.  |v|_norm = |v|/(|v|+s).
 * Larger s → field has to be stronger before saturating to band 3. */
#define SCALE_RADIAL_HALFDIAG  1.0f       /* normalize by half-diagonal */
#define SCALE_GRADIENT         0.5f       /* gradients of scalars       */
#define SCALE_NOISE_GRAD       2.0f       /* noise gradients are small  */
#define SCALE_INV_SQ           0.4f       /* 1/r² fields                */
#define SCALE_INV_R            1.5f       /* 1/r fields (magnetic wire) */
#define SCALE_BOUNDED          0.8f       /* sin/cos bounded outputs    */
#define SCALE_PHASE            2.0f       /* phase-portrait velocities  */

/*
 * Pattern — 30 vector-field visualisations in 6 complexity tiers.
 * Enum order MUST match `vector_patterns[]` in §7 (compiler enforces
 * via fixed-size [N_PATTERNS] initialiser).
 *
 *   Tier 1 GRADIENT  : GRAD_PARABOLOID, GRAD_SADDLE, GRAD_PERIODIC,
 *                      GRAD_RIPPLE, GRAD_NOISE
 *   Tier 2 ANALYTIC  : RADIAL_OUT, RADIAL_IN, ROTATION, SHEAR_X,
 *                      UNIFORM_TILTED
 *   Tier 3 PHYSICS   : POINT_CHARGE, DIPOLE, WIRE_MAGNETIC,
 *                      GRAVITY, QUADRUPOLE
 *   Tier 4 SOLENOID  : CURL_NOISE, STREAM_GRID, VORTEX_PAIR,
 *                      CHANNEL_FLOW, NOISY_UNIFORM
 *   Tier 5 DYNAMICS  : STABLE_NODE, STABLE_SPIRAL, HOPF_CYCLE,
 *                      VAN_DER_POL, PENDULUM
 *   Tier 6 ANIMATED  : ROTATING_DIPOLE, TRAVELLING_WAVE,
 *                      BREATHING_RADIAL, ORBITING_VORTEX, DRIFT_CURL
 */
typedef enum {
    /* Tier 1 — GRADIENT: ∇f for scalar f */
    PATTERN_GRAD_PARABOLOID = 0,
    PATTERN_GRAD_SADDLE,
    PATTERN_GRAD_PERIODIC,
    PATTERN_GRAD_RIPPLE,
    PATTERN_GRAD_NOISE,
    /* Tier 2 — ANALYTIC: classical 2-D fields */
    PATTERN_RADIAL_OUT,
    PATTERN_RADIAL_IN,
    PATTERN_ROTATION,
    PATTERN_SHEAR_X,
    PATTERN_UNIFORM_TILTED,
    /* Tier 3 — PHYSICS: real-world named fields */
    PATTERN_POINT_CHARGE,
    PATTERN_DIPOLE,
    PATTERN_WIRE_MAGNETIC,
    PATTERN_GRAVITY,
    PATTERN_QUADRUPOLE,
    /* Tier 4 — SOLENOID: divergence-free flows */
    PATTERN_CURL_NOISE,
    PATTERN_STREAM_GRID,
    PATTERN_VORTEX_PAIR,
    PATTERN_CHANNEL_FLOW,
    PATTERN_NOISY_UNIFORM,
    /* Tier 5 — DYNAMICS: 2-D ODE phase portraits */
    PATTERN_STABLE_NODE,
    PATTERN_STABLE_SPIRAL,
    PATTERN_HOPF_CYCLE,
    PATTERN_VAN_DER_POL,
    PATTERN_PENDULUM,
    /* Tier 6 — ANIMATED: time-varying */
    PATTERN_ROTATING_DIPOLE,
    PATTERN_TRAVELLING_WAVE,
    PATTERN_BREATHING_RADIAL,
    PATTERN_ORBITING_VORTEX,
    PATTERN_DRIFT_CURL,
    N_PATTERNS,
} Pattern;

/* Forward decls — definitions live in §7 alongside the dispatch table. */
static const char *pattern_name(Pattern p);
static const char *pattern_tier(Pattern p);

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Theme — a named 4-colour band ramp defining one visual style.
 *
 * INTENT
 *   Decouple "which colours" from "where they're used".  §7 patterns
 *   emit a band index ∈ {0..3} per cell; theme_apply() in §3 binds
 *   that index to one of the four xterm-256 colours stored here.
 *   Cycling themes (t/T) re-binds the ncurses pairs in place — no
 *   §5/§6/§7/§8 code has to know about colour at all.
 *
 * CONTEXT
 *   themes[N_THEMES] in §3 is the static table.  PaletteState.current
 *   on Scene is the active index.  theme_apply() pushes this row's
 *   colours into PAIR_BAND_BASE..+3; the renderer reads via
 *   COLOR_PAIR(PAIR_BAND_BASE + band).
 *
 * PALETTE LOGIC
 *   The 4 band slots form a perceptual gradient, NOT arbitrary colours:
 *     band[0] — dim     : low-magnitude cells (field zeros, '.' dots)
 *     band[1] — low     : weak field cells
 *     band[2] — mid     : the dominant on-screen brightness
 *     band[3] — bright  : strong field cells (saturating mag_norm)
 *   Every entry MUST sit ≥ 39 in the xterm-256 cube so even band 0
 *   stays clearly visible on default-black with A_BOLD — important
 *   here because the MOTION layer only changes arrow DIRECTION (not
 *   magnitude / band), so animation is only legible if even the
 *   low-magnitude cells are bright enough to see the glyph rotate.
 *
 * MEMBER LOGIC
 *   name    : ≤ 8-char ALL-CAPS label.  Stored as a literal pointer
 *             (no copy); HUD reads themes[palette.current].name
 *             directly.
 *   band[4] : xterm-256 colour indices for bands 0..3.  Monotone-
 *             brightness ramp; the RELATIVE gradient gives a theme
 *             its character, not absolute hue.
 *
 * REFERENCES
 *   • xterm-256 colour cube — indices 16..231 = 6×6×6 RGB cube,
 *     232..255 = 24-step grayscale ramp.
 *   • ncurses(3X) — start_color, init_pair, COLOR_PAIR.
 */
typedef struct {
    const char *name;
    short       band[4];        /* xterm-256 indices, dim → bright */
} Theme;

#define N_THEMES 10

/* Every band ≥ 39 in the xterm-256 cube so even band 0 stays clearly
 * legible — important here because the MOTION probe only changes arrow
 * DIRECTION (not magnitude/band), so animation is only visible if the
 * low-magnitude cells aren't lost in the dim end of the palette. */
static const Theme themes[N_THEMES] = {
    { "DEFAULT", {  75,  123,  220,  231 } },   /* sky-blue → cyan → yellow → white */
    { "MATRIX",  {  77,  118,  156,  194 } },   /* bright green ramp                */
    { "NOVA",    { 135,  171,  207,  219 } },   /* magenta → pink → pale pink       */
    { "MONO",    { 247,  250,  253,  255 } },   /* light-gray ramp                  */
    { "OCEAN",   {  81,  117,  159,  195 } },   /* cyan → pale cyan                 */
    { "FIRE",    { 208,  214,  220,  227 } },   /* orange → yellow → pale yellow    */
    { "EARTH",   { 143,  179,  215,  222 } },   /* tan → sand → cream               */
    { "FOREST",  { 114,  150,  157,  194 } },   /* sage → light sage → very light   */
    { "DESERT",  { 179,  215,  222,  229 } },   /* sand → cream → pale              */
    { "ARCTIC",  { 117,  159,  195,  231 } },   /* pale blue → near white           */
};

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
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_BAND_BASE + i, t->band[i], -1);
    } else {
        static const short fallback[4] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_BAND_BASE + i, fallback[i], -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  vector primitives — noise, scalar gradients, 2-D curl              */
/* ===================================================================== */

/*
 * NoiseField — per-run hash seed for the value-noise lattice that
 * backs GRAD_NOISE (Tier 1), CURL_NOISE (Tier 4), and the MOTION
 * probe's wander path.
 *
 * INTENT
 *   Give "the noise field" an explicit owner on Scene instead of a
 *   free file-scope global.  Re-rolling the seed on r-press refreshes
 *   every noise-driven pattern at once without disturbing the global
 *   rand() state (which would cascade into any other rand() user —
 *   e.g. the terminal-colour fallback init in §3).
 *
 * CONTEXT
 *   One instance lives on Scene (§8).  Mutated only by
 *   noise_field_reseed() — at scene_init, scene_reset (r-press),
 *   and SIGWINCH-driven app_do_resize.  Read implicitly by every
 *   call to noise_sample() / lattice_scalar() (Tier 1 GRAD_NOISE,
 *   Tier 4 CURL_NOISE, the MOTION probe) via the file-scope
 *   g_lattice_seed mirror.
 *
 * MEMORY
 *   One uint32_t.  No allocation.
 *
 * IMPLEMENTATION
 *   The lattice hash inside lattice_scalar() is hot enough that we
 *   thread the seed through a file-scope variable rather than passing
 *   a NoiseField pointer through every per-cell ScalarFn2D call.
 *   noise_field_reseed() is the only writer; g_lattice_seed is
 *   private to §5.
 *
 * MEMBER LOGIC
 *   seed : 32-bit hash seed mixed into the lattice-corner hash via
 *          `+ g_lattice_seed`.  Re-rolled at every reset as the xor
 *          of two rand() draws; the xor diffuses any rand()
 *          low-bit-quality issues.
 *
 * REFERENCES
 *   • Perlin, K. (1985) — "An Image Synthesizer", SIGGRAPH'85.
 *     Foundational noise paper.  Value noise (used here) is a simpler
 *     hash-lattice variant of Perlin's gradient noise.
 *   • Wang, T. (1997) — "Integer hash function".  hash32() is the
 *     same family of multiply-xor mixers used in many value-noise
 *     implementations to avoid storing a permutation table.
 */
typedef struct {
    uint32_t seed;
} NoiseField;

static uint32_t g_lattice_seed = 0;     /* set only by noise_field_reseed */

static inline uint32_t hash32(uint32_t x)
{
    x = (x ^ (x >> 16)) * 0x7feb352du;
    x = (x ^ (x >> 15)) * 0x846ca68bu;
    x = (x ^ (x >> 16));
    return x;
}

static void noise_field_reseed(NoiseField *nf)
{
    nf->seed       = (uint32_t)rand() ^ ((uint32_t)rand() << 16);
    g_lattice_seed = nf->seed;
}

/* Lattice corner scalar ∈ [0, 1] — same (xi, yi, seed) → same value. */
static inline float lattice_scalar(int xi, int yi)
{
    uint32_t h = (uint32_t)xi * 374761393u
               + (uint32_t)yi * 668265263u
               + g_lattice_seed;
    return (float)(hash32(h) >> 8) * (1.0f / 16777215.0f);
}

static inline float smoothstep01(float t) { return t * t * (3.0f - 2.0f * t); }
static inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }

/* Smooth value noise at continuous (x, y) — bilinear blend of the
 * 4 surrounding lattice corners, smoothstep interpolants. */
static float noise_sample(float x, float y)
{
    int   xi = (int)floorf(x);
    int   yi = (int)floorf(y);
    float ux = smoothstep01(x - (float)xi);
    float uy = smoothstep01(y - (float)yi);
    float v00 = lattice_scalar(xi,     yi);
    float v10 = lattice_scalar(xi + 1, yi);
    float v01 = lattice_scalar(xi,     yi + 1);
    float v11 = lattice_scalar(xi + 1, yi + 1);
    return lerpf(lerpf(v00, v10, ux), lerpf(v01, v11, ux), uy);
}

/*
 * Central finite-difference gradient of any scalar f at (x, y).
 *
 *   ∂f/∂x ≈ (f(x+h) - f(x-h)) / (2h)
 *   ∂f/∂y ≈ (f(y+h) - f(y-h)) / (2h)
 *
 * Step h = 1 cell.  Used by GRAD_NOISE and the §7 helpers that take
 * any scalar function.  The 2D curl helper below shares the same
 * derivative form.
 */
typedef float (*ScalarFn2D)(float x, float y);

static inline void scalar_gradient(ScalarFn2D f, float x, float y,
                                   float *vx, float *vy)
{
    *vx = 0.5f * (f(x + 1.0f, y) - f(x - 1.0f, y));
    *vy = 0.5f * (f(x, y + 1.0f) - f(x, y - 1.0f));
}

/*
 * 2-D curl of a scalar potential φ:
 *   v = (∂φ/∂y, -∂φ/∂x)
 *
 * By construction this v has zero divergence (∇·v = ∂vx/∂x + ∂vy/∂y
 * = ∂²φ/∂x∂y - ∂²φ/∂x∂y = 0).  Used by CURL_NOISE and DRIFT_CURL
 * (Tier 4 / Tier 6) for the divergence-free flow visualisations
 * characteristic of incompressible fluids.
 */
static inline void scalar_curl_2d(ScalarFn2D phi, float x, float y,
                                  float *vx, float *vy)
{
    /*  vx =  ∂φ/∂y     */
    *vx =  0.5f * (phi(x, y + 1.0f) - phi(x, y - 1.0f));
    /*  vy = -∂φ/∂x     */
    *vy = -0.5f * (phi(x + 1.0f, y) - phi(x - 1.0f, y));
}

/* Scalars used by Tier 1 GRADIENT patterns.  Defined here so the
 * §7 patterns can call scalar_gradient() on them. */
static float scalar_paraboloid (float x, float y) { return x * x + y * y; }
static float scalar_saddle     (float x, float y) { return x * x - y * y; }
static float scalar_periodic   (float x, float y)
{
    return sinf(x * GRAD_PERIODIC_FREQ) * cosf(y * GRAD_PERIODIC_FREQ);
}
static float scalar_ripple     (float x, float y)
{
    return cosf(GRAD_RIPPLE_FREQ * sqrtf(x * x + y * y));
}
static float scalar_noise      (float x, float y)
{
    return noise_sample(x * NOISE_FREQ, y * NOISE_FREQ);
}

/* ===================================================================== */
/* §6  arrow rendering — direction → glyph, magnitude → palette band      */
/* ===================================================================== */

/*
 * ARROW_GLYPHS — bin index → ASCII character.  Bins are atan2(vy, vx)
 * rounded to the nearest multiple of π/4.  Bin 0 = east (vx > 0,
 * vy ≈ 0), going CCW in atan2 sense (which is CW visually on screen
 * because Y grows DOWN).
 *
 *   bin 0   east       (atan2 ≈  0)            '>'
 *   bin 1   SE on scrn (atan2 ≈  π/4)          '\'   (math NE)
 *   bin 2   south scrn (atan2 ≈  π/2)          'v'   (math S)
 *   bin 3   SW on scrn (atan2 ≈  3π/4)         '/'   (math NW)
 *   bin 4   west       (atan2 ≈ ±π)            '<'
 *   bin 5   NW on scrn (atan2 ≈ -3π/4)         '\'   (math SW)
 *   bin 6   north scrn (atan2 ≈ -π/2)          '^'   (math N)
 *   bin 7   NE on scrn (atan2 ≈ -π/4)          '/'   (math SE)
 *
 * Slope ambiguity: '/' is shared between bin 3 and bin 7, '\' between
 * bins 1 and 5.  The surrounding flow disambiguates visually.
 */
static const char ARROW_GLYPHS[8] = {
    '>', '\\', 'v', '/', '<', '\\', '^', '/',
};

/* Bin formula: bin = (round(angle / (π/4)) + 8) mod 8.
 * The +8 then & 7 normalises negative-angle results into [0, 8). */
static inline int arrow_bin_from_angle(float angle)
{
    return ((int)floorf(angle / ARROW_BIN_WIDTH + 0.5f) + 8) & 7;
}

/* Magnitude saturator: |v| / (|v| + s).  Bounded in [0, 1), monotone,
 * with the half-saturation point at |v| = s.  Safe for 1/r and 1/r²
 * fields that blow up near singularities; the singularity just maps
 * to band 3 instead of overflowing. */
static inline float mag_saturate(float mag, float scale)
{
    return mag / (mag + scale);
}

/* Quartile bucket: normalised magnitude → band ∈ {0, 1, 2, 3}. */
static inline uint8_t mag_to_band(float mag_norm)
{
    int b = (int)(mag_norm * 3.999f);
    if (b < 0) b = 0;
    if (b > 3) b = 3;
    return (uint8_t)b;
}

/* "Field zero" marker — '.' in band 0 — for cells where direction
 * is meaningless because magnitude is tiny.  Visually marks
 * equilibria, nulls, and pattern centres. */
static inline void cell_emit_dot(float *gl, uint8_t *bn, char *gy)
{
    *gl = 1.0f; *bn = 0; *gy = '.';
}

/*
 * The arrow emitter — picks glyph from atan2(vy, vx) bin, picks
 * band from the supplied normalised magnitude.  Below DEAD_ZONE
 * the cell renders as a dot instead.
 */
static inline void cell_emit_arrow(float vx, float vy, float mag_norm,
                                   float *gl, uint8_t *bn, char *gy)
{
    if (mag_norm < ARROW_DEAD_ZONE) {
        cell_emit_dot(gl, bn, gy);
        return;
    }
    int bin = arrow_bin_from_angle(atan2f(vy, vx));
    *gl = 1.0f;
    *bn = mag_to_band(mag_norm);
    *gy = ARROW_GLYPHS[bin];
}

/* ===================================================================== */
/* §7  patterns — 30 vector-field visualisations + dispatch               */
/* ===================================================================== */

/*
 * Pattern signature — called PER CELL.  Receives the cell coords +
 * grid dims (so patterns can compute the screen centre), plus the
 * animation accumulator field_time (used by Tier 6).  Writes the
 * three per-cell outputs the §9 renderer reads:
 *   out_glow  — 1.0 = paint, 0.0 = skip
 *   out_band  — palette band ∈ {0..3}
 *   out_glyph — ASCII char to draw; 0 = skip
 *
 * Helper conventions inside patterns:
 *   cx, cy   — screen centre (w/2, h/2)
 *   dx, dy   — offset from centre (cell-units)
 *   r, r²    — radial distance + square; ε-softened where needed
 *   half_dg  — half-diagonal of the grid, used as a natural scale
 */
typedef void (*VectorPatternFn)(int x, int y, int w, int h, float field_time,
                                float *out_glow, uint8_t *out_band, char *out_glyph);

static inline float grid_half_diag(int w, int h)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    return sqrtf(cx * cx + cy * cy);
}

/* pattern_is_animated_tier — true iff `p` is Tier 6 (always animates
 * regardless of MOTION; its parameter IS time).  Dispatcher uses
 * this to decide whether to gate t to 0 for T1-T5 when MOTION off. */
static inline bool pattern_is_animated_tier(Pattern p)
{
    return p >= PATTERN_ROTATING_DIPOLE;
}

/* ---------- §7.M — MOTION: wandering probe + spiral composition ---- *
 *
 * Three tiny helpers split the MOTION feature into pure pieces:
 *
 *   motion_probe_position(t, w, h)         — where the invisible
 *                                            probe is at sim time t
 *   motion_spiral_dir   (cell, probe)      — direction of the
 *                                            stable-spiral field
 *                                            centred on the probe,
 *                                            evaluated at one cell
 *   pattern_emit_arrow  (cell, t, v, mag)  — canonical emit used by
 *                                            all 30 patterns; folds
 *                                            the spiral into the
 *                                            cell's vector when t > 0
 *
 * See §1 MOTION docblock for the derivation and the composition rule. */

/* Noise-driven smooth 2-D wander.  Two decorrelated 1-D noise
 * channels feed x and y, giving a smooth organic trajectory across
 * the central PROBE_EXTENT_FRAC × 2 fraction of the viewport.
 * Deterministic given t — same t always gives the same position. */
static inline void motion_probe_position(float t, int w, int h,
                                         float *px, float *py)
{
    /* y-channel uses a large constant offset so it samples a different
     * region of noise space → uncorrelated from x → 2-D wander. */
    float nx = noise_sample(t * PROBE_DRIFT_RATE,        0.0f);
    float ny = noise_sample(0.0f, t * PROBE_DRIFT_RATE + 13.7f);
    *px = 0.5f * (float)w + PROBE_EXTENT_FRAC * (float)w * (2.0f * nx - 1.0f);
    *py = 0.5f * (float)h + PROBE_EXTENT_FRAC * (float)h * (2.0f * ny - 1.0f);
}

/* Eric Treacy's calcVec — stable-spiral linear field centred on
 * (px, py), eigenvalues -1 ± i, evaluated at cell (x, y) and
 * returned as a UNIT vector so it composes with the pattern on
 * equal magnitude footing. */
static inline void motion_spiral_dir(int x, int y, float px, float py,
                                     float *spx, float *spy)
{
    float dx = (float)x - px;
    float dy = (float)y - py;
    *spx =  dy - dx;
    *spy = -dx - dy;
    float m = sqrtf((*spx) * (*spx) + (*spy) * (*spy));
    if (m > 1e-6f) { *spx /= m; *spy /= m; }
}

/* pattern_emit_arrow — canonical pattern-side emit.
 *
 *   t == 0 (MOTION off): pure static pattern direction (identical
 *                         to a direct cell_emit_arrow call).
 *   t  > 0 (MOTION on):  COMPOSE the pattern direction with the
 *                         spiral direction.  Both unit-normalised
 *                         first so neither dominates by raw scale:
 *
 *     out = unit(vx, vy) + MOTION_SPIRAL_WEIGHT · spiral_dir
 *
 * Composition (not replacement) is the key design choice — it keeps
 * every preset visually distinct under MOTION.  Cells where pattern
 * and spiral are anti-aligned cancel out and render as '.' (visible
 * "neutral zones" where the two contributions disagree).
 *
 * mag_norm is passed through unchanged — the pattern's own intensity
 * gradient still drives the colour band. */
static inline void pattern_emit_arrow(int x, int y, int w, int h, float t,
                                      float vx, float vy, float mag_norm,
                                      float *gl, uint8_t *bn, char *gy)
{
    if (t > 0.0f) {
        float px, py;   motion_probe_position(t, w, h, &px, &py);
        float spx, spy; motion_spiral_dir(x, y, px, py, &spx, &spy);

        float pm = sqrtf(vx * vx + vy * vy);
        if (pm > 1e-6f) { vx /= pm; vy /= pm; }

        vx += MOTION_SPIRAL_WEIGHT * spx;
        vy += MOTION_SPIRAL_WEIGHT * spy;
    }
    cell_emit_arrow(vx, vy, mag_norm, gl, bn, gy);
}

/* ---------- Tier 1 — GRADIENT: ∇f for chosen scalars ----------------- */

/* GRAD_PARABOLOID — ∇(x²+y²) = (2x, 2y).  Radial outward; magnitude
 * grows linearly with distance from the centre. */
static void pattern_grad_paraboloid(int x, int y, int w, int h, float t,
                                    float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float vx, vy;
    scalar_gradient(scalar_paraboloid, (float)x - cx, (float)y - cy, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, grid_half_diag(w, h) * SCALE_GRADIENT),
        gl, bn, gy);
}

/* GRAD_SADDLE — ∇(x²-y²) = (2x, -2y).  Pushes outward along ±x,
 * inward along ±y.  Classic saddle equilibrium at origin. */
static void pattern_grad_saddle(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float vx, vy;
    scalar_gradient(scalar_saddle, (float)x - cx, (float)y - cy, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, grid_half_diag(w, h) * SCALE_GRADIENT),
        gl, bn, gy);
}

/* GRAD_PERIODIC — ∇(sin(kx)·cos(ky)).  Periodic grid of maxima/minima
 * with vortex-like rotation patterns at the saddle points between. */
static void pattern_grad_periodic(int x, int y, int w, int h, float t,
                                  float *gl, uint8_t *bn, char *gy)
{
    float vx, vy;
    scalar_gradient(scalar_periodic, (float)x, (float)y, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_BOUNDED), gl, bn, gy);
}

/* GRAD_RIPPLE — ∇cos(k·r), where r = √(x²+y²).  Concentric "lake
 * ripples" of inward/outward arrows alternating each ring. */
static void pattern_grad_ripple(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float vx, vy;
    scalar_gradient(scalar_ripple, (float)x - cx, (float)y - cy, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_BOUNDED), gl, bn, gy);
}

/* GRAD_NOISE — ∇(value-noise).  Smooth-but-random gradient field; the
 * arrows trace the noise's topography (uphill = arrows point up the
 * noise slope, like a steepest-ascent map). */
static void pattern_grad_noise(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    float vx, vy;
    scalar_gradient(scalar_noise, (float)x, (float)y, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_NOISE_GRAD * NOISE_FREQ),
        gl, bn, gy);
}

/* ---------- Tier 2 — ANALYTIC: classical 2-D vector fields ----------- */

/* RADIAL_OUT — v = (dx, dy).  Pure radial source; everything outward. */
static void pattern_radial_out(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float vx = (float)x - cx, vy = (float)y - cy;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, grid_half_diag(w, h) * SCALE_RADIAL_HALFDIAG),
        gl, bn, gy);
}

/* RADIAL_IN — v = -(dx, dy).  Pure radial sink; everything inward. */
static void pattern_radial_in(int x, int y, int w, int h, float t,
                              float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float vx = -((float)x - cx), vy = -((float)y - cy);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, grid_half_diag(w, h) * SCALE_RADIAL_HALFDIAG),
        gl, bn, gy);
}

/* ROTATION — v = (-dy, dx).  Textbook math-CCW rotation; on a screen
 * with Y growing down it APPEARS clockwise (top moves right). */
static void pattern_rotation(int x, int y, int w, int h, float t,
                             float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float dx = (float)x - cx, dy = (float)y - cy;
    float vx = -dy, vy = dx;
    float mag = sqrtf(dx * dx + dy * dy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, grid_half_diag(w, h) * SCALE_RADIAL_HALFDIAG),
        gl, bn, gy);
}

/* SHEAR_X — v = (dy, 0).  Top row moves one way, bottom row the
 * other; magnitude grows linearly with vertical distance from centre. */
static void pattern_shear_x(int x, int y, int w, int h, float t,
                            float *gl, uint8_t *bn, char *gy)
{
    (void)x; (void)w;
    float cy = 0.5f * (float)h;
    float vx = (float)y - cy, vy = 0.0f;
    float mag = fabsf(vx);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, 0.5f * (float)h), gl, bn, gy);
}

/* UNIFORM_TILTED — v = (cos 30°, sin 30°).  Constant flow at 30° below
 * horizontal (mild SE drift).  Sanity-check pattern: every cell
 * should show the same glyph (until MOTION's probe disturbs locally). */
static void pattern_uniform_tilted(int x, int y, int w, int h, float t,
                                   float *gl, uint8_t *bn, char *gy)
{
    float vx = cosf((float)M_PI / 6.0f);
    float vy = sinf((float)M_PI / 6.0f);
    pattern_emit_arrow(x, y, w, h, t, vx, vy, 0.7f, gl, bn, gy);
}

/* ---------- Tier 3 — PHYSICS: real-world named fields --------------- */

/* POINT_CHARGE — Coulomb-like outward field from a single positive
 * charge at the screen centre.  v = (dx, dy) / r², softened by ε to
 * keep the centre cell finite. */
static void pattern_point_charge(int x, int y, int w, int h, float t,
                                 float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float dx = (float)x - cx, dy = (float)y - cy;
    float r2 = dx * dx + dy * dy + COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;
    float vx = dx / r2, vy = dy / r2;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_INV_SQ), gl, bn, gy);
}

/* DIPOLE — two charges at (cx - d, cy) [+1] and (cx + d, cy) [-1].
 * Superposed Coulomb fields → classic dipole pattern: arrows leave
 * the + pole, curve through space, and converge at the - pole. */
static void pattern_dipole(int x, int y, int w, int h, float t,
                           float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float d  = DIPOLE_HALF_SEP;
    float eps2 = COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;

    /* + charge at (cx - d, cy) */
    float ax = (float)x - (cx - d), ay = (float)y - cy;
    float ar2 = ax * ax + ay * ay + eps2;
    /* − charge at (cx + d, cy) → field points toward it (attractive
     * direction for a test + charge) */
    float bx = (float)x - (cx + d), by = (float)y - cy;
    float br2 = bx * bx + by * by + eps2;

    float vx = ax / ar2 - bx / br2;
    float vy = ay / ar2 - by / br2;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_INV_SQ), gl, bn, gy);
}

/* WIRE_MAGNETIC — infinite straight wire carrying current OUT of the
 * page along +z at screen centre.  B = (1/r) · θ̂ where θ̂ is the
 * right-hand-rule tangent.  In screen coords:  vx = -dy/r², vy = dx/r²
 * (factor of 1/r² because we divide unit tangent by r). */
static void pattern_wire_magnetic(int x, int y, int w, int h, float t,
                                  float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float dx = (float)x - cx, dy = (float)y - cy;
    float r2 = dx * dx + dy * dy + COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;
    float vx = -dy / r2, vy = dx / r2;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_INV_R), gl, bn, gy);
}

/* GRAVITY — point mass at screen centre.  Attractive inverse-square:
 * v = -(dx, dy) / r³ (unit-radial scaled by 1/r²). */
static void pattern_gravity(int x, int y, int w, int h, float t,
                            float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float dx = (float)x - cx, dy = (float)y - cy;
    float r2 = dx * dx + dy * dy + COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;
    float r  = sqrtf(r2);
    float vx = -dx / (r2 * r), vy = -dy / (r2 * r);
    float mag = 1.0f / r2;
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_INV_SQ), gl, bn, gy);
}

/* QUADRUPOLE — 4 alternating charges at (±d, ±d) with signs:
 *   (+d, +d): +     (+d, -d): -
 *   (-d, +d): -     (-d, -d): +
 * Net field shows the characteristic 4-fold pattern with crisp null
 * lines along the axes. */
static void pattern_quadrupole(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float d  = QUADRUPOLE_HALF;
    float eps2 = COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;
    float vx = 0.0f, vy = 0.0f;

    /* Charge layout: signs[i] · field_from((cx+ox[i], cy+oy[i])) */
    static const float ox[4] = { +1.0f, +1.0f, -1.0f, -1.0f };
    static const float oy[4] = { +1.0f, -1.0f, +1.0f, -1.0f };
    static const float sg[4] = { +1.0f, -1.0f, -1.0f, +1.0f };

    for (int i = 0; i < 4; i++) {
        float ax = (float)x - (cx + d * ox[i]);
        float ay = (float)y - (cy + d * oy[i]);
        float ar2 = ax * ax + ay * ay + eps2;
        vx += sg[i] * ax / ar2;
        vy += sg[i] * ay / ar2;
    }
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_INV_SQ), gl, bn, gy);
}

/* ---------- Tier 4 — SOLENOID: divergence-free flows ----------------- */

/* CURL_NOISE — v = (∂φ/∂y, -∂φ/∂x), φ = value-noise.  Divergence-free
 * by construction.  Reads as smooth, curling, "fluid-like" flow with
 * no obvious sources or sinks. */
static void pattern_curl_noise(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    float vx, vy;
    scalar_curl_2d(scalar_noise, (float)x, (float)y, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_NOISE_GRAD * NOISE_FREQ), gl, bn, gy);
}

/* Stream function ψ for STREAM_GRID — a 2-D grid of alternating
 * vortices (sin · sin), each cell of the grid spins one way, its
 * neighbours spin the other. */
static float scalar_stream_grid(float x, float y)
{
    return sinf(x * STREAM_FREQ_X) * sinf(y * STREAM_FREQ_Y);
}

/* STREAM_GRID — v = curl(ψ) for the grid stream function above.
 * Produces a tiled grid of counter-rotating vortices. */
static void pattern_stream_grid(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    float vx, vy;
    scalar_curl_2d(scalar_stream_grid, (float)x, (float)y, &vx, &vy);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_BOUNDED * 0.3f), gl, bn, gy);
}

/* VORTEX_PAIR — two opposite rotational centres: one CCW at (cx - d, cy),
 * one CW at (cx + d, cy).  Superposed magnetic-wire fields with
 * opposite currents.  Between them: strong directed flow; outside:
 * decay with distance. */
static void pattern_vortex_pair(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float d  = DIPOLE_HALF_SEP;
    float eps2 = COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;

    /* CCW vortex at (cx - d, cy) */
    float ax = (float)x - (cx - d), ay = (float)y - cy;
    float ar2 = ax * ax + ay * ay + eps2;
    /* CW vortex at (cx + d, cy) — sign flips on the tangent */
    float bx = (float)x - (cx + d), by = (float)y - cy;
    float br2 = bx * bx + by * by + eps2;

    float vx = (-ay / ar2) + ( by / br2);
    float vy = ( ax / ar2) + (-bx / br2);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_INV_R), gl, bn, gy);
}

/* CHANNEL_FLOW — Poiseuille parabolic profile in a channel along x.
 * v = (1 - (2y/h - 1)², 0).  Fastest in the middle, zero at the
 * walls.  Always-positive vx → pure rightward flow with varying speed. */
static void pattern_channel_flow(int x, int y, int w, int h, float t,
                                 float *gl, uint8_t *bn, char *gy)
{
    (void)w;
    float u = 2.0f * (float)y / (float)(h - 1) - 1.0f;     /* u ∈ [-1, +1] */
    float vx = 1.0f - u * u;                               /* parabolic */
    float vy = 0.0f;
    pattern_emit_arrow(x, y, w, h, t, vx, vy, vx * 0.9f, gl, bn, gy);
}

/* NOISY_UNIFORM — v = (1, 0) + small noise gradient.  Uniform
 * rightward flow with a sprinkle of turbulence; demonstrates how a
 * mean flow plus small perturbation reads to the eye. */
static void pattern_noisy_uniform(int x, int y, int w, int h, float t,
                                  float *gl, uint8_t *bn, char *gy)
{
    float nx, ny;
    scalar_gradient(scalar_noise, (float)x, (float)y, &nx, &ny);
    float vx = 1.0f + nx * 4.0f;
    float vy =        ny * 4.0f;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, 1.5f), gl, bn, gy);
}

/* ---------- Tier 5 — DYNAMICS: 2-D ODE phase portraits -------------- *
 *
 * Phase portraits map each screen cell to a point (x_p, y_p) in the
 * ODE's native phase space, then plot (dx/dt, dy/dt) as an arrow.
 *
 * The patterns below use SCREEN-y-down throughout.  Textbook
 * formulas usually assume y-up; we adopt the screen convention so
 * the arrows render literally — no sign flip in the emit call.
 * The visible "up direction" on screen corresponds to NEGATIVE y_p
 * in our parameterisation; this is documented per-pattern.
 */

/* Helper — map screen (x, y) to phase-space (x_p, y_p) ∈ [-h, +h]
 * using the supplied half-extent.  Screen y-down → y_p grows DOWN in
 * the phase-space too (textbook flips this; we don't, see note above). */
static inline void cell_to_phase(int x, int y, int w, int h, float half_extent,
                                 float *xp, float *yp)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    *xp = ((float)x - cx) * (half_extent / cx);
    *yp = ((float)y - cy) * (half_extent / cy);
}

/* STABLE_NODE — dx/dt = -x_p,  dy/dt = -y_p.
 * Every trajectory approaches the origin along straight lines.
 * Looks like RADIAL_IN but with linear-in-distance magnitude. */
static void pattern_stable_node(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    float xp, yp;
    cell_to_phase(x, y, w, h, PHASE_HALF_EXTENT, &xp, &yp);
    float vx = -xp, vy = -yp;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_PHASE), gl, bn, gy);
}

/* STABLE_SPIRAL — dx/dt = -x_p - y_p,  dy/dt =  x_p - y_p.
 * Linear system with complex eigenvalues (negative real part) →
 * spiral attractor: trajectories rotate inward to the origin. */
static void pattern_stable_spiral(int x, int y, int w, int h, float t,
                                  float *gl, uint8_t *bn, char *gy)
{
    float xp, yp;
    cell_to_phase(x, y, w, h, PHASE_HALF_EXTENT, &xp, &yp);
    float vx = -xp - yp;
    float vy =  xp - yp;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_PHASE), gl, bn, gy);
}

/* HOPF_CYCLE — Hopf normal form:
 *   dx/dt = (μ - r²) x - y         where r² = x² + y²
 *   dy/dt = x + (μ - r²) y
 *
 * For μ > 0 there is a STABLE LIMIT CYCLE at r = √μ.  Arrows inside
 * the cycle push outward; arrows outside push inward; arrows ON the
 * cycle rotate tangentially.  The classic Hopf bifurcation visualisation. */
static void pattern_hopf_cycle(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    float xp, yp;
    cell_to_phase(x, y, w, h, PHASE_HALF_EXTENT, &xp, &yp);
    float r2  = xp * xp + yp * yp;
    float mu_minus_r2 = HOPF_MU - r2;
    float vx = mu_minus_r2 * xp - yp;
    float vy = xp + mu_minus_r2 * yp;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_PHASE), gl, bn, gy);
}

/* VAN_DER_POL — relaxation oscillator (van der Pol 1926):
 *   dx/dt = y
 *   dy/dt = μ (1 - x²) y - x
 *
 * For μ > 0, all trajectories converge to a unique LIMIT CYCLE
 * (non-circular, "relaxation" shape).  Origin is unstable. */
static void pattern_van_der_pol(int x, int y, int w, int h, float t,
                                float *gl, uint8_t *bn, char *gy)
{
    float xp, yp;
    cell_to_phase(x, y, w, h, PHASE_HALF_EXTENT, &xp, &yp);
    float vx = yp;
    float vy = VDP_MU * (1.0f - xp * xp) * yp - xp;
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_PHASE * 2.0f), gl, bn, gy);
}

/* PENDULUM — undamped nonlinear pendulum:
 *   dx/dt = y          (x = angle, y = angular velocity)
 *   dy/dt = -sin(x)
 *
 * x ranges over [-π, π] (one full rotation).  Trajectories show:
 *   - Closed orbits near (0, 0): small swings
 *   - Open curves above / below: full rotations
 *   - SEPARATRIX: the curve passing through (±π, 0) — the boundary
 *     between oscillation and rotation. */
static void pattern_pendulum(int x, int y, int w, int h, float t,
                             float *gl, uint8_t *bn, char *gy)
{
    /* Phase x ∈ [-π, π]; phase y ∈ [-3, 3]. */
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float xp = ((float)x - cx) * (PENDULUM_X_HALF_EXTENT / cx);
    float yp = ((float)y - cy) * (PHASE_HALF_EXTENT / cy);
    float vx = yp;
    float vy = -sinf(xp);
    float mag = sqrtf(vx * vx + vy * vy);
    pattern_emit_arrow(x, y, w, h, t, vx, vy,
        mag_saturate(mag, SCALE_PHASE), gl, bn, gy);
}

/* ---------- Tier 6 — ANIMATED: time-varying vector fields ----------- */

/* ROTATING_DIPOLE — like DIPOLE, but the dipole's orientation rotates
 * around the screen centre at constant angular speed.  At t=0 the
 * dipole is horizontal; one period later it's back. */
static void pattern_rotating_dipole(int x, int y, int w, int h, float t,
                                    float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float omega = 2.0f * (float)M_PI / ROT_DIPOLE_PERIOD;
    float ang = omega * t;
    float dx_pole = DIPOLE_HALF_SEP * cosf(ang);
    float dy_pole = DIPOLE_HALF_SEP * sinf(ang);
    float eps2 = COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;

    /* + at (cx + dx_pole, cy + dy_pole); − at the opposite side */
    float ax = (float)x - (cx + dx_pole), ay = (float)y - (cy + dy_pole);
    float ar2 = ax * ax + ay * ay + eps2;
    float bx = (float)x - (cx - dx_pole), by = (float)y - (cy - dy_pole);
    float br2 = bx * bx + by * by + eps2;

    float vx = ax / ar2 - bx / br2;
    float vy = ay / ar2 - by / br2;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_INV_SQ), gl, bn, gy);
}

/* TRAVELLING_WAVE — v = (sin(kx - ωt), 0).  Plane wave moving in
 * the +x direction.  Watch the bright/dark bands ripple rightward. */
static void pattern_travelling_wave(int x, int y, int w, int h, float t,
                                    float *gl, uint8_t *bn, char *gy)
{
    (void)y; (void)w; (void)h;
    float k     = 0.25f;
    float omega = 2.0f * (float)M_PI / TRAVEL_WAVE_PERIOD;
    float vx    = sinf(k * (float)x - omega * t);
    float vy    = 0.0f;
    float mag   = fabsf(vx);
    cell_emit_arrow(vx, vy, mag * 0.95f, gl, bn, gy);
}

/* BREATHING_RADIAL — RADIAL_OUT with magnitude modulated by sin(ωt).
 * Half the cycle: outward source.  Other half: inward sink.  Pulses
 * between the two extremes through zero (where everything reads as
 * '.' field-zero markers). */
static void pattern_breathing_radial(int x, int y, int w, int h, float t,
                                     float *gl, uint8_t *bn, char *gy)
{
    float cx = 0.5f * (float)w, cy = 0.5f * (float)h;
    float dx = (float)x - cx, dy = (float)y - cy;
    float omega = 2.0f * (float)M_PI / BREATHE_PERIOD;
    float s = sinf(omega * t);
    float vx = dx * s, vy = dy * s;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, grid_half_diag(w, h) * SCALE_RADIAL_HALFDIAG),
                    gl, bn, gy);
}

/* ORBITING_VORTEX — a single rotational vortex whose centre traces
 * a circle around the screen centre.  Combines spatial structure
 * with temporal motion. */
static void pattern_orbiting_vortex(int x, int y, int w, int h, float t,
                                    float *gl, uint8_t *bn, char *gy)
{
    float cx     = 0.5f * (float)w, cy = 0.5f * (float)h;
    float half_d = grid_half_diag(w, h);
    float omega  = 2.0f * (float)M_PI / ORBIT_PERIOD;
    float r_orb  = half_d * ORBIT_RADIUS_FRAC;
    float vcx    = cx + r_orb * cosf(omega * t);
    float vcy    = cy + r_orb * sinf(omega * t);
    float eps2   = COULOMB_SOFT_EPS * COULOMB_SOFT_EPS;

    float dx = (float)x - vcx, dy = (float)y - vcy;
    float r2 = dx * dx + dy * dy + eps2;
    float vx = -dy / r2, vy = dx / r2;
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_INV_R), gl, bn, gy);
}

/* DRIFT_CURL — CURL_NOISE with the noise potential drifting in y over
 * time.  Same divergence-free structure as Tier 4 CURL_NOISE but the
 * field evolves continuously, like ASCII smoke. */
static void pattern_drift_curl(int x, int y, int w, int h, float t,
                               float *gl, uint8_t *bn, char *gy)
{
    (void)w; (void)h;
    /* Sample noise at (x, y + drift·t) — finite-diff via 4 reads. */
    float drift = t * 0.7f;
    float n_yp = noise_sample((float)x * NOISE_FREQ, ((float)y + 1.0f + drift) * NOISE_FREQ);
    float n_ym = noise_sample((float)x * NOISE_FREQ, ((float)y - 1.0f + drift) * NOISE_FREQ);
    float n_xp = noise_sample(((float)x + 1.0f) * NOISE_FREQ, ((float)y + drift) * NOISE_FREQ);
    float n_xm = noise_sample(((float)x - 1.0f) * NOISE_FREQ, ((float)y + drift) * NOISE_FREQ);
    /* v = (∂φ/∂y, -∂φ/∂x) */
    float vx =  0.5f * (n_yp - n_ym);
    float vy = -0.5f * (n_xp - n_xm);
    float mag = sqrtf(vx * vx + vy * vy);
    cell_emit_arrow(vx, vy, mag_saturate(mag, SCALE_NOISE_GRAD * NOISE_FREQ),
                    gl, bn, gy);
}

/* ---------- Dispatch table ------------------------------------------- */

/*
 * VectorPattern — one row of the §7 pattern dispatch table.
 *
 * INTENT
 *   Replace a per-cell switch(Pattern) with a single function-pointer
 *   indirect call.  arrow_grid_evaluate's hot loop becomes
 *   `sample(x, y, …)` — no branch on pattern type, one cache-friendly
 *   table.  Adding a new pattern is a three-line edit: write the fn,
 *   append a row here, add its enum.  The fixed-size [N_PATTERNS]
 *   initialiser keyed by enum makes the compiler complain if the
 *   table is incomplete or mis-ordered.
 *
 * CONTEXT
 *   vector_patterns[N_PATTERNS] in §7 is the singular static table.
 *   Read by arrow_grid_evaluate (resolves .sample) and by
 *   pattern_name / pattern_tier (HUD readouts).  Never mutated at
 *   runtime — defined `const` so it lives in .rodata.
 *
 * MEMBER LOGIC
 *   name   : HUD label, RIGHT-padded to exactly 10 chars so the
 *            "%-10s" format produces aligned columns when the user
 *            cycles patterns via n/p.
 *   tier   : "N-XXXX " 7-char label encoding tier index (1..6) plus
 *            a 4-char mnemonic (GRAD/ANAL/PHYS/SOLN/DYNS/ANIM).
 *            Displayed next to the pattern name in the HUD.
 *   sample : Per-cell function pointer matching VectorPatternFn.
 *            Called once per cell per sim tick (~11K cells × 60 Hz
 *            = ~660K calls/sec).  Writes the three output slots
 *            (glow, band, glyph) in the ArrowGrid via
 *            pattern_emit_arrow (which folds in the MOTION layer
 *            when t > 0).
 *
 * REFERENCES
 *   • Designated initialisers (C99 §6.7.9) — `[INDEX] = { … }` lets
 *     the table be defined out of declaration order while staying
 *     enum-keyed.
 *   • Function-pointer dispatch — Bryant & O'Hallaron, "Computer
 *     Systems: A Programmer's Perspective" §3.10 for the compiled-
 *     code shape of an indirect call vs a switch.
 */
typedef struct {
    const char      *name;        /* 10-char padded for HUD alignment */
    const char      *tier;        /* 7-char "N-LABEL " padded         */
    VectorPatternFn  sample;
} VectorPattern;

static const VectorPattern vector_patterns[N_PATTERNS] = {
    /* Tier 1 — GRADIENT */
    [PATTERN_GRAD_PARABOLOID] = { "GRAD-PARAB", "1-GRAD ", pattern_grad_paraboloid },
    [PATTERN_GRAD_SADDLE]     = { "GRAD-SADDL", "1-GRAD ", pattern_grad_saddle     },
    [PATTERN_GRAD_PERIODIC]   = { "GRAD-PERIO", "1-GRAD ", pattern_grad_periodic   },
    [PATTERN_GRAD_RIPPLE]     = { "GRAD-RIPPL", "1-GRAD ", pattern_grad_ripple     },
    [PATTERN_GRAD_NOISE]      = { "GRAD-NOISE", "1-GRAD ", pattern_grad_noise      },
    /* Tier 2 — ANALYTIC */
    [PATTERN_RADIAL_OUT]      = { "RADIAL-OUT", "2-ANLY ", pattern_radial_out      },
    [PATTERN_RADIAL_IN]       = { "RADIAL-IN ", "2-ANLY ", pattern_radial_in       },
    [PATTERN_ROTATION]        = { "ROTATION  ", "2-ANLY ", pattern_rotation        },
    [PATTERN_SHEAR_X]         = { "SHEAR-X   ", "2-ANLY ", pattern_shear_x         },
    [PATTERN_UNIFORM_TILTED]  = { "UNIF-TILT ", "2-ANLY ", pattern_uniform_tilted  },
    /* Tier 3 — PHYSICS */
    [PATTERN_POINT_CHARGE]    = { "POINT-CHG ", "3-PHYS ", pattern_point_charge    },
    [PATTERN_DIPOLE]          = { "DIPOLE    ", "3-PHYS ", pattern_dipole          },
    [PATTERN_WIRE_MAGNETIC]   = { "WIRE-MAG  ", "3-PHYS ", pattern_wire_magnetic   },
    [PATTERN_GRAVITY]         = { "GRAVITY   ", "3-PHYS ", pattern_gravity         },
    [PATTERN_QUADRUPOLE]      = { "QUADRUPOLE", "3-PHYS ", pattern_quadrupole      },
    /* Tier 4 — SOLENOID */
    [PATTERN_CURL_NOISE]      = { "CURL-NOISE", "4-SOLN ", pattern_curl_noise      },
    [PATTERN_STREAM_GRID]     = { "STREAM-GRD", "4-SOLN ", pattern_stream_grid     },
    [PATTERN_VORTEX_PAIR]     = { "VORTX-PAIR", "4-SOLN ", pattern_vortex_pair     },
    [PATTERN_CHANNEL_FLOW]    = { "CHAN-FLOW ", "4-SOLN ", pattern_channel_flow    },
    [PATTERN_NOISY_UNIFORM]   = { "NOISY-UNI ", "4-SOLN ", pattern_noisy_uniform   },
    /* Tier 5 — DYNAMICS */
    [PATTERN_STABLE_NODE]     = { "STBL-NODE ", "5-DYNS ", pattern_stable_node     },
    [PATTERN_STABLE_SPIRAL]   = { "STBL-SPRL ", "5-DYNS ", pattern_stable_spiral   },
    [PATTERN_HOPF_CYCLE]      = { "HOPF-CYCLE", "5-DYNS ", pattern_hopf_cycle      },
    [PATTERN_VAN_DER_POL]     = { "VAN-DR-POL", "5-DYNS ", pattern_van_der_pol     },
    [PATTERN_PENDULUM]        = { "PENDULUM  ", "5-DYNS ", pattern_pendulum        },
    /* Tier 6 — ANIMATED */
    [PATTERN_ROTATING_DIPOLE] = { "ROT-DIPOLE", "6-ANIM ", pattern_rotating_dipole },
    [PATTERN_TRAVELLING_WAVE] = { "TRAV-WAVE ", "6-ANIM ", pattern_travelling_wave },
    [PATTERN_BREATHING_RADIAL]= { "BREATHE   ", "6-ANIM ", pattern_breathing_radial},
    [PATTERN_ORBITING_VORTEX] = { "ORBT-VORTX", "6-ANIM ", pattern_orbiting_vortex },
    [PATTERN_DRIFT_CURL]      = { "DRIFT-CURL", "6-ANIM ", pattern_drift_curl      },
};

static const char *pattern_name(Pattern p)
{
    if ((unsigned)p >= (unsigned)N_PATTERNS) return "?         ";
    return vector_patterns[p].name;
}

static const char *pattern_tier(Pattern p)
{
    if ((unsigned)p >= (unsigned)N_PATTERNS) return "?      ";
    return vector_patterns[p].tier;
}

/* ===================================================================== */
/* §8  scene — ArrowGrid + PatternState + PaletteState + Scene            */
/* ===================================================================== */

/*
 * ArrowGrid — per-cell render-output buffer (the §7 → §9 hand-off).
 *
 * INTENT
 *   Decouple "compute what to draw" (pattern fn writes a vector per
 *   cell, then pattern_emit_arrow converts it to glyph + band) from
 *   "actually draw it" (renderer in §9 reads glyph + band and paints).
 *   Conceptually the same role as a tile/cell framebuffer in a
 *   software renderer — only character-cell granularity, not per-pixel.
 *
 * CONTEXT
 *   One instance lives on Scene (§8).  Written by
 *   arrow_grid_evaluate() once per sim tick (one pattern-fn call per
 *   cell).  Read by arrow_grid_paint() once per render frame.  Index
 *   with arrow_grid_idx(grid, x, y) = y · w + x — row-major.
 *
 * MEMORY
 *   glow[] + band[] + glyph[] = 4 + 1 + 1 = 6 bytes/cell × CELLS_MAX
 *   ≈ 66 KB in BSS.  No allocation.  Cache-cold only at the start
 *   of each evaluate pass; the (sample, write) cycle stays inside
 *   the inner loop.
 *
 * MEMBER LOGIC
 *   w, h    : Grid dimensions in cells.  Set by arrow_grid_reset()
 *             at scene_reset / SIGWINCH.
 *   count   : w · h, cached for the zero-init loop.
 *   glow[]  : Per-cell visibility flag: > 0.0 = paint, ≤ 0.0 = skip.
 *             Stored as float (not bool) so patterns COULD scale it
 *             for intensity if a future feature needs it; the
 *             renderer currently just thresholds against 0.
 *   band[]  : Palette band ∈ {0..3} → PAIR_BAND_BASE + band.  Bottom
 *             2 bits used (`band & 3` clamps); patterns are expected
 *             to write a clean value, the mask is defensive.
 *   glyph[] : ASCII char the renderer emits.  0 also = skip
 *             (defensive double-gate, in case a pattern set glow > 0
 *             but forgot to pick a glyph — which would be a bug).
 *
 * REFERENCES
 *   • Deferred-shading pattern — separate "decide what to draw" from
 *     "draw it" via an intermediate buffer.  Akenine-Möller et al.,
 *     "Real-Time Rendering" (4th ed.) §20.1.
 *   • Struct-of-Arrays (three parallel arrays) instead of
 *     Array-of-Structs (one packed record per cell) — chosen because
 *     the renderer iterates the three fields in lock-step and the
 *     SoA layout avoids padding waste on the 1-byte fields.
 */
typedef struct {
    int      w, h;
    int      count;
    float    glow [CELLS_MAX];
    uint8_t  band [CELLS_MAX];
    char     glyph[CELLS_MAX];
} ArrowGrid;

static inline int arrow_grid_idx(const ArrowGrid *g, int x, int y)
{
    return y * g->w + x;
}

static void arrow_grid_reset(ArrowGrid *g, int w, int h)
{
    g->w     = w;
    g->h     = h;
    g->count = w * h;
    for (int i = 0; i < g->count; i++) {
        g->glow [i] = 0.0f;
        g->band [i] = 0;
        g->glyph[i] = 0;
    }
}

/* arrow_grid_evaluate — sweep every cell, calling the active pattern
 * fn with the supplied animation time t.  Pure grid-walk dispatch —
 * the t-gate decision (motion+tier → t=0 or t=field_time) is made by
 * scene_evaluate above this layer. */
static void arrow_grid_evaluate(ArrowGrid *g, Pattern p, float t)
{
    if ((unsigned)p >= (unsigned)N_PATTERNS) return;
    VectorPatternFn sample = vector_patterns[p].sample;
    for (int y = 0; y < g->h; y++) {
        for (int x = 0; x < g->w; x++) {
            int idx = arrow_grid_idx(g, x, y);
            sample(x, y, g->w, g->h, t,
                   &g->glow [idx],
                   &g->band [idx],
                   &g->glyph[idx]);
        }
    }
}

/*
 * PatternState — active pattern + animation clock + drift speed.
 *
 * INTENT
 *   Group the three values that together describe "what's animating
 *   right now": which pattern is active, how far along the animation
 *   clock has advanced, and at what speed multiplier.  They change
 *   together, they are read together, and they all feed the pattern
 *   dispatcher — one struct centralises the dataflow.
 *
 * CONTEXT
 *   Lives on Scene (§8).  Read by:
 *     • scene_evaluate() — passes t (gated by motion + tier) to
 *                          arrow_grid_evaluate.
 *     • scene_tick()     — advances field_time by drift_mult · dt
 *                          each tick (via pattern_state_advance_clock).
 *     • screen_draw()    — HUD readouts (pattern name, t).
 *   Mutated by app_handle_key():
 *     • n/p   → current      (modular wraparound through N_PATTERNS)
 *     • +/-   → drift_mult   (×2 / ÷2, clamped to [MIN, MAX])
 *
 * MEMBER LOGIC
 *   current     : Active pattern enum, indexes vector_patterns[].
 *                 Defaults to PATTERN_GRAD_PARABOLOID.
 *   field_time  : Drift accumulator (seconds).  Drives Tier 6
 *                 animation directly (their parameter IS time) and
 *                 the MOTION probe's noise sampling when motion is on.
 *                 Monotonically increasing within a run; reset by r.
 *   drift_mult  : Power-of-2 speed multiplier ∈ [DRIFT_MULT_MIN,
 *                 DRIFT_MULT_MAX] that scales field_time advancement.
 *                 Same convention as sibling showcases.
 */
typedef struct {
    Pattern  current;
    float    field_time;
    int      drift_mult;
} PatternState;

static void pattern_state_init(PatternState *ps)
{
    ps->current     = PATTERN_GRAD_PARABOLOID;
    ps->field_time  = 0.0f;
    ps->drift_mult  = DRIFT_MULT_DEF;
}

/* Advance the animation clock by one frame at the current drift_mult. */
static void pattern_state_advance_clock(PatternState *ps, float dt)
{
    ps->field_time += FIELD_DRIFT * (float)ps->drift_mult * dt;
}

/* Cycle through the pattern enum modulo N_PATTERNS. */
static void pattern_state_cycle_next(PatternState *ps)
{
    ps->current = (Pattern)(((int)ps->current + 1) % N_PATTERNS);
}
static void pattern_state_cycle_prev(PatternState *ps)
{
    ps->current = (Pattern)(((int)ps->current + N_PATTERNS - 1) % N_PATTERNS);
}

/* Power-of-2 step on drift_mult, clamped to [DRIFT_MULT_MIN, MAX]. */
static void pattern_state_drift_faster(PatternState *ps)
{
    if (ps->drift_mult < DRIFT_MULT_MAX) ps->drift_mult *= 2;
    if (ps->drift_mult > DRIFT_MULT_MAX) ps->drift_mult  = DRIFT_MULT_MAX;
}
static void pattern_state_drift_slower(PatternState *ps)
{
    ps->drift_mult /= 2;
    if (ps->drift_mult < DRIFT_MULT_MIN) ps->drift_mult = DRIFT_MULT_MIN;
}

/*
 * PaletteState — index of the currently-active Theme.
 *
 * INTENT
 *   A one-int wrapper is overkill data-wise but valuable structurally:
 *   gives "which colour scheme is showing" a stable home on Scene,
 *   matches the sub-struct convention used by sibling showcases, and
 *   makes the t/T key handler read as `s->palette.current = …`
 *   instead of `s->theme_idx = …`.
 *
 * CONTEXT
 *   Lives on Scene (§8).  Mutated by app_handle_key() on t/T (with
 *   modular wraparound through N_THEMES) and by palette_state_init()
 *   at reset.  Whenever it changes, theme_apply() (§3) MUST be
 *   called to push the new Theme's colour indices into the ncurses
 *   colour pairs — this struct holds the index, ncurses holds the
 *   actual pair bindings.  Read by screen_draw() for the HUD label.
 *
 * MEMBER LOGIC
 *   current : Index into themes[] ∈ [0, N_THEMES).  Anything outside
 *             that range is invalid; theme_apply() clamps to 0
 *             defensively.
 */
typedef struct {
    int current;
} PaletteState;

static void palette_state_init(PaletteState *p) { p->current = 0; }

static void palette_state_cycle_next(PaletteState *p)
{
    p->current = (p->current + 1) % N_THEMES;
}
static void palette_state_cycle_prev(PaletteState *p)
{
    p->current = (p->current + N_THEMES - 1) % N_THEMES;
}

/*
 * Scene — composite owner of ALL mutable simulation state.
 *
 * INTENT
 *   Single root that the main loop (§10), the simulation step
 *   (scene_tick), and the renderer (§9) all share.  Composes the
 *   sub-domains as named members so their roles are explicit at the
 *   type level instead of buried in a flat field list.  Each
 *   sub-struct owns one concern; Scene owns the composition + the
 *   two UI gates (paused, motion).
 *
 * THE PIPELINE (top to bottom = dataflow order)
 *
 *   noise    — value-noise lattice seed                 (the SOURCE)
 *               ↓ (read by GRAD_NOISE / CURL_NOISE / MOTION probe)
 *   pattern  — active pattern + field_time + drift_mult (the CONTROL)
 *               ↓ (drives arrow_grid_evaluate)
 *   grid     — per-cell (glow, band, glyph) buffer      (the OUTPUT)
 *               ↓ (read by arrow_grid_paint in §9)
 *   palette  — active theme index                       (the COLOUR)
 *               + paused / motion flags                 (the GATES)
 *
 * CONTEXT
 *   One instance, owned by App in §10.  Touched only from the main
 *   thread — the signal handlers in §10 do NOT reach into Scene;
 *   they flip volatile flags on App and let the main loop respond
 *   at the next frame boundary.
 *
 * LIFETIME
 *   scene_init   : zero-init, sub-struct defaults, call scene_reset.
 *   scene_reset  : re-roll NoiseField, zero ArrowGrid + clocks,
 *                  paint the initial frame.  Called on r and on
 *                  SIGWINCH-driven resize.
 *   scene_tick   : advance pattern clock, re-evaluate the grid.
 *   No teardown — Scene lives in BSS via App; OS reclaims at exit.
 *
 * MEMBER LOGIC
 *   noise   : NoiseField   — value-noise lattice seed.
 *   grid    : ArrowGrid    — per-cell render-output buffer.
 *   pattern : PatternState — active pattern + clocks + drift.
 *   palette : PaletteState — active theme index.
 *   paused  : When true, scene_tick early-returns, freezing both
 *             the field-drift clock and the MOTION probe.  Renderer
 *             keeps drawing the last computed grid → still frame.
 *             Toggled by space.
 *   motion  : When true, scene_evaluate passes t = field_time
 *             (instead of 0) to T1-T5 patterns, enabling
 *             pattern_emit_arrow's spiral composition.  Toggled by m.
 */
typedef struct {
    NoiseField   noise;
    ArrowGrid    grid;
    PatternState pattern;
    PaletteState palette;
    bool         paused;     /* space — freeze sim ticks   */
    bool         motion;     /* m     — enable probe overlay on T1-T5 */
} Scene;

/* scene_evaluate — decide t-gate (motion + tier), then dispatch the
 * active pattern across the grid.  T6 patterns always receive
 * t = field_time (their own animation drives them); T1-T5 patterns
 * receive t = 0 unless MOTION is on (the probe needs t > 0 to fire). */
static void scene_evaluate(Scene *s)
{
    Pattern p = s->pattern.current;
    float   t = (s->motion || pattern_is_animated_tier(p))
              ? s->pattern.field_time : 0.0f;
    arrow_grid_evaluate(&s->grid, p, t);
}

static void scene_reset(Scene *s, int mw, int mh)
{
    noise_field_reseed(&s->noise);
    arrow_grid_reset  (&s->grid, mw, mh);
    s->pattern.field_time = 0.0f;
    scene_evaluate(s);                          /* paint initial frame */
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    pattern_state_init (&s->pattern);
    palette_state_init (&s->palette);
    s->paused = false;
    s->motion = false;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    pattern_state_advance_clock(&s->pattern, dt);
    scene_evaluate(s);
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

/*
 * Screen — ncurses viewport dimensions cache.
 *
 * INTENT
 *   ncurses owns the terminal state.  We just need to know "how
 *   wide / tall is the drawable area right now" so arrow_grid_paint
 *   can centre the grid and the HUD helpers can right-align text
 *   without re-querying ncurses on every paint.  Updated lazily on
 *   resize.
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

/* ---------- arrow_grid_paint helpers (field painter) ----------------- */

/* viewport_centre_grid_origin — top-left screen cell where a grid of
 * grid_w × grid_h sits inside cols × rows.  HUD-aware: leaves
 * HUD_TOP_ROWS reserved above and HUD_BOTTOM_ROWS below.  Clamps to
 * the HUD edge if the grid is larger than the drawable interior. */
static void viewport_centre_grid_origin(int cols, int rows,
                                        int grid_w, int grid_h,
                                        int *gx0, int *gy0)
{
    int interior_h = rows - HUD_BAND_RESERVED_ROWS;
    *gx0 = (cols       - grid_w) / 2;
    *gy0 = (interior_h - grid_h) / 2 + HUD_TOP_ROWS;
    if (*gx0 < 0)            *gx0 = 0;
    if (*gy0 < HUD_TOP_ROWS) *gy0 = HUD_TOP_ROWS;
}

/* arrow_cell_paint — emit one cell from the ArrowGrid at screen
 * (sx, sy), gated by glow > 0 and a non-empty glyph. */
static void arrow_cell_paint(const ArrowGrid *g, int gx, int gy, int sx, int sy)
{
    int  idx = arrow_grid_idx(g, gx, gy);
    if (g->glow[idx] <= 0.0f) return;
    char glyph = g->glyph[idx];
    if (glyph == 0 || glyph == ' ') return;

    int pair = PAIR_BAND_BASE + (g->band[idx] & 3);
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* arrow_grid_paint — paint the ArrowGrid's (glow, band, glyph) per-cell
 * outputs to the terminal.  The patterns decided what each cell looks
 * like; this renderer just looks it up and paints.
 *
 *   STEP 1 — CENTRE the grid inside the viewport (HUD-aware)
 *   STEP 2 — PAINT every in-bounds cell the patterns marked visible
 */
static void arrow_grid_paint(const ArrowGrid *g, int cols, int rows)
{
    int gx0, gy0;
    viewport_centre_grid_origin(cols, rows, g->w, g->h, &gx0, &gy0);

    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            arrow_cell_paint(g, x, y, sx, sy);
        }
    }
}

/* ---------- screen_draw helpers (HUD layout) ------------------------- */

/* hud_draw_top_left_title — fixed "VECTOR FIELD ARROWS" chip on row 0. */
static void hud_draw_top_left_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " VECTOR FIELD ARROWS ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_draw_top_right_status — fps · sim Hz · state · pattern N/M ·
 * drift · MOTION flag, right-aligned on row 0. */
static void hud_draw_top_right_status(int cols, double fps, int sim_fps,
                                      const char *state_str,
                                      int current_idx_zero_based,
                                      int drift_mult, bool motion)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  drift:x%-2d  motion:%s ",
             fps, sim_fps, state_str,
             current_idx_zero_based + 1, N_PATTERNS,
             drift_mult, motion ? "ON " : "off");
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_field_bold_label — draw one bold HUD-pair field on row 1
 * starting at column `x`, advance by `width`, return the new x. */
static int hud_field_bold_label(int x, const char *fmt,
                                const char *val, int width)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, fmt, val);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + width;
}

/* hud_field_palette_swatch — "palette:####" segment on row 1: a label
 * in plain HUD pair, then HUD_N_PALETTE_BANDS '#' chars each in their
 * own band colour.  Returns the new x. */
static int hud_field_palette_swatch(int x)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += HUD_PALETTE_LABEL_W;
    for (int i = 0; i < HUD_N_PALETTE_BANDS; i++) {
        int p = PAIR_BAND_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, '#');
        attroff(COLOR_PAIR(p) | A_BOLD);
        x += 1;
    }
    return x;
}

/* hud_field_meta — trailing "t · map dims" on row 1. */
static void hud_field_meta(int x, float field_time, int grid_w, int grid_h)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  t:%.1fs  map:%dx%d ",
             (double)field_time, grid_w, grid_h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* hud_draw_param_row — row 1 dashboard: pattern · tier · theme ·
 * palette swatch · meta.  Each helper returns the next x so the row
 * reads as a left-to-right pipeline. */
static void hud_draw_param_row(const Scene *s)
{
    const PatternState *ps = &s->pattern;
    int x = HUD_LEFT_MARGIN;
    x = hud_field_bold_label(x, " pattern:%-10s ", pattern_name(ps->current),
                             HUD_PATTERN_FIELD_W);
    x = hud_field_bold_label(x, " tier:%-7s ",     pattern_tier(ps->current),
                             HUD_TIER_FIELD_W);
    x = hud_field_bold_label(x, " theme:%-8s ",    themes[s->palette.current].name,
                             HUD_THEME_FIELD_W);
    x = hud_field_palette_swatch(x);
    hud_field_meta(x, ps->field_time, s->grid.w, s->grid.h);
}

/* hud_draw_bottom_hint — single-row key-binding bar at the bottom. */
static void hud_draw_bottom_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:pattern  t/T:theme  m:motion  +/-:drift  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* screen_draw — full frame composer.
 *
 *   STEP 1 — CLEAR the back buffer
 *   STEP 2 — PAINT the arrow field
 *   STEP 3 — OVERLAY the top HUD row (title + right-aligned status)
 *   STEP 4 — OVERLAY the row-1 parameter dashboard
 *   STEP 5 — OVERLAY the bottom key-hint bar
 */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    arrow_grid_paint(&s->grid, sc->cols, sc->rows);

    const PatternState *ps        = &s->pattern;
    const char         *state_str = s->paused ? "PAUSED    "
                                              : pattern_name(ps->current);
    hud_draw_top_left_title();
    hud_draw_top_right_status(sc->cols, fps, sim_fps, state_str,
                              (int)ps->current, ps->drift_mult, s->motion);
    hud_draw_param_row(s);
    hud_draw_bottom_hint(sc->rows);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §10 app                                                                */
/* ===================================================================== */

/*
 * App — top-level program state; the BSS root.
 *
 * INTENT
 *   Single owner for the simulation, the screen, the sim-rate
 *   target, the active map size, and the two volatile signal flags.
 *   Lives in BSS as g_app (file-scope) for one specific reason:
 *   POSIX signal handlers can ONLY safely touch global async-signal-
 *   safe state, so the two sig_atomic_t flags must be reachable
 *   without arguments.  Everything else on App is touched from the
 *   main thread only.
 *
 * CONTEXT
 *   Exactly one instance: g_app at file scope.  Built in main(),
 *   driven by the main game loop, torn down by atexit(cleanup).
 *   on_exit_signal / on_resize_signal flip the flags; the main loop
 *   polls them at frame boundaries and reacts (clean exit / lazy
 *   resize) — keeping all the heavy work off the signal handler.
 *
 * MEMBER LOGIC
 *   scene       : All mutable simulation state (§8).  See Scene doc
 *                 above for the dataflow pipeline.
 *   screen      : Viewport dimensions cache (§9).
 *   sim_fps     : Target simulation Hz ∈ [SIM_FPS_MIN, SIM_FPS_MAX].
 *                 Drives the fixed-timestep accumulator in main();
 *                 stepped by ]/[ keys.  Independent of render FPS.
 *   map_w, map_h: Grid dims chosen by app_pick_map_size() from the
 *                 current Screen.  Re-derived on resize; clamped to
 *                 [MAP_W_MIN, MAP_W_MAX] × [MAP_H_MIN, MAP_H_MAX].
 *   running     : 0 = exit the main loop next iteration.  Cleared by
 *                 the SIGINT/SIGTERM handlers and by q/ESC in
 *                 app_handle_key().  sig_atomic_t for handler safety.
 *   need_resize : Set by the SIGWINCH handler; main loop polls it
 *                 before the next frame and calls app_do_resize().
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
    int                   map_w, map_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* app_pick_map_size — derive the arrow grid dims from the current
 * viewport, leaving HUD_BAND_RESERVED_ROWS for the dashboard, then
 * clamp to [MAP_*_MIN, MAP_*_MAX] (the BSS cap on samples[]). */
static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS;
    if (mw < MAP_W_MIN) mw = MAP_W_MIN;
    if (mh < MAP_H_MIN) mh = MAP_H_MIN;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

/* app_do_resize — full re-init after SIGWINCH: re-read viewport
 * dims, re-derive map size, rebuild the scene at the new size. */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* app_sim_rate_faster/slower — step sim_fps by SIM_FPS_STEP, clamped
 * to [SIM_FPS_MIN, SIM_FPS_MAX].  Bound to ]/[. */
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
 * named mutator on the appropriate Scene sub-state, so the switch
 * reads as a pseudocode binding table.  Returns false on exit
 * request (q/Q/ESC); true otherwise. */
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
        scene_reset(s, app->map_w, app->map_h);
        break;
    case 'm': case 'M':
        s->motion = !s->motion;     /* probe overlay on T1-T5 */
        break;

    case '=': case '+':  pattern_state_drift_faster(&s->pattern); break;
    case '-':            pattern_state_drift_slower(&s->pattern); break;
    case ']':            app_sim_rate_faster       ( app);        break;
    case '[':            app_sim_rate_slower       ( app);        break;
    case 'n': case 'N':  pattern_state_cycle_next  (&s->pattern); break;
    case 'p': case 'P':  pattern_state_cycle_prev  (&s->pattern); break;

    case 't':
        palette_state_cycle_next(&s->palette);
        theme_apply(s->palette.current);
        break;
    case 'T':
        palette_state_cycle_prev(&s->palette);
        theme_apply(s->palette.current);
        break;

    default: break;
    }
    return true;
}

/* ---------- main-loop helpers ---------------------------------------- */

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

/* app_bootstrap — ncurses + viewport + scene initial state. */
static void app_bootstrap(App *app)
{
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);
}

/* app_handle_pending_resize — if SIGWINCH flagged a resize, do it
 * and reset the frame clocks so dt doesn't include the resize cost
 * (which would falsely trigger the spiral-of-death cap below). */
static void app_handle_pending_resize(App *app,
                                      int64_t *frame_time,
                                      int64_t *sim_accum)
{
    if (!app->need_resize) return;
    app_do_resize(app);
    *frame_time = clock_ns();
    *sim_accum  = 0;
}

/* app_compute_frame_dt — clock delta since last frame, clamped at
 * SIM_MAX_FRAME_DT_MS so a slow terminal can't trigger unbounded
 * sim-tick catch-up below (spiral-of-death guard). */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    int64_t dt_cap = (int64_t)SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    if (dt > dt_cap) dt = dt_cap;
    return dt;
}

/* app_drain_fixed_timestep — Fiedler accumulator: integrate as many
 * fixed-rate sim ticks as fit into the elapsed dt, so the sim runs
 * at exactly sim_fps Hz regardless of render FPS jitter. */
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

/* app_update_fps_meter — sliding-window average over the last
 * FPS_UPDATE_MS of wall-clock time. */
static void app_update_fps_meter(int64_t dt,
                                 int *frame_count,
                                 int64_t *fps_accum,
                                 double *fps_display)
{
    (*frame_count)++;
    *fps_accum += dt;
    if (*fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
        *fps_display = (double)(*frame_count)
                     / ((double)(*fps_accum) / (double)NS_PER_SEC);
        *frame_count = 0;
        *fps_accum   = 0;
    }
}

/* app_throttle_to_render_target — sleep for the remainder of the
 * RENDER_FRAME_BUDGET so the loop doesn't burn CPU between frames. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* app_present_frame — render the world + flush ncurses back-buffer. */
static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* app_poll_keyboard — non-blocking getch + dispatch.  Returns false
 * iff the user requested exit (q / Q / ESC). */
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
