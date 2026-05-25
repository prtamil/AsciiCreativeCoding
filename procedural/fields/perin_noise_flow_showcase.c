/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * perin_noise_flow_showcase.c
 *   — 30 ways to visualise a 2-D Perlin-noise field.
 *
 * DEMO: A single Perlin-noise generator drives THIRTY distinct
 *       visualisations grouped into five tiers of complexity. Four of
 *       the patterns run particles over a derived vector field (FLOW,
 *       VORTEX, CURL, JITTER); the other 26 rasterise a scalar field
 *       per frame. The field slowly drifts via a time offset so every
 *       pattern evolves rather than looping:
 *
 *         Tier 1 — direct vis      : FLOW HEIGHT VALLEYS BANDED BINARY
 *         Tier 2 — shape transforms: RIDGE BILLOW ZEBRA RINGS PLASMA
 *         Tier 3 — domain warps    : CONTOUR WARP WARP_DEEP WARPRIDGE SLOPE
 *         Tier 4 — fractal/octave  : FBM FBM_HIGH RIDGED TURBLENC FBM_INV
 *         Tier 5 — texture/exotic  : MARBLE WOOD FIRE CLOUDS CAVES STARS
 *                                    VORTEX CURL JITTER CHAOS
 *
 *       Press 'r' to re-shuffle the Perlin permutation table for a
 *       wholly new field; n/p cycles patterns. The state bar shows
 *       [N/30] for the active preset.
 *
 * Study alongside: ../generational/voronoi_region_map.c — the other
 *       "field over the whole grid" showcase. Voronoi answers a
 *       discrete classifier question (nearest seed); Perlin noise is
 *       a continuous scalar field that you USE to drive other things
 *       (terrain, particles, textures). This file uses it as a flow
 *       field; the same noise + a height ramp would give terrain.
 *
 * Section map:
 *   §1 config   — grid, patterns, palette, themes, named constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes (N_BANDS trail colours each)
 *   §5 noise    — NoiseField + Perlin 2-D + multifractal kernels +
 *                 curl noise + 26 static-field pattern_*_at evaluators
 *   §6 scene    — Scene composing Grid, RenderBuffers, Particles,
 *                 NoiseField, SimState, Controls; per-frame tick,
 *                 particle pipeline, raster dispatcher
 *   §7 screen   — ASCII render: classify_trail_cell + paint_cell + HUD drawers
 *   §8 app      — signals, resize, named main-loop helpers
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (new noise permutation)
 *   n / N      next pattern (cycles through all 30 in tier order)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster particles / faster field drift
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra perin_noise_flow_showcase.c \
 *       -o perlin_flow -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two pieces composed together.
 *                  (1) Perlin noise — Ken Perlin's 1985 gradient noise
 *                      function. Pseudo-random values at integer lattice
 *                      points are smoothly interpolated to produce a
 *                      continuous, naturally-varying scalar field. We
 *                      use the standard 2-D variant with a 256-entry
 *                      permutation table (duplicated to 512 to avoid
 *                      modulo at lookup time), Perlin's quintic fade
 *                      curve t³(t·(t·6-15)+10), and 8 hashed gradient
 *                      directions per lattice corner.
 *                  (2) Flow visualisation — for each particle, read
 *                      n = perlin(x·scale, y·scale + t) and let
 *                      angle = n · 2π. Step the particle by
 *                      (cos angle, sin angle) · speed each tick.
 *                      Curving paths emerge because the noise field
 *                      is locally smooth, so neighbouring particles
 *                      follow similar directions and the system
 *                      organises into streamlines.
 *
 *                  The TIME OFFSET (the "+ t" in y) makes the field
 *                  drift slowly, so the flow evolves rather than
 *                  looping in a fixed pattern. The drift rate is
 *                  tunable; default ~0.1 noise-units per second.
 *
 * Data-structure : Scene composes six sub-structs (see §6):
 *                    Grid          — w, h, total_cells
 *                    RenderBuffers — per-cell glow + color band (the
 *                                    render contract between sim and
 *                                    display layers)
 *                    Particles     — fixed pool of MAX_PARTICLES,
 *                                    pool[0..n-1] alive
 *                    NoiseField    — Perlin permutation (perm[512]);
 *                                    registered as the active source
 *                                    via noise_activate() so perlin2d
 *                                    reads it without signature noise
 *                    SimState      — field_time (drift offset)
 *                    Controls      — paused, speed, theme, pattern
 *                  Everything BSS — no heap allocation after startup.
 *
 * Rendering      : ASCII only. Per-cell buf.glow drives a density
 *                  ramp via classify_trail_cell (§7):
 *                    glow > GLYPH_HIGH_THRESH → '#'  (BOLD)
 *                    glow > GLYPH_MID_THRESH  → '*'  (BOLD)
 *                    glow > GLOW_THRESHOLD    → '.'  (NORMAL)
 *                    else                     → blank (skip)
 *                  Each cell also tracks the COLOUR of the most-recent
 *                  particle to visit it (one of N_BANDS theme colours),
 *                  so multiple particle "tribes" produce visibly
 *                  distinct streams. The density ramp on top of the
 *                  colour variation gives the flow a textured, painted
 *                  look. paint_cell is the single ncurses-I/O point.
 *
 * Performance    : O(N_particles) per tick — each particle does 1
 *                  Perlin lookup + 1 cell paint. With 256 particles
 *                  at 60 Hz that's ~15K perlin evaluations per second,
 *                  about 120 K arithmetic ops — completely free.
 *                  Trail decay is O(W·H) per tick (one multiply per
 *                  cell), ~700 K ops per second on a 200×56 grid.
 *                  Headroom for any speedup.
 *
 * References     : Algorithm / math
 *                  ────────────────
 *                  • Perlin, K. (1985) — "An image synthesizer",
 *                    SIGGRAPH '85 Proceedings 19(3):287-296. The
 *                    original paper. Introduced gradient noise,
 *                    turbulence (Σ|octaves|), MARBLE, WOOD, FIRE —
 *                    most of Tier-5 in this file is directly from
 *                    this paper:
 *                    https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *                  • Perlin, K. (2002) — "Improving noise",
 *                    SIGGRAPH '02. The QUINTIC FADE curve in §5
 *                    (6t⁵ − 15t⁴ + 10t³) is from this paper; gives
 *                    C² continuity across lattice boundaries vs. the
 *                    1985 cubic's C¹. Why our noise looks smoother
 *                    than a naïve Perlin implementation.
 *                  • Ebert, D. S., Musgrave, F. K., Peachey, D.,
 *                    Perlin, K. & Worley, S. (2003) — "Texturing and
 *                    Modeling: A Procedural Approach" (3rd ed,
 *                    Morgan Kaufmann). The definitive reference for
 *                    procedural texture synthesis: ch. 6 (Perlin)
 *                    covers TURBULENCE / MARBLE / WOOD; ch. 16
 *                    (Musgrave) covers RIDGED multifractal and fBm
 *                    variants; ch. 18 (Worley) covers cellular noise.
 *                    Every Tier-4 and Tier-5 pattern in this file
 *                    traces back to a chapter here.
 *                  • Voss, R. F. (1985) — "Random fractal forgeries",
 *                    in R. A. Earnshaw, ed., "Fundamental Algorithms
 *                    for Computer Graphics", Springer NATO ASI.
 *                    Theoretical foundation of fBm (fbm_octaves) and
 *                    fractional Brownian terrain. Where the "octave
 *                    sum with halved amplitude / doubled frequency"
 *                    convention comes from.
 *                  • Bridson, R., Houriham, J. & Nordenstam, M.
 *                    (2007) — "Curl-noise for procedural fluid flow",
 *                    SIGGRAPH 2007. The CURL pattern is straight
 *                    out of this paper: build a divergence-free
 *                    vector field as the 2-D curl of a scalar
 *                    Perlin potential. Particles following it never
 *                    converge or diverge — incompressible flow.
 *
 *                  Visualisation
 *                  ─────────────
 *                  • Quilez, I. — "Domain warping" (interactive
 *                    article):
 *                    https://iquilezles.org/articles/warp/
 *                    WARP, WARP_DEEP, WARPRIDGE are all from this
 *                    page. Quilez also covers the 2-step recursive
 *                    warp that gives WARP_DEEP its swirly look.
 *                  • Paul Bourke — "Character representation of grey
 *                    scale images":
 *                    https://paulbourke.net/dataformats/asciiart/
 *                    The canonical density → glyph ramp reference.
 *                    Underlies the .:low / *:mid / #:high mapping
 *                    in §7's scene_draw.
 *
 *                  Online tutorials
 *                  ────────────────
 *                  • The Book of Shaders — chapter on noise:
 *                    https://thebookofshaders.com/11/
 *                    Interactive WebGL examples of the same patterns
 *                    implemented in this file. Useful side-by-side
 *                    comparison when calibrating pattern parameters.
 *                  • Wikipedia — "Perlin noise":
 *                    https://en.wikipedia.org/wiki/Perlin_noise
 *                    Concise starting point with the lattice + fade
 *                    + gradient diagrams.
 *
 *                  See also
 *                  ────────
 *                  • ./curl_noise_vector_field.c — the standalone
 *                    showcase of curl noise; CURL pattern here is its
 *                    minimal embedding. That file covers Bridson 2007
 *                    in much more depth.
 *                  • ./domain_warped_noise_iq_style.c — the WARP /
 *                    WARP_DEEP showcase as a standalone Quilez-style
 *                    demo with a deeper treatment of the technique.
 *                  • ./midpoint_displacement_coastline.c,
 *                    ./magnetic_fields.c, ./flow_field_particles.c —
 *                    same Scene / Grid / *Field / SimState / Controls
 *                    composition pattern applied to different
 *                    algorithms; reading them together shows how the
 *                    architecture stays constant while the algorithm
 *                    changes.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Perlin noise is a way to build a SMOOTH, naturally-varying scalar
 * field over an arbitrary domain. Think of it as a height map of
 * rolling hills with no axis bias and no obvious repetition. By
 * itself it's just a cloud-like pattern; combine it with anything
 * directional (here: an angle for particle motion) and you get
 * structures that look organic — wind, fluid, smoke, cellular flow.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the screen has invisible arrows on it, one per cell. Each
 * arrow points in a direction determined by the local noise value.
 * Drop hundreds of dust particles on the screen. Each particle reads
 * its arrow and steps in that direction. The dust accumulates along
 * the streamlines — visually the same as iron filings around a
 * magnet, except the field is a 2-D Perlin "wind".
 *
 * Why does the result look organic? Because Perlin noise is
 * SPATIALLY COHERENT — neighbouring points have nearby values, so
 * neighbouring particles see similar arrows, so streams form. Pure
 * white noise (uncorrelated random per cell) would scatter the
 * particles; Perlin's smoothness is what produces flow.
 *
 * Visible layers:
 *   1. Trails of '.', '*', '#' in 4 theme colours — recently-visited
 *      cells in their colour, density-shaded by glow.
 *   2. A faint structural pattern — high-density convergence zones
 *      (where streams meet) and sparse divergence zones (where they
 *      separate) emerge naturally from the noise topology.
 *   3. The flow EVOLVES — over a few seconds you see streams curve,
 *      bifurcate, dissolve, reform. That's the field-time drift.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Shuffle perm[256], duplicate to perm[512]. Spawn N
 *     particles at random in-bounds positions, each with a random
 *     colour (1 of 4 theme tribes) and a random max_age.
 *  2. PER FRAME (driver is scene_tick in §6):
 *     a. advance_noise_time:  sim.field_time += FIELD_DRIFT · dt.
 *     b. if particle pattern (FLOW/VORTEX/CURL/JITTER):
 *          decay_trail_glow  every cell *= exp(-GLOW_DECAY · dt).
 *          step_all_particles — per particle:
 *            sample velocity via particle_velocity_at (pattern-mode
 *              dispatcher: FLOW = angle·π, VORTEX = +π/2,
 *              CURL = curl_noise_2d (normalised), JITTER = FLOW + noise).
 *            advect_particle_euler: (x, y) ← (x, y) + (vx, vy) · dt.
 *            deposit_trail_hit: buf.glow[idx] = 1.0; buf.color[idx] = tribe.
 *            if particle_is_expired: respawn at random in-bounds.
 *        else (static-field pattern):
 *          rasterise_pattern_field — overwrite every cell from the
 *          active pattern_*_at evaluator.
 *  3. The user can press 'r' to re-shuffle perm[] and respawn all
 *     particles (or 'n'/'p' to switch pattern). There is no automatic
 *     reset — the flow runs indefinitely until you ask for a new one.
 *
 * KEY FORMULAS
 * ────────────
 *  Perlin 2-D                   :
 *    1. (X, Y) = floor(x), floor(y); (xf, yf) = x − X, y − Y
 *    2. (u, v) = fade(xf), fade(yf)   where fade(t) = 6t⁵ − 15t⁴ + 10t³
 *    3. Hash 4 lattice corners via noise.perm[] table
 *    4. Compute 4 gradient·offset dot products (grad function)
 *    5. Bilinear lerp using (u, v) → noise value in roughly [-1, 1]
 *  FLOW angle                   : θ = n · π
 *  VORTEX angle                 : θ = n · π + π/2          (tangent)
 *  JITTER angle                 : θ = n · π + uniform jitter
 *  CURL velocity                : (vx, vy) = (∂ψ/∂y, -∂ψ/∂x), ψ = perlin2d
 *  Step                         : (Δx, Δy) = velocity · dt
 *  Field-drift                  : field_time' = field_time + FIELD_DRIFT · dt
 *  Glow decay                   : glow' = glow · exp(-GLOW_DECAY · dt)
 *
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

    MAX_PARTICLES     = 1024,
    N_PARTICLES_DEF   =  256,

    /* Particle lifetime range (ticks). Random max_age in this window
     * so different particles "tire out" at different times → the
     * streamlines are constantly being refreshed at random points. */
    AGE_MIN_TICKS     =  60,        /* 1.0 s @ 60 Hz */
    AGE_MAX_TICKS     = 360,        /* 6.0 s         */

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    /* Speed multiplier (cells per second per particle). Default 8 ≈
     * 0.13 cells/tick at 60 Hz — slow enough to read individual
     * trails, fast enough that the flow evolves visibly. */
    SPEED_MIN         =   1,
    SPEED_DEF         =   8,
    SPEED_MAX         =  64,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_TRAIL_BASE   =   3,        /* PAIR_TRAIL_BASE..+3 = 4 trail colours */
    PAIR_FLASH        =   7,        /* vestigial — cross-file palette parity */

    /* Palette width — number of distinct colour bands per theme.
     * Every LineSpec / particle carries pair_idx ∈ [0, N_BANDS). */
    N_BANDS           =   4,
};

/* Glow decay rate per second. 0.6 → glow ≈ 5% after 5 seconds, so
 * trails persist long enough to read but fade out cleanly. */
#define GLOW_DECAY          0.6f
#define GLOW_THRESHOLD      0.05f

/* ── HUD layout ──────────────────────────────────────────────────── *
 * Top HUD carries DATA, bottom HUD carries ACTIONS:
 *   row 0           : title + state bar (fps, Hz, state + [N/M], speed)
 *   row 1           : pattern/theme/palette + scale/drift/map
 *   row HUD_TOP..N-2: noise-field map
 *   row N-1         : keyboard action hint
 */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* Noise scale: lower = larger features, higher = finer detail. 0.04
 * gives noise periods of ~25 cells — balanced for 200×56 grids. */
#define NOISE_SCALE         0.04f

/* Field drift per second (in noise-coordinate units). Slow drift so
 * the flow visibly evolves but doesn't churn. */
#define FIELD_DRIFT         0.10f

/* Density thresholds for the ASCII glyph ramp. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/*
 * Pattern — 30 distinct visualisations of a Perlin-noise field,
 * spanning simple → complex. Cycle with n/p.
 *
 * INTENT. The simulation kernel (Perlin noise + per-frame field drift)
 * is shared across all patterns; only the pattern-specific evaluator
 * differs. Four patterns (FLOW, VORTEX, CURL, JITTER) run particles
 * over a derived vector field; the other 26 rasterise a scalar field
 * into the trail buffer each frame.
 *
 * TIER ORGANISATION.
 *   1 (direct vis)        — single Perlin sample → glow/colour
 *   2 (shape transforms)  — ridge, billow, periodic functions of noise
 *   3 (domain warps)      — Quilez-style warps, slope, contour
 *   4 (fractal/octave)    — fBm, ridged, turbulence multifractals
 *   5 (texture + exotic)  — marble, wood, fire, particle modes, chaos
 */
typedef enum {
    /* Tier 1 — direct visualisations */
    PATTERN_FLOW       =  0,    /* particle: angle = noise·π          */
    PATTERN_HEIGHT     =  1,    /* glow = noise·0.5 + 0.5             */
    PATTERN_VALLEYS    =  2,    /* glow = 1 - height                  */
    PATTERN_BANDED     =  3,    /* quantised noise levels             */
    PATTERN_BINARY     =  4,    /* threshold cut at 0.5               */

    /* Tier 2 — shape transforms */
    PATTERN_RIDGE      =  5,    /* 1 - |n| — sharp peaks at n=0       */
    PATTERN_BILLOW     =  6,    /* |n| — cloud bumps                  */
    PATTERN_ZEBRA      =  7,    /* sin(n·freq) — stripes              */
    PATTERN_RINGS      =  8,    /* sin(radius + noise) — concentric   */
    PATTERN_PLASMA     =  9,    /* sin(n + sin(x+y)) — plasma         */

    /* Tier 3 — domain warps + gradient */
    PATTERN_CONTOUR    = 10,    /* level-set bands (topographic)      */
    PATTERN_WARP       = 11,    /* single-step domain warp            */
    PATTERN_WARP_DEEP  = 12,    /* recursive 2-step warp              */
    PATTERN_WARPRIDGE  = 13,    /* ridge of warped field              */
    PATTERN_SLOPE      = 14,    /* gradient magnitude (finite diff)   */

    /* Tier 4 — fractal / octave compositions */
    PATTERN_FBM        = 15,    /* 4-octave fractional Brownian       */
    PATTERN_FBM_HIGH   = 16,    /* 8-octave high-detail fBm           */
    PATTERN_RIDGED     = 17,    /* Musgrave ridged multifractal       */
    PATTERN_TURBLENC   = 18,    /* sum of |octaves| — Perlin marble   */
    PATTERN_FBM_INV    = 19,    /* 1 - fbm                            */

    /* Tier 5 — texture + exotic + particle modes */
    PATTERN_MARBLE     = 20,    /* sin(x·k + turbulence·a) veins      */
    PATTERN_WOOD       = 21,    /* fract(radius·k + turbulence·a)     */
    PATTERN_FIRE       = 22,    /* turbulence shaped by y-mask        */
    PATTERN_CLOUDS     = 23,    /* fbm smoothstep threshold           */
    PATTERN_CAVES      = 24,    /* fbm inverse-threshold (carve out)  */
    PATTERN_STARS      = 25,    /* high-octave sparse threshold       */
    PATTERN_VORTEX     = 26,    /* particle: tangent (perp to flow)   */
    PATTERN_CURL       = 27,    /* particle: divergence-free curl     */
    PATTERN_JITTER     = 28,    /* particle: flow + angle jitter      */
    PATTERN_CHAOS      = 29,    /* multi-noise composite              */

    N_PATTERNS         = 30,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_FLOW:       return "FLOW     ";
    case PATTERN_HEIGHT:     return "HEIGHT   ";
    case PATTERN_VALLEYS:    return "VALLEYS  ";
    case PATTERN_BANDED:     return "BANDED   ";
    case PATTERN_BINARY:     return "BINARY   ";
    case PATTERN_RIDGE:      return "RIDGE    ";
    case PATTERN_BILLOW:     return "BILLOW   ";
    case PATTERN_ZEBRA:      return "ZEBRA    ";
    case PATTERN_RINGS:      return "RINGS    ";
    case PATTERN_PLASMA:     return "PLASMA   ";
    case PATTERN_CONTOUR:    return "CONTOUR  ";
    case PATTERN_WARP:       return "WARP     ";
    case PATTERN_WARP_DEEP:  return "WARP_DEEP";
    case PATTERN_WARPRIDGE:  return "WARPRIDGE";
    case PATTERN_SLOPE:      return "SLOPE    ";
    case PATTERN_FBM:        return "FBM      ";
    case PATTERN_FBM_HIGH:   return "FBM_HIGH ";
    case PATTERN_RIDGED:     return "RIDGED   ";
    case PATTERN_TURBLENC:   return "TURBLENC ";
    case PATTERN_FBM_INV:    return "FBM_INV  ";
    case PATTERN_MARBLE:     return "MARBLE   ";
    case PATTERN_WOOD:       return "WOOD     ";
    case PATTERN_FIRE:       return "FIRE     ";
    case PATTERN_CLOUDS:     return "CLOUDS   ";
    case PATTERN_CAVES:      return "CAVES    ";
    case PATTERN_STARS:      return "STARS    ";
    case PATTERN_VORTEX:     return "VORTEX   ";
    case PATTERN_CURL:       return "CURL     ";
    case PATTERN_JITTER:     return "JITTER   ";
    case PATTERN_CHAOS:      return "CHAOS    ";
    default:                 return "?        ";
    }
}

/* WARP pattern: amount the warp distorts the input coordinates.
 * Higher = more swirly, lower = closer to plain noise. */
#define WARP_AMOUNT         5.0f

/* FBM pattern: how many octaves to stack. 4 is a good sweet spot —
 * adds visible fine detail without becoming noisy. */
#define FBM_OCTAVES         4

/* CONTOUR pattern: noise values that draw bands. The band width
 * controls how thick each contour line appears. */
#define CONTOUR_BAND_WIDTH  0.04f
static const float CONTOUR_LEVELS[4] = { 0.20f, 0.40f, 0.60f, 0.80f };

/* ── Tier-2 shape-transform constants ─────────────────────────────── */
#define BANDED_LEVELS          6      /* quantisation steps for BANDED        */
#define BINARY_THRESHOLD       0.5f   /* cutoff for BINARY                    */
#define ZEBRA_FREQ             3.0f   /* sin frequency in noise units         */
#define RING_FREQ              0.30f  /* radial wave frequency (rings/cell)   */
#define RING_AMP               1.0f   /* noise perturbation of rings          */
#define PLASMA_FREQ            0.05f  /* (x+y) modulation freq for PLASMA     */

/* ── Tier-3 warp + slope constants ────────────────────────────────── */
#define WARP_DEEP_AMOUNT       8.0f   /* magnitude of 2nd-level warp          */
#define SLOPE_EPS              1.0f   /* finite-diff step in cells            */
#define SLOPE_GAIN             4.0f   /* multiply slope magnitude             */

/* ── Tier-4 octave constants ──────────────────────────────────────── */
#define FBM_HIGH_OCTAVES       8      /* high-detail fBm octave count         */
#define MULTIFRAC_LACUNARITY   2.0f   /* per-octave frequency multiplier      */
#define MULTIFRAC_GAIN         0.5f   /* per-octave amplitude decay           */

/* ── Tier-5 texture parameters ────────────────────────────────────── */
#define MARBLE_FREQ            0.06f  /* base marble vein frequency           */
#define MARBLE_AMP             5.0f   /* turbulence amplitude in marble sin   */
#define WOOD_FREQ              0.08f  /* concentric grain ring frequency      */
#define WOOD_AMP               3.0f   /* grain warp amount                    */
#define FIRE_DRIFT_MULT        4.0f   /* fire flickers faster than base drift */
#define CLOUDS_THRESH_LO       0.45f  /* smoothstep lo for CLOUDS             */
#define CLOUDS_THRESH_HI       0.65f  /* smoothstep hi                        */
#define CAVES_THRESH_LO        0.40f  /* smoothstep lo for CAVES (inverted)   */
#define CAVES_THRESH_HI        0.60f
#define STARS_FREQ_MULT        4.0f   /* STARS uses ×4 noise frequency        */
#define STARS_THRESHOLD        0.85f  /* only top 15% intensity becomes stars */
#define JITTER_ANGLE_RANGE     1.5f   /* radians of angle jitter in JITTER    */
#define CHAOS_WARP             10.0f  /* warp amount in CHAOS mix             */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* §8 main loop — spiral-of-death dt clamp + terminal redraw cap
 * (Glenn Fiedler's "Fix Your Timestep"). */
#define DT_MAX_NS       (100 * NS_PER_MS)   /* hard cap on per-frame dt */
#define FRAME_CAP_FPS   60                  /* terminal redraw budget   */

/* Particle-mode helpers — named constants for the small magic
 * numbers in particle_velocity_at and curl_noise_2d. */
#define VORTEX_TURN_RADIANS  ((float)M_PI * 0.5f)   /* 90° perpendicular */
#define VECTOR_EPSILON       1e-6f                  /* below this |v|, skip normalise */

/* glow_to_col scale — multiplies a glow in [0, 1] to land in
 * [0, N_BANDS-ε]; the int-cast then floors into [0, N_BANDS-1]. */
#define GLOW_COL_SCALE       3.99f

/* ── HUD layout (column widths) ──────────────────────────────────── *
 * Row-1 segments are laid out left-to-right at fixed column widths so
 * each segment knows where the next one starts. */
#define HUD_PATTERN_FIELD_W   20    /* " pattern:XXXXXXXXX " slot */
#define HUD_THEME_FIELD_W     17    /* " theme:XXXXXXXX "   slot */
#define HUD_PALETTE_LABEL_W    9    /* " palette:"          slot */

/*
 * Theme — one complete colour palette for the trail layer. Ten of
 * these live in themes[], cycled by t/T.
 *
 * INTENT. The smallest data needed to recolour every cell in the
 * render output: N_BANDS (=4) ramp colours arranged dim → bright,
 * plus a "flash" accent (vestigial here — kept for cross-file palette
 * parity with magnetic_fields.c, midpoint_displacement_coastline.c,
 * etc.). The four-band split forces theme authors to design the ramp
 * as a coherent low → high progression — same discipline as a Houdini
 * ramp parameter or Substance Designer gradient.
 *
 * BANDING MODEL. Both particle-mode and static-field patterns write a
 * pair_idx ∈ [0, 4) per cell into RenderBuffers.color. For particle
 * modes, the band is the spawning particle's "tribe" (rand() & 3 at
 * spawn). For static-field patterns, the band is derived from the
 * noise intensity (glow_to_col). The renderer reads buf.color[idx]
 * and looks up themes[ctrl.current_theme].trail[band] at draw time.
 *
 * COLOUR FORMAT. xterm-256 indices (NOT RGB). When the terminal
 * exposes fewer than 256 colours, theme_apply() substitutes a fixed
 * 8-colour cycle (COLOR_BLUE/CYAN/MAGENTA/YELLOW) so the demo still
 * runs on legacy TTYs.
 *
 * REFERENCE. The 256-colour cube layout (16 base + 6³ cube +
 * 24-step grayscale ramp) is documented in XTerm's ctlseqs.ms. The
 * trail values in themes[] are picked from the cube. See also
 * CLAUDE.md "Theme Palette Brightness" for the project-wide
 * bright-half constraint.
 */
typedef struct {
    const char *name;              /* short uppercase label shown in HUD     */
    short       trail[N_BANDS];    /* ramp: 0 = dim/low, N_BANDS-1 = bright  */
    short       flash;             /* vestigial — kept for cross-file parity */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      trail0 trail1 trail2 trail3 flash */
    { "DEFAULT", {  33,  117,  220,  220 }, 226 },   /* sky / cyan / gold      */
    { "MATRIX",  {  22,   34,   46,  118 }, 226 },   /* greens                 */
    { "NOVA",    {  53,  129,  201,  219 }, 226 },   /* purple → magenta       */
    { "MONO",    { 240,  244,  250,  254 }, 226 },   /* greyscale              */
    { "OCEAN",   {  17,   33,   39,   51 }, 226 },   /* navy → cyan            */
    { "FIRE",    {  88,  124,  208,  226 }, 196 },   /* dark red → yellow      */
    { "EARTH",   {  58,  100,  173,  230 }, 226 },   /* brown → cream          */
    { "FOREST",  {  22,   28,   64,  144 }, 226 },   /* greens                 */
    { "DESERT",  {  94,  130,  173,  222 }, 226 },   /* sandy                  */
    { "ARCTIC",  {  18,   39,  159,  231 }, 226 },   /* navy → ice → white     */
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
        for (int i = 0; i < N_BANDS; i++)
            init_pair(PAIR_TRAIL_BASE + i, t->trail[i], -1);
        init_pair(PAIR_FLASH, t->flash, -1);
    } else {
        static const short fallback[N_BANDS] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < N_BANDS; i++)
            init_pair(PAIR_TRAIL_BASE + i, fallback[i], -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
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
/* §5  noise — Perlin 2-D                                                 */
/* ===================================================================== */

/*
 * NoiseField — the algorithm's data source. Holds Ken Perlin's
 * 256-entry permutation table, duplicated to perm[512] so the
 * lookups perm[X] and perm[X+1] never need an explicit modulo.
 *
 * INTENT. The noise field IS the scene's algorithm state — every
 * pattern in §6 reads from it. Placing perm[] in a struct (rather
 * than at file scope) means it lives on Scene like every other
 * piece of simulation state, and a reader of Scene can see where the
 * noise actually lives.
 *
 * REFERENCE. Perlin (1985) "An image synthesizer" — original paper.
 * The duplication trick (perm[i+256] = perm[i]) is the standard
 * accelerator from Perlin's reference implementation.
 */
typedef struct {
    uint8_t perm[512];     /* shuffled 0..255 repeated twice */
} NoiseField;

/*
 * g_active_noise — registered NoiseField that perlin2d() reads from.
 *
 * RATIONALE. perlin2d is called thousands of times per frame; threading
 * a NoiseField* through every pattern_*_at signature would pollute every
 * function. Instead the active NoiseField is registered once per
 * scene_reset via noise_activate(). Mirrors how ncurses' stdscr works
 * — there's logically one "current" instance and code reads it through
 * a stable global handle.
 */
static const NoiseField *g_active_noise;

static inline void noise_activate(const NoiseField *nf)
{
    g_active_noise = nf;
}

/*
 * shuffle_noise_field — Fisher-Yates the 0..255 sequence using rand(),
 * duplicate into the upper half of perm[], then activate this
 * NoiseField as the current one. Different rand() state → different
 * permutation → wholly different noise field. Called at each reset.
 */
static void shuffle_noise_field(NoiseField *nf)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
    for (int i = 0; i < 256; i++) {
        nf->perm[i]       = base[i];
        nf->perm[i + 256] = base[i];
    }
    noise_activate(nf);
}

/*
 * fade — Perlin's improved-noise quintic fade curve.
 *   fade(t) = 6t⁵ − 15t⁴ + 10t³
 * Has zero first AND second derivative at t=0 and t=1, so the
 * resulting noise is C² continuous across lattice boundaries.
 */
static inline float fade(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

static inline float lerp(float a, float b, float t) { return a + t * (b - a); }

/* ── small math helpers (used by Tier-2..5 patterns) ──────────────── */
static inline float fract(float x)             { return x - floorf(x); }
static inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}
static inline float smoothstep(float a, float b, float x)
{
    float t = clampf((x - a) / (b - a), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/* ridge_shape — convert raw noise n ∈ [-1, 1] to a RIDGE value in
 * [0, 1]: sharp peak at n=0, falling linearly toward |n|=1. The
 * inversion-of-absolute-value trick that Musgrave used to convert
 * fBm into ridged multifractal terrain. */
static inline float ridge_shape(float n)  { return 1.0f - fabsf(n); }

/* billow_shape — convert raw noise n ∈ [-1, 1] to a BILLOW value:
 * zero at n=0, peaks at ±1. Topological inverse of ridge_shape;
 * gives cloud-like rounded bumps. */
static inline float billow_shape(float n) { return fabsf(n); }

/*
 * grad — pick one of 8 gradient directions based on the low 3 bits
 * of the hash, and return its dot product with the offset (x, y).
 * The 8 directions are the diagonals and axes of a 2-D unit square.
 */
static inline float grad(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

/*
 * perlin2d — Perlin's 2-D noise function. Returns a value roughly in
 * [-1, 1]. The magnitude bound is √(N/2) for N-D noise, ≈ 1 in 2-D
 * after the small constant scaling we apply.
 */
static float perlin2d(float x, float y)
{
    const uint8_t *perm = g_active_noise->perm;

    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x);
    y -= floorf(y);

    float u = fade(x);
    float v = fade(y);

    int A  = perm[X    ] + Y;
    int B  = perm[X + 1] + Y;

    float n00 = grad(perm[A    ], x,        y       );
    float n10 = grad(perm[B    ], x - 1.0f, y       );
    float n01 = grad(perm[A + 1], x,        y - 1.0f);
    float n11 = grad(perm[B + 1], x - 1.0f, y - 1.0f);

    return lerp(
        lerp(n00, n10, u),
        lerp(n01, n11, u),
        v
    );
}

/*
 * pattern_height_at — basic noise visualization. Returns noise value
 * remapped to [0, 1] so it can be used directly as a glow intensity.
 */
static float pattern_height_at(float x, float y, float t)
{
    float n = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    return n * 0.5f + 0.5f;
}

/*
 * pattern_warp_at — domain-warped noise (Inigo Quilez style). Sample
 * the noise field, use the result to OFFSET the coordinates, then
 * sample again. Produces curving, organic-looking patterns that feel
 * less "lumpy" than raw Perlin noise. The two-step warp uses
 * different y-offsets so the qx and qy components are statistically
 * independent.
 */
static float pattern_warp_at(float x, float y, float t)
{
    float qx = perlin2d(x * NOISE_SCALE,
                        y * NOISE_SCALE + t);
    float qy = perlin2d((x + 5.2f) * NOISE_SCALE,
                        (y + 1.3f) * NOISE_SCALE + t);
    float n  = perlin2d((x + qx * WARP_AMOUNT) * NOISE_SCALE,
                        (y + qy * WARP_AMOUNT) * NOISE_SCALE + t);
    return n * 0.5f + 0.5f;
}

/*
 * pattern_fbm_at — fractional Brownian motion. Sums multiple octaves
 * of Perlin noise: each octave doubles the frequency and halves the
 * amplitude. The result has the same large-scale shape as plain
 * Perlin but with self-similar fine detail at smaller scales.
 *
 * For terrain, fBm is the standard way to add "rocks on hills on
 * mountains" — every scale gets a contribution.
 */
static float pattern_fbm_at(float x, float y, float t)
{
    float total   = 0.0f;
    float amp     = 1.0f;
    float freq    = 1.0f;
    float max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * perlin2d(x * NOISE_SCALE * freq,
                                  y * NOISE_SCALE * freq + t * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;
}

/* ── generalised octave compositions ───────────────────────────────── *
 * Three multifractal kernels share the same octave-loop shape but
 * combine the per-octave contributions differently. Each returns a
 * value in roughly [0, 1] suitable for direct use as glow. */

/* fbm_octaves — generalised fBm (Voss/Mandelbrot). Sum signed octaves,
 * remap to [0, 1]. Variant of pattern_fbm_at with configurable count. */
static float fbm_octaves(float x, float y, float t,
                          int octaves, float lacunarity, float gain)
{
    float total = 0.0f, amp = 1.0f, freq = 1.0f, max_amp = 0.0f;
    for (int o = 0; o < octaves; o++) {
        total   += amp * perlin2d(x * NOISE_SCALE * freq,
                                   y * NOISE_SCALE * freq + t * freq);
        max_amp += amp;
        amp     *= gain;
        freq    *= lacunarity;
    }
    return (total / max_amp) * 0.5f + 0.5f;
}

/* turbulence_octaves — sum of |perlin| across octaves (Perlin 1985).
 * Result has sharp ridges along zero-crossings — the canonical input
 * to MARBLE, FIRE, WOOD textures. */
static float turbulence_octaves(float x, float y, float t,
                                 int octaves, float lacunarity, float gain)
{
    float total = 0.0f, amp = 1.0f, freq = 1.0f, max_amp = 0.0f;
    for (int o = 0; o < octaves; o++) {
        total   += amp * fabsf(perlin2d(x * NOISE_SCALE * freq,
                                         y * NOISE_SCALE * freq + t * freq));
        max_amp += amp;
        amp     *= gain;
        freq    *= lacunarity;
    }
    return total / max_amp;
}

/* ridged_octaves — Musgrave's ridged multifractal: each octave squares
 * ridge_shape(noise) and weights it by the previous octave's value.
 * Produces sharp mountain-ridge topology. Reference: Musgrave (1998)
 * "Texturing & Modeling: A Procedural Approach", ch. 16. */
static float ridged_octaves(float x, float y, float t,
                             int octaves, float lacunarity, float gain)
{
    float total = 0.0f, amp = 1.0f, freq = 1.0f, max_amp = 0.0f;
    for (int o = 0; o < octaves; o++) {
        float n = perlin2d(x * NOISE_SCALE * freq,
                           y * NOISE_SCALE * freq + t * freq);
        float r = ridge_shape(n);
        total   += amp * r * r;
        max_amp += amp;
        amp     *= gain;
        freq    *= lacunarity;
    }
    return total / max_amp;
}

/* ── curl noise (divergence-free 2-D vector field) ─────────────────── *
 * Computed by central-difference of the scalar potential ψ = perlin2d.
 * The resulting (vx, vy) = (∂ψ/∂y, -∂ψ/∂x) has ∇·v = 0 identically,
 * so particles following it never converge or diverge — they only
 * swirl. The foundation of CURL pattern.
 *
 * Reference: Bridson, Houriham & Nordenstam (2007) "Curl-noise for
 * procedural fluid flow", SIGGRAPH. */
static void curl_noise_2d(float x, float y, float t,
                           float *out_vx, float *out_vy)
{
    const float eps = SLOPE_EPS;
    float dn_dy = (perlin2d(x * NOISE_SCALE,
                             (y + eps) * NOISE_SCALE + t)
                 - perlin2d(x * NOISE_SCALE,
                             (y - eps) * NOISE_SCALE + t)) * 0.5f;
    float dn_dx = (perlin2d((x + eps) * NOISE_SCALE,
                             y * NOISE_SCALE + t)
                 - perlin2d((x - eps) * NOISE_SCALE,
                             y * NOISE_SCALE + t)) * 0.5f;
    *out_vx =  dn_dy;
    *out_vy = -dn_dx;
}

/*
 * pattern_contour_at — only "lights up" cells near specific noise
 * levels (CONTOUR_LEVELS). The result is intensity (1.0 at the level,
 * fading to 0 at ±CONTOUR_BAND_WIDTH) plus the index of the closest
 * level (returned via *out_band, so the caller can colour each
 * contour line distinctly).
 *
 * Visually like a topographic map — the noise value is constant along
 * each line, so neighbouring cells track each other in a smooth path.
 */
static float pattern_contour_at(float x, float y, float t, int *out_band)
{
    float n = pattern_height_at(x, y, t);
    float min_d = 1.0f;
    int closest = 0;
    int n_levels = (int)(sizeof CONTOUR_LEVELS / sizeof CONTOUR_LEVELS[0]);
    for (int i = 0; i < n_levels; i++) {
        float d = fabsf(n - CONTOUR_LEVELS[i]);
        if (d < min_d) { min_d = d; closest = i; }
    }
    *out_band = closest;
    if (min_d < CONTOUR_BAND_WIDTH) {
        return 1.0f - min_d / CONTOUR_BAND_WIDTH;
    }
    return 0.0f;
}

/* ── Tier-1 — direct visualisations ──────────────────────────────── */

/* VALLEYS — the visual inverse of HEIGHT. High noise becomes dark,
 * low noise becomes bright. Same field, opposite reading. */
static float pattern_valleys_at(float x, float y, float t)
{
    return 1.0f - pattern_height_at(x, y, t);
}

/* BANDED — quantise the height field to BANDED_LEVELS discrete steps.
 * Reads as terraced contours, like an old paletted heightmap. */
static float pattern_banded_at(float x, float y, float t)
{
    float h = pattern_height_at(x, y, t);
    return floorf(h * BANDED_LEVELS) / (float)BANDED_LEVELS;
}

/* BINARY — hard threshold at 0.5: any cell above is fully on, below
 * fully off. The cellular-look limit of HEIGHT visualisation. */
static float pattern_binary_at(float x, float y, float t)
{
    return pattern_height_at(x, y, t) > BINARY_THRESHOLD ? 1.0f : 0.0f;
}

/* ── Tier-2 — shape transforms ──────────────────────────────────── */

/* RIDGE — pass raw noise through ridge_shape: peaks where the noise
 * crosses zero. Gives sharp mountain-ridge silhouettes from a single
 * Perlin sample (no octave summation needed). */
static float pattern_ridge_at(float x, float y, float t)
{
    float n = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    return ridge_shape(n);
}

/* BILLOW — pass raw noise through billow_shape: clouds at noise
 * extremes, valleys at zero-crossings. Inverse topology of RIDGE. */
static float pattern_billow_at(float x, float y, float t)
{
    float n = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    return billow_shape(n);
}

/* ZEBRA — apply sin to noise scaled by ZEBRA_FREQ. Produces parallel
 * stripes that curve along the noise iso-lines. */
static float pattern_zebra_at(float x, float y, float t)
{
    float n = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    return 0.5f + 0.5f * sinf(n * (float)M_PI * ZEBRA_FREQ);
}

/* RINGS — concentric rings from screen centre, perturbed by noise.
 * radius·RING_FREQ provides the spacing; noise warps the rings into
 * organic, non-perfect circles. */
static float pattern_rings_at(float x, float y, float t,
                                float cx, float cy)
{
    float dx = x - cx, dy = y - cy;
    float r  = sqrtf(dx * dx + dy * dy);
    float n  = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    return 0.5f + 0.5f * sinf(r * RING_FREQ + n * RING_AMP);
}

/* PLASMA — nested sinusoid: sin(noise·π + sin((x+y)·k)). Combines a
 * spatial sin wave with a noise-driven phase shift to produce the
 * classic 90s "plasma" demoscene effect, with Perlin noise instead
 * of pure trig. */
static float pattern_plasma_at(float x, float y, float t)
{
    float n = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    return 0.5f + 0.5f * sinf(n * (float)M_PI
                              + sinf((x + y) * PLASMA_FREQ));
}

/* ── Tier-3 — domain warps + gradient ───────────────────────────── */

/* WARP_DEEP — recursive 2-step domain warp. The output of the first
 * warp drives a second-stage offset, giving much more swirly, organic
 * structure than single WARP. Inigo Quilez's "deep warp" technique. */
static float pattern_warp_deep_at(float x, float y, float t)
{
    float qx = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    float qy = perlin2d((x + 5.2f) * NOISE_SCALE,
                        (y + 1.3f) * NOISE_SCALE + t);
    float rx = perlin2d((x + qx * WARP_AMOUNT) * NOISE_SCALE,
                        (y + qy * WARP_AMOUNT) * NOISE_SCALE + t);
    float ry = perlin2d((x + qx * WARP_AMOUNT + 5.2f) * NOISE_SCALE,
                        (y + qy * WARP_AMOUNT + 1.3f) * NOISE_SCALE + t);
    float n  = perlin2d((x + rx * WARP_DEEP_AMOUNT) * NOISE_SCALE,
                        (y + ry * WARP_DEEP_AMOUNT) * NOISE_SCALE + t);
    return n * 0.5f + 0.5f;
}

/* WARPRIDGE — domain-warp the input, then apply ridge_shape. The
 * warp curves the ridges into wandering rivers and canyons. */
static float pattern_warpridge_at(float x, float y, float t)
{
    float qx = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    float qy = perlin2d((x + 5.2f) * NOISE_SCALE,
                        (y + 1.3f) * NOISE_SCALE + t);
    float n  = perlin2d((x + qx * WARP_AMOUNT) * NOISE_SCALE,
                        (y + qy * WARP_AMOUNT) * NOISE_SCALE + t);
    return ridge_shape(n);
}

/* SLOPE — finite-difference gradient magnitude of the noise field.
 * Highlights iso-line edges; flat regions are dark, steep cliffs
 * bright. Mathematically: |∇perlin2d|. */
static float pattern_slope_at(float x, float y, float t)
{
    const float eps = SLOPE_EPS;
    float n_xp = perlin2d((x + eps) * NOISE_SCALE, y * NOISE_SCALE + t);
    float n_xm = perlin2d((x - eps) * NOISE_SCALE, y * NOISE_SCALE + t);
    float n_yp = perlin2d(x * NOISE_SCALE, (y + eps) * NOISE_SCALE + t);
    float n_ym = perlin2d(x * NOISE_SCALE, (y - eps) * NOISE_SCALE + t);
    float dn_dx = (n_xp - n_xm) * 0.5f;
    float dn_dy = (n_yp - n_ym) * 0.5f;
    float mag = sqrtf(dn_dx * dn_dx + dn_dy * dn_dy);
    return clampf(mag * SLOPE_GAIN, 0.0f, 1.0f);
}

/* ── Tier-4 — fractal / octave compositions ─────────────────────── */

/* FBM_HIGH — FBM with FBM_HIGH_OCTAVES (default 8). Doubles the
 * detail of standard FBM at ~2× the cost. */
static float pattern_fbm_high_at(float x, float y, float t)
{
    return fbm_octaves(x, y, t, FBM_HIGH_OCTAVES,
                        MULTIFRAC_LACUNARITY, MULTIFRAC_GAIN);
}

/* RIDGED — Musgrave ridged multifractal. Sharp mountain ridges with
 * fractal detail at every scale. */
static float pattern_ridged_at(float x, float y, float t)
{
    return ridged_octaves(x, y, t, FBM_OCTAVES,
                           MULTIFRAC_LACUNARITY, MULTIFRAC_GAIN);
}

/* TURBLENC — Perlin's turbulence: sum of |octaves|. Sharper than
 * fBm, with discontinuous derivatives at zero-crossings. The base
 * input to MARBLE, WOOD, FIRE. */
static float pattern_turblenc_at(float x, float y, float t)
{
    return turbulence_octaves(x, y, t, FBM_OCTAVES,
                               MULTIFRAC_LACUNARITY, MULTIFRAC_GAIN);
}

/* FBM_INV — visual inverse of FBM. Dark regions where FBM is bright,
 * and vice versa. */
static float pattern_fbm_inv_at(float x, float y, float t)
{
    return 1.0f - pattern_fbm_at(x, y, t);
}

/* ── Tier-5 — textures + exotic patterns ────────────────────────── */

/* MARBLE — sin(x·k + turbulence·a). The CANONICAL Perlin texture:
 * Perlin 1985 showed marble could be synthesised as a sine wave
 * warped by turbulence. Vertical-stripe marble veins emerge. */
static float pattern_marble_at(float x, float y, float t)
{
    float turb = turbulence_octaves(x, y, t, FBM_OCTAVES,
                                     MULTIFRAC_LACUNARITY, MULTIFRAC_GAIN);
    return 0.5f + 0.5f * sinf(x * MARBLE_FREQ + turb * MARBLE_AMP);
}

/* WOOD — fract(radius·k + turbulence·a). Concentric grain rings
 * around the centre, warped by turbulence. Perlin 1985 wood-grain
 * texture in 2-D. */
static float pattern_wood_at(float x, float y, float t,
                              float cx, float cy)
{
    float dx = x - cx, dy = y - cy;
    float r  = sqrtf(dx * dx + dy * dy);
    float turb = turbulence_octaves(x, y, t, FBM_OCTAVES,
                                     MULTIFRAC_LACUNARITY, MULTIFRAC_GAIN);
    return fract(r * WOOD_FREQ + turb * WOOD_AMP);
}

/* FIRE — turbulence shaped by a vertical mask: hot at the bottom of
 * the screen, cooler toward the top. The drift multiplier makes fire
 * flicker faster than the default field drift. */
static float pattern_fire_at(float x, float y, float t, int h)
{
    float turb = turbulence_octaves(x, y, t * FIRE_DRIFT_MULT, FBM_OCTAVES,
                                     MULTIFRAC_LACUNARITY, MULTIFRAC_GAIN);
    float mask = 1.0f - (float)y / (float)h;   /* hot at bottom */
    return clampf(turb * 2.0f * mask, 0.0f, 1.0f);
}

/* CLOUDS — smoothstep(fbm) at clouds-thresholds. Hard transition
 * between cloud and clear-sky, but with a soft edge (smoothstep). */
static float pattern_clouds_at(float x, float y, float t)
{
    float n = fbm_octaves(x, y, t, FBM_OCTAVES,
                           MULTIFRAC_LACUNARITY, MULTIFRAC_GAIN);
    return smoothstep(CLOUDS_THRESH_LO, CLOUDS_THRESH_HI, n);
}

/* CAVES — inverse-smoothstep(fbm). Dense fbm regions become OPEN
 * caves; sparse regions become solid rock. Topology inverse of CLOUDS. */
static float pattern_caves_at(float x, float y, float t)
{
    float n = fbm_octaves(x, y, t, FBM_OCTAVES,
                           MULTIFRAC_LACUNARITY, MULTIFRAC_GAIN);
    return 1.0f - smoothstep(CAVES_THRESH_LO, CAVES_THRESH_HI, n);
}

/* STARS — high-frequency Perlin → ridge → high threshold. Only the
 * sharpest peaks survive, producing a sparse stippling that reads
 * as starfield against the dark background. */
static float pattern_stars_at(float x, float y, float t)
{
    float n = perlin2d(x * NOISE_SCALE * STARS_FREQ_MULT,
                       y * NOISE_SCALE * STARS_FREQ_MULT + t);
    float r = ridge_shape(n);
    return r > STARS_THRESHOLD ? r : 0.0f;
}

/* CHAOS — combine three noise samples at different scales, with a
 * warp and a periodic modulation, into a single complex composite.
 * No two locations look the same; maximum visual complexity. */
static float pattern_chaos_at(float x, float y, float t)
{
    float n1 = perlin2d(x * NOISE_SCALE,
                        y * NOISE_SCALE + t);
    float n2 = perlin2d(x * NOISE_SCALE * 2.0f,
                        y * NOISE_SCALE * 2.0f - t);
    float n3 = perlin2d((x + n1 * CHAOS_WARP) * NOISE_SCALE,
                        (y + n2 * CHAOS_WARP) * NOISE_SCALE + t * 1.5f);
    float result = sinf(n1 * 4.0f) * cosf(n2 * 4.0f) + n3;
    return clampf(result * 0.4f + 0.5f, 0.0f, 1.0f);
}

/* ===================================================================== */
/* §6  scene — Grid, Particles, RenderBuffers, SimState, Controls         */
/* ===================================================================== */

/*
 * The scene is built out of six small sub-structs (NoiseField from
 * §5 plus five defined below). Each owns ONE concern, and Scene
 * composes them. Splitting concerns into clearly-typed sub-structs
 * makes function signatures self-describing: any function that takes
 * `const Grid *` clearly cannot mutate buffers; any function that
 * takes `RenderBuffers *` clearly does not advance noise time; and
 * so on. Same architectural shape as the other field files
 * (magnetic_fields.c, flow_field_particles.c, midpoint_displacement_*).
 */

/*
 * Particle — one flowing dust grain. Massless walker advected by the
 * active pattern's vector field.
 *
 * INTENT. Position is integrated directly from the sampled velocity
 * vector each tick, with NO momentum term — this is the standard
 * streamline-tracing integration. The particle aligns instantaneously
 * with whatever the field says at its current cell. Trail history
 * isn't stored per-particle; it lives in RenderBuffers.glow, fading
 * exponentially via decay_trail_glow.
 *
 * FINITE LIFETIME. Random max_age ∈ [AGE_MIN_TICKS, AGE_MAX_TICKS] so
 * particles "tire out" at different times — without this, streamline
 * coverage gets dominated by long-lived particles trapped in slow
 * regions, and the field's overall topology stops reading.
 *
 * REFERENCE. Streamline-tracing integration as used here is the
 * 1-D-along-velocity form of forward-Euler advection — see Helman &
 * Hesselink (1991) "Representation and Display of Vector Field
 * Topology in Fluid Flows" for the broader framework.
 */
typedef struct {
    /* Continuous position in cell units. The renderer truncates to
     * int when painting the trail; the float value preserves sub-cell
     * accuracy across many sub-pixel-per-tick steps. */
    float x, y;

    /* 0..N_BANDS-1 — the particle's "tribe" (one of the 4 theme
     * palette colours), assigned at spawn via rand() & 3. Visually
     * distinct tribes let the viewer see how streams converge/diverge
     * even where their paths cross. */
    int   color_idx;

    /* Ticks since spawn. Compared against max_age each tick. */
    int   age;

    /* Random lifetime cap; respawn once age == max_age (or particle
     * walks out of bounds). Randomising means streamline coverage
     * is continuously refreshed at unpredictable points. */
    int   max_age;
} Particle;

/*
 * Grid — canvas geometry. Pure data: no buffers, no state. Lives at
 * the top of Scene because every layer (rasteriser, particle step,
 * screen centring) needs the dimensions.
 *
 * INDEXING. Row-major: cell (x, y) → y·w + x. Matches the memory
 * layout of RenderBuffers, so the y-outer / x-inner loop order in
 * rasterise_pattern_field is cache-friendly.
 *
 * INVARIANT. w · h ≤ CELLS_MAX always holds; app_pick_map_size()
 * clamps to MAP_W_MAX × MAP_H_MAX = 200 × 56 = 11 200 cells, which is
 * what the RenderBuffers arrays are statically sized for.
 */
typedef struct {
    int w, h;            /* current map width / height in cells */
    int total_cells;     /* = w · h, cached so hot loops skip the multiply */
} Grid;

static inline int  grid_idx       (const Grid *g, int x, int y) { return y * g->w + x; }
static inline bool grid_in_bounds (const Grid *g, int x, int y)
{
    return x >= 0 && x < g->w && y >= 0 && y < g->h;
}

/*
 * RenderBuffers — per-cell raster output. Two parallel arrays indexed
 * by grid_idx(g, x, y):
 *   glow  : trail intensity 0..1 — drives the ASCII glyph ramp
 *           (>0.65 = '#', >0.30 = '*', >0.05 = '.')
 *   color : palette band 0..3 (mod N_BANDS at draw time)
 *
 * The screen layer reads ONLY these two arrays. particle_step writes
 * via grid_idx(...) → buf->glow[idx] = 1.0f, and rasterise_pattern_field
 * overwrites them en bloc for static-field patterns. That's the entire
 * render contract between the simulation (writer) and the display
 * (reader). Decoupling render output from simulation state is the
 * classic graphics-pipeline boundary — the sim becomes display-agnostic.
 *
 * SoA RATIONALE. Two separate arrays (struct-of-arrays) rather than
 * one array of cell-structs because decay_trail_glow touches only
 * glow; storing color separately avoids reading bytes we don't need.
 */
typedef struct {
    /* Per-cell intensity 0..1. particle_step deposits 1.0 at each
     * landing; decay_trail_glow multiplies by expf(-GLOW_DECAY · dt)
     * each tick to create the fading-trail effect. Static-field
     * patterns overwrite the entire buffer each frame. */
    float   glow [CELLS_MAX];

    /* Per-cell palette band. For particle modes, inherited from the
     * particle that last landed at the cell. For static-field patterns,
     * derived from the noise intensity at that cell. Masked with
     * (N_BANDS - 1) at draw time for defence-in-depth. */
    uint8_t color[CELLS_MAX];
} RenderBuffers;

/*
 * Particles — fixed-size pool of dust grains. Standard pool-allocator
 * pattern: a max-sized array plus an active count `n`, so spawn /
 * respawn never touches the heap. Only pool[0..n-1] is alive.
 *
 * SIZING. MAX_PARTICLES (1024) is the static upper bound;
 * N_PARTICLES_DEF (256) is what scene_reset() activates. 256 is
 * enough to populate visible streamlines for all 4 particle-mode
 * patterns without saturating the screen.
 */
typedef struct {
    Particle pool[MAX_PARTICLES];
    int      n;                   /* active count, 0..MAX_PARTICLES */
} Particles;

/*
 * SimState — the single mutable per-tick scalar. Separated from
 * Controls (keyboard-driven knobs) so it's obvious what scene_tick
 * mutates vs. what the user does.
 *
 * field_time threads into every noise-sampling pattern as the time
 * offset added to the noise y-coordinate. Advances by FIELD_DRIFT · dt
 * each tick. Sliding it slowly through "noise space" is what makes
 * the field visibly evolve rather than freeze.
 *
 * One scalar = one named struct. Tiny but worth its own type for
 * symmetry with the other field files.
 */
typedef struct {
    float field_time;     /* drift offset added to noise y-coord */
} SimState;

/*
 * Controls — user-facing knobs. Mutated only by app_handle_key(),
 * read by scene_tick (paused, speed, current_pattern) and screen_draw
 * (theme, pattern).
 *
 * INTENT. Same model-vs-user-intent split as the other field files:
 * the dependency is strictly one-way. app_handle_key writes Controls,
 * never SimState; scene_tick reads Controls but only writes SimState
 * + RenderBuffers + Particles. Keeps the code linear.
 *
 * NO prev_pattern. Switching patterns triggers scene_clear_field so
 * stale particle trails don't ghost into the new pattern's view; the
 * static-field patterns then overwrite the whole buffer on their
 * first tick anyway.
 */
typedef struct {
    bool    paused;
    int     speed;
    int     current_theme;
    Pattern current_pattern;
} Controls;

/*
 * Scene — the umbrella context. Reading this struct top-to-bottom
 * is meant to be the fastest way to understand the program:
 *   grid       → where things live      (geometry)
 *   buf        → what gets drawn        (render output)
 *   particles  → moving agents          (advection walkers)
 *   noise      → the algorithm source   (NoiseField — Perlin perm[])
 *   sim        → animation state        (field_time)
 *   ctrl       → user knobs             (pattern, speed, theme, …)
 *
 * ORDERING. Each sub-struct depends only on those declared above it:
 * grid is leaf-level; buf is sized by CELLS_MAX; particles need grid
 * bounds; noise is independent; sim mutates noise input; ctrl decides
 * which sim path runs. A reader scanning top-down meets every concept
 * before it is used.
 */
typedef struct {
    Grid          grid;       /* canvas dimensions                          */
    RenderBuffers buf;        /* per-cell glow + colour band                */
    Particles     particles;  /* fixed pool of walkers                      */
    NoiseField    noise;      /* Perlin permutation — the algorithm source  */
    SimState      sim;        /* field_time — mutated only by scene_tick    */
    Controls      ctrl;       /* mutated only by app_handle_key             */
} Scene;

/* glow_to_col — uniform mapping of glow ∈ [0,1] to a palette band
 * 0..N_BANDS-1. Multiplies by GLOW_COL_SCALE = 4 − ε so the int cast
 * floors cleanly to [0, N_BANDS-1] even when glow == 1.0. Used by
 * every static-field pattern that doesn't have band-specific logic
 * (all of them except CONTOUR, BINARY, STARS). */
static inline int glow_to_col(float g)
{
    return (int)(g * GLOW_COL_SCALE) & (N_BANDS - 1);
}

/* ── particle pipeline ───────────────────────────────────────────── *
 * One named atomic per particle stage: spawn, velocity, integrate,
 * paint, age, respawn-test. The driver functions below compose these. */

static void particle_spawn(Particle *p, const Grid *g)
{
    p->x         = (float)(rand() % g->w);
    p->y         = (float)(rand() % g->h);
    p->color_idx = rand() & (N_BANDS - 1);
    p->age       = 0;
    p->max_age   = AGE_MIN_TICKS + rand() % (AGE_MAX_TICKS - AGE_MIN_TICKS);
}

/* is_particle_pattern — true for the 4 patterns driven by particles. */
static bool is_particle_pattern(Pattern p)
{
    return p == PATTERN_FLOW   || p == PATTERN_VORTEX
        || p == PATTERN_CURL   || p == PATTERN_JITTER;
}

/* ── small particle-mode primitives ─────────────────────────────── *
 * Each one is exactly the named operation the algorithm refers to —
 * "sample the noise at the particle", "convert an angle to velocity",
 * etc. — so the dispatcher body reads as pure pseudocode. */

/* sample_noise_at_particle — sample raw Perlin at the particle's
 * NOISE_SCALE-scaled position, offset in y by the drifting field_time. */
static inline float sample_noise_at_particle(const Particle *p, float field_time)
{
    return perlin2d(p->x * NOISE_SCALE,
                    p->y * NOISE_SCALE + field_time);
}

/* velocity_from_angle — write (vx, vy) = speed·(cos θ, sin θ) into
 * the out pointers. The "angle → velocity" half of FLOW/VORTEX/JITTER. */
static inline void velocity_from_angle(float angle, int speed,
                                        float *out_vx, float *out_vy)
{
    *out_vx = cosf(angle) * (float)speed;
    *out_vy = sinf(angle) * (float)speed;
}

/* random_jitter_angle — uniform sample in [-range/2, +range/2].
 * JITTER pattern adds this to the FLOW angle for noisy streamlines. */
static inline float random_jitter_angle(float range)
{
    return ((float)rand() / (float)RAND_MAX - 0.5f) * range;
}

/* normalise_2d — in-place unit-length normalisation with epsilon
 * guard. CURL needs this because curl-noise magnitude varies wildly
 * across cells, but we want a uniform visual particle speed. */
static inline void normalise_2d(float *vx, float *vy)
{
    float m = sqrtf((*vx) * (*vx) + (*vy) * (*vy));
    if (m > VECTOR_EPSILON) { *vx /= m; *vy /= m; }
}

/* particle_velocity_at — per-particle-mode dispatcher. Returns the
 * (vx, vy) for the particle's current position. Each branch is one
 * named pattern in the algorithm vocabulary:
 *   FLOW   : angle = noise·π                      (the original demo)
 *   VORTEX : angle = noise·π + π/2                (tangent — swirls)
 *   CURL   : divergence-free curl noise           (Bridson 2007)
 *   JITTER : FLOW angle + uniform random jitter   (chaotic streamlines)
 */
static void particle_velocity_at(Pattern pat, const Particle *p,
                                  float field_time, int speed,
                                  float *out_vx, float *out_vy)
{
    switch (pat) {
    case PATTERN_FLOW: {
        float angle = sample_noise_at_particle(p, field_time) * (float)M_PI;
        velocity_from_angle(angle, speed, out_vx, out_vy);
        return;
    }
    case PATTERN_VORTEX: {
        float angle = sample_noise_at_particle(p, field_time) * (float)M_PI
                    + VORTEX_TURN_RADIANS;
        velocity_from_angle(angle, speed, out_vx, out_vy);
        return;
    }
    case PATTERN_CURL: {
        float vx, vy;
        curl_noise_2d(p->x, p->y, field_time, &vx, &vy);
        normalise_2d(&vx, &vy);
        *out_vx = vx * (float)speed;
        *out_vy = vy * (float)speed;
        return;
    }
    case PATTERN_JITTER: {
        float angle = sample_noise_at_particle(p, field_time) * (float)M_PI
                    + random_jitter_angle(JITTER_ANGLE_RANGE);
        velocity_from_angle(angle, speed, out_vx, out_vy);
        return;
    }
    default:
        *out_vx = 0.0f;
        *out_vy = 0.0f;
        return;
    }
}

/* advect_particle_euler — forward-Euler integration step. Massless
 * advection — no inertia term. */
static void advect_particle_euler(Particle *p, float vx, float vy, float dt)
{
    p->x += vx * dt;
    p->y += vy * dt;
    p->age++;
}

/* deposit_trail_hit — paint the particle's current cell at full
 * intensity. Overwrites (not blends); the trail fade comes from
 * decay_trail_glow between hits. */
static void deposit_trail_hit(RenderBuffers *buf, const Grid *g,
                                int cx, int cy, int color_idx)
{
    if (!grid_in_bounds(g, cx, cy)) return;
    int idx = grid_idx(g, cx, cy);
    buf->glow [idx] = 1.0f;
    buf->color[idx] = (uint8_t)color_idx;
}

/* particle_is_expired — has the walker overstayed its life or walked
 * off the grid? Either triggers a respawn. */
static bool particle_is_expired(const Particle *p, const Grid *g)
{
    return p->age >= p->max_age
        || p->x < 0.0f || p->x >= (float)g->w
        || p->y < 0.0f || p->y >= (float)g->h;
}

/*
 * particle_step — per-particle pseudocode:
 *
 *     sample velocity from the active pattern's vector field
 *     advect by velocity · dt
 *     deposit trail hit at the new cell
 *     respawn if expired (age or OOB)
 *
 * Each step is one named helper above. */
static void particle_step(RenderBuffers *buf, const Grid *g,
                          Particle *p, Pattern pat,
                          float field_time, float dt, int speed)
{
    float vx, vy;
    particle_velocity_at(pat, p, field_time, speed, &vx, &vy);
    advect_particle_euler(p, vx, vy, dt);
    deposit_trail_hit    (buf, g, (int)p->x, (int)p->y, p->color_idx);
    if (particle_is_expired(p, g))
        particle_spawn   (p, g);
}

/*
 * rasterise_pattern_field — for non-particle patterns, recompute glow
 * and colour at every cell using the active pattern's evaluator.
 * Called once per scene_tick. No decay needed because we fully
 * overwrite each frame. The switch is one line per pattern — a pure
 * pseudocode dispatcher.
 */
static void rasterise_pattern_field(RenderBuffers *buf, const Grid *g,
                                     Pattern p, float t)
{
    float cx = (float)g->w * 0.5f;
    float cy = (float)g->h * 0.5f;
    int   h  = g->h;

    for (int y = 0; y < g->h; y++) {
        for (int x = 0; x < g->w; x++) {
            int   idx  = grid_idx(g, x, y);
            float fx   = (float)x, fy = (float)y;
            float glow = 0.0f;
            int   col  = 0;

            switch (p) {

            /* Tier 1 — direct visualisations */
            case PATTERN_HEIGHT:    glow = pattern_height_at  (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_VALLEYS:   glow = pattern_valleys_at (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_BANDED:    glow = pattern_banded_at  (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_BINARY:    glow = pattern_binary_at  (fx, fy, t); col = glow > 0.5f ? 3 : 0; break;

            /* Tier 2 — shape transforms */
            case PATTERN_RIDGE:     glow = pattern_ridge_at   (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_BILLOW:    glow = pattern_billow_at  (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_ZEBRA:     glow = pattern_zebra_at   (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_RINGS:     glow = pattern_rings_at   (fx, fy, t, cx, cy); col = glow_to_col(glow); break;
            case PATTERN_PLASMA:    glow = pattern_plasma_at  (fx, fy, t); col = glow_to_col(glow); break;

            /* Tier 3 — warps + slope */
            case PATTERN_CONTOUR: {
                int band;
                glow = pattern_contour_at(fx, fy, t, &band);
                col  = band & 3;
                break;
            }
            case PATTERN_WARP:      glow = pattern_warp_at      (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_WARP_DEEP: glow = pattern_warp_deep_at (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_WARPRIDGE: glow = pattern_warpridge_at (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_SLOPE:     glow = pattern_slope_at     (fx, fy, t); col = glow_to_col(glow); break;

            /* Tier 4 — fractal compositions */
            case PATTERN_FBM:       glow = pattern_fbm_at       (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_FBM_HIGH:  glow = pattern_fbm_high_at  (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_RIDGED:    glow = pattern_ridged_at    (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_TURBLENC:  glow = pattern_turblenc_at  (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_FBM_INV:   glow = pattern_fbm_inv_at   (fx, fy, t); col = glow_to_col(glow); break;

            /* Tier 5 — textures + exotic statics */
            case PATTERN_MARBLE:    glow = pattern_marble_at    (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_WOOD:      glow = pattern_wood_at      (fx, fy, t, cx, cy); col = glow_to_col(glow); break;
            case PATTERN_FIRE:      glow = pattern_fire_at      (fx, fy, t, h); col = glow_to_col(glow); break;
            case PATTERN_CLOUDS:    glow = pattern_clouds_at    (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_CAVES:     glow = pattern_caves_at     (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_STARS:     glow = pattern_stars_at     (fx, fy, t); col = 3; break;
            case PATTERN_CHAOS:     glow = pattern_chaos_at     (fx, fy, t); col = glow_to_col(glow); break;

            default:
                continue;       /* particle-mode patterns handled by scene_tick */
            }

            buf->glow [idx] = glow;
            buf->color[idx] = (uint8_t)(col & 3);
        }
    }
}

/* ── tick + reset pipeline ───────────────────────────────────────── *
 * scene_tick is the per-frame pseudocode:
 *
 *     if paused: stop.
 *     advance noise time by FIELD_DRIFT · dt.
 *     if particle pattern: decay trails, step every particle.
 *     else:                rasterise the noise-derived field.
 *
 * scene_reset is the init/reset pseudocode:
 *
 *     apply grid dimensions.
 *     reset sim state.
 *     clear render buffers.
 *     install a fresh shuffled noise field.
 *     spawn all particles.
 *
 * Each step is one named call below. */

static void apply_grid_dimensions(Grid *g, int w, int h)
{
    g->w           = w;
    g->h           = h;
    g->total_cells = w * h;
}

static void reset_sim_state(SimState *sim)
{
    sim->field_time = 0.0f;
}

static void buffers_clear(RenderBuffers *buf, int n)
{
    for (int i = 0; i < n; i++) {
        buf->glow [i] = 0.0f;
        buf->color[i] = 0;
    }
}

static void decay_trail_glow(RenderBuffers *buf, int n, float dt)
{
    float decay = expf(-GLOW_DECAY * dt);
    for (int i = 0; i < n; i++) buf->glow[i] *= decay;
}

static void advance_noise_time(SimState *sim, float dt)
{
    sim->field_time += FIELD_DRIFT * dt;
}

static void install_fresh_noise(NoiseField *nf)
{
    shuffle_noise_field(nf);   /* shuffle + activate as current source */
}

static void spawn_all_particles(Particles *ps, const Grid *g)
{
    ps->n = N_PARTICLES_DEF;
    for (int i = 0; i < ps->n; i++)
        particle_spawn(&ps->pool[i], g);
}

static void step_all_particles(Scene *s, float dt)
{
    int spd = s->ctrl.speed;
    for (int i = 0; i < s->particles.n; i++)
        particle_step(&s->buf, &s->grid,
                       &s->particles.pool[i],
                       s->ctrl.current_pattern,
                       s->sim.field_time, dt, spd);
}

/* scene_clear_field — wipe the render buffer. Called from
 * cycle_pattern so leftover particle trails from the previous pattern
 * don't ghost into the new one. */
static void scene_clear_field(Scene *s)
{
    buffers_clear(&s->buf, s->grid.total_cells);
}

static void scene_reset(Scene *s, int w, int h)
{
    apply_grid_dimensions  (&s->grid, w, h);
    reset_sim_state        (&s->sim);
    buffers_clear          (&s->buf, s->grid.total_cells);
    install_fresh_noise    (&s->noise);
    spawn_all_particles    (&s->particles, &s->grid);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->ctrl.paused           = false;
    s->ctrl.speed            = SPEED_DEF;
    s->ctrl.current_theme    = 0;
    s->ctrl.current_pattern  = PATTERN_FLOW;
    scene_reset(s, w, h);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;

    advance_noise_time(&s->sim, dt);

    if (is_particle_pattern(s->ctrl.current_pattern)) {
        decay_trail_glow  (&s->buf, s->grid.total_cells, dt);
        step_all_particles(s, dt);
    } else {
        rasterise_pattern_field(&s->buf, &s->grid,
                                 s->ctrl.current_pattern,
                                 s->sim.field_time);
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — terminal dimensions cached after the last successful
 * getmaxyx(). Refreshed on SIGWINCH via screen_resize().
 *
 * INTENT. Kept tiny because the rest of ncurses' state lives
 * implicitly in stdscr; we only need the dimensions to centre the
 * map and position HUD elements. Anything else (current attribute,
 * cursor position, colour-pair table) is owned by ncurses internals,
 * not by us — exposing it here would just duplicate state.
 *
 * NOT IN SCENE. Screen lives at App-level, not Scene, because Scene
 * is "the simulation" and the simulation is display-agnostic: any
 * scene state should be reusable on a different display backend. The
 * terminal-cell dimensions are a property of the display, not the sim.
 */
typedef struct {
    int cols;   /* terminal width  in character cells */
    int rows;   /* terminal height in character cells */
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
 * CellDraw — bundle of "what to paint at this position": pair, attr,
 * glyph, plus a skip flag. classify_trail_cell decides; paint_cell
 * acts. Routing the per-cell decision through this struct keeps the
 * decision logic pure (no ncurses I/O inside the classifier) and
 * concentrates ncurses calls into ONE place (paint_cell).
 */
typedef struct {
    int  pair;
    int  attr;
    char glyph;
    bool skip;
} CellDraw;

/* classify_trail_cell — pure function. Given a cell's glow and colour
 * band, return how it should be rendered (or .skip = true if below
 * the visibility floor). Three density tiers map to the three glyphs
 * of the Bourke ASCII ramp. */
static CellDraw classify_trail_cell(float glow, uint8_t color)
{
    int pair = PAIR_TRAIL_BASE + (color & (N_BANDS - 1));
    if (glow > GLYPH_HIGH_THRESH) return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = '#' };
    if (glow > GLYPH_MID_THRESH ) return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = '*' };
    if (glow > GLOW_THRESHOLD   ) return (CellDraw){ .pair = pair, .attr = A_NORMAL, .glyph = '.' };
    return (CellDraw){ .skip = true };
}

/* paint_cell — the ONE ncurses I/O point for the trail layer. */
static void paint_cell(int sy, int sx, CellDraw c)
{
    if (c.skip) return;
    attron (COLOR_PAIR(c.pair) | c.attr);
    mvaddch(sy, sx, (chtype)(unsigned char)c.glyph);
    attroff(COLOR_PAIR(c.pair) | c.attr);
}

/* compute_centred_origin — top-left corner where the map is drawn.
 * Centres horizontally; reserves HUD_BAND_RESERVED_ROWS rows total
 * (HUD_TOP_ROWS top + HUD_BOTTOM_ROWS bottom). */
static void compute_centred_origin(const Grid *g, int cols, int rows,
                                    int *out_gx0, int *out_gy0)
{
    int gx0 = (cols - g->w) / 2;
    int gy0 = ((rows - HUD_BAND_RESERVED_ROWS) - g->h) / 2 + HUD_TOP_ROWS;
    if (gx0 < 0)            gx0 = 0;
    if (gy0 < HUD_TOP_ROWS) gy0 = HUD_TOP_ROWS;
    *out_gx0 = gx0;
    *out_gy0 = gy0;
}

/*
 * scene_draw — walk every cell, classify it, paint it. The loop body
 * is exactly three named operations.
 */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const Grid          *g   = &s->grid;
    const RenderBuffers *buf = &s->buf;
    int gx0, gy0;
    compute_centred_origin(g, cols, rows, &gx0, &gy0);

    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            int idx = grid_idx(g, x, y);
            paint_cell(sy, sx,
                        classify_trail_cell(buf->glow[idx], buf->color[idx]));
        }
    }
}

/* ── HUD draw pipeline ───────────────────────────────────────────── *
 * Five named drawers, called from screen_draw in z-order. The TOP HUD
 * (rows 0..1) carries DATA — current state with [N/30] index, parameter
 * readouts, palette swatch. The BOTTOM HUD (row N-1) carries ACTIONS
 * — key bindings only.
 *
 * draw_hud_status_line internally composes several smaller segment
 * drawers, one per fixed-width row-1 field. */

static void draw_hud_state_bar(const Screen *sc, const Scene *s,
                                double fps, int sim_fps)
{
    const Controls *c = &s->ctrl;
    const char *state_str = c->paused ? "PAUSED   "
                                      : pattern_name(c->current_pattern);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  speed:%-3d ",
             fps, sim_fps, state_str,
             (int)c->current_pattern + 1, N_PATTERNS,
             c->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void draw_hud_title(void)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " PERLIN-NOISE FLOW ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* ── row 1 segment drawers ───────────────────────────────────────── *
 * draw_hud_status_line lays out row 1 left-to-right as a sequence of
 * fixed-width segments. Each segment paints its content and returns
 * the new x-cursor; the last (sim_counts) does not need to return. */

static int draw_status_pattern_field(int row, int x, Pattern p)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " pattern:%-9s ", pattern_name(p));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_PATTERN_FIELD_W;
}

static int draw_status_theme_field(int row, int x, int theme_idx)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " theme:%-8s ", themes[theme_idx].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_THEME_FIELD_W;
}

static int draw_status_palette_label(int row, int x)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    return x + HUD_PALETTE_LABEL_W;
}

static int draw_palette_swatch(int row, int x)
{
    for (int i = 0; i < N_BANDS; i++) {
        int pair = PAIR_TRAIL_BASE + i;
        attron (COLOR_PAIR(pair) | A_BOLD);
        mvaddch(row, x, '#');
        attroff(COLOR_PAIR(pair) | A_BOLD);
        x++;
    }
    return x;
}

/* draw_status_sim_counts — last segment: noise scale + drift + map
 * dimensions. No return — end of the row. */
static void draw_status_sim_counts(int row, int x, const Grid *g)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  scale:%.2f  drift:%.2f  map:%dx%d ",
             NOISE_SCALE, FIELD_DRIFT, g->w, g->h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

static void draw_hud_status_line(const Scene *s)
{
    const Controls *c = &s->ctrl;
    int x = HUD_LEFT_MARGIN;
    x = draw_status_pattern_field (1, x, c->current_pattern);
    x = draw_status_theme_field   (1, x, c->current_theme);
    x = draw_status_palette_label (1, x);
    x = draw_palette_swatch       (1, x);
        draw_status_sim_counts    (1, x, &s->grid);
}

/* draw_bottom_hint — row N-1: ACTIONS only (key bindings). */
static void draw_bottom_hint(const Screen *sc)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  r:reset  spc:pause  +/-:speed  ]/[:Hz  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw           (s, sc->cols, sc->rows);
    draw_hud_state_bar   (sc, s, fps, sim_fps);   /* row 0  : data    */
    draw_hud_title       ();                       /* row 0  : data    */
    draw_hud_status_line (s);                      /* row 1  : data    */
    draw_bottom_hint     (sc);                     /* row N-1: actions */
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level program state. One instance, g_app, lives in BSS
 * so the signal handlers can reach it without a global Scene pointer.
 * The App owns the Scene and the Screen and adds:
 *   • simulation parameters that are not really "scene state"
 *     (sim_fps, map_w, map_h);
 *   • signal-driven flags that must be sig_atomic_t for safety.
 *
 * SIGNAL-HANDLER DISCIPLINE. The handlers do nothing but set a flag.
 * The main loop polls those flags and performs the actual work
 * (cleanup, resize) in normal execution context. Standard async-
 * signal-safe pattern — anything that touches ncurses or malloc MUST
 * happen outside the handler.
 *
 * QUALIFIER NOTE. The running / need_resize flags carry BOTH
 * `volatile` and `sig_atomic_t`. sig_atomic_t guarantees writes from
 * a handler are observed atomically by the main loop; volatile
 * prevents the compiler from caching the read in a register across
 * loop iterations. Both qualifiers are required — sig_atomic_t alone
 * permits caching, volatile alone permits torn writes from a handler.
 *
 * REFERENCE. W. Richard Stevens & Stephen Rago — "Advanced Programming
 * in the UNIX Environment" (3rd ed), ch. 10 on signals, for the full
 * discussion of async-signal-safety and sig_atomic_t.
 */
typedef struct {
    Scene                 scene;     /* the simulation                       */
    Screen                screen;    /* current terminal dimensions          */

    int                   sim_fps;   /* tick rate; mutated by '[' and ']'    */
    int                   map_w;     /* chosen map width,  ≤ MAP_W_MAX       */
    int                   map_h;     /* chosen map height, ≤ MAP_H_MAX       */

    volatile sig_atomic_t running;       /* 0 = exit main loop               */
    volatile sig_atomic_t need_resize;   /* 1 = pending SIGWINCH             */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS;
    if (mw < 16) mw = 16;
    if (mh < 8)  mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* ── keyboard handlers ──────────────────────────────────────────────── *
 * Each key is one named action; app_handle_key is just a dispatcher. */

/* bump_speed_geometric — '+' / '-' geometric step on particle speed. */
static void bump_speed_geometric(Controls *c, int dir)
{
    if (dir > 0) {
        if (c->speed < SPEED_MAX) c->speed *= 2;
        if (c->speed > SPEED_MAX) c->speed = SPEED_MAX;
    } else {
        c->speed /= 2;
        if (c->speed < SPEED_MIN) c->speed = SPEED_MIN;
    }
}

/* bump_sim_fps — '[' / ']' linear step on tick rate, clamped. */
static void bump_sim_fps(App *app, int delta)
{
    app->sim_fps += delta;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

static void cycle_theme(Controls *c, int dir)
{
    c->current_theme = (c->current_theme + dir + N_THEMES) % N_THEMES;
    theme_apply(c->current_theme);
}

/* cycle_pattern — n/p step + clear the render buffer. We clear because
 * leftover particle trails would ghost into a static-field pattern's
 * first frame before it overwrites the buffer. */
static void cycle_pattern(App *app, int dir)
{
    Controls *c = &app->scene.ctrl;
    c->current_pattern = (Pattern)(
        ((int)c->current_pattern + dir + N_PATTERNS) % N_PATTERNS);
    scene_clear_field(&app->scene);
}

static bool app_handle_key(App *app, int ch)
{
    Controls *c = &app->scene.ctrl;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           c->paused = !c->paused;                              break;
    case 'r': case 'R': scene_reset(&app->scene, app->map_w, app->map_h);    break;
    case '=': case '+': bump_speed_geometric(c,   +1);                       break;
    case '-':           bump_speed_geometric(c,   -1);                       break;
    case ']':           bump_sim_fps        (app, +SIM_FPS_STEP);            break;
    case '[':           bump_sim_fps        (app, -SIM_FPS_STEP);            break;
    case 't':           cycle_theme         (c,   +1);                       break;
    case 'T':           cycle_theme         (c,   -1);                       break;
    case 'n': case 'N': cycle_pattern       (app, +1);                       break;
    case 'p': case 'P': cycle_pattern       (app, -1);                       break;
    default: break;
    }
    return true;
}

/* ── main-loop helpers ──────────────────────────────────────────────── */

static void install_signal_handlers(void)
{
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* advance_frame_clock — read the monotonic clock, compute dt since
 * the last call, clamp at the spiral-of-death guard. */
static int64_t advance_frame_clock(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > DT_MAX_NS) dt = DT_MAX_NS;
    return dt;
}

/* simulate_pending_ticks — drain the fixed-timestep accumulator.
 * Source: Glenn Fiedler, "Fix Your Timestep". */
static void simulate_pending_ticks(App *app, int64_t *sim_accum,
                                    int64_t tick_ns, float dt_sec)
{
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* maybe_update_fps_counter — every FPS_UPDATE_MS, fold the running
 * frame count into a smoothed fps reading. */
static double maybe_update_fps_counter(int64_t *fps_accum,
                                        int *frame_count,
                                        double previous)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return previous;
    double fps = (double)(*frame_count) /
                  ((double)(*fps_accum) / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
    return fps;
}

/* cap_frame_rate — sleep so frames are at most 1/target_fps apart. */
static void cap_frame_rate(int64_t work_done_ns, int target_fps)
{
    int64_t budget = NS_PER_SEC / target_fps;
    clock_sleep_ns(budget - work_done_ns);
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    install_signal_handlers();

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init      (&app->screen);
    app_pick_map_size(app);
    scene_init       (&app->scene, app->map_w, app->map_h);

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

        int64_t dt      = advance_frame_clock(&frame_time);
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        simulate_pending_ticks(app, &sim_accum, tick_ns, dt_sec);

        frame_count++;
        fps_accum  += dt;
        fps_display = maybe_update_fps_counter(&fps_accum, &frame_count, fps_display);

        cap_frame_rate((clock_ns() - frame_time) + dt, FRAME_CAP_FPS);

        screen_draw   (&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
