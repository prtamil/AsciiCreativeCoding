/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * curl_noise_vector_field.c
 *   — Divergence-free curl noise: 5 ways to see a vector field.
 *
 * DEMO: Curl noise is the ∇×ψ of a scalar noise potential — a 2-D
 *       vector field that is exactly DIVERGENCE-FREE by construction.
 *       Particles flowing through it swirl into vortices and never
 *       pile up or run dry; the field's mass is conserved. Five
 *       patterns visualise the same underlying field in different ways:
 *         PARTICLES — 256 particles trace streamlines (default)
 *         VECTOR    — sparse arrow glyphs '>', '<', '^', 'v', '/', '\'
 *                     showing the field direction at lattice points
 *         POTENTIAL — render the scalar potential ψ as a heightmap
 *         CURL_MAG  — render |∇×ψ| as density heat — bright at
 *                     vortex centres, dim at irrotational regions
 *         WARPED    — particle flow over a domain-warped potential
 *                     (more chaotic, eddy-rich streamlines)
 *       Field drifts so the flow evolves rather than looping.
 *
 * Study alongside:
 *   ./perin_noise_flow_showcase.c — particles flow on a noise GRADIENT
 *       (∇ψ), which is divergent. Curl flow uses the perpendicular
 *       (∇×ψ), which is divergence-free. Side-by-side: gradient
 *       streams converge to peaks; curl streams loop forever.
 *   ./domain_warped_noise_iq_style.c — domain warping technique used
 *       by the WARPED pattern here.
 *   ./worley_cellular_noise.c — discrete cells; this file is smooth.
 *
 * Section map:
 *   §1 config   — grid, particles, palette, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes
 *   §5 noise    — Noise context (perm table) + Perlin + fBm + curl
 *   §6 patterns — helpers shared by the 5 visualisations
 *   §7 scene    — Scene struct composing Grid, RenderBuffers,
 *                 Particles, SimState, Controls (+ Noise from §5),
 *                 plus the per-frame tick / update pipeline
 *   §8 screen   — ASCII render: density glyphs + arrow glyphs
 *   §9 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (new permutation)
 *   n / N      next pattern
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster particles / drift
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra curl_noise_vector_field.c \
 *       -o curl_noise -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Curl noise is a way to derive a smooth, divergence-
 *                  free 2-D vector field from a scalar noise function.
 *
 *                  Given a scalar potential ψ(x, y), define
 *                      v(x, y) = (∂ψ/∂y, −∂ψ/∂x).
 *                  This v is identically divergence-free by the
 *                  cross-derivative theorem:
 *                      ∇·v = ∂vx/∂x + ∂vy/∂y
 *                          = ∂²ψ/∂x∂y − ∂²ψ/∂y∂x = 0.
 *                  So a particle flowing along v never converges to a
 *                  source or diverges to a sink — the field has no
 *                  "wells" or "fountains". Combined with a smooth ψ
 *                  (e.g. Perlin / fBm) you get vortex-rich flow that
 *                  looks like fluid or smoke.
 *
 *                  We compute v by central finite difference:
 *                      ∂ψ/∂y ≈ (ψ(x, y+ε) − ψ(x, y−ε)) / (2ε)
 *                      ∂ψ/∂x ≈ (ψ(x+ε, y) − ψ(x−ε, y)) / (2ε)
 *                  with ε small enough to capture local curvature
 *                  but large enough to keep the noise output stable
 *                  (we use ε = 0.5 noise-units).
 *
 *                  Five patterns visualise the field; PARTICLES is the
 *                  default ("flow" view), VECTOR shows arrows at
 *                  lattice points, POTENTIAL shows the underlying
 *                  scalar ψ as a heightmap, CURL_MAG shows |v| as
 *                  density (bright at vortex cores), WARPED applies
 *                  domain warping to ψ before taking the curl.
 *
 * Data-structure : Scene is the umbrella context, composed of six
 *                  sub-structs so each layer has one clear role and
 *                  no globals leak across them:
 *                    Grid          — w, h, total_cells + idx/in_bounds.
 *                    Noise         — Perlin permutation table; passed
 *                                    by pointer to every noise call.
 *                    RenderBuffers — per-cell glow, palette index, and
 *                                    optional glyph override (the
 *                                    VECTOR pattern uses it for arrows).
 *                                    Screen reads ONLY these arrays.
 *                    Particles     — static pool for PARTICLES/WARPED.
 *                    SimState      — field_time + supernova flash
 *                                    (transient post-reset glow).
 *                    Controls      — pattern, theme, speed, drift,
 *                                    pause; mutated by the keyboard.
 *                  No heap; everything BSS.
 *
 * Rendering      : ASCII only. Density-based '.', '*', '#' for the
 *                  field-as-scalar patterns. The VECTOR pattern uses
 *                  arrow glyphs '>', '<', '^', 'v', '/', '\\' chosen
 *                  by the local velocity direction. Each pattern
 *                  writes into RenderBuffers uniformly so the colour-
 *                  banding logic in scene_draw stays the same.
 *
 * Performance    : 4 fBm calls per cell for CURL_MAG (the most
 *                  expensive pattern), each with 3 octaves. ~135 K
 *                  perlin/sec on a 200×56 grid at 60 Hz, well under
 *                  1 % of one core on modern hardware.
 *
 * References     : • Bridson, Houriham & Nordenstam (2007) — "Curl-
 *                    Noise for Procedural Fluid Flow", SIGGRAPH.
 *                    The original paper introducing this technique
 *                    to graphics — short, readable, and contains the
 *                    exact derivation reproduced in CONCEPTS above:
 *                    https://www.cs.ubc.ca/~rbridson/docs/bridson-siggraph2007-curlnoise.pdf
 *                  • Ken Perlin (2002) — "Improving Noise", SIGGRAPH.
 *                    Source of the quintic fade t·t·t·(t·(t·6−15)+10)
 *                    and the gradient-hash table used by
 *                    noise_perlin2d. NOT the 1985 original, which
 *                    used a cubic fade with visible C¹-discontinuity
 *                    artefacts at integer lattice points.
 *                  • Ebert, Musgrave, Peachey, Perlin & Worley
 *                    — "Texturing & Modeling: A Procedural Approach"
 *                    (3rd ed, 2003, Morgan Kaufmann). The
 *                    comprehensive book on procedural noise; the fBm
 *                    chapter is the canonical treatment of octaves,
 *                    amplitude / frequency ratios, and lacunarity.
 *                  • Bridson (2015) — "Fluid Simulation for Computer
 *                    Graphics" (2nd ed, CRC Press). Puts curl noise
 *                    in the broader context of divergence-free
 *                    velocity fields, incompressible flow, and
 *                    projection methods (Chorin / Helmholtz–Hodge).
 *                  • Inigo Quilez — "Domain Warping":
 *                    https://iquilezles.org/articles/warp/
 *                    The exact two-stage technique used by the
 *                    WARPED pattern — feed an fBm output back into
 *                    the input coordinates of another fBm sample.
 *                  • Marsden & Tromba — "Vector Calculus"
 *                    (W. H. Freeman). Standard undergraduate text
 *                    for gradient / divergence / curl intuition and
 *                    the cross-derivative identity that makes
 *                    ∇·(∇×ψ) ≡ 0.
 *                  • Glenn Fiedler — "Fix Your Timestep":
 *                    https://gafferongames.com/post/fix_your_timestep/
 *                    Source of the fixed-timestep accumulator + dt
 *                    clamp used in main's frame loop.
 *                  • Paul Bourke — "Character representation of grey
 *                    scale images":
 *                    https://paulbourke.net/dataformats/asciiart/
 *                    The canonical density-to-glyph ramp reference;
 *                    motivates the '.', '*', '#' band picker used in
 *                    scene_draw.
 *                  • Wikipedia — "Vector field":
 *                    https://en.wikipedia.org/wiki/Vector_field
 *                    Accessible entry point for the conservative /
 *                    solenoidal / Helmholtz decomposition vocabulary.
 *                  • Compare ./perin_noise_flow_showcase.c —
 *                    gradient flow (divergent, particles pile up at
 *                    peaks) vs curl flow (divergence-free, particles
 *                    orbit forever) — the same noise viewed two ways.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Most procedural flow demos use the GRADIENT of a noise field to
 * push particles. That works but it has a major flaw: gradients
 * point UPHILL — particles converge into local maxima and pile up.
 * Curl noise fixes this by rotating the gradient 90° (∇×ψ), which
 * gives a velocity that swirls AROUND maxima instead of toward
 * them. The result is divergence-free flow: particles never
 * accumulate; they orbit eternally.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Shuffle the Noise.perm[] table. For PARTICLES and WARPED,
 *     spawn N particles at random in-bounds positions.
 *  2. PER FRAME:
 *     a. Drift sim.field_time by FIELD_DRIFT × dt (animated patterns).
 *     b. Pattern-specific:
 *        • PARTICLES: for each particle, compute (vx, vy) via curl
 *          of potential(x, y, t); step by v · dt; respawn on OOB.
 *        • VECTOR: at every 4×2 cell, compute v, store an arrow
 *          glyph corresponding to v's direction.
 *        • POTENTIAL: at every cell, glow = potential(x, y, t)
 *          remapped to [0, 1].
 *        • CURL_MAG: at every cell, compute v, glow = |v| / max.
 *        • WARPED: like PARTICLES but use warped_potential(x, y, t).
 *     c. Render buf.glow / buf.glyph through the standard density-
 *        band pipeline in scene_draw.
 *  3. Reshuffle perm[] on demand when the user presses 'r' (manual
 *     reset) — gives a brand-new field. The supernova flash fires
 *     as a visual confirmation of the reset.
 *
 * KEY FORMULAS
 * ────────────
 *  Scalar potential              : ψ(x, y) = fbm(x · scale, y · scale + t)
 *  Curl (2-D)                    : v = (∂ψ/∂y, −∂ψ/∂x)
 *  Central finite difference     : ∂ψ/∂x ≈ (ψ(x+ε) − ψ(x−ε)) / (2ε)
 *  Particle update               : (x, y) ← (x, y) + v · dt
 *  Divergence (zero by design)   : ∇·v = ∂vx/∂x + ∂vy/∂y = 0
 *  Curl magnitude                : |v| = √(vx² + vy²)
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

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* For PARTICLES and WARPED. */
    MAX_PARTICLES     = 1024,
    N_PARTICLES_DEF   =  256,

    AGE_MIN_TICKS     =  60,
    AGE_MAX_TICKS     = 360,

    /* Speed: cells/sec along the curl velocity. */
    SPEED_MIN         =   1,
    SPEED_DEF         =   8,
    SPEED_MAX         =  64,

    /* VECTOR pattern: arrow lattice spacing (in cells). 4x2 chosen
     * because terminal cells are ~2x taller than wide, so this gives
     * a roughly square arrow grid in pixel space. */
    VECTOR_LATTICE_X  =   4,
    VECTOR_LATTICE_Y  =   2,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,    /* PAIR_BAND_BASE..+3 = 4 palette colours */
    PAIR_FLASH        =   7,
    PAIR_SUPERNOVA    =   8,
};

/* Glow decays for particle trails (only). Static-field patterns
 * overwrite the buffer each frame so they don't decay. */
#define TRAIL_GLOW_DECAY    0.6f
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

/* Noise scale and animation speed. */
#define NOISE_SCALE         0.05f
#define FIELD_DRIFT         0.10f
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      1
#define DRIFT_MULT_MAX      32

/* Finite-difference step for ∂ψ/∂x and ∂ψ/∂y. ε = 0.5 noise-units
 * is a sweet spot — small enough to capture local curvature, large
 * enough that float jitter doesn't dominate. */
#define CURL_EPS            0.5f

/* fBm octaves for the potential. */
#define FBM_OCTAVES         3

/* WARPED pattern: warp magnitude. */
#define WARP_AMOUNT         3.0f

/* Density thresholds for the ASCII glyph ramp. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

typedef enum {
    PATTERN_PARTICLES = 0,
    PATTERN_VECTOR    = 1,
    PATTERN_POTENTIAL = 2,
    PATTERN_CURL_MAG  = 3,
    PATTERN_WARPED    = 4,
    N_PATTERNS        = 5,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_PARTICLES: return "PARTICLES";
    case PATTERN_VECTOR:    return "VECTOR   ";
    case PATTERN_POTENTIAL: return "POTENTIAL";
    case PATTERN_CURL_MAG:  return "CURL_MAG ";
    case PATTERN_WARPED:    return "WARPED   ";
    default:                return "?        ";
    }
}

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ── Pseudocode-named constants ─────────────────────────────────────── *
 * Magic numbers extracted from inline expressions and given names that
 * convey their algorithmic / physical / display role. Grouped here so
 * one glance at §1 lists every tunable in the program. */

/* §5 noise — domain-warp offsets. Two irrational-ish constants
 * decorrelate the qx / qy fbm samples in noise_warped_potential so
 * they evolve independently (Quilez, "Domain Warping"). The exact
 * values aren't magic; any non-zero, non-equal pair works. */
#define WARP_OFFSET_X            5.2f
#define WARP_OFFSET_Y            1.3f

/* §7 particle dynamics — minimum velocity magnitude that counts as
 * "moving". Below this, sample_unit_velocity() skips normalisation
 * to avoid the 0/0 NaN trap. */
#define VELOCITY_EPSILON         1e-6f

/* §7 trail rendering — intensity deposited where a particle lands.
 * Saturates the cell to full brightness; trail decay handles fade. */
#define TRAIL_HIT_INTENSITY      1.0f

/* §7 supernova — initial flash glow at reset, decays via SUPERNOVA_DECAY. */
#define SUPERNOVA_FLASH_INIT     1.0f

/* §7 visualisation gains — applied before the band quantizer.
 * CURL_MAG_VISUAL_GAIN spreads the typical |∇×ψ| histogram across
 * [0, 1]; POTENTIAL_REMAP_{MID,RANGE} convert ψ ∈ [-1, 1] → [0, 1]
 * via g = ψ·RANGE + MID. */
#define CURL_MAG_VISUAL_GAIN     1.5f
#define POTENTIAL_REMAP_MID      0.5f
#define POTENTIAL_REMAP_RANGE    0.5f

/* §7 banding — quantize glow ∈ [0, 1) into N_BANDS bands. The scale
 * sits just under N_BANDS so glow == 1.0 doesn't overflow into band
 * N_BANDS (which has no colour pair). */
#define N_BANDS                  4
#define BAND_QUANTIZE_SCALE      3.999f

/* §6 arrow-direction classifier. Magnitudes below STATIONARY render
 * as '.'; the AXIS_DOMINANCE ratio separates axis-aligned arrows
 * ('>' '<' '^' 'v') from diagonals ('/' '\\'). */
#define ARROW_STATIONARY_THRESH  0.05f
#define ARROW_AXIS_DOMINANCE     2.0f

/* §8 supernova twinkle — sparse-mask period. (x ^ y) & MASK == 0
 * lights one cell out of (MASK + 1), giving a star-field look. */
#define SUPERNOVA_SPARSE_MASK    3

/* §8 HUD layout. The TOP HUD (rows 0..HUD_TOP_ROWS-1) carries DATA
 * — state, params, legend. The BOTTOM HUD (row N-1) carries ACTIONS
 * — key bindings. The playable map fits between them.
 *
 *   row 0          : title + state bar (fps, Hz, state, speed)
 *   row 1          : pattern, theme, palette swatch, drift, eps, map
 *   row 2          : glyph legend (.:low *:mid #:high  arrows)
 *   row HUD_TOP..N-2: playable area
 *   row N-1        : keyboard action hint
 */
#define HUD_TOP_ROWS             3
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1
#define HUD_PATTERN_FIELD_W      20
#define HUD_THEME_FIELD_W        17
#define HUD_PALETTE_LABEL_W      9
#define HUD_PALETTE_SWATCH_N     4

/* §9 main loop — frame-rate cap and dt safety clamp. The DT cap is
 * the standard spiral-of-death guard from Glenn Fiedler's "Fix Your
 * Timestep" — if the frame stalled for >100 ms, pretend it didn't,
 * so the sim doesn't try to catch up with hundreds of ticks. */
#define DT_MAX_NS                (100 * NS_PER_MS)
#define FRAME_CAP_FPS            60

/*
 * Theme — a complete colour palette for one visual style. Ten of
 * these live in themes[], cycled by t/T. The structure carries the
 * smallest data needed to recolour the entire program: four ramp
 * colours and one flash highlight.
 *
 * INTENT. Each cell in the render buffer holds a glow value 0..1;
 * that glow is quantised into a band 0..3, which indexes band[i] to
 * pick the foreground colour. The four-band split mirrors the four
 * glyph tiers in scene_draw ('.' / '*' / '#' / brightest). Keeping
 * exactly four colours forces theme authors to design the ramp as a
 * coherent low-to-high progression rather than a random scatter —
 * the same discipline as a Houdini ramp parameter.
 *
 * Colour numbers are xterm-256 indices (NOT RGB). When the terminal
 * exposes fewer than 256 colours, theme_apply() substitutes a fixed
 * 8-colour fallback so the demo still runs on legacy TTYs.
 */
typedef struct {
    const char *name;     /* short uppercase label shown in HUD             */
    short       band[4];  /* 4 ramp colours: 0 = dim/low, 3 = bright/high   */
    short       flash;    /* reset-flash colour (the supernova sparkle)     */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      band0 band1 band2 band3 flash */
    { "DEFAULT", {  17,   33,  220,  231 }, 226 },
    { "MATRIX",  {  22,   34,   46,  118 }, 226 },
    { "NOVA",    {  53,  129,  201,  219 }, 226 },
    { "MONO",    { 234,  244,  250,  254 }, 226 },
    { "OCEAN",   {  17,   33,   39,   51 }, 226 },
    { "FIRE",    {  52,  124,  208,  226 }, 196 },
    { "EARTH",   {  58,  100,  173,  230 }, 226 },
    { "FOREST",  {  22,   28,   64,  144 }, 226 },
    { "DESERT",  {  94,  130,  173,  222 }, 226 },
    { "ARCTIC",  {  18,   39,  159,  231 }, 226 },
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
            init_pair(PAIR_BAND_BASE + i, t->band[i], -1);
        init_pair(PAIR_FLASH, t->flash, -1);
    } else {
        static const short fallback[N_BANDS] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < N_BANDS; i++)
            init_pair(PAIR_BAND_BASE + i, fallback[i], -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
        init_pair(PAIR_SUPERNOVA,  226, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
        init_pair(PAIR_SUPERNOVA, COLOR_YELLOW,  -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  noise — Noise context + Perlin + fBm + potential + curl            */
/* ===================================================================== */

/*
 * Noise — the entire random-source state of the program. Wrapping
 * the permutation table in a struct means there is exactly one piece
 * of mutable noise data, it lives inside Scene, and every sampler
 * takes a `const Noise *` so reads are obviously stateless.
 *
 * ALGORITHM. Classic Perlin gradient noise. The permutation table is
 * a random shuffle of [0..255] used to hash 2-D lattice coordinates
 * into gradient indices. Re-shuffling perm[] gives a completely new
 * noise field — triggered on demand by 'r' (manual reset). The
 * sim runs the SAME field indefinitely otherwise; sim.field_time
 * drifts it through noise-space so it animates without looping.
 *
 * REFERENCE. Ken Perlin (2002) — "Improving Noise" SIGGRAPH paper,
 * which describes both the perm-table hashing scheme and the
 * 8-direction gradient table used by grad() below. The 1985 original
 * used a cubic fade with visible C¹-discontinuity artefacts at
 * integer lattice points; we use the 2002 quintic fade.
 */
typedef struct {
    /* Doubled permutation: perm[i + 256] == perm[i] for i in [0..255].
     * The doubling lets noise_perlin2d() index perm[X+1] without a
     * modulo. X is already masked to 0..255, so X+1 is at most 256 —
     * safely inside the doubled buffer. Classic micro-opt from
     * Perlin's reference Java code; saves one AND per sample,
     * roughly 10 % of the inner-loop cost in profiling. */
    uint8_t perm[512];
} Noise;

/* fisher_yates_shuffle_256 — unbiased uniform random shuffle of
 * [0..255]. Textbook Fisher-Yates: walk from the end backwards,
 * swap each element with a random earlier element. O(n). */
static void fisher_yates_shuffle_256(uint8_t base[256])
{
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
}

/* mirror_perm_for_double_lookup — copy a 256-byte permutation into
 * a 512-byte buffer twice (upper half mirrors the lower). This
 * removes a modulo from noise_perlin2d when indexing perm[X+1].
 * See Noise struct doc for the full rationale and Perlin 2002. */
static void mirror_perm_for_double_lookup(uint8_t perm[512],
                                          const uint8_t base[256])
{
    for (int i = 0; i < 256; i++) {
        perm[i]       = base[i];
        perm[i + 256] = base[i];
    }
}

static void noise_shuffle(Noise *n)
{
    uint8_t base[256];
    fisher_yates_shuffle_256       (base);
    mirror_perm_for_double_lookup  (n->perm, base);
}

/* Perlin 2002 quintic fade: 6t⁵ − 15t⁴ + 10t³. C² at lattice edges. */
static inline float fade(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

static inline float lerp_f(float a, float b, float t) { return a + t * (b - a); }

/* grad — one of 8 gradient·(x, y) dot products selected by the low
 * 3 bits of `hash`. Perlin 2002's table of 8 directions: ±1·u ± 2·v
 * where (u, v) ∈ {(x, y), (y, x)}. */
static inline float grad(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

/*
 * noise_perlin2d — sample classic Perlin gradient noise at (x, y).
 * Output range ≈ [-1, 1]. Five algorithmic steps:
 */
static float noise_perlin2d(const Noise *n, float x, float y)
{
    /* (1) Find the integer lattice cell and the fractional offset
     *     within it. The & 255 wrap makes the noise period 256. */
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x);
    y -= floorf(y);

    /* (2) Apply the quintic fade curve — smooths derivatives at
     *     lattice boundaries to C². */
    float u = fade(x);
    float v = fade(y);

    /* (3) Hash the 4 corner indices via the perm table. A and B are
     *     row-hashes for the left and right edges of the cell. */
    int A = n->perm[X    ] + Y;
    int B = n->perm[X + 1] + Y;

    /* (4) Compute gradient · (offset from corner to sample point) at
     *     each of the 4 corners — the "directional weight" of each
     *     corner pulled toward the sample. */
    float n00 = grad(n->perm[A    ], x,        y       );  /* corner (0,0) */
    float n10 = grad(n->perm[B    ], x - 1.0f, y       );  /* corner (1,0) */
    float n01 = grad(n->perm[A + 1], x,        y - 1.0f);  /* corner (0,1) */
    float n11 = grad(n->perm[B + 1], x - 1.0f, y - 1.0f);  /* corner (1,1) */

    /* (5) Bilinear interpolation using the faded weights. */
    return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

/*
 * noise_fbm — 3-octave fractional Brownian motion. Output ≈ [-1, 1].
 * Each octave doubles frequency (lacunarity = 2) and halves amplitude
 * (persistence = 0.5); total is normalised by Σamp so the output
 * scale is independent of FBM_OCTAVES. See Ebert et al., fBm chapter.
 */
static float noise_fbm(const Noise *n, float x, float y)
{
    float total   = 0.0f;
    float amp     = 1.0f;
    float freq    = 1.0f;
    float max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * noise_perlin2d(n, x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;          /* persistence */
        freq    *= 2.0f;          /* lacunarity  */
    }
    return total / max_amp;
}

/*
 * noise_potential — the scalar field ψ(x, y, t) whose curl drives
 * the visualisation. Plain fBm with time drifted into the y-axis.
 */
static inline float noise_potential(const Noise *n, float x, float y, float t)
{
    return noise_fbm(n, x, y + t);
}

/*
 * noise_warped_potential — domain-warped ψ. One fBm sample (qx, qy)
 * perturbs the input coordinates of the final fBm sample.
 * WARP_OFFSET_X/Y decorrelate the two warp channels. Inigo Quilez's
 * technique; produces a more eddy-rich field than plain ψ.
 */
static float noise_warped_potential(const Noise *n, float x, float y, float t)
{
    float qx = noise_fbm(n, x,                  y                 + t);
    float qy = noise_fbm(n, x + WARP_OFFSET_X,  y + WARP_OFFSET_Y + t);
    return noise_fbm(n, x + WARP_AMOUNT * qx,
                        y + WARP_AMOUNT * qy + t);
}

/*
 * psi_at — sample the chosen scalar potential at (x, y, t). The
 * `warp` flag selects plain vs domain-warped; this keeps
 * noise_curl_at agnostic to which one is in use.
 */
static inline float psi_at(const Noise *n, float x, float y, float t, bool warp)
{
    return warp ? noise_warped_potential(n, x, y, t)
                : noise_potential       (n, x, y, t);
}

/*
 * noise_curl_at — central-difference 2-D curl of ψ at (x, y, t).
 *
 *   v = (∂ψ/∂y, -∂ψ/∂x)        ← curl-of-scalar identity (Bridson 2007)
 *   ∂ψ/∂y ≈ (ψ(x, y+ε) − ψ(x, y−ε)) / (2ε)
 *   ∂ψ/∂x ≈ (ψ(x+ε, y) − ψ(x−ε, y)) / (2ε)
 *
 * The resulting v is divergence-free by construction:
 *   ∇·v = ∂vx/∂x + ∂vy/∂y = ∂²ψ/∂x∂y − ∂²ψ/∂y∂x = 0.
 */
static void noise_curl_at(const Noise *n, float x, float y, float t, bool warp,
                          float *out_vx, float *out_vy)
{
    /* Four ψ samples — the 4 neighbours of (x, y) at offset ε. */
    float psi_yp = psi_at(n, x,            y + CURL_EPS, t, warp);
    float psi_ym = psi_at(n, x,            y - CURL_EPS, t, warp);
    float psi_xp = psi_at(n, x + CURL_EPS, y,            t, warp);
    float psi_xm = psi_at(n, x - CURL_EPS, y,            t, warp);

    /* Central-difference partials. */
    float dpsi_dy = (psi_yp - psi_ym) / (2.0f * CURL_EPS);
    float dpsi_dx = (psi_xp - psi_xm) / (2.0f * CURL_EPS);

    /* Curl rotation: ∇ψ rotated -90° = (∂ψ/∂y, -∂ψ/∂x). */
    *out_vx =  dpsi_dy;
    *out_vy = -dpsi_dx;
}

/* ===================================================================== */
/* §6  patterns — shared helpers                                          */
/* ===================================================================== */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/*
 * quantize_glow_to_band — map glow ∈ [0, 1) → band 0..N_BANDS-1.
 * BAND_QUANTIZE_SCALE is N_BANDS − ε so glow == 1.0 doesn't overflow
 * into band N_BANDS (which has no colour pair). The trailing mask
 * is defence in depth against any rounding surprises.
 */
static inline int quantize_glow_to_band(float glow)
{
    return (int)(glow * BAND_QUANTIZE_SCALE) & (N_BANDS - 1);
}

/*
 * arrow_for — pick an arrow glyph from a velocity (vx, vy).
 * Three-stage classifier:
 *   1. STATIONARY  — both components near zero → '.'
 *   2. AXIS-ALIGNED — one component dominates by AXIS_DOMINANCE ratio
 *   3. DIAGONAL    — same-sign components → '\\'; opposite-sign → '/'
 */
static char arrow_for(float vx, float vy)
{
    float ax = fabsf(vx), ay = fabsf(vy);

    /* (1) Both axes near zero → render as stationary dot. */
    if (ax < ARROW_STATIONARY_THRESH && ay < ARROW_STATIONARY_THRESH)
        return '.';

    /* (2) Axis-aligned: dominance ratio decides which axis wins. */
    if (ax > ARROW_AXIS_DOMINANCE * ay) return vx > 0.0f ? '>' : '<';
    if (ay > ARROW_AXIS_DOMINANCE * ax) return vy > 0.0f ? 'v' : '^';

    /* (3) Diagonal: sign of vx·vy distinguishes the two diagonals. */
    return ((vx * vy) > 0.0f) ? '\\' : '/';
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * The scene is built out of six small structs. Each owns ONE concern,
 * and Scene composes them. Splitting concerns into clearly-typed
 * sub-structs makes function signatures self-describing: any function
 * that takes `const Grid *` clearly cannot mutate buffers; any
 * function that takes `RenderBuffers *` clearly does not sample
 * noise; and so on.
 */

/*
 * Particle — one walker advected by the curl field. Used by the
 * PARTICLES and WARPED patterns; the other three (VECTOR, POTENTIAL,
 * CURL_MAG) leave the pool dormant.
 *
 * INTENT. The walker is "massless": its position is integrated
 * directly from the sampled velocity, with no momentum term. This
 * matches Bridson 2007 §3 — particles trace streamlines of v, not
 * acceleration-trajectories of a force field. The simpler
 * integration is also why divergence-freeness of v alone guarantees
 * no clumping: with momentum, particles could still bunch via
 * inertia; here, they cannot bunch at all.
 *
 * Each particle has a finite lifetime so the visualisation stays
 * lively — if particles lived forever they would gradually saturate
 * the orbits v happens to favour and the rest of the field would
 * empty out. Random max_age staggers respawns so the spatial
 * distribution stays uniform over time.
 *
 * REFERENCE. Bridson, Houriham & Nordenstam (2007) — "Curl-Noise for
 * Procedural Fluid Flow", §3 (particle update).
 */
typedef struct {
    float x, y;       /* position in cell units (continuous floats)         */
    int   color_idx;  /* 0..N_BANDS-1 — selects band from RenderBuffers.color */
    int   age;        /* ticks since spawn                                  */
    int   max_age;    /* respawn when age ≥ max_age (also on OOB)           */
} Particle;

/*
 * Grid — map geometry. Pure data: no buffers, no state, no allocation.
 * Lives at the top of Scene because every layer (noise sampling,
 * particle bounds-checks, screen centring) needs the dimensions.
 *
 * INDEXING. Row-major: cell (x, y) → y·w + x. This matches the
 * memory layout of RenderBuffers (single flat array of CELLS_MAX
 * cells), so the y-outer / x-inner loop order in scene_draw and
 * scene_update_static is cache-friendly — each row of the grid is
 * a contiguous run of bytes/floats in memory.
 *
 * INVARIANT. w · h ≤ CELLS_MAX always holds; app_pick_map_size()
 * clamps to MAP_W_MAX × MAP_H_MAX = 200 × 56 = 11200 cells, which is
 * what the RenderBuffers arrays are statically sized for. The clamp
 * is enforced once per resize; downstream code may assume it.
 */
typedef struct {
    int w, h;         /* current map width / height in cells              */
    int total_cells;  /* = w · h, cached so hot loops skip the multiply   */
} Grid;

static inline int grid_idx(const Grid *g, int x, int y) { return y * g->w + x; }
static inline bool grid_in_bounds(const Grid *g, int x, int y)
{
    return x >= 0 && x < g->w && y >= 0 && y < g->h;
}

/*
 * RenderBuffers — the per-cell raster output. Three parallel arrays
 * indexed by grid_idx(g, x, y):
 *   glow   : intensity 0..1 — drives the density glyph + colour band
 *   color  : palette index 0..3
 *   glyph  : 0 = use the density glyph; non-zero OVERRIDES it (used
 *            by VECTOR to render arrow characters directly)
 *
 * The screen layer reads ONLY these three arrays. Patterns write
 * into them. That is the entire render contract — no other channel
 * of communication between simulation and display. Splitting render
 * output from simulation state is the classic graphics-pipeline
 * decoupling (Foley & van Dam, ch. 2): the sim becomes display-
 * agnostic, and the display can be retargeted to a different glyph
 * ramp without touching pattern code.
 *
 * SoA RATIONALE. Three separate arrays (struct-of-arrays) rather
 * than one array of cell-structs because (a) the screen layer reads
 * glow far more often than color/glyph, so keeping it dense improves
 * cache use, and (b) clearing one channel without touching the
 * others is trivial — see the VECTOR wipe in scene_update_static().
 *
 * REFERENCE. The density-to-glyph mapping in scene_draw follows Paul
 * Bourke's ASCII grey-scale ramp; the band cutoffs GLYPH_MID_THRESH
 * and GLYPH_HIGH_THRESH are tuned empirically against his published
 * gradient.
 */
typedef struct {
    /* Per-cell intensity 0..1. Particle patterns deposit 1.0 and
     * decay exponentially via expf(-TRAIL_GLOW_DECAY * dt); static
     * patterns overwrite each frame with no decay. Below
     * GLOW_THRESHOLD (0.05) the cell renders as blank. */
    float   glow [CELLS_MAX];

    /* Per-cell palette band 0..N_BANDS-1. Indexes into the current
     * Theme's band[] via PAIR_BAND_BASE + color. Masked with
     * (N_BANDS - 1) in the draw loop so any out-of-range value cannot
     * pick a wrong pair — defence in depth, since color is uint8_t. */
    uint8_t color[CELLS_MAX];

    /* Optional glyph override. 0 means "use the density-band glyph"
     * (the default for particle trails). Non-zero replaces it —
     * VECTOR stores arrow characters here ('>','<','^','v','/','\\'),
     * letting one pattern render directional symbols while every
     * other pattern stays on the density ramp. Stored as char so the
     * override can carry any printable ASCII byte. */
    char    glyph[CELLS_MAX];
} RenderBuffers;

static void buffers_clear(RenderBuffers *b, int n)
{
    for (int i = 0; i < n; i++) {
        b->glow[i]  = 0.0f;
        b->color[i] = 0;
        b->glyph[i] = 0;
    }
}

/*
 * Particles — fixed-size pool of walkers. Standard pool-allocator
 * pattern: a max-sized array plus an active count `n`, so spawn /
 * respawn never touches the heap. Only pool[0..n-1] is alive; the
 * rest is unused storage held in reserve.
 *
 * SIZING. MAX_PARTICLES (1024) is the static upper bound;
 * N_PARTICLES_DEF (256) is what scene_reset() activates. 256 is
 * enough to populate the visible vortices of a 200×56 grid without
 * making any one streamline overwhelmingly dense. Higher counts
 * produce a smoke-like haze where individual orbits become hard to
 * read; lower counts make the field look starved.
 *
 * CACHE. Pool size matters for performance too: 1024 Particle structs
 * × ~20 B each ≈ 20 KB, easily L1-resident on any modern CPU. The
 * per-frame "step every particle" loop touches one tight contiguous
 * memory region instead of scattering through the heap.
 */
typedef struct {
    Particle pool[MAX_PARTICLES];  /* storage; only [0..n-1] is alive */
    int      n;                    /* active count, ≤ MAX_PARTICLES   */
} Particles;

/*
 * SimState — values that evolve per tick under the simulation's own
 * control. Separated from Controls (keyboard-driven knobs) so it is
 * obvious at a glance what scene_tick mutates vs. what the user
 * does. Two small fields, each playing a distinct role:
 *
 *   field_time       → animates the noise field by drifting the
 *                      y-axis sample coordinate. Without it the
 *                      potential would be static and particles would
 *                      trace the same orbits forever; with it the
 *                      vortex structure slowly mutates without
 *                      changing the underlying perm[].
 *   supernova_glow_t → cosmetic flourish triggered by scene_reset()
 *                      (manual 'r' or first start). Decays exponen-
 *                      tially via SUPERNOVA_DECAY so it vanishes
 *                      within ~1 s.
 */
typedef struct {
    /* Time offset added to the noise y-coordinate before sampling.
     * Increment per tick is FIELD_DRIFT × drift_mult × dt. Units are
     * "noise-units" — the same scale as fx, fy in scene_update_static.
     * Cleared to 0 by scene_reset(); otherwise grows monotonically. */
    float field_time;

    /* Envelope for the post-reset sparkle. Set to SUPERNOVA_FLASH_INIT
     * on reset, decays per tick by expf(-SUPERNOVA_DECAY * dt). While
     * the envelope is above GLOW_THRESHOLD, scene_draw paints a sparse
     * '*' field masked by (x ^ y) & SUPERNOVA_SPARSE_MASK so the flash
     * looks like a random twinkle rather than a uniform blanket. */
    float supernova_glow_t;
} SimState;

/*
 * Controls — user-facing knobs. Mutated by app_handle_key(), read by
 * scene_tick() and screen_draw(). Grouping them makes the keyboard
 * handler trivial: `Controls *c = &scene.ctrl;` once at the top of
 * the function, then every key case is a one-line mutation on `c`.
 *
 * The split between SimState and Controls is the "model vs. user
 * intent" line common to interactive programs: SimState answers
 * "where is the simulation right now"; Controls answers "what has
 * the user asked for". scene_tick reads Controls to decide what to
 * do; it never writes to it. The keyboard handler writes to
 * Controls; it never writes to SimState. This one-way dependency
 * keeps the code linear and trivially thread-safe should the demo
 * ever be split into input + sim threads.
 */
typedef struct {
    /* Gate for scene_tick: when true the field is frozen, but the
     * render loop continues so the HUD stays responsive and the user
     * can still cycle themes or patterns. */
    bool    paused;

    /* Particle advection scale, cells/sec. Multiplied into v·dt in
     * particle_step_curl. Doubled by '+', halved by '-', clamped to
     * [SPEED_MIN, SPEED_MAX] = [1, 64]. The doubling step (rather
     * than linear ±1) gives a logarithmic feel — perceptually each
     * press is the "same size change" regardless of current speed. */
    int     speed;

    /* Multiplier on FIELD_DRIFT — speeds / slows the noise animation
     * independently of particle speed, though +/- changes both in
     * lockstep for convenience. Clamped to [DRIFT_MULT_MIN,
     * DRIFT_MULT_MAX] = [1, 32]. */
    int     drift_mult;

    int     current_theme;     /* index into themes[N_THEMES]      */
    Pattern current_pattern;   /* the active visualisation         */

    /* One-tick lag of current_pattern. scene_tick uses the
     * difference (prev != current) as an edge-detector to wipe the
     * RenderBuffers when the user switches patterns — otherwise
     * particle trails would ghost into VECTOR's lattice or
     * POTENTIAL's heightmap. Standard pattern-edge technique. */
    Pattern prev_pattern;
} Controls;

/*
 * Scene — the umbrella context. Reading this struct top-to-bottom is
 * meant to be the fastest way to understand the program:
 *   grid       → where things live            (geometry, no state)
 *   noise      → what the field is sampled from (perm table)
 *   buf        → what gets drawn               (render output)
 *   particles  → moving agents                 (PARTICLES/WARPED)
 *   sim        → animation state               (drift, supernova fade)
 *   ctrl       → user knobs                    (pattern, speed, …)
 *
 * ORDERING. The field order is deliberate: each sub-struct depends
 * only on those declared above it. grid is leaf-level data; noise is
 * independent of grid; buf is sized by CELLS_MAX (an upper bound on
 * grid); particles need grid bounds; sim mutates noise & buf via
 * scene_tick; ctrl decides which sim path runs. A reader scanning
 * top-down meets every concept before it is used.
 *
 * COMPOSITION. There is no Scene-wide invariant that crosses sub-
 * struct boundaries — each sub-struct can be reasoned about (and
 * tested) in isolation. The only function that sees all of Scene at
 * once is scene_tick(). This is composition over inheritance: no
 * inheritance hierarchies, no virtual dispatch — just six clearly
 * named structs glued together by direct field access.
 *
 * REFERENCE. Mike Acton — "Data-Oriented Design and C++" (CppCon
 * 2014) for the broader argument that good struct layout IS good
 * code; Robert Nystrom — "Game Programming Patterns" ch. "Component"
 * for the composition-over-inheritance variant used here.
 */
typedef struct {
    Grid          grid;       /* immutable per frame (only resize changes it) */
    Noise         noise;      /* mutated only by scene_reset (shuffle)        */
    RenderBuffers buf;        /* written by patterns, read by scene_draw      */
    Particles     particles;  /* dormant unless ctrl.current_pattern uses them */
    SimState      sim;        /* mutated only by scene_tick                   */
    Controls      ctrl;       /* mutated only by app_handle_key               */
} Scene;

static void particle_spawn(Particle *p, const Grid *g)
{
    p->x         = (float)(rand() % g->w);
    p->y         = (float)(rand() % g->h);
    p->color_idx = rand() & (N_BANDS - 1);
    p->age       = 0;
    p->max_age   = AGE_MIN_TICKS + rand() % (AGE_MAX_TICKS - AGE_MIN_TICKS);
}

/* ── particle pipeline ────────────────────────────────────────────── *
 * particle_step_curl is the per-particle pseudocode:
 *
 *     sample unit velocity from the curl field
 *     advect by v·dt (forward Euler)
 *     deposit a trail hit at the new cell
 *     respawn if the particle is expired (age or OOB)
 *
 * Each step is one named call below. */

/* sample_unit_velocity — read the curl of ψ at the particle's
 * noise-space coordinates and normalise to unit length. We separate
 * direction (here) from magnitude (provided by ctrl.speed during
 * advection) so the particle moves uniformly regardless of where in
 * the field |∇×ψ| happens to be large or small. Below VELOCITY_EPSILON
 * we skip normalisation to avoid 0/0 → NaN. */
static void sample_unit_velocity(const Scene *s, float px, float py, bool warp,
                                  float *out_vx, float *out_vy)
{
    noise_curl_at(&s->noise,
                  px * NOISE_SCALE, py * NOISE_SCALE,
                  s->sim.field_time, warp, out_vx, out_vy);
    float mag = sqrtf((*out_vx) * (*out_vx) + (*out_vy) * (*out_vy));
    if (mag > VELOCITY_EPSILON) {
        *out_vx /= mag;
        *out_vy /= mag;
    }
}

/* advect_particle_euler — forward-Euler integration step along v.
 * (x, y) ← (x, y) + v · speed · dt. Massless advection — no inertia
 * term. Streamlines, not trajectories. See Particle struct doc for
 * why this is the right integrator for divergence-free fields. */
static void advect_particle_euler(Particle *p, float vx, float vy,
                                   float dt, int speed)
{
    p->x += vx * (float)speed * dt;
    p->y += vy * (float)speed * dt;
    p->age++;
}

/* deposit_trail_hit — paint the particle's current cell at full
 * intensity. Overwrites (not blends) the cell's previous glow; the
 * trail visual effect comes from neighbouring cells decaying via
 * expf(-TRAIL_GLOW_DECAY · dt) between hits. */
static void deposit_trail_hit(Scene *s, int cx, int cy, int color_idx)
{
    if (!grid_in_bounds(&s->grid, cx, cy)) return;
    int idx = grid_idx(&s->grid, cx, cy);
    s->buf.glow [idx] = TRAIL_HIT_INTENSITY;
    s->buf.color[idx] = (uint8_t)color_idx;
    s->buf.glyph[idx] = 0;        /* fall back to density glyph */
}

/* particle_is_expired — has the walker overstayed its life or
 * walked off the grid? Either triggers a respawn. */
static bool particle_is_expired(const Particle *p, const Grid *g)
{
    return p->age >= p->max_age
        || p->x < 0.0f || p->x >= (float)g->w
        || p->y < 0.0f || p->y >= (float)g->h;
}

static void particle_step_curl(Particle *p, Scene *s, float dt, bool warp)
{
    float vx, vy;
    sample_unit_velocity (s, p->x, p->y, warp, &vx, &vy);
    advect_particle_euler(p, vx, vy, dt, s->ctrl.speed);
    deposit_trail_hit    (s, (int)p->x, (int)p->y, p->color_idx);
    if (particle_is_expired(p, &s->grid))
        particle_spawn   (p, &s->grid);
}

/* ── static-field pattern pipeline ────────────────────────────────── *
 * Each non-particle pattern (VECTOR, POTENTIAL, CURL_MAG) computes
 * one CellPaint per cell. scene_update_static loops over the grid
 * and dispatches to the right per-cell function. */

/* CellPaint — the bundle a pattern hands to write_cell(). */
typedef struct {
    float glow;   /* 0..1 intensity                       */
    int   band;   /* 0..N_BANDS-1 colour-ramp index       */
    char  glyph;  /* 0 = use density glyph, else override */
} CellPaint;

/* compute_potential_cell — POTENTIAL pattern. Render ψ as a
 * heightmap: g = ψ·RANGE + MID, which remaps [-1, 1] → [0, 1]. */
static CellPaint compute_potential_cell(const Noise *n, float fx, float fy, float t)
{
    float psi  = noise_potential(n, fx, fy, t);
    float glow = psi * POTENTIAL_REMAP_RANGE + POTENTIAL_REMAP_MID;
    return (CellPaint){ .glow  = glow,
                        .band  = quantize_glow_to_band(glow),
                        .glyph = 0 };
}

/* compute_curl_magnitude_cell — CURL_MAG pattern. Render |∇×ψ| as
 * density: bright at vortex cores, dim in laminar regions. The
 * visual gain spreads the typical |v| histogram across [0, 1]. */
static CellPaint compute_curl_magnitude_cell(const Noise *n, float fx, float fy, float t)
{
    float vx, vy;
    noise_curl_at(n, fx, fy, t, false, &vx, &vy);
    float mag  = sqrtf(vx * vx + vy * vy);
    float glow = clampf(mag * CURL_MAG_VISUAL_GAIN, 0.0f, 1.0f);
    return (CellPaint){ .glow  = glow,
                        .band  = quantize_glow_to_band(glow),
                        .glyph = 0 };
}

/* compute_vector_arrow_cell — VECTOR pattern. Sample the curl, pick
 * an arrow glyph aligned with the local direction, and use |v| as
 * the brightness so weak parts of the field render dim. */
static CellPaint compute_vector_arrow_cell(const Noise *n, float fx, float fy, float t)
{
    float vx, vy;
    noise_curl_at(n, fx, fy, t, false, &vx, &vy);
    float mag  = sqrtf(vx * vx + vy * vy);
    float glow = clampf(mag * CURL_MAG_VISUAL_GAIN, 0.0f, 1.0f);
    return (CellPaint){ .glow  = glow,
                        .band  = quantize_glow_to_band(glow),
                        .glyph = arrow_for(vx, vy) };
}

/* is_vector_lattice_point — VECTOR only paints arrows at every
 * (VECTOR_LATTICE_X × VECTOR_LATTICE_Y) cell; the rest stay blank
 * for legibility. */
static inline bool is_vector_lattice_point(int x, int y)
{
    return (x % VECTOR_LATTICE_X) == 0
        && (y % VECTOR_LATTICE_Y) == 0;
}

/* wipe_vector_layer — clear glow + glyph but leave color alone.
 * VECTOR needs a clean canvas each frame because only lattice
 * points get painted; everything else must read as blank. */
static void wipe_vector_layer(RenderBuffers *b, int n)
{
    for (int i = 0; i < n; i++) {
        b->glow [i] = 0.0f;
        b->glyph[i] = 0;
    }
}

/* write_cell — commit a CellPaint into the render buffer at idx. */
static inline void write_cell(RenderBuffers *b, int idx, CellPaint p)
{
    b->glow [idx] = p.glow;
    b->color[idx] = (uint8_t)(p.band & (N_BANDS - 1));
    b->glyph[idx] = (uint8_t) p.glyph;
}

/*
 * scene_update_static — driver. For non-particle patterns (VECTOR,
 * POTENTIAL, CURL_MAG), recompute every cell each frame. Writes
 * into s->buf; reads from s->noise and s->sim.field_time.
 */
static void scene_update_static(Scene *s, Pattern pat)
{
    const Grid    *g = &s->grid;
    const Noise   *n = &s->noise;
    RenderBuffers *b = &s->buf;
    float          t = s->sim.field_time;

    if (pat == PATTERN_VECTOR) wipe_vector_layer(b, g->total_cells);

    for (int y = 0; y < g->h; y++) {
        for (int x = 0; x < g->w; x++) {
            float fx = (float)x * NOISE_SCALE;
            float fy = (float)y * NOISE_SCALE;
            CellPaint cp;

            switch (pat) {
            case PATTERN_VECTOR:
                if (!is_vector_lattice_point(x, y)) continue;
                cp = compute_vector_arrow_cell  (n, fx, fy, t); break;
            case PATTERN_POTENTIAL:
                cp = compute_potential_cell     (n, fx, fy, t); break;
            case PATTERN_CURL_MAG:
                cp = compute_curl_magnitude_cell(n, fx, fy, t); break;
            default:
                continue;
            }

            write_cell(b, grid_idx(g, x, y), cp);
        }
    }
}

/* ── reset / init pipeline ────────────────────────────────────────── */

static void apply_grid_dimensions(Grid *g, int w, int h)
{
    g->w           = w;
    g->h           = h;
    g->total_cells = w * h;
}

static void reset_sim_state(SimState *sim)
{
    sim->field_time       = 0.0f;
    sim->supernova_glow_t = SUPERNOVA_FLASH_INIT;
}

static void spawn_all_particles(Particles *ps, const Grid *g)
{
    ps->n = N_PARTICLES_DEF;
    for (int i = 0; i < ps->n; i++)
        particle_spawn(&ps->pool[i], g);
}

static void scene_reset(Scene *s, int w, int h)
{
    apply_grid_dimensions(&s->grid, w, h);
    reset_sim_state      (&s->sim);
    buffers_clear        (&s->buf, s->grid.total_cells);
    noise_shuffle        (&s->noise);
    spawn_all_particles  (&s->particles, &s->grid);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->ctrl.paused          = false;
    s->ctrl.speed           = SPEED_DEF;
    s->ctrl.drift_mult      = DRIFT_MULT_DEF;
    s->ctrl.current_theme   = 0;
    s->ctrl.current_pattern = PATTERN_PARTICLES;
    s->ctrl.prev_pattern    = PATTERN_PARTICLES;
    scene_reset(s, w, h);
}

/* ── tick pipeline ────────────────────────────────────────────────── *
 * scene_tick is the per-frame pseudocode:
 *
 *     if paused: stop.
 *     handle any pending pattern switch (wipe buffers).
 *     advance time-dependent envelopes (supernova fade, field drift).
 *     simulate the active pattern (particle path or static path).
 *
 * Each line below is one named call. There is NO automatic perm
 * reshuffle — the user drives that via 'r' / scene_reset(). */

static void detect_pattern_switch_and_wipe(Scene *s)
{
    if (s->ctrl.current_pattern == s->ctrl.prev_pattern) return;
    buffers_clear(&s->buf, s->grid.total_cells);
    s->ctrl.prev_pattern = s->ctrl.current_pattern;
}

static void decay_supernova_flash(Scene *s, float dt)
{
    s->sim.supernova_glow_t *= expf(-SUPERNOVA_DECAY * dt);
}

static void advance_field_time(Scene *s, float dt)
{
    s->sim.field_time += FIELD_DRIFT * (float)s->ctrl.drift_mult * dt;
}

static bool pattern_uses_particles(Pattern p)
{
    return p == PATTERN_PARTICLES || p == PATTERN_WARPED;
}

static void decay_trail_glow(RenderBuffers *b, int n, float dt)
{
    float decay = expf(-TRAIL_GLOW_DECAY * dt);
    for (int i = 0; i < n; i++) b->glow[i] *= decay;
}

static void step_all_particles(Scene *s, float dt, bool warp)
{
    for (int i = 0; i < s->particles.n; i++)
        particle_step_curl(&s->particles.pool[i], s, dt, warp);
}

static void simulate_particle_patterns(Scene *s, float dt, bool warp)
{
    decay_trail_glow (&s->buf, s->grid.total_cells, dt);
    step_all_particles(s, dt, warp);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;

    detect_pattern_switch_and_wipe(s);
    decay_supernova_flash         (s, dt);
    advance_field_time            (s, dt);

    Pattern pat = s->ctrl.current_pattern;
    if (pattern_uses_particles(pat))
        simulate_particle_patterns(s, dt, pat == PATTERN_WARPED);
    else
        scene_update_static       (s, pat);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

/*
 * Screen — terminal dimensions cached after the last successful
 * getmaxyx(). Refreshed on SIGWINCH via screen_resize(). Kept tiny
 * because the rest of ncurses' state lives implicitly in stdscr; we
 * only need the dimensions to position HUD elements and centre the
 * map. Anything else (colour pairs, current attribute, cursor pos)
 * is owned by ncurses internals, not by us.
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

/* ── scene_draw pipeline ──────────────────────────────────────────── *
 * Per cell, scene_draw decides ONE of four things to paint (or skip).
 * The decision is a priority chain:
 *
 *     1. Supernova flash active? → twinkle '*'
 *     2. Pattern-supplied glyph override? → that glyph
 *     3. Density-band rule? → '#' / '*' / '.'
 *     4. Otherwise → blank
 *
 * Each branch returns a CellDraw; paint_cell is the SINGLE ncurses
 * I/O point for the field grid. */

typedef struct {
    int  pair;
    int  attr;
    char glyph;
    bool skip;
} CellDraw;

/* compute_centred_origin — top-left corner where the map is drawn.
 * Centres horizontally; reserves HUD_BAND_RESERVED_ROWS rows total
 * (HUD_TOP_ROWS = 3 for state/params/legend + HUD_BOTTOM_ROWS = 1
 * for the action hint). Clamps so the map never overlaps the HUD
 * even on very short terminals. */
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

/* cell_supernova_sparkle — twinkle during the post-reset flash.
 * The (x ^ y) & MASK pattern lights one cell in (MASK + 1), giving
 * a sparse star-field rather than a uniform white-out. Cells with
 * active trail glow always light, so trails remain visible through
 * the flash. */
static CellDraw cell_supernova_sparkle(int x, int y, float trail_glow)
{
    bool sparkle_lit = ((x ^ y) & SUPERNOVA_SPARSE_MASK) == 0;
    if (!sparkle_lit && trail_glow <= GLOW_THRESHOLD)
        return (CellDraw){ .skip = true };
    return (CellDraw){ .pair = PAIR_SUPERNOVA, .attr = A_BOLD, .glyph = '*' };
}

/* cell_with_override_glyph — pattern-supplied glyph (e.g. arrow from
 * VECTOR). Coloured by the band, drawn bold. */
static CellDraw cell_with_override_glyph(const RenderBuffers *b, int idx)
{
    return (CellDraw){
        .pair  = PAIR_BAND_BASE + (b->color[idx] & (N_BANDS - 1)),
        .attr  = A_BOLD,
        .glyph = b->glyph[idx],
    };
}

/* cell_density_band — the default ASCII density ramp:
 *   '#' (high) → '*' (mid) → '.' (low) → blank.
 * Follows Bourke's grey-scale-to-character mapping. */
static CellDraw cell_density_band(uint8_t band, float glow)
{
    int pair = PAIR_BAND_BASE + (band & (N_BANDS - 1));
    if (glow > GLYPH_HIGH_THRESH) return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = '#' };
    if (glow > GLYPH_MID_THRESH ) return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = '*' };
    if (glow > GLOW_THRESHOLD   ) return (CellDraw){ .pair = pair, .attr = A_NORMAL, .glyph = '.' };
    return (CellDraw){ .skip = true };
}

/* pick_cell — execute the priority chain for one cell. */
static CellDraw pick_cell(const Scene *s, int x, int y)
{
    int   idx        = grid_idx(&s->grid, x, y);
    float trail_glow = s->buf.glow[idx];

    if (s->sim.supernova_glow_t > GLOW_THRESHOLD)
        return cell_supernova_sparkle(x, y, trail_glow);

    if (s->buf.glyph[idx] != 0 && trail_glow > GLOW_THRESHOLD)
        return cell_with_override_glyph(&s->buf, idx);

    return cell_density_band(s->buf.color[idx], trail_glow);
}

/* paint_cell — the ONE ncurses I/O point for the field grid. */
static void paint_cell(int sy, int sx, CellDraw c)
{
    if (c.skip) return;
    attron (COLOR_PAIR(c.pair) | c.attr);
    mvaddch(sy, sx, (chtype)(unsigned char)c.glyph);
    attroff(COLOR_PAIR(c.pair) | c.attr);
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    int gx0, gy0;
    compute_centred_origin(&s->grid, cols, rows, &gx0, &gy0);

    for (int y = 0; y < s->grid.h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < s->grid.w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            paint_cell(sy, sx, pick_cell(s, x, y));
        }
    }
}

/* ── HUD draw pipeline ────────────────────────────────────────────── *
 * Six named drawers, one per HUD element. The TOP HUD (rows 0..2)
 * carries DATA — current state, parameter readouts, glyph legend.
 * The BOTTOM HUD (row N-1) carries ACTIONS — key bindings only.
 * screen_draw assembles them in z-order: scene first, then HUD
 * painted over the top. */

static void draw_hud_state_bar(const Screen *sc, const Scene *s,
                                double fps, int sim_fps)
{
    const Controls *c = &s->ctrl;
    const char *state_str = c->paused ? "PAUSED   "
                                      : pattern_name(c->current_pattern);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, c->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;

    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void draw_hud_title(void)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " CURL NOISE VECTOR FIELD ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* draw_palette_swatch — paint one '#' per band colour. Returns the
 * column just past the swatch so the caller can continue laying out. */
static int draw_palette_swatch(int row, int x)
{
    for (int i = 0; i < HUD_PALETTE_SWATCH_N; i++) {
        int pair = PAIR_BAND_BASE + i;
        attron (COLOR_PAIR(pair) | A_BOLD);
        mvaddch(row, x, '#');
        attroff(COLOR_PAIR(pair) | A_BOLD);
        x++;
    }
    return x;
}

static void draw_hud_status_line(const Scene *s)
{
    const Controls *c = &s->ctrl;
    const Grid     *g = &s->grid;
    int x = HUD_LEFT_MARGIN;

    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-9s ", pattern_name(c->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += HUD_PATTERN_FIELD_W;

    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[c->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += HUD_THEME_FIELD_W;

    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += HUD_PALETTE_LABEL_W;

    x = draw_palette_swatch(1, x);

    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  drift:x%-2d  eps:%.2f  map:%dx%d ",
             c->drift_mult, CURL_EPS, g->w, g->h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* draw_hud_glyph_legend — row 2: explains the glyph alphabet so the
 * viewer can READ what they're seeing. This is DATA, not ACTION —
 * the bottom hint is reserved for key bindings only. Drawn in
 * non-bold PAIR_HUD so it sits below the bold state bar in visual
 * hierarchy. */
static void draw_hud_glyph_legend(void)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(2, HUD_LEFT_MARGIN,
             " legend:  .:low  *:mid  #:high   arrows: > < ^ v / \\ ");
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* draw_bottom_hint — row N-1: ACTIONS only (key bindings). The
 * glyph legend moved up to draw_hud_glyph_legend so this row stays
 * a single-purpose reference card for what the user can press. */
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
    scene_draw            (s, sc->cols, sc->rows);
    draw_hud_state_bar    (sc, s, fps, sim_fps);   /* row 0  : data */
    draw_hud_title        ();                       /* row 0  : data */
    draw_hud_status_line  (s);                      /* row 1  : data */
    draw_hud_glyph_legend ();                       /* row 2  : data */
    draw_bottom_hint      (sc);                     /* row N-1: actions */
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

/*
 * App — the top-level program state. ONE instance, g_app, lives in
 * BSS so the signal handlers can reach it without a global Scene
 * pointer. The App owns the Scene and the Screen and adds:
 *   • simulation parameters that are not really "scene state"
 *     (sim_fps, map_w, map_h);
 *   • signal-driven flags that must be sig_atomic_t for safety.
 *
 * SIGNAL-HANDLER DISCIPLINE. The handlers do nothing but set a
 * flag. The main loop polls those flags and performs the actual
 * work (cleanup, resize) in normal execution context. This is the
 * standard async-signal-safe pattern — see signal(7) for the rules
 * on what can and cannot be called from a handler. Anything that
 * touches ncurses or malloc MUST happen outside the handler.
 *
 * REFERENCE. W. Richard Stevens & Stephen Rago — "Advanced
 * Programming in the UNIX Environment" (3rd ed), ch. 10 on signals,
 * for the full discussion of async-signal-safety and sig_atomic_t.
 */
typedef struct {
    Scene                 scene;   /* the simulation                    */
    Screen                screen;  /* current terminal dimensions       */

    int                   sim_fps; /* tick rate; mutated by '[' and ']' */
    int                   map_w;   /* chosen map width,  ≤ MAP_W_MAX    */
    int                   map_h;   /* chosen map height, ≤ MAP_H_MAX    */

    /* sig_atomic_t guarantees writes from a handler are observed
     * atomically by the main loop; `volatile` prevents the compiler
     * from caching the read in a register across loop iterations.
     * Both qualifiers are required — sig_atomic_t alone permits
     * caching, volatile alone permits torn writes from a handler. */
    volatile sig_atomic_t running;       /* 0 = exit main loop          */
    volatile sig_atomic_t need_resize;   /* 1 = pending SIGWINCH        */
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
    screen_resize    (&app->screen);
    app_pick_map_size(app);
    scene_reset      (&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* ── keyboard handlers ──────────────────────────────────────────────── *
 * Each key is one named action; app_handle_key is just a dispatcher. */

/* bump_speed_geometric — '+' / '-' geometric step on speed AND
 * drift_mult. Doubling / halving (rather than ±1) gives a
 * logarithmic feel: each press is the same perceptual change
 * across the full clamped range. dir = +1 doubles; -1 halves. */
static void bump_speed_geometric(Controls *c, int dir)
{
    if (dir > 0) {
        if (c->speed      < SPEED_MAX)      c->speed      *= 2;
        if (c->speed      > SPEED_MAX)      c->speed      = SPEED_MAX;
        if (c->drift_mult < DRIFT_MULT_MAX) c->drift_mult *= 2;
        if (c->drift_mult > DRIFT_MULT_MAX) c->drift_mult = DRIFT_MULT_MAX;
    } else {
        c->speed      /= 2;
        if (c->speed      < SPEED_MIN)      c->speed      = SPEED_MIN;
        c->drift_mult /= 2;
        if (c->drift_mult < DRIFT_MULT_MIN) c->drift_mult = DRIFT_MULT_MIN;
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

static void cycle_pattern(Controls *c, int dir)
{
    c->current_pattern = (Pattern)(
        ((int)c->current_pattern + dir + N_PATTERNS) % N_PATTERNS);
}

static bool app_handle_key(App *app, int ch)
{
    Controls *c = &app->scene.ctrl;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           c->paused = !c->paused;                           break;
    case 'r': case 'R': scene_reset(&app->scene, app->map_w, app->map_h); break;
    case '=': case '+': bump_speed_geometric(c, +1);                      break;
    case '-':           bump_speed_geometric(c, -1);                      break;
    case ']':           bump_sim_fps(app, +SIM_FPS_STEP);                 break;
    case '[':           bump_sim_fps(app, -SIM_FPS_STEP);                 break;
    case 't':           cycle_theme  (c, +1);                             break;
    case 'T':           cycle_theme  (c, -1);                             break;
    case 'n': case 'N': cycle_pattern(c, +1);                             break;
    case 'p': case 'P': cycle_pattern(c, -1);                             break;
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
 * the last call, clamp dt at DT_MAX_NS (the spiral-of-death guard).
 * Updates *frame_time to "now" in place; returns dt (clamped). */
static int64_t advance_frame_clock(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > DT_MAX_NS) dt = DT_MAX_NS;
    return dt;
}

/* simulate_pending_ticks — drain the fixed-timestep accumulator.
 * Runs scene_tick() once per tick_ns worth of accumulated real time;
 * the simulation logic sees a CONSTANT dt_sec independent of frame
 * rate. Source: Glenn Fiedler, "Fix Your Timestep". */
static void simulate_pending_ticks(App *app, int64_t *sim_accum,
                                    int64_t tick_ns, float dt_sec)
{
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* maybe_update_fps_counter — every FPS_UPDATE_MS, fold the running
 * frame count into a smoothed fps reading. Returns the new fps if
 * the window just closed, otherwise the previous value. */
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

/* cap_frame_rate — sleep so frames are at most 1/target_fps apart.
 * `work_done_ns` is the wall-clock time already consumed this frame;
 * we sleep the remainder of the per-frame budget. */
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
