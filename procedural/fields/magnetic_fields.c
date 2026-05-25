/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * magnetic_fields.c
 *   — Magnetic field visualisation: iron-filings-style flow.
 *
 * DEMO: Hundreds of particles flow along magnetic field lines from N
 *       poles to S poles, tracing the iconic patterns you see when
 *       iron filings are sprinkled around a magnet. Pole positions
 *       are fixed by the active pattern; the field is the sum of
 *       inverse-square contributions from each pole. THIRTY
 *       configurations span the simple → complex spectrum, grouped
 *       roughly by pole count:
 *
 *         1–2 poles:  MONOPOLE DIPOLE DIPOLE_V REPELLER ATTRACT
 *         3–4 poles:  TRIPOLE TRIANGLE HORSHOE QUAD CROSS PINWHEEL
 *         5–8 poles:  CHAIN CHAIN_V HEXAPOLE TWIN_DIP STAR_5
 *                     OCTUPOLE MIRROR
 *         8+ poles :  NESTED SUNSPOT GRID3 RING_8 DBL_RING COIL
 *                     HELMHLTZ SOLENOID
 *         many     :  PLASMA MAZE AURORA CHAOS
 *
 *       Pole markers render as red 'N' / blue 'S' glyphs on top of
 *       the particle trails so polarity stays clear across all themes.
 *       The state bar shows [N/30] so you can see where you are in
 *       the cycle.
 *
 * Study alongside:
 *   ./flow_field_particles.c — same particle-flow architecture, but
 *       with abstract algebraic vector fields. Magnetic fields here
 *       are physically meaningful: every streamline is a real iron-
 *       filing path, every singular point is a magnetic monopole.
 *   ./curl_noise_vector_field.c — divergence-FREE flow. Magnetic
 *       fields are also divergence-free in nature (∇·B = 0, no
 *       monopoles), but our 2-D simulation uses signed monopoles
 *       as a convenient approximation; ∇·B ≠ 0 exactly at each pole.
 *
 * Section map:
 *   §1 config   — grid, particles, palette, themes, named constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes + N/S pole pairs
 *   §5 fields   — magnetic field math: B(r) = Σ qᵢ · r̂ᵢ / rᵢ²
 *   §6 patterns — pole-placement builders + 30 configurations
 *   §7 scene    — Scene struct composing Grid, RenderBuffers,
 *                 Particles, PoleField, SimState, Controls; per-frame
 *                 tick / particle-step pipeline
 *   §8 screen   — ASCII render: trails + N/S markers + HUD drawers
 *   §9 app      — signals, resize, named main-loop helpers
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset
 *   n / N      next pattern
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   g / G      next / previous glyph set (slim → fat)
 *   + / =      faster particles
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra magnetic_fields.c \
 *       -o magnetic -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Particle advection through a 2-D magnetic field. The
 *                  field at a point r is the sum of inverse-square
 *                  contributions from N magnetic poles:
 *
 *                      B(r) = Σᵢ qᵢ · (r − rᵢ) / |r − rᵢ|³
 *
 *                  where qᵢ = +1 for north poles, −1 for south, and rᵢ
 *                  is each pole's position. The field points OUT of
 *                  positive (north) poles and INTO negative (south)
 *                  poles; field lines are continuous arcs from N to S.
 *
 *                  Particles step along the local B direction, leaving
 *                  trails. After enough particles have flowed, the
 *                  trails reveal the FIELD LINES — the same pattern
 *                  iron filings make when sprinkled around a magnet.
 *
 *                  Each pattern is a different pole configuration
 *                  (number, positions, polarities). The simulation
 *                  itself is identical across patterns; only the pole
 *                  list changes.
 *
 *                  Note on physics: real magnetic fields have ∇·B = 0
 *                  everywhere (no magnetic monopoles). Our "monopole"
 *                  poles are a convenient 2-D approximation — at exact
 *                  pole positions, ∇·B blows up, but particle flow on
 *                  the rest of the field is accurate to the physical
 *                  picture.
 *
 * Data-structure : Scene is the umbrella context, composed of six
 *                  sub-structs so each layer has ONE clear role and
 *                  no globals leak across them:
 *                    Grid          — w, h, total_cells + grid_idx() /
 *                                    grid_in_bounds().
 *                    RenderBuffers — per-cell trail glow + colour
 *                                    band; the render contract between
 *                                    the particle layer (§7) and the
 *                                    screen layer (§8).
 *                    Particles     — fixed pool of advection walkers
 *                                    (pool[], n).
 *                    PoleField     — base list of Poles plus a
 *                                    per-frame "active" copy with
 *                                    drift applied. The magnetic
 *                                    SOURCE: field functions read
 *                                    `active`, the renderer reads
 *                                    `active` for N/S markers.
 *                    SimState      — field_time (only mutable per-
 *                                    tick scalar).
 *                    Controls      — paused/speed/theme/glyph/pattern;
 *                                    mutated only by the keyboard.
 *                  No heap; everything BSS.
 *
 * Rendering      : ASCII only. Density-graded particle trails in 4
 *                  theme palette colours. North poles render as red 'N'
 *                  glyphs, south poles as blue 'S' glyphs — colours
 *                  are theme-INDEPENDENT so polarity stays readable on
 *                  every theme.
 *
 * Performance    : Up to 24 pole evaluations per particle per frame
 *                  (CHAOS). With 256 particles at 60 Hz that's ~370 K
 *                  function calls/sec — essentially free on modern
 *                  hardware. Each pole evaluation is one sqrtf + 5
 *                  multiplies.
 *
 * References     : Algorithm / physics
 *                  ───────────────────
 *                  • Griffiths, D. J. — "Introduction to Electrodynamics"
 *                    (5th ed). The undergraduate classic; the inverse-
 *                    square superposition B = Σ qᵢ · r̂ / r² implemented
 *                    in §5 is straight out of ch. 5. Dipoles, multi-
 *                    poles, and field-line geometry all from this one
 *                    source.
 *                  • Purcell, E. M. & Morin, D. J. — "Electricity and
 *                    Magnetism" (3rd ed, Cambridge). The Berkeley
 *                    physics text — geometrically intuitive. Its
 *                    printed field-line figures look almost identical
 *                    to what this file produces on screen.
 *                  • Jackson, J. D. — "Classical Electrodynamics"
 *                    (3rd ed, Wiley). The graduate-level reference;
 *                    ch. 4 multipole expansion places DIPOLE / QUAD /
 *                    OCTUPOLE / DBL_RING etc. into a single expansion
 *                    of an arbitrary current distribution. Answers
 *                    "why does the 8-pole ring look like a high-order
 *                    multipole?" — because it IS one.
 *                  • Faraday, M. (1844) — "Experimental Researches in
 *                    Electricity". The HISTORICAL origin of both field
 *                    lines as a concept AND iron-filings visualisation
 *                    as a method. The particle trails in this file are
 *                    a direct descendant of Faraday's 19th-century
 *                    paper-and-magnet experiments.
 *                  • Chen, F. F. — "Introduction to Plasma Physics and
 *                    Controlled Fusion". For the engineering-physics
 *                    patterns: MIRROR (magnetic mirror confinement),
 *                    HELMHLTZ (uniform B for plasma experiments),
 *                    SOLENOID; sunspot magnetism (relevant to SUNSPOT)
 *                    gets a chapter too.
 *
 *                  Visualisation
 *                  ─────────────
 *                  • Helman, J. & Hesselink, L. (1991) — "Representation
 *                    and Display of Vector Field Topology in Fluid
 *                    Flows", IEEE Computer Graphics 11(3):36-46.
 *                    Direct ancestor of the marker-the-singularities
 *                    + particles-trace-the-flow approach used here.
 *                    Establishes the convention of explicitly drawing
 *                    critical points (our N / S markers) atop the
 *                    streamline density (our trail layer).
 *                  • Paul Bourke — "Character representation of grey
 *                    scale images":
 *                    https://paulbourke.net/dataformats/asciiart/
 *                    The canonical density → glyph ramp reference
 *                    underlying the trail-density tiers (low / mid /
 *                    high) and the five GlyphSet ramps in §1.
 *                  • Wikipedia — "Magnetic field":
 *                    https://en.wikipedia.org/wiki/Magnetic_field
 *                    Accessible starting point with interactive
 *                    diagrams of the standard configurations.
 *
 *                  See also
 *                  ────────
 *                  • Compare ./flow_field_particles.c — same particle
 *                    architecture, but with abstract algebraic vector
 *                    fields rather than physical magnetic ones. The
 *                    architectural overlap (Grid, RenderBuffers,
 *                    Particles, *Field, SimState, Controls) makes
 *                    cross-reading the two files instructive.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Sprinkle iron filings on paper above a magnet. Tap the paper. The
 * filings line up along invisible "field lines" connecting north and
 * south poles. Those lines aren't drawn anywhere — they emerge from
 * the local magnetic force on each filing. We do the same thing
 * computationally: drop particles, let each read the local B at its
 * position, step along that direction. After many particles flow,
 * the cumulative trails ARE the field lines.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Each pole is a "fountain" or a "drain" for the field:
 *   • N pole — field flows OUT in all directions (a fountain).
 *   • S pole — field flows IN from all directions (a drain).
 * When you have both a fountain and a drain on the same plane, the
 * fountain's water curves around to fall into the drain — that's the
 * dipole field-line pattern. Add more fountains and drains and the
 * curves get more elaborate; they NEVER cross (a physical impossibility
 * in 2-D since the field has a unique direction at every point).
 *
 * Visible layers:
 *   1. PARTICLE TRAILS — fading streamlines in 4 theme colours.
 *      Density-graded glyphs ('.', '*', '#') show flux density —
 *      bright cells are where many particles converge (near pole
 *      "drains") or diverge (around pole "fountains").
 *   2. POLE MARKERS — red 'N' and blue 'S' glyphs sitting at each
 *      pole's exact position, ALWAYS visible (rendered after trails).
 *   3. The streamlines curve smoothly. No straight lines except
 *      directly along the axis between two oppositely-charged poles.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Spawn 256 particles uniformly in-bounds. Initialise the
 *     active pattern's pole list (positions + polarities).
 *  2. PER FRAME:
 *     a. Slight orbital drift: each pole's position oscillates by a
 *        few cells around its base position, so the field evolves.
 *     b. For each particle:
 *        i.   Compute B(particle.pos) = Σ qᵢ · (r − rᵢ) / |r − rᵢ|³
 *        ii.  Normalise to unit direction (so visual speed is
 *             consistent regardless of the field's wildly varying
 *             magnitude near poles vs far from them).
 *        iii. Step by direction · speed · dt.
 *        iv.  Increment age. If age ≥ max_age OR position OOB,
 *             respawn at random in-bounds.
 *        v.   Paint trail at the new cell.
 *     c. Render trails first, then pole markers on top.
 *  3. The user can press 'r' to re-seed particles and reinstall the
 *     pattern's pole layout (re-randomising positions for PLASMA /
 *     CHAOS). There is no automatic reset — the field runs
 *     indefinitely until you ask for a new one.
 *
 * KEY FORMULAS
 * ────────────
 *  Single-pole field             :
 *    B_i(r) = qᵢ · (r − rᵢ) / |r − rᵢ|³
 *    (qᵢ = ±1 signed pole charge; r̂ from pole to query point)
 *  Total field                   : B(r) = Σᵢ B_i(r)
 *  Particle step                 :
 *    direction = B / |B|;  Δr = direction · speed · dt
 *  Softening                     :
 *    |r − rᵢ|² + ε  to avoid singularities at exact pole positions
 *  Pole drift                    :
 *    pole_pos = base_pos + (Δx, Δy) · cos/sin(t · ω + φ)
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

    MAX_PARTICLES     = 1024,
    N_PARTICLES_DEF   =  256,

    AGE_MIN_TICKS     =  60,
    AGE_MAX_TICKS     = 360,

    SPEED_MIN         =   1,
    SPEED_DEF         =   8,
    SPEED_MAX         =  64,

    MAX_POLES         =  32,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_TRAIL_BASE   =   3,    /* PAIR_TRAIL_BASE..+(N_BANDS-1) palette */
    PAIR_NORTH        =   7,    /* red 'N' marker — theme-independent   */
    PAIR_SOUTH        =   8,    /* blue 'S' marker — theme-independent  */
};

#define TRAIL_GLOW_DECAY    0.6f
#define GLOW_THRESHOLD      0.05f
#define VELOCITY_EPSILON    1e-6f   /* below this |B|, skip normalisation */
#define TRAIL_HIT_INTENSITY 1.0f    /* glow deposited where a particle lands */

/* Pole drift rate (rad/s) — slow oscillation of base positions so the
 * field evolves rather than freezing. */
#define DRIFT_RATE          0.2f

/* Number of palette bands per theme (quartile of trail colour). */
#define N_BANDS             4

/* Field strengths — tuned so particle motion is visible at SPEED_DEF.
 * Per-pattern values roughly scale inversely with pole count so the
 * net field magnitude stays in a comparable range across configurations. */

/* Tier 1 — simple (1–2 poles) */
#define MONOPOLE_STRENGTH   40.0f
#define DIPOLE_STRENGTH     50.0f
#define DIPOLE_V_STRENGTH   50.0f
#define REPELLER_STRENGTH   40.0f
#define ATTRACT_STRENGTH    40.0f

/* Tier 2 — few poles (3–4) */
#define TRIPOLE_STRENGTH    40.0f
#define TRIANGLE_STRENGTH   35.0f
#define HORSHOE_STRENGTH    45.0f
#define QUAD_STRENGTH       40.0f
#define CROSS_STRENGTH      35.0f
#define PINWHEEL_STRENGTH   25.0f

/* Tier 3 — medium (5–8 poles) */
#define CHAIN_STRENGTH      35.0f
#define CHAIN_V_STRENGTH    35.0f
#define HEXAPOLE_STRENGTH   30.0f
#define TWIN_DIP_STRENGTH   35.0f
#define STAR_5_STRENGTH     30.0f
#define OCTUPOLE_STRENGTH   25.0f
#define MIRROR_STRENGTH     40.0f

/* Tier 4 — structured complex (8+ poles) */
#define NESTED_STRENGTH     25.0f
#define SUNSPOT_STRENGTH    20.0f
#define GRID3_STRENGTH      25.0f
#define RING_8_STRENGTH     25.0f
#define DBL_RING_STRENGTH   20.0f
#define COIL_STRENGTH       18.0f
#define HELMHLTZ_STRENGTH   20.0f
#define SOLENOID_STRENGTH   20.0f

/* Tier 5 — chaotic / dense */
#define PLASMA_STRENGTH     25.0f
#define MAZE_STRENGTH       20.0f
#define AURORA_STRENGTH     18.0f
#define CHAOS_STRENGTH      18.0f

#define EPS_R2              4.0f    /* softening — avoid 1/0 at pole centres */

#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/* ── HUD layout ──────────────────────────────────────────────────── *
 * Top HUD carries DATA, bottom HUD carries ACTIONS:
 *   row 0           : title + state bar (fps, Hz, state + [N/M], speed)
 *   row 1           : pattern/theme/palette/N:S counts/parts/map +
 *                     glyph indicator right
 *   row 2           : legend (trails / N / S glyph meanings)
 *   row HUD_TOP..N-2: playable field
 *   row N-1         : keyboard action hint
 */
#define HUD_TOP_ROWS             3
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1
#define HUD_PATTERN_FIELD_W      20
#define HUD_THEME_FIELD_W        17
#define HUD_PALETTE_LABEL_W       9

/*
 * Pattern — 30 magnetic-pole configurations spanning simple → complex.
 *
 * The simulation kernel (particle advection through B = Σ qᵢ · r̂ᵢ / rᵢ²)
 * is identical for all patterns; only the pole list — count, positions,
 * polarities — changes. Cycle with n/p.
 */
typedef enum {
    /* Tier 1 — simple (1–2 poles) */
    PATTERN_MONOPOLE = 0,
    PATTERN_DIPOLE   = 1,
    PATTERN_DIPOLE_V = 2,
    PATTERN_REPELLER = 3,
    PATTERN_ATTRACT  = 4,

    /* Tier 2 — few poles (3–4) */
    PATTERN_TRIPOLE  = 5,
    PATTERN_TRIANGLE = 6,
    PATTERN_HORSHOE  = 7,
    PATTERN_QUAD     = 8,
    PATTERN_CROSS    = 9,
    PATTERN_PINWHEEL = 10,

    /* Tier 3 — medium (5–8 poles) */
    PATTERN_CHAIN    = 11,
    PATTERN_CHAIN_V  = 12,
    PATTERN_HEXAPOLE = 13,
    PATTERN_TWIN_DIP = 14,
    PATTERN_STAR_5   = 15,
    PATTERN_OCTUPOLE = 16,
    PATTERN_MIRROR   = 17,

    /* Tier 4 — structured complex (8+ poles) */
    PATTERN_NESTED   = 18,
    PATTERN_SUNSPOT  = 19,
    PATTERN_GRID3    = 20,
    PATTERN_RING_8   = 21,
    PATTERN_DBL_RING = 22,
    PATTERN_COIL     = 23,
    PATTERN_HELMHLTZ = 24,
    PATTERN_SOLENOID = 25,

    /* Tier 5 — chaotic / dense */
    PATTERN_PLASMA   = 26,
    PATTERN_MAZE     = 27,
    PATTERN_AURORA   = 28,
    PATTERN_CHAOS    = 29,

    N_PATTERNS       = 30,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_MONOPOLE: return "MONOPOLE";
    case PATTERN_DIPOLE:   return "DIPOLE  ";
    case PATTERN_DIPOLE_V: return "DIPOLE_V";
    case PATTERN_REPELLER: return "REPELLER";
    case PATTERN_ATTRACT:  return "ATTRACT ";
    case PATTERN_TRIPOLE:  return "TRIPOLE ";
    case PATTERN_TRIANGLE: return "TRIANGLE";
    case PATTERN_HORSHOE:  return "HORSHOE ";
    case PATTERN_QUAD:     return "QUAD    ";
    case PATTERN_CROSS:    return "CROSS   ";
    case PATTERN_PINWHEEL: return "PINWHEEL";
    case PATTERN_CHAIN:    return "CHAIN   ";
    case PATTERN_CHAIN_V:  return "CHAIN_V ";
    case PATTERN_HEXAPOLE: return "HEXAPOLE";
    case PATTERN_TWIN_DIP: return "TWIN_DIP";
    case PATTERN_STAR_5:   return "STAR_5  ";
    case PATTERN_OCTUPOLE: return "OCTUPOLE";
    case PATTERN_MIRROR:   return "MIRROR  ";
    case PATTERN_NESTED:   return "NESTED  ";
    case PATTERN_SUNSPOT:  return "SUNSPOT ";
    case PATTERN_GRID3:    return "GRID3   ";
    case PATTERN_RING_8:   return "RING_8  ";
    case PATTERN_DBL_RING: return "DBL_RING";
    case PATTERN_COIL:     return "COIL    ";
    case PATTERN_HELMHLTZ: return "HELMHLTZ";
    case PATTERN_SOLENOID: return "SOLENOID";
    case PATTERN_PLASMA:   return "PLASMA  ";
    case PATTERN_MAZE:     return "MAZE    ";
    case PATTERN_AURORA:   return "AURORA  ";
    case PATTERN_CHAOS:    return "CHAOS   ";
    default:               return "?       ";
    }
}

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* §9 main loop — spiral-of-death dt clamp + frame rate cap (Glenn
 * Fiedler's "Fix Your Timestep"). */
#define DT_MAX_NS       (100 * NS_PER_MS)
#define FRAME_CAP_FPS   60

/*
 * Theme — one complete colour palette for the trail layer. Ten of
 * these live in themes[], cycled by t/T.
 *
 * INTENT. The smallest data needed to recolour every trail cell:
 * N_BANDS = 4 ramp colours arranged dim → bright, plus a "flash"
 * accent (vestigial here — flash is unused in this file but kept
 * for cross-file palette parity with curl_noise_vector_field.c /
 * domain_warped_noise_iq_style.c / flow_field_particles.c).
 *
 * BANDING MODEL. Each particle gets a stable color_idx ∈ [0, N_BANDS)
 * at spawn; the deposited trail carries that band as the cell's
 * palette index. The four-band split forces theme authors to design
 * the ramp as a coherent low → high progression — same discipline
 * as a Houdini ramp parameter or Substance Designer gradient.
 *
 * UI CHROME. PAIR_NORTH (red) and PAIR_SOUTH (blue) stay theme-
 * INDEPENDENT so the 'N' / 'S' pole markers always read clearly
 * regardless of the active palette. The standard physics-textbook
 * red-north / blue-south convention is preserved across every theme.
 *
 * COLOUR FORMAT. xterm-256 indices (NOT RGB). When the terminal
 * exposes fewer than 256 colours, theme_apply() substitutes a fixed
 * 8-colour cycle so the demo still runs on legacy TTYs.
 */
typedef struct {
    const char *name;             /* short uppercase label shown in HUD       */
    short       trail[N_BANDS];   /* ramp colours: 0 = dim/low, 3 = bright    */
    short       flash;            /* vestigial here — kept for cross-file parity */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      trail0 trail1 trail2 trail3 flash */
    { "DEFAULT", {  33,  117,  220,  220 }, 226 },
    { "MATRIX",  {  22,   34,   46,  118 }, 226 },
    { "NOVA",    {  53,  129,  201,  219 }, 226 },
    { "MONO",    { 240,  244,  250,  254 }, 226 },
    { "OCEAN",   {  17,   33,   39,   51 }, 226 },
    { "FIRE",    {  88,  124,  208,  226 }, 196 },
    { "EARTH",   {  58,  100,  173,  230 }, 226 },
    { "FOREST",  {  22,   28,   64,  144 }, 226 },
    { "DESERT",  {  94,  130,  173,  222 }, 226 },
    { "ARCTIC",  {  18,   39,  159,  231 }, 226 },
};

/*
 * GlyphSet — three ASCII characters representing low / mid / high
 * trail density. Five sets exist (SLIM → FAT thickness ramp); cycle
 * with g/G.
 *
 * INTENT. Different themes want different glyph weights to read
 * cleanly. MONO (greyscale) looks better with the fatter HEAVY ramp;
 * MATRIX looks more "wireframe" with SLIM. Decoupling glyph choice
 * from theme choice lets the user pair them freely without coding
 * up theme × ramp combinations.
 *
 * Three tiers map to the three GLYPH_*_THRESH cutoffs in §1:
 *   low  : glow > GLOW_THRESHOLD   (faintest visible)
 *   mid  : glow > GLYPH_MID_THRESH
 *   high : glow > GLYPH_HIGH_THRESH (densest streamline cells)
 *
 * The principle behind density → glyph mapping is Paul Bourke's
 * ASCII grey-scale ramp; we use a coarser 3-step ramp here because
 * the field rendering is monotonic-direction (particles flow ALONG
 * streamlines), and finer ramps would just trail more visual noise.
 */
typedef struct {
    const char *name;       /* short label shown in HUD glyph indicator */
    char        low;        /* glow > GLOW_THRESHOLD                    */
    char        mid;        /* glow > GLYPH_MID_THRESH                  */
    char        high;       /* glow > GLYPH_HIGH_THRESH                 */
} GlyphSet;

#define N_GLYPH_SETS 5

static const GlyphSet glyph_sets[N_GLYPH_SETS] = {
    { "SLIM",    '.', '\'', ':' },
    { "LIGHT",   '.', '*',  '+' },
    { "MEDIUM",  '.', '*',  '#' },
    { "HEAVY",   'o', 'O',  '@' },
    { "FAT",     '+', '#',  'M' },
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
    } else {
        static const short fallback[N_BANDS] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < N_BANDS; i++)
            init_pair(PAIR_TRAIL_BASE + i, fallback[i], -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
        /* N = red, S = blue — physics convention. Theme-independent so
         * polarity is unambiguous on every theme. */
        init_pair(PAIR_NORTH,      196, -1);   /* hot red    */
        init_pair(PAIR_SOUTH,       33, -1);   /* clear blue */
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,   -1);
        init_pair(PAIR_NORTH,     COLOR_RED,    -1);
        init_pair(PAIR_SOUTH,     COLOR_BLUE,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  fields — magnetic field math                                       */
/* ===================================================================== */

/*
 * Pole — one magnetic monopole. The fundamental physical entity
 * generating the field. Static description: a fixed centre plus the
 * parameters of its slow drift orbit. Never mutated per tick; the
 * drift is applied each frame into ActivePole (below).
 *
 * PHYSICS NOTE. Real magnetic fields have ∇·B = 0 everywhere (no
 * monopoles exist — Maxwell's 2nd equation). Our "Pole" is a 2-D
 * approximation: a point source of B with sign ±1 representing the
 * tip of an idealised long bar magnet seen end-on. The field at
 * (x, y) from one pole is:
 *
 *     B_i(r) = qᵢ · (r − rᵢ) / |r − rᵢ|³        [Griffiths ch. 2/5]
 *
 * where qᵢ = ±1 is the polarity (charge in the analogous E-field).
 * Field flows OUT of polarity +1 (north / source) and INTO polarity
 * −1 (south / sink). Multi-pole patterns superpose via Σ B_i.
 *
 * DRIFT ORBIT. The (ox, oy, phase) fields parameterise a small
 * Lissajous orbit around (bx, by). Live position each frame:
 *   x = bx + ox · cos(t · DRIFT_RATE + phase)
 *   y = by + oy · sin(t · DRIFT_RATE + phase)
 * Independent orbits (different ox/oy ratios + per-pole phases)
 * keep multi-pole patterns visually alive rather than rigidly locked.
 *
 * HISTORICAL. Field-line visualisation by iron filings around real
 * magnets — and the FIELD LINE concept itself — was introduced by
 * Faraday (1844). This file's particle trails are a direct
 * descendant of those 19th-century experiments.
 */
typedef struct {
    float bx, by;       /* base position — centre of the drift orbit       */
    float ox, oy;       /* orbital radii — Lissajous ellipse semi-axes     */
    float phase;        /* orbital phase φ so poles drift independently    */
    float strength;     /* multiplier S; in single-pattern world ≈ |q|     */
    int   polarity;     /* +1 = N (field OUT), −1 = S (field IN)           */
} Pole;

/*
 * ActivePole — one frame's evaluated pole, drift applied.
 *
 * INTENT. Classic graphics-pipeline PRECOMPUTE PATTERN: cos/sin trig
 * runs ONCE per frame (in compute_active_poles, ~30 calls) rather than
 * inside field_at where it would fire per cell × per pole (~256 × 30
 * per frame). The slimmer struct also tightens the field_at signature
 * — it only needs the live position, strength, polarity, not the
 * orbital parameters.
 */
typedef struct {
    float x, y;
    float strength;
    int   polarity;
} ActivePole;

/*
 * field_at — evaluate B(x, y) for the given list of poles.
 *
 *   B = Σᵢ qᵢ · (r − rᵢ) / |r − rᵢ|³
 *
 * Implementation: for each pole, compute Δr = (x − pole.x, y − pole.y),
 * its squared length r² + ε (softening), and r³. The contribution
 * is Δr · (q · S / r³), summed.
 */
static void field_at(const ActivePole *poles, int n_poles,
                     float x, float y,
                     float *out_bx, float *out_by)
{
    float bx = 0.0f, by = 0.0f;
    for (int i = 0; i < n_poles; i++) {
        float dx = x - poles[i].x;
        float dy = y - poles[i].y;
        float r2 = dx * dx + dy * dy + EPS_R2;
        float r3 = r2 * sqrtf(r2);
        float w  = (float)poles[i].polarity * poles[i].strength / r3;
        bx += dx * w;
        by += dy * w;
    }
    *out_bx = bx;
    *out_by = by;
}

/* ===================================================================== */
/* §6  patterns — 30 pole configurations                                  */
/* ===================================================================== */

/* ── pole-placement builders ─────────────────────────────────────── *
 * Small named factories used by every pattern's setup. Building
 * blocks: one pole, a chain along a line, a ring on an ellipse, a
 * grid, a random scatter. Each accepts an out[] buffer plus a
 * pointer-to-n cursor so multiple builders can compose. */

static Pole make_pole(float bx, float by, float ox, float oy,
                       float phase, float strength, int polarity)
{
    return (Pole){
        .bx = bx, .by = by, .ox = ox, .oy = oy,
        .phase = phase, .strength = strength, .polarity = polarity,
    };
}

/* place_chain_along — `count` alternating-polarity poles equispaced
 * from (x0, y0) to (x1, y1). First pole gets polarity start_sign;
 * subsequent flip. */
static void place_chain_along(Pole out[], int max_n, int *n,
                               float x0, float y0, float x1, float y1,
                               int count, int start_sign,
                               float strength, float ox, float oy)
{
    for (int i = 0; i < count && *n < max_n; i++) {
        float t  = (count > 1) ? (float)i / (float)(count - 1) : 0.5f;
        float bx = x0 + (x1 - x0) * t;
        float by = y0 + (y1 - y0) * t;
        int   pol = (i & 1) ? -start_sign : start_sign;
        out[(*n)++] = make_pole(bx, by, ox, oy,
                                 (float)i * 0.4f, strength, pol);
    }
}

/* place_ring — `count` poles equispaced on an ellipse centred at
 * (cx, cy) with semi-axes (rx, ry). If `alternate` is non-zero,
 * polarity alternates starting with start_sign; else all start_sign. */
static void place_ring(Pole out[], int max_n, int *n,
                        float cx, float cy, float rx, float ry,
                        int count, int alternate, int start_sign,
                        float strength, float ox, float oy)
{
    for (int i = 0; i < count && *n < max_n; i++) {
        float ang = (float)i * 2.0f * (float)M_PI / (float)count;
        int   pol = alternate ? ((i & 1) ? -start_sign : start_sign)
                              : start_sign;
        out[(*n)++] = make_pole(cx + rx * cosf(ang),
                                  cy + ry * sinf(ang),
                                  ox, oy, ang, strength, pol);
    }
}

/* place_grid — rows × cols checkerboard centred at (cx, cy), step
 * (dx, dy). Polarity follows (gx + gy) parity. */
static void place_grid(Pole out[], int max_n, int *n,
                        float cx, float cy, float dx, float dy,
                        int rows, int cols, float strength)
{
    int k = 0;
    for (int gy = 0; gy < rows; gy++) {
        for (int gx = 0; gx < cols; gx++) {
            if (*n >= max_n) return;
            float bx = cx + ((float)gx - (float)(cols - 1) * 0.5f) * dx;
            float by = cy + ((float)gy - (float)(rows - 1) * 0.5f) * dy;
            int   pol = ((gx + gy) & 1) ? -1 : +1;
            out[(*n)++] = make_pole(bx, by, 1.0f, 1.0f,
                                      (float)k * 0.3f, strength, pol);
            k++;
        }
    }
}

/* place_random — `count` random poles with random polarity, bounded
 * to a margin inside the map. Optional ±jitter on strength. */
static void place_random(Pole out[], int max_n, int *n,
                          int w, int h, int count, float strength,
                          float strength_jitter)
{
    int margin_x = w / 10;
    int margin_y = h / 8;
    for (int i = 0; i < count && *n < max_n; i++) {
        float px = (float)(margin_x + rand() % (w - 2 * margin_x));
        float py = (float)(margin_y + rand() % (h - 2 * margin_y));
        float s  = strength;
        if (strength_jitter > 0.0f) {
            float r = (float)(rand() % 200 - 100) / 100.0f;   /* −1..+1 */
            s *= (1.0f + r * strength_jitter);
        }
        out[(*n)++] = make_pole(px, py, 1.0f, 1.0f,
                                  (float)i * 0.31f, s,
                                  (rand() & 1) ? -1 : +1);
    }
}

/* ── per-pattern pole placers ────────────────────────────────────── *
 * One function per pattern. Each knows the pole layout for ONE
 * configuration and returns the count of poles written. The
 * dispatcher pattern_init_poles below is then one line per case.
 *
 * Names follow the physics: dipole, repeller, quadrupole, octupole,
 * magnetic-mirror, Helmholtz, solenoid, sunspot, current-loop. */

/* ── Tier 1 — simple (1–2 poles) ─────────────────────────────── */

/* MONOPOLE — single N at the centre. The simplest possible source.
 * Radiates outward in all directions (∇·B ≠ 0 here — the textbook
 * "fictional" monopole). */
static int place_monopole_centre(Pole out[], int max_n, float cx, float cy)
{
    if (max_n < 1) return 0;
    out[0] = make_pole(cx, cy, 1.0f, 1.0f, 0.0f,
                        MONOPOLE_STRENGTH, +1);
    return 1;
}

/* DIPOLE — classic horizontal bar magnet (N on left, S on right). */
static int place_dipole_horizontal(Pole out[], int max_n,
                                    float cx, float cy, int w)
{
    if (max_n < 2) return 0;
    float dx = (float)w * 0.20f;
    out[0] = make_pole(cx - dx, cy, 3.0f, 2.0f, 0.0f,
                        DIPOLE_STRENGTH, +1);
    out[1] = make_pole(cx + dx, cy, 3.0f, 2.0f, (float)M_PI,
                        DIPOLE_STRENGTH, -1);
    return 2;
}

/* DIPOLE_V — vertical dipole (N above, S below). Same field rotated 90°. */
static int place_dipole_vertical(Pole out[], int max_n,
                                  float cx, float cy, int h)
{
    if (max_n < 2) return 0;
    float dy = (float)h * 0.25f;
    out[0] = make_pole(cx, cy - dy, 2.0f, 3.0f, 0.0f,
                        DIPOLE_V_STRENGTH, +1);
    out[1] = make_pole(cx, cy + dy, 2.0f, 3.0f, (float)M_PI,
                        DIPOLE_V_STRENGTH, -1);
    return 2;
}

/* REPELLER — two N's close together. Flow is pushed AWAY from both,
 * meets at the midpoint, then escapes along the perpendicular axis. */
static int place_repeller_pair(Pole out[], int max_n,
                                float cx, float cy, int w)
{
    if (max_n < 2) return 0;
    float dx = (float)w * 0.12f;
    out[0] = make_pole(cx - dx, cy, 2.0f, 2.0f, 0.0f,
                        REPELLER_STRENGTH, +1);
    out[1] = make_pole(cx + dx, cy, 2.0f, 2.0f, (float)M_PI,
                        REPELLER_STRENGTH, +1);
    return 2;
}

/* ATTRACT — two S's close together. Flow converges INTO both from
 * infinity, splits at the midpoint. Topological inverse of REPELLER. */
static int place_attract_pair(Pole out[], int max_n,
                               float cx, float cy, int w)
{
    if (max_n < 2) return 0;
    float dx = (float)w * 0.12f;
    out[0] = make_pole(cx - dx, cy, 2.0f, 2.0f, 0.0f,
                        ATTRACT_STRENGTH, -1);
    out[1] = make_pole(cx + dx, cy, 2.0f, 2.0f, (float)M_PI,
                        ATTRACT_STRENGTH, -1);
    return 2;
}

/* ── Tier 2 — few poles (3–4) ────────────────────────────────── */

/* TRIPOLE — N-S-N in a horizontal line. Outer N's both feed the
 * middle S — the field has two dipole loops sharing the central sink. */
static int place_tripole_line(Pole out[], int max_n,
                               float cx, float cy, int w)
{
    if (max_n < 3) return 0;
    float sp = (float)w * 0.18f;
    for (int i = 0; i < 3; i++) {
        out[i] = make_pole(cx + ((float)i - 1.0f) * sp, cy,
                            2.0f, 1.5f, (float)i * 0.5f,
                            TRIPOLE_STRENGTH,
                            (i & 1) ? -1 : +1);
    }
    return 3;
}

/* TRIANGLE — N at top + two S's at bottom corners. Field lines fan
 * down from the N to both S's, forming a symmetric "ribbon" pair. */
static int place_triangle_1n2s(Pole out[], int max_n,
                                float cx, float cy, int h)
{
    if (max_n < 3) return 0;
    float r = (float)h * 0.30f;
    for (int i = 0; i < 3; i++) {
        float ang = (float)i * 2.0f * (float)M_PI / 3.0f
                  - (float)M_PI * 0.5f;
        out[i] = make_pole(cx + r * cosf(ang),
                            cy + r * sinf(ang),
                            1.5f, 1.5f, ang,
                            TRIANGLE_STRENGTH,
                            (i == 0) ? +1 : -1);
    }
    return 3;
}

/* HORSESHOE — magnet tips: N and S close together near the top of
 * the map. The dense, tight field-line arc between them is the
 * defining visual signature of a horseshoe magnet. */
static int place_horseshoe_tips(Pole out[], int max_n,
                                 float cx, float cy, int w, int h)
{
    if (max_n < 2) return 0;
    float gap = (float)w * 0.06f;
    float yp  = cy - (float)h * 0.15f;
    out[0] = make_pole(cx - gap, yp, 1.0f, 1.0f, 0.0f,
                        HORSHOE_STRENGTH, +1);
    out[1] = make_pole(cx + gap, yp, 1.0f, 1.0f, (float)M_PI,
                        HORSHOE_STRENGTH, -1);
    return 2;
}

/* QUAD — 4 alternating-polarity poles at corners of an inner square.
 * The canonical quadrupole — first non-trivial term in the multipole
 * expansion after dipole (Jackson ch. 4). */
static int place_quadrupole_corners(Pole out[], int max_n,
                                     float cx, float cy, int w, int h)
{
    if (max_n < 4) return 0;
    float ax = (float)w * 0.18f;
    float ay = (float)h * 0.22f;
    const struct { float dx, dy; int sign; } cfg[4] = {
        {-ax, -ay, +1}, {+ax, -ay, -1},
        {-ax, +ay, -1}, {+ax, +ay, +1},
    };
    for (int i = 0; i < 4; i++) {
        out[i] = make_pole(cx + cfg[i].dx, cy + cfg[i].dy,
                            2.0f, 2.0f, (float)i * 0.5f,
                            QUAD_STRENGTH, cfg[i].sign);
    }
    return 4;
}

/* CROSS — 4 alternating poles in a + shape (top, right, bottom, left).
 * Quadrupolar topology but rotated 45° from QUAD. */
static int place_quadrupole_cross(Pole out[], int max_n,
                                   float cx, float cy, int w, int h)
{
    if (max_n < 4) return 0;
    float r = (float)h * 0.28f;
    float aspect = (float)h / (float)w;
    const float xs[4] = { cx, cx + r / aspect, cx, cx - r / aspect };
    const float ys[4] = { cy - r, cy, cy + r, cy };
    const int   sg[4] = { +1, -1, +1, -1 };
    for (int i = 0; i < 4; i++) {
        out[i] = make_pole(xs[i], ys[i], 1.5f, 1.5f,
                            (float)i * 0.4f,
                            CROSS_STRENGTH, sg[i]);
    }
    return 4;
}

/* PINWHEEL — 4 small dipoles arranged tangentially around the
 * centre. The combined field shows a clear ROTATIONAL pattern,
 * like looking at a spinning windmill from above. 8 poles total. */
static int place_pinwheel_dipoles(Pole out[], int max_n,
                                   float cx, float cy, int h)
{
    if (max_n < 8) return 0;
    float r   = (float)h * 0.22f;
    float gap = (float)h * 0.06f;
    int n = 0;
    for (int i = 0; i < 4; i++) {
        float ang = (float)i * (float)M_PI * 0.5f;
        float tx  = -sinf(ang);                   /* tangent direction  */
        float ty  =  cosf(ang);
        float ix  = cx + r * cosf(ang);           /* dipole-pair centre */
        float iy  = cy + r * sinf(ang);
        /* N offset along tangent (+), S opposite (−) — the dipole's
         * axis is perpendicular to the radius, giving rotational flow. */
        out[n++] = make_pole(ix + gap * tx, iy + gap * ty,
                              1.0f, 1.0f, ang,
                              PINWHEEL_STRENGTH, +1);
        out[n++] = make_pole(ix - gap * tx, iy - gap * ty,
                              1.0f, 1.0f, ang + (float)M_PI,
                              PINWHEEL_STRENGTH, -1);
    }
    return n;
}

/* ── Tier 3 — medium (5–8 poles) ─────────────────────────────── */

/* CHAIN — 6 alternating poles in a horizontal line. */
static int place_chain_horizontal(Pole out[], int max_n,
                                   float cx, float cy, int w)
{
    int n = 0;
    place_chain_along(out, max_n, &n,
                      cx - (float)w * 0.35f, cy,
                      cx + (float)w * 0.35f, cy,
                      6, +1, CHAIN_STRENGTH, 2.0f, 1.0f);
    return n;
}

/* CHAIN_V — 6 alternating poles in a vertical column. */
static int place_chain_vertical(Pole out[], int max_n,
                                 float cx, float cy, int h)
{
    int n = 0;
    place_chain_along(out, max_n, &n,
                      cx, cy - (float)h * 0.40f,
                      cx, cy + (float)h * 0.40f,
                      6, +1, CHAIN_V_STRENGTH, 1.0f, 2.0f);
    return n;
}

/* HEXAPOLE — 6 alternating poles around a hexagon. */
static int place_hexapole_ring(Pole out[], int max_n,
                                float cx, float cy, int w, int h)
{
    int n = 0;
    place_ring(out, max_n, &n, cx, cy,
               (float)w * 0.25f, (float)h * 0.28f,
               6, /*alternate*/1, +1, HEXAPOLE_STRENGTH, 1.5f, 1.5f);
    return n;
}

/* TWIN_DIP — two horizontal dipoles, one above the other. */
static int place_twin_dipole(Pole out[], int max_n,
                              float cx, float cy, int w, int h)
{
    if (max_n < 4) return 0;
    float dx = (float)w * 0.20f;
    float dy = (float)h * 0.18f;
    out[0] = make_pole(cx - dx, cy - dy, 2.0f, 1.0f,
                        0.0f, TWIN_DIP_STRENGTH, +1);
    out[1] = make_pole(cx + dx, cy - dy, 2.0f, 1.0f,
                        (float)M_PI, TWIN_DIP_STRENGTH, -1);
    out[2] = make_pole(cx - dx, cy + dy, 2.0f, 1.0f,
                        0.5f, TWIN_DIP_STRENGTH, +1);
    out[3] = make_pole(cx + dx, cy + dy, 2.0f, 1.0f,
                        (float)M_PI + 0.5f, TWIN_DIP_STRENGTH, -1);
    return 4;
}

/* STAR_5 — 5 alternating poles on a pentagon. Odd count means
 * polarities alternate NSNSN — the asymmetry creates a visually
 * interesting flow with one "extra" sink/source pair. */
static int place_pentapole_ring(Pole out[], int max_n,
                                 float cx, float cy, int w, int h)
{
    int n = 0;
    place_ring(out, max_n, &n, cx, cy,
               (float)w * 0.27f, (float)h * 0.30f,
               5, /*alternate*/1, +1, STAR_5_STRENGTH, 1.5f, 1.5f);
    return n;
}

/* OCTUPOLE — 8 alternating poles around a ring. The next-higher
 * multipole moment after quadrupole; the field falls off faster at
 * large distances (Jackson ch. 4). */
static int place_octupole_ring(Pole out[], int max_n,
                                float cx, float cy, int w, int h)
{
    int n = 0;
    place_ring(out, max_n, &n, cx, cy,
               (float)w * 0.28f, (float)h * 0.32f,
               8, /*alternate*/1, +1, OCTUPOLE_STRENGTH, 1.5f, 1.5f);
    return n;
}

/* MIRROR — magnetic mirror: two same-sign N's on the y-axis. Field
 * lines curve out from each, meet in the middle, then return —
 * the topology used in plasma-confinement traps (Chen ch. 2). */
static int place_magnetic_mirror(Pole out[], int max_n,
                                  float cx, float cy, int h)
{
    if (max_n < 2) return 0;
    float dy = (float)h * 0.32f;
    out[0] = make_pole(cx, cy - dy, 1.0f, 2.0f, 0.0f,
                        MIRROR_STRENGTH, +1);
    out[1] = make_pole(cx, cy + dy, 1.0f, 2.0f, (float)M_PI,
                        MIRROR_STRENGTH, +1);
    return 2;
}

/* ── Tier 4 — structured complex (8+ poles) ──────────────────── */

/* NESTED — inner square of 4 alternating + outer square of 4
 * alternating with INVERTED polarity. Each inner-outer corner pair
 * forms a radial dipole. 8 poles total. */
static int place_nested_squares(Pole out[], int max_n,
                                 float cx, float cy, int w, int h)
{
    if (max_n < 8) return 0;
    float ix = (float)w * 0.10f, iy = (float)h * 0.12f;
    float ox = (float)w * 0.25f, oy = (float)h * 0.30f;
    const struct { float dx, dy; int sign_in, sign_out; } cfg[4] = {
        {-1, -1, +1, -1}, {+1, -1, -1, +1},
        {-1, +1, -1, +1}, {+1, +1, +1, -1},
    };
    int n = 0;
    for (int i = 0; i < 4; i++) {
        out[n++] = make_pole(cx + cfg[i].dx * ix, cy + cfg[i].dy * iy,
                              1.0f, 1.0f, (float)i * 0.4f,
                              NESTED_STRENGTH, cfg[i].sign_in);
        out[n++] = make_pole(cx + cfg[i].dx * ox, cy + cfg[i].dy * oy,
                              1.0f, 1.0f, (float)i * 0.4f + 0.3f,
                              NESTED_STRENGTH, cfg[i].sign_out);
    }
    return n;
}

/* SUNSPOT — two sunspot regions, each a tight bipolar cluster of
 * 3 N's + 3 S's. Models photospheric flux emergence / return (Chen
 * ch. on solar magnetism). 12 poles total. */
static int place_sunspot_clusters(Pole out[], int max_n,
                                   float cx, float cy, int w)
{
    if (max_n < 12) return 0;
    float dxc = (float)w * 0.22f;
    float sp  = 2.5f;
    const float xs[2] = { cx - dxc, cx + dxc };
    int n = 0;
    for (int s = 0; s < 2; s++) {
        int sign_lr = (s == 0) ? +1 : -1;
        for (int i = 0; i < 3; i++) {
            float dy_i = ((float)i - 1.0f) * sp;
            out[n++] = make_pole(xs[s] - sp, cy + dy_i,
                                   1.0f, 1.0f, (float)(s * 3 + i) * 0.4f,
                                   SUNSPOT_STRENGTH,  sign_lr);
            out[n++] = make_pole(xs[s] + sp, cy + dy_i,
                                   1.0f, 1.0f, (float)(s * 3 + i) * 0.4f + 1.0f,
                                   SUNSPOT_STRENGTH, -sign_lr);
        }
    }
    return n;
}

/* GRID3 — 3×3 checkerboard of alternating poles. 9 poles total. */
static int place_grid_3x3(Pole out[], int max_n,
                           float cx, float cy, int w, int h)
{
    int n = 0;
    place_grid(out, max_n, &n, cx, cy,
               (float)w * 0.22f, (float)h * 0.24f, 3, 3, GRID3_STRENGTH);
    return n;
}

/* RING_8 — 8 same-sign N's on a circle. Radial-outflow "sunburst"
 * with curved arms where neighbours' fields interact. */
static int place_ring8_samesign(Pole out[], int max_n,
                                 float cx, float cy, int w, int h)
{
    int n = 0;
    place_ring(out, max_n, &n, cx, cy,
               (float)w * 0.28f, (float)h * 0.32f,
               8, /*alternate*/0, +1, RING_8_STRENGTH, 1.5f, 1.5f);
    return n;
}

/* DBL_RING — inner ring of N's + outer ring of S's. Field flows
 * radially OUTWARD from inner ring INTO the outer one. */
static int place_concentric_rings(Pole out[], int max_n,
                                   float cx, float cy, int w, int h)
{
    int n = 0;
    place_ring(out, max_n, &n, cx, cy,
               (float)w * 0.15f, (float)h * 0.18f,
               6, /*alternate*/0, +1, DBL_RING_STRENGTH, 1.0f, 1.0f);
    place_ring(out, max_n, &n, cx, cy,
               (float)w * 0.32f, (float)h * 0.36f,
               8, /*alternate*/0, -1, DBL_RING_STRENGTH, 1.0f, 1.0f);
    return n;
}

/* COIL — 12 same-sign poles on a circle. Approximates a single
 * current loop; inside, B is roughly uniform along the loop's
 * axis; outside, the field looks dipolar (Griffiths ch. 5). */
static int place_current_loop(Pole out[], int max_n,
                                float cx, float cy, int w, int h)
{
    int n = 0;
    place_ring(out, max_n, &n, cx, cy,
               (float)w * 0.30f, (float)h * 0.36f,
               12, /*alternate*/0, +1, COIL_STRENGTH, 1.0f, 1.0f);
    return n;
}

/* HELMHLTZ — two parallel rings of N's separated by their radius.
 * Field between them is approximately UNIFORM along the axis — the
 * configuration used in physics labs to generate constant B over
 * a working volume. */
static int place_helmholtz_coils(Pole out[], int max_n,
                                  float cx, float cy, int w, int h)
{
    float ring_rx = (float)w * 0.18f;
    float ring_ry = (float)h * 0.10f;
    float sep     = (float)h * 0.22f;
    int n = 0;
    place_ring(out, max_n, &n, cx, cy - sep, ring_rx, ring_ry,
               6, /*alternate*/0, +1, HELMHLTZ_STRENGTH, 0.5f, 0.5f);
    place_ring(out, max_n, &n, cx, cy + sep, ring_rx, ring_ry,
               6, /*alternate*/0, +1, HELMHLTZ_STRENGTH, 0.5f, 0.5f);
    return n;
}

/* SOLENOID — N-row across the top + S-row across the bottom — a
 * "magnetic capacitor". Field between the rows is roughly uniform,
 * pointing from N row to S row. */
static int place_parallel_NS_plates(Pole out[], int max_n,
                                     float cx, float cy, int w, int h)
{
    int n = 0;
    place_chain_along(out, max_n, &n,
                      cx - (float)w * 0.30f, cy - (float)h * 0.30f,
                      cx + (float)w * 0.30f, cy - (float)h * 0.30f,
                      8, +1, SOLENOID_STRENGTH, 1.0f, 0.5f);
    place_chain_along(out, max_n, &n,
                      cx - (float)w * 0.30f, cy + (float)h * 0.30f,
                      cx + (float)w * 0.30f, cy + (float)h * 0.30f,
                      8, -1, SOLENOID_STRENGTH, 1.0f, 0.5f);
    return n;
}

/* ── Tier 5 — chaotic / dense ────────────────────────────────── */

/* PLASMA — 14 random poles with random polarity (no jitter). */
static int place_plasma_random(Pole out[], int max_n, int w, int h)
{
    int n = 0;
    place_random(out, max_n, &n, w, h, 14, PLASMA_STRENGTH, 0.0f);
    return n;
}

/* MAZE — 16 alternating poles along a sinusoidally-winding path.
 * The chain traces a sine curve across the map width. */
static int place_maze_chain(Pole out[], int max_n,
                             float cy, int w, int h)
{
    const int count = 16;
    int n = 0;
    for (int i = 0; i < count && n < max_n; i++) {
        float t  = (float)i / (float)(count - 1);
        float bx = (float)w * 0.10f + t * (float)w * 0.80f;
        float by = cy + sinf(t * 4.0f * (float)M_PI) * (float)h * 0.20f;
        out[n++] = make_pole(bx, by, 1.0f, 1.0f,
                              (float)i * 0.3f, MAZE_STRENGTH,
                              (i & 1) ? -1 : +1);
    }
    return n;
}

/* AURORA — 18 alternating poles along a wide arc. The arc shape
 * evokes a curtain hanging across the sky. */
static int place_aurora_arc(Pole out[], int max_n,
                             float cx, float cy, int h)
{
    const int count = 18;
    int n = 0;
    for (int i = 0; i < count && n < max_n; i++) {
        float t   = (float)i / (float)(count - 1);
        float ang = -(float)M_PI * 0.55f + t * (float)M_PI * 1.10f;
        float r   = (float)h * 0.55f;
        float bx  = cx + r * sinf(ang) * 1.4f;
        float by  = cy - r * cosf(ang) + (float)h * 0.30f;
        out[n++] = make_pole(bx, by, 0.5f, 0.5f,
                              (float)i * 0.25f, AURORA_STRENGTH,
                              (i & 1) ? -1 : +1);
    }
    return n;
}

/* CHAOS — 24 random poles with random polarity AND ±30 % strength
 * jitter. Maximum visual complexity. */
static int place_chaos_random(Pole out[], int max_n, int w, int h)
{
    int n = 0;
    place_random(out, max_n, &n, w, h, 24, CHAOS_STRENGTH, 0.30f);
    return n;
}

/*
 * pattern_init_poles — DISPATCHER. For each pattern, call its
 * placer and return the count. Reads as one line per pattern —
 * pure pseudocode.
 */
static int pattern_init_poles(Pole out[], int max_n, Pattern p, int w, int h)
{
    float cx = (float)w * 0.5f;
    float cy = (float)h * 0.5f;

    switch (p) {

    /* Tier 1 — simple (1–2 poles) */
    case PATTERN_MONOPOLE: return place_monopole_centre   (out, max_n, cx, cy);
    case PATTERN_DIPOLE:   return place_dipole_horizontal (out, max_n, cx, cy, w);
    case PATTERN_DIPOLE_V: return place_dipole_vertical   (out, max_n, cx, cy, h);
    case PATTERN_REPELLER: return place_repeller_pair     (out, max_n, cx, cy, w);
    case PATTERN_ATTRACT:  return place_attract_pair      (out, max_n, cx, cy, w);

    /* Tier 2 — few poles (3–4) */
    case PATTERN_TRIPOLE:  return place_tripole_line      (out, max_n, cx, cy, w);
    case PATTERN_TRIANGLE: return place_triangle_1n2s     (out, max_n, cx, cy, h);
    case PATTERN_HORSHOE:  return place_horseshoe_tips    (out, max_n, cx, cy, w, h);
    case PATTERN_QUAD:     return place_quadrupole_corners(out, max_n, cx, cy, w, h);
    case PATTERN_CROSS:    return place_quadrupole_cross  (out, max_n, cx, cy, w, h);
    case PATTERN_PINWHEEL: return place_pinwheel_dipoles  (out, max_n, cx, cy, h);

    /* Tier 3 — medium (5–8 poles) */
    case PATTERN_CHAIN:    return place_chain_horizontal  (out, max_n, cx, cy, w);
    case PATTERN_CHAIN_V:  return place_chain_vertical    (out, max_n, cx, cy, h);
    case PATTERN_HEXAPOLE: return place_hexapole_ring     (out, max_n, cx, cy, w, h);
    case PATTERN_TWIN_DIP: return place_twin_dipole       (out, max_n, cx, cy, w, h);
    case PATTERN_STAR_5:   return place_pentapole_ring    (out, max_n, cx, cy, w, h);
    case PATTERN_OCTUPOLE: return place_octupole_ring     (out, max_n, cx, cy, w, h);
    case PATTERN_MIRROR:   return place_magnetic_mirror   (out, max_n, cx, cy, h);

    /* Tier 4 — structured complex (8+ poles) */
    case PATTERN_NESTED:   return place_nested_squares    (out, max_n, cx, cy, w, h);
    case PATTERN_SUNSPOT:  return place_sunspot_clusters  (out, max_n, cx, cy, w);
    case PATTERN_GRID3:    return place_grid_3x3          (out, max_n, cx, cy, w, h);
    case PATTERN_RING_8:   return place_ring8_samesign    (out, max_n, cx, cy, w, h);
    case PATTERN_DBL_RING: return place_concentric_rings  (out, max_n, cx, cy, w, h);
    case PATTERN_COIL:     return place_current_loop      (out, max_n, cx, cy, w, h);
    case PATTERN_HELMHLTZ: return place_helmholtz_coils   (out, max_n, cx, cy, w, h);
    case PATTERN_SOLENOID: return place_parallel_NS_plates(out, max_n, cx, cy, w, h);

    /* Tier 5 — chaotic / dense */
    case PATTERN_PLASMA:   return place_plasma_random     (out, max_n, w, h);
    case PATTERN_MAZE:     return place_maze_chain        (out, max_n, cy, w, h);
    case PATTERN_AURORA:   return place_aurora_arc        (out, max_n, cx, cy, h);
    case PATTERN_CHAOS:    return place_chaos_random      (out, max_n, w, h);

    default:               return 0;
    }
}

/*
 * compute_active_poles — derive each frame's "live" pole positions
 * from base + orbital drift. The slow Lissajous orbit keeps the
 * field visually alive across patterns that don't otherwise evolve.
 */
static int compute_active_poles(const Pole src[], int n_src,
                                ActivePole out[], float t)
{
    for (int i = 0; i < n_src; i++) {
        float ang = t * DRIFT_RATE + src[i].phase;
        out[i].x        = src[i].bx + src[i].ox * cosf(ang);
        out[i].y        = src[i].by + src[i].oy * sinf(ang);
        out[i].strength = src[i].strength;
        out[i].polarity = src[i].polarity;
    }
    return n_src;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * The scene is built out of six small sub-structs. Each owns ONE
 * concern, and Scene composes them. Splitting concerns into
 * clearly-typed sub-structs makes function signatures self-
 * describing: any function that takes `const Grid *` clearly cannot
 * mutate buffers; any function that takes `PoleField *` clearly does
 * not advance sim time; and so on.
 */

/*
 * Particle — one walker advected by the local magnetic field.
 *
 * INTENT. The walker is "massless": its position is integrated
 * directly from the sampled field direction, with NO momentum term.
 * This is the standard streamline-tracing integration in iron-
 * filings simulation — each filing aligns with the local field
 * vector at every instant. ctrl.speed controls visual pace; the
 * field's wildly varying magnitude (huge near poles, tiny far
 * away) is removed by normalising to a unit direction in
 * sample_unit_field_direction.
 *
 * Finite lifetime via random max_age so streamlines aren't dominated
 * by long-lived particles trapped near one pole — random respawn
 * staggers the population so the trail picture stays uniform.
 */
typedef struct {
    float x, y;       /* position in cell units (continuous floats)         */
    int   color_idx;  /* 0..N_BANDS-1 — palette band for this trail         */
    int   age;        /* ticks since spawn                                  */
    int   max_age;    /* respawn when age ≥ max_age (also respawn on OOB)   */
} Particle;

/*
 * Grid — map geometry. Pure data: no buffers, no state. Lives at
 * the top of Scene because every layer (field sampling, screen
 * centring, the particle loop) needs the dimensions.
 *
 * INDEXING. Row-major: cell (x, y) → y·w + x. This matches the
 * memory layout of RenderBuffers, so the y-outer / x-inner loop
 * order in draw_trail_layer is cache-friendly — each row of the
 * grid is a contiguous run of bytes/floats in memory.
 *
 * INVARIANT. w · h ≤ CELLS_MAX always holds; app_pick_map_size()
 * clamps to MAP_W_MAX × MAP_H_MAX = 200 × 56 = 11 200 cells, which
 * is what the RenderBuffers arrays are statically sized for. The
 * clamp is enforced once per resize; downstream code may assume it.
 */
typedef struct {
    int w, h;         /* current map width / height in cells              */
    int total_cells;  /* = w · h, cached so hot loops skip the multiply   */
} Grid;

static inline int  grid_idx       (const Grid *g, int x, int y) { return y * g->w + x; }
static inline bool grid_in_bounds (const Grid *g, int x, int y)
{
    return x >= 0 && x < g->w && y >= 0 && y < g->h;
}

/*
 * RenderBuffers — per-cell raster output for the trail layer. Two
 * parallel arrays indexed by grid_idx(g, x, y):
 *   glow  : trail intensity 0..1 — drives the density-band glyph
 *   color : palette band 0..N_BANDS-1
 *
 * The screen layer reads ONLY these two arrays. particle_step writes
 * into them via deposit_trail_hit. That is the entire render
 * contract for the trail layer; the pole markers live separately
 * in PoleField. Splitting render output from simulation state is
 * the classic graphics-pipeline decoupling — the sim becomes
 * display-agnostic, and the display could be retargeted (different
 * glyph ramp, a non-ncurses backend) without touching the particle
 * step.
 *
 * SoA RATIONALE. Two separate arrays (struct-of-arrays) rather than
 * one array of cell-structs because:
 *   (a) the screen layer reads glow for every cell every frame, but
 *       color only for non-blank cells — keeping glow dense improves
 *       cache hit rate on the inner draw loop.
 *   (b) decay_trail_glow touches only glow; storing color separately
 *       avoids reading bytes we don't need.
 */
typedef struct {
    /* Per-cell intensity 0..1. particle_step deposits 1.0 at each
     * landing; decay_trail_glow multiplies by expf(-TRAIL_GLOW_DECAY · dt)
     * each tick to create the fading-trail effect. */
    float   glow [CELLS_MAX];

    /* Per-cell palette band. Inherited from the particle that last
     * landed at this cell. Masked with (N_BANDS - 1) at draw time
     * for defence-in-depth. */
    uint8_t color[CELLS_MAX];
} RenderBuffers;

static void buffers_clear(RenderBuffers *b, int n)
{
    for (int i = 0; i < n; i++) {
        b->glow [i] = 0.0f;
        b->color[i] = 0;
    }
}

/*
 * Particles — fixed-size pool of walkers. Standard pool-allocator
 * pattern: a max-sized array plus an active count `n`, so spawn /
 * respawn never touches the heap. Only pool[0..n-1] is alive.
 *
 * SIZING. MAX_PARTICLES (1024) is the static upper bound;
 * N_PARTICLES_DEF (256) is what scene_reset() activates. 256 is
 * enough to populate the visible streamlines of all 30 patterns
 * without saturating the screen.
 */
typedef struct {
    Particle pool[MAX_PARTICLES];
    int      n;
} Particles;

/*
 * PoleField — the magnetic source. Base list of Poles (with their
 * drift orbit parameters) plus a per-frame "active" copy with drift
 * applied.
 *
 * INTENT. Classic graphics-pipeline PRECOMPUTE PATTERN: the cos/sin
 * trig of the orbit equation runs ONCE per frame inside
 * refresh_active_poles, not per cell of field_at. The base/active
 * split keeps the inner loop fast (~256 particles × ~30 poles =
 * ~8000 field samples per frame; we don't want trig in the hot
 * path).
 *
 * Same architectural shape as AttractorPool in
 * flow_field_particles.c — see that file for the broader
 * discussion of the precompute pattern.
 */
typedef struct {
    Pole       base [MAX_POLES];   /* static descriptions               */
    int        n_base;
    ActivePole active[MAX_POLES];  /* live positions, refreshed each tick */
    int        n_active;
} PoleField;

/*
 * SimState — the single mutable per-tick scalar. Separated from
 * Controls (keyboard-driven knobs) so it is obvious what scene_tick
 * mutates vs. what the user does.
 *
 * field_time threads into compute_active_poles (it drives the cos/sin
 * orbit angle for every pole), so by mutating just this one scalar
 * we get the slow visual evolution of every drift-enabled pattern.
 */
typedef struct {
    /* Wall time since last scene_reset(), in seconds. Incremented
     * by dt each tick. Cleared to 0 by reset_sim_state. */
    float field_time;
} SimState;

/*
 * Controls — user-facing knobs. Mutated by app_handle_key(), read
 * by scene_tick() and screen_draw(). Grouping them makes the
 * keyboard handler trivial: `Controls *c = &scene.ctrl;` once at
 * the top, then each key case is a one-line mutation on `c`.
 *
 * INTENT. The split between SimState and Controls is the "model
 * vs. user intent" line common to interactive programs. The
 * dependency is strictly one-way: app_handle_key writes Controls,
 * never SimState; scene_tick reads Controls but only writes
 * SimState + buffers. Keeps the code linear.
 *
 * NO prev_pattern. Switching patterns triggers a full scene_reset
 * (see cycle_pattern in §9) because each pattern needs its own
 * pole layout — there's no need to detect the switch with a one-
 * tick lag.
 */
typedef struct {
    /* Gate for scene_tick: when true the field freezes but the
     * render loop continues so the HUD stays responsive. */
    bool    paused;

    /* Particle advection scale, cells/sec. Multiplied into
     * direction · dt in advect_particle_euler. Doubled by '+',
     * halved by '-', clamped to [SPEED_MIN, SPEED_MAX] = [1, 64].
     * The doubling step (rather than ±1) gives a logarithmic feel. */
    int     speed;

    int     current_theme;       /* index into themes[N_THEMES]    */

    /* Index into glyph_sets[N_GLYPH_SETS]. Mutated by g/G via
     * cycle_glyph_set. Note: changing the glyph set does NOT
     * trigger a scene_reset — only the rendering layer reads it,
     * not the simulation. */
    int     current_glyph_set;

    /* The active pole configuration; mutated by n/p via
     * cycle_pattern. Each switch triggers a full scene_reset to
     * install the new pattern's pole layout. */
    Pattern current_pattern;
} Controls;

/*
 * Scene — the umbrella context. Reading this struct top-to-bottom
 * is meant to be the fastest way to understand the program:
 *   grid       → where things live      (geometry)
 *   buf        → what gets drawn        (render output)
 *   particles  → moving agents          (advection walkers)
 *   poles      → the magnetic source    (PoleField)
 *   sim        → animation state        (field_time)
 *   ctrl       → user knobs             (pattern, speed, …)
 *
 * ORDERING. Each sub-struct depends only on those declared above
 * it: grid is leaf-level; buf is sized by CELLS_MAX (upper bound
 * on grid); particles need grid bounds; poles need grid bounds for
 * placement; sim mutates noise & buf via scene_tick; ctrl decides
 * which sim path runs. A reader scanning top-down meets every
 * concept before it is used.
 *
 * COMPOSITION. There is no Scene-wide invariant that crosses sub-
 * struct boundaries — each sub-struct can be reasoned about (and
 * tested) in isolation. The only function that sees all of Scene
 * at once is scene_tick(). This is composition over inheritance:
 * no virtual dispatch — just six clearly named structs glued
 * together by direct field access.
 *
 * REFERENCE. Mike Acton — "Data-Oriented Design and C++" (CppCon
 * 2014). The broader argument that good struct layout IS good
 * code; motivates the Scene split here and in the sibling files
 * (flow_field_particles.c, curl_noise_vector_field.c,
 * domain_warped_noise_iq_style.c).
 */
typedef struct {
    Grid          grid;       /* immutable per frame (only resize mutates) */
    RenderBuffers buf;        /* written by particles, read by scene_draw  */
    Particles     particles;  /* the advection walkers                     */
    PoleField     poles;      /* magnetic source: base + per-frame active  */
    SimState      sim;        /* mutated only by scene_tick                */
    Controls      ctrl;       /* mutated only by app_handle_key            */
} Scene;

/* ── particle pipeline ───────────────────────────────────────────── *
 * particle_step is the per-particle pseudocode:
 *
 *     sample unit field direction at particle's position
 *     advect by direction · speed · dt
 *     deposit trail hit
 *     respawn if expired (age or OOB)
 *
 * Each step is one named helper below. */

static void particle_spawn(Particle *p, const Grid *g)
{
    p->x         = (float)(rand() % g->w);
    p->y         = (float)(rand() % g->h);
    p->color_idx = rand() & (N_BANDS - 1);
    p->age       = 0;
    p->max_age   = AGE_MIN_TICKS + rand() % (AGE_MAX_TICKS - AGE_MIN_TICKS);
}

/* sample_unit_field_direction — sample B at (px, py) and normalise.
 * Decoupling direction from magnitude lets ctrl.speed control visual
 * pace independently of |B|, which is enormous near poles and tiny
 * far from them. Without normalisation, particles near poles would
 * teleport across the map while distant particles stand still. */
static void sample_unit_field_direction(const Scene *s, float px, float py,
                                          float *out_bx, float *out_by)
{
    field_at(s->poles.active, s->poles.n_active, px, py, out_bx, out_by);
    float m = sqrtf((*out_bx) * (*out_bx) + (*out_by) * (*out_by));
    if (m > VELOCITY_EPSILON) {
        *out_bx /= m;
        *out_by /= m;
    }
}

/* advect_particle_euler — forward-Euler integration step along B̂.
 * Massless advection — no inertia term. */
static void advect_particle_euler(Particle *p, float bx, float by,
                                   float dt, int speed)
{
    p->x += bx * (float)speed * dt;
    p->y += by * (float)speed * dt;
    p->age++;
}

/* deposit_trail_hit — paint the particle's current cell at full
 * intensity. Overwrites (not blends); the trail fade comes from
 * decay_trail_glow between hits. */
static void deposit_trail_hit(Scene *s, int cx, int cy, int color_idx)
{
    if (!grid_in_bounds(&s->grid, cx, cy)) return;
    int idx = grid_idx(&s->grid, cx, cy);
    s->buf.glow [idx] = TRAIL_HIT_INTENSITY;
    s->buf.color[idx] = (uint8_t)color_idx;
}

/* particle_is_expired — has the walker overstayed its life or
 * walked off the grid? Either triggers a respawn. */
static bool particle_is_expired(const Particle *p, const Grid *g)
{
    return p->age >= p->max_age
        || p->x < 0.0f || p->x >= (float)g->w
        || p->y < 0.0f || p->y >= (float)g->h;
}

static void particle_step(Scene *s, Particle *p, float dt, int speed)
{
    float bx, by;
    sample_unit_field_direction(s, p->x, p->y, &bx, &by);
    advect_particle_euler      (p, bx, by, dt, speed);
    deposit_trail_hit          (s, (int)p->x, (int)p->y, p->color_idx);
    if (particle_is_expired(p, &s->grid))
        particle_spawn         (p, &s->grid);
}

/* ── tick pipeline ───────────────────────────────────────────────── *
 * scene_tick is the per-frame pseudocode:
 *
 *     if paused: stop.
 *     decay all trail glows exponentially.
 *     advance field_time by dt.
 *     refresh pole positions from base + drift.
 *     step every particle.
 *
 * Each step below is one named helper. */

static void decay_trail_glow(RenderBuffers *b, int n, float dt)
{
    float decay = expf(-TRAIL_GLOW_DECAY * dt);
    for (int i = 0; i < n; i++) b->glow[i] *= decay;
}

static void advance_field_time(SimState *sim, float dt)
{
    sim->field_time += dt;
}

static void refresh_active_poles(PoleField *pf, float sim_time)
{
    pf->n_active = compute_active_poles(pf->base, pf->n_base,
                                          pf->active, sim_time);
}

static void step_all_particles(Scene *s, float dt)
{
    int spd = s->ctrl.speed;
    for (int i = 0; i < s->particles.n; i++)
        particle_step(s, &s->particles.pool[i], dt, spd);
}

/* ── reset / init pipeline ───────────────────────────────────────── */

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

static void spawn_all_particles(Particles *ps, const Grid *g)
{
    ps->n = N_PARTICLES_DEF;
    for (int i = 0; i < ps->n; i++)
        particle_spawn(&ps->pool[i], g);
}

static void install_pattern_poles(PoleField *pf, Pattern p, const Grid *g)
{
    pf->n_base   = pattern_init_poles(pf->base, MAX_POLES, p, g->w, g->h);
    pf->n_active = 0;
}

static void scene_reset(Scene *s, int w, int h)
{
    apply_grid_dimensions    (&s->grid, w, h);
    reset_sim_state          (&s->sim);
    buffers_clear            (&s->buf, s->grid.total_cells);
    install_pattern_poles    (&s->poles, s->ctrl.current_pattern, &s->grid);
    spawn_all_particles      (&s->particles, &s->grid);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->ctrl.paused            = false;
    s->ctrl.speed             = SPEED_DEF;
    s->ctrl.current_theme     = 0;
    s->ctrl.current_glyph_set = 2;        /* MEDIUM */
    s->ctrl.current_pattern   = PATTERN_DIPOLE;
    scene_reset(s, w, h);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;

    decay_trail_glow      (&s->buf, s->grid.total_cells, dt);
    advance_field_time    (&s->sim, dt);
    refresh_active_poles  (&s->poles, s->sim.field_time);
    step_all_particles    (s, dt);
}

/* ===================================================================== */
/* §8  screen                                                             */
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

/* ── scene_draw pipeline ─────────────────────────────────────────── *
 * Two passes:
 *   1. Trail layer — density glyphs from the current GlyphSet,
 *      coloured by buf.color band.
 *   2. Pole layer — bright red 'N' / blue 'S' markers on top, in
 *      theme-independent colours. */

/*
 * CellDraw — the output of the per-cell density-band classifier.
 * A tiny value-type bundling "what to paint at this position";
 * paint_cell() converts it into the actual ncurses I/O triple.
 *
 * INTENT. cell_density_band is a pure function — given (band, glow,
 * GlyphSet) it returns a CellDraw with no side effects. Routing it
 * through CellDraw rather than calling ncurses directly keeps the
 * decision logic separate from the I/O. paint_cell is then the
 * SINGLE place the program touches ncurses for the trail layer.
 */
typedef struct {
    int  pair;
    int  attr;
    char glyph;
    bool skip;
} CellDraw;

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

/* cell_density_band — pick the density-band glyph for a trail cell. */
static CellDraw cell_density_band(uint8_t band, float glow, const GlyphSet *gs)
{
    int pair = PAIR_TRAIL_BASE + (band & (N_BANDS - 1));
    if (glow > GLYPH_HIGH_THRESH) return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = gs->high };
    if (glow > GLYPH_MID_THRESH ) return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = gs->mid  };
    if (glow > GLOW_THRESHOLD   ) return (CellDraw){ .pair = pair, .attr = A_NORMAL, .glyph = gs->low  };
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

/* draw_trail_layer — pass 1. */
static void draw_trail_layer(const Scene *s, int gx0, int gy0, int cols, int rows,
                              const GlyphSet *gs)
{
    const Grid          *g = &s->grid;
    const RenderBuffers *b = &s->buf;
    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            int idx = grid_idx(g, x, y);
            paint_cell(sy, sx, cell_density_band(b->color[idx], b->glow[idx], gs));
        }
    }
}

/* draw_pole_layer — pass 2: theme-independent red 'N' / blue 'S'
 * markers on top of the trail layer. */
static void draw_pole_layer(const PoleField *pf, int gx0, int gy0,
                              int cols, int rows)
{
    for (int i = 0; i < pf->n_active; i++) {
        int sx = gx0 + (int)pf->active[i].x;
        int sy = gy0 + (int)pf->active[i].y;
        if (sx < 0 || sx >= cols) continue;
        if (sy < 0 || sy >= rows) continue;
        int   pair  = (pf->active[i].polarity > 0) ? PAIR_NORTH : PAIR_SOUTH;
        char  glyph = (pf->active[i].polarity > 0) ? 'N' : 'S';
        attron (COLOR_PAIR(pair) | A_BOLD);
        mvaddch(sy, sx, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

static const GlyphSet *active_glyph_set(const Scene *s)
{
    int idx = s->ctrl.current_glyph_set;
    if (idx < 0 || idx >= N_GLYPH_SETS) idx = 0;
    return &glyph_sets[idx];
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    int gx0, gy0;
    compute_centred_origin(&s->grid, cols, rows, &gx0, &gy0);
    const GlyphSet *gs = active_glyph_set(s);
    draw_trail_layer(s, gx0, gy0, cols, rows, gs);
    draw_pole_layer (&s->poles, gx0, gy0, cols, rows);
}

/* ── HUD draw pipeline ───────────────────────────────────────────── *
 * Six named drawers, called from screen_draw in z-order. The TOP HUD
 * (rows 0..2) carries DATA — current state with [N/30] index,
 * parameter readouts, glyph indicator, legend. The BOTTOM HUD
 * (row N-1) carries ACTIONS — key bindings only.
 *
 * draw_hud_status_line internally composes several smaller segment
 * drawers, one per fixed-width row-1 field. */

static void draw_hud_state_bar(const Screen *sc, const Scene *s,
                                double fps, int sim_fps)
{
    const Controls *c = &s->ctrl;
    const char *state_str = c->paused ? "PAUSED  "
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
    mvprintw(0, HUD_LEFT_MARGIN, " MAGNETIC FIELDS ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* ── row 1 segment drawers ───────────────────────────────────────── *
 * draw_hud_status_line lays out row 1 left-to-right as a sequence of
 * fixed-width segments. Each segment paints its content and returns
 * the new x-cursor; pole-count segment also accepts and updates the
 * cursor since the digit widths vary. */

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

/* draw_status_pole_counts — N:n S:n in the physics-convention colours.
 * The N/S split lets the user verify polarity counts at a glance —
 * useful when cycling through patterns to see if e.g. TRIANGLE
 * actually placed 1N + 2S. */
static int draw_status_pole_counts(int row, int x, const PoleField *pf)
{
    int n_north = 0, n_south = 0;
    for (int i = 0; i < pf->n_base; i++) {
        if (pf->base[i].polarity > 0) n_north++; else n_south++;
    }
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  ");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 2;
    attron (COLOR_PAIR(PAIR_NORTH) | A_BOLD);
    mvprintw(row, x, "N:%d", n_north);
    attroff(COLOR_PAIR(PAIR_NORTH) | A_BOLD);
    x += (n_north < 10 ? 3 : 4);
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, " ");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 1;
    attron (COLOR_PAIR(PAIR_SOUTH) | A_BOLD);
    mvprintw(row, x, "S:%d", n_south);
    attroff(COLOR_PAIR(PAIR_SOUTH) | A_BOLD);
    x += (n_south < 10 ? 3 : 4);
    return x;
}

/* draw_status_sim_counts — last segment: live particle count + map
 * dimensions. No return — end of the row. */
static void draw_status_sim_counts(int row, int x, const Scene *s)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  parts:%d  map:%dx%d ",
             s->particles.n, s->grid.w, s->grid.h);
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
    x = draw_status_pole_counts   (1, x, &s->poles);
        draw_status_sim_counts    (1, x, s);
}

/* draw_hud_glyph_indicator — right-aligned on row 1. Live sample of
 * the three glyphs in the current set so the user can see what each
 * set looks like before switching with g/G. Right-aligned so it
 * doesn't collide with the variable-width sim counts on the left. */
static void draw_hud_glyph_indicator(const Scene *s, const Screen *sc)
{
    const GlyphSet *gs = active_glyph_set(s);
    char buf[32];
    snprintf(buf, sizeof buf, " glyph:%-7s [%c%c%c] ",
             gs->name, gs->low, gs->mid, gs->high);
    int gx = sc->cols - (int)strlen(buf);
    if (gx < 0) gx = 0;
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, gx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* ── row 2 legend segment drawers ────────────────────────────────── *
 * The legend lives on row 2 (top HUD). Two visual sub-elements: the
 * trail-tier description (left half) and the pole marker key (right
 * half, painted in the physics-convention colours so it visually
 * matches the on-field N/S glyphs). */

/* draw_legend_trail_tiers — left half: "trails: field-lines …". */
static void draw_legend_trail_tiers(int row, int x)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, " trails: field-lines (low -> mid -> high)   ");
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* draw_one_pole_marker — paints "X: ..." where X is rendered in
 * the pole's actual physics-convention colour, then the description
 * follows in the HUD colour. Returns the x-cursor just past the
 * description so callers can chain. */
static int draw_one_pole_marker(int row, int x,
                                  char glyph, int marker_pair,
                                  const char *description)
{
    attron (COLOR_PAIR(marker_pair) | A_BOLD);
    mvaddch(row, x, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(marker_pair) | A_BOLD);
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x + 1, "%s", description);
    attroff(COLOR_PAIR(PAIR_HUD));
    return x + 1 + (int)strlen(description);
}

/* draw_legend_pole_markers — right half: "N: north pole   S: south
 * pole ", with each glyph in its physics-convention colour. */
static void draw_legend_pole_markers(int row, int x)
{
    x = draw_one_pole_marker(row, x, 'N', PAIR_NORTH, ": north pole   ");
        draw_one_pole_marker(row, x, 'S', PAIR_SOUTH, ": south pole ");
}

/* draw_hud_legend — row 2: glyph-meaning reference. Belongs in the
 * top HUD because it's DATA (how to READ the screen), not an
 * ACTION (what you can press). Composes the two named segments. */
static void draw_hud_legend(void)
{
    draw_legend_trail_tiers (2, HUD_LEFT_MARGIN);
    draw_legend_pole_markers(2, HUD_LEFT_MARGIN + 45);
}

/* draw_bottom_hint — row N-1: ACTIONS only (key bindings). */
static void draw_bottom_hint(const Screen *sc)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  g/G:glyph  t/T:theme  r:reset  spc:pause  +/-:speed  ]/[:Hz  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw              (s, sc->cols, sc->rows);
    draw_hud_state_bar      (sc, s, fps, sim_fps);   /* row 0  : data    */
    draw_hud_title          ();                       /* row 0  : data    */
    draw_hud_status_line    (s);                      /* row 1  : data    */
    draw_hud_glyph_indicator(s, sc);                  /* row 1  : data    */
    draw_hud_legend         ();                       /* row 2  : data    */
    draw_bottom_hint        (sc);                     /* row N-1: actions */
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

/*
 * App — top-level program state. One instance, g_app, lives in BSS
 * so the signal handlers can reach it without a global Scene
 * pointer. The App owns the Scene and the Screen and adds:
 *   • simulation parameters that are not really "scene state"
 *     (sim_fps, map_w, map_h);
 *   • signal-driven flags that must be sig_atomic_t for safety.
 *
 * SIGNAL-HANDLER DISCIPLINE. The handlers do nothing but set a flag.
 * The main loop polls those flags and performs the actual work
 * (cleanup, resize) in normal execution context. Standard async-
 * signal-safe pattern — anything that touches ncurses or malloc
 * MUST happen outside the handler.
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

/* bump_speed_geometric — '+' / '-' geometric step on speed. */
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

static void cycle_glyph_set(Controls *c, int dir)
{
    c->current_glyph_set =
        (c->current_glyph_set + dir + N_GLYPH_SETS) % N_GLYPH_SETS;
}

/* cycle_pattern — n/p step + full scene reset. We reset because each
 * pattern needs its own pole layout; reusing the previous pattern's
 * poles would give a glitched first frame. */
static void cycle_pattern(App *app, int dir)
{
    Controls *c = &app->scene.ctrl;
    c->current_pattern = (Pattern)(
        ((int)c->current_pattern + dir + N_PATTERNS) % N_PATTERNS);
    scene_reset(&app->scene, app->map_w, app->map_h);
}

static bool app_handle_key(App *app, int ch)
{
    Controls *c = &app->scene.ctrl;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           c->paused = !c->paused;                            break;
    case 'r': case 'R': scene_reset(&app->scene, app->map_w, app->map_h);  break;
    case '=': case '+': bump_speed_geometric(c, +1);                       break;
    case '-':           bump_speed_geometric(c, -1);                       break;
    case ']':           bump_sim_fps(app, +SIM_FPS_STEP);                  break;
    case '[':           bump_sim_fps(app, -SIM_FPS_STEP);                  break;
    case 't':           cycle_theme    (c,   +1);                          break;
    case 'T':           cycle_theme    (c,   -1);                          break;
    case 'g':           cycle_glyph_set(c,   +1);                          break;
    case 'G':           cycle_glyph_set(c,   -1);                          break;
    case 'n': case 'N': cycle_pattern  (app, +1);                          break;
    case 'p': case 'P': cycle_pattern  (app, -1);                          break;
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
