/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * reaction_diffusion_gray_scott.c
 *   — Gray-Scott reaction-diffusion: 30 chemical-pattern presets.
 *
 * DEMO: A 2-D reaction-diffusion system simulating two chemicals U
 *       and V that diffuse at different rates and react via
 *       U + 2V → 3V. The system is seeded with a small blob of V in
 *       the centre; over a few seconds, V "eats" U outward, and the
 *       resulting front breaks up into intricate self-organising
 *       patterns. Thirty (F, k) presets — different points in
 *       Pearson's morphology phase diagram — produce visually
 *       distinct outcomes:
 *
 *         Tier 1 (classics)     : SPOTS STRIPES MAZES CORAL WORMS
 *         Tier 2 (Pearson α-ε)  : ALPHA BETA GAMMA DELTA EPSILON
 *         Tier 3 (named)        : MITOSIS SOLITON FINGERS SKATE PSI
 *         Tier 4 (dynamic)      : WAVES PULSES NEBULA NEURONS CHAOS
 *         Tier 5 (extreme)      : HOLES BUBBLES SLUDGE PLANKTON CRYSTAL
 *                                 THICKET DRIFT ZEBRA BANDS BARNACLE
 *
 *       Press n/p to switch presets — the simulation resets each time
 *       so you watch the pattern self-organise from a fresh seed. The
 *       state bar shows [N/30] for the active preset.
 *
 * Study alongside:
 *   ../../fluid/reaction_diffusion.c — same Gray-Scott algorithm, framed
 *       as PDE / numerical-methods pedagogy: 7 presets, 9-point isotropic
 *       Laplacian, cross-refs to cfl_stability_explorer.c.  This file is
 *       the lighter pattern-formation demo; that one is the deep dive.
 *   ./perin_noise_flow_showcase.c — also a "field over the grid", but
 *       the field here EVOLVES via PDE rather than being sampled
 *       from a static noise function. Reaction-diffusion is what
 *       gives biological pigmentation patterns; Perlin/simplex give
 *       static landscape patterns.
 *   ../generational/cellular_automata_cave_4-5_rule_showcase.c — a
 *       discrete cousin: cellular automata are reaction-diffusion's
 *       binary-state equivalent. Rules are integer-state, but the
 *       morphology family is similar (spots, mazes, blobs).
 *
 * Section map:
 *   §1 config   — grid, presets, palette, themes, named constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes (4 bands each)
 *   §5 rd       — Grid + ReactionField + PDE primitives (laplacian_5pt,
 *                 gray_scott_dU/dV, swap_pde_buffers) + seed pipeline +
 *                 step_reaction_field driver
 *   §6 patterns — GSPreset + 30 (F, k) presets across Pearson's phase
 *                 diagram + install_kinetics
 *   §7 scene    — SimState, Controls, Scene composing Grid +
 *                 ReactionField + SimState + Controls; named reset
 *                 and tick pipeline helpers
 *   §8 screen   — ASCII render: classify_v_cell + paint_cell + HUD drawers
 *   §9 app      — signals, resize, named main-loop helpers
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (re-seed + perturb)
 *   n / N      next pattern preset
 *   p / P      previous pattern preset
 *   t / T      next / previous theme
 *   + / =      faster simulation (more sub-steps per frame)
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra reaction_diffusion_gray_scott.c \
 *       -o gray_scott -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Gray-Scott reaction-diffusion (Gray & Scott, 1983).
 *                  Two chemical concentrations U and V on a 2-D grid,
 *                  evolving under a coupled PDE system:
 *
 *                    ∂U/∂t = D_u · ∇²U − U · V² + F · (1 − U)
 *                    ∂V/∂t = D_v · ∇²V + U · V² − (F + k) · V
 *
 *                  Reading right to left:
 *                    • Both U and V DIFFUSE (with different diffusion
 *                      constants D_u > D_v — V is heavier / slower).
 *                    • The REACTION U + 2V → 3V converts U into V at a
 *                      rate proportional to U·V². V "eats" U.
 *                    • U is FED at rate F · (1 − U) — pushed back
 *                      toward 1 wherever it dropped.
 *                    • V is KILLED at rate (F + k) · V — decays back
 *                      toward 0 wherever it isn't being created.
 *
 *                  The competition between these four processes — at
 *                  different rates F, k — produces a phase diagram
 *                  with dramatically different morphologies (Pearson
 *                  1993). We expose 5 of the named regions as
 *                  switchable presets.
 *
 *                  Numerical method: explicit forward-Euler with a
 *                  5-point Laplacian stencil. Time step dt = 1 (the
 *                  unit in which F and k are usually quoted), with
 *                  multiple sub-steps per render frame for visible
 *                  evolution. Boundary condition: NEUMANN (zero-flux,
 *                  i.e. clamp out-of-grid neighbours to the cell
 *                  itself). Periodic boundaries also work but produce
 *                  visible tile-seams when the central pattern
 *                  reaches the edges.
 *
 * Data-structure : Scene composes four sub-structs (see §7):
 *                    Grid          — w, h, total_cells
 *                    ReactionField — U, V, U_next, V_next + F, k
 *                                    (chemistry state + rate constants)
 *                    SimState      — step_count
 *                    Controls      — paused, sub_steps, theme, pattern
 *                  Two pairs of float grids in ReactionField: (U, V)
 *                  for the current state and (U_next, V_next) for the
 *                  buffer being written. Swap (memcpy) at the end of
 *                  each sub-step via swap_pde_buffers. All preset-
 *                  tuning lives in §6 patterns; the simulation itself
 *                  is parameter-agnostic. No heap allocation.
 *
 * Rendering      : ASCII only. Each cell's V value drives a density
 *                  glyph ('.', '*', '#') and a 4-band colour. V
 *                  typically lives in roughly [0, 0.4] for these
 *                  presets, so we multiply by ~2.5 for visibility.
 *
 * Performance    : 5-point Laplacian + Euler step is ≈ 12 multiply-adds
 *                  per cell per sub-step. With 4 sub-steps at 60 Hz on
 *                  a 200×56 grid that's ~32 M ops/sec — well within
 *                  budget on any modern CPU.
 *
 * References     : Algorithm / math
 *                  ────────────────
 *                  • Turing, A. M. (1952) — "The Chemical Basis of
 *                    Morphogenesis", Phil. Trans. R. Soc. B 237:37-72.
 *                    The foundational paper. Predicted — decades
 *                    before any chemical system was known to actually
 *                    do it — that reaction + diffusion of "morphogens"
 *                    could explain biological pattern formation. Gray-
 *                    Scott is one specific realisation of Turing's
 *                    general scheme; the morphologies in §6 are
 *                    Turing's "morphogen patterns" in concrete form.
 *                  • Gray, P. & Scott, S. K. (1983) — "Autocatalytic
 *                    reactions in the isothermal, continuous stirred
 *                    tank reactor", Chemical Engineering Science
 *                    38(1):29-43. The original chemical-engineering
 *                    derivation of the PDE we solve in §5. Source of
 *                    the U + 2V → 3V reaction and the (F, k) feed/kill
 *                    formulation.
 *                  • Pearson, J. E. (1993) — "Complex Patterns in a
 *                    Simple System", Science 261(5118):189-192. The
 *                    phase-diagram paper showing 11 distinct morphology
 *                    regions (Pearson's letter codes α through κ) in
 *                    (F, k) parameter space. The Tier-2 presets in §6
 *                    use his letter codes directly.
 *                  • Murray, J. D. (2003) — "Mathematical Biology II:
 *                    Spatial Models and Biomedical Applications"
 *                    (3rd ed), Springer. The standard graduate textbook
 *                    for reaction-diffusion. Ch. 2 derives the Turing
 *                    instability condition and explains why D_u ≠ D_v
 *                    is essential for patterns to form.
 *                  • Press, W. H., Teukolsky, S. A., Vetterling, W. T.
 *                    & Flannery, B. P. (2007) — "Numerical Recipes"
 *                    (3rd ed), Cambridge. §19 covers explicit forward-
 *                    Euler integration and the 5-point Laplacian
 *                    stencil — the numerical method used in
 *                    step_reaction_field. Also discusses the CFL
 *                    stability bound that constrains GS_DT in §1.
 *
 *                  Visualisation / pattern atlas
 *                  ─────────────────────────────
 *                  • Karl Sims — interactive Gray-Scott explorer
 *                    (lookup: "Karl Sims reaction diffusion"). Slider
 *                    UI for (F, k) with named preset buttons; source
 *                    of the SPOTS / STRIPES / MAZES / CORAL / WORMS
 *                    canonical preset values in §6 Tier 1.
 *                  • Robert Munafo — "xmorphia":
 *                    https://mrob.com/pub/comp/xmorphia/
 *                    Exhaustive Gray-Scott pattern catalogue —
 *                    parameter map, region names (ψ-region, u-skate
 *                    world, frogspawn, etc.), and decades of
 *                    documented experiments. Tier 3 (SKATE, PSI) and
 *                    several Tier 5 presets come straight from his
 *                    pages.
 *                  • Paul Bourke — "Character representation of grey
 *                    scale images":
 *                    https://paulbourke.net/dataformats/asciiart/
 *                    The canonical density → glyph ramp reference.
 *                    Underlies the .:low / *:mid / #:high mapping in
 *                    scene_draw.
 *
 *                  Online tutorials
 *                  ────────────────
 *                  • Wikipedia — "Reaction–diffusion system":
 *                    https://en.wikipedia.org/wiki/Reaction%E2%80%93diffusion_system
 *                    Concise starting point with the PDE form and a
 *                    summary of morphology regimes.
 *
 *                  See also
 *                  ────────
 *                  • ../../fluid/reaction_diffusion.c — same Gray-Scott
 *                    algorithm with deeper PDE / numerical-methods
 *                    pedagogy: 9-point isotropic Laplacian, more
 *                    presets, and cross-refs to CFL stability analysis.
 *                    This file is the lighter pattern-formation demo;
 *                    that one is the deep dive.
 *                  • ./perin_noise_flow_showcase.c — also a "scalar
 *                    field over the grid", but the field there is
 *                    sampled from static Perlin noise instead of
 *                    evolved from a PDE. Reading both side by side
 *                    contrasts the two main ways to generate organic-
 *                    looking 2-D patterns: analytic noise vs.
 *                    integrated dynamics.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two chemicals (U and V) compete on a grid. U is plentiful and
 * spreads slowly; V is rare and spreads slower still, but it can
 * "eat" U and reproduce. Add a slow "feed" of fresh U from outside
 * and a slow "kill" of V, and you have a system that perpetually
 * fights itself into self-organising patterns. Different F (feed)
 * and k (kill) rates change which morphology dominates: spots,
 * stripes, mazes, coral, worms — all from the same equations.
 *
 * Crucially, the patterns are NOT random — they emerge
 * deterministically from physical reaction kinetics. This is one of
 * Alan Turing's "morphogens": the chemical mechanism behind animal
 * pigmentation, fingerprint formation, and many biological textures.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture an ocean of U with a tiny droplet of V at the centre. V is
 * voracious — it converts nearby U into more V at a rate
 * proportional to U·V². At first the droplet just grows outward.
 * But V also dies (at rate F+k), and U gets refreshed (at rate F).
 * If V's spread is ROUGHLY in balance with V's death, the front
 * stops being a smooth circle — small fluctuations get amplified
 * because curvy regions of the V-front have more or less U to consume.
 * That's the morphogenic instability.
 *
 * Once the instability hits, the system evolves toward a stable
 * pattern that depends on F and k:
 *   F LOW, k LOW   — V wins everywhere, pattern dies
 *   F MID, k MID   — instability balances → the interesting morphologies
 *   F HIGH, k HIGH — V dies fast, pattern collapses to U-only
 *
 * The interesting region (Pearson's diagram) is a thin curving sliver
 * in (F, k) space. We pre-pick 5 well-known points in that sliver
 * and label them.
 *
 * Visible layers:
 *   • V CONCENTRATION — every cell's brightness shows local V level.
 *   • PALETTE BAND — quartile of V drives the colour band, so dim,
 *     mid, and bright cells get different colours.
 *   • Time evolution — the pattern develops over a few seconds from
 *     a tiny seed, then often reaches a quasi-stable configuration
 *     (spots/stripes) or keeps drifting (worms).
 *
 * ALGORITHM IN STEPS  (per sub-step — driver: step_reaction_field)
 * ──────────────────
 *  For every cell (x, y):
 *    1. lap_U = laplacian_5pt(U, idx, il, ir, iu, id)
 *    2. lap_V = laplacian_5pt(V, idx, il, ir, iu, id)
 *    3. U_next[idx] = U + dt · gray_scott_dU(u, v, lap_U, F, Du)
 *    4. V_next[idx] = V + dt · gray_scott_dV(u, v, lap_V, F, k, Dv)
 *  After all cells done: swap_pde_buffers (memcpy *_next back into U/V).
 *
 *  scene_tick.run_sub_steps drains ctrl.sub_steps Gray-Scott updates
 *  per render frame so the chemistry evolves fast enough to be
 *  watchable but stable enough to not numerically blow up.
 *
 * KEY FORMULAS
 * ────────────
 *  Diffusion (5-point Laplacian) :
 *      ∇² f(x,y) ≈ f(x-1,y) + f(x+1,y) + f(x,y-1) + f(x,y+1) − 4f(x,y)
 *  Reaction term                : r = U · V²
 *  U update                     : U' = U + dt · (D_u · ∇²U − r + F·(1−U))
 *  V update                     : V' = V + dt · (D_v · ∇²V + r − (F+k)·V)
 *  Stability bound (CFL-ish)    : dt · (D_u + D_v) · 4 ≲ 1
 *  Steady U=1, V=0 fixed point  : trivial; perturb to escape it
 *
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
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    CELLS_MAX         = MAP_W_MAX * MAP_H_MAX,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* Sub-steps per render frame. More = faster pattern development.
     * 6 lets a pattern emerge over ~10 seconds of wall time. */
    SUB_STEPS_MIN     =   1,
    SUB_STEPS_DEF     =   6,
    SUB_STEPS_MAX     = 128,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,    /* PAIR_BAND_BASE..+3 = 4 palette colours */
    PAIR_FLASH        =   7,    /* vestigial — cross-file palette parity  */
};

/* Standard Gray-Scott diffusion rates. */
#define GS_DU               0.16f
#define GS_DV               0.08f
#define GS_DT               1.0f

/* Render scaling: V values typically peak around 0.35-0.45 in the
 * pattern regions; multiply for visibility before mapping to glow. */
#define V_RENDER_GAIN       2.5f

#define GLOW_THRESHOLD      0.05f

/* ── HUD layout ──────────────────────────────────────────────────── *
 * Top HUD carries DATA, bottom HUD carries ACTIONS:
 *   row 0           : title + state bar (fps, Hz, state + [N/M], sub-steps)
 *   row 1           : pattern/theme/palette + F/k/Du/Dv/map
 *   row HUD_TOP..N-2: reaction-diffusion field
 *   row N-1         : keyboard action hint
 */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* Density thresholds for the ASCII glyph ramp. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/*
 * Pattern presets — 30 distinct (F, k) points in Pearson's morphology
 * phase diagram, spanning simple → complex. Cycle with n/p; switching
 * resets the simulation so the chosen pattern starts forming from a
 * fresh seed.
 *
 * INTENT. The simulation kernel (5-point Laplacian + forward-Euler
 * Gray-Scott step) is identical for all 30; only the (F, k) pair
 * changes. Adding a new preset is one enum value + one pattern_name
 * case + one gs_presets row.
 *
 * TIER ORGANISATION.
 *   1 — classic stable patterns (the 5 most recognisable morphologies)
 *   2 — Pearson 1993 region letters (α through ε)
 *   3 — named morphologies from Karl Sims + Munafo's xmorphia catalogue
 *   4 — dynamic / travelling patterns
 *   5 — extreme / complex regions (boundary + chaotic)
 *
 * Values vetted from: Pearson, J. E. (1993) Science 261:189-192;
 * Karl Sims interactive explorer; Munafo's xmorphia
 * (https://mrob.com/pub/comp/xmorphia/).
 */
typedef enum {
    /* Tier 1 — classic stable patterns */
    PATTERN_SPOTS    =  0,
    PATTERN_STRIPES  =  1,
    PATTERN_MAZES    =  2,
    PATTERN_CORAL    =  3,
    PATTERN_WORMS    =  4,

    /* Tier 2 — Pearson 1993 letter regions */
    PATTERN_ALPHA    =  5,
    PATTERN_BETA     =  6,
    PATTERN_GAMMA    =  7,
    PATTERN_DELTA    =  8,
    PATTERN_EPSILON  =  9,

    /* Tier 3 — named morphologies (Karl Sims / Munafo) */
    PATTERN_MITOSIS  = 10,
    PATTERN_SOLITON  = 11,
    PATTERN_FINGERS  = 12,
    PATTERN_SKATE    = 13,
    PATTERN_PSI      = 14,

    /* Tier 4 — dynamic / travelling patterns */
    PATTERN_WAVES    = 15,
    PATTERN_PULSES   = 16,
    PATTERN_NEBULA   = 17,
    PATTERN_NEURONS  = 18,
    PATTERN_CHAOS    = 19,

    /* Tier 5 — extreme / complex regions */
    PATTERN_HOLES    = 20,
    PATTERN_BUBBLES  = 21,
    PATTERN_SLUDGE   = 22,
    PATTERN_PLANKTON = 23,
    PATTERN_CRYSTAL  = 24,
    PATTERN_THICKET  = 25,
    PATTERN_DRIFT    = 26,
    PATTERN_ZEBRA    = 27,
    PATTERN_BANDS    = 28,
    PATTERN_BARNACLE = 29,

    N_PATTERNS       = 30,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_SPOTS:    return "SPOTS    ";
    case PATTERN_STRIPES:  return "STRIPES  ";
    case PATTERN_MAZES:    return "MAZES    ";
    case PATTERN_CORAL:    return "CORAL    ";
    case PATTERN_WORMS:    return "WORMS    ";
    case PATTERN_ALPHA:    return "ALPHA    ";
    case PATTERN_BETA:     return "BETA     ";
    case PATTERN_GAMMA:    return "GAMMA    ";
    case PATTERN_DELTA:    return "DELTA    ";
    case PATTERN_EPSILON:  return "EPSILON  ";
    case PATTERN_MITOSIS:  return "MITOSIS  ";
    case PATTERN_SOLITON:  return "SOLITON  ";
    case PATTERN_FINGERS:  return "FINGERS  ";
    case PATTERN_SKATE:    return "SKATE    ";
    case PATTERN_PSI:      return "PSI      ";
    case PATTERN_WAVES:    return "WAVES    ";
    case PATTERN_PULSES:   return "PULSES   ";
    case PATTERN_NEBULA:   return "NEBULA   ";
    case PATTERN_NEURONS:  return "NEURONS  ";
    case PATTERN_CHAOS:    return "CHAOS    ";
    case PATTERN_HOLES:    return "HOLES    ";
    case PATTERN_BUBBLES:  return "BUBBLES  ";
    case PATTERN_SLUDGE:   return "SLUDGE   ";
    case PATTERN_PLANKTON: return "PLANKTON ";
    case PATTERN_CRYSTAL:  return "CRYSTAL  ";
    case PATTERN_THICKET:  return "THICKET  ";
    case PATTERN_DRIFT:    return "DRIFT    ";
    case PATTERN_ZEBRA:    return "ZEBRA    ";
    case PATTERN_BANDS:    return "BANDS    ";
    case PATTERN_BARNACLE: return "BARNACLE ";
    default:               return "?        ";
    }
}

/*
 * GSPreset — one (F, k) point in Pearson's morphology phase diagram.
 *
 * INTENT. The Gray-Scott system has a 2-D continuous parameter space
 * (F, k); different points produce wildly different morphologies. A
 * GSPreset bundles the two values together so the preset table reads
 * as a column of "named coordinates" — Tier-1 SPOTS at (0.035, 0.065),
 * Tier-3 SKATE at (0.062, 0.0609), and so on.
 *
 * PARAMETER MEANINGS.
 *   F — feed rate. How quickly U is refreshed back toward 1 wherever
 *       it has been depleted. Appears as F·(1 − U) in the U PDE.
 *       Active range roughly F ∈ [0.010, 0.075].
 *   k — kill rate. How quickly V decays back toward 0 wherever it
 *       isn't being created. Appears as (F + k)·V in the V PDE.
 *       Active range roughly k ∈ [0.040, 0.070].
 *
 * Bounds: outside this active region the system collapses to a
 * trivial fixed point (U = 1, V = 0) and produces no visible pattern.
 * In particular F + k ≳ 0.135 puts the system in the "kill-dominant"
 * region where V dies — the screen goes empty. The 30 presets in §6
 * all live inside the active region.
 *
 * REFERENCE. Pearson, J. E. (1993), Science 261:189-192 — Figure 1
 * shows the full phase diagram and labels 11 morphology regions.
 */
typedef struct {
    float F;     /* feed rate; pushes U back toward 1 (PDE term: F·(1−U))  */
    float k;     /* kill rate; pulls V back toward 0 (PDE term: (F+k)·V)   */
} GSPreset;

static const GSPreset gs_presets[N_PATTERNS] = {
    /*  F        k       — vetted Gray-Scott parameter pairs */

    /* Tier 1 — classic stable patterns */
    { 0.035f, 0.065f },   /* SPOTS    — mitosis-like dividing spots (Pearson γ) */
    { 0.022f, 0.051f },   /* STRIPES  — stable parallel stripes                  */
    { 0.029f, 0.057f },   /* MAZES    — labyrinth (Pearson θ)                    */
    { 0.054f, 0.063f },   /* CORAL    — branching coral / dendrites              */
    { 0.046f, 0.063f },   /* WORMS    — slowly-travelling worms (Pearson κ)      */

    /* Tier 2 — Pearson 1993 letter regions */
    { 0.010f, 0.048f },   /* ALPHA    — slow / quiescent waves (Pearson α)       */
    { 0.014f, 0.045f },   /* BETA     — travelling-wave fingerprints (Pearson β) */
    { 0.026f, 0.055f },   /* GAMMA    — large-cell mitosis (Pearson γ proper)    */
    { 0.042f, 0.059f },   /* DELTA    — fingerprint-like (Pearson δ)             */
    { 0.018f, 0.055f },   /* EPSILON  — labyrinth+spots (Pearson ε)              */

    /* Tier 3 — named morphologies (Karl Sims / Munafo) */
    { 0.0367f, 0.0649f }, /* MITOSIS  — rapid spot division (Sims preset)        */
    { 0.014f,  0.054f },  /* SOLITON  — solitary travelling waves                */
    { 0.046f,  0.065f },  /* FINGERS  — finger-growth fronts (Pearson μ)         */
    { 0.062f,  0.0609f }, /* SKATE    — U-skate world (Munafo)                   */
    { 0.044f,  0.063f },  /* PSI      — ψ region (Munafo)                        */

    /* Tier 4 — dynamic / travelling patterns */
    { 0.014f,  0.045f },  /* WAVES    — travelling wave fronts                   */
    { 0.025f,  0.060f },  /* PULSES   — periodic radial pulses                   */
    { 0.018f,  0.051f },  /* NEBULA   — diffuse cloudy blobs                     */
    { 0.041f,  0.0635f }, /* NEURONS  — neuron-like network                      */
    { 0.046f,  0.0594f }, /* CHAOS    — chaotic mixing (Pearson ι)               */

    /* Tier 5 — extreme / complex regions */
    { 0.039f,  0.058f },  /* HOLES    — inverted spots (V dominates)             */
    { 0.012f,  0.052f },  /* BUBBLES  — large round blobs                        */
    { 0.018f,  0.053f },  /* SLUDGE   — slow dense gel-like                      */
    { 0.022f,  0.0585f }, /* PLANKTON — drifting spot fields                     */
    { 0.040f,  0.0608f }, /* CRYSTAL  — crystalline / sharp-edged                */
    { 0.060f,  0.062f },  /* THICKET  — dense branching network                  */
    { 0.020f,  0.057f },  /* DRIFT    — slowly drifting maze                     */
    { 0.026f,  0.052f },  /* ZEBRA    — tight high-contrast stripes              */
    { 0.034f,  0.060f },  /* BANDS    — loose broad stripes                      */
    { 0.0376f, 0.06277f },/* BARNACLE — clustered bumpy spots (Munafo frogspawn) */
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* §9 main loop — spiral-of-death dt clamp + terminal redraw cap
 * (Glenn Fiedler's "Fix Your Timestep"). */
#define DT_MAX_NS       (100 * NS_PER_MS)   /* hard cap on per-frame dt */
#define FRAME_CAP_FPS   60                  /* terminal redraw budget   */

/* V → palette band scaling. Multiplies a glow ∈ [0, 1] to land in
 * [0, N_PALETTE_BANDS − ε]; the int cast then floors cleanly into
 * [0, N_PALETTE_BANDS − 1]. */
#define N_PALETTE_BANDS      4
#define GLOW_COL_SCALE       3.999f

/* ── HUD layout (column widths) ──────────────────────────────────── *
 * Row-1 segments are laid out left-to-right at fixed column widths so
 * each segment knows where the next one starts. */
#define HUD_PATTERN_FIELD_W   20    /* " pattern:XXXXXXXXX " slot */
#define HUD_THEME_FIELD_W     17    /* " theme:XXXXXXXX "   slot */
#define HUD_PALETTE_LABEL_W    9    /* " palette:"          slot */

/*
 * Theme — one complete colour palette for the V-concentration display.
 * Ten of these live in themes[], cycled by t/T.
 *
 * INTENT. The smallest data needed to recolour every cell in the
 * render output: 4 ramp colours arranged dim → bright, plus a "flash"
 * accent (vestigial here — kept for cross-file palette parity with
 * magnetic_fields.c, midpoint_displacement_coastline.c, etc.).
 *
 * BANDING MODEL. scene_draw maps each cell's V·V_RENDER_GAIN value
 * into [0, 1], then derives a band index 0..3 via (g * 3.999f) & 3.
 * The band selects which colour of trail[] to use. Cells in the
 * lowest V tier get band[0] (dim); cells near peak V get band[3]
 * (bright). The four-band split forces theme authors to design the
 * ramp as a coherent low → high progression.
 *
 * COLOUR FORMAT. xterm-256 indices (NOT RGB). When the terminal
 * exposes fewer than 256 colours, theme_apply() substitutes a fixed
 * 8-colour cycle (COLOR_BLUE/CYAN/MAGENTA/YELLOW) so the demo still
 * runs on legacy TTYs.
 *
 * REFERENCE. The 256-colour cube layout (16 base + 6³ cube + 24-step
 * grayscale ramp) is documented in XTerm's ctlseqs.ms; the band values
 * in themes[] are picked from that cube. See also CLAUDE.md "Theme
 * Palette Brightness" for this project's bright-half constraint.
 */
typedef struct {
    const char *name;          /* short uppercase label shown in HUD     */
    short       band[4];       /* ramp: 0 = dim/low, 3 = bright          */
    short       flash;         /* vestigial — kept for cross-file parity */
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
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_BAND_BASE + i, t->band[i], -1);
        init_pair(PAIR_FLASH, t->flash, -1);
    } else {
        static const short fallback[4] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_BAND_BASE + i, fallback[i], -1);
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
/* §5  rd — Gray-Scott reaction-diffusion solver                          */
/* ===================================================================== */

/*
 * Grid — canvas geometry. Pure data: no buffers, no state. Lives at
 * the top of Scene because every layer (PDE step, raster, screen
 * centring) needs the dimensions.
 *
 * INVARIANT. w · h ≤ CELLS_MAX always holds; app_pick_map_size()
 * clamps to MAP_W_MAX × MAP_H_MAX = 200 × 56 = 11 200 cells, which is
 * what the ReactionField arrays are statically sized for.
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
 * ReactionField — the chemistry state on the grid. Holds both the
 * dynamical variables (U, V concentrations) and the PDE rate
 * constants (F, k) for the active preset.
 *
 *   U[]     : per-cell concentration of the "fed" chemical
 *   V[]     : per-cell concentration of the "consumed" chemical
 *   U_next, V_next : double-buffer for the explicit-Euler step
 *   F, k    : feed and kill rates of the active preset
 *
 * INTENT. ReactionField is the algorithm's data source — same role as
 * PoleField in magnetic_fields.c, FractalField in
 * midpoint_displacement_coastline.c, NoiseField in
 * perin_noise_flow_showcase.c. The PDE solver reads and writes ONLY
 * this struct; everything else (grid dims, sim counters, controls)
 * lives outside.
 *
 * BOUNDARY CONDITION. NEUMANN (zero-flux). Out-of-grid neighbours
 * are clamped to the centre cell, so no chemical mass leaves the
 * box. Equivalent to a sealed petri dish. Periodic BCs also work but
 * produce visible tile-seams when the central pattern reaches edges.
 *
 * REFERENCE. Gray, P. & Scott, S. K. (1983), Chem. Eng. Sci.; Pearson,
 * J. E. (1993), Science 261:189–192. See CONCEPTS for the PDE form.
 */
typedef struct {
    /* U[idx] — per-cell concentration of the "fed" chemical. Held at
     * the fixed point U = 1 by the F·(1 − U) term wherever V is
     * absent. Diffuses with rate D_u = GS_DU. */
    float U     [CELLS_MAX];

    /* V[idx] — per-cell concentration of the "consumed" chemical.
     * Starts as a small central blob; grows by consuming U via the
     * U·V² autocatalytic reaction; decays at rate (F + k)·V. Diffuses
     * with rate D_v = GS_DV < D_u — the asymmetric diffusion is
     * Turing's instability condition. */
    float V     [CELLS_MAX];

    /* U_next, V_next — double-buffer for the explicit-Euler step.
     * step_reaction_field writes the new values here, then
     * swap_pde_buffers memcpys them back into U/V. Necessary because
     * each cell's update depends on its neighbours' OLD values; we
     * can't update U in place without corrupting the neighbours we
     * haven't visited yet. */
    float U_next[CELLS_MAX];
    float V_next[CELLS_MAX];

    /* F — feed rate. Installed by install_kinetics() from the active
     * preset; constant during the rest of the simulation. Active
     * range F ∈ [0.010, 0.075]; see GSPreset doc. */
    float F;

    /* k — kill rate. Same lifecycle as F. Active range
     * k ∈ [0.040, 0.070]. Outside the active region the system
     * collapses to U=1, V=0 and the screen goes empty. */
    float k;
} ReactionField;

/* ── Gray-Scott PDE primitives ─────────────────────────────────── *
 * Each one expresses exactly one named operation from the algorithm.
 * The step_reaction_field body composes them so the loop reads as
 * the PDE equations almost line for line. */

/* clamp_neumann_lo / clamp_neumann_hi — zero-flux boundary lookup:
 * step back one cell unless we're at the edge, in which case use the
 * edge itself. Together they implement Neumann boundary conditions
 * without an explicit ghost-cell ring. */
static inline int clamp_neumann_lo(int c)        { return (c > 0)     ? c - 1 : c; }
static inline int clamp_neumann_hi(int c, int n) { return (c < n - 1) ? c + 1 : c; }

/* laplacian_5pt — discrete 5-point Laplacian stencil:
 *   ∇²f(x,y) ≈ f(x-1,y) + f(x+1,y) + f(x,y-1) + f(x,y+1) − 4·f(x,y)
 * Used for both U and V diffusion. */
static inline float laplacian_5pt(const float *f,
                                   int idx, int il, int ir, int iu, int id)
{
    return f[il] + f[ir] + f[iu] + f[id] - 4.0f * f[idx];
}

/* gray_scott_dU — right-hand side of the U PDE:
 *   ∂U/∂t = D_u · ∇²U − U·V² + F·(1 − U)
 * Returns the time-derivative; forward-Euler multiplies by dt. */
static inline float gray_scott_dU(float u, float v, float lap_U,
                                   float F, float Du)
{
    return Du * lap_U - u * v * v + F * (1.0f - u);
}

/* gray_scott_dV — right-hand side of the V PDE:
 *   ∂V/∂t = D_v · ∇²V + U·V² − (F + k)·V
 * Returns the time-derivative; forward-Euler multiplies by dt. */
static inline float gray_scott_dV(float u, float v, float lap_V,
                                   float F, float k, float Dv)
{
    return Dv * lap_V + u * v * v - (F + k) * v;
}

/* swap_pde_buffers — copy *_next back to current for the next sub-step.
 * memcpy is fine; both arrays are ~45 KB at the worst-case grid. */
static void swap_pde_buffers(ReactionField *rxn, int total_cells)
{
    size_t bytes = (size_t)total_cells * sizeof(float);
    memcpy(rxn->U, rxn->U_next, bytes);
    memcpy(rxn->V, rxn->V_next, bytes);
}

/* ── seed / step drivers ──────────────────────────────────────── */

/*
 * SEED_BLOB_FRAC — central seed blob radius as a fraction of the
 * shorter grid edge. 1/12 gives a small but reliable nucleus.
 * U_BLOB / V_BLOB — concentrations inside the seed blob; the V > 0
 * patch is what kicks the system off its trivial fixed point.
 * JITTER_RANGE / JITTER_GAIN — symmetry-breaking U noise; small
 * enough not to perturb the kinetics, large enough that nearby
 * cells diverge instead of producing concentric rings.
 */
#define SEED_BLOB_RADIUS_DENOM  12
#define SEED_BLOB_MIN_RADIUS    3
#define SEED_U_BLOB             0.5f
#define SEED_V_BLOB             0.25f
#define JITTER_RANGE            100        /* uniform sample in [0, JITTER_RANGE) */
#define JITTER_HALFRANGE         50        /* centre jitter on 0: sample - half  */
#define JITTER_GAIN             0.0002f    /* multiplier; keeps jitter tiny      */
#define LAPLACIAN_CENTRE_WEIGHT 4.0f   /* the "-4·f" in 5-pt stencil */
#define U_FIXED_POINT           1.0f   /* trivial steady state */

/* clear_concentrations — set every cell to the trivial fixed point
 * (U = 1, V = 0). Starting from this and ONLY this leaves the system
 * frozen; the seed_central_blob + symmetry-breaking jitter are what
 * actually kick it into morphogenesis. */
static void clear_concentrations(ReactionField *rxn, int total_cells)
{
    for (int i = 0; i < total_cells; i++) {
        rxn->U[i] = U_FIXED_POINT;
        rxn->V[i] = 0.0f;
    }
}

/* seed_central_blob — paint a small disc of (U = 0.5, V = 0.25) at the
 * centre. Provides the non-zero V patch the PDE needs to escape the
 * trivial fixed point. Without this, U stays at 1 and V stays at 0
 * forever. */
static void seed_central_blob(ReactionField *rxn, const Grid *g)
{
    int cx = g->w / 2;
    int cy = g->h / 2;
    int rad = (g->w < g->h ? g->w : g->h) / SEED_BLOB_RADIUS_DENOM;
    if (rad < SEED_BLOB_MIN_RADIUS) rad = SEED_BLOB_MIN_RADIUS;
    int rad_sq = rad * rad;
    for (int dy = -rad; dy <= rad; dy++) {
        for (int dx = -rad; dx <= rad; dx++) {
            int x = cx + dx, y = cy + dy;
            if (!grid_in_bounds(g, x, y))   continue;
            if (dx * dx + dy * dy > rad_sq) continue;
            int idx = grid_idx(g, x, y);
            rxn->U[idx] = SEED_U_BLOB;
            rxn->V[idx] = SEED_V_BLOB;
        }
    }
}

/* salt_with_symmetry_breaking_jitter — add tiny random perturbations
 * to U everywhere. Without this the seed blob would only produce
 * perfectly symmetric concentric rings, because the PDE preserves the
 * radial symmetry of its initial condition. The morphogenic
 * instability needs SOME asymmetry to amplify. */
static void salt_with_symmetry_breaking_jitter(ReactionField *rxn, int total_cells)
{
    for (int i = 0; i < total_cells; i++) {
        int   centred = rand() % JITTER_RANGE - JITTER_HALFRANGE;   /* [-50, +49] */
        float jitter  = (float)centred * JITTER_GAIN;
        rxn->U[i] += jitter;
    }
}

/* seed_reaction_field — the full init pipeline: trivial fixed point,
 * central seed blob, symmetry-breaking jitter. */
static void seed_reaction_field(ReactionField *rxn, const Grid *g)
{
    clear_concentrations                 (rxn, g->total_cells);
    seed_central_blob                    (rxn, g);
    salt_with_symmetry_breaking_jitter   (rxn, g->total_cells);
}

/*
 * step_reaction_field — run one Gray-Scott sub-step on the entire
 * grid. Forward Euler with a 5-point Laplacian, Neumann boundaries.
 * Reads as the PDE equations: laplacian, then PDE right-hand side,
 * then forward-Euler integration step.
 */
static void step_reaction_field(ReactionField *rxn, const Grid *g)
{
    const float Du = GS_DU, Dv = GS_DV, dt = GS_DT;
    const float F  = rxn->F, k = rxn->k;
    const int   w  = g->w,   h = g->h;

    for (int y = 0; y < h; y++) {
        int yu = clamp_neumann_lo(y);
        int yd = clamp_neumann_hi(y, h);
        for (int x = 0; x < w; x++) {
            int xl = clamp_neumann_lo(x);
            int xr = clamp_neumann_hi(x, w);

            int idx = y  * w + x;
            int il  = y  * w + xl;
            int ir  = y  * w + xr;
            int iu  = yu * w + x;
            int id  = yd * w + x;

            float u = rxn->U[idx];
            float v = rxn->V[idx];

            float lap_U = laplacian_5pt(rxn->U, idx, il, ir, iu, id);
            float lap_V = laplacian_5pt(rxn->V, idx, il, ir, iu, id);

            rxn->U_next[idx] = u + dt * gray_scott_dU(u, v, lap_U, F, Du);
            rxn->V_next[idx] = v + dt * gray_scott_dV(u, v, lap_V, F, k, Dv);
        }
    }
    swap_pde_buffers(rxn, g->total_cells);
}

/* ===================================================================== */
/* §6  patterns — preset (F, k) plug-ins                                  */
/* ===================================================================== */

/*
 * install_kinetics — copy a preset's (F, k) rate constants into the
 * ReactionField. The actual simulation kernel reads only rxn->F and
 * rxn->k; the Pattern enum and gs_presets table never escape §6.
 * Caller is responsible for re-seeding the field after switching.
 */
static void install_kinetics(ReactionField *rxn, Pattern p)
{
    if (p < 0 || p >= N_PATTERNS) p = PATTERN_SPOTS;
    rxn->F = gs_presets[p].F;
    rxn->k = gs_presets[p].k;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * The scene is built out of four small sub-structs (ReactionField from
 * §5 plus three defined below). Each owns ONE concern, and Scene
 * composes them. Splitting concerns into clearly-typed sub-structs
 * makes function signatures self-describing: any function that takes
 * `const Grid *` clearly cannot mutate the chemistry; any function
 * that takes `ReactionField *` clearly does not advance sim time; and
 * so on. Same architectural shape as the other field files
 * (magnetic_fields.c, flow_field_particles.c, midpoint_displacement_*).
 */

/*
 * SimState — the single mutable per-tick scalar. Separated from
 * Controls (keyboard-driven knobs) so it is obvious what scene_tick
 * mutates vs. what the user does.
 *
 * step_count is the total number of Gray-Scott sub-steps run since
 * the last seed (i.e. since the last scene_reset). The HUD shows it
 * so the user can compare "this pattern needed N steps to settle"
 * across presets.
 */
typedef struct {
    int step_count;       /* total sub-steps since last seed */
} SimState;

/*
 * Controls — user-facing knobs. Mutated only by app_handle_key(),
 * read by scene_tick (paused, sub_steps) and screen_draw (theme,
 * pattern).
 *
 * INTENT. Same model-vs-user-intent split as the other field files:
 * dependency is strictly one-way. app_handle_key writes Controls,
 * never SimState; scene_tick reads Controls but only writes SimState
 * + ReactionField. Keeps the code linear.
 *
 * NO prev_pattern. Switching patterns triggers a full scene_reset
 * (see cycle_pattern in §9) because each preset needs its own
 * (F, k) AND a fresh seed — there's no need to detect the switch
 * with a one-tick lag.
 */
typedef struct {
    bool    paused;
    int     sub_steps;              /* Gray-Scott updates per frame */
    int     current_theme;
    Pattern current_pattern;
} Controls;

/*
 * Scene — the umbrella context. Reading this struct top-to-bottom
 * is meant to be the fastest way to understand the program:
 *   grid       → where things live       (geometry)
 *   reaction   → chemistry state + rate  (ReactionField — U/V + F/k)
 *   sim        → simulation counter      (step_count)
 *   ctrl       → user knobs              (pattern, sub_steps, theme, …)
 *
 * ORDERING. Each sub-struct depends only on those declared above it:
 * grid is leaf-level; reaction is sized by CELLS_MAX (upper bound on
 * grid); sim mutates reaction via step_reaction_field; ctrl decides
 * which sim path runs. A reader scanning top-down meets every concept
 * before it is used.
 */
typedef struct {
    Grid          grid;       /* canvas dimensions                          */
    ReactionField reaction;   /* U, V concentrations + F, k kinetics        */
    SimState      sim;        /* step_count — mutated only by scene_tick    */
    Controls      ctrl;       /* mutated only by app_handle_key             */
} Scene;

/* ── reset / init pipeline ───────────────────────────────────────── *
 * scene_reset is the init/reset pseudocode:
 *
 *     apply grid dimensions.
 *     install kinetics from the active preset (F, k).
 *     seed the reaction field (U=1 everywhere + central V-blob + jitter).
 *     reset sim state (step_count = 0).
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
    sim->step_count = 0;
}

static void scene_reset(Scene *s, int w, int h)
{
    apply_grid_dimensions  (&s->grid, w, h);
    install_kinetics       (&s->reaction, s->ctrl.current_pattern);
    seed_reaction_field    (&s->reaction, &s->grid);
    reset_sim_state        (&s->sim);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->ctrl.paused           = false;
    s->ctrl.sub_steps        = SUB_STEPS_DEF;
    s->ctrl.current_theme    = 0;
    s->ctrl.current_pattern  = PATTERN_SPOTS;
    scene_reset(s, w, h);
}

/* ── tick pipeline ───────────────────────────────────────────────── *
 * scene_tick is the per-frame pseudocode:
 *
 *     if paused: stop.
 *     run sub_steps Gray-Scott updates, counting each.
 *
 * Each step is one named call. */

static void run_sub_steps(Scene *s)
{
    int n = s->ctrl.sub_steps;
    for (int i = 0; i < n; i++) {
        step_reaction_field(&s->reaction, &s->grid);
        s->sim.step_count++;
    }
}

static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;
    (void)dt;     /* Gray-Scott uses fixed GS_DT, not wall dt */

    run_sub_steps(s);
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
 * glyph, plus a skip flag. classify_v_cell decides; paint_cell acts.
 * Routing the per-cell decision through this struct keeps the
 * decision logic pure (no ncurses I/O inside the classifier) and
 * concentrates ncurses calls into ONE place (paint_cell).
 */
typedef struct {
    int  pair;
    int  attr;
    char glyph;
    bool skip;
} CellDraw;

/* normalize_v_to_glow — map raw V ∈ [0, V_peak] to glow ∈ [0, 1]. V
 * typically peaks at ~0.4 for our presets, so we multiply by
 * V_RENDER_GAIN ≈ 2.5 for visibility, then clamp into the unit
 * interval. The clamped glow drives the glyph-ramp threshold logic. */
static inline float normalize_v_to_glow(float v_raw)
{
    float g = v_raw * V_RENDER_GAIN;
    if (g < 0.0f) g = 0.0f;
    if (g > 1.0f) g = 1.0f;
    return g;
}

/* glow_to_band — derive the palette band 0..N_PALETTE_BANDS-1 from
 * the normalised glow. Same trick as in perin_noise_flow_showcase.c:
 * multiply by GLOW_COL_SCALE = 4 − ε so int-cast floors cleanly. */
static inline int glow_to_band(float glow)
{
    return (int)(glow * GLOW_COL_SCALE) & (N_PALETTE_BANDS - 1);
}

/* classify_v_cell — pure function. Given a cell's normalised glow,
 * return how it should be rendered (or .skip = true if below the
 * visibility floor). Three density tiers map to the three glyphs
 * of the Paul-Bourke ASCII ramp. */
static CellDraw classify_v_cell(float glow)
{
    if (glow > GLYPH_HIGH_THRESH) {
        int pair = PAIR_BAND_BASE + glow_to_band(glow);
        return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = '#' };
    }
    if (glow > GLYPH_MID_THRESH) {
        int pair = PAIR_BAND_BASE + glow_to_band(glow);
        return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = '*' };
    }
    if (glow > GLOW_THRESHOLD) {
        int pair = PAIR_BAND_BASE + glow_to_band(glow);
        return (CellDraw){ .pair = pair, .attr = A_NORMAL, .glyph = '.' };
    }
    return (CellDraw){ .skip = true };
}

/* paint_cell — the ONE ncurses I/O point for the V-concentration layer. */
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
 * scene_draw — walk every cell, normalise its V to a glow, classify,
 * paint. The loop body is exactly three named operations.
 */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const Grid          *g   = &s->grid;
    const ReactionField *rxn = &s->reaction;
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
                        classify_v_cell(normalize_v_to_glow(rxn->V[idx])));
        }
    }
}

/* ── HUD draw pipeline ───────────────────────────────────────────── *
 * Four named drawers, called from screen_draw in z-order. The TOP HUD
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
             " %5.1f fps  %3d Hz  %s [%d/%d]  sub:%-3d  step:%-7d ",
             fps, sim_fps, state_str,
             (int)c->current_pattern + 1, N_PATTERNS,
             c->sub_steps, s->sim.step_count);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void draw_hud_title(void)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " GRAY-SCOTT REACTION-DIFFUSION ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* ── row 1 segment drawers ───────────────────────────────────────── *
 * draw_hud_status_line lays out row 1 left-to-right as a sequence of
 * fixed-width segments. Each segment paints its content and returns
 * the new x-cursor; the last (kinetics_readout) does not need to
 * return. */

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
    for (int i = 0; i < N_PALETTE_BANDS; i++) {
        int pair = PAIR_BAND_BASE + i;
        attron (COLOR_PAIR(pair) | A_BOLD);
        mvaddch(row, x, '#');
        attroff(COLOR_PAIR(pair) | A_BOLD);
        x++;
    }
    return x;
}

/* draw_kinetics_readout — last segment: live (F, k, D_u, D_v) rate
 * constants + map dimensions. No return — end of the row. The
 * numbers update when the user cycles presets. */
static void draw_kinetics_readout(int row, int x, const Scene *s)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  F=%.4f  k=%.4f  Du=%.2f  Dv=%.2f  map:%dx%d ",
             (double)s->reaction.F, (double)s->reaction.k,
             (double)GS_DU, (double)GS_DV,
             s->grid.w, s->grid.h);
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
        draw_kinetics_readout     (1, x, s);
}

/* draw_bottom_hint — row N-1: ACTIONS only (key bindings). */
static void draw_bottom_hint(const Screen *sc)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  r:reset  spc:pause  +/-:sub-steps  ]/[:Hz  q:quit ");
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
/* §9  app                                                                */
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

/* bump_sub_steps — '+' / '-' geometric step on per-frame sub-step count.
 * Doubling/halving gives a logarithmic feel — small/medium/large/extreme
 * are spaced exponentially rather than linearly. */
static void bump_sub_steps(Controls *c, int dir)
{
    if (dir > 0) {
        if (c->sub_steps < SUB_STEPS_MAX) c->sub_steps *= 2;
        if (c->sub_steps > SUB_STEPS_MAX) c->sub_steps = SUB_STEPS_MAX;
    } else {
        c->sub_steps /= 2;
        if (c->sub_steps < SUB_STEPS_MIN) c->sub_steps = SUB_STEPS_MIN;
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

/* cycle_pattern — n/p step + full scene reset. We reset because each
 * preset has its own (F, k) AND benefits from a fresh seed so the
 * user watches the new morphology self-organise from scratch. */
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
    case ' ':           c->paused = !c->paused;                              break;
    case 'r': case 'R': scene_reset(&app->scene, app->map_w, app->map_h);    break;
    case '=': case '+': bump_sub_steps (c,   +1);                            break;
    case '-':           bump_sub_steps (c,   -1);                            break;
    case ']':           bump_sim_fps   (app, +SIM_FPS_STEP);                 break;
    case '[':           bump_sim_fps   (app, -SIM_FPS_STEP);                 break;
    case 't':           cycle_theme    (c,   +1);                            break;
    case 'T':           cycle_theme    (c,   -1);                            break;
    case 'n': case 'N': cycle_pattern  (app, +1);                            break;
    case 'p': case 'P': cycle_pattern  (app, -1);                            break;
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
