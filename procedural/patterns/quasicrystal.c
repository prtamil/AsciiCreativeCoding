/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * quasicrystal.c
 *   — Quasicrystal interference patterns: sum N plane waves at
 *     angles θ_k = k·π/N over the screen, animate by drifting each
 *     wave's phase. Choosing N coprime with 2, 3, 4, 6 (the Bravais
 *     symmetries) yields APERIODIC long-range order — rotational
 *     symmetry without translational repetition. The same recipe
 *     that produces real-world quasicrystals like the Penrose tiling.
 *
 * DEMO: Concentric stars and rosettes flicker into existence and
 *       slowly morph as the wave phases drift. With 5-fold symmetry
 *       you see Penrose-like 10-pointed stars; 7-fold gives 14-pointed
 *       stars; 11-fold is so dense the pattern looks almost cloud-like
 *       at small scales but breaks into intricate symmetric stars at
 *       large.
 *
 *       PATTERN (n / p) — cycle a gallery of 15 presets, shown in the
 *       top HUD as "i/15 NAME".  Each preset tunes the wave count N
 *       (rotational symmetry) and the wavelength λ (density); a few
 *       repeat the most striking orders at fine/giant scales:
 *
 *         HEX-3 .. STAR-13   N = 3..13 sweep (periodic 3/4/6/12
 *                            crystals interleaved with 5/7/8/9/10/11/13
 *                            quasicrystals for contrast)
 *         FINE-5 / GIANT-5   the 10-fold look, dense vs huge
 *         WEAVE-7 / NOVA-11  dense septagonal weave vs big 11-fold rosettes
 *
 *       See the presets[] table in §1 for the exact N / λ of each.
 *
 *       GLYPH (g / G) — how the intensity field is rendered:
 *
 *         RAMP      8-step density ramp across full [-1, +1] range
 *         PEAKS     only positive intensity drawn (highlights)
 *         CONTOUR   only zero-crossings shown (wave fronts)
 *         WAVES     bipolar — peaks bright, troughs dim
 *
 *       'r' randomises the global phase, snapping the pattern to a
 *       fresh state. The phase offset persists; subsequent drift
 *       continues smoothly from there.
 *
 * Study alongside:
 *   ../patterns/truchet_tiles.c
 *      — also a "static-pattern + slow drift" showcase, but
 *        Truchet's structure is per-cell hash; here it's a continuous
 *        scalar field.
 *   ../fields/perin_noise_flow_showcase.c
 *      — Perlin-driven flow vs cosine-sum-driven interference.
 *        Both produce smooth scalar fields; cosine sums uniquely give
 *        rotational long-range order.
 *
 * Section map (cut by CONCERN, not by subsystem — see ARCHITECTURE below):
 *   §1 CONFIG       — constants, presets, glyph/theme tables (data only)
 *   §2 PERFORMANCE  — monotonic timer + sleep (accumulator/frame-cap in §6 main)
 *   §3 LOGIC        — pure decisions: glyph name, hash, intensity→glyph
 *   §4 SIMULATION   — wave vectors, intensity sampler, Scene, scene_tick
 *   §5 RENDER       — colour/themes, screen, field + flash + HUD draw
 *   §6 APP          — signals, resize, input, fixed-step combine (main)
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume phase drift
 *   r          randomise phase offset (snap to new state)
 *   n / N      next preset     (cycles the 15-preset gallery)
 *   p / P      previous preset
 *   g / G      next / previous glyph set
 *   t / T      next / previous theme
 *   + / =      faster phase drift
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/patterns/quasicrystal.c \
 *     -o quasicrystal -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Plane-wave interference. Define N wave vectors
 *
 *                    k̂_m = (cos θ_m, sin θ_m)         θ_m = m · π / N
 *
 *                  for m = 0, 1, … , N−1. Pick a common spatial
 *                  frequency ω = 2π / λ. Pick per-wave phases
 *
 *                    φ_m(t) = t · (r_base + m · r_delta)
 *
 *                  drifting at slightly different rates so the pattern
 *                  morphs over time rather than rigidly translating.
 *                  The intensity field is the sum of N cosine waves:
 *
 *                    I(x, y, t)
 *                      = (1/N) · Σ_m cos( ω·(x·cos θ_m + y·sin θ_m)
 *                                          + φ_m(t) )
 *
 *                  ≈ in [-1, +1].  Renderer maps each cell's intensity
 *                  through one of four glyph sets:
 *
 *                    RAMP    : intensity ∈ [-1, +1] → level 0..7
 *                              → glyph from ' .,:-o#@' ramp.
 *                    PEAKS   : negative intensity → blank;
 *                              positive intensity → bright glyph.
 *                    CONTOUR : |intensity| < ε → glyph;  else blank.
 *                              Renders only the zero-crossings —
 *                              the wave-front network.
 *                    WAVES   : positive → bright (peak crests);
 *                              negative → dim (trough valleys).
 *
 *                  Why N=5, 7, 11 give APERIODIC patterns: the
 *                  Crystallographic Restriction Theorem says only
 *                  rotational symmetries of order 1, 2, 3, 4, 6 can
 *                  arise in 2-D periodic lattices. A cosine-sum of
 *                  N waves at θ_m = m·π/N has 2N-fold rotational
 *                  symmetry; for N coprime with {1, 2, 3, 4, 6} the
 *                  resulting symmetry order (10, 14, 22 etc.) cannot
 *                  be a Bravais lattice — the pattern necessarily
 *                  has long-range order without translational
 *                  periodicity, the defining property of a
 *                  quasicrystal.
 *
 * Data-structure : A WaveVectors struct — N precomputed unit wave vectors
 *                  (parallel cos_theta[]/sin_theta[] arrays) — refreshed when
 *                  the pattern (and thus N) changes, held on the Scene.  No
 *                  grid is stored; every cell's intensity is recomputed each
 *                  frame.
 *
 * Rendering      : ASCII only. Per cell: N cos calls + a few
 *                  multiplies for the dot product with each wave
 *                  vector. With N=11, that's 11 cos/cell, ~12.7M cos
 *                  per second on a 240×80 × 60 fps screen. Around
 *                  6 % of one CPU core. No allocation per frame.
 *
 * Performance    : Dominated by the N cosine evaluations per cell.
 *                  Could be optimised by precomputing a 1-D table of
 *                  cos(ω·distance) values at integer step lengths
 *                  per wave, but the straightforward version is
 *                  already fast enough at terminal resolutions.
 *
 * References     : Theory first, then how to construct & draw it.
 *
 *   CONCEPTS — quasicrystals & aperiodic order
 *   • Levine, D. & Steinhardt, P.J. (1984) — "Quasicrystals: A New
 *     Class of Ordered Structures", Phys. Rev. Lett. 53(26):2477.
 *     Coined "quasicrystal" and modelled the density as a SUM OF
 *     PLANE WAVES — the exact picture this file renders.  Best
 *     starting point for the algorithm.
 *   • Shechtman, D., Blech, I., Gratias, D. & Cahn, J.W. (1984) —
 *     "Metallic Phase with Long-Range Orientational Order and No
 *     Translational Symmetry", Phys. Rev. Lett. 53(20):1951.  The
 *     experimental discovery; Shechtman's 2011 Nobel Prize.
 *   • Senechal, M. (1995) — "Quasicrystals and Geometry", Cambridge
 *     Univ. Press.  Definitive geometry/symmetry text — why 5-, 7-,
 *     11-fold order cannot be periodic.
 *   • Baake, M. & Grimm, U. (2013) — "Aperiodic Order, Vol. 1: A
 *     Mathematical Invitation", Cambridge Univ. Press.  Modern,
 *     comprehensive reference for the whole field.
 *   • Grünbaum, B. & Shephard, G.C. (1987) — "Tilings and Patterns",
 *     W.H. Freeman, ch. 10.  Penrose tilings & aperiodicity (the
 *     real-space cousin of these interference patterns).
 *   • Wikipedia — "Crystallographic restriction theorem"
 *     https://en.wikipedia.org/wiki/Crystallographic_restriction_theorem
 *     Short proof that periodic lattices admit only 1,2,3,4,6-fold
 *     symmetry — the rule that makes N = 5,7,11… aperiodic.
 *
 *   RENDERING — building the pattern from summed waves
 *   • McAllister, K. (2011) — "Quasicrystals as sums of waves in the
 *     plane", mainisusuallyafunction.blogspot.com/2011/10/ .  A
 *     step-by-step build of exactly this cosine-sum-of-N-plane-waves
 *     animation; the closest match to the code here.
 *   • Bostock, M. — "Quasicrystals" (interactive D3 demo)
 *     https://gist.github.com/mbostock/3019563 .  Live sliders for N
 *     and phase; good for building intuition before reading code.
 *   • Bourke, P. — "Character representation of grey scale images",
 *     https://paulbourke.net/dataformats/asciiart/ .  The intensity→
 *     ASCII-ramp mapping behind RAMP_GLYPHS.
 *   • De Bruijn, N.G. (1981) — "Algebraic theory of Penrose's
 *     non-periodic tilings of the plane, I & II", Indag. Math.
 *     43:39-66.  The dual multigrid construction — the OTHER way to
 *     generate the same N-fold symmetry.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To make a pattern with rotational symmetry but no translational
 * periodicity, sum cosine waves in N evenly-spaced directions
 * around the circle. If N is coprime with the lattice symmetries
 * (1, 2, 3, 4, 6), the resulting interference cannot tile
 * periodically — yet it still has perfect rotational symmetry.
 * That is a QUASICRYSTAL: long-range orientational order without
 * a unit cell.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Drop a stone in a still pond and you get circular ripples.
 * Drop two stones at opposite ends and where the ripple-fronts
 * meet, you get an interference pattern — bright bands where the
 * waves add, dark bands where they cancel. Now imagine N stones
 * dropped in a perfect circle around the pond, all at the same
 * moment. Each stone emits a wave; the waves all reach the pond
 * centre simultaneously and cross at angles 2π/N apart.
 *
 * Look down at the surface from above. The interference pattern
 * has N-fold (or 2N-fold) rotational symmetry — rotate it by
 * 2π/N and it looks the same. But for "weird" N like 5, 7, 11,
 * it CANNOT be the result of stamping a single tile periodically;
 * the symmetry is incompatible with any wallpaper group. The
 * pattern is LITERALLY EVERYWHERE, infinite, intricate, ordered,
 * yet has no repeating unit. That is a quasicrystal.
 *
 * Now let each stone's ripple drift at its own slow rate. The
 * pattern is still N-fold symmetric at every moment, but it
 * MORPHS — stars shift, troughs deepen, new constellations of
 * peaks form and dissolve. That is the animation.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. CHOOSE N (the wave count). Precompute the WaveVectors:
 *     cos_theta[m], sin_theta[m] for m = 0..N-1 with angle m·π/N.
 *  2. EACH FRAME:
 *     a. Advance global time t.
 *     b. For every screen cell (sx, sy):
 *        i.   x = sx;  y = sy · ASPECT_Y      // aspect correction
 *        ii.  intensity = 0
 *        iii. For each wave m:
 *               wx     = x · cos_theta[m] + y · sin_theta[m]
 *               φ_m    = (t + offset) · (r_base + m · r_delta)
 *               intensity += cos( ω·wx + φ_m )
 *        iv.  intensity /= N                  // → ≈ [-1, +1]
 *        v.   Map intensity → glyph + colour via active GlyphSet.
 *        vi.  mvaddch(sy, sx, glyph) in selected pair + attr.
 *  3. HUD on top.
 *
 * KEY FORMULAS
 * ────────────
 *  Wave vector for wave m:
 *    θ_m   = m · π / N                  ω = 2π / λ
 *    k̂_m  = (cos θ_m, sin θ_m)
 *
 *  Intensity at (x, y, t):
 *    I = (1/N) · Σ_{m=0..N-1}  cos( ω · k̂_m · (x, y) + φ_m(t) )
 *
 *  Per-wave phase rate (gives morphing animation):
 *    φ_m(t) = t · (r_base + m · r_delta)
 *
 *  Intensity → ramp level (RAMP glyph):
 *    level = clamp(⌊(I + 1)·4⌋, 0, 7)
 *
 *  Zero-crossing (CONTOUR glyph):
 *    drawn when |I| < ε ; brighter when closer to zero.
 *
 *  Bipolar (WAVES glyph):
 *    I > +T_high → core peak '#'
 *    I > 0       → mid peak  '*'
 *    I > -T_high → mid trough '.'
 *    else         → core trough ','
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • N COPRIME RULE. Periodic vs aperiodic depends on N. N = 3 and
 *    N = 6 give visually-perfect HEXAGONAL periodic patterns —
 *    useful for contrast in the demo, but they are NOT
 *    quasicrystals. The rule: N must be coprime with the divisors
 *    of 12 (for 2-D Bravais). 5, 7, 11 work; 3 doesn't.
 *
 *  • COSINE COUNT AT HIGH N. The densest preset, STAR-13, runs 13
 *    cosines per cell × 19 200 cells × 60 fps ≈ 15 M cos calls/s.
 *    Modern CPUs handle it (~7 % of one core). If you raise
 *    N_WAVES_MAX and push N higher, expect framerate to drop.
 *
 *  • ASPECT CORRECTION. Multiply screen y by ASPECT_Y_F (=2) when
 *    sampling so the pattern's circles look round on terminals
 *    where cells are 2× taller than wide. Without it, the
 *    quasicrystal stars become horizontal ovals.
 *
 *  • PHASE-RATE SPREAD. If all waves drift at the SAME rate, the
 *    pattern just translates rigidly across the screen — no
 *    morphing. A small per-wave delta (k · rate_delta, per preset)
 *    makes the relative phases shift, so the pattern visibly
 *    REORGANISES, which is much more interesting.
 *
 *  • CONTOUR THRESHOLD AT N=11. With 11 waves the intensity field
 *    has very fine-scale structure; |I|<0.05 leaves visible gaps in
 *    the contour. For higher N, widen the threshold (or use a
 *    derivative-based contour) so the lines stay continuous.
 *
 *  • RANDOMISE PHASE. 'r' adds a random offset to t. The drift
 *    continues from there, so the offset persists across frames.
 *    Don't subtract the offset on the next frame — that would
 *    snap it back.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • PAUSE (space). Pattern freezes. Resume: drift continues from
 *    exactly where it stopped. Verifies the time accumulator.
 *
 *  • PENROSE-5 preset. Look for 10-pointed stars (5-fold symmetry, but
 *    cosine doubles it). Counting points around the brightest peak
 *    near the centre should give 10. The Penrose-tiling kinship is
 *    visible.
 *
 *  • SEPTA-7 preset. Stars now have 14 points. The pattern is
 *    visibly denser and finer than PENROSE-5 at the same wavelength.
 *
 *  • UNDECA-11 preset. Visual density makes individual stars hard to
 *    count, but rotational symmetry is still perfect about any
 *    centre. Pause and rotate your head 360°/22 ≈ 16.4° — the
 *    pattern looks the same.
 *
 *  • HEX-3 preset (N=3). You'll see HEXAGONAL periodic structure —
 *    cells repeat in a tilable lattice. This is NOT a quasicrystal;
 *    it's included for comparison so the difference is visible
 *    when you switch to PENROSE-5.
 *
 *  • CONTOUR glyph. The drawn lines should form a network of
 *    closed curves — these are the level sets I = 0 of the cosine
 *    sum. They DO NOT cross periodically.
 *
 *  • PEAKS glyph. Should show only the bright cells (positive
 *    intensity). Half the screen (where I < 0) is blank. The
 *    visible "stars" are the wave crests.
 *
 *  • Theme cycle (t / T). Each theme should produce visible
 *    contrast. RAMP and WAVES use the full ramp[0..7]; PEAKS uses
 *    ramp[3..7]; CONTOUR uses bright slots.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * The file is cut into six concern-layers.  Each layer owns a section and a
 * well-defined right to mutate state; nothing reaches across.
 *
 *   LAYER         SECTION   MUTATES
 *   ───────────   ───────   ─────────────────────────────────────────────
 *   CONFIG        §1        nothing — const tables (presets, themes, ramp)
 *   PERFORMANCE   §2 + §6   main-local timing only (frame_time, sim_accum,
 *                           fps_accum, frame_count); never Scene
 *   LOGIC         §3        nothing — pure functions (args → value / out-
 *                           params); no globals written, no I/O
 *   SIMULATION    §4        Scene.{time_secs, phase_offset, flash_t} via
 *                           scene_tick / scene_reseed; Scene.waves
 *                           (WaveVectors) via waves_init
 *   RENDER        §5        ncurses screen + colour pairs only; Screen.{cols,
 *                           rows} on init/resize.  NEVER mutates Scene
 *   APP/EVENTS    §6        Scene.{paused, speed, current_*}, App.{sim_fps,
 *                           running, need_resize} via keys/signals
 *
 * Two concerns are too small for their own section (documented, not faked):
 *   EFFECTS — one cosmetic scalar, Scene.flash_t (the reseed flash).  Set in
 *             scene_reseed, decayed in scene_tick (SIMULATION), drawn as an
 *             overlay in scene_draw (RENDER).  No trail/glow buffers exist.
 *   DELAYS  — one pause gate, Scene.paused, checked at the top of scene_tick.
 *             No holds, countdowns, or timed transitions.
 *
 * compute_intensity (§4) is a PURE LOGIC sampler — no mutation, no I/O — now
 * that it takes the WaveVectors as a const parameter it reads no globals at
 * all.  It is listed under LOGIC's guarantees, only physically grouped beside
 * the WaveVectors type it samples.
 *
 * PER-TICK COMBINE ORDER (the ONE place state advances — main, §6):
 *   1. PERFORMANCE  measure real dt (clock_ns), clamp to 100 ms
 *   2. SIMULATION   drain the fixed-step accumulator → scene_tick(dt) ×K
 *   3. PERFORMANCE  fps accounting + sleep to the 60 fps frame cap
 *   4. RENDER       screen_draw (field + flash + HUD) → screen_present
 * User events — resize (SIGWINCH), keys (app_handle_key) — mutate state but
 * are NOT part of the tick; they run before/after it, never inside scene_tick.
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
/* §1  CONFIG — constants, presets, glyph/theme tables (data only)        */
/* ===================================================================== */

enum {
    SIM_FPS_MIN         =  10,
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,
    SIM_FPS_STEP        =  10,

    /* Render-loop pacing (main).  The sim runs at sim_fps; the DRAW loop is
     * capped separately so a fast machine doesn't spin redrawing. */
    RENDER_FPS_CAP      =  60,    /* hard frame-rate cap (Hz) — per-frame sleep target  */
    MAX_FRAME_DT_MS     = 100,    /* clamp a stalled frame's dt to dodge the spiral of death */

    SPEED_MIN           =   1,
    SPEED_DEF           =   8,
    SPEED_MAX           =  64,

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* Maximum wave count we'll ever use — sized for the STAR-13 preset. */
    N_WAVES_MAX         =  13,

    /* Color pair indices.  PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* +0..+7 = 8 intensity tints       */
    PAIR_FLASH          =  13,    /* phase-randomise flash            */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define ASPECT_Y_F           2.0f

/*
 * Reseed flash (EFFECTS).  'r' sets flash_t = 1.0; it then decays each tick and
 * the twinkle overlay is drawn while it stays bright.
 *   FLASH_DECAY_RATE   — exponential fade rate (1/sec); flash_t *= e^(-rate·dt)
 *   FLASH_VISIBLE_MIN  — draw the overlay only while flash_t exceeds this
 */
#define FLASH_DECAY_RATE     4.0f
#define FLASH_VISIBLE_MIN    0.05f

/*
 * Preset — one showcase configuration of the plane-wave sum.  The whole
 * visual identity of a preset is four numbers, so the gallery is a flat
 * table and adding/retuning a look touches exactly one row:
 *
 *   n_waves    — the SYMMETRY driver.  N waves at θ_m = m·π/N give 2N-fold
 *                rotational symmetry.  N ∈ {3,4,6,12} are PERIODIC crystals
 *                (Bravais-compatible); N ∈ {5,7,8,9,10,11,13} are APERIODIC
 *                quasicrystals (Crystallographic Restriction Theorem — see
 *                CONCEPTS).  Bounded by N_WAVES_MAX.
 *   wavelength — the DENSITY driver, in cell units.  Smaller = tighter, more
 *                wave fronts; larger = big sweeping features.  Two presets at
 *                the same N but different λ look completely different, which
 *                is how the table reaches 15 distinct looks from ~11 symmetries.
 *   rate_base  — radians/sec the WHOLE pattern drifts (uniform; rigid glide).
 *   rate_delta — per-wave INCREMENT on rate_base.  Without it all waves drift
 *                in lockstep and the pattern merely translates; with it the
 *                relative phases evolve so the pattern MORPHS in place.
 *
 * A Preset is the parameterisation of one density-wave quasicrystal: the
 * (N, λ, drift) triple is exactly what Levine & Steinhardt (1984) vary to
 * generate different quasicrystalline orders.  Ref: CONCEPTS + References block.
 */
typedef struct {
    const char *name;       /* HUD label, also the i/N gallery entry          */
    int         n_waves;    /* N — symmetry order (2N-fold); see CRT note above */
    float       wavelength; /* λ in cells — feature size / density             */
    float       rate_base;  /* uniform phase drift rate (rad/s)                */
    float       rate_delta; /* per-wave drift increment → in-place morphing    */
} Preset;

#define N_PRESETS 15

/*
 * The gallery.  Curated for a wide spread of symmetry AND density: a sweep of
 * N = 3..13 (periodic crystals interleaved with quasicrystals for contrast),
 * plus fine/giant density variants of the most striking orders (5, 7, 11).
 * Default boot is PENROSE-5, the canonical 10-fold quasicrystal.
 */
static const Preset presets[N_PRESETS] = {
    /* name         N   λ     base  delta */
    { "HEX-3",      3, 16.0f, 0.50f, 0.07f },  /* periodic hexagon — the reference */
    { "SQUARE-4",   4, 16.0f, 0.45f, 0.06f },  /* 8-fold, square-ish weave         */
    { "PENROSE-5",  5, 14.0f, 0.50f, 0.07f },  /* 10-fold, Penrose-flavoured       */
    { "FLOWER-6",   6, 18.0f, 0.40f, 0.05f },  /* periodic hexagonal rosettes      */
    { "SEPTA-7",    7, 13.0f, 0.55f, 0.08f },  /* 14-fold, denser stars            */
    { "OCTA-8",     8, 14.0f, 0.50f, 0.06f },  /* 16-fold octagonal                */
    { "ENNEA-9",    9, 12.0f, 0.55f, 0.07f },  /* 18-fold                          */
    { "DECA-10",   10, 13.0f, 0.50f, 0.06f },  /* 20-fold                          */
    { "UNDECA-11", 11, 12.0f, 0.60f, 0.08f },  /* 22-fold, near-cloud detail       */
    { "DODECA-12", 12, 13.0f, 0.45f, 0.05f },  /* 24-fold, near-periodic           */
    { "STAR-13",   13, 11.0f, 0.60f, 0.09f },  /* 26-fold, extremely intricate     */
    { "FINE-5",     5,  8.0f, 0.70f, 0.10f },  /* tight decagonal lattice          */
    { "GIANT-5",    5, 22.0f, 0.30f, 0.04f },  /* huge slow 10-pointed stars       */
    { "WEAVE-7",    7,  9.0f, 0.65f, 0.10f },  /* dense septagonal weave           */
    { "NOVA-11",   11, 20.0f, 0.35f, 0.05f },  /* big intricate 11-fold rosettes   */
};

/*
 * GlyphSet — how the scalar intensity field I ∈ [-1,+1] is turned into ASCII
 * (cycled by g/G).  Pure RENDER choice: it changes nothing in the field, only
 * which feature of it the eye sees.  Each mode answers a different question:
 *   RAMP    — the full field as a density gradient (the default "see it all")
 *   PEAKS   — only crests (I > 0); the bright star pattern, troughs blanked
 *   CONTOUR — only the zero-crossings |I|≈0; the wave-front network (level set)
 *   WAVES   — bipolar: crests bright, troughs dim — emphasises the +/- structure
 * Decoded by intensity_to_glyph (§3); the trailing N_GLYPH_SETS bounds the cycle.
 */
typedef enum {
    GLYPH_RAMP    = 0,    /* density ramp over the whole [-1,+1] range */
    GLYPH_PEAKS   = 1,    /* positive intensity only (highlights)       */
    GLYPH_CONTOUR = 2,    /* zero-crossing band only (wave fronts)      */
    GLYPH_WAVES   = 3,    /* bipolar peaks-bright / troughs-dim          */
    N_GLYPH_SETS  = 4,    /* count — bounds the g/G cycle, not a mode    */
} GlyphSet;

/* Density-ramp glyphs (low → high intensity). Used by RAMP / PEAKS /
 * WAVES; CONTOUR uses a single thresholded glyph.  ASCII-only. */
static const char RAMP_GLYPHS[8] = { '`', '.', ',', ':', '-', 'o', '#', '@' };

/* Contour band thickness — cells with |intensity| < CONTOUR_BAND_HALF
 * are drawn; brighter the closer to zero. */
#define CONTOUR_BAND_HALF    0.20f

/*
 * Theme — one named colour palette, cycled live by the t/T keys.  WHY a struct
 * (not loose arrays): a theme is the whole look in one row of the themes[]
 * table, so adding/editing a palette touches exactly one line and theme_apply()
 * just blits the row into ncurses colour pairs.  Every value is a 256-colour-
 * cube index; the CLAUDE.md "Theme Palette Brightness" rule forces them all into
 * the bright half so even A_DIM cells stay legible on a default-black terminal.
 *
 *   name — HUD label, the only part the user reads.
 *   ramp — an 8-tier dark→bright GRADIENT, the heart of the palette.  Loaded
 *          into pairs PAIR_RAMP_BASE+0..+7 and indexed by the intensity level
 *          (0..7) that intensity_to_glyph computes, so the perceived theme is
 *          the RELATIVE gradient across the eight tiers, not any one colour.
 */
typedef struct {
    const char *name;      /* HUD label, cycled by t/T                       */
    short       ramp[8];   /* 8-tier dark→bright gradient (256-cube indices)  */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name       0    1    2    3    4    5    6    7  */
    { "DEFAULT",{ 24,  31,  39,  70,  76, 137, 215, 230 } },
    { "MATRIX", { 28,  34,  40,  46,  76, 118, 154, 192 } },
    { "NOVA",   { 60,  91, 134, 165, 207, 213, 219, 231 } },
    { "MONO",   {240, 243, 245, 247, 249, 251, 253, 255 } },
    { "OCEAN",  { 24,  25,  31,  38,  45,  51, 117, 195 } },
    { "FIRE",   { 88, 124, 130, 166, 202, 208, 214, 226 } },
    { "EARTH",  { 94, 130, 137, 173, 179, 215, 222, 230 } },
    { "FOREST", { 28,  34,  40,  70,  76, 112, 156, 192 } },
    { "DESERT", {130, 137, 143, 173, 179, 215, 222, 229 } },
    { "ARCTIC", { 24,  31,  67, 110, 117, 153, 195, 231 } },
};

/* ===================================================================== */
/* §2  PERFORMANCE — timing primitives                                    */
/* ===================================================================== */
/* Monotonic clock + sleep.  The fixed-timestep accumulator, fps counter   */
/* and 60 fps frame cap that USE them live at the combine point in §6 main. */

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
/* §3  LOGIC — pure decisions (no mutation, no I/O)                       */
/* ===================================================================== */
/* Results depend only on arguments; these write no globals and touch no    */
/* screen, so reordering or deleting any RENDER/EFFECTS code cannot change   */
/* their output.  compute_intensity is also pure but lives in §4 beside the  */
/* WaveVectors type it reads (see ARCHITECTURE).                             */

static const char *glyph_set_name(GlyphSet g)
{
    switch (g) {
    case GLYPH_RAMP:    return "ramp ";
    case GLYPH_PEAKS:   return "peaks";
    case GLYPH_CONTOUR: return "cntr ";
    case GLYPH_WAVES:   return "waves";
    default:            return "?    ";
    }
}

/* wrap_inc / wrap_dec — step an index forward/backward through [0, n) with
 * wraparound: the "next/previous in a cyclic list" the key handler uses to
 * cycle preset / theme / glyph.  wrap_dec adds n before the modulo so the
 * result is never negative. */
static inline int wrap_inc(int v, int n) { return (v + 1) % n; }
static inline int wrap_dec(int v, int n) { return (v + n - 1) % n; }

/* clamp_level — pin a raw index into the valid [0, 7] ramp-gradient range. */
static inline int clamp_level(int level)
{
    if (level < 0) return 0;
    if (level > 7) return 7;
    return level;
}

/* ramp_attr — emphasise the gradient ENDS: brightest tiers bold, darkest dim,
 * mid-tiers plain.  Lifts the read of high/low intensity out of A_DIM mush. */
static inline int ramp_attr(int level)
{
    if (level >= 6) return A_BOLD;
    if (level <= 1) return A_DIM;
    return A_NORMAL;
}

/* hash3 — only used for the reseed-flash overlay. */
static inline uint32_t hash3(int wx, int wy, int wz)
{
    uint32_t h = (uint32_t)wx * 73856093u
               ^ (uint32_t)wy * 19349663u
               ^ (uint32_t)wz * 83492791u;
    h ^= h >> 16;
    h *= 0x85ebca6bu;
    h ^= h >> 13;
    h *= 0xc2b2ae35u;
    h ^= h >> 16;
    return h;
}

/*
 * intensity_to_glyph — choose (glyph, ramp_idx, attr) from the
 * intensity value and the active GlyphSet. Returns false when the
 * cell should be left blank (e.g., negative intensity in PEAKS mode,
 * far-from-zero intensity in CONTOUR mode).
 */
static bool intensity_to_glyph(float intensity, GlyphSet g,
                               char *glyph, int *ramp_idx, int *attr)
{
    *attr = A_NORMAL;

    switch (g) {
    case GLYPH_RAMP: {
        /* full bipolar field [-1,+1] → 8-step ramp [0,7] */
        int level = clamp_level((int)((intensity + 1.0f) * 4.0f));
        *glyph = RAMP_GLYPHS[level];
        *ramp_idx = level;
        *attr = ramp_attr(level);
        return true;
    }

    case GLYPH_PEAKS: {
        /* crests only: troughs blank, [0,+1] → ramp [0,7] */
        if (intensity < 0.0f) return false;
        int level = clamp_level((int)(intensity * 8.0f));
        *glyph = RAMP_GLYPHS[level];
        *ramp_idx = level;
        *attr = ramp_attr(level);
        return true;
    }

    case GLYPH_CONTOUR: {
        /* zero-crossing band: drawn only near I=0, brighter toward it */
        float dist = fabsf(intensity);
        if (dist > CONTOUR_BAND_HALF) return false;
        float strength = 1.0f - dist / CONTOUR_BAND_HALF;     /* [0,1], 1 at I=0 */
        int level = clamp_level((int)(strength * 8.0f));
        *glyph = (strength > 0.65f) ? '*' : (strength > 0.30f) ? '.' : '`';
        *ramp_idx = level;
        if (level >= 6) *attr = A_BOLD;   /* no dim floor: faint contour stays plain */
        return true;
    }

    case GLYPH_WAVES: {
        /* Bipolar — peaks bright, troughs dim. */
        if (intensity > 0.55f) {
            *glyph = '#'; *ramp_idx = 7; *attr = A_BOLD;
        } else if (intensity > 0.15f) {
            *glyph = 'o'; *ramp_idx = 5;
        } else if (intensity > -0.15f) {
            *glyph = '.'; *ramp_idx = 3; *attr = A_DIM;
        } else if (intensity > -0.55f) {
            *glyph = ','; *ramp_idx = 2; *attr = A_DIM;
        } else {
            *glyph = '`'; *ramp_idx = 1; *attr = A_DIM;
        }
        return true;
    }

    case N_GLYPH_SETS:
        return false;
    }
    return false;
}

/* ===================================================================== */
/* §4  SIMULATION — wave vectors, intensity sampler, Scene, scene_tick     */
/* ===================================================================== */
/* The only state that advances.  waves_init fills the wave vectors on       */
/* pattern change; scene_tick advances time_secs each tick.  Folded in here   */
/* (too small for their own section): EFFECTS = the flash_t decay; DELAYS =   */
/* the `paused` gate at the top of scene_tick.                                */

/*
 * WaveVectors — the N unit wave vectors k̂_m = (cos θ_m, sin θ_m) the field is
 * summed over (θ_m = m·π/N).  This IS the quasicrystal's defining data: the
 * density-wave model builds the pattern as the interference of N plane waves
 * in these directions, and N coprime with the Bravais orders forces aperiodic
 * long-range order (Levine & Steinhardt 1984; Crystallographic Restriction
 * Theorem — see the References block).
 *
 * Precomputed on preset change so the per-cell sampler (compute_intensity)
 * never recomputes trig.  Stored as PARALLEL arrays, not an array-of-Vec2, so
 * the inner loop streams cos_theta[]/sin_theta[] contiguously — the hot path
 * touches this N times per cell.
 *   count                  — N, the number of active vectors (≤ N_WAVES_MAX)
 *   cos_theta / sin_theta  — the x / y components, valid for index 0..count-1;
 *                            slots ≥ count are stale and must not be read
 */
typedef struct {
    int   count;                    /* N — active wave-vector count          */
    float cos_theta[N_WAVES_MAX];   /* x-components of k̂_m (0..count-1)      */
    float sin_theta[N_WAVES_MAX];   /* y-components of k̂_m (0..count-1)      */
} WaveVectors;

/* waves_init — fill `w` with the N wave vectors θ_m = m·π/N. Mutator. */
static void waves_init(WaveVectors *w, int n)
{
    if (n < 1) n = 1;
    if (n > N_WAVES_MAX) n = N_WAVES_MAX;
    w->count = n;
    for (int k = 0; k < n; k++) {
        float angle = (float)k * (float)M_PI / (float)n;
        w->cos_theta[k] = cosf(angle);
        w->sin_theta[k] = sinf(angle);
    }
}

/*
 * compute_intensity — sum the plane waves at one cell.  PURE: reads only its
 * arguments (the WaveVectors included) — no globals, no I/O — so it is true
 * LOGIC, placed here only to sit beside the WaveVectors type it reads.
 *
 *   x  = sx                        (screen x in cell units)
 *   y  = sy * ASPECT_Y_F          (aspect-corrected screen y)
 *   ω  = freq = 2π / λ            (spatial frequency, from the preset)
 *   I  = (1/N) Σ cos(ω(x·cos θ_k + y·sin θ_k) + φ_k(t))
 *
 * freq, rate_base and rate_delta come from the active Preset.  Returns roughly
 * [-1, +1] (the maximum is 1.0 when all N waves constructively interfere).
 */
static float compute_intensity(const WaveVectors *w, int sx, int sy, float t,
                               float freq, float rate_base, float rate_delta)
{
    float fx = (float)sx;
    float fy = (float)sy * ASPECT_Y_F;
    float sum = 0.0f;
    for (int k = 0; k < w->count; k++) {
        float wx    = fx * w->cos_theta[k] + fy * w->sin_theta[k];
        float phase = t * (rate_base + (float)k * rate_delta);
        sum += cosf(freq * wx + phase);
    }
    return sum / (float)w->count;
}

/*
 * Scene — the whole simulated world plus the knobs that drive it, laid out as
 * a table of contents.  Render/HUD read it; only the tick orchestrators (init /
 * reseed / pattern_changed / tick) take a Scene* — every other function takes
 * the narrowest sub-type (const WaveVectors*, const Screen*, …).
 */
typedef struct {
    /* WHAT is simulated — the wave vectors the field is summed over. */
    WaveVectors waves;

    /* HOW the user drives the SIMULATION. */
    int      current_preset;     /* index into presets[] — which pattern    */
    int      speed;              /* drift-speed multiplier (+/- keys)        */

    /* WHERE in the animation we are — the drift clock + this run's phase. */
    float    time_secs;          /* simulation clock (advanced by speed)     */
    float    phase_offset;       /* random phase added to the clock ('r')    */
    bool     paused;             /* DELAYS: run gate, checked in scene_tick   */
    float    flash_t;            /* EFFECTS: reseed flash, decays each tick   */

    /* Display options — RENDER concepts, merely toggled by keys. */
    GlyphSet current_glyph;      /* how intensity maps to a glyph            */
    int      current_theme;      /* active Theme index                       */
} Scene;

static void scene_pattern_changed(Scene *s)
{
    waves_init(&s->waves, presets[s->current_preset].n_waves);
}

static void scene_reseed(Scene *s)
{
    /* Randomise the phase offset — pattern snaps to a new state. */
    uint32_t h = hash3((int)(s->time_secs * 1000.0f),
                       (int)(s->phase_offset * 100.0f), 0xDECAF);
    s->phase_offset = ((float)(h & 0xFFFFu) / 65536.0f) * 2.0f * (float)M_PI;
    s->flash_t = 1.0f;
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_preset  = 2;          /* PENROSE-5 — the canonical 10-fold look */
    s->current_glyph   = GLYPH_RAMP;
    s->phase_offset    = 0.0f;
    scene_pattern_changed(s);
}

/*
 * scene_tick — advance time and decay the flash overlay. Speed
 * scales the time advance, so fast/slow drift is just a multiplier
 * on the phase rate.
 */
static void scene_tick(Scene *s, float dt)
{
    s->flash_t *= expf(-FLASH_DECAY_RATE * dt);
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->time_secs += dt * speed_mul;
}

/* ===================================================================== */
/* §5  RENDER — state → screen (reads only, never mutates the model)      */
/* ===================================================================== */
/* Colour/theme setup, screen geometry, then the field + flash + HUD draw.  */
/* Reads Scene and the wave field; writes only ncurses (screen + colour      */
/* pairs) and Screen.{cols,rows} on init/resize.  Never mutates Scene.        */

/* ---- colour: load a theme's ramp + accents into ncurses pairs ----------- */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
    } else {
        static const short fb[8] = {
            COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN,   COLOR_CYAN,
            COLOR_GREEN, COLOR_YELLOW,COLOR_YELLOW, COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), fb[i], -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
        init_pair(PAIR_FLASH, 226, -1);
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
    }
    theme_apply(0);
}

/* ---- screen: terminal viewport ------------------------------------------ */

/*
 * Screen — the terminal viewport, just its size in character cells.  WHY a
 * type for two ints: it is the narrowest read-only handle the render functions
 * (scene_draw, screen_draw) need for geometry, so they take `const Screen*`
 * instead of the whole App — keeping RENDER decoupled from simulation/runtime
 * state.  Captured by getmaxyx at init and refreshed on every SIGWINCH resize;
 * scene_draw derives the field's centre/extent from it each frame.
 */
typedef struct {
    int cols, rows;   /* current terminal width / height in cells */
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
 * draw_field — the render hot path: sample the quasicrystal at every cell and
 * ink it through the active glyph set.  The field is centred so its rotational-
 * symmetry point sits mid-screen; each cell's value is one compute_intensity.
 */
static void draw_field(const Screen *sc, const Scene *s)
{
    int top = 2, bottom = sc->rows - 1;
    const Preset *ps = &presets[s->current_preset];
    float freq = 2.0f * (float)M_PI / ps->wavelength;   /* ω = 2π/λ */
    float t    = s->time_secs + s->phase_offset;
    int   cx   = sc->cols / 2;
    int   cy   = (top + bottom) / 2;

    for (int sy = top; sy < bottom; sy++) {
        int rel_y = sy - cy;
        for (int sx = 0; sx < sc->cols; sx++) {
            int rel_x = sx - cx;
            float intensity = compute_intensity(&s->waves, rel_x, rel_y, t,
                                                freq, ps->rate_base,
                                                ps->rate_delta);

            char glyph;
            int  ramp_idx, attr;
            if (!intensity_to_glyph(intensity, s->current_glyph,
                                    &glyph, &ramp_idx, &attr)) continue;

            int pair = PAIR_RAMP_BASE + ramp_idx;
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/*
 * draw_reseed_flash — brief sparse twinkle overlaid after 'r', while the flash
 * envelope is still bright.  A hash-sparse star field (1-in-8 of a 2×2 grid)
 * that animates by mixing the millisecond clock into the cell test.
 */
static void draw_reseed_flash(const Screen *sc, const Scene *s)
{
    int seed = (int)(s->time_secs * 1000.0f);
    attron(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
    for (int sy = 2; sy < sc->rows - 1; sy += 2) {
        for (int sx = 0; sx < sc->cols; sx += 2) {
            if (((sx ^ sy ^ seed) & 7) == 0)
                mvaddch(sy, sx, '*');
        }
    }
    attroff(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
}

/* scene_draw — the field, then the reseed flash on top while it is still bright. */
static void scene_draw(const Screen *sc, const Scene *s)
{
    draw_field(sc, s);
    if (s->flash_t > FLASH_VISIBLE_MIN)
        draw_reseed_flash(sc, s);
}

/*
 * hud_draw_status_line — row 0: live status on the right (fps / tick Hz /
 * DRIFT|PAUSED / speed) and, on the left, the title plus the preset counter
 * "i/N NAME" so the active gallery entry is always visible.
 */
static void hud_draw_status_line(const Screen *sc, const Scene *s,
                                 double fps, int sim_fps)
{
    const char *state_str = s->paused ? "PAUSED" : "DRIFT ";

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);                  /* right-aligned status */
    mvprintw(0, 1, " QUASICRYSTAL  %2d/%d %-9s ", /* left title + preset */
             s->current_preset + 1, N_PRESETS,
             presets[s->current_preset].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/*
 * hud_draw_param_line — row 1: the parameters the keys control — glyph mode,
 * theme name, a live swatch of the 8 ramp colours, and the wave count / drift
 * phase readout.  Each field prints, then `x` advances past its column width.
 */
static void hud_draw_param_line(const Screen *sc, const Scene *s)
{
    (void)sc;
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " glyph:%-5s ", glyph_set_name(s->current_glyph));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 14;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    /* Ramp swatch — paint each of the 8 gradient tiers in its own pair. */
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " ramp:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 6;
    for (int i = 0; i < 8; i++) {
        int p = PAIR_RAMP_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, (chtype)(unsigned char)RAMP_GLYPHS[i]);
        attroff(COLOR_PAIR(p) | A_BOLD);
        x++;
    }

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             "  N=%d  phase:%5.2f ",
             presets[s->current_preset].n_waves,
             (double)(s->time_secs + s->phase_offset));
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* hud_draw_key_hints — bottom row: the full interactive key legend. */
static void hud_draw_key_hints(const Screen *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  g/G:glyph  t/T:theme  +/-:drift  spc:pause  r:reseed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);                          /* the field + flash */
    hud_draw_status_line(sc, s, fps, sim_fps);  /* row 0: title + preset + fps/state */
    hud_draw_param_line(sc, s);                 /* row 1: glyph/theme/ramp/N/phase */
    hud_draw_key_hints(sc);                     /* bottom row: key legend */
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §6  APP — signals, resize, input + the per-tick combine                */
/* ===================================================================== */
/* main is the ONE place the layers combine per tick (PERFORMANCE →         */
/* SIMULATION → PERFORMANCE → RENDER; see ARCHITECTURE).  Signals and        */
/* app_handle_key mutate state OUTSIDE the tick, never inside scene_tick.    */

/*
 * App — the running program: the simulated Scene plus the screen and the loop
 * runtime that the tick and the signal handlers share.  It is the root the
 * combine point (main) owns; sub-layers still take the narrowest type, so
 * bundling everything here does not re-couple them.
 *
 *   scene / screen — WHAT advances + WHERE it draws (see those types).
 *   sim_fps        — fixed-timestep tick rate ([ / ] keys); sets TICK_NS.
 *   running        — 0 = quit; cleared by SIGINT/SIGTERM.
 *   need_resize    — 1 = a SIGWINCH is pending; serviced before the next tick.
 * running/need_resize are written from async signal handlers, so they are
 * `volatile sig_atomic_t` — the only type a handler may portably touch and the
 * only way the compiler won't optimise the flag-read out of the loop.
 */
typedef struct {
    Scene                 scene;        /* WHAT advances — the simulated field */
    Screen                screen;       /* WHERE it draws — viewport size      */
    int                   sim_fps;      /* fixed tick rate (Hz), [ / ] keys    */
    volatile sig_atomic_t running;      /* 0 = quit; set by SIGINT/SIGTERM     */
    volatile sig_atomic_t need_resize;  /* 1 = SIGWINCH pending; serviced in loop */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app->need_resize = 0;
}

/* app_handle_key — user EVENT, not part of the tick.  May mutate Scene knobs
 * and App fields directly; it never advances simulation state (no scene_tick). */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reseed(s);                               break;

    case '=': case '+':
        if (s->speed < SPEED_MAX) s->speed *= 2;
        if (s->speed > SPEED_MAX) s->speed  = SPEED_MAX;
        break;
    case '-':
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed  = SPEED_MIN;
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
        s->current_theme = wrap_inc(s->current_theme, N_THEMES);
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = wrap_dec(s->current_theme, N_THEMES);
        theme_apply(s->current_theme);
        break;

    case 'n': case 'N':
        s->current_preset = wrap_inc(s->current_preset, N_PRESETS);
        scene_pattern_changed(s);
        break;
    case 'p': case 'P':
        s->current_preset = wrap_dec(s->current_preset, N_PRESETS);
        scene_pattern_changed(s);
        break;

    case 'g':
        s->current_glyph = (GlyphSet)wrap_inc((int)s->current_glyph, N_GLYPH_SETS);
        break;
    case 'G':
        s->current_glyph = (GlyphSet)wrap_dec((int)s->current_glyph, N_GLYPH_SETS);
        break;

    default: break;
    }
    return true;
}

/* app_init — bring the program up: seed the RNG, install signal handlers and
 * the endwin() atexit hook, set the loop's starting state, then open the screen
 * and the scene.  Everything that happens once, before the per-frame loop. */
static void app_init(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    scene_init(&app->scene);
}

int main(void)
{
    App *app = &g_app;
    app_init(app);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;   /* unspent real time owed to the fixed-step sim */
    int64_t fps_accum   = 0;   /* real time accumulated in the current fps window */
    int     frame_count = 0;
    double  fps_display = 0.0;

    const int64_t max_dt_ns    = (int64_t)MAX_FRAME_DT_MS * NS_PER_MS;
    const int64_t frame_cap_ns = NS_PER_SEC / RENDER_FPS_CAP;

    while (app->running) {

        /* EVENT (not the tick): service a pending resize before timing. */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* 1. PERFORMANCE — measure real elapsed time, clamp a stall. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > max_dt_ns) dt = max_dt_ns;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        /* 2. SIMULATION — drain the fixed-step accumulator. */
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* 3. PERFORMANCE — fps accounting + sleep to the frame cap. */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(frame_cap_ns - elapsed);

        /* 4. RENDER — read-only draw of the field + flash + HUD. */
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* EVENT (not the tick): drain one key. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
