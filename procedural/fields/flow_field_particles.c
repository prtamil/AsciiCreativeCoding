/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * flow_field_particles.c
 *   — Particles streaming through closed-form 2-D vector fields.
 *
 * DEMO: 256 particles flow through one of THIRTY algebraically-defined
 *       2-D vector fields. Unlike the noise-based showcases in this
 *       folder, every field here is a CLOSED-FORM expression — pure
 *       (x, y, t) → (vx, vy) math, no Perlin / Worley / etc. The
 *       patterns span the canonical dynamical-systems and fluid-
 *       dynamics textbook taxonomy:
 *
 *         Rotational           : VORTICES SPIRAL CIRCULAR CHAIN GRID
 *                                GALAXY TURBULENT EDDY HURRICANE
 *         Radial / linear      : SADDLE MAGNET RADIAL SHEAR JET
 *                                GRADIENT QUADPOLE PITCHFORK
 *         Time-varying waves   : WAVE RIPPLE STANDING PLANE WAKE
 *         Nonlinear oscillators: DUFFING VANDERPOL PENDULUM LOTKA HOPF
 *                                HAMILTON NEWTON HENON
 *
 *       The top HUD shows the current pattern + [N/30] index. n/p
 *       cycles. Equilibria / singularities / poles are drawn as
 *       bright '@' glyphs on top of the particle trails so the
 *       field's salient points stay visible. Themes shape colour.
 *
 * Study alongside:
 *   ./perin_noise_flow_showcase.c — same particle-flow architecture,
 *       but the field source is Perlin noise. Compare the streamline
 *       SHAPES: noise gives organic eddies, this file gives clean
 *       textbook fields.
 *   ./curl_noise_vector_field.c — divergence-free curl-of-noise
 *       fields. Particles never converge there; here, MAGNET and
 *       SADDLE explicitly DO converge / diverge.
 *
 * Section map:
 *   §1 config   — grid, particles, palette, themes, named constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes
 *   §5 fields   — 30 closed-form (vx, vy) functions
 *   §6 attractors — per-pattern centre / equilibrium placement
 *   §7 scene    — Scene struct composing Grid, RenderBuffers,
 *                 Particles, AttractorPool, SimState, Controls; the
 *                 per-frame tick / particle-step pipeline
 *   §8 screen   — ASCII render: density glyphs + attractor markers
 *                 + named HUD drawers
 *   §9 app      — signals, resize, named main-loop helpers
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (re-seed particles, re-randomise field state)
 *   n / N      next pattern
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   g / G      next / previous glyph set (slim → fat)
 *   + / =      faster particles
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra flow_field_particles.c \
 *       -o flow_field -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Particle advection through a time-varying 2-D vector
 *                  field. For each particle, look up (vx, vy) at its
 *                  current position, step by (vx, vy) · dt, paint a
 *                  trail. Thirty patterns expose thirty different
 *                  field generators; the particle simulation is
 *                  identical across all of them.
 *
 *                  Field generators (closed-form, no noise) span four
 *                  families:
 *
 *                  Rotational / vortex:
 *                    VORTICES, SPIRAL, CIRCULAR, CHAIN, GRID, GALAXY,
 *                    TURBULENT, EDDY, HURRICANE — vortex sums, single
 *                    swirls, Gaussian-envelope hurricanes, etc.
 *
 *                  Radial / linear:
 *                    SADDLE, MAGNET, RADIAL, SHEAR, JET, GRADIENT,
 *                    QUADPOLE, PITCHFORK — source/sink, attract/repel,
 *                    pure outflow, parabolic flow, gradient descent.
 *
 *                  Time-varying waves:
 *                    WAVE, RIPPLE, STANDING, PLANE, WAKE — sinusoidal
 *                    flows, expanding waves, standing patterns,
 *                    plane-wave directional flow, Kármán wakes.
 *
 *                  Nonlinear oscillators (textbook phase planes):
 *                    DUFFING, VANDERPOL, PENDULUM, LOTKA, HOPF,
 *                    HAMILTON, NEWTON, HENON — the canonical
 *                    Strogatz / Guckenheimer-Holmes phase portraits.
 *
 *                  Attractor / vortex POSITIONS that belong to "live"
 *                  patterns drift slowly (orbit around their nominal
 *                  centres) so the flow evolves without resetting.
 *
 * Data-structure : Scene is the umbrella context, composed of six
 *                  sub-structs so each layer has ONE clear role and
 *                  no globals leak across them:
 *                    Grid          — w, h, total_cells + grid_idx() /
 *                                    grid_in_bounds().
 *                    RenderBuffers — per-cell glow + colour band; the
 *                                    complete render contract between
 *                                    the particle layer (§7) and the
 *                                    screen layer (§8).
 *                    Particles     — fixed pool of advection walkers
 *                                    (pool[], n).
 *                    AttractorPool — base list of Attractors plus a
 *                                    per-frame "active" copy with
 *                                    drift applied. Drives the field
 *                                    for attractor-based patterns and
 *                                    supplies '@' marker positions
 *                                    for all patterns.
 *                    SimState      — field_time (the only mutable
 *                                    sim scalar).
 *                    Controls      — paused / speed / theme / glyph
 *                                    set / pattern; mutated only by
 *                                    the keyboard.
 *                  No heap; everything BSS.
 *
 * Rendering      : ASCII only. Density-graded trail glyphs chosen
 *                  from one of five GlyphSets (slim → fat) in 4 theme
 *                  colours. Bright '@' over attractor positions
 *                  (vortex centres, magnet poles, oscillator
 *                  equilibria) so the field's salient points are
 *                  visible against the streamline pattern.
 *
 * Performance    : 1–20 attractor evaluations per particle per frame
 *                  (TURBULENT and GRID are the heaviest at ~20 / 9).
 *                  With 256 particles at 60 Hz that's < 1 % of one
 *                  core on modern hardware.
 *
 * References     : Dynamical-systems theory (covers most patterns)
 *                  ─────────────────────────────────────────────────
 *                  • Strogatz, S. — "Nonlinear Dynamics and Chaos"
 *                    (2nd ed, Westview). THE foundational text for
 *                    everything in this file: phase plane analysis
 *                    (saddle / centre / spiral / node classification),
 *                    separatrix, bifurcations (pitchfork in ch. 3,
 *                    Hopf in ch. 8), limit cycles (Van der Pol in
 *                    ch. 7), basin separators (Duffing). Reads like
 *                    a textbook tour of the patterns in §5.
 *                  • Guckenheimer, J. & Holmes, P. — "Nonlinear
 *                    Oscillations, Dynamical Systems, and Bifurcations
 *                    of Vector Fields" (Springer). The rigorous
 *                    companion to Strogatz; source for the canonical
 *                    DUFFING, VANDERPOL, HOPF, HAMILTON normal forms.
 *                  • Devaney, R. — "An Introduction to Chaotic
 *                    Dynamical Systems" (3rd ed). Newton's-method
 *                    fractals (NEWTON pattern, ch. 11) and iterated
 *                    maps with strange attractors (HENON, ch. 9).
 *
 *                  Seminal papers behind specific patterns
 *                  ───────────────────────────────────────
 *                  • Lotka, A. J. (1925) — "Elements of Physical
 *                    Biology" + Volterra, V. (1926) — "Variazioni e
 *                    fluttuazioni del numero d'individui in specie
 *                    animali conviventi". Independent originals of
 *                    the predator-prey equations the LOTKA pattern
 *                    visualises.
 *                  • von Kármán, T. (1911) — "Über den Mechanismus
 *                    des Widerstandes, den ein bewegter Körper in
 *                    einer Flüssigkeit erfährt", Göttinger Nachr.
 *                    Original analysis of the alternating vortex
 *                    street that CHAIN and WAKE display.
 *                  • Hénon, M. (1976) — "A two-dimensional mapping
 *                    with a strange attractor", Comm. Math. Phys.
 *                    50:69-77. The HENON map; one of the first
 *                    explicitly-computed strange attractors and the
 *                    direct source of the (a, b) = (1.4, 0.3) values.
 *
 *                  Rendering / numerical practice
 *                  ──────────────────────────────
 *                  • Inigo Quilez — "Distance functions for 2-D
 *                    primitives":
 *                    https://iquilezles.org/articles/distfunctions2d/
 *                    Practical stable forms for the inverse-square
 *                    attractor with a softening epsilon (the EPS_R2
 *                    trick used throughout §5).
 *                  • Paul Bourke — "Character representation of grey
 *                    scale images":
 *                    https://paulbourke.net/dataformats/asciiart/
 *                    The canonical density → glyph ramp reference
 *                    behind the trail-density tiers (low / mid /
 *                    high) and the five GlyphSet ramps in §1.
 *
 *                  See also
 *                  ────────
 *                  • Compare ./perin_noise_flow_showcase.c (noise-
 *                    based particle flow) and
 *                    ./curl_noise_vector_field.c (divergence-free
 *                    curl flow) — same particle architecture, but
 *                    the field source differs in revealing ways.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A 2-D vector field assigns a velocity to every point. Drop particles,
 * read off their local velocity, step them forward. The choice of
 * field determines the streamlines: rotational fields make swirls,
 * radial fields make spokes, periodic fields make waves, nonlinear
 * oscillators make limit cycles. All thirty patterns here are pure
 * math — no random component except the seeding of vortex positions.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the screen as a windy plain. The "wind" pattern is what we
 * draw. Drop dust on it and watch the dust trace the wind. Each
 * pattern is a different wind, grouped by family:
 *
 *   Rotational fields:
 *     VORTICES  — four hurricanes spinning at quarter-points.
 *     SPIRAL    — single hurricane that ALSO sucks dust inward.
 *     CIRCULAR  — gentle uniform rotation, concentric orbits.
 *     CHAIN     — Kármán vortex street: alternating-spin row.
 *     GRID      — 3×3 checkerboard of alternating mini-tornadoes.
 *     GALAXY    — central swirl with cos(2θ) spiral arms.
 *     TURBULENT — many small whirlwinds chaotically interacting.
 *     EDDY      — single smooth Gaussian-falloff swirl (no singularity).
 *     HURRICANE — vortex + inflow with a clear calm eye and bright
 *                 eyewall — Maxwell-envelope wind profile.
 *
 *   Radial / linear fields:
 *     RADIAL    — pure source; dust radiates outward.
 *     SADDLE    — wind blows OUT along x and IN along y.
 *     SHEAR     — horizontal wind, faster the further N/S you are.
 *     MAGNET    — opposing poles; flow from − to +.
 *     JET       — parabolic pipe flow; fast centre line.
 *     GRADIENT  — steepest descent on a Gaussian well; converges
 *                 to centre from every direction.
 *     QUADPOLE  — four alternating poles at square corners.
 *     PITCHFORK — pitchfork bifurcation: two stable wells, saddle
 *                 separator on the y-axis.
 *
 *   Time-varying / wave fields:
 *     WAVE      — wind blowing in undulating waves like ocean swell.
 *     RIPPLE    — circular wavefronts radiating from the centre.
 *     STANDING  — standing wave: spatial pattern pulses in time.
 *     PLANE     — directional travelling plane wave.
 *     WAKE      — uniform freestream + Kármán wake downstream of an
 *                 obstacle.
 *
 *   Nonlinear oscillators (phase-plane portraits):
 *     DUFFING   — Duffing oscillator: two stable wells, basin separator.
 *     VANDERPOL — Van der Pol limit cycle (self-sustaining oscillation).
 *     PENDULUM  — simple pendulum: separatrix between libration and
 *                 rotation.
 *     LOTKA     — Lotka-Volterra predator-prey closed orbits.
 *     HOPF      — Hopf bifurcation: limit cycle born "out of nothing".
 *     HAMILTON  — Hamiltonian double-well: figure-eight separatrix.
 *     NEWTON    — Newton's method for z³=1: three basins, fractal
 *                 boundary.
 *     HENON     — continuous-time Hénon strange attractor.
 *
 * Each pattern has its own SIGNATURE streamlines. With practice you
 * can identify the field type at a glance from the particle trails.
 *
 * Visible layers:
 *   1. PARTICLE TRAILS — fading paths (theme-coloured) showing where
 *      particles have flowed. Density encodes how often particles
 *      cross each cell.
 *   2. ATTRACTORS '@' — bright glyphs marking vortex centres, magnet
 *      poles, oscillator equilibria, basin separators. Visible over
 *      the trails so you can see exactly which singularities are
 *      driving the flow.
 *   3. SLOW DRIFT — vortex / magnet positions orbit around their
 *      nominal centres at ~0.2 rad/s, so the flow evolves rather
 *      than freezing. (Time-varying patterns get their time
 *      variation from the formula itself.)
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Spawn N particles uniformly in-bounds. Initialise the
 *     pattern's attractors (count + positions vary per pattern).
 *  2. PER FRAME:
 *     a. Decay the per-cell trail glow exponentially.
 *     b. Advance sim.field_time by dt.
 *     c. Refresh active attractor positions from base + drift.
 *     d. For each particle:
 *        i.   Sample (vx, vy) from the active pattern's field
 *             function at the particle's position.
 *        ii.  Normalise to unit direction.
 *        iii. Step by direction · speed · dt; increment age.
 *        iv.  Paint a trail hit at the new cell.
 *        v.   If age ≥ max_age or position out of bounds, respawn.
 *     e. Render trails (density glyphs) + attractor markers ('@').
 *  3. The user can press 'r' to re-seed particles and re-randomise
 *     attractor positions. There is no automatic reset — the field
 *     runs indefinitely until you ask for a new one.
 *
 * KEY FORMULAS
 * ────────────
 *  Vortex contribution           :
 *    v_i = (−Δy, Δx) · S / max(r², ε)
 *  Spiral (vortex + inflow)      :
 *    v = (−Δy, Δx) · S/r²  +  −(Δx, Δy) · I/r²
 *  Circular (uniform rotation)   :
 *    v = (−Δy, Δx) · α
 *  Galaxy (with spiral arms)     :
 *    arm = 1 + κ · cos(2θ − r·k_arm)
 *    v = (−Δy, Δx) · S·arm/r²  +  −(Δx, Δy) · I/r²
 *  Saddle (hyperbolic)           :
 *    vx = α · (x − cx),  vy = −α · (y − cy)
 *  Radial source                 :
 *    v = (Δx, Δy) · α
 *  Shear (Couette)               :
 *    vx = α · (y − cy),  vy = 0
 *  Magnet (gravity-style)        :
 *    v_i = −sign · (Δx, Δy) / max(r², ε)
 *  Jet (Poiseuille)              :
 *    vx = α · (1 − ((y − cy)/H)²),  vy = 0
 *  Gradient descent              :
 *    v = −∇exp(−r²/2σ²) = −(Δx, Δy)/σ² · exp(−r²/2σ²)
 *  Wave field                    :
 *    vx = sin(ωy + t),  vy = cos(ωx − t)
 *  Ripple (radial wave)          :
 *    v = (Δx/r, Δy/r) · sin(r·k − t·ω) · A
 *  Standing wave                 :
 *    v = (sin(kx), sin(ky)) · cos(ωt) · A
 *  Plane wave                    :
 *    v = (k_x, k_y) / |k| · sin(k·r − ωt) · A
 *  Duffing oscillator            :
 *    vx = y',  vy = −x' − δy' − x'³
 *  Van der Pol oscillator        :
 *    vx = y',  vy = μ(1 − x'²)y' − x'
 *  Pendulum                      :
 *    vx = y',  vy = −sin(x')
 *  Lotka-Volterra                :
 *    vx = αx' − βx'y',  vy = δx'y' − γy'
 *  Hopf bifurcation              :
 *    vx = μx' − y' − x'·(x'² + y'²)
 *    vy = x' + μy' − y'·(x'² + y'²)
 *  Hamilton (double-well)        :
 *    vx = y',  vy = x' − 2x'³
 *  Pitchfork bifurcation         :
 *    vx = μx' − x'³,  vy = −y'
 *  Newton (z³ − 1)               :
 *    v = −f(z)/f'(z),  f(z) = z³ − 1
 *  Hénon (continuous form)       :
 *    vx = 1 − ax'² + y' − x',  vy = bx' − y'
 *  Step (all patterns)           :
 *    (Δx, Δy) = unit(v) · speed · dt
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

    /* Particle pool. */
    MAX_PARTICLES     = 1024,
    N_PARTICLES_DEF   =  256,

    /* Particle lifetime (ticks). Different per particle to break the
     * lockstep "everyone respawns at the same time" effect. */
    AGE_MIN_TICKS     =  60,
    AGE_MAX_TICKS     = 360,

    /* Speed: cells per second. */
    SPEED_MIN         =   1,
    SPEED_DEF         =   8,
    SPEED_MAX         =  64,

    /* Maximum simultaneous attractors (max needed: TURBULENT = 20). */
    MAX_ATTRACTORS    =  32,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_TRAIL_BASE   =   3,    /* PAIR_TRAIL_BASE..+(N_BANDS-1) palette  */
    PAIR_ATTRACT      =   7,    /* '@' attractor markers                  */
};

#define TRAIL_GLOW_DECAY    0.6f
#define GLOW_THRESHOLD      0.05f
#define VELOCITY_EPSILON    1e-6f   /* below this magnitude, skip normalize */
#define TRAIL_HIT_INTENSITY 1.0f    /* glow value deposited at particle cell */

#define DRIFT_RATE          0.2f    /* attractor orbital rate (rad/s) */

/* Field strength scales — tuned so particle motion is consistent
 * across all patterns at SPEED_DEF. Output magnitudes get normalised
 * to unit length in particle_step, so the values below mostly
 * control which attractor wins in multi-attractor sums. */
#define VORTEX_STRENGTH     80.0f
#define MAGNET_STRENGTH     50.0f
#define TURBULENT_STRENGTH   6.0f
#define WAVE_FREQ           0.15f
#define SADDLE_RATE         0.05f
#define EPS_R2              4.0f    /* softening: avoid divide-by-zero at attractor centres */

/* SPIRAL — vortex + radial inflow. Both components decay as 1/r² so
 * the spiral has constant pitch (logarithmic spiral, Bernoulli). */
#define SPIRAL_STRENGTH     80.0f
#define SPIRAL_INFLOW       20.0f

/* RADIAL — pure source radiating outward from map centre. */
#define RADIAL_RATE          0.5f

/* SHEAR — linear horizontal shear; vx grows with vertical distance
 * from the centre line. Classic Couette-flow profile. */
#define SHEAR_RATE           0.3f

/* RIPPLE — radial sinusoidal wave from map centre. */
#define RIPPLE_WAVENUMBER    0.3f   /* spatial frequency along r       */
#define RIPPLE_FREQUENCY     1.5f   /* temporal frequency              */
#define RIPPLE_AMPLITUDE     5.0f

/* CIRCULAR — uniform rotation around map centre. */
#define CIRCULAR_RATE        0.3f

/* CHAIN — Kármán vortex street: row of alternating-sign mini-vortices. */
#define CHAIN_STRENGTH      50.0f
#define CHAIN_COUNT          6
#define CHAIN_ZIGZAG_Y       4.0f   /* alternating vertical offset     */

/* GRID — N×N checkerboard of alternating vortices. */
#define GRID_STRENGTH       30.0f
#define GRID_COUNT           3      /* 3×3 grid                        */

/* GALAXY — central vortex with cos(2θ − k·r) arm modulation. */
#define GALAXY_STRENGTH     80.0f
#define GALAXY_ARM_AMP       0.4f   /* arm contrast (0 = uniform)      */
#define GALAXY_ARM_PITCH     0.15f  /* radial slope of arm phase       */
#define GALAXY_INFLOW       20.0f

/* DUFFING — phase plane of the Duffing oscillator. */
#define DUFFING_DAMPING      0.2f
#define DUFFING_SCALE        0.05f

/* VANDERPOL — Van der Pol limit-cycle oscillator. */
#define VANDERPOL_MU         1.5f
#define VANDERPOL_SCALE      0.07f

/* PENDULUM — phase plane of a simple pendulum (vx = y', vy = -sin x'). */
#define PENDULUM_SCALE       0.08f

/* LOTKA — Lotka-Volterra predator-prey. */
#define LOTKA_ALPHA          1.0f
#define LOTKA_BETA           1.0f
#define LOTKA_GAMMA          1.0f
#define LOTKA_DELTA          1.0f
#define LOTKA_SCALE          0.012f
#define LOTKA_OFFSET         1.0f

/* HOPF bifurcation. */
#define HOPF_MU              1.0f
#define HOPF_SCALE           0.06f

/* NEWTON — Newton's method for z³ = 1. */
#define NEWTON_SCALE         0.04f

/* JET — parabolic pipe-flow (Poiseuille) profile. */
#define JET_STRENGTH         5.0f

/* HURRICANE — vortex + inflow with Maxwell-like radial envelope. */
#define HURRICANE_RADIUS    15.0f
#define HURRICANE_STRENGTH  10.0f
#define HURRICANE_INFLOW     3.0f

/* HAMILTON — Hamiltonian double-well. */
#define HAMILTON_SCALE       0.07f

/* WAKE — uniform freestream + downstream Kármán-street vortices. */
#define WAKE_FREESTREAM      3.0f
#define WAKE_OBSTACLE_REL    0.30f   /* obstacle at 30 % of map width */
#define WAKE_OBSTACLE_GAP    8.0f    /* downstream gap before first vortex */
#define WAKE_ZIGZAG_AMP      3.0f    /* alternating vertical offset of vortices */
#define WAKE_VORTEX_COUNT    5
#define WAKE_VORTEX_STR     30.0f

/* STANDING wave. */
#define STANDING_K           0.3f
#define STANDING_OMEGA       1.0f
#define STANDING_AMP         5.0f

/* PLANE wave. */
#define PLANE_KX             0.2f
#define PLANE_KY             0.1f
#define PLANE_OMEGA          0.7f
#define PLANE_AMP            5.0f

/* QUADPOLE — 4 alternating poles at corners of a square. */
#define QUADPOLE_STRENGTH   30.0f
#define QUADPOLE_SEPARATION  0.18f

/* GRADIENT descent on a 2-D Gaussian well. */
#define GRADIENT_SIGMA      10.0f
#define GRADIENT_STRENGTH   80.0f

/* EDDY — single Gaussian-falloff vortex. */
#define EDDY_SIGMA          12.0f
#define EDDY_STRENGTH        2.0f

/* PITCHFORK bifurcation. */
#define PITCHFORK_MU         1.0f
#define PITCHFORK_SCALE      0.05f

/* HENON map (continuous-time interpretation). */
#define HENON_A              1.4f
#define HENON_B              0.3f
#define HENON_SCALE          0.03f

/* Number of palette bands per theme. Quartile of trail color. */
#define N_BANDS              4

#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/* ── HUD layout ──────────────────────────────────────────────────── *
 * Top HUD carries DATA, bottom HUD carries ACTIONS:
 *   row 0           : title + state bar (fps, Hz, state + [N/M], speed)
 *   row 1           : pattern/theme/palette/parts left, glyph indicator right
 *   row 2           : legend (trails / attractor glyph meanings)
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
 * Pattern — thirty algebraic vector fields. Cycle with n/p.
 *
 *   Rotational:   VORTICES SPIRAL CIRCULAR CHAIN GRID GALAXY TURBULENT
 *                 EDDY HURRICANE
 *   Radial/lin :  SADDLE MAGNET RADIAL SHEAR JET GRADIENT QUADPOLE
 *                 PITCHFORK
 *   Time-vary :   WAVE RIPPLE STANDING PLANE WAKE
 *   Nonlinear  :  DUFFING VANDERPOL PENDULUM LOTKA HOPF HAMILTON
 *                 NEWTON HENON
 */
typedef enum {
    PATTERN_VORTICES  =  0,
    PATTERN_WAVE      =  1,
    PATTERN_SADDLE    =  2,
    PATTERN_MAGNET    =  3,
    PATTERN_TURBULENT =  4,
    PATTERN_SPIRAL    =  5,
    PATTERN_RADIAL    =  6,
    PATTERN_SHEAR     =  7,
    PATTERN_RIPPLE    =  8,
    PATTERN_CIRCULAR  =  9,
    PATTERN_CHAIN     = 10,
    PATTERN_GRID      = 11,
    PATTERN_GALAXY    = 12,
    PATTERN_DUFFING   = 13,
    PATTERN_VANDERPOL = 14,
    PATTERN_PENDULUM  = 15,
    PATTERN_LOTKA     = 16,
    PATTERN_HOPF      = 17,
    PATTERN_NEWTON    = 18,
    PATTERN_JET       = 19,
    PATTERN_HURRICANE = 20,
    PATTERN_HAMILTON  = 21,
    PATTERN_WAKE      = 22,
    PATTERN_STANDING  = 23,
    PATTERN_PLANE     = 24,
    PATTERN_QUADPOLE  = 25,
    PATTERN_GRADIENT  = 26,
    PATTERN_EDDY      = 27,
    PATTERN_PITCHFORK = 28,
    PATTERN_HENON     = 29,
    N_PATTERNS        = 30,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_VORTICES:  return "VORTICES ";
    case PATTERN_WAVE:      return "WAVE     ";
    case PATTERN_SADDLE:    return "SADDLE   ";
    case PATTERN_MAGNET:    return "MAGNET   ";
    case PATTERN_TURBULENT: return "TURBULENT";
    case PATTERN_SPIRAL:    return "SPIRAL   ";
    case PATTERN_RADIAL:    return "RADIAL   ";
    case PATTERN_SHEAR:     return "SHEAR    ";
    case PATTERN_RIPPLE:    return "RIPPLE   ";
    case PATTERN_CIRCULAR:  return "CIRCULAR ";
    case PATTERN_CHAIN:     return "CHAIN    ";
    case PATTERN_GRID:      return "GRID     ";
    case PATTERN_GALAXY:    return "GALAXY   ";
    case PATTERN_DUFFING:   return "DUFFING  ";
    case PATTERN_VANDERPOL: return "VANDERPOL";
    case PATTERN_PENDULUM:  return "PENDULUM ";
    case PATTERN_LOTKA:     return "LOTKA    ";
    case PATTERN_HOPF:      return "HOPF     ";
    case PATTERN_NEWTON:    return "NEWTON   ";
    case PATTERN_JET:       return "JET      ";
    case PATTERN_HURRICANE: return "HURRICANE";
    case PATTERN_HAMILTON:  return "HAMILTON ";
    case PATTERN_WAKE:      return "WAKE     ";
    case PATTERN_STANDING:  return "STANDING ";
    case PATTERN_PLANE:     return "PLANE    ";
    case PATTERN_QUADPOLE:  return "QUADPOLE ";
    case PATTERN_GRADIENT:  return "GRADIENT ";
    case PATTERN_EDDY:      return "EDDY     ";
    case PATTERN_PITCHFORK: return "PITCHFORK";
    case PATTERN_HENON:     return "HENON    ";
    default:                return "?        ";
    }
}

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* §9 main loop — spiral-of-death dt clamp + frame rate cap (Glenn
 * Fiedler's "Fix Your Timestep"). DT_MAX_NS uses NS_PER_MS so it
 * has to be defined here, after the NS_PER_* macros. */
#define DT_MAX_NS       (100 * NS_PER_MS)
#define FRAME_CAP_FPS   60

/*
 * Theme — one complete colour palette for the program. Ten of these
 * live in themes[], cycled by t/T.
 *
 * INTENT. The smallest data needed to recolour every visible cell:
 * N_BANDS = 4 trail colours arranged dim → bright, plus a single
 * "flash" accent reused for the bright '@' attractor markers (so the
 * markers stay legible against any palette, theme-independent of the
 * trail).
 *
 * BANDING MODEL. Each particle gets a stable color_idx ∈ [0, N_BANDS)
 * at spawn time; the deposited trail then carries that band as the
 * cell's palette index. The four-band split forces theme authors to
 * design the ramp as a coherent low → high progression rather than
 * a random scatter — same discipline as a Houdini ramp parameter.
 *
 * COLOUR FORMAT. xterm-256 indices (NOT RGB). When the terminal
 * exposes fewer than 256 colours, theme_apply() falls back to a
 * fixed 8-colour cycle so the demo still runs on legacy TTYs.
 */
typedef struct {
    const char *name;             /* short uppercase label shown in HUD */
    short       trail[N_BANDS];   /* ramp colours: 0 = dim, 3 = bright  */
    short       flash;            /* '@' attractor marker colour        */
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
 * Three tiers map to the three GLYPH_*_THRESH cutoffs in §1
 * (low ≥ GLOW_THRESHOLD, mid ≥ GLYPH_MID_THRESH, high ≥
 * GLYPH_HIGH_THRESH). The principle behind density → glyph mapping
 * is Paul Bourke's ASCII grey-scale ramp; we use a coarser 3-step
 * ramp here because the field rendering is monotonic-direction
 * (particles flow ALONG streamlines) — finer ramps would just trail
 * more visual noise.
 */
typedef struct {
    const char *name;       /* short label shown in HUD glyph indicator */
    char        low;        /* glow > GLOW_THRESHOLD                    */
    char        mid;        /* glow > GLYPH_MID_THRESH                  */
    char        high;       /* glow > GLYPH_HIGH_THRESH                 */
} GlyphSet;

#define N_GLYPH_SETS 5

static const GlyphSet glyph_sets[N_GLYPH_SETS] = {
    /*  name      low  mid  high   visual progression       */
    { "SLIM",    '.', '\'', ':' },   /* thinnest, sparse     */
    { "LIGHT",   '.', '*',  '+' },   /* small but visible    */
    { "MEDIUM",  '.', '*',  '#' },   /* standard (default)   */
    { "HEAVY",   'o', 'O',  '@' },   /* round, dense         */
    { "FAT",     '+', '#',  'M' },   /* maximum coverage     */
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
        init_pair(PAIR_ATTRACT, t->flash, -1);
    } else {
        static const short fallback[N_BANDS] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < N_BANDS; i++)
            init_pair(PAIR_TRAIL_BASE + i, fallback[i], -1);
        init_pair(PAIR_ATTRACT, COLOR_YELLOW, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  fields — 30 closed-form vector fields                              */
/* ===================================================================== */

/*
 * Attractor — the static description of one salient point in the
 * field: a vortex centre, magnet pole, oscillator equilibrium, or
 * decorative '@' marker. Fixed at scene reset; never mutated by
 * the per-tick simulation.
 *
 * INTENT. Encodes both the FIXED part of the point (nominal centre
 * + strength) and the SLOW DRIFT orbit applied each frame. The
 * orbit is a circular Lissajous of independent radii (ox, oy), so
 * different attractors trace different ellipses around their base
 * — the asymmetry keeps multi-attractor patterns visually alive
 * rather than rigidly locked. Phase φ is per-attractor so they
 * don't all swing in lockstep.
 *
 * Live position is:
 *   x = bx + ox · cos(t · DRIFT_RATE + φ)
 *   y = by + oy · sin(t · DRIFT_RATE + φ)
 *
 * Computed once per frame in compute_active_attractors(), stored
 * in ActiveAttractor (below).
 */
typedef struct {
    float bx, by;     /* base position (drift orbit centre)              */
    float ox, oy;     /* orbital radii — ellipse semi-axes               */
    float phase;      /* phase offset φ so attractors drift independently */
    float strength;   /* multiplier; 0 = marker-only, no field contribution */
    int   sign;       /* +1 = attractor / CCW vortex, −1 = repeller / CW */
} Attractor;

/*
 * ActiveAttractor — the per-frame "live" position of one Attractor
 * after drift is applied. This is what field functions and the
 * renderer both consume.
 *
 * INTENT. Classic graphics-pipeline PRECOMPUTE PATTERN, analogous
 * to vertex-skinning: the expensive trig (cos / sin in the drift
 * formula) is evaluated ONCE per frame for the few dozen attractors
 * we have, not at every field-sample call inside particle_step
 * (which runs 256 × 30 ≈ 8000 times per frame).
 *
 * Decoupling from Attractor also lets field functions take just the
 * "live" slice (x, y, strength, sign) without needing the orbit
 * parameters — tighter signature, more obvious data dependencies.
 */
typedef struct {
    float x, y;       /* drifted position, this frame                   */
    float strength;   /* carried through from Attractor.strength        */
    int   sign;       /* carried through from Attractor.sign            */
} ActiveAttractor;

/*
 * field_vortices — sum of N vortex contributions. Shared kernel for
 * VORTICES / TURBULENT / CHAIN / GRID — they only differ in attractor
 * placement. Each vortex contributes a tangential velocity ~ 1/r².
 */
static void field_vortices(const ActiveAttractor *att, int n_att,
                           float x, float y,
                           float *out_vx, float *out_vy)
{
    float vx = 0.0f, vy = 0.0f;
    for (int i = 0; i < n_att; i++) {
        float dx = x - att[i].x;
        float dy = y - att[i].y;
        float r2 = dx * dx + dy * dy + EPS_R2;
        float w  = att[i].strength / r2 * (float)att[i].sign;
        vx += -dy * w;
        vy +=  dx * w;
    }
    *out_vx = vx;
    *out_vy = vy;
}

/* field_wave — sinusoidal flow. */
static void field_wave(float x, float y, float t,
                       float *out_vx, float *out_vy)
{
    *out_vx = sinf(y * WAVE_FREQ + t       ) * 5.0f;
    *out_vy = cosf(x * WAVE_FREQ - t * 0.7f) * 5.0f;
}

/* field_saddle — hyperbolic stagnation point. */
static void field_saddle(float x, float y, float cx, float cy,
                         float *out_vx, float *out_vy)
{
    *out_vx =  (x - cx) * SADDLE_RATE;
    *out_vy = -(y - cy) * SADDLE_RATE;
}

/* field_magnet — gravitational dipole (inverse-square attract/repel). */
static void field_magnet(const ActiveAttractor *att, int n_att,
                         float x, float y,
                         float *out_vx, float *out_vy)
{
    float vx = 0.0f, vy = 0.0f;
    for (int i = 0; i < n_att; i++) {
        float dx = x - att[i].x;
        float dy = y - att[i].y;
        float r2 = dx * dx + dy * dy + EPS_R2;
        float w  = (float)(-att[i].sign) * att[i].strength / r2;
        vx += dx * w;
        vy += dy * w;
    }
    *out_vx = vx;
    *out_vy = vy;
}

/* field_spiral — vortex + 1/r² radial inflow → logarithmic spiral. */
static void field_spiral(const ActiveAttractor *att, int n_att,
                          float x, float y,
                          float *out_vx, float *out_vy)
{
    float vx = 0.0f, vy = 0.0f;
    for (int i = 0; i < n_att; i++) {
        float dx = x - att[i].x;
        float dy = y - att[i].y;
        float r2 = dx * dx + dy * dy + EPS_R2;
        float w  = att[i].strength / r2 * (float)att[i].sign;
        vx += -dy * w;
        vy +=  dx * w;
        vx += -dx * SPIRAL_INFLOW / r2;
        vy += -dy * SPIRAL_INFLOW / r2;
    }
    *out_vx = vx;
    *out_vy = vy;
}

/* field_radial — pure outflow from centre. */
static void field_radial(float x, float y, float cx, float cy,
                          float *out_vx, float *out_vy)
{
    *out_vx = (x - cx) * RADIAL_RATE;
    *out_vy = (y - cy) * RADIAL_RATE;
}

/* field_shear — linear horizontal Couette flow. */
static void field_shear(float x, float y, float cy,
                         float *out_vx, float *out_vy)
{
    (void)x;
    *out_vx = (y - cy) * SHEAR_RATE;
    *out_vy = 0.0f;
}

/* field_ripple — radial sinusoidal wave from centre. */
static void field_ripple(float x, float y, float t, float cx, float cy,
                          float *out_vx, float *out_vy)
{
    float dx = x - cx;
    float dy = y - cy;
    float r  = sqrtf(dx * dx + dy * dy) + 0.001f;
    float wave = sinf(r * RIPPLE_WAVENUMBER - t * RIPPLE_FREQUENCY);
    *out_vx = (dx / r) * wave * RIPPLE_AMPLITUDE;
    *out_vy = (dy / r) * wave * RIPPLE_AMPLITUDE;
}

/* field_circular — pure uniform rotation around centre. */
static void field_circular(float x, float y, float cx, float cy,
                            float *out_vx, float *out_vy)
{
    *out_vx = -(y - cy) * CIRCULAR_RATE;
    *out_vy =  (x - cx) * CIRCULAR_RATE;
}

/* field_galaxy — central vortex with cos(2θ − k·r) arm modulation. */
static void field_galaxy(const ActiveAttractor *att, int n_att,
                          float x, float y,
                          float *out_vx, float *out_vy)
{
    float vx = 0.0f, vy = 0.0f;
    for (int i = 0; i < n_att; i++) {
        float dx = x - att[i].x;
        float dy = y - att[i].y;
        float r2 = dx * dx + dy * dy + EPS_R2;
        float r  = sqrtf(r2);
        float theta = atan2f(dy, dx);
        float arm = 1.0f + GALAXY_ARM_AMP * cosf(theta * 2.0f - r * GALAXY_ARM_PITCH);
        float w  = att[i].strength / r2 * (float)att[i].sign * arm;
        vx += -dy * w;
        vy +=  dx * w;
        vx += -dx * GALAXY_INFLOW / r2;
        vy += -dy * GALAXY_INFLOW / r2;
    }
    *out_vx = vx;
    *out_vy = vy;
}

/* field_duffing — vx = y',  vy = −x' − δy' − x'³. */
static void field_duffing(float x, float y, float cx, float cy,
                           float *out_vx, float *out_vy)
{
    float xp = (x - cx) * DUFFING_SCALE;
    float yp = (y - cy) * DUFFING_SCALE;
    *out_vx = yp;
    *out_vy = -xp - DUFFING_DAMPING * yp - xp * xp * xp;
}

/* field_van_der_pol — vx = y',  vy = μ(1 − x'²)·y' − x'. */
static void field_van_der_pol(float x, float y, float cx, float cy,
                               float *out_vx, float *out_vy)
{
    float xp = (x - cx) * VANDERPOL_SCALE;
    float yp = (y - cy) * VANDERPOL_SCALE;
    *out_vx = yp;
    *out_vy = VANDERPOL_MU * (1.0f - xp * xp) * yp - xp;
}

/* field_pendulum — vx = y',  vy = −sin(x'). Separatrix between
 * libration and rotation. */
static void field_pendulum(float x, float y, float cx, float cy,
                            float *out_vx, float *out_vy)
{
    float xp = (x - cx) * PENDULUM_SCALE;
    float yp = (y - cy) * PENDULUM_SCALE;
    *out_vx = yp;
    *out_vy = -sinf(xp);
}

/* field_lotka — Lotka-Volterra predator-prey. Closed orbits around
 * (γ/δ, α/β) = (1, 1) in dynsys coords. */
static void field_lotka(float x, float y, float cx, float cy,
                         float *out_vx, float *out_vy)
{
    float xp = (x - cx) * LOTKA_SCALE + LOTKA_OFFSET;
    float yp = (y - cy) * LOTKA_SCALE + LOTKA_OFFSET;
    if (xp < 0.01f) xp = 0.01f;
    if (yp < 0.01f) yp = 0.01f;
    *out_vx = LOTKA_ALPHA * xp - LOTKA_BETA  * xp * yp;
    *out_vy = LOTKA_DELTA * xp * yp - LOTKA_GAMMA * yp;
}

/* field_hopf — Hopf bifurcation: attracting limit cycle at r = √μ. */
static void field_hopf(float x, float y, float cx, float cy,
                        float *out_vx, float *out_vy)
{
    float xp = (x - cx) * HOPF_SCALE;
    float yp = (y - cy) * HOPF_SCALE;
    float r2 = xp * xp + yp * yp;
    *out_vx = HOPF_MU * xp - yp - xp * r2;
    *out_vy = xp + HOPF_MU * yp - yp * r2;
}

/* field_newton — Newton's method on z³ = 1; three basins of attraction. */
static void field_newton(float x, float y, float cx, float cy,
                          float *out_vx, float *out_vy)
{
    float zx = (x - cx) * NEWTON_SCALE;
    float zy = (y - cy) * NEWTON_SCALE;
    float fz_re = zx * zx * zx - 3.0f * zx * zy * zy - 1.0f;
    float fz_im = 3.0f * zx * zx * zy - zy * zy * zy;
    float fp_re = 3.0f * (zx * zx - zy * zy);
    float fp_im = 6.0f * zx * zy;
    float denom = fp_re * fp_re + fp_im * fp_im + EPS_R2;
    *out_vx = -(fz_re * fp_re + fz_im * fp_im) / denom;
    *out_vy = -(fz_im * fp_re - fz_re * fp_im) / denom;
}

/* field_jet — parabolic Poiseuille pipe-flow profile. */
static void field_jet(float x, float y, float cy, float h_half,
                       float *out_vx, float *out_vy)
{
    (void)x;
    float dy = (y - cy) / h_half;
    if (dy < -1.0f) dy = -1.0f;
    if (dy >  1.0f) dy =  1.0f;
    *out_vx = JET_STRENGTH * (1.0f - dy * dy);
    *out_vy = 0.0f;
}

/* field_hurricane — vortex + inflow with Maxwell-like radial envelope. */
static void field_hurricane(float x, float y, float cx, float cy,
                             float *out_vx, float *out_vy)
{
    float dx = x - cx;
    float dy = y - cy;
    float r  = sqrtf(dx * dx + dy * dy);
    float t_norm = r / HURRICANE_RADIUS;
    float envelope = t_norm * expf(-t_norm * t_norm);
    float invr = 1.0f / (r + 1.0f);
    *out_vx = (-dy * HURRICANE_STRENGTH + -dx * HURRICANE_INFLOW) * envelope * invr;
    *out_vy = ( dx * HURRICANE_STRENGTH + -dy * HURRICANE_INFLOW) * envelope * invr;
}

/* field_hamilton — Hamiltonian double-well: figure-eight separatrix. */
static void field_hamilton(float x, float y, float cx, float cy,
                            float *out_vx, float *out_vy)
{
    float xp = (x - cx) * HAMILTON_SCALE;
    float yp = (y - cy) * HAMILTON_SCALE;
    *out_vx = yp;
    *out_vy = xp - 2.0f * xp * xp * xp;
}

/* field_wake — freestream + downstream alternating vortices. */
static void field_wake(const ActiveAttractor *att, int n_att,
                        float x, float y,
                        float *out_vx, float *out_vy)
{
    float vx = WAKE_FREESTREAM, vy = 0.0f;
    for (int i = 0; i < n_att; i++) {
        float dx = x - att[i].x;
        float dy = y - att[i].y;
        float r2 = dx * dx + dy * dy + EPS_R2;
        float w  = att[i].strength / r2 * (float)att[i].sign;
        vx += -dy * w;
        vy +=  dx * w;
    }
    *out_vx = vx;
    *out_vy = vy;
}

/* field_standing — standing wave: nodes pulse, antinodes oscillate. */
static void field_standing(float x, float y, float t,
                            float *out_vx, float *out_vy)
{
    float temporal = cosf(t * STANDING_OMEGA);
    *out_vx = sinf(x * STANDING_K) * temporal * STANDING_AMP;
    *out_vy = sinf(y * STANDING_K) * temporal * STANDING_AMP;
}

/* field_plane — directional travelling plane wave. */
static void field_plane(float x, float y, float t,
                         float *out_vx, float *out_vy)
{
    float phase = PLANE_KX * x + PLANE_KY * y - PLANE_OMEGA * t;
    float kmag = sqrtf(PLANE_KX * PLANE_KX + PLANE_KY * PLANE_KY);
    float wave = sinf(phase) * PLANE_AMP;
    *out_vx = (PLANE_KX / kmag) * wave;
    *out_vy = (PLANE_KY / kmag) * wave;
}

/* field_gradient — steepest descent on a 2-D Gaussian well. */
static void field_gradient(float x, float y, float cx, float cy,
                            float *out_vx, float *out_vy)
{
    float dx = x - cx;
    float dy = y - cy;
    float r2 = dx * dx + dy * dy;
    float sigma2 = GRADIENT_SIGMA * GRADIENT_SIGMA;
    float falloff = expf(-r2 / (2.0f * sigma2));
    *out_vx = -dx / sigma2 * falloff * GRADIENT_STRENGTH;
    *out_vy = -dy / sigma2 * falloff * GRADIENT_STRENGTH;
}

/* field_eddy — smooth Gaussian-falloff vortex (no singularity). */
static void field_eddy(float x, float y, float cx, float cy,
                        float *out_vx, float *out_vy)
{
    float dx = x - cx;
    float dy = y - cy;
    float r2 = dx * dx + dy * dy;
    float falloff = expf(-r2 / (2.0f * EDDY_SIGMA * EDDY_SIGMA));
    *out_vx = -dy * EDDY_STRENGTH * falloff;
    *out_vy =  dx * EDDY_STRENGTH * falloff;
}

/* field_pitchfork — vx = μx' − x'³, vy = −y'. Two stable equilibria. */
static void field_pitchfork(float x, float y, float cx, float cy,
                             float *out_vx, float *out_vy)
{
    float xp = (x - cx) * PITCHFORK_SCALE;
    float yp = (y - cy) * PITCHFORK_SCALE;
    *out_vx = PITCHFORK_MU * xp - xp * xp * xp;
    *out_vy = -yp;
}

/* field_henon — continuous-time interpretation of the Hénon map. */
static void field_henon(float x, float y, float cx, float cy,
                         float *out_vx, float *out_vy)
{
    float xp = (x - cx) * HENON_SCALE;
    float yp = (y - cy) * HENON_SCALE;
    *out_vx = 1.0f - HENON_A * xp * xp + yp - xp;
    *out_vy = HENON_B * xp - yp;
}

/* ===================================================================== */
/* §6  attractors — pattern-specific source / sink configuration         */
/* ===================================================================== */

/* ── attractor builders ──────────────────────────────────────────── *
 * Small named factories that compress the repeated boilerplate of
 * attractor construction. Used by the per-pattern placers below. */

/* make_marker — single-point '@' marker. Strength = 0 means the
 * marker shows up in the render layer but contributes nothing to
 * any field sum (used by patterns whose field is closed-form). */
static Attractor make_marker(float x, float y)
{
    return (Attractor){
        .bx = x, .by = y,
        .ox = 0.0f, .oy = 0.0f, .phase = 0.0f,
        .strength = 0.0f, .sign = +1,
    };
}

/* make_drifting_vortex — vortex that orbits its anchor with small
 * Lissajous radii. The orbit keeps multi-vortex patterns visually
 * alive rather than rigidly locked. */
static Attractor make_drifting_vortex(float bx, float by, float ox, float oy,
                                       float phase, float strength, int sign)
{
    return (Attractor){
        .bx = bx, .by = by,
        .ox = ox, .oy = oy,
        .phase = phase,
        .strength = strength, .sign = sign,
    };
}

/* place_centre_marker — convenience for the most common case:
 * a single '@' marker at the map centre, no drift. Returns the
 * number of attractors emitted (1 if there's room, else 0). */
static int place_centre_marker(Attractor out[], int max_n, float cx, float cy)
{
    if (max_n < 1) return 0;
    out[0] = make_marker(cx, cy);
    return 1;
}

/* ── per-pattern placers ─────────────────────────────────────────── *
 * Each placer knows the attractor layout for one pattern. They all
 * have the same shape: write into out[], up to max_n, return the
 * count. pattern_init_attractors below is just a dispatcher. */

/* VORTICES — 4 alternating-spin vortices at the corners of an inner
 * square of half-axes (w/4, h/4) around the map centre. Phases offset
 * by 0.7 rad so they drift independently. */
static int place_vortices_quarters(Attractor out[], int max_n,
                                    float cx, float cy, int w, int h)
{
    float rx = (float)w * 0.25f;
    float ry = (float)h * 0.25f;
    int n = 0;
    for (int i = 0; i < 4 && n < max_n; i++) {
        float ang = (float)i * (float)M_PI * 0.5f;
        out[n++] = make_drifting_vortex(
            cx + rx * cosf(ang), cy + ry * sinf(ang),
            4.0f, 4.0f,                  /* drift orbit radii */
            (float)i * 0.7f,             /* phase desync     */
            VORTEX_STRENGTH,
            (i & 1) ? -1 : +1            /* alternate spin   */
        );
    }
    return n;
}

/* MAGNET — opposing poles 40 % of the map width apart, drifting in
 * phase-anti-phase so they oscillate toward and away from each other. */
static int place_magnet_dipole(Attractor out[], int max_n,
                                float cx, float cy, int w)
{
    if (max_n < 2) return 0;
    float dx = (float)w * 0.20f;
    out[0] = make_drifting_vortex(cx - dx, cy, 3.0f, 2.0f,
                                    0.0f,         MAGNET_STRENGTH, +1);
    out[1] = make_drifting_vortex(cx + dx, cy, 3.0f, 2.0f,
                                    (float)M_PI,  MAGNET_STRENGTH, -1);
    return 2;
}

/* TURBULENT — 20 random-sign mini-vortices scattered uniformly. */
static int place_turbulent_field(Attractor out[], int max_n, int w, int h)
{
    int n = 0;
    for (int i = 0; i < 20 && n < max_n; i++) {
        out[n++] = make_drifting_vortex(
            (float)(rand() % w), (float)(rand() % h),
            2.0f, 2.0f,
            (float)i * 0.31f,
            TURBULENT_STRENGTH,
            (rand() & 1) ? +1 : -1
        );
    }
    return n;
}

/* SPIRAL — one strong vortex at the map centre (no drift). */
static int place_spiral_centre(Attractor out[], int max_n, float cx, float cy)
{
    if (max_n < 1) return 0;
    out[0] = make_drifting_vortex(cx, cy, 0.0f, 0.0f, 0.0f,
                                    SPIRAL_STRENGTH, +1);
    return 1;
}

/* GALAXY — single central vortex (no drift); the spiral arms come
 * from the cos(2θ − k·r) modulation inside field_galaxy. */
static int place_galaxy_centre(Attractor out[], int max_n, float cx, float cy)
{
    if (max_n < 1) return 0;
    out[0] = make_drifting_vortex(cx, cy, 0.0f, 0.0f, 0.0f,
                                    GALAXY_STRENGTH, +1);
    return 1;
}

/* CHAIN — Kármán vortex street: row of alternating-sign vortices
 * across the map width, with a small vertical zigzag to break the
 * line's symmetry. */
static int place_chain_alternating(Attractor out[], int max_n,
                                    float cy, int w)
{
    int n = 0;
    float spacing = (float)w / (float)(CHAIN_COUNT + 1);
    for (int i = 0; i < CHAIN_COUNT && n < max_n; i++) {
        out[n++] = make_drifting_vortex(
            spacing * (float)(i + 1),
            cy + ((i & 1) ? CHAIN_ZIGZAG_Y : -CHAIN_ZIGZAG_Y),
            2.0f, 2.0f,
            (float)i * 0.5f,
            CHAIN_STRENGTH,
            (i & 1) ? -1 : +1
        );
    }
    return n;
}

/* GRID — 3×3 checkerboard of mini-vortices, neighbours spin opposite
 * directions (gx + gy parity flips sign). */
static int place_grid_checkerboard(Attractor out[], int max_n, int w, int h)
{
    float dx = (float)w / (float)(GRID_COUNT + 1);
    float dy = (float)h / (float)(GRID_COUNT + 1);
    int n = 0, i = 0;
    for (int gy = 0; gy < GRID_COUNT && n < max_n; gy++) {
        for (int gx = 0; gx < GRID_COUNT && n < max_n; gx++) {
            out[n++] = make_drifting_vortex(
                dx * (float)(gx + 1),
                dy * (float)(gy + 1),
                1.5f, 1.5f,
                (float)i * 0.4f,
                GRID_STRENGTH,
                ((gx + gy) & 1) ? -1 : +1
            );
            i++;
        }
    }
    return n;
}

/* WAKE — cylinder marker at WAKE_OBSTACLE_REL of map width plus
 * WAKE_VORTEX_COUNT alternating-sign vortices downstream forming
 * the Kármán wake. */
static int place_wake_obstacle(Attractor out[], int max_n, float cy, int w)
{
    if (max_n < 1) return 0;
    float obs_x    = (float)w * WAKE_OBSTACLE_REL;
    float dn_start = obs_x + WAKE_OBSTACLE_GAP;
    float spacing  = ((float)w - dn_start) / (float)(WAKE_VORTEX_COUNT + 1);

    int n = 0;
    out[n++] = make_marker(obs_x, cy);
    for (int i = 0; i < WAKE_VORTEX_COUNT && n < max_n; i++) {
        out[n++] = make_drifting_vortex(
            dn_start + spacing * (float)i,
            cy + ((i & 1) ? WAKE_ZIGZAG_AMP : -WAKE_ZIGZAG_AMP),
            1.5f, 1.5f,
            (float)i * 0.4f,
            WAKE_VORTEX_STR,
            (i & 1) ? -1 : +1
        );
    }
    return n;
}

/* QUADPOLE — 4 alternating poles at the corners of an inner square,
 * separation QUADPOLE_SEPARATION of map w/h. Signs walk +−+− around
 * the perimeter for a true quadrupole field. */
static int place_quadpole_corners(Attractor out[], int max_n,
                                   float cx, float cy, int w, int h)
{
    float dx = (float)w * QUADPOLE_SEPARATION;
    float dy = (float)h * QUADPOLE_SEPARATION;
    float xs[4] = { cx - dx, cx + dx, cx + dx, cx - dx };
    float ys[4] = { cy - dy, cy - dy, cy + dy, cy + dy };
    int   sg[4] = { +1, -1, +1, -1 };
    int n = 0;
    for (int i = 0; i < 4 && n < max_n; i++) {
        out[n++] = make_drifting_vortex(xs[i], ys[i], 0.0f, 0.0f,
                                          0.0f, QUADPOLE_STRENGTH, sg[i]);
    }
    return n;
}

/* DUFFING — two stable equilibria at x' = ±1, y' = 0 in dynsys
 * coords, mapped back to cells via 1/DUFFING_SCALE. Markers only;
 * field is closed-form. */
static int place_duffing_equilibria(Attractor out[], int max_n,
                                     float cx, float cy)
{
    if (max_n < 2) return 0;
    float d = 1.0f / DUFFING_SCALE;
    out[0] = make_marker(cx - d, cy);
    out[1] = make_marker(cx + d, cy);
    return 2;
}

/* NEWTON — three cube roots of unity, mapped to cells. */
static int place_newton_roots(Attractor out[], int max_n, float cx, float cy)
{
    if (max_n < 3) return 0;
    float d      = 1.0f / NEWTON_SCALE;
    float root_y = (float)(sin(M_PI / 3.0)) / NEWTON_SCALE;
    out[0] = make_marker(cx + d,        cy);
    out[1] = make_marker(cx - d * 0.5f, cy + root_y);
    out[2] = make_marker(cx - d * 0.5f, cy - root_y);
    return 3;
}

/* HAMILTON — saddle at origin + two centres at x' = ±1/√2. */
static int place_hamilton_equilibria(Attractor out[], int max_n,
                                      float cx, float cy)
{
    if (max_n < 3) return 0;
    float d = 1.0f / ((float)sqrt(2.0) * HAMILTON_SCALE);
    out[0] = make_marker(cx,     cy);   /* saddle */
    out[1] = make_marker(cx + d, cy);   /* +centre */
    out[2] = make_marker(cx - d, cy);   /* -centre */
    return 3;
}

/* PITCHFORK — saddle at origin + two stable equilibria at x' = ±√μ. */
static int place_pitchfork_equilibria(Attractor out[], int max_n,
                                       float cx, float cy)
{
    if (max_n < 3) return 0;
    float d = sqrtf(PITCHFORK_MU) / PITCHFORK_SCALE;
    out[0] = make_marker(cx,     cy);
    out[1] = make_marker(cx + d, cy);
    out[2] = make_marker(cx - d, cy);
    return 3;
}

/*
 * pattern_init_attractors — DISPATCHER. For each pattern, call its
 * placer and return the count. The dispatcher reads as one line per
 * pattern — pure pseudocode.
 *
 * Patterns fall into four groups:
 *   • Multi-attractor / structured layout  → dedicated placer
 *   • Single centre marker (closed-form
 *     fields whose centre is salient)      → place_centre_marker
 *   • Equilibrium-pair markers              → dedicated placer
 *   • No salient point                      → 0 attractors
 */
static int pattern_init_attractors(Attractor out[], int max_n,
                                   Pattern p, int w, int h)
{
    float cx = (float)w * 0.5f;
    float cy = (float)h * 0.5f;

    switch (p) {

    /* Structured multi-attractor layouts. */
    case PATTERN_VORTICES:   return place_vortices_quarters    (out, max_n, cx, cy, w, h);
    case PATTERN_MAGNET:     return place_magnet_dipole        (out, max_n, cx, cy, w);
    case PATTERN_TURBULENT:  return place_turbulent_field      (out, max_n, w, h);
    case PATTERN_SPIRAL:     return place_spiral_centre        (out, max_n, cx, cy);
    case PATTERN_GALAXY:     return place_galaxy_centre        (out, max_n, cx, cy);
    case PATTERN_CHAIN:      return place_chain_alternating    (out, max_n, cy, w);
    case PATTERN_GRID:       return place_grid_checkerboard    (out, max_n, w, h);
    case PATTERN_WAKE:       return place_wake_obstacle        (out, max_n, cy, w);
    case PATTERN_QUADPOLE:   return place_quadpole_corners     (out, max_n, cx, cy, w, h);

    /* Equilibrium-pair / triple markers (closed-form fields). */
    case PATTERN_DUFFING:    return place_duffing_equilibria   (out, max_n, cx, cy);
    case PATTERN_NEWTON:     return place_newton_roots         (out, max_n, cx, cy);
    case PATTERN_HAMILTON:   return place_hamilton_equilibria  (out, max_n, cx, cy);
    case PATTERN_PITCHFORK:  return place_pitchfork_equilibria (out, max_n, cx, cy);

    /* Single centre marker — closed-form fields with one salient point. */
    case PATTERN_RADIAL:
    case PATTERN_RIPPLE:
    case PATTERN_CIRCULAR:
    case PATTERN_VANDERPOL:
    case PATTERN_PENDULUM:
    case PATTERN_LOTKA:
    case PATTERN_HOPF:
    case PATTERN_HURRICANE:
    case PATTERN_GRADIENT:
    case PATTERN_EDDY:
        return place_centre_marker(out, max_n, cx, cy);

    /* No salient point worth marking — pure flow patterns. */
    case PATTERN_WAVE:
    case PATTERN_SADDLE:
    case PATTERN_SHEAR:
    case PATTERN_JET:
    case PATTERN_STANDING:
    case PATTERN_PLANE:
    case PATTERN_HENON:
    default:
        return 0;
    }
}

/*
 * compute_active_attractors — derive each frame's "live" attractor
 * positions from base + orbital drift.
 */
static int compute_active_attractors(const Attractor src[], int n_src,
                                     ActiveAttractor out[], float t)
{
    for (int i = 0; i < n_src; i++) {
        float ang = t * DRIFT_RATE + src[i].phase;
        out[i].x         = src[i].bx + src[i].ox * cosf(ang);
        out[i].y         = src[i].by + src[i].oy * sinf(ang);
        out[i].strength  = src[i].strength;
        out[i].sign      = src[i].sign;
    }
    return n_src;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * The scene is built out of six small structs. Each owns ONE concern,
 * and Scene composes them. Splitting concerns into clearly-typed
 * sub-structs makes function signatures self-describing: any
 * function that takes `const Grid *` clearly cannot mutate buffers;
 * any function that takes `AttractorPool *` clearly does not advance
 * sim time; and so on.
 */

/*
 * Particle — one walker advected by the active pattern's velocity
 * field. Paints a coloured trail; respawns on OOB or max_age.
 *
 * INTENT. The walker is "massless": its position is integrated
 * directly from the sampled velocity, with NO momentum term. This
 * is the standard streamline-tracing integration in flow
 * visualisation — particles trace streamlines of v, not
 * acceleration-trajectories of a force field. The simpler integrator
 * also means visual speed is controlled entirely by ctrl.speed,
 * independent of which field function is active.
 *
 * FINITE LIFETIME. Each particle has a random max_age so the field
 * visualisation stays lively. If particles lived forever they would
 * gradually saturate the streamlines v happens to favour and the
 * rest of the field would empty out (the failure mode you see in
 * naive flow-vis implementations). Random max_age STAGGERS respawn
 * times so the population distribution stays roughly uniform.
 *
 * STABLE COLOUR. color_idx is fixed at spawn and never changes
 * during the particle's life — gives each trail a consistent colour
 * even as it crosses cells previously painted by other particles.
 */
typedef struct {
    float x, y;       /* position in cell units (continuous floats)         */
    int   color_idx;  /* 0..N_BANDS-1 — palette band for this trail         */
    int   age;        /* ticks since spawn                                  */
    int   max_age;    /* respawn when age ≥ max_age (also respawn on OOB)   */
} Particle;

/*
 * Grid — map geometry. Pure data: no buffers, no state. Lives at the
 * top of Scene because every layer (field sampling, screen centring,
 * the particle loop) needs the dimensions.
 *
 * INDEXING. Row-major: cell (x, y) → y·w + x. This matches the
 * memory layout of RenderBuffers (single flat array of CELLS_MAX
 * cells), so the y-outer / x-inner loop order in scene_draw is
 * cache-friendly — each row is contiguous.
 *
 * INVARIANT. w · h ≤ CELLS_MAX always holds; app_pick_map_size()
 * clamps to MAP_W_MAX × MAP_H_MAX. The clamp is enforced once per
 * resize; downstream code may assume it.
 */
typedef struct {
    int w, h;
    int total_cells;
} Grid;

static inline int  grid_idx       (const Grid *g, int x, int y) { return y * g->w + x; }
static inline bool grid_in_bounds (const Grid *g, int x, int y)
{
    return x >= 0 && x < g->w && y >= 0 && y < g->h;
}

/*
 * RenderBuffers — per-cell raster output. Two parallel arrays
 * indexed by grid_idx(g, x, y):
 *   glow  : trail intensity 0..1 — drives glyph picker
 *   color : palette band 0..N_BANDS-1
 *
 * The screen layer reads ONLY these two arrays. particle_step writes
 * into them. That is the entire render contract for the streamline
 * layer; the attractor markers live separately in AttractorPool.
 *
 * SoA RATIONALE. Two arrays (struct-of-arrays) rather than one array
 * of cell-structs because glow is read every cell every frame, while
 * color is read only for non-blank cells. Keeping glow dense gives
 * better cache hit rate on the dense draw loop.
 */
typedef struct {
    float   glow [CELLS_MAX];
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
 * respawn never touches the heap. Only pool[0..n-1] is alive; the
 * rest is unused storage held in reserve.
 *
 * SIZING. MAX_PARTICLES (1024) is the static upper bound;
 * N_PARTICLES_DEF (256) is what scene_reset() activates. 256 covers
 * a 200×56 grid with visible streamline density without saturating
 * the cells (each cell averages ~1 hit per 4 frames at SPEED_DEF).
 * Higher counts produce a denser, more "drawn-in" look; lower
 * counts give sparser, more skeletal streamlines.
 *
 * CACHE. 1024 × sizeof(Particle) ≈ 24 KB sits comfortably in L1.
 * The per-frame step_all_particles loop walks one tight contiguous
 * memory region rather than scattering through the heap — relevant
 * because particle_step calls field_velocity_at, which can dispatch
 * to attractor-sum kernels with their own memory traffic.
 */
typedef struct {
    Particle pool[MAX_PARTICLES];   /* storage; only [0..n-1] is alive */
    int      n;                     /* active count, ≤ MAX_PARTICLES   */
} Particles;

/*
 * AttractorPool — base list (attractor anchors with drift orbits) +
 * per-frame "active" list (drift applied). The active list is what
 * field functions and the renderer both read. Recomputed once per
 * scene_tick via refresh_active_attractors().
 *
 * INTENT. The base/active split avoids re-running cos/sin trig at
 * every cell of every field function — drift is computed once per
 * frame, not per field-sample. Mirrors the classic graphics-pipeline
 * "skinned-vertex" precompute pattern.
 *
 * MARKER ENTRIES. Patterns that don't compute the field from an
 * attractor sum (closed-form patterns like RADIAL, DUFFING, HAMILTON)
 * still populate the pool with strength=0 entries at salient points
 * — equilibria, sources, eye centres — so they show up as bright
 * '@' glyphs. Field functions with `strength · ...` terms see 0 and
 * contribute nothing.
 */
typedef struct {
    Attractor       base [MAX_ATTRACTORS];
    int             n_base;
    ActiveAttractor active[MAX_ATTRACTORS];
    int             n_active;
} AttractorPool;

/*
 * SimState — the single mutable per-tick scalar (kept as a struct
 * for parallelism with the other refactored files in the project
 * and to leave room for future per-tick state — energy, total flux,
 * particle-count histogram, etc.).
 *
 * INTENT. Separated from Controls (keyboard-driven knobs) so it is
 * obvious what scene_tick mutates vs. what the user does. The model
 * vs. user-intent split common to interactive programs: SimState
 * answers "where is the simulation right now"; Controls answers
 * "what has the user asked for".
 *
 * ROLE OF field_time. Threads into:
 *   • Every time-varying field formula — WAVE, RIPPLE, STANDING,
 *     PLANE — as the `t` argument that drives oscillation.
 *   • Every attractor's drift orbit, via the cos/sin terms in
 *     compute_active_attractors(): orbit angle = t·DRIFT_RATE + φ.
 * Same scalar both places, which keeps wave phases and attractor
 * positions in sync (visually: when a wave crest "hits" an
 * attractor, the attractor's orbital position is reproducible).
 */
typedef struct {
    /* Wall time since last scene_reset(), in seconds. Incremented by
     * dt each tick. Cleared to 0 by reset_sim_state(); otherwise
     * grows monotonically. Units: seconds. */
    float field_time;
} SimState;

/*
 * Controls — user-facing knobs. Mutated by app_handle_key(), read by
 * scene_tick() and screen_draw(). Grouping them makes the keyboard
 * handler trivial: `Controls *c = &scene.ctrl;` once at the top,
 * then every key case is a one-line mutation on `c`.
 *
 * INTENT. The split between SimState and Controls is the "model
 * vs. user intent" boundary common to interactive programs. The
 * dependency is strictly one-way: app_handle_key writes Controls,
 * never SimState; scene_tick reads Controls but writes only
 * SimState + buffers. Keeps the code linear and trivially
 * thread-safe should the demo ever be split into input + sim
 * threads.
 *
 * NO prev_pattern. Unlike particle-decay demos in the project,
 * switching patterns here triggers a full scene_reset (see
 * cycle_pattern in §9) because each pattern needs its own attractor
 * layout — there's no need to detect the switch with a one-tick lag.
 */
typedef struct {
    /* Gate for scene_tick: when true the field freezes but the
     * render loop continues so the HUD stays responsive and the
     * user can still cycle themes / glyph sets. */
    bool    paused;

    /* Particle advection scale, cells / sec. Multiplied into v·dt
     * in advect_particle_euler. Doubled by '+', halved by '-',
     * clamped to [SPEED_MIN, SPEED_MAX] = [1, 64]. The doubling
     * step (rather than ±1) gives a logarithmic feel — perceptually
     * each press is the same-size change regardless of current
     * speed. */
    int     speed;

    int     current_theme;       /* index into themes[N_THEMES]   */

    /* Index into glyph_sets[N_GLYPH_SETS]. Mutated by g/G via
     * cycle_glyph_set(). Note: changing the glyph set does NOT
     * trigger a scene_reset — only the rendering layer reads it,
     * not the simulation. */
    int     current_glyph_set;

    /* The active visualisation; mutated by n/p via cycle_pattern.
     * Each switch triggers a full scene_reset to install the new
     * pattern's attractor layout. */
    Pattern current_pattern;
} Controls;

/*
 * Scene — the umbrella context. Reading this struct top-to-bottom is
 * meant to be the fastest way to understand the program:
 *   grid       → where things live      (geometry)
 *   buf        → what gets drawn        (render output)
 *   particles  → moving agents          (advection walkers)
 *   attractors → field's salient points (vortex centres, equilibria)
 *   sim        → animation state        (field_time)
 *   ctrl       → user knobs             (pattern, speed, …)
 *
 * REFERENCE. Mike Acton — "Data-Oriented Design and C++" (CppCon
 * 2014) for the broader argument that good struct layout IS good
 * code.
 */
typedef struct {
    Grid          grid;
    RenderBuffers buf;
    Particles     particles;
    AttractorPool attractors;
    SimState      sim;
    Controls      ctrl;
} Scene;

/* ── particle pipeline ────────────────────────────────────────────── *
 * particle_step is the per-particle pseudocode:
 *
 *     sample unit velocity from active pattern's field
 *     advect by v·speed·dt
 *     deposit trail hit
 *     respawn if expired (age or OOB)
 *
 * Each step is a named helper below. */

static void particle_spawn(Particle *p, const Grid *g)
{
    p->x         = (float)(rand() % g->w);
    p->y         = (float)(rand() % g->h);
    p->color_idx = rand() & (N_BANDS - 1);
    p->age       = 0;
    p->max_age   = AGE_MIN_TICKS + rand() % (AGE_MAX_TICKS - AGE_MIN_TICKS);
}

/*
 * field_velocity_at — dispatch on pattern, compute velocity at (x, y).
 * Reads from scene->attractors.active for attractor-based patterns;
 * computes closed-form for the rest. The single seam between the
 * "which pattern is active" decision and the per-cell math.
 */
static void field_velocity_at(const Scene *s, Pattern pat,
                              float x, float y,
                              float *out_vx, float *out_vy)
{
    const AttractorPool *ap = &s->attractors;
    float cx = (float)s->grid.w * 0.5f;
    float cy = (float)s->grid.h * 0.5f;

    switch (pat) {
    case PATTERN_VORTICES:
    case PATTERN_CHAIN:
    case PATTERN_GRID:
    case PATTERN_TURBULENT:
        field_vortices(ap->active, ap->n_active, x, y, out_vx, out_vy);
        break;

    case PATTERN_WAVE:
        field_wave(x, y, s->sim.field_time, out_vx, out_vy);
        break;

    case PATTERN_SADDLE:
        field_saddle(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_MAGNET:
    case PATTERN_QUADPOLE:
        /* QUADPOLE reuses the MAGNET kernel — only attractor layout
         * (4 alternating poles at square corners) differs. */
        field_magnet(ap->active, ap->n_active, x, y, out_vx, out_vy);
        break;

    case PATTERN_SPIRAL:
        field_spiral(ap->active, ap->n_active, x, y, out_vx, out_vy);
        break;

    case PATTERN_RADIAL:
        field_radial(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_SHEAR:
        field_shear(x, y, cy, out_vx, out_vy);
        break;

    case PATTERN_RIPPLE:
        field_ripple(x, y, s->sim.field_time, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_CIRCULAR:
        field_circular(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_GALAXY:
        field_galaxy(ap->active, ap->n_active, x, y, out_vx, out_vy);
        break;

    case PATTERN_DUFFING:
        field_duffing(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_VANDERPOL:
        field_van_der_pol(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_PENDULUM:
        field_pendulum(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_LOTKA:
        field_lotka(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_HOPF:
        field_hopf(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_NEWTON:
        field_newton(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_JET:
        field_jet(x, y, cy, (float)s->grid.h * 0.5f, out_vx, out_vy);
        break;

    case PATTERN_HURRICANE:
        field_hurricane(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_HAMILTON:
        field_hamilton(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_WAKE:
        field_wake(ap->active, ap->n_active, x, y, out_vx, out_vy);
        break;

    case PATTERN_STANDING:
        field_standing(x, y, s->sim.field_time, out_vx, out_vy);
        break;

    case PATTERN_PLANE:
        field_plane(x, y, s->sim.field_time, out_vx, out_vy);
        break;

    case PATTERN_GRADIENT:
        field_gradient(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_EDDY:
        field_eddy(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_PITCHFORK:
        field_pitchfork(x, y, cx, cy, out_vx, out_vy);
        break;

    case PATTERN_HENON:
        field_henon(x, y, cx, cy, out_vx, out_vy);
        break;

    default:
        *out_vx = 0.0f;
        *out_vy = 0.0f;
        break;
    }
}

/* sample_unit_velocity — sample the field at (px, py) and normalise.
 * Decoupling direction from magnitude here lets ctrl.speed control
 * visual pace independently of which pattern is active (raw
 * magnitudes vary across patterns by orders of magnitude). */
static void sample_unit_velocity(const Scene *s, Pattern pat,
                                  float px, float py,
                                  float *out_vx, float *out_vy)
{
    field_velocity_at(s, pat, px, py, out_vx, out_vy);
    float m = sqrtf((*out_vx) * (*out_vx) + (*out_vy) * (*out_vy));
    if (m > VELOCITY_EPSILON) {
        *out_vx /= m;
        *out_vy /= m;
    }
}

/* advect_particle_euler — forward-Euler integration step. Massless
 * advection: (x, y) ← (x, y) + v · speed · dt. Streamlines, not
 * trajectories (no inertia). */
static void advect_particle_euler(Particle *p, float vx, float vy,
                                   float dt, int speed)
{
    p->x += vx * (float)speed * dt;
    p->y += vy * (float)speed * dt;
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

static void particle_step(Scene *s, Particle *p, Pattern pat,
                          float dt, int speed)
{
    float vx, vy;
    sample_unit_velocity (s, pat, p->x, p->y, &vx, &vy);
    advect_particle_euler(p, vx, vy, dt, speed);
    deposit_trail_hit    (s, (int)p->x, (int)p->y, p->color_idx);
    if (particle_is_expired(p, &s->grid))
        particle_spawn   (p, &s->grid);
}

/* ── tick pipeline ────────────────────────────────────────────────── *
 * scene_tick is the per-frame pseudocode:
 *
 *     if paused: stop.
 *     decay all trail glows exponentially.
 *     advance field_time by dt.
 *     refresh attractor positions from base + drift.
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

static void refresh_active_attractors(AttractorPool *ap, float sim_time)
{
    ap->n_active = compute_active_attractors(ap->base, ap->n_base,
                                              ap->active, sim_time);
}

static void step_all_particles(Scene *s, float dt)
{
    Pattern pat = s->ctrl.current_pattern;
    int     spd = s->ctrl.speed;
    for (int i = 0; i < s->particles.n; i++)
        particle_step(s, &s->particles.pool[i], pat, dt, spd);
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
    sim->field_time = 0.0f;
}

static void spawn_all_particles(Particles *ps, const Grid *g)
{
    ps->n = N_PARTICLES_DEF;
    for (int i = 0; i < ps->n; i++)
        particle_spawn(&ps->pool[i], g);
}

static void init_pattern_attractors(AttractorPool *ap, Pattern p,
                                     const Grid *g)
{
    ap->n_base = pattern_init_attractors(ap->base, MAX_ATTRACTORS,
                                          p, g->w, g->h);
    ap->n_active = 0;
}

static void scene_reset(Scene *s, int w, int h)
{
    apply_grid_dimensions    (&s->grid, w, h);
    reset_sim_state          (&s->sim);
    buffers_clear            (&s->buf, s->grid.total_cells);
    init_pattern_attractors  (&s->attractors, s->ctrl.current_pattern, &s->grid);
    spawn_all_particles      (&s->particles, &s->grid);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->ctrl.paused            = false;
    s->ctrl.speed             = SPEED_DEF;
    s->ctrl.current_theme     = 0;
    s->ctrl.current_glyph_set = 2;        /* MEDIUM */
    s->ctrl.current_pattern   = PATTERN_VORTICES;
    scene_reset(s, w, h);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;

    decay_trail_glow           (&s->buf, s->grid.total_cells, dt);
    advance_field_time         (&s->sim, dt);
    refresh_active_attractors  (&s->attractors, s->sim.field_time);
    step_all_particles         (s, dt);
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

/* ── scene_draw pipeline ──────────────────────────────────────────── *
 * Two passes:
 *
 *     1. Trail layer — density glyphs from the current GlyphSet,
 *        coloured by buf.color band.
 *     2. Attractor layer — bright '@' markers on top, in the theme's
 *        flash colour. */

/*
 * CellDraw — the output of the per-cell density-band classifier in
 * scene_draw. A tiny value-type bundling "what to paint at this
 * position"; paint_cell() converts it into the actual ncurses
 * attron / mvaddch / attroff triple.
 *
 * INTENT. cell_density_band is a pure function — given (band, glow,
 * GlyphSet) it returns a CellDraw with no side effects. Routing it
 * through CellDraw rather than calling ncurses directly keeps the
 * decision logic separate from the I/O. paint_cell is then the
 * SINGLE place the program touches ncurses for the trail layer,
 * which makes the renderer trivial to retarget (a test harness
 * could collect CellDraws into a buffer for snapshot comparison
 * without ever opening a terminal).
 *
 * The `skip` flag is the explicit "blank — don't draw anything"
 * signal. Distinct from drawing a space character because skipping
 * preserves whatever was painted before (we erase() once per frame,
 * not per cell). Most cells skip — full grid is mostly empty unless
 * particles cross it.
 */
typedef struct {
    int  pair;   /* colour pair index — PAIR_TRAIL_BASE + band index   */
    int  attr;   /* ncurses attributes — A_BOLD for mid/high, A_NORMAL */
    char glyph;  /* ASCII byte from the active GlyphSet                */
    bool skip;   /* true → render nothing for this cell                */
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

/* cell_density_band — pick the density-band glyph for a cell. */
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

/* draw_attractor_layer — pass 2: bright '@' markers on top. */
static void draw_attractor_layer(const AttractorPool *ap,
                                  int gx0, int gy0, int cols, int rows)
{
    attron(COLOR_PAIR(PAIR_ATTRACT) | A_BOLD);
    for (int i = 0; i < ap->n_active; i++) {
        int sx = gx0 + (int)ap->active[i].x;
        int sy = gy0 + (int)ap->active[i].y;
        if (sx < 0 || sx >= cols) continue;
        if (sy < 0 || sy >= rows) continue;
        mvaddch(sy, sx, (chtype)(unsigned char)'@');
    }
    attroff(COLOR_PAIR(PAIR_ATTRACT) | A_BOLD);
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
    draw_trail_layer    (s, gx0, gy0, cols, rows, gs);
    draw_attractor_layer(&s->attractors, gx0, gy0, cols, rows);
}

/* ── HUD draw pipeline ────────────────────────────────────────────── *
 * Six named drawers, called from screen_draw in z-order. The TOP HUD
 * (rows 0..2) carries DATA — current state with [N/30] index,
 * parameter readouts, glyph indicator, legend. The BOTTOM HUD
 * (row N-1) carries ACTIONS — key bindings only.
 *
 * draw_hud_status_line internally composes four smaller segment
 * drawers (one per fixed-width row-1 field); see "row 1 segment
 * drawers" below. */

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
    mvprintw(0, HUD_LEFT_MARGIN, " FLOW FIELD PARTICLES ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* draw_palette_swatch — paint one '#' per band colour. Returns the
 * next x-cursor so the caller can continue laying out. */
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

/* ── row 1 segment drawers ──────────────────────────────────────────── *
 * draw_hud_status_line lays out row 1 left-to-right as a sequence of
 * fixed-width segments. Each segment is a named drawer that takes the
 * current x-cursor, paints its content, and returns the new x-cursor.
 * The driver function then reads as pure pseudocode. */

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

/* draw_status_sim_counts — last segment: live simulation statistics
 * (particle count, attractor count, map dimensions). No return — end
 * of the row. */
static void draw_status_sim_counts(int row, int x, const Scene *s)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  parts:%d  attr:%d  map:%dx%d ",
             s->particles.n, s->attractors.n_active,
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

/* draw_hud_legend — row 2: glyph-meaning reference. Belongs in the
 * top HUD because it's DATA (how to READ the screen), not an
 * ACTION (what you can press). */
static void draw_hud_legend(void)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(2, HUD_LEFT_MARGIN,
             " trails: density-graded (low -> mid -> high)   @: attractor / equilibrium ");
    attroff(COLOR_PAIR(PAIR_HUD));
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
 * so signal handlers can reach it without a global Scene pointer.
 * The App owns the Scene and the Screen and adds simulation
 * parameters (sim_fps, map_w, map_h) plus signal-driven flags.
 *
 * SIGNAL-HANDLER DISCIPLINE. Handlers do nothing but set a flag; the
 * main loop polls and performs the actual work in normal context.
 * Standard async-signal-safe pattern — see signal(7), and Stevens &
 * Rago "Advanced Programming in the UNIX Environment" ch. 10.
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

/* bump_speed_geometric — '+' / '-' geometric step. Doubling /
 * halving gives a logarithmic feel: each press is the same
 * perceptual change across the full clamped range. */
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
 * pattern needs its own attractor configuration; reusing the
 * previous pattern's attractors would give a glitched first frame. */
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
    case 't':           cycle_theme   (c,   +1);                           break;
    case 'T':           cycle_theme   (c,   -1);                           break;
    case 'g':           cycle_glyph_set(c,  +1);                           break;
    case 'G':           cycle_glyph_set(c,  -1);                           break;
    case 'n': case 'N': cycle_pattern (app, +1);                           break;
    case 'p': case 'P': cycle_pattern (app, -1);                           break;
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
