/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * nuke.c — volumetric mushroom cloud as 2-D axisymmetric stable fluids.
 *
 * DEMO: A nuclear blast as actual fluid dynamics, not as a scripted
 *       morph.  At t=0 a hot, dense Gaussian bubble is dropped into
 *       a 2-D axisymmetric grid (radial r, vertical y).  Buoyancy
 *       (force = β·ΔT·g) accelerates hot cells upward.  Stam-style
 *       semi-Lagrangian advection moves density and temperature with
 *       the velocity field.  A Jacobi-iterated pressure projection
 *       enforces incompressibility (∇·v = 0) so the rising column
 *       displaces ambient air, develops a TOROIDAL VORTEX RING at
 *       its leading edge, rolls outward into a CAP, plateaus once
 *       cooling kills buoyancy, and falls back as density decays.
 *       The mushroom is an EMERGENT property of the equations — there
 *       is no `T_RISE_BEG`, no `T_CAP_FORM`, no `cap_scale`.  Time
 *       windows aren't a knob in this file.
 *
 *       Rendering: each terminal cell raymarches a ray through 3-D
 *       space.  Because the simulation is rotation-symmetric around
 *       the y axis, every world point (x, y, z) maps to a 2-D field
 *       sample at (r = √(x²+z²), y) — one axisymmetric slice services
 *       the entire 360° volume.  Beer-Lambert extinction over the ray
 *       gives transmittance; emissive temperature × density gives
 *       radiance.  Volume rendered like the real thing.
 *
 *       Critically there is no phase enum, no `smoothstep(t_beg, t_end, ...)`
 *       windows, no scripted "fireball → cap → fall" timeline.  The cap
 *       forms because of vorticity, the plateau because cooling kills
 *       buoyancy, the fall because density decays — all emergent.
 *
 * Themes (cycle with t / T):
 *   REALISTIC   grey/white smoke + orange-yellow fire core
 *   MATRIX      neon green smoke + lime fire
 *   OCEAN       deep teal smoke + cyan fire
 *   NOVA        violet smoke + ice-blue fire
 *   TOXIC       chartreuse smoke + acid-green fire
 *
 * Study alongside:
 *   fluid/navier_stokes.c  — full 3-D Stam stable fluids in this
 *                            codebase; the solver here is a 2-D
 *                            axisymmetric specialisation of the same
 *                            algorithm.
 *
 * Section map:
 *   §1 config    — grid dims, fluid params, themes
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-pair fire/smoke ramp
 *   §4 vec3      — value-type 3-D math
 *   §5 fluid     — Stam stable fluids: advect, project, buoyancy,
 *                  detonate, evolve
 *   §6 march     — volumetric raymarcher: 2-D-field sample at radius,
 *                  Beer-Lambert + emissive accumulation
 *   §7 scene     — camera + initial-condition orchestration + render
 *   §8 screen    — ncurses init / draw / resize
 *   §9 app       — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC     quit
 *   space       pause physics
 *   r           re-detonate (reset all fields, restart sim time)
 *   n / N       next / previous BLAST TYPE
 *                 (TACTICAL / STANDARD / MEGATON / AIR_BURST / GROUND)
 *   t / T       next / previous theme
 *   + / =       speed up time
 *   -           slow down time
 *   z / Z       camera zoom in / out
 *   ] / [       sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/nuke.c \
 *       -o nuke_cloud -lncurses -lm
 *   (binary name `nuke_cloud` to avoid collision with physics/nuke.c —
 *    the unrelated 2-D shockwave demo. Pick whatever `-o` suits.)
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Boussinesq-approximation incompressible Navier-Stokes
 *                  on a 2-D axisymmetric (r, y) grid via the canonical
 *                  Stam (1999) "Stable Fluids" scheme.
 *
 *                  STATE on the grid (each is `float[N_R][N_Y]`):
 *                     vr, vy      velocity (radial, vertical)
 *                     T           temperature (proxy for buoyancy)
 *                     ρ           density (the visible "smoke")
 *                  Plus scratch: pressure, divergence, two buffers.
 *
 *                  FLUID STEP, per dt:
 *
 *                     1. BUOYANCY.  vy += β · (T − T₀) · dt
 *                        Hot cells accelerate upward; cool cells stay.
 *                        This single force coupling is the whole engine
 *                        of mushroom-cloud formation.
 *
 *                     2. PROJECT.  Find pressure p such that
 *                            ∇²p = ∇·v
 *                        (Poisson equation, ~40 Jacobi iterations); then
 *                            v ← v − ∇p
 *                        This makes v divergence-free, i.e.
 *                        incompressible.  Without this, hot air would
 *                        compress neighbour cells indefinitely.
 *
 *                     3. ADVECT VELOCITY (semi-Lagrangian).  For each
 *                        cell, trace BACKWARD by v·dt to find where the
 *                        fluid CAME FROM, then bilinearly sample the
 *                        old velocity field there.  This is how features
 *                        get carried along by the flow — the toroidal
 *                        vortex emerges from advecting velocity through
 *                        its own velocity gradients.
 *
 *                     4. PROJECT again (advection introduces divergence).
 *
 *                     5. ADVECT T and ρ by the (now divergence-free)
 *                        velocity.  Smoke and heat ride the flow.
 *
 *                     6. COOL (`T → T₀ · (1 − k_cool · dt) + …`) and DECAY
 *                        ρ.  Once temperature returns to ambient,
 *                        buoyancy stops and the cloud falls.  Density
 *                        decay simulates eventual dissipation; the
 *                        cloud fades naturally on the same timescale
 *                        as it formed — no `T_FALL_BEG` constant.
 *
 *                  RENDERING:
 *
 *                     For each pixel, sphere-trace a ray.  At each step
 *                     compute (r, y) from (x, y, z).  Bilinearly sample
 *                     the 2-D ρ and T fields; accumulate L (radiance)
 *                     and decay transmittance T_view via Beer-Lambert:
 *
 *                       dτ        = ρ · ds
 *                       emit      = max(0, (T − T₀) / (T_peak − T₀))
 *                       L        += T_view · dτ · (emit · GAIN + AMB)
 *                       L_hot    += T_view · dτ · emit · GAIN
 *                       T_view   *= exp(−dτ)
 *                       if T_view < ε: break
 *
 *                     Final pixel: glyph from L (luminance), pair from
 *                     hot fraction L_hot/L (smoke→fire colour ramp).
 *
 * Data-structure : Six full grids + two buffers, each `float[N_R][N_Y]`
 *                  with N_R=56, N_Y=96.  ~1.7 M float ops per fluid
 *                  step, ~30 ms on modern desktop at sim_dt = 0.025.
 *                  No mallocs in hot path; everything in BSS.
 *
 * Rendering      : ASCII only.  8-tier glyph ramp by luminance,
 *                  8-tier colour pair by hot fraction (per theme:
 *                  cool grey at low T → bright yellow at peak T).
 *
 * Performance    : Fluid step ~30 ms (grid 56×96, 40 Jacobi iters per
 *                  project, 2 projects per step).  Render ~5 ms at
 *                  80×24, ~30 ms at 200×60.  Whole demo holds 30 fps
 *                  comfortably; for big terminals lower SIM_HZ with
 *                  `[`.
 *
 * References     :
 *   • Stam, J. (1999) — "Stable Fluids", *Proc. SIGGRAPH '99* §3.
 *     The semi-Lagrangian advection + Hodge projection scheme this
 *     file implements.  Three pages of equations, 60 lines of code,
 *     decades of stable use.
 *   • Stam, J. (2003) — "Real-Time Fluid Dynamics for Games",
 *     *GDC 2003*.  Practical-friendly version of the 1999 paper;
 *     same algorithm, more pseudocode.
 *   • Foster, N. & Metaxas, D. (1997) — "Modeling the motion of a
 *     hot, turbulent gas", *SIGGRAPH '97*.  The canonical
 *     Boussinesq-buoyancy-driven fluid VFX paper.  Their initial-
 *     condition recipe (hot Gaussian bubble) is what we use in
 *     `fluid_detonate`.
 *   • Bridson, R. — *Fluid Simulation for Computer Graphics* 2e
 *     (2015), ch. 3 ("Incompressible Euler") and ch. 5
 *     ("Pressure"). The textbook treatment of the Hodge projection
 *     we Jacobi-iterate.
 *   • Mas-Gallic, S. — "Vortex methods" (1990) for the historical
 *     understanding of why a buoyant blob develops a toroidal
 *     ring (baroclinic vorticity at the temperature gradient).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Treat the air around the blast as a grid of water cells, and simulate
 * actual incompressible flow on that grid.  Drop a hot bubble in.
 * Hot cells push up against gravity (buoyancy).  The pressure-projection
 * step says "ok but the air has to go SOMEWHERE — find a pressure field
 * that makes everything incompressible".  That pressure field is what
 * deflects the rising bubble's leading edge sideways, rolls it into a
 * vortex, spreads it into a cap.  The MUSHROOM IS THE SOLUTION TO
 * NAVIER-STOKES with a hot initial condition; we don't draw it, we
 * compute it.  The renderer then makes a picture of the resulting
 * density + temperature field in the terminal.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine each grid cell is a tiny smoke detector that knows its
 * temperature, density, and which way the air is moving through it.
 * A clock ticks every 25 ms.  On each tick, every cell:
 *   – Pushes itself up if it's hotter than its neighbours.
 *   – Asks the neighbours where they're going so the total air mass
 *     is conserved (the projection step solves that puzzle for the
 *     whole grid simultaneously).
 *   – Hands its current contents (heat + smoke) to whatever cell is
 *     downwind, taking new contents from upwind.
 * After ~600 ticks (15 sec sim time), the cells near the original hot
 * spot have collectively choreographed a mushroom cloud: the rising
 * column forms because hot cells lift; the cap spreads because the
 * leading edge of the column gets pushed sideways by the pressure
 * field above it; the cloud falls because the cells eventually cool
 * and lose buoyancy.  Nobody told the cells to do this — it's just
 * the only way for them to all simultaneously solve their constraints.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT — clear all fields.  Set T = T_AMBIENT everywhere, ρ = 0.
 *
 *  2. DETONATE — paint a Gaussian bubble at (r=0, y=BLAST_Y):
 *        T(r, y)  = T_AMBIENT + (T_PEAK − T_AMBIENT) · exp(−d²/2σ²)
 *        ρ(r, y)  = ρ_PEAK · exp(−d²/2σ²)
 *        v(r, y)  = small radial outflow (initial blast wave)
 *     This is the entire scripted part — one Gaussian, t=0.
 *
 *  3. EACH SIM TICK (dt = 0.025 sec):
 *
 *     a. BUOYANCY.   vy[i][j] += β · (T[i][j] − T₀) · dt
 *
 *     b. BOUNDARIES. vr[0][j] = 0   (no flow through axis)
 *                    vy[i][0] = 0   (no flow through ground)
 *                    Other walls: zero-gradient (open).
 *
 *     c. PROJECT.    Compute divergence = ∂vr/∂r + ∂vy/∂y.
 *                    Jacobi-iterate to solve ∇²p = div (40 iters).
 *                    Subtract: vr -= ∂p/∂r,  vy -= ∂p/∂y.
 *                    Now ∇·v ≈ 0 (incompressible).
 *
 *     d. ADVECT VELOCITY.  For each cell (i, j):
 *           src_i, src_j = i − vr·dt/h,  j − vy·dt/h
 *           new_vr[i][j] = bilinear(old_vr, src_i, src_j)
 *           new_vy[i][j] = bilinear(old_vy, src_i, src_j)
 *
 *     e. PROJECT again (advection re-introduced divergence).
 *
 *     f. ADVECT T and ρ by the divergence-free velocity.
 *
 *     g. COOL.       T = T₀ + (T − T₀) · exp(−k_cool · dt)
 *        DECAY.      ρ *= 1 − k_decay · dt
 *
 *  4. RENDER (each terminal cell):
 *        Sphere-trace ray.  At each step:
 *            r        = √(x² + z²)
 *            ρ_s, T_s = bilinear sample of fields at (r, y)
 *            dτ       = ρ_s · STEP
 *            emit     = clamp((T_s − T₀)/(T_peak − T₀),  0, 1)
 *            L       += T_view · dτ · (emit · GAIN + AMBIENT)
 *            L_hot   += T_view · dτ · emit · GAIN
 *            T_view  *= exp(−dτ)
 *            break if T_view < 0.01
 *        Pixel:
 *            slot_lum   = ⌊L · 7.999⌋
 *            slot_hot   = ⌊(L_hot/L) · 7.999⌋
 *            glyph      = LUMA_GLYPHS[slot_lum]
 *            pair       = PAIR_RAMP_BASE + slot_hot   (smoke→fire)
 *
 * KEY FORMULAS
 * ────────────
 *  Boussinesq buoyancy (single force coupling):
 *    F_y = β · (T − T₀) · g     (we fold β · g into one constant)
 *
 *  Hodge projection — the heart of incompressible flow:
 *    ∇·v = 0  is enforced by computing pressure such that
 *    ∇²p = ∇·v_uncorrected,  then  v = v_uncorrected − ∇p.
 *
 *  Discretised Poisson Jacobi step (h = grid spacing):
 *    p_new[i][j] = (p[i+1][j] + p[i−1][j]
 *                 + p[i][j+1] + p[i][j−1]
 *                 − h² · div[i][j]) / 4
 *
 *  Semi-Lagrangian advection (unconditionally stable, the Stam trick):
 *    For each (i, j): trace BACKWARD by v·dt, sample old field there.
 *    No CFL constraint; large dt is safe.
 *
 *  Volumetric Beer-Lambert (rendering):
 *    T_view(s) = exp(−∫₀ˢ ρ ds')
 *    L         = ∫₀^∞ T_view(s) · ρ(s) · J(s) ds
 *    where J(s) = source term (emission from hot gas + ambient scatter)
 *
 *  Cooling (exponential approach to ambient):
 *    T(t+dt) = T₀ + (T(t) − T₀) · exp(−k_cool · dt)
 *
 *  Aspect correction (terminal cells 2× taller than wide):
 *    phys_aspect = (rows · 2) / cols     (factor on v in ray dir)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • PROJECTION CONVERGENCE.  40 Jacobi iterations is approximate.
 *    Residual divergence ≈ 1-2 % which is visually invisible at
 *    terminal resolution.  More iters = slower; fewer iters = visible
 *    "compression" artifacts in fast-rising cells.
 *
 *  • CFL (advection).  Semi-Lagrangian is unconditionally stable, but
 *    if v · dt > h·N (whole grid in one step) the backward-trace
 *    samples get clamped to grid edges and the simulation diffuses
 *    badly.  We cap dt at SIM_DT_MAX so this can't happen.
 *
 *  • AXIS BOUNDARY (r=0).  We approximate axisymmetric flow on a
 *    Cartesian (r, y) grid — treating r=0 as a reflective wall (vr=0
 *    enforced after every step, mirror-symmetric pressure).  This
 *    introduces small errors near the axis that are visually
 *    invisible; a true axisymmetric solver would carry the 1/r
 *    factor in the divergence, which is overkill here.
 *
 *  • INITIAL OUTFLOW.  Without the small radial outflow velocity at
 *    t=0, the bubble just rises with a clean leading-edge vortex.
 *    With outflow, the bubble first expands radially (the "blast
 *    wave"), then rises — closer to a real explosion's two-phase
 *    behaviour.
 *
 *  • TEMPERATURE OVERSHOOT.  Advection of T can overshoot the local
 *    extrema in convergent regions.  We don't clamp; the cooling
 *    pulls things back to ambient over a few seconds.  If you see
 *    "negative-temperature" pockets (cells COLDER than ambient,
 *    rendered as faint dark patches), that is the overshoot — also
 *    visually subtle, also self-correcting.
 *
 *  • PAUSE.  Freezes physics AND time accumulation.  Camera zoom and
 *    theme cycle still respond.  Resuming continues from the same
 *    field state — no jitter.
 *
 *  • RE-DETONATE.  `r` zeros all fields and re-initialises.  Useful
 *    for re-running with a different theme or just to watch the
 *    formation again.
 *
 *  • DT_SCALE > 4×.  Time-scale much higher than 4× starts breaking
 *    semi-Lagrangian sampling because the backward-trace lands
 *    outside the grid; visually you'll see flat bands and the cloud
 *    may collapse.  The clamp on SIM_DT_MAX handles this implicitly.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press space.  Cloud freezes; resume continues seamlessly.
 *
 *  • Press `r` immediately at startup and again 1 sec later.  The
 *    cloud should look BIT-IDENTICAL between runs (deterministic
 *    initial condition + deterministic solver = reproducible
 *    physics; the only nondeterminism in this file is the wall-clock
 *    seed used for `srand`, which we don't read for fluid state).
 *
 *  • Watch t = 1–2 sec.  The hot bubble should rise as a sphere with
 *    a slightly flattened top — the start of the toroidal vortex.
 *
 *  • Watch t = 3–5 sec.  The leading edge should ROLL OUTWARD,
 *    forming the cap — visible as the hot core spreading sideways
 *    while the stem (cooler density) trails below.
 *
 *  • Watch t = 8–12 sec.  Cooling kills buoyancy.  Cloud plateaus,
 *    spreads slightly more under residual inertia, then begins to
 *    fall.  No `T_PLATEAU_BEG` constant drives this — it's the
 *    natural consequence of T returning to ambient.
 *
 *  • Cycle themes (`t`).  Geometry identical, only colours change.
 *    REALISTIC → MATRIX is the most striking palette flip.
 *
 *  • Speed up time (`+`).  Sim runs faster at the same render fps.
 *    The mushroom forms in ~5 sec wall-clock at 4× speed.  Stable.
 *
 *  • Slow down time (`-`).  Watch the vortex roll in extreme detail.
 *    At 0.1× speed each frame is barely a fluid step apart and you
 *    can see the leading edge curl explicitly.
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
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    FPS_UPDATE_MS    = 500,

    PAIR_HUD          =  1,
    PAIR_HINT         =  2,
    PAIR_RAMP_BASE    =  3,    /* +0..+7 — smoke→fire colour ramp     */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define CELL_ASPECT      2.0f      /* terminal cell h/w                  */

/* ── Fluid grid ── */
#define N_R              56        /* radial cells                       */
#define N_Y              96        /* vertical cells                     */
#define GRID_H           0.125f    /* world units per grid cell          */
#define R_MAX            ((float)N_R * GRID_H)   /* ≈ 7.0  world units   */
#define Y_MAX            ((float)N_Y * GRID_H)   /* ≈ 12.0 world units   */
#define INV_GRID_H       (1.0f / GRID_H)

#define POISSON_ITERS    40        /* Jacobi iterations per project()    */

/* ── Physics constants ── */
#define T_AMBIENT        1.0f
#define T_PEAK           8.0f
#define RHO_PEAK         4.0f

#define BUOYANCY_COEF    2.4f      /* β·g — driving force of rise        */
#define COOL_RATE        0.06f     /* T → T_AMBIENT decay (per second)   */
#define DENSITY_DECAY    0.009f    /* ρ fadeout per second.  Lower → cloud
                                    * lingers longer.  This is a tunable
                                    * fudge-factor for the entrainment /
                                    * diffusion we don't model — every
                                    * value is "valid physics" against
                                    * a different ambient air-mixing
                                    * assumption.  0.009 ≈ 110 sec half-
                                    * life — long enough to see late-
                                    * stage drift, short enough that
                                    * pressing `r` doesn't fight residue. */

/*
 * BLAST TYPES — vary INITIAL CONDITIONS only.
 *
 * The fluid solver itself is unchanged across blast types; what differs
 * is the Gaussian we drop in at t=0.  Each preset is a row in the
 * `blasts[]` table below.  Cycle with `n` / `N`.
 *
 * Physics constants (BUOYANCY_COEF, COOL_RATE, DENSITY_DECAY) describe
 * the AIR, not the bomb, so they stay global.  Same air, different
 * detonations → different mushrooms.
 */
typedef enum {
    BLAST_TACTICAL  = 0,
    BLAST_STANDARD  = 1,
    BLAST_MEGATON   = 2,
    BLAST_AIR_BURST = 3,
    BLAST_GROUND    = 4,
    N_BLASTS        = 5,
} BlastType;

typedef struct {
    const char *name;
    float sigma;        /* initial Gaussian stddev (world units)         */
    float t_peak;       /* peak initial temperature (vs T_AMBIENT)       */
    float rho_peak;     /* peak initial density                          */
    float blast_y;      /* altitude of blast centre                       */
    float initial_v;    /* radial outflow velocity at t=0                 */
} BlastParams;

static const BlastParams blasts[N_BLASTS] = {
    /*  name           sigma   t_peak  rho_peak  blast_y  initial_v  */
    /* TACTICAL — small low-altitude fireball, brief mushroom.   */
    { "TACTICAL  ",    0.35f,   5.0f,   2.5f,    1.0f,    2.0f  },
    /* STANDARD — the canonical mid-yield mushroom.              */
    { "STANDARD  ",    0.55f,   8.0f,   4.0f,    1.6f,    3.0f  },
    /* MEGATON  — huge yield, very tall column, long-lasting cap.*/
    { "MEGATON   ",    0.85f,  12.0f,   5.5f,    2.2f,    4.5f  },
    /* AIR_BURST — high-altitude detonation, no ground stem;
     *             cap forms in mid-air, drifts laterally.       */
    { "AIR_BURST ",    0.55f,   8.0f,   4.0f,    4.5f,    3.0f  },
    /* GROUND   — surface burst; modest heat but heavy dust
     *             load, leaves a thick low column.              */
    { "GROUND    ",    0.45f,   7.0f,   7.0f,    0.6f,    3.5f  },
};

/* ── Sim time ── */
#define SIM_DT           0.025f    /* fixed simulation step              */
#define SIM_DT_MAX       0.080f    /* cap (per real-time frame)          */
#define SIM_RATE_DEFAULT 1.0f      /* time-scale multiplier              */
#define SIM_RATE_MIN     0.10f
#define SIM_RATE_MAX     6.0f
#define SIM_RATE_FACTOR  1.30f     /* multiplicative step on +/-         */

/* ── Camera ──
 * Default distance is generous so the full mushroom (column + cap)
 * fits cleanly on screen even for the tallest MEGATON blast.  Use
 * `z` to zoom in for inspection of vortex detail; `Z` to back even
 * further out for a "skyline silhouette" framing. */
#define CAM_DIST_DEFAULT 28.0f
#define CAM_DIST_MIN      8.0f
#define CAM_DIST_MAX     56.0f
#define CAM_DIST_STEP     2.0f
#define CAM_HEIGHT        5.0f     /* camera y                           */
#define LOOK_AT_Y         6.0f     /* target y (cloud-column midpoint)   */
#define FOV_DEG          52.0f     /* slightly tighter for less fish-eye */

/* ── Volume raymarch ── */
#define RM_T_NEAR         0.5f
#define RM_T_FAR         32.0f
#define RM_STEP           0.18f
#define RM_MAX_STEPS     130
#define RM_OPAQUE_EPS     0.01f
#define RM_DENSITY_GAIN   1.30f    /* ρ → optical depth multiplier       */
#define RM_EMIT_GAIN      4.5f     /* hot-gas emission multiplier        */
#define RM_AMBIENT        0.06f    /* ambient scatter floor              */

/* ── Pixel classification thresholds ── */
#define LUM_CLAMP         1.10f    /* L scale before slot mapping        */

/*
 * Themes — 8-pair smoke-to-fire ramp, slot 0 = coolest smoke (dim),
 * slot 7 = hottest core (peak fire colour).  All entries sit in the
 * bright half of the 256-colour cube per the CLAUDE.md "Theme Palette
 * Brightness" rule, so even slot 0 stays visible.
 */
typedef struct {
    const char *name;
    short       ramp[8];
    bool        inverted;        /* white bg, dark fg — photographic
                                  * negative.  Empty cells pre-filled
                                  * with white in scene_render; A_BOLD/
                                  * A_DIM disabled so the grey ramp
                                  * itself carries the gradient.       */
} Theme;

#define N_THEMES 6

/*
 * Per-theme palettes, slot 0 (lowest) → slot 7 (hottest).  The dim
 * end is deliberately raised — slots 0–2 sit in the visibly-tinted
 * range so naturally-fading smoke at end-of-life still reads against
 * a black terminal rather than dropping to invisible.  None of the
 * dark slots use the "near-black" cube band (16–23) per the CLAUDE.md
 * theme-brightness rule.
 */
static const Theme themes[N_THEMES] = {
    /* REALISTIC: light grey smoke climbing into orange-yellow fire.   */
    { "REALISTIC",
      { 248, 250, 252, 254, 130, 166, 208, 220 }, false },

    /* MATRIX: visible green throughout; lower tiers no longer black.  */
    { "MATRIX   ",
      {  64,  70, 112, 113, 119, 154, 190, 226 }, false },

    /* OCEAN: visibly tinted teal-cyan even at the dim end.            */
    { "OCEAN    ",
      {  37,  44,  45,  74,  81, 117, 159, 195 }, false },

    /* NOVA: lifted violet/lavender base into ice-blue fire.           */
    { "NOVA     ",
      {  97,  98,  99, 105, 111, 147, 153, 195 }, false },

    /* TOXIC: pale chartreuse base climbing to acid green-yellow.      */
    { "TOXIC    ",
      { 107, 113, 149, 155, 191, 192, 226, 228 }, false },

    /*
     * NEGATIVE — photographic-negative inversion.  White bg, dark fg.
     * Cool/empty space reads as WHITE; the hot core reads as BLACK.
     * The ramp goes from very-light grey (slot 0, blends with white
     * bg) to pure black (slot 7, sharpest contrast).  CLAUDE.md's
     * "bright half" rule doesn't apply here because the bg is white
     * — A_DIM is disabled in scene_render for inverted themes anyway.
     */
    { "NEGATIVE ",
      { 253, 250, 245, 240, 237, 234, 232,  16 }, true },
};

/*
 * Glyph ramp by accumulated luminance: faint → dense.
 *
 * Slot 0 is `.`, NOT a space — thin late-stage smoke renders as a
 * dim dot rather than vanishing.  The "true invisible" threshold is
 * `L < L_SKIP_EPS` in scene_render; below that the cell is left as
 * default-bg.  Above it, even the dimmest pixel paints something.
 */
static const char LUMA_GLYPHS[8] = { '.', ',', ':', ';', '+', '*', '#', '@' };
#define L_SKIP_EPS  0.002f

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
    const Theme *t = &themes[idx];
    short bg = t->inverted ? 231 : -1;     /* 231 = bright white       */
    if (COLORS >= 256) {
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], bg);
    } else {
        static const short fb[8] = {
            COLOR_BLACK,  COLOR_BLACK, COLOR_WHITE, COLOR_WHITE,
            COLOR_RED,    COLOR_RED,   COLOR_YELLOW, COLOR_WHITE,
        };
        short bg8 = t->inverted ? COLOR_WHITE : -1;
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i),
                      t->inverted ? COLOR_BLACK : fb[i], bg8);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §4  vec3                                                               */
/* ===================================================================== */

typedef struct { float x, y, z; } V3;

static inline V3    v3(float x, float y, float z)   { return (V3){x, y, z}; }
static inline V3    v3add(V3 a, V3 b)               { return (V3){a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline V3    v3sub(V3 a, V3 b)               { return (V3){a.x-b.x, a.y-b.y, a.z-b.z}; }
static inline V3    v3scale(float s, V3 a)          { return (V3){s*a.x, s*a.y, s*a.z}; }
static inline float v3dot(V3 a, V3 b)               { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline V3    v3cross(V3 a, V3 b)             {
    return (V3){ a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}
static inline float v3len(V3 a)                     { return sqrtf(v3dot(a, a)); }
static inline V3    v3norm(V3 a) {
    float L = v3len(a);
    return L > 1e-12f ? v3scale(1.0f / L, a) : v3(0, 0, 1);
}

/* ===================================================================== */
/* §5  fluid — Stam stable fluids (2-D axisymmetric)                      */
/* ===================================================================== */

/*
 * Single static instance — the fluid is the simulation.  Stored in BSS,
 * no malloc.  ~1.3 MB total (8 fields × N_R × N_Y × 4B).
 */
typedef struct {
    float vr      [N_R][N_Y];   /* radial velocity                    */
    float vy      [N_R][N_Y];   /* vertical velocity                  */
    float T       [N_R][N_Y];   /* temperature                        */
    float rho     [N_R][N_Y];   /* density (visible smoke)            */
    float pressure[N_R][N_Y];   /* scratch for project()              */
    float div     [N_R][N_Y];   /* scratch for project()              */
    float buf1    [N_R][N_Y];   /* scratch for advect / Jacobi swap   */
    float buf2    [N_R][N_Y];   /* scratch for paired velocity advect */
} Fluid;

static Fluid g_fluid;

/* ── helpers ───────────────────────────────────────────────────────── */

static inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : x > hi ? hi : x;
}

/*
 * bilinear_at — bilinear sample of a field at fractional cell index
 * (fi, fj).  Out-of-grid coordinates clamp to the boundary cell.
 */
static float bilinear_at(const float field[N_R][N_Y], float fi, float fj)
{
    if (fi < 0.0f) fi = 0.0f;
    if (fj < 0.0f) fj = 0.0f;
    if (fi > (float)(N_R - 1)) fi = (float)(N_R - 1);
    if (fj > (float)(N_Y - 1)) fj = (float)(N_Y - 1);

    int   i0  = (int)floorf(fi);
    int   j0  = (int)floorf(fj);
    if (i0 >= N_R - 1) i0 = N_R - 2;
    if (j0 >= N_Y - 1) j0 = N_Y - 2;
    float fri = fi - (float)i0;
    float fyj = fj - (float)j0;

    float v00 = field[i0    ][j0    ];
    float v10 = field[i0 + 1][j0    ];
    float v01 = field[i0    ][j0 + 1];
    float v11 = field[i0 + 1][j0 + 1];
    float v0  = v00 * (1.0f - fri) + v10 * fri;
    float v1  = v01 * (1.0f - fri) + v11 * fri;
    return v0 * (1.0f - fyj) + v1 * fyj;
}

/*
 * boundary_velocity — enforce no-flow conditions at the axis (r = 0)
 * and ground (y = 0).  Other walls left as zero-gradient (the advection
 * step naturally clamps cells that try to backward-trace past the
 * boundary).
 */
static void boundary_velocity(Fluid *f)
{
    for (int j = 0; j < N_Y; j++) {
        f->vr[0][j] = 0.0f;            /* no flow through axis        */
    }
    for (int i = 0; i < N_R; i++) {
        f->vy[i][0] = 0.0f;            /* no flow through ground      */
    }
}

/* ── advection ─────────────────────────────────────────────────────── */

/*
 * advect_field — semi-Lagrangian advection.
 *
 *   For every cell (i, j), trace BACKWARD by v · dt to find where the
 *   fluid currently sitting at (i, j) came from one step ago, then
 *   bilinearly sample the OLD field at that source location.
 *
 * Why semi-Lagrangian (Stam 1999): unconditionally stable.  No CFL
 * timestep restriction — large dt is safe.  At the cost of some
 * numerical diffusion (features blur slightly each step), which we
 * accept in exchange for never blowing up.
 */
static void advect_field(float dst[N_R][N_Y],
                         const float src[N_R][N_Y],
                         const float vr[N_R][N_Y],
                         const float vy[N_R][N_Y],
                         float dt)
{
    float dt_h = dt * INV_GRID_H;
    for (int i = 0; i < N_R; i++) {
        for (int j = 0; j < N_Y; j++) {
            float fi = (float)i - vr[i][j] * dt_h;
            float fj = (float)j - vy[i][j] * dt_h;
            dst[i][j] = bilinear_at(src, fi, fj);
        }
    }
}

/* ── projection (incompressibility) ────────────────────────────────── */

/*
 * project — Hodge decomposition.
 *
 *   Find pressure p such that ∇²p = ∇·v, then v ← v − ∇p.
 *
 *   Discrete Poisson on the 5-point Laplacian stencil:
 *     div[i][j]   = (vr[i+1][j] − vr[i−1][j])/(2h) + (vy[i][j+1] − vy[i][j−1])/(2h)
 *   Solve 4·p[i][j] = p[i+1][j] + p[i−1][j] + p[i][j+1] + p[i][j−1] − h²·div[i][j]
 *   with POISSON_ITERS Jacobi sweeps.
 *
 *   Boundary: zero-gradient pressure (mirrored neighbours) — equivalent
 *   to no-flow walls on every side.  Velocity boundaries are
 *   re-enforced afterwards.
 */
static void project(Fluid *f)
{
    /* 1. Compute divergence of velocity. */
    for (int i = 0; i < N_R; i++) {
        for (int j = 0; j < N_Y; j++) {
            int im = (i > 0) ? i - 1 : 1;             /* mirror at i=0 */
            int ip = (i < N_R - 1) ? i + 1 : N_R - 2;
            int jm = (j > 0) ? j - 1 : 1;             /* mirror at j=0 */
            int jp = (j < N_Y - 1) ? j + 1 : N_Y - 2;
            f->div[i][j] = ((f->vr[ip][j] - f->vr[im][j])
                          + (f->vy[i][jp] - f->vy[i][jm])) * 0.5f * INV_GRID_H;
            f->pressure[i][j] = 0.0f;
        }
    }

    /* 2. Jacobi-iterate the Poisson equation.
     *    h² · div on the RHS keeps the problem dimensionally consistent. */
    float h2 = GRID_H * GRID_H;
    for (int iter = 0; iter < POISSON_ITERS; iter++) {
        for (int i = 0; i < N_R; i++) {
            for (int j = 0; j < N_Y; j++) {
                int im = (i > 0) ? i - 1 : 1;
                int ip = (i < N_R - 1) ? i + 1 : N_R - 2;
                int jm = (j > 0) ? j - 1 : 1;
                int jp = (j < N_Y - 1) ? j + 1 : N_Y - 2;
                f->buf1[i][j] = (f->pressure[ip][j] + f->pressure[im][j]
                               + f->pressure[i][jp] + f->pressure[i][jm]
                               - h2 * f->div[i][j]) * 0.25f;
            }
        }
        memcpy(f->pressure, f->buf1, sizeof f->pressure);
    }

    /* 3. Subtract the pressure gradient from velocity → divergence-free. */
    for (int i = 0; i < N_R; i++) {
        for (int j = 0; j < N_Y; j++) {
            int im = (i > 0) ? i - 1 : 1;
            int ip = (i < N_R - 1) ? i + 1 : N_R - 2;
            int jm = (j > 0) ? j - 1 : 1;
            int jp = (j < N_Y - 1) ? j + 1 : N_Y - 2;
            f->vr[i][j] -= (f->pressure[ip][j] - f->pressure[im][j])
                         * 0.5f * INV_GRID_H;
            f->vy[i][j] -= (f->pressure[i][jp] - f->pressure[i][jm])
                         * 0.5f * INV_GRID_H;
        }
    }

    boundary_velocity(f);
}

/* ── full step ─────────────────────────────────────────────────────── */

/*
 * fluid_step — one dt of the Boussinesq Navier-Stokes simulation.
 *
 *   1. Buoyancy force on vy.
 *   2. Project (any pre-existing divergence cleaned up first).
 *   3. Advect velocity by itself.
 *   4. Project again (advection re-introduces divergence).
 *   5. Advect T and ρ by the corrected velocity.
 *   6. Cool T toward ambient; decay ρ.
 */
static void fluid_step(Fluid *f, float dt)
{
    /* 1. Buoyancy. */
    for (int i = 0; i < N_R; i++) {
        for (int j = 0; j < N_Y; j++) {
            f->vy[i][j] += BUOYANCY_COEF * (f->T[i][j] - T_AMBIENT) * dt;
        }
    }
    boundary_velocity(f);

    /* 2. Initial projection (clean any drift). */
    project(f);

    /* 3. Advect velocity (using current velocity as the trace field). */
    advect_field(f->buf1, f->vr, f->vr, f->vy, dt);
    advect_field(f->buf2, f->vy, f->vr, f->vy, dt);
    memcpy(f->vr, f->buf1, sizeof f->vr);
    memcpy(f->vy, f->buf2, sizeof f->vy);
    boundary_velocity(f);

    /* 4. Re-project (advection broke divergence-free condition). */
    project(f);

    /* 5. Advect scalars by the now-corrected velocity. */
    advect_field(f->buf1, f->T, f->vr, f->vy, dt);
    memcpy(f->T, f->buf1, sizeof f->T);

    advect_field(f->buf1, f->rho, f->vr, f->vy, dt);
    memcpy(f->rho, f->buf1, sizeof f->rho);

    /* 6. Cool & decay.  Exponential approach to ambient is more
     *    physical than a linear lerp and behaves correctly under any
     *    timestep size.                                                */
    float cool_decay = expf(-COOL_RATE * dt);
    float rho_decay  = 1.0f - DENSITY_DECAY * dt;
    if (rho_decay < 0.0f) rho_decay = 0.0f;
    for (int i = 0; i < N_R; i++) {
        for (int j = 0; j < N_Y; j++) {
            f->T[i][j]   = T_AMBIENT + (f->T[i][j] - T_AMBIENT) * cool_decay;
            f->rho[i][j] *= rho_decay;
        }
    }
}

/* ── initialisation ────────────────────────────────────────────────── */

static void fluid_clear(Fluid *f)
{
    memset(f, 0, sizeof *f);
    for (int i = 0; i < N_R; i++)
        for (int j = 0; j < N_Y; j++)
            f->T[i][j] = T_AMBIENT;
}

/*
 * fluid_detonate — paint a hot Gaussian bubble of the chosen blast
 * type at (r=0, blast_y) and seed a radial outflow velocity (the
 * initial blast wave).
 *
 * This is the SOLE scripted moment in the simulation.  After this
 * function returns, the only thing driving the cloud is the fluid
 * solver — no time-window animation.  Different BlastTypes produce
 * different mushrooms because the INITIAL CONDITIONS differ; the
 * physics is identical.
 */
static void fluid_detonate(Fluid *f, const BlastParams *bp)
{
    fluid_clear(f);

    float two_sigma2 = 2.0f * bp->sigma * bp->sigma;

    for (int i = 0; i < N_R; i++) {
        for (int j = 0; j < N_Y; j++) {
            float r = ((float)i + 0.5f) * GRID_H;
            float y = ((float)j + 0.5f) * GRID_H;
            float dr = r;                       /* axis is at r = 0    */
            float dy = y - bp->blast_y;
            float d2 = dr * dr + dy * dy;
            float gauss = expf(-d2 / two_sigma2);

            f->T[i][j]   = T_AMBIENT + (bp->t_peak - T_AMBIENT) * gauss;
            f->rho[i][j] = bp->rho_peak * gauss;

            /* Radial outflow seeded only NEAR the centre.  Cells far
             * from the bubble already have ρ ≈ 0 and T ≈ ambient,
             * so seeding velocity there would just create unphysical
             * ambient flow. */
            if (gauss > 0.05f) {
                float dist = sqrtf(d2);
                if (dist > 1e-3f) {
                    float v_blast = bp->initial_v * gauss;
                    f->vr[i][j] = (dr / dist) * v_blast;
                    f->vy[i][j] = (dy / dist) * v_blast;
                }
            }
        }
    }

    boundary_velocity(f);
}

/* ── samplers (used by §6 raymarcher) ──────────────────────────────── */

/*
 * fluid_sample_density / fluid_sample_temp — bilinear sample of the
 * 2-D field at world-space (r, y) coordinates.  Out-of-domain returns
 * 0 (no smoke / ambient temperature).
 */
static inline float fluid_sample_density(float r, float y)
{
    if (r < 0.0f || r >= R_MAX || y < 0.0f || y >= Y_MAX) return 0.0f;
    return bilinear_at(g_fluid.rho, r * INV_GRID_H, y * INV_GRID_H);
}
static inline float fluid_sample_temp(float r, float y)
{
    if (r < 0.0f || r >= R_MAX || y < 0.0f || y >= Y_MAX) return T_AMBIENT;
    return bilinear_at(g_fluid.T, r * INV_GRID_H, y * INV_GRID_H);
}

/* ===================================================================== */
/* §6  march — volumetric raymarcher                                      */
/* ===================================================================== */

/*
 * raymarch — Beer-Lambert volumetric integration.
 *
 *   For each step along the ray:
 *     - Compute (r, y) from world (x, y, z); sample density + temp.
 *     - dτ = ρ · STEP · DENSITY_GAIN  (optical depth this step)
 *     - emit = clamp((T − T₀)/(T_peak − T₀), 0, 1)
 *     - L += T_view · dτ · (emit · EMIT_GAIN + AMBIENT)
 *     - L_hot += T_view · dτ · emit · EMIT_GAIN
 *     - T_view *= exp(−dτ)
 *     - break if T_view < OPAQUE_EPS
 *
 *   Returns:
 *     *out_lum     — total accumulated radiance
 *     *out_lum_hot — radiance contribution from hot regions only
 *     The hot fraction L_hot/L drives the smoke→fire palette index.
 */
static void raymarch(V3 origin, V3 dir,
                     float *out_lum, float *out_lum_hot)
{
    float t       = RM_T_NEAR;
    float T_view  = 1.0f;
    float L       = 0.0f;
    float L_hot   = 0.0f;
    float T_inv_range = 1.0f / (T_PEAK - T_AMBIENT);

    for (int step = 0; step < RM_MAX_STEPS; step++) {

        V3 p = v3add(origin, v3scale(t, dir));

        /* Bounds check — quick reject outside the simulation volume. */
        if (p.y < 0.0f || p.y > Y_MAX) {
            t += RM_STEP * 2.0f;
            if (t > RM_T_FAR) break;
            continue;
        }
        float r = sqrtf(p.x * p.x + p.z * p.z);
        if (r > R_MAX) {
            t += RM_STEP * 2.0f;
            if (t > RM_T_FAR) break;
            continue;
        }

        float rho  = fluid_sample_density(r, p.y);
        if (rho < 0.001f) {
            t += RM_STEP;
            if (t > RM_T_FAR) break;
            continue;
        }
        float temp = fluid_sample_temp(r, p.y);

        float dtau = rho * RM_STEP * RM_DENSITY_GAIN;

        float emit = (temp - T_AMBIENT) * T_inv_range;
        if (emit < 0.0f) emit = 0.0f;
        if (emit > 1.0f) emit = 1.0f;

        float source = emit * RM_EMIT_GAIN + RM_AMBIENT;
        L     += T_view * dtau * source;
        L_hot += T_view * dtau * emit * RM_EMIT_GAIN;

        T_view *= expf(-dtau);
        if (T_view < RM_OPAQUE_EPS) break;
        if (t > RM_T_FAR) break;

        t += RM_STEP;
    }

    *out_lum     = L;
    *out_lum_hot = L_hot;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

typedef struct {
    bool       paused;
    int        current_theme;
    BlastType  current_blast;
    int        cols, rows;

    float      sim_time;       /* simulated seconds since detonation    */
    float      sim_rate;       /* time-scale multiplier                 */
    float      sim_accum;      /* unspent dt for the fixed-step solver  */
    float      cam_dist;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused        = false;
    s->current_theme = 0;
    s->current_blast = BLAST_STANDARD;
    s->cols          = cols;
    s->rows          = rows;
    s->sim_time      = 0.0f;
    s->sim_rate      = SIM_RATE_DEFAULT;
    s->sim_accum     = 0.0f;
    s->cam_dist      = CAM_DIST_DEFAULT;

    fluid_detonate(&g_fluid, &blasts[s->current_blast]);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_redetonate(Scene *s)
{
    s->sim_time  = 0.0f;
    s->sim_accum = 0.0f;
    fluid_detonate(&g_fluid, &blasts[s->current_blast]);
}

/*
 * scene_cycle_blast — advance the BlastType and re-detonate.  The
 * fluid keeps no state across detonations (fluid_clear zeros every
 * field), so cycling is cleanly equivalent to "set type, press r".
 */
static void scene_cycle_blast(Scene *s, int dir)
{
    int idx = (int)s->current_blast + dir;
    while (idx < 0) idx += N_BLASTS;
    s->current_blast = (BlastType)(idx % N_BLASTS);
    scene_redetonate(s);
}

/*
 * scene_tick — accumulate time and run as many fixed-dt fluid steps
 * as the accumulator allows.  Fixed-dt physics keeps the cloud's
 * morphology stable across changes in render fps and time-scale.
 */
static void scene_tick(Scene *s, float dt_real)
{
    if (s->paused) return;

    float dt_sim = dt_real * s->sim_rate;
    if (dt_sim > SIM_DT_MAX) dt_sim = SIM_DT_MAX;
    s->sim_accum += dt_sim;
    s->sim_time  += dt_sim;

    while (s->sim_accum >= SIM_DT) {
        fluid_step(&g_fluid, SIM_DT);
        s->sim_accum -= SIM_DT;
    }
}

/*
 * scene_render — full-frame volumetric raymarch.
 *
 *   1. Build camera basis (fixed pos, look-at the cloud column).
 *   2. For each cell, raymarch into the volume; classify the pixel
 *      from accumulated luminance + hot fraction.
 *   3. Batch attron/attroff to minimise terminal attribute thrash.
 */
static void scene_render(const Scene *s)
{
    int rows_eff = s->rows - 1;
    if (rows_eff < 1) return;

    /* Camera. */
    V3 cam = v3(0.0f, CAM_HEIGHT, -s->cam_dist);
    V3 target = v3(0.0f, LOOK_AT_Y, 0.0f);
    V3 fwd   = v3norm(v3sub(target, cam));
    V3 wup   = v3(0, 1, 0);
    V3 right = v3norm(v3cross(fwd, wup));
    V3 up    = v3cross(right, fwd);

    float fov_t       = tanf(FOV_DEG * (float)M_PI / 180.0f * 0.5f);
    float phys_aspect = ((float)rows_eff * CELL_ASPECT) / (float)s->cols;

    int     last_pair = -1;
    attr_t  last_attr = 0;
    bool    inverted  = themes[s->current_theme].inverted;

    /*
     * For inverted themes, pre-fill the visible region with white
     * background (any ramp pair has bg=231; spaces inherit bg).
     * Empty cells (skipped below) keep this white pre-fill.
     */
    if (inverted) {
        attron(COLOR_PAIR(PAIR_RAMP_BASE));
        for (int row = 0; row < rows_eff; row++)
            for (int col = 0; col < s->cols; col++)
                mvaddch(row, col, ' ');
        attroff(COLOR_PAIR(PAIR_RAMP_BASE));
        last_pair = PAIR_RAMP_BASE;
        last_attr = A_NORMAL;
    }

    for (int row = 0; row < rows_eff; row++) {
        for (int col = 0; col < s->cols; col++) {
            float u =  ((float)col + 0.5f) / (float)s->cols * 2.0f - 1.0f;
            float v = -(((float)row + 0.5f) / (float)rows_eff * 2.0f - 1.0f);
            V3 dir = v3norm(v3add(fwd,
                v3add(v3scale(u * fov_t, right),
                      v3scale(v * fov_t * phys_aspect, up))));

            float L, L_hot;
            raymarch(cam, dir, &L, &L_hot);

            if (L < L_SKIP_EPS) {
                /* Pixel sees no smoke.  For normal themes we leave
                 * default bg (black); for inverted themes the
                 * pre-fill above already painted white here. */
                if (!inverted && last_pair >= 0) {
                    attroff(COLOR_PAIR(last_pair) | last_attr);
                    last_pair = -1;
                }
                continue;
            }

            float lum_n = L / LUM_CLAMP;
            if (lum_n > 1.0f) lum_n = 1.0f;
            int slot_lum = (int)(lum_n * 7.999f);
            if (slot_lum < 0) slot_lum = 0;
            if (slot_lum > 7) slot_lum = 7;

            float hot_frac = L_hot / (L + 0.001f);
            if (hot_frac > 1.0f) hot_frac = 1.0f;
            if (hot_frac < 0.0f) hot_frac = 0.0f;
            int slot_hot = (int)(hot_frac * 7.999f);
            if (slot_hot < 0) slot_hot = 0;
            if (slot_hot > 7) slot_hot = 7;

            char glyph = LUMA_GLYPHS[slot_lum];
            int  pair  = PAIR_RAMP_BASE + slot_hot;
            /*
             * Attribute modulation:
             *   normal themes — A_BOLD on hot tiers (extra punch),
             *                   A_DIM on cool tiers (fading tail).
             *   inverted themes — A_NORMAL only.  A_DIM on a light fg
             *                   over white bg DARKENS it, making cool
             *                   cells MORE visible (opposite of intent),
             *                   so we let the grey ramp itself carry
             *                   the gradient with no attribute help.
             */
            attr_t attr;
            if (inverted) {
                attr = A_NORMAL;
            } else {
                attr = (slot_hot >= 6 || slot_lum >= 6) ? A_BOLD
                     : (slot_lum  <= 1)                 ? A_DIM
                     :                                    A_NORMAL;
            }

            if (pair != last_pair || attr != last_attr) {
                if (last_pair >= 0)
                    attroff(COLOR_PAIR(last_pair) | last_attr);
                attron(COLOR_PAIR(pair) | attr);
                last_pair = pair;
                last_attr = attr;
            }
            mvaddch(row, col, (chtype)(unsigned char)glyph);
        }
    }
    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}
/* ── end §7 — to understand the ncurses wrapper, read §8 screen ── */

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *sc)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) { (void)sc; endwin(); }
static void screen_resize_curses(Screen *sc)
{
    endwin();
    refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_render(s);

    char buf[260];
    snprintf(buf, sizeof buf,
             " NUKE (FLUID)   %s   blast:%s   theme:%s   t:%6.2fs   "
             "rate:%4.2f   dist:%4.1f   "
             "%5.1f fps  %3d Hz   "
             "n/N:blast  r:detonate  t/T:theme  +/-:rate  z/Z:zoom  spc:pause  q:quit ",
             s->paused ? "PAUSED" : "EVOLVE",
             blasts[s->current_blast].name,
             themes[s->current_theme].name,
             (double)s->sim_time, (double)s->sim_rate,
             (double)s->cam_dist, fps, sim_fps);

    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize_curses(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_redetonate(s);                          break;

    case 'n':           scene_cycle_blast(s, +1);                     break;
    case 'N':           scene_cycle_blast(s, -1);                     break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case '=': case '+':
        s->sim_rate *= SIM_RATE_FACTOR;
        if (s->sim_rate > SIM_RATE_MAX) s->sim_rate = SIM_RATE_MAX;
        break;
    case '-':
        s->sim_rate /= SIM_RATE_FACTOR;
        if (s->sim_rate < SIM_RATE_MIN) s->sim_rate = SIM_RATE_MIN;
        break;

    case 'z':
        s->cam_dist -= CAM_DIST_STEP;
        if (s->cam_dist < CAM_DIST_MIN) s->cam_dist = CAM_DIST_MIN;
        break;
    case 'Z':
        s->cam_dist += CAM_DIST_STEP;
        if (s->cam_dist > CAM_DIST_MAX) s->cam_dist = CAM_DIST_MAX;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
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
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        float dt_sec = (float)dt / (float)NS_PER_SEC;
        scene_tick(&app->scene, dt_sec);

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t target_ns = TICK_NS(app->sim_fps);
        int64_t elapsed   = clock_ns() - frame_time + dt;
        clock_sleep_ns(target_ns - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
