/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * duffing_oscillator.c
 *   — A DRIVEN nonlinear oscillator: the textbook route from regular
 *     motion to chaos via period doubling.  The "double-well" Duffing
 *     equation has two stable rest states; a periodic external drive
 *     of amplitude A pushes a particle between them.  Below a
 *     critical A the motion is regular; above it the trajectory in
 *     phase space (x, ẋ) becomes a strange attractor.
 *
 * DEMO: Integrate the ODE
 *         ẍ + γ·ẋ − x + x³ = A · cos(ω·t)
 *       with RK4 and draw the trajectory in (x, ẋ) phase space as a
 *       fading ring-buffer trail.  30 presets in 7 groups n/p-cyclable —
 *       single-well oscillation, period-doubling cascade, single-well
 *       chaos, cross-well strange attractor, periodic windows, low-
 *       damping high-energy, off-resonance.  Default opens on
 *       PRESET_SLEW (γ=0.25, ω=1.0, A=0.50 — Holmes' 1979 canon).
 *
 * Study alongside:
 *   ./double_pendulum.c   — un-driven (autonomous) mechanical chaos.
 *   ./bifurcation.c       — discrete-map version of the same
 *                           period-doubling cascade.
 *   ./poincare_section.c  — same trick (stroboscopic sampling) makes
 *                           the Duffing attractor into a 2-D dot
 *                           cloud; here we draw the continuous trail.
 *
 * Section map:
 *   §1 config   — constants, 30-preset table, themes, sim budget
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD pairs + 10 themes (PAIR_TRAIL_BASE+0..3)
 *   §5 physics  — DuffingSystem + DuffingState sub-structs,
 *                 derivative, RK4 with named stage helpers
 *   §6 trail    — (x, v) phase ring buffer + age-band helpers
 *   §7 state    — PresetState + PaletteState typed wrappers
 *   §8 scene    — Scene composes duffing + trail + preset + palette
 *   §9 screen   — PhaseSpaceViewport, axes / trail / live painters
 *   §10 app     — signals, resize, key dispatch table, FrameClock,
 *                 main pseudocode driver + named loop helpers
 *
 * Keys:  q/ESC quit | space pause | r reset | n/p preset |
 *        t/T theme | ]/[ Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/chaos/duffing_oscillator.c \
 *       -o duffing_oscillator -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : 2-D driven, NON-AUTONOMOUS ODE — state (x, v, t).
 *                 RK4 at fixed substep DUFFING_DT = 0.01 s, with
 *                 INT_STEPS_PER_TICK = 8 substeps per render tick.
 *                 Each substep also pushes (x, v) into the phase-
 *                 portrait trail.  See §5 duffing_rk4 for the
 *                 named-stage RK4 scheme.
 *
 * Data-struct   : Duffing splits into TWO named sub-structs —
 *                 DuffingSystem (γ, ω, A — the invariant parameters)
 *                 and DuffingState (x, v, t — the dynamic ODE coords).
 *                 Trail is a TRAIL_MAX-sample ring buffer of recent
 *                 phase-space (x, v) values in PHYSICAL units, with
 *                 TRAIL_BAND_COUNT = 4 colour-fade bands oldest →
 *                 newest assigned by trail_band_for_age().
 *
 * Rendering     : ASCII-only.  Phase-space viewport (gx0, gy0, w, h)
 *                 maps (x, v) ∈ [X_MIN, X_MAX] × [V_MIN, V_MAX] to a
 *                 centred terminal rectangle.  Trail glyph '.' fades
 *                 through the four colour bands; the LIVE point gets
 *                 an '@' in PAIR_LIVE; faint '|' / '-' axes at x=0 /
 *                 v=0 in PAIR_AXIS give the reader a reference frame.
 *
 * References    : THE EQUATION  (where it comes from)
 *                 [1] Duffing, G. (1918) — "Erzwungene Schwingungen
 *                     bei veränderlicher Eigenfrequenz und ihre
 *                     technische Bedeutung", Vieweg.  The original
 *                     paper introducing the cubic-stiffness oscillator.
 *                 [2] Holmes, P. J. (1979) — "A nonlinear oscillator
 *                     with a strange attractor", Phil. Trans. R. Soc.
 *                     Lond. A 292, pp. 419-448.  PROOF that the driven
 *                     Duffing equation exhibits chaos; establishes the
 *                     canonical γ=0.25, ω=1.0, A=0.50 regime that
 *                     PRESET_SLEW (and every textbook since) uses.
 *                 [3] Ueda, Y. (1980) — "Steady motions exhibited by
 *                     Duffing's equation: A picture book of regular
 *                     and chaotic motions", in "New Approaches to
 *                     Nonlinear Problems in Dynamics" (Holmes ed.).
 *                     The phase-portrait ATLAS — the visual reference
 *                     for what each region of (γ, ω, A) space looks
 *                     like, directly relevant to the §1 preset zoo.
 *
 *                 NONLINEAR DYNAMICS  (the broader context)
 *                 [4] Strogatz, S. H. (2015) — "Nonlinear Dynamics
 *                     and Chaos" (2nd ed.), Westview Press, §9-10.
 *                     Pedagogical chapter on Lorenz/Duffing strange
 *                     attractors with the route via period doubling.
 *                 [5] Guckenheimer, J., Holmes, P. J. (1983) —
 *                     "Nonlinear Oscillations, Dynamical Systems, and
 *                     Bifurcations of Vector Fields", Springer §2.2
 *                     and §4.5.  Period-doubling cascade (Group B) and
 *                     periodic windows inside chaos (Group E).
 *                 [6] Thompson, J. M. T., Stewart, H. B. (2002) —
 *                     "Nonlinear Dynamics and Chaos" (2nd ed.), Wiley,
 *                     Ch. 11.  The cross-well strange attractor
 *                     (Group D) and the role of γ in setting the
 *                     attractor's extent (Group F).
 *                 [7] Feigenbaum, M. J. (1978) — "Quantitative
 *                     universality for a class of nonlinear
 *                     transformations", J. Stat. Phys. 19(1).
 *                     The δ = 4.6692... universal ratio that governs
 *                     the Group-B period-doubling cascade.
 *
 *                 NUMERICAL INTEGRATION  (RK4 + fixed-timestep loop)
 *                 [8] Press, W. H., Teukolsky, S. A., Vetterling, W. T.,
 *                     Flannery, B. P. (2007) — "Numerical Recipes"
 *                     (3rd ed.), Cambridge, Ch. 17.1.  RK4 derivation,
 *                     stability, and the Butcher tableau in §5.
 *                 [9] Fiedler, G. (2004) — "Fix Your Timestep!",
 *                     gafferongames.com.  The accumulator pattern
 *                     used by app_drain_fixed_timestep() — decouples
 *                     render rate (variable) from physics rate
 *                     (constant) so the chaos stays deterministic.
 *
 *                 RENDERING & TERMINAL I/O  (how the math reaches the screen)
 *                 [10] Gookin, D. (2007) — "Programmer's Guide to
 *                     NCurses", Wiley.  Double-buffered rendering
 *                     model (wnoutrefresh + doupdate), color-pair
 *                     theory, and SIGWINCH handling used in §3, §9-10.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Picture a ball rolling in a double-well potential V(x) = −x²/2 +
 * x⁴/4 (two valleys separated by a hump at x=0).  Add gentle
 * friction γ·ẋ that drains energy, plus an oscillating push
 * A·cos(ωt) that pumps energy in periodically.  Whether the ball
 * stays in one well or kicks back and forth between them depends
 * on how strong the drive is.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Energy conservation is broken (γ takes away, A puts in).  At
 * STEADY STATE the orbit lives on an ATTRACTOR — a finite shape in
 * (x, v) space that all initial conditions converge onto:
 *
 *   A small  → 1-loop limit cycle inside one well (period of the drive)
 *   A medium → 2-loop limit cycle (period DOUBLED — a bifurcation)
 *   A larger → 4-, 8-, 16-... loops (period-doubling cascade)
 *   A above critical → STRANGE ATTRACTOR (chaos)
 *
 * KEY FORMULAS
 * ────────────
 *   ẍ + γ·ẋ − x + x³ = A · cos(ω · t)
 *
 *   Rewrite as 2 first-order ODEs:
 *     ẋ = v
 *     v̇ = −γ·v + x − x³ + A · cos(ω · t)
 *
 *   Holmes 1979 chaos regime: γ = 0.25, ω = 1.0, A ≳ 0.4.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Group-A preset (e.g. HUM at A=0.20): trail collapses to a
 *    closed loop in one well — the attractor IS the loop.  After
 *    transient, the orbit retraces itself indefinitely.
 *  • Group-B preset (e.g. WALTZ at A=0.28): one-loop becomes
 *    two-loop — the first period doubling.  Each subsequent Group-B
 *    preset doubles again (CADENCE → 4, RIPPLE → 8 ...).
 *  • Group-C preset (SLEW at A=0.50, Holmes' canon): trail fills
 *    a thin chaotic band in one well; no two laps retrace.
 *  • Group-D preset (BLOOM at A=1.00): trail fills both wells,
 *    crossing the saddle x=0 — the iconic Duffing strange attractor.
 *  • Reset (r) with same preset → same trajectory (deterministic).
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    MAP_W_MAX        = 200, MAP_H_MAX = 56,
    SIM_FPS_MIN      =  10, SIM_FPS_DEFAULT = 60, SIM_FPS_MAX = 240, SIM_FPS_STEP = 10,
    HUD_COLS         =  80, FPS_UPDATE_MS = 500,
    PAIR_HUD         =   1, PAIR_HINT = 2,
    PAIR_TRAIL_BASE  =   3,    /* 4 trail-age bands */
    PAIR_AXIS        =   7, PAIR_LIVE = 8,
};

#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

#define NS_PER_SEC               1000000000LL
#define NS_PER_MS                   1000000LL
#define TICK_NS(f)               (NS_PER_SEC / (f))
#define RENDER_FPS_TARGET        60
#define RENDER_FRAME_BUDGET_NS   (NS_PER_SEC / RENDER_FPS_TARGET)
#define SIM_MAX_FRAME_DT_MS      100

#define DUFFING_DT                0.01f
#define INT_STEPS_PER_TICK          8

#define TRAIL_MAX                3000
#define TRAIL_BAND_COUNT            4

/* Phase-space viewport in physical (x, v) units */
#define X_MIN                    -2.0f
#define X_MAX                     2.0f
#define V_MIN                    -2.0f
#define V_MAX                     2.0f

/*
 * Preset — index into the 30-entry presets[] table.  Ordered SIMPLE →
 * COMPLEX so n/p cycling tours the full behavioural zoo of the Holmes
 * double-well Duffing oscillator:
 *
 *   Group A  small-amplitude single-well oscillation     (6 presets)
 *   Group B  period-doubling cascade (route to chaos)    (6 presets)
 *   Group C  single-well chaos                           (4 presets)
 *   Group D  cross-well strange attractor                (5 presets)
 *   Group E  periodic windows inside chaos               (3 presets)
 *   Group F  low-damping, high-energy                    (4 presets)
 *   Group G  off-resonance frequencies                   (2 presets)
 *
 * REFERENCES (numbered in file-header References block)
 *   [2] Holmes (1979)              — canonical γ=0.25, ω=1.0, A=0.50
 *                                    chaotic regime → PRESET_SLEW.
 *   [3] Ueda (1980)                — phase-portrait atlas; visual
 *                                    grounding for choosing distinct
 *                                    Group A-E examples.
 *   [5] Guckenheimer & Holmes §2.2 — period-doubling cascade (Group B)
 *                                    and the periodic windows inside
 *                                    chaos (Group E).
 *   [6] Thompson & Stewart Ch. 11  — cross-well attractor (Group D)
 *                                    and the role of γ in setting the
 *                                    attractor's extent (Group F).
 *   [7] Feigenbaum (1978)          — universality of the Group B
 *                                    cascade (δ = 4.6692...).
 */
typedef enum {
    /* Group A — small-amplitude single-well oscillation */
    PRESET_WHISPER = 0,    /* A=0.05, x₀=+1     tiny drive at right minimum */
    PRESET_HUSH,           /* A=0.05, x₀=-1     mirror in the left well     */
    PRESET_BREATH,         /* A=0.10            barely-perceptible osc      */
    PRESET_PURR,           /* A=0.15            small period-1              */
    PRESET_HUM,            /* A=0.20            steady period-1              */
    PRESET_CHIRP,          /* A=0.20, x₀=saddle small osc from unstable top */

    /* Group B — period-doubling cascade (Feigenbaum route to chaos) */
    PRESET_WALTZ,          /* A=0.28            period-2                   */
    PRESET_RONDO,          /* A=0.32            period-2 fuller            */
    PRESET_CADENCE,        /* A=0.36            period-4                   */
    PRESET_RIPPLE,         /* A=0.40            period-4 / period-8 onset  */
    PRESET_WEAVE,          /* A=0.45            band-merging               */
    PRESET_KNIT,           /* A=0.48            chaos onset                */

    /* Group C — single-well chaos (well-bounded strange attractor) */
    PRESET_SLEW,           /* A=0.50            Holmes' canonical chaos    */
    PRESET_JITTER,         /* A=0.55                                       */
    PRESET_STIR,           /* A=0.58                                       */
    PRESET_SHUFFLE,        /* A=0.60                                       */

    /* Group D — cross-well strange attractor (escape between both wells) */
    PRESET_CROSS,          /* A=0.70, x₀=0      from saddle                */
    PRESET_STRADDLE,       /* A=0.80            wide excursions            */
    PRESET_SWING,          /* A=0.85            both wells, full           */
    PRESET_BLOOM,          /* A=1.00            full Holmes attractor      */
    PRESET_RIOT,           /* A=1.10            high-energy chaos          */

    /* Group E — periodic windows hidden inside chaos */
    PRESET_RESET,          /* A=0.65            period-3 window            */
    PRESET_MIRROR,         /* A=0.72, x₀=-1     period-5 in left well      */
    PRESET_RING,           /* A=0.95            period-3 in cross-well     */

    /* Group F — low damping, high-energy (large attractor, slow decay) */
    PRESET_SLACK,          /* γ=0.10, A=0.40    light damping              */
    PRESET_LOOSE,          /* γ=0.10, A=0.50    extended attractor         */
    PRESET_GLIDE,          /* γ=0.08, A=0.45                               */
    PRESET_DRIFT,          /* γ=0.05, A=0.40    near-Hamiltonian           */

    /* Group G — off-resonance drive frequencies */
    PRESET_SLOW,           /* ω=0.7, A=0.40     sub-resonance              */
    PRESET_RAPID,          /* ω=1.4, A=0.40     super-resonance            */

    N_PRESETS,
} Preset;

/*
 * DuffingPreset — one row of the 30-entry presets[] table: a named
 * 5-tuple (γ, ω, A, x₀, v₀) defining ONE experiment.
 *
 * INTENT
 *   The Duffing equation has TWO independent things you can vary:
 *
 *     1. SYSTEM parameters (γ, ω, A) — what the oscillator IS:
 *        which damping, which drive.  Period-doubling, the
 *        appearance of chaos, the cross-well transition — all
 *        depend on these.  Groups A-E sweep A; F sweeps γ; G ω.
 *
 *     2. INITIAL CONDITIONS (x₀, v₀) — where the orbit STARTS
 *        in phase space.  For a chaotic attractor (most of the
 *        zoo) the long-term behaviour is INDEPENDENT of (x₀, v₀):
 *        every initial point converges onto the same attractor.
 *        But the TRANSIENT — what the first 10-20 s of trail
 *        look like — depends strongly on the starting point.
 *        For a multi-attractor regime (Groups A-B) the initial
 *        condition picks WHICH attractor you land on.
 *
 *   Bundling these five numbers as ONE struct + naming each row
 *   makes the entire behavioural zoo browsable from the table itself.
 *
 * CONTEXT
 *   Lives only in the const-table presets[] (immediately below).
 *   Looked up by §7's PresetState wrapper via preset_state_active().
 *   Consumed in two halves by §5:
 *     • duffing_system_init_from_preset → (γ, ω, A) → DuffingSystem
 *     • duffing_state_init_from_preset  → (x₀, v₀)  → DuffingState
 *   The two-half design lets a future "perturb x₀ but keep system"
 *   feature touch only one initialiser.  Never mutated at runtime.
 *
 * MEMBER LOGIC
 *   name  : 8-char label shown in HUD ([row %-8s] format).  Space-
 *           padded for column alignment.  Picked to evoke each
 *           preset's visual class (WHISPER for tiny, RIOT for
 *           high-energy chaos, RAPID for super-resonance).
 *   gamma : damping coefficient γ.  Holmes-canon γ = 0.25 across
 *           Groups A-E; Group F drops to 0.05-0.10 for near-
 *           Hamiltonian extended attractors.  Always ≥ 0 — physical
 *           damping cannot be negative.
 *   omega : drive angular frequency ω (rad / s).  Holmes-canon
 *           ω = 1.0 (resonant) across Groups A-F; Group G uses
 *           0.7 / 1.4 for sub-/super-resonance.  Period of the
 *           drive: T = 2π / ω.
 *   A     : drive amplitude.  THE chaos knob — sweeps from 0.05
 *           (tiny oscillation) through 0.50 (Holmes' canon) to
 *           1.10 (riot).  Above ~A ≳ 0.4 with γ = 0.25 the
 *           homoclinic bifurcation opens the door to chaos.
 *   x0    : initial position.  Typical values:
 *             +1.0 → right well minimum
 *             -1.0 → left well minimum
 *              0.0 → unstable saddle (the hump between wells)
 *   v0    : initial velocity dx/dt.  Mostly 0.0 (release from
 *           rest); a few presets use a small kick to inject
 *           kinetic energy at t = 0.
 *
 * REFERENCES (numbered in file-header References block)
 *   [2] Holmes (1979)             — canon (γ=0.25, ω=1.0, A=0.50)
 *                                    that PRESET_SLEW reproduces.
 *   [3] Ueda (1980)               — phase-portrait atlas keying
 *                                    each (γ, ω, A) region to its
 *                                    visual signature; used to
 *                                    pick distinct Group examples.
 *   [5] Guckenheimer & Holmes §4.5 — period-doubling cascade
 *                                     parameter values that
 *                                     Group B reproduces.
 */
typedef struct {
    const char *name;
    float       gamma;
    float       omega;
    float       A;
    float       x0, v0;
} DuffingPreset;

/* 30 named initial conditions for the Holmes form
 *   ẍ + γẋ - x + x³ = A · cos(ωt)
 * ordered simple → complex.  Holmes 1979 standard reference parameters
 * are γ = 0.25, ω = 1.0; most of the zoo sweeps the drive amplitude A
 * to walk the period-doubling cascade into chaos (Groups A-E).  Groups
 * F-G perturb γ and ω to expose attractor topology that the A-sweep
 * alone doesn't reach. */
static const DuffingPreset presets[N_PRESETS] = {
    /* Group A — small-amplitude single-well oscillation ─────────────── */
    { "WHISPER ", 0.25f, 1.0f, 0.05f,  1.0f,  0.0f },
    { "HUSH    ", 0.25f, 1.0f, 0.05f, -1.0f,  0.0f },
    { "BREATH  ", 0.25f, 1.0f, 0.10f,  1.0f,  0.0f },
    { "PURR    ", 0.25f, 1.0f, 0.15f,  1.0f,  0.0f },
    { "HUM     ", 0.25f, 1.0f, 0.20f,  1.0f,  0.0f },
    { "CHIRP   ", 0.25f, 1.0f, 0.20f,  0.0f,  0.20f },

    /* Group B — period-doubling cascade ─────────────────────────────── */
    { "WALTZ   ", 0.25f, 1.0f, 0.28f,  1.0f,  0.0f },
    { "RONDO   ", 0.25f, 1.0f, 0.32f,  1.0f,  0.0f },
    { "CADENCE ", 0.25f, 1.0f, 0.36f,  1.0f,  0.0f },
    { "RIPPLE  ", 0.25f, 1.0f, 0.40f,  1.0f,  0.0f },
    { "WEAVE   ", 0.25f, 1.0f, 0.45f,  1.0f,  0.0f },
    { "KNIT    ", 0.25f, 1.0f, 0.48f,  1.0f,  0.0f },

    /* Group C — single-well chaos ───────────────────────────────────── */
    { "SLEW    ", 0.25f, 1.0f, 0.50f,  1.0f,  0.0f },   /* Holmes canon */
    { "JITTER  ", 0.25f, 1.0f, 0.55f,  1.0f,  0.0f },
    { "STIR    ", 0.25f, 1.0f, 0.58f,  1.0f,  0.0f },
    { "SHUFFLE ", 0.25f, 1.0f, 0.60f,  1.0f,  0.0f },

    /* Group D — cross-well strange attractor ────────────────────────── */
    { "CROSS   ", 0.25f, 1.0f, 0.70f,  0.0f,  0.0f },
    { "STRADDLE", 0.25f, 1.0f, 0.80f,  0.0f,  0.0f },
    { "SWING   ", 0.25f, 1.0f, 0.85f,  0.0f,  0.0f },
    { "BLOOM   ", 0.25f, 1.0f, 1.00f,  0.0f,  0.0f },
    { "RIOT    ", 0.25f, 1.0f, 1.10f,  0.0f,  0.0f },

    /* Group E — periodic windows inside chaos ───────────────────────── */
    { "RESET   ", 0.25f, 1.0f, 0.65f,  1.0f,  0.0f },
    { "MIRROR  ", 0.25f, 1.0f, 0.72f, -1.0f,  0.0f },
    { "RING    ", 0.25f, 1.0f, 0.95f,  1.0f,  0.0f },

    /* Group F — low damping, high-energy ────────────────────────────── */
    { "SLACK   ", 0.10f, 1.0f, 0.40f,  1.0f,  0.0f },
    { "LOOSE   ", 0.10f, 1.0f, 0.50f,  0.0f,  0.0f },
    { "GLIDE   ", 0.08f, 1.0f, 0.45f,  0.0f,  0.0f },
    { "DRIFT   ", 0.05f, 1.0f, 0.40f,  0.0f,  0.0f },

    /* Group G — off-resonance frequencies ───────────────────────────── */
    { "SLOW    ", 0.25f, 0.7f, 0.40f,  1.0f,  0.0f },
    { "RAPID   ", 0.25f, 1.4f, 0.40f,  1.0f,  0.0f },
};

/*
 * Theme — colour assignment for every rendered element in one palette.
 *
 * INTENT
 *   Decouples "what gets drawn" (the §9 painters) from "what colour it
 *   should be" so a single cycle key (t/T) swaps the entire visual
 *   identity without changing one line of physics or rendering logic.
 *   The Duffing demo has four render roles (trail-age × N, axis, live
 *   marker) — each gets one Theme field, each Theme[] row is one
 *   coherent palette assignment for all four.
 *
 * CONTEXT
 *   Lives only in the const-table themes[] of N_THEMES entries.
 *   Selected by §7's PaletteState wrapper via palette_state_active().
 *   Pushed into ncurses by §3 theme_apply() which calls init_pair()
 *   on each (PAIR_*, field, -1) triple.  Never mutated at runtime —
 *   t/T just changes which row gets fed to init_pair().
 *
 * MEMBER LOGIC
 *   name           : 8-char label shown on row-1 HUD as the current
 *                    theme name.  Space-padded for column alignment.
 *   band[0..N-1]   : the TRAIL-AGE colour ramp; band[0] = oldest
 *                    visible sample (dimmest), band[N-1] = newest
 *                    (brightest).  Must be a monotonic perceptual
 *                    gradient — see §6 trail_band_for_age which maps
 *                    sample age to band index.  Every entry must sit
 *                    at xterm-256 index ≥ 24 (CLAUDE.md "Theme
 *                    Palette Brightness") so the dimmest tier stays
 *                    legible against default-black with A_BOLD.
 *   axis           : '|' and '-' x=0 / v=0 reference lines.  Picked
 *                    perceptually MUTED so the axes orient the eye
 *                    without competing with the trail.
 *   live           : '@' marker tracking the current (x, v).  The
 *                    most saturated colour in the theme — the eye
 *                    follows the moving marker against the static
 *                    attractor shape.
 *
 * REFERENCES
 *   • CLAUDE.md "Theme Palette Brightness" — project-wide rule that
 *     band[0] must sit at xterm-256 index ≥ 24 to avoid the
 *     "A_DIM = invisible" trap on dark terminals.
 *   • CLAUDE.md "HUD Standard" — PAIR_HUD/PAIR_HINT use bright
 *     yellow + cyan and are NOT per-theme; only the simulation
 *     colours in this struct cycle with t/T.
 */
typedef struct {
    const char *name;
    short       band[TRAIL_BAND_COUNT];
    short       axis;
    short       live;
} Theme;

#define N_THEMES 10
static const Theme themes[N_THEMES] = {
    { "DEFAULT",  {  75, 123, 220, 231 },  244, 196 },
    { "MATRIX",   {  77, 118, 156, 194 },  244, 226 },
    { "NOVA",     { 135, 171, 207, 219 },  244, 226 },
    { "MONO",     { 247, 250, 253, 255 },  240, 226 },
    { "OCEAN",    {  81, 117, 159, 195 },  244, 226 },
    { "FIRE",     { 208, 214, 220, 227 },  244, 231 },
    { "EARTH",    { 143, 179, 215, 222 },  244, 196 },
    { "FOREST",   { 114, 150, 157, 194 },  244, 226 },
    { "DESERT",   { 179, 215, 222, 229 },  244, 196 },
    { "ARCTIC",   { 117, 159, 195, 231 },  244, 196 },
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns) { if (ns <= 0) return;
    struct timespec req = {ns/NS_PER_SEC, ns%NS_PER_SEC}; nanosleep(&req, NULL); }

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/* PAIR_HUD_FG_256 / PAIR_HINT_FG_256 — bright yellow / bright cyan
 * indices in the xterm-256 cube.  Named so the HUD doesn't carry
 * magic 226 / 51 numerals inline. */
#define PAIR_HUD_FG_256    226
#define PAIR_HINT_FG_256    51

/* theme_apply_pairs_256color — push the 2 + N_TRAIL_BANDS colour-pair
 * assignments for a 256-colour terminal.  Each pair maps one render
 * role (axis / live / trail-band-i) to one xterm-256 index from the
 * Theme row.  All foregrounds use the terminal's default background
 * (-1) so the demo composites cleanly on dark or light. */
static void theme_apply_pairs_256color(const Theme *t)
{
    for (int i = 0; i < TRAIL_BAND_COUNT; i++)
        init_pair(PAIR_TRAIL_BASE + i, t->band[i], -1);
    init_pair(PAIR_AXIS, t->axis, -1);
    init_pair(PAIR_LIVE, t->live, -1);
}

/* theme_apply_pairs_8color_fallback — single fallback assignment for
 * 8-colour terminals where per-theme xterm-256 indices can't be
 * expressed.  All N_THEMES collapse to the SAME 8-colour set —
 * trade-off is "lose theme variety, keep legibility everywhere".
 * Picked roles: cool trail, neutral axes, hot live marker. */
static void theme_apply_pairs_8color_fallback(void)
{
    for (int i = 0; i < TRAIL_BAND_COUNT; i++)
        init_pair(PAIR_TRAIL_BASE + i, COLOR_CYAN, -1);
    init_pair(PAIR_AXIS, COLOR_WHITE, -1);
    init_pair(PAIR_LIVE, COLOR_RED,   -1);
}

/* theme_apply — top-level dispatcher: clamp index, branch on terminal
 * capability, delegate to the appropriate helper. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_apply_pairs_256color(&themes[idx]);
    else               theme_apply_pairs_8color_fallback();
}

/* color_init_hud_pairs — set up the project-standard HUD pair
 * (bright yellow) and HINT pair (bright cyan) per CLAUDE.md's "HUD
 * Standard".  These pairs are NOT per-theme — they stay constant as
 * t/T cycles through themes so the status row never loses contrast. */
static void color_init_hud_pairs(void)
{
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  PAIR_HUD_FG_256,  -1);
        init_pair(PAIR_HINT, PAIR_HINT_FG_256, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* color_init — bring ncurses colour subsystem online and load the
 * default (first) theme.
 *
 *   STEP 1 — enable colour + default-bg passthrough
 *   STEP 2 — install the constant HUD pairs (project standard)
 *   STEP 3 — push the first theme's colours
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    color_init_hud_pairs();
    theme_apply(0);
}

/* ===================================================================== */
/* §5  physics — Duffing ODE                                              */
/* ===================================================================== */

/*
 * DuffingSystem — the INVARIANT parameters of the equation:
 *   ẍ + γẋ - x + x³ = A · cos(ω·t)
 *
 * INTENT
 *   Separates "what the oscillator IS" (γ, ω, A) from "what its
 *   current configuration is" (DuffingState).  The math text
 *   reads this way too — Goldstein-style: PARAMETERS define the
 *   system, COORDINATES define the state.  Splitting in code lets:
 *     • duffing_deriv take (state, system) cleanly,
 *     • RK4's intermediate k₁..k₄ states be created without
 *       carrying the parameters around,
 *     • a future "perturb parameters mid-simulation" feature touch
 *       only this struct.
 *
 * CONTEXT
 *   One instance lives inside Duffing.system.  Lifetime:
 *     • duffing_system_init_from_preset writes (once per r/n/p).
 *     • duffing_deriv reads at every RK4 stage (4× per substep,
 *       so 32× per tick at INT_STEPS_PER_TICK=8).
 *     • HUD reads .gamma / .omega / .A for the row-1 status line.
 *   Const during a single sim tick.
 *
 * MEMBER LOGIC
 *   gamma : damping coefficient γ ≥ 0.  Energy SINK per unit velocity
 *           (-γv term in the equation).  Holmes-canon γ = 0.25 sets a
 *           moderate energy-dissipation rate so the attractor settles
 *           on a visible timescale (≈ 10 drive periods).  γ → 0
 *           approaches Hamiltonian behaviour (no settling, long
 *           transients) — Group F probes this regime.
 *   omega : drive angular frequency ω (rad / s).  Sets the period
 *           T = 2π / ω of the cos(ωt) drive and thus the natural
 *           rhythm of every preset.  Holmes-canon ω = 1.0 puts the
 *           drive near the linearised natural frequency √2 of the
 *           small-amplitude single-well oscillator.  Group G detunes.
 *   A     : drive amplitude.  Energy SOURCE pumped in each drive
 *           period.  The bifurcation parameter — sweeping A walks
 *           the period-doubling cascade (Group B) into chaos
 *           (Group C) into cross-well chaos (Group D).
 *
 * REFERENCES (numbered in file-header References block)
 *   [1] Duffing (1918)  — the original cubic-stiffness oscillator.
 *   [2] Holmes (1979)   — proof that the (γ, ω, A) triple admits a
 *                         strange attractor for A above the
 *                         homoclinic-bifurcation threshold.
 *   [4] Strogatz §10    — pedagogical derivation of the double-well
 *                         potential V(x) = -x²/2 + x⁴/4 whose
 *                         gradient yields the (x - x³) restoring term.
 */
typedef struct {
    float gamma;
    float omega;
    float A;
} DuffingSystem;

/*
 * DuffingState — the dynamical state of the ODE.
 *
 * INTENT
 *   The state vector y for the ODE dy/dt = f(y, sys).  "Where we are
 *   in iterating the system" — what evolves under duffing_rk4().
 *   Kept as its own type so it can be passed by value into duffing_
 *   deriv() and combined with state_add() (k₁..k₄ are themselves
 *   DuffingState values) without dragging system parameters along.
 *
 *   CRUCIAL DESIGN POINT — t lives in the STATE, not the system.
 *   The Duffing equation is NON-AUTONOMOUS: the drive A·cos(ωt)
 *   makes f depend explicitly on t, so RK4 must thread t through
 *   every stage (the slope at the midpoint uses cos(ω(t+½dt)),
 *   not cos(ωt)).  Treating t as a dynamic state variable with
 *   dt/dt = 1 lets RK4 advance time COHERENTLY with x and v, no
 *   special-case logic needed.  The textbook view: an n-D non-
 *   autonomous ODE in (x₁..xₙ) becomes an (n+1)-D AUTONOMOUS
 *   ODE in (x₁..xₙ, t) — here n = 2, autonomous in (x, v, t).
 *
 * CONTEXT
 *   One instance lives inside Duffing.state.  Lifetime:
 *     • duffing_state_init_from_preset writes (x₀, v₀, t=0) on r/n/p.
 *     • duffing_rk4 mutates ~480 times/sec (60 Hz × 8 substeps).
 *     • duffing_deriv reads (intermediate k_i stages also).
 *     • scene_tick reads .x, .v at the end of each substep to push
 *       into the Trail ring buffer.
 *     • HUD reads .t for the "t:%.1fs" status field.
 *
 * MEMBER LOGIC
 *   x : position.  Double-well potential V(x) = -x²/2 + x⁴/4 has
 *       minima at x = ±1 and a saddle at x = 0.  Trajectories
 *       typically oscillate within one well, or for A > critical
 *       cross between wells (Group D).  No bound — stored
 *       unclipped; clipping happens only at viewport-paint time.
 *   v : velocity dx/dt.  Same range as x in typical runs (|v| ≲ 2
 *       on the Holmes attractor); the viewport clips to V_MIN..V_MAX.
 *   t : simulation time in seconds since the last r/n/p reset.
 *       Advances by DUFFING_DT × INT_STEPS_PER_TICK per scene_tick.
 *       Stored as float — adequate precision for the ≈ 5-minute
 *       runs typical of this demo; for hours-long runs a double
 *       would be safer (cos(ωt) loses precision past t ≳ 10⁶).
 *
 * REFERENCES (numbered in file-header References block)
 *   [2] Holmes §II — defines the (x, v, t) phase-space coordinates
 *                    used by this struct.
 *   [8] Numerical Recipes Ch. 17.1 — explains why an ODE state
 *                                    vector is best implemented as
 *                                    a flat by-value type for RK4:
 *                                    k_i need cheap structural
 *                                    copies, not pointer aliasing.
 */
typedef struct {
    float x;
    float v;
    float t;
} DuffingState;

/*
 * Duffing — composite: SYSTEM + STATE.
 *
 * INTENT
 *   Bundle the two independent concerns under one named type so
 *   callers that need "the whole oscillator" take one pointer
 *   instead of two.  RK4 mutates only .state; preset cycling
 *   mutates only .system; render reads both.  The two-way split
 *   mirrors the textbook treatment: PARAMETERS / COORDINATES.
 *
 * CONTEXT
 *   One instance lives on Scene.  Sub-structs are touched separately:
 *     .system : written by duffing_system_init_from_preset on r/n/p.
 *     .state  : written by duffing_rk4 every substep; reset by
 *               duffing_state_init_from_preset on r/n/p.
 *
 * MEMBER LOGIC
 *   system : §5 DuffingSystem — γ, ω, A (invariants).
 *   state  : §5 DuffingState  — x, v, t (the ODE phase point).
 *
 * REFERENCES (numbered in file-header References block)
 *   [4] Strogatz §6.1 — the textbook PARAMETER / STATE separation
 *                       that this struct's two-way decomposition
 *                       reproduces in code form.
 */
typedef struct {
    DuffingSystem system;
    DuffingState  state;
} Duffing;

/* duffing_deriv — evaluate the RHS f(state, system) of dy/dt = f(y, sys).
 * Returns a DuffingState whose .x is dx/dt = v (kinematic identity) and
 * whose .v is dv/dt from the Duffing equation:
 *
 *   dx/dt = v
 *   dv/dt = -γ·v + x - x³ + A·cos(ω·t)
 *           └─damping─┘ └ non-linear ┘ └─periodic drive─┘
 *                       restoring force
 *
 * The .t component of the returned state is the kinematic identity
 * dt/dt = 1 — kept on the struct so state_add() can advance time
 * coherently with x and v inside RK4.
 *
 * Equation form: header refs [1] Duffing (1918) for the original
 * cubic-stiffness oscillator and [4] Strogatz §10 for the pedagogical
 * derivation of the double-well potential V(x) = -x²/2 + x⁴/4 whose
 * gradient gives the (x - x³) restoring term. */
static inline DuffingState duffing_deriv(const DuffingState *s,
                                         const DuffingSystem *sys)
{
    float damping_term        = -sys->gamma * s->v;
    float linear_restoring    =  s->x;
    float cubic_stiffening    = -s->x * s->x * s->x;
    float periodic_drive_term =  sys->A * cosf(sys->omega * s->t);

    DuffingState out;
    out.x = s->v;                                                /* dx/dt */
    out.v = damping_term + linear_restoring + cubic_stiffening   /* dv/dt */
          + periodic_drive_term;
    out.t = 1.0f;                                                /* dt/dt */
    return out;
}

/* state_add — y + h·k, returned as a new DuffingState.  Used inside
 * RK4 to build intermediate stage points. */
static inline DuffingState state_add(const DuffingState *a,
                                     float h, const DuffingState *k)
{
    DuffingState r;
    r.x = a->x + h * k->x;
    r.v = a->v + h * k->v;
    r.t = a->t + h * k->t;
    return r;
}

/* rk4_butcher_weighted_average — Simpson-rule combination of the 4
 * slope estimates: (k₁ + 2k₂ + 2k₃ + k₄) / 6.  Classical 4th-order
 * Butcher tableau weights (Press et al. 17.1 eq. 17.1.3). */
#define RK4_BUTCHER_WEIGHT_SUM   6.0f
#define RK4_MIDPOINT_FRACTION    0.5f
static inline DuffingState rk4_butcher_weighted_average(
    const DuffingState *k1, const DuffingState *k2,
    const DuffingState *k3, const DuffingState *k4)
{
    DuffingState avg;
    avg.x = (k1->x + 2.0f*k2->x + 2.0f*k3->x + k4->x) / RK4_BUTCHER_WEIGHT_SUM;
    avg.v = (k1->v + 2.0f*k2->v + 2.0f*k3->v + k4->v) / RK4_BUTCHER_WEIGHT_SUM;
    avg.t = (k1->t + 2.0f*k2->t + 2.0f*k3->t + k4->t) / RK4_BUTCHER_WEIGHT_SUM;
    return avg;
}

/* duffing_rk4 — single RK4 step of size dt over the (x, v, t) state.
 *
 *   STAGE 1 — slope_start    = f(y)
 *   STAGE 2 — slope_mid_1    = f(y + ½dt · slope_start)
 *   STAGE 3 — slope_mid_2    = f(y + ½dt · slope_mid_1)
 *   STAGE 4 — slope_end      = f(y +  dt · slope_mid_2)
 *   UPDATE  — y ← y + dt · (k₁ + 2k₂ + 2k₃ + k₄) / 6
 *
 * Because Duffing is non-autonomous, t MUST advance along with x and
 * v inside the stages — the drive cos(ωt) at the midpoint is different
 * from cos(ωt) at the start.
 *
 * See header ref [8] Numerical Recipes Ch. 17.1 for the Butcher tableau,
 * O(dt⁵) local truncation error, and the stability domain.  Holmes
 * (ref [2]) integrated his original chaos proof with the same scheme
 * at dt ≈ T/200 where T = 2π/ω is the drive period — this file uses
 * DUFFING_DT = 0.01 ≈ T/630, comfortably finer. */
static void duffing_rk4(Duffing *d, float dt)
{
    const DuffingSystem *sys = &d->system;
    float half_dt = RK4_MIDPOINT_FRACTION * dt;

    DuffingState slope_start    = duffing_deriv(&d->state, sys);

    DuffingState midpoint_1     = state_add(&d->state, half_dt, &slope_start);
    DuffingState slope_mid_1    = duffing_deriv(&midpoint_1, sys);

    DuffingState midpoint_2     = state_add(&d->state, half_dt, &slope_mid_1);
    DuffingState slope_mid_2    = duffing_deriv(&midpoint_2, sys);

    DuffingState endpoint       = state_add(&d->state, dt, &slope_mid_2);
    DuffingState slope_end      = duffing_deriv(&endpoint, sys);

    DuffingState effective_slope = rk4_butcher_weighted_average(
        &slope_start, &slope_mid_1, &slope_mid_2, &slope_end);
    d->state = state_add(&d->state, dt, &effective_slope);
}

/* duffing_state_init_from_preset — pure (θ, ω, …)-style initialiser:
 * apply a DuffingPreset's (x₀, v₀) onto the state and reset t to 0.
 * The system parameters are set separately by duffing_system_init_from_preset. */
static inline void duffing_state_init_from_preset(DuffingState *s,
                                                  const DuffingPreset *preset)
{
    s->x = preset->x0;
    s->v = preset->v0;
    s->t = 0.0f;
}

static inline void duffing_system_init_from_preset(DuffingSystem *sys,
                                                   const DuffingPreset *preset)
{
    sys->gamma = preset->gamma;
    sys->omega = preset->omega;
    sys->A     = preset->A;
}

/* ===================================================================== */
/* §6  trail — (x, v) phase-space ring buffer                             */
/* ===================================================================== */

/*
 * Trail — fixed-capacity ring buffer of recent (x, v) phase-space
 * samples.
 *
 * INTENT
 *   The orbit's SHAPE — limit cycle, period-2, period-4, strange
 *   attractor — is what the demo exists to show.  A static frame
 *   shows one (x, v) dot, useless for that purpose.  This struct
 *   keeps the last TRAIL_MAX samples so §9 can re-draw the trace
 *   each frame, fading older points to communicate time direction.
 *   Watching the trail accumulate IS the pedagogical event.
 *
 * CONTEXT
 *   One instance lives on Scene.  Fed by scene_tick() at every RK4
 *   substep (so INT_STEPS_PER_TICK pushes per tick).  Drawn by §9
 *   phase_space_paint_trail() which walks oldest→newest and maps
 *   each sample's age to a colour/glyph band via trail_band_for_age.
 *   Cleared by trail_reset() on r/n/p / SIGWINCH-induced reset.
 *
 *   Capacity TRAIL_MAX = 3000 ≈ 50 s at 60 Hz × no substep push,
 *   or shorter in real-time when accounting for the substep push.
 *   Long enough that a chaotic attractor fills in visibly, short
 *   enough that the buffer fits in ~24 KB of cache.
 *
 * DATA-STRUCTURE LOGIC
 *   Ring buffer chosen over a growable list for two reasons:
 *     1. ZERO ALLOCATION in the hot path — CLAUDE.md "Memory
 *        Allocation" rule (tick must not malloc).
 *     2. CONSTANT cost regardless of program lifetime — push and
 *        full-buffer iteration are O(TRAIL_MAX), no resize ever.
 *   Indexing: newest at .head; oldest at trail_oldest_index().
 *   trail_push() advances head FIRST then writes, so .head is
 *   always the just-written slot.
 *
 * MEMBER LOGIC
 *   x[i] : phase-space x of the i-th stored sample, in PHYSICAL
 *          units (not cells).  Keep float precision so the trail
 *          inherits the integrator's sub-cell accuracy — cell
 *          conversion happens once at paint time via §9
 *          phase_to_cell_x().
 *   v[i] : phase-space v of the i-th stored sample, physical units.
 *   head : index of the MOST RECENT (newest) sample, mod TRAIL_MAX.
 *          When count < TRAIL_MAX, valid slots are
 *          (head - count + 1 .. head) wrapping around.
 *   count: number of valid samples currently stored.  Saturates at
 *          TRAIL_MAX once the buffer fills; from then on each push
 *          overwrites the oldest slot.
 *
 * REFERENCES
 *   • Sedgewick "Algorithms" 4th ed. §1.3 — canonical reference
 *     for ring-buffer / circular-queue implementation.
 *   • Header ref [3] Ueda (1980) — phase-portrait atlas that
 *     captures the kind of trajectory shapes this Trail records.
 */
typedef struct {
    float x[TRAIL_MAX];
    float v[TRAIL_MAX];
    int   head;
    int   count;
} Trail;

static void trail_reset(Trail *tr) { tr->head = 0; tr->count = 0; }

static void trail_push(Trail *tr, float x, float v)
{
    tr->head = (tr->head + 1) % TRAIL_MAX;
    tr->x[tr->head] = x;
    tr->v[tr->head] = v;
    if (tr->count < TRAIL_MAX) tr->count++;
}

/* trail_oldest_index — index of the oldest still-valid sample in
 * the ring.  Works in both "buffer filling" and "buffer full" cases. */
static inline int trail_oldest_index(const Trail *tr)
{
    return (tr->head - tr->count + 1 + TRAIL_MAX) % TRAIL_MAX;
}

/* trail_band_for_age — map sample age to colour-tier index.  Linear
 * partition over [0, count-1] so each band gets ~count/N samples.
 * age = 0 → newest (brightest); age = count-1 → oldest (dimmest). */
static inline int trail_band_for_age(int age, int count)
{
    int band = (TRAIL_BAND_COUNT - 1) - (age * TRAIL_BAND_COUNT) / count;
    if (band < 0)                    band = 0;
    if (band > TRAIL_BAND_COUNT - 1) band = TRAIL_BAND_COUNT - 1;
    return band;
}

/* ===================================================================== */
/* §7  state — typed wrappers around the two cycled selections           */
/* ===================================================================== */

/*
 * PresetState — typed wrapper around "which preset row is loaded".
 *
 * INTENT
 *   The Preset enum is just an integer — it tells you nothing
 *   about the semantics of "next", "previous", or "look up the
 *   active row".  Wrapping it in a struct + helpers makes the
 *   binding table in app_handle_key read as
 *
 *       case 'n': preset_state_cycle_next(&s->preset); scene_reset(s);
 *
 *   instead of inline modular arithmetic.  This is the "Replace
 *   Primitive with Object" pattern — a one-field struct exists
 *   not to hold data but to ATTACH a named API to that data so
 *   the call sites read as intentions, not arithmetic.
 *
 * CONTEXT
 *   One instance lives on Scene.  Mutated by app_cycle_preset_next/
 *   prev when n/p are pressed; read by scene_load_active_preset()
 *   (via preset_state_active → presets[current]) and by the HUD
 *   writers for the row-1 "preset:NAME" label.
 *
 * MEMBER LOGIC
 *   current : index into the presets[] table (§1).  Must be in
 *             [0, N_PRESETS) — the cycle helpers guarantee this
 *             by modular arithmetic.  Initial value set by
 *             scene_init to PRESET_SLEW (Holmes' canonical chaos)
 *             so the demo opens on the headline behaviour.
 *
 * REFERENCES
 *   • Fowler "Refactoring" (2nd ed.), "Replace Primitive with Object".
 *     The pattern this struct implements.
 */
typedef struct {
    int current;
} PresetState;

static void preset_state_init(PresetState *p, int initial) { p->current = initial; }
static void preset_state_cycle_next(PresetState *p)        { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)        { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const DuffingPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }

/*
 * PaletteState — typed wrapper around "which colour theme is active".
 *
 * INTENT
 *   Same shape as PresetState (one-int wrapper + cycle helpers)
 *   but kept as a SEPARATE type because their tables differ
 *   (presets[] vs themes[]) and conflating them would just be
 *   punning.  Distinct types let scene_apply_theme() take exactly
 *   the right wrapper and refuse anything else — avoids the class
 *   of bug where a "next" key on the wrong wrapper would silently
 *   index past N_PRESETS into N_THEMES or vice versa.
 *
 * CONTEXT
 *   One instance lives on Scene.  Mutated by app_cycle_theme_next/
 *   prev when t/T are pressed.  After every mutation the caller
 *   invokes palette_state_apply() → §3 theme_apply(current) which
 *   pushes the new init_pair() values into ncurses.  Read by the
 *   HUD writer for the row-1 "theme:NAME" label.
 *
 * MEMBER LOGIC
 *   current : index into the themes[] table (§3).  Must be in
 *             [0, N_THEMES).  Initial value is 0 (set by
 *             scene_init); theme 0 is conventionally the most
 *             legible default.
 *
 * REFERENCES
 *   • Fowler "Refactoring" — "Replace Primitive with Object",
 *     same pattern as PresetState.  The deliberate type-distinction
 *     between PresetState and PaletteState despite identical shape
 *     is the DDD "Tiny Types" idea: values from different domains
 *     shouldn't share a type just because the bits match.
 */
typedef struct {
    int current;
} PaletteState;

static void palette_state_init(PaletteState *p, int initial) { p->current = initial; }
static void palette_state_cycle_next(PaletteState *p)        { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p)        { p->current = (p->current + N_THEMES - 1) % N_THEMES; }
static const Theme *palette_state_active(const PaletteState *p) { return &themes[p->current]; }
static void palette_state_apply(const PaletteState *p)       { theme_apply(p->current); }

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

/*
 * Scene — composite owner of all mutable simulation state.
 *
 * INTENT
 *   Bundle every piece of mutable state under one named struct so the
 *   App layer owns ONE Scene (no loose globals) and the main loop
 *   drives it through a tiny named-method API.  Each sub-field is a
 *   NAMED CONCEPT: the oscillator, its phase-space history, the
 *   selected initial-condition row, the selected palette.
 *
 *   Decomposition axis is "what the simulation IS MADE OF":
 *     • a driven body (duffing),
 *     • a record of where it has been (trail),
 *     • a selection of which experiment is active (preset),
 *     • a selection of which palette renders it (palette).
 *   Presentation flags (paused) and grid dims sit at the top level —
 *   they belong to the Scene as a whole, not to a sub-system.
 *
 * CONTEXT
 *   One instance embedded inside App.  Lifetime/mutation map:
 *     • scene_tick      (~60 Hz × INT_STEPS_PER_TICK substeps,
 *                        advances .duffing.state + .trail)
 *     • scene_reset     (r / n / p — reloads preset, clears trail)
 *     • scene_resize    (SIGWINCH — re-derives drawable extent)
 *     • app_handle_key  (sub-states .preset, .palette, .paused)
 *   Read by:
 *     • scene_paint     (every frame — draws all sub-systems)
 *     • HUD writers     (preset / palette / γ / ω / A / t / paused)
 *
 * MEMBER LOGIC
 *   duffing : §5 Duffing — DuffingSystem (γ, ω, A) + DuffingState
 *             (x, v, t).  Mutated by scene_tick() at every RK4
 *             substep; read by scene_paint() for the live '@' glyph.
 *   trail   : §6 Trail — ring buffer of recent (x, v) phase-space
 *             samples; the characteristic trace of the attractor.
 *   preset  : §7 PresetState — wrapper around "which preset row is
 *             loaded".  n/p cycle it; scene_reset() applies it.
 *   palette : §7 PaletteState — wrapper around "which theme is
 *             active".  t/T cycle it; scene_apply_theme() pushes the
 *             change into ncurses.
 *   paused  : when true, scene_tick early-returns — physics frozen
 *             but rendering continues so the user can study the pose.
 *   map_w/h : grid dimensions in CELLS available to scene_paint
 *             (terminal size minus HUD_BAND_RESERVED_ROWS).  Drives
 *             phase_space_viewport_build which centres the viewport.
 *
 * REFERENCES
 *   • Gamma, Helm, Johnson, Vlissides (1995) — "Design Patterns",
 *     "Composite" pattern.  Scene is the COMPOSITE root: one object
 *     holding a tree of sub-objects, with one tick / paint call
 *     fanning out to each leaf in deterministic order.
 *   • Sussman, G. J., Wisdom, J. (2014) — "Structure and
 *     Interpretation of Classical Mechanics".  The "system / state /
 *     observable" tri-partition this Scene mirrors at the simulation-
 *     architecture level.
 */
typedef struct {
    Duffing      duffing;
    Trail        trail;
    PresetState  preset;
    PaletteState palette;
    bool         paused;
    int          map_w, map_h;
} Scene;

/* scene_load_active_preset — apply both halves of the active preset
 * row: system parameters (γ, ω, A) AND initial state (x₀, v₀, t=0).
 * Trail is cleared separately by scene_reset(). */
static void scene_load_active_preset(Scene *s)
{
    const DuffingPreset *preset = preset_state_active(&s->preset);
    duffing_system_init_from_preset(&s->duffing.system, preset);
    duffing_state_init_from_preset (&s->duffing.state,  preset);
}

static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

static void scene_reset(Scene *s)
{
    trail_reset(&s->trail);
    scene_load_active_preset(s);
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->map_w  = mw;
    s->map_h  = mh;
    s->paused = false;

    preset_state_init (&s->preset,  PRESET_SLEW);  /* headline: Holmes canon */
    palette_state_init(&s->palette, 0);
    scene_reset(s);
}

static void scene_resize(Scene *s, int mw, int mh)
{
    s->map_w = mw;
    s->map_h = mh;
}

/* scene_tick — advance one rendered frame of physics + trail.  Each
 * frame runs INT_STEPS_PER_TICK fixed-dt RK4 substeps so the
 * integrator stays stable when the orbit whips through the cubic
 * non-linearity. */
static void scene_tick(Scene *s, float dt)
{
    (void)dt;
    if (s->paused) return;
    for (int i = 0; i < INT_STEPS_PER_TICK; i++) {
        duffing_rk4(&s->duffing, DUFFING_DT);
        trail_push(&s->trail, s->duffing.state.x, s->duffing.state.v);
    }
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

/*
 * Screen — terminal-adapter handle: ncurses lifecycle + current
 * window dimensions.
 *
 * INTENT
 *   ncurses owns the actual terminal mode and back-buffer; this
 *   struct is the THIN OWNERSHIP HANDLE that the program passes
 *   around.  Bundling (cols, rows) here lets every painter take ONE
 *   Screen parameter instead of calling getmaxyx() at every draw
 *   site.  Funnelling all init/teardown through screen_init /
 *   screen_free keeps the ncurses dependency at a single boundary —
 *   nothing else in the file calls initscr / endwin / getmaxyx.
 *
 * CONTEXT
 *   One instance lives on App.  Initialised by screen_init() during
 *   app_bootstrap().  Refreshed by screen_resize() inside
 *   app_handle_pending_resize() whenever SIGWINCH fires.  Read by
 *   every painter that needs to clip against the terminal bounds
 *   (phase_space_paint_axes, _trail, _live_marker, HUD writers).
 *   Freed at exit via screen_free(); endwin() also runs from
 *   atexit(cleanup) as a safety net if the program crashes before
 *   main() returns normally.
 *
 * MEMBER LOGIC
 *   cols : current terminal width in CHARACTER CELLS.  Updated only
 *          by screen_init / screen_resize via getmaxyx(stdscr, ...).
 *   rows : current terminal height in CHARACTER CELLS.  Row 0 is the
 *          top of the terminal; row (rows - 1) is the bottom.  The
 *          HUD reserves HUD_TOP_ROWS at the top and HUD_BOTTOM_ROWS
 *          at the bottom (see §1).
 *
 * REFERENCES (numbered in file-header References block)
 *   [10] Gookin "Programmer's Guide to NCurses" Ch. 2 — the
 *        initscr / endwin / getmaxyx / wnoutrefresh / doupdate API
 *        surface that the screen_* helpers wrap.
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s) { initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1); color_init();
    getmaxyx(stdscr, s->rows, s->cols); }
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols); }
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/*
 * PhaseSpaceViewport — the mapping from physical (x, v) ∈ [X_MIN,
 * X_MAX] × [V_MIN, V_MAX] to a rectangle of terminal cells.
 *
 * INTENT
 *   Bundles the four numbers (origin cell + cell dimensions) that
 *   every phase-space painter needs.  Without this struct, every
 *   painter would take 4 loose ints and the screen-space ↔ phase-
 *   space conversion would be scattered.  With it, viewport
 *   construction happens ONCE per frame in scene_paint and every
 *   painter takes one viewport + its specific data.
 *
 *   The PHASE-SPACE PORTRAIT is the canonical way to visualise a
 *   2-D dynamical system since Poincaré: plot (x, ẋ) and let the
 *   eye discover the attractor's shape.  Ueda (1980) — ref [3] —
 *   is the picture-book that pairs every (γ, ω, A) preset region
 *   in §1 with its expected portrait shape.
 *
 * MEMBER LOGIC
 *   gx0, gy0 : cell coordinates of the viewport's TOP-LEFT corner.
 *              Computed by scene_paint to centre the viewport inside
 *              the drawable band.
 *   w, h     : viewport size in cells.  Equals scene's map_w / map_h
 *              modulo clipping.
 *
 * REFERENCES (numbered in file-header References block)
 *   [3]  Ueda (1980) "Picture book" — the visual atlas this viewport
 *        is designed to reproduce in ASCII.
 *   [10] Gookin "Programmer's Guide to NCurses" Ch. 4 — mvaddch /
 *        wnoutrefresh model that every painter funnels through.
 */
typedef struct {
    int gx0, gy0;
    int w, h;
} PhaseSpaceViewport;

/* viewport_centered_origin_x — horizontal cell offset that centres
 * an inner rectangle of width w inside an outer rectangle of width
 * `outer_cols`.  Clipped at 0 so the inner never starts to the left
 * of the screen edge. */
static inline int viewport_centered_origin_x(int outer_cols, int w)
{
    int gx0 = (outer_cols - w) / 2;
    return (gx0 < 0) ? 0 : gx0;
}

/* viewport_centered_origin_y_in_drawable — vertical offset that
 * centres an inner rectangle of height h INSIDE THE DRAWABLE BAND
 * (rows - HUD_BAND_RESERVED_ROWS), shifted down by HUD_TOP_ROWS so
 * the rectangle never overlaps the top HUD.  Clipped to HUD_TOP_ROWS. */
static inline int viewport_centered_origin_y_in_drawable(int outer_rows, int h)
{
    int gy0 = ((outer_rows - HUD_BAND_RESERVED_ROWS) - h) / 2 + HUD_TOP_ROWS;
    return (gy0 < HUD_TOP_ROWS) ? HUD_TOP_ROWS : gy0;
}

/* phase_space_viewport_build — compute the viewport for the current
 * frame: centre an w×h rectangle inside the drawable band, clipped
 * to non-negative offsets so it never overlaps the HUD.
 *
 *   STEP 1 — take inner dims from the scene (cells available)
 *   STEP 2 — derive the centred origin (gx0, gy0) inside the
 *            drawable band via the two named-centring helpers
 */
static PhaseSpaceViewport phase_space_viewport_build(const Scene *s,
                                                     int cols, int rows)
{
    PhaseSpaceViewport vp;
    vp.w   = s->map_w;
    vp.h   = s->map_h;
    vp.gx0 = viewport_centered_origin_x(cols, vp.w);
    vp.gy0 = viewport_centered_origin_y_in_drawable(rows, vp.h);
    return vp;
}

/* phase_to_cell_x / _y — convert a single PHYSICAL coordinate to its
 * cell index inside the viewport (LOCAL coordinates 0..w-1, 0..h-1).
 * The y axis is flipped at paint time (high v → high row from the
 * top) so positive velocity reads as "above" the v=0 axis. */
static inline int phase_to_cell_x(const PhaseSpaceViewport *vp, float x)
{
    int cx = (int)((x - X_MIN) / (X_MAX - X_MIN) * (float)vp->w);
    if (cx < 0)         cx = 0;
    if (cx >= vp->w)    cx = vp->w - 1;
    return cx;
}
static inline int phase_to_cell_y(const PhaseSpaceViewport *vp, float v)
{
    int cy = (int)((v - V_MIN) / (V_MAX - V_MIN) * (float)vp->h);
    if (cy < 0)         cy = 0;
    if (cy >= vp->h)    cy = vp->h - 1;
    return cy;
}

/* phase_space_plot — write one glyph at the SCREEN cell corresponding
 * to physical (x, v).  Single funnel point for the viewport's screen-
 * coord transform: gy0 + (h - 1 - cy) flips the y axis (cells grow
 * down, v grows up). */
static inline void phase_space_plot(const PhaseSpaceViewport *vp,
                                    float x, float v, chtype glyph)
{
    int cx = phase_to_cell_x(vp, x);
    int cy = phase_to_cell_y(vp, v);
    mvaddch(vp->gy0 + (vp->h - 1 - cy), vp->gx0 + cx, glyph);
}

/* viewport_paint_vertical_line — draw a vertical line at LOCAL column
 * `cx_local` (0..w-1), spanning the full viewport height.  Caller
 * sets the attribute / colour before invoking. */
static void viewport_paint_vertical_line(const PhaseSpaceViewport *vp,
                                         int cx_local, chtype glyph)
{
    for (int y = 0; y < vp->h; y++)
        mvaddch(vp->gy0 + (vp->h - 1 - y), vp->gx0 + cx_local, glyph);
}

/* viewport_paint_horizontal_line — draw a horizontal line at LOCAL
 * row `cy_local` (0..h-1), spanning the full viewport width. */
static void viewport_paint_horizontal_line(const PhaseSpaceViewport *vp,
                                           int cy_local, chtype glyph)
{
    for (int x = 0; x < vp->w; x++)
        mvaddch(vp->gy0 + (vp->h - 1 - cy_local), vp->gx0 + x, glyph);
}

/* phase_space_paint_axes — draw the x = 0 and v = 0 reference lines
 * across the viewport.  Crucial visual aid: without them the orbit
 * just floats in space; with them the reader can SEE which well the
 * orbit is in and how often it crosses zero.
 *
 *   STEP 1 — locate the zero columns/rows in viewport-local coords
 *   STEP 2 — paint vertical x = 0 axis (`|`)
 *   STEP 3 — paint horizontal v = 0 axis (`-`)
 */
static void phase_space_paint_axes(const PhaseSpaceViewport *vp)
{
    int x_zero_col = phase_to_cell_x(vp, 0.0f);
    int v_zero_row = phase_to_cell_y(vp, 0.0f);

    attron(COLOR_PAIR(PAIR_AXIS));
    viewport_paint_vertical_line  (vp, x_zero_col, '|');
    viewport_paint_horizontal_line(vp, v_zero_row, '-');
    attroff(COLOR_PAIR(PAIR_AXIS));
}

/* TRAIL_GLYPH — the single ASCII glyph used for every trail sample
 * in this demo.  Named so future variants (per-band glyph) have one
 * obvious place to change. */
#define TRAIL_GLYPH  '.'

/* phase_space_paint_trail — walk the ring oldest→newest, plotting
 * each sample in the band corresponding to its age (fades the tail).
 *
 *   STEP 1 — short-circuit on empty ring
 *   STEP 2 — locate oldest still-valid sample in the ring
 *   STEP 3 — for each sample: derive (age, band) → plot in band's pair
 */
static void phase_space_paint_trail(const PhaseSpaceViewport *vp,
                                    const Trail *tr)
{
    /* STEP 1 */
    if (tr->count == 0) return;

    /* STEP 2 */
    int oldest = trail_oldest_index(tr);

    /* STEP 3 */
    for (int i = 0; i < tr->count; i++) {
        int sample_index = (oldest + i) % TRAIL_MAX;
        int age          = tr->count - 1 - i;          /* 0 = newest */
        int band         = trail_band_for_age(age, tr->count);

        int pair = PAIR_TRAIL_BASE + band;
        attron(COLOR_PAIR(pair) | A_BOLD);
        phase_space_plot(vp, tr->x[sample_index], tr->v[sample_index], TRAIL_GLYPH);
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

/* phase_space_paint_live_marker — '@' glyph at the LIVE (x, v).
 * Drawn last so it sits on top of the trail and axes — the eye
 * tracks the moving marker against the static attractor shape. */
static void phase_space_paint_live_marker(const PhaseSpaceViewport *vp,
                                          const DuffingState *state)
{
    attron(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
    phase_space_plot(vp, state->x, state->v, '@');
    attroff(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
}

/* scene_paint — render one frame, back→front:
 *
 *   STEP 1 — build the phase-space viewport (centre, clip)
 *   STEP 2 — draw x=0 / v=0 reference axes
 *   STEP 3 — draw the trail (oldest→newest, fading)
 *   STEP 4 — draw the LIVE '@' marker on top
 */
static void scene_paint(const Scene *s, int cols, int rows)
{
    PhaseSpaceViewport vp = phase_space_viewport_build(s, cols, rows);
    phase_space_paint_axes(&vp);
    phase_space_paint_trail(&vp, &s->trail);
    phase_space_paint_live_marker(&vp, &s->duffing.state);
}

/* hud_paint_top_left_title — fixed title in row 0 column HUD_LEFT_MARGIN. */
static void hud_paint_top_left_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " DUFFING OSCILLATOR ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_format_top_right_status — format the dynamic status string
 * (fps, Hz, preset name + index, A, t) into a stack buffer.  Pure
 * formatter, no ncurses calls — pulled out so hud_top reads as
 * "format then paint" not a tangled snprintf+positioning block. */
static void hud_format_top_right_status(char *buf, size_t bufsz,
                                        double fps, int sim_fps,
                                        const Scene *s)
{
    const DuffingPreset *active = preset_state_active(&s->preset);
    snprintf(buf, bufsz,
             " %5.1f fps  %3d Hz  %s [%d/%d]  A:%.2f  t:%.1fs ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : active->name,
             s->preset.current + 1, N_PRESETS,
             (double)s->duffing.system.A,
             (double)s->duffing.state.t);
}

/* hud_paint_top_right — paint a right-anchored string at row 0.
 * Right-anchor formula: column = cols - strlen, clipped at 0 so a
 * narrow terminal doesn't push the start negative. */
static void hud_paint_top_right(int cols, const char *text)
{
    int anchor_col = cols - (int)strlen(text);
    if (anchor_col < 0) anchor_col = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, anchor_col, "%s", text);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_top — row-0 HUD driver.
 *
 *   STEP 1 — top-left title cell (constant text)
 *   STEP 2 — format the dynamic status into a buffer
 *   STEP 3 — paint the buffer right-anchored to `cols`
 */
static void hud_top(int cols, double fps, int sim_fps, const Scene *s)
{
    hud_paint_top_left_title();

    char buf[HUD_COLS + 1];
    hud_format_top_right_status(buf, sizeof buf, fps, sim_fps, s);
    hud_paint_top_right(cols, buf);
}

/* HUD_PARAM_CELL_WIDTH_* — printed widths of the three row-1 cells
 * including their leading + trailing spaces.  Sized to the
 * fixed-width %-8s format specifiers so the layout doesn't shift
 * as preset / theme names change. */
#define HUD_PARAM_CELL_WIDTH_PRESET   19   /* " preset:XXXXXXXX " */
#define HUD_PARAM_CELL_WIDTH_THEME    17   /* " theme:XXXXXXXX "  */

/* hud_paint_param_cell — paint one labelled cell at row 1 with the
 * project HUD pair, optionally bold.  Returns the printed-width
 * delta the caller advances cursor_x by. */
static void hud_paint_param_cell_bold(int cursor_x, const char *fmt, const char *value)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, cursor_x, fmt, value);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_param — row-1 HUD: three side-by-side cells.
 *
 *   STEP 1 — PRESET name cell (bold, primary)
 *   STEP 2 — THEME name cell  (bold, primary)
 *   STEP 3 — γ / ω / trail-fill cell (non-bold, secondary metrics)
 */
static void hud_param(const Scene *s)
{
    int cursor_x = HUD_LEFT_MARGIN;

    /* STEP 1 — preset */
    hud_paint_param_cell_bold(cursor_x, " preset:%-8s ",
                              preset_state_active(&s->preset)->name);
    cursor_x += HUD_PARAM_CELL_WIDTH_PRESET;

    /* STEP 2 — theme */
    hud_paint_param_cell_bold(cursor_x, " theme:%-8s ",
                              palette_state_active(&s->palette)->name);
    cursor_x += HUD_PARAM_CELL_WIDTH_THEME;

    /* STEP 3 — γ, ω, trail fill (secondary, non-bold) */
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cursor_x, " γ:%.2f ω:%.2f  trail:%d/%d ",
             (double)s->duffing.system.gamma,
             (double)s->duffing.system.omega,
             s->trail.count, TRAIL_MAX);
    attroff(COLOR_PAIR(PAIR_HUD));
}

static void hud_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:preset  t/T:theme  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_paint(s, sc->cols, sc->rows);
    hud_top(sc->cols, fps, sim_fps, s);
    hud_param(s);
    hud_hint(sc->rows);
}

/* ===================================================================== */
/* §10 app                                                                */
/* ===================================================================== */

/*
 * App — top-level composition root.
 *
 * INTENT
 *   Owns the Scene (mutable simulation state), the Screen (terminal
 *   adapter), the sim tick rate (sim_fps), the chosen map size, and
 *   the two volatile flags written from signal handlers.  Everything
 *   else in the program is reached through this struct — there is no
 *   other mutable global state.  This makes the program read top-
 *   down: main() = "drive the App"; each helper = "do one named thing
 *   to the App".
 *
 *   Concentrating mutable state in one struct also disciplines the
 *   signal-handling story: the SIGWINCH and SIGINT handlers may only
 *   touch volatile sig_atomic_t fields on g_app — every other piece
 *   of state is reachable only through main-loop functions.
 *
 * CONTEXT
 *   Exactly ONE instance exists: the file-scope g_app (immediately
 *   below this struct).  Signal handlers need a callable-from-signal
 *   target, and POSIX async-signal-safety requires that to be a
 *   global; hence g_app rather than a stack-local App.  All non-
 *   signal accesses pass an App* explicitly so the rest of the file
 *   remains free of global lookups.
 *
 *   Lifetime:
 *     app_bootstrap    → fills scene + screen + sim_fps + map dims
 *     main loop        → mutates everything except 'running', which
 *                        is read in the while-condition
 *     atexit(cleanup)  → endwin() safety net
 *
 * MEMBER LOGIC
 *   scene       : §8 Scene — duffing, trail, preset, palette, paused.
 *                 The simulation; "what the user sees".
 *   screen      : §9 Screen — cols / rows + ncurses lifecycle handle.
 *                 The output device.
 *   sim_fps     : current PHYSICS tick rate in Hz.  Distinct from
 *                 RENDER_FPS_TARGET — the integrator can step at any
 *                 rate while the renderer stays locked at 60 fps.
 *                 Adjusted by ] / [.  Clamped to [SIM_FPS_MIN, MAX].
 *   map_w       : cells of horizontal SIMULATION area.
 *   map_h       : cells of vertical SIMULATION area (= screen.rows -
 *                 HUD_BAND_RESERVED_ROWS).  Cached on App so
 *                 scene_paint has stable inputs across a frame;
 *                 recomputed by app_pick_map_size on resize.
 *   running     : 0 → main loop exits.  Set by SIGINT/SIGTERM handler
 *                 and by app_handle_key on q/ESC.  Declared
 *                 volatile sig_atomic_t for async-signal safety per
 *                 POSIX.1-2017 §2.4.3.
 *   need_resize : 1 → next frame triggers a SIGWINCH rebuild via
 *                 app_handle_pending_resize.  Set by the SIGWINCH
 *                 handler.  Same volatile sig_atomic_t requirement.
 *
 * REFERENCES (numbered in file-header References block)
 *   [9]  Fiedler "Fix Your Timestep!" — explains why sim_fps and
 *        RENDER_FPS_TARGET are kept as INDEPENDENT knobs on App:
 *        physics determinism vs render smoothness are orthogonal.
 *   [10] Gookin "Programmer's Guide to NCurses" Ch. 11 (Signal
 *        Handling) — the .need_resize + app_handle_pending_resize
 *        pattern.
 *   • POSIX.1-2017 §2.4.3 — async-signal safety rules that justify
 *     the volatile sig_atomic_t typing of running / need_resize.
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

/* main_install_signal_handlers — wire SIGINT/TERM → quit and
 * SIGWINCH → "resize on next frame".  Called once before the main loop. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* app_pick_map_size — clamp terminal dims minus the HUD reservation
 * into the sensible cell-count envelope. */
static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS;
    if (mw < 16)        mw = 16;
    if (mh < 8)         mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

/* app_bootstrap — first-time initialisation: RNG, sim rate, ncurses,
 * map sizing, scene build. */
static void app_bootstrap(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);
}

/* app_handle_pending_resize — rebuild screen + scene on SIGWINCH. */
static void app_handle_pending_resize(App *app)
{
    if (!app->need_resize) return;
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_resize(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* app_compute_frame_dt — wall-clock since last frame, capped to
 * SIM_MAX_FRAME_DT_MS so a paused-then-resumed terminal can't
 * dump a huge backlog into the integrator. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > SIM_MAX_FRAME_DT_MS * NS_PER_MS)
        dt = SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    return dt;
}

/* app_drain_fixed_timestep — Fiedler's accumulator pattern: pull
 * FIXED-size physics steps until what's left is less than one tick.
 *
 * See header ref [9] Fiedler "Fix Your Timestep!" for the canonical
 * writeup.  Variable-dt would make the chaos NON-DETERMINISTIC —
 * different render speeds would produce different bob trajectories
 * because the cos(ωt) sample times would shift.  app_compute_frame_dt
 * caps dt to prevent the "spiral of death" where one slow frame
 * queues up more substeps than the next frame can process. */
static void app_drain_fixed_timestep(App *app, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* app_update_fps_meter — refresh the displayed fps every
 * FPS_UPDATE_MS milliseconds. */
static void app_update_fps_meter(int64_t *fps_accum, int *frame_count,
                                 double *fps_display)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_display = (double)*frame_count
                 / ((double)*fps_accum / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
}

/* app_throttle_to_render_target — sleep just long enough that the
 * render loop runs at RENDER_FPS_TARGET.  Sleep BEFORE drawing so
 * I/O time doesn't add jitter on top of the budget. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* app_present_frame — paint scene + HUD into the ncurses back buffer
 * and flip.  All ncurses output is funnelled through here. */
static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* Clamped sim-rate mutators */
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

/* One-line mutators — named so the binding table reads as intentions */
static void app_toggle_pause      (App *app) { app->scene.paused = !app->scene.paused; }
static void app_reset_oscillator  (App *app) { scene_reset(&app->scene); }

static void app_cycle_preset_next(App *app)
{
    preset_state_cycle_next(&app->scene.preset);
    scene_reset(&app->scene);
}
static void app_cycle_preset_prev(App *app)
{
    preset_state_cycle_prev(&app->scene.preset);
    scene_reset(&app->scene);
}

static void app_cycle_theme_next(App *app)
{
    palette_state_cycle_next(&app->scene.palette);
    scene_apply_theme(&app->scene);
}
static void app_cycle_theme_prev(App *app)
{
    palette_state_cycle_prev(&app->scene.palette);
    scene_apply_theme(&app->scene);
}

static bool app_handle_key(App *app, int ch);

/* app_poll_keyboard — non-blocking getch + dispatch.  false = quit. */
static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* app_handle_key — key-binding table.  Each case calls ONE named
 * mutator from above. */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            app_toggle_pause     (app); break;
    case 'r': case 'R':  app_reset_oscillator (app); break;
    case ']':            app_sim_rate_faster  (app); break;
    case '[':            app_sim_rate_slower  (app); break;
    case 't':            app_cycle_theme_next (app); break;
    case 'T':            app_cycle_theme_prev (app); break;
    case 'n': case 'N':  app_cycle_preset_next(app); break;
    case 'p': case 'P':  app_cycle_preset_prev(app); break;
    default: break;
    }
    return true;
}

/*
 * FrameClock — the wall-clock + accumulator state the main loop
 * threads through its per-frame helpers.
 *
 * INTENT
 *   Bundle the 5 timing locals that main() would otherwise carry
 *   loose (frame_time, sim_accum, fps_accum, frame_count, fps_display)
 *   into one named concept.  Mirrors the App / Scene pattern: every
 *   piece of mutable state belongs to a NAMED OBJECT, not to main's
 *   stack frame.  The main loop becomes a 10-line pseudocode driver
 *   over FrameClock + App.
 *
 *   The clock separates TWO independent timescales:
 *     • a wall-clock RENDER timeline (frame_time, fps_*) for what
 *       the user sees,
 *     • a PHYSICS accumulator (sim_accum) for the fixed-step Fiedler
 *       drain inside the loop.
 *   Both update from the same dt sample each frame; the helpers below
 *   advance them coherently.
 *
 * CONTEXT
 *   One instance lives on main()'s stack.  Lifetime:
 *     frame_clock_init                 — once before the loop
 *     frame_clock_reset_after_resize   — after each SIGWINCH rebuild
 *     frame_clock_advance              — once per frame
 *     app_drain_fixed_timestep         — reads / mutates .sim_accum
 *     app_update_fps_meter             — reads / mutates .fps_*
 *     app_throttle_to_render_target    — reads .frame_time
 *     app_present_frame                — reads .fps_display
 *
 * MEMBER LOGIC
 *   frame_time  : wall-clock timestamp (ns) at the boundary of the
 *                 LAST frame.  app_compute_frame_dt subtracts the
 *                 current clock from this to get dt.
 *   sim_accum   : leftover ns NOT YET consumed by a physics tick.
 *                 The Fiedler fixed-step accumulator (header ref [9]).
 *   fps_accum   : ns of frame time accumulated since the last fps
 *                 readout refresh.
 *   frame_count : number of frames in the current fps window.
 *   fps_display : most recent fps figure, smoothed by the
 *                 FPS_UPDATE_MS window length.  Displayed in the
 *                 top-right HUD.
 *
 * REFERENCES (numbered in file-header References block)
 *   [9] Fiedler "Fix Your Timestep!" — the accumulator pattern that
 *       .sim_accum implements.
 */
typedef struct {
    int64_t frame_time;
    int64_t sim_accum;
    int64_t fps_accum;
    int     frame_count;
    double  fps_display;
} FrameClock;

static void frame_clock_init(FrameClock *c)
{
    c->frame_time  = clock_ns();
    c->sim_accum   = 0;
    c->fps_accum   = 0;
    c->frame_count = 0;
    c->fps_display = 0.0;
}

static void frame_clock_reset_after_resize(FrameClock *c)
{
    c->frame_time = clock_ns();
    c->sim_accum  = 0;
}

static void frame_clock_advance(FrameClock *c, int64_t dt)
{
    c->sim_accum += dt;
    c->fps_accum += dt;
    c->frame_count++;
}

/* main — the whole simulation as a pseudocode driver.
 *
 *   install signal handlers
 *   bootstrap (curses + scene)
 *   init frame clock
 *   while running:
 *     if pending resize → rebuild screen, reset clock
 *     dt ← wall-clock since last frame
 *     advance clock; drain fixed-timestep physics; update fps meter
 *     throttle to render target; present frame
 *     poll keyboard (may flip running)
 *   tear down
 */
int main(void)
{
    main_install_signal_handlers();

    App *app = &g_app;
    app_bootstrap(app);

    FrameClock clk;
    frame_clock_init(&clk);

    while (app->running) {
        if (app->need_resize) {
            app_handle_pending_resize(app);
            frame_clock_reset_after_resize(&clk);
        }

        int64_t dt = app_compute_frame_dt(&clk.frame_time);
        frame_clock_advance(&clk, dt);
        app_drain_fixed_timestep(app, &clk.sim_accum);
        app_update_fps_meter(&clk.fps_accum, &clk.frame_count, &clk.fps_display);

        app_throttle_to_render_target(clk.frame_time, dt);
        app_present_frame(app, clk.fps_display);

        if (!app_poll_keyboard(app)) app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
