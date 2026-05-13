/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fire.c — three fire-simulation algorithms rendered into ASCII glyphs.
 *
 * DEMO
 * ────
 * A live flame fills the terminal.  Three completely different physics
 * engines take turns producing it: a cellular automaton, a pool of
 * Lagrangian particles, and a procedural plasma defined by three sine
 * harmonics.  Whichever engine fills the grid, the renderer is the
 * same — gamma-correct the heat values, dither them down with
 * Floyd-Steinberg error diffusion, and look up one of nine ramp
 * glyphs per cell.  Six tinted palettes recolour the flame without
 * touching the physics.  Wind shears the source arch.  Fuel intensity
 * raises or lowers the average flame top.
 *
 * Study alongside:
 *   particle_systems/particles.c       — broader particle-system patterns
 *   grids/rect_grids/01_uniform_rect.c — canonical Phase 3 reference file
 *
 * Section map
 * ───────────
 *   §1  includes        — system headers only (no project locals)
 *   §2  loop presets    — sim fps min/max, hud column width
 *   §3  source presets  — arch margin, fuel jitter, warmup ticks
 *   §4  ca presets      — Doom-CA decay tuning
 *   §5  particle presets— life, velocity, spawn, damping
 *   §6  plasma presets  — three sine harmonics
 *   §7  monotonic clock — clock_ns + sleep
 *   §8  ramp + lut      — 9 glyphs and their intensity thresholds
 *   §9  theme palettes  — six colour ramps (256-colour + 8-colour fall-back)
 *   §10 colour setup    — init_pair calls, HUD reserved pairs
 *   §11 grid storage    — Grid, FirePart, allocation, resize
 *   §12 shared helpers  — clampf, warmup, wind, arch, fuel-row seed, splat
 *   §13 algo 0  CA      — Doom-style cellular automaton
 *   §14 algo 1  particle— Lagrangian fire (FirePart pool + 3×3 Gaussian splat)
 *   §15 algo 2  plasma  — procedural fire from three sine harmonics
 *   §16 dispatch        — grid_tick switch on algo index
 *   §17 render pipeline — gamma → Floyd-Steinberg dither → ramp glyph
 *   §18 debug overlays  — d / D cycle through four diagnostic modes
 *   §19 scene           — owns grid + pause + needs_clear flag
 *   §20 screen + hud    — ncurses init, hud, key hint
 *   §21 application     — keys, signal handlers, main loop
 *
 * Keys
 * ────
 *   q / ESC   quit
 *   space     pause / resume
 *   a         next algorithm (CA → particle → plasma → CA)
 *   t         next theme (cycles six palettes)
 *   g / G     fuel intensity up / down
 *   w / W     wind right / left
 *   0         calm (no wind)
 *   [ / ]     sim Hz down / up
 *   d / D     next / previous debug overlay
 *
 * Build
 * ─────
 *   gcc -std=c11 -O2 -Wall -Wextra fire.c -o fire -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * READING ORDER
 *   1. Read CONCEPTS for a one-paragraph answer per algorithm.
 *   2. Read MENTAL MODEL for the unified picture: three sources, one
 *      heat field, one renderer.  An ASCII diagram shows the data flow.
 *   3. Walk GUIDED TUTORIAL §1..§10 in order.  Each tutorial starts
 *      with a question, answers it in plain English, then sketches
 *      pseudocode.  By tutorial 10 you should be able to reimplement
 *      the file blindfolded.
 *   4. Now read code §1..§21.  Every section opens with a preamble
 *      explaining what it adds to the pipeline, and every non-trivial
 *      function carries Purpose / Pseudocode / Mental model / I/O
 *      blocks above its signature.
 *
 * NAMING CONVENTION
 *   Long descriptive names everywhere — `warmup_scale_factor`, not `ws`.
 *   Inside tight loops (≤ 5 lines, single concept) one-letter
 *   coordinates like `x` and `y` are acceptable because the surrounding
 *   context is one glance away.  All tunable constants use
 *   SCREAMING_SNAKE_CASE with a units comment on the same line.
 *
 * REQUIRED BACKGROUND
 *   • Basic C (structs, function pointers, calloc).
 *   • ncurses fundamentals (cell-based output, colour pairs, attributes).
 *   • Cartesian grids — the file treats screen as a (cols × rows) array
 *     where y = 0 is the top row and y = rows − 1 is the bottom row.
 *   No calculus is needed; the only trig used is sin().  Gamma
 *   correction and dithering are introduced from scratch in
 *   GUIDED TUTORIAL §8..§9.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm
 * ─────────
 * Three independent fire simulations share one output buffer:
 *   • Doom-style cellular automaton — bottom row holds maximum heat
 *     fuel; every cell above pulls its value from the row below with
 *     a lateral jitter of ±1 cell and a random per-cell decay.  Heat
 *     rises by diffusion; the flame top emerges where decay accumulates
 *     to equal the source value.
 *   • Lagrangian particles        — a fixed pool of FirePart structs.
 *     Each particle is born at the source arch with an upward velocity,
 *     rises, fades, and on render-time stamps a 3×3 Gaussian splat of
 *     heat onto the grid centred at its current position.
 *   • Procedural plasma           — no carry-over state between frames.
 *     For every column we compute a tongue height as the sum of three
 *     sine harmonics whose phase drifts in time, then shade cells below
 *     the tongue tip with a smooth gradient.
 *
 * Data structure
 * ──────────────
 * Grid::heat is a flat (cols × rows) float array in [0, MAX_HEAT].
 * One cell = one float = one temperature reading.  Grid::prev_heat
 * tracks the previous frame so cells that *were* lit but went cold
 * can be explicitly erased.  Grid::dither is a scratch buffer for
 * Floyd-Steinberg quantisation.  The particle pool is a fixed array
 * of MAX_FIRE_PARTS FirePart structs so the hot path performs no
 * runtime allocation.
 *
 * Rendering
 * ─────────
 * heat[i] → pow(heat[i], 1/2.2)             (perceptual gamma)
 *        → Floyd-Steinberg error diffusion  (visual smoothing)
 *        → 9-step LUT bucket index          (quantisation)
 *        → glyph from " .:+x*X#@"           (ramp character)
 *        → ncurses attribute = colour pair × bold/normal
 * Cells that are cold this frame but were lit last frame receive a
 * literal ' ' to wipe the stale character; cells that have been cold
 * for ≥ 2 frames are skipped entirely so the ncurses diff write stays
 * tiny.
 *
 * Performance
 * ───────────
 * • Allocation only at startup / resize.  Hot path is malloc-free.
 * • CA and plasma are O(cols × rows) per tick.  Particles add
 *   O(MAX_FIRE_PARTS) per tick — 800 entries is well under any sane
 *   terminal cell budget.
 * • Render runs at 60 fps; simulation runs at its own configurable
 *   Hz via a fixed-timestep accumulator.
 *
 * References
 * ──────────
 *   • Sanglard, Fabien.  "How DOOM Fire Was Done."  fabiensanglard.net,
 *     2014.  Reverse-engineering of the original 1993 CA algorithm.
 *   • Floyd, R. W. & Steinberg, L.  "An adaptive algorithm for spatial
 *     gray scale."  Proceedings of the SID, vol 17/2, 1976.  The
 *     canonical error-diffusion dithering paper.
 *   • Sims, Karl.  "Particle Animation and Rendering Using Data
 *     Parallel Computation."  Computer Graphics (SIGGRAPH), 1990.
 *   • Poynton, Charles.  Digital Video and HD: Algorithms and
 *     Interfaces, ch 24 "Gamma".  Morgan Kaufmann, 2003.
 *   • Quilez, Inigo.  "smoothstep" and related articles.
 *     iquilezles.org/articles.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Treat the terminal as a grid of thermometers.  Each tick, one of
 * three heaters writes a fresh temperature into every cell.  A single
 * renderer — shared by all three heaters — converts the temperature
 * grid into ASCII glyphs.  Because the renderer is shared, swapping
 * heaters is a one-line switch.  Everything you read in this file is
 * either a heater that fills the temperature grid or a renderer that
 * consumes it.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine an oven with the lid open.  Inside, thermometers cover every
 * grid cell.  Beneath the oven the cook can plug in one of three heat
 * sources.  Whichever source is plugged in, the thermometers all read
 * a fresh value each tick.  A photographer in the back room reads the
 * temperatures, converts them to glyphs, and prints the image:
 *
 *   +--------------- terminal cells ---------------+
 *   |  '.'  ' '  ' '  ' '  '.'  ' '  ' '  ' '      |   <- cool tops
 *   |  '+'  '.'  '.'  ':'  '+'  ':'  '.'  ' '      |
 *   |  'x'  '*'  '+'  'X'  '*'  '+'  '.'  ' '      |
 *   |  '#'  'X'  '*'  '@'  'X'  '*'  '+'  '.'      |   <- hot body
 *   |  '@'  '#'  'X'  '@'  '#'  'X'  '*'  '+'      |   <- fuel arch
 *   +-----^------------^-------------^-------------+
 *         |            |             |
 *   +------------+ +------------+ +------------+
 *   |  CA logic  | |  particle  | |   plasma   |
 *   |   (Doom)   | |  splatter  | |  sin wave  |
 *   +------------+ +------------+ +------------+
 *           Only one source is plugged in at a time.
 *           Thermometers + photographer never change.
 *
 * DRAWING METHOD  /  ALGORITHM IN STEPS
 * ─────────────────────────────────────
 *   Per-tick (one of three heaters; selected by Grid::algo):
 *     CA      : seed bottom row with arch-shaped fuel; for each cell
 *               from row 1 upward, pull heat from (y+1, x ± jitter)
 *               and subtract a random decay.
 *     Particle: advance every active particle (turbulence, velocity,
 *               heat decay); spawn new particles at the source arch;
 *               clear heat grid; for each active particle 3×3 splat
 *               its current heat onto the grid.
 *     Plasma  : advance plasma_t; for each column compute a tongue
 *               height = base + sum of three sine harmonics times
 *               fuel × warmup × sqrt(arch); for each row shade the
 *               cell from 1.0 at the base to 0 at the tongue tip.
 *
 *   Per-frame (shared by all heaters; runs in grid_draw):
 *     1. For every cell, gamma-correct: v = pow(heat / MAX_HEAT, 1/2.2).
 *     2. For every cell, Floyd-Steinberg quantise: bucket = lut_index(v);
 *        error = v − bucket_midpoint; push error onto four neighbours.
 *     3. Draw glyph k_ramp[bucket] with theme colour attribute.
 *     4. For every cell that was lit last frame but is cold now, draw
 *        a literal ' ' to erase it.
 *     5. Swap heat ↔ prev_heat so next frame can detect what changed.
 *
 * KEY FORMULAS
 * ────────────
 *   CA propagation:
 *     heat[y][x] = max(0, heat[y+1][x + jitter] − decay)
 *     decay      = MAX_HEAT / (rows × CA_REACH_FRAC) × (BASE + rand×RAND)
 *     so the expected flame top sits CA_REACH_FRAC of the way up.
 *
 *   Arch envelope (shared by every heater for the source):
 *     t    = (x − margin − wind_acc) / (cols − 2 × margin)   ∈ [0, 1]
 *     edge = min(t, 1 − t) × 2                               ∈ [0, 1]
 *     arch = edge²                                           peaked centre
 *
 *   3×3 Gaussian splat kernel (sum = 1):
 *       0.0625   0.125   0.0625
 *       0.125    0.25    0.125
 *       0.0625   0.125   0.0625
 *
 *   Plasma tongue:
 *     tongue(x, t) = 0.50
 *                  + 0.28 × sin(wx × 5  + t × 2.2)
 *                  + 0.18 × sin(wx × 11 − t × 1.6)
 *                  + 0.10 × sin(wx × 3  + t × 0.7)
 *     heat(x, y)   = clamp((y/rows − (1 − tongue)) / tongue, 0, 1)
 *
 *   Gamma + Floyd-Steinberg:
 *     v_perceptual = pow(v_linear, 1/2.2)
 *     error        = v_perceptual − lut_midpoint(bucket)
 *     error × 7/16 → right       error × 3/16 → down-left
 *     error × 5/16 → down        error × 1/16 → down-right
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Tiny terminals (rows < 5): the CA decay computed from height
 *     collapses to near zero — heat would never die out.  CA_DECAY_*_MIN
 *     floors the constants so the visual remains sensible on a 24-line
 *     window.
 *   • Particle pool saturation: at full warmup the spawn loop may
 *     exhaust the 800-entry pool.  The spawner walks up to
 *     MAX_FIRE_PARTS round-robin slots before giving up cleanly rather
 *     than overwriting active embers.
 *   • Plasma tongue overshoot: the three harmonics + base can sum to
 *     1.06 in the worst case (0.50 + 0.28 + 0.18 + 0.10).  The
 *     clampf(tongue, 0, 1) call inside the column loop keeps the
 *     per-row gradient sane.
 *   • Floyd-Steinberg leaking into cold cells: cold cells are tagged
 *     dither_buffer[i] = −1.  Each diffusion step guards `≥ 0` before
 *     adding, otherwise stray error would make cold cells flicker.
 *   • Wind wrap: wind_acc resets to 0 once |wind_acc| ≥ cols so the
 *     arch never drifts off-screen permanently.  At max wind ±3 the
 *     flame visibly snaps when it crosses the screen edge — acceptable
 *     in a demo, would need ring-buffer smoothing for production.
 *   • Algorithm switch: the renderer's previous frame is stale and
 *     glyphs from the old algorithm would linger.  Scene::needs_clear
 *     forces a single erase() on the next frame after 'a' or 't'.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Press 'a' while paused: the glyph layout changes shape entirely,
 *     yet the theme colour and ramp characters stay identical — proof
 *     that the renderer is decoupled from the heater.
 *   • Hold 'G' to drop fuel toward 0.10: the flame shrinks to ankle
 *     height within ~3 seconds.  Hold 'g' back up to 1.00: the flame
 *     regrows to roughly 75 % of the screen.
 *   • Press 'w' to add right wind: the arch base leans right and the
 *     entire flame slants in the same direction; '0' resets to vertical.
 *     At max wind ±3 the lean is visible within 5 ticks.
 *   • Press 'd': overlay 1 shows raw linear heat (no gamma, no dither) —
 *     coarse banding visible; overlay 2 shows gamma-corrected heat with
 *     no dither — banding shifted toward the highlights; overlay 3
 *     shows the arch envelope alone — confirms the source distribution.
 *   • Press 't' to cycle themes: the ramp glyph layout stays identical;
 *     only the colour tint shifts.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 * Ten short lessons that build the file from first principles.  Read in
 * order; each lesson assumes the previous one.
 *
 * TUTORIAL 1 — Why a heat field?
 * ──────────────────────────────
 * Fire on a CRT looks continuous, but a terminal can only paint cells.
 * If we tried to draw fire as moving glyphs we would get sparse sparks,
 * not the dense glowing body we expect.  The fix: store a real-valued
 * temperature per cell, then map each cell's temperature to a glyph
 * that resembles its intensity.  Temperatures live in [0, 1] where 0
 * is cold (space) and 1 is white-hot ('@').  Once we have such a heat
 * field, any physics that fills it works.
 *
 * TUTORIAL 2 — Three heaters, one output.
 * ───────────────────────────────────────
 * We provide three completely different ways to fill the heat field:
 *   • a cellular automaton modelled on the original Doom fire,
 *   • a pool of rising particles that splat heat onto the grid,
 *   • a procedural plasma defined by three sine harmonics.
 * The renderer below never inspects which heater filled the field; it
 * only reads cell heat values.  Why bother?  Because each heater
 * teaches a different physics paradigm — Eulerian grid, Lagrangian
 * particles, closed-form procedural — on a shared canvas.  Reading
 * all three side-by-side is the easiest way to feel the difference.
 *
 * TUTORIAL 3 — How does cellular-automaton fire work?
 * ───────────────────────────────────────────────────
 * Fabien Sanglard's reverse-engineering of Doom revealed a rule so
 * simple it fits in two lines:
 *     pulled_value  = heat[y + 1][x + random_jitter(-1, 0, +1)]
 *     heat[y][x]    = max(0, pulled_value − random_decay)
 * Apply this to every cell from bottom to top, with the bottom row
 * held permanently at maximum fuel.  Heat naturally flows upward
 * because each cell copies from the row below.  Each step loses a
 * little energy to the decay term, so by some height the heat reaches
 * zero — and that's where the flame top emerges.  The lateral jitter
 * spreads heat sideways and produces the characteristic licking
 * spires.
 *
 * TUTORIAL 4 — Why the lateral jitter?
 * ────────────────────────────────────
 * Drop the jitter (use x instead of x + rand{−1, 0, +1}) and the
 * flame becomes a set of vertical stripes — every column rises
 * straight up with no mixing.  Add the jitter and adjacent columns
 * exchange heat, which is what gives real flames their twisting
 * motion.  The "right amount" of jitter is exactly ±1 cell per step:
 * enough to mix neighbours, not enough to teleport heat across the
 * screen.
 *
 * TUTORIAL 5 — Where do particles fit?
 * ────────────────────────────────────
 * CA is Eulerian: the grid stays fixed, fluid quantities flow through
 * it.  Particles are Lagrangian: tiny tracers carry their own state
 * and move through space.  For fire, particles let us model individual
 * embers — each rises with its own velocity, decays independently,
 * and feels "turbulence" as random sideways nudges.  When it is time
 * to render, each particle stamps a 3×3 Gaussian blob of heat onto
 * the grid wherever it is.  Sum many particles and you get a dense
 * flame body.
 *
 * TUTORIAL 6 — The 3×3 Gaussian splat.
 * ────────────────────────────────────
 * A single particle that painted only its own cell would produce a
 * thin sparkly flame, not a filled body.  Instead, each particle
 * deposits its heat over a 3×3 neighbourhood using a discrete
 * Gaussian kernel:
 *     1/16   2/16   1/16        =       0.0625   0.125   0.0625
 *     2/16   4/16   2/16                0.125    0.25    0.125
 *     1/16   2/16   1/16                0.0625   0.125   0.0625
 * The kernel sums to 1, which means total deposited energy equals the
 * particle's heat — splatting is just smearing, not amplifying.  The
 * centre cell receives the most, the corners the least, which matches
 * what one would expect from a Gaussian point-spread.
 *
 * TUTORIAL 7 — Plasma without state.
 * ──────────────────────────────────
 * The plasma algorithm is the odd one out: it carries no information
 * between frames.  For every column x we compute a "tongue height":
 *     tongue(x, t) = base + a1 × sin(b1 × x + c1 × t)
 *                         + a2 × sin(b2 × x − c2 × t)
 *                         + a3 × sin(b3 × x + c3 × t)
 * then for every row y we shade cells below the tongue tip with a
 * smooth gradient from hot (base) to cold (tip).  The three sines
 * run at different spatial frequencies (5, 11, 3 cycles across the
 * screen) and different time speeds (+2.2, −1.6, +0.7), so the
 * tongues never repeat exactly and look organic.
 *
 * TUTORIAL 8 — Why gamma correction?
 * ──────────────────────────────────
 * Human eyes don't perceive brightness linearly: a heat value of 0.5
 * does *not* look halfway between 0 and 1.  We perceive luminance
 * roughly as the 2.2 power of physical intensity.  To make the ramp
 * progression look uniform, we apply pow(v, 1/2.2) before quantising.
 * Without gamma correction, the bottom of the ramp would look almost
 * identical between adjacent buckets, while the top would look like a
 * staircase.  Toggling the debug overlay shows the difference.
 *
 * TUTORIAL 9 — Floyd-Steinberg dithering.
 * ───────────────────────────────────────
 * With only nine ramp glyphs we cannot directly represent the
 * hundreds of distinct intensities present in a continuous heat
 * field.  Naive nearest-neighbour quantisation produces obvious
 * banding.  Dithering trades spatial detail for tonal detail: every
 * time we round a value to a ramp bucket, we measure the error
 * (rounded value − exact value) and push that error onto neighbouring
 * cells before they are quantised.  The neighbours absorb the error
 * and produce compensating high/low buckets, which the eye averages
 * back into a smooth gradient.  Floyd-Steinberg's weights — 7/16
 * right, 3/16 down-left, 5/16 down, 1/16 down-right — are the
 * canonical four-neighbour error distribution.
 *
 * TUTORIAL 10 — The borderless redraw trick.
 * ──────────────────────────────────────────
 * Calling erase() every frame would write a space to every cell, and
 * ncurses' diff would conclude that every cell changed.  Instead we
 * keep prev_heat alongside heat.  On the next frame:
 *   • If current and previous are both 0 → skip (no write).
 *   • If current > 0                    → draw the appropriate glyph.
 *   • If current == 0 but previous > 0  → write a single ' ' to erase.
 * Result: the only cells written each frame are the few that actually
 * changed, which keeps the ncurses doupdate() diff blazingly small.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ===================================================================== */
/* §1  includes                                                           */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   System headers only.  This file deliberately includes no project-
 *   local header — every dependency must be either part of libc, math,
 *   or ncurses.  The _POSIX_C_SOURCE feature-test macro exposes
 *   clock_gettime() and nanosleep() from <time.h>.
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §2  loop presets                                                       */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   Tunable constants that govern the outer game loop.  The simulation
 *   advances at SIM_FPS_DEFAULT Hz initially; the user can bracket
 *   between SIM_FPS_MIN and SIM_FPS_MAX with [ and ].  Render is
 *   pinned to 60 fps inside main().  HUD_COLS bounds the width of the
 *   right-aligned status string so it never wraps.
 */

enum {
    SIM_FPS_MIN     =  5,    /* lower bound for [/] cycling                */
    SIM_FPS_DEFAULT = 30,    /* initial simulation rate                    */
    SIM_FPS_MAX     = 60,    /* upper bound for [/] cycling                */
    SIM_FPS_STEP    =  5,    /* one press of [/] adjusts by this many Hz   */

    HUD_COLS        = 64,    /* max width of any HUD status string         */
    FPS_UPDATE_MS   = 500,   /* render-fps averaging window                */

    N_ALGOS         =  3,    /* CA, Particle, Plasma                       */
    N_DEBUG_MODES   =  4,    /* off, raw heat, gamma-only, arch envelope   */
    MAX_FIRE_PARTS  = 800,   /* particle pool size                         */
};

#define MAX_HEAT    1.0f    /* heat-grid ceiling; cells live in [0, MAX_HEAT] */
#define WIND_MAX    3       /* maximum |wind| in cells / tick                 */

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(hz)     (NS_PER_SEC / (hz))

/* ===================================================================== */
/* §3  source presets                                                     */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   Constants that shape the fuel source — the arch along the bottom
 *   row from which every algorithm draws heat.  ARCH_MARGIN_FRAC keeps
 *   the leftmost and rightmost ARCH_MARGIN_FRAC × cols cells cold,
 *   FUEL_JITTER_* adds per-cell randomness so the source line flickers,
 *   and WARMUP_TICKS ramps fuel from 0 to 1 over the first ~2.7 seconds
 *   of life so the flame builds gradually rather than popping into
 *   existence.
 */

#define ARCH_MARGIN_FRAC   0.04f   /* 4 % of cols kept cold at each side       */
#define FUEL_JITTER_BASE   0.82f   /* min random multiplier on fuel             */
#define FUEL_JITTER_RANGE  0.18f   /* additional random range 0 → 0.18          */
#define WARMUP_TICKS       80      /* linear 0 → 1 over the first 80 ticks      */

/* ===================================================================== */
/* §4  ca presets                                                         */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   Constants for algorithm 0 (Doom-style cellular automaton).
 *   CA_REACH_FRAC sets the target average flame height as a fraction of
 *   terminal rows; the per-tick decay is derived from this so the
 *   visual stays consistent across resolutions.  The _MIN floors keep
 *   tiny terminals (rows < 5) from collapsing the decay to zero.
 */

#define CA_REACH_FRAC      0.75f   /* expected flame peak at 75 % of rows       */
#define CA_DECAY_BASE_FRAC 0.55f   /* base decay   = avg_decay × this            */
#define CA_DECAY_RAND_FRAC 0.90f   /* random range = avg_decay × this            */
#define CA_DECAY_BASE_MIN  0.010f  /* floor for tiny terminals                   */
#define CA_DECAY_RAND_MIN  0.015f  /* floor for tiny terminals                   */

/* ===================================================================== */
/* §5  particle presets                                                   */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   Constants for algorithm 1 (Lagrangian particles).  PART_LIFE_*
 *   control how long each ember lives, PART_VY_* the initial upward
 *   thrust, PART_VX_SPREAD a tiny lateral kick at birth, PART_TURB_STEP
 *   a per-tick random nudge that produces flicker, and PART_VX_DAMP a
 *   damping factor so accumulated lateral drift decays.  SPAWN_PER_TICK
 *   is the equilibrium birth rate at full warmup.
 */

#define PART_LIFE_MIN      15.f    /* minimum lifetime, in ticks                 */
#define PART_LIFE_RANGE    20.f    /* uniform random range added on top          */
#define PART_VY_BASE        0.5f   /* minimum upward speed (cells / tick)        */
#define PART_VY_RANGE       0.8f   /* uniform random extra upward speed          */
#define PART_VX_SPREAD      0.5f   /* width of birth lateral kick (±SPREAD/2)    */
#define PART_TURB_STEP      0.15f  /* per-tick random nudge on vx                */
#define PART_VX_DAMP        0.96f  /* lateral damping per tick                   */
#define SPAWN_PER_TICK      20     /* equilibrium birth rate at full warmup      */

/* ===================================================================== */
/* §6  plasma presets                                                     */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   Constants for algorithm 2 (procedural plasma).  PLASMA_TIME_STEP is
 *   the per-tick advance of the internal phase counter.  PLASMA_BASE
 *   is the DC offset so the tongue is always positive.  H1/H2/H3 each
 *   contribute one sine harmonic.  XFREQ counts cycles across the
 *   screen, TSPD counts radians-per-tick of phase drift.
 */

#define PLASMA_TIME_STEP   0.07f
#define PLASMA_BASE        0.50f
#define PLASMA_H1_AMP      0.28f
#define PLASMA_H1_XFREQ    5.0f
#define PLASMA_H1_TSPD     2.2f
#define PLASMA_H2_AMP      0.18f
#define PLASMA_H2_XFREQ   11.0f
#define PLASMA_H2_TSPD     1.6f
#define PLASMA_H3_AMP      0.10f
#define PLASMA_H3_XFREQ    3.0f
#define PLASMA_H3_TSPD     0.7f

/* ===================================================================== */
/* §7  monotonic clock                                                    */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   Two thin wrappers around CLOCK_MONOTONIC.  clock_ns() returns
 *   wall-clock nanoseconds since an arbitrary epoch; clock_sleep_ns()
 *   suspends the thread for a given duration.  Both are used by the
 *   main loop's fixed-timestep accumulator.
 */

/*
 * clock_ns() — read the monotonic clock in nanoseconds.
 *
 *   Purpose      : single source of time for the fixed-timestep loop.
 *   Pseudocode   : t ← clock_gettime(CLOCK_MONOTONIC); return t.sec * 1e9 + t.nsec
 *   Mental model : a stopwatch that never resets and never jumps even
 *                  when the wall clock is adjusted by NTP.
 *   Inputs       : none
 *   Output       : int64_t nanoseconds since an unspecified epoch
 *   Why it lives here: the simulation must never depend on system time
 *                      changes; CLOCK_MONOTONIC guarantees that.
 */
static int64_t clock_ns(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)now.tv_sec * NS_PER_SEC + now.tv_nsec;
}

/*
 * clock_sleep_ns() — sleep for the given number of nanoseconds.
 *
 *   Purpose      : pace the render loop to 60 fps.
 *   Pseudocode   : if (ns ≤ 0) return; ts ← (ns/1e9, ns mod 1e9); nanosleep(ts)
 *   Mental model : a timed pause; the kernel returns control after at
 *                  least the requested duration has elapsed.
 *   Inputs       : ns — duration in nanoseconds (≤ 0 is a no-op)
 *   Output       : none
 *   Why it lives here: keeps sleep duration encapsulated so the main
 *                      loop reads as one line.
 */
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = { (time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC) };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §8  ramp + lut                                                         */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   The visual ramp.  k_ramp is the nine ASCII glyphs the renderer can
 *   pick from, ordered from cold-empty to hot-core.  k_lut_breaks is
 *   the gamma-corrected intensity threshold above which each glyph
 *   becomes the chosen bucket.  Breakpoints are clustered in the
 *   0.3 – 0.75 range because that is where the eye discriminates flame
 *   curvature most strongly.
 *
 *   Glyph chart:
 *
 *     index   glyph   threshold   meaning
 *       0      ' '      0.000      cold / empty
 *       1      '.'      0.080      faint ember glow
 *       2      ':'      0.180      low flame
 *       3      '+'      0.290      mid-low flame
 *       4      'x'      0.390      mid flame
 *       5      '*'      0.500      mid-high flame
 *       6      'X'      0.620      hot flame
 *       7      '#'      0.750      very hot
 *       8      '@'      0.900      peak core
 */

static const char k_ramp[] = " .:+x*X#@";
#define RAMP_N (int)(sizeof k_ramp - 1)   /* 9 visible glyphs */

static const float k_lut_breaks[RAMP_N] = {
    0.000f,  /* ' '  cold      */
    0.080f,  /* '.'  ember     */
    0.180f,  /* ':'  low       */
    0.290f,  /* '+'  mid-low   */
    0.390f,  /* 'x'  mid       */
    0.500f,  /* '*'  mid-high  */
    0.620f,  /* 'X'  hot       */
    0.750f,  /* '#'  very hot  */
    0.900f,  /* '@'  core      */
};

/*
 * lut_index() — gamma-corrected intensity → ramp bucket [0..RAMP_N-1].
 *
 *   Purpose      : pick the right glyph for a given perceptual heat.
 *   Pseudocode   : scan thresholds from top to bottom;
 *                  return the highest bucket whose threshold ≤ v.
 *   Mental model : staircase function: each step is one glyph wide.
 *   Inputs       : v — gamma-corrected heat in [0, 1]
 *   Output       : int in [0, RAMP_N-1]
 *   Why it lives here: the only spot that maps continuous heat to
 *                      discrete glyphs; centralises the staircase.
 */
static int lut_index(float v)
{
    int bucket = 0;
    for (int i = RAMP_N - 1; i >= 0; i--)
        if (v >= k_lut_breaks[i]) { bucket = i; break; }
    return bucket;
}

/*
 * lut_midpoint() — representative intensity of bucket idx.
 *
 *   Purpose      : compute the dither error for a quantised cell.
 *   Pseudocode   : if idx == 0       return 0
 *                  if idx == RAMP_N-1 return 1
 *                  else                return (break[idx] + break[idx+1]) / 2
 *   Mental model : "what would v have been if it sat exactly in this
 *                   bucket?"  The error is v − midpoint.
 *   Inputs       : idx — bucket index
 *   Output       : float midpoint intensity in [0, 1]
 *   Why it lives here: Floyd-Steinberg needs a representative value
 *                      per bucket to compute the error to diffuse.
 */
static float lut_midpoint(int idx)
{
    if (idx <= 0)          return 0.f;
    if (idx >= RAMP_N - 1) return 1.f;
    return (k_lut_breaks[idx] + k_lut_breaks[idx + 1]) * 0.5f;
}

/* ===================================================================== */
/* §9  theme palettes                                                     */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   Six fire palettes.  Each palette assigns one foreground colour per
 *   ramp index.  Design constraints:
 *
 *     • The lowest entries (indices 0-2) sit in the visible-but-faint
 *       range so flame wisps stay readable.  No near-black entries
 *       (xterm 232-236) — they vanish on default dark terminals.
 *     • The highest entries (indices 7-8) burn near-white so the flame
 *       core punches through any background.
 *     • The 8-colour fallback uses base ANSI colours plus A_DIM/A_BOLD
 *       attributes to approximate the gradient on minimal terminals.
 */

typedef struct {
    const char *name;
    int         fg256[RAMP_N];   /* xterm 256-colour indices, per bucket */
    int         fg8[RAMP_N];     /* 8-colour fallback                    */
    attr_t      attr8[RAMP_N];   /* fallback attribute (DIM/NORMAL/BOLD) */
} FireTheme;

#define CP_BASE   1                       /* ramp pairs: CP_BASE .. CP_BASE+RAMP_N-1 */
#define PAIR_HUD  (CP_BASE + RAMP_N)      /* bright yellow status (row 0 / row 1)    */
#define PAIR_HINT (CP_BASE + RAMP_N + 1)  /* bright cyan key hint (row rows-1)       */

static const FireTheme k_themes[] = {
    {   /* 0  fire — classic red / orange / yellow */
        "fire",
        {  88, 124, 160, 196, 202, 208, 214, 220, 231 },
        {  COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_RED,
           COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
           COLOR_WHITE  },
        {  A_NORMAL, A_NORMAL, A_BOLD, A_BOLD,
           A_DIM,    A_NORMAL, A_BOLD, A_BOLD, A_BOLD }
    },
    {   /* 1  ice — sky blue / cyan / white */
        "ice",
        {  25, 27, 33, 39, 45, 51, 87, 159, 231 },
        {  COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
           COLOR_CYAN, COLOR_CYAN, COLOR_WHITE, COLOR_WHITE,
           COLOR_WHITE  },
        {  A_NORMAL, A_BOLD, A_NORMAL, A_NORMAL,
           A_BOLD,   A_BOLD, A_BOLD,  A_BOLD, A_BOLD }
    },
    {   /* 2  plasma — violet / magenta / white */
        "plasma",
        {  55, 91, 93, 129, 165, 201, 207, 213, 231 },
        {  COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
           COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE,   COLOR_WHITE,
           COLOR_WHITE    },
        {  A_NORMAL, A_NORMAL, A_BOLD, A_BOLD,
           A_BOLD,   A_BOLD,   A_DIM,  A_NORMAL, A_BOLD }
    },
    {   /* 3  nova — green / lime / white */
        "nova",
        {  28, 34, 40, 46, 82, 118, 154, 190, 231 },
        {  COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
           COLOR_GREEN, COLOR_GREEN, COLOR_WHITE, COLOR_WHITE,
           COLOR_WHITE  },
        {  A_NORMAL, A_BOLD, A_BOLD, A_BOLD,
           A_BOLD,   A_BOLD, A_BOLD, A_BOLD, A_BOLD }
    },
    {   /* 4  poison — olive / yellow-green / white */
        "poison",
        {  58, 64, 70, 76, 118, 154, 184, 220, 231 },
        {  COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_YELLOW,
           COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,
           COLOR_WHITE   },
        {  A_NORMAL, A_NORMAL, A_BOLD, A_BOLD,
           A_BOLD,   A_BOLD,   A_BOLD, A_BOLD, A_BOLD }
    },
    {   /* 5  gold — amber / orange / yellow */
        "gold",
        {  130, 136, 172, 178, 208, 214, 220, 226, 231 },
        {  COLOR_RED,    COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW,
           COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,
           COLOR_WHITE   },
        {  A_NORMAL, A_BOLD, A_NORMAL, A_BOLD,
           A_BOLD,   A_BOLD,  A_BOLD,  A_BOLD, A_BOLD }
    },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

/* ===================================================================== */
/* §10 colour setup                                                       */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   ncurses pair registration.  theme_apply() rewrites the ramp pairs
 *   for the requested theme; it is called from color_init() and again
 *   whenever the user cycles themes with 't'.  PAIR_HUD and PAIR_HINT
 *   are registered exactly once and never touched by theme_apply, so
 *   HUD legibility is independent of the active palette.
 */

/*
 * theme_apply() — rewrite ramp colour pairs CP_BASE..CP_BASE+RAMP_N-1.
 *
 *   Purpose      : swap the flame's tint without changing HUD pairs.
 *   Pseudocode   : for i in 0..RAMP_N-1:
 *                      init_pair(CP_BASE + i, theme.fg[i], COLOR_BLACK)
 *   Mental model : a single palette swap on the ramp; HUD stays put.
 *   Inputs       : theme_index — which entry of k_themes to apply
 *   Output       : ncurses pair table mutated in place
 *   Why it lives here: separation of concerns — theme_apply touches
 *                      only ramp pairs, color_init also touches HUD pairs.
 */
static void theme_apply(int theme_index)
{
    const FireTheme *theme = &k_themes[theme_index];
    for (int i = 0; i < RAMP_N; i++) {
        if (COLORS >= 256)
            init_pair(CP_BASE + i, theme->fg256[i], COLOR_BLACK);
        else
            init_pair(CP_BASE + i, theme->fg8[i],   COLOR_BLACK);
    }
}

/*
 * color_init() — one-time colour setup at program start.
 *
 *   Purpose      : register ramp pairs for the initial theme plus the
 *                  two HUD pairs that persist for the program's life.
 *   Pseudocode   : start_color();
 *                  theme_apply(initial_theme);
 *                  init_pair(PAIR_HUD,  bright_yellow, black);
 *                  init_pair(PAIR_HINT, bright_cyan,   black);
 *   Mental model : a paint-store opening — all the dye buckets get
 *                  filled once at the start of the day.
 *   Inputs       : initial_theme — index into k_themes for first frame
 *   Output       : ncurses pair table populated
 *   Why it lives here: HUD pairs must be initialised once and survive
 *                      every subsequent theme cycle, so this is the
 *                      one function that must touch them.
 */
static void color_init(int initial_theme)
{
    start_color();
    theme_apply(initial_theme);

    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, COLOR_BLACK);   /* bright yellow */
        init_pair(PAIR_HINT,  51, COLOR_BLACK);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(PAIR_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
}

/*
 * ramp_attr() — combine a ramp colour pair with theme-specific attrs.
 *
 *   Purpose      : produce the ncurses attribute to pass to attron().
 *   Pseudocode   : a ← COLOR_PAIR(CP_BASE + idx)
 *                  if COLORS ≥ 256 and idx ≥ RAMP_N-2: a |= A_BOLD
 *                  else:                                a |= theme.attr8[idx]
 *                  return a
 *   Mental model : in 256-colour mode the colour itself is bright
 *                  enough, so we only bold the top two buckets.  In
 *                  8-colour mode the theme's attr8 array does the
 *                  dim/normal/bold work.
 *   Inputs       : idx          — ramp bucket index
 *                  theme_index  — which palette is active (for attr8)
 *   Output       : attr_t suitable for attron / attroff
 *   Why it lives here: keeps the per-bucket attribute decision in one
 *                      spot, so the renderer reads as one line.
 */
static attr_t ramp_attr(int idx, int theme_index)
{
    attr_t attr = COLOR_PAIR(CP_BASE + idx);
    if (COLORS >= 256) {
        if (idx >= RAMP_N - 2) attr |= A_BOLD;
    } else {
        attr |= k_themes[theme_index].attr8[idx];
    }
    return attr;
}

/* ===================================================================== */
/* §11 grid storage                                                       */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   Persistent simulation state.  FirePart is one Lagrangian particle;
 *   the Grid owns three float buffers (current heat, previous heat,
 *   dither scratch), the algorithm-selector, the user-controlled
 *   wind/fuel knobs, and the entire particle pool inline (no pointer
 *   chasing in the hot path).
 */

/*
 * FirePart — one short-lived ember in algorithm 1.
 *
 *   Born at the source arch, drifts upward, heat fades by `decay` per
 *   tick.  The renderer reads heat × kernel for each splat.  active=false
 *   slots are recycled by fire_part_spawn().
 */
typedef struct {
    float x, y;       /* current position in grid-cell coordinates           */
    float vx, vy;     /* velocity in cells / tick; vy < 0 = upward            */
    float heat;       /* current heat, fades from 1 toward 0                  */
    float decay;      /* heat lost per tick (= 1 / lifetime)                  */
    bool  active;     /* false slots are available for re-spawning            */
} FirePart;

/*
 * Grid — all algorithm state in one struct.
 *
 *   Buffers are flat (cols * rows) float arrays, indexed [y * cols + x].
 *   Row 0 is the top of the terminal; row (rows - 1) holds fuel.
 *   The particle pool is inline rather than a pointer so resize() never
 *   has to relocate it.
 */
typedef struct {
    float    *heat;          /* [rows × cols] current heat                   */
    float    *prev_heat;     /* [rows × cols] last frame's heat              */
    float    *dither;        /* [rows × cols] FS scratch buffer              */
    int       cols, rows;

    float     fuel;          /* user-controlled fuel intensity in [0.1, 1.0] */
    int       wind;          /* user-controlled wind in [-WIND_MAX, WIND_MAX] */
    int       wind_acc;      /* accumulated wind offset for arch shifting    */
    int       theme;         /* index into k_themes                          */
    int       warmup;        /* counts up 0 → WARMUP_TICKS, then sticks      */
    int       algo;          /* 0=CA   1=Particle   2=Plasma                 */
    float     plasma_t;      /* plasma phase counter, advanced each tick     */

    FirePart  parts[MAX_FIRE_PARTS];
    int       part_idx;      /* round-robin spawn cursor                     */
} Grid;

/*
 * grid_alloc() — allocate the three heat buffers for the given size.
 *
 *   Purpose      : reserve memory once at startup / resize.
 *   Pseudocode   : record cols, rows; calloc three buffers of cols*rows.
 *   Mental model : pour concrete for three slabs of identical shape.
 *   Inputs       : grid pointer, cols, rows
 *   Output       : grid->heat, prev_heat, dither point to zeroed memory
 *   Why it lives here: keeps allocation in one spot; the hot path is
 *                      malloc-free as required by project guidelines.
 */
static void grid_alloc(Grid *grid, int cols, int rows)
{
    grid->cols      = cols;
    grid->rows      = rows;
    grid->heat      = calloc((size_t)(cols * rows), sizeof(float));
    grid->prev_heat = calloc((size_t)(cols * rows), sizeof(float));
    grid->dither    = calloc((size_t)(cols * rows), sizeof(float));
}

/*
 * grid_free() — release the three heat buffers.
 *
 *   Pseudocode   : free three buffers; zero the struct.
 *   Mental model : demolish the three slabs, level the lot.
 */
static void grid_free(Grid *grid)
{
    free(grid->heat);
    free(grid->prev_heat);
    free(grid->dither);
    memset(grid, 0, sizeof *grid);
}

/*
 * grid_resize() — free and re-allocate at new dimensions.
 *
 *   Pseudocode   : free; alloc with new size.
 *   Mental model : terminal resize blows the world away — the next
 *                  tick repopulates from fresh fuel + warmup ramp.
 */
static void grid_resize(Grid *grid, int cols, int rows)
{
    grid_free(grid);
    grid_alloc(grid, cols, rows);
}

/*
 * grid_init() — first-time setup at program start.
 *
 *   Pseudocode   : alloc; set fuel=1, wind=0, theme=given, warmup=0,
 *                  algo=0 (CA), plasma_t=0, part_idx=0.
 *   Mental model : factory defaults: classic Doom fire, full fuel,
 *                  no wind, starting from cold.
 */
static void grid_init(Grid *grid, int cols, int rows, int theme)
{
    grid_alloc(grid, cols, rows);
    grid->fuel     = 1.0f;
    grid->wind     = 0;
    grid->wind_acc = 0;
    grid->theme    = theme;
    grid->warmup   = 0;
    grid->algo     = 0;
    grid->plasma_t = 0.f;
    grid->part_idx = 0;
}

/* ===================================================================== */
/* §12 shared helpers                                                     */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   Helpers used by more than one algorithm.  Every fire algorithm
 *   wants:
 *     • a clampf() for saturating arithmetic,
 *     • a warmup_scale_factor() that grows from 0 to 1 over the first
 *       WARMUP_TICKS so the flame doesn't pop into existence,
 *     • a wind accumulator that wraps at ±cols so the arch never drifts
 *       off-screen permanently,
 *     • an arch_envelope() that describes the source distribution along
 *       the bottom row.
 *   Algorithm 0 and 1 additionally use seed_fuel_row() / splat3x3().
 */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline int clamp_int(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/*
 * Named random primitives.  Every algorithm below uses these instead of
 * raw rand() expressions, so the reader can tell what the value MEANS
 * (a unit interval sample, a centred kick, a one-cell jitter) without
 * decoding the arithmetic.
 *
 *   rand_unit()           — uniform sample in [0, 1)
 *   rand_signed_unit()    — uniform sample in [-0.5, 0.5)
 *   rand_lateral_jitter() — uniform integer in {-1, 0, +1}
 */
static inline float rand_unit(void)           { return (float)rand() / RAND_MAX; }
static inline float rand_signed_unit(void)    { return rand_unit() - 0.5f; }
static inline int   rand_lateral_jitter(void) { return (rand() % 3) - 1; }

/*
 * warmup_scale_factor() — 0 → 1 ramp over the first WARMUP_TICKS ticks.
 *
 *   Purpose      : prevent a sudden flame pop at program start /
 *                  algorithm switch.
 *   Pseudocode   : if grid->warmup < WARMUP_TICKS:
 *                      scale ← warmup / WARMUP_TICKS
 *                      warmup++
 *                  else:
 *                      scale ← 1
 *                  return scale
 *   Mental model : a dimmer knob being turned up steadily.
 *   Inputs       : grid (read warmup; possibly increment)
 *   Output       : float in [0, 1]
 *   Why it lives here: every algorithm needs the same ramp behaviour,
 *                      so the counter and the formula live together.
 */
static float warmup_scale_factor(Grid *grid)
{
    float scale = (grid->warmup < WARMUP_TICKS)
                  ? (float)grid->warmup / (float)WARMUP_TICKS
                  : 1.f;
    if (grid->warmup < WARMUP_TICKS) grid->warmup++;
    return scale;
}

/*
 * advance_wind() — shift wind_acc by one tick's worth of wind.
 *
 *   Purpose      : translate the source arch left or right over time.
 *   Pseudocode   : grid->wind_acc += grid->wind
 *                  if |wind_acc| ≥ cols: wind_acc ← 0   (wrap)
 *   Mental model : a conveyor belt under the source arch, moving the
 *                  whole flame sideways.
 *   Inputs       : grid (mutated)
 *   Output       : none
 *   Why it lives here: every algorithm passes wind_acc to arch_envelope,
 *                      so the accumulator lives in one place.
 */
static void advance_wind(Grid *grid)
{
    grid->wind_acc += grid->wind;
    if (grid->wind_acc >=  grid->cols ||
        grid->wind_acc <= -grid->cols)
        grid->wind_acc = 0;
}

/*
 * arch_envelope() — bell-shaped weight along the bottom row.
 *
 *   Purpose      : describe how strongly each column is fuelled.
 *   Pseudocode   : margin ← cols × ARCH_MARGIN_FRAC
 *                  span   ← cols − 2 × margin
 *                  shifted ← x − margin − wind_acc
 *                  t      ← shifted / span
 *                  if t outside [0, 1]: return 0
 *                  edge   ← min(t, 1 − t) × 2
 *                  return edge²
 *   Mental model : a squared triangular hat function: zero at the
 *                  margins, 1.0 at the centre, sharper rolloff than a
 *                  cosine.
 *   Inputs       : x         — column index
 *                  cols      — terminal width in cells
 *                  wind_acc  — current wind offset
 *   Output       : float weight in [0, 1]
 *   Why it lives here: shared by every algorithm's source step.
 */
static float arch_envelope(int x, int cols, int wind_acc)
{
    float margin     = (float)cols * ARCH_MARGIN_FRAC;
    float span       = (float)cols - 2.f * margin;
    float shifted_x  = (float)x - margin - (float)wind_acc;
    float t          = (span > 0.f) ? shifted_x / span : 0.f;
    if (t < 0.f || t > 1.f) return 0.f;
    float edge       = (t < 0.5f) ? t : 1.f - t;
    float weight     = edge * 2.f;
    return weight * weight;
}

/*
 * seed_fuel_row() — write arch-shaped fuel into the bottom row.
 *
 *   Purpose      : provide the source for the CA algorithm (also used
 *                  to seed the plasma's per-column gradient indirectly).
 *   Pseudocode   : for every column x:
 *                      a ← arch_envelope(x)
 *                      if a == 0: cell ← 0, continue
 *                      jitter ← BASE + RANGE × rand()
 *                      cell  ← MAX_HEAT × fuel × a × jitter × warmup
 *   Mental model : a row of gas burners, each set to its own knob.
 *   Inputs       : grid, warmup_scale
 *   Output       : grid->heat bottom row mutated
 *   Why it lives here: bottom row seeding is the only place an
 *                      algorithm explicitly touches the source, so it
 *                      is centralised.
 */
static void seed_fuel_row(Grid *grid, float warmup_scale)
{
    int    cols       = grid->cols;
    int    fuel_y     = grid->rows - 1;
    float *heat_grid  = grid->heat;

    for (int x = 0; x < cols; x++) {
        float arch_weight = arch_envelope(x, cols, grid->wind_acc);
        if (arch_weight <= 0.f) {
            heat_grid[fuel_y * cols + x] = 0.f;
            continue;
        }

        float random_jitter   = FUEL_JITTER_BASE + FUEL_JITTER_RANGE * rand_unit();
        float fuel_at_column  = MAX_HEAT * grid->fuel * arch_weight
                              * random_jitter * warmup_scale;
        heat_grid[fuel_y * cols + x] = fuel_at_column;
    }
}

/*
 * splat3x3() — deposit value v over a 3×3 Gaussian kernel.
 *
 *   Purpose      : translate one particle's heat into nine cell
 *                  contributions.
 *   Pseudocode   : kernel ←  0.0625 0.125 0.0625
 *                            0.125  0.25  0.125
 *                            0.0625 0.125 0.0625
 *                  for dy in -1..1, dx in -1..1:
 *                      if (cx+dx, cy+dy) is in-bounds:
 *                          heat[cy+dy][cx+dx] += v * kernel[dy+1][dx+1]
 *   Mental model : stamp a tiny Gaussian heat blob centred on (cx, cy).
 *   Inputs       : heat array, cols, rows, centre (cx, cy), value v
 *   Output       : heat array mutated additively
 *   Why it lives here: shared with future splat-based effects; also
 *                      neatly encapsulates the kernel constants.
 */
static void splat3x3(float *heat_grid, int cols, int rows,
                     int cx, int cy, float v)
{
    static const float kernel[3][3] = {
        { 0.0625f, 0.125f, 0.0625f },
        { 0.125f,  0.25f,  0.125f  },
        { 0.0625f, 0.125f, 0.0625f },
    };
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int nx = cx + dx, ny = cy + dy;
            if (nx >= 0 && nx < cols && ny >= 0 && ny < rows)
                heat_grid[ny * cols + nx] += v * kernel[dy + 1][dx + 1];
        }
    }
}

/* ===================================================================== */
/* §13 algo 0 — Doom CA fire                                              */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   The first algorithm: a cellular automaton modelled after the fire
 *   effect in Doom (1993).  Every tick the bottom row is reset to a
 *   fresh arch-shaped fuel pattern; every other row pulls its value
 *   from the row below, with a ±1 lateral jitter and a random per-cell
 *   decay.  See GUIDED TUTORIAL §3 for the derivation; this section is
 *   the literal implementation.
 *
 *   The decay is computed adaptively from screen height so that the
 *   expected flame top sits at CA_REACH_FRAC × rows regardless of the
 *   user's terminal size.
 */

/*
 * ca_compute_adaptive_decay() — pick base + range so the flame's mean
 * peak lands at CA_REACH_FRAC × rows.
 *
 *   Reasoning:
 *      "average decay per step" × "steps from base to peak" ≈ MAX_HEAT
 *      so   avg_decay ≈ MAX_HEAT / (rows × CA_REACH_FRAC)
 *      then base and range are fixed fractions of avg_decay,
 *      with a floor for tiny terminals (rows < 5).
 *
 *   Outputs:
 *      *decay_base, *decay_range — used in the propagation loop as
 *      decay = base + rand_unit() × range.
 */
static void ca_compute_adaptive_decay(int rows,
                                      float *decay_base_out,
                                      float *decay_range_out)
{
    float reach_height_in_rows   = (float)rows * CA_REACH_FRAC;
    float average_decay_per_step = (reach_height_in_rows > 1.f)
                                   ? (MAX_HEAT / reach_height_in_rows)
                                   : MAX_HEAT;

    float decay_base  = average_decay_per_step * CA_DECAY_BASE_FRAC;
    float decay_range = average_decay_per_step * CA_DECAY_RAND_FRAC;
    if (decay_base  < CA_DECAY_BASE_MIN) decay_base  = CA_DECAY_BASE_MIN;
    if (decay_range < CA_DECAY_RAND_MIN) decay_range = CA_DECAY_RAND_MIN;

    *decay_base_out  = decay_base;
    *decay_range_out = decay_range;
}

/*
 * ca_propagate_one_cell() — one cell's value from the row below.
 *
 *   The five labelled intermediates mirror the algorithm in steps:
 *     lateral_offset    ← random {-1, 0, +1}   (mixing in x)
 *     source_column     ← x + lateral_offset   (clamped to grid)
 *     source_heat       ← heat[y+1][source_column]
 *     energy_lost       ← decay_base + rand_unit() × decay_range
 *     propagated_heat   ← max(0, source_heat − energy_lost)
 *
 *   Returning the new value (rather than mutating in place) keeps the
 *   inner loop a single assignment.
 */
static float ca_propagate_one_cell(const float *heat_grid,
                                   int cols, int x, int y,
                                   float decay_base, float decay_range)
{
    int   lateral_offset    = rand_lateral_jitter();
    int   source_column     = clamp_int(x + lateral_offset, 0, cols - 1);
    float source_heat       = heat_grid[(y + 1) * cols + source_column];
    float energy_lost       = decay_base + rand_unit() * decay_range;
    float propagated_heat   = source_heat - energy_lost;
    return (propagated_heat < 0.f) ? 0.f : propagated_heat;
}

/*
 * ca_fire_tick() — orchestrate one Doom-CA frame.  The body reads as
 * the four algorithm steps, one per line.
 */
static void ca_fire_tick(Grid *grid)
{
    int    cols      = grid->cols;
    int    rows      = grid->rows;
    float *heat_grid = grid->heat;

    /* Step 1 — warmup ramp + per-tick wind shift of the source. */
    float warmup_scale = warmup_scale_factor(grid);
    advance_wind(grid);

    /* Step 2 — refresh the bottom-row fuel arch. */
    seed_fuel_row(grid, warmup_scale);

    /* Step 3 — calibrate decay so the average flame top lands at
     *          CA_REACH_FRAC of the terminal height. */
    float decay_base, decay_range;
    ca_compute_adaptive_decay(rows, &decay_base, &decay_range);

    /* Step 4 — propagate heat upward, row by row, bottom-most up. */
    for (int y = 0; y < rows - 1; y++) {
        for (int x = 0; x < cols; x++) {
            heat_grid[y * cols + x] =
                ca_propagate_one_cell(heat_grid, cols, x, y,
                                      decay_base, decay_range);
        }
    }
}

/* ===================================================================== */
/* §14 algo 1 — particle fire                                             */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   The second algorithm: a Lagrangian particle pool.  Two functions
 *   live in this section — fire_part_spawn() births one particle at
 *   the arch source, and particle_fire_tick() advances every active
 *   particle, spawns new ones up to SPAWN_PER_TICK × warmup, and
 *   3×3-Gaussian-splats each onto the heat grid.
 *
 *   See GUIDED TUTORIAL §5..§6 for the conceptual background.
 */

/*
 * fire_part_spawn() — birth one particle at the source arch.
 *
 *   Purpose      : pick a birth column weighted by arch_envelope, then
 *                  initialise velocity, heat, and lifetime.
 *   Pseudocode   : margin ← cols × ARCH_MARGIN_FRAC
 *                  span   ← cols − 2 × margin
 *                  for up to 8 tries:
 *                      t ← rand()
 *                      cx ← margin + t × span + wind_acc
 *                      arch ← (min(t, 1-t) × 2)²
 *                      if rand() < arch × fuel × warmup_scale:
 *                          birth_column ← cx; break
 *                  if no try succeeded: birth at cols / 2
 *                  vx ← (rand() − 0.5) × VX_SPREAD
 *                  vy ← -(VY_BASE + rand() × VY_RANGE)
 *                  life ← LIFE_MIN + rand() × LIFE_RANGE
 *                  heat ← 1; decay ← 1 / life; active ← true
 *   Mental model : rejection sampling: throw darts at the arch
 *                  envelope until one sticks; that x becomes the birth
 *                  column.  Centre cells are likelier than edges.
 *   Inputs       : particle, grid, warmup_scale
 *   Output       : particle initialised, active = true
 *   Why it lives here: separates the "where does a particle come from"
 *                      decision from the "how does it move" loop.
 */
/*
 * rejection_sample_birth_column() — pick birth x weighted by the arch.
 *
 *   Algorithm (rejection sampling):
 *      repeat up to 8 times:
 *          t                ← rand_unit()
 *          candidate_column ← margin + t × span + wind_acc
 *          arch_weight      ← (min(t, 1-t) × 2)²    (squared arch)
 *          acceptance_prob  ← arch_weight × fuel × warmup
 *          if rand_unit() < acceptance_prob: return candidate_column
 *      return cols / 2   (fallback after 8 misses)
 *
 *   Rejection sampling gives the right distribution without computing
 *   a CDF: the arch_weight × fuel × warmup product IS the probability
 *   that a candidate is kept.  Centre columns are kept more often.
 */
static float rejection_sample_birth_column(Grid *grid, float warmup_scale)
{
    int   cols              = grid->cols;
    float source_margin     = (float)cols * ARCH_MARGIN_FRAC;
    float source_span       = (float)cols - 2.f * source_margin;
    float fallback_centre   = (float)cols * 0.5f;

    for (int dart = 0; dart < 8; dart++) {
        float t                  = rand_unit();
        float candidate_column   = source_margin + t * source_span
                                 + (float)grid->wind_acc;
        float edge_distance      = (t < 0.5f) ? t : 1.f - t;
        float arch_weight        = (edge_distance * 2.f) * (edge_distance * 2.f);
        float acceptance_prob    = arch_weight * grid->fuel * warmup_scale;
        if (rand_unit() < acceptance_prob)
            return candidate_column;
    }
    return fallback_centre;
}

/*
 * fire_part_spawn() — birth one particle at the source arch.
 *
 *   The body is four labelled steps: pick column → set position →
 *   set velocity → set lifetime.  No nested arithmetic.
 */
static void fire_part_spawn(FirePart *particle, Grid *grid, float warmup_scale)
{
    /* Step 1 — pick a birth column, biased toward the arch centre. */
    float birth_column = rejection_sample_birth_column(grid, warmup_scale);

    /* Step 2 — birth position lives on the bottom row. */
    particle->x = birth_column;
    particle->y = (float)(grid->rows - 1);

    /* Step 3 — initial velocity: small lateral kick, large upward thrust. */
    float initial_vx = rand_signed_unit() * PART_VX_SPREAD;
    float initial_vy = -(PART_VY_BASE + rand_unit() * PART_VY_RANGE);
    particle->vx = initial_vx;
    particle->vy = initial_vy;

    /* Step 4 — lifetime → per-tick heat decay rate. */
    float lifetime_ticks    = PART_LIFE_MIN + rand_unit() * PART_LIFE_RANGE;
    particle->heat          = 1.0f;
    particle->decay         = 1.0f / lifetime_ticks;
    particle->active        = true;
}

/*
 * particle_advance_one() — one tick of motion + heat decay for one ember.
 *
 *   turbulence_kick   ← rand_signed_unit() × PART_TURB_STEP
 *   vx                ← (vx + turbulence_kick) × PART_VX_DAMP
 *   position          ← position + velocity
 *   heat              ← heat - decay
 *   active            ← heat > 0  AND  ember still on grid
 */
static void particle_advance_one(FirePart *ember, int cols)
{
    float turbulence_kick = rand_signed_unit() * PART_TURB_STEP;
    ember->vx            = (ember->vx + turbulence_kick) * PART_VX_DAMP;
    ember->x             += ember->vx;
    ember->y             += ember->vy;
    ember->heat          -= ember->decay;

    bool burned_out      = (ember->heat <= 0.f);
    bool left_top_edge   = (ember->y    <  0.f);
    bool left_side_edge  = (ember->x    <  0.f) || (ember->x >= (float)cols);
    if (burned_out || left_top_edge || left_side_edge)
        ember->active = false;
}

/*
 * phase_a_advance_active_particles() — move every alive ember one tick.
 */
static void phase_a_advance_active_particles(Grid *grid)
{
    int cols = grid->cols;
    for (int i = 0; i < MAX_FIRE_PARTS; i++) {
        FirePart *ember = &grid->parts[i];
        if (!ember->active) continue;
        particle_advance_one(ember, cols);
    }
}

/*
 * phase_b_spawn_new_particles() — birth up to SPAWN_PER_TICK × warmup embers.
 *
 *   For each desired spawn, walk round-robin through the pool until an
 *   inactive slot is found; initialise it via fire_part_spawn.
 */
static void phase_b_spawn_new_particles(Grid *grid, float warmup_scale)
{
    int spawn_target = (int)((float)SPAWN_PER_TICK * warmup_scale) + 1;

    for (int s = 0; s < spawn_target; s++) {
        for (int tries = 0; tries < MAX_FIRE_PARTS; tries++) {
            grid->part_idx = (grid->part_idx + 1) % MAX_FIRE_PARTS;
            FirePart *slot = &grid->parts[grid->part_idx];
            if (!slot->active) {
                fire_part_spawn(slot, grid, warmup_scale);
                break;
            }
        }
    }
}

/*
 * phase_c_splat_all_to_grid() — clear heat grid; stamp 3×3 splat per ember.
 *
 *   Converts the Lagrangian particle pool back to an Eulerian grid that
 *   the shared renderer can consume.
 */
static void phase_c_splat_all_to_grid(Grid *grid)
{
    int cols = grid->cols, rows = grid->rows;

    memset(grid->heat, 0, (size_t)(cols * rows) * sizeof(float));

    for (int i = 0; i < MAX_FIRE_PARTS; i++) {
        const FirePart *ember = &grid->parts[i];
        if (!ember->active) continue;
        int splat_centre_x = (int)(ember->x + 0.5f);
        int splat_centre_y = (int)(ember->y + 0.5f);
        splat3x3(grid->heat, cols, rows, splat_centre_x, splat_centre_y, ember->heat);
    }
}

/*
 * phase_d_clamp_oversaturation() — cap cells where several splats overlap.
 *
 *   The 3×3 Gaussian kernel sums to 1 for a single particle, but many
 *   embers may occupy overlapping neighbourhoods.  Cap the sum at
 *   MAX_HEAT so the gamma + dither passes see a value in [0, 1].
 */
static void phase_d_clamp_oversaturation(Grid *grid)
{
    int total_cells = grid->cols * grid->rows;
    for (int i = 0; i < total_cells; i++)
        if (grid->heat[i] > MAX_HEAT) grid->heat[i] = MAX_HEAT;
}

/*
 * particle_fire_tick() — orchestrate one particle frame.  The body
 * is six lines, one per algorithm phase.
 */
static void particle_fire_tick(Grid *grid)
{
    float warmup_scale = warmup_scale_factor(grid);
    advance_wind(grid);

    phase_a_advance_active_particles(grid);
    phase_b_spawn_new_particles(grid, warmup_scale);
    phase_c_splat_all_to_grid(grid);
    phase_d_clamp_oversaturation(grid);
}

/* ===================================================================== */
/* §15 algo 2 — plasma fire                                               */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   The third algorithm: stateless procedural fire.  Each column gets a
 *   tongue-height function defined as the sum of three sine harmonics;
 *   below the tongue tip we shade cells with a smooth gradient from hot
 *   to cold.  No information is carried between frames except the
 *   global phase counter plasma_t.  See GUIDED TUTORIAL §7 for the
 *   derivation.
 */

/*
 * plasma_tongue_height() — sum three sines, then scale by source weight.
 *
 *   The three harmonics use intentionally non-commensurate frequency /
 *   speed pairs so the pattern never repeats exactly:
 *
 *      H1   amp 0.28   xfreq 5    tspd +2.2     (primary undulation)
 *      H2   amp 0.18   xfreq 11   tspd −1.6     (fine flicker, opposite drift)
 *      H3   amp 0.10   xfreq 3    tspd +0.7     (slow base sway)
 *
 *   Steps:
 *      harmonic_n   ← amp_n × sin(wx × xfreq_n ± t × tspd_n)
 *      raw_tongue   ← base + h1 + h2 + h3
 *      clamped      ← clamp(raw_tongue, 0, 1)        keep in [0,1]
 *      final_height ← clamped × fuel × warmup × √arch_weight
 *
 *   sqrtf(arch_weight) is used (instead of arch_weight directly) so the
 *   tongue falls off more gently at the edges than the squared arch.
 */
static float plasma_tongue_height(float wind_shifted_x, float phase_t,
                                  float fuel, float warmup_scale,
                                  float arch_weight)
{
    float harmonic_1 = PLASMA_H1_AMP *
        sinf(wind_shifted_x * PLASMA_H1_XFREQ + phase_t * PLASMA_H1_TSPD);
    float harmonic_2 = PLASMA_H2_AMP *
        sinf(wind_shifted_x * PLASMA_H2_XFREQ - phase_t * PLASMA_H2_TSPD);
    float harmonic_3 = PLASMA_H3_AMP *
        sinf(wind_shifted_x * PLASMA_H3_XFREQ + phase_t * PLASMA_H3_TSPD);

    float raw_tongue       = PLASMA_BASE + harmonic_1 + harmonic_2 + harmonic_3;
    float clamped_tongue   = clampf(raw_tongue, 0.f, 1.f);
    float final_tongue     = clamped_tongue * fuel * warmup_scale * sqrtf(arch_weight);
    return final_tongue;
}

/*
 * plasma_shade_column() — write per-row heat below the tongue tip.
 *
 *   normalised_y     ← y / rows                     (0 = top, 1 = bottom)
 *   tongue_tip_ny    ← 1 − tongue_height            (where flame begins)
 *   above_tip        ← normalised_y − tongue_tip_ny (>0 inside flame)
 *   heat             ← clamp(above_tip / tongue_height, 0, 1)
 *
 *   The cell at the bottom (normalised_y = 1) gets heat ≈ 1, the cell
 *   at the tongue tip gets 0, and rows above the tip get 0 by clamp.
 */
static void plasma_shade_column(Grid *grid, int x, float tongue_height)
{
    int   cols           = grid->cols;
    int   rows           = grid->rows;
    float inverse_tongue = (tongue_height > 0.01f) ? (1.f / tongue_height) : 100.f;
    float tongue_tip_ny  = 1.f - tongue_height;

    for (int y = 0; y < rows; y++) {
        float normalised_y    = (float)y / (float)rows;
        float above_tongue_tip = normalised_y - tongue_tip_ny;
        float heat_value      = clampf(above_tongue_tip * inverse_tongue, 0.f, 1.f);
        grid->heat[y * cols + x] = heat_value;
    }
}

/*
 * plasma_clear_column() — zero an entire column (used outside the arch).
 */
static void plasma_clear_column(Grid *grid, int x)
{
    int cols = grid->cols;
    int rows = grid->rows;
    for (int y = 0; y < rows; y++) grid->heat[y * cols + x] = 0.f;
}

/*
 * plasma_fire_tick() — orchestrate one plasma frame.  The per-column
 * loop body reads as the algorithm: compute coordinates → measure arch
 * weight → either clear or compute-tongue-and-shade.
 */
static void plasma_fire_tick(Grid *grid)
{
    int cols = grid->cols;

    /* Step 1 — warmup ramp + per-tick wind shift. */
    float warmup_scale = warmup_scale_factor(grid);
    advance_wind(grid);

    /* Step 2 — advance the global phase counter. */
    float phase_t      = grid->plasma_t;
    grid->plasma_t    += PLASMA_TIME_STEP;

    /* Step 3 — for each column: compute tongue height, then shade rows. */
    for (int x = 0; x < cols; x++) {
        float normalised_x   = (float)x / (float)cols;
        float wind_shifted_x = normalised_x + (float)grid->wind_acc / (float)cols;
        float arch_weight    = arch_envelope(x, cols, grid->wind_acc);

        if (arch_weight <= 0.f) {
            plasma_clear_column(grid, x);
            continue;
        }

        float tongue_height = plasma_tongue_height(wind_shifted_x, phase_t,
                                                   grid->fuel, warmup_scale,
                                                   arch_weight);
        plasma_shade_column(grid, x, tongue_height);
    }
}

/* ===================================================================== */
/* §16 dispatch                                                           */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   One-line indirection that lets the renderer be ignorant of which
 *   heater is active.  The user presses 'a' to advance grid->algo;
 *   grid_tick() is the only place the algorithm index is consulted.
 */

/*
 * grid_tick() — dispatch to the selected algorithm's tick function.
 */
static void grid_tick(Grid *grid)
{
    switch (grid->algo) {
    case 0: ca_fire_tick(grid);       break;
    case 1: particle_fire_tick(grid); break;
    case 2: plasma_fire_tick(grid);   break;
    default: break;
    }
}

/* ===================================================================== */
/* §17 render pipeline                                                    */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   The shared renderer.  Heat values are turned into ASCII glyphs in
 *   four passes:
 *     1. Gamma-correct every cell from linear to perceptual.
 *     2. Floyd-Steinberg quantise into one of RAMP_N buckets, pushing
 *        the rounding error onto the four-cell forward neighbourhood.
 *     3. Draw the glyph with the correct theme attribute.
 *     4. Erase cells that went from hot to cold via a literal ' '.
 *
 *   Cells that were never lit and remain cold are skipped entirely —
 *   ncurses leaves them at the terminal background, which is what we
 *   want for the borderless flame effect.
 *
 *   The §18 debug overlays plug into this pipeline by replacing pass 1
 *   and/or pass 2 with diagnostic variants; see below.
 */

/*
 * Forward declaration for the debug-overlay helpers — defined in §18.
 */
typedef enum {
    DEBUG_OFF             = 0,
    DEBUG_RAW_HEAT        = 1,
    DEBUG_GAMMA_NO_DITHER = 2,
    DEBUG_ARCH_ENVELOPE   = 3,
} DebugMode;

static DebugMode g_debug_mode = DEBUG_OFF;

static void debug_fill_dither_buffer(Grid *grid);

/*
 * pipeline_pass1_gamma_correct() — fill dither_buffer with perceptual heat.
 *
 *   For every cell:
 *      linear_heat     ← grid->heat[i]
 *      if cold:        dither_buffer[i] ← -1     (skip sentinel)
 *      else:           saturated   ← min(1, linear_heat / MAX_HEAT)
 *                      perceptual  ← pow(saturated, 1/2.2)
 *                      dither_buffer[i] ← perceptual
 *
 *   The −1 sentinel lets pass 2 distinguish "draw a glyph", "erase a
 *   stale glyph", and "do nothing" without re-reading the heat field.
 */
static void pipeline_pass1_gamma_correct(Grid *grid)
{
    int    total_cells   = grid->cols * grid->rows;
    float *heat_grid     = grid->heat;
    float *dither_buffer = grid->dither;

    for (int i = 0; i < total_cells; i++) {
        float linear_heat = heat_grid[i];
        if (linear_heat <= 0.f) {
            dither_buffer[i] = -1.f;
            continue;
        }
        float saturated_heat = fminf(1.f, linear_heat / MAX_HEAT);
        float perceptual     = powf(saturated_heat, 1.f / 2.2f);
        dither_buffer[i]     = perceptual;
    }
}

/*
 * pipeline_diffuse_quant_error() — push one cell's rounding error to neighbours.
 *
 *   Floyd-Steinberg distribution (cold-tagged neighbours skipped):
 *
 *      ── current ─────► 7/16 → (x+1, y)
 *      3/16 → (x-1, y+1) ── 5/16 → (x, y+1) ── 1/16 → (x+1, y+1)
 */
static void pipeline_diffuse_quant_error(float *dither_buffer, int cols, int rows,
                                         int x, int y, float quant_error)
{
    int i = y * cols + x;

    if (x + 1 < cols && dither_buffer[i + 1] >= 0.f)
        dither_buffer[i + 1] += quant_error * (7.f / 16.f);

    if (y + 1 < rows) {
        if (x - 1 >= 0 && dither_buffer[i + cols - 1] >= 0.f)
            dither_buffer[i + cols - 1] += quant_error * (3.f / 16.f);
        if (dither_buffer[i + cols] >= 0.f)
            dither_buffer[i + cols]     += quant_error * (5.f / 16.f);
        if (x + 1 < cols && dither_buffer[i + cols + 1] >= 0.f)
            dither_buffer[i + cols + 1] += quant_error * (1.f / 16.f);
    }
}

/*
 * pipeline_draw_lit_cell() — quantise one cell and draw its glyph.
 *
 *   bucket        ← lut_index(perceptual)
 *   if dithering: error ← perceptual − lut_midpoint(bucket)
 *                 diffuse error to four-cell forward neighbourhood
 *   draw k_ramp[bucket] with the theme's ramp attribute
 */
static void pipeline_draw_lit_cell(Grid *grid, int x, int y,
                                   float perceptual, bool apply_dither)
{
    int   bucket          = lut_index(perceptual);

    if (apply_dither) {
        float bucket_midpoint = lut_midpoint(bucket);
        float quant_error     = perceptual - bucket_midpoint;
        pipeline_diffuse_quant_error(grid->dither, grid->cols, grid->rows,
                                     x, y, quant_error);
    }

    attr_t glyph_attr = ramp_attr(bucket, grid->theme);
    attron(glyph_attr);
    mvaddch(y, x, (chtype)(unsigned char)k_ramp[bucket]);
    attroff(glyph_attr);
}

/*
 * pipeline_pass2_quantise_and_draw() — walk dither_buffer, draw or erase.
 *
 *   For every cell visible to the terminal:
 *      cold sentinel        → if previous frame lit: write a single ' '
 *      lit                  → pipeline_draw_lit_cell()
 */
static void pipeline_pass2_quantise_and_draw(Grid *grid, int tcols, int trows,
                                             bool apply_dither)
{
    int    cols          = grid->cols;
    int    rows          = grid->rows;
    float *dither_buffer = grid->dither;
    float *previous_heat = grid->prev_heat;

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            if (x >= tcols || y >= trows) continue;

            int   i           = y * cols + x;
            float perceptual  = dither_buffer[i];

            if (perceptual < 0.f) {
                bool was_lit_last_frame = (previous_heat[i] > 0.f);
                if (was_lit_last_frame) mvaddch(y, x, ' ');
                continue;
            }

            pipeline_draw_lit_cell(grid, x, y, perceptual, apply_dither);
        }
    }
}

/*
 * pipeline_pass3_archive_current_frame() — record current heat as previous.
 *
 *   swap pointers: prev ↔ cur   (cheap pointer assignment, no copy)
 *   memcpy cur ← prev            (so next tick's writers see the right
 *                                 buffer, and pass 2 can compare against
 *                                 prev to detect just-extinguished cells)
 */
static void pipeline_pass3_archive_current_frame(Grid *grid)
{
    int   total_cells = grid->cols * grid->rows;
    float *tmp        = grid->prev_heat;
    grid->prev_heat   = grid->heat;
    grid->heat        = tmp;
    memcpy(grid->heat, grid->prev_heat, (size_t)total_cells * sizeof(float));
}

/*
 * grid_draw() — orchestrate the three passes.  The body is the algorithm:
 *      pass 1   load perceptual values (or a debug overlay)
 *      pass 2   quantise + draw, optionally diffuse error
 *      pass 3   archive current frame for next frame's "did this cell
 *               just go cold?" decision
 */
static void grid_draw(Grid *grid, int tcols, int trows)
{
    bool is_normal_render = (g_debug_mode == DEBUG_OFF);

    /* Pass 1 — fill dither_buffer from gamma or a debug overlay. */
    if (is_normal_render) pipeline_pass1_gamma_correct(grid);
    else                  debug_fill_dither_buffer(grid);

    /* Pass 2 — quantise + draw; dithering only in normal-render mode. */
    pipeline_pass2_quantise_and_draw(grid, tcols, trows, is_normal_render);

    /* Pass 3 — archive current frame so next frame can detect changes. */
    pipeline_pass3_archive_current_frame(grid);
}

/* ===================================================================== */
/* §18 debug overlays                                                     */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   Three pedagogical overlays the user can cycle with 'd' (forward)
 *   and 'D' (backward).  Each replaces only the first pass of the
 *   renderer (gamma correction) with a diagnostic variant; the
 *   quantisation and glyph drawing then proceed normally but with
 *   dithering disabled so the staircase is visible.
 *
 *     mode 0   DEBUG_OFF             — normal render (no override)
 *     mode 1   DEBUG_RAW_HEAT        — heat fed directly to the LUT;
 *                                      no gamma curve.  Coarse banding
 *                                      proves why gamma correction is
 *                                      needed.
 *     mode 2   DEBUG_GAMMA_NO_DITHER — gamma is applied but dithering
 *                                      is disabled.  Banding visible
 *                                      in the highlights — proves why
 *                                      dithering is needed.
 *     mode 3   DEBUG_ARCH_ENVELOPE   — every cell is set to its column's
 *                                      arch_envelope() value.  Reveals
 *                                      the source distribution that
 *                                      every algorithm sits on top of.
 */

/*
 * debug_fill_dither_buffer() — populate dither_buffer for overlay modes.
 *
 *   Purpose      : replace gamma correction with a diagnostic mapping
 *                  so the next pass renders the chosen overlay.
 *   Pseudocode   : switch g_debug_mode:
 *                      RAW_HEAT:        v ← clamp(heat / MAX_HEAT, 0, 1)
 *                      GAMMA_NO_DITHER: v ← pow(clamp(heat / MAX_HEAT), 1/2.2)
 *                      ARCH_ENVELOPE:   v ← arch_envelope(x, cols, wind_acc)
 *                  v ≤ 0 → mark cold (-1).
 *   Mental model : three different "what to print here" rules; the
 *                  rest of the pipeline is reused unchanged.
 *   Inputs       : grid (read-only except via dither_buffer write)
 *   Output       : grid->dither populated with overlay-specific values
 *   Why it lives here: keeps every diagnostic mapping in one place; the
 *                      hot path stays free of overlay code paths.
 */
static void debug_fill_dither_buffer(Grid *grid)
{
    int    cols              = grid->cols;
    int    rows              = grid->rows;
    float *heat_grid         = grid->heat;
    float *dither_buffer     = grid->dither;

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            int   i = y * cols + x;
            float v;

            switch (g_debug_mode) {
            case DEBUG_RAW_HEAT:
                v = heat_grid[i];
                if (v <= 0.f) { dither_buffer[i] = -1.f; continue; }
                v = fminf(1.f, v / MAX_HEAT);
                break;

            case DEBUG_GAMMA_NO_DITHER:
                v = heat_grid[i];
                if (v <= 0.f) { dither_buffer[i] = -1.f; continue; }
                v = powf(fminf(1.f, v / MAX_HEAT), 1.f / 2.2f);
                break;

            case DEBUG_ARCH_ENVELOPE:
                v = arch_envelope(x, cols, grid->wind_acc);
                if (v <= 0.f) { dither_buffer[i] = -1.f; continue; }
                break;

            default:
                dither_buffer[i] = -1.f;
                continue;
            }

            dither_buffer[i] = v;
        }
    }
}

/*
 * debug_mode_name() — short label for the active overlay.
 */
static const char *debug_mode_name(DebugMode mode)
{
    switch (mode) {
    case DEBUG_OFF:             return "off";
    case DEBUG_RAW_HEAT:        return "raw-heat";
    case DEBUG_GAMMA_NO_DITHER: return "gamma-only";
    case DEBUG_ARCH_ENVELOPE:   return "arch";
    default:                    return "?";
    }
}

/* ===================================================================== */
/* §19 scene                                                              */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   The scene layer wraps the grid with two pieces of UI state: the
 *   pause flag (set by SPACE, suppresses grid_tick) and the
 *   needs_clear flag (set by algorithm/theme/resize transitions, forces
 *   one erase() on the next frame so stale glyphs from the previous
 *   algo or palette do not linger).
 */

typedef struct {
    Grid  grid;
    bool  paused;
    bool  needs_clear;
} Scene;

static void scene_init(Scene *scene, int cols, int rows, int theme)
{
    memset(scene, 0, sizeof *scene);
    grid_init(&scene->grid, cols, rows, theme);
}

static void scene_free(Scene *scene) { grid_free(&scene->grid); }

/*
 * scene_resize() — react to SIGWINCH by re-allocating at new size.
 *
 *   Pseudocode   : remember theme/fuel/wind/algo
 *                  grid_resize to new dimensions
 *                  restore remembered settings
 *                  reset warmup to 0; raise needs_clear
 *   Mental model : the world map is redrawn but the player's choices
 *                  (theme, fuel, wind, algo) survive the redraw.
 */
static void scene_resize(Scene *scene, int cols, int rows)
{
    int   saved_theme = scene->grid.theme;
    float saved_fuel  = scene->grid.fuel;
    int   saved_wind  = scene->grid.wind;
    int   saved_algo  = scene->grid.algo;

    grid_resize(&scene->grid, cols, rows);

    scene->grid.fuel   = saved_fuel;
    scene->grid.wind   = saved_wind;
    scene->grid.theme  = saved_theme;
    scene->grid.algo   = saved_algo;
    scene->grid.warmup = 0;
    scene->needs_clear = true;
}

static void scene_tick(Scene *scene)
{
    if (scene->paused) return;
    grid_tick(&scene->grid);
}

static void scene_draw(Scene *scene, int cols, int rows)
{
    grid_draw(&scene->grid, cols, rows);
}

/* ===================================================================== */
/* §20 screen + hud                                                       */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   The screen layer is the only place ncurses initialisation lives.
 *   screen_init() configures the terminal for full-screen rendering;
 *   screen_resize() handles SIGWINCH; screen_draw() composes the HUD;
 *   screen_present() flushes the diff to the terminal.
 *
 *   HUD layout:
 *     row 0           — bright yellow + bold:  fps, sim Hz, paused state
 *     row 1           — bright yellow:        theme, algo, fuel, wind, debug
 *     row rows - 1    — bright cyan + bold:   every interactive key
 */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *screen, int initial_theme)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);                /* don't let input interrupt diff write */
    color_init(initial_theme);
    getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_free(Screen *screen) { (void)screen; endwin(); }

static void screen_resize(Screen *screen)
{
    endwin();
    refresh();
    getmaxyx(stdscr, screen->rows, screen->cols);
}

static const char *algo_name(int algo_index)
{
    switch (algo_index) {
    case 0:  return "CA";
    case 1:  return "particles";
    case 2:  return "plasma";
    default: return "?";
    }
}

/*
 * screen_draw() — paint scene then HUD.
 *
 *   Purpose      : drive one frame's output.
 *   Pseudocode   : if needs_clear: erase(); clear flag
 *                  scene_draw(...)
 *                  row 0  bright-yellow bold "fps / Hz / paused"
 *                  row 1  bright-yellow      "theme / algo / fuel / wind / dbg"
 *                  row -1 bright-cyan  bold  "<keys>"
 *   Inputs       : screen, scene, fps_smoothed, sim_fps
 *   Output       : stdscr written, prev_heat updated by grid_draw
 *   Why it lives here: §20 is the only place that knows where the HUD
 *                      anchors live (row 0, row 1, row -1).
 */
static void screen_draw(Screen *screen, Scene *scene, double fps, int sim_fps)
{
    if (scene->needs_clear) { erase(); scene->needs_clear = false; }
    scene_draw(scene, screen->cols, screen->rows);

    const Grid *grid = &scene->grid;
    char buf[HUD_COLS + 1];

    /* row 0 — primary status (bright yellow + bold, right-aligned) */
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  %s ",
             fps, sim_fps, scene->paused ? "PAUSED " : "running");
    int row0_x = screen->cols - (int)strlen(buf);
    if (row0_x < 0) row0_x = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, row0_x, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* row 1 — secondary readouts (same pair, no bold, right-aligned) */
    const char *wind_arrow = grid->wind > 0 ? ">>>"
                          : grid->wind < 0 ? "<<<" : "---";
    snprintf(buf, sizeof buf,
             " theme:%s  algo:%s  fuel:%.2f  wind:%s  dbg:%s ",
             k_themes[grid->theme].name, algo_name(grid->algo),
             grid->fuel, wind_arrow, debug_mode_name(g_debug_mode));
    int row1_x = screen->cols - (int)strlen(buf);
    if (row1_x < 0) row1_x = 0;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, row1_x, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* row rows-1 — key hint (bright cyan + bold, left-aligned) */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(screen->rows - 1, 0,
        " q:quit  spc:pause  a:algo  t:theme  g/G:fuel  w/W:wind  0:calm  [/]:Hz  d/D:debug ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §21 application                                                        */
/* ===================================================================== */
/*
 * SECTION PREAMBLE
 *   The outermost layer: signal handlers, key bindings, fixed-timestep
 *   accumulator, fps averaging.  Render is pinned to 60 fps regardless
 *   of the simulation rate.  SIGINT / SIGTERM clear app->running;
 *   SIGWINCH raises app->need_resize so the main loop can re-init the
 *   grid on the next iteration.
 */

typedef struct {
    Scene  scene;
    Screen screen;
    int    sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App  g_app;
static void on_exit_signal  (int s) { (void)s; g_app.running     = 0; }
static void on_resize_signal(int s) { (void)s; g_app.need_resize = 1; }
static void cleanup_on_exit (void)  { endwin(); }

/*
 * app_handle_key() — dispatch one key press.
 *
 *   Purpose      : convert a keystroke into a state mutation.
 *   Pseudocode   : switch on character:
 *                      q,Q,ESC : return false (request shutdown)
 *                      space   : toggle paused
 *                      a,A     : next algorithm; reset warmup + particles
 *                      t,T     : next theme; reapply pairs
 *                      g       : fuel += 0.05 (capped at 1.0)
 *                      G       : fuel -= 0.05 (floored at 0.1)
 *                      w       : wind++ (capped at +WIND_MAX)
 *                      W       : wind-- (floored at −WIND_MAX)
 *                      0       : wind ← 0
 *                      ]       : sim_fps += STEP (capped at MAX)
 *                      [       : sim_fps -= STEP (floored at MIN)
 *                      d       : debug mode ← (mode + 1) % N_DEBUG_MODES
 *                      D       : debug mode ← (mode + N − 1) % N_DEBUG_MODES
 *   Mental model : a switch where every entry is a single, named user
 *                  intention.  Add a key by adding a case.
 *   Inputs       : app, character read by getch()
 *   Output       : returns false if the user wants to quit, true to keep
 *                  running
 *   Why it lives here: keeps every state mutation triggered by user
 *                      input in exactly one function.
 */
static bool app_handle_key(App *app, int ch)
{
    Grid *grid = &app->scene.grid;
    switch (ch) {
    case 'q': case 'Q': case 27:
        return false;

    case ' ':
        app->scene.paused = !app->scene.paused;
        break;

    case 'a': case 'A':
        grid->algo               = (grid->algo + 1) % N_ALGOS;
        grid->warmup             = 0;
        grid->wind_acc           = 0;
        memset(grid->parts, 0, sizeof grid->parts);
        grid->part_idx           = 0;
        app->scene.needs_clear   = true;
        break;

    case 't': case 'T':
        grid->theme              = (grid->theme + 1) % THEME_COUNT;
        theme_apply(grid->theme);
        app->scene.needs_clear   = true;
        break;

    case 'g':
        grid->fuel += 0.05f;
        if (grid->fuel > 1.0f) grid->fuel = 1.0f;
        break;
    case 'G':
        grid->fuel -= 0.05f;
        if (grid->fuel < 0.1f) grid->fuel = 0.1f;
        break;

    case 'w':
        grid->wind++;
        if (grid->wind >  WIND_MAX) grid->wind =  WIND_MAX;
        break;
    case 'W':
        grid->wind--;
        if (grid->wind < -WIND_MAX) grid->wind = -WIND_MAX;
        break;
    case '0':
        grid->wind = 0;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 'd':
        g_debug_mode = (DebugMode)(((int)g_debug_mode + 1) % N_DEBUG_MODES);
        app->scene.needs_clear = true;
        break;
    case 'D':
        g_debug_mode = (DebugMode)(((int)g_debug_mode + N_DEBUG_MODES - 1)
                                   % N_DEBUG_MODES);
        app->scene.needs_clear = true;
        break;

    default:
        break;
    }
    return true;
}

/*
 * main() — fixed-timestep loop with 60 fps render cap.
 *
 *   Purpose      : entry point; orchestrates the simulation, the
 *                  renderer, the input pump, and the resize flag.
 *   Pseudocode   :
 *      seed RNG; register signal handlers and atexit cleanup.
 *      screen_init; scene_init.
 *      loop:
 *          if need_resize: screen_resize; scene_resize; clear sim_acc.
 *          now ← clock_ns(); dt ← clamped(now - frame_time)
 *          frame_time ← now
 *          sim_accumulator += dt
 *          while sim_accumulator ≥ TICK_NS(sim_fps):
 *              scene_tick(); sim_accumulator -= TICK_NS(sim_fps)
 *          fps_window_count += 1; fps_window_ns += dt
 *          if fps_window_ns ≥ FPS_UPDATE_MS:
 *              fps_smoothed ← count / (window_ns / 1e9); reset
 *          clock_sleep_ns(1/60 s − elapsed_this_iteration)
 *          screen_draw(); screen_present()
 *          ch ← getch(); if app_handle_key returns false: exit
 *      cleanup.
 *   Mental model : a heartbeat — every cycle does the same six things
 *                  in the same order.
 */
int main(void)
{
    srand((unsigned int)clock_ns());
    atexit(cleanup_on_exit);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app      = &g_app;
    app->running  = 1;
    app->sim_fps  = SIM_FPS_DEFAULT;

    screen_init(&app->screen, 0);
    scene_init(&app->scene, app->screen.cols, app->screen.rows, 0);

    int64_t frame_time_ns   = clock_ns();
    int64_t sim_accum_ns    = 0;
    int64_t fps_window_ns   = 0;
    int     fps_frame_count = 0;
    double  fps_smoothed    = 0.0;

    while (app->running) {
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_resize(&app->scene, app->screen.cols, app->screen.rows);
            app->need_resize  = 0;
            frame_time_ns     = clock_ns();
            sim_accum_ns      = 0;
        }

        int64_t now          = clock_ns();
        int64_t delta_ns     = now - frame_time_ns;
        frame_time_ns        = now;
        if (delta_ns > 100 * NS_PER_MS) delta_ns = 100 * NS_PER_MS;

        int64_t tick_period_ns = TICK_NS(app->sim_fps);
        sim_accum_ns          += delta_ns;
        while (sim_accum_ns >= tick_period_ns) {
            scene_tick(&app->scene);
            sim_accum_ns -= tick_period_ns;
        }

        fps_frame_count++;
        fps_window_ns += delta_ns;
        if (fps_window_ns >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_smoothed   = (double)fps_frame_count /
                             ((double)fps_window_ns / (double)NS_PER_SEC);
            fps_frame_count = 0;
            fps_window_ns   = 0;
        }

        int64_t elapsed_this_iteration = clock_ns() - frame_time_ns + delta_ns;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed_this_iteration);

        screen_draw(&app->screen, &app->scene, fps_smoothed, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
