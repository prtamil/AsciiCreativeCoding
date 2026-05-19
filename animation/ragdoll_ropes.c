/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ragdoll_ropes.c — seven Verlet ropes swaying in sinusoidal wind
 *
 * DEMO: Seven ropes of varying lengths hang from a ceiling and sway under a
 *       sinusoidal wind whose phase is offset per rope, producing a Mexican-
 *       wave sway. Each rope is 20 particles bound by distance constraints.
 *
 * Study alongside: ragdoll_figure.c (the same Verlet engine, branched skeleton)
 *
 * Section map:
 *   §1 config   — tunables: gravity, damping, wind defaults, geometry
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 10 themes × 7 rope pairs + PAIR_HUD/PAIR_HINT
 *   §4 coords   — pixel↔cell aspect-ratio bridge
 *   §5 entity   — Scene: Verlet ropes + constraint solver + renderer
 *       §5a rope_verlet_step      — gravity + wind integration per particle
 *       §5b apply_rope_constraints — N_ITERS passes of distance projection
 *       §5c enforce_anchors       — pin particle[0] to its ceiling slot
 *       §5d apply_wind            — sinusoidal lateral force per rope
 *       §5e rope_node_char/_attr  — bead glyph + brightness gradient
 *       §5f draw_rope_beads       — two-pass per-rope render
 *       §5g render_scene          — full frame: ceiling + all ropes
 *   §6 scene    — scene_init / scene_tick / scene_draw
 *   §7 screen   — ncurses double-buffer display layer
 *   §8 app      — signals, resize, main game loop
 *
 * Keys:
 *   q / ESC       quit
 *   space         pause / resume
 *   w/s, ↑/↓      wind amplitude ± 50 px/s²
 *   a/d, ←/→      wind frequency ± 0.05 rad/s
 *   r / R         reset (ropes back to straight, theme preserved)
 *   t / T         next / previous theme
 *   [ / ]         sim Hz − / +
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       ragdoll_ropes.c -o ragdoll_ropes -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Position-Verlet integration with iterative distance-
 *                  constraint projection (Jakobsen). Each rope is a chain
 *                  of N_SEG particles. Each tick: gravity + a sinusoidal
 *                  lateral wind are integrated, then six constraint passes
 *                  pull every adjacent pair back to its rest length. The
 *                  ceiling anchor is re-pinned (both pos AND old_pos) after
 *                  every pass so constraint corrections cannot drift it.
 *                  Per-rope phase offsets (r · 2π / N_ROPES) make ropes
 *                  oscillate out of step — the Mexican-wave look.
 *
 * Data-structure : Scene packs flat 2-D arrays pos[N_ROPES][N_SEG],
 *                  old_pos[][] and prev_pos[][]. rest_len[r] holds the
 *                  per-rope segment length (= rope_len_px[r] / (N_SEG−1)).
 *                  anchor[r] is the fixed ceiling pixel position; a
 *                  pre-computed phase_offset[r] is added to wind_time on
 *                  every sin() call. All positions live in pixel space so
 *                  forces and lengths stay isotropic across cell aspect.
 *
 * Rendering      : Two-pass bead style. Pass 1 walks each segment in 2 px
 *                  increments stamping 'o' with cell-dedup to avoid attr
 *                  flicker. Pass 2 over-stamps each particle with a size-
 *                  graded node ('0' A_BOLD top quarter / 'o' middle / '.'
 *                  A_DIM bottom quarter), so the eye reads tension as a
 *                  brightness gradient. The free tip gets an A_BOLD 'o'
 *                  weight marker. Alpha-interpolated prev_pos → pos keeps
 *                  motion smooth at any render rate.
 *
 * Performance    : 7 × 20 = 140 particles. Six constraint passes × 7 ropes
 *                  × 19 segments = 798 projections per tick — trivial.
 *                  ncurses doupdate is the bottleneck (~200-400 changed
 *                  cells per frame).
 *
 * References
 * ──────────
 *   ── Verlet integration (the §5a integrator) ──────────────────────
 *   [1] Verlet, L. (1967), "Computer Experiments on Classical Fluids.
 *       I.", Phys. Rev. 159(1), pp. 98-103 — the original integrator.
 *       The position-only form used by implicit_velocity() dates from
 *       this paper.
 *   [2] Swope, W. C. et al. (1982), "A computer simulation method for
 *       the calculation of equilibrium constants...", J. Chem. Phys.
 *       76 — velocity-Verlet variant. We use Verlet's ORIGINAL
 *       position-only form (sometimes called Störmer-Verlet).
 *
 *   ── Position-Based Dynamics (the §5b constraint solver) ─────────
 *   [3] Jakobsen, T. (2001), "Advanced Character Physics", GDC —
 *       THE canonical Verlet-rope paper. §2 integrator, §3 distance-
 *       constraint relaxation, §4 collisions. The whole §5 pipeline
 *       follows this paper.
 *       https://www.cs.cmu.edu/afs/cs/academic/class/15462-s13/www/lec_slides/Jakobsen.pdf
 *   [4] Müller, M., Heidelberger, B., Hennix, M. & Ratcliff, J. (2007),
 *       "Position Based Dynamics", J. Vis. Comm. Image Repr. 18(2) —
 *       generalises Jakobsen's distance-only projection to arbitrary
 *       constraint types (bending, volume, etc.).
 *   [5] Bender, J., Müller, M. & Macklin, M. (2017), "A Survey on
 *       Position-Based Simulation Methods in Computer Graphics",
 *       Computer Graphics Forum 36(6) — modern survey covering XPBD
 *       and stability extensions.
 *   [6] Stam, J. (2002), "Real-Time Stable Cloth and Hair", GDC —
 *       same projection idea applied to MANY parallel chains; the
 *       per-rope phase_offset stagger here borrows the "decorrelate
 *       to avoid synchronised look" trick.
 *
 *   ── Numerical methods (Gauss-Seidel ordering) ────────────────────
 *   [7] Saad, Y. (2003), "Iterative Methods for Sparse Linear Systems"
 *       (2nd ed.), SIAM — §4.1 Gauss-Seidel vs Jacobi; relax_rope_chain
 *       uses the Gauss-Seidel ordering (each correction propagates to
 *       the next constraint via the shared endpoint).
 *
 *   ── Classical mechanics (the physics of impulses + collisions) ──
 *   [8] Goldstein, H. (1980), "Classical Mechanics" (2nd ed.),
 *       Addison-Wesley — §2 simple harmonic motion (basis for
 *       sinusoidal_wind_acceleration_at); §3.6 coefficient of
 *       restitution (basis for bounce_against_wall_1d).
 *
 *   ── Timestep / animation ─────────────────────────────────────────
 *   [9] Fiedler, G. (2004), "Fix Your Timestep!", gafferongames.com —
 *       the fixed-step accumulator pattern this file uses; alpha
 *       interpolation between prev_pos and pos comes from here.
 *
 *   ── Rendering / ncurses ─────────────────────────────────────────
 *  [10] Bresenham, J. E. (1965), "Algorithm for computer control of
 *       a digital plotter", IBM Systems Journal 4(1), pp. 25-30 —
 *       the integer line algorithm; §5g draw_rope_beads uses the
 *       parametric oversample because float math is already paid
 *       for in the physics step.
 *  [11] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — basis for
 *       the head/middle/tip bead glyph hierarchy.
 *  [12] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; covers init_pair,
 *       use_default_colors, and the diff pipeline §7 relies on.
 *
 *   ── Online quick reference ───────────────────────────────────────
 *  [13] https://en.wikipedia.org/wiki/Verlet_integration
 *  [14] https://en.wikipedia.org/wiki/Position-based_dynamics
 *  [15] https://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method
 *  [16] https://en.wikipedia.org/wiki/Coefficient_of_restitution
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A rope is just a string of beads being pulled around by gravity and
 * wind, with the rule "neighbouring beads must stay exactly L apart"
 * enforced as a *post-step correction*: integrate first, then project
 * each pair back to the right distance, repeat six times. There is no
 * spring constant, no implicit-velocity solve, no stiffness parameter
 * to tune — the inextensibility falls out of pure geometry.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine seven dangling necklaces side-by-side. Every tick the wind
 * gives each necklace a horizontal nudge — but the wind gusts hit each
 * necklace at a slightly different moment in its cycle, so they never
 * sway together. After every nudge the chain stretches a hair, and you
 * pinch each link back to the right length six times in a row. That
 * pinch is the constraint. The top bead is glued to the ceiling: every
 * time the pinch tries to drag it, you snap it back and zero its
 * implicit velocity by overwriting *both* pos and old_pos.
 *
 * DRAWING METHOD / ALGORITHM IN STEPS
 * ───────────────────────────────────
 *   1. Save prev_pos = pos                (anchor for sub-tick lerp)
 *   2. Advance wind_time by dt
 *   3. For each rope r:
 *      a. wind_x = wind_amp · sin(wind_time · wind_freq + phase[r])
 *      b. For each free particle s ∈ [1, N_SEG):
 *           vel     = (pos − old_pos) · ROPE_DAMPING
 *           old_pos = pos
 *           pos    += vel + (wind_x, gravity) · dt²
 *      c. Clamp particles to floor / walls; reflect old_pos for bounce.
 *   4. Pin every anchor: pos[r][0] = old_pos[r][0] = anchor[r].
 *   5. Repeat N_ITERS times: walk every segment, project both endpoints
 *      by half the length error along the segment direction; re-pin
 *      anchors after each iteration so the projection cannot drag them.
 *   6. Render: lerp prev_pos → pos by alpha; pass-1 fill bead 'o' along
 *      every segment; pass-2 over-stamp graded particle markers.
 *
 * KEY FORMULAS
 * ────────────
 *   Verlet step (per particle, per tick):
 *     vel     = (pos − old_pos) · ROPE_DAMPING
 *     old_pos = pos
 *     pos    += vel + accel · dt²
 *
 *   Distance constraint (per segment, per pass):
 *     v     = pos[s+1] − pos[s]
 *     dist  = |v|
 *     error = (dist − rest) / dist
 *     pos[s]   += 0.5 · error · v
 *     pos[s+1] -= 0.5 · error · v
 *
 *   Sinusoidal wind (per rope, per tick):
 *     accel_x = wind_amp · sin(wind_time · wind_freq + r · 2π / N_ROPES)
 *
 *   Pixel→cell aspect bridge:
 *     cx = round(px / CELL_W)         (CELL_W = 8)
 *     cy = round(py / CELL_H)         (CELL_H = 16)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Coincident particles → dist = 0 → divide by zero. Guard with
 *     `if (dist < 1e-6f) continue;` in apply_rope_constraints.
 *   • Pinning only pos[r][0] (not old_pos[r][0]) leaves an implicit
 *     velocity (anchor − displaced_old_pos) that yanks the anchor off
 *     on the very next tick — drift compounds. Always reset BOTH.
 *   • Constraint projection is equal-mass; one pass nudges the anchor
 *     too. Re-pin after every pass, not just at the end.
 *   • Sub-tick lerp anchor (prev_pos) MUST be saved before any physics
 *     this tick, otherwise the render shows overshoot past the new state.
 *   • Frame cap: never add dt to elapsed (`elapsed = clock_ns() − frame_start`,
 *     no `+ dt`) — adding dt cancels the cap and pegs CPU at 100 %.
 *   • Resize: rope_len_px is recomputed from `rows`, so scene_init must
 *     be re-called with the new dimensions; otherwise lengths are stale.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Set wind_amp = 0 (s key until 0): all 7 ropes hang dead-vertical
 *     with no drift over many seconds — proves the anchor pin is solid.
 *   • Pause (space): every particle freezes in place; no perceptible
 *     micro-stutter at the freeze frame (proves prev_pos save order).
 *   • Crank wind_amp to 1000: ropes whip wildly but never stretch — proves
 *     6 constraint passes are enough.
 *   • Cycle themes (t/T): every theme is legible against a default-bg
 *     terminal under A_DIM (no rope segment "disappears" at the bottom
 *     quarter). Confirms theme palettes obey the brightness rule.
 *   • Sweep wind_freq from 0.05 → 4.0 (d key): visible period should
 *     scale as 2π/wind_freq — at 0.4 ≈ 16 s/cycle, at 4.0 ≈ 1.6 s/cycle.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read ragdoll_figure.c first; the Verlet + Jakobsen
 *      machinery (T2-T3 there) is reused unchanged. The NEW lessons
 *      here are: anchor pinning, sinusoidal wind, and per-rope phase
 *      detuning.
 *   2. §5 entity — THE HEART. In sub-section order:
 *        §5a rope_verlet_step        ← Verlet (read AFTER ragdoll T2)
 *        §5b apply_rope_constraints  ← Jakobsen (ragdoll T3)
 *        §5c enforce_anchors         ← T1 below — the new lesson
 *        §5d apply_wind              ← T2 below
 *        §5e-§5f rendering
 *   3. §6 scene — orchestrator with prev/cur snapshot.
 *   4. §1-§4 + §7-§8 — infrastructure. Skim if seen.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   pos[r][s]               rope r, segment s — current position.
 *   old_pos[r][s]           rope r, segment s — previous position.
 *   prev_pos[r][s]          start-of-tick snapshot for alpha lerp.
 *   anchor[r]               fixed ceiling pixel for rope r.
 *   rest_len[r]             rope r's per-segment rest length.
 *   phase_offset[r]         per-rope wind-phase offset (rad).
 *   wind_time               master wind clock (s).
 *   wind_amp, wind_freq     wind acceleration amplitude (px/s²)
 *                           and frequency (rad/s).
 *   N_ROPES, N_SEG          7 ropes, 20 particles each.
 *   N_ITERS                 6 — Jakobsen passes per tick.
 *   ROPE_DAMPING            energy-loss multiplier on implicit vel.
 *
 * Background you need
 * ───────────────────
 *   - Verlet + Jakobsen (ragdoll_figure.c T2-T3). The integrator
 *     and constraint solver here are IDENTICAL.
 *   - Sinusoid arguments — same as fk_tentacle_forest T2.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Bending stiffness or twist constraints. Ropes flop freely
 *     between segments (Jakobsen distance constraint only).
 *   - Aerodynamic drag. Wind here is a per-particle FORCE, not a
 *     velocity-dependent drag. Real cloth needs proper drag for
 *     realism; we don't.
 *   - Self-intersection. Ropes can pass through each other; we
 *     don't simulate inter-rope collision.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Five tutorials that build a Verlet rope from first principles
 * AND scale it to a parallel array.
 *
 *   T1  Pinning the anchor — why both pos AND old_pos must be reset
 *   T2  Sinusoidal wind — environmental force as a per-tick acceleration
 *   T3  Per-rope phase detuning — making seven ropes look like seven
 *   T4  Rope vs ragdoll — what changes when the graph is a chain
 *   T5  Why we re-anchor INSIDE the constraint loop
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  PINNING THE ANCHOR — WHY BOTH pos AND old_pos MUST BE RESET
 * ───────────────────────────────────────────────────────────────
 * The top particle of each rope is FIXED to the ceiling. That's
 * the simplest "external constraint" — a position that never
 * moves no matter what physics says.
 *
 * Naive pin: just clamp pos[r][0] to anchor[r] every tick.
 *
 *     pos[r][0] = anchor[r]
 *
 * That's INCORRECT. In Verlet, the implicit velocity is
 * (pos − old_pos). If we change pos but leave old_pos at its
 * displaced value, the next tick computes a velocity equal to
 * (anchor − displaced_old_pos) — a phantom velocity that yanks
 * the anchor off!
 *
 * Correct pin:
 *
 *     pos    [r][0] = anchor[r]
 *     old_pos[r][0] = anchor[r]      ← BOTH
 *
 * This sets the anchor's velocity to zero by making the
 * difference (pos − old_pos) zero. The anchor stays put.
 *
 * Same idea applies to ANY external constraint in a Verlet
 * simulation: when you snap a particle's POSITION, also snap
 * its OLD position to define the implied velocity.
 *
 * If you wanted the anchor to MOVE on a path, set:
 *
 *     pos    [r][0] = anchor_path(t)
 *     old_pos[r][0] = anchor_path(t − dt)
 *
 * The implied velocity is (anchor_path(t) − anchor_path(t-dt))
 * — the anchor's actual velocity. The rope follows.
 *
 * T2  SINUSOIDAL WIND — ENVIRONMENTAL FORCE AS A PER-TICK ACCELERATION
 * ────────────────────────────────────────────────────────────────────
 * Wind is just a sideways acceleration applied to every
 * non-anchor particle:
 *
 *     accel.x = WIND.x + wind_amp · sin(wind_time · wind_freq
 *                                       + phase_offset[r])
 *     accel.y = GRAVITY
 *
 * The Verlet step sees this acceleration and integrates it:
 *
 *     pos += vel + accel · dt²
 *
 * That's literally Newton's second law as a numerical update —
 * F = ma, integrated with Verlet.
 *
 * No need for a separate "wind force" function or aerodynamics
 * model. We treat wind as a uniformly-applied lateral force.
 * Cheap, controllable, looks plausible enough for a terminal
 * demo.
 *
 * Frequency and amplitude are user-tunable via arrow keys.
 * Watching wind_freq sweep from 0.05 to 4.0 rad/s is
 * surprisingly hypnotic — the rope's mechanical resonance
 * shows up as some frequencies producing larger swings than
 * others.
 *
 * T3  PER-ROPE PHASE DETUNING — MAKING SEVEN ROPES LOOK LIKE SEVEN
 * ────────────────────────────────────────────────────────────────
 * Same trick as fk_tentacle_forest T4. Without per-rope
 * detuning, all seven ropes would oscillate in lockstep — they
 * see the SAME wind, with the same amplitude, frequency, and
 * starting phase.
 *
 * Phase offset:
 *
 *     phase_offset[r] = r · 2π / N_ROPES        ← evenly spaced
 *
 * Each rope sees a wind argument shifted by phase_offset[r],
 * so the seven ropes are at seven evenly-distributed points
 * in the wind's cycle at any moment. The visual is a
 * MEXICAN-WAVE SWAY — left-to-right propagating wave across
 * the row.
 *
 * Notice how this is the SIMPLEST kind of detuning — fixed
 * phase, no frequency variation. fk_tentacle_forest also
 * detunes FREQUENCIES so the eight tentacles never re-sync;
 * here we don't because the user expects the wave to
 * RECOGNISABLY repeat (it's an effect, not a swarm).
 *
 * T4  ROPE VS RAGDOLL — WHAT CHANGES WHEN THE GRAPH IS A CHAIN
 * ────────────────────────────────────────────────────────────
 * The difference between this file and ragdoll_figure.c is
 * the GRAPH STRUCTURE of the constraint network:
 *
 *   ragdoll_figure  branched skeleton (head — neck — shoulders —
 *                   arms / spine — hips — legs / ...)
 *   ragdoll_ropes   pure chain (anchor — bead — bead — ... — tip)
 *
 * Same Verlet step, same Jakobsen iteration, different topology.
 *
 * Implications:
 *   - Chain has fewer constraints per particle (each particle
 *     has at most 2 neighbours; ragdoll spine particles have
 *     3-4). Convergence is FASTER for chains.
 *   - Chain stretches more easily under heavy loads at the tip
 *     (compounding error along a long chain). Add more iterations
 *     or shorten the chain.
 *   - Chain's "rest pose" is straight; ragdoll's is a T-pose with
 *     specific joint angles. Both are arbitrary choices baked into
 *     constraint rest lengths.
 *
 * The CODE difference is minimal — the constraint solver in §5b
 * differs from ragdoll_figure §5c only in indexing (here we know
 * each constraint connects [s, s+1] in the same rope, simpler
 * loops).
 *
 * T5  WHY WE RE-ANCHOR INSIDE THE CONSTRAINT LOOP
 * ───────────────────────────────────────────────
 * Same structural concern as ragdoll_figure T4: the constraint
 * pass YANKS particles around, and an outer-loop anchor pin
 * can drift if a constraint pulls it.
 *
 *   for iter in 1..N_ITERS:
 *     for s in 0..N_SEG-2:
 *       satisfy(pos[r][s], pos[r][s+1])
 *     enforce_anchors()           ← INSIDE the iteration loop
 *
 * If you only enforce_anchors after the outer loop:
 *
 *   for iter in 1..N_ITERS:
 *     ...satisfy passes...
 *   enforce_anchors()             ← only ONCE
 *
 * Then iteration 1's satisfy pass yanks the anchor a small
 * amount; iteration 2 yanks it a bit more (compounding). By
 * iteration 6, the anchor is visibly above or below its true
 * position. The single end-of-loop pin clamps it back — but
 * the rope spent 5 of 6 iterations operating with a wrong
 * anchor location, so the chain settles into the wrong
 * shape.
 *
 * Re-pinning every iteration costs a couple of memory writes
 * per rope per iteration — utterly negligible — and produces
 * a visibly stiffer, more anchored result.
 *
 * Same principle is reused in ragdoll_figure (§5d ragdoll_tick
 * re-applies platform collisions inside the constraint loop)
 * and in any production cloth/rope simulator.
 *
 * Generalised rule: any external position constraint must be
 * applied AFTER EVERY internal constraint iteration, not just
 * once at the end. Otherwise convergence drags you off the
 * pinned points.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

/*
 * M_PI is a POSIX extension, not standard C99/C11.
 * _POSIX_C_SOURCE 200809L exposes it on most systems, but if a toolchain
 * omits it we define our own constant so the build never fails.
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
/* §1  config                                                             */
/* ===================================================================== */

/*
 * All magic numbers live here.  Change behaviour by editing this block
 * only — never scatter literals through the code.
 */
enum {
    /*
     * SIM_FPS_DEFAULT — target physics tick rate in Hz.
     *
     * The fixed-step accumulator in §8 fires scene_tick() this many times
     * per wall-clock second regardless of render frame rate.  Raising sim
     * Hz improves constraint accuracy (more passes per wall-clock second)
     * but costs more CPU.  [10, 120] is the user-adjustable range; default
     * 60 matches the 60-fps render cap so one physics tick fires per frame.
     *
     *   SIM_FPS_STEP — increment size for the [ and ] keys.
     */
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    /*
     * HUD_COLS — byte budget for the status-bar string built by screen_draw().
     * snprintf() uses this as the buffer size; 96 bytes covers the longest
     * possible HUD string at maximum parameter values.
     *
     * FPS_UPDATE_MS — milliseconds between fps display recalculations.
     * 500 ms gives a stable reading without per-frame flicker.
     */
    HUD_COLS         =  96,
    FPS_UPDATE_MS    = 500,

    /*
     * N_PAIRS — number of rope colour pairs registered with ncurses (1–7).
     *   Rope r uses colour pair (r % N_PAIRS) + 1, cycling through all seven.
     *
     * PAIR_HUD / PAIR_HINT — dedicated pairs for the top status and bottom
     *   key-hint bars per the project HUD spec.  Theme-independent: their
     *   colours never shift when the rope palette cycles, so the HUD stays
     *   readable against any backdrop.
     *
     * N_THEMES — number of switchable palettes in the THEMES[] array (§3).
     *   Cycled at runtime with the t / T keys.
     */
    N_PAIRS          =   7,
    PAIR_HUD         =   8,   /* bright yellow — top status bar  */
    PAIR_HINT        =   9,   /* bright cyan   — bottom key hint */
    N_THEMES         =  10,

    /*
     * N_ROPES — number of independent rope chains hanging from the ceiling.
     *   Seven ropes produce the classic "wind chime" visual density without
     *   crowding the terminal.  Each rope gets its own colour pair and its
     *   own phase offset in the wind oscillation.
     *
     * N_SEG — particles per rope, including the fixed anchor at index [0].
     *   With N_SEG = 20 and rest lengths ranging ~12–28 px, rope visual
     *   lengths range from 35% to 75% of screen height.  More segments
     *   produce smoother curves but cost more constraint iterations.
     *
     * N_ITERS — distance constraint passes per tick (CONSTRAINT_ITERS).
     *   Each pass reduces the positional error roughly by half for a simple
     *   chain.  Six passes converge a 20-node rope to < 0.1% length error,
     *   which is imperceptible.  (ragdoll_figure.c uses 10+ because its
     *   skeleton has branching joints that need more passes to stabilise.)
     */
    N_ROPES = 7,
    N_SEG   = 20,
    N_ITERS =  6,
};

/*
 * GRAVITY — downward acceleration applied to every non-anchor particle (px/s²).
 *
 *   +y is downward in screen space (terminal convention).
 *   800 px/s² with CELL_H = 16 px/row means particles fall at
 *   800/16 = 50 rows/s² — fast enough to keep ropes taut under wind
 *   without making them rigid.
 *
 *   Compared with ragdoll_figure.c (also 800 px/s²): same gravity model,
 *   because ropes and the ragdoll live in the same pixel-space world.
 *
 * ROPE_DAMPING — per-tick velocity scale factor applied in rope_verlet_step().
 *
 *   vel_new = vel_old × ROPE_DAMPING
 *
 *   0.992 means 0.8% velocity loss per tick.  Compare with ragdoll_figure.c
 *   (0.995 = 0.5% loss per tick).  Rope segments are thin and whippy —
 *   physically they have more air resistance than rigid bones.  The slightly
 *   stronger drag prevents ropes from oscillating wildly at high wind_amp
 *   values while keeping visible sway at lower amplitudes.
 *
 *   At 60 Hz: 0.992^60 ≈ 0.619 — a rope released from any initial velocity
 *   loses ~38% of its speed after one second of free sway.
 *
 * BOUNCE_COEFF — fraction of velocity retained after a floor or wall
 *   collision.  0.5 = 50% energy retained; the rope tip bounces but
 *   not elastically.  Values < 0.3 look dead; > 0.8 look rubbery.
 */
#define GRAVITY          800.0f
#define ROPE_DAMPING       0.992f
#define BOUNCE_COEFF       0.5f

/*
 * WIND_AMP_DEFAULT — default lateral wind force amplitude (px/s²).
 *
 *   250 px/s² at 60 Hz adds ≈ 250 × (1/60)² ≈ 0.069 px of lateral
 *   displacement per tick before damping.  Over many ticks this accumulates
 *   to a visible lateral sway of several columns — noticeable but not violent.
 *   Maximum allowed at runtime: 1000 px/s² (↑ key).
 *   Minimum: 0 px/s² (ropes hang straight, only gravity).
 *
 * WIND_FREQ_DEFAULT — default wind oscillation angular frequency (rad/s).
 *
 *   0.4 rad/s → period = 2π / 0.4 ≈ 15.7 seconds per full left-right cycle.
 *   This is a slow, languorous sway — like ropes in a light breeze.
 *   Increasing toward 4.0 rad/s (→ key) produces a rapid rattling shake.
 *   Minimum: 0.05 rad/s → ~126-second period (almost imperceptibly slow).
 *
 * ANCHOR_ROW_CELLS — terminal row of the ceiling anchor line.
 *
 *   Row 2 (third from top) keeps the '#' ceiling line and the 'T' anchor
 *   markers visible below the status bar at row 0, with one blank row at
 *   row 1 as a visual separator.  All rope physics starts from this row.
 */
#define WIND_AMP_DEFAULT   250.0f
#define WIND_FREQ_DEFAULT    0.4f
#define ANCHOR_ROW_CELLS     2

/*
 * Boundary margins in pixel space — how far from each physical edge
 * particles are clamped by rope_boundaries() to prevent them escaping
 * the visible area.
 *
 *   FLOOR_MARGIN  — gap in px above the bottom pixel row.
 *     8 px = 0.5 terminal rows: the floor is just below the last visible row.
 *   LEFT_MARGIN / RIGHT_MARGIN — gap in px from left and right edges.
 *     16 px = 2 terminal columns: a small safety band at each side.
 */
#define FLOOR_MARGIN   8.0f
#define LEFT_MARGIN   16.0f
#define RIGHT_MARGIN  16.0f

/*
 * DRAW_STEP_PX — pixel step used in the dense segment-fill pass of
 * draw_rope_beads().
 *
 * The renderer walks each rope segment in 2 px increments, converting each
 * sample to a terminal cell and stamping an 'o'.  The step must satisfy:
 *   DRAW_STEP_PX < CELL_W  (8 px)
 * so that no terminal cell along a near-horizontal segment is ever skipped.
 *
 * 2 px is intentionally finer than snake_forward_kinematics.c's 3 px
 * because rope segments are often near-vertical (hanging down) and a
 * 16 px tall cell would be skipped at 3 px steps if the angle is steep.
 * At 2 px: even a perfectly vertical segment (angle = 90°) is sampled at
 *   2 / sin(90°) = 2 px steps along y → 2 / 16 = 0.125 cells/step: fine.
 */
#define DRAW_STEP_PX   2.0f

/*
 * Timing primitives — verbatim from framework.c.
 *
 * NS_PER_SEC / NS_PER_MS — unit conversion constants.
 * TICK_NS(f) — period of one physics tick at frequency f Hz, in nanoseconds.
 *   e.g. TICK_NS(60) = 1 000 000 000 / 60 = 16 666 666 ns ≈ 16.7 ms.
 * Used in the fixed-step accumulator in §8.
 */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Terminal cell dimensions — the aspect-ratio bridge between physics
 * and display (see §4 for the full explanation).
 *
 * CELL_W = 8 px, CELL_H = 16 px.
 * A typical terminal cell is about 2× taller than wide in physical pixels.
 * All particle positions are stored in a square pixel space scaled by these
 * constants so that 1 pixel represents the same physical distance in x and y.
 * Only at draw time are positions converted to (col, row) cell coordinates.
 */
#define CELL_W   8    /* physical pixels per terminal column */
#define CELL_H  16    /* physical pixels per terminal row    */

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/*
 * clock_ns() — monotonic wall-clock in nanoseconds.
 *
 * CLOCK_MONOTONIC never goes backward (unlike CLOCK_REALTIME, which can
 * jump on NTP adjustments or DST transitions).  Subtracting two consecutive
 * clock_ns() calls gives the true elapsed wall time regardless of system
 * load or clock adjustments.
 *
 * Returns int64_t (signed 64-bit) so subtraction differences are naturally
 * signed — negative values from the pause-guard cap do not wrap.
 */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/*
 * clock_sleep_ns() — sleep for exactly ns nanoseconds.
 *
 * Called before render in §8 to cap the frame rate at 60 fps.  Sleeping
 * before (not after) the terminal write means only the physics computation
 * time is charged against the frame budget; the ncurses I/O cost does not
 * bleed into the next frame's elapsed time.
 *
 * If ns ≤ 0 the frame is already over-budget (physics took too long);
 * return immediately rather than sleeping a negative duration.
 */
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
/* §3  color — 10-theme palette system (7 rope pairs + HUD pair)         */
/* ===================================================================== */

/*
 * Theme — one complete named colour palette.
 *
 *   name   — displayed in the HUD status bar.
 *   body[] — 7 xterm-256 foreground colour indices for rope pairs 1–7
 *            (rope r always uses pair (r%7)+1).
 *
 * Brightness rule: every body[] entry is in the bright half of the
 * 256-colour cube (≥ 24).  Indices 16–23 (cube near-blacks) and 232–239
 * (gray near-blacks) become invisible under A_DIM at the bottom-quarter
 * tip of each rope, so we avoid them entirely.
 *
 * theme_apply() registers the chosen palette with ncurses init_pair() live;
 * switching themes takes effect on the very next frame without restarting.
 * On terminals with < 256 colours, theme_apply() falls back to 8 ANSI
 * colours that approximate the intended rainbow order.
 */
/*
 * Theme — one named multi-colour palette for the seven rope pairs.
 *
 * Intent
 *   Each theme rebinds pairs 1..N_PAIRS via init_pair (see
 *   theme_apply). Each rope r uses pair (r + 1), so a theme is
 *   really a CHOICE of seven distinct hues — one per rope — that
 *   together read as a coherent gradient or pattern.
 *
 *   HUD/HINT pairs (8, 9) are NOT theme-bound — they stay bright
 *   yellow / bright cyan across every theme so the status bar
 *   remains readable against any palette (CLAUDE.md HUD spec).
 *
 * Slot semantics (body[0..6] → pairs 1..7 → ropes 0..6)
 *   body[r] is the foreground colour for ROPE r. Themes are usually
 *   designed as a gradient (rope 0 dimmest → rope 6 brightest) so
 *   the eye reads the bank of ropes left-to-right as a sweep.
 *
 * Brightness rule (CLAUDE.md "Theme Palette Brightness")
 *   Every entry sits in the BRIGHT HALF of the 256-colour cube —
 *   cube indices ≥ 24, grayscale ≥ 240 — so even A_DIM beads stay
 *   visible. theme_apply falls back to the 8-colour ANSI palette
 *   when COLORS < 256.
 *
 * References [11] Bourke for the gradient-as-depth pattern;
 *   [12] Raymond §init_pair for the rebind mechanism.
 */
typedef struct {
    const char *name;              /* HUD-displayable theme name        */
    int         body[N_PAIRS];     /* xterm-256 fg per rope (0..6)      *
                                    * pair (r+1) = body[r] at apply time */
} Theme;

/*
 * THEMES[10] — the ten built-in palettes.  Column order is
 * pair 1 (rope 0) → pair 7 (rope 6).
 *
 *   Rainbow — classic ROYGBIV.
 *   Neon    — hot pinks, yellows, greens — club-light.
 *   Fire    — deep red to pale yellow — glowing embers.
 *   Ocean   — navy to aqua — cool gradient (lowest tier bumped to 24).
 *   Aurora  — green to violet — northern lights (lowest tier bumped to 24).
 *   Lava    — dark red to orange — molten glow.
 *   Forest  — multiple greens (lowest tier bumped to 24).
 *   Sunset  — purple to warm orange — dusk.
 *   Ice     — light blue to cyan — crystalline.
 *   Matrix  — dark to bright green (lowest tier bumped to 24).
 */
static const Theme THEMES[N_THEMES] = {
    /* name       rope0  rope1  rope2  rope3  rope4  rope5  rope6 */
    {"Rainbow", {196,   208,   226,    46,    51,    27,   129}},
    {"Neon",    {201,   226,   118,   159,   213,   208,    15}},
    {"Fire",    {124,   160,   196,   202,   208,   214,   220}},
    {"Ocean",   { 24,    25,    31,    33,    39,    45,    51}},
    {"Aurora",  { 28,    34,    79,   122,   159,   165,   201}},
    {"Lava",    { 52,    88,   124,   160,   196,   202,   208}},
    {"Forest",  { 28,    34,    40,    70,    76,   106,    82}},
    {"Sunset",  { 54,    91,   128,   165,   202,   209,   208}},
    {"Ice",     {195,   159,   123,    87,    51,    45,    39}},
    {"Matrix",  { 28,    34,    40,    46,    76,    82,   118}},
};

/*
 * theme_apply() — register rope palette idx with ncurses init_pair().
 *
 * Called from color_init() at startup and from app_handle_key() on t/T.
 * Background is -1 (terminal default) so demos respect the user's theme.
 * On 256-colour terminals: uses the xterm-256 indices directly.
 * On 8-colour fallback: maps to the closest ANSI colour by visual feel
 * (red→yellow→green→cyan→blue→magenta arc approximates ROYGBIV order).
 *
 * PAIR_HUD / PAIR_HINT are NOT touched here — they are theme-independent
 * per the project HUD spec, and are configured once in color_init().
 */
static void theme_apply(int idx)
{
    const Theme *th = &THEMES[idx];
    if (COLORS >= 256) {
        for (int p = 1; p <= N_PAIRS; p++)
            init_pair(p, th->body[p - 1], -1);
    } else {
        /* 8-colour ANSI fallback — approximate ROYGBIV order */
        init_pair(1, COLOR_RED,     -1);
        init_pair(2, COLOR_YELLOW,  -1);
        init_pair(3, COLOR_YELLOW,  -1);
        init_pair(4, COLOR_GREEN,   -1);
        init_pair(5, COLOR_CYAN,    -1);
        init_pair(6, COLOR_BLUE,    -1);
        init_pair(7, COLOR_MAGENTA, -1);
    }
}

/*
 * color_init() — one-time colour system setup.
 *
 *   start_color()        — initialise ncurses colour support.
 *   use_default_colors() — allow background = -1 to mean "terminal default".
 *   theme_apply()        — register the initial rope palette.
 *   PAIR_HUD / PAIR_HINT — registered once with bright yellow / cyan
 *                          (256-colour) or YELLOW / CYAN (8-colour fallback)
 *                          per the project HUD spec.
 */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();
    theme_apply(initial_theme);

    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell; the one aspect-ratio fix                     */
/* ===================================================================== */

/*
 * WHY TWO COORDINATE SPACES
 * ─────────────────────────
 * Terminal cells are not square.  A typical cell is ~2× taller than wide
 * in physical pixels (CELL_H = 16 vs CELL_W = 8).  If rope particle
 * positions were stored directly in (col, row) cell coordinates, a rope
 * segment pointing diagonally at 45° in screen space would actually travel
 * twice as far vertically as horizontally in physical space — the rope
 * would look stretched vertically and gravity would appear anisotropic.
 *
 * THE FIX — physics in pixel space, drawing in cell space:
 *   All Vec2 positions (pos[][], old_pos[][], prev_pos[][], anchor[])
 *   are in pixel space where 1 unit = 1 physical pixel.  The pixel grid
 *   is square and isotropic: gravity acts equally in all directions and
 *   segment lengths are Euclidean.  Only at draw time does §5g convert to
 *   cell coordinates via px_to_cell_x / px_to_cell_y.
 *
 * px_to_cell_x / px_to_cell_y — convert pixel coordinate to cell index.
 *
 * Formula:  cell = floor(px / CELL_DIM + 0.5)
 *
 * Adding 0.5 before flooring implements "round half up" — deterministic
 * and symmetric.  This avoids two artefacts:
 *   • roundf() uses "round half to even" (banker's rounding), which can
 *     oscillate when a particle sits exactly on a cell boundary — single-
 *     pixel flicker.
 *   • Truncation ((int)(px / CELL_W)) always rounds toward zero, giving
 *     asymmetric dwell at boundaries and a subtle drift toward the origin.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  entity — Scene: Verlet ropes + constraint solver + renderer        */
/* ===================================================================== */

/*
 * Vec2 — a 2-D point in PIXEL space (sub-cell precision).
 *
 * Intent
 *   Every §5 physics quantity — particle positions, implicit
 *   velocities, wind impulses, displacement vectors — lives in
 *   pixel space. Each character cell is CELL_W × CELL_H sub-pixels
 *   (8 × 16), giving sub-cell precision so a swaying rope moves
 *   smoothly instead of jumping cell-to-cell. The conversion to
 *   cell space happens only inside §5g rendering helpers via
 *   px_to_cell_x/y — the project's "one conversion point" rule.
 *
 * Convention
 *   x : EASTWARD pixel coordinate (positive → right of screen).
 *   y : SOUTHWARD pixel coordinate (positive → DOWN the screen).
 *
 *   The y-down convention matters for the physics:
 *     - GRAVITY is POSITIVE (pulls toward larger y).
 *     - The floor is at LARGER y than the ceiling anchor.
 *     - "Below the anchor" means "larger y" — the rope HANGS DOWN.
 *
 * Why a value type
 *   8 bytes; passes in registers. implicit_velocity, apply_damping,
 *   integrate_acceleration, displacement_and_length all take/return
 *   Vec2 by value — -O2 inlines them into straight-line code.
 *
 * Why named (x, y) instead of two loose floats
 *   Type-checking catches (col, row) confusion at compile time.
 *
 * References [3] Jakobsen §2 for the Verlet position representation.
 */
typedef struct {
    float x;   /* eastward  pixel coordinate (positive → right) */
    float y;   /* southward pixel coordinate (positive → down)  */
} Vec2;

/*
 * Scene — full simulation state for the rope bank.
 *
 * Intent
 *   N_ROPES rope chains share one Scene record. Each rope is N_SEG
 *   Verlet particles plus a fixed anchor and a per-rope wind phase
 *   stagger. The struct cleanly separates FIVE subsystems:
 *
 *     1. VERLET STATE — (pos, old_pos) — the integrator's two-position
 *        memory. Velocity is IMPLICIT: v = pos − old_pos. Modifying
 *        old_pos (without touching pos) injects velocity — this is
 *        how the bounce + anchor-pin idioms work. Refs [1] Verlet.
 *
 *     2. RENDER SNAPSHOT — prev_pos — copied from pos at the START of
 *        each tick. render_scene lerps prev_pos → pos by alpha so
 *        motion stays smooth regardless of sim/render Hz mismatch.
 *        Ref [9] Fiedler.
 *
 *     3. PER-ROPE GEOMETRY — (anchor, rope_len_px, rest_len,
 *        phase_offset) — set ONCE in scene_init from screen size.
 *        Recomputed on SIGWINCH so all ropes scale with the terminal.
 *
 *     4. WIND OSCILLATOR — (wind_time, wind_amp, wind_freq) — the
 *        simple-harmonic-motion clock driving sinusoidal_wind_
 *        acceleration_at(). Each rope reads (wind_time, wind_freq,
 *        phase_offset[r]) to compute its instantaneous lateral
 *        acceleration. Ref [8] Goldstein §2 (SHM).
 *
 *     5. UI / CONTROL — paused (gates physics) + theme_idx (chooses
 *        the §3 palette).
 *
 * Why pos, old_pos, AND prev_pos — three position arrays
 *   - pos     : CURRENT physics position; mutates every tick.
 *   - old_pos : PREVIOUS physics position; defines implicit velocity.
 *   - prev_pos: position at the START of the current tick; anchor for
 *               sub-tick render interpolation.
 *   old_pos and prev_pos cannot be collapsed: old_pos is rewritten by
 *   every constraint pass + boundary bounce throughout the tick;
 *   prev_pos is frozen at tick start. Merging them would corrupt
 *   either the integrator or the renderer.
 *
 * Why per-rope arrays of geometry (not a Rope struct)
 *   N_ROPES is small (7) and N_SEG is small (20), so a struct-of-
 *   arrays (SoA) layout fits in a couple of cache lines. The hot
 *   loops walk all 7 ropes one field at a time (the constraint
 *   solver reads pos[r][·], the wind reads phase_offset[r]); SoA
 *   matches that access pattern. AoS would force gather-style
 *   indexing across more fields per cache line.
 *
 * Why phase_offset is CACHED in the struct (not computed each tick)
 *   It's a constant after scene_init — the formula r·2π/N_ROPES
 *   never changes — so we pay the multiply ONCE and read it
 *   N_ROPES × every-tick thereafter. Cheap memory for cheap CPU.
 *
 * Why theme_idx is "preserved across r/R reset"
 *   r/R is meant to reset the PHYSICS (re-spawn ropes at rest), not
 *   the user's chosen palette. scene_init saves theme_idx before
 *   memset and restores it after — so the visual identity stays
 *   stable even when the simulation is rebuilt.
 *
 * Simulation vs Rendering locality
 *   ── Simulation state ── read+written by scene_tick:
 *      pos, old_pos              Verlet two-position memory
 *      rest_len, rope_len_px,    per-rope geometry (immutable
 *      anchor, phase_offset      after scene_init)
 *      wind_time, wind_amp,      SHM oscillator (wind_time advances
 *      wind_freq                 each tick; amp/freq are runtime-
 *                                 tunable via arrow keys)
 *      paused                    gates scene_tick early-return
 *
 *   ── Render-only state ──
 *      prev_pos                  sub-tick lerp anchor for render
 *      theme_idx                 chooses §3 palette via theme_apply
 *
 *   (No mid-tier "control state" group needed — paused IS sim state
 *    because it gates the integrator; theme_idx is purely render.)
 *
 * Things that DO NOT live here
 *   - sim_fps → §8 App (frame-cadence concern, not part of the
 *     simulated world).
 *   - fps counter → main() local (display-only).
 *   - terminal extents (cols, rows) → §7 Screen.
 *   - signal flags (running, need_resize) → §8 App.
 *
 * References [1][3][4] for the Verlet + PBD framework; [8] Goldstein
 *   for the SHM wind oscillator; [9] Fiedler for the prev_pos lerp.
 */
typedef struct {
    /* ── Verlet state (per-rope, per-segment, SoA) ────────────── *
     * Two-position memory. v = pos − old_pos is the implicit     *
     * velocity. Constraints + bounces write to old_pos to inject *
     * velocity changes.                                          */
    Vec2 pos     [N_ROPES][N_SEG];
    Vec2 old_pos [N_ROPES][N_SEG];

    /* ── Render snapshot (sub-tick interpolation anchor) ──────── *
     * Frozen at tick start; render_scene lerps prev_pos → pos by *
     * alpha ∈ [0, 1).                                            */
    Vec2 prev_pos[N_ROPES][N_SEG];

    /* ── Per-rope geometry (set ONCE in scene_init) ───────────── *
     * anchor       : ceiling pixel position; pos[r][0] is pinned *
     *                here by pin_rope_anchor every tick.         *
     * rope_len_px  : total visual length (35%-75% screen height).*
     * rest_len     : per-segment rest length (= len_px/(N_SEG−1)).*
     * phase_offset : SHM phase shift for rope r (= r·2π/N_ROPES);*
     *                pre-computed so the wind sin() argument is  *
     *                a simple addition each tick.                */
    Vec2  anchor      [N_ROPES];
    float rope_len_px [N_ROPES];
    float rest_len    [N_ROPES];
    float phase_offset[N_ROPES];

    /* ── Wind oscillator (simple-harmonic-motion driver) ─────── *
     * Closed-form sin(ω·t + φ_r) on three knobs:                *
     *   wind_time  : accumulated sim seconds (advances each tick *
     *                unless paused).                              *
     *   wind_amp   : peak lateral acceleration (px/s²), in [0,1000].*
     *   wind_freq  : angular frequency (rad/s), in [0.05, 4.0].   */
    float wind_time;
    float wind_amp;
    float wind_freq;

    /* ── UI / control state ───────────────────────────────────── *
     * paused gates scene_tick (renderer still runs);             *
     * theme_idx is render-only (selects §3 palette).             */
    bool  paused;
    int   theme_idx;
} Scene;

/* ── §5a  rope_verlet_step ──────────────────────────────────────────── */

/*
 * implicit_velocity — Verlet's defining trick.
 *
 *   In Verlet integration, velocity is NEVER stored explicitly. It's
 *   reconstructed from two consecutive positions:
 *
 *       v(t) ≈ pos(t) − pos(t − dt)
 *
 *   This means the state vector (pos, old_pos) carries BOTH position
 *   AND momentum. Modifying old_pos AUTOMATICALLY changes the implicit
 *   velocity — which is why every constraint and collision response
 *   in this file works by writing to old_pos.
 *
 *   Reference: Verlet (1967) "Computer Experiments on Classical
 *   Fluids", Phys. Rev. 159 — original integrator. Jakobsen (2001)
 *   "Advanced Character Physics" GDC — cloth/rope application.
 */
static inline Vec2 implicit_velocity(Vec2 pos, Vec2 old_pos)
{
    return (Vec2){ pos.x - old_pos.x, pos.y - old_pos.y };
}

/*
 * apply_damping — multiplicative velocity damping per tick.
 *
 *   v' = v · DAMPING
 *
 *   ROPE_DAMPING = 0.992 removes ~0.8% kinetic energy per tick —
 *   a thin, whippy rope loses energy faster than a rigid limb
 *   (which uses 0.995 in ragdoll_figure.c). Multiplicative damping
 *   in Verlet form is equivalent to a viscous −β·v term in the
 *   continuous equations of motion.
 */
static inline Vec2 apply_damping(Vec2 v, float damping)
{
    return (Vec2){ v.x * damping, v.y * damping };
}

/*
 * integrate_acceleration — Verlet position step.
 *
 *   pos(t + dt) = pos(t) + v(t)·dt + a·dt²
 *
 *   In our reduced form (with v already · dt from the implicit-
 *   velocity reconstruction) the equation collapses to:
 *
 *       pos' = pos + v + a · dt²
 *
 *   The factor-of-½ that appears in the textbook Verlet derivation
 *   is absorbed into the tuned constants GRAVITY and wind_amp.
 */
static inline Vec2 integrate_acceleration(Vec2 pos, Vec2 v, Vec2 a, float dt2)
{
    return (Vec2){
        pos.x + v.x + a.x * dt2,
        pos.y + v.y + a.y * dt2,
    };
}

/*
 * rope_verlet_step — one Störmer-Verlet integration step on particle (r, s).
 *
 *   1. v       = implicit_velocity(pos, old_pos)      (Verlet trick)
 *   2. v      ← apply_damping(v, ROPE_DAMPING)        (air drag)
 *   3. a       = (wind_x, GRAVITY)                    (external accel)
 *   4. old_pos ← pos                                  (slide state vector)
 *   5. pos    ← integrate_acceleration(pos, v, a, dt²)
 *
 *   Step 4 MUST run before step 5 — otherwise the new pos would
 *   overwrite the cur it depends on. The temp captured at step 1
 *   prevents the aliasing.
 *
 *   GRAVITY is positive because +y is downward in pixel space.
 *
 *   Particle 0 is the ceiling ANCHOR and is never passed here —
 *   enforce_anchors() re-pins it after every Verlet pass.
 *
 *   Reference: Jakobsen (2001) §2 "Verlet Integration".
 */
static void rope_verlet_step(Scene *sc, int r, int s, float wind_x, float dt)
{
    float dt2 = dt * dt;

    Vec2 v = implicit_velocity(sc->pos[r][s], sc->old_pos[r][s]);
    v      = apply_damping(v, ROPE_DAMPING);
    Vec2 a = (Vec2){ wind_x, GRAVITY };

    Vec2 cur = sc->pos[r][s];
    sc->old_pos[r][s] = cur;
    sc->pos[r][s]     = integrate_acceleration(cur, v, a, dt2);
}

/* ── §5b  apply_rope_constraints ────────────────────────────────────── */

/*
 * apply_rope_constraints() — run N_ITERS distance constraint passes for
 * all ropes, re-enforcing anchors after each pass.
 *
 * WHAT: For each adjacent particle pair (s, s+1) in each rope, push the
 *   particles apart or together so their distance equals rest_len[r].
 *
 * WHY ITERATIVE: One pass is not enough.  Correcting pair (0,1) disturbs
 *   particle 1, which then violates the (1,2) constraint, and so on.  Each
 *   additional pass propagates corrections further along the chain.  For a
 *   20-node rope, 6 passes converge the error to < 0.1% of rest_len — the
 *   segment looks inextensible to any human observer.
 *
 * CONSTRAINT EQUATION for one segment:
 *
 *   dx   = pos[r][s+1] − pos[r][s]       ← displacement vector
 *   dist = |dx|                           ← current length
 *   err  = (dist − rest_len[r]) / dist   ← fractional length error
 *
 *   correction = 0.5 × err × dx          ← split equally (equal mass)
 *   pos[r][s  ] += correction             ← push s away from s+1
 *   pos[r][s+1] -= correction             ← push s+1 away from s
 *
 *   Why multiply by 0.5?  Both particles have equal mass.  The total
 *   correction needed is err × dx; dividing it equally between two
 *   particles (0.5 each) conserves the centre of mass of the pair.
 *
 *   Guard: if dist < 1e-6 (particles coincident), skip to avoid divide-
 *   by-zero.  This can only happen in degenerate edge cases.
 *
 * ANCHOR RE-ENFORCEMENT AFTER EACH PASS:
 *
 *   The constraint solver above uses equal-mass corrections, which means
 *   it can displace particle[0] (the anchor) slightly when fixing the
 *   (0,1) segment.  Without re-pinning, these tiny displacements accumulate
 *   across N_ITERS passes and the anchor visibly drifts over time.
 *
 *   After every pass, enforce_anchors() resets pos[r][0] = old_pos[r][0]
 *   = anchor[r] for all ropes.  This zeroes the constraint-induced drift
 *   immediately after each pass, before the next pass can compound it.
 *
 * PARAMETERS:
 *   sc, cols, rows — scene and terminal dimensions (cols/rows passed to
 *                    enforce_anchors() for completeness but not used there).
 */
/*
 * displacement_and_length — Δ = b − a and its Euclidean length, in
 * one pass.
 *
 *   Returns:
 *     *out_delta  ← b − a            (displacement vector)
 *     return val  ← |b − a|         (Euclidean length)
 *
 *   The inner loop of the relaxation calls this 19 segments × 7 ropes
 *   × N_ITERS times per tick — a fused compute saves one redundant
 *   field-write per call.
 */
static inline float displacement_and_length(Vec2 a, Vec2 b, Vec2 *out_delta)
{
    out_delta->x = b.x - a.x;
    out_delta->y = b.y - a.y;
    return sqrtf(out_delta->x * out_delta->x +
                 out_delta->y * out_delta->y);
}

/*
 * relax_one_distance_constraint — Jakobsen equal-mass projection
 * step for ONE adjacent particle pair.
 *
 *   Given particles p_a, p_b and rest length L:
 *
 *       Δ           = p_b − p_a             (current displacement)
 *       d           = |Δ|                   (current length)
 *       fractional  = (d − L) / d           (signed stretch / length)
 *
 *   POSITIVE fractional → stretched, pull endpoints together.
 *   NEGATIVE fractional → compressed, push endpoints apart.
 *
 *   EQUAL-MASS CORRECTION (Jakobsen 2001 §3):
 *
 *       p_a' = p_a + ½ · fractional · Δ     (each endpoint moves
 *       p_b' = p_b − ½ · fractional · Δ      half the error)
 *
 *   The ½ split conserves the CENTRE OF MASS of the pair — both
 *   particles have equal mass, so the geometric mid-point doesn't
 *   move when the constraint corrects the length.
 *
 *   Degenerate guard: if d < 1e-6 the particles coincide and the
 *   fractional stretch is undefined; skip rather than divide by zero.
 */
static inline void relax_one_distance_constraint(Vec2 *a, Vec2 *b,
                                                 float rest_length)
{
    Vec2  delta;
    float length = displacement_and_length(*a, *b, &delta);
    if (length < 1e-6f) return;        /* degenerate: coincident */

    float fractional = (length - rest_length) / length;
    float cx         = 0.5f * fractional * delta.x;
    float cy         = 0.5f * fractional * delta.y;

    a->x += cx;  a->y += cy;
    b->x -= cx;  b->y -= cy;
}

/*
 * relax_rope_chain — one full Gauss-Seidel sweep along rope r.
 *
 *   Walks the chain from anchor end (s = 0) to free tip (s = N_SEG − 1)
 *   applying relax_one_distance_constraint to each adjacent pair
 *   (s, s+1). After this sweep, each individual segment is at rest
 *   length — but BECAUSE each pass disturbs adjacent shared
 *   endpoints, the sweep typically VIOLATES the next-door segments
 *   it just touched. apply_rope_constraints iterates this sweep
 *   N_ITERS times for geometric convergence.
 *
 *   "Gauss-Seidel" because each correction immediately propagates
 *   to the next constraint via the shared endpoint (vs Jacobi,
 *   which would buffer all corrections and apply them together).
 */
static void relax_rope_chain(Scene *sc, int r)
{
    for (int s = 0; s < N_SEG - 1; s++)
        relax_one_distance_constraint(&sc->pos[r][s],
                                      &sc->pos[r][s + 1],
                                      sc->rest_len[r]);
}

/*
 * pin_rope_anchor — zero the implicit velocity of particle (r, 0)
 * by writing BOTH pos AND old_pos to the ceiling anchor.
 *
 *   In Verlet, v = pos − old_pos. Writing BOTH to the same value
 *   forces v = 0 — the anchor is truly immovable. Resetting only
 *   pos would leave old_pos at the previously-displaced location,
 *   so the next Verlet step would yank the anchor with a phantom
 *   velocity. This double-write is the Verlet idiom for "pin".
 */
static inline void pin_rope_anchor(Scene *sc, int r)
{
    sc->pos    [r][0] = sc->anchor[r];
    sc->old_pos[r][0] = sc->anchor[r];
}

/*
 * apply_rope_constraints — Position-Based Dynamics relaxation.
 *
 *   For iter = 0 .. N_ITERS:
 *     for each rope r:
 *       1. relax_rope_chain(r)    (Gauss-Seidel sweep along rope)
 *       2. pin_rope_anchor(r)     (zero the equal-mass drift the
 *                                  sweep induces on particle 0)
 *
 *   N_ITERS controls stiffness: 1 = stretchy rope, 8 = essentially
 *   inextensible. We use 6 — gives < 0.1% residual stretch on a
 *   20-node rope while staying well within frame budget.
 *
 *   The PIN-AFTER-EACH-PASS pattern is what makes "anchor" actually
 *   anchored. Without it, the equal-mass split in relax_one_distance_
 *   constraint would gradually walk the anchor away from the ceiling
 *   over many iterations.
 *
 *   References: Jakobsen (2001) §3 — distance-constraint relaxation;
 *     Müller et al. (2007) "Position Based Dynamics" §3 — modern
 *     generalisation to arbitrary constraint types.
 */
static void apply_rope_constraints(Scene *sc)
{
    for (int iter = 0; iter < N_ITERS; iter++) {
        for (int r = 0; r < N_ROPES; r++) {
            relax_rope_chain(sc, r);
            pin_rope_anchor (sc, r);
        }
    }
}

/* ── §5c  enforce_anchors ───────────────────────────────────────────── */

/*
 * enforce_anchors() — pin every rope's particle[0] to its ceiling position.
 *
 * WHAT: For each rope r, force:
 *   pos[r][0]     = anchor[r]
 *   old_pos[r][0] = anchor[r]
 *
 * WHY RESET BOTH pos AND old_pos:
 *
 *   After a Verlet step, pos[r][0] has been displaced by gravity and wind
 *   (even though it should be fixed).  Resetting only pos[r][0] = anchor[r]
 *   would leave old_pos[r][0] at the previous (Verlet-displaced) location.
 *
 *   On the NEXT tick, rope_verlet_step() computes:
 *     vel = (pos − old_pos) × DAMPING
 *         = (anchor − displaced_old_pos) × DAMPING
 *         ≠ zero
 *
 *   This phantom velocity would yank particle[0] off the anchor immediately,
 *   undoing the pin.  Over many ticks the anchor drifts progressively further
 *   from its ceiling position.
 *
 *   Resetting BOTH zeroes the implicit velocity:
 *     vel = (anchor − anchor) × DAMPING = 0
 *   The particle is truly immovable.
 *
 * WHEN CALLED:
 *   1. After rope_verlet_step() for all particles in a rope (inside
 *      rope_tick() step 4c).
 *   2. After every constraint pass inside apply_rope_constraints() to
 *      prevent constraint drift from accumulating (step 5 of rope_tick()).
 *
 * The double enforcement (after Verlet + after each constraint) completely
 * eliminates anchor drift at any wind_amp or constraint iteration count.
 */
static void enforce_anchors(Scene *sc)
{
    for (int r = 0; r < N_ROPES; r++)
        pin_rope_anchor(sc, r);
}

/* ── §5d  apply_wind ────────────────────────────────────────────────── */

/*
 * apply_wind() — compute lateral wind acceleration for rope r at the
 * current wind_time, and integrate it into all non-anchor particles.
 *
 * WHAT: Applies a horizontal sinusoidal force to every free particle
 *   (s = 1 … N_SEG-1) of rope r.  The force amplitude varies smoothly
 *   over time — positive (rightward) for half the period, negative
 *   (leftward) for the other half.
 *
 * WIND FORMULA:
 *   accel_x = wind_amp × sin(wind_time × wind_freq + phase_offset[r])
 *
 *   wind_time  — accumulated real simulation seconds.
 *   wind_freq  — angular frequency (rad/s).  Controls how fast the
 *                oscillation cycles: period = 2π / wind_freq.
 *   phase_offset[r] = r × 2π / N_ROPES.
 *
 * WHY PHASE OFFSETS:
 *   If all ropes shared the same phase (offset = 0), they would all sway
 *   left at the same time, then all sway right — boring and unrealistic.
 *
 *   Distributing 7 offsets evenly over 2π (spacing = 2π/7 ≈ 51.4°) means
 *   the ropes are always at distinct points in their cycle.  Neighbouring
 *   ropes are 51.4° apart; they never sway in the same direction at the
 *   same time.  The resulting "Mexican wave" is visually rich and complex
 *   despite being computed from a single sin() call per rope per tick.
 *
 * WHY SINUSOIDAL (not random impulse):
 *   Random impulses (like ragdoll_figure.c's gravity noise) create a
 *   jittery, unpredictable sway.  Sinusoidal force is:
 *   • Smooth: no sudden jumps → no constraint explosions even at high
 *     wind_amp.
 *   • Deterministic: the same amplitude/frequency always produces the
 *     same visual rhythm.
 *   • Controllable: wind_amp and wind_freq map directly to physical
 *     intuition (force magnitude, oscillation tempo).
 *
 * PARAMETERS:
 *   sc         — scene (reads wind_time, wind_amp, wind_freq, phase_offset[r]).
 *   r          — rope index: computes wind for this rope's particles only.
 *   dt         — tick duration in seconds (passed through to rope_verlet_step).
 *   cols/rows  — terminal dimensions (passed to rope_boundaries).
 *
 * SIDE EFFECTS: calls rope_verlet_step() for all non-anchor particles,
 *   then rope_boundaries() to clamp them to the screen box.
 */
/*
 * sinusoidal_wind_acceleration_at — closed-form lateral wind force.
 *
 *     a_x(t, r) = wind_amp · sin(wind_time · wind_freq + phase_offset[r])
 *
 *   This is a STANDING WAVE in time: the same closed-form sin() that
 *   describes a simple harmonic oscillator. wind_amp is the
 *   amplitude (peak lateral acceleration, px/s²), wind_freq is the
 *   angular frequency (rad/s; period = 2π / wind_freq), and
 *   phase_offset[r] shifts each rope along the sin cycle.
 *
 *   PHASE OFFSETS evenly distributed over [0, 2π) — set in
 *   scene_init — give the bank of ropes a "Mexican-wave" appearance:
 *   no two ropes are at the same phase, so the eye reads a single
 *   coherent flow instead of seven identical pendulums.
 *
 *   Why sinusoidal (not random impulse)?
 *     - SMOOTH         : no jumps → constraint solver never explodes.
 *     - DETERMINISTIC  : same params → same look.
 *     - CONTROLLABLE   : amp + freq map directly to feel.
 *
 *   Reference: any classical-mechanics text on simple harmonic
 *   motion; e.g. Goldstein §2 "Lagrange's equations".
 */
static inline float sinusoidal_wind_acceleration_at(const Scene *sc, int r)
{
    return sc->wind_amp
         * sinf(sc->wind_time * sc->wind_freq + sc->phase_offset[r]);
}

/*
 * integrate_rope_segments — Verlet step on every non-anchor particle
 * of rope r. Particle 0 (the anchor) is skipped; enforce_anchors
 * re-pins it after this pass.
 */
static inline void integrate_rope_segments(Scene *sc, int r,
                                           float wind_acc, float dt)
{
    for (int s = 1; s < N_SEG; s++)
        rope_verlet_step(sc, r, s, wind_acc, dt);
}

/*
 * bounce_against_wall_1d — one-axis Verlet bounce with restitution.
 *
 *   Given a particle that crossed axis-aligned boundary `wall`:
 *
 *       1. v_axis = pos − old_pos          (implicit velocity, 1-D)
 *       2. pos    ← wall                    (clamp onto surface)
 *       3. old_pos ← wall + v_axis · e     (encode reflected vel)
 *
 *   Step 3 places old_pos PAST the wall so the next tick's implicit
 *   velocity points AWAY from it — the classic Verlet bounce.
 *
 *   BOUNCE_COEFF ∈ (0, 1] is the COEFFICIENT OF RESTITUTION:
 *     e = 1    perfectly elastic (no energy loss)
 *     e → 0    perfectly inelastic (sticks to wall)
 *
 *   Reference: Goldstein "Classical Mechanics" §3.6 on 1-D collisions.
 */
static inline void bounce_against_wall_1d(float *pos, float *old_pos,
                                          float wall)
{
    float v_axis = *pos - *old_pos;
    *pos     = wall;
    *old_pos = wall + v_axis * BOUNCE_COEFF;
}

/*
 * bounce_rope_against_screen_bounds — clip every non-anchor particle
 * of rope r to the screen rectangle (floor + side walls; the ceiling
 * is held by the anchor so no top-wall test is needed).
 */
static inline void bounce_rope_against_screen_bounds(Scene *sc, int r,
                                                     int cols, int rows)
{
    float floor_y = (float)(rows * CELL_H) - FLOOR_MARGIN;
    float left_x  = LEFT_MARGIN;
    float right_x = (float)(cols * CELL_W) - RIGHT_MARGIN;

    for (int s = 1; s < N_SEG; s++) {
        if (sc->pos[r][s].y > floor_y)
            bounce_against_wall_1d(&sc->pos[r][s].y, &sc->old_pos[r][s].y, floor_y);
        if (sc->pos[r][s].x < left_x)
            bounce_against_wall_1d(&sc->pos[r][s].x, &sc->old_pos[r][s].x, left_x);
        if (sc->pos[r][s].x > right_x)
            bounce_against_wall_1d(&sc->pos[r][s].x, &sc->old_pos[r][s].x, right_x);
    }
}

/*
 * apply_wind — drive one rope forward by one physics tick.
 *
 *   1. sinusoidal_wind_acceleration_at(sc, r)   (closed-form SHM)
 *   2. integrate_rope_segments(sc, r, wind, dt) (Verlet on segments)
 *   3. bounce_rope_against_screen_bounds       (floor + side walls)
 *
 *   The wind is computed ONCE per rope per tick (uniform across the
 *   whole chain) — physically equivalent to a perfectly horizontal
 *   gust that affects every particle equally.
 *
 *   References: Jakobsen 2001 §2 (Verlet) + §4 (collisions);
 *     Goldstein §2 (SHM) + §3.6 (restitution).
 */
static void apply_wind(Scene *sc, int r, float dt, int cols, int rows)
{
    float wind_acc = sinusoidal_wind_acceleration_at(sc, r);
    integrate_rope_segments(sc, r, wind_acc, dt);
    bounce_rope_against_screen_bounds(sc, r, cols, rows);
}

/* ── §5e  rope_node_char ────────────────────────────────────────────── */

/*
 * rope_node_char() — choose the bead glyph for particle s based on its
 * position along the rope (anchor end → free tip).
 *
 * WHAT: Returns one of three ASCII characters that stamp at each particle
 *   position in pass 2 of draw_rope_beads():
 *   • '0' (zero)  — top quarter of rope  (s < N_SEG/4 = 5)
 *   • '.'         — bottom quarter        (s ≥ N_SEG×3/4 = 15)
 *   • 'o'         — middle half           (5 ≤ s < 15)
 *
 * PHYSICAL INTUITION — the size gradient mirrors physical tension:
 *   A hanging rope is under the most tension near the ceiling (it must
 *   support all the weight below it) and the least tension at the free
 *   tip (supports nothing).  Higher tension → brighter, thicker glyph.
 *   '0' (widest glyph) at the top suggests a taut, load-bearing segment.
 *   '.' (thinnest) at the bottom suggests a slack, dangling tip.
 *
 * WHY THREE ZONES (not a continuous gradient):
 *   Terminal character sets have limited glyph variety.  Three characters
 *   with visually distinct "weight" give the clearest size gradient within
 *   the ASCII repertoire.  The brightness gradient is reinforced separately
 *   by rope_node_attr() (A_BOLD / A_NORMAL / A_DIM).
 *
 * NUMERIC EXAMPLES (N_SEG = 20):
 *   s = 0  → N_SEG/4 = 5  → s < 5  → '0'   A_BOLD   (anchor particle)
 *   s = 4  → s < 5        → '0'   A_BOLD   (4th node from ceiling)
 *   s = 5  → 5 ≤ s < 15   → 'o'   A_NORMAL
 *   s = 14 → s < 15        → 'o'   A_NORMAL
 *   s = 15 → s ≥ 15        → '.'   A_DIM    (first bottom-quarter node)
 *   s = 19 → s ≥ 15        → '.'   A_DIM    (free tip — weight marker)
 */
static chtype rope_node_char(int s)
{
    if (s < N_SEG / 4)       return (chtype)'0';
    if (s >= N_SEG * 3 / 4)  return (chtype)'.';
    return (chtype)'o';
}

/*
 * rope_node_attr() — ncurses attribute for particle s.
 *
 * Mirrors rope_node_char() brightness logic:
 *   A_BOLD   — top quarter  (high tension, bright)
 *   A_DIM    — bottom quarter (low tension, faint)
 *   A_NORMAL — middle half
 *
 * The brightness gradient gives an immediate visual reading of where the
 * rope's mechanical load is concentrated, without any colour change.
 */
static attr_t rope_node_attr(int s)
{
    if (s < N_SEG / 4)       return A_BOLD;
    if (s >= N_SEG * 3 / 4)  return A_DIM;
    return A_NORMAL;
}

/* ── §5e2  mark_cell — central glyph stamp helper ───────────────────── */

/*
 * mark_cell() — stamp one ASCII glyph at terminal cell (cx,cy).
 *
 * Centralises the (chtype)(unsigned char) cast plus bounds-check that
 * would otherwise be repeated at every mvwaddch site.  The double cast
 * prevents sign-extension on character values > 127 (per CLAUDE.md
 * "Common ncurses Bugs").  Glyph is silently dropped if off-screen.
 */
static void mark_cell(WINDOW *w, int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/* ── §5f  draw_rope_beads ───────────────────────────────────────────── */

/*
 * draw_rope_beads() — render one rope using the two-pass bead technique.
 *
 * WHAT: Fills the rope's visual appearance into WINDOW *w using two passes.
 *   Pass 1 fills every segment with 'o' beads at DRAW_STEP_PX intervals.
 *   Pass 2 overwrites each particle position with a size-graded node marker
 *   ('0', 'o', or '.') at the appropriate brightness attribute, then stamps
 *   an A_BOLD 'o' at the tip (particle N_SEG-1) as the weight marker.
 *
 * WHY TWO PASSES:
 *   Pass 1 fills the segment lines between particles, giving the rope visual
 *   continuity.  Pass 2 overrides particle positions with distinct glyph
 *   sizes, making the articulated bead structure visible.  Drawing node
 *   markers last ensures they always win over the fill beads on overlapping
 *   cells — the rope structure reads clearly even when two segments cross.
 *
 * PASS 1 — SEGMENT FILL:
 *   For each segment (s, s+1):
 *     dx, dy = displacement vector from rp[r][s] to rp[r][s+1].
 *     len    = Euclidean length of the segment.
 *     nsteps = ceil(len / DRAW_STEP_PX) + 1.
 *
 *     Walk t from 0 to nsteps:
 *       u  = t / nsteps                      ← fraction along segment
 *       cx = px_to_cell_x(rp[r][s].x + dx×u)
 *       cy = px_to_cell_y(rp[r][s].y + dy×u)
 *       If (cx, cy) == (prev_cx, prev_cy): skip (dedup guard).
 *       If out of [0,cols)×[0,rows): skip (bounds clipping).
 *       Stamp 'o' with COLOR_PAIR(cpair) | A_NORMAL.
 *
 *   The dedup guard (prev_cx/prev_cy tracking) prevents stamping the same
 *   terminal cell twice on the same segment, which would cause attribute
 *   flicker from the double wattron/wattroff sequence.
 *
 * PASS 2 — NODE MARKERS:
 *   For each particle s in [0, N_SEG):
 *     Convert rp[r][s] to (cx, cy); bounds-check; stamp rope_node_char(s)
 *     with rope_node_attr(s).
 *   Additionally stamp A_BOLD 'o' at the tip (s = N_SEG-1) as the
 *   "weight marker" — a bright blob at the rope's free end suggesting the
 *   mass that keeps the rope taut.
 *
 * PARAMETERS:
 *   sc    — scene (used to select the colour pair per rope).
 *   rp    — alpha-interpolated particle positions (computed by render_scene).
 *   r     — rope index to render.
 *   w     — ncurses WINDOW to draw into (typically stdscr).
 *   cols, rows — terminal dimensions for bounds checking.
 */
static void draw_rope_beads(const Scene *sc, const Vec2 rp[][N_SEG],
                            int r, WINDOW *w, int cols, int rows)
{
    (void)sc;                         /* colour pair derived from r, not sc  */
    int cpair = (r % N_PAIRS) + 1;    /* cycle rope colours through pairs 1–7 */

    /* Pass 1 — fill segments with 'o' beads at DRAW_STEP_PX intervals */
    for (int s = 0; s < N_SEG - 1; s++) {
        float dx  = rp[r][s+1].x - rp[r][s].x;
        float dy  = rp[r][s+1].y - rp[r][s].y;
        float len = sqrtf(dx * dx + dy * dy);
        if (len < 0.1f) continue;   /* degenerate segment */

        int nsteps  = (int)ceilf(len / DRAW_STEP_PX) + 1;
        int prev_cx = -9999, prev_cy = -9999;

        for (int t = 0; t <= nsteps; t++) {
            float u  = (float)t / (float)nsteps;
            int   cx = px_to_cell_x(rp[r][s].x + dx * u);
            int   cy = px_to_cell_y(rp[r][s].y + dy * u);

            if (cx == prev_cx && cy == prev_cy) continue;   /* dedup */
            prev_cx = cx;  prev_cy = cy;
            mark_cell(w, cx, cy, 'o', cpair, A_NORMAL, cols, rows);
        }
    }

    /* Pass 2 — node markers over-stamp fill beads at particle positions */
    for (int s = 0; s < N_SEG; s++) {
        int cx = px_to_cell_x(rp[r][s].x);
        int cy = px_to_cell_y(rp[r][s].y);
        char   ch   = (char)rope_node_char(s);
        attr_t attr = rope_node_attr(s);
        mark_cell(w, cx, cy, ch, cpair, attr, cols, rows);
    }

    /* Weight marker — bright 'o' over-stamps the free tip */
    {
        int cx = px_to_cell_x(rp[r][N_SEG - 1].x);
        int cy = px_to_cell_y(rp[r][N_SEG - 1].y);
        mark_cell(w, cx, cy, 'o', cpair, A_BOLD, cols, rows);
    }
}

/* ── §5g  render_scene ──────────────────────────────────────────────── */

/*
 * lerp_positions() — fill rp[][] with alpha-interpolated render positions.
 *
 *   rp[r][s] = prev_pos[r][s] + (pos[r][s] − prev_pos[r][s]) · alpha
 *
 * alpha ∈ [0,1) is the leftover fraction in the fixed-step accumulator.
 * Without this lerp, motion stutters whenever render Hz ≠ sim Hz; with
 * it, every rope glides smoothly at any combination of the two rates.
 */
static void lerp_positions(const Scene *sc, float alpha,
                           Vec2 rp[N_ROPES][N_SEG])
{
    for (int r = 0; r < N_ROPES; r++) {
        for (int s = 0; s < N_SEG; s++) {
            rp[r][s].x = sc->prev_pos[r][s].x
                       + (sc->pos[r][s].x - sc->prev_pos[r][s].x) * alpha;
            rp[r][s].y = sc->prev_pos[r][s].y
                       + (sc->pos[r][s].y - sc->prev_pos[r][s].y) * alpha;
        }
    }
}

/*
 * draw_ceiling() — render a structural '#' line at ANCHOR_ROW_CELLS.
 *
 * Uses the last rope's colour pair (N_PAIRS) under A_DIM so the ceiling
 * reads as scenery, not as a participating rope.  Rendered before the
 * ropes so any rope cell at the anchor row over-stamps it cleanly.
 */
static void draw_ceiling(WINDOW *w, int cols, int rows)
{
    if (ANCHOR_ROW_CELLS < 0 || ANCHOR_ROW_CELLS >= rows) return;
    for (int cx = 0; cx < cols; cx++) {
        mark_cell(w, cx, ANCHOR_ROW_CELLS, '#',
                  N_PAIRS, A_DIM, cols, rows);
    }
}

/*
 * render_scene() — orchestrate one frame in painter's order.
 *
 *   1. lerp_positions — sub-tick interpolation for all 7 × 20 particles.
 *   2. draw_ceiling   — '#' line at the anchor row (bottom layer).
 *   3. draw_rope_beads × N_ROPES — two-pass per-rope render.
 *
 * Order matters: each later layer over-stamps earlier ones at shared
 * cells, so rope nodes read above the ceiling.
 */
static void render_scene(const Scene *sc, WINDOW *w,
                         int cols, int rows, float alpha)
{
    Vec2 rp[N_ROPES][N_SEG];
    lerp_positions(sc, alpha, rp);

    draw_ceiling(w, cols, rows);
    for (int r = 0; r < N_ROPES; r++) {
        draw_rope_beads(sc, rp, r, w, cols, rows);
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * scene_init() — initialise all 7 ropes to a clean hanging state.
 *
 * Called at startup and on every r/R keypress reset.  Also called from
 * app_do_resize() after a SIGWINCH so rope lengths and anchor positions
 * are recomputed from the new terminal dimensions.
 *
 * THEME PRESERVATION:
 *   Save sc->theme_idx before memset; restore after.  This ensures the
 *   active colour theme survives a reset, matching user expectation that
 *   r/R resets the physics but not the visual style.
 *
 * ANCHOR DISTRIBUTION:
 *   N_ROPES = 7 anchors evenly spaced across the terminal width:
 *     anchor_x[r] = (r + 1) × screen_px_w / (N_ROPES + 1)
 *   Dividing by (N_ROPES + 1) rather than N_ROPES places equal margins
 *   at both sides, avoiding ropes pinned flush against the wall.
 *   e.g. 80-col terminal (640 px): anchors at 80, 160, 240, 320, 400,
 *   480, 560 px → 80 px (10 cols) spacing and margin on each side.
 *
 * ROPE LENGTH VARIATION (35%–75% of screen height):
 *   min_len = screen_px_h × 0.35   (35%: shortest rope — quick oscillation)
 *   max_len = screen_px_h × 0.75   (75%: longest  rope — slow oscillation)
 *   rope_len_px[r] = min_len + r × (max_len − min_len) / (N_ROPES − 1)
 *
 *   Physical intuition: a shorter rope has a higher natural frequency
 *   (like a shorter pendulum swings faster).  Mixing 7 different lengths
 *   means each rope's natural frequency is slightly different from its
 *   neighbours — the staggered sway looks more like a real wind-chime set
 *   than if all ropes were the same length.
 *
 *   rope_len_px is recomputed from 'rows' on every init, so the sim
 *   adapts cleanly to any terminal size including after resize.
 *
 * SEGMENT REST LENGTH:
 *   rest_len[r] = rope_len_px[r] / (N_SEG − 1)
 *   There are N_SEG particles but only (N_SEG − 1) segments between them.
 *   e.g. rope_len_px = 480 px, N_SEG = 20: rest_len = 480 / 19 ≈ 25.26 px.
 *
 * PHASE OFFSETS:
 *   phase_offset[r] = r × 2π / N_ROPES
 *   Pre-computed once here and used every tick in apply_wind().
 *   Spreading 7 offsets evenly over 2π (spacing ≈ 51.4°) guarantees no
 *   two ropes are at the same phase of the wind oscillation.
 *
 * INITIAL PARTICLE POSITIONS:
 *   All particles hang vertically straight down from the anchor:
 *     pos[r][s].x = anchor[r].x          (no lateral displacement)
 *     pos[r][s].y = anchor[r].y + s × rest_len[r]
 *   old_pos = pos: zero initial velocity (rope is at rest).
 *   prev_pos = pos: no-op alpha lerp on the first rendered frame.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    int saved_theme = sc->theme_idx;   /* preserve theme across reset */
    memset(sc, 0, sizeof *sc);
    sc->theme_idx = saved_theme;

    sc->wind_amp  = WIND_AMP_DEFAULT;
    sc->wind_freq = WIND_FREQ_DEFAULT;
    sc->wind_time = 0.0f;
    sc->paused    = false;

    float screen_px_h = (float)(rows * CELL_H);
    float screen_px_w = (float)(cols * CELL_W);
    float anchor_py   = (float)(ANCHOR_ROW_CELLS * CELL_H);

    /* Rope lengths span 35%–75% of screen height */
    float min_len = screen_px_h * 0.35f;
    float max_len = screen_px_h * 0.75f;

    for (int r = 0; r < N_ROPES; r++) {
        /* Evenly spaced ceiling anchor positions */
        sc->anchor[r].x = (float)(r + 1) * screen_px_w / (float)(N_ROPES + 1);
        sc->anchor[r].y = anchor_py;

        /* Rope length linearly interpolated from min to max */
        float len = min_len
                  + (float)r * (max_len - min_len) / (float)(N_ROPES - 1);
        sc->rope_len_px[r] = len;
        sc->rest_len[r]    = len / (float)(N_SEG - 1);

        /* Phase offset: distribute evenly over 2π */
        sc->phase_offset[r] = (float)r * 2.0f * (float)M_PI / (float)N_ROPES;

        /* Particles hang vertically; zero initial velocity */
        for (int s = 0; s < N_SEG; s++) {
            sc->pos[r][s].x     = sc->anchor[r].x;
            sc->pos[r][s].y     = sc->anchor[r].y + (float)s * sc->rest_len[r];
            sc->old_pos[r][s]   = sc->pos[r][s];
            sc->prev_pos[r][s]  = sc->pos[r][s];
        }
    }
}

/*
 * scene_tick() — one fixed-step physics update for all ropes.
 *
 * Called from §8's fixed-step accumulator at exactly sim_fps Hz.
 * dt is the fixed tick duration (= 1.0 / sim_fps) in seconds.
 *
 * ORDER IS CRITICAL:
 *
 *   1. SAVE prev_pos[] FIRST — before any physics runs.
 *      This is the interpolation anchor for render_scene().  It must hold
 *      the state from the END of the previous tick.  Saving after physics
 *      would produce a lerp that starts at the new state and interpolates
 *      beyond it — visual overshoot.
 *
 *   2. RETURN EARLY IF PAUSED — prev_pos is saved regardless so the alpha
 *      lerp in render_scene() produces a clean freeze (prev == curr).
 *
 *   3. ADVANCE wind_time — drives the sin() argument in apply_wind().
 *
 *   4. APPLY WIND + VERLET for each rope:
 *      a. apply_wind() integrates wind and gravity into all non-anchor
 *         particles via rope_verlet_step(), then bounces at boundaries.
 *      b. enforce_anchors() pins particle[0] immediately after Verlet,
 *         before constraints can propagate any anchor displacement.
 *
 *   5. APPLY DISTANCE CONSTRAINTS — apply_rope_constraints() runs N_ITERS
 *      passes, re-enforcing anchors after each pass to prevent drift.
 *
 * The Verlet→anchor→constraint→anchor sequence is the standard "position-
 * based dynamics" pipeline: integrate, then correct geometry.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    /* Step 1 — save snapshot for sub-tick alpha interpolation */
    memcpy(sc->prev_pos, sc->pos, sizeof sc->pos);

    /* Step 2 — skip physics while paused */
    if (sc->paused) return;

    /* Step 3 — advance wind clock */
    sc->wind_time += dt;

    /* Step 4 — Verlet integration + boundary for each rope */
    for (int r = 0; r < N_ROPES; r++) {
        apply_wind(sc, r, dt, cols, rows);   /* a. Verlet + bounce      */
    }
    enforce_anchors(sc);                     /* b. pin anchors post-Verlet */

    /* Step 5 — iterative distance constraints (re-pins anchors inside) */
    apply_rope_constraints(sc);
}

/*
 * scene_draw() — render all ropes; called once per render frame.
 *
 * alpha ∈ [0,1) is the sub-tick interpolation factor from §8.
 * dt_sec is unused for ropes (no entity needs it for rendering).
 */
static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)dt_sec;
    render_scene(sc, w, cols, rows, alpha);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the ncurses display layer.
 *
 * Holds the current terminal dimensions (cols, rows), read after each
 * resize.  See framework.c §7 for the full double-buffer architecture
 * rationale (erase → draw → wnoutrefresh → doupdate).
 */
/*
 * Screen — terminal-extent snapshot in CHARACTER CELLS.
 *
 * Intent
 *   Caches the current terminal size so every §6 entry point reads
 *   (cols, rows) as plain ints rather than re-querying ncurses each
 *   frame. Refreshed only when SIGWINCH sets App::need_resize, then
 *   propagated to scene_init via app_do_resize so per-rope geometry
 *   (anchors + lengths) rescales with the terminal.
 *
 * Why a separate struct (not just two ints in App)
 *   Resize logic (endwin + refresh + getmaxyx) touches NOTHING in App
 *   except this struct. Carving it out makes screen_resize pure and
 *   isolates the ncurses dependency from the simulation layer.
 *
 * Why cells, not pixels
 *   ncurses' coordinate system is cells. Pixel space (CELL_W × CELL_H
 *   sub-pixels per cell) lives only inside §5 — converted at the
 *   draw boundary, per the project's "one conversion point" rule.
 *
 * References [12] Raymond, NCURSES Programming HOWTO.
 */
typedef struct {
    int cols;   /* terminal width  in CHARACTER CELLS */
    int rows;   /* terminal height in CHARACTER CELLS */
} Screen;

/*
 * screen_init() — configure the terminal for animation.
 *
 *   initscr()          — initialise ncurses; must be the first ncurses call.
 *   noecho()           — do not echo typed characters to the screen.
 *   cbreak()           — pass keys immediately, without line buffering.
 *   curs_set(0)        — hide the hardware cursor (no blinking cursor).
 *   nodelay(TRUE)      — getch() returns ERR immediately if no key is
 *                        available; makes input non-blocking so the render
 *                        loop never stalls waiting for user input.
 *   keypad(TRUE)       — decode arrow keys and function keys into single
 *                        KEY_* constants rather than multi-byte escape seqs.
 *   typeahead(-1)      — disable ncurses' habit of calling read() mid-output
 *                        to peek for escape sequences; without this, output
 *                        can be interrupted mid-frame, producing torn frames.
 *   color_init(0)      — register the initial colour palette (theme 0).
 *   getmaxyx()         — read actual terminal dimensions into s->cols/rows.
 */
static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(0);
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_free() — restore the terminal to its pre-animation state.
 * endwin() re-enables echo, shows the cursor, and restores the scroll region.
 */
static void screen_free(Screen *s) { (void)s; endwin(); }

/*
 * screen_resize() — handle a SIGWINCH (terminal resize) event.
 *
 * endwin() + refresh() forces ncurses to re-read LINES and COLS from the
 * kernel and resize its internal virtual screens (curscr/newscr) to match
 * the new terminal dimensions.  Without this, stdscr retains the old size
 * and mvwaddch calls at newly valid coordinates silently fail.
 *
 * After screen_resize(), app_do_resize() calls scene_init() to recompute
 * all rope lengths and anchor positions for the new size.
 */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw() — compose the full frame into stdscr (ncurses' newscr).
 *
 * Frame composition order (must not change):
 *
 *   1. erase()      — write spaces over the entire newscr, erasing stale
 *                     content from the previous frame.  Does NOT write to
 *                     the terminal; only modifies the in-memory newscr.
 *
 *   2. scene_draw() — ceiling line + all 7 ropes.
 *
 *   3. HUD top      — status bar string, right-aligned at row 0, drawn
 *                     AFTER scene_draw() so it always wins over rope pixels
 *                     that land on the top row.
 *                     Content: rope count × segment count, wind amplitude,
 *                     wind frequency, active theme name, fps, sim Hz, state.
 *
 *   4. HUD bottom   — key reference line at the last terminal row, also
 *                     drawn after scene_draw() for the same reason.
 *
 * Nothing reaches the physical terminal until screen_present() calls
 * doupdate() — all writes in this function are to the in-memory newscr.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    /* Top-right status — PAIR_HUD bright yellow, A_BOLD */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%3d Hz  wind:%.0f  freq:%.2f  [%s]  %s ",
             fps, sim_fps,
             sc->wind_amp, sc->wind_freq,
             THEMES[sc->theme_idx].name,
             sc->paused ? "PAUSED " : "swaying");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom-left key hint — PAIR_HINT bright cyan, A_BOLD */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset  w/s:wind  a/d:freq  t:theme  [/]:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/*
 * screen_present() — flush the composed frame to the terminal in one write.
 *
 * wnoutrefresh(stdscr) — copies the in-memory newscr into ncurses' pending-
 *   update structure.  No terminal I/O happens yet.
 * doupdate() — diffs newscr against curscr (what is physically on screen),
 *   sends only the changed cells to the terminal fd, then sets curscr=newscr.
 *
 * This two-step is the correct ncurses flush pattern.  Calling refresh()
 * (= wnoutrefresh + doupdate in one call) is fine for a single window, but
 * the two-step allows future multi-window batching into a single doupdate().
 */
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level container; everything outside the world.
 *
 * Intent
 *   Bundles the simulated world (Scene), the host terminal (Screen),
 *   and the session-level loop-control flags into one record so
 *   main() reads as four-line phases: init / service signals /
 *   step+draw / shutdown. Declared file-scope (g_app) so signal
 *   handlers — which cannot take a user argument — can write
 *   `running` and `need_resize` without globals scattered through
 *   the file.
 *
 * Locality of concern
 *   ── Owned subsystems ── nouns the app composes
 *      scene       — the world being simulated (§6)
 *      screen      — the terminal extent it draws to (§7)
 *
 *   ── Session state ── settings the user controls across resets
 *      sim_fps     — physics tick rate (cycled with [ / ])
 *
 *   ── Loop control ── verbs the loop reads each frame
 *      running     — clear → loop exits; set by SIGINT/SIGTERM
 *      need_resize — set by SIGWINCH; cleared after Screen refresh
 *
 * Why volatile sig_atomic_t (not bool, not int)
 *   `volatile`    : the compiler must not cache the flag across a
 *                   signal-handler write — every loop iteration must
 *                   re-read it from memory.
 *   `sig_atomic_t`: POSIX-guaranteed atomic with respect to async
 *                   signals; a plain `int` could be observed half-
 *                   written on architectures where stores are split.
 *   See [12] Raymond §"Signal handling".
 *
 * Things that DO NOT live here
 *   - Wall-clock timestamps / fps counters — main() locals; no
 *     other code path needs them.
 *   - Rope tuning values (wind_amp, wind_freq, theme_idx) —
 *     simulation/render state in §5 Scene.
 *   - Theme table itself — file-scope `static const` in §3.
 */
typedef struct {
    /* ── Owned subsystems ─────────────────────────────────────── */
    Scene  scene;              /* the world (§6)                       */
    Screen screen;             /* terminal extent (§7)                 */

    /* ── Session state ────────────────────────────────────────── */
    int    sim_fps;            /* physics tick rate (Hz)               */

    /* ── Loop control ─────────────────────────────────────────── */
    volatile sig_atomic_t running;      /* main loop predicate            */
    volatile sig_atomic_t need_resize;  /* SIGWINCH pending               */
} App;

static App g_app;

/* Signal handlers — set flags only; no ncurses or malloc calls here.
 * Signal handlers have severe restrictions (async-signal safety); setting
 * a volatile sig_atomic_t flag is one of the few safe operations. */
static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/*
 * cleanup() — atexit safety net.
 *
 * Registered with atexit() so endwin() is called even if the program exits
 * through an unhandled signal path that bypasses screen_free().  Without
 * this, the terminal could be left in raw/no-echo mode after a crash.
 */
static void cleanup(void) { endwin(); }

/*
 * app_do_resize() — handle a pending SIGWINCH terminal resize.
 *
 * WHAT:
 *   1. screen_resize() forces ncurses to re-read LINES/COLS from the kernel.
 *   2. scene_init() reinitialises all 7 ropes for the new dimensions.
 *      This is simpler and more correct than trying to relocate 7×20=140
 *      particles that may have scattered widely during strong wind.  The
 *      visual reset is instantaneous and imperceptible at resize time.
 *   3. need_resize is cleared.
 *
 * After app_do_resize() returns, the main loop resets frame_time and
 * sim_accum to prevent the large dt that accumulated during the resize
 * from firing a physics avalanche (hundreds of ticks in one frame).
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

/*
 * app_handle_key() — process one keypress; return false to quit.
 *
 * WHAT: Maps key codes to parameter mutations.  No physics logic here —
 *   only clamp + assign.  All clamping uses explicit if-guards rather than
 *   fmaxf/fminf so the logic is easy to follow and modify.
 *
 * KEY MAP:
 *   q / Q / ESC   — quit: return false → main loop exits.
 *   space         — toggle pause (freezes physics, rendering continues).
 *   r / R         — full reset: call scene_init() with current dimensions.
 *                   Theme is preserved by scene_init()'s save/restore logic.
 *   w / ↑         — wind_amp += 50 px/s²  [0, 1000]
 *                   More amplitude → wider lateral swing.
 *   s / ↓         — wind_amp -= 50 px/s²  [0, 1000]
 *                   Zero amplitude → gravity only; ropes hang straight.
 *   d / →         — wind_freq += 0.05 rad/s  [0.05, 4.0]
 *                   Higher frequency → faster oscillation tempo.
 *   a / ←         — wind_freq -= 0.05 rad/s  [0.05, 4.0]
 *                   Minimum 0.05 prevents division-by-zero (not used here)
 *                   and ensures a perceptible wind oscillation.
 *   t             — next colour theme (wraps 0 → N_THEMES-1 → 0).
 *   T             — previous colour theme (wraps 0 → N_THEMES-1 via modular
 *                   arithmetic: (idx − 1 + N_THEMES) % N_THEMES avoids
 *                   negative modulo which is implementation-defined in C).
 *   ] / + / =     — sim_fps += SIM_FPS_STEP  [SIM_FPS_MIN, SIM_FPS_MAX]
 *                   More physics ticks per second → tighter constraints.
 *   [ / -         — sim_fps -= SIM_FPS_STEP
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ':
        sc->paused = !sc->paused;
        break;

    case 'r': case 'R':
        scene_init(sc, app->screen.cols, app->screen.rows);
        break;

    /* Wind amplitude — controls the peak lateral force on each particle */
    case 'w': case KEY_UP:
        sc->wind_amp += 50.0f;
        if (sc->wind_amp > 1000.0f) sc->wind_amp = 1000.0f;
        break;
    case 's': case KEY_DOWN:
        sc->wind_amp -= 50.0f;
        if (sc->wind_amp < 0.0f) sc->wind_amp = 0.0f;
        break;

    /* Wind frequency — controls the oscillation tempo */
    case 'd': case KEY_RIGHT:
        sc->wind_freq += 0.05f;
        if (sc->wind_freq > 4.0f) sc->wind_freq = 4.0f;
        break;
    case 'a': case KEY_LEFT:
        sc->wind_freq -= 0.05f;
        if (sc->wind_freq < 0.05f) sc->wind_freq = 0.05f;
        break;

    /* Colour themes — t cycles forward, T cycles backward */
    case 't':
        sc->theme_idx = (sc->theme_idx + 1) % N_THEMES;
        theme_apply(sc->theme_idx);
        break;
    case 'T':
        sc->theme_idx = (sc->theme_idx - 1 + N_THEMES) % N_THEMES;
        theme_apply(sc->theme_idx);
        break;

    /* Simulation Hz — affects constraint convergence and physics accuracy */
    case ']': case '+': case '=':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[': case '-':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    default: break;
    }
    return true;
}

/* ─────────────────────────────────────────────────────────────────────
 * main() — the game loop  (structure identical to framework.c §8)
 *
 * The loop body executes these eight steps every frame:
 *
 *  ① RESIZE CHECK
 *     Test need_resize (set by SIGWINCH handler) before touching ncurses.
 *     app_do_resize() re-reads terminal size and re-inits the scene.
 *     frame_time and sim_accum are reset immediately after to prevent the
 *     large dt that accumulated during the resize from firing a burst of
 *     physics ticks (a "physics avalanche").
 *
 *  ② MEASURE dt
 *     Wall-clock nanoseconds elapsed since the previous frame.
 *     Capped at 100 ms: if the process was suspended (Ctrl-Z, debugger,
 *     sleep) and resumed, an uncapped dt would fire up to 6 physics ticks
 *     in one frame at 60 Hz × 100 ms — a sudden visual jump.
 *
 *  ③ FIXED-STEP ACCUMULATOR
 *     sim_accum accumulates wall-clock dt each frame.
 *     While sim_accum ≥ tick_ns (period of one physics tick):
 *       fire one scene_tick() and drain tick_ns from sim_accum.
 *     Result: physics runs at exactly sim_fps Hz on average, completely
 *     decoupled from the render frame rate.  If the render is slow
 *     (say 30 fps), two physics ticks fire per render frame.  If the
 *     render is fast (120 fps), one physics tick fires every two frames.
 *
 *  ④ ALPHA — sub-tick interpolation factor
 *     After draining, sim_accum holds the fractional leftover — how far
 *     into the next unfired tick we are.
 *       alpha = sim_accum / tick_ns  ∈ [0, 1)
 *     Passed to render_scene() so particle positions are lerped between
 *     the last physics state and the predicted next state, eliminating
 *     the periodic micro-stutter visible when sim Hz < render Hz.
 *
 *  ⑤ FPS COUNTER
 *     Frames counted over a 500 ms sliding window.
 *     fps = frame_count / (fps_accum_s).
 *     The 500 ms window gives a stable display without per-frame jitter.
 *
 *  ⑥ FRAME CAP — sleep BEFORE render
 *     budget  = NS_PER_SEC / 60   (one 60-fps frame in nanoseconds).
 *     elapsed = time spent on physics since frame_time was last sampled.
 *     sleep   = budget − elapsed.
 *     Sleeping BEFORE the ncurses write means terminal I/O cost is not
 *     charged against the next frame's budget.  If sleep ≤ 0 (frame
 *     already over budget), clock_sleep_ns() returns immediately.
 *
 *  ⑦ DRAW + PRESENT
 *     erase() → scene_draw() → HUD → wnoutrefresh() → doupdate().
 *     One atomic diff write to the terminal fd; no partial frames visible.
 *
 *  ⑧ DRAIN INPUT
 *     Loop getch() until ERR, processing every queued keypress.
 *     Looping (not single-call) drains all key-repeat events within the
 *     same frame, keeping parameter adjustments responsive when held.
 * ───────────────────────────────────────────────────────────────────── */
int main(void)
{
    /* Seed RNG from monotonic clock so each run looks different */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));

    /* Safety net: endwin() even if we exit via an unhandled path */
    atexit(cleanup);

    /* SIGINT / SIGTERM — graceful exit from Ctrl-C or kill */
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);

    /* SIGWINCH — terminal resize; handled at the top of the next iteration */
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();   /* timestamp at start of last frame */
    int64_t sim_accum   = 0;            /* nanoseconds in the physics bucket */
    int64_t fps_accum   = 0;            /* ns elapsed in current fps window  */
    int     frame_count = 0;            /* frames rendered in fps window     */
    double  fps_display = 0.0;          /* smoothed fps shown in HUD         */

    while (app->running) {

        int64_t frame_start = clock_ns();

        /* ── ① resize ────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();   /* reset so dt doesn't spike         */
            sim_accum  = 0;
        }

        /* ── ② dt ────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;   /* suspend guard */

        /* ── ③ fixed-step accumulator ────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);   /* ns per physics tick   */
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── ④ alpha ─────────────────────────────────────────────── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── ⑤ fps counter ───────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── ⑥ frame cap — sleep before render ──────────────────── *
         * Budget = 1/60 s.  elapsed is wall time spent on physics +
         * accounting since frame_start; sleep the remainder so the
         * render rate sits at 60 fps regardless of sim Hz.            */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── ⑦ draw + present ────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
                    alpha, dt_sec);
        screen_present();

        /* ── ⑧ drain all pending input ──────────────────────────── */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }
    }

    screen_free(&app->screen);
    return 0;
}
