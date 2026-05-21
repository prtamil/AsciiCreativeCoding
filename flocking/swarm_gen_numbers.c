/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * swarm_gen_numbers.c — 120-agent steering swarm forms ASCII digits
 *
 * DEMO: A swarm of 120 ASCII particles coordinates itself through
 *       Reynolds steering behaviours to form the shapes of digits 0-9
 *       on screen.  Each '#' in a digit's 5x7 bitmap is subdivided
 *       into a 3x3 grid of mini-slots — 99 to 153 slots per digit —
 *       so the formed digit reads as a dense cluster of glyphs, not
 *       a sparse outline.  Ten switchable strategies (drift, rush,
 *       flow, orbit, flock, pulse, vortex, gravity, spring, wave)
 *       give the same digit ten different formation rhythms.  Ten
 *       colour themes cycle live with t / T.
 *
 * Study alongside: flocking/flocking.c (boid forces + steering primer)
 *                  flocking/crowd.c    (Person model / single flock)
 *
 * Section map:
 *   §1 config   — strategies, slot subdivision, speeds, force weights;
 *                 StrategyParams struct + g_presets[N_STRATEGIES] table
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 10 theme palettes + PAIR_HUD/PAIR_HINT + theme_apply
 *   §4 coords   — pixel↔cell aspect bridge + Vec2 helpers
 *   §5 entity   — Agent + Slot structs, spawn, agent_step
 *   §6 steering — seek / arrive / separate / wander / cohesion / align / spring
 *                 (each takes const StrategyParams *sp where it needs tunables)
 *   §7 strategy — ten tick functions, each taking const StrategyParams *sp
 *                 (dispatch via agent_tick(... sp ...))
 *   §8 digit    — bitmap library, slot extraction, greedy assignment
 *   §9 scene    — World + SimControls + Strategy sub-structs; SwarmScene
 *                 root; scene_set_digit / scene_set_strategy / scene_tick
 *                 / scene_draw / mark_cell
 *   §10 app     — FpsCounter + App; signals, resize, key dispatch,
 *                 main game loop (8 numbered phases)
 *
 * Keys:
 *   0-9    form that digit                a         auto-cycle (3 s/digit)
 *   n / p  next / prev strategy           space     pause / resume
 *   r      scatter agents                 t / T     next / prev theme
 *   q/ESC  quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra flocking/swarm_gen_numbers.c \
 *       -o swarm_gen_numbers -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Reynolds-style steering swarm with greedy slot
 *                  assignment.  Each tick every agent computes a force
 *                  vector — different strategies mix different forces
 *                  (seek, arrive, separate, wander, cohesion, align,
 *                  spring, plus per-strategy extras) — integrates it
 *                  into velocity, clamps to max_speed, and integrates
 *                  position.  The slot-arrive force pulls each agent
 *                  toward its assigned target point in the digit; the
 *                  greedy nearest-unoccupied assignment ensures every
 *                  agent knows where to go without the cost of an
 *                  optimal Hungarian solve.
 *
 *                  Each '#' in the digit's 5x7 bitmap is subdivided
 *                  into a SLOT_GRID x SLOT_GRID grid of mini-slots,
 *                  giving 9 slots per '#' cell at the default.  This
 *                  packs ~100-150 agents into the digit shape so the
 *                  formed digit reads as a filled cluster rather than
 *                  a sparse skeleton.
 *
 * Data-structure : SwarmScene owns a fixed Agent pool (all 120 always
 *                  live), a Slot pool sized to SLOTS_MAX (largest
 *                  possible digit), the active digit/strategy/theme
 *                  indices, and the simulation clock used by PULSE/
 *                  WAVE.  Agents carry pos/prev_pos/vel + their slot
 *                  index (-1 if wandering) + their fixed glyph and
 *                  per-agent colour pair.  Slots carry pos + an
 *                  occupied flag used during greedy assignment.
 *
 * Rendering      : Sub-tick alpha lerp of pos for smooth motion.
 *                  Agent attributes derive from slot state — A_BOLD
 *                  if at slot (within AT_SLOT_DIST), A_DIM if no
 *                  slot, A_NORMAL otherwise.  Every glyph stamp goes
 *                  through a mark_cell helper that performs the
 *                  (chtype)(unsigned char) cast and bounds-check.
 *
 * Performance    : Separation runs O(N²) every tick.  At N = 160,
 *                  160·159 = 25 440 distance checks per tick ≈
 *                  1.5 M/s at 60 Hz — sub-millisecond at -O2.
 *                  FLOCK adds two more O(N²) passes for cohesion
 *                  and alignment but only when active.  Slot
 *                  assignment is O(N·S), runs once per digit change
 *                  (≈ 160·153 = 24 500 ops, microseconds).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 *   ── Canonical steering / flocking ──────────────────────────────
 *   [1] Reynolds, C. W. (1987), "Flocks, Herds, and Schools: A
 *       Distributed Behavioral Model", SIGGRAPH '87 Computer Graphics
 *       21(4), pp. 25-34 — THE source of every force used here.
 *       Defines the original three-rule boid model (separation,
 *       alignment, cohesion).  §6 steer_separate / _cohesion /
 *       _align implement these directly; the FLOCK strategy is
 *       Reynolds 1987 unaltered.
 *   [2] Reynolds, C. W. (1999), "Steering Behaviors for Autonomous
 *       Characters", Game Developers Conference 1999.  Online at
 *       https://www.red3d.com/cwr/steer/ — the seek / arrive /
 *       wander / pursue primer.  §6 steer_seek / _arrive / _wander
 *       are 1:1 implementations of Reynolds's reference algorithms.
 *
 *   ── Collective-behaviour biology ───────────────────────────────
 *   [3] Couzin, I. D., Krause, J., James, R., Ruxton, G. D. &
 *       Franks, N. R. (2002), "Collective memory and spatial sorting
 *       in animal groups", J. Theor. Biol. 218(1), pp. 1-11 —
 *       formalises the zones of REPULSION / ORIENTATION / ATTRACTION.
 *       Justifies the StrategyParams ordering: sep_radius (repulsion
 *       zone) ≤ neighbor_radius (alignment + cohesion zone).
 *   [4] Vicsek, T., Czirók, A., Ben-Jacob, E., Cohen, I. & Shochet, O.
 *       (1995), "Novel type of phase transition in a system of
 *       self-driven particles", Phys. Rev. Lett. 75(6), pp. 1226-1229
 *       — alignment-driven ordering in agent ensembles.  The FLOCK
 *       strategy reproduces this phase transition: low align_weight →
 *       fragmented swirl, high align_weight → coherent flow.
 *
 *   ── Damped oscillator (SPRING strategy) ────────────────────────
 *   [5] Strogatz, S. H. (2014), "Nonlinear Dynamics and Chaos" (2nd
 *       ed.), Westview Press, Ch. 5 — the damped-harmonic-oscillator
 *       ODE
 *
 *           m·ẍ + c·ẋ + k·(x − target) = 0
 *
 *       with damping ratio ζ = c / (2·√(m·k)) ≈ 0.53 at the file's
 *       defaults (SPRING_K=3.5, SPRING_DAMP=2.0) — UNDERDAMPED, so
 *       agents overshoot the slot and ring back.  §6 steer_spring
 *       implements this directly with m = 1.
 *
 *   ── Slot assignment / matching ─────────────────────────────────
 *   [6] Kuhn, H. W. (1955), "The Hungarian method for the assignment
 *       problem", Naval Research Logistics Quarterly 2(1-2), pp. 83-97
 *       — the optimal O(N³) bipartite assignment algorithm.  This
 *       file uses a GREEDY nearest-slot approximation (assign_slots in
 *       §8) instead: O(N·S) per digit change, suboptimal but cheap.
 *       For 160 agents on a small slot count the visual difference
 *       from optimal is negligible.
 *
 *   ── Bitmap typography (digit templates) ────────────────────────
 *   [7] Knuth, D. E. (1979), "TeX and METAFONT: New Directions in
 *       Typesetting", AMS — origin of programmable bitmap fonts.
 *       The 5×7 digit templates in §8 follow the same "small grid,
 *       readable at minimum size" tradition; see also any LCD /
 *       calculator-display font sheet (3×5, 5×7, 5×8 are the
 *       canonical sizes).
 *
 *   ── Game loop / fixed-step physics ─────────────────────────────
 *   [8] Fiedler, G. (2004, updated 2014), "Fix Your Timestep!",
 *       https://gafferongames.com/post/fix_your_timestep/ — the
 *       fixed-step accumulator + sub-tick alpha-lerp pattern
 *       implemented in §10 main.  Caps dt at 100 ms to prevent the
 *       avalanche spiral on slow terminals.
 *
 *   ── Implementation-oriented references ─────────────────────────
 *   [9] Shiffman, D., "The Nature of Code", Ch. 6 (Autonomous Agents)
 *       — reading-level walkthrough of every Reynolds steering force
 *       with Processing examples.  Pair with [2] for the math.
 *       https://natureofcode.com/book/chapter-6-autonomous-agents/
 *
 *   ── Online quick reference ─────────────────────────────────────
 *  [10] Wikipedia: "Boids", "Damped harmonic motion", "Vicsek model",
 *       "Hungarian algorithm".  Useful one-paragraph summaries that
 *       link out to the primary literature.
 *
 *   ── Companion files in this project ────────────────────────────
 *   See also:
 *     flocking/flocking.c    — five switchable flocking algorithms;
 *       the same StrategyParams + dispatch table pattern at a
 *       smaller scale.
 *     flocking/crowd.c       — six steering behaviours on a single
 *       Person model; the steering primitives map 1:1 to this
 *       file's §6.
 *     flocking/murmuration.c — large-N flocking (N=1500) with
 *       density-field rendering and spatial hashing.
 *     flocking/shepherd.c    — Strömbom shepherd-and-sheep model;
 *       sheep use the same separation + cohesion primitives.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Pin a target point to every agent (its slot), then sum a few
 * direction-of-motion forces — "go to the slot", "stay clear of
 * neighbours", plus whatever extra mood the strategy adds — and
 * integrate.  The digit shape is just the spatial pattern of slot
 * positions; the strategy is just the recipe of forces.  Same digit,
 * ten recipes → ten different visual rhythms of the same final
 * formation.
 *
 *
 * ALGORITHM IN STEPS  (per tick, per agent — strategy_*)
 * ──────────────────────────────────────────────────────
 *   1. Compute the strategy's forces (each is a Vec2 in px/s²):
 *        - slot-arrive via agent_arrive_at_slot_if_assigned
 *          (returns slot_arrive × slot_weight if slot_idx ≥ 0,
 *          else 0)
 *        - separation via steer_separate (always present)
 *        - strategy-specific extras:
 *            DRIFT/FLOW/FLOCK   wander (FLOCK also uses
 *                                wander_fade_scale_for_slot)
 *            FLOW               rightward stream until aligned
 *            ORBIT              tangent to digit centroid
 *            FLOCK              cohesion + alignment
 *            PULSE              oscillating slot target
 *            VORTEX             tangent to own slot
 *            GRAVITY            constant downward pull
 *            SPRING             Hooke spring force replaces arrive
 *            WAVE               lateral sin perpendicular to approach
 *   2. force = Σ weighted forces
 *   3. agent_commit_force (a, force, sp, dt, ww, wh) — the final
 *      "agent_step + bounce_pos" sandwich:
 *        a. vel       = clamp(vel + force · dt, sp->max_speed)
 *        b. prev_pos  = pos
 *        c. pos      += vel · dt
 *        d. bounce off walls if outside [0, ww) × [0, wh)
 *
 * KEY FORMULAS
 * ────────────
 *   Seek:        desired = normalize(target − pos) · speed
 *                force   = desired − vel
 *
 *   Arrive (seek with deceleration inside slow_radius):
 *                desired_speed = max_speed · min(1, dist / slow_radius)
 *                force         = desired - vel
 *
 *   Separate (per neighbour with 0 < d < sep_radius):
 *                strength = (sep_radius − d) / sep_radius
 *                force   += normalize(self − neighbour) · strength · 60 px/s
 *
 *   Spring (Hooke + linear damping; SPRING strategy):
 *                force = SPRING_K · (target − pos) − SPRING_DAMP · vel
 *                ζ     = SPRING_DAMP / (2·sqrt(SPRING_K)) ≈ 0.53 (underdamped)
 *
 *   Slot subdivision (digit_load + emit_subslot_grid):
 *                layout = digit_layout_centred(cols, rows)
 *                for each '#' at bitmap (r, c):
 *                  emit_subslot_grid(slots, &n, &layout,
 *                                    c·DIGIT_CELL_W, r·DIGIT_CELL_H)
 *                  // emits SLOT_GRID² cell-aligned sub-slots
 *
 *   Pixel→cell:  cx = round(px / CELL_W)        (CELL_W = 8)
 *                cy = round(py / CELL_H)        (CELL_H = 16)
 *
 *
 * Background you need
 * ───────────────────
 *   - flocking.c T1-T3 (boid + Reynolds rules).
 *   - crowd.c T1, T6 (steering primitives + slot assignment
 *     pattern).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Hungarian algorithm for OPTIMAL assignment. Greedy
 *     nearest is good enough at N=160 (the visual difference
 *     is minor).
 *   - Differential equations / ODE solvers. The SPRING
 *     strategy is just Hooke's law integrated with Euler;
 *     no fancy integrator needed at terminal scale.
 *   - Genetic / agent-based optimisation. Strategies are
 *     hand-tuned, not evolved.
 *
 * ─────────────────────────────────────────────────────────────────────── */


#define _POSIX_C_SOURCE 200809L
#include <float.h>
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                            */
/* ===================================================================== */

/*
 * StrategyParams — all tuneable behaviour constants for one steering
 * preset (one strategy = one row of the g_presets[] table).
 *
 * Intent
 *   Every strategy_* function takes `const StrategyParams *sp` and
 *   reads sp->* for its coefficients.  The SwarmScene's `strategy`
 *   sub-struct (in §9) caches the pointer to the active preset so a
 *   key-press takes effect on the very next tick without re-indexing
 *   the table.
 *
 * Fields
 * ──────
 *   name            HUD label ("DRIFT", "RUSH", ...)
 *   max_speed       velocity cap for all agents (px/s)
 *   arrive_speed    desired speed toward slot (arrive target speed)
 *   slow_radius     px from slot where arrive begins decelerating
 *   slot_weight     multiplier on the slot-arrive steering force
 *   wander_strength amplitude of the Brownian noise force
 *   sep_radius      personal-space radius between agents (px)
 *   sep_weight      separation force multiplier
 *   cohesion_weight FLOCK: pull toward the local group centroid
 *   align_weight    FLOCK: match the local group's average velocity
 *   neighbor_radius FLOCK: radius for counting local neighbours
 *
 * References
 *   [3] Reynolds 1999 — every weight/radius here maps directly to a
 *       Reynolds steering primitive.  Pair this struct with the §6
 *       steer_* helpers to see the 1:1 mapping.
 */
typedef struct {
    const char *name;
    float max_speed;
    float arrive_speed;
    float slow_radius;
    float slot_weight;
    float wander_strength;
    float sep_radius;
    float sep_weight;
    float cohesion_weight;
    float align_weight;
    float neighbor_radius;
} StrategyParams;

/*
 * Ten presets — each strategy produces a distinct visual rhythm:
 *
 *  DRIFT   Agents wander randomly, pulled gently toward slots.
 *          Dreamy, organic; digit forms slowly from the cloud.
 *
 *  RUSH    Agents sprint directly to slots with arrive deceleration.
 *          Snappy and precise; digit snaps into place quickly.
 *
 *  FLOW    A rightward current carries agents across the screen.
 *          Once horizontally aligned with their slot the current
 *          gives way to slot-seek — a "painting" effect.
 *
 *  ORBIT   Agents orbit the digit centroid tangentially while
 *          simultaneously drifting toward their assigned slot.
 *          Produces a galaxy-like spiral collapse.
 *
 *  FLOCK   Reynolds boids: cohesion + alignment + separation +
 *          weak slot attraction.  The flock moves as a single
 *          organism and morphs into the digit shape.
 *
 *  PULSE   Agents reach their slots then are periodically pushed
 *          outward and pulled back.  The digit "breathes".
 *
 *  VORTEX  Like ORBIT but each agent spirals around its OWN slot
 *          rather than the centroid.  Every agent has a private
 *          whirlpool that sucks it toward its slot.
 *
 *  GRAVITY A constant downward force biases all agents' paths.
 *          Slot-seek still wins, but agents approach from above
 *          and overshoot slightly — a "raindrop" aesthetic.
 *
 *  SPRING  Hooke's law replaces the arrive steering.  The spring
 *          is slightly underdamped so agents oscillate around their
 *          slot before settling — like a bouncing ball.
 *
 *  WAVE    Agents wiggle laterally (perpendicular to their approach
 *          direction) as they close in.  Amplitude fades at the slot
 *          so they settle cleanly; en route they snake and weave.
 */
/*
 * Force balance guide — the slot force must dominate all other forces:
 *
 *   slot force  = arrive_speed × slot_weight          (at rest, far from slot)
 *   sep force   = ≤ 60.0 × sep_weight × n_neighbours  (max per neighbour = 60)
 *   wander force= wander_strength                      (constant magnitude)
 *   orbit force = ORBIT_STRENGTH or VORTEX_STRENGTH   (orbit strategies)
 *   gravity     = GRAVITY_PULL                         (GRAVITY strategy only)
 *
 * Rule: arrive_speed × slot_weight >> sep_force + wander + orbit
 *
 * With sep fixed at 60 base (not arrive_speed), a typical cluster of
 * 3 close neighbours contributes 60 × sep_weight × 3.  Keep this below
 * ~50% of the slot force so the digit forms even in dense regions.
 */
#define N_STRATEGIES  10

static const StrategyParams g_presets[N_STRATEGIES] = {
    /*          max   arr   slow  slotW wand  sepR  sepW  coh  aln  nbr  */

    /* DRIFT: meandering path, arrives slowly — wander is 10% of slot force */
    { "DRIFT",   70.0f, 70.0f, 55.0f, 2.5f, 18.0f, 14.0f, 0.8f, 0.0f, 0.0f,  0.0f },

    /* RUSH: fast sprint, arrive deceleration prevents overshoot */
    { "RUSH",   200.0f,180.0f, 80.0f, 6.0f,  0.0f, 14.0f, 0.6f, 0.0f, 0.0f,  0.0f },

    /* FLOW: rightward stream; strong slot force takes over when aligned */
    { "FLOW",   120.0f,100.0f, 70.0f, 4.0f, 10.0f, 14.0f, 0.8f, 0.0f, 0.0f,  0.0f },

    /* ORBIT: slot force (110×4.5=495) >> ORBIT_STRENGTH (40) → spiral in */
    { "ORBIT",  130.0f,110.0f, 90.0f, 4.5f,  0.0f, 14.0f, 0.8f, 0.0f, 0.0f,  0.0f },

    /* FLOCK: boid forces shape the group; slot force (70×3.0=210) guides it */
    { "FLOCK",   90.0f, 70.0f, 80.0f, 3.0f, 12.0f, 18.0f, 1.2f, 0.9f, 0.5f, 80.0f },

    /* PULSE: strong slot keep (140×5.0=700) with oscillating target */
    { "PULSE",  160.0f,140.0f, 50.0f, 5.0f,  0.0f, 14.0f, 0.7f, 0.0f, 0.0f,  0.0f },

    /* VORTEX: per-slot spiral (120×4.0=480) >> VORTEX_STRENGTH (35) */
    { "VORTEX", 150.0f,120.0f, 80.0f, 4.0f,  0.0f, 14.0f, 0.7f, 0.0f, 0.0f,  0.0f },

    /* GRAVITY: slot force (150×5.0=750) >> GRAVITY_PULL (60) */
    { "GRAVITY",180.0f,150.0f, 60.0f, 5.0f,  0.0f, 14.0f, 0.6f, 0.0f, 0.0f,  0.0f },

    /* SPRING: spring k=3.5, damp=2.0 → ζ≈0.53 → underdamped oscillation */
    { "SPRING", 160.0f,130.0f, 70.0f, 4.0f,  0.0f, 14.0f, 0.7f, 0.0f, 0.0f,  0.0f },

    /* WAVE: slot force (110×4.5=495) >> max wave amplitude (40) */
    { "WAVE",   140.0f,110.0f, 65.0f, 4.5f,  0.0f, 14.0f, 0.7f, 0.0f, 0.0f,  0.0f },
};

/* g_presets[] is the const table of all strategy presets, indexed by
 * Strategy.index in §9 SwarmScene.  Strategy.params on Scene caches
 * the pointer to the active preset (= &g_presets[index]) so hot
 * loops avoid the array index. */

/*
 * Digit bitmaps — 5-column × 7-row templates.
 * '#' = filled pixel → becomes one target Slot in pixel space.
 * ' ' = empty pixel  → ignored.
 *
 * Slot counts per digit (for reference when tuning N_AGENTS):
 *   0→16  1→12  2→15  3→15  4→14  5→17  6→16  7→11  8→17  9→16
 *   Maximum = 17  Minimum = 11  (digit 7 has fewest points)
 */
#define DIGIT_NCOLS  5   /* template columns */
#define DIGIT_NROWS  7   /* template rows    */

static const char *const DIGIT_BITMAPS[10][DIGIT_NROWS] = {
    { " ### ", "#   #", "#   #", "#   #", "#   #", "#   #", " ### " }, /* 0 */
    { "  #  ", " ##  ", "  #  ", "  #  ", "  #  ", "  #  ", "#####" }, /* 1 */
    { " ### ", "#   #", "    #", "  ## ", " #   ", "#    ", "#####" }, /* 2 */
    { " ####", "    #", "    #", "  ###", "    #", "    #", " ####" }, /* 3 */
    { "#   #", "#   #", "#   #", "#####", "    #", "    #", "    #" }, /* 4 */
    { "#####", "#    ", "#    ", "#### ", "    #", "    #", "#### " }, /* 5 */
    { " ### ", "#    ", "#    ", "#### ", "#   #", "#   #", " ### " }, /* 6 */
    { "#####", "    #", "   # ", "  #  ", " #   ", "#    ", "#    " }, /* 7 */
    { " ### ", "#   #", "#   #", " ### ", "#   #", "#   #", " ### " }, /* 8 */
    { " ### ", "#   #", "#   #", " ####", "    #", "    #", " ### " }, /* 9 */
};

/*
 * Scaling: each template cell maps to DIGIT_CELL_W × DIGIT_CELL_H
 * terminal cells.  At the default values a digit spans 25 × 21 cells
 * which fits in 80 × 24 with a 2-row HUD reserve.
 *
 * Why DIGIT_CELL_H = 3 (and not 2)?  We subdivide each '#' into a
 * SLOT_GRID × SLOT_GRID grid of sub-slots in §8.  When SLOT_GRID = 3
 * we want 3 distinct terminal rows per cell so each sub-slot row
 * lands cleanly in its own row.  At DIGIT_CELL_H = 2 the third
 * sub-slot row leaks into the NEXT bitmap row's allocated terminal
 * rows (because px_to_cell_y rounds half-up at 8-px boundaries),
 * blurring the digit's vertical structure — the historical "6 looks
 * like 5" bug.  Setting DIGIT_CELL_H = SLOT_GRID = 3 makes the
 * mapping exact: one sub-slot row per terminal row.
 */
#define DIGIT_CELL_W   5   /* terminal columns per template column */
#define DIGIT_CELL_H   3   /* terminal rows per template row       */

/* Simulation sizes
 *
 * SLOT_GRID — sub-slot subdivision per '#' cell in the digit bitmap.
 *   3x3 = 9 sub-slots per '#', so digit slot counts are:
 *      digit 0,6,9 → 16 × 9 = 144 slots
 *      digit 1     → 12 × 9 = 108
 *      digit 2,3   → 15 × 9 = 135
 *      digit 4     → 14 × 9 = 126
 *      digit 5,8   → 17 × 9 = 153  (the maximum)
 *      digit 7     → 11 × 9 =  99  (the minimum)
 *
 * N_AGENTS — total agents in the swarm.  Set to SLOTS_MAX so every
 *   slot of every digit can be filled — including digits 5 and 8
 *   which need all 153 slots populated to read clearly.  Smaller
 *   digits (7 has only 99 slots) leave the remaining agents in the
 *   wander state, milling around the formed digit; this is visually
 *   fine because they render A_DIM and the digit shape stays clear.
 *
 * SLOTS_MAX — upper bound on slot count.  Sized to the maximum
 *   possible (digit 5/8: 17 '#' × SLOT_GRID²); also caps N_AGENTS.
 *
 * N_COLORS — number of body-colour pairs (1..N_COLORS).  Agent i
 *   uses pair (i % N_COLORS) + 1.  Combined with the THEMES table
 *   in §3, the swarm cycles through 7 theme tints round-robin.
 */
enum {
    SLOT_GRID       =   3,   /* 3x3 sub-slot grid per '#' = 9 slots/cell */
    SLOTS_MAX       = 160,   /* 17 # × 9 sub-slots = 153, plus margin    */
    N_AGENTS        = 160,   /* matches SLOTS_MAX so every slot fills    */
    SIM_FPS_DEFAULT =  60,
    TARGET_FPS      =  60,
    FPS_UPDATE_MS   = 500,
    N_COLORS        =   7,
    AUTO_CYCLE_S    =   3,    /* seconds per digit in auto-cycle mode */

    PAIR_HUD        =   8,    /* bright yellow — top status bar  */
    PAIR_HINT       =   9,    /* bright cyan   — bottom key hint */

    N_THEMES        =  10,    /* see THEMES[] in §3 */
};

/* Cell dimensions — physics in px, draw in cells; convert only at render */
#define CELL_W   8
#define CELL_H  16

/* Agent is "at its slot" when closer than this; rendered bold */
#define AT_SLOT_DIST     14.0f

/* Wander: max angular change per second (radians); keeps turns smooth */
#define WANDER_TURN_MAX   5.0f

/* ORBIT: tangential speed around the digit centroid.
 * Must be weaker than the slot force (arrive_speed × slot_weight)
 * so agents eventually settle rather than orbiting forever. */
#define ORBIT_STRENGTH   40.0f

/* FLOW: rightward bias speed and x-distance threshold for switching
 * from flow to slot-arrive.  Once within FLOW_X_THRESH px of the
 * slot's x-coordinate, the agent switches to pure slot-arrive. */
#define FLOW_BIAS        50.0f
#define FLOW_X_THRESH    48.0f

/* PULSE: the digit breathes at PULSE_FREQ Hz with PULSE_AMPLITUDE px */
#define PULSE_FREQ        1.2f
#define PULSE_AMPLITUDE  55.0f
#define TWO_PI            6.28318530f

/* VORTEX: tangential speed around each agent's OWN slot.
 * Scaled by distance so vortex → 0 as agent reaches the slot. */
#define VORTEX_STRENGTH  35.0f
#define VORTEX_FADE_DIST 40.0f  /* px: full strength at this distance */

/* GRAVITY: constant downward acceleration (px/s²) */
#define GRAVITY_PULL     60.0f

/* SPRING: Hooke's law constants.
 * Damping ratio ζ = SPRING_DAMP / (2·√SPRING_K) ≈ 0.53 → underdamped.
 * Agents oscillate ~1.5–2 cycles before settling — visually springy. */
#define SPRING_K          3.5f
#define SPRING_DAMP       2.0f

/* WAVE: lateral sinusoidal oscillation perpendicular to approach.
 * Amplitude fades linearly to 0 inside WAVE_FADE_DIST so agents
 * settle cleanly without perpetual wiggling at the slot. */
#define WAVE_AMPLITUDE   40.0f
#define WAVE_FREQ         2.0f
#define WAVE_FADE_DIST   50.0f  /* px: wave is zero below this distance */

/* Timing */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                            */
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
/* §3  color — 10-theme palette + dedicated HUD pairs                    */
/* ===================================================================== */

/*
 * Theme — one named colour palette for the agent swarm.
 *
 * Intent
 *   The swarm uses N_COLORS distinct ncurses pairs (one per agent
 *   "colour bucket").  Agent i picks pair `(i % N_COLORS) + 1` at
 *   spawn — fixed for the agent's lifetime.  Cycling themes (t / T
 *   keys) re-registers all N_COLORS pairs from the active Theme's
 *   body[] in one call; every previously-spawned agent's glyph
 *   instantly takes on the new palette because the colour pair
 *   number is unchanged, only the pair's foreground colour was
 *   re-set.
 *
 *   This is cheaper than re-keying every agent or re-registering
 *   pairs per-tick: one O(N_COLORS) call on a key event vs O(N)
 *   per tick.
 *
 * Why no HUD / hint colours inside Theme
 *   PAIR_HUD and PAIR_HINT are theme-INDEPENDENT (canonical bright
 *   yellow + bright cyan per CLAUDE.md §"HUD Standard") so the
 *   status row stays legible regardless of which Theme is active.
 *   They're set ONCE in screen_init() and survive every theme cycle.
 *
 * Members
 *   name     HUD label ("Rainbow", "Pastel", "Ocean", ...).
 *   body[]   N_COLORS xterm-256 foreground indices, indexed by
 *            agent colour bucket.  All indices ≥ 24 per the project
 *            palette-brightness rule so A_DIM stays legible (entries
 *            below 24 become invisible against the default black
 *            background when wandering agents are rendered dim).
 *
 * Invariants
 *   24 ≤ body[i] ≤ 255 for every i.
 *   name != NULL.
 *
 * References
 *   See CLAUDE.md §"Theme Palette Brightness" for the brightness-
 *   floor rule.  N_COLORS is fixed at 7 to match the 7-distinct-glyph
 *   AGENT_GLYPHS pool below — together they give 7 × 7 = 49 visually
 *   distinct agent appearances before duplication.
 */
typedef struct {
    const char *name;
    int         body[N_COLORS];
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name        agent 0 ←──────────────────────────→ agent 6 */
    {"Rainbow", { 196, 208, 226,  46,  51,  33, 201}},
    {"Solar",   { 226, 220, 214, 208, 202, 196, 160}},
    {"Ocean",   {  24,  27,  33,  38,  45,  51, 117}},
    {"Fire",    { 196, 202, 208, 214, 220, 226, 227}},
    {"Matrix",  {  28,  34,  40,  46,  76,  82, 118}},
    {"Aurora",  {  28,  34,  79, 122, 159, 165, 201}},
    {"Neon",    { 201, 165, 129,  93,  57,  51,  45}},
    {"Sunset",  {  54,  91, 128, 165, 202, 209, 208}},
    {"Toxic",   {  28,  58,  64,  70,  76,  82, 118}},
    {"Ghost",   { 244, 247, 250, 252, 253, 254, 255}},
};

/*
 * theme_apply — register palette idx with ncurses.  Background = -1
 * (terminal default) so the demo respects the user's theme.  The
 * 8-colour fallback approximates the warm-to-cool feel.
 *
 * PAIR_HUD/PAIR_HINT are theme-independent and configured once in
 * color_init().
 */
static void theme_apply(int idx)
{
    const Theme *th = &THEMES[idx];
    if (COLORS >= 256) {
        for (int p = 0; p < N_COLORS; p++)
            init_pair(p + 1, th->body[p], -1);
    } else {
        static const int fb8[N_COLORS] = {
            COLOR_RED, COLOR_RED, COLOR_YELLOW,
            COLOR_GREEN, COLOR_CYAN, COLOR_BLUE, COLOR_MAGENTA
        };
        for (int p = 0; p < N_COLORS; p++)
            init_pair(p + 1, fb8[p], -1);
    }
}

/*
 * color_init — one-time colour setup.
 *   start_color()        — initialise ncurses colour support.
 *   use_default_colors() — allow background = -1 to mean "terminal default".
 *   theme_apply()        — register the initial body palette (Rainbow).
 *   PAIR_HUD/PAIR_HINT   — bright yellow / cyan, theme-independent.
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
/* §4  coords & vec2                                                    */
/* ===================================================================== */

static inline float pw(int cols) { return (float)(cols * CELL_W); }
static inline float ph(int rows) { return (float)(rows * CELL_H); }

static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/*
 * Vec2 — 2-D float vector used everywhere physics or geometry shows
 * up: position, velocity, force, offset, centroid.
 *
 * Intent
 *   Functions return Vec2 BY VALUE (no out-params); the v2* helpers
 *   below compose into terse chains that mirror the math notation
 *   they implement:
 *
 *       Vec2 force = v2add(v2scale(sep, sp->sep_weight),
 *                          v2scale(coh, sp->cohesion_weight));
 *
 *   reads as `force = w_sep·sep + w_coh·coh` to a math-fluent reader.
 *
 * Why a struct (not float[2] or two scalars)
 *   • Named type makes every steering signature read in the
 *     algorithm's vocabulary: `steer_seek(pos, vel, target, speed)`
 *     rather than `steer_seek(px, py, vx, vy, tx, ty, speed)`.
 *   • Return by value works without spilling to memory on x86-64 +
 *     ARM ABIs (8 bytes = one register pair); no perf penalty.
 *   • Eliminates the out-parameter idiom in every steering helper.
 *
 * Members
 *   x, y    Cartesian components in PIXEL space (units = px).  Cell-
 *           space conversion happens only in scene_draw via
 *           px_to_cell_x/y.  Physics never sees cells.
 *
 * References
 *   [2] Reynolds 1999 — every steering primitive returns a Vec2
 *       force; this file's §6 mirrors that API shape.
 *   [9] Shiffman, *Nature of Code* Ch.1 — PVector pedagogy that this
 *       type is the C analogue of.
 */
typedef struct { float x, y; } Vec2;

static inline Vec2  v2(float x, float y)      { return (Vec2){x, y};           }
static inline Vec2  v2add(Vec2 a, Vec2 b)      { return v2(a.x+b.x, a.y+b.y);  }
static inline Vec2  v2sub(Vec2 a, Vec2 b)      { return v2(a.x-b.x, a.y-b.y);  }
static inline Vec2  v2scale(Vec2 v, float s)   { return v2(v.x*s,   v.y*s);    }
static inline float v2len(Vec2 v)              { return sqrtf(v.x*v.x+v.y*v.y); }
static inline float v2len2(Vec2 v)             { return v.x*v.x + v.y*v.y;     }

static inline Vec2 v2norm(Vec2 v)
{
    float l = v2len(v);
    return (l > 0.001f) ? v2scale(v, 1.0f/l) : v2(0, 0);
}
static inline Vec2 v2clamp_len(Vec2 v, float max_len)
{
    float l = v2len(v);
    return (l > max_len) ? v2scale(v2norm(v), max_len) : v;
}

/* bounce_pos — elastic wall bounce; velocity component flips on contact */
static void bounce_pos(Vec2 *pos, Vec2 *vel, float ww, float wh)
{
    if (pos->x <  0)  { pos->x = 0;    vel->x =  fabsf(vel->x); }
    if (pos->x >= ww) { pos->x = ww-1; vel->x = -fabsf(vel->x); }
    if (pos->y <  0)  { pos->y = 0;    vel->y =  fabsf(vel->y); }
    if (pos->y >= wh) { pos->y = wh-1; vel->y = -fabsf(vel->y); }
}

static float randf(void) { return (float)rand() / (float)RAND_MAX; }

/* ===================================================================== */
/* §5  entity                                                           */
/* ===================================================================== */

/*
 * Agent — one steered particle in the swarm.
 *
 * Intent
 *   The atomic unit of the simulation.  Each tick, every Agent runs
 *   the active strategy_*() function — a weighted sum of Reynolds
 *   steering forces ([1], [2]) — and integrates the result via
 *   agent_step.  N_AGENTS = 160 agents share the screen; each is
 *   assigned to exactly one Slot when forming a digit.
 *
 * Why prev_pos (a snapshot, not just current pos)
 *   Physics ticks at sim_fps (typically 60 Hz) but the renderer can
 *   draw at any rate.  The renderer LINEARLY INTERPOLATES between
 *   prev_pos and pos using the sub-tick alpha ∈ [0, 1) from the
 *   fixed-step accumulator (Fiedler [8]).  Without this, fast agents
 *   would jitter visibly between tick boundaries.
 *
 * Why a per-agent wander_angle (not a fresh random each tick)
 *   Reynolds [2] §"Wander" specifies a *slowly rotating* heading so
 *   the wander force has TEMPORAL COHERENCE — agents drift in a
 *   curving path, not a jittery zigzag.  wander_angle is the
 *   integrated heading; the per-tick noise rotates it by at most
 *   WANDER_TURN_MAX·dt radians.
 *
 * Why glyph + color_pair are FIXED at spawn (not per-tick)
 *   Visual identity stability — each agent is one specific
 *   "creature" the viewer can track across the screen.  Switching
 *   glyphs every tick would dissolve the swarm into mosaic noise.
 *
 * Members
 *   ── kinematic state ──────────────────────────────────────────
 *   pos          current position in PIXEL space (units = px).
 *                Toroidally bounced at world edges via bounce_pos.
 *   prev_pos     position at start of this tick — snapshotted by
 *                agent_step before integrating, for the renderer's
 *                alpha lerp.
 *   vel          velocity in PIXELS PER SECOND.  Capped at
 *                sp->max_speed by agent_step.
 *
 *   ── steering state ───────────────────────────────────────────
 *   wander_angle Slowly-integrating heading (radians) for wander
 *                noise.  Read+written by steer_wander; preserved
 *                across ticks.  Range unbounded (cos/sin handle
 *                the wrap).
 *
 *   ── assignment state ─────────────────────────────────────────
 *   slot_idx     Index into SwarmScene.slots[] — the target slot
 *                this agent is "claiming" while forming the
 *                current digit.  −1 = unassigned (more agents than
 *                slots in this digit), agent wanders freely.
 *
 *   ── visual state (fixed at spawn) ────────────────────────────
 *   glyph        ASCII character from AGENT_GLYPHS[i % N_GLYPHS].
 *   color_pair   ncurses pair number 1..N_COLORS; chosen as
 *                (id % N_COLORS) + 1 at spawn.
 *
 * Invariants
 *   0 ≤ pos.x < world.width AND 0 ≤ pos.y < world.height after every
 *   agent_step+bounce_pos pair.
 *   |vel| ≤ sp->max_speed after agent_step.
 *   slot_idx ∈ [0, scene.n_slots) OR == −1.
 *   1 ≤ color_pair ≤ N_COLORS.
 *
 * References
 *   [1] Reynolds 1987 — three-rule flocking model; FLOCK strategy
 *       uses all three rules; other strategies use a subset.
 *   [2] Reynolds 1999 — every steering primitive in §6 returns
 *       a force; agent_step accumulates the weighted sum.
 *   [8] Fiedler 2014 — the prev_pos / alpha-lerp pattern.
 */
typedef struct {
    /* kinematic state */
    Vec2  pos;
    Vec2  prev_pos;
    Vec2  vel;

    /* steering state */
    float wander_angle;

    /* assignment state */
    int   slot_idx;

    /* visual state (fixed at spawn) */
    char  glyph;
    int   color_pair;
} Agent;

/*
 * Slot — one target point in the digit's pixel-space point cloud.
 *
 * Intent
 *   When the user presses 0..9, §8 digit_load walks the 5×7 bitmap
 *   template for that digit and emits one Slot per filled pixel
 *   (each subdivided into SLOT_GRID² sub-slots for finer texture).
 *   §8 assign_slots then performs a GREEDY nearest-slot match
 *   (Kuhn [6] approximation) — each agent picks the closest free
 *   slot.  Strategies in §7 then steer agents toward their
 *   slot_idx, producing the digit silhouette.
 *
 * Why the occupied flag (instead of letting agents fight over slots)
 *   Without it, two agents might both pick the same nearest slot
 *   and oscillate between them.  Marking a slot occupied when
 *   assigned makes the greedy match O(N·S) with no contention —
 *   suboptimal vs the Hungarian algorithm [6] but cheap and
 *   visually indistinguishable for N=160.
 *
 * Members
 *   pos        Pixel-space position of this target point.  Derived
 *              by digit_load from the bitmap '#' coordinate, scaled
 *              by DIGIT_CELL_W × DIGIT_CELL_H and centred on screen.
 *   occupied   true while assigned to an agent (set by assign_slots).
 *              Reset to false on every digit change.
 *
 * Invariants
 *   pos.x ∈ [0, world.width), pos.y ∈ [0, world.height) (clamped by
 *   digit_load's centre + scale math).
 *
 * References
 *   [6] Kuhn 1955 — optimal assignment; this file's greedy
 *       nearest-slot match is the cheap O(N·S) approximation.
 *   [7] Knuth, TeX/METAFONT — bitmap-font lineage for the 5×7
 *       digit templates whose '#' pixels become Slot positions.
 */
typedef struct {
    Vec2 pos;
    bool occupied;
} Slot;

/* Glyph pool — visually distinct characters cycling through the swarm */
static const char AGENT_GLYPHS[] = "*@+#oO0x~=";

/*
 * agent_spawn — randomise one agent's position within the world.
 * glyph and color_pair are fixed at spawn and never change.
 */
static void agent_spawn(Agent *a, int id, float ww, float wh)
{
    a->pos          = v2(randf() * ww, randf() * wh);
    a->prev_pos     = a->pos;
    a->vel          = v2(0, 0);
    a->wander_angle = randf() * 6.2832f;   /* random start direction */
    a->slot_idx     = -1;
    a->glyph        = AGENT_GLYPHS[id % (int)(sizeof(AGENT_GLYPHS) - 1)];
    a->color_pair   = (id % N_COLORS) + 1;
}

/*
 * agent_step — Euler integration shared by every strategy.
 *
 *   vel     += force × dt  (then clamped to max_speed)
 *   prev_pos = pos
 *   pos     += vel × dt
 */
static void agent_step(Agent *a, Vec2 force, float max_speed, float dt)
{
    a->vel      = v2clamp_len(v2add(a->vel, v2scale(force, dt)), max_speed);
    a->prev_pos = a->pos;
    a->pos      = v2add(a->pos, v2scale(a->vel, dt));
}

/*
 * agent_commit_force — integrate the agent + bounce off world walls.
 *
 * Every strategy ends with this two-line sequence; centralising it
 * (a) drops the boilerplate from each strategy's body, and (b) means
 * any future change (e.g. wrapping vs bouncing) is one edit, not ten.
 *
 *   agent_step  (a, force, sp->max_speed, dt)   — vel ← vel+force·dt
 *   bounce_pos  (&a->pos, &a->vel, world_w, h)  — reflect at edges
 */
static void agent_commit_force(Agent *a, Vec2 force,
                                const StrategyParams *sp,
                                float dt, float ww, float wh)
{
    agent_step(a, force, sp->max_speed, dt);
    bounce_pos(&a->pos, &a->vel, ww, wh);
}

/* ===================================================================== */
/* §6  steering forces                                                  */
/* ===================================================================== */

/*
 * steer_seek — force toward (target) at (speed).
 *
 *   desired = normalise(target − pos) × speed
 *   force   = desired − vel
 *
 * Subtracting current velocity means when the agent already moves at
 * full desired speed toward target, force → 0 (no over-correction).
 */
static Vec2 steer_seek(Vec2 pos, Vec2 vel, Vec2 target, float speed)
{
    Vec2 desired = v2scale(v2norm(v2sub(target, pos)), speed);
    return v2sub(desired, vel);
}

/*
 * steer_arrive — seek with deceleration inside slow_radius.
 *
 *   desired_speed = max_speed × min(1, dist / slow_radius)
 *
 * Agents smoothly decelerate as they approach the target, preventing
 * the oscillation that a pure seek produces near the goal.
 */
static Vec2 steer_arrive(Vec2 pos, Vec2 vel, Vec2 target,
                          float max_speed, float slow_radius)
{
    Vec2  to_tgt       = v2sub(target, pos);
    float dist         = v2len(to_tgt);
    float desired_spd  = (dist < slow_radius)
                         ? max_speed * (dist / slow_radius)
                         : max_speed;
    Vec2  desired      = v2scale(v2norm(to_tgt), desired_spd);
    return v2sub(desired, vel);
}

/*
 * steer_separate — push away from agents within sep_radius.
 *
 * Repulsion strength = (sep_radius − dist) / sep_radius ∈ (0,1].
 *
 * IMPORTANT: the base force magnitude is a fixed 60 px/s, NOT
 * arrive_speed.  Tying it to arrive_speed made separation scale wildly
 * between strategies (RUSH arrive=180 → sep force 180× per neighbour,
 * completely overpowering the slot force and causing scattering).
 * A fixed base keeps separation consistent across all strategies so the
 * only tuning knob needed is sep_weight.
 */
#define SEP_BASE_FORCE  60.0f   /* fixed repulsion magnitude (px/s) */

static Vec2 steer_separate(const Agent *agents, int n_agents, int self,
                            const StrategyParams *sp)
{
    Vec2 force = v2(0, 0);
    for (int i = 0; i < n_agents; i++) {
        if (i == self) continue;
        Vec2  away = v2sub(agents[self].pos, agents[i].pos);
        float d    = v2len(away);
        if (d < sp->sep_radius && d > 0.001f) {
            float strength = (sp->sep_radius - d) / sp->sep_radius;
            force = v2add(force,
                          v2scale(v2norm(away), strength * SEP_BASE_FORCE));
        }
    }
    return force;
}

/*
 * steer_wander — smoothly-rotating random force.
 *
 * The agent's wander_angle rotates by a bounded random amount each tick.
 * The resulting force points in that direction at magnitude 'strength'.
 * This gives organic wandering without sudden direction changes.
 */
static Vec2 steer_wander(Agent *a, float strength, float dt)
{
    a->wander_angle += (randf() - 0.5f) * WANDER_TURN_MAX * dt;
    return v2scale(v2(cosf(a->wander_angle), sinf(a->wander_angle)), strength);
}

/*
 * steer_cohesion — seek toward the average position of nearby agents.
 *
 * Creates "group clustering": the local neighbourhood stays together.
 * Used only by FLOCK.
 */
static Vec2 steer_cohesion(const Agent *agents, int n_agents, int self,
                             const StrategyParams *sp, float speed)
{
    Vec2 sum = v2(0, 0);
    int  n   = 0;
    for (int i = 0; i < n_agents; i++) {
        if (i == self) continue;
        if (v2len(v2sub(agents[i].pos, agents[self].pos)) > sp->neighbor_radius)
            continue;
        sum = v2add(sum, agents[i].pos);
        n++;
    }
    if (n == 0) return v2(0, 0);
    Vec2 local_centre = v2scale(sum, 1.0f / n);
    return steer_seek(agents[self].pos, agents[self].vel, local_centre, speed);
}

/*
 * steer_align — match the average velocity of nearby agents.
 *
 * Creates "group alignment": agents fly in the same direction as their
 * neighbours.  Used only by FLOCK.
 */
static Vec2 steer_align(const Agent *agents, int n_agents, int self,
                          const StrategyParams *sp)
{
    Vec2 sum = v2(0, 0);
    int  n   = 0;
    for (int i = 0; i < n_agents; i++) {
        if (i == self) continue;
        if (v2len(v2sub(agents[i].pos, agents[self].pos)) > sp->neighbor_radius)
            continue;
        sum = v2add(sum, agents[i].vel);
        n++;
    }
    if (n == 0) return v2(0, 0);
    Vec2 avg_vel = v2scale(sum, 1.0f / n);
    return v2sub(avg_vel, agents[self].vel);   /* steer toward average */
}

/*
 * steer_spring — Hooke's law: spring pull toward target, damped by velocity.
 *
 *   force = k × (target − pos) − damping × vel
 *
 * With damping ratio ζ = damping / (2·√k):
 *   ζ < 1 → underdamped (oscillates, then settles)  ← visually interesting
 *   ζ = 1 → critically damped (fastest non-oscillating settle)
 *   ζ > 1 → overdamped (slow exponential approach, no overshoot)
 *
 * SPRING_K=3.5, SPRING_DAMP=2.0 → ζ ≈ 0.53 → underdamped.
 */
static Vec2 steer_spring(Vec2 pos, Vec2 vel, Vec2 target, float k, float damping)
{
    Vec2 spring_pull = v2scale(v2sub(target, pos), k);
    Vec2 damp_drag   = v2scale(vel, damping);
    return v2sub(spring_pull, damp_drag);
}

/* ===================================================================== */
/* §7  strategy tick functions                                          */
/* ===================================================================== */

/*
 * Each strategy function applies a different mixture of forces and then
 * commits the result via agent_commit_force.  All tuneables come through
 * the `const StrategyParams *sp` parameter — no file-scope state.
 *
 * Common pattern (Reynolds [2] weighted-sum steering):
 *   1. slot_force  = agent_arrive_at_slot_if_assigned(...)
 *   2. sep_force   = steer_separate(...)
 *   3. extra_force = (strategy-specific addition)
 *   4. force       = Σ weighted_forces
 *   5. agent_commit_force(a, force, sp, dt, ww, wh)
 *
 * agent_arrive_at_slot_if_assigned encapsulates step 1; agent_commit_force
 * encapsulates step 5; the steering helpers in §6 cover step 2.  The
 * BODY of each strategy is just steps 3 and 4 — the part that makes
 * the strategy distinctive.
 */

/*
 * agent_arrive_at_slot_if_assigned — return the slot-arrive steering
 * force (already scaled by sp->slot_weight) or zero if the agent has
 * no slot assigned.
 *
 * Used by 9 of 10 strategies (every one except SPRING, which uses a
 * harmonic-oscillator force instead).  Hoists the `if (slot_idx >= 0)`
 * guard + arrive call + scale-by-weight pattern out of each strategy.
 */
static Vec2 agent_arrive_at_slot_if_assigned(const Agent *a,
                                              const Slot *slots,
                                              const StrategyParams *sp)
{
    if (a->slot_idx < 0) return v2(0, 0);

    Vec2 arrive = steer_arrive(a->pos, a->vel, slots[a->slot_idx].pos,
                                sp->arrive_speed, sp->slow_radius);
    return v2scale(arrive, sp->slot_weight);
}

/* ── §7.1  DRIFT ──────────────────────────────────────────────────── */
/*
 * Agents wander with Brownian noise and are pulled toward their slot.
 * Produces a dreamy, meandering convergence.
 *
 * Wander fades to zero inside WANDER_FADE_DIST px of the slot.
 * Without fading, wander would keep kicking agents out of their slot
 * after they arrive, preventing the digit from forming clearly.
 *
 * force = wander × fade_scale  +  slot_arrive × slot_weight  +  sep × sep_weight
 */
#define WANDER_FADE_DIST  55.0f   /* px: wander is zero below this distance to slot */

/*
 * wander_fade_scale_for_slot — linear ramp ∈ [0, 1] that scales wander
 * strength based on how far the agent is from its target slot:
 *
 *   d >= WANDER_FADE_DIST → 1.0  (full wander, far from slot)
 *   d < WANDER_FADE_DIST  → d / WANDER_FADE_DIST  (fades to 0 at slot)
 *   no slot               → 1.0  (free wander)
 *
 * Used by DRIFT and FLOCK so agents stop wandering after arriving at
 * their slot — otherwise wander would keep kicking them out and the
 * digit shape never crystallises.
 */
static float wander_fade_scale_for_slot(const Agent *a, const Slot *slots)
{
    if (a->slot_idx < 0) return 1.0f;
    float d = v2len(v2sub(slots[a->slot_idx].pos, a->pos));
    return (d >= WANDER_FADE_DIST) ? 1.0f : d / WANDER_FADE_DIST;
}

static void strategy_drift(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                            const Slot *slots, float ww, float wh, float dt)
{
    Agent *a = &agents[self];

    /* Component forces (Reynolds [2] weighted sum) */
    Vec2  sep_force    = steer_separate(agents, n_agents, self, sp);
    float wander_scale = wander_fade_scale_for_slot(a, slots);
    Vec2  wander_force = steer_wander(a, sp->wander_strength * wander_scale, dt);
    Vec2  slot_force   = agent_arrive_at_slot_if_assigned(a, slots, sp);

    /* force = wander + slot_arrive + sep·sep_weight */
    Vec2 force = v2add(v2add(wander_force, slot_force),
                       v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.2  RUSH ───────────────────────────────────────────────────── */
/*
 * Agents sprint directly to their slot using arrive (smooth stop on
 * arrival) with no wander.  Digit snaps into place briskly.
 *
 * force = slot_arrive × slot_weight  +  sep × sep_weight
 */
#define RUSH_NO_SLOT_DAMPING  0.90f   /* per-tick velocity multiplier when unassigned */

static void strategy_rush(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                           const Slot *slots, float ww, float wh, float dt)
{
    Agent *a = &agents[self];

    /* Unassigned agents glide to a halt (no force will push them) */
    if (a->slot_idx < 0) a->vel = v2scale(a->vel, RUSH_NO_SLOT_DAMPING);

    Vec2 sep_force  = steer_separate(agents, n_agents, self, sp);
    Vec2 slot_force = agent_arrive_at_slot_if_assigned(a, slots, sp);

    Vec2 force = v2add(slot_force, v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.3  FLOW ───────────────────────────────────────────────────── */
/*
 * A rightward current pushes agents across the screen.  When an agent
 * is within FLOW_X_THRESH px of its slot's x-coordinate the current
 * gives way and slot-arrive takes over.  Agents "paint" the digit
 * from left to right.
 *
 * force = (if far left of slot)  rightward_stream + weak slot pull
 *         (if near slot x)       slot_arrive + wander
 */
/* FLOW per-phase attenuations.  When the agent is FAR LEFT of its
 * slot column, the slot-arrive is reduced so the rightward current
 * dominates; when CLOSE, wander is reduced so the agent settles into
 * its slot rather than continuing to drift. */
#define FLOW_FAR_ARRIVE_SCALE   0.3f   /* slot pull while still in the current */
#define FLOW_NEAR_WANDER_SCALE  0.4f   /* wander once aligned with slot column */
#define FLOW_NO_SLOT_BIAS_SCALE 0.6f   /* current strength for unassigned agents */

static void strategy_flow(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                           const Slot *slots, float ww, float wh, float dt)
{
    Agent *a = &agents[self];
    Vec2   sep_force = steer_separate(agents, n_agents, self, sp);
    Vec2   task_force;

    if (a->slot_idx < 0) {
        /* (a) Unassigned: ride the current at reduced strength + wander */
        Vec2 stream       = v2(FLOW_BIAS * FLOW_NO_SLOT_BIAS_SCALE, 0.0f);
        Vec2 wander_force = steer_wander(a, sp->wander_strength, dt);
        task_force        = v2add(stream, wander_force);

    } else {
        Vec2  slot_pos = slots[a->slot_idx].pos;
        float dx_to_slot = slot_pos.x - a->pos.x;
        bool  still_in_current = dx_to_slot > FLOW_X_THRESH;

        if (still_in_current) {
            /* (b) Far left of slot: stream rightward + faint slot pull */
            Vec2 stream      = v2(FLOW_BIAS, 0.0f);
            Vec2 weak_arrive = steer_arrive(a->pos, a->vel, slot_pos,
                                             sp->arrive_speed * FLOW_FAR_ARRIVE_SCALE,
                                             sp->slow_radius);
            task_force       = v2add(stream, weak_arrive);
        } else {
            /* (c) Aligned with slot column: full slot-arrive + soft wander */
            Vec2 slot_force   = agent_arrive_at_slot_if_assigned(a, slots, sp);
            Vec2 wander_force = steer_wander(a, sp->wander_strength * FLOW_NEAR_WANDER_SCALE, dt);
            task_force        = v2add(slot_force, wander_force);
        }
    }

    Vec2 force = v2add(task_force, v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.4  ORBIT ──────────────────────────────────────────────────── */
/*
 * Each agent orbits the digit centroid with a tangential force while
 * simultaneously seeking its assigned slot.  The two forces compete:
 * the orbit keeps agents spinning, the slot-seek pulls them inward.
 * Result: a galaxy-like spiral collapse into the digit shape.
 *
 * tangent = rotate(pos − centroid, +90°) → clockwise orbit direction
 * force   = tangent × ORBIT_STRENGTH  +  slot_arrive × slot_weight
 */
static void strategy_orbit(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                            const Slot *slots, float ww, float wh, float dt,
                            Vec2 digit_centroid)
{
    Agent *a = &agents[self];

    /* Compute tangential orbit direction: 90° CW rotation of the radial
     * (pos − centroid) vector.  The +90° rotation of (x, y) is (−y, x). */
    Vec2  radial         = v2sub(a->pos, digit_centroid);
    Vec2  tangent_dir    = v2norm(v2(-radial.y, radial.x));
    Vec2  orbit_force    = v2scale(tangent_dir, ORBIT_STRENGTH);

    Vec2  sep_force      = steer_separate(agents, n_agents, self, sp);
    Vec2  slot_force     = agent_arrive_at_slot_if_assigned(a, slots, sp);

    /* force = orbit + slot_arrive + sep·sep_weight
     *    orbit keeps agent spinning, slot_arrive pulls inward → spiral collapse */
    Vec2 force = v2add(v2add(orbit_force, slot_force),
                       v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.5  FLOCK ──────────────────────────────────────────────────── */
/*
 * Full Reynolds boids: cohesion + alignment + separation + weak slot seek.
 * The flock moves as one organism; the slot attraction gradually morphs
 * it into the digit shape.
 *
 * force = cohesion × cohesion_weight
 *       + alignment × align_weight
 *       + sep × sep_weight
 *       + slot_arrive × slot_weight   (weak: lets group dynamics dominate)
 */
static void strategy_flock(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                            const Slot *slots, float ww, float wh, float dt)
{
    Agent *a = &agents[self];

    /* Reynolds [1] three-rule boid forces */
    Vec2  sep_force = steer_separate(agents, n_agents, self, sp);
    Vec2  coh_force = steer_cohesion(agents, n_agents, self, sp, sp->arrive_speed);
    Vec2  aln_force = steer_align   (agents, n_agents, self, sp);

    /* Wander fades to 0 near slot so agents settle instead of drifting */
    float wander_scale = wander_fade_scale_for_slot(a, slots);
    Vec2  wander_force = steer_wander(a, sp->wander_strength * wander_scale, dt);

    Vec2  slot_force   = agent_arrive_at_slot_if_assigned(a, slots, sp);

    /* force = w_coh·coh + w_aln·aln + w_sep·sep + slot + wander */
    Vec2 group_force = v2add(v2scale(coh_force, sp->cohesion_weight),
                             v2scale(aln_force, sp->align_weight));
    Vec2 personal_force = v2add(v2add(v2scale(sep_force, sp->sep_weight),
                                       slot_force),
                                wander_force);
    Vec2 force = v2add(group_force, personal_force);

    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.6  PULSE ──────────────────────────────────────────────────── */
/*
 * Agents arrive at their slot, then receive a periodic outward push
 * based on a sine wave.  They spring back via the slot-arrive force.
 * The net effect: the formed digit "breathes" in and out.
 *
 * oscillating_target = slot_pos + away_from_slot × sin(2π·PULSE_FREQ·t) × AMP
 *
 * When sin > 0 the oscillating target is pushed outward.
 * When sin < 0 it is pulled inward (helping the slot-arrive snap back).
 */
#define PULSE_NO_SLOT_DAMPING  0.94f   /* per-tick velocity decay when unassigned */
#define PULSE_DEGENERATE_EPS   0.001f  /* min |centroid→slot| before fallback dir */

static void strategy_pulse(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                            const Slot *slots, int n_slots, float ww, float wh,
                            float dt, float sim_time, Vec2 digit_centroid)
{
    (void)n_slots;
    Agent *a = &agents[self];
    Vec2   sep_force = steer_separate(agents, n_agents, self, sp);
    Vec2   task_force;

    if (a->slot_idx < 0) {
        /* (a) Unassigned: friction-only; agent drifts to rest */
        a->vel     = v2scale(a->vel, PULSE_NO_SLOT_DAMPING);
        task_force = v2(0, 0);
    } else {
        Vec2 slot_pos = slots[a->slot_idx].pos;

        /* (b) Push direction = centroid → slot (NOT agent → slot).
         *     Using the centroid-relative direction keeps it stable even
         *     when the agent sits exactly on its slot — where the
         *     agent→slot vector would be a degenerate zero. */
        Vec2 centroid_to_slot = v2sub(slot_pos, digit_centroid);
        Vec2 push_dir         = v2norm(centroid_to_slot);
        if (v2len(push_dir) < PULSE_DEGENERATE_EPS) push_dir = v2(1.0f, 0.0f);

        /* (c) Oscillating target: slot ± amplitude·sin(2π·f·t).
         *     sin > 0 pushes outward, sin < 0 inward, average = slot. */
        float wave_phase     = sinf(TWO_PI * PULSE_FREQ * sim_time);
        Vec2  pulse_offset   = v2scale(push_dir, wave_phase * PULSE_AMPLITUDE);
        Vec2  oscillating_tgt = v2add(slot_pos, pulse_offset);

        /* (d) Steer to the oscillating target (not the raw slot).
         *     slot_weight must exceed PULSE_AMPLITUDE so agents track
         *     the target instead of drifting away. */
        Vec2 arrive_force = steer_arrive(a->pos, a->vel, oscillating_tgt,
                                          sp->arrive_speed, sp->slow_radius);
        task_force = v2scale(arrive_force, sp->slot_weight);
    }

    Vec2 force = v2add(task_force, v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.7  VORTEX ─────────────────────────────────────────────────── */
/*
 * Each agent has its OWN private whirlpool centred on its assigned slot.
 * A tangential force (perpendicular to the radius from the slot) makes
 * the agent spiral inward as slot-arrive simultaneously pulls it closer.
 *
 * Unlike ORBIT (which orbits the digit centroid), each agent here orbits
 * its own slot independently — multiple small whirlpools forming the digit.
 *
 * The vortex strength is scaled by (dist / VORTEX_FADE_DIST) so it fades
 * smoothly to zero when the agent reaches its slot.  Without fading, agents
 * would perpetually orbit the slot at small radius instead of settling.
 *
 * tangent  = rotate_90(pos − slot)      ← CW perpendicular to radius
 * vortex_f = tangent × VORTEX_STRENGTH × min(dist/VORTEX_FADE_DIST, 1)
 * force    = vortex_f + slot_arrive × slot_weight + sep × sep_weight
 */
static void strategy_vortex(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                              const Slot *slots, float ww, float wh, float dt)
{
    Agent *a = &agents[self];
    Vec2   sep_force    = steer_separate(agents, n_agents, self, sp);
    Vec2   slot_force   = agent_arrive_at_slot_if_assigned(a, slots, sp);
    Vec2   vortex_force = v2(0, 0);

    if (a->slot_idx >= 0) {
        Vec2  slot_pos       = slots[a->slot_idx].pos;
        Vec2  radial         = v2sub(a->pos, slot_pos);
        float radius         = v2len(radial);

        /* Tangent: 90° CCW rotation of radial → (−y, x) */
        Vec2  tangent_dir    = v2norm(v2(-radial.y, radial.x));

        /* Fade scale: linear ramp 0 → 1 over [0, VORTEX_FADE_DIST].
         * Without this, agents would orbit forever at small radius
         * instead of settling on the slot. */
        float fade_scale     = fminf(radius / VORTEX_FADE_DIST, 1.0f);
        vortex_force         = v2scale(tangent_dir, VORTEX_STRENGTH * fade_scale);
    }

    Vec2 force = v2add(v2add(vortex_force, slot_force),
                       v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.8  GRAVITY ────────────────────────────────────────────────── */
/*
 * A constant downward "gravity" force biases every agent's trajectory.
 * The slot-arrive force still dominates (slot_force >> GRAVITY_PULL),
 * so agents do reach their slots — but they approach from above, overshoot
 * slightly, and bounce back.  The digit "rains" into place.
 *
 * force = gravity_down + slot_arrive × slot_weight + sep × sep_weight
 */
static void strategy_gravity(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                               const Slot *slots, float ww, float wh, float dt)
{
    Agent *a = &agents[self];
    Vec2   sep_force     = steer_separate(agents, n_agents, self, sp);
    Vec2   slot_force    = agent_arrive_at_slot_if_assigned(a, slots, sp);
    Vec2   gravity_force = v2(0.0f, GRAVITY_PULL);     /* +y = screen-downward */

    Vec2 force = v2add(v2add(gravity_force, slot_force),
                       v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.9  SPRING ─────────────────────────────────────────────────── */
/*
 * Hooke's law spring force replaces the arrive steering.
 *
 *   force = SPRING_K × (slot − pos) − SPRING_DAMP × vel
 *
 * With SPRING_K=3.5 and SPRING_DAMP=2.0, the damping ratio is:
 *   ζ = SPRING_DAMP / (2·√SPRING_K) ≈ 0.53  →  underdamped
 *
 * Underdamped means agents oscillate 1–2 cycles around the slot before
 * settling.  They "bounce" into position like rubber balls.
 *
 * force = spring(slot) + sep × sep_weight
 */
#define SPRING_NO_SLOT_DAMPING 0.92f   /* per-tick velocity decay when unassigned */

static void strategy_spring(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                              const Slot *slots, float ww, float wh, float dt)
{
    Agent *a = &agents[self];

    /* Unassigned agents have no spring to anchor them — damp to rest. */
    if (a->slot_idx < 0) a->vel = v2scale(a->vel, SPRING_NO_SLOT_DAMPING);

    Vec2 sep_force    = steer_separate(agents, n_agents, self, sp);

    /* spring_force = K·(slot − pos) − DAMP·vel.  Strogatz [5] underdamped
     * (ζ ≈ 0.53) so agents oscillate 1–2 cycles before settling.  Unlike
     * the other strategies, SPRING does NOT scale by sp->slot_weight —
     * the K/DAMP constants ARE the tuning knobs. */
    Vec2 spring_force = (a->slot_idx >= 0)
        ? steer_spring(a->pos, a->vel, slots[a->slot_idx].pos, SPRING_K, SPRING_DAMP)
        : v2(0, 0);

    Vec2 force = v2add(spring_force, v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/* ── §7.10  WAVE ──────────────────────────────────────────────────── */
/*
 * Agents advance toward their slot while oscillating laterally (side to side)
 * with a sinusoidal force perpendicular to the approach direction.
 *
 * The lateral amplitude is:
 *   amp = min(dist × 0.5, WAVE_AMPLITUDE) × (dist / WAVE_FADE_DIST)
 *
 * Two fade mechanisms work together:
 *   • dist × 0.5        : amplitude grows from zero so agents start straight
 *   • dist / WAVE_FADE_DIST : fades to zero near the slot so agents settle
 *
 * This produces the "snake" appearance: wide wiggling at mid-range,
 * converging smoothly at the slot.
 *
 * force = slot_arrive × slot_weight + perp × amp × sin(2π·WAVE_FREQ·t)
 *       + sep × sep_weight
 */
#define WAVE_RAMP_SLOPE  0.5f   /* px-of-amplitude per px-of-distance (ramp-up phase) */

static void strategy_wave(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                            const Slot *slots, float ww, float wh,
                            float dt, float sim_time)
{
    Agent *a = &agents[self];
    Vec2   sep_force  = steer_separate(agents, n_agents, self, sp);
    Vec2   slot_force = agent_arrive_at_slot_if_assigned(a, slots, sp);
    Vec2   wave_force = v2(0, 0);

    if (a->slot_idx >= 0) {
        /* Compute approach axis and its perpendicular (lateral axis) */
        Vec2  to_slot         = v2sub(slots[a->slot_idx].pos, a->pos);
        float dist_to_slot    = v2len(to_slot);
        Vec2  approach_dir    = v2norm(to_slot);
        Vec2  lateral_dir     = v2(-approach_dir.y, approach_dir.x);

        /* Lateral amplitude has TWO fade envelopes multiplied together:
         *   ramp_amplitude : grows linearly with distance, capped at
         *                    WAVE_AMPLITUDE.  Keeps the wave subtle when
         *                    the agent is close to the slot.
         *   fade_envelope  : falls to 0 inside WAVE_FADE_DIST so the
         *                    final approach is straight (settling phase).
         * Product → zero amplitude near slot AND when agent is on top of
         * slot — both conditions force smooth settling. */
        float ramp_amplitude  = fminf(dist_to_slot * WAVE_RAMP_SLOPE, WAVE_AMPLITUDE);
        float fade_envelope   = fminf(dist_to_slot / WAVE_FADE_DIST, 1.0f);
        float lateral_amplitude = ramp_amplitude * fade_envelope;

        float wave_phase      = sinf(TWO_PI * WAVE_FREQ * sim_time);
        wave_force            = v2scale(lateral_dir, lateral_amplitude * wave_phase);
    }

    Vec2 force = v2add(v2add(slot_force, wave_force),
                       v2scale(sep_force, sp->sep_weight));
    agent_commit_force(a, force, sp, dt, ww, wh);
}

/*
 * agent_tick — dispatch to the active strategy for one agent.
 *
 * digit_centroid and sim_time are passed through because ORBIT, PULSE,
 * and WAVE need them; other strategies ignore them.
 */
static void agent_tick(Agent *agents, int n_agents, int self, const StrategyParams *sp,
                       const Slot *slots, int n_slots,
                       float ww, float wh, float dt,
                       int strategy, Vec2 digit_centroid, float sim_time)
{
    switch (strategy) {
    case 0: strategy_drift   (agents, n_agents, self, sp, slots,          ww, wh, dt); break;
    case 1: strategy_rush    (agents, n_agents, self, sp, slots,          ww, wh, dt); break;
    case 2: strategy_flow    (agents, n_agents, self, sp, slots,          ww, wh, dt); break;
    case 3: strategy_orbit   (agents, n_agents, self, sp, slots,          ww, wh, dt, digit_centroid); break;
    case 4: strategy_flock   (agents, n_agents, self, sp, slots,          ww, wh, dt); break;
    case 5: strategy_pulse   (agents, n_agents, self, sp, slots, n_slots, ww, wh, dt, sim_time, digit_centroid); break;
    case 6: strategy_vortex  (agents, n_agents, self, sp, slots,          ww, wh, dt); break;
    case 7: strategy_gravity (agents, n_agents, self, sp, slots,          ww, wh, dt); break;
    case 8: strategy_spring  (agents, n_agents, self, sp, slots,          ww, wh, dt); break;
    case 9: strategy_wave    (agents, n_agents, self, sp, slots,          ww, wh, dt, sim_time); break;
    }
}

/* ===================================================================== */
/* §8  digit library                                                    */
/* ===================================================================== */

/*
 * DigitLayout — pre-computed pixel-space geometry of a digit's bounding
 * box.  Computed ONCE per digit_load call and passed to the inner loops
 * so they don't repeat the divisions on every '#' pixel.
 *
 *   origin       top-left corner of the bitmap's bounding box (px)
 *   stride_cells distance between consecutive sub-slots, in TERMINAL CELLS
 *                (chosen so first sub-slot is at offset 0 and last is at
 *                the bitmap-cell's far edge).  Integer division gives
 *                clean alignment when (DIM − 1) divides (SLOT_GRID − 1).
 */
typedef struct {
    Vec2 origin;
    int  stride_x_cells;
    int  stride_y_cells;
} DigitLayout;

/* digit_layout_centred — compute the centred bounding-box origin and the
 * stride between sub-slots, given the current terminal dimensions. */
static DigitLayout digit_layout_centred(int cols, int rows)
{
    const float bitmap_cell_px_w = (float)(DIGIT_CELL_W * CELL_W);
    const float bitmap_cell_px_h = (float)(DIGIT_CELL_H * CELL_H);
    const float bbox_px_w        = (float)DIGIT_NCOLS * bitmap_cell_px_w;
    const float bbox_px_h        = (float)DIGIT_NROWS * bitmap_cell_px_h;

    return (DigitLayout){
        .origin         = v2((pw(cols) - bbox_px_w) * 0.5f,
                             (ph(rows) - bbox_px_h) * 0.5f),
        .stride_x_cells = (DIGIT_CELL_W - 1) / (SLOT_GRID - 1),  /* = 2 */
        .stride_y_cells = (DIGIT_CELL_H - 1) / (SLOT_GRID - 1),  /* = 1 */
    };
}

/*
 * emit_subslot_grid — place SLOT_GRID × SLOT_GRID sub-slots inside one
 * bitmap cell (the cell whose top-left in terminal-cell coordinates is
 * (cell_col0, cell_row0)).  Writes them into slots[*n..] and bumps *n.
 *
 *   px = origin.x + (cell_col0 + sc · stride_x_cells) · CELL_W
 *   py = origin.y + (cell_row0 + sr · stride_y_cells) · CELL_H
 *
 *   stride_x_cells = (5 − 1) / (3 − 1) = 2  → sub-cols 0, 2, 4 in a 5-wide cell
 *   stride_y_cells = (3 − 1) / (3 − 1) = 1  → sub-rows 0, 1, 2 in a 3-tall cell
 *
 * The "no rounding leak" trick
 *   Sub-slot coordinates are CELL-ALIGNED (sc · stride_x_cells in INTEGER
 *   terminal cells, no half-cell offset) so the later px → cell rounding
 *   in scene_draw lands EXACTLY on the intended cell.  At fractional
 *   positions like (sc + 0.5) / SLOT_GRID the third sub-slot lands at
 *   py = 26.7 which the round-half-up `floor(py/16 + 0.5)` maps to row 2
 *   — the NEXT cell's row 0 — so the third sub-row of bitmap row 0
 *   would render on top of bitmap row 1.  That's the "6 looks like 5"
 *   bug this alignment prevents.
 */
static void emit_subslot_grid(Slot *slots, int *n, const DigitLayout *lay,
                              int cell_col0, int cell_row0)
{
    for (int sr = 0; sr < SLOT_GRID && *n < SLOTS_MAX; sr++) {
        for (int sc = 0; sc < SLOT_GRID && *n < SLOTS_MAX; sc++) {
            int sub_col = cell_col0 + sc * lay->stride_x_cells;
            int sub_row = cell_row0 + sr * lay->stride_y_cells;
            slots[*n].pos      = v2(lay->origin.x + (float)sub_col * (float)CELL_W,
                                     lay->origin.y + (float)sub_row * (float)CELL_H);
            slots[*n].occupied = false;
            (*n)++;
        }
    }
}

/*
 * digit_load — extract Slot positions from a digit's 5×7 bitmap.
 *
 *   (1) compute the centred bounding-box geometry (once)
 *   (2) for each '#' pixel in the bitmap, emit a SLOT_GRID × SLOT_GRID
 *       grid of cell-aligned sub-slots via emit_subslot_grid
 *
 * Returns the total slot count (SLOT_GRID² × bitmap-fill-count).
 * The two work-horse helpers — digit_layout_centred (geometry) and
 * emit_subslot_grid (slot emission) — keep this function a 12-line
 * coordinator instead of a 4-level-nested loop.
 */
static int digit_load(Slot *slots, int digit, int cols, int rows)
{
    /* (1) compute centred bounding-box geometry once */
    DigitLayout layout = digit_layout_centred(cols, rows);

    /* (2) walk the 5×7 bitmap; for every '#' pixel emit its sub-slot grid */
    int n = 0;
    for (int r = 0; r < DIGIT_NROWS && n < SLOTS_MAX; r++) {
        const char *row = DIGIT_BITMAPS[digit][r];
        for (int c = 0; row[c] && c < DIGIT_NCOLS; c++) {
            if (row[c] != '#') continue;

            /* Top-left of this '#' cell, in TERMINAL-CELL coordinates. */
            int cell_col0 = c * DIGIT_CELL_W;
            int cell_row0 = r * DIGIT_CELL_H;
            emit_subslot_grid(slots, &n, &layout, cell_col0, cell_row0);
        }
    }
    return n;
}

/*
 * digit_centroid — average position of all slot points.
 * Used by ORBIT and PULSE to find the centre to orbit/push around.
 */
static Vec2 digit_centroid(const Slot *slots, int n_slots)
{
    if (n_slots == 0) return v2(0, 0);
    Vec2 sum = v2(0, 0);
    for (int i = 0; i < n_slots; i++)
        sum = v2add(sum, slots[i].pos);
    return v2scale(sum, 1.0f / n_slots);
}

/*
 * assign_slots — greedy nearest-available slot assignment.
 *
 * For each agent (in index order), find the nearest slot not yet
 * taken by a previous agent and assign it.  Agents beyond n_slots
 * get slot_idx = -1 (they will wander freely).
 *
 * Complexity: O(N × S) — fine for N=25, S=17.
 *
 * Note: greedy assignment is not globally optimal (Hungarian algorithm
 * would be), but it converges quickly and looks natural.
 */
static void assign_slots(Agent *agents, int n_agents,
                         Slot *slots, int n_slots)
{
    /* Mark all slots as available */
    for (int s = 0; s < n_slots; s++) slots[s].occupied = false;

    for (int i = 0; i < n_agents; i++) {
        agents[i].slot_idx = -1;
        float best_d2 = FLT_MAX;
        int   best_s  = -1;
        for (int s = 0; s < n_slots; s++) {
            if (slots[s].occupied) continue;
            float d2 = v2len2(v2sub(slots[s].pos, agents[i].pos));
            if (d2 < best_d2) { best_d2 = d2; best_s = s; }
        }
        if (best_s >= 0) {
            agents[i].slot_idx    = best_s;
            slots[best_s].occupied = true;
        }
    }
}

/* ===================================================================== */
/* §9  scene                                                            */
/* ===================================================================== */

/*
 * World — pixel-space simulation extent (refreshed each tick / resize).
 *
 * Intent
 *   Bundles the pair of dimensions that EVERY steering function needs
 *   (for boundary bouncing, centroid clamping, wander limits).  Before
 *   this struct, every helper carried `float ww, float wh` as two
 *   separate parameters — 60+ call sites in this file.  Bundling makes
 *   `bounce_pos(pos, vel, &world)` instead of `bounce_pos(pos, vel,
 *   ww, wh)`.
 *
 * Members
 *   width    World width in PIXELS  (= cols × CELL_W).
 *   height   World height in PIXELS (= (rows − 1) × CELL_H, reserving
 *            the bottom row for the HUD).
 *
 * Invariants
 *   width > 0, height > 0.  Recomputed in scene_tick + scene_init from
 *   the live Screen.cols / Screen.rows so a resize propagates without
 *   any reconciliation logic.
 */
typedef struct {
    float width;
    float height;
} World;

/*
 * SimControls — user-facing playback knobs.
 *
 * Intent
 *   The input handler writes these; scene_tick reads them.  Bundling
 *   keeps the top-level Scene free for actual simulation state.
 *
 * Members
 *   paused        true → scene_tick early-returns; HUD shows "PAUSED".
 *                 Toggled by SPACE.
 *   auto_cycle    true → scene_tick advances to the next digit every
 *                 AUTO_CYCLE_S seconds.  Toggled by 'a'.
 *   cycle_timer   seconds elapsed since the last auto-advance (used
 *                 only when auto_cycle is true; reset on every manual
 *                 digit selection).
 *
 * Invariants
 *   cycle_timer ≥ 0; resets to 0 when it crosses AUTO_CYCLE_S.
 */
typedef struct {
    bool  paused;
    bool  auto_cycle;
    float cycle_timer;
} SimControls;

/*
 * Strategy — which of the N_STRATEGIES presets is active.
 *
 * Intent
 *   The strategy choice is TWO pieces of state that must stay in
 *   lockstep: an index (for cycling with n/p and for HUD display) and
 *   a pointer to the active StrategyParams (read by every strategy
 *   function in §7).  Bundling them prevents the historical bug of
 *   the index pointing one place while the params pointer points
 *   somewhere else after a key dispatch.
 *
 * Members
 *   index      Currently active preset in [0, N_STRATEGIES).  n/p
 *              cycles forward/backward (modular wrap).
 *   params     Pointer to &g_presets[index] — the const table of all
 *              steering coefficients (sep_weight, max_speed, …).
 *              Always re-set together with `index` via
 *              scene_set_strategy.
 *
 * Invariants
 *   0 ≤ index < N_STRATEGIES.
 *   params == &g_presets[index].
 */
typedef struct {
    int                          index;
    const StrategyParams        *params;
} Strategy;

/*
 * SwarmScene — owns ALL simulation state for one run.
 *
 * Layered ownership
 *
 *     SwarmScene
 *       ├── agents[N_AGENTS]   : Agent[]       ← particle pool (fixed-size)
 *       ├── slots[SLOTS_MAX]   : Slot[]        ← digit target slots
 *       ├── n_slots            : int           ← active prefix length
 *       ├── current_digit      : int           ← 0..9
 *       ├── strategy           : Strategy      ← index + params pointer
 *       ├── theme_idx          : int           ← colour palette index
 *       ├── sim_time           : float         ← drives PULSE / WAVE
 *       ├── world              : World         ← pixel-space extent
 *       └── sim                : SimControls   ← paused / auto_cycle / cycle_timer
 *
 *   Every persistent simulation value is reachable from one
 *   SwarmScene*.  No file-scope globals carry simulation state;
 *   g_presets[] is a const table, not state.
 *
 * Why this hierarchy
 *   • Replaceability: a future "two swarms side by side" demo would
 *     allocate two SwarmScene instances.
 *   • Reading aid: every helper takes `SwarmScene *`, so there's ONE
 *     place to look for "what state exists".
 */
typedef struct {
    /* agent + slot pools — first n_slots slots active per digit */
    Agent agents[N_AGENTS];
    Slot  slots[SLOTS_MAX];
    int   n_slots;

    /* active selection */
    int      current_digit;     /* 0..9                                 */
    Strategy strategy;          /* index + cached params pointer        */
    int      theme_idx;         /* 0..N_THEMES-1; t/T cycles            */

    /* simulation clock + world extent */
    float sim_time;             /* drives PULSE/WAVE oscillations       */
    World world;                /* pixel-space extent, refreshed/tick   */

    /* user-facing controls */
    SimControls sim;
} SwarmScene;

/* scene_scatter — randomise all agent positions (used on 'r' key and init) */
static void scene_scatter(SwarmScene *s)
{
    for (int i = 0; i < N_AGENTS; i++) {
        s->agents[i].pos      = v2(randf() * s->world.width, randf() * s->world.height);
        s->agents[i].prev_pos = s->agents[i].pos;
        s->agents[i].vel      = v2(0, 0);
    }
}

/* scene_set_digit — switch the active digit (0..9): rebuild slot
 * positions from the new bitmap, then greedy-assign agents to slots.
 * Called from scene_init, the 0..9 keys, and the auto-cycle timer. */
static void scene_set_digit(SwarmScene *s, int digit)
{
    s->current_digit = digit;
    s->n_slots = digit_load(s->slots, digit,
                             (int)(s->world.width / CELL_W),
                             (int)(s->world.height / CELL_H));
    assign_slots(s->agents, N_AGENTS, s->slots, s->n_slots);
}

/*
 * scene_set_strategy — switch the active steering preset.  Keeps
 * Strategy.index and Strategy.params in lockstep so a key-press takes
 * effect on the very next tick.  Wraps modularly over N_STRATEGIES.
 */
static void scene_set_strategy(SwarmScene *s, int new_index)
{
    new_index = ((new_index % N_STRATEGIES) + N_STRATEGIES) % N_STRATEGIES;
    s->strategy.index  = new_index;
    s->strategy.params = &g_presets[new_index];
}

/*
 * scene_init — bring a SwarmScene to a fresh start.
 *
 *   (1) zero everything, then set the World extent from cols/rows
 *   (2) point Strategy at preset 0 (DRIFT) — index+params in lockstep
 *   (3) spawn N_AGENTS at random positions with assigned glyph + colour
 *   (4) load digit 0's slots and assign agents to them
 */
static void scene_init(SwarmScene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->world.width  = pw(cols);
    s->world.height = ph(rows);

    scene_set_strategy(s, 0);

    for (int i = 0; i < N_AGENTS; i++)
        agent_spawn(&s->agents[i], i, s->world.width, s->world.height);

    scene_set_digit(s, 0);
}

static void scene_tick(SwarmScene *s, float dt, int cols, int rows)
{
    s->world.width = pw(cols);
    s->world.height = ph(rows);
    if (s->sim.paused) return;

    s->sim_time    += dt;
    s->sim.cycle_timer += dt;

    /* Auto-cycle: advance to next digit every AUTO_CYCLE_S seconds */
    if (s->sim.auto_cycle && s->sim.cycle_timer >= (float)AUTO_CYCLE_S) {
        s->sim.cycle_timer = 0.0f;
        scene_set_digit(s, (s->current_digit + 1) % 10);
    }

    Vec2 centroid = digit_centroid(s->slots, s->n_slots);

    for (int i = 0; i < N_AGENTS; i++)
        agent_tick(s->agents, N_AGENTS, i, s->strategy.params,
                   s->slots, s->n_slots,
                   s->world.width, s->world.height, dt,
                   s->strategy.index, centroid, s->sim_time);
}

/*
 * mark_cell — stamp one ASCII glyph at terminal cell (cx, cy).
 *
 * Centralises the (chtype)(unsigned char) cast plus bounds-check that
 * would otherwise be repeated at every mvwaddch site.  The double
 * cast prevents sign-extension on character values > 127 (per
 * CLAUDE.md "Common ncurses Bugs").  Off-screen cells are silently
 * dropped.
 */
static void mark_cell(WINDOW *w, int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/*
 * scene_draw — render all agents into WINDOW *w.
 *
 * Per-agent attribute ladder (drives the "what's settled" reading):
 *   within AT_SLOT_DIST px of slot   → A_BOLD   (agent has arrived)
 *   no slot assigned                 → A_DIM    (wandering, low priority)
 *   otherwise (en route)             → A_NORMAL
 *
 * Sub-tick alpha interpolation:
 *   draw_pos = prev_pos + (pos − prev_pos) × alpha
 * eliminates micro-stutter between fixed physics ticks.
 */
static void scene_draw(const SwarmScene *s, WINDOW *w,
                       int cols, int rows, float alpha)
{
    for (int i = 0; i < N_AGENTS; i++) {
        const Agent *a = &s->agents[i];

        Vec2 dp = v2add(a->prev_pos,
                        v2scale(v2sub(a->pos, a->prev_pos), alpha));

        attr_t attr;
        if (a->slot_idx < 0) {
            attr = A_DIM;            /* unassigned: wandering              */
        } else {
            float d = v2len(v2sub(s->slots[a->slot_idx].pos, a->pos));
            attr = (d < AT_SLOT_DIST) ? A_BOLD : A_NORMAL;
        }

        mark_cell(w, px_to_cell_x(dp.x), px_to_cell_y(dp.y),
                  a->glyph, a->color_pair, attr, cols, rows);
    }
}

/* ===================================================================== */
/* §10  app — screen, input, main loop                                 */
/* ===================================================================== */

/*
 * Screen — cell-space (terminal) extent + ncurses lifecycle wrapper.
 *
 * Intent
 *   Owns the TERMINAL side of the world.  Where SwarmScene.world
 *   tracks the pixel-space simulation box, this struct tracks the
 *   cell-space terminal grid that ncurses paints onto.  The two are
 *   linked but live in DIFFERENT spaces:
 *
 *      World.width  = Screen.cols × CELL_W      (CELL_W = 8 px)
 *      World.height = Screen.rows × CELL_H      (CELL_H = 16 px)
 *
 *   Keeping them in SEPARATE structs prevents the class of "drew in
 *   cell-space when I meant pixel-space" aspect-ratio bugs — physics
 *   never sees cells, the renderer never sees pixels except through
 *   the one px_to_cell_x/y bridge in §4.
 *
 * Why a tiny 2-field struct (not flat ints on App)
 *   • Lifecycle isolation: only screen_init / screen_resize /
 *     screen_free / screen_draw touch ncurses' initscr / endwin /
 *     mvprintw.  They all take `Screen *` to make this layer
 *     explicit at the type level.
 *   • Symmetry with World: simulation and rendering each get one
 *     struct named for the space they live in.
 *
 * Members
 *   cols   Terminal width in CELLS (from getmaxyx).  Used by the
 *          renderer for cell-bounds checks after px → cell conversion,
 *          and by screen_draw for right-aligning the top HUD row.
 *   rows   Terminal height in CELLS.  Bottom-row index is rows − 1
 *          (where the key-hint line paints).
 *
 * Invariants
 *   cols > 0 AND rows > 0 (from getmaxyx).
 *   SwarmScene.world.width  == cols × CELL_W,
 *   SwarmScene.world.height == rows × CELL_H,
 *   maintained in lockstep by scene_init / app_do_resize.
 *
 * References
 *   None directly — terminal extent is a rendering substrate concern.
 *   Aspect-ratio compensation (CELL_W vs CELL_H = 1:2) is project-
 *   specific; see CLAUDE.md §"Coordinate / Physics".
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s, int initial_theme)
{
    initscr(); noecho(); cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(initial_theme);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw — assemble one complete frame: agents, then HUD on top.
 *
 * HUD layout:
 *   Row 0:      [ SWARM ]  digit name  strategy name (index/total)  auto flag
 *   Row 1 right: fps / sim Hz
 *   Last row:   key hints
 *
 * The digit is always drawn in the centre by scene_draw; the HUD
 * overlays at the edges so it rarely overlaps the formation.
 */
/*
 * count_agents_at_slot — HUD metric: how many agents are currently
 * within AT_SLOT_DIST pixels of their assigned slot.  Reported as
 * "N/total formed" — gives the viewer a "completion percentage" for
 * the in-progress digit.
 */
static int count_agents_at_slot(const SwarmScene *sc)
{
    int at_slot = 0;
    for (int i = 0; i < N_AGENTS; i++) {
        const Agent *a = &sc->agents[i];
        if (a->slot_idx < 0) continue;
        float distance_to_slot = v2len(v2sub(sc->slots[a->slot_idx].pos, a->pos));
        if (distance_to_slot < AT_SLOT_DIST) at_slot++;
    }
    return at_slot;
}

/* format_hud_status — write the top-row HUD line into `buf`.
 * Format: fps, sim rate, digit, strategy(idx/total), theme, formed-
 * counter, and an AUTO/PAUSED suffix when those modes are active. */
static void format_hud_status(const SwarmScene *sc, double fps, int sim_fps,
                              char *buf, size_t buflen)
{
    int   formed_count = count_agents_at_slot(sc);
    const char *auto_suffix   = sc->sim.auto_cycle ? "  AUTO"   : "";
    const char *paused_suffix = sc->sim.paused     ? "  PAUSED" : "";

    snprintf(buf, buflen,
             " %5.0f fps  sim:%3d Hz  digit:%d  %s (%d/%d)  [%s]  %d/%d formed%s%s ",
             fps, sim_fps,
             sc->current_digit,
             sc->strategy.params->name, sc->strategy.index + 1, N_STRATEGIES,
             THEMES[sc->theme_idx].name,
             formed_count, sc->n_slots,
             auto_suffix, paused_suffix);
}

/* hud_paint_text — attron / mvprintw / attroff sandwich shared by both
 * HUD rows so the colour-pair setup isn't duplicated per call. */
static void hud_paint_text(int row, int col, int pair, const char *text)
{
    attron (COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* draw_hud_status — right-aligned status on row 0 (PAIR_HUD yellow) */
static void draw_hud_status(const Screen *s, const SwarmScene *sc,
                             double fps, int sim_fps)
{
    enum { HUD_TOP_ROW = 0 };
    char buf[120];
    format_hud_status(sc, fps, sim_fps, buf, sizeof buf);

    int right_col = s->cols - (int)strlen(buf);
    if (right_col < 0) right_col = 0;
    hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, buf);
}

/* draw_hud_hint — bottom-row key bindings strip (PAIR_HINT cyan) */
static void draw_hud_hint(const Screen *s)
{
    static const char *KEY_HINT =
        " q:quit  spc:pause  0-9:digit  n/p:strategy  t/T:theme  "
        "a:auto  r:scatter ";
    hud_paint_text(s->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

static void screen_draw(Screen *s, const SwarmScene *sc,
                        double fps, int sim_fps, float alpha)
{
    /* (1) clear offscreen buffer */
    erase();
    /* (2) paint the simulation (slots + agents) */
    scene_draw(sc, stdscr, s->cols, s->rows, alpha);
    /* (3) HUD on top — data row + action row */
    draw_hud_status(s, sc, fps, sim_fps);
    draw_hud_hint  (s);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── App ── */

/*
 * FpsCounter — rolling-window frame-rate estimator.
 *
 * Intent
 *   Per-frame fps would jitter; this struct accumulates frame_count +
 *   elapsed nanoseconds over FPS_WINDOW_NS (500 ms) and emits a
 *   smoothed `display` value each time the window fills.  Same shape
 *   as the FpsCounter on every other file in this project.
 *
 * Lifecycle
 *   fps_counter_init  — zero everything at startup.
 *   fps_counter_tick  — call once per frame with the frame's dt (ns);
 *                       emits a fresh `display` only when the window
 *                       fills.
 */
typedef struct {
    int     frame_count;
    int64_t window_ns;
    double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f)
{
    f->frame_count = 0;
    f->window_ns   = 0;
    f->display     = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt)
{
    const int64_t FPS_WINDOW_NS = (int64_t)NS_PER_SEC / 2;     /* 500 ms */
    f->frame_count++;
    f->window_ns += dt;
    if (f->window_ns < FPS_WINDOW_NS) return;
    f->display     = (double)f->frame_count
                   * (double)NS_PER_SEC / (double)f->window_ns;
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * App — top-level container for every persistent value.
 *
 *   scene       simulation state (sub-structs reachable from here)
 *   screen      terminal cell extent + ncurses lifecycle
 *   fps         rolling-window fps estimator for HUD
 *   sim_fps     target physics tick rate (config; not user-tweakable)
 *   running     sig_atomic_t flag cleared by SIGINT/TERM + 'q' key
 *   need_resize sig_atomic_t flag set by SIGWINCH; main reacts at
 *               the start of the next iteration
 */
typedef struct {
    SwarmScene            scene;
    Screen                screen;
    FpsCounter            fps;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;
static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app->need_resize = 0;
    /* Reload digit slots to fit new terminal size */
    scene_set_digit(&app->scene, app->scene.current_digit);
}

/*
 * app_handle_key — process one keystroke.
 *
 *   0–9     load that digit (rebuilds slots, reassigns agents)
 *   n/N     cycle to next strategy (wraps 10→0)
 *   p/P     cycle to prev strategy (wraps 0→9)
 *   a       toggle auto-cycle mode
 *   r       scatter agents (randomise positions, keep digit)
 *   space   pause / resume
 *   q/ESC   quit
 */
static bool app_handle_key(App *app, int ch)
{
    SwarmScene *sc = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': sc->sim.paused = !sc->sim.paused;             break;
    case 'a': case 'A':
        sc->sim.auto_cycle  = !sc->sim.auto_cycle;
        sc->sim.cycle_timer = 0.0f;
        break;
    case 'r': case 'R':
        scene_scatter(sc);
        assign_slots(sc->agents, N_AGENTS, sc->slots, sc->n_slots);
        break;
    case 'n': case 'N':
        scene_set_strategy(sc, sc->strategy.index + 1);
        break;
    case 'p': case 'P':
        scene_set_strategy(sc, sc->strategy.index - 1);
        break;
    case 't':
        /* Cycle to next theme, wrapping at N_THEMES */
        sc->theme_idx = (sc->theme_idx + 1) % N_THEMES;
        theme_apply(sc->theme_idx);
        break;
    case 'T':
        /* Cycle to previous theme, +N-1 mod N to avoid negative modulo */
        sc->theme_idx = (sc->theme_idx + N_THEMES - 1) % N_THEMES;
        theme_apply(sc->theme_idx);
        break;
    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9':
        /* Digit keys 0–9: select that digit */
        scene_set_digit(sc, ch - '0');
        sc->sim.cycle_timer = 0.0f;
        break;
    default: break;
    }
    return true;
}

/*
 * main — game loop (fixed-step accumulator; Fiedler [8]).
 *
 *   (1) handle pending SIGWINCH + reset timers
 *   (2) measure dt since last frame, capped at DT_CAP_NS
 *   (3) drain sim_accum: scene_tick at fixed dt_sec until caught up
 *   (4) sub-tick alpha for the renderer (= leftover ns / TICK_LEN_NS)
 *   (5) rolling-window fps via fps_counter_tick
 *   (6) sleep BEFORE render so terminal I/O stays inside FRAME_BUDGET_NS
 *   (7) draw + present
 *   (8) drain non-blocking input via app_handle_key
 */
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
    fps_counter_init(&app->fps);

    screen_init(&app->screen, 0 /* initial theme = Rainbow */);
    scene_init (&app->scene, app->screen.cols, app->screen.rows);

    const int64_t DT_CAP_NS       = 100 * NS_PER_MS;         /* avalanche guard */
    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS; /* render cadence  */
    const int64_t TICK_LEN_NS     = TICK_NS(app->sim_fps);
    const float   TICK_LEN_SEC    = (float)TICK_LEN_NS / (float)NS_PER_SEC;

    int64_t frame_time = clock_ns();
    int64_t sim_accum  = 0;

    while (app->running) {
        int64_t frame_start = clock_ns();

        /* (1) handle SIGWINCH */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* (2) measure dt, capped to avoid avalanche */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;

        /* (3) drain accumulator: fixed-step physics until caught up */
        sim_accum += dt;
        while (sim_accum >= TICK_LEN_NS) {
            scene_tick(&app->scene, TICK_LEN_SEC,
                       app->screen.cols, app->screen.rows);
            sim_accum -= TICK_LEN_NS;
        }

        /* (4) sub-tick alpha for renderer */
        float alpha = (float)sim_accum / (float)TICK_LEN_NS;

        /* (5) rolling-window fps counter */
        fps_counter_tick(&app->fps, dt);

        /* (6) sleep BEFORE render so terminal I/O stays inside budget */
        int64_t budget_left = FRAME_BUDGET_NS - (clock_ns() - frame_start);
        clock_sleep_ns(budget_left);

        /* (7) draw + present */
        screen_draw(&app->screen, &app->scene,
                    app->fps.display, app->sim_fps, alpha);
        screen_present();

        /* (8) drain input */
        int key = getch();
        if (key != ERR && !app_handle_key(app, key))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
