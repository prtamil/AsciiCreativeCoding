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
 *   §1 config   — strategies, slot subdivision, speeds, force weights
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 10 theme palettes + PAIR_HUD/PAIR_HINT + theme_apply
 *   §4 coords   — pixel↔cell aspect bridge + Vec2 helpers
 *   §5 entity   — Agent + Slot structs, spawn, agent_step
 *   §6 steering — seek / arrive / separate / wander / cohesion / align / spring
 *   §7 strategy — ten tick functions, one per strategy
 *   §8 digit    — bitmap library, slot extraction, greedy assignment
 *   §9 scene    — SwarmScene, scene_tick, scene_draw, mark_cell
 *   §10 app     — signals, resize, main game loop
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
 * References     : Reynolds, "Flocks, Herds, and Schools: A
 *                    Distributed Behavioral Model," SIGGRAPH 1987 —
 *                    the source of every force used here.
 *                  Reynolds, "Steering Behaviors for Autonomous
 *                    Characters," 1999 (red3d.com/cwr/steer/) — the
 *                    seek/arrive/separate/wander primer.
 *                  Wikipedia: "Boids", "Hooke's law" — the alignment
 *                    rule and the SPRING strategy's underlying
 *                    second-order ODE (ζ ≈ 0.53 here, underdamped).
 *                  Shiffman, "The Nature of Code" ch. 6 —
 *                    accessible walkthrough of the same forces.
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
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a stadium where 120 fans are about to spell "5".  Each
 * fan has a card with a seat number — that is the slot assignment.
 * They walk to their seats from wherever they currently are.  How
 * they walk is the strategy: a directed sprint (RUSH), a leisurely
 * wander (DRIFT), a left-to-right stream (FLOW), a rotating queue
 * (ORBIT), a herd that drifts together until everyone is somehow
 * in the right spot (FLOCK), a falling drop (GRAVITY), a bouncing
 * spring (SPRING), and so on.  When everyone is at their seat the
 * digit is visible; when the digit changes the cards are reissued
 * and the walk starts over.
 *
 * ALGORITHM IN STEPS  (per tick, per agent)
 * ─────────────────────────────────────────
 *   1. Compute the strategy's forces (each is a Vec2 in px/s²):
 *        - slot-arrive toward agents[i].slot pos (always present
 *          if agent has a slot)
 *        - separation from nearby neighbours (always present)
 *        - strategy-specific extras:
 *            DRIFT/FLOW/FLOCK   wander
 *            FLOW               rightward stream until aligned
 *            ORBIT              tangent to digit centroid
 *            FLOCK              cohesion + alignment
 *            PULSE              oscillating slot target
 *            VORTEX             tangent to own slot
 *            GRAVITY            constant downward pull
 *            SPRING             Hooke spring force replaces arrive
 *            WAVE               lateral sin perpendicular to approach
 *   2. force = Σ weighted forces
 *   3. vel = clamp(vel + force · dt, max_speed)
 *   4. prev_pos = pos
 *   5. pos = pos + vel · dt; bounce off walls.
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
 *   Slot subdivision (digit_load):
 *                for each '#' at bitmap (r, c):
 *                  for sr in 0..SLOT_GRID-1:
 *                    for sc in 0..SLOT_GRID-1:
 *                      slot.pos = cell_origin + sub_offset(sr, sc)
 *
 *   Pixel→cell:  cx = round(px / CELL_W)        (CELL_W = 8)
 *                cy = round(py / CELL_H)        (CELL_H = 16)
 *
 * WORKED EXAMPLE  (defaults: 160 agents, 80x24 terminal)
 * ──────────────────────────────────────────────────────
 *   World box        : 80 cols × 24 rows = 640 × 384 pixels.
 *   Digit footprint  : 5 cols × 5 cells/bitmap-col = 25 cell-cols
 *                       wide; 7 rows × 3 cells/bitmap-row = 21
 *                       cell-rows tall.  Fills ~31 % of screen width,
 *                       ~88 % of height — leaves 1 row top + 1 row
 *                       bottom for HUD and key-hint bars.
 *   Slot count       : digit 0/6/9 → 144 slots; digit 1 → 108;
 *                       digit 2/3 → 135; digit 4 → 126;
 *                       digit 5/8 → 153 (max); digit 7 → 99 (min).
 *                       N_AGENTS = 160 ensures EVERY slot of EVERY
 *                       digit can be filled.  On digit 7 (99 slots),
 *                       the surplus 61 agents wander A_DIM around
 *                       the formed shape; on digit 5/8 (153 slots),
 *                       only 7 wander.
 *   Per tick (1/60 s):
 *     a RUSH agent at arrive_speed 180 px/s travels 3 px ≈ less than
 *       half a column — visibly smooth.
 *     a DRIFT agent at arrive_speed 70 px/s travels 1.2 px — slow
 *       meander.
 *   Steering cost    : separation O(N²): 160·159 = 25 440 checks/tick
 *                       × 60 Hz ≈ 1.5 M/s.  Sub-millisecond at -O2.
 *   Slot assignment  : O(N·S) = 160·153 ≈ 24 500 ops, runs only on
 *                       digit change (key '0'-'9' or auto-cycle tick).
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   - More agents than slots: agents beyond n_slots get slot_idx = -1
 *     and wander.  scene_draw renders them A_DIM so the eye
 *     distinguishes "lost" agents from "settled" ones.
 *   - Greedy nearest assignment is not globally optimal (Hungarian
 *     algorithm would be), but the visual difference is minor and
 *     the cost is much lower.
 *   - Pulse direction at exact slot: when agent sits ON its slot,
 *     v2norm(agent − slot) is the zero vector.  PULSE uses
 *     centroid → slot direction instead so the push direction is
 *     stable.
 *   - Wander out of slot: DRIFT and FLOCK have a wander force that
 *     would keep kicking agents off the slot.  Both fade wander to
 *     zero inside WANDER_FADE_DIST so the digit can settle.
 *   - Frame cap: do NOT add dt back into elapsed (the bug fixed in
 *     this rewrite); use a `frame_start = clock_ns()` snapshot.
 *
 * HOW TO VERIFY
 * ─────────────
 *   - On startup, agents scatter randomly then all pile into digit 0
 *     within 2-3 seconds (default DRIFT strategy).  HUD shows
 *     "144/144 formed" once settled.
 *   - Press 1, 2, ... 9 — each digit forms a recognisable cluster of
 *     glyphs.  The number of mini-slots per '#' (9) is tight enough
 *     that the digit shape is clear, loose enough to read individual
 *     agents.
 *   - Press n to advance through strategies.  RUSH snaps in, DRIFT
 *     wanders in slowly, FLOW comes in left to right, ORBIT spirals,
 *     GRAVITY rains down, SPRING bounces, etc.
 *   - Press t / T — palette cycles through 10 themes; the formed
 *     digit re-tints on the next render.
 *   - Press a (auto-cycle): every 3 s the digit advances 0→1→...→9→0.
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
 * StrategyParams — all tuneable behaviour constants in one struct.
 *
 * g_sp always points to the active preset.  All strategy logic reads
 * through g_sp so a key-press takes effect on the very next tick.
 *
 * Fields
 * ──────
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

static const StrategyParams *g_sp        = &g_presets[0];
static int                   g_strat_idx = 0;

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
 * Theme — one named palette.  body[0..6] holds the seven xterm-256
 * foreground indices that agents cycle through (agent i uses pair
 * (i % N_COLORS) + 1).  Themes are cycled with t / T.
 *
 * Every entry sits in the bright half of the 256-colour cube (≥ 24)
 * per the project palette-brightness rule; entries below 24 become
 * invisible under A_DIM (used by wandering agents).
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
 * Agent — one particle in the swarm.
 *
 * pos / prev_pos   pixel-space position this tick and last tick.
 *                  Renderer lerps between them by alpha for smooth draw.
 * vel              velocity (px/s).
 * wander_angle     slowly-rotating heading angle for DRIFT / FLOW wander.
 *                  Accumulated each tick: += rand * WANDER_TURN_MAX * dt.
 * slot_idx         index into the current Slot array.
 *                  -1 = no slot assigned → agent wanders freely.
 * glyph            fixed ASCII character for this agent.
 * color_pair       ncurses color pair (1–7); fixed at spawn.
 */
typedef struct {
    Vec2  pos;
    Vec2  prev_pos;
    Vec2  vel;
    float wander_angle;
    int   slot_idx;
    char  glyph;
    int   color_pair;
} Agent;

/*
 * Slot — one target point in the digit's pixel-space point cloud.
 *
 * pos       pixel-space position derived from the bitmap '#'.
 * occupied  true while assigned to an agent (prevents double-assignment).
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

static Vec2 steer_separate(const Agent *agents, int n_agents, int self)
{
    Vec2 force = v2(0, 0);
    for (int i = 0; i < n_agents; i++) {
        if (i == self) continue;
        Vec2  away = v2sub(agents[self].pos, agents[i].pos);
        float d    = v2len(away);
        if (d < g_sp->sep_radius && d > 0.001f) {
            float strength = (g_sp->sep_radius - d) / g_sp->sep_radius;
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
                             float speed)
{
    Vec2 sum = v2(0, 0);
    int  n   = 0;
    for (int i = 0; i < n_agents; i++) {
        if (i == self) continue;
        if (v2len(v2sub(agents[i].pos, agents[self].pos)) > g_sp->neighbor_radius)
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
static Vec2 steer_align(const Agent *agents, int n_agents, int self)
{
    Vec2 sum = v2(0, 0);
    int  n   = 0;
    for (int i = 0; i < n_agents; i++) {
        if (i == self) continue;
        if (v2len(v2sub(agents[i].pos, agents[self].pos)) > g_sp->neighbor_radius)
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
 * calls agent_step() + bounce_pos().  All read tuneable values from g_sp.
 *
 * Common pattern:
 *   1. Compute slot-arrive force toward assigned slot (or wander if no slot).
 *   2. Compute separation force.
 *   3. Add any strategy-specific forces.
 *   4. Sum with weights, call agent_step, bounce.
 */

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

static void strategy_drift(Agent *agents, int n_agents, int self,
                            const Slot *slots, float ww, float wh, float dt)
{
    Agent *a   = &agents[self];
    Vec2   sep = steer_separate(agents, n_agents, self);

    /* Wander strength fades to 0 as agent closes in on its slot */
    float wander_scale = 1.0f;
    if (a->slot_idx >= 0) {
        float d = v2len(v2sub(slots[a->slot_idx].pos, a->pos));
        wander_scale = (d > WANDER_FADE_DIST) ? 1.0f : d / WANDER_FADE_DIST;
    }
    Vec2 wand   = steer_wander(a, g_sp->wander_strength * wander_scale, dt);
    Vec2 slot_f = v2(0, 0);

    if (a->slot_idx >= 0) {
        slot_f = steer_arrive(a->pos, a->vel, slots[a->slot_idx].pos,
                               g_sp->arrive_speed, g_sp->slow_radius);
        slot_f = v2scale(slot_f, g_sp->slot_weight);
    }

    Vec2 force = v2add(v2add(wand, slot_f),
                        v2scale(sep, g_sp->sep_weight));
    agent_step(a, force, g_sp->max_speed, dt);
    bounce_pos(&a->pos, &a->vel, ww, wh);
}

/* ── §7.2  RUSH ───────────────────────────────────────────────────── */
/*
 * Agents sprint directly to their slot using arrive (smooth stop on
 * arrival) with no wander.  Digit snaps into place briskly.
 *
 * force = slot_arrive × slot_weight  +  sep × sep_weight
 */
static void strategy_rush(Agent *agents, int n_agents, int self,
                           const Slot *slots, float ww, float wh, float dt)
{
    Agent *a   = &agents[self];
    Vec2   sep = steer_separate(agents, n_agents, self);
    Vec2   slot_f = v2(0, 0);

    if (a->slot_idx >= 0) {
        slot_f = steer_arrive(a->pos, a->vel, slots[a->slot_idx].pos,
                               g_sp->arrive_speed, g_sp->slow_radius);
        slot_f = v2scale(slot_f, g_sp->slot_weight);
    } else {
        /* No slot: glide to a stop */
        a->vel = v2scale(a->vel, 0.90f);
    }

    Vec2 force = v2add(slot_f, v2scale(sep, g_sp->sep_weight));
    agent_step(a, force, g_sp->max_speed, dt);
    bounce_pos(&a->pos, &a->vel, ww, wh);
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
static void strategy_flow(Agent *agents, int n_agents, int self,
                           const Slot *slots, float ww, float wh, float dt)
{
    Agent *a   = &agents[self];
    Vec2   sep = steer_separate(agents, n_agents, self);
    Vec2   task_f;

    if (a->slot_idx >= 0) {
        float dx = slots[a->slot_idx].pos.x - a->pos.x;

        if (dx > FLOW_X_THRESH) {
            /* Still left of slot column: stream rightward + faint slot pull */
            Vec2 stream = v2(FLOW_BIAS, 0.0f);
            Vec2 weak_arrive = steer_arrive(a->pos, a->vel, slots[a->slot_idx].pos,
                                             g_sp->arrive_speed * 0.3f, g_sp->slow_radius);
            task_f = v2add(stream, weak_arrive);
        } else {
            /* Aligned with slot x: switch fully to slot-arrive + wander */
            Vec2 slot_f = steer_arrive(a->pos, a->vel, slots[a->slot_idx].pos,
                                        g_sp->arrive_speed, g_sp->slow_radius);
            Vec2 wand   = steer_wander(a, g_sp->wander_strength * 0.4f, dt);
            task_f = v2add(v2scale(slot_f, g_sp->slot_weight), wand);
        }
    } else {
        /* No slot: ride the current with wander */
        Vec2 stream = v2(FLOW_BIAS * 0.6f, 0.0f);
        Vec2 wand   = steer_wander(a, g_sp->wander_strength, dt);
        task_f = v2add(stream, wand);
    }

    Vec2 force = v2add(task_f, v2scale(sep, g_sp->sep_weight));
    agent_step(a, force, g_sp->max_speed, dt);
    bounce_pos(&a->pos, &a->vel, ww, wh);
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
static void strategy_orbit(Agent *agents, int n_agents, int self,
                            const Slot *slots, float ww, float wh, float dt,
                            Vec2 digit_centroid)
{
    Agent *a      = &agents[self];
    Vec2   sep    = steer_separate(agents, n_agents, self);

    /* Tangent to the circle around digit_centroid (90° rotation) */
    Vec2 radial  = v2sub(a->pos, digit_centroid);
    Vec2 tangent = v2norm(v2(-radial.y, radial.x));  /* perpendicular, CW */
    Vec2 orbit_f = v2scale(tangent, ORBIT_STRENGTH);

    Vec2 slot_f = v2(0, 0);
    if (a->slot_idx >= 0) {
        slot_f = steer_arrive(a->pos, a->vel, slots[a->slot_idx].pos,
                               g_sp->arrive_speed, g_sp->slow_radius);
        slot_f = v2scale(slot_f, g_sp->slot_weight);
    }

    Vec2 force = v2add(v2add(orbit_f, slot_f),
                        v2scale(sep, g_sp->sep_weight));
    agent_step(a, force, g_sp->max_speed, dt);
    bounce_pos(&a->pos, &a->vel, ww, wh);
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
static void strategy_flock(Agent *agents, int n_agents, int self,
                            const Slot *slots, float ww, float wh, float dt)
{
    Agent *a = &agents[self];

    Vec2 sep = steer_separate(agents, n_agents, self);
    Vec2 coh = steer_cohesion(agents, n_agents, self, g_sp->arrive_speed);
    Vec2 aln = steer_align   (agents, n_agents, self);

    /* Wander fades near slot so agents settle instead of wandering out */
    float wander_scale = 1.0f;
    if (a->slot_idx >= 0) {
        float d = v2len(v2sub(slots[a->slot_idx].pos, a->pos));
        wander_scale = (d > WANDER_FADE_DIST) ? 1.0f : d / WANDER_FADE_DIST;
    }
    Vec2 wand = steer_wander(a, g_sp->wander_strength * wander_scale, dt);

    Vec2 slot_f = v2(0, 0);
    if (a->slot_idx >= 0) {
        slot_f = steer_arrive(a->pos, a->vel, slots[a->slot_idx].pos,
                               g_sp->arrive_speed, g_sp->slow_radius);
        slot_f = v2scale(slot_f, g_sp->slot_weight);
    }

    Vec2 force = v2add(
        v2add(v2scale(coh, g_sp->cohesion_weight),
              v2scale(aln, g_sp->align_weight)),
        v2add(v2add(v2scale(sep, g_sp->sep_weight), slot_f), wand)
    );
    agent_step(a, force, g_sp->max_speed, dt);
    bounce_pos(&a->pos, &a->vel, ww, wh);
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
static void strategy_pulse(Agent *agents, int n_agents, int self,
                            const Slot *slots, int n_slots, float ww, float wh,
                            float dt, float sim_time, Vec2 digit_centroid)
{
    Agent *a   = &agents[self];
    Vec2   sep = steer_separate(agents, n_agents, self);
    Vec2   task_f;

    if (a->slot_idx >= 0) {
        Vec2  slot_pos = slots[a->slot_idx].pos;

        /* Push direction: from digit centroid outward through the slot.
         * Using centroid→slot (not agent→slot) means the direction is
         * stable even when the agent sits exactly on its slot, where
         * v2norm(agent - slot) would be a degenerate zero vector. */
        Vec2  push_dir = v2norm(v2sub(slot_pos, digit_centroid));
        if (v2len(push_dir) < 0.001f) push_dir = v2(1.0f, 0.0f); /* fallback */

        float phase   = sinf(TWO_PI * PULSE_FREQ * sim_time);
        Vec2  osc_tgt = v2add(slot_pos, v2scale(push_dir, phase * PULSE_AMPLITUDE));

        /* First: converge to the oscillating target (not the raw slot).
         * The slot_weight must exceed the PULSE_AMPLITUDE perturbation so
         * agents always track the target rather than drifting freely. */
        task_f = steer_arrive(a->pos, a->vel, osc_tgt,
                               g_sp->arrive_speed, g_sp->slow_radius);
        task_f = v2scale(task_f, g_sp->slot_weight);
    } else {
        /* No slot: gentle friction; agent drifts to a stop */
        a->vel = v2scale(a->vel, 0.94f);
        task_f = v2(0, 0);
    }

    (void)n_slots;
    Vec2 force = v2add(task_f, v2scale(sep, g_sp->sep_weight));
    agent_step(a, force, g_sp->max_speed, dt);
    bounce_pos(&a->pos, &a->vel, ww, wh);
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
static void strategy_vortex(Agent *agents, int n_agents, int self,
                              const Slot *slots, float ww, float wh, float dt)
{
    Agent *a      = &agents[self];
    Vec2   sep    = steer_separate(agents, n_agents, self);
    Vec2   slot_f = v2(0, 0);
    Vec2   vortex_f = v2(0, 0);

    if (a->slot_idx >= 0) {
        Vec2  slot_pos  = slots[a->slot_idx].pos;
        Vec2  from_slot = v2sub(a->pos, slot_pos);
        float dist      = v2len(from_slot);

        /* Tangent: 90° CCW rotation of radius vector */
        Vec2  tangent = v2norm(v2(-from_slot.y, from_slot.x));
        /* Fade: full strength at VORTEX_FADE_DIST, zero at slot */
        float scale   = fminf(dist / VORTEX_FADE_DIST, 1.0f);
        vortex_f = v2scale(tangent, VORTEX_STRENGTH * scale);

        slot_f = steer_arrive(a->pos, a->vel, slot_pos,
                               g_sp->arrive_speed, g_sp->slow_radius);
        slot_f = v2scale(slot_f, g_sp->slot_weight);
    }

    Vec2 force = v2add(v2add(vortex_f, slot_f),
                        v2scale(sep, g_sp->sep_weight));
    agent_step(a, force, g_sp->max_speed, dt);
    bounce_pos(&a->pos, &a->vel, ww, wh);
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
static void strategy_gravity(Agent *agents, int n_agents, int self,
                               const Slot *slots, float ww, float wh, float dt)
{
    Agent *a    = &agents[self];
    Vec2   sep  = steer_separate(agents, n_agents, self);
    Vec2   grav = v2(0.0f, GRAVITY_PULL);   /* positive y = downward */
    Vec2   slot_f = v2(0, 0);

    if (a->slot_idx >= 0) {
        slot_f = steer_arrive(a->pos, a->vel, slots[a->slot_idx].pos,
                               g_sp->arrive_speed, g_sp->slow_radius);
        slot_f = v2scale(slot_f, g_sp->slot_weight);
    }

    Vec2 force = v2add(v2add(grav, slot_f),
                        v2scale(sep, g_sp->sep_weight));
    agent_step(a, force, g_sp->max_speed, dt);
    bounce_pos(&a->pos, &a->vel, ww, wh);
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
static void strategy_spring(Agent *agents, int n_agents, int self,
                              const Slot *slots, float ww, float wh, float dt)
{
    Agent *a   = &agents[self];
    Vec2   sep = steer_separate(agents, n_agents, self);
    Vec2   slot_f = v2(0, 0);

    if (a->slot_idx >= 0) {
        slot_f = steer_spring(a->pos, a->vel, slots[a->slot_idx].pos,
                               SPRING_K, SPRING_DAMP);
    } else {
        /* No slot: dampen to rest */
        a->vel = v2scale(a->vel, 0.92f);
    }

    Vec2 force = v2add(slot_f, v2scale(sep, g_sp->sep_weight));
    agent_step(a, force, g_sp->max_speed, dt);
    bounce_pos(&a->pos, &a->vel, ww, wh);
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
static void strategy_wave(Agent *agents, int n_agents, int self,
                            const Slot *slots, float ww, float wh,
                            float dt, float sim_time)
{
    Agent *a   = &agents[self];
    Vec2   sep = steer_separate(agents, n_agents, self);
    Vec2   slot_f = v2(0, 0);
    Vec2   wave_f = v2(0, 0);

    if (a->slot_idx >= 0) {
        Vec2  slot_pos = slots[a->slot_idx].pos;
        Vec2  to_slot  = v2sub(slot_pos, a->pos);
        float dist     = v2len(to_slot);
        Vec2  toward   = v2norm(to_slot);

        /* Perpendicular to approach direction (lateral axis) */
        Vec2  perp     = v2(-toward.y, toward.x);

        /* Amplitude: ramps up from 0, then fades back to 0 near slot */
        float ramp_amp = fminf(dist * 0.5f, WAVE_AMPLITUDE);
        float fade     = fminf(dist / WAVE_FADE_DIST, 1.0f);
        float amp      = ramp_amp * fade;

        wave_f = v2scale(perp, amp * sinf(TWO_PI * WAVE_FREQ * sim_time));

        slot_f = steer_arrive(a->pos, a->vel, slot_pos,
                               g_sp->arrive_speed, g_sp->slow_radius);
        slot_f = v2scale(slot_f, g_sp->slot_weight);
    }

    Vec2 force = v2add(v2add(slot_f, wave_f),
                        v2scale(sep, g_sp->sep_weight));
    agent_step(a, force, g_sp->max_speed, dt);
    bounce_pos(&a->pos, &a->vel, ww, wh);
}

/*
 * agent_tick — dispatch to the active strategy for one agent.
 *
 * digit_centroid and sim_time are passed through because ORBIT, PULSE,
 * and WAVE need them; other strategies ignore them.
 */
static void agent_tick(Agent *agents, int n_agents, int self,
                       const Slot *slots, int n_slots,
                       float ww, float wh, float dt,
                       int strategy, Vec2 digit_centroid, float sim_time)
{
    switch (strategy) {
    case 0: strategy_drift   (agents, n_agents, self, slots,          ww, wh, dt); break;
    case 1: strategy_rush    (agents, n_agents, self, slots,          ww, wh, dt); break;
    case 2: strategy_flow    (agents, n_agents, self, slots,          ww, wh, dt); break;
    case 3: strategy_orbit   (agents, n_agents, self, slots,          ww, wh, dt, digit_centroid); break;
    case 4: strategy_flock   (agents, n_agents, self, slots,          ww, wh, dt); break;
    case 5: strategy_pulse   (agents, n_agents, self, slots, n_slots, ww, wh, dt, sim_time, digit_centroid); break;
    case 6: strategy_vortex  (agents, n_agents, self, slots,          ww, wh, dt); break;
    case 7: strategy_gravity (agents, n_agents, self, slots,          ww, wh, dt); break;
    case 8: strategy_spring  (agents, n_agents, self, slots,          ww, wh, dt); break;
    case 9: strategy_wave    (agents, n_agents, self, slots,          ww, wh, dt, sim_time); break;
    }
}

/* ===================================================================== */
/* §8  digit library                                                    */
/* ===================================================================== */

/*
 * digit_load — extract Slot positions from a digit's bitmap.
 *
 * The 5x7 template is scaled by DIGIT_CELL_W × DIGIT_CELL_H terminal
 * cells and centred on the screen.  Each '#' becomes a SLOT_GRID ×
 * SLOT_GRID grid of mini-slots packed into the scaled cell.
 *
 * Sub-slot placement (the "no rounding leak" trick):
 *
 *   We place each sub-slot at the EXACT TOP-LEFT of a terminal cell —
 *   px = ox + (c · DIGIT_CELL_W + sc · stride_x) · CELL_W
 *   py = oy + (r · DIGIT_CELL_H + sr · stride_y) · CELL_H
 *   where stride_x = (DIGIT_CELL_W − 1) / (SLOT_GRID − 1)
 *         stride_y = (DIGIT_CELL_H − 1) / (SLOT_GRID − 1)
 *
 *   stride_x = (5 − 1) / (3 − 1) = 2  → sub-cols 0, 2, 4 in a 5-wide cell
 *   stride_y = (3 − 1) / (3 − 1) = 1  → sub-rows 0, 1, 2 in a 3-tall cell
 *
 *   Putting sub-slots at terminal-cell-aligned coordinates means
 *   px_to_cell_y/x rounds them exactly to the intended row/col with
 *   no boundary leak.  At fractional positions like (sc + 0.5)/SLOT_GRID,
 *   the third sub-slot lands at py = 26.7 which the round-half-up
 *   `floor(py/16 + 0.5)` maps to row 2 — the NEXT cell's row 0 — so
 *   the third sub-row of bitmap row 0 ends up rendering on top of
 *   bitmap row 1.  That is the "6 looks like 5" bug fix.
 *
 * Returns the total slot count (SLOT_GRID² × bitmap-fill-count).
 */
static int digit_load(Slot *slots, int digit, int cols, int rows)
{
    int n = 0;

    /* Pixel-space origin: top-left corner of the digit's bounding box */
    const float cell_w_px  = (float)(DIGIT_CELL_W * CELL_W);
    const float cell_h_px  = (float)(DIGIT_CELL_H * CELL_H);
    const float digit_px_w = (float)DIGIT_NCOLS * cell_w_px;
    const float digit_px_h = (float)DIGIT_NROWS * cell_h_px;
    const float ox         = (pw(cols) - digit_px_w) * 0.5f;
    const float oy         = (ph(rows) - digit_px_h) * 0.5f;

    /* Stride between sub-slots in TERMINAL CELLS.  Distributes
     * SLOT_GRID sub-slots evenly across DIGIT_CELL_W / DIGIT_CELL_H
     * terminal cells, with first at offset 0 and last at the cell's
     * far edge (col DIGIT_CELL_W − 1 / row DIGIT_CELL_H − 1).
     * Integer division — works cleanly when (DIM − 1) divides
     * (SLOT_GRID − 1), which holds for our defaults. */
    const int stride_x = (DIGIT_CELL_W - 1) / (SLOT_GRID - 1);  /* 2 */
    const int stride_y = (DIGIT_CELL_H - 1) / (SLOT_GRID - 1);  /* 1 */

    for (int r = 0; r < DIGIT_NROWS && n < SLOTS_MAX; r++) {
        const char *row = DIGIT_BITMAPS[digit][r];
        for (int c = 0; row[c] && c < DIGIT_NCOLS; c++) {
            if (row[c] != '#') continue;

            /* Top-left of this '#' cell, in TERMINAL-CELL coordinates. */
            int cell_col0 = c * DIGIT_CELL_W;
            int cell_row0 = r * DIGIT_CELL_H;

            /* SLOT_GRID × SLOT_GRID sub-slots, each at a distinct
             * terminal cell aligned by stride_x / stride_y. */
            for (int sr = 0; sr < SLOT_GRID && n < SLOTS_MAX; sr++) {
                for (int sc = 0; sc < SLOT_GRID && n < SLOTS_MAX; sc++) {
                    int sub_col = cell_col0 + sc * stride_x;
                    int sub_row = cell_row0 + sr * stride_y;
                    slots[n].pos      = v2(ox + (float)sub_col * (float)CELL_W,
                                           oy + (float)sub_row * (float)CELL_H);
                    slots[n].occupied = false;
                    n++;
                }
            }
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
 * SwarmScene — complete simulation state.
 *
 * Field groups:
 *   - agent + slot pools (touched every tick by every steering rule)
 *   - active selection (which digit, which strategy, which theme)
 *   - simulation clock + world dimensions
 *   - user mode flags (paused, auto_cycle)
 */
typedef struct {
    /* agent + slot pools — first n_slots slots active per digit */
    Agent agents[N_AGENTS];
    Slot  slots[SLOTS_MAX];
    int   n_slots;

    /* active selection */
    int   current_digit;       /* 0..9                                 */
    int   strategy;            /* 0..N_STRATEGIES-1                    */
    int   theme_idx;           /* 0..N_THEMES-1; t/T cycles            */

    /* simulation clock + world dimensions */
    float sim_time;            /* drives PULSE/WAVE oscillations       */
    float world_w, world_h;    /* refreshed each tick from cols/rows   */

    /* user mode flags */
    bool  paused;
    bool  auto_cycle;          /* auto-advance digit every AUTO_CYCLE_S */
    float cycle_timer;         /* seconds since last auto-advance       */
} SwarmScene;

/* scene_scatter — randomise all agent positions (used on 'r' key and init) */
static void scene_scatter(SwarmScene *s)
{
    for (int i = 0; i < N_AGENTS; i++) {
        s->agents[i].pos      = v2(randf() * s->world_w, randf() * s->world_h);
        s->agents[i].prev_pos = s->agents[i].pos;
        s->agents[i].vel      = v2(0, 0);
    }
}

static void scene_set_digit(SwarmScene *s, int digit)
{
    s->current_digit = digit;
    s->n_slots = digit_load(s->slots, digit,
                             (int)(s->world_w / CELL_W),
                             (int)(s->world_h / CELL_H));
    assign_slots(s->agents, N_AGENTS, s->slots, s->n_slots);
}

static void scene_init(SwarmScene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->world_w = pw(cols);
    s->world_h = ph(rows);

    for (int i = 0; i < N_AGENTS; i++)
        agent_spawn(&s->agents[i], i, s->world_w, s->world_h);

    scene_set_digit(s, 0);
}

static void scene_tick(SwarmScene *s, float dt, int cols, int rows)
{
    s->world_w = pw(cols);
    s->world_h = ph(rows);
    if (s->paused) return;

    s->sim_time    += dt;
    s->cycle_timer += dt;

    /* Auto-cycle: advance to next digit every AUTO_CYCLE_S seconds */
    if (s->auto_cycle && s->cycle_timer >= (float)AUTO_CYCLE_S) {
        s->cycle_timer = 0.0f;
        scene_set_digit(s, (s->current_digit + 1) % 10);
    }

    Vec2 centroid = digit_centroid(s->slots, s->n_slots);

    for (int i = 0; i < N_AGENTS; i++)
        agent_tick(s->agents, N_AGENTS, i,
                   s->slots, s->n_slots,
                   s->world_w, s->world_h, dt,
                   s->strategy, centroid, s->sim_time);
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
static void screen_draw(Screen *s, const SwarmScene *sc,
                        double fps, int sim_fps, float alpha)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha);

    /* Count agents currently at their slot */
    int at_slot = 0;
    for (int i = 0; i < N_AGENTS; i++) {
        if (sc->agents[i].slot_idx < 0) continue;
        float d = v2len(v2sub(sc->slots[sc->agents[i].slot_idx].pos,
                               sc->agents[i].pos));
        if (d < AT_SLOT_DIST) at_slot++;
    }

    /* Top-right status — PAIR_HUD bright yellow, A_BOLD */
    char buf[120];
    snprintf(buf, sizeof buf,
             " %5.0f fps  sim:%3d Hz  digit:%d  %s (%d/%d)  [%s]  %d/%d formed%s%s ",
             fps, sim_fps,
             sc->current_digit,
             g_sp->name, g_strat_idx + 1, N_STRATEGIES,
             THEMES[sc->theme_idx].name,
             at_slot, sc->n_slots,
             sc->auto_cycle ? "  AUTO"   : "",
             sc->paused     ? "  PAUSED" : "");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom-left key hint — PAIR_HINT bright cyan, A_BOLD */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  0-9:digit  n/p:strategy  t/T:theme  a:auto  r:scatter ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── App ── */
typedef struct {
    SwarmScene            scene;
    Screen                screen;
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
    case ' ': sc->paused = !sc->paused;             break;
    case 'a': case 'A':
        sc->auto_cycle  = !sc->auto_cycle;
        sc->cycle_timer = 0.0f;
        break;
    case 'r': case 'R':
        scene_scatter(sc);
        assign_slots(sc->agents, N_AGENTS, sc->slots, sc->n_slots);
        break;
    case 'n': case 'N':
        /* Cycle to next strategy, wrapping 9 → 0 */
        g_strat_idx      = (g_strat_idx + 1) % N_STRATEGIES;
        g_sp             = &g_presets[g_strat_idx];
        sc->strategy     = g_strat_idx;
        break;
    case 'p': case 'P':
        /* Cycle to previous strategy, wrapping 0 → 9 (+9 mod 10 = -1 mod 10) */
        g_strat_idx      = (g_strat_idx + N_STRATEGIES - 1) % N_STRATEGIES;
        g_sp             = &g_presets[g_strat_idx];
        sc->strategy     = g_strat_idx;
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
        sc->cycle_timer = 0.0f;
        break;
    default: break;
    }
    return true;
}

/*
 * main — game loop (fixed-step accumulator; same pattern as war.c §8).
 *
 * ① dt: wall-clock elapsed since last frame, capped at 100 ms.
 * ② Drain sim_accum: scene_tick at fixed dt_sec until empty.
 * ③ alpha = leftover ns / tick_ns ∈ [0,1) — sub-tick render offset.
 * ④ Sleep remaining TARGET_FPS budget before render.
 * ⑤ erase → scene_draw → HUD → doupdate.
 * ⑥ Non-blocking getch.
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

    screen_init(&app->screen, 0 /* initial theme = Rainbow */);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        int64_t frame_start = clock_ns();

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns(); sim_accum = 0;
        }

        /* ① dt */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ② fixed-step accumulator */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ③ alpha */
        float alpha = (float)sim_accum / (float)tick_ns;

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0; fps_accum = 0;
        }

        /* ④ frame cap — sleep BEFORE render to keep terminal I/O off
         * the next frame's budget.  elapsed = wall time spent on
         * physics + accounting since frame_start.  Do NOT add dt back
         * into elapsed — that cancels the cap, sleep is always 0,
         * CPU pegs at 100 %.                                         */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        /* ⑤ */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha);
        screen_present();

        /* ⑥ */
        int key = getch();
        if (key != ERR) {
            if (!app_handle_key(app, key)) app->running = 0;
        }
    }

    screen_free(&app->screen);
    return 0;
}
