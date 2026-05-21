/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * shepherd.c — autonomous border collie herds scattered sheep back into a pen
 *
 * DEMO: A flock of sheep grazes calmly inside a compound drawn at the
 *       centre of the screen.  Press SPACE: every sheep gets an outward
 *       impulse and runs for the boundary.  A single border collie
 *       outside the compound runs to whichever sheep has strayed
 *       furthest, positions itself behind that sheep relative to the
 *       compound centre, and the sheep — fleeing the dog — runs back
 *       through the boundary on its own.  The dog repeats with the
 *       next outlier until every sheep is back inside.
 *
 *       The compound starts as a circle and can be cycled through five
 *       regular polygons (CIRCLE → SQUARE → TRIANGLE → HEXAGON → OCTAGON)
 *       with u / v keys.  Each shape has different concavity and area;
 *       the dog handles all five without special-case logic.
 *
 * Study alongside: flocking/flocking.c (the boid forces sheep use)
 *                  flocking/crowd.c    (steering primer + Person model)
 *
 * Section map:
 *   §1 config   — pen geometry, sheep/dog speeds, force weights,
 *                 CompoundShape enum + COMPOUND_DEFS (5 regular polygons)
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — pen / sheep / dog pairs + PAIR_HUD/PAIR_HINT
 *   §4 coords   — pixel↔cell aspect-ratio bridge + Vec2 helpers
 *   §4.5 world  — World struct (px extent) + Compound struct (shape +
 *                 centre + bounding_radius) + compound_is_inside /
 *                 _distance_outside / _name / _cycle_shape helpers
 *   §5 sheep    — Sheep struct, sheep_spawn_in_pen, four steering forces
 *   §6 dog      — Dog struct + Strömbom collect/patrol controller
 *   §7 scene    — SimControls + Scene (= sheep + dog + sim + compound
 *                 + world + in_pen_count); tick + scatter + render
 *                 (draw_pen + draw_compound + draw_pen_or_compound)
 *   §8 app      — Screen, FpsCounter, App (= scene + screen + fps +
 *                 sim_fps + sig flags); signals, resize, main game loop
 *
 * Keys:
 *   q / ESC    quit                       space    SCATTER (panic the herd)
 *   S          mega-scatter (2x impulse)  c        toggle continuous chaos
 *   u / v      cycle compound shape       p        pause / resume
 *              (CIRCLE → SQUARE → TRIANGLE
 *               → HEXAGON → OCTAGON → CIRCLE)
 *   r          reset (regather in pen)
 *   + / -      add / remove 5 sheep
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra flocking/shepherd.c -o shepherd -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two interlocking pieces share the screen.
 *
 *                  SHEEP — each sheep is a steering-behaviour particle
 *                  (Reynolds 1987 style) with four forces summed each
 *                  tick: separation from close neighbours, soft cohesion
 *                  toward the local centroid, flee from the dog (only
 *                  when within DOG_FLEE_RADIUS), and a gentle inward
 *                  pull whenever the sheep is outside the pen.  Forces
 *                  are integrated into velocity with a per-tick damping
 *                  factor so motion settles to rest when no force
 *                  applies (sheep can actually stop and graze).
 *
 *                  DOG — a Strömbom-style controller (Strömbom et al.
 *                  2014, "Solving the shepherding problem").  Each tick
 *                  the dog picks one of two modes:
 *                    1. COLLECT — if any sheep is more than
 *                       (pen_radius + tolerance) from the pen centre,
 *                       find the worst outlier and place the dog at
 *                       sheep_pos + APPROACH_OFFSET · (sheep − pen)/|...|.
 *                       I.e. directly OUTSIDE the sheep relative to the
 *                       pen.  The sheep, fleeing the dog, runs toward
 *                       the pen interior — the goal direction emerges
 *                       from positioning, not from a goal-aware sheep.
 *                    2. PATROL — when all sheep are inside the pen
 *                       (with tolerance), the dog walks a slow circle
 *                       around the pen at radius pen_r + PATROL_OFFSET.
 *                       Patrolling rather than parking keeps the dog
 *                       visible and reactive when the user triggers a
 *                       new scatter event.
 *
 *                  The dog has no path planner — it simply steers
 *                  toward its target position at DOG_SPEED.  The
 *                  emergent herding comes from the two-mode position
 *                  selection.
 *
 * Data-structure : Scene owns a Pen (centre + radius), a fixed-capacity
 *                  Sheep pool (only first n_sheep slots active), a
 *                  single Dog, and the current world dimensions
 *                  (refreshed each tick from terminal cols/rows).
 *                  Sheep carry pos/prev_pos/vel + a `fleeing` flag set
 *                  each tick by the flee force.  Dog carries pos +
 *                  target + the controller's mode + the sheep index it
 *                  is currently collecting.
 *
 * Rendering      : Painter's order — pen ring (`*` and `.` alternated
 *                  for a dashed look) → sheep (`o` calm, `O` panicking)
 *                  → dog (`&`, A_BOLD).  Sub-tick alpha lerp of pos
 *                  smooths motion at any sim-rate / render-rate combo.
 *                  Every glyph stamp goes through a `mark_cell()`
 *                  helper that carries the (chtype)(unsigned char) cast
 *                  and the bounds-check.
 *
 * Performance    : Sheep separation/cohesion are O(N²); dog mode is
 *                  O(N) (one scan to find the worst outlier).  At
 *                  N = 30 sheep this is 30·29 = 870 distance checks
 *                  per tick — under 0.1 ms on any modern CPU.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 *   ── Canonical shepherding paper ────────────────────────────────
 *   [1] Strömbom, D., Mann, R. P., Wilson, A. M., Hailes, S.,
 *       Morton, A. J., Sumpter, D. J. T. & King, A. J. (2014),
 *       "Solving the shepherding problem: heuristics for herding
 *       autonomous, interacting agents", J. R. Soc. Interface
 *       11(100), 20140719 — THE source for this file's algorithm.
 *       Introduces the COLLECT-vs-DRIVE decomposition: find the worst
 *       outlier, position the dog on the far side of it relative to
 *       the target, the sheep flees the dog AND happens to flee
 *       toward the target.  §6 dog_decide_target is a direct
 *       implementation of this heuristic.
 *
 *   ── Underlying boid model ──────────────────────────────────────
 *   [2] Reynolds, C. W. (1987), "Flocks, Herds, and Schools: A
 *       Distributed Behavioral Model", SIGGRAPH '87 Computer Graphics
 *       21(4), pp. 25-34 — separation + alignment + cohesion as the
 *       three local rules that produce flocking.  The sheep in §5
 *       use these rules (separation + cohesion only; alignment is
 *       dropped because real sheep don't strongly align — they
 *       cluster instead).
 *   [3] Reynolds, C. W. (1999), "Steering Behaviors for Autonomous
 *       Characters", Game Developers Conference 1999.  Online at
 *       https://www.red3d.com/cwr/steer/ — canonical primer on the
 *       seek/flee/wander/separate primitives.  sheep_flee_dog and
 *       sheep_pen_pull use the "seek" form (force = desired − vel)
 *       defined here.
 *
 *   ── Collective-behaviour biology ───────────────────────────────
 *   [4] Couzin, I. D., Krause, J., James, R., Ruxton, G. D. &
 *       Franks, N. R. (2002), "Collective memory and spatial sorting
 *       in animal groups", J. Theor. Biol. 218(1), pp. 1-11 —
 *       formalises zones of repulsion / orientation / attraction.
 *       Justifies SHEEP_SEP_RADIUS < SHEEP_COH_RADIUS and the choice
 *       to omit alignment (sheep are far from boids — they prefer
 *       clustering to streaming).
 *   [5] King, A. J., Wilson, A. M., Wilshin, S. D., Lowe, J., Haddadi,
 *       H., Hailes, S. & Morton, A. J. (2012), "Selfish-herd
 *       behaviour of sheep under threat", Current Biology 22(14),
 *       pp. R561-R562 — empirical observation that sheep under
 *       predator pressure cluster strongly toward the flock centroid.
 *       Validates the pen-pull + cohesion combo used here.
 *
 *   ── Robotic / computational shepherding precedents ─────────────
 *   [6] Vaughan, R., Sumpter, N., Henderson, J., Frost, A. & Cameron,
 *       S. (2000), "Experiments in automatic flock control",
 *       Robotics and Autonomous Systems 31, pp. 109-117 — early
 *       hardware demonstration of robotic shepherding with a real
 *       dog and live ducks.  Empirical complement to Strömbom [1].
 *   [7] Lien, J.-M., Bayazit, O. B., Sowell, R. T., Rodriguez, S. &
 *       Amato, N. M. (2004), "Shepherding behaviors", IEEE Int.
 *       Conf. Robotics and Automation, pp. 4159-4164 — earlier
 *       computer-graphics implementation of shepherding with
 *       multiple shepherds and obstacle avoidance.  Precedes
 *       Strömbom's heuristic synthesis.
 *
 *   ── Implementation-oriented references ─────────────────────────
 *   [8] Shiffman, D., "The Nature of Code", Ch. 6 — reading-level
 *       walkthrough of Reynolds steering with Processing examples.
 *       The single most accessible introduction; pair with [3].
 *       https://natureofcode.com/book/chapter-6-autonomous-agents/
 *
 *   ── Game-loop / fixed-step physics ─────────────────────────────
 *   [9] Fiedler, G. (2004, updated 2014), "Fix Your Timestep!",
 *       https://gafferongames.com/post/fix_your_timestep/ — the
 *       fixed-step accumulator + sub-tick interpolation pattern
 *       implemented in main() (drains sim_accum until caught up)
 *       and scene_draw (alpha-lerp via prev_pos → pos).  Explains
 *       why the alpha lerp is essential to keep motion smooth at
 *       any render-rate / sim-rate combination.
 *
 *   ── Computational geometry (compound polygons) ─────────────────
 *  [10] O'Rourke, J. (1998), "Computational Geometry in C" (2nd ed.),
 *       Cambridge University Press, §1.5 + §2.4 — the cross-product
 *       point-in-convex-polygon test used by §4.5
 *       compound_is_inside, and the point-to-segment distance
 *       formula used by compound_distance_outside.
 *
 *   ── Online quick reference ─────────────────────────────────────
 *  [11] Wikipedia: "Sheepdog trial", "Herding behavior", "Boids" —
 *       https://en.wikipedia.org/wiki/Sheepdog_trial
 *       https://en.wikipedia.org/wiki/Herding_behavior
 *       https://en.wikipedia.org/wiki/Boids — context for the
 *       real-world task this file simulates and the three-rule
 *       model the sheep obey.
 *
 *   ── Companion files in this project ────────────────────────────
 *   See also:
 *     flocking/flocking.c    — five switchable flocking algorithms
 *       (boids / chase / vicsek / orbit / predator-prey).  Read first
 *       for the per-rule flocking story.
 *     flocking/crowd.c       — single-flock crowd with six behaviour
 *       modes; the steering primitives map 1:1 to this file's sheep.
 *     flocking/murmuration.c — large-N flock (N=1500) with density-
 *       field rendering and spatial hashing.  Same boid base, very
 *       different rendering approach.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The dog never tells a sheep where to go.  The dog picks WHERE TO
 * STAND, and a fleeing sheep does the rest.  If the dog stands on the
 * far side of a sheep relative to the pen, the sheep — running away
 * from the dog — happens to run toward the pen.  Repeat for every
 * outlier and the herd collapses back inward.  The whole "intelligence"
 * is one geometric placement rule applied each tick.
 *
 *
 * ALGORITHM IN STEPS  (per tick)
 * ──────────────────────────────
 *   1. Read inputs: SPACE → impulse every sheep outward; 'c' → toggle
 *      continuous random nudges; u/v → cycle compound shape; arrow
 *      keys not used (autonomous).
 *   2. Dog controller:
 *        a. scan sheep, find argmax(compound_distance_outside(sheep_pos)).
 *           For CIRCLE: equivalent to "furthest past the radius".  For
 *           polygons: shortest distance to ANY edge (since polygons
 *           are convex; a union's exterior distance is the min over
 *           edges).
 *        b. if worst_outside_dist > PEN_TOLERANCE:
 *               mode = COLLECT
 *               target = sheep[worst].pos
 *                      + DOG_APPROACH_OFFSET · normalize(sheep[worst].pos
 *                                                         − compound.centre)
 *           else:
 *               mode = PATROL
 *               patrol_phase += DOG_PATROL_OMEGA · dt
 *               target = compound.centre
 *                      + (bounding_radius + DOG_PATROL_OFFSET)
 *                      · (cos φ, sin φ)
 *        c. dog.vel = DOG_SPEED · normalize(target − dog.pos)
 *           dog.pos += dog.vel · dt
 *   3. Sheep update (two-stage so all sheep see the OLD positions):
 *        a. for each sheep i compute force = Σ weights · (sep, coh, flee, pen_pull)
 *        b. for each sheep i: vel = (vel + force·dt) · DAMPING ; clamp |vel|
 *        c. for each sheep i: prev_pos = pos ; pos += vel · dt ; clamp to world.
 *   4. Render: compound outline → sheep (alpha-lerped) → dog (alpha-
 *      lerped, on top).
 *
 * KEY FORMULAS
 * ────────────
 *   Sheep separation (per neighbour with 0 < d < SHEEP_SEP_RADIUS):
 *     strength = (SHEEP_SEP_RADIUS − d) / SHEEP_SEP_RADIUS
 *     force   += normalize(self − neighbour) · strength · SHEEP_GRAZE_SPEED
 *
 *   Sheep flee from dog (only if within DOG_FLEE_RADIUS):
 *     strength = (DOG_FLEE_RADIUS − d) / DOG_FLEE_RADIUS
 *     force   += normalize(self − dog) · strength · SHEEP_FLEE_SPEED
 *
 *   Sheep pen pull (only if OUTSIDE the active compound):
 *     force   += normalize(compound.centre − self) · SHEEP_GRAZE_SPEED
 *
 *   Dog COLLECT target (the geometric trick):
 *     target  = sheep[worst].pos
 *             + DOG_APPROACH_OFFSET · normalize(sheep[worst].pos
 *                                                − compound.centre)
 *
 *     i.e. the dog stands directly outside the sheep, on the line from
 *     compound_centre-through-sheep, at offset DOG_APPROACH_OFFSET
 *     further out.  The sheep, fleeing radially away from the dog,
 *     ends up moving along the SAME line but in the OPPOSITE direction
 *     — back toward the compound.
 *
 *   Dog PATROL target:
 *     target  = compound.centre
 *             + (bounding_radius + DOG_PATROL_OFFSET) · (cos φ, sin φ)
 *     φ      += DOG_PATROL_OMEGA · dt
 *
 *   Pixel→cell aspect bridge:
 *     cx = round(px / CELL_W)        (CELL_W = 8)
 *     cy = round(py / CELL_H)        (CELL_H = 16)
 *
 *
 * Background you need
 * ───────────────────
 *   - flocking.c T1-T7 (boid + Reynolds rules + 2-stage update).
 *   - crowd.c T1-T2 (steering primitives — seek/flee).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Reinforcement learning / trained dog policies. Strömbom's
 *     algorithm is a HAND-CRAFTED rule that works without
 *     training.
 *   - Goal-aware sheep with mental maps of the pen. Sheep are
 *     PURELY REACTIVE — they don't even know the pen exists in
 *     a meaningful sense (only that "outside is bad, inside is
 *     fine").
 *   - Multi-dog coordination. We have ONE dog; multi-dog
 *     herding adds a coordination layer Strömbom (2014) covers
 *     but we don't.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_DEFAULT = 60,
    TARGET_FPS      = 60,

    HUD_COLS        = 96,
    FPS_UPDATE_MS   = 500,

    /* Colour pair IDs — see §3 color_init for the actual colour values. */
    PAIR_PEN_RING    = 1,   /* dashed pen boundary circle               */
    PAIR_SHEEP_CALM  = 2,   /* sheep grazing inside pen                 */
    PAIR_SHEEP_FLEE  = 3,   /* sheep panicking from dog or scatter      */
    PAIR_DOG         = 4,   /* the border collie                        */
    PAIR_HUD         = 8,   /* bright yellow — top status               */
    PAIR_HINT        = 9,   /* bright cyan   — bottom key hint          */

    SHEEP_MIN        = 5,
    SHEEP_MAX        = 80,
    SHEEP_DEFAULT    = 30,
    SHEEP_STEP       = 5,
};

/* Cell dimensions — physics is in pixel space (CELL_W × CELL_H sub-pixels
 * per terminal cell) so motion is isotropic regardless of the terminal's
 * taller-than-wide cells. */
#define CELL_W   8
#define CELL_H  16

/* ── pen geometry ────────────────────────────────────────────────────── */
/*
 * PEN_RADIUS_FRAC — pen radius as a fraction of min(world_w, world_h).
 *   0.18 → on an 80x24 terminal (640x384 px), pen radius ≈ 69 px ≈ 9
 *   columns / 4.3 rows.  Big enough to contain 30 sheep with comfortable
 *   spacing; small enough that the dog has visible room outside it.
 *
 * PEN_TOLERANCE  — extra slack added to pen_radius before a sheep counts
 *   as "outlier" (i.e. dog should COLLECT it).  16 px ≈ 2 columns:
 *   sheep teetering exactly on the boundary do not trigger a fresh
 *   collect cycle.
 */
#define PEN_RADIUS_FRAC  0.18f
#define PEN_TOLERANCE   16.0f

/* ── compound shapes (u / v keys cycle through the pen geometry) ────── *
 *
 * Default is CIRCLE (the original pen).  Pressing `u` advances to the
 * next shape; `v` cycles back.  All shapes are REGULAR POLYGONS
 * inscribed in the unit circle (vertices at distance 1 from origin) in
 * normalized [-1, +1] × [-1, +1] coordinates, scaled at runtime by
 * pen_radius and centred on pen_centre.
 *
 * Cycle:  CIRCLE → SQUARE → TRIANGLE → HEXAGON → OCTAGON → CIRCLE
 *
 * Inscribed-polygon area as a fraction of the bounding circle area:
 *
 *   shape    sides   area / πr²    relative roominess
 *   CIRCLE     ∞       1.000        100 %   (easiest — uniform curvature)
 *   SQUARE     4       0.637         64 %
 *   TRIANGLE   3       0.413         41 %   (peak difficulty — narrow
 *                                            vertex tips trap sheep)
 *   HEXAGON    6       0.827         83 %
 *   OCTAGON    8       0.900         90 %   (nearly circular again)
 *
 * The progression rises to peak difficulty at TRIANGLE then RELAXES
 * back toward circle through HEXAGON and OCTAGON — a "challenge wave"
 * rather than a monotonic ramp.
 *
 * All regular polygons are CONVEX, so point-in-shape uses a single
 * cross-product test per edge.  Drawing uses the grid-boundary scan
 * (cell is on the outline iff it has at least one neighbour with the
 * opposite inside/outside status).
 */
typedef enum {
    COMPOUND_CIRCLE = 0,
    COMPOUND_SQUARE,
    COMPOUND_TRIANGLE,
    COMPOUND_HEXAGON,
    COMPOUND_OCTAGON,
    COMPOUND_COUNT
} CompoundShape;

/*
 * CompoundDef — a regular polygon by side count + rotation offset.
 *
 * The N vertices of a regular polygon inscribed in the unit circle sit
 * at angles  θ_k = rotation_offset_rad + k · (2π / N),  k = 0..N−1.
 * Each vertex is therefore  (cos θ_k, sin θ_k)  in normalized coords.
 *
 * y-down convention (positive y = down on screen):
 *   angle   0      →  right     (+x)
 *   angle  +π/2    →  down      (+y on screen)
 *   angle  −π/2    →  up        (−y on screen)
 *
 * rotation_offset_rad is chosen per-shape so the polygon looks "right"
 * — square with horizontal/vertical edges, triangle / hexagon with a
 * vertex at the top, octagon with a flat edge at the top.
 *
 * Members
 *   name                 HUD label ("CIRCLE", "SQUARE", "TRIANGLE", ...).
 *   n_vertices           0 for the CIRCLE special case; 3-8 for the
 *                        regular polygons.
 *   rotation_offset_rad  per-shape rotation so the silhouette looks
 *                        natural on screen.
 *
 * References
 *   [10] O'Rourke, *Computational Geometry in C* §1.5 — regular
 *        polygon parameterisation by (N, rotation).  The cross-
 *        product point-in-polygon test in §4.5 compound_is_inside
 *        is from §2.4 of the same text.
 */
typedef struct {
    const char *name;
    int         n_vertices;          /* 0 = circle (special case)        */
    float       rotation_offset_rad; /* angle of vertex 0 from +x axis   */
} CompoundDef;

/*
 * COMPOUND_DEFS — the 5 shapes the user can cycle through with u / v.
 *
 *   CIRCLE   : special case; n_vertices = 0, distance-from-centre test
 *   SQUARE   : 4 verts at ±π/4, ±3π/4 → axis-aligned edges
 *   TRIANGLE : 3 verts, vertex 0 at −π/2 → vertex on top ("▲")
 *   HEXAGON  : 6 verts, vertex 0 at −π/2 → vertex on top
 *   OCTAGON  : 8 verts, vertex 0 at −π/2 + π/8 → flat edge on top
 */
static const CompoundDef COMPOUND_DEFS[COMPOUND_COUNT] = {
    [COMPOUND_CIRCLE]   = { "CIRCLE",   0,  0.0f },
    [COMPOUND_SQUARE]   = { "SQUARE",   4,  (float)M_PI / 4.0f },
    [COMPOUND_TRIANGLE] = { "TRIANGLE", 3, -(float)M_PI / 2.0f },
    [COMPOUND_HEXAGON]  = { "HEXAGON",  6, -(float)M_PI / 2.0f },
    [COMPOUND_OCTAGON]  = { "OCTAGON",  8, -(float)M_PI / 2.0f + (float)M_PI / 8.0f },
};


/* ── sheep speeds (px/s) ─────────────────────────────────────────────── */
/*
 * SHEEP_GRAZE_SPEED — relaxed milling speed when no forces apply.
 * SHEEP_FLEE_SPEED  — speed scaling on flee force (note: this is the
 *                     desired-flee MAGNITUDE used in the formula, not
 *                     a hard cap; max actual speed is SHEEP_MAX_SPEED).
 * SHEEP_MAX_SPEED   — hard cap on |vel| each tick after force integration.
 *                     Set above SHEEP_FLEE_SPEED so flee can saturate
 *                     but below DOG_SPEED so the dog can outflank.
 */
#define SHEEP_GRAZE_SPEED   20.0f
#define SHEEP_FLEE_SPEED   140.0f
#define SHEEP_MAX_SPEED    180.0f

/* ── sheep flocking radii (px) ───────────────────────────────────────── */
/*
 * SHEEP_SEP_RADIUS — personal-space bubble.  At 24 px ≈ 3 cols, two
 *   adjacent sheep on the same row (8 px apart) are well inside it and
 *   gently push apart.
 *
 * SHEEP_COH_RADIUS — local cohesion sensing range.  At 96 px ≈ 12 cols,
 *   a sheep sees most of its flockmates without seeing the entire screen.
 *
 * DOG_FLEE_RADIUS  — sheep flee disc around the dog.  At 120 px ≈ 15
 *   cols, the dog's "scary aura" extends well past the pen tolerance,
 *   so sheep at the boundary still feel pressure.
 */
#define SHEEP_SEP_RADIUS    24.0f
#define SHEEP_COH_RADIUS    96.0f
#define DOG_FLEE_RADIUS    120.0f

/* ── sheep force weights ─────────────────────────────────────────────── */
/*
 * Order of magnitude reflects priority:
 *   FLEE   (2.6) overrides COH (0.4) so panic disperses, not clumps.
 *   SEP    (1.6) is firm enough to prevent overlap during the panic
 *                  but does not dominate FLEE.
 *   PEN_PULL (0.3) is the weakest — only relevant when outside the pen
 *                  AND the dog is far away.  It is a *gentle* nudge, not
 *                  an attractor that competes with the dog's repulsion.
 */
#define W_SHEEP_SEP    1.6f
#define W_SHEEP_COH    0.4f
#define W_SHEEP_FLEE   2.6f
#define W_PEN_PULL     0.3f

/* SHEEP_DAMPING — per-tick velocity scale.
 *   0.95 at 60 Hz → 0.95^60 ≈ 4.6 % retained after 1 second of zero force.
 *   That is, a sheep with no forces decays from 100 px/s to ~5 px/s in
 *   a second — looks like grazing, not a perpetual-motion glide. */
#define SHEEP_DAMPING  0.95f

/* ── scatter event ───────────────────────────────────────────────────── */
/*
 * SCATTER_IMPULSE — radial outward push (px/s) applied instantaneously
 *   to every sheep on SPACE.  220 px/s is well above SHEEP_FLEE_SPEED so
 *   the impulse is the dominant motion for the first ~0.5 s after press.
 *
 * SCATTER_JITTER  — angular randomisation per sheep, in radians.
 *   ±0.6 rad ≈ ±34° so sheep fan out radially rather than firing along
 *   parallel rays from the same direction.
 */
#define SCATTER_IMPULSE 220.0f
#define SCATTER_JITTER    0.6f

/* ── dog ─────────────────────────────────────────────────────────────── */
/*
 * DOG_SPEED              — dog's cruise speed.  Must exceed SHEEP_FLEE_SPEED
 *                          (140) so the dog can outflank a fleeing sheep
 *                          rather than chase from behind.
 *
 * DOG_APPROACH_OFFSET    — distance from sheep to dog when COLLECTing.
 *                          Too small (< sheep flee radius) and the sheep
 *                          panics in random directions because the flee
 *                          force saturates; too large and the dog takes
 *                          forever to influence the sheep.  60 px sits
 *                          comfortably inside DOG_FLEE_RADIUS (120 px).
 *
 * DOG_PATROL_OFFSET      — gap between pen ring and dog's patrol path.
 *                          40 px keeps the dog visible just outside the
 *                          ring without risking accidentally entering it.
 *
 * DOG_PATROL_OMEGA       — patrol angular speed (rad/s).
 *                          0.5 rad/s → ~12.6 s for one full lap; slow
 *                          and watchful, not frantic.
 */
#define DOG_SPEED              180.0f
#define DOG_APPROACH_OFFSET     60.0f
#define DOG_PATROL_OFFSET       40.0f
#define DOG_PATROL_OMEGA         0.5f

/* Continuous-chaos mode ('c' key): tiny random nudges per tick.
 * RANDOM_NUDGE in px/s², applied as Δv = RANDOM_NUDGE · dt each tick. */
#define RANDOM_NUDGE  120.0f

/* Timing primitives (verbatim from framework.c). */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/* clock_ns — monotonic wall-clock in nanoseconds.  CLOCK_MONOTONIC never
 * goes backward (unlike CLOCK_REALTIME), so dt = clock_ns() - prev is
 * always non-negative regardless of NTP or DST jumps. */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/* clock_sleep_ns — sleep exactly ns nanoseconds; ns ≤ 0 returns
 * immediately so the caller never has to bounds-check the budget. */
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

/*
 * color_init — register all ncurses colour pairs.
 *
 * Background = -1 (terminal default) so the demo respects the user's
 * theme.  All foreground tints sit in the bright half of the 256-colour
 * cube (≥ 33) per the project palette-brightness rule.  HUD pairs use
 * the canonical bright yellow + cyan combo regardless of theme.
 *
 *   PAIR_PEN_RING   220 — warm yellow ring (visible against grass/black)
 *   PAIR_SHEEP_CALM 255 — near-white grazing sheep
 *   PAIR_SHEEP_FLEE 196 — pure red panicking sheep
 *   PAIR_DOG         33 — dodger blue border collie
 *   PAIR_HUD        226 — bright yellow status
 *   PAIR_HINT        51 — bright cyan key hint
 */
/* palette_xterm256 — vivid xterm-256 indices for the 6 pen/sheep/dog/HUD
 * pairs.  All indices are in the BRIGHT half of the cube (≥ 33) per the
 * project palette-brightness rule so A_DIM stays legible. */
static void palette_xterm256(void)
{
    init_pair(PAIR_PEN_RING,    220, -1);  /* warm yellow ring         */
    init_pair(PAIR_SHEEP_CALM,  255, -1);  /* near-white grazing sheep */
    init_pair(PAIR_SHEEP_FLEE,  196, -1);  /* pure red panicking sheep */
    init_pair(PAIR_DOG,          33, -1);  /* dodger blue border collie */
    init_pair(PAIR_HUD,         226, -1);  /* bright yellow — top bar  */
    init_pair(PAIR_HINT,         51, -1);  /* bright cyan   — key hint */
}

/* palette_ansi8 — closest-equivalent ANSI 8-colour fallback. */
static void palette_ansi8(void)
{
    init_pair(PAIR_PEN_RING,    COLOR_YELLOW, -1);
    init_pair(PAIR_SHEEP_CALM,  COLOR_WHITE,  -1);
    init_pair(PAIR_SHEEP_FLEE,  COLOR_RED,    -1);
    init_pair(PAIR_DOG,         COLOR_CYAN,   -1);
    init_pair(PAIR_HUD,         COLOR_YELLOW, -1);
    init_pair(PAIR_HINT,        COLOR_CYAN,   -1);
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) palette_xterm256();
    else               palette_ansi8();
}

/* ===================================================================== */
/* §4  coords + Vec2                                                      */
/* ===================================================================== */

/*
 * Two coordinate spaces:
 *   PIXEL space — physics; isotropic; 1 unit = 1 sub-pixel.
 *   CELL space  — terminal rows/cols where drawing happens.
 * Only the draw step converts pixel→cell via px_to_cell_x/y.
 */
static inline float pw(int cols) { return (float)(cols * CELL_W); }
static inline float ph(int rows) { return (float)(rows * CELL_H); }

/*
 * px_to_cell_x/y — round to nearest cell.  Adding 0.5 before flooring is
 * "round half up" — deterministic and symmetric, unlike roundf() which
 * uses banker's rounding and can flicker on exact half-boundaries.
 */
static inline int px_to_cell_x(float px) { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int px_to_cell_y(float py) { return (int)floorf(py / (float)CELL_H + 0.5f); }

/*
 * Vec2 — 2-D float vector used everywhere physics or geometry shows up.
 *
 * Intent
 *   Position, velocity, acceleration, force, offset, centroid — every
 *   quantity with an x and a y is a Vec2.  Functions return Vec2 BY
 *   VALUE (no out-params); the v2* helpers below compose into terse
 *   chains that read like the math notation they implement:
 *
 *      Vec2 force = v2add(v2scale(sep, W_SHEEP_SEP),
 *                         v2scale(coh, W_SHEEP_COHESION));
 *
 *   reads as `force = W_SEP·sep + W_COH·coh` to a maths-fluent reader.
 *
 * Why a struct (not float[2] or two scalars)
 *   • Named type makes signatures read in the algorithm's vocabulary:
 *     `sheep_flee_dog(sheep_pos, dog_pos)` rather than
 *     `sheep_flee_dog(sheep_x, sheep_y, dog_x, dog_y)`.
 *   • Return by value works without spilling to memory on x86-64 +
 *     ARM ABIs (8 bytes = one register pair); no perf penalty.
 *   • Eliminates two-out-parameter idioms in every steering helper.
 *
 * Members
 *   x, y    Cartesian components in PIXEL space (units = px).  Cell-
 *           space conversion happens only inside scene_draw via
 *           px_to_cell_x/y.  Physics never sees cells.
 *
 * References
 *   [3] Reynolds 1999 — every steering primitive returns a Vec2
 *       force; this file's §5 mirrors that API shape.
 *   [8] Shiffman, *Nature of Code* Ch.1 — PVector pedagogy that
 *       this type is the C analogue of.
 */
typedef struct { float x, y; } Vec2;

static inline Vec2  v2(float x, float y)        { return (Vec2){x, y}; }
static inline Vec2  v2add(Vec2 a, Vec2 b)        { return v2(a.x+b.x, a.y+b.y); }
static inline Vec2  v2sub(Vec2 a, Vec2 b)        { return v2(a.x-b.x, a.y-b.y); }
static inline Vec2  v2scale(Vec2 v, float s)     { return v2(v.x*s, v.y*s); }
static inline float v2len(Vec2 v)                { return sqrtf(v.x*v.x + v.y*v.y); }

/* v2norm — unit vector; returns zero vector if input is near-zero so
 * coincident-point divides never blow up. */
static inline Vec2 v2norm(Vec2 v)
{
    float l = v2len(v);
    return (l > 0.001f) ? v2scale(v, 1.0f/l) : v2(0, 0);
}

/* v2clamp_len — cap magnitude to max_len, preserving direction. */
static inline Vec2 v2clamp_len(Vec2 v, float max_len)
{
    float l = v2len(v);
    return (l > max_len) ? v2scale(v2norm(v), max_len) : v;
}

/* randf — uniform float in [0, 1].  Used for sheep spawn jitter and
 * the scatter angle perturbation. */
static float randf(void) { return (float)rand() / (float)RAND_MAX; }

/* ===================================================================== */
/* §4.5  world + compound — pixel-space extents and shape-agnostic       */
/*                          pen queries                                   */
/* ===================================================================== */

/*
 * World — pixel-space extent of the simulation box.
 *
 * Intent
 *   Every physics + render routine needs "the size of the world" for
 *   boid wrap, sheep clamp, dog clamp, and resize.  Before this struct,
 *   sheep + dog clamp loops carried `world_w` and `world_h` as separate
 *   floats; bundling makes their joint role explicit:
 *
 *     bound_to_world(pos, world)   reads better than
 *     bound_to_world(pos, ww, wh)
 *
 * Members
 *   width   x-extent in pixels = cols × CELL_W (CELL_W = 8 sub-px/col).
 *   height  y-extent in pixels = rows × CELL_H (CELL_H = 16 sub-px/row,
 *           double width to compensate for terminal cell aspect ratio).
 *
 * Invariants
 *   width > 0  AND  height > 0   (set at scene_init via pw/ph).
 */
typedef struct {
    float width;
    float height;
} World;

/*
 * world_from_terminal — derive the pixel-space World from a terminal
 * (cols, rows).  Called at scene_init and on every SIGWINCH so the
 * physics box matches the visible terminal extent.
 */
static inline World world_from_terminal(int cols, int rows)
{
    return (World){ .width = pw(cols), .height = ph(rows) };
}

/*
 * Compound — a placed pen instance in world coordinates.
 *
 * Intent
 *   Bundles the THREE values every sheep/dog/renderer routine needs:
 *
 *     shape           : which polygon (or circle) is active
 *     centre          : world-pixel position of the geometric centre
 *     bounding_radius : world-pixel half-size of the bounding box;
 *                       for CIRCLE this is the literal radius, for
 *                       polygons it's the circumradius (vertices sit
 *                       exactly at this distance from centre).
 *
 *   Before this struct, every signature carried these three as separate
 *   arguments — sheep_pen_pull, sheep_spawn_in_pen, dog_decide_target,
 *   compound_is_inside, compound_distance_outside, draw_pen_or_compound
 *   were all 7-or-more parameter calls.  Bundling makes the API:
 *
 *     sheep_pen_pull(pos, &scene->compound)
 *
 *   instead of:
 *
 *     sheep_pen_pull(pos, scene->pen_centre, scene->pen_radius,
 *                    scene->compound)
 *
 * Members
 *   shape           one of CompoundShape — selects the circle branch or
 *                   polygon edge tests in the helpers.
 *   centre          geometric centre in PIXEL space.  World centre at
 *                   scene_init; recomputed on resize.
 *   bounding_radius half-size of the bounding box in pixels.  Used as
 *                   the dog's patrol-orbit radius and as the scale for
 *                   normalising positions into [-1, +1] coords.
 *
 * Invariants
 *   0 ≤ shape < COMPOUND_COUNT.
 *   bounding_radius > 0.
 *   centre lies inside [0, World.width) × [0, World.height).
 *
 * References
 *   [1]  Strömbom et al. 2014 — the "target region" the dog herds
 *        sheep into.  Their paper assumes a convex target; all five
 *        of this file's shapes (circle + regular polygons) are
 *        convex, so the COLLECT heuristic transfers without change.
 *   [10] O'Rourke, *Computational Geometry in C* — point-in-convex-
 *        polygon and point-to-segment-distance algorithms used by
 *        the helpers that read this struct.
 */
typedef struct {
    CompoundShape shape;
    Vec2          centre;
    float         bounding_radius;
} Compound;

/*
 * compound_normalise — convert a world-pixel position into [-1, +1]
 * normalized coordinates relative to (centre, scale).  Centre maps to
 * (0, 0); points on the bounding-box edge map to ±1.
 */
static inline Vec2 compound_normalise(Vec2 pos, Vec2 centre, float scale)
{
    return v2((pos.x - centre.x) / scale, (pos.y - centre.y) / scale);
}

/*
 * polygon_vertex — compute the k-th vertex (in normalized coords) of a
 * regular polygon inscribed in the unit circle.
 *
 *   θ_k = rotation_offset + k · (2π / N)
 *   v_k = (cos θ_k, sin θ_k)
 */
static inline Vec2 polygon_vertex(const CompoundDef *def, int k)
{
    float angle = def->rotation_offset_rad
                + (float)k * 2.0f * (float)M_PI / (float)def->n_vertices;
    return v2(cosf(angle), sinf(angle));
}

/*
 * point_to_segment_distance — Euclidean distance from point p to the
 * closest point on segment a→b.  Used by compound_distance_outside.
 *
 *   t   = clamp((p − a) · (b − a) / |b − a|² , 0, 1)
 *   q   = a + t · (b − a)              [closest point on segment]
 *   d   = |p − q|
 */
static float point_to_segment_distance(Vec2 p, Vec2 a, Vec2 b)
{
    Vec2  ab          = v2sub(b, a);
    Vec2  ap          = v2sub(p, a);
    float ab_len_sq   = ab.x * ab.x + ab.y * ab.y;
    if (ab_len_sq < 1e-12f) return v2len(v2sub(p, a));      /* degenerate */
    float t = (ap.x * ab.x + ap.y * ab.y) / ab_len_sq;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    Vec2 closest = v2add(a, v2scale(ab, t));
    return v2len(v2sub(p, closest));
}

/*
 * compound_is_inside — true if `pos` lies inside the placed compound `c`.
 *
 *   CIRCLE  :  |pos − c.centre| < c.bounding_radius
 *   polygon :  cross-product test on every edge — all crosses have the
 *              SAME SIGN iff the point is on the same side of every
 *              edge, which (for a convex polygon) means inside.
 *
 * Used by sheep_pen_pull (decides when the inward force kicks in),
 * scene_step_sheep (in_pen_count for HUD), and the dog's outlier check.
 */
static bool compound_is_inside(const Compound *c, Vec2 pos)
{
    const CompoundDef *def = &COMPOUND_DEFS[c->shape];

    /* CIRCLE special case: trivial distance test */
    if (def->n_vertices == 0) {
        return v2len(v2sub(pos, c->centre)) < c->bounding_radius;
    }

    /* Convex polygon: same-sign cross product across every edge.
     *   for each edge v_k → v_{k+1}:
     *       cross = edge.x · (p − v_k).y − edge.y · (p − v_k).x
     *   if any cross flips sign, p is on the wrong side of that edge */
    Vec2 n = compound_normalise(pos, c->centre, c->bounding_radius);
    int  sign = 0;                     /* 0 = unset, +1 / -1 = seen */
    for (int k = 0; k < def->n_vertices; k++) {
        Vec2 v0   = polygon_vertex(def, k);
        Vec2 v1   = polygon_vertex(def, (k + 1) % def->n_vertices);
        Vec2 edge = v2sub(v1, v0);
        Vec2 to_p = v2sub(n, v0);
        float cross = edge.x * to_p.y - edge.y * to_p.x;
        if      (cross > 1e-6f) { if (sign == -1) return false; sign = +1; }
        else if (cross < -1e-6f) { if (sign == +1) return false; sign = -1; }
    }
    return true;
}

/*
 * compound_distance_outside — world-pixel distance from `pos` to the
 * nearest point on the compound boundary.  Returns 0 when pos is inside.
 *
 *   CIRCLE  : max(0, |pos − c.centre| − c.bounding_radius)
 *   polygon : 0 if inside; otherwise the EUCLIDEAN distance to the
 *             closest edge (each edge is a line segment, closest point
 *             is either a projection or an endpoint).
 *
 * Used by dog_decide_target to find the "worst outlier" — the sheep
 * the dog should currently chase.
 */
static float compound_distance_outside(const Compound *c, Vec2 pos)
{
    const CompoundDef *def = &COMPOUND_DEFS[c->shape];

    /* CIRCLE special case */
    if (def->n_vertices == 0) {
        float distance_from_centre = v2len(v2sub(pos, c->centre));
        return (distance_from_centre > c->bounding_radius)
             ? distance_from_centre - c->bounding_radius : 0.0f;
    }

    /* Inside the polygon: zero distance.  Outside: minimum distance to
     * any edge (in normalised coords; scaled to pixels at the end). */
    if (compound_is_inside(c, pos)) return 0.0f;

    Vec2  n               = compound_normalise(pos, c->centre, c->bounding_radius);
    float best_norm_dist  = 1e9f;
    for (int k = 0; k < def->n_vertices; k++) {
        Vec2  v0 = polygon_vertex(def, k);
        Vec2  v1 = polygon_vertex(def, (k + 1) % def->n_vertices);
        float d  = point_to_segment_distance(n, v0, v1);
        if (d < best_norm_dist) best_norm_dist = d;
    }
    return best_norm_dist * c->bounding_radius;     /* normalised → world px */
}

/*
 * compound_name — HUD label for the active compound shape.
 */
static inline const char *compound_name(const Compound *c)
{
    return COMPOUND_DEFS[c->shape].name;
}

/*
 * compound_cycle_shape — advance the compound's shape by `dir` (+1 = next,
 * −1 = previous), wrapping over COMPOUND_COUNT.
 */
static inline void compound_cycle_shape(Compound *c, int dir)
{
    int next = ((int)c->shape + dir + COMPOUND_COUNT) % COMPOUND_COUNT;
    c->shape = (CompoundShape)next;
}

/* ===================================================================== */
/* §5  sheep                                                              */
/* ===================================================================== */

/*
 * Sheep — one prey agent.
 *
 * Intent
 *   The atomic unit of the herd.  A Sheep is a point particle with
 *   position + velocity that accumulates four steering forces each
 *   tick — separation, cohesion, flee-from-dog, pen-pull — and a
 *   render hint flag the renderer reads for "calm vs panic" glyph
 *   choice.
 *
 * Why no alignment force (unlike boids in flocking.c)
 *   Real sheep CLUSTER rather than STREAM.  The selfish-herd response
 *   under predator pressure [5] is "move toward the centre of the
 *   group", not "match the heading of your neighbours".  This file
 *   uses cohesion + flee + pen_pull only; alignment is intentionally
 *   omitted.  See sheep_step in §5 for the force composition.
 *
 * Why a `fleeing` render hint (not a recomputation at draw time)
 *   sheep_step already knows whether the flee force was non-zero
 *   (the dog was inside SHEEP_FLEE_RADIUS).  Caching that bit avoids
 *   a redundant distance test in scene_draw at no memory cost —
 *   bool fits the trailing struct padding.  scene_draw reads it to
 *   pick the glyph ('O' panic vs 'o' calm) and colour pair.
 *
 * Members
 *   pos          current position in PIXEL space (units = px).  Clamped
 *                to the world box by scene_step_sheep — sheep never
 *                wrap (the world is a closed paddock, not a torus).
 *
 *   prev_pos     position at start of this tick.  Snapshotted in
 *                scene_step_sheep BEFORE pos is integrated, so the
 *                renderer can lerp prev_pos → pos by sub-tick alpha
 *                (Fiedler [9]).
 *
 *   vel          velocity in PIXELS PER SECOND.  Integration rule:
 *                  vel += force·dt              [steering accumulator]
 *                  vel *= SHEEP_DAMPING         [per-tick drag]
 *                  vel  = clamp(vel, MAX_SPEED) [hard cap]
 *                  pos += vel·dt                [Euler integration]
 *                The damping is what makes sheep CLUSTER rather than
 *                oscillate — without it, opposing forces cause
 *                jittery limit cycles.
 *
 *   fleeing      true on this tick iff the dog is inside
 *                SHEEP_FLEE_RADIUS.  Set by sheep_step's flee branch;
 *                read only by scene_draw for visual cueing.
 *
 * Invariants
 *   0 ≤ pos.x < World.width AND 0 ≤ pos.y < World.height (clamped
 *   each tick — sheep cannot leave the paddock).
 *   |vel| ≤ SHEEP_MAX_SPEED after scene_step_sheep.
 *   fleeing is purely a render hint; physics doesn't read it.
 *
 * References
 *   [2] Reynolds 1987 — base flocking model; sheep here use a SUBSET
 *       (separation + cohesion only, no alignment).
 *   [3] Reynolds 1999 — the seek/flee formulation used by the four
 *       sheep steering primitives.
 *   [4] Couzin et al. 2002 — zones-of-attraction model justifying
 *       SHEEP_SEP_RADIUS < SHEEP_COH_RADIUS.
 *   [5] King et al. 2012 — empirical "selfish-herd" sheep clustering
 *       under threat; the design rationale for dropping alignment.
 *   [9] Fiedler 2014 — the prev_pos / alpha-lerp pattern.
 */
typedef struct {
    /* kinematic state — updated every tick */
    Vec2 pos;          /* current position, pixel space          */
    Vec2 prev_pos;     /* position at start of tick — alpha lerp */
    Vec2 vel;          /* velocity, px/s; capped at SHEEP_MAX_SPEED */

    /* render hint — set each tick by sheep_step's flee branch */
    bool fleeing;
} Sheep;

/*
 * sheep_spawn_in_pen — uniform random spawn inside the active compound
 * shape.  Rejection-samples a unit square at 0.85 × scale (leaving a
 * ~15 % safety margin) and checks `compound_is_inside`; samples that
 * miss the compound (e.g. fall into a letter's interior hole or
 * outside a letter stroke) are rejected and the loop retries.
 *
 * For circle: every accepted sample lies inside a 0.85·r disc.
 * For letters: only samples landing on a stroke are accepted, so the
 * sheep start packed into the letter silhouette from frame one.
 *
 * Spawn_tries caps the loop so a pathological compound can't hang.
 */
static void sheep_spawn_in_pen(Sheep *s, const Compound *c)
{
    const float SPAWN_INSET_FRAC = 0.85f;     /* keep off the boundary */
    const int   MAX_SPAWN_TRIES  = 256;
    const float r                = c->bounding_radius;

    Vec2 spawn_pos = c->centre;               /* fallback */
    for (int attempt = 0; attempt < MAX_SPAWN_TRIES; attempt++) {
        float dx = randf() * 2.0f - 1.0f;     /* uniform in [-1, 1] */
        float dy = randf() * 2.0f - 1.0f;
        Vec2 candidate = v2(c->centre.x + dx * r * SPAWN_INSET_FRAC,
                            c->centre.y + dy * r * SPAWN_INSET_FRAC);
        if (compound_is_inside(c, candidate)) {
            spawn_pos = candidate;
            break;
        }
    }

    s->pos      = spawn_pos;
    s->prev_pos = s->pos;

    /* Small initial graze velocity in a random direction so the spawn
     * doesn't look frozen on frame one. */
    float ang = randf() * 2.0f * (float)M_PI;
    s->vel    = v2(cosf(ang) * SHEEP_GRAZE_SPEED * 0.5f,
                   sinf(ang) * SHEEP_GRAZE_SPEED * 0.5f);
    s->fleeing = false;
}

/* ── sheep steering forces ──────────────────────────────────────────── *
 * Each force returns a Vec2 in px/s² (logical acceleration).  scene_tick
 * sums them with weights and integrates into vel.
 */

/*
 * sheep_separate — push this sheep away from each neighbour within
 * SHEEP_SEP_RADIUS.  Linear falloff: touching → strong push, edge → ~0.
 *
 * Why this shape: linear falloff makes sheep settle into a comfortable
 * spacing rather than a hard exclusion shell.  At inner-shell distances
 * the push is small enough that other forces (cohesion, pen pull) can
 * still influence the sheep.
 */
static Vec2 sheep_separate(const Sheep *flock, int n, int self)
{
    const float DIST_EPSILON = 0.001f;       /* zero-distance guard */
    Vec2 force = v2(0, 0);
    Vec2 me    = flock[self].pos;

    for (int i = 0; i < n; i++) {
        if (i == self) continue;

        /* away_offset : neighbour → self displacement (we push that way)
         * distance    : how far the neighbour is right now */
        Vec2  away_offset = v2sub(me, flock[i].pos);
        float distance    = v2len(away_offset);

        bool inside_personal_space = distance < SHEEP_SEP_RADIUS;
        bool well_separated        = distance > DIST_EPSILON;
        if (!inside_personal_space || !well_separated) continue;

        /* personal_space_intrusion ∈ (0,1]: 1 at d=0, 0 at d=SEP_RADIUS.
         * push_dir : unit vector pointing AWAY from the neighbour. */
        float personal_space_intrusion = (SHEEP_SEP_RADIUS - distance) / SHEEP_SEP_RADIUS;
        Vec2  push_dir     = v2norm(away_offset);
        Vec2  contribution = v2scale(push_dir,
                                     personal_space_intrusion * SHEEP_GRAZE_SPEED);
        force = v2add(force, contribution);
    }
    return force;
}

/*
 * sheep_cohere — gentle pull toward the mean position of neighbours
 * within SHEEP_COH_RADIUS.  Returned force has magnitude
 * SHEEP_GRAZE_SPEED so cohesion competes on a graze-scale, not a
 * panic-scale.
 *
 * Returns zero when the sheep is alone in its perception disc.
 */
static Vec2 sheep_cohere(const Sheep *flock, int n, int self)
{
    /* (1) sum positions of every neighbour inside the cohesion radius */
    Vec2 position_sum  = v2(0, 0);
    int  n_neighbours  = 0;
    Vec2 me            = flock[self].pos;
    for (int i = 0; i < n; i++) {
        if (i == self) continue;
        bool inside_cohesion_radius = v2len(v2sub(flock[i].pos, me)) < SHEEP_COH_RADIUS;
        if (inside_cohesion_radius) {
            position_sum = v2add(position_sum, flock[i].pos);
            n_neighbours++;
        }
    }
    if (n_neighbours == 0) return v2(0, 0);

    /* (2) centroid_of_neighbours = mean position; toward_centroid is the
     *     unit vector pointing from self toward the group centre */
    Vec2 centroid_of_neighbours = v2scale(position_sum, 1.0f / (float)n_neighbours);
    Vec2 toward_centroid        = v2norm(v2sub(centroid_of_neighbours, me));

    /* (3) graze-scale pull (Reynolds rule 3 with mass=1, no velocity match) */
    return v2scale(toward_centroid, SHEEP_GRAZE_SPEED);
}

/*
 * sheep_flee_dog — repulsion from the dog with linear falloff inside
 * DOG_FLEE_RADIUS, zero outside.  Magnitude scales toward
 * SHEEP_FLEE_SPEED at the wall (d → 0).
 *
 * Returns zero magnitude when the dog is outside the disc — the caller
 * uses |force| > 0 as the trigger to set sheep->fleeing for rendering.
 */
static Vec2 sheep_flee_dog(Vec2 sheep_pos, Vec2 dog_pos)
{
    const float DIST_EPSILON = 0.001f;

    /* away_from_dog : dog → sheep displacement; the direction the sheep
     *                 will be pushed (subtracted-form unit vector). */
    Vec2  away_from_dog    = v2sub(sheep_pos, dog_pos);
    float distance_to_dog  = v2len(away_from_dog);

    bool inside_panic_zone = distance_to_dog <= DOG_FLEE_RADIUS;
    bool well_separated    = distance_to_dog >  DIST_EPSILON;
    if (!inside_panic_zone || !well_separated) return v2(0, 0);

    /* panic_intensity ∈ (0, 1]: 1 right next to dog, 0 at the flee radius
     * (linear falloff makes the dog's influence feel like a smooth
     * pressure field rather than a hard wall). */
    float panic_intensity = (DOG_FLEE_RADIUS - distance_to_dog) / DOG_FLEE_RADIUS;
    Vec2  flee_dir        = v2norm(away_from_dog);
    return v2scale(flee_dir, panic_intensity * SHEEP_FLEE_SPEED);
}

/*
 * sheep_pen_pull — gentle inward force, zero when the sheep is inside
 * the pen.  Magnitude is SHEEP_GRAZE_SPEED (graze-scale, not panic) so
 * this never overrides the dog flee force; it just keeps sheep that
 * have wandered out from drifting forever.
 */
static Vec2 sheep_pen_pull(Vec2 sheep_pos, const Compound *c)
{
    /* Inside the active compound: no pull (sheep grazes freely). */
    if (compound_is_inside(c, sheep_pos)) return v2(0, 0);

    /* Outside: pull toward the compound centre at SHEEP_GRAZE_SPEED.
     * Even for non-circle compounds, the centre is a sensible pull
     * target — the dog handles the precise routing. */
    Vec2 inward = v2sub(c->centre, sheep_pos);
    if (v2len(inward) < 0.001f) return v2(0, 0);
    return v2scale(v2norm(inward), SHEEP_GRAZE_SPEED);
}

/* ===================================================================== */
/* §6  dog — Strömbom-style controller                                    */
/* ===================================================================== */

/*
 * DogMode — the Strömbom shepherd controller's two-state FSM.
 *
 * Intent
 *   Strömbom [1] showed that ONE binary decision ("is the worst
 *   outlier beyond tolerance?") is sufficient to drive a competent
 *   shepherding dog.  No path planning, no goal vector, no per-sheep
 *   targeting — the dog just answers that one question each tick and
 *   picks one of two trivial behaviours.
 *
 * State diagram
 *
 *                   any sheep outside compound by > PEN_TOLERANCE
 *      ┌────────┐ ──────────────────────────────────────────────► ┌─────────┐
 *      │ PATROL │                                                  │ COLLECT │
 *      └────────┘ ◄────────────────────────────────────────────── └─────────┘
 *                   all sheep inside compound + tolerance
 *
 *   PATROL  : every sheep is inside (compound + tolerance).  Walk a
 *             slow circle around the bounding box so the dog stays
 *             visible to the herd and reactive to any sheep that
 *             wanders out.  No threat to the flock — sheep graze.
 *
 *   COLLECT : at least one sheep is beyond (compound + tolerance).
 *             Position the dog directly outside the WORST OUTLIER
 *             relative to the compound centre.  The sheep, fleeing
 *             the dog radially, runs AWAY from the dog and TOWARD
 *             the compound — herding emerges from the geometry
 *             alone, without the dog needing a goal vector.
 *
 * Why a two-state FSM (not separate worker/idle behaviours)
 *   • Single binary check per tick: O(N) scan for worst outlier,
 *     compare to threshold.  No planning, no path search.
 *   • The COLLECT trick (stand on the far side of the sheep relative
 *     to the target) is the entire intelligence — Strömbom [1] proves
 *     it suffices for any convex target region with a single dog.
 *   • PATROL exists only so the dog has SOMETHING to do when the
 *     flock is settled — visual continuity, no algorithmic role.
 *
 * Members
 *   DOG_PATROL  = 0   default; orbit the compound waiting for work
 *   DOG_COLLECT = 1   chasing the worst outlier
 *
 * References
 *   [1] Strömbom et al. 2014 — the FSM and the "stand on the far
 *       side" heuristic in COLLECT.
 *   [6] Vaughan et al. 2000 — hardware analogue: a real robot using
 *       a similar collect-or-orbit decomposition.
 */
typedef enum {
    DOG_PATROL = 0,
    DOG_COLLECT,
} DogMode;

/*
 * Dog — one border collie.
 *
 * Intent
 *   The shepherd.  ONE Dog instance lives on Scene; each tick its
 *   controller (dog_decide_target) picks a target position, and
 *   dog_step steers toward that target at DOG_SPEED.  Mode + target
 *   sheep + patrol phase are READ by the HUD for status display but
 *   are NOT consumed by the physics — they're just the controller's
 *   trace for the user to inspect.
 *
 * Why all three states (target + mode + target_sheep + patrol_phase)
 *   in one struct
 *   • target is the physics input — dog_step reads it.
 *   • mode + target_sheep are the user-facing trace — HUD reads them.
 *   • patrol_phase is the PATROL state's only persistent variable
 *     (it accumulates between ticks so the orbit advances smoothly).
 *   Bundling all four lets the controller take ONE Dog* and read /
 *   write everything it needs.
 *
 * Why PATROL has a phase (not a position)
 *   The dog's PATROL position is recomputed each tick from
 *   (centre + r·(cos φ, sin φ)).  Storing φ (not x,y) means the orbit
 *   is exactly circular and survives a window resize cleanly — pos
 *   re-derives from the new world centre + new radius without any
 *   "where on the old orbit was the dog" reconciliation.
 *
 * Members
 *   ── kinematic state (rewritten every tick by dog_step) ──────────
 *   pos          current position in PIXEL space.  Source for the
 *                sheep flee force.
 *   prev_pos     position at start of this tick — for the renderer's
 *                alpha lerp.  Snapshotted in dog_step BEFORE pos is
 *                integrated.
 *   vel          velocity in px/s; magnitude is always exactly
 *                DOG_SPEED while the dog is moving (no acceleration
 *                model; dog snaps to the target direction).
 *
 *   ── controller output (rewritten every tick by dog_decide_target)
 *   target       world-pixel position the dog wants to be at this
 *                tick.  In COLLECT mode: outside the worst outlier.
 *                In PATROL mode: a point on the orbit at angle
 *                patrol_phase.
 *
 *   ── controller introspection (read by HUD, not by physics) ──────
 *   mode         current FSM state (PATROL or COLLECT).
 *   target_sheep index of the sheep being chased in COLLECT, or −1
 *                in PATROL.
 *   patrol_phase orbit angle in RADIANS.  Advances by
 *                DOG_PATROL_OMEGA·dt each PATROL tick.  Frozen during
 *                COLLECT so the orbit resumes from where it paused
 *                when the flock is regathered.
 *
 * Invariants
 *   target is always set (never uninitialised — see scene_init).
 *   target_sheep ∈ [0, n_sheep) IF mode == DOG_COLLECT, else −1.
 *   |vel| == 0 OR |vel| ≈ DOG_SPEED (dog teleports velocity to the
 *   target direction; no smoothing).
 *
 * References
 *   [1] Strömbom et al. 2014 — dog_decide_target implements the
 *       paper's COLLECT/PATROL algorithm verbatim.
 *   [3] Reynolds 1999 §"Pursue" — the dog's COLLECT positioning
 *       (target = sheep_pos + offset·unit(sheep − centre)) is a
 *       variant of Reynolds's "pursue" with an offset target.
 *   [6] Vaughan et al. 2000 — hardware analogue.
 *   [7] Lien et al. 2004 — earlier multi-shepherd CG implementation.
 */
typedef struct {
    /* kinematic state — written each tick by dog_step */
    Vec2 pos;
    Vec2 prev_pos;
    Vec2 vel;

    /* controller output: where the dog wants to be this tick */
    Vec2 target;

    /* controller introspection (read by HUD; physics doesn't use) */
    DogMode mode;
    int     target_sheep;   /* sheep index in COLLECT mode; -1 in PATROL */
    float   patrol_phase;   /* PATROL mode angle around compound (rad)   */
} Dog;

/*
 * dog_decide_target — Strömbom-style mode selection.
 *
 * Step 1: scan every sheep with compound_distance_outside; pick the
 *         WORST OUTLIER — the sheep with the largest "how far past the
 *         compound boundary" distance.  For CIRCLE this is equivalent
 *         to "furthest from centre minus radius"; for polygons it's
 *         min-distance to any edge (since polygons are convex).
 * Step 2: if that distance exceeds PEN_TOLERANCE, the dog must
 *         COLLECT this sheep.  Place the dog at
 *           target = sheep_pos
 *                  + DOG_APPROACH_OFFSET · (sheep − compound.centre) /
 *                                          |sheep − compound.centre|
 *         i.e. on the same line as compound_centre → sheep, but
 *         DOG_APPROACH_OFFSET further out.  The sheep, fleeing the
 *         dog radially, moves along that line toward the compound.
 * Step 3: otherwise, switch to PATROL.  Advance patrol_phase by
 *         DOG_PATROL_OMEGA · dt and place the dog at
 *           target = compound.centre
 *                  + (bounding_radius + DOG_PATROL_OFFSET) ·
 *                    (cos φ, sin φ)
 *
 * Reads only from flock[]; never writes back.
 */
static void dog_decide_target(Dog *d, const Sheep *flock, int n,
                              const Compound *c, float dt)
{
    /* Step 1: find the sheep that is FURTHEST OUTSIDE the compound (not
     * just furthest from centre — for a non-circle shape, "distance from
     * centre" doesn't tell us whether the sheep is in or out). */
    float worst_outside_dist = 0.0f;
    int   worst_i            = -1;
    for (int i = 0; i < n; i++) {
        float outside_dist = compound_distance_outside(c, flock[i].pos);
        if (outside_dist > worst_outside_dist) {
            worst_outside_dist = outside_dist;
            worst_i            = i;
        }
    }

    /* Step 2: COLLECT if any sheep is meaningfully outside the compound.
     *
     *   centre  →  sheep  →  DOG_STAND_HERE
     *   compound      worst-outlier      far-side-approach
     *
     * The dog stands DOG_APPROACH_OFFSET past the worst outlier along
     * the (centre → sheep) ray.  The sheep, fleeing the dog radially,
     * runs back toward the compound centre — Strömbom's [1] core trick. */
    if (worst_i >= 0 && worst_outside_dist > PEN_TOLERANCE) {
        d->mode         = DOG_COLLECT;
        d->target_sheep = worst_i;

        Vec2 centre_to_sheep = v2sub(flock[worst_i].pos, c->centre);
        Vec2 outward_dir     = v2norm(centre_to_sheep);
        Vec2 approach_offset = v2scale(outward_dir, DOG_APPROACH_OFFSET);
        d->target            = v2add(flock[worst_i].pos, approach_offset);
        return;
    }

    /* Step 3: all sheep home — patrol a circle around the compound's
     * bounding box.  The orbit stays circular for ANY compound shape
     * because bounding_radius is the same for all (and the patrol orbit
     * always sits OUTSIDE the polygon's outermost vertex). */
    d->mode         = DOG_PATROL;
    d->target_sheep = -1;
    d->patrol_phase += DOG_PATROL_OMEGA * dt;

    /* Polar (orbit_radius, patrol_phase) around the compound centre
     * → Cartesian patrol_target_pos */
    float orbit_radius      = c->bounding_radius + DOG_PATROL_OFFSET;
    Vec2  patrol_unit_dir   = v2(cosf(d->patrol_phase), sinf(d->patrol_phase));
    Vec2  orbit_offset      = v2scale(patrol_unit_dir, orbit_radius);
    d->target               = v2add(c->centre, orbit_offset);
}

/*
 * dog_step — move the dog one tick toward d->target at DOG_SPEED.
 *
 * No acceleration, no inertia: the dog snaps its velocity to point at
 * the target each tick.  This is acceptable because (a) the target
 * itself moves smoothly (sheep move smoothly, patrol phase is C¹),
 * and (b) it keeps the algorithm visibly "geometric" — the dog goes
 * where the algorithm tells it to go, no internal dynamics to debug.
 */
static void dog_step(Dog *d, float dt)
{
    const float ARRIVED_EPSILON = 0.001f;

    /* (1) point velocity at the controller's target.  If we're already
     *     on top of the target (rare — dog usually overshoots), vel = 0. */
    Vec2  to_target          = v2sub(d->target, d->pos);
    float distance_to_target = v2len(to_target);

    Vec2 desired_velocity;
    if (distance_to_target > ARRIVED_EPSILON) {
        Vec2 toward_target_dir = v2norm(to_target);
        desired_velocity       = v2scale(toward_target_dir, DOG_SPEED);
    } else {
        desired_velocity       = v2(0, 0);
    }

    /* (2) commit velocity; snapshot prev_pos for alpha-lerp; integrate */
    d->vel      = desired_velocity;
    d->prev_pos = d->pos;
    d->pos      = v2add(d->pos, v2scale(d->vel, dt));
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * Scene — pen + sheep pool + dog + world dimensions.
 *
 * Field groups:
 *   - pen geometry (set by scene_init / app_do_resize).
 *   - sheep pool (first n_sheep slots active; rest pre-spawned for
 *     instant '+' reveal).
 *   - the dog (singleton).
 *   - user mode flags (paused, continuous_chaos).
 *   - HUD-only derived state (in_pen_count, recomputed each tick).
 *   - world dimensions in pixels (refreshed each tick from cols/rows).
 */
/*
 * SimControls — user-facing playback knobs.
 *
 * Intent
 *   Two flags the input handler writes and the tick + HUD read.  Keeps
 *   app_handle_key's switch readable and the Scene's top level free
 *   for future state additions.
 *
 * Members
 *   paused           true → scene_tick early-returns; HUD shows "PAUSED".
 *                    Toggled by `p` / `P`.
 *   continuous_chaos true → scene_apply_chaos perturbs sheep every tick.
 *                    Toggled by `c` / `C`; HUD shows "CHAOS".
 */
typedef struct {
    bool paused;
    bool continuous_chaos;
} SimControls;

/*
 * Scene — owns ALL simulation state for one run.
 *
 * Layered ownership
 *
 *     Scene
 *       ├── sheep[SHEEP_MAX] : Sheep[]       ← agent pool (first n active)
 *       ├── n_sheep          : int           ← active prefix length
 *       ├── dog              : Dog           ← singleton shepherd
 *       ├── sim              : SimControls   ← paused + continuous_chaos
 *       ├── compound         : Compound      ← shape + centre + radius
 *       ├── world            : World         ← pixel-space extent
 *       └── in_pen_count     : int           ← HUD derived (refreshed/tick)
 *
 *   Every persistent simulation value is reachable from one Scene*.
 *   Terminal state (cols, rows, ncurses lifecycle) lives on Screen in §8;
 *   POSIX signal flags live on App in §8 (signal handlers can't receive
 *   a context pointer).
 *
 * Why this hierarchy
 *   • Replaceability: a future split-screen demo allocates multiple
 *     Scenes; file-scope globals would prevent that.
 *   • Reading aid: every helper takes `Scene *`, giving the reader ONE
 *     place to look for "what state exists".
 *   • The Compound sub-struct carries shape + centre + radius together
 *     so every sheep/dog/renderer takes &scene->compound instead of
 *     threading three flat fields.
 */
typedef struct {
    /* sheep pool — first `n_sheep` slots active */
    Sheep sheep[SHEEP_MAX];
    int   n_sheep;

    /* singleton shepherd */
    Dog   dog;

    /* user-facing knobs */
    SimControls sim;

    /* pen geometry (shape + centre + radius); refreshed on resize */
    Compound compound;

    /* pixel-space simulation extent; refreshed on resize */
    World world;

    /* HUD-only derived state — recomputed in scene_tick */
    int in_pen_count;
} Scene;

/*
 * scene_init — fresh scene with sheep scattered inside the active
 * compound and the dog parked at the east edge of the patrol orbit.
 *
 *   compound.centre          = world centre
 *   compound.bounding_radius = PEN_RADIUS_FRAC × min(w, h)
 *
 * Using min(w, h) (not max, not w·h) keeps the bounding box square
 * regardless of terminal aspect ratio.  compound.shape is preserved
 * across reset (so 'r' regathers into whatever shape the user picked
 * with u / v) — only the geometry is recomputed for the new world.
 */
static void scene_init(Scene *s, int cols, int rows)
{
    /* Preserve the active compound shape across reset; zero everything else. */
    CompoundShape saved_shape = s->compound.shape;
    memset(s, 0, sizeof *s);
    s->compound.shape = saved_shape;

    /* (1) world extent (cells → pixels) */
    s->world   = world_from_terminal(cols, rows);
    s->n_sheep = SHEEP_DEFAULT;

    /* (2) compound geometry: world centre, bounding radius = a fraction
     * of min(w, h) so the pen stays inside-screen on any aspect ratio */
    float min_dim = (s->world.width < s->world.height) ? s->world.width
                                                       : s->world.height;
    s->compound.centre          = v2(s->world.width  * 0.5f,
                                      s->world.height * 0.5f);
    s->compound.bounding_radius = min_dim * PEN_RADIUS_FRAC;

    /* (3) pre-spawn the entire pool so '+' reveals already-placed sheep
     * without stuttering while they randomise */
    for (int i = 0; i < SHEEP_MAX; i++)
        sheep_spawn_in_pen(&s->sheep[i], &s->compound);

    /* (4) dog starts at the east edge of the patrol circle */
    s->dog.pos          = v2(s->compound.centre.x
                              + s->compound.bounding_radius + DOG_PATROL_OFFSET,
                             s->compound.centre.y);
    s->dog.prev_pos     = s->dog.pos;
    s->dog.target       = s->dog.pos;
    s->dog.vel          = v2(0, 0);
    s->dog.mode         = DOG_PATROL;
    s->dog.target_sheep = -1;
    s->dog.patrol_phase = 0.0f;
}

/*
 * scene_scatter — apply an outward radial impulse to every sheep, with
 * per-sheep angular jitter so the herd fans out instead of firing along
 * straight rays.
 *
 * For each sheep:
 *   1. Compute outward direction = normalise(pos − pen_centre).
 *      If the sheep is at (or very near) the centre, fall back to a
 *      uniform random direction so divide-by-zero doesn't strand it.
 *   2. Rotate the direction by a random angle in [-SCATTER_JITTER,
 *      +SCATTER_JITTER] radians.
 *   3. Add SCATTER_IMPULSE × strength_mul × jittered_dir to vel.
 *
 * strength_mul: 1.0 for normal SPACE, 2.0 for 'S' mega-scatter.
 */
static void scene_scatter(Scene *s, float strength_mul)
{
    const float DIST_EPSILON = 0.001f;

    for (int i = 0; i < s->n_sheep; i++) {
        Sheep *sh = &s->sheep[i];

        /* Step 1: outward_dir from compound centre (or random fallback
         * for sheep that happen to be exactly on the centre).  Without
         * this fallback, a sheep at the centre would have outward=0 and
         * never get scattered. */
        Vec2  centre_to_sheep    = v2sub(sh->pos, s->compound.centre);
        float distance_to_centre = v2len(centre_to_sheep);
        Vec2  outward_dir;
        if (distance_to_centre > DIST_EPSILON) {
            outward_dir = v2norm(centre_to_sheep);
        } else {
            float random_heading_rad = randf() * 2.0f * (float)M_PI;
            outward_dir = v2(cosf(random_heading_rad), sinf(random_heading_rad));
        }

        /* Step 2: rotate outward_dir by a per-sheep random angle in
         * [−SCATTER_JITTER, +SCATTER_JITTER] using the 2-D rotation
         * matrix [[cos −sin], [sin cos]].  Without the jitter, sheep
         * would fly along radial spokes from the centre — the rotation
         * fans them out into a more natural panic spread. */
        float jitter_rad         = (randf() * 2.0f - 1.0f) * SCATTER_JITTER;
        float cos_j              = cosf(jitter_rad);
        float sin_j              = sinf(jitter_rad);
        Vec2  jittered_dir       = v2(outward_dir.x * cos_j - outward_dir.y * sin_j,
                                      outward_dir.x * sin_j + outward_dir.y * cos_j);

        /* Step 3: add SCATTER_IMPULSE · strength_mul · jittered_dir to vel.
         * strength_mul is 1.0 for SPACE, 2.0 for mega-scatter 'S'. */
        Vec2 impulse = v2scale(jittered_dir, SCATTER_IMPULSE * strength_mul);
        sh->vel      = v2add(sh->vel, impulse);
    }
}

/*
 * scene_apply_chaos — sprinkle a tiny random velocity nudge into every
 * sheep this tick.  Used when the user has toggled continuous-chaos
 * mode ('c' key); the nudge is small enough that a single tick is
 * imperceptible, but accumulated over hundreds of ticks the herd
 * never quite settles — the dog stays in COLLECT mode indefinitely,
 * which is the visual point of the mode.
 *
 * Magnitude per tick: |Δv| = RANDOM_NUDGE · dt = 120 · 1/60 = 2 px/s.
 * Direction: uniform random angle.
 */
static void scene_apply_chaos(Scene *s, float dt)
{
    for (int i = 0; i < s->n_sheep; i++) {
        float ang = randf() * 2.0f * (float)M_PI;
        float mag = RANDOM_NUDGE * dt;
        s->sheep[i].vel = v2add(s->sheep[i].vel,
                                v2(cosf(ang) * mag, sinf(ang) * mag));
    }
}

/*
 * scene_step_sheep — two-stage sheep update.
 *
 * The two-stage pattern is essential:
 *   Stage A reads ALL sheep positions, computes new_vel[i] for every i.
 *   Stage B writes new_vel back, integrates pos, clamps to world.
 * Without two stages, sheep[5] would react to sheep[0..4]'s NEW
 * positions but sheep[0..4] would react to sheep[5..29]'s OLD
 * positions — a subtle index-order drift visible over many ticks.
 *
 * Side effect: also sets each sheep's `fleeing` flag (rendering hint)
 * and counts how many sheep ended up inside the pen this tick (HUD).
 */
/*
 * sheep_total_force — compose the four weighted steering forces for one
 * sheep into a single force vector.  Also sets the sheep's `fleeing`
 * render hint as a side effect (only place that flag is written).
 *
 *   force =  W_SHEEP_SEP  · separate(self)
 *         +  W_SHEEP_COH  · cohere  (self)
 *         +  W_SHEEP_FLEE · flee_dog(self)
 *         +  W_PEN_PULL   · pen_pull(self)
 *
 * Reads OLD sheep positions only — caller is responsible for the two-
 * stage simultaneous update.
 */
static Vec2 sheep_total_force(Scene *s, int idx)
{
    const float FLEE_NONZERO_EPSILON = 0.001f;
    Sheep *sh = &s->sheep[idx];

    Vec2 sep_force  = sheep_separate (s->sheep, s->n_sheep, idx);
    Vec2 coh_force  = sheep_cohere   (s->sheep, s->n_sheep, idx);
    Vec2 flee_force = sheep_flee_dog (sh->pos, s->dog.pos);
    Vec2 pen_force  = sheep_pen_pull (sh->pos, &s->compound);

    /* fleeing flag: render hint for scene_draw; set iff dog is in
     * panic zone (flee_force has non-trivial magnitude). */
    sh->fleeing = v2len(flee_force) > FLEE_NONZERO_EPSILON;

    /* Weighted sum (Reynolds-style steering composition) */
    Vec2 force = v2(0, 0);
    force = v2add(force, v2scale(sep_force,  W_SHEEP_SEP));
    force = v2add(force, v2scale(coh_force,  W_SHEEP_COH));
    force = v2add(force, v2scale(flee_force, W_SHEEP_FLEE));
    force = v2add(force, v2scale(pen_force,  W_PEN_PULL));
    return force;
}

/*
 * sheep_integrate_kinematics — symplectic-Euler step for one sheep:
 *
 *   vel  += force · dt          (steering accumulator)
 *   vel  *= SHEEP_DAMPING       (per-tick drag — kills oscillations)
 *   vel   = clamp(vel, MAX)     (hard speed cap)
 *
 * Returns the new velocity (caller writes it back in stage B).
 */
static Vec2 sheep_integrate_kinematics(Vec2 vel, Vec2 force, float dt)
{
    Vec2 accumulated_vel = v2add(vel, v2scale(force, dt));
    Vec2 damped_vel      = v2scale(accumulated_vel, SHEEP_DAMPING);
    Vec2 capped_vel      = v2clamp_len(damped_vel, SHEEP_MAX_SPEED);
    return capped_vel;
}

/*
 * clamp_pos_to_world — keep a position inside [0, World.width) ×
 * [0, World.height).  Sheep can't escape the paddock — unlike a torus,
 * the boundary is solid (sheep can wander out via pen_radius +
 * tolerance, but the dog herds them back; the world boundary is an
 * absolute clamp that prevents off-screen sheep).
 */
static void clamp_pos_to_world(Vec2 *pos, World world)
{
    if (pos->x <  0.0f)         pos->x = 0.0f;
    if (pos->x >= world.width)  pos->x = world.width  - 1.0f;
    if (pos->y <  0.0f)         pos->y = 0.0f;
    if (pos->y >= world.height) pos->y = world.height - 1.0f;
}

static void scene_step_sheep(Scene *s, float dt)
{
    /* STAGE A: read OLD positions, compute every new_vel in one pass.
     * No sheep reacts to an already-moved neighbour. */
    Vec2 new_vel[SHEEP_MAX];
    for (int i = 0; i < s->n_sheep; i++) {
        Vec2 force = sheep_total_force(s, i);
        new_vel[i] = sheep_integrate_kinematics(s->sheep[i].vel, force, dt);
    }

    /* STAGE B: write velocities, integrate position, clamp; count in-pen. */
    int in_pen = 0;
    for (int i = 0; i < s->n_sheep; i++) {
        Sheep *sh    = &s->sheep[i];
        sh->vel      = new_vel[i];
        sh->prev_pos = sh->pos;
        sh->pos      = v2add(sh->pos, v2scale(sh->vel, dt));
        clamp_pos_to_world(&sh->pos, s->world);

        if (compound_is_inside(&s->compound, sh->pos)) in_pen++;
    }
    s->in_pen_count = in_pen;
}

/*
 * scene_step_dog — run the controller, integrate the dog, clamp to world.
 *
 * Decomposed out of scene_tick so the orchestrator stays linear and
 * each step's purpose is named.  The clamp at the end is a safety net
 * for the rare case where a single tick of DOG_SPEED could push the
 * dog past the screen edge (e.g. a chased sheep escapes to a corner).
 */
static void scene_step_dog(Scene *s, float dt)
{
    dog_decide_target(&s->dog, s->sheep, s->n_sheep, &s->compound, dt);
    dog_step(&s->dog, dt);
    clamp_pos_to_world(&s->dog.pos, s->world);
}

/*
 * scene_tick — one fixed-step physics update; thin orchestrator.
 *
 * Order matters:
 *   1. Continuous chaos (if enabled) — perturb sheep velocities.
 *   2. Dog: controller picks target, then dog steps.  Done BEFORE
 *      sheep step so sheep_flee sees the dog's NEW position
 *      (sheep flee from where the dog IS, not where it WAS).
 *   3. Sheep: two-stage update.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->sim.paused) return;

    if (s->sim.continuous_chaos) scene_apply_chaos(s, dt);
    scene_step_dog  (s, dt);
    scene_step_sheep(s, dt);
}

/* ── render ──────────────────────────────────────────────────────────── */

/*
 * mark_cell — stamp one ASCII glyph at terminal cell (cx, cy).
 *
 * Centralises the (chtype)(unsigned char) cast plus bounds-check that
 * would otherwise be repeated at every mvwaddch site.  The double cast
 * prevents sign-extension on character values > 127 (per CLAUDE.md
 * "Common ncurses Bugs").  Off-screen cells are silently dropped.
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
 * draw_pen — render a CIRCULAR pen boundary as a dashed ring.
 *
 * Sample N points evenly around the circle in PIXEL space, convert
 * each sample to a cell, alternate '*' and '.' by sample index for a
 * dashed look.  Drawing in pixel space (not cell space) means the
 * circle stays circular regardless of terminal aspect ratio — a
 * cell-space circle would appear as a vertically-squished oval.
 *
 * N samples chosen so adjacent samples are ~4 px apart along the arc:
 *   N = ceil(2π · r / 4)
 * For r = 69 px (default), N ≈ 109.
 */
static void draw_pen(WINDOW *w, Vec2 c, float r, int cols, int rows)
{
    int n = (int)ceilf(2.0f * (float)M_PI * r / 4.0f);
    if (n < 16) n = 16;
    for (int i = 0; i < n; i++) {
        float a   = (float)i / (float)n * 2.0f * (float)M_PI;
        float px  = c.x + r * cosf(a);
        float py  = c.y + r * sinf(a);
        char  ch  = (i & 1) ? '.' : '*';   /* alternate for dashed look */
        mark_cell(w, px_to_cell_x(px), px_to_cell_y(py),
                  ch, PAIR_PEN_RING, A_BOLD, cols, rows);
    }
}

/*
 * draw_compound — render the boundary of a NON-CIRCLE compound.
 *
 * Algorithm: for each terminal cell in (and just around) the compound's
 * bounding box, check whether that cell is INSIDE the compound; if its
 * inside/outside status differs from ANY 4-neighbour cell, it's a
 * BOUNDARY cell and we paint it.  This naturally draws the OUTER
 * outline of the rect union without drawing the interior edges where
 * rects meet — since at those interior edges both sides are "inside",
 * neither is a boundary cell.
 *
 * Alternates '*' and '.' for the same dashed look as draw_pen.
 */
static void draw_compound(WINDOW *w, const Compound *cmp,
                          int cols, int rows)
{
    /* Bounding box on the cell grid, with a 2-cell margin so we don't
     * miss the boundary right on the bounding-box edge. */
    enum { BB_MARGIN_CELLS = 2 };
    const float r = cmp->bounding_radius;
    const Vec2  c = cmp->centre;
    int cx_min = px_to_cell_x(c.x - r) - BB_MARGIN_CELLS;
    int cx_max = px_to_cell_x(c.x + r) + BB_MARGIN_CELLS;
    int cy_min = px_to_cell_y(c.y - r) - BB_MARGIN_CELLS;
    int cy_max = px_to_cell_y(c.y + r) + BB_MARGIN_CELLS;
    if (cx_min < 0)         cx_min = 0;
    if (cy_min < 0)         cy_min = 0;
    if (cx_max > cols - 1)  cx_max = cols - 1;
    if (cy_max > rows - 1)  cy_max = rows - 1;

    /* Cell-centre sampling: convert cell (cx, cy) to its centre in
     * pixel space, then test compound membership at that pixel. */
    int dash_step = 0;
    for (int cy = cy_min; cy <= cy_max; cy++) {
        for (int cx = cx_min; cx <= cx_max; cx++) {
            Vec2 here     = v2(cx * (float)CELL_W + CELL_W * 0.5f,
                               cy * (float)CELL_H + CELL_H * 0.5f);
            bool here_in  = compound_is_inside(cmp, here);

            /* Is THIS cell a boundary cell?  Yes iff any 4-neighbour is
             * on the opposite side of the compound boundary. */
            static const int NB_DX[4] = {-1, +1,  0,  0};
            static const int NB_DY[4] = { 0,  0, -1, +1};
            bool is_boundary = false;
            for (int k = 0; k < 4 && !is_boundary; k++) {
                Vec2 nb = v2(here.x + NB_DX[k] * (float)CELL_W,
                             here.y + NB_DY[k] * (float)CELL_H);
                if (compound_is_inside(cmp, nb) != here_in)
                    is_boundary = true;
            }

            if (is_boundary) {
                char ch = (dash_step++ & 1) ? '.' : '*';
                mark_cell(w, cx, cy, ch, PAIR_PEN_RING, A_BOLD, cols, rows);
            }
        }
    }
}

/*
 * draw_pen_or_compound — dispatcher: pick draw_pen for CIRCLE, otherwise
 * use the grid-boundary draw_compound.
 */
static void draw_pen_or_compound(WINDOW *w, const Compound *cmp,
                                 int cols, int rows)
{
    if (cmp->shape == COMPOUND_CIRCLE)
        draw_pen(w, cmp->centre, cmp->bounding_radius, cols, rows);
    else
        draw_compound(w, cmp, cols, rows);
}

/*
 * scene_draw — paint the frame in painter's order:
 *   1. Compound boundary (dashed ring for CIRCLE; grid-boundary scan
 *      for polygons — both via draw_pen_or_compound).
 *   2. Sheep at alpha-interpolated positions ('o' calm / 'O' fleeing).
 *   3. Dog last so it always reads above the sheep.
 *
 * alpha ∈ [0, 1) is the fixed-step accumulator's leftover fraction; it
 * lerps prev_pos → pos so motion stays smooth at any sim/render rate
 * combination.
 */
static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha)
{
    /* 1. compound boundary (circle ring OR polygon outline) */
    draw_pen_or_compound(w, &s->compound, cols, rows);

    /* 2. sheep */
    for (int i = 0; i < s->n_sheep; i++) {
        const Sheep *sh = &s->sheep[i];
        Vec2 dp = v2add(sh->prev_pos,
                        v2scale(v2sub(sh->pos, sh->prev_pos), alpha));

        char   ch   = sh->fleeing ? 'O' : 'o';
        int    pair = sh->fleeing ? PAIR_SHEEP_FLEE : PAIR_SHEEP_CALM;
        attr_t attr = sh->fleeing ? A_BOLD : A_NORMAL;
        mark_cell(w, px_to_cell_x(dp.x), px_to_cell_y(dp.y),
                  ch, pair, attr, cols, rows);
    }

    /* 3. dog */
    {
        const Dog *d = &s->dog;
        Vec2 dp = v2add(d->prev_pos,
                        v2scale(v2sub(d->pos, d->prev_pos), alpha));
        mark_cell(w, px_to_cell_x(dp.x), px_to_cell_y(dp.y),
                  '&', PAIR_DOG, A_BOLD, cols, rows);
    }
}

/* ===================================================================== */
/* §8  app — screen + signals + main loop                                 */
/* ===================================================================== */

/*
 * Screen — cell-space (terminal) extent + ncurses lifecycle wrapper.
 *
 * Intent
 *   Owns the TERMINAL side of the world.  Where Scene.world tracks the
 *   pixel-space simulation box, this struct tracks the cell-space
 *   terminal grid that ncurses paints onto.  The two are linked but
 *   live in DIFFERENT spaces:
 *
 *      World.width  = Screen.cols × CELL_W      (CELL_W = 8 px)
 *      World.height = Screen.rows × CELL_H      (CELL_H = 16 px)
 *
 *   Keeping them in SEPARATE structs prevents the entire class of
 *   "rendered too early in cell space" aspect-ratio bugs — physics
 *   never sees cells, the renderer never sees pixels except through
 *   the one px_to_cell_x/y bridge in §4.
 *
 * Why a tiny 2-field struct (not flat ints on App)
 *   • Lifecycle isolation: only screen_init / screen_resize /
 *     screen_free / screen_draw / screen_present touch ncurses'
 *     initscr / endwin / mvprintw.  They all take `Screen *` to
 *     make this layer explicit at the type level.
 *   • Symmetry with World: simulation and rendering each get one
 *     struct named for the space they live in.
 *
 * Members
 *   cols   terminal width in CELLS (read from getmaxyx).  Used by the
 *          renderer for cell-bounds checks after px → cell conversion,
 *          and by screen_draw for right-aligning the top HUD row.
 *   rows   terminal height in CELLS.  Same role on the vertical axis;
 *          bottom-row index is `rows - 1` (where the key hint paints).
 *
 * Invariants
 *   cols > 0 AND rows > 0 (set by screen_init via getmaxyx).
 *   Scene.world.width  == cols × CELL_W,
 *   Scene.world.height == rows × CELL_H,
 *   maintained in lockstep by scene_init / app_do_resize.
 *
 * References
 *   None directly — terminal extent is a rendering substrate concern.
 *   Aspect-ratio compensation (CELL_W vs CELL_H = 1:2) is project-
 *   specific; see CLAUDE.md §"Coordinate / Physics".
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);    /* getch returns ERR if no key — non-blocking */
    keypad(stdscr, TRUE);
    typeahead(-1);             /* prevent input polling from interrupting output */
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();                 /* force ncurses to re-read LINES/COLS */
    getmaxyx(stdscr, s->rows, s->cols);
}

static const char *DOG_MODE_NAMES[] = { "PATROL ", "COLLECT" };

/*
 * screen_draw — compose one frame: scene + HUD bars.
 *
 * Order matters: scene_draw first so HUD always over-stamps any glyph
 * that landed on the same cell.  Top-right uses PAIR_HUD (bright
 * yellow) with A_BOLD; bottom-left uses PAIR_HINT (bright cyan) with
 * A_BOLD.  The hint bar uses A_BOLD (NEVER A_DIM) so it stays readable
 * against any animation behind it (per the project HUD spec).
 */
/*
 * format_hud_status — write the top-row HUD text into `buf`.
 * Shows fps, sim rate, sheep count + compound label, dog mode + a
 * mode suffix (PAUSED / CHAOS / nothing).
 */
static void format_hud_status(const Scene *sc, double fps, int sim_fps,
                              char *buf, size_t buflen)
{
    const char *mode_suffix = sc->sim.paused           ? "  PAUSED"
                            : sc->sim.continuous_chaos ? "  CHAOS"
                                                       : "";
    snprintf(buf, buflen,
             " %5.1f fps  sim:%3d Hz  sheep:%d/%d in [%s]  dog:%s%s ",
             fps, sim_fps,
             sc->in_pen_count, sc->n_sheep,
             compound_name(&sc->compound),
             DOG_MODE_NAMES[sc->dog.mode],
             mode_suffix);
}

/* hud_paint_text — attron / mvprintw / attroff sandwich; both HUD rows
 * use this so the colour-pair setup isn't duplicated at each call site. */
static void hud_paint_text(int row, int col, int pair, const char *text)
{
    attron (COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* draw_hud_status — right-align the status text on row 0 */
static void draw_hud_status(const Screen *s, const Scene *sc,
                            double fps, int sim_fps)
{
    enum { HUD_TOP_ROW = 0 };
    char buf[HUD_COLS + 1];
    format_hud_status(sc, fps, sim_fps, buf, sizeof buf);

    int right_col = s->cols - (int)strlen(buf);
    if (right_col < 0) right_col = 0;
    hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, buf);
}

/* draw_hud_hint — bottom-row key bindings strip */
static void draw_hud_hint(const Screen *s)
{
    static const char *KEY_HINT =
        " q:quit  spc:scatter  S:mega-scatter  u/v:compound "
        "(CIRCLE→SQUARE→TRIANGLE→HEXAGON→OCTAGON)  c:chaos  "
        "p:pause  r:reset  +/-:sheep ";
    hud_paint_text(s->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha)
{
    /* (1) clear offscreen buffer */
    erase();
    /* (2) paint the simulation (pen + sheep + dog) */
    scene_draw(sc, stdscr, s->cols, s->rows, alpha);
    /* (3) HUD on top — status data + key hints */
    draw_hud_status(s, sc, fps, sim_fps);
    draw_hud_hint  (s);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── App + signal handlers ──────────────────────────────────────────── */

/*
 * FpsCounter — rolling-window frame-rate estimator.
 *
 * Intent
 *   Per-frame fps would jitter wildly (a single 1 ms variance swings the
 *   instantaneous reading by ~6 fps).  This struct accumulates
 *   frame_count + elapsed nanoseconds over a fixed window
 *   (FPS_UPDATE_MS = 500 ms) and emits a smoothed `display` figure each
 *   time the window fills, so the HUD reads a stable number.
 *
 * Lifecycle
 *   fps_counter_init  — zero everything at startup.
 *   fps_counter_tick  — call once per frame with frame dt (ns); emits
 *                       a fresh `display` only when the window fills.
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

/* fps_counter_tick — tally one frame; emit smoothed reading when window fills. */
static void fps_counter_tick(FpsCounter *f, int64_t dt)
{
    const int64_t FPS_WINDOW_NS = (int64_t)FPS_UPDATE_MS * NS_PER_MS;
    f->frame_count++;
    f->window_ns += dt;
    if (f->window_ns < FPS_WINDOW_NS) return;
    f->display = (double)f->frame_count
               / ((double)f->window_ns / (double)NS_PER_SEC);
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * App — top-level container for every persistent value.
 *
 * Owns:
 *   scene       — simulation state (sheep, dog, sim, compound, world)
 *   screen      — terminal cell extent + ncurses lifecycle
 *   fps         — rolling-window frame-rate estimator
 *   sim_fps     — target physics tick rate (config; not user-tweakable)
 *   running     — sig_atomic_t flag cleared by on_exit_signal
 *   need_resize — sig_atomic_t flag set by on_resize_signal
 *
 * Lives as a single file-scope `g_app` so signal handlers can reach
 * the flags through static storage.  Everything else is reachable via
 * `App *app` parameter — see flocking.c / murmuration.c App docblocks
 * for the broader rationale.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    FpsCounter            fps;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/*
 * app_do_resize — handle a pending SIGWINCH.
 *
 *   (1) screen_resize: tear ncurses down + up, re-read cols/rows
 *   (2) refresh Scene.world from the new terminal extent
 *   (3) recompute compound centre + bounding_radius (shape unchanged)
 *   (4) clamp every sheep and the dog into the new world box
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Scene *sc = &app->scene;

    /* (1) refresh world extent from the new terminal size */
    sc->world = world_from_terminal(app->screen.cols, app->screen.rows);

    /* (2) recompute compound centre + bounding radius from new world */
    float min_dim = (sc->world.width < sc->world.height) ? sc->world.width
                                                         : sc->world.height;
    sc->compound.centre          = v2(sc->world.width  * 0.5f,
                                       sc->world.height * 0.5f);
    sc->compound.bounding_radius = min_dim * PEN_RADIUS_FRAC;

    /* (3) clamp every active sheep into the new world bounds */
    World world = sc->world;
    for (int i = 0; i < sc->n_sheep; i++) {
        Sheep *sh = &sc->sheep[i];
        if (sh->pos.x >= world.width)  sh->pos.x = world.width  - 1.0f;
        if (sh->pos.y >= world.height) sh->pos.y = world.height - 1.0f;
        sh->prev_pos = sh->pos;
    }
    if (sc->dog.pos.x >= world.width)  sc->dog.pos.x = world.width  - 1.0f;
    if (sc->dog.pos.y >= world.height) sc->dog.pos.y = world.height - 1.0f;
    sc->dog.prev_pos = sc->dog.pos;

    app->need_resize = 0;
}

/*
 * app_handle_key — dispatch a single keypress; return false to quit.
 *
 *   q / Q / ESC    quit
 *   space          single scatter pulse
 *   S              mega-scatter (2x impulse)
 *   u              cycle compound forward  (CIRCLE → SQUARE → TRIANGLE → HEXAGON → OCTAGON)
 *   v              cycle compound backward (OCTAGON → HEXAGON → TRIANGLE → SQUARE → CIRCLE)
 *   c / C          toggle continuous-chaos sprinkle mode
 *   p / P          pause / resume
 *   r / R          reset (regather all sheep into the active compound)
 *   + / =          add SHEEP_STEP sheep (cap SHEEP_MAX)
 *   -              remove SHEEP_STEP sheep (floor SHEEP_MIN)
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':  scene_scatter(sc, 1.0f);                 break;
    case 'S':  scene_scatter(sc, 2.0f);                 break;
    /* compound cycle — sheep stay in place; dog naturally re-herds any
     * that fell outside the new shape, giving a live difficulty ramp */
    case 'u':  compound_cycle_shape(&sc->compound, +1); break;
    case 'v':  compound_cycle_shape(&sc->compound, -1); break;
    case 'c': case 'C': sc->sim.continuous_chaos = !sc->sim.continuous_chaos; break;
    case 'p': case 'P': sc->sim.paused           = !sc->sim.paused;           break;
    case 'r': case 'R':
        scene_init(sc, app->screen.cols, app->screen.rows);
        break;
    case '+': case '=':
        sc->n_sheep += SHEEP_STEP;
        if (sc->n_sheep > SHEEP_MAX) sc->n_sheep = SHEEP_MAX;
        break;
    case '-':
        sc->n_sheep -= SHEEP_STEP;
        if (sc->n_sheep < SHEEP_MIN) sc->n_sheep = SHEEP_MIN;
        break;
    default: break;
    }
    return true;
}

/*
 * main — the game loop.
 *
 *   (1) handle pending SIGWINCH resize + reset timers
 *   (2) measure dt since last frame, capped at DT_CAP_NS
 *   (3) drain fixed-step physics until caught up
 *   (4) sub-tick alpha for the renderer
 *   (5) rolling-window fps counter
 *   (6) sleep BEFORE render so terminal I/O stays inside FRAME_BUDGET_NS
 *   (7) draw + present
 *   (8) drain non-blocking input
 *
 * Frame cap uses `clock_ns() − frame_start` (NOT `... + dt`).  Adding
 * dt back into elapsed cancels the cap; sleep becomes 0 and CPU pegs.
 */
int main(void)
{
    /* (a) RNG + signal handlers + atexit */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    /* (b) bring up screen + scene + fps counter */
    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    fps_counter_init(&app->fps);

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    /* (c) loop-local constants */
    const int64_t DT_CAP_NS       = 100 * NS_PER_MS;             /* avalanche guard */
    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;     /* render cadence  */
    const int64_t TICK_LEN_NS     = TICK_NS(app->sim_fps);
    const float   TICK_LEN_SEC    = (float)TICK_LEN_NS / (float)NS_PER_SEC;

    int64_t frame_time = clock_ns();
    int64_t sim_accum  = 0;

    while (app->running) {
        int64_t frame_start = clock_ns();

        /* (1) handle pending resize: reload extent + reset timers */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* (2) measure dt since last frame, capped to prevent avalanche */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;

        /* (3) drain accumulator: fixed-step physics until caught up */
        sim_accum += dt;
        while (sim_accum >= TICK_LEN_NS) {
            scene_tick(&app->scene, TICK_LEN_SEC);
            sim_accum -= TICK_LEN_NS;
        }

        /* (4) sub-tick alpha for renderer */
        float alpha = (float)sim_accum / (float)TICK_LEN_NS;

        /* (5) rolling-window fps counter */
        fps_counter_tick(&app->fps, dt);

        /* (6) sleep BEFORE render so I/O stays inside the budget */
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
