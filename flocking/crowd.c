/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * crowd.c — six switchable steering-behaviour crowds in ASCII
 *
 * DEMO: A pool of up to 150 ASCII "people" move around the terminal under
 *       one of six crowd modes — wander, boids flock, panic-flee from a
 *       roaming '!' threat, gather to the centre, chain-follow the '@'
 *       leader, or queue at a right-edge service counter. Press 1-6 to
 *       cycle behaviours live.
 *
 * Study alongside: flocking/flocking.c (the same boids rules in isolation)
 *
 * Section map:
 *   §1 config   — tunables: counts, speeds, radii, weights, queue layout
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — palette_xterm256 / palette_ansi8 + PAIR_HUD/PAIR_HINT
 *   §4 coords   — pixel↔cell aspect-ratio bridge + Vec2 helpers + wrap /
 *                 bounce / clamp boundary primitives
 *   §5 entity   — Person struct, person_spawn, person_step (3-stage Euler),
 *                 person_at_target / person_pick_random_target
 *   §6 steering — steer_seek / steer_flee / steer_separate / steer_align /
 *                 steer_cohere force generators (named-intermediate math)
 *   §6.5        — blend2_forces / blend3_forces weighted-sum composer
 *   §7 scene    — Scene + sub-structs (Crowd, Threat, SimControls); shared
 *                 helpers (lerp_render_pos, gather/queue_approach_speed,
 *                 queue_slot_for_index, threat_advance); six tick_*
 *                 behaviours; scene_draw + draw_threat_marker /
 *                 draw_queue_counter / draw_follow_leader / draw_person
 *   §8 app      — POSIX signal flags, screen_init, draw_hud (hud_paint_text),
 *                 clamp_all_active_people + handle_resize_if_pending,
 *                 adjust_person_count + handle_input, FpsCounter
 *                 (init+tick), drain_sim_accumulator, main game loop
 *
 * Keys:
 *   q / ESC    quit                     space      pause / resume
 *   1          WANDER  (random targets) 2          FLOCK   (boids)
 *   3          PANIC   (flee threat)    4          GATHER  (to centre)
 *   5          FOLLOW  (chain leader)   6          QUEUE   (right-edge)
 *   + / =      add 5 people (max 150)   -          remove 5 (min 5)
 *   r          reset all positions
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra flocking/crowd.c -o crowd -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Reynolds steering behaviours. Each tick every agent
 *                  computes a weighted sum of force vectors — seek, flee,
 *                  separate, align, cohere — and integrates them into
 *                  velocity (capped at SPEED_MAX) then position. The
 *                  weighted sum is the entire intelligence: no rule
 *                  hierarchy, no FSM, no global planner. Six behaviours
 *                  share the same five primitive forces, with different
 *                  weights and target geometry per behaviour.
 *
 * Data-structure : Scene owns a fixed-capacity pool of Person (up to 150).
 *                  Only the first `count` people are simulated each tick;
 *                  the rest are pre-spawned so '+' can reveal them
 *                  instantly without re-randomising. Each Person holds
 *                  pos, prev_pos, vel, target, glyph, color pair index.
 *                  prev_pos is snapshotted before every tick so the
 *                  renderer can sub-tick lerp for smooth motion at any
 *                  render-rate / sim-rate combination. The PANIC threat
 *                  has its own pos/vel/target on Scene.
 *
 * Rendering      : Each frame, every active person is drawn at
 *                  alpha-interpolated `lerp(prev_pos, pos, alpha)` cells.
 *                  Per-behaviour decorations draw first (PANIC '!' threat,
 *                  QUEUE counter '|' + '>>|', FOLLOW leader '@') so they
 *                  sit underneath the bulk crowd render. A single
 *                  `mark_cell()` helper carries the (chtype)(unsigned char)
 *                  cast and bounds-check that every glyph stamp needs.
 *
 * Performance    : Steering is O(N²) — each person checks every other for
 *                  separation/align/cohere. At N=150 that is 22 500
 *                  distance checks per tick, trivial at 60 Hz. A spatial
 *                  hash would scale to thousands; not needed here.
 *
 * References
 * ──────────
 *   ── Original behavioural-steering papers ───────────────────────
 *   [1] Reynolds, C. W. (1987), "Flocks, Herds, and Schools: A
 *       Distributed Behavioral Model", SIGGRAPH '87 Computer Graphics
 *       21(4), pp. 25-34 — the ORIGINAL boids paper.  Introduces
 *       separation + alignment + cohesion as the three local rules
 *       sufficient to generate flocking; coins "boid" (bird-oid).
 *   [2] Reynolds, C. W. (1999), "Steering Behaviors for Autonomous
 *       Characters", Game Developers Conference 1999.  Online at
 *       https://www.red3d.com/cwr/steer/ — the canonical steering
 *       primer: seek, flee, arrive, pursue, evade, wander, separate,
 *       align, cohere, path-follow, obstacle-avoid.  This file's §6
 *       is a direct subset.
 *
 *   ── Statistical-physics flocking models ────────────────────────
 *   [3] Vicsek, T., Czirók, A., Ben-Jacob, E., Cohen, I. & Shochet, O.
 *       (1995), "Novel type of phase transition in a system of self-
 *       driven particles", Phys. Rev. Lett. 75(6), pp. 1226-1229 —
 *       the canonical Vicsek model: align-with-neighbours + noise.
 *       Establishes flocking as a continuous phase transition.
 *   [4] Toner, J. & Tu, Y. (1995, 1998), "Long-Range Order in a
 *       Two-Dimensional Dynamical XY Model: How Birds Fly Together"
 *       (PRL 75, pp. 4326-4329) + "Flocks, herds, and schools"
 *       (Phys. Rev. E 58, pp. 4828-4858) — hydrodynamic continuum
 *       theory of flocking; the macroscopic field theory behind the
 *       Vicsek microscopic model.
 *
 *   ── Crowd / pedestrian-dynamics models ─────────────────────────
 *   [5] Helbing, D. & Molnár, P. (1995), "Social force model for
 *       pedestrian dynamics", Phys. Rev. E 51(5), pp. 4282-4286 —
 *       the social-force model: desired velocity + repulsion from
 *       walls and neighbours.  More physically realistic than
 *       Reynolds steering for human crowds; this file's PANIC + QUEUE
 *       modes are simplifications of the same idea.
 *   [6] Helbing, D., Farkas, I. & Vicsek, T. (2000), "Simulating
 *       dynamical features of escape panic", Nature 407(6803),
 *       pp. 487-490 — applies the social-force model to evacuation;
 *       directly relevant to this file's PANIC behaviour.
 *
 *   ── Collective-behaviour biology ───────────────────────────────
 *   [7] Couzin, I. D., Krause, J., James, R., Ruxton, G. D. & Franks,
 *       N. R. (2002), "Collective memory and spatial sorting in
 *       animal groups", J. Theor. Biol. 218(1), pp. 1-11 — extends
 *       boids with radius-based zones of repulsion / alignment /
 *       attraction; explains how leadership and structure emerge.
 *
 *   ── Implementation-oriented textbooks ──────────────────────────
 *   [8] Shiffman, D., "The Nature of Code", Ch. 6 — reading-level
 *       walkthrough of Reynolds steering with Processing examples.
 *       The single most accessible introduction; pair with [2].
 *   [9] Millington, I. & Funge, J. (2019), "Artificial Intelligence
 *       for Games" (3rd ed.), CRC Press — Ch. 3 covers every steering
 *       primitive with pseudocode + integration math, alongside
 *       higher-level decision systems.
 *  [10] Buckland, M. (2004), "Programming Game AI by Example",
 *       Wordware — Ch. 3 implements the Reynolds steering library in
 *       C++ with the same five forces this file's §6 exposes.
 *
 *   ── Online quick reference ─────────────────────────────────────
 *  [11] Wikipedia "Boids" — https://en.wikipedia.org/wiki/Boids —
 *       concise statement of the three rules, history, and links to
 *       implementations in every major language.
 *
 *   ── Companion file in this project ─────────────────────────────
 *   See also: flocking/flocking.c — the same Reynolds rules in
 *     isolation (boids only, no behaviour switching).  Read it first
 *     if you want the three-rule story before the six-behaviour
 *     toolbox below.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Every agent is a particle with mass = 1, accumulating a steering force
 * each tick. The force is a weighted sum of direction vectors — "go
 * toward target", "go away from neighbour", "match neighbour velocity",
 * "go toward neighbour centre". Add force×dt to velocity, cap velocity,
 * add velocity×dt to position. There is no global goal, no top-down
 * scheduling — every behaviour is just a different recipe of weights.
 *
 * ALGORITHM IN STEPS  (per tick, per person)
 * ──────────────────────────────────────────
 *   1. force = 0
 *   2. force += W_seek      · steer_seek(pos, vel, target, speed)
 *   3. force += W_separate  · steer_separate(neighbours within SEP_RADIUS)
 *   4. force += W_align     · steer_align    (neighbours within ALIGN_RADIUS)   [FLOCK only]
 *   5. force += W_cohere    · steer_cohere   (neighbours within COHESION_RADIUS) [FLOCK only]
 *   6. vel  = clamp(vel + force·dt, SPEED_MAX)
 *   7. prev_pos = pos
 *   8. pos += vel · dt
 *   9. apply boundary: wrap (most modes) / bounce (PANIC) / clamp (QUEUE)
 *
 * KEY FORMULAS
 * ────────────
 *   Seek:        desired = normalise(target − pos) · speed
 *                force   = desired − vel
 *
 *   Flee:        force   = − seek(target=threat)
 *
 *   Separate:    for each neighbour with d < SEP_RADIUS:
 *                  strength = (SEP_RADIUS − d) / SEP_RADIUS
 *                  force   += normalise(pos − neighbour.pos) · strength · SPEED_BASE
 *
 *   Align:       v_avg   = mean(neighbour.vel : d < ALIGN_RADIUS)
 *                force   = v_avg − vel
 *
 *   Cohere:      p_avg   = mean(neighbour.pos : d < COHESION_RADIUS)
 *                force   = seek(target=p_avg)
 *
 *   Pixel→cell:  cx = round(px / CELL_W)    (CELL_W = 8)
 *                cy = round(py / CELL_H)    (CELL_H = 16)
 *
 * WORKED EXAMPLE  (defaults: 60 people, 200x60 terminal, 60 Hz)
 * ────────────────────────────────────────────────────────────
 *   World box       : 200 cols x 60 rows = 1600 x 960 pixels.
 *   Per tick (1/60 s):
 *     a person at SPEED_BASE (80 px/s) advances 80/60 ≈ 1.3 px,
 *     i.e. one new cell every ~6 ticks (~100 ms) — visibly smooth.
 *   Personal space  : SEP_RADIUS = 40 px = 5 columns. Two people
 *     in adjacent cells (8 px apart) are well inside the bubble:
 *     strength = (40−8)/40 = 0.8, so each pushes the other with
 *     0.8·SPEED_BASE = 64 px/s² of acceleration — they will be
 *     two cells apart within ~3 ticks.
 *   Steering cost   : separation alone is O(N²). At 60 people it
 *     is 60·59 = 3540 distance checks per tick, ~210 000 per
 *     second — microseconds total on any modern CPU.
 *   Boundary        : WANDER/FLOCK/GATHER/FOLLOW wrap at edges
 *     (a person leaving x = 1599 reappears at x = 0); PANIC
 *     bounces (vel.x flips sign at the wall) so the threat
 *     traps the crowd; QUEUE clamps so people stop at the line.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS + MENTAL MODEL above — read first as prose.
 *      Read flocking.c first for the three-rule story (separation +
 *      alignment + cohesion).  This file extends that into a LIBRARY
 *      of steering primitives composed by mode into six behaviours.
 *   2. §6 steering — a TOOLBOX of pure functions: seek, flee,
 *      separate, align, cohere.  Each is documented inline.
 *   3. §7 scene — six tick_* functions, one per behaviour.  Each
 *      composes 2-4 steering primitives with mode-specific weights.
 *   4. §5 entity — Person struct, integration step.
 *   5. §1-§4, §8 — config / clock / colour / coords / app loop.
 *      Skim if you've seen the framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   Person                 the agent. Holds pos, prev_pos, vel,
 *                          target, glyph, color.
 *   force                  Vec2 sum of weighted steering primitives.
 *   target                 per-Person Vec2 of "where to seek."
 *   threat                 PANIC mode's wandering '!' marker.
 *   leader                 FOLLOW mode's '@' marker, free-ranging.
 *   slot                   QUEUE mode's per-Person target position
 *                          on the staggered queue line.
 *   W_SEEK, W_FLEE,        per-behaviour weight constants. Tuning
 *   W_SEPARATE, W_ALIGN,   these is what makes one behaviour look
 *   W_COHERE               different from another.
 *
 * Background you need
 * ───────────────────
 *   - flocking.c T1-T3 (boid + three Reynolds rules + weight
 *     tuning).
 *   - Vec2 arithmetic.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - A* / pathfinding. Crowd uses purely REACTIVE steering, no
 *     planning ahead.
 *   - Social-force / pedestrian dynamics. Helbing (2000) and
 *     friends model crowd panic with sophisticated friction +
 *     desire forces; we use a simplified Reynolds toolkit.
 *   - Spatial hashing. O(N²) at N=150 is microseconds.
 *
 * ─────────────────────────────────────────────────────────────────────── */


#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config — every tunable constant lives here, nowhere else          */
/* ===================================================================== */

/*
 * Simulation and display rates.
 * SIM_FPS_DEFAULT controls how many physics steps run per second.
 * Raising it makes physics more precise but costs more CPU.
 * TARGET_FPS is the render cap — 60 is plenty for a terminal.
 */
enum {
    SIM_FPS_DEFAULT =  60,
    TARGET_FPS      =  60,

    HUD_COLS        =  72,   /* max chars in the status-bar string */
    FPS_UPDATE_MS   = 500,   /* how often the displayed fps refreshes */

    /*
     * Colour-pair IDs.  Pairs 1-7 paint the agents (one per band of the
     * rainbow); PAIR_HUD / PAIR_HINT paint the two HUD lines per the
     * project HUD spec.  Both are theme-independent so the HUD stays
     * readable against any animation backdrop.
     */
    N_COLORS        =   7,   /* agent palette: pairs 1-7 */
    PAIR_HUD        =   8,   /* bright yellow — top status bar  */
    PAIR_HINT       =   9,   /* bright cyan   — bottom key hint */

    CROWD_MIN       =   5,   /* fewest people allowed */
    CROWD_MAX       = 150,   /* pool size; also the hard cap */
    CROWD_DEFAULT   =  60,   /* people on startup */
    CROWD_STEP      =   5,   /* how many +/- adds or removes */
};

/*
 * Cell dimensions in pixels.
 * A typical terminal cell is 8 px wide × 16 px tall (ratio 1:2).
 * Physics positions are stored in pixel units so that moving at
 * the same speed in X and Y covers equal physical distance.
 * Change these only if your terminal has a different cell ratio.
 */
#define CELL_W   8    /* pixels per terminal column */
#define CELL_H  16    /* pixels per terminal row    */

/*
 * Steering neighbourhood radii (pixels).
 *
 * SEP_RADIUS      — personal-space bubble; neighbours inside get pushed away.
 *                   Smaller → people cluster tighter; larger → roomier crowd.
 * ALIGN_RADIUS    — boids only: match heading of neighbours within this.
 *                   Larger → bigger "flock units"; smaller → chaotic swarms.
 * COHESION_RADIUS — boids only: seek average position of neighbours within.
 *                   Must be > ALIGN_RADIUS for cohesion to pull strays back.
 */
#define SEP_RADIUS       40.0f
#define ALIGN_RADIUS     80.0f
#define COHESION_RADIUS 120.0f

/*
 * Speeds (pixels per second).
 *
 * SPEED_BASE    — relaxed cruising speed for wander/flock/gather/follow.
 *                 At 60fps a person moves SPEED_BASE/60 px per frame (~1.3 px).
 * SPEED_PANIC   — speed used by flee force; higher = more frantic escape.
 * SPEED_MAX     — hard cap: no agent can ever exceed this regardless of forces.
 *                 Prevents runaway acceleration when many forces stack.
 * THREAT_SPEED  — how fast the panic threat moves; slower than people so
 *                 the crowd can escape but must keep moving.
 */
#define SPEED_BASE    80.0f
#define SPEED_PANIC  140.0f
#define SPEED_MAX    160.0f
#define THREAT_SPEED  55.0f

/*
 * Steering force weights.
 * The final force = W_X * forceX + W_Y * forceY + ...
 * Increasing a weight makes that rule dominate.
 * W_SEP > W_SEEK keeps separation from being overridden by targeting.
 */
#define W_SEEK    1.0f   /* seek / flee multiplier                     */
#define W_FLEE    1.6f   /* flee weighted higher so panic feels urgent  */
#define W_SEP     1.8f   /* separation weighted highest — no overlap    */
#define W_ALIGN   1.0f   /* boids alignment                             */
#define W_COHERE  0.8f   /* boids cohesion (softer than alignment)      */

/*
 * Arrival thresholds (pixels).
 *
 * ARRIVE_DIST        — a person is "close enough" to their target when
 *                      within this radius.  They pick a new target then.
 *                      ~3 cells wide at the default CELL_W=8.
 * GATHER_SLOW_RADIUS — GATHER mode slows people down when this close to
 *                      the centre so they mill around rather than overshooting.
 * QUEUE_SLOW_RADIUS  — similar slow-down when near a queue slot.
 * QUEUE_IDLE_SPEED   — shuffle speed once nearly at slot position.
 */
#define ARRIVE_DIST         24.0f
#define GATHER_SLOW_RADIUS  (ARRIVE_DIST * 4.0f)   /* 96 px ≈ 12 cells  */
#define QUEUE_SLOW_RADIUS   (ARRIVE_DIST * 2.0f)   /* 48 px ≈  6 cells  */
#define QUEUE_IDLE_SPEED    (SPEED_BASE  * 0.3f)   /* 24 px/s — shuffle */

/*
 * Queue layout geometry.
 * Slots extend leftward from the service counter in a 3-row stagger.
 * QUEUE_SLOT_W — horizontal gap between slots (1.5 cells apart).
 * QUEUE_SLOT_H — vertical stagger between the 3 rows (1.5 cells apart).
 */
#define QUEUE_SLOT_W  ((float)CELL_W  * 1.5f)
#define QUEUE_SLOT_H  ((float)CELL_H  * 1.5f)

/* Timing helpers */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                             */
/* ===================================================================== */

/*
 * clock_ns — monotonic wall-clock in nanoseconds.
 * CLOCK_MONOTONIC never jumps backward (unlike CLOCK_REALTIME which can
 * shift on NTP updates), making it safe for dt measurement.
 */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/*
 * clock_sleep_ns — sleep for exactly ns nanoseconds.
 * Called BEFORE the render step so that terminal I/O time does not
 * eat into the frame budget (see §8 main loop for the reasoning).
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
/* §3  color                                                             */
/* ===================================================================== */

/*
 * color_init — define agent and HUD ncurses colour pairs.
 *
 * Background = -1 (terminal default) so the demo respects the user's
 * theme.  All foreground tints sit in the bright half of the 256-colour
 * cube (≥ 33) per the project palette-brightness rule.
 *
 *   Agent palette (pairs 1-7):
 *     1=red  2=orange  3=yellow  4=green  5=cyan  6=blue  7=magenta
 *
 *   HUD pairs (per project spec):
 *     PAIR_HUD  (8)  bright yellow 226 — top status bar, A_BOLD
 *     PAIR_HINT (9)  bright cyan   51  — bottom key hint, A_BOLD
 */
/* palette_xterm256 — bright agent + HUD pairs picked from the 256-colour
 * cube.  Every index sits in the BRIGHT half (≥ 33) so A_DIM stays legible. */
static void palette_xterm256(void)
{
    init_pair(1, 196, -1);   /* red     */
    init_pair(2, 208, -1);   /* orange  */
    init_pair(3, 226, -1);   /* yellow  */
    init_pair(4,  46, -1);   /* green   */
    init_pair(5,  51, -1);   /* cyan    */
    init_pair(6,  33, -1);   /* blue    */
    init_pair(7, 201, -1);   /* magenta */
    init_pair(PAIR_HUD,  226, -1);   /* bright yellow — top status bar  */
    init_pair(PAIR_HINT,  51, -1);   /* bright cyan   — bottom key hint */
}

/* palette_ansi8 — fallback for 8-colour terminals; closest equivalents. */
static void palette_ansi8(void)
{
    init_pair(1, COLOR_RED,     -1);
    init_pair(2, COLOR_RED,     -1);
    init_pair(3, COLOR_YELLOW,  -1);
    init_pair(4, COLOR_GREEN,   -1);
    init_pair(5, COLOR_CYAN,    -1);
    init_pair(6, COLOR_BLUE,    -1);
    init_pair(7, COLOR_MAGENTA, -1);
    init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN,   -1);
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) palette_xterm256();
    else               palette_ansi8();
}

/* ===================================================================== */
/* §4  coords & vec2                                                     */
/* ===================================================================== */

/* pixel-space world dimensions from terminal cell counts */
static inline float pw(int cols) { return (float)(cols * CELL_W); }
static inline float ph(int rows) { return (float)(rows * CELL_H); }

/*
 * px_to_cell_x/y — convert pixel coordinate to terminal cell index.
 * Uses floorf(px/CELL + 0.5) — "round half up" — for stable rounding.
 * Plain truncation would round toward zero and cause staircase artefacts;
 * roundf() can oscillate at .5 boundaries due to banker's rounding.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── Vec2 — lightweight 2-D vector type ───────────────────────────── */
/*
 * Vec2 — a 2-D vector used everywhere physics or geometry shows up.
 *
 * Intent
 *   Position, velocity, acceleration, force, target offset — every
 *   quantity that has an x and a y is one of these.  Functions return
 *   Vec2 by value (no out-params), and the helpers v2add / v2sub /
 *   v2scale / v2norm / v2clamp_len read like the math notation they
 *   implement.
 *
 * Why a struct (not float[2] or two parameters)
 *   • Named type makes function signatures read in the algorithm's
 *     own vocabulary: `steer_seek(pos, vel, target, speed)` rather
 *     than `steer_seek(px, py, vx, vy, tx, ty, speed)`.
 *   • Return by value works on x86-64 + ARM ABIs without spilling
 *     to memory; no perf penalty over passing two floats.
 *   • Eliminates two-out-parameter idioms in the steering helpers.
 *
 * Members
 *   x, y    Cartesian components in PIXEL space (px).  Cell space is
 *           recovered via px_to_cell_x/y when rendering.
 *
 * References
 *   [2] Reynolds 1999 — every steering primitive (seek, flee, separate,
 *       align, cohere) returns a Vec2 force; this file mirrors that API.
 *   [8] Shiffman, *The Nature of Code*, Ch. 1 — PVector pedagogy that
 *       this type is the C analogue of.
 */
typedef struct { float x, y; } Vec2;

static inline Vec2  v2(float x, float y)        { return (Vec2){x, y}; }
static inline Vec2  v2add(Vec2 a, Vec2 b)        { return v2(a.x+b.x, a.y+b.y); }
static inline Vec2  v2sub(Vec2 a, Vec2 b)        { return v2(a.x-b.x, a.y-b.y); }
static inline Vec2  v2scale(Vec2 v, float s)     { return v2(v.x*s, v.y*s); }
static inline float v2len(Vec2 v)                { return sqrtf(v.x*v.x + v.y*v.y); }

/*
 * v2norm — return unit vector; returns zero-vector if v is near-zero
 * to avoid divide-by-zero when two people sit on the same pixel.
 */
static inline Vec2 v2norm(Vec2 v)
{
    float l = v2len(v);
    return (l > 0.001f) ? v2scale(v, 1.0f/l) : v2(0, 0);
}

/*
 * v2clamp_len — cap the vector's magnitude to max_len.
 * Used after accumulating forces to enforce SPEED_MAX.
 */
static inline Vec2 v2clamp_len(Vec2 v, float max_len)
{
    float l = v2len(v);
    return (l > max_len) ? v2scale(v2norm(v), max_len) : v;
}

/* ── boundary helpers ──────────────────────────────────────────────── */

/*
 * wrap_pos — torus wrapping.  Walking off the right edge reappears
 * on the left.  Used by WANDER, FLOCK, GATHER, FOLLOW.
 */
static void wrap_pos(Vec2 *pos, float ww, float wh)
{
    if (pos->x <  0)   pos->x += ww;
    if (pos->x >= ww)  pos->x -= ww;
    if (pos->y <  0)   pos->y += wh;
    if (pos->y >= wh)  pos->y -= wh;
}

/*
 * bounce_pos — elastic wall collision.  Hitting an edge flips the
 * matching velocity component.  Used by PANIC so the crowd is trapped.
 */
static void bounce_pos(Vec2 *pos, Vec2 *vel, float ww, float wh)
{
    if (pos->x <  0)   { pos->x = 0;      vel->x =  fabsf(vel->x); }
    if (pos->x >= ww)  { pos->x = ww-1;   vel->x = -fabsf(vel->x); }
    if (pos->y <  0)   { pos->y = 0;      vel->y =  fabsf(vel->y); }
    if (pos->y >= wh)  { pos->y = wh-1;   vel->y = -fabsf(vel->y); }
}

/* clamp_pos — hard-clip to world bounds.  Used by QUEUE. */
static void clamp_pos(Vec2 *pos, float ww, float wh)
{
    if (pos->x <  0)   pos->x = 0;
    if (pos->x >= ww)  pos->x = ww-1;
    if (pos->y <  0)   pos->y = 0;
    if (pos->y >= wh)  pos->y = wh-1;
}

/* ===================================================================== */
/* §5  entity                                                            */
/* ===================================================================== */

typedef enum {
    BEH_WANDER = 0,
    BEH_FLOCK,
    BEH_PANIC,
    BEH_GATHER,
    BEH_FOLLOW,
    BEH_QUEUE,
    BEH_COUNT
} Behaviour;

static const char *BEH_NAMES[BEH_COUNT] = {
    "WANDER", "FLOCK", "PANIC", "GATHER", "FOLLOW", "QUEUE"
};

/*
 * Glyphs assigned round-robin by person index.
 * Mix of round and angular ASCII chars that suggest human silhouettes.
 */
static const char GLYPHS[] = "oO0abcdefghijklmnpqrstuvwxyz";
#define N_GLYPHS ((int)(sizeof(GLYPHS) - 1))

/*
 * Person — one agent in the steering simulation (Reynolds "boid").
 *
 * Intent
 *   The atomic unit of the crowd.  Each tick every Person computes a
 *   weighted sum of steering forces (seek / flee / separate / align /
 *   cohere), integrates the force into velocity (capped at SPEED_MAX),
 *   and integrates velocity into position.  Six different behaviours
 *   share the same Person — they differ only in which steering forces
 *   their tick_* function applies.
 *
 * Why kinematic + render identity in ONE struct
 *   The struct is split into two cleanly-disjoint groups:
 *     • kinematic state — written by physics every tick;
 *     • render identity — written ONCE at spawn, read only by the
 *       renderer.
 *   The two groups NEVER cross: physics never reads glyph/color, the
 *   renderer never reads vel.  Keeping them in one struct lets the
 *   crowd pool live in a single flat array (good for cache locality
 *   across the O(N²) inner loop) without forcing parallel arrays.
 *
 * Why prev_pos
 *   At any frame the simulation has done one or more fixed-timestep
 *   ticks and may be PART-WAY into the next one.  Drawing always at
 *   `pos` causes a stutter when render rate ≠ sim rate.  Snapshotting
 *   the previous tick's position into `prev_pos` lets the renderer
 *   draw at `lerp(prev_pos, pos, alpha)` — smooth motion at any
 *   render-rate / sim-rate combination.  Same trick as Fiedler's
 *   "Fix Your Timestep!" article.
 *
 * Why per-Person target
 *   Most behaviours (WANDER, FOLLOW, QUEUE) give each Person an
 *   INDIVIDUAL destination instead of a shared global target.  Storing
 *   the target on the Person lets the steering primitive `steer_seek`
 *   stay pure (no global lookup); the tick_* function decides what
 *   the target IS for that mode.
 *
 * Members
 *   ── kinematic state (mutated every tick) ────────────────────────
 *   pos        current position in PIXEL space.  Cell-space rendering
 *              recovers (col, row) via px_to_cell_x/y.
 *   prev_pos   position at start of this tick.  Snapshotted by
 *              person_step BEFORE pos is integrated, so the renderer
 *              can lerp between the two for sub-tick smoothness.
 *   vel        velocity in px/s, capped at SPEED_MAX (or SPEED_PANIC
 *              in BEH_PANIC).  Mass implicitly = 1, so force·dt is
 *              added directly to vel.
 *   target     personal destination.  Read by steer_seek; written by
 *              the active behaviour's tick_* (random wander point,
 *              the person ahead in FOLLOW, the queue slot in QUEUE).
 *
 *   ── render identity (immutable after spawn) ──────────────────────
 *   glyph      ASCII character (one of GLYPHS[]) drawn for this person.
 *              Picked at spawn from a fixed glyph palette so the eye
 *              can track individual agents through the crowd.
 *   color      ncurses colour pair index in [1, N_COLORS].  Round-robin
 *              + shuffle at spawn so adjacent indices don't share a
 *              colour by construction.
 *
 * Invariants
 *   pos ∈ world box (enforced by wrap_pos / bounce_pos / clamp_pos
 *   in the active tick_* — choice depends on behaviour).
 *   |vel| ≤ SPEED_MAX after person_step (or SPEED_PANIC in PANIC mode).
 *   glyph + color set once at spawn; never mutated thereafter.
 *
 * References
 *   [1] Reynolds 1987 — Person ≈ "boid": a point particle with a
 *       desired velocity and a steering force budget.
 *   [2] Reynolds 1999 §2 — the same kinematic model used here: force →
 *       velocity → position, with velocity capped to max_speed.
 *   Fiedler, "Fix Your Timestep!" — gafferongames.com/post/
 *       fix_your_timestep — the prev_pos + alpha-lerp interpolation
 *       pattern used by scene_draw.
 *   [8] Shiffman *Nature of Code* Ch. 6 — same Person/Vehicle data
 *       layout in Processing pseudocode.
 */
typedef struct {
    /* kinematic state — written by physics every tick */
    Vec2  pos;        /* current position, pixel space (px)              */
    Vec2  prev_pos;   /* position at start of this tick — for alpha lerp */
    Vec2  vel;        /* velocity, pixels per second                     */
    Vec2  target;     /* personal destination (used by WANDER/QUEUE/...) */

    /* render identity — written once at spawn, read by scene_draw only */
    char  glyph;      /* ASCII character drawn for this person           */
    int   color;      /* ncurses colour pair index, 1 .. N_COLORS        */
} Person;

static float randf(void) { return (float)rand() / (float)RAND_MAX; }

/* person_spawn — randomise all fields of one person within the world. */
static void person_spawn(Person *p, int id, float ww, float wh)
{
    p->pos      = v2(randf()*ww, randf()*wh);
    p->prev_pos = p->pos;
    p->vel      = v2((randf()-0.5f)*SPEED_BASE, (randf()-0.5f)*SPEED_BASE);
    p->target   = v2(randf()*ww, randf()*wh);
    p->glyph    = GLYPHS[id % N_GLYPHS];
    p->color    = (id % N_COLORS) + 1;
}

/*
 * person_step — shared integration used by every behaviour.
 *
 * 1. Accumulate: add acceleration × dt to velocity.
 * 2. Clamp:      cap speed to SPEED_MAX.
 * 3. Snapshot:   save current pos as prev_pos for alpha interpolation.
 * 4. Integrate:  advance position by velocity × dt.
 *
 * Boundary handling (wrap / bounce / clamp) is left to the caller
 * because each behaviour uses a different boundary condition.
 */
static void person_step(Person *p, Vec2 accel, float dt)
{
    /* (1) symplectic-Euler velocity update: v += a·dt, then cap at SPEED_MAX */
    Vec2 vel_delta = v2scale(accel, dt);
    Vec2 new_vel   = v2add(p->vel, vel_delta);
    p->vel = v2clamp_len(new_vel, SPEED_MAX);

    /* (2) snapshot prev_pos BEFORE moving — renderer lerps prev→pos by alpha */
    p->prev_pos = p->pos;

    /* (3) position update: x += v·dt */
    Vec2 pos_delta = v2scale(p->vel, dt);
    p->pos = v2add(p->pos, pos_delta);
}

/*
 * person_at_target — arrival test used by WANDER / FOLLOW / threat:
 *   true once |target − pos| drops below ARRIVE_DIST (~3 cells).
 */
static inline bool person_at_target(const Person *p)
{
    return v2len(v2sub(p->target, p->pos)) < ARRIVE_DIST;
}

/*
 * person_pick_random_target — uniform-random destination across the
 * world box.  Called after arrival so each person keeps wandering.
 */
static inline void person_pick_random_target(Person *p, float ww, float wh)
{
    p->target = v2(randf()*ww, randf()*wh);
}

/* ===================================================================== */
/* §6  steering — force functions, all return Vec2                       */
/* ===================================================================== */

/*
 * steer_seek — generate a force that steers toward (target) at (speed).
 *
 *   desired = normalise(target − pos) × speed   ← where we want to go
 *   force   = desired − vel                      ← error correction term
 *
 * The subtraction of current velocity is the key insight: when we're
 * already going at full desired speed toward the target, force → 0.
 * This gives natural deceleration without any explicit slow-down code.
 */
static Vec2 steer_seek(Vec2 pos, Vec2 vel, Vec2 target, float speed)
{
    /* offset       : displacement vector self → target
     * toward_dir   : unit vector in that direction
     * desired_vel  : full-speed velocity we WISH we had right now
     * force        : how much we must add to vel to BECOME desired_vel */
    Vec2 offset      = v2sub(target, pos);
    Vec2 toward_dir  = v2norm(offset);
    Vec2 desired_vel = v2scale(toward_dir, speed);
    return v2sub(desired_vel, vel);
}

/*
 * steer_flee — steer directly AWAY from (threat).
 * Implemented as seek with the result negated; all the same math applies.
 */
static Vec2 steer_flee(Vec2 pos, Vec2 vel, Vec2 threat, float speed)
{
    return v2scale(steer_seek(pos, vel, threat, speed), -1.0f);
}

/*
 * steer_separate — push away from all neighbours within SEP_RADIUS.
 *
 * For each neighbour closer than SEP_RADIUS:
 *   push_dir                 = normalise(my_pos − neighbour_pos)   ← unit AWAY
 *   personal_space_intrusion = (SEP_RADIUS − distance) / SEP_RADIUS ← closer = stronger
 *   force += push_dir · personal_space_intrusion · SPEED_BASE
 *
 * The linear falloff means neighbours at the edge of the radius
 * contribute almost nothing; only very close ones produce strong pushes.
 * This creates soft "personal space" rather than a hard exclusion zone.
 */
static Vec2 steer_separate(const Person *people, int count, int self)
{
    const float DIST_EPSILON = 0.001f;       /* zero-distance guard */
    Vec2 force = v2(0, 0);
    Vec2 pos   = people[self].pos;

    for (int i = 0; i < count; i++) {
        if (i == self) continue;

        /* away_offset : neighbour → self displacement (we push that way)
         * distance    : how far the neighbour is right now */
        Vec2  away_offset = v2sub(pos, people[i].pos);
        float distance    = v2len(away_offset);

        bool inside_personal_space = distance < SEP_RADIUS;
        bool well_separated        = distance > DIST_EPSILON;
        if (!inside_personal_space || !well_separated) continue;

        /* personal_space_intrusion ∈ (0,1] — linear falloff, 1 at d=0.
         * push_dir is a unit vector AWAY from the neighbour. */
        float personal_space_intrusion = (SEP_RADIUS - distance) / SEP_RADIUS;
        Vec2  push_dir = v2norm(away_offset);
        Vec2  contribution = v2scale(push_dir, personal_space_intrusion * SPEED_BASE);
        force = v2add(force, contribution);
    }
    return force;
}

/*
 * steer_align — match the average heading of nearby neighbours (boids rule 2).
 *
 * Computes mean velocity of everyone within ALIGN_RADIUS, then returns
 * a force that nudges this person's velocity toward that average.
 * Effect: birds that are going roughly the same direction converge
 * to exactly the same direction → the "flying together" look.
 */
static Vec2 steer_align(const Person *people, int count, int self)
{
    /* (1) sum velocity vectors of every neighbour inside ALIGN_RADIUS */
    Vec2 velocity_sum     = v2(0, 0);
    int  n_neighbours     = 0;
    Vec2 pos              = people[self].pos;
    for (int i = 0; i < count; i++) {
        if (i == self) continue;
        bool inside_align_radius = v2len(v2sub(people[i].pos, pos)) < ALIGN_RADIUS;
        if (inside_align_radius) {
            velocity_sum = v2add(velocity_sum, people[i].vel);
            n_neighbours++;
        }
    }
    if (n_neighbours == 0) return v2(0, 0);

    /* (2) mean velocity vector = "the direction the crowd is heading"
     *     force = mean_velocity − own_velocity  (Reynolds rule 2) */
    Vec2 mean_velocity = v2scale(velocity_sum, 1.0f / (float)n_neighbours);
    return v2sub(mean_velocity, people[self].vel);
}

/*
 * steer_cohere — seek the centre of mass of nearby neighbours (boids rule 3).
 *
 * Finds the average position of everyone within COHESION_RADIUS, then
 * seeks toward that point.  Prevents the flock from drifting apart.
 * Note: this reuses steer_seek, which already handles the "slow down
 * when arriving" behaviour — the group acts as a soft attractor.
 */
static Vec2 steer_cohere(const Person *people, int count, int self)
{
    /* (1) sum positions of every neighbour inside COHESION_RADIUS */
    Vec2 position_sum = v2(0, 0);
    int  n_neighbours = 0;
    Vec2 pos          = people[self].pos;
    for (int i = 0; i < count; i++) {
        if (i == self) continue;
        bool inside_cohesion_radius = v2len(v2sub(people[i].pos, pos)) < COHESION_RADIUS;
        if (inside_cohesion_radius) {
            position_sum = v2add(position_sum, people[i].pos);
            n_neighbours++;
        }
    }
    if (n_neighbours == 0) return v2(0, 0);

    /* (2) centre of mass = average neighbour position
     *     force = steer_seek(centre_of_mass)  — reuses arrival behaviour */
    Vec2 centre_of_mass = v2scale(position_sum, 1.0f / (float)n_neighbours);
    return steer_seek(pos, people[self].vel, centre_of_mass, SPEED_BASE);
}

/* ── §6.5 force composition — blend weighted steering vectors ──────── */
/*
 * blend2_forces / blend3_forces — pseudocode-shape helpers used by every
 * tick_* function.  The body is just a v2add chain of weighted vectors,
 * but writing it inline drowns each tick in nested calls.  Naming the
 * blend lets each tick body read as one line:
 *     force = blend2_forces(seek_force, W_SEEK, sep_force, W_SEP);
 * which is exactly the algorithm Reynolds [2] describes.
 */
static inline Vec2 blend2_forces(Vec2 a, float wa, Vec2 b, float wb)
{
    return v2add(v2scale(a, wa), v2scale(b, wb));
}
static inline Vec2 blend3_forces(Vec2 a, float wa,
                                  Vec2 b, float wb,
                                  Vec2 c, float wc)
{
    return v2add(v2add(v2scale(a, wa), v2scale(b, wb)), v2scale(c, wc));
}

/* ===================================================================== */
/* §7  scene                                                             */
/* ===================================================================== */

/*
 * Crowd — the agent pool.
 *
 * Intent
 *   Owns the active population of Person.  Every steering primitive
 *   (separate / align / cohere) needs the full neighbour list as
 *   input, so the pool lives as ONE flat array indexed by integer.
 *
 * Why fixed-size [CROWD_MAX] + active prefix `count`
 *   • Project rule: no allocation after init.  A compile-time
 *     CROWD_MAX cap lets the entire scene live in BSS / on main()'s
 *     stack.
 *   • The "pre-spawned, active prefix" idiom lets '+'/'−' grow and
 *     shrink the visible crowd by changing `count` alone — no
 *     re-randomise on every keypress.  Even agents currently OUTSIDE
 *     the active prefix were placed at scene_init so they appear at
 *     plausible positions the moment the user adds them.
 *
 * Why bundle (people[] + count) instead of two Scene fields
 *   Every loop that reads `people[]` needs `count` to bound it; they
 *   always move together.  Bundling makes every loop body read
 *   `for (int i = 0; i < s->crowd.count; i++) … s->crowd.people[i] …`
 *   — one sub-struct accessed twice — instead of two scattered Scene
 *   fields the reader must keep in mind separately.
 *
 * Members
 *   people[CROWD_MAX]   the entire pool, pre-spawned at scene_init.
 *                       Index in this array is meaningless to the user
 *                       (Persons are indistinguishable except by their
 *                       glyph + colour); only spatial arrangement matters.
 *   count               number of CURRENTLY ACTIVE persons; first
 *                       `count` slots are simulated and drawn, the
 *                       rest lie dormant.  Range: [CROWD_MIN, CROWD_MAX].
 *
 * Invariants
 *   CROWD_MIN ≤ count ≤ CROWD_MAX  (handle_input clamps).
 *   Every people[i] in [0, count) has pos inside the world box
 *   (enforced by the boundary helpers in §4).
 *   people[CROWD_MAX − 1] was spawned at scene_init even if
 *   count < CROWD_MAX; its pos is valid but it is not drawn / ticked.
 *
 * References
 *   [1] Reynolds 1987 — flocking on a fixed pool of N boids; this
 *       file's pool layout matches that O(N²) reference implementation.
 *   [10] Buckland *Programming Game AI by Example* Ch. 3 — same
 *       "world owns the agent vector" data layout for steering.
 */
typedef struct {
    Person people[CROWD_MAX];
    int    count;
} Crowd;

/*
 * Threat — the PANIC-only roaming '!' marker.
 *
 * Intent
 *   The repulsor that gives BEH_PANIC its drama.  Every Person flees
 *   away from `threat.pos` via steer_flee.  The threat itself wanders
 *   (slowly) toward random targets via the same steer_seek primitive
 *   the crowd uses, so it explores the room rather than sitting still.
 *
 * Why a small kinematic struct (not just a Vec2 pos)
 *   The threat itself needs to move smoothly: it accelerates toward
 *   its current target, bounces off walls, picks a new target on
 *   arrival.  That's the same (pos, vel, target) trio that a Person
 *   has — so the threat is a tiny "Person without rendering identity".
 *   Bundling makes tick_panic touch one sub-struct instead of three
 *   scattered Scene fields.
 *
 * Why pos / vel / target (not just pos)
 *   A teleporting threat would look fake.  A constant-velocity threat
 *   would clip walls.  Giving it the same kinematic model as the
 *   crowd (and capping its speed BELOW the crowd's panic speed) means
 *   the cloud always escapes — which is what makes the visual work.
 *
 * Members
 *   pos      pixel-space position; flee target for every Person.
 *   vel      px/s, capped at THREAT_SPEED.  See tick_panic for how
 *            it's integrated (steer_seek toward target + v2clamp_len).
 *   target   random wander point the threat is currently seeking.
 *            Reseeded to a new uniform-random point when |target−pos|
 *            < ARRIVE_DIST in tick_panic.
 *
 * Invariants
 *   pos lies inside the world box (clamped + bounced by tick_panic).
 *   |vel| ≤ THREAT_SPEED (enforced by v2clamp_len in tick_panic).
 *   THREAT_SPEED < SPEED_PANIC by design — the crowd always escapes.
 *
 * References
 *   [6] Helbing, Farkas & Vicsek 2000 (*Nature*) — "Simulating
 *       dynamical features of escape panic" — models a crowd fleeing
 *       a hazard with a desired velocity AWAY from the hazard centre,
 *       which is exactly what BEH_PANIC implements with steer_flee.
 *   [5] Helbing & Molnár 1995 — social-force model; the threat acts
 *       as the strong repulsive source in that framework.
 */
typedef struct {
    Vec2 pos;
    Vec2 vel;
    Vec2 target;
} Threat;

/*
 * SimControls — user-facing playback knobs.
 *
 * Intent
 *   Three scalars the input handler writes and scene_tick + HUD read.
 *   Bundling them keeps handle_input's switch readable and the HUD's
 *   read paths short.
 *
 * Why a sub-struct (three small fields could be flat Scene members)
 *   The boundary "user-controlled run parameters" is conceptually
 *   distinct from "simulation state": behaviour, paused, fps are all
 *   written by ONE function (handle_input) and read by TWO consumers
 *   (scene_tick + draw_hud).  Bundling makes their shared role
 *   explicit at the type level and leaves Scene's top level free for
 *   future additions (time_scale, debug overlays) without churn.
 *
 * Why behaviour lives here (not on Scene directly)
 *   The behaviour selector is the single most user-facing piece of
 *   state in the file — keys 1..6 swap it, the HUD displays its name,
 *   scene_tick dispatches on it.  Placing it inside SimControls
 *   groups it with the other knobs the user controls, separating it
 *   from "data the simulation HAS" (crowd, threat).
 *
 * Members
 *   behaviour   one of BEH_WANDER / BEH_FLOCK / BEH_PANIC / BEH_GATHER /
 *               BEH_FOLLOW / BEH_QUEUE — chosen by keys 1..6.  Drives
 *               the switch in scene_tick that selects tick_wander /
 *               tick_flock / ... and conditional decorations in
 *               scene_draw (the '!' threat, '@' leader, '|' counter).
 *   paused      true → scene_tick is a no-op; HUD shows "PAUSED".
 *               Toggled by SPACE.
 *   fps         simulation tick rate (Hz).  Set at scene_init to
 *               SIM_FPS_DEFAULT; not directly user-adjustable in this
 *               file (the +/- keys grow / shrink the crowd, not fps).
 *               Exposed as a field so future extensions (time
 *               scrubbing, slowmo) plug in without restructuring.
 *
 * Invariants
 *   0 ≤ behaviour < BEH_COUNT.
 *   fps > 0 (used as divisor in TICK_NS).
 *
 * References
 *   [1] Reynolds 1987 — the "behaviour selection" pattern: same
 *       low-level mechanics, different high-level rule set per agent.
 *   Fiedler "Fix Your Timestep!" — sim.fps drives the fixed-timestep
 *       accumulator in main(); decoupling sim rate from render rate.
 */
typedef struct {
    Behaviour behaviour;
    bool      paused;
    int       fps;
} SimControls;

/*
 * Scene — owns ALL simulation + render state for one run.
 *
 * Layered ownership
 *
 *     Scene
 *       ├── crowd  : Crowd                ← agent pool (§5 Person + count)
 *       │     ├── people[CROWD_MAX]: Person[]
 *       │     └── count
 *       │
 *       ├── threat : Threat               ← PANIC roaming '!' marker
 *       │     ├── pos, vel, target  : Vec2[]
 *       │
 *       ├── sim    : SimControls          ← user-facing knobs
 *       │     ├── behaviour : Behaviour
 *       │     ├── paused    : bool
 *       │     └── fps       : int
 *       │
 *       ├── world_w, world_h              ← world extent in PIXEL space
 *       │                                   (pw(scene_cols), ph(scene_rows))
 *       │
 *       └── scene_cols, scene_rows        ← terminal extent in CELL space
 *
 *   Every persistent value the program needs is reachable from a
 *   single Scene*.  No file-scope simulation state exists; only the
 *   two POSIX signal flags (g_running, g_need_resize) remain global
 *   in §8 because signal handlers cannot receive a context pointer.
 *
 * Intent
 *   One instance lives on main()'s stack and is passed by pointer to
 *   every tick, renderer, and input handler.  Pure observers take
 *   `const Scene *`; mutators take `Scene *`.  The signature alone
 *   communicates which kind of access each callee performs.
 *
 * Why a SCENE struct (not file-scope globals)
 *   • Replaceability: future variants (split-screen demo, recorded
 *     replay) allocate multiple Scenes — impossible with globals.
 *   • Testability: scene_tick becomes a pure transformation s → s';
 *     no hidden inputs.
 *   • Reading aid: every helper begins with `Scene *s` (or const),
 *     giving the reader ONE place to look for "what state exists".
 *
 * Why TWO extents (world_w/h AND scene_cols/rows)
 *   They live in DIFFERENT spaces:
 *     • scene_cols × scene_rows is the TERMINAL grid (cells), used
 *       by the renderer + HUD layout.
 *     • world_w × world_h is the SIMULATION space (pixels), used by
 *       every physics quantity.  Computed as pw(scene_cols),
 *       ph(scene_rows) so the world scales with terminal resize.
 *   Keeping both pre-computed avoids re-multiplying CELL_W / CELL_H
 *   inside every steering force evaluation.
 *
 * Members
 *   crowd           Crowd — the active agent pool.
 *   threat          Threat — PANIC-only roaming repulsor.
 *   sim             SimControls — behaviour + paused + fps.
 *   world_w/h       pixel-space world extent; scales with resize.
 *   scene_cols/rows cell-space terminal extent; updated by SIGWINCH
 *                   handler via handle_resize_if_pending.
 *
 * Invariants
 *   scene_cols > 0 AND scene_rows > 0 (set by screen_init).
 *   world_w == pw(scene_cols) AND world_h == ph(scene_rows).
 *   All sub-struct invariants hold (Crowd.count clamp, Threat speed
 *   cap, SimControls behaviour range).
 *   g_running / g_need_resize are NOT here — POSIX signal handlers
 *   need sig_atomic_t globals; they live in §8.
 *
 * References
 *   [10] Buckland *Programming Game AI by Example* — single "World"
 *       object owns the agent vector and the simulation parameters,
 *       same pattern as Scene here.
 *   Nystrom, "Game Programming Patterns" Ch. 9 (Game Loop) — the
 *       single-Scene + fixed-timestep accumulator design that main()
 *       implements.
 */
typedef struct {
    Crowd       crowd;
    Threat      threat;
    SimControls sim;
    float       world_w, world_h;        /* pixel-space world extent     */
    int         scene_cols, scene_rows;  /* cell-space terminal extent   */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);

    /* (1) extents — terminal in cells, simulation in pixels */
    s->scene_cols = cols;
    s->scene_rows = rows;
    s->world_w    = pw(cols);
    s->world_h    = ph(rows);

    /* (2) user-facing knobs (active behaviour, pause flag, sim rate) */
    s->crowd.count   = CROWD_DEFAULT;
    s->sim.behaviour = BEH_WANDER;
    s->sim.paused    = false;
    s->sim.fps       = SIM_FPS_DEFAULT;

    /* (3) pre-spawn the FULL pool — '+' just grows `count`, no re-randomise */
    for (int i = 0; i < CROWD_MAX; i++)
        person_spawn(&s->crowd.people[i], i, s->world_w, s->world_h);

    /* (4) PANIC threat seeded at world centre, heading for a random target */
    const float WORLD_CENTRE_FRAC = 0.5f;
    s->threat.pos    = v2(s->world_w * WORLD_CENTRE_FRAC,
                          s->world_h * WORLD_CENTRE_FRAC);
    s->threat.target = v2(randf()*s->world_w, randf()*s->world_h);
}

/* ── behaviour helpers — shared by tick_* and scene_draw ──────────── */

/*
 * lerp_render_pos — sub-tick interpolation between two physics snapshots.
 *
 *   draw_pos = prev + (cur − prev) × alpha          (alpha ∈ [0,1))
 *
 * Used by scene_draw so an agent moves smoothly between ticks even when
 * render rate ≠ sim rate (Fiedler, "Fix Your Timestep!").
 */
static inline Vec2 lerp_render_pos(Vec2 prev, Vec2 cur, float alpha)
{
    /* tick_delta : how far the agent moved during the last physics tick
     * partial_step: the alpha-fraction of that motion we draw THIS frame */
    Vec2 tick_delta   = v2sub(cur, prev);
    Vec2 partial_step = v2scale(tick_delta, alpha);
    return v2add(prev, partial_step);
}

/*
 * gather_approach_speed — linear ramp from 0 at the centre to SPEED_BASE
 * at the edge of the slow ring.  Outside the ring people cruise at full
 * speed; inside they decelerate so the crowd mills rather than oscillates.
 *
 *   speed(dist) = { SPEED_BASE · dist / SLOW_RADIUS   if dist < SLOW_RADIUS
 *                 { SPEED_BASE                         otherwise
 */
static inline float gather_approach_speed(float dist_to_centre)
{
    if (dist_to_centre >= GATHER_SLOW_RADIUS) return SPEED_BASE;
    /* slow_ramp_fraction ∈ [0, 1): 0 at centre, 1 at edge of slow ring */
    float slow_ramp_fraction = dist_to_centre / GATHER_SLOW_RADIUS;
    return SPEED_BASE * slow_ramp_fraction;
}

/*
 * queue_approach_speed — binary step: shuffle near the slot, cruise else.
 * The hard step (rather than a ramp) is what makes the line feel patient:
 * people walk briskly until they're nearly there, then noticeably slow.
 */
static inline float queue_approach_speed(float dist_to_slot)
{
    return (dist_to_slot < QUEUE_SLOW_RADIUS) ? QUEUE_IDLE_SPEED : SPEED_BASE;
}

/*
 * queue_slot_for_index — geometry of the staggered queue line.
 *
 *   counter  : sits COUNTER_INSET_CELLS cells inside the right edge,
 *              on the vertical mid-line of the world.
 *   slot[N]  : N gaps to the left of the counter, on row (N mod 3) − 1
 *              of a STAGGER_ROWS=3 vertical pattern so the line isn't
 *              a dead-straight column.
 *   clamp    : slots that would fall off the left edge get pinned one
 *              cell inside.
 */
static Vec2 queue_slot_for_index(int i, float ww, float wh)
{
    enum { COUNTER_INSET_CELLS = 2 };
    enum { STAGGER_ROWS        = 3 };
    const float WORLD_VERTICAL_CENTRE_FRAC = 0.5f;
    const float LEFT_EDGE_INSET_PX         = (float)CELL_W;

    /* counter position: a few cells inside the right edge, vertically centred */
    const float counter_x = ww - (float)CELL_W * (float)COUNTER_INSET_CELLS;
    const float counter_y = wh * WORLD_VERTICAL_CENTRE_FRAC;

    /* slot N: N gaps left of counter, on stagger row (N mod 3) − 1 ∈ {−1,0,+1}
     * so successive people land on top / middle / bottom of the 3-row line. */
    int   stagger_row_offset = (i % STAGGER_ROWS) - 1;       /* {-1, 0, +1} */
    float slot_x             = counter_x - (float)i * QUEUE_SLOT_W;
    float slot_y             = counter_y + (float)stagger_row_offset * QUEUE_SLOT_H;

    /* clamp: very far-back slots would otherwise fall off the left edge */
    if (slot_x < LEFT_EDGE_INSET_PX) slot_x = LEFT_EDGE_INSET_PX;
    return v2(slot_x, slot_y);
}

/*
 * threat_advance — PANIC's roaming '!' marker integrates like a slow Person.
 *
 *   1. on arrival within THREAT_ARRIVE_RADIUS, pick a fresh wander target
 *   2. otherwise accelerate toward target, velocity capped at THREAT_SPEED
 *   3. integrate position by velocity·dt and clamp inside the world box
 *
 * THREAT_SPEED < SPEED_PANIC by design so the crowd can always escape —
 * the threat is a pressure source, not a chaser.
 */
static void threat_advance(Threat *t, float dt, float ww, float wh)
{
    /* threat re-picks a target once it gets within 2× the arrival ring */
    const float THREAT_ARRIVE_RADIUS = ARRIVE_DIST * 2.0f;

    /* (1) on arrival, re-roll a fresh random wander target */
    Vec2  offset_to_target   = v2sub(t->target, t->pos);
    float distance_to_target = v2len(offset_to_target);
    if (distance_to_target < THREAT_ARRIVE_RADIUS) {
        t->target = v2(randf()*ww, randf()*wh);
    } else {
        /* (2) otherwise accelerate toward target, velocity capped at top speed
         *     toward_dir : unit vector pointing at the target
         *     accel_v    : "desired-velocity" vector at THREAT_SPEED */
        Vec2 toward_dir = v2norm(offset_to_target);
        Vec2 accel_v    = v2scale(toward_dir, THREAT_SPEED);
        Vec2 new_vel    = v2add(t->vel, v2scale(accel_v, dt));
        t->vel          = v2clamp_len(new_vel, THREAT_SPEED);
    }

    /* (3) Euler integrate position, then clamp inside the world walls */
    Vec2 step = v2scale(t->vel, dt);
    t->pos    = v2add(t->pos, step);
    clamp_pos(&t->pos, ww, wh);
}

/* ── behaviour tick functions ──────────────────────────────────────── */
/*
 * Each tick_* function applies one behaviour to all active people.
 * Every body follows the same pseudocode shape:
 *
 *   for each Person p in crowd:
 *       1. (optional) re-pick target on arrival
 *            → person_at_target + person_pick_random_target
 *       2. compute named force components from §6 steering primitives
 *            seek_force / sep_force / flee_force / align_force / ...
 *       3. blend them with mode-specific weights
 *            → blend2_forces or blend3_forces                (§6.5)
 *       4. integrate force into vel + pos
 *            → person_step(p, force, dt)                     (§5)
 *       5. enforce boundary (wrap / bounce / clamp)          (§4)
 *
 * The behaviour-specific helpers (gather_approach_speed,
 * queue_approach_speed, queue_slot_for_index, threat_advance) are
 * defined just above and named by what they DO algorithmically.
 */

/*
 * WANDER — individuals drift to random targets; separation enforces
 * personal space.  On arrival each person picks a new random target.
 * The world wraps toroidally so nobody gets stuck at edges.
 */
static void tick_wander(Scene *s, float dt)
{
    Crowd *crowd = &s->crowd;
    for (int i = 0; i < crowd->count; i++) {
        Person *p = &crowd->people[i];

        /* (1) on arrival, re-roll a random destination */
        if (person_at_target(p))
            person_pick_random_target(p, s->world_w, s->world_h);

        /* (2) force = W_SEEK · seek_target + W_SEP · separate_from_neighbours */
        Vec2 seek_force = steer_seek    (p->pos, p->vel, p->target, SPEED_BASE);
        Vec2 sep_force  = steer_separate(crowd->people, crowd->count, i);
        Vec2 force      = blend2_forces(seek_force, W_SEEK, sep_force, W_SEP);

        /* (3) integrate, then torus-wrap at the world edges */
        person_step(p, force, dt);
        wrap_pos(&p->pos, s->world_w, s->world_h);
    }
}

/*
 * FLOCK — classic Reynolds boids with three weighted rules.
 * No explicit target: the group itself becomes the attractor.
 * Try raising/lowering W_ALIGN and W_COHERE to see the flock change.
 */
static void tick_flock(Scene *s, float dt)
{
    Crowd *crowd = &s->crowd;
    for (int i = 0; i < crowd->count; i++) {
        Person *p = &crowd->people[i];

        /* (1) Reynolds' three local rules, all O(N²) over the crowd */
        Vec2 sep_force    = steer_separate(crowd->people, crowd->count, i);
        Vec2 align_force  = steer_align   (crowd->people, crowd->count, i);
        Vec2 cohere_force = steer_cohere  (crowd->people, crowd->count, i);

        /* (2) weighted sum — SEP dominates so flockmates never overlap */
        Vec2 force = blend3_forces(sep_force,    W_SEP,
                                   align_force,  W_ALIGN,
                                   cohere_force, W_COHERE);

        /* (3) integrate and wrap */
        person_step(p, force, dt);
        wrap_pos(&p->pos, s->world_w, s->world_h);
    }
}

/*
 * PANIC — everyone flees a slowly roaming threat '!'.
 * The threat is slower than the crowd (THREAT_SPEED < SPEED_PANIC)
 * so escape is possible, but only if people keep moving.
 * Walls bounce — people can't escape off-screen, adding pressure.
 */
static void tick_panic(Scene *s, float dt)
{
    /* (1) advance the roaming threat first — crowd flees its CURRENT pos */
    threat_advance(&s->threat, dt, s->world_w, s->world_h);

    /* (2) each person: flee threat + keep personal space; walls bounce */
    Crowd *crowd = &s->crowd;
    for (int i = 0; i < crowd->count; i++) {
        Person *p = &crowd->people[i];

        Vec2 flee_force = steer_flee    (p->pos, p->vel, s->threat.pos, SPEED_PANIC);
        Vec2 sep_force  = steer_separate(crowd->people, crowd->count, i);
        Vec2 force      = blend2_forces(flee_force, W_FLEE, sep_force, W_SEP);

        person_step(p, force, dt);
        bounce_pos(&p->pos, &p->vel, s->world_w, s->world_h);
    }
}

/*
 * GATHER — everyone seeks the screen centre.
 * Speed scales down linearly inside GATHER_SLOW_RADIUS so people
 * mill around once gathered rather than overshooting and oscillating.
 */
static void tick_gather(Scene *s, float dt)
{
    const float WORLD_CENTRE_FRAC = 0.5f;
    const Vec2  centre = v2(s->world_w * WORLD_CENTRE_FRAC,
                            s->world_h * WORLD_CENTRE_FRAC);
    Crowd *crowd = &s->crowd;

    for (int i = 0; i < crowd->count; i++) {
        Person *p = &crowd->people[i];

        /* (1) ramp approach speed so people slow as they reach the centre */
        float dist_to_centre = v2len(v2sub(centre, p->pos));
        float approach_speed = gather_approach_speed(dist_to_centre);

        /* (2) force = seek_centre (eased) + separate */
        Vec2 seek_force = steer_seek    (p->pos, p->vel, centre, approach_speed);
        Vec2 sep_force  = steer_separate(crowd->people, crowd->count, i);
        Vec2 force      = blend2_forces(seek_force, W_SEEK, sep_force, W_SEP);

        person_step(p, force, dt);
        wrap_pos(&p->pos, s->world_w, s->world_h);
    }
}

/*
 * FOLLOW — chain following: person[i] seeks person[i-1].
 * Person[0] is the leader (drawn as '@') and wanders freely.
 * The chain produces a snake-like procession; separation prevents
 * the tail from bunching up against the leader.
 */
static void tick_follow(Scene *s, float dt)
{
    enum { LEADER_INDEX = 0 };
    Crowd *crowd = &s->crowd;

    /* (1) leader wanders freely — same rule as WANDER but no separation */
    Person *leader = &crowd->people[LEADER_INDEX];
    if (person_at_target(leader))
        person_pick_random_target(leader, s->world_w, s->world_h);

    Vec2 lead_force = steer_seek(leader->pos, leader->vel,
                                  leader->target, SPEED_BASE);
    person_step(leader, lead_force, dt);
    wrap_pos(&leader->pos, s->world_w, s->world_h);

    /* (2) each follower chases person ahead in the chain, with separation */
    for (int i = LEADER_INDEX + 1; i < crowd->count; i++) {
        Person *p    = &crowd->people[i];
        Person *prev = &crowd->people[i - 1];

        Vec2 chase_force = steer_seek    (p->pos, p->vel, prev->pos, SPEED_BASE);
        Vec2 sep_force   = steer_separate(crowd->people, crowd->count, i);
        Vec2 force       = blend2_forces(chase_force, W_SEEK, sep_force, W_SEP);

        person_step(p, force, dt);
        wrap_pos(&p->pos, s->world_w, s->world_h);
    }
}

/*
 * QUEUE — each person is assigned a reserved slot in a line that
 * extends leftward from the right-edge service counter.
 * Slots stagger across 3 rows to prevent a dead-straight column.
 * Speed drops to QUEUE_IDLE_SPEED near the slot (patient shuffling).
 * Separation weight is halved — people tolerate being packed in line.
 */
static void tick_queue(Scene *s, float dt)
{
    /* halved separation weight: people tolerate close packing in a line */
    const float W_SEP_PACKED = W_SEP * 0.5f;
    Crowd *crowd = &s->crowd;

    for (int i = 0; i < crowd->count; i++) {
        Person *p = &crowd->people[i];

        /* (1) reserved slot for this person on the 3-row staggered line */
        Vec2  slot           = queue_slot_for_index(i, s->world_w, s->world_h);
        float dist_to_slot   = v2len(v2sub(slot, p->pos));
        float approach_speed = queue_approach_speed(dist_to_slot);

        /* (2) force = seek_slot + relaxed_separation */
        Vec2 seek_force = steer_seek    (p->pos, p->vel, slot, approach_speed);
        Vec2 sep_force  = steer_separate(crowd->people, crowd->count, i);
        Vec2 force      = blend2_forces(seek_force, W_SEEK, sep_force, W_SEP_PACKED);

        /* (3) integrate, hard-clip to walls (queue stops at the counter) */
        person_step(p, force, dt);
        clamp_pos(&p->pos, s->world_w, s->world_h);
    }
}

/* ── scene dispatch and draw ───────────────────────────────────────── */

/*
 * mark_cell — stamp one ASCII glyph at terminal cell (cx,cy).
 *
 * Centralises the (chtype)(unsigned char) cast, bounds-check, and the
 * wattron / wattroff sandwich so the renderer stays linear and short.
 * The double cast prevents sign-extension on character values > 127
 * (per CLAUDE.md "Common ncurses Bugs").  Off-screen cells are silently
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

static void scene_tick(Scene *s, float dt)
{
    /* Refresh world extent in case a resize has just landed.  Cheap. */
    s->world_w = pw(s->scene_cols);
    s->world_h = ph(s->scene_rows);
    if (s->sim.paused) return;
    switch (s->sim.behaviour) {
    case BEH_WANDER: tick_wander(s, dt); break;
    case BEH_FLOCK:  tick_flock (s, dt); break;
    case BEH_PANIC:  tick_panic (s, dt); break;
    case BEH_GATHER: tick_gather(s, dt); break;
    case BEH_FOLLOW: tick_follow(s, dt); break;
    case BEH_QUEUE:  tick_queue (s, dt); break;
    default: break;
    }
}

/* draw_threat_marker — PANIC's roaming '!' painted at the threat position. */
static void draw_threat_marker(WINDOW *w, const Threat *t, int cols, int rows)
{
    enum { THREAT_COLOR = 1 };       /* red pair */
    int tx = px_to_cell_x(t->pos.x);
    int ty = px_to_cell_y(t->pos.y);
    mark_cell(w, tx, ty, '!', THREAT_COLOR, A_BOLD | A_BLINK, cols, rows);
}

/*
 * draw_queue_counter — QUEUE's service counter ('|' bar) and arrow tip
 * ('>>|') painted on the right edge.  The arrow points INTO the counter,
 * cueing the queue's direction of motion.
 */
static void draw_queue_counter(WINDOW *w, int cols, int rows)
{
    enum { COUNTER_COLOR        = 3 };       /* yellow */
    enum { COUNTER_INSET_CELLS  = 2 };       /* from right edge */
    enum { COUNTER_HALF_HEIGHT  = 3 };       /* cells above/below mid-row */
    enum { ARROW_TIP_OFFSET     = 1 };       /* '|' one cell left of counter */
    enum { ARROW_BODY_OFFSET    = 2 };       /* first '>' two cells left  */
    enum { ARROW_TAIL_OFFSET    = 3 };       /* second '>' three cells left */

    const int counter_col = cols - COUNTER_INSET_CELLS;
    const int counter_row = rows / 2;

    /* vertical bar — the counter itself */
    for (int dy = -COUNTER_HALF_HEIGHT; dy <= COUNTER_HALF_HEIGHT; dy++)
        mark_cell(w, counter_col, counter_row + dy, '|',
                  COUNTER_COLOR, A_BOLD, cols, rows);

    /* arrow leading into the counter: '>' '>' '|' */
    mark_cell(w, counter_col - ARROW_TAIL_OFFSET, counter_row, '>',
              COUNTER_COLOR, A_BOLD, cols, rows);
    mark_cell(w, counter_col - ARROW_BODY_OFFSET, counter_row, '>',
              COUNTER_COLOR, A_BOLD, cols, rows);
    mark_cell(w, counter_col - ARROW_TIP_OFFSET,  counter_row, '|',
              COUNTER_COLOR, A_BOLD, cols, rows);
}

/* draw_follow_leader — FOLLOW's '@', bold + underline so it stands out. */
static void draw_follow_leader(WINDOW *w, const Person *ldr, float alpha,
                                int cols, int rows)
{
    enum { LEADER_COLOR = 3 };       /* yellow */
    Vec2 dp = lerp_render_pos(ldr->prev_pos, ldr->pos, alpha);
    mark_cell(w, px_to_cell_x(dp.x), px_to_cell_y(dp.y),
              '@', LEADER_COLOR, A_BOLD | A_UNDERLINE, cols, rows);
}

/* draw_person — alpha-lerped position, person's own glyph and colour. */
static void draw_person(WINDOW *w, const Person *p, float alpha,
                         int cols, int rows)
{
    Vec2 dp = lerp_render_pos(p->prev_pos, p->pos, alpha);
    mark_cell(w, px_to_cell_x(dp.x), px_to_cell_y(dp.y),
              p->glyph, p->color, A_NORMAL, cols, rows);
}

/*
 * scene_draw — render the current scene into WINDOW *w.
 *
 * Algorithm:
 *   (1) per-behaviour decoration FIRST so the crowd lays on top of it
 *       (PANIC '!' threat, QUEUE counter + arrow, FOLLOW leader '@')
 *   (2) every active person at alpha-lerped position; in FOLLOW skip
 *       slot 0 because the leader was already drawn at step (1).
 *
 *   draw_pos = prev_pos + (pos − prev_pos) × alpha       (lerp_render_pos)
 */
static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha)
{
    enum { LEADER_INDEX = 0 };

    /* (1) per-behaviour decorations */
    if (s->sim.behaviour == BEH_PANIC)
        draw_threat_marker(w, &s->threat, cols, rows);

    if (s->sim.behaviour == BEH_QUEUE)
        draw_queue_counter(w, cols, rows);

    if (s->sim.behaviour == BEH_FOLLOW && s->crowd.count > 0)
        draw_follow_leader(w, &s->crowd.people[LEADER_INDEX], alpha, cols, rows);

    /* (2) every active person — skip slot 0 in FOLLOW (leader already drawn) */
    int start = (s->sim.behaviour == BEH_FOLLOW) ? LEADER_INDEX + 1 : 0;
    for (int i = start; i < s->crowd.count; i++)
        draw_person(w, &s->crowd.people[i], alpha, cols, rows);
}

/* ===================================================================== */
/* §8  app — screen, signals, input, main loop                          */
/* ===================================================================== */

/*
 * POSIX signal flags — must be file-scope `volatile sig_atomic_t` so
 * async signal handlers can touch them safely.  Set by handlers,
 * polled by main(), cleared by main() once acted on.
 */
static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_exit_signal(int sig)   { (void)sig; g_running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Bring up ncurses, then read the terminal extent into the Scene. */
static void screen_init(Scene *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);   /* non-blocking getch */
    keypad(stdscr, TRUE);
    typeahead(-1);            /* don't let ncurses interrupt output to read ahead */
    color_init();
    getmaxyx(stdscr, s->scene_rows, s->scene_cols);
    s->world_w = pw(s->scene_cols);
    s->world_h = ph(s->scene_rows);
}

/* hud_paint_text — stamp text at (row, col) with a pair + bold attribute.
 * Centralises the attron / mvprintw / attroff sandwich for the two HUD rows. */
static void hud_paint_text(int row, int col, int pair, const char *text)
{
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/*
 * draw_hud — canonical top status (data) + bottom hint (actions).
 *
 * Algorithm:
 *   (1) format status string into a local buffer
 *   (2) paint right-aligned on row 0  (PAIR_HUD  + A_BOLD)
 *   (3) paint key bindings on last row (PAIR_HINT + A_BOLD)
 */
static void draw_hud(const Scene *s, double fps)
{
    enum { HUD_TOP_ROW = 0 };
    static const char *KEY_HINT =
        " q:quit  spc:pause  1:wander 2:flock 3:panic 4:gather 5:follow 6:queue  +/-:people  r:reset ";

    /* (1) status text — fps, sim rate, crowd size, active behaviour, pause */
    char status[HUD_COLS + 1];
    snprintf(status, sizeof status,
             " %5.1f fps  sim:%3d Hz  n:%3d  [%s]%s ",
             fps, s->sim.fps, s->crowd.count,
             BEH_NAMES[s->sim.behaviour],
             s->sim.paused ? "  PAUSED" : "");

    /* (2) right-align status on row 0 */
    int right_col = s->scene_cols - (int)strlen(status);
    if (right_col < 0) right_col = 0;
    hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, status);

    /* (3) key bindings on the last row */
    hud_paint_text(s->scene_rows - 1, 0, PAIR_HINT, KEY_HINT);
}

/*
 * frame_render — erase, paint scene, paint HUD, flush via one doupdate.
 *
 * Order matters: scene_draw first, HUD second, so the HUD lays on top.
 * Nothing reaches the terminal until doupdate() at the end.
 */
static void frame_render(const Scene *s, double fps, float alpha)
{
    erase();
    scene_draw(s, stdscr, s->scene_cols, s->scene_rows, alpha);
    draw_hud(s, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/*
 * clamp_all_active_people — after a terminal shrink, pin any person who
 * was sitting past the new edge back inside the (smaller) world box and
 * reset prev_pos so the renderer doesn't lerp a giant snap-back.
 */
static void clamp_all_active_people(Scene *s)
{
    for (int i = 0; i < s->crowd.count; i++) {
        Person *p = &s->crowd.people[i];
        clamp_pos(&p->pos, s->world_w, s->world_h);
        p->prev_pos = p->pos;
    }
}

/*
 * handle_resize_if_pending — SIGWINCH dance.
 *
 *   (1) tear ncurses down + up so stdscr matches the new terminal extent
 *   (2) recompute pixel-space world extent from the new cell extent
 *   (3) clamp_all_active_people pulls anyone past the new edge back in
 *
 * Cheap no-op when g_need_resize is unset.
 */
static void handle_resize_if_pending(Scene *s)
{
    if (!g_need_resize) return;
    g_need_resize = 0;

    /* (1) tear ncurses down + up so stdscr matches new terminal size */
    endwin(); refresh();
    getmaxyx(stdscr, s->scene_rows, s->scene_cols);

    /* (2) recompute pixel-space world extent from new cell extent */
    s->world_w = pw(s->scene_cols);
    s->world_h = ph(s->scene_rows);

    /* (3) pull any out-of-bounds people back inside the new world box */
    clamp_all_active_people(s);
}

/*
 * handle_input — translate one keystroke into a Scene mutation.
 *
 *   q / Q / ESC  → request quit (clear g_running)
 *   SPACE        → toggle pause
 *   1..6         → select behaviour (WANDER..QUEUE)
 *   + / =        → adjust_person_count(+CROWD_STEP)   (capped at CROWD_MAX)
 *   -            → adjust_person_count(−CROWD_STEP)   (floored at CROWD_MIN)
 *   r / R        → re-init scene at current extent (re-randomise)
 */
/*
 * adjust_person_count — move crowd.count by `delta`, clamped to
 * [CROWD_MIN, CROWD_MAX].  Pool is pre-spawned so this just slides the
 * active prefix; positive delta reveals already-placed people instantly.
 */
static void adjust_person_count(Crowd *crowd, int delta)
{
    int next = crowd->count + delta;
    if (next < CROWD_MIN) next = CROWD_MIN;
    if (next > CROWD_MAX) next = CROWD_MAX;
    crowd->count = next;
}

static void handle_input(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */:
        g_running = 0;                                       break;

    case ' ':  s->sim.paused    = !s->sim.paused;            break;

    /* behaviour select — keys 1..6 pick a mode */
    case '1':  s->sim.behaviour = BEH_WANDER;                break;
    case '2':  s->sim.behaviour = BEH_FLOCK;                 break;
    case '3':  s->sim.behaviour = BEH_PANIC;                 break;
    case '4':  s->sim.behaviour = BEH_GATHER;                break;
    case '5':  s->sim.behaviour = BEH_FOLLOW;                break;
    case '6':  s->sim.behaviour = BEH_QUEUE;                 break;

    /* crowd-size adjust — slides the active prefix in pre-spawned pool */
    case '+': case '=':
        adjust_person_count(&s->crowd, +CROWD_STEP);         break;
    case '-':
        adjust_person_count(&s->crowd, -CROWD_STEP);         break;

    /* reset: re-init the whole scene at current terminal extent */
    case 'r': case 'R':
        scene_init(s, s->scene_cols, s->scene_rows);         break;

    default: break;
    }
}

/*
 * FpsCounter — rolling-window frame-rate estimator.
 *
 * Each frame the loop tallies one count and dt nanoseconds.  Once the
 * window fills (FPS_UPDATE_MS), `display` updates to count / seconds and
 * the accumulators reset.  Keeps the HUD readable instead of flickering.
 */
typedef struct {
    int     frame_count;     /* frames seen this window */
    int64_t window_ns;       /* time accumulated this window, ns */
    double  display;         /* smoothed fps shown in HUD */
} FpsCounter;

static void fps_counter_init(FpsCounter *f)
{
    f->frame_count = 0;
    f->window_ns   = 0;
    f->display     = 0.0;
}

/* Tally one frame; emit a fresh smoothed reading once the window fills. */
static void fps_counter_tick(FpsCounter *f, int64_t dt)
{
    const int64_t FPS_WINDOW_NS = (int64_t)FPS_UPDATE_MS * NS_PER_MS;

    f->frame_count++;
    f->window_ns += dt;
    if (f->window_ns < FPS_WINDOW_NS) return;

    f->display     = (double)f->frame_count
                   / ((double)f->window_ns / (double)NS_PER_SEC);
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * drain_sim_accumulator — fixed-step physics: replay scene_tick(dt_sec)
 * once per pending tick_ns slot in the accumulator (Fiedler "Fix Your
 * Timestep!" — decouples sim rate from render rate).
 */
static void drain_sim_accumulator(Scene *s, int64_t *sim_accum,
                                   int64_t tick_ns, float dt_sec)
{
    while (*sim_accum >= tick_ns) {
        scene_tick(s, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/*
 * main — the game loop (identical structure to framework.c).
 *
 * Each iteration:
 *   (1) honour pending SIGWINCH resize (reload extent, reset timers)
 *   (2) measure dt, capped at DT_CAP_NS to prevent physics avalanche
 *   (3) drain accumulator: run scene_tick once per pending fixed step
 *   (4) compute sub-tick alpha = leftover / tick_ns  ∈ [0, 1)
 *   (5) tally rolling-window fps counter
 *   (6) sleep BEFORE render so terminal I/O stays inside the budget
 *   (7) frame_render — erase → scene → HUD → doupdate
 *   (8) poll input (non-blocking)
 */
int main(void)
{
    /* (a) RNG + atexit + POSIX signal handlers */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    /* (b) ncurses bring-up and Scene initial state */
    Scene scene;
    screen_init(&scene);
    scene_init(&scene, scene.scene_cols, scene.scene_rows);

    /* (c) loop-local constants and time accumulators */
    const int64_t DT_CAP_NS       = 100 * NS_PER_MS;             /* avalanche guard */
    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;     /* render cadence  */

    int64_t    frame_time = clock_ns();
    int64_t    sim_accum  = 0;
    FpsCounter fps;
    fps_counter_init(&fps);

    while (g_running) {
        int64_t frame_start = clock_ns();

        /* (1) handle pending resize: reload extent, reset timers */
        if (g_need_resize) {
            handle_resize_if_pending(&scene);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* (2) measure dt since last frame, capped to avoid physics avalanche */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;

        /* (3) drain accumulator: fixed-step physics until caught up */
        const int64_t tick_ns = TICK_NS(scene.sim.fps);
        const float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
        sim_accum += dt;
        drain_sim_accumulator(&scene, &sim_accum, tick_ns, dt_sec);

        /* (4) sub-tick interpolation factor for renderer */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* (5) rolling-window fps counter */
        fps_counter_tick(&fps, dt);

        /* (6) sleep before render so I/O time doesn't eat the budget */
        int64_t budget_left = FRAME_BUDGET_NS - (clock_ns() - frame_start);
        clock_sleep_ns(budget_left);

        /* (7) render: scene + HUD, flushed in one diff */
        frame_render(&scene, fps.display, alpha);

        /* (8) input (non-blocking getch) */
        int key = getch();
        if (key != ERR) handle_input(&scene, key);
    }

    return 0;
}
