/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * flocking.c — five switchable flocking algorithms across three flocks
 *
 * DEMO: Three flocks, each led by a wandering leader, share the screen and
 *       run simultaneously. Press 1-5 to switch between Reynolds boids,
 *       leader chase, Vicsek noise-driven alignment, orbit formation, and
 *       predator-prey hunt-flee. Boids wrap toroidally at the screen edges
 *       so flocking behaviour is uniform everywhere.
 *
 * Study alongside: flocking/crowd.c (the same forces in single-flock form)
 *
 * Section map:
 *   §1 config   — tunables: counts, speeds, radii, weights, mode params
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — palette_xterm256 / palette_ansi8 + PAIR_HUD/PAIR_HINT
 *   §4 coords   — pixel↔cell aspect bridge + World pixel-extent struct
 *   §5 boid     — rand_uniform_* helpers; Boid struct; spawn (rejection
 *                 sample); speed clamp; torus wrap; toroidal_delta;
 *                 velocity_dir_char (8-sector compass)
 *   §6 flock    — leader: snap_angle_to_bearing +
 *                 integrate_leader_at_constant_speed + leader_tick;
 *                 force primitives: clamp2d / add_separation_force /
 *                 seek_force_toward;
 *                 boids: NeighbourSums + scan_perception_neighbours +
 *                 accumulate_{separation,alignment,cohesion}_force +
 *                 accumulate_leader_pull + boids_steer;
 *                 chase_steer; vicsek: sum_velocities_in_perception +
 *                 vicsek_steer; orbit: orbit_target_for_slot +
 *                 orbit_steer; flock_tick (compute_steer_for_mode +
 *                 apply_followers_step); flock_init (with
 *                 starting_quadrant_origin / pick_scattered_spawn_pos /
 *                 pick_cruise_speed_variation)
 *   §7 scene    — SimControls + Scene (= flocks + sim + world);
 *                 predator: predator_find_nearest_prey +
 *                 predator_leader_tick; prey: accumulate_flee_from_one
 *                 + accumulate_flee_from_predator + prey_boids_steer;
 *                 scene_tick_predator (tick_predator_pack +
 *                 tick_one_prey_flock); scene_tick; renderer:
 *                 follower_brightness, RenderTiming +
 *                 extrapolate_to_render_pos + clamp_to_world_box +
 *                 paint_glyph_at_cell + draw_boid + scene_draw
 *   §8 screen   — ncurses double-buffer display layer; HUD: count_total_boids
 *                 + format_hud_status + hud_paint_text +
 *                 draw_hud_status / draw_hud_hint; screen_draw
 *   §9 app      — FpsCounter; App (= scene + screen + fps + sig flags);
 *                 signals, app_do_resize (clamp_boid_inside_world +
 *                 clamp_all_boids_to_world), app_handle_key
 *                 (adjust_vicsek_noise + adjust_follower_count_all_flocks),
 *                 main game loop
 *
 * Keys:
 *   q / ESC    quit                       space    pause / resume
 *   1          BOIDS    (sep+align+coh)   2        CHASE   (leader homing)
 *   3          VICSEK   (align+noise)     4        ORBIT   (spinning ring)
 *   5          PREDATOR (hunt/flee)
 *   n / m      Vicsek noise − / +         + / -    add / remove a follower
 *   r          reset all flocks to starting quadrants
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra flocking.c -o flocking -lncurses -lm
 */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 *   ── Original behavioural-steering papers ───────────────────────
 *   [1] Reynolds, C. W. (1987), "Flocks, Herds, and Schools: A
 *       Distributed Behavioral Model", SIGGRAPH '87 Computer Graphics
 *       21(4), pp. 25-34 — the ORIGINAL boids paper.  Introduces
 *       separation + alignment + cohesion as the three local rules
 *       sufficient to generate flocking; coins "boid" (bird-oid).
 *       This file's MODE_BOIDS is a direct implementation.
 *   [2] Reynolds, C. W. (1999), "Steering Behaviors for Autonomous
 *       Characters", Game Developers Conference 1999.  Online at
 *       https://www.red3d.com/cwr/steer/ — the canonical steering
 *       primer: seek, flee, arrive, pursue, evade, wander, separate,
 *       align, cohere.  This file's chase_steer and the leader-pull
 *       term in boids_steer are direct subsets.
 *
 *   ── Statistical-physics flocking models ────────────────────────
 *   [3] Vicsek, T., Czirók, A., Ben-Jacob, E., Cohen, I. & Shochet, O.
 *       (1995), "Novel type of phase transition in a system of self-
 *       driven particles", Phys. Rev. Lett. 75(6), pp. 1226-1229 —
 *       canonical Vicsek model: align-with-neighbours + noise.
 *       Establishes flocking as a continuous phase transition; the
 *       n / m noise keys in MODE_VICSEK slide the system across that
 *       transition live.
 *   [4] Toner, J. & Tu, Y. (1995, 1998), "Long-Range Order in a
 *       Two-Dimensional Dynamical XY Model" (PRL 75, pp. 4326-4329) +
 *       "Flocks, herds, and schools" (Phys. Rev. E 58, pp. 4828-4858)
 *       — hydrodynamic continuum theory of flocking; the macroscopic
 *       field theory behind the Vicsek microscopic model.  Read after
 *       [3] when you want to understand WHY the phase transition is
 *       sharp.
 *
 *   ── Collective-behaviour biology ───────────────────────────────
 *   [5] Couzin, I. D., Krause, J., James, R., Ruxton, G. D. & Franks,
 *       N. R. (2002), "Collective memory and spatial sorting in
 *       animal groups", J. Theor. Biol. 218(1), pp. 1-11 — extends
 *       boids with radius-based zones of repulsion / orientation /
 *       attraction.  Justifies SEPARATION_RADIUS < PERCEPTION_RADIUS
 *       and motivates the ~1:3 ratio used here.
 *
 *   ── Predator–prey dynamics ─────────────────────────────────────
 *   [6] Helbing, D., Farkas, I. & Vicsek, T. (2000), "Simulating
 *       dynamical features of escape panic", Nature 407(6803),
 *       pp. 487-490 — applies the social-force model to a hazard +
 *       fleeing crowd.  MODE_PREDATOR is a simplification: predator
 *       acts as the hazard, prey use boids-with-flee instead of full
 *       social forces.
 *
 *   ── Implementation-oriented textbooks ──────────────────────────
 *   [7] Shiffman, D., "The Nature of Code", Ch. 6 — reading-level
 *       walkthrough of Reynolds steering with Processing examples.
 *       The single most accessible introduction; pair with [2].
 *       https://natureofcode.com/book/chapter-6-autonomous-agents/
 *   [8] Millington, I. & Funge, J. (2019), "Artificial Intelligence
 *       for Games" (3rd ed.), CRC Press — Ch. 3 covers every steering
 *       primitive with pseudocode + integration math; the pursue and
 *       evade sections inform MODE_PREDATOR's bearing-snap pursuit.
 *
 *   ── Game-loop / rendering interpolation ────────────────────────
 *   [9] Fiedler, G. (2004, updated 2014), "Fix Your Timestep!",
 *       https://gafferongames.com/post/fix_your_timestep/ — the
 *       fixed-step accumulator + sub-tick interpolation pattern
 *       implemented by main() (drains sim_accum until caught up) and
 *       scene_draw() / draw_boid() (alpha-lerp via RenderTiming).
 *       Explains why prev/curr + alpha lerp beats variable timestep
 *       at low render rates.
 *
 *   ── Online quick reference ─────────────────────────────────────
 *  [10] Wikipedia "Boids" + "Vicsek model" —
 *       https://en.wikipedia.org/wiki/Boids and
 *       https://en.wikipedia.org/wiki/Vicsek_model — concise summaries
 *       of the rules, history, and implementations in every major
 *       language.
 *
 *   ── Companion file in this project ─────────────────────────────
 *   See also: flocking/crowd.c — the same Reynolds toolbox applied to
 *     a SINGLE-flock crowd with six behaviour modes (WANDER, FLOCK,
 *     PANIC, GATHER, FOLLOW, QUEUE).  Read it after this file when
 *     you want to see how the same primitives compose into different
 *     emergent behaviours.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Every boid is a particle with a constant cruise speed; what changes is
 * *direction*. Each tick, a boid sums steering forces (separation,
 * alignment, cohesion, leader pull, flee, …), adds them to its velocity,
 * clamps the magnitude, integrates position. The flock as a whole is
 * just an emergent pattern in those independent local rules — no
 * scheduler, no formation table, no central planner. Switching modes
 * (1-5) only changes which forces are summed; the integration is the
 * same.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine three swarms of ASCII fish, each chasing its own slightly
 * faster glider (the leader). Every fish has three rules in its head at
 * once — "don't bump into neighbours", "swim parallel to neighbours",
 * "drift toward the centre of nearby neighbours" — and a quieter pull
 * toward its own leader. Add these tugs together, point in their
 * direction, swim. That single recipe with different weight choices
 * produces all five modes: pure boids (1), drop alignment+cohesion and
 * dial up leader pull (2), throw out everything except a mean-heading
 * align with random noise (3), replace the leader pull with "go to slot
 * i on the rotating ring" (4), or add a strong flee force per visible
 * predator (5).
 *
 * ALGORITHM IN STEPS  (per tick, per flock)
 * ─────────────────────────────────────────
 *   1. leader_tick: jitter the leader's heading by ±WANDER_JITTER rad,
 *      derive velocity, integrate position, wrap toroidally.
 *   2. Stage 1 — read-only: for every follower compute new_v[i] using
 *      the active mode's steering function. Reads OLD positions only.
 *   3. Stage 2 — write: for every follower copy new_v[i] into vel,
 *      clamp speed, integrate position, wrap toroidally.
 *
 * Two stages so no boid reacts to an already-moved neighbour — without
 * this, the flock drifts in array order over time.
 *
 * KEY FORMULAS
 * ────────────
 *   Toroidal delta (shortest signed displacement on a wrapped axis):
 *     d = b − a
 *     if d >  extent/2 : d −= extent
 *     if d < −extent/2 : d += extent
 *
 *   Boids separation (per neighbour with 0 < d < SEPARATION_RADIUS):
 *     closeness = (SEPARATION_RADIUS − d) / SEPARATION_RADIUS
 *     sep_x    -= (dx/d) · closeness                     (sum over neighbours)
 *     sep_y    -= (dy/d) · closeness
 *     steer    += W_SEPARATION · normalise(sep) · MAX_STEER
 *
 *   Boids alignment (per neighbour with d < PERCEPTION_RADIUS):
 *     desired = normalise(mean_neighbour_velocity) · cruise_speed
 *     steer  += W_ALIGNMENT · (desired − vel)
 *
 *   Boids cohesion (toroidal centre of mass — accumulate offsets, not
 *   absolute positions, because a wrapped torus makes naïve averaging
 *   meaningless):
 *     desired = normalise(mean_neighbour_offset) · cruise_speed
 *     steer  += W_COHESION  · (desired − vel)
 *
 *   Vicsek update:
 *     avg_angle = atan2(Σ neighbour_vy, Σ neighbour_vx)
 *     new_v     = cruise_speed · (cos, sin)(avg_angle + η)   (η ∈ [−noise,
 * noise])
 *
 *   Pixel→cell aspect bridge:
 *     cx = round(px / CELL_W)        (CELL_W = 8)
 *     cy = round(py / CELL_H)        (CELL_H = 16)
 *
 * Background you need
 * ───────────────────
 *   - Pixel-space simulation (CLAUDE.md: physics in pixels, render
 *     converts to cells in §7 scene_draw).
 *   - Vector arithmetic on Vec2 floats.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Spatial hashing / kd-trees. We do O(N²) neighbour scan;
 *     small N keeps it microseconds.
 *   - Quaternions. 2-D motion uses (cos, sin) on a scalar
 *     heading.
 *   - GPU compute / parallel boids. Single-threaded CPU is fine
 *     up to ~1000 boids in a terminal.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/*
 * _GNU_SOURCE exposes M_PI from <math.h>.
 * _POSIX_C_SOURCE alone does not include M_PI (it is a POSIX extension).
 */
#define _GNU_SOURCE

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

/*
 * Terminal cell geometry.
 * Physics lives in pixel space where 1 unit ≈ 1 physical pixel.
 * Terminal cells are roughly twice as tall as wide (CELL_H / CELL_W = 2),
 * so we give each cell 8 horizontal and 16 vertical sub-pixels.
 * All physics uses pixel units; only the final draw step converts to cells.
 * This makes diagonal motion and speed look correct on screen.
 */
#define CELL_W 8  /* sub-pixel columns per terminal cell */
#define CELL_H 16 /* sub-pixel rows    per terminal cell */

/*
 * Simulation and render rates.
 * Fixed-timestep accumulator decouples physics accuracy from wall-clock FPS.
 */
enum {
  SIM_FPS = 60,        /* physics ticks per second               */
  RENDER_FPS = 60,     /* display frame cap                      */
  FPS_UPDATE_MS = 500, /* HUD fps counter refresh interval (ms)  */
};

/*
 * Flock counts.
 * Three flocks → three cosine hue phases spaced 120° apart (0, 1/3, 2/3),
 * keeping flocks visually distinct as the palette slowly cycles.
 * 12 followers per flock gives ~39 total boids — enough for visible
 * flocking without O(n²) neighbor search becoming noticeable.
 */
enum {
  FLOCKS = 3,
  FOLLOWERS_DEFAULT = 12,
  FOLLOWERS_MIN = 3,
  FOLLOWERS_MAX = 20,
};

/*
 * Boid speeds in pixels per second.
 *
 * Smoothness threshold (from bounce.c):
 *   A boid must cross at least one cell boundary every ~4 ticks to avoid
 *   staircase artifacts. For vertical cells (taller dimension):
 *     speed_min ≥ CELL_H × SIM_FPS / 4 = 16 × 60 / 4 = 240 px/s
 * We set BOID_SPEED above that floor with a comfortable margin.
 * LEADER_SPEED is slightly higher so the leader naturally pulls ahead.
 */
#define BOID_SPEED 280.0f   /* follower cruising speed, px/s */
#define LEADER_SPEED 340.0f /* leader cruising speed, px/s   */
#define MIN_SPEED 80.0f     /* floor — boids never fully stop */
#define MAX_SPEED 500.0f    /* ceiling — keeps flocks legible */

/*
 * Boid perception radii in pixel space.
 * PERCEPTION_RADIUS: how far a boid "sees" neighbors for all three boids rules.
 * SEPARATION_RADIUS: closer range where boids push apart.
 *   Set to ~1/3 of PERCEPTION_RADIUS so boids have a "personal space" zone
 *   while still responding to the wider group outside it.
 */
#define PERCEPTION_RADIUS 180.0f /* px — neighbor sensing range   */
#define SEPARATION_RADIUS 60.0f  /* px — minimum comfortable gap  */

/*
 * Boids steering weights — scale each rule's contribution to the force vector.
 *
 * Separation is strongest to prevent crowding.
 * Alignment is moderate so boids match direction without locking in formation.
 * Cohesion is weakest so groups stay loose rather than collapsing to a point.
 * Leader pull is gentle — it keeps the group near its leader without
 * overriding the emergent boids behavior among followers.
 */
#define W_SEPARATION 1.8f  /* repulsion from nearby boids       */
#define W_ALIGNMENT 1.0f   /* match average heading of group    */
#define W_COHESION 0.5f    /* drift toward group center of mass */
#define W_LEADER_PULL 0.4f /* gentle attraction toward leader   */

/*
 * Maximum steering force applied per tick (px/s added to velocity).
 * Clamping the turn rate makes boids curve smoothly rather than snap.
 */
#define MAX_STEER 130.0f

/*
 * Leader wander — smooth random walk on the heading angle.
 * Each tick the leader's heading shifts by up to ±WANDER_JITTER radians,
 * producing gently curved paths instead of sharp random turns.
 */
#define WANDER_JITTER 0.10f /* max heading change per tick (radians) */

/*
 * Vicsek model parameters.
 *
 * In the Vicsek model (1995) each boid aligns to its neighbors' average
 * heading and then adds a random noise angle. This single parameter drives
 * a phase transition between two regimes:
 *   low noise  → all boids stream in one direction (ordered, polarised)
 *   high noise → every boid moves independently (disordered, chaotic)
 *
 * VICSEK_NOISE is the maximum random angle perturbation per tick (radians).
 * 0.30 rad ≈ 17° — enough to see occasional bursts of disorder without
 * making the flock completely random at normal density.
 * Press 'n' / 'm' at runtime to decrease / increase noise interactively.
 */
#define VICSEK_NOISE_DEFAULT 0.30f /* starting noise level, radians     */
#define VICSEK_NOISE_MIN 0.05f     /* near-perfect order (parallel beams) */
#define VICSEK_NOISE_MAX 1.80f     /* near-random (barely any coherence) */
#define VICSEK_NOISE_STEP 0.10f    /* per keypress increment            */

/*
 * Orbit formation parameters.
 *
 * Followers are assigned evenly-spaced slots on a ring of radius ORBIT_RADIUS
 * centred on their leader. The ring rotates at ORBIT_SPEED rad/s, so each
 * boid's target position moves along the ring — it chases a moving point,
 * producing a spinning halo effect.
 *
 * Tangential ring speed = ORBIT_RADIUS × ORBIT_SPEED = 120 × 1.4 ≈ 168 px/s.
 * BOID_SPEED (280 px/s) > 168 px/s, so followers always keep up with their
 * assigned slot.
 */
#define ORBIT_RADIUS 120.0f /* px — ring radius around leader         */
#define ORBIT_SPEED 1.4f    /* rad/s — ring rotation speed            */

/*
 * Predator-Prey parameters.
 *
 * Flock 0 is the predator; flocks 1 and 2 are prey.
 *
 * PREDATOR_CHASE_RADIUS: how far the predator leader can "see" prey boids.
 *   When a prey boid enters this range the leader steers toward the nearest
 *   one; outside the range it wanders normally.
 *
 * PREY_FLEE_RADIUS: how close a predator boid must be before prey start
 *   fleeing. Larger than separation radius so prey react before contact.
 *
 * W_FLEE: flee force weight. Must be substantially stronger than cohesion
 *   (0.5) so prey actually break out of their flock rather than merely
 *   jostling and staying put.
 */
#define PREDATOR_CHASE_RADIUS 280.0f /* px — predator sight range        */
#define PREY_FLEE_RADIUS 160.0f      /* px — prey panic zone             */
#define W_FLEE 3.0f                  /* flee force weight (beats cohesion) */

/* Common time constants */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(fps) (NS_PER_SEC / (fps))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Each flock has TWO color pairs: one for followers, one for the leader.
 * Leader color contrasts its flock so it always pops visually.
 *
 * We use xterm-256 when available for the three vivid flock colors —
 * orange in particular has no equivalent in the 16-color palette.
 * 16-color fallbacks are provided for basic terminals.
 *
 * Follower colors (xterm-256 indices):
 *   flock 0 — matrix green  : 46   (#00ff00  pure bright green)
 *   flock 1 — fire orange   : 208  (#ff8700  deep amber-orange)
 *   flock 2 — electric blue : 33   (#0087ff  dodger blue)
 *
 * Leader colors — chosen to be complementary / high-contrast:
 *   flock 0 green  leader — bright yellow : 226  (warm, pops on green)
 *   flock 1 orange leader — ice cyan      : 51   (cool complement to orange)
 *   flock 2 blue   leader — bright white  : 231  (neutral, clean on blue)
 *
 * 16-color fallbacks (when COLORS < 256):
 *   followers: COLOR_GREEN / COLOR_RED / COLOR_BLUE
 *   leaders:   COLOR_YELLOW / COLOR_CYAN / COLOR_CYAN
 *
 * Color pair layout:
 *   pairs 1 … FLOCKS          — follower colors
 *   pairs FLOCKS+1 … 2*FLOCKS — leader colors
 *   pair  2*FLOCKS + 1        — PAIR_HUD  (top status — bright yellow)
 *   pair  2*FLOCKS + 2        — PAIR_HINT (bottom key hint — bright cyan)
 */

/* xterm-256 follower colors */
#define COLOR_256_MATRIX_GREEN 46 /* #00ff00 — pure bright green         */
#define COLOR_256_FIRE_ORANGE 208 /* #ff8700 — deep amber-orange         */
#define COLOR_256_ELECTRIC_BLUE                                                \
  33 /* #0087ff — vivid dodger blue;                                         \
      * violet (93) was too dark on black                                      \
      * backgrounds — blue-purples lose                                      \
      * brightness fast on 256-color palettes.                                 \
      * Dodger blue sits in the middle of the                                  \
      * blue ramp where luminance is highest. */

/* xterm-256 leader colors — complementary to their flock */
#define COLOR_256_LEADER_YELLOW 226 /* #ffff00 — warm yellow  (vs green) */
#define COLOR_256_LEADER_CYAN 51    /* #00ffff — ice cyan     (vs orange) */
#define COLOR_256_LEADER_WHITE                                                 \
  231 /* #ffffff — bright white (vs blue,                                    \
       * neutral contrast works well here                                      \
       * because blue is already saturated) */

static int follower_color_pair(int flock_idx) { return flock_idx + 1; }
static int leader_color_pair(int flock_idx) { return flock_idx + 1 + FLOCKS; }
#define PAIR_HUD (2 * FLOCKS + 1)  /* bright yellow — top status   */
#define PAIR_HINT (2 * FLOCKS + 2) /* bright cyan   — bottom hint  */

/*
 * color_init — register all ncurses colour pairs.
 *
 * Background = -1 (terminal default) so the demo respects the user's
 * terminal theme.  use_default_colors() must be called once before any
 * init_pair(.., .., -1) so ncurses understands -1 as "the default".
 *
 * All foreground tints sit in the bright half of the 256-colour cube
 * (≥ 33) per the project palette-brightness rule — even leaders that
 * read as "neutral" use 231 (near-white) rather than the dark grey
 * region.  HUD pairs use the canonical bright yellow + cyan combo.
 */
/* palette_xterm256 — vivid 256-colour follower + leader + HUD pairs.
 * Every index sits in the BRIGHT half (≥ 33) per the project palette-
 * brightness rule so A_DIM stays legible against default-black. */
static void palette_xterm256(void) {
  enum { HUD_YELLOW = 226, HUD_CYAN = 51 };

  init_pair(follower_color_pair(0), COLOR_256_MATRIX_GREEN,  -1);
  init_pair(follower_color_pair(1), COLOR_256_FIRE_ORANGE,   -1);
  init_pair(follower_color_pair(2), COLOR_256_ELECTRIC_BLUE, -1);

  init_pair(leader_color_pair(0),   COLOR_256_LEADER_YELLOW, -1);
  init_pair(leader_color_pair(1),   COLOR_256_LEADER_CYAN,   -1);
  init_pair(leader_color_pair(2),   COLOR_256_LEADER_WHITE,  -1);

  init_pair(PAIR_HUD,  HUD_YELLOW, -1);   /* bright yellow — top status */
  init_pair(PAIR_HINT, HUD_CYAN,   -1);   /* bright cyan   — key hint   */
}

/* palette_ansi8 — 16-colour fallback; closest equivalents. */
static void palette_ansi8(void) {
  init_pair(follower_color_pair(0), COLOR_GREEN, -1);
  init_pair(follower_color_pair(1), COLOR_RED,   -1);
  init_pair(follower_color_pair(2), COLOR_BLUE,  -1);

  init_pair(leader_color_pair(0),   COLOR_YELLOW, -1);
  init_pair(leader_color_pair(1),   COLOR_CYAN,   -1);
  init_pair(leader_color_pair(2),   COLOR_CYAN,   -1);

  init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
  init_pair(PAIR_HINT, COLOR_CYAN,   -1);
}

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) palette_xterm256();
  else               palette_ansi8();
}

/* ===================================================================== */
/* §4  coords + world — the one place aspect ratio is handled            */
/* ===================================================================== */

/*
 * pw / ph — pixel space dimensions given terminal cell dimensions.
 * Everything in physics uses pixel units; only scene_draw converts back.
 */
static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

/*
 * World — pixel-space extent of the simulation box.
 *
 * Intent
 *   Every physics routine needs "the size of the world" to do TWO
 *   things, both fundamental to flocking on a finite domain:
 *
 *     1. Toroidal wrapping (boid_wrap):
 *          if px <  0      → px += width
 *          if px >= width  → px -= width
 *        A boid leaving one edge re-enters at the opposite edge.  This
 *        eliminates "corner cases" where a boid has no neighbours on
 *        one side and starts behaving differently than the rest of the
 *        flock — flocking density stays uniform everywhere.
 *
 *     2. Shortest-path distance (toroidal_delta):
 *          d = b − a
 *          if |d| > extent/2 : take the wrap path instead
 *        Without this, boids at px=5 and px=width−5 would consider
 *        themselves "far apart" even though they're 10 px apart across
 *        the wrap boundary.  Used by every steering primitive
 *        (separation, alignment, cohesion, leader pull, predator chase).
 *
 *   Before this struct, every signature carried a separate (max_px,
 *   max_py) pair — 24+ functions, all threading the same two floats.
 *   Bundling them into one named type makes signatures read in the
 *   physics vocabulary: `boid_wrap(b, world)` instead of
 *   `boid_wrap(b, max_px, max_py)`.
 *
 * Why pixel-space only (no cols/rows)
 *   Terminal cell extent (cols, rows) is a SCREEN concern, not a
 *   SIMULATION concern.  Cell extent stays on the Screen struct in §8
 *   and is passed explicitly to the renderer where it is needed for
 *   bounds-checking after px_to_cell_x/y conversion.  Physics never
 *   touches cells — keeping these two spaces disjoint in the type
 *   system prevents the entire class of "rendered too early in cell
 *   space" aspect-ratio bugs (see CLAUDE.md §4 Coordinate / Physics).
 *
 * Members
 *   width   x-extent in pixels = cols × CELL_W.  All physics x-positions
 *           live in [0, width); positions at exactly `width` are wrapped
 *           back to 0 by boid_wrap.  CELL_W=8 sub-pixels per terminal
 *           column gives smooth horizontal motion at SIM_FPS=60 Hz.
 *   height  y-extent in pixels = rows × CELL_H.  Same semantics on the
 *           vertical axis.  CELL_H=16 (twice CELL_W) compensates for the
 *           terminal cell aspect ratio so a diagonal `vel = (k, k)`
 *           travels visually equal distance in x and y.
 *
 * Invariants
 *   width > 0 AND height > 0 (set by world_from_terminal).
 *   width  == cols × CELL_W,  height == rows × CELL_H, kept in sync
 *   with Screen.cols / Screen.rows by scene_init / app_do_resize.
 *   All boid positions satisfy 0 ≤ px < width and 0 ≤ py < height
 *   after every boid_wrap() call.
 *
 * References
 *   [2] Reynolds 1999 — every steering primitive operates on a
 *       displacement vector to a neighbour; on a torus, that vector
 *       must use the shortest-path (toroidal_delta) form to match
 *       the wrapping physics.
 *   [3] Vicsek et al. 1995 — the original Vicsek model is defined on
 *       a torus precisely so the phase transition isn't perturbed by
 *       boundary effects.  Same reason here.
 */
typedef struct {
    float width;
    float height;
} World;

/*
 * world_from_terminal — derive the pixel-space World from a terminal
 * (cols, rows) reading.  Called once at scene_init and again after every
 * SIGWINCH so the physics box matches the visible terminal extent.
 */
static inline World world_from_terminal(int cols, int rows) {
    return (World){
        .width  = (float)pw(cols),
        .height = (float)ph(rows),
    };
}

/*
 * px_to_cell_x/y — convert pixel coordinate to terminal cell.
 *
 * Uses floorf(px / CELL + 0.5f) — "round half up" — instead of roundf().
 * roundf uses "round half to even" (banker's rounding): a boid sitting
 * exactly on a cell boundary oscillates between two cells every frame,
 * producing a flickering doubled character. Adding 0.5 before floor breaks
 * the tie in one consistent direction, eliminating the flicker.
 */
static inline int px_to_cell_x(float px) {
  return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py) {
  return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  boid                                                               */
/* ===================================================================== */

/* ── random-uniform helpers ────────────────────────────────────────── */
/*
 * rand_uniform_01 / rand_uniform_signed / rand_uniform_angle_2pi
 *
 * Three named samplers for the three uniform distributions this file
 * actually uses.  The OLD `(rand() % 2001) / 1000.0f - 1.0f` idiom
 * appeared in six functions verbatim; readers had to recognise the
 * pattern by eye each time.  Naming each variant makes intent obvious
 * at every call site.
 *
 * Implementation notes
 *   - rand() is the C standard PRNG, seeded by srand() in main().
 *     It is not cryptographically strong but is adequate for visual
 *     stochastic motion.  Period ≥ 2³² on any sensible libc.
 *   - The (%N) / N idiom introduces a tiny low-bit bias when RAND_MAX
 *     is not a multiple of N; for visual jitter this is invisible.
 *   - Resolution: 1/1000 ≈ 0.001 — finer than any visible motion
 *     change at 60 Hz, coarse enough that the modulus is cheap.
 */

/* Returns a uniform sample in [0.0, 1.0] (inclusive at both ends). */
static inline float rand_uniform_01(void) {
  enum { RAND_DENOM = 1000 };
  return (float)(rand() % (RAND_DENOM + 1)) / (float)RAND_DENOM;
}

/* Returns a uniform sample in [-1.0, +1.0].  Used for jitter terms. */
static inline float rand_uniform_signed(void) {
  enum { RAND_DENOM = 1000 };
  return (float)(rand() % (2 * RAND_DENOM + 1) - RAND_DENOM) / (float)RAND_DENOM;
}

/* Returns a uniform angle in [0, 2π).  Used for random initial heading. */
static inline float rand_uniform_angle_2pi(void) {
  enum { ANGLE_TICKS = 6283 }; /* round(2π × 1000) = 6283.18..  */
  return (float)(rand() % (ANGLE_TICKS + 1)) / 1000.0f;
}

/*
 * Boid — one agent's complete kinematic state (Reynolds [1]: "bird-oid").
 *
 * Intent
 *   The atomic unit of every flocking algorithm in this file.  A Boid
 *   is a point particle on a torus with three pieces of state — WHERE
 *   you are, WHICH WAY and HOW FAST you are moving, and HOW FAST you
 *   LIKE to move.  Steering forces accumulate into vx/vy each tick;
 *   px/py integrate from vx/vy; cruise_speed stays constant after
 *   spawn and feeds the alignment / cohesion / chase rules with the
 *   "desired speed" they need.
 *
 * Why these three concepts and no more
 *   • No mass — every boid is implicitly mass 1, so steering force in
 *     px/s² is added directly into velocity in px/s without an F/m
 *     division.  Reynolds [2] uses this same simplification: it makes
 *     weights tunable as raw scalars rather than per-mass forces.
 *   • No heading angle stored separately — heading is implicit in
 *     atan2(vy, vx) and re-derived per draw call in velocity_dir_char.
 *     Avoids the desync risk of an angle that has to be kept in sync
 *     with velocity components every tick.  (The LEADER does store an
 *     angle in Flock.wander_angle because its motion is constructed
 *     FROM the angle, not derived from it.)
 *   • No mode-specific scratch — orbit_phase, vicsek_noise, etc. live
 *     on the containing Flock / Scene structures so Boid stays a pure
 *     kinematic point usable by every steering algorithm.
 *
 * Why per-boid cruise_speed (and not a shared global)
 *   Followers spawn with cruise_speed = BOID_SPEED × (0.85 .. 1.15),
 *   i.e. ±15 % variation.  This single line of jitter is what gives
 *   the flock its organic depth: fast boids push toward the front
 *   edge, slow ones drift to the rear, producing a 3-D-looking shape
 *   in a 2-D simulation.  Without it, alignment locks everyone to the
 *   same speed and the flock looks like a solid sheet.  See [5]
 *   Couzin et al. 2002 §"Spatial sorting" for the biological analogue
 *   (fish in a school sort by speed preference).
 *
 * Why PIXEL space everywhere
 *   Storing px/py in cell coordinates would lose sub-cell motion —
 *   a boid at SPEED_BASE=280 px/s advances ~4.7 px/tick at 60 Hz
 *   (CELL_W=8 means about 0.6 cell/tick).  In cell space this becomes
 *   "stay in cell A for one tick, jump to cell B the next" — visible
 *   staircase artefacts.  Float pixel space + alpha-lerp at draw
 *   produces continuous motion.  See CLAUDE.md §"Coordinate / Physics".
 *
 * Members
 *   px, py        position in PIXEL space, pixel units (px).
 *                 INVARIANT: 0 ≤ px < World.width and 0 ≤ py <
 *                 World.height after every boid_wrap() — the torus
 *                 wraps anything that strays outside.  Cell-space
 *                 rendering recovers (col, row) via px_to_cell_x/y.
 *
 *   vx, vy        velocity in PIXELS PER SECOND.  Integration rule:
 *                   vel  += steer · dt   (steering accumulator)
 *                   pos  += vel  · dt   (Euler integration)
 *                 Magnitude is enforced into [MIN_SPEED, MAX_SPEED]
 *                 by boid_clamp_speed every tick:
 *                   - MIN_SPEED prevents stall when opposing forces
 *                     cancel (a stalled boid loses its direction and
 *                     can't recover).
 *                   - MAX_SPEED prevents runaway acceleration when
 *                     many separation pushes stack.
 *
 *   cruise_speed  personal "desired" speed in px/s, set ONCE at spawn
 *                 in boid_spawn_at as BOID_SPEED × (0.85..1.15) for
 *                 followers, LEADER_SPEED exactly for leaders.  Read
 *                 by alignment/cohesion/chase to compute the desired-
 *                 velocity vector (desired = unit(direction) · cruise).
 *                 Never mutated after spawn — it IS the boid's
 *                 personality.
 *
 * Invariants
 *   |vel| ∈ [MIN_SPEED, MAX_SPEED] after boid_clamp_speed.
 *   0 ≤ px < World.width AND 0 ≤ py < World.height after boid_wrap.
 *   cruise_speed > 0; set once at spawn, never mutated thereafter.
 *
 * References
 *   [1] Reynolds 1987 — the original boid: point particle with
 *       position + velocity that accumulates steering forces.
 *   [2] Reynolds 1999 §"Vehicle Model" — same kinematic model used
 *       here (force → velocity → position, with magnitude caps).
 *   [5] Couzin et al. 2002 — per-individual speed preferences as the
 *       biological motivation for cruise_speed variation.
 *   [7] Shiffman *Nature of Code* Ch.6 — same Vehicle data layout in
 *       Processing pseudocode; a great companion read for this struct.
 */
typedef struct {
  float px, py;       /* position, pixel space (px); torus-wrapped       */
  float vx, vy;       /* velocity, px/s; clamped to [MIN_SPEED,MAX_SPEED]*/
  float cruise_speed; /* desired speed, px/s; ±15 % per follower (depth) */
} Boid;

/*
 * velocity_dir_char — pick an ASCII glyph for the boid's direction of travel.
 *
 * Each flock uses its own diagonal characters so flocks stay visually
 * identifiable when they intermix — you can distinguish them by shape,
 * not just color. The four cardinal glyphs (< ^ > v) are shared since
 * they clearly suggest direction; only the diagonals differ:
 *
 *   flock 0: slash/backslash   < \ ^ / > \ v /   (sharp angles)
 *   flock 1: tilde             < ~ ^ ~ > ~ v ~   (wave-like)
 *   flock 2: plus              < + ^ + > + v +   (cross-like)
 *
 * Sector layout (clockwise from West, +y = down in pixel space):
 *   index: 0   1    2   3    4   5    6   7
 *   dir:   W   NW   N   NE   E   SE   S   SW
 *
 * Formula: atan2(vy,vx) → shift to [0,2π] → center sectors by adding π/8
 *          → divide by sector width π/4 → floor → mod 8 = sector 0–7.
 */
static char velocity_dir_char(float vx, float vy, int flock_idx) {
  enum { N_SECTORS = 8 };                              /* 8-way compass    */
  const float SECTOR_WIDTH      = (float)M_PI / 4.0f;  /* 45° per sector   */
  const float SECTOR_HALF_WIDTH = (float)M_PI / 8.0f;  /* centring offset  */

  static const char k_chars[FLOCKS][N_SECTORS] = {
      {'<', '\\', '^', '/', '>', '\\', 'v', '/'}, /* flock 0: \ and / */
      {'<', '~', '^', '~', '>', '~', 'v', '~'},   /* flock 1: ~       */
      {'<', '+', '^', '+', '>', '+', 'v', '+'},   /* flock 2: +       */
  };

  /* Compass heading in 3 steps:
   *   (1) raw atan2 lives in [−π, π] (West = ±π, East = 0)
   *   (2) shift to [0, 2π) then add half-sector so sectors are CENTRED
   *       on the eight compass directions (W, NW, N, NE, E, SE, S, SW)
   *   (3) divide by sector width and mod 8 → integer sector index */
  float heading_radians         = atan2f(vy, vx);
  float heading_shifted_centred = heading_radians + (float)M_PI + SECTOR_HALF_WIDTH;
  int   sector_index            = (int)floorf(heading_shifted_centred / SECTOR_WIDTH) % N_SECTORS;
  return k_chars[flock_idx][sector_index];
}

/*
 * boid_spawn_at — place a boid at (px, py) with a random direction at
 * the given speed. Uses rejection-sample unit vector to get a uniform
 * random angle distribution without calling atan2/sinf/cosf at spawn.
 */
static void boid_spawn_at(Boid *b, float px, float py, float speed) {
  b->px = px;
  b->py = py;
  b->cruise_speed = speed;

  /* Rejection-sample a uniform-random unit direction:
   *   keep (dx, dy) iff it falls inside the unit disc (exclude origin
   *   and corners of the [-1,+1]² square).  This is uniform in angle
   *   without any atan2/sin/cos call. */
  const float UNIT_DISC_MIN_SQ = 0.01f;   /* skip near-origin */
  const float UNIT_DISC_MAX_SQ = 1.0f;
  float dx, dy, len_sq;
  do {
    dx = rand_uniform_signed();
    dy = rand_uniform_signed();
    len_sq = dx * dx + dy * dy;
  } while (len_sq < UNIT_DISC_MIN_SQ || len_sq > UNIT_DISC_MAX_SQ);

  /* Convert the accepted (dx, dy) sample inside the unit disc into a
   * unit-vector direction, then scale to the requested speed. */
  float sample_length = sqrtf(len_sq);
  float direction_x   = dx / sample_length;
  float direction_y   = dy / sample_length;
  b->vx = direction_x * speed;
  b->vy = direction_y * speed;
}

/*
 * boid_clamp_speed — enforce MIN_SPEED ≤ |velocity| ≤ MAX_SPEED.
 * Without a floor, boids stall when opposing forces cancel out.
 * Without a ceiling, large separation forces cause runaway acceleration.
 */
static void boid_clamp_speed(Boid *b) {
  const float SPEED_EPSILON = 0.001f;        /* near-zero magnitude: stall */
  float current_speed = hypotf(b->vx, b->vy);

  /* (a) full stall: no direction information left — pick +x and continue */
  if (current_speed < SPEED_EPSILON) {
    b->vx = MIN_SPEED;
    b->vy = 0.0f;
    return;
  }

  /* (b) below floor: rescale to MIN_SPEED in the same direction */
  if (current_speed < MIN_SPEED) {
    float dir_x = b->vx / current_speed;
    float dir_y = b->vy / current_speed;
    b->vx = dir_x * MIN_SPEED;
    b->vy = dir_y * MIN_SPEED;
    return;
  }

  /* (c) above ceiling: rescale to MAX_SPEED in the same direction */
  if (current_speed > MAX_SPEED) {
    float dir_x = b->vx / current_speed;
    float dir_y = b->vy / current_speed;
    b->vx = dir_x * MAX_SPEED;
    b->vy = dir_y * MAX_SPEED;
  }
}

/*
 * boid_wrap — toroidal boundary: a boid leaving one edge re-enters at
 * the opposite edge. This means no boid is ever "cornered" with no
 * neighbors on one side, keeping flocking behavior uniform everywhere.
 */
static void boid_wrap(Boid *b, World world) {
  if (b->px < 0.0f)
    b->px += world.width;
  if (b->px >= world.width)
    b->px -= world.width;
  if (b->py < 0.0f)
    b->py += world.height;
  if (b->py >= world.height)
    b->py -= world.height;
}

/*
 * toroidal_delta — shortest signed displacement from position a to b
 * on a single axis of length `extent`.
 *
 * On a toroidal (wrap-around) axis, there are two paths between any two
 * points: the "direct" path and the "wrap" path. We always return the
 * shorter one, with sign indicating direction.
 *
 * Example (extent=100): a=5, b=95 → direct=+90, wrap=-10 → returns -10.
 * Used by boids rules and brightness so boids near opposite edges are
 * treated as close, matching the wrapping physics.
 */
static float toroidal_delta(float a, float b, float extent) {
  const float HALF_EXTENT = extent * 0.5f;   /* wrap-decision boundary */

  float straight_delta = b - a;
  /* if the straight path is more than half the world wide, the WRAP
   * path is shorter — switch by ±extent (sign chosen to flip direction). */
  if (straight_delta >  HALF_EXTENT)  return straight_delta - extent;
  if (straight_delta < -HALF_EXTENT)  return straight_delta + extent;
  return straight_delta;
}

/* ===================================================================== */
/* §6  flock                                                              */
/* ===================================================================== */

/*
 * FlockMode — which steering algorithm is active right now.
 *
 * Intent
 *   ONE small enum is the entire user-facing variety of this demo.
 *   Keys 1..5 write a value here; flock_tick switches on it to pick
 *   which steering function runs.  Five values, four distinct papers'
 *   worth of theory — each mode is a different research community's
 *   answer to the same question: "how do agents that only sense their
 *   neighbours produce coordinated group motion?"
 *
 * Why an enum (not function pointers per Flock)
 *   • A scalar fits in HUD text without a separate name field.
 *   • App-level reset to default is trivial (memset Scene to 0 →
 *     mode = MODE_BOIDS automatically).
 *   • Mode-switching is a USER action that affects EVERY flock the
 *     same tick — per-flock function pointers would make the user
 *     change three things to switch one mode.
 *   • The switch in flock_tick is a TEACHING surface: a reader sees
 *     all five algorithms in one place, side-by-side.  Function
 *     pointers would scatter them.
 *
 * Members
 *   MODE_BOIDS    = 0   Reynolds three-rule flocking [1]:
 *                       force = W_SEP·separate + W_ALI·align
 *                             + W_COH·cohere   + W_LEAD·leader_pull
 *                       The textbook flocking algorithm.  Used by
 *                       boids_steer().
 *   MODE_CHASE    = 1   Direct homing [2]: followers ignore peers and
 *                       seek the leader, with separation just to
 *                       prevent stacking.  Produces "comet tail"
 *                       formations.  Used by chase_steer().
 *   MODE_VICSEK   = 2   Vicsek model [3]: align to the average heading
 *                       of all neighbours within PERCEPTION_RADIUS,
 *                       then add a random noise angle.  The single
 *                       noise scalar drives a phase transition between
 *                       polarised flow (low noise) and disordered
 *                       motion (high noise) — adjustable LIVE with
 *                       n / m keys.  Used by vicsek_steer().
 *   MODE_ORBIT    = 3   Slot-based formation: each follower is assigned
 *                       a slot on a ring of ORBIT_RADIUS around its
 *                       leader.  The ring rotates at ORBIT_SPEED, so
 *                       each slot is a MOVING target — followers chase
 *                       it.  Closest match in literature is "leader-
 *                       referenced formation" in [8] Millington & Funge
 *                       Ch.3.  Used by orbit_steer().
 *   MODE_PREDATOR = 4   Hunt/flee inspired by escape-panic models [6]:
 *                       flock 0 (predator) leader locks on to nearest
 *                       prey boid within PREDATOR_CHASE_RADIUS; prey
 *                       flocks (1, 2) use boids steering PLUS a strong
 *                       flee force from any predator boid within
 *                       PREY_FLEE_RADIUS.  Used by predator_leader_tick
 *                       + prey_boids_steer.
 *   MODE_COUNT          sentinel = 5; sized for k_mode_names[] below
 *                       and clamping in app_handle_key.
 *
 * Invariants
 *   0 ≤ value < MODE_COUNT.  Set by keys 1..5 in app_handle_key; reset
 *   to MODE_BOIDS by scene_init.
 *
 * References
 *   [1] Reynolds 1987 — MODE_BOIDS
 *   [2] Reynolds 1999 — MODE_CHASE (seek behaviour) + leader-pull
 *   [3] Vicsek et al. 1995 — MODE_VICSEK (the phase-transition model)
 *   [6] Helbing, Farkas & Vicsek 2000 — inspiration for MODE_PREDATOR
 *   [8] Millington & Funge — slot formations (MODE_ORBIT analogue)
 */
typedef enum {
  MODE_BOIDS = 0,    /* Reynolds 1987 — separation + alignment + cohesion */
  MODE_CHASE = 1,    /* leader homing with separation                     */
  MODE_VICSEK = 2,   /* Vicsek 1995 — average-heading + noise             */
  MODE_ORBIT = 3,    /* followers circle leader on a rotating ring        */
  MODE_PREDATOR = 4, /* flock 0 hunts; flocks 1-2 flee                    */
  MODE_COUNT
} FlockMode;

static const char *k_mode_names[MODE_COUNT] = {
    "BOIDS    sep+align+cohesion", "CHASE    leader homing",
    "VICSEK   align+noise",        "ORBIT    spinning ring",
    "PREDATOR hunt/flee",
};

/*
 * Flock — one leader plus its pool of followers, with state that lives
 * BETWEEN ticks of the steering algorithms.
 *
 * Intent
 *   The unit of swarm in this file.  Three Flocks share the screen and
 *   each runs the SAME active FlockMode but with its OWN leader,
 *   wander_angle, and slot-rotation phase.  Bundling the leader, the
 *   follower pool, and the per-leader / per-mode scratch into one
 *   struct lets every steering primitive read everything it needs from
 *   a single `Flock *f` parameter — no global lookup, no scattered
 *   "find the right leader for this flock" plumbing.
 *
 * Why ONE struct (not separate Leader + Followers + ModeState)
 *   • Spatial relationship: the leader is the WHERE for every steering
 *     mode (chase target, orbit centre, predator hunter).  Co-locating
 *     it with followers[] keeps cache-friendly access during the O(N²)
 *     neighbour scan.
 *   • Per-flock identity: each mode-specific scalar (wander_angle,
 *     orbit_phase) belongs to exactly one flock and advances with it.
 *     A global parallel-array layout would force every read to carry
 *     a flock index.
 *   • Reset is trivial: scene_init memsets Scene to zero, then
 *     flock_init re-spawns everything from a known starting position
 *     — no cleanup of "the OTHER flock's state by accident".
 *
 * Why fixed-size followers[FOLLOWERS_MAX]
 *   Per CLAUDE.md "Memory Allocation": no dynamic allocation after
 *   init.  The pool is pre-spawned to FOLLOWERS_MAX at flock_init, and
 *   `n` controls how many slots are ACTIVE.  '+' / '-' just slides `n`
 *   between FOLLOWERS_MIN and FOLLOWERS_MAX — no malloc/free, no
 *   re-randomise on every keypress.  See [10] Wikipedia "Boids" for
 *   pool-of-N-boids being the standard reference layout.
 *
 * Why a separate `leader` field (not followers[0])
 *   The leader runs DIFFERENT physics from followers:
 *     - constant LEADER_SPEED (no force integration)
 *     - smooth random walk on a HEADING angle, not on velocity
 *     - bypasses boid_clamp_speed (its speed is constructed, not
 *       derived from forces)
 *   Putting it in slot 0 of followers[] would require every steering
 *   loop to special-case index 0.  A separate field makes the
 *   different role explicit at the type level.
 *
 * Members
 *   leader              Boid representing the flock's leader, drawn with
 *                       leader_color_pair() and the flock's own glyph.
 *                       In all modes EXCEPT MODE_PREDATOR, leader_tick
 *                       advances it via wander_angle random walk.  In
 *                       MODE_PREDATOR, predator_leader_tick (flock 0)
 *                       locks wander_angle onto nearest prey bearing.
 *
 *   followers[]         pre-spawned pool of Boid; only the first `n`
 *                       slots are simulated and drawn.  Each follower
 *                       runs the active FlockMode's steering function.
 *                       Followers do NOT have a personal angle — their
 *                       heading is implicit in their velocity vector.
 *
 *   n                   active follower count, in [FOLLOWERS_MIN,
 *                       FOLLOWERS_MAX].  Live-adjustable with +/- keys
 *                       (app_handle_key clamps the range).  Every
 *                       tick/draw loop bounds itself with `i < f->n`.
 *
 *   wander_angle        leader's current heading in RADIANS (not vx,vy).
 *                       Stored as an angle because the leader's motion
 *                       is CONSTRUCTED from a random-walk-on-angle, not
 *                       derived from accumulated forces:
 *                         wander_angle += rand() · WANDER_JITTER
 *                         vx = LEADER_SPEED · cos(wander_angle)
 *                         vy = LEADER_SPEED · sin(wander_angle)
 *                       This makes the leader path SMOOTHLY curved
 *                       instead of jaggedly bouncing under summed
 *                       forces.  In MODE_PREDATOR, predator_leader_tick
 *                       snaps wander_angle to atan2(prey_dy, prey_dx)
 *                       when prey is in sight — same integration
 *                       machinery, different angle source.
 *
 *   orbit_phase         MODE_ORBIT only: current rotation angle of the
 *                       ring formation, in radians.  Advanced by
 *                       ORBIT_SPEED·dt per tick (independent of any
 *                       boid position).  Each follower's slot angle is
 *                       orbit_phase + 2π·i/n, so the ring acts like a
 *                       rigid rotating reference frame and every slot
 *                       drags its follower around the leader.  Reset
 *                       to 0 by scene_init's memset; advancement
 *                       happens only when MODE_ORBIT is active so
 *                       switching modes doesn't disturb other modes.
 *
 *   color_phase         hue offset associated with this flock, set
 *                       ONCE at flock_init to (float)flock_idx.  Kept
 *                       on the struct for a future hue-cycle palette
 *                       feature; currently the colour PAIR is decided
 *                       in §3 directly from flock_idx, and this field
 *                       is only used as a flock-identity marker for
 *                       potential future use.
 *
 * Invariants
 *   FOLLOWERS_MIN ≤ n ≤ FOLLOWERS_MAX (clamped in app_handle_key).
 *   leader.cruise_speed == LEADER_SPEED (set at flock_init).
 *   wander_angle is unbounded (radians; atan2/cos/sin handle the wrap).
 *   orbit_phase grows without bound while MODE_ORBIT is selected; cos /
 *   sin make this safe regardless of magnitude.
 *
 * References
 *   [1] Reynolds 1987 — flocking on a fixed pool of N boids; this
 *       struct's pool layout matches that reference implementation.
 *   [2] Reynolds 1999 §"Leader Following" — the leader-pull steering
 *       primitive uses this struct's `leader` as its target.
 *   [10] Wikipedia "Boids" — same fixed-pool, indexed-follower data
 *        layout used by every educational implementation.
 */
typedef struct {
  /* ── boid pool ─ leader + active followers[0..n-1] ───────────── */
  Boid leader;                   /* flock leader; drawn with its own pair  */
  Boid followers[FOLLOWERS_MAX]; /* pre-spawned pool; only first `n` live  */
  int  n;                        /* active follower count                  */

  /* ── leader steering state ───────────────────────────────────── */
  float wander_angle;            /* leader heading in radians (random walk)*/

  /* ── mode-specific extras ────────────────────────────────────── */
  float orbit_phase;             /* MODE_ORBIT ring rotation angle, radians*/

  /* ── render identity (set once at flock_init) ────────────────── */
  float color_phase;             /* hue offset; reserved for future palette*/
} Flock;

/* ── leader ──────────────────────────────────────────────────────────── */

/*
 * snap_angle_to_bearing — set angle to atan2(offset) toward (tx, ty)
 * via shortest toroidal path.  Used when the predator has acquired a
 * prey: its heading "locks on" to the prey's current bearing.
 */
static void snap_angle_to_bearing(const Boid *from,
                                  float tx, float ty, World world,
                                  float *angle_out) {
  /* toroidal offset from → target */
  float offset_x = toroidal_delta(from->px, tx, world.width);
  float offset_y = toroidal_delta(from->py, ty, world.height);

  /* atan2 gives the bearing in [−π, π]; the leader's velocity will be
   * reconstructed from this angle in integrate_leader_at_constant_speed */
  *angle_out = atan2f(offset_y, offset_x);
}

/*
 * integrate_leader_at_constant_speed — leader-style motion:
 *   vel = LEADER_SPEED · (cos θ, sin θ)
 *   pos += vel · dt
 *   wrap toroidally
 *
 * Shared by leader_tick and predator_leader_tick — both construct
 * velocity FROM an angle rather than integrating forces.
 */
static void integrate_leader_at_constant_speed(Boid *leader, float angle,
                                                float dt, World world) {
  /* (1) heading angle → unit-vector heading direction */
  float heading_dir_x = cosf(angle);
  float heading_dir_y = sinf(angle);

  /* (2) velocity = heading_direction · constant speed (no forces, no
   *     acceleration — leader physics is "wander on an angle") */
  leader->vx = heading_dir_x * LEADER_SPEED;
  leader->vy = heading_dir_y * LEADER_SPEED;

  /* (3) Euler integrate, then wrap to torus */
  leader->px += leader->vx * dt;
  leader->py += leader->vy * dt;
  boid_wrap(leader, world);
}

/*
 * leader_tick — advance the leader one physics step.
 *
 * The leader uses a smooth random walk on its heading angle (wander_angle).
 * Each tick the heading shifts by a small random amount in
 * [-WANDER_JITTER, +WANDER_JITTER] radians — just enough to curve gently
 * without making sharp unpredictable turns that followers cannot track.
 * Speed is constant; we derive velocity from the angle rather than
 * integrating forces, so the leader never slows or stalls.
 */
static void leader_tick(Flock *f, float dt, World world) {
  /* (1) random walk on the heading angle */
  f->wander_angle += rand_uniform_signed() * WANDER_JITTER;

  /* (2) construct velocity from angle, integrate, wrap */
  integrate_leader_at_constant_speed(&f->leader, f->wander_angle, dt, world);
}

/* ── steering primitives ─────────────────────────────────────────────── */
/*
 * Shared force-building helpers used by every steering mode.  All three
 * take an in/out (*steer_x, *steer_y) accumulator and ADD their force
 * contribution — the caller composes the final steering vector from
 * several of these calls, then passes the result to clamp2d.
 *
 *   clamp2d              — cap |steer| to a max magnitude (turn-rate cap)
 *   add_separation_force — close-range push away from neighbours
 *   seek_force_toward    — Reynolds "seek" toward (target_px, target_py)
 */

/*
 * clamp2d — clamp a 2D vector (fx, fy) so its magnitude does not exceed
 * max_mag. Preserves direction; only scales down if over the limit.
 * Avoids a sqrt when the vector is already within bounds.
 */
static void clamp2d(float *fx, float *fy, float max_mag) {
  float mag_sq = (*fx) * (*fx) + (*fy) * (*fy);
  if (mag_sq > max_mag * max_mag) {
    float mag = sqrtf(mag_sq);
    *fx = (*fx / mag) * max_mag;
    *fy = (*fy / mag) * max_mag;
  }
}

/*
 * add_separation_force — accumulate "personal-space" repulsion from any
 * neighbour within SEPARATION_RADIUS.
 *
 *   for each neighbour j (j != self_idx):
 *     d  = toroidal_distance(self, others[j])
 *     if d < SEPARATION_RADIUS:
 *       closeness = (SEPARATION_RADIUS − d) / SEPARATION_RADIUS
 *       steer    -= unit(toward_neighbour) · closeness · MAX_STEER
 *
 * Used by both chase_steer and orbit_steer — without this helper the
 * loop body was duplicated verbatim in two places.  Boids steering keeps
 * its own integrated separation accumulator because it shares the
 * neighbour-scan loop with alignment + cohesion.
 */
static void add_separation_force(const Boid *self, const Boid *others,
                                 int n, int self_idx, World world,
                                 float *steer_x, float *steer_y) {
  const float DIST_EPSILON = 0.001f;       /* zero-distance guard */

  for (int j = 0; j < n; j++) {
    if (j == self_idx) continue;

    /* toroidal offset self → neighbour */
    float offset_x = toroidal_delta(self->px, others[j].px, world.width);
    float offset_y = toroidal_delta(self->py, others[j].py, world.height);
    float distance = hypotf(offset_x, offset_y);

    bool inside_personal_space = distance < SEPARATION_RADIUS;
    bool well_separated        = distance > DIST_EPSILON;
    if (!inside_personal_space || !well_separated) continue;

    /* personal_space_intrusion ∈ (0,1]; closer neighbour ⇒ stronger push */
    float personal_space_intrusion = (SEPARATION_RADIUS - distance) / SEPARATION_RADIUS;

    /* unit vector pointing AWAY from neighbour (subtract = flip self→nb) */
    float push_away_x = -offset_x / distance;
    float push_away_y = -offset_y / distance;

    /* contribute weighted push (scaled by MAX_STEER) */
    *steer_x += push_away_x * personal_space_intrusion * MAX_STEER;
    *steer_y += push_away_y * personal_space_intrusion * MAX_STEER;
  }
}

/*
 * seek_force_toward — seek-with-arrival force in this file's style.
 *
 *   offset  = toroidal_delta(self → target)
 *   desired = unit(offset) · cruise_speed
 *   steer  += desired − self.vel
 *
 * Caller decides whether to apply a weight + MAX_STEER clamp later.
 * Used by orbit_steer and chase_steer (after they pick their target).
 */
static void seek_force_toward(const Boid *b, float target_px, float target_py,
                               World world,
                               float *steer_x, float *steer_y) {
  const float OFFSET_EPSILON = 0.001f;

  /* toroidal offset self → target */
  float offset_x          = toroidal_delta(b->px, target_px, world.width);
  float offset_y          = toroidal_delta(b->py, target_py, world.height);
  float distance_to_target = hypotf(offset_x, offset_y);
  if (distance_to_target <= OFFSET_EPSILON) return;

  /* unit vector pointing FROM self TOWARD target */
  float toward_target_x = offset_x / distance_to_target;
  float toward_target_y = offset_y / distance_to_target;

  /* desired velocity: head to target at MY cruise speed */
  float desired_vx = toward_target_x * b->cruise_speed;
  float desired_vy = toward_target_y * b->cruise_speed;

  /* steering = desired − current_velocity  [Reynolds 1999 §"Seek"] */
  *steer_x += desired_vx - b->vx;
  *steer_y += desired_vy - b->vy;
}

/* ── boids steering (MODE_BOIDS) ─────────────────────────────────────── */

/*
 * NeighbourSums — per-rule accumulators built in ONE pass over the
 * neighbour list.
 *
 * Intent
 *   All three Reynolds rules (separation, alignment, cohesion) need to
 *   visit every neighbour within PERCEPTION_RADIUS.  Doing three
 *   separate O(N) passes would visit the same boids three times.  This
 *   struct holds the partial sums from a SINGLE pass that the three
 *   force-accumulator helpers (accumulate_separation_force /
 *   _alignment_force / _cohesion_force) then read.
 *
 * Members
 *   sep_x, sep_y          unit-push vectors away from boids inside
 *                         SEPARATION_RADIUS, summed; each term scaled
 *                         by `personal_space_intrusion` ∈ (0,1] so close
 *                         neighbours dominate and far ones barely
 *                         contribute (linear falloff).
 *   ali_vx, ali_vy        plain sum of neighbour velocity vectors
 *                         (within PERCEPTION_RADIUS, not SEP).  Divided
 *                         by neighbour_count to get the mean heading.
 *   coh_dx, coh_dy        plain sum of TOROIDAL OFFSETS to neighbours
 *                         (NOT absolute positions — see the cohesion
 *                         comment for why averaging absolutes is wrong
 *                         on a torus).  Divided by neighbour_count to
 *                         get the relative centre-of-mass direction.
 *   neighbour_count       boids inside PERCEPTION_RADIUS (drives align
 *                         + cohesion averages).
 *   close_count           boids inside SEPARATION_RADIUS (≤ neighbour_
 *                         count); ≥ 1 means a separation force exists.
 *
 * Reference
 *   [1] Reynolds 1987 — the three-rule structure; this struct is just
 *       the bookkeeping for "compute all three in one scan".
 */
typedef struct {
  float sep_x, sep_y;
  float ali_vx, ali_vy;
  float coh_dx, coh_dy;
  int   neighbour_count;
  int   close_count;
} NeighbourSums;

/*
 * scan_perception_neighbours — single O(N) pass that fills every
 * NeighbourSums field for follower[idx].
 *
 *   for each j != idx:
 *     d  = toroidal_distance(self, followers[j])
 *     if d >= PERCEPTION_RADIUS or d < epsilon: skip
 *     ali_v  += neighbour.vel                          (alignment)
 *     coh_d  += toroidal_offset(self → neighbour)      (cohesion)
 *     neighbour_count++
 *     if d < SEPARATION_RADIUS:
 *       sep -= unit(offset) · (SEP − d) / SEP          (separation)
 *       close_count++
 */
static NeighbourSums scan_perception_neighbours(const Flock *f, int idx,
                                                 World world) {
  const float DIST_EPSILON = 0.001f;     /* zero-distance guard (co-located boids) */
  const Boid *b = &f->followers[idx];
  NeighbourSums s = {0};

  for (int j = 0; j < f->n; j++) {
    if (j == idx) continue;
    const Boid *nb = &f->followers[j];

    /* (a) toroidal displacement: shortest signed offset self → neighbour */
    float offset_x   = toroidal_delta(b->px, nb->px, world.width);
    float offset_y   = toroidal_delta(b->py, nb->py, world.height);
    float distance   = hypotf(offset_x, offset_y);

    /* (b) skip if outside sight range or right on top of self */
    bool inside_perception = distance < PERCEPTION_RADIUS;
    bool well_separated    = distance >= DIST_EPSILON;
    if (!inside_perception || !well_separated) continue;

    /* (c) alignment + cohesion sums — every visible neighbour contributes.
     *     Cohesion sums OFFSETS (not absolute positions): on a torus,
     *     averaging absolute positions gives the wrong centre.  See
     *     accumulate_cohesion_force docblock for the full explanation. */
    s.ali_vx += nb->vx;
    s.ali_vy += nb->vy;
    s.coh_dx += offset_x;
    s.coh_dy += offset_y;
    s.neighbour_count++;

    /* (d) separation: closer neighbours push proportionally harder.
     *     personal_space_intrusion ∈ (0, 1]:  1 at d=0, 0 at d=SEP_RADIUS.
     *     push_unit_(x,y) is a unit vector pointing AWAY from neighbour
     *     (offset is self→neighbour, so we subtract to flip direction). */
    if (distance < SEPARATION_RADIUS) {
      float personal_space_intrusion = (SEPARATION_RADIUS - distance) / SEPARATION_RADIUS;
      float push_unit_x              = offset_x / distance;
      float push_unit_y              = offset_y / distance;
      s.sep_x -= push_unit_x * personal_space_intrusion;
      s.sep_y -= push_unit_y * personal_space_intrusion;
      s.close_count++;
    }
  }
  return s;
}

/*
 * accumulate_separation_force — convert sep_x/sep_y accumulator to a
 * weighted unit-vector steering force.
 *
 *   if close_count > 0 and |sep| > epsilon:
 *     steer += unit(sep) · W_SEPARATION · MAX_STEER
 */
static void accumulate_separation_force(const NeighbourSums *s,
                                        float *steer_x, float *steer_y) {
  const float MAG_EPSILON = 0.001f;
  if (s->close_count == 0) return;

  /* normalise the (sum of weighted away-vectors) → unit push direction */
  float push_magnitude = hypotf(s->sep_x, s->sep_y);
  if (push_magnitude <= MAG_EPSILON) return;
  float push_dir_x = s->sep_x / push_magnitude;
  float push_dir_y = s->sep_y / push_magnitude;

  /* contribute W_SEPARATION · MAX_STEER in that direction */
  *steer_x += push_dir_x * W_SEPARATION * MAX_STEER;
  *steer_y += push_dir_y * W_SEPARATION * MAX_STEER;
}

/*
 * accumulate_alignment_force — match average neighbour heading at this
 * boid's OWN cruise speed (Reynolds rule 2).
 *
 *   avg_v   = mean(neighbour velocities)
 *   desired = unit(avg_v) · b->cruise_speed
 *   steer  += (desired − b->vel) · W_ALIGNMENT
 *
 * Using cruise_speed (not BOID_SPEED) is what gives the flock its
 * ±15 % depth — fast boids push forward, slow ones drift back.
 */
static void accumulate_alignment_force(const NeighbourSums *s, const Boid *b,
                                       float *steer_x, float *steer_y) {
  const float MAG_EPSILON = 0.001f;
  if (s->neighbour_count == 0) return;

  /* mean velocity vector of all visible neighbours */
  float mean_heading_x = s->ali_vx / (float)s->neighbour_count;
  float mean_heading_y = s->ali_vy / (float)s->neighbour_count;
  float mean_heading_magnitude = hypotf(mean_heading_x, mean_heading_y);
  if (mean_heading_magnitude <= MAG_EPSILON) return;

  /* unit vector pointing in the mean-heading direction */
  float aligned_dir_x = mean_heading_x / mean_heading_magnitude;
  float aligned_dir_y = mean_heading_y / mean_heading_magnitude;

  /* desired = aligned_direction scaled to MY own cruise speed (so fast boids
   * push forward, slow ones drift back — see Boid struct doc) */
  float desired_vx = aligned_dir_x * b->cruise_speed;
  float desired_vy = aligned_dir_y * b->cruise_speed;

  /* steering = (desired − current_velocity) · W_ALIGNMENT  [Reynolds 1999] */
  *steer_x += (desired_vx - b->vx) * W_ALIGNMENT;
  *steer_y += (desired_vy - b->vy) * W_ALIGNMENT;
}

/*
 * accumulate_cohesion_force — steer toward the toroidal centre-of-mass
 * of visible neighbours (Reynolds rule 3).
 *
 *   avg_offset = mean(toroidal offsets to neighbours)
 *   desired    = unit(avg_offset) · b->cruise_speed
 *   steer     += (desired − b->vel) · W_COHESION
 *
 * Why average OFFSETS, not absolute positions
 *   Two boids at px=5 and px=95 on a width-100 torus are 10 px apart
 *   across the wrap.  Averaging their absolute positions gives px=50
 *   (the OTHER side of the world).  Averaging their offsets relative
 *   to `b` gives the correct direction to steer.
 */
static void accumulate_cohesion_force(const NeighbourSums *s, const Boid *b,
                                      float *steer_x, float *steer_y) {
  const float MAG_EPSILON = 0.001f;
  if (s->neighbour_count == 0) return;

  /* mean toroidal OFFSET to neighbours (NOT mean absolute position — see
   * the "Why average OFFSETS" note in this function's docblock above) */
  float mean_offset_x = s->coh_dx / (float)s->neighbour_count;
  float mean_offset_y = s->coh_dy / (float)s->neighbour_count;
  float distance_to_centre_of_mass = hypotf(mean_offset_x, mean_offset_y);
  if (distance_to_centre_of_mass <= MAG_EPSILON) return;

  /* unit vector pointing toward the group centre-of-mass */
  float toward_centre_x = mean_offset_x / distance_to_centre_of_mass;
  float toward_centre_y = mean_offset_y / distance_to_centre_of_mass;

  /* desired velocity: head to centre at MY cruise speed */
  float desired_vx = toward_centre_x * b->cruise_speed;
  float desired_vy = toward_centre_y * b->cruise_speed;

  /* steering = (desired − current_velocity) · W_COHESION */
  *steer_x += (desired_vx - b->vx) * W_COHESION;
  *steer_y += (desired_vy - b->vy) * W_COHESION;
}

/*
 * accumulate_leader_pull — gentle toroidal-shortest-path attraction
 * toward the flock's own leader.  Prevents the flock from drifting away
 * indefinitely; weight is small so it never overrides the three rules.
 *
 *   steer += unit(offset_to_leader) · W_LEADER_PULL · MAX_STEER
 */
static void accumulate_leader_pull(const Boid *b, const Boid *leader,
                                   World world,
                                   float *steer_x, float *steer_y) {
  const float MAG_EPSILON = 0.001f;

  /* toroidal offset self → leader */
  float offset_x = toroidal_delta(b->px, leader->px, world.width);
  float offset_y = toroidal_delta(b->py, leader->py, world.height);
  float distance_to_leader = hypotf(offset_x, offset_y);
  if (distance_to_leader <= MAG_EPSILON) return;

  /* unit vector pointing FROM self TOWARD leader */
  float toward_leader_x = offset_x / distance_to_leader;
  float toward_leader_y = offset_y / distance_to_leader;

  /* W_LEADER_PULL · MAX_STEER pull in that direction */
  *steer_x += toward_leader_x * W_LEADER_PULL * MAX_STEER;
  *steer_y += toward_leader_y * W_LEADER_PULL * MAX_STEER;
}

/*
 * boids_steer — compute the new velocity for followers[idx] under the
 * canonical Reynolds 1987 three-rule model + a gentle leader pull.
 *
 * Algorithm
 *   (1) one O(N) neighbour scan into NeighbourSums
 *   (2) build steering force = Σ weighted Reynolds rules + leader pull
 *   (3) cap steering magnitude to MAX_STEER (smooth turns, no snap)
 *   (4) new_vel = current_vel + capped_steer
 *
 * Read-only on f->followers[] — the caller (flock_tick) pre-computes
 * every new velocity before applying any, so all boids react to the
 * SAME position snapshot ("simultaneous update").
 *
 * References
 *   [1] Reynolds 1987 — the three rules.
 *   [2] Reynolds 1999 §"Leader Following" — the leader-pull term.
 */
static void boids_steer(const Flock *f, int idx, World world,
                        float *out_vx, float *out_vy) {
  const Boid *b = &f->followers[idx];

  /* (1) one neighbour-scan pass fills every rule's accumulator */
  NeighbourSums sums = scan_perception_neighbours(f, idx, world);

  /* (2) sum the four weighted forces into one steering vector */
  float steer_x = 0.0f, steer_y = 0.0f;
  accumulate_separation_force(&sums,           &steer_x, &steer_y);
  accumulate_alignment_force (&sums, b,        &steer_x, &steer_y);
  accumulate_cohesion_force  (&sums, b,        &steer_x, &steer_y);
  accumulate_leader_pull     (b, &f->leader, world, &steer_x, &steer_y);

  /* (3) cap turn rate, (4) apply to current velocity */
  clamp2d(&steer_x, &steer_y, MAX_STEER);
  *out_vx = b->vx + steer_x;
  *out_vy = b->vy + steer_y;
}

/* ── chase steering (MODE_CHASE) ─────────────────────────────────────── */

/*
 * chase_steer — simple direct homing on the leader.
 * Each follower ignores its peers and steers straight toward its leader.
 * Only separation at close range is applied, so boids don't stack on top
 * of each other while chasing. Produces "comet tail" formation shapes.
 */
static void chase_steer(const Flock *f, int idx, World world,
                        float *out_vx, float *out_vy) {
  const Boid *b = &f->followers[idx];
  float steer_x = 0.0f, steer_y = 0.0f;

  /* (1) seek leader via shortest toroidal path */
  seek_force_toward(b, f->leader.px, f->leader.py, world, &steer_x, &steer_y);

  /* (2) separation: prevent boids from piling up while chasing */
  add_separation_force(b, f->followers, f->n, idx, world, &steer_x, &steer_y);

  /* (3) cap turn rate, apply to current velocity */
  clamp2d(&steer_x, &steer_y, MAX_STEER);
  *out_vx = b->vx + steer_x;
  *out_vy = b->vy + steer_y;
}

/* ── Vicsek steering (MODE_VICSEK) ───────────────────────────────────── */

/*
 * vicsek_steer — Vicsek (1995) model: each boid aligns to the average
 * heading of all neighbors within PERCEPTION_RADIUS, then adds a random
 * noise angle.
 *
 * This one-parameter model drives a sharp phase transition:
 *   low noise  → entire flock streams in one direction (ordered / polarised)
 *   high noise → every boid moves independently (disordered / chaotic)
 *
 * Unlike Reynolds boids there is no explicit cohesion or separation force;
 * the spatial coherence comes purely from the alignment interaction.
 *
 * The leader is included in the neighbor set so the flock stays oriented
 * around its leader's wandering direction at low noise.
 *
 * noise_scale: max random perturbation in radians (VICSEK_NOISE_DEFAULT).
 *              Adjustable live with 'n' / 'm' keys.
 */
/*
 * sum_velocities_in_perception — Vicsek's "look at everyone within
 * PERCEPTION_RADIUS and add up their velocity vectors".  Includes self
 * (Vicsek model averages self into the heading too) and the leader
 * (anchors the flock when noise is low).
 *
 *   sum_v = self.vel + leader.vel? + Σ neighbour.vel (within R)
 *   count = 1        + (0 or 1)    + #neighbours_within_R
 *
 * Caller divides sum_v by count to get the mean heading vector and
 * then converts to an angle via atan2.
 */
static void sum_velocities_in_perception(const Flock *f, int idx, World world,
                                          float *out_sum_vx,
                                          float *out_sum_vy,
                                          int   *out_count) {
  const Boid *b = &f->followers[idx];
  float sum_vx = b->vx;
  float sum_vy = b->vy;
  int   count  = 1;

  /* Leader: anchors the flock direction at low noise */
  {
    float dx = toroidal_delta(b->px, f->leader.px, world.width);
    float dy = toroidal_delta(b->py, f->leader.py, world.height);
    if (hypotf(dx, dy) < PERCEPTION_RADIUS) {
      sum_vx += f->leader.vx;
      sum_vy += f->leader.vy;
      count++;
    }
  }

  /* Followers within perception radius */
  for (int j = 0; j < f->n; j++) {
    if (j == idx) continue;
    float dx = toroidal_delta(b->px, f->followers[j].px, world.width);
    float dy = toroidal_delta(b->py, f->followers[j].py, world.height);
    if (hypotf(dx, dy) < PERCEPTION_RADIUS) {
      sum_vx += f->followers[j].vx;
      sum_vy += f->followers[j].vy;
      count++;
    }
  }
  *out_sum_vx = sum_vx;
  *out_sum_vy = sum_vy;
  *out_count  = count;
}

static void vicsek_steer(const Flock *f, int idx, World world,
                         float noise_scale, float *out_vx, float *out_vy) {
  const Boid *b = &f->followers[idx];

  /* (1) Σ velocity vectors of every visible neighbour (self+leader+flock) */
  float sum_vx, sum_vy;
  int   n_velocity_contributors;
  sum_velocities_in_perception(f, idx, world,
                                &sum_vx, &sum_vy, &n_velocity_contributors);

  /* (2a) mean velocity → mean heading angle.  atan2f reduces a 2-D
   *      direction to a scalar angle in [−π, π] — the order parameter
   *      of the Vicsek model. */
  float mean_vx          = sum_vx / (float)n_velocity_contributors;
  float mean_vy          = sum_vy / (float)n_velocity_contributors;
  float mean_heading_rad = atan2f(mean_vy, mean_vx);

  /* (2b) add noise η ∈ [−noise_scale, +noise_scale] (Vicsek's stochastic
   *      term — η drives the polarised ↔ disordered phase transition) */
  float noise_eta        = rand_uniform_signed() * noise_scale;
  float new_heading_rad  = mean_heading_rad + noise_eta;

  /* (3) reconstruct velocity from new heading at MY OWN cruise_speed
   *     (Vicsek dictates direction; magnitude stays personal) */
  *out_vx = cosf(new_heading_rad) * b->cruise_speed;
  *out_vy = sinf(new_heading_rad) * b->cruise_speed;
}

/* ── orbit steering (MODE_ORBIT) ─────────────────────────────────────── */

/*
 * orbit_steer — assign each follower a slot on a ring of radius ORBIT_RADIUS
 * centred on the leader. The ring rotates at ORBIT_SPEED rad/s (stored in
 * f->orbit_phase and advanced each tick). Each follower chases its moving slot.
 *
 * Slot assignment: slot angle = orbit_phase + 2π × idx / n
 * Evenly-spaced slots produce a spinning halo; as the phase advances every
 * boid moves along the ring rather than parking in one place.
 *
 * Separation is still applied so boids don't stack when they close in
 * on the same slot or when the flock is small.
 */
/*
 * orbit_target_for_slot — geometry of the rotating ring formation.
 *
 *   slot_angle  = orbit_phase + 2π · idx / n
 *   target_px   = leader.px + ORBIT_RADIUS · cos(slot_angle)
 *   target_py   = leader.py + ORBIT_RADIUS · sin(slot_angle)
 *
 * Evenly-spaced slots produce a spinning halo; as orbit_phase advances
 * every follower's target moves along the ring.
 */
static void orbit_target_for_slot(const Boid *leader, float orbit_phase,
                                  int idx, int n,
                                  float *target_px, float *target_py) {
  const float FULL_TURN_RADIANS = 2.0f * (float)M_PI;

  /* even angular spacing of slots around the ring (FULL_TURN / n apart) */
  float slot_offset_angle = FULL_TURN_RADIANS * (float)idx / (float)n;
  float slot_angle        = orbit_phase + slot_offset_angle;

  /* polar (ORBIT_RADIUS, slot_angle) around leader → Cartesian world pos */
  *target_px = leader->px + ORBIT_RADIUS * cosf(slot_angle);
  *target_py = leader->py + ORBIT_RADIUS * sinf(slot_angle);
}

static void orbit_steer(const Flock *f, int idx, World world,
                        float *out_vx, float *out_vy) {
  const Boid *b = &f->followers[idx];

  /* (1) where on the ring this follower belongs RIGHT NOW */
  float target_px, target_py;
  orbit_target_for_slot(&f->leader, f->orbit_phase, idx, f->n,
                        &target_px, &target_py);

  /* (2) seek toward that target via shortest toroidal path */
  float steer_x = 0.0f, steer_y = 0.0f;
  seek_force_toward(b, target_px, target_py, world, &steer_x, &steer_y);

  /* (3) separation: prevent boids from overlapping en route to slots */
  add_separation_force(b, f->followers, f->n, idx, world, &steer_x, &steer_y);

  /* (4) cap turn rate, apply to current velocity */
  clamp2d(&steer_x, &steer_y, MAX_STEER);
  *out_vx = b->vx + steer_x;
  *out_vy = b->vy + steer_y;
}

/* ── flock tick ──────────────────────────────────────────────────────── */

/*
 * flock_tick — advance one flock by dt seconds.
 *
 * Two-stage simultaneous update:
 *   Stage 1 — compute new velocity for every follower; reads OLD
 *             positions only.  Dispatched per-follower by
 *             compute_steer_for_mode based on the active FlockMode.
 *   Stage 2 — write new velocities, clamp speed, integrate position,
 *             toroidally wrap.  Implemented by apply_followers_step.
 *
 * Without two stages, boids earlier in the array would react to already-
 * moved boids later in the array — the flock would drift in array order.
 */
/*
 * apply_followers_step — stage-2 write of the two-stage update.
 *
 *   for each follower i:
 *     vel = new_vx[i], new_vy[i]
 *     clamp |vel| to [MIN_SPEED, MAX_SPEED]
 *     pos += vel · dt
 *     wrap pos toroidally to the world box
 *
 * Reused by flock_tick (single-mode update) and scene_tick_predator (the
 * predator-prey path runs the same write loop on three flocks back-to-
 * back).  Extracting this kept the predator path readable.
 */
static void apply_followers_step(Flock *f, const float *new_vx,
                                 const float *new_vy, float dt, World world) {
  for (int i = 0; i < f->n; i++) {
    Boid *b = &f->followers[i];
    b->vx = new_vx[i];
    b->vy = new_vy[i];
    boid_clamp_speed(b);
    b->px += b->vx * dt;
    b->py += b->vy * dt;
    boid_wrap(b, world);
  }
}

/*
 * compute_steer_for_mode — dispatch one follower's stage-1 steering
 * computation to the active mode's steering function.  Centralises the
 * mode switch so flock_tick's body reads as three named stages, and so
 * adding a sixth mode means touching exactly one switch.
 */
static void compute_steer_for_mode(const Flock *f, int i, FlockMode mode,
                                   float vicsek_noise, World world,
                                   float *out_vx, float *out_vy) {
  switch (mode) {
    case MODE_BOIDS:
      boids_steer (f, i, world,                out_vx, out_vy); break;
    case MODE_VICSEK:
      vicsek_steer(f, i, world, vicsek_noise,  out_vx, out_vy); break;
    case MODE_ORBIT:
      orbit_steer (f, i, world,                out_vx, out_vy); break;
    default: /* MODE_CHASE and any unhandled: simple homing */
      chase_steer (f, i, world,                out_vx, out_vy); break;
  }
}

static void flock_tick(Flock *f, FlockMode mode, float vicsek_noise, float dt,
                       World world) {
  /* (1) advance leader (wander random walk) */
  leader_tick(f, dt, world);

  /* (2) advance the rotating-ring reference frame, only when ORBIT active */
  if (mode == MODE_ORBIT)
    f->orbit_phase += ORBIT_SPEED * dt;

  /* (3) STAGE 1 — compute each follower's new velocity (read-only on f) */
  float new_vx[FOLLOWERS_MAX];
  float new_vy[FOLLOWERS_MAX];
  for (int i = 0; i < f->n; i++)
    compute_steer_for_mode(f, i, mode, vicsek_noise, world,
                            &new_vx[i], &new_vy[i]);

  /* (4) STAGE 2 — write velocities, clamp, integrate, wrap */
  apply_followers_step(f, new_vx, new_vy, dt, world);
}

/* ── flock init ──────────────────────────────────────────────────────── */

/*
 * SPAWN_SCATTER — radius (px) within which followers are initially spread
 * around the leader's starting position. Large enough that boids don't
 * all start on the same cell, small enough that they start as a group.
 */
#define SPAWN_SCATTER 70.0f

/*
 * flock_init — place a flock in a screen quadrant with scattered followers.
 *
 * Starting quadrants (as fractions of pixel-space dimensions):
 *   flock 0 — top-left  (0.25, 0.25)
 *   flock 1 — top-right (0.75, 0.25)
 *   flock 2 — bottom    (0.50, 0.75)
 *
 * Placing flocks in different quadrants means the first few seconds show
 * three separate groups that gradually merge and interact — more visually
 * interesting than starting with everything in the center.
 */
/*
 * starting_quadrant_origin — fixed-table lookup of each flock's starting
 * position as fractions of the world extent.
 *
 *   flock 0 — top-left   (0.25, 0.25)
 *   flock 1 — top-right  (0.75, 0.25)
 *   flock 2 — bottom-mid (0.50, 0.75)
 *
 * Spreading flocks across distinct quadrants means the first few seconds
 * show three separate groups that gradually merge and interact — more
 * visually interesting than everything starting at the centre.
 */
static void starting_quadrant_origin(int flock_idx, World world,
                                     float *out_ox, float *out_oy) {
  static const float QUADRANT_FRAC_X[FLOCKS] = {0.25f, 0.75f, 0.50f};
  static const float QUADRANT_FRAC_Y[FLOCKS] = {0.25f, 0.25f, 0.75f};
  *out_ox = world.width  * QUADRANT_FRAC_X[flock_idx];
  *out_oy = world.height * QUADRANT_FRAC_Y[flock_idx];
}

/*
 * pick_scattered_spawn_pos — uniform-random position within SPAWN_SCATTER
 * of (ox, oy), clamped inside the world box.
 *
 *   sx = ox + rand_signed · SPAWN_SCATTER       (uniform in a square)
 *   sy = oy + rand_signed · SPAWN_SCATTER
 *   clamp sx ∈ [0, world.width  − 1]
 *   clamp sy ∈ [0, world.height − 1]
 */
static void pick_scattered_spawn_pos(float ox, float oy, World world,
                                     float *out_sx, float *out_sy) {
  float sx = ox + rand_uniform_signed() * SPAWN_SCATTER;
  float sy = oy + rand_uniform_signed() * SPAWN_SCATTER;
  if (sx < 0)              sx = 0;
  if (sx >= world.width)   sx = world.width  - 1.0f;
  if (sy < 0)              sy = 0;
  if (sy >= world.height)  sy = world.height - 1.0f;
  *out_sx = sx;
  *out_sy = sy;
}

/*
 * pick_cruise_speed_variation — sample a follower's personal cruise-speed
 * factor in [0.85, 1.15] (a ±15 % spread around BOID_SPEED).
 *
 * The ±15 % spread is the entire source of the flock's organic depth:
 * fast boids push to the front, slow ones drift back.  See Boid struct
 * doc for the [5] Couzin et al. biological analogue.
 */
static float pick_cruise_speed_variation(void) {
  const float SPEED_VARIATION_MIN  = 0.85f;
  const float SPEED_VARIATION_SPAN = 0.30f;   /* span of [0.85, 1.15] */
  return SPEED_VARIATION_MIN + SPEED_VARIATION_SPAN * rand_uniform_01();
}

static void flock_init(Flock *f, int flock_idx, int n_followers, World world) {
  /* (1) bookkeeping: count, identity, random initial leader heading */
  f->n            = n_followers;
  f->color_phase  = (float)flock_idx;
  f->wander_angle = rand_uniform_angle_2pi();

  /* (2) place the leader at this flock's quadrant origin */
  float ox, oy;
  starting_quadrant_origin(flock_idx, world, &ox, &oy);
  boid_spawn_at(&f->leader, ox, oy, LEADER_SPEED);

  /* (3) spawn each follower scattered around the origin with ±15 %
   *     personal cruise-speed variation */
  for (int i = 0; i < f->n; i++) {
    float sx, sy;
    pick_scattered_spawn_pos(ox, oy, world, &sx, &sy);
    float variation = pick_cruise_speed_variation();
    boid_spawn_at(&f->followers[i], sx, sy, BOID_SPEED * variation);
  }
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * SimControls — user-facing playback knobs.
 *
 * Intent
 *   Three scalars the input handler writes and scene_tick + HUD read.
 *   Bundling them keeps app_handle_key's switch readable, the HUD's read
 *   paths short, and the Scene top level free for future state additions
 *   (time-scrubbing, debug overlays, alternative integrators) without
 *   churn.
 *
 * Why bundle
 *   The boundary "USER-controlled run parameters" is conceptually
 *   distinct from "what the simulation HAS" (flocks, world).  Bundling
 *   makes that role explicit at the type level, and every helper that
 *   only needs to read knobs takes `const SimControls *` rather than
 *   reaching into Scene.
 *
 * Why these three (and not e.g. dt_scale, single-step, debug overlays)
 *   These are the MINIMUM set that lets the demo show all five flocking
 *   algorithms interactively.  Other knobs (time scaling, frame
 *   stepping, debug visualisation) would belong here when added — the
 *   struct's role is "everything the user can twiddle in real time" and
 *   the type signals that exact contract.
 *
 * Members
 *   mode          one of the five FlockMode enum values — drives the
 *                 steering-function switch in flock_tick + scene_tick.
 *                 Also drives:
 *                   - HUD label (k_mode_names lookup)
 *                   - scene_tick_predator dispatch (when == MODE_PREDATOR)
 *                   - orbit_phase advancement (only when == MODE_ORBIT)
 *                 Set by keys 1..5 in app_handle_key; reset to
 *                 MODE_BOIDS by scene_init.  See [1][2][3][6][8] for
 *                 the papers behind each mode.
 *
 *   paused        true → scene_tick early-returns; physics frozen, HUD
 *                 still updates "PAUSED" indicator.  Toggled by SPACE.
 *                 Pause does NOT freeze the render-time interpolation,
 *                 so glyphs are stationary even mid-tick.
 *
 *   vicsek_noise  current Vicsek-noise amplitude in RADIANS.  In the
 *                 Vicsek model [3] each boid's heading is set to
 *                       new_angle = avg_neighbour_angle + η
 *                 where η is drawn uniformly from [−noise, +noise].
 *                 This single scalar is the order parameter of the
 *                 model — a continuous phase transition at η ≈ 0.5
 *                 separates ordered (low noise → all boids stream
 *                 together) from disordered (high noise → independent
 *                 motion).  Live-adjustable with n / m keys; clamp
 *                 range [VICSEK_NOISE_MIN, VICSEK_NOISE_MAX] in
 *                 app_handle_key lets the user traverse the transition
 *                 LIVE on screen.  Ignored by every other mode.
 *
 * Invariants
 *   0 ≤ mode < MODE_COUNT.
 *   VICSEK_NOISE_MIN ≤ vicsek_noise ≤ VICSEK_NOISE_MAX (clamped in
 *   app_handle_key on every n / m keypress).
 *
 * References
 *   [1][2][3][6][8] — see the FlockMode docblock above; SimControls.mode
 *       SELECTS which paper's algorithm runs each tick.
 *   [3] Vicsek et al. 1995 — the noise→phase-transition mapping that
 *       makes `vicsek_noise` worth exposing as a live-tweakable knob.
 *   [4] Toner & Tu 1995 — continuum theory predicting WHY the noise
 *       transition is sharp; useful background when tuning the
 *       VICSEK_NOISE_MIN/MAX limits.
 */
typedef struct {
  FlockMode mode;          /* active steering algorithm; keys 1..5      */
  bool      paused;        /* SPACE toggles; scene_tick early-returns   */
  float     vicsek_noise;  /* η in radians; n/m keys traverse [3]'s     */
                           /* phase transition                          */
} SimControls;

/*
 * Scene — owns ALL simulation state for one run.
 *
 * Layered ownership
 *
 *     Scene
 *       ├── flocks[FLOCKS]  : Flock[]      ← agent pool (3 flocks, each
 *       │                                      with leader + followers[])
 *       ├── sim             : SimControls  ← mode + paused + vicsek_noise
 *       │                                      (every user-controlled knob)
 *       └── world           : World        ← pixel-space simulation extent;
 *                                              source of truth for every
 *                                              boid_wrap / toroidal_delta
 *
 *   Every persistent simulation value is reachable from a single Scene*.
 *   Terminal-side state (cols, rows, ncurses lifecycle) lives on the
 *   Screen struct in §8; the two POSIX signal flags live on App in §9
 *   because signal handlers cannot receive a context pointer.
 *
 * Intent
 *   One instance lives inside App on the file-scope g_app and is passed
 *   by pointer to every tick, renderer, and input handler.  Pure
 *   observers take `const Scene *`; mutators take `Scene *`.  The
 *   signature alone communicates which kind of access each callee
 *   performs.
 *
 * Why a Scene-rooted hierarchy (not file-scope globals)
 *   • Replaceability: a future split-screen demo or recorded replay
 *     allocates multiple Scenes — impossible with globals.
 *   • Testability: scene_tick becomes a pure transformation s → s'
 *     with no hidden inputs.
 *   • Reading aid: every helper begins with `Scene *s` (or const), so
 *     the reader has ONE place to look for "what state exists".
 *   • Layered docs: each sub-struct (Flock, SimControls, World) owns
 *     its OWN invariants — Scene's docblock doesn't need to repeat them.
 *
 * Why FLOCKS=3 (and not 1, or N)
 *   • Three is the smallest count that exposes inter-flock dynamics:
 *     leaders bounce off each other's flocks in MODE_BOIDS, one flock
 *     can chase another in MODE_PREDATOR, and the three rainbow hues
 *     (0°, 120°, 240°) are visually distinct without colour overlap.
 *   • Fixed at compile time so the Scene struct has a deterministic
 *     size — no dynamic allocation, no resize logic when the user
 *     wants more / fewer flocks.  Adjusting FLOCKS in §1 propagates
 *     to color_init, scene_init, scene_tick_predator without code
 *     changes.
 *
 * Members
 *   flocks[FLOCKS]   the THREE flock instances.  Index 0 is special
 *                    in MODE_PREDATOR — it's the predator; 1 and 2 are
 *                    prey.  In all other modes the three are
 *                    indistinguishable except by colour and starting
 *                    quadrant (see flock_init's k_origin_x/y table).
 *
 *   sim              the user-facing knobs (see SimControls docblock).
 *                    Read by scene_tick (mode dispatch + pause check)
 *                    and screen_draw (HUD labels); written by
 *                    app_handle_key.
 *
 *   world            pixel-space simulation extent (see World
 *                    docblock).  Derived from Screen.cols / Screen.rows
 *                    by world_from_terminal; refreshed by app_do_resize
 *                    on every SIGWINCH so the simulation always
 *                    matches the visible terminal.
 *
 * Invariants
 *   All sub-struct invariants hold (Flock.n clamp, SimControls.mode
 *   range, SimControls.vicsek_noise clamp, World.width/height > 0).
 *   g_running / g_need_resize are NOT here — POSIX signal handlers
 *   need file-scope sig_atomic_t storage; they live on App in §9.
 *
 * References
 *   [1] Reynolds 1987 — the simulation owns the agent vector; this
 *       struct's flocks[] layout matches that reference design.
 *   [10] Wikipedia "Boids" — same single-owner pattern in every
 *        major-language implementation.
 *
 * See also: bounce.c "Scene" (single body) and crowd.c "Scene" (single
 *   flock) for two reductions of the same pattern; this Scene's
 *   contribution is the FLOCKS-sized array + mode-driven dispatch.
 */
typedef struct {
  Flock       flocks[FLOCKS];  /* the three flocks; index 0 = predator      */
                               /* in MODE_PREDATOR                          */
  SimControls sim;             /* mode + paused + vicsek_noise              */
  World       world;           /* pixel-space extent (source of truth for   */
                               /* every boid_wrap + toroidal_delta)         */
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);

  /* (1) user-facing knobs */
  s->sim.mode         = MODE_BOIDS;
  s->sim.paused       = false;
  s->sim.vicsek_noise = VICSEK_NOISE_DEFAULT;

  /* (2) world extent — derived from the current terminal size */
  s->world = world_from_terminal(cols, rows);

  /* (3) place each flock in its starting quadrant */
  for (int i = 0; i < FLOCKS; i++)
    flock_init(&s->flocks[i], i, FOLLOWERS_DEFAULT, s->world);
}

/* ── predator-prey tick (MODE_PREDATOR) ──────────────────────────────── */

/*
 * predator_leader_tick — advance the predator's (flock 0) leader.
 *
 * If any prey boid is within PREDATOR_CHASE_RADIUS the leader steers toward
 * the nearest one; otherwise it wanders normally. The wander_angle is set
 * to the bearing of the target, so the same LEADER_SPEED constant is used.
 *
 * prey_flocks: pointer to flocks[1] (first prey flock).
 * n_prey:      number of prey flocks (FLOCKS - 1 = 2).
 */
/*
 * predator_find_nearest_prey — closest-prey scan inside the predator's
 * sight range.
 *
 *   nearest_dist = PREDATOR_CHASE_RADIUS + epsilon         (sentinel)
 *   for each prey flock pf:
 *     consider pf.leader AND every pf.followers[i]:
 *       d = toroidal_distance(hunter_pos, prey_pos)
 *       if d < nearest_dist:
 *         update nearest_dist + target
 *   return true if at least one prey was inside range.
 *
 * Returns false when the predator has nothing to chase — the caller
 * falls back to a normal wander.
 */
static bool predator_find_nearest_prey(float hunter_px, float hunter_py,
                                       const Flock *prey_flocks, int n_prey,
                                       World world,
                                       float *target_px, float *target_py) {
  /* Sentinel: any real distance ≤ PREDATOR_CHASE_RADIUS will beat this
   * initial value, so the first in-range candidate wins automatically.
   * Anything still equal to the sentinel at end-of-scan = no prey found. */
  const float OUT_OF_SIGHT_SENTINEL = PREDATOR_CHASE_RADIUS + 1.0f;
  float nearest_distance = OUT_OF_SIGHT_SENTINEL;
  bool  found            = false;

  for (int fi = 0; fi < n_prey; fi++) {
    /* candidate list = prey leader + every active prey follower */
    const Boid *candidates[1 + FOLLOWERS_MAX];
    int n_candidates = 0;
    candidates[n_candidates++] = &prey_flocks[fi].leader;
    for (int i = 0; i < prey_flocks[fi].n; i++)
      candidates[n_candidates++] = &prey_flocks[fi].followers[i];

    /* linear scan for the closest candidate to (hunter_px, hunter_py) */
    for (int k = 0; k < n_candidates; k++) {
      const Boid *prey = candidates[k];
      float offset_x   = toroidal_delta(hunter_px, prey->px, world.width);
      float offset_y   = toroidal_delta(hunter_py, prey->py, world.height);
      float distance   = hypotf(offset_x, offset_y);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        *target_px = prey->px;
        *target_py = prey->py;
        found = true;
      }
    }
  }
  return found;
}

static void predator_leader_tick(Flock *predator, const Flock *prey_flocks,
                                 int n_prey, float dt, World world) {
  /* (1) scan for nearest prey within PREDATOR_CHASE_RADIUS */
  float target_px = 0.0f, target_py = 0.0f;
  bool  has_target = predator_find_nearest_prey(predator->leader.px,
                                                predator->leader.py,
                                                prey_flocks, n_prey, world,
                                                &target_px, &target_py);

  /* (2) heading: lock on to prey bearing if any, else random-walk */
  if (has_target) {
    snap_angle_to_bearing(&predator->leader, target_px, target_py, world,
                          &predator->wander_angle);
  } else {
    predator->wander_angle += rand_uniform_signed() * WANDER_JITTER;
  }

  /* (3) integrate at constant LEADER_SPEED, wrap to torus */
  integrate_leader_at_constant_speed(&predator->leader,
                                      predator->wander_angle, dt, world);
}

/*
 * prey_boids_steer — boids rules (separation + alignment + cohesion) PLUS
 * a flee impulse away from any predator boid within PREY_FLEE_RADIUS.
 *
 * The flee force is added after boids_steer (which writes out_vx/out_vy),
 * so we first call boids_steer and then accumulate the flee vector on top.
 * W_FLEE (3.0) >> W_COHESION (0.5) so prey actually break formation rather
 * than just jostling in place.
 */
/*
 * accumulate_flee_from_one — add a unit-push-away vector if `threat` is
 * inside PREY_FLEE_RADIUS of `self`.
 *
 *   d  = toroidal_distance(self, threat)
 *   if d < PREY_FLEE_RADIUS:
 *     flee -= unit(offset_to_threat)
 *
 * Unscaled (caller weights + clamps).  Reused for predator leader AND
 * every predator follower.
 */
static void accumulate_flee_from_one(const Boid *self, const Boid *threat,
                                     World world,
                                     float *flee_x, float *flee_y) {
  const float DIST_EPSILON = 0.001f;

  /* toroidal offset self → threat */
  float offset_x = toroidal_delta(self->px, threat->px, world.width);
  float offset_y = toroidal_delta(self->py, threat->py, world.height);
  float distance_to_threat = hypotf(offset_x, offset_y);

  bool inside_panic_zone = distance_to_threat < PREY_FLEE_RADIUS;
  bool well_separated    = distance_to_threat > DIST_EPSILON;
  if (!inside_panic_zone || !well_separated) return;

  /* unit vector pointing AWAY from threat (subtract offset = flip direction) */
  float away_from_threat_x = -offset_x / distance_to_threat;
  float away_from_threat_y = -offset_y / distance_to_threat;

  *flee_x += away_from_threat_x;
  *flee_y += away_from_threat_y;
}

/*
 * accumulate_flee_from_predator — sum unit-push vectors away from EVERY
 * predator boid (its leader + every follower) within PREY_FLEE_RADIUS.
 *
 *   for each predator boid p:
 *     accumulate_flee_from_one(self, p, world, &flee)
 */
static void accumulate_flee_from_predator(const Boid *self,
                                          const Flock *predator,
                                          World world,
                                          float *flee_x, float *flee_y) {
  accumulate_flee_from_one(self, &predator->leader, world, flee_x, flee_y);
  for (int i = 0; i < predator->n; i++)
    accumulate_flee_from_one(self, &predator->followers[i], world,
                              flee_x, flee_y);
}

static void prey_boids_steer(const Flock *f, int idx, const Flock *predator,
                             World world, float *out_vx, float *out_vy) {
  const float FLEE_EPSILON = 0.001f;

  /* (1) baseline: normal boids steering (sep+align+coh+leader pull) */
  boids_steer(f, idx, world, out_vx, out_vy);

  /* (2) Σ away-from-threat unit-pushes from every predator boid in range */
  const Boid *b = &f->followers[idx];
  float flee_sum_x = 0.0f, flee_sum_y = 0.0f;
  accumulate_flee_from_predator(b, predator, world, &flee_sum_x, &flee_sum_y);

  /* (3) normalise to a unit panic-direction, apply at W_FLEE · MAX_STEER.
   *     W_FLEE > W_COHESION so prey ACTUALLY break formation instead of
   *     just jostling in place inside the flock. */
  float flee_magnitude = hypotf(flee_sum_x, flee_sum_y);
  if (flee_magnitude > FLEE_EPSILON) {
    float panic_dir_x = flee_sum_x / flee_magnitude;
    float panic_dir_y = flee_sum_y / flee_magnitude;
    *out_vx += panic_dir_x * W_FLEE * MAX_STEER;
    *out_vy += panic_dir_y * W_FLEE * MAX_STEER;
  }
}

/*
 * scene_tick_predator — one predator-prey physics step.
 *
 * Flock 0 (predator):
 *   - Leader hunts nearest prey boid within PREDATOR_CHASE_RADIUS.
 *   - Followers chase their own leader (same as MODE_CHASE).
 *
 * Flocks 1-2 (prey):
 *   - Leaders wander normally.
 *   - Followers: boids_steer + flee from any predator boid in range.
 *
 * All prey flocks run simultaneously (same two-stage pattern), so no
 * prey flock reacts to the already-moved positions of another.
 */
/*
 * tick_predator_pack — advance flock 0 (the hunter) for one physics step.
 *
 *   (1) predator leader: lock onto nearest prey, else wander
 *   (2) followers: chase OWN leader (same as MODE_CHASE)
 *   (3) stage-2 write: clamp / integrate / wrap
 *
 * Prey state is read-only here; the prey flocks update in a later loop.
 */
static void tick_predator_pack(Flock *predator, const Flock *prey_flocks,
                                int n_prey, float dt, World world) {
  /* (1) predator leader: hunt-or-wander */
  predator_leader_tick(predator, prey_flocks, n_prey, dt, world);

  /* (2) predator followers: chase their own leader */
  float new_vx[FOLLOWERS_MAX], new_vy[FOLLOWERS_MAX];
  for (int i = 0; i < predator->n; i++)
    chase_steer(predator, i, world, &new_vx[i], &new_vy[i]);

  /* (3) stage-2 write */
  apply_followers_step(predator, new_vx, new_vy, dt, world);
}

/*
 * tick_one_prey_flock — advance one prey flock for one physics step.
 *
 *   (1) prey leader wanders normally (no flee on the leader)
 *   (2) followers: normal boids steer PLUS flee from any predator boid
 *       within PREY_FLEE_RADIUS
 *   (3) stage-2 write
 */
static void tick_one_prey_flock(Flock *prey, const Flock *predator,
                                 float dt, World world) {
  /* (1) prey leader */
  leader_tick(prey, dt, world);

  /* (2) prey followers: boids + flee-from-predator */
  float new_vx[FOLLOWERS_MAX], new_vy[FOLLOWERS_MAX];
  for (int i = 0; i < prey->n; i++)
    prey_boids_steer(prey, i, predator, world, &new_vx[i], &new_vy[i]);

  /* (3) stage-2 write */
  apply_followers_step(prey, new_vx, new_vy, dt, world);
}

static void scene_tick_predator(Scene *s, float dt) {
  World world = s->world;
  Flock *predator = &s->flocks[0];

  /* (1) predator pack — hunts nearest prey */
  tick_predator_pack(predator, &s->flocks[1], FLOCKS - 1, dt, world);

  /* (2) every prey flock — flees from predator boids */
  for (int fi = 1; fi < FLOCKS; fi++)
    tick_one_prey_flock(&s->flocks[fi], predator, dt, world);
}

static void scene_tick(Scene *s, float dt) {
  if (s->sim.paused)
    return;

  if (s->sim.mode == MODE_PREDATOR) {
    scene_tick_predator(s, dt);
    return;
  }

  for (int i = 0; i < FLOCKS; i++)
    flock_tick(&s->flocks[i], s->sim.mode, s->sim.vicsek_noise, dt, s->world);
}

/*
 * follower_brightness — choose A_BOLD or A_NORMAL based on proximity to leader.
 *
 * Uses toroidal shortest-path distance so the gradient stays correct even
 * when the leader has just wrapped to the opposite edge — followers that are
 * physically close across the wrap boundary still glow bright.
 */
static int follower_brightness(const Boid *follower, const Boid *leader,
                               World world) {
  float dx = toroidal_delta(follower->px, leader->px, world.width);
  float dy = toroidal_delta(follower->py, leader->py, world.height);
  float ratio = hypotf(dx, dy) / PERCEPTION_RADIUS;
  return (ratio < 0.35f) ? A_BOLD : A_NORMAL;
}

/*
 * RenderTiming — sub-tick interpolation factor passed from main into
 * the renderer.
 *
 * Intent
 *   Each frame the main loop is part-way into the NEXT physics tick
 *   (the fixed-step accumulator leaves a fractional slot of leftover
 *   time).  Drawing always at the last tick's position would visibly
 *   stutter when render rate ≠ sim rate — boids would appear to "step"
 *   one cell at a time instead of glide smoothly.
 *
 *   This struct carries the two scalars draw_boid needs to project a
 *   boid's position forward to wall-clock "now":
 *
 *     draw_pos = pos + vel · alpha · dt_sec
 *
 *   where alpha ∈ [0,1) is how far into the next tick we are.  At
 *   alpha=0 we draw at the last tick's pos exactly; at alpha→1 we draw
 *   one full tick of motion ahead, just before the next tick will
 *   advance pos by that same amount.  The result is sub-pixel-smooth
 *   motion at ANY render rate regardless of sim rate.
 *
 * Why a struct (not two args, not stored on Scene)
 *   • These two values are computed in main() per FRAME, not per tick.
 *     They have no place on Scene (which is per-TICK state).
 *   • Threading two args through scene_draw → draw_boid → (clamping,
 *     cell conversion) cluttered every signature.  Bundling makes it
 *     one named value: "the timing context for this frame".
 *   • A future addition (e.g. extrapolation cap, slow-mo factor) would
 *     extend this struct without touching any caller.
 *
 * Why velocity-based extrapolation (not prev/cur lerp)
 *   The standard interpolation pattern is `lerp(prev_pos, cur_pos,
 *   alpha)`, requiring boids to store BOTH last-tick and current
 *   positions.  This file uses `cur_pos + vel · alpha · dt_sec` which
 *   needs only the current state.  Trade-offs:
 *     + halves Boid struct size (no prev_px/prev_py)
 *     + correct exactly when motion is uniform between ticks (which is
 *       overwhelmingly the case here — forces change slowly compared
 *       to dt_sec)
 *     − slightly wrong during sharp turns (overshoots by ~½ tick of
 *       angular change) but indistinguishable at 60 Hz / typical
 *       steering weights.
 *   See [9] Fiedler for both variants; either works for soft motion.
 *
 * Members
 *   alpha   sub-tick fraction in [0, 1) — leftover_ns / tick_ns from
 *           the accumulator in main().  Drives forward-extrapolation
 *           amount.
 *   dt_sec  one physics tick length in SECONDS = TICK_NS / NS_PER_SEC.
 *           Used to convert vel (px/s) × alpha (dimensionless) into
 *           a position delta (px) of the right magnitude.
 *
 * Invariants
 *   0.0f ≤ alpha < 1.0f (enforced by the `while (sim_accum >= tick_ns)`
 *   drain loop in main).
 *   dt_sec > 0 (constant; computed once at the top of main).
 *
 * References
 *   [9] Fiedler 2014, "Fix Your Timestep!" — the canonical reference
 *       for fixed-step physics + sub-tick render interpolation.  This
 *       struct is the data half of that pattern; main()'s `sim_accum`
 *       loop is the logic half.
 */
typedef struct {
  float alpha;   /* sub-tick fraction in [0,1) — leftover / tick_ns       */
  float dt_sec;  /* one physics tick in seconds — TICK_NS / NS_PER_SEC    */
} RenderTiming;

/*
 * draw_boid — draw a single boid into window w using render interpolation.
 *
 * Interpolation: project position forward by (alpha × dt_sec) seconds from
 * the last physics tick using current velocity. This makes drawn positions
 * match wall-clock "now" rather than the last tick boundary, eliminating
 * micro-stutter. Same technique as bounce.c §6 scene_draw.
 */
/*
 * extrapolate_to_render_pos — project a boid's pixel position forward
 * by the sub-tick alpha.  See RenderTiming docblock + [9] Fiedler for
 * the rationale ("draw at wall-clock NOW", not at last tick boundary).
 *
 *   draw_pos = pos + vel · alpha · dt_sec
 */
static void extrapolate_to_render_pos(const Boid *b, RenderTiming rt,
                                       float *out_px, float *out_py) {
  /* fraction of a physics tick we are PAST the last tick boundary,
   * converted to seconds: (alpha ∈ [0,1)) · (one tick in seconds) */
  float sub_tick_seconds = rt.alpha * rt.dt_sec;

  /* Euler extrapolation: pos += vel · sub_tick_seconds  [Fiedler 2014] */
  *out_px = b->px + b->vx * sub_tick_seconds;
  *out_py = b->py + b->vy * sub_tick_seconds;
}

/*
 * clamp_to_world_box — pin (px, py) inside [0, world.width] ×
 * [0, world.height].  alpha < 1 means any overshoot is sub-cell, so
 * this is a guard rather than a frequent correction.
 */
static void clamp_to_world_box(float *px, float *py, World world) {
  if (*px < 0)              *px = 0;
  if (*px > world.width)    *px = world.width;
  if (*py < 0)              *py = 0;
  if (*py > world.height)   *py = world.height;
}

/*
 * paint_glyph_at_cell — stamp one ASCII char at terminal cell (cx, cy)
 * with the given color/attribute, IF inside the visible window.
 *
 *   - bounds-check against (cols, rows) is the only thing standing
 *     between physics overshoot and an ncurses out-of-bounds write.
 *   - (chtype)(unsigned char) cast prevents sign-extension on chars
 *     > 127 (per CLAUDE.md "Common ncurses Bugs").
 */
static void paint_glyph_at_cell(WINDOW *w, int cx, int cy,
                                 char ch, int color_attr,
                                 int cols, int rows) {
  if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
  wattron (w, color_attr);
  mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
  wattroff(w, color_attr);
}

static void draw_boid(WINDOW *w, const Boid *b, char ch, int color_attr,
                      int cols, int rows, World world, RenderTiming rt) {
  /* (1) extrapolate to wall-clock NOW for sub-tick smoothness */
  float draw_px, draw_py;
  extrapolate_to_render_pos(b, rt, &draw_px, &draw_py);

  /* (2) clamp to world box, then convert pixel → cell */
  clamp_to_world_box(&draw_px, &draw_py, world);
  int cx = px_to_cell_x(draw_px);
  int cy = px_to_cell_y(draw_py);

  /* (3) paint glyph if inside the visible terminal window */
  paint_glyph_at_cell(w, cx, cy, ch, color_attr, cols, rows);
}

/*
 * scene_draw — draw all flocks using interpolated positions.
 *
 * Draw order per flock: followers first, leader last.
 * Leaders are drawn on top so they're never obscured by their own flock.
 */
static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows,
                       RenderTiming rt) {
  World world = s->world;

  for (int fi = 0; fi < FLOCKS; fi++) {
    const Flock *f = &s->flocks[fi];

    /* Followers: flock color, direction char, brightness by toroidal distance
     */
    int follower_pair = follower_color_pair(fi);
    for (int i = 0; i < f->n; i++) {
      const Boid *b = &f->followers[i];
      char ch = velocity_dir_char(b->vx, b->vy, fi);
      int attr = COLOR_PAIR(follower_pair) |
                 follower_brightness(b, &f->leader, world);
      draw_boid(w, b, ch, attr, cols, rows, world, rt);
    }

    /* Leader: its own color, direction char (shows heading), always bold */
    int leader_pair = leader_color_pair(fi);
    char leader_ch = velocity_dir_char(f->leader.vx, f->leader.vy, fi);
    draw_boid(w, &f->leader, leader_ch, COLOR_PAIR(leader_pair) | A_BOLD,
              cols, rows, world, rt);
  }
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

/*
 * Screen — cell-space (terminal) extent + ncurses-lifecycle layer.
 *
 * Intent
 *   Owns the TERMINAL side of the world.  Where `World` (in §4) tracks
 *   the pixel-space simulation box, this struct tracks the cell-space
 *   terminal grid that ncurses paints onto.  The two are linked but
 *   live in DIFFERENT spaces:
 *
 *      World.width  = Screen.cols × CELL_W           (CELL_W = 8 px)
 *      World.height = Screen.rows × CELL_H           (CELL_H = 16 px)
 *
 *   Keeping them in separate structs prevents the entire "rendered too
 *   early in cell space" bug class — physics never sees cells, the
 *   renderer never sees pixels EXCEPT through the one px_to_cell_x/y
 *   bridge in §4.
 *
 * Why a tiny 2-field struct (and not flat ints on App)
 *   • Lifecycle isolation: screen_init / screen_resize / screen_free
 *     are the ONLY functions that touch ncurses' initscr/endwin.  They
 *     all take `Screen *` to make this layer explicit at the type
 *     level — anything else taking `Screen *` (or `const`) gets to
 *     READ the extent without permission to bring ncurses up/down.
 *   • Symmetry with World: simulation and rendering each get one
 *     struct named for the space they live in; reading the code feels
 *     like a single design instead of two coincidences.
 *   • Future-proof: a future feature (a status panel, split-screen,
 *     viewport offsets) adds fields here without disturbing Scene or
 *     the steering algorithms.
 *
 * Why duplicated with World (not "single source of truth")
 *   Could store cols/rows on World and remove Screen entirely.
 *   Avoided because:
 *     • Physics doesn't need cols/rows — it needs PIXEL width/height.
 *       Adding cell-space fields to World mixes spaces in the type
 *       intended to be pure pixel.
 *     • ncurses' `getmaxyx` reads cols/rows directly; storing them
 *       alongside the ncurses lifecycle keeps the read close to its
 *       source.
 *     • scene_init / app_do_resize update both in lockstep — they're
 *       NEVER out of sync because exactly one function refreshes both
 *       at the same call site (world_from_terminal(s->cols, s->rows)).
 *
 * Members
 *   cols   terminal width in CELLS, as read by getmaxyx from stdscr.
 *          Used by the renderer for bounds-checking after the px →
 *          cell conversion: `if (cx >= 0 && cx < cols && ...)`.  Also
 *          drives HUD positioning: top-status bar is right-aligned to
 *          cols, key hint is on the last row.
 *
 *   rows   terminal height in CELLS.  Same role on the vertical axis;
 *          used for cell-bounds clamping and for `rows - 1` as the row
 *          index of the bottom key hint.
 *
 * Invariants
 *   cols > 0 AND rows > 0 (set by screen_init via getmaxyx).
 *   Scene.world.width  == cols × CELL_W,
 *   Scene.world.height == rows × CELL_H,
 *   maintained by scene_init and app_do_resize.
 *
 * References
 *   None directly — terminal extent is a rendering substrate concern,
 *   not an algorithmic one.  The aspect-ratio compensation (CELL_W vs
 *   CELL_H) is project-specific (see CLAUDE.md §"Coordinate / Physics").
 */
typedef struct {
  int cols;   /* terminal width in cells; getmaxyx via screen_init    */
  int rows;   /* terminal height in cells                              */
} Screen;

static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1); /* never interrupt output to poll stdin */
  color_init();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) {
  (void)s;
  endwin();
}

static void screen_resize(Screen *s) {
  endwin();
  refresh(); /* re-reads LINES and COLS after terminal resize */
  getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw — build one complete frame in stdscr (ncurses' newscr).
 *
 * Order:
 *   1. erase() — clear newscr so stale boid glyphs become spaces.
 *   2. scene_draw() — write all boids at interpolated positions.
 *   3. HUD line — overwrite top-left after boids (always on top).
 *
 * Nothing reaches the terminal until screen_present() is called.
 */
/*
 * count_total_boids — total agents on screen: every flock's leader
 * (1 each) plus its active follower count.  Used by the HUD only.
 */
static int count_total_boids(const Scene *sc) {
  int total = 0;
  for (int i = 0; i < FLOCKS; i++)
    total += sc->flocks[i].n + 1;  /* +1 for the leader */
  return total;
}

/*
 * format_hud_status — write the top-row status text into `buf`.
 * MODE_VICSEK shows the live noise value so the user sees the η knob
 * change as they press n / m; other modes omit it for brevity.
 */
static void format_hud_status(const Scene *sc, double fps, int total_boids,
                              char *buf, size_t buflen) {
  const char *run_state = sc->sim.paused ? "PAUSED " : "running";
  if (sc->sim.mode == MODE_VICSEK) {
    snprintf(buf, buflen,
             " %5.0f fps  mode:%s  noise:%.2f  boids:%d  %s ",
             fps, k_mode_names[sc->sim.mode], sc->sim.vicsek_noise,
             total_boids, run_state);
  } else {
    snprintf(buf, buflen,
             " %5.0f fps  mode:%s  boids:%d  %s ",
             fps, k_mode_names[sc->sim.mode], total_boids, run_state);
  }
}

/*
 * hud_paint_text — stamp `text` at (row, col) with a pair + A_BOLD.
 * Centralises the attron / mvprintw / attroff sandwich used by both
 * HUD rows.
 */
static void hud_paint_text(int row, int col, int pair, const char *text) {
  attron (COLOR_PAIR(pair) | A_BOLD);
  mvprintw(row, col, "%s", text);
  attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* draw_hud_status — top-right data line: fps, mode, noise, count, run-state. */
static void draw_hud_status(const Screen *s, const Scene *sc, double fps) {
  enum { HUD_TOP_ROW = 0, HUD_BUFLEN = 160 };

  int  total_boids = count_total_boids(sc);
  char buf[HUD_BUFLEN];
  format_hud_status(sc, fps, total_boids, buf, sizeof buf);

  /* right-aligned: column = cols − strlen(buf), clamped non-negative */
  int right_col = s->cols - (int)strlen(buf);
  if (right_col < 0) right_col = 0;
  hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, buf);
}

/* draw_hud_hint — bottom-left key hint, fixed string. */
static void draw_hud_hint(const Screen *s) {
  static const char *KEY_HINT =
      " q:quit  spc:pause  1:boids 2:chase 3:vicsek 4:orbit 5:predator  "
      "n/m:noise  +/-:boids  r:reset ";
  hud_paint_text(s->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

static void screen_draw(Screen *s, Scene *sc, double fps, RenderTiming rt) {
  /* (1) clear the offscreen buffer */
  erase();
  /* (2) paint the simulation */
  scene_draw(sc, stdscr, s->cols, s->rows, rt);
  /* (3) HUD: data on top, key hint on bottom (always on top of boids) */
  draw_hud_status(s, sc, fps);
  draw_hud_hint  (s);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

/*
 * FpsCounter — rolling-window frame-rate estimator.
 *
 * Intent
 *   Per-frame fps would jitter wildly — at ~60 fps, a single 1 ms
 *   variation in dt swings the instantaneous reading by ~6 fps.  This
 *   struct accumulates frame_count and elapsed nanoseconds over a
 *   fixed window (FPS_UPDATE_MS = 500 ms by default) and emits a
 *   smoothed `display` figure each time the window fills.  HUD then
 *   reads `display` twice per second, giving the user a stable number
 *   that still responds to real performance changes within ~500 ms.
 *
 * Algorithm
 *   per frame:
 *     window_ns   += dt
 *     frame_count += 1
 *     if window_ns ≥ FPS_WINDOW_NS:
 *       display    = frame_count / (window_ns / NS_PER_SEC)
 *       reset window_ns and frame_count
 *
 *   Equivalent to a non-overlapping moving average of frames-per-
 *   window; cheaper than an exponential filter and predictable in its
 *   responsiveness.
 *
 * Lifecycle
 *   fps_counter_init  — zero everything (once at startup).
 *   fps_counter_tick  — call once per frame with the frame's dt (ns).
 *                       Updates `display` only when the window fills.
 *
 * Why a struct (not three flat main() locals)
 *   • Same logic appears in every demo in this project; bundling the
 *     three state variables under one name makes the loop body short
 *     and makes the fps machinery a SINGLE thing the reader can locate
 *     (and skip if they're focused on the steering algorithm).
 *   • Lifts the smoothing logic into named helpers (init / tick) that
 *     read like spec instead of arithmetic in the middle of the main
 *     loop.
 *
 * Members
 *   frame_count   number of frames observed in the current window.
 *                 Resets to 0 when the window fills and `display` is
 *                 recomputed.
 *
 *   window_ns     time accumulated in the current window, in
 *                 nanoseconds.  Compared against FPS_WINDOW_NS (=
 *                 FPS_UPDATE_MS × NS_PER_MS) to decide when to emit a
 *                 new smoothed reading.
 *
 *   display       most recent smoothed fps figure (frames per second).
 *                 Read by screen_draw for the HUD.  Updated only when
 *                 a window completes — between updates, the HUD shows
 *                 a constant value rather than flicker.
 *
 * Invariants
 *   frame_count ≥ 0 AND window_ns ≥ 0 between resets.
 *   display ≥ 0 (zero until the first window completes).
 *
 * References
 *   [9] Fiedler 2014, "Fix Your Timestep!" — recommends a fixed
 *       smoothing window for any in-game fps display, on the same
 *       reasoning: per-frame readings are unstable, fixed-window
 *       averaging is the simplest robust answer.
 */
typedef struct {
  int     frame_count;   /* frames observed in the current window           */
  int64_t window_ns;     /* nanoseconds accumulated in the current window   */
  double  display;       /* smoothed fps; HUD reads this value              */
} FpsCounter;

static void fps_counter_init(FpsCounter *f) {
  f->frame_count = 0;
  f->window_ns   = 0;
  f->display     = 0.0;
}

/* Tally one frame; emit a fresh smoothed reading once the window fills. */
static void fps_counter_tick(FpsCounter *f, int64_t dt) {
  const int64_t FPS_WINDOW_NS = (int64_t)FPS_UPDATE_MS * NS_PER_MS;

  f->frame_count++;
  f->window_ns += dt;
  if (f->window_ns < FPS_WINDOW_NS) return;

  f->display =
      (double)f->frame_count / ((double)f->window_ns / (double)NS_PER_SEC);
  f->frame_count = 0;
  f->window_ns   = 0;
}

/*
 * App — top-level container for every persistent value in the program.
 *
 * Layered ownership
 *
 *     App
 *       ├── scene      : Scene         ← simulation state
 *       │      ├── flocks[]: Flock[]       ← agent pool
 *       │      ├── sim    : SimControls   ← mode + paused + vicsek_noise
 *       │      └── world  : World         ← pixel-space extent
 *       │
 *       ├── screen     : Screen        ← terminal cell extent
 *       ├── fps        : FpsCounter    ← rolling-window frame-rate
 *       │
 *       ├── running    : sig_atomic_t  ← cleared by on_exit_signal;
 *       │                                loops until 0
 *       └── need_resize: sig_atomic_t  ← set by on_resize_signal;
 *                                        main() reacts at next iteration
 *
 * Intent
 *   The single "where everything lives" structure.  One static instance
 *   `g_app` is the sole file-scope variable in the program; every
 *   helper reaches state through `&g_app` (signal handlers) or through
 *   an `App *app` argument (everything else).
 *
 * Why a file-scope global (the project's only one)
 *   POSIX `signal()` handlers have signature `void (int)` — they can't
 *   receive a context pointer.  The two flags `running` and
 *   `need_resize` MUST be reachable through file-scope storage so
 *   on_exit_signal / on_resize_signal can write them.  Everything else
 *   (scene, screen, fps) could in principle live on main()'s stack —
 *   they sit here too only so the single global pointer reaches them
 *   in one hop.
 *
 * Why `volatile sig_atomic_t` for the flags
 *   • `sig_atomic_t` is guaranteed by C11 (§5.1.2.3) to be readable
 *     and writable in a single, uninterruptible operation from a
 *     signal handler.  Any wider type risks a torn read.
 *   • `volatile` prevents the compiler from caching the flag's value
 *     in a register inside the main loop — the loop MUST re-read each
 *     iteration in case a signal fired between iterations.
 *   Together they're the canonical "flag set by signal, polled by
 *   main" pattern (Stevens APUE §10.3, or any UNIX systems-programming
 *   text).
 *
 * Why ONE App (not one Scene + one Screen + flat flags)
 *   • Replaceability: a future feature (multiple simultaneous demos in
 *     tabs, recorded replay player) allocates multiple Apps — flat
 *     globals would prevent that.
 *   • Reading aid: every helper begins with `App *app` (or sub-struct
 *     pointer), giving the reader ONE place to look for "all the state
 *     in this program".
 *   • Signal-handler reach: handlers see ONE thing, write ONE flag.
 *     Adding state to App never disturbs the signal-handling
 *     contract.
 *
 * Members
 *   scene         the simulation half — see Scene docblock.  Read by
 *                 every tick / draw; written by app_handle_key
 *                 (mode/pause/noise) and app_do_resize (world).
 *
 *   screen        the terminal-substrate half — see Screen docblock.
 *                 Read by screen_draw + scene_draw for bounds; written
 *                 by screen_init / screen_resize.
 *
 *   fps           rolling fps state — see FpsCounter docblock.  Read
 *                 by screen_draw (HUD); written by fps_counter_tick
 *                 in main()'s loop.
 *
 *   running       0 = exit, 1 = keep looping.  Cleared by:
 *                   - on_exit_signal (SIGINT / SIGTERM)
 *                   - app_handle_key on 'q' / 'Q' / ESC
 *                 Polled by main's `while (app->running)` test.
 *
 *   need_resize   0 = idle, 1 = SIGWINCH pending.  Set by
 *                 on_resize_signal; cleared by app_do_resize once the
 *                 main loop reaches the resize step.  Decoupling the
 *                 signal arrival from the resize work means
 *                 app_do_resize can do ncurses-unsafe things (endwin /
 *                 refresh / getmaxyx / boid clamping) without racing
 *                 the signal handler.
 *
 * Invariants
 *   running, need_resize ∈ {0, 1} (sig_atomic_t in this codebase).
 *   All sub-struct invariants hold (Scene, Screen, FpsCounter).
 *
 * References
 *   None directly — this is a project-structure pattern, not an
 *   algorithm.  Nystrom, "Game Programming Patterns" Ch.9 (Game Loop)
 *   describes the same "one root, signal flags via globals, everything
 *   else via pointer" arrangement; it's the standard skeleton for a
 *   real-time game loop in C.
 */
typedef struct {
  Scene      scene;        /* simulation state                              */
  Screen     screen;       /* terminal cell extent + ncurses lifecycle      */
  FpsCounter fps;          /* rolling-window fps for HUD                    */
  volatile sig_atomic_t running;     /* 0 = exit; SIGINT/TERM clear it      */
  volatile sig_atomic_t need_resize; /* 1 = SIGWINCH pending; cleared on    */
                                     /* handling                            */
} App;

static App g_app;

static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

/*
 * clamp_boid_inside_world — pin one boid's position back inside the
 * (possibly smaller) world box.  Used only after a SIGWINCH-driven
 * shrink; the regular boid_wrap handles the inside-of-world case.
 */
static void clamp_boid_inside_world(Boid *b, World world) {
  if (b->px >= world.width)   b->px = world.width  - 1.0f;
  if (b->py >= world.height)  b->py = world.height - 1.0f;
}

/*
 * clamp_all_boids_to_world — apply clamp_boid_inside_world to every
 * leader + active follower across every flock.  After a terminal
 * shrink, any boid that was past the new edge would otherwise sit
 * off-screen until natural toroidal wrap brought it back; clamping
 * immediately keeps the visible state consistent with the new extent.
 */
static void clamp_all_boids_to_world(Scene *sc) {
  World world = sc->world;
  for (int fi = 0; fi < FLOCKS; fi++) {
    Flock *f = &sc->flocks[fi];
    clamp_boid_inside_world(&f->leader, world);
    for (int i = 0; i < f->n; i++)
      clamp_boid_inside_world(&f->followers[i], world);
  }
}

static void app_do_resize(App *app) {
  /* (1) ncurses tear-down + re-init so stdscr matches new terminal size */
  screen_resize(&app->screen);

  /* (2) refresh Scene.world to the new extent */
  app->scene.world = world_from_terminal(app->screen.cols, app->screen.rows);

  /* (3) pull any boids that were past the new edge back inside */
  clamp_all_boids_to_world(&app->scene);

  /* (4) acknowledge the SIGWINCH so we don't re-handle it */
  app->need_resize = 0;
}

/*
 * adjust_vicsek_noise — move η by `delta` radians, clamp to
 * [VICSEK_NOISE_MIN, VICSEK_NOISE_MAX].  Lets the user traverse [3]'s
 * phase transition live on screen.
 */
static void adjust_vicsek_noise(SimControls *sim, float delta) {
  sim->vicsek_noise += delta;
  if (sim->vicsek_noise < VICSEK_NOISE_MIN) sim->vicsek_noise = VICSEK_NOISE_MIN;
  if (sim->vicsek_noise > VICSEK_NOISE_MAX) sim->vicsek_noise = VICSEK_NOISE_MAX;
}

/*
 * adjust_follower_count_all_flocks — change every flock's active
 * follower count by `delta`, clamped to [FOLLOWERS_MIN, FOLLOWERS_MAX].
 * All flocks change together so the visual balance is preserved.
 */
static void adjust_follower_count_all_flocks(Scene *s, int delta) {
  for (int i = 0; i < FLOCKS; i++) {
    int next = s->flocks[i].n + delta;
    if (next < FOLLOWERS_MIN) next = FOLLOWERS_MIN;
    if (next > FOLLOWERS_MAX) next = FOLLOWERS_MAX;
    s->flocks[i].n = next;
  }
}

static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;

  switch (ch) {
  case 'q': case 'Q': case 27 /* ESC */:
    return false;

  case ' ':  s->sim.paused = !s->sim.paused;     break;

  /* mode select — keys 1..5 pick a FlockMode */
  case '1':  s->sim.mode = MODE_BOIDS;           break;
  case '2':  s->sim.mode = MODE_CHASE;           break;
  case '3':  s->sim.mode = MODE_VICSEK;          break;
  case '4':  s->sim.mode = MODE_ORBIT;           break;
  case '5':  s->sim.mode = MODE_PREDATOR;        break;

  /* Vicsek noise η: only meaningful in MODE_VICSEK */
  case 'n': case 'N':
    adjust_vicsek_noise(&s->sim, -VICSEK_NOISE_STEP); break;
  case 'm': case 'M':
    adjust_vicsek_noise(&s->sim, +VICSEK_NOISE_STEP); break;

  /* follower count for every flock */
  case '=': case '+':
    adjust_follower_count_all_flocks(s, +1); break;
  case '-':
    adjust_follower_count_all_flocks(s, -1); break;

  /* reset everything to starting state */
  case 'r': case 'R':
    scene_init(s, app->screen.cols, app->screen.rows);
    break;

  default: break;
  }
  return true;
}

int main(void) {
  /* ── (a) RNG + signal handlers + atexit ─────────────────────── */
  srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  /* ── (b) bring up screen + scene ────────────────────────────── */
  App *app = &g_app;
  app->running = 1;
  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);
  fps_counter_init(&app->fps);

  /* ── (c) loop-local constants ───────────────────────────────── */
  const int64_t DT_CAP_NS       = 100 * NS_PER_MS;          /* avalanche guard */
  const int64_t FRAME_BUDGET_NS = NS_PER_SEC / RENDER_FPS;  /* render cadence */
  const int64_t TICK_LEN_NS     = TICK_NS(SIM_FPS);
  const float   TICK_LEN_SEC    = (float)TICK_LEN_NS / (float)NS_PER_SEC;

  int64_t frame_time = clock_ns();
  int64_t sim_accum  = 0;

  while (app->running) {
    int64_t frame_start = clock_ns();

    /* (1) handle pending SIGWINCH: reload extent + reset timers */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* (2) measure dt since last frame, capped to prevent physics avalanche */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > DT_CAP_NS) dt = DT_CAP_NS;

    /* (3) drain accumulator: fixed-step physics until caught up */
    sim_accum += dt;
    while (sim_accum >= TICK_LEN_NS) {
      scene_tick(&app->scene, TICK_LEN_SEC);
      sim_accum -= TICK_LEN_NS;
    }

    /* (4) sub-tick interpolation factor for the renderer */
    RenderTiming rt = {
        .alpha  = (float)sim_accum / (float)TICK_LEN_NS,
        .dt_sec = TICK_LEN_SEC,
    };

    /* (5) rolling-window fps counter */
    fps_counter_tick(&app->fps, dt);

    /* (6) sleep BEFORE render so I/O time doesn't eat the budget */
    int64_t budget_left = FRAME_BUDGET_NS - (clock_ns() - frame_start);
    clock_sleep_ns(budget_left);

    /* (7) draw + present */
    screen_draw(&app->screen, &app->scene, app->fps.display, rt);
    screen_present();

    /* (8) input */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
