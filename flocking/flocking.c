/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * flocking.c — five switchable flocking algorithms across three flocks.
 * Each flock follows a wandering leader; keys 1-5 switch the steering rule
 * (Reynolds boids, leader chase, Vicsek, orbit, predator-prey). Boids wrap
 * around the screen edges. Refs: Reynolds 1987 (boids) & 1999 (steering),
 * Vicsek et al. 1995 (align+noise), Couzin et al. 2002, Helbing et al. 2000
 * (escape panic), Fiedler "Fix Your Timestep!". Companion: flocking/crowd.c.
 */

/* M_PI comes from <math.h> only when _GNU_SOURCE is set. */
#define _GNU_SOURCE

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — counts, speeds, radii, weights, mode tunables ── */

/*
 * Physics runs in "pixels"; the screen is drawn in character cells. A cell is
 * about twice as tall as it is wide, so we give each cell 8 sub-pixel columns
 * and 16 sub-pixel rows. Working in pixels and only converting to cells at
 * draw time keeps diagonal motion and speed looking right on screen.
 */
#define CELL_W 8  /* sub-pixel columns per terminal cell */
#define CELL_H 16 /* sub-pixel rows    per terminal cell */

enum {
  SIM_FPS = 60,        /* physics ticks per second               */
  RENDER_FPS = 60,     /* display frame cap                      */
  FPS_UPDATE_MS = 500, /* HUD fps counter refresh interval (ms)  */
};

/*
 * Three flocks, 12 followers each (~39 boids total). Enough to look like a
 * real flock while the all-pairs neighbour scan stays cheap.
 */
enum {
  FLOCKS = 3,
  FOLLOWERS_DEFAULT = 12,
  FOLLOWERS_MIN = 3,
  FOLLOWERS_MAX = 20,
};

/*
 * Cruise speeds in pixels per second. BOID_SPEED sits well above the floor
 * where motion would start to look like a staircase; the leader is a touch
 * faster so it naturally pulls ahead of its flock. MIN/MAX bound every boid's
 * actual speed so they never fully stop and never run away.
 */
#define BOID_SPEED 280.0f   /* follower cruising speed, px/s */
#define LEADER_SPEED 340.0f /* leader cruising speed, px/s   */
#define MIN_SPEED 80.0f     /* floor — boids never fully stop */
#define MAX_SPEED 500.0f    /* ceiling — keeps flocks legible */

/*
 * How far a boid senses others. PERCEPTION is the wide "I can see you" range;
 * SEPARATION is the tighter personal-space bubble where boids push apart. The
 * ~1:3 ratio gives each boid breathing room while still reacting to the group.
 */
#define PERCEPTION_RADIUS 180.0f /* px — neighbor sensing range   */
#define SEPARATION_RADIUS 60.0f  /* px — minimum comfortable gap  */

/*
 * How loud each of the boids rules is. Separation wins so they don't crowd;
 * cohesion is quiet so groups stay loose instead of collapsing to a dot; the
 * leader pull is gentle so it nudges without overriding the flocking.
 */
#define W_SEPARATION 1.8f  /* repulsion from nearby boids       */
#define W_ALIGNMENT 1.0f   /* match average heading of group    */
#define W_COHESION 0.5f    /* drift toward group center of mass */
#define W_LEADER_PULL 0.4f /* gentle attraction toward leader   */

/* Cap on how hard a boid can steer in one tick — bigger turns curve, not snap. */
#define MAX_STEER 130.0f

/* How much the leader's heading can drift per tick — small, so it curves gently. */
#define WANDER_JITTER 0.10f /* max heading change per tick (radians) */

/*
 * Vicsek mode: each boid turns toward its neighbours' average heading, then
 * jitters by a random angle. That one noise knob slides the flock between two
 * extremes — low noise streams everyone the same way, high noise scatters
 * them. Keys n / m turn it down / up live.
 */
#define VICSEK_NOISE_DEFAULT 0.30f /* starting noise level, radians     */
#define VICSEK_NOISE_MIN 0.05f     /* near-perfect order (parallel beams) */
#define VICSEK_NOISE_MAX 1.80f     /* near-random (barely any coherence) */
#define VICSEK_NOISE_STEP 0.10f    /* per keypress increment            */

/*
 * Orbit mode: followers take evenly-spaced seats on a ring around the leader,
 * and the ring spins. Each follower chases its moving seat, making a spinning
 * halo. The ring's edge moves slower than BOID_SPEED, so followers keep up.
 */
#define ORBIT_RADIUS 120.0f /* px — ring radius around leader         */
#define ORBIT_SPEED 1.4f    /* rad/s — ring rotation speed            */

/*
 * Predator-prey mode: flock 0 hunts, flocks 1-2 flee. The predator chases the
 * nearest prey it can see (CHASE_RADIUS); prey panic and bolt when a predator
 * gets within FLEE_RADIUS. W_FLEE is much louder than cohesion so prey
 * actually scatter instead of just shuffling inside their flock.
 */
#define PREDATOR_CHASE_RADIUS 280.0f /* px — predator sight range        */
#define PREY_FLEE_RADIUS 160.0f      /* px — prey panic zone             */
#define W_FLEE 3.0f                  /* flee force weight (beats cohesion) */

/* Common time constants */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(fps) (NS_PER_SEC / (fps))

/* ── §2 clock — monotonic timer + sleep ── */

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

/* ── §3 color — flock + leader + HUD ncurses pairs ── */

/*
 * Each flock gets two colours: one for its followers, one for its leader. The
 * leader colour is picked to contrast its flock so it always stands out. Rich
 * 256-colour shades are used when the terminal has them, with plain 16-colour
 * stand-ins otherwise.
 *
 * Pair numbering: followers are pairs 1..FLOCKS, leaders FLOCKS+1..2*FLOCKS,
 * then the two HUD pairs last.
 */

/* Follower colours (256-colour palette). */
#define COLOR_256_MATRIX_GREEN 46 /* pure bright green   */
#define COLOR_256_FIRE_ORANGE 208 /* deep amber-orange   */
/* Dodger blue: a true violet read too dark on black, so we use a mid-blue
 * where the palette is brightest. */
#define COLOR_256_ELECTRIC_BLUE 33

/* Leader colours — each contrasts its flock's follower colour. */
#define COLOR_256_LEADER_YELLOW 226 /* warm yellow  (vs green)  */
#define COLOR_256_LEADER_CYAN 51    /* ice cyan     (vs orange) */
#define COLOR_256_LEADER_WHITE 231  /* bright white (vs blue)   */

static int follower_color_pair(int flock_idx) { return flock_idx + 1; }
static int leader_color_pair(int flock_idx) { return flock_idx + 1 + FLOCKS; }
#define PAIR_HUD (2 * FLOCKS + 1)  /* bright yellow — top status   */
#define PAIR_HINT (2 * FLOCKS + 2) /* bright cyan   — bottom hint  */

/* Foreground colours on the terminal's own background (-1). All shades sit in
 * the bright half of the palette so they stay readable on a dark terminal. */
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

/* 16-colour fallback — closest plain-colour equivalents. */
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

/* ── §4 coords — pixel/cell bridge + World extent ── */

/* Terminal size in cells → simulation size in pixels. */
static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

/*
 * The size of the simulation box, in pixels. Physics needs it for two things:
 * wrapping a boid that leaves one edge back in at the opposite edge, and
 * measuring the shortest distance between two boids on that wrapped world (so
 * boids near opposite edges count as close, not far). Kept separate from the
 * terminal's cell size on purpose — physics only ever thinks in pixels.
 *
 *   width   x-size in pixels (cols * CELL_W); boids stay in [0, width)
 *   height  y-size in pixels (rows * CELL_H); boids stay in [0, height)
 *
 * Both are always > 0 and stay in sync with the terminal size.
 */
typedef struct {
    float width;
    float height;
} World;

/* Build the pixel-space World from the current terminal size. */
static inline World world_from_terminal(int cols, int rows) {
    return (World){
        .width  = (float)pw(cols),
        .height = (float)ph(rows),
    };
}

/*
 * Pixel coordinate → terminal cell. We round half-up by hand instead of using
 * roundf, whose round-to-even rule makes a boid sitting exactly on a cell
 * boundary flicker between two cells each frame.
 */
static inline int px_to_cell_x(float px) {
  return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py) {
  return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5 boid — one agent's state + spawn/clamp/wrap/distance helpers ── */

/*
 * Three small random samplers, one per shape of randomness this file needs.
 * They share the same trick: take a random int and scale it into a float
 * range. Good enough for visual jitter; not for anything security-sensitive.
 */

/* Random float in [0, 1]. */
static inline float rand_uniform_01(void) {
  enum { RAND_DENOM = 1000 };
  return (float)(rand() % (RAND_DENOM + 1)) / (float)RAND_DENOM;
}

/* Random float in [-1, +1]. Used for jitter. */
static inline float rand_uniform_signed(void) {
  enum { RAND_DENOM = 1000 };
  return (float)(rand() % (2 * RAND_DENOM + 1) - RAND_DENOM) / (float)RAND_DENOM;
}

/* Random angle in [0, 2π). Used to pick a random starting heading. */
static inline float rand_uniform_angle_2pi(void) {
  enum { ANGLE_TICKS = 6283 }; /* round(2π × 1000) = 6283.18..  */
  return (float)(rand() % (ANGLE_TICKS + 1)) / 1000.0f;
}

/*
 * One boid: a moving point, the basic unit of every mode here. It carries
 * three things — where it is, how it's moving, and the speed it likes to move
 * at. Each tick, steering forces add into the velocity, then the position
 * follows the velocity. The boid has no mass (forces add straight into
 * velocity) and no stored heading angle (its heading is just the direction of
 * its velocity, worked out when needed).
 *
 *   px, py        position in pixels; always inside the world after wrapping
 *   vx, vy        velocity in pixels/sec; its length is kept in
 *                 [MIN_SPEED, MAX_SPEED] so a boid never stalls or runs away
 *   cruise_speed  the speed this boid prefers, set once at spawn and never
 *                 changed. Followers get BOID_SPEED ±15%; that small spread is
 *                 what gives a flock its organic look — faster boids drift to
 *                 the front, slower ones to the back (Couzin et al. 2002).
 *                 Leaders get exactly LEADER_SPEED.
 */
typedef struct {
  float px, py;       /* position, pixel space (px); torus-wrapped       */
  float vx, vy;       /* velocity, px/s; clamped to [MIN_SPEED,MAX_SPEED]*/
  float cruise_speed; /* desired speed, px/s; ±15 % per follower (depth) */
} Boid;

/*
 * Pick an ASCII arrow showing which way a boid is heading. Each flock uses its
 * own diagonal glyphs so you can tell flocks apart by shape, not just colour:
 * flock 0 uses \ and /, flock 1 uses ~, flock 2 uses +. The eight directions
 * (W, NW, N, ... clockwise) come from the boid's velocity angle.
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

  /* Velocity angle → one of 8 sectors. The half-sector shift centres each
   * arrow on its compass direction rather than on the boundary between two. */
  float heading_radians         = atan2f(vy, vx);
  float heading_shifted_centred = heading_radians + (float)M_PI + SECTOR_HALF_WIDTH;
  int   sector_index            = (int)floorf(heading_shifted_centred / SECTOR_WIDTH) % N_SECTORS;
  return k_chars[flock_idx][sector_index];
}

/* Place a boid at (px, py) moving at `speed` in a random direction. */
static void boid_spawn_at(Boid *b, float px, float py, float speed) {
  b->px = px;
  b->py = py;
  b->cruise_speed = speed;

  /* Pick a random direction by throwing darts at a square and keeping only
   * ones that land inside the unit circle — gives an even spread of angles
   * with no trig calls. The near-origin reject avoids dividing by ~zero. */
  const float UNIT_DISC_MIN_SQ = 0.01f;   /* skip near-origin */
  const float UNIT_DISC_MAX_SQ = 1.0f;
  float dx, dy, len_sq;
  do {
    dx = rand_uniform_signed();
    dy = rand_uniform_signed();
    len_sq = dx * dx + dy * dy;
  } while (len_sq < UNIT_DISC_MIN_SQ || len_sq > UNIT_DISC_MAX_SQ);

  float sample_length = sqrtf(len_sq);
  float direction_x   = dx / sample_length;
  float direction_y   = dy / sample_length;
  b->vx = direction_x * speed;
  b->vy = direction_y * speed;
}

/*
 * Keep a boid's speed between MIN_SPEED and MAX_SPEED, leaving its direction
 * alone. The floor stops it stalling when opposing forces cancel; the ceiling
 * stops it bolting when many separation pushes stack up.
 */
static void boid_clamp_speed(Boid *b) {
  const float SPEED_EPSILON = 0.001f;        /* near-zero magnitude: stall */
  float current_speed = hypotf(b->vx, b->vy);

  /* Fully stalled: no direction left to keep, so just point it along +x. */
  if (current_speed < SPEED_EPSILON) {
    b->vx = MIN_SPEED;
    b->vy = 0.0f;
    return;
  }

  if (current_speed < MIN_SPEED) {
    float dir_x = b->vx / current_speed;
    float dir_y = b->vy / current_speed;
    b->vx = dir_x * MIN_SPEED;
    b->vy = dir_y * MIN_SPEED;
    return;
  }

  if (current_speed > MAX_SPEED) {
    float dir_x = b->vx / current_speed;
    float dir_y = b->vy / current_speed;
    b->vx = dir_x * MAX_SPEED;
    b->vy = dir_y * MAX_SPEED;
  }
}

/*
 * Wrap a boid that has crossed an edge back in at the opposite edge. This way
 * no boid is ever stuck in a corner with neighbours only on one side, so
 * flocking looks the same everywhere on screen.
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
 * Shortest signed distance from a to b on one wrapped axis of length `extent`.
 * On a wrap-around world there are two ways to get from a to b; this returns
 * the shorter one, with a sign for direction. Example (extent 100): a=5, b=95
 * gives -10, not +90 — they're close across the edge, not far the long way.
 */
static float toroidal_delta(float a, float b, float extent) {
  const float HALF_EXTENT = extent * 0.5f;   /* wrap-decision boundary */

  float straight_delta = b - a;
  /* If the direct path is more than half the world, the wrap path is shorter. */
  if (straight_delta >  HALF_EXTENT)  return straight_delta - extent;
  if (straight_delta < -HALF_EXTENT)  return straight_delta + extent;
  return straight_delta;
}

/* ── §6 flock — leader, the five steering rules, per-flock tick + init ── */

/*
 * Which steering rule is running right now. Keys 1-5 set this; the tick reads
 * it to pick a steering function. Each mode is a different model of how local
 * rules produce group motion:
 *
 *   MODE_BOIDS     Reynolds 1987: separation + alignment + cohesion + a gentle
 *                  leader pull. The classic flocking rule.
 *   MODE_CHASE     followers home straight in on the leader (plus separation so
 *                  they don't pile up). Makes comet-tail shapes.
 *   MODE_VICSEK    Vicsek 1995: match neighbours' average heading, then add
 *                  random noise. The noise knob (n/m) tips the flock between
 *                  ordered streaming and scattered motion.
 *   MODE_ORBIT     followers take seats on a spinning ring around the leader.
 *   MODE_PREDATOR  flock 0 hunts the nearest prey; flocks 1-2 flock normally
 *                  but flee any predator that gets close.
 *   MODE_COUNT     how many modes there are (array size / clamp limit).
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
 * One flock: a leader plus its followers, with the bits of state that persist
 * from tick to tick. Three flocks share the screen, all running the same mode
 * but each with its own leader and motion. Everything a steering function
 * needs is reachable from one `Flock *`.
 *
 * The leader is a separate field, not followers[0], because it moves
 * differently: it cruises at a constant speed by walking a heading angle
 * around, rather than being pushed around by forces like the followers.
 *
 *   leader        the flock's leader boid, drawn in the leader colour
 *   followers[]   a fixed pool of boids; only the first `n` are alive (no
 *                 allocation at runtime — +/- just changes `n`)
 *   n             how many followers are currently active (FOLLOWERS_MIN..MAX)
 *   wander_angle  leader's heading in radians. We store an angle (not a
 *                 velocity) because the leader's motion is built from it:
 *                 nudge the angle a little each tick, then point the leader
 *                 that way at LEADER_SPEED. In predator mode this angle snaps
 *                 toward the nearest prey instead.
 *   orbit_phase   orbit mode only: how far the ring has spun, in radians.
 *                 Each follower's seat is at orbit_phase + its share of the
 *                 circle, so spinning this drags every follower around.
 *   color_phase   per-flock identity tag, set once; reserved for a future
 *                 colour-cycling feature and otherwise unused.
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

/* ── leader ── */

/*
 * Aim `angle_out` at the target (tx, ty) along the shortest wrap-around path.
 * Used when the predator locks its heading onto the prey it's chasing.
 */
static void snap_angle_to_bearing(const Boid *from,
                                  float tx, float ty, World world,
                                  float *angle_out) {
  float offset_x = toroidal_delta(from->px, tx, world.width);
  float offset_y = toroidal_delta(from->py, ty, world.height);
  *angle_out = atan2f(offset_y, offset_x);
}

/*
 * Move a leader one step: point it along `angle` at LEADER_SPEED, advance, and
 * wrap. Shared by the wandering leader and the hunting predator leader — both
 * build their velocity from an angle rather than from forces.
 */
static void integrate_leader_at_constant_speed(Boid *leader, float angle,
                                                float dt, World world) {
  float heading_dir_x = cosf(angle);
  float heading_dir_y = sinf(angle);

  leader->vx = heading_dir_x * LEADER_SPEED;
  leader->vy = heading_dir_y * LEADER_SPEED;

  leader->px += leader->vx * dt;
  leader->py += leader->vy * dt;
  boid_wrap(leader, world);
}

/*
 * Advance a wandering leader one step. Its heading drifts by a small random
 * amount each tick — enough to curve gently, not enough to make sharp turns
 * its followers can't track. Speed stays constant, so it never stalls.
 */
static void leader_tick(Flock *f, float dt, World world) {
  f->wander_angle += rand_uniform_signed() * WANDER_JITTER;
  integrate_leader_at_constant_speed(&f->leader, f->wander_angle, dt, world);
}

/* ── steering primitives ── */
/*
 * Shared force-building helpers. Each one ADDS its push into the running
 * (*steer_x, *steer_y) total; a steering function calls several, then clamps
 * the result. clamp2d caps the turn rate, add_separation_force pushes away
 * from close neighbours, seek_force_toward steers toward a target point.
 */

/*
 * Shrink the vector (fx, fy) so it's no longer than max_mag, keeping its
 * direction. Skips the square root entirely when it's already short enough.
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
 * Push away from any neighbour inside the personal-space bubble. The closer the
 * neighbour, the harder the push. Used by chase and orbit modes (boids mode
 * does its own separation inside the shared neighbour scan).
 */
static void add_separation_force(const Boid *self, const Boid *others,
                                 int n, int self_idx, World world,
                                 float *steer_x, float *steer_y) {
  const float DIST_EPSILON = 0.001f;       /* zero-distance guard */

  for (int j = 0; j < n; j++) {
    if (j == self_idx) continue;

    float offset_x = toroidal_delta(self->px, others[j].px, world.width);
    float offset_y = toroidal_delta(self->py, others[j].py, world.height);
    float distance = hypotf(offset_x, offset_y);

    bool inside_personal_space = distance < SEPARATION_RADIUS;
    bool well_separated        = distance > DIST_EPSILON;
    if (!inside_personal_space || !well_separated) continue;

    /* 1 when on top of each other, fading to 0 at the bubble's edge. */
    float personal_space_intrusion = (SEPARATION_RADIUS - distance) / SEPARATION_RADIUS;

    /* Point away from the neighbour (offset points toward it, so negate). */
    float push_away_x = -offset_x / distance;
    float push_away_y = -offset_y / distance;

    *steer_x += push_away_x * personal_space_intrusion * MAX_STEER;
    *steer_y += push_away_y * personal_space_intrusion * MAX_STEER;
  }
}

/*
 * Steer toward a target point. The push is "the velocity I'd want minus the
 * one I have" — head straight at the target at my cruise speed. Used by chase
 * and orbit once they've picked a target. Caller weights/clamps the result.
 */
static void seek_force_toward(const Boid *b, float target_px, float target_py,
                               World world,
                               float *steer_x, float *steer_y) {
  const float OFFSET_EPSILON = 0.001f;

  float offset_x          = toroidal_delta(b->px, target_px, world.width);
  float offset_y          = toroidal_delta(b->py, target_py, world.height);
  float distance_to_target = hypotf(offset_x, offset_y);
  if (distance_to_target <= OFFSET_EPSILON) return;

  float toward_target_x = offset_x / distance_to_target;
  float toward_target_y = offset_y / distance_to_target;

  float desired_vx = toward_target_x * b->cruise_speed;
  float desired_vy = toward_target_y * b->cruise_speed;

  *steer_x += desired_vx - b->vx;
  *steer_y += desired_vy - b->vy;
}

/* ── boids steering (MODE_BOIDS) ── */

/*
 * Running totals for the three boids rules, all gathered in one sweep over a
 * boid's neighbours so we don't loop three times. The three accumulate_*
 * helpers below turn these sums into steering pushes.
 *
 *   sep_x, sep_y      summed push-away directions from close neighbours
 *   ali_vx, ali_vy    summed neighbour velocities (for the average heading)
 *   coh_dx, coh_dy    summed offsets to neighbours (for the group's centre)
 *   neighbour_count   how many neighbours were in sight
 *   close_count       how many were inside the separation bubble
 */
typedef struct {
  float sep_x, sep_y;
  float ali_vx, ali_vy;
  float coh_dx, coh_dy;
  int   neighbour_count;
  int   close_count;
} NeighbourSums;

/*
 * One sweep over follower[idx]'s neighbours, filling in the running totals for
 * all three boids rules at once. Cohesion sums offsets to each neighbour
 * rather than their raw positions — see accumulate_cohesion_force for why that
 * matters on a wrap-around world.
 */
static NeighbourSums scan_perception_neighbours(const Flock *f, int idx,
                                                 World world) {
  const float DIST_EPSILON = 0.001f;     /* zero-distance guard (co-located boids) */
  const Boid *b = &f->followers[idx];
  NeighbourSums s = {0};

  for (int j = 0; j < f->n; j++) {
    if (j == idx) continue;
    const Boid *nb = &f->followers[j];

    float offset_x   = toroidal_delta(b->px, nb->px, world.width);
    float offset_y   = toroidal_delta(b->py, nb->py, world.height);
    float distance   = hypotf(offset_x, offset_y);

    /* Skip neighbours out of sight or sitting right on top of us. */
    bool inside_perception = distance < PERCEPTION_RADIUS;
    bool well_separated    = distance >= DIST_EPSILON;
    if (!inside_perception || !well_separated) continue;

    /* Alignment and cohesion: every visible neighbour counts. */
    s.ali_vx += nb->vx;
    s.ali_vy += nb->vy;
    s.coh_dx += offset_x;
    s.coh_dy += offset_y;
    s.neighbour_count++;

    /* Separation: only neighbours inside the bubble, closer ones harder. */
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

/* Turn the summed push-away directions into a separation steering force. */
static void accumulate_separation_force(const NeighbourSums *s,
                                        float *steer_x, float *steer_y) {
  const float MAG_EPSILON = 0.001f;
  if (s->close_count == 0) return;

  float push_magnitude = hypotf(s->sep_x, s->sep_y);
  if (push_magnitude <= MAG_EPSILON) return;
  float push_dir_x = s->sep_x / push_magnitude;
  float push_dir_y = s->sep_y / push_magnitude;

  *steer_x += push_dir_x * W_SEPARATION * MAX_STEER;
  *steer_y += push_dir_y * W_SEPARATION * MAX_STEER;
}

/*
 * Steer toward the neighbours' average heading (Reynolds rule 2). Aims at that
 * heading at THIS boid's own cruise speed, so faster boids pull ahead — that's
 * what gives a flock its layered, organic look.
 */
static void accumulate_alignment_force(const NeighbourSums *s, const Boid *b,
                                       float *steer_x, float *steer_y) {
  const float MAG_EPSILON = 0.001f;
  if (s->neighbour_count == 0) return;

  float mean_heading_x = s->ali_vx / (float)s->neighbour_count;
  float mean_heading_y = s->ali_vy / (float)s->neighbour_count;
  float mean_heading_magnitude = hypotf(mean_heading_x, mean_heading_y);
  if (mean_heading_magnitude <= MAG_EPSILON) return;

  float aligned_dir_x = mean_heading_x / mean_heading_magnitude;
  float aligned_dir_y = mean_heading_y / mean_heading_magnitude;

  float desired_vx = aligned_dir_x * b->cruise_speed;
  float desired_vy = aligned_dir_y * b->cruise_speed;

  *steer_x += (desired_vx - b->vx) * W_ALIGNMENT;
  *steer_y += (desired_vy - b->vy) * W_ALIGNMENT;
}

/*
 * Steer toward the middle of the nearby group (Reynolds rule 3). We average the
 * OFFSETS to neighbours, not their raw positions: on a wrap-around world, two
 * boids near opposite edges average to a point on the wrong side of the screen,
 * whereas averaging offsets relative to this boid points the right way.
 */
static void accumulate_cohesion_force(const NeighbourSums *s, const Boid *b,
                                      float *steer_x, float *steer_y) {
  const float MAG_EPSILON = 0.001f;
  if (s->neighbour_count == 0) return;

  float mean_offset_x = s->coh_dx / (float)s->neighbour_count;
  float mean_offset_y = s->coh_dy / (float)s->neighbour_count;
  float distance_to_centre_of_mass = hypotf(mean_offset_x, mean_offset_y);
  if (distance_to_centre_of_mass <= MAG_EPSILON) return;

  float toward_centre_x = mean_offset_x / distance_to_centre_of_mass;
  float toward_centre_y = mean_offset_y / distance_to_centre_of_mass;

  float desired_vx = toward_centre_x * b->cruise_speed;
  float desired_vy = toward_centre_y * b->cruise_speed;

  *steer_x += (desired_vx - b->vx) * W_COHESION;
  *steer_y += (desired_vy - b->vy) * W_COHESION;
}

/*
 * A gentle tug toward the flock's own leader so the group doesn't drift away.
 * The weight is small so it never overpowers the three boids rules.
 */
static void accumulate_leader_pull(const Boid *b, const Boid *leader,
                                   World world,
                                   float *steer_x, float *steer_y) {
  const float MAG_EPSILON = 0.001f;

  float offset_x = toroidal_delta(b->px, leader->px, world.width);
  float offset_y = toroidal_delta(b->py, leader->py, world.height);
  float distance_to_leader = hypotf(offset_x, offset_y);
  if (distance_to_leader <= MAG_EPSILON) return;

  float toward_leader_x = offset_x / distance_to_leader;
  float toward_leader_y = offset_y / distance_to_leader;

  *steer_x += toward_leader_x * W_LEADER_PULL * MAX_STEER;
  *steer_y += toward_leader_y * W_LEADER_PULL * MAX_STEER;
}

/*
 * The classic flocking rule: work out one follower's new velocity from
 * separation + alignment + cohesion + a leader pull, capped so turns curve.
 * Read-only on the flock — the caller computes everyone's new velocity from
 * the same snapshot before applying any, so no boid reacts to a half-moved one.
 */
static void boids_steer(const Flock *f, int idx, World world,
                        float *out_vx, float *out_vy) {
  const Boid *b = &f->followers[idx];

  NeighbourSums sums = scan_perception_neighbours(f, idx, world);

  float steer_x = 0.0f, steer_y = 0.0f;
  accumulate_separation_force(&sums,           &steer_x, &steer_y);
  accumulate_alignment_force (&sums, b,        &steer_x, &steer_y);
  accumulate_cohesion_force  (&sums, b,        &steer_x, &steer_y);
  accumulate_leader_pull     (b, &f->leader, world, &steer_x, &steer_y);

  clamp2d(&steer_x, &steer_y, MAX_STEER);
  *out_vx = b->vx + steer_x;
  *out_vy = b->vy + steer_y;
}

/* ── chase steering (MODE_CHASE) ── */

/*
 * Followers ignore each other and home straight in on the leader, with just
 * enough close-range separation that they don't pile up. Makes comet-tail
 * shapes.
 */
static void chase_steer(const Flock *f, int idx, World world,
                        float *out_vx, float *out_vy) {
  const Boid *b = &f->followers[idx];
  float steer_x = 0.0f, steer_y = 0.0f;

  seek_force_toward(b, f->leader.px, f->leader.py, world, &steer_x, &steer_y);
  add_separation_force(b, f->followers, f->n, idx, world, &steer_x, &steer_y);

  clamp2d(&steer_x, &steer_y, MAX_STEER);
  *out_vx = b->vx + steer_x;
  *out_vy = b->vy + steer_y;
}

/* ── Vicsek steering (MODE_VICSEK) ── */

/*
 * Add up the velocities of everyone this boid can see, so the caller can find
 * the average heading. It includes the boid itself (the Vicsek model averages
 * itself in) and the leader (which anchors the flock when noise is low).
 */
static void sum_velocities_in_perception(const Flock *f, int idx, World world,
                                          float *out_sum_vx,
                                          float *out_sum_vy,
                                          int   *out_count) {
  const Boid *b = &f->followers[idx];
  float sum_vx = b->vx;
  float sum_vy = b->vy;
  int   count  = 1;

  /* Leader: anchors the flock direction at low noise. */
  {
    float dx = toroidal_delta(b->px, f->leader.px, world.width);
    float dy = toroidal_delta(b->py, f->leader.py, world.height);
    if (hypotf(dx, dy) < PERCEPTION_RADIUS) {
      sum_vx += f->leader.vx;
      sum_vy += f->leader.vy;
      count++;
    }
  }

  /* Followers within sight range. */
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

/*
 * Vicsek mode: turn this boid to match the average heading of everyone it sees,
 * then jitter it by a random angle. The speed stays the boid's own cruise
 * speed; only the direction changes. Turning the noise up (n/m) tips the whole
 * flock from streaming together to scattering.
 */
static void vicsek_steer(const Flock *f, int idx, World world,
                         float noise_scale, float *out_vx, float *out_vy) {
  const Boid *b = &f->followers[idx];

  float sum_vx, sum_vy;
  int   n_velocity_contributors;
  sum_velocities_in_perception(f, idx, world,
                                &sum_vx, &sum_vy, &n_velocity_contributors);

  /* Average velocity → its heading angle. */
  float mean_vx          = sum_vx / (float)n_velocity_contributors;
  float mean_vy          = sum_vy / (float)n_velocity_contributors;
  float mean_heading_rad = atan2f(mean_vy, mean_vx);

  /* Nudge the heading by a random angle within ±noise_scale. */
  float noise_eta        = rand_uniform_signed() * noise_scale;
  float new_heading_rad  = mean_heading_rad + noise_eta;

  *out_vx = cosf(new_heading_rad) * b->cruise_speed;
  *out_vy = sinf(new_heading_rad) * b->cruise_speed;
}

/* ── orbit steering (MODE_ORBIT) ── */

/*
 * Where follower `idx` should be on the spinning ring right now: its evenly
 * spaced seat angle, offset by how far the ring has rotated, placed
 * ORBIT_RADIUS out from the leader.
 */
static void orbit_target_for_slot(const Boid *leader, float orbit_phase,
                                  int idx, int n,
                                  float *target_px, float *target_py) {
  const float FULL_TURN_RADIANS = 2.0f * (float)M_PI;

  float slot_offset_angle = FULL_TURN_RADIANS * (float)idx / (float)n;
  float slot_angle        = orbit_phase + slot_offset_angle;

  *target_px = leader->px + ORBIT_RADIUS * cosf(slot_angle);
  *target_py = leader->py + ORBIT_RADIUS * sinf(slot_angle);
}

/*
 * Orbit mode: each follower chases its own seat on the spinning ring, with
 * separation so they don't bunch up while closing in.
 */
static void orbit_steer(const Flock *f, int idx, World world,
                        float *out_vx, float *out_vy) {
  const Boid *b = &f->followers[idx];

  float target_px, target_py;
  orbit_target_for_slot(&f->leader, f->orbit_phase, idx, f->n,
                        &target_px, &target_py);

  float steer_x = 0.0f, steer_y = 0.0f;
  seek_force_toward(b, target_px, target_py, world, &steer_x, &steer_y);
  add_separation_force(b, f->followers, f->n, idx, world, &steer_x, &steer_y);

  clamp2d(&steer_x, &steer_y, MAX_STEER);
  *out_vx = b->vx + steer_x;
  *out_vy = b->vy + steer_y;
}

/* ── flock tick ── */

/*
 * Write each follower's already-computed new velocity, then clamp its speed,
 * move it, and wrap it. This is the "apply" half of the two-pass update, shared
 * by the normal tick and the predator-prey tick.
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

/* Pick the steering function for the active mode and run it for one follower. */
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

/*
 * Advance one flock by dt seconds. Move the leader, spin the orbit ring if
 * active, then update followers in two passes: first work out everyone's new
 * velocity from the current positions, then apply them all at once. The two
 * passes stop boids from reacting to neighbours that have already moved this
 * tick, which would otherwise make the flock creep along in array order.
 */
static void flock_tick(Flock *f, FlockMode mode, float vicsek_noise, float dt,
                       World world) {
  leader_tick(f, dt, world);

  if (mode == MODE_ORBIT)
    f->orbit_phase += ORBIT_SPEED * dt;

  /* Pass 1: compute new velocities from the current (unchanged) positions. */
  float new_vx[FOLLOWERS_MAX];
  float new_vy[FOLLOWERS_MAX];
  for (int i = 0; i < f->n; i++)
    compute_steer_for_mode(f, i, mode, vicsek_noise, world,
                            &new_vx[i], &new_vy[i]);

  /* Pass 2: apply them. */
  apply_followers_step(f, new_vx, new_vy, dt, world);
}

/* ── flock init ── */

/* How far followers are scattered around the leader at spawn time. */
#define SPAWN_SCATTER 70.0f

/*
 * Each flock's starting spot, as fractions of the world: flock 0 top-left,
 * flock 1 top-right, flock 2 bottom-middle. Spreading them out means the first
 * few seconds show three separate groups drifting together — livelier than
 * starting everyone in a pile at the centre.
 */
static void starting_quadrant_origin(int flock_idx, World world,
                                     float *out_ox, float *out_oy) {
  static const float QUADRANT_FRAC_X[FLOCKS] = {0.25f, 0.75f, 0.50f};
  static const float QUADRANT_FRAC_Y[FLOCKS] = {0.25f, 0.25f, 0.75f};
  *out_ox = world.width  * QUADRANT_FRAC_X[flock_idx];
  *out_oy = world.height * QUADRANT_FRAC_Y[flock_idx];
}

/* Random spawn point near (ox, oy), kept inside the world. */
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
 * A random cruise-speed multiplier in [0.85, 1.15] — the ±15% spread that makes
 * faster boids drift to the front and slower ones to the back of the flock.
 */
static float pick_cruise_speed_variation(void) {
  const float SPEED_VARIATION_MIN  = 0.85f;
  const float SPEED_VARIATION_SPAN = 0.30f;   /* span of [0.85, 1.15] */
  return SPEED_VARIATION_MIN + SPEED_VARIATION_SPAN * rand_uniform_01();
}

static void flock_init(Flock *f, int flock_idx, int n_followers, World world) {
  f->n            = n_followers;
  f->color_phase  = (float)flock_idx;
  f->wander_angle = rand_uniform_angle_2pi();

  /* Leader sits at this flock's starting spot. */
  float ox, oy;
  starting_quadrant_origin(flock_idx, world, &ox, &oy);
  boid_spawn_at(&f->leader, ox, oy, LEADER_SPEED);

  /* Followers scatter around it, each with its own slightly different speed. */
  for (int i = 0; i < f->n; i++) {
    float sx, sy;
    pick_scattered_spawn_pos(ox, oy, world, &sx, &sy);
    float variation = pick_cruise_speed_variation();
    boid_spawn_at(&f->followers[i], sx, sy, BOID_SPEED * variation);
  }
}

/* ── §7 scene — owns the sim; predator-prey tick + the renderer ── */

/*
 * The knobs the user can change while it runs. Bundled so the input handler,
 * the tick, and the HUD all share one place.
 *
 *   mode          which steering rule is active (keys 1-5)
 *   paused        SPACE toggles; when true the sim freezes but still draws
 *   vicsek_noise  Vicsek noise amount in radians; n/m turn it down/up. Only
 *                 matters in Vicsek mode. Kept inside VICSEK_NOISE_MIN..MAX.
 */
typedef struct {
  FlockMode mode;          /* active steering algorithm; keys 1..5      */
  bool      paused;        /* SPACE toggles; scene_tick early-returns   */
  float     vicsek_noise;  /* Vicsek noise in radians; n/m adjust it    */
} SimControls;

/*
 * All the simulation state for one run, in one place: the three flocks, the
 * user knobs, and the world size. Everything that lives across ticks hangs off
 * a single Scene pointer. (Terminal size and signal flags live elsewhere —
 * Screen in §8 and App in §9.) In predator mode, flock 0 is the predator and
 * flocks 1-2 are prey; in every other mode the three are alike apart from
 * colour and starting spot.
 */
typedef struct {
  Flock       flocks[FLOCKS];  /* index 0 is the predator in predator mode  */
  SimControls sim;             /* mode + paused + vicsek_noise              */
  World       world;           /* pixel-space size, used for wrap/distance  */
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);

  s->sim.mode         = MODE_BOIDS;
  s->sim.paused       = false;
  s->sim.vicsek_noise = VICSEK_NOISE_DEFAULT;

  s->world = world_from_terminal(cols, rows);

  for (int i = 0; i < FLOCKS; i++)
    flock_init(&s->flocks[i], i, FOLLOWERS_DEFAULT, s->world);
}

/* ── predator-prey tick (MODE_PREDATOR) ── */

/*
 * Find the prey boid closest to the hunter, but only within sight range. Looks
 * at every prey flock's leader and followers. Returns false when nothing is in
 * range, so the caller can fall back to normal wandering.
 */
static bool predator_find_nearest_prey(float hunter_px, float hunter_py,
                                       const Flock *prey_flocks, int n_prey,
                                       World world,
                                       float *target_px, float *target_py) {
  /* Start just past sight range so the first in-range prey always wins, and
   * "still at this value" cleanly means "found nothing". */
  const float OUT_OF_SIGHT_SENTINEL = PREDATOR_CHASE_RADIUS + 1.0f;
  float nearest_distance = OUT_OF_SIGHT_SENTINEL;
  bool  found            = false;

  for (int fi = 0; fi < n_prey; fi++) {
    /* Each prey flock's leader plus its active followers. */
    const Boid *candidates[1 + FOLLOWERS_MAX];
    int n_candidates = 0;
    candidates[n_candidates++] = &prey_flocks[fi].leader;
    for (int i = 0; i < prey_flocks[fi].n; i++)
      candidates[n_candidates++] = &prey_flocks[fi].followers[i];

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

/*
 * Advance the predator's leader. If any prey is in sight, lock the heading onto
 * the nearest one; otherwise wander like a normal leader. Either way it moves
 * at the constant leader speed.
 */
static void predator_leader_tick(Flock *predator, const Flock *prey_flocks,
                                 int n_prey, float dt, World world) {
  float target_px = 0.0f, target_py = 0.0f;
  bool  has_target = predator_find_nearest_prey(predator->leader.px,
                                                predator->leader.py,
                                                prey_flocks, n_prey, world,
                                                &target_px, &target_py);

  if (has_target) {
    snap_angle_to_bearing(&predator->leader, target_px, target_py, world,
                          &predator->wander_angle);
  } else {
    predator->wander_angle += rand_uniform_signed() * WANDER_JITTER;
  }

  integrate_leader_at_constant_speed(&predator->leader,
                                      predator->wander_angle, dt, world);
}

/*
 * Add a push directly away from one threat, but only if it's inside the prey's
 * panic range. Unweighted — the caller scales it. Used for both the predator's
 * leader and each of its followers.
 */
static void accumulate_flee_from_one(const Boid *self, const Boid *threat,
                                     World world,
                                     float *flee_x, float *flee_y) {
  const float DIST_EPSILON = 0.001f;

  float offset_x = toroidal_delta(self->px, threat->px, world.width);
  float offset_y = toroidal_delta(self->py, threat->py, world.height);
  float distance_to_threat = hypotf(offset_x, offset_y);

  bool inside_panic_zone = distance_to_threat < PREY_FLEE_RADIUS;
  bool well_separated    = distance_to_threat > DIST_EPSILON;
  if (!inside_panic_zone || !well_separated) return;

  /* Point away from the threat (offset points toward it, so negate). */
  float away_from_threat_x = -offset_x / distance_to_threat;
  float away_from_threat_y = -offset_y / distance_to_threat;

  *flee_x += away_from_threat_x;
  *flee_y += away_from_threat_y;
}

/* Sum the flee pushes from every predator boid (leader + followers) in range. */
static void accumulate_flee_from_predator(const Boid *self,
                                          const Flock *predator,
                                          World world,
                                          float *flee_x, float *flee_y) {
  accumulate_flee_from_one(self, &predator->leader, world, flee_x, flee_y);
  for (int i = 0; i < predator->n; i++)
    accumulate_flee_from_one(self, &predator->followers[i], world,
                              flee_x, flee_y);
}

/*
 * Prey steering: normal flocking, plus a strong shove away from any predator
 * that gets close. The flee push is much louder than cohesion, so frightened
 * prey actually break out of the flock instead of just shuffling inside it.
 */
static void prey_boids_steer(const Flock *f, int idx, const Flock *predator,
                             World world, float *out_vx, float *out_vy) {
  const float FLEE_EPSILON = 0.001f;

  boids_steer(f, idx, world, out_vx, out_vy);

  const Boid *b = &f->followers[idx];
  float flee_sum_x = 0.0f, flee_sum_y = 0.0f;
  accumulate_flee_from_predator(b, predator, world, &flee_sum_x, &flee_sum_y);

  float flee_magnitude = hypotf(flee_sum_x, flee_sum_y);
  if (flee_magnitude > FLEE_EPSILON) {
    float panic_dir_x = flee_sum_x / flee_magnitude;
    float panic_dir_y = flee_sum_y / flee_magnitude;
    *out_vx += panic_dir_x * W_FLEE * MAX_STEER;
    *out_vy += panic_dir_y * W_FLEE * MAX_STEER;
  }
}

/*
 * Advance the predator pack one step: the leader hunts the nearest prey, the
 * followers chase their own leader. Prey are read-only here — they update
 * afterwards.
 */
static void tick_predator_pack(Flock *predator, const Flock *prey_flocks,
                                int n_prey, float dt, World world) {
  predator_leader_tick(predator, prey_flocks, n_prey, dt, world);

  float new_vx[FOLLOWERS_MAX], new_vy[FOLLOWERS_MAX];
  for (int i = 0; i < predator->n; i++)
    chase_steer(predator, i, world, &new_vx[i], &new_vy[i]);

  apply_followers_step(predator, new_vx, new_vy, dt, world);
}

/*
 * Advance one prey flock one step: the leader wanders normally, the followers
 * flock and flee any nearby predator.
 */
static void tick_one_prey_flock(Flock *prey, const Flock *predator,
                                 float dt, World world) {
  leader_tick(prey, dt, world);

  float new_vx[FOLLOWERS_MAX], new_vy[FOLLOWERS_MAX];
  for (int i = 0; i < prey->n; i++)
    prey_boids_steer(prey, i, predator, world, &new_vx[i], &new_vy[i]);

  apply_followers_step(prey, new_vx, new_vy, dt, world);
}

/* One predator-prey step: move the hunter pack, then each prey flock. */
static void scene_tick_predator(Scene *s, float dt) {
  World world = s->world;
  Flock *predator = &s->flocks[0];

  tick_predator_pack(predator, &s->flocks[1], FLOCKS - 1, dt, world);

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
 * Followers close to their leader render bold (brighter), the rest normal.
 * Uses wrap-aware distance so a follower near the leader across a screen edge
 * still counts as close.
 */
static int follower_brightness(const Boid *follower, const Boid *leader,
                               World world) {
  float dx = toroidal_delta(follower->px, leader->px, world.width);
  float dy = toroidal_delta(follower->py, leader->py, world.height);
  float ratio = hypotf(dx, dy) / PERCEPTION_RADIUS;
  return (ratio < 0.35f) ? A_BOLD : A_NORMAL;
}

/*
 * Timing info for drawing one frame. Physics moves in fixed steps, but frames
 * land at moments in between, so drawing at the last step's position would look
 * jerky. We nudge each boid forward a fraction of a step before drawing so it
 * lands where it really is "now".
 *
 *   alpha   how far we are into the next physics step, 0 up to (not incl.) 1
 *   dt_sec  length of one physics step in seconds
 *
 * (Fiedler, "Fix Your Timestep!" describes this fixed-step-plus-interpolation
 * pattern.)
 */
typedef struct {
  float alpha;   /* fraction into the next physics step, [0,1)            */
  float dt_sec;  /* one physics step in seconds                          */
} RenderTiming;

/*
 * Nudge a boid forward from its last-step position by the leftover fraction of
 * a step, using its current velocity, so it draws where it actually is now
 * rather than where the last physics step left it.
 */
static void extrapolate_to_render_pos(const Boid *b, RenderTiming rt,
                                       float *out_px, float *out_py) {
  float sub_tick_seconds = rt.alpha * rt.dt_sec;
  *out_px = b->px + b->vx * sub_tick_seconds;
  *out_py = b->py + b->vy * sub_tick_seconds;
}

/*
 * Keep a drawn position inside the world box. The forward-nudge can push a boid
 * a hair past the edge, so this is a small safety clamp, not a common case.
 */
static void clamp_to_world_box(float *px, float *py, World world) {
  if (*px < 0)              *px = 0;
  if (*px > world.width)    *px = world.width;
  if (*py < 0)              *py = 0;
  if (*py > world.height)   *py = world.height;
}

/*
 * Stamp one ASCII char at a terminal cell, but only if it's on screen. The
 * bounds check is what stops an off-screen boid from writing out of bounds; the
 * cast keeps chars above 127 from being misread as negative.
 */
static void paint_glyph_at_cell(WINDOW *w, int cx, int cy,
                                 char ch, int color_attr,
                                 int cols, int rows) {
  if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
  wattron (w, color_attr);
  mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
  wattroff(w, color_attr);
}

/* Draw one boid: nudge to its now-position, convert pixels to a cell, stamp it. */
static void draw_boid(WINDOW *w, const Boid *b, char ch, int color_attr,
                      int cols, int rows, World world, RenderTiming rt) {
  float draw_px, draw_py;
  extrapolate_to_render_pos(b, rt, &draw_px, &draw_py);

  clamp_to_world_box(&draw_px, &draw_py, world);
  int cx = px_to_cell_x(draw_px);
  int cy = px_to_cell_y(draw_py);

  paint_glyph_at_cell(w, cx, cy, ch, color_attr, cols, rows);
}

/* Draw every flock. Followers first, leader last so a leader is never hidden. */
static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows,
                       RenderTiming rt) {
  World world = s->world;

  for (int fi = 0; fi < FLOCKS; fi++) {
    const Flock *f = &s->flocks[fi];

    /* Followers: flock colour, heading arrow, brighter when near the leader. */
    int follower_pair = follower_color_pair(fi);
    for (int i = 0; i < f->n; i++) {
      const Boid *b = &f->followers[i];
      char ch = velocity_dir_char(b->vx, b->vy, fi);
      int attr = COLOR_PAIR(follower_pair) |
                 follower_brightness(b, &f->leader, world);
      draw_boid(w, b, ch, attr, cols, rows, world, rt);
    }

    /* Leader: its own colour, heading arrow, always bold. */
    int leader_pair = leader_color_pair(fi);
    char leader_ch = velocity_dir_char(f->leader.vx, f->leader.vy, fi);
    draw_boid(w, &f->leader, leader_ch, COLOR_PAIR(leader_pair) | A_BOLD,
              cols, rows, world, rt);
  }
}

/* ── §8 screen — ncurses display layer + HUD ── */

/*
 * The terminal's size in character cells, and the home of the ncurses
 * setup/teardown. This is the screen side of things; the World struct (§4) is
 * the pixel side. They're kept separate so physics never deals in cells and the
 * renderer only crosses over through the one px-to-cell conversion. Both
 * cols and rows are > 0 and stay in step with the World size.
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

/* Total boids on screen: each flock's leader plus its active followers. */
static int count_total_boids(const Scene *sc) {
  int total = 0;
  for (int i = 0; i < FLOCKS; i++)
    total += sc->flocks[i].n + 1;  /* +1 for the leader */
  return total;
}

/*
 * Build the top status line. Vicsek mode also shows the live noise value so you
 * can watch it change as you press n / m; other modes leave it off.
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

/* Print one bold HUD line at (row, col). */
static void hud_paint_text(int row, int col, int pair, const char *text) {
  attron (COLOR_PAIR(pair) | A_BOLD);
  mvprintw(row, col, "%s", text);
  attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Top status line: fps, mode, noise, count, run-state, right-aligned. */
static void draw_hud_status(const Screen *s, const Scene *sc, double fps) {
  enum { HUD_TOP_ROW = 0, HUD_BUFLEN = 160 };

  int  total_boids = count_total_boids(sc);
  char buf[HUD_BUFLEN];
  format_hud_status(sc, fps, total_boids, buf, sizeof buf);

  int right_col = s->cols - (int)strlen(buf);
  if (right_col < 0) right_col = 0;
  hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, buf);
}

/* Bottom line: the key hints. */
static void draw_hud_hint(const Screen *s) {
  static const char *KEY_HINT =
      " q:quit  spc:pause  1:boids 2:chase 3:vicsek 4:orbit 5:predator  "
      "n/m:noise  +/-:boids  r:reset ";
  hud_paint_text(s->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

/* Build one frame: clear, draw the flocks, then the HUD on top. */
static void screen_draw(Screen *s, Scene *sc, double fps, RenderTiming rt) {
  erase();
  scene_draw(sc, stdscr, s->cols, s->rows, rt);
  draw_hud_status(s, sc, fps);
  draw_hud_hint  (s);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §9 app — fps counter, App container, signals, input, main loop ── */

/*
 * A smoothed frame-rate readout. A raw per-frame number jumps around too much
 * to read, so this counts frames over a fixed window (about half a second) and
 * publishes one steady figure each time the window fills.
 *
 *   frame_count   frames seen so far in this window
 *   window_ns     time accumulated in this window, in nanoseconds
 *   display       the last published fps; the HUD reads this
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
 * Everything the program owns, in one place. There's a single instance, g_app,
 * which is the program's only global — it has to be, because signal handlers
 * can't be handed a pointer and need to reach the two flags below.
 *
 *   scene        the simulation
 *   screen       terminal size + ncurses setup
 *   fps          the frame-rate readout
 *   running      0 tells the main loop to stop; SIGINT/SIGTERM or 'q' clear it
 *   need_resize  set to 1 by a window-resize signal; the loop handles it next
 *                pass. Both flags are volatile sig_atomic_t so a signal handler
 *                can set them safely and the loop always re-reads them.
 */
typedef struct {
  Scene      scene;        /* simulation state                              */
  Screen     screen;       /* terminal cell extent + ncurses lifecycle      */
  FpsCounter fps;          /* rolling-window fps for HUD                    */
  volatile sig_atomic_t running;     /* 0 = exit; SIGINT/TERM clear it      */
  volatile sig_atomic_t need_resize; /* 1 = SIGWINCH pending                 */
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

/* Pull one boid back inside the world if it's past the edge (after a shrink). */
static void clamp_boid_inside_world(Boid *b, World world) {
  if (b->px >= world.width)   b->px = world.width  - 1.0f;
  if (b->py >= world.height)  b->py = world.height - 1.0f;
}

/*
 * After the terminal shrinks, pull every boid that's now off the edge back on
 * screen right away, instead of waiting for it to wrap around naturally.
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

/* Handle a terminal resize: redo ncurses, refresh the world size, pull boids in. */
static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  app->scene.world = world_from_terminal(app->screen.cols, app->screen.rows);
  clamp_all_boids_to_world(&app->scene);
  app->need_resize = 0;
}

/* Nudge the Vicsek noise up or down, kept within its allowed range. */
static void adjust_vicsek_noise(SimControls *sim, float delta) {
  sim->vicsek_noise += delta;
  if (sim->vicsek_noise < VICSEK_NOISE_MIN) sim->vicsek_noise = VICSEK_NOISE_MIN;
  if (sim->vicsek_noise > VICSEK_NOISE_MAX) sim->vicsek_noise = VICSEK_NOISE_MAX;
}

/* Add/remove followers on every flock at once, kept within the allowed range. */
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

  /* keys 1-5 pick the steering mode */
  case '1':  s->sim.mode = MODE_BOIDS;           break;
  case '2':  s->sim.mode = MODE_CHASE;           break;
  case '3':  s->sim.mode = MODE_VICSEK;          break;
  case '4':  s->sim.mode = MODE_ORBIT;           break;
  case '5':  s->sim.mode = MODE_PREDATOR;        break;

  /* Vicsek noise down / up (only does anything in Vicsek mode) */
  case 'n': case 'N':
    adjust_vicsek_noise(&s->sim, -VICSEK_NOISE_STEP); break;
  case 'm': case 'M':
    adjust_vicsek_noise(&s->sim, +VICSEK_NOISE_STEP); break;

  /* add / remove a follower from every flock */
  case '=': case '+':
    adjust_follower_count_all_flocks(s, +1); break;
  case '-':
    adjust_follower_count_all_flocks(s, -1); break;

  /* reset everything to its starting state */
  case 'r': case 'R':
    scene_init(s, app->screen.cols, app->screen.rows);
    break;

  default: break;
  }
  return true;
}

int main(void) {
  /* Seed the RNG, restore the terminal on exit, catch quit/resize signals. */
  srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);
  fps_counter_init(&app->fps);

  const int64_t DT_CAP_NS       = 100 * NS_PER_MS;          /* avalanche guard */
  const int64_t FRAME_BUDGET_NS = NS_PER_SEC / RENDER_FPS;  /* render cadence */
  const int64_t TICK_LEN_NS     = TICK_NS(SIM_FPS);
  const float   TICK_LEN_SEC    = (float)TICK_LEN_NS / (float)NS_PER_SEC;

  int64_t frame_time = clock_ns();
  int64_t sim_accum  = 0;

  while (app->running) {
    int64_t frame_start = clock_ns();

    /* Handle a pending resize before anything else this frame. */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* Time since last frame. Capped so a long pause (e.g. laptop sleep)
     * doesn't make the sim try to catch up with a flood of steps. */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > DT_CAP_NS) dt = DT_CAP_NS;

    /* Run the physics in fixed steps until it's caught up to real time. */
    sim_accum += dt;
    while (sim_accum >= TICK_LEN_NS) {
      scene_tick(&app->scene, TICK_LEN_SEC);
      sim_accum -= TICK_LEN_NS;
    }

    /* How far into the next step we are, for smooth drawing. */
    RenderTiming rt = {
        .alpha  = (float)sim_accum / (float)TICK_LEN_NS,
        .dt_sec = TICK_LEN_SEC,
    };

    fps_counter_tick(&app->fps, dt);

    /* Sleep before drawing so the time spent writing to the terminal doesn't
     * push us over the frame budget. */
    int64_t budget_left = FRAME_BUDGET_NS - (clock_ns() - frame_start);
    clock_sleep_ns(budget_left);

    screen_draw(&app->screen, &app->scene, app->fps.display, rt);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
