/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/* dune_rocket.c — Harkonnen siege: homing missiles pound the Arrakis desert.
 * Homing/pursuit steering after Reynolds (GDC 1999); ballistic spark
 * particles after Reeves (ACM TOG 1983).
 */

/* ── §1 config — tunable constants, all named with units ── */
#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TICK_NS 33333333L /* 30 Hz */
#define MAX_ROCKETS 28
#define MAX_EXPLOSIONS 16
#define N_SPARKS 22
#define TRAIL_LEN 30
#define MAX_PORTS 8
#define SHIP_TOP_ROW 1 /* topmost ship row */
#define SHIP_ROWS 5    /* ship height in rows */

#define ROCKET_SPEED0 15.0f   /* launch speed (cells/sec) */
#define ROCKET_SPEEDMAX 44.0f /* terminal speed */
#define TURN_RATE 3.0f        /* steering sharpness */
#define WOBBLE_AMP 0.28f      /* lateral wobble (radians equiv.) */
#define WOBBLE_FREQ 4.0f      /* oscillations per second */
#define LAUNCH_RATE0 1.1f     /* seconds between auto-launches */
#define LAUNCH_RATE_STEP                                                       \
  0.2f /* +/- adjusts the auto-launch interval by this (sec) */
#define LAUNCH_RATE_MIN                                                        \
  0.2f /* fastest auto-launch interval the user can set (sec) */
#define LAUNCH_RATE_MAX                                                        \
  5.0f              /* slowest auto-launch interval the user can set (sec) */
#define DT_MAX 0.1f /* clamp on frame dt — avoids spiral-of-death (sec) */

/* rocket flight & guidance */
#define ROCKET_ACCEL 20.0f   /* speed ramp toward terminal (cells/sec^2) */
#define ROCKET_ARRIVE 1.5f   /* dist to target counted as a hit (cells) */
#define TURN_BLEND_MAX 0.99f /* cap on per-tick heading blend fraction */
#define WOBBLE_FADE_DIST                                                       \
  10.0f /* wobble fades to zero within this range (cells) */

/* explosion, sparks & shockwave ring */
#define SPARK_GRAVITY 7.0f /* downward accel on debris sparks (cells/sec^2) */
#define EXPLO_LIFE 2.0f    /* explosion lifetime (sec) */
#define RING_SPEED 14.0f   /* shockwave expansion rate (cells/sec) */
#define RING_MAX_R 22.0f   /* radius past which the ring stops drawing (cells) \
                            */
#define RING_ASPECT 0.45f  /* vertical squash so the ring reads as a circle */

/* terrain scorch & launch ports */
#define SCORCH_RADIUS 5.5f /* blast scorch radius on the dunes (cells) */
#define SCORCH_MAX 4 /* scorch intensity at blast centre (0..SCORCH_MAX) */
#define SCORCH_COOL_S 3.0f /* seconds between scorch cool-down steps */
#define PORT_FLASH_S 0.15f /* muzzle-flash duration after a port fires (sec)   \
                            */

/* small random variation added to launches and flight so nothing looks robotic */
#define VEL_EPS 0.01f /* speeds below this read as "no heading" (cells/sec) */
#define LAUNCH_LEAN_SPAN                                                       \
  201 /* discrete steps of the sideways launch shove (symmetric) */
#define LAUNCH_LEAN_SCALE 0.018f /* cells/sec per lean step (peak ~ +/-1.8) */
#define WOBBLE_FREQ_JITTER                                                     \
  0.35f /* per-rocket wobble frequency varies +/-this around WOBBLE_FREQ */
#define WOBBLE_AMP_JITTER                                                      \
  0.45f /* per-rocket wobble amplitude varies +/-this around WOBBLE_AMP */
#define LAUNCH_JITTER                                                          \
  0.6f /* auto-launch interval varies +/-this around launch_rate */
#define FIRST_LAUNCH_DELAY                                                     \
  0.3f /* seconds before the first launch after a reset */

/* spark emission — one ember, in spark_emit/explosion_spawn */
#define SPARK_SPEED_MIN 4.0f  /* slowest ember (cells/sec) */
#define SPARK_SPEED_MAX 13.0f /* fastest ember (cells/sec) */
#define SPARK_LIFE_MIN 0.35f  /* shortest ember lifetime (sec) */
#define SPARK_LIFE_MAX 1.05f  /* longest ember lifetime (sec) */
#define SPARK_UP_CLAMP                                                         \
  0.25f /* cap on sin(angle) so embers burst up/out, not down */
#define SPARK_UP_KICK                                                          \
  3.0f /* extra upward velocity on every ember (cells/sec) */
#define SPARK_SCATTER_SPAN                                                     \
  5 /* discrete spawn-scatter positions about the blast centre */
#define SPARK_SCATTER_SCALE 0.3f /* cells per scatter step (peak ~ +/-0.6) */
#define SPARK_ANGLE_SPAN                                                       \
  100 /* discrete angle-jitter steps on the even spark fan */
#define SPARK_ANGLE_SCALE                                                      \
  0.01f /* radians per angle-jitter step (peak ~ +/-0.5) */

/* targeting & launch cadence */
#define TARGET_MARGIN                                                          \
  2 /* keep impact targets this many columns off each edge */
#define TARGET_ABOVE_GROUND                                                    \
  0.3f /* aim this far above the sand surface (cells) */
#define AUTO_LAUNCH_RESERVE                                                    \
  2 /* auto-launch only while this many pool slots stay free */

/* dune height field — surface row measured up from the bottom HUD */
#define GROUND_BASE_OFF                                                        \
  7 /* nominal surface sits this many rows above the bottom */
#define GROUND_TOP_OFF                                                         \
  12 /* highest the surface may rise (rows above bottom) — clears ship */
#define GROUND_BOT_OFF                                                         \
  4 /* lowest the surface may fall (rows above bottom) — clears HUD */

/* carrier hull geometry */
#define HULL_W_MIN 30 /* narrowest hull (columns) */
#define HULL_W_MAX 82 /* widest hull (columns) */
#define PORT_INSET                                                             \
  4 /* launch ports inset this many columns from each hull end */

/* render — brightness-ramp breakpoints (fraction of life/age, freshest = 1) */
#define TRAIL_HOT                                                              \
  0.70f /* trail above this age-fraction -> bright orange + bold */
#define TRAIL_MID                                                              \
  0.38f /* ... above this -> mid orange; below -> dim dark-red */
#define EXP_RING_HOT 0.55f /* ring above this life-fraction -> white '*' */
#define EXP_RING_MID                                                           \
  0.25f /* ... above this -> yellow '+'; below -> orange '.' */
#define EXP_CORE_FRAC                                                          \
  0.65f                /* draw the bright core sprite while fresher than this */
#define SPARK_HOT 0.6f /* ember above this life-fraction -> white '*' */
#define SPARK_MID 0.3f /* ... above this -> yellow '+'; below -> orange '.' */

/* render — shockwave ring sampling & sand texture */
#define RING_PTS_PER_R 6.28f /* ~2*pi: ring sample count ~ circumference */
#define RING_MIN_PTS 12 /* floor on ring samples so small rings still read */
#define SAND_SPECKLE_PERIOD                                                    \
  11 /* 2-in-this-many sub-surface cells get a speckle dot */
#define SLOPE_AXIS_RATIO                                                       \
  1.73f /* ~tan(60deg): trail slope counts as axis-aligned past this */

#define MAX_COLS 512

/* ── §2 timing — monotonic clock and sleep ── */
static long long clock_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void clock_sleep_ns(long long ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* ── §3 state — domain types and the Scene aggregate ── */

/* names for the ncurses color pairs set up in color_init */
enum {
  CP_SHIP_HULL = 1,
  CP_SHIP_PORT,
  CP_RKT_HEAD,
  CP_RKT_HOT,
  CP_RKT_MID,
  CP_RKT_DIM,
  CP_EXP_CORE,
  CP_EXP_MID,
  CP_EXP_DIM,
  CP_GROUND,
  CP_SAND,
  CP_SCORCH,
  CP_HUD,
  CP_HINT
};

/* Terrain — the desert floor. We store the dunes as one "height field": just
 * the surface row for each screen column, since the sand is drawn one glyph
 * tall per column. That's far cheaper than a full 2-D grid, and the slope of
 * the surface (which gives us the / \ _ glyph) is just the difference between
 * neighbouring columns. The dune shape itself is a stack of sine waves built
 * in terrain_init. Scorch marks where blasts have charred the sand; it slowly
 * fades back to clean over time.
 *
 *   ground[col]  : screen row of the sand surface at this column. Bigger value
 *                  = lower on screen (row 0 is the top). Kept inside
 *                  [rows-GROUND_TOP_OFF, rows-GROUND_BOT_OFF] so dunes never
 *                  climb into the ship or bury the bottom HUD.
 *   scorch[col]  : how burnt this column is, 0 (clean) to SCORCH_MAX (fresh
 *                  blast centre). Stamped by terrain_scorch; draw_terrain shows
 *                  it as '*' near the centre, '.' further out.
 *   scorch_decay : one shared timer counting toward the next cool-down step.
 *                  Rather than a separate timer per scar, when this passes
 *                  SCORCH_COOL_S every burnt column drops one level at once. */
typedef struct {
  int ground[MAX_COLS];
  int scorch[MAX_COLS];
  float scorch_decay;
} Terrain;

/* Carrier — the ship across the top of the screen that fires every rocket. The
 * hull is just decoration, so we only keep its column span and the row rockets
 * drop from. The real content is the row of launch ports, stored as two
 * matching arrays: where each port sits (fixed) and how recently it fired (a
 * fading muzzle flash). The hull width scales to the terminal in carrier_init.
 *
 *   left, right  : leftmost / rightmost hull column on screen (border included).
 *   launch_row   : row rockets spawn from, just under the hull belly.
 *   n_ports      : how many launch ports there are; also the used length of
 *                  both arrays below.
 *   port_x[i]    : screen column of port i, spread evenly along the hull.
 *   port_flash[i]: seconds of muzzle flash left on port i. Set to PORT_FLASH_S
 *                  when it fires, counts down each tick; while above 0 the port
 *                  shows a red '*', otherwise a gray '|'. */
typedef struct {
  int left, right;
  int launch_row;
  int n_ports;
  int port_x[MAX_PORTS];
  float port_flash[MAX_PORTS];
} Carrier;

/* RState — what stage of life a rocket is in. RS_FLYING means it is still in
 * the air and being steered each tick. RS_DONE means it has finished: hit its
 * target, struck the sand, or flown off-screen. A rocket marked RS_DONE is
 * turned into an explosion and its slot freed on the same tick, so it is never
 * actually drawn — it is just a one-frame "clean me up" flag. */
typedef enum { RS_FLYING = 0, RS_DONE } RState;

/* Rocket — one homing missile chasing a fixed spot on the ground. Each tick it
 * turns its heading a little toward the target rather than snapping straight at
 * it (a turn-rate-limited "seek", after Reynolds), so the path curves like a
 * real missile. On top of that it sways side to side with a sine wobble to look
 * powered, and the sway fades out as it nears the target so the final dive is
 * clean. It also keeps a short history of past positions to draw a fading trail
 * behind it.
 *
 *   x, y        : position in cells (fractional; rounded only when drawn).
 *   vx, vy      : velocity in cells/sec = heading direction times speed.
 *   tx, ty      : the ground spot it is aiming for, chosen at launch and never
 *                 changed. Within ROCKET_ARRIVE cells of it counts as a hit.
 *   wobble_ph   : where it is in its side-to-side sway, in radians.
 *   wobble_freq : how fast this rocket sways (rad/sec). Randomised per rocket
 *                 so they don't all sway in lock-step.
 *   wobble_amp  : how far this rocket sways. Randomised per rocket, then faded
 *                 down to zero as it closes in on the target.
 *   speed       : how fast it is going (cells/sec). Tracked on its own because
 *                 the heading gets re-normalised each tick; speed ramps up from
 *                 launch toward ROCKET_SPEEDMAX.
 *   state       : flying or done (see RState above).
 *   tbx[],tby[] : ring buffer of the last few positions, for the trail.
 *   t_head      : next slot in that buffer to overwrite (wraps around).
 *   t_len       : how many slots are filled, up to TRAIL_LEN. A ring buffer
 *                 lets us add a point without allocating or shifting anything.
 *   active      : 1 if this slot of the fixed rocket pool is in use. Slots are
 *                 reused instead of malloc'd, so the hot loop never allocates. */

typedef struct {
  float x, y;      /* position (cells) */
  float vx, vy;    /* velocity (cells/sec) */
  float tx, ty;    /* ground target */
  float wobble_ph; /* wobble phase (radians) */
  float wobble_freq;
  float wobble_amp;
  float speed; /* current speed magnitude */
  RState state;
  /* trail ring buffer */
  float tbx[TRAIL_LEN];
  float tby[TRAIL_LEN];
  int t_head; /* next-write index */
  int t_len;  /* filled entries 0..TRAIL_LEN */
  int active;
} Rocket;

/* Spark — one flying ember thrown out by an explosion. It just arcs through the
 * air under gravity and fades; sparks don't interact with each other, which
 * keeps them cheap and is plenty convincing. Sparks fly up and out, never down
 * into the ground (the downward part of their launch is clamped at spawn).
 *
 *   x, y      : position in cells.  vx, vy : velocity in cells/sec.
 *   life      : seconds of life left; counts down, gone at 0.
 *   max_life  : life it started with. life/max_life goes 1 -> 0 over its life
 *               and is the only "how old am I" signal, driving its color fade.
 *   active    : 1 while alive — slot flag in the explosion's spark pool. */
typedef struct {
  float x, y, vx, vy;
  float life, max_life;
  int active;
} Spark;

/* Explosion — one impact burst, made of three things drawn on top of each
 * other: a brief bright core, an expanding shockwave ring, and a cloud of
 * flying sparks. They all start at the same place and age off one shared clock,
 * which is why they live in one struct. The ring is squashed vertically
 * (RING_ASPECT) so it looks round on screen, since character cells are about
 * twice as tall as they are wide.
 *
 *   x, y      : centre of the blast, in cells.
 *   life      : seconds remaining (starts at EXPLO_LIFE); burst dies at 0.
 *   max_life  : life it started with. life/max_life goes 1 (fresh) -> 0 (dying)
 *               and picks the ring/core color and glyph (* -> + -> .).
 *   ring_r    : current shockwave radius in cells; grows over time, stops being
 *               drawn past RING_MAX_R. Tracked apart from life so the ring can
 *               finish expanding while the core and sparks are still fading.
 *   sparks    : this burst's pool of embers, all thrown at spawn.
 *   active    : 1 while this slot of the explosion pool is in use. */
typedef struct {
  float x, y;
  float life, max_life;
  float ring_r; /* expanding shockwave radius */
  Spark sparks[N_SPARKS];
  int active;
} Explosion;

/* Scene — the whole simulation gathered into one struct. Most functions take
 * just the piece they need rather than the whole Scene. The rocket and
 * explosion arrays are fixed-size pools: nothing is allocated after startup,
 * each entry has an `active` flag, and launching scans for a free slot (if the
 * pool is full, the launch is simply dropped).
 *
 *   terrain      : the dunes and their scorch marks.
 *   carrier      : the ship and its launch ports.
 *   rockets[]    : pool of up to MAX_ROCKETS missiles in flight.
 *   explosions[] : pool of up to MAX_EXPLOSIONS active bursts.
 *   launch_rate  : seconds between automatic launches; the only thing the user
 *                  tunes (+/- keys). Survives reset and resize, unlike the rest.
 *   launch_timer : seconds counting down to the next auto-launch. Re-armed with
 *                  a bit of randomness so firing never feels metronomic.
 *   paused       : 1 freezes the simulation; drawing keeps going. */
typedef struct {
  Terrain terrain;
  Carrier carrier;
  Rocket rockets[MAX_ROCKETS];
  Explosion explosions[MAX_EXPLOSIONS];
  float launch_rate;
  float launch_timer;
  int paused;
} Scene;

/* the single simulation instance (BSS); launch_rate persists across resets */
static Scene g_scene = {.launch_rate = LAUNCH_RATE0};

/* ── §4 logic — pure helpers that read inputs and return a value ── */

/* random number from lo up to hi, in steps of 0.01 */
static float frand_range(float lo, float hi) {
  int steps = (int)((hi - lo) * 100.f + 0.5f);
  return lo + (float)(rand() % steps) * 0.01f;
}

/* random offset spread evenly around 0, with n possible steps of size scale */
static float rand_centered(int n, float scale) {
  int off = rand() % n - n / 2;
  return (float)off * scale;
}

/* shrink a vector to length 1 in place; a near-zero vector is left alone */
static void normalize(float *x, float *y) {
  float len = sqrtf(*x * *x + *y * *y);
  if (len > 0.0001f) {
    *x /= len;
    *y /= len;
  }
}

/* turn a "t-th oldest" trail position into its real slot in the ring buffer */
static int trail_index(const Rocket *r, int t) {
  return (r->t_head - r->t_len + t + TRAIL_LEN * 2) % TRAIL_LEN;
}

/* has the rocket flown off the sides, into the sand, or below the play area? */
static int rocket_off_field(const Rocket *r, const Terrain *terrain, int cols,
                            int rows) {
  int ic = (int)(r->x + 0.5f);
  if (ic < 0 || ic >= cols)
    return 1; /* off the sides */
  if (r->y >= (float)terrain->ground[ic])
    return 1; /* into the sand */
  if (r->y >= (float)(rows - 2))
    return 1; /* below the field */
  return 0;
}

/* pick the glyph for the rocket nose by which of 8 directions it points */
static char dir_char_rocket(float vx, float vy) {
  float a = atan2f(vy, vx) * (180.f / (float)M_PI);
  if (a > -22.5f && a <= 22.5f)
    return '>';
  if (a > 22.5f && a <= 67.5f)
    return '\\';
  if (a > 67.5f && a <= 112.5f)
    return 'v';
  if (a > 112.5f && a <= 157.5f)
    return '/';
  if (a > 157.5f || a <= -157.5f)
    return '<';
  if (a > -157.5f && a <= -112.5f)
    return '\\';
  if (a > -112.5f && a <= -67.5f)
    return '^';
  return '/'; /* up-right */
}

/* pick a line glyph (| - / \) matching the slope of a trail segment */
static char dir_char_trail(float vx, float vy) {
  float ax = fabsf(vx), ay = fabsf(vy);
  if (ay > ax * SLOPE_AXIS_RATIO)
    return '|';
  if (ax > ay * SLOPE_AXIS_RATIO)
    return '-';
  return (vx * vy > 0.f) ? '\\' : '/';
}

/* how many rockets are currently in flight */
static int count_active(const Rocket *rockets) {
  int n = 0;
  for (int i = 0; i < MAX_ROCKETS; i++)
    if (rockets[i].active)
      n++;
  return n;
}

/* ── §5 simulation — functions that advance the world one tick ── */

/* burn a circle of sand around column cx, darkest at the centre */
static void terrain_scorch(Terrain *terrain, float cx, float radius) {
  int c0 = (int)(cx - radius);
  int c1 = (int)(cx + radius + 1.f);
  for (int c = c0; c <= c1; c++) {
    if (c < 0 || c >= MAX_COLS)
      continue;
    float d = fabsf((float)c - cx) / radius;
    if (d >= 1.f)
      continue;
    int s = SCORCH_MAX - (int)(d * SCORCH_MAX); /* fades with distance */
    if (s > terrain->scorch[c])
      terrain->scorch[c] = s;
  }
}

/* every SCORCH_COOL_S seconds, let all scorch marks heal one level */
static void terrain_step(Terrain *terrain, float dt, int cols) {
  terrain->scorch_decay += dt;
  if (terrain->scorch_decay > SCORCH_COOL_S) {
    terrain->scorch_decay = 0.f;
    for (int c = 0; c < cols; c++)
      if (terrain->scorch[c] > 0)
        terrain->scorch[c]--;
  }
}

/* spawn a rocket from the given port aimed at (tx,ty); no-op if the pool is full */
static void rocket_launch(Rocket *rockets, Carrier *carrier, int port, float tx,
                          float ty) {
  for (int i = 0; i < MAX_ROCKETS; i++) {
    if (rockets[i].active)
      continue;
    Rocket *r = &rockets[i];
    memset(r, 0, sizeof *r);
    r->active = 1;
    r->x = (float)carrier->port_x[port];
    r->y = (float)carrier->launch_row + 0.5f;
    /* slight random sideways lean at launch */
    float lean = rand_centered(LAUNCH_LEAN_SPAN, LAUNCH_LEAN_SCALE);
    r->vx = lean;
    r->vy = ROCKET_SPEED0;
    r->tx = tx;
    r->ty = ty;
    r->wobble_ph = frand_range(0.f, 6.28f); /* random point in the sway cycle */
    r->wobble_freq = WOBBLE_FREQ * frand_range(1.f - WOBBLE_FREQ_JITTER,
                                               1.f + WOBBLE_FREQ_JITTER);
    r->wobble_amp = WOBBLE_AMP * frand_range(1.f - WOBBLE_AMP_JITTER,
                                             1.f + WOBBLE_AMP_JITTER);
    r->speed = ROCKET_SPEED0;
    r->state = RS_FLYING;
    carrier->port_flash[port] = PORT_FLASH_S;
    return;
  }
}

/* record the rocket's current spot in its trail history */
static void trail_push(Rocket *r) {
  r->tbx[r->t_head] = r->x;
  r->tby[r->t_head] = r->y;
  r->t_head = (r->t_head + 1) % TRAIL_LEN;
  if (r->t_len < TRAIL_LEN)
    r->t_len++;
}

/* steer the rocket one tick: turn toward target, sway, speed up, move */
static void rocket_step(Rocket *r, const Terrain *terrain, float dt, int cols,
                        int rows) {
  if (!r->active || r->state == RS_DONE)
    return;
  trail_push(r);

  /* how far to the target; close enough counts as a hit */
  float dx = r->tx - r->x;
  float dy = r->ty - r->y;
  float dist = sqrtf(dx * dx + dy * dy);
  if (dist < ROCKET_ARRIVE) {
    r->state = RS_DONE;
    return;
  }

  /* the direction we'd like to face: straight at the target */
  float ddx = dx / dist, ddy = dy / dist;

  /* the direction we face right now, from current velocity */
  float spd = sqrtf(r->vx * r->vx + r->vy * r->vy);
  float cdx = (spd > VEL_EPS) ? r->vx / spd : 0.f;
  float cdy = (spd > VEL_EPS) ? r->vy / spd : 1.f;

  /* turn only part way toward the target this tick, so the path curves */
  float tr = fminf(TURN_RATE * dt, TURN_BLEND_MAX);
  cdx += (ddx - cdx) * tr;
  cdy += (ddy - cdy) * tr;
  normalize(&cdx, &cdy);

  /* side-to-side sway, fading out as the rocket nears its target */
  float wscale = fminf(1.f, dist / WOBBLE_FADE_DIST);
  float wobble = r->wobble_amp * wscale * sinf(r->wobble_ph);
  r->wobble_ph += r->wobble_freq * dt;

  /* push the heading sideways by the sway, then re-normalise so the sway only
     bends the direction and doesn't change speed */
  float wdx = cdx - cdy * wobble;
  float wdy = cdy + cdx * wobble;
  normalize(&wdx, &wdy);

  /* ramp speed up toward the cap, then step the position forward */
  r->speed = fminf(r->speed + ROCKET_ACCEL * dt, ROCKET_SPEEDMAX);
  r->vx = wdx * r->speed;
  r->vy = wdy * r->speed;
  r->x += r->vx * dt;
  r->y += r->vy * dt;

  if (rocket_off_field(r, terrain, cols, rows)) {
    r->state = RS_DONE;
    return;
  }
}

/* throw one ember out from (ox,oy) along angle ang, biased upward */
static void spark_emit(Spark *sp, float ox, float oy, float ang) {
  float spd = frand_range(SPARK_SPEED_MIN, SPARK_SPEED_MAX);
  float up = sinf(ang);
  if (up > SPARK_UP_CLAMP)
    up = SPARK_UP_CLAMP; /* keep debris flying up, not down */
  sp->vx = cosf(ang) * spd;
  sp->vy = up * spd - SPARK_UP_KICK;
  sp->x = ox + rand_centered(SPARK_SCATTER_SPAN, SPARK_SCATTER_SCALE);
  sp->y = oy;
  sp->life = frand_range(SPARK_LIFE_MIN, SPARK_LIFE_MAX);
  sp->max_life = sp->life;
  sp->active = 1;
}

/* start an explosion at (x,y): scorch the sand and throw a ring of sparks */
static void explosion_spawn(Explosion *explosions, Terrain *terrain, float x,
                            float y) {
  terrain_scorch(terrain, x, SCORCH_RADIUS);
  for (int i = 0; i < MAX_EXPLOSIONS; i++) {
    if (explosions[i].active)
      continue;
    Explosion *e = &explosions[i];
    memset(e, 0, sizeof *e);
    e->active = 1;
    e->x = x;
    e->y = y;
    e->life = EXPLO_LIFE;
    e->max_life = EXPLO_LIFE;
    e->ring_r = 0.f;
    /* spread sparks evenly around a circle, with a little random jitter */
    for (int s = 0; s < N_SPARKS; s++) {
      float ang = (float)s / (float)N_SPARKS * 2.f * (float)M_PI +
                  rand_centered(SPARK_ANGLE_SPAN, SPARK_ANGLE_SCALE);
      spark_emit(&e->sparks[s], x, y, ang);
    }
    return;
  }
}

/* age one explosion: grow its ring, drift its sparks under gravity */
static void explosion_step(Explosion *e, float dt) {
  if (!e->active)
    return;
  e->life -= dt;
  if (e->life <= 0.f) {
    e->active = 0;
    return;
  }
  e->ring_r += RING_SPEED * dt;
  for (int s = 0; s < N_SPARKS; s++) {
    Spark *sp = &e->sparks[s];
    if (!sp->active)
      continue;
    sp->life -= dt;
    if (sp->life <= 0.f) {
      sp->active = 0;
      continue;
    }
    sp->vy += SPARK_GRAVITY * dt; /* gravity */
    sp->x += sp->vx * dt;
    sp->y += sp->vy * dt;
  }
}

/* choose a random aim point sitting just on top of the sand surface */
static void random_ground_target(const Terrain *terrain, int cols, float *tx,
                                 float *ty) {
  int tc = TARGET_MARGIN + rand() % (cols - 2 * TARGET_MARGIN);
  *tx = (float)tc;
  *ty = (float)terrain->ground[tc] - TARGET_ABOVE_GROUND;
}

/* fire one rocket from a random port, then reset the launch timer with jitter */
static void scene_auto_launch(Scene *s, int cols) {
  int port = rand() % s->carrier.n_ports;
  float tx, ty;
  random_ground_target(&s->terrain, cols, &tx, &ty);
  rocket_launch(s->rockets, &s->carrier, port, tx, ty);
  s->launch_timer =
      s->launch_rate * frand_range(1.f - LAUNCH_JITTER, 1.f + LAUNCH_JITTER);
}

/* advance the whole world one tick: the only place state moves forward */
static void scene_tick(Scene *s, float dt, int cols, int rows) {
  /* fade the muzzle flashes */
  for (int p = 0; p < s->carrier.n_ports; p++)
    if (s->carrier.port_flash[p] > 0.f)
      s->carrier.port_flash[p] -= dt;

  /* auto-launch when the timer runs out and there's room in the pool */
  s->launch_timer -= dt;
  if (s->launch_timer <= 0.f &&
      count_active(s->rockets) < MAX_ROCKETS - AUTO_LAUNCH_RESERVE)
    scene_auto_launch(s, cols);

  /* move each rocket; a finished one turns into an explosion and frees its slot */
  for (int i = 0; i < MAX_ROCKETS; i++) {
    Rocket *r = &s->rockets[i];
    if (!r->active)
      continue;
    rocket_step(r, &s->terrain, dt, cols, rows);
    if (r->state == RS_DONE) {
      explosion_spawn(s->explosions, &s->terrain, r->x, r->y);
      r->active = 0;
    }
  }

  for (int i = 0; i < MAX_EXPLOSIONS; i++)
    explosion_step(&s->explosions[i], dt);

  terrain_step(&s->terrain, dt, cols);
}

/* ── §6 init — build fresh world state at start, reset, and resize ── */

/* shape the dunes: add up four sine waves of decreasing height and increasing
   wiggle for a natural ridged look, then clamp into the allowed row band */
static void terrain_init(Terrain *terrain, int cols, int rows) {
  static const float DUNE_FREQ[4] = {2.3f, 5.7f, 12.1f, 27.3f};
  static const float DUNE_AMP[4] = {2.0f, 1.3f, 0.7f, 0.3f};
  static const float DUNE_PHASE[4] = {0.f, 0.73f, 1.47f, 0.23f};
  for (int c = 0; c < cols; c++) {
    float x = (float)c / (float)(cols > 1 ? cols - 1 : 1);
    float h = 0.f;
    for (int k = 0; k < 4; k++)
      h += sinf(x * (float)M_PI * DUNE_FREQ[k] + DUNE_PHASE[k]) * DUNE_AMP[k];
    int gr = (rows - GROUND_BASE_OFF) + (int)(h + 0.5f);
    if (gr < rows - GROUND_TOP_OFF)
      gr = rows - GROUND_TOP_OFF;
    if (gr > rows - GROUND_BOT_OFF)
      gr = rows - GROUND_BOT_OFF;
    terrain->ground[c] = gr;
  }
  memset(terrain->scorch, 0, sizeof terrain->scorch);
  terrain->scorch_decay = 0.f;
}

/* place the hull and spread its launch ports evenly inside it */
static void carrier_init(Carrier *carrier, int cols) {
  int w = (cols * 3) / 4; /* hull spans 3/4 of the screen width */
  if (w > HULL_W_MAX)
    w = HULL_W_MAX;
  if (w < HULL_W_MIN)
    w = HULL_W_MIN;
  int cx = cols / 2;
  carrier->left = cx - w / 2;
  carrier->right = carrier->left + w;
  carrier->launch_row = SHIP_TOP_ROW + SHIP_ROWS;

  carrier->n_ports = MAX_PORTS;
  int span = w - 2 * PORT_INSET; /* width available for ports inside the hull */
  int div = carrier->n_ports > 1 ? carrier->n_ports - 1 : 1;
  for (int i = 0; i < carrier->n_ports; i++) {
    carrier->port_x[i] = carrier->left + PORT_INSET + span * i / div;
    carrier->port_flash[i] = 0.f;
  }
}

static void scene_reset(Scene *s, int cols, int rows) {
  memset(s->rockets, 0, sizeof s->rockets);
  memset(s->explosions, 0, sizeof s->explosions);
  terrain_init(&s->terrain, cols, rows);
  carrier_init(&s->carrier, cols);
  s->launch_timer = FIRST_LAUNCH_DELAY;
}

/* ── §7 render — draw the current state to the screen, reading only ── */

/* set up the color pairs once; falls back to 8-color terminals */
static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(CP_SHIP_HULL, 250, 236); /* light gray / dark gray */
    init_pair(CP_SHIP_PORT, 196, 236); /* bright red / dark gray */
    init_pair(CP_RKT_HEAD, 231, -1);   /* bright white */
    init_pair(CP_RKT_HOT, 214, -1);    /* orange */
    init_pair(CP_RKT_MID, 202, -1);    /* red-orange */
    init_pair(CP_RKT_DIM, 88, -1);     /* dark red */
    init_pair(CP_EXP_CORE, 231, -1);   /* white */
    init_pair(CP_EXP_MID, 226, -1);    /* yellow */
    init_pair(CP_EXP_DIM, 208, -1);    /* orange */
    init_pair(CP_GROUND, 136, -1);     /* sandy gold */
    init_pair(CP_SAND, 94, -1);        /* dark sand */
    init_pair(CP_SCORCH, 52, -1);      /* very dark red */
    init_pair(CP_HUD, 226, -1);        /* bright yellow — top data bar */
    init_pair(CP_HINT, 51, -1);        /* bright cyan   — bottom action bar */
  } else {
    init_pair(CP_SHIP_HULL, COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_SHIP_PORT, COLOR_RED, COLOR_BLACK);
    init_pair(CP_RKT_HEAD, COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_RKT_HOT, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_RKT_MID, COLOR_RED, COLOR_BLACK);
    init_pair(CP_RKT_DIM, COLOR_RED, COLOR_BLACK);
    init_pair(CP_EXP_CORE, COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_EXP_MID, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_EXP_DIM, COLOR_RED, COLOR_BLACK);
    init_pair(CP_GROUND, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_SAND, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_SCORCH, COLOR_RED, COLOR_BLACK);
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
}

static void draw_terrain(const Terrain *terrain, int cols, int rows) {
  for (int c = 0; c < cols; c++) {
    int gr = terrain->ground[c];
    if (gr < 0 || gr >= rows)
      continue;

    /* pick the surface glyph from whether neighbours rise or fall */
    int prev_gr = (c > 0) ? terrain->ground[c - 1] : gr;
    int next_gr = (c < cols - 1) ? terrain->ground[c + 1] : gr;
    int slope = next_gr - prev_gr;
    char sch = (slope < -1) ? '/' : (slope > 1) ? '\\' : '_';

    if (terrain->scorch[c] > 0) {
      /* burnt sand replaces the normal surface glyph */
      attron(COLOR_PAIR(CP_SCORCH) | A_BOLD);
      mvaddch(gr, c, terrain->scorch[c] >= SCORCH_MAX - 1 ? '*' : '.');
      attroff(COLOR_PAIR(CP_SCORCH) | A_BOLD);
    } else {
      attron(COLOR_PAIR(CP_GROUND) | A_BOLD);
      mvaddch(gr, c, (chtype)(unsigned char)sch);
      attroff(COLOR_PAIR(CP_GROUND) | A_BOLD);
    }

    /* fill the body of the dune below the surface, with occasional speckles */
    attron(COLOR_PAIR(CP_SAND));
    for (int r = gr + 1; r < rows - 1; r++) {
      char fc = ((c + r * 3) % SAND_SPECKLE_PERIOD < 2) ? '.' : ' ';
      mvaddch(r, c, (chtype)(unsigned char)fc);
    }
    attroff(COLOR_PAIR(CP_SAND));
  }
}

static void draw_rockets(const Rocket *rockets) {
  for (int i = 0; i < MAX_ROCKETS; i++) {
    const Rocket *r = &rockets[i];
    if (!r->active || r->state == RS_DONE)
      continue;

    /* draw the trail from the oldest point to the newest */
    for (int t = 0; t < r->t_len; t++) {
      int idx = trail_index(r, t);
      int tr = (int)(r->tby[idx] + 0.5f);
      int tc = (int)(r->tbx[idx] + 0.5f);
      if (tr < 0 || tc < 0)
        continue;

      /* direction toward the next point, used to pick the line glyph */
      float dvx, dvy;
      if (t < r->t_len - 1) {
        int nxt = trail_index(r, t + 1);
        dvx = r->tbx[nxt] - r->tbx[idx];
        dvy = r->tby[nxt] - r->tby[idx];
      } else {
        dvx = r->vx;
        dvy = r->vy;
      }
      char tch = dir_char_trail(dvx, dvy);

      /* newer trail points are brighter (0 = oldest, 1 = freshest) */
      float af = (r->t_len > 1) ? (float)t / (float)(r->t_len - 1) : 1.f;
      int pair;
      attr_t attr = 0;
      if (af > TRAIL_HOT) {
        pair = CP_RKT_HOT;
        attr = A_BOLD;
      } else if (af > TRAIL_MID) {
        pair = CP_RKT_MID;
      } else {
        pair = CP_RKT_DIM;
        attr = A_DIM;
      }

      attron(COLOR_PAIR(pair) | attr);
      mvaddch(tr, tc, (chtype)(unsigned char)tch);
      attroff(COLOR_PAIR(pair) | attr);
    }

    /* rocket head */
    int hr = (int)(r->y + 0.5f);
    int hc = (int)(r->x + 0.5f);
    if (hr >= 0 && hc >= 0) {
      attron(COLOR_PAIR(CP_RKT_HEAD) | A_BOLD);
      mvaddch(hr, hc, (chtype)(unsigned char)dir_char_rocket(r->vx, r->vy));
      attroff(COLOR_PAIR(CP_RKT_HEAD) | A_BOLD);
    }
  }
}

static void draw_explosions(const Explosion *explosions, int cols, int rows) {
  for (int i = 0; i < MAX_EXPLOSIONS; i++) {
    const Explosion *e = &explosions[i];
    if (!e->active)
      continue;
    float frac = e->life / e->max_life; /* 1 = fresh, 0 = dying */

    /* the expanding shockwave ring, squashed so it looks circular */
    if (e->ring_r < RING_MAX_R) {
      int n_pts = (int)(e->ring_r * RING_PTS_PER_R) + RING_MIN_PTS;
      int pair = (frac > EXP_RING_HOT)   ? CP_EXP_CORE
                 : (frac > EXP_RING_MID) ? CP_EXP_MID
                                         : CP_EXP_DIM;
      char rch = (frac > EXP_RING_HOT)   ? '*'
                 : (frac > EXP_RING_MID) ? '+'
                                         : '.';
      for (int p = 0; p < n_pts; p++) {
        float a = (float)p / (float)n_pts * 2.f * (float)M_PI;
        int rc = (int)(e->x + cosf(a) * e->ring_r + 0.5f);
        int rr = (int)(e->y + sinf(a) * e->ring_r * RING_ASPECT + 0.5f);
        if (rr < 0 || rr >= rows || rc < 0 || rc >= cols)
          continue;
        attron(COLOR_PAIR(pair));
        mvaddch(rr, rc, (chtype)(unsigned char)rch);
        attroff(COLOR_PAIR(pair));
      }
    }

    /* bright core flash on fresh explosions */
    if (frac > EXP_CORE_FRAC) {
      int er = (int)(e->y + 0.5f);
      int ec = (int)(e->x + 0.5f);
      attron(COLOR_PAIR(CP_EXP_CORE) | A_BOLD);
      if (er >= 0 && er < rows && ec >= 0 && ec < cols)
        mvaddch(er, ec, '#');
      if (er - 1 >= 0) {
        mvaddch(er - 1, ec, '*');
        if (ec - 1 >= 0)
          mvaddch(er - 1, ec - 1, '+');
        if (ec + 1 < cols)
          mvaddch(er - 1, ec + 1, '+');
      }
      if (ec - 1 >= 0 && er < rows)
        mvaddch(er, ec - 1, '*');
      if (ec + 1 < cols && er < rows)
        mvaddch(er, ec + 1, '*');
      attroff(COLOR_PAIR(CP_EXP_CORE) | A_BOLD);
    }

    /* spark particles */
    for (int s = 0; s < N_SPARKS; s++) {
      const Spark *sp = &e->sparks[s];
      if (!sp->active)
        continue;
      int sr = (int)(sp->y + 0.5f);
      int sco = (int)(sp->x + 0.5f);
      if (sr < 0 || sr >= rows || sco < 0 || sco >= cols)
        continue;
      float sf = sp->life / sp->max_life;
      int pair = (sf > SPARK_HOT)   ? CP_EXP_CORE
                 : (sf > SPARK_MID) ? CP_EXP_MID
                                    : CP_EXP_DIM;
      char sch = (sf > SPARK_HOT) ? '*' : (sf > SPARK_MID) ? '+' : '.';
      attron(COLOR_PAIR(pair));
      mvaddch(sr, sco, (chtype)(unsigned char)sch);
      attroff(COLOR_PAIR(pair));
    }
  }
}

/* draw the ship: title, hull box, and the row of launch ports */
static void draw_carrier(const Carrier *carrier) {
  int L = carrier->left, R = carrier->right;
  int row = SHIP_TOP_ROW;

  /* title row */
  const char *title = "HARKONNEN  CARRIER";
  int tx = (L + R) / 2 - (int)strlen(title) / 2;
  attron(COLOR_PAIR(CP_SHIP_HULL) | A_BOLD);
  mvprintw(row, tx, "%s", title);
  row++;

  /* top border */
  mvaddch(row, L, '+');
  for (int c = L + 1; c < R; c++)
    mvaddch(row, c, '-');
  mvaddch(row, R, '+');
  row++;

  /* hull body with a hatched armor pattern */
  attroff(A_BOLD);
  mvaddch(row, L, '|');
  for (int c = L + 1; c < R; c++) {
    char bc = (((c - L) * 3 + row) % 7 < 2) ? '#' : '=';
    mvaddch(row, c, (chtype)(unsigned char)bc);
  }
  mvaddch(row, R, '|');
  row++;

  /* bottom border, with a notch at each launch port */
  attron(A_BOLD);
  mvaddch(row, L, '+');
  for (int c = L + 1; c < R; c++) {
    int is_port = 0;
    for (int p = 0; p < carrier->n_ports; p++)
      if (c == carrier->port_x[p]) {
        is_port = 1;
        break;
      }
    mvaddch(row, c, is_port ? '+' : '-');
  }
  mvaddch(row, R, '+');
  row++;

  /* port shafts: flash red on launch, otherwise gray */
  attroff(A_BOLD);
  for (int p = 0; p < carrier->n_ports; p++) {
    if (carrier->port_flash[p] > 0.f) {
      attron(COLOR_PAIR(CP_SHIP_PORT) | A_BOLD);
      mvaddch(row, carrier->port_x[p], '*');
      attroff(COLOR_PAIR(CP_SHIP_PORT) | A_BOLD);
    } else {
      attron(COLOR_PAIR(CP_SHIP_HULL));
      mvaddch(row, carrier->port_x[p], '|');
      attroff(COLOR_PAIR(CP_SHIP_HULL));
    }
  }

  attroff(COLOR_PAIR(CP_SHIP_HULL) | A_BOLD);
}

/* draw the two status bars: stats along the top row, key legend along the
   bottom. Text is clipped so a narrow terminal can't overflow or wrap. */
static void draw_hud(float rate, int n_active, int paused, int rows, int cols) {
  if (cols < 1 || rows < 1)
    return;

  /* top row: title on the left, live stats on the right */
  char left[24], right[64];
  snprintf(left, sizeof left, " DUNE ROCKET ");
  snprintf(right, sizeof right, " rate:%.1fs   rockets:%d/%d   %s ", rate,
           n_active, MAX_ROCKETS, paused ? "PAUSED" : "running");
  int rx = cols - (int)strlen(right); /* where the right-aligned stats start */
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  for (int c = 0; c < cols; c++)
    mvaddch(0, c, ' ');
  if (rx >= 0) {
    mvprintw(0, 0, "%.*s", rx, left); /* title, clipped before the stats */
    mvprintw(0, rx, "%s", right);
  } else {
    mvprintw(0, 0, "%.*s", cols, right); /* too narrow: show stats only */
  }
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* bottom row: the list of keys */
  int brow = rows - 1;
  if (brow > 0) {
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    for (int c = 0; c < cols; c++)
      mvaddch(brow, c, ' ');
    mvprintw(brow, 0, "%.*s", cols,
             " q:quit  p:pause  r:reset  +/-:rate  spc:salvo ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
  }
}

/* ── §8 events — signal handlers and key-driven actions ── */
static volatile sig_atomic_t g_running = 1;
static volatile sig_atomic_t g_need_resize = 0;
static void on_sigint(int s) {
  (void)s;
  g_running = 0;
}
static void on_sigwinch(int s) {
  (void)s;
  g_need_resize = 1;
}

/* fire all ports at once — one rocket per port toward random targets */
static void scene_salvo(Scene *s, int cols) {
  for (int p = 0;
       p < s->carrier.n_ports && count_active(s->rockets) < MAX_ROCKETS - 1;
       p++) {
    float tx, ty;
    random_ground_target(&s->terrain, cols, &tx, &ty);
    rocket_launch(s->rockets, &s->carrier, p, tx, ty);
  }
}

/* ── §9 main — the frame loop: input, then simulate, then draw ── */
int main(void) {
  srand((unsigned)time(NULL));
  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);
  signal(SIGWINCH, on_sigwinch);

  initscr();
  noecho();
  cbreak();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  typeahead(-1);
  color_init();

  int rows = LINES, cols = COLS;
  scene_reset(&g_scene, cols, rows);

  long long prev = clock_ns();

  while (g_running) {
    int ch;
    while ((ch = getch()) != ERR) {
      switch (ch) {
      case 'q':
      case 'Q':
      case 27:
        g_running = 0;
        break;
      case 'p':
      case 'P':
        g_scene.paused = !g_scene.paused;
        break;
      case 'r':
      case 'R':
        scene_reset(&g_scene, cols, rows);
        break;
      case ' ':
        scene_salvo(&g_scene, cols);
        break;
      case '+':
      case '=':
        g_scene.launch_rate =
            fmaxf(LAUNCH_RATE_MIN, g_scene.launch_rate - LAUNCH_RATE_STEP);
        break;
      case '-':
        g_scene.launch_rate =
            fminf(LAUNCH_RATE_MAX, g_scene.launch_rate + LAUNCH_RATE_STEP);
        break;
      case KEY_RESIZE:
        g_need_resize = 1;
        break;
      }
    }

    if (g_need_resize) {
      g_need_resize = 0;
      endwin();
      refresh();
      rows = LINES;
      cols = COLS;
      scene_reset(&g_scene, cols, rows);
    }

    long long now = clock_ns();
    float dt = (float)(now - prev) * 1e-9f;
    if (dt > DT_MAX)
      dt = DT_MAX;
    prev = now;

    if (!g_scene.paused)
      scene_tick(&g_scene, dt, cols, rows);

    erase();
    draw_terrain(&g_scene.terrain, cols, rows);
    draw_explosions(g_scene.explosions, cols, rows);
    draw_rockets(g_scene.rockets);
    draw_carrier(&g_scene.carrier);
    draw_hud(g_scene.launch_rate, count_active(g_scene.rockets), g_scene.paused,
             rows, cols);
    wnoutrefresh(stdscr);
    doupdate();

    clock_sleep_ns(TICK_NS - (clock_ns() - now));
  }

  endwin();
  return 0;
}
