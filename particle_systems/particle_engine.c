/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * particle_systems/particle_engine.c — a tiny 2-D particle playground in the
 * terminal. It spawns lots of little dots, pushes them around with gravity
 * and drag, bounces or kills them at the screen edges, and draws the result.
 * Four built-in presets (fountain, fireworks, rainfall, explosion) are just
 * different settings for the same machinery.
 *
 * The fixed-timestep main loop and the ncurses display layer mirror the
 * project's shared framework.c — see that file for the loop walkthrough.
 *
 * References: Reeves, "Particle Systems" (ACM TOG, 1983) for the pool +
 * emitter + per-particle loop; Witkin & Baraff SIGGRAPH course notes for
 * force accumulation; Hairer/Lubich/Wanner on why symplectic Euler keeps
 * energy from drifting over long runs.
 */

#define _POSIX_C_SOURCE 200809L
#define _USE_MATH_DEFINES

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ── */

/* Every tunable number lives here, so changing how the demo behaves means
 * editing this one block instead of hunting through the code. */

/* ── loop / display ── */
enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,

  TARGET_FPS = 60,     /* how many frames we aim to draw per second  */
  FPS_UPDATE_MS = 500, /* how often the on-screen fps number refreshes */
  HUD_COLS = 72,       /* widest a HUD string is allowed to get       */
};

/* ── particle pool ── */
enum {
  MAX_PARTICLES = 2048, /* the most particles that can be alive at once */
  TRAIL_LEN = 6,        /* how many past positions each particle remembers */
  MAX_BURST = 300,      /* the most particles a single burst can spawn  */
};

/* ── physics ── */
/* All of these are in pixel units (see CELL_W / CELL_H below). A typical
 * terminal works out to roughly 200 px wide by 800 px tall. */
#define GRAVITY_DEFAULT 200.0f /* default downward pull, pixels per second² */
#define GRAVITY_STEP 30.0f     /* how much each g / G keypress changes it    */
#define GRAVITY_MIN 0.0f
#define GRAVITY_MAX 600.0f
#define DRAG_COEFF 0.80f      /* how strongly drag slows things down        */
#define BOUNCE_DAMPING 0.45f  /* speed kept after a bounce (rest is lost)   */
#define MAX_SPEED_NORM 650.0f /* speed that maps to the brightest glyph     */

/* ── density grid (heatmap mode) ── */
/* A fixed-size grid big enough for the largest terminal we support, so it
 * can live in static memory and never needs the heap (about 120 KB). */
#define GRID_MAX_W 300
#define GRID_MAX_H 100

/* ── coordinate system ── */
/* A terminal cell is about twice as tall as it is wide. We run the physics
 * in fine-grained pixel units and only convert to whole cells when drawing,
 * which lets particles drift smoothly instead of jumping cell to cell. */
#define CELL_W 8  /* pixel steps across one column */
#define CELL_H 16 /* pixel steps down one row      */

static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }
static inline int px_to_cx(float px) {
  return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cy(float py) {
  return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── render modes ── */
enum {
  RENDER_GLYPH = 0,   /* faster = denser character, dimmer as it dies */
  RENDER_TRAIL = 1,   /* same, plus a fading tail of past positions   */
  RENDER_HEATMAP = 2, /* particles blur together into a smoke/heat map */
  RENDER_ARROW = 3,   /* an arrow showing which way each one is moving */
  RENDER_COUNT = 4,
};

static const char *const k_render_names[RENDER_COUNT] = {"glyph", "trail",
                                                         "heatmap", "arrow"};

/* ── presets ── */
enum {
  PRESET_FOUNTAIN = 0,
  PRESET_FIREWORKS = 1,
  PRESET_RAINFALL = 2,
  PRESET_EXPLOSION = 3,
  PRESET_COUNT = 4,
};

static const char *const k_preset_names[PRESET_COUNT] = {
    "fountain", "fireworks", "rainfall", "explosion"};

/* ── timing ── */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* ── §2 clock ── */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec r = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&r, NULL);
}

/* ── §3 theme — palettes, the brightness ramp, and the value→glyph lookup ── */

/* The brightness ramp: nine characters from emptiest to most solid-looking.
 * Anything we draw is boiled down to one of these nine levels — a faint '.'
 * for something dim, a packed '@' for something intense. */
static const char k_ramp[] = " .:+x*X#@";
#define RAMP_N (int)(sizeof k_ramp - 1) /* nine levels */

/* The cut-off value for each ramp level. We don't compare raw numbers
 * directly: the eye sees brightness unevenly, so we first squash the value
 * with a gamma curve (pow(v, 1/2.2)). That spreads the mid-tones out so they
 * don't all collapse into the same dull character. */
static const float k_lut_breaks[RAMP_N] = {
    0.000f, 0.080f, 0.180f, 0.290f, 0.390f, 0.500f, 0.620f, 0.750f, 0.900f,
};

/* Turn a 0..1 brightness value into a ramp level (0..RAMP_N-1). */
static int lut_index(float v) {
  if (v <= 0.0f)
    return 0;
  if (v >= 1.0f)
    return RAMP_N - 1;
  float g = powf(v, 1.0f / 2.2f);
  int idx = 0;
  for (int i = RAMP_N - 1; i >= 0; i--)
    if (g >= k_lut_breaks[i]) {
      idx = i;
      break;
    }
  return idx;
}

/* Four colour themes, one colour per ramp level. We set all four up at
 * startup, so the 't' key can switch between them instantly — there's no
 * work to do, just a different base offset into the colour pairs.
 *
 *   theme 0  fire    — red / orange / yellow   (fireworks, explosion)
 *   theme 1  ocean   — blue / cyan / white      (fountain, rainfall)
 *   theme 2  nature  — green / lime             (general, organic)
 *   theme 3  cosmic  — violet / magenta / white (cosmic, energy)
 *
 * The colour pairs are laid out so each theme owns a contiguous block of
 * RAMP_N pairs, leaving the highest ids for the HUD bars. */
#define CP_BASE 1     /* first colour pair we use (1..36: 4 themes × 9 levels) */
#define PAIR_HUD 40   /* the top status bar — bright yellow, same in every theme */
#define PAIR_HINT 41  /* the bottom key-hint bar — bright cyan, same everywhere   */
#define N_THEMES 4

/* One colour theme: its name plus a colour for each of the nine ramp levels.
 * We keep both a rich 256-colour version and a plain 8-colour fallback so the
 * demo still looks right on a basic terminal. */
typedef struct {
  const char *name;
  int fg256[RAMP_N];    /* 256-colour value for each ramp level     */
  int fg8[RAMP_N];      /* fallback colour for 8-colour terminals   */
  attr_t attr8[RAMP_N]; /* bold/dim/normal per level, 8-colour only */
} PTheme;

static const PTheme k_themes[N_THEMES] = {
    {
        /* 0  fire */
        "fire",
        {88, 124, 160, 196, 202, 208, 214, 220, 231},
        {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW,
         COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE},
        {A_DIM, A_NORMAL, A_BOLD, A_BOLD, A_DIM, A_NORMAL, A_BOLD, A_BOLD,
         A_BOLD},
    },
    {
        /* 1  ocean */
        "ocean",
        {17, 19, 25, 33, 39, 45, 51, 159, 231},
        {COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN,
         COLOR_CYAN, COLOR_WHITE, COLOR_WHITE},
        {A_DIM, A_NORMAL, A_BOLD, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
         A_BOLD},
    },
    {
        /* 2  nature */
        "nature",
        {22, 28, 34, 40, 46, 82, 118, 154, 231},
        {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
         COLOR_GREEN, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
        {A_DIM, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
         A_BOLD},
    },
    {
        /* 3  cosmic */
        "cosmic",
        {53, 91, 93, 129, 165, 201, 207, 213, 231},
        {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
         COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
        {A_DIM, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_DIM, A_NORMAL,
         A_BOLD},
    },
};

static void color_init(void) {
  start_color();
  use_default_colors();
  for (int t = 0; t < N_THEMES; t++) {
    for (int i = 0; i < RAMP_N; i++) {
      int pair = CP_BASE + t * RAMP_N + i;
      if (COLORS >= 256)
        init_pair(pair, k_themes[t].fg256[i], COLOR_BLACK);
      else
        init_pair(pair, k_themes[t].fg8[i], COLOR_BLACK);
    }
  }
  /* The HUD bars use fixed bright colours that don't change with the theme,
   * so they stay easy to read no matter what's animating behind them. */
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, COLOR_BLACK); /* bright yellow */
    init_pair(PAIR_HINT, 51, COLOR_BLACK); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, COLOR_BLACK);
    init_pair(PAIR_HINT, COLOR_CYAN, COLOR_BLACK);
  }
}

/* Hand back the colour-and-brightness to draw with for a given theme and
 * ramp level — what the rest of the renderer turns on before printing. */
static attr_t theme_attr(int theme, int level) {
  int pair = CP_BASE + theme * RAMP_N + level;
  attr_t a = COLOR_PAIR(pair);
  if (COLORS >= 256) {
    if (level >= RAMP_N - 2)
      a |= A_BOLD;
  } else {
    a |= k_themes[theme].attr8[level];
  }
  return a;
}

/* ── §4 structs — Particle and Emitter ── */

/* Particle — one dot in the world, and one slot in the fixed-size pool.
 *
 * It carries everything the physics step and the renderer need: where it is,
 * how it's moving, how long it has left to live, and a short memory of where
 * it just was (for drawing trails). A slot is either alive (updated and
 * drawn) or dead (skipped, free to be reused for a new particle).
 *
 * Life story of a slot: spawn_one fills in every field and flips it alive;
 * each tick it moves and ages a little; once its lifetime runs out or it
 * hits a wall set to "kill", it goes dead and the slot is free again.
 *
 * Positions and velocities are kept in fine pixel units, not whole cells, so
 * a particle can move a fraction of a cell per frame and look like it's
 * gliding. We only round to a cell at the moment we draw it. */
typedef struct {
  float x, y;          /* where it is now, in pixels                         */
  float vx, vy;        /* how fast it's moving, pixels/sec (vy<0 = going up) */
  float ax, ay;        /* this tick's pushes, wiped and rebuilt every step   */
  float lifetime;      /* seconds left to live; hits 0 → the particle dies   */
  float max_lifetime;  /* the lifetime it started with; used for the fade    */
  float density;       /* its "heaviness": heavier ones speed up less and    */
                       /* are slowed by drag faster (range about 0.1 .. 3.0) */
  int color;           /* optional fixed ramp level; 0 means pick by speed   */
  bool alive;          /* true = in use; false = empty slot, free to reuse   */
  int trail_cx[TRAIL_LEN]; /* recent cell positions, kept for drawing the    */
  int trail_cy[TRAIL_LEN]; /* trail (already in cells so we don't re-convert)*/
  int trail_head;          /* where the next trail entry gets written        */
} Particle;

/* Emitter — the nozzle that makes new particles. It holds the recipe for a
 * fresh particle: where to put it, which way to fling it, how fast, how long
 * it lives, how heavy it is. It only creates particles — once one is born the
 * emitter has nothing more to do with it. Switching preset replaces the whole
 * recipe at once (see preset_apply in §10).
 *
 * Two ways particles come out, both driven by the Scene, not the emitter:
 *   - steady stream: while `active` and `rate` > 0, drip out particles every
 *     tick (fountain, rainfall).
 *   - bursts: something calls spawn_burst(N) at a chosen moment and a whole
 *     clump appears at once (fireworks, explosion). Bursts ignore `active`.
 * Both can happen together — e.g. a running stream plus a manual 'b' burst. */
typedef struct {
  float x, y;        /* where particles are born, in pixels                 */
  float angle;       /* the average direction to fling them (radians):      */
                     /* -90° points straight up, +90° straight down         */
  float spread;      /* width of the spray cone; 0 = a tight jet,           */
                     /* a full circle = blast out in every direction        */
  float speed_min;   /* birth speed range, pixels/sec — each particle gets  */
  float speed_max;   /* a random speed picked between these two             */
  float life_min;    /* how many seconds a new particle lives — picked      */
  float life_max;    /* at random in this range                             */
  float density_min; /* heaviness range — picked at random per particle     */
  float density_max; /* (see Particle.density for what heaviness does)      */
  float rate;        /* steady-stream rate, particles per second;           */
                     /* 0 means burst-only (no steady stream)               */
  float rate_accum;  /* leftover fraction of a particle carried between     */
                     /* ticks, so a slow rate still spawns the right number */
                     /* on average instead of rounding down to zero forever */
  float spawn_width; /* how far to scatter births sideways; 0 = a point,    */
                     /* full screen width = spread across the whole top edge*/
  bool active;       /* is the steady stream on? toggled by the 'e' key     */
} Emitter;

/* ── §5 pool — fixed-size particle storage, no malloc ── */

/* All the particles live in one fixed array set aside when the program
 * starts. We never grow or shrink it, so spawning a particle is just filling
 * in an empty slot — there's no malloc anywhere in the hot loop.
 *
 * To find a free slot, pool_alloc keeps a cursor and scans forward from where
 * it left off, wrapping around the end. Since particles are born and dying at
 * a similar pace, an empty slot is almost always close by, so this is cheap.
 *
 * If every slot is taken, pool_alloc just returns NULL and the new particle
 * is quietly skipped — the demo keeps running instead of crashing. */
static Particle g_particles[MAX_PARTICLES];
static int g_cursor = 0; /* where the next slot search starts from */

static void pool_clear(void) {
  memset(g_particles, 0, sizeof g_particles);
  g_cursor = 0;
}

/* Hand back a free slot, or NULL if the pool is completely full. */
static Particle *pool_alloc(void) {
  for (int i = 0; i < MAX_PARTICLES; i++) {
    int idx = (g_cursor + i) % MAX_PARTICLES;
    if (!g_particles[idx].alive) {
      g_cursor = (idx + 1) % MAX_PARTICLES;
      return &g_particles[idx];
    }
  }
  return NULL;
}

/* Count how many particles are currently alive (for the on-screen stats). */
static int pool_alive_count(void) {
  int n = 0;
  for (int i = 0; i < MAX_PARTICLES; i++)
    if (g_particles[i].alive)
      n++;
  return n;
}

/* ── §6 forces — gravity and drag ── */

/* Work out the pushes on one particle for this tick. We wipe last tick's
 * pushes and add each force in turn. Forces are kept separate on purpose: to
 * switch one off you just delete its line, nothing else changes. Right now
 * the only push is gravity; drag is handled separately in §7 because it works
 * better as a slow-down on speed than as another push. */
static void apply_forces(Particle *p, float gravity) {
  /* start fresh — forces are recomputed from scratch every tick */
  p->ax = 0.0f;
  p->ay = 0.0f;

  /* gravity pulls everything down equally, no matter how heavy */
  p->ay += gravity;
}

/* ── §7 physics — moving particles and bouncing them off walls ── */

/* ── per-particle helpers that only read (no changes) ── */

/* How fast this particle is going (the length of its velocity). */
static inline float particle_speed(const Particle *p) {
  return sqrtf(p->vx * p->vx + p->vy * p->vy);
}

/* A rough "energy" number for this particle — heavier and faster means more.
 * We add these up across all particles as a sanity gauge: on a sealed-up
 * simulation the total shouldn't wander far. */
static inline float particle_kinetic_energy(const Particle *p) {
  return 0.5f * p->density * (p->vx * p->vx + p->vy * p->vy);
}

/* ── per-particle helpers that change the particle ── */

/* Slow a particle down a touch to mimic air resistance. We scale its speed
 * down rather than adding a backwards push: scaling can never overshoot and
 * fling the particle backwards, even with a big timestep, so it's rock
 * stable. Heavier particles are slowed more. */
static inline void particle_apply_drag(Particle *p, float dt) {
  float damp = 1.0f - DRAG_COEFF * p->density * dt;
  if (damp < 0.0f)
    damp = 0.0f;
  p->vx *= damp;
  p->vy *= damp;
}

/* Move the particle one step. First update its speed from the pushes, then
 * move it using that brand-new speed. Doing it in that order (speed first,
 * then position) keeps energy from slowly creeping up, so a fountain left
 * running for a long time won't drift higher and higher. */
static inline void particle_integrate_symplectic(Particle *p, float dt) {
  p->vx += p->ax * dt;
  p->vy += p->ay * dt;
  p->x += p->vx * dt;
  p->y += p->vy * dt;
}

/* Age the particle by one tick; mark it dead once its time runs out.
 * Returns true if it's still alive afterwards. */
static inline bool particle_age(Particle *p, float dt) {
  p->lifetime -= dt;
  if (p->lifetime <= 0.0f) {
    p->alive = false;
    return false;
  }
  return true;
}

/* Remember where the particle is now by adding its current cell to its little
 * trail memory. We store it in cells (not pixels) so the trail can be drawn
 * straight away without converting. The memory holds the last few spots and
 * the oldest one quietly drops off. */
static inline void particle_push_trail(Particle *p) {
  p->trail_cx[p->trail_head] = px_to_cx(p->x);
  p->trail_cy[p->trail_head] = px_to_cy(p->y);
  p->trail_head = (p->trail_head + 1) % TRAIL_LEN;
}

/* ── one update step over every particle ── */

/* For each alive particle: figure out the pushes, optionally slow it with
 * drag, move it, age it, record its trail spot, and tally up speed and energy
 * for the stats panel. */
static void update_particles(float dt, float gravity, bool drag_on,
                             float *out_avg_vel, float *out_energy) {
  double vel_sum = 0.0;
  double energy_sum = 0.0;
  int alive = 0;

  for (int i = 0; i < MAX_PARTICLES; i++) {
    Particle *p = &g_particles[i];
    if (!p->alive)
      continue;

    apply_forces(p, gravity);
    if (drag_on)
      particle_apply_drag(p, dt);
    particle_integrate_symplectic(p, dt);
    if (!particle_age(p, dt))
      continue;
    particle_push_trail(p);

    vel_sum += particle_speed(p);
    energy_sum += particle_kinetic_energy(p);
    alive++;
  }

  *out_avg_vel = (alive > 0) ? (float)(vel_sum / alive) : 0.0f;
  *out_energy = (float)energy_sum;
}

/* Keep particles inside the screen. Each of the four edges is set up to do
 * one of two things when a particle reaches it: kill it (it just disappears,
 * like raindrops hitting the ground) or bounce it back (with some speed lost,
 * so it doesn't bounce forever). The Scene's three "kills" flags pick which
 * behaviour each edge uses. */

/* ── one helper per edge ── *
 * Each one checks a single edge. If the particle hasn't reached it, leave it
 * alone and return true. If it has: either kill it (return false) or nudge it
 * back inside, flip the right part of its velocity, and bleed off some speed
 * so the bounce isn't perfectly elastic. */

static inline bool particle_resolve_floor(Particle *p, float world_h,
                                          bool kills) {
  if (p->y < world_h)
    return true;
  if (kills) {
    p->alive = false;
    return false;
  }
  p->y = world_h;
  p->vy = -fabsf(p->vy) * BOUNCE_DAMPING; /* send it back up        */
  p->vx *= BOUNCE_DAMPING;                /* scrub off sideways speed */
  return true;
}

static inline bool particle_resolve_ceiling(Particle *p, bool kills) {
  if (p->y >= 0.0f)
    return true;
  if (kills) {
    p->alive = false;
    return false;
  }
  p->y = 0.0f;
  p->vy = fabsf(p->vy) * BOUNCE_DAMPING; /* send it back down */
  return true;
}

static inline bool particle_resolve_left_wall(Particle *p, bool kills) {
  if (p->x >= 0.0f)
    return true;
  if (kills) {
    p->alive = false;
    return false;
  }
  p->x = 0.0f;
  p->vx = fabsf(p->vx) * BOUNCE_DAMPING; /* send it back right */
  return true;
}

static inline bool particle_resolve_right_wall(Particle *p, float world_w,
                                               bool kills) {
  if (p->x < world_w)
    return true;
  if (kills) {
    p->alive = false;
    return false;
  }
  p->x = world_w;
  p->vx = -fabsf(p->vx) * BOUNCE_DAMPING; /* send it back left */
  return true;
}

/* ── check every edge against every particle ── */

/* For each particle, test the four edges in turn. The moment one edge kills
 * it, stop and move on to the next particle; a bounce just nudges it back and
 * the remaining edge checks still run. */
static void handle_collisions(int cols, int rows, bool floor_kills,
                              bool ceiling_kills, bool wall_kills) {
  float world_w = (float)pw(cols) - 1.0f;
  float world_h = (float)ph(rows) - 1.0f;

  for (int i = 0; i < MAX_PARTICLES; i++) {
    Particle *p = &g_particles[i];
    if (!p->alive)
      continue;

    if (!particle_resolve_floor(p, world_h, floor_kills))
      continue;
    if (!particle_resolve_ceiling(p, ceiling_kills))
      continue;
    if (!particle_resolve_left_wall(p, wall_kills))
      continue;
    if (!particle_resolve_right_wall(p, world_w, wall_kills))
      continue;
  }
}

/* ── §8 emitter — spawning a steady stream and spawning bursts ── */

/* A random number somewhere between lo and hi. */
static float rand_float(float lo, float hi) {
  return lo + ((float)(rand() % 100000) / 100000.0f) * (hi - lo);
}

/* Make one new particle following the emitter's recipe: pick a spot, a
 * direction within the spray cone, a speed, a lifetime, and a heaviness, all
 * randomly within the emitter's ranges. Returns false if the pool was full. */
static bool spawn_one(const Emitter *em, int theme) {
  Particle *p = pool_alloc();
  if (!p)
    return false;

  /* scatter the start sideways for wide emitters (rain across the top edge) */
  float bx =
      em->x + rand_float(-em->spawn_width * 0.5f, em->spawn_width * 0.5f);
  float by = em->y;

  /* pick a direction somewhere inside the spray cone */
  float a = em->angle + rand_float(-em->spread * 0.5f, em->spread * 0.5f);
  float s = rand_float(em->speed_min, em->speed_max);

  float life = rand_float(em->life_min, em->life_max);
  float density = rand_float(em->density_min, em->density_max);

  p->x = bx;
  p->y = by;
  p->vx = cosf(a) * s;
  p->vy = sinf(a) * s;
  p->ax = 0.0f;
  p->ay = 0.0f;
  p->lifetime = life;
  p->max_lifetime = life;
  p->density = density;
  p->color = theme;
  p->alive = true;
  p->trail_head = 0;

  /* Seed the whole trail with the birth spot, otherwise the first few frames
   * would draw a trail streaking in from the top-left corner. */
  int cx = px_to_cx(bx), cy = px_to_cy(by);
  for (int t = 0; t < TRAIL_LEN; t++) {
    p->trail_cx[t] = cx;
    p->trail_cy[t] = cy;
  }
  return true;
}

/* Drip out the steady stream for one tick. The emitter's rate is "per
 * second", but a tick is much shorter, so a tick might earn less than one
 * whole particle. We keep the leftover fraction and carry it forward; once it
 * adds up to a full particle, one is born. Over time this hits the rate
 * exactly instead of always rounding down to nothing.
 * Returns how many were actually spawned this tick. */
static int emitter_tick(Emitter *em, float dt, int theme) {
  if (!em->active)
    return 0;

  em->rate_accum += em->rate * dt;
  int to_spawn = (int)em->rate_accum;
  if (to_spawn > MAX_BURST)
    to_spawn = MAX_BURST;
  em->rate_accum -= (float)to_spawn;

  int spawned = 0;
  for (int i = 0; i < to_spawn; i++)
    if (spawn_one(em, theme))
      spawned++;
  return spawned;
}

/* Spit out a clump of `count` particles all at once. Used by the 'b' key, the
 * auto-burst timer, and firework pops. Bursts always fire, even when the
 * steady stream is switched off. */
static int spawn_burst(const Emitter *em, int count, int theme) {
  if (count > MAX_BURST)
    count = MAX_BURST;
  int spawned = 0;
  for (int i = 0; i < count; i++)
    if (spawn_one(em, theme))
      spawned++;
  return spawned;
}

/* ── §9 render — four ways to draw the particles, plus a stats panel ── */

/* DensityField — the grid that powers the smoke/heat-map view.
 *
 * Instead of drawing each particle as a dot, the heat map smears them into a
 * grid of "how much stuff is here" values, one number per cell, then colours
 * each cell by how crowded it is. That gives the soft, glowing look.
 *
 * It keeps two grids:
 *   curr : this frame's totals. Wiped clean each frame, then every particle
 *          dabs a little blob into it. The drawing pass reads this.
 *   prev : last frame's totals, kept so we can spot cells that just went
 *          empty and blank only those — much cheaper than clearing the whole
 *          screen every frame, which would make the terminal flicker.
 *
 * It's one fixed-size grid sized for the biggest terminal we support, so it
 * sits in static memory and never needs the heap. There's a single shared
 * instance, g_field, cleared whenever the scene resets or the view changes so
 * an old frame can't bleed into a new one. */
typedef struct {
  float curr[GRID_MAX_H][GRID_MAX_W]; /* this frame's per-cell totals */
  float prev[GRID_MAX_H][GRID_MAX_W]; /* last frame's, for spotting cleared cells */
} DensityField;

static DensityField g_field;

/* Wipe both grids — used on reset and when switching into the heat-map view,
 * so the new frame starts from a blank field. */
static inline void density_field_reset(void) {
  memset(g_field.curr, 0, sizeof g_field.curr);
  memset(g_field.prev, 0, sizeof g_field.prev);
}

/* Wipe only this frame's grid, ready to be filled in again. Last frame's grid
 * is left alone so we can still compare against it. */
static inline void density_field_begin_frame(void) {
  memset(g_field.curr, 0, sizeof g_field.curr);
}

/* Save this frame's grid as last frame's, so next frame has something to
 * compare against when it looks for cells that went empty. */
static inline void density_field_snapshot(void) {
  memcpy(g_field.prev, g_field.curr, sizeof g_field.curr);
}

/* Dab one soft 3×3 blob into the grid, centred on a cell, so a single
 * particle spreads a little glow over its neighbours instead of lighting just
 * one cell. The centre gets the most, the corners the least, and it all adds
 * up to the particle's full weight. Anything off-grid is skipped. */
static void density_field_splat(int cx, int cy, float weight, int cols,
                                int rows) {
  static const float k[3][3] = {
      {0.0625f, 0.125f, 0.0625f},
      {0.125f, 0.25f, 0.125f},
      {0.0625f, 0.125f, 0.0625f},
  };
  for (int dy = -1; dy <= 1; dy++) {
    int gy = cy + dy;
    if (gy < 0 || gy >= rows || gy >= GRID_MAX_H)
      continue;
    for (int dx = -1; dx <= 1; dx++) {
      int gx = cx + dx;
      if (gx < 0 || gx >= cols || gx >= GRID_MAX_W)
        continue;
      g_field.curr[gy][gx] += weight * k[dy + 1][dx + 1];
    }
  }
}

/* The eight little arrows for the arrow view, one per direction of travel,
 * going clockwise starting from "moving right". '/' and '\' each show up
 * twice because the same stroke fits two opposite diagonals. */
static const char k_arrows[8] = {
    '>',  /* right      */
    '\\', /* right-down */
    'v',  /* down       */
    '/',  /* left-down  */
    '<',  /* left       */
    '\\', /* left-up    */
    '^',  /* up         */
    '/',  /* right-up   */
};

/* Pick the arrow that matches which way a particle is heading. */
static char velocity_arrow(float vx, float vy) {
  float angle = atan2f(vy, vx);
  float norm = angle + (float)M_PI;
  int oct = (int)(norm / (float)M_PI * 4.0f) % 8;
  return k_arrows[oct];
}

/* ── the heat-map view, done in three passes ── */

/* How much glow one particle adds to the heat map. Fast, young particles
 * glow brightest; a particle sitting still adds nothing; a fading one dims on
 * its own, so we don't need special dimming logic. Capped at 1. */
static inline float particle_heat_weight(const Particle *p) {
  float lf = p->lifetime / p->max_lifetime;
  float w = (particle_speed(p) / MAX_SPEED_NORM) * lf;
  return (w > 1.0f) ? 1.0f : w;
}

/* Pass 1 — blank out only the cells that lit up last frame but are empty now.
 * We compare last frame's grid to this one and erase just those, then save
 * this frame's grid for next time. Touching only the cells that changed keeps
 * the terminal from flickering. */
static void heatmap_clear_emptied_cells(WINDOW *w, int cols, int rows) {
  for (int r = 0; r < rows && r < GRID_MAX_H; r++)
    for (int c = 0; c < cols && c < GRID_MAX_W; c++)
      if (g_field.prev[r][c] > 0.01f && g_field.curr[r][c] < 0.01f)
        mvwaddch(w, r, c, ' ');
  density_field_snapshot();
}

/* Pass 2 — rebuild the grid from scratch: wipe it, then have every alive
 * particle dab its glow into it. Rebuilding (rather than adding onto last
 * frame) means particles that just died leave no ghost behind. */
static void heatmap_accumulate_density(int cols, int rows) {
  density_field_begin_frame();
  for (int i = 0; i < MAX_PARTICLES; i++) {
    const Particle *p = &g_particles[i];
    if (!p->alive)
      continue;
    density_field_splat(px_to_cx(p->x), px_to_cy(p->y), particle_heat_weight(p),
                        cols, rows);
  }
}

/* Pass 3 — actually draw the grid: for each lit cell, turn its value into a
 * ramp character in the theme's colour. Near-empty cells are skipped (pass 1
 * already blanked the ones that just emptied). */
static void heatmap_blit_density_field(WINDOW *w, int cols, int rows,
                                       int theme) {
  for (int r = 0; r < rows && r < GRID_MAX_H; r++) {
    for (int c = 0; c < cols && c < GRID_MAX_W; c++) {
      float d = g_field.curr[r][c];
      if (d < 0.01f)
        continue;
      int lvl = lut_index(d);
      if (lvl == 0)
        continue;
      attr_t attr = theme_attr(theme, lvl);
      wattron(w, attr);
      mvwaddch(w, r, c, k_ramp[lvl]);
      wattroff(w, attr);
    }
  }
}

/* Draw the heat-map view: blank emptied cells, rebuild the grid, draw it. */
static void render_heatmap(WINDOW *w, int cols, int rows, int theme) {
  heatmap_clear_emptied_cells(w, cols, rows);
  heatmap_accumulate_density(cols, rows);
  heatmap_blit_density_field(w, cols, rows, theme);
}

/* The other three views (glyph, trail, arrow), which all draw one character
 * per particle. They share the same "faster = brighter, dying = dimmer" logic
 * and only differ in which character lands on the screen: a density character
 * for glyph and trail, a direction arrow for arrow. Trail also paints a short
 * fading tail behind each particle first. */

/* ── helpers for drawing one particle ── */

/* Turn a particle's speed into a brightness level. Faster looks denser. We
 * never let it fall below 1, so even a still particle stays faintly visible
 * instead of disappearing. */
static inline int particle_ramp_level(const Particle *p) {
  float n = particle_speed(p) / MAX_SPEED_NORM;
  if (n > 1.0f)
    n = 1.0f;
  int lvl = lut_index(n);
  return (lvl < 1) ? 1 : lvl;
}

/* Work out the colour-and-brightness to draw a particle with: the theme
 * colour for its level, dimmed once it's into the last third of its life so
 * it visibly fades out as it dies. */
static inline attr_t particle_render_attr(const Particle *p, int theme,
                                          int level) {
  attr_t a = theme_attr(theme, level);
  float lf = p->lifetime / p->max_lifetime;
  if (lf < 0.30f)
    a = (a & ~A_BOLD) | A_DIM;
  return a;
}

/* Draw the fading tail: step back through the particle's recent positions,
 * newest first, each one a little dimmer than the last. We draw the tail
 * before the head so the head can be painted on top and stay sharp, instead
 * of being smudged by the next particle's tail. */
static void particle_draw_trail(WINDOW *w, const Particle *p, int cols,
                                int rows, int level, int theme) {
  for (int t = 0; t < TRAIL_LEN - 1; t++) {
    int tidx = (p->trail_head - 1 - t + TRAIL_LEN) % TRAIL_LEN;
    int tx = p->trail_cx[tidx];
    int ty = p->trail_cy[tidx];
    if (tx < 0 || tx >= cols || ty < 0 || ty >= rows)
      continue;
    int tlvl = level - t / 2;
    if (tlvl < 1)
      tlvl = 1;
    attr_t ta = theme_attr(theme, tlvl) | A_DIM;
    wattron(w, ta);
    mvwaddch(w, ty, tx, k_ramp[tlvl]);
    wattroff(w, ta);
  }
}

/* Draw the particle itself at its current cell: a direction arrow in arrow
 * view, otherwise a density character chosen by its speed. */
static inline void particle_draw_head(WINDOW *w, const Particle *p, int cx,
                                      int cy, int mode, int level,
                                      attr_t attr) {
  char ch =
      (mode == RENDER_ARROW) ? velocity_arrow(p->vx, p->vy) : k_ramp[level];
  wattron(w, attr);
  mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
  wattroff(w, attr);
}

/* Draw the glyph / trail / arrow views: for each alive particle, find its
 * cell, pick its brightness and colour, paint the tail first if we're in
 * trail view, then paint the particle on top. */
static void render_per_particle(WINDOW *w, int cols, int rows, int mode,
                                int theme) {
  for (int i = 0; i < MAX_PARTICLES; i++) {
    const Particle *p = &g_particles[i];
    if (!p->alive)
      continue;

    int cx = px_to_cx(p->x);
    int cy = px_to_cy(p->y);
    int lvl = particle_ramp_level(p);
    attr_t attr = particle_render_attr(p, theme, lvl);

    if (mode == RENDER_TRAIL)
      particle_draw_trail(w, p, cols, rows, lvl, theme);

    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows)
      continue;
    particle_draw_head(w, p, cx, cy, mode, lvl, attr);
  }
}

/* Draw the particles using whichever view is currently selected. */
static void render_particles(WINDOW *w, int cols, int rows, int mode,
                             int theme) {
  if (mode == RENDER_HEATMAP) {
    render_heatmap(w, cols, rows, theme);
  } else {
    render_per_particle(w, cols, rows, mode, theme);
  }
}

/* Draw the little stats panel in the bottom-left corner: live counts, timing,
 * speed and energy readouts, plus the current preset / theme / view / gravity
 * so you can always see what state the demo is in. Drawn last so it sits on
 * top of the particles. */
static void render_overlay(WINDOW *w, int cols, int rows, int particle_count,
                           float dt_sec, float sim_time, int spawn_rate,
                           float avg_vel, float energy, float gravity,
                           bool drag_on, int preset_id, int theme_id,
                           int render_mode, bool paused) {
  /* a small fixed-size box near the bottom-left */
  int panel_w = 28;
  int panel_h = 10;
  int ox = 1;
  int oy = rows - panel_h - 1;
  if (oy < 0)
    oy = 0;
  if (ox + panel_w > cols)
    return; /* screen too narrow for the panel — just skip it */

  /* top border */
  wattron(w, COLOR_PAIR(CP_BASE + theme_id * RAMP_N + 5) | A_DIM);
  mvwprintw(w, oy, ox, "+-- PARTICLE ENGINE ------+");

  /* the readouts */
  mvwprintw(w, oy + 1, ox, "| cnt  %5d / %-5d      |", particle_count,
            MAX_PARTICLES);
  mvwprintw(w, oy + 2, ox, "| dt   %7.4f s          |", dt_sec);
  mvwprintw(w, oy + 3, ox, "| time %7.2f s          |", sim_time);
  mvwprintw(w, oy + 4, ox, "| rate %5d /tick        |", spawn_rate);
  mvwprintw(w, oy + 5, ox, "| avgv %7.1f px/s       |", avg_vel);
  mvwprintw(w, oy + 6, ox, "| nrg  %9.1f          |", energy);

  /* current preset / theme / view / gravity */
  mvwprintw(w, oy + 7, ox, "| %-6s %-6s %-4s %4.0fg |",
            k_preset_names[preset_id], k_themes[theme_id].name,
            k_render_names[render_mode], gravity);

  /* drag + pause state, then the bottom border */
  mvwprintw(w, oy + 8, ox, "| drag:%-3s  %s              |",
            drag_on ? "ON " : "OFF", paused ? "PAUSED " : "running");
  mvwprintw(w, oy + 9, ox, "+-------------------------+");
  wattroff(w, COLOR_PAIR(CP_BASE + theme_id * RAMP_N + 5) | A_DIM);
}

/* ── §10 presets — fountain, fireworks, rainfall, explosion ── */

/* Each preset is just one set of emitter + physics settings that produces a
 * recognisable effect. They take the terminal size because the emitter's
 * position depends on where the edges and centre of the screen are.
 *
 *   fountain  — a tall upward spray that arcs back down; drops bounce at the
 *               floor and pile up. Drag is off so the jet reaches full height.
 *   fireworks — repeated pops of sparks; bursts overlap so the sky keeps
 *               shimmering. No steady stream, sparks die at the edges.
 *   rainfall  — drops falling from across the whole top edge; they die at the
 *               floor instead of bouncing, like real rain. Shown as arrows.
 *   explosion — a blast outward in every direction from the centre; strong
 *               drag makes it dissipate fast. Shown as an expanding heat map. */

/* Scene is defined in §11; presets fill it in, so we name it early here. */
typedef struct Scene Scene;

static void preset_fountain(Scene *s, int cols, int rows);
static void preset_fireworks(Scene *s, int cols, int rows);
static void preset_rainfall(Scene *s, int cols, int rows);
static void preset_explosion(Scene *s, int cols, int rows);

static void preset_apply(Scene *s, int id, int cols, int rows);

/* ── §11 scene — the whole simulation's state, plus tick and draw ── */

/* Scene — all the changeable state that doesn't live on individual particles.
 * It splits cleanly into two halves: the simulation half (what the physics
 * step reads and writes) and the render half (what the drawing step uses to
 * pick colours and glyphs). The render half never affects the physics, so the
 * particles behave the same no matter which theme or view is showing.
 *
 * The Scene knows nothing about ncurses; it's handed a window to draw into.
 * That keeps the physics separate from the display, so it could in principle
 * be run with no terminal at all. */
struct Scene {
  /* ── simulation half: the physics step reads and writes these ── */

  /* the nozzle that makes new particles; preset_apply rewrites it wholesale,
   * the 'e' key just flips its steady stream on or off */
  Emitter emitter;

  /* the global forces applied to every particle:
   *   gravity : downward pull; g/G change it, presets set it. Bigger means
   *             flatter arcs and a quicker fall.
   *   drag_on : is air resistance on? 'd' toggles it. Off = pure arcs. */
  float gravity;
  bool drag_on;

  /* what each edge does when a particle reaches it: true = the particle dies
   * there, false = it bounces back (losing some speed). Presets set these —
   * e.g. rain kills at the floor, a fountain bounces there. */
  bool floor_kills;
  bool ceiling_kills;
  bool wall_kills;

  /* when true the simulation freezes (space-bar toggles it). Drawing keeps
   * going so you see a still frame rather than a blank screen. */
  bool paused;

  /* the timer that fires bursts on its own, for fireworks and explosions:
   *   burst_interval : seconds between automatic bursts; 0 turns it off
   *   burst_timer    : counts down each tick and fires a burst at zero
   *   burst_count    : how many particles each automatic burst makes */
  float burst_interval;
  float burst_timer;
  int burst_count;

  /* numbers measured at the end of each tick and shown in the HUD; nothing
   * outside the tick should change them:
   *   dt_sec          : the length of the last step, in seconds
   *   simulation_time : seconds simulated so far (the 'r' key resets it)
   *   particle_count  : how many particles are alive right now
   *   spawn_rate_last : how many were born in the last tick
   *   avg_velocity    : average speed across the live particles
   *   energy_estimate : a rough total-energy gauge across all particles */
  float dt_sec;
  float simulation_time;
  int particle_count;
  int spawn_rate_last;
  float avg_velocity;
  float energy_estimate;

  /* which preset is loaded (p/P cycle through them). It lives on the
   * simulation side because switching it rewrites all the physics settings
   * above, not just the look. */
  int preset_id;

  /* ── render half: the drawing step reads these, the physics ignores them ── */

  /* which colour theme is showing ('t' cycles). Pure looks — particles behave
   * the same whatever theme is active. */
  int theme_id;

  /* which of the four views is showing — glyph, trail, heat map, or arrows
   * ('v' cycles). Same particles, just drawn differently. */
  int render_mode;
};

/* ── the four presets ── */

static void preset_fountain(Scene *s, int cols, int rows) {
  Emitter *em = &s->emitter;
  em->x = (float)pw(cols) * 0.5f;
  em->y = (float)ph(rows) - 2.0f;
  em->angle = -(float)M_PI * 0.5f; /* straight up */
  em->spread = (float)M_PI / 3.0f; /* a 60-degree spray */
  /* These speeds are tuned so the jet rises about half to all of a normal
   * terminal's height — tall enough to read clearly as a fountain. */
  em->speed_min = 380.0f;
  em->speed_max = 500.0f;
  em->life_min = 2.5f;
  em->life_max = 5.0f;
  em->density_min = 0.6f;
  em->density_max = 1.2f;
  /* this rate and lifetime settle at roughly 300 particles — a dense column */
  em->rate = 80.0f;
  em->rate_accum = 0.0f;
  em->spawn_width = 0.0f;
  em->active = true;

  s->gravity = 200.0f;
  /* Drag off on purpose: with it on the jet only rose about half as high.
   * Off, it reaches full height and reads clearly as a fountain. */
  s->drag_on = false;
  s->floor_kills = false; /* drops bounce at the floor */
  s->ceiling_kills = true;
  s->wall_kills = false;
  s->burst_interval = 0.0f;
  s->burst_count = 80;
  s->render_mode = RENDER_TRAIL;
  s->theme_id = 1; /* ocean */
}

static void preset_fireworks(Scene *s, int cols, int rows) {
  Emitter *em = &s->emitter;
  em->x = (float)pw(cols) * 0.5f;
  em->y = (float)ph(rows) - 2.0f;
  em->angle = -(float)M_PI * 0.5f; /* launched upward */
  em->spread = 2.0f * (float)M_PI; /* spray in every direction */
  em->speed_min = 200.0f;
  em->speed_max = 600.0f;
  em->life_min = 1.5f;
  em->life_max = 3.5f;
  em->density_min = 0.4f;
  em->density_max = 0.9f;
  em->rate = 0.0f; /* no steady stream — bursts only */
  em->rate_accum = 0.0f;
  em->spawn_width = 0.0f;
  em->active = false;

  s->gravity = 180.0f;
  s->drag_on = false;
  s->floor_kills = true;
  s->ceiling_kills = false;
  s->wall_kills = true;
  /* New bursts fire before the previous one's sparks have died, so a few
   * pops are always overlapping — that's what keeps the sky shimmering. */
  s->burst_interval = 1.8f;
  s->burst_count = 600;
  s->burst_timer = 0.5f; /* fire the first burst soon after switching in */
  s->render_mode = RENDER_GLYPH;
  s->theme_id = 0; /* fire */
}

static void preset_rainfall(Scene *s, int cols, int rows) {
  (void)rows;
  Emitter *em = &s->emitter;
  em->x = (float)pw(cols) * 0.5f;
  em->y = 0.0f;
  em->angle = (float)M_PI * 0.5f; /* falling straight down */
  em->spread = 0.25f;             /* barely any spread     */
  em->speed_min = 180.0f;
  em->speed_max = 380.0f;
  em->life_min = 1.5f;
  em->life_max = 3.5f;
  em->density_min = 0.5f;
  em->density_max = 1.0f;
  em->rate = 45.0f;
  em->rate_accum = 0.0f;
  em->spawn_width = (float)pw(cols); /* born anywhere along the top edge */
  em->active = true;

  s->gravity = 280.0f;
  s->drag_on = true;
  s->floor_kills = true;
  s->ceiling_kills = false;
  s->wall_kills = false;
  s->burst_interval = 0.0f;
  s->burst_count = 60;
  s->render_mode = RENDER_ARROW;
  s->theme_id = 1; /* ocean */
}

static void preset_explosion(Scene *s, int cols, int rows) {
  Emitter *em = &s->emitter;
  em->x = (float)pw(cols) * 0.5f;
  em->y = (float)ph(rows) * 0.5f;
  em->angle = 0.0f;
  em->spread = 2.0f * (float)M_PI; /* blast out in every direction */
  em->speed_min = 40.0f;
  em->speed_max = 480.0f;
  em->life_min = 0.8f;
  em->life_max = 2.8f;
  em->density_min = 0.3f;
  em->density_max = 1.5f;
  em->rate = 0.0f;
  em->rate_accum = 0.0f;
  em->spawn_width = 0.0f;
  em->active = false;

  s->gravity = 220.0f;
  s->drag_on = true;
  s->floor_kills = false;
  s->ceiling_kills = false;
  s->wall_kills = false;
  s->burst_interval = 3.5f;
  s->burst_count = 200;
  s->burst_timer = 0.2f;
  s->render_mode = RENDER_HEATMAP;
  s->theme_id = 0; /* fire */
}

static void preset_apply(Scene *s, int id, int cols, int rows) {
  /* park the burst timer far out so a half-finished countdown from the old
   * preset can't trigger a stray burst; each preset sets its own value */
  s->burst_timer = 9999.0f;

  switch (id) {
  case PRESET_FOUNTAIN:
    preset_fountain(s, cols, rows);
    break;
  case PRESET_FIREWORKS:
    preset_fireworks(s, cols, rows);
    break;
  case PRESET_RAINFALL:
    preset_rainfall(s, cols, rows);
    break;
  case PRESET_EXPLOSION:
    preset_explosion(s, cols, rows);
    break;
  default:
    break;
  }
  s->preset_id = id;
}

/* ── setting up and resetting the scene ── */

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->gravity = GRAVITY_DEFAULT;
  s->drag_on = true;
  s->paused = false;
  pool_clear();
  density_field_reset();
  preset_apply(s, PRESET_FOUNTAIN, cols, rows);
}

static void scene_reset(Scene *s, int cols, int rows) {
  pool_clear();
  density_field_reset();
  s->simulation_time = 0.0f;
  /* reload the current preset to put the emitter back to defaults */
  preset_apply(s, s->preset_id, cols, rows);
}

/* ── one simulation tick, broken into small steps ── */

/* Move the simulation's clock forward by one step. */
static inline void scene_advance_clock(Scene *s, float dt) {
  s->dt_sec = dt;
  s->simulation_time += dt;
}

/* Make whatever new particles this tick calls for: the steady stream from the
 * emitter, plus an automatic burst if the burst timer just ran out. Returns
 * the total spawned, for the stats panel. */
static int scene_spawn_step(Scene *s, float dt) {
  int spawned = emitter_tick(&s->emitter, dt, s->theme_id);

  if (s->burst_interval > 0.0f) {
    s->burst_timer -= dt;
    if (s->burst_timer <= 0.0f) {
      spawned += spawn_burst(&s->emitter, s->burst_count, s->theme_id);
      s->burst_timer = s->burst_interval;
    }
  }
  return spawned;
}

/* Move every particle one step, then bounce or kill the ones that hit an
 * edge. The average-speed and energy numbers get written straight into the
 * scene here so the stats panel can show them. */
static inline void scene_physics_step(Scene *s, float dt, int cols, int rows) {
  update_particles(dt, s->gravity, s->drag_on, &s->avg_velocity,
                   &s->energy_estimate);
  handle_collisions(cols, rows, s->floor_kills, s->ceiling_kills,
                    s->wall_kills);
}

/* Update the live particle count for the stats panel. */
static inline void scene_measure_stats(Scene *s) {
  s->particle_count = pool_alive_count();
}

/* One full simulation step: do nothing if paused, otherwise advance the
 * clock, spawn, move and collide everything, then refresh the stats. */
static void scene_tick(Scene *s, float dt, int cols, int rows) {
  if (s->paused)
    return;

  scene_advance_clock(s, dt);
  s->spawn_rate_last = scene_spawn_step(s, dt);
  scene_physics_step(s, dt, cols, rows);
  scene_measure_stats(s);
}

/* Draw the whole scene into the given window: the particles first, then the
 * stats panel on top. `alpha` (how far we are between physics steps) is taken
 * for a consistent signature but isn't used — at 60 fps smoothing the
 * positions with it makes no visible difference. */
static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows,
                       float alpha, float dt_sec) {
  (void)alpha;
  (void)dt_sec;

  render_particles(w, cols, rows, s->render_mode, s->theme_id);

  render_overlay(w, cols, rows, s->particle_count, s->dt_sec,
                 s->simulation_time, s->spawn_rate_last, s->avg_velocity,
                 s->energy_estimate, s->gravity, s->drag_on, s->preset_id,
                 s->theme_id, s->render_mode, s->paused);
}

/* ── §12 screen — the ncurses display layer ── */

/* Screen — the terminal display, the same setup the project's framework.c
 * uses. Each frame is built up off-screen and then pushed out in one write,
 * so the terminal never flickers. It just remembers the current size. */
typedef struct {
  int cols;
  int rows;
} Screen;

static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) {
  (void)s;
  endwin();
}

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

/* Draw the two HUD bars over the scene: a live status line along the top
 * (preset, theme, view, counts, timing, fps...) and a key-hint line along the
 * bottom. Each bar is painted with a solid colour across the full width so it
 * stays readable, and both go on after the particles so nothing bleeds
 * through them. The detailed stats box is handled separately in scene_draw. */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps,
                        float alpha, float dt_sec) {
  (void)dt_sec;

  erase();

  scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

  /* top bar: live status, rebuilt every frame */
  char status[200];
  snprintf(status, sizeof status,
           " PARTICLES   preset:%-9s  theme:%-6s  view:%-7s  "
           "alive:%4d/%-4d  g:%4.0f  drag:%-3s  %s   "
           "t:%5.1fs  %5.1f fps  %3d Hz ",
           k_preset_names[sc->preset_id], k_themes[sc->theme_id].name,
           k_render_names[sc->render_mode], sc->particle_count, MAX_PARTICLES,
           sc->gravity, sc->drag_on ? "ON " : "OFF",
           sc->paused ? "PAUSED " : "running", sc->simulation_time, fps,
           sim_fps);

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  for (int x = 0; x < s->cols; x++)
    mvaddch(0, x, ' ');
  mvprintw(0, 0, "%s", status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* bottom bar: the full list of keys you can press */
  const char *hints = " q:quit  spc:pause  b:burst  e:emitter  g/G:grav  "
                      "d:drag  r:reset  p/P:preset  t:theme  v:view  ]/[:Hz ";

  int hint_row = s->rows - 1;
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  for (int x = 0; x < s->cols; x++)
    mvaddch(hint_row, x, ' ');
  mvprintw(hint_row, 0, "%s", hints);
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §13 app — signals, resize, keyboard, and the main loop ── */

typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
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

static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  /* reload the preset so the emitter snaps to the new screen size */
  preset_apply(&app->scene, app->scene.preset_id, app->screen.cols,
               app->screen.rows);
  app->need_resize = 0;
}

/* Handle one keypress. Returns false only when it's time to quit. */
static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  Screen *sc = &app->screen;

  switch (ch) {
  /* quit / pause */
  case 'q':
  case 'Q':
  case 27:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;

  /* 'b' fires a burst right now; 'e' turns the steady stream on or off */
  case 'b':
  case 'B':
    spawn_burst(&s->emitter, s->burst_count, s->theme_id);
    break;
  case 'e':
  case 'E':
    s->emitter.active = !s->emitter.active;
    break;

  /* stronger / weaker gravity; can go all the way to zero for floaty clouds */
  case 'g':
    s->gravity += GRAVITY_STEP;
    if (s->gravity > GRAVITY_MAX)
      s->gravity = GRAVITY_MAX;
    break;
  case 'G':
    s->gravity -= GRAVITY_STEP;
    if (s->gravity < GRAVITY_MIN)
      s->gravity = GRAVITY_MIN;
    break;

  /* turn drag on/off; off lets particles fly in clean arcs */
  case 'd':
  case 'D':
    s->drag_on = !s->drag_on;
    break;

  /* clear everything and start the current preset over */
  case 'r':
  case 'R':
    scene_reset(s, sc->cols, sc->rows);
    break;

  /* step to the next/previous preset, clearing the screen so it starts fresh */
  case 'p':
    pool_clear();
    density_field_reset();
    preset_apply(s, (s->preset_id + 1) % PRESET_COUNT, sc->cols, sc->rows);
    break;
  case 'P':
    pool_clear();
    density_field_reset();
    preset_apply(s, (s->preset_id + PRESET_COUNT - 1) % PRESET_COUNT, sc->cols,
                 sc->rows);
    break;

  /* next colour theme — instant, since all the colours are already set up */
  case 't':
  case 'T':
    s->theme_id = (s->theme_id + 1) % N_THEMES;
    break;

  /* next view; clear the heat-map grid when switching to it so no old frame
   * lingers */
  case 'v':
  case 'V':
    s->render_mode = (s->render_mode + 1) % RENDER_COUNT;
    if (s->render_mode == RENDER_HEATMAP)
      density_field_reset();
    break;

  /* faster / slower physics updates: higher is smoother but uses more CPU */
  case ']':
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX)
      app->sim_fps = SIM_FPS_MAX;
    break;
  case '[':
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN)
      app->sim_fps = SIM_FPS_MIN;
    break;

  default:
    break;
  }
  return true;
}

/* The main loop. The physics runs at a steady rate no matter how fast the
 * screen draws, and we draw as often as we can up to a frame cap. This is the
 * same loop shape the project's framework.c uses; see there for the full
 * walkthrough. */
int main(void) {
  srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
  atexit(cleanup);

  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    /* the window was resized — rebuild for the new size */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* how long since the last frame */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS; /* cap it, so a long pause can't fast-forward */

    /* Run as many fixed-size physics steps as the elapsed time covers. Any
     * leftover time is kept and used next frame, so the simulation stays in
     * step with the clock no matter how often we draw. */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec, app->screen.cols, app->screen.rows);
      sim_accum -= tick_ns;
    }

    /* how far we are between physics steps (handed to the renderer) */
    float alpha = (float)sim_accum / (float)tick_ns;

    /* recompute the displayed fps every so often */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* Sleep to hit the frame cap, and do it before drawing so the time spent
     * writing to the terminal isn't counted against the next frame's budget. */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

    /* draw the frame and push it to the terminal */
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps, alpha,
                dt_sec);
    screen_present();

    /* handle one keypress, if any */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
