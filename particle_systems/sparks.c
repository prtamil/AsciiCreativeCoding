/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sparks.c — fast flying sparks that arc under gravity, bounce off the
 *            floor, and drag a short fading streak behind them.
 *
 * Sparks shoot out of an emitter, curve down as gravity pulls on them,
 * and bounce off the floor (the row just above the key-hint bar),
 * losing a bit of energy each bounce. The bright dot you watch is the
 * spark's "head"; the dim tail behind it is just where the spark was
 * a moment ago, so it reads like motion blur. Ten built-in effects
 * (welder, grinder, campfire, Tesla coil, sparkler, ground spinner,
 * and so on) all run on the same code — what makes them look different
 * is one row of numbers in the pattern_params[] table.
 *
 * The closest sibling is embers.c: same skeleton, but embers rise and
 * cool with no bounce, while sparks fall fast and bounce. Read embers.c
 * first; this is the fast-and-bouncy version. fountain.c also bounces;
 * fireworks.c is the radial-burst cousin of the TESLA effect here.
 *
 * Section map:
 *   §1 config — constants, the ten pattern presets, the colour themes
 *   §2 clock  — a steady timer and a sleep
 *   §3 color  — the cool-to-hot colour ramp
 *   §4 spark  — one spark: its state, physics, and trail
 *   §5 scene  — the spark pool: spawn, simulate, draw
 *   §6 screen — ncurses setup, drawing, and the HUD
 *   §7 app    — signals and the main loop
 *
 * Keys:
 *   q / ESC    quit                 space      pause / resume
 *   r          clear and restart    n / N      next / prev effect
 *   p / P      previous effect      t / T      next / prev theme
 *   + / =      faster               -          slower
 *   ] / [      raise / lower tick rate
 *   w / W      shift emitter right / left
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/sparks.c \
 *       -o sparks -lncurses -lm
 *
 * References (ideas the code can't show you on its own):
 *   Reeves (1983), "Particle Systems", ACM TOG 2(2) — the spawn /
 *     simulate / cull / draw pool design, and the idea of one engine
 *     plus a small table of constants per effect.
 *   Hertz (1882) — the bounciness number (restitution) used at the floor.
 *   Bourg & Bywalec (2013), "Physics for Game Developers" — gravity +
 *     drag motion, and the rotating-circle math behind the SPINNER.
 *   Millington (2010), "Game Physics Engine Development", Ch. 6 — the
 *     frame-rate-independent drag and the "let tired sparks sleep" idea.
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config ── */

enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,

  SPEED_MIN = 1,
  SPEED_DEF = 8,
  SPEED_MAX = 64,

  MAX_SPARKS = 800,
  TRAIL_LEN = 3, /* how many past positions each spark remembers */

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* Colour-pair slots. The HUD and hint bar get fixed colours; the
   * eight ramp slots (PAIR_HEAT_BASE + 0..7) go from dim to bright. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_HEAT_BASE = 3,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* How far w/W slides the emitter sideways, in cells. */
#define EMITTER_SHIFT_STEP 8.0f

/* When a spark is moving this slowly after a bounce, we just kill it.
 * It no longer has enough energy to make a bounce you'd notice, and
 * left alone it would jitter against the floor forever. The numbers
 * are tuned by eye: high enough to retire a sliding spark within about
 * a second, low enough that real bounces still happen. (cells/sec) */
#define SETTLE_VY 6.0f
#define SETTLE_VX 10.0f

/* If a spark flies this many cells above the top of the screen it's
 * never coming back (drag wins), so we retire it early and free the slot. */
#define TOP_KILL_MARGIN 4.0f

/* SPARKLERS: a parent spark "pops" partway through its life into a few
 * smaller sparks that scatter in all directions from where the parent
 * was, slower and shorter-lived. That second burst is what gives a
 * sparkler its crackle. */
#define SPARKLER_SPLIT_AGE_FRAC 0.40f   /* pop at 40% of the parent's life */
#define SPARKLER_CHILDREN_PER_SPLIT 3
#define SPARKLER_CHILD_SPEED_MIN 14.0f
#define SPARKLER_CHILD_SPEED_MAX 26.0f
#define SPARKLER_CHILD_LIFE_MIN 0.3f
#define SPARKLER_CHILD_LIFE_MAX 0.7f

/* SPINNER: sparks fly off the rim of a spinning wheel. The launch
 * points ride a small circle around the centre, the wheel turns at
 * SPINNER_OMEGA, and sparks leave sideways (along the rim, not straight
 * out) so it reads as a real pinwheel rather than a centred blast. */
#define SPINNER_OMEGA 14.0f          /* turn speed, ~2.2 spins/sec */
#define SPINNER_RADIUS 5.0f          /* wheel radius, in cells     */
#define SPINNER_TANGENT_JITTER 0.25f /* a little wobble on the launch angle */

/* The ten effects, in the order n/p cycle through them. */
typedef enum {
  PATTERN_WELDING = 0,
  PATTERN_GRINDER = 1,
  PATTERN_CAMPFIRE = 2,
  PATTERN_TESLA = 3,
  PATTERN_SPARKLERS = 4,
  PATTERN_GROUND_SPINNER = 5,
  PATTERN_FLOWER_POT = 6,
  PATTERN_CHRYSANTHEMUM = 7,
  PATTERN_WILLOW = 8,
  PATTERN_WATERFALL = 9,
  N_PATTERNS = 10,
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_WELDING:
    return "WELDING  ";
  case PATTERN_GRINDER:
    return "GRINDER  ";
  case PATTERN_CAMPFIRE:
    return "CAMPFIRE ";
  case PATTERN_TESLA:
    return "TESLA    ";
  case PATTERN_SPARKLERS:
    return "SPARKLERS";
  case PATTERN_GROUND_SPINNER:
    return "SPINNER  ";
  case PATTERN_FLOWER_POT:
    return "FLOWERPOT";
  case PATTERN_CHRYSANTHEMUM:
    return "CHRYSANTH";
  case PATTERN_WILLOW:
    return "WILLOW   ";
  case PATTERN_WATERFALL:
    return "WATERFALL";
  default:
    return "?        ";
  }
}

/* Where on the screen sparks are born. Four spots cover all ten
 * effects — each effect is placed where it has room to do its thing. */
typedef enum {
  EMIT_CENTER_LEFT,   /* left edge, mid-height — WELDING jets rightward  */
  EMIT_CENTER_BOTTOM, /* bottom-centre — ground effects rise and fall back
                         (GRINDER, CAMPFIRE, FLOWERPOT, WILLOW)          */
  EMIT_CENTER,        /* dead centre — all-directions bursts use the whole
                         screen (TESLA, SPARKLERS, SPINNER, CHRYSANTHEMUM) */
  EMIT_CENTER_TOP,    /* top-centre — WATERFALL rains down the full height */
} EmitPos;

/*
 * PatternParams — the recipe for one effect: where sparks start, how
 * fast and which way they fly, how heavy they are, how bouncy, and how
 * long they live. There's one of these per effect; switching effects
 * just means reading a different row. This is the whole trick of the
 * file: the simulation code never asks "which effect is this?" — it
 * just reads these numbers and does what they say.
 *
 * (Two effects need a little extra code the numbers can't express:
 * SPINNER launches sparks off a spinning rim, and SPARKLERS pops each
 * spark into smaller ones partway through its life. Everything else is
 * pure data.)
 *
 * The design — one engine plus a table of constants per effect — comes
 * from Reeves' particle-systems paper (1983, §4).
 *
 * FIELDS:
 *   target_sparks  How many sparks the effect tries to keep alive at
 *                  once. We top up toward this each frame (gradually,
 *                  so resuming after a pause doesn't dump them all at
 *                  once). Bigger means a denser, busier look. Ranges
 *                  from 200 (SPARKLERS — leaves room in the pool for
 *                  the extra sparks it spawns) up to 400 (FLOWERPOT, a
 *                  thick fountain).
 *
 *   emitter        Which of the four launch spots to use (see EmitPos).
 *
 *   emit_x_jitter  How much to randomly fuzz the birth position, in
 *   emit_y_jitter  cells, left/right and up/down. Without this the
 *                  sparks all pour out of one exact point and look
 *                  mechanical. CAMPFIRE uses a wide-but-flat fuzz
 *                  (3.0 / 0.5) to spread out like the base of a flame;
 *                  most effects use about 0.5.
 *
 *   speed_min      How fast a new spark flies, in cells/sec, picked at
 *   speed_max      random between these two. Faster sparks travel
 *                  farther before gravity and drag slow them down.
 *                  Ranges from 28-50 (SPARKLERS, a gentle fizz) up to
 *                  100-150 (FLOWERPOT, a tall fountain).
 *
 *   angle_min      Which directions sparks can launch, as an angle in
 *   angle_max      radians, picked at random in this range. 0 points
 *                  right, -π/2 points straight up, +π/2 straight down,
 *                  ±π points left (y grows downward on screen). A full
 *                  2π-wide range fires in every direction (TESLA,
 *                  SPARKLERS, SPINNER, CHRYSANTHEMUM); narrow ranges
 *                  shape a cone — e.g. WELDING fires roughly rightward,
 *                  WATERFALL roughly downward, FLOWERPOT roughly up.
 *                  SPINNER ignores this; its launch angle comes from
 *                  the wheel instead.
 *
 *   gravity        Downward pull, in cells/sec² (always positive since
 *                  y grows downward). Applied to a spark's vertical
 *                  speed every frame. Low gravity (18, CHRYSANTHEMUM)
 *                  lets sparks hang in the air; high gravity (100,
 *                  WILLOW) yanks them straight back down.
 *
 *   drag_coeff     How quickly air resistance bleeds off speed. We use
 *                  it as a fading factor each frame, written so the
 *                  result is the same no matter the frame rate (unlike
 *                  a plain "multiply speed by 0.99", which would change
 *                  if the tick rate changed). Low (0.15, WILLOW — sparks
 *                  coast far) to high (0.55, CAMPFIRE — sparks settle
 *                  fast). The fade-factor form is from Millington (2010)
 *                  Ch. 6.
 *
 *   restitution    How bouncy the floor is, 0 to 1. After a downward
 *                  hit the upward speed is this fraction of the old
 *                  speed: 0 means the spark sticks dead on first touch,
 *                  1 would bounce forever. This is the classic
 *                  "coefficient of restitution" from Hertz (1882).
 *                  Range: 0.30 (WILLOW thuds) to 0.65 (TESLA, WATERFALL
 *                  stay bouncy).
 *
 *   floor_friction How much sideways speed survives a bounce, as a
 *                  fraction. A simple per-bounce scaling rather than
 *                  true sliding friction. Range 0.60-0.78.
 *
 *   life_min       How long a spark lives, in seconds, picked at random
 *   life_max       in this range. Lifetime also drives the colour fade:
 *                  a fresh spark is white-hot, an old one is dim. Short
 *                  (0.4-1.0, SPARKLERS crackle) to long (3.0-4.5,
 *                  CHRYSANTHEMUM's slow bloom).
 */
typedef struct {
  int target_sparks;
  EmitPos emitter;
  float emit_x_jitter;
  float emit_y_jitter;
  float speed_min, speed_max;
  float angle_min, angle_max;
  float gravity;
  float drag_coeff;
  float restitution;
  float floor_friction;
  float life_min, life_max;
} PatternParams;

/* Quick reminder on launch angles below (in radians):
 *   0      points right        -π/2 (≈ -1.57)  points straight up
 *   ±π     points left         +π/2 (≈ +1.57)  points straight down
 * So an up-and-right spray wants angles between -π/2 and 0, and a
 * "fires everywhere" spray wants a full 2π-wide range. */
static const PatternParams pattern_params[N_PATTERNS] = {
    /* Columns, in order:
     *  target  emitter  jitter_x jitter_y  speed_min speed_max
     *  angle_min angle_max  gravity drag  bounce friction  life_min life_max
     */
    /* WELDING       */ {380, EMIT_CENTER_LEFT, 1.0f, 1.0f, 55.0f, 90.0f,
                         -0.55f, 0.55f, 78.0f, 0.40f, 0.55f, 0.78f, 1.0f, 2.2f},
    /* GRINDER       */
    {320, EMIT_CENTER_BOTTOM, 0.6f, 0.6f, 72.0f, 110.0f, -1.30f, -0.40f, 92.0f,
     0.30f, 0.50f, 0.70f, 0.8f, 1.8f},
    /* CAMPFIRE      */
    {220, EMIT_CENTER_BOTTOM, 3.0f, 0.5f, 34.0f, 56.0f, -2.20f, -0.94f, 36.0f,
     0.55f, 0.40f, 0.60f, 1.5f, 3.0f},
    /* TESLA         */
    {260, EMIT_CENTER, 0.4f, 0.4f, 55.0f, 85.0f, -(float)M_PI, (float)M_PI,
     26.0f, 0.20f, 0.65f, 0.78f, 0.5f, 1.2f},
    /* SPARKLERS     */
    {200, EMIT_CENTER, 0.3f, 0.3f, 28.0f, 50.0f, -(float)M_PI, (float)M_PI,
     60.0f, 0.50f, 0.35f, 0.65f, 0.4f, 1.0f},
    /* SPINNER       */
    {380, EMIT_CENTER, 0.5f, 0.5f, 90.0f, 130.0f, -(float)M_PI, (float)M_PI,
     50.0f, 0.30f, 0.55f, 0.75f, 1.0f, 2.0f},
    /* FLOWERPOT     */
    {400, EMIT_CENTER_BOTTOM, 0.5f, 0.5f, 100.0f, 150.0f,
     -(float)(M_PI / 2 + 0.6), -(float)(M_PI / 2 - 0.6), 70.0f, 0.25f, 0.45f,
     0.70f, 2.0f, 3.5f},
    /* CHRYSANTHEMUM */
    {320, EMIT_CENTER, 0.5f, 0.5f, 50.0f, 75.0f, -(float)M_PI, (float)M_PI,
     18.0f, 0.18f, 0.40f, 0.65f, 3.0f, 4.5f},
    /* WILLOW        */
    {280, EMIT_CENTER_BOTTOM, 0.5f, 0.5f, 50.0f, 80.0f,
     -(float)(M_PI / 2 + 1.2), -(float)(M_PI / 2 - 1.2), 100.0f, 0.15f, 0.30f,
     0.60f, 2.8f, 4.5f},
    /* WATERFALL     */
    {380, EMIT_CENTER_TOP, 0.5f, 0.5f, 35.0f, 65.0f, (float)(M_PI / 2 - 0.35),
     (float)(M_PI / 2 + 0.35), 45.0f, 0.25f, 0.65f, 0.78f, 1.8f, 3.0f},
};

/*
 * Themes — each one is an 8-colour ramp running from dim (slot 0) to
 * bright (slot 7), used to colour a spark by how hot it still is.
 * Each theme sticks to one colour family so flipping themes with t/T
 * is obviously different. NOVA is the odd one out — it sweeps through
 * several hues (blue → magenta → orange → yellow → white) like a real
 * supernova.
 *
 *   MATRIX   green          FOREST   olive → leaf → gold
 *   FIRE     coal → white   DESERT   sand → cream
 *   OCEANIC  blue → teal    NEON     purple → hot pink
 *   ICE      frost → white  ECLIPSE  purple → crimson
 *   NOVA     full spectrum  MONO     grayscale
 *
 * Even the dimmest colour in each ramp is kept out of the darkest part
 * of the palette, so a faded tail stays visible on a black terminal
 * (the "Theme Palette Brightness" rule in CLAUDE.md).
 */
typedef struct {
  const char *name; /* shown in the HUD */
  short heat[8];    /* dim (oldest/coolest) → bright (newest/hottest) */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name        heat[0..7]  (cool dim → hot bright)                  */
    {"MATRIX", {28, 34, 40, 46, 82, 118, 154, 190}}, /* phosphor green        */
    {"FIRE", {52, 88, 124, 160, 202, 208, 220, 231}}, /* coal→flame→white-hot */
    {"OCEANIC", {24, 25, 32, 38, 44, 80, 116, 152}}, /* deep blue→teal        */
    {"NEON", {54, 92, 128, 165, 201, 207, 213, 219}},   /* purple→hot pink   */
    {"MONO", {240, 244, 247, 250, 252, 253, 254, 255}}, /* grayscale */
    {"ICE", {117, 153, 159, 195, 231, 251, 253, 255}},  /* pale frost→white  */
    {"NOVA", {27, 99, 165, 201, 208, 220, 226, 231}}, /* SPECTRUM (multi-hue) */
    {"FOREST", {58, 64, 70, 106, 142, 148, 184, 220}}, /* olive→leaf→gold */
    {"DESERT", {94, 130, 137, 173, 215, 222, 228, 230}}, /* dune→sand→cream */
    {"ECLIPSE",
     {53, 54, 89, 125, 161, 197, 204, 209}}, /* void purple→crimson   */
};

/*
 * Which character to draw, by heat. The head uses solid-looking marks
 * (* + # @) so it reads as a bright point; the trail uses lighter ones
 * (, . :) so it reads as a fading streak behind. Both go from faint
 * (slot 0) to bold (slot 7).
 */
static const char HEAD_GLYPHS[8] = {'`', '.', ':', ';', '*', '+', '#', '@'};
static const char TRAIL_GLYPHS[8] = {'`', '.', '.', ',', ':', ';', '+', '*'};

/* ── §2 clock ── */

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

/* ── §3 color ── */

static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  if (COLORS >= 256) {
    const Theme *t = &themes[idx];
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_HEAT_BASE + i), t->heat[i], -1);
  } else {
    /* On a basic 8-colour terminal, fake the ramp with red→yellow→white. */
    static const short fb[8] = {
        COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_YELLOW,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,
    };
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_HEAT_BASE + i), fb[i], -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow status bar */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan key-hint bar */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
  theme_apply(0);
}

/* ── §4 spark ── */

/*
 * Spark — one flying spark, plus the short memory of where it just was
 * (that memory is what we draw as the fading trail).
 *
 * Every spark lives in one slot of a fixed array set up once at startup
 * — there's no allocation while the demo runs. The `active` flag says
 * whether a slot is in use; to make a new spark we just scan for the
 * first free slot. Keeping all of a spark's numbers together in one
 * struct (rather than in separate arrays) is friendly to the CPU cache,
 * since the simulation touches them together every frame.
 *
 * A spark's life:
 *   born   — placed at the emitter (with a little random fuzz), aimed
 *            in some direction at some speed, trail seeded at its start.
 *   each frame — remember the current spot, then move: gravity pulls it
 *            down, drag bleeds off speed, position advances, it ages.
 *   bounce — if it hits the floor heading down, it flips upward (losing
 *            energy) and loses a little sideways speed. If it's barely
 *            moving afterward we retire it instead of letting it jitter.
 *   pop    — SPARKLERS only: partway through its life a spark bursts
 *            into a few smaller ones.
 *   dies   — when it's too old, flies off-screen, or settles. Its slot
 *            is then free for a new spark.
 *
 * The pool-and-lifetime design is from Reeves (1983); the bounciness
 * number is Hertz's coefficient of restitution (1882); the gravity +
 * drag motion is standard game physics (Bourg & Bywalec 2013).
 *
 * FIELDS:
 *   x, y          Where the spark is, in cells. y grows downward (the
 *                 ncurses convention), so gravity pushes y up in value.
 *                 Kept as floats for smooth motion; drawing rounds to
 *                 the nearest cell.
 *
 *   vx, vy        How fast and which way the spark is moving, in
 *                 cells/sec. Set from its launch angle and speed at
 *                 birth, then changed every frame by gravity, drag, and
 *                 bounces.
 *
 *   age, life     Both in seconds. `age` counts up every frame; `life`
 *                 is rolled once at birth and never changes. The spark
 *                 dies when age reaches life, and its colour fades from
 *                 hot to dim as it ages.
 *
 *   trail_x[]     The last few positions the spark held, oldest first.
 *   trail_y[]     We fill these with the birth position so a brand-new
 *                 spark draws its trail right on top of itself (no
 *                 streak) until it actually moves. Drawn as the fading
 *                 tail behind the head.
 *
 *   active        Whether this slot holds a live spark. Dead slots are
 *                 skipped everywhere and reused by the next new spark.
 *
 *   has_split     SPARKLERS only. Marks a spark that has already popped
 *                 into smaller ones (or is itself one of those smaller
 *                 ones), so it won't pop again — that caps the chain at
 *                 two generations. It also lets the spawn logic count
 *                 only the original sparks, so the extra ones don't
 *                 crowd out new launches. Always false for other effects.
 */
typedef struct {
  float x, y;
  float vx, vy;
  float age, life;
  float trail_x[TRAIL_LEN];
  float trail_y[TRAIL_LEN];
  bool active;
  bool has_split;
} Spark;

/* A tiny, fast random-number generator. Same constants as embers.c so
 * the sibling demos share the same "feel" of randomness. */
static inline uint32_t lcg_next(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}
static inline float lcg_unit(uint32_t *st) {
  return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/* ── Random sampling ── */

/* A random number somewhere between lo and hi. Everything else builds on this. */
static inline float sample_uniform_in_range(uint32_t *rng, float lo, float hi) {
  return lo + lcg_unit(rng) * (hi - lo);
}

/* A fully random direction (any angle). Used when SPARKLERS children
 * scatter every which way. */
static inline float sample_random_phase_2pi(uint32_t *rng) {
  return lcg_unit(rng) * 2.0f * (float)M_PI - (float)M_PI;
}

/* ── Launch and initial state ── */

/* Turn a speed and a direction into left/right and up/down motion.
 * (0 = right, -π/2 = up, +π/2 = down, matching the rest of the file.) */
static inline void velocity_from_polar(float speed, float angle, float *vx,
                                       float *vy) {
  *vx = speed * cosf(angle);
  *vy = speed * sinf(angle);
}

/* Start a new spark's trail sitting exactly where the spark is, so it
 * shows no streak until it actually moves. Skip this and a fresh spark
 * would draw a trail back to wherever the slot's previous spark died. */
static inline void spark_seed_trail_at_position(Spark *e, float x, float y) {
  for (int k = 0; k < TRAIL_LEN; k++) {
    e->trail_x[k] = x;
    e->trail_y[k] = y;
  }
}

/* Set up a fresh spark's position, motion, and age from a speed and
 * direction. Leaves `active` and `has_split` alone — the caller sets
 * those, since they depend on whether this is an original spark or a
 * popped child. */
static inline void spark_init_kinematics(Spark *e, float x, float y,
                                         float speed, float angle, float life) {
  e->x = x;
  e->y = y;
  velocity_from_polar(speed, angle, &e->vx, &e->vy);
  e->age = 0.0f;
  e->life = life;
}

/* ── Physics, one frame at a time ── */

/* The factor speed gets multiplied by this frame to model air drag.
 * Written so the slowdown is the same regardless of frame rate — a
 * plain "speed *= 0.99" would drift if the tick rate changed.
 * Millington (2010) Ch. 6. */
static inline float compute_drag_factor_continuous(float drag_coeff, float dt) {
  return expf(-drag_coeff * dt);
}

/* Push the current position into the trail history (dropping the oldest
 * one). Done before the spark moves, so the newest trail point is
 * exactly where the spark is now and the streak connects to the head. */
static inline void spark_shift_trail_history(Spark *e) {
  for (int k = 0; k < TRAIL_LEN - 1; k++) {
    e->trail_x[k] = e->trail_x[k + 1];
    e->trail_y[k] = e->trail_y[k + 1];
  }
  e->trail_x[TRAIL_LEN - 1] = e->x;
  e->trail_y[TRAIL_LEN - 1] = e->y;
}

/* Advance one spark by one frame: update its speed first (gravity pulls
 * it down, drag bleeds speed off), then move it using that new speed,
 * then age it. Updating speed before position is what keeps the motion
 * stable (Bourg 2013 Ch. 2 / Millington Ch. 6). */
static inline void integrate_spark_semi_implicit_euler(Spark *e, float gravity,
                                                       float drag_factor,
                                                       float dt) {
  e->vy += gravity * dt;
  e->vx *= drag_factor;
  e->vy *= drag_factor;
  e->x += e->vx * dt;
  e->y += e->vy * dt;
  e->age += dt;
}

/* True when the spark has dipped to or below the floor while heading
 * down — that's the moment to bounce it. */
static inline bool spark_crossed_floor_descending(const Spark *e,
                                                  float floor_y) {
  return e->y >= floor_y && e->vy > 0.0f;
}

/* Bounce a spark off the floor: lift it back above the floor, flip it
 * upward keeping `restitution` of its speed (the bounciness number),
 * and shave its sideways speed by `friction`. Returns true if it ended
 * up barely moving — the caller should then retire it, so it doesn't
 * sit there jittering against the floor forever. */
static inline bool reflect_spark_off_floor_and_test_settle(Spark *e,
                                                           float floor_y,
                                                           float restitution,
                                                           float friction) {
  float overshoot = e->y - floor_y;
  e->y = floor_y - overshoot; /* mirror it back above the floor   */
  if (e->y > floor_y)
    e->y = floor_y;             /* guard against rounding past it  */
  e->vy = -e->vy * restitution; /* flip upward, lose some energy   */
  e->vx *= friction;            /* lose some sideways speed too    */
  return fabsf(e->vy) < SETTLE_VY && fabsf(e->vx) < SETTLE_VX;
}

/* Should this spark go away? Yes if it's lived out its life, drifted
 * off the sides, or shot up off the top of the screen. */
static inline bool spark_should_die(const Spark *e, int cols) {
  if (e->age >= e->life)
    return true;
  if (e->x < -2.0f || e->x > (float)(cols + 2))
    return true;
  if (e->y < -TOP_KILL_MARGIN)
    return true;
  return false;
}

/* ── §5 scene — pool, tick, draw ── */

/*
 * Scene — all the state that changes while the demo runs, in one place.
 *
 * The fields split into two groups by who touches them. The simulation
 * (scene_tick) reads and writes the first group: which effect is
 * playing, the emitter shift, the spinner's angle, the random-number
 * state, the screen size, and the spark pool itself. The drawing code
 * only needs the current theme. They're grouped this way partly to make
 * the code easier to follow and partly because the simulation runs every
 * frame, so keeping its fields close together is friendlier to the cache.
 *
 * The Scene knows nothing about ncurses: the simulation writes here, the
 * drawing code reads from here. That clean split means you could run the
 * simulation with no terminal at all (say, to profile it).
 */
typedef struct {
  /* ── Simulation: the tick reads and writes these ── */

  /* When true, the simulation freezes but drawing keeps going, so you
   * see a still frame with every spark held mid-flight. Toggled by space. */
  bool paused;

  /* A time multiplier for slow-mo / fast-forward. It just speeds up or
   * slows down simulated time — the physics itself is unchanged. +/=
   * doubles it, - halves it, within SPEED_MIN/MAX. */
  int speed;

  /* Which effect is playing (an index into pattern_params[]). n/p cycle
   * through them. Switching doesn't wipe the screen — old sparks fade
   * out under the old rules while new ones launch under the new ones;
   * press r to clear instantly. This is the one field both the
   * simulation and the drawing code read. */
  Pattern current_pattern;

  /* How far the whole show is shifted sideways, set by w/W. Lets you
   * slide any effect left or right without changing what it is. Survives
   * effect switches; r resets it to 0. */
  float emitter_offset_x;

  /* The SPINNER wheel's current angle, in radians. It turns a steady
   * amount each frame (wrapped so it never grows without bound) and
   * drives both where SPINNER sparks launch from and the spinning axle
   * drawn at the hub. It keeps turning even when SPINNER isn't the
   * current effect, so the wheel picks up smoothly when you switch to
   * it. The circle math is from Bourg & Bywalec Ch. 8. */
  float spinner_phase;

  /* The random-number state for this scene. Seeded from the clock at
   * startup and re-rolled on r. Everything random pulls from here:
   * launch fuzz, speeds, lifetimes, SPARKLERS scatter, SPINNER wobble.
   * No globals — the whole generator is this one 32-bit value. */
  uint32_t rng;

  /* The terminal size, remembered here so the hot path never has to ask
   * ncurses for it. Refreshed at startup and whenever the window
   * resizes. Used for the emitter position, off-screen checks, and
   * drawing bounds. */
  int rows, cols;

  /* The spark pool — a fixed array set up once, never resized or
   * reallocated. Spawning a spark scans for the first free slot. See
   * the Spark struct for what each slot holds. */
  Spark sparks[MAX_SPARKS];

  /* ── Rendering: only the drawing code reads this ── */

  /* Which colour theme is active (an index into themes[]). t/T cycle
   * through them. Purely cosmetic — sparks behave the same whatever
   * theme is showing. Changing it repaints the colour ramp. */
  int current_theme;
} Scene;

/* Sparks bounce off the floor, which sits one row above the key-hint
 * bar (the bar is on the very bottom row). */
static inline float scene_floor_y(const Scene *s) {
  return (float)(s->rows - 2);
}

static int spark_pool_find_inactive(Scene *s) {
  for (int i = 0; i < MAX_SPARKS; i++)
    if (!s->sparks[i].active)
      return i;
  return -1;
}

static void scene_clear_sparks(Scene *s) {
  for (int i = 0; i < MAX_SPARKS; i++)
    s->sparks[i].active = false;
}

/* Work out where sparks are born for the current effect — one of four
 * spots (see EmitPos), shifted sideways by the w/W offset. */
static void scene_emitter_xy(const Scene *s, float *cx, float *cy) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  switch (pp->emitter) {
  case EMIT_CENTER_LEFT:
    *cx = 6.0f;
    *cy = (float)s->rows * 0.50f;
    break;
  case EMIT_CENTER_BOTTOM:
    *cx = (float)s->cols * 0.50f;
    *cy = (float)(s->rows - 4);
    break;
  case EMIT_CENTER_TOP:
    *cx = (float)s->cols * 0.50f;
    *cy = 3.0f;
    break;
  case EMIT_CENTER:
  default:
    *cx = (float)s->cols * 0.50f;
    *cy = (float)s->rows * 0.50f;
    break;
  }
  *cx += s->emitter_offset_x;
}

/* ── Picking a launch direction and spot ── */

/* Roll a launch speed inside the effect's allowed range. */
static inline float sample_speed_from_cone(uint32_t *rng,
                                           const PatternParams *pp) {
  return sample_uniform_in_range(rng, pp->speed_min, pp->speed_max);
}

/* Roll how long this spark will live, inside the effect's range. */
static inline float sample_lifetime_from_pattern(uint32_t *rng,
                                                 const PatternParams *pp) {
  return sample_uniform_in_range(rng, pp->life_min, pp->life_max);
}

/* Find the point on the spinning wheel's rim at the current angle.
 * The rim point rides a circle of radius SPINNER_RADIUS around the
 * centre (Bourg & Bywalec Ch. 8). */
static inline void wheel_rim_position(float cx, float cy, float phase,
                                      float *out_x, float *out_y) {
  *out_x = cx + SPINNER_RADIUS * cosf(phase);
  *out_y = cy + SPINNER_RADIUS * sinf(phase);
}

/* The direction a spark leaves the rim: sideways along the wheel, not
 * straight out from the centre. Launching along the rim is what makes
 * the SPINNER look like a spinning pinwheel instead of a plain burst. */
static inline float wheel_tangent_angle(float phase) {
  return phase + (float)M_PI / 2.0f;
}

/* SPINNER launch: pick one of the two rim points on opposite sides
 * (50/50), put a spark there, and send it off along the rim with a
 * touch of random wobble so the stream isn't a perfectly straight line. */
static void wheel_compute_tangential_emission(uint32_t *rng, float phase_now,
                                              float cx, float cy,
                                              float *spawn_x, float *spawn_y,
                                              float *emit_angle) {
  float side = (lcg_unit(rng) < 0.5f) ? 0.0f : (float)M_PI;
  float phase = phase_now + side;
  wheel_rim_position(cx, cy, phase, spawn_x, spawn_y);
  float jitter = (lcg_unit(rng) - 0.5f) * 2.0f * SPINNER_TANGENT_JITTER;
  *emit_angle = wheel_tangent_angle(phase) + jitter;
}

/* The ordinary launch used by every effect except SPINNER: pick a
 * random direction inside the effect's allowed range, and a birth spot
 * near the emitter with a little random fuzz. */
static void cone_compute_emission(uint32_t *rng, const PatternParams *pp,
                                  float cx, float cy, float *spawn_x,
                                  float *spawn_y, float *emit_angle) {
  *emit_angle = sample_uniform_in_range(rng, pp->angle_min, pp->angle_max);
  *spawn_x = cx + (lcg_unit(rng) - 0.5f) * 2.0f * pp->emit_x_jitter;
  *spawn_y = cy + (lcg_unit(rng) - 0.5f) * 2.0f * pp->emit_y_jitter;
}

/* Launch one new spark for the current effect. SPINNER throws it off
 * the spinning rim; every other effect uses the ordinary cone launch.
 * Marked has_split=false, meaning it's an original (a SPARKLERS one is
 * still allowed to pop later). */
static void scene_spawn_spark(Scene *s) {
  int idx = spark_pool_find_inactive(s);
  if (idx < 0)
    return;
  Spark *e = &s->sparks[idx];

  const PatternParams *pp = &pattern_params[s->current_pattern];

  float cx, cy;
  scene_emitter_xy(s, &cx, &cy);

  float spawn_x, spawn_y, emit_angle;
  if (s->current_pattern == PATTERN_GROUND_SPINNER)
    wheel_compute_tangential_emission(&s->rng, s->spinner_phase, cx, cy,
                                      &spawn_x, &spawn_y, &emit_angle);
  else
    cone_compute_emission(&s->rng, pp, cx, cy, &spawn_x, &spawn_y, &emit_angle);

  float speed = sample_speed_from_cone(&s->rng, pp);
  float life = sample_lifetime_from_pattern(&s->rng, pp);

  spark_init_kinematics(e, spawn_x, spawn_y, speed, emit_angle, life);
  spark_seed_trail_at_position(e, spawn_x, spawn_y);
  e->has_split = false;
  e->active = true;
}

/* Spawn one of the smaller sparks when a SPARKLERS spark pops: a random
 * direction, slower and shorter-lived, starting where the parent was.
 * Marked has_split=true so it can't pop again (caps the chain at two
 * generations). */
static void scene_spawn_child_spark(Scene *s, float x, float y) {
  int idx = spark_pool_find_inactive(s);
  if (idx < 0)
    return; /* pool's full — fine to drop a few of these */
  Spark *e = &s->sparks[idx];

  float angle = sample_random_phase_2pi(&s->rng);
  float speed = sample_uniform_in_range(&s->rng, SPARKLER_CHILD_SPEED_MIN,
                                        SPARKLER_CHILD_SPEED_MAX);
  float life = sample_uniform_in_range(&s->rng, SPARKLER_CHILD_LIFE_MIN,
                                       SPARKLER_CHILD_LIFE_MAX);

  spark_init_kinematics(e, x, y, speed, angle, life);
  spark_seed_trail_at_position(e, x, y);
  e->has_split = true; /* a popped spark never pops again */
  e->active = true;
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->current_theme = 0;
  s->current_pattern = PATTERN_WELDING;
  s->emitter_offset_x = 0.0f;
  s->rng = (uint32_t)clock_ns();
  s->cols = cols;
  s->rows = rows;
  scene_clear_sparks(s);
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
}

static void scene_reseed(Scene *s) {
  s->rng = (uint32_t)clock_ns() ^ 0xDEADBEEFu;
  s->emitter_offset_x = 0.0f;
  s->spinner_phase = 0.0f;
  scene_clear_sparks(s);
}

/* ── Running one tick ── */

/* Turn the SPINNER wheel a little this frame, wrapping the angle so it
 * never grows without bound. Always runs, so the wheel keeps its
 * position even while another effect is on screen. */
static inline void wheel_phase_advance_and_wrap(Scene *s, float dt) {
  s->spinner_phase += SPINNER_OMEGA * dt;
  if (s->spinner_phase > 2.0f * (float)M_PI)
    s->spinner_phase -= 2.0f * (float)M_PI;
}

/* Count every live spark. */
static int count_active_sparks(const Scene *s) {
  int n = 0;
  for (int i = 0; i < MAX_SPARKS; i++)
    if (s->sparks[i].active)
      n++;
  return n;
}

/* Count only the original sparks (not the popped children). SPARKLERS
 * uses this so its extra sparks don't get counted toward the target and
 * choke off new launches. */
static int count_active_parent_sparks(const Scene *s) {
  int n = 0;
  for (int i = 0; i < MAX_SPARKS; i++)
    if (s->sparks[i].active && !s->sparks[i].has_split)
      n++;
  return n;
}

/* How many sparks to launch this frame: enough to climb back toward the
 * effect's target, but capped per frame (the cap scales with frame time)
 * so resuming after a long pause doesn't spit them all out at once. */
static int compute_spawn_count_for_tick(int active, const PatternParams *pp,
                                        float dt) {
  int target = pp->target_sparks;
  if (target > MAX_SPARKS)
    target = MAX_SPARKS;
  int spawn_cap = (int)((float)pp->target_sparks * dt * 6.0f) + 4;
  int n = target - active;
  if (n < 0)
    n = 0;
  if (n > spawn_cap)
    n = spawn_cap;
  return n;
}

/* Launch n new sparks. */
static void spark_pool_topup_parents(Scene *s, int n) {
  for (int k = 0; k < n; k++)
    scene_spawn_spark(s);
}

/* The SPARKLERS pop: any original spark that's lived past 40% of its
 * life bursts into a few smaller ones where it currently is, then is
 * marked so it won't burst again. */
static void cascade_sparkler_split_pass(Scene *s) {
  for (int i = 0; i < MAX_SPARKS; i++) {
    Spark *e = &s->sparks[i];
    if (!e->active || e->has_split)
      continue;
    if (e->age < e->life * SPARKLER_SPLIT_AGE_FRAC)
      continue;
    e->has_split = true;
    for (int c = 0; c < SPARKLER_CHILDREN_PER_SPLIT; c++)
      scene_spawn_child_spark(s, e->x, e->y);
  }
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  dt *= (float)s->speed / (float)SPEED_DEF;

  const PatternParams *pp = &pattern_params[s->current_pattern];

  /* Turn the spinner wheel. */
  wheel_phase_advance_and_wrap(s, dt);

  /* Launch enough new sparks to stay near the effect's target count. */
  int active = (s->current_pattern == PATTERN_SPARKLERS)
                   ? count_active_parent_sparks(s)
                   : count_active_sparks(s);
  int to_spawn = compute_spawn_count_for_tick(active, pp, dt);
  spark_pool_topup_parents(s, to_spawn);

  /* Move every spark, then bounce or retire the ones that need it. */
  float floor_y = scene_floor_y(s);
  float drag_factor = compute_drag_factor_continuous(pp->drag_coeff, dt);

  for (int i = 0; i < MAX_SPARKS; i++) {
    Spark *e = &s->sparks[i];
    if (!e->active)
      continue;

    spark_shift_trail_history(e);
    integrate_spark_semi_implicit_euler(e, pp->gravity, drag_factor, dt);

    if (spark_crossed_floor_descending(e, floor_y) &&
        reflect_spark_off_floor_and_test_settle(e, floor_y, pp->restitution,
                                                pp->floor_friction)) {
      e->active = false; /* settled at the floor — retire it */
      continue;
    }

    if (spark_should_die(e, s->cols))
      e->active = false;
  }

  /* SPARKLERS: pop the sparks that have reached the right age. */
  if (s->current_pattern == PATTERN_SPARKLERS)
    cascade_sparkler_split_pass(s);
}

/* Pick the colour/brightness slot for a spark's head based on how much
 * life it has left: a fresh spark is the brightest slot, a dying one is
 * the dimmest. Same fade as the embers in embers.c. */
static inline int spark_head_slot(const Spark *e) {
  float T = 1.0f - e->age / e->life;
  if (T < 0.0f)
    T = 0.0f;
  if (T > 1.0f)
    T = 1.0f;
  int slot = (int)(T * 7.999f);
  if (slot < 0)
    slot = 0;
  if (slot > 7)
    slot = 7;
  return slot;
}

/* ── Drawing ── */

/* Colour slot for one point of the trail: dimmer the farther back you
 * go, so the streak fades behind the head. Returns -1 once a point is
 * too dim to bother drawing. */
static inline int trail_ramp_slot_for_offset(int head_slot, int k) {
  return head_slot - (TRAIL_LEN - k);
}

/* Boldness for a colour slot: bright slots drawn bold, dim slots drawn
 * faint, the rest normal. Used for both heads and trails so they share
 * the same gradient feel. */
static inline int head_attr_by_brightness_slot(int slot) {
  if (slot >= 6)
    return A_BOLD;
  if (slot <= 1)
    return A_DIM;
  return A_NORMAL;
}

/* Pick the spinning-axle character (| / - \) from the wheel's angle, so
 * the hub looks like it's turning along with the rim. */
static inline char wheel_axle_glyph_for_phase(float phase) {
  static const char rot_chars[4] = {'|', '/', '-', '\\'};
  int rot_idx = ((int)(phase * (2.0f / (float)M_PI))) & 3;
  return rot_chars[rot_idx];
}

/* Draw one spark's fading tail — the remembered positions, oldest and
 * dimmest first, brightening up to just below the head. */
static void trail_draw_one_spark(const Spark *e, int head_slot, int cols,
                                 int rows_eff) {
  for (int k = 0; k < TRAIL_LEN; k++) {
    int slot = trail_ramp_slot_for_offset(head_slot, k);
    if (slot < 0)
      continue; /* too dim to draw */

    int ix = (int)(e->trail_x[k] + 0.5f);
    int iy = (int)(e->trail_y[k] + 0.5f);
    if (ix < 0 || ix >= cols)
      continue;
    if (iy < 0 || iy >= rows_eff)
      continue;

    char glyph = TRAIL_GLYPHS[slot];
    int attr = (slot <= 1) ? A_DIM : A_NORMAL;
    int pair = PAIR_HEAT_BASE + slot;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(iy, ix, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
  }
}

/* Draw the bright dot — the spark's head — at where it is now. */
static void head_draw_one_spark(const Spark *e, int cols, int rows_eff) {
  int ix = (int)(e->x + 0.5f);
  int iy = (int)(e->y + 0.5f);
  if (ix < 0 || ix >= cols)
    return;
  if (iy < 0 || iy >= rows_eff)
    return;

  int slot = spark_head_slot(e);
  char glyph = HEAD_GLYPHS[slot];
  int attr = head_attr_by_brightness_slot(slot);
  int pair = PAIR_HEAT_BASE + slot;
  attron(COLOR_PAIR(pair) | attr);
  mvaddch(iy, ix, (chtype)(unsigned char)glyph);
  attroff(COLOR_PAIR(pair) | attr);
}

/* Draw all the tails first, so the bright heads (drawn next) land on
 * top and a passing streak never covers up the dot you're watching. */
static void spark_pool_draw_trails(const Scene *s, int rows_eff) {
  for (int i = 0; i < MAX_SPARKS; i++) {
    const Spark *e = &s->sparks[i];
    if (e->active)
      trail_draw_one_spark(e, spark_head_slot(e), s->cols, rows_eff);
  }
}

/* Draw all the heads, on top of the tails. */
static void spark_pool_draw_heads(const Scene *s, int rows_eff) {
  for (int i = 0; i < MAX_SPARKS; i++) {
    const Spark *e = &s->sparks[i];
    if (e->active)
      head_draw_one_spark(e, s->cols, rows_eff);
  }
}

/* Draw the spinning axle (| / - \) at the centre of the SPINNER wheel. */
static void wheel_axle_draw(const Scene *s, int rows_eff) {
  float cx, cy;
  scene_emitter_xy(s, &cx, &cy);
  int ix = (int)(cx + 0.5f);
  int iy = (int)(cy + 0.5f);
  if (ix < 0 || ix >= s->cols)
    return;
  if (iy < 0 || iy >= rows_eff)
    return;

  char glyph = wheel_axle_glyph_for_phase(s->spinner_phase);
  int pair = PAIR_HEAT_BASE + 7;
  attron(COLOR_PAIR(pair) | A_BOLD);
  mvaddch(iy, ix, (chtype)(unsigned char)glyph);
  attroff(COLOR_PAIR(pair) | A_BOLD);
}

static void scene_draw(const Scene *s) {
  int rows_eff = s->rows - 1; /* keep the bottom row for the hint bar */

  spark_pool_draw_trails(s, rows_eff); /* dim tails first    */
  spark_pool_draw_heads(s, rows_eff);  /* bright heads on top */

  if (s->current_pattern == PATTERN_GROUND_SPINNER)
    wheel_axle_draw(s, rows_eff);
}

/* ── §6 screen ── */

typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *sc) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) {
  (void)sc;
  endwin();
}
static void screen_resize_curses(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static int scene_active_count(const Scene *s) {
  int n = 0;
  for (int i = 0; i < MAX_SPARKS; i++)
    if (s->sparks[i].active)
      n++;
  return n;
}

/* Draw the sparks, then lay two bars over them: a status line across
 * the top (effect, theme, spark count, fps, etc.) and a key-hint line
 * across the bottom. Both bars fill their whole row with colour and are
 * drawn last, so no spark shows through them. */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(s);

  int active = scene_active_count(s);
  const char *state_str =
      s->paused ? "PAUSED   " : pattern_name(s->current_pattern);

  /* Top row: the live status line. */
  char status[220];
  snprintf(status, sizeof status,
           " SPARKS   %s   theme:%-8s   sparks:%4d   "
           "emit_dx:%+5.1f   %5.1f fps  %3d Hz  speed:%-3d ",
           state_str, themes[s->current_theme].name, active,
           (double)s->emitter_offset_x, fps, sim_fps, s->speed);

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  for (int x = 0; x < sc->cols; x++)
    mvaddch(0, x, ' ');
  mvprintw(0, 0, "%s", status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* Bottom row: the key hints. */
  const char *hints = " q:quit  spc:pause  r:reseed  n/p:pattern  t/T:theme  "
                      "w/W:emitter  +/-:speed  ]/[:Hz ";

  int hint_row = sc->rows - 1;
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  for (int x = 0; x < sc->cols; x++)
    mvaddch(hint_row, x, ' ');
  mvprintw(hint_row, 0, "%s", hints);
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §7 app ── */

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
  screen_resize_curses(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;
  case 'r':
  case 'R':
    scene_reseed(s);
    break;

  case '=':
  case '+':
    if (s->speed < SPEED_MAX)
      s->speed *= 2;
    if (s->speed > SPEED_MAX)
      s->speed = SPEED_MAX;
    break;
  case '-':
    s->speed /= 2;
    if (s->speed < SPEED_MIN)
      s->speed = SPEED_MIN;
    break;

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

  case 't':
    s->current_theme = (s->current_theme + 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;
  case 'T':
    s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;

  case 'n':
  case 'N':
    s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
    break;
  case 'p':
  case 'P':
    s->current_pattern =
        (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
    break;

  case 'w':
    s->emitter_offset_x += EMITTER_SHIFT_STEP;
    break;
  case 'W':
    s->emitter_offset_x -= EMITTER_SHIFT_STEP;
    break;

  default:
    break;
  }
  return true;
}

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

    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }

    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
