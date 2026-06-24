/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * burst.c — fireworks in the terminal.  Bursts go off at random spots,
 * each one flashing then throwing a fan of ASCII sparks that fly out, fade,
 * and die.  Spots that sparks pass over keep a faint scorch mark, so the
 * screen slowly remembers where past bursts happened.
 *
 * The idea of throwing lots of tiny short-lived particles to fake fire and
 * smoke comes from Reeves (1983), "Particle Systems", ACM TOG 2(2): 91.
 *
 * Build:  gcc -std=c11 -O2 -Wall -Wextra particle_systems/burst.c \
 *             -o burst -lncurses -lm
 */

#define _POSIX_C_SOURCE 200809L

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

/* ── §1  config ── */

enum {
  SIM_FPS_MIN = 5,
  SIM_FPS_DEFAULT = 24,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 4,

  BURSTS_MIN = 1,
  BURSTS_DEFAULT = 5,
  BURSTS_MAX = 16,

  PARTICLES = 48,   /* sparks thrown by one burst                */
  BURST_TICKS = 22, /* hard cap on how long sparks fly           */
  FUSE_MIN = 8,     /* shortest wait before a burst goes off     */
  FUSE_RANGE = 20,  /* random extra wait on top of the minimum   */

  BURST_WAVES = 4,     /* sparks leave in this many timed rings      */
  BURST_MAX_DELAY = 5, /* head start gap between first and last ring */

  HUD_COLS = 64, /* width of the top-right status text         */
  FPS_UPDATE_MS = 500,
};

/* The tuning knobs below are fractions, so they live as #define rather than
 * the integer enum above.  Every magic number sits here, leaving the loops
 * further down free of bare literals. */
#define DRAG_FACTOR 0.82f          /* speed kept each tick; the rest is lost */
#define FLASH_LIFE_THRESHOLD 0.65f /* a spark glows bold while younger than this */

#define BURST_ANGLE_JITTER 0.2f /* random wobble added to each spark's heading */
#define BURST_SPEED_MIN 1.8f    /* slowest a fresh spark can fly */
#define BURST_SPEED_MAX 4.6f    /* fastest a fresh spark can fly */

#define PARTICLE_LIFE_MIN 0.8f   /* a fresh spark starts with at least this much life */
#define PARTICLE_LIFE_MAX 1.0f   /* ...and at most this much */
#define PARTICLE_DECAY_MIN 0.05f /* slowest a spark fades per tick */
#define PARTICLE_DECAY_MAX 0.09f /* fastest a spark fades per tick */

#define FUSE_NEVER (INT32_MAX / 2) /* a parked burst slot that never lights itself */

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(fps) (NS_PER_SEC / (fps))

/* ── §2  clock ── */

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

/* ── §3  random + math ── */

/* Small helpers for random numbers and clamping, named so the call sites
 * below read like plain English. */
static inline float rand_unit(void) { return (float)rand() / RAND_MAX; }
static inline float rand_range(float lo, float hi) {
  return lo + rand_unit() * (hi - lo);
}
static inline int rand_int_below(int n) { return rand() % n; }
static inline int clamp_int(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/* ── §4  themes ── */

/* The seven spark colours.  Each name is also the ncurses colour-pair id
 * it gets bound to (C_RED is pair 1, and so on), so we can pass a Hue
 * straight into COLOR_PAIR(). */
typedef enum {
  C_RED = 1,
  C_ORANGE = 2,
  C_YELLOW = 3,
  C_GREEN = 4,
  C_CYAN = 5,
  C_BLUE = 6,
  C_MAGENTA = 7,
  C_COUNT = 7,
} Hue;

/* The status-bar colours get pair ids past the seven spark colours, so
 * switching themes never accidentally repaints the HUD. */
#define PAIR_HUD 8
#define PAIR_HINT 9

/*
 * BurstTheme — one named colour scheme for the sparks.
 *
 *   name    the label shown in the status bar when this theme is active
 *   fg256   a colour for each of the 7 spark slots, on terminals that
 *           support the full 256-colour range
 *   fg8     a fallback colour per slot for plain 8-colour terminals
 *
 * The table of themes is read-only.  Pressing t/T just changes which row
 * we point at and re-binds the colour pairs; sparks already on screen pick
 * up the new colours on their next redraw, with no flicker.
 */
typedef struct {
  const char *name;
  int fg256[C_COUNT];
  int fg8[C_COUNT];
} BurstTheme;

#define THEME_COUNT 10

static const BurstTheme k_themes[THEME_COUNT] = {
    {"matrix",
     {22, 28, 34, 40, 46, 82, 118},
     {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_GREEN, COLOR_WHITE}},
    {"neon",
     {201, 207, 213, 159, 226, 195, 51},
     {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN, COLOR_YELLOW,
      COLOR_CYAN, COLOR_CYAN}},
    {"nova",
     {52, 88, 124, 160, 196, 208, 220},
     {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW,
      COLOR_YELLOW}},
    {"ocean",
     {24, 31, 39, 45, 51, 123, 195},
     {COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,
      COLOR_WHITE}},
    {"fire",
     {196, 202, 208, 214, 220, 226, 231},
     {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
      COLOR_WHITE}},
    {"toxic",
     {28, 40, 46, 154, 190, 226, 220},
     {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_YELLOW, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_YELLOW}},
    {"gold",
     {130, 136, 178, 214, 220, 226, 230},
     {COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
      COLOR_WHITE, COLOR_WHITE}},
    {"ice",
     {21, 27, 33, 39, 45, 51, 195},
     {COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN,
      COLOR_WHITE}},
    {"aurora",
     {28, 35, 50, 86, 121, 207, 219},
     {COLOR_GREEN, COLOR_GREEN, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN,
      COLOR_MAGENTA, COLOR_MAGENTA}},
    {"plasma",
     {53, 91, 129, 165, 207, 213, 51},
     {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_CYAN, COLOR_CYAN}},
};

/* Point the 7 spark colour pairs at the chosen theme's colours. */
static void theme_apply(int theme) {
  const BurstTheme *th = &k_themes[theme];
  for (int i = 0; i < C_COUNT; i++) {
    int slot = i + 1; /* slot 0 of the array is pair 1 (C_RED), etc. */
    if (COLORS >= 256)
      init_pair(slot, th->fg256[i], COLOR_BLACK);
    else
      init_pair(slot, th->fg8[i], COLOR_BLACK);
  }
}

/* Turn on colour and set up every pair once at startup. */
static void color_init(int theme) {
  start_color();
  use_default_colors();
  theme_apply(theme);

  /* The status bar colours never change with the theme, so set them once. */
  init_pair(PAIR_HUD, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
  init_pair(PAIR_HINT, COLORS >= 256 ? 51 : COLOR_CYAN, -1);

  /* Four bright, easy-to-tell-apart colours used only by the debug views,
   * one per wave.  Also fixed, untouched by theme switching. */
  if (COLORS >= 256) {
    init_pair(10, 196, -1); /* wave 0: red    */
    init_pair(11, 226, -1); /* wave 1: yellow */
    init_pair(12, 46, -1);  /* wave 2: green  */
    init_pair(13, 51, -1);  /* wave 3: cyan   */
  } else {
    init_pair(10, COLOR_RED, -1);
    init_pair(11, COLOR_YELLOW, -1);
    init_pair(12, COLOR_GREEN, -1);
    init_pair(13, COLOR_CYAN, -1);
  }
}

static Hue hue_rand(void) { return (Hue)(1 + rand() % C_COUNT); }

/* ── §5  debug overlay ── */

/* Extra ways to draw the sparks so you can see what's going on under the
 * hood.  Press d/D to cycle through them. */
typedef enum {
  DBG_NORMAL = 0,   /* the normal pretty view                          */
  DBG_WAVES = 1,    /* colour each spark by its wave, so the rings show */
  DBG_VELOCITY = 2, /* draw an arrow pointing where each spark is going */
  DBG_FUSE = 3,     /* print each burst's countdown / age at its centre */
  DBG_COUNT = 4,
} DebugMode;

static const char *const k_debug_names[DBG_COUNT] = {
    "normal",
    "waves",
    "velocity",
    "fuse",
};

static DebugMode g_debug = DBG_NORMAL;

/* Turn a velocity into one of eight arrow glyphs pointing that way. */
static char dir_char(float vx, float vy) {
  static const char k_dirs[8] = {'>', '\\', 'v', '/', '<', '\\', '^', '/'};
  float a = atan2f(vy, vx);
  if (a < 0.0f)
    a += 2.0f * (float)M_PI;
  int idx = (int)((a + (float)M_PI / 8.0f) / ((float)M_PI / 4.0f)) & 7;
  return k_dirs[idx];
}

/* Map a wave number to its debug colour pair (the four set up above). */
static int wave_pair(int wave) { return 10 + (wave & 3); }

/* ── §6  particle ── */

/* Terminal cells are about twice as tall as they are wide, so a step of the
 * same size sideways covers more screen than the same step up or down.  We
 * multiply horizontal distance by this to make motion look even. */
#define ASPECT 2.0f

/*
 * Particle — one ASCII spark, the smallest piece of the whole show.
 *
 *   cx, cy   the burst's centre, in terminal cells.  Fixed once at spawn;
 *            this is the point the spark flies away from.
 *   rx, ry   how far the spark has drifted from that centre, kept in fine
 *            sub-cell units so slow motion still looks smooth.
 *   vx, vy   current speed in each direction (sub-cells per tick).
 *   life     how much glow is left: starts near 1.0, dies at 0.0.
 *   decay    how much life this spark loses each tick (each spark a touch
 *            different so they don't all wink out together).
 *   delay    ticks to wait before this spark starts moving; this is what
 *            makes the burst leave in rings instead of all at once.
 *   wave     which ring (0..3) this spark belongs to; only the "waves"
 *            debug view actually looks at it.
 *   sym      the character drawn for this spark, picked once so the eye
 *            can follow a single spark across the screen.
 *   hue      the spark's colour, also fixed at spawn.
 *   alive    false means the slot is free and gets skipped everywhere.
 *
 * A spark is filled in at burst time, nudged once per tick, and drawn once
 * per frame.  When its life runs out or it leaves the screen, alive flips
 * to false and the slot is ready to be reused.  No allocation ever happens
 * here — every burst owns a fixed array of these.
 */
typedef struct {
  float cx, cy;
  float rx, ry;
  float vx, vy;
  float life;
  float decay;
  int delay;
  int wave;
  char sym;
  Hue hue;
  bool alive;
} Particle;

static const char k_syms[] = "*.+o#@%&$!^~-=|/\\:;,`'\"";
#define NSYMS (int)(sizeof k_syms - 1)

/* Set up one fresh spark when a burst goes off. */
static void particle_spawn(Particle *p, float cx, float cy, float angle,
                           float speed, int delay_ticks, int wave) {
  /* Remember where it started from; this never changes. */
  p->cx = cx;
  p->cy = cy;

  /* It hasn't drifted yet. */
  p->rx = 0.0f;
  p->ry = 0.0f;

  /* Turn a heading + speed into sideways and vertical speed. */
  p->vx = cosf(angle) * speed;
  p->vy = sinf(angle) * speed;

  /* Give it a random starting glow and a random fade rate. */
  p->life = rand_range(PARTICLE_LIFE_MIN, PARTICLE_LIFE_MAX);
  p->decay = rand_range(PARTICLE_DECAY_MIN, PARTICLE_DECAY_MAX);

  /* Which ring it's in and how long to hold before it starts moving. */
  p->delay = delay_ticks;
  p->wave = wave;

  /* Lock in a character and colour so it's easy to follow. */
  p->sym = k_syms[rand_int_below(NSYMS)];
  p->hue = hue_rand();
  p->alive = true;
}

/*
 * Move one spark forward by a tick: slow it down, drift it, fade it, and
 * retire it if it has burned out or flown off-screen.
 *
 * We slow the spark down BEFORE moving it.  That matches how air drag really
 * feels, and it stops a spark from leaping clean off the screen in a single
 * big step when the frame timing hiccups.
 */
static void particle_tick(Particle *p, int cols, int rows) {
  /* Already dead? Nothing to do. */
  if (!p->alive)
    return;

  /* Still waiting for its ring's turn — sit tight and count down. */
  if (p->delay > 0) {
    p->delay--;
    return;
  }

  /* Bleed off a little speed (air drag). */
  p->vx *= DRAG_FACTOR;
  p->vy *= DRAG_FACTOR;

  /* Drift by the current speed. */
  p->rx += p->vx;
  p->ry += p->vy;

  /* Dim a little. */
  p->life -= p->decay;

  /* Retire it if it's faded out or wandered off the screen.  We work out the
   * real screen position the same way the drawing code does, so the two agree
   * on where the spark is. */
  float screen_x = p->cx + p->rx * ASPECT;
  float screen_y = p->cy + p->ry;
  bool burned_out = (p->life <= 0.0f);
  bool off_screen = (screen_x < 0.f || screen_x >= (float)cols ||
                     screen_y < 0.f || screen_y >= (float)rows);
  if (burned_out || off_screen)
    p->alive = false;
}

/* Work out which screen cell a spark currently sits in. */
static void particle_pixel_to_cell(const Particle *p, int *cell_x,
                                   int *cell_y) {
  *cell_x = (int)(p->cx + p->rx * ASPECT);
  *cell_y = (int)(p->cy + p->ry);
}

/* Draw one spark.  The current debug mode decides what it looks like. */
static void particle_draw(const Particle *p, WINDOW *w, int cols, int rows) {
  /* Skip dead or still-waiting sparks, same as the movement code. */
  if (!p->alive || p->delay > 0)
    return;

  int cell_x, cell_y;
  particle_pixel_to_cell(p, &cell_x, &cell_y);
  if (cell_x < 0 || cell_x >= cols || cell_y < 0 || cell_y >= rows)
    return;

  /* Same spark either way — only its glyph and colour change with the mode. */
  chtype glyph;
  attr_t attr;
  switch (g_debug) {
  case DBG_WAVES:
    /* Tint by ring so you can see the waves leave one after another. */
    glyph = (chtype)(unsigned char)p->sym;
    attr = COLOR_PAIR(wave_pair(p->wave)) | A_BOLD;
    break;
  case DBG_VELOCITY:
    /* Show an arrow pointing the way the spark is moving. */
    glyph = (chtype)(unsigned char)dir_char(p->vx, p->vy);
    attr = COLOR_PAIR(p->hue) | A_BOLD;
    break;
  case DBG_FUSE:
  case DBG_NORMAL:
  default: {
    /* The normal look: its own glyph, drawn bold while it's still bright. */
    bool is_fresh_spark = (p->life > FLASH_LIFE_THRESHOLD);
    glyph = (chtype)(unsigned char)p->sym;
    attr = COLOR_PAIR(p->hue) | (is_fresh_spark ? A_BOLD : 0);
    break;
  }
  }

  wattron(w, attr);
  mvwaddch(w, cell_y, cell_x, glyph);
  wattroff(w, attr);
}

/* ── §7  burst FSM ── */

/* The three things a burst can be doing at any moment. */
typedef enum {
  BS_IDLE = 0,  /* waiting, fuse ticking down; nothing on screen */
  BS_FLASH = 1, /* one single frame of a bright '*+' flash       */
  BS_LIVE = 2,  /* sparks flying; goes back to idle once they're all gone */
} BurstState;

/*
 * Burst — one firework: a batch of sparks plus a tiny state machine that
 * runs it through wait → flash → sparks → wait again.
 *
 *   cx, cy   where this burst goes off, in terminal cells.
 *   state    which of the three phases it's in right now (see above).
 *   ticks    while sparks are flying, counts up from 0; used to force the
 *            burst to end after BURST_TICKS even if a few sparks linger.
 *   fuse     while waiting, counts down to 0; at 0 the burst lights.
 *   parts    this burst's own fixed batch of sparks.
 *
 * The cycle:
 *   waiting ── fuse hits 0 ──▶ flash (all sparks are created here)
 *   flash   ── one frame   ──▶ sparks flying
 *   sparks  ── all gone, or out of time ──▶ waiting again (fuse re-rolled)
 *
 * Why bother with a separate one-frame flash instead of just "on/off"? That
 * single bright pop is what sells it as an explosion — a bang, then debris.
 * Skip it and the burst just looks like a puff of dots appearing.
 *
 * Each burst keeps its own fuse and runs on its own clock, knowing nothing
 * about the others.  A screenful of them ticking down independently is what
 * gives the display its uneven, natural rhythm.
 */
typedef struct {
  float cx, cy;
  BurstState state;
  int ticks;
  int fuse;
  Particle parts[PARTICLES];
} Burst;

/* Pick a random spot to go off, kept a couple cells in from the edges so
 * the flash cross always has room to draw. */
static void pick_detonation_centre(int cols, int rows, float *cx, float *cy) {
  int safe_cols_extent = (cols - 4) > 1 ? (cols - 4) : 1;
  int safe_rows_extent = (rows - 2) > 1 ? (rows - 2) : 1;
  *cx = (float)(2 + rand_int_below(safe_cols_extent));
  *cy = (float)(1 + rand_int_below(safe_rows_extent));
}

/* Spread the sparks evenly around the full circle, with a little random
 * wobble so the fan looks natural instead of mechanical. */
static float compute_emission_angle(int i, int total_particles) {
  float evenly_spaced =
      ((float)i / (float)total_particles) * 2.0f * (float)M_PI;
  float jitter = rand_unit() * BURST_ANGLE_JITTER;
  return evenly_spaced + jitter;
}

/* A random launch speed for one spark. */
static float compute_emission_speed(void) {
  return rand_range(BURST_SPEED_MIN, BURST_SPEED_MAX);
}

/*
 * Give each ring a slightly longer head start so the burst goes out as a few
 * spreading rings rather than one instant circle.  Ring 0 leaves now, the
 * last ring waits the longest.  With 4 rings and a max delay of 5, the waits
 * come out to 0, 1, 3, and 5 ticks.
 */
static int compute_wave_delay(int wave, int wave_count, int max_delay) {
  if (wave_count <= 1)
    return 0;
  return (wave * max_delay) / (wave_count - 1);
}

/* Light the burst: choose a spot, flash, and create all its sparks. */
static void burst_ignite(Burst *b, int cols, int rows) {
  /* Where it goes off. */
  pick_detonation_centre(cols, rows, &b->cx, &b->cy);

  /* Show the one-frame flash next. */
  b->state = BS_FLASH;
  b->ticks = 0;

  /* Throw the sparks out in a fan, ring by ring. */
  for (int i = 0; i < PARTICLES; i++) {
    float angle = compute_emission_angle(i, PARTICLES);
    float speed = compute_emission_speed();
    int wave = i % BURST_WAVES;
    int delay = compute_wave_delay(wave, BURST_WAVES, BURST_MAX_DELAY);
    particle_spawn(&b->parts[i], b->cx, b->cy, angle, speed, delay, wave);
  }
}

/* ── §8  burst tick ── */

/* Move every spark of this burst one tick, and report whether any are still
 * alive (so the burst knows when it's over). */
static bool burst_advance_live_particles(Burst *b, int cols, int rows) {
  bool any_alive = false;
  for (int i = 0; i < PARTICLES; i++) {
    particle_tick(&b->parts[i], cols, rows);
    if (b->parts[i].alive)
      any_alive = true;
  }
  return any_alive;
}

/* The burst just finished: leave a scorch mark, roll a fresh wait, and go
 * back to waiting.  The mark is left through a callback so this code doesn't
 * need to know the scorch grid exists. */
static void burst_complete_and_rearm(Burst *b,
                                     void (*scorch_cb)(int, int, void *),
                                     void *ud) {
  if (scorch_cb)
    scorch_cb((int)b->cx, (int)b->cy, ud);
  b->fuse = FUSE_MIN + rand_int_below(FUSE_RANGE);
  b->state = BS_IDLE;
}

/*
 * One step of a single burst.  This is the heart of the file: whatever you
 * see a burst doing on screen, it's in one of these three branches.
 *
 *   waiting  — count the fuse down; light the burst when it hits zero.
 *   flash    — the flash only lasts a frame, so move straight to the sparks.
 *   sparks   — move them all; when they're gone (or we've waited long enough)
 *              wrap up and start waiting again.
 *
 * scorch_cb is called once, right when a burst ends, to mark where it was;
 * pass NULL to skip leaving marks entirely.
 */
static void burst_tick(Burst *b, int cols, int rows,
                       void (*scorch_cb)(int x, int y, void *ud), void *ud) {
  switch (b->state) {
  case BS_IDLE:
    b->fuse--;
    if (b->fuse <= 0)
      burst_ignite(b, cols, rows);
    break;

  case BS_FLASH:
    b->state = BS_LIVE;
    b->ticks = 0;
    break;

  case BS_LIVE: {
    bool any_alive = burst_advance_live_particles(b, cols, rows);
    b->ticks++;
    bool out_of_time = (b->ticks >= BURST_TICKS);
    if (!any_alive || out_of_time)
      burst_complete_and_rearm(b, scorch_cb, ud);
    break;
  }
  }
}

/* ── §9  burst render ── */

/* Draw the opening flash: a bright '*' with a '+' on each side. */
static void draw_flash_cross(WINDOW *w, int cx, int cy, int cols, int rows) {
  if (cx < 0 || cx >= cols || cy < 0 || cy >= rows)
    return;

  wattron(w, COLOR_PAIR(C_YELLOW) | A_BOLD);
  mvwaddch(w, cy, cx, '*');
  if (cx > 0)
    mvwaddch(w, cy, cx - 1, '+');
  if (cx < cols - 1)
    mvwaddch(w, cy, cx + 1, '+');
  if (cy > 0)
    mvwaddch(w, cy - 1, cx, '+');
  if (cy < rows - 1)
    mvwaddch(w, cy + 1, cx, '+');
  wattroff(w, COLOR_PAIR(C_YELLOW) | A_BOLD);
}

/* Debug helper: print a little number at the burst's centre — "f" plus the
 * fuse while it's waiting, "t" plus the age while its sparks are flying. */
static void draw_fuse_overlay(const Burst *b, WINDOW *w, int cx, int cy,
                              int cols, int rows) {
  bool centre_in_bounds = (cx >= 0 && cx < cols - 3 && cy >= 0 && cy < rows);
  if (!centre_in_bounds)
    return;

  char label[8];
  if (b->state == BS_IDLE)
    snprintf(label, sizeof label, "f%d", b->fuse);
  else if (b->state == BS_LIVE)
    snprintf(label, sizeof label, "t%d", b->ticks);
  else
    return;

  wattron(w, COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvwaddstr(w, cy, cx, label);
  wattroff(w, COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/*
 * Draw a burst based on what it's doing: the flash if it just lit, its
 * flying sparks if they're out, nothing while it's just waiting.  The fuse
 * number is drawn on top only in the matching debug view.
 */
static void burst_draw(const Burst *b, WINDOW *w, int cols, int rows) {
  int cx = (int)b->cx;
  int cy = (int)b->cy;

  if (b->state == BS_FLASH) {
    draw_flash_cross(w, cx, cy, cols, rows);
    return;
  }

  if (b->state == BS_LIVE) {
    for (int i = 0; i < PARTICLES; i++)
      particle_draw(&b->parts[i], w, cols, rows);
  }

  if (g_debug == DBG_FUSE)
    draw_fuse_overlay(b, w, cx, cy, cols, rows);
}

/* ── §10  field ── */

/*
 * Field — the whole scene: every burst, plus the memory of past ones.
 *
 *   bursts          all the burst slots; each runs itself independently.
 *   scorch          one character per screen cell: a '.' is left wherever
 *                   a burst has gone off, '\0' everywhere else.  This is
 *                   what makes the screen "remember".  Owned here, freed by
 *                   field_free.
 *   cols, rows      screen size in cells.
 *   active_bursts   how many of the slots are currently switched on.
 *
 * Set up at startup (and again on resize or 'r'), stepped and drawn every
 * frame, freed at exit.  Bursts leave their scorch marks through a callback
 * rather than touching the grid directly, so a burst on its own knows
 * nothing about scorch.
 */
typedef struct {
  Burst bursts[BURSTS_MAX];
  char *scorch;
  int cols;
  int rows;
  int active_bursts;
} Field;

/* The callback bursts use to leave their scorch mark when they finish. */
static void field_scorch_cb(int x, int y, void *ud) {
  Field *f = (Field *)ud;
  if (x >= 0 && x < f->cols && y >= 0 && y < f->rows)
    f->scorch[y * f->cols + x] = '.';
}

/* Set up the scene: allocate the scorch grid and prime every burst slot. */
static void field_init(Field *f, int cols, int rows, int burst_count) {
  f->cols = cols;
  f->rows = rows;
  f->active_bursts = burst_count;
  f->scorch = calloc((size_t)(cols * rows), sizeof(char));

  /* Start the on bursts with spread-out fuses so they don't all pop on the
   * very first frame; the off ones get a fuse that never fires until '+'
   * switches them on. */
  int stagger_step =
      FUSE_MIN + (burst_count > 0 ? FUSE_RANGE / burst_count : 0);

  for (int i = 0; i < BURSTS_MAX; i++) {
    memset(&f->bursts[i], 0, sizeof(Burst));
    f->bursts[i].state = BS_IDLE;
    f->bursts[i].fuse = (i < burst_count) ? (i * stagger_step) : FUSE_NEVER;
  }
}

/* Release the scorch grid and blank the struct. */
static void field_free(Field *f) {
  free(f->scorch);
  *f = (Field){0};
}

/* Advance every switched-on burst by one tick. */
static void field_tick(Field *f) {
  for (int i = 0; i < f->active_bursts; i++)
    burst_tick(&f->bursts[i], f->cols, f->rows, field_scorch_cb, f);
}

/* Paint the faint scorch marks first, so the live sparks sit on top. */
static void field_draw_scorch_layer(const Field *f, WINDOW *w) {
  int total_cells = f->cols * f->rows;

  wattron(w, COLOR_PAIR(C_ORANGE) | A_DIM);
  for (int i = 0; i < total_cells; i++) {
    char scorch_glyph = f->scorch[i];
    if (!scorch_glyph)
      continue;
    int cell_y = i / f->cols;
    int cell_x = i % f->cols;
    mvwaddch(w, cell_y, cell_x, (chtype)(unsigned char)scorch_glyph);
  }
  wattroff(w, COLOR_PAIR(C_ORANGE) | A_DIM);
}

/* Draw every switched-on burst, on top of the scorch layer. */
static void field_draw_active_bursts(const Field *f, WINDOW *w) {
  for (int i = 0; i < f->active_bursts; i++)
    burst_draw(&f->bursts[i], w, f->cols, f->rows);
}

/* Draw the whole scene: scorch underneath, live bursts over it. */
static void field_draw(const Field *f, WINDOW *w) {
  field_draw_scorch_layer(f, w);
  field_draw_active_bursts(f, w);
}

/* ── §11  screen + HUD ── */

typedef struct {
  int cols;
  int rows;
} Screen;

static void screen_init(Screen *s, int theme) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init(theme);
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

static void screen_draw_field(Screen *s, const Field *f) {
  (void)s; /* the size is only needed for the HUD; the field draws to stdscr */
  erase();
  field_draw(f, stdscr);
}

/*
 * Draw the two status lines: stats in the top-right corner, the key list
 * along the bottom.  Both use their own colour pairs that the sparks never
 * touch, so the text stays readable no matter what's going off behind it.
 */
static void screen_draw_hud(Screen *s, double fps, int sim_fps, int bursts,
                            int theme) {
  char buf[HUD_COLS + 1];
  /* Only show the debug label when a debug view is on, to keep it tidy. */
  if (g_debug == DBG_NORMAL) {
    snprintf(buf, sizeof buf, " %5.1f fps  spd:%d  burst:%d  [%s] ", fps,
             sim_fps, bursts, k_themes[theme].name);
  } else {
    snprintf(buf, sizeof buf, " %5.1f fps  spd:%d  burst:%d  [%s]  dbg:%s ",
             fps, sim_fps, bursts, k_themes[theme].name,
             k_debug_names[g_debug]);
  }
  int hx = s->cols - (int)strlen(buf);
  if (hx < 0)
    hx = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, hx, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q/ESC:quit  ]/[:speed  +/-:bursts  r:clear-scorch"
           "  t/T:theme  d/D:debug ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §12  app ── */

typedef struct {
  Field field;
  Screen screen;
  int sim_fps;
  int bursts;
  int theme; /* which row of the theme table is active */
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
  field_free(&app->field);
  screen_resize(&app->screen);
  field_init(&app->field, app->screen.cols, app->screen.rows, app->bursts);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

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

  case '=':
  case '+':
    if (app->bursts < BURSTS_MAX) {
      int i = app->bursts;
      memset(&app->field.bursts[i], 0, sizeof(Burst));
      app->field.bursts[i].state = BS_IDLE;
      app->field.bursts[i].fuse = 2 + rand() % FUSE_RANGE;
      app->bursts++;
      app->field.active_bursts = app->bursts;
    }
    break;
  case '-':
    if (app->bursts > BURSTS_MIN) {
      app->bursts--;
      app->field.active_bursts = app->bursts;
    }
    break;

  case 'r':
  case 'R':
    field_free(&app->field);
    field_init(&app->field, app->screen.cols, app->screen.rows, app->bursts);
    break;

  case 't':
    app->theme = (app->theme + 1) % THEME_COUNT;
    theme_apply(app->theme);
    break;
  case 'T':
    app->theme = (app->theme + THEME_COUNT - 1) % THEME_COUNT;
    theme_apply(app->theme);
    break;

  case 'd':
    g_debug = (DebugMode)((g_debug + 1) % DBG_COUNT);
    break;
  case 'D':
    g_debug = (DebugMode)((g_debug + DBG_COUNT - 1) % DBG_COUNT);
    break;

  default:
    break;
  }
  return true;
}

/* ── §13  main ── */

int main(void) {
  srand((unsigned int)clock_ns());

  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;
  app->bursts = BURSTS_DEFAULT;
  app->theme = 0; /* start on the first theme in the table */

  screen_init(&app->screen, app->theme);
  field_init(&app->field, app->screen.cols, app->screen.rows, app->bursts);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    /* Rebuild everything if the terminal was resized. */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* How long since the last frame, capped so a long pause (or a debugger
     * stop) can't make the sim try to catch up with a huge burst of ticks. */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    /* Run the sim at a steady tick rate no matter the frame rate: bank the
     * elapsed time and spend it one fixed tick at a time. */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      field_tick(&app->field);
      sim_accum -= tick_ns;
    }
    float alpha = (float)sim_accum / (float)tick_ns;
    (void)alpha;

    /* Refresh the on-screen fps number a couple times a second. */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* Sleep to hold ~60 fps.  We sleep before drawing so the time spent
     * writing to the terminal doesn't slowly push the frame rate off. */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    /* Draw the scene and the status bars. */
    screen_draw_field(&app->screen, &app->field);
    screen_draw_hud(&app->screen, fps_display, app->sim_fps, app->bursts,
                    app->theme);
    screen_present();

    /* Handle one keypress, if any. */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  field_free(&app->field);
  screen_free(&app->screen);
  return 0;
}
