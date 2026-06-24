/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * pixel_dissolve.c — a word is drawn as a cloud of glowing dots, then
 * the dots scatter and fly back together to spell the next word, over
 * and over.
 *
 * How it works: each lit pixel of a small built-in 5x7 font becomes a
 * few particles. A spring pulls each particle to its spot so the word
 * snaps into place, holds for a moment, then every particle gets a
 * shove (blow apart, swirl, rain down, or drift off) and the cycle
 * repeats with the next word.
 *
 * The 5x7 font follows the public-domain Adafruit GFX layout
 * (7 bytes per character, one byte per row, leftmost column = top bit).
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

  MAX_PARTICLES =
      1500, /* enough dots for the longest word, with room to spare */
  PARTICLES_PER_PIXEL =
      3, /* a few dots per lit pixel so the word looks dense       */

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* Colour-pair slots. HUD and HINT are reserved for the on-screen text. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_RAMP_BASE = 3, /* slots +0..+7 hold the left-to-right colour gradient */
  PAIR_SKY = 11,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

#define ASPECT_Y 2.0f /* a terminal cell is about twice as tall as it is wide */

/* The spring that pulls each dot to its spot: K is how hard it pulls,
 * D is how much it brakes. These two are tuned to overshoot a touch
 * and settle with a little "boing" rather than glide in dead-flat. */
#define SPRING_K 18.0f
#define SPRING_D 8.0f
#define HOLD_JITTER 4.0f   /* tiny random nudges so a held word seems to breathe */
#define DISSOLVE_DRAG 0.6f /* how fast scattered dots lose speed each second      */

/* How long each stage lasts, in seconds. Kept generous so the word is
 * readable while it holds and the scatter has time to play out; press
 * '+' to run the whole thing faster. */
#define ASSEMBLE_DUR 2.5f
#define HOLD_DUR 6.0f
#define DISSOLVE_DUR 2.5f

/* How hard each scatter style flings the dots when the word breaks apart. */
#define EXPLODE_SPEED 45.0f
#define SWIRL_SPEED 35.0f
#define RAIN_SPEED 50.0f
#define RAIN_GRAVITY 120.0f
#define DRIFT_SPEED 25.0f

/*
 * Pattern — the four ways a word can break apart when it dissolves.
 *   EXPLODE  dots fly straight out from the centre of the word
 *   SWIRL    dots spin around the centre, like water down a drain
 *   RAIN     dots fall down under gravity
 *   DRIFT    dots wander off in random directions, like smoke
 * N_PATTERNS is the count, used for cycling through them with n/p.
 */
typedef enum {
  PATTERN_EXPLODE = 0,
  PATTERN_SWIRL = 1,
  PATTERN_RAIN = 2,
  PATTERN_DRIFT = 3,
  N_PATTERNS = 4,
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_EXPLODE:
    return "EXPLODE";
  case PATTERN_SWIRL:
    return "SWIRL  ";
  case PATTERN_RAIN:
    return "RAIN   ";
  case PATTERN_DRIFT:
    return "DRIFT  ";
  default:
    return "?      ";
  }
}

/*
 * Phase — which stage of the loop the word is in right now.
 *   ASSEMBLE  dots are flying in and snapping into the word
 *   HOLD      the word sits readable for a few seconds
 *   DISSOLVE  the word has broken apart and the dots are scattering
 */
typedef enum {
  PHASE_ASSEMBLE = 0,
  PHASE_HOLD = 1,
  PHASE_DISSOLVE = 2,
} Phase;

/*
 * Theme — one colour scheme for the word.
 *   name  what it's called, shown in the status bar
 *   ramp  eight colours from left edge of the word to the right edge,
 *         so each word fades across like a graded title screen
 *   sky   the background tone for this theme
 * All colours stay in the bright half of the 256-colour set so even
 * the darkest one is still visible.
 */
typedef struct {
  const char *name;
  short ramp[8];
  short sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name        ramp[0..7]                                       sky */

    {"DEFAULT", {87, 117, 153, 195, 218, 217, 211, 196}, 234},
    {"FIRE", {88, 124, 130, 166, 196, 208, 214, 226}, 233},
    {"ICE", {24, 31, 67, 110, 117, 153, 195, 231}, 234},
    {"NEON", {53, 91, 134, 165, 207, 213, 219, 225}, 234},
    {"AURORA", {43, 79, 115, 121, 157, 195, 230, 231}, 234},
    {"VIOLET", {53, 54, 91, 134, 135, 176, 213, 219}, 233},
    {"TROPICAL", {29, 35, 37, 44, 50, 86, 122, 159}, 234},
    {"FOREST", {28, 34, 40, 64, 70, 112, 156, 192}, 234},
    {"MONO", {240, 243, 245, 247, 249, 251, 253, 255}, 232},
    {"MATRIX", {22, 28, 34, 40, 46, 82, 118, 154}, 232},
};

/* The words the demo cycles through. */
static const char *WORDS[] = {
    "BOOM", "ASCII",    "PIXEL",   "MORPH", "DUST",
    "CODE", "DISSOLVE", "USELESS", "TAMIL",
};
#define N_WORDS ((int)(sizeof WORDS / sizeof WORDS[0]))

/*
 * font_5x7 — the shape of each letter, 5 dots wide by 7 dots tall.
 *
 * One row per byte, top to bottom. Inside a byte the five low bits are
 * the five columns: bit 4 is the leftmost dot, bit 0 the rightmost. A
 * set bit means "draw a dot here". Indexed by character code, so the
 * unlisted slots stay all-zero (blank). Covers space, A-Z, 0-9, ! ? .
 */
static const uint8_t font_5x7[256][7] = {
    [' '] = {0, 0, 0, 0, 0, 0, 0},
    ['!'] = {0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04},
    ['?'] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04},
    ['.'] = {0, 0, 0, 0, 0, 0x00, 0x04},

    ['A'] = {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    ['B'] = {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},
    ['C'] = {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E},
    ['D'] = {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E},
    ['E'] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F},
    ['F'] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10},
    ['G'] = {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E},
    ['H'] = {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    ['I'] = {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E},
    ['J'] = {0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C},
    ['K'] = {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11},
    ['L'] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F},
    ['M'] = {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11},
    ['N'] = {0x11, 0x11, 0x19, 0x15, 0x13, 0x11, 0x11},
    ['O'] = {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},
    ['P'] = {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10},
    ['Q'] = {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D},
    ['R'] = {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11},
    ['S'] = {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E},
    ['T'] = {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
    ['U'] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},
    ['V'] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04},
    ['W'] = {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11},
    ['X'] = {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11},
    ['Y'] = {0x11, 0x11, 0x11, 0x0A, 0x04, 0x04, 0x04},
    ['Z'] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F},

    ['0'] = {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E},
    ['1'] = {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    ['2'] = {0x0E, 0x11, 0x01, 0x06, 0x08, 0x10, 0x1F},
    ['3'] = {0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E},
    ['4'] = {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02},
    ['5'] = {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},
    ['6'] = {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E},
    ['7'] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10},
    ['8'] = {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E},
    ['9'] = {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},
};

#define FONT_W 5
#define FONT_H 7
#define FONT_KERN                                                              \
  3 /* blank columns between letters. A 1-cell gap let neighbouring          \
     * strokes blur together, so we space them out wider. */

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
      init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
    init_pair(PAIR_SKY, t->sky, -1);
  } else {
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_RAMP_BASE + i), COLOR_WHITE, -1);
    init_pair(PAIR_SKY, COLOR_BLACK, -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
  theme_apply(0);
}

/* ── §4 font ── */

/* How many columns wide the whole word will be once drawn. */
static int word_width_cells(const char *word) {
  int n = (int)strlen(word);
  if (n <= 0)
    return 0;
  return n * FONT_W + (n - 1) * FONT_KERN;
}

/* ── §5 particle ── */

/*
 * Particle — one glowing dot. The whole demo is just a pool of these.
 *   px, py       where it is now, in screen cells
 *   vx, vy       how fast it's moving, in cells per second
 *   tx, ty       the spot in the word it wants to reach
 *   color_slot   0..7, which colour in the theme gradient it wears
 *   active       false means this slot is unused right now
 */
typedef struct {
  float px, py;
  float vx, vy;
  float tx, ty;
  int color_slot;
  bool active;
} Particle;

/* A fast, throwaway random-number generator (good enough for jitter). */
static inline uint32_t lcg_next(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}
/* A random number between 0 and 1. */
static inline float lcg_unit(uint32_t *st) {
  return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/* ── §6 scene ── */

/*
 * Scene — the whole running demo in one place: the dots plus all the
 * state that decides what they're doing.
 *
 *   paused / speed              user controls: frozen, and how fast time runs
 *   current_theme               which colour scheme is active
 *   current_pattern             which scatter style the next dissolve uses
 *   rng                         seed for the throwaway random generator
 *   rows, cols                  current terminal size
 *
 *   phase / phase_t             which stage we're in, and how long we've
 *                               been in it (seconds)
 *   word_idx                    which word from WORDS is showing
 *   word_cx, word_cy            centre of the word, remembered so the
 *                               scatter styles know where to push out from
 *   word_w_cells                width of the word, kept for the colour gradient
 *
 *   n_particles                 how many dots are in use right now
 *   particles                   the fixed pool of dots (slots past
 *                               n_particles are inactive)
 */
typedef struct {
  bool paused;
  int speed;
  int current_theme;
  Pattern current_pattern;
  uint32_t rng;
  int rows, cols;

  Phase phase;
  float phase_t;
  int word_idx;
  float word_cx, word_cy;
  int word_w_cells;

  int n_particles;
  Particle particles[MAX_PARTICLES];
} Scene;

static void scene_clear_particles(Scene *s) {
  s->n_particles = 0;
  for (int i = 0; i < MAX_PARTICLES; i++)
    s->particles[i].active = false;
}

/* ── turning a word into a list of target spots ── */

/* Picks which of the 8 gradient colours a dot wears, based on how far
 * across the word it sits: left edge gets colour 0, right edge gets 7. */
static inline int target_color_slot(float pixel_x, float start_x, int word_w) {
  float frac = (pixel_x - start_x) / (float)(word_w > 1 ? word_w - 1 : 1);
  int slot = (int)(frac * 7.999f);
  if (slot < 0)
    slot = 0;
  if (slot > 7)
    slot = 7;
  return slot;
}

/* Makes the few dots that share one lit pixel. The first sits dead on
 * the pixel; the rest are nudged a fraction of a cell off it. While
 * the word holds they pile up into one bright spot, but when it
 * scatters each drifts its own way. Returns false once the target list
 * is full so the caller can stop. */
static bool target_emit_pixel_replicas(Scene *s, float *out_x, float *out_y,
                                       int *out_slot, int *n_total,
                                       int max_targets, float px, float py,
                                       int slot) {
  for (int rep = 0; rep < PARTICLES_PER_PIXEL; rep++) {
    if (*n_total >= max_targets)
      return false;
    float jx = (rep == 0) ? 0.0f : (lcg_unit(&s->rng) - 0.5f) * 0.5f;
    float jy = (rep == 0) ? 0.0f : (lcg_unit(&s->rng) - 0.5f) * 0.5f;
    out_x[*n_total] = px + jx;
    out_y[*n_total] = py + jy;
    out_slot[*n_total] = slot;
    (*n_total)++;
  }
  return true;
}

/* ── building the full target list for a word ── */

/* Walks the word letter by letter, lights up the font dots for each
 * one centred on screen, and records every spot a dot should fly to.
 * Also remembers the word's centre and width for later. */
static int scene_build_targets(Scene *s, const char *word, float *out_targets_x,
                               float *out_targets_y, int *out_color_slots,
                               int max_targets) {
  int rows_eff = s->rows - 1;
  int word_w = word_width_cells(word);
  float start_x = (float)(s->cols - word_w) * 0.5f;
  float start_y = (float)(rows_eff - FONT_H) * 0.5f;

  int n_total = 0;
  float char_x = start_x;
  int n_chars = (int)strlen(word);

  for (int c = 0; c < n_chars; c++) {
    unsigned ch = (unsigned char)word[c];
    const uint8_t *bits = font_5x7[ch];

    for (int row = 0; row < FONT_H; row++) {
      uint8_t b = bits[row];
      for (int col = 0; col < FONT_W; col++) {
        if (!(b & (1 << (FONT_W - 1 - col))))
          continue;

        float px = char_x + (float)col;
        float py = start_y + (float)row;
        int slot = target_color_slot(px, start_x, word_w);

        if (!target_emit_pixel_replicas(s, out_targets_x, out_targets_y,
                                        out_color_slots, &n_total, max_targets,
                                        px, py, slot))
          goto done;
      }
    }
    char_x += FONT_W + FONT_KERN;
  }
done:
  s->word_w_cells = word_w;
  s->word_cx = start_x + (float)word_w * 0.5f;
  s->word_cy = start_y + (float)FONT_H * 0.5f;
  return n_total;
}

/* ── adding and removing dots from the pool ── */

/* Drops a fresh dot just off one of the four screen edges, standing
 * still, so the spring can sweep it into the word from outside. */
static inline void particle_spawn_at_edge(Particle *p, uint32_t *rng, int cols,
                                          int rows) {
  int edge = (int)(lcg_unit(rng) * 4.0f);
  float r1 = lcg_unit(rng);
  switch (edge) {
  case 0:
    p->px = r1 * (float)cols;
    p->py = -3.0f;
    break;
  case 1:
    p->px = -3.0f;
    p->py = r1 * (float)rows;
    break;
  case 2:
    p->px = (float)cols + 3.0f;
    p->py = r1 * (float)rows;
    break;
  default:
    p->px = r1 * (float)cols;
    p->py = (float)rows + 3.0f;
    break;
  }
  p->vx = 0.0f;
  p->vy = 0.0f;
  p->active = true;
}

/* Adds dots until we have one per target spot. */
static void particle_pool_grow_to(Scene *s, int n_targets) {
  while (s->n_particles < n_targets) {
    particle_spawn_at_edge(&s->particles[s->n_particles], &s->rng, s->cols,
                           s->rows);
    s->n_particles++;
  }
}

/* Switches off the leftover dots when the new word needs fewer. */
static void particle_pool_shrink_to(Scene *s, int n_targets) {
  for (int i = n_targets; i < s->n_particles; i++)
    s->particles[i].active = false;
  s->n_particles = n_targets;
}

/* Hands each dot the next target in line. We don't bother pairing dots
 * to their closest spot; the random crossing paths actually look livelier
 * than if everything took the shortest route. */
static void particle_pool_assign_targets(Scene *s, const float *tx,
                                         const float *ty, const int *slot,
                                         int n) {
  for (int i = 0; i < n; i++) {
    Particle *p = &s->particles[i];
    p->tx = tx[i];
    p->ty = ty[i];
    p->color_slot = slot[i];
  }
}

/* ── loading a word into the dot pool ── */

/* Gets the current word ready to assemble: works out its target spots,
 * resizes the dot pool to match, and tells each dot where to go. */
static void scene_load_word(Scene *s) {
  static float targets_x[MAX_PARTICLES];
  static float targets_y[MAX_PARTICLES];
  static int color_slots[MAX_PARTICLES];

  int n_targets = scene_build_targets(s, WORDS[s->word_idx], targets_x,
                                      targets_y, color_slots, MAX_PARTICLES);

  particle_pool_grow_to(s, n_targets);
  particle_pool_shrink_to(s, n_targets);
  particle_pool_assign_targets(s, targets_x, targets_y, color_slots, n_targets);
}

/* ── the shove each scatter style gives a dot ── */

/* Works out which way a dot points away from the word's centre, used
 * by the explode and swirl styles. A dot sitting right on the centre
 * has no real direction, so we just pick a random one to avoid leaving
 * it stuck in place. */
static inline void particle_radial_unit(const Particle *p, float cx, float cy,
                                        uint32_t *rng, float *out_ux,
                                        float *out_uy) {
  float dx = p->px - cx;
  float dy = (p->py - cy) * ASPECT_Y;
  float r = sqrtf(dx * dx + dy * dy);
  if (r < 0.5f) {
    float ang = lcg_unit(rng) * 2.0f * (float)M_PI;
    dx = cosf(ang);
    dy = sinf(ang);
    r = 1.0f;
  }
  *out_ux = dx / r;
  *out_uy = dy / r;
}

/* EXPLODE — fling the dot straight outward, with a bit of speed variety. */
static inline void particle_kick_explode(Particle *p, float ux, float uy,
                                         uint32_t *rng) {
  float speed = EXPLODE_SPEED * (0.7f + lcg_unit(rng) * 0.6f);
  p->vx = ux * speed;
  p->vy = uy * speed / ASPECT_Y;
}

/* SWIRL — push the dot sideways instead of outward, so it spins around
 * the centre rather than away from it. */
static inline void particle_kick_swirl(Particle *p, float ux, float uy,
                                       uint32_t *rng) {
  float speed = SWIRL_SPEED * (0.7f + lcg_unit(rng) * 0.6f);
  p->vx = -uy * speed; /* turned 90 degrees from "straight out" */
  p->vy = ux * speed / ASPECT_Y;
}

/* RAIN — drop the dot downward with just a little sideways wobble. */
static inline void particle_kick_rain(Particle *p, uint32_t *rng) {
  p->vx = (lcg_unit(rng) - 0.5f) * 6.0f;
  p->vy = RAIN_SPEED * (0.7f + lcg_unit(rng) * 0.5f);
}

/* DRIFT — send the dot off in a fully random direction, like smoke. */
static inline void particle_kick_drift(Particle *p, uint32_t *rng) {
  p->vx = (lcg_unit(rng) - 0.5f) * 2.0f * DRIFT_SPEED;
  p->vy = (lcg_unit(rng) - 0.5f) * 2.0f * DRIFT_SPEED / ASPECT_Y;
}

/* ── breaking the word apart ── */

/* The moment the word dissolves: gives every dot the right shove for
 * the chosen scatter style. */
static void scene_apply_dissolve_velocity(Scene *s) {
  for (int i = 0; i < s->n_particles; i++) {
    Particle *p = &s->particles[i];
    if (!p->active)
      continue;

    float ux, uy;
    particle_radial_unit(p, s->word_cx, s->word_cy, &s->rng, &ux, &uy);

    switch (s->current_pattern) {
    case PATTERN_EXPLODE:
      particle_kick_explode(p, ux, uy, &s->rng);
      break;
    case PATTERN_SWIRL:
      particle_kick_swirl(p, ux, uy, &s->rng);
      break;
    case PATTERN_RAIN:
      particle_kick_rain(p, &s->rng);
      break;
    case PATTERN_DRIFT:
      particle_kick_drift(p, &s->rng);
      break;
    case N_PATTERNS:
      break;
    }
  }
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->current_theme = 0;
  s->current_pattern = PATTERN_EXPLODE;
  s->rng = (uint32_t)clock_ns();
  s->cols = cols;
  s->rows = rows;

  s->phase = PHASE_ASSEMBLE;
  s->phase_t = 0.0f;
  s->word_idx = 0;

  scene_clear_particles(s);
  scene_load_word(s);
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
  /* Re-place the word now that the screen is a different size. */
  scene_load_word(s);
}

static void scene_reseed(Scene *s) {
  s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
  s->word_idx = (s->word_idx + 1) % N_WORDS;
  s->phase = PHASE_ASSEMBLE;
  s->phase_t = 0.0f;
  scene_load_word(s);
}

/* ── moving one dot forward by a small time step ── */

/* The spring step, used while the word is assembling and holding. It
 * pulls the dot toward its spot and brakes it so it doesn't oscillate
 * forever. The vertical pull is eased because cells are tall, keeping
 * the word from looking squashed. While holding, tiny random nudges
 * make the word seem to breathe. We update speed first, then position
 * (the order that keeps the bouncing stable over long runs). */
static inline void particle_integrate_spring(Particle *p, float dt,
                                             bool holding, uint32_t *rng) {
  float fx = SPRING_K * (p->tx - p->px) - SPRING_D * p->vx;
  float fy = (SPRING_K * (p->ty - p->py)) / ASPECT_Y - SPRING_D * p->vy;
  if (holding) {
    fx += (lcg_unit(rng) - 0.5f) * HOLD_JITTER;
    fy += (lcg_unit(rng) - 0.5f) * HOLD_JITTER;
  }
  p->vx += fx * dt;
  p->vy += fy * dt;
  p->px += p->vx * dt;
  p->py += p->vy * dt;
}

/* The free-flight step, used while the word is scattered. No spring
 * here: the dot just coasts and slowly loses speed to drag. The rain
 * style also adds gravity so its dots fall. The caller works out the
 * drag amount once per frame and passes it in. */
static inline void particle_integrate_drift(Particle *p, float drag, float dt,
                                            bool gravity_on) {
  if (gravity_on)
    p->vy += RAIN_GRAVITY * dt;
  p->vx *= drag;
  p->vy *= drag;
  p->px += p->vx * dt;
  p->py += p->vy * dt;
}

/* ── the stage timer ── */

/* Counts down the current stage and moves to the next when its time is
 * up: assemble, then hold, then dissolve, then back to assemble. The
 * word, colour, and scatter style only change when the user presses a key. */
static void scene_advance_phase_machine(Scene *s) {
  switch (s->phase) {
  case PHASE_ASSEMBLE:
    if (s->phase_t >= ASSEMBLE_DUR) {
      s->phase = PHASE_HOLD;
      s->phase_t = 0.0f;
    }
    break;
  case PHASE_HOLD:
    if (s->phase_t >= HOLD_DUR) {
      s->phase = PHASE_DISSOLVE;
      s->phase_t = 0.0f;
      scene_apply_dissolve_velocity(s); /* shove every dot as we enter */
    }
    break;
  case PHASE_DISSOLVE:
    if (s->phase_t >= DISSOLVE_DUR) {
      /* Replay the same word: clear the scattered dots and let them
       * respawn fresh at the edges. The r/t/n keys are what pick a
       * different word, colour, or scatter style. */
      scene_clear_particles(s);
      scene_load_word(s);
      s->phase = PHASE_ASSEMBLE;
      s->phase_t = 0.0f;
    }
    break;
  }
}

/* Moves every scattered dot one step (used while dissolving). */
static void scene_step_drift_layer(Scene *s, float dt) {
  float drag = expf(-DISSOLVE_DRAG * dt);
  bool rain_gravity = (s->current_pattern == PATTERN_RAIN);
  for (int i = 0; i < s->n_particles; i++) {
    Particle *p = &s->particles[i];
    if (!p->active)
      continue;
    particle_integrate_drift(p, drag, dt, rain_gravity);
  }
}

/* Moves every dot one step toward its spot (used while assembling/holding). */
static void scene_step_spring_layer(Scene *s, float dt) {
  bool holding = (s->phase == PHASE_HOLD);
  for (int i = 0; i < s->n_particles; i++) {
    Particle *p = &s->particles[i];
    if (!p->active)
      continue;
    particle_integrate_spring(p, dt, holding, &s->rng);
  }
}

/* ── one step of the whole simulation ── */

/* Advances time by one step: scale it by the speed control, tick the
 * stage timer, then move the dots the right way for the current stage. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  dt *= (float)s->speed / (float)SPEED_DEF;
  s->phase_t += dt;

  scene_advance_phase_machine(s);

  if (s->phase == PHASE_DISSOLVE)
    scene_step_drift_layer(s, dt);
  else
    scene_step_spring_layer(s, dt);
}

/* ── choosing how each dot looks ── */

/* How far a dot is from its spot (squared, to skip a square root). We
 * use it as a quick gauge of whether the dot is parked, on its way, or
 * far gone. */
static inline float particle_target_dist_sq(const Particle *p) {
  float dx = p->tx - p->px;
  float dy = p->ty - p->py;
  return dx * dx + dy * dy;
}

/* Picks the character and brightness for a dot from how close it is:
 * a bright '*' when parked on its spot, a plain '+' while in flight,
 * and a faint '.' once it's scattered far away. */
static inline void particle_phase_glyph(const Particle *p, char *out_glyph,
                                        int *out_attr) {
  float d2 = particle_target_dist_sq(p);
  if (d2 < 1.0f) {
    *out_glyph = '*';
    *out_attr = A_BOLD;
  } else if (d2 < 25.0f) {
    *out_glyph = '+';
    *out_attr = A_NORMAL;
  } else {
    *out_glyph = '.';
    *out_attr = A_DIM;
  }
}

/* ── drawing the dots ── */

/* Paints every active dot: round its position to a cell, skip it if
 * it's off-screen or under the top status bar, then draw it in its
 * gradient colour with the character its distance picked. */
static void scene_draw(const Scene *s) {
  int rows_eff = s->rows - 1;
  for (int i = 0; i < s->n_particles; i++) {
    const Particle *p = &s->particles[i];
    if (!p->active)
      continue;

    int ix = (int)(p->px + 0.5f);
    int iy = (int)(p->py + 0.5f);
    if (ix < 0 || ix >= s->cols)
      continue;
    if (iy < 0 || iy >= rows_eff)
      continue;

    char glyph;
    int attr;
    particle_phase_glyph(p, &glyph, &attr);

    int pair = PAIR_RAMP_BASE + p->color_slot;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(iy, ix, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
  }
}

/* ── §7 screen ── */

/* Screen — the terminal's current size in columns and rows. */
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

static const char *phase_str(Phase p) {
  switch (p) {
  case PHASE_ASSEMBLE:
    return "ASSEMBLE";
  case PHASE_HOLD:
    return "HOLD    ";
  case PHASE_DISSOLVE:
    return "DISSOLVE";
  default:
    return "?       ";
  }
}

/*
 * Draws the dots, then lays two text bars on top: a status line along
 * the top (what's playing right now) and a key-hint line along the
 * bottom. Both bars get a full-width coloured background and are drawn
 * last so no flying dot pokes through them.
 */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(s);

  const char *state_str =
      s->paused ? "PAUSED " : pattern_name(s->current_pattern);

  /* Top bar: live status. */
  char status[220];
  snprintf(status, sizeof status,
           " PIXEL_DISSOLVE   %s   theme:%-8s   word:'%-8s'   phase:%s   "
           "N:%3d   %5.1f fps  %3d Hz  speed:%-3d ",
           state_str, themes[s->current_theme].name, WORDS[s->word_idx],
           phase_str(s->phase), s->n_particles, fps, sim_fps, s->speed);

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  for (int x = 0; x < sc->cols; x++)
    mvaddch(0, x, ' ');
  mvprintw(0, 0, "%s", status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* Bottom bar: the keys you can press. */
  const char *hints =
      " q:quit  spc:pause  r:next-word  n/p:pattern  t/T:theme  "
      "+/-:speed  ]/[:Hz ";

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

/* ── §8 app ── */

/*
 * App — ties everything together for the main loop.
 *   scene        the dots and their state
 *   screen       the terminal size
 *   sim_fps      how many simulation steps per second we aim for
 *   running      set to 0 by a quit key or signal to end the loop
 *   need_resize  set by the resize signal so the loop refits next frame
 * The two flags are touched from signal handlers, so they're marked
 * volatile sig_atomic_t to stay safe across that boundary.
 */
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
  case 27: /* the Escape key */
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
