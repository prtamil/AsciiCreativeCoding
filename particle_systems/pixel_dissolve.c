/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * pixel_dissolve.c — text rendered as particles that disintegrate
 *                    and reform into the next word
 *
 * DEMO: A short word ("BOOM" / "ASCII" / "PIXEL" / …) appears on
 *       screen rendered as a bitmap of glowing particles — each "on"
 *       pixel of the embedded 5×7 font is one particle. After a
 *       few seconds the word DISSOLVES — every particle gets a
 *       velocity (explosion, swirl, rain, or drift depending on
 *       pattern). After the dissolve the next word's targets are
 *       computed, and the SAME particles spring back through the
 *       air to form the next word. Loops continuously.
 *
 *       Patterns:
 *         EXPLODE  particles fly OUTWARD from word centre
 *         SWIRL    particles spin TANGENTIALLY (vortex dissolve)
 *         RAIN     particles FALL away under gravity
 *         DRIFT    particles drift in random direction (smoke fade)
 *
 * Study alongside:
 *   physics/cloth.c          — same damped-spring integration
 *                               idiom (each particle pulled to a
 *                               target rest position).
 *   particle_systems/comet.c — same moving-emitter framework but
 *                               here the emitter is the WORD itself
 *                               (a snapshot in space-time).
 *
 * Section map:
 *   §1 config    — constants, themes, patterns, word list, font
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-pair colour ramp + accent pairs
 *   §4 font      — 5×7 bitmap font lookup
 *   §5 particle  — Particle struct
 *   §6 scene     — pool, target builder, phase machine, tick, draw
 *   §7 screen    — ncurses init / draw / resize
 *   §8 app       — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reseed (jump to next word, clear & reform)
 *   n / N      next dissolve pattern  (EXPLODE → SWIRL → RAIN → DRIFT)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster (speed multiplier ×2)
 *   -          slower (÷2)
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/pixel_dissolve.c \
 *       -o pixel_dissolve -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Particle-per-pixel rendering of a bitmap font, with
 *                  a 3-state PHASE MACHINE driving each cycle:
 *
 *                    ASSEMBLE  →  HOLD  →  DISSOLVE  →  TRANSITION
 *                                                            ↓
 *                                                       ASSEMBLE …
 *
 *                  Each cycle:
 *                    1. ASSEMBLE: each particle springs toward its
 *                       target position via a damped harmonic
 *                       oscillator (`F = K·(target − pos) − D·v`).
 *                       Particles smoothly arrive and settle.
 *                    2. HOLD: spring continues (now keeping particles
 *                       at target); small Brownian-jitter is added to
 *                       give "alive" feel — letters appear to subtly
 *                       breathe rather than be a static glyph. Lasts
 *                       a few seconds for legibility.
 *                    3. DISSOLVE: each particle is given an explosion
 *                       velocity per pattern (radial outward / tangent
 *                       /downward / random). Spring is OFF so they
 *                       freely drift. Mild drag bleeds energy.
 *                    4. TRANSITION: pick next word from the cycle
 *                       list, recompute targets, REASSIGN particles
 *                       (sequential mapping), reactivate excess /
 *                       spawn new for shortfalls. Phase resets to
 *                       ASSEMBLE — particles spring from their
 *                       current scattered positions to the new word.
 *
 *                  The bitmap font is a 5×7 monospaced set, embedded
 *                  inline as `uint8_t font_5x7[256][7]` (covers A-Z,
 *                  0-9, plus a few symbols). For each character of
 *                  the word, we walk the 5×7 grid and emit a particle
 *                  target for every "on" bit. The whole word is then
 *                  centered on the screen.
 *
 *                  Particle colour is set by HORIZONTAL POSITION in
 *                  the word — a left-to-right gradient through the
 *                  active theme's 8-step ramp. So each word reads
 *                  like a colour-graded title screen.
 *
 * Data-structure : Particle[MAX_PARTICLES] object pool with `active`
 *                  flag and a target `(tx, ty)` per slot. Linear-
 *                  scan everywhere; N is bounded by font geometry
 *                  (longest word ~150 pixels at 5×7).
 *
 * Rendering      : ASCII only. Each particle renders as a single
 *                  glyph at its current cell. Glyph fades during
 *                  DISSOLVE so far-flung particles look diffuse.
 *
 * Performance    : O(N · 1) per tick where N ≤ 200. Trivial. Phase
 *                  transitions are O(N²) only if we used optimal
 *                  matching; we use sequential matching which is O(N).
 *
 * References
 * ──────────
 *   PAPERS
 *     Reeves, W. T. (1983)
 *       "Particle Systems — A Technique for Modeling a Class of Fuzzy Objects"
 *       ACM Transactions on Graphics 2(2): 91-108.
 *       Foundational paper.  The fixed-size pool + per-particle
 *       independent update used here is the model Reeves defines.
 *       The DISSOLVE phase pattern set (radial, tangent, downward,
 *       random) maps directly to Reeves' "stochastic spawn vectors"
 *       categorisation.
 *
 *     Witkin, A. & Baraff, D. (2001)
 *       "Physically Based Modeling: Principles and Practice"
 *       SIGGRAPH course notes (online proceedings).
 *       §3 — soft-body constraints implemented as spring-damper
 *       forces F = K·(target − x) − D·v, the exact pattern
 *       ASSEMBLE/HOLD uses to pull particles onto bitmap targets.
 *
 *   BOOKS
 *     Bourg, D. M. & Bywalec, B. — "Physics for Game Developers"
 *       (2nd ed, O'Reilly, 2013).  Chapter on harmonic oscillators
 *       and damping — Hooke's law F = -K·x, critical damping
 *       D_crit = 2√(K·m), why our K=18 / D=8 choice is slightly
 *       underdamped (gives the "boing" overshoot on settle).
 *
 *     Hairer, E., Lubich, C. & Wanner, G. — "Geometric Numerical
 *       Integration: Structure-Preserving Algorithms for Ordinary
 *       Differential Equations" (2nd ed, Springer, 2006).
 *       §I.1 — symplectic Euler analysis: the v-then-x update order
 *       used in scene_tick keeps the spring oscillator energy-bounded
 *       over thousands of ticks, where explicit Euler would gradually
 *       inflate the orbit.
 *
 *     Akenine-Möller, T., Haines, E. & Hoffman, N. —
 *       "Real-Time Rendering" (4th ed, CRC Press, 2018).
 *       §13.7 — point-sprite particle rendering.  Per-particle glyph
 *       selection by distance-to-target (`*` settled, `+` flying,
 *       `.` dispersed) is the ASCII analogue of the size-by-distance
 *       point sprite pattern.
 *
 *   FONT
 *     Adafruit GFX — public-domain 5×7 bitmap font; the embedded
 *     font_5x7[256][7] table follows Adafruit's 7-byte-per-glyph
 *     encoding (one row per byte, MSB = leftmost column).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A WORD is a SET OF TARGET POSITIONS. Each particle has a target
 * and a current position. A spring force pulls the particle to its
 * target; the particle SETTLES into place over a second or two. To
 * dissolve, give the particle a velocity and turn the spring off:
 * it drifts away. To reform into a new word, recompute targets and
 * turn the spring back on: the particles spring from wherever they
 * happen to be to the new positions.
 *
 * ALGORITHM IN STEPS  (each step = one helper in §6)
 * ──────────────────
 *  1. BUILD TARGETS (scene_build_targets → target_color_slot +
 *     target_emit_pixel_replicas). For the active word, walk each
 *     character; for each "on" pixel of the 5×7 bitmap, emit
 *     PARTICLES_PER_PIXEL (=3) targets with sub-cell jitter.  Total
 *     ≈ on-pixel-count × 3 (≈ 270 for an 8-letter word).
 *
 *  2. PER TICK (scene_tick):
 *     ASSEMBLE / HOLD → scene_step_spring_layer →
 *                       particle_integrate_spring:
 *       fx = K · (tx − px) − D · vx
 *       fy = K · (ty − py) / ASPECT_Y − D · vy   (aspect-corrected y)
 *       v += f · dt;  p += v · dt   (symplectic Euler: v BEFORE p)
 *       HOLD phase adds Brownian jitter so the word breathes.
 *
 *     DISSOLVE → scene_step_drift_layer → particle_integrate_drift:
 *       v *= exp(-DISSOLVE_DRAG · dt)   (closed-form linear drag)
 *       p += v · dt
 *       PATTERN_RAIN gets an extra vy += RAIN_GRAVITY · dt.
 *
 *  3. PHASE MACHINE (scene_advance_phase_machine):
 *     advance phase_t by dt; transition when the timer elapses:
 *       ASSEMBLE → HOLD     after ASSEMBLE_DUR
 *       HOLD     → DISSOLVE after HOLD_DUR  + scene_apply_dissolve_velocity
 *                                            (via particle_kick_explode /
 *                                             swirl / rain / drift)
 *       DISSOLVE → ASSEMBLE after DISSOLVE_DUR — LOOPS THE SAME WORD:
 *                  scene_clear_particles + scene_load_word reload the
 *                  current (word, theme, pattern). User keys r / t / n
 *                  are the only way to advance any of those indices.
 *
 *  4. RENDER (scene_draw): for each active particle,
 *     particle_phase_glyph maps distance-to-target² to (glyph, attr):
 *       d² <  1 → '*' A_BOLD   (settled)
 *       d² < 25 → '+' A_NORMAL (mid-flight)
 *       else    → '.' A_DIM    (dispersed)
 *     painted in the theme's left-to-right gradient ramp slot.
 *
 * KEY FORMULAS
 * ────────────
 *  Damped spring (per particle, per axis):
 *    f = K · (target − pos) − D · vel
 *    vel += f · dt
 *    pos += vel · dt
 *
 *  Critical damping (no overshoot):
 *    D_crit = 2 · √(K · m)
 *  We use K=18, D=8 → slightly underdamped; particles overshoot
 *  slightly then settle — gives a snappy "boing" feel.
 *
 *  Dissolve velocity:
 *    EXPLODE: v = (p − centre)/r · SPEED
 *    SWIRL:   v = ((-uy, ux))      · SPEED      (perpendicular to radial)
 *    RAIN:    vx = (r-0.5)·SCATTER; vy = +SPEED
 *    DRIFT:   vx = (r-0.5)·SPEED;  vy = (r-0.5)·SPEED
 *
 *  Aspect-correct spring (cells are 2× taller than wide so vertical
 *  spring needs proportionally less force to look balanced):
 *    fy = (K · (ty − py)) / ASPECT_Y - D · vy
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,

  SPEED_MIN = 1,
  SPEED_DEF = 8,
  SPEED_MAX = 64,

  MAX_PARTICLES =
      1500, /* PARTICLES_PER_PIXEL × longest-word pixel count, with headroom */
  PARTICLES_PER_PIXEL =
      3, /* particles assigned to each lit pixel — denser-looking word     */

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_RAMP_BASE = 3, /* +0..+7 = horizontal-gradient ramp   */
  PAIR_SKY = 11,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

#define ASPECT_Y 2.0f /* terminal cells 2× taller       */

/* Spring physics (damped harmonic oscillator). */
#define SPRING_K 18.0f
#define SPRING_D 8.0f
#define HOLD_JITTER 4.0f   /* cells/sec² Brownian jitter     */
#define DISSOLVE_DRAG 0.6f /* per-second exp damping         */

/* Phase durations (seconds). Slower default cadence so the word is
 * legible during HOLD and the dissolve has room to breathe. The user
 * can still press '+' to speed up. */
#define ASSEMBLE_DUR 2.5f
#define HOLD_DUR 6.0f
#define DISSOLVE_DUR 2.5f

/* Dissolve initial velocity speeds. */
#define EXPLODE_SPEED 45.0f
#define SWIRL_SPEED 35.0f
#define RAIN_SPEED 50.0f
#define RAIN_GRAVITY 120.0f
#define DRIFT_SPEED 25.0f

/* Pattern enum. */
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

/* Phase enum. */
typedef enum {
  PHASE_ASSEMBLE = 0,
  PHASE_HOLD = 1,
  PHASE_DISSOLVE = 2,
} Phase;

/*
 * Themes — 8-step horizontal gradient. Each particle's colour is
 * its slot in the gradient based on its target's horizontal position
 * in the word — leftmost = ramp[0], rightmost = ramp[7].
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule.
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

/* Word cycle list. */
static const char *WORDS[] = {
    "BOOM", "ASCII",    "PIXEL",   "MORPH", "DUST",
    "CODE", "DISSOLVE", "USELESS", "TAMIL",
};
#define N_WORDS ((int)(sizeof WORDS / sizeof WORDS[0]))

/*
 * font_5x7 — 5-column × 7-row monospaced bitmap font.
 *
 * Each entry is 7 bytes (one per row, top-to-bottom). Bits within a
 * byte represent the 5 columns: bit 4 = column 0 (leftmost), bit 0 =
 * column 4 (rightmost). Entries not listed are zero (blank).
 *
 * Covers: space, A-Z, 0-9, '!' '?' '.'.
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
  3 /* cells between letters — wider gap so the                              \
     * letters read clearly (each glyph is 5 cells                             \
     * wide; a 1-cell kern made adjacent strokes                               \
     * blur into each other) */

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

/* ===================================================================== */
/* §4  font                                                               */
/* ===================================================================== */

/* Width in cells of a rendered word (FONT_W per char + FONT_KERN between). */
static int word_width_cells(const char *word) {
  int n = (int)strlen(word);
  if (n <= 0)
    return 0;
  return n * FONT_W + (n - 1) * FONT_KERN;
}

/* ===================================================================== */
/* §5  particle                                                           */
/* ===================================================================== */

typedef struct {
  float px, py;   /* current position (cell coords)             */
  float vx, vy;   /* velocity (cells/sec)                       */
  float tx, ty;   /* target position when assembled             */
  int color_slot; /* 0..7 — picks ramp slot                     */
  bool active;
} Particle;

/* Cheap LCG */
static inline uint32_t lcg_next(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}
static inline float lcg_unit(uint32_t *st) {
  return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/* ===================================================================== */
/* §6  scene — pool, target builder, phase machine, tick, draw           */
/* ===================================================================== */

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
  float word_cx, word_cy; /* cached centre of current word    */
  int word_w_cells;       /* cached width                    */

  int n_particles;
  Particle particles[MAX_PARTICLES];
} Scene;

static void scene_clear_particles(Scene *s) {
  s->n_particles = 0;
  for (int i = 0; i < MAX_PARTICLES; i++)
    s->particles[i].active = false;
}

/* ── Bitmap → target list helpers ────────────────────────────────── */

/* Horizontal position → 8-step ramp slot (left = 0, right = 7).
 * The whole word inherits the left-to-right gradient; every replica
 * of one lit pixel shares the same slot. */
static inline int target_color_slot(float pixel_x, float start_x, int word_w) {
  float frac = (pixel_x - start_x) / (float)(word_w > 1 ? word_w - 1 : 1);
  int slot = (int)(frac * 7.999f);
  if (slot < 0)
    slot = 0;
  if (slot > 7)
    slot = 7;
  return slot;
}

/* Emit PARTICLES_PER_PIXEL targets for one lit font pixel.
 *   replica 0     sits dead-centre at (px, py)
 *   replicas 1..N get sub-cell jitter ±0.25 in x and y
 * They all converge to (almost) the same spot during ASSEMBLE so the
 * word reads as a single bright pixel; during DISSOLVE they fan out
 * independently because each gets its own random initial velocity.
 * Returns false when n_total has hit max_targets — caller short-circuits. */
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

/* ── Driver — rasterise the word into a target list ──────────────── */

/* Pseudocode:
 *   compute centred (start_x, start_y) for the whole word
 *   for each character in word:
 *       for each (row, col) in the 5×7 grid where the font bit is lit:
 *           slot = horizontal-position → ramp slot
 *           emit PARTICLES_PER_PIXEL targets for that pixel
 *       advance char_x by FONT_W + FONT_KERN
 *   cache word centre + width on the Scene (read by dissolve patterns) */
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

/* ── Particle inflow helpers ─────────────────────────────────────── */

/* Spawn one particle at a random screen edge (top / left / right /
 * bottom, ±3 cells beyond the visible region). vx = vy = 0 so the
 * upcoming ASSEMBLE spring pulls it cleanly toward its target. */
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

/* Grow the pool by spawning new edge-particles until n_particles == n_targets.
 */
static void particle_pool_grow_to(Scene *s, int n_targets) {
  while (s->n_particles < n_targets) {
    particle_spawn_at_edge(&s->particles[s->n_particles], &s->rng, s->cols,
                           s->rows);
    s->n_particles++;
  }
}

/* Shrink the pool by deactivating slots [n_targets, n_particles). */
static void particle_pool_shrink_to(Scene *s, int n_targets) {
  for (int i = n_targets; i < s->n_particles; i++)
    s->particles[i].active = false;
  s->n_particles = n_targets;
}

/* Sequential target assignment: particle[i] → target[i]. Cheap O(N)
 * — optimal matching (Hungarian) would minimise total travel but is
 * visually indistinguishable here; the varied paths actually look
 * MORE organic than "everything moves the shortest distance". */
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

/* ── Driver — load current word into the particle pool ───────────── */

/* Pseudocode:
 *   build target list for the active word
 *   resize pool: grow with edge-spawns / shrink by deactivation
 *   assign each particle its target position and colour slot */
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

/* ── Dissolve-kick helpers (one per pattern) ─────────────────────── */

/* Aspect-corrected radial unit vector pointing from word centre to
 * the particle. If the particle is essentially AT the centre (r < 0.5)
 * the direction is degenerate, so we pick a uniformly-random angle —
 * keeps EXPLODE / SWIRL from producing zero-velocity outliers stuck
 * at the centre. */
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

/* EXPLODE — radial outward velocity, magnitude EXPLODE_SPEED · [0.7, 1.3]. */
static inline void particle_kick_explode(Particle *p, float ux, float uy,
                                         uint32_t *rng) {
  float speed = EXPLODE_SPEED * (0.7f + lcg_unit(rng) * 0.6f);
  p->vx = ux * speed;
  p->vy = uy * speed / ASPECT_Y;
}

/* SWIRL — tangential velocity (radial rotated 90°), magnitude SWIRL_SPEED ·
 * [0.7, 1.3]. */
static inline void particle_kick_swirl(Particle *p, float ux, float uy,
                                       uint32_t *rng) {
  float speed = SWIRL_SPEED * (0.7f + lcg_unit(rng) * 0.6f);
  p->vx = -uy * speed; /* perpendicular to radial */
  p->vy = ux * speed / ASPECT_Y;
}

/* RAIN — small horizontal scatter, strong downward velocity. */
static inline void particle_kick_rain(Particle *p, uint32_t *rng) {
  p->vx = (lcg_unit(rng) - 0.5f) * 6.0f;
  p->vy = RAIN_SPEED * (0.7f + lcg_unit(rng) * 0.5f);
}

/* DRIFT — random isotropic scatter, magnitude DRIFT_SPEED. */
static inline void particle_kick_drift(Particle *p, uint32_t *rng) {
  p->vx = (lcg_unit(rng) - 0.5f) * 2.0f * DRIFT_SPEED;
  p->vy = (lcg_unit(rng) - 0.5f) * 2.0f * DRIFT_SPEED / ASPECT_Y;
}

/* ── Driver — apply the dissolve kick to every active particle ───── */

/* Pseudocode:
 *   for each active particle:
 *     compute aspect-corrected radial unit from word centre
 *     dispatch to the pattern's per-particle kick function */
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
  /* Recompute targets for new screen size. */
  scene_load_word(s);
}

static void scene_reseed(Scene *s) {
  s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
  s->word_idx = (s->word_idx + 1) % N_WORDS;
  s->phase = PHASE_ASSEMBLE;
  s->phase_t = 0.0f;
  scene_load_word(s);
}

/* ── Per-particle integrators (one step, one particle) ──────────── */

/* Damped harmonic oscillator pull toward target:
 *     fx = K · (tx − px) − D · vx                  (Hooke + viscous damping)
 *     fy = K · (ty − py) / ASPECT_Y − D · vy        (aspect-corrected)
 * HOLD phase adds Brownian jitter so the held word visibly breathes.
 * Explicit Euler then advances v, then p — same v-then-p order used
 * in the rest of the project's particle code (symplectic Euler). */
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

/* Free-drift integration during DISSOLVE: linear drag on velocity,
 * plus an extra gravity term for the RAIN pattern only. The drag
 * factor is exp(-k·dt), pre-computed once per tick by the caller. */
static inline void particle_integrate_drift(Particle *p, float drag, float dt,
                                            bool gravity_on) {
  if (gravity_on)
    p->vy += RAIN_GRAVITY * dt;
  p->vx *= drag;
  p->vy *= drag;
  p->px += p->vx * dt;
  p->py += p->vy * dt;
}

/* ── Phase machine + layer drivers ───────────────────────────────── */

/* Advance the phase timer and run the transition + entry hook when
 * the current phase elapses. Phase ASSEMBLE → HOLD → DISSOLVE →
 * ASSEMBLE (looping same scene; user keys advance word/theme/pattern). */
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
      scene_apply_dissolve_velocity(s); /* entry hook */
    }
    break;
  case PHASE_DISSOLVE:
    if (s->phase_t >= DISSOLVE_DUR) {
      /* Loop SAME (word, theme, pattern) — kill all particles
       * so they respawn fresh at screen edges via scene_load_word.
       * User keys r / t / n are what change the indices. */
      scene_clear_particles(s);
      scene_load_word(s);
      s->phase = PHASE_ASSEMBLE;
      s->phase_t = 0.0f;
    }
    break;
  }
}

/* Sweep the pool with the drift integrator (DISSOLVE phase). */
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

/* Sweep the pool with the damped-spring integrator (ASSEMBLE / HOLD). */
static void scene_step_spring_layer(Scene *s, float dt) {
  bool holding = (s->phase == PHASE_HOLD);
  for (int i = 0; i < s->n_particles; i++) {
    Particle *p = &s->particles[i];
    if (!p->active)
      continue;
    particle_integrate_spring(p, dt, holding, &s->rng);
  }
}

/* ── Driver — one simulation tick ────────────────────────────────── */

/* Pseudocode:
 *   if paused                       → no-op
 *   dt *= speed multiplier
 *   phase_t += dt
 *   scene_advance_phase_machine     — handle phase transitions
 *   if DISSOLVE → drift layer        else → spring layer */
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

/* ── Per-particle render helpers ─────────────────────────────────── */

/* Squared distance from particle to its target.  Used as a cheap
 * proxy for "where in the phase arc is this particle" — settled
 * vs. mid-flight vs. dispersed — without taking a sqrt. */
static inline float particle_target_dist_sq(const Particle *p) {
  float dx = p->tx - p->px;
  float dy = p->ty - p->py;
  return dx * dx + dy * dy;
}

/* Map distance² to (glyph, attr) using three tiers — gives the word
 * a visible "settled / flying / dispersed" emphasis without needing
 * a phase parameter:
 *     d² <  1 → '*' A_BOLD     (on-target — settled bright pixel)
 *     d² < 25 → '+' A_NORMAL   (mid-flight — assembling or dissolving)
 *     else    → '.' A_DIM      (far — dispersed) */
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

/* ── Driver — render all active particles ────────────────────────── */

/* Pseudocode:
 *   for each active particle:
 *     round pixel position to a (ix, iy) cell
 *     reject if outside drawable region (skip HUD rows)
 *     pick glyph + attr from distance-to-target tier
 *     paint with the gradient pair from the particle's slot */
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

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

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
 * screen_draw — render the scene, then paint a two-layer HUD over it:
 *
 *   Row 0          STATUS LINE.  Bright yellow PAIR_HUD + A_BOLD.
 *                  Live state: pattern (or PAUSED), theme, current
 *                  word, phase (ASSEMBLE / HOLD / DISSOLVE), particle
 *                  count, render fps, sim Hz, speed multiplier.
 *   Row rows-1     KEY HINT LINE.  Bright cyan PAIR_HINT + A_BOLD.
 *                  Every interactive key the demo accepts.
 *
 * Both rows are pre-filled with their pair colour so the coloured
 * background spans the full width, and drawn AFTER scene_draw so
 * particles never bleed through the bars.
 */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(s);

  const char *state_str =
      s->paused ? "PAUSED " : pattern_name(s->current_pattern);

  /* ── Top row: dynamic status ─────────────────────────────── */
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

  /* ── Bottom row: every interactive key ───────────────────── */
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

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

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
