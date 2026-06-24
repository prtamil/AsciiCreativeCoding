/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * charged_particles.c — charged dots that push and pull on each other.
 *
 * Like charges shove each other apart; opposite charges pull together,
 * so they orbit, slingshot, and pair up. Every dot drags a fading
 * comet trail behind it so you can see the paths. Four starting setups:
 * a clean two-body orbit, a chaotic three-body dance, a fixed dipole
 * with test particles drifting through, and a 40-particle gas.
 *
 * Companion files: physics/barnes_hut.c does the same many-body motion
 * with gravity (which only ever pulls); here charge sign decides push
 * vs pull. vortex.c is force-driven motion too, but around one fixed
 * centre instead of every pair.
 *
 * The pull/push law and the choices around it come from a few books;
 * the [n] markers below point at the reference list at the bottom of
 * this header.
 */

/* ── CONCEPTS ── *
 *
 * The idea: every pair of charges pushes or pulls along the line
 * between them, with a strength that drops off fast as they get
 * farther apart (Coulomb's law — the famous inverse-square law).
 * Like signs repel, opposite signs attract. Each step we add up the
 * force on a particle from all the others, nudge its speed, and move
 * it. This is the same many-body trick from [3].
 *
 * Two guards keep it from blowing up:
 *   - Softening: when two particles get very close the force would
 *     shoot to infinity, so we pretend they can't be nearer than a
 *     minimum distance. Keeps the motion sane [3].
 *   - A touch of drag each step. The simple stepping we use slowly
 *     fakes extra energy into orbits over time [4]; the drag (plus a
 *     little speed loss on every wall bounce) quietly bleeds it back
 *     off so particles don't fly off-screen.
 *
 * Walls: particles bounce off the screen edges, losing a little speed
 * each time so the fastest ones calm down instead of speeding up.
 *
 * Trails: each particle remembers its last few positions and draws
 * them faintly behind itself, so orbits show up without any extra
 * field-line overlay.
 *
 * The four setups only differ in what particles you start with —
 * where they are, how fast, what charge. The stepping is identical for
 * all four. The three-body one is the electric cousin of the classic
 * three-body problem [2]: tiny changes in the start send it down wildly
 * different paths.
 *
 * One subtlety: terminal cells are about twice as tall as they are
 * wide, so we do all the physics in a stretched "physical" space where
 * up and down count double. That keeps orbits round instead of
 * squashed into ovals; we squash back to cell coordinates only when
 * drawing.
 *
 * Storage: a fixed array of particles, each carrying a small ring of
 * past positions for its trail. N is small, so checking every pair
 * each step is plenty fast.
 *
 * Drawing is ASCII only: '+' for positive, '-' for negative, '.' for
 * near-neutral, '@' for the heavy fixed anchors. Colour shows charge —
 * a diverging palette [5] running from one hue at most-negative through
 * a neutral middle to the opposite hue at most-positive. Trails fade
 * through five glyphs; free heads get a small glow halo; a wall hit
 * flashes a bright '*'; fast or heavy heads draw bold.
 *
 * Speed-wise this is trivial — at 40 particles and 60 fps it's about
 * 96k pair-forces a second. Barnes-Hut (physics/barnes_hut.c) would be
 * faster for big N but is overkill here.
 *
 * References:
 *
 *   [1] Griffiths, D. J. — Introduction to Electrodynamics, 4th ed.,
 *       Cambridge Univ. Press (2017). Ch. 2 is Coulomb's law and the
 *       rule that forces from many charges just add up — the physics
 *       behind the force loop.
 *
 *   [2] Goldstein, Poole, Safko — Classical Mechanics, 3rd ed.,
 *       Addison-Wesley (2002). §3 gives the orbit speed used to launch
 *       the two-body setup; §11 covers why the three-body one is
 *       chaotic with no neat solution.
 *
 *   [3] Hockney & Eastwood — Computer Simulation Using Particles,
 *       McGraw-Hill (1981, IOP 1988). The standard reference for
 *       many-body force stepping, softening near zero distance, and
 *       reusing a fixed particle pool.
 *
 *   [4] Hairer, Lubich, Wanner — Geometric Numerical Integration,
 *       2nd ed., Springer (2006). Explains why our simple stepping
 *       slowly gains energy and why a little damping is a cheap fix
 *       for a visual demo.
 *
 *   [5] Ware, C. — Information Visualization: Perception for Design,
 *       4th ed., Morgan Kaufmann (2020). Ch. 4 on diverging colour
 *       ramps for plus/minus data — the basis of the themes in §1.
 */

/* ── MENTAL MODEL ── *
 *
 * The whole thing in one breath: every charge pulls or pushes every
 * other charge, harder when they're close, along the line between
 * them. Add up all those pulls on a particle, divide by its weight to
 * get acceleration, and step it forward. The four setups differ only
 * in what you start with.
 *
 * Each step does:
 *   1. Forces. For each movable particle, sum the push/pull from every
 *      other one. (Fixed anchors push on others but never move
 *      themselves.)
 *   2. Speed update. Nudge velocity by the force, shave a bit off for
 *      drag, and cap it so a near-collision can't fling it across the
 *      screen.
 *   3. Move. Advance position by velocity.
 *   4. Bounce off the walls, losing a little speed each hit.
 *   5. Record the new position into the trail ring.
 *   6. Draw trail, then halo, then the head on top.
 *
 * Worth keeping in mind:
 *   - Orbit launch speed for the two-body setup is v = sqrt(k/(2·m·d))
 *     — the speed that exactly balances the pull into a circle.
 *   - Charge maps to a colour slot 0..7 (most-negative to most-positive,
 *     neutral in the middle).
 *   - Physical y is cell-y doubled, so distances come out round (see
 *     the note in the CONCEPTS block).
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

  MAX_PARTICLES = 80,
  TRAIL_LEN = 30,          /* how many past positions each trail keeps —
                            * longer makes orbits easier to see       */
  BOUNCE_FLASH_FRAMES = 6, /* frames a head flashes bright after a hit */
  HUD_TOP = 1,             /* rows reserved for the HUD: status at top */
  HUD_BOT = 1,             /*                            keys at bottom */

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* Colour-pair slots. The first two are the shared HUD colours. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_RAMP_BASE = 3, /* slots 3..10 hold the 8-step charge palette */
  PAIR_TRAIL = 11,
  PAIR_SKY = 12,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

#define ASPECT_Y 2.0f /* a cell is ~2x taller than wide; we stretch y by
                       * this so distances come out round */

/* Knobs for the push/pull law and the stepping. */
#define COULOMB_K 8000.0f
#define R_SOFT_MIN 2.5f     /* closest two particles may act — avoids the
                            * blow-up when they nearly touch */
#define VELOCITY_DAMP 0.15f /* gentle drag that bleeds off fake energy */
#define BOUNCE_REST 0.85f   /* speed kept after a wall bounce (< 1 = loses) */
#define MAX_VELOCITY 400.0f /* hard speed cap so nothing teleports */
#define VBOLD_THRESHOLD                                                        \
  60.0f /* a head this fast or faster draws bold */

typedef enum {
  PATTERN_BINARY = 0,
  PATTERN_TRINARY = 1,
  PATTERN_DIPOLE_FIELD = 2,
  PATTERN_RANDOM_GAS = 3,
  N_PATTERNS = 4,
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_BINARY:
    return "BINARY      ";
  case PATTERN_TRINARY:
    return "TRINARY     ";
  case PATTERN_DIPOLE_FIELD:
    return "DIPOLE_FIELD";
  case PATTERN_RANDOM_GAS:
    return "RANDOM_GAS  ";
  default:
    return "?           ";
  }
}

/*
 * Theme — one colour scheme for charge.
 *
 * The 8-step ramp runs from most-negative (slot 0) through a neutral
 * middle (slots 3-4) to most-positive (slot 7). It's a diverging
 * palette: one hue for negative, the opposite hue for positive, so the
 * sign of a charge is obvious at a glance. All colours sit in the
 * bright half of the 256-colour set so nothing vanishes on a dark
 * terminal.
 *
 *   name  — label shown in the HUD
 *   ramp  — the 8 charge colours, most-negative first
 *   trail — a neutral trail colour the theme loads (heads still colour
 *           their own trails by charge)
 *   sky   — a background tint the theme loads
 */
typedef struct {
  const char *name;
  short ramp[8];
  short trail;
  short sky;
} Theme;

#define N_THEMES 12

/*
 * The themes. Each runs one hue for negative charge, the opposite hue
 * for positive, with a bright neutral in the middle — so you can always
 * tell a charge's sign by colour [5]. Some go cool-to-warm in two
 * hues, some pass through a third hue in the middle; MONO is a plain
 * grayscale fallback. Every colour is kept bright enough to stay
 * visible on a dark terminal.
 */
static const Theme themes[N_THEMES] = {
    /* name, the 8 charge colours (most-neg first), trail colour, sky */

    {"VOLT",
     {24, 33, 39, 117, 230, 220, 214, 196},
     246,
     240}, /* electric blue → yellow → red (tri)     */
    {"COPPER",
     {25, 32, 67, 110, 230, 222, 214, 202},
     246,
     240}, /* steel blue → cream → copper red (di)   */
    {"NEON",
     {27, 39, 87, 195, 230, 213, 207, 199},
     246,
     240}, /* blue → cyan → pink (tri)               */
    {"ICE_FIRE",
     {24, 31, 39, 87, 224, 209, 202, 196},
     246,
     240}, /* ice blue → flame red (di, classic)     */
    {"AURORA",
     {28, 34, 79, 159, 224, 213, 207, 199},
     246,
     240}, /* green → pink (di, aurora-like)         */
    {"VIOLET",
     {53, 90, 134, 213, 224, 223, 215, 196},
     246,
     240}, /* violet → red (di)                      */
    {"CYBER",
     {28, 34, 121, 195, 230, 219, 207, 197},
     246,
     240}, /* green → cyan → red (tri)               */
    {"PASTEL",
     {110, 117, 153, 195, 224, 217, 218, 211},
     246,
     240}, /* soft blue → soft pink (di, low contrast) */
    {"TWILIGHT",
     {54, 60, 97, 104, 218, 211, 209, 196},
     246,
     240}, /* indigo → coral (di)                    */
    {"SODIUM",
     {130, 166, 172, 215, 195, 87, 51, 39},
     246,
     240}, /* sodium amber → mercury cyan (di)       */
    {"ECLIPSE",
     {240, 244, 247, 250, 196, 202, 208, 226},
     246,
     240}, /* shadow gray → corona red (di)          */
    {"MONO",
     {240, 243, 245, 247, 249, 251, 253, 255},
     246,
     240}, /* grayscale reference                    */
};

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
    init_pair(PAIR_TRAIL, t->trail, -1);
    init_pair(PAIR_SKY, t->sky, -1);
  } else {
    static const short fb[8] = {
        COLOR_BLUE,  COLOR_BLUE,   COLOR_CYAN, COLOR_WHITE,
        COLOR_WHITE, COLOR_YELLOW, COLOR_RED,  COLOR_RED,
    };
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_RAMP_BASE + i), fb[i], -1);
    init_pair(PAIR_TRAIL, COLOR_WHITE, -1);
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

/* Turn a charge (about -2..+2) into a colour slot 0..7. */
static inline int charge_to_slot(float q) {
  int slot = (int)(3.5f + q * 1.5f + 0.5f);
  if (slot < 0)
    slot = 0;
  if (slot > 7)
    slot = 7;
  return slot;
}

/* ── §4 particle ── */

/*
 * Particle — one charged dot.
 *
 * Positions live in a stretched "physical" space: px is the cell
 * column, but py is the cell row times ASPECT_Y (2). Cells are about
 * twice as tall as wide, so this stretch makes a step up count the same
 * distance as a step sideways — which keeps orbits round and lets the
 * force loop just write r² = dx² + dy². We squash py back to a real
 * row only when drawing.
 *
 * Motion uses the simplest possible stepping: nudge velocity by force,
 * then nudge position by velocity. It's not energy-perfect [4] — orbits
 * slowly gain energy — but the small drag each step plus the speed loss
 * on each wall bounce cancel that out, and over a few seconds it looks
 * just as real as fancier methods, with no extra state to carry.
 *
 * The 'fixed' flag marks an anchor: it still pushes and pulls on
 * everyone else, but never moves itself. Used only for the two heavy
 * +5/-5 anchors in the dipole setup.
 *
 * Each particle carries a small ring of its recent positions for the
 * trail. trail_head points at the slot to write next; trail_count is
 * how many of the slots are real (it stops growing once the ring is
 * full); the oldest sample is one slot past the head. scene_tick writes
 * a new sample each step; the drawing code only reads.
 *
 * bounce_age is the wall-flash timer. A wall hit sets it to
 * BOUNCE_FLASH_FRAMES; it counts down one per step. While it's above
 * zero the head and its halo flash a bright '*' — a quick "just smacked
 * the wall" cue.
 *
 * The push/pull law is Griffiths [1]; the orbit launch speed is
 * Goldstein [2]; the pair-by-pair stepping and softening are Hockney &
 * Eastwood [3]; the drag-to-cancel-drift trick is Hairer et al. [4].
 */
typedef struct {
  /* Where it is and how fast, advanced every step (physical coords). */
  float px, py; /* position (py is the row stretched by ASPECT_Y) */
  float vx, vy; /* velocity, physical units per second */

  /* What it is — set once at spawn. */
  float charge; /* sign picks push vs pull; size sets strength and colour */
  float mass;   /* weight; 1 for free dots, huge for fixed anchors */
  bool fixed;   /* true = an anchor: affects others but never moves */
  bool active;  /* false = this slot is empty */

  /* Recent positions for the trail (read newest-to-oldest when drawing). */
  float trail_px[TRAIL_LEN];
  float trail_py[TRAIL_LEN];
  int trail_head;  /* slot to write next */
  int trail_count; /* how many slots are real (caps at TRAIL_LEN) */

  /* Wall-bounce flash timer. */
  int bounce_age; /* > 0 = flash the head and halo bright; counts down */
} Particle;

/* Tiny fast random-number generator (good enough for scattering dots). */
static inline uint32_t lcg_next(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}
static inline float lcg_unit(uint32_t *st) {
  return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/* ── §5 scene ── */

/*
 * Scene — everything about one running simulation. There's a single
 * copy of it, tucked inside the file-wide App; helpers reach it through
 * a `Scene *s`.
 *
 * The fields fall into two groups, and the split is deliberate:
 *
 *   Simulation — anything that affects the motion: which setup is
 *     running, the speed multiplier, pause, the random seed, the
 *     chamber size the walls use, and the particles themselves. Changed
 *     by keys that touch the physics (pause, pattern, reseed, speed).
 *
 *   Rendering — just the chosen theme. Flipping it while paused must
 *     leave every particle exactly where it was; only the colours
 *     change. Changed by the theme keys.
 *
 * The split is for whoever reads this next, not for the machine. If you
 * add a field, ask whether it changes how things move or spawn. If yes
 * it's a simulation field; if it only changes how things look it's a
 * rendering field. Putting a motion flag in the rendering group is how
 * you accidentally let the display steer the physics.
 *
 * cols/rows live here because in this demo the walls ARE the screen
 * edges — the bounce code reads them directly, so it's handy to keep a
 * copy on the Scene instead of passing geometry into every call. A
 * resize updates them in place without disturbing the particles.
 *
 * A few related things live on App instead, on purpose: the quit and
 * resize flags (set by signal handlers, so they must sit at file scope
 * to be safe), the frame-rate setting (it controls how OFTEN we step,
 * not what a step does), and the ncurses copy of the screen size (kept
 * in sync with cols/rows on resize).
 */
typedef struct {
  /* Simulation. */
  Pattern current_pattern; /* which of the four setups is running */
  int speed;               /* time multiplier; +/- keys halve and double */
  bool paused;             /* space — freezes the motion */
  uint32_t rng;            /* random seed for spawning; 'r' refreshes it */
  int cols, rows;          /* chamber size in cells (= screen); walls use it */
  int n_particles;         /* how many pool slots are in use */
  Particle particles[MAX_PARTICLES];

  /* Rendering. */
  int current_theme; /* which colour scheme; t/T cycle it, never moves dots */
} Scene;

static void scene_clear_particles(Scene *s) {
  s->n_particles = 0;
  for (int i = 0; i < MAX_PARTICLES; i++) {
    s->particles[i].active = false;
    s->particles[i].fixed = false;
    s->particles[i].trail_head = 0;
    s->particles[i].trail_count = 0;
  }
}

/* Drop one particle into the pool; gives back its index, or -1 if full. */
static int scene_add_particle(Scene *s, float px, float py, float vx, float vy,
                              float charge, float mass, bool fixed) {
  if (s->n_particles >= MAX_PARTICLES)
    return -1;
  int idx = s->n_particles++;
  Particle *p = &s->particles[idx];
  p->px = px;
  p->py = py;
  p->vx = vx;
  p->vy = vy;
  p->charge = charge;
  p->mass = mass < 1e-3f ? 1e-3f : mass;
  p->fixed = fixed;
  p->active = true;
  p->trail_head = 0;
  p->trail_count = 0;
  /* Pre-fill trail with current position so first frames don't show
   * a long line trailing from (0, 0). */
  for (int t = 0; t < TRAIL_LEN; t++) {
    p->trail_px[t] = px;
    p->trail_py[t] = py;
  }
  return idx;
}

/* Middle of the chamber, in the stretched physical space. The
 * symmetric setups (binary, trinary, dipole) spawn around it. */
static void chamber_centre(const Scene *s, float *cx, float *cy) {
  int rows_eff = s->rows - 1;
  *cx = (float)s->cols * 0.5f;
  *cy = (float)rows_eff * 0.5f * ASPECT_Y;
}

/* Two opposite charges launched into a clean circular orbit around
 * their shared centre. The launch speed is the one that exactly
 * balances the pull into a circle [2]; at these numbers a loop takes
 * a few seconds, slow enough to watch the curve. */
static void init_pattern_binary(Scene *s) {
  float cx, cy;
  chamber_centre(s, &cx, &cy);
  float d = 25.0f;
  float v = sqrtf(COULOMB_K / (2.0f * 1.0f * d));
  scene_add_particle(s, cx - d * 0.5f, cy, 0.0f, -v, +1.0f, 1.0f, false);
  scene_add_particle(s, cx + d * 0.5f, cy, 0.0f, +v, -1.0f, 1.0f, false);
}

/* Three charges on the corners of a triangle, two +1 and one −2. The
 * lopsided −2 means there's no neat orbit to settle into, so it
 * tumbles into the chaotic three-body dance [2]. Launch speed is set a
 * bit low on purpose so it breaks up fast. */
static void init_pattern_trinary(Scene *s) {
  float cx, cy;
  chamber_centre(s, &cx, &cy);
  float radius = 18.0f;
  float v = 0.6f * sqrtf(COULOMB_K / (3.0f * radius));
  for (int k = 0; k < 3; k++) {
    float angle = (float)k * 2.0f * (float)M_PI / 3.0f;
    float pcx = cx + radius * cosf(angle);
    float pcy = cy + radius * sinf(angle);
    float vx = -v * sinf(angle); /* aim it sideways, going round */
    float vy = v * cosf(angle);
    float q = (k == 2) ? -2.0f : +1.0f; /* the symmetry-breaker */
    scene_add_particle(s, pcx, pcy, vx, vy, q, 1.0f, false);
  }
}

/* Two heavy fixed anchors, one +5 and one −5, with 30 light dots
 * drifting through the push and pull between them [1]. The anchors stay
 * put; the dots get shoved away from the matching-sign anchor and
 * pulled toward the opposite one, tracing the field by hand. */
static void init_pattern_dipole_field(Scene *s) {
  float cx, cy;
  chamber_centre(s, &cx, &cy);
  int rows_eff = s->rows - 1;
  float dipole_d = (float)s->cols * 0.45f;

  /* The two anchors. Mass is huge so they never budge. */
  scene_add_particle(s, cx - dipole_d * 0.5f, cy, 0, 0, +5.0f, 1e9f, true);
  scene_add_particle(s, cx + dipole_d * 0.5f, cy, 0, 0, -5.0f, 1e9f, true);

  /* 30 light dots scattered across the whole chamber. */
  for (int k = 0; k < 30; k++) {
    float r1 = lcg_unit(&s->rng);
    float r2 = lcg_unit(&s->rng);
    float r3 = lcg_unit(&s->rng);
    float r4 = lcg_unit(&s->rng);
    float r5 = lcg_unit(&s->rng);
    float pcx = r1 * (float)s->cols;
    float pcy = r2 * (float)rows_eff * ASPECT_Y;
    float vx = (r3 - 0.5f) * 8.0f;
    float vy = (r4 - 0.5f) * 8.0f;
    float q = (r5 < 0.5f) ? -1.0f : +1.0f;
    scene_add_particle(s, pcx, pcy, vx, vy, q, 1.0f, false);
  }
}

/* 40 random dots with mixed charges, dropped into the middle of the
 * chamber. With everything pushing and pulling on everything else they
 * pair up and fling each other out [3] — the "plasma" run. */
static void init_pattern_random_gas(Scene *s) {
  int rows_eff = s->rows - 1;
  for (int k = 0; k < 40; k++) {
    float r1 = lcg_unit(&s->rng);
    float r2 = lcg_unit(&s->rng);
    float r3 = lcg_unit(&s->rng);
    float r4 = lcg_unit(&s->rng);
    float r5 = lcg_unit(&s->rng);

    /* Keep them off the walls at the start so they aren't all
     * bouncing from frame one. */
    float pcx = (float)s->cols * (0.10f + r1 * 0.80f);
    float pcy = (float)rows_eff * ASPECT_Y * (0.10f + r2 * 0.80f);
    float vx = (r3 - 0.5f) * 8.0f;
    float vy = (r4 - 0.5f) * 8.0f;
    float q;
    if (r5 < 0.25f)
      q = -2.0f;
    else if (r5 < 0.50f)
      q = -1.0f;
    else if (r5 < 0.75f)
      q = +1.0f;
    else
      q = +2.0f;
    scene_add_particle(s, pcx, pcy, vx, vy, q, 1.0f, false);
  }
}

/* Wipe the pool and rebuild it for whichever setup is selected. */
static void scene_init_pattern(Scene *s) {
  scene_clear_particles(s);
  switch (s->current_pattern) {
  case PATTERN_BINARY:
    init_pattern_binary(s);
    break;
  case PATTERN_TRINARY:
    init_pattern_trinary(s);
    break;
  case PATTERN_DIPOLE_FIELD:
    init_pattern_dipole_field(s);
    break;
  case PATTERN_RANDOM_GAS:
    init_pattern_random_gas(s);
    break;
  case N_PATTERNS:
    break;
  }
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->current_theme = 0;
  s->current_pattern = PATTERN_BINARY;
  s->rng = (uint32_t)clock_ns();
  s->cols = cols;
  s->rows = rows;
  scene_init_pattern(s);
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
}

static void scene_reseed(Scene *s) {
  s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
  scene_init_pattern(s);
}

/* Add up the total push/pull on one particle from all the others [1].
 * Like charges come out repelling (away), opposite charges attracting
 * (toward). The force grows fast as two get close, so we never let the
 * distance drop below a floor — otherwise a near-collision would send
 * the force to infinity and blow the whole thing up [3]. */
static void sum_coulomb_force_on(const Scene *s, int i, float *fx_out,
                                 float *fy_out) {
  const Particle *pi = &s->particles[i];
  float r_soft2 = R_SOFT_MIN * R_SOFT_MIN;
  float fx = 0.0f, fy = 0.0f;

  for (int j = 0; j < s->n_particles; j++) {
    if (i == j)
      continue;
    const Particle *pj = &s->particles[j];
    if (!pj->active)
      continue;

    float dx = pj->px - pi->px;
    float dy = pj->py - pi->py;
    float r2 = dx * dx + dy * dy;
    if (r2 < r_soft2)
      r2 = r_soft2; /* don't let them get closer than the floor */
    float r = sqrtf(r2);

    float factor = -COULOMB_K * pi->charge * pj->charge / (r2 * r);
    fx += factor * dx;
    fy += factor * dy;
  }
  *fx_out = fx;
  *fy_out = fy;
}

/* Nudge the speed by the force (heavier things move less). This simple
 * stepping slowly fakes energy into orbits [4]; the drag below and the
 * speed loss on wall bounces quietly bleed it back off. */
static void apply_force_to_velocity(Particle *p, float fx, float fy, float dt) {
  p->vx += fx / p->mass * dt;
  p->vy += fy / p->mass * dt;
}

/* Shave a little off every speed each step — the gentle drag that
 * cancels the slow energy creep [4] and keeps the dots from flying
 * apart. The shrink factor is worked out once and just multiplied in. */
static void apply_velocity_drag(Particle *p, float damp) {
  p->vx *= damp;
  p->vy *= damp;
}

/* Hard speed limit. Even with the distance floor, a tight encounter
 * can still kick out a big speed; without a cap the dot would teleport
 * across the screen in one step. */
static void clamp_velocity_magnitude(Particle *p, float vmax) {
  float vmag2 = p->vx * p->vx + p->vy * p->vy;
  if (vmag2 > vmax * vmax) {
    float scale = vmax / sqrtf(vmag2);
    p->vx *= scale;
    p->vy *= scale;
  }
}

/* Move the dot by its speed. Runs after drag and the cap, so it moves
 * by the final, tamed speed. */
static void integrate_position(Particle *p, float dt) {
  p->px += p->vx * dt;
  p->py += p->vy * dt;
}

/* Bounce a dot off the screen edges. On a hit we pull it back inside,
 * flip and shrink the speed in that direction (so it loses a little
 * each time), and start the wall-flash timer. The top and bottom HUD
 * rows are kept out of bounds. */
static void reflect_at_chamber_walls(Particle *p, int cols, float py_max) {
  if (p->px < 1.0f) {
    p->px = 1.0f;
    p->vx = -p->vx * BOUNCE_REST;
    p->bounce_age = BOUNCE_FLASH_FRAMES;
  } else if (p->px > (float)(cols - 2)) {
    p->px = (float)(cols - 2);
    p->vx = -p->vx * BOUNCE_REST;
    p->bounce_age = BOUNCE_FLASH_FRAMES;
  }
  if (p->py < ASPECT_Y) {
    p->py = ASPECT_Y;
    p->vy = -p->vy * BOUNCE_REST;
    p->bounce_age = BOUNCE_FLASH_FRAMES;
  } else if (p->py > py_max) {
    p->py = py_max;
    p->vy = -p->vy * BOUNCE_REST;
    p->bounce_age = BOUNCE_FLASH_FRAMES;
  }
}

/* Remember where the dot is now, for its comet trail. */
static void push_trail_sample(Particle *p) {
  p->trail_head = (p->trail_head + 1) % TRAIL_LEN;
  p->trail_px[p->trail_head] = p->px;
  p->trail_py[p->trail_head] = p->py;
  if (p->trail_count < TRAIL_LEN)
    p->trail_count++;
}

/* One step of the whole simulation. Done in three passes so every
 * dot feels the same forces before any of them move: first add up the
 * forces and nudge speeds, then tame and move each dot and bounce it
 * off the walls, then record the trail. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  dt *= (float)s->speed / (float)SPEED_DEF;

  /* Pass 1: add up forces, nudge speeds. */
  for (int i = 0; i < s->n_particles; i++) {
    Particle *pi = &s->particles[i];
    if (!pi->active || pi->fixed)
      continue;
    float fx, fy;
    sum_coulomb_force_on(s, i, &fx, &fy);
    apply_force_to_velocity(pi, fx, fy, dt);
  }

  /* Pass 2: drag, cap, move, bounce, tick the flash timer. */
  float damp = expf(-VELOCITY_DAMP * dt);
  float py_max = (float)(s->rows - 2) * ASPECT_Y;
  for (int i = 0; i < s->n_particles; i++) {
    Particle *p = &s->particles[i];
    if (!p->active || p->fixed)
      continue;
    apply_velocity_drag(p, damp);
    clamp_velocity_magnitude(p, MAX_VELOCITY);
    integrate_position(p, dt);
    reflect_at_chamber_walls(p, s->cols, py_max);
    if (p->bounce_age > 0)
      p->bounce_age--;
  }

  /* Pass 3: record trails. */
  for (int i = 0; i < s->n_particles; i++) {
    Particle *p = &s->particles[i];
    if (!p->active)
      continue;
    push_trail_sample(p);
  }
}

/* Turn a dot's stretched physical position into a real screen cell.
 * The physics stretched y to keep orbits round; here we squash it back
 * so the dot lands on the right row. */
static void physical_to_cell(float px, float py, int *ix, int *iy) {
  *ix = (int)(px + 0.5f);
  *iy = (int)(py / ASPECT_Y + 0.5f);
}

/* Pick a trail glyph by how recent the sample is (1 = freshest,
 * 0 = oldest). Fresh points get dense glyphs, old ones get sparse, so
 * the trail visibly fades behind the dot [5]. */
static void trail_age_glyph(float newness, chtype *glyph, attr_t *attr) {
  if (newness > 0.85f) {
    *glyph = '*';
    *attr = A_BOLD;
  } else if (newness > 0.65f) {
    *glyph = '+';
    *attr = A_NORMAL;
  } else if (newness > 0.45f) {
    *glyph = ':';
    *attr = A_NORMAL;
  } else if (newness > 0.25f) {
    *glyph = '.';
    *attr = A_NORMAL;
  } else {
    *glyph = '`';
    *attr = A_NORMAL;
  }
}

/* Draw one dot's fading comet trail, in its own charge colour. Fixed
 * anchors don't move, so they get no trail. */
static void paint_particle_trail(const Scene *s, const Particle *p,
                                 int top_clip, int bot_clip) {
  if (p->fixed)
    return;
  int n = p->trail_count;
  if (n < 2)
    return;

  int slot = charge_to_slot(p->charge);
  int pair = PAIR_RAMP_BASE + slot;
  float denom = (float)(n - 2 > 0 ? n - 2 : 1);

  for (int k = 0; k < n - 1; k++) {
    /* oldest first; the newest slot is where the head itself draws */
    int idx = (p->trail_head + 1 + k) % TRAIL_LEN;
    int ix, iy;
    physical_to_cell(p->trail_px[idx], p->trail_py[idx], &ix, &iy);
    if (ix < 0 || ix >= s->cols)
      continue;
    if (iy < top_clip || iy >= bot_clip)
      continue;

    chtype glyph;
    attr_t attr;
    trail_age_glyph((float)k / denom, &glyph, &attr);
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(iy, ix, glyph);
    attroff(COLOR_PAIR(pair) | attr);
  }
}

/* A little four-cell glow around a moving dot's head. The side cells
 * use a denser glyph than the top/bottom ones, because cells are
 * taller than wide and that keeps the glow looking round. Right after
 * a wall hit the whole halo flashes bright '*' for a few frames. */
static void paint_particle_halo(const Scene *s, const Particle *p, int top_clip,
                                int bot_clip) {
  if (p->fixed)
    return;

  int ix, iy;
  physical_to_cell(p->px, p->py, &ix, &iy);
  int slot = charge_to_slot(p->charge);
  int pair = PAIR_RAMP_BASE + slot;

  bool flash = (p->bounce_age > 0);
  chtype halo_h = flash ? '*' : '.';
  chtype halo_v = flash ? '*' : '`';
  attr_t halo_attr = flash ? A_BOLD : A_NORMAL;

  struct {
    int dx, dy;
    chtype gl;
  } halo[] = {
      {-1, 0, halo_h},
      {+1, 0, halo_h},
      {0, -1, halo_v},
      {0, +1, halo_v},
  };

  attron(COLOR_PAIR(pair) | halo_attr);
  for (int k = 0; k < 4; k++) {
    int hx = ix + halo[k].dx;
    int hy = iy + halo[k].dy;
    if (hx < 0 || hx >= s->cols)
      continue;
    if (hy < top_clip || hy >= bot_clip)
      continue;
    mvaddch(hy, hx, halo[k].gl);
  }
  attroff(COLOR_PAIR(pair) | halo_attr);
}

/* Draw the dot itself. '@' for a heavy anchor, '*' right after a wall
 * hit, otherwise '+' for positive, '-' for negative, '.' for nearly
 * neutral. Heavy or fast dots are drawn bold so they stand out. */
static void paint_particle_head(const Scene *s, const Particle *p, int top_clip,
                                int bot_clip) {
  int ix, iy;
  physical_to_cell(p->px, p->py, &ix, &iy);
  if (ix < 0 || ix >= s->cols)
    return;
  if (iy < top_clip || iy >= bot_clip)
    return;

  chtype glyph;
  attr_t attr;
  if (p->fixed) {
    glyph = '@';
    attr = A_BOLD;
  } else if (p->bounce_age > 0) {
    glyph = '*';
    attr = A_BOLD;
  } else {
    if (p->charge > 0.5f)
      glyph = '+';
    else if (p->charge < -0.5f)
      glyph = '-';
    else
      glyph = '.';
    float vmag2 = p->vx * p->vx + p->vy * p->vy;
    bool heavy = fabsf(p->charge) > 1.5f;
    bool fast = vmag2 > VBOLD_THRESHOLD * VBOLD_THRESHOLD;
    attr = (heavy || fast) ? A_BOLD : A_NORMAL;
  }

  int slot = charge_to_slot(p->charge);
  int pair = PAIR_RAMP_BASE + slot;
  attron(COLOR_PAIR(pair) | attr);
  mvaddch(iy, ix, glyph);
  attroff(COLOR_PAIR(pair) | attr);
}

/* Paint the scene in three layers: all the trails, then all the halos,
 * then all the heads. Doing it in separate passes (not trail-halo-head
 * per dot) guarantees no dot's head gets buried under another dot's
 * trail or glow. */
static void scene_draw(const Scene *s) {
  int top_clip = HUD_TOP;
  int bot_clip = s->rows - HUD_BOT;

  for (int i = 0; i < s->n_particles; i++) {
    const Particle *p = &s->particles[i];
    if (p->active)
      paint_particle_trail(s, p, top_clip, bot_clip);
  }
  for (int i = 0; i < s->n_particles; i++) {
    const Particle *p = &s->particles[i];
    if (p->active)
      paint_particle_halo(s, p, top_clip, bot_clip);
  }
  for (int i = 0; i < s->n_particles; i++) {
    const Particle *p = &s->particles[i];
    if (p->active)
      paint_particle_head(s, p, top_clip, bot_clip);
  }
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

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(s);

  /* Top-right status line. */
  const char *state_str = s->paused ? "PAUSED " : "running";
  char top[200];
  snprintf(top, sizeof top,
           " %s  theme:%s  N=%d  speed:%d  %s  %.0f fps  %dHz ",
           pattern_name(s->current_pattern), themes[s->current_theme].name,
           s->n_particles, s->speed, state_str, fps, sim_fps);
  int top_len = (int)strlen(top);
  int top_col = sc->cols - top_len;
  if (top_col < 0)
    top_col = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvaddnstr(0, top_col, top, sc->cols);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* Bottom-left key hints; shorten them if the screen is narrow. */
  const char *hint_full = " q:quit  spc:pause  r:reseed  n/p:pattern  "
                          "t/T:theme  +/-:speed  ]/[:fps ";
  const char *hint_short = " q:quit  spc:pause  r:reseed  n/p:pat  t/T:theme ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= sc->cols - 1)
    hint = hint_short;
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvaddnstr(sc->rows - 1, 0, hint, sc->cols);
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
    scene_init_pattern(s);
    break;
  case 'p':
  case 'P':
    s->current_pattern =
        (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
    scene_init_pattern(s);
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
