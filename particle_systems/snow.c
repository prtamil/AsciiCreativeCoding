/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * snow.c — drifting snowflakes with sin-sway + sandpile accumulation
 *
 * DEMO: Snowflakes drift down the screen with subtle horizontal SWAY
 *       (each flake has its own amplitude and frequency, so no two
 *       follow the same path). Wind adds a steady horizontal drift
 *       per pattern. When a flake reaches the top of the snow pile
 *       at its column, it DEPOSITS — `pile[col]++` — and dies. The
 *       deposit checks its two neighbours: if either is lower, the
 *       flake "rolls" there instead, so valleys fill before peaks
 *       and the pile naturally smooths into drifts.
 *
 *       Patterns:
 *         FLURRY    sparse, slow, big sway — light first snow
 *         SNOWFALL  medium density and speed, classic snow scene
 *         BLIZZARD  dense, fast, strong wind, sway suppressed
 *
 * Study alongside:
 *   rain.c                 — same particle-pool + pattern framework.
 *   grids/cell_grids/sandpile.c — the sandpile-stacking idea this
 *                            file's pile-deposit mechanism mirrors.
 *
 * Section map:
 *   §1 config    — constants, themes, per-pattern parameters
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-pair theme ramp + pile + sky pairs
 *   §4 flake     — Flake struct, sway formula, spawn samplers + Euler step
 *   §5 pile      — column-height array, valley-fill deposit, contact math
 *   §6 scene     — pool, tick driver + helpers, draw driver + helpers
 *   §7 screen    — ncurses init / draw / resize
 *   §8 app       — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reseed (clear pile + flakes, re-prewarm)
 *   c          clear pile only (keep flakes)
 *   n / N      next pattern    (FLURRY → SNOWFALL → BLIZZARD)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster (speed multiplier ×2)
 *   -          slower (÷2)
 *   ] / [      raise / lower tick Hz
 *   w / W      wind right / left (override pattern wind by ±3 c/s)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/snow.c \
 *       -o snow -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Pool-based 2-D particle system (one species: flakes)
 *                  + a 1-D INT array for the snow pile, one entry per
 *                  screen column storing the current pile height in
 *                  cells. Each tick:
 *
 *                    1. Top up the flake pool to `pattern.target_flakes`
 *                       (capped per-tick proportional to dt to avoid
 *                       spawn spikes).
 *                    2. Integrate each flake: vertical fall by `vy·dt`,
 *                       wind drift `drift_vx·dt`, sway position via
 *                       `sin(sway_freq·age + phase)·sway_amp`.
 *                    3. Detect pile contact: if flake.y crosses the top
 *                       of the pile at flake's column, DEPOSIT.
 *                    4. Deposit checks the two neighbouring columns: if
 *                       either is lower than the contact column, deposit
 *                       there instead. This lets falling snow fill
 *                       valleys before peaks — mimicking the angle-of-
 *                       repose behaviour of real snow.
 *                    5. (No melt.)  The pile caps at PILE_MAX_FRAC =
 *                       15 % of screen height and stays there.  Once
 *                       a column reaches the cap, additional flakes
 *                       die silently in pile_deposit (no-op).  User
 *                       presses 'c' to wipe the pile and rebuild.
 *
 *                  Flake glyph picked at spawn from a small variety
 *                  set so the falling field reads as different snow
 *                  flake sizes (large `*`, medium `+`, tiny `.` `,`).
 *
 * Data-structure : Flake[MAX_FLAKES] with active flag — same pattern
 *                  as rain.c. pile[MAX_COLS] of int — height per col.
 *                  No malloc at runtime.
 *
 * Rendering      : ASCII only. Flakes use an 8-step glyph ramp
 *                  ` ` . ' , : + * * ` indexed by size class (small dim
 *                  → big bright).  Pile uses three depth tiers:
 *                  `*` (surface), `#` (packed), `+` (settled).  All
 *                  cells coloured from the active theme's 8-step ramp.
 *
 * Performance    : O(MAX_FLAKES) per tick.  At BLIZZARD 700 flakes ×
 *                  constant work, ~28k mvaddch per frame at 60 fps —
 *                  well inside ncurses budget.
 *
 * References
 * ──────────
 *   PAPERS
 *     Reeves, W. T. (1983)
 *       "Particle Systems — A Technique for Modeling a Class of Fuzzy Objects"
 *       ACM Transactions on Graphics 2(2): 91-108.
 *       Foundational paper.  The pool-based flake model used here
 *       (per-particle x, y, vy, sway phase, lifetime via off-screen
 *       cull) is exactly Reeves' particle-system design.  §4 discusses
 *       natural-phenomena pools — snow/rain are explicit examples.
 *
 *     Bak, P., Tang, C. & Wiesenfeld, K. (1987)
 *       "Self-Organized Criticality: An Explanation of 1/f Noise"
 *       Physical Review Letters 59(4): 381-384.
 *       The original sandpile cellular automaton.  Our valley-fill
 *       deposit (a flake landing on a tall column rolls into a lower
 *       neighbour) is a one-step approximation of Bak/Tang/Wiesenfeld's
 *       grain-redistribution rule — it gives the pile an emergent
 *       angle of repose ≈ 45° in cell space without explicit physics.
 *
 *   BOOKS
 *     Bourg, D. M. & Bywalec, B. — "Physics for Game Developers"
 *       (2nd ed, O'Reilly, 2013).
 *       Ch. 2-3 covers ballistic motion under gravity + drag (real
 *       flakes reach terminal velocity in ~1-3 m/s; we model that as
 *       constant `vy`).  Ch. 8 §1 covers harmonic oscillators — the
 *       sin-sway pattern x(t) = base + amp·sin(ω·t + φ) is Bourg's
 *       canonical undamped oscillator.
 *
 *     Duran, J. — "Sands, Powders, and Grains: An Introduction
 *       to the Physics of Granular Materials" (Springer, 2000).
 *       Ch. 1-2 — angle of repose and column-based pile dynamics.
 *       Grounds the valley-fill rule used here: a grain landing on
 *       a slope rolls to the lower of its two neighbours until the
 *       slope is locally flat.
 *
 *     Akenine-Möller, T., Haines, E. & Hoffman, N. — "Real-Time
 *       Rendering" (4th ed, CRC Press, 2018).
 *       §13.7 — point-sprite particle rendering.  The size-to-ramp-
 *       slot glyph mapping used for the FLAKE_GLYPHS ramp here is
 *       the ASCII analogue of size-by-mass point-sprite sizing.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE — DATA-DRIVEN PATTERN ENGINE ────────────────────────── *
 *
 * Three snowfall intensities (FLURRY, SNOWFALL, BLIZZARD) are
 * produced by a SINGLE generic flake-physics + sandpile engine
 * driven by an array of PatternParams structs.  scene_tick and
 * scene_draw never branch on the pattern enum to decide HOW the
 * physics works — they read pattern_params[s->current_pattern] and
 * compute accordingly.  Adding a new intensity is a matter of
 * appending one row to pattern_params[]; no new code paths.
 *
 *
 * THE GENERIC ENGINE (pseudocode)
 * ───────────────────────────────
 *
 *   loop forever (each tick of dt seconds):
 *
 *     pp = pattern_params[scene.current_pattern]   # read inputs
 *
 *     # 1. SPAWN — top up flake pool toward target density
 *     while count(active flakes) < pp.target_flakes:
 *         f = next_inactive_slot()
 *         f.center_x   = wind_aware_uniform_x(scene.cols, pp.wind_x)
 *         f.y          = uniform(-6, -1)               # just above top
 *         f.vy         = pp.fall_speed   · jitter      # terminal velocity
 *         f.drift_vx   = pp.wind_x       + jitter      # horizontal wind
 *         f.sway_amp   = uniform(pp.sway_amp_min,  pp.sway_amp_max)
 *         f.sway_freq  = uniform(pp.sway_freq_min, pp.sway_freq_max)
 *         f.sway_phase = uniform(0, 2π)
 *         f.size_idx   = small_biased(0..7)            # r²-weighted
 *         f.active     = true
 *
 *     # 2. INTEGRATE — explicit Euler advances each flake
 *     for f in active flakes:
 *         f.center_x += f.drift_vx · dt
 *         f.y        += f.vy       · dt
 *         f.age      += dt
 *         # rendered x = center_x + sway_amp · sin(sway_freq·age + phase)
 *
 *     # 3. PILE CONTACT — flake meets the snow pile at its column
 *     for f in active flakes:
 *         col = round(rendered_x(f))
 *         if f.y >= rows - 2 - pile[col]:
 *             if bernoulli(pp.pile_growth_mul):       # probabilistic
 *                 pile_deposit(pile, col, max_h)      # valley-fill rule
 *             f.active = false
 *
 *     # 4. CULL — flakes drifted off-screen sideways die quietly
 *     for f in active flakes:
 *         if f.center_x off-screen:  f.active = false
 *
 *     # 5. RENDER — pile columns + active flakes overlaid
 *     pile_draw_columns(pile, rows-2)        # *  surface
 *                                            # #  packed
 *                                            # +  settled
 *     for f in active flakes:
 *         glyph = FLAKE_GLYPHS[f.size_idx]
 *         pair  = PAIR_FLAKE_BASE + f.size_idx
 *         paint(rendered_x(f), f.y, glyph, pair)
 *
 *
 * PATTERNPARAMS FIELD → ENGINE HOOK
 * ─────────────────────────────────
 *
 *   target_flakes    →  spawn-loop refill cap        (sky density)
 *   fall_speed       →  vy at spawn  (× jitter)      (terminal velocity)
 *   wind_x           →  drift_vx + spawn-x extension (Bourg Ch.3)
 *   sway_amp_min     →  per-flake sway amplitude     (oscillator A)
 *   sway_amp_max
 *   sway_freq_min    →  per-flake sway frequency     (oscillator ω)
 *   sway_freq_max
 *   pile_growth_mul  →  Bernoulli p at pile contact  (visual pacing knob)
 *
 * Every other engine constant — MAX_FLAKES, MAX_COLS, PILE_MAX_FRAC,
 * FLAKE_SPEED_VARIANCE, FLAKE_WIND_JITTER, WIND_STEP — is a GLOBAL
 * tuning knob shared across all patterns.  Patterns differ ONLY in
 * the eight fields above.
 *
 *
 * ARCHITECTURAL REFERENCES
 * ────────────────────────
 *
 *   Reeves, W. T. (1983)
 *     "Particle Systems — A Technique for Modeling a Class of
 *     Fuzzy Objects", ACM TOG 2(2): 91-108.
 *     §4 makes the explicit argument that ONE engine + a struct
 *     of physical constants per phenomenon is the right
 *     architecture for natural-particle simulations — snow is one
 *     of Reeves' enumerated examples.  This file applies the idea
 *     literally: three intensity presets, one engine.
 *
 *   Gamma, E., Helm, R., Johnson, R. & Vlissides, J. (1994)
 *     "Design Patterns" (Addison-Wesley) — STRATEGY pattern (§5.9).
 *     A family of algorithms (FLURRY / SNOWFALL / BLIZZARD)
 *     interchangeable behind a single interface (the engine
 *     reading PatternParams).  In procedural C the "interface" is
 *     the struct shape; "concrete strategies" are the rows of
 *     pattern_params[]; "selecting a strategy" is updating
 *     scene.current_pattern.
 *
 *   Acton, M. (2014)
 *     "Data-Oriented Design and C++" (CppCon 2014 keynote).
 *     Argues that variation between behaviours should be
 *     represented as DATA (struct fields) rather than as control
 *     flow (if/switch on type).  pattern_params[] is a compact
 *     data table; the engine has no per-pattern code paths.
 *
 *   Nystrom, R. (2014)
 *     "Game Programming Patterns" (Genever Benning).
 *     TYPE OBJECT chapter — PatternParams is a Type Object: one
 *     shared instance per "kind" of snowfall, every Flake
 *     implicitly references it via Scene.current_pattern.  DATA
 *     LOCALITY chapter — Flake is a flat-laid-out POD swept
 *     linearly by the integrator each tick, exactly the layout
 *     Nystrom recommends for hot inner loops.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each flake is an INDEPENDENT particle with its own fall speed,
 * sway amplitude, and sway frequency. The horizontal position is a
 * BASE x (which drifts with wind) plus a sin oscillation. When a
 * flake reaches the pile's top at its column, it lands — but if a
 * neighbour column is lower, it rolls there instead. The pile
 * grows column by column as drifts.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. SPAWN. Each tick, count active flakes; if below target, scan
 *     the pool for an inactive slot and spawn at random x with a
 *     random spawn-y in [-6, -1] (just above the visible top). On
 *     init / reseed / pattern-change, prewarm with random y across
 *     the WHOLE visible range (no "starting line" effect).
 *
 *  2. INTEGRATE per flake:
 *       flake.center_x += flake.drift_vx · dt
 *       flake.y        += flake.vy · dt
 *       flake.age      += dt
 *       flake.x        =  flake.center_x
 *                       + flake.sway_amp · sin(flake.sway_freq · flake.age
 *                                              + flake.sway_phase)
 *
 *  3. PILE CONTACT. Compute floor_y(col) = (rows − 2) − pile[col].
 *     If flake.y >= floor_y(col): DEPOSIT then deactivate.
 *
 *  4. DEPOSIT (with valley-fill):
 *       cl = col − 1, cr = col + 1
 *       if cl in range and pile[cl] < pile[col]: target = cl
 *       elif cr in range and pile[cr] < pile[col]: target = cr
 *       else target = col
 *       pile[target]++   (capped at PILE_MAX_HEIGHT)
 *
 *  5. (No melt — pile is monotonically non-decreasing.)
 *     pile is capped at PILE_MAX_FRAC × (rows − 2) = 15 % of usable
 *     height.  Once any column hits the cap, future deposits there
 *     are no-ops in pile_deposit and the flake dies anyway.
 *
 *  6. RENDER:
 *       For each active flake: render its glyph at (x, y) in a
 *       theme ramp colour based on the flake's "size" (big = bright).
 *       For each column c with pile[c] > 0: draw the pile from
 *       (rows-2) up to (rows-2 - pile[c]+1).  Surface cell = `*`,
 *       next two = `#`, deeper = `+`.
 *
 *  7. HUD on bottom row.
 *
 * KEY FORMULAS
 * ────────────
 *  Sway oscillation:
 *    x(t) = base_x + drift_vx·(t − t_spawn)
 *               + sway_amp · sin(sway_freq·(t − t_spawn) + sway_phase)
 *
 *  Pile contact (per flake's column c):
 *    floor_y = (rows − 2) − pile[c]
 *    contact ⟺ flake.y ≥ floor_y
 *
 *  Valley-fill deposit:
 *    target = col,
 *      if pile[col-1] < pile[col]: target = col-1
 *      elif pile[col+1] < pile[col]: target = col+1
 *    pile[target] = min(pile[target] + 1, PILE_MAX)
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

  MAX_FLAKES = 900,
  MAX_COLS = 800, /* fixed-size pile array              */

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_FLAKE_BASE = 3, /* +0..+7 = 8 flake tints (small→big) */
  PAIR_PILE_BASE = 11, /* +0..+7 = 8 pile tints (deep→top)   */
  PAIR_SKY = 19,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* Per-flake physics jitter — same idea as rain.c. */
#define FLAKE_SPEED_VARIANCE 0.50f /* ±25% per-flake fall speed */
#define FLAKE_WIND_JITTER 1.0f     /* ±0.5 c/s per flake        */

/* Pile mechanics. */
#define PILE_MAX_FRAC                                                          \
  0.15f /* pile capped at 15% screen; sticks at cap (no melt) */

/* Wind override step. */
#define WIND_STEP 3.0f

/* Pattern enum. */
typedef enum {
  PATTERN_FLURRY = 0,
  PATTERN_SNOWFALL = 1,
  PATTERN_BLIZZARD = 2,
  N_PATTERNS = 3,
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_FLURRY:
    return "FLURRY  ";
  case PATTERN_SNOWFALL:
    return "SNOWFALL";
  case PATTERN_BLIZZARD:
    return "BLIZZARD";
  default:
    return "?       ";
  }
}

/*
 * PatternParams — physics + density knobs that distinguish one snow
 * INTENSITY pattern from another. The simulation engine never branches
 * on the pattern enum; it reads these fields and behaves accordingly.
 * Same code, three different snowfalls.  (Reeves 1983 §4 — natural-
 * phenomena particle pools parameterised by a small struct of physical
 * constants.)
 *
 *   target_flakes  : steady-state count of active flakes.  The spawn
 *                    loop refills toward this each tick (with a per-
 *                    tick cap so a long pause doesn't dump a flood).
 *                    Higher = denser sky.
 *                    FLURRY 120 (sparse), BLIZZARD 700 (whiteout).
 *
 *   fall_speed     : nominal vy in cells/sec.  Real snowflakes reach
 *                    TERMINAL VELOCITY almost immediately (~1 m/s wet,
 *                    ~3 m/s dry — Bourg & Bywalec Ch. 3); we model that
 *                    as a constant vy with ± per-flake jitter applied
 *                    at spawn.  Slow flurries get 10 c/s, blizzards 45 c/s.
 *
 *   wind_x         : default horizontal velocity in cells/sec applied
 *                    at spawn as `drift_vx`.  Sign convention: positive
 *                    = rightward, negative = leftward.  User adds
 *                    Scene.wind_override on top via w / W keys.
 *
 *   sway_amp_min,  : sway AMPLITUDE range in cells — per-flake amplitude
 *   sway_amp_max     sampled uniformly between min and max at spawn.
 *                    The amp · sin(ω·t + φ) sway is a basic UNDAMPED
 *                    HARMONIC OSCILLATOR (Bourg & Bywalec Ch. 8 §1).
 *                    FLURRY 1.5..4.0 cells (visible swing); BLIZZARD
 *                    0.3..1.0 (wind dominates, sway barely visible).
 *
 *   sway_freq_min, : sway angular FREQUENCY range in rad/sec — per-flake
 *   sway_freq_max    frequency sampled at spawn.  FLURRY 0.4..1.3 rad/sec
 *                    (slow gentle swing); BLIZZARD 0.2..0.8 (slow but
 *                    invisible under high wind).
 *
 *   pile_growth_mul: probability ∈ [0, 1] that an impact actually deposits
 *                    onto the pile.  BLIZZARD = 1.0 (every hit piles);
 *                    FLURRY = 0.60 (60 % of hits pile, rest disappear) so
 *                    a light flurry doesn't build dunes as fast as a
 *                    blizzard even at the same flake count.  This is
 *                    NOT physical — it's a knob for visual pacing.
 */
typedef struct {
  int target_flakes;
  float fall_speed;
  float wind_x;
  float sway_amp_min, sway_amp_max;
  float sway_freq_min, sway_freq_max;
  float pile_growth_mul;
} PatternParams;

static const PatternParams pattern_params[N_PATTERNS] = {
    /* FLURRY    */ {120, 10.0f, 3.0f, 1.5f, 4.0f, 0.40f, 1.30f, 0.60f},
    /* SNOWFALL  */ {320, 18.0f, 6.0f, 0.8f, 2.5f, 0.30f, 1.10f, 0.85f},
    /* BLIZZARD  */ {700, 45.0f, 22.0f, 0.3f, 1.0f, 0.20f, 0.80f, 1.00f},
};

/*
 * Themes — flake[8] is a SMALL→BIG ramp (smaller flakes fade dimmer,
 * bigger ones brighter). pile[8] is a DEEP→TOP ramp (older snow deeper
 * darker, fresh top brighter).
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule.
 */
typedef struct {
  const char *name;
  short flake[8]; /* small → big   */
  short pile[8];  /* deep  → top   */
  short sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name        flake[0..7]  (small dim → big bright)            pile[0..7]
       (deep dark → top bright)            sky */

    {"MATRIX",
     {28, 34, 40, 46, 82, 118, 154, 190},
     {28, 34, 40, 46, 82, 118, 154, 190},
     234},
    {"FIRE",
     {88, 124, 130, 166, 202, 208, 214, 226},
     {52, 88, 124, 160, 196, 202, 208, 220},
     233},
    {"OCEANIC",
     {24, 31, 38, 44, 51, 87, 159, 195},
     {24, 25, 31, 38, 44, 51, 87, 159},
     234},
    {"NEON",
     {53, 91, 134, 165, 201, 207, 213, 219},
     {53, 91, 134, 165, 201, 207, 213, 225},
     234},
    {"MONO",
     {244, 246, 248, 250, 252, 253, 254, 255},
     {240, 244, 247, 249, 251, 253, 254, 255},
     232},
    {"ICE",
     {117, 153, 159, 195, 225, 231, 254, 255},
     {110, 117, 153, 159, 195, 231, 254, 255},
     235},
    {"NOVA",
     {24, 75, 117, 159, 195, 219, 226, 231},
     {60, 75, 117, 159, 195, 219, 226, 231},
     234},
    {"FOREST",
     {28, 64, 70, 76, 112, 148, 184, 220},
     {28, 64, 70, 76, 112, 148, 184, 220},
     234},
    {"DESERT",
     {94, 130, 137, 173, 179, 215, 222, 229},
     {94, 130, 137, 143, 179, 215, 222, 229},
     234},
    {"ECLIPSE",
     {52, 88, 95, 131, 167, 173, 209, 215},
     {52, 88, 95, 131, 167, 173, 209, 215},
     232},
};

/* Variety of flake glyphs, indexed by SIZE class (small → big). */
static const char FLAKE_GLYPHS[8] = {'`', '.', '\'', ',', ':', '+', '*', '*'};

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
    for (int i = 0; i < 8; i++) {
      init_pair((short)(PAIR_FLAKE_BASE + i), t->flake[i], -1);
      init_pair((short)(PAIR_PILE_BASE + i), t->pile[i], -1);
    }
    init_pair(PAIR_SKY, t->sky, -1);
  } else {
    for (int i = 0; i < 8; i++) {
      init_pair((short)(PAIR_FLAKE_BASE + i), COLOR_WHITE, -1);
      init_pair((short)(PAIR_PILE_BASE + i), COLOR_WHITE, -1);
    }
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
/* §4  flake                                                              */
/* ===================================================================== */

/*
 * Flake — one falling snowflake.
 *
 * REFERENCES:
 *   Reeves, W. T. (1983) "Particle Systems — A Technique for Modeling
 *     a Class of Fuzzy Objects", ACM TOG 2(2): 91-108.
 *     The fixed pool + per-particle state + lifetime-via-off-screen-cull
 *     model is straight from Reeves §3.  §4 specifically mentions snow
 *     as a natural-phenomena instance of his framework.
 *
 *   Bourg, D. M. & Bywalec, B. (2013) "Physics for Game Developers"
 *     (2nd ed, O'Reilly).  Ch. 3 — terminal velocity (real flakes reach
 *     it almost immediately, modelled as constant vy); Ch. 8 §1 — the
 *     undamped harmonic oscillator pattern x(t) = base + amp·sin(ω·t + φ)
 *     used for the side-to-side sway.
 *
 * INTENT: hold every per-flake value the integrator needs in a flat
 * layout that fits into a fixed-size BSS pool (no malloc after init).
 * One inactive slot can host the next spawn — `active` is the single
 * source of truth.
 *
 * LIFECYCLE: spawn just above the visible top with random x, jittered
 * fall speed, jittered sway parameters → integrate forward each tick
 * (x drifts with wind, y falls at vy, age increments) → when y >=
 * pile-top OR drifted off-screen sideways, mark inactive (pile may
 * receive a deposit via pile_deposit if y triggered death).  Slots
 * are then reused linearly by the next spawn.
 *
 * SWAY MODEL: the visible x position is NOT center_x — it's
 *     x(t) = center_x + sway_amp · sin(sway_freq · age + sway_phase)
 * Splitting center_x (drift) from the sway oscillation lets the wind
 * push the AVERAGE position while the flake still wobbles around it.
 * Per-flake random sway_phase de-correlates the field so flakes don't
 * all sway in sync (which would look mechanical).
 *
 *   center_x   : drift centre in cells.  Advances at drift_vx each tick.
 *                The flake's RENDERED x is computed via flake_x() which
 *                adds the sway oscillation on top.  Storing the centre
 *                separately means wind doesn't fight with the sway.
 *
 *   y          : vertical position in cells.  Increases monotonically
 *                (positive y = downward in ncurses) at rate `vy` until
 *                pile contact or off-screen exit.
 *
 *   vy         : fall speed in cells/sec.  Per-flake jitter ±25 %
 *                (FLAKE_SPEED_VARIANCE) applied at spawn so the field
 *                doesn't descend in lockstep (Reeves' "stochastic
 *                spawn" idea).  Models terminal velocity (Bourg Ch. 3).
 *
 *   drift_vx   : horizontal wind component in cells/sec, sampled at
 *                spawn as (pattern.wind_x + scene.wind_override +
 *                random jitter).  Constant for the life of the flake.
 *
 *   sway_amp   : sway amplitude in cells, sampled per-flake at spawn
 *                from [pattern.sway_amp_min, pattern.sway_amp_max].
 *                Bigger amp = wider swing on each oscillation cycle.
 *
 *   sway_freq  : sway ANGULAR frequency in rad/sec, sampled per-flake
 *                from [pattern.sway_freq_min, pattern.sway_freq_max].
 *                Period of one full sway cycle = 2π / sway_freq seconds.
 *
 *   sway_phase : per-flake phase offset ∈ [0, 2π).  De-correlates the
 *                field so flakes don't all start swaying from the same
 *                position (which would form an obvious vertical band).
 *
 *   age        : seconds since spawn.  Feeds the sway sin() argument.
 *                Not used to KILL the flake (death comes from y or x
 *                bounds, not lifetime) — purely a phase clock.
 *
 *   size_idx   : 0..7 size class chosen at spawn.  Indexes both the
 *                FLAKE_GLYPHS ramp (` . ' , : + * *`) and the active
 *                theme's flake[] colour ramp.  Bigger size = denser
 *                glyph + brighter colour, mimicking real snow's
 *                size-to-mass-to-brightness perception (Akenine-Möller
 *                §13.7 point-sprite analogue).
 *
 *   active     : pool-slot occupancy flag.  Inactive slots are skipped
 *                by every loop; spawn finds the first inactive index
 *                via flake_pool_find_inactive (linear scan).
 */
typedef struct {
  float center_x;
  float y;
  float vy;
  float drift_vx;
  float sway_amp;
  float sway_freq;
  float sway_phase;
  float age;
  int size_idx;
  bool active;
} Flake;

/* Cheap LCG — per-scene state, no global aliasing */
static inline uint32_t lcg_next(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}
static inline float lcg_unit(uint32_t *st) {
  return (float)(lcg_next(st) >> 8) / (float)(1u << 24); /* [0, 1) */
}

/* Compute a flake's actual screen x at this moment. */
static inline float flake_x(const Flake *f) {
  return f->center_x +
         f->sway_amp * sinf(f->sway_freq * f->age + f->sway_phase);
}

/* Uniform sample over [lo, hi) — base primitive every other sampler uses. */
static inline float sample_uniform_in_range(uint32_t *rng, float lo, float hi) {
  return lo + lcg_unit(rng) * (hi - lo);
}

/* Terminal velocity with stochastic per-flake variance (Bourg Ch. 3).
 * Returns base_vy · uniform(1 − V/2, 1 + V/2) so the field doesn't fall in
 * lockstep. */
static inline float sample_terminal_velocity_jittered(uint32_t *rng,
                                                      float base_vy) {
  float scale = (1.0f - FLAKE_SPEED_VARIANCE * 0.5f) +
                lcg_unit(rng) * FLAKE_SPEED_VARIANCE;
  return base_vy * scale;
}

/* Drift velocity with additive jitter: wind + uniform(−J/2, +J/2). */
static inline float sample_drift_velocity_jittered(uint32_t *rng,
                                                   float wind_base) {
  float jitter = (lcg_unit(rng) - 0.5f) * 2.0f * FLAKE_WIND_JITTER;
  return wind_base + jitter;
}

/* Random oscillator phase φ ∈ [0, 2π) — de-correlates sin() across the field.
 */
static inline float sample_random_phase_2pi(uint32_t *rng) {
  return lcg_unit(rng) * 2.0f * (float)M_PI;
}

/* Size class 0..7 with small-bias via r² mapping — most flakes small, few big.
 */
static inline int sample_size_class_small_biased(uint32_t *rng) {
  float r = lcg_unit(rng);
  int idx = (int)(r * r * 7.999f);
  if (idx < 0)
    idx = 0;
  if (idx > 7)
    idx = 7;
  return idx;
}

/* Spawn x with the range extended UPWIND so wind-blown flakes refill the
 * visible field uniformly rather than piling up against the leeward edge. */
static inline float sample_spawn_x_wind_extended(uint32_t *rng, int cols,
                                                 float wind) {
  float over = fabsf(wind) * 0.5f;
  float r = lcg_unit(rng);
  if (wind > 0.5f)
    return r * ((float)cols + over) - over;
  if (wind < -0.5f)
    return r * ((float)cols + over);
  return r * (float)cols;
}

/* Explicit Euler step: drift x by drift_vx·dt, fall y by vy·dt, age the clock.
 */
static inline void integrate_flake_euler(Flake *f, float dt) {
  f->center_x += f->drift_vx * dt;
  f->y += f->vy * dt;
  f->age += dt;
}

/* True once the flake's drift centre has crossed the off-screen sideways cull.
 */
static inline bool flake_drifted_offscreen_x(const Flake *f, int cols) {
  return f->center_x < -8.0f || f->center_x > (float)(cols + 8);
}

/* ===================================================================== */
/* §5  pile — accumulation array + valley-fill deposit                   */
/* ===================================================================== */

/*
 * pile_deposit() — add one cell of snow at column `col` with neighbour
 * preference: if col-1 or col+1 is lower, deposit there instead.
 *
 * This is the angle-of-repose trick — falling snow naturally fills
 * valleys before raising peaks. Without it the pile grows as straight
 * vertical columns (one per flake hit), which looks artificial.
 *
 * Deposit fails silently (no-op) if all three columns are at PILE_MAX.
 */
static void pile_deposit(int *pile, int cols, int col, int max_h) {
  if (col < 0 || col >= cols)
    return;

  int target = col;
  if (col > 0 && pile[col - 1] < pile[target])
    target = col - 1;
  if (col < cols - 1 && pile[col + 1] < pile[target])
    target = col + 1;

  if (pile[target] < max_h)
    pile[target] += 1;
}

/* Saturation cap for any single column — PILE_MAX_FRAC of usable rows. */
static inline int compute_pile_height_cap(int rows) {
  int max_h = (int)((float)(rows - 2) * PILE_MAX_FRAC);
  return max_h < 1 ? 1 : max_h;
}

/* World-y of the pile's top surface at a column — flake contacts when y ≥ this.
 */
static inline float pile_floor_y(int pile_h, int rows) {
  return (float)(rows - 2 - pile_h);
}

/* Bernoulli trial: true with probability p ∈ [0,1]. */
static inline bool bernoulli_trial(uint32_t *rng, float p) {
  return lcg_unit(rng) < p;
}

/* ===================================================================== */
/* §6  scene — pool, tick, draw                                          */
/* ===================================================================== */

/*
 * Scene — owns every piece of mutable state for the snow simulation.
 * Two clearly-separated halves:
 *
 *   SIMULATION half — what scene_tick reads + writes.  Owns the active
 *                     pattern, force overrides, RNG, cached terminal
 *                     dimensions, the clock for the sway phase, the
 *                     flake pool, and the 1-D pile column-height array.
 *                     Mutated by scene_tick and the key handler.
 *
 *   RENDER half     — what scene_draw consults to pick colours.  Purely
 *                     visual selection index; never read inside the
 *                     physics tick.
 *
 * REFERENCES (each cited inline at the relevant member):
 *   Reeves (1983)     — particle pool design
 *   Bak/Tang/Wiesenfeld (1987) — sandpile valley-fill (used by pile_deposit)
 *   Duran (2000)      — granular angle of repose
 *   Bourg/Bywalec (2013) — terminal velocity + harmonic oscillator
 *
 * The Scene knows nothing about ncurses — physics writes to the pool +
 * pile, the render layer reads them.  That separation lets the
 * simulation be exercised without a terminal.
 */
typedef struct {
  /* ──────────────────────────────────────────────────────────────
   *  SIMULATION HALF — physics tick reads + writes these
   * ────────────────────────────────────────────────────────────── */

  /* PAUSE — scene_tick is a no-op when set.  Toggled by space.
   * Render keeps running so the user sees a frozen frame with
   * flakes held mid-sway. */
  bool paused;

  /* SPEED — integer multiplier on dt.  Default SPEED_DEF means
   * 1× wall clock; +/= keys double, − halves.  Bounded by
   * SPEED_MIN/MAX.  Doesn't change physics constants — just
   * compresses or stretches simulated time. */
  int speed;

  /* PATTERN — index into pattern_params (FLURRY / SNOWFALL /
   * BLIZZARD).  Cycled by n / N.  Switching pattern doesn't
   * rebuild pools — the new target_flakes self-fills / drains
   * over a few seconds, which reads as a natural intensity change. */
  Pattern current_pattern;

  /* WIND OVERRIDE — added to pattern.wind_x to compute the actual
   * drift_vx applied to newly-spawned flakes.  w / W keys add
   * ±WIND_STEP; persists across pattern switches; reset on 'r'. */
  float wind_override;

  /* RNG — per-scene LCG state, seeded from clock_ns() at init and
   * re-seeded (XOR'd) on 'r'.  Used by every randomness consumer:
   * spawn jitter (x position, fall speed, sway params, size class),
   * pile_growth_mul probability check at deposit.  No globals —
   * full state in this byte. */
  uint32_t rng;

  /* CACHED TERMINAL DIMENSIONS — read every frame by spawn (x range
   * for the top edge), the integrator (off-screen sideways check),
   * and the pile renderer.  Cached at init and on SIGWINCH so the
   * hot path never calls getmaxyx(). */
  int rows, cols;

  /* CLOCK — seconds since the scene started, advanced by dt each
   * tick.  Not currently read by the integrator (each flake's `age`
   * is what feeds the sway sin()), but kept available for future
   * effects that need a global phase clock. */
  float time_accum;

  /* FLAKE POOL — fixed-size BSS array, no allocation after init.
   * See Flake for per-slot detail.  flake_pool_find_inactive scans
   * for the first inactive index when spawn needs a slot.
   * (Reeves 1983 — particle pool.) */
  Flake flakes[MAX_FLAKES];

  /* PILE COLUMN HEIGHTS — 1-D array, pile[c] = current snow height
   * at column c (in cells, measured up from the bottom HUD row).
   * MAX_COLS is the hard cap; we draw up to scene.cols only.
   * Modified by pile_deposit (which applies a valley-fill rule:
   * Bak/Tang/Wiesenfeld 1987 sandpile + Duran 2000 angle of repose).
   * Each column saturates at PILE_MAX_FRAC × (rows − 2) = 15 % of
   * usable height; no melt — pile is monotonically non-decreasing
   * until 'c' clears it. */
  int pile[MAX_COLS];

  /* ──────────────────────────────────────────────────────────────
   *  RENDER HALF — scene_draw reads this; physics tick ignores it
   * ────────────────────────────────────────────────────────────── */

  /* THEME — index into themes[].  Cycled by t / T.  Selects the
   * 8-step flake ramp (size → colour) AND the 8-step pile ramp
   * (depth → colour).  Pure render concern — flake physics
   * (count, motion, lifetime, pile dynamics) behaves identically
   * regardless of which theme is active.  theme_apply rewrites
   * pairs PAIR_FLAKE_BASE..+7 and PAIR_PILE_BASE..+7 on change. */
  int current_theme;
} Scene;

static int flake_pool_find_inactive(Scene *s) {
  for (int i = 0; i < MAX_FLAKES; i++)
    if (!s->flakes[i].active)
      return i;
  return -1;
}

/*
 * scene_spawn_flake — activate one flake.
 *   y_min, y_max : initial-y range.  Normal spawn passes (-6, -1) so
 *                  the flake enters from just above the visible top;
 *                  prewarm passes (-6, rows-2) for steady-state fill.
 *
 * Each field is filled by a dedicated sampler so the body reads as a
 * declaration of the flake's INITIAL PHYSICS STATE: position, velocity,
 * oscillator, size class.  Per-flake jitter lives inside the samplers.
 */
static void scene_spawn_flake(Scene *s, float y_min, float y_max) {
  int idx = flake_pool_find_inactive(s);
  if (idx < 0)
    return;
  Flake *f = &s->flakes[idx];

  const PatternParams *pp = &pattern_params[s->current_pattern];
  float wind_total = pp->wind_x + s->wind_override;

  f->center_x = sample_spawn_x_wind_extended(&s->rng, s->cols, wind_total);
  f->y = sample_uniform_in_range(&s->rng, y_min, y_max);
  f->vy = sample_terminal_velocity_jittered(&s->rng, pp->fall_speed);
  f->drift_vx = sample_drift_velocity_jittered(&s->rng, wind_total);
  f->sway_amp =
      sample_uniform_in_range(&s->rng, pp->sway_amp_min, pp->sway_amp_max);
  f->sway_freq =
      sample_uniform_in_range(&s->rng, pp->sway_freq_min, pp->sway_freq_max);
  f->sway_phase = sample_random_phase_2pi(&s->rng);
  f->age = 0.0f;
  f->size_idx = sample_size_class_small_biased(&s->rng);
  f->active = true;
}

static void scene_clear_flakes(Scene *s) {
  for (int i = 0; i < MAX_FLAKES; i++)
    s->flakes[i].active = false;
}

static void scene_clear_pile(Scene *s) { memset(s->pile, 0, sizeof s->pile); }

/*
 * scene_prewarm — fill the flake pool to target with flakes scattered
 * uniformly across the visible y range so the screen looks like
 * steady-state snow from frame 1, not a marching-band wave at the top.
 */
static void scene_prewarm(Scene *s) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  int target = pp->target_flakes;
  if (target > MAX_FLAKES)
    target = MAX_FLAKES;

  int active = 0;
  for (int i = 0; i < MAX_FLAKES; i++)
    if (s->flakes[i].active)
      active++;

  float y_max = (float)(s->rows - 2);
  for (int k = active; k < target; k++)
    scene_spawn_flake(s, -6.0f, y_max);
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->current_theme = 0;
  s->current_pattern = PATTERN_SNOWFALL;
  s->wind_override = 0.0f;
  s->rng = (uint32_t)clock_ns();
  s->cols = cols;
  s->rows = rows;
  s->time_accum = 0.0f;
  scene_clear_flakes(s);
  scene_clear_pile(s);
  scene_prewarm(s);
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
  /* Pile data preserved (truncated if cols shrinks). */
}

static void scene_reseed(Scene *s) {
  s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
  s->wind_override = 0.0f;
  scene_clear_flakes(s);
  scene_clear_pile(s);
  scene_prewarm(s);
}

/* Population census — linear scan of the pool's active flags. */
static int count_active_flakes(const Scene *s) {
  int n = 0;
  for (int i = 0; i < MAX_FLAKES; i++)
    if (s->flakes[i].active)
      n++;
  return n;
}

/* Spawn budget for this tick — refill toward pattern.target_flakes, but
 * cap the per-tick burst proportional to dt so a long pause doesn't
 * dump a flood of flakes when the sim resumes. */
static int compute_spawn_count_for_tick(int active, const PatternParams *pp,
                                        float dt) {
  int target = pp->target_flakes;
  if (target > MAX_FLAKES)
    target = MAX_FLAKES;
  int spawn_cap = (int)((float)pp->target_flakes * dt * 4.0f) + 4;
  int n = target - active;
  if (n < 0)
    n = 0;
  if (n > spawn_cap)
    n = spawn_cap;
  return n;
}

/* Spawn `n` flakes just above the visible top (y ∈ [-6, -1]). */
static void flake_pool_topup_from_sky(Scene *s, int n) {
  for (int k = 0; k < n; k++)
    scene_spawn_flake(s, -6.0f, -1.0f);
}

/* Resolve one flake against the pile at its swayed column.  On contact:
 * apply the pattern's growth Bernoulli (so FLURRY accumulates slower than
 * BLIZZARD) then valley-fill deposit and deactivate.  If the sway pushed
 * the flake briefly off-screen the contact test is skipped — it stays in
 * flight and will swing back into a valid column next tick. */
static void try_pile_contact_and_deposit(Scene *s, Flake *f,
                                         const PatternParams *pp,
                                         int max_pile_h) {
  float fx = flake_x(f);
  int col = (int)(fx + 0.5f);
  if (col < 0 || col >= s->cols)
    return;

  float floor_y = pile_floor_y(s->pile[col], s->rows);
  if (f->y < floor_y)
    return;

  if (bernoulli_trial(&s->rng, pp->pile_growth_mul))
    pile_deposit(s->pile, s->cols, col, max_pile_h);
  f->active = false;
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  dt *= (float)s->speed / (float)SPEED_DEF;
  s->time_accum += dt;

  const PatternParams *pp = &pattern_params[s->current_pattern];

  /* 1. Top up flake pool toward pattern's target density. */
  int to_spawn = compute_spawn_count_for_tick(count_active_flakes(s), pp, dt);
  flake_pool_topup_from_sky(s, to_spawn);

  /* 2. Integrate each flake; resolve off-screen cull and pile contact. */
  int max_pile_h = compute_pile_height_cap(s->rows);
  for (int i = 0; i < MAX_FLAKES; i++) {
    Flake *f = &s->flakes[i];
    if (!f->active)
      continue;

    integrate_flake_euler(f, dt);

    if (flake_drifted_offscreen_x(f, s->cols)) {
      f->active = false;
      continue;
    }

    try_pile_contact_and_deposit(s, f, pp, max_pile_h);
  }
}

/* Brightness attribute by ramp slot — top of ramp BOLD, bottom DIM.
 * Shared rule between pile depth-ramp and flake size-ramp so both layers
 * read the gradient consistently. */
static inline int ramp_attr_by_brightness_slot(int slot) {
  if (slot >= 6)
    return A_BOLD;
  if (slot <= 1)
    return A_DIM;
  return A_NORMAL;
}

/* Pile glyph picked by depth from the surface — `*` snow-cap, `#` packed,
 * `+` settled.  Encodes the visual "depth feel" without using non-ASCII. */
static inline char pile_cell_glyph_by_depth(int depth_from_top) {
  if (depth_from_top == 0)
    return '*';
  if (depth_from_top < 3)
    return '#';
  return '+';
}

/* Draw one column's stack of pile cells from the bottom row upward.
 * k=0 is the topmost (surface) cell and gets the brightest ramp slot. */
static void pile_column_draw(int col, int height, int rows_eff) {
  for (int k = 0; k < height; k++) {
    int y = (rows_eff - 1) - k;
    if (y < 0)
      break;

    int ramp_slot = 7 - k;
    if (ramp_slot < 0)
      ramp_slot = 0;

    char glyph = pile_cell_glyph_by_depth(k);
    int attr = ramp_attr_by_brightness_slot(ramp_slot);
    int pair = PAIR_PILE_BASE + ramp_slot;

    attron(COLOR_PAIR(pair) | attr);
    mvaddch(y, col, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
  }
}

/* Render the pile across every visible column — drawn FIRST so falling
 * flakes overlay it cleanly. */
static void pile_draw_all_columns(const Scene *s, int rows_eff) {
  int cap = s->cols < MAX_COLS ? s->cols : MAX_COLS;
  for (int c = 0; c < cap; c++) {
    int h = s->pile[c];
    if (h > 0)
      pile_column_draw(c, h, rows_eff);
  }
}

/* Render one active flake at its current swayed (x, y), using size-class
 * for glyph (FLAKE_GLYPHS ramp) and colour pair (PAIR_FLAKE_BASE + size). */
static void flake_draw(const Flake *f, int cols, int rows_eff) {
  int ix = (int)(flake_x(f) + 0.5f);
  int iy = (int)(f->y + 0.5f);
  if (ix < 0 || ix >= cols)
    return;
  if (iy < 0 || iy >= rows_eff)
    return;

  char glyph = FLAKE_GLYPHS[f->size_idx];
  int pair = PAIR_FLAKE_BASE + f->size_idx;
  int attr = ramp_attr_by_brightness_slot(f->size_idx);

  attron(COLOR_PAIR(pair) | attr);
  mvaddch(iy, ix, (chtype)(unsigned char)glyph);
  attroff(COLOR_PAIR(pair) | attr);
}

/* Render every active flake on top of the pile. */
static void flakes_draw_all_active(const Scene *s, int rows_eff) {
  for (int i = 0; i < MAX_FLAKES; i++) {
    const Flake *f = &s->flakes[i];
    if (f->active)
      flake_draw(f, s->cols, rows_eff);
  }
}

static void scene_draw(const Scene *s) {
  int rows_eff = s->rows - 1;         /* leave bottom row for HUD */
  pile_draw_all_columns(s, rows_eff); /* pile first — flakes overlay */
  flakes_draw_all_active(s, rows_eff);
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

/* Active flake count + max pile height for HUD. */
static void scene_counts(const Scene *s, int *out_flakes, int *out_max_pile) {
  int n = 0;
  for (int i = 0; i < MAX_FLAKES; i++)
    if (s->flakes[i].active)
      n++;
  *out_flakes = n;

  int mp = 0;
  int cap = s->cols < MAX_COLS ? s->cols : MAX_COLS;
  for (int c = 0; c < cap; c++)
    if (s->pile[c] > mp)
      mp = s->pile[c];
  *out_max_pile = mp;
}

/*
 * screen_draw — render the scene, then paint a two-layer HUD over it:
 *
 *   Row 0          STATUS LINE.  Bright yellow PAIR_HUD + A_BOLD.
 *                  Live state: pattern (or PAUSED), theme, flake count,
 *                  max pile height, wind (cells/sec), fps, sim Hz,
 *                  speed multiplier.
 *   Row rows-1     KEY HINT LINE.  Bright cyan PAIR_HINT + A_BOLD.
 *                  Every interactive key the demo accepts.
 *
 * Both rows are pre-filled with their pair colour so the coloured
 * background spans the full width, and drawn AFTER scene_draw so
 * flakes never bleed through the bars.
 */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(s);

  int flakes, max_pile;
  scene_counts(s, &flakes, &max_pile);
  const PatternParams *pp = &pattern_params[s->current_pattern];
  float wind = pp->wind_x + s->wind_override;

  const char *state_str =
      s->paused ? "PAUSED " : pattern_name(s->current_pattern);

  /* ── Top row: dynamic status ─────────────────────────────── */
  char status[220];
  snprintf(status, sizeof status,
           " SNOW   %s   theme:%-8s   flakes:%4d  pile_h:%2d   "
           "wind:%+5.1f c/s   %5.1f fps  %3d Hz  speed:%-3d ",
           state_str, themes[s->current_theme].name, flakes, max_pile,
           (double)wind, fps, sim_fps, s->speed);

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  for (int x = 0; x < sc->cols; x++)
    mvaddch(0, x, ' ');
  mvprintw(0, 0, "%s", status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* ── Bottom row: every interactive key ───────────────────── */
  const char *hints =
      " q:quit  spc:pause  r:reseed  c:clear  n/p:pattern  t/T:theme  "
      "w/W:wind  +/-:speed  ]/[:Hz ";

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
  case 'c':
    scene_clear_pile(s);
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
    scene_prewarm(s);
    break;
  case 'p':
  case 'P':
    s->current_pattern =
        (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
    scene_prewarm(s);
    break;

  case 'w':
    s->wind_override += WIND_STEP;
    break;
  case 'W':
    s->wind_override -= WIND_STEP;
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
