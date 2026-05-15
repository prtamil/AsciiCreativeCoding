/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * rain.c — falling rain with motion-blur trails and ground splashes
 *
 * DEMO: Rain drops fall diagonally across the screen (wind-tunable),
 *       each drop drawn as a short motion-blur streak in the active
 *       theme's colour ramp (head bright, tail fading). When a drop
 *       reaches the bottom of the screen it disappears and spawns a
 *       small handful of SPLASH particles that arc up-and-outward
 *       under gravity, then fall back. Density / speed / wind are
 *       set per pattern: DRIZZLE → SHOWER → STORM → MONSOON.
 *
 * Study alongside:
 *   brust.c      — same particle-pool pattern (active flag, fixed-
 *                  size array, object reuse).
 *   physics/bounce_ball.c — same gravity-integration idiom for the
 *                  splash particles' arc.
 *
 * Section map:
 *   §1 config    — constants, themes, per-pattern parameters
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-pair theme ramp + splash + sky pairs
 *   §4 drop      — Drop struct, drop_tick, drop_glyph_for_slope
 *   §5 splash    — Splash struct, splash_tick
 *   §6 scene     — pools, scene_tick, scene_draw, scene_spawn
 *   §7 screen    — ncurses init / draw / resize
 *   §8 app       — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reseed (clear & re-spawn drops with new RNG)
 *   n / N      next pattern   (DRIZZLE → SHOWER → STORM → MONSOON)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster (speed multiplier ×2)
 *   -          slower (÷2)
 *   ] / [      raise / lower tick Hz
 *   w / W      wind right / left (override pattern wind by ±5 c/s)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/rain.c \
 *       -o rain -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Pool-based 2-D particle system with two species.
 *                  DROPS spawn at the top, fall under constant gravity +
 *                  wind, and DIE when they cross the bottom row. On
 *                  death each drop emits SPLASH particles that arc up
 *                  briefly then fall under gravity. Both pools are
 *                  fixed-size object pools (no malloc per particle):
 *                  on spawn we scan for the first inactive slot;
 *                  on death we clear `active = false` and the slot is
 *                  available for reuse next tick.
 *
 *                  Per pattern (DRIZZLE / SHOWER / STORM / MONSOON) we
 *                  set: target_drops (steady-state count), drop_speed,
 *                  wind_x, motion-blur length range, splash multiplier.
 *                  Each tick we count active drops; if below target we
 *                  spawn one new drop at a random column. Steady-state
 *                  density is therefore self-correcting under resize.
 *
 *                  Drop rendering uses the SLOPE of the velocity vector
 *                  to pick a glyph (vertical → '|', diagonal → '/'/'\\',
 *                  shallow → '~'). The motion-blur trail is `length`
 *                  cells of progressively-dimmer glyphs along the
 *                  velocity direction (head bright, tail faint), which
 *                  reads as motion-streak at low res.
 *
 * Data-structure : Two object pools — Drop[MAX_DROPS] and
 *                  Splash[MAX_SPLASHES]. Each entry has an `active`
 *                  flag and the rest of its physics fields. Spawn does
 *                  a linear scan (small N — fine). No allocator.
 *
 * Rendering      : ASCII only. Drops render as a motion-blur streak
 *                  using glyphs `|`, `/`, `\`, `~` plus the project's
 *                  airy ramp `' .,:;-+*'` for the fading tail. Splash
 *                  particles render as one of `.,'`. No background fill.
 *
 * Performance    : O(MAX_DROPS + MAX_SPLASHES) per tick. With MONSOON
 *                  defaults at 700 drops + ~600 splashes, that's
 *                  ~1300 particles × ~5 cells of trail draw each ≈ 7k
 *                  mvaddch per frame. At 60 fps ≈ 420 k/sec — well
 *                  inside ncurses' write budget.
 *
 * References
 * ──────────
 *   PAPERS
 *     Reeves, W. T. (1983)
 *       "Particle Systems — A Technique for Modeling a Class of Fuzzy Objects"
 *       ACM Transactions on Graphics 2(2): 91-108.
 *       Foundational paper.  The pool-based two-species model used
 *       here (drops + impact-triggered splash particles) is exactly
 *       Reeves' design; splash-on-impact is the canonical "particle
 *       spawns particles" composition pattern.
 *
 *     Garg, K. & Nayar, S. K. (2007)
 *       "Vision and Rain"
 *       International Journal of Computer Vision 75(1): 3-27.
 *       Physical optics of rain streaks: drops reach terminal velocity
 *       almost immediately, and what cameras see is the motion-blur
 *       STRETCH along the velocity vector — not a point at the drop's
 *       current position.  The slope-based glyph ('|' vertical, '/'
 *       and '\' diagonal) and the multi-cell tail-to-head trail in
 *       scene_draw are the ASCII analogue of camera-blur streaks.
 *
 *   BOOKS
 *     Bourg, D. M. & Bywalec, B. — "Physics for Game Developers"
 *       (2nd ed, O'Reilly, 2013).  Chapters 2-4: terminal velocity
 *       under drag (why real drops — and these — fall at a constant
 *       speed rather than accelerating indefinitely under gravity);
 *       ballistic motion + drag for the splash arc.
 *
 *     Witkin, A. & Baraff, D. (2001)
 *       "Physically Based Modeling: Principles and Practice"
 *       SIGGRAPH course notes (online proceedings).  §1 — particle
 *       dynamics under constant force fields, the integrator the
 *       splash species uses (vy += g·dt; vx *= drag; p += v·dt).
 *
 *     Akenine-Möller, T., Haines, E. & Hoffman, N. — "Real-Time
 *       Rendering" (4th ed, CRC Press, 2018).
 *       §13.7 — point-sprite particle rendering with motion-blur
 *       smearing; the 8-step tail-to-head density ramp here is the
 *       cell-grid analogue of camera-blur-stretched point sprites.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE — DATA-DRIVEN PATTERN ENGINE ────────────────────────── *
 *
 * Four rain intensities (DRIZZLE, SHOWER, STORM, MONSOON) are
 * produced by a SINGLE generic two-species engine — falling DROPS
 * + bouncing SPLASH particles — driven by an array of PatternParams
 * structs.  scene_tick and scene_draw never branch on the pattern
 * enum to decide HOW the physics works; they read
 * pattern_params[s->current_pattern] and compute accordingly.
 * Adding a new intensity is a matter of appending one row to
 * pattern_params[]; no new code paths.
 *
 *
 * THE GENERIC ENGINE (pseudocode)
 * ───────────────────────────────
 *
 *   loop forever (each tick of dt seconds):
 *
 *     pp = pattern_params[scene.current_pattern]   # read inputs
 *
 *     # 1. SPAWN DROPS — top up drop pool toward target density
 *     while count(active drops) < pp.target_drops:
 *         d = next_inactive_drop_slot()
 *         d.x      = wind_aware_uniform_x(scene.cols, pp.wind_x)
 *         d.y      = uniform(-6, -1)               # just above top
 *         d.vy     = pp.drop_speed                 # terminal velocity
 *         d.vx     = pp.wind_x + scene.wind_override
 *         d.length = uniform(pp.length_min, pp.length_max)
 *         d.active = true
 *
 *     # 2. INTEGRATE DROPS — constant-velocity fall (no gravity:
 *     #    raindrops reach terminal velocity almost immediately,
 *     #    Garg & Nayar 2007).
 *     for d in active drops:
 *         d.x += d.vx · dt
 *         d.y += d.vy · dt
 *
 *     # 3. FLOOR IMPACT — drop hits the bottom row → spawn splashes
 *     for d in active drops where d.y >= floor:
 *         n_splash = round(SPLASH_BASE_PER_DROP · pp.splash_mul)
 *         for k in 0..n_splash:
 *             s = next_inactive_splash_slot()
 *             s.x, s.y = d.x, floor
 *             (s.vx, s.vy) = polar(angle≈up, speed_jitter)
 *             s.life       = SPLASH_LIFE
 *             s.active     = true
 *         d.active = false      # drop consumed
 *
 *     # 4. INTEGRATE SPLASHES — explicit Euler with gravity + drag
 *     for s in active splashes:
 *         s.vy += GRAVITY · dt
 *         s.vx *= SPLASH_DRAG_FACTOR
 *         s.x  += s.vx · dt
 *         s.y  += s.vy · dt
 *         s.age += dt
 *         if s.age >= s.life:  s.active = false
 *
 *     # 5. CULL — drops drifted off-screen sideways die quietly
 *     for d in active drops:
 *         if d.x off-screen:  d.active = false
 *
 *     # 6. RENDER — drops as 8-step tail→head motion-blur streaks,
 *     #    splash particles as single dim glyphs.
 *     for d in active drops:
 *         glyph = drop_glyph_for_slope(d.vx, d.vy)   # | / \ ~
 *         paint_motion_blur_streak(d, ramp[0..7])
 *     for s in active splashes:
 *         paint(s.x, s.y, '·', PAIR_SPLASH)
 *
 *
 * PATTERNPARAMS FIELD → ENGINE HOOK
 * ─────────────────────────────────
 *
 *   target_drops     →  spawn-loop refill cap        (sky density)
 *   drop_speed       →  vy at spawn                  (terminal velocity)
 *   wind_x           →  vx + spawn-x extension       (Garg & Nayar slope)
 *   length_min       →  motion-blur streak length    (camera-blur stretch)
 *   length_max
 *   splash_mul       →  splashes per impact          (= mul·SPLASH_BASE)
 *
 * Every other engine constant — MAX_DROPS, MAX_SPLASHES, SPLASH_BASE_PER_DROP,
 * SPLASH_LIFE, SPLASH_DRAG_FACTOR, GRAVITY, WIND_STEP — is a GLOBAL
 * tuning knob shared across all patterns.  Patterns differ ONLY in
 * the six fields above.
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
 *     architecture for natural-particle simulations — rain is one
 *     of Reeves' enumerated examples.  The two-species design here
 *     (drops + splashes, each with its own pool) is also Reeves'
 *     suggestion for compound natural phenomena.
 *
 *   Gamma, E., Helm, R., Johnson, R. & Vlissides, J. (1994)
 *     "Design Patterns" (Addison-Wesley) — STRATEGY pattern (§5.9).
 *     A family of algorithms (DRIZZLE / SHOWER / STORM / MONSOON)
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
 *     data table; the engine has no per-pattern code paths and
 *     the cache footprint stays predictable across pattern
 *     switches.
 *
 *   Nystrom, R. (2014)
 *     "Game Programming Patterns" (Genever Benning).
 *     TYPE OBJECT chapter — PatternParams is a Type Object: one
 *     shared instance per "kind" of rainstorm.  DATA LOCALITY
 *     chapter — both Drop and Splash are flat-laid-out PODs swept
 *     linearly by the integrator each tick, exactly the layout
 *     Nystrom recommends for hot inner loops.  OBJECT POOL chapter
 *     — the fixed-size BSS arrays here implement Nystrom's pool
 *     idiom directly (active flag + linear scan for next slot).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A pool of N inactive drops sits in memory. Each frame, the scene
 * checks how many drops are active; if fewer than the pattern's
 * target count, it activates the next inactive slot — placing it at
 * the top of the screen with the pattern's wind + speed. Active
 * drops integrate forward; when they reach the bottom, they
 * deactivate and trigger a small flurry of splash particles. The
 * SAME slot is reused next frame for the next drop. Nothing is
 * ever allocated or freed at runtime.
 *
 * ALGORITHM IN STEPS  (each step = one helper in §6)
 * ──────────────────
 *  1. EMIT TO TARGET (drops_emit_to_target → scene_spawn_drop):
 *     count active drops; if below pattern.target_drops, spawn the
 *     difference, capped at (target × dt × 4 + 4) so a long pause
 *     doesn't release a flood when the loop resumes. New drops get
 *     a wind-aware spawn x (drop_spawn_x_wind_aware) so wind-tilted
 *     streams enter from the side as well as from above.
 *
 *  2. INTEGRATE DROPS (drops_integrate_and_cull →
 *     drop_step_constant_velocity):
 *       drop.vx = pattern.wind_x + scene.wind_override (jittered)
 *       drop.vy = pattern.drop_speed                   (jittered)
 *       x += vx · dt;  y += vy · dt
 *     Terminal-velocity model — no gravity applied to the drop layer
 *     (real drops reach terminal velocity almost immediately).
 *
 *  3. KILL & SPLASH. drops_integrate_and_cull triggers
 *     scene_emit_splashes when drop.y >= rows - 2: spawn
 *     K = round(SPLASH_BASE_PER_DROP · pattern.splash_mul) splash
 *     particles at the impact with random outward velocity (vy
 *     starts negative for the upward kick + symmetric vx).
 *
 *  4. INTEGRATE SPLASHES (splashes_integrate_and_cull →
 *     splash_step_kinematic):
 *       vy += SPLASH_GRAVITY · dt          (constant gravity)
 *       vx *= exp(-SPLASH_DRAG · dt)        (closed-form drag)
 *       x += vx · dt;  y += vy · dt
 *       age += dt
 *       deactivate when age ≥ life OR y past kill_y
 *
 *  5. RENDER (scene_draw):
 *     drops_render → drop_render_with_trail: drop_glyph_for_slope
 *       picks the head ('|', '/', '\', '~'); the trail walks BACK
 *       along (-vx, -vy) for `length` cells with glyphs from the
 *       tail-to-head airy ramp.
 *     splashes_render → splash_render: splash_life_phase maps age/life
 *       to ('*' / '+' / '.', A_BOLD / NORMAL / DIM).
 *
 *  6. TWO-LAYER HUD (screen_draw):
 *     top row → screen_paint_status_bar (bright yellow status with
 *       pattern, theme, drop count, splash count, wind, fps, Hz, speed);
 *     bottom row → screen_paint_hint_bar (bright cyan key hints).
 *
 * KEY FORMULAS
 * ────────────
 *  Velocity slope (for glyph selection):
 *    slope = vy / |vx|     // ∞ if vx=0 (vertical)
 *    glyph = |    if |slope| > 4
 *          = /    if vx < 0 and slope < 4
 *          = \    if vx > 0 and slope < 4
 *          = ~    if |slope| < 0.5  (shallow / horizontal)
 *
 *  Motion-blur trail (length L cells along (-vx, -vy)):
 *    step_x = -vx / |v| · TRAIL_SPACING
 *    step_y = -vy / |v| · TRAIL_SPACING
 *    for i = 0..L-1:
 *      px = drop.x + step_x · i
 *      py = drop.y + step_y · i
 *      tint = ramp[max(0, 7 - i)]                 // head 7, fades down
 *      glyph = ramp_glyph[max(0, 7 - i)]
 *
 *  Splash arc (born with vy < 0 → goes up before falling):
 *    vy0 = -SPLASH_KICK_UP · (0.6 + rand·0.4)     // upward
 *    vx0 = SPLASH_KICK_X · (rand·2 - 1)           // ±horizontal
 *    life = 0.30 + rand·0.40                       // seconds
 *    vy += SPLASH_GRAVITY · dt
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

  MAX_DROPS = 1024,
  MAX_SPLASHES = 800,
  SPLASH_BASE_PER_DROP = 4,

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_RAIN_BASE = 3, /* +0..+7 = 8 rain ramp tints (tail→head) */
  PAIR_SPLASH = 11,
  PAIR_SKY = 12,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* Splash physics (cells / second; cells / second²). */
#define SPLASH_GRAVITY 200.0f
#define SPLASH_KICK_UP 35.0f /* initial upward velocity        */
#define SPLASH_KICK_X 25.0f  /* ± horizontal kick range        */
#define SPLASH_DRAG 2.0f     /* multiplicative per second      */
#define SPLASH_LIFE_MIN 0.30f
#define SPLASH_LIFE_MAX 0.65f

/* Drop trail spacing (cells per trail step). */
#define TRAIL_SPACING_MIN 0.55f
#define TRAIL_SPACING_MAX 0.85f

/* Per-drop physics jitter — breaks up synchronised "block descent"
 * that otherwise makes slow patterns (DRIZZLE) look layered.
 *
 * SPEED_VARIANCE is the ±fraction applied to each drop's terminal
 * velocity at spawn (0.40 → ±20% of pattern speed). With variance,
 * drops at the same starting y reach the bottom at different times,
 * so the overall rain looks like independent particles rather than
 * a uniform sheet sliding down.
 *
 * WIND_JITTER is per-drop ±cells/sec added to the pattern wind so
 * drops don't all slant at exactly the same angle. */
#define DROP_SPEED_VARIANCE 0.40f
#define DROP_WIND_JITTER 2.0f

/* Wind override step (cells/sec). */
#define WIND_STEP 5.0f

/* Pattern enum. */
typedef enum {
  PATTERN_DRIZZLE = 0,
  PATTERN_SHOWER = 1,
  PATTERN_STORM = 2,
  PATTERN_MONSOON = 3,
  N_PATTERNS = 4,
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_DRIZZLE:
    return "DRIZZLE";
  case PATTERN_SHOWER:
    return "SHOWER ";
  case PATTERN_STORM:
    return "STORM  ";
  case PATTERN_MONSOON:
    return "MONSOON";
  default:
    return "?      ";
  }
}

/*
 * PatternParams — physics + density knobs that distinguish one rain
 * INTENSITY pattern from another. The simulation engine never branches
 * on the pattern enum; it reads these fields and behaves accordingly.
 * Same code, four different storms. Switching pattern (n/N keys) just
 * swaps which row of this table the spawn loop reads.
 *
 *   target_drops  : steady-state count of active drops on screen.
 *                   The spawn loop refills toward this each tick.
 *                   Higher = denser sheet of rain.
 *                   DRIZZLE 150 (light), MONSOON 800 (wall of water).
 *
 *   drop_speed    : drop TERMINAL VELOCITY in cells/sec. Drops fall at
 *                   constant speed — no gravity is applied to the drop
 *                   layer because real raindrops reach terminal
 *                   velocity almost immediately (see Garg & Nayar).
 *                   Higher = visibly faster streaks.
 *                   35 c/s drizzle → 190 c/s monsoon.
 *
 *   wind_x        : default horizontal velocity in cells/sec applied
 *                   at spawn. Sign convention: negative = leftward
 *                   wind, positive = rightward. The (wind_x, drop_speed)
 *                   pair sets the streak slope that drop_glyph_for_slope
 *                   reads to pick '|', '/', '\', or '~'. The user adds
 *                   Scene.wind_override on top via w/W keys.
 *
 *   length_min,   : motion-blur trail length range in cells. Per-drop
 *   length_max     length sampled uniformly between min and max at
 *                   spawn. Longer = stretched-out streaks (the camera-
 *                   motion-blur effect Garg & Nayar formalise).
 *                   DRIZZLE 1.0-2.0 cells; MONSOON 4.0-6.5 cells.
 *
 *   splash_mul    : multiplier on SPLASH_BASE_PER_DROP — controls how
 *                   many splash particles each bottom-row impact spawns.
 *                   DRIZZLE 0.4 (sparse splatters); MONSOON 1.3 (heavy
 *                   continuous line of splashes along the bottom).
 */
typedef struct {
  int target_drops;
  float drop_speed;
  float wind_x;
  float length_min;
  float length_max;
  float splash_mul;
} PatternParams;

static const PatternParams pattern_params[N_PATTERNS] = {
    /* DRIZZLE  */ {150, 35.0f, 3.0f, 1.0f, 2.0f, 0.40f},
    /* SHOWER   */ {240, 75.0f, 12.0f, 2.0f, 3.5f, 0.70f},
    /* STORM    */ {480, 130.0f, 35.0f, 3.0f, 5.0f, 1.00f},
    /* MONSOON  */ {800, 190.0f, 60.0f, 4.0f, 6.5f, 1.30f},
};

/*
 * Themes — 8-step ramp from TAIL (sparse, dim) to HEAD (dense, bright)
 * of a rain drop's motion-blur trail. ramp[0] is tail tint, ramp[7] is
 * head tint. Splash colour and optional sky tint round it out.
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule.
 */
typedef struct {
  const char *name;
  short ramp[8]; /* tail → head */
  short splash;
  short sky;
} Theme;

#define N_THEMES 11

static const Theme themes[N_THEMES] = {
    /* name          ramp[0..7]  (tail dim → head bright)            splash sky
     */

    {"MATRIX", {28, 34, 40, 46, 82, 118, 154, 190}, 154, 234},
    {"FIRE", {88, 124, 130, 166, 196, 208, 214, 226}, 226, 233},
    {"OCEANIC", {24, 25, 31, 38, 44, 51, 87, 159}, 159, 234},
    {"NEON", {53, 91, 134, 165, 201, 207, 213, 219}, 219, 234},
    {"MONO", {240, 243, 245, 247, 249, 251, 253, 255}, 255, 232},
    {"ICE", {24, 31, 67, 110, 117, 153, 195, 231}, 231, 235},
    {"NOVA", {24, 75, 117, 159, 195, 219, 226, 231}, 231, 234},
    {"FOREST", {28, 64, 70, 76, 112, 148, 184, 220}, 184, 234},
    {"DESERT", {94, 130, 137, 143, 179, 215, 222, 229}, 222, 234},
    {"ECLIPSE", {52, 88, 95, 131, 167, 173, 209, 215}, 209, 232},
    {"TROPICAL", {29, 35, 37, 44, 50, 86, 122, 159}, 158, 234},
};

/* Tail-to-head density ramp glyphs (project-airy variant). */
static const char RAMP_GLYPHS[8] = {'`', '.', ',', ':', ';', '-', '+', '*'};

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
      init_pair((short)(PAIR_RAIN_BASE + i), t->ramp[i], -1);
    init_pair(PAIR_SPLASH, t->splash, -1);
    init_pair(PAIR_SKY, t->sky, -1);
  } else {
    static const short fb[8] = {
        COLOR_BLUE, COLOR_BLUE,  COLOR_CYAN,  COLOR_CYAN,
        COLOR_CYAN, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
    };
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_RAIN_BASE + i), fb[i], -1);
    init_pair(PAIR_SPLASH, COLOR_CYAN, -1);
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
/* §4  drop — one falling rain particle                                  */
/* ===================================================================== */

/*
 * Drop — one falling rain particle.
 *
 * INTENT: hold every per-drop value the integrator (§7) and render
 * pass (§9) need, in a flat layout that fits into a fixed-size BSS
 * array (no malloc after init). Drops never accelerate — the physics
 * model is terminal velocity (constant vy), so once spawned the only
 * way velocity changes is at the user's wind toggle.
 *
 * LIFECYCLE: spawn at top edge with a random column + the pattern's
 * (wind_x + scene.wind_override, drop_speed) velocity → integrate
 * forward each tick → on bottom-row impact, mark inactive and emit
 * splash species via scene_emit_splashes. Slots are reused linearly
 * by the next spawn — pool_find_inactive returns the first inactive
 * index it sees.
 *
 *   x, y    : current position in cell-space FLOAT. Sub-cell precision
 *             is what lets the motion-blur trail render as smooth
 *             motion rather than snapping between integer cells each
 *             frame. Rounding to integer cells happens once, at render
 *             time, inside scene_draw.
 *
 *   vx, vy  : velocity in cells/sec.
 *               vx = pattern.wind_x + scene.wind_override
 *                   (signed; negative = leftward, positive = right).
 *               vy = pattern.drop_speed
 *                   (always positive = falling; no gravity applied —
 *                    terminal-velocity model, see PatternParams).
 *             The (vx, vy) pair drives BOTH the integrator and the
 *             streak-glyph slope decision in drop_glyph_for_slope —
 *             vertical slope picks '|', diagonals pick '/' or '\\',
 *             shallow slope picks '~'.
 *
 *   length  : motion-blur trail length in cells, sampled at spawn
 *             from [pattern.length_min, pattern.length_max]. Longer
 *             = stretchier streak (the ASCII analogue of the camera-
 *             motion-blur stretch Garg & Nayar describe).
 *
 *   spacing : trail step size in cells — distance between consecutive
 *             trail glyphs along the (-vx, -vy) direction. Tuned per-
 *             drop so a slow drizzle drop still shows a recognisable
 *             trail instead of collapsing onto its head.
 *
 *   active  : pool-slot occupancy flag. Inactive slots are skipped by
 *             every loop; spawn finds the first inactive index via
 *             linear scan (cheap at MAX_DROPS).
 */
typedef struct {
  float x, y;
  float vx, vy;
  float length;
  float spacing;
  bool active;
} Drop;

/* Cheap RNG (LCG) — per-scene state, no global aliasing */
static inline uint32_t lcg_next(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}
static inline float lcg_unit(uint32_t *st) {
  return (float)(lcg_next(st) >> 8) / (float)(1u << 24); /* [0, 1) */
}

/*
 * drop_glyph_for_slope() — pick a streak glyph from velocity direction.
 *
 *   nearly vertical (|vy/|vx|| > 4)         → '|'
 *   diagonal right  (vx > 0, slope > 0.5)   → '\'   (looks like \  )
 *   diagonal left   (vx < 0, slope > 0.5)   → '/'
 *   shallow         (|slope| < 0.5)         → '~'
 *
 * Returns the head-of-streak glyph; the trail behind uses the
 * airy ramp tail-to-head.
 */
static char drop_glyph_for_slope(float vx, float vy) {
  float ax = fabsf(vx);
  float ay = fabsf(vy);
  if (ax < 1e-3f)
    return '|';
  float slope = ay / ax;
  if (slope > 4.0f)
    return '|';
  if (slope < 0.5f)
    return '~';
  return (vx > 0.0f) ? '\\' : '/';
}

/* ===================================================================== */
/* §5  splash — small particles emitted on drop impact                   */
/* ===================================================================== */

/*
 * Splash — short-lived fragment emitted at the impact point when a
 * Drop hits the bottom row.  Different physics from Drop:
 *
 *   Drop    : constant velocity (terminal-velocity model), no gravity.
 *   Splash  : ballistic motion with gravity AND linear drag — the
 *             classic bounce-up-then-fall arc you see when raindrops
 *             hit a puddle.
 *
 * LIFECYCLE: born at (impact_x, impact_y) with an upward kick + a
 * symmetric horizontal kick → integrates under gravity + drag → dies
 * when age >= life. Pool-slot reuse identical to Drop.
 *
 *   x, y    : current position in cells. Initialised by
 *             scene_emit_splashes to the drop's impact coordinates.
 *
 *   vx, vy  : velocity in cells/sec.  At spawn:
 *               vy starts NEGATIVE (upward kick of magnitude
 *                  SPLASH_KICK_UP × random ∈ [0.6, 1.0])
 *               vx is symmetric around zero (±SPLASH_KICK_X)
 *             so fragments fan outward from the impact.  During
 *             integration vx decays exponentially via SPLASH_DRAG
 *             while vy accumulates SPLASH_GRAVITY → the parabolic
 *             arc.
 *
 *   age     : seconds since this splash was emitted. Drives BOTH
 *             death (age >= life) and the glyph selector in
 *             scene_draw (fresh → '*', mid → '+', old → '.').
 *
 *   life    : random target lifetime drawn at spawn from
 *             [SPLASH_LIFE_MIN, SPLASH_LIFE_MAX]. Per-splash
 *             randomness scatters the death moments — what reads as
 *             organic fade instead of every splash dying in lockstep.
 *
 *   active  : pool-slot occupancy flag (same role as Drop.active).
 */
typedef struct {
  float x, y;
  float vx, vy;
  float age;
  float life;
  bool active;
} Splash;

/* ===================================================================== */
/* §6  scene — pools, tick, draw                                         */
/* ===================================================================== */

/*
 * Scene — owns every piece of mutable state for the rain simulation.
 * Two clearly-separated halves:
 *
 *   SIMULATION half — what the physics tick reads + writes. Owns the
 *                     active pattern, force overrides, RNG, cached
 *                     terminal dims, and both particle pools. Mutated
 *                     by scene_tick() and the key handler.
 *
 *   RENDER half     — what scene_draw() consults to pick colours.
 *                     Purely visual selection index; never read inside
 *                     the physics tick.
 *
 * The Scene knows nothing about ncurses — physics writes to the pools,
 * the render layer consults them. That separation lets the physics
 * be exercised without a terminal (useful for headless tests).
 */
typedef struct {
  /* ──────────────────────────────────────────────────────────────
   *  SIMULATION HALF — physics tick reads + writes these
   * ────────────────────────────────────────────────────────────── */

  /* PAUSE — scene_tick is a no-op when set. Toggled by space-bar.
   * Render keeps running so the user sees a frozen frame with
   * drops + splashes mid-flight. */
  bool paused;

  /* SPEED — integer multiplier on dt. Default SPEED_DEF means 1×
   * wall clock; +/= keys double, - halves. Bounded by
   * SPEED_MIN/MAX. Doesn't change physics constants — just
   * compresses or stretches simulated time. */
  int speed;

  /* PATTERN — index into pattern_params (DRIZZLE / SHOWER / STORM /
   * MONSOON). Cycled by n / N. Switching pattern doesn't rebuild
   * pools — the new target_drops self-fills (or self-drains) over
   * a few seconds, which reads as a natural intensity change rather
   * than a jarring scene cut. */
  Pattern current_pattern;

  /* WIND OVERRIDE — added to pattern.wind_x to compute the actual
   * vx applied to newly-spawned drops. The w / W keys add ±WIND_STEP
   * here. Persists across pattern switches; reset on 'r' along
   * with the rest of the scene. Lets the user push the rain
   * sideways live without rebuilding the pattern table. */
  float wind_override;

  /* RNG — per-scene LCG state, seeded from clock_ns() at init and
   * re-seeded (XOR'd) on 'r'. Used by every randomness consumer:
   * spawn (column jitter, length, spacing) and splash emission
   * (kick directions, life). No globals — full state in this byte. */
  uint32_t rng;

  /* CACHED TERMINAL DIMENSIONS — read every frame by spawn (top
   * edge x range) and by the integrator (bottom-row kill check).
   * Cached at init and on SIGWINCH so the hot path never calls
   * getmaxyx(). */
  int rows, cols;

  /* PARTICLE POOLS — fixed-size BSS arrays, no allocation after
   * init. drops[] holds in-flight rain; splashes[] holds the burst
   * fragments spawned at each drop's bottom impact. Both pools use
   * the `active` flag as the source-of-truth for occupancy —
   * spawn linearly scans for the first inactive slot. */
  Drop drops[MAX_DROPS];
  Splash splashes[MAX_SPLASHES];

  /* ──────────────────────────────────────────────────────────────
   *  RENDER HALF — scene_draw reads this; physics tick ignores it
   * ────────────────────────────────────────────────────────────── */

  /* THEME — index into themes[]. Cycled by t / T. Selects the
   * colour-pair palette used by the 8-step trail ramp + splash
   * colour. Pure render concern — drops behave identically (same
   * count, motion, lifetime) regardless of which theme is active. */
  int current_theme;
} Scene;

static void scene_clear_pools(Scene *s) {
  for (int i = 0; i < MAX_DROPS; i++)
    s->drops[i].active = false;
  for (int i = 0; i < MAX_SPLASHES; i++)
    s->splashes[i].active = false;
}

static int drop_pool_find_inactive(Scene *s) {
  for (int i = 0; i < MAX_DROPS; i++)
    if (!s->drops[i].active)
      return i;
  return -1;
}

static int splash_pool_find_inactive(Scene *s) {
  for (int i = 0; i < MAX_SPLASHES; i++)
    if (!s->splashes[i].active)
      return i;
  return -1;
}

/* ── Spawn helpers (one-particle initial conditions) ─────────────── */

/* Wind-aware spawn x: extend the sampling range on the windward edge
 * so wind-tilted streams enter from the SIDE, not just from directly
 * above every column. Without this, strong wind makes the top edge
 * "leak" rain only along the leading half.
 *
 *   wind >  0.5  → x ∈ [-over, cols)         (rightward wind, leak from left)
 *   wind < -0.5  → x ∈ [0,    cols+over)     (leftward wind, leak from right)
 *   else         → x ∈ [0,    cols)          (still air, uniform)
 *
 * `over = |wind| · 0.5` scales the overhang with wind strength. */
static inline float drop_spawn_x_wind_aware(uint32_t *rng, float wind,
                                            int cols) {
  float rngx = lcg_unit(rng);
  float over = fabsf(wind) * 0.5f;
  if (wind > 0.5f)
    return rngx * ((float)cols + over) - over;
  if (wind < -0.5f)
    return rngx * ((float)cols + over);
  return rngx * (float)cols;
}

/* Apply per-drop birth velocity. The pattern provides the AVERAGE
 *   (vx = wind, vy = drop_speed)
 * but each drop gets ±DROP_SPEED_VARIANCE/2 multiplicative jitter on
 * vy and ±DROP_WIND_JITTER additive jitter on vx — this breaks the
 * "layered marching band" look that comes from every drop having
 * identical speed. */
static inline void drop_apply_birth_velocity(Drop *d, uint32_t *rng,
                                             float drop_speed, float wind) {
  float speed_jitter =
      (1.0f - DROP_SPEED_VARIANCE * 0.5f) + lcg_unit(rng) * DROP_SPEED_VARIANCE;
  float wind_jitter = (lcg_unit(rng) - 0.5f) * 2.0f * DROP_WIND_JITTER;
  d->vx = wind + wind_jitter;
  d->vy = drop_speed * speed_jitter;
}

/* Apply per-drop trail-render parameters. length sampled from the
 * pattern's [length_min, length_max]; spacing from the global
 * [TRAIL_SPACING_MIN, TRAIL_SPACING_MAX]. Per-drop variation makes
 * the curtain of rain look organic instead of regimented. */
static inline void drop_apply_birth_trail(Drop *d, uint32_t *rng,
                                          const PatternParams *pp) {
  d->length =
      pp->length_min + lcg_unit(rng) * (pp->length_max - pp->length_min);
  d->spacing = TRAIL_SPACING_MIN +
               lcg_unit(rng) * (TRAIL_SPACING_MAX - TRAIL_SPACING_MIN);
}

/* ── Driver — spawn one drop at a free pool slot ─────────────────── */

/* Pseudocode:
 *   find an inactive pool slot                  (linear scan)
 *   compute wind from pattern + scene override
 *   x = wind-aware spawn x
 *   y = uniform sample in [y_min, y_max]
 *   apply birth velocity (jittered around pattern average)
 *   apply birth trail params (length + spacing)
 *   flip alive
 *
 * The (y_min, y_max) range lets one function serve two callers:
 *   normal top-edge spawn   uses (-6, -2)        → drops enter from above
 *   prewarm at init/reseed  uses (-6, rows-2)    → scatter across the
 *                                                  whole column so the first
 *                                                  frame already looks full */
static void scene_spawn_drop(Scene *s, float y_min, float y_max) {
  int idx = drop_pool_find_inactive(s);
  if (idx < 0)
    return;
  Drop *d = &s->drops[idx];

  const PatternParams *pp = &pattern_params[s->current_pattern];
  float wind = pp->wind_x + s->wind_override;

  d->x = drop_spawn_x_wind_aware(&s->rng, wind, s->cols);
  d->y = y_min + lcg_unit(&s->rng) * (y_max - y_min);
  drop_apply_birth_velocity(d, &s->rng, pp->drop_speed, wind);
  drop_apply_birth_trail(d, &s->rng, pp);
  d->active = true;
}

/*
 * scene_prewarm — fill the drop pool to the pattern's target with
 * drops scattered UNIFORMLY across the entire visible y range. Called
 * on init, on reseed, and on pattern change. Without this, drops
 * spawn from the top edge only and the first second of rain looks
 * like a marching-band wave instead of a proper full-screen downpour.
 */
static void scene_prewarm(Scene *s) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  int target = pp->target_drops;
  if (target > MAX_DROPS)
    target = MAX_DROPS;

  /* Count current active so we only top up — pattern up-shifts
   * (DRIZZLE → STORM) keep the existing drops and just add more. */
  int active = 0;
  for (int i = 0; i < MAX_DROPS; i++)
    if (s->drops[i].active)
      active++;

  float y_max = (float)(s->rows - 2);
  for (int k = active; k < target; k++)
    scene_spawn_drop(s, -6.0f, y_max);
}

/*
 * scene_emit_splashes — called when a drop hits the bottom. Spawns
 * SPLASH_BASE_PER_DROP × pattern.splash_mul small particles at the
 * impact point with random outward velocity (small upward + ±x kick).
 */
static void scene_emit_splashes(Scene *s, float impact_x, float impact_y) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  int n = (int)((float)SPLASH_BASE_PER_DROP * pp->splash_mul + 0.5f);
  for (int k = 0; k < n; k++) {
    int idx = splash_pool_find_inactive(s);
    if (idx < 0)
      return;
    Splash *sp = &s->splashes[idx];

    float r1 = lcg_unit(&s->rng);
    float r2 = lcg_unit(&s->rng);
    float r3 = lcg_unit(&s->rng);

    sp->x = impact_x;
    sp->y = impact_y;
    sp->vx = SPLASH_KICK_X * (r1 * 2.0f - 1.0f);
    sp->vy = -SPLASH_KICK_UP * (0.6f + r2 * 0.4f);
    sp->age = 0.0f;
    sp->life = SPLASH_LIFE_MIN + r3 * (SPLASH_LIFE_MAX - SPLASH_LIFE_MIN);
    sp->active = true;
  }
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->current_theme = 0;
  s->current_pattern = PATTERN_SHOWER;
  s->wind_override = 0.0f;
  s->rng = (uint32_t)clock_ns();
  s->cols = cols;
  s->rows = rows;
  scene_clear_pools(s);
  scene_prewarm(s); /* fill the screen with rain immediately */
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
  /* Keep existing pools — they self-correct via target_drops. */
}

static void scene_reseed(Scene *s) {
  s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
  s->wind_override = 0.0f;
  scene_clear_pools(s);
  scene_prewarm(s); /* re-fill screen with new RNG seed */
}

/* ── Tick step helpers ───────────────────────────────────────────── */

/* Count occupied drop slots — feedback signal for the emission-rate
 * controller in drops_emit_to_target. */
static int drops_count_active(const Scene *s) {
  int n = 0;
  for (int i = 0; i < MAX_DROPS; i++)
    if (s->drops[i].active)
      n++;
  return n;
}

/* Reeves emission step — refill the pool toward pattern.target_drops,
 * capped at (target × dt × 4 + 4) per tick. The cap is proportional
 * to dt so a long pause doesn't dump a flood when the loop resumes;
 * +4 keeps the spawn alive at small dt. */
static void drops_emit_to_target(Scene *s, float dt) {
  const PatternParams *pp = &pattern_params[s->current_pattern];
  int active = drops_count_active(s);
  int target = pp->target_drops;
  if (target > MAX_DROPS)
    target = MAX_DROPS;
  int spawn_cap = (int)((float)pp->target_drops * dt * 4.0f) + 4;
  int to_spawn = target - active;
  if (to_spawn > spawn_cap)
    to_spawn = spawn_cap;
  for (int k = 0; k < to_spawn; k++)
    scene_spawn_drop(s, -6.0f, -2.0f);
}

/* One drop step under the terminal-velocity model: pure advection
 *     x ← x + vx · dt
 *     y ← y + vy · dt
 * No acceleration — terminal-velocity assumption (Garg & Nayar). */
static inline void drop_step_constant_velocity(Drop *d, float dt) {
  d->x += d->vx * dt;
  d->y += d->vy * dt;
}

/* Advance every drop one step and reap those that died this frame.
 * Death causes:
 *   - drifted off screen sideways (off-domain cull, ±8 cells slack)
 *   - reached the bottom row       (impact event → emit splashes) */
static void drops_integrate_and_cull(Scene *s, float dt) {
  float kill_y = (float)(s->rows - 2);
  for (int i = 0; i < MAX_DROPS; i++) {
    Drop *d = &s->drops[i];
    if (!d->active)
      continue;

    drop_step_constant_velocity(d, dt);

    if (d->x < -8.0f || d->x > (float)(s->cols + 8)) {
      d->active = false;
      continue;
    }
    if (d->y >= kill_y) {
      scene_emit_splashes(s, d->x, kill_y);
      d->active = false;
    }
  }
}

/* One splash step under gravity with linear drag:
 *     vy ← vy + g · dt              (constant gravity)
 *     vx ← vx · exp(-k · dt)         (closed-form decay over dt)
 *     (x, y) ← (x + vx · dt, y + vy · dt)
 *     age ← age + dt
 * The drag factor is precomputed once per tick by the caller. */
static inline void splash_step_kinematic(Splash *sp, float drag_factor,
                                         float dt) {
  sp->vy += SPLASH_GRAVITY * dt;
  sp->vx *= drag_factor;
  sp->x += sp->vx * dt;
  sp->y += sp->vy * dt;
  sp->age += dt;
}

/* Advance every splash one step and deactivate those past their life
 * or that fell below the ground row. */
static void splashes_integrate_and_cull(Scene *s, float dt) {
  float kill_y = (float)(s->rows - 2);
  float drag_factor = expf(-SPLASH_DRAG * dt);

  for (int i = 0; i < MAX_SPLASHES; i++) {
    Splash *sp = &s->splashes[i];
    if (!sp->active)
      continue;
    splash_step_kinematic(sp, drag_factor, dt);
    if (sp->age >= sp->life || sp->y > kill_y + 1.0f)
      sp->active = false;
  }
}

/* ── Driver — one simulation tick ────────────────────────────────── */

/* Pseudocode:
 *   if paused                    → no-op
 *   dt *= speed multiplier
 *   drops_emit_to_target         — Reeves emission with per-tick cap
 *   drops_integrate_and_cull     — advect drops, splash on impact
 *   splashes_integrate_and_cull  — gravity + drag + life expiry */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  dt *= (float)s->speed / (float)SPEED_DEF;

  drops_emit_to_target(s, dt);
  drops_integrate_and_cull(s, dt);
  splashes_integrate_and_cull(s, dt);
}

/* ── Per-particle render helpers ─────────────────────────────────── */

/* Three-tier emphasis on the 8-step ramp: ends get A_BOLD / A_DIM,
 * middle stays A_NORMAL — pushes contrast past what 8 palette
 * entries alone can give. */
static inline int ramp_slot_attr(int slot) {
  if (slot >= 6)
    return A_BOLD;
  if (slot <= 1)
    return A_DIM;
  return A_NORMAL;
}

/* Reverse-direction unit vector — points BACKWARDS along the drop's
 * velocity, which is the direction the motion-blur trail extends.
 * Returns false if the drop has no velocity (degenerate). */
static inline bool drop_trail_unit_back(const Drop *d, float *out_ux,
                                        float *out_uy) {
  float vlen = sqrtf(d->vx * d->vx + d->vy * d->vy);
  if (vlen < 1e-3f)
    return false;
  *out_ux = -d->vx / vlen;
  *out_uy = -d->vy / vlen;
  return true;
}

/* Render one drop and its tail-to-head motion-blur trail.
 *   t = 0  → head glyph (from drop_glyph_for_slope) at ramp slot 7
 *   t > 0  → trail glyph from RAMP_GLYPHS[slot] at decreasing slots
 * The trail walks BACKWARDS along the velocity at `spacing` steps. */
static void drop_render_with_trail(const Drop *d, int cols, int rows) {
  float ux, uy;
  if (!drop_trail_unit_back(d, &ux, &uy))
    return;

  char head_glyph = drop_glyph_for_slope(d->vx, d->vy);
  int trail_n = (int)d->length;
  if (trail_n < 1)
    trail_n = 1;
  if (trail_n > 7)
    trail_n = 7; /* cap at ramp depth */

  for (int t = 0; t <= trail_n; t++) {
    int ix = (int)(d->x + ux * d->spacing * (float)t + 0.5f);
    int iy = (int)(d->y + uy * d->spacing * (float)t + 0.5f);
    if (ix < 0 || ix >= cols)
      continue;
    if (iy < 0 || iy >= rows - 1)
      continue;

    int ramp_slot = 7 - t;
    if (ramp_slot < 0)
      ramp_slot = 0;

    char glyph = (t == 0) ? head_glyph : RAMP_GLYPHS[ramp_slot];
    int attr = ramp_slot_attr(ramp_slot);
    int pair = PAIR_RAIN_BASE + ramp_slot;

    attron(COLOR_PAIR(pair) | attr);
    mvaddch(iy, ix, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
  }
}

/* Map a splash's age/life ratio ∈ [0, 1] to a (glyph, attr) pair
 * — three-tier fade matching what the eye reads as bright→fading:
 *     [0.00, 0.30) → '*' A_BOLD    (fresh, brightest)
 *     [0.30, 0.65) → '+' A_NORMAL  (mid-life)
 *     [0.65, 1.00] → '.' A_DIM     (dying) */
static inline void splash_life_phase(float life_ratio, char *out_g,
                                     int *out_attr) {
  if (life_ratio < 0.30f) {
    *out_g = '*';
    *out_attr = A_BOLD;
  } else if (life_ratio < 0.65f) {
    *out_g = '+';
    *out_attr = A_NORMAL;
  } else {
    *out_g = '.';
    *out_attr = A_DIM;
  }
}

/* Render one splash at its current cell. */
static void splash_render(const Splash *sp, int cols, int rows) {
  int ix = (int)(sp->x + 0.5f);
  int iy = (int)(sp->y + 0.5f);
  if (ix < 0 || ix >= cols)
    return;
  if (iy < 0 || iy >= rows - 1)
    return;

  char glyph;
  int attr;
  splash_life_phase(sp->age / sp->life, &glyph, &attr);

  attron(COLOR_PAIR(PAIR_SPLASH) | attr);
  mvaddch(iy, ix, (chtype)(unsigned char)glyph);
  attroff(COLOR_PAIR(PAIR_SPLASH) | attr);
}

/* Loop the drop layer. Each active drop renders as head + N-cell trail. */
static void drops_render(const Scene *s) {
  for (int i = 0; i < MAX_DROPS; i++) {
    const Drop *d = &s->drops[i];
    if (!d->active)
      continue;
    drop_render_with_trail(d, s->cols, s->rows);
  }
}

/* Loop the splash layer. Each active splash renders as one age-faded glyph. */
static void splashes_render(const Scene *s) {
  for (int i = 0; i < MAX_SPLASHES; i++) {
    const Splash *sp = &s->splashes[i];
    if (!sp->active)
      continue;
    splash_render(sp, s->cols, s->rows);
  }
}

/* ── Driver — render all active particles to ncurses ─────────────── */

/* Pseudocode:
 *   drops_render     — drops with backward-extending motion-blur trails
 *   splashes_render  — splash fragments with age-based glyph + colour */
static void scene_draw(const Scene *s) {
  drops_render(s);
  splashes_render(s);
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

/* Count active drops/splashes for the HUD. */
static void scene_counts(const Scene *s, int *out_drops, int *out_spl) {
  int d = 0, p = 0;
  for (int i = 0; i < MAX_DROPS; i++)
    if (s->drops[i].active)
      d++;
  for (int i = 0; i < MAX_SPLASHES; i++)
    if (s->splashes[i].active)
      p++;
  *out_drops = d;
  *out_spl = p;
}

/* ── HUD-bar helpers ─────────────────────────────────────────────── */

/* Top row: dynamic status line. Pre-fills the row with PAIR_HUD as a
 * background, then overlays the formatted status string. Builds the
 * status text from live Scene state (pattern, theme, counts, wind,
 * speed multiplier) plus the loop's fps / sim_fps measurements. */
static void screen_paint_status_bar(Screen *sc, const Scene *s, double fps,
                                    int sim_fps) {
  int drops, spls;
  scene_counts(s, &drops, &spls);
  const PatternParams *pp = &pattern_params[s->current_pattern];
  float wind = pp->wind_x + s->wind_override;
  const char *state_str =
      s->paused ? "PAUSED " : pattern_name(s->current_pattern);

  char status[200];
  snprintf(status, sizeof status,
           " RAIN   %s   theme:%-8s   drops:%4d  splashes:%3d   "
           "wind:%+5.1f c/s   %5.1f fps  %3d Hz  speed:%-3d ",
           state_str, themes[s->current_theme].name, drops, spls, (double)wind,
           fps, sim_fps, s->speed);

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  for (int x = 0; x < sc->cols; x++)
    mvaddch(0, x, ' ');
  mvprintw(0, 0, "%s", status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Bottom row: static key-hint line. Same pre-fill trick with
 * PAIR_HINT for the coloured background bar. */
static void screen_paint_hint_bar(Screen *sc) {
  const char *hints = " q:quit  spc:pause  r:reseed  n/p:pattern  t/T:theme  "
                      "w/W:wind  +/-:speed  ]/[:Hz ";

  int hint_row = sc->rows - 1;
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  for (int x = 0; x < sc->cols; x++)
    mvaddch(hint_row, x, ' ');
  mvprintw(hint_row, 0, "%s", hints);
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── Driver — render scene, then paint two-layer HUD over it ─────── */

/*
 * screen_draw — render the scene, then paint a two-layer HUD over it:
 *
 *   Row 0          STATUS LINE.  Bright yellow PAIR_HUD + A_BOLD.
 *   Row rows-1     KEY HINT LINE.  Bright cyan PAIR_HINT + A_BOLD.
 *
 * Both rows are pre-filled with their pair colour so the coloured
 * background spans the full width, and drawn AFTER scene_draw so
 * drops never bleed through the bars.
 */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(s);
  screen_paint_status_bar(sc, s, fps, sim_fps);
  screen_paint_hint_bar(sc);
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
    scene_prewarm(s); /* top up to new pattern's target */
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
