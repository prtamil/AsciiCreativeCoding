/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * particle_systems/particle_engine.c — 2-D Terminal Particle Simulation Engine
 *
 * Reusable foundation for fluid, smoke, fire, gravity, sand, and explosion
 * effects.  Every subsystem lives in its own numbered section and is written
 * to be extracted into a separate compilation unit as the project grows.
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────────
 *  §1  config    — all tunable constants in one place
 *  §2  clock     — monotonic nanosecond clock + sleep
 *  §3  theme     — color palettes (ramps + theme-independent HUD pairs)
 *  §4  structs   — Particle and Emitter data structures
 *  §5  pool      — static particle pool, O(1)-amortised slot allocator
 *  §6  forces    — gravity, drag; force-accumulation pattern
 *  §7  physics   — symplectic Euler integrator (update_particles)
 *                  + per-wall collision response (handle_collisions)
 *  §8  emitter   — continuous emission and burst spawning
 *  §9  render    — DensityField + four render modes + stats overlay
 *  §10 presets   — fountain, fireworks, rainfall, explosion
 *  §11 scene     — Scene struct (SIMULATION + RENDER halves), tick + draw
 *  §12 screen    — ncurses display + two-layer HUD (status bar + key hints)
 *  §13 app       — signals, resize, input, main loop
 * ─────────────────────────────────────────────────────────────────────────
 *
 * Keys:
 *   q / ESC    quit                 b     spawn burst
 *   space      pause / resume       e     toggle emitter
 *   g / G      gravity ±            d     toggle drag
 *   r          reset simulation     p/P   cycle presets
 *   t          cycle themes         v     cycle render modes
 *   ] / [      sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/particle_engine.c \
 *       -o particle_engine -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Particle integration (§7):
 *   Symplectic (semi-implicit) Euler integration is used:
 *     vel += accel * dt        ← velocity updated FIRST
 *     pos += vel   * dt        ← position updated with NEW velocity
 *   Unlike explicit Euler (pos updated with old vel), symplectic Euler is
 *   energy-conserving for harmonic oscillators and stays bounded for
 *   gravity wells.  For visual particle effects the difference is subtle,
 *   but the habit is important for any physics-accurate simulation.
 *
 * Force accumulation (§6):
 *   At the start of each tick, acceleration is cleared to zero.
 *   Each force function then ADDS its contribution to accel.
 *   This keeps forces completely modular: commenting out one function
 *   disables that force without touching anything else.
 *   F = ma → a = F/m.  Here 'density' plays the role of mass-per-volume.
 *   A denser particle (density > 1) responds less to the same force.
 *
 * Velocity update — drag (§7):
 *   Drag is NOT added as an acceleration; instead velocity is scaled:
 *     vx *= 1 - drag_coeff * density * dt
 *   This is the analytical solution of dv/dt = -k*v over interval dt.
 *   It never overshoots zero (no oscillation at large timesteps), whereas
 *   adding drag as an acceleration a = -k*v can flip sign if k*dt > 2.
 *
 * Boundary handling (§7):
 *   Three policies: BOUNCE (reflect + energy damping), KILL (particle dies),
 *   WRAP (teleport to opposite wall).  Each wall can have its own policy.
 *   Bounce damping < 1.0 removes energy on impact (inelastic collision).
 *   Kill policy is used for rain (drops vanish at floor) and sparks.
 *
 * Visual encoding (§9):
 *   Speed   → char density:  fast ↔ dense glyph ('O','@'), slow ↔ sparse ('.')
 *   Life    → brightness:    A_DIM when lifetime fraction < 0.3
 *   Angle   → arrow glyph:   8-way '>\/^<\/v/' encodes flow direction
 *   Trail   → spatial path:  ring buffer of past cell positions shows
 * trajectory Heat    → density grid:  Gaussian splat accumulates per-cell
 * brightness
 *
 * References
 * ──────────
 *   PAPERS
 *     Reeves, W. T. (1983)
 *       "Particle Systems — A Technique for Modeling a Class of Fuzzy Objects"
 *       ACM Transactions on Graphics 2(2): 91-108.
 *       Foundational paper.  The pool + cone-emitter + per-particle
 *       integration loop in §5..§8 is exactly the model Reeves
 *       defines, including the rate-accumulator emission with
 *       fractional carry that we use to keep birth counts stable
 *       across frames.  §4 of the paper covers presets like fire
 *       and fireworks — the same parameter classes we expose in §10.
 *
 *     Witkin, A. & Baraff, D. (2001)
 *       "Physically Based Modeling: Principles and Practice"
 *       SIGGRAPH course notes (online proceedings).
 *       §1 — particle dynamics with explicit force accumulation
 *       (clear accel, add each force, integrate) is exactly the
 *       pattern §6 + §7 use.  §2 covers numerical integrators,
 *       including the symplectic-vs-explicit Euler distinction this
 *       file's §7 takes a stance on.
 *
 *   BOOKS
 *     Hairer, E., Lubich, C. & Wanner, G. — "Geometric Numerical
 *       Integration: Structure-Preserving Algorithms for Ordinary
 *       Differential Equations" (2nd ed, Springer, 2006).
 *       Authoritative reference on symplectic integrators.  Explains
 *       WHY swapping the order of v += a·dt and x += v·dt (i.e.
 *       using explicit instead of symplectic Euler) causes energy
 *       to drift over long simulations — visible as "ghost rises"
 *       in a fountain run for thousands of ticks.
 *
 *     Bourg, D. M. & Bywalec, B. — "Physics for Game Developers"
 *       (2nd ed, O'Reilly, 2013).  Chapters 2-4 cover ballistic
 *       motion under gravity, restitution at bounce boundaries, and
 *       linear-velocity drag.  Direct support for §6 (gravity/drag),
 *       §7 (collisions), and the preset values chosen in §10.
 *
 *     Akenine-Möller, T., Haines, E. & Hoffman, N. — "Real-Time
 *       Rendering" (4th ed, CRC Press, 2018).
 *       §13.7 covers point-sprite particle rendering — the speed-
 *       to-glyph-density mapping in §9 is the ASCII analogue of
 *       size-by-velocity point sprites.  §5.6 motivates the gamma
 *       correction used in the LUT ramp index (^1/2.2 in KEY FORMULAS).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A particle system is just an object pool plus three independent layers:
 * (1) where particles are BORN (emitter), (2) what FORCES act on them
 * (gravity/drag), (3) what happens at the EDGE (bounce/kill/wrap). Every
 * preset — fountain, fireworks, rainfall, explosion — is just different
 * values for these three knobs over the same fixed-step integrator. Master
 * this single file and you can build any 2-D particle effect.
 *
 * ALGORITHM IN STEPS  (each step = one helper in §5..§11)
 * ──────────────────
 *  Initial setup (per scene_init):
 *    1. Pool: 2048 Particle slots in BSS. pool_alloc() linear-scans
 *       from a cursor for the first !alive slot; returns NULL when
 *       full (spawn silently dropped). No malloc after init.
 *
 *  Per tick (scene_tick at fixed dt = 1/sim_fps):
 *    2. scene_advance_clock  — bump dt_sec + simulation_time.
 *    3. scene_spawn_step:
 *         emitter_tick: rate_accum += rate·dt; spawn ⌊accum⌋ particles
 *                       into the cone (angle ± spread/2). Carry the
 *                       fractional remainder for exact long-term rate.
 *         auto-burst:   countdown burst_timer; on cross-zero fire
 *                       spawn_burst(burst_count).
 *    4. scene_physics_step → update_particles, per alive particle:
 *         apply_forces                   → clear (ax,ay), add gravity
 *         particle_apply_drag            → v *= (1 − k·ρ·dt) closed-form
 *         particle_integrate_symplectic  → v += a·dt; x += v·dt
 *         particle_age                   → lifetime -= dt; kill at 0
 *         particle_push_trail            → record cell to ring buffer
 *         accumulate Σ|v| and Σ½ρv²
 *    5. scene_physics_step → handle_collisions, per alive particle,
 *       short-circuit on kill:
 *         particle_resolve_floor / ceiling / left_wall / right_wall
 *       each KILL or BOUNCE (reflect + BOUNCE_DAMPING restitution).
 *    6. scene_measure_stats — recount alive for the HUD.
 *
 *  Per frame (scene_draw → render_particles):
 *    7. Render dispatch by mode:
 *       GLYPH / TRAIL / ARROW → render_per_particle, per particle:
 *            particle_ramp_level   speed → ramp index 1..RAMP_N-1
 *            particle_render_attr  theme colour + A_DIM if dying
 *            particle_draw_trail   (TRAIL only) ring-buffer walk
 *            particle_draw_head    glyph at current cell
 *       HEATMAP → render_heatmap, three passes over DensityField:
 *            heatmap_clear_emptied_cells   dirty-rect blank
 *            heatmap_accumulate_density    density_field_splat per particle
 *            heatmap_blit_density_field    density → ramp glyph
 *    8. screen_draw paints the two-layer HUD (top status, bottom keys)
 *       and the render_overlay panel on top of the particle layer.
 *
 * KEY FORMULAS
 * ────────────
 *  v += a·dt; x += v·dt           symplectic Euler (v BEFORE x)
 *  v *= 1 − k·ρ·dt                drag: closed-form of dv/dt = -k·v
 *  bounce:  v ← −v · DAMPING      1 − DAMPING² fraction of KE lost
 *  rate accumulator: spawn ⌊accum⌋ each tick, keep frac for fairness
 *  pixel↔cell: cx = ⌊px/CELL_W + 0.5⌋ where CELL_W=8, CELL_H=16
 *  ramp_idx = first i where v^(1/2.2) ≥ k_lut_breaks[i]    gamma-corrected
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/*
 * All magic numbers live here.  Changing behaviour means editing this
 * block only — never scatter literals through implementation code.
 */

/* ── loop / display ─────────────────────────────────────────────────── */
enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,

  TARGET_FPS = 60,     /* render cap                               */
  FPS_UPDATE_MS = 500, /* recalculate displayed fps every 500 ms   */
  HUD_COLS = 72,       /* max width of any HUD string              */
};

/* ── particle pool ──────────────────────────────────────────────────── */
enum {
  MAX_PARTICLES = 2048, /* hard pool ceiling — never reallocated    */
  TRAIL_LEN = 6,        /* positions kept in each particle's trail  */
  MAX_BURST = 300,      /* hard cap: particles per single burst     */
};

/* ── physics ────────────────────────────────────────────────────────── */
/*
 * All physics values are in PIXEL SPACE (see §4 / CELL_W, CELL_H).
 * Typical terminal is ~200 px wide × 800 px tall in pixel space.
 */
#define GRAVITY_DEFAULT 200.0f /* px / s²  downward                 */
#define GRAVITY_STEP 30.0f     /* change per 'g'/'G' keypress        */
#define GRAVITY_MIN 0.0f
#define GRAVITY_MAX 600.0f
#define DRAG_COEFF 0.80f      /* velocity fraction removed per s    */
#define BOUNCE_DAMPING 0.45f  /* kinetic energy fraction kept per bounce */
#define MAX_SPEED_NORM 650.0f /* px/s mapped to highest ramp level  */

/* ── density grid (heatmap mode) ────────────────────────────────────── */
/*
 * Static grid sized for the largest terminal we expect to support.
 * 300 × 100 × 4 bytes = 120 KB in BSS — no heap needed.
 */
#define GRID_MAX_W 300
#define GRID_MAX_H 100

/* ── coordinate system ──────────────────────────────────────────────── */
/*
 * Terminal cells are ~2× taller than wide in physical pixels.
 * Physics runs in PIXEL SPACE; rendering converts to CELL SPACE once.
 * See §12 for the coordinate system explanation from framework.c.
 */
#define CELL_W 8  /* logical sub-pixel steps per column */
#define CELL_H 16 /* logical sub-pixel steps per row    */

static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }
static inline int px_to_cx(float px) {
  return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cy(float py) {
  return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── render modes ───────────────────────────────────────────────────── */
enum {
  RENDER_GLYPH = 0,   /* char density ∝ speed, brightness ∝ life  */
  RENDER_TRAIL = 1,   /* glyph + dim spatial history               */
  RENDER_HEATMAP = 2, /* Gaussian-splatted density grid + LUT      */
  RENDER_ARROW = 3,   /* 8-way direction glyph per velocity angle  */
  RENDER_COUNT = 4,
};

static const char *const k_render_names[RENDER_COUNT] = {"glyph", "trail",
                                                         "heatmap", "arrow"};

/* ── presets ────────────────────────────────────────────────────────── */
enum {
  PRESET_FOUNTAIN = 0,
  PRESET_FIREWORKS = 1,
  PRESET_RAINFALL = 2,
  PRESET_EXPLOSION = 3,
  PRESET_COUNT = 4,
};

static const char *const k_preset_names[PRESET_COUNT] = {
    "fountain", "fireworks", "rainfall", "explosion"};

/* ── timing ─────────────────────────────────────────────────────────── */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

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
  struct timespec r = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  theme — color palettes, ASCII ramps, LUT pipeline                 */
/* ===================================================================== */

/*
 * ASCII ramp — characters ordered by visual ink density (sparse → dense).
 * These are the 9 display levels the rendering pipeline quantises to.
 * Chosen to match ncurses character-cell perception on dark terminals.
 *
 *   ' '  0%   cold / empty
 *   '.'  8%   trace / faint
 *   ':'  18%  low intensity
 *   '+'  29%  medium-low
 *   'x'  39%  medium
 *   '*'  50%  medium-high
 *   'X'  62%  high
 *   '#'  75%  very high
 *   '@'  90%  peak / core
 */
static const char k_ramp[] = " .:+x*X#@";
#define RAMP_N (int)(sizeof k_ramp - 1) /* 9 levels */

/*
 * LUT breakpoints — gamma-corrected thresholds for each ramp level.
 * Gamma correction: raw_value → pow(v, 1/2.2) before lookup.
 * This maps physics-linear values to perceptually-uniform brightness
 * so mid-intensity particles aren't all the same dim character.
 */
static const float k_lut_breaks[RAMP_N] = {
    0.000f, 0.080f, 0.180f, 0.290f, 0.390f, 0.500f, 0.620f, 0.750f, 0.900f,
};

static int lut_index(float v) {
  /* Clamp, gamma-correct, then binary-scan for the highest level ≤ v */
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

/*
 * Themes — four palettes, each mapping RAMP_N levels to colors.
 * We define all four simultaneously so switching themes at runtime
 * costs nothing (no init_pair calls, just change the color-pair base).
 *
 *   theme 0  fire    — red / orange / yellow   (fireworks, explosion)
 *   theme 1  ocean   — blue / cyan / white      (fountain, rainfall)
 *   theme 2  nature  — green / lime             (general, organic)
 *   theme 3  cosmic  — violet / magenta / white (cosmic, energy)
 *
 * Color pair layout:
 *   pairs [ CP_BASE + theme*RAMP_N .. CP_BASE + theme*RAMP_N + RAMP_N-1 ]
 *   Max pair id = CP_BASE + 4*9 - 1 = 36.  ncurses supports ≥ 256.
 */
#define CP_BASE                                                                \
  1 /* first theme/ramp pair id we own (1..36 = 4 themes × 9 levels) */
#define PAIR_HUD                                                               \
  40 /* full-width top status bar — bright yellow, theme-independent */
#define PAIR_HINT                                                              \
  41 /* full-width bottom key-hint bar — bright cyan, theme-independent */
#define N_THEMES 4

typedef struct {
  const char *name;
  int fg256[RAMP_N];    /* xterm-256 foreground per level               */
  int fg8[RAMP_N];      /* 8-color fallback                             */
  attr_t attr8[RAMP_N]; /* A_BOLD / A_DIM / A_NORMAL per level (8-clr)  */
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
  /* Theme-independent HUD bars — bright high-contrast colours per
   * CLAUDE.md "HUD Standard" so they stay legible against any theme. */
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, COLOR_BLACK); /* bright yellow */
    init_pair(PAIR_HINT, 51, COLOR_BLACK); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, COLOR_BLACK);
    init_pair(PAIR_HINT, COLOR_CYAN, COLOR_BLACK);
  }
}

/* Return ncurses attribute for (theme, ramp_level). */
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

/* ===================================================================== */
/* §4  structs — Particle and Emitter                                     */
/* ===================================================================== */

/*
 * Particle — one element in the static pool.
 *
 * INTENT: hold every piece of per-entity state the physics tick (§7)
 * and render pass (§9) need, in a flat layout that fits into a single
 * fixed-size BSS array (no malloc after init). One Particle slot can
 * be in one of two states: alive (participates in tick + draw) or
 * dead (skipped; available for reuse by pool_alloc).
 *
 * LIFECYCLE: spawn_particle() initialises every field of a free slot
 * (alive flips to true); update_particles() integrates motion and
 * decrements lifetime; when lifetime ≤ 0 or a kill-boundary is hit,
 * alive flips back to false and the slot becomes available again.
 *
 * COORDINATE SPACE: position + velocity live in PIXEL space
 * (sub-cell: CELL_W=8 px/col, CELL_H=16 px/row). Only the trail and
 * the final render write convert to CELL space. Pixel precision is
 * what lets a particle move sub-cell distances per frame so motion
 * reads as smooth instead of snapping between cells.
 *
 *   x, y         : current position in pixel space. Read by render,
 *                  mutated by the integrator (symplectic Euler):
 *                  x += vx · dt, y += vy · dt.
 *   vx, vy       : velocity in px/sec. Set at spawn from emitter
 *                  cone; mutated each tick by `vx += ax·dt` (and
 *                  multiplicatively scaled by drag, if enabled).
 *                  Sign convention: vy < 0 = rising (ncurses y
 *                  increases downward), vy > 0 = falling.
 *   ax, ay       : acceleration accumulator (px/s²). CLEARED at the
 *                  top of every tick (apply_forces); each active
 *                  force (gravity, future fields…) ADDS its
 *                  contribution. Modular by design — comment out
 *                  one force function to disable it.
 *   lifetime     : seconds remaining before death. Decremented by
 *                  dt each tick. When ≤ 0 the particle dies and the
 *                  slot returns to the free pool.
 *   max_lifetime : ORIGINAL lifetime at spawn — never mutated.
 *                  Render needs it to compute the fade fraction
 *                  `lifetime / max_lifetime`: < 0.3 → A_DIM.
 *   density      : mass-like parameter ∈ [0.1, 3.0]. Two effects:
 *                  (1) force response — acceleration is divided by
 *                      density so heavier particles accelerate less.
 *                  (2) drag scaling — drag damps heavier particles
 *                      FASTER (more cross-section per unit area).
 *                  density > 1: sluggish + heavy; < 1: light, buoyant.
 *   color        : optional ramp-level override [0..RAMP_N-1].
 *                  0 = auto (render picks level from speed).
 *                  Non-zero is rarely used — reserved for presets
 *                  that want a fixed colour regardless of speed.
 *   alive        : pool-slot occupancy flag. False means this slot
 *                  is skipped by every loop AND available to
 *                  pool_alloc — same single source of truth.
 *   trail_cx[]   : ring buffer of TRAIL_LEN most recent CELL x-coords.
 *   trail_cy[]   : ring buffer of TRAIL_LEN most recent CELL y-coords.
 *                  Stored in CELL space (not pixel) so the TRAIL
 *                  render mode can blit them directly without
 *                  re-converting every frame. Updated once per tick
 *                  AFTER position integration.
 *   trail_head   : ring buffer write cursor. The newest entry is at
 *                  (trail_head - 1) mod TRAIL_LEN. trail_head wraps
 *                  forever — readers always work mod TRAIL_LEN.
 */
typedef struct {
  float x, y;
  float vx, vy;
  float ax, ay;
  float lifetime;
  float max_lifetime;
  float density;
  int color;
  bool alive;
  int trail_cx[TRAIL_LEN];
  int trail_cy[TRAIL_LEN];
  int trail_head;
} Particle;

/*
 * Emitter — the "spigot" that drips new particles into the pool.
 *
 * INTENT: encapsulate every knob needed to randomise spawn parameters
 * for a new particle. The Emitter never owns particles — it only
 * SPAWNS them; once a particle is born, the emitter has no further
 * influence on it. Switching preset rewrites every field at once
 * (see preset_apply in §10).
 *
 * EMISSION MODES (driven by Scene, not the emitter itself):
 *   - CONTINUOUS: active == true and rate > 0 → spawn at a fixed
 *                 rate every tick. Used by fountain + rainfall.
 *   - BURST:      caller invokes spawn_burst(N) at chosen moments,
 *                 emitter parameters describe ONE burst's geometry.
 *                 Used by fireworks + explosion.
 *
 * The two modes can coexist (active emitter + manual burst on 'b').
 *
 *   x, y         : emitter position in PIXEL space. Set by preset
 *                  from terminal dims (e.g. fountain → bottom-centre,
 *                  rainfall → top-edge). Mutated only on resize.
 *   angle        : mean emission direction (radians). Convention:
 *                  -π/2 = straight up (fountain), +π/2 = down
 *                  (rainfall). Per-particle angle is sampled uniformly
 *                  from [angle − spread/2, angle + spread/2].
 *   spread       : total angular cone width (radians). Fountain uses
 *                  π/3 (60°), explosion uses 2π (full radial burst).
 *   speed_min,   : birth speed range in px/sec. Per-particle initial
 *   speed_max     speed sampled uniformly between min/max. Higher =
 *                  particles reach higher apex (v²/(2g) for upward).
 *   life_min,    : lifetime range in seconds — sampled per particle
 *   life_max      at spawn into Particle.max_lifetime. Sets how long
 *                  the particle stays alive AFTER spawn.
 *   density_min, : density range — per-particle density sampled
 *   density_max   uniformly and stored in Particle.density. Affects
 *                  acceleration response and drag scaling.
 *   rate         : continuous emission rate, particles per second.
 *                  0 = no continuous emission (burst-only presets).
 *                  Read by spawn_continuous each tick.
 *   rate_accum   : FRACTIONAL particle accumulator. Each tick we add
 *                  `rate · dt` to rate_accum, spawn floor(rate_accum)
 *                  particles, then `rate_accum -= floor(rate_accum)`.
 *                  Why: rate · dt may be < 1, so naive int truncation
 *                  would never spawn anything. The fractional carry
 *                  gives an EXACT long-term average rate.
 *   spawn_width  : x-axis scatter half-width in pixels.
 *                  0     = point source (fountain, explosion).
 *                  pw(c) = whole-screen-width (rainfall fills the top
 *                  edge). Per-particle x sampled in [x − w, x + w].
 *   active       : enable/disable continuous emission entirely.
 *                  Toggled by the 'e' key. Burst mode ignores this
 *                  flag — bursts always fire when requested.
 */
typedef struct {
  float x, y;
  float angle;
  float spread;
  float speed_min;
  float speed_max;
  float life_min;
  float life_max;
  float density_min;
  float density_max;
  float rate;
  float rate_accum;
  float spawn_width;
  bool active;
} Emitter;

/* ===================================================================== */
/* §5  pool — static particle pool, no-malloc allocator                  */
/* ===================================================================== */

/*
 * g_particles[] is a BSS-segment static array — allocated at program start,
 * never reallocated.  The "no heap allocation inside the update loop"
 * requirement is satisfied because spawn_particle() only sets fields inside
 * an already-allocated slot; it never calls malloc/realloc.
 *
 * Allocation strategy: linear cursor scan.
 *   The cursor advances from its last position, wrapping around.
 *   Because birth_rate ≈ death_rate in steady state, the cursor
 *   travels an expected O(N / free_fraction) before finding a free slot —
 *   essentially O(1) amortised when the pool is < 90% full.
 *
 * When the pool is completely full, pool_alloc() returns NULL and the
 * spawn is silently dropped.  The simulation degrades gracefully rather
 * than crashing or heap-allocating.
 */
static Particle g_particles[MAX_PARTICLES];
static int g_cursor = 0; /* allocation cursor */

static void pool_clear(void) {
  memset(g_particles, 0, sizeof g_particles);
  g_cursor = 0;
}

/* Find a free slot; returns NULL if pool is full. */
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

/* Count alive particles (used for stats; called once per tick). */
static int pool_alive_count(void) {
  int n = 0;
  for (int i = 0; i < MAX_PARTICLES; i++)
    if (g_particles[i].alive)
      n++;
  return n;
}

/* ===================================================================== */
/* §6  forces — gravity, drag; force-accumulation pattern                */
/* ===================================================================== */

/*
 * apply_forces() — accumulate all active forces into particle acceleration.
 *
 * PATTERN: This function is called FIRST each physics tick, before
 * velocity and position are integrated.  It clears accel then adds each
 * force separately.  To disable a force, simply comment out its line —
 * nothing else changes.
 *
 * Forces currently modelled:
 *   1. Gravity  — constant downward acceleration.
 *      In screen space +y points down, so gravity is positive ay.
 *      Galileo's principle: gravity is independent of mass/density.
 *      (To add buoyancy for smoke: ay += gravity * (1.0 - density) )
 *
 *   2. Drag     — velocity-proportional deceleration.
 *      Applied as a velocity scale in update_particles() (not here),
 *      because scaling is the exact analytical solution of dv/dt = -k*v
 *      and never goes unstable.  See §7.
 *
 * Extending: add wind, vortices, attractors, etc. as:
 *   p->ax += wind_force_x / p->density;
 *   p->ay += wind_force_y / p->density;
 */
static void apply_forces(Particle *p, float gravity) {
  /* Clear: forces are re-evaluated fresh every tick */
  p->ax = 0.0f;
  p->ay = 0.0f;

  /* Gravity: uniform downward field.
   * Independent of density (equivalence principle). */
  p->ay += gravity;
}

/* ===================================================================== */
/* §7  physics — update_particles, handle_collisions                     */
/* ===================================================================== */

/*
 * update_particles() — advance all alive particles by one fixed timestep.
 *
 * Called from the scene_tick() accumulator loop; dt is always the fixed
 * tick duration (1 / sim_fps seconds).  Never called with variable dt.
 *
 * Per-particle steps (order matters):
 *   1. apply_forces()  — clear accel, re-accumulate forces
 *   2. Drag damping    — scale velocity before integration (exact solution)
 *   3. Symplectic Euler integration
 *                        vel += accel * dt   (velocity from NEW accel)
 *                        pos += vel   * dt   (position from NEW vel)
 *   4. Age the particle — decrease lifetime
 *   5. Update trail ring buffer
 *   6. Kill if lifetime expired
 *
 * The spawn_out / energy accumulators are for the overlay stats panel.
 */
/* ── Per-particle math (pure / no side effects) ──────────────────── */

/* Instantaneous speed |v| = √(vx² + vy²). */
static inline float particle_speed(const Particle *p) {
  return sqrtf(p->vx * p->vx + p->vy * p->vy);
}

/* Kinetic energy ½·ρ·v² — density playing the role of mass per
 * unit area in this 2-D system. Summed into Scene.energy_estimate
 * to give a rough "is the simulation conserving energy?" gauge. */
static inline float particle_kinetic_energy(const Particle *p) {
  return 0.5f * p->density * (p->vx * p->vx + p->vy * p->vy);
}

/* ── Per-particle physics (mutating) ─────────────────────────────── */

/* Multiplicative linear drag — analytic solution of dv/dt = −k·v
 * over interval dt, density-scaled:
 *     v(t+dt) = v(t) · (1 − k·ρ·dt)        (first-order expansion)
 * Floored at 0 so a huge dt cannot flip the velocity's sign — the
 * additive-acceleration form a -= k·v would explode when k·dt > 2,
 * this form is unconditionally stable. */
static inline void particle_apply_drag(Particle *p, float dt) {
  float damp = 1.0f - DRAG_COEFF * p->density * dt;
  if (damp < 0.0f)
    damp = 0.0f;
  p->vx *= damp;
  p->vy *= damp;
}

/* Symplectic (semi-implicit) Euler step:
 *     v ← v + a · dt          (velocity FIRST)
 *     x ← x + v_new · dt       (position uses the JUST-updated v)
 * The order is the difference from explicit Euler — symplectic
 * conserves energy to first order on Hamiltonian systems, so
 * long-running fountains don't drift upward over time. */
static inline void particle_integrate_symplectic(Particle *p, float dt) {
  p->vx += p->ax * dt;
  p->vy += p->ay * dt;
  p->x += p->vx * dt;
  p->y += p->vy * dt;
}

/* Decrement remaining lifetime by dt; flip `alive` false when it
 * expires. Returns true if the particle survived this tick. */
static inline bool particle_age(Particle *p, float dt) {
  p->lifetime -= dt;
  if (p->lifetime <= 0.0f) {
    p->alive = false;
    return false;
  }
  return true;
}

/* Push the particle's current CELL coordinates onto its trail ring.
 * Stored in CELL space (not pixel) so the trail renderer can blit
 * directly without re-converting every frame. trail_head wraps
 * mod TRAIL_LEN; the oldest entry is silently overwritten. */
static inline void particle_push_trail(Particle *p) {
  p->trail_cx[p->trail_head] = px_to_cx(p->x);
  p->trail_cy[p->trail_head] = px_to_cy(p->y);
  p->trail_head = (p->trail_head + 1) % TRAIL_LEN;
}

/* ── Driver — one tick over the whole pool ───────────────────────── */

/* Pseudocode per alive particle, in order:
 *   1. apply_forces            — clear accel, add gravity (+ future fields)
 *   2. particle_apply_drag      — closed-form velocity damping (if enabled)
 *   3. particle_integrate_symplectic — vel += a·dt; pos += v_new·dt
 *   4. particle_age             — lifetime -= dt; kill if expired
 *   5. particle_push_trail      — record current cell for trail render
 *   6. accumulate stats         — Σ|v| and Σ½ρv² for the HUD overlay */
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

/*
 * handle_collisions() — enforce world boundaries on all alive particles.
 *
 * World boundaries are 0 and pw(cols)-1 in x, 0 and ph(rows)-1 in y.
 * Three policies per wall:
 *   BOUNCE  — reflect the relevant velocity component and apply BOUNCE_DAMPING.
 *             Kinetic energy is reduced by (1 - BOUNCE_DAMPING²) per bounce.
 *   KILL    — mark particle dead.  Used for rainfall drops, spark embers.
 *   WRAP    — teleport to opposite side.  Used for wind-tunnel loops.
 *
 * The floor_kills / ceiling_kills / wall_kills booleans in Scene map to:
 *   true  → KILL policy
 *   false → BOUNCE policy
 * WRAP is not exposed as a preset option here but is trivial to add.
 */
/* ── Per-wall boundary response ──────────────────────────────────── *
 *
 * Each helper checks one wall. If the particle has crossed it:
 *   - kills == true → flip alive=false (return false to caller)
 *   - kills == false → snap to the wall, reflect the normal-component
 *                      velocity, multiply by BOUNCE_DAMPING (loss of
 *                      1 − BOUNCE_DAMPING² of kinetic energy per bounce
 *                      — inelastic restitution).
 *   - no cross → particle untouched, return true.
 * Caller threads the booleans so a kill on one wall short-circuits the
 * remaining wall checks for that particle.
 */

static inline bool particle_resolve_floor(Particle *p, float world_h,
                                          bool kills) {
  if (p->y < world_h)
    return true;
  if (kills) {
    p->alive = false;
    return false;
  }
  p->y = world_h;
  p->vy = -fabsf(p->vy) * BOUNCE_DAMPING; /* normal reflection upward */
  p->vx *= BOUNCE_DAMPING;                /* tangential floor friction */
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
  p->vy = fabsf(p->vy) * BOUNCE_DAMPING; /* reflect downward */
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
  p->vx = fabsf(p->vx) * BOUNCE_DAMPING; /* reflect right */
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
  p->vx = -fabsf(p->vx) * BOUNCE_DAMPING; /* reflect left */
  return true;
}

/* ── Driver — apply every wall to every alive particle ───────────── */

/* Pseudocode per particle:
 *   if killed by floor    → done
 *   if killed by ceiling  → done
 *   if killed by left     → done
 *   if killed by right    → done
 *   otherwise: continue to next particle. Non-fatal crosses bounce
 *   in-place and the particle keeps participating in remaining checks. */
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

/* ===================================================================== */
/* §8  emitter — continuous emission and burst spawning                  */
/* ===================================================================== */

/*
 * rand_float() — uniform random float in [lo, hi).
 * Uses integer rand() to avoid floating-point random state issues.
 */
static float rand_float(float lo, float hi) {
  return lo + ((float)(rand() % 100000) / 100000.0f) * (hi - lo);
}

/*
 * spawn_one() — fill a pool slot with one freshly born particle.
 *
 * Birth position is emitter.x ± spawn_width/2 (x) and emitter.y (y).
 * Birth velocity is emitter.speed in direction (angle ± spread/2).
 * Returns true if a slot was found, false if the pool was full.
 */
static bool spawn_one(const Emitter *em, int theme) {
  Particle *p = pool_alloc();
  if (!p)
    return false;

  /* Scatter x for wide emitters (e.g. rainfall across full top edge) */
  float bx =
      em->x + rand_float(-em->spawn_width * 0.5f, em->spawn_width * 0.5f);
  float by = em->y;

  /* Random emission angle within the cone */
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
  p->color = theme; /* store theme so theme-change affects live particles */
  p->alive = true;
  p->trail_head = 0;

  /* Pre-fill trail with birth position so first TRAIL_LEN frames
   * don't show trails from (0,0) interpolating to birth pos.    */
  int cx = px_to_cx(bx), cy = px_to_cy(by);
  for (int t = 0; t < TRAIL_LEN; t++) {
    p->trail_cx[t] = cx;
    p->trail_cy[t] = cy;
  }
  return true;
}

/*
 * emitter_tick() — continuous emission for one physics tick.
 *
 * Fractional accumulator pattern: we want `rate` particles per SECOND,
 * but ticks are shorter than a second.  We accumulate rate*dt each tick
 * and spawn floor(accum) particles, keeping the fractional remainder.
 * This gives an accurate average birth rate with no systematic bias.
 *
 * Returns number of particles actually spawned this tick (for stats).
 */
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

/*
 * spawn_burst() — immediately emit `count` particles.
 *
 * Used for: manual 'b' keypress, auto-burst timer, firework explosions.
 * Ignores emitter.active — bursts always fire regardless of toggle state.
 */
static int spawn_burst(const Emitter *em, int count, int theme) {
  if (count > MAX_BURST)
    count = MAX_BURST;
  int spawned = 0;
  for (int i = 0; i < count; i++)
    if (spawn_one(em, theme))
      spawned++;
  return spawned;
}

/* ===================================================================== */
/* §9  render — four render modes + overlay stats panel                  */
/* ===================================================================== */

/* ── DensityField — scalar heat grid for the heatmap render mode ───── *
 *
 * INTENT: a single named home for the two parallel density buffers
 * the heatmap mode needs, instead of two loose `g_density` /
 * `g_prev_density` globals scattered through the file. The
 * abstraction encodes the "double-buffer for dirty-cell diffing"
 * pattern that the three-pass heatmap pipeline depends on.
 *
 * SPACE: CELL space (rows × cols), not pixel. Sized to GRID_MAX_H ×
 * GRID_MAX_W so the grid is allocated once in BSS and the render path
 * never calls malloc. The bounds-check in density_field_splat clamps
 * any cell outside that hard cap.
 *
 * BUFFERS:
 *   curr : this frame's accumulated density. Zeroed at the start of
 *          each render via density_field_begin_frame, then filled by
 *          density_field_splat per alive particle. Read by the blit
 *          pass to pick glyph + colour.
 *   prev : last frame's snapshot of `curr`. Compared against `curr`
 *          to find cells that DROPPED below the visibility threshold
 *          since last frame — those cells get a single space char
 *          written (dirty-rectangle erase), keeping the terminal
 *          write count proportional to changed cells.
 *
 * LIFETIME: single static instance `g_field`. density_field_reset()
 * zeros both buffers, called at scene_init / scene_reset / preset
 * switch / render-mode switch — anywhere a stale frame would bleed
 * through into the new state.
 */
typedef struct {
  float curr[GRID_MAX_H][GRID_MAX_W];
  float prev[GRID_MAX_H][GRID_MAX_W];
} DensityField;

static DensityField g_field;

/* Zero BOTH buffers — called on scene reset, preset switch, and when
 * switching INTO heatmap mode, so the new frame starts from a blank
 * field. */
static inline void density_field_reset(void) {
  memset(g_field.curr, 0, sizeof g_field.curr);
  memset(g_field.prev, 0, sizeof g_field.prev);
}

/* Zero only `curr` — top of an accumulation pass. `prev` is left
 * intact so the next clear-emptied pass can diff against it. */
static inline void density_field_begin_frame(void) {
  memset(g_field.curr, 0, sizeof g_field.curr);
}

/* Copy curr → prev. Called after the dirty-cell clear has finished
 * reading `prev`, snapshotting the just-rendered frame so NEXT frame
 * can diff against it. */
static inline void density_field_snapshot(void) {
  memcpy(g_field.prev, g_field.curr, sizeof g_field.curr);
}

/*
 * density_field_splat() — deposit a Gaussian blob into curr[].
 *
 * 3×3 normalised kernel (Σ = 1.0):
 *   corners     : 0.0625  (1/16)
 *   edge-centres: 0.125   (2/16)
 *   centre      : 0.25    (4/16)
 *
 * Weight is whatever the caller pre-computed (typically
 * particle_heat_weight = speed × life_fraction, clamped). Cells
 * outside the visible region OR outside the hard GRID_MAX cap are
 * silently skipped — no out-of-bounds writes.
 */
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

/*
 * 8-way direction arrows for RENDER_ARROW mode.
 *
 * Octant mapping (screen space: +x right, +y DOWN):
 *   angle = atan2(vy, vx)  — gives angle in [-π, π]
 *   We map this to 8 octants (0 = right, going clockwise).
 *
 * Characters '/' and '\' appear twice because they are symmetric —
 * the same slash encodes both (right-up) and (left-down) motion.
 * This is accurate: think of '/' as "the particle is going diagonally
 * along this stroke's direction", same for '\'.
 */
static const char k_arrows[8] = {
    '>',  /* 0: right        [−π/8 ,  π/8)  */
    '\\', /* 1: right-down   [ π/8 , 3π/8)  */
    'v',  /* 2: down         [3π/8 , 5π/8)  */
    '/',  /* 3: left-down    [5π/8 , 7π/8)  */
    '<',  /* 4: left         [7π/8 ,−7π/8)  */
    '\\', /* 5: left-up      [−7π/8,−5π/8)  */
    '^',  /* 6: up           [−5π/8,−3π/8)  */
    '/',  /* 7: right-up     [−3π/8,−π/8)   */
};

static char velocity_arrow(float vx, float vy) {
  float angle = atan2f(vy, vx);     /* [-π, π]          */
  float norm = angle + (float)M_PI; /* [0, 2π)          */
  int oct = (int)(norm / (float)M_PI * 4.0f) % 8;
  return k_arrows[oct];
}

/*
 * render_heatmap() — continuous density-field view of the particle cloud.
 *
 * This mode treats all particles as sources of a scalar density field rather
 * than discrete points.  The result looks like infrared heat or smoke density.
 *
 * Algorithm (three passes):
 *
 *   PASS 1 — erase stale cells:
 *     Cells that had density > threshold last frame but are now empty need
 *     explicit blanking.  We compare g_prev_density vs g_density (before
 *     rebuilding) instead of clearing the whole screen each frame.
 *     WHY: clearing all ROWS×COLS cells each frame causes terminal flicker
 *     because ncurses still has to repaint them all.  Differential erase
 *     only touches cells that actually changed.
 *
 *   PASS 2 — accumulate density:
 *     g_density is zeroed, then every alive particle splats a 3×3 Gaussian
 *     blob weighted by (speed / MAX_SPEED_NORM) × life_fraction.
 *     WHY weight by speed: fast particles carry more kinetic energy and should
 *     appear brighter (hotter).  A stationary particle contributes nothing.
 *     WHY weight by life_frac: dying particles fade out naturally in the field
 *     without needing per-particle dim logic.
 *
 *   PASS 3 — render density grid:
 *     Normalised density → lut_index → k_ramp character + theme colour pair.
 *     Cells below 0.01 are skipped (background stays dark).
 */
/* ── Heatmap helpers — three-pass density pipeline ───────────────── */

/* Per-particle heat contribution to the density field:
 *   weight = clamp(|v| / MAX_SPEED_NORM × lifetime_fraction, 0, 1)
 * Fast YOUNG particles add the most heat; stationary particles add
 * zero; dying particles fade out automatically without per-particle
 * dim logic. The clamp keeps the splat amplitude bounded. */
static inline float particle_heat_weight(const Particle *p) {
  float lf = p->lifetime / p->max_lifetime;
  float w = (particle_speed(p) / MAX_SPEED_NORM) * lf;
  return (w > 1.0f) ? 1.0f : w;
}

/* PASS 1 — clear cells that DROPPED below the visibility threshold
 * this frame (compares last frame's snapshot vs current density,
 * emits a space at any cell that lost its occupancy), then snapshot
 * current density for next frame's diff. Dirty-rectangle trick that
 * keeps the terminal write count proportional to changed cells, not
 * total cells. */
static void heatmap_clear_emptied_cells(WINDOW *w, int cols, int rows) {
  for (int r = 0; r < rows && r < GRID_MAX_H; r++)
    for (int c = 0; c < cols && c < GRID_MAX_W; c++)
      if (g_field.prev[r][c] > 0.01f && g_field.curr[r][c] < 0.01f)
        mvwaddch(w, r, c, ' ');
  density_field_snapshot();
}

/* PASS 2 — accumulate the density field from scratch. Each alive
 * particle splats a 3×3 Gaussian (handled by splat_density) weighted
 * by particle_heat_weight. Re-build, don't increment, so dead-now
 * particles don't leave ghosts. */
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

/* PASS 3 — map density value → ramp level → glyph + theme colour,
 * write to the window. Cells below 0.01 are skipped — they were
 * already blanked in PASS 1 (or were never lit). */
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

/* ── Driver — render the heatmap mode as three sequential passes ─── */
static void render_heatmap(WINDOW *w, int cols, int rows, int theme) {
  heatmap_clear_emptied_cells(w, cols, rows);
  heatmap_accumulate_density(cols, rows);
  heatmap_blit_density_field(w, cols, rows, theme);
}

/*
 * render_per_particle() — GLYPH / TRAIL / ARROW modes.
 *
 * Every alive particle is drawn as a single character at its current cell
 * position.  The three modes share the same speed→level and life→dim logic;
 * they differ only in which character is placed:
 *
 *   GLYPH  — k_ramp[lvl]: density glyph encodes kinetic energy directly.
 *   TRAIL  — draws TRAIL_LEN-1 past cell positions first (fading), then the
 *             current glyph.  Older trail entries are dimmer (tlvl -= t/2).
 *             WHY draw trail before current: ensures the head glyph is on top
 *             and not overwritten by a trail entry from the next particle.
 *   ARROW  — velocity_arrow(vx, vy): 8-way direction char instead of density.
 *             Good for visualising flow fields and force alignment.
 *
 * Life-based dimming at 30%:
 *   At lifetime_fraction < 0.30, A_DIM replaces A_BOLD.  This creates a
 *   visible "dying" fade that doesn't require gradual alpha — just two
 *   brightness levels available in all ncurses terminals.
 */
/* ── Per-particle render helpers ─────────────────────────────────── */

/* Speed → ramp level in [1, RAMP_N-1]. Faster particle = denser glyph.
 * Floored at 1 so any alive particle is at least faintly visible
 * (otherwise stationary particles would vanish into the background). */
static inline int particle_ramp_level(const Particle *p) {
  float n = particle_speed(p) / MAX_SPEED_NORM;
  if (n > 1.0f)
    n = 1.0f;
  int lvl = lut_index(n);
  return (lvl < 1) ? 1 : lvl;
}

/* Compose the base render attribute: theme colour at the given ramp
 * level, with A_BOLD swapped for A_DIM when the particle is in the
 * last 30% of its life. Two-tier brightness fade (BOLD → DIM) works
 * on every ncurses terminal without needing alpha blending. */
static inline attr_t particle_render_attr(const Particle *p, int theme,
                                          int level) {
  attr_t a = theme_attr(theme, level);
  float lf = p->lifetime / p->max_lifetime;
  if (lf < 0.30f)
    a = (a & ~A_BOLD) | A_DIM;
  return a;
}

/* Walk the trail ring buffer backwards (newest first), drawing each
 * past CELL position one level dimmer per 2 steps. Older entries are
 * always A_DIM. Drawn BEFORE the head glyph so the head paints on
 * top and stays crisp — otherwise a trail entry from the next
 * particle could overwrite this particle's head. */
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

/* Draw the particle's HEAD glyph at its current cell. mode picks:
 *   RENDER_ARROW → velocity_arrow(vx, vy)  (8-way direction char)
 *   otherwise    → k_ramp[level]            (density glyph by speed) */
static inline void particle_draw_head(WINDOW *w, const Particle *p, int cx,
                                      int cy, int mode, int level,
                                      attr_t attr) {
  char ch =
      (mode == RENDER_ARROW) ? velocity_arrow(p->vx, p->vy) : k_ramp[level];
  wattron(w, attr);
  mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
  wattroff(w, attr);
}

/* ── Driver — GLYPH / TRAIL / ARROW modes ────────────────────────── */

/* Pseudocode per alive particle:
 *   1. cell coords  = pixel → cell conversion
 *   2. ramp level   = particle_ramp_level (speed → density glyph)
 *   3. base attr    = particle_render_attr (theme colour + dim-if-dying)
 *   4. if TRAIL  → particle_draw_trail (paint history first)
 *   5. head glyph → particle_draw_head (paint current cell on top) */
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

/*
 * render_particles() — dispatch to the active render mode.
 *
 * Visual encoding summary by mode:
 *   HEATMAP — continuous density field (Gaussian splat per particle).
 *             Best for fluid/smoke where individual identity is irrelevant.
 *   GLYPH   — per-particle: speed → character density.
 *   TRAIL   — per-particle: past positions fading behind each particle.
 *   ARROW   — per-particle: velocity angle → 8-way direction glyph.
 */
static void render_particles(WINDOW *w, int cols, int rows, int mode,
                             int theme) {
  if (mode == RENDER_HEATMAP) {
    render_heatmap(w, cols, rows, theme);
  } else {
    render_per_particle(w, cols, rows, mode, theme);
  }
}

/*
 * render_overlay() — stats panel in the bottom-left corner.
 *
 * Displays the six key simulation metrics requested by the spec:
 *   particle_count, dt, simulation_time, spawn_rate, avg_velocity,
 *   energy_estimate.
 *
 * Also shows active preset, theme, render mode, and gravity so the
 * user always knows the current simulation state.
 *
 * Drawn LAST inside screen_draw so it appears on top of particles.
 */
static void render_overlay(WINDOW *w, int cols, int rows, int particle_count,
                           float dt_sec, float sim_time, int spawn_rate,
                           float avg_vel, float energy, float gravity,
                           bool drag_on, int preset_id, int theme_id,
                           int render_mode, bool paused) {
  /* Choose a position that fits: bottom-left, 28 cols wide, 10 rows tall */
  int panel_w = 28;
  int panel_h = 10;
  int ox = 1;
  int oy = rows - panel_h - 1;
  if (oy < 0)
    oy = 0;
  if (ox + panel_w > cols)
    return; /* terminal too narrow — skip */

  /* Box top */
  wattron(w, COLOR_PAIR(CP_BASE + theme_id * RAMP_N + 5) | A_DIM);
  mvwprintw(w, oy, ox, "+-- PARTICLE ENGINE ------+");

  /* Stats rows */
  mvwprintw(w, oy + 1, ox, "| cnt  %5d / %-5d      |", particle_count,
            MAX_PARTICLES);
  mvwprintw(w, oy + 2, ox, "| dt   %7.4f s          |", dt_sec);
  mvwprintw(w, oy + 3, ox, "| time %7.2f s          |", sim_time);
  mvwprintw(w, oy + 4, ox, "| rate %5d /tick        |", spawn_rate);
  mvwprintw(w, oy + 5, ox, "| avgv %7.1f px/s       |", avg_vel);
  mvwprintw(w, oy + 6, ox, "| nrg  %9.1f          |", energy);

  /* State row */
  mvwprintw(w, oy + 7, ox, "| %-6s %-6s %-4s %4.0fg |",
            k_preset_names[preset_id], k_themes[theme_id].name,
            k_render_names[render_mode], gravity);

  /* Box bottom */
  mvwprintw(w, oy + 8, ox, "| drag:%-3s  %s              |",
            drag_on ? "ON " : "OFF", paused ? "PAUSED " : "running");
  mvwprintw(w, oy + 9, ox, "+-------------------------+");
  wattroff(w, COLOR_PAIR(CP_BASE + theme_id * RAMP_N + 5) | A_DIM);
}

/* ===================================================================== */
/* §10 presets — fireworks, fountain, rainfall, explosion                 */
/* ===================================================================== */

/*
 * Each preset configures the Scene's emitter + physics parameters.
 * Presets require cols/rows because the emitter position is derived
 * from the terminal dimensions (pixel space).
 *
 * Design notes per preset:
 *
 *   FOUNTAIN  — classic upward spray. Gravity curves arcs downward.
 *               Drag is OFF so the closed-form ballistic apex
 *               (v²/(2g)) is reached and the column reads tall on
 *               screen. Bounce floor so drops accumulate visually.
 *               60° cone, rate 80/s, speed 380-500 → ~300 alive.
 *
 *   FIREWORKS — burst-only (no continuous emission), 600 sparks per
 *               burst on a 1.8 s cadence so successive bursts overlap
 *               with the previous one's still-flying debris. Full
 *               radial cone (spread = 2π), low drag, kill at walls.
 *               Steady-state ≈ 800-1000 sparks on screen.
 *
 *   RAINFALL  — particles fill the entire top edge (spawn_width = pw(cols)).
 *               Strong gravity, narrow downward cone, kill at floor so drops
 *               don't bounce (realistic rain behaviour).
 *               Arrow render mode makes the fall direction clearly visible.
 *
 *   EXPLOSION — radial burst (spread = 2π) from screen centre.
 *               Strong drag dissipates the burst quickly.  Heatmap mode
 *               shows the density shockwave expanding outward.
 *               Auto-burst every 3 s with brief quiet gap.
 */

/* Forward declaration — Emitter lives in Scene defined in §11 */
typedef struct Scene Scene;

static void preset_fountain(Scene *s, int cols, int rows);
static void preset_fireworks(Scene *s, int cols, int rows);
static void preset_rainfall(Scene *s, int cols, int rows);
static void preset_explosion(Scene *s, int cols, int rows);

static void preset_apply(Scene *s, int id, int cols, int rows);

/* ===================================================================== */
/* §11 scene — top-level simulation state; tick + draw                   */
/* ===================================================================== */

/*
 * Scene — owns every piece of mutable state that isn't on individual
 * particles. Two clearly-separated halves:
 *
 *   SIMULATION half — what the physics tick reads/writes. Owns the
 *                     emitter, force constants, boundary policies,
 *                     the auto-burst scheduler, the paused flag,
 *                     and the per-tick measured statistics. Mutated
 *                     by scene_tick() and the key handler.
 *
 *   RENDER half     — what scene_draw() consults to pick colours,
 *                     glyphs, and overlays. Purely visual selection
 *                     indices; never read inside the physics tick.
 *
 * The Scene knows nothing about ncurses — it integrates physics and
 * writes to a WINDOW* passed in by screen_draw. That separation lets
 * the physics be exercised without a terminal (useful for headless
 * scripted runs or future tests).
 */
struct Scene {
  /* ──────────────────────────────────────────────────────────────
   *  SIMULATION HALF — physics tick reads + writes these
   * ────────────────────────────────────────────────────────────── */

  /* SOURCE — the particle factory. preset_apply replaces every
   * Emitter field at once; the 'e' key toggles its `active` flag. */
  Emitter emitter;

  /* FORCES — global physics parameters applied to every alive
   * particle in apply_forces (§6).
   *   gravity : downward acceleration in px/s². Bumped/lowered
   *             by g/G keys. Set by preset_apply at preset switch.
   *             Larger → flatter arcs, faster fall.
   *   drag_on : enable linear-velocity drag? When true, every
   *             velocity is multiplied by exp(-k·density·dt) each
   *             tick (closed-form decay; unconditionally stable).
   *             Toggled by 'd' key. Off = pure ballistic motion. */
  float gravity;
  bool drag_on;

  /* BOUNDARY POLICIES — what happens when a particle crosses an
   * edge. Each wall has its own policy (set by preset_apply).
   *   floor_kills   : true  → particle dies at floor (rainfall).
   *                   false → particle bounces with energy damping.
   *   ceiling_kills : true  → particle dies at top (fountain peak).
   *   wall_kills    : true  → particle dies at left/right edges
   *                          (fireworks, explosion).
   * Bouncing branches in handle_collisions apply restitution
   * (DAMPING) so each bounce loses ~30% of kinetic energy. */
  bool floor_kills;
  bool ceiling_kills;
  bool wall_kills;

  /* PAUSE — when true, scene_tick is a no-op (no force application,
   * no integration, no spawn). Render keeps running so the user
   * sees a frozen frame. Toggled by space-bar. */
  bool paused;

  /* AUTO-BURST SCHEDULER — drives burst-only presets (fireworks,
   * explosion). Every burst_interval seconds the scene fires a
   * spawn_burst(burst_count) automatically.
   *   burst_interval : seconds between auto-bursts. 0 disables
   *                    the scheduler entirely (fountain/rainfall).
   *   burst_timer    : countdown — decrements by dt every tick,
   *                    triggers a burst when it crosses 0 and
   *                    reloads to burst_interval.
   *   burst_count    : number of particles per scheduled burst.
   *                    Higher = denser explosion / fireworks. */
  float burst_interval;
  float burst_timer;
  int burst_count;

  /* CLOCK + MEASURED STATS — written at the end of each
   * scene_tick; read by render_overlay AND the top HUD bar.
   * Treated as read-only outside scene_tick.
   *   dt_sec          : the dt the most recent tick used.
   *                     Surfaces variable step length in the HUD.
   *   simulation_time : monotonically-increasing wall-clock of
   *                     simulated seconds. Resets on 'r' key.
   *   particle_count  : number of alive slots after the tick.
   *                     The "live count" displayed in the HUD.
   *   spawn_rate_last : particles spawned in this tick. Useful
   *                     to confirm rate accumulator behaviour.
   *   avg_velocity    : Σ|v| / particle_count over alive set.
   *                     Read by render_overlay as a temperature-ish
   *                     gauge.
   *   energy_estimate : Σ(½·m·v² + m·g·y) over alive set. A loose
   *                     proxy for total kinetic+potential energy;
   *                     should be near-constant on a closed sim. */
  float dt_sec;
  float simulation_time;
  int particle_count;
  int spawn_rate_last;
  float avg_velocity;
  float energy_estimate;

  /* CURRENT PRESET — index into the preset table. Cycled by p/P
   * keys. Switching preset overwrites every SIMULATION field above
   * via preset_apply, so this is the load-state pointer; the per-
   * field values are the live truth. Lives in the SIMULATION half
   * because changing it changes physics, not just visuals. */
  int preset_id;

  /* ──────────────────────────────────────────────────────────────
   *  RENDER HALF — scene_draw reads these; physics tick ignores them
   * ────────────────────────────────────────────────────────────── */

  /* COLOUR PALETTE — index into k_themes. Cycled by 't' key.
   * theme_attr(theme_id, level) maps a ramp level to the actual
   * ncurses pair + attribute for the current theme. Pure render
   * concern — particles look different but behave identically
   * regardless of which theme is active. */
  int theme_id;

  /* RENDER MODE — which of the four visual encodings to use:
   *   RENDER_GLYPH   : speed → density-ramp char + life → A_DIM
   *   RENDER_TRAIL   : glyph + dim trail of past CELL positions
   *   RENDER_HEATMAP : Gaussian-splat density grid, ramp by total
   *   RENDER_ARROW   : 8-way arrow glyph from atan2(vy, vx)
   * Cycled by 'v' key. Same particles, different visualisation. */
  int render_mode;
};

/* ── preset implementations ────────────────────────────────────────── */

static void preset_fountain(Scene *s, int cols, int rows) {
  Emitter *em = &s->emitter;
  em->x = (float)pw(cols) * 0.5f;
  em->y = (float)ph(rows) - 2.0f;
  em->angle = -(float)M_PI * 0.5f; /* straight up */
  em->spread = (float)M_PI / 3.0f; /* 60° cone    */
  /* Apex (no drag) = v²/(2·g). With g = 200, speed 380→apex 361 px
   * = ~22 cells; speed 500→apex 625 px = ~39 cells. So even the
   * slowest particles clear ~half a 30-row terminal, fastest reach
   * the ceiling on standard terminals. */
  em->speed_min = 380.0f;
  em->speed_max = 500.0f;
  em->life_min = 2.5f;
  em->life_max = 5.0f;
  em->density_min = 0.6f;
  em->density_max = 1.2f;
  /* 80/s × ~3.75 s avg life → ~300 alive steady-state — dense column. */
  em->rate = 80.0f;
  em->rate_accum = 0.0f;
  em->spawn_width = 0.0f;
  em->active = true;

  s->gravity = 200.0f;
  /* Drag OFF — drag was cutting the apex roughly in half; turning it
   * off restores the full ballistic arc so the fountain reads tall. */
  s->drag_on = false;
  s->floor_kills = false; /* bounce at floor */
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
  em->angle = -(float)M_PI * 0.5f; /* upward launch */
  em->spread = 2.0f * (float)M_PI; /* full radial   */
  em->speed_min = 200.0f;
  em->speed_max = 600.0f;
  em->life_min = 1.5f;
  em->life_max = 3.5f;
  em->density_min = 0.4f;
  em->density_max = 0.9f;
  em->rate = 0.0f; /* burst-only, no continuous */
  em->rate_accum = 0.0f;
  em->spawn_width = 0.0f;
  em->active = false;

  s->gravity = 180.0f;
  s->drag_on = false;
  s->floor_kills = true;
  s->ceiling_kills = false;
  s->wall_kills = true;
  /* 600 sparks per burst × overlap (interval 1.8s vs avg life 2.5s) →
   * roughly 800-1000 alive at any moment, well under MAX_PARTICLES.
   * Previous burst's sparks are still in flight when the next fires,
   * which is what gives the sky a continuous shimmer. */
  s->burst_interval = 1.8f;
  s->burst_count = 600;
  s->burst_timer = 0.5f; /* first burst soon after switch */
  s->render_mode = RENDER_GLYPH;
  s->theme_id = 0; /* fire */
}

static void preset_rainfall(Scene *s, int cols, int rows) {
  (void)rows;
  Emitter *em = &s->emitter;
  em->x = (float)pw(cols) * 0.5f;
  em->y = 0.0f;
  em->angle = (float)M_PI * 0.5f; /* straight down  */
  em->spread = 0.25f;             /* narrow cone    */
  em->speed_min = 180.0f;
  em->speed_max = 380.0f;
  em->life_min = 1.5f;
  em->life_max = 3.5f;
  em->density_min = 0.5f;
  em->density_max = 1.0f;
  em->rate = 45.0f;
  em->rate_accum = 0.0f;
  em->spawn_width = (float)pw(cols); /* scatter across full top */
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
  em->spread = 2.0f * (float)M_PI; /* full 360° */
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
  /* Reset burst timer so presets don't inherit stale countdown */
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

/* ── scene lifecycle ────────────────────────────────────────────────── */

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
  /* Re-apply current preset to reset emitter state */
  preset_apply(s, s->preset_id, cols, rows);
}

/*
 * scene_tick() — advance the simulation by one fixed timestep.
 *
 * Called from the accumulator loop in §13.  dt is always exactly
 * 1.0 / sim_fps seconds.  The order of operations is important:
 *
 *   1. Emitter tick       — spawn new particles (fractional accumulator)
 *   2. Auto-burst timer   — fire periodic bursts
 *   3. update_particles   — forces → drag → integrate → age → trail
 *   4. handle_collisions  — enforce world boundaries
 *   5. Update stats       — particle_count, avg_vel, energy (for overlay)
 */
/* ── Scene step helpers ──────────────────────────────────────────── */

/* Advance the wall clock used by stats + the auto-burst timer. */
static inline void scene_advance_clock(Scene *s, float dt) {
  s->dt_sec = dt;
  s->simulation_time += dt;
}

/* Run the spawn pipeline for one tick. Two parallel sources:
 *   CONTINUOUS: emitter_tick — floor(rate · dt) births with fractional carry.
 *   BURST     : auto-burst timer counts down; when it crosses 0 we
 *               fire spawn_burst(burst_count) and reload the timer.
 * Returns total particles spawned this tick (for the HUD). */
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

/* One physics half-step: integrate every alive particle one symplectic
 * Euler step, then resolve boundary collisions. avg_velocity and
 * energy_estimate are written directly into the scene by
 * update_particles so the HUD picks them up next frame. */
static inline void scene_physics_step(Scene *s, float dt, int cols, int rows) {
  update_particles(dt, s->gravity, s->drag_on, &s->avg_velocity,
                   &s->energy_estimate);
  handle_collisions(cols, rows, s->floor_kills, s->ceiling_kills,
                    s->wall_kills);
}

/* Recount the live pool — exposed to HUD + overlay. avg_velocity and
 * energy_estimate are already in the scene from scene_physics_step;
 * this only updates the alive count. */
static inline void scene_measure_stats(Scene *s) {
  s->particle_count = pool_alive_count();
}

/* ── Driver — one simulation tick ────────────────────────────────── */

/* Pseudocode:
 *   if paused                  → no-op
 *   scene_advance_clock        — dt_sec, simulation_time
 *   scene_spawn_step           — continuous + burst emission
 *   scene_physics_step         — forces → drag → integrate → collide
 *   scene_measure_stats        — alive count for the HUD */
static void scene_tick(Scene *s, float dt, int cols, int rows) {
  if (s->paused)
    return;

  scene_advance_clock(s, dt);
  s->spawn_rate_last = scene_spawn_step(s, dt);
  scene_physics_step(s, dt, cols, rows);
  scene_measure_stats(s);
}

/*
 * scene_draw() — render all particles and the overlay into WINDOW *w.
 *
 * alpha is the sub-tick interpolation factor ∈ [0,1).
 * For a pure particle system we could lerp positions by alpha * vel * dt,
 * but at 60 fps the visual difference is imperceptible for fast particles.
 * We accept alpha for API consistency and use it only to skip the render
 * while paused (alpha = 0 implies no progress to show).
 */
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

/* ===================================================================== */
/* §12 screen — ncurses double-buffer display layer                      */
/* ===================================================================== */

/*
 * Screen — the ncurses display layer (identical to framework.c §7).
 *
 * FRAME SEQUENCE (one atomic write per frame):
 *   erase()              — blank newscr
 *   scene_draw()         — particle render + overlay
 *   key hint row         — always written last, always on top
 *   wnoutrefresh(stdscr) — mark newscr ready; no terminal I/O yet
 *   doupdate()           — ONE diff write to the terminal fd
 */
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

/*
 * screen_draw — two-layer HUD over the scene:
 *
 *   Row 0           STATUS LINE.  Bright yellow PAIR_HUD + A_BOLD.
 *                   Refreshed every frame from live Scene state — the
 *                   "dynamic window" of simulation status: preset,
 *                   theme, render mode, alive count / capacity,
 *                   simulation time, fps, sim Hz, paused indicator.
 *                   Drag/gravity toggles also surface here.
 *   Row rows-1      KEY HINT LINE.  Bright cyan PAIR_HINT + A_BOLD.
 *                   Every interactive key the demo accepts, grouped.
 *
 * Both rows are pre-filled with their pair colour so the coloured
 * background spans the full width regardless of text length, and
 * are drawn AFTER scene_draw so particles never bleed through the
 * HUD bars. The render_overlay panel inside scene_draw remains
 * untouched — it surfaces the longer detailed stats list.
 */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps,
                        float alpha, float dt_sec) {
  (void)dt_sec;

  erase();

  scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

  /* ── Top row: dynamic status ─────────────────────────────── */
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

  /* ── Bottom row: every interactive key ───────────────────── */
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

/* ===================================================================== */
/* §13 app — signals, resize, input, main loop                           */
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
  screen_resize(&app->screen);
  /* Re-apply current preset so emitter positions use new dimensions */
  preset_apply(&app->scene, app->scene.preset_id, app->screen.cols,
               app->screen.rows);
  app->need_resize = 0;
}

/*
 * app_handle_key() — all user input in one place.
 *
 * Returns false to signal quit; true to continue.
 * Controls are grouped by category:
 *   navigation (q/ESC/space), spawning (b/e), physics (g/G/d),
 *   simulation (r/p/P), visual (t/v), Hz (]/[).
 */
static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  Screen *sc = &app->screen;

  switch (ch) {
  /* ── quit / pause ──────────────────────────────────────────── */
  case 'q':
  case 'Q':
  case 27:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;

  /* ── spawning ────────────────────────────────────────────────
   * 'b' triggers an immediate burst regardless of emitter state.
   * 'e' toggles the continuous emitter on/off.               */
  case 'b':
  case 'B':
    spawn_burst(&s->emitter, s->burst_count, s->theme_id);
    break;
  case 'e':
  case 'E':
    s->emitter.active = !s->emitter.active;
    break;

  /* ── gravity ─────────────────────────────────────────────────
   * 'g' increases gravity (heavier), 'G' decreases (lighter).
   * Gravity can go to zero for zero-g particle clouds.       */
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

  /* ── drag ────────────────────────────────────────────────────
   * Toggling drag off lets particles travel in pure ballistic
   * arcs — useful for visualising initial velocity directions. */
  case 'd':
  case 'D':
    s->drag_on = !s->drag_on;
    break;

  /* ── reset ───────────────────────────────────────────────────
   * Clears all particles and resets the emitter to preset defaults. */
  case 'r':
  case 'R':
    scene_reset(s, sc->cols, sc->rows);
    break;

  /* ── cycle presets ───────────────────────────────────────────
   * 'p' advances forward, 'P' goes backward through the 4 presets.
   * Resets the particle pool so the new effect starts clean.  */
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

  /* ── cycle themes ────────────────────────────────────────────
   * No color pair reinitialisation needed — all 4×9 = 36 pairs
   * were defined at startup.  Switching is zero-cost.        */
  case 't':
  case 'T':
    s->theme_id = (s->theme_id + 1) % N_THEMES;
    break;

  /* ── cycle render modes ──────────────────────────────────────
   * Switching modes is instant; no state to reset except the
   * density grid which we clear to avoid a stale frame.      */
  case 'v':
  case 'V':
    s->render_mode = (s->render_mode + 1) % RENDER_COUNT;
    if (s->render_mode == RENDER_HEATMAP)
      density_field_reset();
    break;

  /* ── simulation Hz ───────────────────────────────────────────
   * Higher Hz = finer physics timestep = more accurate but
   * more CPU.  Lower Hz = coarser but cheaper.               */
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

/* ─────────────────────────────────────────────────────────────────────
 * main() — fixed-timestep accumulator game loop
 *
 * Identical in structure to framework.c main().  Only scene_*() calls
 * change between animations.  See framework.c §8 for a detailed
 * explanation of each loop phase.
 * ───────────────────────────────────────────────────────────────────── */
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

    /* ── resize ──────────────────────────────────────────────── */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* ── dt ──────────────────────────────────────────────────── */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS; /* pause guard */

    /* ── sim accumulator (fixed timestep) ────────────────────── *
     * Physics always runs at exactly sim_fps Hz regardless of the
     * render frame rate.  The leftover in sim_accum carries forward
     * to the next frame — no time is ever lost or invented.      */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec, app->screen.cols, app->screen.rows);
      sim_accum -= tick_ns;
    }

    /* ── alpha — sub-tick render interpolation factor ─────────── */
    float alpha = (float)sim_accum / (float)tick_ns;

    /* ── FPS counter (500 ms sliding window) ─────────────────── */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* ── frame cap — sleep BEFORE render ─────────────────────── *
     * Sleeping before render means terminal I/O time is NOT charged
     * against the frame budget.  The next frame's dt measurement
     * starts after the sleep ends, not after doupdate() returns.  */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

    /* ── draw + present ──────────────────────────────────────── */
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps, alpha,
                dt_sec);
    screen_present();

    /* ── input ───────────────────────────────────────────────── */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
