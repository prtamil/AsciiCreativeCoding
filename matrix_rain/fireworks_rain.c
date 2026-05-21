/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fireworks_rain.c — fireworks with matrix-rain arc trails
 *
 * DEMO: Rockets rise from the bottom of the screen, decelerate to apex,
 *       and explode into 72 sparks per burst. Every spark grows a
 *       16-character matrix-rain trail that follows the exact arc
 *       it traces under gravity — head bright, body dimming, glyphs
 *       rerolling every frame for the classic Matrix shimmer.
 *
 *           ASCII sketch of one spark in flight:
 *
 *               head (white, bold)
 *                ↓
 *                A   ← cache[0]   (newest, BOLD, TRAIL_HOT band)
 *                 q  ← cache[1]   BOLD
 *                  W ← cache[2]   BOLD
 *                   z              normal (TRAIL_WARM band)
 *                    7             normal
 *                     R            DIM   (TRAIL_COOL band)
 *                      e           DIM
 *                       %          DIM   (oldest)
 *
 *       The trail is the LAST 16 head positions kept in a per-spark
 *       ring buffer. Each frame: slide the buffer down, push the new
 *       head into slot [0], integrate physics, reroll most cache
 *       glyphs. The same trajectory therefore re-renders with
 *       constantly-changing characters — the Matrix-rain shimmer.
 *
 * Study alongside:
 *   matrix_rain/matrix_rain.c       — same shimmer-cache trick on
 *                                     plain vertical rain (read first
 *                                     if the cache reroll is unfamiliar).
 *   particle_systems/fireworks.c    — the rocket-and-burst skeleton
 *                                     without trails.
 *   matrix_rain/pulsar_rain.c       — the same shimmer-cache trick
 *                                     on rotating beams.
 *   matrix_rain/matrix_snowflake.c  — rain accumulating into snow.
 *
 * Section map:
 *   §1  config       — every tunable in one place, grouped by concept
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus 5 themed spark palettes
 *   §4  spark        — Vec2, Spark, burst, tick, draw
 *   §5  rocket       — IDLE → RISING → EXPLODED state machine
 *   §6  scene        — RocketPool + SimControls sub-structs; Scene root;
 *                       scene_init / _tick / _draw / _free /
 *                       _change_rockets / _scale_speed / _cycle_theme
 *                       + park_idle_slot / launch_idle_slot helpers
 *   §7  screen       — ncurses init / present / HUD
 *   §8  app          — FpsCounter + App; signals, resize, key dispatch,
 *                       variable-dt main loop (7 numbered phases)
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space / p        pause / resume
 *   r                reset the show
 *   ]   [            speed up / slow down
 *   = / +            more rockets
 *   -                fewer rockets
 *   t                cycle theme (vivid / matrix / fire / ice / plasma)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra matrix_rain/fireworks_rain.c \
 *       -o fireworks_rain -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Three independent ideas stacked on top of each other.
 *
 *                 (A) ROCKET   — a 3-state machine. IDLE counts down a
 *                                fuse (in seconds). RISING climbs under
 *                                gravity until vy ≥ 0 (apex) or y < 2
 *                                (top edge). EXPLODED ticks all sparks
 *                                until none are alive, then returns to
 *                                IDLE with a fresh random fuse.
 *
 *                 (B) SPARK    — one explosion particle with a head
 *                                position + velocity, a 16-slot trail
 *                                history of past head positions, and
 *                                a parallel cache of random ASCII
 *                                glyphs. Each frame: slide history,
 *                                integrate physics, shimmer cache,
 *                                decay life.
 *
 *                 (C) SHIMMER  — for each cache slot, with probability
 *                                1 − 1/SHIMMER_KEEP_ONE_IN, reroll the
 *                                glyph. KEEP_ONE_IN = 4 → 75 % rerolled
 *                                each frame, the classic Matrix-rain
 *                                shimmer rate.
 *
 * Data-structure: Scene ⊃ RocketPool.rockets[16] ⊃ Spark[72] (per
 *                 rocket).  All inline; no heap allocation post-init.
 *                 Scene also carries SimControls (paused, speed_scale)
 *                 and a theme_idx alongside the pool.  Each Spark owns:
 *                 a Vec2 head, a Vec2 velocity, a Vec2[TRAIL_LEN]
 *                 history, a char[TRAIL_LEN] glyph cache, plus life /
 *                 decay / color / active.  The trail history is laid
 *                 out newest-at-[0] oldest-at-[N−1] so painter's-order
 *                 drawing reads as a normal for-loop.
 *
 * Rendering     : Painter's order is load-bearing. For each spark,
 *                 paint oldest trail slot first, newer over older,
 *                 then a single white head on top. Drawing dim
 *                 entries first lets the brighter head overwrite them
 *                 at cells where multiple positions map to the same
 *                 column/row (common near burst centres where sparks
 *                 are tightly packed). trail_attr() maps slot index
 *                 → BOLD / NORMAL / DIM bands; if life < FADING_THRESH
 *                 every slot goes DIM (death fade).
 *
 * Performance   : O(R · P · T) per tick where R = active rockets ≤ 16,
 *                 P = sparks per burst = 72, T = trail slots = 16 —
 *                 about 18k vec2 ops worst case. At 60 fps that's
 *                 ~1.1 M ops/sec, microseconds. Trail history uses a
 *                 literal shift-down (O(T) per push) instead of a ring
 *                 buffer with head index because T = 16 makes the
 *                 constant factor invisible and the shift is easier
 *                 to read.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 *   ── Canonical particle-system paper ────────────────────────────
 *   [1] Reeves, W. T. (1983), "Particle Systems — A Technique for
 *       Modeling a Class of Fuzzy Objects", ACM SIGGRAPH '83 / ACM
 *       TOG 2(2), pp. 91-108 — the FOUNDATIONAL particle-systems
 *       paper.  Introduces the per-particle (pos, vel, accel, age,
 *       colour, alpha) state vector and the spawn / step / expire
 *       lifecycle that every spark in §4 follows.  Originally used
 *       to render the genesis effect in *Star Trek II* (the same
 *       year).  §4 Spark + §5 Rocket are direct descendants.
 *
 *   ── Particle dynamics course notes ─────────────────────────────
 *   [2] Witkin, A. & Baraff, D. (1997), "Particle System Dynamics",
 *       SIGGRAPH '97 course notes — clean pedagogical exposition of
 *       the Euler integrator and particle pool management.  §4
 *       spark_integrate uses the gravity-only update
 *         head += vel · dt;  vel.y += g · dt
 *       directly from Witkin/Baraff's particle-with-acceleration
 *       template (no drag — see "Why no drag" note below).
 *   [3] Akenine-Möller, T., Haines, E. & Hoffman, N. (2018),
 *       "Real-Time Rendering" (4th ed.), Ch. 13 — particle-system
 *       implementation patterns for games, including the fixed-pool
 *       + active-prefix design used by RocketPool in §6.
 *
 *   ── Numerical integration ──────────────────────────────────────
 *   [4] Hairer, E., Lubich, C. & Wanner, G. (2006), "Geometric
 *       Numerical Integration" (2nd ed.), Springer — explicit-Euler
 *       reference.  §4 spark_integrate is plain forward Euler:
 *         head += vel · dt   (pos update)
 *         vel  += g · dt     (velocity update — same tick)
 *       Per-spark gravity is JITTERED (×(1 ± SPARK_GRAVITY_JITTER))
 *       so a burst of sparks spreads naturally rather than all
 *       tracing identical parabolas.
 *
 *   ── Projectile mechanics ───────────────────────────────────────
 *   [5] Symon, K. R. (1971), "Mechanics" (3rd ed.), Addison-Wesley,
 *       Ch. 3 — classical projectile motion under gravity (no air
 *       drag).  Each spark traces an EXACT PARABOLA in the absence
 *       of drag (see Why-no-drag note below).
 *
 *   Why no drag
 *     This file uses pure gravity, no aerodynamic forces.  At
 *     terminal-cell resolution the drag effect on a 1-second
 *     trajectory is sub-cell — invisible.  Skipping drag keeps the
 *     integrator at two lines and the arcs as analytically-tractable
 *     parabolas (modulo the per-spark gravity jitter).
 *
 *   ── Game loop / fixed-cap render ───────────────────────────────
 *   [6] Fiedler, G. (2004, updated 2014), "Fix Your Timestep!",
 *       https://gafferongames.com/post/fix_your_timestep/ — the
 *       variable-dt loop pattern in §8 main.  Unlike physics-heavy
 *       demos this file uses a VARIABLE dt (not fixed-step
 *       accumulator) because rockets are decorative — exact
 *       reproducibility isn't required.  The 100 ms dt cap
 *       (DT_CAP_SEC) is still from Fiedler.
 *
 *   ── Implementation-oriented references ─────────────────────────
 *   [7] Shiffman, D., "The Nature of Code", Ch. 4 (Particle Systems)
 *       — reading-level walkthrough of the particle-system pattern
 *       in Processing.  Pair with [2] for the math.  Online at
 *       https://natureofcode.com/book/chapter-4-particle-systems/
 *
 *   ── Online quick reference ─────────────────────────────────────
 *   [8] Wikipedia: "Particle system", "Euler method",
 *       "Trajectory of a projectile" — quick-reference summaries.
 *       https://en.wikipedia.org/wiki/Particle_system
 *
 *   ── Flavour ────────────────────────────────────────────────────
 *   [9] *The Matrix* (1999, dir. Wachowski) — visual inspiration
 *       for the rerolling-glyph trail, here decorating each spark's
 *       trajectory rather than falling vertically.
 *
 *   ── Companion files in this project ────────────────────────────
 *   See also:
 *     matrix_rain/matrix_rain.c — the same shimmer-cache pattern
 *       applied to plain vertical rain.  Read first if the
 *       per-cell glyph-reroll trick is unfamiliar.
 *     flocking/murmuration.c    — large-N particle pool (1500) with
 *       similar fixed-pool BSS layout but different physics
 *       (Reynolds boid forces vs ballistic + drag).
 *     fluid/fluid_sph.c         — SPH Lagrangian particle pool with
 *       neighbour-list dynamics; same "pool[] + count" storage
 *       pattern as RocketPool.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A normal firework spark traces a parabolic arc, but its visible body
 * is just one bright point. Here we keep the last 16 positions per
 * spark in a buffer, redraw all of them, and randomly reshuffle the
 * GLYPHS at those positions every frame. The result: the spark is no
 * longer a dot but a 16-character snake of jumping ASCII that traces
 * its own trajectory, brightest at the head, dim at the tail.
 *
 * ALGORITHM IN STEPS  (per frame, per spark — read with spark_tick)
 * ──────────────────
 *  1. ADVANCE TRAIL
 *       slide trail[i] = trail[i-1] for i = N-1 down to 1
 *       trail[0] = head            (newest snapshot)
 *  2. INTEGRATE PHYSICS
 *       g          = SPARK_GRAVITY · uniform(1−J, 1+J)
 *       head      += vel · dt
 *       vel.y     += g · dt
 *  3. SHIMMER CACHE
 *       for each k in 0..N-1:
 *         if rand() % SHIMMER_KEEP_ONE_IN ≠ 0:
 *           cache[k] = rand_glyph()
 *  4. LIFE DECAY
 *       life -= decay_rate · dt
 *       if life ≤ 0: active = false
 *
 * KEY FORMULAS
 * ────────────
 *   angle_i  = 2π·i/N + jitter             even fan-out around centre
 *   (vx,vy)  = (cos a · s, sin a · s)      polar→cartesian initial vel
 *   head    += vel · dt                    cells/sec × seconds = cells
 *   vy      += g · dt                      cells/sec² × seconds = cells/sec
 *   trail[0] = head                        most recent position; index = age
 *   shimmer  = 1 − 1/KEEP_ONE_IN           fraction of cache rerolled/frame
 *   life(t)  = life(0) − decay_rate · t    linear decay
 *
 *
 * Background you need
 * ───────────────────
 *   - matrix_rain T3-T4 (shimmer cache + 6-band brightness ramp).
 *   - Newton's laws: pos += vel · dt; vel += g · dt.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Air drag / wind. We use plain gravity, no aerodynamic
 *     forces. The arcs are exact parabolas.
 *   - Particle-particle collision. Sparks pass through each other.
 *   - Sound or smoke effects.
 *
 * ─────────────────────────────────────────────────────────────────────── */


#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
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

/* ── §1.1 frame rate ──────────────────────────────────────────────── */
enum {
    TARGET_FPS = 60,
};

/* ── §1.2 rocket pool ──────────────────────────────────────────────── */
enum {
    ROCKETS_MIN     =  1,
    ROCKETS_DEFAULT =  5,
    ROCKETS_MAX     = 16,
    MAX_ROCKETS     = ROCKETS_MAX,
};

/* Initial fuse stagger — rocket i starts at i × STAGGER seconds after
 * launch. Without this they all fire at t = 0 and bunch up at apex. */
#define INITIAL_FUSE_STAGGER_SEC  0.13f

/* ── §1.3 rocket physics (cells/sec, cells/sec²) ──────────────────── */
/*
 * Launch velocity range. Negative = upward in screen-row coordinates
 * (which increase downward). Slow rocket explodes ~1 sec after launch
 * about 5 rows up; fast rocket ~1.5 sec, ~30 rows up.
 */
#define ROCKET_LAUNCH_VY_MIN_CPS  -18.0f   /* slow rocket             */
#define ROCKET_LAUNCH_VY_MAX_CPS  -48.0f   /* fast rocket             */

/*
 * ROCKET_GRAVITY_CPS2 — downward acceleration on rockets. Higher
 * values → rockets explode lower on screen. 30 cells/sec² puts
 * explosions throughout the upper half.
 */
#define ROCKET_GRAVITY_CPS2       30.0f

/*
 * ROCKET_TOP_EXPLODE_ROW — safety net: explode early if a rocket
 * hits the top edge (rare for too-fast launches).
 */
#define ROCKET_TOP_EXPLODE_ROW    2.0f

/* Random fuse range after a rocket finishes (seconds). 0.5 .. 2.5 s
 * before the same rocket relaunches. */
#define FUSE_MIN_SEC      0.5f
#define FUSE_VAR_SEC      2.0f

/* ── §1.4 spark physics (cells/sec, cells/sec²) ───────────────────── */
enum { PARTICLES_PER_BURST = 72 };

/*
 * Spark initial-velocity magnitude range. Each spark fires off the
 * burst origin at a polar angle with this speed.
 */
#define SPARK_SPEED_MIN_CPS    12.0f
#define SPARK_SPEED_MAX_CPS    40.0f

/*
 * SPARK_GRAVITY_CPS2 — downward acceleration on every spark.
 * SPARK_GRAVITY_JITTER — per-spark variance (±20 % at default 0.20)
 *   so sparks don't all trace the SAME parabola. Without jitter
 *   the whole burst falls in lock-step.
 */
#define SPARK_GRAVITY_CPS2     32.0f
#define SPARK_GRAVITY_JITTER    0.20f

/*
 * Spark life — initial value uniform([MIN, MIN+VAR]) on [0, 1] scale.
 * Decay rate uniform([MIN_RPS, MIN+VAR_RPS]) per second. Lifespan
 * = life / decay → 0.33 s at fastest decay, 1.33 s at slowest.
 */
#define SPARK_LIFE_MIN          0.6f
#define SPARK_LIFE_VAR          0.4f
#define SPARK_DECAY_MIN_RPS     0.75f
#define SPARK_DECAY_VAR_RPS     1.05f

/*
 * BURST_ANGLE_JITTER — radians of random offset added to evenly-
 * spaced burst angles. Without this 72 sparks form a perfect ring;
 * with it the cloud reads as a sphere, not a circle.
 */
#define BURST_ANGLE_JITTER      0.30f

/* ── §1.5 trail + shimmer ─────────────────────────────────────────── */
enum {
    /* TRAIL_LEN — historic positions kept per spark.
     * At 60 fps a trail of 16 spans ~267 ms of arc history. */
    TRAIL_LEN       = 16,

    /* Brightness band boundaries (used by trail_attr). */
    TRAIL_HOT_END   = 2,                /* [0..2]      → BOLD            */
    TRAIL_WARM_END  = TRAIL_LEN / 2,    /* [3..N/2-1]  → NORMAL          */
                                        /* [N/2..N-1]  → DIM             */

    /* Per-frame, each cache slot has a 1-in-KEEP_ONE_IN chance of
     * surviving; the rest reroll. KEEP_ONE_IN = 4 → reroll fraction
     * = 0.75, the classic 75 % Matrix-rain shimmer. */
    SHIMMER_KEEP_ONE_IN = 4,
};

/* ── §1.6 fade threshold ──────────────────────────────────────────── */
/* When life drops below this, every trail slot is forced DIM —
 * the spark visibly fades out before disappearing. */
#define FADING_LIFE_THRESHOLD  0.25f

/* ── §1.7 speed scale (rain global multiplier, [/] keys) ─────────── */
#define SPEED_SCALE_DEFAULT    1.0f
#define SPEED_SCALE_MIN        0.25f
#define SPEED_SCALE_MAX        4.0f
#define SPEED_SCALE_STEP       1.25f

/* ── §1.8 ncurses pair IDs ────────────────────────────────────────── */
enum {
    /* 1..7 — spark colours, theme-controlled */
    CP_RED          = 1,
    CP_ORANGE,
    CP_YELLOW,
    CP_GREEN,
    CP_CYAN,
    CP_BLUE,
    CP_MAGENTA,

    /* 8 — trail head, always white */
    CP_TRAIL_HEAD,

    /* 9..10 — HUD spec, theme-independent */
    PAIR_HUD,
    PAIR_HINT,
};

#define N_SPARK_COLORS  7

/* ── §1.9 dt cap + timing primitives ─────────────────────────────── */
#define DT_CAP_SEC    0.10f
#define NS_PER_SEC    1000000000LL
#define NS_PER_MS     1000000LL
#define HUD_BUF_LEN   96

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Color pairs.
 *
 *   1..7  CP_RED..CP_MAGENTA  spark colour SLOTS, remapped per theme.
 *                             The names refer to the default vivid
 *                             theme; under e.g. the matrix theme
 *                             CP_RED is a dark green. Spark code
 *                             never relies on the literal hue.
 *   8     CP_TRAIL_HEAD       always white (theme-independent).
 *   9     PAIR_HUD            yellow on default bg (CLAUDE.md HUD).
 *   10    PAIR_HINT           cyan on default bg (CLAUDE.md HINT).
 */

/*
 * Theme — one named 7-colour palette for the spark slots.
 *
 * Intent
 *   Each spark in a burst is assigned ONE colour from the active
 *   theme's `colors[]` (or `fallback[]` on 8-colour terminals).  The
 *   assignment is fixed at spawn time — a spark keeps its hue across
 *   its entire trail, so the burst looks like N coloured rays
 *   radiating from the explosion point.
 *
 *   The `t` key cycles through THEME_COUNT presets at runtime;
 *   `theme_apply()` re-registers the colour pairs in one O(N_COLORS)
 *   call.  Each agent's `color_pair` field is the same pair number
 *   throughout — only the pair's foreground changes when the theme
 *   switches.
 *
 * Why two parallel palettes (colors[] vs fallback[])
 *   256-colour terminals get fine-grained hue ramps for visible
 *   shading along each trail; 8-colour terminals can't — fallback[]
 *   picks the closest ANSI primary, accepting that adjacent shades
 *   collapse into the same equivalence class.
 *
 * Themes (preset table)
 *   vivid  — multi-hue (red/orange/yellow/green/cyan/blue/magenta)
 *   matrix — green ramp; trails look like Matrix rain arcs
 *   fire   — reds/oranges/yellows; every burst is a flame arc
 *   ice    — blues/cyans; cold, crystalline trails
 *   plasma — purples/magentas; electric neon arcs
 *
 * Members
 *   name        HUD label ("vivid", "matrix", …) shown in the status row.
 *   colors[]    N_SPARK_COLORS xterm-256 indices.
 *   fallback[]  N_SPARK_COLORS ANSI-8 indices for non-256 terminals.
 *
 * Invariants
 *   Every colors[i] ∈ [24, 255] per CLAUDE.md "Theme Palette Brightness"
 *   — cube indices 16–23 and gray 232–239 are deliberately excluded
 *   (they render as black on default background, making sparks invisible).
 *   24–29 only used as the LOWEST tier (trail tail).
 *   name != NULL.
 *
 * References
 *   CLAUDE.md §"Theme Palette Brightness" for the cube-floor rule.
 *   See also DyeRenderer + ColorTheme in fluid_sph.c / fluid_navier
 *   for the same theme-cycling design pattern at file-scope scale.
 */
typedef struct {
    const char *name;
    int         colors  [N_SPARK_COLORS];
    int         fallback[N_SPARK_COLORS];
} Theme;

static const Theme k_themes[] = {
    { "vivid",
      { 196, 208, 226,  46,  51,  33, 201 },
      { COLOR_RED, COLOR_YELLOW, COLOR_YELLOW,
        COLOR_GREEN, COLOR_CYAN, COLOR_BLUE, COLOR_MAGENTA } },
    { "matrix",
      {  28,  34,  40,  46,  82, 118, 154 },
      { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
        COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN } },
    { "fire",
      { 196, 160, 202, 208, 214, 220,  88 },
      { COLOR_RED, COLOR_RED, COLOR_YELLOW,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED } },
    { "ice",
      {  24,  33,  39,  45,  51,  87, 153 },
      { COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
        COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE } },
    { "plasma",
      {  53,  57,  93, 129, 165, 201, 207 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA } },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static bool g_has_256 = false;

/*
 * theme_apply — re-bind spark pairs 1..7 plus trail-head pair 8 for
 * the chosen theme. PAIR_HUD and PAIR_HINT are NEVER touched here —
 * they carry semantic meaning that must not change with theme.
 */
static void theme_apply(int idx)
{
    const Theme *t = &k_themes[idx];
    for (int i = 0; i < N_SPARK_COLORS; i++) {
        int fg = g_has_256 ? t->colors[i] : t->fallback[i];
        init_pair(i + 1, fg, COLOR_BLACK);
    }
    init_pair(CP_TRAIL_HEAD, g_has_256 ? 255 : COLOR_WHITE, COLOR_BLACK);
}

/*
 * hud_pairs_init — bind PAIR_HUD and PAIR_HINT once at startup. Both
 * use the default terminal background (-1) so the HUD row sits on
 * whatever background the user actually has, never a forced black box.
 */
static void hud_pairs_init(void)
{
    init_pair(PAIR_HUD,  g_has_256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, g_has_256 ?  51 : COLOR_CYAN,   -1);
}

static int color_rand(void) { return 1 + rand() % N_SPARK_COLORS; }

/* ===================================================================== */
/* §4  spark — the heart of the matrix-rain effect                        */
/* ===================================================================== */

/* ── §4.1 ASCII glyph pool + tiny utilities ─────────────────────────── */

/* The shimmer pool — same letters/digits/punctuation as matrix_rain.c. */
static const char k_glyphs[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "!@#$%^&*()-_=+[]{}|;:,.<>?/~`";

#define GLYPHS_LEN (int)(sizeof k_glyphs - 1)

static char  rand_glyph(void) { return k_glyphs[rand() % GLYPHS_LEN]; }
static float urand01   (void) { return (float)rand() / (float)RAND_MAX; }

/* ── §4.2 Vec2 + Spark types ────────────────────────────────────────── */

/*
 * Vec2 — 2-D float vector used everywhere physics or geometry shows up.
 *
 * Intent
 *   Position, velocity, trail-history snapshots — every quantity with
 *   an (x, y) pair is a Vec2.  Returning Vec2 BY VALUE (no out-params)
 *   from helpers lets call sites read linearly:
 *
 *      spark->vel.y += GRAVITY_CELLS_PER_SEC_SQ * dt;
 *      spark->head   = v2_add(spark->head,
 *                              v2_scale(spark->vel, dt));
 *
 * Why a struct (not float[2] or two scalars)
 *   • Named type makes signatures read in the algorithm's vocabulary:
 *     `Vec2 spawn_at` rather than `float spawn_x, spawn_y`.
 *   • Return-by-value works without spilling to memory on x86-64 +
 *     ARM ABIs (8 bytes = one register pair); no perf penalty.
 *   • Eliminates the two-out-parameter idiom from every helper that
 *     wants to "return a 2-D coordinate".
 *
 * Members
 *   x, y    Cartesian components in CELL space (units = terminal cells,
 *           with FRACTIONAL precision — sparks live between cells and
 *           round to integers only at draw time).  Origin at top-left;
 *           y increases downward (matches terminal cell convention).
 *
 * References
 *   [1] Reeves 1983 — every particle in his original paper carries a
 *       position + velocity Vec2 of exactly this shape.
 */
typedef struct { float x, y; } Vec2;

/*
 * Spark — one explosion particle with a matrix-rain arc trail.
 *
 * Intent
 *   A Reeves [1] particle augmented with a HISTORY BUFFER.  Standard
 *   particle systems draw ONE point per particle; this file draws a
 *   COMETARY TRAIL by remembering the last TRAIL_LEN head positions.
 *   The trail is then DECORATED with random ASCII glyphs that reroll
 *   each frame — the Matrix-rain shimmer applied per-particle.
 *
 *   Each tick:
 *     1. spark_advance_trail   — shift trail[] / cache[] down by 1
 *                                 (newest enters at index 0)
 *     2. integrate physics     — vel += accel·dt; head += vel·dt
 *                                (Symon [5] linear drag in vel)
 *     3. reroll glyphs         — ~75 % of cache[] entries get a fresh
 *                                 random ASCII char (the shimmer)
 *     4. age + decay           — life -= decay_rps·dt; active = life > 0
 *
 *   Drawn newest-first so the head glyph sits on top of any older
 *   trail slots that share the same cell.
 *
 * Why a per-spark trail (and not a global trail-cell grid)
 *   Sparks fly along arcs — their trails are CURVED lines through cell
 *   space.  A global grid would lose the ordering ("which spark left
 *   THIS glyph at THIS cell?") and would clobber sparks that cross.
 *   Per-spark trail keeps each arc's history isolated.
 *
 * Why a parallel glyph cache (not random-on-draw)
 *   Without the cache, the same trail cell would show a different
 *   glyph EVERY frame — too much shimmer, the trail loses shape.
 *   Caching + selective reroll means each glyph persists for several
 *   frames and the shimmer reads as TEXTURE, not noise.  Same trick
 *   as matrix_rain/matrix_rain.c — see [9] for the visual heritage.
 *
 * Members
 *   ── kinematic state (rewritten every frame by physics integrator)
 *   head          Current position in cell-space (float; sub-cell
 *                 precision is what makes smooth arcs visible).
 *   vel           Velocity in cells / second.  Forward-Euler updated
 *                 (pos uses OLD vel, then vel takes its g·dt kick —
 *                 see [4]).  At dt ≈ 1/60 s and gravity ≈ 15 cells/s²,
 *                 energy drift per second is sub-cell — invisible.
 *
 *   ── trail history buffer (ring-shift each frame) ────────────────
 *   trail[]       Last TRAIL_LEN head positions, NEWEST AT [0],
 *                 OLDEST AT [TRAIL_LEN-1].  Shifted down by 1 each
 *                 frame in spark_advance_trail (literal memmove —
 *                 T = 16 makes the O(T) cost invisible).
 *   trail_fill    How many slots are actually populated.  Ramps
 *                 0 → TRAIL_LEN over the spark's first TRAIL_LEN
 *                 frames so the trail GROWS from the burst point
 *                 instead of appearing instantly.
 *
 *   ── shimmer cache (parallel to trail[]) ─────────────────────────
 *   cache[]       Random ASCII glyph at each trail position.  Each
 *                 frame ~GLYPH_REROLL_PROB of slots are rerolled —
 *                 the "Matrix" shimmer applied per-particle.
 *
 *   ── lifetime + visual state ─────────────────────────────────────
 *   life          Decays 1.0 (fresh) → 0.0 (dead).  Dimensionless.
 *                 Renderer reads this to pick the trail's tail-fade
 *                 colour pair (life close to 0 → dimmest tier).
 *   decay_rps     Per-second `life` drop.  Randomised per spark at
 *                 burst time so a single explosion fades GRADUALLY
 *                 (sparks die over a spread of times) rather than
 *                 collapsing in unison.
 *   color         Hue assigned at burst time (theme-dependent pair).
 *                 Constant for the spark's lifetime.
 *   active        false once life ≤ 0; rocket_tick stops integrating
 *                 inactive sparks.  Particle slot stays in the pool
 *                 until the rocket reloads.
 *
 * Invariants
 *   0 ≤ trail_fill ≤ TRAIL_LEN.
 *   trail[0] == head (after every spark_advance_trail).
 *   active == (life > 0).
 *
 * References
 *   [1] Reeves 1983 — the per-particle state vector (pos, vel, life,
 *       colour, active) this struct extends.
 *   [4] Hairer et al. — Euler-integration reference.  spark_integrate
 *       uses plain forward Euler (cheap; drift is invisible at the
 *       sub-second lifetimes sparks have).
 *   [5] Symon Mechanics — linear-drag term in the velocity update.
 *   [9] *The Matrix* — visual heritage for the cache reroll.
 */
typedef struct {
    /* kinematic state */
    Vec2  head;
    Vec2  vel;

    /* trail history (ring-shifted each frame) */
    Vec2  trail[TRAIL_LEN];
    int   trail_fill;

    /* parallel glyph cache (matrix-rain shimmer) */
    char  cache[TRAIL_LEN];

    /* lifetime + visual */
    float life;
    float decay_rps;
    int   color;
    bool  active;
} Spark;

/* ── §4.3 spark_burst_spawn — factory for a full explosion ──────────── */

/* burst_angle_for — evenly-spaced ring angle for spark i, plus a
 * uniform jitter so the ring isn't perfectly regular (a perfect ring
 * reads as a circular outline rather than an explosion). */
static inline float burst_angle_for(int i, int count) {
    float ring_angle   = ((float)i / (float)count) * 2.0f * (float)M_PI;
    float random_skew  = urand01() * BURST_ANGLE_JITTER;
    return ring_angle + random_skew;
}

/* random_uniform_in — sample uniformly from [lo, hi). */
static inline float random_uniform_in(float lo, float hi) {
    return lo + urand01() * (hi - lo);
}

/* fill_glyph_cache_random — populate one spark's entire trail-glyph
 * cache with random ASCII characters at burst time.  Subsequent
 * frames only reroll ~75 % via spark_shimmer; the initial fill is
 * what gives the brand-new trail something to display before
 * shimmer takes over. */
static inline void fill_glyph_cache_random(Spark *p) {
    for (int k = 0; k < TRAIL_LEN; k++)
        p->cache[k] = rand_glyph();
}

/*
 * spark_burst_spawn — initialise `count` sparks at `origin`.
 *
 *   For each spark:
 *     (1) pick angle           — evenly spaced + jitter
 *     (2) pick radial speed    — uniform [MIN, MAX]
 *     (3) seed kinematic state — head = origin, vel = speed·(cos,sin)
 *     (4) seed lifetime        — jittered so the burst fades gradually
 *     (5) pick colour          — random from theme palette
 *     (6) fill glyph cache     — initial random ASCII per trail slot
 *     (7) flag active          — ticker starts processing this slot
 */
static void spark_burst_spawn(Spark *pool, int count, Vec2 origin)
{
    for (int i = 0; i < count; i++) {
        Spark *p = &pool[i];

        /* (1) angular direction in the radial fan */
        float angle = burst_angle_for(i, count);

        /* (2) outward speed (cells per second) */
        float speed = random_uniform_in(SPARK_SPEED_MIN_CPS,
                                         SPARK_SPEED_MAX_CPS);

        /* (3) kinematic state */
        p->head        = origin;
        p->vel.x       = cosf(angle) * speed;
        p->vel.y       = sinf(angle) * speed;
        p->trail_fill  = 0;

        /* (4) lifetime + decay — jittered so the burst fades GRADUALLY */
        p->life        = SPARK_LIFE_MIN  + urand01() * SPARK_LIFE_VAR;
        p->decay_rps   = SPARK_DECAY_MIN_RPS
                       + urand01() * SPARK_DECAY_VAR_RPS;

        /* (5) visual */
        p->color       = color_rand();

        /* (6) glyph cache — random ASCII per trail slot */
        fill_glyph_cache_random(p);

        /* (7) flag active — spark_tick will start integrating */
        p->active      = true;
    }
}

/* ── §4.4 per-frame helpers — one named function per ALGORITHM step ── */

/*
 * Step 1 — push current head into trail[0], slide older entries down.
 *
 * The shift loop must run in REVERSE (i = N-1 down to 1). Running it
 * forward would copy the same value across the whole buffer because
 * trail[i] would be overwritten before being read into trail[i+1].
 */
static void spark_advance_trail(Spark *p)
{
    for (int i = TRAIL_LEN - 1; i > 0; i--)
        p->trail[i] = p->trail[i - 1];
    p->trail[0] = p->head;
    if (p->trail_fill < TRAIL_LEN) p->trail_fill++;
}

/*
 * Step 2 — explicit Euler integration with per-spark gravity variance.
 *
 * Each spark's effective gravity is SPARK_GRAVITY_CPS2 scaled by
 * uniform(1−J, 1+J), so sparks fan out instead of all tracing the
 * same parabola. Velocities are in cells/sec; multiplying by dt
 * gives the cell-space displacement directly — no magic scale factor.
 */
static void spark_integrate(Spark *p, float dt)
{
    float jitter = 1.0f - SPARK_GRAVITY_JITTER + urand01() * 2.0f * SPARK_GRAVITY_JITTER;
    float g      = SPARK_GRAVITY_CPS2 * jitter;
    p->head.x += p->vel.x * dt;
    p->head.y += p->vel.y * dt;
    p->vel.y  += g * dt;
}

/*
 * Step 3 — for each cache slot, with probability 1 − 1/KEEP_ONE_IN
 * reroll the glyph. KEEP_ONE_IN = 4 → 75 % rerolled per frame —
 * the classic Matrix-rain shimmer.
 */
static void spark_shimmer(Spark *p)
{
    for (int k = 0; k < TRAIL_LEN; k++)
        if (rand() % SHIMMER_KEEP_ONE_IN != 0)
            p->cache[k] = rand_glyph();
}

/* ── §4.5 spark_tick — orchestrator: one of each helper, in order ───── */

/*
 * One simulation step. Reads as the recipe in MENTAL MODEL → ALGORITHM
 * IN STEPS, line for line: advance trail, integrate, shimmer, decay.
 * Drop the spark from the live pool when life reaches zero.
 */
static void spark_tick(Spark *p, float dt)
{
    if (!p->active) return;

    spark_advance_trail(p);                        /* 1. slide history */
    spark_integrate    (p, dt);                    /* 2. physics       */
    spark_shimmer      (p);                        /* 3. reroll cache  */

    p->life -= p->decay_rps * dt;                  /* 4. life decay    */
    if (p->life <= 0.0f) p->active = false;
}

/* ── §4.6 trail_attr — band the brightness gradient ─────────────────── */

/*
 * Map a trail slot index to its ncurses attribute.
 *
 *   i in [0..TRAIL_HOT_END]    → BOLD   (hot, near-head)
 *   i in [..TRAIL_WARM_END-1]  → NORMAL (mid-fade)
 *   else (deep tail)           → DIM
 *
 *   life < FADING_LIFE_THRESHOLD overrides everything to DIM,
 *   producing a clean death fade as the spark dies.
 */
static attr_t trail_attr(int i, int cp, bool fading)
{
    if (fading)              return COLOR_PAIR(cp) | A_DIM;
    if (i <= TRAIL_HOT_END)  return COLOR_PAIR(cp) | A_BOLD;
    if (i <  TRAIL_WARM_END) return COLOR_PAIR(cp);
    return COLOR_PAIR(cp) | A_DIM;
}

/* ── §4.7 spark_draw — paint trail (oldest first) then head on top ──── */

/*
 * Painter's-algorithm render. Drawing dim entries first lets the
 * brighter head and near-head slots overwrite them at cells where
 * multiple positions map to the same column/row — common near burst
 * centres where many sparks are tightly packed.
 */
/*
 * cell_in_screen — bounds check used by the renderer; off-screen
 * positions are silently dropped without crashing ncurses. */
static inline bool cell_in_screen(int x, int y, int cols, int rows) {
    return x >= 0 && x < cols && y >= 0 && y < rows;
}

/* paint_glyph_at — attron/mvaddch/attroff sandwich for one ASCII
 * character.  Centralises the (chtype)(unsigned char) cast that
 * prevents sign-extension on chars > 127 (CLAUDE.md ncurses bug). */
static inline void paint_glyph_at(int y, int x, char glyph, attr_t attrs) {
    attron(attrs);
    mvaddch(y, x, (chtype)(unsigned char)glyph);
    attroff(attrs);
}

/* paint_trail_slot — paint ONE slot of the trail history buffer.
 * Skips off-screen positions; picks the slot's attr via trail_attr
 * (tier-based brightness with optional A_DIM for fading sparks). */
static inline void paint_trail_slot(const Spark *p, int slot_idx,
                                     int cols, int rows, bool fading) {
    int x = (int)roundf(p->trail[slot_idx].x);
    int y = (int)roundf(p->trail[slot_idx].y);
    if (!cell_in_screen(x, y, cols, rows)) return;
    paint_glyph_at(y, x, p->cache[slot_idx],
                   trail_attr(slot_idx, p->color, fading));
}

/* paint_live_head — paint the spark's current position.  Drawn LAST
 * so it always wins overlap with overlapping trail slots from this
 * or neighbouring sparks.  Uses CP_TRAIL_HEAD + A_BOLD while alive,
 * or the spark's own colour + A_DIM once fading. */
static inline void paint_live_head(const Spark *p, int cols, int rows,
                                    bool fading) {
    int hx = (int)roundf(p->head.x);
    int hy = (int)roundf(p->head.y);
    if (!cell_in_screen(hx, hy, cols, rows)) return;

    attr_t head_attrs = fading
        ? (COLOR_PAIR(p->color)      | A_DIM)
        : (COLOR_PAIR(CP_TRAIL_HEAD) | A_BOLD);
    paint_glyph_at(hy, hx, p->cache[0], head_attrs);
}

/*
 * spark_draw — paint one spark's trail + live head.
 *
 *   (1) skip if inactive
 *   (2) compute the fading-flag once (life threshold check)
 *   (3) trail: oldest → newest (newest paints OVER older slots that
 *       map to the same cell, the painter's-order trick)
 *   (4) head: drawn last, always wins overlap
 */
static void spark_draw(const Spark *p, int cols, int rows)
{
    if (!p->active) return;

    bool fading = (p->life < FADING_LIFE_THRESHOLD);

    /* (3) trail — oldest first so newer slots paint on top */
    for (int slot = p->trail_fill - 1; slot >= 0; slot--)
        paint_trail_slot(p, slot, cols, rows, fading);

    /* (4) live head — drawn last, always wins overlap */
    paint_live_head(p, cols, rows, fading);
}

/* ===================================================================== */
/* §5  rocket — IDLE → RISING → EXPLODED state machine                    */
/* ===================================================================== */

/* ── §5.1 RocketState enum + Rocket type ─────────────────────────── */

/*
 * RocketState — the three-phase state machine of one firework.
 *
 * Intent
 *   A real firework has four stages (loaded → lit → rising → bursting
 *   → fading) compressed here into three because "loaded" and "lit"
 *   become the SAME state from the simulation's view: counting down
 *   a fuse with no visible motion.
 *
 * State diagram
 *
 *      ┌─────────────┐  fuse hits 0    ┌──────────────┐
 *      │   IDLE      │ ───────────────►│   RISING     │
 *      │ (fuse > 0,  │                 │ (climbs at   │
 *      │  no glyph)  │                 │   apex_vy,   │
 *      └─────────────┘                 │  gravity     │
 *           ▲                          │  decelerates)│
 *           │                          └──────┬───────┘
 *           │ all sparks                     │ apex reached
 *           │ inactive                       │ (vy ≥ 0)
 *           │                                ▼
 *      ┌─────────────┐  spark_burst    ┌──────────────┐
 *      │  reload     │ ◄───────────────│   EXPLODED   │
 *      │  (idle      │                 │ (burst alive,│
 *      │   loops to  │                 │  sparks      │
 *      │   IDLE)     │                 │  ticking)    │
 *      └─────────────┘                 └──────────────┘
 *
 * Members
 *   RS_IDLE     = 0  Counting down `fuse_sec`; no rocket body drawn.
 *                    Once the fuse hits 0, transition to RS_RISING.
 *   RS_RISING   = 1  Climbing under gravity until apex (vy ≥ 0).
 *                    A single '|' glyph is drawn at (x, y).  Once
 *                    velocity flips sign, transition to RS_EXPLODED
 *                    after spawning the burst of sparks.
 *   RS_EXPLODED = 2  Burst alive — all PARTICLES_PER_BURST sparks
 *                    ticking independently.  When every spark has
 *                    `active = false`, the rocket recycles back to
 *                    RS_IDLE with a fresh random fuse.
 *
 * Why RS_IDLE = 0 deliberately
 *   memset(0)-initialised rockets start in IDLE without an explicit
 *   initialiser; scene_init uses this for parking inactive slots.
 *
 * References
 *   [1] Reeves 1983 — particle systems lifecycle (spawn / step /
 *       expire); RocketState is the per-rocket version of that
 *       lifecycle, with EXPLODED → IDLE being the recycle phase.
 */
typedef enum {
    RS_IDLE     = 0,    /* counting down a fuse, then launches */
    RS_RISING   = 1,    /* climbing under gravity until apex   */
    RS_EXPLODED = 2,    /* burst alive; sparks ticking         */
} RocketState;

/*
 * Rocket — one ascending streak plus its inline pool of explosion sparks.
 *
 * Intent
 *   Each Rocket owns the FULL LIFECYCLE: the rising body, the burst at
 *   apex, the sparks that fly outward, and the recycle back to IDLE.
 *   Embedding the sparks inline (`Spark particles[PARTICLES_PER_BURST]`)
 *   means no malloc / free per burst — slots are simply marked
 *   `active = false` and overwritten on the next spawn.
 *
 *   At any given moment a rocket is in ONE of three states (see
 *   RocketState).  Vertical motion is 1-D physics (no horizontal
 *   drift); the burst's outward radial velocity comes from the
 *   sparks, not the rocket itself.
 *
 * Why an inline particle pool (not a separate sparks_pool[])
 *   • Each rocket has a hard-coded burst count.  A heap allocation
 *     per burst would be allocator-heavy at high rocket counts
 *     (ROCKETS_MAX × bursts_per_minute).
 *   • Inline storage matches CLAUDE.md "no dynamic allocation after
 *     init" rule — the entire rocket pool is one BSS array, no malloc
 *     anywhere in the hot path.
 *   • Memory locality: when rocket_tick iterates particles, they all
 *     sit in the same cache line as the rocket.
 *
 * Why vy only (no vx)
 *   Rockets fly STRAIGHT UP — they don't drift sideways while rising.
 *   The visible variety comes from (1) the random launch column, and
 *   (2) the radial spark fan AT THE APEX.  Adding vx would diffuse
 *   the visual impact of the upward streak without making the bursts
 *   any more interesting.
 *
 * Members
 *   x, y         Current cell-space position (y fractional; positive
 *                downward by terminal convention, so a CLIMBING rocket
 *                has DECREASING y).
 *   vy           Vertical velocity (cells / sec; NEGATIVE while
 *                climbing because y points down).  Forward-Euler
 *                updated under gravity until vy ≥ 0 marks the apex.
 *   color        Rocket-BODY hue used while RISING (theme-driven).
 *                Not read once the rocket explodes — each spark
 *                picks its own colour from the theme palette.
 *   state        Current RocketState (IDLE / RISING / EXPLODED).
 *   fuse_sec     Seconds remaining on the IDLE-state fuse.  Set to
 *                a fresh random value on every recycle; counted down
 *                each tick by rocket_tick_idle.
 *   particles[]  Inline pool of PARTICLES_PER_BURST Sparks.  Never
 *                allocated/freed — just deactivated and overwritten
 *                on the next burst.  Iterated by rocket_tick_exploded
 *                and rocket_draw.
 *
 * Invariants
 *   state == RS_IDLE     ⇒ fuse_sec > 0  AND  every particle.active == false
 *   state == RS_RISING   ⇒ vy < 0  (until the next tick crosses apex)
 *   state == RS_EXPLODED ⇒ at least one particle.active == true
 *                          (transitions to IDLE when none remain)
 *
 * References
 *   [1] Reeves 1983 — the inline particle-pool design predates the
 *       free-list / arena patterns and is still preferred for small
 *       fixed-size effects.
 *   [3] Akenine-Möller §13 — fixed-pool particle storage in modern
 *       real-time renderers (same design at scale).
 *   [4] Hairer et al. — Euler-integration reference for
 *       rocket_tick_rising (same forward-Euler form as spark_integrate).
 */
typedef struct {
    float        x, y;
    float        vy;
    int          color;
    RocketState  state;
    float        fuse_sec;
    Spark        particles[PARTICLES_PER_BURST];
} Rocket;

/* ── §5.2 rocket_launch — fresh rising rocket from the bottom ─────── */

static void rocket_launch(Rocket *r, int cols, int rows)
{
    r->x     = (float)(rand() % cols);
    r->y     = (float)(rows - 1);
    r->vy    = ROCKET_LAUNCH_VY_MIN_CPS
             + urand01() * (ROCKET_LAUNCH_VY_MAX_CPS - ROCKET_LAUNCH_VY_MIN_CPS);
    r->color = color_rand();
    r->state = RS_RISING;

    for (int i = 0; i < PARTICLES_PER_BURST; i++)
        r->particles[i].active = false;
}

/* ── §5.3 per-state tick functions — one per RocketState ─────────── */

/*
 * RS_IDLE — count down the fuse; launch when it hits zero.
 *
 * cols/rows are forwarded to rocket_launch which uses them to pick
 * a random column for the new rocket.
 */
static void rocket_tick_idle(Rocket *r, float dt, int cols, int rows)
{
    r->fuse_sec -= dt;
    if (r->fuse_sec <= 0.0f)
        rocket_launch(r, cols, rows);
}

/*
 * RS_RISING — integrate position and gravity; explode at apex
 * (vy ≥ 0) or if the rocket clears the top edge (rare too-fast
 * launch). When exploding, spawn PARTICLES_PER_BURST sparks at
 * the rocket's current position.
 */
static void rocket_tick_rising(Rocket *r, float dt)
{
    r->y  += r->vy * dt;
    r->vy += ROCKET_GRAVITY_CPS2 * dt;

    if (r->vy >= 0.0f || r->y < ROCKET_TOP_EXPLODE_ROW) {
        Vec2 origin = { r->x, r->y };
        spark_burst_spawn(r->particles, PARTICLES_PER_BURST, origin);
        r->state = RS_EXPLODED;
    }
}

/*
 * RS_EXPLODED — tick every spark; when none remain alive, return
 * to RS_IDLE with a fresh random fuse.
 */
static void rocket_tick_exploded(Rocket *r, float dt)
{
    bool any_alive = false;
    for (int i = 0; i < PARTICLES_PER_BURST; i++) {
        spark_tick(&r->particles[i], dt);
        if (r->particles[i].active) any_alive = true;
    }
    if (!any_alive) {
        r->fuse_sec = FUSE_MIN_SEC + urand01() * FUSE_VAR_SEC;
        r->state    = RS_IDLE;
    }
}

/* ── §5.4 rocket_tick — dispatch on state ───────────────────────── */

static void rocket_tick(Rocket *r, float dt, int cols, int rows)
{
    switch (r->state) {
    case RS_IDLE:     rocket_tick_idle    (r, dt, cols, rows); break;
    case RS_RISING:   rocket_tick_rising  (r, dt);             break;
    case RS_EXPLODED: rocket_tick_exploded(r, dt);             break;
    }
}

/* ── §5.5 rocket_draw — body while rising, sparks when exploded ──── */

/*
 * Body while rising; particle trails while exploded. IDLE rockets
 * draw nothing — they're "below ground" waiting on the fuse.
 */
/* Glyphs that compose the rising-rocket body. */
enum {
    ROCKET_BODY_GLYPH = '|',   /* bright head (this cell, A_BOLD) */
    ROCKET_TAIL_GLYPH = '\'',  /* faint tail (one cell below)     */
};

/*
 * draw_rising_rocket_body — paint the two-cell rocket streak.
 *
 *   y     : bright '|' in the rocket's colour (A_BOLD)
 *   y + 1 : faint '\'' tail in the same colour (no bold) — gives the
 *           illusion of a one-cell smoke trail without persisting
 *           historical positions.
 *
 * Off-screen positions are silently dropped.
 */
static void draw_rising_rocket_body(const Rocket *r, int cols, int rows) {
    int x = (int)r->x;
    int y = (int)r->y;
    if (!cell_in_screen(x, y, cols, rows)) return;

    paint_glyph_at(y, x, ROCKET_BODY_GLYPH, COLOR_PAIR(r->color) | A_BOLD);

    bool tail_row_visible = (y + 1 < rows);
    if (tail_row_visible)
        paint_glyph_at(y + 1, x, ROCKET_TAIL_GLYPH, COLOR_PAIR(r->color));
}

/*
 * draw_exploded_burst — paint every active spark in the burst.  No
 * back-to-front sort is needed: spark_draw uses painter's order
 * within each spark (oldest trail slot first, live head last).
 */
static void draw_exploded_burst(const Rocket *r, int cols, int rows) {
    for (int i = 0; i < PARTICLES_PER_BURST; i++)
        spark_draw(&r->particles[i], cols, rows);
}

/*
 * rocket_draw — dispatch by RocketState.  IDLE rockets draw nothing
 * (they're "below ground" waiting on the fuse).
 */
static void rocket_draw(const Rocket *r, int cols, int rows)
{
    switch (r->state) {
    case RS_RISING:   draw_rising_rocket_body(r, cols, rows); break;
    case RS_EXPLODED: draw_exploded_burst    (r, cols, rows); break;
    case RS_IDLE:                                              break;
    }
}

/* ===================================================================== */
/* §6  scene — Scene root + RocketPool + SimControls sub-structs          */
/* ===================================================================== */

/*
 * RocketPool — the fixed-size rocket array plus its active prefix.
 *
 * Intent
 *   `rockets[]` is sized at MAX_ROCKETS (the hard ceiling); the first
 *   `active_count` slots are the LIVE rockets, the rest sit idle as
 *   STATE = RS_IDLE with a giant fuse so rocket_tick effectively
 *   ignores them.  Bumping active_count up via the '+' key wakes the
 *   next idle slot.
 *
 * Why a prefix-active design (not a free-list)
 *   Simpler bookkeeping: no need to track which slots are free.  The
 *   pool fits in BSS with no malloc, and inactive slots cost only a
 *   tick of "is your fuse ready? no" — cheap compared to the active
 *   particle physics they would otherwise run.
 *
 * Members
 *   rockets[]      Fixed-capacity pool; slots [0, active_count) live.
 *   active_count   Number of rockets currently in active rotation
 *                  (∈ [ROCKETS_MIN, ROCKETS_MAX], user-tunable
 *                  via + / − keys).
 *
 * Invariants
 *   0 ≤ active_count ≤ MAX_ROCKETS.
 *   rockets[i].state == RS_IDLE for i ≥ active_count
 *   (and their fuse_sec is set to a huge value to prevent firing).
 */
typedef struct {
    Rocket rockets[MAX_ROCKETS];
    int    active_count;
} RocketPool;

/*
 * SimControls — user-facing playback knobs.
 *
 * Intent
 *   Bundles the two boolean / float knobs that the input handler
 *   writes and scene_tick reads.  Same shape as the SimControls
 *   sub-struct on every other Scene in this project.
 *
 * Members
 *   paused        true → scene_tick early-returns; HUD shows "PAUSED".
 *                 Toggled by SPACE / p key.
 *   speed_scale   Multiplier applied to dt before rocket_tick reads
 *                 it.  Default 1.0; live-tunable with [ and ] keys
 *                 (geometric scaling by SPEED_SCALE_STEP per press).
 *                 Clamped to [SPEED_SCALE_MIN, SPEED_SCALE_MAX].
 *
 * Invariants
 *   SPEED_SCALE_MIN ≤ speed_scale ≤ SPEED_SCALE_MAX.
 */
typedef struct {
    bool  paused;
    float speed_scale;
} SimControls;

/*
 * Scene — owns ALL simulation state for one run.
 *
 * Layered ownership
 *
 *     Scene
 *       ├── pool       : RocketPool   ← fixed-size rocket array + active count
 *       ├── sim        : SimControls  ← paused + speed_scale
 *       └── theme_idx  : int          ← active colour palette (t/T cycles)
 *
 *   Every persistent simulation value is reachable from one `Scene *s`.
 *   theme_idx lives here (not on App) because it's a SCENE-level
 *   rendering concern — different scenes could in principle pick
 *   different default themes.
 */
typedef struct {
    RocketPool  pool;
    SimControls sim;
    int         theme_idx;
} Scene;

/*
 * launch_idle_slot — wake one rocket slot with a quick fuse so it
 * fires shortly.  Used by scene_init (to stagger initial firings)
 * and by scene_change_rockets (to wake newly-activated slots).
 *
 * The launch params (`cols`, `rows`) come from the Screen extent —
 * rocket_launch needs them to pick the launch column + apex height.
 */
static void launch_idle_slot(Scene *s, int slot_index, int cols, int rows,
                             float fuse_sec) {
    Rocket *r = &s->pool.rockets[slot_index];
    rocket_launch(r, cols, rows);
    r->fuse_sec = fuse_sec;
    r->state    = RS_IDLE;
}

/*
 * park_idle_slot — put a slot into "effectively never fires" state.
 * Sets fuse_sec to a huge value so rocket_tick_idle never trips it,
 * and zeros every particle.active flag.
 *
 * Used by scene_init to park slots ABOVE active_count.
 */
static void park_idle_slot(Scene *s, int slot_index) {
    Rocket *r = &s->pool.rockets[slot_index];
    r->state    = RS_IDLE;
    r->fuse_sec = 1e9f;        /* effectively never */
    for (int j = 0; j < PARTICLES_PER_BURST; j++)
        r->particles[j].active = false;
}

static void scene_init(Scene *s, int cols, int rows, int rocket_count) {
    s->pool.active_count = rocket_count;
    s->sim.paused        = false;
    s->sim.speed_scale   = SPEED_SCALE_DEFAULT;

    for (int i = 0; i < MAX_ROCKETS; i++) {
        if (i < rocket_count) {
            /* Stagger initial firings so they don't all explode at once. */
            float staggered_fuse = (float)i * INITIAL_FUSE_STAGGER_SEC;
            launch_idle_slot(s, i, cols, rows, staggered_fuse);
        } else {
            park_idle_slot(s, i);
        }
    }
}

static void scene_free(Scene *s) { memset(s, 0, sizeof *s); }

static void scene_tick(Scene *s, float dt, int cols, int rows) {
    if (s->sim.paused) return;
    float scaled_dt = dt * s->sim.speed_scale;
    for (int i = 0; i < s->pool.active_count; i++)
        rocket_tick(&s->pool.rockets[i], scaled_dt, cols, rows);
}

static void scene_draw(const Scene *s, int cols, int rows) {
    for (int i = 0; i < s->pool.active_count; i++)
        rocket_draw(&s->pool.rockets[i], cols, rows);
}

/* Scene input helpers — used by app_handle_key. */

/* scene_change_rockets — bump the active_count by `delta`, clamp to
 * [ROCKETS_MIN, ROCKETS_MAX].  When growing, wake the next idle slot
 * with a short fuse so the new rocket fires soon (not after a long
 * wait that would feel unresponsive). */
static void scene_change_rockets(Scene *s, int delta, int cols, int rows) {
    int next_count = s->pool.active_count + delta;
    if (next_count < ROCKETS_MIN) next_count = ROCKETS_MIN;
    if (next_count > ROCKETS_MAX) next_count = ROCKETS_MAX;

    if (next_count > s->pool.active_count) {
        int waking_slot = s->pool.active_count;
        launch_idle_slot(s, waking_slot, cols, rows, INITIAL_FUSE_STAGGER_SEC);
    }
    s->pool.active_count = next_count;
}

/* scene_scale_speed — multiply speed_scale by `factor`, clamped.
 *   factor > 1 (']' key) → faster
 *   factor < 1 ('[' key) → slower
 * Geometric scaling means a press has the same proportional effect
 * regardless of current speed. */
static void scene_scale_speed(Scene *s, float factor) {
    s->sim.speed_scale *= factor;
    if (s->sim.speed_scale < SPEED_SCALE_MIN) s->sim.speed_scale = SPEED_SCALE_MIN;
    if (s->sim.speed_scale > SPEED_SCALE_MAX) s->sim.speed_scale = SPEED_SCALE_MAX;
}

/* scene_cycle_theme — t/T → next palette index, wrap at THEME_COUNT,
 * re-register the colour pairs.  Pure render mutation. */
static void scene_cycle_theme(Scene *s) {
    s->theme_idx = (s->theme_idx + 1) % THEME_COUNT;
    theme_apply(s->theme_idx);
}

/* ===================================================================== */
/* §7  screen — ncurses init / present / HUD                              */
/* ===================================================================== */

/*
 * Screen — terminal cell extent + ncurses lifecycle wrapper.
 *
 * Intent
 *   Tracks the TERMINAL side of the demo: cell dimensions for HUD
 *   placement and field clipping.  ncurses owns the back buffer;
 *   this struct is the source-of-truth for cell-space dimensions,
 *   refreshed via getmaxyx in screen_init / screen_resize.
 *
 *   Sparks live in CELL space (with sub-cell precision for smooth
 *   arcs) and the Spark.head Vec2 components are in the same units
 *   as Screen.cols / Screen.rows.  No pixel-vs-cell aspect bridge —
 *   this is a pure cell-space demo.
 *
 * Why a tiny 2-field struct (not flat ints on App)
 *   • Lifecycle isolation: only screen_init / screen_resize /
 *     screen_free / screen_draw_hud touch ncurses' initscr / endwin
 *     / mvprintw.  They all take `Screen *` to make this layer
 *     explicit at the type level.
 *   • Symmetry with the rest of the project — every demo's Screen
 *     uses the same {cols, rows} shape.
 *
 * Members
 *   cols   Terminal width in CELLS (getmaxyx).
 *   rows   Terminal height in CELLS.  Bottom row index is rows − 1
 *          (where the key hint paints).  Row 0 is the status row.
 *
 * Invariants
 *   cols > 0, rows > 0 after screen_init.
 *   Both refreshed on every SIGWINCH via screen_resize +
 *   app_do_resize (which also re-inits the scene at the new size).
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);              /* don't let stdin interrupt frame writes */
    start_color();
    use_default_colors();       /* lets HUD pairs use -1 background       */
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw_hud — required HUD per CLAUDE.md spec.
 *
 *   Row 0       PAIR_HUD  + A_BOLD  (yellow) — fps + state + params
 *   Bottom row  PAIR_HINT + A_BOLD  (cyan)   — full key list
 *
 * Both pairs sit on default background (-1) so they stay legible
 * regardless of theme. theme_apply() never touches them.
 */
static void screen_draw_hud(const Screen *sc, double fps,
                            const Scene *scene)
{
    char buf[HUD_BUF_LEN];
    snprintf(buf, sizeof buf,
             " %5.1f fps  spd:%.2fx  rkt:%d  [%s] %s ",
             fps, scene->sim.speed_scale, scene->pool.active_count,
             k_themes[scene->theme_idx].name,
             scene->sim.paused ? "PAUSED " : "running");

    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  []:speed  +/-:rockets  t:theme ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app — signals, resize, variable-dt main loop                       */
/* ===================================================================== */

/*
 * FpsCounter — rolling-window frame-rate estimator.
 *
 * Per-frame fps would jitter; this accumulates frame_count + elapsed
 * nanoseconds over a 500 ms window and emits a smoothed `display`
 * value each time the window fills.  Same shape as the FpsCounter on
 * every other file in this project.
 */
typedef struct {
    int     frame_count;
    int64_t window_ns;
    double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f) {
    f->frame_count = 0;
    f->window_ns   = 0;
    f->display     = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt_ns) {
    const int64_t FPS_WINDOW_NS = NS_PER_SEC / 2;     /* 500 ms */
    f->frame_count++;
    f->window_ns += dt_ns;
    if (f->window_ns < FPS_WINDOW_NS) return;
    f->display     = (double)f->frame_count
                   * (double)NS_PER_SEC / (double)f->window_ns;
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * App — top-level container for every persistent value.
 *
 *   scene         simulation state (rocket pool + sim + theme)
 *   screen        terminal cell extent + ncurses lifecycle
 *   fps           rolling-window fps estimator for HUD
 *   running       sig_atomic_t flag cleared by SIGINT / SIGTERM + 'q'
 *   need_resize   sig_atomic_t flag set by SIGWINCH; main reacts at
 *                 the top of the next iteration
 *
 * `g_app` is the program's only file-scope mutable state.  Signal
 * handlers reach the flags through it; everything else flows via
 * `App *app` parameter.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    FpsCounter            fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/*
 * app_do_resize — full re-init of the scene on the new screen size,
 * preserving user-tuned theme and rocket count.
 */
static void app_do_resize(App *app)
{
    int   saved_n     = app->scene.pool.active_count;
    float saved_speed = app->scene.sim.speed_scale;
    int   saved_theme = app->scene.theme_idx;

    scene_free(&app->scene);
    screen_resize(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows, saved_n);
    app->scene.sim.speed_scale = saved_speed;
    app->scene.theme_idx       = saved_theme;
    app->need_resize           = 0;
}

/*
 * app_handle_key — process one keystroke.  Returns false on quit.
 *
 *   q / Q / ESC      quit
 *   p / P / SPACE    toggle paused
 *   r / R            reset (re-init scene at current rocket count)
 *   ]                speed up (×SPEED_SCALE_STEP)
 *   [                slow down (÷SPEED_SCALE_STEP)
 *   + / =            add a rocket
 *   -                remove a rocket
 *   t / T            cycle theme
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    int    cols = app->screen.cols;
    int    rows = app->screen.rows;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ': case 'p': case 'P': s->sim.paused = !s->sim.paused;       break;
    case 'r': case 'R':           scene_init(s, cols, rows,
                                              s->pool.active_count);    break;

    case ']':                     scene_scale_speed(s, SPEED_SCALE_STEP);          break;
    case '[':                     scene_scale_speed(s, 1.0f / SPEED_SCALE_STEP);   break;

    case '=': case '+':           scene_change_rockets(s, +1, cols, rows); break;
    case '-':                     scene_change_rockets(s, -1, cols, rows); break;

    case 't': case 'T':           scene_cycle_theme(s);                  break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)clock_ns());

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App   *app   = &g_app;
    Scene *scene = &app->scene;
    app->running = 1;
    fps_counter_init(&app->fps);
    scene->theme_idx = 0;

    screen_init(&app->screen);
    g_has_256 = (COLORS >= 256);
    theme_apply(scene->theme_idx);
    hud_pairs_init();
    scene_init(scene, app->screen.cols, app->screen.rows, ROCKETS_DEFAULT);

    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;
    int64_t last_ns = clock_ns();

    while (app->running) {

        /* (1) handle resize first so subsequent steps see the new size */
        if (app->need_resize) {
            app_do_resize(app);
            last_ns = clock_ns();
        }

        /* (2) measure dt, capped to prevent spiral-of-death */
        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - last_ns;
        last_ns        = now_ns;
        float   dt     = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        /* (3) drain input */
        for (int ch; (ch = getch()) != ERR; ) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }

        /* (4) advance the scene */
        scene_tick(scene, dt, app->screen.cols, app->screen.rows);

        /* (5) rolling fps counter */
        fps_counter_tick(&app->fps, dt_ns);

        /* (6) draw + present */
        erase();
        scene_draw(scene, app->screen.cols, app->screen.rows);
        screen_draw_hud(&app->screen, app->fps.display, scene);
        screen_present();

        /* (7) frame cap — sleep before the NEXT frame's I/O */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
    }

    scene_free(scene);
    screen_free(&app->screen);
    return 0;
}
