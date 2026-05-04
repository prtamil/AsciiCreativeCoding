/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * embers.c — heat-rising particles with cooling colour over age
 *
 * DEMO: Glowing embers spawn at a heat source at the bottom of the
 *       screen and rise upward on continuous buoyancy. Each ember
 *       has its own lifetime; over its lifetime its colour walks
 *       down the active theme's HEAT RAMP — bright white-hot at
 *       spawn, then yellow, orange, red, deep crimson, finally
 *       dark and dead. Slight per-tick TURBULENCE in vx makes the
 *       rising column flicker like real fire updraft. The
 *       collective effect is a tongue of fire/embers that looks
 *       distinct from a heat-grid CA fire (`fire.c`) — these are
 *       individual free particles you can see flickering against
 *       each other.
 *
 *       Patterns:
 *         BONFIRE   wide source, many embers, medium heat (default)
 *         FORGE     narrow focused source, fast/hot embers
 *         DRAGON    source oscillates left↔right ("breathing")
 *         HEARTH    small gentle source, sparse + slow
 *
 * Study alongside:
 *   fire.c    — CA-based fire on a heat grid (compare the visual
 *               with embers.c: same phenomenon, different abstraction).
 *   rain.c, snow.c, fountain.c — same pool / spawn-to-target /
 *               tick / draw shape; embers.c is the heat-rising
 *               counterpart to those gravity-driven systems.
 *
 * Section map:
 *   §1 config    — constants, heat-ramp themes, per-pattern params
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-pair heat ramp (cool→hot)
 *   §4 ember     — Ember struct
 *   §5 scene     — pool, tick (buoyancy + cooling + turbulence), draw
 *   §6 screen    — ncurses init / draw / resize
 *   §7 app       — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reseed (clear & re-prewarm)
 *   n / N      next pattern   (BONFIRE → FORGE → DRAGON → HEARTH)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *   w / W      shift source right / left
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/embers.c \
 *       -o embers -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Pool-based 2-D particle system with one species
 *                  (ember). Each tick:
 *
 *                    1. Top up to `pattern.target_embers` by spawning
 *                       at the source — a horizontal strip near the
 *                       bottom of the screen with width
 *                       `2 · source_x_spread`. Spawn position: source
 *                       centre ± random within spread; spawn velocity:
 *                       small upward (vy ≈ -vy_init_mag) + small
 *                       horizontal scatter.
 *
 *                    2. Integrate per ember:
 *                         ember.vy   += buoyancy_accel · dt    (always negative — up)
 *                         ember.vx   += turbulence · (r-0.5) · dt
 *                         ember.x    += ember.vx · dt
 *                         ember.y    += ember.vy · dt
 *                         ember.age  += dt
 *
 *                    3. COOLING. The ember's "temperature" is
 *                       `1 − age / life`. We map this to a ramp index
 *                       0..7 (cool/dim → hot/bright) and use that as
 *                       both glyph density and theme colour pair. The
 *                       ember thus visually walks the heat ramp from
 *                       white-hot at spawn through yellow/orange/red
 *                       to dim crimson before vanishing.
 *
 *                    4. Death: when age >= life OR ember leaves the
 *                       screen.
 *
 *                  Pattern.oscillation_amp_frac > 0 makes the source
 *                  centre move sinusoidally each frame (DRAGON). This
 *                  produces the "dragon breath" effect — a stream of
 *                  embers that sweeps from one side of the screen to
 *                  the other and back.
 *
 *                  Distinct from `fire.c` (cellular-automaton fire on
 *                  a 2-D heat grid): there, every cell is a CA state;
 *                  here, every ember is an independent particle with
 *                  its own velocity, age, and trajectory. The CA
 *                  approach gives a soft continuous flame; the
 *                  particle approach gives sparkly individual embers
 *                  that you can track flickering against each other.
 *
 * Data-structure : Ember[MAX_EMBERS] object pool with `active` flag.
 *                  Linear-scan spawn (small N — fine), no malloc.
 *
 * Rendering      : ASCII only. Glyph from temperature (the airy ramp
 *                  `' .,:;-+*'` tail-to-head) + theme heat-ramp pair
 *                  by the same temperature. A_BOLD on hot embers,
 *                  A_DIM on dying ones.
 *
 * Performance    : O(MAX_EMBERS) per tick. With BONFIRE 350 active
 *                  embers, ~350 mvaddch per frame. Trivial at 60 fps.
 *
 * References     :
 *   • Reeves, W. T. (1983) — "Particle Systems: A Technique for
 *     Modelling a Class of Fuzzy Objects", *ACM TOG* 2(2):91–108.
 *     Reeves' original paper used particle systems for the genesis-
 *     planet fireball in *Star Trek II: The Wrath of Khan*. Same
 *     idea, smaller scale.
 *   • Wikipedia — [Black-body radiation](https://en.wikipedia.org/wiki/Black-body_radiation).
 *     Real embers cool through a temperature-driven colour sequence
 *     (~3000 K white → 2000 K orange → 1000 K dim red → cold black)
 *     that the heat-ramp theme palette directly mirrors.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each ember is an independent rising particle that COOLS over its
 * lifetime. The colour is read from a heat ramp by `temperature =
 * 1 − age/life`; the brightest ramp slots map to white-hot at
 * spawn, the dimmest to cold-dead at end-of-life. Combined with
 * upward buoyancy + horizontal turbulence, hundreds of these
 * stacked above a heat source produce a recognisable flame /
 * ember plume.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a row of tiny glowing bugs that hatch at the bottom of
 * the screen, fly upward at their own pace, and fade through the
 * sunset colour palette as they age — cool down — and finally
 * vanish. The shape of the swarm is determined by the source width
 * (narrow source → tall tongue; wide source → broad sheet). The
 * dragon variant moves the spawn point side-to-side over time so
 * the swarm sweeps across the screen.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. SPAWN. Each tick, count active embers. If below target,
 *     spawn (target − active) new embers (capped per tick) at:
 *       cx_t   = cx_base + osc_amp · sin(2π · t / osc_period)   (DRAGON)
 *              = cx_base                                         (others)
 *       x      = cx_t + (r − 0.5) · 2 · source_x_spread
 *       y      = source_y - r' · 2.0
 *       vy     = -vy_init_mag · (0.7 + r" · 0.6)                (upward)
 *       vx     = (r"' − 0.5) · 2 · vx_spread
 *       life   = life_min + r"" · (life_max − life_min)
 *
 *  2. INTEGRATE per ember:
 *       ember.vy  += buoyancy_accel · dt          (always negative)
 *       ember.vx  += (r − 0.5) · turbulence · dt  (random walk)
 *       ember.x   += ember.vx · dt
 *       ember.y   += ember.vy · dt
 *       ember.age += dt
 *
 *  3. COOL & RENDER. temperature = clamp(1 − age/life, 0, 1).
 *       ramp_slot = round(temperature · 7)
 *       glyph     = RAMP_GLYPHS[ramp_slot]
 *       colour    = PAIR_HEAT_BASE + ramp_slot       (theme heat ramp)
 *       attr      = A_BOLD if slot >= 6, A_DIM if slot <= 1
 *
 *  4. KILL. age >= life OR off-screen → deactivate.
 *
 *  5. HUD on bottom row.
 *
 * KEY FORMULAS
 * ────────────
 *  Source oscillation (DRAGON only):
 *    cx(t) = cx_base + osc_amp · sin(2π · t / osc_period)
 *
 *  Initial upward velocity (negative = up):
 *    vy = -vy_init_mag · (0.7 + r · 0.6)              // ±30% jitter
 *
 *  Buoyancy update (constant negative acceleration):
 *    ember.vy += buoyancy_accel · dt                   // accelerates upward
 *
 *  Turbulence (random walk in vx):
 *    ember.vx += (r − 0.5) · turbulence · dt           // jitters laterally
 *
 *  Temperature (cooling over age):
 *    T = max(0, 1 − age / life)
 *    ramp_slot = ⌊T · 7.999⌋
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • UNCAPPED VY. With constant buoyancy and no drag, vy grows
 *    without bound. Embers fly off the top of the screen quickly.
 *    For a slow-flicker visual, cap |vy| at some terminal velocity:
 *    in this implementation we just let `life_max` end the ember
 *    before it accelerates too far.
 *
 *  • OFF-SCREEN HORIZONTAL. Turbulence + DRAGON oscillation can
 *    push embers past the screen edges. We deactivate them when
 *    `x < -2 || x > cols + 2` so the pool slots are reused.
 *
 *  • RAMP SLOT 0. At T = 0 the ember is "cold". With ramp_slot = 0
 *    and `A_DIM` it's still visible (per the bright-half theme rule
 *    in CLAUDE.md), so dying embers fade rather than pop out.
 *
 *  • SOURCE Y. Embers spawn at `source_y = rows - 2 - source_y_offset`
 *    (just above the HUD). Some pattern variants raise this slightly
 *    so the source has a small visible vertical band (~2 cells) where
 *    new embers concentrate.
 *
 *  • DRAGON OSCILLATION TIMING. The oscillation phase uses the
 *    Scene's `time_accum` (advanced only when not paused), so pause
 *    freezes the source mid-sweep. Resume picks up where it stopped.
 *
 *  • PATTERN CHANGE DOES NOT CLEAR. n/N keeps existing embers, new
 *    ones are spawned per the new pattern. They mix until the old
 *    ones cool & die. Press r to clear immediately.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space). Embers freeze in mid-rise. Resume: motion
 *    continues from where it stopped.
 *
 *  • BONFIRE. Wide tongue of embers stretches up the bottom of
 *    the screen, fading through the heat colours. Visible
 *    flicker from per-ember turbulence.
 *
 *  • FORGE. Narrow column of fast/hot embers — a focused jet.
 *    Hottest at the source, cool at the top.
 *
 *  • DRAGON. The ember stream sweeps left and right over a few
 *    seconds. Easily identifiable as "breathing".
 *
 *  • HEARTH. Small, gentle, slow. Sparse — only a handful of
 *    embers visible at any moment.
 *
 *  • Wind override (`w`/`W`). Shifts the source horizontally so
 *    you can place the fire wherever on screen.
 *
 *  • Theme cycle (`t`/`T`). DEFAULT is classic fire (red→yellow);
 *    BLUE_FLAME is gas-jet blue→cyan→white; GREEN_DRAGON is for
 *    fantasy dragon breath; FORGE is intense red-orange; etc.
 *    Each gives a recognisably different flame character.
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
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    SPEED_MIN        =   1,
    SPEED_DEF        =   8,
    SPEED_MAX        =  64,

    MAX_EMBERS       = 1000,

    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,

    /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD         =   1,
    PAIR_HINT        =   2,
    PAIR_HEAT_BASE   =   3,    /* +0..+7 = 8 heat ramp slots         */
    PAIR_SKY         =  11,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Ember source x shift step (cells). */
#define SOURCE_SHIFT_STEP   8.0f

/* Pattern enum. */
typedef enum {
    PATTERN_BONFIRE = 0,
    PATTERN_FORGE   = 1,
    PATTERN_DRAGON  = 2,
    PATTERN_HEARTH  = 3,
    N_PATTERNS      = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_BONFIRE: return "BONFIRE";
    case PATTERN_FORGE:   return "FORGE  ";
    case PATTERN_DRAGON:  return "DRAGON ";
    case PATTERN_HEARTH:  return "HEARTH ";
    default:              return "?      ";
    }
}

/*
 * PatternParams — per-pattern physics + visuals.
 *
 *   target_embers       : steady-state active ember count
 *   source_x_spread     : ± cells around source centre
 *   source_y_offset     : cells above the bottom HUD row
 *   vy_init_mag         : initial upward speed magnitude (cells/sec)
 *   vx_init_spread      : ± cells/sec random initial horizontal
 *   buoyancy_accel      : negative = upward acceleration (cells/sec²)
 *   turbulence          : random vx kick per second (cells/sec²)
 *   life_min, life_max  : ember lifetime range in seconds
 *   oscillation_amp_frac: fraction of cols for source x oscillation
 *                         (0 = stationary; >0 = source sweeps)
 *   oscillation_period  : seconds for one full sweep cycle
 */
typedef struct {
    int   target_embers;
    float source_x_spread;
    int   source_y_offset;
    float vy_init_mag;
    float vx_init_spread;
    float buoyancy_accel;
    float turbulence;
    float life_min, life_max;
    float oscillation_amp_frac;
    float oscillation_period;
} PatternParams;

static const PatternParams pattern_params[N_PATTERNS] = {
    /* Field order:
     *  target  xspr  yoff  vy_i  vx_i  buoy    turb   lmin  lmax  oscamp  oscper
     */
    /* BONFIRE */ {  380,  18.0f,  1, 32.0f, 12.0f, -22.0f,  60.0f, 2.5f, 4.2f,  0.00f, 0.0f },
    /* FORGE   */ {  220,   3.5f,  1, 60.0f,  4.0f, -38.0f,  90.0f, 1.6f, 2.8f,  0.00f, 0.0f },
    /* DRAGON  */ {  480,  10.0f,  1, 48.0f, 22.0f, -28.0f,  85.0f, 2.6f, 4.0f,  0.22f, 3.5f },
    /* HEARTH  */ {  160,   6.0f,  1, 22.0f,  5.0f, -16.0f,  40.0f, 3.0f, 5.0f,  0.00f, 0.0f },
};

/*
 * Themes — each is an 8-step HEAT RAMP from cool/dim (slot 0) to
 * hot/bright (slot 7). DEFAULT is the classic fire ramp; the others
 * remap the same spectrum to alternative colour families.
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule, so even ramp[0]
 * "cool/dying" ember stays visible with A_DIM.
 */
typedef struct {
    const char *name;
    short       heat[8];   /* cool → hot */
    short       sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name          heat[0..7]                                       sky */

    { "DEFAULT",   {  88, 124, 160, 166, 202, 208, 214, 226 },         234 },
    { "COAL",      {  52,  88, 124, 160, 166, 202, 208, 220 },         233 },
    { "FORGE",     { 124, 160, 196, 202, 208, 214, 220, 226 },         234 },
    { "BLUE",      {  17,  19,  27,  39,  45,  87, 153, 195 },         233 },
    { "GREEN",     {  22,  28,  34,  64,  70, 112, 156, 192 },         234 },
    { "VIOLET",    {  53,  91, 134, 165, 207, 213, 219, 225 },         234 },
    { "COPPER",    { 130, 137, 173, 179, 215, 222, 229, 230 },         234 },
    { "AURORA",    {  43,  79, 115, 121, 157, 195, 230, 231 },         234 },
    { "WHITE_HOT", { 240, 243, 245, 247, 249, 251, 253, 255 },         232 },
    { "MONO",      { 244, 246, 248, 250, 252, 253, 254, 255 },         232 },
};

/* Density-glyph ramp from sparse (cool) to dense (hot). */
static const char RAMP_GLYPHS[8] = { '`', '.', ',', ':', ';', '-', '+', '*' };

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

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_HEAT_BASE + i), t->heat[i], -1);
        init_pair(PAIR_SKY, t->sky, -1);
    } else {
        static const short fb[8] = {
            COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_RED,
            COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_HEAT_BASE + i), fb[i], -1);
        init_pair(PAIR_SKY, COLOR_BLACK, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §4  ember                                                              */
/* ===================================================================== */

typedef struct {
    float x, y;        /* current position (cells)         */
    float vx, vy;      /* velocity (cells/sec)             */
    float age;         /* seconds since spawn              */
    float life;        /* seconds until death (cooling)    */
    bool  active;
} Ember;

/* Cheap LCG */
static inline uint32_t lcg_next(uint32_t *st)
{
    *st = *st * 1664525u + 1013904223u;
    return *st;
}
static inline float lcg_unit(uint32_t *st)
{
    return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/* ===================================================================== */
/* §5  scene — pool, tick, draw                                          */
/* ===================================================================== */

typedef struct {
    bool      paused;
    int       speed;
    int       current_theme;
    Pattern   current_pattern;
    float     source_offset_x;   /* user-controlled shift via w/W      */
    uint32_t  rng;
    int       rows, cols;

    float     time_accum;        /* seconds since start (DRAGON osc.)  */

    Ember     embers[MAX_EMBERS];
} Scene;

static int ember_pool_find_inactive(Scene *s)
{
    for (int i = 0; i < MAX_EMBERS; i++)
        if (!s->embers[i].active) return i;
    return -1;
}

static void scene_clear_embers(Scene *s)
{
    for (int i = 0; i < MAX_EMBERS; i++) s->embers[i].active = false;
}

/* Compute current source x centre (with DRAGON oscillation + user shift). */
static float scene_source_cx(const Scene *s)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    float cx = (float)s->cols * 0.5f + s->source_offset_x;
    if (pp->oscillation_amp_frac > 0.0f && pp->oscillation_period > 0.0f) {
        float amp = pp->oscillation_amp_frac * (float)s->cols;
        float w   = 2.0f * (float)M_PI / pp->oscillation_period;
        cx += amp * sinf(w * s->time_accum);
    }
    return cx;
}

/*
 * scene_spawn_ember — emit one ember at the source.
 *
 *   age      = 0
 *   life     = pattern.life_min + r·(life_max−life_min)
 *   vy       = -pattern.vy_init_mag · (0.7 + r·0.6)        // upward
 *   vx       = (r − 0.5)·2·pattern.vx_init_spread          // ± lateral
 *   x        = source_cx + (r − 0.5)·2·pattern.source_x_spread
 *   y        = (rows − 2 − source_y_offset) − r·1.5        // small spawn band
 */
static void scene_spawn_ember(Scene *s)
{
    int idx = ember_pool_find_inactive(s);
    if (idx < 0) return;
    Ember *e = &s->embers[idx];

    const PatternParams *pp = &pattern_params[s->current_pattern];
    float source_cx = scene_source_cx(s);

    float r1 = lcg_unit(&s->rng);
    float r2 = lcg_unit(&s->rng);
    float r3 = lcg_unit(&s->rng);
    float r4 = lcg_unit(&s->rng);
    float r5 = lcg_unit(&s->rng);

    e->x      = source_cx + (r1 - 0.5f) * 2.0f * pp->source_x_spread;
    e->y      = (float)(s->rows - 2 - pp->source_y_offset) - r2 * 1.5f;
    e->vx     = (r3 - 0.5f) * 2.0f * pp->vx_init_spread;
    e->vy     = -pp->vy_init_mag * (0.7f + r4 * 0.6f);
    e->age    = 0.0f;
    e->life   = pp->life_min + r5 * (pp->life_max - pp->life_min);
    e->active = true;
}

/*
 * scene_prewarm — fill the pool to target with embers at random ages
 * so the screen looks like steady-state fire from frame 1.
 *
 * Embers are spawned at the source (as normal) but their `age` is
 * offset by a random fraction of `life`. Their position is then
 * advanced by `age * vy` to where they would be at that moment in
 * a steady-state simulation. This is a quick approximation —
 * accurate enough for visual smoothness without simulating from
 * t = -infinity.
 */
static void scene_prewarm(Scene *s)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    int target = pp->target_embers;
    if (target > MAX_EMBERS) target = MAX_EMBERS;

    int active = 0;
    for (int i = 0; i < MAX_EMBERS; i++) if (s->embers[i].active) active++;

    for (int k = active; k < target; k++) {
        scene_spawn_ember(s);
        /* Find the just-spawned ember and age it forward. */
        for (int i = 0; i < MAX_EMBERS; i++) {
            Ember *e = &s->embers[i];
            if (e->active && e->age == 0.0f) {
                float age_frac = lcg_unit(&s->rng);
                e->age = age_frac * e->life;
                /* Estimate position after `age` seconds of buoyancy. */
                e->y += e->vy * e->age + 0.5f * pp->buoyancy_accel * e->age * e->age;
                e->x += e->vx * e->age;
                break;
            }
        }
    }
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_BONFIRE;
    s->source_offset_x = 0.0f;
    s->rng             = (uint32_t)clock_ns();
    s->cols            = cols;
    s->rows            = rows;
    s->time_accum      = 0.0f;
    scene_clear_embers(s);
    scene_prewarm(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_reseed(Scene *s)
{
    s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
    s->source_offset_x = 0.0f;
    scene_clear_embers(s);
    scene_prewarm(s);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    dt *= speed_mul;

    s->time_accum += dt;

    const PatternParams *pp = &pattern_params[s->current_pattern];

    /* 1. Top up to target. */
    int active = 0;
    for (int i = 0; i < MAX_EMBERS; i++) if (s->embers[i].active) active++;
    int target    = pp->target_embers;
    if (target > MAX_EMBERS) target = MAX_EMBERS;
    int spawn_cap = (int)((float)pp->target_embers * dt * 4.0f) + 4;
    int to_spawn  = target - active;
    if (to_spawn > spawn_cap) to_spawn = spawn_cap;
    for (int k = 0; k < to_spawn; k++) scene_spawn_ember(s);

    /* 2. Integrate embers (buoyancy + turbulence + cooling). */
    for (int i = 0; i < MAX_EMBERS; i++) {
        Ember *e = &s->embers[i];
        if (!e->active) continue;

        /* Random turbulence kick on vx — a per-tick random walk. */
        float turb = (lcg_unit(&s->rng) - 0.5f) * pp->turbulence;

        e->vy += pp->buoyancy_accel * dt;
        e->vx += turb * dt;
        e->x  += e->vx * dt;
        e->y  += e->vy * dt;
        e->age += dt;

        /* Death. */
        if (e->age >= e->life) { e->active = false; continue; }
        if (e->x < -2.0f || e->x > (float)(s->cols + 2)) {
            e->active = false; continue;
        }
        if (e->y < -2.0f) { e->active = false; continue; }
    }
}

/*
 * scene_draw — render embers + a small visible heat-source glow at
 * the source x band (so the fire looks anchored to a base, not just
 * floating particles).
 */
static void scene_draw(const Scene *s)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    int rows_eff = s->rows - 1;     /* leave bottom row for HUD */

    /* ── 1. Heat-source glow ────────────────────────────────────────
     * A 2-row band at the source gets an extra-bright fill so the
     * fire base reads as solid heat rather than just sparse embers. */
    float source_cx = scene_source_cx(s);
    int   src_y0    = s->rows - 2 - pp->source_y_offset;
    int   src_y1    = src_y0 + 1;
    int   half      = (int)(pp->source_x_spread + 0.5f);

    for (int dy = 0; dy <= 1; dy++) {
        int y = (dy == 0) ? src_y0 : src_y1;
        if (y < 0 || y >= rows_eff) continue;
        for (int dx = -half; dx <= half; dx++) {
            int x = (int)(source_cx + 0.5f) + dx;
            if (x < 0 || x >= s->cols) continue;
            /* Bright at centre, dimmer at edges. */
            float r = (float)abs(dx) / (float)(half + 1);
            int   slot = (int)((1.0f - r) * 7.0f + 0.5f);
            if (slot < 4) slot = 4;       /* always reasonably hot   */
            if (slot > 7) slot = 7;
            char  glyph = (slot >= 6) ? '*' : '+';
            int   attr  = A_BOLD;
            int   pair  = PAIR_HEAT_BASE + slot;
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(y, x, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }

    /* ── 2. Embers ──────────────────────────────────────────────── */
    for (int i = 0; i < MAX_EMBERS; i++) {
        const Ember *e = &s->embers[i];
        if (!e->active) continue;
        int ix = (int)(e->x + 0.5f);
        int iy = (int)(e->y + 0.5f);
        if (ix < 0 || ix >= s->cols) continue;
        if (iy < 0 || iy >= rows_eff) continue;

        /* Temperature → ramp slot (cool → hot). */
        float T = 1.0f - e->age / e->life;
        if (T < 0.0f) T = 0.0f;
        if (T > 1.0f) T = 1.0f;
        int slot = (int)(T * 7.999f);
        if (slot < 0) slot = 0;
        if (slot > 7) slot = 7;

        char glyph = RAMP_GLYPHS[slot];
        int  attr  = (slot >= 6) ? A_BOLD
                   : (slot <= 1) ? A_DIM
                   :               A_NORMAL;
        int  pair  = PAIR_HEAT_BASE + slot;
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(iy, ix, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | attr);
    }
}

/* ===================================================================== */
/* §6  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *sc)
{
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
static void screen_free(Screen *sc) { (void)sc; endwin(); }
static void screen_resize_curses(Screen *sc)
{
    endwin();
    refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

/* Active ember count for HUD. */
static int scene_active_count(const Scene *s)
{
    int n = 0;
    for (int i = 0; i < MAX_EMBERS; i++) if (s->embers[i].active) n++;
    return n;
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s);

    int active = scene_active_count(s);

    const char *state_str = s->paused ? "PAUSED " : pattern_name(s->current_pattern);

    char buf[200];
    snprintf(buf, sizeof buf,
             " EMBERS   %s   theme:%-10s   embers:%4d   "
             "src_x_off:%+5.1f   %5.1f fps  %3d Hz  speed:%-3d   "
             "n/p:pat  t/T:theme  w/W:source  +/-:speed  spc:pause  r:reseed  q:quit ",
             state_str, themes[s->current_theme].name, active,
             (double)s->source_offset_x, fps, sim_fps, s->speed);

    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize_curses(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reseed(s);                              break;

    case '=': case '+':
        if (s->speed < SPEED_MAX) s->speed *= 2;
        if (s->speed > SPEED_MAX) s->speed  = SPEED_MAX;
        break;
    case '-':
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed  = SPEED_MIN;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case 'n': case 'N':
        s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
        scene_prewarm(s);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        scene_prewarm(s);
        break;

    case 'w':
        s->source_offset_x += SOURCE_SHIFT_STEP;
        break;
    case 'W':
        s->source_offset_x -= SOURCE_SHIFT_STEP;
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
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
