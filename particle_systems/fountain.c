/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fountain.c — water particles ejected from a source, arc back under gravity
 *
 * DEMO: A source position emits water particles each tick — narrow
 *       column for GEYSER, wide cone for FOUNTAIN, falling sheet for
 *       WATERFALL, hot wide cone for VOLCANIC. Each particle has an
 *       initial velocity from the source's cone (small angle =
 *       narrow stream; wide angle = spray); gravity pulls them back
 *       toward the ground. When a particle hits the ground it
 *       SPLASHES — small bouncing fragments — and dies.
 *
 *       Patterns:
 *         GEYSER     thin vertical column, fast
 *         FOUNTAIN   wide cone, classic park-fountain shape
 *         WATERFALL  wide downward sheet from the top of the screen
 *         VOLCANIC   wide hot cone — uses the lava palette
 *
 * Study alongside:
 *   rain.c            — same particle-pool + splash mechanism.
 *   snow.c            — same per-particle jitter for organic feel.
 *   physics/bounce_ball.c — same parabolic ballistics (constant gravity).
 *
 * Section map:
 *   §1 config    — constants, themes, per-pattern parameters
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — theme ramp (water + lava) + splash + sky pairs
 *   §4 drop      — Drop struct, drop_glyph_for_velocity
 *   §5 splash    — Splash struct
 *   §6 scene     — pools, tick, draw, prewarm, reseed
 *   §7 screen    — ncurses init / draw / resize
 *   §8 app       — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reseed (clear pools, re-prewarm)
 *   n / N      next pattern   (GEYSER → FOUNTAIN → WATERFALL → VOLCANIC)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster (speed multiplier ×2)
 *   -          slower (÷2)
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/fountain.c \
 *       -o fountain -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Pool-based 2-D particle system with two species —
 *                  DROPS ejected from a source, SPLASH particles emitted
 *                  on ground impact. Each pattern defines a SOURCE
 *                  (position + x spread) and an EJECTION CONE (initial
 *                  speed magnitude + half-angle). At spawn time we draw
 *                  a random angle in [-half_angle, +half_angle] and
 *                  random speed within ±15 % of the base, then resolve
 *                  to (vx, vy) = speed · (sin α, −cos α) for upward
 *                  patterns or (sin α, +cos α) for the falling
 *                  WATERFALL pattern.
 *
 *                  Each tick: each drop integrates explicit Euler
 *                  under constant gravity (vy += g·dt); the parabolic
 *                  trajectory is the result. When y reaches the bottom
 *                  HUD-edge row, the drop spawns N splashes (per
 *                  pattern.splash_mul) and dies.
 *
 *                  Pattern.hot_palette flag selects the LAVA colour
 *                  ramp (orange/red gradient) instead of the WATER
 *                  ramp (blue) without changing any other parameters.
 *                  Same engine, different colour interpretation.
 *
 * Data-structure : Drop[MAX_DROPS] + Splash[MAX_SPLASHES] object pools
 *                  with `active` flag — same shape as rain.c. Linear-
 *                  scan spawn (small N — fine), no malloc at runtime.
 *
 * Rendering      : ASCII only. Glyph picked from velocity magnitude
 *                  and direction: rising fast → `^`/`'`, peak → `*`,
 *                  falling fast → `,`/`.`, mid → `:`/`+`. Colour from
 *                  the active theme's water or lava ramp by drop's
 *                  HEIGHT (peak = brightest). No background fill.
 *
 * Performance    : O(MAX_DROPS + MAX_SPLASHES) per tick. With FOUNTAIN
 *                  defaults (~350 active drops + ~200 splashes), that's
 *                  ~550 particles × 1-cell render ≈ 550 mvaddch per
 *                  frame. Trivial at 60 fps.
 *
 * References     :
 *   • Reeves, W. T. (1983) — "Particle Systems: A Technique for
 *     Modelling a Class of Fuzzy Objects", *ACM TOG* 2(2):91–108.
 *   • Wikipedia — [Projectile motion](https://en.wikipedia.org/wiki/Projectile_motion).
 *     The closed-form parabola y(t) = v₀·t·sin α − ½·g·t² that this
 *     numerical integration approximates with explicit Euler.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each frame the source ejects a few new drops with a random angle
 * inside its cone. Each drop is then a tiny ballistic projectile —
 * its own initial velocity, just gravity pulling it back. The
 * collective shape of the spray emerges from the cone width and the
 * speed/gravity ratio (high speed + low gravity = tall arc; wide
 * cone + medium speed = umbrella shape). Hit the ground → splash.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a cluster of tiny cannons all aimed near vertically, each
 * one slightly different in angle and muzzle velocity. Each cannon
 * fires a ball that arcs into the air and falls back down — the
 * combined effect is a fountain-shape envelope. Tighten the cone
 * (less angular spread) → you get a column / geyser. Widen it → you
 * get a parasol-shape spray. Aim them DOWN from the top → you get
 * a waterfall. Make the muzzle velocity bigger and add hot colour →
 * volcanic eruption. Same engine, four configurations.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. SPAWN. Each tick, count active drops; if below
 *     `pattern.target_drops`, spawn the difference (capped per tick
 *     so a long pause doesn't dump a flood). At spawn:
 *       angle  = (rand − 0.5) · 2 · cone_half_angle
 *       speed  = speed_init · (0.85 + rand · 0.30)
 *       vx     = speed · sin α
 *       vy     = upward ? -speed · cos α  :  +speed · cos α
 *       (x, y) = source position with small x scatter
 *
 *  2. INTEGRATE per drop:
 *       drop.vy += pattern.gravity · dt
 *       drop.x  += drop.vx · dt
 *       drop.y  += drop.vy · dt
 *       drop.age += dt
 *
 *  3. KILL & SPLASH. When drop.y >= rows-2 (or drop.age > life_max,
 *     or drop drifts off-screen sideways): spawn `K = round(splash_mul ·
 *     SPLASH_BASE)` splash particles at the impact (x, y_floor).
 *
 *  4. INTEGRATE SPLASHES (same as rain.c — bounce-ball physics):
 *       splash.vy += SPLASH_GRAVITY · dt
 *       splash.vx *= splash_drag
 *       update position; deactivate when life expires.
 *
 *  5. RENDER. Drop glyph from velocity (rising/peak/falling).
 *     Colour from height fraction (peak = brightest ramp slot,
 *     bottom = dimmest) using the WATER ramp by default and the
 *     LAVA ramp when pattern.hot_palette is set. Splash glyph by
 *     remaining life: fresh = `*`, mid = `+`, old = `.`.
 *
 *  6. HUD on bottom row.
 *
 * KEY FORMULAS
 * ────────────
 *  Cone-distributed initial velocity:
 *    α = (r − 0.5) · 2 · cone_half_angle    (r ∈ [0,1])
 *    v = speed_init · (0.85 + r' · 0.30)
 *    vx = v · sin α
 *    vy = ∓ v · cos α                       (− for upward, + for waterfall)
 *
 *  Closed-form parabolic peak (for an upward fountain at gravity g):
 *    apex_height = (v · cos α)² / (2 · g)        // cells above source
 *    flight_time = 2 · v · cos α / g              // sec from launch to landing
 *    horizontal_range = v · sin α · flight_time   // cells horizontal travel
 *
 *  Height fraction for ramp lookup:
 *    h_frac = clamp((source_y - drop.y) / source_y, 0, 1)   // 0 at ground, 1 at top
 *    ramp_idx = round(h_frac · 7)
 *
 *  Velocity-based glyph:
 *    if |vy| < 4 c/s             : '*'   (apex / hovering)
 *    elif vy > 0 (falling)       : '.'  ',' depending on speed
 *    else (rising)               : '\''  '^' depending on speed
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • UPWARD vs DOWNWARD CONES. The cone formula uses cos α, which
 *    points the cone axis along ±y. For GEYSER/FOUNTAIN/VOLCANIC
 *    cone points UP (vy = −v·cos α). For WATERFALL cone points DOWN
 *    (vy = +v·cos α). The pattern's `upward` flag selects.
 *
 *  • HUGE SPEED + LOW GRAVITY. Drops fly off the top of the screen
 *    and are gone forever. We bound by `life_max` (typically 4-5
 *    sec) so even off-screen drops eventually die and free their
 *    pool slots.
 *
 *  • WATERFALL SOURCE Y. WATERFALL spawns at y ∈ [-2, 0] (just
 *    above top edge). Drops then fall into view immediately.
 *    For GEYSER/FOUNTAIN/VOLCANIC, source is at y = rows-3 (just
 *    above the bottom HUD).
 *
 *  • SPAWN OFF-SCREEN X. Source x with scatter can land outside
 *    [0, cols). The drop renders off-screen until wind/gravity
 *    brings it on-screen — same handling as rain.c.
 *
 *  • PATTERN CHANGE DOESN'T CLEAR. n/N keeps existing drops; new
 *    drops are spawned per the new pattern. They mix briefly until
 *    old drops die. Press r to clear immediately.
 *
 *  • LIFE_MAX KILLS APEX-HOVERING. A perfectly vertical shot has
 *    vx ≈ 0 and arcs back exactly to the source. life_max ensures
 *    we don't keep iterating it forever if it somehow stays
 *    on-screen.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space). Drops freeze mid-arc; splashes freeze mid-bounce.
 *    Resume: motion continues from where it stopped.
 *
 *  • GEYSER. Narrow vertical column shooting up from the bottom-
 *    centre. Drops nearly all aligned vertically; only a few cells
 *    wide. Splashes a small mound where they land.
 *
 *  • FOUNTAIN. Wide umbrella shape — drops spray out at clear angles
 *    from the source, arc through their parabolas, land in a ring
 *    of splashes around the source.
 *
 *  • WATERFALL. Wide downward sheet from near the top of the screen.
 *    Very different visual character — a wall of water flowing down.
 *    Heavy splash line at the bottom.
 *
 *  • VOLCANIC. Wide hot cone — orange/red drops thrown high. Splashes
 *    in matching hot palette. Definitely reads as "lava", not water.
 *
 *  • Theme cycle (`t`/`T`). Each theme produces a distinctively-
 *    coloured fountain; VOLCANIC always uses the theme's lava ramp.
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

    MAX_DROPS        =  900,
    MAX_SPLASHES     =  600,
    SPLASH_BASE_PER_DROP = 4,

    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,

    /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD         =   1,
    PAIR_HINT        =   2,
    PAIR_WATER_BASE  =   3,    /* +0..+7 = 8 water tints (low→high)  */
    PAIR_LAVA_BASE   =  11,    /* +0..+7 = 8 lava  tints (low→high)  */
    PAIR_SPLASH      =  19,
    PAIR_SKY         =  20,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Splash physics. */
#define SPLASH_GRAVITY   220.0f
#define SPLASH_KICK_UP    25.0f
#define SPLASH_KICK_X     20.0f
#define SPLASH_DRAG        2.5f
#define SPLASH_LIFE_MIN    0.25f
#define SPLASH_LIFE_MAX    0.55f

/* Per-drop spawn jitter. */
#define DROP_SPEED_VARIANCE  0.30f      /* ±15% per-drop speed */

/* Pattern enum. */
typedef enum {
    PATTERN_GEYSER    = 0,
    PATTERN_FOUNTAIN  = 1,
    PATTERN_WATERFALL = 2,
    PATTERN_VOLCANIC  = 3,
    N_PATTERNS        = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_GEYSER:    return "GEYSER   ";
    case PATTERN_FOUNTAIN:  return "FOUNTAIN ";
    case PATTERN_WATERFALL: return "WATERFALL";
    case PATTERN_VOLCANIC:  return "VOLCANIC ";
    default:                return "?        ";
    }
}

/*
 * PatternParams — physics + visuals per pattern.
 *
 *   target_drops      : steady-state active drop count
 *   source_top        : true → spawn near top edge (WATERFALL)
 *                       false → spawn just above bottom HUD
 *   source_x_spread   : ± cells around horizontal centre
 *   speed_init        : nominal initial speed magnitude (cells/sec)
 *   cone_half_angle   : half of cone opening (radians)
 *   upward            : true  → vy_init = -speed·cos α (cone points up)
 *                       false → vy_init = +speed·cos α (cone points down)
 *   gravity           : downward acceleration (cells/sec²)
 *   life_max          : hard cap on drop lifetime (sec)
 *   hot_palette       : true  → use LAVA ramp (VOLCANIC)
 *                       false → use WATER ramp
 *   splash_mul        : scales SPLASH_BASE_PER_DROP at impact
 */
typedef struct {
    int   target_drops;
    bool  source_top;
    float source_x_spread;
    float speed_init;
    float cone_half_angle;
    bool  upward;
    float gravity;
    float life_max;
    bool  hot_palette;
    float splash_mul;
} PatternParams;

static const PatternParams pattern_params[N_PATTERNS] = {
    /*                  target  top   xspr  speed  cone(rad)  upward  grav   life  hot   splash */
    /* GEYSER     */ {  240,  false,   2.0f,  85.0f,  0.10f,  true,  120.0f,  5.0f, false, 0.50f },
    /* FOUNTAIN   */ {  380,  false,   3.0f,  72.0f,  0.55f,  true,  110.0f,  4.5f, false, 0.85f },
    /* WATERFALL  */ {  450,  true,   28.0f,  20.0f,  0.10f,  false, 130.0f,  3.5f, false, 1.10f },
    /* VOLCANIC   */ {  340,  false,   5.0f, 115.0f,  0.75f,  true,  130.0f,  5.0f, true,  1.40f },
};

/*
 * Themes — two 8-step ramps per theme:
 *   water[8] — cool ramp, used by GEYSER/FOUNTAIN/WATERFALL
 *   lava [8] — hot ramp,  used by VOLCANIC
 *
 * Each ramp: index 0 = bottom-of-arc dim tint, index 7 = peak-of-arc
 * brightest tint.
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule.
 */
typedef struct {
    const char *name;
    short       water[8];   /* bottom → peak */
    short       lava [8];   /* bottom → peak */
    short       splash;
    short       sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name        water[0..7]                                       lava[0..7]                                      splash sky */

    { "DEFAULT",  {  24,  31,  38,  45,  87, 117, 153, 195 },        {  88, 124, 130, 166, 196, 208, 214, 226 },     117,  234 },
    { "TROPICAL", {  29,  35,  37,  44,  50,  86, 122, 159 },        {  76, 112, 148, 184, 190, 226, 227, 230 },     158,  234 },
    { "ICE",      {  24,  31,  67, 110, 117, 153, 195, 231 },        { 117, 153, 159, 195, 230, 231, 254, 255 },     231,  235 },
    { "METALLIC", { 240, 243, 245, 247, 249, 251, 253, 255 },        { 240, 243, 245, 247, 249, 251, 253, 255 },     255,  234 },
    { "NEON",     {  53,  91, 134, 165, 201, 207, 213, 219 },        { 198, 199, 200, 201, 207, 213, 219, 225 },     219,  234 },
    { "COPPER",   { 130, 137, 173, 179, 215, 222, 229, 230 },        { 130, 166, 172, 208, 215, 220, 222, 229 },     223,  234 },
    { "EMERALD",  {  28,  34,  40,  64,  70, 112, 156, 192 },        {  64,  70, 112, 154, 191, 192, 226, 230 },     192,  234 },
    { "AURORA",   {  43,  44,  79,  85, 121, 157, 195, 230 },        {  91, 127, 163, 199, 207, 213, 219, 225 },     159,  234 },
    { "MIDNIGHT", {  60,  61,  98, 104, 146, 153, 195, 231 },        {  88, 124, 160, 196, 202, 208, 214, 226 },     153,  232 },
    { "MONO",     { 240, 243, 245, 247, 249, 251, 253, 255 },        { 240, 243, 245, 247, 249, 251, 253, 255 },     255,  232 },
};

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
        for (int i = 0; i < 8; i++) {
            init_pair((short)(PAIR_WATER_BASE + i), t->water[i], -1);
            init_pair((short)(PAIR_LAVA_BASE  + i), t->lava [i], -1);
        }
        init_pair(PAIR_SPLASH, t->splash, -1);
        init_pair(PAIR_SKY,    t->sky,    -1);
    } else {
        for (int i = 0; i < 8; i++) {
            init_pair((short)(PAIR_WATER_BASE + i), COLOR_CYAN,   -1);
            init_pair((short)(PAIR_LAVA_BASE  + i), COLOR_YELLOW, -1);
        }
        init_pair(PAIR_SPLASH, COLOR_WHITE, -1);
        init_pair(PAIR_SKY,    COLOR_BLACK, -1);
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
/* §4  drop                                                               */
/* ===================================================================== */

typedef struct {
    float x, y;        /* current position (cells)         */
    float vx, vy;      /* velocity (cells/sec)             */
    float age;         /* seconds since spawn              */
    bool  active;
} Drop;

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

/*
 * drop_glyph_for_velocity() — pick a glyph that reads as motion
 * direction at terminal resolution.
 *
 *   apex / nearly stopped: '*' (the brightest moment of the arc)
 *   rising fast:           '\''   '^'
 *   falling fast:          '.'    ','
 *   slow rising:           '+'
 *   slow falling:          ':'
 */
static char drop_glyph_for_velocity(float vy)
{
    float a = fabsf(vy);
    if (a < 6.0f)  return '*';                /* near apex */
    if (vy < 0.0f) return (a > 40.0f ? '^' : '\'');
    return (a > 40.0f) ? ',' : '.';
}

/* ===================================================================== */
/* §5  splash                                                             */
/* ===================================================================== */

typedef struct {
    float x, y;
    float vx, vy;
    float age, life;
    bool  active;
} Splash;

/* ===================================================================== */
/* §6  scene — pools, tick, draw                                         */
/* ===================================================================== */

typedef struct {
    bool      paused;
    int       speed;
    int       current_theme;
    Pattern   current_pattern;
    uint32_t  rng;
    int       rows, cols;

    Drop      drops    [MAX_DROPS];
    Splash    splashes [MAX_SPLASHES];
} Scene;

static int drop_pool_find_inactive(Scene *s)
{
    for (int i = 0; i < MAX_DROPS; i++)
        if (!s->drops[i].active) return i;
    return -1;
}

static int splash_pool_find_inactive(Scene *s)
{
    for (int i = 0; i < MAX_SPLASHES; i++)
        if (!s->splashes[i].active) return i;
    return -1;
}

static void scene_clear_pools(Scene *s)
{
    for (int i = 0; i < MAX_DROPS;    i++) s->drops[i].active    = false;
    for (int i = 0; i < MAX_SPLASHES; i++) s->splashes[i].active = false;
}

/*
 * scene_spawn_drop — emit one drop from the pattern's source with a
 * cone-distributed initial velocity.
 *
 *   angle  ∈ [-cone_half_angle, +cone_half_angle]
 *   speed  ∈ speed_init · [0.85, 1.15]
 *   vx     = speed · sin α
 *   vy     = ∓ speed · cos α     (− for upward fountain, + for waterfall)
 */
static void scene_spawn_drop(Scene *s)
{
    int idx = drop_pool_find_inactive(s);
    if (idx < 0) return;
    Drop *d = &s->drops[idx];

    const PatternParams *pp = &pattern_params[s->current_pattern];

    float r1 = lcg_unit(&s->rng);
    float r2 = lcg_unit(&s->rng);
    float r3 = lcg_unit(&s->rng);

    /* Source position. */
    float src_cx = (float)s->cols * 0.5f;
    float src_x  = src_cx + (r1 - 0.5f) * 2.0f * pp->source_x_spread;
    float src_y  = pp->source_top
                 ? (-1.0f - r2 * 1.5f)
                 : (float)(s->rows - 3);

    /* Cone angle + speed. */
    float angle  = (r2 - 0.5f) * 2.0f * pp->cone_half_angle;
    float speed  = pp->speed_init
                 * ((1.0f - DROP_SPEED_VARIANCE * 0.5f) + r3 * DROP_SPEED_VARIANCE);
    float vx     = speed * sinf(angle);
    float vy     = pp->upward ? -speed * cosf(angle) : speed * cosf(angle);

    d->x      = src_x;
    d->y      = src_y;
    d->vx     = vx;
    d->vy     = vy;
    d->age    = 0.0f;
    d->active = true;
}

/* Emit splash particles at impact point. */
static void scene_emit_splashes(Scene *s, float impact_x, float impact_y)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    int n = (int)((float)SPLASH_BASE_PER_DROP * pp->splash_mul + 0.5f);
    for (int k = 0; k < n; k++) {
        int idx = splash_pool_find_inactive(s);
        if (idx < 0) return;
        Splash *sp = &s->splashes[idx];

        float r1 = lcg_unit(&s->rng);
        float r2 = lcg_unit(&s->rng);
        float r3 = lcg_unit(&s->rng);

        sp->x    = impact_x;
        sp->y    = impact_y;
        sp->vx   = SPLASH_KICK_X * (r1 * 2.0f - 1.0f);
        sp->vy   = -SPLASH_KICK_UP * (0.6f + r2 * 0.4f);
        sp->age  = 0.0f;
        sp->life = SPLASH_LIFE_MIN + r3 * (SPLASH_LIFE_MAX - SPLASH_LIFE_MIN);
        sp->active = true;
    }
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_FOUNTAIN;
    s->rng             = (uint32_t)clock_ns();
    s->cols            = cols;
    s->rows            = rows;
    scene_clear_pools(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_reseed(Scene *s)
{
    s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
    scene_clear_pools(s);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    dt *= speed_mul;

    const PatternParams *pp = &pattern_params[s->current_pattern];

    /* 1. Spawn to target. */
    int active = 0;
    for (int i = 0; i < MAX_DROPS; i++) if (s->drops[i].active) active++;
    int target    = pp->target_drops;
    if (target > MAX_DROPS) target = MAX_DROPS;
    int spawn_cap = (int)((float)pp->target_drops * dt * 4.0f) + 4;
    int to_spawn  = target - active;
    if (to_spawn > spawn_cap) to_spawn = spawn_cap;
    for (int k = 0; k < to_spawn; k++) scene_spawn_drop(s);

    /* 2. Integrate drops. */
    float kill_y = (float)(s->rows - 2);
    for (int i = 0; i < MAX_DROPS; i++) {
        Drop *d = &s->drops[i];
        if (!d->active) continue;
        d->vy += pp->gravity * dt;
        d->x  += d->vx * dt;
        d->y  += d->vy * dt;
        d->age += dt;

        if (d->age > pp->life_max) {
            d->active = false;
            continue;
        }
        if (d->x < -8.0f || d->x > (float)(s->cols + 8)) {
            d->active = false;
            continue;
        }
        if (d->y >= kill_y) {
            scene_emit_splashes(s, d->x, kill_y);
            d->active = false;
        }
    }

    /* 3. Integrate splashes (gravity + drag). */
    float drag_factor = expf(-SPLASH_DRAG * dt);
    for (int i = 0; i < MAX_SPLASHES; i++) {
        Splash *sp = &s->splashes[i];
        if (!sp->active) continue;
        sp->vy += SPLASH_GRAVITY * dt;
        sp->vx *= drag_factor;
        sp->x  += sp->vx * dt;
        sp->y  += sp->vy * dt;
        sp->age += dt;
        if (sp->age >= sp->life || sp->y > kill_y + 1.0f) {
            sp->active = false;
        }
    }
}

/*
 * scene_draw — render drops + splashes.
 *
 * Drop colour: HEIGHT-driven ramp lookup (peak = brightest).
 *   For upward patterns, height fraction = (source_y - drop.y) / source_y
 *   For WATERFALL, height fraction = drop.y / kill_y         (top = bright)
 * Drop glyph: from velocity (apex / rising / falling).
 * Drop palette: water vs lava chosen by pattern.hot_palette.
 *
 * Splash glyph + colour: by life remaining.
 */
static void scene_draw(const Scene *s)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    int   pair_base = pp->hot_palette ? PAIR_LAVA_BASE : PAIR_WATER_BASE;

    float kill_y = (float)(s->rows - 2);

    /* ── Drops ──────────────────────────────────────────────────── */
    for (int i = 0; i < MAX_DROPS; i++) {
        const Drop *d = &s->drops[i];
        if (!d->active) continue;
        int ix = (int)(d->x + 0.5f);
        int iy = (int)(d->y + 0.5f);
        if (ix < 0 || ix >= s->cols) continue;
        if (iy < 0 || iy >= s->rows - 1) continue;

        /* Height fraction → ramp slot (peak bright, ground dim). */
        float h_frac;
        if (pp->source_top) {
            /* WATERFALL: top is bright, bottom is dim */
            h_frac = 1.0f - d->y / kill_y;
        } else {
            /* Upward fountains: peak high, source low */
            float source_y = (float)(s->rows - 3);
            h_frac = (source_y - d->y) / (source_y + 1.0f);
        }
        if (h_frac < 0.0f) h_frac = 0.0f;
        if (h_frac > 1.0f) h_frac = 1.0f;

        int   ramp_slot = (int)(h_frac * 7.0f + 0.5f);
        if (ramp_slot < 0) ramp_slot = 0;
        if (ramp_slot > 7) ramp_slot = 7;

        char glyph = drop_glyph_for_velocity(d->vy);
        int  attr  = (ramp_slot >= 6) ? A_BOLD
                   : (ramp_slot <= 1) ? A_DIM
                   :                    A_NORMAL;
        int  pair  = pair_base + ramp_slot;
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(iy, ix, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | attr);
    }

    /* ── Splashes ───────────────────────────────────────────────── */
    for (int i = 0; i < MAX_SPLASHES; i++) {
        const Splash *sp = &s->splashes[i];
        if (!sp->active) continue;
        int ix = (int)(sp->x + 0.5f);
        int iy = (int)(sp->y + 0.5f);
        if (ix < 0 || ix >= s->cols) continue;
        if (iy < 0 || iy >= s->rows - 1) continue;

        float r = sp->age / sp->life;
        char  g;
        int   attr;
        if      (r < 0.30f) { g = '*'; attr = A_BOLD;   }
        else if (r < 0.65f) { g = '+'; attr = A_NORMAL; }
        else                { g = '.'; attr = A_DIM;    }

        /* For VOLCANIC, splash colour also uses lava ramp. */
        int splash_pair = pp->hot_palette ? (PAIR_LAVA_BASE + 6) : PAIR_SPLASH;
        attron(COLOR_PAIR(splash_pair) | attr);
        mvaddch(iy, ix, (chtype)(unsigned char)g);
        attroff(COLOR_PAIR(splash_pair) | attr);
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
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

static void scene_counts(const Scene *s, int *out_drops, int *out_spl)
{
    int d = 0, p = 0;
    for (int i = 0; i < MAX_DROPS; i++)    if (s->drops[i].active)    d++;
    for (int i = 0; i < MAX_SPLASHES; i++) if (s->splashes[i].active) p++;
    *out_drops = d;
    *out_spl   = p;
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s);

    int drops, spls;
    scene_counts(s, &drops, &spls);

    const char *state_str = s->paused ? "PAUSED   " : pattern_name(s->current_pattern);

    char buf[200];
    snprintf(buf, sizeof buf,
             " FOUNTAIN   %s   theme:%-9s   drops:%4d  splashes:%3d   "
             "%5.1f fps  %3d Hz  speed:%-3d   "
             "n/p:pat  t/T:theme  +/-:speed  spc:pause  r:reseed  q:quit ",
             state_str, themes[s->current_theme].name,
             drops, spls, fps, sim_fps, s->speed);

    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
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
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
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
