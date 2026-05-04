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
 * References     :
 *   • Reeves, W. T. (1983) — "Particle Systems: A Technique for
 *     Modelling a Class of Fuzzy Objects", *ACM TOG* 2(2):91–108.
 *     Foundational paper on pool-based particle systems for fire,
 *     explosions, and natural phenomena like rain/snow.
 *   • Wikipedia — [Rain](https://en.wikipedia.org/wiki/Rain).
 *     Real-world drop-size and terminal-velocity tables (drizzle ~9
 *     km/h, heavy storm ~33 km/h).
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
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a rain stage with N hooks on the rafter. Each hook can
 * hold ONE drop at a time. When you need a new drop, walk along
 * the row and grab the first empty hook; when a drop hits the
 * floor you put the hook back. The total drops on stage at any
 * moment is at most N, and we keep refilling as drops die. The
 * splash particles work the same way with their own row of hooks.
 * Wind is just an extra horizontal velocity; gravity is `vy +=
 * GRAVITY · dt`. Drops are a streak along the velocity direction;
 * splashes are a few sparks that arc and fade.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. ACTIVATE-TO-TARGET. Each tick, count active drops. If less
 *     than `pattern.target`, scan the pool for an inactive slot and
 *     spawn a new drop at a random column above the screen.
 *
 *  2. INTEGRATE DROPS:
 *       drop.vy = pattern.drop_speed         // constant terminal velocity
 *       drop.vx = pattern.wind_x + extra
 *       drop.x += drop.vx · dt
 *       drop.y += drop.vy · dt
 *
 *  3. KILL & SPLASH. When `drop.y >= rows - 2`:
 *       - mark drop inactive
 *       - spawn `K = round(SPLASHES_PER_DROP · pattern.splash_mul)`
 *         splash particles at the impact point with random outward
 *         velocity (small horizontal kick + small upward).
 *
 *  4. INTEGRATE SPLASHES (bounce-ball physics):
 *       splash.vy += SPLASH_GRAVITY · dt
 *       splash.vx *= SPLASH_DRAG (per second)
 *       splash.x += splash.vx · dt
 *       splash.y += splash.vy · dt
 *       splash.age += dt
 *       if (splash.age > splash.life) deactivate
 *
 *  5. RENDER:
 *       For each active drop: pick glyph from |slope| of (vx, vy);
 *       draw a motion-blur trail of `length` cells along (-vx, -vy)
 *       with glyphs from the airy ramp tail-to-head.
 *       For each active splash: pick `.`, `,` or `'` based on age
 *       (newer → brighter glyph) and render at its current cell.
 *
 *  6. HUD on bottom row.
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
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DROP WRAP AT SCREEN EDGE. Wind can push a drop off the side
 *    before it reaches the bottom. We let it die quietly off-screen
 *    (no splash) — the `target_drops` self-correct mechanism just
 *    spawns a replacement.
 *
 *  • SPLASH ARRAY EXHAUSTION. When MONSOON kills 50 drops/sec and
 *    each drops spawns 5 splashes = 250 splash spawns/sec, with
 *    splash life of 0.5 sec we have ~125 active splashes. Sized
 *    MAX_SPLASHES = 600 so this never overflows. If pattern is
 *    pushed harder, splash spawns silently fail and the pattern
 *    just looks slightly less wet — graceful degradation.
 *
 *  • TRAIL OFF SCREEN. The motion-blur trail extends BACKWARDS
 *    from the drop head. Above the screen (y < 0) cells are
 *    skipped via bounds check; the visible trail stays visually
 *    intact as the drop enters from the top.
 *
 *  • PAUSE. Respected by both drop and splash tick — `dt = 0` if
 *    paused, so positions and ages don't advance. Drawing
 *    continues unchanged.
 *
 *  • RESEED. Press `r` to clear all drops + splashes and re-spawn
 *    the pattern at full density immediately. Useful when changing
 *    patterns to skip the warm-up period.
 *
 *  • WIND OVERRIDE. `w`/`W` add ±WIND_STEP cells/sec to whatever
 *    the pattern's base wind is. The wind override persists across
 *    pattern changes; press `r` to also reset wind to pattern base.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space). Drops freeze mid-fall; splashes freeze mid-arc.
 *    Resume: motion continues from where it stopped.
 *
 *  • DRIZZLE. Light, slow, sparse. Maybe 100 drops on screen at
 *    once. Each drop falls almost vertically. Splashes are tiny.
 *
 *  • SHOWER. Medium intensity. Some visible wind slant. Drops
 *    obviously diagonal. Splash count noticeably higher.
 *
 *  • STORM. Many drops, fast, strong wind. Drops slant heavily;
 *    motion-blur trails are 3–5 cells long. Splashes burst
 *    visibly along the bottom.
 *
 *  • MONSOON. Wall of rain. Drops fast and dense; whole screen
 *    is moving. Wind is dominant. Splashes form a near-continuous
 *    line at the bottom.
 *
 *  • Wind override (`w`/`W`). Press `w` repeatedly to push the
 *    rain to the right; `W` to push left. The slope of every drop
 *    on screen smoothly reflects the new wind.
 *
 *  • Theme cycle (`t`/`T`). Each theme should produce a
 *    distinctively-coloured rain (blue, silver, neon, etc.) while
 *    drop count and motion are unchanged.
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

    MAX_DROPS        = 1024,
    MAX_SPLASHES     =  800,
    SPLASH_BASE_PER_DROP = 4,

    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,

    /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD         =   1,
    PAIR_HINT        =   2,
    PAIR_RAIN_BASE   =   3,    /* +0..+7 = 8 rain ramp tints (tail→head) */
    PAIR_SPLASH      =  11,
    PAIR_SKY         =  12,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Splash physics (cells / second; cells / second²). */
#define SPLASH_GRAVITY   200.0f
#define SPLASH_KICK_UP    35.0f      /* initial upward velocity        */
#define SPLASH_KICK_X     25.0f      /* ± horizontal kick range        */
#define SPLASH_DRAG        2.0f      /* multiplicative per second      */
#define SPLASH_LIFE_MIN    0.30f
#define SPLASH_LIFE_MAX    0.65f

/* Drop trail spacing (cells per trail step). */
#define TRAIL_SPACING_MIN  0.55f
#define TRAIL_SPACING_MAX  0.85f

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
#define DROP_SPEED_VARIANCE  0.40f
#define DROP_WIND_JITTER     2.0f

/* Wind override step (cells/sec). */
#define WIND_STEP          5.0f

/* Pattern enum. */
typedef enum {
    PATTERN_DRIZZLE = 0,
    PATTERN_SHOWER  = 1,
    PATTERN_STORM   = 2,
    PATTERN_MONSOON = 3,
    N_PATTERNS      = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_DRIZZLE: return "DRIZZLE";
    case PATTERN_SHOWER:  return "SHOWER ";
    case PATTERN_STORM:   return "STORM  ";
    case PATTERN_MONSOON: return "MONSOON";
    default:              return "?      ";
    }
}

/*
 * PatternParams — physics knobs per pattern.
 *
 *   target_drops      : steady-state active drop count
 *   drop_speed        : terminal vy in cells/sec
 *   wind_x            : default horizontal velocity in cells/sec
 *   length_min/max    : motion-blur trail length range in cells
 *   splash_mul        : multiplier on SPLASH_BASE_PER_DROP
 */
typedef struct {
    int   target_drops;
    float drop_speed;
    float wind_x;
    float length_min;
    float length_max;
    float splash_mul;
} PatternParams;

static const PatternParams pattern_params[N_PATTERNS] = {
    /* DRIZZLE  */ { 150,  35.0f,   3.0f, 1.0f, 2.0f, 0.40f },
    /* SHOWER   */ { 240,  75.0f,  12.0f, 2.0f, 3.5f, 0.70f },
    /* STORM    */ { 480, 130.0f,  35.0f, 3.0f, 5.0f, 1.00f },
    /* MONSOON  */ { 800, 190.0f,  60.0f, 4.0f, 6.5f, 1.30f },
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
    short       ramp[8];    /* tail → head */
    short       splash;
    short       sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name          ramp[0..7]                                       splash sky */

    { "DEFAULT",    {  24,  25,  31,  38,  45,  87, 117, 195 },        153,  234 },
    { "STORM",      { 236, 240, 243, 245, 247, 249, 251, 254 },        117,  233 },
    { "TROPICAL",   {  29,  35,  37,  44,  50,  86, 122, 159 },        158,  234 },
    { "ARCTIC",     {  24,  31,  67, 110, 117, 153, 195, 231 },        231,  235 },
    { "DESERT",     { 130, 137, 143, 173, 179, 215, 222, 229 },        222,  234 },
    { "FIRE",       {  88, 124, 130, 166, 196, 208, 214, 226 },        226,  233 },
    { "NEON",       {  53,  91, 134, 165, 201, 207, 213, 219 },        219,  234 },
    { "FOREST",     {  28,  34,  40,  64,  70, 112, 156, 192 },        192,  234 },
    { "MONO",       { 240, 243, 245, 247, 249, 251, 253, 255 },        255,  232 },
    { "VIOLET",     {  53,  54,  91, 134, 135, 176, 213, 219 },        219,  233 },
};

/* Tail-to-head density ramp glyphs (project-airy variant). */
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
            init_pair((short)(PAIR_RAIN_BASE + i), t->ramp[i], -1);
        init_pair(PAIR_SPLASH, t->splash, -1);
        init_pair(PAIR_SKY,    t->sky,    -1);
    } else {
        static const short fb[8] = {
            COLOR_BLUE, COLOR_BLUE,  COLOR_CYAN,  COLOR_CYAN,
            COLOR_CYAN, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAIN_BASE + i), fb[i], -1);
        init_pair(PAIR_SPLASH, COLOR_CYAN,  -1);
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
/* §4  drop — one falling rain particle                                  */
/* ===================================================================== */

typedef struct {
    float x, y;        /* position — cell coords (float)   */
    float vx, vy;      /* velocity — cells / second        */
    float length;      /* motion-blur trail length (cells) */
    float spacing;     /* trail step size (cells)          */
    bool  active;
} Drop;

/* Cheap RNG (LCG) — per-scene state, no global aliasing */
static inline uint32_t lcg_next(uint32_t *st)
{
    *st = *st * 1664525u + 1013904223u;
    return *st;
}
static inline float lcg_unit(uint32_t *st)
{
    return (float)(lcg_next(st) >> 8) / (float)(1u << 24);     /* [0, 1) */
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
static char drop_glyph_for_slope(float vx, float vy)
{
    float ax = fabsf(vx);
    float ay = fabsf(vy);
    if (ax < 1e-3f) return '|';
    float slope = ay / ax;
    if (slope > 4.0f) return '|';
    if (slope < 0.5f) return '~';
    return (vx > 0.0f) ? '\\' : '/';
}

/* ===================================================================== */
/* §5  splash — small particles emitted on drop impact                   */
/* ===================================================================== */

typedef struct {
    float x, y;        /* position — cells     */
    float vx, vy;      /* velocity — cells/sec */
    float age;         /* seconds since spawn  */
    float life;        /* total life (sec)     */
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
    float     wind_override;           /* added to pattern.wind_x */
    uint32_t  rng;
    int       rows, cols;

    Drop      drops    [MAX_DROPS];
    Splash    splashes [MAX_SPLASHES];
} Scene;

static void scene_clear_pools(Scene *s)
{
    for (int i = 0; i < MAX_DROPS;    i++) s->drops[i].active    = false;
    for (int i = 0; i < MAX_SPLASHES; i++) s->splashes[i].active = false;
}

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

/*
 * scene_spawn_drop — activate one inactive drop slot.
 *
 *   y_min, y_max : range from which to draw the spawn y coordinate.
 *
 * Normal top-of-screen spawn uses (−6, −2) so drops enter from above.
 * Pre-warm at init/reseed/pattern-change uses (−6, rows−2) to scatter
 * drops UNIFORMLY across the visible column — this prevents the
 * "starting line" effect where all drops bunch into a thin band at
 * the top and march downward together.
 */
static void scene_spawn_drop(Scene *s, float y_min, float y_max)
{
    int idx = drop_pool_find_inactive(s);
    if (idx < 0) return;
    Drop *d = &s->drops[idx];

    const PatternParams *pp = &pattern_params[s->current_pattern];

    /* Random spawn x covers slightly past the screen on the windward
     * side so wind-tilted streams enter from the side, not just from
     * directly above each column. */
    float wind = pp->wind_x + s->wind_override;
    float rngx = lcg_unit(&s->rng);
    float over = fabsf(wind) * 0.5f;       /* extension on windward edge */
    float spawn_x;
    if (wind > 0.5f) {
        /* wind blowing right — drops can also enter from the left */
        spawn_x = rngx * ((float)s->cols + over) - over;
    } else if (wind < -0.5f) {
        /* wind blowing left — drops can also enter from the right */
        spawn_x = rngx * ((float)s->cols + over);
    } else {
        spawn_x = rngx * (float)s->cols;
    }
    float spawn_y = y_min + lcg_unit(&s->rng) * (y_max - y_min);

    /* Per-drop physics jitter — speed variance is the load-bearing
     * fix for "layered" appearance at slow patterns. */
    float speed_jitter = (1.0f - DROP_SPEED_VARIANCE * 0.5f)
                       + lcg_unit(&s->rng) * DROP_SPEED_VARIANCE;
    float wind_jitter  = (lcg_unit(&s->rng) - 0.5f) * 2.0f * DROP_WIND_JITTER;

    d->x       = spawn_x;
    d->y       = spawn_y;
    d->vx      = wind + wind_jitter;
    d->vy      = pp->drop_speed * speed_jitter;
    d->length  = pp->length_min + lcg_unit(&s->rng) * (pp->length_max - pp->length_min);
    d->spacing = TRAIL_SPACING_MIN + lcg_unit(&s->rng) * (TRAIL_SPACING_MAX - TRAIL_SPACING_MIN);
    d->active  = true;
}

/*
 * scene_prewarm — fill the drop pool to the pattern's target with
 * drops scattered UNIFORMLY across the entire visible y range. Called
 * on init, on reseed, and on pattern change. Without this, drops
 * spawn from the top edge only and the first second of rain looks
 * like a marching-band wave instead of a proper full-screen downpour.
 */
static void scene_prewarm(Scene *s)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    int target = pp->target_drops;
    if (target > MAX_DROPS) target = MAX_DROPS;

    /* Count current active so we only top up — pattern up-shifts
     * (DRIZZLE → STORM) keep the existing drops and just add more. */
    int active = 0;
    for (int i = 0; i < MAX_DROPS; i++)
        if (s->drops[i].active) active++;

    float y_max = (float)(s->rows - 2);
    for (int k = active; k < target; k++)
        scene_spawn_drop(s, -6.0f, y_max);
}

/*
 * scene_emit_splashes — called when a drop hits the bottom. Spawns
 * SPLASH_BASE_PER_DROP × pattern.splash_mul small particles at the
 * impact point with random outward velocity (small upward + ±x kick).
 */
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
    s->current_pattern = PATTERN_SHOWER;
    s->wind_override   = 0.0f;
    s->rng             = (uint32_t)clock_ns();
    s->cols            = cols;
    s->rows            = rows;
    scene_clear_pools(s);
    scene_prewarm(s);     /* fill the screen with rain immediately */
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
    /* Keep existing pools — they self-correct via target_drops. */
}

static void scene_reseed(Scene *s)
{
    s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
    s->wind_override = 0.0f;
    scene_clear_pools(s);
    scene_prewarm(s);     /* re-fill screen with new RNG seed */
}

/*
 * scene_tick — advance physics by dt seconds.
 *  1. Activate-to-target: spawn one new drop per call until pool
 *     hits target. (Limiting to one-per-call avoids big spikes
 *     after a long pause.)
 *  2. Integrate all active drops; kill + splash on bottom contact.
 *  3. Integrate all active splashes; kill on age expiry.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    dt *= speed_mul;

    const PatternParams *pp = &pattern_params[s->current_pattern];

    /* 1. Top up to target. */
    int active_drops = 0;
    for (int i = 0; i < MAX_DROPS; i++)
        if (s->drops[i].active) active_drops++;

    int target = pp->target_drops;
    if (target > MAX_DROPS) target = MAX_DROPS;
    /* Spawn up to (target − active) new drops this tick — but cap the
     * burst size proportional to dt so we don't dump a huge spike
     * after coming out of pause. */
    int spawn_cap = (int)((float)pp->target_drops * dt * 4.0f) + 4;
    int to_spawn  = target - active_drops;
    if (to_spawn > spawn_cap) to_spawn = spawn_cap;
    for (int k = 0; k < to_spawn; k++) scene_spawn_drop(s, -6.0f, -2.0f);

    /* 2. Integrate drops. */
    float kill_y = (float)(s->rows - 2);
    for (int i = 0; i < MAX_DROPS; i++) {
        Drop *d = &s->drops[i];
        if (!d->active) continue;
        d->x += d->vx * dt;
        d->y += d->vy * dt;

        /* Off-screen sideways → die quietly. */
        if (d->x < -8.0f || d->x > (float)(s->cols + 8)) {
            d->active = false;
            continue;
        }
        /* Hit ground → splash. */
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
 * scene_draw — render all active particles to ncurses.
 *
 * Drops: head at (x, y) with the slope-glyph in PAIR_RAIN_BASE+7
 *        (brightest ramp slot); trail BACKWARDS along (-vx, -vy)
 *        with progressively-dimmer ramp slots.
 * Splashes: glyph by remaining life — fresh = '*', mid = '+', old = '.'
 */
static void scene_draw(const Scene *s)
{
    /* ── Drops with motion-blur trails ───────────────────────────── */
    for (int i = 0; i < MAX_DROPS; i++) {
        const Drop *d = &s->drops[i];
        if (!d->active) continue;

        /* Velocity unit vector backwards (for the trail direction). */
        float vlen = sqrtf(d->vx * d->vx + d->vy * d->vy);
        if (vlen < 1e-3f) continue;
        float ux = -d->vx / vlen;
        float uy = -d->vy / vlen;

        char head_glyph = drop_glyph_for_slope(d->vx, d->vy);
        int  trail_n    = (int)d->length;
        if (trail_n < 1) trail_n = 1;
        if (trail_n > 7) trail_n = 7;     /* cap at ramp depth */

        for (int t = 0; t <= trail_n; t++) {
            float px = d->x + ux * d->spacing * (float)t;
            float py = d->y + uy * d->spacing * (float)t;
            int   ix = (int)(px + 0.5f);
            int   iy = (int)(py + 0.5f);
            if (ix < 0 || ix >= s->cols) continue;
            if (iy < 0 || iy >= s->rows - 1) continue;

            /* Ramp slot: head = 7, decreasing along the trail. */
            int ramp_slot = 7 - t;
            if (ramp_slot < 0) ramp_slot = 0;

            char glyph = (t == 0) ? head_glyph : RAMP_GLYPHS[ramp_slot];
            int  attr  = (ramp_slot >= 6) ? A_BOLD
                       : (ramp_slot <= 1) ? A_DIM
                       :                    A_NORMAL;
            int  pair  = PAIR_RAIN_BASE + ramp_slot;

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(iy, ix, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }

    /* ── Splashes ─────────────────────────────────────────────────── */
    for (int i = 0; i < MAX_SPLASHES; i++) {
        const Splash *sp = &s->splashes[i];
        if (!sp->active) continue;
        int ix = (int)(sp->x + 0.5f);
        int iy = (int)(sp->y + 0.5f);
        if (ix < 0 || ix >= s->cols) continue;
        if (iy < 0 || iy >= s->rows - 1) continue;

        float r = sp->age / sp->life;     /* 0=new, 1=dying */
        char  g;
        int   attr;
        if      (r < 0.30f) { g = '*'; attr = A_BOLD;  }
        else if (r < 0.65f) { g = '+'; attr = A_NORMAL;}
        else                { g = '.'; attr = A_DIM;   }

        attron(COLOR_PAIR(PAIR_SPLASH) | attr);
        mvaddch(iy, ix, (chtype)(unsigned char)g);
        attroff(COLOR_PAIR(PAIR_SPLASH) | attr);
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

/* Count active drops/splashes for the HUD. */
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
    const PatternParams *pp = &pattern_params[s->current_pattern];
    float wind = pp->wind_x + s->wind_override;

    const char *state_str = s->paused ? "PAUSED " : pattern_name(s->current_pattern);

    char buf[200];
    snprintf(buf, sizeof buf,
             " RAIN   %s   theme:%-8s   drops:%4d  splashes:%3d  "
             "wind:%+5.1f c/s   %5.1f fps  %3d Hz  speed:%-3d   "
             "n/p:pat  t/T:theme  w/W:wind  +/-:speed  spc:pause  r:reseed  q:quit ",
             state_str, themes[s->current_theme].name,
             drops, spls, (double)wind, fps, sim_fps, s->speed);

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
        scene_prewarm(s);     /* top up to new pattern's target */
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        scene_prewarm(s);
        break;

    case 'w':
        s->wind_override += WIND_STEP;
        break;
    case 'W':
        s->wind_override -= WIND_STEP;
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
