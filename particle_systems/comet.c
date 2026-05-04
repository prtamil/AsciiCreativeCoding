/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * comet.c — moving emitter traces an arc; leaves a fading particle trail
 *
 * DEMO: One or more COMETS spawn from a random screen edge, aimed at
 *       a random point in the opposite quadrant. As each comet
 *       crosses the screen it continuously EMITS trail particles at
 *       its current position. The particles stay (roughly) where
 *       emitted, age, and fade — so as the comet flies on, the
 *       particles it just emitted form a fading streak behind it.
 *       Older particles in the trail are dimmer; the head itself is
 *       a bright glowing glyph plus a small halo.
 *
 *       Patterns:
 *         SHOOTING_STAR  fast, straight, tight bright trail (meteor)
 *         FIREBALL       slower, puffy outward-drifting trail (warm)
 *         PLASMA_BOLT    fast erratic — random angle kicks, electric
 *                        crackling trail
 *
 * Study alongside:
 *   embers.c               — same age-driven cooling palette technique.
 *   rain.c, snow.c, fountain.c, vortex.c — same pool / tick / draw
 *                            framework. Distinct here: the EMITTER moves.
 *
 * Section map:
 *   §1 config   — constants, themes, per-pattern parameters
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 8-pair colour ramp
 *   §4 comet    — Comet struct + spawn + tick (motion + emission)
 *   §5 trail    — TrailParticle struct + tick
 *   §6 scene    — pools, tick, draw, prewarm, reseed
 *   §7 screen   — ncurses init / draw / resize
 *   §8 app      — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reseed (clear pools)
 *   n / N      next pattern   (SHOOTING_STAR → FIREBALL → PLASMA_BOLT)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/comet.c \
 *       -o comet -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Pool-based 2-D particle system with TWO species:
 *                  COMETS (the moving emitters) and TRAIL particles
 *                  (left behind by the comets).
 *
 *                  Each tick:
 *                    1. Spawn comets up to `pattern.max_comets` if any
 *                       are inactive. Spawn from a random screen edge,
 *                       aimed at a random target on the OPPOSITE side.
 *                    2. Integrate each comet's position with explicit
 *                       Euler. PLASMA_BOLT pattern adds a random
 *                       per-tick angular kick (electric crackle).
 *                    3. Each comet emits `emit_rate · dt` new trail
 *                       particles at its current position with small
 *                       perpendicular spread; FIREBALL gives trail
 *                       particles outward velocity for puff feel.
 *                    4. Integrate trail particles: position += vel·dt,
 *                       vel *= drag (per second), age += dt; deactivate
 *                       at age >= life.
 *                    5. Comet leaves screen (with margin) → deactivate.
 *
 *                  Render order:
 *                    A. Trail particles (oldest first → newest last).
 *                    B. Comet heads with a small glowing halo.
 *
 *                  Rendering each trail particle: glyph + colour from
 *                  remaining life (fresh = brightest ramp slot, dying
 *                  = dimmest). Comet head always uses ramp[7] +
 *                  A_BOLD plus a 3-cell halo for "glow" feel.
 *
 *                  Distinct from `embers.c` (free particles rising
 *                  from a stationary source) and from `rain.c`/`snow.c`
 *                  (mass particles falling from a stationary line)
 *                  — here the EMITTER moves, so the trail is laid
 *                  along the emitter's PATH rather than emerging
 *                  from a fixed location.
 *
 * Data-structure : Comet[MAX_COMETS] (typically 1–3 active) +
 *                  TrailParticle[MAX_TRAIL] (a few hundred). Both are
 *                  fixed-size object pools with `active` flags. Linear
 *                  scan to find inactive slot.
 *
 * Rendering      : ASCII only. Trail glyphs from the airy ramp
 *                  `' .,:;-+*'` indexed by remaining life. Comet head
 *                  glyph by speed: `*` for fast, `O` for slow. Head
 *                  halo: 3-cell radial dim glow (`+` `:`).
 *
 * Performance    : O(MAX_COMETS · emission + MAX_TRAIL) per tick.
 *                  At MAX_TRAIL = 600 + 1–3 comets, well under any
 *                  realistic budget.
 *
 * References     :
 *   • Reeves, W. T. (1983) — "Particle Systems: A Technique for
 *     Modelling a Class of Fuzzy Objects", *ACM TOG* 2(2):91–108.
 *     Reeves' original demo was the genesis-planet fireball — the
 *     direct inspiration for the FIREBALL pattern here.
 *   • Wikipedia — [Meteor](https://en.wikipedia.org/wiki/Meteor).
 *     Real meteors trace nearly-straight paths through the atmosphere
 *     leaving an ionisation trail that decays in 1–10 seconds — the
 *     SHOOTING_STAR pattern's parameter set is calibrated against this.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Picture a sparkler waved through the air. The TIP of the sparkler
 * (the comet head) is bright. As you wave it, it spits sparks (the
 * trail particles) along its path. The sparks stay roughly where
 * they were born and slowly fade. The whole image is the bright
 * head plus the cooling streak it left behind.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * The comet is the EMITTER — it has a position and a velocity, and
 * it travels across the screen. Each frame it spits some particles
 * AT ITS CURRENT POSITION. Those particles don't follow the comet
 * — they sit roughly where they were born and fade out over a
 * second or two. So as the comet flies, it pulls a streak of
 * fading particles behind it. Different patterns just change how
 * fast the comet moves, how it curves, how many sparks it sheds,
 * and the colour palette.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. SPAWN COMET. Pick a random screen edge (with a bias against
 *     the bottom). Place the comet just outside that edge at a
 *     random offset. Pick a target point in the opposite-side
 *     quadrant. Velocity = unit vector toward target × speed.
 *
 *  2. INTEGRATE COMET each tick:
 *       comet.x += comet.vx · dt
 *       comet.y += comet.vy · dt
 *       (PLASMA_BOLT only: rotate (vx, vy) by random ±KICK rad)
 *     Deactivate when comet leaves the screen with margin.
 *
 *  3. EMIT TRAIL each tick:
 *       n_emit = comet.emit_carry + emit_rate · dt
 *       emit_count = ⌊n_emit⌋
 *       comet.emit_carry = n_emit − emit_count
 *       for k in 1..emit_count:
 *         spawn TrailParticle at comet position
 *         with perpendicular spread ± particle_spread
 *         velocity = perpendicular kick · spread_drift_factor
 *         life = pattern.particle_life · (0.7 + r · 0.6)
 *
 *  4. INTEGRATE TRAIL each tick:
 *       trail.x += trail.vx · dt
 *       trail.y += trail.vy · dt
 *       trail.vx *= exp(−drag · dt)
 *       trail.vy *= exp(−drag · dt)
 *       trail.age += dt
 *       deactivate when age >= life
 *
 *  5. RENDER:
 *       Trail particles: ramp slot = (1 − age/life) · 7
 *       Comet heads: ramp slot 7 + A_BOLD + 3-cell halo
 *
 *  6. HUD on bottom row.
 *
 * KEY FORMULAS
 * ────────────
 *  Spawn velocity (toward random target on opposite quadrant):
 *    dx = target_x − spawn_x
 *    dy = target_y − spawn_y
 *    len = √(dx² + dy²)
 *    vx = speed · dx / len     vy = speed · dy / len
 *
 *  Plasma-bolt angular kick (per tick):
 *    α = (r − 0.5) · 2 · ANGULAR_KICK
 *    vx' = vx · cos α − vy · sin α
 *    vy' = vx · sin α + vy · cos α
 *
 *  Perpendicular trail spread:
 *    speed = √(vx² + vy²)
 *    perp_x = −vy / speed       perp_y = vx / speed
 *    kick = (r − 0.5) · 2 · particle_spread
 *    trail.x = comet.x + perp_x · kick
 *    trail.y = comet.y + perp_y · kick
 *    trail.vx = perp_x · kick · spread_drift_factor
 *    trail.vy = perp_y · kick · spread_drift_factor
 *
 *  Trail brightness:
 *    f = 1 − trail.age / trail.life       (1 fresh, 0 dying)
 *    ramp_slot = ⌊f · 7⌋
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • EMITTER MOVES BUT TRAIL DOESN'T. Trail particles are emitted
 *    at the comet's CURRENT position with low/zero velocity. As
 *    the comet flies on, those particles stay behind — the streak
 *    is automatic. If the trail particles inherited the comet's
 *    velocity, they'd FOLLOW the comet (no trail visible). This is
 *    the central trick of the moving-emitter pattern.
 *
 *  • EMIT FRACTIONAL CARRY. With emit_rate = 60/sec at 60 fps that's
 *    1.0 emit/tick; at 30 fps that's 2.0. But emit_rate = 90 at
 *    60 fps gives 1.5 — we lose the fractional half each tick.
 *    `comet.emit_carry` accumulates the fractional remainder so the
 *    long-run emission rate stays accurate.
 *
 *  • COMET POOL EXHAUSTION. With max_comets = 2 and a pattern
 *    spawning a new comet whenever a slot frees up, there's never
 *    overflow. The pool just keeps cycling through 2 active comets.
 *
 *  • TRAIL POOL EXHAUSTION. PLASMA_BOLT emits ~150 particles/sec ×
 *    life 0.8 sec ≈ 120 active. With 2 comets that's 240. Pool sized
 *    600 — plenty. If pool fills, emit silently fails — visual just
 *    shows slightly thinner trails.
 *
 *  • PLASMA_BOLT ANGULAR KICK. The kick rotates (vx, vy) using a
 *    proper rotation matrix. Without this, naive `vx += kick` would
 *    change the speed magnitude — the comet would gradually slow or
 *    speed up. Rotation preserves |v|.
 *
 *  • COMET LEAVES OFFSCREEN. We give a margin (2-5 cells) before
 *    deactivating, so the trail emitted at the edge of the screen
 *    has time to extend visibly. Without a margin, trails get cut
 *    off as soon as the comet head crosses the screen edge.
 *
 *  • PAUSE. Both comet and trail integration skip when paused. New
 *    comets won't spawn either (since spawn is conditional on
 *    inactive slots which won't change without integration).
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space). Comet freezes mid-flight, trail particles freeze
 *    in place. Resume: motion continues from where it stopped.
 *
 *  • SHOOTING_STAR. Single fast comet draws a near-straight line
 *    across the screen with a tight bright trail of about 10–20
 *    cells. Reaches the opposite edge in under a second.
 *
 *  • FIREBALL. Slower, with a wider warmer trail that visibly
 *    PUFFS outward perpendicular to the flight direction. Trail
 *    cells live longer (1+ sec) so the streak hangs in the air.
 *
 *  • PLASMA_BOLT. Erratic path — the comet noticeably zigzags as
 *    the random angular kick changes its direction every frame.
 *    Often two bolts visible at once. Trail looks "electric".
 *
 *  • Theme cycle (`t`/`T`). Each theme produces a distinctive
 *    comet colour: ICE (blue/cyan) for SHOOTING_STAR, FIRE (red/
 *    orange) for FIREBALL, PLASMA (purple/cyan) for PLASMA_BOLT.
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

    MAX_COMETS       =   8,
    MAX_TRAIL        = 1000,

    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,

    /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD         =   1,
    PAIR_HINT        =   2,
    PAIR_RAMP_BASE   =   3,    /* +0..+7 = 8 trail tints (cool→hot)   */
    PAIR_HEAD        =  11,    /* always-bright head colour            */
    PAIR_HALO        =  12,    /* head halo                            */
    PAIR_SKY         =  13,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Comet spawn margin (off-screen distance for spawn/kill). */
#define EDGE_MARGIN       3.0f

/* Pattern enum. */
typedef enum {
    PATTERN_SHOOTING_STAR = 0,
    PATTERN_FIREBALL      = 1,
    PATTERN_PLASMA_BOLT   = 2,
    N_PATTERNS            = 3,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_SHOOTING_STAR: return "SHOOTING_STAR";
    case PATTERN_FIREBALL:      return "FIREBALL     ";
    case PATTERN_PLASMA_BOLT:   return "PLASMA_BOLT  ";
    default:                    return "?            ";
    }
}

/*
 * PatternParams — physics + visuals per pattern.
 *
 *   max_comets         : simultaneous active comets
 *   speed              : comet speed in cells/sec
 *   speed_jitter       : ± fraction of speed
 *   angular_kick       : per-tick random rotation (rad)
 *   emit_rate          : trail particles emitted per second per comet
 *   particle_life      : trail particle lifetime (sec)
 *   particle_spread    : ± perpendicular offset at spawn (cells)
 *   spread_drift_factor: how much initial perpendicular kick × becomes
 *                        velocity (0 = static trail; >0 = puff trail)
 *   trail_drag         : per-second velocity damping for trail
 *   head_glyph         : single character at the head
 */
typedef struct {
    int   max_comets;
    float speed;
    float speed_jitter;
    float angular_kick;
    float emit_rate;
    float particle_life;
    float particle_spread;
    float spread_drift_factor;
    float trail_drag;
    char  head_glyph;
} PatternParams;

static const PatternParams pattern_params[N_PATTERNS] = {
    /*                       cnt  spd  jit  ang   emit  life  spread sdrift drag  head */
    /* SHOOTING_STAR    */ { 1, 150.0f, 0.20f, 0.00f,  90.0f, 0.40f, 0.4f,  0.0f, 0.5f, '*' },
    /* FIREBALL         */ { 1,  55.0f, 0.20f, 0.00f, 180.0f, 1.20f, 1.5f,  6.0f, 1.0f, 'O' },
    /* PLASMA_BOLT      */ { 2, 130.0f, 0.30f, 0.10f, 150.0f, 0.80f, 1.0f,  3.0f, 1.5f, '*' },
};

/*
 * Themes — 8-step ramp for the trail (cool/dim → hot/bright). Plus
 * dedicated head and halo colours. All entries sit in the BRIGHT
 * HALF of the 256-colour cube per the CLAUDE.md "Theme Palette
 * Brightness" rule.
 */
typedef struct {
    const char *name;
    short       ramp[8];   /* dying → fresh */
    short       head;      /* always-bright head */
    short       halo;      /* halo around head */
    short       sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name        ramp[0..7]                                       head halo sky */

    { "DEFAULT",  { 110, 117, 153, 159, 195, 195, 231, 255 },        231, 195, 234 },
    { "ICE",      {  24,  31,  67, 110, 117, 153, 195, 231 },        231, 195, 235 },
    { "FIRE",     {  88, 124, 130, 166, 196, 208, 214, 226 },        226, 220, 234 },
    { "PLASMA",   {  53,  91, 134, 165, 207, 213, 219, 225 },        225, 219, 234 },
    { "GOLD",     { 130, 137, 173, 179, 215, 222, 229, 230 },        230, 222, 234 },
    { "GREEN",    {  28,  34,  40,  64,  70, 112, 156, 192 },        192, 156, 234 },
    { "AURORA",   {  43,  79, 115, 121, 157, 195, 230, 231 },        231, 195, 234 },
    { "ROSE",     {  88, 131, 167, 174, 211, 217, 218, 231 },        231, 218, 234 },
    { "MONO",     { 244, 246, 248, 250, 252, 253, 254, 255 },        255, 252, 232 },
    { "VIOLET",   {  53,  54,  91, 134, 135, 176, 213, 219 },        225, 219, 233 },
};

/* Trail glyph ramp (dying → fresh). */
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
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
        init_pair(PAIR_HEAD, t->head, -1);
        init_pair(PAIR_HALO, t->halo, -1);
        init_pair(PAIR_SKY,  t->sky,  -1);
    } else {
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), COLOR_WHITE, -1);
        init_pair(PAIR_HEAD, COLOR_WHITE,  -1);
        init_pair(PAIR_HALO, COLOR_WHITE,  -1);
        init_pair(PAIR_SKY,  COLOR_BLACK,  -1);
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
/* §4  comet                                                              */
/* ===================================================================== */

typedef struct {
    float x, y;
    float vx, vy;
    float emit_carry;     /* fractional emission accumulator           */
    float age;
    bool  active;
} Comet;

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
/* §5  trail particle                                                     */
/* ===================================================================== */

typedef struct {
    float x, y;
    float vx, vy;
    float age, life;
    bool  active;
} TrailParticle;

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

    Comet         comets[MAX_COMETS];
    TrailParticle trail [MAX_TRAIL];
} Scene;

static int comet_pool_find_inactive(Scene *s)
{
    for (int i = 0; i < MAX_COMETS; i++)
        if (!s->comets[i].active) return i;
    return -1;
}

static int trail_pool_find_inactive(Scene *s)
{
    for (int i = 0; i < MAX_TRAIL; i++)
        if (!s->trail[i].active) return i;
    return -1;
}

static void scene_clear_pools(Scene *s)
{
    for (int i = 0; i < MAX_COMETS; i++) s->comets[i].active = false;
    for (int i = 0; i < MAX_TRAIL;  i++) s->trail [i].active = false;
}

/*
 * scene_spawn_comet — pick an edge, spawn just outside it, aim at a
 * random target point on the opposite-side quadrant.
 */
static void scene_spawn_comet(Scene *s)
{
    int idx = comet_pool_find_inactive(s);
    if (idx < 0) return;
    Comet *c = &s->comets[idx];

    const PatternParams *pp = &pattern_params[s->current_pattern];

    /* Pick spawn edge with bias against the bottom (less common). */
    float r_edge = lcg_unit(&s->rng);
    int edge;     /* 0=top, 1=left, 2=right, 3=bottom */
    if (r_edge < 0.45f)      edge = 0;
    else if (r_edge < 0.70f) edge = 1;
    else if (r_edge < 0.95f) edge = 2;
    else                     edge = 3;

    float spawn_x = 0, spawn_y = 0;
    float r1 = lcg_unit(&s->rng);
    switch (edge) {
    case 0: spawn_x = r1 * (float)s->cols; spawn_y = -EDGE_MARGIN; break;
    case 1: spawn_x = -EDGE_MARGIN; spawn_y = r1 * (float)s->rows; break;
    case 2: spawn_x = (float)s->cols + EDGE_MARGIN;
            spawn_y = r1 * (float)s->rows; break;
    case 3: spawn_x = r1 * (float)s->cols;
            spawn_y = (float)s->rows + EDGE_MARGIN; break;
    }

    /* Pick target on the opposite side (with the edge zone biased to
     * the opposite-quadrant area for visible diagonal arcs). */
    float r2 = lcg_unit(&s->rng);
    float r3 = lcg_unit(&s->rng);
    float target_x = 0, target_y = 0;
    switch (edge) {
    case 0: target_x = r2 * (float)s->cols;
            target_y = (float)s->rows * (0.55f + r3 * 0.45f); break;
    case 1: target_x = (float)s->cols * (0.55f + r2 * 0.45f);
            target_y = r3 * (float)s->rows; break;
    case 2: target_x = (float)s->cols * (0.0f + r2 * 0.45f);
            target_y = r3 * (float)s->rows; break;
    case 3: target_x = r2 * (float)s->cols;
            target_y = (float)s->rows * (0.0f + r3 * 0.45f); break;
    }

    float dx = target_x - spawn_x;
    float dy = target_y - spawn_y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 1e-3f) len = 1.0f;

    /* Speed with jitter. */
    float speed = pp->speed
                * ((1.0f - pp->speed_jitter * 0.5f)
                   + lcg_unit(&s->rng) * pp->speed_jitter);

    c->x          = spawn_x;
    c->y          = spawn_y;
    c->vx         = speed * dx / len;
    c->vy         = speed * dy / len;
    c->emit_carry = 0.0f;
    c->age        = 0.0f;
    c->active     = true;
}

/*
 * scene_emit_trail — spawn one trail particle at the comet's current
 * position with a perpendicular spread + drift.
 */
static void scene_emit_trail(Scene *s, const Comet *c)
{
    int idx = trail_pool_find_inactive(s);
    if (idx < 0) return;
    TrailParticle *p = &s->trail[idx];

    const PatternParams *pp = &pattern_params[s->current_pattern];

    float speed = sqrtf(c->vx * c->vx + c->vy * c->vy);
    if (speed < 1e-3f) speed = 1.0f;
    float perp_x = -c->vy / speed;
    float perp_y =  c->vx / speed;

    float kick = (lcg_unit(&s->rng) - 0.5f) * 2.0f * pp->particle_spread;

    float r_life = lcg_unit(&s->rng);

    p->x      = c->x + perp_x * kick;
    p->y      = c->y + perp_y * kick;
    p->vx     = perp_x * kick * pp->spread_drift_factor;
    p->vy     = perp_y * kick * pp->spread_drift_factor;
    p->age    = 0.0f;
    p->life   = pp->particle_life * (0.7f + r_life * 0.6f);
    p->active = true;
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_SHOOTING_STAR;
    s->rng             = (uint32_t)clock_ns();
    s->cols            = cols;
    s->rows            = rows;
    scene_clear_pools(s);
    /* Seed the first comet so the screen is not empty. */
    scene_spawn_comet(s);
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
    scene_spawn_comet(s);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    dt *= speed_mul;

    const PatternParams *pp = &pattern_params[s->current_pattern];

    /* 1. Top up comets to max. */
    int active_comets = 0;
    for (int i = 0; i < MAX_COMETS; i++) if (s->comets[i].active) active_comets++;
    int target_comets = pp->max_comets;
    if (target_comets > MAX_COMETS) target_comets = MAX_COMETS;
    for (int k = active_comets; k < target_comets; k++) scene_spawn_comet(s);

    /* 2. Integrate comets (motion + emission + plasma kick). */
    for (int i = 0; i < MAX_COMETS; i++) {
        Comet *c = &s->comets[i];
        if (!c->active) continue;

        /* Plasma-bolt: random rotation of velocity vector. */
        if (pp->angular_kick > 0.0f) {
            float ang = (lcg_unit(&s->rng) - 0.5f) * 2.0f * pp->angular_kick;
            float ca = cosf(ang);
            float sa = sinf(ang);
            float nvx = ca * c->vx - sa * c->vy;
            float nvy = sa * c->vx + ca * c->vy;
            c->vx = nvx;
            c->vy = nvy;
        }

        /* Position integration. */
        c->x += c->vx * dt;
        c->y += c->vy * dt;
        c->age += dt;

        /* Off-screen with margin → die. */
        if (c->x < -EDGE_MARGIN || c->x > (float)s->cols + EDGE_MARGIN ||
            c->y < -EDGE_MARGIN || c->y > (float)s->rows + EDGE_MARGIN) {
            c->active = false;
            continue;
        }

        /* Trail emission with fractional carry. */
        c->emit_carry += pp->emit_rate * dt;
        int n_emit = (int)c->emit_carry;
        c->emit_carry -= (float)n_emit;
        for (int k = 0; k < n_emit; k++) scene_emit_trail(s, c);
    }

    /* 3. Integrate trail particles. */
    float drag_factor = expf(-pp->trail_drag * dt);
    for (int i = 0; i < MAX_TRAIL; i++) {
        TrailParticle *p = &s->trail[i];
        if (!p->active) continue;
        p->x  += p->vx * dt;
        p->y  += p->vy * dt;
        p->vx *= drag_factor;
        p->vy *= drag_factor;
        p->age += dt;
        if (p->age >= p->life) p->active = false;
    }
}

/*
 * scene_draw — trail particles first (oldest under newest), then
 * comet heads with halos on top.
 */
static void scene_draw(const Scene *s)
{
    int rows_eff = s->rows - 1;     /* leave bottom row for HUD */

    /* ── Trail particles ─────────────────────────────────────────── */
    for (int i = 0; i < MAX_TRAIL; i++) {
        const TrailParticle *p = &s->trail[i];
        if (!p->active) continue;
        int ix = (int)(p->x + 0.5f);
        int iy = (int)(p->y + 0.5f);
        if (ix < 0 || ix >= s->cols) continue;
        if (iy < 0 || iy >= rows_eff) continue;

        float f = 1.0f - p->age / p->life;
        if (f < 0.0f) f = 0.0f;
        if (f > 1.0f) f = 1.0f;
        int slot = (int)(f * 7.999f);
        if (slot < 0) slot = 0;
        if (slot > 7) slot = 7;

        char glyph = RAMP_GLYPHS[slot];
        int  attr  = (slot >= 6) ? A_BOLD
                   : (slot <= 1) ? A_DIM
                   :               A_NORMAL;
        int  pair  = PAIR_RAMP_BASE + slot;
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(iy, ix, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | attr);
    }

    /* ── Comet heads with halo ───────────────────────────────────── */
    const PatternParams *pp = &pattern_params[s->current_pattern];
    for (int i = 0; i < MAX_COMETS; i++) {
        const Comet *c = &s->comets[i];
        if (!c->active) continue;
        int hx = (int)(c->x + 0.5f);
        int hy = (int)(c->y + 0.5f);

        /* Halo: 3-cell radial dim glow around head. */
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;
                int hxd = hx + dx;
                int hyd = hy + dy;
                if (hxd < 0 || hxd >= s->cols) continue;
                if (hyd < 0 || hyd >= rows_eff) continue;
                /* Inner ring (4-neighbour) is brighter than corners. */
                int ax = (dx == 0 || dy == 0);
                char hg = ax ? '+' : ':';
                int  hat = ax ? A_BOLD : A_DIM;
                attron(COLOR_PAIR(PAIR_HALO) | hat);
                mvaddch(hyd, hxd, (chtype)(unsigned char)hg);
                attroff(COLOR_PAIR(PAIR_HALO) | hat);
            }
        }

        /* Head itself. */
        if (hx >= 0 && hx < s->cols && hy >= 0 && hy < rows_eff) {
            attron(COLOR_PAIR(PAIR_HEAD) | A_BOLD);
            mvaddch(hy, hx, (chtype)(unsigned char)pp->head_glyph);
            attroff(COLOR_PAIR(PAIR_HEAD) | A_BOLD);
        }
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

static void scene_counts(const Scene *s, int *out_comets, int *out_trail)
{
    int c = 0, t = 0;
    for (int i = 0; i < MAX_COMETS; i++) if (s->comets[i].active) c++;
    for (int i = 0; i < MAX_TRAIL;  i++) if (s->trail [i].active) t++;
    *out_comets = c;
    *out_trail  = t;
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s);

    int comets, trails;
    scene_counts(s, &comets, &trails);

    const char *state_str = s->paused ? "PAUSED       " : pattern_name(s->current_pattern);

    char buf[200];
    snprintf(buf, sizeof buf,
             " COMET   %s   theme:%-8s   comets:%d  trail:%4d   "
             "%5.1f fps  %3d Hz  speed:%-3d   "
             "n/p:pat  t/T:theme  +/-:speed  spc:pause  r:reseed  q:quit ",
             state_str, themes[s->current_theme].name,
             comets, trails, fps, sim_fps, s->speed);

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
