/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * comet.c — moving-emitter comets that drop a fading trail and detonate on impact.
 *
 * Comets enter from a random screen edge aimed at a random point in
 * the opposite quadrant.  As each comet flies it spawns trail
 * particles AT ITS CURRENT POSITION (without inheriting velocity), so
 * the trail "falls behind" as a tapered streak coloured through an
 * 8-step theme ramp.  When a comet crosses the floor heading down it
 * detonates: a one-frame '*+' flash followed by a 32-spark wave-
 * staggered radial fan that drags to a halt over ~0.6 s.  Three
 * patterns: SHOOTING_STAR, FIREBALL, PLASMA_BOLT.  10 themes cycle
 * the colour-pair IDs without redrawing.
 *
 * Section map
 *   §1 config       sim/pattern/blast constants + theme table
 *   §2 clock        monotonic-ns timer + sleep
 *   §3 color        palette + theme cycle
 *   §4 comet        Comet struct + LCG + render-time named primitives
 *   §5 trail/blast  TrailParticle, BlastParticle, Blast structs
 *   §6 scene        pools, lifecycle, tick phases, draw layers
 *   §7 screen       ncurses init/draw/resize, HUD bar
 *   §8 app          signals, fixed-step main loop
 */

/* ── CONCEPTS & ALGORITHMS ───────────────────────────────────────────── *
 *
 *   Moving-emitter trick   Emitter moves; particles spawn at its CURRENT
 *                          position with NO inherited velocity → trail
 *                          falls behind naturally as a tapered streak.
 *                          → Reeves (1983), ACM TOG 2(2): 91
 *
 *   Bresenham accumulator  Fractional emission carry: emit_carry +=
 *                          rate · dt; spawn floor(carry); keep remainder.
 *                          → Bresenham (1965); standard DDA pattern
 *
 *   Polar emission         Radial fan from a centre: vel = (cos α, sin α)·s.
 *                          Used for impact-blast wave-staggered emission.
 *
 *   2-D rotation matrix    PLASMA_BOLT angular kick preserves |v|; naive
 *                          additive jitter would change speed magnitude.
 *                          → Foley et al. §5.6
 *
 *   Exponential drag       v *= exp(-rate · dt); analytic per-frame form,
 *                          stable under any dt.
 *
 *   Object pool            Three fixed-size pools (comets/trail/blasts)
 *                          with `active` flag + linear-scan find-free-slot.
 *                          No malloc after init.
 *
 *   Painter's algorithm    3 z-ordered layers: trail (background) →
 *                          blasts (mid) → comets (foreground).
 *                          → Newell, Newell & Sancha (1972)
 *
 *   Fixed-step loop        Deterministic physics; render Hz independent.
 *                          → Glenn Fiedler, "Fix Your Timestep!" (2004)
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── OVERALL PSEUDOCODE ──────────────────────────────────────────────── *
 *
 *   init:
 *     install signals; start ncurses (themes + HUD/HINT pairs)
 *     scene_init: clear pools; spawn ONE seed comet so screen is not empty
 *
 *   loop while running:
 *     if need_resize:        endwin → refresh → re-query terminal extent
 *     dt = clock_now − last_frame_time      (capped at 100 ms)
 *     dt *= speed_mul                       (user "+/-" time dial)
 *     while sim_accum ≥ tick_ns:
 *       scene_tick(dt_sec):  4 phases (top-up + advance + trail + blasts)
 *       sim_accum -= tick_ns
 *     scene_draw:            3-layer painter (trail, blasts, comets)
 *     screen_draw HUD bottom-bar: pattern + theme + counts + fps + speed
 *     screen_present
 *     getch + app_handle_key q/spc/r/n/p/t/T/+/-/]/[
 *     sleep to ~60 fps
 *
 *   cleanup:
 *     atexit endwin restores the terminal
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── DRIVER PSEUDOCODE ───────────────────────────────────────────────── *
 *
 * scene_tick(dt):                              (per-frame physics, 4 phases)
 *   if paused:  return
 *   dt *= speed / SPEED_DEF                    user time dial
 *   pp ← pattern_params[current_pattern]
 *   phase1_topup_comet_pool   (s, pp)          spawn until pool reaches max
 *   phase2_advance_all_comets (s, dt, pp)      kick → integrate → die → emit
 *   phase3_advance_trail      (s, dt, pp)      drag + Euler + age
 *   phase4_advance_all_blasts (s, dt)          flash TTL + per-spark tick
 *
 *   Phase order matters: 1 produces actors, 2 reads them and emits trail,
 *   3 ages the just-emitted trail, 4 is independent and runs last.
 *
 *
 * scene_draw(s):                               (3-layer painter's algorithm)
 *   rows_playable ← rows - 1                   bottom row reserved for HUD
 *   scene_draw_trail_layer  (s)                LAYER A (background)
 *   scene_draw_blast_layer  (s)                LAYER B (mid-ground)
 *   scene_draw_comet_layer  (s)                LAYER C (foreground; on top)
 *
 *   Layer order is z-by-category, not by depth.  Trails are "the past",
 *   comets are "the present" — comets win every cell collision.
 *
 *
 * scene_emit_trail(s, c):                      (THE moving-emitter trick)
 *   slot ← trail_pool_find_inactive             bail if pool full
 *   speed ← |c->vel|; if < 1e-3: speed = 1.0   unit-vector safety
 *   perp ← (-vy/speed, vx/speed)                90° CCW rotation
 *   kick ← uniform(-1, 1) · 2 · particle_spread
 *   p->pos ← c->pos + perp · kick               offset along perpendicular
 *   p->vel ← perp · kick · spread_drift_factor  drift (NOT inherited from c)
 *   p->life ← particle_life · uniform(0.7, 1.3)
 *
 *   The decoupling of p->vel from c->vel is the mechanism that makes
 *   the trail "fall behind" — break it and the trail flies along with
 *   the comet (no streak).
 *
 *
 * blast_ignite(s, cx, cy):                     (impact-blast spawn)
 *   slot ← blast_pool_find_inactive             bail if pool full
 *   b->centre ← (cx, cy);  b->flash_ttl ← BLAST_FLASH_SEC
 *   for i = 0..BLAST_PARTICLES-1:
 *     angle  = blast_emission_angle(i, N, rng)  ∈ [π, 2π] (UPPER hemisphere)
 *     speed  = blast_emission_speed(rng)
 *     wave   = i mod BLAST_WAVE_COUNT
 *     delay  = blast_wave_delay(wave, count, max)
 *     blast_spawn_one_spark(b->parts[i], angle, speed, delay, rng)
 *
 *   Angle restricted to [π, 2π] so sparks rise/spread sideways (sin≤0
 *   in screen coords).  Wave-stagger gives the burst a shockwave reading
 *   instead of a single instantaneous ring.
 *
 *
 * scene_spawn_comet(s):                        (edge-pick + opposite-aim)
 *   slot ← comet_pool_find_inactive             bail if pool full
 *   edge ← pick_spawn_edge(rng)                 weighted: TOP 45 / L 25 / R 25 / BOT 5
 *   spawn_pt  ← compute_spawn_point(edge)       just OUTSIDE that edge
 *   target_pt ← compute_target_point(edge)      opposite-quadrant band
 *   d ← target_pt - spawn_pt
 *   speed ← compute_speed_with_jitter(pp)
 *   c->pos ← spawn_pt
 *   c->vel ← d / |d| · speed                    unit-vector toward target
 *
 *   Aiming at the opposite quadrant guarantees a diagonal arc that
 *   visibly crosses the screen.  Uniform-direction random vectors
 *   would half the time send the comet back out the edge it entered on.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/*
 * Keys
 *   q | Q | ESC      quit                spc        pause / resume
 *   r                reseed pools (clear + spawn one fresh comet)
 *   n / N            next pattern        p / P      prev pattern
 *   t / T            next / prev theme
 *   + / =            speed × 2           -          speed ÷ 2
 *   ] / [            raise / lower sim_fps
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/comet.c \
 *       -o comet -lncurses -lm
 */

/* ── REFERENCES ──────────────────────────────────────────────────────── *
 *
 * PAPERS
 *   Reeves, W. T. (1983)
 *     "Particle Systems: A Technique for Modelling a Class of Fuzzy Objects"
 *     ACM Transactions on Graphics 2(2): 91–108.
 *   Bresenham, J. E. (1965)
 *     "Algorithm for computer control of a digital plotter"
 *     IBM Systems Journal 4(1): 25–30.   (Fractional accumulator pattern.)
 *   Newell, M. E., Newell, R. G. & Sancha, T. L. (1972)
 *     "A solution to the hidden surface problem"
 *     Proc. ACM Annual Conference: 443–450.   (Painter's algorithm.)
 *
 * BOOKS
 *   Foley, J. et al. — "Computer Graphics: Principles and Practice"
 *     (Addison-Wesley, 2nd ed. 1990)
 *     §5.6 covers 2-D rotation matrices; §17.x particle-system mechanics.
 *   Press et al. — "Numerical Recipes in C"  (3rd ed., 2007)
 *     §17.x fixed-step integration; §7.1.2 LCG random number primitives.
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
    MAX_BLASTS       =   8,    /* one per comet possible at impact */

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

/* Cells of slack outside the screen before a spawned/dead comet is killed. */
#define EDGE_MARGIN       3.0f

/* Impact-blast tuning (seconds-based; drag 3.0/s ≈ 95% loss/s, 4 waves @ 0.1s spread). */
#define BLAST_PARTICLES         32     /* sparks per blast */
#define BLAST_FLASH_SEC         0.06f  /* central '*+' flash duration */
#define BLAST_LIFE_BASE         0.50f  /* baseline spark lifetime */
#define BLAST_LIFE_JITTER       0.25f  /* + uniform[0, this] for variety */
#define BLAST_SPEED_MIN        18.0f   /* initial spark speed lower bound */
#define BLAST_SPEED_MAX        42.0f   /* initial spark speed upper bound */
#define BLAST_DRAG_PER_SEC      3.0f   /* exponential vel decay rate */
#define BLAST_WAVE_COUNT        4      /* shockwave rings */
#define BLAST_MAX_DELAY_SEC     0.10f  /* delay of last wave behind wave 0 */

typedef enum {
    PATTERN_SHOOTING_STAR = 0,
    PATTERN_FIREBALL      = 1,
    PATTERN_PLASMA_BOLT   = 2,
    N_PATTERNS            = 3,
} Pattern;

/* pattern_name — HUD label for the active pattern (padded to fixed width). */
static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_SHOOTING_STAR: return "SHOOTING_STAR";
    case PATTERN_FIREBALL:      return "FIREBALL     ";
    case PATTERN_PLASMA_BOLT:   return "PLASMA_BOLT  ";
    default:                    return "?            ";
    }
}

typedef struct {
    int   max_comets;          /* simultaneous active comets */
    float speed;               /* comet speed (cells/sec) */
    float speed_jitter;        /* ± fraction of speed */
    float angular_kick;        /* per-tick random rotation (rad) */
    float emit_rate;           /* trail particles emitted per second */
    float particle_life;       /* trail particle lifetime (sec) */
    float particle_spread;     /* ± perpendicular spawn offset (cells) */
    float spread_drift_factor; /* perp-kick · this becomes vel; 0 = static */
    float trail_drag;          /* per-second velocity damping for trail */
    char  head_glyph;          /* single character drawn at the comet head */
} PatternParams;

static const PatternParams pattern_params[N_PATTERNS] = {
    /*                       cnt  spd  jit  ang   emit  life  spread sdrift drag  head */
    /* SHOOTING_STAR    */ { 1, 150.0f, 0.20f, 0.00f,  90.0f, 0.40f, 0.4f,  0.0f, 0.5f, '*' },
    /* FIREBALL         */ { 1,  55.0f, 0.20f, 0.00f, 180.0f, 1.20f, 1.5f,  6.0f, 1.0f, 'O' },
    /* PLASMA_BOLT      */ { 2, 130.0f, 0.30f, 0.10f, 150.0f, 0.80f, 1.0f,  3.0f, 1.5f, '*' },
};

typedef struct {
    const char *name;
    short       ramp[8];   /* slot 0 = dying/dim → slot 7 = fresh/bright */
    short       head;      /* always-bright head colour */
    short       halo;      /* halo around the head */
    short       sky;       /* (reserved; not currently rendered) */
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
/* Two POSIX-clock wrappers used by the main loop: a monotonic ns timer
 * and a non-blocking sleep that no-ops on negative durations. */

/* clock_ns — current monotonic time in nanoseconds. */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/* clock_sleep_ns — sleep ns nanoseconds; safe to call with ns <= 0. */
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

/* theme_apply — rebind the 8 ramp pairs + head/halo/sky for theme[idx]. */
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

/* color_init — start_color + pin HUD/HINT pairs + apply theme 0. */
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

/*
 * Comet — the moving emitter.  No glyph or colour stored here; those come
 * from pattern_params + themes at draw time, so any (pattern × theme)
 * combination works without respawning.
 *
 *   x, y         centre in cells (float for sub-cell motion)
 *   vx, vy       velocity in cells/sec
 *   emit_carry   Bresenham fractional accumulator for trail emission
 *   age          seconds since spawn (HUD-only; doesn't drive death)
 *   active       pool slot in use?
 *
 * Lifecycle: scene_spawn_comet at startup / 'r' / pool top-up →
 *   phase2_advance_one_comet per tick (kick + integrate + die-check + emit) →
 *   active=false on ground impact OR off-screen.
 */
typedef struct {
    float x, y;
    float vx, vy;
    float emit_carry;
    float age;
    bool  active;
} Comet;

/* lcg_next — Numerical Recipes LCG; one mul, one add per call. */
static inline uint32_t lcg_next(uint32_t *st)
{
    *st = *st * 1664525u + 1013904223u;
    return *st;
}

/* lcg_unit — uniform float in [0, 1).  Top 24 bits → 24-bit precision. */
static inline float lcg_unit(uint32_t *st)
{
    return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/* lcg_signed — uniform in [-0.5, 0.5). */
static inline float lcg_signed(uint32_t *rng)        { return lcg_unit(rng) - 0.5f; }
/* lcg_range — uniform in [lo, hi). */
static inline float lcg_range (uint32_t *rng,
                               float lo, float hi)   { return lo + lcg_unit(rng) * (hi - lo); }
/* clamp_int / clampf — clamp value to closed interval [lo, hi]. */
static inline int   clamp_int (int v, int lo, int hi){ return v < lo ? lo : (v > hi ? hi : v); }
static inline float clampf    (float v, float lo,
                               float hi)             { return v < lo ? lo : (v > hi ? hi : v); }

/* ramp_slot_from_freshness — clamp(floor(freshness · 7.999), 0, 7).  The
 * 7.999 multiplier (not 8) prevents freshness=1 from rounding to slot 8. */
static inline int ramp_slot_from_freshness(float freshness_0_to_1)
{
    float clamped = clampf(freshness_0_to_1, 0.0f, 1.0f);
    int   bucket  = (int)(clamped * 7.999f);
    return clamp_int(bucket, 0, 7);
}

/* round_to_cell — float position → integer cell index (round half up). */
static inline int round_to_cell(float v)
{
    return (int)(v + 0.5f);
}

/* cell_visible — true if (cell_x, cell_y) is inside the playable area. */
static inline bool cell_visible(int cell_x, int cell_y, int cols, int rows_playable)
{
    return cell_x >= 0 && cell_x <  cols
        && cell_y >= 0 && cell_y <  rows_playable;
}

/* trail_freshness — 1.0 at birth, linearly down to 0.0 at age == life. */
static inline float trail_freshness(float age, float life)
{
    if (life <= 0.0f) return 0.0f;
    return 1.0f - age / life;
}

/* blast_freshness — 1.0 at birth, 0.0 at death (life counts DOWN). */
static inline float blast_freshness(float remaining_life, float max_life)
{
    if (max_life <= 0.0f) return 0.0f;
    return remaining_life / max_life;
}

/* trail_attr_for_slot — slot 6+ BOLD, slot 0/1 DIM, else NORMAL. */
static inline int trail_attr_for_slot(int ramp_slot)
{
    if (ramp_slot >= 6) return A_BOLD;
    if (ramp_slot <= 1) return A_DIM;
    return A_NORMAL;
}

/* blast_attr_for_slot — slot 6+ BOLD, else NORMAL.  No DIM tier:
 * a faint blast spark reads as a smudge, so they vanish instead of fading. */
static inline int blast_attr_for_slot(int ramp_slot)
{
    return (ramp_slot >= 6) ? A_BOLD : A_NORMAL;
}

/* paint_cell — one mvaddch wrapped in attron/off so callers stay one-liners. */
static inline void paint_cell(int cell_y, int cell_x, char glyph,
                              int color_pair, int attr)
{
    attron(COLOR_PAIR(color_pair) | attr);
    mvaddch(cell_y, cell_x, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(color_pair) | attr);
}

/* ===================================================================== */
/* §5  trail + blast particles                                            */
/* ===================================================================== */

/*
 * TrailParticle — fading dot left behind by a moving comet.
 *
 *   x, y         ABSOLUTE position in cells (NOT relative to comet)
 *   vx, vy       drift velocity in cells/sec (NOT inherited from comet)
 *   age, life    age counts UP; particle dies when age ≥ life
 *   active       pool slot in use?
 *
 * Lifecycle: scene_emit_trail spawns ~emit_rate per second per active
 *   comet → phase3_advance_trail per tick (drag + Euler + age) →
 *   active=false at age ≥ life.
 */
typedef struct {
    float x, y;
    float vx, vy;
    float age, life;
    bool  active;
} TrailParticle;

/*
 * BlastParticle — one spark in an impact blast.
 *
 *   rx, ry       OFFSET from parent Blast centre, in cells (NOT absolute)
 *   vx, vy       velocity in cells/sec
 *   life         remaining seconds; counts DOWN to zero
 *   max_life     original life at spawn (for freshness ratio = life / max_life)
 *   delay        wave-stagger countdown in seconds; spark gated until 0
 *   sym          glyph chosen at spawn from "*+.,oO!#" — fixed for spark's life
 *   alive        pool slot in use?
 *
 * Position is OFFSET (not absolute) so the parent Blast owns one centre
 * and all 32 sparks compose against it.  Centralises bounds-clipping
 * in the draw layer.
 */
typedef struct {
    float rx, ry;
    float vx, vy;
    float life;
    float max_life;
    float delay;
    char  sym;
    bool  alive;
} BlastParticle;

/*
 * Blast — one impact event: central '*+' flash + 32-spark radial fan.
 *
 *   cx, cy                impact centre in cells
 *   flash_ttl             '*+' cross seconds remaining; 0 = no flash
 *   active                pool slot in use?
 *   parts[BLAST_PARTICLES] inline pool — Blast OWNS its 32 sparks
 *
 * Lifecycle: blast_ignite called the frame a comet crosses the floor →
 *   phase4_advance_one_blast per tick (flash TTL + per-spark tick) →
 *   active=false once flash has expired AND every spark is dead.
 */
typedef struct {
    float         cx, cy;
    float         flash_ttl;
    bool          active;
    BlastParticle parts[BLAST_PARTICLES];
} Blast;

/* ===================================================================== */
/* §6  scene — pools, tick, draw                                         */
/* ===================================================================== */

/*
 * Scene — owns three fixed-size pools + run-time knobs.  Whole simulation
 * state in one struct.  No malloc after init; no other dynamic state.
 *
 *   paused              if true, scene_tick early-returns
 *   speed               time-dial multiplier; SPEED_DEF = 1.0×
 *   current_theme       index into themes[] (cycled by t/T)
 *   current_pattern     index into pattern_params[] (cycled by n/p)
 *   rng                 LCG state — single source of randomness
 *   rows, cols          terminal extent in cells
 *   comets[MAX_COMETS=8]      moving emitter pool
 *   trail [MAX_TRAIL=1000]    fading trail pool
 *   blasts[MAX_BLASTS=8]      one impact-blast slot per possible comet
 *
 * Lifecycle: scene_init at startup → scene_tick + scene_draw per frame →
 *   scene_resize on SIGWINCH (no realloc; pools are fixed) →
 *   scene_reseed on 'r' (clear + spawn one).  Pool find_inactive helpers
 *   do an O(n) scan; at sizes 8/1000/8 this is cheaper than a free-list.
 */
typedef struct {
    bool      paused;
    int       speed;
    int       current_theme;
    Pattern   current_pattern;
    uint32_t  rng;
    int       rows, cols;

    Comet         comets[MAX_COMETS];
    TrailParticle trail [MAX_TRAIL];
    Blast         blasts[MAX_BLASTS];
} Scene;

/* {comet,trail,blast}_pool_find_inactive — O(n) scan; returns slot index or -1. */
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

static int blast_pool_find_inactive(Scene *s)
{
    for (int i = 0; i < MAX_BLASTS; i++)
        if (!s->blasts[i].active) return i;
    return -1;
}

/* scene_clear_pools — flag every slot inactive; O(MAX_TRAIL) dominated. */
static void scene_clear_pools(Scene *s)
{
    for (int i = 0; i < MAX_COMETS; i++) s->comets[i].active = false;
    for (int i = 0; i < MAX_TRAIL;  i++) s->trail [i].active = false;
    for (int i = 0; i < MAX_BLASTS; i++) s->blasts[i].active = false;
}

/* blast_emission_angle — π + (i/N)·π + jitter ∈ [π, 2π], upper hemisphere. */
static float blast_emission_angle(int i, int total, uint32_t *rng)
{
    float evenly_spaced = (float)M_PI + ((float)i / (float)total) * (float)M_PI;
    float jitter        = lcg_signed(rng) * 0.2f;
    return evenly_spaced + jitter;
}

/* blast_emission_speed — uniform in [BLAST_SPEED_MIN, BLAST_SPEED_MAX). */
static float blast_emission_speed(uint32_t *rng)
{
    return lcg_range(rng, BLAST_SPEED_MIN, BLAST_SPEED_MAX);
}

/* blast_wave_delay — staggered spawn delay for shockwave reading. */
static float blast_wave_delay(int wave, int wave_count, float max_delay_sec)
{
    if (wave_count <= 1) return 0.0f;
    return (float)wave * (max_delay_sec / (float)(wave_count - 1));
}

/* blast_spawn_one_spark — fill one BlastParticle: pos=0, polar→Cartesian vel, life, sym. */
static void blast_spawn_one_spark(BlastParticle *p, float angle, float speed,
                                  float delay_sec, uint32_t *rng)
{
    static const char k_blast_syms[] = "*+.,oO!#";
    static const int  n_syms         = (int)sizeof k_blast_syms - 1;

    p->rx       = 0.0f;
    p->ry       = 0.0f;
    p->vx       = cosf(angle) * speed;
    p->vy       = sinf(angle) * speed;
    p->max_life = lcg_range(rng, BLAST_LIFE_BASE, BLAST_LIFE_BASE + BLAST_LIFE_JITTER);
    p->life     = p->max_life;
    p->delay    = delay_sec;
    p->sym      = k_blast_syms[lcg_next(rng) % (unsigned)n_syms];
    p->alive    = true;
}

/*
 * blast_ignite — DRIVER.  Spawn one ground-impact blast at (cx, cy).
 *
 *   Step 1   slot ← blast_pool_find_inactive            bail if pool full
 *   Step 2   b->centre ← (cx, cy);  b->flash_ttl ← BLAST_FLASH_SEC; active=true
 *   Step 3   for i = 0..BLAST_PARTICLES-1:
 *              angle = blast_emission_angle(i, N, rng)   ∈ [π, 2π]  (UPPER hemi)
 *              speed = blast_emission_speed(rng)
 *              wave  = i mod BLAST_WAVE_COUNT
 *              delay = blast_wave_delay(wave, count, max)
 *              blast_spawn_one_spark(&parts[i], angle, speed, delay, rng)
 *
 * INPUTS / UNITS
 *   s          Scene mutated; one Blast slot becomes active.
 *   cx, cy     impact centre in cells (cy is the floor row, rows-2).
 *
 * WHY UPPER-HEMISPHERE ONLY
 *   In screen coordinates +y is DOWN.  Sparks aimed downward (angles
 *   in [0, π]) would immediately leave the playable area below the floor.
 *   Restricting to [π, 2π] gives sin(angle) ≤ 0 → vy ≤ 0 → all sparks
 *   rise or fly sideways.
 *
 * WHY WAVE STAGGER
 *   Without staggered delays, all 32 sparks fire in the same frame and
 *   the burst reads as a single instantaneous ring.  4 waves with
 *   delays 0/0.033/0.067/0.10 s give the burst a "shockwave" reading
 *   — a leading ring expands while a second ring is still spawning.
 */
static void blast_ignite(Scene *s, float cx, float cy)
{
    /* Step 1 — locate an inactive Blast slot; bail if pool full. */
    int slot_index = blast_pool_find_inactive(s);
    if (slot_index < 0) return;
    Blast *b = &s->blasts[slot_index];

    /* Step 2 — initialise centre + central-flash TTL. */
    b->cx        = cx;
    b->cy        = cy;
    b->flash_ttl = BLAST_FLASH_SEC;
    b->active    = true;

    /* Step 3 — spawn the upper-hemisphere spark fan, wave-staggered. */
    for (int i = 0; i < BLAST_PARTICLES; i++) {
        float angle      = blast_emission_angle(i, BLAST_PARTICLES, &s->rng);
        float speed      = blast_emission_speed(&s->rng);
        int   wave_index = i % BLAST_WAVE_COUNT;
        float delay_sec  = blast_wave_delay(wave_index, BLAST_WAVE_COUNT,
                                            BLAST_MAX_DELAY_SEC);
        blast_spawn_one_spark(&b->parts[i], angle, speed, delay_sec, &s->rng);
    }
}

/* SpawnEdge — which side of the screen a fresh comet enters from. */
typedef enum {
    EDGE_TOP    = 0,
    EDGE_LEFT   = 1,
    EDGE_RIGHT  = 2,
    EDGE_BOTTOM = 3,
} SpawnEdge;

/* pick_spawn_edge — weighted: TOP 45% / LEFT 25% / RIGHT 25% / BOTTOM 5%. */
static SpawnEdge pick_spawn_edge(uint32_t *rng)
{
    float u = lcg_unit(rng);
    if (u < 0.45f) return EDGE_TOP;
    if (u < 0.70f) return EDGE_LEFT;
    if (u < 0.95f) return EDGE_RIGHT;
    return EDGE_BOTTOM;
}

/* compute_spawn_point — (x, y) just OUTSIDE chosen edge so entry reads "from off-screen". */
static void compute_spawn_point(SpawnEdge edge, int cols, int rows,
                                uint32_t *rng,
                                float *spawn_x, float *spawn_y)
{
    float along_edge = lcg_unit(rng);
    switch (edge) {
    case EDGE_TOP:
        *spawn_x = along_edge * (float)cols;
        *spawn_y = -EDGE_MARGIN;
        break;
    case EDGE_LEFT:
        *spawn_x = -EDGE_MARGIN;
        *spawn_y = along_edge * (float)rows;
        break;
    case EDGE_RIGHT:
        *spawn_x = (float)cols + EDGE_MARGIN;
        *spawn_y = along_edge * (float)rows;
        break;
    case EDGE_BOTTOM:
        *spawn_x = along_edge * (float)cols;
        *spawn_y = (float)rows + EDGE_MARGIN;
        break;
    }
}

/* compute_target_point — point in the OPPOSITE-edge quadrant band; guarantees diagonal arc. */
static void compute_target_point(SpawnEdge edge, int cols, int rows,
                                 uint32_t *rng,
                                 float *target_x, float *target_y)
{
    float r_a = lcg_unit(rng);
    float r_b = lcg_unit(rng);
    switch (edge) {
    case EDGE_TOP:
        *target_x = r_a * (float)cols;
        *target_y = (float)rows * (0.55f + r_b * 0.45f);
        break;
    case EDGE_LEFT:
        *target_x = (float)cols * (0.55f + r_a * 0.45f);
        *target_y = r_b * (float)rows;
        break;
    case EDGE_RIGHT:
        *target_x = (float)cols * (0.00f + r_a * 0.45f);
        *target_y = r_b * (float)rows;
        break;
    case EDGE_BOTTOM:
        *target_x = r_a * (float)cols;
        *target_y = (float)rows * (0.00f + r_b * 0.45f);
        break;
    }
}

/* compute_speed_with_jitter — pp.speed · uniform[1 − jit/2, 1 + jit/2). */
static float compute_speed_with_jitter(const PatternParams *pp, uint32_t *rng)
{
    float jitter_lo  = 1.0f - pp->speed_jitter * 0.5f;
    float jitter_hi  = jitter_lo + pp->speed_jitter;
    return pp->speed * lcg_range(rng, jitter_lo, jitter_hi);
}

/*
 * scene_spawn_comet — DRIVER.  Top up the pool with one new comet.
 *
 *   Step 1   slot ← comet_pool_find_inactive            bail if pool full
 *   Step 2   edge ← pick_spawn_edge(rng)                 weighted edge selector
 *   Step 3   spawn  ← compute_spawn_point(edge, ...)     just OUTSIDE edge
 *   Step 4   target ← compute_target_point(edge, ...)    opposite-quadrant band
 *   Step 5   d ← target − spawn
 *            speed ← compute_speed_with_jitter(pp)
 *   Step 6   c.pos ← spawn
 *            c.vel ← (speed·d.x/|d|, speed·d.y/|d|)      unit-vector · speed
 *            c.emit_carry ← 0; c.age ← 0; c.active ← true
 *
 * INPUTS / UNITS
 *   s   Scene mutated; one inactive Comet slot becomes active.
 *   pos in cells, vel in cells/sec.
 *
 * WHY THE OPPOSITE-QUADRANT TARGET
 *   Aiming at the opposite quadrant guarantees a diagonal arc that
 *   visibly crosses the screen.  Uniform-direction random vectors would
 *   half the time send the comet back out the edge it entered on
 *   (invisible flight, frustrating "where did it go?" effect).
 *
 * WHY IT EXISTS (vs. inlining in scene_tick / phase1)
 *   phase1_topup_comet_pool stays at orchestrator level — "top up until
 *   pool reaches max".  All edge-picking + target-aiming + velocity
 *   computation lives here as one ~30-line unit.
 */
static void scene_spawn_comet(Scene *s)
{
    /* Step 1 — find free slot. */
    int slot_index = comet_pool_find_inactive(s);
    if (slot_index < 0) return;
    Comet *c = &s->comets[slot_index];
    const PatternParams *pp = &pattern_params[s->current_pattern];

    /* Step 2 — pick a weighted spawn edge. */
    SpawnEdge edge = pick_spawn_edge(&s->rng);

    /* Step 3 — spawn point just outside the edge. */
    float spawn_x, spawn_y;
    compute_spawn_point(edge, s->cols, s->rows, &s->rng, &spawn_x, &spawn_y);

    /* Step 4 — target point in the opposite quadrant. */
    float target_x, target_y;
    compute_target_point(edge, s->cols, s->rows, &s->rng, &target_x, &target_y);

    /* Step 5 — velocity = unit(target − spawn) × speed. */
    float delta_x        = target_x - spawn_x;
    float delta_y        = target_y - spawn_y;
    float distance       = sqrtf(delta_x * delta_x + delta_y * delta_y);
    if (distance < 1e-3f) distance = 1.0f;
    float speed          = compute_speed_with_jitter(pp, &s->rng);

    /* Step 6 — write the slot. */
    c->x          = spawn_x;
    c->y          = spawn_y;
    c->vx         = speed * delta_x / distance;
    c->vy         = speed * delta_y / distance;
    c->emit_carry = 0.0f;
    c->age        = 0.0f;
    c->active     = true;
}

/*
 * scene_emit_trail — DRIVER.  Drop ONE trail particle at the comet's position.
 *                    THE moving-emitter mechanism — the file's algorithmic core.
 *
 *   Step 1   slot ← trail_pool_find_inactive             bail if pool full
 *   Step 2   speed ← |comet.vel|;  if < 1e-3 set to 1.0  unit-vector safety
 *   Step 3   perp ← (-vy/speed, vx/speed)                90° CCW rotation
 *   Step 4   kick ← uniform(-1, 1) · 2 · particle_spread lateral cells
 *   Step 5   offset ← perp · kick                        position offset
 *   Step 6   drift ← offset · spread_drift_factor        DRIFT VELOCITY
 *                                                         (NOT inherited from comet)
 *   Step 7   life ← particle_life · uniform(0.7, 1.3)    ±30% jitter
 *            write slot: pos=c.pos+offset, vel=drift, age=0, life, active
 *
 * INPUTS / UNITS
 *   s   Scene mutated; one inactive TrailParticle slot becomes active.
 *   c   emitting comet (const — read only).
 *   pos in cells, vel in cells/sec, life in seconds.
 *
 * WHY THE TRAIL "FALLS BEHIND" — THE WHOLE POINT
 *   The drift velocity at Step 6 is NOT the comet's velocity.  It's a
 *   small perpendicular kick.  Because the trail particle moves slowly
 *   (or barely at all) while the comet keeps flying at its full speed,
 *   the trail visually trails BEHIND the comet — exactly because the
 *   particle did NOT inherit the comet's motion.  Break this decoupling
 *   (e.g. p->vx = c->vx) and the trail rides along with the comet,
 *   producing no streak.
 *
 * WHY IT EXISTS (vs. inlining in comet_emit_trail_particles)
 *   comet_emit_trail_particles handles the fractional-carry accumulator
 *   that decides HOW MANY particles to spawn this frame.  This function
 *   handles WHERE each one goes.  Two responsibilities, two functions.
 */
static void scene_emit_trail(Scene *s, const Comet *c)
{
    /* Step 1 — find an inactive TrailParticle slot; bail if pool full. */
    int slot_index = trail_pool_find_inactive(s);
    if (slot_index < 0) return;
    TrailParticle       *p  = &s->trail[slot_index];
    const PatternParams *pp = &pattern_params[s->current_pattern];

    /* Step 2 — perpendicular unit vector to the comet's flight direction.
     *          In 2-D, rotating (vx, vy) by 90° CCW gives (-vy, vx). */
    float comet_speed = sqrtf(c->vx * c->vx + c->vy * c->vy);
    if (comet_speed < 1e-3f) comet_speed = 1.0f;
    float perp_x = -c->vy / comet_speed;
    float perp_y =  c->vx / comet_speed;

    /* Step 3 — lateral kick magnitude, uniform in ±particle_spread cells. */
    float kick_magnitude = lcg_signed(&s->rng) * 2.0f * pp->particle_spread;

    /* Step 4 — position offset along the perpendicular axis. */
    float offset_x = perp_x * kick_magnitude;
    float offset_y = perp_y * kick_magnitude;

    /* Step 5 — drift velocity. 0 for SHOOTING_STAR (tight streak); >0 for
     *          FIREBALL/PLASMA_BOLT (sparks puff outward as they age). */
    float drift_vx = offset_x * pp->spread_drift_factor;
    float drift_vy = offset_y * pp->spread_drift_factor;

    /* Step 6 — per-particle lifetime jitter, uniform in 0.7×..1.3× the
     *          pattern's base particle_life. */
    float life_factor = lcg_range(&s->rng, 0.7f, 1.3f);
    float life_sec    = pp->particle_life * life_factor;

    /* Step 7 — write the slot. */
    p->x      = c->x + offset_x;
    p->y      = c->y + offset_y;
    p->vx     = drift_vx;
    p->vy     = drift_vy;
    p->age    = 0.0f;
    p->life   = life_sec;
    p->active = true;
}

/* scene_init — zero Scene, install defaults, spawn one seed comet. */
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
    scene_spawn_comet(s);
}

/* scene_resize — record new terminal extent; pools are fixed-size, so nothing else to do. */
static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

/* scene_reseed — bound to 'r': clear all pools, reseed RNG, spawn one comet. */
static void scene_reseed(Scene *s)
{
    s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
    scene_clear_pools(s);
    scene_spawn_comet(s);
}

/* count_active_comets — how many comet slots are currently in flight. */
static int count_active_comets(const Scene *s)
{
    int active = 0;
    for (int i = 0; i < MAX_COMETS; i++)
        if (s->comets[i].active) active++;
    return active;
}

/* phase1_topup_comet_pool — spawn enough comets to reach pp->max_comets. */
static void phase1_topup_comet_pool(Scene *s, const PatternParams *pp)
{
    int active_now    = count_active_comets(s);
    int target_count  = (pp->max_comets > MAX_COMETS) ? MAX_COMETS : pp->max_comets;
    for (int k = active_now; k < target_count; k++) scene_spawn_comet(s);
}

/* comet_apply_plasma_kick — 2-D rotation by uniform(-k, +k); preserves |v|.
 * k=0 (SHOOTING_STAR/FIREBALL) → early return; only PLASMA_BOLT rotates. */
static void comet_apply_plasma_kick(Comet *c, uint32_t *rng, float kick_strength)
{
    if (kick_strength <= 0.0f) return;

    float angle = lcg_signed(rng) * 2.0f * kick_strength;
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    float new_vx = cos_a * c->vx - sin_a * c->vy;
    float new_vy = sin_a * c->vx + cos_a * c->vy;
    c->vx = new_vx;
    c->vy = new_vy;
}

/* comet_hit_ground — predicate+side-effect: if vy>0 and y≥rows-2, ignite blast + deactivate. */
static bool comet_hit_ground(Scene *s, Comet *c)
{
    float ground_y = (float)s->rows - 2.0f;
    if (!(c->vy > 0.0f && c->y >= ground_y)) return false;

    float impact_x = clampf(c->x, 0.0f, (float)s->cols - 1.0f);
    blast_ignite(s, impact_x, ground_y);
    c->active = false;
    return true;
}

/* comet_off_screen — predicate-with-side-effect.  Deactivates a comet
 * that has drifted ±EDGE_MARGIN past any edge; returns true if it did. */
static bool comet_off_screen(Scene *s, Comet *c)
{
    bool off_left  = (c->x < -EDGE_MARGIN);
    bool off_right = (c->x > (float)s->cols + EDGE_MARGIN);
    bool off_top   = (c->y < -EDGE_MARGIN);
    bool off_below = (c->y > (float)s->rows + EDGE_MARGIN);
    if (off_left || off_right || off_top || off_below) {
        c->active = false;
        return true;
    }
    return false;
}

/* comet_emit_trail_particles — Bresenham fractional carry: spawn floor(carry) per frame. */
static void comet_emit_trail_particles(Scene *s, Comet *c,
                                       float dt, const PatternParams *pp)
{
    c->emit_carry += pp->emit_rate * dt;
    int emit_count  = (int)c->emit_carry;
    c->emit_carry  -= (float)emit_count;
    for (int k = 0; k < emit_count; k++) scene_emit_trail(s, c);
}

/* phase2_advance_one_comet — per-comet body: plasma kick → integrate → die → emit. */
static void phase2_advance_one_comet(Scene *s, Comet *c,
                                     float dt, const PatternParams *pp)
{
    /* Step 1 — pattern-specific velocity rotation. */
    comet_apply_plasma_kick(c, &s->rng, pp->angular_kick);

    /* Step 2 — integrate position. */
    c->x   += c->vx * dt;
    c->y   += c->vy * dt;
    c->age += dt;

    /* Step 3 — death tests (impact takes precedence over off-screen). */
    if (comet_hit_ground(s, c)) return;
    if (comet_off_screen(s, c)) return;

    /* Step 4 — emit trail particles for this frame. */
    comet_emit_trail_particles(s, c, dt, pp);
}

/* phase2_advance_all_comets — iterate every active comet slot. */
static void phase2_advance_all_comets(Scene *s, float dt, const PatternParams *pp)
{
    for (int i = 0; i < MAX_COMETS; i++) {
        Comet *c = &s->comets[i];
        if (!c->active) continue;
        phase2_advance_one_comet(s, c, dt, pp);
    }
}

/* phase3_advance_trail — drag + explicit Euler + age + deactivate at age≥life. */
static void phase3_advance_trail(Scene *s, float dt, const PatternParams *pp)
{
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

/* blast_particle_tick_one — wave-delay gate, then drag + integrate + age + die. */
static bool blast_particle_tick_one(BlastParticle *p, const Blast *b,
                                    float drag_factor, float dt,
                                    int cols, int rows)
{
    if (!p->alive) return false;

    /* Gate — wave-stagger countdown holds the spark at centre. */
    if (p->delay > 0.0f) { p->delay -= dt; return true; }

    /* Step 1 — multiplicative drag (exponential decay). */
    p->vx *= drag_factor;
    p->vy *= drag_factor;

    /* Step 2 — explicit Euler integration in offset space. */
    p->rx += p->vx * dt;
    p->ry += p->vy * dt;

    /* Step 3 — age the life counter. */
    p->life -= dt;

    /* Step 4 — death test (life exhausted OR left the playable area). */
    float screen_x   = b->cx + p->rx;
    float screen_y   = b->cy + p->ry;
    bool  burned_out = (p->life <= 0.0f);
    bool  off_screen = (screen_x < 0.0f || screen_x >= (float)cols
                     || screen_y < 0.0f || screen_y >= (float)(rows - 1));
    if (burned_out || off_screen) { p->alive = false; return false; }
    return true;
}

/* phase4_advance_one_blast — flash TTL + per-spark tick; returns "stay active". */
static bool phase4_advance_one_blast(Blast *b, float drag_factor, float dt,
                                     int cols, int rows)
{
    if (b->flash_ttl > 0.0f) b->flash_ttl -= dt;

    bool any_alive = false;
    for (int j = 0; j < BLAST_PARTICLES; j++) {
        if (blast_particle_tick_one(&b->parts[j], b, drag_factor, dt, cols, rows))
            any_alive = true;
    }

    bool flash_done = (b->flash_ttl <= 0.0f);
    return any_alive || !flash_done;
}

/* phase4_advance_all_blasts — drag factor once, then per-blast tick. */
static void phase4_advance_all_blasts(Scene *s, float dt)
{
    float drag_factor = expf(-BLAST_DRAG_PER_SEC * dt);

    for (int i = 0; i < MAX_BLASTS; i++) {
        Blast *b = &s->blasts[i];
        if (!b->active) continue;
        if (!phase4_advance_one_blast(b, drag_factor, dt, s->cols, s->rows))
            b->active = false;
    }
}

/*
 * scene_tick — DRIVER.  Advance the whole simulation one frame (4 phases).
 *
 *   if paused:                                    return
 *   dt *= speed / SPEED_DEF                       user "+/-" time dial
 *   pp ← pattern_params[current_pattern]
 *   phase1_topup_comet_pool   (s, pp)             spawn new comets
 *   phase2_advance_all_comets (s, dt, pp)         per-comet kick/move/emit/die
 *   phase3_advance_trail      (s, dt, pp)         trail drag + Euler + age
 *   phase4_advance_all_blasts (s, dt)             blast flash + per-spark tick
 *
 * INPUTS / UNITS
 *   s   Scene mutated; every active slot in every pool advances by dt.
 *   dt  frame time in seconds (capped at 100 ms by the main loop).
 *   speed multiplier scales dt before any phase runs — one knob slows or
 *   speeds the entire simulation uniformly.
 *
 * WHY THE PHASES ARE IN THIS ORDER
 *   1 first   need comets to integrate.
 *   2 next    trail emission reads comet.pos updated in this same step.
 *   3 after 2 lets trail particles cull before they're rendered.
 *   4 last    blasts are independent; doing them last keeps the comet/trail
 *             working set tighter in cache.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float global_speed_mul = (float)s->speed / (float)SPEED_DEF;
    dt *= global_speed_mul;

    const PatternParams *pp = &pattern_params[s->current_pattern];

    phase1_topup_comet_pool  (s, pp);
    phase2_advance_all_comets(s, dt, pp);
    phase3_advance_trail     (s, dt, pp);
    phase4_advance_all_blasts(s, dt);
}

/* draw_one_trail_particle — round to cell, freshness→slot→glyph+attr, paint. */
static void draw_one_trail_particle(const TrailParticle *p,
                                    int cols, int rows_playable)
{
    int cell_x = round_to_cell(p->x);
    int cell_y = round_to_cell(p->y);
    if (!cell_visible(cell_x, cell_y, cols, rows_playable)) return;

    float freshness  = trail_freshness(p->age, p->life);
    int   ramp_slot  = ramp_slot_from_freshness(freshness);
    char  glyph      = RAMP_GLYPHS[ramp_slot];
    int   color_pair = PAIR_RAMP_BASE + ramp_slot;
    int   attr       = trail_attr_for_slot(ramp_slot);

    paint_cell(cell_y, cell_x, glyph, color_pair, attr);
}

/* scene_draw_trail_layer — LAYER A: paint every active trail particle. */
static void scene_draw_trail_layer(const Scene *s, int rows_playable)
{
    for (int i = 0; i < MAX_TRAIL; i++) {
        const TrailParticle *p = &s->trail[i];
        if (!p->active) continue;
        draw_one_trail_particle(p, s->cols, rows_playable);
    }
}

/* draw_blast_flash — central '*' + 3 '+' (left/right/above; below omitted). */
static void draw_blast_flash(const Blast *b, int cols, int rows_playable)
{
    if (b->flash_ttl <= 0.0f) return;

    int cx = round_to_cell(b->cx);
    int cy = round_to_cell(b->cy);
    if (!cell_visible(cx, cy, cols, rows_playable)) return;

    paint_cell(cy, cx, '*', PAIR_HEAD, A_BOLD);
    if (cx > 0)         paint_cell(cy,     cx - 1, '+', PAIR_HEAD, A_BOLD);
    if (cx < cols - 1)  paint_cell(cy,     cx + 1, '+', PAIR_HEAD, A_BOLD);
    if (cy > 0)         paint_cell(cy - 1, cx,     '+', PAIR_HEAD, A_BOLD);
}

/* draw_one_blast_spark — gate dead/delayed; offset→absolute; freshness→slot+attr; paint. */
static void draw_one_blast_spark(const Blast *b, const BlastParticle *p,
                                 int cols, int rows_playable)
{
    if (!p->alive || p->delay > 0.0f) return;

    int cell_x = round_to_cell(b->cx + p->rx);
    int cell_y = round_to_cell(b->cy + p->ry);
    if (!cell_visible(cell_x, cell_y, cols, rows_playable)) return;

    float freshness  = blast_freshness(p->life, p->max_life);
    int   ramp_slot  = ramp_slot_from_freshness(freshness);
    int   color_pair = PAIR_RAMP_BASE + ramp_slot;
    int   attr       = blast_attr_for_slot(ramp_slot);

    paint_cell(cell_y, cell_x, p->sym, color_pair, attr);
}

/* draw_one_blast — flash + every spark of one Blast slot. */
static void draw_one_blast(const Blast *b, int cols, int rows_playable)
{
    draw_blast_flash(b, cols, rows_playable);
    for (int j = 0; j < BLAST_PARTICLES; j++)
        draw_one_blast_spark(b, &b->parts[j], cols, rows_playable);
}

/* scene_draw_blast_layer — LAYER B: paint every active Blast. */
static void scene_draw_blast_layer(const Scene *s, int rows_playable)
{
    for (int i = 0; i < MAX_BLASTS; i++) {
        const Blast *b = &s->blasts[i];
        if (!b->active) continue;
        draw_one_blast(b, s->cols, rows_playable);
    }
}

/* draw_comet_halo — 8 neighbours: '+' on cardinals (BOLD), ':' on diagonals (DIM). */
static void draw_comet_halo(int head_x, int head_y,
                            int cols, int rows_playable)
{
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;

            int cell_x = head_x + dx;
            int cell_y = head_y + dy;
            if (!cell_visible(cell_x, cell_y, cols, rows_playable)) continue;

            bool is_cardinal = (dx == 0 || dy == 0);
            char glyph       = is_cardinal ? '+'    : ':';
            int  attr        = is_cardinal ? A_BOLD : A_DIM;

            paint_cell(cell_y, cell_x, glyph, PAIR_HALO, attr);
        }
    }
}

/* draw_one_comet — halo first, then the head glyph on top of it. */
static void draw_one_comet(const Comet *c, char head_glyph,
                           int cols, int rows_playable)
{
    int head_x = round_to_cell(c->x);
    int head_y = round_to_cell(c->y);

    draw_comet_halo(head_x, head_y, cols, rows_playable);

    if (!cell_visible(head_x, head_y, cols, rows_playable)) return;
    paint_cell(head_y, head_x, head_glyph, PAIR_HEAD, A_BOLD);
}

/* scene_draw_comet_layer — LAYER C: paint every active comet. */
static void scene_draw_comet_layer(const Scene *s, int rows_playable)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    for (int i = 0; i < MAX_COMETS; i++) {
        const Comet *c = &s->comets[i];
        if (!c->active) continue;
        draw_one_comet(c, pp->head_glyph, s->cols, rows_playable);
    }
}

/*
 * scene_draw — DRIVER.  Three-layer painter's algorithm.
 *
 *   rows_playable ← rows - 1                bottom row reserved for HUD
 *   scene_draw_trail_layer  (s)             LAYER A: background (past)
 *   scene_draw_blast_layer  (s)             LAYER B: mid-ground (event)
 *   scene_draw_comet_layer  (s)             LAYER C: foreground (present)
 *
 * INPUTS / UNITS
 *   s   Scene to read (const).
 *
 * Layer order is z-by-category, not by depth.  Trails are "the past",
 * blasts are "the event", comets are "the present" — comets win every
 * cell collision.
 *
 * WHY IT EXISTS (vs inlining in screen_draw)
 *   screen_draw stays at "erase + scene + HUD".  All scene-rendering
 *   detail lives here as one 4-line orchestrator.
 */
static void scene_draw(const Scene *s)
{
    int rows_playable = s->rows - 1;
    scene_draw_trail_layer(s, rows_playable);
    scene_draw_blast_layer(s, rows_playable);
    scene_draw_comet_layer(s, rows_playable);
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

static inline const char *hud_status_label(const Scene *s)
{
    return s->paused ? "PAUSED       " : pattern_name(s->current_pattern);
}

static void format_hud_text(char *buf, size_t n,
                            const Scene *s, double fps, int sim_fps)
{
    int comets, trails;
    scene_counts(s, &comets, &trails);

    snprintf(buf, n,
             " COMET   %s   theme:%-8s   comets:%d  trail:%4d   "
             "%5.1f fps  %3d Hz  speed:%-3d   "
             "n/p:pat  t/T:theme  +/-:speed  spc:pause  r:reseed  q:quit ",
             hud_status_label(s), themes[s->current_theme].name,
             comets, trails, fps, sim_fps, s->speed);
}

static void paint_hud_bar(int row, int cols, const char *text)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", text);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s);

    char hud_text[200];
    format_hud_text(hud_text, sizeof hud_text, s, fps, sim_fps);
    paint_hud_bar(sc->rows - 1, sc->cols, hud_text);
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

static void app_install_signals(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    app_install_signals();

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
