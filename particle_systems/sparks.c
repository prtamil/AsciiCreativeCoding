/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sparks.c — fast electric sparks with motion-blur trails, bouncing
 *            off the floor with elastic restitution.
 *
 * DEMO: Bright, FAST sparks fly out of an emitter on cone-shaped
 *       initial trajectories, arc under gravity, and BOUNCE off the
 *       floor (the row above the HUD) with energy loss given by the
 *       pattern's restitution coefficient. Each spark drags a short
 *       motion-blur trail of dimmer glyphs behind it — the head is
 *       hot/bright, the tail is dark/dim, a learner-readable picture
 *       of the spark's recent path. Distinct from `embers.c`:
 *
 *         embers.c — slow rising particles, no trail, no bounce,
 *                    cool over a long lifetime (fire flicker).
 *         sparks.c — FAST flying particles, motion-blur trails,
 *                    elastic floor bounces, short crackly lifetime.
 *
 *       Patterns:
 *         WELDING   horizontal jet of orange-yellow sparks from the
 *                   left wall — they arc rightward, hit the floor,
 *                   bounce a couple of times, and die.
 *         GRINDER   a wheel at the bottom-left throws a tight cone
 *                   of bright sparks up and to the right; very fast,
 *                   strong gravity, multiple floor bounces.
 *         CAMPFIRE  a base at the bottom-centre pops gentle sparks
 *                   straight up; slower speed, soft gravity.
 *         TESLA     a coil at screen centre discharges a short-lived
 *                   omnidirectional crackle — sparks fly in every
 *                   direction with low gravity, then bounce.
 *
 * Study alongside:
 *   embers.c    — same pool / spawn / tick / draw skeleton, but
 *                 buoyancy + cooling instead of gravity + bounce.
 *                 Read first; sparks.c is the "FAST + BOUNCY"
 *                 counterpart.
 *   fountain.c  — also gravity + bouncing particles. Compare the
 *                 emit cone and trail rendering.
 *   fireworks.c — radial bursts; TESLA pattern is its closest cousin.
 *
 * Section map:
 *   §1 config   — constants, pattern params, theme heat ramps
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 8-pair heat ramp with 8-colour fallback
 *   §4 spark    — Spark struct + motion-blur trail history
 *   §5 scene    — pool, spawn, tick (integrate + bounce), draw (trail+head)
 *   §6 screen   — ncurses init / draw / resize
 *   §7 app      — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reseed (clear pool)
 *   n / N      next pattern   (WELDING → GRINDER → CAMPFIRE → TESLA)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster      (sim speed multiplier)
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *   w / W      shift emitter right / left
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/sparks.c \
 *       -o sparks -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Pool-based 2-D particle system with one species
 *                  (spark) and a short motion-blur history per particle.
 *                  Each tick:
 *
 *                    1. SPAWN. Top up the active pool to the pattern's
 *                       target count by emitting fresh sparks at the
 *                       emitter point. Each spark is born with a random
 *                       angle in [angle_min, angle_max] and a random
 *                       speed in [speed_min, speed_max], giving a CONE
 *                       of trajectories rather than a single direction.
 *
 *                    2. INTEGRATE. Push each spark's previous (x, y)
 *                       into a short ring of trail history, then update:
 *                          vy  += gravity · dt
 *                          v   *= drag_factor                (per tick)
 *                          x   += vx · dt
 *                          y   += vy · dt
 *                          age += dt
 *
 *                    3. BOUNCE. If the spark crosses the FLOOR (the row
 *                       just above the HUD) while moving downward, it
 *                       bounces ELASTICALLY with restitution e ∈ (0,1):
 *                          y'  = floor_y − (y − floor_y)         // reflect
 *                          vy' = −vy · e                          // flip + lose energy
 *                          vx' = vx · floor_friction              // tangential friction
 *                       If both speed components are below the SETTLE
 *                       threshold, the spark is killed (it would otherwise
 *                       keep bouncing infinitesimally forever).
 *
 *                    4. RENDER. Draw each trail-history point as a dim
 *                       glyph from cool-end of the heat ramp; draw the
 *                       head with a hot/bold glyph from the high end.
 *                       The head's ramp slot is read from the spark's
 *                       remaining-life fraction, so older sparks fade
 *                       through the same heat sequence as embers.
 *
 *                    5. KILL. age >= life, or off-screen, or settled.
 *
 *                  Distinct from `embers.c` in three concrete ways:
 *
 *                    – embers have buoyancy (vy negative); sparks have
 *                      gravity (vy grows positive).
 *                    – embers have no trail; sparks store TRAIL_LEN
 *                      previous positions and render them dimly.
 *                    – embers die when they cool/exit; sparks also
 *                      bounce off the floor with restitution and die
 *                      when they settle (low |v|).
 *
 * Data-structure : Spark[MAX_SPARKS] object pool with `active` flag
 *                  and a small ring of `(trail_x, trail_y)` history.
 *                  No malloc, linear-scan spawn (small N — fine).
 *
 * Rendering      : ASCII only. Trail glyphs are sparse (`,` `.` `:`),
 *                  head glyphs are dense (`*` `+` `#`). Colour from a
 *                  theme heat ramp by remaining-life fraction; the
 *                  trail uses a slot 1–3 below the head's slot so it
 *                  fades behind the spark.
 *
 * Performance    : O(MAX_SPARKS · (1 + TRAIL_LEN)) per frame. With
 *                  TRAIL_LEN = 3 and MAX_SPARKS ≈ 800, that is at most
 *                  ~3200 mvaddch per frame. Trivial at 60 fps.
 *
 * References     :
 *   • Reeves, W. T. (1983) — "Particle Systems: A Technique for
 *     Modelling a Class of Fuzzy Objects", *ACM TOG* 2(2):91–108.
 *     Foundational paper on stochastic particle systems for fire,
 *     sparks, smoke, and explosions.
 *   • Wikipedia — [Coefficient of restitution](https://en.wikipedia.org/wiki/Coefficient_of_restitution).
 *     The `e` in the bounce equation `vy' = −e · vy` is exactly the
 *     COR; this file's `pp->restitution` is the same number.
 *   • Millington, I. — *Game Physics Engine Development*, §6
 *     ("Particle Physics") and §7 ("Particle Contacts"). Plain-English
 *     derivation of the same Euler integration + impulse bounce we
 *     use here.
 *   • Bourke, P. — [Character density ramps](http://paulbourke.net/dataformats/asciiart/).
 *     Source of the cool→hot glyph ramps used for the trail/head.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A spark is a fast, short-lived projectile with memory. Each spark
 * remembers its last few positions; the screen shows that memory as
 * a fading streak behind a bright head. Gravity arcs the spark down,
 * the floor bounces it back up with an energy loss, and the heat
 * ramp colours it from white-hot at birth to dark and dead at the
 * end of its life. Hundreds of these together look like a welding
 * torch / grinder wheel / Tesla coil sparking.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a tracer bullet on a 2-D billiard table that has only a
 * floor (no side walls), with gravity pulling it down. The bullet
 * leaves a short, fading streak behind it. When it hits the floor
 * it bounces back up, but only as high as `e²` of the previous
 * height (for restitution e). After a few bounces it has so little
 * vertical energy left that it slides along the floor until friction
 * stops it. Sparks from a real welder behave exactly like this —
 * which is why the visual reads correctly.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. EMIT. Each tick, count active sparks. If below target, spawn
 *     (target − active) new sparks (capped per tick to avoid bursts)
 *     at the pattern's emitter point. Each new spark gets:
 *
 *        angle ∈ [angle_min, angle_max]      // pattern cone
 *        speed ∈ [speed_min, speed_max]
 *        vx    = speed · cos(angle)
 *        vy    = speed · sin(angle)           // y down on screen
 *        x, y  = emitter point + small jitter
 *        life  = life_min + r · (life_max − life_min)
 *        trail = TRAIL_LEN copies of (x, y)   // start with no trail
 *
 *  2. SHIFT TRAIL & INTEGRATE. For each active spark:
 *
 *        for k = 0..TRAIL_LEN−2:
 *            trail[k] = trail[k+1]            // drop oldest, shift down
 *        trail[TRAIL_LEN−1] = (x, y)          // store current as newest prev
 *
 *        vy   += gravity · dt
 *        v    *= exp(−drag_coeff · dt)        // continuous drag
 *        x    += vx · dt
 *        y    += vy · dt
 *        age  += dt
 *
 *  3. BOUNCE. If y >= floor_y and vy > 0:
 *
 *        y    = floor_y − (y − floor_y)       // reflect about floor
 *        vy   = −vy · restitution             // elastic energy loss
 *        vx   *= floor_friction               // tangential drag
 *        if |vy| < SETTLE_VY and |vx| < SETTLE_VX → kill (settled)
 *
 *  4. KILL. age >= life, or x off-screen, or y above-screen, or
 *     settled at floor. Slot the pool entry inactive.
 *
 *  5. RENDER.
 *        For each active spark:
 *          Draw trail[0..TRAIL_LEN−1] as dim glyphs. Older slots
 *          (lower index) get the cooler ramp slots; newer slots
 *          get warmer.
 *
 *          Compute head's heat slot from remaining life fraction:
 *            T          = max(0, 1 − age / life)
 *            head_slot  = ⌊T · 7.999⌋
 *            head_glyph = HEAD_GLYPHS[head_slot]
 *            head_pair  = PAIR_HEAT_BASE + head_slot
 *            head_attr  = A_BOLD if slot >= 6 else A_NORMAL
 *
 *  6. HUD on bottom row.
 *
 * KEY FORMULAS
 * ────────────
 *  Initial velocity (cone emission):
 *    angle = angle_min + r · (angle_max − angle_min)
 *    speed = speed_min + r · (speed_max − speed_min)
 *    vx    = speed · cos(angle)
 *    vy    = speed · sin(angle)        // y increases downward
 *
 *  Gravity update (positive = downward):
 *    vy += gravity · dt
 *
 *  Continuous drag (frame-rate-independent damping):
 *    drag_factor = exp(−drag_coeff · dt)
 *    v          *= drag_factor
 *
 *  Floor bounce (about the line y = floor_y):
 *    y'  = 2·floor_y − y                // reflect
 *    vy' = −vy · restitution
 *    vx' = vx · floor_friction
 *
 *  Heat-ramp slot from remaining life:
 *    T          = max(0, 1 − age / life)
 *    head_slot  = ⌊T · 7.999⌋           // 0..7
 *    trail_slot = max(0, head_slot − 1 − k)  // dimmer for older trail points
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • SETTLED SPARKS THAT BOUNCE FOREVER. Without a settle check,
 *    a spark with very small |vy| will be re-bounced on every tick
 *    by gravity → tiny vy → bounce → repeat. Kill it when both
 *    |vy| < SETTLE_VY and |vx| < SETTLE_VX immediately after a
 *    bounce so the pool slot can be reused.
 *
 *  • TRAIL GHOSTS AT SPAWN. If we left trail history uninitialised,
 *    fresh sparks would draw a streak from a random previous owner's
 *    last position. We fill all TRAIL_LEN slots with the spawn (x,y)
 *    at birth — for the first few ticks the trail draws on top of
 *    the head, harmlessly invisible.
 *
 *  • OFF-SCREEN KILL. A TESLA spark fired upward might leave the
 *    top of the screen at speed and never come back (drag eats it
 *    before gravity wins). Kill at y < TOP_KILL_MARGIN so dormant
 *    pool slots get reclaimed.
 *
 *  • PATTERN CHANGE DOES NOT CLEAR THE POOL. Existing sparks finish
 *    their lifetimes under the old physics and fade away while new
 *    ones spawn under the new pattern. Press `r` to clear instantly.
 *
 *  • PAUSE FREEZES TIME. The tick early-returns when paused, so
 *    sparks freeze mid-arc. Resuming continues the same trajectory.
 *
 *  • EMITTER OFF-SCREEN. The user can shift the emitter past the
 *    edge with `w`/`W` — the cone still emits but most sparks
 *    immediately die at the off-screen kill check. Press `r` (which
 *    resets the offset to 0) to recover.
 *
 *  • RESTITUTION = 1. With perfect elasticity sparks would bounce
 *    forever. Patterns use restitution ∈ [0.4, 0.7] so energy decays
 *    visibly across a few bounces.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • PAUSE (space). The whole jet freezes mid-flight, trails frozen.
 *    Resume: motion continues seamlessly.
 *
 *  • WELDING. Bright orange-yellow sparks shoot rightward from the
 *    left wall, arc down, hit the floor, bounce 1–2 times, die.
 *    Trail visibly drags behind each spark.
 *
 *  • GRINDER. A tight cone shoots up-and-right from a wheel at the
 *    bottom-left. Fast — sparks span most of the screen before they
 *    arc back down. Bounce off the floor, sometimes twice.
 *
 *  • CAMPFIRE. Sparks pop straight up from the bottom-centre, arc
 *    down, bounce gently a couple of times. Slower, sparser.
 *
 *  • TESLA. Sparks fire in EVERY direction from the screen centre.
 *    Some go up and arc back; some go down and immediately bounce.
 *    Short-lived crackle.
 *
 *  • `w`/`W`. Shift the emitter; the cone follows. The trail of
 *    in-flight sparks is unaffected (they were emitted before the
 *    shift).
 *
 *  • `t`/`T`. Theme cycle changes only colours. Physics identical.
 *    DEFAULT (welding orange) → ELECTRIC (blue/cyan/white) is the
 *    most striking transition.
 *
 *  • Reduce `restitution` mentally by editing the pattern: a value
 *    of 0 should make sparks stick (die on first floor contact).
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

    MAX_SPARKS       = 800,
    TRAIL_LEN        =   3,        /* motion-blur history slots          */

    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,

    /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD         =   1,
    PAIR_HINT        =   2,
    PAIR_HEAT_BASE   =   3,        /* +0..+7 = 8 heat ramp slots         */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Emitter horizontal-shift step (cells) when the user presses w/W. */
#define EMITTER_SHIFT_STEP   8.0f

/* Bounce settle thresholds (cells/sec). Below these magnitudes after
 * a bounce, the spark is killed — it has too little energy to make a
 * visible bounce and would otherwise keep micro-bouncing forever.
 * Tuned by visual inspection — high enough to cull sliding sparks
 * within ~1 second, low enough that real bounces still register. */
#define SETTLE_VY            6.0f
#define SETTLE_VX           10.0f

/* Off-screen kill margin. A spark whose y goes more than this many
 * cells above the screen never comes back (drag dominates), so kill
 * early to free the pool slot. */
#define TOP_KILL_MARGIN      4.0f

/* Pattern enum. */
typedef enum {
    PATTERN_WELDING  = 0,
    PATTERN_GRINDER  = 1,
    PATTERN_CAMPFIRE = 2,
    PATTERN_TESLA    = 3,
    N_PATTERNS       = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_WELDING:  return "WELDING ";
    case PATTERN_GRINDER:  return "GRINDER ";
    case PATTERN_CAMPFIRE: return "CAMPFIRE";
    case PATTERN_TESLA:    return "TESLA   ";
    default:               return "?       ";
    }
}

/* Where the emitter sits on the screen — patterns differ. */
typedef enum {
    EMIT_LEFT_MID,    /* x = small fixed cells, y = ~55% rows  (WELDING)  */
    EMIT_BOT_LEFT,    /* x = ~18% cols,         y = rows-4     (GRINDER)  */
    EMIT_BOT_CENTER,  /* x = 50% cols,          y = rows-4     (CAMPFIRE) */
    EMIT_CENTER,      /* x = 50% cols,          y = 50% rows   (TESLA)    */
} EmitPos;

/*
 * PatternParams — per-pattern physics + emission cone + visuals.
 *
 *   target_sparks        : steady-state active spark count
 *   emitter              : where on the screen the emitter is anchored
 *   emit_x_jitter        : ± cells of spawn-position jitter (x)
 *   emit_y_jitter        : ± cells of spawn-position jitter (y)
 *   speed_min, speed_max : initial speed range (cells/sec)
 *   angle_min, angle_max : initial direction range (radians).
 *                          0 = +x (right), -π/2 = -y (up).
 *                          Setting angle_max - angle_min ≥ 2π gives
 *                          full omnidirectional emission (TESLA).
 *   gravity              : downward acceleration (+cells/sec², always positive)
 *   drag_coeff           : per-second velocity damping (continuous)
 *   restitution          : floor bounce coefficient (0..1)
 *   floor_friction       : tangential vx multiplier per bounce
 *   life_min, life_max   : spark lifetime range (seconds)
 */
typedef struct {
    int     target_sparks;
    EmitPos emitter;
    float   emit_x_jitter;
    float   emit_y_jitter;
    float   speed_min, speed_max;
    float   angle_min, angle_max;
    float   gravity;
    float   drag_coeff;
    float   restitution;
    float   floor_friction;
    float   life_min, life_max;
} PatternParams;

/* Angle conventions for the cones below.
 *   +0.0           = pointing right (+x)
 *   -M_PI/2  ≈ -1.57 = pointing straight up (-y)
 *   +M_PI/2  ≈ +1.57 = pointing straight down (+y)
 *   ±M_PI    ≈ ±3.14 = pointing left
 *
 * To get an "up-and-right" cone you want angles in (-π/2, 0). To get
 * "everywhere" you want a 2π-wide range. */
static const PatternParams pattern_params[N_PATTERNS] = {
    /* Field order:
     *  target  emitter         emit_jx emit_jy  spd_min spd_max  ang_min       ang_max     grav   drag   rest  fric   lmin  lmax
     */
    /* WELDING  */ { 380, EMIT_LEFT_MID,    1.0f,  1.0f,   55.0f,  90.0f,  -0.55f,        0.55f,    78.0f, 0.40f, 0.55f, 0.78f, 1.0f, 2.2f },
    /* GRINDER  */ { 320, EMIT_BOT_LEFT,    0.6f,  0.6f,   72.0f, 110.0f,  -1.30f,       -0.40f,    92.0f, 0.30f, 0.50f, 0.70f, 0.8f, 1.8f },
    /* CAMPFIRE */ { 220, EMIT_BOT_CENTER,  3.0f,  0.5f,   34.0f,  56.0f,  -2.20f,       -0.94f,    36.0f, 0.55f, 0.40f, 0.60f, 1.5f, 3.0f },
    /* TESLA    */ { 260, EMIT_CENTER,      0.4f,  0.4f,   55.0f,  85.0f,  -(float)M_PI,  (float)M_PI, 26.0f, 0.20f, 0.65f, 0.78f, 0.5f, 1.2f },
};

/*
 * Themes — each is an 8-step HEAT RAMP from cool/dim (slot 0) to
 * hot/bright (slot 7). DEFAULT is the classic welding orange-yellow;
 * the rest remap the same idea to alternative colour families.
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule, so even ramp[0]
 * (the dimmest "tail" colour) stays visible against a black terminal
 * with A_DIM applied.
 */
typedef struct {
    const char *name;
    short       heat[8];   /* cool → hot */
} Theme;

#define N_THEMES 8

static const Theme themes[N_THEMES] = {
    /* name           heat[0..7]                                      */
    { "WELDING",   { 130, 166, 202, 208, 214, 220, 226, 231 } },
    { "ELECTRIC",  {  24,  27,  33,  39,  45,  51, 123, 195 } },
    { "WHITE_HOT", { 244, 246, 248, 250, 252, 253, 254, 231 } },
    { "COPPER",    { 130, 137, 173, 179, 215, 222, 229, 230 } },
    { "PLASMA",    {  53,  91, 134, 165, 207, 213, 219, 225 } },
    { "GREEN_NEON",{  28,  34,  64,  70, 112, 156, 192, 231 } },
    { "NEON_PINK", {  89, 125, 161, 197, 198, 213, 219, 225 } },
    { "ICE",       {  24,  25,  31,  38,  45,  51, 117, 195 } },
};

/*
 * Glyph ramps for the head and trail. The head uses dense punctuation
 * (`*` `+` `#`) to read as a hot point. The trail uses sparse glyphs
 * (`,` `.` `:`) to read as a fading streak BEHIND the head. Both are
 * indexed cool (slot 0, dim) → hot (slot 7, bright).
 */
static const char HEAD_GLYPHS [8] = { '`', '.', ':', ';', '*', '+', '#', '@' };
static const char TRAIL_GLYPHS[8] = { '`', '.', '.', ',', ':', ';', '+', '*' };

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
    } else {
        /* 8-colour fallback: roughly cool→hot using basic palette. */
        static const short fb[8] = {
            COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_YELLOW,
            COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_HEAT_BASE + i), fb[i], -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §4  spark                                                              */
/* ===================================================================== */

/*
 * Spark — one projectile with motion-blur history.
 *
 *   x, y          : current position (cells)
 *   vx, vy        : velocity (cells / second).  +y is downward on
 *                   screen, so gravity adds positive vy.
 *   age, life     : seconds. Spark dies at age >= life.
 *   trail_x/y[]   : ring of TRAIL_LEN previous positions, oldest at
 *                   index 0, newest (most recent prev) at index
 *                   TRAIL_LEN-1. Filled with spawn (x,y) at birth so
 *                   the renderer never sees uninitialised history.
 *   active        : pool-slot occupancy flag.
 */
typedef struct {
    float x, y;
    float vx, vy;
    float age, life;
    float trail_x[TRAIL_LEN];
    float trail_y[TRAIL_LEN];
    bool  active;
} Spark;

/* Cheap LCG — same constants as embers.c so the visual "noise feel"
 * is consistent across the particle_systems/ family. */
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
    int       speed;                /* sim-speed multiplier */
    int       current_theme;
    Pattern   current_pattern;
    float     emitter_offset_x;     /* user shift via w/W (cells) */
    uint32_t  rng;
    int       rows, cols;
    Spark     sparks[MAX_SPARKS];
} Scene;

/* The floor — sparks bounce off the row above the HUD. The HUD lives
 * on row (rows - 1); sparks reflect about y = rows - 2. */
static inline float scene_floor_y(const Scene *s)
{
    return (float)(s->rows - 2);
}

static int spark_pool_find_inactive(Scene *s)
{
    for (int i = 0; i < MAX_SPARKS; i++)
        if (!s->sparks[i].active) return i;
    return -1;
}

static void scene_clear_sparks(Scene *s)
{
    for (int i = 0; i < MAX_SPARKS; i++) s->sparks[i].active = false;
}

/*
 * scene_emitter_xy — compute the emitter's (cx, cy) for the current
 * pattern, including the user's horizontal offset.
 *
 * Each pattern anchors to a different region of the screen so the
 * four scenes look distinct — a welder lives on the left wall, a
 * grinder at the bottom-left, a campfire at the bottom-centre, a
 * Tesla coil at the screen centre. The offset_x lets the user drag
 * any of them sideways without rebuilding the pattern table.
 */
static void scene_emitter_xy(const Scene *s, float *cx, float *cy)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    switch (pp->emitter) {
    case EMIT_LEFT_MID:
        *cx = 6.0f;
        *cy = (float)s->rows * 0.55f;
        break;
    case EMIT_BOT_LEFT:
        *cx = (float)s->cols * 0.18f;
        *cy = (float)(s->rows - 4);
        break;
    case EMIT_BOT_CENTER:
        *cx = (float)s->cols * 0.50f;
        *cy = (float)(s->rows - 4);
        break;
    case EMIT_CENTER:
    default:
        *cx = (float)s->cols * 0.50f;
        *cy = (float)s->rows * 0.50f;
        break;
    }
    *cx += s->emitter_offset_x;
}

/*
 * scene_spawn_spark — emit one spark at the emitter point.
 *
 * Why initialise the entire trail history to the spawn (x, y):
 * if we left it uninitialised, the renderer would draw a streak from
 * a random previous occupant's last position the moment this spark
 * was born — visually noisy and confusing. Filling all TRAIL_LEN
 * slots with the spawn point means the trail is invisible (drawn on
 * top of the head) for the first TRAIL_LEN ticks, then naturally
 * stretches out as the spark moves.
 *
 * Equivalent to scene_spawn_ember() in embers.c; the difference is
 * that the velocity here comes from a polar (angle, speed) cone
 * instead of a "mostly upward + lateral jitter" rectangle.
 */
static void scene_spawn_spark(Scene *s)
{
    int idx = spark_pool_find_inactive(s);
    if (idx < 0) return;
    Spark *e = &s->sparks[idx];

    const PatternParams *pp = &pattern_params[s->current_pattern];

    float cx, cy;
    scene_emitter_xy(s, &cx, &cy);

    float r1 = lcg_unit(&s->rng);
    float r2 = lcg_unit(&s->rng);
    float r3 = lcg_unit(&s->rng);
    float r4 = lcg_unit(&s->rng);
    float r5 = lcg_unit(&s->rng);

    float angle = pp->angle_min + r1 * (pp->angle_max - pp->angle_min);
    float speed = pp->speed_min + r2 * (pp->speed_max - pp->speed_min);

    e->x      = cx + (r3 - 0.5f) * 2.0f * pp->emit_x_jitter;
    e->y      = cy + (r4 - 0.5f) * 2.0f * pp->emit_y_jitter;
    e->vx     = speed * cosf(angle);
    e->vy     = speed * sinf(angle);
    e->age    = 0.0f;
    e->life   = pp->life_min + r5 * (pp->life_max - pp->life_min);

    for (int k = 0; k < TRAIL_LEN; k++) {
        e->trail_x[k] = e->x;
        e->trail_y[k] = e->y;
    }
    e->active = true;
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused           = false;
    s->speed            = SPEED_DEF;
    s->current_theme    = 0;
    s->current_pattern  = PATTERN_WELDING;
    s->emitter_offset_x = 0.0f;
    s->rng              = (uint32_t)clock_ns();
    s->cols             = cols;
    s->rows             = rows;
    scene_clear_sparks(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_reseed(Scene *s)
{
    s->rng = (uint32_t)clock_ns() ^ 0xDEADBEEFu;
    s->emitter_offset_x = 0.0f;
    scene_clear_sparks(s);
}

/*
 * scene_tick — advance the simulation by `dt` seconds (after applying
 * the user's speed multiplier).
 *
 * Order of operations matters:
 *   1. Top up the pool with new spawns FIRST so that newly-emitted
 *      sparks also receive an integration step this tick (otherwise
 *      they'd look frozen for one frame).
 *   2. For each active spark, shift trail BEFORE integrating so the
 *      newest trail slot stores the position the spark is leaving
 *      this tick (the visual head→trail link).
 *   3. Bounce check AFTER integration so we react to the newly-
 *      computed (x, y) rather than the previous frame.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    dt *= speed_mul;

    const PatternParams *pp = &pattern_params[s->current_pattern];

    /* 1. Top up to target. */
    int active = 0;
    for (int i = 0; i < MAX_SPARKS; i++) if (s->sparks[i].active) active++;
    int target = pp->target_sparks;
    if (target > MAX_SPARKS) target = MAX_SPARKS;
    /* Cap per-tick spawns so a long pause doesn't dump the entire
     * pool in a single frame on resume. */
    int spawn_cap = (int)((float)pp->target_sparks * dt * 6.0f) + 4;
    int to_spawn  = target - active;
    if (to_spawn > spawn_cap) to_spawn = spawn_cap;
    for (int k = 0; k < to_spawn; k++) scene_spawn_spark(s);

    /* 2 & 3. Integrate + bounce. */
    float floor_y    = scene_floor_y(s);
    float drag_factor = expf(-pp->drag_coeff * dt);
    /* drag_factor < 1 — frame-rate-independent because the exp() form
     * commutes correctly across substeps (unlike v *= 0.99). */

    for (int i = 0; i < MAX_SPARKS; i++) {
        Spark *e = &s->sparks[i];
        if (!e->active) continue;

        /* Shift trail history: drop oldest, push current as newest prev. */
        for (int k = 0; k < TRAIL_LEN - 1; k++) {
            e->trail_x[k] = e->trail_x[k + 1];
            e->trail_y[k] = e->trail_y[k + 1];
        }
        e->trail_x[TRAIL_LEN - 1] = e->x;
        e->trail_y[TRAIL_LEN - 1] = e->y;

        /* Integrate (semi-implicit Euler — gravity into velocity first). */
        e->vy += pp->gravity * dt;
        e->vx *= drag_factor;
        e->vy *= drag_factor;
        e->x  += e->vx * dt;
        e->y  += e->vy * dt;
        e->age += dt;

        /* Floor bounce. */
        if (e->y >= floor_y && e->vy > 0.0f) {
            float overshoot = e->y - floor_y;
            e->y  = floor_y - overshoot;          /* reflect about floor */
            if (e->y > floor_y) e->y = floor_y;   /* numeric safety      */
            e->vy = -e->vy * pp->restitution;
            e->vx *=  pp->floor_friction;
            /* Settle: if energy is too low to make a visible bounce,
             * kill the spark so we don't burn ticks micro-bouncing it. */
            if (fabsf(e->vy) < SETTLE_VY && fabsf(e->vx) < SETTLE_VX) {
                e->active = false;
                continue;
            }
        }

        /* Death conditions. */
        if (e->age >= e->life) { e->active = false; continue; }
        if (e->x < -2.0f || e->x > (float)(s->cols + 2)) {
            e->active = false; continue;
        }
        if (e->y < -TOP_KILL_MARGIN) { e->active = false; continue; }
    }
}

/*
 * spark_head_slot — heat-ramp slot for the spark's head from its
 * remaining-life fraction. T = 1 at birth (slot 7, white-hot), T = 0
 * at death (slot 0, dim). Same mapping as ember temperature in
 * embers.c so trained eyes read sparks consistently.
 */
static inline int spark_head_slot(const Spark *e)
{
    float T = 1.0f - e->age / e->life;
    if (T < 0.0f) T = 0.0f;
    if (T > 1.0f) T = 1.0f;
    int slot = (int)(T * 7.999f);
    if (slot < 0) slot = 0;
    if (slot > 7) slot = 7;
    return slot;
}

/*
 * scene_draw — render trails first, then heads on top.
 *
 * Why two passes (trail loop then head loop): drawing the trail of
 * spark A could otherwise overwrite the head of spark B if they
 * crossed. Trail-first / head-second guarantees heads always sit
 * on top of any trail — the bright dot you actually track with your
 * eye is never shadowed by a passing streak.
 */
static void scene_draw(const Scene *s)
{
    int rows_eff = s->rows - 1;     /* leave bottom row for HUD */

    /* ── 1. Trails (dim, drawn first so heads can overwrite) ────────── */
    for (int i = 0; i < MAX_SPARKS; i++) {
        const Spark *e = &s->sparks[i];
        if (!e->active) continue;

        int head_slot = spark_head_slot(e);

        for (int k = 0; k < TRAIL_LEN; k++) {
            /* Older trail points (lower k) get cooler ramp slots so
             * the streak fades behind the spark. */
            int slot = head_slot - (TRAIL_LEN - k);
            if (slot < 0) continue;        /* too cool to render        */

            int ix = (int)(e->trail_x[k] + 0.5f);
            int iy = (int)(e->trail_y[k] + 0.5f);
            if (ix < 0 || ix >= s->cols)  continue;
            if (iy < 0 || iy >= rows_eff) continue;

            char glyph = TRAIL_GLYPHS[slot];
            int  attr  = (slot <= 1) ? A_DIM : A_NORMAL;
            int  pair  = PAIR_HEAT_BASE + slot;
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(iy, ix, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }

    /* ── 2. Heads (bright, drawn last so they sit on top) ──────────── */
    for (int i = 0; i < MAX_SPARKS; i++) {
        const Spark *e = &s->sparks[i];
        if (!e->active) continue;

        int ix = (int)(e->x + 0.5f);
        int iy = (int)(e->y + 0.5f);
        if (ix < 0 || ix >= s->cols)  continue;
        if (iy < 0 || iy >= rows_eff) continue;

        int  slot  = spark_head_slot(e);
        char glyph = HEAD_GLYPHS[slot];
        int  attr  = (slot >= 6) ? A_BOLD
                   : (slot <= 1) ? A_DIM
                   :               A_NORMAL;
        int  pair  = PAIR_HEAT_BASE + slot;
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(iy, ix, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | attr);
    }
}
/* ── end §5 — to understand the ncurses I/O wrapper, read §6 screen ── */

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

static int scene_active_count(const Scene *s)
{
    int n = 0;
    for (int i = 0; i < MAX_SPARKS; i++) if (s->sparks[i].active) n++;
    return n;
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s);

    int active = scene_active_count(s);

    const char *state_str = s->paused
                          ? "PAUSED  "
                          : pattern_name(s->current_pattern);

    char buf[220];
    snprintf(buf, sizeof buf,
             " SPARKS   %s   theme:%-10s   sparks:%4d   "
             "emit_dx:%+5.1f   %5.1f fps  %3d Hz  speed:%-3d   "
             "n/p:pat  t/T:theme  w/W:emitter  +/-:speed  spc:pause  r:reseed  q:quit ",
             state_str, themes[s->current_theme].name, active,
             (double)s->emitter_offset_x, fps, sim_fps, s->speed);

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
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        break;

    case 'w':
        s->emitter_offset_x += EMITTER_SHIFT_STEP;
        break;
    case 'W':
        s->emitter_offset_x -= EMITTER_SHIFT_STEP;
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
