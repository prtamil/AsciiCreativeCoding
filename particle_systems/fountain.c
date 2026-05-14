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
 *   §6 scene     — pools, spawn/emit, tick + helpers, draw + helpers, reseed
 *   §7 screen    — ncurses init / draw / resize
 *   §8 app       — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reseed (clear pools; emission refills on next tick)
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
 * Rendering      : ASCII only. Five drop glyphs picked from velocity
 *                  (drop_glyph_for_velocity): apex / near-stationary
 *                  → `*`; rising → `'` (slow) or `^` (fast); falling
 *                  → `.` (slow) or `,` (fast). Colour from the active
 *                  theme's water or lava ramp indexed by drop HEIGHT
 *                  fraction (peak = brightest). No background fill.
 *
 * Performance    : O(MAX_DROPS + MAX_SPLASHES) per tick. With FOUNTAIN
 *                  defaults (380 target drops + ~250 splashes), that's
 *                  ~630 particles × 1-cell render ≈ 630 mvaddch per
 *                  frame. Trivial at 60 fps.
 *
 * References
 * ──────────
 *   PAPERS
 *     Reeves, W. T. (1983)
 *       "Particle Systems — A Technique for Modeling a Class of Fuzzy Objects"
 *       ACM Transactions on Graphics 2(2): 91-108.
 *       Foundational paper.  Source-with-cone-ejection is exactly the
 *       drop-spawn model used here; splashes are a second species pool.
 *
 *     Witkin, A. & Baraff, D. (2001)
 *       "Physically Based Modeling: Principles and Practice"
 *       SIGGRAPH course notes (online proceedings).
 *       §1 — particle dynamics under constant force fields, the
 *       integrator drop_step_ballistic implements.
 *
 *   BOOKS
 *     Bourg, D. M. & Bywalec, B. — "Physics for Game Developers" (2nd ed,
 *       O'Reilly, 2013).  Ch. 2-4: ballistic projectile motion under
 *       gravity, closed-form parabolic trajectory + the explicit Euler
 *       discretisation used by drop_step_ballistic.
 *
 *     Akenine-Möller, T., Haines, E. & Hoffman, N. — "Real-Time Rendering"
 *       (4th ed, CRC Press, 2018).  §13.7 covers point-sprite particle
 *       rendering; the height-fraction → ramp-slot trick is the same
 *       intensity-from-state pattern.
 *
 *     Foley, J. D., van Dam, A., Feiner, S. K. & Hughes, J. F. — "Computer
 *       Graphics: Principles and Practice" (3rd ed, Addison-Wesley, 2013).
 *       §17.6.2: particle systems as stochastic modelling primitive,
 *       cone-ejection as the canonical fountain/spray generator.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE — DATA-DRIVEN PATTERN ENGINE ────────────────────────── *
 *
 * Four fountain effects (GEYSER, FOUNTAIN, WATERFALL, VOLCANIC) are
 * produced by a SINGLE generic two-species engine — ballistic DROPS
 * + bouncing SPLASH particles — driven by an array of PatternParams
 * structs.  scene_tick and the render layer never branch on the
 * pattern enum to decide HOW the physics works; they read
 * pattern_params[s->current_pattern] and compute accordingly.
 * Adding a new fountain is a matter of appending one row to
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
 *     # 1. SPAWN DROPS — top up pool toward target density
 *     while count(active drops) < pp.target_drops:
 *         d = next_inactive_drop_slot()
 *         d.x      = scene.cols/2 + uniform(-pp.source_x_spread,
 *                                           +pp.source_x_spread)
 *         d.y      = pp.source_top ? top_row : (scene.rows - 2)
 *         alpha    = uniform(-pp.cone_half_angle, +pp.cone_half_angle)
 *         speed    = pp.speed_init · jitter(±15%)        # Reeves stochastic
 *         d.vx     = speed · sin(alpha)
 *         d.vy     = (pp.upward ? -1 : +1) · speed · cos(alpha)
 *         d.life   = uniform(pp.life_max·0.6, pp.life_max)
 *         d.active = true
 *
 *     # 2. INTEGRATE DROPS — semi-implicit Euler with constant gravity
 *     for d in active drops:
 *         d.vy += pp.gravity · dt                        # always positive
 *         d.x  += d.vx · dt
 *         d.y  += d.vy · dt
 *         d.age += dt
 *
 *     # 3. FLOOR IMPACT — drop hits floor → spawn splashes
 *     for d in active drops where d.y >= floor:
 *         n_splash = round(SPLASH_BASE_PER_DROP · pp.splash_mul)
 *         for k in 0..n_splash:
 *             s = next_inactive_splash_slot()
 *             s.x, s.y = d.x, floor
 *             (s.vx, s.vy) = polar(angle≈up_jitter, speed_jitter)
 *             s.life       = SPLASH_LIFE
 *             s.active     = true
 *         d.active = false
 *
 *     # 4. INTEGRATE SPLASHES — Euler with gravity + drag
 *     for s in active splashes:
 *         s.vy += GRAVITY · dt
 *         s.vx *= SPLASH_DRAG_FACTOR
 *         s.x  += s.vx · dt
 *         s.y  += s.vy · dt
 *         s.age += dt
 *         if s.age >= s.life:  s.active = false
 *
 *     # 5. CULL — drops past lifetime cap or off-screen
 *     for d in active drops:
 *         if d.age >= d.life or d.x off-screen:  d.active = false
 *
 *     # 6. RENDER — drops by height-fraction along ramp,
 *     #             splashes as small dim glyphs.
 *     ramp = pp.hot_palette ? theme.lava : theme.water
 *     for d in active drops:
 *         f    = drop_height_fraction(d, pp, scene.rows)
 *         slot = ⌊f · 7⌋
 *         paint(round(d.x), round(d.y), DROP_GLYPHS[slot], ramp[slot])
 *     for s in active splashes:
 *         paint(round(s.x), round(s.y), '·', PAIR_SPLASH)
 *
 *
 * PATTERNPARAMS FIELD → ENGINE HOOK
 * ─────────────────────────────────
 *
 *   target_drops      →  spawn-loop refill cap        (stream density)
 *   source_top        →  spawn-y row + ramp direction (top vs bottom emit)
 *   source_x_spread   →  spawn-x jitter               (jet vs sheet)
 *   speed_init        →  base spawn speed             (× ±15% jitter)
 *   cone_half_angle   →  α range at spawn             (Foley cone-ejection)
 *   upward            →  sign of vy at spawn          (-1 up, +1 down)
 *   gravity           →  vy update each tick          (Bourg Ch. 3)
 *   life_max          →  drop lifetime cap            (zombie cull)
 *   hot_palette       →  ramp[] choice                (water vs lava)
 *   splash_mul        →  splashes per impact          (= mul·SPLASH_BASE)
 *
 * Every other engine constant — MAX_DROPS, MAX_SPLASHES,
 * SPLASH_BASE_PER_DROP, SPLASH_LIFE, SPLASH_DRAG_FACTOR,
 * DROP_SPEED_VARIANCE, DROP_GLYPHS layout — is a GLOBAL tuning knob
 * shared across all patterns.  Patterns differ ONLY in the ten
 * fields above.
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
 *     architecture for natural-particle simulations.  Reeves
 *     enumerates fountain/spray as a canonical use case for
 *     stochastic cone emission — exactly what scene_spawn_drop
 *     does here under cone_half_angle.
 *
 *   Foley, van Dam, Feiner & Hughes (2013)
 *     "Computer Graphics: Principles and Practice" (3rd ed),
 *     §17.6.2.  Treats particle systems as a stochastic modelling
 *     primitive and cone-ejection as the canonical generator —
 *     this file is a textbook implementation of that pattern.
 *
 *   Gamma, E., Helm, R., Johnson, R. & Vlissides, J. (1994)
 *     "Design Patterns" (Addison-Wesley) — STRATEGY pattern (§5.9).
 *     A family of algorithms (GEYSER / FOUNTAIN / WATERFALL /
 *     VOLCANIC) interchangeable behind a single interface (the
 *     engine reading PatternParams).  In procedural C the
 *     "interface" is the struct shape; "concrete strategies" are
 *     the rows of pattern_params[]; "selecting a strategy" is
 *     updating scene.current_pattern.
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
 *     shared instance per "kind" of fountain.  DATA LOCALITY
 *     chapter — Drop and Splash are flat-laid-out PODs swept
 *     linearly by the integrator each tick, exactly the layout
 *     Nystrom recommends for hot inner loops.  OBJECT POOL chapter
 *     — fixed-size BSS arrays implement Nystrom's pool idiom
 *     directly.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * ALGORITHM IN STEPS  (each numbered step = one helper in §6)
 * ──────────────────
 *  1. SPAWN.  drops_emit_to_target (calls scene_spawn_drop per slot).
 *     Count active drops; if below `pattern.target_drops`, spawn the
 *     difference (capped per tick so a long pause doesn't dump a flood).
 *     At spawn:
 *       angle  = (rand − 0.5) · 2 · cone_half_angle
 *       speed  = speed_init · (0.85 + rand · 0.30)
 *       vx     = speed · sin α
 *       vy     = upward ? -speed · cos α  :  +speed · cos α
 *       (x, y) = source position with small x scatter
 *
 *  2. INTEGRATE per drop.  drop_step_ballistic (explicit Euler):
 *       drop.vy += pattern.gravity · dt
 *       drop.x  += drop.vx · dt
 *       drop.y  += drop.vy · dt
 *       drop.age += dt
 *
 *  3. KILL & SPLASH.  drops_integrate_and_cull (calls scene_emit_splashes
 *     on impact).  When drop.y >= rows-2 (or drop.age > life_max, or
 *     drop drifts off-screen sideways) the drop dies; on ground impact
 *     it spawns `K = round(splash_mul · SPLASH_BASE)` splashes at
 *     (x, y_floor).
 *
 *  4. INTEGRATE SPLASHES.  splashes_integrate_and_cull (calls
 *     splash_step_kinematic per active splash; same bounce-ball physics
 *     as rain.c):
 *       splash.vy += SPLASH_GRAVITY · dt
 *       splash.vx *= exp(-SPLASH_DRAG · dt)
 *       update position; deactivate when age >= life.
 *
 *  5. RENDER.  scene_draw → drops_render + splashes_render.
 *     Drop glyph from velocity (drop_glyph_for_velocity: apex `*`,
 *     rising `'`/`^`, falling `.`/`,`). Colour from height fraction
 *     (drop_height_fraction → ramp slot 0..7, peak = brightest) using
 *     the WATER ramp by default and the LAVA ramp when
 *     pattern.hot_palette is set. Splash glyph from age/life ratio
 *     (splash_life_phase): fresh = `*`, mid = `+`, old = `.`.
 *
 *  6. HUD on bottom row (screen_draw).
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
 *  Velocity-based glyph (drop_glyph_for_velocity, threshold 6 / 40 c/s):
 *    if |vy| < 6 c/s             : '*'   (apex / hovering)
 *    elif vy > 0 (falling)       : '.'  (|vy| ≤ 40)   ','  (|vy| > 40)
 *    else (rising)               : '\'' (|vy| ≤ 40)   '^'  (|vy| > 40)
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
 * PatternParams — physics + visual knobs that distinguish one fountain
 * style from another. The simulation engine never branches on pattern
 * TYPE; it reads these fields and behaves accordingly. Same code, four
 * very different fountains.
 *
 *   target_drops     : steady-state count of active drops.
 *                      drops_emit_to_target refills toward this each
 *                      tick (with a per-tick cap so a long pause does
 *                      not flood the pool when the loop resumes).
 *                      Higher = denser stream.
 *   source_top       : true  → spawn near top edge (WATERFALL falls
 *                              INTO the frame from above).
 *                      false → spawn just above the bottom HUD row.
 *                      Also flips the colour-ramp direction in
 *                      drop_height_fraction (top bright vs apex bright).
 *   source_x_spread  : ± cells around horizontal centre where new
 *                      drops appear. Tight (GEYSER ≈ 2) → a column;
 *                      wide (WATERFALL = 28) → a sheet across the top.
 *   speed_init       : nominal initial speed magnitude (cells/sec).
 *                      ACTUAL spawn speed is jittered ±15 % around
 *                      this — see DROP_SPEED_VARIANCE.
 *   cone_half_angle  : half of cone opening, in radians. Spawn draws
 *                      α uniformly ∈ [-half, +half]. 0.10 ≈ 6° (narrow
 *                      jet); 0.75 ≈ 43° (wide spray).
 *   upward           : true  → vy_init = -speed·cos α (cone points up;
 *                              gravity arcs drops back down).
 *                      false → vy_init = +speed·cos α (cone points
 *                              down; WATERFALL drops fall with extra g).
 *   gravity          : downward acceleration (cells/sec²). Tunes arc
 *                      height for a given speed_init — bigger g =
 *                      flatter arcs, faster fall.
 *   life_max         : hard cap on drop lifetime (sec). Prevents
 *                      zombies that drift sideways forever (e.g. when
 *                      |vx| > 0 but vy stays near zero).
 *   hot_palette      : true  → LAVA ramp (orange/red) + warm splash
 *                              colour. VOLCANIC only.
 *                      false → WATER ramp (blue/cyan).
 *   splash_mul       : scales SPLASH_BASE_PER_DROP at impact.
 *                      VOLCANIC's 1.40 = chunky lava splatters;
 *                      GEYSER's 0.50 = thin water mist.
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

    { "OCEANIC",  {  24,  30,  31,  38,  44,  51,  87, 159 },        {  30,  36,  73,  79, 122, 159, 195, 231 },     159,  234 },
    { "MATRIX",   {  28,  34,  40,  46,  82, 118, 154, 190 },        {  58,  64, 100, 106, 142, 184, 190, 226 },     154,  234 },
    { "NEON",     {  53,  91, 134, 165, 201, 207, 213, 219 },        { 198, 199, 200, 201, 207, 213, 219, 225 },     219,  234 },
    { "FIRE",     {  52,  88, 124, 160, 196, 202, 208, 214 },        {  88, 124, 160, 196, 202, 208, 214, 226 },     214,  234 },
    { "ICE",      {  24,  31,  67, 110, 117, 153, 195, 231 },        { 117, 153, 159, 195, 230, 231, 254, 255 },     231,  235 },
    { "NOVA",     {  24,  75, 117, 159, 195, 219, 226, 231 },        { 130, 166, 202, 208, 214, 220, 226, 231 },     231,  234 },
    { "SUNSET",   {  95, 131, 167, 174, 210, 217, 224, 230 },        {  88, 124, 160, 166, 202, 208, 214, 220 },     217,  234 },
    { "FOREST",   {  28,  64,  70,  76, 112, 148, 184, 220 },        {  94, 130, 136, 172, 208, 214, 220, 226 },     184,  234 },
    { "AMETHYST", {  54,  91,  92,  98, 134, 141, 177, 219 },        {  88, 125, 162, 199, 200, 207, 213, 219 },     213,  234 },
    { "ECLIPSE",  {  52,  88,  95, 131, 167, 173, 209, 215 },        {  52,  88, 124, 160, 196, 202, 208, 214 },     209,  232 },
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

/*
 * Drop — one water particle ejected from the source on its parabolic arc.
 *
 *   x, y    : current position in cells (float for sub-cell precision).
 *             The fractional bits are what make the arc read as smooth
 *             motion instead of snapping between integer rows each tick;
 *             round-to-nearest happens once, at render time, in drop_render.
 *   vx, vy  : velocity in cells/sec. Sign convention: vy < 0 = RISING
 *             (ncurses y increases downward), vy > 0 = FALLING. The pair
 *             encodes both the cone angle the drop was launched at AND
 *             its instantaneous position along the arc — vy ≈ 0 is the
 *             apex, which drop_glyph_for_velocity renders as '*'.
 *   age     : seconds since spawn. drops_integrate_and_cull tests this
 *             against pattern.life_max so drops that drift sideways with
 *             a tiny vy still eventually die.
 *   active  : pool-slot occupancy flag. Inactive slots are skipped by
 *             every loop; spawn finds the first inactive index via
 *             linear scan (cheap at MAX_DROPS = 900).
 */
typedef struct {
    float x, y;
    float vx, vy;
    float age;
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
 * drop_glyph_for_velocity — pick a glyph that reads as motion
 * direction at terminal resolution. Five outcomes, thresholds at
 * |vy| = 6 (apex band) and |vy| = 40 (slow / fast band):
 *
 *   |vy| <  6                : '*'   apex / nearly stopped — the
 *                                    brightest moment of the arc
 *   vy < 0,  6 ≤ |vy| ≤ 40   : '\''  rising, slow
 *   vy < 0,      |vy| > 40   : '^'   rising, fast
 *   vy > 0,  6 ≤ |vy| ≤ 40   : '.'   falling, slow
 *   vy > 0,      |vy| > 40   : ','   falling, fast
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

/*
 * Splash — short-lived fragment emitted at the impact point when a Drop
 * hits the ground row. Same kinematics as Drop, plus an explicit `life`
 * field so each fragment fades at its own random pace.
 *
 *   x, y    : current position in cells. Initialised to (impact_x,
 *             impact_y) by scene_emit_splashes.
 *   vx, vy  : velocity in cells/sec. vy starts NEGATIVE (upward kick of
 *             magnitude SPLASH_KICK_UP × random ∈ [0.6, 1.0]); vx is
 *             symmetric around zero (±SPLASH_KICK_X) so fragments fan
 *             outward from the impact. During integration vx decays
 *             exponentially via SPLASH_DRAG while vy accumulates
 *             SPLASH_GRAVITY → the classic bounce-and-fall trajectory.
 *   age     : seconds since this splash was emitted.
 *   life    : random target lifetime drawn at spawn from
 *             [SPLASH_LIFE_MIN, SPLASH_LIFE_MAX]. When age >= life the
 *             splash deactivates. Per-splash randomness scatters the
 *             death moments — what reads as organic fade. life is also
 *             the denominator in the glyph/attr lookup (see
 *             splash_life_phase): fresh '*' → mid '+' → dying '.'.
 *   active  : pool-slot occupancy flag (same role as Drop.active).
 */
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

/* Count currently-occupied drop slots — the feedback signal for the
 * Reeves emission-rate controller below. */
static int drops_count_active(const Scene *s)
{
    int n = 0;
    for (int i = 0; i < MAX_DROPS; i++) if (s->drops[i].active) n++;
    return n;
}

/*
 * drops_emit_to_target — Reeves emission step.
 * Refill the drop pool toward pp->target_drops, but cap emissions per
 * tick at (target × dt × 4 + 4) so a frame stall or a long pause does
 * not release a flood when the loop resumes — the cap is proportional
 * to dt so steady-state emission rate is dt-independent.
 */
static void drops_emit_to_target(Scene *s, float dt)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    int active    = drops_count_active(s);
    int target    = pp->target_drops;
    if (target > MAX_DROPS) target = MAX_DROPS;
    int spawn_cap = (int)((float)pp->target_drops * dt * 4.0f) + 4;
    int to_spawn  = target - active;
    if (to_spawn > spawn_cap) to_spawn = spawn_cap;
    for (int k = 0; k < to_spawn; k++) scene_spawn_drop(s);
}

/*
 * drop_step_ballistic — explicit Euler integration of one drop under
 * constant downward gravity:
 *     vy ← vy + g·dt
 *     (x, y) ← (x + vx·dt, y + vy·dt)
 *     age ← age + dt
 * This is the discretised form of the closed-form parabolic projectile.
 */
static void drop_step_ballistic(Drop *d, float gravity, float dt)
{
    d->vy  += gravity * dt;
    d->x   += d->vx   * dt;
    d->y   += d->vy   * dt;
    d->age += dt;
}

/*
 * drops_integrate_and_cull — advance every drop one step and reap those
 * that died this frame. Death causes (checked in this order):
 *   - age exceeds pattern.life_max          (timeout — drifting drops die)
 *   - drifted off screen sideways           (off-domain cull, ±8 cells slack)
 *   - reached the ground row → emit splash  (impact event, the visual payoff)
 */
static void drops_integrate_and_cull(Scene *s, float dt)
{
    const PatternParams *pp = &pattern_params[s->current_pattern];
    float kill_y = (float)(s->rows - 2);

    for (int i = 0; i < MAX_DROPS; i++) {
        Drop *d = &s->drops[i];
        if (!d->active) continue;

        drop_step_ballistic(d, pp->gravity, dt);

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
}

/*
 * splash_step_kinematic — Euler integration of one splash under gravity
 * with linear-velocity drag:
 *     vy ← vy + g·dt                  (constant gravity)
 *     vx ← vx · exp(-k·dt)            (closed-form decay over dt)
 *     (x, y) ← (x + vx·dt, y + vy·dt)
 *     age ← age + dt
 * The drag factor is precomputed once per tick by the caller — it is
 * the same for every active splash this frame.
 */
static void splash_step_kinematic(Splash *sp, float drag_factor, float dt)
{
    sp->vy  += SPLASH_GRAVITY * dt;
    sp->vx  *= drag_factor;
    sp->x   += sp->vx * dt;
    sp->y   += sp->vy * dt;
    sp->age += dt;
}

/* Advance every splash one step and deactivate those past their `life`
 * or that fell below the ground row. */
static void splashes_integrate_and_cull(Scene *s, float dt)
{
    float kill_y      = (float)(s->rows - 2);
    float drag_factor = expf(-SPLASH_DRAG * dt);

    for (int i = 0; i < MAX_SPLASHES; i++) {
        Splash *sp = &s->splashes[i];
        if (!sp->active) continue;
        splash_step_kinematic(sp, drag_factor, dt);
        if (sp->age >= sp->life || sp->y > kill_y + 1.0f)
            sp->active = false;
    }
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    dt *= (float)s->speed / (float)SPEED_DEF;

    drops_emit_to_target       (s, dt);
    drops_integrate_and_cull   (s, dt);
    splashes_integrate_and_cull(s, dt);
}

/*
 * drop_height_fraction — map a drop's vertical position to a [0,1]
 * colour-ramp coordinate, with 1.0 = brightest ramp end.
 *   - upward patterns: fraction = (source_y - drop.y) / (source_y + 1)
 *                      → 0 at the source row, 1 at the apex.
 *   - WATERFALL (source_top): fraction = 1 - drop.y / kill_y
 *                      → 1 at the top of the screen, 0 at the ground.
 * The +1 in the upward denominator avoids divide-by-zero when the
 * screen is so short that source_y = 0.
 */
static float drop_height_fraction(const Drop *d, const PatternParams *pp,
                                  int rows, float kill_y)
{
    float h;
    if (pp->source_top) {
        h = 1.0f - d->y / kill_y;
    } else {
        float source_y = (float)(rows - 3);
        h = (source_y - d->y) / (source_y + 1.0f);
    }
    if (h < 0.0f) h = 0.0f;
    if (h > 1.0f) h = 1.0f;
    return h;
}

/* Three-tier emphasis on the 8-step ramp: the bright end gets A_BOLD,
 * the dim end gets A_DIM, the middle stays A_NORMAL — pushes contrast
 * past what 8 palette entries alone can give. */
static int ramp_slot_attr(int slot)
{
    if (slot >= 6) return A_BOLD;
    if (slot <= 1) return A_DIM;
    return A_NORMAL;
}

/* Render one drop. Skips drops that round outside the drawable region. */
static void drop_render(const Drop *d, const PatternParams *pp,
                        int pair_base, int rows, int cols, float kill_y)
{
    int ix = (int)(d->x + 0.5f);
    int iy = (int)(d->y + 0.5f);
    if (ix < 0 || ix >= cols)     return;
    if (iy < 0 || iy >= rows - 1) return;

    float h_frac    = drop_height_fraction(d, pp, rows, kill_y);
    int   ramp_slot = (int)(h_frac * 7.0f + 0.5f);
    if (ramp_slot < 0) ramp_slot = 0;
    if (ramp_slot > 7) ramp_slot = 7;

    char glyph = drop_glyph_for_velocity(d->vy);
    int  attr  = ramp_slot_attr(ramp_slot);
    int  pair  = pair_base + ramp_slot;

    attron(COLOR_PAIR(pair) | attr);
    mvaddch(iy, ix, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

static void drops_render(const Scene *s, const PatternParams *pp,
                         int pair_base, float kill_y)
{
    for (int i = 0; i < MAX_DROPS; i++) {
        const Drop *d = &s->drops[i];
        if (!d->active) continue;
        drop_render(d, pp, pair_base, s->rows, s->cols, kill_y);
    }
}

/*
 * splash_life_phase — map a splash's age/life ratio ∈ [0, 1] to a
 * (glyph, attr) pair so each splash visibly fades over its lifetime:
 *     [0.00, 0.30) → '*' A_BOLD     (fresh, brightest)
 *     [0.30, 0.65) → '+' A_NORMAL   (mid-life)
 *     [0.65, 1.00] → '.' A_DIM      (dying)
 */
static void splash_life_phase(float life_ratio, char *out_glyph, int *out_attr)
{
    if      (life_ratio < 0.30f) { *out_glyph = '*'; *out_attr = A_BOLD;   }
    else if (life_ratio < 0.65f) { *out_glyph = '+'; *out_attr = A_NORMAL; }
    else                         { *out_glyph = '.'; *out_attr = A_DIM;    }
}

static void splash_render(const Splash *sp, const PatternParams *pp,
                          int rows, int cols)
{
    int ix = (int)(sp->x + 0.5f);
    int iy = (int)(sp->y + 0.5f);
    if (ix < 0 || ix >= cols)     return;
    if (iy < 0 || iy >= rows - 1) return;

    char glyph;
    int  attr;
    splash_life_phase(sp->age / sp->life, &glyph, &attr);

    /* VOLCANIC splashes glow with the brightest LAVA tint; the rest use
     * the dedicated PAIR_SPLASH (water-tinted white). */
    int splash_pair = pp->hot_palette ? (PAIR_LAVA_BASE + 6) : PAIR_SPLASH;
    attron(COLOR_PAIR(splash_pair) | attr);
    mvaddch(iy, ix, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(splash_pair) | attr);
}

static void splashes_render(const Scene *s, const PatternParams *pp)
{
    for (int i = 0; i < MAX_SPLASHES; i++) {
        const Splash *sp = &s->splashes[i];
        if (!sp->active) continue;
        splash_render(sp, pp, s->rows, s->cols);
    }
}

static void scene_draw(const Scene *s)
{
    const PatternParams *pp        = &pattern_params[s->current_pattern];
    int                  pair_base = pp->hot_palette ? PAIR_LAVA_BASE : PAIR_WATER_BASE;
    float                kill_y    = (float)(s->rows - 2);

    drops_render   (s, pp, pair_base, kill_y);
    splashes_render(s, pp);
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

/*
 * screen_draw — render the scene, then paint a two-layer HUD over it:
 *
 *   Row 0          — STATUS LINE.  Bright yellow PAIR_HUD + A_BOLD.
 *                    Shows pattern (or PAUSED), theme, active particle
 *                    counts, render fps, sim Hz, and the speed multiplier.
 *   Row rows-1     — KEY HINT LINE.  Bright cyan PAIR_HINT + A_BOLD.
 *                    Lists every interactive key the demo accepts so the
 *                    reader never has to dig through the source.
 *
 * Both rows are cleared with their pair colour first so the coloured
 * background fills the whole row even when the text is shorter than
 * sc->cols. Drawing the HUD AFTER scene_draw guarantees particles
 * never bleed through the bars.
 */
static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s);

    int drops, spls;
    scene_counts(s, &drops, &spls);

    const char *state_str = s->paused ? "PAUSED   " : pattern_name(s->current_pattern);

    /* ── Top row: status ──────────────────────────────────────── */
    char status[200];
    snprintf(status, sizeof status,
             " FOUNTAIN   %s   theme:%-9s   drops:%4d  splashes:%3d   "
             "%5.1f fps  %3d Hz  speed:%-3d ",
             state_str, themes[s->current_theme].name,
             drops, spls, fps, sim_fps, s->speed);

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(0, x, ' ');
    mvprintw(0, 0, "%s", status);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* ── Bottom row: key hints (every interactive key) ────────── */
    const char *hints =
        " q:quit  spc:pause  r:reseed  n/p:pattern  t/T:theme  +/-:speed  ]/[:Hz ";

    int hint_row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(hint_row, x, ' ');
    mvprintw(hint_row, 0, "%s", hints);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
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
