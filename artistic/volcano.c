/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * volcano.c — erupting volcano with lava bombs, ash plume, sparks,
 *             lava flows, and random AMBIENT BURSTS.  Heavy particle
 *             counts; ~600-800 active simultaneously for a visually
 *             dense eruption.
 *
 * DEMO: A stratovolcano fills the screen with a glowing crater at its
 *       summit.  Lava bombs trace ballistic arcs with smoke trails;
 *       embers drift up the rising plume; ash spreads slowly into the
 *       upper sky; sparks streak from the crater rim.  An fBm-modulated
 *       plume column rises and curls in the wind.  Lava flows trace
 *       glowing rivers down the mountain slopes.  Five rendering
 *       layers compose the final frame; one large particle pool
 *       (1024 slots) drives everything that moves.
 *
 *       Distinct from a single-particle-system demo: volcano is a
 *       LAYERED COMPOSITION — sky, mountain silhouette, lava flows,
 *       plume column, and particle effects are each their own pass.
 *       That layering plus the dense particle pool gives the dramatic
 *       eruption look that a one-pool one-type demo can't reach.
 *
 *       Eruption patterns (cycle with n / N):
 *         STROMBOLIAN  rhythmic moderate bursts, ~1-2 sec interval
 *         VULCANIAN    periodic violent explosions (large burst spawns)
 *         PLINIAN      continuous massive plume, heavy ash, tall column
 *         HAWAIIAN     strong lava fountain, lighter ash, slope flows
 *
 *       AMBIENT BURSTS — every 8-22 sec, a dramatic random surge fires
 *       regardless of pattern: ~28 bombs + ~18 sparks + ~40 embers all
 *       at once.  The interval is randomised per occurrence, so the
 *       pacing is unpredictable.  Adds drama to even the quietest
 *       patterns.
 *
 *       Themes (cycle with t / T):
 *         DAY       blue sky, mountain silhouette, orange lava
 *         DUSK      sunset orange/pink sky, dramatic dark silhouette
 *         NIGHT     dark sky, bright white-hot lava (most dramatic)
 *         MARS      red planet — pink sky, rust mountain, white-hot lava
 *         ASHFALL   grey muted everything, heavy ash dominance
 *         NEGATIVE  white-paper bg, dark fg (silhouette study)
 *
 * Section map:
 *   §1 config       — themes, patterns, particle counts, geometry
 *   §2 clock        — monotonic timer + sleep
 *   §3 color        — sky gradient + heat ramp + mountain palette
 *   §4 vec2 + util  — 2-D math, RNG
 *   §5 noise        — 2-D fBm (for plume + mountain surface)
 *   §6 mountain     — heightmap silhouette, crater, lava-flow streams
 *   §7 particles    — single pool, 4 types, physics + render
 *   §8 plume        — fBm density column with wind advection
 *   §9 scene        — compose layers + per-frame update
 *   §10 screen + app
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume eruption
 *   r            reseed mountain shape + clear particles
 *   n / N        next / previous eruption pattern
 *   t / T        next / previous theme
 *   + / =        eruption intensity up
 *   -            eruption intensity down
 *   ] / [        sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/volcano.c \
 *       -o volcano -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Five-layer composition with a shared particle pool.
 *                  Each frame, in back-to-front order:
 *
 *                    1. SKY: vertical gradient over the visible region,
 *                       theme-tinted (deep blue → cyan → pale near
 *                       horizon, or sunset / night / mars equivalents).
 *
 *                    2. MOUNTAIN: heightmap silhouette painted as a
 *                       solid-coloured shape from `silhouette_y(col)`
 *                       down to the bottom row.  The silhouette is
 *                       generated once at startup from a parametric
 *                       cone profile + 1-D fBm noise on the surface.
 *                       Crater is a small dip near the peak.
 *
 *                    3. LAVA FLOWS: a few streamlines traced down the
 *                       outer slopes from the crater rim.  Each cell
 *                       on a flow is painted hot (orange/yellow) with
 *                       gentle rate-of-temperature falloff with
 *                       distance from the crater.
 *
 *                    4. PLUME: a column of fBm noise above the crater.
 *                       Cells where the density is above threshold
 *                       paint as ash glyph in plume colour; the column
 *                       drifts in the +x direction (wind), so each
 *                       frame the texture shifts as a coherent flow.
 *
 *                    5. PARTICLES: one pool of 1024 slots, four types
 *                       sharing one struct:
 *                         BOMB   — launched from crater with random
 *                                  ballistic velocity, gravity pulls
 *                                  it down, stores a 4-tail trail.
 *                         EMBER  — spawned in the plume column with
 *                                  upward drift + lateral noise; cools
 *                                  with age.
 *                         ASH    — like ember but slower, larger lateral
 *                                  drift, fades to grey.
 *                         SPARK  — fast small particles near crater rim,
 *                                  short lifetime, very bright.
 *
 *                  Pattern selection (`n`/`N`) varies the SPAWN RATES
 *                  per type and the burst behaviour (e.g. VULCANIAN
 *                  occasionally dumps 60 bombs at once).
 *
 *                  Theme selection (`t`/`T`) re-applies the colour
 *                  palette (sky + mountain + lava ramp) without
 *                  rebuilding any geometry.
 *
 * Data-structure : Static `Particle pool[1024]` with type tag, position,
 *                  velocity, age, life, temperature.  `Mountain g_mtn`
 *                  with `silhouette_y[cols]` array.  All in BSS, no
 *                  malloc.  Particle pool dominates memory: 1024 × 56
 *                  bytes = ~57 KB.
 *
 * Rendering      : ASCII only.  Heat ramp `' .,:;-+*#@'` for hot
 *                  particles; sky gradient via cell-row indexing; ash
 *                  uses sparse glyphs (`,` `.` `:`); mountain silhouette
 *                  uses block-like chars (`@` `#`).
 *
 * Performance    : Per frame: 1024 particle ticks (each ~12 ops) +
 *                  cols·rows sky fill + plume fBm sample at each
 *                  visible plume cell + a few hundred lava-flow cells.
 *                  At 80×24 ≈ 30 K ops per frame, trivially 60 fps.
 *
 * References     :
 *   • Pyle, D. M. (2015) — *Volcanoes: Encyclopedia of Earth's Living
 *     Systems*.  Eruption-style taxonomy (Strombolian, Vulcanian,
 *     Plinian, Hawaiian) and ejecta classification.
 *   • Reeves, W. T. (1983) — "Particle Systems: A Technique for
 *     Modelling a Class of Fuzzy Objects", *ACM TOG* 2(2):91-108.
 *     The foundational particle-system paper; we use the same pool +
 *     active-flag pattern.
 *   • Witkin, A. & Heckbert, P. (1994) — "Using Particles to Sample
 *     and Control Implicit Surfaces", *SIGGRAPH '94*.  Inspiration
 *     for the layered-particle-system approach.
 *   • Mandelbrot, B. — *The Fractal Geometry of Nature* (1982), §28.
 *     Multi-octave fBm for the plume + mountain surface noise.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A volcano demo is FIVE separate things layered on top of each other:
 * sky behind, mountain shape, hot lava streams down slopes, smoke plume
 * rising up, particles flying everywhere.  Treat each as its own pass
 * with its own simple rules; let them composite.  No one master loop
 * tries to do "the eruption" — each layer just does its piece, and the
 * pile of layers IS the eruption.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine you're painting an erupting volcano.  You start with a sky
 * background, paint the mountain silhouette over it, drag bright lines
 * of orange down the slopes for lava flows, smudge a column of grey
 * smoke above the peak, then dot in dozens of bright orange specks
 * (bombs) following arcs across the sky and tiny embers drifting up
 * through the smoke.  Each step is simple; together they're an
 * eruption.  That's exactly the rendering pipeline.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. STARTUP — generate `silhouette_y[cols]`:
 *
 *       For each column x:
 *           cone_y = peak_y + |x − crater_x| · slope
 *           noise  = fbm_1d(x · scale, seed) · amp
 *           silhouette_y[x] = cone_y + noise
 *
 *     Crater is a small dip in `silhouette_y` around `crater_x`.
 *
 *  2. PER FRAME, IN ORDER:
 *
 *       a. Erase screen.
 *
 *       b. Sky fill:
 *          for each (row, col):
 *              if row < silhouette_y[col]:
 *                  sky_slot = (row * 4) / silhouette_y[col]
 *                  paint with theme.sky[sky_slot]
 *
 *       c. Mountain fill:
 *          for each col:
 *              y_top = silhouette_y[col]
 *              for row = y_top to rows-1:
 *                  paint with theme.mountain (with surface noise
 *                  modulating brightness)
 *
 *       d. Lava flows (HAWAIIAN + sometimes others):
 *          For each pre-traced flow path { (x, y) }:
 *              paint with theme.lava ramp by distance-from-crater
 *
 *       e. Plume column:
 *          For each (row, col) above silhouette_y[col]:
 *              x_w = (col − crater_x) − wind · time
 *              y_w = (silhouette_y[crater_x] − row)
 *              if (within plume cone)
 *                  density = fbm2d(x_w · s, y_w · s + time · drift)
 *                  density *= cone_falloff(x_w, y_w)
 *                  if density > threshold:
 *                      paint plume glyph + theme.plume colour
 *
 *       f. Particles:
 *          For each active particle:
 *              update_physics(p, dt)
 *              project to screen, paint glyph + colour
 *              draw bomb trails as fading dots
 *
 *       g. HUD on bottom row.
 *
 *  3. SPAWN RULES (per pattern):
 *
 *       BOMBS:    rate scales with intensity; angle uniform around
 *                 vertical-up; speed gives apex 0.5-0.8 of screen height.
 *       EMBERS:   spawned in a small disc around crater; upward bias.
 *       ASH:      higher up the plume column; slow horizontal drift.
 *       SPARKS:   crater rim; fast outward jets.
 *
 *       VULCANIAN: every 4-6 sec, dump 60-80 BOMBS in one frame for a
 *                  violent burst.
 *
 *  4. PARTICLE PHYSICS (per type):
 *
 *       BOMB:   v.y += GRAVITY · dt  ; v *= drag  ; pos += v · dt
 *               temp -= cooling · dt ; trail.shift_in(pos)
 *       EMBER:  v.y −= BUOYANCY · dt ; v.x += wind · dt + jitter
 *               temp -= cooling · dt
 *       ASH:    v.y −= small_buoy · dt ; v.x += wind · 0.6 · dt
 *               (cools to grey faster than ember)
 *       SPARK:  fast initial speed; fast cooling; lifetime <1s
 *
 *  5. CYCLE: n/N → swap pattern (different spawn rates).  t/T → swap
 *     theme (re-apply colour pairs).  r → reseed mountain + clear
 *     particles.  +/− → intensity up/down.  Pause freezes physics.
 *
 * KEY FORMULAS
 * ────────────
 *  Stratovolcano cone (parametric):
 *    silhouette_y(x) = peak_y + |x − crater_x| · slope + fbm_1d(x · s)·amp
 *
 *  Crater dip (small bowl at the summit):
 *    if |x − crater_x| < crater_radius:
 *        silhouette_y(x) += crater_depth · cos(π · (x − crater_x) / crater_radius)
 *
 *  Bomb ballistic arc:
 *    v_y(t) = v_y(0) + g · t                 // g positive (screen y down)
 *    y(t)   = y(0) + v_y(0) · t + ½ g t²
 *
 *  Plume cone-falloff (so the plume doesn't extend forever sideways):
 *    cone_falloff(dx, dy) = exp(−(dx²/(spread·dy + base)²))
 *
 *  Aspect correction (any 2-D shape that should look round in pixels):
 *    dy_pixels = dy_cells · CELL_ASPECT
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • POOL EXHAUSTION.  Spawn rate · lifetime > pool size → newest
 *    spawns drop on the floor.  We ALSO clamp per-frame spawn so a
 *    pause-then-resume doesn't dump the entire pool in one frame.
 *
 *  • CRATER BORDER.  The crater dip in `silhouette_y` makes a small
 *    bowl at the peak; particles spawn from the bowl's bottom row,
 *    not from the cone's mathematical peak.  This is the visible
 *    "crater glow" anchor point.
 *
 *  • SLOPE LAVA AT CRATER.  Lava flows are pre-traced from the crater
 *    rim DOWN one or both sides.  We don't simulate fluid dynamics —
 *    just paint cells along the precomputed path.
 *
 *  • PLUME CONE.  Without the `cone_falloff` term, fBm density would
 *    spread the plume across the whole sky.  The cone restricts it to
 *    a roughly conical region above the crater, widening with altitude.
 *
 *  • WIND DRIFT.  Plume noise samples at `(x − wind·time)`, so the
 *    pattern flows in +x consistently; the column appears to drift
 *    sideways as it rises.  Same trick used for cloud demos.
 *
 *  • PAUSE freezes ALL physics — bombs hang in mid-air, plume stops
 *    drifting.  Resume picks up cleanly.
 *
 *  • THEME CYCLE doesn't reset the particle pool, just re-applies
 *    colour pairs.  Active particles immediately take new colours.
 *
 *  • INVERTED THEME (NEGATIVE).  Pre-fill white "paper" before drawing,
 *    use dark fg pairs, disable A_BOLD/A_DIM (would invert intent on
 *    light fg over white bg).  Standard repo recipe.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • At default (STROMBOLIAN, DAY) you should see a mountain with
 *    ~30 bombs in arcs, ~200 embers in plume, ~150 ash, plus crater
 *    glow and a steady column rising upward.
 *
 *  • Press space.  Everything freezes — bombs in mid-air, plume frozen.
 *    Resume — clean continuation.
 *
 *  • Press `n` to VULCANIAN.  Within a few seconds you'll see a
 *    violent burst — 60+ bombs erupt simultaneously, then quiet again
 *    for 4-6 sec, then another burst.
 *
 *  • Press `n` to PLINIAN.  Plume becomes huge — ash count triples,
 *    column reaches near top of screen, bombs less frequent.
 *
 *  • Press `n` to HAWAIIAN.  Lava flows down slopes brighten; bombs
 *    arc lower (modest fountain); ash drops considerably.
 *
 *  • Press `t` cycles theme.  Mountain silhouette stays the same;
 *    only colours swap.  NIGHT is most dramatic — bright lava on
 *    dark sky.
 *
 *  • Press `r`.  Mountain regenerates with a different surface profile
 *    + crater position; particles clear and respawn.
 *
 *  • Press `+`.  Spawn rates increase; particle count rises until the
 *    pool saturates.  Press `-` to back down.
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

    FPS_UPDATE_MS    = 500,

    PAIR_HUD          =  1,
    PAIR_HINT         =  2,
    PAIR_SKY_BASE     =  3,    /* +0..+3 — top→horizon sky gradient    */
    PAIR_MOUNTAIN     =  7,    /* mountain silhouette                  */
    PAIR_LAVA_BASE    =  8,    /* +0..+5 — heat ramp dim→hot           */
    PAIR_PLUME_BASE   = 14,    /* +0..+3 — plume gradient core→edge    */
    PAIR_PAPER        = 18,    /* NEGATIVE white-paper bg              */

    /* Particle pool. */
    PARTICLES_MAX     = 1024,

    /* Mountain heightmap maximum width. */
    MTN_MAX_W         = 280,

    /* Plume buffer dimensions (mirror screen extents up to limits). */
    SCREEN_MAX_W      = 280,
    SCREEN_MAX_H      = 90,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define CELL_ASPECT      2.0f      /* terminal cell h/w                  */

/* Mountain. */
#define MTN_PEAK_FRAC    0.42f     /* peak y as fraction of rows         */
#define MTN_SLOPE        0.55f     /* rise per col away from peak        */
#define MTN_NOISE_AMP    1.4f      /* 1-D surface noise amplitude (rows) */
#define MTN_NOISE_SCALE  0.18f     /* 1-D surface noise scale            */
#define CRATER_RADIUS    4.0f      /* cells; the bowl at the summit      */
#define CRATER_DEPTH     2.0f      /* dip at crater centre (rows)        */

/* Plume. */
#define PLUME_BASE_W     5.0f      /* base width at crater rim (cells)   */
#define PLUME_SPREAD     0.40f     /* width grows with altitude          */
#define PLUME_NOISE_S    0.18f     /* fBm domain scale                   */
#define PLUME_WIND       2.5f      /* +x drift speed (cells/sec)         */
#define PLUME_RISE       1.8f      /* column upward drift in noise space */
#define PLUME_THRESHOLD  0.45f     /* fbm above this → visible plume     */

/* Particle physics. */
#define GRAVITY          24.0f     /* +y, cells/sec²                     */
#define BOMB_DRAG        0.04f     /* per second                         */
#define EMBER_BUOYANCY   12.0f     /* upward acceleration                */
#define ASH_BUOYANCY      4.0f
#define COOL_BOMB         0.18f    /* temp decay per sec                 */
#define COOL_EMBER        0.30f
#define COOL_SPARK        2.50f    /* sparks fade fast                   */

/* Ranges per particle type for spawn-time randomisation. */
#define BOMB_LIFE_MIN     2.0f
#define BOMB_LIFE_MAX     4.0f
#define BOMB_SPEED_MIN   18.0f
#define BOMB_SPEED_MAX   30.0f
#define BOMB_ANGLE       1.05f     /* ±radians from straight up          */

#define EMBER_LIFE_MIN    3.0f
#define EMBER_LIFE_MAX    6.0f
#define EMBER_SPEED_MIN   3.0f
#define EMBER_SPEED_MAX   8.0f

#define ASH_LIFE_MIN      6.0f
#define ASH_LIFE_MAX     12.0f
#define ASH_SPEED_MIN     2.0f
#define ASH_SPEED_MAX     5.0f

#define SPARK_LIFE_MIN    0.4f
#define SPARK_LIFE_MAX    0.9f
#define SPARK_SPEED_MIN  18.0f
#define SPARK_SPEED_MAX  35.0f

/* Bomb trail length. */
#define BOMB_TRAIL_LEN   4

/* Eruption pattern.  Each pattern is a row of spawn rates + one flag. */
typedef enum {
    PAT_STROMBOLIAN = 0,
    PAT_VULCANIAN   = 1,
    PAT_PLINIAN     = 2,
    PAT_HAWAIIAN    = 3,
    N_PATTERNS      = 4,
} Pattern;

typedef struct {
    const char *name;
    float bomb_per_sec;
    float ember_per_sec;
    float ash_per_sec;
    float spark_per_sec;
    float plume_intensity;     /* multiplier on plume density            */
    bool  vulcanian_bursts;    /* periodic large bomb dumps              */
    float lava_flow_amount;    /* how brightly the slope flows render    */
} PatternParams;

static const PatternParams patterns[N_PATTERNS] = {
    /* name           bomb  ember  ash    spark  plume  burst  flow  */
    /* STROMBOLIAN — moderate rhythmic                              */
    { "STROMBOLIAN", 12.0f,  60.0f,  35.0f,  30.0f, 1.0f,  false, 0.30f },
    /* VULCANIAN   — periodic violent bursts                        */
    { "VULCANIAN  ",  4.0f,  35.0f,  25.0f,  18.0f, 0.8f,  true,  0.20f },
    /* PLINIAN     — continuous massive plume                       */
    { "PLINIAN    ",  6.0f,  90.0f,  90.0f,  20.0f, 1.8f,  false, 0.25f },
    /* HAWAIIAN    — strong fountain + lava flows, lighter ash     */
    { "HAWAIIAN   ", 22.0f, 100.0f,  20.0f,  60.0f, 0.6f,  false, 0.95f },
};

#define VULCAN_BURST_INTERVAL_MIN  4.0f
#define VULCAN_BURST_INTERVAL_MAX  7.0f
#define VULCAN_BURST_BOMBS         55

/*
 * AMBIENT BURST — a random-frequency dramatic eruption that fires for
 * EVERY pattern (not just VULCANIAN).  Wide interval distribution so
 * sometimes you get 8 sec of calm and sometimes 25 sec — keeps the
 * timing unpredictable, which is the point of "occasional drama".
 *
 * Particle count is moderate (smaller than VULCANIAN's 55-bomb dump)
 * so that VULCANIAN bursts still feel distinctly violent compared
 * with the universal ambient ones.
 */
#define AMBIENT_BURST_INTERVAL_MIN  8.0f
#define AMBIENT_BURST_INTERVAL_MAX 22.0f
#define AMBIENT_BURST_BOMBS         28
#define AMBIENT_BURST_SPARKS        18
#define AMBIENT_BURST_EMBERS        40

/* Theme. */
typedef struct {
    const char *name;
    short       sky[4];        /* top → horizon                         */
    short       mountain[3];   /* shadow / mid / lit                    */
    short       lava[6];       /* dim → white-hot                        */
    short       plume[4];      /* dense → wispy                          */
    bool        inverted;
} Theme;

#define N_THEMES 6

static const Theme themes[N_THEMES] = {
    /* DAY: cobalt sky → cyan, grey-rock mountain, classic orange lava,
     * grey ash plume.                                                  */
    { "DAY      ",
      {  24,  31,  39,  75 },
      { 240, 244, 248 },
      {  88, 124, 196, 208, 220, 226 },
      { 240, 244, 248, 252 }, false },

    /* DUSK: sunset gradient (deep magenta → orange → pale), dark
     * mountain silhouette, vivid lava (pops on dark), warm ash.        */
    { "DUSK     ",
      {  53,  88, 167, 215 },
      {  60,  66,  72 },
      { 124, 160, 196, 208, 220, 229 },
      { 235, 240, 244, 248 }, false },

    /* NIGHT: deep navy sky, dark mountain, hottest lava (white-hot
     * core), dim grey plume.                                           */
    { "NIGHT    ",
      {  17,  18,  24,  60 },
      {  60,  66,  72 },
      {  88, 124, 196, 220, 226, 231 },
      { 234, 238, 242, 246 }, false },

    /* MARS: pink-rust sky, rust mountain, white-hot lava, brown ash.   */
    { "MARS     ",
      { 124,  88, 130, 173 },
      { 130, 137, 173 },
      { 130, 166, 208, 220, 226, 231 },
      { 137, 144, 173, 180 }, false },

    /* ASHFALL: muted grey throughout, plume dominates.                 */
    { "ASHFALL  ",
      { 235, 240, 244, 247 },
      { 234, 238, 242 },
      {  88, 130, 166, 202, 208, 214 },
      { 248, 250, 252, 254 }, false },

    /* NEGATIVE: white-paper bg, dark fg, photographic-negative look.   */
    { "NEGATIVE ",
      { 253, 250, 247, 244 },
      { 240, 237, 234 },
      { 232, 234, 237, 240, 243,  16 },
      { 237, 240, 244, 247 }, true  },
};

/* Glyph ramps. */
static const char LAVA_GLYPHS[6]   = { '.', ':', '+', '*', '#', '@' };
static const char EMBER_GLYPHS[6]  = { '.', ',', ':', ';', '+', '*' };
static const char ASH_GLYPHS[4]    = { '.', ',', ':', ';' };
static const char PLUME_GLYPHS[4]  = { ',', ':', '+', '*' };
static const char SPARK_GLYPHS[3]  = { '.', '*', '#' };
static const char MTN_GLYPH        = '#';
static const char MTN_RIDGE_GLYPH  = '@';   /* topmost mountain pixel  */

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
    const Theme *t = &themes[idx];
    short bg = t->inverted ? 231 : -1;

    if (COLORS >= 256) {
        for (int i = 0; i < 4; i++)
            init_pair((short)(PAIR_SKY_BASE + i), t->sky[i], bg);
        init_pair(PAIR_MOUNTAIN, t->mountain[1], bg);   /* one dominant tone */
        for (int i = 0; i < 6; i++)
            init_pair((short)(PAIR_LAVA_BASE + i), t->lava[i], bg);
        for (int i = 0; i < 4; i++)
            init_pair((short)(PAIR_PLUME_BASE + i), t->plume[i], bg);
        init_pair(PAIR_PAPER, 16, t->inverted ? 231 : -1);
    } else {
        for (int i = 0; i < 4; i++)
            init_pair((short)(PAIR_SKY_BASE + i), COLOR_BLUE,    -1);
        init_pair(PAIR_MOUNTAIN, COLOR_WHITE, -1);
        static const short fb_lava[6] = {
            COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW,
            COLOR_YELLOW, COLOR_WHITE,
        };
        for (int i = 0; i < 6; i++)
            init_pair((short)(PAIR_LAVA_BASE + i),
                      t->inverted ? COLOR_BLACK : fb_lava[i],
                      t->inverted ? COLOR_WHITE : (short)-1);
        for (int i = 0; i < 4; i++)
            init_pair((short)(PAIR_PLUME_BASE + i),
                      t->inverted ? COLOR_BLACK : COLOR_WHITE,
                      t->inverted ? COLOR_WHITE : (short)-1);
        init_pair(PAIR_PAPER, COLOR_BLACK, COLOR_WHITE);
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
/* §4  vec2 + utilities                                                   */
/* ===================================================================== */

typedef struct { float x, y; } V2;

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Cheap LCG. */
static inline uint32_t lcg_next(uint32_t *st)
{
    *st = *st * 1664525u + 1013904223u;
    return *st;
}

static inline float lcg_unit(uint32_t *st)
{
    return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

static inline float lcg_range(uint32_t *st, float lo, float hi)
{
    return lo + (hi - lo) * lcg_unit(st);
}

static uint32_t g_rng = 0xCAFEBABEu;

/* ===================================================================== */
/* §5  noise — 2-D fBm (used for plume + 1-D for mountain surface)        */
/* ===================================================================== */

static inline uint32_t hash2d(int x, int z, uint32_t seed)
{
    uint32_t h = (uint32_t)x * 374761393u
               + (uint32_t)z * 668265263u
               + seed        * 2147483647u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}

static inline float hash_unit(int x, int z, uint32_t seed)
{
    return (float)(hash2d(x, z, seed) >> 8) / (float)(1u << 24);
}

static inline float smoothstep01(float t) { return t * t * (3.0f - 2.0f * t); }

static float vnoise2d(float x, float z, uint32_t seed)
{
    int   xi = (int)floorf(x), zi = (int)floorf(z);
    float fx = x - (float)xi,   fz = z - (float)zi;
    float v00 = hash_unit(xi,     zi,     seed);
    float v10 = hash_unit(xi + 1, zi,     seed);
    float v01 = hash_unit(xi,     zi + 1, seed);
    float v11 = hash_unit(xi + 1, zi + 1, seed);
    float sx  = smoothstep01(fx);
    float sz  = smoothstep01(fz);
    float a   = v00 * (1.0f - sx) + v10 * sx;
    float b   = v01 * (1.0f - sx) + v11 * sx;
    return a * (1.0f - sz) + b * sz;
}

static float fbm2d(float x, float z, uint32_t seed)
{
    float h = 0.0f, amp = 1.0f, freq = 1.0f, norm = 0.0f;
    for (int i = 0; i < 3; i++) {
        h    += amp * vnoise2d(x * freq, z * freq, seed + (uint32_t)i * 17u);
        norm += amp;
        amp  *= 0.5f;
        freq *= 2.0f;
    }
    return h / norm;
}

/* 1-D fBm for the mountain surface (just the y=0 slice). */
static float fbm1d(float x, uint32_t seed)
{
    return fbm2d(x, 0.0f, seed);
}

/* ===================================================================== */
/* §6  mountain — heightmap silhouette + crater + lava-flow streams       */
/* ===================================================================== */

typedef struct {
    int   silhouette_y[MTN_MAX_W]; /* row index of mountain top per col   */
    int   crater_x;                /* column of crater centre              */
    int   crater_y;                /* row of crater bottom (for spawning) */
    /* Pre-traced lava-flow paths — sets of (col, row) cells. */
    int   flow_l_x[MTN_MAX_W];     /* left flow path                       */
    int   flow_l_y[MTN_MAX_W];
    int   flow_l_n;
    int   flow_r_x[MTN_MAX_W];     /* right flow path                      */
    int   flow_r_y[MTN_MAX_W];
    int   flow_r_n;
} Mountain;

static Mountain g_mtn;

/*
 * mountain_build — generate stratovolcano silhouette + crater + lava
 * flow paths.  The mountain is a cone profile: peak at `crater_x`,
 * sides slope outward at `MTN_SLOPE` cells per col, with 1-D fBm
 * surface noise riding on top to break the perfect-cone look.
 *
 * Crater is a small cosine-weighted dip in the silhouette around
 * `crater_x` with depth `CRATER_DEPTH` rows.  `crater_y` (the
 * deepest row of the dip) is where particles spawn.
 *
 * Lava flow paths are pre-traced from the crater rim down each side
 * of the mountain.  Each step in the flow walks one column outward
 * from the crater and lands at silhouette_y[col] + small_offset to
 * sit just below the rim.  Stored once at startup; rendered each
 * frame as bright cells.
 */
static void mountain_build(Mountain *m, int cols, int rows, uint32_t seed)
{
    if (cols > MTN_MAX_W) cols = MTN_MAX_W;

    int peak_y   = (int)((float)rows * MTN_PEAK_FRAC);
    int crater_x = cols / 2 + (int)((lcg_unit(&seed) - 0.5f) * (float)cols * 0.3f);
    if (crater_x < cols / 4)         crater_x = cols / 4;
    if (crater_x > 3 * cols / 4)     crater_x = 3 * cols / 4;

    /* 1. Build cone + noise silhouette. */
    for (int x = 0; x < cols; x++) {
        float dx = (float)abs(x - crater_x);
        float cone = (float)peak_y + dx * MTN_SLOPE;
        float n = fbm1d((float)x * MTN_NOISE_SCALE, seed) - 0.5f;
        cone += n * 2.0f * MTN_NOISE_AMP;
        if (cone < 0)            cone = 0;
        if (cone > rows - 2)     cone = rows - 2;
        m->silhouette_y[x] = (int)(cone + 0.5f);
    }

    /* 2. Carve crater dip — cosine-weighted deepening. */
    for (int x = crater_x - (int)CRATER_RADIUS;
             x <= crater_x + (int)CRATER_RADIUS; x++) {
        if (x < 0 || x >= cols) continue;
        float dxn = ((float)(x - crater_x)) / CRATER_RADIUS;
        if (dxn < -1.0f || dxn > 1.0f) continue;
        float depth = CRATER_DEPTH * 0.5f * (1.0f + cosf((float)M_PI * dxn));
        m->silhouette_y[x] += (int)depth;
        if (m->silhouette_y[x] > rows - 2) m->silhouette_y[x] = rows - 2;
    }

    m->crater_x = crater_x;
    m->crater_y = m->silhouette_y[crater_x];

    /* 3. Pre-trace lava flows down each side from the crater rim.
     *    We start one step away from the crater on each side and walk
     *    outward, putting each flow cell just below the silhouette so
     *    it sits ON the mountain surface.  Length is the distance
     *    from crater to where the flow "exhausts" (a quarter the way
     *    to the edge or 16 columns, whichever is smaller). */
    int max_flow = (cols / 4 < 24) ? cols / 4 : 24;

    int n = 0;
    for (int dx = (int)CRATER_RADIUS + 1; dx < max_flow; dx++) {
        int x = crater_x - dx;
        if (x < 0 || x >= cols) break;
        m->flow_l_x[n] = x;
        m->flow_l_y[n] = m->silhouette_y[x] + 1;
        n++;
    }
    m->flow_l_n = n;

    n = 0;
    for (int dx = (int)CRATER_RADIUS + 1; dx < max_flow; dx++) {
        int x = crater_x + dx;
        if (x < 0 || x >= cols) break;
        m->flow_r_x[n] = x;
        m->flow_r_y[n] = m->silhouette_y[x] + 1;
        n++;
    }
    m->flow_r_n = n;
}

/* ===================================================================== */
/* §7  particles — single pool, four types                                */
/* ===================================================================== */

typedef enum {
    PT_BOMB  = 0,
    PT_EMBER = 1,
    PT_ASH   = 2,
    PT_SPARK = 3,
} ParticleType;

typedef struct {
    bool         active;
    ParticleType type;
    float        x, y;
    float        vx, vy;
    float        age, life;
    float        temp;        /* 0 = cold, 1 = white-hot                */
    /* Bomb trail — last BOMB_TRAIL_LEN positions. */
    float        tx[BOMB_TRAIL_LEN];
    float        ty[BOMB_TRAIL_LEN];
    int          trail_n;
} Particle;

static Particle g_pool[PARTICLES_MAX];

static int particle_alloc(void)
{
    for (int i = 0; i < PARTICLES_MAX; i++) {
        if (!g_pool[i].active) return i;
    }
    return -1;       /* pool full */
}

static void particles_clear(void)
{
    memset(g_pool, 0, sizeof g_pool);
}

/*
 * particle_spawn_bomb — random ballistic launch from crater.  Velocity
 * is angled within ±BOMB_ANGLE around vertical-up; speed gives an
 * apex around 0.5-0.8 of screen height (set by speed range).
 */
static void particle_spawn_bomb(int crater_x, int crater_y, float intensity)
{
    int idx = particle_alloc();
    if (idx < 0) return;
    Particle *p = &g_pool[idx];
    p->active = true;
    p->type   = PT_BOMB;
    p->x      = (float)crater_x + lcg_range(&g_rng, -1.5f, 1.5f);
    p->y      = (float)crater_y + 0.5f;
    float angle_off = lcg_range(&g_rng, -BOMB_ANGLE, BOMB_ANGLE);
    float speed     = lcg_range(&g_rng, BOMB_SPEED_MIN, BOMB_SPEED_MAX) * intensity;
    p->vx     = sinf(angle_off) * speed;
    p->vy     = -cosf(angle_off) * speed;
    p->age    = 0;
    p->life   = lcg_range(&g_rng, BOMB_LIFE_MIN, BOMB_LIFE_MAX);
    p->temp   = 1.0f;
    /* Initialise trail to current position so we don't see uninit data. */
    for (int k = 0; k < BOMB_TRAIL_LEN; k++) {
        p->tx[k] = p->x;
        p->ty[k] = p->y;
    }
    p->trail_n = 0;
}

static void particle_spawn_ember(int crater_x, int crater_y)
{
    int idx = particle_alloc();
    if (idx < 0) return;
    Particle *p = &g_pool[idx];
    p->active = true;
    p->type   = PT_EMBER;
    p->x      = (float)crater_x + lcg_range(&g_rng, -2.0f, 2.0f);
    p->y      = (float)crater_y;
    float speed = lcg_range(&g_rng, EMBER_SPEED_MIN, EMBER_SPEED_MAX);
    p->vx     = lcg_range(&g_rng, -1.5f, 1.5f);
    p->vy     = -speed;
    p->age    = 0;
    p->life   = lcg_range(&g_rng, EMBER_LIFE_MIN, EMBER_LIFE_MAX);
    p->temp   = 0.9f;
    p->trail_n = 0;
}

static void particle_spawn_ash(int crater_x, int crater_y)
{
    int idx = particle_alloc();
    if (idx < 0) return;
    Particle *p = &g_pool[idx];
    p->active = true;
    p->type   = PT_ASH;
    p->x      = (float)crater_x + lcg_range(&g_rng, -3.0f, 3.0f);
    p->y      = (float)crater_y - lcg_range(&g_rng, 0.0f, 4.0f);
    float speed = lcg_range(&g_rng, ASH_SPEED_MIN, ASH_SPEED_MAX);
    p->vx     = lcg_range(&g_rng, -2.0f, 2.0f);
    p->vy     = -speed;
    p->age    = 0;
    p->life   = lcg_range(&g_rng, ASH_LIFE_MIN, ASH_LIFE_MAX);
    p->temp   = 0.4f;
    p->trail_n = 0;
}

static void particle_spawn_spark(int crater_x, int crater_y)
{
    int idx = particle_alloc();
    if (idx < 0) return;
    Particle *p = &g_pool[idx];
    p->active = true;
    p->type   = PT_SPARK;
    p->x      = (float)crater_x + lcg_range(&g_rng, -2.0f, 2.0f);
    p->y      = (float)crater_y;
    float angle = lcg_range(&g_rng, -1.5f, 1.5f);
    float speed = lcg_range(&g_rng, SPARK_SPEED_MIN, SPARK_SPEED_MAX);
    p->vx     = sinf(angle) * speed;
    p->vy     = -cosf(angle) * speed * 0.8f;
    p->age    = 0;
    p->life   = lcg_range(&g_rng, SPARK_LIFE_MIN, SPARK_LIFE_MAX);
    p->temp   = 1.0f;
    p->trail_n = 0;
}

/*
 * particles_tick — physics for ALL active particles.  Branches on type
 * to apply the right force model + cooling rate.
 */
static void particles_tick(float dt, int rows, int cols)
{
    float bomb_drag = 1.0f - BOMB_DRAG * dt;
    if (bomb_drag < 0) bomb_drag = 0;

    for (int i = 0; i < PARTICLES_MAX; i++) {
        Particle *p = &g_pool[i];
        if (!p->active) continue;

        switch (p->type) {
        case PT_BOMB:
            /* Trail update — shift down, push current. */
            for (int k = BOMB_TRAIL_LEN - 1; k > 0; k--) {
                p->tx[k] = p->tx[k - 1];
                p->ty[k] = p->ty[k - 1];
            }
            p->tx[0] = p->x;
            p->ty[0] = p->y;
            p->trail_n = BOMB_TRAIL_LEN;

            p->vy += GRAVITY * dt;
            p->vx *= bomb_drag;
            p->vy *= bomb_drag;
            p->x  += p->vx * dt;
            p->y  += p->vy * dt;
            p->temp -= COOL_BOMB * dt;
            break;

        case PT_EMBER:
            p->vy -= EMBER_BUOYANCY * dt;
            p->vx += lcg_range(&g_rng, -2.0f, 2.0f) * dt;       /* turbulence  */
            p->vx += PLUME_WIND * 0.4f * dt;                    /* gentle wind */
            p->x  += p->vx * dt;
            p->y  += p->vy * dt;
            p->temp -= COOL_EMBER * dt;
            break;

        case PT_ASH:
            p->vy -= ASH_BUOYANCY * dt;
            p->vx += lcg_range(&g_rng, -1.0f, 1.0f) * dt;
            p->vx += PLUME_WIND * 0.6f * dt;
            p->x  += p->vx * dt;
            p->y  += p->vy * dt;
            p->temp -= 0.05f * dt;     /* very slow cooling */
            break;

        case PT_SPARK:
            p->vy += GRAVITY * 0.5f * dt;
            p->x  += p->vx * dt;
            p->y  += p->vy * dt;
            p->temp -= COOL_SPARK * dt;
            break;
        }

        p->age += dt;

        /* Death conditions. */
        if (p->age >= p->life)         p->active = false;
        else if (p->temp <= 0)         p->active = false;
        else if (p->y < -2.0f)         p->active = false;
        else if (p->y > (float)rows + 2.0f) p->active = false;
        else if (p->x < -3.0f || p->x > (float)cols + 3.0f) p->active = false;
    }
}

/* Simple per-particle render: pick glyph by temperature, paint cell.
 * Bomb trail is drawn first (back-to-front so head stamps over). */
static void particles_render(int rows, int cols, bool inverted)
{
    int rows_eff = rows - 1;       /* leave bottom row for HUD */

    int last_pair = -1;
    attr_t last_attr = 0;
    (void)last_pair;
    (void)last_attr;

    /* First pass — bomb trails (so heads overlap). */
    for (int i = 0; i < PARTICLES_MAX; i++) {
        const Particle *p = &g_pool[i];
        if (!p->active || p->type != PT_BOMB) continue;
        for (int k = 1; k < p->trail_n; k++) {
            int tx = (int)(p->tx[k] + 0.5f);
            int ty = (int)(p->ty[k] + 0.5f);
            if (tx < 0 || tx >= cols)     continue;
            if (ty < 0 || ty >= rows_eff) continue;
            int slot = (int)(p->temp * 5.999f) - k;
            if (slot < 0) slot = 0;
            if (slot > 5) slot = 5;
            char glyph = (k == 1) ? '*' : (k == 2 ? '+' : '.');
            attr_t a = inverted ? A_NORMAL : A_DIM;
            attron(COLOR_PAIR(PAIR_LAVA_BASE + slot) | a);
            mvaddch(ty, tx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(PAIR_LAVA_BASE + slot) | a);
        }
    }

    /* Second pass — heads + non-bomb particles. */
    for (int i = 0; i < PARTICLES_MAX; i++) {
        const Particle *p = &g_pool[i];
        if (!p->active) continue;
        int ix = (int)(p->x + 0.5f);
        int iy = (int)(p->y + 0.5f);
        if (ix < 0 || ix >= cols)     continue;
        if (iy < 0 || iy >= rows_eff) continue;

        char glyph;
        int  pair;
        attr_t attr;
        float temp = p->temp;
        if (temp < 0) temp = 0;
        if (temp > 1) temp = 1;

        switch (p->type) {
        case PT_BOMB: {
            int slot = (int)(temp * 5.999f);
            if (slot < 0) slot = 0;
            if (slot > 5) slot = 5;
            glyph = LAVA_GLYPHS[slot];
            pair  = PAIR_LAVA_BASE + slot;
            attr  = (slot >= 4 && !inverted) ? A_BOLD : A_NORMAL;
            break;
        }
        case PT_EMBER: {
            int slot = (int)(temp * 5.999f);
            if (slot < 0) slot = 0;
            if (slot > 5) slot = 5;
            glyph = EMBER_GLYPHS[slot];
            pair  = PAIR_LAVA_BASE + slot;
            attr  = (slot <= 1 && !inverted) ? A_DIM
                  : (slot >= 5 && !inverted) ? A_BOLD
                  :                             A_NORMAL;
            break;
        }
        case PT_ASH: {
            int slot = (int)(temp * 3.999f);
            if (slot < 0) slot = 0;
            if (slot > 3) slot = 3;
            glyph = ASH_GLYPHS[slot];
            pair  = PAIR_PLUME_BASE + slot;
            attr  = inverted ? A_NORMAL : A_DIM;
            break;
        }
        case PT_SPARK:
        default: {
            int slot = (int)(temp * 2.999f);
            if (slot < 0) slot = 0;
            if (slot > 2) slot = 2;
            glyph = SPARK_GLYPHS[slot];
            pair  = PAIR_LAVA_BASE + 4 + (slot >= 1 ? 1 : 0);
            attr  = inverted ? A_NORMAL : A_BOLD;
            break;
        }
        }
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(iy, ix, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | attr);
    }
}

/* ===================================================================== */
/* §8  plume — fBm density column with wind advection                     */
/* ===================================================================== */

/*
 * plume_render — for each cell ABOVE the mountain silhouette near the
 * crater, sample a 2-D fBm density.  If above threshold AND inside the
 * widening cone from crater upward, paint the cell.
 *
 * The fBm sample point drifts in x with `time · PLUME_WIND` (wind) and
 * in the noise's y-axis with `time · PLUME_RISE` (so the column itself
 * boils upward).  Combined, this gives a coherent flowing-column look.
 */
static void plume_render(const Mountain *mtn, int rows, int cols,
                          float time, float intensity, bool inverted)
{
    int rows_eff = rows - 1;
    int crater_x = mtn->crater_x;
    int crater_y = mtn->crater_y;

    for (int row = 0; row < rows_eff; row++) {
        if (row > crater_y) break;     /* below crater = inside mountain */

        /* Cone widening upward — height above crater determines width. */
        float h = (float)(crater_y - row);
        if (h < 0.5f) continue;
        float half_w = PLUME_BASE_W + h * PLUME_SPREAD;

        for (int col = 0; col < cols; col++) {
            float dx = (float)(col - crater_x);
            if (fabsf(dx) > half_w) continue;

            /* Skip cells inside the mountain silhouette. */
            if (row > mtn->silhouette_y[col]) continue;

            /* Cone falloff (gaussian-style fade at edges). */
            float falloff = 1.0f - (dx * dx) / (half_w * half_w);
            if (falloff <= 0) continue;

            /* Sample fBm with wind drift + upward rise. */
            float nx = (dx + (float)col * 0.0f) * PLUME_NOISE_S
                     - time * PLUME_WIND * PLUME_NOISE_S;
            float ny = h * PLUME_NOISE_S - time * PLUME_RISE * PLUME_NOISE_S;
            float density = fbm2d(nx, ny, 7777u) * intensity * falloff;

            if (density < PLUME_THRESHOLD) continue;

            /* Map density to plume gradient slot. */
            float t_n = (density - PLUME_THRESHOLD) / (1.0f - PLUME_THRESHOLD);
            if (t_n > 1.0f) t_n = 1.0f;
            int slot = (int)(t_n * 3.999f);
            if (slot < 0) slot = 0;
            if (slot > 3) slot = 3;

            char glyph = PLUME_GLYPHS[slot];
            int  pair  = PAIR_PLUME_BASE + slot;
            attr_t attr = inverted ? A_NORMAL
                        : (slot <= 1) ? A_DIM : A_NORMAL;
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(row, col, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* ===================================================================== */
/* §9  scene — compose layers, per-frame update                           */
/* ===================================================================== */

typedef struct {
    bool   paused;
    int    current_pattern;
    int    current_theme;
    int    cols, rows;
    uint32_t seed;

    float  time;             /* accumulated seconds                     */
    float  intensity;        /* user-controlled multiplier on rates     */
    float  bomb_accum, ember_accum, ash_accum, spark_accum;

    /* VULCANIAN burst timer (only fires for that pattern). */
    float  next_burst_at;

    /* AMBIENT burst timer (fires for ALL patterns at random intervals).
     * Different from the VULCANIAN one — smaller, more frequent. */
    float  next_ambient_at;

    /* For HUD: time since last ambient burst (visual feedback so the
     * user can confirm bursts ARE happening even mid-quiet patterns). */
    float  last_ambient_age;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->current_pattern = PAT_STROMBOLIAN;
    s->current_theme   = 0;
    s->cols            = cols;
    s->rows            = rows;
    s->seed            = (uint32_t)clock_ns();
    s->time            = 0.0f;
    s->intensity       = 1.0f;
    s->next_burst_at   = lcg_range(&g_rng, VULCAN_BURST_INTERVAL_MIN,
                                            VULCAN_BURST_INTERVAL_MAX);
    s->next_ambient_at = lcg_range(&g_rng, AMBIENT_BURST_INTERVAL_MIN,
                                            AMBIENT_BURST_INTERVAL_MAX);
    s->last_ambient_age = 0.0f;
    g_rng = s->seed ^ 0xBEEFu;

    mountain_build(&g_mtn, cols, rows, s->seed);
    particles_clear();
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
    mountain_build(&g_mtn, cols, rows, s->seed);
    particles_clear();
}

static void scene_reset(Scene *s)
{
    s->seed = (uint32_t)clock_ns() ^ 0xA5A5A5A5u;
    g_rng   = s->seed ^ 0xBEEFu;
    s->time = 0.0f;
    s->next_burst_at = lcg_range(&g_rng, VULCAN_BURST_INTERVAL_MIN,
                                          VULCAN_BURST_INTERVAL_MAX);
    s->next_ambient_at = lcg_range(&g_rng, AMBIENT_BURST_INTERVAL_MIN,
                                            AMBIENT_BURST_INTERVAL_MAX);
    s->last_ambient_age = 0.0f;
    mountain_build(&g_mtn, s->cols, s->rows, s->seed);
    particles_clear();
}

/*
 * scene_tick — advance time, spawn new particles per pattern rates,
 * update all active particles.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->time += dt;

    const PatternParams *pp = &patterns[s->current_pattern];
    float intensity = s->intensity;

    /* Spawn accumulators — Poisson-style. */
    s->bomb_accum  += dt * pp->bomb_per_sec  * intensity;
    s->ember_accum += dt * pp->ember_per_sec * intensity;
    s->ash_accum   += dt * pp->ash_per_sec   * intensity;
    s->spark_accum += dt * pp->spark_per_sec * intensity;

    while (s->bomb_accum >= 1.0f) {
        particle_spawn_bomb(g_mtn.crater_x, g_mtn.crater_y, intensity);
        s->bomb_accum -= 1.0f;
    }
    while (s->ember_accum >= 1.0f) {
        particle_spawn_ember(g_mtn.crater_x, g_mtn.crater_y);
        s->ember_accum -= 1.0f;
    }
    while (s->ash_accum >= 1.0f) {
        particle_spawn_ash(g_mtn.crater_x, g_mtn.crater_y);
        s->ash_accum -= 1.0f;
    }
    while (s->spark_accum >= 1.0f) {
        particle_spawn_spark(g_mtn.crater_x, g_mtn.crater_y);
        s->spark_accum -= 1.0f;
    }

    /* VULCANIAN-pattern-specific periodic burst (massive 55-bomb dump). */
    if (pp->vulcanian_bursts && s->time >= s->next_burst_at) {
        for (int i = 0; i < VULCAN_BURST_BOMBS; i++)
            particle_spawn_bomb(g_mtn.crater_x, g_mtn.crater_y,
                                 intensity * 1.3f);
        s->next_burst_at = s->time
                         + lcg_range(&g_rng, VULCAN_BURST_INTERVAL_MIN,
                                              VULCAN_BURST_INTERVAL_MAX);
    }

    /*
     * AMBIENT BURST — random dramatic eruption regardless of pattern.
     * Fires every 8-22 sec (unpredictable on purpose); ejects a mixed
     * cocktail of bombs + sparks + embers so it reads as a "sudden
     * extra eruption surge" no matter which pattern is active.  Even
     * during the calm phases of STROMBOLIAN or PLINIAN you get
     * occasional drama.
     */
    if (s->time >= s->next_ambient_at) {
        for (int i = 0; i < AMBIENT_BURST_BOMBS; i++)
            particle_spawn_bomb(g_mtn.crater_x, g_mtn.crater_y,
                                 intensity * 1.1f);
        for (int i = 0; i < AMBIENT_BURST_SPARKS; i++)
            particle_spawn_spark(g_mtn.crater_x, g_mtn.crater_y);
        for (int i = 0; i < AMBIENT_BURST_EMBERS; i++)
            particle_spawn_ember(g_mtn.crater_x, g_mtn.crater_y);
        s->next_ambient_at = s->time
                           + lcg_range(&g_rng, AMBIENT_BURST_INTERVAL_MIN,
                                                AMBIENT_BURST_INTERVAL_MAX);
        s->last_ambient_age = 0.0f;
    } else {
        s->last_ambient_age += dt;
    }

    particles_tick(dt, s->rows, s->cols);
}

/*
 * scene_draw_sky — vertical gradient over rows above the silhouette.
 * Each cell's slot is determined by row position: top → slot 0 (deepest
 * sky), horizon → slot 3 (palest).
 */
static void scene_draw_sky(const Scene *s, bool inverted)
{
    int rows_eff = s->rows - 1;
    /* Use the maximum silhouette y as the "horizon" for gradient
     * normalisation; below that we're inside the mountain. */
    int max_sil = 0;
    for (int x = 0; x < s->cols; x++)
        if (g_mtn.silhouette_y[x] > max_sil) max_sil = g_mtn.silhouette_y[x];
    if (max_sil < 1) max_sil = 1;

    for (int row = 0; row < rows_eff; row++) {
        if (row >= max_sil) break;
        int slot = (row * 4) / max_sil;
        if (slot < 0) slot = 0;
        if (slot > 3) slot = 3;

        char glyph = inverted ? ' ' : ' ';   /* sky cells get bg colour only */
        attr_t attr = (inverted) ? A_NORMAL
                    : (slot == 0) ? A_DIM : A_NORMAL;
        int pair = PAIR_SKY_BASE + slot;
        attron(COLOR_PAIR(pair) | attr);
        for (int col = 0; col < s->cols; col++) {
            if (row > g_mtn.silhouette_y[col]) continue;     /* mountain   */
            mvaddch(row, col, (chtype)(unsigned char)glyph);
        }
        attroff(COLOR_PAIR(pair) | attr);
    }
}

/*
 * scene_draw_mountain — paint solid silhouette from `silhouette_y`
 * downward.  Top edge gets a slightly different glyph (`@`) for ridge
 * emphasis.
 */
static void scene_draw_mountain(const Scene *s, bool inverted)
{
    int rows_eff = s->rows - 1;
    attr_t attr = inverted ? A_NORMAL : A_BOLD;
    attron(COLOR_PAIR(PAIR_MOUNTAIN) | attr);
    for (int col = 0; col < s->cols; col++) {
        int top = g_mtn.silhouette_y[col];
        if (top < 0)            top = 0;
        if (top >= rows_eff)    continue;
        for (int row = top; row < rows_eff; row++) {
            char ch = (row == top) ? MTN_RIDGE_GLYPH : MTN_GLYPH;
            mvaddch(row, col, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_MOUNTAIN) | attr);
}

/*
 * scene_draw_lava_flows — bright cells along the pre-traced flow paths.
 * Brightness decays with distance from the crater (slot drops from
 * hottest at idx=0 down to dim at far end).
 */
static void scene_draw_lava_flows(const Scene *s, bool inverted)
{
    const PatternParams *pp = &patterns[s->current_pattern];
    int rows_eff = s->rows - 1;
    if (pp->lava_flow_amount < 0.05f) return;

    for (int side = 0; side < 2; side++) {
        const int *xs = (side == 0) ? g_mtn.flow_l_x : g_mtn.flow_r_x;
        const int *ys = (side == 0) ? g_mtn.flow_l_y : g_mtn.flow_r_y;
        int n          = (side == 0) ? g_mtn.flow_l_n : g_mtn.flow_r_n;

        for (int i = 0; i < n; i++) {
            float frac   = (float)i / (float)(n > 1 ? n - 1 : 1);
            float bright = (1.0f - frac) * pp->lava_flow_amount;
            int   slot   = 5 - (int)(bright * 6.0f);
            if (slot < 0) slot = 0;
            if (slot > 5) slot = 5;

            int  yy = ys[i];
            int  xx = xs[i];
            if (yy < 0 || yy >= rows_eff) continue;
            if (xx < 0 || xx >= s->cols)  continue;

            char glyph = LAVA_GLYPHS[slot];
            attr_t attr = (slot >= 4 && !inverted) ? A_BOLD : A_NORMAL;
            attron(COLOR_PAIR(PAIR_LAVA_BASE + slot) | attr);
            mvaddch(yy, xx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(PAIR_LAVA_BASE + slot) | attr);
        }
    }
}

/*
 * scene_draw_crater_glow — a few cells inside the crater bowl that
 * pulse with hot-lava colours, anchoring the eruption point visually.
 */
static void scene_draw_crater_glow(const Scene *s, bool inverted)
{
    int rows_eff = s->rows - 1;
    int cx = g_mtn.crater_x;
    int cy = g_mtn.crater_y;

    for (int dx = -(int)CRATER_RADIUS + 1;
             dx <= (int)CRATER_RADIUS - 1; dx++) {
        int x = cx + dx;
        if (x < 0 || x >= s->cols) continue;
        int y = cy - 1;            /* one row above the bowl bottom */
        if (y < 0 || y >= rows_eff) continue;

        float pulse = 0.5f + 0.5f * sinf(s->time * 3.0f + (float)dx * 0.5f);
        int slot = 4 + (int)(pulse * 1.5f);
        if (slot < 4) slot = 4;
        if (slot > 5) slot = 5;
        char glyph = LAVA_GLYPHS[slot];
        attr_t attr = inverted ? A_NORMAL : A_BOLD;
        attron(COLOR_PAIR(PAIR_LAVA_BASE + slot) | attr);
        mvaddch(y, x, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(PAIR_LAVA_BASE + slot) | attr);
    }
}

static int scene_active_count(void)
{
    int n = 0;
    for (int i = 0; i < PARTICLES_MAX; i++)
        if (g_pool[i].active) n++;
    return n;
}

static void scene_render(const Scene *s)
{
    bool inverted = themes[s->current_theme].inverted;
    int rows_eff = s->rows - 1;

    /* Inverted theme: pre-fill white "paper" before drawing. */
    if (inverted) {
        attron(COLOR_PAIR(PAIR_PAPER));
        for (int row = 0; row < rows_eff; row++)
            for (int col = 0; col < s->cols; col++)
                mvaddch(row, col, ' ');
        attroff(COLOR_PAIR(PAIR_PAPER));
    }

    scene_draw_sky(s, inverted);
    scene_draw_mountain(s, inverted);
    scene_draw_lava_flows(s, inverted);
    scene_draw_crater_glow(s, inverted);

    plume_render(&g_mtn, s->rows, s->cols, s->time,
                  patterns[s->current_pattern].plume_intensity, inverted);

    particles_render(s->rows, s->cols, inverted);
}

/* ===================================================================== */
/* §10 screen + app                                                       */
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

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_render(s);

    char buf[260];
    snprintf(buf, sizeof buf,
             " VOLCANO   %s   pattern:%s   theme:%s   particles:%4d   "
             "intensity:%4.2f   "
             "%5.1f fps  %3d Hz   "
             "n/N:pat  t/T:theme  +/-:int  r:reseed  spc:pause  q:quit ",
             s->paused ? "PAUSED " : "ERUPT  ",
             patterns[s->current_pattern].name,
             themes[s->current_theme].name,
             scene_active_count(),
             (double)s->intensity,
             fps, sim_fps);

    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

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
    case 'r': case 'R': scene_reset(s);                               break;

    case 'n':
        s->current_pattern = (s->current_pattern + 1) % N_PATTERNS;
        s->next_burst_at = s->time
                         + lcg_range(&g_rng, VULCAN_BURST_INTERVAL_MIN,
                                              VULCAN_BURST_INTERVAL_MAX);
        break;
    case 'N':
        s->current_pattern = (s->current_pattern + N_PATTERNS - 1) % N_PATTERNS;
        s->next_burst_at = s->time
                         + lcg_range(&g_rng, VULCAN_BURST_INTERVAL_MIN,
                                              VULCAN_BURST_INTERVAL_MAX);
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case '=': case '+':
        s->intensity *= 1.15f;
        if (s->intensity > 4.0f) s->intensity = 4.0f;
        break;
    case '-':
        s->intensity *= 0.85f;
        if (s->intensity < 0.20f) s->intensity = 0.20f;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
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
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        float dt_sec = (float)dt / (float)NS_PER_SEC;
        scene_tick(&app->scene, dt_sec);

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t target_ns = TICK_NS(app->sim_fps);
        int64_t elapsed   = clock_ns() - frame_time + dt;
        clock_sleep_ns(target_ns - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
