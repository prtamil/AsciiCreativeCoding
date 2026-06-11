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
 *         MONO      monochrome white / light-grey (silhouette study)
 *
 * Section map (re-cut into concern-separated layers — see ARCHITECTURE):
 *   §1 CONFIG       — constants, enums, theme + pattern tables, glyph ramps
 *   §2 LOGIC        — pure math, RNG, fBm noise (no mutation, no I/O)
 *   §3 PERFORMANCE  — monotonic clock + sleep (frame cap lives in §8 main)
 *   §4 RENDER-SETUP — colour-pair / theme palette configuration
 *   §5 SIMULATION   — mountain build + particle pool/physics + scene_tick
 *   §6 RENDER       — state→screen draws (sky/mtn/lava/plume/particles)
 *   §7 EVENTS       — init / resize / reset (mutate state OUTSIDE the tick)
 *   §8 SCREEN + APP — ncurses I/O, HUD, input, main loop + frame cap
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
 * Data-structure : One `Scene` aggregate owns everything: a `Mountain`
 *                  (silhouette_y[cols] heightfield + crater + lava-flow
 *                  paths) and `ejecta[1024]`, a pool of `Particle`s (type
 *                  tag, position, velocity, age, life, temperature).  Held
 *                  in BSS via the global App, no malloc.  The ejecta pool
 *                  dominates memory: 1024 × ~80 bytes ≈ 80 KB.
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
 *   Particle systems (§5 bombs / embers / ash / sparks)
 *     [1] Reeves, "Particle Systems: A Technique for Modelling a Class of Fuzzy
 *         Objects," ACM TOG 2(2) (1983) — the pool + active-flag pattern used
 *         here, and the emit / age / cool / recycle lifecycle.
 *     [2] Witkin & Heckbert, "Using Particles to Sample and Control Implicit
 *         Surfaces," SIGGRAPH (1994) — the layered-particle-system approach.
 *   Procedural noise — plume + mountain surface (§1/§4 fBm)
 *     [3] Mandelbrot, "The Fractal Geometry of Nature" (1982), §28 — the
 *         self-similar (fractal) basis of the multi-octave noise.
 *     [4] Perlin, "An Image Synthesizer," SIGGRAPH (1985) — gradient noise; the
 *         octave sums (fBm) that texture the plume and the silhouette.
 *     [5] Ebert, Musgrave, Peachey, Perlin & Worley, "Texturing & Modeling: A
 *         Procedural Approach" (2003) — fBm recipes for clouds/plumes and for
 *         terrain heightfields (the mountain profile).
 *   Volcanology (§1 eruption patterns)
 *     [6] Pyle, "Volcanoes: Encyclopedia of Earth's Living Systems" (2015) —
 *         eruption-style taxonomy (Strombolian / Vulcanian / Plinian /
 *         Hawaiian) and ejecta classification driving the spawn patterns.
 *   ASCII / terminal rendering (§3 colour, §6 render)
 *     [7] Bourke, "Character representation of grey scale images" (1997) — the
 *         heat / density → glyph ramp (' .,:;-+*#@').
 *     [8] Padala, "NCURSES Programming HOWTO" (TLDP) — colour pairs and the
 *         erase → draw → refresh frame model.
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
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layer separation) ─────────────────────────────────── *
 *
 * This file was re-cut into concern-separated layers.  Each function's
 * read/mutate role is FIXED by the layer it lives in:
 *
 *   LAYER          SECTION    MUTATES                        I/O
 *   ──────────────────────────────────────────────────────────────────────
 *   LOGIC          §2         nothing global (lcg_* advance   none
 *                             the *uint32_t handed to them)
 *   PERFORMANCE    §3, §8     nothing (clock_sleep_ns sleeps; reads clock,
 *                             main owns dt-cap/fps/frame-cap)  sleeps
 *   RENDER-SETUP   §4         ncurses colour pairs            init_pair
 *   SIMULATION     §5         scene.ejecta, scene.mountain,    none
 *                             Scene time / accum / burst timers
 *   RENDER         §6         nothing — reads scene.mountain,  ncurses out
 *                             scene.ejecta via narrow pointers
 *   EVENTS         §7, §8     scene.ejecta, scene.mountain,    ncurses
 *                             Scene, Screen, App — OUTSIDE tick init/resize
 *
 * LOGIC (§2) is provably uncorruptable from RENDER/EFFECTS: it touches no
 * global state and does no I/O, so reordering or deleting any draw cannot
 * change a noise / clamp / count result.  RENDER (§6) is read-only w.r.t.
 * simulation state — it can be re-run or skipped without altering physics.
 *
 * PER-TICK COMBINE — scene_tick() (§5) is the ONE place that advances
 * simulation, in this fixed named order:
 *     1. GUARD      if paused → return (nothing advances)
 *     2. TIME       s->time += dt
 *     3. SPAWN      accumulators += rate·dt·intensity; drain → spawn
 *                   bomb / ember / ash / spark
 *     4. BURST/VULC VULCANIAN burst timer → 55-bomb dump (pattern-gated)
 *     5. BURST/AMB  AMBIENT burst timer  → bomb+spark+ember dump (all pats)
 *     6. PHYSICS    particles_tick(dt) → integrate + cool + die + trail
 * Nothing outside scene_tick advances simulation state.
 *
 * USER EVENTS are NOT ticks.  scene_init / scene_resize / scene_reset (§7)
 * and app_do_resize / app_handle_key (§8) may mutate scene.ejecta,
 * scene.mountain, Scene, Screen and App — rebuild the world, toggle pause,
 * switch pattern/theme,
 * change intensity/Hz — but they run from the input / resize path, never
 * from scene_tick.
 *
 * EFFECTS — no dedicated layer.  The only cosmetic-only STORED state is the
 * bomb trail (tx/ty/trail_n), and it is shifted inside the bomb branch of
 * particles_tick (§5) as part of that particle's own physics step.  Every
 * other flourish — crater-glow pulse, sky/plume dimming, ember/bomb
 * brightness — is DERIVED at render time from `time` / `temp`, not stored,
 * so there is nothing to separate out.
 *
 * DELAYS — no dedicated layer.  `paused` is a one-line gate at the top of
 * scene_tick (§5); the eruption-burst timers (next_burst_at /
 * next_ambient_at) are simulation timers living inside the tick.  The only
 * real wall-clock hold is clock_sleep_ns (§3), driven by the frame cap in
 * main (§8) — it belongs to PERFORMANCE.
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
/* §1  CONFIG — constants, enums, theme + pattern tables, glyph ramps     */
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
#define CRATER_X_JITTER  0.30f     /* vent column wander, fraction of cols */
#define MTN_FLOW_MAX_LEN 24        /* longest lava-flow streamline (cells) */

/* Plume. */
#define PLUME_BASE_W     5.0f      /* base width at crater rim (cells)   */
#define PLUME_SPREAD     0.40f     /* width grows with altitude          */
#define PLUME_NOISE_S    0.18f     /* fBm domain scale                   */
#define PLUME_WIND       2.5f      /* +x drift speed (cells/sec)         */
#define PLUME_RISE       1.8f      /* column upward drift in noise space */
#define PLUME_THRESHOLD  0.45f     /* fbm above this → visible plume     */
#define PLUME_NOISE_SEED 7777u     /* fixed seed for the plume fBm field */

/* Particle physics. */
#define GRAVITY          24.0f     /* +y, cells/sec²                     */
#define BOMB_DRAG        0.04f     /* per second                         */
#define EMBER_BUOYANCY   12.0f     /* upward acceleration                */
#define ASH_BUOYANCY      4.0f
#define COOL_BOMB         0.18f    /* temp decay per sec                 */
#define COOL_EMBER        0.30f
#define COOL_ASH          0.05f    /* ash barely cools — long grey drift */
#define COOL_SPARK        2.50f    /* sparks fade fast                   */

/* Per-type force coupling (multipliers on shared forces). */
#define EMBER_TURBULENCE      2.0f /* ember vx random kick (±, cells/s²) */
#define EMBER_WIND_FACTOR     0.4f /* fraction of PLUME_WIND felt by embers */
#define ASH_TURBULENCE        1.0f /* ash vx random kick (±, cells/s²)   */
#define ASH_WIND_FACTOR       0.6f /* fraction of PLUME_WIND felt by ash */
#define SPARK_GRAVITY_FACTOR  0.5f /* sparks feel reduced gravity        */

/* Off-screen cull margins — how far past the edge before a particle dies. */
#define PARTICLE_CULL_MARGIN_Y 2.0f /* rows above top / below bottom     */
#define PARTICLE_CULL_MARGIN_X 3.0f /* cols past left / right            */

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

/* Spawn scatter (± cells around the vent) and initial heat (0..1). */
#define BOMB_SPAWN_DX     1.5f
#define EMBER_SPAWN_DX    2.0f
#define EMBER_SPAWN_VX    1.5f      /* ember initial horizontal velocity (±) */
#define EMBER_SPAWN_TEMP  0.9f
#define ASH_SPAWN_DX      3.0f
#define ASH_SPAWN_LIFT    4.0f      /* ash starts up to this many rows above vent */
#define ASH_SPAWN_VX      2.0f      /* ash initial horizontal velocity (±)   */
#define ASH_SPAWN_TEMP    0.4f
#define SPARK_SPAWN_DX    2.0f
#define SPARK_SPAWN_ANGLE 1.5f      /* ± radians from vertical for spark jets */
#define SPARK_VY_FACTOR   0.8f      /* spark upward-velocity scale           */

/* Burst intensity boosts (multiply the base intensity for that volley). */
#define VULCAN_BURST_BOOST   1.3f
#define AMBIENT_BURST_BOOST  1.1f

/*
 * Pattern — the four eruption styles the demo cycles through (n/N), used as
 * an index into patterns[].  The names are the standard volcanological
 * eruption-style taxonomy (ref [6]); each implies a characteristic ejecta
 * mix, which the matching EruptionPattern row encodes as spawn rates:
 *   PAT_STROMBOLIAN — frequent moderate bursts of incandescent bombs; the
 *                     "rhythmic" baseline, ember-heavy.
 *   PAT_VULCANIAN   — short violent explosions: a low steady rate punctuated
 *                     by periodic 55-bomb dumps (the vulcanian_bursts flag).
 *   PAT_PLINIAN     — sustained towering ash column: heavy ash + ember, the
 *                     tallest/densest plume (plume_intensity 1.8).
 *   PAT_HAWAIIAN    — fluid lava fountaining: high bomb/spark rate, bright
 *                     slope flows, little ash.
 * N_PATTERNS is the count sentinel — keep it last so the wrap
 * `(cur + 1) % N_PATTERNS` and the patterns[] table size stay in sync.
 */
typedef enum {
    PAT_STROMBOLIAN = 0,
    PAT_VULCANIAN   = 1,
    PAT_PLINIAN     = 2,
    PAT_HAWAIIAN    = 3,
    N_PATTERNS      = 4,
} Pattern;

/*
 * EruptionPattern — one immutable row of the eruption-style table
 * (patterns[]), one per Pattern.  This is the data-driven heart of the
 * demo: switching style (n/N) only swaps WHICH row scene_tick reads —
 * there is no per-style branching in the physics, so a new style is just a
 * new row.  Each style (ref [6]) is reduced to four Poisson spawn rates
 * plus two cosmetic weights and one burst flag; the numbers in patterns[]
 * are tuned so the four read as visually distinct (see Pattern).
 */
typedef struct {
    const char *name;          /* HUD label, space-padded to a fixed width        */
    float bomb_per_sec;        /* mean ballistic-bomb spawns/sec (Poisson rate)   */
    float ember_per_sec;       /* mean ember spawns/sec (rising glow specks)      */
    float ash_per_sec;         /* mean ash spawns/sec (slow grey drift)           */
    float spark_per_sec;       /* mean spark spawns/sec (fast crater-rim jets)    */
    float plume_intensity;     /* scales fBm plume density: 0.6 wispy → 1.8 towering */
    bool  vulcanian_bursts;    /* true → also dump VULCAN_BURST_BOMBS every few sec  */
    float lava_flow_amount;    /* 0..1 render brightness of the pre-traced slope flows */
} EruptionPattern;

static const EruptionPattern patterns[N_PATTERNS] = {
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

/* Render tuning. */
#define GLOW_PULSE_RATE   3.0f     /* crater-glow pulse rate (radians/sec) */
#define FLOW_VISIBLE_MIN  0.05f    /* skip drawing flows dimmer than this  */

/* User intensity knob (+/-): geometric step, clamped to [MIN, MAX]. */
#define INTENSITY_STEP_UP    1.15f
#define INTENSITY_STEP_DOWN  0.85f
#define INTENSITY_MAX        4.0f
#define INTENSITY_MIN        0.20f

/* Frame loop: cap simulated dt so a long stall can't trigger a spiral
 * of death (one giant catch-up tick).  Milliseconds. */
#define DT_CAP_MS         100

/*
 * Theme — a complete colour palette for the scene, swapped live with t/T by
 * re-running init_pair (ref [8]); no geometry rebuilds.  Each field is an
 * ORDERED gradient of xterm-256 colour indices that the renderer addresses
 * by a normalised quantity — sky by screen row, lava by particle temp,
 * plume by fBm density — so an array position MEANS "how high / how hot /
 * how dense", it is not an arbitrary slot.  Every index sits in the bright
 * half of the cube so even the darkest tier stays legible against the
 * default background under A_DIM (project palette-brightness rule).
 */
typedef struct {
    const char *name;          /* HUD label, space-padded                       */
    short       sky[4];        /* vertical gradient: [0]=zenith → [3]=horizon    */
    short       mountain[3];   /* edifice shading: [0]=shadow [1]=mid [2]=lit    */
    short       lava[6];       /* heat ramp: [0]=cooled/dim → [5]=white-hot      */
    short       plume[4];      /* smoke ramp: [0]=dense core → [3]=wispy edge    */
    bool        inverted;      /* true → "paper" study: dark ink on a white fill */
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

    /* MONO: monochrome — white / light-grey throughout; depth comes from the
     * A_DIM / A_BOLD brightness attrs, not hue (silhouette study).          */
    { "MONO     ",
      { 244, 248, 250, 252 },
      { 248, 251, 254 },
      { 245, 248, 250, 252, 254, 231 },
      { 247, 250, 252, 254 }, false },
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
/* §2  LOGIC — pure math, RNG, fBm noise                                  */
/*                                                                        */
/* Pure: these functions mutate no global program state and do no I/O.    */
/* The lcg_* helpers advance the uint32_t handed to them by pointer —     */
/* that is the caller's RNG cursor, not hidden state — so their result    */
/* is fully determined by their inputs.  Nothing in RENDER or EFFECTS     */
/* can corrupt a noise / clamp / count value here.                        */
/* ===================================================================== */

/* V2 — a 2-D vector (x, y).  Vestigial: declared for pixel-space helpers
 * but currently UNUSED — particles store x/y/vx/vy as loose floats instead.
 * Kept only because removing it is out of scope for a docs pass. */
typedef struct { float x, y; } V2;

/* Clamp v into [lo, hi]: below lo → lo, above hi → hi, else unchanged. */
static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Quantise a normalised value t01∈[0,1] to a ramp index in [0, n_slots-1].
 * The -0.001 nudge keeps t01==1.0 from overflowing to n_slots; used to map
 * heat / density / brightness onto a glyph-or-colour ramp. */
static inline int unit_to_slot(float t01, int n_slots)
{
    int s = (int)(t01 * ((float)n_slots - 0.001f));
    if (s < 0)         s = 0;
    if (s >= n_slots)  s = n_slots - 1;
    return s;
}

/*
 * lcg_next — one step of a linear congruential generator: st = a·st + c
 * (mod 2^32, the modulo happening for free as uint32 overflow).  The
 * constants are the classic Numerical Recipes pair — a = 1664525 (the
 * multiplier) and c = 1013904223 (the increment) — chosen together to give
 * a full period of 2^32 before the sequence repeats.
 */
static inline uint32_t lcg_next(uint32_t *st)
{
    *st = *st * 1664525u + 1013904223u;
    return *st;
}

/*
 * lcg_unit — next random float in [0, 1).  An LCG's LOW bits are poor
 * quality (the lowest bit just toggles), so we drop the bottom 8 (>> 8) and
 * keep the high 24 bits, then divide by 2^24 (1u << 24 = 16777216) to map
 * that 24-bit integer in [0, 2^24) onto [0, 1).
 */
static inline float lcg_unit(uint32_t *st)
{
    return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/* lcg_range — uniform random float in [lo, hi): just rescale a [0,1) unit. */
static inline float lcg_range(uint32_t *st, float lo, float hi)
{
    return lo + (hi - lo) * lcg_unit(st);
}

static uint32_t g_rng = 0xCAFEBABEu;

/*
 * hash2d — deterministic 32-bit hash of an integer lattice point (x,z) plus
 * a seed.  Each input is multiplied by a distinct large odd prime so they
 * scatter to different parts of the word before being summed — 374761393
 * and 668265263 are well-known spatial-hash primes, and 2147483647 (the
 * Mersenne prime 2^31-1) decorrelates the seed.  The last two lines are an
 * xorshift-multiply "finalizer" (xor a high-bit-shifted copy, multiply by
 * the mixing prime 1274126177, xor again) that avalanches the bits so two
 * adjacent cells give completely unrelated outputs.  Shifts 13 and 16 are
 * the standard mixing amounts for a 32-bit word.
 */
static inline uint32_t hash2d(int x, int z, uint32_t seed)
{
    uint32_t h = (uint32_t)x * 374761393u
               + (uint32_t)z * 668265263u
               + seed        * 2147483647u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}

/* hash_unit — hash2d normalised to [0,1): drop the weak low 8 bits, keep
 * the high 24, divide by 2^24 (same recipe as lcg_unit). */
static inline float hash_unit(int x, int z, uint32_t seed)
{
    return (float)(hash2d(x, z, seed) >> 8) / (float)(1u << 24);
}

/*
 * smoothstep01 — the Hermite ease curve S(t) = 3t² - 2t³ = t²(3 - 2t),
 * mapping [0,1]→[0,1].  The constants 3 and 2 are not free: they are the
 * unique cubic with S(0)=0, S(1)=1 AND zero slope at both ends (S'(0)=
 * S'(1)=0), which is what removes the visible creases at lattice
 * boundaries when it eases the noise interpolation weights below.
 */
static inline float smoothstep01(float t) { return t * t * (3.0f - 2.0f * t); }

/*
 * vnoise2d — value noise in [0,1].  Split the point into its integer cell
 * (xi,zi) and fractional offset (fx,fz)∈[0,1).  Hash the four cell corners
 * (xi+0/1, zi+0/1) to random corner values, then bilinearly blend them —
 * but using smoothstep01-eased weights (sx,sz) instead of raw fx,fz, so the
 * field is smooth across cell boundaries.  a/b interpolate the bottom/top
 * edges in x; the final line interpolates those two in z.
 */
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

/*
 * fbm2d — fractional Brownian motion: sum several octaves of value noise to
 * get detail at multiple scales (refs [3][4]).  Per octave: freq *= 2.0
 * (lacunarity — each octave is twice as fine) and amp *= 0.5 (gain /
 * persistence — and contributes half as much).  3 octaves is plenty for a
 * terminal-resolution field.  `norm` sums the amplitudes (1 + 0.5 + 0.25)
 * so the final h/norm stays in [0,1] regardless of octave count.  The
 * `seed + i*17` gives each octave its OWN hash field (17 is just an
 * arbitrary odd offset) so the layers don't line up and reinforce.
 */
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
/* §3  PERFORMANCE — monotonic clock + sleep                              */
/*                                                                        */
/* Timing primitives only.  clock_ns reads the OS clock; clock_sleep_ns   */
/* is the program's single wall-clock hold (the per-frame cap).  The      */
/* fixed-frame-cap arithmetic that USES these lives in main() (§8).       */
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
/* §4  RENDER-SETUP — colour-pair / theme palette configuration           */
/*                                                                        */
/* The only state these mutate is the ncurses colour table (init_pair).   */
/* Called from screen_init (startup) and the t/T key (EVENTS) — never     */
/* from the tick.                                                         */
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
/* §5  SIMULATION — state + advance                                       */
/*                                                                        */
/* This layer owns every piece of mutable simulation state, all now        */
/* members of Scene (no file-scope globals):                               */
/*   • scene.mountain (Mountain)      — built by mountain_build (events)   */
/*   • scene.ejecta   (Particle pool) — spawned / advanced / recycled here */
/*   • Scene time / accumulators / burst timers — advanced by scene_tick   */
/* scene_tick() is the ONE per-tick combiner (combine order documented     */
/* in the ARCHITECTURE block).  mountain_build runs only from EVENTS       */
/* (§7), not the tick.  The bomb-trail shift inside particles_tick is the  */
/* sole cosmetic-only stored state — see the EFFECTS note in ARCHITECTURE. */
/* ===================================================================== */

/* --- mountain: heightmap silhouette + crater + lava-flow streams ------ */

/*
 * Mountain — the static volcanic edifice, generated once per seed by
 * mountain_build and then only READ each frame.  Because the view is a
 * side-on silhouette, the terrain needs just a 1-D HEIGHT FIELD, not a 2-D
 * grid: silhouette_y[col] is the screen row where rock starts in that
 * column, so the whole shape is one row-per-column array.  The profile is a
 * parametric cone (peak + |dx|·slope) textured with 1-D fBm so the surface
 * reads as eroded rock rather than a perfect triangle (Perlin gradient
 * noise summed over octaves, refs [4][5]); a cosine bowl carves the crater.
 *
 * Lava flows are STREAMLINES traced once down each slope from the crater
 * rim and cached as parallel (x,y) arrays.  They are stored, not recomputed,
 * because the terrain is immutable — the renderer just re-colours the cached
 * cells with a distance-from-crater falloff each frame.
 */
typedef struct {
    int   silhouette_y[MTN_MAX_W]; /* HEIGHT FIELD: top rock-row per column (smaller y = taller) */
    int   crater_x;                /* column of the summit crater centre               */
    int   crater_y;                /* row of the crater floor — the particle spawn point */
    /* Pre-traced lava-flow streamlines: parallel arrays of (col,row) cells
     * walked outward from the crater; flow_*_n is how many cells were laid. */
    int   flow_l_x[MTN_MAX_W];     /* left-slope flow: column of cell i                */
    int   flow_l_y[MTN_MAX_W];     /* left-slope flow: row of cell i                   */
    int   flow_l_n;                /* number of cells in the left flow                 */
    int   flow_r_x[MTN_MAX_W];     /* right-slope flow: column of cell i               */
    int   flow_r_y[MTN_MAX_W];     /* right-slope flow: row of cell i                  */
    int   flow_r_n;                /* number of cells in the right flow                */
} Mountain;

/*
 * pick_crater_column — choose the vent column: screen-centred, jittered up
 * to ±CRATER_X_JITTER of the width, then clamped to the middle half so the
 * full cone always fits on screen.  Advances *seed (the build RNG cursor).
 */
static int pick_crater_column(int cols, uint32_t *seed)
{
    int crater_x = cols / 2
                 + (int)((lcg_unit(seed) - 0.5f) * (float)cols * CRATER_X_JITTER);
    if (crater_x < cols / 4)     crater_x = cols / 4;
    if (crater_x > 3 * cols / 4) crater_x = 3 * cols / 4;
    return crater_x;
}

/*
 * build_cone_silhouette — lay the base height field: a parametric cone
 * (peak + |dx|·slope) plus 1-D fBm surface noise, clamped to the screen
 * (refs [4][5]).  Writes silhouette_y[0..cols).
 */
static void build_cone_silhouette(Mountain *m, int cols, int rows,
                                  int crater_x, int peak_y, uint32_t seed)
{
    for (int x = 0; x < cols; x++) {
        float dx   = (float)abs(x - crater_x);
        float cone = (float)peak_y + dx * MTN_SLOPE;
        float n    = fbm1d((float)x * MTN_NOISE_SCALE, seed) - 0.5f;
        cone += n * 2.0f * MTN_NOISE_AMP;
        if (cone < 0)        cone = 0;
        if (cone > rows - 2) cone = rows - 2;
        m->silhouette_y[x] = (int)(cone + 0.5f);
    }
}

/*
 * carve_crater — deepen a cosine bowl of radius CRATER_RADIUS at the summit,
 * so the vent reads as a dip rather than a sharp peak.
 */
static void carve_crater(Mountain *m, int cols, int rows, int crater_x)
{
    for (int x = crater_x - (int)CRATER_RADIUS;
             x <= crater_x + (int)CRATER_RADIUS; x++) {
        if (x < 0 || x >= cols) continue;
        float dxn = ((float)(x - crater_x)) / CRATER_RADIUS;
        if (dxn < -1.0f || dxn > 1.0f) continue;
        float depth = CRATER_DEPTH * 0.5f * (1.0f + cosf((float)M_PI * dxn));
        m->silhouette_y[x] += (int)depth;
        if (m->silhouette_y[x] > rows - 2) m->silhouette_y[x] = rows - 2;
    }
}

/*
 * trace_lava_flow — walk a streamline outward from the crater rim in
 * direction `dir` (-1 left, +1 right), laying one cell per column just below
 * the surface, up to max_len cells.  Writes into xs/ys; returns the count.
 */
static int trace_lava_flow(const Mountain *m, int cols, int crater_x, int dir,
                           int max_len, int *xs, int *ys)
{
    int n = 0;
    for (int d = (int)CRATER_RADIUS + 1; d < max_len; d++) {
        int x = crater_x + dir * d;
        if (x < 0 || x >= cols) break;
        xs[n] = x;
        ys[n] = m->silhouette_y[x] + 1;
        n++;
    }
    return n;
}

/*
 * mountain_build — generate the static edifice in four steps: choose the
 * vent, lay the cone+fBm silhouette, carve the crater bowl, then pre-trace a
 * lava-flow streamline down each slope.  Reads top-to-bottom as that recipe.
 */
static void mountain_build(Mountain *m, int cols, int rows, uint32_t seed)
{
    if (cols > MTN_MAX_W) cols = MTN_MAX_W;

    int peak_y   = (int)((float)rows * MTN_PEAK_FRAC);
    int crater_x = pick_crater_column(cols, &seed);

    build_cone_silhouette(m, cols, rows, crater_x, peak_y, seed);
    carve_crater(m, cols, rows, crater_x);

    m->crater_x = crater_x;
    m->crater_y = m->silhouette_y[crater_x];

    /* Flows exhaust a quarter of the way to the edge, capped at MTN_FLOW_MAX_LEN. */
    int max_len = (cols / 4 < MTN_FLOW_MAX_LEN) ? cols / 4 : MTN_FLOW_MAX_LEN;
    m->flow_l_n = trace_lava_flow(m, cols, crater_x, -1, max_len,
                                  m->flow_l_x, m->flow_l_y);
    m->flow_r_n = trace_lava_flow(m, cols, crater_x, +1, max_len,
                                  m->flow_r_x, m->flow_r_y);
}

/* --- particles: single pool, four types ------------------------------- */

/*
 * ParticleType — which of the four ejecta kinds a Particle is.  All four
 * share ONE struct and ONE pool (the tagged / particle-system pattern, ref
 * [1]): particles_tick switches on this tag to pick the force model and
 * cooling rate, so new behaviour is a new switch case, not a new array.
 * Physically:
 *   PT_BOMB  — lava bomb: heavy ballistic arc under gravity, leaves a trail.
 *   PT_EMBER — buoyant glowing speck that rises through the plume and cools.
 *   PT_ASH   — fine near-neutral-buoyancy grey particle, long-lived drift.
 *   PT_SPARK — tiny, very hot, very short-lived crater-rim jet.
 */
typedef enum {
    PT_BOMB  = 0,
    PT_EMBER = 1,
    PT_ASH   = 2,
    PT_SPARK = 3,
} ParticleType;

/*
 * Particle — one slot in the fixed pool (ref [1], Reeves).  There is no
 * free list: `active` marks a live slot, particle_alloc linear-scans for a
 * dead one, and a particle frees itself (active=false) on death, so the
 * pool size (PARTICLES_MAX) is the hard cap on simultaneous ejecta.
 * Position/velocity are in CELL units (this sim is cell-space, not pixel).
 * Fields group into kinematics, lifecycle, look, and the bomb-only trail:
 */
typedef struct {
    bool         active;      /* slot live? false = free for reuse              */
    ParticleType type;        /* selects the physics branch in particles_tick   */
    /* kinematics — cells and cells/sec */
    float        x, y;        /* position; y grows DOWNWARD (screen convention) */
    float        vx, vy;      /* velocity; vy < 0 is upward                     */
    /* lifecycle — seconds */
    float        age, life;   /* dies when age >= life (also on cooling/off-screen) */
    /* look */
    float        temp;        /* heat 0=cold → 1=white-hot; indexes the lava heat ramp */
    /* bomb-only smoke trail — last BOMB_TRAIL_LEN positions, newest at [0] */
    float        tx[BOMB_TRAIL_LEN]; /* trailing x history (cosmetic only)      */
    float        ty[BOMB_TRAIL_LEN]; /* trailing y history                      */
    int          trail_n;     /* count of valid trail samples (0 until it moves)*/
} Particle;

static int particle_alloc(Particle *pool)
{
    for (int i = 0; i < PARTICLES_MAX; i++) {
        if (!pool[i].active) return i;
    }
    return -1;       /* pool full */
}

static void particles_clear(Particle *pool)
{
    memset(pool, 0, PARTICLES_MAX * sizeof *pool);
}

/*
 * particle_spawn_bomb — random ballistic launch from crater.  Velocity
 * is angled within ±BOMB_ANGLE around vertical-up; speed gives an
 * apex around 0.5-0.8 of screen height (set by speed range).
 */
static void particle_spawn_bomb(Particle *pool, int crater_x, int crater_y,
                                float intensity)
{
    int idx = particle_alloc(pool);
    if (idx < 0) return;
    Particle *p = &pool[idx];
    p->active = true;
    p->type   = PT_BOMB;
    p->x      = (float)crater_x + lcg_range(&g_rng, -BOMB_SPAWN_DX, BOMB_SPAWN_DX);
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

static void particle_spawn_ember(Particle *pool, int crater_x, int crater_y)
{
    int idx = particle_alloc(pool);
    if (idx < 0) return;
    Particle *p = &pool[idx];
    p->active = true;
    p->type   = PT_EMBER;
    p->x      = (float)crater_x + lcg_range(&g_rng, -EMBER_SPAWN_DX, EMBER_SPAWN_DX);
    p->y      = (float)crater_y;
    float speed = lcg_range(&g_rng, EMBER_SPEED_MIN, EMBER_SPEED_MAX);
    p->vx     = lcg_range(&g_rng, -EMBER_SPAWN_VX, EMBER_SPAWN_VX);
    p->vy     = -speed;
    p->age    = 0;
    p->life   = lcg_range(&g_rng, EMBER_LIFE_MIN, EMBER_LIFE_MAX);
    p->temp   = EMBER_SPAWN_TEMP;
    p->trail_n = 0;
}

static void particle_spawn_ash(Particle *pool, int crater_x, int crater_y)
{
    int idx = particle_alloc(pool);
    if (idx < 0) return;
    Particle *p = &pool[idx];
    p->active = true;
    p->type   = PT_ASH;
    p->x      = (float)crater_x + lcg_range(&g_rng, -ASH_SPAWN_DX, ASH_SPAWN_DX);
    p->y      = (float)crater_y - lcg_range(&g_rng, 0.0f, ASH_SPAWN_LIFT);
    float speed = lcg_range(&g_rng, ASH_SPEED_MIN, ASH_SPEED_MAX);
    p->vx     = lcg_range(&g_rng, -ASH_SPAWN_VX, ASH_SPAWN_VX);
    p->vy     = -speed;
    p->age    = 0;
    p->life   = lcg_range(&g_rng, ASH_LIFE_MIN, ASH_LIFE_MAX);
    p->temp   = ASH_SPAWN_TEMP;
    p->trail_n = 0;
}

static void particle_spawn_spark(Particle *pool, int crater_x, int crater_y)
{
    int idx = particle_alloc(pool);
    if (idx < 0) return;
    Particle *p = &pool[idx];
    p->active = true;
    p->type   = PT_SPARK;
    p->x      = (float)crater_x + lcg_range(&g_rng, -SPARK_SPAWN_DX, SPARK_SPAWN_DX);
    p->y      = (float)crater_y;
    float angle = lcg_range(&g_rng, -SPARK_SPAWN_ANGLE, SPARK_SPAWN_ANGLE);
    float speed = lcg_range(&g_rng, SPARK_SPEED_MIN, SPARK_SPEED_MAX);
    p->vx     = sinf(angle) * speed;
    p->vy     = -cosf(angle) * speed * SPARK_VY_FACTOR;
    p->age    = 0;
    p->life   = lcg_range(&g_rng, SPARK_LIFE_MIN, SPARK_LIFE_MAX);
    p->temp   = 1.0f;
    p->trail_n = 0;
}

/* Shift the bomb's position history one slot (newest at [0]) — the smoke
 * trail.  A small ring-buffer write; cosmetic-only state (see Particle). */
static void bomb_trail_push(Particle *p)
{
    for (int k = BOMB_TRAIL_LEN - 1; k > 0; k--) {
        p->tx[k] = p->tx[k - 1];
        p->ty[k] = p->ty[k - 1];
    }
    p->tx[0] = p->x;
    p->ty[0] = p->y;
    p->trail_n = BOMB_TRAIL_LEN;
}

/* PT_BOMB — ballistic arc: gravity down, light air drag, slow cooling;
 * records a trail.  `drag` is the per-tick velocity-retention factor. */
static void bomb_step(Particle *p, float dt, float drag)
{
    bomb_trail_push(p);
    p->vy += GRAVITY * dt;
    p->vx *= drag;
    p->vy *= drag;
    p->x  += p->vx * dt;
    p->y  += p->vy * dt;
    p->temp -= COOL_BOMB * dt;
}

/* PT_EMBER — buoyant rise + gentle wind + turbulence; cools fast. */
static void ember_step(Particle *p, float dt)
{
    p->vy -= EMBER_BUOYANCY * dt;
    p->vx += lcg_range(&g_rng, -EMBER_TURBULENCE, EMBER_TURBULENCE) * dt;
    p->vx += PLUME_WIND * EMBER_WIND_FACTOR * dt;
    p->x  += p->vx * dt;
    p->y  += p->vy * dt;
    p->temp -= COOL_EMBER * dt;
}

/* PT_ASH — weak buoyancy, stronger wind coupling, near-zero cooling. */
static void ash_step(Particle *p, float dt)
{
    p->vy -= ASH_BUOYANCY * dt;
    p->vx += lcg_range(&g_rng, -ASH_TURBULENCE, ASH_TURBULENCE) * dt;
    p->vx += PLUME_WIND * ASH_WIND_FACTOR * dt;
    p->x  += p->vx * dt;
    p->y  += p->vy * dt;
    p->temp -= COOL_ASH * dt;
}

/* PT_SPARK — fast ballistic with reduced gravity; very fast cooling. */
static void spark_step(Particle *p, float dt)
{
    p->vy += GRAVITY * SPARK_GRAVITY_FACTOR * dt;
    p->x  += p->vx * dt;
    p->y  += p->vy * dt;
    p->temp -= COOL_SPARK * dt;
}

/* A particle dies when it outlives its lifespan, cools to black, or drifts
 * past the screen edge (with a small margin so it doesn't pop at the border). */
static bool particle_is_dead(const Particle *p, int rows, int cols)
{
    if (p->age >= p->life)                            return true;
    if (p->temp <= 0)                                 return true;
    if (p->y < -PARTICLE_CULL_MARGIN_Y)               return true;
    if (p->y > (float)rows + PARTICLE_CULL_MARGIN_Y)  return true;
    if (p->x < -PARTICLE_CULL_MARGIN_X ||
        p->x > (float)cols + PARTICLE_CULL_MARGIN_X)  return true;
    return false;
}

/*
 * particles_tick — advance every live particle one step: dispatch to the
 * per-type integrator, age it, then recycle the slot if it died.
 */
static void particles_tick(Particle *pool, float dt, int rows, int cols)
{
    float bomb_drag = 1.0f - BOMB_DRAG * dt;
    if (bomb_drag < 0) bomb_drag = 0;

    for (int i = 0; i < PARTICLES_MAX; i++) {
        Particle *p = &pool[i];
        if (!p->active) continue;

        switch (p->type) {
        case PT_BOMB:  bomb_step (p, dt, bomb_drag); break;
        case PT_EMBER: ember_step(p, dt);            break;
        case PT_ASH:   ash_step  (p, dt);            break;
        case PT_SPARK: spark_step(p, dt);            break;
        }

        p->age += dt;
        if (particle_is_dead(p, rows, cols)) p->active = false;
    }
}

/* --- scene state + the per-tick combiner ------------------------------ */

/*
 * Scene — the whole erupting volcano, the one aggregate the tick
 * orchestrates.  Reads as a table of contents:
 *   WHAT is simulated — the mountain edifice + the live ejecta pool.
 *   HOW it is driven   — the user knobs (style, intensity, pause).
 *   WHEN we are        — the sim clock + the emission cadence (spawn
 *                        accumulators + burst timers) that share its units.
 *   render + viewport  — palette choice and terminal dims (a RENDER concern
 *                        kept OUT of the simulation-knob group on purpose).
 * Sub-functions take the narrowest member they need (const Mountain* /
 * Particle*), never the whole Scene — only init / reset / tick do.
 */
typedef struct {
    /* WHAT is simulated — the domain objects */
    Mountain mountain;                  /* static edifice: heightfield + crater + lava-flow paths */
    Particle ejecta[PARTICLES_MAX];     /* live pool: bombs / embers / ash / sparks               */

    /* HOW the eruption is driven — user knobs */
    int    current_pattern;             /* index into patterns[] (eruption style)  */
    float  intensity;                   /* multiplier on every spawn rate           */
    bool   paused;                      /* freeze spawning + physics                */

    /* WHEN — sim clock + emission cadence (seconds, except the accumulators) */
    float  time;                        /* accumulated sim seconds                  */
    float  bomb_accum, ember_accum, ash_accum, spark_accum; /* fractional spawns pending */
    float  next_burst_at;               /* sim time of next VULCANIAN bomb dump     */
    float  next_ambient_at;             /* sim time of next universal ambient burst */
    float  last_ambient_age;            /* seconds since last ambient (HUD feedback)*/

    /* world generation */
    uint32_t seed;                      /* reseeds mountain shape + spawn RNG       */

    /* render + viewport */
    int    cols, rows;                  /* sim's copy of terminal dims              */
    int    current_theme;               /* index into themes[] (colour palette)     */
} Scene;

/*
 * spawn_continuous — the steady eruption.  Advance each Poisson accumulator
 * by rate·dt·intensity, then spawn one particle per whole unit that crosses
 * 1.0 (ref [1]).  This is the only emission that runs every tick.
 */
static void spawn_continuous(Scene *s, const EruptionPattern *pp, float dt)
{
    Particle *pool     = s->ejecta;
    int       crater_x = s->mountain.crater_x;
    int       crater_y = s->mountain.crater_y;
    float     intensity = s->intensity;

    s->bomb_accum  += dt * pp->bomb_per_sec  * intensity;
    s->ember_accum += dt * pp->ember_per_sec * intensity;
    s->ash_accum   += dt * pp->ash_per_sec   * intensity;
    s->spark_accum += dt * pp->spark_per_sec * intensity;

    while (s->bomb_accum >= 1.0f) {
        particle_spawn_bomb(pool, crater_x, crater_y, intensity);
        s->bomb_accum -= 1.0f;
    }
    while (s->ember_accum >= 1.0f) {
        particle_spawn_ember(pool, crater_x, crater_y);
        s->ember_accum -= 1.0f;
    }
    while (s->ash_accum >= 1.0f) {
        particle_spawn_ash(pool, crater_x, crater_y);
        s->ash_accum -= 1.0f;
    }
    while (s->spark_accum >= 1.0f) {
        particle_spawn_spark(pool, crater_x, crater_y);
        s->spark_accum -= 1.0f;
    }
}

/*
 * maybe_vulcanian_burst — VULCANIAN styles only: when the burst timer
 * elapses, dump a violent VULCAN_BURST_BOMBS volley and schedule the next.
 */
static void maybe_vulcanian_burst(Scene *s, const EruptionPattern *pp)
{
    if (pp->vulcanian_bursts && s->time >= s->next_burst_at) {
        for (int i = 0; i < VULCAN_BURST_BOMBS; i++)
            particle_spawn_bomb(s->ejecta, s->mountain.crater_x,
                                s->mountain.crater_y,
                                s->intensity * VULCAN_BURST_BOOST);
        s->next_burst_at = s->time
                         + lcg_range(&g_rng, VULCAN_BURST_INTERVAL_MIN,
                                              VULCAN_BURST_INTERVAL_MAX);
    }
}

/*
 * maybe_ambient_burst — fires for EVERY pattern at a random 8-22 s interval:
 * a mixed bomb+spark+ember surge so even quiet styles get occasional drama.
 * Between surges it just ages the HUD "time since last burst" counter.
 */
static void maybe_ambient_burst(Scene *s, float dt)
{
    if (s->time >= s->next_ambient_at) {
        Particle *pool = s->ejecta;
        int cx = s->mountain.crater_x, cy = s->mountain.crater_y;
        for (int i = 0; i < AMBIENT_BURST_BOMBS; i++)
            particle_spawn_bomb(pool, cx, cy, s->intensity * AMBIENT_BURST_BOOST);
        for (int i = 0; i < AMBIENT_BURST_SPARKS; i++)
            particle_spawn_spark(pool, cx, cy);
        for (int i = 0; i < AMBIENT_BURST_EMBERS; i++)
            particle_spawn_ember(pool, cx, cy);
        s->next_ambient_at = s->time
                           + lcg_range(&g_rng, AMBIENT_BURST_INTERVAL_MIN,
                                                AMBIENT_BURST_INTERVAL_MAX);
        s->last_ambient_age = 0.0f;
    } else {
        s->last_ambient_age += dt;
    }
}

/*
 * scene_tick — THE per-tick combiner, and the ONLY function that advances
 * simulation state.  Reads as the combine order documented in ARCHITECTURE:
 *   guard pause → advance clock → continuous emission → burst timers →
 *   integrate particles.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->time += dt;

    const EruptionPattern *pp = &patterns[s->current_pattern];

    spawn_continuous(s, pp, dt);
    maybe_vulcanian_burst(s, pp);
    maybe_ambient_burst(s, dt);
    particles_tick(s->ejecta, dt, s->rows, s->cols);
}

/* ===================================================================== */
/* §6  RENDER — state → screen (read-only)                                */
/*                                                                        */
/* Every function here READS the Mountain / ejecta pool (via narrow const  */
/* pointers) and paints cells; none writes simulation state.  Reordering   */
/* or skipping any draw cannot change the simulation.  All cosmetic shading*/
/* (glow pulse, dimming, brightness slots) is DERIVED here from time/temp. */
/* ===================================================================== */

/*
 * draw_bomb_trail — paint one bomb's fading smoke trail.  Skips index 0
 * (that's the live head, drawn in pass 2); each older sample k uses a cooler
 * ramp slot (temp-slot minus k) and a dimmer glyph so the tail recedes.
 */
static void draw_bomb_trail(const Particle *p, int rows_eff, int cols, bool inverted)
{
    for (int k = 1; k < p->trail_n; k++) {
        int tx = (int)(p->tx[k] + 0.5f);
        int ty = (int)(p->ty[k] + 0.5f);
        if (tx < 0 || tx >= cols)     continue;
        if (ty < 0 || ty >= rows_eff) continue;
        int slot = unit_to_slot(p->temp, 6) - k;   /* cooler further back */
        if (slot < 0) slot = 0;
        if (slot > 5) slot = 5;
        char glyph = (k == 1) ? '*' : (k == 2 ? '+' : '.');
        attr_t a = inverted ? A_NORMAL : A_DIM;
        attron(COLOR_PAIR(PAIR_LAVA_BASE + slot) | a);
        mvaddch(ty, tx, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(PAIR_LAVA_BASE + slot) | a);
    }
}

/*
 * draw_particle_head — choose glyph + colour pair + attribute for one
 * particle from its type and temperature, then stamp it at (ix,iy).  Hotter
 * temp → higher ramp slot → brighter glyph; each type uses its own ramp.
 */
static void draw_particle_head(const Particle *p, int ix, int iy, bool inverted)
{
    float temp = p->temp;
    if (temp < 0) temp = 0;
    if (temp > 1) temp = 1;

    char   glyph;
    int    pair;
    attr_t attr;

    switch (p->type) {
    case PT_BOMB: {
        int slot = unit_to_slot(temp, 6);
        glyph = LAVA_GLYPHS[slot];
        pair  = PAIR_LAVA_BASE + slot;
        attr  = (slot >= 4 && !inverted) ? A_BOLD : A_NORMAL;
        break;
    }
    case PT_EMBER: {
        int slot = unit_to_slot(temp, 6);
        glyph = EMBER_GLYPHS[slot];
        pair  = PAIR_LAVA_BASE + slot;
        attr  = (slot <= 1 && !inverted) ? A_DIM
              : (slot >= 5 && !inverted) ? A_BOLD
              :                             A_NORMAL;
        break;
    }
    case PT_ASH: {
        int slot = unit_to_slot(temp, 4);
        glyph = ASH_GLYPHS[slot];
        pair  = PAIR_PLUME_BASE + slot;
        attr  = inverted ? A_NORMAL : A_DIM;
        break;
    }
    case PT_SPARK:
    default: {
        int slot = unit_to_slot(temp, 3);
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

/*
 * particles_draw — two passes over the pool: trails first so live heads
 * stamp over them, then each particle's head projected to a screen cell.
 */
static void particles_draw(const Particle *pool, int rows, int cols, bool inverted)
{
    int rows_eff = rows - 1;       /* leave bottom row for HUD */

    /* Pass 1 — bomb trails (drawn under the heads). */
    for (int i = 0; i < PARTICLES_MAX; i++) {
        const Particle *p = &pool[i];
        if (p->active && p->type == PT_BOMB)
            draw_bomb_trail(p, rows_eff, cols, inverted);
    }

    /* Pass 2 — every live particle's head, projected and culled to screen. */
    for (int i = 0; i < PARTICLES_MAX; i++) {
        const Particle *p = &pool[i];
        if (!p->active) continue;
        int ix = (int)(p->x + 0.5f);
        int iy = (int)(p->y + 0.5f);
        if (ix < 0 || ix >= cols)     continue;
        if (iy < 0 || iy >= rows_eff) continue;
        draw_particle_head(p, ix, iy, inverted);
    }
}

/* Plume cone half-width (cells) at height h above the crater. */
static inline float plume_half_width(float h)
{
    return PLUME_BASE_W + h * PLUME_SPREAD;
}

/* Parabolic edge fade across the cone: 1 on the axis, 0 at the wall
 * (|dx| == half_w), negative beyond (caller treats <= 0 as outside). */
static inline float plume_cone_falloff(float dx, float half_w)
{
    return 1.0f - (dx * dx) / (half_w * half_w);
}

/*
 * plume_density — fBm smoke density at one cell.  The noise field is
 * ADVECTED: the sample x drifts with time·PLUME_WIND (wind) and the sample
 * y with time·PLUME_RISE (the column boils upward), giving a coherent
 * flowing column.  Weighted by style intensity and the cone edge fade.
 * Refs [4][5].
 */
static float plume_density(float dx, float h, float time,
                           float intensity, float falloff)
{
    float nx = dx * PLUME_NOISE_S - time * PLUME_WIND * PLUME_NOISE_S;
    float ny = h  * PLUME_NOISE_S - time * PLUME_RISE * PLUME_NOISE_S;
    return fbm2d(nx, ny, PLUME_NOISE_SEED) * intensity * falloff;
}

/*
 * plume_draw — for each cell inside the widening cone above the crater,
 * sample the advected fBm density and, if it clears the threshold, paint it
 * in the plume ramp.  Reads top-to-bottom: cone width → inside-cone test →
 * skip rock → edge fade → density → threshold → slot → paint.
 */
static void plume_draw(const Mountain *mtn, int rows, int cols,
                          float time, float intensity, bool inverted)
{
    int rows_eff = rows - 1;
    int crater_x = mtn->crater_x;
    int crater_y = mtn->crater_y;

    for (int row = 0; row < rows_eff; row++) {
        if (row > crater_y) break;     /* below crater = inside mountain */

        float h = (float)(crater_y - row);     /* height above the vent */
        if (h < 0.5f) continue;
        float half_w = plume_half_width(h);

        for (int col = 0; col < cols; col++) {
            float dx = (float)(col - crater_x);
            if (fabsf(dx) > half_w) continue;             /* outside cone   */
            if (row > mtn->silhouette_y[col]) continue;   /* inside mountain*/

            float falloff = plume_cone_falloff(dx, half_w);
            if (falloff <= 0) continue;

            float density = plume_density(dx, h, time, intensity, falloff);
            if (density < PLUME_THRESHOLD) continue;

            /* Above-threshold density, normalised, → plume ramp slot. */
            float t_n = (density - PLUME_THRESHOLD) / (1.0f - PLUME_THRESHOLD);
            if (t_n > 1.0f) t_n = 1.0f;
            int slot = unit_to_slot(t_n, 4);

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

/* Lowest rock row across all columns — the sky-gradient horizon (never 0). */
static int silhouette_max(const Mountain *mtn, int cols)
{
    int m = 0;
    for (int x = 0; x < cols; x++)
        if (mtn->silhouette_y[x] > m) m = mtn->silhouette_y[x];
    return m < 1 ? 1 : m;
}

/*
 * scene_draw_sky — vertical gradient over rows above the silhouette.
 * Each cell's slot is determined by row position: top → slot 0 (deepest
 * sky), horizon → slot 3 (palest).
 */
static void scene_draw_sky(const Mountain *mtn, int cols, int rows, bool inverted)
{
    int rows_eff = rows - 1;
    int horizon_y = silhouette_max(mtn, cols);   /* gradient normalised over this */

    for (int row = 0; row < rows_eff; row++) {
        if (row >= horizon_y) break;
        int slot = (row * 4) / horizon_y;
        if (slot < 0) slot = 0;
        if (slot > 3) slot = 3;

        char glyph = inverted ? ' ' : ' ';   /* sky cells get bg colour only */
        attr_t attr = (inverted) ? A_NORMAL
                    : (slot == 0) ? A_DIM : A_NORMAL;
        int pair = PAIR_SKY_BASE + slot;
        attron(COLOR_PAIR(pair) | attr);
        for (int col = 0; col < cols; col++) {
            if (row > mtn->silhouette_y[col]) continue;     /* mountain   */
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
static void scene_draw_mountain(const Mountain *mtn, int cols, int rows, bool inverted)
{
    int rows_eff = rows - 1;
    attr_t attr = inverted ? A_NORMAL : A_BOLD;
    attron(COLOR_PAIR(PAIR_MOUNTAIN) | attr);
    for (int col = 0; col < cols; col++) {
        int top = mtn->silhouette_y[col];
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
static void scene_draw_lava_flows(const Mountain *mtn, int cols, int rows,
                                  float lava_flow_amount, bool inverted)
{
    int rows_eff = rows - 1;
    if (lava_flow_amount < FLOW_VISIBLE_MIN) return;

    for (int side = 0; side < 2; side++) {
        const int *xs = (side == 0) ? mtn->flow_l_x : mtn->flow_r_x;
        const int *ys = (side == 0) ? mtn->flow_l_y : mtn->flow_r_y;
        int n          = (side == 0) ? mtn->flow_l_n : mtn->flow_r_n;

        for (int i = 0; i < n; i++) {
            float frac   = (float)i / (float)(n > 1 ? n - 1 : 1);
            float bright = (1.0f - frac) * lava_flow_amount;
            int   slot   = 5 - (int)(bright * 6.0f);
            if (slot < 0) slot = 0;
            if (slot > 5) slot = 5;

            int  yy = ys[i];
            int  xx = xs[i];
            if (yy < 0 || yy >= rows_eff) continue;
            if (xx < 0 || xx >= cols)     continue;

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
static void scene_draw_crater_glow(const Mountain *mtn, int cols, int rows,
                                   float time, bool inverted)
{
    int rows_eff = rows - 1;
    int cx = mtn->crater_x;
    int cy = mtn->crater_y;

    for (int dx = -(int)CRATER_RADIUS + 1;
             dx <= (int)CRATER_RADIUS - 1; dx++) {
        int x = cx + dx;
        if (x < 0 || x >= cols) continue;
        int y = cy - 1;            /* one row above the bowl bottom */
        if (y < 0 || y >= rows_eff) continue;

        float pulse = 0.5f + 0.5f * sinf(time * GLOW_PULSE_RATE + (float)dx * 0.5f);
        int slot = 4 + (int)(pulse * 1.5f);     /* pulse between the 2 hottest slots */
        if (slot < 4) slot = 4;
        if (slot > 5) slot = 5;
        char glyph = LAVA_GLYPHS[slot];
        attr_t attr = inverted ? A_NORMAL : A_BOLD;
        attron(COLOR_PAIR(PAIR_LAVA_BASE + slot) | attr);
        mvaddch(y, x, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(PAIR_LAVA_BASE + slot) | attr);
    }
}

/* Pure read — counts active particles for the HUD; mutates nothing. */
static int particles_active_count(const Particle *pool)
{
    int n = 0;
    for (int i = 0; i < PARTICLES_MAX; i++)
        if (pool[i].active) n++;
    return n;
}

static void scene_render(const Scene *s)
{
    bool                   inverted = themes[s->current_theme].inverted;
    const EruptionPattern *style    = &patterns[s->current_pattern];
    int                    rows_eff = s->rows - 1;

    /* Inverted theme: pre-fill white "paper" before drawing. */
    if (inverted) {
        attron(COLOR_PAIR(PAIR_PAPER));
        for (int row = 0; row < rows_eff; row++)
            for (int col = 0; col < s->cols; col++)
                mvaddch(row, col, ' ');
        attroff(COLOR_PAIR(PAIR_PAPER));
    }

    scene_draw_sky        (&s->mountain, s->cols, s->rows, inverted);
    scene_draw_mountain   (&s->mountain, s->cols, s->rows, inverted);
    scene_draw_lava_flows (&s->mountain, s->cols, s->rows,
                           style->lava_flow_amount, inverted);
    scene_draw_crater_glow(&s->mountain, s->cols, s->rows, s->time, inverted);

    plume_draw    (&s->mountain, s->rows, s->cols, s->time,
                   style->plume_intensity, inverted);
    particles_draw(s->ejecta, s->rows, s->cols, inverted);
}

/* ===================================================================== */
/* §7  EVENTS — init / resize / reset (mutate state OUTSIDE the tick)     */
/*                                                                        */
/* These rebuild the world (mountain + particle pool) and reset Scene     */
/* timers.  They mutate scene.mountain, scene.ejecta, g_rng and Scene —   */
/* but they run                                                           */
/* from startup / the r key / SIGWINCH, NEVER from scene_tick.  Keeping   */
/* them out of the tick is what lets §5's combine order stay the single   */
/* source of simulation advancement.                                     */
/* ===================================================================== */

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

    mountain_build(&s->mountain, cols, rows, s->seed);
    particles_clear(s->ejecta);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
    mountain_build(&s->mountain, cols, rows, s->seed);
    particles_clear(s->ejecta);
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
    mountain_build(&s->mountain, s->cols, s->rows, s->seed);
    particles_clear(s->ejecta);
}

/* ===================================================================== */
/* §8  SCREEN + APP — ncurses I/O, HUD, input, main loop + frame cap      */
/*                                                                        */
/* Terminal setup/teardown + HUD (RENDER infrastructure), the input       */
/* handler (EVENTS), and the main loop which owns the PERFORMANCE         */
/* frame cap (dt clamp, fps measure, target-Hz sleep).                    */
/* ===================================================================== */

/*
 * Screen — the terminal viewport in character cells: the authoritative size
 * read from ncurses getmaxyx (ref [8]) at startup and after every SIGWINCH.
 * Scene keeps its own cols/rows copy for the simulation; THIS is ground
 * truth and drives the resize that rebuilds the mountain to the new extent.
 */
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

    /* ── top row: data readout (yellow), clipped so it never wraps ── */
    char buf[200];
    snprintf(buf, sizeof buf,
             " VOLCANO   %s   pattern:%s   theme:%s   particles:%4d   "
             "intensity:%4.2f   %5.1f fps  %3d Hz ",
             s->paused ? "PAUSED " : "ERUPT  ",
             patterns[s->current_pattern].name,
             themes[s->current_theme].name,
             particles_active_count(s->ejecta),
             (double)s->intensity,
             fps, sim_fps);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(0, x, ' ');
    mvprintw(0, 0, "%.*s", sc->cols, buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* ── bottom row: action keys (cyan), clipped ── */
    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%.*s", sc->cols,
             " spc:pause  r:reseed  n/N:pat  t/T:theme  +/-:int  [/]:Hz  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/*
 * App — the top-level program aggregate.  A single static instance (g_app)
 * exists so the async signal handlers can reach the run flags.  It bundles
 * the simulation (scene), the terminal it draws to (screen), the run-rate
 * knob, and the two flags the handlers poke.  running/need_resize are
 * `volatile sig_atomic_t`: volatile stops the main loop from caching them
 * in a register (the handler can change them between iterations), and
 * sig_atomic_t guarantees the read/write is indivisible across a signal.
 */
typedef struct {
    Scene                 scene;       /* the whole simulation + its render state */
    Screen                screen;      /* terminal viewport (ground-truth dims)   */
    int                   sim_fps;     /* target ticks/sec — the frame cap ([ / ])*/
    volatile sig_atomic_t running;     /* SIGINT/SIGTERM clear it → main loop exits */
    volatile sig_atomic_t need_resize; /* SIGWINCH sets it → rebuild on next frame  */
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
        s->intensity *= INTENSITY_STEP_UP;
        if (s->intensity > INTENSITY_MAX) s->intensity = INTENSITY_MAX;
        break;
    case '-':
        s->intensity *= INTENSITY_STEP_DOWN;
        if (s->intensity < INTENSITY_MIN) s->intensity = INTENSITY_MIN;
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

        /* 1. apply a pending terminal resize before measuring time. */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
        }

        /* 2. timestep since last frame, capped against spiral-of-death. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_MS * NS_PER_MS) dt = DT_CAP_MS * NS_PER_MS;

        /* 3. advance the simulation by that timestep. */
        float dt_sec = (float)dt / (float)NS_PER_SEC;
        scene_tick(&app->scene, dt_sec);

        /* 4. rolling fps measurement (refresh every FPS_UPDATE_MS). */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* 5. frame cap — sleep out the remainder of this tick's budget. */
        int64_t target_ns = TICK_NS(app->sim_fps);
        int64_t elapsed   = clock_ns() - frame_time + dt;
        clock_sleep_ns(target_ns - elapsed);

        /* 6. render the frame + HUD. */
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* 7. drain one input event. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
