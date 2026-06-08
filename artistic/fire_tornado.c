/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fire_tornado.c — fire tornado from a swirling pool of embers (Stage 2)
 *
 * DEMO: A tall conical column of embers rotates around a central vertical
 *       axis, narrowing as it rises. Each ember lives in cylindrical
 *       coordinates (height, phase, radius); a side-view 2-D projection
 *       maps cylindrical position to screen cells, with depth conveyed
 *       by which side of the column the ember is currently on (sin of
 *       its phase). Heat fades from white-hot at the base to dim red at
 *       the top, both via a 5-stop ASCII ramp (`# o * . `) and a
 *       matching colour ramp.
 *
 *       Stage 2 adds three layers of embellishment on top of Stage 1:
 *         • A 1-D heat strip at the ground row that flickers, diffuses
 *           and self-injects fuel (a tiny cellular-automaton fire mat).
 *         • Sparks — particles spawned at random ember positions with
 *           outward radial velocity; they fly out, fall under gravity,
 *           cool fast, and disappear.
 *         • Wind — a slow horizontal sway of the upper funnel via a
 *           single sinusoid, scaled by `(y / height)` so the base stays
 *           planted while the top swings.
 *
 * Study alongside: particle_systems/fire.c (heat-diffusion CA — different
 *                  approach; this file uses 3-D-ish particles instead),
 *                  particle_systems/smoke.c (similar pool + buoyancy
 *                  pattern, no rotation),
 *                  flocking/flocking.c (similar agent + force model).
 *
 * Section map (cut by layer — see ARCHITECTURE):
 *   §1 config    — pool size, height, base radius, heat / spark / wind knobs
 *   §2 clock     — PERFORMANCE: monotonic timer + sleep
 *   §3 data      — DATA: heat/colour tables + Ember / Spark / Tornado types
 *   §4 logic     — LOGIC: frand, heat_bucket, wind_offset (pure)
 *   §5 ember     — SIMULATION: ember respawn + tick (the rotating column)
 *   §6 effects   — EFFECTS: base flame mat + spark tick/spawn (embellishment)
 *   §7 init      — INIT/RESET: geometry + reseed
 *   §8 combine   — the per-tick combine (tornado_tick)
 *   §9 render    — RENDER: colour init + ember/spark/base/HUD draws
 *   §10 screen   — ncurses init / cleanup
 *   §11 app      — signals, the frame loop, key handling
 *
 * Keys:  [/]   ember count (50..600)
 *        -/+   spin speed (0.25× .. 4×)
 *        ,/.   funnel height (shorter / taller)
 *        w     toggle wind (off / default amplitude)
 *        t     cycle theme   r reseed   p pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/fire_tornado.c \
 *       -o fire_tornado -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Three layered systems share the same heat ramp:
 *                 (1) ember pool — cylindrical particles around the axis
 *                     (Stage 1); each ember rises, rotates, drifts inward,
 *                     cools, and respawns at the base.
 *                 (2) base flame mat — 1-D heat array at the ground row.
 *                     Each frame: exponential decay, 3-tap sideways
 *                     diffusion, and 1–3 random heat injections per
 *                     frame biased toward the axis (triangular dist).
 *                     Renders as flickering glyphs along the bottom.
 *                 (3) spark pool — small particles spawned at random
 *                     ember positions with outward radial velocity;
 *                     gravity pulls them down, they cool fast, draw
 *                     bright then fade out.
 *                 Wind is a single horizontal sinusoid in time, scaled
 *                 by `y / height` so the base is fixed and the upper
 *                 funnel sways. Applied uniformly to every projection.
 *
 * Data-structure: Tornado aggregates Ember[N_EMBERS_MAX], Spark[N_SPARKS_MAX]
 *                 and a FlameMat (the 1-D ground fire mat) inline. No
 *                 allocation after init; active counts/modes are knobs.
 *
 * Rendering     : Per frame, in this draw order:
 *                   base_heat → embers (back) → embers (front) → sparks
 *                 The two-pass ember loop preserves depth ordering. The
 *                 base flame mat draws first so embers near the base
 *                 (y ≈ 0) overpaint it cleanly. Sparks draw last so
 *                 they appear in front of everything else.
 *
 * Performance   : O(N_EMBERS + N_SPARKS + BASE_HEAT_W) per frame. At
 *                 N_EMBERS=250, N_SPARKS=40, BASE_HEAT_W=60 it's a few
 *                 thousand mvaddch + a 1-D blur per frame — trivial.
 *
 * References    :
 *   Particle systems & fire (the ember / spark pools, §5/§6):
 *     Reeves, "Particle Systems — A Technique for Modelling a Class of
 *       Fuzzy Objects" SIGGRAPH (1983) — the foundational fire-particle
 *       paper; this file is a stylised cylindrical-pool variant.
 *     Nguyen, Fedkiw & Jensen, "Physically Based Modeling and Animation
 *       of Fire" SIGGRAPH (2002) — the canonical physically-based fire
 *       model; the rigorous counterpoint to this file's cheap stylisation.
 *     Witkin & Baraff, "Physically Based Modeling: Principles and
 *       Practice" SIGGRAPH course notes — explicit Euler integration, the
 *       basis for ember/spark advance and spark gravity (ember_tick /
 *       spark_tick: pos += vel·dt, vel += accel·dt).
 *
 *   1-D heat fire mat (the base flame strip, §6):
 *     Sanglard, "Game Engine Black Book: DOOM" (2018) ch. 11 — the 1-D
 *       heat-diffusion fire algorithm that inspired base_heat_tick.
 *     Stam, "Real-Time Fluid Dynamics for Games" GDC (2003) — the
 *       diffusion step; the 3-tap blur in base_heat_tick is one discrete
 *       pass of the heat equation (decay + spread).
 *
 *   Rendering (ASCII ramp + depth ordering, §9):
 *     Bourke, "Character representation of greyscale images"
 *       (paulbourke.net/dataformats/asciiart/) — the heat → glyph ramp
 *       (HEAT_GLYPH / heat_bucket).
 *     Foley, van Dam, Feiner & Hughes, "Computer Graphics: Principles
 *       and Practice" — the painter's algorithm / back-to-front depth
 *       sort behind the two-pass (back then front) ember draw in scene_draw.
 *
 * ─────────────────────────────────────────────────────────────────────── */


/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * The file is cut into LAYERS by concern. All state lives on one Tornado
 * aggregate (§3); each layer reads and/or mutates a named slice of it. The
 * const-pointer "reads vs mutates" signature convention is deferred to a later
 * types pass — here the split is by SECTION and documented in this table.
 *
 *   Layer        Section            Mutates
 *   ─────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2 clock           nothing (reads OS clock, sleeps)
 *   DATA         §3 data            — type & lookup-table declarations only —
 *   LOGIC        §4 logic           nothing (pure: maps inputs → values)
 *   SIMULATION   §5 ember           tornado.embers (the rotating column)
 *   EFFECTS      §6 effects         tornado.base_heat, tornado.sparks
 *   INIT/RESET   §7 init            ALL tornado state (geometry + reseed)
 *   —            §8 combine         tornado_tick — the per-tick combine
 *   RENDER       §9 render          the screen only — never tornado state
 *   —            §10 screen         ncurses init / teardown
 *   —            §11 app            signals, the frame loop, key events
 *
 * EFFECTS is real here: the base flame mat and the sparks are cosmetic-only
 * embellishments (Stage 2). Sparks READ ember positions to spawn from, but
 * neither sparks nor base_heat are ever read back by the ember simulation, so
 * deleting them cannot change the column. (The DEMO header calls them exactly
 * that: "three layers of embellishment on top of Stage 1".)
 *
 * LOGIC (frand, heat_bucket, wind_offset) does no mutation and no I/O — only
 * frand advances the shared PRNG — so reordering or deleting RENDER/EFFECTS
 * cannot change a LOGIC result.
 *
 * No DELAYS layer: pause is a single flag (tornado.paused) tested once in main
 * before the tick; world_time and the spark-spawn accumulator are owned by and
 * advanced inside the tick. There are no global holds.
 *
 * PERFORMANCE is the §2 clock plus main's loop: a fixed FRAME_NS cap, a
 * DT_CAP_S clamp on dt, and an exponential fps average — all in §11, not a
 * separate per-tick layer.
 *
 * PER-TICK COMBINE — tornado_tick (§8) is the ONLY place sim/effect state
 * advances, in order:
 *   1. ember_tick     each ember            (SIMULATION)
 *   2. base_heat_tick the base flame mat    (EFFECTS)
 *   3. spark_tick     each spark            (EFFECTS)
 *   4. spawn sparks at the accumulated rate (EFFECTS)
 *   5. advance world_time (the wind clock)
 *
 * User events (quit, pause, reset r, theme t, wind w, embers [/], spin -/+,
 * height ,/. and resize) DO mutate state but are NOT part of the tick — they
 * run in main's input/resize handling, outside and before the gated
 * `if (!paused) tornado_tick(...)`. Reset r and resize re-invoke INIT (§7).
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

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS          60

/* Pool sizing. Embers above N_EMBERS_DEFAULT are unused but the array
 * is sized for the max so '[' / ']' just changes the active count. */
#define N_EMBERS_DEFAULT    250
#define N_EMBERS_MIN         50
#define N_EMBERS_MAX        600
#define N_EMBERS_STEP        50

/* Geometry. Height is in screen cells; base radius is in cells too. */
#define HEIGHT_FRAC          0.85f
#define HEIGHT_FRAC_MIN      0.40f
#define HEIGHT_FRAC_MAX      0.95f
#define HEIGHT_FRAC_STEP     0.05f
#define BASE_RADIUS_CELLS    9.0f

/* Dynamics. y_vel and cool rate are tuned so the average ember spends
 * ~3 seconds rising from base to top, plenty of time to spin visibly. */
#define Y_VEL_BASE           6.0f
#define OMEGA_BASE           5.0f
#define OMEGA_MULT_DEFAULT   1.0f
#define OMEGA_MULT_MIN       0.25f
#define OMEGA_MULT_MAX       4.0f
#define OMEGA_MULT_STEP      0.25f
#define RADIUS_DECAY         0.6f
#define COOL_RATE            0.32f

/* Ember spawn profile — inner embers rise & spin faster so the column tightens
 * into a funnel (set once per spawn in ember_respawn). */
#define EMBER_RISE_AXIS      1.4f   /* rise-speed gain on the axis (× Y_VEL_BASE) */
#define EMBER_RISE_FALLOFF   0.7f   /* … minus this × normalized radius (edge ≈ 0.7×) */
#define EMBER_OMEGA_RSCALE   0.4f   /* omega = OMEGA_BASE / max(radius·this, floor)  */
#define EMBER_OMEGA_FLOOR    0.6f   /* … floor so axis embers don't spin infinitely  */
#define EMBER_TEMP_MIN       0.92f  /* spawn heat in [MIN, MIN+JITTER)               */
#define EMBER_TEMP_JITTER    0.08f
#define EMBER_INIT_COOL      0.7f   /* initial fill is cooler with height (1 − this·y/h) */

/* Cell aspect — terminal cells are ~2× taller than wide. Stretch the
 * horizontal radius to compensate so the column reads as round. */
#define ASPECT_X             2.0f

/* Base flame mat — 1-D heat strip at the ground row. */
#define BASE_HEAT_W          60       /* cells, centred on the axis        */
#define BASE_HEAT_DECAY      2.0f     /* heat *= max(0, 1 − decay·dt)      */
#define BASE_INJECT_MIN      1        /* injections per frame, lower bound */
#define BASE_INJECT_MAX      3
#define BASE_INJECT_HEAT     0.85f    /* injected heat level (+ small jitter) */
#define BASE_INJECT_SPREAD   0.35f    /* injection spread = this × width (axis-biased) */
#define BASE_INJECT_JITTER   0.15f    /* + up to this much heat jitter per injection   */
#define BASE_HEAT_VISIBLE    0.05f    /* skip drawing flame cells dimmer than this     */

/* Sparks — particles thrown outward from the column. */
#define N_SPARKS_MAX         40
#define SPARK_SPAWN_HZ      10.0f     /* spawns per second on average      */
#define SPARK_SPEED_MIN      8.0f     /* cells/sec at spawn                 */
#define SPARK_SPEED_MAX     20.0f
#define SPARK_GRAVITY       20.0f     /* cells/sec², pulls sparks down     */
#define SPARK_COOL           1.5f     /* temp/sec — sparks cool fast        */
#define SPARK_GLYPH          '*'
#define SPARK_SRC_TRIES      8        /* rejection-sample tries to find a live source ember */
#define SPARK_VY_SCALE       0.5f     /* vertical launch speed = this × speed (cells are tall) */
#define SPARK_UP_BIAS        0.3f     /* … minus this × speed → net upward launch           */
#define SPARK_TEMP_MIN       0.95f    /* spawn heat in [MIN, MIN+JITTER)                    */
#define SPARK_TEMP_JITTER    0.05f

/* Wind — slow horizontal sway of the upper funnel. */
#define WIND_AMP_DEFAULT     3.0f     /* max cells of horizontal tilt at top */
#define WIND_FREQ            0.45f    /* rad/sec of axis sway                */

#define DT_CAP_S             0.10f
#define N_THEMES             4

/* Colour pair IDs */
#define PAIR_HEAT_0   1   /* coolest                                       */
#define PAIR_HEAT_1   2
#define PAIR_HEAT_2   3
#define PAIR_HEAT_3   4
#define PAIR_HEAT_4   5   /* hottest — white                                */
#define PAIR_HUD      6
#define PAIR_HINT     7

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  PERFORMANCE — clock                                                 */
/* ═══════════════════════════════════════════════════════════════════════ *
 * Monotonic clock + sleep. Mutates nothing. The frame cap (FRAME_NS), dt clamp
 * (DT_CAP_S) and fps average that use these live in main's loop (§11).        */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  DATA — types & lookup tables                                        */
/* ═══════════════════════════════════════════════════════════════════════ *
 * Declarations only; no behaviour. State is mutated by SIMULATION (§5) /
 * EFFECTS (§6) / INIT (§7) and read by LOGIC (§4) / RENDER (§9).              */

/* HEAT_256 / HEAT_8 — the per-theme COLOUR RAMP: 5 foreground colours from
 * coolest (index 0) to hottest (4). WHY 5 stops paired with HEAT_GLYPH: a single
 * heat_bucket(temp) indexes BOTH tables, so glyph and colour always agree — a
 * discrete luminance ramp (Bourke, character-as-greyscale). Theme 0 walks the
 * black-body fire progression (dark red → orange → yellow → white); the other
 * themes restyle the same 5 slots. HEAT_256 holds xterm-256 indices (16..231 =
 * 6×6×6 cube, 232..255 = grays); HEAT_8 is the 8-colour fallback. One row per
 * theme; the live choice is Tornado.theme. */
static const short HEAT_256[N_THEMES][5] = {
    /* classic fire */ {  52, 196, 208, 226, 231 },
    /* blue fire    */ {  17,  21,  39,  51, 231 },
    /* toxic green  */ {  22,  28,  76, 154, 231 },
    /* hellfire     */ {  53, 127, 165, 213, 231 },
};
static const short HEAT_8[N_THEMES][5] = {
    { COLOR_RED,     COLOR_RED,     COLOR_YELLOW,  COLOR_YELLOW,  COLOR_WHITE },
    { COLOR_BLUE,    COLOR_BLUE,    COLOR_CYAN,    COLOR_CYAN,    COLOR_WHITE },
    { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW,  COLOR_WHITE },
    { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE },
};

/* HEAT_GLYPH — the 5-stop ASCII LUMINANCE RAMP, sparse → dense as heat rises
 * (` . * o #): coldest at index 0, white-hot at 4. Indexed by the SAME
 * heat_bucket(temp) as the colour ramp, so glyph and colour never disagree
 * (Bourke, "character representation of greyscale images"). */
static const char HEAT_GLYPH[5] = { '`', '.', '*', 'o', '#' };

/* Ember — one particle in the rotating fire column, the Stage-1 CORE system.
 * Each ember lives in CYLINDRICAL coordinates (height y, angular phase, radius)
 * about a central vertical axis — a stylised cylindrical-pool variant of the
 * classic particle-system fire (Reeves, SIGGRAPH 1983). Per tick it rises
 * (y += y_vel·dt), rotates (phase += omega·dt), drifts inward (radius decays),
 * and cools, then respawns at the base — explicit Euler throughout (Witkin &
 * Baraff). INTENT of the spawn values: inner embers get a faster rise
 * (y_vel ∝ 1.4 − 0.7·r_norm) and faster spin (omega ∝ 1/radius), so the column
 * reads as a tightening funnel. There is no real 3-D — DEPTH is faked from
 * sin(phase) (+1 front, −1 back) to choose A_BOLD vs A_DIM at draw time. Lives
 * in a fixed object pool (alive = slot in use, no allocation after init). */
typedef struct {
    float y;          /* height above base (cells); respawns at y >= height  */
    float phase;      /* angle around axis (radians); cos→x, sin→front/back   */
    float radius;     /* horizontal distance from axis (cells); decays inward */
    float y_vel;      /* rise speed (cells/sec), set at spawn from radius     */
    float omega;      /* spin rate (rad/sec), set at spawn (smaller r→faster) */
    float temp;       /* 0..1 heat; cools each tick, drives glyph + colour    */
    int   alive;      /* 1 while in flight (object-pool slot flag)            */
} Ember;

/* ───────── Spark — a single outward-thrown ember. ─────────────────────── */

/* Spark — one ballistic ember flung outward from a live ember's position when a
 * spark spawns: a Stage-2 cosmetic EFFECT, never read back by the column. A
 * textbook particle (Reeves) on explicit Euler under CONSTANT gravity (Witkin &
 * Baraff): each tick vy += SPARK_GRAVITY·dt then pos += vel·dt (spark_tick), no
 * inter-particle forces. Unlike an Ember it is stored already PROJECTED in
 * screen space (the cylinder→screen map is done once at spawn), so drawing is a
 * plain round-to-cell. Lives in a fixed object pool. */
typedef struct {
    float x, y;       /* screen position (cells, float; rounded at draw)     */
    float vx, vy;     /* velocity (cells/sec); outward+up, bent down by grav  */
    float temp;       /* 0..1 heat; cools fast (SPARK_COOL) → glyph + colour  */
    int   alive;      /* 1 while in flight (object-pool slot flag)            */
} Spark;

/* ───────── FlameMat — the 1-D ground fire mat (Doom-style). ──────────── */

/* FlameMat — the base flame as a 1-D HEAT STRIP along the ground row: one heat
 * value per screen column, centred on the axis (NOT a 2-D grid — the strip is
 * the lesson). It is the classic DOOM 1-D fire buffer (Sanglard, "Game Engine
 * Black Book: DOOM" ch. 11): each tick base_heat_tick decays it, blurs it
 * sideways, and self-injects fuel near the axis. The 3-tap blur is one discrete
 * pass of the heat equation (Stam, "Real-Time Fluid Dynamics for Games").
 * Cosmetic-only (EFFECTS): never read back by the ember simulation.
 *   cell[i] : heat 0..1 at column i (i in [0,BASE_HEAT_W)); selects the glyph +
 *             colour bucket at draw time. */
typedef struct {
    float cell[BASE_HEAT_W];
} FlameMat;

/* ───────── Tornado — the whole simulation as a table of contents. ─────── */

/* Tornado — the whole simulation in one aggregate, read like a table of
 * contents. WHY one aggregate: state lives in one place, yet functions still
 * take the NARROWEST slice they need (const X* read, X* mutate) — only
 * tornado_position / tornado_reseed / tornado_tick take Tornado* — so the
 * layers never re-couple. The pools are fixed-size with an `alive` flag per
 * slot; nothing is allocated after start-up.
 *   WHAT   — embers (the rotating column) plus the two cosmetic effects
 *            (sparks thrown outward, base_heat the ground fire mat).
 *   WHERE  — the funnel's screen placement (axis_x, base_y, height), derived
 *            from the terminal size and height_frac at reset/resize.
 *   HOW    — the user-tunable knobs (n active embers, height fraction, spin
 *            multiplier, wind amplitude).
 *   WHEN   — run clocks (world_time the wind clock, spark_accum the fractional
 *            spawn carry) and run state (paused).
 *   RENDER — theme: the selected colour-ramp index (a render choice cycled by
 *            't', kept here so reset/resize preserve it). */
typedef struct {
    /* WHAT — the domain objects */
    Ember    embers[N_EMBERS_MAX];   /* the rotating column      (SIMULATION) */
    Spark    sparks[N_SPARKS_MAX];   /* outward-thrown embers    (EFFECTS)    */
    FlameMat base_heat;              /* 1-D ground fire mat      (EFFECTS)    */

    /* WHERE — screen placement, set by tornado_position */
    int   axis_x;                    /* column centre x (cells)               */
    int   base_y;                    /* ground row (cells)                    */
    float height;                    /* column height (cells)                 */

    /* HOW — user-tunable simulation knobs */
    int   n;                         /* active ember count [50..600]          */
    float height_frac;               /* height as a fraction of (rows-1)      */
    float omega_mult;                /* spin multiplier [0.25..4]             */
    float wind_amp;                  /* horizontal sway amplitude; 0 = off    */

    /* WHEN — run clocks + run state */
    float world_time;                /* seconds; advances only when unpaused  */
    float spark_accum;               /* fractional spark spawns carried over  */
    int   paused;                    /* 1 freezes the tick (render continues) */

    /* RENDER — palette selection */
    int   theme;                     /* index into HEAT_256 / HEAT_8          */
} Tornado;

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  LOGIC — pure maps (no mutation, no I/O)                             */
/* ═══════════════════════════════════════════════════════════════════════ *
 * Each reads its inputs and returns a value; only frand touches global state
 * (it advances the shared PRNG). Reordering or deleting RENDER/EFFECTS cannot
 * change a result here.                                                       */

static float frand(void)
{
    return (float)rand() / (float)RAND_MAX;
}

/* Map a temperature in [0, 1] to a heat-bucket index in [0, 4]. */
static int heat_bucket(float temp)
{
    int b = (int)(temp * 4.99f);
    if (b < 0) b = 0;
    if (b > 4) b = 4;
    return b;
}

/* Wind tilt at height `y` for the current world time.  Zero at the
 * base, sinusoidal at the top, scaled by wind amplitude. */
static float wind_offset(float y, float height, float world_time, float wind_amp)
{
    if (height < 1.0f) return 0.0f;
    float t_norm = y / height;
    return wind_amp * sinf(world_time * WIND_FREQ) * t_norm;
}

/* Project an ember's CYLINDRICAL position (y, phase, radius) to a screen cell
 * (float, side view): x = axis + wind tilt + aspect-stretched radius·cos(phase),
 * y = base − height. The single source of the cylinder→screen map, shared by
 * ember_draw and the spark spawn so both stay in lock-step. */
static void ember_to_screen(const Ember *e, int axis_x, int base_y, float height,
                            float world_time, float wind_amp, float *sx, float *sy)
{
    float w_off = wind_offset(e->y, height, world_time, wind_amp);
    *sx = (float)axis_x + w_off + ASPECT_X * e->radius * cosf(e->phase);
    *sy = (float)base_y - e->y;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  SIMULATION — embers (the rotating column)                           */
/* ═══════════════════════════════════════════════════════════════════════ *
 * The core Stage-1 system: each ember rises, rotates, drifts inward, cools,
 * and respawns. Mutates only its own Ember. Advanced once per tick (§8).      */

/*
 * ember_respawn — drop the ember back at the base with a fresh random
 * phase and radius. Inner-radius embers get faster vertical velocity
 * and faster spin (omega ∝ 1/radius).
 *
 *   initial=1: spread the ember somewhere along the lifetime so the
 *              column is fully populated from frame 0 (used at startup
 *              and on 'r' / shape change).
 *   initial=0: classic respawn at the base.
 */
static void ember_respawn(Ember *e, float height, int initial)
{
    e->y      = initial ? frand() * height : 0.0f;
    e->phase  = frand() * 2.0f * (float)M_PI;
    float u   = frand();
    e->radius = BASE_RADIUS_CELLS * sqrtf(u);   /* sqrt(u) → uniform over the disk AREA */
    float r_norm = e->radius / BASE_RADIUS_CELLS;
    e->y_vel  = Y_VEL_BASE * (EMBER_RISE_AXIS - EMBER_RISE_FALLOFF * r_norm);
    e->omega  = OMEGA_BASE / fmaxf(e->radius * EMBER_OMEGA_RSCALE, EMBER_OMEGA_FLOOR);
    e->temp   = EMBER_TEMP_MIN + EMBER_TEMP_JITTER * frand();
    if (initial) e->temp *= (1.0f - EMBER_INIT_COOL * (e->y / height));
    e->alive  = 1;
}

/*
 * ember_tick — one simulation step. Advance phase, height; cool; pull
 * radius inward. If expired (top reached or burned out), respawn.
 */
static void ember_tick(Ember *e, float height, float omega_mult, float dt)
{
    if (!e->alive) {
        ember_respawn(e, height, 0);
        return;
    }
    e->phase  += e->omega * omega_mult * dt;
    e->y      += e->y_vel * dt;
    e->radius *= (1.0f - RADIUS_DECAY * dt);
    e->temp   -= COOL_RATE * dt;
    if (e->y >= height || e->temp <= 0.0f) e->alive = 0;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  EFFECTS — base flame mat + sparks (cosmetic embellishment)          */
/* ═══════════════════════════════════════════════════════════════════════ *
 * Stage-2 cosmetic systems. They advance over time and READ ember positions
 * (spark spawn) but are NEVER read back by the ember simulation, so deleting
 * them cannot change the column. Advanced once per tick (§8).                 */

/* exponential cool toward zero: heat *= max(0, 1 − BASE_HEAT_DECAY·dt) */
static void flame_decay(FlameMat *mat, float dt)
{
    float keep = 1.0f - BASE_HEAT_DECAY * dt;
    if (keep < 0.0f) keep = 0.0f;
    for (int i = 0; i < BASE_HEAT_W; i++)
        mat->cell[i] *= keep;
}

/* one [0.25 0.5 0.25] 3-tap diffusion pass with reflective edges — a single
 * discrete step of the heat equation, double-buffered via a static scratch. */
static void flame_blur(FlameMat *mat)
{
    float *heat = mat->cell;
    static float tmp[BASE_HEAT_W];
    for (int i = 0; i < BASE_HEAT_W; i++) {
        float l = (i > 0)              ? heat[i - 1] : heat[i];
        float r = (i < BASE_HEAT_W - 1) ? heat[i + 1] : heat[i];
        tmp[i] = 0.25f * l + 0.50f * heat[i] + 0.25f * r;
    }
    memcpy(heat, tmp, sizeof tmp);
}

/* inject fuel at 1..3 random columns, axis-biased via a triangular distribution
 * (sum of two uniform samples); each injection raises that cell toward full. */
static void flame_inject(FlameMat *mat)
{
    float *heat  = mat->cell;
    int n_inject = BASE_INJECT_MIN + (rand() % (BASE_INJECT_MAX - BASE_INJECT_MIN + 1));
    float center = BASE_HEAT_W * 0.5f;
    float spread = BASE_HEAT_W * BASE_INJECT_SPREAD;
    for (int k = 0; k < n_inject; k++) {
        float u   = frand() + frand() - 1.0f;          /* [-1, 1] triangular */
        int   idx = (int)(center + u * spread);
        if (idx < 0)             idx = 0;
        if (idx >= BASE_HEAT_W)  idx = BASE_HEAT_W - 1;
        float h = BASE_INJECT_HEAT + BASE_INJECT_JITTER * frand();
        if (heat[idx] < h) heat[idx] = h;
    }
}

/* base_heat_tick — one frame of the 1-D Doom fire: cool, spread, refuel. */
static void base_heat_tick(FlameMat *mat, float dt)
{
    flame_decay(mat, dt);   /* exponential cool                 */
    flame_blur(mat);        /* sideways diffusion (heat eqn)     */
    flame_inject(mat);      /* axis-biased fuel injection        */
}

/*
 * spark_tick — apply gravity, advance, cool. Die on cool-out or
 * leaving the screen.
 */
static void spark_tick(Spark *s, float dt, int rows, int cols)
{
    if (!s->alive) return;
    s->vy  += SPARK_GRAVITY * dt;
    s->x   += s->vx * dt;
    s->y   += s->vy * dt;
    s->temp -= SPARK_COOL * dt;
    if (s->temp <= 0.0f
        || s->x < 0 || s->x >= cols
        || s->y < 0 || s->y >= rows - 1)
        s->alive = 0;
}

/* first free slot in the spark pool, or -1 if full (object-pool allocate) */
static int spark_free_slot(const Spark *sparks)
{
    for (int i = 0; i < N_SPARKS_MAX; i++)
        if (!sparks[i].alive) return i;
    return -1;
}

/* rejection-sample a live ember to source a spark from (up to SPARK_SRC_TRIES
 * tries); -1 if none of the tries landed on a live one. */
static int random_live_ember(const Ember *embers, int n)
{
    for (int tries = 0; tries < SPARK_SRC_TRIES; tries++) {
        int k = rand() % n;
        if (embers[k].alive) return k;
    }
    return -1;
}

/*
 * tornado_spawn_spark — allocate a spark slot, pick a live source ember,
 * project it to screen, and launch the spark outward with an upward bias.
 */
static void tornado_spawn_spark(Tornado *t)
{
    int slot = spark_free_slot(t->sparks);
    if (slot < 0) return;
    int src = random_live_ember(t->embers, t->n);
    if (src < 0) return;
    const Ember *e = &t->embers[src];

    float x, y;
    ember_to_screen(e, t->axis_x, t->base_y, t->height, t->world_time, t->wind_amp, &x, &y);

    Spark *s = &t->sparks[slot];
    s->x = x;
    s->y = y;
    float ang   = frand() * 2.0f * (float)M_PI;
    float speed = SPARK_SPEED_MIN + frand() * (SPARK_SPEED_MAX - SPARK_SPEED_MIN);
    s->vx = cosf(ang) * speed * ASPECT_X;
    s->vy = sinf(ang) * speed * SPARK_VY_SCALE - speed * SPARK_UP_BIAS;
    s->temp  = SPARK_TEMP_MIN + SPARK_TEMP_JITTER * frand();
    s->alive = 1;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  INIT/RESET — geometry & reseed (setup, NOT part of the tick)        */
/* ═══════════════════════════════════════════════════════════════════════ *
 * Mutates ALL tornado state wholesale. Called at startup, on 'r', on any shape
 * change, and on resize — all outside tornado_tick.                           */

static void tornado_position(Tornado *t, int rows, int cols)
{
    t->axis_x = cols / 2;
    t->base_y = rows - 2;
    t->height = (float)(rows - 1) * t->height_frac;
}

/*
 * tornado_reseed — re-randomise all embers along the lifetime, clear
 * the base heat strip, kill all sparks, reset the wind clock. Called
 * at startup and on 'r' or any shape change.
 */
static void tornado_reseed(Tornado *t)
{
    for (int i = 0; i < t->n; i++)
        ember_respawn(&t->embers[i], t->height, /*initial=*/1);
    for (int i = 0; i < BASE_HEAT_W; i++)
        t->base_heat.cell[i] = 0.0f;
    for (int i = 0; i < N_SPARKS_MAX; i++)
        t->sparks[i].alive = 0;
    t->spark_accum = 0.0f;
    t->world_time  = 0.0f;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  COMBINE — the per-tick advance (the ONLY place state moves forward) */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * tornado_tick — advance every sub-system by dt. Sparks spawn at a
 * fractional rate handled via accumulator + while-loop so spawn rate
 * stays accurate at any frame rate.
 */
static void tornado_tick(Tornado *t, float dt, int rows, int cols)
{
    /* Embers. */
    for (int i = 0; i < t->n; i++)
        ember_tick(&t->embers[i], t->height, t->omega_mult, dt);

    /* Base flame mat. */
    base_heat_tick(&t->base_heat, dt);

    /* Sparks. */
    for (int i = 0; i < N_SPARKS_MAX; i++)
        spark_tick(&t->sparks[i], dt, rows, cols);

    t->spark_accum += SPARK_SPAWN_HZ * dt;
    while (t->spark_accum >= 1.0f) {
        t->spark_accum -= 1.0f;
        tornado_spawn_spark(t);
    }

    t->world_time += dt;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §9  RENDER — state → screen (reads only, never mutates tornado state)   */
/* ═══════════════════════════════════════════════════════════════════════ */

static void color_init(int theme)
{
    start_color(); use_default_colors();
    int x256 = (COLORS >= 256);
    for (int i = 0; i < 5; i++) {
        short fg = x256 ? HEAT_256[theme][i] : HEAT_8[theme][i];
        init_pair((short)(PAIR_HEAT_0 + i), fg, -1);
    }
    init_pair(PAIR_HUD,  x256 ? 226 : COLOR_YELLOW, -1);  /* bright yellow — top data bar    */
    init_pair(PAIR_HINT, x256 ?  51 : COLOR_CYAN,   -1);  /* bright cyan   — bottom action bar */
}

/*
 * ember_draw — draw one ember for the given depth pass. Cull embers on the
 * wrong side of the column (depth = sin(phase): +1 front, −1 back), project the
 * rest via ember_to_screen, then emit the heat glyph — A_BOLD on the near side,
 * A_DIM on the far side, the alternation that makes the rotation legible from a
 * 2-D side view.
 */
static void ember_draw(const Ember *e,
                       int axis_x, int base_y, float height,
                       float world_time, float wind_amp,
                       int rows, int cols, int back_pass)
{
    if (!e->alive) return;
    float depth = sinf(e->phase);
    int   is_back = (depth < 0.0f);
    if ( back_pass &&  !is_back) return;   /* this pass draws only the far side  */
    if (!back_pass &&   is_back) return;   /* … and this one only the near side  */

    float sx, sy;
    ember_to_screen(e, axis_x, base_y, height, world_time, wind_amp, &sx, &sy);
    int sc = (int)sx, sr = (int)sy;
    if (sr < 0 || sr >= rows - 1) return;
    if (sc < 0 || sc >= cols)     return;

    int    bucket = heat_bucket(e->temp);
    chtype attr   = COLOR_PAIR(PAIR_HEAT_0 + bucket);
    attr |= is_back ? A_DIM : A_BOLD;
    attron(attr);
    mvaddch(sr, sc, (chtype)(unsigned char)HEAT_GLYPH[bucket]);
    attroff(attr);
}

static void spark_draw(const Spark *s, int rows, int cols)
{
    if (!s->alive) return;
    int sr = (int)s->y, sc = (int)s->x;
    if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) return;
    int bucket = heat_bucket(s->temp);
    attron(COLOR_PAIR(PAIR_HEAT_0 + bucket) | A_BOLD);
    mvaddch(sr, sc, (chtype)(unsigned char)SPARK_GLYPH);
    attroff(COLOR_PAIR(PAIR_HEAT_0 + bucket) | A_BOLD);
}

static void base_heat_draw(const FlameMat *mat, int axis_x, int base_y,
                           int rows, int cols)
{
    if (base_y < 0 || base_y >= rows - 1) return;
    const float *heat = mat->cell;
    int half = BASE_HEAT_W / 2;
    for (int i = 0; i < BASE_HEAT_W; i++) {
        if (heat[i] < BASE_HEAT_VISIBLE) continue;
        int sc = axis_x - half + i;
        if (sc < 0 || sc >= cols) continue;
        int bucket = heat_bucket(heat[i]);
        attron(COLOR_PAIR(PAIR_HEAT_0 + bucket) | A_BOLD);
        mvaddch(base_y, sc, (chtype)(unsigned char)HEAT_GLYPH[bucket]);
        attroff(COLOR_PAIR(PAIR_HEAT_0 + bucket) | A_BOLD);
    }
}

/*
 * draw_hud — the standard two bars, a read-only summary of the whole tornado:
 *   Top row 0      DATA    — title (left) + sim stats (right-aligned, yellow).
 *   Bottom rows-1  ACTIONS — the key legend (cyan).
 * Both rows are filled first, then clipped with "%.*s" so a narrow terminal can
 * neither overflow nor wrap.
 */
static void draw_hud(const Tornado *t, double fps, int rows, int cols)
{
    char left[24], right[96];
    snprintf(left,  sizeof left,  " FIRE TORNADO ");
    snprintf(right, sizeof right,
             " embers:%d  spin:%.2fx  height:%.0f%%  wind:%s  theme:%d  %.0f fps  %s ",
             t->n, t->omega_mult, t->height_frac * 100.0f,
             (t->wind_amp > 0.001f) ? "on" : "off",
             t->theme, fps, t->paused ? "PAUSED" : "running");
    int rx = cols - (int)strlen(right);          /* right-aligned stats column */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int c = 0; c < cols; c++) mvaddch(0, c, ' ');
    if (rx >= 0) {
        mvprintw(0, 0,  "%.*s", rx, left);       /* title clipped clear of stats */
        mvprintw(0, rx, "%s", right);
    } else {
        mvprintw(0, 0,  "%.*s", cols, right);    /* too narrow: data only */
    }
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    for (int c = 0; c < cols; c++) mvaddch(rows - 1, c, ' ');
    mvprintw(rows - 1, 0, "%.*s", cols,
             " [/]:embers  -/+:spin  ,/.:height  w:wind  t:theme  r:reset  p:pause  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Tornado *t, double fps)
{
    erase();

    /* Layered back-to-front (painter's algorithm): ground flame, then the
       two ember depth passes, then sparks on top, then the HUD. */
    base_heat_draw(&t->base_heat, t->axis_x, t->base_y, rows, cols);
    for (int i = 0; i < t->n; i++)
        ember_draw(&t->embers[i],
                   t->axis_x, t->base_y, t->height,
                   t->world_time, t->wind_amp,
                   rows, cols, /*back_pass=*/1);
    for (int i = 0; i < t->n; i++)
        ember_draw(&t->embers[i],
                   t->axis_x, t->base_y, t->height,
                   t->world_time, t->wind_amp,
                   rows, cols, /*back_pass=*/0);
    for (int i = 0; i < N_SPARKS_MAX; i++)
        spark_draw(&t->sparks[i], rows, cols);
    draw_hud(t, fps, rows, cols);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §10  screen                                                             */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §11  app                                                                */
/* ═══════════════════════════════════════════════════════════════════════ *
 * Signals + the frame loop. PERFORMANCE: a fixed FRAME_NS cap, a DT_CAP_S
 * clamp on dt, and an exponential fps average. User events mutate state here
 * but are NOT part of the tick (the gated tornado_tick is the per-tick combine,
 * §8); 'r' and resize re-invoke INIT (§7).                                    */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

static Tornado g_tornado;

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    srand((unsigned)time(NULL));

    g_tornado.n           = N_EMBERS_DEFAULT;
    g_tornado.height_frac = HEIGHT_FRAC;
    g_tornado.omega_mult  = OMEGA_MULT_DEFAULT;
    g_tornado.wind_amp    = WIND_AMP_DEFAULT;
    g_tornado.theme       = 0;

    screen_init(g_tornado.theme);

    int rows = LINES, cols = COLS;
    tornado_position(&g_tornado, rows, cols);
    tornado_reseed(&g_tornado);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps         = TARGET_FPS;
    int64_t t_fps_prev  = clock_ns();
    int64_t t_tick_prev = t_fps_prev;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            tornado_position(&g_tornado, rows, cols);
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
                case 'q': case 27: g_running = 0; break;
                case 'p':          g_tornado.paused ^= 1; break;
                case 'r':          tornado_reseed(&g_tornado); break;
                case 't':          g_tornado.theme = (g_tornado.theme + 1) % N_THEMES;
                                   color_init(g_tornado.theme); break;
                case 'w':
                    g_tornado.wind_amp = (g_tornado.wind_amp > 0.001f)
                                       ? 0.0f : WIND_AMP_DEFAULT;
                    break;
                case '[':
                    if (g_tornado.n - N_EMBERS_STEP >= N_EMBERS_MIN)
                        g_tornado.n -= N_EMBERS_STEP;
                    break;
                case ']':
                    if (g_tornado.n + N_EMBERS_STEP <= N_EMBERS_MAX) {
                        int old_n = g_tornado.n;
                        g_tornado.n += N_EMBERS_STEP;
                        for (int i = old_n; i < g_tornado.n; i++)
                            ember_respawn(&g_tornado.embers[i],
                                          g_tornado.height, /*initial=*/1);
                    }
                    break;
                case '-':
                    if (g_tornado.omega_mult - OMEGA_MULT_STEP >= OMEGA_MULT_MIN)
                        g_tornado.omega_mult -= OMEGA_MULT_STEP;
                    break;
                case '+': case '=':
                    if (g_tornado.omega_mult + OMEGA_MULT_STEP <= OMEGA_MULT_MAX)
                        g_tornado.omega_mult += OMEGA_MULT_STEP;
                    break;
                case ',':
                    if (g_tornado.height_frac - HEIGHT_FRAC_STEP >= HEIGHT_FRAC_MIN) {
                        g_tornado.height_frac -= HEIGHT_FRAC_STEP;
                        tornado_position(&g_tornado, rows, cols);
                    }
                    break;
                case '.':
                    if (g_tornado.height_frac + HEIGHT_FRAC_STEP <= HEIGHT_FRAC_MAX) {
                        g_tornado.height_frac += HEIGHT_FRAC_STEP;
                        tornado_position(&g_tornado, rows, cols);
                    }
                    break;
            }
        }

        int64_t now = clock_ns();
        float   dt  = (float)(now - t_tick_prev) / 1e9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;
        t_tick_prev = now;
        if (!g_tornado.paused) tornado_tick(&g_tornado, dt, rows, cols);

        fps = fps * 0.95 + (1e9 / (double)(now - t_fps_prev + 1)) * 0.05;
        t_fps_prev = now;

        scene_draw(rows, cols, &g_tornado, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
