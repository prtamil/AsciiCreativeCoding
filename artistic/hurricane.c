/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * hurricane.c — top-down hurricane / cyclone with calm eye
 *
 * DEMO: Bird's-eye view of a tropical cyclone. A pool of cloud particles
 *       orbits a central eye following the Rankine vortex velocity
 *       profile — solid-body rotation inside the eyewall, decaying
 *       1/r outside. Each particle slowly spirals inward (radial
 *       inflow) and is recycled when it reaches the eye boundary,
 *       respawning at a random outer radius. Three concentric "bands"
 *       of different colour intensity give a satellite-view texture:
 *       outer rim dim grey, middle band brighter cloud, eyewall white.
 *       The eye itself is a calm dark spot at the centre. The screen
 *       reads as a still-frame of a hurricane satellite image, but
 *       animated.  n/p cycle 15 preset storms — different colour, spin,
 *       eye size, density, handedness, and spiral-vs-ring flow.
 *
 * Study alongside: artistic/fire_tornado.c (rotation, but side-view),
 *                  fluid/navier_stokes.c (real fluid dynamics),
 *                  geometry/voronoi.c (radial-distance ideas).
 *
 * Section map (cut by layer — see ARCHITECTURE):
 *   §1 config      — constants, colour-pair ids
 *   §2 performance — monotonic clock + sleep
 *   §3 data        — presets + palette tables + Cloud/Hurricane types
 *   §4 logic       — pure maps (frand, Rankine ω, radial zone, wind glyph)
 *   §5 simulation  — cloud respawn / tick + hurricane_tick (the per-tick advance)
 *   §6 render      — colour, cloud/eye draw, scene_draw + HUD
 *   §7 init/reset  — geometry, reseed, apply_preset
 *   §8 events      — signals, screen setup
 *   §9 app         — the frame loop (the per-tick combine)
 *
 * Keys:  n/p   next / previous preset (15 storms)
 *        [/]   particle count        -/+  spin speed
 *        ,/.   eye radius            i    toggle inflow
 *        t     cycle theme   r reseed   space pause   q/ESC quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/hurricane.c \
 *       -o hurricane -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Each cloud particle is stored in polar coordinates
 *                 around the screen centre: (radius, theta, age). Each
 *                 frame: theta advances by ω(r)·dt where the angular
 *                 velocity ω comes from the Rankine vortex profile —
 *                 solid-body rotation `ω = ω_max · r/R_eye` inside the
 *                 eyewall, and `ω = ω_max · R_eye/r` outside (so v(r)
 *                 = ω_max·R_eye is constant in the eyewall, then
 *                 decays as 1/r). Radial inflow pulls particles
 *                 inward at a small constant rate; particles that
 *                 cross the eye are recycled to the outer radius.
 *
 *                 Every cloud renders as a wind-aligned SLOPE glyph
 *                 (- \ | /, from atan2 of its velocity) so the swirl
 *                 direction reads even on a still frame. COLOUR comes
 *                 from the radial zone instead:
 *                   inside eye   → left empty (a clean calm centre)
 *                   eyewall      → bright white, bold (high winds)
 *                   middle band  → mid-tone cloud
 *                   outer rim    → dimmer grey
 *
 * Data-structure: Hurricane holds Cloud[N_CLOUDS_MAX] inline. Each Cloud
 *                 carries (radius, theta, alive). No allocation after init.
 *
 * Rendering     : Per frame: erase, paint each cloud at its (radius,
 *                 theta) projection — with cell-aspect correction so the
 *                 hurricane reads as round, not vertically squashed.
 *
 * Performance   : O(N_CLOUDS) per frame. 600 particles is fine.
 *
 * References    :
 *   Vortex & hurricane physics (the Rankine profile + storm structure):
 *     Rankine, "A Manual of Applied Mechanics" (1858) §625 — the original
 *       two-zone vortex (solid-body core + 1/r free vortex) this sim uses.
 *     Lamb, "Hydrodynamics" (6th ed. 1932) — the classic vortex-motion
 *       reference; the Rankine combined vortex is its textbook case.
 *     Holland, "An Analytic Model of the Wind and Pressure Profiles in
 *       Hurricanes" (Monthly Weather Review 108, 1980) — the modern wind
 *       profile that Rankine simplifies (rankine_omega).
 *     Emanuel, "Divine Wind: The History and Science of Hurricanes" (2005)
 *       — accessible account of the eye / eyewall / spiral bands / radial
 *       inflow that the radial zones and glyphs here reproduce.
 *
 *   Particle rendering (cloud pool, polar projection, glyph ramp):
 *     Reeves, "Particle Systems — A Technique for Modeling a Class of Fuzzy
 *       Objects" (ACM TOG 1983) — the spawn / advect / recycle cloud pool (§5).
 *     Bourke, "Character representation of greyscale images" — the radius →
 *       intensity → glyph mapping that textures the bands.
 *     Foley, van Dam, Feiner & Hughes, "Computer Graphics: Principles and
 *       Practice" — polar→screen projection with cell-aspect correction.
 *
 * ─────────────────────────────────────────────────────────────────────── */


/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * The file is cut into LAYERS by concern. All state lives on one Hurricane
 * aggregate (§3); each layer reads and/or mutates a named slice of it. Functions
 * take the NARROWEST type they need — a Cloud* plus a const RankineVortex* for
 * one particle, Hurricane* only for the whole-storm orchestrators — so the
 * layers never re-couple. The split is by SECTION; this table lists what each
 * mutates.
 *
 *   Layer        Section            Mutates
 *   ─────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2 performance     nothing (reads OS clock, sleeps)
 *   DATA         §3 data            — types, tables + the Hurricane instance —
 *   LOGIC        §4 logic           nothing (pure maps; frand advances rand())
 *   SIMULATION   §5 simulation      h.clouds[] (r, theta, alive)
 *   RENDER       §6 render          the screen + the colour pairs; never Hurricane
 *   INIT/RESET   §7 init            ALL of Hurricane (geometry / reseed / preset)
 *   EVENTS       §8 events          g_running, g_need_resize (+ keys mutate h in main)
 *   —            §9 app             the per-frame driver (combines layers)
 *
 * No EFFECTS layer: the storm IS the cloud particle pool (SIMULATION). Each
 * cloud's GLYPH and colour ZONE are derived at render time from its radius and
 * velocity (radial_zone / wind_glyph) — not stored — so there is no cosmetic-
 * only state to advance separately.
 *
 * No DELAYS layer: pause is a single flag (h.paused) tested once in main before
 * hurricane_tick; the fps window is a PERFORMANCE counter in main. No holds.
 *
 * LOGIC (frand, rankine_omega, radial_zone, wind_glyph) does no I/O and (apart
 * from frand's PRNG) no mutation — it maps inputs to a value — so reordering or
 * deleting RENDER cannot change a LOGIC result.
 *
 * PER-TICK COMBINE — main's loop (§9) advances state in ONE place:
 *   hurricane_tick(dt) — advance every cloud (theta via Rankine ω, r via inflow,
 *                        recycle across the eye/rim)            (SIMULATION)
 *   then every frame:  scene_draw(...) — clouds + eye + HUD     (RENDER)
 *
 * User events (quit, pause, next/prev preset, reseed, theme, inflow, clouds,
 * spin, eye, resize) DO mutate state but are NOT part of the tick — they run in
 * main's input/resize handling, before the gated hurricane_tick. Preset change
 * (n/p) and reseed (r) re-invoke INIT (§7); resize re-runs the geometry (§7).
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

#define TARGET_FPS         60

#define N_CLOUDS_MAX       800
#define N_CLOUDS_MIN       100
#define N_CLOUDS_STEP      100

/* Vortex profile. Radii are in cells. Startup values come from PRESETS[0];
 * these MIN/MAX/STEP bound the live tweak keys ([/], -/+, ,/.). */
#define EYE_RADIUS_MIN        2.0f
#define EYE_RADIUS_MAX        8.0f
#define EYE_RADIUS_STEP       1.0f
#define OUTER_RADIUS_FRAC     0.85f   /* fraction of min(rows, cols/2)    */

#define OMEGA_MAX_MIN         0.5f
#define OMEGA_MAX_MAX         4.0f
#define OMEGA_MAX_STEP        0.25f

/* Cell aspect — terminal cells ~2× tall as wide. Stretch r·cos(θ) so
 * the hurricane is visually round on screen. */
#define ASPECT_X              2.0f

#define DT_CAP_S              0.10f
#define N_THEMES              4

/* fps display smoothing — exponential moving-average weight on each new frame */
#define FPS_EMA_ALPHA       0.05

/* small-radius guard in rankine_omega (avoid 1/r blow-up at the exact centre) */
#define VORTEX_CENTRE_EPS   1e-3f

/* Radial band boundaries (× r_eye or × r_outer) — see radial_zone */
#define ZONE_EYE_FRAC       0.70f  /* r < r_eye·this   → inside the calm eye     */
#define ZONE_EYEWALL_FRAC   1.15f  /* r < r_eye·this   → the eyewall (peak wind) */
#define ZONE_MIDBAND_FRAC   0.60f  /* r < r_outer·this → mid cloud band          */

/* Cloud spawn + recycle bounds (× r_outer or × r_eye) */
#define SPAWN_R2_MIN        0.40f  /* spawn radius² in [MIN, MIN+SPAN]; sqrt →   */
#define SPAWN_R2_SPAN       0.60f  /*   area-uniform, biased toward the rim      */
#define RECYCLE_INNER_FRAC  0.50f  /* r < r_eye·this   → sucked into eye, recycle */
#define RECYCLE_OUTER_FRAC  1.05f  /* r > r_outer·this → past the rim,   recycle */

/* Colour pairs */
#define PAIR_OUTER    1
#define PAIR_BAND     2
#define PAIR_EYEWALL  3
#define PAIR_EYE      4
#define PAIR_HUD      5
#define PAIR_HINT     6

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  PERFORMANCE — clock                                                 */
/* ═══════════════════════════════════════════════════════════════════════ *
 * Monotonic clock + sleep. Mutates nothing. The frame cap, dt clamp and fps
 * window that use these live in main's loop (§9).                            */

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
/* §3  DATA — presets, palette tables, Cloud / RankineVortex / Hurricane     */
/* ═══════════════════════════════════════════════════════════════════════ *
 * Declarations only; no behaviour. Mutated by SIMULATION (§5) / INIT (§7) /
 * EVENTS (§8) and read by LOGIC (§4) / RENDER (§6).                          */

/* ─── Presets ─────────────────────────────────────────────────────────── *
 * HurricanePreset — one named storm as a parameter RECIPE. n/p cycle through
 * them and hurricane_apply_preset (§7) copies a preset onto the live state: its
 * physics fields seed the RankineVortex, plus the cloud count and palette. A
 * preset is deliberately FLAT and screen-independent — it carries no r_outer
 * (that is derived from the terminal size at runtime), so the same recipe reads
 * correctly at any window size. Variety comes from the colour theme, eyewall
 * spin, eye size, cloud density, whether the flow SPIRALS in (inflow=1) or
 * orbits in concentric rings (inflow=0), the spin HANDEDNESS (spin_dir +1 =
 * counter-clockwise like a northern-hemisphere cyclone, -1 = clockwise like a
 * southern one), and how tightly it winds in (inflow_rate). */
typedef struct {
    const char *name;   /* label shown in the HUD                           */
    int   theme;        /* palette index 0..N_THEMES-1                      */
    float omega_max;    /* eyewall spin (rad/sec)                           */
    float r_eye;        /* eye radius (cells)                               */
    int   n;            /* cloud count                                      */
    int   inflow;       /* 1 = spiral inward, 0 = concentric orbit rings    */
    float spin_dir;     /* +1 = counter-clockwise, -1 = clockwise           */
    float inflow_rate;  /* radius units/sec inward (spiral tightness)       */
} HurricanePreset;

#define N_PRESETS 15
static const HurricanePreset PRESETS[N_PRESETS] = {
    /*  name              thm  omega   eye    n  inflow spin   rate */
    { "Eye of Calm",       0,  0.9f,  8.0f, 350,  0,   +1.f,  0.5f },
    { "Category Five",     0,  3.6f,  2.0f, 750,  1,   +1.f,  1.0f },
    { "Blue Whirl",        1,  1.8f,  4.0f, 500,  1,   +1.f,  0.7f },
    { "Pinwheel",          1,  3.2f,  5.0f, 600,  0,   -1.f,  0.5f },
    { "Golden Fury",       2,  3.0f,  3.0f, 680,  1,   +1.f,  0.9f },
    { "Solar Flare",       2,  4.0f,  2.0f, 800,  1,   -1.f,  1.2f },
    { "Rose Cyclone",      3,  1.6f,  5.0f, 520,  1,   +1.f,  0.6f },
    { "Crimson Vortex",    3,  3.4f,  3.0f, 700,  1,   -1.f,  1.0f },
    { "Wisps",             0,  2.2f,  4.0f, 180,  1,   +1.f,  0.8f },
    { "Concentric",        0,  1.2f,  6.0f, 550,  0,   +1.f,  0.5f },
    { "Maelstrom",         1,  3.8f,  2.0f, 760,  1,   +1.f,  1.3f },
    { "Slow Drift",        2,  0.7f,  7.0f, 300,  1,   +1.f,  0.3f },
    { "Twin Bands",        3,  2.0f,  5.0f, 640,  0,   -1.f,  0.5f },
    { "Tempest",           1,  2.8f,  3.0f, 680,  1,   -1.f,  0.9f },
    { "Supercell",         0,  4.0f,  2.0f, 800,  1,   +1.f,  1.4f },
};

/* 4-zone palette per theme: outer / band / eyewall / eye.
 *
 * Brightness gradient is encoded in the colour values themselves
 * (outer = mid-bright, eyewall = white) plus A_BOLD on the eyewall —
 * we no longer apply A_DIM to the outer band, since on most terminals
 * dim × medium-saturation = invisible.
 *
 * Eye colour is a fixed visible gray (240) regardless of theme — the
 * eye is a static "calm centre" feature, not part of the storm hue. */
static const short PAL_256[N_THEMES][4] = {
    /* 0 satellite white — light gray → bright white                   */
    { 250, 254, 231, 240 },
    /* 1 blue storm      — sky blue → cyan → white                     */
    {  39,  45,  51, 240 },
    /* 2 gold storm      — orange → gold → white                       */
    { 178, 214, 231, 240 },
    /* 3 magenta storm   — pink → light pink → white                   */
    { 175, 213, 231, 240 },
};
static const short PAL_8[N_THEMES][4] = {
    { COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE },
    { COLOR_BLUE,    COLOR_CYAN,    COLOR_WHITE,   COLOR_WHITE },
    { COLOR_YELLOW,  COLOR_YELLOW,  COLOR_WHITE,   COLOR_WHITE },
    { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE,   COLOR_WHITE },
};

/* Cloud — one advected particle in the storm's cloud pool. Stored in POLAR
 * coordinates about the eye centre because the vortex velocity field is itself
 * polar: a tick advances theta by ω(r)·dt and pulls r inward by the inflow —
 * two cheap adds — and the (r, theta) → screen projection is deferred to render
 * time (cloud_draw). The pool is fixed-size and recycled, never reallocated
 * (Reeves, "Particle Systems", ACM TOG 1983): a particle that crosses the eye
 * or the outer rim has `alive` cleared and is re-seeded into the slot next tick. */
typedef struct {
    float r;          /* radius from eye centre (cells)                     */
    float theta;      /* angle about the eye (radians); ω(r)·dt per tick    */
    int   alive;      /* 0 = spent slot — cloud_respawn re-seeds it          */
} Cloud;

/* RankineVortex — the parameterised velocity field the clouds move in. The
 * tangential wind follows the Rankine profile (solid-body core spinning up to
 * the eyewall at r_eye, then 1/r decay outside); a weak radial INFLOW spirals
 * clouds in. r_eye / omega_max / spin_dir / inflow / inflow_rate come from the
 * active preset; r_outer is the screen-fit outer extent where clouds recycle.
 *
 * The two-zone tangential profile is Rankine's combined vortex (Rankine,
 * "A Manual of Applied Mechanics", 1858, §625; Lamb, "Hydrodynamics", 1932):
 *     ω(r) = ω_max · r / r_eye    for r ≤ r_eye   (solid-body core)
 *     ω(r) = ω_max · r_eye / r    for r >  r_eye   (1/r free vortex)
 * so the tangential wind v = ω·r peaks exactly at r_eye — the eyewall, where a
 * real cyclone's strongest winds sit. See rankine_omega (§4) for the code. */
typedef struct {
    float r_eye;        /* eye / eyewall radius (cells) — peak-wind cusp      */
    float r_outer;      /* outer extent (cells) — recycle + outer-rim bound   */
    float omega_max;    /* angular velocity at the eyewall (rad/sec)          */
    float spin_dir;     /* +1 = counter-clockwise, -1 = clockwise            */
    int   inflow;       /* 1 = spiral inward, 0 = orbit at fixed radius       */
    float inflow_rate;  /* radial inflow speed (cells/sec) — spiral tightness */
} RankineVortex;

/* Hurricane — the whole runtime scene, read like a table of contents. Functions
 * take the NARROWEST slice they need (Cloud* / const Cloud* for one particle,
 * const RankineVortex* to read the field); only the whole-storm orchestrators
 * (hurricane_geometry / reseed / apply_preset / tick) take Hurricane*.
 *   WHAT  — the cloud particle pool: clouds[0..n).
 *   HOW   — vortex: the Rankine velocity field that pool moves in.
 *   WHERE — preset_idx / theme (current selection) + paused (run state). */
typedef struct {
    /* WHAT — the cloud particle pool */
    Cloud clouds[N_CLOUDS_MAX];
    int   n;                 /* active clouds [0..n)                        */
    /* HOW — the velocity field they move in */
    RankineVortex vortex;
    /* WHERE — selection + run state */
    int   preset_idx;        /* current PRESETS[] index (n/p cycle it)      */
    int   theme;             /* current palette index                       */
    int   paused;            /* 1 freezes the per-tick advance              */
} Hurricane;

static Hurricane g_h;

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  LOGIC — pure maps (no mutation, no I/O)                             */
/* ═══════════════════════════════════════════════════════════════════════ *
 * Each maps its inputs to a value; only frand touches global state (it advances
 * the libc PRNG). Reordering or deleting RENDER cannot change a result here.   */

static float frand(void) { return (float)rand() / (float)RAND_MAX; }

/*
 * rankine_omega — Rankine vortex angular velocity at radius r.
 *   Inside  eye/eyewall (r ≤ R_eye):  ω = ω_max · r / R_eye   (solid body)
 *   Outside              (r >  R_eye): ω = ω_max · R_eye / r  (free vortex)
 * The cusp at r = R_eye is where the maximum tangential wind v = ω_max·R_eye
 * occurs — physically the eyewall.
 */
static float rankine_omega(float r, float r_eye, float omega_max)
{
    if (r < VORTEX_CENTRE_EPS) return omega_max * VORTEX_CENTRE_EPS / r_eye;
    if (r <= r_eye) return omega_max * r / r_eye;
    return omega_max * r_eye / r;
}

/* keep an angle within (-2π, 2π) so a cloud's theta never grows without bound. */
static float wrapped_angle(float a)
{
    if (a >  2.0f * (float)M_PI) a -= 2.0f * (float)M_PI;
    if (a < -2.0f * (float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/*
 * radial_zone — pick a band index (0..3) by radius.
 *   0 = outer rim, 1 = mid band, 2 = eyewall, 3 = inside eye.
 */
static int radial_zone(float r, float r_eye, float r_outer)
{
    if (r < r_eye * ZONE_EYE_FRAC)        return 3;   /* inside eye         */
    if (r < r_eye * ZONE_EYEWALL_FRAC)    return 2;   /* eyewall            */
    if (r < r_outer * ZONE_MIDBAND_FRAC)  return 1;   /* mid band           */
    return 0;                                         /* outer rim          */
}

/* a cloud has left the active annulus if the inflow pulled it inside the eye or
 * it drifted past the outer rim — the caller recycles it. */
static bool cloud_escaped(const Cloud *c, const RankineVortex *v)
{
    return c->r < v->r_eye   * RECYCLE_INNER_FRAC
        || c->r > v->r_outer * RECYCLE_OUTER_FRAC;
}

/*
 * wind_glyph — directional character for a particle moving at angle
 * `theta_vel` (atan2 of (vx, vy)). 8 sectors map to 8 glyphs that
 * trace the swirl direction even on a still frame.
 */
static char wind_glyph(float vx, float vy)
{
    static const char G[8] = { '-', '\\', '|', '/', '-', '\\', '|', '/' };
    float ang = atan2f(vy, vx);                  /* -π..π */
    if (ang < 0) ang += 2.0f * (float)M_PI;
    int sect = (int)(ang / (2.0f * (float)M_PI / 8.0f));
    if (sect < 0) sect = 0;
    if (sect > 7) sect = 7;
    return G[sect];
}

/* glyph tracing a cloud's LOCAL wind direction. Tangential velocity v = ω × r
 * (perpendicular to the radius), aspect-stretched on x to match the screen. */
static char cloud_wind_glyph(const Cloud *c, const RankineVortex *v)
{
    float omega = rankine_omega(c->r, v->r_eye, v->omega_max);
    float vx = -sinf(c->theta) * omega * c->r * ASPECT_X;
    float vy =  cosf(c->theta) * omega * c->r;
    return wind_glyph(vx, vy);
}

/* project a cloud's polar position to a screen cell about the eye centre
 * (cx, cy); the ASPECT_X stretch on x makes the round storm read round. */
static void cloud_to_cell(const Cloud *c, int cx, int cy, int *sr, int *sc)
{
    *sc = (int)((float)cx + ASPECT_X * c->r * cosf(c->theta));
    *sr = (int)((float)cy + c->r * sinf(c->theta));
}

/* true if a cell is off the canvas — past an edge, or in the bottom row that
 * the HUD action bar reserves. */
static bool off_screen(int sr, int sc, int rows, int cols)
{
    return sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols;
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  SIMULATION — cloud advance + the per-tick combine                   */
/* ═══════════════════════════════════════════════════════════════════════ *
 * Advances h.clouds[] (radius, theta, alive). cloud_respawn re-seeds one slot
 * (reused by INIT §7); hurricane_tick is the SINGLE per-tick advance.         */

static void cloud_respawn(Cloud *c, float r_outer)
{
    /* Bias toward larger radii (uniform in r²): sqrt of an area-uniform sample. */
    c->r     = r_outer * sqrtf(SPAWN_R2_MIN + SPAWN_R2_SPAN * frand());
    c->theta = frand() * 2.0f * (float)M_PI;
    c->alive = 1;
}

/*
 * cloud_tick — advance theta via Rankine omega; pull r inward by the
 * inflow rate. Recycle when the cloud crosses the eye into nothingness.
 */
static void cloud_tick(Cloud *c, const RankineVortex *v, float dt)
{
    if (!c->alive) {
        cloud_respawn(c, v->r_outer);
        return;
    }
    c->theta += rankine_omega(c->r, v->r_eye, v->omega_max) * v->spin_dir * dt;
    if (v->inflow) c->r -= v->inflow_rate * dt;
    c->theta = wrapped_angle(c->theta);
    if (cloud_escaped(c, v)) cloud_respawn(c, v->r_outer);
}

static void hurricane_tick(Hurricane *h, float dt)
{
    for (int i = 0; i < h->n; i++)
        cloud_tick(&h->clouds[i], &h->vortex, dt);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  RENDER — state → screen (reads only, never mutates Hurricane)       */
/* ═══════════════════════════════════════════════════════════════════════ *
 * color_init loads the palette pairs; cloud_draw / draw_eye stamp cells;
 * scene_draw composes the frame + HUD. Glyph and colour zone are derived here
 * from each cloud's radius/velocity, not stored.                              */

static void color_init(int theme)
{
    start_color(); use_default_colors();
    int x256 = (COLORS >= 256);
    init_pair(PAIR_OUTER,   x256 ? PAL_256[theme][0] : PAL_8[theme][0], -1);
    init_pair(PAIR_BAND,    x256 ? PAL_256[theme][1] : PAL_8[theme][1], -1);
    init_pair(PAIR_EYEWALL, x256 ? PAL_256[theme][2] : PAL_8[theme][2], -1);
    init_pair(PAIR_EYE,     x256 ? PAL_256[theme][3] : PAL_8[theme][3], -1);
    init_pair(PAIR_HUD,     x256 ? 226 : COLOR_YELLOW, -1);  /* bright yellow — top data bar    */
    init_pair(PAIR_HINT,    x256 ?  51 : COLOR_CYAN,   -1);  /* bright cyan   — bottom action bar */
}

/* colour pair + emphasis for a radial band: the eyewall is bold, the bands step
 * down in brightness. (zone 3, the eye interior, is culled by the caller.) */
static chtype zone_attr(int zone)
{
    short pair  = (zone == 2) ? PAIR_EYEWALL
                : (zone == 1) ? PAIR_BAND
                :                PAIR_OUTER;
    chtype attr = COLOR_PAIR(pair);
    if (zone == 2) attr |= A_BOLD;          /* eyewall pops */
    return attr;
}

static void cloud_draw(const Cloud *c, int cx, int cy,
                       const RankineVortex *v,
                       int rows, int cols)
{
    if (!c->alive) return;

    int sr, sc;
    cloud_to_cell(c, cx, cy, &sr, &sc);
    if (off_screen(sr, sc, rows, cols)) return;

    int zone = radial_zone(c->r, v->r_eye, v->r_outer);
    if (zone == 3) return;          /* eye interior stays empty — see draw_eye */

    chtype attr = zone_attr(zone);
    char   ch   = cloud_wind_glyph(c, v);
    attron(attr);
    mvaddch(sr, sc, (chtype)(unsigned char)ch);
    attroff(attr);
}

/* Faint eye marker — three '.' at the very centre, dim. */
static void draw_eye(int cx, int cy, int rows, int cols)
{
    if (cy < 0 || cy >= rows - 1) return;
    attron(COLOR_PAIR(PAIR_EYE) | A_BOLD);
    if (cx >= 0 && cx < cols)            mvaddch(cy, cx,     (chtype)'.');
    if (cx - 2 >= 0 && cx - 2 < cols)    mvaddch(cy, cx - 2, (chtype)'.');
    if (cx + 2 >= 0 && cx + 2 < cols)    mvaddch(cy, cx + 2, (chtype)'.');
    attroff(COLOR_PAIR(PAIR_EYE) | A_BOLD);
}

/* draw_hud — the two standard bars: a top yellow DATA line (preset / spin / eye
 * / fps / run-state) and a bottom cyan ACTION legend. Each row is filled, then
 * clipped with "%.*s" so a narrow terminal can neither overflow nor wrap. */
static void draw_hud(const Hurricane *h, double fps, int rows, int cols)
{
    char left[24], right[96];
    snprintf(left,  sizeof left,  " HURRICANE ");
    snprintf(right, sizeof right,
             " preset %2d/%d: %-14s  spin:%.1f  eye:%.0f  %.0f fps  %s ",
             h->preset_idx + 1, N_PRESETS, PRESETS[h->preset_idx].name,
             h->vortex.omega_max, h->vortex.r_eye, fps,
             h->paused ? "PAUSED" : "running");
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
             " n/p:preset  [/]:clouds  -/+:spin  ,/.:eye  i:inflow  t:theme  r:reseed  spc:pause  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Hurricane *h, double fps)
{
    erase();
    int cx = cols / 2;
    int cy = (rows - 1) / 2;

    for (int i = 0; i < h->n; i++)
        cloud_draw(&h->clouds[i], cx, cy, &h->vortex, rows, cols);
    draw_eye(cx, cy, rows, cols);
    draw_hud(h, fps, rows, cols);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  INIT/RESET — geometry, reseed, preset load (NOT part of the tick)   */
/* ═══════════════════════════════════════════════════════════════════════ *
 * Mutate Hurricane state wholesale. Called at startup, on resize (geometry),
 * and on reseed / preset-change keys — all outside hurricane_tick.            */

static void hurricane_geometry(Hurricane *h, int rows, int cols)
{
    /* Outer radius is the largest circle that fits inside the screen,
     * accounting for cell-aspect: horizontal extent scales by ASPECT_X. */
    float max_h = (float)(rows - 2) * 0.5f;
    float max_w = (float)cols * 0.5f / ASPECT_X;
    float bound = (max_h < max_w) ? max_h : max_w;
    h->vortex.r_outer = bound * OUTER_RADIUS_FRAC;
}

static void hurricane_reseed(Hurricane *h)
{
    for (int i = 0; i < h->n; i++) cloud_respawn(&h->clouds[i], h->vortex.r_outer);
}

/* Load preset `idx`: set the storm parameters, apply its palette, and reseed
 * the cloud pool so the new look starts fresh. Geometry (r_outer) must already
 * be set, since reseed scatters clouds out to the outer radius. */
static void hurricane_apply_preset(Hurricane *h, int idx)
{
    const HurricanePreset *p = &PRESETS[idx];
    h->preset_idx         = idx;
    h->theme              = p->theme;
    h->n                  = p->n;
    h->vortex.omega_max   = p->omega_max;
    h->vortex.r_eye       = p->r_eye;
    h->vortex.inflow      = p->inflow;
    h->vortex.spin_dir    = p->spin_dir;
    h->vortex.inflow_rate = p->inflow_rate;
    color_init(p->theme);
    hurricane_reseed(h);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  EVENTS — signals & screen setup (mutate state, NOT part of the tick) */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

/* apply one keypress to the storm — cycle presets, tweak vortex knobs, pause,
 * reseed, or quit. An EVENT, run before the tick and never part of it. */
static void hurricane_handle_key(Hurricane *h, int ch)
{
    switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case ' ':          h->paused ^= 1; break;
        case 'n': case KEY_RIGHT:
            h->preset_idx = (h->preset_idx + 1) % N_PRESETS;
            hurricane_apply_preset(h, h->preset_idx);
            break;
        case 'p': case KEY_LEFT:
            h->preset_idx = (h->preset_idx + N_PRESETS - 1) % N_PRESETS;
            hurricane_apply_preset(h, h->preset_idx);
            break;
        case 'r':          hurricane_reseed(h); break;
        case 't':          h->theme = (h->theme + 1) % N_THEMES;
                           color_init(h->theme); break;
        case 'i':          h->vortex.inflow ^= 1; break;
        case '[':
            if (h->n - N_CLOUDS_STEP >= N_CLOUDS_MIN)
                h->n -= N_CLOUDS_STEP;
            break;
        case ']':
            if (h->n + N_CLOUDS_STEP <= N_CLOUDS_MAX) {
                int old = h->n;
                h->n += N_CLOUDS_STEP;
                for (int i = old; i < h->n; i++)
                    cloud_respawn(&h->clouds[i], h->vortex.r_outer);
            }
            break;
        case '-':
            if (h->vortex.omega_max - OMEGA_MAX_STEP >= OMEGA_MAX_MIN)
                h->vortex.omega_max -= OMEGA_MAX_STEP;
            break;
        case '+': case '=':
            if (h->vortex.omega_max + OMEGA_MAX_STEP <= OMEGA_MAX_MAX)
                h->vortex.omega_max += OMEGA_MAX_STEP;
            break;
        case ',':
            if (h->vortex.r_eye - EYE_RADIUS_STEP >= EYE_RADIUS_MIN)
                h->vortex.r_eye -= EYE_RADIUS_STEP;
            break;
        case '.':
            if (h->vortex.r_eye + EYE_RADIUS_STEP <= EYE_RADIUS_MAX)
                h->vortex.r_eye += EYE_RADIUS_STEP;
            break;
    }
}

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
/* §9  app — the frame loop (the per-tick combine)                         */
/* ═══════════════════════════════════════════════════════════════════════ */

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    srand((unsigned)time(NULL));

    screen_init(PRESETS[0].theme);
    int rows = LINES, cols = COLS;
    hurricane_geometry(&g_h, rows, cols);     /* sets r_outer, needed by reseed */
    hurricane_apply_preset(&g_h, 0);          /* params + palette + reseed      */

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps         = TARGET_FPS;
    int64_t t_fps_prev  = clock_ns();
    int64_t t_tick_prev = t_fps_prev;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            hurricane_geometry(&g_h, rows, cols);
        }

        int ch;
        while ((ch = getch()) != ERR) hurricane_handle_key(&g_h, ch);

        int64_t now = clock_ns();
        float   dt  = (float)(now - t_tick_prev) / 1e9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;
        t_tick_prev = now;
        if (!g_h.paused) hurricane_tick(&g_h, dt);

        /* fps = exponential moving average of the instantaneous rate */
        double inst_fps = 1e9 / (double)(now - t_fps_prev + 1);
        fps = fps * (1.0 - FPS_EMA_ALPHA) + inst_fps * FPS_EMA_ALPHA;
        t_fps_prev = now;

        scene_draw(rows, cols, &g_h, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
