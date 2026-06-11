/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * phoenix.c — perched phoenix that self-immolates and is reborn from ash
 *
 * DEMO: A still owl-like phoenix sits on a perch, breathing softly.
 *       After a long calm, it ignites — sparks crawl across the body,
 *       feathers turn to flame, then the whole bird becomes a roaring
 *       conflagration.  The silhouette dissolves; embers fall and
 *       smoulder on the perch as ash and smoke drift upward.  After
 *       a long quiet, a single bright spark appears at the head of
 *       the perch and the bird grows back outward from that seed
 *       until the owl is whole again, eyes glowing, and the cycle
 *       repeats.
 *
 *       Lifecycle (one full cycle ≈ 26 seconds):
 *         PERCH    (12s)  still owl, gentle breathing
 *         IGNITE    (2s)  bird heats; sparks crawl; flame buds form
 *         BLAZE     (3s)  full conflagration; bright core
 *         COLLAPSE  (2s)  silhouette dissolves; embers fall as ash
 *         ASH       (4s)  smouldering remains drift; perch glows red
 *         REBIRTH   (3s)  spark at head expands outward; bird reforms
 *
 * Study alongside: artistic/fire_tornado.c (heat ramp + buoyancy),
 *                  artistic/volcano.c (free-flight cooling embers),
 *                  artistic/ant_colony.c (anchor-template particle
 *                  silhouette pattern from a different domain).
 *
 * Section map (layers):
 *   §1  config       — sizes, phase timings, fire physics, palette IDs
 *   §2  performance  — monotonic clock + sleep (frame cap in §7)
 *   §3  types        — Anchor / BodyParticle / Spark / Phoenix / Scene
 *   §4  logic        — pure: heat ramp + phase tables (no mutation, no I/O)
 *   §5  simulation   — PRNG, anchors, fire physics, lifecycle FSM, ticks
 *   §6  render        — heat colours, body/spark/perch draw, HUD, ncurses
 *   §7  app          — signals, the per-tick combine loop, key handling
 *
 * Keys:
 *   q / ESC      quit
 *   space / p    pause / resume
 *   r            reset (immediate PERCH)
 *   s            skip to next phase
 *   i            ignite NOW (skip to IGNITE)
 *   t / T        next / previous theme
 *   ,/.          decrease / increase body density
 *   ;/'          decrease / increase spark density
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/phoenix.c \
 *       -o phoenix -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Two cooperating particle systems, one anchor template,
 *                 one finite-state lifecycle.
 *
 *                 The BODY is a fixed array of anchor points in body-local
 *                 coordinates that together describe an owl silhouette
 *                 — head ellipse, body ellipse, eye discs, beak, ear
 *                 tufts.  Each frame, body particles either re-bind to a
 *                 weighted-random anchor (BOUND) or integrate free-flight
 *                 fire physics (FREE).  Whether a particle is BOUND or
 *                 FREE is dictated by the lifecycle phase: PERCH/IGNITE/
 *                 BLAZE force BOUND; ASH forces FREE; COLLAPSE flips
 *                 particles BOUND→FREE over time; REBIRTH flips them
 *                 FREE→BOUND.
 *
 *                 During REBIRTH, only anchors whose `alive_at` value is
 *                 below the growth fraction can be re-bound to.  alive_at
 *                 is the normalised distance from a SEED point (the head
 *                 centre).  As growth_frac rises 0→1, the alive set
 *                 expands outward from the seed, so the bird grows back
 *                 visually from the eyes outward.
 *
 *                 Spark particles are a separate, smaller, pure-free-
 *                 flight pool.  They're emitted from random body anchors
 *                 during IGNITE/BLAZE/COLLAPSE — the wisps that rise
 *                 above the burning silhouette and trail away as smoke.
 *
 *                 Both populations integrate the same fire physics:
 *                   – buoyancy   : vy -= BUOYANCY · (0.2 + 0.8·temp)·dt
 *                   – turbulence : v  += SHEAR · sin/cos field · dt
 *                   – drag       : v  *= 1 − DRAG · dt
 *                   – cooling    : T  *= exp(-COOL · dt)
 *
 *                 Heat ramp: temp ∈ [0,1] mapped to 6 colour buckets
 *                 from smoke (bucket 0, dim grey) through dark theme
 *                 tint, mid theme, bright theme, hot accent, white-hot
 *                 core (bucket 5).  At rest the owl's anchors carry low
 *                 base_temp values (~0.2) so the silhouette renders in
 *                 buckets 1-2 (dark feather colours).  Ignition raises
 *                 temps; blaze pegs them at ~1.0; rebirth eases them
 *                 back down.
 *
 * Data-structure: Phoenix holds inline pools — BodyParticle[N_BODY] and
 *                 Spark[N_SPARK] — plus an Anchor[] array recomputed at
 *                 init (the owl is static, so anchors don't change per
 *                 frame; only the per-particle position/state does).
 *
 * Rendering     : ASCII only.  Heat-ramp glyph set ` . + * # @ %` from
 *                 coolest to hottest.  Bucket 0 uses A_DIM, bucket ≥4
 *                 uses A_BOLD.  Perch is drawn as a static `-` strip
 *                 with a dim brown colour pair below the bird.
 *
 * Performance   : O(N_body + N_spark) per frame.  At defaults
 *                 (350 body + 200 spark) ≈ 550 mvaddch — trivial well
 *                 past 60 fps.
 *
 * References    :
 *   Particle systems & fire physics (§5 fire_step, sparks)
 *     [1] Reeves, "Particle Systems — A Technique for Modeling a Class of
 *         Fuzzy Objects," SIGGRAPH (1983) — the emit / integrate / cool /
 *         recycle loop used by sparks and free body particles.
 *     [2] Nguyen, Fedkiw & Jensen, "Physically Based Modeling and Animation
 *         of Fire," SIGGRAPH (2002) — the buoyant, cooling, blackbody-coloured
 *         flame this fakes cheaply with a per-particle temperature.
 *     [3] Stam, "Real-Time Fluid Dynamics for Games," GDC (2003) — the sin/cos
 *         shear field is a cheap stand-in for real velocity advection.
 *     [4] Boussinesq buoyancy approximation (any combustion text): the hot/cold
 *         density gradient drives the rise (vy -= BUOYANCY · temp · dt).
 *   Heat → colour & ASCII rendering (§4 heat_attrs, §6 draw)
 *     [5] Bourke, "Character representation of grey scale images" (1997) — the
 *         temperature → glyph ramp ('.' '+' '*' '#' '@' '%').
 *     [6] Padala, "NCURSES Programming HOWTO" (TLDP) — colour pairs and the
 *         erase → draw → refresh frame model.
 *   Myth
 *     [7] Phoenix mythology — the bird that ignites itself, burns to ash and
 *         regrows from the embers: Bulfinch's Mythology (1855), the Egyptian
 *         "Bennu" (Book of the Dead), the Persian "Simurgh".
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The owl is a SHAPE, not a thing.  The shape is a list of anchor
 * points; the thing is a swarm of particles that snaps to the shape
 * each frame.  Lifecycle phases change the rules of snapping: snap
 * always (PERCH), snap with heat (IGNITE), snap at white-hot (BLAZE),
 * stop snapping piece by piece (COLLAPSE), don't snap at all and float
 * away (ASH), then snap back from the inside out (REBIRTH).  At the
 * core, every visible character is just a particle deciding "where am
 * I and how hot am I" — the lifecycle decides which question to ask.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a flock of fireflies trained to fly to a pose.  At rest they
 * land in formation tracing an owl.  Heat them up: they glow brighter.
 * Heat them more: the formation breaks; some leave the pose and rise
 * as embers.  Wait long enough, and the heat fades out.  Now drop one
 * bright firefly back where the owl's eye should be, and call the
 * others home — they return one by one, nearest pose-points first,
 * and the silhouette grows back from a single point.  That's the
 * phoenix.
 *
 * DRAWING METHOD  (per frame, FSM-driven)
 * ──────────────
 *  1. erase(); draw the perch (static).
 *  2. Advance world_t and phase_t.  If phase_t exceeds the phase's
 *     duration, advance to the next phase (wraps PERCH → … → REBIRTH
 *     → PERCH).
 *  3. For every BODY PARTICLE, decide BOUND vs FREE for this frame:
 *       PERCH/IGNITE/BLAZE   → force BOUND
 *       COLLAPSE             → with prob frac·release_rate·dt, become
 *                              FREE for the rest of the cycle (until
 *                              REBIRTH or reset)
 *       ASH                  → force FREE
 *       REBIRTH              → with prob capture_rate·dt, try to
 *                              rebind to an anchor whose alive_at <=
 *                              growth_frac; if successful, BOUND.
 *  4. For each BOUND particle: re-pick a weighted-random anchor (from
 *     the alive set during REBIRTH; from the full set otherwise).
 *     Snap position to anchor world coord + small radial jitter.  Set
 *     temp to a phase-modified version of anchor.base_temp.
 *  5. For each FREE particle: integrate buoyancy + turbulence + drag
 *     + cooling.  If temp drops below FLOOR or particle drifts off the
 *     screen, mark inactive.
 *  6. SPARK pool: during IGNITE/BLAZE/COLLAPSE emit at phase-dependent
 *     rate from random anchors.  Each spark integrates the same fire
 *     physics, gets recycled when life or temp expires.
 *  7. Render: BODY (BOUND first, then FREE) → SPARKS → HUD.
 *
 * KEY FORMULAS
 * ────────────
 *  Heat-ramp bucket:
 *      b = clamp(floor(temp · 6), 0, 5)
 *      bucket  glyph  attr      colour role
 *      0       '.'    A_DIM     smoke
 *      1       '+'    -         dark theme tint   (owl feathers)
 *      2       '*'    -         mid theme tint    (warm feathers)
 *      3       '#'    -         bright theme tint (ember)
 *      4       '@'    A_BOLD    hot accent       (flame)
 *      5       '%'    A_BOLD    white-hot core
 *
 *  Anchor alive_at (radial wake order during REBIRTH):
 *      d_i      = sqrt((dx_i − seed_x)^2 + (dy_i − seed_y)^2)
 *      d_max    = max_i d_i
 *      alive_at = d_i / d_max                       ∈ [0,1]
 *      anchor i is alive when growth_frac >= alive_at
 *
 *  Phase temp modifier for BOUND particles:
 *      PERCH    : T = base_temp                         + jitter
 *      IGNITE   : T = lerp(base_temp, 1.0, frac)        + jitter
 *      BLAZE    : T = 0.85 + 0.15·frand_pos             (white-hot)
 *      COLLAPSE : T = lerp(1.0, 0.4, frac)
 *      REBIRTH  : T = lerp(hot, base_temp, frac)        (owl cools into shape)
 *
 *  Free-flight fire physics (same for body-FREE and sparks):
 *      vy   -= BUOYANCY · (0.20 + 0.80·T) · dt
 *      v    += SHEAR · (sin(t·hz + .3x + .45y),
 *                       cos(t·hz + .45x − .3y)) · dt
 *      v    *= 1 − DRAG · dt
 *      x,y  += v · dt
 *      T    *= exp(-COOL · dt)
 *      life -= dt    (sparks only; body-FREE has no life cap)
 *
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

/* ── ARCHITECTURE (layer separation) ───────────────────────────────────── *
 *
 * Four real layers; two are intentionally absent.
 *
 *   LAYER        SECTION  MUTATES
 *   ────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       nothing — clock primitives (frame cap / fps in §7)
 *   LOGIC        §4       nothing — pure: heat_attrs (temp → colour/glyph/attr)
 *                           and the phase tables (phase_name / phase_duration /
 *                           spark_hz_for_phase).  Read-only, no I/O.
 *   SIMULATION   §3,§5    §3 holds the state; §5 advances it: every BodyParticle
 *                           and Spark (pos/vel/temp/state), the Phoenix FSM
 *                           (phase, phase_t, world_t, spark_emit_acc) and Scene;
 *                           builds the Anchor template; advances the global PRNG.
 *   RENDER       §6       the terminal + colour pairs only; READS the Scene,
 *                           never writes it.
 *
 *   EFFECTS  : ABSENT — a particle's glow / colour is derived at render time
 *              from its `temp` (heat_attrs), never stored as separate state.
 *   DELAYS   : the only pause is Scene.paused; the phase holds (PERCH 12 s, …)
 *              are the FSM's phase_t-vs-phase_duration test, not a module.
 *
 * LOGIC (§4) is provably uncorruptable from RENDER: it does no mutation and no
 * I/O, so deleting or reordering any draw cannot change heat_attrs / phase_*.
 *
 * Per-tick combine order — main() (§7) is the only place that advances state,
 * and phoenix_tick() (via scene_tick) is the only simulation mutator:
 *     1. PERFORMANCE  measure dt (capped at DT_CAP_S)        §7
 *     2. SIMULATION   scene_tick(dt)  [skipped if paused]    §5
 *     3. RENDER       scene_draw()  (read-only)              §6
 *     4. PERFORMANCE  fps tally + sleep to the frame cap     §7
 *
 * User events (keys: pause/reset/skip/ignite/theme/density; SIGWINCH resize)
 * mutate Scene but are NOT part of the tick — handled before it in §7
 * (getch branch → app_handle_key, and the resize block).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS         60

/* Particle pools. */
#define N_BODY_MAX         600
#define N_BODY_DEFAULT     360
#define N_BODY_MIN         150
#define N_BODY_STEP         40

#define N_SPARK_MAX        400
#define N_SPARK_DEFAULT    220
#define N_SPARK_MIN         60
#define N_SPARK_STEP        30

#define N_ANCHOR_MAX       80

/* Display aspect — terminal cells are tall+narrow; multiply x by this
 * factor when drawing so the owl reads round, not stretched vertical. */
#define ASPECT_X           2.0f

/* Owl silhouette geometry (cells; head at origin, +y down).
 * The seed (REBIRTH growth centre) is the head centre. */
#define HEAD_CY           (-3.0f)
#define HEAD_RX            4.0f
#define HEAD_RY            3.0f
#define BODY_CY            ( 2.5f)
#define BODY_RX            3.0f
#define BODY_RY            3.5f
#define EYE_DX             2.0f
#define EYE_DY            (-3.0f)
#define BEAK_DY           (-1.4f)
#define EARTUFT_DX         3.4f
#define EARTUFT_DY        (-6.2f)

#define SEED_DX            0.0f
#define SEED_DY           (-3.0f)     /* head centre — REBIRTH source   */

/* Lifecycle timings (seconds). */
#define T_PERCH           12.0f
#define T_IGNITE           2.0f
#define T_BLAZE            3.0f
#define T_COLLAPSE         2.0f
#define T_ASH              4.0f
#define T_REBIRTH          3.0f

/* Body particle behaviour. */
#define BODY_JITTER_R      0.55f
#define BREATH_HZ          0.25f      /* slow chest rise, cycles/sec    */
#define BREATH_AMP         0.20f      /* tiny head/body bob, cells       */

/* IGNITE / BLAZE / COLLAPSE / REBIRTH temperature shaping.  See the
 * MENTAL MODEL "phase temp modifier" formulas. */
#define BLAZE_TEMP_MIN     0.85f
#define BLAZE_TEMP_MAX     1.00f
#define COLLAPSE_TEMP_HOT  1.00f
#define COLLAPSE_TEMP_COOL 0.30f

/* COLLAPSE: rate at which BOUND body particles flip BOUND→FREE.
 * With release rate 4/s and frac ramping 0→1, near end of COLLAPSE we
 * release ~4 particles/s/particle — i.e. virtually all in the last
 * fraction of the phase. */
#define COLLAPSE_RELEASE_RATE  6.0f

/* REBIRTH: rate at which FREE body particles try to recapture to an
 * alive anchor.  Each frame, prob = rate · dt — 5 means most particles
 * will have at least one capture attempt per second. */
#define REBIRTH_CAPTURE_RATE   5.0f

/* Spark emission rate, particles/sec, by phase. */
#define SPARK_HZ_PERCH       0.0f
#define SPARK_HZ_IGNITE     20.0f
#define SPARK_HZ_BLAZE     120.0f
#define SPARK_HZ_COLLAPSE   60.0f
#define SPARK_HZ_ASH         8.0f
#define SPARK_HZ_REBIRTH    45.0f      /* flames lick around the reforming owl */
#define SPARK_BURST_MAX     16
#define REBIRTH_FLARE_FRAC   4         /* onset flare = n_spark / this         */

/* Spark physics. */
#define SPARK_LIFE_BASE     1.6f
#define SPARK_LIFE_VAR      0.5f
#define SPARK_TEMP_INIT     0.95f
#define SPARK_TEMP_VAR      0.10f
#define SPARK_VEL_BASE      4.0f      /* initial speed, cells/sec       */
#define SPARK_VEL_VAR       0.6f
#define SPARK_VEL_UP        2.5f      /* extra upward bias              */

/* Free-flight fire physics — applied to body-FREE particles AND sparks. */
#define FIRE_BUOYANCY      11.0f
#define FIRE_DRAG           1.4f
#define FIRE_COOL           0.85f
#define FIRE_SHEAR_AMP      6.0f
#define FIRE_SHEAR_HZ       1.5f
#define FIRE_TEMP_FLOOR     0.04f

/* COLLAPSE release initial velocity range. */
#define COLLAPSE_VEL_LATERAL  3.0f    /* ±cells/sec sideways            */
#define COLLAPSE_VEL_DOWN     2.0f    /* base downward fall, cells/sec  */
#define COLLAPSE_VEL_UPVAR    3.5f    /* random component (some go up)  */

#define DT_CAP_S           0.10f
#define N_THEMES           4

/* Colour pair IDs */
#define PAIR_HEAT_0   1
#define PAIR_HEAT_1   2
#define PAIR_HEAT_2   3
#define PAIR_HEAT_3   4
#define PAIR_HEAT_4   5
#define PAIR_HEAT_5   6
#define PAIR_PERCH    7
#define PAIR_HUD      8
#define PAIR_HINT     9

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  performance — timing primitives (frame cap applied in §7)           */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec t;
    t.tv_sec  = ns / 1000000000LL;
    t.tv_nsec = ns % 1000000000LL;
    nanosleep(&t, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  types — the data SIMULATION owns (built/mutated in §5, read in §6)   */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Anchor — one fixed point of the owl pose, in body-local cells.  The owl is a
 * SHAPE (a list of ~38 of these), not a drawn image.
 *
 * WHY anchors + a swarm: rather than draw an owl, we list its pose points and
 * let a particle swarm snap to them each frame (the anchor-template pattern,
 * cf. ant_colony.c).  That makes ignite / dissolve / regrow trivial — the SAME
 * particles just stop or resume snapping — so one swarm renders owl AND fire.
 *
 * VALUE LOGIC:
 *   dx,dy     — position relative to the owl centre (cells; x is ×ASPECT_X at
 *               draw time so the bird reads round on tall terminal cells).
 *   weight    — relative draw probability (eyes 5.0 ≫ body-fill 0.7), so dense
 *               regions get proportionally more particles.
 *   base_temp — PERCH heat ∈ ~[0.14..0.55]: eyes hot (glow), body coolest.
 *   alive_at  — normalised distance from the SEED (head centre), 0..1.  REBIRTH
 *               admits anchor i only once growth_frac ≥ alive_at, so the bird
 *               grows outward from the eyes (KEY FORMULAS in the header). */
typedef struct {
    float dx, dy;      /* body-local position (cells)                  */
    float weight;      /* draw probability                              */
    float base_temp;   /* PERCH temperature                             */
    float alive_at;    /* REBIRTH wake threshold (0 = at seed, 1 = far) */
} Anchor;

/* BodyParticle — one owl-forming particle (a Reeves particle system, ref [1]).
 *
 * WHY two modes: the whole effect is ONE swarm that either traces the owl or
 * burns.  BOUND (released=false) snaps to an anchor every frame → the
 * silhouette.  FREE (released=true) integrates fire physics (vx/vy via
 * fire_step) → an ember that peeled off in COLLAPSE/ASH and drifts home in
 * REBIRTH.  The lifecycle FSM (§5) flips the mode per phase; nothing else
 * about the particle changes — that's what lets the owl become fire and back.
 *
 * VALUE LOGIC: x,y in world cells; vx,vy meaningful only while FREE; temp ∈
 * [0,1] drives the heat-ramp colour in BOTH modes (a perched owl is just
 * low-temp feathers); anchor_idx is the anchor it last snapped to. */
typedef struct {
    float x, y;          /* world cell coords                            */
    float vx, vy;        /* used while FREE (released)                   */
    float temp;          /* drives heat-ramp bucket                      */
    int   anchor_idx;    /* current anchor, when BOUND                   */
    bool  released;      /* false = BOUND, true = FREE                   */
} BodyParticle;

/* Spark — a pure free-flight ember (a Reeves particle system, ref [1]); a
 * separate, smaller pool from the body.
 *
 * WHY a second pool: body particles must return to the owl, so they can't fly
 * off forever.  Sparks CAN — they're the disposable wisps emitted from the
 * burning silhouette that rise, cool and die, plus the flare the owl bursts
 * from at REBIRTH.  Same fire_step physics as a FREE body particle, but with a
 * hard life cap so a slot frees up for re-emission.
 *
 * VALUE LOGIC: x,y / vx,vy in world cells & cells/s; temp ∈ [0,1] cools every
 * tick (drives the heat-ramp colour); life counts down seconds; active=false
 * marks a free slot, reused by the next emission. */
typedef struct {
    float x, y;
    float vx, vy;
    float temp;
    float life;
    bool  active;
} Spark;

/* PhoenixPhase — the self-immolation lifecycle as a finite state machine,
 * advanced in a fixed loop (phoenix myth, ref [7]):
 *   PERCH → IGNITE → BLAZE → COLLAPSE → ASH → REBIRTH → (PERCH)
 *
 * WHY an FSM: every visible difference between "owl" and "fire" is just which
 * BOUND/FREE rule and which temperature shaping is active.  Encoding that as one
 * enum keeps the per-particle update a clean switch and the whole cycle a single
 * counter (phase_t vs phase_duration).  Per-phase durations are the T_* config
 * literals; per-phase spark rates the SPARK_HZ_* ones. */
typedef enum {
    PHX_PERCH = 0,
    PHX_IGNITE,
    PHX_BLAZE,
    PHX_COLLAPSE,
    PHX_ASH,
    PHX_REBIRTH,
    N_PHX_PHASES,
} PhoenixPhase;

/* Phoenix — the bird as a whole: two cooperating particle systems over one
 * static anchor template, driven by one lifecycle FSM.
 *
 * WHY this shape: the owl POSE is fixed (anchors, built once at init); only
 * which particles snap to it — and how hot they are — changes over the cycle.
 * So the struct cleanly separates the unchanging template (anchors) from the
 * moving swarm (body, sparks) and the clock that drives them (world_t / phase /
 * phase_t).  Pools are fixed-capacity and never grown — no malloc in the hot
 * path; n_body / n_spark are the live counts the user tunes. */
typedef struct {
    /* Pose — owl centre + the perch it sits on (world cells). */
    float ox, oy;                /* world centre of owl                  */
    float perch_y;               /* perch row                             */

    /* Lifecycle clock — total time, time in phase, current phase. */
    float world_t;               /* seconds since reset (drives turbulence) */
    float phase_t;               /* seconds in the current phase            */
    PhoenixPhase phase;          /* current lifecycle phase                 */

    /* Anchor template — the owl shape, built once at init (never per-frame). */
    Anchor anchors[N_ANCHOR_MAX];
    int    n_anchor;             /* anchors actually written                */

    /* Particle pools — fixed capacity; only the first n_* are live. */
    BodyParticle body[N_BODY_MAX];
    int          n_body;         /* live body particles (,/. keys)          */
    Spark        sparks[N_SPARK_MAX];
    int          n_spark;        /* live spark slots (;/' keys)             */

    /* Spark emission — fractional carry so a fixed Hz emits smoothly. */
    float spark_emit_acc;
} Phoenix;

/* Scene — the whole program in one aggregate (the WHAT + WHERE + the knobs).
 * WHAT: phx, the phoenix.  WHERE/when: rows/cols (terminal) and fps (measured).
 * RENDER knob: theme (palette).  run-state: paused.  The body/spark density
 * knobs live on Phoenix beside the pools they size — those are simulation
 * knobs, not screen state. */
typedef struct {
    int     rows, cols;   /* terminal dimensions (WHERE)                      */
    Phoenix phx;          /* the phoenix: owl shape + fire + lifecycle (WHAT) */
    int     theme;        /* active colour palette — a RENDER choice          */
    bool    paused;       /* run-state                                        */
    float   fps;          /* measured frame rate, shown in the HUD            */
} Scene;

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  logic — pure decisions: no mutation, no I/O                         */
/* ═══════════════════════════════════════════════════════════════════════ */

/* heat_attrs — map a per-particle temperature to (color pair, glyph,
 * extra attributes).  Bucket 0 dim, buckets 1-3 normal, buckets 4-5
 * bold so the white-hot core dominates.  Glyph density (sparser →
 * denser) parallels heat (cool dot, bright fill char). */
static void heat_attrs(float temp, short *pair, char *glyph, attr_t *attrs)
{
    static const char glyphs[6] = { '.', '+', '*', '#', '@', '%' };
    int b = (int)floorf(temp * 6.0f);
    if (b < 0) b = 0;
    if (b > 5) b = 5;
    *glyph = glyphs[b];
    *pair  = (short)(PAIR_HEAT_0 + b);
    *attrs = (b == 0) ? A_DIM : (b >= 4 ? A_BOLD : 0);
}

static const char *phase_name(PhoenixPhase p)
{
    switch (p) {
    case PHX_PERCH:    return "PERCH   ";
    case PHX_IGNITE:   return "IGNITE  ";
    case PHX_BLAZE:    return "BLAZE   ";
    case PHX_COLLAPSE: return "COLLAPSE";
    case PHX_ASH:      return "ASH     ";
    case PHX_REBIRTH:  return "REBIRTH ";
    default:           return "?       ";
    }
}

static float phase_duration(PhoenixPhase p)
{
    switch (p) {
    case PHX_PERCH:    return T_PERCH;
    case PHX_IGNITE:   return T_IGNITE;
    case PHX_BLAZE:    return T_BLAZE;
    case PHX_COLLAPSE: return T_COLLAPSE;
    case PHX_ASH:      return T_ASH;
    case PHX_REBIRTH:  return T_REBIRTH;
    default:           return 1.f;
    }
}

static float spark_hz_for_phase(PhoenixPhase p)
{
    switch (p) {
    case PHX_PERCH:    return SPARK_HZ_PERCH;
    case PHX_IGNITE:   return SPARK_HZ_IGNITE;
    case PHX_BLAZE:    return SPARK_HZ_BLAZE;
    case PHX_COLLAPSE: return SPARK_HZ_COLLAPSE;
    case PHX_ASH:      return SPARK_HZ_ASH;
    case PHX_REBIRTH:  return SPARK_HZ_REBIRTH;
    default:           return 0.f;
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  simulation — advances state (sole mutator of the Phoenix / Scene)    */
/* ═══════════════════════════════════════════════════════════════════════ */

static float frand(void)         { return (float)rand() / (float)RAND_MAX; }
static float frand_signed(void)  { return frand() * 2.f - 1.f; }

/* emit_ellipse — write n anchors evenly around an ellipse centred at (cx,cy)
 * with radii (rx,ry), all sharing kind/weight/base_temp.  Returns next index. */
static int emit_ellipse(Anchor *out, int idx, int n, float cx, float cy,
                        float rx, float ry, float weight, float base_temp)
{
    for (int i = 0; i < n; i++) {
        float a = (float)i * (float)(2.0 * M_PI / n);
        out[idx].dx        = cx + rx * cosf(a);
        out[idx].dy        = cy + ry * sinf(a);
        out[idx].weight    = weight;
        out[idx].base_temp = base_temp;
        idx++;
    }
    return idx;
}

/* emit_point — write one feature anchor at (dx,dy).  Returns next index. */
static int emit_point(Anchor *out, int idx, float dx, float dy,
                      float weight, float base_temp)
{
    out[idx].dx        = dx;
    out[idx].dy        = dy;
    out[idx].weight    = weight;
    out[idx].base_temp = base_temp;
    return idx + 1;
}

/* normalize_alive_at — set each anchor's alive_at to its distance from the SEED,
 * normalised to [0,1], so REBIRTH grows the bird outward from the head centre
 * (eyes near the seed reappear first, ear tufts last). */
static void normalize_alive_at(Anchor *out, int n)
{
    float max_d = 0.f;
    for (int i = 0; i < n; i++) {
        float ddx = out[i].dx - SEED_DX;
        float ddy = out[i].dy - SEED_DY;
        float d   = sqrtf(ddx * ddx + ddy * ddy);
        if (d > max_d) max_d = d;
    }
    if (max_d < 0.001f) max_d = 1.f;
    for (int i = 0; i < n; i++) {
        float ddx = out[i].dx - SEED_DX;
        float ddy = out[i].dy - SEED_DY;
        float d   = sqrtf(ddx * ddx + ddy * ddy);
        out[i].alive_at = d / max_d;
    }
}

/*
 * anchor_table_build — populate `out[]` with the owl silhouette.
 * Returns the number of anchors written.
 *
 * Outline rings (head + body) give the silhouette; a small interior ring adds
 * mass; high-weight feature points (eyes, beak, ear tufts) read as features.
 * Finally alive_at is normalised so REBIRTH can grow the bird outward.
 */
static int anchor_table_build(Anchor *out)
{
    int idx = 0;

    /* Outline + mass: head ring, body ring (taller than wide), inner body fill. */
    idx = emit_ellipse(out, idx, 12, 0.f, HEAD_CY, HEAD_RX, HEAD_RY,
                       1.4f, 0.18f);
    idx = emit_ellipse(out, idx, 14, 0.f, BODY_CY, BODY_RX, BODY_RY,
                       1.2f, 0.16f);
    idx = emit_ellipse(out, idx,  5, 0.f, BODY_CY, BODY_RX * 0.45f, BODY_RY * 0.45f,
                       0.7f, 0.14f);

    /* Features: two glowing eyes, a beak, two ear tufts (high weight/temp). */
    idx = emit_point(out, idx, -EYE_DX,     EYE_DY,     5.0f, 0.55f);
    idx = emit_point(out, idx,  EYE_DX,     EYE_DY,     5.0f, 0.55f);
    idx = emit_point(out, idx,  0.f,        BEAK_DY,    2.0f, 0.40f);
    idx = emit_point(out, idx, -EARTUFT_DX, EARTUFT_DY, 1.5f, 0.22f);
    idx = emit_point(out, idx,  EARTUFT_DX, EARTUFT_DY, 1.5f, 0.22f);

    normalize_alive_at(out, idx);
    return idx;
}

/* anchor_pick — weighted-random anchor index over ALL anchors. */
static int anchor_pick(const Anchor *anch, int n)
{
    float total = 0.f;
    for (int i = 0; i < n; i++) total += anch[i].weight;
    float r = frand() * total;
    float acc = 0.f;
    for (int i = 0; i < n; i++) {
        acc += anch[i].weight;
        if (r <= acc) return i;
    }
    return n - 1;
}

/* anchor_pick_alive — weighted-random over anchors with alive_at <=
 * growth_frac (used during REBIRTH).  Returns -1 if none alive. */
static int anchor_pick_alive(const Anchor *anch, int n, float growth_frac)
{
    float total = 0.f;
    for (int i = 0; i < n; i++)
        if (anch[i].alive_at <= growth_frac) total += anch[i].weight;
    if (total <= 0.f) return -1;
    float r = frand() * total;
    float acc = 0.f;
    int last_alive = -1;
    for (int i = 0; i < n; i++) {
        if (anch[i].alive_at <= growth_frac) {
            last_alive = i;
            acc += anch[i].weight;
            if (r <= acc) return i;
        }
    }
    return last_alive;
}

/*
 * body_rebind_to — set particle position to anchor world pos + jitter,
 * record anchor index, mark BOUND.  Caller chooses the anchor so this
 * works for both unrestricted (PERCH/IGNITE/BLAZE) and alive-only
 * (REBIRTH) bind paths.
 */
static void body_rebind_to(BodyParticle *p, const Anchor *anch, int idx,
                           float ox, float oy, float bob)
{
    p->anchor_idx = idx;
    float ax = ox + anch[idx].dx * ASPECT_X;
    float ay = oy + anch[idx].dy + bob;
    p->x = ax + frand_signed() * BODY_JITTER_R * ASPECT_X;
    p->y = ay + frand_signed() * BODY_JITTER_R;
    p->released = false;
}

/*
 * body_release — convert a BOUND particle to FREE.  Initial velocity
 * is mostly downward (the bird is collapsing onto the perch) with
 * lateral spread and an upward random component (some embers fly up
 * even as the silhouette falls).
 *
 * Position is left at its current value (just snapped from rebind),
 * temperature is preserved so the released particle starts hot and
 * cools naturally — a sudden temp drop on release would look like
 * teleportation rather than disintegration.
 */
static void body_release(BodyParticle *p)
{
    p->released = true;
    p->vx = frand_signed() * COLLAPSE_VEL_LATERAL;
    /* y velocity: mostly down, with random component that occasionally
     * sends a piece upward — visually "embers escaping the collapse". */
    p->vy = COLLAPSE_VEL_DOWN + frand_signed() * COLLAPSE_VEL_UPVAR;
}

/*
 * fire_step — one integration step of the free-flight fire physics.
 * Used by FREE body particles AND by sparks.  Same buoyancy + sin-shear
 * + drag + cooling formulas as fire_tornado.c / volcano.c.
 */
static void fire_step(float *x, float *y, float *vx, float *vy, float *temp,
                      float dt, float t)
{
    /* Turbulence — sine-shear field, position-dependent so neighbours
     * get different forces (otherwise the column sways as a block). */
    float shx = sinf(FIRE_SHEAR_HZ * t + 0.30f * (*x) + 0.45f * (*y));
    float shy = cosf(FIRE_SHEAR_HZ * t + 0.45f * (*x) - 0.30f * (*y));
    *vx += FIRE_SHEAR_AMP * shx * dt;
    *vy += FIRE_SHEAR_AMP * shy * dt * 0.6f;

    /* Buoyancy (negative y = up).  Smoke (low temp) keeps a small floor
     * lift so wisps drift up. */
    float lift = FIRE_BUOYANCY * (0.20f + 0.80f * (*temp));
    *vy -= lift * dt;

    /* Drag — multiplicative velocity damping. */
    float k = 1.f - FIRE_DRAG * dt;
    if (k < 0.f) k = 0.f;
    *vx *= k;
    *vy *= k;

    /* Integrate. */
    *x += *vx * dt;
    *y += *vy * dt;

    /* Cooling — exponential. */
    *temp *= expf(-FIRE_COOL * dt);
    if (*temp < 0.f) *temp = 0.f;
}

/*
 * spark_emit — one new spark at (sx,sy), with mostly-upward velocity
 * (sparks rise from a fire) plus lateral spread.  Life and temp
 * randomised so the resulting trail isn't striped.
 */
static void spark_emit(Spark *s, float sx, float sy)
{
    float speed   = SPARK_VEL_BASE * (1.f + SPARK_VEL_VAR * frand_signed());
    float ang     = (float)(-M_PI / 2.0)               /* up */
                  + frand_signed() * 0.7f;             /* ±~40° */
    s->x          = sx + frand_signed() * 0.6f;
    s->y          = sy + frand_signed() * 0.4f;
    s->vx         = cosf(ang) * speed;
    s->vy         = sinf(ang) * speed - SPARK_VEL_UP;
    s->temp       = SPARK_TEMP_INIT + frand_signed() * SPARK_TEMP_VAR;
    if (s->temp > 1.f) s->temp = 1.f;
    if (s->temp < 0.f) s->temp = 0.f;
    s->life       = SPARK_LIFE_BASE * (1.f + SPARK_LIFE_VAR * frand_signed());
    s->active     = true;
}

static void spark_tick(Spark *s, float dt, float t)
{
    if (!s->active) return;
    fire_step(&s->x, &s->y, &s->vx, &s->vy, &s->temp, dt, t);
    s->life -= dt;
    if (s->life <= 0.f || s->temp <= FIRE_TEMP_FLOOR) s->active = false;
}

/* Compute the per-frame BOUND temperature for a body particle, given
 * the anchor's base_temp and the current phase + phase fraction.
 * This is where each phase's "temperament" lives — see the MENTAL
 * MODEL "phase temp modifier" section.  (Not in §4 LOGIC: it draws on
 * the PRNG via frand_signed.) */
static float bound_temp(PhoenixPhase phase, float frac, float base_temp,
                        float alive_at)
{
    switch (phase) {
    case PHX_PERCH:
        return base_temp + frand_signed() * 0.05f;
    case PHX_IGNITE: {
        /* Linearly blend from base_temp toward 1.0.  At frac=1 the bird
         * is fully ignited (white-hot); at frac=0 it's still itself. */
        float t = base_temp + (1.0f - base_temp) * frac;
        return t + frand_signed() * 0.05f;
    }
    case PHX_BLAZE:
        return BLAZE_TEMP_MIN + (BLAZE_TEMP_MAX - BLAZE_TEMP_MIN) * frand();
    case PHX_COLLAPSE: {
        float t = COLLAPSE_TEMP_HOT
                + (COLLAPSE_TEMP_COOL - COLLAPSE_TEMP_HOT) * frac;
        return t + frand_signed() * 0.05f;
    }
    case PHX_REBIRTH: {
        /* Each particle is born hot (flame at the seed, cooler at the edges),
         * then cools toward its settled feather colour (base_temp) as the bird
         * completes (frac → 1).  So the owl EMERGES from the flames and resolves
         * into the cool silhouette, rather than staying on fire the whole time. */
        float hot = 0.95f - 0.30f * alive_at;       /* flamy core, cooler out */
        float t   = hot + (base_temp - hot) * frac; /* flame → feather over phase */
        return t + frand_signed() * 0.05f;
    }
    default:
        return base_temp;
    }
}

/*
 * phoenix_init — build anchors, place owl at the screen centre slightly
 * above the perch, distribute body particles to anchors with valid
 * initial state.  Sparks all inactive.
 */
static void phoenix_init(Phoenix *p, int rows, int cols, int n_body, int n_spark)
{
    memset(p, 0, sizeof *p);
    p->ox       = (float)cols * 0.5f;
    p->oy       = (float)rows * 0.45f;
    p->perch_y  = (float)rows * 0.45f + (BODY_CY + BODY_RY + 1.0f);
    p->phase    = PHX_PERCH;
    p->n_body   = n_body;
    p->n_spark  = n_spark;
    p->n_anchor = anchor_table_build(p->anchors);

    /* Initial bind so frame 0 is already an owl, not a smear. */
    for (int i = 0; i < p->n_body; i++) {
        int idx = anchor_pick(p->anchors, p->n_anchor);
        body_rebind_to(&p->body[i], p->anchors, idx, p->ox, p->oy, 0.f);
        p->body[i].temp = p->anchors[idx].base_temp;
        p->body[i].vx = p->body[i].vy = 0.f;
    }
    for (int i = 0; i < p->n_spark; i++) p->sparks[i].active = false;
    p->spark_emit_acc = 0.f;
}

static void phoenix_reset(Phoenix *p, int rows, int cols)
{
    int nb = p->n_body, ns = p->n_spark;
    phoenix_init(p, rows, cols, nb, ns);
}

static void phoenix_resize(Phoenix *p, int rows, int cols)
{
    p->ox      = (float)cols * 0.5f;
    p->oy      = (float)rows * 0.45f;
    p->perch_y = (float)rows * 0.45f + (BODY_CY + BODY_RY + 1.0f);
}

/* On entering PERCH, snap any still-FREE particle back to an anchor so the
 * resting owl is whole, not half-rebuilt. */
static void rebind_strays_to_owl(Phoenix *p)
{
    for (int i = 0; i < p->n_body; i++) {
        if (p->body[i].released) {
            int idx = anchor_pick(p->anchors, p->n_anchor);
            body_rebind_to(&p->body[i], p->anchors, idx, p->ox, p->oy, 0.f);
            p->body[i].temp = p->anchors[idx].base_temp;
            p->body[i].vx = p->body[i].vy = 0.f;
        }
    }
}

/* On entering REBIRTH, force every particle FREE so the rebuild starts from a
 * clean slate (no leftover bound "ghost owl" from the COLLAPSE end). */
static void release_all_body(Phoenix *p)
{
    for (int i = 0; i < p->n_body; i++) {
        if (!p->body[i].released) {
            p->body[i].released = true;
            /* Inherit current position; assign small drift. */
            p->body[i].vx = frand_signed() * 1.5f;
            p->body[i].vy = -1.0f + frand_signed() * 0.8f;
        }
    }
}

/* Flame flare at the seed — the burst of upward fire the owl bursts out of and
 * then reforms from (REBIRTH onset). */
static void emit_seed_flare(Phoenix *p)
{
    float sx = p->ox + SEED_DX * ASPECT_X;
    float sy = p->oy + SEED_DY;
    int flare = p->n_spark / REBIRTH_FLARE_FRAC, lit = 0;
    for (int i = 0; i < p->n_spark && lit < flare; i++) {
        if (p->sparks[i].active) continue;
        spark_emit(&p->sparks[i], sx, sy);
        lit++;
    }
}

/* phoenix_advance_phase — phase_t exceeded duration; step to the next phase
 * and run its entry action (clean owl on PERCH; release + flare on REBIRTH). */
static void phoenix_advance_phase(Phoenix *p)
{
    p->phase_t = 0.f;
    p->phase   = (PhoenixPhase)((int)(p->phase + 1) % N_PHX_PHASES);

    if (p->phase == PHX_PERCH)   rebind_strays_to_owl(p);
    if (p->phase == PHX_REBIRTH) { release_all_body(p); emit_seed_flare(p); }
}

/* Snap a body particle to a weighted-random anchor (full pool) and set its
 * temperature for the current phase.  Used by PERCH/IGNITE/BLAZE and by the
 * still-bound branch of COLLAPSE. */
static void body_bind_random(Phoenix *p, BodyParticle *b, float frac, float bob)
{
    int idx = anchor_pick(p->anchors, p->n_anchor);
    body_rebind_to(b, p->anchors, idx, p->ox, p->oy, bob);
    b->temp = bound_temp(p->phase, frac,
                         p->anchors[idx].base_temp, p->anchors[idx].alive_at);
}

/* Try to snap a body particle onto an AWAKE anchor (alive_at ≤ growth) during
 * REBIRTH; returns true if it bound, false if none are awake yet. */
static bool body_bind_alive(Phoenix *p, BodyParticle *b, float frac, float bob)
{
    int idx = anchor_pick_alive(p->anchors, p->n_anchor, frac);
    if (idx < 0) return false;
    body_rebind_to(b, p->anchors, idx, p->ox, p->oy, bob);
    b->temp = bound_temp(PHX_REBIRTH, frac,
                         p->anchors[idx].base_temp, p->anchors[idx].alive_at);
    return true;
}

/* COLLAPSE: a still-bound particle has a frac-rising chance to release; once
 * FREE it integrates fire physics.  By the end virtually all are FREE. */
static void body_tick_collapse(Phoenix *p, BodyParticle *b, float frac, float dt,
                               float bob)
{
    if (!b->released) {
        if (frand() < frac * COLLAPSE_RELEASE_RATE * dt) body_release(b);
        else                                             body_bind_random(p, b, frac, bob);
    }
    if (b->released)
        fire_step(&b->x, &b->y, &b->vx, &b->vy, &b->temp, dt, p->world_t);
}

/* ASH: force FREE (smouldering wisps drift up) and integrate fire physics. */
static void body_tick_ash(Phoenix *p, BodyParticle *b, float dt)
{
    if (!b->released) {
        b->released = true;
        b->vx = frand_signed() * 1.0f;
        b->vy = -0.5f + frand_signed() * 0.8f;
    }
    fire_step(&b->x, &b->y, &b->vx, &b->vy, &b->temp, dt, p->world_t);
}

/* REBIRTH: a FREE particle tries to capture onto an awake anchor each frame
 * (else keeps flying); a bound straggler re-binds to an awake anchor. */
static void body_tick_rebirth(Phoenix *p, BodyParticle *b, float frac, float dt,
                              float bob)
{
    if (!b->released) {
        body_bind_alive(p, b, frac, bob);
        return;
    }
    bool captured = (frand() < REBIRTH_CAPTURE_RATE * dt)
                 && body_bind_alive(p, b, frac, bob);
    if (!captured)
        fire_step(&b->x, &b->y, &b->vx, &b->vy, &b->temp, dt, p->world_t);
}

/*
 * phoenix_tick_body — update every body particle by dispatching to the current
 * phase's rule: BOUND (snap to the owl) for PERCH/IGNITE/BLAZE, the BOUND→FREE
 * release for COLLAPSE, free drift for ASH, and the FREE→BOUND recapture for
 * REBIRTH.  See each body_tick_* / body_bind_* helper.
 */
static void phoenix_tick_body(Phoenix *p, float dt, float bob)
{
    float frac = p->phase_t / phase_duration(p->phase);
    if (frac > 1.f) frac = 1.f;

    for (int i = 0; i < p->n_body; i++) {
        BodyParticle *b = &p->body[i];
        switch (p->phase) {
        case PHX_PERCH:
        case PHX_IGNITE:
        case PHX_BLAZE:    body_bind_random(p, b, frac, bob);        break;
        case PHX_COLLAPSE: body_tick_collapse(p, b, frac, dt, bob);  break;
        case PHX_ASH:      body_tick_ash(p, b, dt);                  break;
        case PHX_REBIRTH:  body_tick_rebirth(p, b, frac, dt, bob);   break;
        default: break;
        }
    }
}

/*
 * phoenix_tick_sparks — emit sparks from random anchors at the rate
 * dictated by the current phase, then integrate every active spark.
 */
static void phoenix_tick_sparks(Phoenix *p, float dt)
{
    float hz = spark_hz_for_phase(p->phase);
    p->spark_emit_acc += dt * hz;
    int to_emit = (int)p->spark_emit_acc;
    p->spark_emit_acc -= (float)to_emit;
    if (to_emit > SPARK_BURST_MAX) to_emit = SPARK_BURST_MAX;

    int emitted = 0;
    for (int i = 0; i < p->n_spark && emitted < to_emit; i++) {
        if (p->sparks[i].active) continue;
        int idx = anchor_pick(p->anchors, p->n_anchor);
        float sx = p->ox + p->anchors[idx].dx * ASPECT_X;
        float sy = p->oy + p->anchors[idx].dy;
        spark_emit(&p->sparks[i], sx, sy);
        emitted++;
    }

    for (int i = 0; i < p->n_spark; i++)
        spark_tick(&p->sparks[i], dt, p->world_t);
}

/*
 * phoenix_tick — top-level update.  Advance time, advance phase, then
 * step body and sparks.  Body bob is a slow vertical breath so even
 * the perched owl looks alive.
 */
static void phoenix_tick(Phoenix *p, float dt)
{
    p->world_t += dt;
    p->phase_t += dt;

    if (p->phase_t >= phase_duration(p->phase))
        phoenix_advance_phase(p);

    /* Breath bob — only obvious during PERCH but harmless during
     * fire phases (the rebind absorbs the small offset). */
    float bob = (p->phase == PHX_PERCH)
              ? sinf(p->world_t * (float)(2.0 * M_PI) * BREATH_HZ) * BREATH_AMP
              : 0.f;

    phoenix_tick_body(p, dt, bob);
    phoenix_tick_sparks(p, dt);
}

static void scene_init(Scene *s, int rows, int cols)
{
    memset(s, 0, sizeof *s);
    s->rows = rows;
    s->cols = cols;
    phoenix_init(&s->phx, rows, cols, N_BODY_DEFAULT, N_SPARK_DEFAULT);
}

static void scene_resize(Scene *s, int rows, int cols)
{
    s->rows = rows;
    s->cols = cols;
    phoenix_resize(&s->phx, rows, cols);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    phoenix_tick(&s->phx, dt);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  render — state → screen; reads only, never mutates sim state         */
/* ═══════════════════════════════════════════════════════════════════════ */

/* Theme — a 6-step heat ramp (cool → hot) that doubles as BOTH the owl's
 * feather palette and the fire palette, plus the perch colour.
 *
 * WHY one ramp for feathers AND fire: a particle's colour is purely a function
 * of its temperature (heat_attrs maps temp → bucket, ref [2] blackbody-style
 * fire colour).  A perched owl is just the cool end of the ramp (buckets 1-2);
 * a blaze is the hot end (4-5).  So one 6-colour ramp per theme defines the
 * whole look — feathers, embers, white-hot core — and 't' swaps the bird's
 * "species" and its fire at once.  Bucket 0 is smoke/ash (grey); all entries
 * sit in the bright half of the 256 cube (CLAUDE.md palette-brightness rule). */
typedef struct {
    const char *name;
    short       heat[6];     /* cool → hot, indexed by heat bucket 0..5  */
    short       perch;        /* perch (branch) colour                   */
} Theme;

static const Theme g_themes[N_THEMES] = {
    /* CLASSIC — brown owl, red/orange/yellow fire, white core. */
    { "CLASSIC", { 244, 130, 166, 202, 214, 231 }, 94 },
    /* IRIS    — purple/pink phoenix. */
    { "IRIS   ", { 244,  96, 134, 170, 213, 231 }, 60 },
    /* JADE    — green-jade phoenix. */
    { "JADE   ", { 244,  28,  70, 112, 154, 231 }, 64 },
    /* GOLD    — copper/gold phoenix. */
    { "GOLD   ", { 244,  94, 130, 172, 214, 231 }, 94 },
};

static void color_init(int theme_idx)
{
    if (!has_colors()) return;
    start_color();
    use_default_colors();
    const Theme *th = &g_themes[theme_idx % N_THEMES];

    if (COLORS >= 256) {
        init_pair(PAIR_HEAT_0, th->heat[0], -1);
        init_pair(PAIR_HEAT_1, th->heat[1], -1);
        init_pair(PAIR_HEAT_2, th->heat[2], -1);
        init_pair(PAIR_HEAT_3, th->heat[3], -1);
        init_pair(PAIR_HEAT_4, th->heat[4], -1);
        init_pair(PAIR_HEAT_5, th->heat[5], -1);
        init_pair(PAIR_PERCH,  th->perch,   -1);
        init_pair(PAIR_HUD,    226,         -1);
        init_pair(PAIR_HINT,    51,         -1);
    } else {
        init_pair(PAIR_HEAT_0, COLOR_WHITE,   -1);
        init_pair(PAIR_HEAT_1, COLOR_RED,     -1);
        init_pair(PAIR_HEAT_2, COLOR_RED,     -1);
        init_pair(PAIR_HEAT_3, COLOR_YELLOW,  -1);
        init_pair(PAIR_HEAT_4, COLOR_YELLOW,  -1);
        init_pair(PAIR_HEAT_5, COLOR_WHITE,   -1);
        init_pair(PAIR_PERCH,  COLOR_YELLOW,  -1);
        init_pair(PAIR_HUD,    COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,   COLOR_CYAN,    -1);
    }
}

static void body_draw(const BodyParticle *p, int rows, int cols)
{
    int cx = (int)lroundf(p->x);
    int cy = (int)lroundf(p->y);
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;

    short  pair;
    char   gl;
    attr_t at;
    heat_attrs(p->temp, &pair, &gl, &at);
    attron(COLOR_PAIR(pair) | at);
    mvaddch(cy, cx, (chtype)(unsigned char)gl);
    attroff(COLOR_PAIR(pair) | at);
}

static void spark_draw(const Spark *s, int rows, int cols)
{
    if (!s->active) return;
    int cx = (int)lroundf(s->x);
    int cy = (int)lroundf(s->y);
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;

    short  pair;
    char   gl;
    attr_t at;
    heat_attrs(s->temp, &pair, &gl, &at);
    attron(COLOR_PAIR(pair) | at);
    mvaddch(cy, cx, (chtype)(unsigned char)gl);
    attroff(COLOR_PAIR(pair) | at);
}

/*
 * draw_perch — static branch under the bird, dim themed brown.
 * Length ≈ body width + padding, plus two short "leg" anchors below
 * to suggest depth.  Drawn before particles so the bird can sit on it.
 */
static void draw_perch(const Scene *s)
{
    int row = (int)lroundf(s->phx.perch_y);
    if (row < 0 || row >= s->rows) return;
    int half = (int)(BODY_RX * ASPECT_X) + 4;
    int cx   = (int)lroundf(s->phx.ox);
    int x0   = cx - half;
    int x1   = cx + half;
    if (x0 < 0)        x0 = 0;
    if (x1 >= s->cols) x1 = s->cols - 1;

    attron(COLOR_PAIR(PAIR_PERCH) | A_BOLD);
    for (int x = x0; x <= x1; x++)
        mvaddch(row, x, (chtype)(unsigned char)'-');
    /* End caps: slight upturn so it looks like a branch, not a beam. */
    if (x0 - 1 >= 0)        mvaddch(row, x0 - 1,
                                    (chtype)(unsigned char)'/');
    if (x1 + 1 <  s->cols)  mvaddch(row, x1 + 1,
                                    (chtype)(unsigned char)'\\');
    /* Drop strut so it grounds visually. */
    if (row + 1 < s->rows)
        mvaddch(row + 1, cx, (chtype)(unsigned char)'|');
    attroff(COLOR_PAIR(PAIR_PERCH) | A_BOLD);
}

/* HUD — phase + theme + counts + fps top-right; key hints bottom-left. */
static void draw_hud(const Scene *s)
{
    char buf[120];
    snprintf(buf, sizeof buf,
             " PHOENIX  %s   %s  body:%d  spark:%d  %5.1f fps ",
             g_themes[s->theme].name,
             phase_name(s->phx.phase),
             s->phx.n_body, s->phx.n_spark,
             s->fps);
    int len = (int)strlen(buf);
    if (len > s->cols) len = s->cols;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, s->cols - len, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset  s:skip  i:ignite  t/T:theme  ,/.:body  ;/':sparks ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/*
 * scene_draw — render order: perch → body (BOUND first, then FREE) →
 * sparks → HUD.  Drawing BOUND before FREE ensures fire that's flying
 * over the silhouette doesn't get overwritten by a re-snap below it.
 */
static void scene_draw(const Scene *s)
{
    erase();
    draw_perch(s);

    /* BOUND body first (the silhouette). */
    for (int i = 0; i < s->phx.n_body; i++)
        if (!s->phx.body[i].released)
            body_draw(&s->phx.body[i], s->rows, s->cols);

    /* FREE body next — released embers, drawn on top of the silhouette
     * they're disintegrating from. */
    for (int i = 0; i < s->phx.n_body; i++)
        if (s->phx.body[i].released)
            body_draw(&s->phx.body[i], s->rows, s->cols);

    /* Sparks (pure free-flight pool) on top. */
    for (int i = 0; i < s->phx.n_spark; i++)
        spark_draw(&s->phx.sparks[i], s->rows, s->cols);

    draw_hud(s);
}

static void screen_init(int theme_idx)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    typeahead(-1);
    curs_set(0);
    color_init(theme_idx);
}

static void screen_cleanup(void)
{
    if (!isendwin()) {
        curs_set(1);
        endwin();
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  app — combine point (per-tick order) + user events + frame cap       */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running = 0;
}

static void install_signals(void)
{
    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGWINCH,&sa, NULL);
}

static void atexit_cleanup(void) { screen_cleanup(); }

/* phoenix_skip_phase — finish the current phase immediately so
 * advance fires at the next tick.  Used by 's' and 'i' (jump to
 * IGNITE). */
static void phoenix_skip_phase(Phoenix *p)
{
    p->phase_t = phase_duration(p->phase);
}

static void phoenix_jump_to_ignite(Phoenix *p)
{
    p->phase   = PHX_IGNITE;
    p->phase_t = 0.f;
}

/* User event — mutates Scene, but NOT part of the per-tick combine order. */
static bool app_handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':
    case 'p': case 'P': s->paused = !s->paused;                       break;
    case 'r': case 'R': phoenix_reset(&s->phx, s->rows, s->cols);     break;
    case 's': case 'S': phoenix_skip_phase(&s->phx);                  break;
    case 'i': case 'I': phoenix_jump_to_ignite(&s->phx);              break;

    case 't':
        s->theme = (s->theme + 1) % N_THEMES;
        color_init(s->theme);
        break;
    case 'T':
        s->theme = (s->theme + N_THEMES - 1) % N_THEMES;
        color_init(s->theme);
        break;

    case '.': case '>':
        if (s->phx.n_body + N_BODY_STEP <= N_BODY_MAX)
            s->phx.n_body += N_BODY_STEP;
        break;
    case ',': case '<':
        if (s->phx.n_body - N_BODY_STEP >= N_BODY_MIN)
            s->phx.n_body -= N_BODY_STEP;
        break;

    case '\'': case '"':
        if (s->phx.n_spark + N_SPARK_STEP <= N_SPARK_MAX)
            s->phx.n_spark += N_SPARK_STEP;
        break;
    case ';': case ':':
        if (s->phx.n_spark - N_SPARK_STEP >= N_SPARK_MIN)
            s->phx.n_spark -= N_SPARK_STEP;
        break;

    default: break;
    }
    return true;
}

static void app_do_resize(Scene *s)
{
    endwin();
    refresh();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    scene_resize(s, rows, cols);
    g_need_resize = 0;
}

int main(void)
{
    install_signals();
    atexit(atexit_cleanup);

    int seed = (int)(clock_ns() & 0x7FFFFFFF);
    srand((unsigned)seed);

    screen_init(0);
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    Scene scene;
    scene_init(&scene, rows, cols);

    const int64_t frame_ns = 1000000000LL / TARGET_FPS;
    int64_t prev = clock_ns();

    int frames = 0;
    int64_t fps_t0 = prev;

    while (g_running) {
        /* USER EVENT (out of tick): resize */
        if (g_need_resize) app_do_resize(&scene);

        /* USER EVENT (out of tick): key handling */
        int ch = getch();
        if (ch != ERR && !app_handle_key(&scene, ch)) g_running = 0;

        /* 1. PERFORMANCE — measure dt, capped to avoid spiral-of-death */
        int64_t now = clock_ns();
        float dt = (float)(now - prev) * 1e-9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;
        prev = now;

        /* 2. SIMULATION — the sole state advance (skipped while paused) */
        scene_tick(&scene, dt);

        /* 3. RENDER — read-only */
        scene_draw(&scene);
        wnoutrefresh(stdscr);
        doupdate();

        /* 4. PERFORMANCE — fps tally + sleep to the frame cap */
        frames++;
        if (now - fps_t0 >= 500000000LL) {
            scene.fps = (float)frames * 1e9f / (float)(now - fps_t0);
            frames    = 0;
            fps_t0    = now;
        }

        int64_t sleep_ns = frame_ns - (clock_ns() - now);
        clock_sleep_ns(sleep_ns);
    }

    return 0;
}
