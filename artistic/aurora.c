/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * aurora.c — Aurora Borealis (side view: curtains over a horizon)
 *
 * The photo angle: vertical aurora curtains hang in the sky over a jagged dark
 * land silhouette, under a field of twinkling stars. Each curtain is brightest
 * GREEN along a rippling lower hem and fades UPWARD through cyan to PURPLE tips;
 * an 18-colour bright ramp + a sideways-rolling colour wave + vertical drapery
 * folds give it depth and motion. (See aurora_draw / hem_row / mountain_top.)
 *
 * Keys:
 *   q/ESC quit   space pause   r regen (new seed)   t/T colour theme
 *   ] / [  sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra aurora.c -o aurora -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Closed-form scene, no state grid. Every cell is a pure
 *                  function of (col, row, time, seed) evaluated fresh each frame.
 *                  Per COLUMN we know two boundaries: the land ridge
 *                  (mountain_top) and the curtain's bright lower hem (hem_row).
 *                  Down a column the bands are land → sky → curtain → sky.
 *
 * Math           : x = col / cols × 2π (horizontal phase). For a cell `d` rows
 *                  above the hem (dn = d / curtain_len ∈ [0,1]):
 *                    intensity = fade_up(dn) · folds(col) · shimmer(x, d)
 *                  where shimmer is a two-octave product-of-sines, folds is a
 *                  per-column sinusoid, fade_up = 1 − dn makes the hem brightest.
 *                  A seed (LCG + bit-mix hash, seed_unit) phases the mountains,
 *                  hem, texture and colour and jitters the mood parameters.
 *
 * Rendering      : Cells below the floor fall back to a stateless star hash.
 *                  Curtain colour runs the active theme ramp GREEN at the hem →
 *                  PURPLE at the tips (aurora_pair), shifted by a rolling colour
 *                  wave; brightness picks the glyph '.'/':'/'|'/'!'/'#' and
 *                  A_DIM/A_BOLD (curtain_glyph). Land is a dark silhouette.
 *
 * References     : Aurora physics & colour —
 *  • Chamberlain, J.W. *Physics of the Aurora and Airglow* (Academic Press
 *    1961; AGU reprint 1995) — the emission lines behind the colour themes:
 *    atomic-oxygen 557.7 nm GREEN, oxygen 630 nm RED, N2+ 427.8 nm
 *    BLUE/VIOLET, and why altitude sorts them bottom-to-top.
 *  • Eather, R.H. *Majestic Lights: The Aurora in Science, History, and the
 *    Arts* (AGU 1980) — auroral forms: arcs, rays, curtains/draperies and the
 *    bright lower border this side view models.
 *  • Akasofu, S.-I. *Exploring the Secrets of the Aurora* (2nd ed., Springer
 *    2007) — curtain morphology and dynamics.
 *                  Procedural rendering, noise & hashing —
 *  • Perlin, K. "An Image Synthesizer," SIGGRAPH 1985 — procedural noise; the
 *    basis for the organic shimmer term.
 *  • Ebert, Musgrave, Peachey, Perlin & Worley, *Texturing & Modeling: A
 *    Procedural Approach* (3rd ed., Morgan Kaufmann 2003) — fBm, layered
 *    noise, procedural skies, and the heightfield idea behind the ridge.
 *  • Finch, M. "Effective Water Simulation from Physical Models," GPU Gems
 *    ch. 1 (2004) — sum-of-sines surface animation; the additive sinusoids
 *    that drive the curtain ripple and drapery folds.
 *  • Jarzynski, M. & Olano, M. "Hash Functions for GPU Rendering," JCGT 9(3),
 *    2020 — integer bit-mixing hashes (the h*=0x7feb352d finalizer style)
 *    behind the stateless star field and the seed fan-out (seed_unit).
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The aurora is NOT a particle system, NOT a fluid sim — the whole display is a
 * closed-form function of (col, row, time, seed) painted fresh each frame, with
 * no grid of state. State is just two numbers (Aurora.time, Aurora.seed): time
 * moves the scene, seed chooses which scene. A SIDE view — curtains hang in the
 * sky over a dark land silhouette, brightest along a rippling GREEN hem and
 * fading UP to PURPLE tips. Stars are stateless too: a hash of (col, row) decides
 * each one, so they never move or need storage.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Per column there are two horizontal lines: the land ridge (mountain_top) and
 * the curtain hem (hem_row), the latter rippling/drifting over time. Walking
 * DOWN a column you pass: dark land (below the ridge), then sky, then the curtain
 * band hanging above its hem, then more sky. The curtain's brightness is a fade
 * from the bright hem upward, carved by vertical drapery folds and an organic
 * two-octave shimmer; its colour climbs the theme ramp by height and is swept by
 * a colour wave rolling sideways. Drift comes from `time` entering the sin/cos
 * phases LINEARLY. A new `seed` (r key) re-phases every sinusoid and re-jitters
 * the mood parameters, so the skyline, curtain reach and colours all change.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Each tick (pause freezes it): time += dt (aurora_tick).
 *  2. Per frame, derive from seed: phases for mountains/hem/texture/colour and
 *     mood params (mtn_amp1, hem_base, curtain_len, fold_freq). Precompute the
 *     ridge[] and hem[] row of every column.
 *  3. For each cell (col, row), resolve its band:
 *       row ≥ ridge[col]            → dark land silhouette (draw_land).
 *       above_hem = hem[col] − row
 *       above_hem < 0               → below the hem → sky (draw_star).
 *       dn = above_hem / curtain_len; dn > CURTAIN_TOP → past the tip → sky.
 *       bright = curtain_intensity(); bright < AURORA_FLOOR → sky.
 *       else → curtain: pair = aurora_pair(dn,…) (green hem → purple tip + colour
 *              wave); ch/attr = curtain_glyph(bright).
 *
 * KEY FORMULAS
 * ────────────
 *  Phase            x = col / cols · 2π
 *  Curtain coord    dn = (hem[col] − row) / curtain_len     0 at hem → 1 at tip
 *  Brightness       I = max(0, 1−dn) · fold(col) · (0.55 + 0.45·shimmer(x, d))
 *  Hue ramp index   idx = dn·(N_AURORA_RAMP−1) + colwave·COLOR_WAVE_AMP
 *  Star hash        h = col·1234597 ⊕ row·987659 ⊕ (col + 31·row);
 *                   h ⊕= h>>13;  h *= 0x5bd1e995;  h ⊕= h>>15;
 *                   star ⇔ (h & 0xFF) < STAR_THRESH (≈ 2%)
 *  Reseed (r)       seed = seed·1664525 + 1013904223   (LCG step)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • The shimmer octaves ride on time at different rates/frequencies, so the
 *    pattern never exactly repeats; make them equal and it becomes static.
 *  • bright < AURORA_FLOOR is the transparency floor; lowering it paints the
 *    whole band and drowns the stars showing through the curtain.
 *  • dn > CURTAIN_TOP (slightly past 1.0) lets the tip fade out rather than cut.
 *  • The star/curtain hashes depend only on (col,row,t/seed), so RESIZE re-maps
 *    cells but the field stays locally stable — no per-cell storage to corrupt.
 *  • Pause freezes time but drawing continues — a frozen frame, not a black one.
 *  • SIM_FPS sets how OFTEN time advances, not the drift SPEED (that is in the
 *    sinusoid coefficients).
 *
 * HOW TO VERIFY
 * ─────────────
 *  • The bottom carries a jagged dark land ridge; curtains hang above it with a
 *    bright green hem fading up to purple, stars filling the sky around them.
 *  • Press r repeatedly: skyline, curtain shape and colours should all change.
 *  • Cycle t/T: the palette switches (EMERALD/SPECTRUM/CRIMSON/VIOLET) while the
 *    shape stays put.
 *  • Pause (space): motion stops, the frame holds. Tap ]/[ to change Hz —
 *    smoothness changes, drift speed does not.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * Re-cut from first principles into separated concern-layers (a SEPARATION
 * pass: RELOCATE + LABEL only — every function body is byte-identical, nothing
 * renamed). The aurora is a CLOSED-FORM scene: every cell is a pure function of
 * (col, row, time, seed), so there is almost no state to advance. Layer →
 * section → what it mutates:
 *
 *   LAYER        §   MUTATES
 *   ─────────────────────────────────────────────────────────────────────
 *   CONFIG       §1  nothing — constants, colour themes, composition fractions.
 *   PERFORMANCE  §2  nothing — clock_ns / clock_sleep_ns; the frame cap + the
 *                    fixed-timestep accumulator are POLICY in main (§8).
 *   LOGIC        §3  nothing — pure decisions (star_at, seed_unit, mountain_top,
 *                    hem_row, curtain_intensity). No render/effects reorder can
 *                    change their result.
 *   SIMULATION   §4  Aurora.{time,seed} — aurora_tick advances time, aurora_reseed
 *                    rolls the seed. The ONLY writers of sim state. (The Scene
 *                    aggregate also carries paused → §6 and theme → §7, flipped
 *                    by user events in §8.)
 *   EFFECTS      §5  (none) — shimmer / folds / twinkle / colour-wave / glow are
 *                    derived at render time from (col,row,time,seed); none stored.
 *   DELAYS       §6  (none) — only the pause flag (Scene.paused), checked in
 *                    scene_tick (§4). No holds or timers.
 *   RENDER       §7  ncurses back buffer + colour-pair table only (apply_aurora_
 *                    theme, color_init, aurora_draw, draw_star, scene_draw,
 *                    screen_*). Reads sim state; never writes it.
 *   APP          §8  App.{running,need_resize,sim_fps}; drives Scene via the
 *                    combine + user events.
 *
 * PER-TICK COMBINE (the one place state advances — main(), §8):
 *
 *     while (sim_accum >= tick_ns)        // PERFORMANCE: fixed timestep
 *         scene_tick()                    //   SIMULATION (time += dt unless paused)
 *     screen_draw() ; screen_present()    // RENDER (whole scene re-derived each frame)
 *     getch() → app_handle_key()          // USER EVENTS — see below
 *
 * Nothing other than scene_tick()→aurora_tick() advances simulation state, and
 * only while not paused. User events (app_handle_key: spc pause, r reseed, t/T
 * theme, [/] Hz) and SIGWINCH resize mutate state on a keypress/signal, once per
 * frame OUTSIDE the accumulator loop — they are NOT part of the tick.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ===================================================================== */
/* §1  CONFIG  -- constants, colour themes, composition fractions        */
/* ===================================================================== */

enum {
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    = 10,
    FPS_UPDATE_MS   = 500,
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Side-view composition (curtains hanging over a horizon — the photo angle).
 * Down the screen the rows run: starry sky, aurora curtains (a bright green
 * lower hem fading UP through cyan to purple tips), a sky gap, then a jagged
 * dark-land silhouette along the bottom. All *_FRAC are fractions of screen
 * height (0 = top).
 */
#define AUR_MAX_COLS      1024  /* per-column scratch capacity              */
#define HORIZON_FRAC      0.82f /* land base: the ridge valleys sit here    */
#define MTN_AMP1          0.10f /* mountain ridge amplitudes (× rows)       */
#define MTN_AMP2          0.05f
#define MTN_FREQ1         0.055f/* ridge frequencies (per column)           */
#define MTN_FREQ2         0.130f
#define HEM_BASE_FRAC     0.54f /* the curtain's bright lower edge floats here*/
#define HEM_RIPPLE_FRAC   0.07f /* how far the hem ripples (× rows)         */
#define HEM_RIPPLE_FREQ   0.10f /* hem ripple frequency (per column)        */
#define HEM_DRIFT_SPEED   0.25f /* hem sideways drift speed                 */
#define CURTAIN_LEN_FRAC  0.34f /* curtain height above the hem (× rows)    */
#define FOLD_FREQ         0.42f /* drapery-fold density (per column)        */
#define FOLD_SPEED        0.30f /* folds drift sideways                     */
#define CURTAIN_TOP       1.15f /* dn beyond this = past the curtain tip → sky */
#define AURORA_FLOOR      0.07f /* curtain brightness below this → sky / stars */

/* Star density: probability that a background cell is a star.
 * A hash of (col,row) provides a deterministic, storage-free star map. */
#define STAR_THRESH  5          /* out of 256: ~2% of background cells    */

/*
 * Aurora colour themes — each ramp is the bright gradient the curtain is tinted
 * from, hem (low index, bright base) → tips (high index). The hue a cell picks
 * is its HEIGHT plus a sideways-rolling colour wave, so colour bands drift.
 * Every ramp is a REAL aurora palette and every entry sits in the bright half
 * of the 256-colour cube so A_DIM cells stay visible. Cycle with t/T:
 *   EMERALD  — classic oxygen-green (the commonest aurora), green → pale mint.
 *   SPECTRUM — high-energy green → cyan → blue → purple → pink fringe.
 *   CRIMSON  — green base climbing to rare high-altitude red-oxygen tips.
 *   VIOLET   — nitrogen-dominated blue → violet → magenta.
 */
#define PAIR_AURORA_BASE  10
#define N_AURORA_RAMP     18
#define N_AURORA_THEMES    4
static const char *const aurora_theme_names[N_AURORA_THEMES] = {
    "EMERALD ", "SPECTRUM", "CRIMSON ", "VIOLET  ",
};
static const short aurora_themes[N_AURORA_THEMES][N_AURORA_RAMP] = {
    {  40,  46,  47,  48,  49,  50,  83,  84,  85, 120, 121, 122, 156, 157, 158, 159, 194, 195 },
    {  46,  47,  48,  49,  50,  51,  45,  39,  75, 111, 147, 141, 135, 171, 177, 207, 213, 219 },
    {  46,  47,  83,  84, 120, 154, 191, 184, 220, 214, 208, 202, 196, 203, 197, 161, 168, 211 },
    {  51,  45,  39,  75,  81, 117, 147, 111, 105,  99, 141, 135, 171, 177, 183, 219, 213, 225 },
};

/* Rolling colour waves — the hue drifts sideways over time. */
#define COLOR_WAVE_FREQ   1.10f  /* horizontal colour-band frequency        */
#define COLOR_WAVE_SPEED  0.35f  /* how fast the colour bands roll sideways  */
#define COLOR_WAVE_AMP    4.0f   /* ± ramp indices the wave shifts the hue   */

#define PAIR_LAND          8     /* dark-slate land silhouette              */

/* ===================================================================== */
/* §2  PERFORMANCE  -- timing primitives (throttle policy in main, §8)   */
/* ===================================================================== */

/* Timing primitives only. The frame cap and the fixed-timestep accumulator
 * that decide how often scene_tick() runs are POLICY, in main (§8). */

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
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  LOGIC  -- pure decisions: no mutation, no I/O                     */
/* ===================================================================== */

/* Pure decisions — no mutation, no I/O, fully determined by their arguments:
 *  • star_at      — is this cell a star? (deterministic (col,row) hash)
 *  • seed_unit    — one seed → many independent pseudo-random values
 *  • mountain_top — screen row of the land ridge at a column
 *  • hem_row      — screen row of the curtain hem at a column
 *  • curtain_intensity — curtain brightness ∈ [0,1] at a point above the hem
 * Nothing in RENDER/EFFECTS can change a LOGIC result. (draw_star/draw_land,
 * which do ncurses I/O, are NOT here — they live in §7.) */

/*
 * star_at — deterministic star presence check.
 *
 * No storage needed: a fast integer hash of (col, row) decides whether a
 * star occupies this cell.  Same result every frame → no flicker.
 */
static bool star_at(int col, int row, char *out_ch, int *out_pair)
{
    unsigned h = (unsigned)(col * 1234597u ^ row * 987659u ^ (col + row * 31));
    h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
    if ((h & 0xFF) >= STAR_THRESH) return false;
    *out_ch   = ((h >> 8) & 3) == 0 ? '*' : ((h >> 8) & 3) == 1 ? '+' : '.';
    *out_pair = ((h >> 10) & 1) ? 3 : 6;   /* yellow or blue */
    return true;
}

/* seed_unit — deterministic pseudo-random float in [0,1) from a seed and a
 * channel index, so one seed fans out into many independent values (phases,
 * amplitudes) that all reroll together when the seed changes (r key). */
static float seed_unit(unsigned seed, int chan)
{
    unsigned h = seed + (unsigned)chan * 0x9e3779b9u;
    h ^= h >> 16; h *= 0x7feb352du;
    h ^= h >> 15; h *= 0x846ca68bu;
    h ^= h >> 16;
    return (float)h / 4294967296.0f;
}

/* mountain_top — screen row of the land ridge at this column. Layered abs-sine
 * octaves make jagged peaks; rows at or below this are dark land. amp1 + the
 * phases come from the seed, so each regen gets a different skyline. */
static int mountain_top(int col, int rows, float amp1, float ph1, float ph2)
{
    float base  = (float)rows * HORIZON_FRAC;
    float ridge = (float)rows * amp1     * fabsf(sinf((float)col * MTN_FREQ1 + ph1))
                + (float)rows * MTN_AMP2 * fabsf(sinf((float)col * MTN_FREQ2 + ph2));
    return (int)(base - ridge);
}

/* hem_row — screen row of the curtain's bright lower edge at this column. It
 * floats at `base`, ripples, and drifts sideways over time; `ph` (from the
 * seed) shifts the ripple so regens differ. */
static float hem_row(int col, float t, int rows, float base, float ph)
{
    float ripple  = sinf((float)col * HEM_RIPPLE_FREQ + t * HEM_DRIFT_SPEED + ph);
    ripple += 0.4f * sinf((float)col * HEM_RIPPLE_FREQ * 2.3f
                          - t * HEM_DRIFT_SPEED * 0.7f + ph);
    return base + (float)rows * HEM_RIPPLE_FRAC * ripple;
}

/* curtain_intensity — brightness ∈ [0,1] of the curtain at a point `d` rows
 * above the hem (dn = d/curtain_len ∈ [0,1], hem→tip). The product of three
 * factors: an organic two-octave SHIMMER, vertical drapery FOLDS, and a linear
 * FADE-UP that makes the bright hem the floor of the curtain. x is the column's
 * horizontal phase; ph_tex / fold_freq are the seed's texture phase + density. */
static float curtain_intensity(float x, int col, float d, float dn, float t,
                               float ph_tex, float fold_freq)
{
    float n1      = sinf(x * 1.5f + t * 0.20f + ph_tex) * cosf(d * 0.25f + t * 0.50f);
    float n2      = cosf(x * 2.3f - t * 0.15f + ph_tex) * sinf(d * 0.40f + t * 0.80f);
    float shimmer = (n1 * 0.6f + n2 * 0.4f) * 0.5f + 0.5f;                /* [0,1] */
    float fold    = 0.55f + 0.45f * sinf((float)col * fold_freq + t * FOLD_SPEED + ph_tex);
    float fade_up = 1.0f - dn; if (fade_up < 0.0f) fade_up = 0.0f;
    return fade_up * fold * (0.55f + 0.45f * shimmer);
}

/* ===================================================================== */
/* §4  SIMULATION  -- advances state (time + seed of the Aurora)         */
/* ===================================================================== */

/* The ONLY writers of simulation state. The aurora is a CLOSED-FORM scene —
 * every cell is a pure function of (col,row,time,seed) — so there is almost no
 * state to advance: aurora_tick adds dt to `time`, aurora_reseed rolls `seed`
 * (which reseeds the skyline, curtain shape and colour phases). Mutates:
 * Aurora.{time,seed}. scene_tick() gates aurora_tick on the pause flag and is
 * the single per-tick entry, called only from main (§8). aurora_init/scene_init
 * set up state; aurora_reseed fires on the r key and is NOT part of the tick.
 * The Scene aggregate also carries paused (a DELAYS flag, §6) and theme (a
 * RENDER choice, §7), both flipped by user events in §8. */

/* ── Aurora ────────────────────────────────────────────────────────────── *
 * The domain object: the closed-form aurora scene, captured in just two values.
 * WHY so small — this is an ANALYTIC display, not a particle or fluid sim: every
 * cell is a pure function f(col, row, time, seed) evaluated fresh each frame
 * (aurora_draw, §7), so nothing needs storing between frames. The two fields are
 * the only inputs that vary — one MOVES the scene, one CHOOSES it.
 *   time  Animation clock in SECONDS, monotonic from 0. aurora_tick adds the
 *         frame dt; the sideways drift and shimmer arise because `time` enters
 *         the sin/cos phases LINEARLY (a sine scrolls as its argument grows).
 *         Single precision is ample — it only feeds trig phase — and main() caps
 *         dt at 100 ms so a stall can't jump the phase discontinuously.
 *   seed  The scene's 32-bit random IDENTITY — which skyline, curtain shape and
 *         colour phases appear. seed_unit() (§3) fans this one number into many
 *         independent values via a bit-mixing hash; aurora_reseed() advances it
 *         (r key) with one LCG step. UNSIGNED for defined modular wraparound.
 *         The initial 0x9E3779B9 is the golden-ratio constant (2^32/φ), a well-
 *         mixed default that gives a stable first view.
 * Refs: Ebert, Musgrave, Peachey, Perlin & Worley, *Texturing & Modeling*
 *       (analytic/procedural fields); Jarzynski & Olano, JCGT 2020 (the seed
 *       bit-mixing hash); Knuth TAOCP vol.2 / Numerical Recipes ch.7 (the
 *       1664525, 1013904223 linear-congruential step in aurora_reseed). */
typedef struct {
    float    time;
    unsigned seed;
} Aurora;

static void aurora_init(Aurora *a)
{
    a->time = 0.0f;
    a->seed = 0x9e3779b9u;   /* fixed first view; r rerolls it */
}

static void aurora_tick(Aurora *a, float dt)
{
    a->time += dt;
}

/* aurora_reseed — r key: roll a new seed (LCG step) so the whole scene —
 * mountains, curtain shape, colours — regenerates into a fresh random aurora. */
static void aurora_reseed(Aurora *a)
{
    a->seed = a->seed * 1664525u + 1013904223u;
}

/* ── Scene ─────────────────────────────────────────────────────────────── *
 * The whole display as a table of contents: the aurora being shown plus how the
 * user drives it. It exists to keep the layers decoupled — scene_tick (the only
 * per-tick writer) and the user-event knobs gather in one struct, yet each
 * function still takes the narrowest field it needs (the step-5 convention), so
 * "everything hangs off Scene" never re-couples SIM to RENDER.
 *   aurora  WHAT is shown — the closed-form scene above; §4 advances/reseeds it.
 *   paused  RUN-STATE — when true, scene_tick skips aurora_tick so `time` (and
 *           hence all motion) freezes while drawing continues. Toggled by spc.
 *           Lives HERE, not in Aurora: it drives the sim, it is not part of the
 *           scene's mathematical identity, so pausing must not touch the domain
 *           object.
 *   theme   HOW IT LOOKS — an index in [0, N_AURORA_THEMES) into aurora_themes[]
 *           (§1): the active colour palette. A RENDER choice, NOT simulation —
 *           the palettes follow real auroral emission lines (atomic-O 557.7 nm
 *           green, O 630 nm red, N2+ 427.8 nm blue/violet). Cycled by t/T,
 *           bound to the colour pairs by apply_aurora_theme(). Kept off Aurora
 *           so recolouring never perturbs the scene.
 * Ref: Chamberlain, *Physics of the Aurora and Airglow* (the emission lines). */
typedef struct {
    Aurora aurora;
    bool   paused;
    int    theme;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    (void)cols; (void)rows;
    memset(s, 0, sizeof *s);   /* paused=false, theme=0 */
    aurora_init(&s->aurora);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)cols; (void)rows;
    if (s->paused) return;
    aurora_tick(&s->aurora, dt);
}

/* ===================================================================== */
/* §5  EFFECTS  -- cosmetic-only state                                   */
/* ===================================================================== */

/* No EFFECTS layer. Nothing cosmetic is stored: the shimmer, drapery folds,
 * star twinkle, rolling colour wave and glow are ALL derived at render time
 * from (col,row,time,seed) — there is no trail / flash / glow buffer. */

/* ===================================================================== */
/* §6  DELAYS  -- pauses, holds, timers                                  */
/* ===================================================================== */

/* No separate layer. The only control is the pause flag (Scene.paused), which
 * early-returns scene_tick (§4) so `time` stops advancing. No timers. */

/* ===================================================================== */
/* §7  RENDER  -- state -> screen (reads only, never mutates sim)        */
/* ===================================================================== */

/* state -> screen. apply_aurora_theme / color_init bind the colour pairs;
 * aurora_draw reads the Aurora (+ the §3 LOGIC helpers) and paints the
 * curtains, land and stars; draw_star plots one twinkling star; the screen_*
 * helpers own the ncurses surface and overlay the HUD. Reads simulation state;
 * writes ONLY the ncurses back buffer + the colour-pair table. The theme it
 * consults is set by a user event (t/T, §8), never here. */

/* apply_aurora_theme — (re)bind the PAIR_AURORA_BASE.. ramp pairs to theme t.
 * Safe at runtime (t/T key); the next refresh shows the new palette. In the
 * 8-colour fallback the ramp collapses to green→cyan→blue→magenta. */
static void apply_aurora_theme(int t)
{
    if (t < 0 || t >= N_AURORA_THEMES) t = 0;
    if (COLORS >= 256) {
        for (int i = 0; i < N_AURORA_RAMP; i++)
            init_pair((short)(PAIR_AURORA_BASE + i), aurora_themes[t][i], COLOR_BLACK);
    } else {
        static const short fb[4] = { COLOR_GREEN, COLOR_CYAN, COLOR_BLUE, COLOR_MAGENTA };
        for (int i = 0; i < N_AURORA_RAMP; i++)
            init_pair((short)(PAIR_AURORA_BASE + i), fb[i * 4 / N_AURORA_RAMP], COLOR_BLACK);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    /* HUD + stars use pairs 3/5/6 (yellow/cyan/blue); the land is PAIR_LAND;
     * the aurora curtain uses the PAIR_AURORA_BASE ramp (set by the theme). */
    if (COLORS >= 256) {
        init_pair(3, 226, COLOR_BLACK);   /* yellow — HUD data / star */
        init_pair(5,  51, COLOR_BLACK);   /* cyan   — HUD title       */
        init_pair(6,  33, COLOR_BLACK);   /* blue   — HUD hint / star */
        init_pair(PAIR_LAND, 238, COLOR_BLACK);   /* dark-slate land */
    } else {
        init_pair(3, COLOR_YELLOW, COLOR_BLACK);
        init_pair(5, COLOR_CYAN,   COLOR_BLACK);
        init_pair(6, COLOR_BLUE,   COLOR_BLACK);
        init_pair(PAIR_LAND, COLOR_BLUE, COLOR_BLACK);
    }
    apply_aurora_theme(0);   /* default theme */
}

/* draw_star — plot the star at (col,row) if any, with a slow twinkle: a hash
 * folded with a coarse time bucket makes a few stars flare A_BOLD now and then,
 * so the field sparkles instead of sitting frozen. */
static void draw_star(WINDOW *w, int col, int row, float t)
{
    char sch; int spair;
    if (!star_at(col, row, &sch, &spair)) return;
    unsigned th = (unsigned)(col * 7u + row * 131u) ^ (unsigned)(int)(t * 1.7f);
    th ^= th >> 13; th *= 0x5bd1e995u; th ^= th >> 15;
    chtype attr = ((th & 15u) == 0u) ? A_BOLD : A_DIM;   /* occasional flare */
    wattron(w, COLOR_PAIR(spair) | attr);
    mvwaddch(w, row, col, (chtype)(unsigned char)sch);
    wattroff(w, COLOR_PAIR(spair) | attr);
}

/* draw_land — plot one cell of the dark land silhouette. */
static void draw_land(WINDOW *w, int row, int col)
{
    wattron(w, COLOR_PAIR(PAIR_LAND));
    mvwaddch(w, row, col, (chtype)(unsigned char)'#');
    wattroff(w, COLOR_PAIR(PAIR_LAND));
}

/* aurora_pair — colour pair for a curtain point: hue runs GREEN at the hem
 * (dn=0) to PURPLE at the tips (dn=1) along the active theme ramp, shifted by a
 * colour wave rolling sideways over time. */
static int aurora_pair(float dn, float x, float t, float ph_col)
{
    float colwave = sinf(x * COLOR_WAVE_FREQ + t * COLOR_WAVE_SPEED + ph_col);
    int   idx = (int)(dn * (float)(N_AURORA_RAMP - 1) + colwave * COLOR_WAVE_AMP + 0.5f);
    if (idx < 0) idx = 0;
    else if (idx >= N_AURORA_RAMP) idx = N_AURORA_RAMP - 1;
    return PAIR_AURORA_BASE + idx;
}

/* curtain_glyph — glyph + attribute for a curtain brightness: faint dots rising
 * to dense vertical strokes, brightest streaks A_BOLD and faintest A_DIM. The
 * thresholds ARE the brightness ramp. */
static char curtain_glyph(float bright, chtype *attr)
{
    *attr = (bright > 0.55f) ? A_BOLD : (bright < 0.15f) ? A_DIM : 0;
    if      (bright < 0.14f) return '.';
    else if (bright < 0.28f) return ':';
    else if (bright < 0.46f) return '|';
    else if (bright < 0.68f) return '!';
    else                     return '#';
}

/*
 * aurora_draw — side view: curtains hang in the sky over a dark horizon.
 *
 * For each column we know the land ridge (mountain_top) and the curtain's
 * bright lower hem (hem_row). Down a column the bands are:
 *   row ≥ ridge        → dark-land silhouette
 *   below the hem      → clear sky (stars)
 *   above the hem      → curtain: brightest GREEN at the hem, fading UP and
 *                        shifting through cyan to PURPLE at the tips; carved
 *                        into vertical folds and washed by rolling colour waves.
 */
static void aurora_draw(const Aurora *a, WINDOW *w, int cols, int rows)
{
    float t  = a->time;
    unsigned sd = a->seed;

    /* Seed-derived phases (reroll the look) and a few parameters (reroll the
     * mood: skyline height, curtain reach, hem position, fold density). */
    const float TWO_PI = 2.0f * (float)M_PI;
    float ph_mtn1 = seed_unit(sd, 0) * TWO_PI;
    float ph_mtn2 = seed_unit(sd, 1) * TWO_PI;
    float ph_hem  = seed_unit(sd, 2) * TWO_PI;
    float ph_tex  = seed_unit(sd, 3) * TWO_PI;   /* shimmer + folds */
    float ph_col  = seed_unit(sd, 4) * TWO_PI;

    float mtn_amp1    = MTN_AMP1 * (0.65f + 0.70f * seed_unit(sd, 5));
    float hem_base    = (float)rows * (HEM_BASE_FRAC + (seed_unit(sd, 6) - 0.5f) * 0.08f);
    float curtain_len = (float)rows * CURTAIN_LEN_FRAC * (0.80f + 0.40f * seed_unit(sd, 7));
    float fold_freq   = FOLD_FREQ * (0.70f + 0.60f * seed_unit(sd, 8));
    if (curtain_len < 1.0f) curtain_len = 1.0f;

    /* Per-column scratch: ridge + hem depend only on column (and time), so
     * compute them once instead of once per cell. */
    int   ridge[AUR_MAX_COLS];
    float hem[AUR_MAX_COLS];
    int   ncol = cols < AUR_MAX_COLS ? cols : AUR_MAX_COLS;
    for (int c = 0; c < ncol; c++) {
        ridge[c] = mountain_top(c, rows, mtn_amp1, ph_mtn1, ph_mtn2);
        hem[c]   = hem_row(c, t, rows, hem_base, ph_hem);
    }

    for (int row = 1; row < rows - 1; row++) {
        for (int col = 0; col < ncol; col++) {

            /* Down a column: land at the bottom, then sky, with the curtain
             * hanging in a band above its hem. Resolve which band we're in. */
            if (row >= ridge[col]) { draw_land(w, row, col); continue; }

            float above_hem = hem[col] - (float)row;    /* >0 = up into curtain */
            if (above_hem < 0.0f)        { draw_star(w, col, row, t); continue; }
            float dn = above_hem / curtain_len;         /* 0 at hem → 1 at tip  */
            if (dn > CURTAIN_TOP)        { draw_star(w, col, row, t); continue; }

            float x = (float)col / (float)cols * 2.0f * (float)M_PI;   /* phase */
            float bright = curtain_intensity(x, col, above_hem, dn, t, ph_tex, fold_freq);
            if (bright < AURORA_FLOOR)   { draw_star(w, col, row, t); continue; }

            /* Curtain cell: colour by height (green hem → purple tip), glyph by
             * brightness. */
            int    pair = aurora_pair(dn, x, t, ph_col);
            chtype attr;
            char   ch = curtain_glyph(bright, &attr);
            wattron(w, COLOR_PAIR(pair) | attr);
            mvwaddch(w, row, col, (chtype)(unsigned char)ch);
            wattroff(w, COLOR_PAIR(pair) | attr);
        }
    }
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    aurora_draw(&s->aurora, w, cols, rows);
}

/* ── Screen ────────────────────────────────────────────────────────────── *
 * The terminal viewport: its current size in character cells. Pure presentation
 * geometry — it holds NO simulation state, so resizing it can never disturb the
 * scene. Read at startup and re-read on every SIGWINCH (main, §8). aurora_draw
 * samples the closed-form scene across these cells, and the HUD is overlaid on
 * row 0 and row rows-1.
 *   cols  width  in cells — valid x is 0..cols-1.
 *   rows  height in cells — valid y is 0..rows-1. */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

/* Fill a HUD row with spaces so it reads as a clean bar over the aurora
 * (the animation is drawn first; the bar then covers that row). */
static void hud_clear_row(int row, int cols)
{
    for (int x = 0; x < cols; x++) mvaddch(row, x, ' ');
}

/*
 * screen_draw — draw the aurora, then overlay a two-line HUD:
 *   top    (row 0)      DATA    — title (left) + fps / Hz / run-state (right);
 *   bottom (row rows-1) ACTIONS — the key legend.
 * Each bar is filled so the aurora never shows through, the fps block is held
 * clear of the title (no overlap), and every field is mvprintw-clipped at the
 * screen edge so a narrow terminal can't overflow.
 */
static void screen_draw(const Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    int cols = s->cols;

    /* ── Top row 0: title (left) + fps / Hz / run-state (right). ── */
    hud_clear_row(0, cols);

    char lbuf[64];
    int  th = sc->theme;
    snprintf(lbuf, sizeof lbuf, " AURORA BOREALIS   theme:%s %d/%d ",
             aurora_theme_names[th], th + 1, N_AURORA_THEMES);
    int llen = (int)strlen(lbuf);
    attron(COLOR_PAIR(5) | A_BOLD);                 /* cyan title + theme */
    mvprintw(0, 1, "%s", lbuf);
    attroff(COLOR_PAIR(5) | A_BOLD);

    char buf[64];
    snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz  %s ",
             fps, sim_fps, sc->paused ? "PAUSED" : "");
    int hx = cols - (int)strlen(buf);
    if (hx < llen + 2) hx = llen + 2;               /* never collide with title */
    attron(COLOR_PAIR(3) | A_BOLD);                 /* yellow data */
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    /* ── Bottom row: the key legend (actions), bright. ── */
    int brow = s->rows - 1;
    hud_clear_row(brow, cols);
    attron(COLOR_PAIR(6) | A_BOLD);                 /* bright blue hint */
    mvprintw(brow, 1, "q:quit   r:regen   t/T:theme   spc:pause   [/]:Hz");
    attroff(COLOR_PAIR(6) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  APP  -- events + per-tick combine + main loop                     */
/* ===================================================================== */

/* Owns the App aggregate, signal flags, the user-event handler and the main
 * loop. main() is the ONE place that combines the layers per tick: PERFORMANCE
 * gates SIMULATION (scene_tick) -> RENDER (screen_draw -> screen_present) ->
 * input. app_handle_key() mutates state on USER EVENTS (spc pause, r reseed,
 * t/T theme, [/] Hz) and SIGWINCH resize is handled in the loop — both OUTSIDE
 * the tick. */

/* ── App ───────────────────────────────────────────────────────────────── *
 * The process-level aggregate: everything one running instance owns. Not a
 * domain concept — it bundles the simulated display, the viewport, the tick-rate
 * knob and the two signal flags so main()'s fixed-timestep loop has a single
 * handle to thread through.
 *   scene        the display being simulated + its user knobs (§4/§6/§7).
 *   screen       the terminal viewport (§7).
 *   sim_fps      simulation tick rate in Hz, clamped to [SIM_FPS_MIN,
 *                SIM_FPS_MAX]. Sets how often scene_tick runs inside the
 *                accumulator — i.e. the temporal RESOLUTION of the clock, NOT
 *                the drift speed (that is fixed in the formula). Adjusted by [/].
 *   running      loop continues while non-zero; a 0 ends it. Written from the
 *                SIGINT/SIGTERM handlers, hence volatile sig_atomic_t — the only
 *                type the C standard guarantees is safe to read/write from a
 *                signal handler (atomic, no tearing).
 *   need_resize  set to 1 by the SIGWINCH handler; the loop re-reads the
 *                terminal size, rebuilds Screen and clears the flag. Same
 *                signal-safety reason for the type.
 * Ref: Fiedler, "Fix Your Timestep!" — the accumulator loop main() runs. */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': app->scene.paused = !app->scene.paused; break;
    case 'r': case 'R': aurora_reseed(&app->scene.aurora); break;
    case 't':
        app->scene.theme = (app->scene.theme + 1) % N_AURORA_THEMES;
        apply_aurora_theme(app->scene.theme);
        break;
    case 'T':
        app->scene.theme =
            (app->scene.theme + N_AURORA_THEMES - 1) % N_AURORA_THEMES;
        apply_aurora_theme(app->scene.theme);
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
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {
        if (app->need_resize) {
            endwin(); refresh();
            getmaxyx(stdscr, app->screen.rows, app->screen.cols);
            app->need_resize = 0;
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
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        float alpha = (float)sim_accum / (float)tick_ns;

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

        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha, dt_sec);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
