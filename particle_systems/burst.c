/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * burst.c  —  ncurses random ASCII burst field
 *
 * Effect: bursts appear at random screen positions with no rocket.
 * Each burst explodes outward with mixed-color ASCII particles that
 * slow down and fade.  Multiple bursts overlap at different ages.
 * Scorch marks accumulate where bursts land.
 *
 * Keys:
 *   q / ESC   quit
 *   ]  [      speed up / slow down
 *   =  -      more / fewer simultaneous bursts
 *   r         clear scorch marks
 *   t  T      next / previous theme  (rainbow / fire / ice / neon / toxic / gold)
 *   d  D      next / previous debug overlay  (normal / waves / velocity / fuse)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra burst.c -o burst -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config   — every tunable constant
 *   §2  clock    — monotonic nanosecond clock + sleep
 *   §3  color    — 7 vivid hue pairs + dedicated HUD/HINT pairs
 *   §4  particle — one flying ASCII fragment
 *   §5  burst    — one explosion: particle pool + state + tick + draw
 *   §6  field    — burst pool + scorch layer + tick + draw
 *   §7  screen   — single stdscr, ncurses internal double buffer + HUD
 *   §8  app      — dt loop, input, resize, cleanup
 *
 * For Phase-3 readers: the prose blocks above this section list
 * (CONCEPTS + MENTAL MODEL) and below it (HOW TO READ THIS FILE +
 * GUIDED TUTORIAL) are the textbook layer.  The simulation itself is
 * the code from §4 onwards.
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * READING ORDER
 *   1. CONCEPTS + MENTAL MODEL (below) — the algorithm in plain English.
 *   2. GUIDED TUTORIAL (below) — 7 mini-lessons that build the burst
 *      system from "what is a particle" up to the FSM-driven pool.
 *   3. §1 config — constants you'll tweak when experimenting.
 *   4. §4 particle — one spark: state + tick + render.
 *   5. §5 burst — pool of sparks + FSM (IDLE → FLASH → LIVE → IDLE).
 *   6. §6 field — pool of bursts + scorch grid persistence.
 *   7. §7 screen — HUD + double-buffer flush.
 *   8. §8 app — fixed-step loop + key dispatch.
 *
 * NAMING
 *   parts[]    spark pool inside a Burst (PARTICLES = 48 entries)
 *   bursts[]   burst pool inside a Field (BURSTS_MAX = 16 entries)
 *   scorch[]   persistent char grid: one stamp per cell where a burst died
 *   fuse       countdown to next ignition (frames) — keeps bursts alternating
 *   wave       0..3 — staggered spawn ring, used to compute per-spark delay
 *   rx, ry     spark position in PIXEL space (sub-cell precision)
 *   ASPECT     2.0 — horizontal stretch so circular motion reads as circular
 *
 * BACKGROUND ASSUMED
 *   • Plain C, fixed-size arrays, no malloc in hot path.
 *   • Finite-state machines at a "traffic light" level.
 *   • Object-pool pattern (reuse fixed slots, never allocate per event).
 *   • Basic 2-D polar emission (radial spread from a centre point).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Particle burst system with scorch mark persistence.
 *                  Each burst emits N_PARTICLES sparks in random directions;
 *                  each spark has velocity, colour, and lifetime.  Scorch marks
 *                  are written to a separate grid that persists across bursts.
 *
 * Physics        : Spark trajectory: explicit Euler integration — pos += vel×dt,
 *                  vel *= DRAG (per-step speed retention).  Gravity added to vy
 *                  each tick: vy += GRAVITY×dt.  Spark fades (alpha decreases)
 *                  as lifetime runs down.
 *
 * Data-structure : Burst pool — fixed array of MAX_BURSTS burst structs, each
 *                  owning N_PARTICLES spark structs.  Active bursts processed
 *                  each tick; inactive slots reused (object pool pattern).
 *                  Scorch grid: 2-D char array persisting brightest char hit
 *                  at each cell — cleared on 'r' key.
 *
 * Rendering      : Sparks rendered at decreasing brightness as they age.
 *                  Scorch marks use dim attribute.  Multiple bursts overlap
 *                  at different ages, creating layered explosion fields.
 *
 * Performance    : O(active_bursts × PARTICLES) per tick — at MAX 16 bursts
 *                  × 48 particles = 768 spark updates/frame, trivial.
 *                  Scorch grid is cols×rows bytes, swept once per frame.
 *                  No per-tick allocation; everything is pooled.
 *
 * References     :
 *   Reeves, "Particle Systems — A Technique for Modeling a Class of Fuzzy
 *     Objects", SIGGRAPH 1983 (the canonical particle-system paper)
 *   Reynolds, "Steering Behaviors For Autonomous Characters", GDC 1999
 *     (for the inverse-square decay and steering-vector intuition)
 *   ../animation/easing.c — easing curves applied per particle
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * No rockets, no rising trajectory — just bursts that pop into existence
 * at random screen cells, throw 48 ASCII shards radially outward, and die
 * within ~22 ticks. Each burst is a finite-state machine (IDLE → FLASH →
 * LIVE → IDLE). When a burst fades, it stamps a permanent '.' at its
 * centre on a separate scorch grid, so the screen accumulates evidence of
 * past bursts even after sparks vanish.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a fireworks finale where every shell skips the rocket and
 * teleports to its detonation altitude. The sky is the terminal; the
 * scorch grid is an underexposed photograph of all impacts so far. Each
 * burst is staged in 4 "waves" (ring-after-ring delay) so a single
 * explosion looks like an expanding shockwave rather than a single ring.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Field owns up to BURSTS_MAX (16) Burst slots, plus a cols×rows
 *     scorch[] char grid. Active bursts process; idle bursts count down a
 *     fuse and re-ignite.
 *  2. burst_ignite: pick (cx,cy) inside the screen. For each of 48
 *     particles: angle = 2π·i/48 + jitter, speed in [1.8, 4.6], wave =
 *     i%4, delay = wave·MAX_DELAY/(waves-1). State → FLASH.
 *  3. FLASH frame draws a bright '*' surrounded by '+' as the impact, then
 *     transitions to LIVE next tick.
 *  4. Each LIVE tick (particle_tick): if delay>0, decrement and skip;
 *     else vx,vy *= 0.82 (drag), rx += vx, ry += vy, life -= decay. The
 *     particle is dead when life≤0 or it leaves the screen.
 *  5. ASPECT=2.0 stretches horizontal travel so circular motion in physics
 *     reads as circular on screen (cells are taller than wide).
 *  6. When all 48 die or BURST_TICKS hit, scorch_cb stamps '.' at (cx,cy);
 *     fuse resets to 8 + rand()%20; state → IDLE.
 *  7. field_draw paints scorch first (DIM orange), then bursts on top.
 *
 * KEY FORMULAS
 * ────────────
 *  vx,vy   = cos(a)·s, sin(a)·s        radial fan-out from centre
 *  v *= 0.82                            multiplicative drag (≈18%/tick loss)
 *  life -= decay (~0.05–0.09)           ~12–20 ticks of visible life
 *  screen_x = cx + rx·ASPECT            horizontal aspect compensation
 *  wave_delay = wave·MAX_DELAY/(N-1)    staggered shell rings
 *  bold gate: life > 0.65               brightest only in first third
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Burst centre clamps to (2 .. cols-4, 1 .. rows-2) so the FLASH '+'
 *    cross doesn't index off-screen.
 *  • Drag *= 0.82 each tick → after 22 ticks v has decayed to ~1.5% of
 *    original; longer BURST_TICKS won't extend visible motion noticeably.
 *  • Scorch grid is char-typed; only one stamp per cell, so dense bursting
 *    will overwrite older marks invisibly. Calling 'r' (reset) reallocs
 *    via field_free + field_init — old scorch lost.
 *  • Wave count divides into PARTICLES; if PARTICLES isn't a multiple of
 *    waves(=4), the last wave gets fewer particles. Currently 48/4 = 12
 *    each, exact.
 *  • Adding a burst with '+' inserts directly into the next slot WITHOUT
 *    seeding parts[]; first ignition cycle still works because burst_tick
 *    calls burst_ignite when fuse runs out.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press '-' to drop to 1 burst; you should see exactly one detonation
 *    at a time, with a clear FLASH (yellow '*+++') frame before the
 *    coloured shrapnel appears.
 *  • Particle motion should be visibly circular, not oval — if it looks
 *    flat, ASPECT is wrong (should be 2.0 for default 8×16 cell).
 *  • After 30 seconds of running, scorch dots should pepper the screen;
 *    'r' must wipe them instantly.
 *  • Slowing to SIM_FPS_MIN (5) makes the wave-delay visible — you'll see
 *    rings 0,1,2,3 emerge in turn.
 *  • Each spark glyph should remain stable for its lifetime (k_syms is
 *    chosen once at spawn, never re-rolled).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 * Seven mini-lessons.  Read them in order; each ends with the pseudocode
 * that maps onto a real function below.
 *
 * ─── 1.  What is a particle?  ────────────────────────────────────────── *
 *   The smallest unit here is a Particle struct:
 *       float rx, ry      sub-cell position
 *       float vx, vy      velocity (cells/tick × ASPECT)
 *       float life        1.0 = freshly spawned, 0.0 = dead
 *       char  glyph       chosen at spawn, never re-rolled
 *       Hue   hue         colour pair, also fixed at spawn
 *   No "alive" flag — life ≤ 0 means dead.  No allocation: 48 of them sit
 *   in a fixed array inside every Burst.
 *
 *       Particle ≡ { pos, vel, life, glyph, hue }
 *
 * ─── 2.  Polar emission (the radial fan-out)  ────────────────────────── *
 *   To send 48 sparks outward from (cx, cy):
 *       for i in 0..47:
 *           angle = 2π · i / 48 + jitter(±0.06)
 *           speed = uniform(1.8, 4.6)
 *           parts[i].vx = cos(angle) · speed
 *           parts[i].vy = sin(angle) · speed · ASPECT_FIX
 *   The jitter prevents 48 perfectly evenly-spaced lines (which looks
 *   mathematical, not organic).  The speed range gives an irregular ring.
 *
 *                    ↖  ↑  ↗
 *                    ←  ●  →     ← cx, cy
 *                    ↙  ↓  ↘
 *
 * ─── 3.  The Burst FSM  ──────────────────────────────────────────────── *
 *   Each burst slot is in one of three states:
 *
 *       IDLE   ──fuse hits 0──▶  FLASH
 *       FLASH  ──one tick────▶   LIVE
 *       LIVE   ──all dead────▶   IDLE  (and re-arms fuse)
 *
 *   IDLE  ticks down a fuse and sits invisible.
 *   FLASH is exactly ONE tick: draw bright '*' surrounded by '+' at (cx,cy).
 *          This is the impact frame — the human eye reads it as the
 *          detonation flash before the shrapnel.
 *   LIVE  ticks every particle: drag, gravity, life decay; renders sparks.
 *
 *   The FSM keeps each burst slot independent — 16 bursts at different
 *   states all coexist in the same pool with zero coordination.
 *
 * ─── 4.  Wave delay (turning a ring into a shockwave)  ───────────────── *
 *   A single ring of 48 sparks looks like one cartoon explosion.  Split
 *   them into 4 WAVES of 12, each delayed by a different number of ticks:
 *       wave  = i % 4
 *       delay = wave · MAX_DELAY / (waves − 1)
 *   Now you see ring 0 first, ring 1 a few ticks later, ring 2 later
 *   still — an expanding shockwave instead of a single flash.
 *
 *       t=0:  ●               ring 0 (wave 0)
 *       t=2:  ○●○             ring 1 (wave 1) appears, wave 0 expanding
 *       t=4:  ◌○●○◌           ring 2 (wave 2) appears
 *       t=6:  ·◌○●○◌·         all four waves visible
 *
 * ─── 5.  Aspect compensation (cells aren't square)  ──────────────────── *
 *   Terminal cells are roughly 2× taller than wide (an 8×16 pixel font).
 *   So a "circular" motion in cell-space draws as an oval on screen.
 *   Fix at draw time, not at physics time:
 *       screen_x = cx + rx · ASPECT      (with ASPECT = 2.0)
 *       screen_y = cy + ry · 1.0
 *   Physics stays simple (circles are circles); rendering compensates once.
 *
 * ─── 6.  Multiplicative drag (why *=0.82 looks natural)  ─────────────── *
 *   Each tick, multiply every spark's velocity by DRAG = 0.82.  Why
 *   multiplicative instead of subtractive?
 *       additive   v -= 0.1     → fast sparks coast, slow sparks stop suddenly
 *       multiplicative v *= 0.82 → all sparks lose 18% per tick proportionally
 *   The multiplicative form gives exponential decay, which matches how real
 *   air resistance scales with speed (Stokes drag, F ∝ v).  After 22 ticks
 *   the velocity has dropped to 0.82²² ≈ 1.4% of its initial value — the
 *   spark has effectively stopped.
 *
 * ─── 7.  Scorch (the photographic memory layer)  ─────────────────────── *
 *   Sparks die in 22 ticks.  Without persistence the screen would forget
 *   every burst instantly.  Scorch[] is a cols×rows char grid that
 *   accumulates ONE stamp at every burst's centre when it ends.
 *
 *       on burst death:  scorch[cy * cols + cx] = '.';
 *       on draw:         for each non-zero cell, mvaddch in dim orange.
 *
 *   Result: an underexposed photograph of all detonations so far.
 *   'r' wipes the layer (via field re-init).
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN      =  5,
    SIM_FPS_DEFAULT  = 24,
    SIM_FPS_MAX      = 60,
    SIM_FPS_STEP     =  4,

    BURSTS_MIN       =  1,
    BURSTS_DEFAULT   =  5,
    BURSTS_MAX       = 16,

    PARTICLES        = 48,
    BURST_TICKS      = 22,
    FUSE_MIN         =  8,
    FUSE_RANGE       = 20,

    HUD_COLS         = 64,    /* fits fps + spd + burst + [theme] + dbg */
    FPS_UPDATE_MS    = 500,
};

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(fps) (NS_PER_SEC / (fps))

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

typedef enum {
    C_RED     = 1,
    C_ORANGE  = 2,
    C_YELLOW  = 3,
    C_GREEN   = 4,
    C_CYAN    = 5,
    C_BLUE    = 6,
    C_MAGENTA = 7,
    C_COUNT   = 7,
} Hue;

/*
 * HUD/HINT pairs sit AFTER the 7-hue rendering palette, in their own
 * slots so theme changes can't accidentally clobber them.  Both drawn
 * with A_BOLD per CLAUDE.md HUD standard — never A_DIM.
 *   PAIR_HUD  — bright yellow 226 (8-color fallback: COLOR_YELLOW)
 *   PAIR_HINT — bright cyan   51  (8-color fallback: COLOR_CYAN)
 */
#define PAIR_HUD   8
#define PAIR_HINT  9

/*
 * Theme = a single mapping from the 7 hue slots to specific 256-color
 * indices (with an 8-color fallback).  Every theme reassigns the SAME
 * seven pair IDs (1..7), so existing sparks and scorch automatically
 * pick up the new colours on the next mvaddch — no needs_clear flag.
 *
 * Slot order is by Hue enum: [C_RED-1] .. [C_MAGENTA-1].
 *
 *   rainbow  — the original 7 distinct hues (variety)
 *   fire     — reds → oranges → yellows → white-hot
 *   ice      — deep blue → cyan → white
 *   neon     — magenta / pink / yellow / cyan (cyberpunk)
 *   toxic    — green → yellow-green → yellow
 *   gold     — brown → ochre → gold → cream
 */
typedef struct {
    const char *name;
    int         fg256[C_COUNT];
    int         fg8  [C_COUNT];
} BurstTheme;

#define THEME_COUNT  6

static const BurstTheme k_themes[THEME_COUNT] = {
    { "rainbow",
      { 196, 208, 226,  46,  51,  33, 201 },
      { COLOR_RED,  COLOR_YELLOW, COLOR_YELLOW, COLOR_GREEN,
        COLOR_CYAN, COLOR_BLUE,   COLOR_MAGENTA } },
    { "fire",
      { 196, 202, 208, 214, 220, 226, 231 },
      { COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_YELLOW,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE } },
    { "ice",
      {  21,  27,  33,  39,  45,  51, 195 },
      { COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
        COLOR_CYAN, COLOR_CYAN, COLOR_WHITE } },
    { "neon",
      { 201, 207, 213, 159, 226, 195, 51 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN,
        COLOR_YELLOW,  COLOR_CYAN,    COLOR_CYAN } },
    { "toxic",
      {  28,  40,  46, 154, 190, 226, 220 },
      { COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_YELLOW,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW } },
    { "gold",
      { 130, 136, 178, 214, 220, 226, 230 },
      { COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
        COLOR_YELLOW, COLOR_WHITE,  COLOR_WHITE } },
};

/*
 * theme_apply — bind the 7 hue pair IDs to the chosen theme's colours.
 * Called once at startup and again every 't'/'T' keypress.  HUD/HINT
 * pairs sit outside this loop (slots 8/9) so they stay stable.
 */
static void theme_apply(int theme)
{
    const BurstTheme *th = &k_themes[theme];
    for (int i = 0; i < C_COUNT; i++) {
        int slot = i + 1;        /* C_RED=1 .. C_MAGENTA=7 */
        if (COLORS >= 256)
            init_pair(slot, th->fg256[i], COLOR_BLACK);
        else
            init_pair(slot, th->fg8[i],   COLOR_BLACK);
    }
}

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    theme_apply(theme);

    /* HUD pairs are theme-independent — init ONCE, theme_apply leaves them alone. */
    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);

    /* Debug overlay pairs (pairs 10..13) — also theme-independent.
     * Used only by debug rendering modes; reserved bright distinct hues. */
    if (COLORS >= 256) {
        init_pair(10, 196, -1);   /* wave 0: red       */
        init_pair(11, 226, -1);   /* wave 1: yellow    */
        init_pair(12,  46, -1);   /* wave 2: green     */
        init_pair(13,  51, -1);   /* wave 3: cyan      */
    } else {
        init_pair(10, COLOR_RED,    -1);
        init_pair(11, COLOR_YELLOW, -1);
        init_pair(12, COLOR_GREEN,  -1);
        init_pair(13, COLOR_CYAN,   -1);
    }
}

static Hue hue_rand(void) { return (Hue)(1 + rand() % C_COUNT); }

/* ===================================================================== */
/* §3b debug overlays — turn the simulation into a learning instrument   */
/* ===================================================================== */

/*
 * The debug overlay system lets the user SEE intermediate state that
 * the normal render hides.  Each mode replaces parts of the render
 * with diagnostic markup; cycling through them is the fastest way to
 * understand which knob in §1 controls which visible effect.
 *
 * Per CLAUDE.md Phase-3 recipe: "The debug helper's source IS part of
 * the lesson."  So each mode is designed to make ONE concept visible:
 *
 *   DBG_NORMAL    — production rendering (the artistic view)
 *
 *   DBG_WAVES     — colour each spark by its wave index (0..3).
 *                   Makes the staggered emission VISIBLE.  You should
 *                   see four concentric rings appear in sequence —
 *                   red first, then yellow, green, cyan.  Slow the sim
 *                   with '[' and watch wave 0 expand before wave 1 fires.
 *
 *   DBG_VELOCITY  — replace each spark's glyph with an arrow showing
 *                   its current velocity direction.  Drag is visible as
 *                   the arrows becoming "noisier" — frame to frame each
 *                   spark's direction stays the same, but its SPEED falls,
 *                   so over time the arrows move shorter distances.
 *
 *   DBG_FUSE      — write the IDLE fuse countdown (or LIVE age) as a
 *                   number at each burst's centre.  Makes the FSM
 *                   readable: you can see the 16 slots independently
 *                   ticking down, igniting, ageing, re-arming.
 *
 * The g_debug global is the simplest way to thread one int down to
 * particle_draw + burst_draw without changing four function signatures.
 * The trade-off (a static in file scope) is contained: only burst.c
 * reads it; it never affects physics; it never persists across runs.
 */
typedef enum {
    DBG_NORMAL   = 0,
    DBG_WAVES    = 1,
    DBG_VELOCITY = 2,
    DBG_FUSE     = 3,
    DBG_COUNT    = 4,
} DebugMode;

static const char *const k_debug_names[DBG_COUNT] = {
    "normal", "waves", "velocity", "fuse",
};

static DebugMode g_debug = DBG_NORMAL;

/*
 * dir_char — pick an ASCII arrow for a velocity vector.
 *
 * Eight octants, π/4 wide each, centred on the cardinal directions:
 *
 *           ^                  Screen coords: +y is DOWN, so the
 *        \  |  /               diagonals look "rotated" relative to
 *          \|/                 math convention.  Mapping per octant:
 *      < ---+--- >               atan2(vy, vx) in [-π, π], shifted by
 *          /|\                   +π/8 then divided by π/4 → bin 0..7.
 *        /  |  \
 *           v
 *
 * Returns one of: '>'  '\\'  'v'  '/'  '<'  '\\'  '^'  '/'
 */
static char dir_char(float vx, float vy)
{
    static const char k_dirs[8] = { '>', '\\', 'v', '/', '<', '\\', '^', '/' };
    float a = atan2f(vy, vx);
    if (a < 0.0f) a += 2.0f * (float)M_PI;
    int idx = (int)((a + (float)M_PI / 8.0f) / ((float)M_PI / 4.0f)) & 7;
    return k_dirs[idx];
}

/* Wave index → debug-palette colour pair (10..13). */
static int wave_pair(int wave) { return 10 + (wave & 3); }

/* ===================================================================== */
/* §4  particle                                                           */
/* ===================================================================== */

/*
 * §4 PREAMBLE — one flying ASCII fragment
 * ────────────────────────────────────────
 *
 * The ATOM of the simulation.  A Particle is everything the renderer
 * needs to draw a single spark for its short life: position, velocity,
 * remaining lifetime, a fixed glyph, and a fixed colour.
 *
 * Three operations:
 *   particle_spawn  — fill the struct once, at burst ignition
 *   particle_tick   — advance one frame: drag, integrate, decay life
 *   particle_draw   — paint at current cell with current life-attribute
 *
 * No malloc; sparks live in fixed-size pools owned by their Burst.
 * Position is in PIXEL space (sub-cell precision); only draw converts
 * back to terminal cells via ASPECT compensation.
 *
 * COORDINATE SYSTEMS USED HERE
 *   (cx, cy)   burst centre — terminal CELLS
 *   (rx, ry)   spark offset from centre — PIXEL space
 *   (vx, vy)   velocity — pixels/tick (vx scales by ASPECT at draw time)
 *   life       1.0 = fresh, 0.0 = dead, ticks down by `decay` each frame
 */

#define ASPECT 2.0f

typedef struct {
    float cx, cy;
    float rx, ry;
    float vx, vy;
    float life;
    float decay;
    int   delay;
    int   wave;       /* 0..3 — set at spawn, read by DBG_WAVES overlay */
    char  sym;
    Hue   hue;
    bool  alive;
} Particle;

static const char k_syms[] = "*.+o#@%&$!^~-=|/\\:;,`'\"";
#define NSYMS (int)(sizeof k_syms - 1)

/*
 * particle_spawn — fill one Particle struct at burst-ignite time.
 *
 * PURPOSE
 *   Initialise a freshly-spawned spark.  Called PARTICLES (48) times per
 *   burst, never per-tick.  ALL randomness is consumed here so the
 *   inner tick loop has a predictable performance budget.
 *
 * PSEUDOCODE
 *   pos       = (cx, cy)               // burst centre
 *   offset    = (0, 0)                  // grows during tick
 *   velocity  = (cos a, sin a) * speed   // radial outward fan
 *   life      = uniform(0.8, 1.0)       // small jitter so they fade unevenly
 *   decay     = uniform(0.05, 0.09)     // → 11-20 visible ticks
 *   delay     = delay_ticks              // wave-stagger countdown
 *   glyph     = random from k_syms      // FIXED for the spark's lifetime
 *   hue       = random from C_*         // ditto
 *
 * MENTAL MODEL
 *   A spark is a launched projectile with a paint-can attached.  Spawn
 *   loads the gun (pos/vel/life), picks the paint colour, and fires.
 *   After this call, only physics changes — the glyph and colour stay
 *   stable, which lets the eye TRACK individual sparks across frames
 *   instead of seeing RGB noise.
 *
 * INPUTS / OUTPUTS
 *   p           ← Particle to fill (mutated, must be valid memory)
 *   cx, cy      → burst centre in CELL coordinates
 *   angle       → radians from +x axis (math convention, atan2-friendly)
 *   speed       → pixels/tick before ASPECT compensation
 *   delay_ticks → frames to skip before particle_tick starts moving this spark
 *
 * WHY IT EXISTS (vs inlining in burst_ignite)
 *   Forces a clear API boundary: anyone who can call particle_spawn can
 *   make a new "burst" of any shape (line, ring, cone, random) just by
 *   choosing different (angle, speed) sequences in the caller.  burst.c
 *   uses radial-fan; future variants could reuse the same Particle pool.
 */
static void particle_spawn(Particle *p, float cx, float cy,
                            float angle, float speed,
                            int delay_ticks, int wave)
{
    p->cx    = cx;
    p->cy    = cy;
    p->rx    = 0.0f;
    p->ry    = 0.0f;
    p->vx    = cosf(angle) * speed;
    p->vy    = sinf(angle) * speed;
    p->life  = 0.8f + ((float)rand() / RAND_MAX) * 0.2f;
    p->decay = 0.05f + ((float)rand() / RAND_MAX) * 0.04f;
    p->delay = delay_ticks;
    p->wave  = wave;
    p->sym   = k_syms[rand() % NSYMS];
    p->hue   = hue_rand();
    p->alive = true;
}

/*
 * particle_tick — advance one spark one frame.
 *
 * PURPOSE
 *   Apply drag, integrate position, age the life counter, kill if dead
 *   or off-screen.  The whole simulation's per-frame cost lives here:
 *   bursts × particles × this function.
 *
 * PSEUDOCODE
 *   if not alive:        return
 *   if delay > 0:        delay--; return       // wave-stagger gate
 *   velocity *= DRAG                            // multiplicative decay (0.82)
 *   offset   += velocity                        // explicit Euler integrate
 *   life     -= decay                           // age
 *   screen_pos = centre + offset · ASPECT       // for off-screen test
 *   if life <= 0 OR off-screen:  alive = false
 *
 * MENTAL MODEL
 *   Drag * 0.82 is the ONLY force here.  No gravity, no air-resistance
 *   coefficient.  After 22 ticks v has decayed to 0.82^22 ≈ 1.4% of
 *   its initial value, so the spark visibly stops — that's why
 *   BURST_TICKS = 22 was chosen (extending it would show no extra motion).
 *
 *   The off-screen kill is critical: a spark drifting at the edge would
 *   otherwise sit "alive" in the pool forever, wasting a slot.
 *
 * INPUTS / OUTPUTS
 *   p          ← Particle to advance (mutated)
 *   cols, rows → terminal extent in cells (for the off-screen test)
 *
 * UNITS / COORDINATE SYSTEMS
 *   vx, vy: pixels per tick (vx has NOT been ASPECT-compensated yet)
 *   rx, ry: pixel offset from centre
 *   sx, sy: screen-cell position, computed only to clamp
 *
 * WHY IT EXISTS (vs inlining in burst_tick)
 *   burst_tick orchestrates the FSM and counts living sparks.  Keeping
 *   the per-spark physics here means burst_tick stays at FSM level —
 *   no mixing of state-machine and integrator concerns.
 */
static void particle_tick(Particle *p, int cols, int rows)
{
    if (!p->alive) return;
    if (p->delay > 0) { p->delay--; return; }

    p->vx *= 0.82f;
    p->vy *= 0.82f;
    p->rx += p->vx;
    p->ry += p->vy;
    p->life -= p->decay;

    float sx = p->cx + p->rx * ASPECT;
    float sy = p->cy + p->ry;

    if (p->life <= 0.0f
        || sx < 0 || sx >= (float)cols
        || sy < 0 || sy >= (float)rows)
        p->alive = false;
}

/*
 * particle_draw — paint one spark at its current cell.
 *
 * PURPOSE
 *   Convert pixel-space (cx+rx·ASPECT, cy+ry) to terminal cell, choose
 *   bold-vs-normal based on life, and mvaddch the fixed glyph in the
 *   fixed hue.  Idempotent — calling it twice in a row paints the same
 *   character twice with no side effect.
 *
 * PSEUDOCODE
 *   if dead or in delay-window:  return
 *   x = floor(cx + rx · ASPECT)        // pixel → cell, x axis
 *   y = floor(cy + ry)                  // pixel → cell, y axis (no aspect)
 *   if off-screen:  return
 *   attr = COLOR_PAIR(hue)              // chosen at spawn, never changes
 *   if life > 0.65:  attr |= A_BOLD     // brightest only in first ⅓ of life
 *   wattron(w, attr); mvwaddch(w, y, x, sym); wattroff(w, attr)
 *
 * MENTAL MODEL
 *   ASPECT = 2.0 is the WHOLE point of separating pixel-space physics
 *   from cell-space rendering.  In pixel space the spark moves on a
 *   true circle; the ·ASPECT at draw stretches horizontal travel so the
 *   eye sees a circle instead of an oval.  Physics never knows.
 *
 *   The life > 0.65 gate concentrates A_BOLD in the spark's first
 *   third (fresh sparks pop out; old ones fade).
 *
 * INPUTS / OUTPUTS
 *   p          → particle to paint (const — never mutated)
 *   w          → destination WINDOW (typically stdscr, but generic for
 *                future overlay buffers)
 *   cols, rows → cell-space clipping bounds
 *
 * WHY IT EXISTS (vs writing wattron+mvwaddch inline)
 *   Centralising the "spark → cell" mapping means ASPECT lives in one
 *   place.  Change ASPECT here and every spark in every burst reflows;
 *   you can't accidentally have two different aspect compensations in
 *   different draw paths.
 */
static void particle_draw(const Particle *p, WINDOW *w, int cols, int rows)
{
    if (!p->alive || p->delay > 0) return;

    int x = (int)(p->cx + p->rx * ASPECT);
    int y = (int)(p->cy + p->ry);
    if (x < 0 || x >= cols || y < 0 || y >= rows) return;

    /*
     * Debug-mode dispatch.  Each branch makes ONE concept visible.
     * Physics is identical across modes — only the rendering changes.
     */
    chtype  glyph;
    attr_t  attr;
    switch (g_debug) {
    case DBG_WAVES:
        /* Colour by wave (0..3) so staggered emission is legible. */
        glyph = (chtype)(unsigned char)p->sym;
        attr  = COLOR_PAIR(wave_pair(p->wave)) | A_BOLD;
        break;
    case DBG_VELOCITY:
        /* Replace the glyph with an arrow showing velocity direction. */
        glyph = (chtype)(unsigned char)dir_char(p->vx, p->vy);
        attr  = COLOR_PAIR(p->hue) | A_BOLD;
        break;
    case DBG_FUSE:
    case DBG_NORMAL:
    default:
        /* Production view — fixed glyph + fading bold gate at life > 0.65. */
        glyph = (chtype)(unsigned char)p->sym;
        attr  = COLOR_PAIR(p->hue);
        if (p->life > 0.65f) attr |= A_BOLD;
        break;
    }

    wattron(w, attr);
    mvwaddch(w, y, x, glyph);
    wattroff(w, attr);
}

/* ===================================================================== */
/* §5  burst                                                              */
/* ===================================================================== */

/*
 * §5 PREAMBLE — one explosion
 * ────────────────────────────
 *
 * A Burst is a pool of Particles plus a finite-state machine that
 * decides WHEN to ignite, flash, propagate, and re-arm.
 *
 *      IDLE   ──fuse hits 0──▶  FLASH
 *      FLASH  ──one tick────▶   LIVE
 *      LIVE   ──all dead OR ticks≥BURST_TICKS──▶ IDLE  (re-arm fuse)
 *
 * Why three states and not just one "active" flag?  The FLASH state
 * is exactly ONE TICK of bright white-on-yellow at the centre — the
 * detonation flash before the shrapnel.  Without it the burst's
 * appearance is "puff of particles", which doesn't read as an
 * explosion.  With it: "BANG, then debris" — much more legible.
 *
 * The IDLE fuse is what gives the screen its rhythm: 16 bursts each
 * independently counting down (8–28 ticks) means roughly one burst
 * appears every (sim_fps / N_BURSTS / mean_fuse) seconds.  Each slot
 * sleeps and re-ignites without ever knowing the others exist.
 *
 * Three operations:
 *   burst_ignite — pick (cx,cy), spawn 48 sparks with wave delays
 *   burst_tick   — advance the FSM and per-spark physics
 *   burst_draw   — render FLASH cross OR live sparks (state-dependent)
 */

typedef enum {
    BS_IDLE  = 0,
    BS_FLASH = 1,
    BS_LIVE  = 2,
} BurstState;

typedef struct {
    float      cx, cy;
    BurstState state;
    int        ticks;
    int        fuse;
    Particle   parts[PARTICLES];
} Burst;

/*
 * burst_ignite — pick a detonation point and seed all 48 sparks.
 *
 * PURPOSE
 *   Called by burst_tick when the IDLE fuse hits 0.  Chooses a screen
 *   position, sets state=FLASH, and spawns the spark fan with staggered
 *   wave delays.  ALL randomness for this burst lifecycle is here +
 *   particle_spawn.  burst_tick afterwards does only deterministic work.
 *
 * PSEUDOCODE
 *   cx, cy  = random inside (2..cols-4, 1..rows-2)   // clamp away from edge
 *   state   = FLASH
 *   ticks   = 0
 *   for i = 0 .. 47:
 *       angle = (i/48) · 2π  +  jitter(±0.1)         // even fan + texture
 *       speed = uniform(1.8, 4.6)                     // irregular ring
 *       wave  = i mod 4                                // 0..3 → 12 sparks each
 *       delay = wave · MAX_DELAY / (waves-1)           // → 0, 1, 3, 5 ticks
 *       particle_spawn(parts[i], cx, cy, angle, speed, delay)
 *
 * MENTAL MODEL
 *   The (i / 48) · 2π loop creates 48 evenly-spaced angles around the
 *   compass; the ±0.1 jitter prevents 48 perfectly straight lines
 *   (which looks mathematical, not organic).  The wave delay splits
 *   the fan into 4 RINGS that appear at slightly different times —
 *   that's what turns a single ring into an expanding shockwave.
 *
 *   The (cols-4, rows-2) clamp keeps the FLASH cross fully on-screen;
 *   if the centre were at col 0, the '+' to its left would index off
 *   the screen.
 *
 * INPUTS / OUTPUTS
 *   b          ← Burst struct (mutated; all 48 parts[] entries written)
 *   cols, rows → terminal extent for the position clamp
 *
 * UNITS
 *   cx, cy: cells (float for sub-cell consistency with particle pixel space)
 *   angle:  radians [0, 2π) plus small jitter — may exceed 2π, no wrap needed
 *   speed:  pixels per tick
 *
 * WHY IT EXISTS (vs inlining in burst_tick's IDLE→FLASH transition)
 *   Keeps the state-machine transition (in burst_tick) at one line:
 *   "if fuse done, burst_ignite(b, cols, rows)".  All the spawn detail
 *   is here, where you can read it as one unit instead of buried in a
 *   3-state switch.
 */
static void burst_ignite(Burst *b, int cols, int rows)
{
    b->cx    = (float)(2 + rand() % (cols - 4));
    b->cy    = (float)(1 + rand() % (rows - 2));
    b->ticks = 0;
    b->state = BS_FLASH;

    const int MAX_DELAY = 5;
    const int waves     = 4;

    for (int i = 0; i < PARTICLES; i++) {
        float angle = ((float)i / PARTICLES) * 2.0f * (float)M_PI
                      + ((float)rand() / RAND_MAX) * 0.2f;
        float speed = 1.8f + ((float)rand() / RAND_MAX) * 2.8f;
        int   wave  = i % waves;
        int   delay = (wave * MAX_DELAY) / (waves - 1);
        particle_spawn(&b->parts[i], b->cx, b->cy,
                       angle, speed, delay, wave);
    }
}

/*
 * burst_tick — run the burst FSM for one frame.
 *
 * PURPOSE
 *   The state machine.  Per state:
 *     IDLE   countdown the fuse; ignite when it hits 0
 *     FLASH  one-tick transient → LIVE
 *     LIVE   per-particle physics; on completion fire scorch_cb + re-arm
 *
 * PSEUDOCODE
 *   switch state:
 *     IDLE:
 *       if --fuse <= 0:  burst_ignite()             // IDLE → FLASH
 *     FLASH:
 *       state, ticks  =  LIVE, 0                     // FLASH → LIVE
 *     LIVE:
 *       any = false
 *       for each spark: particle_tick(); any |= alive
 *       ticks++
 *       if !any or ticks >= BURST_TICKS:
 *         scorch_cb(cx, cy)                          // stamp memory
 *         fuse  = FUSE_MIN + rand() % FUSE_RANGE     // re-arm
 *         state = IDLE
 *
 * MENTAL MODEL
 *   Each state does the minimum to either stay put or advance.  IDLE
 *   wastes no work on the spark pool.  FLASH is so short (one tick) it
 *   could have been folded into IDLE→LIVE, but keeping it as its own
 *   state means burst_draw can paint a DIFFERENT thing during FLASH
 *   (the bright '*+' cross) than during LIVE (the spark trail) without
 *   any branching in burst_draw beyond the switch.
 *
 *   The scorch_cb callback is the only outward effect — it lets the
 *   field own the scorch grid without burst needing to know about it.
 *
 * INPUTS / OUTPUTS
 *   b          ← Burst (mutated: state/ticks/fuse + every parts[i])
 *   cols, rows → terminal extent (passed to burst_ignite + particle_tick)
 *   scorch_cb  → callback fired ONCE when burst transitions LIVE → IDLE,
 *                with the burst's last cell-space centre
 *   ud         → opaque pointer passed back to scorch_cb (typically the Field)
 *
 * WHY IT EXISTS (vs inlining the FSM in field_tick)
 *   field_tick is a one-line loop over bursts.  All FSM detail lives
 *   here.  This is the canonical "engine vs orchestrator" split:
 *   burst owns its own clock, field merely keeps the slots in sync.
 */
static void burst_tick(Burst *b, int cols, int rows,
                       void (*scorch_cb)(int x, int y, void *ud), void *ud)
{
    switch (b->state) {
    case BS_IDLE:
        if (--b->fuse <= 0) burst_ignite(b, cols, rows);
        break;
    case BS_FLASH:
        b->state = BS_LIVE;
        b->ticks = 0;
        break;
    case BS_LIVE: {
        bool any = false;
        for (int i = 0; i < PARTICLES; i++) {
            particle_tick(&b->parts[i], cols, rows);
            if (b->parts[i].alive) any = true;
        }
        b->ticks++;
        if (!any || b->ticks >= BURST_TICKS) {
            if (scorch_cb) scorch_cb((int)b->cx, (int)b->cy, ud);
            b->fuse  = FUSE_MIN + rand() % FUSE_RANGE;
            b->state = BS_IDLE;
        }
        break;
    }
    }
}

/*
 * burst_draw — paint the burst's visible state.
 *
 * PURPOSE
 *   Render the FLASH cross during BS_FLASH, or the 48 sparks during
 *   BS_LIVE.  Does nothing during BS_IDLE (the burst is dark, fuse
 *   counting down).
 *
 * PSEUDOCODE
 *   if state == FLASH:
 *     mvaddch '*' at (cx, cy)
 *     mvaddch '+' at the four cardinal neighbours (bounds-checked)
 *     in bright yellow + A_BOLD
 *     return
 *   if state == LIVE:
 *     for each spark:  particle_draw()
 *
 * MENTAL MODEL
 *   The cross of '+' around a central '*' is the visual signature
 *   readers parse as "explosion impact".  It's drawn for EXACTLY one
 *   tick (BS_FLASH lifetime), then dissolves into the 48-spark fan
 *   which is the shrapnel.  Two frames, two distinct shapes — the eye
 *   reads them as a sequence.
 *
 *   IDLE state intentionally renders nothing: an active-bursts-counting
 *   slot is invisible while it waits.  Scorch marks (drawn by the
 *   FIELD, not the burst) are what makes IDLE slots visible.
 *
 * INPUTS / OUTPUTS
 *   b          → Burst to read (const)
 *   w          → destination WINDOW
 *   cols, rows → cell-space clipping bounds
 *
 * WHY IT EXISTS (vs a single big render loop in field_draw)
 *   Encapsulation: a Burst knows how to draw itself in each state.
 *   field_draw becomes "for each burst, burst_draw" — easy to add a
 *   new state (e.g. ECHO for residual sparkles) without touching field.
 */
static void burst_draw(const Burst *b, WINDOW *w, int cols, int rows)
{
    int cx = (int)b->cx;
    int cy = (int)b->cy;

    if (b->state == BS_FLASH) {
        if (cx >= 0 && cx < cols && cy >= 0 && cy < rows) {
            wattron(w, COLOR_PAIR(C_YELLOW) | A_BOLD);
            mvwaddch(w, cy, cx, '*');
            if (cx > 0)       mvwaddch(w, cy,   cx-1, '+');
            if (cx < cols-1)  mvwaddch(w, cy,   cx+1, '+');
            if (cy > 0)       mvwaddch(w, cy-1, cx,   '+');
            if (cy < rows-1)  mvwaddch(w, cy+1, cx,   '+');
            wattroff(w, COLOR_PAIR(C_YELLOW) | A_BOLD);
        }
        return;
    }

    if (b->state == BS_LIVE) {
        for (int i = 0; i < PARTICLES; i++)
            particle_draw(&b->parts[i], w, cols, rows);
    }

    /*
     * DBG_FUSE overlay — annotate each burst's centre with its state
     * counter.  IDLE: countdown to ignition.  LIVE: tick number since
     * FLASH (0..BURST_TICKS).  Makes the FSM legible at a glance.
     */
    if (g_debug == DBG_FUSE && cx >= 0 && cx < cols-3 && cy >= 0 && cy < rows) {
        char buf[8];
        if (b->state == BS_IDLE)
            snprintf(buf, sizeof buf, "f%d", b->fuse);
        else if (b->state == BS_LIVE)
            snprintf(buf, sizeof buf, "t%d", b->ticks);
        else
            return;
        wattron(w, COLOR_PAIR(PAIR_HUD) | A_BOLD);
        mvwaddstr(w, cy, cx, buf);
        wattroff(w, COLOR_PAIR(PAIR_HUD) | A_BOLD);
    }
}

/* ===================================================================== */
/* §6  field                                                              */
/* ===================================================================== */

/*
 * §6 PREAMBLE — burst pool + persistent scorch layer
 * ────────────────────────────────────────────────────
 *
 * A Field owns:
 *   - a pool of MAX_BURSTS Bursts (independent FSMs, no cross-talk)
 *   - a cols×rows char grid `scorch[]` that accumulates ONE '.' at
 *     every burst's final centre
 *
 * Without scorch, the screen forgets every burst the moment its 48
 * sparks die — there would be nothing on-screen during IDLE.  Scorch
 * is the photograph-of-events layer: drawn first (dim orange), so
 * live sparks overlay it; persists across burst lifecycles; cleared
 * only by 'r' (which re-inits the whole field).
 *
 * Why a callback (field_scorch_cb)?  burst_tick doesn't know about
 * scorch — it only knows it has a *callback* to fire when LIVE ends.
 * That decoupling means burst.c could be reused in a sky-with-no-
 * memory variant by passing scorch_cb = NULL.
 *
 * Three operations:
 *   field_init / field_free  — alloc / free the scorch grid + slots
 *   field_tick               — one tick per active burst
 *   field_draw               — scorch first, then bursts on top
 */

typedef struct {
    Burst  bursts[BURSTS_MAX];
    char  *scorch;
    int    cols;
    int    rows;
    int    active_bursts;
} Field;

static void field_scorch_cb(int x, int y, void *ud)
{
    Field *f = (Field *)ud;
    if (x >= 0 && x < f->cols && y >= 0 && y < f->rows)
        f->scorch[y * f->cols + x] = '.';
}

static void field_init(Field *f, int cols, int rows, int burst_count)
{
    f->cols          = cols;
    f->rows          = rows;
    f->active_bursts = burst_count;
    f->scorch        = calloc((size_t)(cols * rows), sizeof(char));

    for (int i = 0; i < BURSTS_MAX; i++) {
        memset(&f->bursts[i], 0, sizeof(Burst));
        f->bursts[i].state = BS_IDLE;
        if (i < burst_count)
            f->bursts[i].fuse = i * (FUSE_MIN + FUSE_RANGE / burst_count);
        else
            f->bursts[i].fuse = INT32_MAX / 2;
    }
}

static void field_free(Field *f)
{
    free(f->scorch);
    *f = (Field){0};
}

static void field_tick(Field *f)
{
    for (int i = 0; i < f->active_bursts; i++)
        burst_tick(&f->bursts[i], f->cols, f->rows, field_scorch_cb, f);
}

static void field_draw(const Field *f, WINDOW *w)
{
    int total = f->cols * f->rows;

    wattron(w, COLOR_PAIR(C_ORANGE) | A_DIM);
    for (int i = 0; i < total; i++) {
        if (!f->scorch[i]) continue;
        int y = i / f->cols;
        int x = i % f->cols;
        if (x < f->cols && y < f->rows)
            mvwaddch(w, y, x, (chtype)(unsigned char)f->scorch[i]);
    }
    wattroff(w, COLOR_PAIR(C_ORANGE) | A_DIM);

    for (int i = 0; i < f->active_bursts; i++)
        burst_draw(&f->bursts[i], w, f->cols, f->rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — single stdscr, ncurses' internal double buffer.
 *
 * erase()            — clear newscr (back buffer), no terminal I/O
 * field_draw(stdscr) — write scene into newscr
 * mvprintw / attron  — write HUD into newscr after scene (always on top)
 * wnoutrefresh()     — mark newscr ready, still no terminal I/O
 * doupdate()         — ONE atomic write: diff newscr vs curscr → terminal
 *
 * typeahead(-1) prevents ncurses interrupting output mid-flush to poll
 * stdin, eliminating tearing at high tick rates.
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s, int theme)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(theme);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s)
{
    (void)s;
    endwin();
}

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw_field(Screen *s, const Field *f)
{
    (void)s;     /* s only needed for HUD; field draw uses stdscr directly */
    erase();
    field_draw(f, stdscr);
}

/*
 * HUD layout per CLAUDE.md:
 *   row 0        — fps + sim_fps + burst count in bright bold yellow (top-right)
 *   row rows-1   — every interactive key in bright bold cyan (bottom-left)
 * Both pairs are dedicated (PAIR_HUD / PAIR_HINT), never reused by sparks
 * or scorch, so the colour stays stable across all bursts.
 */
static void screen_draw_hud(Screen *s, double fps, int sim_fps, int bursts,
                            int theme)
{
    char buf[HUD_COLS + 1];
    /* Debug-mode suffix only shown when non-normal; keeps top-right tidy. */
    if (g_debug == DBG_NORMAL) {
        snprintf(buf, sizeof buf, " %5.1f fps  spd:%d  burst:%d  [%s] ",
                 fps, sim_fps, bursts, k_themes[theme].name);
    } else {
        snprintf(buf, sizeof buf,
                 " %5.1f fps  spd:%d  burst:%d  [%s]  dbg:%s ",
                 fps, sim_fps, bursts, k_themes[theme].name,
                 k_debug_names[g_debug]);
    }
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q/ESC:quit  ]/[:speed  +/-:bursts  r:clear-scorch"
             "  t/T:theme  d/D:debug ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Field                 field;
    Screen                screen;
    int                   sim_fps;
    int                   bursts;
    int                   theme;      /* index into k_themes[] */
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    field_free(&app->field);
    screen_resize(&app->screen);
    field_init(&app->field, app->screen.cols, app->screen.rows, app->bursts);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case '=': case '+':
        if (app->bursts < BURSTS_MAX) {
            int i = app->bursts;
            memset(&app->field.bursts[i], 0, sizeof(Burst));
            app->field.bursts[i].state = BS_IDLE;
            app->field.bursts[i].fuse  = 2 + rand() % FUSE_RANGE;
            app->bursts++;
            app->field.active_bursts = app->bursts;
        }
        break;
    case '-':
        if (app->bursts > BURSTS_MIN) {
            app->bursts--;
            app->field.active_bursts = app->bursts;
        }
        break;

    case 'r': case 'R':
        field_free(&app->field);
        field_init(&app->field, app->screen.cols, app->screen.rows,
                   app->bursts);
        break;

    case 't':
        app->theme = (app->theme + 1) % THEME_COUNT;
        theme_apply(app->theme);
        break;
    case 'T':
        app->theme = (app->theme + THEME_COUNT - 1) % THEME_COUNT;
        theme_apply(app->theme);
        break;

    case 'd':
        g_debug = (DebugMode)((g_debug + 1) % DBG_COUNT);
        break;
    case 'D':
        g_debug = (DebugMode)((g_debug + DBG_COUNT - 1) % DBG_COUNT);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)clock_ns());

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    app->bursts  = BURSTS_DEFAULT;
    app->theme   = 0;       /* start on "rainbow" */

    screen_init(&app->screen, app->theme);
    field_init(&app->field, app->screen.cols, app->screen.rows, app->bursts);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── dt ──────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── sim accumulator ─────────────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            field_tick(&app->field);
            sim_accum -= tick_ns;
        }
        float alpha = (float)sim_accum / (float)tick_ns;
        (void)alpha;

        /* ── HUD counter ─────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap (sleep BEFORE render so I/O doesn't drift) ── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── render + HUD ────────────────────────────────────────── */
        screen_draw_field(&app->screen, &app->field);
        screen_draw_hud(&app->screen, fps_display,
                         app->sim_fps, app->bursts, app->theme);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    field_free(&app->field);
    screen_free(&app->screen);
    return 0;
}
