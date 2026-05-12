/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * burst.c — radial ASCII spark bursts.  A field of independent finite-
 *           state machines, each cycling IDLE → FLASH → LIVE → IDLE
 *           at its own random pace.
 *
 * DEMO: Up to 16 burst slots blink on screen at random intervals.
 *       Each slot owns its own fuse (8–28 ticks); when the fuse hits
 *       zero the slot DETONATES.  Detonation is exactly one tick of
 *       a bright '*+' cross (the flash), immediately followed by a
 *       fan of 48 ASCII sparks ('*' '.' '+' '#' …) flying outward
 *       in 4 staggered waves so the burst expands as a multi-ring
 *       shockwave rather than a single instant ring.  Each spark
 *       fades over ~22 ticks under multiplicative drag (×0.82 per
 *       tick); when all 48 are dead the slot re-arms its fuse and
 *       the cycle repeats.  Every burst's final centre is stamped
 *       onto a persistent "scorch" grid drawn in dim orange — an
 *       underexposed photograph of every detonation.  10 themes
 *       (`t`/`T`) recolour the spark palette without disturbing
 *       the silhouette.  Four debug overlays (`d`/`D`) expose the
 *       wave staggering, velocity arrows, and per-slot FSM counters.
 *
 * Study alongside:
 *   particle_systems/comet.c    Comet's ground-impact blast is a
 *                               direct port of THIS file's Burst FSM
 *                               with the angle range restricted to
 *                               the upper hemisphere.  Diff the two
 *                               to see how the algorithm adapts to a
 *                               one-shot (vs cyclic) explosion.
 *   particle_systems/fountain.c Same per-spark physics, but the
 *                               emitter is stationary and the sparks
 *                               launch continuously rather than in
 *                               one explosion.
 *
 * Section map:
 *   §1  config           — sim / burst / particle constants, timing
 *   §2  clock            — monotonic timer + sleep
 *   §3  random + math    — rand_unit / rand_range / clamp_int
 *   §4  themes           — 10 hue palettes + Hue enum + theme_apply
 *   §5  debug overlay    — DebugMode + dir_char + wave_pair
 *   §6  particle         — Particle struct + spawn + tick + draw
 *   §7  burst FSM        — Burst struct + burst_ignite + helpers
 *   §8  burst tick       — per-state advance + complete + re-arm
 *   §9  burst render     — flash cross + fuse overlay + burst_draw
 *   §10 field            — burst pool + scorch grid + init/tick/draw
 *   §11 screen + HUD     — ncurses init + HUD strip
 *   §12 app              — App struct + signal handlers + key dispatch
 *   §13 main             — fixed-step loop
 *
 * Keys:
 *   q / ESC    quit                  r           clear scorch (re-init)
 *   ] / [      sim Hz up / down       + / -       more / fewer bursts
 *   t / T      next / prev theme     d / D       next / prev debug mode
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/burst.c \
 *       -o burst -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * NAMING
 *   Particle          one spark — pos, vel, life, fixed glyph + hue.
 *   Burst             one explosion — 48 Particles + FSM state + fuse.
 *   BurstState        IDLE / FLASH / LIVE — the three FSM states.
 *   Field             pool of up to MAX_BURSTS Bursts + persistent scorch.
 *   BurstTheme        named palette mapping the 7 hue slots to colours.
 *   Hue               C_RED..C_MAGENTA (1..7) — colour pair IDs.
 *   DebugMode         d/D selector: NORMAL / WAVES / VELOCITY / FUSE.
 *   ASPECT            2.0 — horizontal stretch at draw time so a
 *                     pixel-space circle reads as a screen circle.
 *   k_syms            spark glyph alphabet: "*.+o#@..." — fixed at spawn.
 *   k_themes[]        10 named palettes.
 *   wave              0..3 — within-burst staggering index.
 *   scorch[]          cols × rows char grid — '.' at every detonation centre.
 *   FUSE_NEVER        sentinel: an inactive slot's fuse never reaches 0.
 *
 * BACKGROUND ASSUMED
 *   • Finite-state machine — explicit states + transitions, no hidden mode bits.
 *   • Polar emission — (cos θ, sin θ) × speed converts angle + speed to velocity.
 *   • Multiplicative drag — v ← v × 0.82 per tick.  After N ticks, v ≈ v₀ × 0.82^N.
 *   • Explicit Euler integration — pos += vel × dt (dt = 1 tick here).
 *   • Pixel space vs cell space — physics in fractional pixels; ncurses paints
 *     integer cells.  ASPECT bridges the two at draw time.
 *   • ncurses double-buffer (single stdscr).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm     : A FIELD of up to MAX_BURSTS (16) independent Bursts.
 *                 Each Burst is a tiny finite-state machine:
 *
 *                   IDLE   ──fuse hits 0──▶  FLASH
 *                   FLASH  ──exactly 1 tick▶  LIVE
 *                   LIVE   ──ticks ≥ N OR no live sparks──▶ IDLE  (re-arm)
 *
 *                 LIVE state runs explicit-Euler physics on 48 sparks
 *                 with multiplicative drag (×0.82) and per-spark life
 *                 decay (~0.07/tick).  Sparks are spawned in 4 staggered
 *                 waves (0 / 1 / 3 / 5 tick delay) so each burst expands
 *                 as a 4-ring shockwave.  When a burst transitions
 *                 LIVE → IDLE, it fires a scorch callback that stamps
 *                 its final centre onto a persistent grid.
 *
 * Data-structure: Three nested fixed-size pools:
 *                   Field      one (global App.field)
 *                   Bursts     16 per Field — independent FSMs, no cross-talk
 *                   Particles  48 per Burst — pixel-space pos + vel + life
 *                 Plus one cols × rows char[] for the scorch layer.
 *                 No malloc in the hot path; the whole simulation runs
 *                 out of statically-shaped arrays.
 *
 * Performance   : Per tick: O(active_bursts × PARTICLES) for the physics
 *                 plus O(cols × rows) for the scorch sweep.  At 16
 *                 bursts × 48 sparks = 768 particle ticks per frame
 *                 plus ~3 200 scorch cells on a 120×40 terminal = ~4 k
 *                 operations.  60 fps is trivial; mvaddch is the
 *                 bottleneck, not the physics.
 *
 * References    :
 *   Reeves, W. T. — "Particle Systems: A Technique for Modelling a
 *     Class of Fuzzy Objects" (ACM TOG 2(2), 1983).  The per-spark
 *     spawn + integrate + age pattern here is a direct descendant.
 *   particle_systems/comet.c — sibling file that ports this exact FSM
 *     into a one-shot ground-impact blast (upper-hemisphere only).
 *   Inigo Quilez, "Easing functions" — the multiplicative-drag
 *     0.82^N curve is identical to a discrete exponential easing.
 *     https://iquilezles.org/articles/functions/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each burst is a TINY STANDALONE FSM with its own fuse.  Sixteen of
 * them blink at random intervals because each fuse independently
 * counts down 8–28 ticks and ignites on zero.  No central scheduler,
 * no event queue — just sixteen instances of the same 3-state machine
 * sharing the same screen.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a fireworks display.  Many shells in the air at once, each
 * fired at its own moment, each going through the same lifecycle
 * (rising — flash — debris — gone — reloaded).  The shells don't know
 * about each other; the show emerges from their independent timers.
 * burst.c is the same idea simplified to a single position (no rising
 * stage) and turned into an infinite loop.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Field allocates MAX_BURSTS slots; each starts IDLE with a
 *     staggered initial fuse so they don't all ignite on frame 0.
 *  2. Each tick: every active slot runs its FSM:
 *       IDLE   → fuse--; if ≤ 0, burst_ignite
 *       FLASH  → state ← LIVE; ticks ← 0
 *       LIVE   → tick all 48 sparks; if all dead OR ticks ≥ 22:
 *                  scorch_cb(cx, cy); fuse ← rand(8..28); IDLE
 *  3. burst_ignite picks a safe-area centre and spawns 48 sparks.
 *     Each spark gets:
 *       angle  ← (i / 48) × 2π + jitter(±0.1)        (radial fan + texture)
 *       speed  ← uniform(1.8, 4.6)                    (irregular ring)
 *       wave   ← i mod 4                              (0..3, 12 sparks each)
 *       delay  ← wave × 5 / 3                         (→ 0, 1, 3, 5 ticks)
 *  4. Per spark, per tick (only after delay reaches 0):
 *       v   ← v × 0.82                                (multiplicative drag)
 *       pos += v                                       (explicit Euler)
 *       life -= decay (0.05..0.09)
 *       die if life ≤ 0 OR pos off-screen
 *  5. Render: scorch layer (dim orange) first → active bursts on top.
 *     Each burst's draw is state-dependent: FLASH paints the '*+'
 *     cross; LIVE paints the 48-spark fan; IDLE is invisible.
 *  6. The debug overlay (`d`/`D`) replaces parts of the render to
 *     expose wave indices, velocity arrows, or per-slot FSM counters.
 *
 * KEY FORMULAS
 * ────────────
 *  Polar emission:
 *    vx = cos(angle) × speed
 *    vy = sin(angle) × speed
 *
 *  Wave staggering (waves = 4, max_delay = 5):
 *    delay = wave × max_delay / (waves − 1)       →  0, 1, 3, 5 ticks
 *
 *  Multiplicative drag:
 *    v ← v × DRAG_FACTOR        DRAG_FACTOR = 0.82
 *    After N ticks: v ≈ v₀ × 0.82^N
 *    At N = 22:    v ≈ v₀ × 0.014 — effectively stopped.
 *
 *  Pixel → cell at draw time:
 *    cell_x = floor(cx + rx × ASPECT)     ASPECT = 2.0 fixes h-stretch
 *    cell_y = floor(cy + ry)
 *
 *  Spark brightness gate:
 *    A_BOLD if life > FLASH_LIFE_THRESHOLD (0.65)  → fresh third of life
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • FUSE_NEVER for inactive slots.  When the user shrinks the burst
 *    count with '-', slots above active_bursts keep their state but
 *    never tick.  field_tick only loops 0..active_bursts.
 *  • ASPECT consistency.  particle_pixel_to_cell is the ONE place the
 *    pixel→cell mapping happens.  Any other paint code must route
 *    through it or the visual will desync from the bounds check.
 *  • Safe-area clamp at ignite.  The FLASH '*+' cross needs 1 cell of
 *    margin on each side; pick_detonation_centre uses (2..cols-4,
 *    1..rows-2).  Change this and the cross clips on the edges.
 *  • Wave-delay rounding.  With waves=4, max_delay=5: integer division
 *    yields 0, 1, 3, 5 (not 0, 1.67, 3.33, 5).  Waves 0 and 1 fire
 *    close together, then 2, then 3.  Intentional.
 *  • Scorch grid bounds.  field_scorch_cb checks x/y before writing —
 *    without that, an edge-of-screen centre could write past the buffer.
 *  • Theme cycle is free.  theme_apply rebinds the 7 pair IDs in
 *    place; every active spark + the scorch layer pick up the new
 *    colour on the next mvaddch.  No needs_clear, no flicker.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Wave staggering: press d → DBG_WAVES, slow with '[' to ~5 fps,
 *    watch the four rings ignite in sequence (red, yellow, green, cyan).
 *  • Velocity arrows: press d twice → DBG_VELOCITY.  Each spark shows
 *    an arrow; the fan reads as outward radial rays.
 *  • FSM legibility: press d three times → DBG_FUSE.  Each slot shows
 *    "fN" (idle countdown) or "tN" (live tick); you can see all 16
 *    counters ticking independently.
 *  • Theme cycle: t / T cycles 10 palettes without changing the burst
 *    silhouette.  The scorch layer also reflects the active theme's
 *    orange slot.
 *  • One-burst mode: shrink bursts with '-' until only one slot
 *    remains.  Watch its IDLE → FLASH → LIVE → IDLE cycle in isolation.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 * Six lessons in dependency order.  Each ends with the pseudocode or
 * struct that maps onto a real symbol below.
 *
 * ─── 1.  Why FSM, not a bool flag?  ──────────────────────────────────── *
 *   The simplest design — "burst is either live or not" — paints a puff
 *   of particles each tick.  That doesn't read as an EXPLOSION; it
 *   reads as smoke.  Real explosions have a distinct flash before the
 *   debris.
 *
 *   So we split into THREE states:
 *
 *       IDLE   silent, fuse counting down
 *       FLASH  one tick of bright '*+' cross
 *       LIVE   the 48-spark fan flying outward
 *
 *   Each state has its own renderer; the eye reads them as a sequence:
 *   "tick, tick, tick … BANG, then shrapnel."  Three states; three
 *   transitions; everything else (per-spark physics) lives inside LIVE.
 *
 *       state ∈ { IDLE, FLASH, LIVE }
 *
 * ─── 2.  Polar emission — (cos θ, sin θ) × speed  ────────────────────── *
 *   The radial fan is the visual signature of any explosion.  Each
 *   spark gets its own outbound direction; the easiest way is polar:
 *
 *       angle = (i / 48) × 2π        spark i of 48 → angle around circle
 *       speed = uniform(1.8, 4.6)    irregular ring, not a perfect circle
 *       vx    = cos(angle) × speed
 *       vy    = sin(angle) × speed
 *
 *   Add a tiny ±0.1 jitter to angle so the fan doesn't look
 *   mathematically perfect.  Output: 48 vectors evenly distributed
 *   around 360°, each with its own speed and slight angle wiggle.
 *
 *       (vx, vy) = (cos α, sin α) × speed
 *
 * ─── 3.  Wave staggering — ring → shockwave  ─────────────────────────── *
 *   A single instantaneous ring is unimpressive.  Real explosions read
 *   as concentric expanding rings.  We split the 48 sparks into 4
 *   GROUPS by index modulo 4:
 *
 *       wave i ← i mod 4              0..3, 12 sparks per group
 *       delay  ← wave × MAX_DELAY / (waves − 1)
 *
 *   With MAX_DELAY = 5 and waves = 4, integer division gives delays:
 *
 *       wave 0: 0 ticks   (ignites with the flash)
 *       wave 1: 1 tick    (ring appears 1 frame later)
 *       wave 2: 3 ticks   (skip — integer division)
 *       wave 3: 5 ticks   (outermost ring)
 *
 *   While a spark's delay > 0, particle_tick HOLDS it at its starting
 *   cell.  Slow the sim with '[' and watch each ring expand for a
 *   couple of frames before the next one fires.
 *
 *       if delay > 0:  delay--; continue
 *
 * ─── 4.  Pixel space vs cell space (and ASPECT = 2.0)  ───────────────── *
 *   Terminal cells are ~2× taller than they are wide.  A perfect
 *   circle in CELL coordinates renders as a vertical oval.  Two fixes:
 *
 *     (a) Run physics in PIXEL space — square units, true circles.
 *     (b) Stretch horizontally by ASPECT = 2.0 only at draw time.
 *
 *   So a Particle stores:
 *
 *       (cx, cy)   burst centre in CELLS
 *       (rx, ry)   offset from centre in PIXELS
 *       (vx, vy)   velocity in PIXELS / tick
 *
 *   At draw:
 *
 *       cell_x = floor(cx + rx × ASPECT)
 *       cell_y = floor(cy + ry)
 *
 *   The ×ASPECT lives in ONE function (particle_pixel_to_cell).
 *   Every overlay routes through it; the bounds check in particle_tick
 *   also routes through it; the two can never disagree.
 *
 * ─── 5.  Drag + life — two decoupled clocks  ─────────────────────────── *
 *   Real explosions slow down as gas resistance dominates.  Two
 *   mechanisms control the spark's "death":
 *
 *     (a) MULTIPLICATIVE DRAG on velocity:
 *           v ← v × DRAG_FACTOR          DRAG_FACTOR = 0.82
 *         After 22 ticks: v ≈ v₀ × 0.014 — effectively stopped.
 *
 *     (b) LINEAR DECAY on life:
 *           life ← life − decay          decay uniform in [0.05, 0.09]
 *         When life ≤ 0, the spark dies.
 *
 *   Why two mechanisms?  Drag stops the spark in PLACE; life decay
 *   stops it in BRIGHTNESS.  Some sparks coast to a stop and fade over
 *   many frames (low decay); others flame out quickly (high decay).
 *   The mix is what gives the burst its organic texture.
 *
 *       v    *= 0.82
 *       life -= decay
 *       if life ≤ 0:  alive = false
 *
 * ─── 6.  Scorch — memory via callback  ───────────────────────────────── *
 *   Without a memory layer, the screen forgets every burst the moment
 *   its 48 sparks die.  An idle slot is invisible, so during the 8–28-
 *   tick fuse the screen has NOTHING from that burst.
 *
 *   The fix is the scorch grid: a cols × rows char[] that holds '.'
 *   at every burst's final centre.  Drawn first (dim orange), so live
 *   sparks render on top.
 *
 *   Notice the decoupling — burst_tick doesn't know what scorch IS;
 *   it just calls back when LIVE ends:
 *
 *       on burst death:  scorch_cb(cx, cy, user_data)
 *
 *   The Field provides scorch_cb; the Burst could be reused in a
 *   no-memory variant by passing scorch_cb = NULL.  That callback
 *   boundary is the only line of communication between Burst and Field.
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

    PARTICLES        = 48,    /* sparks per burst                          */
    BURST_TICKS      = 22,    /* max LIVE-state duration                   */
    FUSE_MIN         =  8,    /* idle-fuse minimum (ticks)                 */
    FUSE_RANGE       = 20,    /* idle-fuse extra uniform range             */

    BURST_WAVES      =  4,    /* concentric rings inside one burst         */
    BURST_MAX_DELAY  =  5,    /* outermost wave's spawn delay (ticks)      */

    HUD_COLS         = 64,    /* fits fps + spd + burst + [theme] + dbg    */
    FPS_UPDATE_MS    = 500,
};

/*
 * Float constants that read better as #define than as enum.  Grouped by
 * concept and annotated with units / role so the inner loops below stay
 * pure mechanics — every magic number lives here.
 */
#define DRAG_FACTOR              0.82f   /* per-tick velocity retention (≈18% loss) */
#define FLASH_LIFE_THRESHOLD     0.65f   /* sparks bold while life > this           */

#define BURST_ANGLE_JITTER       0.2f    /* radians of extra angle per spark        */
#define BURST_SPEED_MIN          1.8f    /* pixels / tick (lower bound)             */
#define BURST_SPEED_MAX          4.6f    /* pixels / tick (upper bound)             */

#define PARTICLE_LIFE_MIN        0.8f    /* fresh-spawn life (lower bound)          */
#define PARTICLE_LIFE_MAX        1.0f    /* fresh-spawn life (upper bound)          */
#define PARTICLE_DECAY_MIN       0.05f   /* per-tick life decay (lower bound)       */
#define PARTICLE_DECAY_MAX       0.09f   /* per-tick life decay (upper bound)       */

#define FUSE_NEVER               (INT32_MAX / 2)   /* idle slot's fuse: never fires */

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
/* §3  random + math — uniform samples + integer clamp                   */
/* ===================================================================== */

/*
 * Named primitives so the inner loops below read as INTENT — "uniform
 * sample in [lo, hi)" instead of "arithmetic on RAND_MAX".
 *
 *   rand_unit()              uniform float in [0, 1).
 *   rand_range(lo, hi)       uniform float in [lo, hi).
 *   rand_int_below(n)        uniform integer in [0, n).  (n must be ≥ 1)
 *   clamp_int(v, lo, hi)     clamp integer to closed interval.
 */
static inline float rand_unit(void) { return (float)rand() / RAND_MAX; }
static inline float rand_range(float lo, float hi) { return lo + rand_unit() * (hi - lo); }
static inline int   rand_int_below(int n) { return rand() % n; }
static inline int   clamp_int(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ===================================================================== */
/* §4  themes — 10 hue palettes + Hue enum + theme_apply + color_init    */
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
 * Slot order is by Hue enum: [C_RED-1] .. [C_MAGENTA-1].  Each theme
 * paints the 7 hue slots with its own family of colours, so hue_rand()
 * still picks random slots but the visual stays consistent with the
 * theme.  Themes follow the LITERATE doctrine's canonical 10 — same
 * names as in fire.c, comet.c, aafire_port.c for cross-file consistency.
 */
typedef struct {
    const char *name;
    int         fg256[C_COUNT];
    int         fg8  [C_COUNT];
} BurstTheme;

#define THEME_COUNT  10

static const BurstTheme k_themes[THEME_COUNT] = {
    { "matrix",
      {  22,  28,  34,  40,  46,  82, 118 },
      { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
        COLOR_GREEN, COLOR_GREEN, COLOR_WHITE } },
    { "neon",
      { 201, 207, 213, 159, 226, 195,  51 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN,
        COLOR_YELLOW,  COLOR_CYAN,    COLOR_CYAN } },
    { "nova",
      {  52,  88, 124, 160, 196, 208, 220 },
      { COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_RED,
        COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW } },
    { "ocean",
      {  24,  31,  39,  45,  51, 123, 195 },
      { COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
        COLOR_CYAN, COLOR_WHITE, COLOR_WHITE } },
    { "fire",
      { 196, 202, 208, 214, 220, 226, 231 },
      { COLOR_RED,    COLOR_RED,    COLOR_RED,    COLOR_YELLOW,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE } },
    { "toxic",
      {  28,  40,  46, 154, 190, 226, 220 },
      { COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_YELLOW,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW } },
    { "gold",
      { 130, 136, 178, 214, 220, 226, 230 },
      { COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
        COLOR_YELLOW, COLOR_WHITE,  COLOR_WHITE } },
    { "ice",
      {  21,  27,  33,  39,  45,  51, 195 },
      { COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
        COLOR_CYAN, COLOR_CYAN, COLOR_WHITE } },
    { "aurora",
      {  28,  35,  50,  86, 121, 207, 219 },
      { COLOR_GREEN, COLOR_GREEN, COLOR_CYAN, COLOR_CYAN,
        COLOR_CYAN,  COLOR_MAGENTA, COLOR_MAGENTA } },
    { "plasma",
      {  53,  91, 129, 165, 207, 213,  51 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_MAGENTA, COLOR_CYAN,    COLOR_CYAN } },
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
/* §5  debug overlay — turn the simulation into a learning instrument    */
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
/* §6  particle — Particle struct + spawn + tick + draw                   */
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
 * particle_spawn — fill one Particle at burst-ignite time.
 *
 *   pos      = (cx, cy)                   anchor in CELL space
 *   offset   = (0, 0)                     grows during tick
 *   velocity = (cos α, sin α) × speed     polar → Cartesian
 *   life     = uniform(0.8, 1.0)
 *   decay    = uniform(0.05, 0.09)        per-tick life loss
 *   delay    = delay_ticks                wave-stagger countdown
 *   sym, hue = randomly chosen, FIXED for the spark's lifetime
 */
static void particle_spawn(Particle *p, float cx, float cy,
                            float angle, float speed,
                            int delay_ticks, int wave)
{
    /* Anchor — burst centre in CELL space, never moves after spawn. */
    p->cx    = cx;
    p->cy    = cy;

    /* Offset — pixel-space deviation from centre, grows during tick. */
    p->rx    = 0.0f;
    p->ry    = 0.0f;

    /* Velocity — radial outward fan (polar → Cartesian). */
    p->vx    = cosf(angle) * speed;
    p->vy    = sinf(angle) * speed;

    /* Life budget — fresh in [LIFE_MIN, LIFE_MAX), drains by `decay`/tick. */
    p->life  = rand_range(PARTICLE_LIFE_MIN,  PARTICLE_LIFE_MAX);
    p->decay = rand_range(PARTICLE_DECAY_MIN, PARTICLE_DECAY_MAX);

    /* FSM bookkeeping — wave index and per-spark spawn delay. */
    p->delay = delay_ticks;
    p->wave  = wave;

    /* Appearance — glyph and hue fixed at spawn so the eye can track. */
    p->sym   = k_syms[rand_int_below(NSYMS)];
    p->hue   = hue_rand();
    p->alive = true;
}

/*
 * particle_tick — advance one spark one frame.  Tier-3: this function
 * carries the simulation's per-frame physics; every other helper is in
 * service of this one.
 *
 *   Pseudocode:
 *     if not alive:                       return
 *     if delay > 0:    delay--;            return     wave-stagger gate
 *     velocity *= DRAG_FACTOR                          multiplicative drag (0.82)
 *     offset   += velocity                             explicit Euler integrate
 *     life     -= decay                                age the life counter
 *     screen_pos = centre + offset · ASPECT            for off-screen test
 *     if life <= 0 OR off-screen:  alive = false
 *
 *   Inputs / outputs / units:
 *     p              Particle to advance (mutated).
 *     cols, rows     terminal extent in CELLS, for the off-screen test.
 *     vx, vy         pixels per tick (NOT ASPECT-compensated yet).
 *     rx, ry         pixel offset from centre.
 *
 *   Why it exists (vs inlining in burst_tick):
 *   burst_tick orchestrates the FSM and counts living sparks.  Keeping
 *   the per-spark physics here means burst_tick stays at FSM level —
 *   no mixing of state-machine and integrator concerns.
 */
static void particle_tick(Particle *p, int cols, int rows)
{
    /* Gate 1 — dead sparks are inert. */
    if (!p->alive) return;

    /* Gate 2 — wave-stagger delay holds the spark in its starting cell. */
    if (p->delay > 0) { p->delay--; return; }

    /* Step 1 — multiplicative drag (Stokes-like: force ∝ velocity). */
    p->vx *= DRAG_FACTOR;
    p->vy *= DRAG_FACTOR;

    /* Step 2 — explicit Euler integration in pixel space. */
    p->rx += p->vx;
    p->ry += p->vy;

    /* Step 3 — age the life counter. */
    p->life -= p->decay;

    /* Step 4 — die if life is exhausted OR the spark left the screen.
     *           ASPECT compensation applied so the bounds test matches
     *           what particle_draw will actually paint. */
    float screen_x   = p->cx + p->rx * ASPECT;
    float screen_y   = p->cy + p->ry;
    bool  burned_out = (p->life <= 0.0f);
    bool  off_screen = (screen_x < 0.f || screen_x >= (float)cols
                     || screen_y < 0.f || screen_y >= (float)rows);
    if (burned_out || off_screen) p->alive = false;
}

/*
 * particle_pixel_to_cell — collapse pixel-space (cx+rx·ASPECT, cy+ry)
 *                          to a single terminal cell.  ONE place owns
 *                          the ×ASPECT step, so draw + bounds-check
 *                          can never disagree.
 */
static void particle_pixel_to_cell(const Particle *p, int *cell_x, int *cell_y)
{
    *cell_x = (int)(p->cx + p->rx * ASPECT);
    *cell_y = (int)(p->cy + p->ry);
}

/*
 * particle_draw — paint one spark at its current cell.  Tier-3 because
 * the body switches on g_debug to render four different things.
 *
 *   Pseudocode:
 *     if dead or in delay-window: return
 *     cell = particle_pixel_to_cell()
 *     if off-screen:              return
 *     switch g_debug:
 *       WAVES    : glyph=sym; attr = wave-coloured + BOLD
 *       VELOCITY : glyph=arrow(vx,vy); attr = hue + BOLD
 *       FUSE / NORMAL / default :
 *                  glyph=sym; attr = hue + (BOLD if life > 0.65)
 *     paint with mvwaddch
 *
 *   Why it exists (vs inline render in field_draw):
 *   Centralising the "spark → cell" mapping means ASPECT lives in one
 *   place.  Adding a new debug overlay touches ONLY this switch.
 */
static void particle_draw(const Particle *p, WINDOW *w, int cols, int rows)
{
    /* Gates — same suppression rules as particle_tick. */
    if (!p->alive || p->delay > 0) return;

    int cell_x, cell_y;
    particle_pixel_to_cell(p, &cell_x, &cell_y);
    if (cell_x < 0 || cell_x >= cols || cell_y < 0 || cell_y >= rows) return;

    /*
     * Debug-mode dispatch — pick (glyph, attr) per mode.  Physics is
     * identical across modes; only the rendering changes.
     */
    chtype glyph;
    attr_t attr;
    switch (g_debug) {
    case DBG_WAVES:
        /* Colour by wave (0..3) so staggered emission is legible. */
        glyph = (chtype)(unsigned char)p->sym;
        attr  = COLOR_PAIR(wave_pair(p->wave)) | A_BOLD;
        break;
    case DBG_VELOCITY:
        /* Replace glyph with an arrow showing velocity direction. */
        glyph = (chtype)(unsigned char)dir_char(p->vx, p->vy);
        attr  = COLOR_PAIR(p->hue) | A_BOLD;
        break;
    case DBG_FUSE:
    case DBG_NORMAL:
    default: {
        /* Production view — fixed glyph + fading bold gate. */
        bool is_fresh_spark = (p->life > FLASH_LIFE_THRESHOLD);
        glyph = (chtype)(unsigned char)p->sym;
        attr  = COLOR_PAIR(p->hue) | (is_fresh_spark ? A_BOLD : 0);
        break;
    }
    }

    wattron(w, attr);
    mvwaddch(w, cell_y, cell_x, glyph);
    wattroff(w, attr);
}

/* ===================================================================== */
/* §7  burst FSM — Burst struct + state enum + burst_ignite               */
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
/*
 * pick_detonation_centre() — choose a random (cx, cy) inside a safe area.
 *
 *   Safe area excludes a 2-cell margin left/right and 1-cell margin
 *   top/bottom so the FLASH '+' cross drawn by burst_draw fits even at
 *   the screen edge.
 */
static void pick_detonation_centre(int cols, int rows, float *cx, float *cy)
{
    int safe_cols_extent = (cols - 4) > 1 ? (cols - 4) : 1;
    int safe_rows_extent = (rows - 2) > 1 ? (rows - 2) : 1;
    *cx = (float)(2 + rand_int_below(safe_cols_extent));
    *cy = (float)(1 + rand_int_below(safe_rows_extent));
}

/*
 * compute_emission_angle() — even ring spacing + small jitter.
 *
 *   evenly_spaced ← (i / total) × 2π            48 angles around a circle
 *   jitter        ← uniform(0, BURST_ANGLE_JITTER)
 *   return evenly_spaced + jitter               organic, not mathematical
 */
static float compute_emission_angle(int i, int total_particles)
{
    float evenly_spaced = ((float)i / (float)total_particles) * 2.0f * (float)M_PI;
    float jitter        = rand_unit() * BURST_ANGLE_JITTER;
    return evenly_spaced + jitter;
}

/*
 * compute_emission_speed() — uniform in [BURST_SPEED_MIN, BURST_SPEED_MAX).
 *   Speed range makes the burst ring irregular instead of perfectly round.
 */
static float compute_emission_speed(void)
{
    return rand_range(BURST_SPEED_MIN, BURST_SPEED_MAX);
}

/*
 * compute_wave_delay() — staggered spawn delay so the burst expands as
 * a multi-ring shockwave instead of a single instant ring.
 *
 *   delay = wave × MAX_DELAY / (waves - 1)
 *   With waves=4, MAX_DELAY=5 → delays 0, 1 (rounded down), 3, 5 ticks.
 */
static int compute_wave_delay(int wave, int wave_count, int max_delay)
{
    if (wave_count <= 1) return 0;
    return (wave * max_delay) / (wave_count - 1);
}

/*
 * burst_ignite() — pick a detonation point and seed all 48 sparks.
 *
 *   The body is three labelled steps:
 *     Step 1 — pick the detonation centre (safe-area clamp).
 *     Step 2 — enter FLASH state for exactly one tick.
 *     Step 3 — spawn PARTICLES sparks in a wave-staggered radial fan.
 */
static void burst_ignite(Burst *b, int cols, int rows)
{
    /* Step 1 — detonation point. */
    pick_detonation_centre(cols, rows, &b->cx, &b->cy);

    /* Step 2 — enter FLASH state. */
    b->state = BS_FLASH;
    b->ticks = 0;

    /* Step 3 — spawn the radial fan. */
    for (int i = 0; i < PARTICLES; i++) {
        float angle = compute_emission_angle(i, PARTICLES);
        float speed = compute_emission_speed();
        int   wave  = i % BURST_WAVES;
        int   delay = compute_wave_delay(wave, BURST_WAVES, BURST_MAX_DELAY);
        particle_spawn(&b->parts[i], b->cx, b->cy, angle, speed, delay, wave);
    }
}

/* ===================================================================== */
/* §8  burst tick — FSM advance + complete + re-arm                       */
/* ===================================================================== */

/*
 * §8 PREAMBLE — the state machine
 * ───────────────────────────────
 *
 * burst_tick is a switch over BurstState.  Each case does the minimum
 * work to either stay put (IDLE counts down its fuse) or advance to
 * the next state.  The LIVE → IDLE transition is the only one that
 * touches outside state (via scorch_cb).
 *
 *   burst_advance_live_particles  per-spark physics for one frame.
 *   burst_complete_and_rearm      fire scorch_cb; set a new random fuse.
 *   burst_tick                    the FSM dispatch itself.
 */

/*
 * burst_tick — run the burst FSM for one frame.  Tier-3.
 *
 *   Pseudocode:
 *     switch state:
 *       IDLE:   if --fuse <= 0:    burst_ignite();   IDLE → FLASH
 *       FLASH:  state=LIVE; ticks=0;                  FLASH → LIVE
 *       LIVE:   any = burst_advance_live_particles()
 *               ticks++
 *               if !any or ticks >= BURST_TICKS:
 *                 burst_complete_and_rearm()         LIVE → IDLE
 *
 *   Inputs / outputs:
 *     b              Burst (mutated: state, ticks, fuse, every parts[i]).
 *     cols, rows     terminal extent, passed through.
 *     scorch_cb, ud  callback + opaque pointer; fired ONCE on LIVE → IDLE.
 *
 *   Why it exists (vs inlining in field_tick):
 *   field_tick is a one-line loop; all FSM detail lives here.  Engine
 *   vs orchestrator — burst owns its own clock.
 */
/*
 * burst_advance_live_particles() — tick every spark; return whether any
 * are still alive after the tick.  The bool result IS the LIVE-state
 * termination signal.
 */
static bool burst_advance_live_particles(Burst *b, int cols, int rows)
{
    bool any_alive = false;
    for (int i = 0; i < PARTICLES; i++) {
        particle_tick(&b->parts[i], cols, rows);
        if (b->parts[i].alive) any_alive = true;
    }
    return any_alive;
}

/*
 * burst_complete_and_rearm() — fire scorch callback, set new fuse, IDLE.
 *
 *   Called when LIVE ends — either because every spark died, or because
 *   the BURST_TICKS budget has been spent.
 *
 *      scorch_cb(cx, cy)                stamp memory of this burst
 *      fuse  ← FUSE_MIN + rand(FUSE_RANGE)
 *      state ← IDLE
 */
static void burst_complete_and_rearm(Burst *b,
                                     void (*scorch_cb)(int, int, void *),
                                     void *ud)
{
    if (scorch_cb) scorch_cb((int)b->cx, (int)b->cy, ud);
    b->fuse  = FUSE_MIN + rand_int_below(FUSE_RANGE);
    b->state = BS_IDLE;
}

/*
 * burst_tick() — run the burst FSM for one frame.  The body is a single
 * switch over BurstState; each case is the minimum work to either stay
 * or advance.
 */
static void burst_tick(Burst *b, int cols, int rows,
                       void (*scorch_cb)(int x, int y, void *ud), void *ud)
{
    switch (b->state) {
    case BS_IDLE:
        /* Countdown the fuse; ignite at zero. */
        b->fuse--;
        if (b->fuse <= 0) burst_ignite(b, cols, rows);
        break;

    case BS_FLASH:
        /* FLASH lasts exactly one tick. */
        b->state = BS_LIVE;
        b->ticks = 0;
        break;

    case BS_LIVE: {
        bool any_alive   = burst_advance_live_particles(b, cols, rows);
        b->ticks++;
        bool out_of_time = (b->ticks >= BURST_TICKS);
        if (!any_alive || out_of_time)
            burst_complete_and_rearm(b, scorch_cb, ud);
        break;
    }
    }
}

/* ===================================================================== */
/* §9  burst render — flash cross + fuse overlay + burst_draw             */
/* ===================================================================== */

/*
 * §9 PREAMBLE — state-dependent rendering
 * ───────────────────────────────────────
 *
 * burst_draw paints DIFFERENT things depending on state.  Encapsulation
 * keeps the FSM-dispatch in one place; adding a new state's visual
 * means adding one case here without touching field_draw.
 *
 *   draw_flash_cross    one-tick '*+' detonation marker.
 *   draw_fuse_overlay   debug-mode annotation showing FSM counter.
 *   burst_draw          the per-state dispatch.
 */

/*
 * draw_flash_cross — paint the one-tick detonation marker.
 *
 *   Draws '*' at the centre and '+' on each of the four cardinal
 *   neighbours that lie inside the screen.  Bright yellow + A_BOLD —
 *   designed to read as "BANG" before the spark fan emerges.
 */
static void draw_flash_cross(WINDOW *w, int cx, int cy, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;

    wattron(w, COLOR_PAIR(C_YELLOW) | A_BOLD);
    mvwaddch(w, cy, cx, '*');
    if (cx > 0)       mvwaddch(w, cy,     cx - 1, '+');
    if (cx < cols-1)  mvwaddch(w, cy,     cx + 1, '+');
    if (cy > 0)       mvwaddch(w, cy - 1, cx,     '+');
    if (cy < rows-1)  mvwaddch(w, cy + 1, cx,     '+');
    wattroff(w, COLOR_PAIR(C_YELLOW) | A_BOLD);
}

/*
 * draw_fuse_overlay() — annotate the burst centre with its FSM counter.
 *
 *   IDLE  →  "f<fuse>"   (countdown to ignition)
 *   LIVE  →  "t<ticks>"  (tick number since FLASH, 0..BURST_TICKS)
 *   FLASH →  no annotation (the cross is the annotation)
 *
 *   Used only when g_debug == DBG_FUSE.  Makes the FSM legible at a
 *   glance — you can see all 16 slots ticking down independently.
 */
static void draw_fuse_overlay(const Burst *b, WINDOW *w,
                              int cx, int cy, int cols, int rows)
{
    bool centre_in_bounds = (cx >= 0 && cx < cols - 3 && cy >= 0 && cy < rows);
    if (!centre_in_bounds) return;

    char label[8];
    if      (b->state == BS_IDLE) snprintf(label, sizeof label, "f%d", b->fuse);
    else if (b->state == BS_LIVE) snprintf(label, sizeof label, "t%d", b->ticks);
    else return;

    wattron(w, COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvwaddstr(w, cy, cx, label);
    wattroff(w, COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/*
 * burst_draw() — render the burst according to its FSM state.
 *
 *   FLASH  →  draw_flash_cross() and return (no sparks visible yet)
 *   LIVE   →  particle_draw() for every spark
 *   IDLE   →  nothing (invisible; scorch layer is the field's job)
 *
 *   The DBG_FUSE overlay annotates IDLE and LIVE slots with their
 *   FSM counter and is layered on top of the normal render.
 */
static void burst_draw(const Burst *b, WINDOW *w, int cols, int rows)
{
    int cx = (int)b->cx;
    int cy = (int)b->cy;

    if (b->state == BS_FLASH) {
        draw_flash_cross(w, cx, cy, cols, rows);
        return;
    }

    if (b->state == BS_LIVE) {
        for (int i = 0; i < PARTICLES; i++)
            particle_draw(&b->parts[i], w, cols, rows);
    }

    if (g_debug == DBG_FUSE) draw_fuse_overlay(b, w, cx, cy, cols, rows);
}

/* ===================================================================== */
/* §10  field — burst pool + persistent scorch grid                       */
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

    /*
     * Initialise every slot.  Active slots get a staggered initial fuse
     * so all `burst_count` bursts don't fire on the same frame.  Inactive
     * slots get FUSE_NEVER so they sit dormant until the user increases
     * `bursts` via '+'.
     */
    int stagger_step = FUSE_MIN + (burst_count > 0 ? FUSE_RANGE / burst_count : 0);

    for (int i = 0; i < BURSTS_MAX; i++) {
        memset(&f->bursts[i], 0, sizeof(Burst));
        f->bursts[i].state = BS_IDLE;
        f->bursts[i].fuse  = (i < burst_count) ? (i * stagger_step)
                                               : FUSE_NEVER;
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

/*
 * field_draw_scorch_layer() — paint every non-zero scorch cell in
 * dim orange.  This is the photographic-memory layer — drawn first
 * so live sparks render on top.
 */
static void field_draw_scorch_layer(const Field *f, WINDOW *w)
{
    int total_cells = f->cols * f->rows;

    wattron(w, COLOR_PAIR(C_ORANGE) | A_DIM);
    for (int i = 0; i < total_cells; i++) {
        char scorch_glyph = f->scorch[i];
        if (!scorch_glyph) continue;
        int cell_y = i / f->cols;
        int cell_x = i % f->cols;
        mvwaddch(w, cell_y, cell_x, (chtype)(unsigned char)scorch_glyph);
    }
    wattroff(w, COLOR_PAIR(C_ORANGE) | A_DIM);
}

/*
 * field_draw_active_bursts() — paint every active burst slot on top
 * of the scorch layer.  Each slot decides for itself how to draw
 * based on its FSM state.
 */
static void field_draw_active_bursts(const Field *f, WINDOW *w)
{
    for (int i = 0; i < f->active_bursts; i++)
        burst_draw(&f->bursts[i], w, f->cols, f->rows);
}

/*
 * field_draw() — two named passes: scorch underneath, bursts on top.
 */
static void field_draw(const Field *f, WINDOW *w)
{
    field_draw_scorch_layer (f, w);
    field_draw_active_bursts(f, w);
}

/* ===================================================================== */
/* §11  screen + HUD — ncurses init + status strip                        */
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
/* §12  app — App struct + signal handlers + key dispatch                 */
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

/* ===================================================================== */
/* §13  main — fixed-step loop                                            */
/* ===================================================================== */

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
