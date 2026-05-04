/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * pixel_dissolve.c — text rendered as particles that disintegrate
 *                    and reform into the next word
 *
 * DEMO: A short word ("BOOM" / "ASCII" / "PIXEL" / …) appears on
 *       screen rendered as a bitmap of glowing particles — each "on"
 *       pixel of the embedded 5×7 font is one particle. After a
 *       few seconds the word DISSOLVES — every particle gets a
 *       velocity (explosion, swirl, rain, or drift depending on
 *       pattern). After the dissolve the next word's targets are
 *       computed, and the SAME particles spring back through the
 *       air to form the next word. Loops continuously.
 *
 *       Patterns:
 *         EXPLODE  particles fly OUTWARD from word centre
 *         SWIRL    particles spin TANGENTIALLY (vortex dissolve)
 *         RAIN     particles FALL away under gravity
 *         DRIFT    particles drift in random direction (smoke fade)
 *
 * Study alongside:
 *   physics/cloth.c          — same damped-spring integration
 *                               idiom (each particle pulled to a
 *                               target rest position).
 *   particle_systems/comet.c — same moving-emitter framework but
 *                               here the emitter is the WORD itself
 *                               (a snapshot in space-time).
 *
 * Section map:
 *   §1 config    — constants, themes, patterns, word list, font
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-pair colour ramp + accent pairs
 *   §4 font      — 5×7 bitmap font lookup
 *   §5 particle  — Particle struct
 *   §6 scene     — pool, target builder, phase machine, tick, draw
 *   §7 screen    — ncurses init / draw / resize
 *   §8 app       — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reseed (jump to next word, clear & reform)
 *   n / N      next dissolve pattern  (EXPLODE → SWIRL → RAIN → DRIFT)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster (speed multiplier ×2)
 *   -          slower (÷2)
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/pixel_dissolve.c \
 *       -o pixel_dissolve -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Particle-per-pixel rendering of a bitmap font, with
 *                  a 3-state PHASE MACHINE driving each cycle:
 *
 *                    ASSEMBLE  →  HOLD  →  DISSOLVE  →  TRANSITION
 *                                                            ↓
 *                                                       ASSEMBLE …
 *
 *                  Each cycle:
 *                    1. ASSEMBLE: each particle springs toward its
 *                       target position via a damped harmonic
 *                       oscillator (`F = K·(target − pos) − D·v`).
 *                       Particles smoothly arrive and settle.
 *                    2. HOLD: spring continues (now keeping particles
 *                       at target); small Brownian-jitter is added to
 *                       give "alive" feel — letters appear to subtly
 *                       breathe rather than be a static glyph. Lasts
 *                       a few seconds for legibility.
 *                    3. DISSOLVE: each particle is given an explosion
 *                       velocity per pattern (radial outward / tangent
 *                       /downward / random). Spring is OFF so they
 *                       freely drift. Mild drag bleeds energy.
 *                    4. TRANSITION: pick next word from the cycle
 *                       list, recompute targets, REASSIGN particles
 *                       (sequential mapping), reactivate excess /
 *                       spawn new for shortfalls. Phase resets to
 *                       ASSEMBLE — particles spring from their
 *                       current scattered positions to the new word.
 *
 *                  The bitmap font is a 5×7 monospaced set, embedded
 *                  inline as `uint8_t font_5x7[256][7]` (covers A-Z,
 *                  0-9, plus a few symbols). For each character of
 *                  the word, we walk the 5×7 grid and emit a particle
 *                  target for every "on" bit. The whole word is then
 *                  centered on the screen.
 *
 *                  Particle colour is set by HORIZONTAL POSITION in
 *                  the word — a left-to-right gradient through the
 *                  active theme's 8-step ramp. So each word reads
 *                  like a colour-graded title screen.
 *
 * Data-structure : Particle[MAX_PARTICLES] object pool with `active`
 *                  flag and a target `(tx, ty)` per slot. Linear-
 *                  scan everywhere; N is bounded by font geometry
 *                  (longest word ~150 pixels at 5×7).
 *
 * Rendering      : ASCII only. Each particle renders as a single
 *                  glyph at its current cell. Glyph fades during
 *                  DISSOLVE so far-flung particles look diffuse.
 *
 * Performance    : O(N · 1) per tick where N ≤ 200. Trivial. Phase
 *                  transitions are O(N²) only if we used optimal
 *                  matching; we use sequential matching which is O(N).
 *
 * References     :
 *   • Wikipedia — [Damped harmonic oscillator](https://en.wikipedia.org/wiki/Harmonic_oscillator).
 *     The K·(target − x) − D·v force pulling particles to their
 *     target is the classic spring-damper equation.
 *   • Adafruit — GFX 5×7 font (public-domain). The bitmap font
 *     embedded here follows the Adafruit-style 7-byte-per-glyph
 *     encoding.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A WORD is a SET OF TARGET POSITIONS. Each particle has a target
 * and a current position. A spring force pulls the particle to its
 * target; the particle SETTLES into place over a second or two. To
 * dissolve, give the particle a velocity and turn the spring off:
 * it drifts away. To reform into a new word, recompute targets and
 * turn the spring back on: the particles spring from wherever they
 * happen to be to the new positions.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine 100 fireflies tied to invisible rubber bands, each band
 * fixed to one pixel of a glowing letter. Pluck the bands: the
 * fireflies snap to position and spell out the word. Cut the bands
 * and shoot the fireflies sideways: they scatter. Re-tie the bands
 * to a different word's pixels: they spring to the new positions.
 * That's it.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. BUILD TARGETS. For the active word, walk each character; for
 *     each "on" pixel of the 5×7 bitmap, append a target position.
 *     Centre the whole word on screen. Total particles = total "on"
 *     pixels (typically 80-150 for 4-8 letter words).
 *
 *  2. PER TICK:
 *     ASSEMBLE / HOLD: damped spring toward target.
 *       fx = K · (tx − px) − D · vx
 *       fy = K · (ty − py) − D · vy   (with aspect-correct y)
 *       v += f · dt    p += v · dt
 *       Plus tiny Brownian jitter during HOLD.
 *
 *     DISSOLVE: no spring, only drag.
 *       v *= exp(-DRAG · dt)
 *       p += v · dt
 *
 *  3. PHASE MACHINE: advance phase_t by dt; transition when timer
 *     elapses:
 *       ASSEMBLE → HOLD  after ASSEMBLE_DUR
 *       HOLD     → DISSOLVE after HOLD_DUR  (apply dissolve velocity)
 *       DISSOLVE → ASSEMBLE after DISSOLVE_DUR
 *                  (transition: pick next word, rebuild targets,
 *                   reassign particles, springs back on)
 *
 *  4. RENDER all active particles at integer cell positions in the
 *     theme's gradient colour by horizontal position.
 *
 * KEY FORMULAS
 * ────────────
 *  Damped spring (per particle, per axis):
 *    f = K · (target − pos) − D · vel
 *    vel += f · dt
 *    pos += vel · dt
 *
 *  Critical damping (no overshoot):
 *    D_crit = 2 · √(K · m)
 *  We use K=18, D=8 → slightly underdamped; particles overshoot
 *  slightly then settle — gives a snappy "boing" feel.
 *
 *  Dissolve velocity:
 *    EXPLODE: v = (p − centre)/r · SPEED
 *    SWIRL:   v = ((-uy, ux))      · SPEED      (perpendicular to radial)
 *    RAIN:    vx = (r-0.5)·SCATTER; vy = +SPEED
 *    DRIFT:   vx = (r-0.5)·SPEED;  vy = (r-0.5)·SPEED
 *
 *  Aspect-correct spring (cells are 2× taller than wide so vertical
 *  spring needs proportionally less force to look balanced):
 *    fy = (K · (ty − py)) / ASPECT_Y - D · vy
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • PARTICLE COUNT MISMATCH BETWEEN WORDS. Shorter words need
 *    fewer particles than longer. On transition: extras get
 *    deactivated (they fade off as they leave screen during
 *    dissolve); shortfalls get spawned at random off-screen
 *    positions and spring in.
 *
 *  • SEQUENTIAL VS OPTIMAL MATCHING. We assign particle[i] →
 *    target[i] in order. Optimal matching (Hungarian algorithm)
 *    would minimise total travel distance. Sequential is much
 *    cheaper and the visual is nearly indistinguishable — particles
 *    take varied paths, which actually looks MORE organic than
 *    "everything moves the shortest distance".
 *
 *  • TARGET ORDERING. We walk word left-to-right, top-to-bottom.
 *    So targets[0] is top-left of the first letter; targets[N-1]
 *    is bottom-right of the last letter. With sequential matching,
 *    particle[0] ends up roughly top-left consistently, which can
 *    look "ordered". For more chaotic transitions, shuffle
 *    targets[] before assignment.
 *
 *  • BITMAP FONT BIT ORDER. font_5x7[c][row] is a uint8 with the
 *    LSB representing column 4 (rightmost), bit 4 representing
 *    column 0 (leftmost). When extracting bits, mask with
 *    (1 << (4 - col)).
 *
 *  • UNKNOWN CHARACTERS. Character not in the font (e.g. lowercase,
 *    accents) renders as space (no particles emitted). Words with
 *    only A-Z + 0-9 + ! ? . avoid this.
 *
 *  • SPRING STIFFNESS vs DURATION. ASSEMBLE_DUR must be long enough
 *    that the spring (K=18, D=8) has time to settle from the
 *    initial scattered state. Empirically 1.4 sec works.
 *
 *  • PAUSE. Phase timer skips, particle integration skips, but the
 *    HUD timer counts on (small cosmetic — fix later if needed).
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space) during DISSOLVE. Particles freeze mid-flight.
 *    Resume: motion continues.
 *
 *  • EXPLODE pattern. Particles all fly OUTWARD from word centre,
 *    scattering radially. After dissolve, they spring back from
 *    every direction.
 *
 *  • SWIRL pattern. Particles spin around the word centre — clean
 *    tangential vortex outward. Reform happens by reverse spiral.
 *
 *  • RAIN pattern. Particles all fall DOWN under gravity. Reform:
 *    particles fly up from below to the new word.
 *
 *  • DRIFT pattern. Particles wander randomly — most natural-looking
 *    "smoke fade".
 *
 *  • Word cycle. Watch BOOM → ASCII → PIXEL → MORPH → DUST → CODE
 *    → DISSOLVE → USELESS → back to BOOM.
 *
 *  • Theme cycle (`t`/`T`). Each theme produces a distinctively
 *    coloured word (FIRE = red/orange, ICE = blue/cyan, etc.).
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

    MAX_PARTICLES    =  400,    /* enough for ~10-letter word at 5×7  */

    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,

    /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD         =   1,
    PAIR_HINT        =   2,
    PAIR_RAMP_BASE   =   3,    /* +0..+7 = horizontal-gradient ramp   */
    PAIR_SKY         =  11,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define ASPECT_Y          2.0f       /* terminal cells 2× taller       */

/* Spring physics (damped harmonic oscillator). */
#define SPRING_K          18.0f
#define SPRING_D           8.0f
#define HOLD_JITTER        4.0f      /* cells/sec² Brownian jitter     */
#define DISSOLVE_DRAG      0.6f      /* per-second exp damping         */

/* Phase durations (seconds). */
#define ASSEMBLE_DUR       1.5f
#define HOLD_DUR           3.0f
#define DISSOLVE_DUR       1.5f

/* Dissolve initial velocity speeds. */
#define EXPLODE_SPEED      45.0f
#define SWIRL_SPEED        35.0f
#define RAIN_SPEED         50.0f
#define RAIN_GRAVITY      120.0f
#define DRIFT_SPEED        25.0f

/* Pattern enum. */
typedef enum {
    PATTERN_EXPLODE = 0,
    PATTERN_SWIRL   = 1,
    PATTERN_RAIN    = 2,
    PATTERN_DRIFT   = 3,
    N_PATTERNS      = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_EXPLODE: return "EXPLODE";
    case PATTERN_SWIRL:   return "SWIRL  ";
    case PATTERN_RAIN:    return "RAIN   ";
    case PATTERN_DRIFT:   return "DRIFT  ";
    default:              return "?      ";
    }
}

/* Phase enum. */
typedef enum {
    PHASE_ASSEMBLE = 0,
    PHASE_HOLD     = 1,
    PHASE_DISSOLVE = 2,
} Phase;

/*
 * Themes — 8-step horizontal gradient. Each particle's colour is
 * its slot in the gradient based on its target's horizontal position
 * in the word — leftmost = ramp[0], rightmost = ramp[7].
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule.
 */
typedef struct {
    const char *name;
    short       ramp[8];
    short       sky;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name        ramp[0..7]                                       sky */

    { "DEFAULT",  {  87, 117, 153, 195, 218, 217, 211, 196 },        234 },
    { "FIRE",     {  88, 124, 130, 166, 196, 208, 214, 226 },        233 },
    { "ICE",      {  24,  31,  67, 110, 117, 153, 195, 231 },        234 },
    { "NEON",     {  53,  91, 134, 165, 207, 213, 219, 225 },        234 },
    { "AURORA",   {  43,  79, 115, 121, 157, 195, 230, 231 },        234 },
    { "VIOLET",   {  53,  54,  91, 134, 135, 176, 213, 219 },        233 },
    { "TROPICAL", {  29,  35,  37,  44,  50,  86, 122, 159 },        234 },
    { "FOREST",   {  28,  34,  40,  64,  70, 112, 156, 192 },        234 },
    { "MONO",     { 240, 243, 245, 247, 249, 251, 253, 255 },        232 },
    { "MATRIX",   {  22,  28,  34,  40,  46,  82, 118, 154 },        232 },
};

/* Word cycle list. */
static const char *WORDS[] = {
    "BOOM",
    "ASCII",
    "PIXEL",
    "MORPH",
    "DUST",
    "CODE",
    "DISSOLVE",
    "USELESS",
};
#define N_WORDS ((int)(sizeof WORDS / sizeof WORDS[0]))

/*
 * font_5x7 — 5-column × 7-row monospaced bitmap font.
 *
 * Each entry is 7 bytes (one per row, top-to-bottom). Bits within a
 * byte represent the 5 columns: bit 4 = column 0 (leftmost), bit 0 =
 * column 4 (rightmost). Entries not listed are zero (blank).
 *
 * Covers: space, A-Z, 0-9, '!' '?' '.'.
 */
static const uint8_t font_5x7[256][7] = {
    [' '] = {0,0,0,0,0,0,0},
    ['!'] = {0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04},
    ['?'] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04},
    ['.'] = {0,0,0,0,0,0x00,0x04},

    ['A'] = {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    ['B'] = {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},
    ['C'] = {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E},
    ['D'] = {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E},
    ['E'] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F},
    ['F'] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10},
    ['G'] = {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E},
    ['H'] = {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    ['I'] = {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E},
    ['J'] = {0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C},
    ['K'] = {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11},
    ['L'] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F},
    ['M'] = {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11},
    ['N'] = {0x11, 0x11, 0x19, 0x15, 0x13, 0x11, 0x11},
    ['O'] = {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},
    ['P'] = {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10},
    ['Q'] = {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D},
    ['R'] = {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11},
    ['S'] = {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E},
    ['T'] = {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
    ['U'] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},
    ['V'] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04},
    ['W'] = {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11},
    ['X'] = {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11},
    ['Y'] = {0x11, 0x11, 0x11, 0x0A, 0x04, 0x04, 0x04},
    ['Z'] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F},

    ['0'] = {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E},
    ['1'] = {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    ['2'] = {0x0E, 0x11, 0x01, 0x06, 0x08, 0x10, 0x1F},
    ['3'] = {0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E},
    ['4'] = {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02},
    ['5'] = {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},
    ['6'] = {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E},
    ['7'] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10},
    ['8'] = {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E},
    ['9'] = {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},
};

#define FONT_W 5
#define FONT_H 7
#define FONT_KERN 1     /* cells between letters */

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
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
        init_pair(PAIR_SKY, t->sky, -1);
    } else {
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), COLOR_WHITE, -1);
        init_pair(PAIR_SKY, COLOR_BLACK, -1);
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
/* §4  font                                                               */
/* ===================================================================== */

/* Width in cells of a rendered word (FONT_W per char + FONT_KERN between). */
static int word_width_cells(const char *word)
{
    int n = (int)strlen(word);
    if (n <= 0) return 0;
    return n * FONT_W + (n - 1) * FONT_KERN;
}

/* ===================================================================== */
/* §5  particle                                                           */
/* ===================================================================== */

typedef struct {
    float px, py;        /* current position (cell coords)             */
    float vx, vy;        /* velocity (cells/sec)                       */
    float tx, ty;        /* target position when assembled             */
    int   color_slot;    /* 0..7 — picks ramp slot                     */
    bool  active;
} Particle;

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

/* ===================================================================== */
/* §6  scene — pool, target builder, phase machine, tick, draw           */
/* ===================================================================== */

typedef struct {
    bool      paused;
    int       speed;
    int       current_theme;
    Pattern   current_pattern;
    uint32_t  rng;
    int       rows, cols;

    Phase     phase;
    float     phase_t;
    int       word_idx;
    float     word_cx, word_cy;     /* cached centre of current word    */
    int       word_w_cells;         /* cached width                    */

    int       n_particles;
    Particle  particles[MAX_PARTICLES];
} Scene;

static void scene_clear_particles(Scene *s)
{
    s->n_particles = 0;
    for (int i = 0; i < MAX_PARTICLES; i++) s->particles[i].active = false;
}

/*
 * scene_build_targets — walk the active word's pixels and fill in the
 * target positions for each "on" pixel. Centres the word horizontally
 * and vertically. Caches centre / width on Scene.
 *
 * Returns the number of targets emitted.
 */
static int scene_build_targets(Scene *s, const char *word,
                               float *out_targets_x,
                               float *out_targets_y,
                               int   *out_color_slots,
                               int    max_targets)
{
    int rows_eff = s->rows - 1;
    int word_w = word_width_cells(word);
    float start_x = (float)(s->cols - word_w) * 0.5f;
    float start_y = (float)(rows_eff - FONT_H) * 0.5f;

    int n_total = 0;
    float char_x = start_x;
    int n_chars = (int)strlen(word);

    for (int c = 0; c < n_chars; c++) {
        unsigned ch = (unsigned char)word[c];
        const uint8_t *bits = font_5x7[ch];

        for (int row = 0; row < FONT_H; row++) {
            uint8_t b = bits[row];
            for (int col = 0; col < FONT_W; col++) {
                if (b & (1 << (FONT_W - 1 - col))) {
                    if (n_total >= max_targets) goto done;
                    float tx = char_x + (float)col;
                    float ty = start_y + (float)row;
                    /* Colour slot from horizontal position in the word. */
                    float frac = (float)(tx - start_x) / (float)(word_w > 1 ? word_w - 1 : 1);
                    int slot = (int)(frac * 7.999f);
                    if (slot < 0) slot = 0;
                    if (slot > 7) slot = 7;

                    out_targets_x  [n_total] = tx;
                    out_targets_y  [n_total] = ty;
                    out_color_slots[n_total] = slot;
                    n_total++;
                }
            }
        }
        char_x += FONT_W + FONT_KERN;
    }
done:
    s->word_w_cells = word_w;
    s->word_cx = start_x + (float)word_w * 0.5f;
    s->word_cy = start_y + (float)FONT_H * 0.5f;
    return n_total;
}

/*
 * scene_load_word — build targets for the current word and reassign
 * particles. If new word needs more particles than currently active,
 * spawn more at random off-screen positions; if fewer, deactivate
 * the surplus.
 */
static void scene_load_word(Scene *s)
{
    static float targets_x[MAX_PARTICLES];
    static float targets_y[MAX_PARTICLES];
    static int   color_slots[MAX_PARTICLES];

    const char *word = WORDS[s->word_idx];
    int n_targets = scene_build_targets(s, word,
                                         targets_x, targets_y, color_slots,
                                         MAX_PARTICLES);

    /* Spawn new particles at random off-screen positions if needed. */
    while (s->n_particles < n_targets) {
        Particle *p = &s->particles[s->n_particles];
        /* Random position around the screen edges. */
        int edge = (int)(lcg_unit(&s->rng) * 4.0f);
        float r1 = lcg_unit(&s->rng);
        switch (edge) {
        case 0: p->px = r1 * (float)s->cols; p->py = -3.0f; break;
        case 1: p->px = -3.0f; p->py = r1 * (float)s->rows; break;
        case 2: p->px = (float)s->cols + 3.0f;
                p->py = r1 * (float)s->rows; break;
        default: p->px = r1 * (float)s->cols;
                 p->py = (float)s->rows + 3.0f; break;
        }
        p->vx = 0.0f;
        p->vy = 0.0f;
        p->active = true;
        s->n_particles++;
    }

    /* Deactivate surplus. */
    for (int i = n_targets; i < s->n_particles; i++)
        s->particles[i].active = false;
    s->n_particles = n_targets;

    /* Assign targets sequentially. */
    for (int i = 0; i < n_targets; i++) {
        Particle *p = &s->particles[i];
        p->tx         = targets_x[i];
        p->ty         = targets_y[i];
        p->color_slot = color_slots[i];
    }
}

/*
 * scene_apply_dissolve_velocity — give each particle an outward
 * velocity per the active dissolve pattern.
 */
static void scene_apply_dissolve_velocity(Scene *s)
{
    for (int i = 0; i < s->n_particles; i++) {
        Particle *p = &s->particles[i];
        if (!p->active) continue;

        /* Vector from word centre (aspect-corrected for radial calcs). */
        float dx = p->px - s->word_cx;
        float dy = (p->py - s->word_cy) * ASPECT_Y;
        float r = sqrtf(dx * dx + dy * dy);
        if (r < 0.5f) {
            /* Particle effectively at centre — pick random direction. */
            float ang = lcg_unit(&s->rng) * 2.0f * (float)M_PI;
            dx = cosf(ang);
            dy = sinf(ang);
            r  = 1.0f;
        }
        float ux = dx / r;
        float uy = dy / r;

        switch (s->current_pattern) {
        case PATTERN_EXPLODE: {
            float speed = EXPLODE_SPEED * (0.7f + lcg_unit(&s->rng) * 0.6f);
            p->vx = ux * speed;
            p->vy = uy * speed / ASPECT_Y;
            break;
        }
        case PATTERN_SWIRL: {
            float speed = SWIRL_SPEED * (0.7f + lcg_unit(&s->rng) * 0.6f);
            /* Tangent direction: perpendicular to radial. */
            p->vx = -uy * speed;
            p->vy =  ux * speed / ASPECT_Y;
            break;
        }
        case PATTERN_RAIN: {
            p->vx = (lcg_unit(&s->rng) - 0.5f) * 6.0f;
            p->vy = RAIN_SPEED * (0.7f + lcg_unit(&s->rng) * 0.5f);
            break;
        }
        case PATTERN_DRIFT: {
            p->vx = (lcg_unit(&s->rng) - 0.5f) * 2.0f * DRIFT_SPEED;
            p->vy = (lcg_unit(&s->rng) - 0.5f) * 2.0f * DRIFT_SPEED / ASPECT_Y;
            break;
        }
        case N_PATTERNS: break;
        }
    }
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_EXPLODE;
    s->rng             = (uint32_t)clock_ns();
    s->cols            = cols;
    s->rows            = rows;

    s->phase           = PHASE_ASSEMBLE;
    s->phase_t         = 0.0f;
    s->word_idx        = 0;

    scene_clear_particles(s);
    scene_load_word(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
    /* Recompute targets for new screen size. */
    scene_load_word(s);
}

static void scene_reseed(Scene *s)
{
    s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
    s->word_idx = (s->word_idx + 1) % N_WORDS;
    s->phase = PHASE_ASSEMBLE;
    s->phase_t = 0.0f;
    scene_load_word(s);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    dt *= speed_mul;

    s->phase_t += dt;

    /* Phase transitions. */
    switch (s->phase) {
    case PHASE_ASSEMBLE:
        if (s->phase_t >= ASSEMBLE_DUR) {
            s->phase = PHASE_HOLD;
            s->phase_t = 0.0f;
        }
        break;
    case PHASE_HOLD:
        if (s->phase_t >= HOLD_DUR) {
            s->phase = PHASE_DISSOLVE;
            s->phase_t = 0.0f;
            scene_apply_dissolve_velocity(s);
        }
        break;
    case PHASE_DISSOLVE:
        if (s->phase_t >= DISSOLVE_DUR) {
            /* Transition to next word: re-target, re-spawn, → ASSEMBLE. */
            s->word_idx = (s->word_idx + 1) % N_WORDS;
            scene_load_word(s);
            s->phase = PHASE_ASSEMBLE;
            s->phase_t = 0.0f;
        }
        break;
    }

    /* Per-particle integration. */
    if (s->phase == PHASE_DISSOLVE) {
        /* Free drift with mild drag; RAIN gets gravity. */
        float drag = expf(-DISSOLVE_DRAG * dt);
        for (int i = 0; i < s->n_particles; i++) {
            Particle *p = &s->particles[i];
            if (!p->active) continue;
            if (s->current_pattern == PATTERN_RAIN)
                p->vy += RAIN_GRAVITY * dt;
            p->vx *= drag;
            p->vy *= drag;
            p->px += p->vx * dt;
            p->py += p->vy * dt;
        }
    } else {
        /* ASSEMBLE / HOLD: damped spring toward target. */
        float k = SPRING_K;
        float d = SPRING_D;
        for (int i = 0; i < s->n_particles; i++) {
            Particle *p = &s->particles[i];
            if (!p->active) continue;

            float fx = k * (p->tx - p->px) - d * p->vx;
            /* Aspect-correct y spring: cells are 2× taller, so a unit
             * "displacement" in cells corresponds to ASPECT_Y phys units.
             * Scale spring force by 1/ASPECT_Y so vertical settling
             * looks balanced. */
            float fy = (k * (p->ty - p->py)) / ASPECT_Y - d * p->vy;

            if (s->phase == PHASE_HOLD) {
                fx += (lcg_unit(&s->rng) - 0.5f) * HOLD_JITTER;
                fy += (lcg_unit(&s->rng) - 0.5f) * HOLD_JITTER;
            }

            p->vx += fx * dt;
            p->vy += fy * dt;
            p->px += p->vx * dt;
            p->py += p->vy * dt;
        }
    }
}

static void scene_draw(const Scene *s)
{
    int rows_eff = s->rows - 1;
    for (int i = 0; i < s->n_particles; i++) {
        const Particle *p = &s->particles[i];
        if (!p->active) continue;
        int ix = (int)(p->px + 0.5f);
        int iy = (int)(p->py + 0.5f);
        if (ix < 0 || ix >= s->cols) continue;
        if (iy < 0 || iy >= rows_eff) continue;

        /* Glyph by phase: assembled = '*', flying = '+', dispersed = '.'.
         * Approximate by distance to target. */
        float dx = p->tx - p->px;
        float dy = p->ty - p->py;
        float d2 = dx * dx + dy * dy;
        char  glyph;
        int   attr;
        if      (d2 < 1.0f)  { glyph = '*'; attr = A_BOLD;   }
        else if (d2 < 25.0f) { glyph = '+'; attr = A_NORMAL; }
        else                 { glyph = '.'; attr = A_DIM;    }

        int slot = p->color_slot;
        int pair = PAIR_RAMP_BASE + slot;
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(iy, ix, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | attr);
    }
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

static const char *phase_str(Phase p)
{
    switch (p) {
    case PHASE_ASSEMBLE: return "ASSEMBLE";
    case PHASE_HOLD:     return "HOLD    ";
    case PHASE_DISSOLVE: return "DISSOLVE";
    default:             return "?       ";
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s);

    const char *state_str = s->paused ? "PAUSED " : pattern_name(s->current_pattern);

    char buf[220];
    snprintf(buf, sizeof buf,
             " PIXEL_DISSOLVE   %s   theme:%-8s   word:'%-8s'   phase:%s   N:%3d   "
             "%5.1f fps  %3d Hz  speed:%-3d   "
             "n/p:pat  t/T:theme  +/-:speed  spc:pause  r:next  q:quit ",
             state_str, themes[s->current_theme].name,
             WORDS[s->word_idx], phase_str(s->phase), s->n_particles,
             fps, sim_fps, s->speed);

    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
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
