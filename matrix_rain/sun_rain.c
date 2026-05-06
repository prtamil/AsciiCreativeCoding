/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sun_rain.c — radial matrix-rain sun corona
 *
 * DEMO: A single '@' burns at the screen centre. 180 independent
 *       radial streams of matrix glyphs shoot outward from it in
 *       all directions — a continuous solar wind with no circle,
 *       no disc, no border. Each stream has its own random speed
 *       and a stagger offset so they appear at different distances
 *       from the core at any given moment, producing a stochastic
 *       field of beams rather than a single synchronised burst.
 *
 *       Sketch (one frozen frame; 180 rays, only 12 drawn for
 *       clarity, with their head positions × and tails →):
 *
 *                  ×
 *               × ←  ← ←
 *           ×   ←
 *           ×       ←
 *        ×             ←
 *           ×              ×
 *                @           ×
 *           ×              ×
 *        ×             ←
 *           ×       ←
 *           ×   ←
 *               × ←  ← ←
 *                  ×
 *
 *       The core '@' is drawn LAST, every frame, so no ray ever
 *       overwrites it. Each ray's trail fades from a bright white
 *       head through five themed shades to a barely-visible tail.
 *
 * Study alongside:
 *   matrix_rain/matrix_rain.c       — same shimmer-cache trick on
 *                                     plain vertical falling streams.
 *   matrix_rain/pulsar_rain.c       — the rotating sibling: rays go
 *                                     SIDEWAYS instead of OUTWARD;
 *                                     this file is what you get if
 *                                     you spin omega = 0 and let the
 *                                     rays slide along their angle.
 *   matrix_rain/fireworks_rain.c    — same cache-shimmer trick on
 *                                     parabolic-arc spark trails.
 *   matrix_rain/matrix_snowflake.c  — rain accumulating into snow.
 *
 * Section map:
 *   §1  config       — every tunable in one place, grouped by concept
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus 5 themed 5-shade palettes
 *   §4  sun          — Ray + Sun, init, tick, draw (sub-sectioned)
 *   §5  screen       — ncurses init / present / HUD
 *   §6  app          — signals, resize, variable-dt main loop
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space / p        pause / resume
 *   r                reset (re-stagger all 180 rays)
 *   t                cycle theme (5 themes)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra matrix_rain/sun_rain.c \
 *       -o sun_rain -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Polar-coordinate radial sweep with shimmer cache.
 *
 *                 (A) RAYS    — N_RAYS independent streams, one per
 *                     evenly-spaced angle θ_i = 2π · i / N. Each ray
 *                     owns a head distance r_off (cells from centre,
 *                     float for sub-cell smoothness), a speed
 *                     (cells/sec), and a per-ray cache of RAY_TRAIL
 *                     random ASCII glyphs that reroll every frame.
 *                     When the tail clears max_r, the ray "dies"
 *                     and is reset with a new stagger and speed.
 *
 *                 (B) MOTION  — variable-dt integration: each frame,
 *                     `r_off += speed_cps · dt`. No fixed-step
 *                     accumulator, no alpha interpolation: the float
 *                     r_off gives smooth sub-cell motion at any
 *                     frame rate. Variable dt is unconditionally
 *                     stable here because the sim is non-stiff
 *                     (constant per-ray speed, no springs, no
 *                     contacts).
 *
 *                 (C) SHIMMER — for each cache slot, with probability
 *                     1 − 1/SHIMMER_KEEP_ONE_IN, reroll the glyph.
 *                     KEEP_ONE_IN = 4 → 75 % rerolled each frame —
 *                     the classic Matrix-rain shimmer rate.
 *
 * Data-structure: Sun owns 180 Ray slots in a fixed array (no heap
 *                 allocation post-init). Each Ray carries pre-baked
 *                 cos_a/sin_a direction components (set once at
 *                 sun_init), the float r_off head distance, the
 *                 speed (cells/sec), and a 16-byte glyph cache. The
 *                 Sun also caches the screen centre (cx, cy) and the
 *                 max_r recycle threshold.
 *
 * Rendering     : For each ray, walk i = 0..RAY_TRAIL-1 from head
 *                 backward (i = 0 is the head, i = RAY_TRAIL-1 is
 *                 the tail tip). At each step compute
 *                     ri = r_off - i
 *                 and break if ri < CORE_RESERVED_RADIUS so the trail
 *                 never overwrites the central '@'. The remaining
 *                 cells project to (col, row) via the pre-baked
 *                 (cos_a, sin_a · ASPECT) direction. ASPECT (≈ 0.45)
 *                 is BAKED INTO sin_a so terminal-cell anisotropy is
 *                 corrected exactly once.
 *
 * Performance   : Per frame: O(N_RAYS · RAY_TRAIL) cell paints. With
 *                 defaults (180 · 16 = 2880 mvaddch + 180 · 16 cache
 *                 rerolls), microseconds. ncurses redraw is the
 *                 dominant cost. Pre-baked direction vectors mean
 *                 zero trig in the inner loop.
 *
 * References    :
 *   Wikipedia, "Solar wind / Corona" — what the demo is visually
 *     emulating: hot plasma streaming radially outward from a star.
 *     https://en.wikipedia.org/wiki/Solar_wind
 *   "The Matrix" (1999, Wachowski) — the rerolling-glyph rain
 *     aesthetic the head + trail borrows.
 *   This project, matrix_rain/matrix_rain.c — the same shimmer
 *     cache on plain vertical streams; read first if the cache
 *     reroll trick is unfamiliar.
 *   This project, matrix_rain/pulsar_rain.c — the same polar-
 *     coordinate machinery with rays that ROTATE instead of
 *     translate outward.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Take The Matrix's vertical green rain, replace gravity with a
 * centre, and let each drop choose its own outward angle. With 180
 * angles spaced 2° apart, each carrying its own speed and stagger,
 * you get a continuous solar-corona effect: characters streaming
 * radially in every direction from a single '@' core, never
 * repeating, never pausing. The whole thing is the same per-stream
 * cache + fade-by-distance trick from matrix_rain.c, just using
 * polar coordinates instead of (x, y).
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a clock face with 180 hour-marks. From each mark, a
 * comet shoots outward through space. Each comet has a head, a
 * 16-character glittering tail, a personal speed, and was launched
 * at a different time so the field looks chaotic rather than
 * synchronised. The trick that makes it look round (and not a tall
 * ellipse) is the "tall cell" correction: terminal cells are about
 * 2:1 in aspect, so raw sin(θ) values would make vertical rays walk
 * twice as fast as horizontal ones. We multiply sin(θ) by ASPECT
 * once per ray at init time and bake the result into sin_a, so the
 * inner draw loop has no per-cell trig.
 *
 * SUN GEOMETRY DIAGRAM
 * ────────────────────
 *
 *   180 rays at angles θ_i = 2π · i / 180, for i = 0..179.
 *   For ONE ray with pre-baked (cos_a, sin_a) and head distance
 *   r_off, the cell at trail index i is at:
 *
 *       ri  = r_off - i                        (rays beyond head)
 *       col = cx + round(ri · cos_a)
 *       row = cy + round(ri · sin_a)           (sin_a has ASPECT baked in)
 *
 *   ┌─────────────────────────────────────────────────┐
 *   │                                                 │
 *   │             X      ← head (i=0)                 │
 *   │            /       ← i=1 (HOT, BOLD)            │
 *   │           /        ← i=2 (BRIGHT, BOLD)         │
 *   │          .         ← i=3..N/2 (MID)             │
 *   │         .          ← .                          │
 *   │        .           ← i=N/2+1..N-2 (DARK)        │
 *   │       .                                         │
 *   │      .             ← i=N-1 (FADE, DIM)          │
 *   │     /                                           │
 *   │    @ ← core (drawn LAST, always on top)         │
 *   │                                                 │
 *   │   ASPECT bakes into sin_a so the head circle    │
 *   │   looks round, not stretched vertically.        │
 *   │                                                 │
 *   └─────────────────────────────────────────────────┘
 *
 * BRIGHTNESS RAMP
 * ───────────────
 *
 *   i=0           HEAD     white     BOLD
 *   i=1           HOT      theme[4]  BOLD
 *   i=2           BRIGHT   theme[3]  BOLD
 *   i=3..N/2      MID      theme[2]  NORMAL
 *   i=N/2+1..N-2  DARK     theme[1]  NORMAL
 *   i=N-1..       FADE     theme[0]  DIM
 *
 *   Identical mapping to matrix_rain.c, fireworks_rain.c, and
 *   pulsar_rain.c — once you learn the bands here you know them
 *   everywhere.
 *
 * RAY LIFECYCLE
 * ─────────────
 *
 *   At ray_init: angle is set once, cos_a/sin_a baked.
 *
 *   At ray_reset (called whenever the ray dies):
 *     r_off       = uniform([−STAGGER_FRAC · max_r, 0])
 *                   (negative = "pre-emerged"; tail is below the
 *                   core for this many frames before the head
 *                   actually appears at distance 0).
 *     speed       = uniform([SPEED_MIN_CPS, SPEED_MAX_CPS])
 *     cache[i]    = rand_glyph()                   (all 16 cells)
 *
 *   At ray_tick (every frame):
 *     r_off      += speed · dt
 *     reroll cache cells with probability 1 − 1/KEEP_ONE_IN
 *     return r_off - RAY_TRAIL < max_r       (false → tail is gone,
 *                                              respawn the ray)
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. dt = wall-clock seconds since last frame, capped at DT_CAP_SEC.
 *  2. sun_tick(dt):
 *     For each ray:
 *       if ray_tick(ray, dt) returns false → ray_reset(ray)
 *  3. erase()
 *  4. sun_draw():
 *       For each ray:
 *         For i = 0..RAY_TRAIL-1:
 *           ri = ray->r_off - i
 *           break if ri < CORE_RESERVED_RADIUS
 *           col = cx + round(ri · cos_a)
 *           row = cy + round(ri · sin_a)
 *           paint ray->cache[i] in dist_attr(i)
 *       Draw '@' core at (cx, cy) — drawn LAST so it always wins.
 *  5. HUD: yellow status row 0; cyan hint strip row last.
 *  6. doupdate; sleep to TARGET_FPS cap.
 *
 * KEY FORMULAS
 * ────────────
 *   θ_i       = 2π · i / N_RAYS                  ray angle
 *   cos_a     = cos(θ)                            horizontal direction
 *   sin_a     = sin(θ) · ASPECT                   vertical, anisotropy-corrected
 *   col, row  = cx + round(ri · cos_a),
 *               cy + round(ri · sin_a)
 *   ri        = r_off - i                         distance for trail slot i
 *   r_off(t)  = r_off(0) + speed_cps · t          smooth radial motion
 *   max_r     = cols + rows / ASPECT              farthest corner from centre
 *   stagger   = -uniform([0, STAGGER_FRAC · max_r])
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Forgetting the ASPECT compression makes vertical rays look
 *    longer than horizontal — the sun becomes a tall ellipse. Test
 *    by removing the · ASPECT and watching the corona squash.
 *
 *  • The `if (ri < CORE_RESERVED_RADIUS) break;` in ray_draw is
 *    critical — without it, trails draw over the centre and the
 *    '@' flickers off whenever a tail crosses (cx, cy).
 *
 *  • Pass-2 '@' must be drawn AFTER all rays. Reordering = invisible
 *    core whenever any ray's trail covers the centre.
 *
 *  • r_off starts negative; off-by-one in the recycle test
 *    `r_off - RAY_TRAIL < max_r` would let rays leak beyond the
 *    screen and never reset. Verify with a long session: the cell
 *    at corner-of-screen never lights up after a few seconds.
 *
 *  • Resize calls sun_init which fully resets the cache and
 *    recomputes max_r/cx/cy. All in-flight rays are restaggered.
 *
 *  • SHIMMER_KEEP_ONE_IN extreme: KEEP_ONE_IN = 1 → never reroll
 *    (rays look like static lines). KEEP_ONE_IN very large →
 *    every frame rerolls every cell (looks like noise).
 *
 * HOW TO VERIFY
 * ─────────────
 *  • At startup the screen should fill in gradually over ~2 seconds
 *    (the stagger), not pop fully populated.
 *
 *  • Press 'r' to reset; the sun blooms outward from centre.
 *
 *  • Press space to pause: rays freeze in place; resume picks up
 *    exactly where they were.
 *
 *  • The '@' must remain visible at every moment — the most
 *    fragile bug. Watch for at least 30 seconds.
 *
 *  • Look at the white-headed cells: exactly N_RAYS = 180 white
 *    glyphs should be visible at any time (one per ray, once all
 *    rays have emerged).
 *
 *  • Cycle themes ('t'): hue ramp swaps among five palettes; head
 *    and core stay white in every theme; HUD stays bright yellow.
 *
 *  • Resize the terminal: rays re-init centred on the new midpoint
 *    with no crash; old trails are erased on next frame.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/* ── §1.1 frame rate ──────────────────────────────────────────────── */
enum {
    TARGET_FPS = 60,
};

/* ── §1.2 ray geometry ────────────────────────────────────────────── */
enum {
    /* N_RAYS — number of radial streams emanating from the core.
     * 180 rays = 2° angular spacing — dense enough to read as a
     * continuous corona; smaller leaves visible gaps. */
    N_RAYS    = 180,

    /* RAY_TRAIL — characters per ray tail. Also the array bound on
     * each ray's glyph cache. */
    RAY_TRAIL = 16,

    /* Brightness band boundaries (used by dist_attr). */
    TRAIL_HOT_END   = 2,                /* [0..2]      → BOLD          */
    TRAIL_WARM_END  = RAY_TRAIL / 2,    /* [3..N/2-1]  → NORMAL        */
                                        /* [N/2..N-1]  → DIM           */
};

/*
 * ASPECT — terminal cell height/width ratio correction.
 * Baked into sin_a (= sin θ · ASPECT) once per ray at init, so every
 * inner-loop position formula is just:
 *     col = cx + ri · cos_a
 *     row = cy + ri · sin_a
 * Without ASPECT, vertical rays walk twice as fast as horizontal ones
 * (terminal cells are ~2× tall as they are wide) and the sun becomes
 * a tall ellipse instead of round.
 */
#define ASPECT  0.45f

/*
 * CORE_RESERVED_RADIUS — distance from centre at which ray trails
 * stop drawing. The '@' core is at radius 0; we leave a 1-cell
 * guard so no trail can overwrite it. ray_draw breaks the inner
 * loop as soon as ri < this.
 */
#define CORE_RESERVED_RADIUS  1.0f

/* ── §1.3 ray motion (cells/sec — physical units) ─────────────────── */
/*
 * Each ray's speed is uniform-sampled in [SPEED_MIN_CPS, SPEED_MAX_CPS]
 * at spawn. Rays at different speeds produce the natural-looking
 * variance — fast rays sprint to the rim while slow ones still hug
 * the core.
 */
#define SPEED_MIN_CPS  30.0f
#define SPEED_MAX_CPS  80.0f

/*
 * STAGGER_FRAC — at spawn, each ray's r_off is uniform in
 * [-STAGGER_FRAC · max_r, 0]. Negative = "pre-emerged"; the ray's
 * tail is below the core for stagger/speed seconds before the head
 * actually crosses the core. With 0.55 the screen fills smoothly
 * over the first ~1-2 seconds rather than popping fully populated.
 */
#define STAGGER_FRAC  0.55f

/* ── §1.4 shimmer ─────────────────────────────────────────────────── */
/* Per-frame, each glyph has a 1-in-KEEP_ONE_IN chance of surviving;
 * the rest reroll. KEEP_ONE_IN = 4 → 75 % rerolled per frame. */
#define SHIMMER_KEEP_ONE_IN  4

/* ── §1.5 ncurses pair IDs ────────────────────────────────────────── */
enum {
    /* 1..5 — trail bands (theme-controlled) */
    SHADE_FADE     = 1,
    SHADE_DARK,
    SHADE_MID,
    SHADE_BRIGHT,
    SHADE_HOT,

    /* 6 — ray head, always white (theme-independent) */
    SHADE_HEAD,

    /* 7 — sun core '@', always white */
    SHADE_CORE,

    /* 8..9 — HUD spec, theme-independent */
    PAIR_HUD,
    PAIR_HINT,
};

/* ── §1.6 dt cap (spiral-of-death guard) ──────────────────────────── */
#define DT_CAP_SEC  0.10f

/* ── §1.7 timing primitives ───────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* ── §1.8 HUD ─────────────────────────────────────────────────────── */
#define HUD_BUF_LEN  72

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

/*
 * Theme — 5-colour wake ramp (FADE..HOT). SHADE_HEAD and SHADE_CORE
 * are always white, theme-independent. Every entry is in the bright
 * half of the 256-colour space (CLAUDE.md brightness rule: cube ≥ 24,
 * grayscale ≥ 240). Values 16-23 are deliberately excluded.
 *
 * Themes:
 *   solar   — amber / gold (the eponymous look)
 *   green   — Matrix green
 *   nova    — cool blue → white burst
 *   plasma  — purple / magenta corona
 *   fire    — red / orange flame
 */
typedef struct {
    const char *name;
    int         fg  [5];
    int         fg_8[5];
} Theme;

static const Theme k_themes[] = {
    { "solar",
      { 130, 166, 202, 214, 220 },
      { COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW  } },
    { "green",
      {  28,  34,  40,  46,  82 },
      { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN   } },
    { "nova",
      {  24,  33,  51, 159, 255 },
      { COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,    COLOR_WHITE   } },
    { "plasma",
      {  53,  57,  93, 129, 201 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA } },
    { "fire",
      {  52,  88, 124, 160, 196 },
      { COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_RED     } },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static bool g_has_256 = false;

/*
 * theme_apply — bind wake bands and head/core for the chosen theme.
 * PAIR_HUD and PAIR_HINT are NEVER touched here — they carry semantic
 * meaning that must not change with theme.
 */
static void theme_apply(int idx)
{
    const int *fg = g_has_256 ? k_themes[idx].fg : k_themes[idx].fg_8;
    init_pair(SHADE_FADE,   fg[0],       COLOR_BLACK);
    init_pair(SHADE_DARK,   fg[1],       COLOR_BLACK);
    init_pair(SHADE_MID,    fg[2],       COLOR_BLACK);
    init_pair(SHADE_BRIGHT, fg[3],       COLOR_BLACK);
    init_pair(SHADE_HOT,    fg[4],       COLOR_BLACK);
    init_pair(SHADE_HEAD,   COLOR_WHITE, COLOR_BLACK);
    init_pair(SHADE_CORE,   COLOR_WHITE, COLOR_BLACK);
}

/*
 * hud_pairs_init — bind PAIR_HUD and PAIR_HINT once at startup. Both
 * use the default terminal background (-1) so the HUD sits on the
 * user's real background instead of a forced black box.
 */
static void hud_pairs_init(void)
{
    init_pair(PAIR_HUD,  g_has_256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, g_has_256 ?  51 : COLOR_CYAN,   -1);
}

/*
 * dist_attr — map distance from ray head to ncurses attr.
 *
 *   i=0           HEAD     white     BOLD
 *   i=1           HOT      theme[4]  BOLD
 *   i=2           BRIGHT   theme[3]  BOLD
 *   i=3..N/2      MID      theme[2]  NORMAL
 *   i=N/2+1..N-2  DARK     theme[1]  NORMAL
 *   i=N-1..       FADE     theme[0]  DIM
 */
static attr_t dist_attr(int i)
{
    if (i == 0)              return COLOR_PAIR(SHADE_HEAD)   | A_BOLD;
    if (i == 1)              return COLOR_PAIR(SHADE_HOT)    | A_BOLD;
    if (i <= TRAIL_HOT_END)  return COLOR_PAIR(SHADE_BRIGHT) | A_BOLD;
    if (i <= TRAIL_WARM_END) return COLOR_PAIR(SHADE_MID);
    if (i <= RAY_TRAIL - 2)  return COLOR_PAIR(SHADE_DARK);
    return COLOR_PAIR(SHADE_FADE) | A_DIM;
}

/* ===================================================================== */
/* §4  sun — Ray + Sun, init, tick, draw                                  */
/* ===================================================================== */

/* ── §4.1 ASCII glyph pool + tiny utility ─────────────────────────── */

static const char k_glyphs[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789!@#$%^&*()-_=+[]{}|;:,.<>?/~`";

#define GLYPHS_LEN (int)(sizeof k_glyphs - 1)

static char  rand_glyph(void) { return k_glyphs[rand() % GLYPHS_LEN]; }
static float urand01   (void) { return (float)rand() / (float)RAND_MAX; }

/* ── §4.2 Ray type ────────────────────────────────────────────────── */

/*
 * Ray — one radial stream.
 *
 *   cos_a    pre-baked cos(θ); horizontal direction component.
 *            Set once at sun_init, never changes.
 *   sin_a    pre-baked sin(θ) · ASPECT; vertical direction with
 *            anisotropy correction baked in.
 *
 *   r_off    head distance from centre (cells, float).
 *            < 0 → pre-emergence (stagger delay; not yet visible).
 *            ≥ 1 → head visible at distance r_off.
 *
 *   speed    cells per second; constant for one ray's lifetime,
 *            re-randomised at every ray_reset.
 *
 *   cache[]  per-ray random ASCII glyphs; ~75 % reroll per frame.
 *
 * One Ray: 4 floats + 16 chars = 32 bytes. 180 rays = 5760 bytes.
 */
typedef struct {
    float cos_a, sin_a;
    float r_off;
    float speed;
    char  cache[RAY_TRAIL];
} Ray;

/* ── §4.3 ray_init / ray_reset ────────────────────────────────────── */

/*
 * ray_reset — re-stagger an already-initialised ray. Called when the
 * tail clears max_r so the ray can "loop" back to a fresh start.
 * Direction (cos_a, sin_a) is left untouched — only r_off, speed,
 * and the glyph cache are randomised.
 */
static void ray_reset(Ray *r, float max_r)
{
    r->r_off = -urand01() * STAGGER_FRAC * max_r;
    r->speed = SPEED_MIN_CPS + urand01() * (SPEED_MAX_CPS - SPEED_MIN_CPS);
    for (int i = 0; i < RAY_TRAIL; i++)
        r->cache[i] = rand_glyph();
}

/*
 * ray_init — set this ray's direction (cos_a, sin_a baked from
 * angle), then call ray_reset for fresh r_off/speed/cache.
 */
static void ray_init(Ray *r, float angle, float max_r)
{
    r->cos_a = cosf(angle);
    r->sin_a = sinf(angle) * ASPECT;       /* ASPECT BAKED IN */
    ray_reset(r, max_r);
}

/* ── §4.4 ray_tick — advance head, shimmer cache ─────────────────── */

/*
 * One per-frame physics step. Returns true while the ray is still
 * partially on screen, false once even the tail tip has cleared
 * max_r (caller respawns via ray_reset).
 */
static bool ray_tick(Ray *r, float dt, float max_r)
{
    r->r_off += r->speed * dt;

    /* Shimmer: reroll most cache cells (1-in-KEEP_ONE_IN survives). */
    for (int i = 0; i < RAY_TRAIL; i++)
        if (rand() % SHIMMER_KEEP_ONE_IN != 0)
            r->cache[i] = rand_glyph();

    return (r->r_off - (float)RAY_TRAIL) < max_r;
}

/* ── §4.5 ray_draw — paint head + trail ──────────────────────────── */

/*
 * Walk i = 0..RAY_TRAIL-1 from head outward (in trail-index space —
 * physically the trail goes INWARD from the head, but the index
 * grows from the bright end to the dim end). The break at
 * ri < CORE_RESERVED_RADIUS is critical: it keeps any trail from
 * stamping on the centre cell where the '@' core lives.
 *
 * The position formula is just two adds and two multiplies per
 * cell — all the trig was done at ray_init time and baked into
 * (cos_a, sin_a).
 */
static void ray_draw(const Ray *r, int cx, int cy, int cols, int rows)
{
    for (int i = 0; i < RAY_TRAIL; i++) {
        float ri = r->r_off - (float)i;
        if (ri < CORE_RESERVED_RADIUS) break;       /* leave centre alone */

        int col = cx + (int)roundf(ri * r->cos_a);
        int row = cy + (int)roundf(ri * r->sin_a);
        if (col < 0 || col >= cols || row < 0 || row >= rows) continue;

        attr_t attr = dist_attr(i);
        attron(attr);
        mvaddch(row, col, (chtype)(unsigned char)r->cache[i]);
        attroff(attr);
    }
}

/* ── §4.6 Sun type ────────────────────────────────────────────────── */

/*
 * Sun — collection of N_RAYS rays plus geometry and runtime state.
 *
 *   rays[]      one Ray per slot, indexed 0..N_RAYS-1.
 *   cx, cy      screen-cell centre.
 *   max_r       upper-bound recycle distance — when a ray's tail
 *               clears this, the ray respawns. Slightly larger than
 *               the corner-to-centre distance so trails fully exit
 *               before recycle.
 *   theme_idx   0..THEME_COUNT-1.
 *   paused      when true, sun_tick returns early.
 */
typedef struct {
    Ray  rays[N_RAYS];
    int  cx, cy;
    float max_r;
    int  theme_idx;
    bool paused;
} Sun;

/* ── §4.7 sun_init — geometry + initial ray placement ────────────── */

static void sun_init(Sun *s, int cols, int rows)
{
    s->cx        = cols / 2;
    s->cy        = rows / 2;
    s->paused    = false;
    s->theme_idx = 0;

    /*
     * max_r — conservative upper bound so rays travel until even
     * the longest tail clears the farthest screen corner.
     *
     *   horizontal ray:   must reach cols / 2
     *   vertical ray:     must reach (rows / 2) / ASPECT (because
     *                     sin_a has ASPECT baked in)
     *   diagonal corner:  bounded by cols + rows / ASPECT
     */
    s->max_r = (float)cols + (float)rows / ASPECT;

    for (int i = 0; i < N_RAYS; i++) {
        float angle = (float)i * (2.0f * (float)M_PI / (float)N_RAYS);
        ray_init(&s->rays[i], angle, s->max_r);
    }
}

/* sun_reset — re-stagger every ray without recomputing angles or
 * geometry. Used by the 'r' key to bloom a fresh sun. */
static void sun_reset(Sun *s)
{
    for (int i = 0; i < N_RAYS; i++)
        ray_reset(&s->rays[i], s->max_r);
}

/* ── §4.8 sun_tick — advance every ray ────────────────────────────── */

/*
 * For each ray, advance one frame. If the ray's tail has cleared
 * the screen, respawn it with a fresh stagger and speed (preserving
 * its angle). When paused, return immediately — rays freeze in
 * place but the screen continues to redraw, so the HUD stays live.
 */
static void sun_tick(Sun *s, float dt)
{
    if (s->paused) return;

    for (int i = 0; i < N_RAYS; i++) {
        if (!ray_tick(&s->rays[i], dt, s->max_r))
            ray_reset(&s->rays[i], s->max_r);
    }
}

/* ── §4.9 sun_draw_core — '@' on top of everything ────────────────── */

static void sun_draw_core(const Sun *s, int cols, int rows)
{
    if (s->cx < 0 || s->cx >= cols || s->cy < 0 || s->cy >= rows) return;
    attron(COLOR_PAIR(SHADE_CORE) | A_BOLD);
    mvaddch(s->cy, s->cx, '@');
    attroff(COLOR_PAIR(SHADE_CORE) | A_BOLD);
}

/* ── §4.10 sun_draw — orchestrator: 180 rays + core last ─────────── */

static void sun_draw(const Sun *s, int cols, int rows)
{
    /* Pass 1 — every ray's head + trail. */
    for (int i = 0; i < N_RAYS; i++)
        ray_draw(&s->rays[i], s->cx, s->cy, cols, rows);

    /* Pass 2 — core LAST, always wins overlap. */
    sun_draw_core(s, cols, rows);
}

/* ── §4.11 input helpers (used by app_handle_key) ─────────────────── */

static void sun_cycle_theme(Sun *s)
{
    s->theme_idx = (s->theme_idx + 1) % THEME_COUNT;
    theme_apply(s->theme_idx);
}

/* ===================================================================== */
/* §5  screen — ncurses init / present / HUD                              */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);              /* don't let stdin interrupt frame writes */
    start_color();
    use_default_colors();       /* lets HUD pairs use -1 background       */
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw_hud — required HUD per CLAUDE.md spec.
 *
 *   Row 0       PAIR_HUD  + A_BOLD  (yellow) — fps + state + theme
 *   Bottom row  PAIR_HINT + A_BOLD  (cyan)   — full key list
 *
 * Both pairs sit on default background (-1) so they stay legible
 * regardless of theme. theme_apply() never touches them.
 */
static void screen_draw_hud(const Screen *sc, double fps, const Sun *s)
{
    char buf[HUD_BUF_LEN];
    snprintf(buf, sizeof buf, " %5.1f fps  rays:%d  [%s] %s ",
             fps, N_RAYS, k_themes[s->theme_idx].name,
             s->paused ? "PAUSED " : "running");

    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  t:theme ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §6  app — signals, resize, variable-dt main loop                       */
/* ===================================================================== */

typedef struct {
    Sun                   sun;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/*
 * app_do_resize — recompute geometry + cache for the new screen
 * size, but preserve user-tuned theme and paused flag across the
 * resize.
 */
static void app_do_resize(App *app)
{
    int  saved_theme  = app->sun.theme_idx;
    bool saved_paused = app->sun.paused;

    screen_resize(&app->screen);
    sun_init(&app->sun, app->screen.cols, app->screen.rows);

    app->sun.theme_idx = saved_theme;
    app->sun.paused    = saved_paused;
    app->need_resize   = 0;
}

/* Map one keypress to an action. Returns false on quit. */
static bool app_handle_key(App *app, int ch)
{
    Sun *s = &app->sun;
    switch (ch) {

    case 'q': case 'Q': case 27 /* ESC */:
        return false;

    case ' ': case 'p': case 'P':
        s->paused = !s->paused;
        break;

    case 'r': case 'R':
        sun_reset(s);
        break;

    case 't': case 'T':
        sun_cycle_theme(s);
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

    screen_init(&app->screen);
    g_has_256 = (COLORS >= 256);
    sun_init(&app->sun, app->screen.cols, app->screen.rows);
    theme_apply(app->sun.theme_idx);
    hud_pairs_init();

    int64_t last_ns      = clock_ns();
    int64_t fps_accum_ns = 0;
    int     fps_frames   = 0;
    double  fps_display  = 0.0;
    const int64_t TICK_NS = NS_PER_SEC / TARGET_FPS;

    while (app->running) {

        /* (1) handle resize first so subsequent steps see the new size */
        if (app->need_resize) {
            app_do_resize(app);
            last_ns = clock_ns();
        }

        /* (2) measure dt, capped to prevent spiral-of-death */
        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - last_ns;
        last_ns        = now_ns;
        float   dt     = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        /* (3) drain input */
        for (int ch; (ch = getch()) != ERR; ) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }

        /* (4) advance the sun (every ray) */
        sun_tick(&app->sun, dt);

        /* (5) rolling fps display */
        fps_accum_ns += dt_ns;
        fps_frames++;
        if (fps_accum_ns >= NS_PER_SEC / 2) {
            fps_display = (double)fps_frames * 1e9
                        / (double)fps_accum_ns;
            fps_accum_ns = 0;
            fps_frames   = 0;
        }

        /* (6) draw + present */
        erase();
        sun_draw(&app->sun, app->screen.cols, app->screen.rows);
        screen_draw_hud(&app->screen, fps_display, &app->sun);
        screen_present();

        /* (7) frame cap — sleep before the NEXT frame's I/O */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(TICK_NS - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
