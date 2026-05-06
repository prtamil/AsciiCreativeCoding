/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * matrix_snowflake.c — matrix rain falls, glyphs freeze, snow piles up
 *
 * DEMO: Streams of matrix-rain glyphs fall from the top of the screen.
 *       When a stream's head reaches the top of the snow pile in its
 *       column, that single glyph FREEZES into the pile — it changes
 *       colour from green-rain to white-snow, the pile grows by one
 *       row in that column, and the stream respawns above the screen
 *       to fall again. Different columns fill at different rates
 *       because each stream has its own random speed and respawn
 *       delay. When every column is full (pile reaches the top across
 *       the whole screen) the whole pile flashes bright for ~1 second
 *       and the world resets.
 *
 *           top of screen (rain spawns here)
 *           ┌─────────────────────────────────────┐
 *           │  .   k       3       z              │  ← falling streams
 *           │  .   7       Q       b              │     (bright head +
 *           │  X   .       .       9              │      fading trail)
 *           │  m   X       Y                      │
 *           │  .       .       .                  │
 *           │  .   .   .   .   .   .   .   .   .  │
 *           ├──── pile_top boundary, varies per col ─┤
 *           │  : * # +  : * # +  + * @            │  ← frozen pile
 *           │  # @ * : # @ * +  : # # @ * #       │     (snow colour,
 *           │  * # # # @ @ * #  @ # # # # +       │      fresh = BOLD,
 *           │  # @ # @ @ # # @  # # @ # # # @     │      packed = normal)
 *           ├─────────────────── HUD hint strip ──┤
 *           └─────────────────────────────────────┘
 *
 *       Two simulations, one per column, one boundary between them.
 *       That's it.
 *
 * Study alongside:
 *   matrix_rain/matrix_rain.c       — pure matrix rain, no pile.
 *                                     Read first if matrix rain is
 *                                     unfamiliar.
 *   matrix_rain/fireworks_rain.c    — same shimmer-cache trick but on
 *                                     arc trails and explosion bursts.
 *   particle_systems/sandpile.c     — if present, true falling-sand
 *                                     CA where chars cascade laterally
 *                                     into gaps. We deliberately do
 *                                     NOT do that here — each column
 *                                     accumulates independently for
 *                                     pedagogical simplicity.
 *
 * Section map:
 *   §1  config       — every tunable in one place, grouped by concept
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus 5 themed (rain, snow) palettes
 *   §4  stream       — RainStream type, spawn, advance, draw helpers
 *   §5  snow         — Snow type, pile state, freeze, full check, draw
 *   §6  scene        — Scene state, FALL ↔ FLASH state machine
 *   §7  screen       — ncurses init / present / HUD
 *   §8  app          — signals, resize, variable-dt main loop
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space / p        pause / resume
 *   r                reset (clear pile, respawn streams)
 *   ]   [            rain faster / slower
 *   t                cycle theme (5 themes)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra matrix_rain/matrix_snowflake.c \
 *       -o matrix_snowflake -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Two simulations stacked vertically, with a moving
 *                 boundary between them.
 *
 *                 (A) RAIN     — one matrix-rain stream per column,
 *                                each with a float head position, a
 *                                fixed-length trail, a random speed,
 *                                and a per-stream glyph cache whose
 *                                top three slots reroll every frame
 *                                for the head shimmer.
 *
 *                 (B) SNOW     — a per-column pile. When a stream's
 *                                head reaches `pile_top[c] - 1`, the
 *                                head glyph freezes at that cell, the
 *                                pile's top in that column rises by 1
 *                                row, and the stream respawns above
 *                                the screen after a short random
 *                                delay. The pile is render-only — it
 *                                doesn't move, just accumulates.
 *
 *                 (C) STATE    — a 2-state machine for the whole
 *                                scene: FALL (rain feeding the pile)
 *                                and FLASH (pile is complete; everything
 *                                glares bright for FLASH_FRAMES). After
 *                                FLASH, the pile clears and FALL
 *                                resumes. Loops forever.
 *
 * Data-structure: Scene owns Snow + RainStream[ncols]. Snow owns
 *                 pile_chars[rows][cols] (the frozen glyph at each
 *                 cell, 0 if empty) and pile_top[cols] (topmost
 *                 frozen row per column, decreasing as snow grows).
 *                 Each RainStream owns its own glyph cache; no shared
 *                 2D rain grid. All allocations are static — nothing
 *                 on the heap after startup.
 *
 * Rendering     : Two-pass painter's algorithm.
 *                 (1) SNOW pass: for each column, render frozen rows
 *                     pile_top[c]..rows-2 with snow colour. The top
 *                     SNOW_FRESH_DEPTH rows of each column's pile use
 *                     CP_SNOW_FRESH (BOLD) — visible "fresh snow"
 *                     band. Rows below use CP_SNOW_PACKED.
 *                 (2) RAIN pass: for each active stream, walk dist =
 *                     0..trail_len-1 from head backward, painting
 *                     glyphs[dist] in band(dist). Skip rows
 *                     ≥ pile_top[c] (where the snow already lives)
 *                     and the bottom HUD row.
 *
 * Performance   : O(cols · trail_len) per frame for rain draw,
 *                 O(cols · pile_height) for snow draw. With 200
 *                 columns × 14 trail × 30 fps, ~84k mvaddch/sec for
 *                 rain alone — microseconds. ncurses redraw dominates.
 *
 * References    :
 *   "The Matrix" (1999, Wachowski) — original vertical green-glyph rain.
 *   Wikipedia, "Matrix digital rain" — history and visual conventions.
 *     https://en.wikipedia.org/wiki/Matrix_digital_rain
 *   Wikipedia, "Sandpile model" — the cellular-automaton ancestor of
 *     the per-column accumulator we use here. We deliberately avoid
 *     lateral cascading to keep the algorithm focused.
 *     https://en.wikipedia.org/wiki/Abelian_sandpile_model
 *   This project, matrix_rain/matrix_rain.c — the same per-stream
 *     glyph-cache shimmer trick without the snow pile.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each terminal column is its own little accumulator. A stream falls
 * down it; when the stream's head reaches the top of the existing
 * snow pile in that column, the head glyph freezes there and the
 * pile rises by one row. The stream respawns above the screen after
 * a short delay and falls again, freezing its next head one row
 * higher. Repeat until the pile reaches the top of the column. When
 * EVERY column has filled, the whole screen flashes for ~1 s and the
 * world resets to empty + rain.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * A row of N hourglasses, side by side. In each hourglass the upper
 * chamber is the rain region (where a glyph stream is currently
 * falling) and the lower chamber is the snow pile. The "neck"
 * between chambers — the boundary where a falling glyph turns into
 * a frozen one — is the row index `pile_top[c] − 1`. As snow piles
 * up, that boundary moves UP the screen (smaller row indices). When
 * the boundary in every column reaches the top edge, the whole
 * setup glares bright and refills empty for another round.
 *
 * COLUMN SCHEMATIC (one column over time)
 * ───────────────────────────────────────
 *
 *   t = 0 (empty pile)        t = mid                     t = full
 *   ┌──┐                      ┌──┐                        ┌──┐
 *   │  │                      │  │                        │##│
 *   │  │                      │  │                        │##│
 *   │X │ ← head               │  │                        │##│
 *   │k │   trail              │X │                        │##│
 *   │7 │                      │k │ ← head                 │##│
 *   │  │                      │7 │                        │##│
 *   │  │                      │##│ ← pile_top moved up    │##│ ← pile_top = 0
 *   │  │                      │##│                        │##│   COLUMN FULL
 *   │##│ ← pile_top           │##│                        │##│
 *   │##│   (just bottom row)  │##│                        │##│
 *   ├──┤ ← HUD hint strip     ├──┤                        ├──┤
 *
 *   pile_top[c]:  rows-1      pile_top[c]: ~rows/2        pile_top[c]: 0
 *
 *   The pile_top moves UP (numerically smaller) as snow grows.
 *
 * SCENE STATE MACHINE
 * ───────────────────
 *
 *      ┌───────┐  every column full   ┌───────┐
 *      │ FALL  │ ──────────────────►  │ FLASH │
 *      │       │   (snow_is_full)      │       │
 *      │ rain  │                       │ pile  │
 *      │ feeds │                       │ glares│
 *      │ pile  │                       │ white │
 *      │       │  ← FLASH_FRAMES = 0 ──│       │
 *      └───────┘  (scene_reset clears  └───────┘
 *                  pile, respawns streams)
 *
 * ALGORITHM IN STEPS (per frame, FALL state)
 * ──────────────────────────────────────────
 *  1. dt = wall-clock seconds since previous frame, capped at 100 ms.
 *  2. For each column c:
 *      a. If stream is INACTIVE (cooling down between falls):
 *           restart_delay -= dt
 *           if delay ≤ 0 AND column not yet full: stream_spawn(c)
 *           continue.
 *      b. land_row = pile_top[c] - 1
 *         If land_row < 0: column full → deactivate stream forever
 *           until next reset.
 *      c. stream.head += stream.speed · dt · rain_speed_scale.
 *         Reroll top RAIN_HEAD_FLICKER glyphs with high probability
 *         (head shimmer).
 *      d. If round(stream.head) ≥ land_row:
 *           snow_freeze(c, stream.glyphs[0])
 *           stream.active = false
 *           stream.restart_delay = uniform(MIN, MIN+VAR) seconds.
 *  3. If snow_is_full() → state = FLASH, flash_tick = FLASH_FRAMES.
 *
 * RENDER
 *  4. erase()
 *  5. Pass 1 — snow_draw: for each column, render rows pile_top[c]
 *               ..rows-2. Top SNOW_FRESH_DEPTH rows of each column's
 *               pile in CP_SNOW_FRESH | BOLD; rest in CP_SNOW_PACKED.
 *  6. Pass 2 — rain_draw: for each active stream, walk trail. Skip
 *               cells where row ≥ pile_top[c] (snow lives there) or
 *               row ≥ rows-1 (HUD).
 *  7. HUD: yellow status row 0; cyan hint strip row rows-1.
 *  8. doupdate.
 *
 * KEY FORMULAS
 * ────────────
 *   pile_top[c]      row index of topmost frozen row in column c.
 *                    rows pile_top[c]..rows-2 are frozen;
 *                    rows 0..pile_top[c]-1 are empty.
 *                    Decreases as the pile grows.
 *
 *   land_row         pile_top[c] - 1 — the row a stream freezes AT
 *                    when its head reaches it.
 *
 *   head_row         floor(stream.head + 0.5) — round-half-up so
 *                    the head doesn't oscillate between rows at
 *                    fractional .5 (would happen with banker's
 *                    rounding from roundf()).
 *
 *   snow depth       depth = r - pile_top[c]
 *                    depth < SNOW_FRESH_DEPTH → fresh (BOLD)
 *                    depth ≥ SNOW_FRESH_DEPTH → packed (NORMAL)
 *
 *   snow_is_full     ∀c: pile_top[c] == 0
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Round-half-even bug. roundf(0.5) = 0 but roundf(1.5) = 2
 *    (banker's rounding). At head = N + 0.5 the head would oscillate
 *    between rows N and N+1 across frames. We use floor(x + 0.5)
 *    which always rounds UP at .5 — deterministic, no flicker.
 *
 *  • Glyph cache rerolls per FRAME (top 3 cells, 67% chance each).
 *    The rest of the trail stays stable so the falling stream reads
 *    as a coherent snake; only the head sparkles.
 *
 *  • Trail off the top. While head climbs from -trail_len upward,
 *    some trail rows have negative row indices. We clip via
 *    `if (r < 0 || r >= max_r) continue;` — no special case.
 *
 *  • Trail through the snow line. Once the head freezes, the rest
 *    of the trail vanishes immediately (stream deactivates). We do
 *    NOT animate the trail dissipating — keeps the algorithm clean.
 *
 *  • Resize (SIGWINCH) calls scene_init which clears the pile and
 *    respawns all streams. In-flight pile is lost. By design.
 *
 *  • Theme switch keeps frozen pile glyphs intact (just remaps the
 *    snow colour pairs to the new palette).
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press 'r' to reset; the pile clears, streams begin falling
 *    again from the top edge with staggered head positions.
 *
 *  • Watch one specific column. Each time a stream freezes there,
 *    the pile in that column should rise by exactly one row. After
 *    pile_height rises by 1, the next stream takes longer to reach
 *    the new pile top, so frozen-cell rate slows naturally as the
 *    column fills.
 *
 *  • Different columns fill at different rates because per-stream
 *    speed and restart_delay are randomised. The pile profile
 *    (pile_top vs col) should NEVER be flat — always wavy.
 *
 *  • '[' to slow the rain to ~0.25× — frozen-cell rate slows
 *    proportionally. Pile takes ~4× longer to fill.
 *
 *  • Press space to pause; the pile holds, no streams advance.
 *    Press space again; everything resumes.
 *
 *  • Press 't' through all five themes. Rain hue and snow hue
 *    should always contrast (e.g. green rain + white snow in
 *    Classic). HUD stays bright yellow / cyan in every theme.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/* ── §1.1 grid bounds + frame rate ────────────────────────────────── */
enum {
    ROWS_MAX        = 100,
    COLS_MAX        = 400,
    TARGET_FPS      = 30,
};

/* ── §1.2 stream geometry & speed ─────────────────────────────────── */
enum {
    RAIN_TRAIL_MIN     =  4,
    RAIN_TRAIL_MAX     = 14,           /* also array bound on glyphs[] */

    /* Top N cells of each stream reroll every frame for head shimmer.
     * Smaller = more stable trail; larger = noisier head zone. */
    RAIN_HEAD_FLICKER  =  3,
};

/* Stream speeds (rows / second). Per-stream randomised in this range
 * so different columns fall at different rates. */
#define RAIN_SPEED_MIN   6.0f
#define RAIN_SPEED_MAX  20.0f

/* Per-frame chance to reroll a head-zone glyph. 0.67 ≈ rand()%3 != 0. */
#define RAIN_HEAD_REROLL_PROB   0.67f

/* Random delay between a stream freezing and its next spawn,
 * uniform([MIN, MIN+VAR]) seconds. Higher = more natural pause. */
#define STREAM_RESTART_MIN     0.30f
#define STREAM_RESTART_VAR     1.20f

/* On respawn, head_y starts uniform in [-INIT_OFFSCREEN_FRAC · rows,
 * 0) so streams enter staggered, not in lock-step. */
#define STREAM_INIT_OFFSCREEN_FRAC  0.4f

/* ── §1.3 global rain speed scale ([ / ] keys) ────────────────────── */
#define RAIN_SPEED_SCALE_DEF   1.0f
#define RAIN_SPEED_SCALE_MIN   0.25f
#define RAIN_SPEED_SCALE_MAX   4.0f
#define RAIN_SPEED_SCALE_STEP  1.25f

/* ── §1.4 snow ────────────────────────────────────────────────────── */
/* Top SNOW_FRESH_DEPTH rows of each column's pile render with
 * CP_SNOW_FRESH | A_BOLD ("fresh snow"); rows below use
 * CP_SNOW_PACKED ("packed snow"). 3 reads as a clear bright crust
 * over a duller mass. */
#define SNOW_FRESH_DEPTH       3

/* ── §1.5 scene state machine ─────────────────────────────────────── */
enum {
    /* Frames the FLASH state lasts before the pile clears.
     * 28 frames at 30 fps ≈ 0.93 s. */
    FLASH_FRAMES = 28,
};

/* ── §1.6 dt cap (spiral-of-death guard) ──────────────────────────── */
#define DT_CAP_SEC  0.10f

/* ── §1.7 ncurses pair IDs ────────────────────────────────────────── */
enum {
    /* 1..3 — rain bands, theme-controlled */
    CP_RAIN_HEAD = 1,
    CP_RAIN_MID,
    CP_RAIN_FADE,

    /* 4..5 — snow bands, theme-controlled */
    CP_SNOW_FRESH,
    CP_SNOW_PACKED,

    /* 6..7 — HUD spec, theme-independent */
    PAIR_HUD,
    PAIR_HINT,
};

/* ── §1.8 timing + HUD ────────────────────────────────────────────── */
#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define HUD_BUF_LEN  96

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
 * Theme — a 3-colour rain ramp + a 2-colour snow ramp. Each theme
 * pairs a rain hue with a contrasting snow hue so the pile is always
 * visually distinct from the rain falling onto it.
 *
 * Every entry is in the bright half of the 256-colour space
 * (CLAUDE.md brightness rule: cube ≥ 24, grayscale ≥ 240). 8-colour
 * fallbacks are the closest stdcurses colour.
 */
typedef struct {
    const char *name;
    int  rain  [3];   /* head, mid, fade — 256-colour            */
    int  rain_8[3];   /* 8-colour fallback                         */
    int  snow  [2];   /* fresh, packed — 256-colour              */
    int  snow_8[2];   /* 8-colour fallback                         */
} Theme;

static const Theme k_themes[] = {
    { "Classic",                                /* green rain + white snow  */
      {  46,  40,  28 },
      { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN },
      { 231, 195 },
      { COLOR_WHITE, COLOR_WHITE } },

    { "Inferno",                                /* red rain + gold snow     */
      { 196, 124,  88 },
      { COLOR_RED, COLOR_RED, COLOR_RED },
      { 226, 220 },
      { COLOR_YELLOW, COLOR_YELLOW } },

    { "Nebula",                                 /* purple rain + cyan snow  */
      { 201, 165,  93 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA },
      { 159,  87 },
      { COLOR_CYAN, COLOR_CYAN } },

    { "Toxic",                                  /* cyan rain + pink snow    */
      {  51,  39,  30 },
      { COLOR_CYAN, COLOR_CYAN, COLOR_CYAN },
      { 219, 207 },
      { COLOR_MAGENTA, COLOR_MAGENTA } },

    { "Gold",                                   /* yellow rain + lavender   */
      { 226, 220, 178 },
      { COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW },
      { 183, 141 },
      { COLOR_MAGENTA, COLOR_MAGENTA } },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static bool g_has_256 = false;

/*
 * theme_apply — bind rain and snow pairs for the chosen theme.
 * PAIR_HUD and PAIR_HINT are NEVER touched here — they carry
 * semantic meaning that must not change with theme.
 */
static void theme_apply(int idx)
{
    const Theme *t = &k_themes[idx];
    const int *rain = g_has_256 ? t->rain : t->rain_8;
    const int *snow = g_has_256 ? t->snow : t->snow_8;

    init_pair(CP_RAIN_HEAD,   rain[0], -1);
    init_pair(CP_RAIN_MID,    rain[1], -1);
    init_pair(CP_RAIN_FADE,   rain[2], -1);
    init_pair(CP_SNOW_FRESH,  snow[0], -1);
    init_pair(CP_SNOW_PACKED, snow[1], -1);
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

/* ===================================================================== */
/* §4  stream — falling matrix-rain glyphs                                */
/* ===================================================================== */

/* ── §4.1 ASCII glyph pool + tiny utilities ──────────────────────── */

static const char k_glyphs[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
    "0123456789!#$%&*+-<>=?@^~|";

#define GLYPHS_LEN (int)(sizeof k_glyphs - 1)

static char  rand_glyph(void) { return k_glyphs[rand() % GLYPHS_LEN]; }
static float urand01(void)    { return (float)rand() / (float)RAND_MAX; }

/* ── §4.2 RainStream type ────────────────────────────────────────── */

/*
 * RainStream — one falling stream of glyphs.
 *
 *   col            terminal column index, set once at spawn.
 *   head           current head row, FLOAT for sub-row smoothness.
 *   trail_len      visible tail length.
 *   speed          rows / second, varied per stream.
 *   glyphs[]       per-stream glyph cache. glyphs[0] is the head;
 *                  glyphs[i] is i rows above the head. Top
 *                  RAIN_HEAD_FLICKER cells reroll every frame.
 *   active         true while the stream is falling.
 *   restart_delay  seconds remaining before the stream respawns
 *                  (only meaningful when !active).
 */
typedef struct {
    int    col;
    float  head;
    int    trail_len;
    float  speed;
    char   glyphs[RAIN_TRAIL_MAX];
    bool   active;
    float  restart_delay;
} RainStream;

/* ── §4.3 stream_spawn — fresh stream above the screen ───────────── */

static void stream_spawn(RainStream *s, int col, int rows)
{
    s->col           = col;
    s->head          = -urand01() * STREAM_INIT_OFFSCREEN_FRAC * (float)rows;
    s->trail_len     = RAIN_TRAIL_MIN
                     + rand() % (RAIN_TRAIL_MAX - RAIN_TRAIL_MIN + 1);
    s->speed         = RAIN_SPEED_MIN
                     + urand01() * (RAIN_SPEED_MAX - RAIN_SPEED_MIN);
    s->active        = true;
    s->restart_delay = 0.0f;
    for (int i = 0; i < s->trail_len; i++)
        s->glyphs[i] = rand_glyph();
}

/* ── §4.4 stream_advance — move head, flicker top glyphs ─────────── */

/*
 * One per-frame physics step. Returns true when the head has reached
 * or passed `land_row`, signalling the caller to freeze and respawn.
 * Returns false while the head is still above the pile.
 */
static bool stream_advance(RainStream *s, float dt, float scale,
                           int land_row)
{
    s->head += s->speed * dt * scale;

    /* Reroll top RAIN_HEAD_FLICKER cells with high probability —
     * visible head shimmer. The rest of the trail stays stable so
     * the falling stream reads as a coherent snake. */
    int n = s->trail_len < RAIN_HEAD_FLICKER
          ? s->trail_len : RAIN_HEAD_FLICKER;
    for (int i = 0; i < n; i++)
        if (urand01() < RAIN_HEAD_REROLL_PROB)
            s->glyphs[i] = rand_glyph();

    /* Round-half-up: floor(x + 0.5). roundf() uses banker's rounding
     * which would let the head linger at fractional .5 row positions
     * across frames. */
    int head_row = (int)floorf(s->head + 0.5f);
    return head_row >= land_row;
}

/* ── §4.5 rain_band_attr — distance → brightness band ─────────────── */

/*
 * Map distance from head to ncurses attr.
 *
 *   dist = 0                  → CP_RAIN_HEAD | A_BOLD  (bright head)
 *   dist 1..trail_len/2       → CP_RAIN_MID            (mid-fade)
 *   else (deep tail)          → CP_RAIN_FADE | A_DIM   (tail)
 */
static attr_t rain_band_attr(int dist, int trail_len)
{
    if (dist == 0)              return COLOR_PAIR(CP_RAIN_HEAD) | A_BOLD;
    if (dist <= trail_len / 2)  return COLOR_PAIR(CP_RAIN_MID);
    return COLOR_PAIR(CP_RAIN_FADE) | A_DIM;
}

/* ===================================================================== */
/* §5  snow — accumulating per-column pile                                */
/* ===================================================================== */

/* ── §5.1 Snow type ──────────────────────────────────────────────── */

/*
 * Snow — the pile state.
 *
 *   pile_chars[r][c]  glyph that landed at this cell, or 0 if empty.
 *                     Set once on freeze; never modified otherwise
 *                     (until scene reset clears the whole array).
 *
 *   pile_top[c]       Row index of the topmost frozen row in column
 *                     c. Rows pile_top[c]..rows-2 are frozen; rows
 *                     0..pile_top[c]-1 are empty. Decreases (moves
 *                     UP the screen) as the pile grows.
 *
 *                     Initial: rows-1 (one past the last usable row,
 *                     so the first freeze in the column lands at
 *                     pile_top[c] - 1 = rows - 2 = the bottom-most
 *                     usable row). Reaches 0 when the column is full.
 *
 *   cols, rows        Current usable extent (rows-1 is reserved for
 *                     the HUD hint strip; pile only fills 0..rows-2).
 *
 *   frozen_count      Total cells frozen since last reset; HUD readout.
 */
typedef struct {
    char pile_chars[ROWS_MAX][COLS_MAX];
    int  pile_top  [COLS_MAX];
    int  cols, rows;
    int  frozen_count;
} Snow;

/* ── §5.2 snow_init — empty pile across all columns ──────────────── */

static void snow_init(Snow *snow, int cols, int rows)
{
    if (cols > COLS_MAX) cols = COLS_MAX;
    if (rows > ROWS_MAX) rows = ROWS_MAX;
    snow->cols         = cols;
    snow->rows         = rows;
    snow->frozen_count = 0;

    memset(snow->pile_chars, 0, sizeof snow->pile_chars);
    for (int c = 0; c < cols; c++)
        snow->pile_top[c] = rows - 1;     /* one past the last usable row */
}

/* ── §5.3 snow_freeze — add one glyph at the top of column's pile ── */

/*
 * Freeze `glyph` at the topmost empty row in column `col`. The new
 * cell becomes pile_top[col] - 1; pile_top[col] then decrements to
 * record that the pile has grown by one row.
 *
 * No-ops when col is out of bounds or the column is already full.
 */
static void snow_freeze(Snow *snow, int col, char glyph)
{
    if (col < 0 || col >= snow->cols) return;
    int land_row = snow->pile_top[col] - 1;
    if (land_row < 0) return;

    snow->pile_chars[land_row][col] = glyph;
    snow->pile_top[col]             = land_row;
    snow->frozen_count++;
}

/* ── §5.4 snow_is_full — every column reached the top? ───────────── */

static bool snow_is_full(const Snow *snow)
{
    for (int c = 0; c < snow->cols; c++)
        if (snow->pile_top[c] > 0) return false;
    return true;
}

/* ── §5.5 snow_draw — render the pile, fresh on top ──────────────── */

/*
 * For each column, walk from pile_top down to rows-2 (skipping the
 * HUD row at rows-1). Cells within SNOW_FRESH_DEPTH of the column's
 * top render BOLD ("fresh snow"); deeper cells render NORMAL
 * ("packed snow"). In FLASH state every cell renders as bright fresh.
 */
static void snow_draw(const Snow *snow, bool flashing)
{
    int last_row = snow->rows - 1;          /* HUD lives at last_row */

    for (int c = 0; c < snow->cols; c++) {
        int top = snow->pile_top[c];

        for (int r = top; r < last_row; r++) {
            char g = snow->pile_chars[r][c];
            if (g == 0) continue;

            int    depth = r - top;          /* 0 = freshly fallen */
            attr_t attr;
            if (flashing || depth < SNOW_FRESH_DEPTH)
                attr = COLOR_PAIR(CP_SNOW_FRESH) | A_BOLD;
            else
                attr = COLOR_PAIR(CP_SNOW_PACKED);

            attron(attr);
            mvaddch(r, c, (chtype)(unsigned char)g);
            attroff(attr);
        }
    }
}

/* ===================================================================== */
/* §6  scene — Snow + Streams + FALL/FLASH state machine                  */
/* ===================================================================== */

typedef enum { STATE_FALL = 0, STATE_FLASH = 1 } SceneState;

/*
 * Scene — full per-frame state.
 *
 *   streams[]          one RainStream per terminal column.
 *   snow               the per-column accumulator.
 *   cols, rows         current terminal extents.
 *   state              FALL (rain feeding pile) or FLASH (pile full).
 *   flash_tick         frames remaining in FLASH state.
 *   theme_idx          index into k_themes[].
 *   paused             when true, scene_tick returns immediately.
 *   rain_speed_scale   global multiplier on stream speeds, [/]
 *                      keys tune in [MIN, MAX] multiplicatively.
 */
typedef struct {
    RainStream  streams[COLS_MAX];
    Snow        snow;
    int         cols, rows;
    SceneState  state;
    int         flash_tick;
    int         theme_idx;
    bool        paused;
    float       rain_speed_scale;
} Scene;

/*
 * scene_reset — clear pile, respawn all streams. Mode-, theme-, and
 * speed-scale-state survive (the user wouldn't expect those to flip
 * on `r`).
 */
static void scene_reset(Scene *s)
{
    snow_init(&s->snow, s->cols, s->rows);
    for (int c = 0; c < s->cols; c++) {
        stream_spawn(&s->streams[c], c, s->rows);
        /* Spread initial heads across the full screen so the very
         * first frames show streams everywhere, not just at the top. */
        s->streams[c].head = urand01() * (float)s->rows;
    }
    s->state      = STATE_FALL;
    s->flash_tick = 0;
}

static void scene_init(Scene *s, int cols, int rows)
{
    s->cols             = cols;
    s->rows             = rows;
    s->theme_idx        = 0;
    s->paused           = false;
    s->rain_speed_scale = RAIN_SPEED_SCALE_DEF;
    scene_reset(s);
}

/* ── §6.1 scene_tick_one_stream — per-column step, FALL state ───── */

/*
 * Advance one stream by one frame. Three cases:
 *   (A) Stream is INACTIVE → count down restart_delay, respawn
 *                            when it hits zero (unless column full).
 *   (B) Stream's column is FULL → deactivate forever (until reset).
 *   (C) Stream is ACTIVE → physics step + check freeze line.
 *                          If head reached land_row, freeze head
 *                          glyph onto pile and start cooldown.
 */
static void scene_tick_one_stream(Scene *s, int c, float dt)
{
    RainStream *st = &s->streams[c];

    /* (A) inactive — count down respawn timer */
    if (!st->active) {
        st->restart_delay -= dt;
        if (st->restart_delay <= 0.0f && s->snow.pile_top[c] > 0)
            stream_spawn(st, c, s->rows);
        return;
    }

    int land_row = s->snow.pile_top[c] - 1;

    /* (B) column full — no more freezing possible until reset */
    if (land_row < 0) {
        st->active        = false;
        st->restart_delay = 1e9f;          /* effectively never        */
        return;
    }

    /* (C) advance physics; freeze if the head crossed the pile top */
    if (stream_advance(st, dt, s->rain_speed_scale, land_row)) {
        snow_freeze(&s->snow, c, st->glyphs[0]);
        st->active        = false;
        st->restart_delay = STREAM_RESTART_MIN
                          + urand01() * STREAM_RESTART_VAR;
    }
}

/* ── §6.2 scene_tick — orchestrator + state machine ─────────────── */

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    if (s->state == STATE_FLASH) {
        if (--s->flash_tick <= 0) scene_reset(s);
        return;
    }

    /* STATE_FALL */
    for (int c = 0; c < s->cols; c++)
        scene_tick_one_stream(s, c, dt);

    if (snow_is_full(&s->snow)) {
        s->state      = STATE_FLASH;
        s->flash_tick = FLASH_FRAMES;
    }
}

/* ── §6.3 scene_draw — snow first, rain on top above pile_top ───── */

static void scene_draw(const Scene *s)
{
    bool flashing = (s->state == STATE_FLASH);
    int  last_row = s->rows - 1;

    /* Pass 1 — snow pile (paints rows pile_top[c]..rows-2). */
    snow_draw(&s->snow, flashing);

    /* Pass 2 — rain trails (above pile_top in each column). */
    for (int c = 0; c < s->cols; c++) {
        const RainStream *st = &s->streams[c];
        if (!st->active) continue;

        int max_r    = s->snow.pile_top[c];          /* exclusive upper */
        int head_row = (int)floorf(st->head + 0.5f);

        for (int d = 0; d < st->trail_len; d++) {
            int r = head_row - d;
            if (r < 0 || r >= max_r || r >= last_row) continue;

            attr_t attr = rain_band_attr(d, st->trail_len);
            attron(attr);
            mvaddch(r, c, (chtype)(unsigned char)st->glyphs[d]);
            attroff(attr);
        }
    }
}

/* ── §6.4 scene helpers used by app_handle_key ──────────────────── */

static void scene_scale_rain_speed(Scene *s, float factor)
{
    s->rain_speed_scale *= factor;
    if (s->rain_speed_scale < RAIN_SPEED_SCALE_MIN)
        s->rain_speed_scale = RAIN_SPEED_SCALE_MIN;
    if (s->rain_speed_scale > RAIN_SPEED_SCALE_MAX)
        s->rain_speed_scale = RAIN_SPEED_SCALE_MAX;
}

static void scene_cycle_theme(Scene *s)
{
    s->theme_idx = (s->theme_idx + 1) % THEME_COUNT;
    theme_apply(s->theme_idx);
}

/* ===================================================================== */
/* §7  screen — ncurses init / present / HUD                              */
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
 *   Row 0       PAIR_HUD  + A_BOLD  (yellow) — fps + state + params
 *   Bottom row  PAIR_HINT + A_BOLD  (cyan)   — full key list
 *
 * Both pairs sit on default background (-1) so they stay legible
 * regardless of theme. theme_apply() never touches them.
 */
static void screen_draw_hud(const Screen *sc, double fps, const Scene *s)
{
    char buf[HUD_BUF_LEN];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %s  rain:%.2fx  frozen:%d  [%s] ",
             fps,
             s->state == STATE_FLASH ? "FLASH " :
             (s->paused ? "PAUSED" : "FALL  "),
             s->rain_speed_scale,
             s->snow.frozen_count,
             k_themes[s->theme_idx].name);

    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  []:rain speed  t:theme ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app — signals, resize, variable-dt main loop                       */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/*
 * app_do_resize — preserve user-tuned settings (theme, speed scale)
 * across the resize-induced scene_init.
 */
static void app_do_resize(App *app)
{
    int   saved_theme = app->scene.theme_idx;
    float saved_speed = app->scene.rain_speed_scale;

    screen_resize(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    app->scene.theme_idx        = saved_theme;
    app->scene.rain_speed_scale = saved_speed;
    app->need_resize            = 0;
}

/* Map one keypress to an action. Returns false on quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {

    case 'q': case 'Q': case 27 /* ESC */:
        return false;

    case ' ': case 'p': case 'P':
        s->paused = !s->paused;
        break;

    case 'r': case 'R':
        scene_reset(s);
        break;

    case ']':
        scene_scale_rain_speed(s, RAIN_SPEED_SCALE_STEP);
        break;

    case '[':
        scene_scale_rain_speed(s, 1.0f / RAIN_SPEED_SCALE_STEP);
        break;

    case 't': case 'T':
        scene_cycle_theme(s);
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
    theme_apply(0);
    hud_pairs_init();
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

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

        /* (4) advance the scene state machine */
        scene_tick(&app->scene, dt);

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
        scene_draw(&app->scene);
        screen_draw_hud(&app->screen, fps_display, &app->scene);
        screen_present();

        /* (7) frame cap — sleep before the NEXT frame's I/O */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(TICK_NS - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
