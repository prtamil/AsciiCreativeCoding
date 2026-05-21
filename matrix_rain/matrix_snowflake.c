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
 *   §5  snow         — Snow type, pile state, freeze, full check;
 *                       snow_draw split into snow_cell_attr +
 *                       paint_snow_glyph + draw_snow_column
 *   §6  scene        — StreamPool + World + Mode + SimControls sub-structs;
 *                       Scene root (pool + snow + world + mode + sim +
 *                       theme_idx); FALL ↔ FLASH state machine;
 *                       scene_tick + scene_tick_one_stream split into
 *                       (A) tick_inactive_stream,
 *                       (B) park_stream_until_reset,
 *                       (C) tick_active_stream → freeze_head_into_pile
 *                           + random_restart_delay_sec helpers;
 *                       scene_draw two-pass painter (snow_draw +
 *                       draw_rain_pass with head_row_for_stream and
 *                       draw_rain_stream sub-helpers); input helpers
 *                       (scene_scale_rain_speed, scene_cycle_theme)
 *   §7  screen       — ncurses init / present; screen_draw_hud split
 *                       into hud_paint_text / state_label /
 *                       format_hud_status / draw_hud_status /
 *                       draw_hud_hint
 *   §8  app          — FpsCounter + App; signals, resize, key dispatch,
 *                       variable-dt main loop (7 numbered phases)
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
 * Data-structure: Scene = StreamPool + Snow + World + Mode +
 *                 SimControls + theme_idx, each a named sub-struct
 *                 (see §6 for the layered-ownership diagram).
 *                 StreamPool wraps RainStream[COLS_MAX] (one stream
 *                 per terminal column).  Snow owns
 *                 pile_chars[rows][cols] (the frozen glyph at each
 *                 cell, 0 if empty) + pile_top[cols] (topmost
 *                 frozen row per column, decreasing as snow grows).
 *                 Each RainStream owns its own glyph cache; no shared
 *                 2D rain grid.  All allocations are static — nothing
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
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 *   ── Visual heritage (the EFFECT this file extends) ─────────────
 *   [1] *The Matrix* (1999, dir. Wachowski) — the original
 *       vertical-stream green-code visual.  Title sequence by
 *       Simon Whiteley; on-screen "code" used MIRROR-IMAGE katakana
 *       plus Latin digits.  This file uses ASCII for portability
 *       across non-UTF-8 terminals (CLAUDE.md §"ASCII-Only Rendering").
 *   [2] Wikipedia, "Matrix digital rain" — history of the effect,
 *       Simon Whiteley's font, demoscene precedents from the 1990s.
 *       https://en.wikipedia.org/wiki/Matrix_digital_rain
 *
 *   ── Per-column accumulation (the SNOW PILE half) ───────────────
 *   [3] Bak, P., Tang, C. & Wiesenfeld, K. (1988), "Self-organized
 *       criticality", Phys. Rev. A 38(1), pp. 364-374 — the original
 *       Abelian Sandpile model.  Each terminal column here is a 1-D
 *       sandpile (sand grain = frozen glyph; pile_top[c] = current
 *       sand level).  We DELIBERATELY skip the lateral cascade
 *       (no toppling when pile reaches a threshold) to keep the
 *       algorithm focused — see "Why no lateral cascading" note in
 *       Snow struct doc (§5).
 *   [4] Wikipedia, "Abelian sandpile model" — quick reference for
 *       the cellular-automaton ancestry.
 *       https://en.wikipedia.org/wiki/Abelian_sandpile_model
 *
 *   ── Per-stream particle dynamics (the RAIN half) ───────────────
 *   [5] Reeves, W. T. (1983), "Particle Systems — A Technique for
 *       Modeling a Class of Fuzzy Objects", ACM SIGGRAPH '83 / ACM
 *       TOG 2(2), pp. 91-108 — canonical particle-systems paper.
 *       Each RainStream in §4 is a Reeves particle in 1-D (vertical
 *       lane) with state (head, speed, trail_len, glyphs[], active),
 *       and the spawn → fall → freeze → respawn lifecycle is
 *       Reeves's "expire → recycle" pattern with the freeze
 *       event substituting for off-screen expiry.
 *
 *   ── Two-state FSM driving the outer rhythm ─────────────────────
 *   [6] Hopcroft, J. E. & Ullman, J. D. (1979), "Introduction to
 *       Automata Theory, Languages, and Computation", Addison-Wesley
 *       — the formal FSM apparatus underlying §6 SceneState.
 *       FALL → FLASH → reset → FALL is a Moore-machine 2-state
 *       cycle with transition triggers `snow_is_full()` and
 *       `flash_tick == 0`.
 *
 *   ── Variable-timestep game loop ────────────────────────────────
 *   [7] Fiedler, G. (2004, updated 2014), "Fix Your Timestep!",
 *       https://gafferongames.com/post/fix_your_timestep/ — the
 *       canonical case for FIXED-step physics.  This file
 *       deliberately uses VARIABLE dt: the rain is non-stiff
 *       (no springs, no fast oscillators) and a single dt per frame
 *       is unconditionally stable.  Fiedler's DT_CAP rule (100 ms
 *       hard cap, DT_CAP_SEC here) is the only piece we adopt.
 *
 *   ── ncurses rendering substrate ────────────────────────────────
 *   [8] Raymond, E. S., "NCURSES Programming HOWTO" — §6 (colour
 *       pairs) for the theme-cycling design, §11 (output options)
 *       for the wnoutrefresh + doupdate diff-only write pattern
 *       that keeps the field flicker-free at 60 fps.
 *
 *   ── Companion files in this project ────────────────────────────
 *   See also:
 *     matrix_rain/matrix_rain.c    — same per-stream glyph-cache
 *       shimmer trick WITHOUT the snow pile.  Read this first if
 *       matrix rain is unfamiliar; this file is matrix_rain.c plus
 *       the Snow accumulator on top.
 *     matrix_rain/fireworks_rain.c — same shimmer-cache trick on
 *       ARC trajectories (rocket bursts) instead of vertical lanes.
 *     matrix_rain/pulsar_rain.c    — radial pulses; another shimmer-
 *       cache variant.
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
 *
 * Background you need
 * ───────────────────
 *   - matrix_rain T1-T5 (stream-per-column + shimmer + bands).
 *   - The painter's algorithm: draw far layer first, near layer on
 *     top. Snow first, rain over it; rain skips cells already
 *     occupied by snow.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - True falling-sand cellular automaton (sand cascades laterally
 *     into gaps). We DELIBERATELY skip lateral cascade — each
 *     column is its own independent accumulator. That's a
 *     pedagogical simplification; if you want real sand, see
 *     particle_systems/sandpile.c.
 *   - Particle-particle collision. There's no collision — only
 *     stream-versus-pile-top.
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
  ROWS_MAX = 100,
  COLS_MAX = 400,
  TARGET_FPS = 30,
};

/* ── §1.2 stream geometry & speed ─────────────────────────────────── */
enum {
  RAIN_TRAIL_MIN = 4,
  RAIN_TRAIL_MAX = 14, /* also array bound on glyphs[] */

  /* Top N cells of each stream reroll every frame for head shimmer.
   * Smaller = more stable trail; larger = noisier head zone. */
  RAIN_HEAD_FLICKER = 3,
};

/* Stream speeds (rows / second). Per-stream randomised in this range
 * so different columns fall at different rates. */
#define RAIN_SPEED_MIN 6.0f
#define RAIN_SPEED_MAX 20.0f

/* Per-frame chance to reroll a head-zone glyph. 0.67 ≈ rand()%3 != 0. */
#define RAIN_HEAD_REROLL_PROB 0.67f

/* Random delay between a stream freezing and its next spawn,
 * uniform([MIN, MIN+VAR]) seconds. Higher = more natural pause. */
#define STREAM_RESTART_MIN 0.30f
#define STREAM_RESTART_VAR 1.20f

/* On respawn, head_y starts uniform in [-INIT_OFFSCREEN_FRAC · rows,
 * 0) so streams enter staggered, not in lock-step. */
#define STREAM_INIT_OFFSCREEN_FRAC 0.4f

/* ── §1.3 global rain speed scale ([ / ] keys) ────────────────────── */
#define RAIN_SPEED_SCALE_DEF 1.0f
#define RAIN_SPEED_SCALE_MIN 0.25f
#define RAIN_SPEED_SCALE_MAX 4.0f
#define RAIN_SPEED_SCALE_STEP 1.25f

/* ── §1.4 snow ────────────────────────────────────────────────────── */
/* Top SNOW_FRESH_DEPTH rows of each column's pile render with
 * CP_SNOW_FRESH | A_BOLD ("fresh snow"); rows below use
 * CP_SNOW_PACKED ("packed snow"). 3 reads as a clear bright crust
 * over a duller mass. */
#define SNOW_FRESH_DEPTH 3

/* ── §1.5 scene state machine ─────────────────────────────────────── */
enum {
  /* Frames the FLASH state lasts before the pile clears.
   * 28 frames at 30 fps ≈ 0.93 s. */
  FLASH_FRAMES = 28,
};

/* ── §1.6 dt cap (spiral-of-death guard) ──────────────────────────── */
#define DT_CAP_SEC 0.10f

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
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define HUD_BUF_LEN 96

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Theme — one named palette combining a 3-tier rain ramp + a 2-tier
 * snow ramp.
 *
 * Intent
 *   Two visual layers share the screen — falling rain and accumulating
 *   snow.  Each theme pairs a rain hue with a CONTRASTING snow hue so
 *   the pile is always visually distinct from the rain falling onto
 *   it (rain on rain-coloured snow would dissolve into one mass).
 *
 *   The `t` key cycles through THEME_COUNT presets — `theme_apply()`
 *   re-registers the rain + snow colour pairs in one call.  Existing
 *   streams + pile cells stay in place (their PAIR_* numbers don't
 *   change); only the pair foregrounds get re-tinted.
 *
 * Why three rain tiers (head / mid / fade)
 *   Matrix-rain streams are drawn as a brightness gradient along
 *   the trail.  Three tiers strike the balance between visible
 *   structure (single colour would look like a coloured bar) and
 *   simplicity (more tiers add palette real-estate without changing
 *   the visual character).
 *
 * Why two snow tiers (fresh / packed)
 *   Newly-fallen snow within SNOW_FRESH_DEPTH of pile_top renders
 *   bold; deeper cells render normal.  Two tiers communicate "this
 *   cell just landed" vs "this cell has been here a while" — a tiny
 *   bit of temporal feedback without needing per-cell timestamps.
 *
 * Why HUD pairs are NOT in this struct
 *   PAIR_HUD and PAIR_HINT are theme-INDEPENDENT (canonical bright
 *   yellow + bright cyan per CLAUDE.md §"HUD Standard").  They're
 *   set ONCE in hud_pairs_init and survive every theme cycle.
 *
 * Members
 *   name      HUD label ("matrix", "lava", …).
 *   rain[3]   xterm-256 fg indices: [head bright, mid, fade dim].
 *   rain_8[3] ANSI-8 fallback indices, same ordering.
 *   snow[2]   xterm-256 fg indices: [fresh bold, packed normal].
 *   snow_8[2] ANSI-8 fallback indices, same ordering.
 *
 * Invariants
 *   Every entry sits in the bright half of the 256-colour space
 *   (CLAUDE.md brightness rule: cube ≥ 24, grayscale ≥ 240) so
 *   even dim tiers remain visible against the default-black bg.
 *   rain[i] perceptually ≤ rain[i+1] (monotone bright→dim along the
 *   array; the HEAD draws brightest).
 *   name != NULL.
 *
 * References
 *   CLAUDE.md §"Theme Palette Brightness" for the cube-floor rule.
 *   Same theme-cycling design as matrix_rain.c — see its Theme
 *   docblock for the per-tier rationale.
 */
typedef struct {
  const char *name;
  int rain[3];   /* head, mid, fade — 256-colour            */
  int rain_8[3]; /* 8-colour fallback                         */
  int snow[2];   /* fresh, packed — 256-colour              */
  int snow_8[2]; /* 8-colour fallback                         */
} Theme;

static const Theme k_themes[] = {
    {"Classic", /* green rain + white snow  */
     {46, 40, 28},
     {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN},
     {231, 195},
     {COLOR_WHITE, COLOR_WHITE}},

    {"Inferno", /* red rain + gold snow     */
     {196, 124, 88},
     {COLOR_RED, COLOR_RED, COLOR_RED},
     {226, 220},
     {COLOR_YELLOW, COLOR_YELLOW}},

    {"Nebula", /* purple rain + cyan snow  */
     {201, 165, 93},
     {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA},
     {159, 87},
     {COLOR_CYAN, COLOR_CYAN}},

    {"Toxic", /* cyan rain + pink snow    */
     {51, 39, 30},
     {COLOR_CYAN, COLOR_CYAN, COLOR_CYAN},
     {219, 207},
     {COLOR_MAGENTA, COLOR_MAGENTA}},

    {"Gold", /* yellow rain + lavender   */
     {226, 220, 178},
     {COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW},
     {183, 141},
     {COLOR_MAGENTA, COLOR_MAGENTA}},
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static bool g_has_256 = false;

/*
 * theme_apply — bind rain and snow pairs for the chosen theme.
 * PAIR_HUD and PAIR_HINT are NEVER touched here — they carry
 * semantic meaning that must not change with theme.
 */
static void theme_apply(int idx) {
  const Theme *t = &k_themes[idx];
  const int *rain = g_has_256 ? t->rain : t->rain_8;
  const int *snow = g_has_256 ? t->snow : t->snow_8;

  init_pair(CP_RAIN_HEAD, rain[0], -1);
  init_pair(CP_RAIN_MID, rain[1], -1);
  init_pair(CP_RAIN_FADE, rain[2], -1);
  init_pair(CP_SNOW_FRESH, snow[0], -1);
  init_pair(CP_SNOW_PACKED, snow[1], -1);
}

/*
 * hud_pairs_init — bind PAIR_HUD and PAIR_HINT once at startup. Both
 * use the default terminal background (-1) so the HUD sits on the
 * user's real background instead of a forced black box.
 */
static void hud_pairs_init(void) {
  init_pair(PAIR_HUD, g_has_256 ? 226 : COLOR_YELLOW, -1);
  init_pair(PAIR_HINT, g_has_256 ? 51 : COLOR_CYAN, -1);
}

/* ===================================================================== */
/* §4  stream — falling matrix-rain glyphs                                */
/* ===================================================================== */

/* ── §4.1 ASCII glyph pool + tiny utilities ──────────────────────── */

static const char k_glyphs[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
    "0123456789!#$%&*+-<>=?@^~|";

#define GLYPHS_LEN (int)(sizeof k_glyphs - 1)

static char rand_glyph(void) { return k_glyphs[rand() % GLYPHS_LEN]; }
static float urand01(void) { return (float)rand() / (float)RAND_MAX; }

/* ── §4.2 RainStream type ────────────────────────────────────────── */

/*
 * RainStream — one falling matrix-rain stream in a single terminal lane.
 *
 * Intent
 *   The atomic unit of the RAIN half of the demo.  Each RainStream
 *   owns ONE vertical lane (`col`) and represents a Reeves [5]
 *   particle in 1-D: a bright head + a trail of cached glyphs that
 *   reroll near the head to produce the shimmer.
 *
 *   Lifecycle (the FREEZE substitution for Reeves's standard expiry):
 *
 *     1. SPAWN   — stream_spawn picks random head_y (above screen),
 *                  random speed, random trail_len, fresh glyph cache,
 *                  active = true.
 *     2. FALL    — stream_advance: head += speed · dt · scale.
 *                  Top RAIN_HEAD_FLICKER cache slots reroll every
 *                  frame (the shimmer).
 *     3. FREEZE  — when head ≥ pile_top - 1 in its column,
 *                  scene_tick_one_stream calls snow_freeze with
 *                  glyphs[0] (the head glyph) and deactivates the
 *                  stream.  This is the FREEZE event substituting
 *                  for Reeves's off-screen expiry.
 *     4. WAIT    — restart_delay counts down dt seconds; on hit-zero
 *                  the stream respawns (back to step 1), unless its
 *                  column is full (pile_top == 0).
 *
 *   The genius of the demo: each FREEZE feeds the Snow accumulator
 *   by ONE glyph, so the pile grows from below at the rate the rain
 *   falls.  Different columns fill at different rates because each
 *   stream has its own random speed AND its own random restart_delay.
 *
 * Why per-stream glyph cache (not random-on-draw)
 *   Without the cache, each draw would generate a fresh random glyph
 *   at each trail cell — too much motion to track.  Caching means
 *   each glyph PERSISTS for many frames; only the top
 *   RAIN_HEAD_FLICKER positions re-roll per frame, giving the
 *   characteristic Matrix shimmer at the LEADING EDGE only.
 *   Same trick as matrix_rain.c — read that file first if the cache
 *   shimmer is unfamiliar.
 *
 * Why glyphs[0] is the head (not the tail)
 *   stream_draw + scene_draw iterate `dist = 0..trail_len-1` from
 *   head BACKWARD (up the column).  dist=0 is the head row drawn
 *   brightest, dist=trail_len-1 is the dimmest tip.  Indexing head
 *   at [0] makes the loop read naturally.
 *
 * Members
 *   ── set ONCE at spawn ────────────────────────────────────────
 *   col            Terminal column index (x).  stream_spawn(s, c, …)
 *                  sets s->col = c; never reassigned.
 *
 *   ── per-frame mutable state ──────────────────────────────────
 *   head           Current head ROW position (float).  Sub-row
 *                  precision so stream_advance can use fractional
 *                  speed·dt without integer stuttering.
 *   trail_len      Visible tail length.  Randomised at spawn from
 *                  [RAIN_TRAIL_MIN, RAIN_TRAIL_MAX].
 *   speed          Vertical velocity in CELLS / SECOND.  Constant
 *                  for this stream's lifetime; randomised across
 *                  streams at spawn (different lanes fall at
 *                  different rates → organic look, not synchronised
 *                  waterfall).
 *   glyphs[]       Cached random ASCII per trail slot.  glyphs[0]
 *                  is the HEAD (re-rolled every frame regardless of
 *                  shimmer cadence so the leading char stays fresh);
 *                  glyphs[1..RAIN_HEAD_FLICKER-1] re-roll every
 *                  frame; glyphs[RAIN_HEAD_FLICKER..trail_len-1]
 *                  persist for the full stream lifetime.
 *   active         true while the stream is falling.  Cleared by
 *                  FREEZE or column-full events; scene_tick_one_stream
 *                  ticks down restart_delay on inactive streams.
 *   restart_delay  Seconds remaining before this stream respawns.
 *                  Set by FREEZE event to a random value in
 *                  [STREAM_RESTART_MIN, … + STREAM_RESTART_VAR].
 *                  Only meaningful when !active.
 *
 * Invariants
 *   0 ≤ col < world.cols.
 *   active == true   ⇒  restart_delay irrelevant
 *   active == false  ⇒  restart_delay > 0 OR snow column full
 *                       (in which case delay is effectively never).
 *   RAIN_TRAIL_MIN ≤ trail_len ≤ RAIN_TRAIL_MAX.
 *
 * References
 *   [5] Reeves 1983 — the canonical particle-systems lifecycle;
 *       this struct is its 1-D specialisation.
 *   [1] Wachowski film — the visual heritage for the shimmer.
 *   See also matrix_rain.c §4 Column — same struct without
 *   restart_delay (no freeze event), useful for comparing the
 *   freeze-extension this file adds.
 */
typedef struct {
  int col;
  float head;
  int trail_len;
  float speed;
  char glyphs[RAIN_TRAIL_MAX];
  bool active;
  float restart_delay;
} RainStream;

/* ── §4.3 stream_spawn — fresh stream above the screen ───────────── */

static void stream_spawn(RainStream *s, int col, int rows) {
  s->col = col;
  s->head = -urand01() * STREAM_INIT_OFFSCREEN_FRAC * (float)rows;
  s->trail_len =
      RAIN_TRAIL_MIN + rand() % (RAIN_TRAIL_MAX - RAIN_TRAIL_MIN + 1);
  s->speed = RAIN_SPEED_MIN + urand01() * (RAIN_SPEED_MAX - RAIN_SPEED_MIN);
  s->active = true;
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
static bool stream_advance(RainStream *s, float dt, float scale, int land_row) {
  s->head += s->speed * dt * scale;

  /* Reroll top RAIN_HEAD_FLICKER cells with high probability —
   * visible head shimmer. The rest of the trail stays stable so
   * the falling stream reads as a coherent snake. */
  int n = s->trail_len < RAIN_HEAD_FLICKER ? s->trail_len : RAIN_HEAD_FLICKER;
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
static attr_t rain_band_attr(int dist, int trail_len) {
  if (dist == 0)
    return COLOR_PAIR(CP_RAIN_HEAD) | A_BOLD;
  if (dist <= trail_len / 2)
    return COLOR_PAIR(CP_RAIN_MID);
  return COLOR_PAIR(CP_RAIN_FADE) | A_DIM;
}

/* ===================================================================== */
/* §5  snow — accumulating per-column pile                                */
/* ===================================================================== */

/* ── §5.1 Snow type ──────────────────────────────────────────────── */

/*
 * Snow — the per-column accumulator pile (the SANDPILE half).
 *
 * Intent
 *   A 1-D Abelian Sandpile [3] per terminal column.  Each FREEZE event
 *   from a RainStream adds exactly ONE grain to ONE column's pile;
 *   pile_top[c] tracks the current sand level (top of the pile in
 *   column c).  The pile grows UPWARD over time — pile_top[c] starts
 *   at rows−1 (one past the bottom-most usable row) and DECREMENTS
 *   each FREEZE until it reaches 0 (column full).
 *
 *   The whole snow accumulator is the second half of this demo (the
 *   first half being the rain).  The rain-half feeds it grain by grain;
 *   the snow-half just renders the accumulator state.  scene_tick is
 *   the orchestrator that connects them via snow_freeze calls.
 *
 * Why no lateral cascading (the "departure from Bak-Tang-Wiesenfeld")
 *   The Abelian Sandpile model [3] has a TOPPLING RULE: when any cell
 *   exceeds a threshold (e.g. 4 grains), it distributes 1 grain to
 *   each of its 4 cardinal neighbours.  This is what gives sandpiles
 *   their self-organized criticality — large avalanches at all scales.
 *
 *   We DELIBERATELY skip this rule.  Each column accumulates
 *   independently.  Reasoning:
 *     • Pedagogical clarity: per-column accumulation maps directly
 *       to one stream → one column → one pile mental model.  Lateral
 *       cascading would require a parallel CA tick after each freeze.
 *     • Visual goal: a snowfall LOOKS like glyphs piling up below
 *       where they fell, not avalanching sideways.
 *     • Cost: lateral cascading is iterative until stable — adds
 *       a CA pass per frame.
 *
 *   The "Sandpile model" mention in [3]/[4] is for the ancestral
 *   data layout (pile_top[c] level per column), not the dynamics.
 *
 * Why a flat 2-D pile_chars (not a sparse data structure)
 *   The pile is naturally dense once columns start filling — within
 *   seconds at default rates every column has ≥ 1 cell.  A flat
 *   ROWS_MAX × COLS_MAX byte array is ~few KB and gives O(1) read
 *   per cell at draw time (no hash lookup).  Empty cells store 0,
 *   distinguished from valid ASCII glyphs (all ≥ 0x20).
 *
 * Members
 *   pile_chars[r][c]  Glyph that landed at this cell, or 0 if empty.
 *                     Set ONCE on freeze; never modified otherwise
 *                     (until scene_reset clears the whole array).
 *                     scene_draw paints non-zero cells with snow
 *                     colour pairs.
 *
 *   pile_top[c]       Row index of the topmost FROZEN row in column
 *                     c.  Rows [pile_top[c], rows-2] are frozen;
 *                     rows [0, pile_top[c]-1] are empty.  Decreases
 *                     (moves UP the screen) as the pile grows.
 *
 *                     Initial: rows-1 (one past the last usable row,
 *                     so the first freeze in the column lands at
 *                     pile_top[c] - 1 = rows - 2 = the bottom-most
 *                     usable row).  Reaches 0 when the column is full
 *                     (snow_is_full returns true ↔ every pile_top == 0).
 *
 *   cols, rows        Current usable extent.  Snow has its OWN copy
 *                     because helpers like snow_freeze access the
 *                     struct without a Scene pointer.  Scene.world
 *                     is the authoritative source; snow_init copies
 *                     from it.
 *                     Note: rows-1 is reserved for the HUD hint
 *                     strip; the pile only ever fills rows [0,rows-2].
 *
 *   frozen_count      Cumulative number of cells frozen since the
 *                     last reset.  Used by the HUD as a progress
 *                     readout (≈ how full the screen is).
 *
 * Invariants
 *   For every c ∈ [0, cols): 0 ≤ pile_top[c] ≤ rows-1.
 *   pile_top[c] == r  ⇒  pile_chars[r..rows-2][c] all nonzero AND
 *                        pile_chars[0..r-1][c] all zero.
 *   frozen_count == Σ_c (rows-1 − pile_top[c]).
 *   snow_is_full(snow)  ⇔  ∀c: pile_top[c] == 0.
 *
 * References
 *   [3] Bak, Tang & Wiesenfeld 1988 — the canonical sandpile paper.
 *       This struct's data layout (per-cell glyph grid + per-column
 *       level tracker) follows their conventions.  We omit the
 *       toppling rule — see "Why no lateral cascading" above.
 *   [4] Wikipedia Abelian sandpile model — quick reference.
 */
typedef struct {
  char pile_chars[ROWS_MAX][COLS_MAX];
  int pile_top[COLS_MAX];
  int cols, rows;
  int frozen_count;
} Snow;

/* ── §5.2 snow_init — empty pile across all columns ──────────────── */

static void snow_init(Snow *snow, int cols, int rows) {
  if (cols > COLS_MAX)
    cols = COLS_MAX;
  if (rows > ROWS_MAX)
    rows = ROWS_MAX;
  snow->cols = cols;
  snow->rows = rows;
  snow->frozen_count = 0;

  memset(snow->pile_chars, 0, sizeof snow->pile_chars);
  for (int c = 0; c < cols; c++)
    snow->pile_top[c] = rows - 1; /* one past the last usable row */
}

/* ── §5.3 snow_freeze — add one glyph at the top of column's pile ── */

/*
 * Freeze `glyph` at the topmost empty row in column `col`. The new
 * cell becomes pile_top[col] - 1; pile_top[col] then decrements to
 * record that the pile has grown by one row.
 *
 * No-ops when col is out of bounds or the column is already full.
 */
static void snow_freeze(Snow *snow, int col, char glyph) {
  if (col < 0 || col >= snow->cols)
    return;
  int land_row = snow->pile_top[col] - 1;
  if (land_row < 0)
    return;

  snow->pile_chars[land_row][col] = glyph;
  snow->pile_top[col] = land_row;
  snow->frozen_count++;
}

/* ── §5.4 snow_is_full — every column reached the top? ───────────── */

static bool snow_is_full(const Snow *snow) {
  for (int c = 0; c < snow->cols; c++)
    if (snow->pile_top[c] > 0)
      return false;
  return true;
}

/* ── §5.5 snow_draw — render the pile, fresh on top ──────────────── */

/* snow_cell_attr — pick the attr for ONE pile cell.
 *   depth_from_top = (r - pile_top[c]); 0 = the just-frozen cell at
 *   the top of the pile in this column.
 *   In STATE_FLASH, every cell renders as bright-fresh.
 *   Otherwise, cells within SNOW_FRESH_DEPTH of the top are BOLD
 *   ("fresh snow"); deeper cells are NORMAL ("packed snow"). */
static inline attr_t snow_cell_attr(int depth_from_top, bool flashing) {
  bool is_fresh_band = flashing || (depth_from_top < SNOW_FRESH_DEPTH);
  return is_fresh_band
      ? (COLOR_PAIR(CP_SNOW_FRESH)  | A_BOLD)
      :  COLOR_PAIR(CP_SNOW_PACKED);
}

/* paint_snow_glyph — attron / mvaddch / attroff sandwich for one cell. */
static inline void paint_snow_glyph(int r, int c, char glyph, attr_t attrs) {
  attron(attrs);
  mvaddch(r, c, (chtype)(unsigned char)glyph);
  attroff(attrs);
}

/* draw_snow_column — paint one column's pile from pile_top down to
 * the last simulation row (rows-2; rows-1 is reserved for HUD).
 * Skips zero-valued (empty) cells defensively, though by invariant
 * every cell at r ≥ pile_top is non-zero. */
static inline void draw_snow_column(const Snow *snow, int c,
                                     int last_sim_row, bool flashing) {
  int pile_top_row = snow->pile_top[c];
  for (int r = pile_top_row; r <= last_sim_row; r++) {
    char glyph = snow->pile_chars[r][c];
    if (glyph == 0) continue;
    int depth_from_top = r - pile_top_row;
    paint_snow_glyph(r, c, glyph, snow_cell_attr(depth_from_top, flashing));
  }
}

/*
 * snow_draw — paint the entire snow pile.
 *
 *   For each column c:
 *     walk r = pile_top[c] .. rows-2  (skip the HUD row at rows-1)
 *     paint each non-zero cell with snow_cell_attr(depth, flashing)
 *
 * The painter's-order trick: snow draws FIRST in scene_draw, then
 * rain paints over it ABOVE pile_top (rain never enters the pile).
 */
static void snow_draw(const Snow *snow, bool flashing) {
  const int LAST_SIM_ROW = snow->rows - 2;   /* rows-1 = HUD strip */
  for (int c = 0; c < snow->cols; c++)
    draw_snow_column(snow, c, LAST_SIM_ROW, flashing);
}

/* ===================================================================== */
/* §6  scene — Snow + Streams + FALL/FLASH state machine                  */
/* ===================================================================== */

/*
 * SceneState — the two-state FSM driving the demo's outer rhythm.
 *
 * State diagram
 *
 *      ┌───────────────────┐  snow_is_full(snow)  ┌───────────────────┐
 *      │   STATE_FALL      │ ───────────────────► │   STATE_FLASH     │
 *      │ (rain falls,      │                      │ (whole pile +     │
 *      │  feeds the pile)  │  flash_tick == 0     │  rain rendered    │
 *      └───────────────────┘ ◄─────── reset ───── │  in bright fresh) │
 *                              scene_reset        └───────────────────┘
 *
 *   STATE_FALL — the steady-state animation: streams drop, freeze on
 *                contact with the pile-top, restart after a cooldown.
 *                FSM advances when every column reaches the top
 *                (snow_is_full returns true).
 *
 *   STATE_FLASH — celebratory whiteout for FLASH_FRAMES frames; the
 *                ENTIRE pile renders with the bright-fresh attribute
 *                so the user sees "the screen filled".  When
 *                flash_tick decrements to 0, scene_reset wipes the
 *                pile and returns to STATE_FALL.
 *
 * Why STATE_FALL = 0 deliberately
 *   memset-initialised Scenes start in FALL without an explicit
 *   initialiser; scene_reset uses this implicitly.
 */
typedef enum { STATE_FALL = 0, STATE_FLASH = 1 } SceneState;

/*
 * StreamPool — fixed-capacity array of per-column rain streams.
 *
 * Intent
 *   Exactly one stream per terminal column (COLS_MAX bound, sized
 *   for worst-case width).  Index INTO the array == column number,
 *   so there's no lookup or mapping — `pool.streams[c]` IS column c.
 *
 *   Bundling the array into a named type makes the symmetry with the
 *   other matrix_rain demos explicit (ColumnPool in matrix_rain.c
 *   has the same one-stream-per-column design).
 *
 * Members
 *   streams[COLS_MAX]   Inline array; only the first `world.cols`
 *                       slots are touched per tick.  Slots beyond
 *                       world.cols sit dormant in BSS.
 *
 * Invariants
 *   For every i ∈ [0, world.cols): streams[i].col == i (set by
 *   stream_spawn; never reassigned during a stream's lifetime).
 */
typedef struct {
  RainStream streams[COLS_MAX];
} StreamPool;

/*
 * World — current terminal extent in cell-space.
 *
 * Intent
 *   Pixel-space is the same as cell-space in this demo (no aspect
 *   bridge), so `cols` and `rows` are simply the active grid.
 *   Refreshed from Screen in scene_init / app_do_resize.  Snow has
 *   its OWN cols/rows because it's a standalone type that helpers
 *   manipulate without a Scene pointer; World on Scene is the
 *   AUTHORITATIVE source-of-truth that Snow's copy is derived from.
 *
 * Members
 *   cols   Active grid width (= terminal width).
 *   rows   Active grid height (= terminal height; last row is HUD).
 *
 * Invariants
 *   cols > 0, rows > 1 (at least one usable simulation row + HUD).
 *   Snow.cols == world.cols, Snow.rows == world.rows after every
 *   scene_init / app_do_resize.
 */
typedef struct {
  int cols;
  int rows;
} World;

/*
 * SimControls — user-facing playback knobs.
 *
 * Intent
 *   The input handler writes these; scene_tick reads them.  Same
 *   shape as the SimControls sub-struct on every other demo's
 *   Scene in this project — a reader who knows one knows them all.
 *
 * Members
 *   paused             true → scene_tick early-returns; HUD shows
 *                      "PAUSED".  Toggled by SPACE / p key.
 *   rain_speed_scale   Multiplier on per-stream speeds.  Default 1.0;
 *                      live-tunable with [ and ] keys (multiplicative
 *                      step by RAIN_SPEED_SCALE_STEP per press).
 *                      Clamped to [RAIN_SPEED_SCALE_MIN, …MAX].
 *
 * Invariants
 *   RAIN_SPEED_SCALE_MIN ≤ rain_speed_scale ≤ RAIN_SPEED_SCALE_MAX.
 */
typedef struct {
  bool  paused;
  float rain_speed_scale;
} SimControls;

/*
 * Mode — current FSM state + the FLASH-state countdown.
 *
 * Intent
 *   Two pieces of state that always move together: the SceneState
 *   enum + the per-state counter.  Bundling them prevents the bug
 *   of "state == FLASH but flash_tick is stale from last cycle".
 *
 * Members
 *   state         Current SceneState (STATE_FALL or STATE_FLASH).
 *   flash_tick    Frames remaining in STATE_FLASH.  Unused in
 *                 STATE_FALL (set to 0 there for cleanliness).
 *
 * Invariants
 *   state == STATE_FALL  ⇒ flash_tick == 0 (set by scene_reset).
 *   state == STATE_FLASH ⇒ flash_tick > 0 (set by the transition
 *                         in scene_tick; decremented each frame).
 */
typedef struct {
  SceneState state;
  int        flash_tick;
} Mode;

/*
 * Scene — owns ALL simulation + render state for one run.
 *
 * Layered ownership
 *
 *     Scene
 *       ├── pool       : StreamPool    ← RainStream[COLS_MAX]
 *       ├── snow       : Snow          ← per-column accumulator (its
 *       │                                 own typed struct; see §5)
 *       ├── world      : World         ← active cell extent
 *       ├── mode       : Mode          ← FALL/FLASH FSM
 *       ├── sim        : SimControls   ← paused + rain_speed_scale
 *       └── theme_idx  : int           ← active colour palette
 *
 *   Every persistent value is reachable from one `Scene *s`.  Each
 *   sub-struct names its responsibility, so a reader scrolling the
 *   body of any helper can answer "is this touching sim or render?"
 *   from the field path alone.
 *
 * Why theme_idx is flat on Scene (not under render.*)
 *   No other render-only state needs grouping — promoting theme_idx
 *   to its own RenderControls sub-struct would be a single-field
 *   struct.  Keeping it flat is the cheaper abstraction.
 */
typedef struct {
  StreamPool  pool;
  Snow        snow;
  World       world;
  Mode        mode;
  SimControls sim;
  int         theme_idx;
} Scene;

/*
 * scene_reset — clear pile, respawn all streams. Mode-, theme-, and
 * speed-scale-state survive (the user wouldn't expect those to flip
 * on `r`).
 */
static void scene_reset(Scene *s) {
  snow_init(&s->snow, s->world.cols, s->world.rows);
  for (int c = 0; c < s->world.cols; c++) {
    stream_spawn(&s->pool.streams[c], c, s->world.rows);
    /* Spread initial heads across the full screen so the very
     * first frames show streams everywhere, not just at the top. */
    s->pool.streams[c].head = urand01() * (float)s->world.rows;
  }
  s->mode.state = STATE_FALL;
  s->mode.flash_tick = 0;
}

static void scene_init(Scene *s, int cols, int rows) {
  s->world.cols = cols;
  s->world.rows = rows;
  s->theme_idx = 0;
  s->sim.paused = false;
  s->sim.rain_speed_scale = RAIN_SPEED_SCALE_DEF;
  scene_reset(s);
}

/* ── §6.1 scene_tick_one_stream — per-column step, FALL state ───── */

/* Sentinel restart delay used to "park" a stream forever once its
 * column fills up.  The countdown never reaches zero before the
 * pile is reset, so the stream stays dormant. */
enum { STREAM_RESTART_NEVER_SEC = 1000000000 };

/* tick_inactive_stream — case (A) of scene_tick_one_stream.
 * Count down the respawn timer.  When it hits zero, respawn the
 * stream — UNLESS its column is already full (pile_top == 0), in
 * which case the stream stays parked (its dt subtraction below
 * keeps the delay near zero but the spawn check fails). */
static inline void tick_inactive_stream(Scene *s, RainStream *st,
                                         int c, float dt) {
  st->restart_delay -= dt;
  bool cooldown_done   = (st->restart_delay <= 0.0f);
  bool column_has_room = (s->snow.pile_top[c] > 0);
  if (cooldown_done && column_has_room)
    stream_spawn(st, c, s->world.rows);
}

/* park_stream_until_reset — case (B): the stream's column is full.
 * Set active=false and parks the restart timer so far in the future
 * that the stream stays dormant until scene_reset wipes the pile
 * and respawns everything. */
static inline void park_stream_until_reset(RainStream *st) {
  st->active        = false;
  st->restart_delay = (float)STREAM_RESTART_NEVER_SEC;
}

/* random_restart_delay_sec — uniform sample over
 * [STREAM_RESTART_MIN, STREAM_RESTART_MIN + STREAM_RESTART_VAR).
 * Per-stream randomness so the next batch of streams doesn't
 * respawn in lock-step after a synchronised freeze event. */
static inline float random_restart_delay_sec(void) {
  return STREAM_RESTART_MIN + urand01() * STREAM_RESTART_VAR;
}

/* freeze_head_into_pile — case (C) tail end.  The stream just
 * reached its land_row → snap glyphs[0] (the head glyph) into the
 * pile at column c, deactivate the stream, and schedule a randomised
 * respawn delay. */
static inline void freeze_head_into_pile(Scene *s, RainStream *st, int c) {
  snow_freeze(&s->snow, c, st->glyphs[0]);
  st->active        = false;
  st->restart_delay = random_restart_delay_sec();
}

/* tick_active_stream — case (C): advance physics, freeze on contact.
 * Falls through to freeze_head_into_pile if the stream's head
 * crossed the land_row this frame. */
static inline void tick_active_stream(Scene *s, RainStream *st,
                                       int c, int land_row, float dt) {
  bool hit_land = stream_advance(st, dt, s->sim.rain_speed_scale, land_row);
  if (hit_land)
    freeze_head_into_pile(s, st, c);
}

/*
 * scene_tick_one_stream — one column's FSM step within STATE_FALL.
 *
 *   (A) INACTIVE        → tick_inactive_stream  (cooldown + respawn)
 *   (B) COLUMN FULL     → park_stream_until_reset
 *   (C) ACTIVE          → tick_active_stream  (advance + maybe freeze)
 */
static void scene_tick_one_stream(Scene *s, int c, float dt) {
  RainStream *st = &s->pool.streams[c];

  /* (A) inactive — count down respawn timer */
  if (!st->active) {
    tick_inactive_stream(s, st, c, dt);
    return;
  }

  /* (B) column full — no more freezing possible until reset */
  int land_row = s->snow.pile_top[c] - 1;
  if (land_row < 0) {
    park_stream_until_reset(st);
    return;
  }

  /* (C) active — physics step; freeze if head crossed land_row */
  tick_active_stream(s, st, c, land_row, dt);
}

/* ── §6.2 scene_tick — orchestrator + state machine ─────────────── */

static void scene_tick(Scene *s, float dt) {
  if (s->sim.paused)
    return;

  if (s->mode.state == STATE_FLASH) {
    if (--s->mode.flash_tick <= 0)
      scene_reset(s);
    return;
  }

  /* STATE_FALL */
  for (int c = 0; c < s->world.cols; c++)
    scene_tick_one_stream(s, c, dt);

  if (snow_is_full(&s->snow)) {
    s->mode.state = STATE_FLASH;
    s->mode.flash_tick = FLASH_FRAMES;
  }
}

/* ── §6.3 scene_draw — snow first, rain on top above pile_top ───── */

/* paint_rain_cell — attron / mvaddch / attroff sandwich for one rain
 * cell, with the (chtype)(unsigned char) cast that prevents
 * sign-extension on chars > 127 (CLAUDE.md ncurses bug). */
static inline void paint_rain_cell(int r, int c, char glyph, attr_t attrs) {
  attron(attrs);
  mvaddch(r, c, (chtype)(unsigned char)glyph);
  attroff(attrs);
}

/* head_row_for_stream — round the stream's float head_y to the
 * integer row it currently OCCUPIES on screen.  +0.5f + floor =
 * "round half up" (deterministic at .5 boundaries, unlike banker's
 * roundf which alternates ties).  Same trick as matrix_rain.c. */
static inline int head_row_for_stream(const RainStream *st) {
  return (int)floorf(st->head + 0.5f);
}

/* draw_rain_stream — paint ONE active stream's visible trail.
 *
 *   For dist = 0..trail_len-1 from head BACKWARD (= UP the column),
 *   compute the trail-cell row, skip if off-screen, skip if at-or-
 *   below pile_top (rain never enters the pile), skip if on the HUD
 *   row, paint the cached glyph in the band's colour pair.
 *
 *   dist = 0 is the head (brightest); larger dist = dimmer tail.
 */
static inline void draw_rain_stream(const RainStream *st, int c,
                                     int pile_top_for_column,
                                     int hud_row) {
  int head_row = head_row_for_stream(st);
  for (int dist = 0; dist < st->trail_len; dist++) {
    int r = head_row - dist;
    bool off_screen_top   = (r < 0);
    bool inside_pile      = (r >= pile_top_for_column);
    bool on_hud_row       = (r >= hud_row);
    if (off_screen_top || inside_pile || on_hud_row) continue;
    paint_rain_cell(r, c, st->glyphs[dist],
                    rain_band_attr(dist, st->trail_len));
  }
}

/* draw_rain_pass — paint every active stream.  Per-column iteration:
 * inactive streams skip; active ones call draw_rain_stream, which
 * clips against pile_top + HUD row + screen top. */
static void draw_rain_pass(const Scene *s) {
  const int HUD_ROW = s->world.rows - 1;
  for (int c = 0; c < s->world.cols; c++) {
    const RainStream *st = &s->pool.streams[c];
    if (!st->active) continue;
    draw_rain_stream(st, c, s->snow.pile_top[c], HUD_ROW);
  }
}

/*
 * scene_draw — two-pass painter (snow first, rain on top above the
 * pile_top boundary).
 *
 *   PASS 1  snow_draw            — paints rows [pile_top[c], rows-2]
 *                                  for every column.
 *   PASS 2  draw_rain_pass       — paints rows above pile_top[c]
 *                                  for every ACTIVE stream.
 *
 * The two passes never overwrite each other because the rain pass
 * skips rows ≥ pile_top[c] (clipping done inside draw_rain_stream).
 */
static void scene_draw(const Scene *s) {
  bool flashing = (s->mode.state == STATE_FLASH);
  snow_draw    (&s->snow, flashing);      /* PASS 1 — snow pile  */
  draw_rain_pass(s);                       /* PASS 2 — rain trails */
}

/* ── §6.4 scene helpers used by app_handle_key ──────────────────── */

static void scene_scale_rain_speed(Scene *s, float factor) {
  s->sim.rain_speed_scale *= factor;
  if (s->sim.rain_speed_scale < RAIN_SPEED_SCALE_MIN)
    s->sim.rain_speed_scale = RAIN_SPEED_SCALE_MIN;
  if (s->sim.rain_speed_scale > RAIN_SPEED_SCALE_MAX)
    s->sim.rain_speed_scale = RAIN_SPEED_SCALE_MAX;
}

static void scene_cycle_theme(Scene *s) {
  s->theme_idx = (s->theme_idx + 1) % THEME_COUNT;
  theme_apply(s->theme_idx);
}

/* ===================================================================== */
/* §7  screen — ncurses init / present / HUD                              */
/* ===================================================================== */

/*
 * Screen — terminal cell extent + ncurses lifecycle wrapper.
 *
 * Intent
 *   Tracks the TERMINAL side of the demo: cell dimensions for HUD
 *   placement and field clipping.  ncurses owns the back buffer;
 *   this struct is the source-of-truth for cell-space dimensions,
 *   refreshed via getmaxyx in screen_init / screen_resize.
 *
 *   Scene.world.cols and Scene.world.rows TRACK this struct (they're
 *   the simulation's view of the same dimensions, copied in
 *   scene_init / app_do_resize).  Snow.cols and Snow.rows track
 *   Scene.world.  Three copies — but each lives in a struct whose
 *   helpers don't have access to the others (Snow helpers take
 *   `Snow *`, not Scene; ncurses helpers take `Screen *`).
 *
 * Why a tiny 2-field struct (not flat ints on App)
 *   • Lifecycle isolation: only screen_init / screen_resize /
 *     screen_free / screen_draw_hud touch ncurses' initscr / endwin /
 *     mvprintw.  They all take `Screen *` to make this layer
 *     explicit at the type level.
 *   • Symmetry with every other demo in this project — `{cols, rows}`
 *     is the canonical Screen shape.
 *
 * Members
 *   cols   Terminal width in CELLS (getmaxyx).
 *   rows   Terminal height in CELLS.  Bottom row (rows−1) hosts the
 *          HUD hint strip; the simulation paints into rows [0, rows−2].
 *
 * Invariants
 *   cols > 0, rows > 1 after screen_init.
 *   Both refreshed on every SIGWINCH via screen_resize +
 *   app_do_resize.
 */
typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1); /* don't let stdin interrupt frame writes */
  start_color();
  use_default_colors(); /* lets HUD pairs use -1 background       */
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) {
  (void)s;
  endwin();
}

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

/* hud_paint_text — attron / mvprintw / attroff sandwich shared by
 * both HUD rows.  Centralises the colour-pair setup. */
static inline void hud_paint_text(int row, int col, int pair, const char *text) {
  attron(COLOR_PAIR(pair) | A_BOLD);
  mvprintw(row, col, "%s", text);
  attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* state_label — three-way text tag for the Scene's macro state.
 * Mirrors the state diagram in SceneState's docblock + the paused
 * gate from SimControls. */
static inline const char *state_label(const Scene *s) {
  if (s->mode.state == STATE_FLASH) return "FLASH ";
  if (s->sim.paused)                return "PAUSED";
  return                                  "FALL  ";
}

/* format_hud_status — write the top-row status string into `buf`.
 * Reports fps, macro state, rain speed scale, frozen-cell count,
 * active theme name. */
static void format_hud_status(const Scene *s, double fps,
                              char *buf, size_t buflen) {
  snprintf(buf, buflen,
           " %5.1f fps  %s  rain:%.2fx  frozen:%d  [%s] ",
           fps, state_label(s),
           s->sim.rain_speed_scale, s->snow.frozen_count,
           k_themes[s->theme_idx].name);
}

/* draw_hud_status — right-aligned status row 0 (PAIR_HUD yellow). */
static void draw_hud_status(const Screen *sc, const Scene *s, double fps) {
  enum { HUD_TOP_ROW = 0 };
  char buf[HUD_BUF_LEN];
  format_hud_status(s, fps, buf, sizeof buf);
  int right_col = sc->cols - (int)strlen(buf);
  if (right_col < 0) right_col = 0;
  hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, buf);
}

/* draw_hud_hint — bottom-row key bindings strip (PAIR_HINT cyan). */
static void draw_hud_hint(const Screen *sc) {
  static const char *KEY_HINT =
      " q:quit  spc:pause  r:reset  []:rain speed  t:theme ";
  hud_paint_text(sc->rows - 1, 0, PAIR_HINT, KEY_HINT);
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
static void screen_draw_hud(const Screen *sc, double fps, const Scene *s) {
  draw_hud_status(sc, s, fps);
  draw_hud_hint  (sc);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §8  app — signals, resize, variable-dt main loop                       */
/* ===================================================================== */

/*
 * FpsCounter — rolling-window frame-rate estimator.
 *
 * Per-frame fps would jitter; this accumulates frame_count + elapsed
 * nanoseconds over a 500 ms window and emits a smoothed `display`
 * value each time the window fills.  Same shape as the FpsCounter on
 * every other file in this project.
 */
typedef struct {
  int     frame_count;
  int64_t window_ns;
  double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f) {
  f->frame_count = 0;
  f->window_ns   = 0;
  f->display     = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt_ns) {
  const int64_t FPS_WINDOW_NS = NS_PER_SEC / 2;       /* 500 ms */
  f->frame_count++;
  f->window_ns += dt_ns;
  if (f->window_ns < FPS_WINDOW_NS) return;
  f->display     = (double)f->frame_count
                 * (double)NS_PER_SEC / (double)f->window_ns;
  f->frame_count = 0;
  f->window_ns   = 0;
}

/*
 * App — top-level container for every persistent value.
 *
 *   scene         simulation + render state (Snow + Streams + FSM + …)
 *   screen        terminal cell extent + ncurses lifecycle
 *   fps           rolling-window fps estimator for HUD
 *   running       sig_atomic_t flag cleared by SIGINT / SIGTERM + 'q'
 *   need_resize   sig_atomic_t flag set by SIGWINCH; main reacts at
 *                 the top of the next iteration
 *
 * `g_app` is the program's only file-scope mutable state.  Signal
 * handlers reach the flags through it; everything else flows via
 * `App *app` parameter.
 */
typedef struct {
  Scene                 scene;
  Screen                screen;
  FpsCounter            fps;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

/*
 * app_do_resize — preserve user-tuned settings (theme, speed scale)
 * across the resize-induced scene_init.
 */
static void app_do_resize(App *app) {
  int   saved_theme = app->scene.theme_idx;
  float saved_speed = app->scene.sim.rain_speed_scale;

  screen_resize(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  app->scene.theme_idx          = saved_theme;
  app->scene.sim.rain_speed_scale = saved_speed;
  app->need_resize              = 0;
}

/* Map one keypress to an action. Returns false on quit. */
static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {

  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;

  case ' ':
  case 'p':
  case 'P':
    s->sim.paused = !s->sim.paused;
    break;

  case 'r':
  case 'R':
    scene_reset(s);
    break;

  case ']':
    scene_scale_rain_speed(s, RAIN_SPEED_SCALE_STEP);
    break;

  case '[':
    scene_scale_rain_speed(s, 1.0f / RAIN_SPEED_SCALE_STEP);
    break;

  case 't':
  case 'T':
    scene_cycle_theme(s);
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned int)clock_ns());

  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  fps_counter_init(&app->fps);

  screen_init(&app->screen);
  g_has_256 = (COLORS >= 256);
  theme_apply(0);
  hud_pairs_init();
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;
  int64_t last_ns = clock_ns();

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
    for (int ch; (ch = getch()) != ERR;) {
      if (!app_handle_key(app, ch)) {
        app->running = 0;
        break;
      }
    }

    /* (4) advance the scene state machine */
    scene_tick(&app->scene, dt);

    /* (5) rolling-window fps counter */
    fps_counter_tick(&app->fps, dt_ns);

    /* (6) draw + present */
    erase();
    scene_draw(&app->scene);
    screen_draw_hud(&app->screen, app->fps.display, &app->scene);
    screen_present();

    /* (7) frame cap — sleep before the NEXT frame's I/O */
    int64_t elapsed = clock_ns() - now_ns;
    clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
  }

  screen_free(&app->screen);
  return 0;
}
