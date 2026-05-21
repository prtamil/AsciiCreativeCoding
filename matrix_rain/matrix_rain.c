/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * matrix_rain.c — falling green code from The Matrix, in your terminal
 *
 * DEMO: Each terminal column hosts an independent "stream" — a bright
 *       head that marches downward and a fading trail behind it. The
 *       glyphs are random ASCII letters/digits/punctuation that reroll
 *       every tick, so the same falling trajectory looks like
 *       continuous shimmer instead of a static curve.
 *
 *       Close-up of one column (time freezes, head is at the bottom):
 *
 *               col_x
 *                │
 *           ┌── row 0
 *           │     b   ← glyphs[7]   FADE   dim
 *           │     k   ← glyphs[6]   DARK
 *           │     7   ← glyphs[5]   DARK
 *           │     2   ← glyphs[4]   MID
 *           │     Q   ← glyphs[3]   MID
 *           │     m   ← glyphs[2]   BRIGHT bold
 *           │     g   ← glyphs[1]   HOT    bold
 *           ▼     A   ← glyphs[0]   HEAD   white bold     ← "live" head
 *               row N
 *
 *       The HEAD is always pure white. Behind it, five theme-coloured
 *       bands fade from HOT (brightest) to FADE (darkest). Press 't'
 *       to cycle four themes — green / amber / blue / white.
 *
 * Study alongside:
 *   matrix_rain/fireworks_rain.c   — same shimmer-cache trick on
 *                                    arc trails instead of vertical
 *                                    streams.
 *   matrix_rain/pulsar_rain.c      — radial pulses with the same
 *                                    rerolling-glyph effect.
 *   matrix_rain/sun_rain.c         — sun-shaped variant.
 *
 * Section map:
 *   §1  config       — every tunable in one place, grouped by concept
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus 4 themed 5-shade palettes
 *   §4  column       — Column type, spawn, tick, draw, shade bands
 *   §5  scene        — ColumnPool + SimControls sub-structs; Scene root;
 *                       scene_init / _tick / _draw / _free + named
 *                       sub-helpers (seed_initial_streams,
 *                       tick_shimmer_pulse, tick_one_column,
 *                       respawn_probability_per_frame); input helpers
 *                       (scene_scale_speed / _change_density /
 *                       _cycle_theme); scene_reset_preserving_knobs
 *                       (shared by 'r' key + SIGWINCH)
 *   §6  screen       — ncurses init / present; screen_draw_hud split
 *                       into hud_paint_text / format_hud_status /
 *                       draw_hud_status / draw_hud_hint
 *   §7  app          — FpsCounter + App; signals, resize, key dispatch,
 *                       variable-dt main loop (3a–3g numbered phases)
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space            pause / resume
 *   r                reset all columns
 *   ]   [            speed up / slow down
 *   = / +            denser  (more columns lit)
 *   -                sparser
 *   t                cycle theme (green / amber / blue / white)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra matrix_rain/matrix_rain.c \
 *       -o matrix_rain -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : One independent stream per terminal column. Each
 *                 stream owns:
 *                   - a FLOAT head row that advances by speed · dt
 *                     each frame (variable timestep, no accumulator);
 *                   - a fixed-length trail of cached glyphs that
 *                     reroll every tick, producing the shimmer;
 *                   - a constant per-stream speed (rows/sec) so
 *                     different streams visibly fall at different
 *                     rates.
 *                 When a stream's tail clears the bottom edge, the
 *                 column becomes inactive and may be respawned
 *                 above the screen with fresh randoms (random
 *                 head_y, length, speed, glyphs).
 *
 * Data-structure: Scene owns a ColumnPool (Column[ncols] flat array
 *                 indexed by terminal column) plus a SimControls
 *                 sub-struct (paused / speed / density / shimmer
 *                 timer) and a theme_idx.  Each Column carries the
 *                 head position, length, speed, glyph cache, and
 *                 active flag.  One heap allocation at scene_init
 *                 (ColumnPool.columns); no malloc per frame; no
 *                 Grid; no second framebuffer.  Rendering iterates
 *                 the Column array directly.
 *
 * Rendering     : For each active column, walk dist = 0..length-1
 *                 from head backward, mapping dist to a brightness
 *                 BAND (HEAD / HOT / BRIGHT / MID / DARK / FADE).
 *                 Round the float head_y to a terminal row using
 *                 floor(head_y - dist + 0.5) — "round half up",
 *                 deterministic at .5 boundaries (unlike banker's
 *                 roundf). Paint the cached glyph at that row with
 *                 the band's colour pair and attribute. The HEAD
 *                 band is always pure white regardless of theme.
 *
 * Performance   : O(cols · trail_len) per frame — at 100 cols × 24
 *                 trail = 2400 mvaddch per frame, microseconds.
 *                 Glyph cache is rerolled per simulation tick (not
 *                 per render frame), so the shimmer rate is decoupled
 *                 from the smoothness of fall.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 *   ── Visual heritage (the EFFECT this file reproduces) ──────────
 *   [1] *The Matrix* (1999, dir. Wachowski) — the original
 *       vertical-stream green-code visual.  Title sequence by
 *       Simon Whiteley; the on-screen "code" used MIRROR-IMAGE
 *       katakana plus Latin digits.  This file uses ASCII for
 *       portability across non-UTF-8 terminals (CLAUDE.md
 *       §"ASCII-Only Rendering").
 *   [2] Wikipedia, "Matrix digital rain" — history of the effect,
 *       Simon Whiteley's font, demoscene precedents from the
 *       1990s.  https://en.wikipedia.org/wiki/Matrix_digital_rain
 *
 *   ── Per-column streams (the underlying data layout) ────────────
 *   [3] Reeves, W. T. (1983), "Particle Systems — A Technique for
 *       Modeling a Class of Fuzzy Objects", ACM SIGGRAPH '83 / ACM
 *       TOG 2(2), pp. 91-108 — the canonical particle-system paper.
 *       Each Column in §4 is a Reeves particle in 1-D (vertical
 *       lane) with state vector (head_y, speed, trail_len, glyphs[],
 *       active).  The respawn-on-tail-clear lifecycle is Reeves's
 *       "expire → recycle" pattern.
 *
 *   ── Sub-cell smooth motion ─────────────────────────────────────
 *   [4] Catmull, E. (1974), "A Subdivision Algorithm for Computer
 *       Display of Curved Surfaces", PhD thesis, Univ. of Utah —
 *       early work on sub-pixel rendering precision.  We use a
 *       FLOAT head_y + `floor(head_y - dist + 0.5)` round-half-up
 *       so streams glide continuously between integer rows instead
 *       of jumping; same idea at terminal-cell scale.
 *
 *   ── Poisson respawn process ────────────────────────────────────
 *   [5] Ross, S. M. (2014), "Introduction to Probability Models"
 *       (11th ed.), Ch. 5 — Poisson processes.  The per-frame
 *       respawn check
 *         P(spawn) = RESPAWN_RATE_PER_SEC · dt / density
 *       is the discrete approximation of a Poisson process with
 *       intensity RATE/density.  At small dt this converges to a
 *       true Poisson interarrival distribution — streams wake up
 *       at roughly exponential intervals.
 *
 *   ── Variable-timestep game loop ────────────────────────────────
 *   [6] Fiedler, G. (2004, updated 2014), "Fix Your Timestep!",
 *       https://gafferongames.com/post/fix_your_timestep/ — the
 *       canonical case for FIXED-step physics + sub-tick
 *       interpolation.  This file deliberately uses VARIABLE dt
 *       because the simulation is non-stiff (no springs, no fast
 *       oscillators) and a single dt per frame is unconditionally
 *       stable — Fiedler's DT_CAP rule (100 ms hard cap) is the
 *       only piece we adopt.
 *
 *   ── Implementation-oriented reference ──────────────────────────
 *   [7] Shiffman, D., "The Nature of Code", Ch. 0 (Random walks +
 *       Perlin noise), Ch. 4 (Particle Systems) — reading-level
 *       walkthrough of the per-particle state vector + reroll
 *       patterns this file uses.  Pair with [3] for the math.
 *
 *   ── ncurses rendering substrate ────────────────────────────────
 *   [8] Raymond, E. S., "NCURSES Programming HOWTO" — particularly
 *       §6 (colour pairs) for the theme-cycling design, and §11
 *       (output options) for the wnoutrefresh + doupdate diff-only
 *       write pattern that keeps the field flicker-free at 60 fps.
 *
 *   ── Companion files in this project ────────────────────────────
 *   See also:
 *     matrix_rain/fireworks_rain.c — same shimmer-cache pattern
 *       applied to ARC trajectories instead of vertical lanes.
 *       Read this file first if the per-particle glyph cache trick
 *       is unfamiliar.
 *     matrix_rain/pulsar_rain.c    — radial pulses with the same
 *       rerolling-glyph effect; another variant of the shimmer
 *       cache idea.
 *     matrix_rain/sun_rain.c       — sun-shaped variant.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * One stream per terminal column. The stream's head is a single
 * floating-point row number that increases over time (downward in
 * screen coordinates). Behind the head sits a fixed-length trail of
 * cached glyphs. Each render frame, walk the trail from the head
 * backward and paint dist rows above the head — that's the visible
 * snake. Each tick, reroll most of the cache so the same falling
 * trajectory looks like continuous shimmer instead of a static
 * curve. When the tail goes off-screen, the column dies and may be
 * respawned at the top.
 *
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT
 *       For each terminal column x: if x % density == 0, spawn a
 *       fresh stream there. Spawn:
 *         head_y   = uniform(-rows/2, 0)        (above the screen)
 *         length   = uniform(TRAIL_MIN, TRAIL_MAX)
 *         speed    = uniform(SPEED_MIN, SPEED_MAX)   (rows / second)
 *         glyphs[] = TRAIL_MAX random ASCII chars
 *         active   = true
 *
 *  2. PER FRAME (variable dt):
 *       a. dt seconds since last frame (capped at 100 ms).
 *       b. For each column:
 *            if active:
 *              head_y   += speed · dt · speed_scale    (advance)
 *              [optional, every glyph-reroll period]:
 *              for k in 0..length-1: glyphs[k] = rand_glyph()
 *              if (head_y - length) > rows: deactivate
 *            else:
 *              with chance RESPAWN_RATE_PER_SEC · dt / density:
 *                spawn a fresh stream at this column
 *
 *  3. RENDER
 *       erase()
 *       for each active column:
 *         for dist = 0..length-1:
 *           row = floor(head_y - dist + 0.5)
 *           if row in [0, rows): paint glyphs[dist] in band(dist)
 *       paint HUD (status row 0, hint strip on bottom row)
 *       wnoutrefresh + doupdate
 *
 * KEY FORMULAS
 * ────────────
 *   head_y(t+dt)  = head_y(t) + speed · dt · speed_scale
 *   row           = floor(head_y - dist + 0.5)         round half UP
 *   band(dist) → HEAD/HOT/BRIGHT/MID/DARK/FADE         see col_band
 *   respawn_p     = RESPAWN_RATE_PER_SEC · dt / density (per frame)
 *
 *
 * Background you need
 * ───────────────────
 *   - ncurses cell-space rendering (mvaddch + colour pairs +
 *     attributes A_BOLD / A_DIM).
 *   - Variable-timestep main loop (Glenn Fiedler "Fix Your
 *     Timestep!"). We use ONE dt per frame, not a fixed-step
 *     accumulator.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Particle systems with N_max particles. Here EACH COLUMN owns
 *     ONE stream — no general particle pool.
 *   - Texture atlases / sprite sheets. Glyphs are random ASCII;
 *     no images.
 *   - GPU shaders. The "shimmer" is a software trick of rerolling
 *     a tiny char cache.
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

/* ── §1.1 frame rate + speed control ──────────────────────────────── */
enum {
    TARGET_FPS       = 60,            /* render cap (frames per second)  */

    /* Per-second glyph reroll rate. Lower → chunkier shimmer.           */
    SHIMMER_HZ       = 20,
};

/*
 * Speed scale — global multiplier on stream speeds, toggled by [ and ].
 * 1.0 = nominal speed. Values outside [MIN, MAX] are rejected by input.
 */
#define SPEED_SCALE_DEFAULT  1.0f
#define SPEED_SCALE_MIN      0.25f
#define SPEED_SCALE_MAX      4.0f
#define SPEED_SCALE_STEP     1.25f      /* multiplicative — feels uniform  */

/* ── §1.2 stream geometry ─────────────────────────────────────────── */
enum {
    TRAIL_MIN        =  6,            /* min stream length (rows)        */
    TRAIL_MAX        = 24,            /* max + array bound on glyphs[]   */
};

/* Stream speeds in rows/second. Varied per column so streams visibly
 * fall at different rates. Total uniform: [SPEED_MIN, SPEED_MAX]. */
#define SPEED_MIN_RPS   8.0f
#define SPEED_MAX_RPS  24.0f

/* ── §1.3 density + respawn ───────────────────────────────────────── */
enum {
    DENSITY_MIN      =  1,    /* every column is lit                    */
    DENSITY_DEFAULT  =  2,    /* every other column                     */
    DENSITY_MAX      =  6,    /* roughly 1 in 6 columns                 */
};

/* RESPAWN_RATE_PER_SEC — base probability per second that a dead
 * column wakes up. Divided by density at runtime so sparser settings
 * also wait longer between respawns. 0.6 → ~1.7 s expected wait at
 * density 1; ~10 s at density 6. */
#define RESPAWN_RATE_PER_SEC  0.6f

/* On spawn, head_y is placed uniform in [-rows · OFFSCREEN_FRAC, 0)
 * so streams enter at staggered offsets and don't all start in lock-
 * step at the top edge. */
#define SPAWN_OFFSCREEN_FRAC  0.5f

/* ── §1.4 dt cap (spiral-of-death guard) ──────────────────────────── */
#define DT_CAP_SEC  0.10f

/* ── §1.5 ncurses pair IDs ────────────────────────────────────────── */
enum {
    /* 1..5 — trail bands (theme-controlled) */
    SHADE_FADE      = 1,
    SHADE_DARK,
    SHADE_MID,
    SHADE_BRIGHT,
    SHADE_HOT,

    /* 6 — head, always white (theme-independent) */
    SHADE_HEAD,

    /* 7..8 — HUD spec (theme-independent) */
    PAIR_HUD,                         /* bright yellow on default bg     */
    PAIR_HINT,                        /* bright cyan on default bg       */
};

/* ── §1.6 timing primitives ───────────────────────────────────────── */
#define NS_PER_SEC    1000000000LL
#define NS_PER_MS     1000000LL

/* ── §1.7 HUD layout ──────────────────────────────────────────────── */
#define HUD_BUF_LEN  64

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
 * Theme — one named 5-band brightness palette for the trail tail.
 *
 * Intent
 *   Each falling stream draws its trail in FIVE brightness tiers, from
 *   FADE (dimmest, oldest cells) to HOT (brightest, near the head),
 *   plus a sixth "HEAD" cell that is always white regardless of theme.
 *   Mapping (distance-from-head → tier) happens in §4 col_band; this
 *   struct holds the per-theme fg colour for each tier.
 *
 *   The `t` key cycles through THEME_COUNT presets — theme_apply()
 *   re-registers all the colour pairs in one O(PAIR_COUNT) call.
 *   Each column's PAIR ID stays the same across theme switches; only
 *   the pair's fg colour changes, so previously-spawned streams
 *   instantly adopt the new look without re-spawn.
 *
 * Why two parallel palettes (fg[] vs fg_8[])
 *   256-colour terminals can use a fine-grained brightness ramp
 *   (5 distinct cube indices per theme); 8-colour terminals can't —
 *   fg_8[] picks the closest ANSI primary, accepting that the ramp
 *   collapses into 1–2 equivalence classes.
 *
 * Why HEAD is theme-independent
 *   The bright white "leading character" of a falling stream is the
 *   signature visual of Matrix rain — making it theme-tinted would
 *   wash out the contrast against the trail tail.  Hardcoded white
 *   keeps the head readable across every theme.
 *
 * Themes (preset table)
 *   green  — classic Matrix (the canonical look)
 *   amber  — orange / sodium-vapour
 *   blue   — cool, cyber-noir
 *   white  — black & white film grain
 *
 * Members
 *   name    HUD label shown in the status row ("green", "amber", …).
 *   fg[]    5 xterm-256 fg indices ordered SHADE_FADE..SHADE_HOT
 *           (dimmest → brightest).
 *   fg_8[]  5 ANSI-8 fallback indices, same ordering.
 *
 * Invariants
 *   Every fg[i] ∈ [24, 255] per CLAUDE.md "Theme Palette Brightness"
 *   (cube 16–23 and gray 232–239 render as black on default-black
 *   terminals, so they'd make trails invisible).
 *   fg[i] perceptually ≤ fg[i+1] in brightness (monotone ramp).
 *   name != NULL.
 *
 * References
 *   CLAUDE.md §"Theme Palette Brightness" for the cube-floor rule.
 *   The Wachowski film [9] (see References block) for the original
 *   "bright head + fading tail" visual that this palette structure
 *   reproduces.
 */
typedef struct {
    const char *name;
    int         fg[5];                /* SHADE_FADE..SHADE_HOT (256-colour) */
    int         fg_8[5];              /* 8-colour fallback                  */
} Theme;

static const Theme k_themes[] = {
    { "green",
      {  28,  34,  40,  46,  82 },
      { COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN  } },
    { "amber",
      {  94, 130, 172, 214, 220 },
      { COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW } },
    { "blue",
      {  24,  33,  39,  45,  51 },
      { COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN   } },
    { "white",
      { 240, 244, 248, 252, 255 },
      { COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE  } },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static bool g_has_256 = false;

/*
 * theme_apply — bind pairs SHADE_FADE..SHADE_HEAD for the chosen theme.
 * Pairs PAIR_HUD and PAIR_HINT are NEVER touched here — they carry
 * semantic meaning that must not change with theme.
 */
static void theme_apply(int theme_idx)
{
    const int *fg = g_has_256
                    ? k_themes[theme_idx].fg
                    : k_themes[theme_idx].fg_8;

    init_pair(SHADE_FADE,   fg[0],       COLOR_BLACK);
    init_pair(SHADE_DARK,   fg[1],       COLOR_BLACK);
    init_pair(SHADE_MID,    fg[2],       COLOR_BLACK);
    init_pair(SHADE_BRIGHT, fg[3],       COLOR_BLACK);
    init_pair(SHADE_HOT,    fg[4],       COLOR_BLACK);
    init_pair(SHADE_HEAD,   COLOR_WHITE, COLOR_BLACK);
}

/*
 * hud_pairs_init — bind PAIR_HUD and PAIR_HINT once at startup. Both
 * use the default terminal background (-1) so the HUD row sits on
 * whatever background the user actually has rather than a forced
 * black box. Theme cycling never touches these pairs.
 */
static void hud_pairs_init(void)
{
    init_pair(PAIR_HUD,  g_has_256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, g_has_256 ?  51 : COLOR_CYAN,   -1);
}

/* ===================================================================== */
/* §4  column                                                             */
/* ===================================================================== */

/* ── §4.1 ASCII glyph pool + tiny utility ─────────────────────────── */

static const char k_glyphs[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "!@#$%^&*()-_=+[]{}|;:,.<>?/~`";

#define GLYPHS_LEN (int)(sizeof k_glyphs - 1)

static char  rand_glyph(void)  { return k_glyphs[rand() % GLYPHS_LEN]; }
static float urand01(void)     { return (float)rand() / (float)RAND_MAX; }

/* ── §4.2 Column type ─────────────────────────────────────────────── */

/*
 * Column — one falling stream of characters in a single terminal lane.
 *
 * Intent
 *   The atomic unit of the rain effect.  Each Column owns ONE vertical
 *   lane (`col`) and represents the trail of glyphs cascading down it.
 *   The visible stream is HEAD + TRAIL where head_y is the bright
 *   leading row (drawn white) and the trail extends UPWARD by
 *   `trail_len` rows.  Older trail cells fade through the 5 theme
 *   bands (FADE..HOT).
 *
 *   Each tick:
 *     1. ADVANCE  — head_y += speed · dt  (no acceleration; rain is
 *                   constant-velocity)
 *     2. SHIMMER  — at SHIMMER_HZ pulses, glyphs[] gets re-rerolled
 *                   so the cached letters jump between visible frames
 *     3. CULL     — once the HEAD has fallen far enough that the
 *                   entire trail is off-screen, mark active=false;
 *                   the slot becomes available for respawn
 *
 * Why per-column glyph cache (not random-on-draw)
 *   Without the cache, each draw would generate a fresh random glyph
 *   at each cell — readers would see a wall of fully-shuffled letters
 *   every frame, far too much motion to track.  Caching means each
 *   glyph PERSISTS for ~1/SHIMMER_HZ seconds; the shimmer reads as
 *   TEXTURE rather than noise.  This is the canonical Matrix-rain
 *   trick (Wachowski [9]) and the same pattern fireworks_rain.c
 *   uses for its spark trails.
 *
 * Why fixed-size glyphs[TRAIL_MAX] (not VLA)
 *   trail_len varies per stream (each gets a random length in
 *   [TRAIL_MIN, TRAIL_MAX]) but the buffer is sized for the worst
 *   case.  Inline storage means the entire ColumnPool sits in one
 *   contiguous heap block with zero per-stream malloc.
 *
 * Why HEAD ON FIRST INDEX (glyphs[0]), not last
 *   col_draw iterates dist = 0 → trail_len - 1; dist=0 is the head
 *   row drawn brightest, increasing dist climbs UP the trail to dimmer
 *   cells.  Indexing the head at [0] makes this loop read naturally.
 *
 * Members
 *   col          Terminal column index (x).  Set ONCE at col_spawn
 *                and never changes — col_spawn(c, x, …) makes c->col
 *                equal to its array index in ColumnPool.
 *   head_y       Current head ROW position (float).  Sub-row precision
 *                so col_advance can use fractional `speed·dt` without
 *                stuttering.  Integer-rounded only at draw time.
 *   speed        Vertical velocity in CELLS / SECOND.  Constant for
 *                this stream's lifetime; randomised across streams at
 *                spawn (different lanes fall at different rates →
 *                organic look, not synchronised waterfall).
 *   trail_len    Total length of the visible snake (head + tail).
 *                Randomised at spawn from [TRAIL_MIN, TRAIL_MAX].
 *   glyphs[]     Cached random ASCII characters, one per trail slot.
 *                glyphs[0] = head (always re-rolled even when shimmer
 *                isn't firing — keeps the leading character fresh);
 *                glyphs[1..trail_len-1] = tail.  Rerolled wholesale at
 *                SHIMMER_HZ to produce the shimmer effect.
 *   active       true while at least part of the trail is on screen;
 *                false once head_y has fallen far enough that the
 *                entire trail is below the bottom row.  scene_tick
 *                (via tick_one_column) rolls dice on inactive slots
 *                for respawn.
 *
 * Invariants
 *   0 ≤ col < ColumnPool.ncols.
 *   0 ≤ trail_len ≤ TRAIL_MAX.
 *   active == false  ⇒  this slot is a candidate for respawn.
 *
 * References
 *   [1] Wachowski film — the visual heritage.
 *   See also Spark in matrix_rain/fireworks_rain.c — the same
 *   per-particle glyph-cache + shimmer pattern applied to arc
 *   trajectories instead of vertical lanes.
 */
typedef struct {
    int    col;
    float  head_y;
    float  speed;
    int    trail_len;
    char   glyphs[TRAIL_MAX];
    bool   active;
} Column;

/* ── §4.3 col_spawn — fresh stream above the screen ───────────────── */

/*
 * Spawn a fresh stream at column x. head_y starts in the negative
 * "above-screen" zone so streams enter staggered, not in lock-step.
 * length, speed, and glyphs are independently randomised per spawn.
 */
static void col_spawn(Column *c, int x, int rows)
{
    c->col       = x;
    c->head_y    = -urand01() * SPAWN_OFFSCREEN_FRAC * (float)rows;
    c->trail_len = TRAIL_MIN + rand() % (TRAIL_MAX - TRAIL_MIN + 1);
    c->speed     = SPEED_MIN_RPS + urand01() * (SPEED_MAX_RPS - SPEED_MIN_RPS);
    c->active    = true;

    for (int i = 0; i < c->trail_len; i++)
        c->glyphs[i] = rand_glyph();
}

/* ── §4.4 col_advance — move the head down and check for end-of-life ─ */

/*
 * One per-frame physics step. Returns true while the stream is still
 * partially on screen, false once even the tail tip has cleared the
 * bottom edge (caller deactivates).
 */
static bool col_advance(Column *c, float dt, int rows)
{
    c->head_y += c->speed * dt;
    return (c->head_y - (float)c->trail_len) < (float)rows;
}

/* ── §4.5 col_shimmer — reroll all cached glyphs ──────────────────── */

/*
 * Reroll every glyph in the cache. Called at SHIMMER_HZ rate (not
 * every frame) so the shimmer reads as a discrete blink instead of
 * a 60 Hz blur.
 */
static void col_shimmer(Column *c)
{
    for (int i = 0; i < c->trail_len; i++)
        c->glyphs[i] = rand_glyph();
}

/* ── §4.6 col_band — map distance from head → brightness band ─────── */

/*
 * The mapping is the canonical Matrix-rain gradient:
 *
 *     dist 0          → SHADE_HEAD    (white,    BOLD)
 *     dist 1          → SHADE_HOT     (theme[4], BOLD)
 *     dist 2          → SHADE_BRIGHT  (theme[3], BOLD)
 *     dist 3..len/2   → SHADE_MID     (theme[2], NORMAL)
 *     dist ..len-2    → SHADE_DARK    (theme[1], NORMAL)
 *     dist len-1      → SHADE_FADE    (theme[0], DIM)
 *
 * Returns the ncurses pair-and-attr combination ready for attron().
 */
static attr_t col_band(int dist, int trail_len)
{
    if (dist == 0)               return COLOR_PAIR(SHADE_HEAD)   | A_BOLD;
    if (dist == 1)               return COLOR_PAIR(SHADE_HOT)    | A_BOLD;
    if (dist == 2)               return COLOR_PAIR(SHADE_BRIGHT) | A_BOLD;
    if (dist <= trail_len / 2)   return COLOR_PAIR(SHADE_MID);
    if (dist <= trail_len - 2)   return COLOR_PAIR(SHADE_DARK);
    return COLOR_PAIR(SHADE_FADE) | A_DIM;
}

/* ── §4.7 col_draw — paint the trail head-first ───────────────────── */

/*
 * Walk dist = 0..trail_len-1 from the head backward, mapping each
 * distance to a brightness band and painting the cached glyph at
 * the appropriate row.
 *
 * Row mapping uses floor(head_y - dist + 0.5) — "round half up" —
 * for deterministic rounding at .5 boundaries. roundf(x) would use
 * banker's rounding which causes a row to flicker between two
 * neighbouring rows when the fractional part lands exactly at .5.
 */
static void col_draw(const Column *c, int rows)
{
    for (int dist = 0; dist < c->trail_len; dist++) {
        int row = (int)floorf(c->head_y - (float)dist + 0.5f);
        if (row < 0 || row >= rows) continue;

        attr_t attr = col_band(dist, c->trail_len);
        attron(attr);
        mvaddch(row, c->col, (chtype)(unsigned char)c->glyphs[dist]);
        attroff(attr);
    }
}

/* ===================================================================== */
/* §5  scene — Scene root + ColumnPool + SimControls sub-structs          */
/* ===================================================================== */

/*
 * ColumnPool — one Column slot per terminal column, plus the active
 * grid extent.
 *
 * Intent
 *   The pool is sized ONE-TO-ONE with the terminal: every visible
 *   terminal column gets one Column slot.  Slots are zero-initialised
 *   to `active = false` by calloc(); col_spawn populates them lazily
 *   (a density-spaced subset at startup, the rest wake up over time
 *   via the per-frame respawn roll).
 *
 *   The pool is the ONLY heap allocation in this demo — sized at
 *   init from `ncols`, freed by scene_free, re-allocated on every
 *   SIGWINCH via app_do_resize.  Everything else lives in BSS.
 *
 * Why "pool sized = terminal cols" (not a free-list of streams)
 *   Each terminal column has at most ONE active stream at any time.
 *   That's the visual signature of Matrix rain: vertical streams in
 *   their own lanes.  Sizing the pool to terminal cols makes the
 *   array index BE the column number, eliminating any lookup.
 *
 * Members
 *   columns   Heap array of `ncols` Column slots.  Index = terminal
 *             column number; slot.col == its array index.
 *   ncols     Terminal width (number of vertical lanes).
 *   nrows     Terminal height (used by col_advance to deactivate
 *             streams that fall off the bottom edge).
 *
 * Invariants
 *   ncols > 0, nrows > 0 after scene_init.
 *   columns != NULL after scene_init, NULL after scene_free.
 *   For every i: columns[i].col == i (col_spawn sets this; never
 *   re-assigned).
 */
typedef struct {
    Column *columns;
    int     ncols;
    int     nrows;
} ColumnPool;

/*
 * SimControls — user-facing playback knobs + the shimmer-pulse timer.
 *
 * Intent
 *   Bundles every value the input handler writes (paused, speed_scale,
 *   density) AND the small piece of internal SIM TIMING (shimmer_accum)
 *   that gates the SHIMMER_HZ pulse.  The timer lives here because it's
 *   conceptually a SimControls-rate accumulator (advanced once per
 *   scene_tick from the same dt that the physics consumes).
 *
 * Members
 *   paused          true → scene_tick early-returns; HUD shows "PAUSED".
 *                   Toggled by SPACE.
 *   speed_scale     Multiplier on per-stream speeds (typically 0.5 – 2.0).
 *                   ] makes streams faster, [ slower.  Clamped to
 *                   [SPEED_SCALE_MIN, SPEED_SCALE_MAX].
 *   density         Stream spacing — only every density-th column
 *                   starts active at init; respawns happen anywhere
 *                   but at a rate divided by density (sparser settings
 *                   stay sparser).  + / − keys adjust; clamped to
 *                   [DENSITY_MIN, DENSITY_MAX].
 *   shimmer_accum   Seconds since last cache reroll.  When ≥
 *                   1 / SHIMMER_HZ, EVERY active column rerolls its
 *                   glyph cache and the accumulator resets.  Decoupled
 *                   from the physics dt so the visual shimmer rate is
 *                   constant regardless of frame rate.
 *
 * Invariants
 *   SPEED_SCALE_MIN ≤ speed_scale ≤ SPEED_SCALE_MAX.
 *   DENSITY_MIN     ≤ density     ≤ DENSITY_MAX.
 *   shimmer_accum ∈ [0, 1/SHIMMER_HZ).
 */
typedef struct {
    bool  paused;
    float speed_scale;
    int   density;
    float shimmer_accum;
} SimControls;

/*
 * Scene — owns ALL simulation state for one run.
 *
 * Layered ownership
 *
 *     Scene
 *       ├── pool       : ColumnPool   ← Column[ncols] + dims
 *       ├── sim        : SimControls  ← paused + speed + density + shimmer
 *       └── theme_idx  : int          ← active colour palette (t/T cycles)
 *
 *   Every persistent simulation value is reachable from one `Scene *s`.
 *   theme_idx lives on Scene (not App) because it's a SCENE-level
 *   rendering concern.
 */
typedef struct {
    ColumnPool  pool;
    SimControls sim;
    int         theme_idx;
} Scene;

/*
 * seed_initial_streams — wake every density-th column at startup so the
 * field isn't empty for the first few seconds while the respawn-roll
 * gradually populates it.  The rest of the columns start dead and
 * wake up over time.
 */
static void seed_initial_streams(Scene *s) {
    for (int x = 0; x < s->pool.ncols; x++) {
        if (x % s->sim.density == 0)
            col_spawn(&s->pool.columns[x], x, s->pool.nrows);
    }
}

static void scene_init(Scene *s, int cols, int rows, int density)
{
    /* (1) ColumnPool — heap array sized to terminal width */
    s->pool.columns = calloc((size_t)cols, sizeof(Column));
    s->pool.ncols   = cols;
    s->pool.nrows   = rows;

    /* (2) SimControls — defaults; user keys mutate later */
    s->sim.paused        = false;
    s->sim.speed_scale   = SPEED_SCALE_DEFAULT;
    s->sim.density       = density;
    s->sim.shimmer_accum = 0.0f;

    /* (3) seed the field so it's visible immediately */
    seed_initial_streams(s);
}

static void scene_free(Scene *s)
{
    free(s->pool.columns);
    s->pool.columns = NULL;
    s->pool.ncols   = 0;
    s->pool.nrows   = 0;
}

/*
 * tick_shimmer_pulse — accumulate dt; once we cross the per-tick
 * interval (1 / SHIMMER_HZ seconds), return TRUE and reset the
 * accumulator.  The caller uses the return as a "reroll glyphs now"
 * trigger for every active column.
 */
static bool tick_shimmer_pulse(SimControls *sim, float dt) {
    sim->shimmer_accum += dt;
    bool fired = (sim->shimmer_accum >= 1.0f / (float)SHIMMER_HZ);
    if (fired) sim->shimmer_accum = 0.0f;
    return fired;
}

/*
 * respawn_probability_per_frame — per-frame probability that ONE dead
 * column wakes up.  Derivation:
 *
 *   RESPAWN_RATE_PER_SEC is the target wakeups-per-second across the
 *   WHOLE FIELD.  Sparser densities should have proportionally fewer
 *   wakeups (so density = 8 has ⅛ the activity of density = 1).
 *   Dividing by density gives the right rate.  Multiplying by dt
 *   converts rate (Hz) into a per-frame probability.
 */
static inline float respawn_probability_per_frame(int density, float dt) {
    return (RESPAWN_RATE_PER_SEC * dt) / (float)density;
}

/*
 * tick_one_column — per-column branch of scene_tick:
 *   active   → advance by scaled_dt; deactivate if it fell off the
 *              bottom; otherwise shimmer-reroll if the pulse fired.
 *   inactive → roll dice; on success, spawn a fresh stream.
 */
static inline void tick_one_column(Column *c, int col_x, int nrows,
                                    float scaled_dt, float respawn_p,
                                    bool shimmer_now) {
    if (c->active) {
        if (!col_advance(c, scaled_dt, nrows))
            c->active = false;
        else if (shimmer_now)
            col_shimmer(c);
    } else {
        if (urand01() < respawn_p)
            col_spawn(c, col_x, nrows);
    }
}

/*
 * scene_tick — one frame of the matrix-rain field.
 *
 *   (1) bail if paused (HUD still re-renders)
 *   (2) advance the shimmer-pulse timer; was a pulse this frame?
 *   (3) precompute scaled_dt + respawn_p (once, not per column)
 *   (4) for each column: tick_one_column (active → advance; dead → roll)
 *
 * Glyph cache rerolls at SHIMMER_HZ — independent of the per-frame
 * physics rate; the visual shimmer stays constant whether the game
 * loop runs at 30 fps or 120 fps.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->sim.paused) return;

    bool  shimmer_now = tick_shimmer_pulse(&s->sim, dt);
    float scaled_dt   = dt * s->sim.speed_scale;
    float respawn_p   = respawn_probability_per_frame(s->sim.density, dt);

    for (int x = 0; x < s->pool.ncols; x++) {
        tick_one_column(&s->pool.columns[x], x, s->pool.nrows,
                        scaled_dt, respawn_p, shimmer_now);
    }
}

static void scene_draw(const Scene *s)
{
    for (int x = 0; x < s->pool.ncols; x++) {
        const Column *c = &s->pool.columns[x];
        if (c->active) col_draw(c, s->pool.nrows);
    }
}

/* Scene input helpers — used by app_handle_key. */

/* scene_scale_speed — ] / [ keys; multiplicative clamp. */
static void scene_scale_speed(Scene *s, float factor) {
    s->sim.speed_scale *= factor;
    if (s->sim.speed_scale < SPEED_SCALE_MIN) s->sim.speed_scale = SPEED_SCALE_MIN;
    if (s->sim.speed_scale > SPEED_SCALE_MAX) s->sim.speed_scale = SPEED_SCALE_MAX;
}

/* scene_change_density — + key densifies (LOWER density value =
 * MORE streams), − key sparsens.  Clamped to [DENSITY_MIN, MAX]. */
static void scene_change_density(Scene *s, int delta) {
    int next = s->sim.density + delta;
    if (next < DENSITY_MIN) next = DENSITY_MIN;
    if (next > DENSITY_MAX) next = DENSITY_MAX;
    s->sim.density = next;
}

/* scene_cycle_theme — t/T → next palette index, re-register pairs. */
static void scene_cycle_theme(Scene *s) {
    s->theme_idx = (s->theme_idx + 1) % THEME_COUNT;
    theme_apply(s->theme_idx);
}

/*
 * scene_reset_preserving_knobs — wipe and re-init the scene at
 * (new_cols, new_rows), preserving the user-tuned settings that
 * SHOULD survive a reset (speed_scale, density, theme_idx).
 *
 * Used by:
 *   - app_handle_key('r')  — explicit user reset: same size, new field
 *   - app_do_resize        — SIGWINCH: new size, same knobs
 *
 * Both call sites want the same "save knobs → free → init → restore
 * knobs" dance; centralising it ensures the saved set stays in lockstep
 * across both code paths (any future knob added here is automatically
 * preserved by both).
 *
 * Note: shimmer_accum is INTENTIONALLY reset to 0 — it's an internal
 * sim-timing accumulator, not a user knob, so a reset should restart
 * the pulse cleanly.
 */
static void scene_reset_preserving_knobs(Scene *s, int new_cols, int new_rows) {
    /* (1) snapshot user-tuned settings */
    float saved_speed = s->sim.speed_scale;
    int   saved_dens  = s->sim.density;
    int   saved_theme = s->theme_idx;

    /* (2) wipe + re-init at the (possibly new) extent */
    scene_free(s);
    scene_init(s, new_cols, new_rows, saved_dens);

    /* (3) restore the preserved knobs (density already restored via
     *     scene_init's third arg; speed + theme need an explicit copy) */
    s->sim.speed_scale = saved_speed;
    s->theme_idx       = saved_theme;
}

/* ===================================================================== */
/* §6  screen                                                             */
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
 *   Scene.pool.ncols and Scene.pool.nrows TRACK this struct (they're
 *   the simulation's view of the same dimensions, copied in
 *   scene_init / app_do_resize).  Keeping them in TWO places lets
 *   the sim helpers read pool.nrows for cull tests without reaching
 *   back to App.screen — the simulation stays decoupled from the
 *   render layer.
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
 *   cols   Terminal width in CELLS (getmaxyx).  One Column slot
 *          per cell in the active rain.
 *   rows   Terminal height in CELLS.  Bottom row index is rows − 1
 *          (where the key hint paints).  Row 0 is the status row.
 *
 * Invariants
 *   cols > 0, rows > 0 after screen_init.
 *   Both refreshed on every SIGWINCH via screen_resize +
 *   app_do_resize (which also re-inits the Scene at the new size,
 *   reallocating ColumnPool.columns).
 */
typedef struct {
    int cols;
    int rows;
} Screen;

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
 *   Row 0       PAIR_HUD  + A_BOLD  (bright yellow)  — fps + state
 *   Bottom row  PAIR_HINT + A_BOLD  (bright cyan)    — full key list
 *
 * Both pairs sit on default background (-1) so they stay legible
 * regardless of theme. theme_apply() never touches them.
 */
/* hud_paint_text — attron/mvprintw/attroff sandwich shared by both
 * HUD rows.  Centralises the colour-pair setup. */
static inline void hud_paint_text(int row, int col, int pair, const char *text) {
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* format_hud_status — write the top-row status text into `buf`.
 * Reports fps, speed scale, density, active theme, paused state. */
static void format_hud_status(const Scene *scene, double fps,
                              char *buf, size_t buflen) {
    snprintf(buf, buflen,
             " %5.1f fps  spd:%.2fx  den:%d  [%s] %s ",
             fps, scene->sim.speed_scale, scene->sim.density,
             k_themes[scene->theme_idx].name,
             scene->sim.paused ? "PAUSED " : "running");
}

/* draw_hud_status — right-aligned status row 0 (PAIR_HUD yellow) */
static void draw_hud_status(const Screen *s, const Scene *scene, double fps) {
    enum { HUD_TOP_ROW = 0 };
    char buf[HUD_BUF_LEN];
    format_hud_status(scene, fps, buf, sizeof buf);
    int right_col = s->cols - (int)strlen(buf);
    if (right_col < 0) right_col = 0;
    hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, buf);
}

/* draw_hud_hint — bottom-row key bindings strip (PAIR_HINT cyan) */
static void draw_hud_hint(const Screen *s) {
    static const char *KEY_HINT =
        " q:quit  spc:pause  r:reset  []:speed  +/-:density  t:theme ";
    hud_paint_text(s->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

static void screen_draw_hud(const Screen *s, double fps, const Scene *scene)
{
    draw_hud_status(s, scene, fps);
    draw_hud_hint  (s);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §7  app — signals, resize, variable-dt main loop                       */
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
 *   scene         simulation state (column pool + sim + theme)
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

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/*
 * app_do_resize — full re-init of the scene on the new screen size,
 * preserving the user-tuned speed, density, and theme.  In-flight
 * streams are lost (cosmetic flicker for one frame).
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);   /* (1) read new terminal extent */
    scene_reset_preserving_knobs(  /* (2) re-init Scene at new size */
        &app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;          /* (3) clear the SIGWINCH flag   */
}

/*
 * app_handle_key — process one keystroke.  Returns false on quit.
 *
 *   q / Q / ESC      quit
 *   SPACE            toggle paused
 *   r / R            reset (re-init scene at current density)
 *   ] / [            speed up / slow down (multiplicative)
 *   + / =            densify (smaller density → more streams)
 *   -                sparsen
 *   t / T            cycle theme
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ':           s->sim.paused = !s->sim.paused;                 break;

    case 'r': case 'R':
        scene_reset_preserving_knobs(s, app->screen.cols, app->screen.rows);
        break;

    case ']':           scene_scale_speed(s, SPEED_SCALE_STEP);          break;
    case '[':           scene_scale_speed(s, 1.0f / SPEED_SCALE_STEP);   break;

    case '=': case '+': scene_change_density(s, -1); break;   /* − value = MORE streams */
    case '-':           scene_change_density(s, +1); break;   /* + value = fewer streams */

    case 't': case 'T': scene_cycle_theme(s);        break;

    default: break;
    }
    return true;
}

/*
 * main — variable-dt render loop (no fixed-step accumulator; the field
 * is visual, not physics-critical).
 *
 *   (1) RNG + atexit + signal handlers
 *   (2) bring up Screen + App; init Scene at current terminal size
 *   (3) loop:
 *       (a) handle pending SIGWINCH
 *       (b) measure dt (capped to prevent spiral-of-death)
 *       (c) drain non-blocking input
 *       (d) advance the field (scene_tick — ONE call, no accumulator)
 *       (e) rolling-window fps counter
 *       (f) draw + present
 *       (g) frame cap: sleep so we don't burn the next slot's budget
 */
int main(void)
{
    /* (1) RNG + atexit + signal handlers */
    srand((unsigned int)clock_ns());
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    /* (2) Bring up Screen + App; init Scene */
    App   *app   = &g_app;
    Scene *scene = &app->scene;
    app->running = 1;
    fps_counter_init(&app->fps);

    screen_init(&app->screen);
    g_has_256 = (COLORS >= 256);
    scene->theme_idx = 0;
    theme_apply(scene->theme_idx);
    hud_pairs_init();
    scene_init(scene, app->screen.cols, app->screen.rows, DENSITY_DEFAULT);

    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;
    int64_t last_ns = clock_ns();

    while (app->running) {

        /* (3a) handle resize first so subsequent steps see the new size */
        if (app->need_resize) {
            app_do_resize(app);
            last_ns = clock_ns();
        }

        /* (3b) measure dt, capped to prevent spiral-of-death */
        int64_t now_ns  = clock_ns();
        int64_t dt_ns   = now_ns - last_ns;
        last_ns         = now_ns;
        float   dt      = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        /* (3c) drain input */
        for (int ch; (ch = getch()) != ERR; ) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }

        /* (3d) advance the field — ONE call per frame, no accumulator */
        scene_tick(scene, dt);

        /* (3e) rolling-window fps counter */
        fps_counter_tick(&app->fps, dt_ns);

        /* (3f) draw + present */
        erase();
        scene_draw(scene);
        screen_draw_hud(&app->screen, app->fps.display, scene);
        screen_present();

        /* (3g) frame cap — sleep before the NEXT frame's I/O */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
    }

    scene_free(scene);
    screen_free(&app->screen);
    return 0;
}
