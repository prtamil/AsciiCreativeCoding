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
 *   §5  rain         — Rain: array of Column + tick + draw + respawn
 *   §6  screen       — ncurses init / present / HUD
 *   §7  app          — signals, resize, variable-dt main loop
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
 * Data-structure: One Rain owns a Column[ncols] flat array indexed
 *                 by terminal column. Each Column carries the head
 *                 position, length, speed, glyph cache, and active
 *                 flag. No heap allocation post-init; no Grid; no
 *                 second framebuffer. Rendering iterates the
 *                 Column array directly.
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
 * References    :
 *   "The Matrix" (1999, Wachowski) — the original vertical-stream
 *     green code visual. The "code" originally used mirror-image
 *     katakana plus digits; this file uses ASCII for portability.
 *   Wikipedia, "Matrix digital rain" — history of the effect, font
 *     details, prior demoscene precedents.
 *     https://en.wikipedia.org/wiki/Matrix_digital_rain
 *   Glenn Fiedler, "Fix Your Timestep!" (gafferongames.com) — the
 *     case for fixed-step physics with sub-tick interpolation. We
 *     deliberately use VARIABLE timestep here because the simulation
 *     is non-stiff (no springs, no fast oscillators) and a single
 *     dt per frame is unconditionally stable.
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
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture the screen as a vertical comb. Each tooth of the comb is
 * a terminal column. Each column has at most ONE bright stream
 * sliding down it like a Roman candle pointing earthward. The
 * bright tip is the HEAD; behind it, a tail that fades from "hot
 * white" through theme colour all the way down to a barely-visible
 * tinted glyph. The streams slide independently — different lengths,
 * different speeds, different start times — so the screen looks
 * like dense rainfall rather than a synchronized fall.
 *
 * COLUMN BAND STRUCTURE
 * ─────────────────────
 *
 *   dist=0  HEAD     ← white BOLD                 ┐
 *   dist=1  HOT      ← theme[4] BOLD              │ HOT/BRIGHT zone
 *   dist=2  BRIGHT   ← theme[3] BOLD              ┘ (still vivid)
 *   dist=3  MID      ← theme[2]                   ┐
 *   dist=4  MID                                   │ middle fade
 *   ...                                           │ (length/2 boundary)
 *   dist=k  MID                                   ┘
 *   dist=k+1 DARK    ← theme[1]                   ┐ tail
 *   ...                                           │ (length-2)
 *   dist=N-2 DARK                                 ┘
 *   dist=N-1 FADE    ← theme[0] DIM               ← tail tip
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
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Round-half-even bug. roundf(0.5) = 0 but roundf(1.5) = 2 (banker's
 *    rounding). At head_y = N + 0.5 the head would oscillate between
 *    rows N and N+1 across frames. We use floor(x + 0.5) which always
 *    rounds up at .5 — deterministic, no flicker.
 *
 *  • Glyph cache rerolls per TICK, not per frame. With shimmer rate
 *    SHIMMER_HZ = 20 the cache changes 20 times/sec while the render
 *    runs at 60 fps. Frames in the same tick reuse the same glyphs;
 *    that's why the shimmer reads as 20 Hz "blink" instead of 60 Hz
 *    blur. Tweak SHIMMER_HZ to taste.
 *
 *  • Trail off the top. While the head climbs from above-screen
 *    (head_y < 0), some trail rows have negative row indices. We
 *    clip via `if (row < 0 || row >= rows) continue;` — no special
 *    case needed.
 *
 *  • TRAIL_MAX is the array bound on glyphs[]. Random length picks
 *    from [TRAIL_MIN, TRAIL_MAX] inclusive; never set the runtime
 *    length above TRAIL_MAX or you read past the array.
 *
 *  • Resize. SIGWINCH triggers rain_init with the new (cols, rows).
 *    All in-flight streams are reset; cosmetic flicker for one frame.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Slow the speed scale to its minimum ('[' a few times) and watch
 *    a single head: it should slide CONTINUOUSLY between integer
 *    rows, not jump. That's the float head_y doing its job.
 *
 *  • Press space. Streams freeze in place; resume picks up exactly
 *    where they left off.
 *
 *  • Press 't' through all four themes. The HEAD glyph stays pure
 *    white in every theme; the trail bands change hue.
 *
 *  • '+' down to density 1 fills every column; '-' up to density 6
 *    leaves about 1 in 6 columns active.
 *
 *  • Eyeball one column over a few seconds: the head should fall a
 *    distance ≈ speed · time. With speed = 20 rows/sec, the head
 *    crosses a 30-row screen in ~1.5 seconds.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. This is the FOUNDATIONAL file of matrix_rain/; the
 *      other files (snowflake, pulsar, sun, fireworks) reuse the
 *      same shimmer-cache trick on different geometries.
 *   2. §4 column — Column type + spawn + tick + draw + band lookup.
 *      THE HEART of the file. Read AFTER tutorials T1-T5.
 *   3. §5 rain — orchestrator: array of Columns, per-frame tick,
 *      respawn handling.
 *   4. §3 color — 4 themed 5-shade palettes + HUD pairs.
 *   5. §1 + §2 + §6 + §7 — config, clock, screen, app loop.
 *      Skim if you've seen the framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   head_y         FLOAT row position of the column's bright head.
 *                  Increases over time (downward). Float, not int —
 *                  the smooth fall depends on it.
 *   length         number of trail glyphs CURRENTLY VISIBLE per
 *                  column (≤ TRAIL_MAX).
 *   speed          this column's per-second fall rate (rows/sec).
 *   glyphs[k]      k-th cached glyph — k = 0 is the head, k =
 *                  length-1 is the tail tip.
 *   active         bool — false when the column has no in-flight
 *                  stream.
 *   dist           render-time variable — distance ABOVE the head
 *                  (0 = head, 1 = HOT, ..., length-1 = FADE tail
 *                  tip).
 *   speed_scale    user-tunable global multiplier (1.0 default).
 *   density        spawn 1 column per `density` terminal columns
 *                  (1 = every column, 6 = sparse).
 *   PAIR_HEAD      pure-white pair, theme-independent.
 *   PAIR_HOT/BRIGHT/MID/DARK/FADE
 *                  per-band colour pairs that change with the theme.
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

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Six tutorials that build the Matrix rain effect from first
 * principles.
 *
 *   T1  The "stream per column" decomposition
 *   T2  Float head_y — why the fall must be sub-cell smooth
 *   T3  The shimmer cache — what makes glyphs FEEL alive
 *   T4  Brightness bands — how the trail fades over distance
 *   T5  Respawn logic — keeping the screen perpetually busy
 *   T6  Why this is CELL-SPACE (no §4 coords section)
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  THE "STREAM PER COLUMN" DECOMPOSITION
 * ─────────────────────────────────────────
 * The Matrix rain LOOKS like a sea of falling glyphs. The trick:
 * decompose into a flat array of INDEPENDENT streams — one per
 * terminal column.
 *
 *     struct Column {
 *       float head_y;          row position of bright head
 *       int   length;          how long the trail is (1..TRAIL_MAX)
 *       float speed;           rows / second
 *       char  glyphs[MAX];     cached glyphs of the trail
 *       bool  active;          is there a stream falling here?
 *     };
 *     struct Rain {
 *       Column cols[ncols];    one entry per terminal column
 *     };
 *
 * Streams in different columns are completely INDEPENDENT — no
 * physics couples them. Each column ticks itself, rerolls its
 * own cache, decides its own respawn timing.
 *
 *      ┌──────────────────────────────────────────────┐
 *      │  the rain decomposed into vertical streams:  │
 *      │                                              │
 *      │  col 0   col 1   col 2   col 3   col 4       │
 *      │   X       e       7              .           │
 *      │   q       z       k              5           │
 *      │   ▼       L       z              S           │
 *      │           o       2              ▼           │
 *      │           ▼       7                          │
 *      │                   8                          │
 *      │                   ▼                          │
 *      │                                              │
 *      │  ▼ marks each column's HEAD                  │
 *      └──────────────────────────────────────────────┘
 *
 * The screen is the parallel rendering of all those streams.
 * The "rain" emerges from independent, non-interacting parts.
 *
 * Same pattern in every variant of this file:
 *   - matrix_snowflake: streams of glyphs blown by lateral wind.
 *   - pulsar_rain     : radial outward streams from screen centre.
 *   - sun_rain        : streams emerge from a sun disc.
 *   - fireworks_rain  : streams ARE arcs from explosion centres.
 *
 * Different geometries, same "stream + cache" core.
 *
 * T2  FLOAT head_y — WHY THE FALL MUST BE SUB-CELL SMOOTH
 * ───────────────────────────────────────────────────────
 * The terminal renders at INTEGER cell positions — there's no
 * such thing as "between row 5 and row 6." But if we updated
 * head_y as an INTEGER ("advance by 1 row each tick"), the
 * fall would step in jagged jumps:
 *
 *     tick 1:  head at row 5
 *     tick 2:  head at row 6
 *     tick 3:  head at row 7
 *
 * Slow streams visibly stutter; fast streams skip rows. Both
 * look bad.
 *
 * Solution: head_y is a FLOAT advanced by speed · dt:
 *
 *     head_y += speed · dt · speed_scale
 *
 * Then at render time, MAP THE FLOAT to the nearest integer row:
 *
 *     row = floor(head_y - dist + 0.5)        ← round half UP
 *
 * Now the head can be at "row 5.7" — at render time it paints
 * row 6, but on the next frame at "row 5.8" it stays on row 6,
 * and at "row 6.5" it crosses to row 7. The fall reads as
 * smooth even though the rendering is integer.
 *
 * The "round half up" via floor(x + 0.5) is deliberate. C's
 * stdlib roundf() uses "banker's rounding" (round half EVEN)
 * which would oscillate at exactly x.5 boundaries — head_y =
 * 5.5 → row 6, head_y = 6.5 → row 6 again, then head_y = 7.5
 * → row 8. That gives a one-frame "stall and skip" that the
 * eye notices. floor(x + 0.5) is monotonic and predictable.
 *
 * T3  THE SHIMMER CACHE — WHAT MAKES GLYPHS FEEL ALIVE
 * ────────────────────────────────────────────────────
 * Without rerolling glyphs, the falling trail would look like a
 * static piece of text scrolling — each glyph keeps the same
 * character forever, just with changing brightness. Boring.
 *
 * The trick: cache ONE glyph per trail position, and REROLL
 * those glyphs periodically (the "shimmer cache").
 *
 *     glyphs[length] = static array per column
 *     SHIMMER_HZ = 20      ← rerolls per second
 *
 *     every shimmer tick:
 *       for k in 0..length-1:
 *         glyphs[k] = random ASCII printable char
 *
 * The shimmer happens at a SLOWER rate than the render
 * (SHIMMER_HZ = 20 vs render fps = 60). Within one shimmer tick
 * (50 ms), three render frames see the SAME cache. That gives
 * the eye a perceptible BLINK PATTERN — each glyph holds its
 * value for 50 ms, then changes. Faster than the eye registers
 * as discrete blinks; slower than the render so the trajectory
 * is still smooth.
 *
 * SHIMMER_HZ tuning:
 *   - Too low (5 Hz):  glyphs READ AS letters, you can almost
 *                      tell what they are. Reads as "scrolling
 *                      text," not Matrix rain.
 *   - Too high (60 Hz): glyphs become an unrecognisable blur,
 *                      no individual character readable. Rain
 *                      reads as PURE NOISE rather than "code."
 *   - 20 Hz: sweet spot — characters glimpsed but not read.
 *
 * T4  BRIGHTNESS BANDS — HOW THE TRAIL FADES OVER DISTANCE
 * ────────────────────────────────────────────────────────
 * The visible trail isn't uniformly bright. It has SIX BANDS
 * of progressively darker colour from head to tip:
 *
 *     dist=0        HEAD     pure white BOLD
 *     dist=1        HOT      theme[4] BOLD
 *     dist=2        BRIGHT   theme[3] BOLD
 *     dist=3..k     MID      theme[2]
 *     dist=k+1..N-2 DARK     theme[1]
 *     dist=N-1      FADE     theme[0] DIM
 *
 * Where do the band BOUNDARIES live? §4's col_band function
 * computes them from `length` so a short trail has the same
 * head/hot/bright/fade structure as a long trail — just with
 * less MID in the middle.
 *
 * The HEAD band is theme-INDEPENDENT (always pure white). Why?
 * Because:
 *   - White provides a clear visual cue: "this is the leading
 *     edge of a stream." Without it the head blends into the
 *     trail.
 *   - It works regardless of theme. Every other band shifts hue
 *     across themes (green/amber/blue/white); HEAD stays bright
 *     white, anchoring the visual against any background.
 *
 * Per-theme palette: 5 256-colour codes per theme, indexed
 * theme[0..4]. Strict brightness gradient: theme[0] is DIM-safe
 * (≥ palette index 24), theme[4] is the brightest. Compliance
 * with the project's "Theme Palette Brightness" rule —
 * every band stays legible against default-black with A_DIM.
 *
 * T5  RESPAWN LOGIC — KEEPING THE SCREEN PERPETUALLY BUSY
 * ───────────────────────────────────────────────────────
 * When a stream's tail clears the bottom edge, the column goes
 * INACTIVE. If we never respawned, columns would empty out and
 * the screen would gradually go dark.
 *
 * Respawn rule: per inactive column, per frame, with small
 * probability:
 *
 *     respawn_p = RESPAWN_RATE_PER_SEC · dt / density
 *     if (random() < respawn_p):
 *       spawn a fresh stream at this column
 *
 * `RESPAWN_RATE_PER_SEC` is the EXPECTED number of new streams
 * per column per second. With density = 3 and rate = 1, each
 * inactive column has a 1-in-3 chance per second of spawning.
 *
 * Spawn parameters are FRESH for every new stream:
 *
 *     head_y   = uniform(-rows / 2, 0)        ← above the screen
 *     length   = uniform(TRAIL_MIN, TRAIL_MAX)
 *     speed    = uniform(SPEED_MIN, SPEED_MAX)
 *     glyphs[] = randomised
 *
 * Initial head_y < 0 means the new stream FALLS IN from above
 * the screen — visually graceful, not a sudden pop at the top
 * row. By the time head_y reaches row 0, the stream looks like
 * it has always been there.
 *
 * Density (the `=` and `-` keys) effectively scales the per-
 * column respawn rate. density=1: every column eligible →
 * dense screen. density=6: only ~1 in 6 columns active at once
 * → sparse / minimal aesthetic.
 *
 * T6  WHY THIS IS CELL-SPACE (NO §4 coords SECTION)
 * ─────────────────────────────────────────────────
 * The project framework distinguishes two coordinate spaces:
 *
 *   PIXEL-SPACE  continuous floats; needs §4 coords for
 *                pixel↔cell mapping. Used by bouncing balls,
 *                Lorenz attractors, FK creatures.
 *
 *   CELL-SPACE   discrete grid of terminal cells. Each cell IS
 *                one character. No sub-pixel motion.
 *
 * matrix_rain is CELL-SPACE — there's no §4 coords. ONE float
 * (head_y) is the only continuous quantity, and it converts to
 * an integer row directly via floor(x + 0.5). Everything else
 * is integer column / row indexing.
 *
 * The decision is the FIRST ARCHITECTURAL CHOICE for any new
 * sim:
 *   - "Each cell IS one character / particle / dot" → CELL-SPACE.
 *     Examples: matrix_rain, fire, sand, reaction-diffusion.
 *   - "Smooth motion, decoupled from cell grid" → PIXEL-SPACE.
 *     Examples: bounce_ball, nbody, cloth, all of animation/.
 *
 * matrix_rain landed in cell-space because:
 *   - The visual is GLYPH-CENTRIC: each cell IS one Matrix
 *     character. There's no "between cells" position that
 *     needs to be drawn.
 *   - Streams fall straight down: only one continuous parameter
 *     (vertical position) needs floats. Horizontal is locked
 *     to terminal columns.
 *
 * Comparing to fireworks_rain (T1 there): the arc trails curve
 * smoothly but the file is STILL cell-space because every
 * arc point is rounded to a terminal cell at render time —
 * the ONE float per particle is its progress along the arc.
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
 * Theme — five fg colours for the five trail bands (FADE..HOT).
 * The HEAD band is always white, theme-independent.
 *
 * Every entry is in the bright half of the 256-colour space
 * (CLAUDE.md brightness rule: cube ≥ 24, grayscale ≥ 240).
 *
 * Themes:
 *   green  — classic Matrix
 *   amber  — orange / sodium-vapour
 *   blue   — cool, cyber-noir
 *   white  — black & white film grain
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
 * Column — one falling stream of characters.
 *
 *   col          terminal column index (x); set once at spawn.
 *   head_y       current head row, FLOAT so motion is sub-row smooth.
 *   speed        rows per second; constant per stream, varied across
 *                streams so different columns fall at different rates.
 *   trail_len    total length of the visible snake (head + tail).
 *   glyphs[]     cached random ASCII characters, one per trail slot.
 *                glyphs[0] is the head; glyphs[trail_len-1] is the tip.
 *                Rerolled at SHIMMER_HZ to produce the shimmer effect.
 *   active       true while at least part of the trail is on screen.
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
/* §5  rain                                                               */
/* ===================================================================== */

/*
 * Rain — one Column slot per terminal column, plus runtime knobs.
 *
 *   columns        flat array, [x] is the stream for terminal column x.
 *                  Zero-initialised to active=false; populated by spawn.
 *   ncols, nrows   current terminal extents.
 *   density        spacing — only every density-th column starts active
 *                  at init; respawns happen anywhere but at a rate
 *                  divided by density so sparser settings stay sparser.
 *   speed_scale    global multiplier on per-stream speeds, [/] to tune.
 *   shimmer_accum  seconds since last cache reroll, fires at SHIMMER_HZ.
 *   paused         when true, col_advance and respawn are skipped.
 */
typedef struct {
    Column *columns;
    int     ncols;
    int     nrows;
    int     density;
    float   speed_scale;
    float   shimmer_accum;
    bool    paused;
} Rain;

static void rain_init(Rain *r, int cols, int rows, int density)
{
    r->ncols         = cols;
    r->nrows         = rows;
    r->density       = density;
    r->speed_scale   = SPEED_SCALE_DEFAULT;
    r->shimmer_accum = 0.0f;
    r->paused        = false;
    r->columns       = calloc((size_t)cols, sizeof(Column));

    /* Light up every density-th column at startup. The rest start dead
     * and may wake up via the per-frame respawn roll. */
    for (int x = 0; x < cols; x++) {
        if (x % density == 0)
            col_spawn(&r->columns[x], x, rows);
    }
}

static void rain_free(Rain *r)
{
    free(r->columns);
    *r = (Rain){0};
}

/*
 * rain_tick — advance every active stream by speed · dt, deactivate
 * streams that have fallen off the bottom edge, and roll the dice on
 * each dead column to potentially respawn it. Glyph cache is
 * rerolled at SHIMMER_HZ — independent of the per-frame physics rate.
 */
static void rain_tick(Rain *r, float dt)
{
    if (r->paused) return;

    /* Shimmer pulse: accumulate dt; when we cross the per-tick interval
     * reroll every active column's cache and reset the accumulator. */
    r->shimmer_accum += dt;
    bool shimmer_now = (r->shimmer_accum >= 1.0f / (float)SHIMMER_HZ);
    if (shimmer_now) r->shimmer_accum = 0.0f;

    float scaled_dt = dt * r->speed_scale;
    float respawn_p = (RESPAWN_RATE_PER_SEC * dt) / (float)r->density;

    for (int x = 0; x < r->ncols; x++) {
        Column *c = &r->columns[x];

        if (c->active) {
            if (!col_advance(c, scaled_dt, r->nrows))
                c->active = false;
            else if (shimmer_now)
                col_shimmer(c);
        } else {
            if (urand01() < respawn_p)
                col_spawn(c, x, r->nrows);
        }
    }
}

static void rain_draw(const Rain *r)
{
    for (int x = 0; x < r->ncols; x++) {
        const Column *c = &r->columns[x];
        if (c->active) col_draw(c, r->nrows);
    }
}

/* ===================================================================== */
/* §6  screen                                                             */
/* ===================================================================== */

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
static void screen_draw_hud(const Screen *s, double fps,
                            const Rain *r, int theme_idx)
{
    char buf[HUD_BUF_LEN];
    snprintf(buf, sizeof buf,
             " %5.1f fps  spd:%.2fx  den:%d  [%s] %s ",
             fps, r->speed_scale, r->density,
             k_themes[theme_idx].name,
             r->paused ? "PAUSED " : "running");

    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset  []:speed  +/-:density  t:theme ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §7  app — signals, resize, variable-dt main loop                       */
/* ===================================================================== */

typedef struct {
    Rain                  rain;
    Screen                screen;
    int                   density;
    int                   theme_idx;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/*
 * On resize, free the old Rain and re-init for the new (cols, rows).
 * In-flight streams are lost; cosmetic flicker for one frame.
 */
static void app_do_resize(App *app)
{
    float saved_speed = app->rain.speed_scale;
    rain_free(&app->rain);
    screen_resize(&app->screen);
    rain_init(&app->rain, app->screen.cols, app->screen.rows, app->density);
    app->rain.speed_scale = saved_speed;
    app->need_resize = 0;
}

/*
 * Map one keypress to an action. Returns false on quit, true otherwise.
 * Keys not listed are silently ignored.
 */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {

    case 'q': case 'Q': case 27 /* ESC */:
        return false;

    case ' ':
        app->rain.paused = !app->rain.paused;
        break;

    case 'r': case 'R': {
        float saved = app->rain.speed_scale;
        rain_free(&app->rain);
        rain_init(&app->rain, app->screen.cols, app->screen.rows,
                  app->density);
        app->rain.speed_scale = saved;
        break;
    }

    case ']':
        app->rain.speed_scale *= SPEED_SCALE_STEP;
        if (app->rain.speed_scale > SPEED_SCALE_MAX)
            app->rain.speed_scale = SPEED_SCALE_MAX;
        break;

    case '[':
        app->rain.speed_scale /= SPEED_SCALE_STEP;
        if (app->rain.speed_scale < SPEED_SCALE_MIN)
            app->rain.speed_scale = SPEED_SCALE_MIN;
        break;

    case '=': case '+':
        if (app->density > DENSITY_MIN) {
            app->density--;
            app->rain.density = app->density;
        }
        break;

    case '-':
        if (app->density < DENSITY_MAX) {
            app->density++;
            app->rain.density = app->density;
        }
        break;

    case 't': case 'T':
        app->theme_idx = (app->theme_idx + 1) % THEME_COUNT;
        theme_apply(app->theme_idx);
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

    App *app       = &g_app;
    app->running   = 1;
    app->density   = DENSITY_DEFAULT;
    app->theme_idx = 0;

    screen_init(&app->screen);
    g_has_256 = (COLORS >= 256);
    theme_apply(app->theme_idx);
    hud_pairs_init();
    rain_init(&app->rain, app->screen.cols, app->screen.rows, app->density);

    int64_t last_ns     = clock_ns();
    int64_t fps_accum   = 0;
    int     fps_frames  = 0;
    double  fps_display = 0.0;
    const int64_t TICK_NS = NS_PER_SEC / TARGET_FPS;

    while (app->running) {

        /* (1) handle resize first so subsequent steps see the new size */
        if (app->need_resize) {
            app_do_resize(app);
            last_ns = clock_ns();
        }

        /* (2) measure dt, capped to prevent spiral-of-death */
        int64_t now_ns  = clock_ns();
        int64_t dt_ns   = now_ns - last_ns;
        last_ns         = now_ns;
        float   dt      = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        /* (3) drain input */
        for (int ch; (ch = getch()) != ERR; ) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }

        /* (4) advance physics — ONE call per frame, no accumulator */
        rain_tick(&app->rain, dt);

        /* (5) rolling fps display */
        fps_accum += dt_ns;
        fps_frames++;
        if (fps_accum >= NS_PER_SEC / 2) {       /* 500 ms window */
            fps_display = (double)fps_frames * 1e9
                        / (double)fps_accum;
            fps_accum = 0;
            fps_frames = 0;
        }

        /* (6) draw + present */
        erase();
        rain_draw(&app->rain);
        screen_draw_hud(&app->screen, fps_display, &app->rain,
                        app->theme_idx);
        screen_present();

        /* (7) frame cap — sleep before the NEXT frame's I/O */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(TICK_NS - elapsed);
    }

    rain_free(&app->rain);
    screen_free(&app->screen);
    return 0;
}
