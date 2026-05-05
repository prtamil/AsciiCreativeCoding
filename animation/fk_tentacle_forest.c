/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fk_tentacle_forest.c — eight ocean tentacles swaying in a current
 *
 * DEMO: Eight seaweed strands rooted along the bottom of the terminal
 *       sway left and right in a simulated underwater current. Each
 *       strand has its own phase and a tiny frequency detuning so the
 *       forest never lapses into mechanical lockstep. Motion is pure
 *       stateless forward kinematics — every frame is recomputed from
 *       a closed-form sine of wave_time alone.
 *
 * Study alongside: snake_forward_kinematics.c (path-following FK contrast)
 *                  fk_centipede.c             (same variable-timestep loop)
 *
 * Section map:
 *   §1  config       — all tunables in one place
 *   §2  clock        — monotonic clock + sleep (verbatim from framework)
 *   §3  color        — deep-sea 7-step palette + spec HUD/hint pairs
 *   §4  coords       — pixel↔cell aspect-ratio helpers
 *   §5  entity       — Tentacle: stateless FK chain + two-pass render
 *       §5a  tentacle_tick   — closed-form joint placement
 *       §5b  rendering helpers (seg_pair, seg_attr, seg_glyph)
 *       §5c  draw_segment_dense
 *       §5d  render_tentacle (orchestrator)
 *   §6  scene        — Scene wrapping N_TENTACLES, scene_init/tick/draw
 *   §7  screen       — ncurses double-buffer display layer
 *   §8  app          — signals, resize, main game loop
 *
 * Keys:  q / ESC      quit                space   pause / resume
 *        w / ↑        wave freq + 0.15    s / ↓   wave freq − 0.15
 *        d / →        amplitude + 0.10    a / ←   amplitude − 0.10
 *        [ / ]        time scale (0.25× .. 4×)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra animation/fk_tentacle_forest.c \
 *       -o fk_tentacle_forest -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Stateless forward kinematics for fixed-root chains.
 *                 For each tentacle, every joint position is recomputed
 *                 from scratch each frame by accumulating per-segment
 *                 bend angles up the chain:
 *                     δθᵢ              = amp · sin(ω·t + ph + i·PSEG)
 *                     cumulative_angle += δθᵢ
 *                     joint[i+1]       = joint[i]
 *                                      + seg_len · (cos cumul, sin cumul)
 *                 No history is needed because re-evaluating the formula
 *                 at the same wave_time yields identical positions —
 *                 the chain is fully determined by wave_time alone.
 *                 Per-strand root_phase + tiny freq_offset prevent the
 *                 eight chains from re-synchronising into lockstep.
 *
 * Data-structure: Tentacle holds the fixed root, two per-strand wave
 *                 constants (root_phase, freq_offset), and a single
 *                 joint[] array of (N_SEGS+1) pixel positions. Scene
 *                 owns N_TENTACLES of them plus shared wave parameters.
 *                 No trail buffer, no prev/cur snapshots, no alpha
 *                 interpolation — variable timestep makes them unneeded.
 *
 * Rendering     : Two-pass per tentacle. Pass 1 stamps direction glyphs
 *                 ('-' '/' '|' '\\') along each segment via dense bead-
 *                 stepping; the colour-pair gradient runs deep blue at
 *                 the root → bright yellow-green at the tip with depth-
 *                 cued attributes (A_DIM / A_NORMAL / A_BOLD). Pass 2
 *                 stamps bold node glyphs (#, O, o, ., *) at every joint
 *                 on top of the lines, giving the chain its knuckled
 *                 silhouette. A dim '~' seabed row anchors the scene.
 *
 * Performance   : Variable timestep at render rate — no fixed-step
 *                 accumulator, no alpha lerp, no prev_joint snapshots.
 *                 The whole simulation is closed-form trigonometry
 *                 (non-stiff), so variable dt is provably safe and
 *                 gives motion as smooth as the terminal can render.
 *                 Per frame: O(N_TENTACLES · N_SEGS) — 8 · 16 = 128
 *                 sinf/cosf calls, microseconds total.
 *
 * References    :
 *   Reynolds, "Steering Behaviors for Autonomous Characters" (1999) —
 *     framework for stateless analytic motion. The "wander" behaviour
 *     in particular is conceptually the per-segment δθᵢ used here.
 *     https://www.red3d.com/cwr/steer/
 *   Glenn Fiedler, "Fix Your Timestep!" (gafferongames.com) —
 *     articulates when fixed-step matters and when variable step is
 *     the better tool. Non-stiff sin-based sims fall in the latter camp.
 *   Wikipedia, "Forward kinematics" — the chain-of-rotations method
 *     used in tentacle_tick(); accumulating local bends is the joint-
 *     space FK transform restricted to 2-D.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each tentacle is a chain of stiff segments hinged end-to-end, rooted
 * at a fixed point on the sea floor. At any moment, every segment has
 * a small bend δθᵢ relative to its parent, computed from a sine of
 * (wave_time, segment index, per-strand phase). The world-space
 * direction of segment i is the SUM of all bends from 0 through i, so
 * a wave that ripples through the bend values appears to ripple up the
 * physical chain. Nothing is stored between frames except wave_time.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a marionette with strings attached to every joint, and one
 * sinusoidal hand controls all the strings together — but with a small
 * delay (i·PSEG) for each joint up the chain. The lower joints jerk
 * first, the upper ones follow. That delay is what makes the wave
 * appear to *travel* along the tentacle rather than just shake the
 * whole rod. Per-strand `root_phase` shifts each marionette's hand
 * by a different starting angle; per-strand `freq_offset` shakes each
 * one a hair faster or slower so the eight marionettes never quite re-
 * synchronise.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Measure dt = wall-clock since last frame; multiply by time_scale.
 *  2. wave_time += dt   (the only persistent simulation state).
 *  3. For each tentacle k:
 *       a. joint[0] = (root_px, root_py)              (fixed anchor)
 *       b. cumulative_angle = −π/2                    (straight up)
 *       c. for i in 0..N_SEGS−1:
 *            δθᵢ = amp · sin((freq + freq_offset[k])·wave_time
 *                            + root_phase[k] + i·PHASE_PER_SEG)
 *            cumulative_angle += δθᵢ
 *            joint[i+1] = joint[i] + seg_len ·
 *                                    (cos cumulative_angle,
 *                                     sin cumulative_angle)
 *  4. Render two-pass: segment dense-fill glyphs first, then bold
 *     node markers on top. Repeat for all tentacles.
 *
 * KEY FORMULAS
 * ────────────
 *  Per-segment bend  : δθᵢ = amp · sin((freq + freq_offset)·wave_time
 *                                        + root_phase + i·PHASE_PER_SEG)
 *  Cumulative angle  : ang_i = base + Σ_{j<i} δθⱼ      (FK accumulation)
 *  Segment placement : joint[i+1] = joint[i]
 *                                 + seg_len · (cos ang_i, sin ang_i)
 *  Wave travel speed : freq / PHASE_PER_SEG  segments per second
 *                      (the same phase appears at increasing i over time)
 *  Total spatial span: N_SEGS · PHASE_PER_SEG = 7.2 rad ≈ 1.15 cycles
 *                      → roughly one S-curve visible from root to tip
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  - High amplitude (above ~0.5 rad/seg) lets the cumulative angle
 *    wrap, so the chain can fold back on itself. Visible as a tight
 *    spiral when AMP is cranked. Not a bug — but the `AMP_MAX = 1.2`
 *    ceiling is intentionally just shy of the spiral threshold.
 *
 *  - Pause: scene_tick() returns early so wave_time stops advancing.
 *    Joints stay at the moment of pause. No prev/cur lerp needed.
 *
 *  - Resize: roots and seg_len_px must be recomputed for the new
 *    geometry. wave_time MUST be preserved so the animation continues
 *    from the same point in its cycle, not snapped back to t=0.
 *
 *  - Suspend / lid-close: dt can grow huge between frames; capped at
 *    100 ms so the tentacles don't suddenly flail through ~hundreds of
 *    radians of accumulated wave_time.
 *
 *  - Glyph aliasing: the four ASCII line glyphs sample a continuous
 *    angle. Near boundaries (22.5°, 67.5°, ...) the glyph flickers as
 *    the segment angle crosses. Folding to [0°, 180°) halves the
 *    boundary crossings; residual flicker visible only at very low
 *    time_scale.
 *
 * HOW TO VERIFY
 * ─────────────
 *  - Default config: each tentacle shows roughly one visible S-curve
 *    (~1.15 cycles); the eight chains are clearly out of phase, never
 *    all bending the same direction at once.
 *  - Press space → tentacles freeze instantly. Un-pause → motion
 *    resumes exactly from the freeze frame (no jitter).
 *  - Crank amplitude with `d` → curves get more dramatic. The cumulative
 *    angle never makes the chain stretch (segments stay seg_len_px
 *    apart) regardless of amp.
 *  - Set frequency to 0 → all chains hold their last shape (sin
 *    argument stops advancing); they remain in whatever pose wave_time
 *    last placed them.
 *  - Press `[` to slow time (4× slow-mo) → motion is buttery; press
 *    `]` to fast-forward to 4× → still smooth (variable timestep
 *    guarantees this).
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

/* M_PI is a POSIX extension, not standard C99/C11 — provide a fallback
 * so the build never fails on strict-conformance toolchains. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
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

/* All magic numbers live here. Never scatter literals through the code. */
enum {
    /* Render frame-rate target. Variable-timestep simulation, so the
     * only thing this controls is the sleep cap at end of frame. */
    TARGET_FPS    = 60,

    /* HUD layout. */
    HUD_COLS      = 96,    /* status-bar byte budget        */
    FPS_UPDATE_MS = 500,   /* fps display refresh interval  */

    /* ncurses pair IDs. Pairs 1..N_PAIRS = body gradient (root→tip);
     * PAIR_HUD/PAIR_HINT are reserved per CLAUDE.md HUD spec. */
    N_PAIRS   = 7,
    PAIR_HUD  = 8,
    PAIR_HINT = 9,

    /*
     * N_TENTACLES — strands rooted along the sea floor. 8 fills an
     * 80–220-column terminal comfortably with even spacing.
     *
     * N_SEGS — rigid segments per chain. More segments → smoother
     * curvature; 16 at the dynamic seg_len_px gives one visible S-curve
     * with no visible faceting at typical terminal sizes.
     */
    N_TENTACLES = 8,
    N_SEGS      = 16,
};

/*
 * DRAW_STEP_PX — pixel stride for dense glyph stamping in §5c. Must be
 * < CELL_W (8) so a near-horizontal segment never skips a column.
 *
 * 5 px ≈ 0.625 cell widths — guarantees every cell the segment crosses
 * is sampled at least once, while leaving texture sparse enough that
 * the §5d node markers can read through the fill clearly.
 */
#define DRAW_STEP_PX   5.0f

/*
 * PHASE_PER_SEG — per-segment phase advance (radians). Determines how
 * fast the wave appears to travel up the chain.
 *
 * Geometric interpretation:
 *   0.0   → all segments in phase → chain bends as a rigid rod
 *   π/2   → quarter wavelength per segment → tight zigzag
 *   0.45  → ~1/14 wavelength per segment → one gentle S-curve over
 *           N_SEGS=16 (16·0.45 = 7.2 rad ≈ 1.15 cycles per chain)
 */
#define PHASE_PER_SEG  0.45f

/*
 * Wave amplitude and frequency — user-tunable at runtime.
 *
 *   AMP   ≈ 0.28 rad ≈ 16° peak per-segment bend at default. With 16
 *         segments and PHASE_PER_SEG distributing the bends, the typical
 *         tip deflection is around ±2 rad (≈115° from vertical) — vigorous
 *         but non-spiralling sway.
 *
 *   FREQ  ≈ 0.8 rad/s → period 2π/0.8 ≈ 7.9 s per oscillation, matching
 *         the leisurely pace of real seaweed in a gentle tidal current.
 */
#define AMP_DEFAULT    0.28f
#define AMP_MIN        0.0f
#define AMP_MAX        1.2f
#define FREQ_DEFAULT   0.8f
#define FREQ_MIN       0.1f
#define FREQ_MAX       5.0f

/* Per-strand frequency detuning amplitude. Each tentacle gets a tiny
 * additive offset on the global frequency so they never re-synchronise.
 * Magnitude 0.04 → adjacent strands drift ~0.32 rad apart per cycle. */
#define FREQ_OFFSET_MAG  0.04f

/*
 * Time scale — user-controlled simulation speed multiplier.
 * 0.25× to 4×, default 1×. Stepped by ×/÷ TIME_SCALE_STEP via [/].
 */
#define TIME_SCALE_DEFAULT  1.0f
#define TIME_SCALE_MIN      0.25f
#define TIME_SCALE_MAX      4.0f
#define TIME_SCALE_STEP     1.5f

/* Timing primitives — verbatim from framework.c. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* Terminal cell dimensions — the aspect-ratio bridge between physics
 * and display. 1 px represents the same physical distance in x and y;
 * a typical cell is ~2× taller than wide. */
#define CELL_W   8
#define CELL_H  16

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);    /* never goes backward */
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;                   /* over-budget frame: skip */
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color — deep-sea 7-step palette + spec-fixed HUD pairs            */
/* ===================================================================== */

/*
 * The body gradient runs deep blue at the root → bright yellow-green
 * at the tip, evoking water depth (cold/dark below, sunlit/warm above).
 *
 * All entries sit in the bright half of the 256-colour space (>= 24 in
 * the cube) per CLAUDE.md theme palette brightness rule, so even the
 * dim root segments stay visible under A_DIM rendering.
 *
 * HUD/HINT pairs are theme-independent (CLAUDE.md HUD spec) — bright
 * yellow status, bright cyan hint, both on default background so they
 * stay readable against any animation behind them.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(1,  24, -1);   /* deep blue       — root, sea floor    */
        init_pair(2,  27, -1);   /* medium blue                          */
        init_pair(3,  33, -1);   /* cyan-blue                            */
        init_pair(4,  51, -1);   /* bright cyan     — mid-body           */
        init_pair(5,  86, -1);   /* cyan-green                           */
        init_pair(6, 118, -1);   /* yellow-green                         */
        init_pair(7, 154, -1);   /* bright yellow-green — tip, sunlit    */
    } else {
        /* 8-color fallback — coarser gradient, still directionally right. */
        init_pair(1, COLOR_BLUE,  -1);
        init_pair(2, COLOR_BLUE,  -1);
        init_pair(3, COLOR_CYAN,  -1);
        init_pair(4, COLOR_CYAN,  -1);
        init_pair(5, COLOR_CYAN,  -1);
        init_pair(6, COLOR_GREEN, -1);
        init_pair(7, COLOR_GREEN, -1);
    }

    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ===================================================================== */
/* §4  coords — pixel↔cell; the one aspect-ratio fix                     */
/* ===================================================================== */

/*
 * All entity positions live in square pixel space (1 unit = 1 px).
 * Only at draw time do these helpers convert to cell coordinates,
 * undoing the 8:16 cell aspect ratio.
 *   cell = floor(px / CELL_DIM + 0.5)    — nearest-integer rounding
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  entity — Tentacle: stateless FK chain                             */
/* ===================================================================== */

/* Vec2 — 2-D position vector in pixel space.
 * x increases eastward; y increases downward (terminal convention). */
typedef struct { float x, y; } Vec2;

/*
 * Tentacle — one seaweed strand.
 *
 * The five state fields divide cleanly into "fixed at init" (all of
 * them) and "computed each frame" (joint[]). No prev/cur snapshot is
 * needed: the entire chain is a closed-form function of wave_time.
 */
typedef struct {
    /* fixed root anchor (pixel space) */
    float root_px, root_py;

    /* per-strand wave constants — set once in scene_init */
    float root_phase;     /* phase stagger (0..2π)                   */
    float freq_offset;    /* tiny freq detuning, prevents re-sync     */

    /* joint chain — computed each frame */
    Vec2  joint[N_SEGS + 1];   /* [0]=root, [N_SEGS]=tip            */
} Tentacle;

/* ── §5a  tentacle_tick ─────────────────────────────────────────────── */

/*
 * tentacle_tick — recompute all joint positions from wave_time.
 *
 * The cumulative-angle FK trick: each segment's WORLD direction is the
 * SUM of all upstream bends. By seeding cumulative_angle at −π/2
 * ("up" in pixel space) and adding δθᵢ each iteration, every segment
 * inherits all the bends from segments below it — that's how a wave at
 * the root propagates to the tip without any explicit force model.
 *
 * Per-strand freq_offset and root_phase enter the sine argument so
 * each chain oscillates at a slightly different rate and starting phase.
 */
static void tentacle_tick(Tentacle *t,
                          float wave_time,
                          float amplitude,
                          float frequency,
                          float seg_len_px)
{
    t->joint[0].x = t->root_px;
    t->joint[0].y = t->root_py;

    float cumulative_angle = -(float)M_PI * 0.5f;   /* start pointing up */

    for (int i = 0; i < N_SEGS; i++) {
        float delta = amplitude
                    * sinf((frequency + t->freq_offset) * wave_time
                           + t->root_phase
                           + (float)i * PHASE_PER_SEG);
        cumulative_angle += delta;

        t->joint[i + 1].x = t->joint[i].x
                          + seg_len_px * cosf(cumulative_angle);
        t->joint[i + 1].y = t->joint[i].y
                          + seg_len_px * sinf(cumulative_angle);
    }
}

/* ── §5b  rendering helpers ─────────────────────────────────────────── */

/* seg_pair — body gradient pair for segment i.
 *   i = 0           → pair 1       (root, deep blue)
 *   i = N_SEGS - 1  → pair N_PAIRS (tip, bright yellow-green)
 * Integer linear interpolation across the gradient. */
static int seg_pair(int i)
{
    return 1 + (i * (N_PAIRS - 1)) / (N_SEGS - 1);
}

/* seg_attr — depth-cued attribute for segment i.
 *   root quarter : A_DIM    (deep water absorbs light)
 *   middle half  : A_NORMAL (neutral)
 *   tip quarter  : A_BOLD   (sunlit shallows, vibrant tip)         */
static attr_t seg_attr(int i)
{
    if (i <     N_SEGS / 4) return A_DIM;
    if (i > 3 * N_SEGS / 4) return A_BOLD;
    return A_NORMAL;
}

/* node_marker — bold joint glyph by position along chain.
 *   0          '#'  root anchor — thick, grounded
 *   upper qtr  'O'  lower body  — wide, fat node
 *   middle     'o'  mid body    — medium node
 *   lower qtr  '.'  upper body  — small, wispy
 *   N_SEGS     '*'  tip         — bright spark, free end           */
static chtype node_marker(int i)
{
    if (i == 0)                  return (chtype)(unsigned char)'#';
    if (i <=     N_SEGS / 4)     return (chtype)(unsigned char)'O';
    if (i <= 3 * N_SEGS / 4)     return (chtype)(unsigned char)'o';
    if (i <  N_SEGS)             return (chtype)(unsigned char)'.';
    return                              (chtype)(unsigned char)'*';
}

/*
 * seg_glyph — best ASCII direction glyph for vector (dx, dy).
 *
 * Glyphs partition the angular circle, folded to [0°, 180°) since each
 * glyph is symmetric under 180° rotation:
 *   '-'  near-horizontal     ( 0°,  22.5° ) ∪ (157.5°, 180°)
 *   '\'  down-right diagonal ( 22.5°,  67.5° )
 *   '|'  near-vertical       ( 67.5°, 112.5° )
 *   '/'  down-left diagonal  (112.5°, 157.5° )
 *
 * dy is negated before atan2f so the angle matches visual direction
 * (terminal y grows down; math y grows up).
 */
static chtype seg_glyph(float dx, float dy)
{
    float ang = atan2f(-dy, dx);
    float deg = ang * (180.0f / (float)M_PI);
    if (deg <    0.0f) deg += 360.0f;
    if (deg >= 180.0f) deg -= 180.0f;

    if (deg < 22.5f || deg >= 157.5f) return (chtype)(unsigned char)'-';
    if (deg < 67.5f)                   return (chtype)(unsigned char)'\\';
    if (deg < 112.5f)                  return (chtype)(unsigned char)'|';
    return                             (chtype)(unsigned char)'/';
}

/* ── §5c  draw_segment_dense ────────────────────────────────────────── */

/*
 * draw_segment_dense — stamp a direction glyph at every cell the line
 * a→b passes through.
 *
 * WHY DENSE STEPPING? A cell is CELL_W=8 px wide. Drawing only at the
 * endpoints leaves visible gaps on segments longer than one cell.
 * Stepping every DRAW_STEP_PX=5 px (< CELL_W) guarantees at least one
 * sample per cell along the segment.
 *
 * DEDUP CURSOR (prev_cx, prev_cy): caller-owned so it persists across
 * the segments of one tentacle. Initialise to a sentinel (-9999) before
 * the first segment so the first cell always draws. Sharing across
 * segments prevents double-stamping at segment boundaries.
 *
 * Rows 0 and rows-1 are skipped — those are the HUD bars.
 */
static void draw_segment_dense(WINDOW *w,
                                Vec2 a, Vec2 b,
                                int pair, attr_t attr,
                                int cols, int rows,
                                int *prev_cx, int *prev_cy)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;       /* degenerate: nothing to draw */

    chtype glyph  = seg_glyph(dx, dy);
    int    nsteps = (int)ceilf(len / DRAW_STEP_PX) + 1;

    for (int s = 0; s <= nsteps; s++) {
        float u  = (float)s / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == *prev_cx && cy == *prev_cy) continue;
        *prev_cx = cx;
        *prev_cy = cy;

        if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1) continue;

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, glyph);
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/* ── §5d  render_tentacle ───────────────────────────────────────────── */

/* draw_tentacle_lines — pass 1: dense direction-glyph fill, root → tip.
 * Drawing root → tip means upper segments overwrite lower ones where
 * the chain curves back on itself (tight curls at high amplitude). */
static void draw_tentacle_lines(const Tentacle *t, WINDOW *w,
                                int cols, int rows)
{
    int prev_cx = -9999, prev_cy = -9999;   /* dedup cursor */
    for (int i = 0; i < N_SEGS; i++) {
        draw_segment_dense(w, t->joint[i], t->joint[i + 1],
                           seg_pair(i), seg_attr(i),
                           cols, rows, &prev_cx, &prev_cy);
    }
}

/* draw_tentacle_nodes — pass 2: bold node glyph at each joint, on top
 * of the line fill. Marker shape encodes position along the chain (#
 * grounded root → * bright tip), reinforcing the size-gradient look. */
static void draw_tentacle_nodes(const Tentacle *t, WINDOW *w,
                                int cols, int rows)
{
    for (int i = 0; i <= N_SEGS; i++) {
        int cx = px_to_cell_x(t->joint[i].x);
        int cy = px_to_cell_y(t->joint[i].y);
        if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1) continue;

        int    pair  = seg_pair(i < N_SEGS ? i : N_SEGS - 1);
        chtype glyph = node_marker(i);

        wattron(w, COLOR_PAIR(pair) | A_BOLD);
        mvwaddch(w, cy, cx, glyph);
        wattroff(w, COLOR_PAIR(pair) | A_BOLD);
    }
}

/* render_tentacle — orchestrator. Painter's order: lines first (so node
 * markers always overwrite line glyphs at joint cells), nodes on top. */
static void render_tentacle(const Tentacle *t, WINDOW *w,
                            int cols, int rows)
{
    draw_tentacle_lines(t, w, cols, rows);
    draw_tentacle_nodes(t, w, cols, rows);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene — the tentacle forest. All strands share wave_time, amplitude,
 * frequency, and seg_len_px (recomputed on resize from terminal height).
 */
typedef struct {
    Tentacle t[N_TENTACLES];

    /* shared simulation state */
    float wave_time;       /* monotonic sim clock (s)                */
    float amplitude;       /* peak per-segment bend (rad), tunable   */
    float frequency;       /* base oscillation rate (rad/s), tunable */
    float seg_len_px;      /* segment length (px), set from screen   */

    bool  paused;
} Scene;

/*
 * scene_init — distribute roots evenly along the sea floor and assign
 * per-strand phase + frequency offsets.
 *
 * Non-obvious bits:
 *  - Root spacing uses (N_TENTACLES + 1) divisor so no strand sits at
 *    the screen edge — fractions 1/9, 2/9, ... 8/9 give equal margins.
 *  - root_py = bottom − 4 px places roots inside the seabed '~' row.
 *  - root_phase is evenly distributed over [0, 2π) for max initial
 *    desynchronisation.
 *  - freq_offset is centred symmetrically around 0 so the average
 *    frequency equals the user's `frequency` value (no global drift).
 *  - seg_len_px = 55% of screen height divided across N_SEGS — tips
 *    visually terminate near mid-screen even at full sway amplitude.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    sc->amplitude  = AMP_DEFAULT;
    sc->frequency  = FREQ_DEFAULT;
    sc->wave_time  = 0.0f;
    sc->paused     = false;
    sc->seg_len_px = (float)(rows * CELL_H) * 0.55f / (float)N_SEGS;

    float screen_wpx = (float)(cols * CELL_W);
    float root_py    = (float)(rows * CELL_H) - 4.0f;

    for (int i = 0; i < N_TENTACLES; i++) {
        Tentacle *t = &sc->t[i];

        t->root_px     = (float)(i + 1) * screen_wpx
                       / (float)(N_TENTACLES + 1);
        t->root_py     = root_py;
        t->root_phase  = (float)i * 2.0f * (float)M_PI
                       / (float)N_TENTACLES;
        t->freq_offset = ((float)i - (float)N_TENTACLES * 0.5f)
                       * FREQ_OFFSET_MAG;

        /* Seed all joints straight up — overwritten on first tick. */
        for (int k = 0; k <= N_SEGS; k++) {
            t->joint[k].x = t->root_px;
            t->joint[k].y = t->root_py - (float)k * sc->seg_len_px;
        }
    }
}

/*
 * scene_tick — one variable-timestep update. dt is the wall-clock
 * delta scaled by the caller's time_scale. When paused, do nothing —
 * wave_time is frozen, so the next render produces an identical frame.
 */
static void scene_tick(Scene *sc, float dt)
{
    if (sc->paused) return;
    sc->wave_time += dt;
    for (int i = 0; i < N_TENTACLES; i++) {
        tentacle_tick(&sc->t[i], sc->wave_time,
                      sc->amplitude, sc->frequency, sc->seg_len_px);
    }
}

/* draw_seabed — dim '~' row beneath the tentacles for atmosphere.
 * The '~' is the traditional ASCII wave glyph. A_DIM keeps it visually
 * receding behind the bright tentacle roots. */
static void draw_seabed(WINDOW *w, int cols, int rows)
{
    int seabed_row = rows - 2;     /* rows-1 is the hint bar */
    if (seabed_row < 1) return;

    wattron(w, COLOR_PAIR(1) | A_DIM);
    for (int c = 0; c < cols; c++)
        mvwaddch(w, seabed_row, c, (chtype)(unsigned char)'~');
    wattroff(w, COLOR_PAIR(1) | A_DIM);
}

/* scene_draw — read-only render: tentacles + seabed atmosphere. */
static void scene_draw(const Scene *sc, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_TENTACLES; i++)
        render_tentacle(&sc->t[i], w, cols, rows);
    draw_seabed(w, cols, rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

/* The non-obvious call here is typeahead(-1): without it, ncurses peeks
 * at stdin during output writes, which can tear frames mid-update. */
static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

/* SIGWINCH path: endwin()+refresh() forces ncurses to re-read LINES/COLS
 * from the kernel; we then sample the fresh dimensions. */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw — compose one full frame:
 *   erase → tentacles + seabed → status (top right) → key hint (bottom).
 *
 * HUD pairs are spec-fixed (PAIR_HUD = bright yellow, PAIR_HINT = bright
 * cyan, both A_BOLD on default bg) so they read against any animation
 * behind them. NEVER use A_DIM on the hint strip — it disappears on
 * bright frames.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, float time_scale)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %.1ffps  %.2fx  amp:%.2f  freq:%.2f  %s ",
             fps, time_scale, sc->amplitude, sc->frequency,
             sc->paused ? "PAUSED" : "swaying");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  w/s:freq  a/d:amp  [/]:time ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level state. File-scope global so signal handlers (no user
 * argument) can write the running / need_resize flags.
 *
 * volatile sig_atomic_t: volatile prevents register caching across
 * handler writes; sig_atomic_t guarantees atomic read/write on POSIX.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    float                 time_scale;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/* atexit safety net — endwin() called on every exit path. */
static void cleanup(void) { endwin(); }

/*
 * app_do_resize — handle a pending SIGWINCH.
 *
 * Geometry (root_px, root_py, seg_len_px) is recomputed for the new
 * screen size, but wave_time + user-tunable params (amplitude,
 * frequency) are PRESERVED so animation continues from the same point
 * in its cycle rather than snapping back to t=0.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);

    float saved_wave_time = app->scene.wave_time;
    float saved_amp       = app->scene.amplitude;
    float saved_freq      = app->scene.frequency;

    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    app->scene.wave_time = saved_wave_time;
    app->scene.amplitude = saved_amp;
    app->scene.frequency = saved_freq;

    app->need_resize = 0;
}

/*
 * app_handle_key — dispatch one keypress; return false to quit.
 * Letter aliases (w/s, a/d) are provided because some terminals
 * swallow arrow-key escape sequences.
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': sc->paused = !sc->paused; break;

    case KEY_UP:   case 'w': case 'W':
        sc->frequency += 0.15f;
        if (sc->frequency > FREQ_MAX) sc->frequency = FREQ_MAX;
        break;
    case KEY_DOWN: case 's': case 'S':
        sc->frequency -= 0.15f;
        if (sc->frequency < FREQ_MIN) sc->frequency = FREQ_MIN;
        break;

    case KEY_RIGHT: case 'd': case 'D':
        sc->amplitude += 0.10f;
        if (sc->amplitude > AMP_MAX) sc->amplitude = AMP_MAX;
        break;
    case KEY_LEFT:  case 'a': case 'A':
        sc->amplitude -= 0.10f;
        if (sc->amplitude < AMP_MIN) sc->amplitude = AMP_MIN;
        break;

    case ']':
        app->time_scale *= TIME_SCALE_STEP;
        if (app->time_scale > TIME_SCALE_MAX) app->time_scale = TIME_SCALE_MAX;
        break;
    case '[':
        app->time_scale /= TIME_SCALE_STEP;
        if (app->time_scale < TIME_SCALE_MIN) app->time_scale = TIME_SCALE_MIN;
        break;

    default: break;
    }
    return true;
}

/*
 * main — variable-timestep render loop. Per-frame phases:
 *   ① INPUT      drain getch() — a press takes effect on this frame
 *   ② RESIZE     handle pending SIGWINCH before touching ncurses
 *   ③ MEASURE dt wall-clock ns since last frame; capped at 100 ms
 *                (see EDGE CASES "Suspend / lid-close")
 *   ④ TICK       one simulation step at exactly dt · time_scale
 *   ⑤ FPS        smoothed over a 500 ms window
 *   ⑥ RENDER     erase → draw → wnoutrefresh → doupdate
 *   ⑦ FRAME CAP  sleep so total frame ≈ 1/TARGET_FPS
 */
int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app        = &g_app;
    app->running    = 1;
    app->time_scale = TIME_SCALE_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    const int64_t target_ns = NS_PER_SEC / TARGET_FPS;

    int64_t last_time   = clock_ns();
    int64_t fps_accum   = 0;
    int     fps_frames  = 0;
    double  fps_display = 0.0;

    while (app->running) {

        int64_t frame_start = clock_ns();

        /* ① drain input */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }
        if (!app->running) break;

        /* ② resize */
        if (app->need_resize) {
            app_do_resize(app);
            last_time = clock_ns();
        }

        /* ③ measure dt */
        int64_t dt_ns = frame_start - last_time;
        last_time     = frame_start;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;
        float dt = (float)dt_ns / (float)NS_PER_SEC;

        /* ④ tick */
        scene_tick(&app->scene, dt * app->time_scale);

        /* ⑤ fps counter */
        fps_frames++;
        fps_accum += dt_ns;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)fps_frames
                        / ((double)fps_accum / (double)NS_PER_SEC);
            fps_frames = 0;
            fps_accum  = 0;
        }

        /* ⑥ render */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->time_scale);
        screen_present();

        /* ⑦ frame cap — sleep so total frame ≈ target_ns. The math is
         *    just (target − elapsed); no spurious +dt terms. */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(target_ns - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
