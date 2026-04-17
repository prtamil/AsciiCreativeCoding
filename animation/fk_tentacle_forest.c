/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fk_tentacle_forest.c — 8 Ocean Tentacles Swaying in an Underwater Current
 *
 * DEMO: 8 seaweed / tentacle strands rooted at the bottom of the terminal
 *       sway left and right in a simulated underwater current.  Motion is
 *       driven by pure stateless Forward Kinematics (FK): each frame the
 *       entire chain is recomputed from a closed-form sine formula — no trail
 *       buffer, no integration, no stored velocity.  Because the roots are
 *       fixed, there is nothing to remember between ticks beyond wave_time.
 *
 *       Each tentacle has its own root phase and a tiny frequency offset so
 *       all strands move independently, producing the organic asynchrony of
 *       real seaweed in a gentle current.  A seabed row of '~' glyphs
 *       completes the atmosphere.
 *
 * STUDY ALONGSIDE: framework.c               (canonical loop / timing template)
 *                  snake_forward_kinematics.c  (path-following FK, trail buffer)
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────────
 *   §1  config        — all tunables in one place
 *   §2  clock         — monotonic clock + sleep (verbatim from framework)
 *   §3  color         — deep-sea 7-step palette + dedicated HUD pair
 *   §4  coords        — pixel↔cell aspect-ratio helpers
 *   §5  entity        — Tentacle struct + stateless FK chain computation
 *       §5a  stateless FK — compute_fk_chain: angle accumulation algorithm
 *       §5b  rendering helpers — glyph selector, seg_pair, seg_attr
 *       §5c  draw_segment_dense — dense cell-fill for one segment
 *       §5d  render_tentacle — two-pass chain compositor
 *   §6  scene         — scene_init / scene_tick / scene_draw
 *   §7  screen        — ncurses double-buffer display layer
 *   §8  app           — signals, resize, main game loop
 * ─────────────────────────────────────────────────────────────────────────
 *
 * HOW STATELESS FK WORKS FOR FIXED-ROOT CHAINS
 * ─────────────────────────────────────────────
 *
 * Each tentacle has N_SEGS rigid segments joined end-to-end.  Joint[0] is
 * the root, fixed on the sea floor.  Every simulation tick ALL joint positions
 * are recomputed from scratch using this algorithm:
 *
 *   base_angle       = -π/2                 (pointing straight up)
 *   cumulative_angle = base_angle
 *   joint[0]         = root                 (fixed anchor)
 *
 *   for i in [0, N_SEGS):
 *     δθᵢ = amplitude × sin(frequency × wave_time + root_phase + i × PHASE_PER_SEG)
 *     cumulative_angle += δθᵢ
 *     joint[i+1].x = joint[i].x + SEG_LEN_PX × cos(cumulative_angle)
 *     joint[i+1].y = joint[i].y + SEG_LEN_PX × sin(cumulative_angle)
 *
 * WHY CUMULATIVE ANGLE ACCUMULATION:
 *   Each segment bends by a LOCAL angle δθᵢ relative to the previous
 *   segment's direction.  By accumulating δθᵢ into cumulative_angle, every
 *   upstream bend propagates to all downstream segments — just as real tendons
 *   transmit motion through a biological chain.  This is the core FK insight:
 *   the world-space angle of segment i is the SUM of all local bends from 0→i.
 *
 * WHY THE WAVE PROPAGATES UP THE TENTACLE:
 *   The per-segment phase offset i × PHASE_PER_SEG means lower segments (small
 *   i) lead the wave and upper segments (large i) follow.  This mimics the
 *   physics of a wave travelling through an elastic medium: the root moves
 *   first (driven by the current), and the motion propagates toward the tip
 *   with a delay proportional to position.
 *
 * WHY NO TRAIL BUFFER IS NEEDED:
 *   In snake_forward_kinematics.c the root is MOBILE, so the body must follow
 *   the path the head actually carved through space — hence the circular trail
 *   buffer.  Here the root is FIXED, so joint[0] is always at the same pixel.
 *   Computing the same formula twice with the same wave_time is algebraically
 *   identical: the chain is fully determined by wave_time alone.  There is no
 *   path history to store.
 *
 * PER-TENTACLE PHASE AND FREQUENCY OFFSET:
 *   root_phase   — shifts each tentacle's sine input by a fixed offset so
 *                  strands are at different points in their oscillation cycle.
 *                  Distributed evenly over [0, 2π) in scene_init().
 *   freq_offset  — tiny additive variation on top of the base frequency.
 *                  No two strands oscillate at exactly the same period, which
 *                  breaks the mechanical lockstep look and creates organic drift.
 *
 * SUB-TICK ALPHA INTERPOLATION:
 *   prev_joint[] is saved before each FK recompute.  The renderer lerps between
 *   prev_joint and joint using alpha ∈ [0, 1) — the fraction of the current
 *   physics tick that has elapsed.  This makes motion visually smooth at any
 *   render frame rate, even when the physics clock is much slower than the
 *   display refresh.
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   ↑ / w      frequency + 0.15 rad/s   (faster sway)
 *   ↓ / s      frequency − 0.15 rad/s   (slower sway)
 *   → / d      amplitude + 0.10 rad     (wider sway)
 *   ← / a      amplitude − 0.10 rad     (narrower sway)
 *   [ / ]      simulation Hz ±10
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       fk_tentacle_forest.c -o fk_tentacle_forest -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Stateless Forward Kinematics for fixed-root chains.
 *                  Every tick, joint positions are derived from a sine formula
 *                  that accumulates local bend angles (δθᵢ) up the chain:
 *                  cumulative_angle += amplitude × sin(ω×t + phase + i×φ).
 *                  No historical state is required because re-evaluating the
 *                  formula at the same wave_time yields identical positions.
 *                  prev_joint[] is saved solely for sub-tick alpha interpolation
 *                  at render time — not for simulation correctness.
 *
 * Data-structure : Tentacle: fixed root position (root_px, root_py), per-strand
 *                  root_phase (stagger) and freq_offset (slight detuning), plus
 *                  joint[] and prev_joint[] arrays of Vec2 pixel positions.
 *                  Scene: N_TENTACLES Tentacle entities, shared wave_time,
 *                  amplitude, frequency, and the dynamically computed seg_len_px.
 *
 * Rendering      : draw_segment_dense() walks each segment in DRAW_STEP_PX
 *                  increments, stamping direction glyphs (- | / \) at every
 *                  terminal cell crossed.  A 7-step deep-sea palette (deep
 *                  blue at root → bright yellow-green at tip) is applied with
 *                  gradient color pairs.  A seabed row of '~' glyphs is drawn
 *                  in dim deep blue at the bottom content row for atmosphere.
 *                  Two-pass render: segment lines first, joint node markers on
 *                  top (so knuckle markers always overwrite fill chars).
 *
 * Performance    : No trail buffer means O(N_TENTACLES × N_SEGS) work per
 *                  tick — 8 × 16 = 128 sin/cos evaluations at 60 Hz = ~7 680
 *                  trig calls/second, trivially fast on any modern FPU.
 *                  The fixed-step accumulator (§8) decouples physics from
 *                  render Hz.  ncurses doupdate() sends only changed cells to
 *                  the terminal fd, typically 200–400 cells per frame.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

/*
 * M_PI is a POSIX extension, not standard C99/C11.
 * _POSIX_C_SOURCE 200809L exposes it on most systems, but if a toolchain
 * omits it we define our own constant so the build never fails.
 */
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/*
 * All magic numbers live here.  Change behaviour by editing this block
 * only — never scatter literals through the code.
 */
enum {
    /*
     * SIM_FPS_DEFAULT — target physics tick rate in Hz.
     *
     * The fixed-step accumulator in §8 fires scene_tick() exactly this many
     * times per wall-clock second, regardless of how fast the render loop runs.
     * Raising sim Hz reduces the maximum per-tick motion step (dt = 1/Hz),
     * which makes the sine integration smoother at the cost of more CPU.
     *
     * [SIM_FPS_MIN, SIM_FPS_MAX] is the user-adjustable range via [ / ].
     * SIM_FPS_STEP is the per-keypress increment.
     * Default 60 Hz matches typical display refresh so one physics tick fires
     * per rendered frame — no interpolation artefact under normal conditions.
     */
    SIM_FPS_MIN     =  10,
    SIM_FPS_DEFAULT =  60,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    =  10,   /* Hz increment for [ / ] keys                 */

    /*
     * HUD_COLS — byte budget for the status bar snprintf buffer.
     * 96 bytes comfortably holds the longest possible HUD string at max values.
     * snprintf truncates safely if the string somehow exceeds this limit.
     *
     * FPS_UPDATE_MS — how often the displayed fps value is recalculated.
     * 500 ms gives a stable, readable value without flickering every frame.
     * Per-frame fps would divide by tiny elapsed times, producing wild swings.
     */
    HUD_COLS      = 96,
    FPS_UPDATE_MS = 500,   /* ms per fps averaging window                   */

    /*
     * N_PAIRS — number of gradient color pairs for the tentacle body (§3).
     *
     * Pairs 1..7 map root→tip from deep blue to bright yellow-green.
     * HUD_PAIR = 8 is reserved exclusively for both HUD bars so their color
     * never accidentally changes if N_PAIRS is adjusted.
     *
     * The 7-step gradient gives the eye a clear read of depth: deep blue at
     * the root suggests cold, dark water near the sea floor; bright green-
     * yellow at the tip suggests shallower, sunlit water above.
     */
    N_PAIRS  = 7,
    HUD_PAIR = 8,          /* dedicated HUD pair — never part of tentacle   */

    /*
     * N_TENTACLES — number of seaweed strands rooted along the bottom edge.
     *
     * 8 strands fill the screen width with a comfortable gap between roots.
     * Fewer strands look sparse; more than ~12 start overlapping on narrow
     * terminals (< 80 columns).  8 is the sweet spot for a 80–220 column range.
     *
     * N_SEGS — rigid segments per tentacle chain.
     *
     * More segments → smoother visible curvature (each segment bends only a
     * fraction of the total angle), but also more sin/cos evaluations per tick.
     * 16 segments at the dynamic segment length (see §1 DRAW_STEP_PX comment)
     * produce a curve with enough joints to look fluid without aliasing.
     *
     * With N_SEGS=16 and seg_len_px dynamically set to 55% of screen height
     * divided by 16, each segment spans about 1.5–3 terminal rows depending
     * on terminal size — fine enough for the chain to look smooth on screen.
     */
    N_TENTACLES = 8,
    N_SEGS      = 16,
};

/*
 * DRAW_STEP_PX — pixel stride between successive glyph stamps in
 * draw_segment_dense().
 *
 * The renderer walks each segment from joint[i] to joint[i+1] in increments
 * of DRAW_STEP_PX, converting each sample to a terminal cell and stamping
 * a direction glyph.  The stride controls glyph density:
 *
 *   Small stride (e.g. 3 px): nearly every cell in the segment path is
 *     filled — looks solid and wall-to-wall, like a thick rope.
 *   Large stride (e.g. 7 px): only every other cell or so is stamped —
 *     looks open with visible gaps, like a wire or strand.
 *
 * 5.0 px ≈ 0.625 cell widths (CELL_W = 8 px) — tight enough to cover
 * every cell the segment crosses (no cell ever skipped) while leaving a
 * slightly open texture that the joint-node markers (§5d pass 2) can read
 * through clearly.
 *
 * Critical constraint: DRAW_STEP_PX must be < CELL_W (8 px) so that no
 * terminal column is ever skipped by a near-horizontal segment.  With
 * DRAW_STEP_PX = 5 px, a horizontal segment advances 5/8 = 0.625 columns
 * per sample — guaranteed to visit every column it crosses.
 *
 * SEG_LEN_PX is NOT a compile-time constant here.  scene_init() computes
 * it dynamically from the terminal height so tentacle tips always reach
 * approximately mid-screen:
 *   seg_len_px = rows × CELL_H × 0.55 / N_SEGS
 *
 * Example at 40 rows: 40×16×0.55/16 = 22 px/segment.
 *   Total tentacle length = 16 × 22 = 352 px = 22 terminal rows.
 *   On a 40-row screen that puts the tip at row 18 — just above centre.
 * The 0.55 factor (55% of screen height) rather than 0.50 compensates for
 * the fact that a swaying tentacle traces a longer arc than a straight line,
 * so it visually terminates near mid-screen even at full amplitude.
 *
 * seg_len_px is stored in Scene and recalculated on every SIGWINCH resize.
 */
#define DRAW_STEP_PX   5.0f   /* px stride per glyph stamp; must be < CELL_W */

/*
 * PHASE_PER_SEG — phase advance per segment index (radians).
 *
 * In the FK formula:
 *   δθᵢ = amplitude × sin(ω×t + root_phase + i × PHASE_PER_SEG)
 *
 * This term shifts each segment's sine phase by 0.45 rad ≈ 26° relative to
 * the segment below it.  The result is that the wave appears to TRAVEL up the
 * tentacle: the root bends first, and each successive segment follows with a
 * delay of one PHASE_PER_SEG step.
 *
 * Geometric meaning:
 *   PHASE_PER_SEG = 0.0  → all segments in phase → tentacle bends rigidly,
 *                           like a solid rod tilted by the wave (no S-curve).
 *   PHASE_PER_SEG = π/2  → quarter-wave delay per segment → tight zigzag.
 *   PHASE_PER_SEG = 0.45 → about 1/14 wavelength per segment → a gentle
 *                           single S-curve up the 16-segment chain, which
 *                           is the most biologically realistic look.
 *
 * With N_SEGS = 16 segments, total phase span across the tentacle:
 *   16 × 0.45 = 7.2 rad ≈ 1.15 full oscillation cycles.
 * So the tentacle displays slightly more than one full wave from root to tip.
 */
#define PHASE_PER_SEG  0.45f   /* rad/segment; wave propagation speed up chain */

/*
 * AMP_DEFAULT — default peak bend angle per segment (radians).
 *
 * Each segment's local bend δθᵢ oscillates between −AMP and +AMP.
 * 0.28 rad ≈ 16° per segment.  With 16 segments all bending in the same
 * direction (synchronised peak), the theoretical maximum tip deflection
 * from vertical would be 16 × 0.28 = 4.48 rad ≈ 257°, which would wrap
 * the tentacle into a spiral.
 *
 * In practice the cumulative angle is spread sinusoidally across segments
 * (via PHASE_PER_SEG), so segments rarely all peak simultaneously.  The
 * typical tip deflection is roughly ±(N_SEGS/2) × AMP_DEFAULT ≈ ±2.2 rad
 * — about 130° from vertical, which looks like vigorous but non-spiralling
 * sway.  AMP_DEFAULT = 0.28 was chosen to produce realistic seaweed motion.
 *
 * AMP_MIN = 0.0 → all δθᵢ = 0 → tentacle hangs perfectly straight up.
 * AMP_MAX = 1.2 rad/segment → strong current; tentacle curls dramatically.
 *
 * FREQ_DEFAULT — base oscillation frequency (radians per second).
 *
 * The wave_time advances at 1 second per simulated second, so ω × wave_time
 * completes one full cycle when wave_time = 2π / FREQ_DEFAULT.
 *   FREQ_DEFAULT = 0.8 rad/s → period = 2π / 0.8 ≈ 7.9 seconds per sway cycle.
 * 7.9 seconds is a leisurely, meditative pace — consistent with seaweed in a
 * gentle tidal current (real seaweed: 4–12 seconds per oscillation).
 *
 * FREQ_MIN = 0.1 rad/s → period ≈ 63 s (almost static).
 * FREQ_MAX = 5.0 rad/s → period ≈ 1.3 s (rapid flickering, hurricane current).
 */
#define AMP_DEFAULT    0.28f   /* rad per segment; peak bend angle            */
#define AMP_MIN        0.0f    /* 0 = straight up, no sway                    */
#define AMP_MAX        1.2f    /* 1.2 rad/seg = fierce current, tight curls   */
#define FREQ_DEFAULT   0.8f    /* rad/s; period = 2π/0.8 ≈ 7.9 s per cycle   */
#define FREQ_MIN       0.1f    /* nearly static — very slow current           */
#define FREQ_MAX       5.0f    /* rapid zigzag — turbulent current            */

/*
 * Timing primitives — verbatim from framework.c.
 *
 * NS_PER_SEC / NS_PER_MS: unit conversion constants.
 *   1 second = 1 000 000 000 nanoseconds; 1 ms = 1 000 000 ns.
 *
 * TICK_NS(f): converts frequency f (Hz) to the period in nanoseconds.
 *   e.g. TICK_NS(60) = 1 000 000 000 / 60 ≈ 16 666 667 ns per physics tick.
 *   Used by the fixed-step accumulator in §8 to fire scene_tick() at exactly
 *   sim_fps Hz regardless of render frame rate.
 */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Terminal cell dimensions — the aspect-ratio bridge between physics
 * and display (see §4 for a full explanation of why two spaces are needed).
 *
 * CELL_W = 8 px, CELL_H = 16 px.
 * A typical terminal character cell is roughly twice as tall as it is wide
 * in physical pixels.  All Vec2 positions are stored in a square pixel space
 * where 1 unit represents the same physical distance horizontally and
 * vertically.  Only at draw time are pixel coordinates converted to cell
 * coordinates via the helpers in §4.
 *
 * These values match the most common default font metrics (e.g. an 8×16
 * bitmap terminal font).  They are intentionally not queried at runtime:
 * the exact cell dimensions do not need to be pixel-perfect, only in the
 * right ratio.  2:1 height-to-width is accurate for virtually all terminals.
 */
#define CELL_W   8    /* physical pixels per terminal column                 */
#define CELL_H  16    /* physical pixels per terminal row                    */

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/*
 * clock_ns() — monotonic wall-clock in nanoseconds.
 *
 * CLOCK_MONOTONIC is guaranteed never to go backward, unlike CLOCK_REALTIME
 * which can jump on NTP corrections or DST transitions.  Subtracting two
 * consecutive clock_ns() calls gives the true elapsed wall time regardless
 * of system load, suspend/resume events, or clock adjustments.
 *
 * Returns int64_t (signed 64-bit) so that dt differences are naturally signed.
 * Negative values can arise from the 100 ms pause-guard cap (§8 step ②),
 * and signed arithmetic handles that correctly without unsigned wrap-around.
 *
 * Range: int64_t overflows at ~292 years of continuous uptime — not a
 * practical concern.
 */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/*
 * clock_sleep_ns() — sleep for exactly ns nanoseconds.
 *
 * Called BEFORE the render step in §8 to cap the frame rate at 60 fps.
 * Sleeping before (not after) terminal I/O means only the physics computation
 * time is charged against the frame budget; the terminal write cost of the
 * previous frame is not included.
 *
 * Why sleep before render rather than after?
 *   If we slept after render, the time spent writing to the terminal would
 *   delay the next frame's physics, causing a timing drift that accumulates.
 *   Sleeping before render keeps the physics budget clean: elapsed = physics
 *   time only, and the render overlaps with the next sleep window.
 *
 * If ns ≤ 0, the frame is already over-budget (physics took longer than the
 * 60-fps window); return immediately rather than sleeping a negative duration.
 */
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
/* §3  color — deep-sea 7-step palette + dedicated HUD pair              */
/* ===================================================================== */

/*
 * color_init() — define all ncurses color pairs used in the animation.
 *
 * TENTACLE GRADIENT (pairs 1–7): deep blue root → bright yellow-green tip.
 *
 *   The gradient is designed to suggest water depth.  Deep blue (pair 1) at
 *   the root evokes cold, dark water near the sea floor.  As the chain rises
 *   toward the tip, colors shift through cyan and green toward yellow-green,
 *   suggesting shallower, sunlit water.  This color metaphor is immediately
 *   readable: the bright tips draw the eye and communicate "this end floats".
 *
 *   Pair  256-col  Approx name           Role
 *   ──────────────────────────────────────────────────────────
 *     1      21    deep blue             root (sea floor, cold)
 *     2      27    medium blue
 *     3      33    cyan-blue
 *     4      51    bright cyan           mid-body
 *     5      86    cyan-green
 *     6     118    yellow-green
 *     7     154    bright yellow-green   tip (shallowest, warmest)
 *     8     226    bright yellow         HUD bars only (not a tentacle color)
 *
 * 8-COLOR FALLBACK:
 *   On terminals with fewer than 256 colors, the closest basic colors are
 *   substituted.  The gradient is coarser but still readable.
 *
 * use_default_colors() allows -1 as a background color index, which means
 * "use the terminal's default background" rather than forcing COLOR_BLACK.
 * This is important for terminals with non-black default backgrounds.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(1,  21, COLOR_BLACK);   /* deep blue — root, sea floor     */
        init_pair(2,  27, COLOR_BLACK);   /* medium blue                     */
        init_pair(3,  33, COLOR_BLACK);   /* cyan-blue                       */
        init_pair(4,  51, COLOR_BLACK);   /* bright cyan — mid-body          */
        init_pair(5,  86, COLOR_BLACK);   /* cyan-green                      */
        init_pair(6, 118, COLOR_BLACK);   /* yellow-green                    */
        init_pair(7, 154, COLOR_BLACK);   /* bright yellow-green — tip       */
        init_pair(8, 226, COLOR_BLACK);   /* bright yellow — HUD bars only   */
    } else {
        /* 8-color fallback: coarser gradient, still directionally correct   */
        init_pair(1, COLOR_BLUE,   COLOR_BLACK);
        init_pair(2, COLOR_BLUE,   COLOR_BLACK);
        init_pair(3, COLOR_CYAN,   COLOR_BLACK);
        init_pair(4, COLOR_CYAN,   COLOR_BLACK);
        init_pair(5, COLOR_CYAN,   COLOR_BLACK);
        init_pair(6, COLOR_GREEN,  COLOR_BLACK);
        init_pair(7, COLOR_GREEN,  COLOR_BLACK);
        init_pair(8, COLOR_YELLOW, COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell; the one aspect-ratio fix                     */
/* ===================================================================== */

/*
 * WHY TWO COORDINATE SPACES
 * ─────────────────────────
 * Terminal cells are not square.  A typical cell is ~2× taller than wide in
 * physical pixels (CELL_H=16 vs CELL_W=8).  If tentacle joint positions were
 * stored directly in cell (row, column) coordinates, moving 1 unit in the
 * y-direction would cover twice as much physical distance as 1 unit in x.
 * The tentacles would appear geometrically distorted: a 45° angle in physics
 * space would look like a 63° angle on screen (atan(16/8) = 63°).
 *
 * THE FIX — physics in pixel space, drawing in cell space:
 *   All Vec2 positions (joint[], prev_joint[]) are in PIXEL space where
 *   1 unit = 1 physical pixel.  The grid is square and isotropic: moving
 *   1 unit in x or y covers the same physical distance.
 *   Only at draw time (§5c, §5d) are positions converted to cell coordinates.
 *
 * px_to_cell_x / px_to_cell_y — round pixel coordinate to nearest cell index.
 *
 * Formula:  cell = floor(px / CELL_DIM + 0.5)
 *   Adding 0.5 before flooring implements "round half up" — deterministic and
 *   symmetric.  Two alternatives and why they are worse:
 *   • roundf(): uses "round half to even" (banker's rounding), which can flip
 *     between adjacent cells when px lands exactly on a boundary, causing
 *     one-frame flicker.
 *   • (int)(px / CELL_W): plain truncation, always rounds toward zero —
 *     asymmetric, gives the lower cell an extra pixel of dwell at boundaries.
 *   floor + 0.5 avoids both artefacts and costs the same as the alternatives.
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
/* §5  entity — Tentacle struct + stateless FK chain                     */
/* ===================================================================== */

/*
 * Vec2 — lightweight 2-D position vector in pixel space.
 * x increases eastward (+right); y increases downward (+down).
 * This matches terminal screen convention: row 0 is at the top,
 * and cell (row, col) = (y/CELL_H, x/CELL_W).
 */
typedef struct { float x, y; } Vec2;

/*
 * Tentacle — complete per-strand state for one seaweed tentacle.
 *
 * ROOT POSITION (fixed for the lifetime of the simulation):
 *   root_px, root_py  Pixel-space coordinates of joint[0] (the sea floor
 *                     anchor).  Set once in scene_init() and never changed.
 *                     Tentacle roots are evenly spaced along the bottom of
 *                     the screen (see scene_init() for the spacing formula).
 *
 * PER-TENTACLE WAVE PARAMETERS (set once, never changed at runtime):
 *   root_phase   Phase offset in radians added to the global sine argument.
 *                Each tentacle receives a different root_phase ∈ [0, 2π)
 *                so strands are at different points in their oscillation at
 *                all times.  Without this, all 8 strands would sway perfectly
 *                in lockstep, looking mechanical rather than organic.
 *
 *   freq_offset  Tiny additive variation on the global frequency (rad/s).
 *                Computed as (i − N_TENTACLES/2) × 0.04 in scene_init(),
 *                giving values in the range [−0.16, +0.16] rad/s for 8 strands.
 *                No two strands oscillate at exactly the same period, which
 *                prevents long-term phase re-synchronisation and makes the
 *                forest look perpetually alive.
 *
 * JOINT POSITION ARRAYS:
 *   joint[0]         Root — always equals (root_px, root_py).
 *   joint[1..N_SEGS] Body and tip — recomputed from scratch every tick by
 *                    compute_fk_chain().
 *   joint[N_SEGS]    Tip — the free end farthest from the sea floor.
 *
 *   prev_joint[]     Snapshot of joint[] from the END of the previous tick.
 *                    Saved by compute_fk_chain() before overwriting joint[].
 *                    render_tentacle() lerps between prev_joint and joint
 *                    using alpha ∈ [0, 1) to produce sub-tick smooth motion.
 *                    Without prev_joint, the tentacle would jump discretely
 *                    at each physics tick boundary, visibly stuttering when
 *                    sim Hz is lower than render Hz.
 */
typedef struct {
    float root_px, root_py;              /* fixed sea-floor anchor (px)      */
    float root_phase;                    /* per-strand phase stagger (rad)   */
    float freq_offset;                   /* per-strand detuning (rad/s)      */
    Vec2  joint[N_SEGS + 1];            /* [0]=root … [N_SEGS]=tip          */
    Vec2  prev_joint[N_SEGS + 1];       /* joint[] at start of last tick    */
} Tentacle;

/* ── §5a  stateless FK computation ─────────────────────────────────── */

/*
 * compute_fk_chain() — recompute all joint positions for one tentacle.
 *
 * This is the heart of the simulation.  It is called every tick for every
 * tentacle.  All parameters come from the Scene (wave_time, amplitude,
 * frequency) or the Tentacle itself (root_phase, freq_offset).
 *
 * ALGORITHM (step by step):
 *
 *   Step 1 — Save joint[] into prev_joint[].
 *     Before any joint positions change, copy the current array to prev_joint.
 *     This is the interpolation anchor for render_tentacle(): it holds the
 *     positions from the end of the PREVIOUS tick, so lerping from prev to
 *     current sweeps through the correct intermediate positions.
 *
 *   Step 2 — Anchor joint[0] at the fixed root.
 *     The root never moves.  Although the memcpy above correctly copies
 *     joint[0] to prev_joint[0], we set joint[0] explicitly to ensure
 *     floating-point drift from any future changes never accumulates here.
 *
 *   Step 3 — Initialise cumulative_angle = -π/2.
 *     -π/2 points straight up in terminal pixel space (y increases downward,
 *     so "upward" is the negative-y direction, which corresponds to angle
 *     -π/2 in the atan2 / cos-sin convention).
 *     If cumulative_angle started at 0.0, the tentacle would initially point
 *     rightward (east) — sideways, not upward.
 *
 *   Step 4 — FK loop: for each segment i in [0, N_SEGS):
 *
 *     4a. Compute the local bend angle for this segment:
 *           δθᵢ = amplitude × sin((frequency + freq_offset) × wave_time
 *                                  + root_phase + i × PHASE_PER_SEG)
 *
 *         Why amplitude modulates the sine and not the cos?  The sine is
 *         dimensionless on [−1, +1]; multiplying by amplitude (radians)
 *         gives a bend angle in radians.  The tentacle bends most when
 *         sin(…) = ±1, i.e., at the wave peaks and troughs.
 *
 *         The argument (frequency + freq_offset) × wave_time advances the
 *         oscillation phase at a slightly different rate per strand (via
 *         freq_offset), preventing long-term lockstep re-synchronisation.
 *
 *         root_phase shifts the entire tentacle's cycle by a fixed amount,
 *         ensuring strands start at different positions in their oscillation.
 *
 *         i × PHASE_PER_SEG advances the phase by an additional 0.45 rad per
 *         segment, making the wave propagate upward through the chain.
 *
 *     4b. Accumulate: cumulative_angle += δθᵢ.
 *         This is the FK insight: the world-space direction of segment i is
 *         the SUM of all local bends from 0 through i.  Each call to +=
 *         applies one more local bend on top of all the previous ones.
 *
 *     4c. Compute joint[i+1] by extending from joint[i] along cumulative_angle:
 *           joint[i+1].x = joint[i].x + seg_len_px × cos(cumulative_angle)
 *           joint[i+1].y = joint[i].y + seg_len_px × sin(cumulative_angle)
 *         cos gives the eastward (x) component; sin gives the downward (y)
 *         component.  With cumulative_angle ≈ -π/2 (nearly up), cos ≈ 0 and
 *         sin ≈ -1, so the joint moves mostly upward (negative y), as expected.
 *
 * NUMERICAL PROPERTIES:
 *   No integration or running sum accumulates floating-point error over time
 *   because every tick starts fresh from wave_time (a monotonically growing
 *   float).  The only approximation is single-precision sinf/cosf, which has
 *   an error of about 1 ULP — irrelevant for screen-resolution animation.
 *
 * CALLING CONVENTION:
 *   Called by scene_tick() for every tentacle.  scene_tick() also decides
 *   whether to advance wave_time (skipped when paused).  compute_fk_chain()
 *   itself does not check the paused flag — it always runs; if wave_time is
 *   unchanged, the output is identical to the previous tick (no visual change).
 *   The prev_joint save still happens, which is correct: the lerp becomes a
 *   no-op (prev == current) and no stutter occurs.
 */
static void compute_fk_chain(Tentacle *t,
                              float wave_time,
                              float amplitude,
                              float frequency,
                              float seg_len_px)
{
    /* Step 1: save current positions as previous before overwriting */
    memcpy(t->prev_joint, t->joint, sizeof t->joint);

    /* Step 2: anchor the root — never moves */
    t->joint[0].x = t->root_px;
    t->joint[0].y = t->root_py;

    /* Step 3: start pointing straight up (-π/2 in pixel-space convention) */
    float cumulative_angle = -(float)M_PI * 0.5f;

    /* Step 4: FK loop — extend the chain one segment at a time */
    for (int i = 0; i < N_SEGS; i++) {
        /*
         * 4a — local bend angle for segment i.
         * The full sine argument:
         *   (frequency + freq_offset) × wave_time   wave phase at this moment
         *   + root_phase                             per-strand stagger
         *   + i × PHASE_PER_SEG                     upward wave propagation
         */
        float delta = amplitude
                    * sinf((frequency + t->freq_offset) * wave_time
                           + t->root_phase
                           + (float)i * PHASE_PER_SEG);

        /* 4b — accumulate: world-space angle = sum of all upstream bends */
        cumulative_angle += delta;

        /* 4c — extend joint[i+1] along the accumulated direction */
        t->joint[i + 1].x = t->joint[i].x
                           + seg_len_px * cosf(cumulative_angle);
        t->joint[i + 1].y = t->joint[i].y
                           + seg_len_px * sinf(cumulative_angle);
    }
}

/* ── §5b  rendering helpers ─────────────────────────────────────────── */

/*
 * seg_pair() — ncurses color pair index for segment i.
 *
 * Maps segment index i linearly from root (i=0 → pair 1, deep blue) to
 * tip (i=N_SEGS-1 → pair N_PAIRS=7, bright yellow-green):
 *
 *   pair = 1 + (i × (N_PAIRS − 1)) / (N_SEGS − 1)
 *
 * Integer arithmetic with N_SEGS=16, N_PAIRS=7:
 *   i= 0 → 1 + (0×6)/15 = 1   (deep blue — root)
 *   i= 3 → 1 + (3×6)/15 = 2   (medium blue)
 *   i= 5 → 1 + (5×6)/15 = 3   (cyan-blue)
 *   i= 7 → 1 + (7×6)/15 = 3   (still cyan-blue, mid-body)
 *   i=10 → 1 + (10×6)/15= 5   (cyan-green)
 *   i=12 → 1 + (12×6)/15= 5   (cyan-green, upper body)
 *   i=15 → 1 + (15×6)/15= 7   (bright yellow-green — tip)
 *
 * The guard (N_SEGS <= 1) prevents division by zero for pathological configs.
 */
static int seg_pair(int i)
{
    if (N_SEGS <= 1) return 1;
    return 1 + (i * (N_PAIRS - 1)) / (N_SEGS - 1);
}

/*
 * seg_attr() — ncurses rendering attribute for segment i.
 *
 * Varies brightness by chain position to reinforce the depth metaphor:
 *
 *   Root quarter   (i < N_SEGS/4 = 4)        → A_DIM
 *     Deep segments are dim: in deep water, colours are absorbed by the water
 *     column above.  DIM also visually "anchors" the root to the sea floor.
 *
 *   Tip quarter    (i > 3*N_SEGS/4 = 12)     → A_BOLD
 *     The uppermost segments catch the most light and movement — bright,
 *     vibrant tips draw the eye to the free end of the tentacle.
 *
 *   Mid-body       (4 ≤ i ≤ 12)              → A_NORMAL
 *     Neutral brightness in the middle provides a smooth visual transition.
 *
 * Combined with the color gradient from §5b seg_pair(), these attributes
 * create a convincing impression of a 3-D strand floating in water.
 */
static attr_t seg_attr(int i)
{
    if (i < N_SEGS / 4)       return A_DIM;    /* deep, dark root section   */
    if (i > 3 * N_SEGS / 4)   return A_BOLD;   /* bright, light tip section */
    return A_NORMAL;                             /* mid-body: neutral         */
}

/*
 * seg_glyph() — ASCII character that best represents a segment's direction.
 *
 * Maps the direction vector (dx, dy) to one of four line characters that
 * most closely match the segment's visual orientation on screen.
 *
 * The four ASCII glyphs and their angle ranges (folded to [0°, 180°)):
 *   '-'  horizontal:   [  0°, 22.5°) ∪ [157.5°, 180°)
 *   '\'  diagonal ↘:   [ 22.5°,  67.5°)
 *   '|'  vertical:     [ 67.5°, 112.5°)
 *   '/'  diagonal ↗:   [112.5°, 157.5°)
 *
 * FOLD TO [0°, 180°):
 *   A line segment from A to B looks the same as one from B to A — a '/'
 *   slanting up-left or down-right is the same glyph.  Folding the atan2
 *   result to [0°, 180°) handles both directions with the same lookup.
 *
 * dy NEGATION:
 *   atan2f uses the mathematical convention where +y is upward.  In terminal
 *   pixel space, +y is downward.  Negating dy before atan2f converts from
 *   terminal convention to math convention so that a segment pointing straight
 *   up (dy < 0 in pixels) maps to 90° (vertical → '|') as expected.
 *
 * Why not just compare angle to pi/4 boundaries?
 *   atan2f naturally returns the full [−π, +π] range; converting to degrees
 *   and folding makes the 45° boundary conditions easy to read and audit.
 */
static chtype seg_glyph(float dx, float dy)
{
    float ang = atan2f(-dy, dx);              /* convert to math convention  */
    float deg = ang * (180.0f / (float)M_PI);
    if (deg <    0.0f) deg += 360.0f;         /* normalise to [0°, 360°)     */
    if (deg >= 180.0f) deg -= 180.0f;         /* fold to [0°, 180°)          */

    if (deg < 22.5f || deg >= 157.5f)  return (chtype)'-';  /* horizontal    */
    if (deg < 67.5f)                    return (chtype)'\\'; /* diagonal ↘    */
    if (deg < 112.5f)                   return (chtype)'|';  /* vertical      */
    return                              (chtype)'/';          /* diagonal ↗    */
}

/* ── §5c  draw_segment_dense ────────────────────────────────────────── */

/*
 * draw_segment_dense() — stamp a direction glyph at every terminal cell
 * that the line segment from pixel point a to pixel point b passes through.
 *
 * HOW IT WORKS:
 *   Walk from a to b in DRAW_STEP_PX increments (parametric lerp u ∈ [0, 1]).
 *   At each step, convert the pixel position to a cell coordinate (cx, cy).
 *   If the cell has changed since the previous stamp (dedup check), and the
 *   cell is within the drawable region, stamp the direction glyph there.
 *
 * DEDUPLICATION (prev_cx, prev_cy):
 *   Two adjacent sample points often land in the same terminal cell (because
 *   DRAW_STEP_PX = 5 px is smaller than CELL_H = 16 px for near-vertical
 *   segments).  Stamping the same cell twice is wasteful but correct — except
 *   that the second stamp uses wattroff/wattron, which could momentarily turn
 *   off attributes mid-render.  The dedup check avoids all such artefacts.
 *
 *   prev_cx and prev_cy are shared across all segments of one tentacle (passed
 *   via pointer from render_tentacle).  This prevents the segment-endpoint
 *   cells from being double-stamped when two consecutive segments share a
 *   boundary cell — which would happen if each segment used its own local
 *   prev_cx = -9999.
 *
 * CLIPPING:
 *   Cells with cx < 0 or cx >= cols are outside the terminal width.
 *   Cells with cy < 1 or cy >= rows-1 are in the HUD bar rows (row 0 = top
 *   HUD, row rows-1 = bottom hint bar).  Both ranges are silently skipped.
 *   This handles toroidal body wrap and HUD protection with no special code.
 *
 * WHY +1 STEPS:
 *   nsteps = ceil(len / DRAW_STEP_PX) + 1 ensures we always stamp the
 *   endpoint b (at u = nsteps/nsteps = 1.0), even when len is an exact
 *   multiple of DRAW_STEP_PX.  Without the +1, the last cell of the segment
 *   could be skipped, leaving a gap visible at segment endpoints.
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
    if (len < 0.1f) return;   /* degenerate zero-length segment — skip       */

    chtype glyph  = seg_glyph(dx, dy);
    int    nsteps = (int)ceilf(len / DRAW_STEP_PX) + 1;

    for (int s = 0; s <= nsteps; s++) {
        float u  = (float)s / (float)nsteps;      /* parametric [0, 1]      */
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        /* Dedup: skip if this cell was already stamped by the previous step */
        if (cx == *prev_cx && cy == *prev_cy) continue;
        *prev_cx = cx;
        *prev_cy = cy;

        /* Clip: skip cells outside drawable area (HUD rows excluded) */
        if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1) continue;

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, glyph);
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/* ── §5d  render_tentacle ───────────────────────────────────────────── */

/*
 * render_tentacle() — draw one tentacle chain into WINDOW *w.
 *
 * Two-pass render for an organic "beaded chain" look:
 *
 * STEP 1 — sub-tick alpha interpolation.
 *   rj[i] = lerp(prev_joint[i], joint[i], alpha) for all joints.
 *   alpha ∈ [0, 1) is the fraction of the current physics tick that has
 *   elapsed.  Using interpolated positions rather than raw joint[] produces
 *   visually smooth motion at any render frame rate, even when sim Hz is much
 *   lower than display refresh (e.g., 30 Hz sim on a 144 Hz monitor).
 *
 * STEP 2 (pass 1) — segment direction glyphs, root → tip.
 *   draw_segment_dense() walks each segment in DRAW_STEP_PX increments and
 *   stamps '|', '/', '\', or '-' using the gradient pair for that segment.
 *   prev_cx / prev_cy are shared across all N_SEGS segments so segment-
 *   boundary cells are not double-stamped.
 *   Drawing root → tip means the upper segments overwrite lower segments
 *   where the chain curves back on itself (tight curls at high amplitude).
 *
 * STEP 3 (pass 2) — joint node markers, drawn last (on top of pass 1).
 *   A distinct marker character is stamped at each interpolated joint position,
 *   overwriting whatever direction glyph pass 1 placed there.  This gives the
 *   tentacle its characteristic "knuckled" appearance — the joints are visually
 *   distinct from the segment fill:
 *
 *     i = 0              '#'  root anchor — thick, grounded character
 *     1 ≤ i ≤ N_SEGS/4   'O'  lower body — wide, fat node
 *     N_SEGS/4 < i ≤ 3N_SEGS/4  'o'  mid body — medium node
 *     3N_SEGS/4 < i < N_SEGS     '.'  upper body — small, wispy
 *     i = N_SEGS         '*'  tip — bright spark, the free end
 *
 *   All markers are drawn with A_BOLD so they dominate the segment characters
 *   below them.  The varying glyph size (# O o . *) reinforces the size
 *   gradient from thick root to thin tip — matching the sea creature look.
 *
 * WHY PASS 2 IS SEPARATE:
 *   If joint markers were stamped inside the pass 1 loop, segment lines drawn
 *   afterward (for the next segment) would overwrite the markers at shared
 *   boundary cells.  A separate pass 2 ensures all markers win.
 */
static void render_tentacle(const Tentacle *t, WINDOW *w,
                             int cols, int rows, float alpha)
{
    /* Step 1 — build interpolated render positions */
    Vec2 rj[N_SEGS + 1];
    for (int i = 0; i <= N_SEGS; i++) {
        rj[i].x = t->prev_joint[i].x
                + (t->joint[i].x - t->prev_joint[i].x) * alpha;
        rj[i].y = t->prev_joint[i].y
                + (t->joint[i].y - t->prev_joint[i].y) * alpha;
    }

    /* Step 2 (pass 1) — segment direction glyphs, root → tip */
    int prev_cx = -9999, prev_cy = -9999;   /* dedup cursor, shared across segs */
    for (int i = 0; i < N_SEGS; i++) {
        draw_segment_dense(w,
                           rj[i], rj[i + 1],
                           seg_pair(i), seg_attr(i),
                           cols, rows,
                           &prev_cx, &prev_cy);
    }

    /* Step 3 (pass 2) — joint node markers: drawn last, always on top */
    for (int i = 0; i <= N_SEGS; i++) {
        int cx = px_to_cell_x(rj[i].x);
        int cy = px_to_cell_y(rj[i].y);
        if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1) continue;

        /* Color pair: use the segment below this joint for natural gradient */
        int    p    = seg_pair(i < N_SEGS ? i : N_SEGS - 1);
        attr_t attr = A_BOLD;   /* always bold — markers must dominate lines */

        chtype marker;
        if      (i == 0)                 marker = (chtype)'#'; /* root anchor: thick  */
        else if (i <= N_SEGS / 4)        marker = (chtype)'O'; /* lower body: fat     */
        else if (i <= 3 * N_SEGS / 4)   marker = (chtype)'o'; /* mid body: medium    */
        else if (i < N_SEGS)             marker = (chtype)'.'; /* upper body: wispy   */
        else                             marker = (chtype)'*'; /* tip spark: free end */

        wattron(w, COLOR_PAIR(p) | attr);
        mvwaddch(w, cy, cx, marker);
        wattroff(w, COLOR_PAIR(p) | attr);
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene — complete simulation state for the tentacle forest.
 *
 * t[]          Array of N_TENTACLES Tentacle entities.  Each has a fixed root,
 *              a unique root_phase, a unique freq_offset, and its own joint[]/
 *              prev_joint[] arrays.  All entities share wave_time, amplitude,
 *              and frequency from this Scene struct.
 *
 * wave_time    Accumulated simulation time in seconds.  The sole clock input
 *              to compute_fk_chain().  Advances by dt each tick when not
 *              paused.  Being a single monotonically increasing float, it
 *              drives all tentacles coherently — they all see the same moment
 *              in their oscillation cycle (differentiated only by root_phase
 *              and freq_offset).
 *
 * amplitude    Global peak bend angle per segment (radians) — the sway strength.
 *              Applies to every tentacle.  User-adjustable via → / ← (or d/a).
 *              Range [AMP_MIN, AMP_MAX]; see §1 for geometric meaning.
 *
 * frequency    Base oscillation frequency (rad/s) — the sway speed.
 *              Applies to every tentacle as the base; each adds freq_offset.
 *              User-adjustable via ↑ / ↓ (or w/s).
 *              Range [FREQ_MIN, FREQ_MAX]; see §1 for period calculation.
 *
 * seg_len_px   Pixel length of each rigid segment.  NOT a compile-time constant.
 *              Computed by scene_init() from terminal dimensions:
 *                seg_len_px = rows × CELL_H × 0.55 / N_SEGS
 *              Recalculated on every SIGWINCH resize in app_do_resize() so that
 *              tentacle tips always reach approximately mid-screen regardless
 *              of terminal height.
 *
 * paused       When true, scene_tick() skips the wave_time advance.  The FK
 *              recompute still runs (because compute_fk_chain() does not check
 *              paused), but since wave_time is unchanged, joint[] is identical
 *              to the previous tick and the lerp is a clean freeze.
 */
typedef struct {
    Tentacle t[N_TENTACLES];   /* all tentacle entities                      */
    float    wave_time;        /* monotonic simulation clock (s)             */
    float    amplitude;        /* peak bend per segment (rad)                */
    float    frequency;        /* base oscillation rate (rad/s)              */
    float    seg_len_px;       /* rigid segment length (px); dynamic         */
    bool     paused;           /* when true, wave_time is frozen             */
} Scene;

/*
 * scene_init() — distribute N_TENTACLES roots evenly across the sea floor
 * and place each tentacle in a straight-up (zero-sway) starting posture.
 *
 * ROOT HORIZONTAL SPACING:
 *   root_px[i] = (i + 1) × screen_width_px / (N_TENTACLES + 1)
 *
 *   Dividing by (N_TENTACLES + 1) rather than N_TENTACLES ensures that no
 *   root falls exactly at the left or right edge of the screen.  The fractions
 *   1/9, 2/9, … 8/9 (for N_TENTACLES = 8) give equal margins on both sides
 *   and equal gaps between adjacent roots.
 *
 * ROOT VERTICAL POSITION:
 *   root_py = rows × CELL_H − 4.0 px
 *
 *   Placed 4 pixels above the absolute bottom pixel row.  The seabed '~' row
 *   is drawn at terminal row (rows − 2) which spans y pixels from
 *   (rows−2)×CELL_H to (rows−1)×CELL_H − 1.  The 4 px offset places the
 *   root visually inside the seabed row, so each tentacle appears to grow
 *   directly out of the seabed texture.
 *
 * ROOT PHASE SPACING:
 *   root_phase[i] = i × 2π / N_TENTACLES
 *
 *   Evenly distributes N_TENTACLES phase values around the full [0, 2π)
 *   circle, maximising the desynchronisation between strands from the very
 *   first frame.  With 8 strands: phases are 0, π/4, π/2, 3π/4, π, 5π/4,
 *   3π/2, 7π/4 — at any moment, strands are spread across all 8 evenly
 *   separated positions in their oscillation cycle.
 *
 * FREQUENCY OFFSET:
 *   freq_offset[i] = (i − N_TENTACLES × 0.5) × 0.04 rad/s
 *
 *   For 8 strands this gives offsets: −0.16, −0.12, −0.08, −0.04,
 *   0.00, +0.04, +0.08, +0.12 rad/s.  The centre two strands oscillate
 *   at FREQ_DEFAULT (net offset ≈ 0); edge strands oscillate slightly
 *   faster or slower.  The magnitude 0.04 was chosen so that after one
 *   full oscillation period (≈7.9 s), adjacent strands drift apart by
 *   0.04 × 7.9 ≈ 0.32 rad ≈ 18° — enough to prevent visible re-sync.
 *
 * INITIAL JOINT POSITIONS (straight up):
 *   joint[k].x = root_px   (all joints directly above the root)
 *   joint[k].y = root_py − k × seg_len_px
 *
 *   This is the tentacle's rest posture when wave_time = 0 and amplitude = 0.
 *   It provides a sensible starting frame; on the first tick wave_time advances
 *   and the FK formula immediately starts bending the chain.
 *   prev_joint is set equal to joint so the alpha lerp on frame 0 is a no-op.
 *
 * SEG_LEN_PX CALCULATION:
 *   seg_len_px = rows × CELL_H × 0.55 / N_SEGS
 *
 *   0.55 targets 55% of screen height as the straight-up tentacle length.
 *   The extra 5% above 50% accounts for the fact that a swaying tentacle
 *   traces a longer arc than a straight vertical line; with full sway the
 *   tip visually terminates near mid-screen despite the arc being longer.
 *   With rows = 40: seg_len_px = 640 × 0.55 / 16 = 22 px/segment.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    sc->amplitude  = AMP_DEFAULT;
    sc->frequency  = FREQ_DEFAULT;
    sc->wave_time  = 0.0f;
    sc->paused     = false;

    /* Dynamic segment length: 55% of pixel height divided evenly across segs */
    sc->seg_len_px = (float)(rows * CELL_H) * 0.55f / (float)N_SEGS;

    float screen_wpx = (float)(cols * CELL_W);
    float root_py    = (float)(rows * CELL_H) - 4.0f; /* 4 px above bottom */

    for (int i = 0; i < N_TENTACLES; i++) {
        Tentacle *t = &sc->t[i];

        /* Evenly spaced across screen width: fractions 1/9, 2/9, … 8/9     */
        t->root_px = (float)(i + 1) * screen_wpx / (float)(N_TENTACLES + 1);
        t->root_py = root_py;

        /* Phase evenly over [0, 2π) for maximum initial desynchronisation   */
        t->root_phase  = (float)i * 2.0f * (float)M_PI / (float)N_TENTACLES;

        /* Tiny per-tentacle frequency variation centred on FREQ_DEFAULT      */
        t->freq_offset = ((float)i - (float)N_TENTACLES * 0.5f) * 0.04f;

        /* Seed all joints straight up from root (rest posture)               */
        for (int k = 0; k <= N_SEGS; k++) {
            t->joint[k].x = t->root_px;
            t->joint[k].y = t->root_py - (float)k * sc->seg_len_px;
        }

        /* prev_joint = joint so the first-frame lerp is a no-op             */
        memcpy(t->prev_joint, t->joint, sizeof t->joint);
    }
}

/*
 * scene_tick() — one fixed-step physics update; called by §8 accumulator.
 *
 * dt — fixed tick duration in seconds (= 1.0 / sim_fps).
 *      At 60 Hz: dt = 1/60 ≈ 0.0167 s.  The accumulator guarantees this
 *      is called exactly sim_fps times per wall-clock second on average.
 *
 * cols, rows — current terminal dimensions.  The tentacle roots are fixed;
 *      no boundary clamping is needed here (unlike the swimming snake which
 *      wraps around the screen).  The parameters are accepted to match the
 *      calling convention used across all scene_tick() implementations in
 *      this codebase (framework.c §6 pattern).
 *
 * ORDER IS IMPORTANT:
 *   1. Advance wave_time (if not paused).
 *      wave_time is the sole input that changes between ticks.  Advancing it
 *      BEFORE calling compute_fk_chain() ensures the FK uses this tick's
 *      time — not last tick's.
 *
 *   2. For each tentacle, call compute_fk_chain().
 *      compute_fk_chain() saves prev_joint internally before recomputing
 *      joint[], so no external memcpy is needed here.  All tentacles are
 *      updated with the same wave_time, so they are always temporally
 *      consistent (no one tentacle is "one tick ahead" of another).
 *
 * NOTE ON PAUSED BEHAVIOUR:
 *   When paused, wave_time is not advanced, but compute_fk_chain() still runs.
 *   Since wave_time is unchanged, the FK outputs the same positions as the
 *   previous tick, making joint[] == prev_joint[] after the internal save.
 *   The alpha lerp in render_tentacle() then produces exactly the previous
 *   frame's positions — a clean, stutter-free freeze.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    (void)cols; (void)rows;   /* roots are fixed; no boundary check needed   */

    /* Step 1: advance simulation clock (skipped when paused)                 */
    if (!sc->paused)
        sc->wave_time += dt;

    /* Step 2: recompute every tentacle's FK chain for the new wave_time      */
    for (int i = 0; i < N_TENTACLES; i++) {
        compute_fk_chain(&sc->t[i],
                         sc->wave_time,
                         sc->amplitude,
                         sc->frequency,
                         sc->seg_len_px);
    }
}

/*
 * scene_draw() — render all tentacles plus the seabed atmosphere row.
 *
 * This function is PURE READ: it reads scene state (joint[], prev_joint[],
 * paused) but mutates NOTHING.  All state changes happen in scene_tick().
 * The clean separation between tick (mutate) and draw (read) makes it safe
 * to call scene_draw() multiple times per tick (for debugging overlays) or
 * to skip it entirely (for headless benchmarks) without corrupting physics.
 *
 * alpha ∈ [0, 1) — sub-tick interpolation factor from §8 step ④.
 *   alpha = 0.0: render exactly at the last tick's joint positions.
 *   alpha = 0.9: render 90% of the way toward the next tick's positions.
 *   Passed through to render_tentacle() for each strand.
 *
 * SEABED ROW:
 *   The last content row (rows − 2, since rows − 1 is the hint bar) is
 *   filled with '~' characters in dim deep blue (pair 1, A_DIM).
 *   '~' is the traditional ASCII wave glyph, evoking a sandy or rocky sea
 *   floor from which the tentacles grow.  A_DIM keeps it visually receding
 *   behind the bright tentacle roots without disappearing entirely.
 *   The guard (seabed_row >= 1) prevents writing into row 0 (HUD bar) on
 *   extremely short terminals (< 3 rows).
 */
static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha)
{
    /* Draw all tentacle chains (read-only access to sc) */
    for (int i = 0; i < N_TENTACLES; i++) {
        render_tentacle(&sc->t[i], w, cols, rows, alpha);
    }

    /* Seabed atmosphere: row of '~' at the bottom content row               */
    int seabed_row = rows - 2;   /* rows-1 is the hint bar, not content      */
    if (seabed_row >= 1) {
        wattron(w, COLOR_PAIR(1) | A_DIM);
        for (int c = 0; c < cols; c++) {
            mvwaddch(w, seabed_row, c, (chtype)'~');
        }
        wattroff(w, COLOR_PAIR(1) | A_DIM);
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the ncurses display layer.
 *
 * Holds the current terminal dimensions (cols, rows) as read by getmaxyx()
 * after each resize.  All rendering is done into stdscr using these dimensions
 * for bounds checking (no draws outside [0, cols) × [0, rows)).
 *
 * See framework.c §7 for the full double-buffer architecture rationale:
 *   erase()          — write spaces over newscr in memory (no terminal I/O)
 *   scene_draw()     — stamp glyphs into newscr
 *   wnoutrefresh()   — copy newscr to ncurses' pending-update structure
 *   doupdate()       — diff pending against curscr; send only changed cells
 * This sequence ensures one atomic write to the terminal fd per frame.
 */
typedef struct { int cols, rows; } Screen;

/*
 * screen_init() — configure the terminal for animation.
 *
 *   initscr()        Initialise ncurses; must be the first ncurses call.
 *                    Saves and clears the terminal; allocates stdscr.
 *
 *   noecho()         Suppress echoing of typed characters to the screen.
 *                    Without this, each keypress would appear as a visible
 *                    character, corrupting the animation frame.
 *
 *   cbreak()         Pass key events to the program immediately, without
 *                    waiting for Enter (line-buffered mode).  Required for
 *                    responsive single-keypress input.
 *
 *   curs_set(0)      Hide the hardware cursor.  Without this, the cursor
 *                    would blink at the last mvwaddch position every frame,
 *                    distracting from the animation.
 *
 *   nodelay(TRUE)    Make getch() return ERR immediately when no key is
 *                    pending, rather than blocking.  This is what makes the
 *                    input poll in §8 step ⑧ non-blocking.
 *
 *   keypad(TRUE)     Decode multi-byte escape sequences (arrow keys, function
 *                    keys) into single KEY_* integer constants.  Without this,
 *                    pressing ↑ would deliver three separate characters:
 *                    ESC, '[', 'A'.
 *
 *   typeahead(-1)    Disable ncurses' habit of calling read() mid-output to
 *                    check for pending escape sequences.  Without this, the
 *                    terminal output stream can be interrupted, causing
 *                    partially rendered frames to appear on screen.
 */
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

/*
 * screen_free() — restore the terminal to its pre-animation state.
 * endwin() re-enables echo, shows the cursor, and restores the scroll region.
 */
static void screen_free(Screen *s) { (void)s; endwin(); }

/*
 * screen_resize() — handle a SIGWINCH (terminal resize) event.
 *
 * endwin() + refresh() forces ncurses to re-read LINES and COLS from the
 * kernel and resize its internal virtual screens (curscr / newscr) to match
 * the new terminal dimensions.  Without this sequence, stdscr retains the
 * old dimensions and mvwaddch calls for newly valid coordinates silently fail.
 *
 * Called from app_do_resize() which also repositions tentacle roots to the
 * new screen width and recomputes seg_len_px for the new height.
 */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw() — compose the full frame into stdscr (ncurses' newscr buffer).
 *
 * Frame composition order (must not change):
 *   1. erase()           — fill newscr with spaces in memory only (no I/O).
 *                          Clears all stale content from the previous frame.
 *   2. scene_draw()      — stamp tentacle chains and seabed '~' row.
 *   3. Top HUD bar       — status line; drawn after scene_draw() so it always
 *                          overlays any tentacle glyph in row 0.
 *   4. Bottom hint bar   — key reference; always on top of seabed or tentacle.
 *
 * Nothing is written to the terminal fd until screen_present() calls doupdate().
 *
 * HUD CONTENT:
 *   fps       — smoothed frames-per-second (render rate)
 *   Hz        — physics simulation rate (scene_tick() calls per second)
 *   amp       — current sway amplitude (peak bend per segment, rad)
 *   freq      — current sway frequency (rad/s)
 *   state     — "swaying" or "PAUSED"
 *
 * The HUD string is right-aligned (hx = cols − strlen(buf)) so it never
 * collides with tentacle tips that may extend into row 0 on tall terminals.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha);

    /* Top HUD bar — right-aligned simulation state */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %.1ffps  %dHz  amp:%.2f  freq:%.2f  %s ",
             fps, sim_fps,
             sc->amplitude, sc->frequency,
             sc->paused ? "PAUSED" : "swaying");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);

    /* Bottom hint bar — keyboard quick reference */
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  w/s(↑↓):freq  a/d(←→):amp  [/]:Hz ");
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);
}

/*
 * screen_present() — flush the composed frame to the terminal in one write.
 *
 * wnoutrefresh(stdscr) — copies the in-memory newscr model into ncurses'
 *   internal "pending update" structure.  No terminal I/O happens yet.
 *
 * doupdate() — diffs the pending update against curscr (what is physically
 *   on screen), sends only the changed cells to the terminal file descriptor,
 *   then updates curscr to match.  For a gently swaying tentacle forest, this
 *   typically sends 200–400 cells per frame (vs. the ~4000 cells needed for
 *   a full erase() + rewrite).  The diff is the key efficiency win.
 *
 * This two-step sequence is the correct way to flush ncurses output.  Calling
 * refresh() directly is equivalent for a single window but cannot be batched
 * with other windows; the two-step approach scales to multi-window layouts.
 */
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level application state, accessible from POSIX signal handlers.
 *
 * Declared as a file-scope global (g_app) because POSIX signal handlers
 * receive no user-defined argument — they can only communicate with the main
 * loop through global or static variables.
 *
 * running    — main-loop sentinel; set to 0 by SIGINT / SIGTERM handler.
 * need_resize — set to 1 by SIGWINCH handler; checked at the top of each
 *               frame so resize is handled before any ncurses operations.
 *
 * Both fields are volatile sig_atomic_t because:
 *   volatile     — prevents the compiler from caching the value in a register
 *                  across the signal-handler write.  Without volatile, the
 *                  compiler might hoist the check out of the loop entirely,
 *                  making the signal never visible to the main thread.
 *   sig_atomic_t — the only integer type that POSIX guarantees can be read
 *                  and written atomically from a signal handler.  Using int
 *                  would be technically undefined behaviour on some platforms.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

/* Signal handlers — set flags only; no ncurses or malloc calls permitted */
static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/*
 * cleanup() — atexit safety net.
 *
 * Registered with atexit() so that endwin() is called even if the program
 * exits via an unhandled signal path that skips the normal screen_free()
 * call.  Without this, the terminal could be left in raw/no-echo mode after
 * a crash, requiring a manual "reset" command to recover.
 */
static void cleanup(void) { endwin(); }

/*
 * app_do_resize() — handle a pending SIGWINCH terminal resize.
 *
 * Called at the TOP of the main loop (step ①) when need_resize is set,
 * before any other ncurses operations.  This ordering is critical: ncurses
 * calls made before screen_resize() would use stale LINES/COLS values.
 *
 * WAVE STATE PRESERVATION:
 *   The wave state (wave_time, amplitude, frequency) is saved before
 *   scene_init() and restored afterward.  Without this, every resize would
 *   reset the animation to t=0, producing a visible jerk as tentacles snap
 *   back to their straight-up rest posture.  Preserving wave_time lets the
 *   animation continue smoothly from exactly where it was.
 *
 * WHY scene_init() IS CALLED (not just screen_resize()):
 *   Tentacle roots must be repositioned for the new terminal width.  seg_len_px
 *   must be recalculated for the new terminal height.  Both are encoded in the
 *   root_px, root_py, and seg_len_px fields that scene_init() computes.
 *   A targeted re-root function would be equivalent but more fragile; calling
 *   scene_init() guarantees all geometry is consistent.
 *
 * frame_time and sim_accum are reset in main() after this returns to prevent
 * a physics avalanche.  During a resize operation, the process can be briefly
 * suspended by the kernel while the terminal driver updates its window size.
 * The resulting large dt would fire many physics ticks in one frame if not
 * reset — the tentacles would suddenly flail as if many seconds had passed.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);

    /* Save wave state before scene_init() clears it */
    float saved_wave_time = app->scene.wave_time;
    float saved_amp       = app->scene.amplitude;
    float saved_freq      = app->scene.frequency;

    /* Recompute geometry for new terminal dimensions */
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    /* Restore wave state so animation continues without visual discontinuity */
    app->scene.wave_time = saved_wave_time;
    app->scene.amplitude = saved_amp;
    app->scene.frequency = saved_freq;

    app->need_resize = 0;
}

/*
 * app_handle_key() — process a single keypress; return false to quit.
 *
 * All parameter changes take effect on the NEXT call to scene_tick() because
 * scene_tick() reads amplitude and frequency directly from the Scene struct
 * that this function modifies.  There is no need to signal or synchronise;
 * the main loop processes all keys before scene_tick() each frame.
 *
 * KEY MAP (letter keys alias arrow keys for home-row ergonomics):
 *
 *   q / Q / ESC (27)   quit
 *   space              toggle paused
 *   ↑  / w / W         frequency + 0.15 rad/s  (faster sway)
 *   ↓  / s / S         frequency − 0.15 rad/s  (slower sway)
 *   →  / d / D         amplitude + 0.10 rad    (wider sway, stronger current)
 *   ←  / a / A         amplitude − 0.10 rad    (narrower sway, calmer current)
 *   ]                  sim_fps + SIM_FPS_STEP   (more physics ticks/second)
 *   [                  sim_fps − SIM_FPS_STEP   (fewer physics ticks/second)
 *
 * STEP SIZES — why 0.15 / 0.10?
 *   0.15 rad/s for frequency: at FREQ_DEFAULT = 0.8, a single press changes
 *   period by 2π/(0.95) − 2π/0.8 ≈ 1.0 second — clearly audible change in pace.
 *   0.10 rad for amplitude: changes tip deflection by roughly ±0.8 rad ≈ 46°
 *   — a noticeable swing width change with each keypress.
 *   Both are intentionally generous so the user can tune by feel quickly.
 *
 * CLAMPING:
 *   Each parameter is clamped to its physical range after adjustment.
 *   Clamping prevents runaway values (e.g., negative frequency) that would
 *   produce undefined visual behaviour or reverse the direction of time.
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;   /* 27 = ESC                */

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

/* ─────────────────────────────────────────────────────────────────────
 * main() — the game loop  (structure identical to framework.c §8)
 *
 * The loop runs indefinitely at ~60 fps until the user quits.  Each frame
 * body executes eight steps:
 *
 *  ① RESIZE CHECK
 *     Inspect the need_resize flag (set by SIGWINCH handler) before any
 *     ncurses operations.  Call app_do_resize() which re-reads LINES/COLS,
 *     repositions roots, and recomputes seg_len_px.  Reset frame_time and
 *     sim_accum so the large dt that accumulated during the OS resize
 *     operation does not trigger a physics avalanche.
 *
 *  ② MEASURE dt
 *     dt = clock_ns() − frame_time  (wall-clock nanoseconds since last frame).
 *     Capped at 100 ms: if the process was suspended (Ctrl-Z, debugger pause,
 *     or OS scheduler stall) and then resumed, the dt would be enormous.
 *     Injecting that many nanoseconds into the physics accumulator would fire
 *     hundreds of ticks at once, making the tentacles jump discontinuously.
 *     The 100 ms cap is the "suspend guard" — it bounds the maximum physics
 *     catch-up to 6 ticks at 60 Hz.
 *
 *  ③ FIXED-STEP ACCUMULATOR
 *     sim_accum accumulates wall-clock dt nanoseconds each frame.
 *     tick_ns = 1 000 000 000 / sim_fps  (nanoseconds per one physics tick).
 *     While sim_accum ≥ tick_ns: fire scene_tick(dt_sec), drain tick_ns.
 *     This decouples physics Hz from render Hz:
 *       - Render faster than sim (e.g., 120 fps display, 60 Hz physics):
 *         every other frame fires 0 ticks; frames in between fire 1 tick.
 *       - Render slower than sim (e.g., 30 fps display, 60 Hz physics):
 *         every frame fires 2 ticks to keep up.
 *     In all cases, scene_tick() is called at the correct average rate.
 *
 *  ④ ALPHA — sub-tick interpolation factor
 *     After draining whole ticks, sim_accum holds the fractional leftover —
 *     how far into the NEXT unfired tick we are.
 *       alpha = sim_accum / tick_ns  ∈ [0.0, 1.0)
 *     Passed to render_tentacle() so joint positions are lerped between the
 *     last completed tick and the next one, eliminating micro-stutter that
 *     would otherwise appear whenever render Hz is not an exact multiple of
 *     physics Hz.
 *
 *  ⑤ FPS COUNTER
 *     Frames are counted over a 500 ms sliding window.  Every FPS_UPDATE_MS
 *     milliseconds, fps_display is updated:
 *       fps_display = frame_count / (fps_accum / NS_PER_SEC)
 *     This per-window average is far more stable than per-frame measurement,
 *     which would oscillate wildly due to nanosecond-level timing jitter.
 *
 *  ⑥ FRAME CAP — sleep BEFORE render
 *     elapsed = time spent on physics and overhead this frame.
 *     budget  = NS_PER_SEC / 60  (≈16.67 ms for one 60-fps frame).
 *     sleep   = budget − elapsed.
 *     Sleeping BEFORE the terminal write (not after) means the write cost
 *     is not charged against the next frame's budget.  If sleep ≤ 0, the
 *     frame is over-budget; clock_sleep_ns() returns immediately.
 *
 *  ⑦ DRAW + PRESENT
 *     erase() → scene_draw() → HUD bars → wnoutrefresh() → doupdate().
 *     The two-step wnoutrefresh/doupdate sequence sends only changed cells
 *     to the terminal fd — typically 200–400 cells for swaying tentacles on
 *     a dark background.
 *
 *  ⑧ DRAIN INPUT
 *     Loop getch() until ERR, processing every queued key event.
 *     Looping (not a single call) ensures all key-repeat events generated
 *     while the frame was rendering are consumed before the next physics tick,
 *     keeping parameter adjustments responsive when keys are held down.
 * ───────────────────────────────────────────────────────────────────── */
int main(void)
{
    /* Safety net: endwin() even if we exit via an unhandled signal path     */
    atexit(cleanup);

    /* SIGINT / SIGTERM — graceful exit from Ctrl-C or kill                  */
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);

    /* SIGWINCH — terminal resize; handled at step ① of the next iteration   */
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();   /* timestamp of the last frame start */
    int64_t sim_accum   = 0;            /* nanoseconds in the physics bucket  */
    int64_t fps_accum   = 0;            /* ns elapsed in current fps window   */
    int     frame_count = 0;            /* frames rendered in fps window      */
    double  fps_display = 0.0;          /* smoothed fps value shown in HUD   */

    while (app->running) {

        /* ── ① resize ─────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();   /* reset so large dt doesn't spike    */
            sim_accum  = 0;
        }

        /* ── ② dt — wall-clock elapsed, capped at 100 ms ─────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;   /* suspend guard  */

        /* ── ③ fixed-step accumulator — fire physics at sim_fps Hz ── */
        int64_t tick_ns = TICK_NS(app->sim_fps);   /* ns per one physics tick */
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── ④ alpha — sub-tick interpolation factor ∈ [0, 1) ─────── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── ⑤ fps counter — 500 ms sliding window average ─────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── ⑥ frame cap — sleep before render to target 60 fps ───── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── ⑦ draw + present — one atomic diff write to terminal ──── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha);
        screen_present();

        /* ── ⑧ drain all pending input — responsive to key repeat ──── */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }
    }

    screen_free(&app->screen);
    return 0;
}
