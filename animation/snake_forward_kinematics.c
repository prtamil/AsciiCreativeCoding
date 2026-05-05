/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * snake_forward_kinematics.c — autonomous FK snake bouncing inside the screen
 *
 * DEMO: A 32-segment snake swims autonomously along a smooth sinusoidal
 *       S-curve. When it nears any screen edge, an edge-bias steering term
 *       turns it inward — the snake naturally curves back rather than
 *       wrapping or flying off.
 *
 * Study alongside: snake_inverse_kinematics.c (same body, IK head + bouncing target)
 *
 * Section map:
 *   §1 config   — tunables: speeds, wave, edge bias, geometry
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 10 themes × 7 body pairs + PAIR_HUD/PAIR_HINT
 *   §4 coords   — pixel↔cell aspect-ratio bridge
 *   §5 entity   — Snake: trail buffer, path-following FK, renderer
 *       §5a trail helpers      — push, index, arc-length sampler
 *       §5b move_head          — wave + edge bias + translate + clamp + record
 *       §5c compute_joints     — body placement from trail
 *       §5d rendering helpers  — glyphs, pair, attribute
 *       §5e mark_cell          — central glyph stamp helper
 *       §5f draw_segment_beads — bead fill for one segment
 *       §5g render_chain       — full frame composition
 *   §6 scene    — scene_init / scene_tick / scene_draw
 *   §7 screen   — ncurses double-buffer display layer
 *   §8 app      — signals, resize, main game loop
 *
 * Keys (simulation parameters only — motion is fully autonomous):
 *   q / ESC       quit
 *   space         pause / resume
 *   ↑ / ↓         move speed faster / slower
 *   ← / →, a / d  undulation frequency slower / faster
 *   w / s         swim amplitude wider / narrower
 *   + / -         wave-time scale faster / slower
 *   t / T         next / previous theme
 *   r / R         reset (theme preserved)
 *   ] / [         sim Hz + / -
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       snake_forward_kinematics.c -o snake_fk -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Path-following FK with sinusoidal auto-steering and
 *                  edge-bias soft fence. The head's past pixel positions
 *                  are stored tick-by-tick in a circular trail buffer.
 *                  Each body joint is placed by walking that buffer until
 *                  the accumulated arc length equals i · SEG_LEN_PX, then
 *                  linearly interpolating. No per-joint angle formula is
 *                  needed; the trail encodes the full geometry of the
 *                  past path. The autonomous steering integrates a sine-
 *                  wave turn rate into the heading each tick, producing
 *                  a continuous S-curve. When the head approaches any
 *                  edge, an additional inward turn is added proportional
 *                  to penetration depth (an artificial-potential field
 *                  with linear falloff inside an EDGE_MARGIN_PX band).
 *
 * Data-structure : trail[TRAIL_CAP] is a Vec2 ring buffer; trail_head is
 *                  the write cursor; trail_at(k) returns the entry k ticks
 *                  back. joint[N_SEGS+1] is the current chain; prev_joint[]
 *                  is its snapshot at the start of the tick (anchor for
 *                  sub-tick alpha lerp). All positions live in pixel space
 *                  — square, isotropic — so the curve's geometry is correct
 *                  regardless of terminal cell aspect ratio.
 *
 * Rendering      : draw_segment_beads() walks each of the 32 segments in
 *                  DRAW_STEP_PX increments, stamping 'o' at every cell the
 *                  line crosses (cell-dedup avoids attr flicker). A second
 *                  pass over-stamps each joint with a graded node ('0' /
 *                  'o' / '.') and the head is drawn last as a directional
 *                  arrow. A 10-theme warm-to-cool palette runs head→tail.
 *                  Alpha interpolation between prev_joint and joint keeps
 *                  motion smooth at any sim/render rate combination.
 *
 * Performance    : Fixed-step accumulator decouples physics Hz from render
 *                  Hz. trail_sample is O(distance/speed) per joint; with
 *                  32 joints at 60 Hz this is well under 1 µs per frame.
 *                  ncurses doupdate transmits only changed cells.
 *
 * References     : Khoshrou, "Snake Robot Locomotion," 2009 — analysis of
 *                    serpentine FK on a moving curve.
 *                  Reynolds, "Steering Behaviors for Autonomous Characters,"
 *                    1999 — the seminal source for edge-bias / containment.
 *                  Wikipedia: "Forward kinematics" — chained-frame model.
 *                  Red Blob Games, "2D Visibility / Field of View" — the
 *                    cell-walk pattern reused here for bead rendering.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The body never has its own physics — it just remembers where the head
 * has been. Place joint i exactly i·SEG_LEN_PX behind the head along the
 * recorded path, and the chain bends naturally with every curve. The
 * head's heading is itself driven by a sine wave (autonomous wiggle); a
 * second steering term bends it inward whenever it gets close to any
 * screen edge, so the snake stays on-screen forever without wrapping.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a marker drawn across paper, with 32 dots strung behind the
 * pen at fixed bead spacings along the inked line. Whatever the pen's
 * curve looks like, the dots curve the same way — they ARE the line,
 * just sampled at later distances. The wiggle comes from the pen's
 * built-in oscillating "wrist". The wall-bounce is a second hand
 * gently nudging the wrist whenever it strays too close to the edge of
 * the page — gradual, never a sharp slap.
 *
 * DRAWING METHOD / ALGORITHM IN STEPS
 * ───────────────────────────────────
 *   1. Save prev_joint = joint                     (anchor for alpha lerp)
 *   2. wave_time += dt · speed_scale
 *   3. turn_wave  = amplitude · sin(frequency · wave_time)
 *   4. turn_edge  = edge_bias_turn(joint[0], wpx, hpx) (see KEY FORMULAS)
 *   5. heading   += (turn_wave + turn_edge) · dt
 *   6. joint[0]  += move_speed · (cos heading, sin heading) · dt
 *   7. Hard-clamp joint[0] to [0,wpx] × [0,hpx]      (safety net)
 *   8. trail_push(joint[0])
 *   9. For i in 1..N_SEGS: joint[i] = trail_sample(i · SEG_LEN_PX)
 *  10. Render: lerp prev_joint → joint by alpha; bead-fill, node markers,
 *      head arrow.
 *
 * KEY FORMULAS
 * ────────────
 *   Sinusoidal auto-turn:
 *     turn_wave = amplitude · sin(frequency · wave_time)            [rad/s]
 *
 *   Edge-bias steering (linear potential inside EDGE_MARGIN_PX):
 *     fx = max(0, (margin − x)/margin) − max(0, (x − (wpx−margin))/margin)
 *     fy = max(0, (margin − y)/margin) − max(0, (y − (hpx−margin))/margin)
 *     desired = atan2(fy, fx)             (inward direction unit vector)
 *     delta   = wrap_pi(desired − heading)
 *     turn_edge = delta · sqrt(fx² + fy²) · EDGE_TURN_GAIN          [rad/s]
 *
 *   Trail arc-length sampling (place joint i):
 *     walk trail newest→oldest, accumulating |trail[k+1] − trail[k]|;
 *     when accum ≥ i · SEG_LEN_PX, lerp into the segment that crossed it.
 *
 *   Pixel→cell aspect bridge:
 *     cx = round(px / CELL_W)              (CELL_W = 8)
 *     cy = round(py / CELL_H)              (CELL_H = 16)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • If two consecutive trail samples land on the same pixel (paused or
 *     speed=0), trail_sample's normalisation would divide by zero. Guard
 *     with `seg = max(seg, 1e-4f)` in the lerp denominator.
 *   • Heading is allowed to grow unbounded — sin/cos handle wrap. Do NOT
 *     normalise to ±π in move_head; the discontinuity at the wrap point
 *     causes a one-frame direction jerk.
 *   • Edge bias must clamp the position too — at very high speeds (500 px/s)
 *     the turn rate may not catch up, so a hard clamp is the safety net.
 *   • Resize: rope_len_px scales with terminal rows but trail content
 *     does not — the body length stays constant in pixels regardless of
 *     terminal size. That is intentional; body length should not change
 *     when the user just makes the window wider.
 *   • Frame cap: never `elapsed = clock_ns() − frame_time + dt` — the +dt
 *     cancels the cap; sleep is always 0 and CPU pegs at 100 %. Use
 *     `clock_ns() − frame_start` with a fresh snapshot.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Set amplitude=0 (s key until 0): the snake swims straight. Drive it
 *     toward any wall — it should curve away within EDGE_MARGIN_PX of the
 *     edge, not wrap to the other side and not exit.
 *   • Crank move_speed to MAX (UP×many): even at 500 px/s the head must
 *     stay inside (the hard clamp catches anything edge bias misses).
 *   • Pause (space): joints freeze in place; resume is glitch-free.
 *   • Theme cycle (t/T): every theme entry is legible against a default
 *     terminal background under A_DIM. No segment "disappears" at the tail
 *     end. Confirms the cube ≥ 24 brightness rule.
 *   • Reset (r/R) repeatedly: every reset places the head ≥ EDGE_MARGIN_PX
 *     from any edge so it has room to start swimming without an immediate
 *     edge-bias kick.
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
     * The accumulator loop in §8 fires scene_tick() this many times
     * per wall-clock second regardless of render frame rate.
     * Raising sim Hz makes physics more accurate but costs more CPU.
     * [10, 120] is the user-adjustable range; default 60 matches the
     * 60-fps render cap so one physics tick fires per rendered frame.
     */
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,   /* step size for [/] keys                  */

    /*
     * HUD_COLS — byte budget for the status bar string.
     * strlen(buf) must never exceed this; snprintf truncates safely.
     * 96 bytes covers the longest possible HUD string at max values.
     *
     * FPS_UPDATE_MS — how often the displayed fps value is recalculated.
     * 500 ms gives a stable reading without flickering on each frame.
     */
    HUD_COLS         =  96,
    FPS_UPDATE_MS    = 500,

    /*
     * N_PAIRS — number of gradient color pairs for the snake body (§3).
     * Pairs 1..7 map head→tail from warm (yellow) to cool (blue).
     *
     * PAIR_HUD / PAIR_HINT — dedicated, theme-independent pairs for the
     * top status and bottom key-hint bars per the project HUD spec.
     * Their colours never shift when the body palette cycles, so the
     * HUD stays readable against any backdrop.
     */
    N_PAIRS          =   7,
    PAIR_HUD         =   8,   /* bright yellow — top status bar  */
    PAIR_HINT        =   9,   /* bright cyan   — bottom key hint */
    N_THEMES         =  10,

    /*
     * N_SEGS — number of rigid body segments.
     *
     * More segments means more joint positions sampled from the trail,
     * which gives finer arc-length resolution along the body curve.
     * With 32 segments at 18 px each, total snake body = 576 px ≈ 72
     * terminal columns.  On a 200-column terminal this fills ~36% of
     * the screen width — long enough to show the full S-curve.
     *
     * Compared with 24 segments at 28 px (the previous default), the
     * shorter segments (18 px ≈ 2.25 cols vs 3.5 cols) change direction
     * more gradually and the "corner" artefact between adjacent segments
     * is much less visible.
     */
    N_SEGS           =  32,

    /*
     * TRAIL_CAP — capacity of the circular head-position history buffer.
     *
     * The trail must hold enough entries to cover the full snake body
     * length (N_SEGS × SEG_LEN_PX = 576 px) at any allowed speed.
     *
     * Worst case: MOVE_SPEED_MIN (20 px/s) at SIM_FPS_DEFAULT (60 Hz).
     *   Distance added per tick = 20 / 60 ≈ 0.33 px.
     *   Entries needed to cover 576 px = 576 / 0.33 ≈ 1745.
     *   4096 gives a ~2.3× safety margin.
     *
     * At the other extreme (500 px/s, 60 Hz), each tick adds 8.3 px
     * and only ~70 entries cover the full body — TRAIL_CAP is far
     * more than enough.
     *
     * Memory: 4096 × sizeof(Vec2) = 4096 × 8 = 32 KB.  Snake is a
     * global (g_app → scene → snake) so this lives in BSS, not stack.
     */
    TRAIL_CAP        = 4096,
};

/*
 * SEG_LEN_PX — pixel length of each rigid body segment.
 *
 * With CELL_W = 8 px per column, 18 px ≈ 2.25 terminal columns per
 * segment.  This gives the curve enough joints-per-cell that direction
 * transitions look gradual rather than blocky.
 *
 * DRAW_STEP_PX — pixel step size used by draw_segment_dense().
 *
 * The dense renderer walks each segment in increments of DRAW_STEP_PX,
 * converting each sample to a terminal cell and stamping a glyph.  The
 * step must be small enough that no cell is ever skipped.
 *
 * Critical constraint: DRAW_STEP_PX < CELL_W (8 px).  With 3.0 px:
 *   Horizontal segment (18 px): ceil(18/3)+1 = 7 samples → covers
 *     18/8 = 2.25 cells.  At 3.0 px/sample = 0.375 cols/sample: 6
 *     samples easily span 2.25 cells.  No cell missed. ✓
 *   Near-vertical segment (18 px, 89°): step_y ≈ 3.0 px.  Cells are
 *     16 px tall so step_y/CELL_H = 0.19 cells/sample: plenty of
 *     density for a 18/16 = 1.125 cell vertical span. ✓
 */
#define SEG_LEN_PX    18.0f
#define DRAW_STEP_PX   3.0f

/*
 * MOVE_SPEED_DEFAULT — head translation speed in pixel space (px/s).
 *
 * At 72 px/s the snake crosses a 200-column terminal (1600 px wide) in
 * about 22 seconds — comfortable to watch.  The body (576 px long) takes
 * 8 seconds to fully clear the screen at this speed.
 *
 * MOVE_SPEED_MIN / MAX define the keyboard-adjustable range.
 * The ↑/↓ keys scale by 1.20× per press (20% increments).
 */
#define MOVE_SPEED_DEFAULT   72.0f   /* px/s                               */
#define MOVE_SPEED_MIN       20.0f   /* slowest crawl                      */
#define MOVE_SPEED_MAX      500.0f   /* sprint — hard to follow visually   */

/*
 * Auto-swim wave parameters — the engine of the snake's locomotion.
 *
 * The heading angle changes at rate:
 *   dheading/dt = amplitude × sin(frequency × wave_time)
 *
 * Integrating over one full oscillation period (T = 2π/frequency):
 *   Net heading change = 0   (symmetric, no drift)
 *   Peak heading swing = amplitude / frequency  [radians]
 *
 * The snake's path in space is an approximate sinusoid whose:
 *   lateral amplitude ≈ (move_speed × amplitude) / frequency²  [px]
 *   spatial wavelength = move_speed × 2π / frequency           [px]
 *
 * DEFAULT VALUES and why they produce a natural-looking S-curve:
 *
 *   AMPLITUDE_DEFAULT = 0.52 rad/s
 *   FREQUENCY_DEFAULT = 0.95 rad/s
 *
 *   Peak heading swing  = 0.52 / 0.95 ≈ 0.55 rad ≈ 31°
 *     A 31° lateral arc is wide enough to be clearly visible but does
 *     not produce U-turns (which would need > 90°).
 *
 *   Oscillation period  = 2π / 0.95 ≈ 6.6 seconds
 *     The snake completes one full left-right swing in 6.6 s.  At
 *     60 Hz that is 396 ticks per cycle — very gradual.
 *
 *   Spatial wavelength  = 72 × 2π / 0.95 ≈ 476 px ≈ 59 columns
 *     The body (576 px) contains 576/476 ≈ 1.2 full wavelengths,
 *     so a single clean S-curve is visible along the full body length.
 *
 *   Lateral displacement = (72 × 0.52) / 0.95² ≈ 41 px ≈ 5 columns
 *     The snake drifts ±5 columns left and right of its mean heading —
 *     noticeable but contained; it stays well within the visible area.
 *
 * SPEED_SCALE_DEFAULT — multiplier applied to wave_time advancement.
 *   speed_scale = 1 → wave runs at natural rate.
 *   speed_scale = 2 → wave runs twice as fast (tighter curves, same path).
 *   The +/- keys adjust speed_scale in ×1.25 / ÷1.25 steps.
 */
#define AMPLITUDE_DEFAULT    0.52f   /* peak turn rate (rad/s)             */
#define AMPLITUDE_MIN        0.0f    /* 0 → perfectly straight swim        */
#define AMPLITUDE_MAX        4.0f    /* > 2 creates tight spirals          */
#define FREQUENCY_DEFAULT    0.95f   /* oscillation angular frequency (rad/s)*/
#define FREQUENCY_MIN        0.10f   /* very long lazy curves              */
#define FREQUENCY_MAX        6.00f   /* extremely rapid zigzag             */
#define SPEED_SCALE_DEFAULT  1.0f

/*
 * Edge-bias steering — keeps the snake on-screen without toroidal wrap.
 *
 * Inside an EDGE_MARGIN_PX band along each edge, an inward-pointing
 * "potential" is summed and converted to an extra turn rate.  Outside
 * the band the bias is zero, so the wave-driven motion is unaffected
 * in the screen interior.
 *
 * EDGE_MARGIN_PX — width of the soft-fence band, in pixels.
 *   80 px ≈ 10 columns / 5 rows.  Wide enough that the edge-turn starts
 *   gently and curves the snake back over several body lengths; narrow
 *   enough that the snake still uses most of the screen interior.
 *
 * EDGE_TURN_GAIN — gain on the edge-bias turn rate (rad/s per unit
 *   penetration).  At max penetration (snake right at the wall) the
 *   strength factor is ~1, so the peak edge turn ≈ EDGE_TURN_GAIN rad/s.
 *   3.5 turns the snake about 200°/s at the wall — fast enough to round
 *   the corner before crossing it, slow enough to look like a deliberate
 *   curve, not a hard reflection.
 */
#define EDGE_MARGIN_PX     80.0f
#define EDGE_TURN_GAIN      3.5f

/*
 * Timing primitives — verbatim from framework.c.
 *
 * NS_PER_SEC / NS_PER_MS: unit conversion constants.
 * TICK_NS(f): converts a frequency f (Hz) to the period in nanoseconds.
 *   e.g. TICK_NS(60) = 1 000 000 000 / 60 ≈ 16 666 667 ns per tick.
 * Used in the fixed-step accumulator (see §8 main()).
 */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Terminal cell dimensions — the aspect-ratio bridge between physics
 * and display (see §4 for full explanation).
 *
 * CELL_W = 8 px, CELL_H = 16 px.  A typical terminal cell is twice as
 * tall as it is wide in physical pixels.  All FK positions are stored
 * in a square pixel space scaled by these constants so that 1 pixel
 * represents the same physical distance in x and y.
 */
#define CELL_W   8    /* physical pixels per terminal column */
#define CELL_H  16    /* physical pixels per terminal row    */

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/*
 * clock_ns() — monotonic wall-clock in nanoseconds.
 *
 * CLOCK_MONOTONIC is guaranteed never to go backward (unlike
 * CLOCK_REALTIME, which can jump on NTP corrections or DST changes).
 * Subtracting two consecutive clock_ns() calls gives the true elapsed
 * wall time regardless of system load or clock adjustments.
 *
 * Returns int64_t (signed 64-bit) so dt differences are naturally
 * signed and negative values (e.g., from the pause-guard cap) don't
 * wrap around.
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
 * Called BEFORE render in §8 to cap the frame rate at 60 fps.
 * Sleeping before (not after) terminal I/O means only the physics
 * computation time is charged against the budget; the terminal write
 * cost does not accumulate into the next frame's elapsed time.
 *
 * If ns ≤ 0 the frame is already over-budget (physics took too long);
 * skip the sleep entirely rather than sleeping a negative duration.
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
/* §3  color — warm-to-cool 7-step gradient + dedicated HUD pair         */
/* ===================================================================== */

/*
 * Theme — one named body palette.
 *
 *   name   — displayed in the HUD status bar.
 *   body[] — 7 xterm-256 foreground indices for pairs 1..7 (head→tail).
 *
 * Brightness rule: every body[] entry is in the bright half of the
 * 256-colour cube (≥ 24).  Indices 16–23 (cube near-blacks) and 232–239
 * (gray near-blacks) become invisible under A_DIM at the tail-quarter
 * of the body, so they are avoided.
 *
 * theme_apply() calls init_pair() live; switching themes takes effect on
 * the very next frame.  PAIR_HUD/PAIR_HINT are theme-independent and are
 * configured once in color_init().
 */
typedef struct {
    const char *name;
    int         body[N_PAIRS];
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name      head ←─────────────────────→ tail */
    {"Solar",  {226, 220, 214, 208, 202, 196, 160}},
    {"Matrix", { 28,  34,  40,  76,  46,  82, 118}},
    {"Ocean",  { 24,  25,  31,  33,  39,  45,  51}},
    {"Fire",   {196, 202, 208, 214, 220, 226, 227}},
    {"Nova",   { 54,  55,  56,  57,  93, 129, 165}},
    {"Medusa", { 57,  63,  93,  99, 105, 111, 159}},
    {"Lava",   { 52,  88, 124, 160, 196, 202, 208}},
    {"Ghost",  {244, 245, 247, 249, 251, 253, 255}},
    {"Aurora", { 28,  34,  64,  71,  78, 121, 159}},
    {"Neon",   {201, 165, 129,  93,  57,  51,  45}},
};

/*
 * theme_apply() — register the body palette with ncurses.
 *
 * Background is -1 (terminal default) so demos respect the user's theme.
 * On 8-colour terminals, falls back to a yellow→green→cyan→blue gradient
 * that approximates the warm-to-cool feel of every 256-colour theme.
 */
static void theme_apply(int idx)
{
    const Theme *th = &THEMES[idx];
    if (COLORS >= 256) {
        for (int p = 0; p < N_PAIRS; p++)
            init_pair(p + 1, th->body[p], -1);
    } else {
        static const int fb8[N_PAIRS] = {
            COLOR_YELLOW, COLOR_YELLOW, COLOR_GREEN,
            COLOR_GREEN,  COLOR_CYAN,   COLOR_CYAN, COLOR_BLUE
        };
        for (int p = 0; p < N_PAIRS; p++)
            init_pair(p + 1, fb8[p], -1);
    }
}

/*
 * color_init() — one-time colour system setup.
 *
 *   start_color()        — initialise ncurses colour support.
 *   use_default_colors() — allow background = -1 to mean "terminal default".
 *   theme_apply()        — register the initial body palette.
 *   PAIR_HUD / PAIR_HINT — bright yellow / cyan, theme-independent per
 *                          the project HUD spec.
 */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();
    theme_apply(initial_theme);

    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell; the one aspect-ratio fix                     */
/* ===================================================================== */

/*
 * WHY TWO COORDINATE SPACES
 * ─────────────────────────
 * Terminal cells are not square.  A typical cell is ~2× taller than wide
 * in physical pixels (CELL_H=16 vs CELL_W=8).  If snake positions were
 * stored directly in cell coordinates, moving the head by (dx, dy) cells
 * would travel twice as far vertically as horizontally in physical space.
 * The snake's path would look squashed, and the sinusoidal curvature
 * would appear asymmetric depending on the angle of travel.
 *
 * THE FIX — physics in pixel space, drawing in cell space:
 *   All Vec2 positions (trail[], joint[], prev_joint[]) are in pixel
 *   space where 1 unit = 1 physical pixel.  The pixel grid is square and
 *   isotropic: moving 1 unit in x or y covers the same physical distance.
 *   Only at draw time does §5e convert to cell coordinates.
 *
 * px_to_cell_x / px_to_cell_y — round to nearest cell.
 *
 * Formula:  cell = floor(px / CELL_DIM + 0.5)
 *   Adding 0.5 before flooring = "round half up" — deterministic and
 *   symmetric.  roundf() uses "round half to even" (banker's rounding)
 *   which can oscillate when px lands exactly on a cell boundary, causing
 *   a single-cell flicker.  Truncation ((int)(px/CELL_W)) always rounds
 *   down, giving asymmetric dwell at boundaries.  floor+0.5 avoids both.
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
/* §5  entity — Snake: trail buffer + path-following FK                  */
/* ===================================================================== */

/*
 * Vec2 — lightweight 2-D position vector in pixel space.
 * x increases eastward; y increases downward (terminal convention).
 */
typedef struct { float x, y; } Vec2;

/*
 * Snake — complete simulation state for the swimming snake.
 *
 * TRAIL BUFFER (circular, newest entry at trail[trail_head]):
 *
 *   trail[]      Array of Vec2 pixel positions, one pushed per sim tick.
 *                Oldest entry is silently overwritten once the buffer fills.
 *                Indexed via trail_at(k): k=0 = newest, k=1 = one older.
 *
 *   trail_head   Write cursor (index of the most recently pushed entry).
 *                Advances by 1 (mod TRAIL_CAP) on each push.
 *
 *   trail_count  Number of valid entries, clamped to TRAIL_CAP.
 *                Reaches TRAIL_CAP after TRAIL_CAP ticks (~68 s at 60 Hz)
 *                and stays there for the life of the simulation.
 *
 * BODY POSITIONS:
 *
 *   joint[0]        Head — set by move_head() each tick.
 *   joint[1..N_SEGS] Body and tail — set by compute_joints() from the trail.
 *                    joint[N_SEGS] is the tail tip (farthest from head).
 *
 *   prev_joint[]    Snapshot of joint[] at the START of the current tick,
 *                   saved before any physics runs.  render_chain() lerps
 *                   between prev_joint and joint using alpha ∈ [0,1) to
 *                   produce sub-tick smooth motion at any render frame rate.
 *                   Without prev_joint, motion would stutter visibly at the
 *                   tick boundary when sim Hz < render Hz.
 *
 * WAVE (autonomous steering):
 *
 *   heading      Current travel direction in radians.  0 = east, π/2 = south
 *                (y increases downward in terminal space).  Updated each tick
 *                by integrating the sinusoidal turn rate.
 *
 *   wave_time    Accumulated simulation time (seconds), scaled by speed_scale.
 *                Drives the sin() argument: frequency × wave_time.
 *                Separated from wall time so speed_scale can stretch/compress
 *                the wave without touching heading or move_speed.
 *
 *   amplitude    Peak turn rate for auto-swim (rad/s).  Larger = wider arcs.
 *   frequency    Oscillation angular frequency (rad/s).  Larger = faster zigzag.
 *   speed_scale  Multiplier on wave_time advancement.  Does not affect move_speed.
 *   move_speed   Head translation speed (px/s), constant each tick.
 *
 *   paused       When true, move_head() and compute_joints() are skipped;
 *                prev_joint is still saved so alpha lerp freezes cleanly.
 */
typedef struct {
    Vec2  trail[TRAIL_CAP];        /* circular position history            */
    int   trail_head;              /* write pointer (most recent entry)    */
    int   trail_count;             /* valid entries, ≤ TRAIL_CAP           */

    Vec2  joint[N_SEGS + 1];      /* [0]=head … [N_SEGS]=tail tip         */
    Vec2  prev_joint[N_SEGS + 1]; /* joint[] at start of tick (for α lerp)*/

    float heading;     /* travel direction, radians                        */
    float move_speed;  /* translation speed, px/s                          */
    float wave_time;   /* accumulated scaled time driving sin()            */
    float amplitude;   /* auto-swim peak turn rate, rad/s                  */
    float frequency;   /* auto-swim oscillation frequency, rad/s           */
    float speed_scale; /* wave_time advancement multiplier                 */

    int   theme_idx;   /* index into THEMES[]; t/T keys cycle it           */
    bool  paused;
} Snake;

/* ── §5a  trail helpers ─────────────────────────────────────────────── */

/*
 * trail_push() — append joint[0]'s new position to the circular buffer.
 *
 * trail_head advances by 1 (wrapping at TRAIL_CAP) on each call, so it
 * always points to the most recently written slot after the push.
 *
 * When trail_count == TRAIL_CAP the buffer is full; the next push
 * overwrites the oldest entry (the one trail_head is about to move to).
 * This is correct behaviour: the tail of the snake never needs history
 * older than N_SEGS × SEG_LEN_PX / move_speed_min seconds ≈ 28.8 s,
 * and at 60 Hz × 4096 entries that covers 68 seconds — far more than
 * enough even at the slowest speed.
 */
static void trail_push(Snake *s, Vec2 pos)
{
    s->trail_head = (s->trail_head + 1) % TRAIL_CAP;
    s->trail[s->trail_head] = pos;
    if (s->trail_count < TRAIL_CAP) s->trail_count++;
}

/*
 * trail_at() — retrieve the entry k steps back from the newest.
 *
 *   k = 0  →  trail[trail_head]          newest (current head position)
 *   k = 1  →  trail[(trail_head-1)%N]    one tick older
 *   k = n  →  trail[(trail_head-n)%N]    n ticks older
 *
 * The expression  (trail_head + TRAIL_CAP - k) % TRAIL_CAP  avoids
 * negative modulo (C's % on negatives is implementation-defined before
 * C99 and can produce negative results in some compilers).  Adding
 * TRAIL_CAP before subtracting k ensures the operand is always positive
 * as long as k < TRAIL_CAP, which is guaranteed by the callers.
 *
 * Caller responsibility: k must be < trail_count (no bounds check here
 * for performance — trail_sample() enforces this via its loop bound).
 */
static inline Vec2 trail_at(const Snake *s, int k)
{
    return s->trail[(s->trail_head + TRAIL_CAP - k) % TRAIL_CAP];
}

/*
 * trail_sample() — interpolated position at arc-length dist from head.
 *
 * This is the core of path-following FK.  It answers the question:
 * "where on the head's historical path is the point that is exactly
 *  dist pixels behind the current head position?"
 *
 * ALGORITHM:
 *   Walk the trail from newest (k=0) to oldest (k=trail_count-1),
 *   accumulating the Euclidean distance between consecutive entries.
 *   When the accumulated length first reaches or exceeds dist:
 *     t = (dist − accum_before_this_segment) / segment_length
 *     return lerp(entry_before, entry_after, t)
 *
 * COST:
 *   O(dist / pixels_per_tick).  At MOVE_SPEED_DEFAULT 72 px/s and
 *   60 Hz, each tick adds 72/60 = 1.2 px to the trail.  The farthest
 *   body joint (joint[32]) sits 32 × 18 = 576 px behind the head,
 *   requiring 576 / 1.2 = 480 iterations in the worst case.  With 32
 *   joints called each tick: 32 × 480 ≈ 15 360 iterations/tick.  At
 *   60 Hz this is ~921 600 iterations/second — trivial on any modern CPU.
 *   At maximum speed (500 px/s) the cost drops to 32 × 69 ≈ 2208 iters.
 *
 * EDGE CASE:
 *   During the first TRAIL_CAP ticks (68 s at 60 Hz) the trail has not
 *   yet filled enough to cover 576 px.  In practice scene_init()
 *   pre-populates TRAIL_CAP entries so this never occurs in normal use.
 *   The fallback (return oldest entry) handles it gracefully if it does.
 *
 * NUMERICAL GUARD:
 *   Division by seg is guarded with max(seg, 1e-4f) to avoid divide-by-
 *   zero when two consecutive trail entries are at the same pixel (head
 *   standing still while paused — though pausing skips trail_push).
 */
static Vec2 trail_sample(const Snake *s, float dist)
{
    float accum = 0.0f;
    Vec2  a     = trail_at(s, 0);   /* newest trail entry = current head */

    for (int k = 1; k < s->trail_count; k++) {
        Vec2  b   = trail_at(s, k);          /* one tick older than a    */
        float dx  = b.x - a.x;
        float dy  = b.y - a.y;
        float seg = sqrtf(dx * dx + dy * dy); /* distance between a and b */

        if (accum + seg >= dist) {
            /* Target distance falls within this segment; interpolate */
            float t = (dist - accum) / (seg > 1e-4f ? seg : 1e-4f);
            return (Vec2){ a.x + dx * t, a.y + dy * t };
        }

        accum += seg;   /* haven't reached dist yet; keep walking */
        a      = b;
    }

    /* Trail exhausted before reaching dist; return oldest known point */
    return trail_at(s, s->trail_count - 1);
}

/* ── §5b  move_head ─────────────────────────────────────────────────── */

/*
 * wrap_pi() — fold any radian value into the canonical (−π, π] range.
 *
 * Used by edge_bias_turn() so the (desired − heading) angular delta is
 * a *signed shortest-arc* turn rather than the raw difference, which
 * could be 2π − ε (turn the long way around) when the heading and the
 * desired direction straddle the ±π wrap boundary.
 */
static inline float wrap_pi(float a)
{
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/*
 * edge_bias_turn() — soft-fence inward steering rate, in rad/s.
 *
 * WHAT: Returns an additional turn rate to add to the autonomous wave
 *   turn whenever the head is within EDGE_MARGIN_PX of any screen edge.
 *   Outside that band the function returns 0 — the snake's interior
 *   motion is unaffected.
 *
 * HOW (artificial-potential field with linear falloff):
 *   1. For each side, measure how deep the head has penetrated into the
 *      margin band (0 = right at the edge of the band, 1 = at the wall):
 *        depth_left = max(0, (margin − x) / margin)
 *        depth_right, depth_top, depth_bot — analogous.
 *
 *   2. Sum depth contributions into an inward-pointing 2-D vector:
 *        fx = depth_left − depth_right        (push east − push west)
 *        fy = depth_top  − depth_bot          (push south − push north)
 *
 *      |(fx, fy)| ∈ [0, √2] grows as the head penetrates further.
 *
 *   3. Convert to a desired heading (atan2) and take the signed shortest
 *      arc to the current heading.
 *
 *   4. Scale by penetration magnitude × EDGE_TURN_GAIN to get a turn
 *      rate in rad/s.  Deeper penetration → harder turn-back.
 *
 * WHY THIS SHAPE OF FALLOFF:
 *   Linear (rather than quadratic or stepped) falloff makes the steering
 *   start gently at the band's outer edge and ramp up smoothly toward
 *   the wall — visually it looks like the snake "notices" the wall and
 *   curves away, rather than slapping into a hard reflection.
 */
static float edge_bias_turn(const Snake *s, float wpx, float hpx)
{
    float margin = EDGE_MARGIN_PX;
    float x = s->joint[0].x;
    float y = s->joint[0].y;

    float fx = 0.0f, fy = 0.0f;
    if (x < margin)          fx += (margin - x)         / margin;
    if (x > wpx - margin)    fx -= (x - (wpx - margin)) / margin;
    if (y < margin)          fy += (margin - y)         / margin;
    if (y > hpx - margin)    fy -= (y - (hpx - margin)) / margin;

    if (fx == 0.0f && fy == 0.0f) return 0.0f;

    float strength = sqrtf(fx * fx + fy * fy);
    float desired  = atan2f(fy, fx);
    float delta    = wrap_pi(desired - s->heading);
    return delta * strength * EDGE_TURN_GAIN;
}

/*
 * move_head() — advance wave_time, update heading, translate, clamp, record.
 *
 * Called once per simulation tick by scene_tick().  This function is the
 * sole writer of wave_time, heading, and joint[0].
 *
 * STEP 1 — advance wave_time (scaled).
 * STEP 2 — sinusoidal autonomous turn rate, INTEGRATED into heading.
 *          Integration (not assignment) gives smooth continuous curvature.
 * STEP 3 — edge-bias turn rate, also integrated into heading.
 *          Soft-fence inside EDGE_MARGIN_PX; zero outside.
 * STEP 4 — translate head along heading by move_speed × dt.
 * STEP 5 — hard-clamp position into [0,wpx] × [0,hpx].  This is the safety
 *          net for the rare case where, at very high speed, the edge-bias
 *          turn cannot redirect the head fast enough.  Without this, a
 *          single very-large-dt frame (debugger pause + resume) could fling
 *          the head past the wall.
 * STEP 6 — push the new head position into the trail.  compute_joints()
 *          then samples this freshly pushed trail to place body joints.
 *
 * heading is NOT normalised to (−π, π] inside this function — sin/cos are
 * periodic, and skipping normalisation prevents a one-frame direction
 * jerk when the heading crosses the wrap boundary.  edge_bias_turn() does
 * its own wrap-pi on the angular delta, so unbounded heading is fine here.
 */
static void move_head(Snake *s, float dt, int cols, int rows)
{
    /* Step 1: advance the wave clock */
    s->wave_time += dt * s->speed_scale;

    /* Step 2: sinusoidal autonomous turn */
    float turn_wave = s->amplitude * sinf(s->frequency * s->wave_time);

    /* Step 3: edge-bias turn (zero unless within EDGE_MARGIN_PX of an edge) */
    float wpx       = (float)(cols * CELL_W);
    float hpx       = (float)(rows * CELL_H);
    float turn_edge = edge_bias_turn(s, wpx, hpx);

    s->heading += (turn_wave + turn_edge) * dt;

    /* Step 4: translate head in pixel space */
    s->joint[0].x += s->move_speed * cosf(s->heading) * dt;
    s->joint[0].y += s->move_speed * sinf(s->heading) * dt;

    /* Step 5: hard clamp — safety net for the highest speeds */
    if (s->joint[0].x < 0.0f) s->joint[0].x = 0.0f;
    if (s->joint[0].x > wpx)  s->joint[0].x = wpx;
    if (s->joint[0].y < 0.0f) s->joint[0].y = 0.0f;
    if (s->joint[0].y > hpx)  s->joint[0].y = hpx;

    /* Step 6: record this tick's head position in the trail */
    trail_push(s, s->joint[0]);
}

/* ── §5c  compute_joints ────────────────────────────────────────────── */

/*
 * compute_joints() — place all body joints by sampling the head's trail.
 *
 * joint[0] is already set by move_head() for this tick.
 * For each body joint i (1 … N_SEGS):
 *   target_dist = i × SEG_LEN_PX   (arc-length from head in pixel space)
 *   joint[i]    = trail_sample(target_dist)
 *
 * WHY THIS LOOKS PHYSICALLY CORRECT:
 *   Each body joint is literally at the position the head occupied some
 *   time in the past — specifically, when the accumulated distance since
 *   then equals i × SEG_LEN_PX.  Any curve the head carves (S-turn,
 *   spiral, straight line) propagates down the body exactly as it would
 *   in a real snake, because the body is following the actual recorded
 *   path, not an approximation derived from a formula.
 *
 *   No per-joint angle or rotation matrix is needed.  The trail buffer
 *   encodes all the geometry implicitly.
 *
 * RIGID SEGMENT LENGTHS:
 *   By using fixed multiples i × SEG_LEN_PX, all segments maintain the
 *   same arc length (SEG_LEN_PX = 18 px) in pixel space.  The segment
 *   "bends" at each joint because adjacent joints are on different points
 *   of the curved trail.  The bending angle at joint i is determined by
 *   how much the head's heading changed during the time it took to travel
 *   SEG_LEN_PX pixels — exactly the right physical relationship.
 */
static void compute_joints(Snake *s)
{
    for (int i = 1; i <= N_SEGS; i++) {
        s->joint[i] = trail_sample(s, (float)i * SEG_LEN_PX);
    }
}

/* ── §5d  rendering helpers ─────────────────────────────────────────── */

/*
 * seg_pair() — ncurses color pair index for body segment i.
 *
 * Maps i linearly from head (i=0, pair 1, yellow) to tail
 * (i=N_SEGS-1, pair N_PAIRS=7, blue):
 *
 *   pair = 1 + (i × (N_PAIRS − 1)) / (N_SEGS − 1)
 *
 * Integer arithmetic: for N_SEGS=32, N_PAIRS=7:
 *   i= 0 → 1 + (0×6)/31 = 1   (bright yellow)
 *   i= 5 → 1 + (5×6)/31 = 1   (still yellow, near head)
 *   i=10 → 1 + (10×6)/31 = 2  (orange)
 *   i=15 → 1 + (15×6)/31 = 3  (yellow-green, middle body)
 *   i=20 → 1 + (20×6)/31 = 4  (green)
 *   i=25 → 1 + (25×6)/31 = 5  (cyan)
 *   i=31 → 1 + (31×6)/31 = 7  (blue, tail tip)
 *
 * The gradient gives an instant visual reading of which end is the head.
 */
static int seg_pair(int i)
{
    return 1 + (i * (N_PAIRS - 1)) / (N_SEGS - 1);
}

/*
 * seg_attr() — ncurses attribute for body segment i.
 *
 * The front quarter of the body (segments close to the head) uses A_BOLD
 * for extra brightness — these segments have the most saturated colour and
 * the clearest direction glyphs, drawing the eye to the head.
 *
 * The rear quarter (near the tail) uses A_DIM to fade the already-cool
 * blue colour further, emphasising that it is the trailing end.
 *
 * Thresholds with N_SEGS = 32:
 *   i in [ 0,  7] → A_BOLD   (head quarter)
 *   i in [ 8, 23] → A_NORMAL (mid-body)
 *   i in [24, 31] → A_DIM    (tail quarter)
 */
static attr_t seg_attr(int i)
{
    if (i < N_SEGS / 4)       return A_BOLD;
    if (i > 3 * N_SEGS / 4)   return A_DIM;
    return A_NORMAL;
}

/*
 * joint_node_char() — bead marker glyph at joint position i.
 * Head third:  '0' — thick, prominent node.
 * Middle body: 'o' — standard bead.
 * Tail third:  '.' — small, receding.
 * This gradient of sizes visually reinforces the head→tail colour gradient.
 */
static chtype joint_node_char(int i)
{
    if (i <= (N_SEGS - 1) / 3)     return '0';
    if (i >= (N_SEGS - 1) * 2 / 3) return '.';
    return 'o';
}

/*
 * head_glyph() — directional arrow character for the snake's head.
 *
 * Maps heading (radians, unnormalised) to one of four ASCII arrows that
 * best indicate the travel direction.  The terminal y-axis is downward,
 * so heading = π/2 (south in pixel space) maps to 'v' (pointing down).
 *
 * The heading is first normalised to [0°, 360°) via while-loops (not
 * fmod, which can return negative values for negative inputs in C).
 *
 * QUADRANT MAP:
 *   [315°, 360°) ∪ [0°,  45°)  →  '>'  east
 *   [ 45°,       135°)           →  'v'  south (y downward)
 *   [135°,       225°)           →  '<'  west
 *   [225°,       315°)           →  '^'  north
 *
 * 45° boundaries give each arrow a full 90° coverage arc, cleanly
 * corresponding to the four screen quadrants.
 */
static chtype head_glyph(float heading)
{
    float deg = heading * (180.0f / (float)M_PI);
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;

    if (deg <  45.0f || deg >= 315.0f) return (chtype)'>';
    if (deg < 135.0f)                  return (chtype)'v';
    if (deg < 225.0f)                  return (chtype)'<';
    return                             (chtype)'^';
}

/* ── §5e  mark_cell — central glyph stamp helper ────────────────────── */

/*
 * mark_cell() — stamp one ASCII glyph at terminal cell (cx,cy).
 *
 * Centralises the (chtype)(unsigned char) cast, bounds-check, and the
 * wattron/wattroff sandwich that would otherwise be repeated at every
 * mvwaddch site.  The double cast prevents sign-extension on character
 * values > 127 (per CLAUDE.md "Common ncurses Bugs").  Glyph is silently
 * dropped if the cell is off-screen.
 */
static void mark_cell(WINDOW *w, int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/* ── §5f  draw_segment_beads ────────────────────────────────────────── */

/*
 * draw_segment_beads() — fill segment a→b with 'o' beads at DRAW_STEP_PX
 * intervals (pass 1 of the two-pass bead render).
 *
 * Walks the segment in pixel space; per-call dedup (prev_cx/prev_cy)
 * prevents a cell from being stamped twice, which would cause attribute
 * flicker.  Cells outside the screen are silently skipped by mark_cell.
 *
 * Pass 2 (in render_chain) over-stamps joint positions with '0'/'o'/'.'
 * node markers, giving the chain its articulated bead appearance.
 */
static void draw_segment_beads(WINDOW *w,
                                Vec2 a, Vec2 b,
                                int pair, attr_t attr,
                                int cols, int rows)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    int nsteps  = (int)ceilf(len / DRAW_STEP_PX) + 1;
    int prev_cx = -9999, prev_cy = -9999;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx;  prev_cy = cy;
        mark_cell(w, cx, cy, 'o', pair, attr, cols, rows);
    }
}

/* ── §5g  render_chain ──────────────────────────────────────────────── */

/*
 * lerp_joints() — fill rj[] with alpha-interpolated render positions.
 *
 * rj[i] = prev_joint[i] + (joint[i] − prev_joint[i]) · alpha
 *
 * alpha ∈ [0,1) is the leftover fraction in the fixed-step accumulator.
 * Without this lerp, motion stutters whenever render Hz ≠ sim Hz; with
 * it, the snake glides smoothly at any combination of the two rates.
 */
static void lerp_joints(const Snake *s, float alpha, Vec2 rj[N_SEGS + 1])
{
    for (int i = 0; i <= N_SEGS; i++) {
        rj[i].x = s->prev_joint[i].x
                + (s->joint[i].x - s->prev_joint[i].x) * alpha;
        rj[i].y = s->prev_joint[i].y
                + (s->joint[i].y - s->prev_joint[i].y) * alpha;
    }
}

/*
 * draw_body_fill() — bead-fill every segment, tail → head.
 *
 * Tail-first ordering ensures head-end segments over-stamp tail segments
 * on cells where two segments cross — the warmer head colour wins.
 */
static void draw_body_fill(WINDOW *w, const Vec2 rj[N_SEGS + 1],
                           int cols, int rows)
{
    for (int i = N_SEGS - 1; i >= 0; i--) {
        draw_segment_beads(w,
                           rj[i + 1], rj[i],
                           seg_pair(i), seg_attr(i),
                           cols, rows);
    }
}

/*
 * draw_body_nodes() — over-stamp graded node markers at every joint.
 *
 * Drawn tail → head so the warmest colour wins on overlap.  Each marker
 * uses its segment's pair (i−1), keeping the colour gradient continuous
 * across the body.
 */
static void draw_body_nodes(WINDOW *w, const Vec2 rj[N_SEGS + 1],
                            int cols, int rows)
{
    for (int i = N_SEGS; i >= 1; i--) {
        int cx = px_to_cell_x(rj[i].x);
        int cy = px_to_cell_y(rj[i].y);
        char ch = (char)joint_node_char(i);
        mark_cell(w, cx, cy, ch,
                  seg_pair(i - 1), seg_attr(i - 1),
                  cols, rows);
    }
}

/*
 * draw_head() — arrow glyph at joint[0], always on top.
 *
 * The arrow direction is computed from the current heading, NOT from
 * the lerp — the head's direction should not stutter when alpha = 0.
 */
static void draw_head(WINDOW *w, const Vec2 *head_pos, float heading,
                      int cols, int rows)
{
    int cx = px_to_cell_x(head_pos->x);
    int cy = px_to_cell_y(head_pos->y);
    char ch = (char)head_glyph(heading);
    mark_cell(w, cx, cy, ch, 1 /* PAIR_HEAD */, A_BOLD, cols, rows);
}

/*
 * render_chain() — orchestrate one frame in painter's order.
 *
 *   1. lerp_joints     — sub-tick interpolation for all 33 joints.
 *   2. draw_body_fill  — bead fill (bottom layer, tail → head).
 *   3. draw_body_nodes — graded node markers over-stamp the fill.
 *   4. draw_head       — directional arrow, always on top.
 */
static void render_chain(const Snake *s, WINDOW *w,
                          int cols, int rows, float alpha)
{
    Vec2 rj[N_SEGS + 1];
    lerp_joints(s, alpha, rj);

    draw_body_fill (w, rj, cols, rows);
    draw_body_nodes(w, rj, cols, rows);
    draw_head      (w, &rj[0], s->heading, cols, rows);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct { Snake snake; } Scene;

/*
 * scene_init() — initialise the snake to a clean, immediately-animated state.
 *
 * STARTING POSITION:
 *   Head placed at 38% from the left edge and 50% from the top of the
 *   screen in pixel space.  This gives room to swim rightward into view
 *   without immediately exiting the opposite edge.  Centred vertically.
 *
 * STARTING HEADING:
 *   π/8 ≈ 22.5° south-east.  A slight downward angle ensures the snake
 *   drifts toward the visual centre of most terminal sizes rather than
 *   immediately hitting the top or bottom edge.
 *
 * STARTING WAVE_TIME (mid-phase):
 *   wave_time is initialised to π/2 rather than 0.
 *   At wave_time = 0:  sin(frequency × 0) = 0 → turn rate = 0 → the
 *     snake swims straight for several seconds before the wave builds up.
 *   At wave_time = π/2: sin(frequency × π/2) ≈ sin(1.49) ≈ 1.0 →
 *     the sinusoidal turn rate starts at its peak, so the snake is already
 *     carving a visible curve on the very first frame.
 *
 * TRAIL PRE-POPULATION:
 *   At startup, trail_count = 0 and trail_sample() would return the oldest
 *   entry (just the head position) for every body joint → the snake would
 *   appear as a single point until enough ticks accumulate.
 *
 *   To avoid this, the trail is pre-filled with TRAIL_CAP positions that
 *   extend behind the head in the direction opposite to heading.  Each
 *   entry is spaced 1 px apart, matching the density the trail naturally
 *   builds at slow speeds.  trail_head = 0 is the newest entry (the head);
 *   trail[k] for increasing k goes further behind the head.
 *
 *   With TRAIL_CAP = 4096 entries at 1 px each, 4096 px of trail history
 *   is available from frame one — 7× the full snake body length (576 px).
 *   compute_joints() correctly samples this from frame one at any speed.
 *
 * prev_joint is set equal to joint after the first compute_joints() so
 * that the alpha lerp in render_chain() is a no-op on the first frame.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    int saved_theme = sc->snake.theme_idx;
    memset(sc, 0, sizeof *sc);
    Snake *s       = &sc->snake;
    s->theme_idx   = saved_theme;
    s->move_speed  = MOVE_SPEED_DEFAULT;
    s->amplitude   = AMPLITUDE_DEFAULT;
    s->frequency   = FREQUENCY_DEFAULT;
    s->speed_scale = SPEED_SCALE_DEFAULT;
    s->paused      = false;

    /* wave_time at π/2 = peak of sine → immediately curving on frame 1 */
    s->wave_time = (float)M_PI * 0.5f;

    /* Heading slightly south-east: drifts toward centre on most terminals */
    s->heading = (float)M_PI / 8.0f;

    /* Head at 38% from left, vertically centred — well inside the
     * EDGE_MARGIN_PX band so the snake starts swimming under wave-only
     * steering, with no edge bias kicking in until it actually nears a wall. */
    s->joint[0].x = (float)(cols * CELL_W) * 0.38f;
    s->joint[0].y = (float)(rows * CELL_H) * 0.50f;

    /*
     * Pre-populate trail: TRAIL_CAP entries, 1 px apart, extending behind
     * the head in the direction opposite to heading (i.e., heading + π).
     *
     * bx, by form a unit vector pointing AWAY from the initial heading.
     * trail[0] = newest = head; trail[k] = head + k × (bx, by).
     */
    float bx = cosf(s->heading + (float)M_PI);   /* unit vec backward */
    float by = sinf(s->heading + (float)M_PI);
    for (int k = 0; k < TRAIL_CAP; k++) {
        s->trail[k].x = s->joint[0].x + (float)k * bx;
        s->trail[k].y = s->joint[0].y + (float)k * by;
    }
    s->trail_head  = 0;       /* index 0 is the newest (= head) entry    */
    s->trail_count = TRAIL_CAP;

    compute_joints(s);
    memcpy(s->prev_joint, s->joint, sizeof s->joint);
}

/*
 * scene_tick() — one fixed-step physics update, called by §8 accumulator.
 *
 * dt is the fixed tick duration in seconds (= 1.0 / sim_fps).
 *
 * ORDER IS IMPORTANT:
 *   1. Save prev_joint[] FIRST — before any physics runs.
 *      This is the interpolation anchor for render_chain(); it must hold
 *      the state from the end of the PREVIOUS tick, not this one.
 *      Saving after move_head would produce a lerp that overshoots.
 *
 *   2. Return early if paused — prev_joint is saved regardless so the
 *      alpha lerp in render_chain() produces a clean freeze (prev = curr).
 *
 *   3. move_head() — advances wave_time, updates heading, translates
 *      joint[0], wraps it, pushes it into the trail.
 *
 *   4. compute_joints() — samples the now-updated trail to set joint[1..N].
 *      Must run AFTER move_head() so the body follows this tick's head.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    Snake *s = &sc->snake;
    memcpy(s->prev_joint, s->joint, sizeof s->joint);   /* Step 1 */
    if (s->paused) return;                               /* Step 2 */
    move_head(s, dt, cols, rows);                        /* Step 3 */
    compute_joints(s);                                   /* Step 4 */
}

/*
 * scene_draw() — render the scene; called once per render frame.
 * alpha ∈ [0, 1) is the sub-tick interpolation factor (see §5f).
 * dt_sec is unused here (no entity needs it for interpolation).
 */
static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)dt_sec;
    render_chain(&sc->snake, w, cols, rows, alpha);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the ncurses display layer.
 *
 * Holds the current terminal dimensions (cols, rows), read after each
 * resize.  See framework.c §7 for the full double-buffer architecture
 * rationale (erase → draw → wnoutrefresh → doupdate).
 */
typedef struct { int cols, rows; } Screen;

/*
 * screen_init() — configure the terminal for animation.
 *
 *   initscr()         initialise ncurses; must be first.
 *   noecho()          do not echo typed characters to the screen.
 *   cbreak()          pass keys immediately, without line buffering.
 *   curs_set(0)       hide the hardware cursor (no blinking cursor).
 *   nodelay(TRUE)     getch() returns ERR immediately if no key — makes
 *                     input polling non-blocking so the render loop
 *                     never stalls waiting for input.
 *   keypad(TRUE)      decode arrow keys and function keys into single
 *                     KEY_* constants rather than multi-byte sequences.
 *   typeahead(-1)     disable ncurses' habit of calling read() mid-output
 *                     to look for escape sequences; without this, terminal
 *                     output can be interrupted and frames arrive torn.
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
    color_init(0);
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
 * kernel and resize its internal virtual screens (curscr/newscr) to match
 * the new terminal dimensions.  Without this, stdscr retains the old size
 * and mvwaddch at coordinates in the newly valid area silently fails.
 *
 * Called from app_do_resize() which also clamps joint[0] to new bounds.
 */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw() — compose the full frame into stdscr (ncurses' newscr).
 *
 * Frame composition order (must not change):
 *   1. erase()        — write spaces over the entire newscr, erasing stale
 *                       content from the previous frame.  Does NOT write to
 *                       the terminal; only modifies the in-memory newscr.
 *   2. scene_draw()   — write snake body and head glyphs.
 *   3. HUD (top)      — status bar written last so it is always on top of
 *                       any snake glyph that might occupy the same row.
 *   4. Hint bar (bottom) — key reference line, also on top.
 *
 * Nothing reaches the terminal until screen_present() calls doupdate().
 *
 * HUD content: fps · sim Hz · move speed · wave amplitude · wave frequency ·
 *              speed scale · state (swimming / PAUSED).
 * Hint bar: brief keyboard reference for all controllable parameters.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    /* Top-right status — PAIR_HUD bright yellow, A_BOLD */
    const Snake *sn = &sc->snake;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%3d Hz  spd:%.0f  amp:%.2f  freq:%.1f  x%.2f  [%s]  %s ",
             fps, sim_fps,
             sn->move_speed,
             sn->amplitude,
             sn->frequency,
             sn->speed_scale,
             THEMES[sn->theme_idx].name,
             sn->paused ? "PAUSED " : "swimming");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom-left key hint — PAIR_HINT bright cyan, A_BOLD */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset  UD:spd  LR/ad:freq  ws:amp  +/-:wave-x  t/T:theme  [/]:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/*
 * screen_present() — flush the composed frame to the terminal in one write.
 *
 * wnoutrefresh(stdscr) copies the in-memory newscr model into ncurses'
 *   internal "pending update" structure.  No terminal I/O yet.
 * doupdate() diffs newscr against curscr (what is physically on screen),
 *   sends only the changed cells to the terminal fd, then sets curscr=newscr.
 *
 * This two-step sequence is the correct way to flush in ncurses.  Calling
 * refresh() (= wrefresh(stdscr) = wnoutrefresh + doupdate in one call)
 * is fine for a single window, but the two-step allows multiple windows to
 * be batched into one doupdate() for truly atomic multi-window renders.
 */
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level application state, accessible from signal handlers.
 *
 * Declared as a file-scope global (g_app) because POSIX signal handlers
 * receive no user-defined argument; they can only communicate through
 * globals.
 *
 * running and need_resize are volatile sig_atomic_t because:
 *   volatile     — prevents the compiler from caching the value in a
 *                  register across the signal-handler write.
 *   sig_atomic_t — the only integer type POSIX guarantees can be read
 *                  and written atomically from a signal handler on all
 *                  conforming implementations.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

/* Signal handlers — set flags only; no ncurses or malloc calls here */
static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/*
 * cleanup() — atexit safety net.
 * Registered with atexit() so that endwin() is always called even if the
 * program exits via an unhandled signal path that bypasses screen_free().
 */
static void cleanup(void) { endwin(); }

/*
 * app_do_resize() — handle a pending SIGWINCH terminal resize.
 *
 * screen_resize() re-reads LINES/COLS from the kernel.  If the terminal
 * was made smaller and joint[0] now falls outside the new pixel bounds,
 * it is clamped to just inside the boundary rather than re-centred — the
 * snake continues swimming from wherever it is rather than teleporting.
 *
 * frame_time and sim_accum are reset in the main loop after this returns
 * to prevent a physics avalanche from the large dt that would otherwise
 * accumulate during the resize operation.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Snake *s   = &app->scene.snake;
    float  wpx = (float)(app->screen.cols * CELL_W);
    float  hpx = (float)(app->screen.rows * CELL_H);
    if (s->joint[0].x >= wpx) s->joint[0].x = wpx - 1.0f;
    if (s->joint[0].y >= hpx) s->joint[0].y = hpx - 1.0f;
    app->need_resize = 0;
}

/*
 * app_handle_key() — process a single keypress; return false to quit.
 *
 * All keys adjust simulation parameters — there is no manual steering.
 * The snake's heading is driven entirely by the autonomous wave (§5b).
 *
 * KEY MAP:
 *   q / Q / ESC   quit
 *   space         toggle paused
 *   r / R         reset simulation (theme preserved)
 *   ↑  KEY_UP     move_speed × 1.20   [MOVE_SPEED_MIN, MAX]
 *   ↓  KEY_DOWN   move_speed ÷ 1.20
 *   ← / a / A     frequency − 0.1     [FREQUENCY_MIN, MAX]
 *   → / d / D     frequency + 0.1
 *   w / W         amplitude + 0.1     [AMPLITUDE_MIN, MAX]
 *   s / S         amplitude − 0.1
 *   + / =         speed_scale × 1.25  [0.05, 8.0]
 *   -             speed_scale ÷ 1.25
 *   t             next color theme (wraps 0..N_THEMES-1)
 *   T             previous color theme
 *   ]             sim_fps + step      [SIM_FPS_MIN, MAX]
 *   [             sim_fps − step
 */
static bool app_handle_key(App *app, int ch)
{
    Snake *s = &app->scene.snake;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': s->paused = !s->paused; break;

    case 'r': case 'R':
        scene_init(&app->scene, app->screen.cols, app->screen.rows);
        break;

    /* Move speed */
    case KEY_UP:
        s->move_speed *= 1.20f;
        if (s->move_speed > MOVE_SPEED_MAX) s->move_speed = MOVE_SPEED_MAX;
        break;
    case KEY_DOWN:
        s->move_speed /= 1.20f;
        if (s->move_speed < MOVE_SPEED_MIN) s->move_speed = MOVE_SPEED_MIN;
        break;

    /* Undulation frequency */
    case KEY_LEFT: case 'a': case 'A':
        s->frequency -= 0.1f;
        if (s->frequency < FREQUENCY_MIN) s->frequency = FREQUENCY_MIN;
        break;
    case KEY_RIGHT: case 'd': case 'D':
        s->frequency += 0.1f;
        if (s->frequency > FREQUENCY_MAX) s->frequency = FREQUENCY_MAX;
        break;

    /* Swim amplitude */
    case 'w': case 'W':
        s->amplitude += 0.1f;
        if (s->amplitude > AMPLITUDE_MAX) s->amplitude = AMPLITUDE_MAX;
        break;
    case 's': case 'S':
        s->amplitude -= 0.1f;
        if (s->amplitude < AMPLITUDE_MIN) s->amplitude = AMPLITUDE_MIN;
        break;

    /* Wave-time speed scale */
    case '=': case '+':
        s->speed_scale *= 1.25f;
        if (s->speed_scale > 8.0f) s->speed_scale = 8.0f;
        break;
    case '-':
        s->speed_scale /= 1.25f;
        if (s->speed_scale < 0.05f) s->speed_scale = 0.05f;
        break;

    /* Color themes */
    case 't':
        s->theme_idx = (s->theme_idx + 1) % N_THEMES;
        theme_apply(s->theme_idx);
        break;
    case 'T':
        s->theme_idx = (s->theme_idx + N_THEMES - 1) % N_THEMES;
        theme_apply(s->theme_idx);
        break;

    /* Simulation Hz */
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
 * The loop body executes these seven steps every frame:
 *
 *  ① RESIZE CHECK
 *     Handle a pending SIGWINCH before touching ncurses state.
 *     Reset frame_time and sim_accum so the large dt that accumulated
 *     during the resize does not inject a physics jump.
 *
 *  ② MEASURE dt
 *     Wall-clock nanoseconds since the previous frame start.
 *     Capped at 100 ms: if the process was suspended (Ctrl-Z, debugger)
 *     and resumed, an uncapped dt would fire hundreds of physics ticks
 *     in one frame — a physics avalanche that looks like a sudden jump.
 *
 *  ③ FIXED-STEP ACCUMULATOR
 *     sim_accum accumulates wall-clock dt each frame.
 *     While sim_accum ≥ tick_ns (one physics tick duration), fire one
 *     scene_tick() and drain tick_ns from sim_accum.
 *     Result: physics runs at exactly sim_fps Hz on average, regardless
 *     of how fast or slow the render loop runs.
 *
 *  ④ ALPHA — sub-tick interpolation factor
 *     After draining, sim_accum holds the fractional leftover — how far
 *     into the next unfired tick we are.
 *       alpha = sim_accum / tick_ns  ∈ [0, 1)
 *     Passed to render_chain() so joint positions are lerped between the
 *     last tick and the current tick, eliminating micro-stutter.
 *
 *  ⑤ FPS COUNTER
 *     Frames counted over a 500 ms sliding window.  Dividing the frame
 *     count by elapsed seconds gives a smoothed fps estimate.  This avoids
 *     per-frame division (which would oscillate wildly) and per-frame
 *     string formatting (which is slow).
 *
 *  ⑥ FRAME CAP — sleep BEFORE render
 *     elapsed = time spent on physics since frame_time was updated.
 *     budget  = NS_PER_SEC / 60  (one 60-fps frame).
 *     sleep   = budget − elapsed.
 *     Sleeping before terminal I/O means the I/O cost is not charged
 *     against the next frame's budget.  If sleep is negative (frame
 *     over-budget), clock_sleep_ns() returns immediately.
 *
 *  ⑦ DRAW + PRESENT
 *     erase() → scene_draw() → HUD → wnoutrefresh() → doupdate().
 *     One atomic diff write; no partial frames reach the terminal.
 *
 *  ⑧ DRAIN INPUT
 *     Loop getch() until ERR, processing every queued key event.
 *     Looping (not single-call) ensures all key-repeat events are
 *     consumed within the same frame they arrive, keeping parameter
 *     adjustments responsive when keys are held.
 * ───────────────────────────────────────────────────────────────────── */
int main(void)
{
    /* Seed RNG from monotonic clock so each run looks different */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));

    /* Safety net: endwin() even if we exit via an unhandled path */
    atexit(cleanup);

    /* SIGINT / SIGTERM — graceful exit from Ctrl-C or kill */
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);

    /* SIGWINCH — terminal resize; handled at the top of the next iteration */
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();   /* timestamp at start of last frame */
    int64_t sim_accum   = 0;            /* nanoseconds in the physics bucket */
    int64_t fps_accum   = 0;            /* ns elapsed in current fps window  */
    int     frame_count = 0;            /* frames rendered in fps window     */
    double  fps_display = 0.0;          /* smoothed fps shown in HUD         */

    while (app->running) {

        int64_t frame_start = clock_ns();

        /* ── ① resize ────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();   /* reset so dt doesn't spike         */
            sim_accum  = 0;
        }

        /* ── ② dt ────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;   /* suspend guard */

        /* ── ③ fixed-step accumulator ────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);   /* ns per physics tick   */
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── ④ alpha ─────────────────────────────────────────────── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── ⑤ fps counter ───────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── ⑥ frame cap — sleep before render ──────────────────── *
         * Budget = 1/60 s.  elapsed is wall time spent on physics +
         * accounting since frame_start; sleep the remainder so the
         * render rate sits at 60 fps regardless of sim Hz.            */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── ⑦ draw + present ────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
                    alpha, dt_sec);
        screen_present();

        /* ── ⑧ drain all pending input ──────────────────────────── */
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
