/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fk_medusa.c — A Bioluminescent Jellyfish Swimming in the Deep
 *
 * DEMO: A jellyfish (medusa) oscillates up and down through the terminal,
 *       pulsing its bell rhythmically.  8 trailing tentacles hang below the
 *       bell, computed via stateless Forward Kinematics.  As the bell rises,
 *       its tentacles trail downward and tilt (velocity-based trailing).  As
 *       the bell descends, they tilt back the opposite way.  The bell itself
 *       is rendered as a dynamic ASCII half-ellipse that pulses in size.
 *
 * STUDY ALONGSIDE: framework.c               (canonical loop / timing template)
 *                  fk_tentacle_forest.c       (stateless FK fundamentals)
 *                  snake_forward_kinematics.c (path-following FK contrast)
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────────
 *   §1  config       — all tunables in one place
 *   §2  clock        — monotonic clock + sleep (verbatim from framework)
 *   §3  color        — bioluminescent 7-step palette + dedicated HUD pair
 *   §4  coords       — pixel↔cell aspect-ratio helpers
 *   §5  entity       — stateless FK tentacle computation + bell rendering
 *       §5a  rendering helpers   — seg_pair, seg_attr, draw_segment_beads
 *       §5b  FK chain computation — compute_tentacle_fk (stateless)
 *       §5c  bell rendering      — draw_bell (dynamic half-ellipse)
 *   §6  scene        — Scene struct, scene_init / scene_tick / scene_draw
 *   §7  screen       — ncurses double-buffer display layer
 *   §8  app          — signals, resize, main game loop
 * ─────────────────────────────────────────────────────────────────────────
 *
 * HOW THE JELLYFISH WORKS
 * ───────────────────────
 *
 * BELL OSCILLATION — analytic sine + exact derivative:
 *
 *   bell_cy = bell_base_cy + BELL_AMP_PX × sin(bell_time × ω)
 *             where ω = 2π / BELL_PERIOD
 *
 *   This is a pure sinusoidal vertical oscillation; the bell center
 *   moves up and down in a smooth, continuous arc.  Rather than
 *   computing the velocity as a finite difference (bell_cy[t] − bell_cy[t-1])
 *   — which would be noisy at low sim Hz — we use the analytic derivative
 *   of the same sine:
 *
 *   bell_vy = BELL_AMP_PX × ω × cos(bell_time × ω)
 *
 *   This is exact at every tick regardless of dt.  At peak displacement
 *   (sin = ±1) the velocity is zero; at the equilibrium (sin = 0) the
 *   velocity is maximal (±BELL_AMP_PX × ω).
 *
 * VELOCITY-BASED TRAILING EFFECT — drag illusion without memory:
 *
 *   Tentacle roots are fixed around the bottom semicircle of the bell.
 *   The base heading for each tentacle chain is:
 *
 *     trailing_angle = clamp(bell_vy × TRAIL_FACTOR / 100, −MAX_TRAIL, MAX_TRAIL)
 *     base_heading   = π/2 + trailing_angle
 *
 *   π/2 is "straight down" in terminal pixel space (y increases downward).
 *   When bell_vy > 0 (bell moving down), tentacles tilt forward (leading
 *   the motion), as if pushed down by the water pressure.  When bell_vy < 0
 *   (bell moving up), trailing_angle flips sign and tentacles stream upward
 *   behind the rising bell — the classic jellyfish drag silhouette.
 *
 *   Crucially, this effect is STATELESS: it requires no stored history of
 *   previous positions.  The trailing angle is derived entirely from the
 *   current instantaneous velocity, which is itself derived analytically
 *   from the sine oscillator.  The result looks physically plausible because
 *   the tentacle angle is proportional to bell speed, just as hydrodynamic
 *   drag would produce in reality.
 *
 * STATELESS FK CHAIN — per-segment sinusoidal bend:
 *
 *   cumulative_angle = base_heading   (initialized from trailing calculation)
 *   joint[0] = root  (on bell bottom edge, translated with bell each tick)
 *
 *   for i in [0 … N_SEGS − 1]:
 *     δθᵢ = amplitude × sin(frequency × wave_time + root_phase[k] + i × PHASE_PER_SEG)
 *     cumulative_angle += δθᵢ
 *     joint[i+1] = joint[i] + seg_len_px × (cos(cumul_angle), sin(cumul_angle))
 *
 *   Each segment adds a small angular perturbation δθᵢ to the running
 *   angle.  The PHASE_PER_SEG offset advances the wave phase by 0.3 rad
 *   per segment, creating the appearance of a wave traveling down the
 *   tentacle toward the tip.  Because cumulative_angle is a sum, the tip
 *   segment's direction is the integral of all per-segment bends from
 *   root to tip — this is the FK accumulation property.
 *
 *   The entire chain is recomputed from scratch each tick from wave_time
 *   and root_phase alone.  No per-joint angle state is stored across ticks;
 *   prev_joint is saved only to enable sub-tick alpha interpolation at render.
 *
 * BELL RENDERING — dynamic half-ellipse per row, scaled by pulse:
 *
 *   For each integer row dy in [−ceil(BELL_HEIGHT_CELLS × pulse) … 0]:
 *
 *     t  = −dy / (BELL_HEIGHT_CELLS × pulse)   (0 at equator, 1 at crown)
 *     hw = BELL_RADIUS_X_CELLS × pulse × sqrt(1 − t²)
 *
 *   This is the standard ellipse equation: x²/a² + y²/b² = 1, solved for x.
 *   Multiplying by 'pulse' scales BOTH axes simultaneously, so the bell
 *   grows and shrinks proportionally rather than stretching in just one
 *   direction.  Left and right edge columns are stamped '(' and ')';
 *   the interior is filled with '-' (or '=' at the equator row where
 *   tentacles attach).
 *
 *   pulse = 0.85 + 0.15 × sin(pulse_phase × 3)
 *   The pulse_phase advances at 2× the simulation time, and the ×3
 *   multiplier makes the pulse frequency 6 rad/s in the phase domain ≈
 *   0.95 cycles/second — almost exactly one bell-pulse per oscillation
 *   cycle, giving the impression that contraction drives upward motion.
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   w / ↑      wave frequency + 0.15 rad/s  (faster tentacle undulation)
 *   s / ↓      wave frequency − 0.15 rad/s  (slower tentacle undulation)
 *   d / →      wave amplitude + 0.10 rad    (wider tentacle swing)
 *   a / ←      wave amplitude − 0.10 rad    (narrower tentacle swing)
 *   t / T      cycle colour theme forward / backward
 *   [ / ]      simulation Hz − / + 10
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       fk_medusa.c -o fk_medusa -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Stateless FK with velocity-based trailing.  The bell
 *                  center is driven by a sine oscillator.  Its instantaneous
 *                  velocity (bell_vy, the analytic derivative of the sine)
 *                  tilts the FK base heading for all tentacles, producing the
 *                  trailing-drag illusion without any stored history.  Each
 *                  tentacle chain is derived algebraically each tick from
 *                  wave_time; prev_joint is saved only for alpha lerp.
 *                  Contrast with snake_forward_kinematics.c which uses a
 *                  circular trail buffer (path-following FK, stateful).
 *
 * Data-structure : Scene holds bell kinematics (bell_cy, bell_vy, bell_time,
 *                  pulse_phase) and per-tentacle arrays: joint[N_TENTACLES]
 *                  [N_SEGS+1] and prev_joint[N_TENTACLES][N_SEGS+1], plus
 *                  root_phase[] and root_angle[] pre-computed in scene_init.
 *                  No trail buffer is needed; the formula is the entire state.
 *
 * Rendering      : Bell is drawn as a dynamic half-ellipse: for each row,
 *                  a half-width is computed from the ellipse equation scaled
 *                  by the pulse factor, then '(' / '-' / ')' are stamped.
 *                  Tentacles use draw_segment_beads() with a bioluminescent
 *                  palette (deep purple → bright cyan), then a second pass
 *                  stamps '0'/'o'/'.' node markers at each joint position.
 *
 * Performance    : O(N_TENTACLES × N_SEGS) per tick for FK, plus O(rows)
 *                  for bell rendering.  No trail buffer; fully cache-friendly.
 *                  At 60 Hz: 8 tentacles × 18 segments = 144 FK evaluations
 *                  per tick, each one sinf + cosf — negligible CPU cost.
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
     * The accumulator loop in §8 fires scene_tick() this many times per
     * wall-clock second, regardless of render frame rate.  Raising sim Hz
     * makes physics more accurate (smaller dt per step) but costs more CPU.
     *
     * The user-adjustable range is [SIM_FPS_MIN, SIM_FPS_MAX].
     * SIM_FPS_STEP is the amount added/removed per '[' / ']' keypress.
     *
     * Default 60 matches the 60-fps render cap so one physics tick fires
     * per rendered frame under normal load — the simplest relationship.
     */
    SIM_FPS_MIN     =  10,   /* Hz — slowest tick rate allowed              */
    SIM_FPS_DEFAULT =  60,   /* Hz — default (matches render cap)           */
    SIM_FPS_MAX     = 120,   /* Hz — fastest tick rate allowed              */
    SIM_FPS_STEP    =  10,   /* Hz step per '[' / ']' keypress              */

    /*
     * HUD_COLS — byte budget for the status bar string.
     * strlen(buf) must never exceed this; snprintf truncates safely.
     * 96 bytes covers the longest possible HUD string at maximum values
     * (theme name + all numeric fields).
     *
     * FPS_UPDATE_MS — how often the displayed fps value is recalculated.
     * 500 ms gives a stable reading without flickering on each frame.
     * Any shorter window makes the number oscillate faster than readable.
     */
    HUD_COLS      = 96,    /* bytes available for HUD snprintf buffer       */
    FPS_UPDATE_MS = 500,   /* ms between fps display updates                */

    /*
     * N_PAIRS — number of gradient color pairs for tentacle segments (§3).
     * Pairs 1..7 map root→tip from deep (dark) to bright (tip glows).
     *
     * HUD_PAIR = 8 is dedicated to both HUD bars so their color is
     * independent of the tentacle gradient and never accidentally changes
     * if N_PAIRS is adjusted.
     *
     * N_THEMES — number of selectable colour palettes (cycle with 't').
     */
    N_PAIRS  = 7,    /* gradient pairs 1..7, root→tip                       */
    HUD_PAIR = 8,    /* pair 8: exclusive to HUD bars                       */

    /*
     * N_TENTACLES — number of tentacle chains hanging from the bell.
     *   8 tentacles are evenly spaced around the bottom semicircle.
     *   The bottom semicircle spans angles [π, 2π] (in standard math coords),
     *   i.e., the half of the ellipse where sin(angle) ≥ 0, placing all roots
     *   below the bell center — correct anatomy for a jellyfish medusa.
     *
     * N_SEGS — number of rigid segments per tentacle chain.
     *   18 segments × SEG_LEN_PX (computed dynamically from screen height)
     *   typically reaches just past mid-screen on a standard 24-row terminal.
     *   More segments give smoother curvature but cost more FK evaluations.
     *
     * N_THEMES — colour palettes selectable at runtime with 't'.
     */
    N_TENTACLES = 8,    /* tentacle chains around the bell equator          */
    N_SEGS      = 18,   /* rigid segments per tentacle                      */
    N_THEMES    = 10,   /* number of colour themes in THEMES[]              */
};

/*
 * DRAW_STEP_PX — pixel step between bead fill samples along each segment.
 *
 * draw_segment_beads() walks each segment a→b in DRAW_STEP_PX increments,
 * converting each sample to a terminal cell and stamping 'o'.
 *
 * 5.0 px gives a visible gap between bead positions in cell space, making
 * the chain look articulated rather than a solid line.  It is less than
 * CELL_H (16 px) so no row is skipped even for near-vertical tentacles.
 *
 * Compare: snake_forward_kinematics.c uses 3.0 px (denser fill) because
 * snake segments are shorter (18 px) and the body must look solid.
 * Tentacles are longer and the sparse bead style is intentional.
 */
#define DRAW_STEP_PX  5.0f   /* px between bead samples along a segment    */

/*
 * PHASE_PER_SEG — phase advance per segment along the tentacle (radians).
 *
 * In the FK formula, the sine argument for segment i is:
 *   frequency × wave_time + root_phase[k] + i × PHASE_PER_SEG
 *
 * As i increases from 0 (root) to N_SEGS−1 (tip), the sine phase
 * advances by PHASE_PER_SEG per step.  With 0.3 rad ≈ 17° per segment
 * and N_SEGS=18, the total phase span across one tentacle is:
 *   17 × 0.3 = 5.1 rad ≈ 0.81 full sine cycles
 *
 * This creates roughly one peak-trough-peak wave along each tentacle,
 * giving the undulating seaweed appearance.  A larger value (e.g. 0.6)
 * would produce tighter waves with more peaks; 0.0 would make all
 * segments bend in the same direction simultaneously (boring).
 */
#define PHASE_PER_SEG  0.3f   /* rad per segment — wave travel speed        */

/*
 * AMP_DEFAULT — default peak angular bend per FK segment (radians).
 *   0.2 rad ≈ 11°.  Each segment turns 11° relative to its parent,
 *   and the bends accumulate: tip offset ≈ N_SEGS × AMP × seg_len_px,
 *   but the alternating sine means the tip rarely swings more than
 *   3–4 segment-lengths to either side.
 *
 * AMP_MIN — 0.0 rad: all segments point straight down, tentacles rigid.
 * AMP_MAX — 1.2 rad ≈ 69°: extreme whip-like cracking motion.
 *
 * FREQ_DEFAULT — default FK wave oscillation frequency (rad/s).
 *   1.2 rad/s gives period T = 2π/1.2 ≈ 5.2 s per full wave cycle.
 *   Slow enough to look fluid, fast enough to clearly show the undulation.
 *
 * FREQ_MIN / FREQ_MAX — keyboard-adjustable range.
 */
#define AMP_DEFAULT    0.2f   /* rad — default peak bend per segment        */
#define AMP_MIN        0.0f   /* rad — rigid straight tentacles             */
#define AMP_MAX        1.2f   /* rad — maximum whipping motion              */
#define FREQ_DEFAULT   1.2f   /* rad/s — default tentacle wave frequency    */
#define FREQ_MIN       0.1f   /* rad/s — very slow undulation               */
#define FREQ_MAX       6.0f   /* rad/s — rapid zigzag                       */

/*
 * BELL_PERIOD — duration of one complete vertical oscillation (seconds).
 *   4.0 s: the bell completes one full up-down-up cycle every 4 seconds.
 *   At 60 Hz this is 240 ticks per cycle — very smooth.
 *   Angular frequency: ω = 2π / 4.0 ≈ 1.57 rad/s.
 *
 * BELL_AMP_PX — vertical half-amplitude of the bell oscillation (pixels).
 *   40 px / CELL_H (16 px/row) = 2.5 terminal rows of vertical travel.
 *   Peak velocity: BELL_AMP_PX × ω ≈ 40 × 1.57 ≈ 62.8 px/s.
 *   This is the maximum bell_vy, used in the TRAIL_FACTOR formula.
 *
 * BELL_RADIUS_X — horizontal semi-axis of the bell ellipse (pixels).
 *   18 px / CELL_W (8 px/col) ≈ 2.25 columns of half-width.
 *   Total bell width at equator: ~4.5 columns — compact but visible.
 *
 * BELL_RADIUS_Y — vertical semi-axis of the bell ellipse (pixels).
 *   12 px / CELL_H (16 px/row) = 0.75 rows — the dome is less than
 *   one row tall.  This thin dome matches the flattened disc shape of
 *   a real medusa and keeps the bell compact at the top of the screen.
 *   Increasing to 32 px (2 rows) would give a rounder, more balloon-like bell.
 */
#define BELL_PERIOD    4.0f    /* s — vertical oscillation period            */
#define BELL_AMP_PX   40.0f   /* px — bell oscillation half-amplitude       */
#define BELL_RADIUS_X 18.0f   /* px — bell horizontal semi-axis             */
#define BELL_RADIUS_Y 12.0f   /* px — bell vertical semi-axis (dome height) */

/*
 * TRAIL_FACTOR — coefficient mapping bell velocity to tentacle tilt angle.
 *
 * The trailing angle for all tentacles is:
 *   trailing_angle = clamp(bell_vy × TRAIL_FACTOR / 100, −MAX_TRAIL, MAX_TRAIL)
 *
 * Why divide by 100?
 *   bell_vy is in pixels per second.  At maximum amplitude:
 *     |bell_vy_max| = BELL_AMP_PX × (2π / BELL_PERIOD) ≈ 62.8 px/s
 *   Without the /100, TRAIL_FACTOR × 62.8 ≈ 37.7 rad — far beyond MAX_TRAIL.
 *   Dividing by 100 scales the velocity into a useful angular range:
 *     62.8 × 0.6 / 100 ≈ 0.38 rad ≈ 22°
 *   A 22° tilt is noticeable but the tentacles still clearly point downward.
 *
 * MAX_TRAIL — maximum allowed tilt in radians.
 *   0.8 rad ≈ 46°.  Beyond this the tentacles start pointing sideways,
 *   which looks unnatural for a jellyfish.  The clamp is a safety rail.
 */
#define TRAIL_FACTOR  0.6f   /* velocity-to-angle coefficient               */
#define MAX_TRAIL     0.8f   /* rad — maximum tilt clamp (~46°)             */

/*
 * Timing primitives — verbatim from framework.c.
 *
 * NS_PER_SEC / NS_PER_MS: unit conversion constants.
 * TICK_NS(f): converts a frequency f (Hz) to the period in nanoseconds.
 *   e.g. TICK_NS(60) = 1 000 000 000 / 60 ≈ 16 666 667 ns per tick.
 * Used in the fixed-step accumulator (see §8 main()).
 */
#define NS_PER_SEC  1000000000LL   /* nanoseconds in one second             */
#define NS_PER_MS      1000000LL   /* nanoseconds in one millisecond        */
#define TICK_NS(f)  (NS_PER_SEC / (f))  /* ns per tick at frequency f Hz   */

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
 * signed and negative values (e.g., from the pause-guard cap) do not
 * wrap around to large positive numbers.
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
 * computation time is charged against the sleep budget; the terminal
 * write cost does not accumulate into the next frame's elapsed time.
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
/* §3  color — themed 7-step palette + dedicated HUD pair                 */
/* ===================================================================== */

/*
 * Theme — one named color palette.
 *
 * body[0..N_PAIRS-1]: xterm-256 foreground color indices for ncurses
 *   color pairs 1..7.  These map root→tip along each tentacle.
 *   The Medusa default runs deep purple (57) → bright cyan (159),
 *   mimicking bioluminescent glow that intensifies at the tentacle tips.
 *
 * hud: foreground color index for HUD_PAIR (pair 8).
 *   Kept separate so HUD color is always readable regardless of which
 *   tentacle theme is active.
 *
 * theme_apply() calls init_pair() live at runtime; switching themes
 * takes effect on the very next frame without restarting ncurses.
 */
typedef struct {
    const char *name;
    int body[N_PAIRS];   /* 7 entries: pairs 1..7, root→tip */
    int hud;             /* pair 8: HUD bar foreground       */
} Theme;

/*
 * THEMES[10] — ten selectable palettes.  Cycle forward with 't'.
 *
 * Colour index comments: head ← (root, dark) ──────────── (tip, bright) → tail
 *
 *  0 Medusa — deep purple → bright cyan (default bioluminescent)
 *  1 Matrix — dark olive → phosphor green
 *  2 Fire   — deep red → bright amber
 *  3 Ocean  — deep navy → bright aqua
 *  4 Nova   — dark violet → light pink
 *  5 Toxic  — dark olive → neon lime
 *  6 Lava   — deep maroon → amber orange
 *  7 Ghost  — dark grey → near-white
 *  8 Aurora — dark teal → warm gold
 *  9 Neon   — deep blue-violet → hot pink
 */
static const Theme THEMES[N_THEMES] = {
    /* name      root←────────────────────────────→tip    hud */
    { "Medusa", { 57, 63, 69, 75,  51, 123, 159}, 226 },
    { "Matrix", { 22, 28, 34, 40,  46,  82, 118},  46 },
    { "Fire",   { 52, 88,124,160, 196, 208, 226}, 208 },
    { "Ocean",  { 17, 18, 20, 27,  33,  45,  51},  51 },
    { "Nova",   { 54, 93,129,165, 201, 213, 225}, 213 },
    { "Toxic",  { 58, 64, 70, 76,  82, 118, 154}, 118 },
    { "Lava",   { 52, 58, 94,130, 166, 202, 208}, 202 },
    { "Ghost",  {234,238,242,246, 250, 254, 231}, 252 },
    { "Aurora", { 23, 29, 35, 71, 107, 143, 221}, 143 },
    { "Neon",   { 57, 93,129,165, 201, 199, 197}, 197 },
};

/*
 * theme_apply() — re-bind ncurses color pairs to the given theme at runtime.
 *
 * init_pair(n, fg, bg) sets the foreground/background for color pair n.
 * Pairs are re-initialized live; ncurses picks up the new palette on the
 * next draw cycle without any restart.
 *
 * If COLORS < 256 (8-color terminal), the 256-color indices are invalid
 * and init_pair would fail silently or produce garbage.  We skip the
 * 256-color block; color_init() handles the 8-color fallback separately.
 */
static void theme_apply(int idx)
{
    const Theme *t = &THEMES[idx];
    if (COLORS >= 256) {
        for (int p = 0; p < N_PAIRS; p++)
            init_pair(p + 1, t->body[p], COLOR_BLACK);
        init_pair(HUD_PAIR, t->hud, COLOR_BLACK);
    }
    /* 8-color fallback: pairs were already set in color_init; no change. */
}

/*
 * color_init() — one-time ncurses color setup; loads the initial theme.
 *
 * start_color()        enables color mode; must be called before init_pair().
 * use_default_colors() allows COLOR_BLACK background to be transparent
 *                      (inheriting the terminal's background color) rather
 *                      than a hard black fill.
 *
 * Bell always draws with pair 5 (the mid-gradient colour in any theme);
 * this is hardcoded in draw_bell() and not affected by theme changes.
 *
 * 8-color fallback: maps the 7 gradient pairs to blue/cyan/white so the
 * tentacle root-to-tip progression is still visible on limited terminals.
 */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        theme_apply(initial_theme);
    } else {
        /* 8-color terminal: approximate the deep-to-bright gradient */
        init_pair(1, COLOR_BLUE,   COLOR_BLACK);
        init_pair(2, COLOR_BLUE,   COLOR_BLACK);
        init_pair(3, COLOR_CYAN,   COLOR_BLACK);
        init_pair(4, COLOR_CYAN,   COLOR_BLACK);
        init_pair(5, COLOR_CYAN,   COLOR_BLACK);
        init_pair(6, COLOR_WHITE,  COLOR_BLACK);
        init_pair(7, COLOR_WHITE,  COLOR_BLACK);
        init_pair(8, COLOR_YELLOW, COLOR_BLACK);   /* HUD_PAIR */
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell; the one aspect-ratio fix                     */
/* ===================================================================== */

/*
 * WHY TWO COORDINATE SPACES
 * ─────────────────────────
 * Terminal cells are not square.  A typical cell is ~2× taller than wide
 * in physical pixels (CELL_H=16 vs CELL_W=8).  If jellyfish positions were
 * stored directly in cell coordinates, moving the bell by (dx, dy) cells
 * would travel twice as far vertically as horizontally in physical space.
 * The bell's circular oscillation would look like an oval squashed
 * sideways, and the tentacle curvature would look different depending on
 * whether you viewed the terminal portrait vs landscape.
 *
 * THE FIX — physics in pixel space, drawing in cell space:
 *   All Vec2 positions (joint[][], prev_joint[][]) are in pixel space
 *   where 1 unit = 1 physical pixel.  The pixel grid is square and
 *   isotropic: moving 1 unit in x or y covers the same physical distance.
 *   Only at draw time does §5a convert to cell coordinates.
 *
 * px_to_cell_x / px_to_cell_y — round to nearest cell.
 *
 * Formula:  cell = floor(px / CELL_DIM + 0.5)
 *   Adding 0.5 before flooring = "round half up" — deterministic and
 *   symmetric.  roundf() uses "round half to even" (banker's rounding),
 *   which can oscillate when px lands exactly on a cell boundary, causing
 *   a single-cell flicker.  Truncation ((int)(px/CELL_W)) always rounds
 *   down, giving asymmetric dwell at cell boundaries.  floor+0.5 avoids both.
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
/* §5  entity — stateless FK tentacles + bell rendering                  */
/* ===================================================================== */

/*
 * Vec2 — lightweight 2-D position vector in pixel space.
 * x increases eastward; y increases downward (terminal convention).
 * All joint positions and the bell center are stored as Vec2.
 */
typedef struct { float x, y; } Vec2;

/*
 * clampf() — clamp value v to the closed interval [lo, hi].
 *
 * Used by compute_tentacle_fk() to prevent the trailing tilt angle from
 * exceeding MAX_TRAIL, which would fold the tentacles sideways or back
 * over the bell.  A simple ternary chain — branchless on most compilers.
 */
static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ── §5a  rendering helpers ─────────────────────────────────────────── */

/*
 * seg_pair() — ncurses color pair index for tentacle segment i.
 *
 * Maps i linearly from root (i=0, pair 1, deep/dark) to tip
 * (i=N_SEGS-1, pair 7, bright/glowing):
 *
 *   pair = 1 + (i × (N_PAIRS − 1)) / (N_SEGS − 1)
 *
 * Integer arithmetic example for N_SEGS=18, N_PAIRS=7:
 *   i= 0 → 1 + (0 × 6)/17 = 1  (dark root, deep purple in Medusa theme)
 *   i= 3 → 1 + (3 × 6)/17 = 2  (slightly brighter)
 *   i= 8 → 1 + (8 × 6)/17 = 3  (mid-tentacle)
 *   i=12 → 1 + (12× 6)/17 = 5  (brighter towards tip)
 *   i=17 → 1 + (17× 6)/17 = 7  (bright tip, cyan glow in Medusa theme)
 *
 * This gradient mirrors bioluminescent tentacles: darker at the attachment
 * point (where pigment is denser), brightest at the stinging cell tips.
 *
 * Guard: N_SEGS ≤ 1 is degenerate; return pair 1 to avoid division-by-zero.
 */
static int seg_pair(int i)
{
    if (N_SEGS <= 1) return 1;
    return 1 + (i * (N_PAIRS - 1)) / (N_SEGS - 1);
}

/*
 * seg_attr() — ncurses attribute for tentacle segment i.
 *
 * Inverted compared to the snake: tentacle ROOTS are heavy/dim (like the
 * thick muscular base of a real tentacle) and TIPS are bright/bold (like
 * bioluminescent nematocysts flaring at the ends).
 *
 * Thresholds with N_SEGS = 18:
 *   i in [ 0,  4] → A_DIM    (root quarter: dark and heavy)
 *   i in [ 5, 13] → A_NORMAL (mid-body: standard brightness)
 *   i in [14, 17] → A_BOLD   (tip quarter: bioluminescent glow)
 */
static attr_t seg_attr(int i)
{
    if (i < N_SEGS / 4)       return A_DIM;    /* root: heavy, dark        */
    if (i > 3 * N_SEGS / 4)   return A_BOLD;   /* tip: glowing, bright     */
    return A_NORMAL;                             /* mid-body: neutral        */
}

/*
 * draw_segment_beads() — stamp 'o' beads along segment a→b at DRAW_STEP_PX
 * intervals (pass 1 of the two-pass render for one tentacle).
 *
 * ALGORITHM:
 *   1. Compute the vector (dx, dy) and length of the segment.
 *   2. Walk from a to b in nsteps evenly-spaced parameter steps u ∈ [0,1].
 *      nsteps = ceil(len / DRAW_STEP_PX) + 1 guarantees the step size in
 *      pixel space is at most DRAW_STEP_PX — no gap wider than one bead.
 *   3. At each u, convert pixel position to cell (cx, cy).
 *   4. Dedup: if (cx, cy) matches the previous sample, skip (same cell).
 *      This avoids double-stamping which can cause colour flicker in ncurses.
 *   5. Bounds check: skip cells outside [0,cols) × [1, rows-2].
 *      Row 0 and row rows-1 are reserved for HUD bars; never draw there.
 *   6. Stamp 'o' with the segment's gradient pair and attribute.
 *
 * Pass 2 (in scene_draw) overwrites joint positions with '0'/'o'/'.' node
 * markers, giving the chain its articulated bead-chain appearance.
 *
 * The dedup state (prev_cx, prev_cy) is shared across ALL segments of one
 * tentacle in scene_draw() — this prevents double-stamping at the boundary
 * between adjacent segments where joint[j+1] == joint[j+1].
 *
 * Parameters:
 *   prev_cx, prev_cy — dedup cursor, updated by this function (in/out).
 */
static void draw_segment_beads(WINDOW *w,
                                Vec2 a, Vec2 b,
                                int pair, attr_t attr,
                                int cols, int rows,
                                int *prev_cx, int *prev_cy)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;   /* degenerate segment: nothing to draw        */

    /* nsteps: enough steps that no cell is skipped; +1 for the endpoint   */
    int nsteps = (int)ceilf(len / DRAW_STEP_PX) + 1;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        /* Dedup: skip if we are still in the same cell as the last sample  */
        if (cx == *prev_cx && cy == *prev_cy) continue;
        *prev_cx = cx;
        *prev_cy = cy;

        /* Bounds: skip HUD rows (0 and rows-1) and off-screen cells        */
        if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1) continue;

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, (chtype)'o');
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/* ── §5b  FK chain computation ──────────────────────────────────────── */

/*
 * compute_tentacle_fk() — recompute all joint positions for one tentacle.
 *
 * This is the STATELESS FK formula.  Every joint position is derived
 * entirely from the current wave_time and the pre-computed root_phase;
 * no history of previous positions is needed or stored.
 *
 * STEP 1 — compute trailing tilt angle from bell velocity:
 *
 *   trailing_angle = clamp(bell_vy × TRAIL_FACTOR / 100, −MAX_TRAIL, MAX_TRAIL)
 *   base_heading   = π/2 + trailing_angle
 *
 *   π/2 rad is "straight down" in terminal pixel coordinates (y increases
 *   downward).  Adding trailing_angle tilts the whole chain:
 *     bell_vy > 0 (moving down) → trailing_angle > 0 → tilted forward
 *     bell_vy < 0 (moving up)   → trailing_angle < 0 → tilted backward
 *   This is the velocity-based trailing illusion (see file header).
 *
 * STEP 2 — set joint[0] to the given root position on the bell edge.
 *   The root is already in pixel space, translated with the bell each tick.
 *
 * STEP 3 — accumulate segments by integrating per-segment angle deltas:
 *
 *   cumulative_angle starts at base_heading (pointing down ± tilt).
 *   For each segment i:
 *     δθᵢ = amplitude × sin(frequency × wave_time + root_phase + i × PHASE_PER_SEG)
 *     cumulative_angle += δθᵢ
 *     joint[i+1] = joint[i] + seg_len_px × (cos(cumul), sin(cumul))
 *
 *   cumulative_angle is the running sum of all previous δθ values.
 *   It is the total heading direction at the END of segment i.  Using
 *   this cumulative angle is the FK accumulation property: each segment
 *   "inherits" the bending of all segments above it, so the tip's angle
 *   is the integral of all bends from root to tip.
 *
 *   PHASE_PER_SEG advances the sine argument by 0.3 rad per segment,
 *   creating the appearance of a traveling wave from root to tip.
 *
 * Parameters:
 *   joint[]      — write target array of N_SEGS+1 Vec2 positions
 *   root         — current root position on the bell bottom edge (px space)
 *   root_phase   — per-tentacle constant phase offset for desynchronisation
 *   wave_time    — accumulated simulation time driving the sine (seconds)
 *   amplitude    — peak angular bend per segment (radians)
 *   frequency    — wave oscillation angular frequency (rad/s)
 *   bell_vy      — instantaneous bell vertical velocity (px/s)
 *   seg_len_px   — rigid segment length (pixels), computed from screen height
 */
static void compute_tentacle_fk(Vec2 *joint,
                                 Vec2  root,
                                 float root_phase,
                                 float wave_time,
                                 float amplitude,
                                 float frequency,
                                 float bell_vy,
                                 float seg_len_px)
{
    /* Step 1: trailing tilt — scale bell_vy to an angular offset          */
    float trailing_angle = clampf(bell_vy * TRAIL_FACTOR / 100.0f,
                                   -MAX_TRAIL, MAX_TRAIL);
    float base_heading   = (float)M_PI * 0.5f + trailing_angle;

    /* Step 2: root joint at the bell edge; cumulative angle from base     */
    joint[0] = root;
    float cumulative_angle = base_heading;

    /* Step 3: accumulate segments root→tip                                */
    for (int i = 0; i < N_SEGS; i++) {
        float delta = amplitude
                    * sinf(frequency * wave_time
                           + root_phase
                           + (float)i * PHASE_PER_SEG);
        cumulative_angle += delta;

        joint[i + 1].x = joint[i].x + seg_len_px * cosf(cumulative_angle);
        joint[i + 1].y = joint[i].y + seg_len_px * sinf(cumulative_angle);
    }
}

/* ── §5c  bell rendering ────────────────────────────────────────────── */

/*
 * draw_bell() — render the jellyfish bell as a dynamic half-ellipse.
 *
 * A jellyfish bell is a dome (upper hemisphere of an ellipsoid).  We
 * approximate it as the upper half of a 2-D ellipse in the terminal.
 *
 * ELLIPSE EQUATION:
 *   For a semi-major axis a (horizontal) and semi-minor axis b (vertical):
 *   x²/a² + y²/b² = 1   →   x = a × sqrt(1 − y²/b²)
 *
 *   In our case:
 *     a = bell_rx_cells = (BELL_RADIUS_X / CELL_W) × pulse  (columns)
 *     b = bell_h_cells  = (BELL_RADIUS_Y / CELL_H) × pulse  (rows)
 *
 *   Both axes are multiplied by pulse, so the bell expands and contracts
 *   proportionally — a uniform scale, not a stretch.
 *
 * RENDERING LOOP:
 *   For each row dy from −ceil(b) to 0 (dome crown to equator):
 *     t  = −dy / b           (normalised height: 0 at equator, 1 at crown)
 *     hw = a × sqrt(1 − t²)  (half-width at this row, from ellipse equation)
 *     left  = bcx − round(hw)
 *     right = bcx + round(hw)
 *
 *   Characters stamped:
 *     '(' at left  edge column — bell outline curve, left side
 *     ')' at right edge column — bell outline curve, right side
 *     '-' at all interior columns — filled dome surface
 *     '=' at dy=0 (equator row) — the rim where tentacles attach
 *
 *   The bell is drawn with COLOR_PAIR(5) | A_BOLD.  Pair 5 is the
 *   middle of the gradient (cyan in Medusa, green in Matrix, etc.),
 *   which looks like a translucent bell glowing from within.
 *
 *   Rows outside [1, rows-2] are skipped (bounds clip; HUD rows reserved).
 *
 * Parameters:
 *   w                ncurses WINDOW to draw into
 *   bell_cx, bell_cy bell center in pixel space (changes each tick)
 *   pulse            current pulse scale factor (≈ 0.70 … 1.00)
 *   cols, rows       terminal dimensions for bounds checking
 */
static void draw_bell(WINDOW *w,
                      float bell_cx, float bell_cy,
                      float pulse,
                      int cols, int rows)
{
    /* Convert bell center from pixel space to cell space                  */
    int bcx = px_to_cell_x(bell_cx);
    int bcy = px_to_cell_y(bell_cy);

    /* Vertical semi-axis in cell units — how many rows the dome spans up  */
    float bell_h_cells   = (BELL_RADIUS_Y / (float)CELL_H) * pulse;
    /* Horizontal semi-axis in cell units at the equator (dy = 0)          */
    float bell_rx_cells  = (BELL_RADIUS_X / (float)CELL_W) * pulse;

    /* Render each row of the dome from crown (most negative dy) to equator */
    int dy_min = -(int)ceilf(bell_h_cells);
    for (int dy = dy_min; dy <= 0; dy++) {
        /* Normalised vertical position:
         *   t = 0 at equator (dy=0), t = 1 at the crown (dy = -b)         */
        float t = (bell_h_cells > 0.0f)
                ? (float)(-dy) / bell_h_cells
                : 0.0f;
        if (t > 1.0f) t = 1.0f;   /* clamp: float rounding can overshoot  */

        /* Ellipse half-width at this row                                   */
        float hw    = bell_rx_cells * sqrtf(1.0f - t * t);
        int   left  = bcx - (int)floorf(hw + 0.5f);
        int   right = bcx + (int)floorf(hw + 0.5f);
        int   row   = bcy + dy;   /* absolute row on screen                */

        /* Skip HUD rows and out-of-bounds rows                             */
        if (row < 1 || row >= rows - 1) continue;

        /* Equator row uses '=' to visually separate bell from tentacles    */
        chtype fill_ch = (dy == 0) ? (chtype)'=' : (chtype)'-';

        wattron(w, COLOR_PAIR(5) | A_BOLD);
        for (int c = left; c <= right; c++) {
            if (c < 0 || c >= cols) continue;
            chtype ch;
            if      (c == left)  ch = (chtype)'(';   /* left outline curve  */
            else if (c == right) ch = (chtype)')';   /* right outline curve */
            else                 ch = fill_ch;        /* interior fill       */
            mvwaddch(w, row, c, ch);
        }
        wattroff(w, COLOR_PAIR(5) | A_BOLD);
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene — complete simulation state for the medusa jellyfish.
 *
 * BELL KINEMATICS:
 *   bell_cx, bell_cy    Current bell center in pixel space.
 *                       bell_cx is constant (no horizontal motion).
 *                       bell_cy oscillates: = bell_base_cy + BELL_AMP_PX × sin(ωt).
 *   bell_base_cy        Resting (equilibrium) center y — vertical midpoint of screen.
 *                       Updated on resize; the sine oscillates symmetrically around it.
 *   bell_vy             Instantaneous vertical velocity of the bell (px/s).
 *                       Computed analytically as the derivative of bell_cy:
 *                       = BELL_AMP_PX × ω × cos(ω × bell_time).
 *                       Never stored across ticks — always recomputed from bell_time.
 *   bell_time           Accumulated real simulation time (seconds) driving bell motion.
 *   pulse_phase         Accumulated time (seconds) driving the bell size pulse.
 *                       Advances faster than bell_time (× 2.0) for higher pulse freq.
 *   pulse               Current pulse scale factor ∈ [0.70, 1.00].
 *                       = 0.85 + 0.15 × sin(pulse_phase × 3).
 *                       Applied to both BELL_RADIUS_X and BELL_RADIUS_Y uniformly.
 *
 * TENTACLE WAVE PARAMETERS:
 *   wave_time           Accumulated simulation time (seconds) driving tentacle FK.
 *                       Separate from bell_time so tentacle and bell phases are
 *                       independent — they can be out of sync at startup.
 *   amplitude           Peak FK bend per segment (radians).  Keyboard-adjustable.
 *   frequency           FK wave oscillation angular frequency (rad/s).  Keyboard.
 *   seg_len_px          Rigid segment length (pixels).  Computed dynamically in
 *                       scene_init() from screen height so tentacles reach mid-screen
 *                       regardless of terminal size.  Not keyboard-adjustable.
 *
 * TENTACLE STATE:
 *   joint[k][i]         Current pixel position of joint i on tentacle k.
 *                       joint[k][0] = root (on bell edge).
 *                       joint[k][N_SEGS] = tip (farthest from bell).
 *   prev_joint[k][i]    joint values at the START of the PREVIOUS tick.
 *                       Saved before FK recomputation each tick.
 *                       scene_draw() lerps between prev and curr by alpha
 *                       to produce sub-tick smooth motion at any render rate.
 *   root_phase[k]       Pre-computed per-tentacle phase offset (0 … 2π).
 *                       Makes each tentacle undulate independently, starting
 *                       at a different point in the wave cycle.
 *   root_angle[k]       Angle (radians) of tentacle k's attachment point
 *                       around the bell ellipse.  In [π, 2π] (bottom semicircle):
 *                       k=0 → π (leftmost), k=7 → 2π (rightmost).
 *                       sin(root_angle) ≥ 0 for all values in this range,
 *                       ensuring all roots are on the BOTTOM of the bell.
 *
 * CONTROL STATE:
 *   paused              When true, scene_tick() skips physics but still saves
 *                       prev_joint so the alpha lerp freezes the image cleanly.
 *   theme_idx           Index into THEMES[]; 't' key advances it cyclically.
 */
typedef struct {
    /* Bell kinematics */
    float bell_cx, bell_cy;   /* bell center, pixel space                  */
    float bell_base_cy;       /* oscillation equilibrium (screen mid-y)    */
    float bell_vy;            /* instantaneous vertical velocity, px/s     */
    float bell_time;          /* accumulator driving bell sine (seconds)   */
    float pulse_phase;        /* accumulator driving bell pulse (seconds)  */
    float pulse;              /* current size scale factor [0.70 … 1.00]   */

    /* Tentacle wave */
    float wave_time;          /* accumulator driving FK sine (seconds)     */
    float amplitude;          /* peak bend per segment (radians)           */
    float frequency;          /* wave oscillation frequency (rad/s)        */
    float seg_len_px;         /* segment length, px — set from screen h    */
    bool  paused;             /* physics frozen when true                  */

    /* Theme */
    int theme_idx;            /* index into THEMES[]; 't' advances it      */

    /* Tentacle state — all in pixel space */
    Vec2  joint[N_TENTACLES][N_SEGS + 1];       /* current positions       */
    Vec2  prev_joint[N_TENTACLES][N_SEGS + 1];  /* start-of-tick snapshot  */
    float root_phase[N_TENTACLES];               /* per-tentacle wave phase */
    float root_angle[N_TENTACLES];               /* attachment angle on bell*/
} Scene;

/*
 * scene_init() — initialise the medusa at screen centre.
 *
 * BELL POSITION:
 *   bell_cx = screen_width_px / 2      — horizontally centred; never moves.
 *   bell_base_cy = screen_height_px / 2 — vertical midpoint of oscillation.
 *   bell_cy = bell_base_cy at t=0 (sin(0) = 0, so it starts at equilibrium).
 *   bell_vy = 0 at t=0  (cos(0) = 1, so it is about to move downward).
 *
 * DYNAMIC SEGMENT LENGTH:
 *   seg_len_px = (rows × CELL_H × 0.55) / N_SEGS
 *   The 0.55 factor reserves 45% of screen height above the equator for
 *   the bell and the upward motion of the bell during oscillation.  Since
 *   tentacles trace arcs (not straight lines), a straight-line length of
 *   55% of screen height results in tentacles visually reaching ~40–50%
 *   of the screen height — approximately mid-screen.  This scales correctly
 *   to any terminal size from 10 rows to 50+ rows.
 *
 * TENTACLE ROOT DISTRIBUTION (bottom semicircle):
 *   root_angle[i] = π + i × π / (N_TENTACLES − 1)   for i = 0 … N_TENTACLES−1
 *
 *   i=0: angle = π  (leftmost, 9 o'clock position on clock face)
 *   i=7: angle = 2π (rightmost, 3 o'clock position — same as 0 after wrap)
 *
 *   All intermediate tentacles are evenly spaced across the 180° arc from
 *   left to right along the bottom of the ellipse.  sin(angle) ≥ 0 for
 *   all these angles, so all roots are below the bell center — correct
 *   for a medusa jellyfish (tentacles hang down from the bell rim).
 *
 * PHASE DISTRIBUTION:
 *   root_phase[i] = i × 2π / N_TENTACLES
 *   Evenly spaces 8 tentacles across the 360° phase circle so they are
 *   never all at the same wave state simultaneously.  This gives the
 *   ensemble an organic, flowing look — some bend left while others bend
 *   right — rather than all waving in unison.
 *
 * INITIAL JOINT POSITIONS:
 *   All joints are seeded to the bell center (sc->bell_cx, sc->bell_cy).
 *   compute_tentacle_fk() will fully overwrite them on the first scene_tick()
 *   call; the initial value only matters if scene_draw() is called before
 *   any scene_tick() (which doesn't happen in normal use).
 *   prev_joint is set equal to joint so the first-frame alpha lerp is a no-op.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);

    sc->amplitude   = AMP_DEFAULT;
    sc->frequency   = FREQ_DEFAULT;
    sc->wave_time   = 0.0f;
    sc->bell_time   = 0.0f;
    sc->pulse_phase = 0.0f;
    sc->pulse       = 1.0f;
    sc->paused      = false;
    sc->theme_idx   = 0;

    /* Dynamic segment length: 55% of screen height distributed across segs.
     * 0.55 factor: leaves room for bell motion and keeps tips near mid-screen.*/
    sc->seg_len_px = (float)(rows * CELL_H) * 0.55f / (float)N_SEGS;

    /* Bell starts at screen centre, velocity zero (sin(0)=0, bell at rest) */
    sc->bell_cx      = (float)(cols * CELL_W) * 0.5f;
    sc->bell_base_cy = (float)(rows * CELL_H) * 0.5f;
    sc->bell_cy      = sc->bell_base_cy;
    sc->bell_vy      = 0.0f;

    for (int i = 0; i < N_TENTACLES; i++) {
        /* Root angles: π (left) → 2π (right) across the bottom semicircle */
        sc->root_angle[i] = (float)M_PI
                           + (float)i * (float)M_PI / (float)(N_TENTACLES - 1);

        /* Phase offsets: evenly distribute 0 → 2π across all N_TENTACLES  */
        sc->root_phase[i] = (float)i * 2.0f * (float)M_PI / (float)N_TENTACLES;

        /* Seed all joints at bell centre; overwritten on first scene_tick() */
        for (int j = 0; j <= N_SEGS; j++) {
            sc->joint[i][j].x = sc->bell_cx;
            sc->joint[i][j].y = sc->bell_cy;
        }
        memcpy(sc->prev_joint[i], sc->joint[i],
               (N_SEGS + 1) * sizeof(Vec2));
    }
}

/*
 * scene_tick() — one fixed-step physics update; called by the §8 accumulator.
 *
 * dt is the fixed tick duration in seconds (= 1.0 / sim_fps).
 *
 * ORDER IS IMPORTANT — each step depends on the previous:
 *
 *   Step 1: Save prev_joint for ALL tentacles BEFORE overwriting joint[].
 *     This is the interpolation anchor used by scene_draw().  It must hold
 *     the state from the END of the previous tick.  Saving after any FK
 *     computation would collapse the lerp interval to zero and re-introduce
 *     the micro-stutter that alpha interpolation is designed to eliminate.
 *
 *   Step 2: Return early if paused.
 *     prev_joint was already saved, so the lerp in scene_draw() sees
 *     prev == curr and renders a clean frozen image.
 *
 *   Step 3: Advance bell_time and compute bell_cy + bell_vy.
 *     bell_cy  = bell_base_cy + BELL_AMP_PX × sin(omega × bell_time)
 *     bell_vy  = BELL_AMP_PX × omega × cos(omega × bell_time)
 *     Using the analytic derivative (cos) rather than a finite difference
 *     (bell_cy[t] − bell_cy[t-1] / dt) ensures bell_vy is exact and smooth
 *     at any sim Hz, with no noise amplification at low frame rates.
 *
 *   Step 4: Advance pulse_phase and compute the new pulse scale.
 *     pulse_phase advances at dt × 2.0 (2 time-units per second of real time).
 *     pulse = 0.85 + 0.15 × sin(pulse_phase × 3)
 *     Range: [0.85 − 0.15, 0.85 + 0.15] = [0.70, 1.00].
 *     The ×3 in the sine gives a frequency of 2 × 3 / (2π) ≈ 0.95 pulses/s,
 *     slightly less than one pulse per bell oscillation cycle (1/4 Hz),
 *     creating an organic, slightly-off-beat contraction rhythm.
 *
 *   Step 5: Advance wave_time (tentacle FK phase accumulator).
 *
 *   Step 6: For each tentacle, compute its current root position on the
 *     bell edge, then call compute_tentacle_fk() to derive all joint positions.
 *     Root position formula:
 *       root_px = bell_cx + BELL_RADIUS_X × cos(root_angle[i]) × pulse
 *       root_py = bell_cy + BELL_RADIUS_Y × sin(root_angle[i]) × pulse
 *     Scaling by pulse makes the tentacle attachment points move with the
 *     bell's size pulsation — they stay on the ellipse rim even as it shrinks.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    (void)cols; (void)rows;   /* dimensions unused; jellyfish stays centred */

    /* Step 1: snapshot joint positions before any FK recomputation */
    for (int i = 0; i < N_TENTACLES; i++) {
        memcpy(sc->prev_joint[i], sc->joint[i],
               (N_SEGS + 1) * sizeof(Vec2));
    }

    if (sc->paused) return;   /* Step 2: freeze cleanly */

    /* Step 3: bell oscillation — analytic position and velocity            */
    sc->bell_time += dt;
    float omega  = 2.0f * (float)M_PI / BELL_PERIOD;   /* rad/s            */
    sc->bell_cy  = sc->bell_base_cy
                 + BELL_AMP_PX * sinf(sc->bell_time * omega);
    sc->bell_vy  = BELL_AMP_PX * omega * cosf(sc->bell_time * omega);

    /* Step 4: bell size pulse — modulates both horizontal and vertical radii */
    sc->pulse_phase += dt * 2.0f;   /* advances at 2× real time             */
    sc->pulse = 0.85f + 0.15f * sinf(sc->pulse_phase * 3.0f);

    /* Step 5: tentacle wave accumulator */
    sc->wave_time += dt;

    /* Step 6: compute FK for each tentacle from current root position      */
    for (int i = 0; i < N_TENTACLES; i++) {
        /* Root is on the BOTTOM semicircle of the bell ellipse, scaled by pulse */
        Vec2 root;
        root.x = sc->bell_cx
               + BELL_RADIUS_X * cosf(sc->root_angle[i]) * sc->pulse;
        root.y = sc->bell_cy
               + BELL_RADIUS_Y * sinf(sc->root_angle[i]) * sc->pulse;

        compute_tentacle_fk(sc->joint[i],
                            root,
                            sc->root_phase[i],
                            sc->wave_time,
                            sc->amplitude,
                            sc->frequency,
                            sc->bell_vy,
                            sc->seg_len_px);
    }
}

/*
 * scene_draw() — render the full medusa frame into WINDOW *w.
 *
 * Called once per render frame by screen_draw().  This function ONLY reads
 * simulation state — it does not write to any Scene field.  All physics
 * mutation is confined to scene_tick().
 *
 * alpha ∈ [0, 1) is the sub-tick interpolation factor from the §8
 * accumulator.  For each joint:
 *   rj[i] = prev_joint[i] + (joint[i] − prev_joint[i]) × alpha
 * This blends between the last completed tick and the next (unfired) tick,
 * eliminating micro-stutter when the render rate differs from the sim rate.
 *
 * DRAW ORDER — painters algorithm, back to front:
 *
 *   Step 1: All tentacles (drawn first so bell appears in front of roots).
 *     For each tentacle i:
 *       Pass 1: fill each segment root→tip with 'o' beads via
 *               draw_segment_beads().  Root-to-tip draw order means brighter
 *               tip segments overwrite darker root segments on any overlap
 *               — enforcing the bioluminescent glow gradient visually.
 *       Pass 2: stamp joint node markers ('0' at root, 'o' mid-body, '.'
 *               near tips) at each rj[j] position, overwriting the fill
 *               beads at joint positions.  This gives the chain its
 *               articulated, segmented appearance distinct from the fill.
 *
 *   Step 2: Bell drawn on top, obscuring tentacle root ends as desired.
 *     The bell center is lerped: bell_cy is smooth because it changes
 *     continuously via the sine formula, so we use the current sc->bell_cy
 *     directly (the analytic rate already produces smooth motion).
 */
static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha)
{
    /* Step 1: draw all tentacles behind the bell */
    for (int i = 0; i < N_TENTACLES; i++) {
        /* Build alpha-interpolated render positions for tentacle i          */
        Vec2 rj[N_SEGS + 1];
        for (int j = 0; j <= N_SEGS; j++) {
            rj[j].x = sc->prev_joint[i][j].x
                    + (sc->joint[i][j].x - sc->prev_joint[i][j].x) * alpha;
            rj[j].y = sc->prev_joint[i][j].y
                    + (sc->joint[i][j].y - sc->prev_joint[i][j].y) * alpha;
        }

        /* Pass 1: bead fill, root→tip order (bright tips win on overlap)   */
        int prev_cx = -9999, prev_cy = -9999;   /* dedup state              */
        for (int j = 0; j < N_SEGS; j++) {
            draw_segment_beads(w,
                               rj[j], rj[j + 1],
                               seg_pair(j), seg_attr(j),
                               cols, rows,
                               &prev_cx, &prev_cy);
        }

        /* Pass 2: joint node markers stamped on top of the bead fill.
         * '0' root (heaviest node, attachment to bell),
         * 'o' mid-body (standard bead),
         * '.' near tip (smallest, almost-invisible trailing cell).           */
        for (int j = 0; j <= N_SEGS; j++) {
            int cx = px_to_cell_x(rj[j].x);
            int cy = px_to_cell_y(rj[j].y);
            if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1) continue;
            int p = seg_pair(j < N_SEGS ? j : N_SEGS - 1);

            chtype marker;
            if      (j == 0)            marker = (chtype)'0';   /* root      */
            else if (j <= N_SEGS / 3)   marker = (chtype)'0';   /* upper 3rd */
            else if (j <= 2*N_SEGS / 3) marker = (chtype)'o';   /* middle    */
            else                        marker = (chtype)'.';   /* lower tip */

            wattron(w, COLOR_PAIR(p) | A_BOLD);
            mvwaddch(w, cy, cx, marker);
            wattroff(w, COLOR_PAIR(p) | A_BOLD);
        }
    }

    /* Step 2: bell on top — covers tentacle roots, as a real bell would    */
    draw_bell(w, sc->bell_cx, sc->bell_cy, sc->pulse, cols, rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the ncurses display layer.
 *
 * Holds the current terminal dimensions (cols, rows), read after each
 * resize event.  The simulation physics uses cols/rows to compute pixel
 * bounds; screen_draw uses them for HUD placement and bounds checking.
 */
typedef struct { int cols, rows; } Screen;

/*
 * screen_init() — configure the terminal for full-screen animation.
 *
 *   initscr()         initialise ncurses internal structures; must be first.
 *   noecho()          do not echo typed characters to the screen — keys are
 *                     consumed by getch() silently.
 *   cbreak()          pass keys immediately without waiting for Enter; no
 *                     line buffering.  Required for real-time input.
 *   curs_set(0)       hide the hardware cursor; no blinking cursor artifact
 *                     in the top-left corner between frames.
 *   nodelay(TRUE)     make getch() non-blocking — return ERR immediately
 *                     if no key is queued, so the render loop never stalls.
 *   keypad(TRUE)      decode arrow keys and function keys (KEY_UP etc.)
 *                     into single integer constants instead of byte sequences.
 *   typeahead(-1)     disable ncurses' habit of calling read() between
 *                     output writes to look for input; without this, terminal
 *                     output can be interrupted mid-frame causing torn frames.
 *   color_init(0)     set up color pairs with the first theme (Medusa).
 *   getmaxyx()        read terminal dimensions into Screen.cols / Screen.rows.
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
 * endwin() re-enables line buffering, shows the cursor, disables keypad
 * mode, and restores the scroll region.  Always safe to call multiple times.
 */
static void screen_free(Screen *s) { (void)s; endwin(); }

/*
 * screen_resize() — handle a SIGWINCH (terminal resize) event.
 *
 * endwin() + refresh() forces ncurses to re-read LINES and COLS from the
 * kernel and resize its internal virtual screens (curscr/newscr) to match
 * the new terminal dimensions.
 *
 * Without this call, stdscr retains its pre-resize size.  Any mvwaddch()
 * targeting coordinates that are valid in the new (larger) terminal but
 * were invalid before the resize would silently fail, leaving gaps in
 * the frame.  Conversely, if the terminal shrinks, ncurses may attempt
 * to write past the new bottom-right — which some terminals render as
 * garbage at the edge.
 *
 * app_do_resize() calls this, then calls scene_init() to re-centre the
 * bell for the new dimensions.
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
 * Frame composition order (must not be changed):
 *
 *   1. erase()           — write spaces over the entire newscr, erasing
 *                          stale content from the previous frame.  Does NOT
 *                          write to the terminal; only modifies in-memory newscr.
 *   2. scene_draw()      — write tentacle and bell glyphs into newscr.
 *   3. HUD (top, row 0)  — status bar written last so it is always on top
 *                          of any glyph that might overlap row 0.
 *   4. Hint bar (bottom) — key reference on the last row, same reason.
 *
 * Nothing reaches the physical terminal until screen_present() calls doupdate().
 *
 * HUD content: fps · sim Hz · amplitude · frequency · pulse · theme · state.
 * Hint bar: one-line keyboard reference for all interactive controls.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha);

    /* Top HUD bar — right-aligned so it does not obscure the bell          */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %.1ffps  %dHz  amp:%.2f  freq:%.2f  pulse:%.2f  [%s]  %s ",
             fps, sim_fps,
             sc->amplitude, sc->frequency, sc->pulse,
             THEMES[sc->theme_idx].name,
             sc->paused ? "PAUSED" : "swimming");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);

    /* Bottom hint bar — left-aligned; truncated if terminal is very narrow */
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  w/s:freq  a/d:amp  t:theme  [/]:Hz ");
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);
}

/*
 * screen_present() — flush the composed frame to the terminal in one write.
 *
 * wnoutrefresh(stdscr) copies the in-memory newscr model into ncurses'
 *   internal "pending update" structure.  No terminal I/O yet.
 * doupdate() diffs newscr against curscr (what is currently on screen),
 *   generates the minimal escape sequence to update only changed cells,
 *   sends it to the terminal fd, then sets curscr = newscr.
 *
 * This two-step is the correct way to flush in ncurses.  Using refresh()
 * (which combines both calls) is fine for a single window, but the split
 * allows multiple windows to batch into a single doupdate() for atomic
 * multi-window renders — a habit worth keeping even in single-window code.
 */
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level application state, accessible from signal handlers.
 *
 * Declared as a file-scope global (g_app) because POSIX signal handlers
 * receive no user-defined argument; they can only communicate with the
 * main loop through global variables.
 *
 * running and need_resize are volatile sig_atomic_t because:
 *   volatile     — prevents the compiler from caching the value in a register
 *                  across the signal-handler write.  Without volatile, the
 *                  compiler may hoist the read outside the while loop,
 *                  producing an infinite loop that never sees the SIGTERM.
 *   sig_atomic_t — the only integer type POSIX guarantees can be written
 *                  atomically from a signal handler on all conforming
 *                  implementations.  On most platforms this is an int,
 *                  but using sig_atomic_t is required by the standard.
 */
typedef struct {
    Scene                 scene;        /* complete simulation state        */
    Screen                screen;       /* terminal dimensions              */
    int                   sim_fps;      /* physics tick rate (Hz)           */
    volatile sig_atomic_t running;      /* 0 when it is time to exit        */
    volatile sig_atomic_t need_resize;  /* 1 when SIGWINCH was received     */
} App;

static App g_app;   /* file-scope so signal handlers can reach it          */

/*
 * Signal handlers — write flags only; no ncurses, no malloc, no printf.
 * POSIX allows very few functions in signal handlers; flag-writes are safe.
 */
static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/*
 * cleanup() — atexit safety net.
 *
 * Registered with atexit() so endwin() is always called even if the
 * program exits through an unhandled signal path that bypasses the normal
 * screen_free() at the bottom of main().  Without this, a crash or an
 * unexpected signal can leave the terminal in raw mode (no echo, invisible
 * cursor, broken line wrapping) until the user manually types 'reset'.
 */
static void cleanup(void) { endwin(); }

/*
 * app_do_resize() — handle a pending SIGWINCH terminal resize event.
 *
 * Called at the top of the main loop whenever need_resize == 1.
 *
 * STRATEGY:
 *   Preserve all simulation time accumulators (wave_time, bell_time,
 *   pulse_phase) and the user-adjustable parameters (amplitude, frequency,
 *   theme) so animation continues smoothly from the same phase.  Then call
 *   scene_init() which re-computes the bell position for the new terminal
 *   dimensions (new bell_cx, bell_base_cy, seg_len_px) and reseeds joints.
 *   Restore the saved values afterward.
 *
 * The main loop resets frame_time and sim_accum immediately after this
 * returns, so the large dt that accumulated during the resize does not
 * cause a physics avalanche (hundreds of ticks firing at once to catch up).
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);

    /* Preserve simulation phase continuity across the resize */
    float saved_wave_time   = app->scene.wave_time;
    float saved_bell_time   = app->scene.bell_time;
    float saved_pulse_phase = app->scene.pulse_phase;
    float saved_amp         = app->scene.amplitude;
    float saved_freq        = app->scene.frequency;
    int   saved_theme       = app->scene.theme_idx;

    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    /* Restore: geometry reset but physics phase preserved               */
    app->scene.wave_time   = saved_wave_time;
    app->scene.bell_time   = saved_bell_time;
    app->scene.pulse_phase = saved_pulse_phase;
    app->scene.amplitude   = saved_amp;
    app->scene.frequency   = saved_freq;
    app->scene.theme_idx   = saved_theme;

    app->need_resize = 0;
}

/*
 * app_handle_key() — process a single keypress; return false to quit.
 *
 * Called for every key event drained from the ncurses input queue at the
 * bottom of the main loop.  Adjusts simulation parameters directly in the
 * Scene struct; changes take effect on the next scene_tick() call.
 *
 * KEY MAP:
 *   q / Q / ESC (27)   quit — return false to signal the main loop
 *   space              toggle sc->paused
 *   w / KEY_UP         frequency + 0.15 rad/s  [FREQ_MIN, FREQ_MAX]
 *   s / KEY_DOWN       frequency − 0.15 rad/s
 *   d / KEY_RIGHT      amplitude + 0.10 rad    [AMP_MIN, AMP_MAX]
 *   a / KEY_LEFT       amplitude − 0.10 rad
 *   t / T              cycle theme_idx forward (both t and T go forward)
 *   ]                  sim_fps + SIM_FPS_STEP   [SIM_FPS_MIN, SIM_FPS_MAX]
 *   [                  sim_fps − SIM_FPS_STEP
 *
 * Note: 'w'/'s' control tentacle FREQUENCY and 'a'/'d' control AMPLITUDE —
 * the opposite of the snake file, where w/s are amplitude and a/d frequency.
 * This was the original design choice in fk_medusa.c and is preserved here.
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ':
        sc->paused = !sc->paused;
        break;

    /* Tentacle wave frequency — how fast the undulation cycles             */
    case 'w': case KEY_UP:
        sc->frequency += 0.15f;
        if (sc->frequency > FREQ_MAX) sc->frequency = FREQ_MAX;
        break;
    case 's': case KEY_DOWN:
        sc->frequency -= 0.15f;
        if (sc->frequency < FREQ_MIN) sc->frequency = FREQ_MIN;
        break;

    /* Tentacle wave amplitude — how wide each segment swings               */
    case 'd': case KEY_RIGHT:
        sc->amplitude += 0.10f;
        if (sc->amplitude > AMP_MAX) sc->amplitude = AMP_MAX;
        break;
    case 'a': case KEY_LEFT:
        sc->amplitude -= 0.10f;
        if (sc->amplitude < AMP_MIN) sc->amplitude = AMP_MIN;
        break;

    /* Colour theme — both 't' and 'T' cycle forward (10 themes total)     */
    case 't': case 'T':
        sc->theme_idx = (sc->theme_idx + 1) % N_THEMES;
        theme_apply(sc->theme_idx);
        break;

    /* Simulation Hz — controls physics tick rate, not render rate          */
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
 * The loop body executes these eight steps every iteration:
 *
 *  ① RESIZE CHECK
 *     If SIGWINCH arrived, call app_do_resize() before touching ncurses.
 *     Reset frame_time and sim_accum so the dt that built up during the
 *     resize does not inject a burst of physics ticks (physics avalanche).
 *
 *  ② MEASURE dt
 *     Wall-clock nanoseconds since the previous frame start.
 *     Capped at 100 ms: if the process was suspended (Ctrl-Z, debugger)
 *     and resumed, an uncapped dt would fire ~6 seconds of physics in one
 *     frame at 60 Hz — a jump visible as a sudden phase discontinuity.
 *
 *  ③ FIXED-STEP ACCUMULATOR
 *     sim_accum accumulates wall-clock dt each frame.
 *     While sim_accum ≥ tick_ns (one physics tick duration), fire one
 *     scene_tick() and drain tick_ns from sim_accum.
 *     Result: physics runs at exactly sim_fps Hz on average, regardless
 *     of how fast or slow the render loop executes.
 *
 *  ④ ALPHA — sub-tick interpolation factor
 *     After draining complete ticks, sim_accum holds the fractional leftover —
 *     how far we are into the next unfired tick:
 *       alpha = sim_accum / tick_ns  ∈ [0, 1)
 *     scene_draw() lerps joint positions by alpha between prev and current,
 *     eliminating the micro-stutter that would otherwise occur when the
 *     render frame rate exceeds the physics tick rate.
 *
 *  ⑤ FPS COUNTER
 *     Frames counted over a 500 ms sliding window.  Dividing the frame
 *     count by elapsed seconds gives a smoothed fps estimate.  This avoids
 *     per-frame division (which would oscillate wildly for a single-frame
 *     window) and per-frame string formatting (slow).
 *
 *  ⑥ FRAME CAP — sleep BEFORE render
 *     elapsed = time spent on physics since frame_time was last updated.
 *     budget  = NS_PER_SEC / 60  (one 60-fps frame in nanoseconds).
 *     sleep   = budget − elapsed.
 *     Sleeping before terminal I/O means the ncurses write cost is not
 *     charged against the next frame's budget.  If sleep ≤ 0 (frame is
 *     over budget from heavy physics), clock_sleep_ns() returns immediately.
 *
 *  ⑦ DRAW + PRESENT
 *     erase() → scene_draw() → HUD overlay → wnoutrefresh() → doupdate().
 *     One atomic diff write; no partial frames reach the terminal.
 *
 *  ⑧ DRAIN INPUT
 *     Loop getch() until ERR, processing every queued key event in order.
 *     Looping (not single-call) ensures all key-repeat events are consumed
 *     within the same frame they arrive, keeping parameter adjustments
 *     responsive when a key is held down.
 * ───────────────────────────────────────────────────────────────────── */
int main(void)
{
    /* Safety net: endwin() is always called, even on abnormal exit         */
    atexit(cleanup);

    /* SIGINT / SIGTERM — graceful exit from Ctrl-C or kill                 */
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);

    /* SIGWINCH — terminal resize; handled at the top of the next iteration */
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();   /* timestamp at start of last frame  */
    int64_t sim_accum   = 0;            /* nanoseconds in the physics bucket  */
    int64_t fps_accum   = 0;            /* ns elapsed in current fps window   */
    int     frame_count = 0;            /* frames rendered in fps window      */
    double  fps_display = 0.0;          /* smoothed fps shown in HUD          */

    while (app->running) {

        /* ── ① resize ────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();   /* reset: skip dt from resize pause  */
            sim_accum  = 0;
        }

        /* ── ② dt ────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;   /* suspend guard */

        /* ── ③ fixed-step accumulator ────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);   /* ns per one physics tick */
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── ④ alpha ─────────────────────────────────────────────── */
        float alpha = (float)sim_accum / (float)tick_ns;   /* ∈ [0, 1)     */

        /* ── ⑤ fps counter ───────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── ⑥ frame cap — sleep before render ──────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);   /* target 60 fps display */

        /* ── ⑦ draw + present ────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha);
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
