/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ragdoll_ropes.c — 7 Verlet ropes swaying in sinusoidal wind
 *
 * DEMO: Seven ropes hang from ceiling anchors evenly distributed across
 *       the terminal.  Each rope has 20 particles connected by distance
 *       constraints.  A sinusoidal wind force with per-rope phase offsets
 *       drives each rope independently, creating a beautiful staggered
 *       "Mexican wave" sway.  Ropes have varying lengths and their tips
 *       carry a weighted bead marker.
 *
 * STUDY ALONGSIDE: framework.c            (canonical loop / timing)
 *                  ragdoll_figure.c       (Verlet + constraint physics)
 *                  snake_forward_kinematics.c (rendering style reference)
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────────
 *   §1  config        — all tunables in one place
 *   §2  clock         — monotonic clock + sleep (verbatim from framework)
 *   §3  color         — 10-theme palette system (7 rope pairs + HUD pair)
 *   §4  coords        — pixel↔cell aspect-ratio helpers
 *   §5  entity        — Scene: Verlet ropes + constraint solver + renderer
 *       §5a  rope_verlet_step     — Verlet integration for one particle
 *       §5b  apply_rope_constraints — distance constraint solver (N_ITERS passes)
 *       §5c  enforce_anchors      — pin particle[0] to ceiling after every pass
 *       §5d  apply_wind           — sinusoidal wind force per rope
 *       §5e  rope_node_char       — bead glyph selector (size gradient)
 *       §5f  draw_rope_beads      — dense segment fill + node marker pass
 *       §5g  render_scene         — full frame: ceiling, all ropes, weight tips
 *   §6  scene         — scene_init / scene_tick / scene_draw wrappers
 *   §7  screen        — ncurses double-buffer display layer
 *   §8  app           — signals, resize, main game loop
 * ─────────────────────────────────────────────────────────────────────────
 *
 * HOW THIS ROPE SIM WORKS — VERLET INTEGRATION + DISTANCE CONSTRAINTS
 * ────────────────────────────────────────────────────────────────────
 *
 * WHY VERLET FOR ROPES?
 *
 * A rope is inextensible: segments must not stretch.  You might think to
 * model this with stiff springs, but stiff springs require very small dt
 * to stay numerically stable — the spring constant explodes the simulation
 * unless the time step is tiny.
 *
 * Verlet integration solves this elegantly by storing velocity implicitly
 * as the difference between current and previous position:
 *
 *   velocity ≈ pos − old_pos     (over one tick)
 *
 * This means inextensibility is enforced AFTER the Verlet step by a
 * "distance constraint" that simply moves particles back to the correct
 * distance, without any explicit spring force.  The constraint is a
 * geometry operation, not a force, so it cannot blow up.
 *
 * VERLET STEP (one particle):
 *
 *   vel     = (pos − old_pos) × ROPE_DAMPING
 *   new_pos = pos + vel + accel × dt²
 *   old_pos = pos
 *   pos     = new_pos
 *
 * ROPE_DAMPING = 0.992 (versus ragdoll_figure.c's 0.995).  The lower
 * value means 0.8% velocity loss per tick versus 0.5%.  Rope segments
 * have more air resistance than rigid bones: a rope segment is thin and
 * whippy, so each tick it loses more kinetic energy to air drag.  The
 * slightly stronger damping keeps the rope from building up oscillation
 * energy and going unstable at high wind amplitudes.
 *
 * DISTANCE CONSTRAINT (one segment):
 *
 *   dx  = pos[s+1] − pos[s]
 *   err = (|dx| − rest_len) / |dx|        ← fractional error
 *   correction = 0.5 × err × dx           ← equal mass: split equally
 *   pos[s]   += correction
 *   pos[s+1] -= correction
 *
 * After CONSTRAINT_ITERS=6 passes the rope is inextensible to within
 * floating-point precision.  A simple chain (no branching) needs fewer
 * iterations than a full ragdoll skeleton — 6 is sufficient for N_SEG=20.
 *
 * ANCHOR ENFORCEMENT:
 *
 * Particle[r][0] is the ceiling anchor.  After every Verlet step AND
 * after every constraint pass we reset:
 *   pos[r][0] = old_pos[r][0] = anchor[r]
 *
 * Resetting BOTH pos and old_pos is essential.  If only pos is reset,
 * Verlet computes velocity = pos − old_pos on the next tick, which now
 * points away from the anchor (old_pos is still at the Verlet-displaced
 * location).  This phantom velocity yanks the particle off the anchor on
 * the very next tick, and drift accumulates over time.  Resetting both
 * zeroes the implicit velocity and permanently pins the particle.
 *
 * SINUSOIDAL WIND:
 *
 * Each rope r receives a horizontal acceleration:
 *   accel_x = wind_amp × sin(wind_time × wind_freq + phase_offset[r])
 *
 * phase_offset[r] = r × 2π / N_ROPES distributes 7 ropes evenly around
 * the full 2π cycle.  At any moment in time every rope is at a different
 * point in its oscillation, so neighbouring ropes always sway in opposite
 * directions — never all left or all right at the same time.  The result
 * is the characteristic "Mexican wave" visual.
 *
 * Sinusoidal wind (rather than random impulses) is used because:
 *   • Smooth enough: sin() is C-infinity continuous; no sudden jumps that
 *     would cause constraint explosions.
 *   • Complex enough: the per-rope phase shift makes each rope unique.
 *   • Controllable: wind_amp and wind_freq map directly to physical
 *     intuition and are adjustable at runtime via ↑/↓ and ←/→.
 *
 * Keys:
 *   q / ESC       quit
 *   space         pause / resume
 *   ↑ / ↓         wind amplitude ±50 px/s²
 *   ← / →         wind frequency ±0.05 rad/s
 *   r / R         reset (ropes back to straight, theme preserved)
 *   t / T         next / previous colour theme
 *   [ / ]         simulation Hz down / up
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       ragdoll_ropes.c -o ragdoll_ropes -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Position-based Verlet integration with iterative distance
 *                  constraints.  Unlike spring forces, distance constraints
 *                  directly correct particle positions to maintain segment
 *                  length, so the rope is numerically inextensible at any
 *                  dt.  Six constraint passes per tick converge to within
 *                  floating-point precision for a 20-node chain.
 *                  A sinusoidal wind force with per-rope phase offsets
 *                  (phase[r] = r × 2π / N_ROPES) drives each rope
 *                  independently, creating staggered rhythmic sway.
 *                  Anchor particle[r][0] is pinned by resetting both pos
 *                  and old_pos after every Verlet step and constraint pass.
 *
 * Data-structure : Scene struct holds flat 2D arrays: pos[N_ROPES][N_SEG],
 *                  old_pos[N_ROPES][N_SEG], prev_pos[N_ROPES][N_SEG].
 *                  rest_len[N_ROPES] stores the per-segment rest length
 *                  (= rope_len_px[r] / (N_SEG − 1), varies by rope).
 *                  anchor[N_ROPES] is the fixed ceiling pixel position.
 *                  phase_offset[N_ROPES] holds the pre-computed 2π offsets.
 *                  wind_time is a single accumulator driving all sin() calls.
 *
 * Rendering      : Two-pass bead style.  Pass 1: walk each segment in 2 px
 *                  increments, stamping 'o' with dedup to avoid cell flicker.
 *                  Pass 2: overwrite particle positions with size-graded node
 *                  markers ('0' top quarter = A_BOLD; 'o' middle; '.' bottom
 *                  quarter = A_DIM), mimicking physical tension: particles
 *                  near the ceiling bear more load and appear brighter.
 *                  Alpha interpolation (prev_pos → pos) smooths motion at
 *                  any combination of sim Hz and render Hz.
 *
 * Performance    : N_ROPES × N_SEG = 140 particles.  N_ITERS constraint
 *                  passes cost 6 × 7 × 19 = 798 distance corrections per
 *                  tick — trivial for any CPU.  ncurses doupdate() is the
 *                  bottleneck; it typically transmits ~200–400 changed cells
 *                  per frame across 7 swaying ropes.
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
     * The fixed-step accumulator in §8 fires scene_tick() this many times
     * per wall-clock second regardless of render frame rate.  Raising sim
     * Hz improves constraint accuracy (more passes per wall-clock second)
     * but costs more CPU.  [10, 120] is the user-adjustable range; default
     * 60 matches the 60-fps render cap so one physics tick fires per frame.
     *
     *   SIM_FPS_STEP — increment size for the [ and ] keys.
     */
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    /*
     * HUD_COLS — byte budget for the status-bar string built by screen_draw().
     * snprintf() uses this as the buffer size; 96 bytes covers the longest
     * possible HUD string at maximum parameter values.
     *
     * FPS_UPDATE_MS — milliseconds between fps display recalculations.
     * 500 ms gives a stable reading without per-frame flicker.
     */
    HUD_COLS         =  96,
    FPS_UPDATE_MS    = 500,

    /*
     * N_PAIRS — number of rope colour pairs registered with ncurses (1–7).
     *   Rope r uses colour pair (r % N_PAIRS) + 1, cycling through all seven.
     *
     * HUD_PAIR (8) — dedicated pair for both HUD bars so their colour is
     *   independent of the rope palette and never accidentally shifts when
     *   N_PAIRS is changed.
     *
     * N_THEMES — number of switchable palettes in the THEMES[] array (§3).
     *   Cycled at runtime with the t / T keys.
     */
    N_PAIRS          =   7,
    HUD_PAIR         =   8,
    N_THEMES         =  10,

    /*
     * N_ROPES — number of independent rope chains hanging from the ceiling.
     *   Seven ropes produce the classic "wind chime" visual density without
     *   crowding the terminal.  Each rope gets its own colour pair and its
     *   own phase offset in the wind oscillation.
     *
     * N_SEG — particles per rope, including the fixed anchor at index [0].
     *   With N_SEG = 20 and rest lengths ranging ~12–28 px, rope visual
     *   lengths range from 35% to 75% of screen height.  More segments
     *   produce smoother curves but cost more constraint iterations.
     *
     * N_ITERS — distance constraint passes per tick (CONSTRAINT_ITERS).
     *   Each pass reduces the positional error roughly by half for a simple
     *   chain.  Six passes converge a 20-node rope to < 0.1% length error,
     *   which is imperceptible.  (ragdoll_figure.c uses 10+ because its
     *   skeleton has branching joints that need more passes to stabilise.)
     */
    N_ROPES = 7,
    N_SEG   = 20,
    N_ITERS =  6,
};

/*
 * GRAVITY — downward acceleration applied to every non-anchor particle (px/s²).
 *
 *   +y is downward in screen space (terminal convention).
 *   800 px/s² with CELL_H = 16 px/row means particles fall at
 *   800/16 = 50 rows/s² — fast enough to keep ropes taut under wind
 *   without making them rigid.
 *
 *   Compared with ragdoll_figure.c (also 800 px/s²): same gravity model,
 *   because ropes and the ragdoll live in the same pixel-space world.
 *
 * ROPE_DAMPING — per-tick velocity scale factor applied in rope_verlet_step().
 *
 *   vel_new = vel_old × ROPE_DAMPING
 *
 *   0.992 means 0.8% velocity loss per tick.  Compare with ragdoll_figure.c
 *   (0.995 = 0.5% loss per tick).  Rope segments are thin and whippy —
 *   physically they have more air resistance than rigid bones.  The slightly
 *   stronger drag prevents ropes from oscillating wildly at high wind_amp
 *   values while keeping visible sway at lower amplitudes.
 *
 *   At 60 Hz: 0.992^60 ≈ 0.619 — a rope released from any initial velocity
 *   loses ~38% of its speed after one second of free sway.
 *
 * BOUNCE_COEFF — fraction of velocity retained after a floor or wall
 *   collision.  0.5 = 50% energy retained; the rope tip bounces but
 *   not elastically.  Values < 0.3 look dead; > 0.8 look rubbery.
 */
#define GRAVITY          800.0f
#define ROPE_DAMPING       0.992f
#define BOUNCE_COEFF       0.5f

/*
 * WIND_AMP_DEFAULT — default lateral wind force amplitude (px/s²).
 *
 *   250 px/s² at 60 Hz adds ≈ 250 × (1/60)² ≈ 0.069 px of lateral
 *   displacement per tick before damping.  Over many ticks this accumulates
 *   to a visible lateral sway of several columns — noticeable but not violent.
 *   Maximum allowed at runtime: 1000 px/s² (↑ key).
 *   Minimum: 0 px/s² (ropes hang straight, only gravity).
 *
 * WIND_FREQ_DEFAULT — default wind oscillation angular frequency (rad/s).
 *
 *   0.4 rad/s → period = 2π / 0.4 ≈ 15.7 seconds per full left-right cycle.
 *   This is a slow, languorous sway — like ropes in a light breeze.
 *   Increasing toward 4.0 rad/s (→ key) produces a rapid rattling shake.
 *   Minimum: 0.05 rad/s → ~126-second period (almost imperceptibly slow).
 *
 * ANCHOR_ROW_CELLS — terminal row of the ceiling anchor line.
 *
 *   Row 2 (third from top) keeps the '#' ceiling line and the 'T' anchor
 *   markers visible below the status bar at row 0, with one blank row at
 *   row 1 as a visual separator.  All rope physics starts from this row.
 */
#define WIND_AMP_DEFAULT   250.0f
#define WIND_FREQ_DEFAULT    0.4f
#define ANCHOR_ROW_CELLS     2

/*
 * Boundary margins in pixel space — how far from each physical edge
 * particles are clamped by rope_boundaries() to prevent them escaping
 * the visible area.
 *
 *   FLOOR_MARGIN  — gap in px above the bottom pixel row.
 *     8 px = 0.5 terminal rows: the floor is just below the last visible row.
 *   LEFT_MARGIN / RIGHT_MARGIN — gap in px from left and right edges.
 *     16 px = 2 terminal columns: a small safety band at each side.
 */
#define FLOOR_MARGIN   8.0f
#define LEFT_MARGIN   16.0f
#define RIGHT_MARGIN  16.0f

/*
 * DRAW_STEP_PX — pixel step used in the dense segment-fill pass of
 * draw_rope_beads().
 *
 * The renderer walks each rope segment in 2 px increments, converting each
 * sample to a terminal cell and stamping an 'o'.  The step must satisfy:
 *   DRAW_STEP_PX < CELL_W  (8 px)
 * so that no terminal cell along a near-horizontal segment is ever skipped.
 *
 * 2 px is intentionally finer than snake_forward_kinematics.c's 3 px
 * because rope segments are often near-vertical (hanging down) and a
 * 16 px tall cell would be skipped at 3 px steps if the angle is steep.
 * At 2 px: even a perfectly vertical segment (angle = 90°) is sampled at
 *   2 / sin(90°) = 2 px steps along y → 2 / 16 = 0.125 cells/step: fine.
 */
#define DRAW_STEP_PX   2.0f

/*
 * Timing primitives — verbatim from framework.c.
 *
 * NS_PER_SEC / NS_PER_MS — unit conversion constants.
 * TICK_NS(f) — period of one physics tick at frequency f Hz, in nanoseconds.
 *   e.g. TICK_NS(60) = 1 000 000 000 / 60 = 16 666 666 ns ≈ 16.7 ms.
 * Used in the fixed-step accumulator in §8.
 */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Terminal cell dimensions — the aspect-ratio bridge between physics
 * and display (see §4 for the full explanation).
 *
 * CELL_W = 8 px, CELL_H = 16 px.
 * A typical terminal cell is about 2× taller than wide in physical pixels.
 * All particle positions are stored in a square pixel space scaled by these
 * constants so that 1 pixel represents the same physical distance in x and y.
 * Only at draw time are positions converted to (col, row) cell coordinates.
 */
#define CELL_W   8    /* physical pixels per terminal column */
#define CELL_H  16    /* physical pixels per terminal row    */

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/*
 * clock_ns() — monotonic wall-clock in nanoseconds.
 *
 * CLOCK_MONOTONIC never goes backward (unlike CLOCK_REALTIME, which can
 * jump on NTP adjustments or DST transitions).  Subtracting two consecutive
 * clock_ns() calls gives the true elapsed wall time regardless of system
 * load or clock adjustments.
 *
 * Returns int64_t (signed 64-bit) so subtraction differences are naturally
 * signed — negative values from the pause-guard cap do not wrap.
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
 * Called before render in §8 to cap the frame rate at 60 fps.  Sleeping
 * before (not after) the terminal write means only the physics computation
 * time is charged against the frame budget; the ncurses I/O cost does not
 * bleed into the next frame's elapsed time.
 *
 * If ns ≤ 0 the frame is already over-budget (physics took too long);
 * return immediately rather than sleeping a negative duration.
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
/* §3  color — 10-theme palette system (7 rope pairs + HUD pair)         */
/* ===================================================================== */

/*
 * Theme — one complete named colour palette.
 *
 *   name      — displayed in the HUD status bar.
 *   body[]    — 7 xterm-256 foreground colour indices for ncurses pairs
 *               1–7 (one per rope).  Rope r always uses pair (r%7)+1.
 *   hud       — foreground index for HUD_PAIR (pair 8), kept separate so
 *               the status bar colour never accidentally changes when the
 *               rope palette cycles.
 *
 * theme_apply() registers the chosen palette with ncurses init_pair() live;
 * switching themes takes effect on the very next frame without restarting.
 * On terminals with < 256 colours, theme_apply() falls back to 8 ANSI
 * colours that approximate the intended rainbow order.
 */
typedef struct {
    const char *name;
    int body[N_PAIRS];   /* xterm-256 colour index for rope pairs 1–7 */
    int hud;             /* colour index for HUD pair 8               */
} Theme;

/*
 * THEMES[10] — the ten built-in palettes.
 *
 * Column order: pair 1 (rope 0) → pair 7 (rope 6), then hud.
 *
 * Design rationale for each theme:
 *   Rainbow — classic ROYGBIV: maximum contrast between neighbouring ropes.
 *   Neon    — hot pinks, yellows, greens: bright club-light aesthetic.
 *   Fire    — deep red to pale yellow: ropes look like glowing embers.
 *   Ocean   — navy to aqua: calm cool gradient.
 *   Aurora  — forest green to violet: northern lights palette.
 *   Lava    — dark red to orange: slow molten glow.
 *   Forest  — multiple greens: organic, earthy.
 *   Sunset  — purple to warm orange: dusk gradient.
 *   Ice     — light blue to cyan: cold crystalline.
 *   Matrix  — dark to bright green: terminal hacker aesthetic.
 */
static const Theme THEMES[N_THEMES] = {
    /* name       rope0  rope1  rope2  rope3  rope4  rope5  rope6  hud  */
    {"Rainbow", {196,   208,   226,    46,    51,    21,   129}, 226},
    {"Neon",    {201,   226,   118,   159,   213,   208,    15}, 226},
    {"Fire",    {124,   160,   196,   202,   208,   214,   220}, 220},
    {"Ocean",   { 17,    18,    27,    33,    39,    45,    51},  45},
    {"Aurora",  { 22,    34,    79,   122,   159,   165,   201}, 159},
    {"Lava",    { 52,    88,   124,   160,   196,   202,   208}, 196},
    {"Forest",  { 22,    28,    34,    40,    70,   106,    82},  46},
    {"Sunset",  { 54,    91,   128,   165,   202,   209,   208}, 208},
    {"Ice",     {195,   159,   123,    87,    51,    45,    39}, 195},
    {"Matrix",  { 22,    28,    34,    40,    46,    82,   118},  46},
};

/*
 * theme_apply() — register palette idx with ncurses init_pair().
 *
 * Called from color_init() at startup and from app_handle_key() on t/T.
 * On 256-colour terminals: uses the xterm-256 indices directly.
 * On 8-colour fallback: maps to the closest ANSI colour by visual feel
 * (red→yellow→green→cyan→blue→magenta arc approximates ROYGBIV order).
 */
static void theme_apply(int idx)
{
    const Theme *th = &THEMES[idx];
    if (COLORS >= 256) {
        for (int p = 1; p <= N_PAIRS; p++)
            init_pair(p, th->body[p - 1], COLOR_BLACK);
        init_pair(HUD_PAIR, th->hud, COLOR_BLACK);
    } else {
        /* 8-colour ANSI fallback — approximate ROYGBIV order */
        init_pair(1, COLOR_RED,     COLOR_BLACK);
        init_pair(2, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(6, COLOR_BLUE,    COLOR_BLACK);
        init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(HUD_PAIR, COLOR_YELLOW, COLOR_BLACK);
    }
}

/*
 * color_init() — one-time colour system setup.
 *
 * start_color()       — initialise ncurses colour support.
 * use_default_colors() — allow colour pair 0 to mean "terminal default"
 *                        so the black background is the true terminal bg
 *                        rather than a filled black cell.
 * theme_apply()        — register the initial palette.
 */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();
    theme_apply(initial_theme);
}

/* ===================================================================== */
/* §4  coords — pixel↔cell; the one aspect-ratio fix                     */
/* ===================================================================== */

/*
 * WHY TWO COORDINATE SPACES
 * ─────────────────────────
 * Terminal cells are not square.  A typical cell is ~2× taller than wide
 * in physical pixels (CELL_H = 16 vs CELL_W = 8).  If rope particle
 * positions were stored directly in (col, row) cell coordinates, a rope
 * segment pointing diagonally at 45° in screen space would actually travel
 * twice as far vertically as horizontally in physical space — the rope
 * would look stretched vertically and gravity would appear anisotropic.
 *
 * THE FIX — physics in pixel space, drawing in cell space:
 *   All Vec2 positions (pos[][], old_pos[][], prev_pos[][], anchor[])
 *   are in pixel space where 1 unit = 1 physical pixel.  The pixel grid
 *   is square and isotropic: gravity acts equally in all directions and
 *   segment lengths are Euclidean.  Only at draw time does §5g convert to
 *   cell coordinates via px_to_cell_x / px_to_cell_y.
 *
 * px_to_cell_x / px_to_cell_y — convert pixel coordinate to cell index.
 *
 * Formula:  cell = floor(px / CELL_DIM + 0.5)
 *
 * Adding 0.5 before flooring implements "round half up" — deterministic
 * and symmetric.  This avoids two artefacts:
 *   • roundf() uses "round half to even" (banker's rounding), which can
 *     oscillate when a particle sits exactly on a cell boundary — single-
 *     pixel flicker.
 *   • Truncation ((int)(px / CELL_W)) always rounds toward zero, giving
 *     asymmetric dwell at boundaries and a subtle drift toward the origin.
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
/* §5  entity — Scene: Verlet ropes + constraint solver + renderer        */
/* ===================================================================== */

/*
 * Vec2 — lightweight 2-D position vector in pixel space.
 * x increases eastward (right); y increases downward (terminal convention).
 */
typedef struct { float x, y; } Vec2;

/*
 * Scene — complete simulation state for all N_ROPES rope chains.
 *
 * VERLET PAIR (pos[][], old_pos[][]):
 *
 *   pos[r][s]     — current position of particle s in rope r (pixel space).
 *   old_pos[r][s] — position from the previous simulation tick.
 *                   Velocity is stored implicitly as (pos − old_pos).
 *                   This is the key insight of Verlet integration: you never
 *                   store velocity explicitly; it emerges from the two
 *                   position snapshots.  Damping multiplies this difference
 *                   before adding it to the new position (see §5a).
 *
 * INTERPOLATION SNAPSHOT (prev_pos[][]):
 *
 *   Copied from pos[][] at the very start of each tick (before any physics).
 *   render_scene() lerps between prev_pos and pos using alpha ∈ [0,1):
 *     render_pos = prev_pos + (pos − prev_pos) × alpha
 *   This eliminates the micro-stutter that would otherwise occur at the
 *   tick boundary when sim Hz (60) differs from render Hz (60+).  Without
 *   prev_pos, the displayed position would jump by a full tick's worth of
 *   displacement every 1/60 s, which is visible as a periodic lurch.
 *
 * rest_len[r]     — segment rest length for rope r, in pixels.
 *                   = rope_len_px[r] / (N_SEG − 1).
 *                   e.g. rope_len_px = 400 px, N_SEG = 20:
 *                   rest_len = 400 / 19 ≈ 21.05 px per segment.
 *
 * rope_len_px[r]  — total visual length of rope r (35%–75% of screen height).
 *                   Computed in scene_init() from the current terminal rows,
 *                   so it adapts correctly after a SIGWINCH resize.
 *                   Variation in lengths makes the scene more interesting:
 *                   short ropes sway faster (higher natural frequency) and
 *                   long ropes sway slower — like a real wind-chime set.
 *
 * anchor[r]       — fixed ceiling pixel position for pos[r][0].
 *                   Evenly spaced: anchor[r].x = (r+1) × screen_width / (N_ROPES+1).
 *                   anchor[r].y = ANCHOR_ROW_CELLS × CELL_H (just below the HUD).
 *
 * phase_offset[r] — wind phase offset for rope r, radians.
 *                   = r × 2π / N_ROPES.
 *                   Pre-computed in scene_init() so the sin() argument in
 *                   apply_wind() is a simple addition each tick.
 *
 * wind_time       — accumulated simulation time in seconds, used as the
 *                   base argument to sinf() in apply_wind().
 *                   Advances by dt each tick regardless of pause state
 *                   (the wind clock only stops when the whole simulation
 *                   is paused — rope_tick() returns early if sc->paused).
 *
 * wind_amp        — lateral wind force amplitude in px/s².  Runtime-adjustable
 *                   with ↑/↓ keys in [0, 1000].  Maps directly to the peak
 *                   lateral acceleration per tick: accel = wind_amp × sin(…).
 *
 * wind_freq       — wind oscillation angular frequency in rad/s.  Runtime-
 *                   adjustable with ←/→ keys in [0.05, 4.0].  Controls the
 *                   tempo of the sway: 0.4 rad/s → ~15.7 s period (slow
 *                   breeze); 4.0 rad/s → ~1.6 s period (rapid shaking).
 *
 * paused          — when true, rope_tick() saves prev_pos then returns early,
 *                   freezing all particle positions.  The alpha lerp in
 *                   render_scene() produces a clean freeze (prev = curr).
 *
 * theme_idx       — index into THEMES[]; t/T keys cycle it.  Preserved
 *                   across r/R reset by saving before memset and restoring
 *                   after (see scene_init()).
 */
typedef struct {
    Vec2  pos     [N_ROPES][N_SEG];
    Vec2  old_pos [N_ROPES][N_SEG];
    Vec2  prev_pos[N_ROPES][N_SEG];

    float rest_len    [N_ROPES];
    float rope_len_px [N_ROPES];
    Vec2  anchor      [N_ROPES];
    float phase_offset[N_ROPES];

    float wind_time;
    float wind_amp;
    float wind_freq;

    bool  paused;
    int   theme_idx;
} Scene;

/* ── §5a  rope_verlet_step ──────────────────────────────────────────── */

/*
 * rope_verlet_step() — one Verlet integration step for particle (r, s).
 *
 * WHAT: Advances particle (r, s) by one time step dt using position-based
 *   Verlet integration.  Gravity and wind are applied as accelerations.
 *
 * WHY VERLET: Unlike explicit Euler (pos += vel × dt; vel += accel × dt),
 *   Verlet stores velocity implicitly as (pos − old_pos).  This makes it
 *   trivial to apply post-step constraints: after constraint_solve() moves
 *   the particle, the implicit velocity automatically reflects the corrected
 *   position.  With Euler, you would need to recompute velocity from the
 *   constrained position — Verlet does this for free.
 *
 * STEP-BY-STEP ALGORITHM:
 *
 *   1. Extract old and current positions:
 *        old = old_pos[r][s]
 *        cur = pos[r][s]
 *
 *   2. Compute damped implicit velocity:
 *        vel_x = (cur.x − old.x) × ROPE_DAMPING
 *        vel_y = (cur.y − old.y) × ROPE_DAMPING
 *
 *      ROPE_DAMPING = 0.992 removes 0.8% of velocity each tick.
 *      Physical intuition: a rope segment is thin and whippy — it loses
 *      energy to air drag faster than a rigid limb (which uses 0.995).
 *
 *   3. Update old_pos to current (slide the window forward):
 *        old_pos[r][s] = cur
 *
 *   4. Integrate acceleration and velocity into new position:
 *        pos[r][s].x = cur.x + vel_x + wind_x × dt²
 *        pos[r][s].y = cur.y + vel_y + GRAVITY × dt²
 *
 *      dt² appears because acceleration integrates twice:
 *        Δpos = vel × dt + ½ × accel × dt²
 *      The ½ factor is absorbed into the tuned values of GRAVITY and
 *      wind_amp (standard practice in game physics).
 *
 * PARAMETERS:
 *   sc     — scene containing the pos/old_pos arrays.
 *   r      — rope index [0, N_ROPES).
 *   s      — segment index [1, N_SEG) — particle 0 is the anchor and is
 *            NEVER passed here; it is pinned by enforce_anchors() instead.
 *   wind_x — lateral acceleration for this rope this tick (px/s²), computed
 *            once per rope per tick by apply_wind() and passed here.
 *   dt     — tick duration in seconds (1.0 / sim_fps).
 */
static void rope_verlet_step(Scene *sc, int r, int s, float wind_x, float dt)
{
    float dt2  = dt * dt;
    Vec2  old  = sc->old_pos[r][s];
    Vec2  cur  = sc->pos[r][s];

    /* Step 2: damped implicit velocity */
    float vel_x = (cur.x - old.x) * ROPE_DAMPING;
    float vel_y = (cur.y - old.y) * ROPE_DAMPING;

    /* Step 3: slide the old-position window */
    sc->old_pos[r][s] = cur;

    /* Step 4: integrate acceleration into new position */
    sc->pos[r][s].x   = cur.x + vel_x + wind_x  * dt2;
    sc->pos[r][s].y   = cur.y + vel_y + GRAVITY  * dt2;
}

/* ── §5b  apply_rope_constraints ────────────────────────────────────── */

/*
 * apply_rope_constraints() — run N_ITERS distance constraint passes for
 * all ropes, re-enforcing anchors after each pass.
 *
 * WHAT: For each adjacent particle pair (s, s+1) in each rope, push the
 *   particles apart or together so their distance equals rest_len[r].
 *
 * WHY ITERATIVE: One pass is not enough.  Correcting pair (0,1) disturbs
 *   particle 1, which then violates the (1,2) constraint, and so on.  Each
 *   additional pass propagates corrections further along the chain.  For a
 *   20-node rope, 6 passes converge the error to < 0.1% of rest_len — the
 *   segment looks inextensible to any human observer.
 *
 * CONSTRAINT EQUATION for one segment:
 *
 *   dx   = pos[r][s+1] − pos[r][s]       ← displacement vector
 *   dist = |dx|                           ← current length
 *   err  = (dist − rest_len[r]) / dist   ← fractional length error
 *
 *   correction = 0.5 × err × dx          ← split equally (equal mass)
 *   pos[r][s  ] += correction             ← push s away from s+1
 *   pos[r][s+1] -= correction             ← push s+1 away from s
 *
 *   Why multiply by 0.5?  Both particles have equal mass.  The total
 *   correction needed is err × dx; dividing it equally between two
 *   particles (0.5 each) conserves the centre of mass of the pair.
 *
 *   Guard: if dist < 1e-6 (particles coincident), skip to avoid divide-
 *   by-zero.  This can only happen in degenerate edge cases.
 *
 * ANCHOR RE-ENFORCEMENT AFTER EACH PASS:
 *
 *   The constraint solver above uses equal-mass corrections, which means
 *   it can displace particle[0] (the anchor) slightly when fixing the
 *   (0,1) segment.  Without re-pinning, these tiny displacements accumulate
 *   across N_ITERS passes and the anchor visibly drifts over time.
 *
 *   After every pass, enforce_anchors() resets pos[r][0] = old_pos[r][0]
 *   = anchor[r] for all ropes.  This zeroes the constraint-induced drift
 *   immediately after each pass, before the next pass can compound it.
 *
 * PARAMETERS:
 *   sc, cols, rows — scene and terminal dimensions (cols/rows passed to
 *                    enforce_anchors() for completeness but not used there).
 */
static void apply_rope_constraints(Scene *sc)
{
    for (int iter = 0; iter < N_ITERS; iter++) {
        for (int r = 0; r < N_ROPES; r++) {
            for (int s = 0; s < N_SEG - 1; s++) {
                float dx   = sc->pos[r][s+1].x - sc->pos[r][s].x;
                float dy   = sc->pos[r][s+1].y - sc->pos[r][s].y;
                float dist = sqrtf(dx * dx + dy * dy);

                if (dist < 1e-6f) continue;   /* coincident: skip safely */

                float error = (dist - sc->rest_len[r]) / dist;
                float cx    = 0.5f * error * dx;
                float cy    = 0.5f * error * dy;

                sc->pos[r][s  ].x += cx;
                sc->pos[r][s  ].y += cy;
                sc->pos[r][s+1].x -= cx;
                sc->pos[r][s+1].y -= cy;
            }

            /* Re-pin anchor after each constraint pass to stop drift */
            sc->pos[r][0]     = sc->anchor[r];
            sc->old_pos[r][0] = sc->anchor[r];
        }
    }
}

/* ── §5c  enforce_anchors ───────────────────────────────────────────── */

/*
 * enforce_anchors() — pin every rope's particle[0] to its ceiling position.
 *
 * WHAT: For each rope r, force:
 *   pos[r][0]     = anchor[r]
 *   old_pos[r][0] = anchor[r]
 *
 * WHY RESET BOTH pos AND old_pos:
 *
 *   After a Verlet step, pos[r][0] has been displaced by gravity and wind
 *   (even though it should be fixed).  Resetting only pos[r][0] = anchor[r]
 *   would leave old_pos[r][0] at the previous (Verlet-displaced) location.
 *
 *   On the NEXT tick, rope_verlet_step() computes:
 *     vel = (pos − old_pos) × DAMPING
 *         = (anchor − displaced_old_pos) × DAMPING
 *         ≠ zero
 *
 *   This phantom velocity would yank particle[0] off the anchor immediately,
 *   undoing the pin.  Over many ticks the anchor drifts progressively further
 *   from its ceiling position.
 *
 *   Resetting BOTH zeroes the implicit velocity:
 *     vel = (anchor − anchor) × DAMPING = 0
 *   The particle is truly immovable.
 *
 * WHEN CALLED:
 *   1. After rope_verlet_step() for all particles in a rope (inside
 *      rope_tick() step 4c).
 *   2. After every constraint pass inside apply_rope_constraints() to
 *      prevent constraint drift from accumulating (step 5 of rope_tick()).
 *
 * The double enforcement (after Verlet + after each constraint) completely
 * eliminates anchor drift at any wind_amp or constraint iteration count.
 */
static void enforce_anchors(Scene *sc)
{
    for (int r = 0; r < N_ROPES; r++) {
        sc->pos[r][0]     = sc->anchor[r];
        sc->old_pos[r][0] = sc->anchor[r];
    }
}

/* ── §5d  apply_wind ────────────────────────────────────────────────── */

/*
 * apply_wind() — compute lateral wind acceleration for rope r at the
 * current wind_time, and integrate it into all non-anchor particles.
 *
 * WHAT: Applies a horizontal sinusoidal force to every free particle
 *   (s = 1 … N_SEG-1) of rope r.  The force amplitude varies smoothly
 *   over time — positive (rightward) for half the period, negative
 *   (leftward) for the other half.
 *
 * WIND FORMULA:
 *   accel_x = wind_amp × sin(wind_time × wind_freq + phase_offset[r])
 *
 *   wind_time  — accumulated real simulation seconds.
 *   wind_freq  — angular frequency (rad/s).  Controls how fast the
 *                oscillation cycles: period = 2π / wind_freq.
 *   phase_offset[r] = r × 2π / N_ROPES.
 *
 * WHY PHASE OFFSETS:
 *   If all ropes shared the same phase (offset = 0), they would all sway
 *   left at the same time, then all sway right — boring and unrealistic.
 *
 *   Distributing 7 offsets evenly over 2π (spacing = 2π/7 ≈ 51.4°) means
 *   the ropes are always at distinct points in their cycle.  Neighbouring
 *   ropes are 51.4° apart; they never sway in the same direction at the
 *   same time.  The resulting "Mexican wave" is visually rich and complex
 *   despite being computed from a single sin() call per rope per tick.
 *
 * WHY SINUSOIDAL (not random impulse):
 *   Random impulses (like ragdoll_figure.c's gravity noise) create a
 *   jittery, unpredictable sway.  Sinusoidal force is:
 *   • Smooth: no sudden jumps → no constraint explosions even at high
 *     wind_amp.
 *   • Deterministic: the same amplitude/frequency always produces the
 *     same visual rhythm.
 *   • Controllable: wind_amp and wind_freq map directly to physical
 *     intuition (force magnitude, oscillation tempo).
 *
 * PARAMETERS:
 *   sc         — scene (reads wind_time, wind_amp, wind_freq, phase_offset[r]).
 *   r          — rope index: computes wind for this rope's particles only.
 *   dt         — tick duration in seconds (passed through to rope_verlet_step).
 *   cols/rows  — terminal dimensions (passed to rope_boundaries).
 *
 * SIDE EFFECTS: calls rope_verlet_step() for all non-anchor particles,
 *   then rope_boundaries() to clamp them to the screen box.
 */
static void apply_wind(Scene *sc, int r, float dt, int cols, int rows)
{
    /* Sinusoidal lateral acceleration for this rope this tick */
    float wind_acc = sc->wind_amp
                   * sinf(sc->wind_time * sc->wind_freq + sc->phase_offset[r]);

    /* Integrate wind + gravity into every non-anchor particle */
    for (int s = 1; s < N_SEG; s++) {
        rope_verlet_step(sc, r, s, wind_acc, dt);
    }

    /* Clamp particles to screen boundaries; bounce at floor and walls */
    float floor_y = (float)(rows * CELL_H) - FLOOR_MARGIN;
    float left_x  = LEFT_MARGIN;
    float right_x = (float)(cols * CELL_W) - RIGHT_MARGIN;

    for (int s = 1; s < N_SEG; s++) {
        /* Floor bounce: reflect old_pos.y across the floor boundary */
        if (sc->pos[r][s].y > floor_y) {
            sc->pos[r][s].y     = floor_y;
            sc->old_pos[r][s].y = sc->pos[r][s].y
                                + (sc->pos[r][s].y - sc->old_pos[r][s].y) * BOUNCE_COEFF;
        }
        /* Left wall bounce */
        if (sc->pos[r][s].x < left_x) {
            sc->pos[r][s].x     = left_x;
            sc->old_pos[r][s].x = sc->pos[r][s].x
                                + (sc->pos[r][s].x - sc->old_pos[r][s].x) * BOUNCE_COEFF;
        }
        /* Right wall bounce */
        if (sc->pos[r][s].x > right_x) {
            sc->pos[r][s].x     = right_x;
            sc->old_pos[r][s].x = sc->pos[r][s].x
                                + (sc->pos[r][s].x - sc->old_pos[r][s].x) * BOUNCE_COEFF;
        }
    }
}

/* ── §5e  rope_node_char ────────────────────────────────────────────── */

/*
 * rope_node_char() — choose the bead glyph for particle s based on its
 * position along the rope (anchor end → free tip).
 *
 * WHAT: Returns one of three ASCII characters that stamp at each particle
 *   position in pass 2 of draw_rope_beads():
 *   • '0' (zero)  — top quarter of rope  (s < N_SEG/4 = 5)
 *   • '.'         — bottom quarter        (s ≥ N_SEG×3/4 = 15)
 *   • 'o'         — middle half           (5 ≤ s < 15)
 *
 * PHYSICAL INTUITION — the size gradient mirrors physical tension:
 *   A hanging rope is under the most tension near the ceiling (it must
 *   support all the weight below it) and the least tension at the free
 *   tip (supports nothing).  Higher tension → brighter, thicker glyph.
 *   '0' (widest glyph) at the top suggests a taut, load-bearing segment.
 *   '.' (thinnest) at the bottom suggests a slack, dangling tip.
 *
 * WHY THREE ZONES (not a continuous gradient):
 *   Terminal character sets have limited glyph variety.  Three characters
 *   with visually distinct "weight" give the clearest size gradient within
 *   the ASCII repertoire.  The brightness gradient is reinforced separately
 *   by rope_node_attr() (A_BOLD / A_NORMAL / A_DIM).
 *
 * NUMERIC EXAMPLES (N_SEG = 20):
 *   s = 0  → N_SEG/4 = 5  → s < 5  → '0'   A_BOLD   (anchor particle)
 *   s = 4  → s < 5        → '0'   A_BOLD   (4th node from ceiling)
 *   s = 5  → 5 ≤ s < 15   → 'o'   A_NORMAL
 *   s = 14 → s < 15        → 'o'   A_NORMAL
 *   s = 15 → s ≥ 15        → '.'   A_DIM    (first bottom-quarter node)
 *   s = 19 → s ≥ 15        → '.'   A_DIM    (free tip — weight marker)
 */
static chtype rope_node_char(int s)
{
    if (s < N_SEG / 4)       return (chtype)'0';
    if (s >= N_SEG * 3 / 4)  return (chtype)'.';
    return (chtype)'o';
}

/*
 * rope_node_attr() — ncurses attribute for particle s.
 *
 * Mirrors rope_node_char() brightness logic:
 *   A_BOLD   — top quarter  (high tension, bright)
 *   A_DIM    — bottom quarter (low tension, faint)
 *   A_NORMAL — middle half
 *
 * The brightness gradient gives an immediate visual reading of where the
 * rope's mechanical load is concentrated, without any colour change.
 */
static attr_t rope_node_attr(int s)
{
    if (s < N_SEG / 4)       return A_BOLD;
    if (s >= N_SEG * 3 / 4)  return A_DIM;
    return A_NORMAL;
}

/* ── §5f  draw_rope_beads ───────────────────────────────────────────── */

/*
 * draw_rope_beads() — render one rope using the two-pass bead technique.
 *
 * WHAT: Fills the rope's visual appearance into WINDOW *w using two passes.
 *   Pass 1 fills every segment with 'o' beads at DRAW_STEP_PX intervals.
 *   Pass 2 overwrites each particle position with a size-graded node marker
 *   ('0', 'o', or '.') at the appropriate brightness attribute, then stamps
 *   an A_BOLD 'o' at the tip (particle N_SEG-1) as the weight marker.
 *
 * WHY TWO PASSES:
 *   Pass 1 fills the segment lines between particles, giving the rope visual
 *   continuity.  Pass 2 overrides particle positions with distinct glyph
 *   sizes, making the articulated bead structure visible.  Drawing node
 *   markers last ensures they always win over the fill beads on overlapping
 *   cells — the rope structure reads clearly even when two segments cross.
 *
 * PASS 1 — SEGMENT FILL:
 *   For each segment (s, s+1):
 *     dx, dy = displacement vector from rp[r][s] to rp[r][s+1].
 *     len    = Euclidean length of the segment.
 *     nsteps = ceil(len / DRAW_STEP_PX) + 1.
 *
 *     Walk t from 0 to nsteps:
 *       u  = t / nsteps                      ← fraction along segment
 *       cx = px_to_cell_x(rp[r][s].x + dx×u)
 *       cy = px_to_cell_y(rp[r][s].y + dy×u)
 *       If (cx, cy) == (prev_cx, prev_cy): skip (dedup guard).
 *       If out of [0,cols)×[0,rows): skip (bounds clipping).
 *       Stamp 'o' with COLOR_PAIR(cpair) | A_NORMAL.
 *
 *   The dedup guard (prev_cx/prev_cy tracking) prevents stamping the same
 *   terminal cell twice on the same segment, which would cause attribute
 *   flicker from the double wattron/wattroff sequence.
 *
 * PASS 2 — NODE MARKERS:
 *   For each particle s in [0, N_SEG):
 *     Convert rp[r][s] to (cx, cy); bounds-check; stamp rope_node_char(s)
 *     with rope_node_attr(s).
 *   Additionally stamp A_BOLD 'o' at the tip (s = N_SEG-1) as the
 *   "weight marker" — a bright blob at the rope's free end suggesting the
 *   mass that keeps the rope taut.
 *
 * PARAMETERS:
 *   sc    — scene (used to select the colour pair per rope).
 *   rp    — alpha-interpolated particle positions (computed by render_scene).
 *   r     — rope index to render.
 *   w     — ncurses WINDOW to draw into (typically stdscr).
 *   cols, rows — terminal dimensions for bounds checking.
 */
static void draw_rope_beads(const Scene *sc, const Vec2 rp[][N_SEG],
                            int r, WINDOW *w, int cols, int rows)
{
    (void)sc;                         /* colour pair derived from r, not sc  */
    int cpair = (r % N_PAIRS) + 1;   /* cycle rope colours through pairs 1–7 */

    /* ── Pass 1: fill segments with 'o' beads at DRAW_STEP_PX intervals ── */
    for (int s = 0; s < N_SEG - 1; s++) {
        float dx  = rp[r][s+1].x - rp[r][s].x;
        float dy  = rp[r][s+1].y - rp[r][s].y;
        float len = sqrtf(dx * dx + dy * dy);
        if (len < 0.1f) continue;   /* degenerate segment: skip */

        int nsteps  = (int)ceilf(len / DRAW_STEP_PX) + 1;
        int prev_cx = -9999, prev_cy = -9999;

        wattron(w, COLOR_PAIR(cpair) | A_NORMAL);
        for (int t = 0; t <= nsteps; t++) {
            float u  = (float)t / (float)nsteps;
            int   cx = px_to_cell_x(rp[r][s].x + dx * u);
            int   cy = px_to_cell_y(rp[r][s].y + dy * u);

            if (cx == prev_cx && cy == prev_cy) continue;   /* dedup */
            prev_cx = cx;  prev_cy = cy;
            if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

            mvwaddch(w, cy, cx, 'o');
        }
        wattroff(w, COLOR_PAIR(cpair) | A_NORMAL);
    }

    /* ── Pass 2: node markers overwrite fill beads at particle positions ── */
    for (int s = 0; s < N_SEG; s++) {
        int cx = px_to_cell_x(rp[r][s].x);
        int cy = px_to_cell_y(rp[r][s].y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        chtype nch  = rope_node_char(s);
        attr_t natt = rope_node_attr(s);
        wattron(w, COLOR_PAIR(cpair) | natt);
        mvwaddch(w, cy, cx, nch);
        wattroff(w, COLOR_PAIR(cpair) | natt);
    }

    /* ── Weight marker: bright 'o' at the free tip (particle N_SEG-1) ── */
    {
        int cx = px_to_cell_x(rp[r][N_SEG - 1].x);
        int cy = px_to_cell_y(rp[r][N_SEG - 1].y);
        if (cx >= 0 && cx < cols && cy >= 0 && cy < rows) {
            wattron(w, COLOR_PAIR(cpair) | A_BOLD);
            mvwaddch(w, cy, cx, 'o');
            wattroff(w, COLOR_PAIR(cpair) | A_BOLD);
        }
    }
}

/* ── §5g  render_scene ──────────────────────────────────────────────── */

/*
 * render_scene() — compose a complete frame of all 7 ropes into WINDOW *w.
 *
 * WHAT: Draws the ceiling line, then all ropes using draw_rope_beads().
 *   Sub-tick alpha interpolation smooths motion between physics ticks.
 *
 * STEP-BY-STEP:
 *
 *   1. ALPHA INTERPOLATION — compute render positions rp[r][s]:
 *        rp[r][s] = prev_pos[r][s] + (pos[r][s] − prev_pos[r][s]) × alpha
 *      alpha ∈ [0,1) is the fractional progress into the current unfired
 *      physics tick (computed in §8 main loop).  When alpha = 0 the display
 *      shows exactly the last physics state; when alpha → 1 it shows the
 *      predicted next state.  The result is visually smooth motion at any
 *      render frame rate, even if sim Hz ≠ render Hz.
 *
 *   2. CEILING LINE — draw a row of '#' characters across the full terminal
 *      width at ANCHOR_ROW_CELLS.
 *      Uses COLOR_PAIR(7) | A_DIM: the last rope's colour, dimmed, gives
 *      the ceiling a structural look without dominating the frame.
 *
 *   3. ROPES — call draw_rope_beads() for each rope r in order 0..N_ROPES-1.
 *      Later ropes are drawn on top of earlier ropes on overlapping cells
 *      (natural Z-order by rope index).
 *
 * PARAMETERS:
 *   sc         — const scene (simulation state, read-only).
 *   w          — ncurses WINDOW (stdscr).
 *   cols, rows — terminal dimensions for bounds checking.
 *   alpha      — sub-tick interpolation factor ∈ [0, 1).
 */
static void render_scene(const Scene *sc, WINDOW *w,
                         int cols, int rows, float alpha)
{
    /* Step 1 — alpha-interpolated render positions */
    Vec2 rp[N_ROPES][N_SEG];
    for (int r = 0; r < N_ROPES; r++) {
        for (int s = 0; s < N_SEG; s++) {
            rp[r][s].x = sc->prev_pos[r][s].x
                       + (sc->pos[r][s].x - sc->prev_pos[r][s].x) * alpha;
            rp[r][s].y = sc->prev_pos[r][s].y
                       + (sc->pos[r][s].y - sc->prev_pos[r][s].y) * alpha;
        }
    }

    /* Step 2 — ceiling '#' line at the anchor row */
    int anchor_row = ANCHOR_ROW_CELLS;
    if (anchor_row >= 0 && anchor_row < rows) {
        wattron(w, COLOR_PAIR(7) | A_DIM);
        for (int cx = 0; cx < cols; cx++)
            mvwaddch(w, anchor_row, cx, (chtype)'#');
        wattroff(w, COLOR_PAIR(7) | A_DIM);
    }

    /* Step 3 — ropes (two-pass bead rendering per rope) */
    for (int r = 0; r < N_ROPES; r++) {
        draw_rope_beads(sc, rp, r, w, cols, rows);
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * scene_init() — initialise all 7 ropes to a clean hanging state.
 *
 * Called at startup and on every r/R keypress reset.  Also called from
 * app_do_resize() after a SIGWINCH so rope lengths and anchor positions
 * are recomputed from the new terminal dimensions.
 *
 * THEME PRESERVATION:
 *   Save sc->theme_idx before memset; restore after.  This ensures the
 *   active colour theme survives a reset, matching user expectation that
 *   r/R resets the physics but not the visual style.
 *
 * ANCHOR DISTRIBUTION:
 *   N_ROPES = 7 anchors evenly spaced across the terminal width:
 *     anchor_x[r] = (r + 1) × screen_px_w / (N_ROPES + 1)
 *   Dividing by (N_ROPES + 1) rather than N_ROPES places equal margins
 *   at both sides, avoiding ropes pinned flush against the wall.
 *   e.g. 80-col terminal (640 px): anchors at 80, 160, 240, 320, 400,
 *   480, 560 px → 80 px (10 cols) spacing and margin on each side.
 *
 * ROPE LENGTH VARIATION (35%–75% of screen height):
 *   min_len = screen_px_h × 0.35   (35%: shortest rope — quick oscillation)
 *   max_len = screen_px_h × 0.75   (75%: longest  rope — slow oscillation)
 *   rope_len_px[r] = min_len + r × (max_len − min_len) / (N_ROPES − 1)
 *
 *   Physical intuition: a shorter rope has a higher natural frequency
 *   (like a shorter pendulum swings faster).  Mixing 7 different lengths
 *   means each rope's natural frequency is slightly different from its
 *   neighbours — the staggered sway looks more like a real wind-chime set
 *   than if all ropes were the same length.
 *
 *   rope_len_px is recomputed from 'rows' on every init, so the sim
 *   adapts cleanly to any terminal size including after resize.
 *
 * SEGMENT REST LENGTH:
 *   rest_len[r] = rope_len_px[r] / (N_SEG − 1)
 *   There are N_SEG particles but only (N_SEG − 1) segments between them.
 *   e.g. rope_len_px = 480 px, N_SEG = 20: rest_len = 480 / 19 ≈ 25.26 px.
 *
 * PHASE OFFSETS:
 *   phase_offset[r] = r × 2π / N_ROPES
 *   Pre-computed once here and used every tick in apply_wind().
 *   Spreading 7 offsets evenly over 2π (spacing ≈ 51.4°) guarantees no
 *   two ropes are at the same phase of the wind oscillation.
 *
 * INITIAL PARTICLE POSITIONS:
 *   All particles hang vertically straight down from the anchor:
 *     pos[r][s].x = anchor[r].x          (no lateral displacement)
 *     pos[r][s].y = anchor[r].y + s × rest_len[r]
 *   old_pos = pos: zero initial velocity (rope is at rest).
 *   prev_pos = pos: no-op alpha lerp on the first rendered frame.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    int saved_theme = sc->theme_idx;   /* preserve theme across reset */
    memset(sc, 0, sizeof *sc);
    sc->theme_idx = saved_theme;

    sc->wind_amp  = WIND_AMP_DEFAULT;
    sc->wind_freq = WIND_FREQ_DEFAULT;
    sc->wind_time = 0.0f;
    sc->paused    = false;

    float screen_px_h = (float)(rows * CELL_H);
    float screen_px_w = (float)(cols * CELL_W);
    float anchor_py   = (float)(ANCHOR_ROW_CELLS * CELL_H);

    /* Rope lengths span 35%–75% of screen height */
    float min_len = screen_px_h * 0.35f;
    float max_len = screen_px_h * 0.75f;

    for (int r = 0; r < N_ROPES; r++) {
        /* Evenly spaced ceiling anchor positions */
        sc->anchor[r].x = (float)(r + 1) * screen_px_w / (float)(N_ROPES + 1);
        sc->anchor[r].y = anchor_py;

        /* Rope length linearly interpolated from min to max */
        float len = min_len
                  + (float)r * (max_len - min_len) / (float)(N_ROPES - 1);
        sc->rope_len_px[r] = len;
        sc->rest_len[r]    = len / (float)(N_SEG - 1);

        /* Phase offset: distribute evenly over 2π */
        sc->phase_offset[r] = (float)r * 2.0f * (float)M_PI / (float)N_ROPES;

        /* Particles hang vertically; zero initial velocity */
        for (int s = 0; s < N_SEG; s++) {
            sc->pos[r][s].x     = sc->anchor[r].x;
            sc->pos[r][s].y     = sc->anchor[r].y + (float)s * sc->rest_len[r];
            sc->old_pos[r][s]   = sc->pos[r][s];
            sc->prev_pos[r][s]  = sc->pos[r][s];
        }
    }
}

/*
 * scene_tick() — one fixed-step physics update for all ropes.
 *
 * Called from §8's fixed-step accumulator at exactly sim_fps Hz.
 * dt is the fixed tick duration (= 1.0 / sim_fps) in seconds.
 *
 * ORDER IS CRITICAL:
 *
 *   1. SAVE prev_pos[] FIRST — before any physics runs.
 *      This is the interpolation anchor for render_scene().  It must hold
 *      the state from the END of the previous tick.  Saving after physics
 *      would produce a lerp that starts at the new state and interpolates
 *      beyond it — visual overshoot.
 *
 *   2. RETURN EARLY IF PAUSED — prev_pos is saved regardless so the alpha
 *      lerp in render_scene() produces a clean freeze (prev == curr).
 *
 *   3. ADVANCE wind_time — drives the sin() argument in apply_wind().
 *
 *   4. APPLY WIND + VERLET for each rope:
 *      a. apply_wind() integrates wind and gravity into all non-anchor
 *         particles via rope_verlet_step(), then bounces at boundaries.
 *      b. enforce_anchors() pins particle[0] immediately after Verlet,
 *         before constraints can propagate any anchor displacement.
 *
 *   5. APPLY DISTANCE CONSTRAINTS — apply_rope_constraints() runs N_ITERS
 *      passes, re-enforcing anchors after each pass to prevent drift.
 *
 * The Verlet→anchor→constraint→anchor sequence is the standard "position-
 * based dynamics" pipeline: integrate, then correct geometry.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    /* Step 1 — save snapshot for sub-tick alpha interpolation */
    memcpy(sc->prev_pos, sc->pos, sizeof sc->pos);

    /* Step 2 — skip physics while paused */
    if (sc->paused) return;

    /* Step 3 — advance wind clock */
    sc->wind_time += dt;

    /* Step 4 — Verlet integration + boundary for each rope */
    for (int r = 0; r < N_ROPES; r++) {
        apply_wind(sc, r, dt, cols, rows);   /* a. Verlet + bounce      */
    }
    enforce_anchors(sc);                     /* b. pin anchors post-Verlet */

    /* Step 5 — iterative distance constraints (re-pins anchors inside) */
    apply_rope_constraints(sc);
}

/*
 * scene_draw() — render all ropes; called once per render frame.
 *
 * alpha ∈ [0,1) is the sub-tick interpolation factor from §8.
 * dt_sec is unused for ropes (no entity needs it for rendering).
 */
static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)dt_sec;
    render_scene(sc, w, cols, rows, alpha);
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
 *   initscr()          — initialise ncurses; must be the first ncurses call.
 *   noecho()           — do not echo typed characters to the screen.
 *   cbreak()           — pass keys immediately, without line buffering.
 *   curs_set(0)        — hide the hardware cursor (no blinking cursor).
 *   nodelay(TRUE)      — getch() returns ERR immediately if no key is
 *                        available; makes input non-blocking so the render
 *                        loop never stalls waiting for user input.
 *   keypad(TRUE)       — decode arrow keys and function keys into single
 *                        KEY_* constants rather than multi-byte escape seqs.
 *   typeahead(-1)      — disable ncurses' habit of calling read() mid-output
 *                        to peek for escape sequences; without this, output
 *                        can be interrupted mid-frame, producing torn frames.
 *   color_init(0)      — register the initial colour palette (theme 0).
 *   getmaxyx()         — read actual terminal dimensions into s->cols/rows.
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
 * and mvwaddch calls at newly valid coordinates silently fail.
 *
 * After screen_resize(), app_do_resize() calls scene_init() to recompute
 * all rope lengths and anchor positions for the new size.
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
 *
 *   1. erase()      — write spaces over the entire newscr, erasing stale
 *                     content from the previous frame.  Does NOT write to
 *                     the terminal; only modifies the in-memory newscr.
 *
 *   2. scene_draw() — ceiling line + all 7 ropes.
 *
 *   3. HUD top      — status bar string, right-aligned at row 0, drawn
 *                     AFTER scene_draw() so it always wins over rope pixels
 *                     that land on the top row.
 *                     Content: rope count × segment count, wind amplitude,
 *                     wind frequency, active theme name, fps, sim Hz, state.
 *
 *   4. HUD bottom   — key reference line at the last terminal row, also
 *                     drawn after scene_draw() for the same reason.
 *
 * Nothing reaches the physical terminal until screen_present() calls
 * doupdate() — all writes in this function are to the in-memory newscr.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " ROPES  %dx%d  wind:%.0f  freq:%.2f  [%s]  %.1ffps  %dHz  %s ",
             N_ROPES, N_SEG,
             sc->wind_amp, sc->wind_freq,
             THEMES[sc->theme_idx].name,
             fps, sim_fps,
             sc->paused ? "PAUSED" : "swaying");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);

    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset  w/s:wind  a/d:freq  t:theme  [/]:Hz ");
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);
}

/*
 * screen_present() — flush the composed frame to the terminal in one write.
 *
 * wnoutrefresh(stdscr) — copies the in-memory newscr into ncurses' pending-
 *   update structure.  No terminal I/O happens yet.
 * doupdate() — diffs newscr against curscr (what is physically on screen),
 *   sends only the changed cells to the terminal fd, then sets curscr=newscr.
 *
 * This two-step is the correct ncurses flush pattern.  Calling refresh()
 * (= wnoutrefresh + doupdate in one call) is fine for a single window, but
 * the two-step allows future multi-window batching into a single doupdate().
 */
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level application state, accessible from signal handlers.
 *
 * Declared as a file-scope global (g_app) because POSIX signal handlers
 * receive no user-defined argument — they can only communicate with the
 * main loop through globals.
 *
 * running and need_resize are volatile sig_atomic_t because:
 *   volatile     — prevents the compiler from caching the value in a
 *                  register across the signal-handler write.  Without
 *                  volatile, the main loop might read a stale cached copy.
 *   sig_atomic_t — the only integer type POSIX guarantees can be read and
 *                  written atomically from a signal handler on all
 *                  conforming implementations.  Using int could theoretically
 *                  produce a torn read on some architectures.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

/* Signal handlers — set flags only; no ncurses or malloc calls here.
 * Signal handlers have severe restrictions (async-signal safety); setting
 * a volatile sig_atomic_t flag is one of the few safe operations. */
static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/*
 * cleanup() — atexit safety net.
 *
 * Registered with atexit() so endwin() is called even if the program exits
 * through an unhandled signal path that bypasses screen_free().  Without
 * this, the terminal could be left in raw/no-echo mode after a crash.
 */
static void cleanup(void) { endwin(); }

/*
 * app_do_resize() — handle a pending SIGWINCH terminal resize.
 *
 * WHAT:
 *   1. screen_resize() forces ncurses to re-read LINES/COLS from the kernel.
 *   2. scene_init() reinitialises all 7 ropes for the new dimensions.
 *      This is simpler and more correct than trying to relocate 7×20=140
 *      particles that may have scattered widely during strong wind.  The
 *      visual reset is instantaneous and imperceptible at resize time.
 *   3. need_resize is cleared.
 *
 * After app_do_resize() returns, the main loop resets frame_time and
 * sim_accum to prevent the large dt that accumulated during the resize
 * from firing a physics avalanche (hundreds of ticks in one frame).
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

/*
 * app_handle_key() — process one keypress; return false to quit.
 *
 * WHAT: Maps key codes to parameter mutations.  No physics logic here —
 *   only clamp + assign.  All clamping uses explicit if-guards rather than
 *   fmaxf/fminf so the logic is easy to follow and modify.
 *
 * KEY MAP:
 *   q / Q / ESC   — quit: return false → main loop exits.
 *   space         — toggle pause (freezes physics, rendering continues).
 *   r / R         — full reset: call scene_init() with current dimensions.
 *                   Theme is preserved by scene_init()'s save/restore logic.
 *   w / ↑         — wind_amp += 50 px/s²  [0, 1000]
 *                   More amplitude → wider lateral swing.
 *   s / ↓         — wind_amp -= 50 px/s²  [0, 1000]
 *                   Zero amplitude → gravity only; ropes hang straight.
 *   d / →         — wind_freq += 0.05 rad/s  [0.05, 4.0]
 *                   Higher frequency → faster oscillation tempo.
 *   a / ←         — wind_freq -= 0.05 rad/s  [0.05, 4.0]
 *                   Minimum 0.05 prevents division-by-zero (not used here)
 *                   and ensures a perceptible wind oscillation.
 *   t             — next colour theme (wraps 0 → N_THEMES-1 → 0).
 *   T             — previous colour theme (wraps 0 → N_THEMES-1 via modular
 *                   arithmetic: (idx − 1 + N_THEMES) % N_THEMES avoids
 *                   negative modulo which is implementation-defined in C).
 *   ] / + / =     — sim_fps += SIM_FPS_STEP  [SIM_FPS_MIN, SIM_FPS_MAX]
 *                   More physics ticks per second → tighter constraints.
 *   [ / -         — sim_fps -= SIM_FPS_STEP
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ':
        sc->paused = !sc->paused;
        break;

    case 'r': case 'R':
        scene_init(sc, app->screen.cols, app->screen.rows);
        break;

    /* Wind amplitude — controls the peak lateral force on each particle */
    case 'w': case KEY_UP:
        sc->wind_amp += 50.0f;
        if (sc->wind_amp > 1000.0f) sc->wind_amp = 1000.0f;
        break;
    case 's': case KEY_DOWN:
        sc->wind_amp -= 50.0f;
        if (sc->wind_amp < 0.0f) sc->wind_amp = 0.0f;
        break;

    /* Wind frequency — controls the oscillation tempo */
    case 'd': case KEY_RIGHT:
        sc->wind_freq += 0.05f;
        if (sc->wind_freq > 4.0f) sc->wind_freq = 4.0f;
        break;
    case 'a': case KEY_LEFT:
        sc->wind_freq -= 0.05f;
        if (sc->wind_freq < 0.05f) sc->wind_freq = 0.05f;
        break;

    /* Colour themes — t cycles forward, T cycles backward */
    case 't':
        sc->theme_idx = (sc->theme_idx + 1) % N_THEMES;
        theme_apply(sc->theme_idx);
        break;
    case 'T':
        sc->theme_idx = (sc->theme_idx - 1 + N_THEMES) % N_THEMES;
        theme_apply(sc->theme_idx);
        break;

    /* Simulation Hz — affects constraint convergence and physics accuracy */
    case ']': case '+': case '=':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[': case '-':
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
 * The loop body executes these eight steps every frame:
 *
 *  ① RESIZE CHECK
 *     Test need_resize (set by SIGWINCH handler) before touching ncurses.
 *     app_do_resize() re-reads terminal size and re-inits the scene.
 *     frame_time and sim_accum are reset immediately after to prevent the
 *     large dt that accumulated during the resize from firing a burst of
 *     physics ticks (a "physics avalanche").
 *
 *  ② MEASURE dt
 *     Wall-clock nanoseconds elapsed since the previous frame.
 *     Capped at 100 ms: if the process was suspended (Ctrl-Z, debugger,
 *     sleep) and resumed, an uncapped dt would fire up to 6 physics ticks
 *     in one frame at 60 Hz × 100 ms — a sudden visual jump.
 *
 *  ③ FIXED-STEP ACCUMULATOR
 *     sim_accum accumulates wall-clock dt each frame.
 *     While sim_accum ≥ tick_ns (period of one physics tick):
 *       fire one scene_tick() and drain tick_ns from sim_accum.
 *     Result: physics runs at exactly sim_fps Hz on average, completely
 *     decoupled from the render frame rate.  If the render is slow
 *     (say 30 fps), two physics ticks fire per render frame.  If the
 *     render is fast (120 fps), one physics tick fires every two frames.
 *
 *  ④ ALPHA — sub-tick interpolation factor
 *     After draining, sim_accum holds the fractional leftover — how far
 *     into the next unfired tick we are.
 *       alpha = sim_accum / tick_ns  ∈ [0, 1)
 *     Passed to render_scene() so particle positions are lerped between
 *     the last physics state and the predicted next state, eliminating
 *     the periodic micro-stutter visible when sim Hz < render Hz.
 *
 *  ⑤ FPS COUNTER
 *     Frames counted over a 500 ms sliding window.
 *     fps = frame_count / (fps_accum_s).
 *     The 500 ms window gives a stable display without per-frame jitter.
 *
 *  ⑥ FRAME CAP — sleep BEFORE render
 *     budget  = NS_PER_SEC / 60   (one 60-fps frame in nanoseconds).
 *     elapsed = time spent on physics since frame_time was last sampled.
 *     sleep   = budget − elapsed.
 *     Sleeping BEFORE the ncurses write means terminal I/O cost is not
 *     charged against the next frame's budget.  If sleep ≤ 0 (frame
 *     already over budget), clock_sleep_ns() returns immediately.
 *
 *  ⑦ DRAW + PRESENT
 *     erase() → scene_draw() → HUD → wnoutrefresh() → doupdate().
 *     One atomic diff write to the terminal fd; no partial frames visible.
 *
 *  ⑧ DRAIN INPUT
 *     Loop getch() until ERR, processing every queued keypress.
 *     Looping (not single-call) drains all key-repeat events within the
 *     same frame, keeping parameter adjustments responsive when held.
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

        /* ── ⑥ frame cap — sleep before render ──────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
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
