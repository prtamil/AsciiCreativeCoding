/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * rossler_attractor.c
 *   — The Rössler attractor: the simpler-cousin of Lorenz.  Same
 *     3-D ODE recipe but with ONLY ONE nonlinear term, yet it still
 *     exhibits a band-shaped strange attractor with a clean single
 *     scroll.  Famously analytically tractable enough to teach the
 *     "stretch + fold" mechanism by which 3-D continuous chaos
 *     arises.
 *
 * DEMO: Integrate Rössler's ODE with RK4; project the 3-D state
 *       to screen via a slowly-rotating camera.  Trail ring buffer
 *       fades oldest → newest in 4 bands.  16 presets cycle the
 *       parameter space — n/p step the active row — grouped from
 *       trivial → exotic so the user walks the full bifurcation
 *       cascade and the alternative topologies:
 *
 *         GROUP A  fixed point / decay   STILL, SPIRAL
 *         GROUP B  period-doubling       CYCLE_1, _2, _4, _8
 *         GROUP C  onset & chaos         ONSET, CHAOS_S, CHAOS,
 *                                        CHAOS_L, CHAOS_XL, BANDS
 *         GROUP D  periodic-3 window     CYCLE_3
 *         GROUP E  alternative topology  FUNNEL, SCREW, SCREW_L
 *
 *       The camera auto-fits each preset (per-preset bounding-sphere
 *       extent), so tiny limit cycles AND large screw attractors
 *       both fill the terminal.
 *
 * Study alongside:
 *   ./strange_attractor.c   — Lorenz + 9 other attractors.
 *   ./poincare_section.c    — same Lorenz, viewed as a 2-D return map.
 *
 * Section map:
 *   §1  config    — constants, 16-preset table, themes
 *   §2  clock     — monotonic timer + sleep
 *   §3  color     — HUD pairs + 10 themes
 *   §4  camera    — Camera (yaw/pitch/scale), camera_fit_to_screen,
 *                   project()
 *   §5  physics   — Rössler (system + state) + named-stage RK4
 *   §6  trail     — Trail ring buffer
 *   §7  state     — PresetState + PaletteState typed wrappers
 *   §8  scene     — Scene composite (Rossler + Trail + Camera +
 *                   PresetState + PaletteState + viewport)
 *   §9  screen    — paint_trail, scene_paint, HUD writers
 *   §10 app       — signals, resize, key dispatch table, FrameClock,
 *                   main pseudocode driver + named loop helpers
 *
 * Keys: q/ESC quit | space pause | r reset | n/p preset | t/T theme | ]/[ Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/chaos/rossler_attractor.c \
 *       -o rossler_attractor -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : RK4 with dt = 0.01 s and INT_STEPS_PER_TICK = 15
 *                 substeps per tick.  Camera is a slowly-rotating
 *                 azimuth (yaw) about z plus a fixed elevation
 *                 (pitch).  Each trail sample projects to a cell
 *                 via three named stages: rotate_about_z_yaw →
 *                 rotate_about_x_pitch → orthographic_to_cell.
 *
 * Data-struct   : Rossler composite (RosslerSystem + RosslerState)
 *                 + Trail ring buffer of TRAIL_MAX (3000) world
 *                 samples, walked oldest→newest and age-banded for
 *                 colour.  Camera = (yaw, pitch, scale); yaw
 *                 advances at CAM_YAW_RATE rad/s, scale auto-fits
 *                 the active preset's bounding-sphere extent.
 *
 * Rendering     : ASCII '.' for trail (4 age bands per Theme),
 *                 '@' for the live point on top.  Trail length
 *                 tuned so the full attractor fades over ~3 s —
 *                 enough to see the shape, not so long it
 *                 overwhelms the eye.
 *
 * References    : Numbered so inline code can cite [n].
 *
 *   PHYSICS / DYNAMICAL SYSTEMS
 *   [1] Rössler, O. E. (1976).  "An equation for continuous chaos",
 *       Physics Letters A 57(5), pp. 397-398.  THE original paper:
 *       defines the (x', y', z') equations and the canonical
 *       (a, b, c) = (0.2, 0.2, 5.7) regime.  Designed deliberately
 *       as the SIMPLEST 3-D ODE with one nonlinear term that still
 *       exhibits a strange attractor.
 *   [2] Strogatz, S. H. (2015).  *Nonlinear Dynamics and Chaos*
 *       (2nd ed.), §10.6 (Rössler system & period-doubling
 *       cascade) and §12.1-12.3 (chaotic attractors, stretch +
 *       fold).  Pedagogical reference for the bifurcation
 *       cascade visible in our PRESET_CYCLE_1..PRESET_CYCLE_8
 *       sequence, the period-3 window, and the chaos onset.
 *   [3] Letellier, C., Dutertre, P., Maheu, B. (1995).  "Unstable
 *       periodic orbits and templates of the Rössler system",
 *       Chaos 5(1), pp. 271-282.  Topological classification of
 *       the funnel, screw and other Rössler-family attractors —
 *       the basis for our PRESET_FUNNEL, _SCREW, _SCREW_L rows.
 *
 *   NUMERICS
 *   [4] Press, W. H. et al. (2007).  *Numerical Recipes* (3rd
 *       ed.), §17.1.  Classical 4-stage Runge-Kutta with the
 *       Butcher (1/6, 1/3, 1/3, 1/6) tableau used by
 *       rossler_rk4_step.
 *
 *   RENDERING
 *   [5] Foley, J. D., van Dam, A. et al. (1996).  *Computer
 *       Graphics: Principles and Practice* (2nd ed.), ch. 5-6.
 *       3-D rotation matrices + orthographic projection — the
 *       basis for the yaw/pitch projection in §4 (project()).
 *
 *   FRAMEWORK
 *   [6] Fiedler, G. (2004).  "Fix Your Timestep!",
 *       gafferongames.com.  Accumulator pattern used by
 *       app_drain_fixed_timestep so the integrator and the
 *       renderer decouple.
 *   [7] Sussman, G. J., Wisdom, J. (2014).  *Structure and
 *       Interpretation of Classical Mechanics* (2nd ed.), §1.6
 *       "Coordinate systems and states".  The System / State /
 *       Composite split used in §5 (RosslerSystem + RosslerState
 *       + Rossler composite).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Imagine a 2-D spiral that slowly grows outward in the (x, y)
 * plane.  Once x exceeds c, the z-component shoots up and PULLS
 * the trajectory toward small (x, y), then drops back down.
 * That's stretch + fold — the geometric mechanism for 3-D
 * continuous chaos.
 *
 * KEY FORMULAS  (Rössler 1976 [1])
 * ────────────
 *   dx/dt = −y − z
 *   dy/dt =  x + a·y
 *   dz/dt =  b + z·(x − c)
 *
 *   Classic chaotic regime: (a, b, c) = (0.20, 0.20, 5.70).
 *   Period-1 limit cycle  : c = 3.50.  Between these you can find
 *   the entire period-doubling cascade by varying c (Strogatz [2]
 *   §10.6); cycle these in PRESET_CYCLE_1..PRESET_CYCLE_8.
 *
 * HOW TO VERIFY  (texture / topology naming follows Letellier [3])
 * ─────────────
 *  • CHAOS:    trail traces a thin band-shaped attractor with
 *              occasional z-spikes pulling it toward the centre.
 *  • CYCLE_n:  trail collapses onto a closed loop with n leaves
 *              after transient (period-doubling cascade).
 *  • FUNNEL:   asymmetric "trumpet" spiral.
 *  • SCREW:    cylindrical attractor with long z-axis excursions
 *              — visible at PRESET_SCREW / _SCREW_L.
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN      =  10, SIM_FPS_DEFAULT = 60, SIM_FPS_MAX = 240, SIM_FPS_STEP = 10,
    HUD_COLS         =  80, FPS_UPDATE_MS = 500,
    PAIR_HUD         =   1, PAIR_HINT = 2,
    PAIR_TRAIL_BASE  =   3,    /* 4 trail bands */
    PAIR_LIVE        =   7,
};

#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1
#define NS_PER_SEC               1000000000LL
#define NS_PER_MS                   1000000LL
#define TICK_NS(f)               (NS_PER_SEC / (f))
#define RENDER_FPS_TARGET        60
#define RENDER_FRAME_BUDGET_NS   (NS_PER_SEC / RENDER_FPS_TARGET)
#define SIM_MAX_FRAME_DT_MS      100

#define ROSS_DT                   0.01f
#define INT_STEPS_PER_TICK         15
#define TRAIL_MAX                3000
#define TRAIL_BAND_COUNT            4

/* Camera */
#define CAM_PITCH                 0.45f      /* fixed elevation (rad) */
#define CAM_YAW_RATE              0.12f      /* yaw advance (rad / s) */

/* Camera fit — derive cam.scale from terminal size and the active
 * preset's bounding-sphere extent so the attractor fills the drawable
 * band regardless of resolution. */
#define ROSS_FIT_MARGIN            0.85f    /* leave a small border    */
#define ROSS_SCALE_MIN             0.45f    /* graceful for tiny terms */
#define CAM_FIT_DRAWABLE_HEIGHT_MIN  4      /* floor under degenerate sizes */
#define CAM_FIT_EXTENT_MIN         1.0f    /* avoid div-by-zero       */
#define CAM_HALF_FRACTION          0.5f    /* viewport centre split   */

/* Projection — z-axis compression before projecting onto the screen.
 * The Rössler z-component reaches ~80 in the screw regime; without
 * this squash the attractor would tower far off-screen vertically.
 * Matches the convention in ./strange_attractor.c. */
#define Z_SQUASH_FACTOR            0.5f

/* HUD display — radians → degrees for the yaw readout. */
#define RAD_TO_DEG                (180.0f / (float)M_PI)

/*
 * Preset — the 16-entry index space for the presets[] table.
 *
 * INTENT
 *   Named indices into the presets[] table, grouped by attractor
 *   topology (A trivial → E exotic).  Ordering is simple → complex
 *   so 'n' walks visual variety monotonically and the user
 *   experiences the period-doubling cascade BEFORE landing in
 *   chaos.  The period-3 window (CYCLE_3) is the one deliberate
 *   out-of-cascade insertion — it shows a PERIODIC island inside
 *   the chaotic band (Strogatz [2] §10.6, "type-I intermittency").
 *
 * CONTEXT
 *   Used only as table indices; entries themselves are defined in
 *   the presets[] static array below.  Group dividers in the enum
 *   line up with dividers in the array initialiser.  PresetState
 *   (§7) wraps a single instance of this enum and cycles modulo
 *   N_PRESETS.
 *
 * MEMBER LOGIC
 *   Each name follows the SHAPE/NUMBER convention.  Groups:
 *     A (2)  : fixed point / decay   STILL, SPIRAL
 *     B (4)  : period-doubling       CYCLE_1, _2, _4, _8
 *     C (6)  : onset & chaos         ONSET, CHAOS_S, CHAOS,
 *                                    CHAOS_L, CHAOS_XL, BANDS
 *     D (1)  : period-3 window       CYCLE_3
 *     E (3)  : alternative topology  FUNNEL, SCREW, SCREW_L
 *   PRESET_CHAOS is the canonical Rössler row [1] and is the
 *   boot-state default chosen by scene_init.
 *   N_PRESETS is the count sentinel; PresetState cycles modulo it.
 *
 * REFERENCES
 *   [1] Rössler 1976 — canonical CHAOS regime.
 *   [2] Strogatz §10.6 — the period-doubling cascade these
 *       presets are designed to demonstrate.
 *   [3] Letellier et al. (1995) — funnel & screw topology.
 */
typedef enum {
    /* Group A — fixed point / decay (2) */
    PRESET_STILL = 0,
    PRESET_SPIRAL,
    /* Group B — period-doubling cascade (4) */
    PRESET_CYCLE_1,
    PRESET_CYCLE_2,
    PRESET_CYCLE_4,
    PRESET_CYCLE_8,
    /* Group C — onset & chaos (6) */
    PRESET_ONSET,
    PRESET_CHAOS_S,
    PRESET_CHAOS,            /* canonical Rössler — default boot state */
    PRESET_CHAOS_L,
    /* Group D — periodic-3 window (1) */
    PRESET_CYCLE_3,
    /* Group C cont. — wider chaos (2) */
    PRESET_CHAOS_XL,
    PRESET_BANDS,
    /* Group E — alternative topologies (3) */
    PRESET_FUNNEL,
    PRESET_SCREW,
    PRESET_SCREW_L,
    N_PRESETS,
} Preset;

/*
 * RossPreset — one named (a, b, c) row of the parameter zoo plus its
 * camera-fit extent.
 *
 * INTENT
 *   Each row of the presets[] table fully describes one entry in
 *   the 16-preset zoo: 8-char HUD label, three Rössler [1] knobs,
 *   and the world-space bounding sphere used by the camera fit.
 *   Cycling through this table is the only way the user changes
 *   what the demo shows.
 *
 * CONTEXT
 *   Read by §5 rossler_system_from_preset (extracts a, b, c into
 *   the RosslerSystem), by §8 scene_compute_geometry (reads extent
 *   to re-fit the camera), and by §9 HUD writers (display name +
 *   parameters).  The PresetState wrapper in §7 holds an index
 *   INTO this table; cycling that index re-reads this row.
 *
 * MEMBER LOGIC
 *   name    : 8-char HUD label, padded so the parameter column aligns.
 *   a, b, c : the three Rössler [1] parameters.  All entries here use
 *             a ∈ {0.10, 0.20, 0.32}, b ∈ {0.10, 0.20, 0.30},
 *             c ∈ [1.5, 18.0].  Varying c alone walks the cascade;
 *             the FUNNEL and SCREW rows perturb a + b for topology.
 *   extent  : bounding-sphere radius in projected world units
 *             (x, y, z·½  — matching scene_paint's z-squash).
 *             Tuned per row so the camera fit gives every preset
 *             roughly the same on-screen size: limit cycles use
 *             ~3-8 cells, classic CHAOS ~14, SCREW_L ~38.
 *
 * REFERENCES
 *   [1] Rössler 1976 — parameter naming and canonical values.
 *   [2] Strogatz §10.6 — period-doubling cascade tuning of c.
 *   [3] Letellier et al. (1995) — funnel & screw parameter regimes.
 */
typedef struct { const char *name; float a, b, c; float extent; } RossPreset;
static const RossPreset presets[N_PRESETS] = {
    /* Group A — fixed point / decay ─────────────────────────────── */
    { "STILL   ", 0.20f, 0.20f,  1.50f,  3.0f },
    { "SPIRAL  ", 0.20f, 0.20f,  2.50f,  4.0f },
    /* Group B — period-doubling cascade ─────────────────────────── */
    { "CYCLE_1 ", 0.20f, 0.20f,  3.00f,  5.0f },
    { "CYCLE_2 ", 0.20f, 0.20f,  4.00f,  6.0f },
    { "CYCLE_4 ", 0.20f, 0.20f,  4.55f,  7.0f },
    { "CYCLE_8 ", 0.20f, 0.20f,  4.66f,  8.0f },
    /* Group C — onset & chaos ───────────────────────────────────── */
    { "ONSET   ", 0.20f, 0.20f,  4.72f,  9.0f },
    { "CHAOS_S ", 0.20f, 0.20f,  5.00f, 11.0f },
    { "CHAOS   ", 0.20f, 0.20f,  5.70f, 14.0f },
    { "CHAOS_L ", 0.20f, 0.20f,  6.00f, 16.0f },
    /* Group D — periodic window inside chaos ────────────────────── */
    { "CYCLE_3 ", 0.20f, 0.20f,  6.30f, 16.0f },
    /* Group C cont. ─────────────────────────────────────────────── */
    { "CHAOS_XL", 0.20f, 0.20f,  7.00f, 20.0f },
    { "BANDS   ", 0.20f, 0.20f,  8.00f, 24.0f },
    /* Group E — alternative topologies (Letellier et al. [3]) ──── */
    { "FUNNEL  ", 0.32f, 0.30f,  4.50f, 12.0f },   /* asymmetric one-fold  */
    { "SCREW   ", 0.10f, 0.10f, 14.00f, 30.0f },   /* cylindrical screw    */
    { "SCREW_L ", 0.10f, 0.10f, 18.00f, 38.0f },   /* larger-radius screw  */
};

/*
 * Theme — one named palette for the attractor renderer.
 *
 * INTENT
 *   Group the colour codes that define an attractor's visual
 *   identity into ONE named row, so a "next theme" key cycles a
 *   single flat table.  Every theme provides FOUR trail-band
 *   shades (oldest → newest) plus a live-point colour.
 *
 * CONTEXT
 *   Indexed by PaletteState (§7); applied by theme_apply (§3)
 *   which pushes each band into PAIR_TRAIL_BASE + i and the
 *   live colour into PAIR_LIVE.  Cycled by t/T via
 *   app_cycle_theme_next/prev.  All palette values follow the
 *   project Brightness Rule (CLAUDE.md): every code lives in
 *   the bright half of the 256-colour cube so the dots stay
 *   legible on default black.
 *
 * MEMBER LOGIC
 *   name   : 7-char HUD label.
 *   band[] : TRAIL_BAND_COUNT (4) 256-colour codes, sorted
 *            dimmest → brightest.  paint_trail picks band[0]
 *            for the oldest sample and band[3] for the newest.
 *   live   : colour of the '@' live-point glyph (PAIR_LIVE).
 *            Usually a high-saturation hot colour so the
 *            "current state" pops against the trail.
 *
 * REFERENCES
 *   CLAUDE.md "Theme Palette Brightness".
 */
typedef struct { const char *name; short band[TRAIL_BAND_COUNT]; short live; } Theme;
#define N_THEMES 10
static const Theme themes[N_THEMES] = {
    { "DEFAULT", {  75, 123, 220, 231 }, 196 },
    { "MATRIX",  {  77, 118, 156, 194 }, 226 },
    { "NOVA",    { 135, 171, 207, 219 }, 226 },
    { "MONO",    { 247, 250, 253, 255 }, 226 },
    { "OCEAN",   {  81, 117, 159, 195 }, 226 },
    { "FIRE",    { 208, 214, 220, 227 }, 231 },
    { "EARTH",   { 143, 179, 215, 222 }, 196 },
    { "FOREST",  { 114, 150, 157, 194 }, 226 },
    { "DESERT",  { 179, 215, 222, 229 }, 196 },
    { "ARCTIC",  { 117, 159, 195, 231 }, 196 },
};

/* ===================================================================== */
/* §2 clock + §3 color                                                    */
/* ===================================================================== */

static int64_t clock_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec; }
static void clock_sleep_ns(int64_t ns) { if (ns <= 0) return;
    struct timespec req = {ns/NS_PER_SEC, ns%NS_PER_SEC}; nanosleep(&req, NULL); }

/* theme_install_256 — push a theme's bright-cube codes into the
 * two ncurses pair classes (trail bands, live point). */
static inline void theme_install_256(const Theme *t)
{
    for (int i = 0; i < TRAIL_BAND_COUNT; i++)
        init_pair(PAIR_TRAIL_BASE + i, t->band[i], -1);
    init_pair(PAIR_LIVE, t->live, -1);
}

/* theme_install_8color_fallback — graceful degradation when the
 * terminal advertises only 8 colours.  All trail bands collapse to
 * cyan; live point becomes red. */
static inline void theme_install_8color_fallback(void)
{
    for (int i = 0; i < TRAIL_BAND_COUNT; i++)
        init_pair(PAIR_TRAIL_BASE + i, COLOR_CYAN, -1);
    init_pair(PAIR_LIVE, COLOR_RED, -1);
}

/* theme_apply — push the chosen palette to the ncurses pair table.
 *
 * DRIVER PSEUDOCODE
 *   if idx out of range:  idx = 0                    // graceful fallback
 *   if terminal has 256 colours:
 *       theme_install_256(themes[idx])
 *   else:
 *       theme_install_8color_fallback()
 */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_install_256(&themes[idx]);
    else               theme_install_8color_fallback();
}
static void color_init(void)
{ start_color(); use_default_colors();
  if (COLORS >= 256) { init_pair(PAIR_HUD, 226, -1); init_pair(PAIR_HINT, 51, -1); }
  else { init_pair(PAIR_HUD, COLOR_YELLOW, -1); init_pair(PAIR_HINT, COLOR_CYAN, -1); }
  theme_apply(0); }

/* ===================================================================== */
/* §4  camera — yaw + pitch projection                                    */
/* ===================================================================== */

/*
 * Camera — orbiting orthographic camera onto the 3-D attractor.
 *
 * INTENT
 *   The 3-D attractor needs to project to 2-D character cells.  A
 *   FULL projection matrix would obscure the geometry; instead we
 *   carry just the three scalars that drive the look: yaw (which
 *   slowly rotates so the user sees the attractor from all sides),
 *   pitch (fixed elevation so the z-axis stays vertical-ish), and
 *   scale (auto-fit to terminal × preset extent).
 *
 * CONTEXT
 *   Owned by Scene.  yaw is advanced each tick by CAM_YAW_RATE
 *   (radians per sim second).  scale is set by camera_fit_to_screen
 *   from (cols, rows, active preset's extent); pitch is fixed at
 *   CAM_PITCH = 0.45 rad (≈ 26°).  Consumed by project(), which
 *   composes the yaw + pitch rotation and the orthographic squash.
 *
 * MEMBER LOGIC
 *   yaw   : azimuth angle (radians) around the z-axis.  Advances
 *           monotonically; not normalised since sinf/cosf wrap
 *           naturally.
 *   pitch : elevation angle (radians) above the xy-plane.  Fixed
 *           — varying pitch made the rotation feel disorienting
 *           in ncurses-resolution renders.
 *   scale : world-units → cells multiplier.  Re-derived on init,
 *           resize, and preset cycle by camera_fit_to_screen.
 *
 * REFERENCES
 *   [5] Foley & van Dam (1996) ch. 5 — rotation + orthographic
 *       projection math.
 */
typedef struct { float yaw, pitch, scale; } Camera;

#define CELL_ASPECT 0.5f   /* characters are roughly 2× tall as wide */

/* screen_drawable_height — viewport rows MINUS the HUD reservation,
 * floored at CAM_FIT_DRAWABLE_HEIGHT_MIN so the fit math stays
 * well-defined on tiny terminals. */
static inline int screen_drawable_height(int rows)
{
    int draw_h = rows - HUD_BAND_RESERVED_ROWS;
    if (draw_h < CAM_FIT_DRAWABLE_HEIGHT_MIN) draw_h = CAM_FIT_DRAWABLE_HEIGHT_MIN;
    return draw_h;
}

/* horizontal_fit_scale — largest world→cell scale that keeps an
 * attractor of radius `extent` inside cols × MARGIN cells horizontally.
 *
 *   half_x_cells × MARGIN ≥ extent × scale   ⇒   scale ≤ …
 */
static inline float horizontal_fit_scale(int cols, float extent, float margin)
{
    float half_x_cells = (float)cols * CAM_HALF_FRACTION;
    return (half_x_cells * margin) / extent;
}

/* vertical_fit_scale — same logic, vertical axis.  Account for
 * CELL_ASPECT: vertical world units occupy fewer cells than
 * horizontal because characters are 2× tall as wide. */
static inline float vertical_fit_scale(int rows, float extent, float margin, float aspect)
{
    int   draw_h       = screen_drawable_height(rows);
    float half_y_cells = (float)draw_h * CAM_HALF_FRACTION;
    return (half_y_cells * margin) / (extent * aspect);
}

/* camera_fit_to_screen — pick cam.scale so the attractor fills the
 * drawable band on any terminal size.
 *
 * DRIVER PSEUDOCODE
 *   extent_safe = max(extent, EXTENT_MIN)             // avoid /0
 *   sx = horizontal_fit_scale(cols, extent_safe, MARGIN)
 *   sy = vertical_fit_scale  (rows, extent_safe, MARGIN, CELL_ASPECT)
 *   scale = min(sx, sy)                                // tightest axis wins
 *   scale = max(scale, SCALE_MIN)                      // graceful floor
 *   cam.scale ← scale
 */
static void camera_fit_to_screen(Camera *cam, int cols, int rows, float extent)
{
    if (extent < CAM_FIT_EXTENT_MIN) extent = CAM_FIT_EXTENT_MIN;

    float sx = horizontal_fit_scale(cols, extent, ROSS_FIT_MARGIN);
    float sy = vertical_fit_scale  (rows, extent, ROSS_FIT_MARGIN, CELL_ASPECT);

    float scale = (sx < sy) ? sx : sy;
    if (scale < ROSS_SCALE_MIN) scale = ROSS_SCALE_MIN;
    cam->scale = scale;
}

/* rotate_about_z_yaw — Foley/van Dam [5] ch.5 right-handed rotation
 * by yaw radians about the z-axis.  Applied to a 3-D point's (x, y);
 * z passes through unchanged.
 *
 *   ┌xr┐ = ┌ cos(θ)  −sin(θ) ┐ ┌x┐
 *   └yr┘   └ sin(θ)   cos(θ) ┘ └y┘
 */
static inline void rotate_about_z_yaw(float x, float y,
                                      float cos_yaw, float sin_yaw,
                                      float *xr, float *yr)
{
    *xr = x * cos_yaw - y * sin_yaw;
    *yr = x * sin_yaw + y * cos_yaw;
}

/* rotate_about_x_pitch — Foley/van Dam [5] ch.5 right-handed rotation
 * by pitch radians about the x-axis, applied to (yr, z).  Returns
 * the new vertical-world component yr2; the xr component is unchanged
 * by an x-axis rotation, so we only return what changes. */
static inline float rotate_about_x_pitch(float yr, float z,
                                         float cos_pitch, float sin_pitch)
{
    return yr * cos_pitch - z * sin_pitch;
}

/* orthographic_to_cell — final stage of project().  Drop the depth
 * axis, multiply by cam.scale, flip y (screen-y points DOWN, world-y
 * points UP), scale by CELL_ASPECT so characters look proportional.
 *
 *   sx_cells = cx + xr_world  · scale
 *   sy_cells = cy − yr2_world · scale · aspect
 *
 * Foley/van Dam [5] §6.2: parallel projection + viewport transform. */
static inline void orthographic_to_cell(float xr_world, float yr2_world,
                                        float scale, float aspect,
                                        int cx, int cy, int *sx, int *sy)
{
    *sx = cx + (int)( xr_world  * scale);
    *sy = cy + (int)(-yr2_world * scale * aspect);
}

/* project — world (x, y, z) → cell (sx, sy).  Three named stages
 * (Foley/van Dam [5] ch.5-6):
 *
 * DRIVER PSEUDOCODE
 *   STAGE 1 — rotate_about_z_yaw   (yaw   about z) → (xr,  yr)
 *   STAGE 2 — rotate_about_x_pitch (pitch about x) →       yr2
 *   STAGE 3 — orthographic_to_cell                  → (sx,  sy)
 */
static void project(const Camera *cam,
                    float x, float y, float z,
                    int cx_centre, int cy_centre, float aspect,
                    int *sx, int *sy)
{
    float cos_yaw  = cosf(cam->yaw),   sin_yaw   = sinf(cam->yaw);
    float cos_pitch = cosf(cam->pitch), sin_pitch = sinf(cam->pitch);

    float xr, yr;
    rotate_about_z_yaw(x, y, cos_yaw, sin_yaw, &xr, &yr);
    float yr2 = rotate_about_x_pitch(yr, z, cos_pitch, sin_pitch);
    orthographic_to_cell(xr, yr2, cam->scale, aspect, cx_centre, cy_centre, sx, sy);
}

/* ===================================================================== */
/* §5  Rössler ODE — system + state composite, RK4 integrator             */
/* ===================================================================== */

/*
 * Vec3 — generic 3-component vector.
 *
 * INTENT
 *   The single 3-tuple this file traffics in.  Used three ways:
 *     • a Rössler phase point  (RosslerState alias)
 *     • a Rössler derivative   (output of rossler_deriv)
 *     • a trail sample         (geometry data fed to the renderer)
 *   Flat by-value struct so RK4 stages combine cheaply via state_add
 *   without extra heap allocation.
 *
 * CONTEXT
 *   Returned by rossler_deriv, stored as RosslerState inside Rossler,
 *   pushed into Trail by trail_push, projected to (sx, sy) by project().
 *
 * MEMBER LOGIC
 *   x, y, z : scalar components.  In Rössler context they are the
 *             three observables of the ODE; in trail/render context
 *             they are just a 3-tuple to project onto cells.
 */
typedef struct { float x, y, z; } Vec3;

/*
 * RosslerState — semantic alias for Vec3 at the "ODE phase point" site.
 *
 * INTENT
 *   Same memory layout as Vec3; the typedef just labels intent so a
 *   call like state_add(RosslerState, dt, slope) reads as ODE arith-
 *   metic, not generic vector arithmetic.  No new fields.  Mirrors
 *   the pattern in ./strange_attractor.c (LorenzState = Vec3).
 *
 * CONTEXT
 *   Lives inside Rossler::state.  Mutated by rossler_rk4_step.  Read
 *   by scene_paint to project the live point.
 *
 * REFERENCES
 *   [1] Rössler 1976 — the (x, y, z) coordinates these scalars name.
 */
typedef Vec3 RosslerState;

/*
 * RosslerSystem — invariant parameters of the Rössler ODE.
 *
 * INTENT
 *   Split parameters away from state so rossler_deriv becomes a PURE
 *   function of (state, system).  Cycling a preset only re-assigns
 *   the system — the state is reset separately to its canonical
 *   seed (1, 1, 1).  Mirrors the System+State split used in every
 *   other chaos demo (Sussman & Wisdom [7] §1.6).
 *
 * CONTEXT
 *   Loaded from the active preset row via
 *   rossler_system_from_preset().  Read-only inside rossler_deriv
 *   and rossler_rk4_step.  Mutated only on preset cycle.
 *
 * MEMBER LOGIC
 *   a : weight on the y-component of the second equation.  Controls
 *       the rate of spiral expansion in the (x, y) plane.  Canonical
 *       value 0.2; FUNNEL uses 0.32, SCREW uses 0.10.
 *   b : constant pump in the z-equation.  Prevents z from settling
 *       to zero.  Canonical 0.2; SCREW uses 0.10.
 *   c : threshold at which z spikes (when x ≥ c the z-equation
 *       grows exponentially).  Controls the period-doubling cascade:
 *       c = 3 → period-1, 4 → period-2, 4.55 → period-4, 4.66 → -8,
 *       4.72 → chaos onset, 5.7 → classic.  Larger c yields screw
 *       attractors (14, 18).
 *
 * REFERENCES
 *   [1] Rössler 1976 — equations 1-3 and parameter naming.
 *   [2] Strogatz §10.6 — bifurcation diagram in c.
 */
typedef struct { float a, b, c; } RosslerSystem;

/*
 * Rossler — composite: invariant system + current state.
 *
 * INTENT
 *   One value carrying everything needed to advance the ODE; matches
 *   the abstraction layout used by every chaos demo in this project
 *   (Lorenz, double pendulum, Hénon-Heiles, Duffing).  Lets
 *   rossler_rk4_step take a single argument instead of a state
 *   pointer plus four loose float parameters.
 *
 * CONTEXT
 *   Owned by Scene; rossler_rk4_step mutates its state field while
 *   reading its system field as const.  scene_apply_preset rebuilds
 *   it on every preset change.
 *
 * MEMBER LOGIC
 *   system : the (a, b, c) triple — never mutated by rossler_rk4_step.
 *   state  : the (x, y, z) phase point — mutated each RK4 step.
 *
 * REFERENCES
 *   [4] Numerical Recipes §17.1 — RK4 integrator that operates on
 *       this composite.
 *   [7] Sussman & Wisdom §1.6 — System/State separation rationale.
 */
typedef struct {
    RosslerSystem system;
    RosslerState  state;
} Rossler;

/* rossler_deriv — evaluate dy/dt = f(state, system).  Rössler 1976
 * [1]; each line of the body maps 1-to-1 to one published formula:
 *
 *   dx/dt = −y − z
 *   dy/dt =  x + a·y
 *   dz/dt =  b + z·(x − c)
 *
 * The ONE nonlinear term (z·x) is what makes Rössler the simplest
 * 3-D ODE that exhibits a strange attractor (see Strogatz [2] §12.1
 * on stretch + fold mechanism). */
static inline RosslerState rossler_deriv(const RosslerState *s,
                                         const RosslerSystem *sys)
{
    RosslerState dy;
    dy.x = -s->y - s->z;                            /* −y − z          */
    dy.y =  s->x + sys->a * s->y;                   /*  x + a·y        */
    dy.z =  sys->b + s->z * (s->x - sys->c);        /*  b + z·(x − c)  */
    return dy;
}

/* state_add — y + h·k, returned as a new RosslerState.  Pure helper
 * for combining an RK4 stage's slope into a midpoint estimate. */
static inline RosslerState state_add(const RosslerState *a,
                                     float h, const RosslerState *k)
{
    RosslerState r;
    r.x = a->x + h * k->x;
    r.y = a->y + h * k->y;
    r.z = a->z + h * k->z;
    return r;
}

/* RK4 Butcher tableau constants — Numerical Recipes [4] §17.1. */
#define RK4_BUTCHER_WEIGHT_SUM   6.0f
#define RK4_MIDPOINT_FRACTION    0.5f

/* rk4_butcher_weighted_average — (k₁ + 2k₂ + 2k₃ + k₄) / 6. */
static inline RosslerState rk4_butcher_weighted_average(
    const RosslerState *k1, const RosslerState *k2,
    const RosslerState *k3, const RosslerState *k4)
{
    RosslerState avg;
    avg.x = (k1->x + 2.0f*k2->x + 2.0f*k3->x + k4->x) / RK4_BUTCHER_WEIGHT_SUM;
    avg.y = (k1->y + 2.0f*k2->y + 2.0f*k3->y + k4->y) / RK4_BUTCHER_WEIGHT_SUM;
    avg.z = (k1->z + 2.0f*k2->z + 2.0f*k3->z + k4->z) / RK4_BUTCHER_WEIGHT_SUM;
    return avg;
}

/* rossler_rk4_step — one fixed-step RK4 update of the composite.
 * Classical 4-stage Runge-Kutta with the Butcher (1/6, 1/3, 1/3, 1/6)
 * tableau (Numerical Recipes [4] §17.1).
 *
 * DRIVER PSEUDOCODE
 *   half_dt        = dt / 2
 *   slope_start    = f(state)
 *   midpoint_1     = state + half_dt · slope_start
 *   slope_mid_1    = f(midpoint_1)
 *   midpoint_2     = state + half_dt · slope_mid_1
 *   slope_mid_2    = f(midpoint_2)
 *   endpoint       = state + dt · slope_mid_2
 *   slope_end      = f(endpoint)
 *   effective_slope = (slope_start + 2·slope_mid_1
 *                                  + 2·slope_mid_2
 *                                  +   slope_end ) / 6
 *   state ← state + dt · effective_slope
 *
 * f here is rossler_deriv (system-parameterised), so the call sites
 * read 1-to-1 with the published Butcher tableau. */
static void rossler_rk4_step(Rossler *r, float dt)
{
    const RosslerSystem *sys = &r->system;
    float half_dt = RK4_MIDPOINT_FRACTION * dt;

    RosslerState slope_start    = rossler_deriv(&r->state, sys);
    RosslerState midpoint_1     = state_add(&r->state, half_dt, &slope_start);
    RosslerState slope_mid_1    = rossler_deriv(&midpoint_1, sys);
    RosslerState midpoint_2     = state_add(&r->state, half_dt, &slope_mid_1);
    RosslerState slope_mid_2    = rossler_deriv(&midpoint_2, sys);
    RosslerState endpoint       = state_add(&r->state, dt, &slope_mid_2);
    RosslerState slope_end      = rossler_deriv(&endpoint, sys);

    RosslerState effective_slope = rk4_butcher_weighted_average(
        &slope_start, &slope_mid_1, &slope_mid_2, &slope_end);
    r->state = state_add(&r->state, dt, &effective_slope);
}

/* rossler_system_from_preset — construct a RosslerSystem from one
 * row of the §1 presets[] table.  Named so the call site reads
 * "rossler = system_from_preset" rather than "rossler.a = p->a; …". */
static inline RosslerSystem rossler_system_from_preset(const RossPreset *p)
{
    return (RosslerSystem){ p->a, p->b, p->c };
}

/* ===================================================================== */
/* §6  trail — fixed-capacity ring buffer of recent samples              */
/* ===================================================================== */

/*
 * Trail — ring buffer of the last TRAIL_MAX Rössler samples.
 *
 * INTENT
 *   Decouple "how long the visible streak is" from "how fast we
 *   advance the integrator" — the renderer reads up to TRAIL_MAX
 *   samples, the integrator pushes one every RK4 step.  No malloc;
 *   the whole buffer is a static field of Scene.  Storing the
 *   three axes as PARALLEL float arrays (struct-of-arrays) keeps
 *   each axis cache-coherent during paint_trail's age-banded walk.
 *
 * CONTEXT
 *   Written by scene_tick (one trail_push per RK4 step inside the
 *   INT_STEPS_PER_TICK substep loop).  Read by paint_trail in §9
 *   which walks oldest → newest, age-banding into TRAIL_BAND_COUNT
 *   colour tiers (oldest → dimmest).  Reset on r-key, preset
 *   change, and SIGWINCH so a fresh attractor isn't ghosted.
 *
 * MEMBER LOGIC
 *   x[], y[], z[] : per-axis sample arrays of length TRAIL_MAX
 *                   (3000).  Index 'head' is the newest sample;
 *                   (head - count + 1) mod TRAIL_MAX is the oldest.
 *   head          : index of the most recent sample.  Incremented
 *                   modulo TRAIL_MAX on every trail_push.
 *   count         : number of valid samples, capped at TRAIL_MAX.
 *                   Saturates once the buffer fills (after ~3
 *                   seconds at the default sim rate).
 *
 * REFERENCES
 *   Standard ring buffer; nothing exotic.  Same shape as the trail
 *   used by ./strange_attractor.c and ./poincare_section.c.
 */
typedef struct {
    float x[TRAIL_MAX], y[TRAIL_MAX], z[TRAIL_MAX];
    int   head, count;
} Trail;

static void trail_reset(Trail *t) { t->head = 0; t->count = 0; }
static void trail_push(Trail *t, Vec3 p)
{
    t->head = (t->head + 1) % TRAIL_MAX;
    t->x[t->head] = p.x;
    t->y[t->head] = p.y;
    t->z[t->head] = p.z;
    if (t->count < TRAIL_MAX) t->count++;
}

/* trail_oldest_index — ring-buffer offset of the oldest valid sample.
 * (head − count + 1) mod TRAIL_MAX, with TRAIL_MAX added to keep the
 * intermediate non-negative under C's truncating mod. */
static inline int trail_oldest_index(const Trail *t)
{
    return (t->head - t->count + 1 + TRAIL_MAX) % TRAIL_MAX;
}

/* trail_sample_at — i-th sample from oldest (i=0) to newest
 * (i=count-1).  Centralises the ring-buffer wrap so paint_trail
 * doesn't need to know the storage layout. */
static inline Vec3 trail_sample_at(const Trail *t, int i_from_oldest)
{
    int idx = (trail_oldest_index(t) + i_from_oldest) % TRAIL_MAX;
    return (Vec3){ t->x[idx], t->y[idx], t->z[idx] };
}

/* trail_age_band — map a sample's age (0=newest, n-1=oldest) onto
 * one of TRAIL_BAND_COUNT colour tiers.  Newer samples get higher
 * band indices = brighter colours.  Clamped to [0, BAND_COUNT-1]. */
static inline int trail_age_band(int age_from_newest, int n)
{
    int band = (TRAIL_BAND_COUNT - 1) - (age_from_newest * TRAIL_BAND_COUNT) / n;
    if (band < 0)                     band = 0;
    if (band > TRAIL_BAND_COUNT - 1)  band = TRAIL_BAND_COUNT - 1;
    return band;
}

/* world_to_view_space — apply Z_SQUASH_FACTOR.  Names the convention
 * that the renderer "sees" a shorter z than the integrator computed,
 * keeping screw-attractor z-spikes from overflowing the viewport.
 * Same identity used by paint_trail and scene_paint's live-point. */
static inline Vec3 world_to_view_space(Vec3 world)
{
    return (Vec3){ world.x, world.y, world.z * Z_SQUASH_FACTOR };
}

/* ===================================================================== */
/* §7  state — typed wrappers around the two cycled selections           */
/* ===================================================================== */

/*
 * PresetState — typed wrapper around "which signal preset is loaded".
 *
 * INTENT
 *   The bare Preset enum is just an int; wrapping it in a struct
 *   plus cycle helpers makes the key-binding table read as a list
 *   of intentions ("cycle to next preset") rather than modular
 *   arithmetic on N_PRESETS.  Replace-Primitive-with-Object
 *   pattern (Fowler).  Critically, it makes a "next theme" key
 *   physically unable to cycle the presets[] table — the wrong
 *   index can no longer end up in the wrong slot.
 *
 * CONTEXT
 *   Owned by Scene; mutated only via preset_state_init,
 *   preset_state_cycle_next, preset_state_cycle_prev.  Read via
 *   preset_state_active (returns const RossPreset *).  The 'n'/'N'
 *   and 'p'/'P' keys are the only runtime mutation paths.  Changing
 *   the active index ALSO requires re-fitting the camera (the new
 *   preset has its own bounding-sphere extent) — handled by
 *   app_cycle_preset_next/prev which call scene_compute_geometry.
 *
 * MEMBER LOGIC
 *   current : index into the presets[] table (§1).  Must be in
 *             [0, N_PRESETS) = [0, 16).  Cycle helpers maintain
 *             this invariant via modular arithmetic on N_PRESETS.
 *
 * REFERENCES
 *   Fowler, M. — *Refactoring*, "Replace Primitive with Object".
 */
typedef struct { int current; } PresetState;

static void preset_state_init      (PresetState *p, int initial) { p->current = initial; }
static void preset_state_cycle_next(PresetState *p)              { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)              { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const RossPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }

/*
 * PaletteState — typed wrapper around "which colour theme is active".
 *
 * INTENT
 *   Same shape as PresetState, distinct type.  Bundles the active
 *   index plus the helper that re-pushes the theme to ncurses pairs.
 *   Distinct typing prevents a "next theme" callsite from
 *   accidentally cycling presets[] — the wrong index can no longer
 *   reach the wrong slot.
 *
 * CONTEXT
 *   Owned by Scene; mutated via palette_state_cycle_next/prev
 *   (t/T keys) followed immediately by scene_apply_theme, which
 *   pushes the resulting band[]/live/axis codes back into the
 *   ncurses pair table so the next frame paints with the new
 *   palette.
 *
 * MEMBER LOGIC
 *   current : index into the themes[] table (§1).  Must be in
 *             [0, N_THEMES) = [0, 10); cycle helpers maintain
 *             this invariant via modular arithmetic on N_THEMES.
 *
 * REFERENCES
 *   Fowler, M. — *Refactoring*, "Replace Primitive with Object".
 */
typedef struct { int current; } PaletteState;

static void palette_state_init      (PaletteState *p, int initial) { p->current = initial; }
static void palette_state_cycle_next(PaletteState *p)              { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p)              { p->current = (p->current + N_THEMES - 1) % N_THEMES; }
static const Theme *palette_state_active(const PaletteState *p)    { return &themes[p->current]; }
static void palette_state_apply     (const PaletteState *p)        { theme_apply(p->current); }

/* ===================================================================== */
/* §8  scene — composite owner of all mutable simulation state           */
/* ===================================================================== */

/*
 * Scene — composite owner of all mutable simulation state.
 *
 * INTENT
 *   Bundle every piece of mutable state into one struct so the App
 *   layer owns ONE Scene and the main loop drives it through a tiny
 *   named-method API (scene_init, scene_resize, scene_reset,
 *   scene_tick).  Each sub-field is its own typed named concept so
 *   the compiler enforces the boundary between physics (rossler),
 *   geometry (trail, cam), selection (preset, palette), viewport
 *   (cols/rows) and runtime (paused).
 *
 * CONTEXT
 *   Owned by App as a value.  Accessed by the main-loop helpers in
 *   §10 and the painters in §9.  All mutation goes through named
 *   scene_* / app_* helpers so call sites stay declarative.
 *
 * MEMBER LOGIC
 *   rossler   : §5 Rössler composite — invariant (a, b, c) + state
 *               (x, y, z).  Advanced by rossler_rk4_step per tick.
 *   trail     : §6 Trail ring buffer — the visible streak.
 *   cam       : §4 Camera — yaw advances with time; scale auto-fits
 *               viewport + active preset extent.
 *   preset    : §7 PresetState — which row of presets[].
 *   palette   : §7 PaletteState — which colour theme.
 *   paused    : when true, scene_tick early-returns; trail freezes.
 *   cols/rows : current viewport in cells.  Cached here so the n/p
 *               preset cycle can re-fit the camera without round-
 *               tripping through Screen.
 *
 * REFERENCES
 *   [7] Sussman & Wisdom (2014), *Structure and Interpretation of
 *       Classical Mechanics*, §1.6 — the System / State / Composite
 *       split this layer mirrors at the simulation-root level.
 */
typedef struct {
    Rossler      rossler;
    Trail        trail;
    Camera       cam;
    PresetState  preset;
    PaletteState palette;
    bool         paused;
    int          cols, rows;
} Scene;

/* scene_active_preset / scene_active_system / scene_apply_theme —
 * convenience accessors so call sites don't repeat "presets[idx]". */
static inline const RossPreset *scene_active_preset(const Scene *s)
{
    return preset_state_active(&s->preset);
}
static inline RosslerSystem scene_active_system(const Scene *s)
{
    return rossler_system_from_preset(scene_active_preset(s));
}
static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

/* scene_apply_preset — load the active preset onto the Rossler
 * composite (system + reset state to canonical seed) and clear the
 * trail so the new attractor isn't ghosted by the previous one. */
static void scene_apply_preset(Scene *s)
{
    s->rossler.system = scene_active_system(s);
    s->rossler.state  = (RosslerState){ 1.0f, 1.0f, 1.0f };
    trail_reset(&s->trail);
}

static void scene_reset(Scene *s) { scene_apply_preset(s); }

/* scene_compute_geometry — re-derive cam.scale for the current
 * viewport AND the active preset's bounding-sphere extent.  Called
 * from scene_init, scene_resize, and the n/p preset key handler so
 * every preset fills the drawable band regardless of terminal size. */
static void scene_compute_geometry(Scene *s)
{
    camera_fit_to_screen(&s->cam, s->cols, s->rows,
                         scene_active_preset(s)->extent);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->cols      = cols;
    s->rows      = rows;
    preset_state_init (&s->preset,  PRESET_CHAOS);   /* canonical default */
    palette_state_init(&s->palette, 0);              /* DEFAULT theme     */
    s->cam.yaw   = 0.0f;
    s->cam.pitch = CAM_PITCH;
    scene_compute_geometry(s);   /* sets cam.scale to fit terminal */
    scene_reset(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
    scene_compute_geometry(s);   /* re-fit on SIGWINCH */
    scene_reset(s);              /* clear trail so it redraws clean */
}

/* rossler_advance_substeps — INT_STEPS_PER_TICK fixed-step RK4
 * sub-iterations; each pushes one trail sample.  Substeps decouple
 * "physics granularity" (ROSS_DT = 10 ms) from "tick granularity"
 * (sim_fps) so the trajectory stays smooth even at low sim_fps. */
static inline void rossler_advance_substeps(Rossler *r, Trail *trail)
{
    for (int i = 0; i < INT_STEPS_PER_TICK; i++) {
        rossler_rk4_step(r, ROSS_DT);
        trail_push(trail, r->state);
    }
}

/* camera_orbit — advance yaw by CAM_YAW_RATE · dt.  Names the
 * "slowly orbiting camera" idiom so the trajectory's shape is
 * revealed from every angle over time. */
static inline void camera_orbit(Camera *cam, float dt)
{
    cam->yaw += CAM_YAW_RATE * dt;
}

/* scene_tick — advance one rendered frame of physics + camera.
 *
 * DRIVER PSEUDOCODE
 *   if paused: return
 *   rossler_advance_substeps(rossler, trail)    // INT_STEPS_PER_TICK × RK4
 *   camera_orbit(cam, dt)                       // yaw += rate · dt
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    rossler_advance_substeps(&s->rossler, &s->trail);
    camera_orbit(&s->cam, dt);
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

/*
 * Screen — terminal viewport dimensions cache.
 *
 * INTENT
 *   ncurses' COLS / LINES are global macros; caching them in a
 *   struct gives the layout code one explicit handle to read and
 *   means a resize touches a single named state.  Every painter
 *   takes Screen* (or cols/rows) explicitly so the layout math is
 *   decoupled from ncurses globals — easier to reason about and
 *   easier to test in isolation.
 *
 * CONTEXT
 *   Owned by App.  Initialised by screen_init; resized by
 *   screen_resize when SIGWINCH fires; consulted everywhere a draw
 *   needs the viewport.  Also fed into scene_init / scene_resize so
 *   the Scene's cached cols/rows stays in sync — this is what lets
 *   the n/p preset cycle re-fit the camera without round-tripping
 *   through the App layer.
 *
 * MEMBER LOGIC
 *   cols : current terminal width  in cells.
 *   rows : current terminal height in cells.
 */
typedef struct { int cols, rows; } Screen;
static void screen_init(Screen *s) { initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(); getmaxyx(stdscr, s->rows, s->cols); }
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols); }
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* viewport_center_in_drawable_band — (cx, cy) of the drawable band
 * (the rows BETWEEN HUD_TOP and HUD_BOTTOM).  Painting is centred
 * here, not at the raw terminal centre, so the HUD never overlaps. */
static inline void viewport_center_in_drawable_band(int cols, int rows,
                                                    int *cx, int *cy)
{
    *cx = cols / 2;
    *cy = (rows - HUD_BAND_RESERVED_ROWS) / 2 + HUD_TOP_ROWS;
}

/* cell_in_drawable_band — clipping predicate.  True iff (sx, sy)
 * is strictly inside the drawable band (HUD top + HUD bottom both
 * excluded).  Used by both painters as their per-cell guard. */
static inline bool cell_in_drawable_band(int sx, int sy, int cols, int rows)
{
    return sx >= 0 && sx < cols
        && sy >= HUD_TOP_ROWS && sy < rows - HUD_BOTTOM_ROWS;
}

/* paint_cell — bracket attron / mvaddch / attroff so painters can
 * place a glyph in one line instead of three.  Glyph cast handles
 * the chtype sign-extension trap (CLAUDE.md "Common ncurses Bugs"). */
static inline void paint_cell(int sy, int sx, char glyph, short pair_id)
{
    attron(COLOR_PAIR(pair_id) | A_BOLD);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair_id) | A_BOLD);
}

/* project_view_to_cell — convenience wrapper: takes a Vec3 already in
 * view space (post Z_SQUASH_FACTOR), runs it through project(), and
 * returns the cell coords.  Encapsulates the CELL_ASPECT pass-through
 * so the two painters share one call shape. */
static inline void project_view_to_cell(const Camera *cam, Vec3 v,
                                        int cx, int cy, int *sx, int *sy)
{
    project(cam, v.x, v.y, v.z, cx, cy, CELL_ASPECT, sx, sy);
}

/* paint_trail — walk the ring buffer oldest → newest, projecting each
 * sample and painting a '.' glyph in its age band's colour.
 *
 * DRIVER PSEUDOCODE
 *   if trail empty: return
 *   n = trail.count
 *   for i = 0 .. n-1:                          // oldest → newest
 *       sample_view = world_to_view_space(trail_sample_at(trail, i))
 *       (sx, sy)    = project_view_to_cell(cam, sample_view, cx, cy)
 *       if !cell_in_drawable_band(sx, sy): continue
 *       band  = trail_age_band(age = n−1−i, n)
 *       paint_cell(sy, sx, '.', PAIR_TRAIL_BASE + band)
 */
static void paint_trail(const Trail *tr, const Camera *cam,
                        int cx, int cy, int cols, int rows)
{
    if (tr->count == 0) return;
    int n = tr->count;

    for (int i = 0; i < n; i++) {
        Vec3 sample_view = world_to_view_space(trail_sample_at(tr, i));
        int  sx, sy;
        project_view_to_cell(cam, sample_view, cx, cy, &sx, &sy);
        if (!cell_in_drawable_band(sx, sy, cols, rows)) continue;

        int  age_from_newest = n - 1 - i;
        int  band            = trail_age_band(age_from_newest, n);
        paint_cell(sy, sx, '.', (short)(PAIR_TRAIL_BASE + band));
    }
}

/* scene_paint — paint the visible attractor: trail first (oldest →
 * newest, so newest dots overdraw older ones), then the '@' live
 * point on top so the user always sees "where the integrator is now".
 *
 * DRIVER PSEUDOCODE
 *   (cx, cy)   = viewport_center_in_drawable_band(cols, rows)
 *   paint_trail(trail, cam, cx, cy, cols, rows)
 *   live_view  = world_to_view_space(rossler.state)
 *   (sx, sy)   = project_view_to_cell(cam, live_view, cx, cy)
 *   if cell_in_drawable_band(sx, sy):
 *       paint_cell(sy, sx, '@', PAIR_LIVE)
 */
static void scene_paint(const Scene *s, int cols, int rows)
{
    int cx, cy;
    viewport_center_in_drawable_band(cols, rows, &cx, &cy);

    paint_trail(&s->trail, &s->cam, cx, cy, cols, rows);

    Vec3 live_view = world_to_view_space(s->rossler.state);
    int sx, sy;
    project_view_to_cell(&s->cam, live_view, cx, cy, &sx, &sy);
    if (cell_in_drawable_band(sx, sy, cols, rows))
        paint_cell(sy, sx, '@', PAIR_LIVE);
}

/* hud_label_segment_widths — column widths used by hud_param so the
 * three labels align consistently as the user cycles through presets
 * and themes (8-char names + bracket padding). */
#define HUD_PARAM_PRESET_WIDTH  19
#define HUD_PARAM_THEME_WIDTH   17

/* hud_write_title — left-aligned bold "RÖSSLER ATTRACTOR" banner. */
static inline void hud_write_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " RÖSSLER ATTRACTOR ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_write_status_right — right-aligned status text on row 0
 * (fps, sim_fps, preset name + [n/N], c). */
static inline void hud_write_status_right(int cols, double fps, int sim_fps,
                                          const Scene *s)
{
    const RossPreset *active = scene_active_preset(s);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  c:%.2f ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : active->name,
             s->preset.current + 1, (int)N_PRESETS,
             (double)active->c);

    int hx = cols - (int)strlen(buf); if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_top — row 0 of the HUD.
 *
 * DRIVER PSEUDOCODE
 *   hud_write_title()                   // left-aligned banner
 *   hud_write_status_right(...)         // right-aligned fps + preset
 */
static void hud_top(int cols, double fps, int sim_fps, const Scene *s)
{
    hud_write_title();
    hud_write_status_right(cols, fps, sim_fps, s);
}

/* hud_write_preset_label / _theme_label / _abc_yaw — three column
 * segments of row 1, each a one-line "label : value" pair.  Split so
 * hud_param() reads as a column layout, not as printf calls. */
static inline void hud_write_preset_label(int x, const RossPreset *active)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " preset:%-8s ", active->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}
static inline void hud_write_theme_label(int x, const Scene *s)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", palette_state_active(&s->palette)->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}
static inline void hud_write_abc_yaw(int x, const RossPreset *active,
                                     float yaw_radians)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " a:%.2f b:%.2f  yaw:%.1f° ",
             (double)active->a, (double)active->b,
             (double)(yaw_radians * RAD_TO_DEG));
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* hud_param — row 1 of the HUD: three labelled column segments.
 *
 * DRIVER PSEUDOCODE
 *   x = HUD_LEFT_MARGIN
 *   hud_write_preset_label(x);  x += PRESET_WIDTH
 *   hud_write_theme_label (x);  x += THEME_WIDTH
 *   hud_write_abc_yaw     (x)                       // a, b, yaw°
 */
static void hud_param(const Scene *s)
{
    const RossPreset *active = scene_active_preset(s);
    int x = HUD_LEFT_MARGIN;

    hud_write_preset_label(x, active); x += HUD_PARAM_PRESET_WIDTH;
    hud_write_theme_label (x, s);      x += HUD_PARAM_THEME_WIDTH;
    hud_write_abc_yaw     (x, active, s->cam.yaw);
}
static void hud_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:preset  t/T:theme  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{ erase(); scene_paint(s, sc->cols, sc->rows);
  hud_top(sc->cols, fps, sim_fps, s); hud_param(s); hud_hint(sc->rows); }

/* ===================================================================== */
/* §10 app — signals, resize, key dispatch, FrameClock, main loop        */
/* ===================================================================== */

/*
 * App — top-level composition root.
 *
 * INTENT
 *   The "everything else" container.  Owns the simulation (Scene),
 *   the viewport (Screen), the simulation-rate knob (sim_fps), and
 *   the two volatile signal-handler flags.  Lives as a file-scope
 *   g_app so signal handlers can touch it without indirection.
 *
 * CONTEXT
 *   Created by app_bootstrap; mutated by app_handle_key and its
 *   named mutators; torn down by screen_free + atexit cleanup.
 *
 * MEMBER LOGIC
 *   scene       : the entire simulation state (§8 Scene) — Rossler
 *                 composite + Trail + Camera + selections.
 *   screen      : cached terminal dimensions (§9 Screen).
 *   sim_fps     : fixed-timestep rate scene_tick is driven at;
 *                 clamped to [SIM_FPS_MIN, SIM_FPS_MAX] = [10, 240].
 *                 Drives TICK_NS(sim_fps) which feeds the Fiedler
 *                 accumulator in app_drain_fixed_timestep.
 *   running     : main-loop guard.  Cleared by SIGINT/TERM via
 *                 on_exit_signal, or by 'q'/ESC via app_handle_key.
 *   need_resize : SIGWINCH flag; consumed by app_handle_pending_resize
 *                 at the top of each main-loop iteration.  Both
 *                 flags MUST be volatile sig_atomic_t (signal-handler
 *                 write + main-loop read, no other synchronisation).
 *
 * REFERENCES
 *   POSIX.1-2008 §2.4.3 — signal-safe writes via sig_atomic_t.
 *   [6] Fiedler — "Fix Your Timestep!" (drives sim_fps usage).
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* main_install_signal_handlers — wire SIGINT/TERM → quit, SIGWINCH → resize. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* app_bootstrap — first-time initialisation. */
static void app_bootstrap(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
}

/* app_handle_pending_resize — rebuild screen + scene on SIGWINCH. */
static void app_handle_pending_resize(App *app)
{
    if (!app->need_resize) return;
    screen_resize(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

/* app_compute_frame_dt — wall-clock since last frame, capped to
 * SIM_MAX_FRAME_DT_MS. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > SIM_MAX_FRAME_DT_MS * NS_PER_MS)
        dt = SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    return dt;
}

/* app_drain_fixed_timestep — Fiedler [6] accumulator pattern.  Runs
 * scene_tick at the chosen sim_fps regardless of the render rate. */
static void app_drain_fixed_timestep(App *app, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* app_update_fps_meter — refresh fps every FPS_UPDATE_MS ms. */
static void app_update_fps_meter(int64_t *fps_accum, int *frame_count,
                                 double *fps_display)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_display = (double)*frame_count
                 / ((double)*fps_accum / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
}

/* app_throttle_to_render_target — sleep so render runs at 60 fps. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* app_present_frame — paint + flip back buffer. */
static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* Clamped sim-rate mutators. */
static void app_sim_rate_faster(App *app)
{
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
}
static void app_sim_rate_slower(App *app)
{
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

/* One-line mutators — named so the binding table reads as intentions. */
static void app_toggle_pause     (App *app) { app->scene.paused = !app->scene.paused; }
static void app_reset_attractor  (App *app) { scene_reset(&app->scene); }
static void app_cycle_theme_next (App *app) { palette_state_cycle_next(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_theme_prev (App *app) { palette_state_cycle_prev(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_preset_next(App *app)
{
    preset_state_cycle_next(&app->scene.preset);
    scene_compute_geometry(&app->scene);   /* re-fit camera to new extent */
    scene_reset(&app->scene);
}
static void app_cycle_preset_prev(App *app)
{
    preset_state_cycle_prev(&app->scene.preset);
    scene_compute_geometry(&app->scene);
    scene_reset(&app->scene);
}

static bool app_handle_key(App *app, int ch);

/* app_poll_keyboard — non-blocking getch + dispatch.  false = quit. */
static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* app_handle_key — key-binding table; each case calls ONE named mutator. */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            app_toggle_pause     (app); break;
    case 'r': case 'R':  app_reset_attractor  (app); break;
    case ']':            app_sim_rate_faster  (app); break;
    case '[':            app_sim_rate_slower  (app); break;
    case 't':            app_cycle_theme_next (app); break;
    case 'T':            app_cycle_theme_prev (app); break;
    case 'n': case 'N':  app_cycle_preset_next(app); break;
    case 'p': case 'P':  app_cycle_preset_prev(app); break;
    default: break;
    }
    return true;
}

/*
 * FrameClock — wall-clock + accumulator state for the main loop.
 *
 * INTENT
 *   Bundle the 5 timing locals (frame_time, sim_accum, fps_accum,
 *   frame_count, fps_display) into ONE named concept so main() reads
 *   as a pseudocode driver: "init clock, advance clock, drain
 *   simulation, throttle to render rate, present frame".  Threading
 *   them through helpers as a single FrameClock* removes 5 separate
 *   parameter lists from app_compute_frame_dt /
 *   app_drain_fixed_timestep / app_update_fps_meter.
 *
 * CONTEXT
 *   Lives as a local in main(); not owned by App because it is pure
 *   loop scaffolding (a different driver would replace it whole).
 *   Mutated by frame_clock_init, frame_clock_advance,
 *   frame_clock_reset_after_resize, app_drain_fixed_timestep, and
 *   app_update_fps_meter.
 *
 * MEMBER LOGIC
 *   frame_time  : wall-clock ns at the START of the previous frame.
 *                 app_compute_frame_dt subtracts this from clock_ns()
 *                 and stamps it forward.
 *   sim_accum   : ns of unspent simulation time.  Drained in multiples
 *                 of TICK_NS(sim_fps) by the inner while-loop of
 *                 app_drain_fixed_timestep — this is the Fiedler [6]
 *                 accumulator that lets simulation rate and render
 *                 rate move independently.
 *   fps_accum   : ns elapsed since the last fps recalculation.
 *   frame_count : frames rendered since the last fps recalculation.
 *   fps_display : the displayed fps value, refreshed every
 *                 FPS_UPDATE_MS = 500 ms so the HUD doesn't flicker.
 *
 * REFERENCES
 *   [6] Fiedler — "Fix Your Timestep!" — the accumulator pattern
 *       this struct implements.
 */
typedef struct {
    int64_t frame_time;
    int64_t sim_accum;
    int64_t fps_accum;
    int     frame_count;
    double  fps_display;
} FrameClock;

static void frame_clock_init(FrameClock *c)
{
    c->frame_time  = clock_ns();
    c->sim_accum   = 0;
    c->fps_accum   = 0;
    c->frame_count = 0;
    c->fps_display = 0.0;
}
static void frame_clock_reset_after_resize(FrameClock *c)
{
    c->frame_time = clock_ns();
    c->sim_accum  = 0;
}
static void frame_clock_advance(FrameClock *c, int64_t dt)
{
    c->sim_accum += dt;
    c->fps_accum += dt;
    c->frame_count++;
}

/* main — the whole simulation as a pseudocode driver.
 *
 * DRIVER PSEUDOCODE
 *   main_install_signal_handlers()                  // SIGINT/TERM/WINCH
 *   app_bootstrap(app)                              // ncurses + Scene
 *   clk = frame_clock_init()
 *   while running:
 *       if need_resize:
 *           app_handle_pending_resize(app)
 *           frame_clock_reset_after_resize(clk)
 *       dt = app_compute_frame_dt(&clk.frame_time)
 *       frame_clock_advance(clk, dt)
 *       app_drain_fixed_timestep(app, &clk.sim_accum)
 *       app_update_fps_meter(...)
 *       app_throttle_to_render_target(clk.frame_time, dt)
 *       app_present_frame(app, clk.fps_display)
 *       if !app_poll_keyboard(app): running = 0
 *   screen_free(app)                                // endwin via atexit
 */
int main(void)
{
    main_install_signal_handlers();

    App *app = &g_app;
    app_bootstrap(app);

    FrameClock clk;
    frame_clock_init(&clk);

    while (app->running) {
        if (app->need_resize) {
            app_handle_pending_resize(app);
            frame_clock_reset_after_resize(&clk);
        }

        int64_t dt = app_compute_frame_dt(&clk.frame_time);
        frame_clock_advance(&clk, dt);
        app_drain_fixed_timestep(app, &clk.sim_accum);
        app_update_fps_meter(&clk.fps_accum, &clk.frame_count, &clk.fps_display);

        app_throttle_to_render_target(clk.frame_time, dt);
        app_present_frame(app, clk.fps_display);

        if (!app_poll_keyboard(app)) app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
