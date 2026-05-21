/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * pulsar_rain.c — rotating pulsar beams with matrix-rain shimmer
 *
 * DEMO: A neutron-star pulsar in your terminal. N beams sweep around
 *       a single '@' core like a lighthouse, each leaving an angular
 *       wake of fading characters behind it. The wake glyphs are
 *       random ASCII letters/digits/punctuation that reroll every
 *       frame — the same matrix-rain shimmer trick from
 *       matrix_rain.c, applied to a rotating beam instead of a
 *       falling stream.
 *
 *           SINGLE-BEAM SCHEMATIC (one frame, beam pointing east)
 *           ─────────────────────────────────────────────────────
 *
 *                       k=WAKE_LEN  (oldest, FADE/DIM)
 *                       │
 *                       ▼
 *                       . . . . .
 *                      . . . . . .
 *                     . . . . . . .
 *                    . . . . . . . .
 *                   . . . . . . . . .         ←  N_RADII radial samples
 *                  . . . . . . . . . .            walk outward from core
 *                 @ . . . . . . . . . X
 *                  ▲   ↑           ↑    ↑
 *                core  HOT (k=1)   |    HEAD (k=0)
 *                                  MID (k≈WAKE_LEN/2)
 *
 *           The wake spans WAKE_LEN · WAKE_STEP = 16 · 0.05 ≈ 46°
 *           behind the head. Drawing dim slots first lets the
 *           bright HEAD always win at any cell where multiple
 *           angular slots round to the same column/row (which
 *           happens at small radius).
 *
 *           Default n_beams = 2 (classic pulsar pair, 180° apart).
 *           Press ']' to add beams (3=tri-blade, 4=cross, ...).
 *           Press '[' to remove. 1..16 evenly spaced at 360°/N.
 *
 * Study alongside:
 *   matrix_rain/matrix_rain.c       — same shimmer trick on plain
 *                                     vertical falling streams.
 *   matrix_rain/fireworks_rain.c    — shimmer on arc trails.
 *   matrix_rain/matrix_snowflake.c  — rain accumulating into snow.
 *
 * Section map:
 *   §1  config       — every tunable in one place, grouped by concept
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus 5 themed 5-shade palettes
 *   §4  scene        — Rotation + BeamGeometry + GlyphCache +
 *                       SimControls sub-structs; Scene root (rot +
 *                       geom + cache + sim + theme_idx);
 *                       scene_init / _reset / _tick / _draw +
 *                       cache_seed_random / cache_shimmer_reroll /
 *                       compute_beam_geometry / wrap_angle_to_2pi /
 *                       rotations_per_sec_to_rad_per_sec helpers;
 *                       pulsar_draw_beam decomposed into
 *                       precompute_wake_direction_table +
 *                       polar_to_screen_cell + paint_wake_slot +
 *                       paint_one_ring_dim_first; pulsar_draw_core
 *                       (the '@'); input helpers (scene_change_spin,
 *                       scene_change_beams, scene_cycle_theme);
 *                       scene_reinit_preserving_knobs (shared with
 *                       SIGWINCH path)
 *   §5  screen       — ncurses init / present; screen_draw_hud split
 *                       into hud_paint_text / format_hud_status /
 *                       draw_hud_status / draw_hud_hint
 *   §6  app          — FpsCounter + App; signals, resize, key dispatch,
 *                       variable-dt main loop (7 numbered phases)
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space / p        pause / resume
 *   r                reset (zero angle, default beam count)
 *   = / +            spin faster
 *   -                spin slower
 *   ]                add a beam (1 → 16)
 *   [                remove a beam
 *   t                cycle theme (5 themes)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra matrix_rain/pulsar_rain.c \
 *       -o pulsar_rain -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Polar-coordinate sweep with shimmer cache.
 *
 *                 (A) ROTATION — a single float `angle` advances by
 *                     `omega · dt` each frame, where omega = spin_rps
 *                     · 2π. No fixed-step accumulator, no alpha
 *                     interpolation: the float angle gives smooth
 *                     sub-cell rotation at any frame rate. Variable-
 *                     dt is unconditionally stable here because the
 *                     simulation is non-stiff (no springs, no fast
 *                     oscillators, no contact constraints).
 *
 *                 (B) BEAM   — for each beam b in 0..n_beams-1, the
 *                     base angle is `angle + b · 2π/n_beams`. A
 *                     beam consists of N_RADII radial samples
 *                     (steps outward from the core) crossed with
 *                     WAKE_LEN+1 angular wake slots (each step
 *                     trailing the head by WAKE_STEP radians). The
 *                     direction vectors for the wake_len+1 slots
 *                     are pre-computed once per beam (only 17 trig
 *                     calls per beam regardless of N_RADII), then
 *                     reused at every radial sample.
 *
 *                 (C) SHIMMER — a 2-D char cache `glyphs[N_RADII]
 *                     [WAKE_LEN+1]` holds one ASCII glyph per
 *                     (radius, wake-slot) pair. Every frame ~75% of
 *                     cells reroll, so the same beam geometry
 *                     re-renders with constantly-changing characters
 *                     — the matrix shimmer.
 *
 * Data-structure: Scene = Rotation + BeamGeometry + GlyphCache +
 *                 SimControls + theme_idx, each a named sub-struct
 *                 (see §4 for the layered-ownership diagram).
 *                 Rotation holds angle + spin_rps + n_beams;
 *                 BeamGeometry holds (cx, cy, max_r, r_step);
 *                 GlyphCache holds the N_RADII × (WAKE_LEN+1)
 *                 shimmer cache.  Everything inline; no heap
 *                 allocation post-init.
 *
 * Rendering     : For each beam, pre-compute cw[k]/sw[k] direction
 *                 vectors once, then walk N_RADII radial samples.
 *                 At each ri walk angular slots k = WAKE_LEN..0
 *                 (DIM FIRST) so the bright HEAD slot wins overlap
 *                 at small radius where multiple slots round to the
 *                 same cell. Beams paint in any order; all share
 *                 the same brightness ramp at each k. Core '@' paints
 *                 LAST so it always sits on top of every beam.
 *
 * Performance   : Per frame: O(beams · (WAKE_LEN+1) trig + beams ·
 *                 N_RADII · (WAKE_LEN+1) cells). With defaults (2
 *                 beams, 80 radii, 17 wake slots) that's 34 trig +
 *                 ~2700 mvaddch — microseconds. ncurses redraw is
 *                 the dominant cost.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 *   ── Astrophysics: what pulsars actually are ─────────────────────
 *   [1] Lyne, A. & Graham-Smith, F. (2012), "Pulsar Astronomy"
 *       (4th ed.), Cambridge University Press — the canonical
 *       reference for pulsar physics: rotating neutron stars,
 *       lighthouse model, beam radiation geometry.  Real pulsars
 *       have ONE OR TWO beams aligned with the magnetic poles; the
 *       multi-beam variants exposed via `[/]` keys are deliberate
 *       artistic license.
 *   [2] Wikipedia, "Pulsar" + "Pulsar#Lighthouse_model" — quick
 *       online reference for the lighthouse-model visual.
 *       https://en.wikipedia.org/wiki/Pulsar
 *
 *   ── Rotational kinematics (the math in scene_tick) ─────────────
 *   [3] Goldstein, H., Poole, C. & Safko, J. (2002), "Classical
 *       Mechanics" (3rd ed.), Addison-Wesley, Ch. 4 — rigid-body
 *       rotation around a fixed axis under prescribed angular
 *       velocity ω.  §4 Rotation struct + scene_tick are the
 *       minimal kinematic instance: angle = ω · t (mod 2π), no
 *       inertia, no torque, ω set directly by the user knob.
 *
 *   ── Polar-to-screen sampling (the BeamGeometry math) ───────────
 *   [4] Bresenham, J. E. (1965), "Algorithm for computer control
 *       of a digital plotter", IBM Sys. J. 4(1).  Bresenham's
 *       integer line/circle algorithms are the lineage for our
 *       discrete radial sampling at (cx + r·cosθ, cy + r·sinθ).
 *       We use float roundf instead of integer DDA because the
 *       glyph cache breaks the constant-line assumption Bresenham
 *       relies on — but the sampling pattern (concentric rings of
 *       cells) is the same idea.
 *   [5] Cell-aspect correction reference: terminal cells are
 *       ~1 column wide × 2 rows tall.  We multiply sin(θ) by
 *       ASPECT to make beams appear circular instead of vertically
 *       squished.  Same trick used across this project — see
 *       CLAUDE.md §"Coordinate / Physics" + the inline note in
 *       pulsar_draw_beam.
 *
 *   ── Shimmer cache (the visual heritage) ────────────────────────
 *   [6] *The Matrix* (1999, dir. Wachowski) — visual inspiration
 *       for the rerolling-glyph effect.  Original used mirror-image
 *       katakana; this file uses ASCII for portability across
 *       non-UTF-8 terminals (CLAUDE.md §"ASCII-Only Rendering").
 *   [7] Wikipedia, "Matrix digital rain" — Simon Whiteley's font
 *       + demoscene precedents.
 *       https://en.wikipedia.org/wiki/Matrix_digital_rain
 *
 *   ── Variable-timestep game loop ────────────────────────────────
 *   [8] Fiedler, G. (2004, updated 2014), "Fix Your Timestep!",
 *       https://gafferongames.com/post/fix_your_timestep/ — the
 *       canonical case for FIXED-step physics.  This file
 *       deliberately uses VARIABLE dt: rotation is non-stiff (no
 *       springs, no fast oscillators) and a single forward-Euler
 *       angle += ω·dt step is unconditionally stable.  Fiedler's
 *       DT_CAP rule (100 ms hard cap, DT_CAP_SEC here) is the only
 *       piece we adopt.
 *
 *   ── ncurses rendering substrate ────────────────────────────────
 *   [9] Raymond, E. S., "NCURSES Programming HOWTO" — §6 (colour
 *       pairs) for the theme-cycling design, §11 (output options)
 *       for the wnoutrefresh + doupdate diff-only write pattern
 *       that keeps the field flicker-free at 60 fps.
 *
 *   ── Companion files in this project ────────────────────────────
 *   See also:
 *     matrix_rain/matrix_rain.c     — same shimmer-cache pattern
 *       on plain VERTICAL streams (no rotation).  Read this first
 *       if the glyph-cache trick is unfamiliar; pulsar_rain.c
 *       reuses the cache idea on a radial coordinate system.
 *     matrix_rain/fireworks_rain.c  — same shimmer on ARC
 *       trajectories (rocket bursts).
 *     matrix_rain/matrix_snowflake.c — matrix rain plus an
 *       accumulator pile.  Different domain pairing, same
 *       cache-shimmer ancestor.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The pulsar is a SHEAF OF DIRECTIONS, not a pixel buffer. At any
 * instant the simulation owns N angular sweep directions (`angle`,
 * `angle + 2π/N`, ...). Each direction emits a beam consisting of
 * N_RADII radial samples (going outward from the core) and a
 * WAKE_LEN-slot angular tail (going back behind the head). Rendering
 * one beam is therefore TWO indices: ri (how far along the beam) and
 * k (how far behind the head). The head k=0 is white-bold; growing k
 * fades the colour through HOT → BRIGHT → MID → DARK → FADE while
 * also rotating the cell back through the wake. No off-screen pixel
 * buffer for the rotation — the angle alone advances; everything
 * else is recomputed every frame from cos/sin.
 *
 *
 * ALGORITHM IN STEPS  (per frame)
 * ──────────────────
 *  1. Measure dt (wall-clock seconds since last frame, capped).
 *  2. scene_tick(dt):
 *       angle += spin_rps · 2π · dt          (advance rotation)
 *       wrap angle into [0, 2π)
 *       cache_shimmer_reroll(): reroll ~75% of glyphs in the cache
 *                               (1 in SHIMMER_KEEP_ONE_IN survives)
 *  3. erase()
 *  4. scene_draw():
 *       For each beam b in 0..n_beams-1:
 *           base = angle + b · (2π / n_beams)
 *           pulsar_draw_beam(base):
 *               Pre-compute cw[k] = cos(base − k·WAKE_STEP)
 *                          sw[k] = sin(base − k·WAKE_STEP) · ASPECT
 *                                                         (k = 0..WAKE_LEN)
 *               For ri = 0..N_RADII-1:
 *                   r = (ri+1) · r_step
 *                   For k = WAKE_LEN..0:        ← DIM-FIRST is load-bearing
 *                       col = cx + round(r · cw[k])
 *                       row = cy + round(r · sw[k])
 *                       paint glyphs[ri][k] with wake_attr(k)
 *       pulsar_draw_core():
 *           paint '@' at (cx, cy) — drawn LAST, always on top
 *  5. HUD: yellow status row 0; cyan hint strip on bottom row.
 *  6. doupdate; sleep to 60-fps cap.
 *
 * KEY FORMULAS
 * ────────────
 *   omega       = spin_rps · 2π                    rad / sec
 *   angle(t+dt) = angle(t) + omega · dt
 *
 *   For each beam b:
 *     base_b = angle + b · (2π / n_beams)
 *
 *   For each wake slot k (0..WAKE_LEN):
 *     θ_k = base_b - k · WAKE_STEP
 *     cw[k] = cos θ_k
 *     sw[k] = sin θ_k · ASPECT
 *
 *   For each radial sample ri (0..N_RADII-1):
 *     r = (ri + 1) · r_step
 *     col = cx + round(r · cw[k])
 *     row = cy + round(r · sw[k])
 *
 *   Wake arc      = WAKE_LEN · WAKE_STEP = 16 · 0.05 = 0.80 rad ≈ 46°
 *   Shimmer rate  = 1 − 1/SHIMMER_KEEP_ONE_IN = 1 − 1/4 = 0.75
 *   Coverage      = WAKE_STEP · r ≈ 1 cell at r ≈ 1/STEP = 20 cells
 *                   (sets the minimum radius for solid-arc wake density)
 *   max_r         = √((cols/2)² + (rows/(2·ASPECT))²) · MAX_R_OVERSHOOT
 *
 *
 * Background you need
 * ───────────────────
 *   - matrix_rain T1-T5 (especially T3 shimmer cache).
 *   - Polar coordinates: (r, θ) → (r cos θ, r sin θ).
 *   - Terminal aspect ratio: cells are ~2× as tall as wide.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Pulsar physics (general relativity, magnetic dipoles). The
 *     "rotating beam" is purely a visual analogy.
 *   - Anti-aliasing. We deliberately rely on cell snapping.
 *   - Sub-pixel rotation. The angle advances continuously; the
 *     visual look is fine without ANY anti-aliasing.
 *
 * ─────────────────────────────────────────────────────────────────────── */


#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
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

/* ── §1.1 frame rate ──────────────────────────────────────────────── */
enum {
    TARGET_FPS = 60,                /* render and physics rate */
};

/* ── §1.2 beam geometry ───────────────────────────────────────────── */
enum {
    /* N_RADII — number of radial samples per beam. 80 is enough for
     * a continuous beam from core to far corner of any reasonable
     * terminal; smaller leaves gaps at the rim. */
    N_RADII   = 80,

    /* WAKE_LEN — angular wake depth. Each beam has WAKE_LEN+1 slots
     * (head plus tail) trailing behind it. 16 spans about 46° of arc. */
    WAKE_LEN  = 16,
};

/*
 * WAKE_STEP — angular gap between consecutive wake slots (radians).
 *   Rationale: at radius r, one slot spans r · WAKE_STEP cells.
 *   With WAKE_STEP = 0.05, slots are 1 cell wide at r = 20 cells.
 *   Smaller WAKE_STEP = thinner wake; larger = chunkier.
 *   Total wake arc = WAKE_LEN · WAKE_STEP = 16 · 0.05 = 0.8 rad ≈ 46°.
 */
#define WAKE_STEP   0.05f

/*
 * ASPECT — terminal cell height/width ratio correction.
 *   Baked into sw[k] (sin · ASPECT) so every position formula is:
 *       col = cx + r · cos θ
 *       row = cy + r · sin θ · ASPECT
 *   Without ASPECT, beams look stretched vertically because terminal
 *   cells are ~2× tall as they are wide.
 */
#define ASPECT   0.45f

/*
 * MAX_R_OVERSHOOT — fraction past the screen-corner distance that
 * we walk N_RADII radial samples to. > 1 ensures the beam reaches
 * just past the corners so no visible gap appears at the edges.
 */
#define MAX_R_OVERSHOOT  1.05f

/* ── §1.3 spin (rotations per second — physical units) ────────────── */
#define SPIN_DEFAULT_RPS  0.50f
#define SPIN_MIN_RPS      0.00f
#define SPIN_MAX_RPS      4.00f
#define SPIN_STEP_RPS     0.10f

/* ── §1.4 beam count range ────────────────────────────────────────── */
enum {
    BEAMS_MIN     =  1,
    BEAMS_DEFAULT =  2,
    BEAMS_MAX     = 16,
};

/* ── §1.5 shimmer ──────────────────────────────────────────────────── */
/* Per-frame, each glyph has a 1-in-KEEP_ONE_IN chance of surviving
 * (the rest reroll). KEEP_ONE_IN = 4 → reroll fraction = 0.75, the
 * classic Matrix-rain shimmer rate. Lower the value (smaller) to
 * make the shimmer noisier; raise it to feel frozen. */
#define SHIMMER_KEEP_ONE_IN  4

/* ── §1.6 ncurses pair IDs ────────────────────────────────────────── */
enum {
    /* 1..5 — wake bands (theme-controlled) */
    SHADE_FADE     = 1,
    SHADE_DARK,
    SHADE_MID,
    SHADE_BRIGHT,
    SHADE_HOT,

    /* 6 — beam head, always white (theme-independent) */
    SHADE_HEAD,

    /* 7 — pulsar core '@', always white (theme-independent) */
    SHADE_CORE,

    /* 8..9 — HUD spec, theme-independent */
    PAIR_HUD,
    PAIR_HINT,
};

/* ── §1.7 dt cap (spiral-of-death guard) ──────────────────────────── */
#define DT_CAP_SEC  0.10f

/* ── §1.8 timing primitives ───────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* ── §1.9 HUD layout ──────────────────────────────────────────────── */
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
 * Theme — one named 5-colour brightness palette for the beam wake.
 *
 * Intent
 *   Each beam draws a HEAD cell + WAKE_LEN trailing cells along the
 *   angular direction.  As the wake index `k` increases (0 = head,
 *   k = WAKE_LEN = oldest), the cell renders dimmer.  The mapping
 *   (k → brightness tier) happens in §4 wake_attr; this struct
 *   holds the per-theme fg colour for each tier (SHADE_FADE..HOT).
 *
 *   The `t` key cycles through THEME_COUNT presets — theme_apply()
 *   re-registers the colour pairs in one O(SHADE_COUNT) call.
 *   Each beam's PAIR ID stays the same across theme switches; only
 *   the pair's fg colour changes, so existing beams instantly adopt
 *   the new look without re-spawn.
 *
 * Why two parallel palettes (fg[] vs fg_8[])
 *   256-colour terminals can use a fine-grained brightness ramp
 *   (5 distinct cube indices per theme); 8-colour terminals can't —
 *   fg_8[] picks the closest ANSI primary, accepting that adjacent
 *   shades collapse into the same equivalence class.
 *
 * Why HEAD + CORE are theme-INDEPENDENT
 *   SHADE_HEAD (the bright leading cell of each beam wake) and
 *   SHADE_CORE (the central '@' glyph) are always WHITE regardless
 *   of theme.  Theme-tinting them would wash out the contrast
 *   against the wake tail and make the rotation centre disappear
 *   into the beams.  Hardcoded white keeps both legible across
 *   every theme.
 *
 * Themes (preset table)
 *   green   — classic Matrix (the canonical look)
 *   amber   — orange / sodium-vapour
 *   blue    — cool, cyber-noir
 *   plasma  — purple / magenta — the eponymous pulsar plasma
 *   fire    — red / orange — supernova hue
 *
 * Members
 *   name    HUD label ("green", "amber", …) shown in the status row.
 *   fg[]    5 xterm-256 fg indices ordered SHADE_FADE..SHADE_HOT
 *           (dimmest → brightest).  Indexed by the brightness tier
 *           that wake_attr maps each `k` slot to.
 *   fg_8[]  5 ANSI-8 fallback indices, same ordering.
 *
 * Invariants
 *   Every fg[i] ∈ [24, 255] per CLAUDE.md "Theme Palette Brightness"
 *   (cube 16–23 and gray 232–239 render as black on default-black
 *   terminals, so they'd make the wake tail invisible).  Values
 *   24–29 used only as the LOWEST ramp tier (oldest wake cells).
 *   fg[i] perceptually ≤ fg[i+1] (monotone bright→dim along array).
 *   name != NULL.
 *
 * References
 *   CLAUDE.md §"Theme Palette Brightness" for the cube-floor rule.
 *   Same theme-cycling design as matrix_rain.c / matrix_snowflake.c
 *   Theme docblocks — a reader who knows one knows the rest.
 */
typedef struct {
    const char *name;
    int         fg  [5];     /* 256-colour                              */
    int         fg_8[5];     /* 8-colour fallback                       */
} Theme;

static const Theme k_themes[] = {
    { "green",
      {  28,  34,  40,  46,  82 },
      { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN   } },
    { "amber",
      {  94, 130, 172, 214, 220 },
      { COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW  } },
    { "blue",
      {  24,  33,  39,  45,  51 },
      { COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN    } },
    { "plasma",
      {  53,  57,  93, 129, 201 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA } },
    { "fire",
      {  52,  88, 124, 160, 196 },
      { COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_RED     } },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static bool g_has_256 = false;

/*
 * theme_apply — bind wake bands and head/core for the chosen theme.
 * PAIR_HUD and PAIR_HINT are NEVER touched here — they carry semantic
 * meaning that must not change with theme.
 */
static void theme_apply(int idx)
{
    const int *fg = g_has_256 ? k_themes[idx].fg : k_themes[idx].fg_8;
    init_pair(SHADE_FADE,   fg[0],       COLOR_BLACK);
    init_pair(SHADE_DARK,   fg[1],       COLOR_BLACK);
    init_pair(SHADE_MID,    fg[2],       COLOR_BLACK);
    init_pair(SHADE_BRIGHT, fg[3],       COLOR_BLACK);
    init_pair(SHADE_HOT,    fg[4],       COLOR_BLACK);
    init_pair(SHADE_HEAD,   COLOR_WHITE, COLOR_BLACK);
    init_pair(SHADE_CORE,   COLOR_WHITE, COLOR_BLACK);
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

/*
 * wake_attr — map angular distance k from beam head to an ncurses
 * attr (colour pair + bold/normal/dim).
 *
 *   k=0           HEAD     white     BOLD
 *   k=1           HOT      theme[4]  BOLD
 *   k=2           BRIGHT   theme[3]  BOLD
 *   k=3..N/2      MID      theme[2]  NORMAL
 *   k=N/2+1..N-2  DARK     theme[1]  NORMAL
 *   k=N-1..N      FADE     theme[0]  DIM
 */
static attr_t wake_attr(int k)
{
    if (k == 0)              return COLOR_PAIR(SHADE_HEAD)   | A_BOLD;
    if (k == 1)              return COLOR_PAIR(SHADE_HOT)    | A_BOLD;
    if (k == 2)              return COLOR_PAIR(SHADE_BRIGHT) | A_BOLD;
    if (k <= WAKE_LEN / 2)   return COLOR_PAIR(SHADE_MID);
    if (k <= WAKE_LEN - 2)   return COLOR_PAIR(SHADE_DARK);
    return COLOR_PAIR(SHADE_FADE) | A_DIM;
}

/* ===================================================================== */
/* §4  scene — Rotation + BeamGeometry + GlyphCache + SimControls         */
/* ===================================================================== */

/* ── §4.1 ASCII glyph pool + tiny utility ─────────────────────────── */

static const char k_glyphs[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789!@#$%^&*()-_=+[]{}|;:,.<>?/~`";

#define GLYPHS_LEN (int)(sizeof k_glyphs - 1)

static char rand_glyph(void) { return k_glyphs[rand() % GLYPHS_LEN]; }

/* ── §4.2 Scene sub-structs ───────────────────────────────────────── */

/*
 * Rotation — the spinning state of the pulsar's beam array.
 *
 * Intent
 *   Models the pulsar as a RIGID ROTATION around the screen centre:
 *   `angle` is a single scalar tracking where beam 0 currently points;
 *   the other beams sit at angle + b · 2π/n_beams.  spin_rps gives
 *   the angular rate in physical units (rotations per second, NOT
 *   radians/sec — easier for the user to verify with a stopwatch).
 *
 *   scene_tick advances `angle` by ω · dt where ω = spin_rps · 2π,
 *   then wraps into [0, 2π).
 *
 * Members
 *   angle      Beam-zero angle in radians, wrapped to [0, 2π).
 *   spin_rps   Spin rate in ROTATIONS per second (NOT rad/sec).
 *              User-tunable via +/- keys; clamped to
 *              [SPIN_MIN_RPS, SPIN_MAX_RPS].
 *   n_beams    Number of beams, evenly spaced at 2π/n_beams.
 *              User-tunable via [/] keys; clamped to
 *              [BEAMS_MIN, BEAMS_MAX].
 *
 * Invariants
 *   0 ≤ angle < 2π   (enforced each scene_tick).
 *   SPIN_MIN_RPS ≤ spin_rps ≤ SPIN_MAX_RPS.
 *   BEAMS_MIN ≤ n_beams ≤ BEAMS_MAX.
 *
 * References
 *   Goldstein, "Classical Mechanics" Ch. 4 — rigid-body rotation
 *   under a constant angular velocity ω.  This struct is the
 *   minimal kinematic state (no inertia, no torque) — the spin is
 *   prescribed directly by the user knob.
 */
typedef struct {
    float angle;
    float spin_rps;
    int   n_beams;
} Rotation;

/*
 * BeamGeometry — the radial sampling geometry, derived from screen
 * extent at scene_init time.
 *
 * Intent
 *   Each beam is rendered as N_RADII radial samples outward from
 *   (cx, cy), spaced by r_step.  max_r is the FURTHEST sample — sized
 *   so beams reach the screen corner (and a little beyond, set by
 *   MAX_R_OVERSHOOT, to hide the discrete-radius artefact at the
 *   visible edge).
 *
 *   Recomputed in scene_init (and again in app_do_resize) from the
 *   current Screen extent.  Frozen for the duration of one screen size.
 *
 * Why ASPECT is baked into y-sampling (not into max_r)
 *   Terminal cells are ~1 column wide × 2 rows tall.  Without
 *   correction, sin(angle)·r would compute a y-offset in CELL units
 *   that looks elliptical instead of circular.  max_r is computed in
 *   ISOTROPIC units (hy divided by ASPECT during the corner-distance
 *   calculation) so the radius extends symmetrically; ASPECT is then
 *   multiplied back into sin(angle) at draw time to map back to
 *   anisotropic cells.  See pulsar_draw_beam for the sin·ASPECT.
 *
 * Members
 *   cx, cy     Screen-cell coordinates of the rotation centre
 *              (= cols/2, rows/2 at init).
 *   max_r      Farthest radius any beam reaches, in isotropic units
 *              (corner-of-screen distance + MAX_R_OVERSHOOT padding).
 *   r_step     max_r / N_RADII — gap between radial samples.
 *
 * Invariants
 *   max_r > 0; r_step > 0 (positive radii after init).
 *   cx ∈ [0, screen.cols), cy ∈ [0, screen.rows) at init time
 *   (off-screen positions are skipped by the renderer's clip check).
 *
 * References
 *   The ASPECT correction trick is the same one matrix_rain.c uses
 *   for sub-cell head precision — see CLAUDE.md §"Coordinate /
 *   Physics" and the inline note in pulsar_draw_beam.
 */
typedef struct {
    int   cx, cy;
    float max_r;
    float r_step;
} BeamGeometry;

/*
 * GlyphCache — the shimmer cache for the pulsar beams.
 *
 * Intent
 *   ONE shared cache across all beams (not n_beams × this array)
 *   because the eye can't track WHICH beam is showing which character
 *   — only the per-(radius, wake-slot) pattern matters visually.
 *   Allocating per-beam would waste memory + cache footprint with no
 *   visible benefit.
 *
 *   Each tick, cache_shimmer_reroll picks ~75 % of slots and rerolls
 *   them (1-in-SHIMMER_KEEP_ONE_IN survive).  Slots that don't
 *   change still visibly "move" as the rotating beam paints them
 *   at a new screen position — the perceived motion is the BEAM
 *   sweeping, the perceived shimmer is the cache reroll.
 *
 * Members
 *   glyphs[ri][k]   ASCII glyph for radius index ri and wake slot k.
 *                   ri ∈ [0, N_RADII); k ∈ [0, WAKE_LEN] inclusive
 *                   (head + WAKE_LEN trailing slots → WAKE_LEN+1
 *                   columns).
 *
 * References
 *   [1] Wachowski film — the shimmer-cache trick is matrix_rain.c's
 *       lineage, applied here to radial sampling instead of vertical
 *       lanes.  Same idea, different coordinate system.
 */
typedef struct {
    char glyphs[N_RADII][WAKE_LEN + 1];
} GlyphCache;

/*
 * SimControls — user-facing playback knob.
 *
 * Intent
 *   This demo has ONE pure playback knob (paused).  Other user-
 *   controllable parameters live in Rotation (spin_rps, n_beams) or
 *   on Scene directly (theme_idx) because they're tied to specific
 *   semantic domains.  Bundling them all here would obscure that
 *   distinction.
 *
 * Members
 *   paused   true → scene_tick early-returns; rotation freezes and
 *            the shimmer cache stops rerolling.  Toggled by SPACE/p.
 */
typedef struct {
    bool paused;
} SimControls;

/*
 * Scene — owns ALL simulation + render state for one run.
 *
 * Layered ownership
 *
 *     Scene
 *       ├── rot        : Rotation       ← angle + spin + n_beams
 *       ├── geom       : BeamGeometry   ← cx/cy + max_r + r_step
 *       ├── cache      : GlyphCache     ← per-(r, k) ASCII glyphs
 *       ├── sim        : SimControls    ← paused
 *       └── theme_idx  : int            ← active colour palette
 *
 *   Every persistent value reachable from one `Scene *s`.  Helpers
 *   that touch only one sub-domain take just that sub-struct (e.g.
 *   cache_shimmer_reroll takes `GlyphCache *`), which makes the
 *   "what does this function touch" question answerable from the
 *   signature.
 *
 * Why no World sub-struct (unlike matrix_snowflake.c)
 *   Pulsar geometry is centre + radii, NOT a cell grid.  The Screen
 *   extent is read ONCE in scene_init to compute (cx, cy, max_r,
 *   r_step) — after that, the sim doesn't need cols/rows; only the
 *   renderer's clip check does, and it gets them from app->screen
 *   via pulsar_draw's parameters.
 */
typedef struct {
    Rotation     rot;
    BeamGeometry geom;
    GlyphCache   cache;
    SimControls  sim;
    int          theme_idx;
} Scene;

/* ── §4.3 cache helpers — fill + shimmer-reroll the glyph cache ──── */

/* cache_seed_random — fill EVERY slot with a fresh random glyph.
 * Used by scene_init / scene_reset to give the very first frame a
 * fully-populated cache. */
static void cache_seed_random(GlyphCache *cache) {
    for (int ri = 0; ri < N_RADII; ri++)
        for (int k = 0; k <= WAKE_LEN; k++)
            cache->glyphs[ri][k] = rand_glyph();
}

/* cache_shimmer_reroll — per-tick reroll.  Each slot has a
 * (SHIMMER_KEEP_ONE_IN − 1)/SHIMMER_KEEP_ONE_IN probability of being
 * re-rolled to a fresh random glyph (KEEP_ONE_IN = 4 → ~75 %
 * rerolled per tick).  Slots that survive look "still" while the
 * rotating beam sweeps past them; rerolled slots produce the
 * matrix shimmer. */
static void cache_shimmer_reroll(GlyphCache *cache) {
    for (int ri = 0; ri < N_RADII; ri++)
        for (int k = 0; k <= WAKE_LEN; k++)
            if (rand() % SHIMMER_KEEP_ONE_IN != 0)
                cache->glyphs[ri][k] = rand_glyph();
}

/* ── §4.4 geometry — derive (cx, cy, max_r, r_step) from screen ──── */

/*
 * compute_beam_geometry — derive the radial sampling geometry from
 * the screen extent.
 *
 *   centre        (cols/2, rows/2)
 *   half-width    hx = cols / 2
 *   half-height   hy = rows / 2 / ASPECT     (isotropic units; see
 *                 BeamGeometry docblock for why we divide here)
 *   max_r         √(hx² + hy²) · MAX_R_OVERSHOOT   (corner distance
 *                 with overshoot padding to hide the discrete-radius
 *                 artefact at the visible edge)
 *   r_step        max_r / N_RADII
 */
static void compute_beam_geometry(BeamGeometry *geom, int cols, int rows) {
    geom->cx = cols / 2;
    geom->cy = rows / 2;

    float screen_half_width_iso  = (float)cols * 0.5f;
    float screen_half_height_iso = (float)rows * 0.5f / ASPECT;
    float corner_distance_iso = sqrtf(screen_half_width_iso  *
                                       screen_half_width_iso
                                     + screen_half_height_iso *
                                       screen_half_height_iso);
    geom->max_r  = corner_distance_iso * MAX_R_OVERSHOOT;
    geom->r_step = geom->max_r / (float)N_RADII;
}

/* ── §4.5 scene_init / scene_reset ────────────────────────────────── */

/*
 * scene_init — full reset: rotation defaults + geometry + cache
 * + UI defaults.  Called at startup AND on resize (app_do_resize
 * preserves user knobs across the call).  scene_reset is the
 * lighter version called by the user's 'r' key (keeps spin / theme).
 */
static void scene_init(Scene *s, int cols, int rows) {
    /* (1) Rotation defaults — user knobs at startup values */
    s->rot.angle    = 0.0f;
    s->rot.spin_rps = SPIN_DEFAULT_RPS;
    s->rot.n_beams  = BEAMS_DEFAULT;

    /* (2) BeamGeometry — derived from screen extent */
    compute_beam_geometry(&s->geom, cols, rows);

    /* (3) GlyphCache — fully populated so frame 1 has glyphs */
    cache_seed_random(&s->cache);

    /* (4) SimControls + theme defaults */
    s->sim.paused = false;
    s->theme_idx  = 0;
}

/*
 * scene_reset — user-triggered 'r' key.  Wipes the cache + zeros
 * the rotation + resets n_beams, but PRESERVES spin_rps + theme_idx
 * because those are user knobs the user wouldn't expect to flip
 * back on a reset.
 */
static void scene_reset(Scene *s) {
    s->rot.angle   = 0.0f;
    s->rot.n_beams = BEAMS_DEFAULT;
    cache_seed_random(&s->cache);
}

/* ── §4.6 scene_tick — advance angle, wrap, then shimmer ──────────── */

/* wrap_angle_to_2pi — keep `angle` in [0, 2π).  while-loop handles
 * arbitrarily large positive or negative arguments (a one-shot
 * fmodf would be cheaper for huge dt, but the while loop is clearer
 * and dt is capped at DT_CAP_SEC so the loop body runs at most a
 * few times). */
static inline void wrap_angle_to_2pi(float *angle) {
    while (*angle >= 2.0f * (float)M_PI) *angle -= 2.0f * (float)M_PI;
    while (*angle <  0.0f)               *angle += 2.0f * (float)M_PI;
}

/* rotations_per_sec_to_rad_per_sec — name for the 2π · rps unit
 * conversion.  Lets scene_tick read "ω = rotations_per_sec_to_rad…"
 * instead of an inline `* 2.0f * M_PI` magic expression. */
static inline float rotations_per_sec_to_rad_per_sec(float rps) {
    return rps * 2.0f * (float)M_PI;
}

/*
 * scene_tick — one per-frame physics step.
 *
 *   ω           = spin_rps · 2π     (rad/sec)
 *   angle      += ω · dt            (forward Euler — angle is the
 *                                    only state, no momentum)
 *   wrap angle into [0, 2π)
 *   reroll the glyph cache (shimmer)
 *
 * When paused, returns early — rotation AND shimmer both freeze.
 */
static void scene_tick(Scene *s, float dt) {
    if (s->sim.paused) return;

    float omega_rad_per_sec = rotations_per_sec_to_rad_per_sec(s->rot.spin_rps);
    s->rot.angle += omega_rad_per_sec * dt;
    wrap_angle_to_2pi(&s->rot.angle);

    cache_shimmer_reroll(&s->cache);
}

/* ── §4.7 pulsar_draw_beam — one beam + its angular wake ─────────── */

/*
 * precompute_wake_direction_table — fill (cos, sin·ASPECT) lookup
 * tables for the WAKE_LEN+1 angular slots of one beam.
 *
 * For slot k (k=0 is the head, increasing k = trailing into the past):
 *   wake_angle = base_angle − k · WAKE_STEP
 *   cw[k]      = cos(wake_angle)
 *   sw[k]      = sin(wake_angle) · ASPECT    (cell-aspect correction
 *                                              baked in here so the
 *                                              draw loop multiplies
 *                                              once instead of N_RADII
 *                                              times)
 *
 * Pre-computing the table is the per-beam optimisation: we need
 * only (WAKE_LEN+1) trig pairs regardless of N_RADII radial samples.
 *
 * Why cell-aspect correction lives in sw[] (not cw[])
 *   Terminal cells are ~1 col × 2 rows.  Multiplying sin(θ) by
 *   ASPECT < 1 compresses the y-component, making circles appear
 *   circular instead of vertically squished.  cos(θ) is along x,
 *   which doesn't need correction.  See BeamGeometry doc + [5] for
 *   the full ASPECT story.
 */
static inline void precompute_wake_direction_table(float base_angle,
                                                    float cw[WAKE_LEN + 1],
                                                    float sw[WAKE_LEN + 1]) {
    for (int k = 0; k <= WAKE_LEN; k++) {
        float wake_angle = base_angle - (float)k * WAKE_STEP;
        cw[k] = cosf(wake_angle);
        sw[k] = sinf(wake_angle) * ASPECT;    /* aspect baked in */
    }
}

/*
 * polar_to_screen_cell — map (radius, slot direction) to a discrete
 * screen cell.  Bresenham [4] would do this with integer DDA; we
 * use float roundf because each polar point is INDEPENDENT (no
 * constant-line assumption — different (ri, k) yield non-collinear
 * cells in general).  Rounding is round-half-toward-even per IEEE,
 * which is fine here because we don't need ties to be deterministic
 * — overlapping cells get the brightest tier via the dim-first
 * paint order below.
 */
static inline void polar_to_screen_cell(const BeamGeometry *geom,
                                         float r, float cw, float sw,
                                         int *out_col, int *out_row) {
    *out_col = geom->cx + (int)roundf(r * cw);
    *out_row = geom->cy + (int)roundf(r * sw);
}

/*
 * paint_wake_slot — paint ONE wake cell with the correct attr,
 * dropping it silently if it falls off-screen.
 */
static inline void paint_wake_slot(const Scene *s, int ri, int k,
                                    int col, int row, int cols, int rows) {
    if (col < 0 || col >= cols || row < 0 || row >= rows) return;
    attr_t attr = wake_attr(k);
    attron(attr);
    mvaddch(row, col, (chtype)(unsigned char)s->cache.glyphs[ri][k]);
    attroff(attr);
}

/*
 * paint_one_ring_dim_first — paint all WAKE_LEN+1 wake slots at
 * radius index ri, in DESCENDING k order (oldest/dimmest first,
 * head LAST).
 *
 * Why dim-first painter's order
 *   At small radius, multiple wake slots can round to the SAME cell.
 *   We want the BRIGHT HEAD (k=0) to win those collisions, so it
 *   paints last and overwrites the dimmer slots underneath.
 */
static inline void paint_one_ring_dim_first(const Scene *s, int ri,
                                             const float cw[WAKE_LEN + 1],
                                             const float sw[WAKE_LEN + 1],
                                             int cols, int rows) {
    float ring_radius = (float)(ri + 1) * s->geom.r_step;
    for (int k = WAKE_LEN; k >= 0; k--) {
        int col, row;
        polar_to_screen_cell(&s->geom, ring_radius, cw[k], sw[k], &col, &row);
        paint_wake_slot(s, ri, k, col, row, cols, rows);
    }
}

/*
 * pulsar_draw_beam — paint ONE rotating beam at `base_angle`
 * (head + WAKE_LEN trailing slots, across N_RADII radial samples).
 *
 *   (1) precompute the wake direction table — WAKE_LEN+1 (cos, sin·ASPECT)
 *       pairs, evaluated ONCE per beam regardless of N_RADII.
 *   (2) for each radial sample ri, paint that ring's WAKE_LEN+1 cells
 *       in dim-first order so the bright head wins overlap.
 *
 * Complexity per beam: O(WAKE_LEN) trig + O(N_RADII · WAKE_LEN) cells.
 */
static void pulsar_draw_beam(const Scene *s, float base_angle,
                             int cols, int rows)
{
    /* (1) wake direction table — WAKE_LEN+1 (cos, sin·ASPECT) pairs */
    float cw[WAKE_LEN + 1], sw[WAKE_LEN + 1];
    precompute_wake_direction_table(base_angle, cw, sw);

    /* (2) walk N_RADII rings outward; paint each ring dim-first */
    for (int ri = 0; ri < N_RADII; ri++)
        paint_one_ring_dim_first(s, ri, cw, sw, cols, rows);
}

/* ── §4.8 pulsar_draw_core — the '@' on top of everything ─────────── */

static void pulsar_draw_core(const Scene *s, int cols, int rows)
{
    int cx = s->geom.cx, cy = s->geom.cy;
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    attron(COLOR_PAIR(SHADE_CORE) | A_BOLD);
    mvaddch(cy, cx, '@');
    attroff(COLOR_PAIR(SHADE_CORE) | A_BOLD);
}

/* ── §4.9 scene_draw — orchestrator: N beams + core last ─────────── */

static void scene_draw(const Scene *s, int cols, int rows)
{
    /* N beams evenly spaced at 2π/N. Order between beams doesn't
     * matter because they share the same brightness ramp at each k. */
    float beam_angle_step = 2.0f * (float)M_PI / (float)s->rot.n_beams;
    for (int b = 0; b < s->rot.n_beams; b++)
        pulsar_draw_beam(s, s->rot.angle + (float)b * beam_angle_step,
                         cols, rows);

    /* Core LAST — always wins overlap. */
    pulsar_draw_core(s, cols, rows);
}

/* ── §4.10 input helpers (used by app_handle_key) ─────────────────── */

static void scene_change_spin(Scene *s, float delta_rps)
{
    s->rot.spin_rps += delta_rps;
    if (s->rot.spin_rps < SPIN_MIN_RPS) s->rot.spin_rps = SPIN_MIN_RPS;
    if (s->rot.spin_rps > SPIN_MAX_RPS) s->rot.spin_rps = SPIN_MAX_RPS;
}

static void scene_change_beams(Scene *s, int delta)
{
    s->rot.n_beams += delta;
    if (s->rot.n_beams < BEAMS_MIN) s->rot.n_beams = BEAMS_MIN;
    if (s->rot.n_beams > BEAMS_MAX) s->rot.n_beams = BEAMS_MAX;
}

static void scene_cycle_theme(Scene *s)
{
    s->theme_idx = (s->theme_idx + 1) % THEME_COUNT;
    theme_apply(s->theme_idx);
}

/*
 * scene_reinit_preserving_knobs — re-run scene_init at a new extent,
 * preserving the user-tuned knobs that SHOULD survive a re-init
 * (spin_rps, n_beams, theme_idx, paused).
 *
 * Used by app_do_resize (SIGWINCH: new screen size, same knobs).
 * scene_reset (the 'r' key) deliberately does NOT use this — the
 * user wants the angle + cache reset but the knobs preserved
 * already, and we don't want to recompute geometry at the same
 * extent (would be wasteful).
 *
 * Centralising the snapshot/restore dance keeps the "which knobs
 * survive a resize" list in one place — any future knob added here
 * is automatically preserved by the SIGWINCH path.
 */
static void scene_reinit_preserving_knobs(Scene *s, int new_cols, int new_rows) {
    /* (1) snapshot user-tuned knobs */
    float saved_spin   = s->rot.spin_rps;
    int   saved_beams  = s->rot.n_beams;
    int   saved_theme  = s->theme_idx;
    bool  saved_paused = s->sim.paused;

    /* (2) full re-init (resets EVERYTHING including knobs) */
    scene_init(s, new_cols, new_rows);

    /* (3) restore preserved knobs */
    s->rot.spin_rps = saved_spin;
    s->rot.n_beams  = saved_beams;
    s->theme_idx    = saved_theme;
    s->sim.paused   = saved_paused;
}

/* ===================================================================== */
/* §5  screen — ncurses init / present / HUD                              */
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
 *   Scene.geom.{cx, cy, max_r, r_step} ARE DERIVED from these
 *   dimensions (see compute_beam_geometry in §4).  The simulation
 *   then doesn't need cols/rows anymore — only the renderer does,
 *   for its clip-to-screen check.  Cols/rows flow into pulsar_draw_*
 *   as explicit parameters.
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
 *   rows   Terminal height in CELLS.  Bottom row index is rows − 1
 *          (where the key hint paints).  Row 0 is the status row.
 *
 * Invariants
 *   cols > 0, rows > 0 after screen_init.
 *   Both refreshed on every SIGWINCH via screen_resize +
 *   app_do_resize (which also re-runs compute_beam_geometry at the
 *   new extent).
 */
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

/* hud_paint_text — attron / mvprintw / attroff sandwich shared by
 * both HUD rows.  Centralises the colour-pair setup. */
static inline void hud_paint_text(int row, int col, int pair, const char *text) {
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* format_hud_status — write the top-row status string into `buf`.
 * Reports fps, spin (rps), beam count + pluralisation, theme name,
 * paused state. */
static void format_hud_status(const Scene *s, double fps,
                              char *buf, size_t buflen) {
    const char *beam_suffix = (s->rot.n_beams == 1) ? " " : "s";
    snprintf(buf, buflen,
             " %5.1f fps  %5.2f rps  %d beam%s  [%s] %s ",
             fps, (double)s->rot.spin_rps, s->rot.n_beams,
             beam_suffix,
             k_themes[s->theme_idx].name,
             s->sim.paused ? "PAUSED " : "running");
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
        " q:quit  spc:pause  r:reset  +/-:spin  []:beams  t:theme ";
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
static void screen_draw_hud(const Screen *sc, double fps, const Scene *s)
{
    draw_hud_status(sc, s, fps);
    draw_hud_hint  (sc);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §6  app — signals, resize, variable-dt main loop                       */
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
 *   scene         simulation + render state (Rotation + Geometry +
 *                  GlyphCache + SimControls + theme_idx)
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
 * app_do_resize — handle a pending SIGWINCH.
 *
 *   (1) refresh the Screen extent (ncurses getmaxyx)
 *   (2) re-init the Scene at the new extent, preserving user knobs
 *   (3) clear the resize flag so main resumes normal frames
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);                                 /* (1) */
    scene_reinit_preserving_knobs(&app->scene,                   /* (2) */
                                   app->screen.cols,
                                   app->screen.rows);
    app->need_resize = 0;                                        /* (3) */
}

/*
 * app_handle_key — process one keystroke.  Returns false on quit.
 *
 *   q / Q / ESC      quit
 *   SPACE / p        toggle paused
 *   r                reset (zero angle + reseed cache; keep knobs)
 *   + / =            spin faster
 *   -                spin slower
 *   ]                more beams
 *   [                fewer beams
 *   t / T            cycle theme
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ': case 'p': case 'P': s->sim.paused = !s->sim.paused;  break;

    case 'r': case 'R':           scene_reset(s);                 break;

    case '=': case '+':           scene_change_spin (s, +SPIN_STEP_RPS); break;
    case '-':                     scene_change_spin (s, -SPIN_STEP_RPS); break;

    case ']':                     scene_change_beams(s, +1);       break;
    case '[':                     scene_change_beams(s, -1);       break;

    case 't': case 'T':           scene_cycle_theme (s);           break;

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

    App   *app   = &g_app;
    Scene *scene = &app->scene;
    app->running = 1;
    fps_counter_init(&app->fps);

    screen_init(&app->screen);
    g_has_256 = (COLORS >= 256);
    scene_init(scene, app->screen.cols, app->screen.rows);
    theme_apply(scene->theme_idx);
    hud_pairs_init();

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
        for (int ch; (ch = getch()) != ERR; ) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }

        /* (4) advance scene (rotation + shimmer) */
        scene_tick(scene, dt);

        /* (5) rolling-window fps counter */
        fps_counter_tick(&app->fps, dt_ns);

        /* (6) draw + present */
        erase();
        scene_draw(scene, app->screen.cols, app->screen.rows);
        screen_draw_hud(&app->screen, app->fps.display, scene);
        screen_present();

        /* (7) frame cap — sleep before the NEXT frame's I/O */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
