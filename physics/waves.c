/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * waves.c — Scalar 2-D wave field, two engines side-by-side
 *
 * DEMO   : Interference patterns on the terminal grid, computed two ways.
 *           Press `m` to toggle between:
 *
 *             FDTD       — Finite-Difference Time-Domain numerical integration
 *                          of  ∂²u/∂t² = c² ∇²u.  Three time levels stored,
 *                          inner stencil + hard-wall reflections + damping.
 *                          Sources DRIVE the field by adding A·sin(ωt+φ_0)
 *                          to their cell every sub-step.
 *
 *             ANALYTIC   — Closed-form superposition u = Σ_i sin(ω_i t − k_i
 * r_i + φ_0i). No grid state, no CFL stability worry, no warm-up. Phase term
 * k_i·r_i − φ_0i  is cached per source per cell so the hot path is N_active ×
 * sinf() per cell.
 *
 *          Same source list feeds both engines.  Same themes, same HUD.  Five
 *          presets (1–5) reseat the source array into known interference
 *          configurations: double-slit, ripple tank, beat, radial star,
 *          five-oscillator default.
 *
 * SECTIONS §1 config  §2 clock  §3 color  §4 fdtd-grid  §5 sources
 *          §6 analytic §7 scene  §8 screen §9 app
 *
 * KEYS
 *   q / ESC    quit
 *   m          switch engine (FDTD ↔ analytic)
 *   space      pause / resume
 *   r          reset / re-apply current preset
 *   1–5        preset: 1 DBL-SLIT  2 RIPPLE  3 BEAT  4 RADIAL  5 FIVE-OSC
 *   t / T      next / previous theme  (10 themes)
 *   TAB / BTAB select source (cycle forward / back)
 *   arrows     move selected source by 1 cell
 *   n          add new source at centre   x   delete selected
 *   + / -      analytic: selected source wavelength  λ
 *              FDTD    : FDTD sub-steps per render frame
 *   f / F      selected source angular frequency  ω
 *   d / D      FDTD damping coefficient
 *   p          FDTD: drop a Gaussian impulse at centre
 *
 * BUILD
 *   gcc -std=c11 -O2 -Wall -Wextra physics/waves.c -o waves -lncurses -lm
 *
 * SCOPE NOTE
 *   This file is ~1500 lines — larger than the 250-450 phase-1 target —
 *   because it deliberately carries TWO engines + rich pedagogical
 *   commentary.  The contrast (numerical vs analytic, same physics, two
 *   solvers) IS the teaching value.  §4 (FDTD) and §6 (analytic) are
 *   independent ~80-line modules; §5 (sources) and §7 (scene) are the
 *   glue.  Splitting would lose the side-by-side comparison.
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * System architecture
 *
 *           USER KEYS / SIGNALS
 *                   |
 *                   v
 *          +------------------+
 *          |  App + main()    |   §9   signal-driven loop
 *          +--------+---------+
 *                   | owns
 *                   v
 *          +------------------+
 *          |  Scene           |   §7   ←─ the integration point
 *          +--+------+--------+
 *             |      |        \---- runs ----+
 *             v      v                       v
 *          Screen   Grid              Engine = FDTD or ANALYTIC
 *           §8      §4                       |
 *          ncurses  FDTD field      +--------+--------+
 *                                   |                 |
 *                                   v                 v
 *                            (reads from)        (reads from)
 *                            g_src[] §5          g_src[]   §5
 *                                                g_at      §6 phase cache
 *
 *   Scene OWNS Grid and Screen.  Source list (g_src[]) and analytic
 *   phase cache (g_at) are file-scope globals shared by both engines.
 *
 * Reading paths through this file
 *
 *   FIRST READ (the overview tour, ~5 min):
 *     this header → CONCEPTS → MENTAL MODEL → §1 config → §7 Scene struct
 *
 *   FDTD path  (how does the numerical PDE solver work?):
 *     §4 Grid struct → grid_tick orchestrator
 *       └─ laplacian_4point_cross  +  wave_central_difference_step
 *       └─ sweep_interior_with_central_diff
 *       └─ rotate_time_level_buffers
 *     §7 scene_fdtd_inject  →  tick_fdtd_engine
 *
 *   ANALYTIC path  (how does the closed-form solver work?):
 *     §5 Source struct  →  §6 AnalyticTable struct
 *     §6 analytic_recompute  →  analytic_field
 *     §7 tick_analytic_engine
 *
 *   RENDERING path  (how does drawing work?):
 *     §3 Theme  +  amplitude_level  +  level_attr   (signed glyph ramp)
 *     §7 paint_field_cell  →  scene_draw_field / scene_draw_sources
 *     §8 hud_draw  →  screen_draw  →  screen_present
 *
 *   UI / PRESETS path  (how do user keys reshape the simulation?):
 *     §5 source_* helpers  →  preset_apply  →  seed_double_slit / …
 *     §9 app_handle_key (m, 1-5, TAB, arrows, n, x, +/-, f/F, d/D, p)
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two solvers for the same scalar 2-D wave PDE.
 *                    (1) FDTD — explicit 2nd-order central differences:
 *                        u_new = 2·u − u_prev + C²·(Σ neighbours − 4u)
 *                        then multiplied by damping ∈ (0,1) to dissipate
 * energy. (2) ANALYTIC — closed-form Σ sin(ω t − k r + φ_0). k = 2π / (λ ·
 * CELL_W) ; r = pixel-space distance.
 *
 * Physics        : Linear scalar wave equation  ∂²u/∂t² = c² ∇²u.
 *                  Each source is a cylindrical Huygens emitter.
 *                  Superposition (linearity) gives interference patterns:
 *                  constructive when crests align (u ≈ N·A), destructive
 *                  when crest meets trough (u ≈ 0 → nodal lines).
 *
 * Stability      : FDTD only.  CFL: c·√2 ≤ 1 → c ≤ 0.707.
 *                  Here C_SPEED=0.45 → CFL=0.636 (stable margin).
 *                  Analytic is unconditionally stable — no time integration.
 *
 * Performance    : FDTD: O(W·H·STEPS) per render — STEPS sub-steps decouples
 *                  visual speed from CFL.  ~150 K ops at 200×60.
 *                  Analytic: O(N_active·W·H) per render — Σ sinf() per cell.
 *                  Phase table precomputed once per source-move amortises r.
 *
 * References     :
 *   ── Wave physics / interference ──────────────────────────────────
 *    [1] Crawford, F. S., "Waves", Berkeley Physics Course Vol. III
 *        (McGraw-Hill 1968).  Best intuitive intro to the linear scalar
 *        wave equation, superposition, beats, and standing waves.
 *    [2] Hecht, E., "Optics", 5th ed. (Pearson 2017), ch. 7 & 9 —
 *        Huygens-Fresnel principle, double-slit fringes, coherence;
 *        directly explains presets 1 (DBL-SLIT) and 2 (RIPPLE).
 *    [3] Born, M. & Wolf, E., "Principles of Optics", 7th ed. (CUP
 *        1999) — rigorous treatment of cylindrical-wave superposition
 *        underpinning the ANALYTIC engine.
 *    [4] Griffiths, D. J., "Introduction to Electrodynamics", 4th ed.
 *        ch. 9 — derivation of the scalar wave equation ∂²u/∂t² = c²∇²u.
 *
 *   ── FDTD / numerical PDE ─────────────────────────────────────────
 *    [5] Yee, K. S. (1966), "Numerical solution of initial boundary
 *        value problems involving Maxwell's equations in isotropic
 *        media", IEEE Trans. Antennas Propag. AP-14, pp. 302-307 —
 *        THE original FDTD paper; our explicit 2nd-order stencil is
 *        the scalar special case.
 *    [6] Taflove, A. & Hagness, S. C., "Computational Electrodynamics:
 *        The Finite-Difference Time-Domain Method", 3rd ed. (Artech
 *        2005) — the FDTD bible.
 *    [7] Strikwerda, J. C., "Finite Difference Schemes and Partial
 *        Differential Equations", 2nd ed. (SIAM 2004) — CFL stability
 *        theorems that justify the c·√2 ≤ 1 condition.
 *    [8] LeVeque, R. J., "Finite Difference Methods for ODEs and
 *        PDEs" (SIAM 2007) — explicit Euler / FDTD theory, energy
 *        conservation arguments for the damping multiplier.
 *
 *   ── Rendering / ncurses ──────────────────────────────────────────
 *    [9] Raymond, E. S., "NCURSES Programming HOWTO" —
 *        tldp.org/HOWTO/NCURSES-Programming-HOWTO; covers the
 *        newscr/curscr diff model used in screen_draw and the
 *        init_pair / use_default_colors semantics for our themes.
 *   [10] Bourke, P. (1997), "Character representation of grayscale
 *        images", paulbourke.net/dataformats/asciiart — design of
 *        ASCII intensity ramps; the signed 8-level ramp
 *        ", . - ~ + * # @" was tuned by hand against Bourke's logic.
 *
 *   ── Online quick reference ───────────────────────────────────────
 *   [11] https://en.wikipedia.org/wiki/Finite-difference_time-domain_method
 *   [12] https://en.wikipedia.org/wiki/Huygens%E2%80%93Fresnel_principle
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   The same physics can be computed by stepping a grid (FDTD) or by summing
 *   sinusoids (ANALYTIC).  The grid solver KNOWS NOTHING about its sources
 *   except where they inject energy each tick; it produces interference as
 *   an emergent consequence of wave propagation.  The analytic solver
 *   computes the field exactly from the source list — no field state exists.
 *
 * ALGORITHM IN STEPS  (per render frame)
 *   FDTD mode:
 *     1. for sub_i in [0, STEPS):
 *          - for each active source: u_cur[src_xy] += A·sin(ω·t + φ_0)
 *          - for each inner cell: u_new = 2u − u_prev + C²·(Σ neighbours − 4u)
 *          - u_new *= damping; rotate (prev,cur,next)
 *          - t += 1/STEPS
 *     2. draw cells using amplitude_to_glyph(u_cur)
 *
 *   ANALYTIC mode:
 *     1. if any source moved: recompute kphase[s][r][c] = k_s·dist(s,r,c) −
 * φ_0s
 *     2. t += 1
 *     3. for each cell: u = Σ_s sin(ω_s · t − kphase[s][r][c])
 *     4. draw cell using amplitude_to_glyph(u / N_active)
 *
 * KEY FORMULAS
 *   FDTD inject       : u[src] += A·sin(ω t + φ_0)
 *   FDTD step         : u_new = (2u − u_prev + C²·∇²u) · damping
 *   FDTD CFL          : c·√2 ≤ 1
 *   Analytic field    : u(x,y,t) = Σ_s sin(ω_s t − k_s r_s + φ_0s)
 *   Wave number       : k = 2π / λ_pixels
 *   Aspect-corrected r: r² = (Δx·CELL_W)² + (Δy·CELL_H)²
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
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
/* What you get: every tunable constant in one place — magic numbers
 *               are banned in the code below.  Most defaults trade off
 *               smoothness vs CPU cost (FDTD substeps, render fps) or
 *               vs visual range (lambda, omega).                        */

/* Terminal cell pixel dimensions (project convention) — used for aspect
 * correction when computing source-to-cell distance in the analytic engine. */
#define CELL_W 8
#define CELL_H 16

/* Field caps (clamped at runtime to terminal extent) */
#define GRID_W_MAX 300
#define GRID_H_MAX 100

/* HUD layout (per CLAUDE.md): row 0 right-aligned status, row 1 left
 * parameter readouts, row rows-1 left-aligned hint. */
#define HUD_ROWS_TOP 2
#define HUD_ROWS_BOT 1

/* ── FDTD engine ────────────────────────────────────────────────────── */
#define C_SPEED 0.45f /* cells/tick; CFL: c·√2 ≤ 1 → ≤ 0.707  */
#define C_SQ (C_SPEED * C_SPEED)
#define DAMPING_DEF 0.993f
#define DAMPING_STEP 0.002f
#define DAMPING_MIN 0.960f
#define DAMPING_MAX 0.999f
#define FDTD_STEPS_DEF 4 /* sub-steps per render frame */
#define FDTD_STEPS_MIN 1
#define FDTD_STEPS_MAX 16
#define SOURCE_AMP 3.0f
#define IMPULSE_AMP 6.0f
#define IMPULSE_RADIUS 3

/* ── Analytic engine ────────────────────────────────────────────────── */
#define OMEGA_DEF 0.15f /* rad per render-frame; ~42 frames/cycle */
#define OMEGA_STEP 0.01f
#define LAMBDA_DEF 20.0f /* wavelength in columns */
#define LAMBDA_STEP 2.0f
#define LAMBDA_MIN 6.0f
#define LAMBDA_MAX 60.0f

/* ── Shared rendering ───────────────────────────────────────────────── */
#define MAX_AMP 4.0f    /* normalisation ceiling for FDTD */
#define ZERO_BAND 0.06f /* |u/MAX| < this → blank (nodal lines show) */
#define N_LEVELS 4      /* ramp tiers per sign */
#define N_FIELD_CP 8    /* 4 trough + 4 crest pairs */
#define N_THEMES 10
#define N_SRC_MAX 8

/* ── Colour pair indices ────────────────────────────────────────────── */
enum {
  CP_FIELD0 = 1, /* trough levels 0..3 (faintest → strongest) ... */
  CP_FIELD7 = CP_FIELD0 + N_FIELD_CP - 1,
  CP_HUD,     /* top status row 0 + param row 1                 */
  CP_HINT,    /* bottom hint row rows-1                         */
  CP_SRC_SEL, /* selected source marker                         */
  CP_SRC_OFF, /* unselected source marker                       */
};

/* ── Timing ─────────────────────────────────────────────────────────── */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define RENDER_FPS 30
#define RENDER_NS (NS_PER_SEC / RENDER_FPS)

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */
/* What you get: monotonic ns clock + sleep wrapper.  Used only by the
 *               main loop in §9 for frame timing.  Nothing here is
 *               algorithm-specific — pure boilerplate.                  */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec r = {(time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC)};
  nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */
/* What you get: 10 named themes (k_themes[]), the signed amplitude →
 *               (glyph + colour-pair + attr) mapping used by every cell
 *               painter, and the colour-pair indices.  Themes touch
 *               ONLY the field pairs — HUD pairs are fixed yellow/cyan
 *               so the readout always wins legibility.                  */

/*
 * Theme — palette bundle for ONE of the 10 named looks (MATRIX, FIRE, …).
 *
 * Intent
 *   Each theme defines exactly the 8 colour pairs in the signed FIELD
 *   ramp (CP_FIELD0 .. CP_FIELD7): four trough tiers (indices 0-3,
 *   "cool / receding") and four crest tiers (indices 4-7, "warm /
 *   emerging").  The HUD pairs (CP_HUD bright-yellow, CP_HINT bright-
 *   cyan) are NOT themed — they stay maximally legible against any
 *   field animation, per CLAUDE.md HUD spec.
 *
 * Why two palette tracks
 *   fg256 holds 256-colour cube/gray indices used when COLORS >= 256.
 *   fg8_neg / fg8_pos are the SINGLE colour applied to all four trough
 *   tiers / all four crest tiers respectively on 8-colour terminals.
 *   The two-bucket fallback is intentional — 8-colour systems can't
 *   show four distinct trough intensities, so we pick a sign-meaningful
 *   pair (e.g. blue troughs, cyan crests) and let the glyph ramp
 *   communicate intensity instead.
 *
 * Brightness rule (CLAUDE.md "Theme Palette Brightness")
 *   Every fg256[i] is kept ≥ 24 in the cube range so even the faintest
 *   tier stays visible.  Cube indices 16-23 / gray 232-239 read as
 *   black against the terminal's default background.
 *
 * References
 *   [9]  Raymond, "NCURSES Programming HOWTO" — init_pair() semantics.
 *   [10] Bourke, "Character representation of grayscale images" — the
 *        glyph ramp ", . - ~ + * # @" pairs with these colour tiers.
 */
typedef struct {
  const char *name;        /* short ASCII label shown in HUD     */
  short fg256[N_FIELD_CP]; /* 8 indices: [0..3]=trough,[4..7]=crest */
  short fg8_neg;           /* fallback colour for all trough tiers  */
  short fg8_pos;           /* fallback colour for all crest tiers   */
} Theme;

static const Theme k_themes[N_THEMES] = {
    {"MATRIX", {28, 34, 40, 46, 82, 118, 154, 231}, COLOR_GREEN, COLOR_GREEN},
    {"FIRE", {52, 88, 124, 160, 208, 214, 220, 231}, COLOR_BLUE, COLOR_YELLOW},
    {"OCEANIC", {24, 25, 31, 39, 45, 51, 159, 231}, COLOR_BLUE, COLOR_CYAN},
    {"NEON",
     {53, 56, 93, 129, 165, 200, 207, 231},
     COLOR_MAGENTA,
     COLOR_MAGENTA},
    {"MONO",
     {240, 241, 242, 243, 250, 252, 254, 231},
     COLOR_WHITE,
     COLOR_WHITE},
    {"ICE", {25, 31, 32, 39, 44, 51, 195, 231}, COLOR_BLUE, COLOR_CYAN},
    {"NOVA",
     {54, 56, 92, 128, 164, 200, 213, 231},
     COLOR_MAGENTA,
     COLOR_MAGENTA},
    {"FOREST", {28, 34, 40, 46, 100, 142, 184, 231}, COLOR_GREEN, COLOR_YELLOW},
    {"DESERT", {52, 94, 130, 166, 208, 214, 222, 231}, COLOR_RED, COLOR_YELLOW},
    {"ECLIPSE",
     {53, 89, 125, 161, 197, 213, 219, 231},
     COLOR_MAGENTA,
     COLOR_WHITE},
};

/*
 * 8-level signed glyph ramp.
 *   troughs:  ',' '.' '-' '~'   (hollow / receding)
 *   crests :  '+' '*' '#' '@'   (solid  / emerging)
 */
static const char k_field_chars[N_FIELD_CP] = {',', '.', '-', '~',
                                               '+', '*', '#', '@'};

static void theme_apply(int ti) {
  const Theme *t = &k_themes[ti];
  bool full = (COLORS >= 256);
  for (int i = 0; i < N_FIELD_CP; i++) {
    short fg = full ? t->fg256[i] : (i < N_LEVELS ? t->fg8_neg : t->fg8_pos);
    init_pair((short)(CP_FIELD0 + i), fg, -1);
  }
  init_pair(CP_HUD, full ? 226 : COLOR_YELLOW, -1); /* bright yellow */
  init_pair(CP_HINT, full ? 51 : COLOR_CYAN, -1);   /* bright cyan   */
  init_pair(CP_SRC_SEL, full ? 226 : COLOR_YELLOW, -1);
  init_pair(CP_SRC_OFF, full ? 244 : COLOR_WHITE, -1);
}

static void color_init(int theme) {
  start_color();
  use_default_colors();
  theme_apply(theme);
}

/*
 * amplitude_level — map normalised u ∈ [−1, 1] to ramp index.
 *   |u| ≤ ZERO_BAND  → −1 (draw nothing; background shows nodal lines)
 *   u < 0            → trough tier 0..3
 *   u > 0            → crest  tier 4..7
 */
static int amplitude_level(float u_norm) {
  if (u_norm > 1.f)
    u_norm = 1.f;
  if (u_norm < -1.f)
    u_norm = -1.f;
  float au = u_norm < 0.f ? -u_norm : u_norm;
  if (au <= ZERO_BAND)
    return -1;
  float t = (au - ZERO_BAND) / (1.f - ZERO_BAND);
  int lv = (int)(t * (float)N_LEVELS);
  if (lv >= N_LEVELS)
    lv = N_LEVELS - 1;
  return (u_norm < 0.f) ? lv : (N_LEVELS + lv);
}

static attr_t level_attr(int lv) {
  attr_t a = COLOR_PAIR(CP_FIELD0 + lv);
  if (lv == N_FIELD_CP - 1)
    a |= A_BOLD; /* brightest crest glows */
  return a;
}

/* ===================================================================== */
/* §4  fdtd-grid                                                          */
/* ===================================================================== */
/* What you get: the FDTD engine's field state (Grid struct) and ONE
 *               update step (grid_tick).  Self-contained — this section
 *               knows nothing about sources, themes, or rendering.
 *               grid_tick is a pseudocode orchestrator over 4 named
 *               helpers (Laplacian, central-diff, sweep, rotate).       */

/*
 * Grid — FDTD field state, three time levels stored explicitly.
 *
 * Intent
 *   The wave equation ∂²u/∂t² = c²∇²u is 2nd-order in time, so the
 *   explicit central-difference scheme needs THREE adjacent slices to
 *   compute the next one:
 *      u_new = 2·u − u_prev + C²·∇²u
 *   We allocate three buffers and ROTATE pointers each step rather than
 *   copying data — the slots cycle prev ← cur ← next, scratch becomes
 *   the new `next`.  See [5] Yee 1966; [6] Taflove §3.6.
 *
 * Why pointer rotation instead of memcpy
 *   Rotating three pointers is O(1) per step vs O(W·H) for memcpy.
 *   On a 300×100 grid that's 30 000 cells avoided × FDTD_STEPS substeps
 *   × 30 fps ≈ 3.6 M cells/s saved.
 *
 * Why damping lives here (not in Scene)
 *   damping is a per-Grid setting (the user adjusts it with d/D in
 *   FDTD mode and it has no meaning in analytic mode).  Multiplied
 *   into u_new each step, amplitude decays as damping^N over N steps —
 *   converts boundary reflections from a permanent "box mode" into a
 *   slow energy bleed.  See [8] LeVeque §1.6 on stability via
 *   numerical dissipation.
 *
 * Memory note
 *   The three buffers are calloc'd once at scene_init (and reallocated
 *   only on SIGWINCH).  The hot path makes no allocations.
 */
typedef struct {
  /* ── Three time levels of the central-difference scheme ───────── *
   * Indexed row-major y*cols + x.  prev = u(t-Δt), cur = u(t),      *
   * next = u(t+Δt) being written this step.  Pointers ROTATE each  *
   * step (no data copy).                                            */
  float *prev;
  float *cur;
  float *next;

  /* ── Grid extent in cells ─────────────────────────────────────── */
  int cols;
  int rows;

  /* ── Per-step amplitude multiplier ∈ (DAMPING_MIN, DAMPING_MAX)  *
   * Applied to u_new every FDTD step; damping^N ≈ 0.5 sets the     *
   * half-life of an injected pulse.                                 */
  float damping;
} Grid;

static void grid_alloc(Grid *g, int cols, int rows) {
  size_t n = (size_t)cols * (size_t)rows;
  g->cols = cols;
  g->rows = rows;
  g->prev = calloc(n, sizeof(float));
  g->cur = calloc(n, sizeof(float));
  g->next = calloc(n, sizeof(float));
}

static void grid_free(Grid *g) {
  free(g->prev);
  free(g->cur);
  free(g->next);
  memset(g, 0, sizeof *g);
}

static void grid_clear(Grid *g) {
  size_t n = (size_t)g->cols * (size_t)g->rows * sizeof(float);
  memset(g->prev, 0, n);
  memset(g->cur, 0, n);
  memset(g->next, 0, n);
}

static void grid_resize(Grid *g, int cols, int rows) {
  float d = g->damping;
  grid_free(g);
  grid_alloc(g, cols, rows);
  g->damping = d;
}

/* 4-point cross Laplacian at interior cell i:
 *   ∇²u ≈ u_E + u_W + u_N + u_S − 4·u_C
 * Grid spacing h = 1 cell, absorbed into the stability number C².
 * Refs [5] Yee 1966; [6] Taflove §3.6. */
static inline float laplacian_4point_cross(const float *u, int i, int north_i,
                                           int south_i) {
  return u[i + 1] + u[i - 1] + u[north_i] + u[south_i] - 4.f * u[i];
}

/* Central-difference time step for the scalar wave equation:
 *   u_new = 2·u_cur − u_prev + C²·∇²u
 * 2nd-order accurate in both time and space.  Refs [5] Yee 1966;
 * [8] LeVeque §1.4. */
static inline float wave_central_difference_step(float u_cur, float u_prev,
                                                 float laplacian, float csq) {
  return 2.f * u_cur - u_prev + csq * laplacian;
}

/* Sweep all INTERIOR cells of the grid with the central-difference scheme
 * and multiply by the per-step damping factor.  Boundary rows/cols are
 * left at 0 → hard reflecting walls; damping converts what would be a
 * permanent box-mode standing wave into a slow energy bleed.            */
static void sweep_interior_with_central_diff(Grid *g) {
  const int cols = g->cols, rows = g->rows;
  const float csq = C_SQ;
  const float damp = g->damping;
  const float *cu = g->cur, *pv = g->prev;
  float *nx = g->next;

  for (int y = 1; y < rows - 1; y++) {
    int row = y * cols;
    int rowp = row + cols; /* row index of y+1 (south) */
    int rowm = row - cols; /* row index of y-1 (north) */
    for (int x = 1; x < cols - 1; x++) {
      int i = row + x;
      float lap = laplacian_4point_cross(cu, i, rowm + x, rowp + x);
      nx[i] = wave_central_difference_step(cu[i], pv[i], lap, csq) * damp;
    }
  }
}

/* Rotate the three time-level pointers: prev ← cur ← next, scratch
 * becomes the new `next`.  Three-pointer cycle is O(1) vs O(W·H) for
 * an equivalent memcpy.  See Grid doc.                                  */
static inline void rotate_time_level_buffers(Grid *g) {
  float *tmp = g->prev;
  g->prev = g->cur;
  g->cur = g->next;
  g->next = tmp;
}

/*
 * grid_tick — ONE FDTD step of the scalar wave equation.
 *
 * Pseudocode:
 *   for each interior cell c:
 *     ∇²u ← 4-point cross Laplacian
 *     u_new[c] ← (2·u[c] − u_prev[c] + C²·∇²u) · damping
 *   rotate time-level buffers (prev ← cur ← next)
 */
static void grid_tick(Grid *g) {
  sweep_interior_with_central_diff(g);
  rotate_time_level_buffers(g);
}

/*
 * grid_impulse — Gaussian blob centred at (cx, cy).
 * Initial conditions: set both cur and prev to the same blob so the wave
 * starts at rest with non-zero amplitude.  (If only cur is set with prev=0,
 * the first step gives velocity 2·cur — not what we want.)
 */
static void grid_impulse(Grid *g, int cx, int cy) {
  int R = IMPULSE_RADIUS;
  for (int dy = -R; dy <= R; dy++) {
    for (int dx = -R; dx <= R; dx++) {
      int x = cx + dx, y = cy + dy;
      if (x < 1 || x >= g->cols - 1 || y < 1 || y >= g->rows - 1)
        continue;
      float d2 = (float)(dx * dx + dy * dy);
      float w = expf(-d2 / (float)(R * R));
      int i = y * g->cols + x;
      g->cur[i] += IMPULSE_AMP * w;
      g->prev[i] += IMPULSE_AMP * w;
    }
  }
}

/* ===================================================================== */
/* §5  sources                                                            */
/* ===================================================================== */
/* What you get: the Source struct (shared by BOTH engines), the global
 *               g_src[] array + g_nactive + g_selected, the source-
 *               editing helpers (add/delete/move/select), and the 5
 *               preset seeders (double-slit, ripple, beat, radial,
 *               five-oscillator) dispatched by preset_apply.            */

/*
 * Source — one coherent cylindrical point emitter, shared by BOTH engines.
 *
 * Intent
 *   The FDTD inject step and the analytic field sum read THE SAME
 *   N_SRC_MAX-element g_src[] array.  Keeping one schema means presets
 *   look the same in both engines — the analytic version is the exact
 *   limit of the FDTD version as Δt → 0.
 *
 * Coordinate convention
 *   x, y are CELL coordinates inside the field rect (NOT pixel, NOT
 *   terminal-relative).  The field rect starts at row HUD_ROWS_TOP and
 *   is gw×gh cells.  Each engine maps these to its own draw position:
 *     FDTD     → integer (int)x, (int)y as the cell to inject into
 *     ANALYTIC → real distance r_s(c,r) = √((Δx·CELL_W)² + (Δy·CELL_H)²)
 *                aspect-corrected so cylindrical wavefronts look round
 *                instead of vertically stretched [3] Born & Wolf §8.6.
 *
 * Why omega is in render-frame units
 *   ω is rad PER RENDER FRAME (not per FDTD substep).  In FDTD mode
 *   scene_fdtd_inject scales by 1/fdtd_steps internally so the source
 *   completes ω of phase per render frame regardless of fdtd_steps.
 *   This keeps the VISIBLE frequency identical when the user toggles
 *   engines or adjusts steps — the engine-toggle invariant.
 *
 * Why two phase fields
 *   phase_init is the FIXED offset set by the preset (the radial-star
 *   preset gives source i an offset 2π·i/N to produce the rotation
 *   symmetry).  Read by BOTH engines:
 *     FDTD     uses phase_run as a running accumulator, initialised to
 *              phase_init at preset load
 *     ANALYTIC reads phase_init as the −φ_0 term in sin(ωt − kr + φ_0)
 *   phase_run is FDTD-only working state — kept on Source so each
 *   source carries its own phase clock through preset switches.
 *
 * Why lambda is analytic-only
 *   FDTD's spatial wavelength is implicitly determined by ω and the
 *   wave speed C_SPEED.  lambda is consumed only by the analytic engine
 *   via k = 2π / (λ · CELL_W).  Adjusting lambda while in FDTD mode is
 *   harmless but invisible — hence +/- has engine-dependent semantics
 *   in app_handle_key.
 *
 * Reference [3] Born & Wolf §8.6 — cylindrical-wave superposition.
 */
typedef struct {
  /* ── Geometry inside the field rect ──────────────────────────── */
  float x; /* cell column, 0 .. gw                       */
  float y; /* cell row,    0 .. gh                       */

  /* ── Frequency and wavelength (engine-dependent meaning) ─────── */
  float omega;  /* rad / render frame     (both engines)      */
  float lambda; /* wavelength in columns  (analytic only)     */

  /* ── Phase bookkeeping ─────────────────────────────────────── */
  float phase_init; /* fixed offset set by preset                 */
  float phase_run;  /* FDTD running phase (advances each substep) */

  /* ── Lifecycle ──────────────────────────────────────────────── */
  bool active; /* toggled by source_add / source_delete      */
} Source;

static Source g_src[N_SRC_MAX];
static int g_nactive = 0;
static int g_selected = 0;

static void source_clear_all(void) {
  memset(g_src, 0, sizeof g_src);
  g_nactive = 0;
  g_selected = 0;
}

static int source_add(float x, float y, float omega, float lambda,
                      float phase0) {
  for (int s = 0; s < N_SRC_MAX; s++) {
    if (!g_src[s].active) {
      g_src[s] = (Source){x, y, omega, lambda, phase0, phase0, true};
      g_nactive++;
      g_selected = s;
      return s;
    }
  }
  return -1;
}

static void source_delete(int s) {
  if (s < 0 || s >= N_SRC_MAX || !g_src[s].active)
    return;
  g_src[s].active = false;
  g_nactive--;
  for (int i = 0; i < N_SRC_MAX; i++)
    if (g_src[i].active) {
      g_selected = i;
      return;
    }
  g_selected = 0;
}

static void source_select_step(int dir) {
  if (g_nactive == 0)
    return;
  int s = g_selected;
  for (int i = 0; i < N_SRC_MAX; i++) {
    s = (s + dir + N_SRC_MAX) % N_SRC_MAX;
    if (g_src[s].active) {
      g_selected = s;
      return;
    }
  }
}

static void source_move(int dc, int dr, int gw, int gh) {
  if (!g_src[g_selected].active)
    return;
  Source *s = &g_src[g_selected];
  s->x += dc;
  s->y += dr;
  if (s->x < 0)
    s->x = 0;
  if (s->x >= (float)gw)
    s->x = (float)(gw - 1);
  if (s->y < 0)
    s->y = 0;
  if (s->y >= (float)gh)
    s->y = (float)(gh - 1);
}

/* Preset 1 — DOUBLE SLIT.  Two coherent, in-phase sources on a vertical
 * line in the left quarter; the right half displays the classic hyperbolic
 * nodal/antinodal fringe pattern of Young's double-slit experiment.
 * Ref [2] Hecht ch. 9. */
static inline void seed_double_slit(int gw, int gh, float omega, float lambda) {
  float cy = (float)gh * 0.5f;
  float x = (float)gw * 0.28f;
  float sep = (float)gh * 0.22f;
  source_add(x, cy - sep * 0.5f, omega, lambda, 0.f);
  source_add(x, cy + sep * 0.5f, omega, lambda, 0.f);
}

/* Preset 2 — RIPPLE TANK.  Four corner emitters, same frequency and phase.
 * The linear superposition Σ sin(ωt − kr_i) produces a 4-fold cross-hatch
 * fringe pattern — the geometry of mechanical ripple tanks used in optics
 * teaching labs.  Ref [1] Crawford ch. 4. */
static inline void seed_ripple_tank(int gw, int gh, float omega, float lambda) {
  float cx = (float)gw * 0.5f, cy = (float)gh * 0.5f;
  float rx = (float)gw * 0.32f, ry = (float)gh * 0.30f;
  source_add(cx - rx, cy - ry, omega, lambda, 0.f);
  source_add(cx + rx, cy - ry, omega, lambda, 0.f);
  source_add(cx - rx, cy + ry, omega, lambda, 0.f);
  source_add(cx + rx, cy + ry, omega, lambda, 0.f);
}

/* Preset 3 — BEAT.  Two sources with Δω/ω ≈ 15%.  Trig identity:
 *   sin(ω₁ t) + sin(ω₂ t) = 2 cos((Δω/2)·t) · sin(ω̄·t)
 * gives a slowly-drifting amplitude envelope at frequency Δω/2 — the
 * classic beat phenomenon.  Ref [1] Crawford ch. 1. */
static inline void seed_beat(int gw, int gh, float omega, float lambda) {
  float cy = (float)gh * 0.5f;
  float x0 = (float)gw * 0.30f, x1 = (float)gw * 0.70f;
  source_add(x0, cy, omega, lambda, 0.f);
  source_add(x1, cy, omega * 1.15f, lambda, 0.f);
}

/* Preset 4 — RADIAL STAR.  Six sources equally spaced around an aspect-
 * corrected ellipse (Ry = R/2 to undo the terminal's 2:1 cell aspect
 * ratio) with progressive phase offsets 2π·i/6.  Yields a 6-fold radial
 * standing-wave pattern from rotational interference symmetry.  Refs
 * [3] Born & Wolf §8.6. */
static inline void seed_radial_star(int gw, int gh, float omega, float lambda) {
  float cx = (float)gw * 0.5f, cy = (float)gh * 0.5f;
  float R = fminf(cx, cy) * 0.55f;
  float Ry = R * 0.5f; /* aspect correction */
  const int N = 6;
  for (int i = 0; i < N; i++) {
    float a = (float)i * 2.f * (float)M_PI / (float)N;
    source_add(cx + R * cosf(a), cy + Ry * sinf(a), omega, lambda,
               (float)i * 2.f * (float)M_PI / (float)N);
  }
}

/* Preset 5 — FIVE OSCILLATOR.  Four corners + centre with five slightly
 * detuned frequencies (0.190..0.260 rad/frame) straddling a common
 * value.  Adjacent pairs beat slowly against each other, producing a
 * complex never-quite-repeating fringe pattern across the whole grid. */
static inline void seed_five_oscillator_grid(int gw, int gh, float lambda) {
  const float fx[5] = {0.22f, 0.78f, 0.50f, 0.22f, 0.78f};
  const float fy[5] = {0.25f, 0.25f, 0.50f, 0.75f, 0.75f};
  const float freq[5] = {0.220f, 0.260f, 0.190f, 0.240f, 0.210f};
  for (int i = 0; i < 5; i++)
    source_add(fx[i] * (float)gw, fy[i] * (float)gh, freq[i], lambda, 0.f);
}

/*
 * preset_apply — clear g_src[] and reseed it with one of 5 named
 * interference configurations.  Each seeder is named after the physics
 * it demonstrates; preset_apply itself is just the dispatcher.
 *
 * Pseudocode:
 *   clear all sources
 *   dispatch on p:
 *     1 → seed_double_slit
 *     2 → seed_ripple_tank
 *     3 → seed_beat
 *     4 → seed_radial_star
 *     else → seed_five_oscillator_grid
 */
static void preset_apply(int p, int gw, int gh) {
  source_clear_all();
  const float o = OMEGA_DEF;
  const float lam = LAMBDA_DEF;

  switch (p) {
  case 1:
    seed_double_slit(gw, gh, o, lam);
    break;
  case 2:
    seed_ripple_tank(gw, gh, o, lam);
    break;
  case 3:
    seed_beat(gw, gh, o, lam);
    break;
  case 4:
    seed_radial_star(gw, gh, o, lam);
    break;
  default:
    seed_five_oscillator_grid(gw, gh, lam);
    break;
  }
}

/* ===================================================================== */
/* §6  analytic field                                                     */
/* ===================================================================== */
/* What you get: the analytic engine — closed-form Σ sin(ωt − kr + φ_0).
 *               AnalyticTable caches the time-independent (k·r − φ_0)
 *               term per source per cell so the per-frame field-eval
 *               drops to ONE sinf() per active source per cell.         */

/*
 * AnalyticTable — precomputed phase cache for the analytic engine.
 *
 * Intent
 *   The analytic field at cell (r,c) and time t is:
 *
 *      u(r,c,t)  =  Σ_s  sin( ω_s · t  −  k_s · r_s(c,r)  +  φ_0s )
 *
 *   The term [k_s · r_s − φ_0s] depends only on the source's position,
 *   wavelength, and phase offset — NOT on time.  Cache it once per
 *   source-move and per-frame cost drops to ONE sinf() per active
 *   source per cell:
 *
 *      u(r,c,t)  =  Σ_s  sin( ω_s · t  −  kphase[s][r][c] )
 *
 *   This is a textbook lookup-table-vs-recomputation trade in
 *   superposition computation.  See [3] Born & Wolf §8.6.
 *
 * Why BSS storage
 *     N_SRC_MAX × GRID_H_MAX × GRID_W_MAX × sizeof(float)
 *       = 8 × 100 × 300 × 4  =  960 KB
 *   Free in BSS — zero-cost allocation, lazily paged in.  Mallocing
 *   this on every source-move would cost a real syscall.  CLAUDE.md
 *   "No dynamic allocation after init" — this is exactly the use case.
 *
 * Why a single dirty flag and not a per-source generation counter
 *   Invalidation is atomic: ANY source position / lambda / active /
 *   phase change rebuilds the WHOLE table.  We never rebuild a subset,
 *   so a single bool is sufficient.  Source editing (Tab + arrows + n
 *   + x + +/- + f/F) is human-scale interactive — even at 30 events/s
 *   the full O(N_SRC · gw · gh) recompute is a few ms.
 *
 * Indexing
 *   [source][row][col] order — innermost iteration over col is unit
 *   stride and matches the scene_draw_field row-major scan order, so
 *   the hot read in analytic_field() is cache-friendly.
 */
typedef struct {
  /* Per-source-per-cell phase: kphase[s][r][c] = k_s·r_s(c,r) − φ_0s */
  float kphase[N_SRC_MAX][GRID_H_MAX][GRID_W_MAX];

  /* If true, analytic_recompute() rebuilds the whole table at the
   * start of the next scene_tick. */
  bool dirty;
} AnalyticTable;

static AnalyticTable g_at;

static void analytic_recompute(int gw, int gh) {
  g_at.dirty = false;
  for (int s = 0; s < N_SRC_MAX; s++) {
    if (!g_src[s].active)
      continue;
    float k = 2.f * (float)M_PI / (g_src[s].lambda * (float)CELL_W);
    float sx = g_src[s].x, sy = g_src[s].y;
    float ph = g_src[s].phase_init;
    for (int r = 0; r < gh; r++) {
      float dy = ((float)r - sy) * (float)CELL_H;
      for (int c = 0; c < gw; c++) {
        float dx = ((float)c - sx) * (float)CELL_W;
        g_at.kphase[s][r][c] = k * sqrtf(dx * dx + dy * dy) - ph;
      }
    }
  }
}

static float analytic_field(int r, int c, float t) {
  float u = 0.f;
  for (int s = 0; s < N_SRC_MAX; s++) {
    if (!g_src[s].active)
      continue;
    u += sinf(g_src[s].omega * t - g_at.kphase[s][r][c]);
  }
  return u;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */
/* What you get: Scene composes everything above + the Engine toggle.
 *               scene_tick drives the active engine (FDTD substeps OR
 *               analytic cache refresh).  scene_draw_field paints
 *               u(r,c) → glyphs through the shared paint_field_cell.
 *               This is where Source data, the FDTD Grid, and the
 *               AnalyticTable cache all meet.                           */

/*
 * Engine — which solver computes u(x,y,t) for the current frame.
 *
 *   ENGINE_FDTD     Explicit central-difference time stepping of the
 *                    scalar wave PDE.  Field state lives in Grid; the
 *                    bob of energy injected at a source propagates
 *                    outward at speed C_SPEED.  Subject to CFL [7].
 *                    Refs [5][6].
 *   ENGINE_ANALYTIC Closed-form Σ sin(ωt − kr + φ_0) per cell using
 *                    AnalyticTable as a phase cache.  No field state,
 *                    no time-stepping, unconditionally stable.  Refs
 *                    [2][3].
 *
 * Toggled by 'm'; the toggle clears Grid and resets each source's
 * phase_run so the two engines start from the same observable state.
 */
typedef enum { ENGINE_FDTD = 0, ENGINE_ANALYTIC = 1 } Engine;

static const char *engine_name(Engine e) {
  return e == ENGINE_FDTD ? "FDTD" : "ANALYTIC";
}

/*
 * Scene — the single owner of everything visible on screen.
 *
 * Intent
 *   Scene is the integration point between the SIMULATION (Grid plus
 *   the file-scope g_src[] and g_at, none of which know about a
 *   terminal) and the RENDERING (an ncurses surface that doesn't know
 *   about wave PDEs).  The two layers never talk directly — they meet
 *   here.  Anything that doesn't fit in Grid, Source, AnalyticTable,
 *   or Screen lives in Scene.
 *
 * Locality (sim vs render)
 *   The fields below are EXPLICITLY GROUPED so a reader can tell at a
 *   glance which subsystem reads each one:
 *     - scene_tick reads it                 → simulation
 *     - scene_draw_field / scene_draw_sources / hud_draw reads it
 *                                          → rendering
 *     - both sides read it (gw, gh)        → geometry
 *
 *   Mis-classifying a field is a real source of bugs — a render-only
 *   value accidentally read by the tick would couple physics to the
 *   terminal extent, defeating the engine-toggle invariant ("same
 *   source list ⇒ same physics regardless of how it is drawn").
 *
 * Why these specific fields and no others
 *   - engine        Chooses backend in scene_tick AND branch in
 *                   scene_draw_field.  Toggling clears Grid + resets
 *                   source phase_run so visuals stay continuous.
 *   - grid          Allocated EVEN in analytic mode so engine toggle
 *                   is alloc-free.  See Grid doc.
 *   - t             Render-frame clock, read by BOTH engines:
 *                     FDTD     → scene_fdtd_inject phase argument
 *                     analytic → ω·t term in sin(ω·t − kphase)
 *   - gw, gh        Field rect in cells.  Simulation reads gw,gh in
 *                   analytic_recompute (sets the kphase extent).
 *                   Rendering reads them as clip bounds.
 *   - preset        Selects which preset_apply() row reset reloads;
 *                   also the label shown in HUD.
 *   - theme         Pure render — repaints colour pairs via
 *                   theme_apply() on cycle.  Survives engine toggle.
 *   - fdtd_steps    Sim tuning — substeps per render frame.
 *                   Meaningless in analytic mode (no time-stepping).
 *   - paused        Gate for scene_tick (no-op when set).
 *   - needs_redraw  Render-only flag that forces erase() on the next
 *                   frame after theme or engine change so stale field
 *                   glyphs don't linger.
 *
 * Things that DO NOT live here
 *   - Source list                → g_src[] (file-scope; both engines
 *                                   plus source_* helpers share it)
 *   - Analytic phase cache       → g_at (file-scope, 960 KB BSS)
 *   - Frame-timing accumulators  → locals in main()
 *   - Signal flags               → App
 *   - ncurses buffers            → owned by the library
 *
 * Reference [6] Taflove §3.6 on state ownership in FDTD implementations.
 */
typedef struct {
  /* ── Geometry  (shared by simulation AND rendering) ─────────── *
   * Field rect in cells; gw × gh.  Updated only at scene_init and *
   * scene_resize.  Both analytic_recompute (sim) and the draw     *
   * functions (render) read these.                                */
  int gw;
  int gh;

  /* ── Simulation  (read by scene_tick) ──────────────────────── *
   * The actual field state plus control flags that gate it.       *
   * Engine selects the backend; preset is "control" but grouped   *
   * here because it survives a theme change.                      */
  Engine engine;
  Grid grid;
  float t;        /* render-frame clock                   */
  int preset;     /* 1 .. 5; reset reloads this row       */
  int fdtd_steps; /* FDTD substeps per render frame       */
  bool paused;    /* if true scene_tick is a no-op        */

  /* ── Rendering  (read by scene_draw_field / hud_draw only) ──── *
   * Pure user-visible choices.  Changing theme MUST NOT touch the *
   * simulation — it only repaints colour pairs.  needs_redraw is  *
   * a one-shot flag cleared by the next erase().                  */
  int theme;         /* 0 .. N_THEMES-1; indexes k_themes[]  */
  bool needs_redraw; /* forces erase() after theme/engine    */
} Scene;

static void scene_init(Scene *s, int term_cols, int term_rows) {
  memset(s, 0, sizeof *s);
  s->engine = ENGINE_FDTD;
  s->preset = 1;
  s->theme = 2; /* OCEANIC by default — fits the wave aesthetic  */
  s->fdtd_steps = FDTD_STEPS_DEF;
  s->paused = false;
  s->needs_redraw = true;

  int gw = term_cols;
  int gh = term_rows - HUD_ROWS_TOP - HUD_ROWS_BOT;
  if (gw > GRID_W_MAX)
    gw = GRID_W_MAX;
  if (gh > GRID_H_MAX)
    gh = GRID_H_MAX;
  if (gw < 1)
    gw = 1;
  if (gh < 1)
    gh = 1;
  s->gw = gw;
  s->gh = gh;

  grid_alloc(&s->grid, gw, gh);
  s->grid.damping = DAMPING_DEF;
  theme_apply(s->theme);

  preset_apply(s->preset, gw, gh);
  g_at.dirty = true;
}

static void scene_free(Scene *s) { grid_free(&s->grid); }

static void scene_resize(Scene *s, int term_cols, int term_rows) {
  int gw = term_cols;
  int gh = term_rows - HUD_ROWS_TOP - HUD_ROWS_BOT;
  if (gw > GRID_W_MAX)
    gw = GRID_W_MAX;
  if (gh > GRID_H_MAX)
    gh = GRID_H_MAX;
  if (gw < 1)
    gw = 1;
  if (gh < 1)
    gh = 1;
  s->gw = gw;
  s->gh = gh;
  grid_resize(&s->grid, gw, gh);
  preset_apply(s->preset, gw, gh);
  g_at.dirty = true;
  s->needs_redraw = true;
}

static void scene_toggle_engine(Scene *s) {
  s->engine = (s->engine == ENGINE_FDTD) ? ENGINE_ANALYTIC : ENGINE_FDTD;
  grid_clear(&s->grid);
  for (int i = 0; i < N_SRC_MAX; i++)
    g_src[i].phase_run = g_src[i].phase_init;
  s->t = 0.f;
  g_at.dirty = true;
  s->needs_redraw = true;
}

/*
 * FDTD inject: each active source adds A·sin(phase_run) to its cell, then
 * advances phase_run by ω/STEPS so the source completes ω rad of cycle per
 * render frame (matching the analytic engine's time base).
 */
static void scene_fdtd_inject(Scene *s) {
  float dphase = 1.f / (float)s->fdtd_steps;
  for (int i = 0; i < N_SRC_MAX; i++) {
    Source *src = &g_src[i];
    if (!src->active)
      continue;
    int sx = (int)src->x;
    int sy = (int)src->y;
    if (sx < 1 || sx >= s->grid.cols - 1)
      continue;
    if (sy < 1 || sy >= s->grid.rows - 1)
      continue;
    s->grid.cur[sy * s->grid.cols + sx] += SOURCE_AMP * sinf(src->phase_run);
    src->phase_run += src->omega * dphase;
  }
}

/* Run one render-frame's worth of FDTD substeps.  Each substep injects
 * the active sources, then advances the wave one tick via grid_tick.
 * fdtd_steps decouples visual wavefront speed from the CFL stability
 * constraint — more substeps per render frame → faster-looking waves
 * without changing the per-step C².  See [6] Taflove §3.6.             */
static void tick_fdtd_engine(Scene *s) {
  for (int i = 0; i < s->fdtd_steps; i++) {
    scene_fdtd_inject(s);
    grid_tick(&s->grid);
  }
}

/* The analytic engine has no field state to evolve — u(r,c,t) is
 * computed directly from sources every frame in scene_draw_field.  All
 * this tick has to do is REBUILD THE PHASE CACHE if a source moved
 * (source-editing keys set g_at.dirty).  See AnalyticTable doc.        */
static void tick_analytic_engine(Scene *s) {
  if (g_at.dirty)
    analytic_recompute(s->gw, s->gh);
}

/* Advance the render-frame clock that BOTH engines read:
 *   FDTD     → scene_fdtd_inject uses t for source phase
 *   ANALYTIC → analytic_field uses t as the ω·t term in sin(ω·t − kphase) */
static inline void advance_render_frame_clock(Scene *s) { s->t += 1.f; }

/*
 * scene_tick — one render-frame of simulation work.
 *
 * Pseudocode:
 *   if paused → no-op
 *   dispatch on engine:
 *     FDTD     → run fdtd_steps substeps of (inject + grid_tick)
 *     ANALYTIC → rebuild phase cache if dirty
 *   advance the render-frame clock
 */
static void scene_tick(Scene *s) {
  if (s->paused)
    return;

  if (s->engine == ENGINE_FDTD)
    tick_fdtd_engine(s);
  else
    tick_analytic_engine(s);

  advance_render_frame_clock(s);
}

/* Paint ONE field cell.  Maps the normalised amplitude u ∈ [-1, 1] to a
 * signed glyph + colour pair via amplitude_level → k_field_chars[] +
 * level_attr().  Level −1 (dead zone around u=0) leaves the cell blank
 * so nodal lines show through as background.  Display row is offset by
 * HUD_ROWS_TOP so the HUD rows stay clear.  Ref [10] Bourke for the
 * intensity-ramp design. */
static inline void paint_field_cell(int field_r, int field_c, float u_norm) {
  int lv = amplitude_level(u_norm);
  if (lv < 0)
    return;
  attr_t at = level_attr(lv);
  attron(at);
  mvaddch(field_r + HUD_ROWS_TOP, field_c,
          (chtype)(unsigned char)k_field_chars[lv]);
  attroff(at);
}

/* FDTD field reader: row-major scan of grid.cur, normalised by MAX_AMP
 * because the FDTD field grows freely (bounded only by damping and CFL).
 * Inner loop is unit-stride and cache-friendly. */
static void paint_field_from_fdtd(Scene *s) {
  const float *cu = s->grid.cur;
  const int cols = s->grid.cols;
  for (int r = 0; r < s->gh; r++) {
    int row = r * cols;
    for (int c = 0; c < s->gw; c++)
      paint_field_cell(r, c, cu[row + c] / MAX_AMP);
  }
}

/* Analytic field reader: sums the N_active sinusoidal source
 * contributions per cell via analytic_field(), normalised by 1/N_active
 * so peak |u| stays ≈ 1 regardless of source count.  The spatial term
 * is served from the precomputed AnalyticTable cache. */
static void paint_field_from_analytic(Scene *s) {
  float norm = (g_nactive > 0) ? 1.f / (float)g_nactive : 1.f;
  for (int r = 0; r < s->gh; r++) {
    for (int c = 0; c < s->gw; c++)
      paint_field_cell(r, c, analytic_field(r, c, s->t) * norm);
  }
}

/*
 * scene_draw_field — render the gw × gh field rect at display origin
 * (HUD_ROWS_TOP, 0).  Engine selects which reader supplies u(r,c); the
 * glyph mapping is shared across engines.
 *
 * Pseudocode:
 *   if engine == FDTD     → paint_field_from_fdtd     (read grid.cur)
 *   else                  → paint_field_from_analytic (sum sinusoids)
 */
static void scene_draw_field(Scene *s) {
  if (s->engine == ENGINE_FDTD)
    paint_field_from_fdtd(s);
  else
    paint_field_from_analytic(s);
}

static void scene_draw_sources(Scene *s) {
  for (int i = 0; i < N_SRC_MAX; i++) {
    if (!g_src[i].active)
      continue;
    int dr = (int)g_src[i].y + HUD_ROWS_TOP;
    int dc = (int)g_src[i].x;
    if (dr < HUD_ROWS_TOP || dr >= HUD_ROWS_TOP + s->gh)
      continue;
    if (dc < 0 || dc >= s->gw)
      continue;
    bool sel = (i == g_selected);
    int cp = sel ? CP_SRC_SEL : CP_SRC_OFF;
    chtype gl = sel ? 'X' : 'o';
    attron(COLOR_PAIR(cp) | A_BOLD);
    mvaddch(dr, dc, gl);
    attroff(COLOR_PAIR(cp) | A_BOLD);
  }
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */
/* What you get: terminal extent (Screen struct), the three CLAUDE.md
 *               HUD rows (status row 0 right yellow + bold, params row
 *               1 left yellow, hint row rows-1 left cyan + bold), and
 *               the diff-buffer present pipeline.                       */

/*
 * Screen — terminal-extent record.
 *
 * Intent
 *   Screen is deliberately tiny.  The actual back-buffer (newscr) and
 *   front-buffer (curscr) live INSIDE ncurses; we only carry the cell
 *   dimensions so callers can place HUD rows and bound the field rect.
 *   A user-managed back-buffer would just duplicate ncurses's diff
 *   model with no improvement on flicker.
 *
 * Render pipeline (one frame)
 *   erase()              clears newscr in memory only, NO terminal I/O
 *   scene_draw_field     writes field glyphs → newscr
 *   scene_draw_sources   writes source markers → newscr
 *   hud_draw()           writes HUD rows → newscr (LAST so HUD wins
 *                        overlap with the field)
 *   wnoutrefresh(stdscr) marks newscr ready, STILL no terminal I/O
 *   doupdate()           ONE atomic diff (newscr vs curscr) → terminal
 *
 * Reference [9] ESR's NCURSES HOWTO §11 — the newscr/curscr diff model.
 */
typedef struct {
  int cols; /* terminal width  in cells (getmaxyx)         */
  int rows; /* terminal height in cells (getmaxyx)         */
} Screen;

static void screen_init(Screen *sc, int theme) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init(theme);
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_resize(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_free(Screen *sc) {
  (void)sc;
  endwin();
}

static const char *preset_name(int p) {
  switch (p) {
  case 1:
    return "DBL-SLIT";
  case 2:
    return "RIPPLE";
  case 3:
    return "BEAT";
  case 4:
    return "RADIAL";
  default:
    return "FIVE-OSC";
  }
}

/*
 * HUD layout per CLAUDE.md spec:
 *   row 0       : right-aligned bright-yellow + bold status (fps, engine,
 * state) row 1       : left-aligned  bright-yellow no-bold parameter readouts
 *   row rows-1  : left-aligned  bright-cyan + bold key hint
 * Everything between is the field.
 */
static void hud_draw(Screen *sc, Scene *s, double fps) {
  char status[80];
  snprintf(status, sizeof status, " %s  %5.1f fps  %s ", engine_name(s->engine),
           fps, s->paused ? "PAUSED " : "running");

  attr_t a_status = COLOR_PAIR(CP_HUD) | A_BOLD;
  attron(a_status);
  mvprintw(0, sc->cols - (int)strlen(status), "%s", status);
  attroff(a_status);

  char params[160];
  Source *sel = g_src[g_selected].active ? &g_src[g_selected] : NULL;
  if (s->engine == ENGINE_FDTD) {
    snprintf(params, sizeof params,
             " preset:%d %-8s  theme:%-7s  src:%d/%d sel:%d  "
             "damp:%.3f  steps:%d  ",
             s->preset, preset_name(s->preset), k_themes[s->theme].name,
             g_nactive, N_SRC_MAX, g_selected, s->grid.damping, s->fdtd_steps);
  } else if (sel) {
    snprintf(params, sizeof params,
             " preset:%d %-8s  theme:%-7s  src:%d/%d sel:%d  "
             "lam:%.0f  omega:%.3f  ",
             s->preset, preset_name(s->preset), k_themes[s->theme].name,
             g_nactive, N_SRC_MAX, g_selected, sel->lambda, sel->omega);
  } else {
    snprintf(params, sizeof params,
             " preset:%d %-8s  theme:%-7s  src:0/%d  (no source) ", s->preset,
             preset_name(s->preset), k_themes[s->theme].name, N_SRC_MAX);
  }
  attron(COLOR_PAIR(CP_HUD));
  mvprintw(1, 0, "%s", params);
  attroff(COLOR_PAIR(CP_HUD));

  attr_t a_hint = COLOR_PAIR(CP_HINT) | A_BOLD;
  attron(a_hint);
  mvprintw(sc->rows - 1, 0,
           " q:quit  m:engine  spc:pause  r:reset  1-5:preset  t/T:theme  "
           "TAB:select  arrows:move  n:add  x:del  +/-:lam|steps  "
           "f/F:omega  d/D:damp  p:impulse ");
  attroff(a_hint);
}

static void screen_draw(Screen *sc, Scene *s, double fps) {
  if (s->needs_redraw) {
    erase();
    s->needs_redraw = false;
  } else {
    erase(); /* simplest correct path; ncurses diffs internally */
  }
  scene_draw_field(s);
  scene_draw_sources(s);
  hud_draw(sc, s, fps);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */
/* What you get: the main loop + signal handlers + key dispatch.  This
 *               is where user keystrokes meet the simulation — every
 *               key in the HOW TO READ keys list is wired here.  The
 *               loop is the standard fixed-render-rate pattern: read
 *               input, tick scene, sleep, draw, present.               */

/*
 * App — top-level container, lives in BSS as the single g_app instance.
 *
 * Intent
 *   The signal handlers (on_exit_sig, on_resize) need to reach state
 *   that the main loop polls.  Stashing it in main()'s locals would
 *   force handlers to use globals anyway, so we declare g_app globally
 *   and let handlers and main() share it.
 *
 * Why the volatile sig_atomic_t flags
 *   POSIX requires signal handlers touch only sig_atomic_t variables
 *   and only with simple writes — any wider type or operation is
 *   undefined.  `volatile` forces every read in the main loop to go
 *   back to memory (the compiler must not cache the value into a
 *   register across signal arrival).  Together they form the standard
 *   "wake the main loop" pattern: handler flips a bit, main loop
 *   notices, main loop does the work in user context.
 *
 * What is NOT here
 *   - Frame-timing accumulators (sim_accum, fps_accum, frame_time);
 *     those are pure locals in main().  Lifting them to App would leak
 *     loop bookkeeping into App's public footprint.
 *   - Render FPS / sim FPS:  fixed by RENDER_FPS / RENDER_NS, not
 *     user-adjustable here (sim cadence in FDTD is controlled by
 *     Scene.fdtd_steps instead).
 */
typedef struct {
  Scene scene;                       /* the whole world             */
  Screen screen;                     /* terminal extent             */
  volatile sig_atomic_t running;     /* SIGINT/TERM clears this     */
  volatile sig_atomic_t need_resize; /* SIGWINCH sets this          */
} App;

static App g_app;
static void on_exit_sig(int s) {
  (void)s;
  g_app.running = 0;
}
static void on_resize(int s) {
  (void)s;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

/*
 * key dispatch — engine-context-aware where it matters.
 *   '+' '-' adjust selected source λ in ANALYTIC mode, FDTD sub-steps in FDTD.
 *   'p' drops impulse only in FDTD (analytic has no field state to perturb).
 *   'd' 'D' adjust damping only in FDTD.
 *   Other keys do the same thing in both engines.
 */
static bool app_handle_key(App *a, int ch) {
  Scene *sc = &a->scene;

  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;
  case ' ':
    sc->paused = !sc->paused;
    break;
  case 'm':
  case 'M':
    scene_toggle_engine(sc);
    break;

  case 'r':
  case 'R':
    preset_apply(sc->preset, sc->gw, sc->gh);
    grid_clear(&sc->grid);
    sc->t = 0.f;
    g_at.dirty = true;
    sc->needs_redraw = true;
    break;

  case '1':
  case '2':
  case '3':
  case '4':
  case '5':
    sc->preset = ch - '0';
    preset_apply(sc->preset, sc->gw, sc->gh);
    grid_clear(&sc->grid);
    sc->t = 0.f;
    g_at.dirty = true;
    sc->needs_redraw = true;
    break;

  case 't':
    sc->theme = (sc->theme + 1) % N_THEMES;
    theme_apply(sc->theme);
    sc->needs_redraw = true;
    break;
  case 'T':
    sc->theme = (sc->theme + N_THEMES - 1) % N_THEMES;
    theme_apply(sc->theme);
    sc->needs_redraw = true;
    break;

  case '\t':
    source_select_step(+1);
    break;
  case KEY_BTAB:
    source_select_step(-1);
    break;

  case KEY_UP:
    source_move(0, -1, sc->gw, sc->gh);
    g_at.dirty = true;
    break;
  case KEY_DOWN:
    source_move(0, +1, sc->gw, sc->gh);
    g_at.dirty = true;
    break;
  case KEY_LEFT:
    source_move(-1, 0, sc->gw, sc->gh);
    g_at.dirty = true;
    break;
  case KEY_RIGHT:
    source_move(+1, 0, sc->gw, sc->gh);
    g_at.dirty = true;
    break;

  case 'n':
  case 'N':
    source_add((float)sc->gw * 0.5f, (float)sc->gh * 0.5f, OMEGA_DEF,
               LAMBDA_DEF, 0.f);
    g_at.dirty = true;
    break;
  case 'x':
  case 'X':
    if (g_nactive > 0) {
      source_delete(g_selected);
      g_at.dirty = true;
    }
    break;

  case '+':
  case '=':
    if (sc->engine == ENGINE_ANALYTIC && g_src[g_selected].active) {
      float *l = &g_src[g_selected].lambda;
      *l += LAMBDA_STEP;
      if (*l > LAMBDA_MAX)
        *l = LAMBDA_MAX;
      g_at.dirty = true;
    } else {
      sc->fdtd_steps++;
      if (sc->fdtd_steps > FDTD_STEPS_MAX)
        sc->fdtd_steps = FDTD_STEPS_MAX;
    }
    break;
  case '-':
    if (sc->engine == ENGINE_ANALYTIC && g_src[g_selected].active) {
      float *l = &g_src[g_selected].lambda;
      *l -= LAMBDA_STEP;
      if (*l < LAMBDA_MIN)
        *l = LAMBDA_MIN;
      g_at.dirty = true;
    } else {
      sc->fdtd_steps--;
      if (sc->fdtd_steps < FDTD_STEPS_MIN)
        sc->fdtd_steps = FDTD_STEPS_MIN;
    }
    break;

  case 'f':
    if (g_src[g_selected].active) {
      g_src[g_selected].omega -= OMEGA_STEP;
      if (g_src[g_selected].omega < 0.01f)
        g_src[g_selected].omega = 0.01f;
    }
    break;
  case 'F':
    if (g_src[g_selected].active)
      g_src[g_selected].omega += OMEGA_STEP;
    break;

  case 'd':
    if (sc->engine == ENGINE_FDTD) {
      sc->grid.damping -= DAMPING_STEP;
      if (sc->grid.damping < DAMPING_MIN)
        sc->grid.damping = DAMPING_MIN;
    }
    break;
  case 'D':
    if (sc->engine == ENGINE_FDTD) {
      sc->grid.damping += DAMPING_STEP;
      if (sc->grid.damping > DAMPING_MAX)
        sc->grid.damping = DAMPING_MAX;
    }
    break;

  case 'p':
  case 'P':
    if (sc->engine == ENGINE_FDTD)
      grid_impulse(&sc->grid, sc->grid.cols / 2, sc->grid.rows / 2);
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit_sig);
  signal(SIGTERM, on_exit_sig);
  signal(SIGWINCH, on_resize);

  App *app = &g_app;
  app->running = 1;

  /* screen first so COLORS is initialised before scene_init touches it */
  int initial_theme = 2; /* OCEANIC */
  screen_init(&app->screen, initial_theme);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);
  app->scene.theme = initial_theme;
  theme_apply(initial_theme);

  int64_t ft = clock_ns(), fa = 0;
  int fc = 0;
  double fps = 0.;

  while (app->running) {
    if (app->need_resize) {
      screen_resize(&app->screen);
      scene_resize(&app->scene, app->screen.cols, app->screen.rows);
      app->need_resize = 0;
      ft = clock_ns();
      fa = 0;
      fc = 0;
    }

    int64_t now = clock_ns(), dt = now - ft;
    ft = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    fc++;
    fa += dt;
    if (fa >= 500 * NS_PER_MS) {
      fps = (double)fc / ((double)fa / (double)NS_PER_SEC);
      fc = 0;
      fa = 0;
    }

    scene_tick(&app->scene);

    int64_t elapsed = clock_ns() - ft + dt;
    clock_sleep_ns(RENDER_NS - elapsed);

    screen_draw(&app->screen, &app->scene, fps);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  scene_free(&app->scene);
  screen_free(&app->screen);
  return 0;
}
