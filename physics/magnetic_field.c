/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * magnetic_field.c — 2D Magnetic Field Lines from Configurable Dipoles
 *
 * Visualises the static magnetic field of 1–4 bar magnets (dipoles)
 * placed on screen [1, 4].  Each dipole is modelled as a pair of equal-
 * and-opposite magnetic charges (monopoles) separated by a fixed length
 * — a two-pole approximation valid for r >> dipole length [2, 3].  The
 * field at every point is the vector superposition of Coulomb-like
 * fields from all monopoles:
 *
 *   B(r) = sum_i  q_i * (r − r_i) / |r − r_i|³
 *
 * Field lines (the curves everywhere tangent to B, in Faraday's original
 * sense [4]) are traced by 4th-order Runge-Kutta streamline integration
 * [5] along B, seeded at N_SEEDS points around each positive pole.
 * Arrows on each line segment show the field direction; the 3-tier
 * brightness ramp [8] encodes distance from the source pole.
 *
 * Presets:
 *   0  Dipole      — single bar magnet, classic closed-loop lines
 *   1  Quadrupole  — two magnets head-to-head, X-type neutral point
 *   2  Attract     — two magnets N→S facing each other
 *   3  Repel       — two magnets N→N facing each other
 *
 * Keys:
 *   q / ESC    quit
 *   r          restart / rebuild current preset
 *   n / N      next / previous preset
 *   t / T      next / previous theme
 *   ] / [      trace speed up / down  (lines per tick)
 *   p / space  pause / resume
 *
 * ═════════════════════════════════════════════════════════════════════
 *  REFERENCES  (cite inline as [n])
 * ═════════════════════════════════════════════════════════════════════
 *
 *  ── Magnetism & dipole-field theory ────────────────────────────────
 *
 *   [1] Griffiths, D. J. (2017) — *Introduction to Electrodynamics*,
 *       4th ed., Cambridge University Press.  Ch.5-6 cover magnetic
 *       dipoles and the inverse-cube far-field decay used in the
 *       monopole-pair approximation of §4.  Best gentle starting
 *       point for the physics.
 *
 *   [2] Jackson, J. D. (1998) — *Classical Electrodynamics*, 3rd ed.,
 *       Wiley.  Ch.5 covers magnetostatics and the multipole
 *       expansion; gives the conditions under which the "two-
 *       monopole bar magnet" approximation is valid (r ≫ ℓ_dipole)
 *       and how it breaks down at close range.  Graduate-level
 *       reference; the deeper read after [1].
 *
 *   [3] Purcell, E. M.; Morin, D. J. (2013) — *Electricity and
 *       Magnetism*, 3rd ed., Cambridge.  Famous for its geometric
 *       intuition.  Ch.6 gives the clearest derivation of why field
 *       lines look the way they do — including the loop-closure
 *       property you see in preset 0 (single dipole) and the
 *       X-shaped null point in preset 1 (quadrupole).
 *
 *   [4] Faraday, M. (1855) — *Experimental Researches in Electricity*,
 *       Vol. III, Taylor & Francis.  Series XXVIII (paragraph
 *       3243+).  THE ORIGINAL concept of magnetic FIELD LINES as a
 *       way to visualise the invisible field — the direct ancestor
 *       of the streamlines this demo traces.  Historical primary
 *       source.
 *
 *  ── Numerical streamline integration ───────────────────────────────
 *
 *   [5] Press, W. H.; Teukolsky, S. A.; Vetterling, W. T.; Flannery,
 *       B. P. (2007) — *Numerical Recipes*, 3rd ed., Cambridge.
 *       Ch.17 covers classical RK4 with local-error analysis and
 *       step-size control.  Backs the RK4_H = 0.35 step-size choice
 *       (cell-resolution integration error well below the visible
 *       pixel grid).
 *
 *  ── Vector-field visualisation ─────────────────────────────────────
 *
 *   [6] Helman, J. L.; Hesselink, L. (1991) — "Visualizing Vector
 *       Field Topology in Fluid Flows", *IEEE Computer Graphics &
 *       Applications* 11 (3), 36-46.  Foundational paper on
 *       streamline-based vector-field visualisation.  Catalogues the
 *       "critical points" of a vector field (saddles, foci, nodes)
 *       where streamline integration fails — exactly the null points
 *       this code stops at via the B_MIN cutoff.
 *
 *   [7] Cabral, B.; Leedom, L. C. (1993) — "Imaging vector fields
 *       using line integral convolution", *Proceedings of SIGGRAPH
 *       '93*, 263-270.  Classic alternative to streamline tracing
 *       (the LIC technique fills space with noise-convolved
 *       direction texture).  Cited for context — this demo's
 *       streamline+arrow approach trades smooth coverage for
 *       directional clarity, well-suited to terminal output.
 *
 *  ── Perception & rendering ─────────────────────────────────────────
 *
 *   [8] Ware, C. (2020) — *Information Visualization: Perception
 *       for Design*, 4th ed., Morgan Kaufmann.  Ch.4 on perceptual
 *       contrast and SEQUENTIAL colour mapping backs the three-tier
 *       brightness ramp (CP_LINE_DIM / MID / BRT) encoding distance
 *       from the source pole — the textbook "monotone-luminance"
 *       sequential map.
 *
 * ═════════════════════════════════════════════════════════════════════
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/magnetic_field.c \
 *       -o magnetic_field -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Classical four-stage Runge-Kutta (RK4) streamline
 *                  integration of a 2-D vector field [5, 6].  A field
 *                  line is a curve everywhere tangent to B(r) — the
 *                  original Faraday concept [4].  Starting at seed
 *                  points near the positive poles, RK4 integrates the
 *                  unit-arc-length ODE:
 *                      dx/ds = B(x) / |B(x)|
 *                  Step size RK4_H = 0.35 cells balances visual
 *                  resolution against the cost of up to 600 RK4 steps
 *                  per line (~10 µs / line on commodity CPU).
 *
 * Physics        : Magnetic dipole field via the monopole approximation
 *                  [1, 2].  A bar magnet is modelled as two equal-and-
 *                  opposite "magnetic charges" at its ends; each
 *                  contributes B ∝ q·r̂/r² (Coulomb-like).  The total
 *                  field at any point is the linear superposition
 *                  (Maxwell's equations are linear in B → fields just
 *                  add).  Valid for r >> dipole length [3]; near each
 *                  pole the SOFT regulator caps the inverse-cube
 *                  divergence at a finite value.
 *
 * Math           : NULL POINTS — locations where B = 0 — appear naturally
 *                  between opposing poles (preset 1 quadrupole, preset
 *                  3 repel).  At a null, dx/ds = 0/0 is undefined and
 *                  streamline integration becomes numerically unstable;
 *                  we cut traces at |B| < B_MIN.  Helman & Hesselink [6]
 *                  catalogue these "critical points" as the fundamental
 *                  structural features of a vector field's topology.
 *
 * Rendering      : Three-tier theme-coloured BRIGHTNESS RAMP [8] —
 *                  CP_LINE_BRT for cells close to the source pole (a
 *                  fresh-line look), CP_LINE_MID for mid-distance, and
 *                  CP_LINE_DIM for the tail of the line approaching a
 *                  null point or the other pole.  Per-segment arrow
 *                  glyphs `> v < ^` every ARR_STRIDE step show flow
 *                  direction.  N/S pole markers use the theme's accent
 *                  colours.  Canonical CLAUDE.md two-row HUD: bright
 *                  yellow status (row 0) + bright cyan key legend (row
 *                  rows-1), both A_BOLD across every theme.
 *
 * Performance    : Lines traced progressively — LINES_PER_TICK fully-
 *                  traced lines are REVEALED per render frame.  This
 *                  keeps the UI responsive during tracing: each tick
 *                  draws a few more lines rather than blocking for
 *                  seconds at startup.  Internally trace_line is still
 *                  O(MAX_LINE_STEPS) per line; the throttle is purely
 *                  for visual pacing.
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
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
  SIM_FPS_DEFAULT = 30,
  SIM_FPS_MIN = 5,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 5,

  MAX_MONOPOLES = 8,    /* max magnetic charges (2 per dipole)        */
  MAX_DIPOLES = 4,      /* max bar magnets                            */
  N_SEEDS = 16,         /* field lines seeded around each + pole      */
  MAX_LINE_STEPS = 600, /* max RK4 steps per field line               */
  MAX_LINES = MAX_DIPOLES * N_SEEDS, /* total lines to trace     */

  N_PRESETS = 4,
  N_THEMES = 5,

  LINES_PER_TICK_DEF = 2, /* field lines traced per tick                */
  LINES_PER_TICK_MIN = 1,
  LINES_PER_TICK_MAX = 8,

  ROWS_MAX = 128,
  COLS_MAX = 512,
};

#define NS_PER_SEC 1000000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* RK4 step size in normalised cell units */
#define RK4_H 0.35f
/* Stop tracing if |B| < this (near null points) */
#define B_MIN 1e-6f
/* Monopole softening radius in cell units */
#define SOFT 0.8f
/* Seed circle radius around a pole, in cell units */
#define SEED_R 1.2f
/* Arrow placed every ARR_STRIDE steps along a line */
#define ARR_STRIDE 18
/* Aspect ratio correction: terminal cells are ~2x taller than wide */
#define ASPECT_R 2.0f

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color / theme                                                      */
/* ===================================================================== */

/*
 * Color pair assignments:
 *   1   field lines (positive/dim) — theme-dependent
 *   2   field lines (stronger)     — theme-dependent
 *   3   field lines (brightest)    — theme-dependent
 *   4   North pole symbol (N)      — theme-dependent
 *   5   South pole symbol (S)      — theme-dependent
 *   6   dipole body                — theme-dependent
 *   7   HUD status (top row)       — canonical bright yellow 226
 *   8   HINT action keys (bot row) — canonical bright cyan  51
 *
 * CP_HUD and CP_HINT are CANONICAL (CLAUDE.md HUD Standard) — they
 * stay yellow/cyan across every theme so the two reserved HUD rows
 * remain legible against any field-line palette.
 */
enum {
  CP_LINE_DIM = 1,
  CP_LINE_MID = 2,
  CP_LINE_BRT = 3,
  CP_NORTH = 4,
  CP_SOUTH = 5,
  CP_BODY = 6,
  CP_HUD = 7,
  CP_HINT = 8,
};

/* HUD inset rows reserved at the top + bottom of the terminal.  Field
 * lines render in the band [HUD_TOP_ROWS, term_rows - HUD_BOT_ROWS). */
#define HUD_TOP_ROWS 1
#define HUD_BOT_ROWS 1

/*
 * Theme — one named visual identity (256-colour primary + 8-colour fallback).
 *
 * WHY a struct, not loose const arrays:
 *   The three field-line ramp colours (dim → mid → bright) must form
 *   a PERCEPTUALLY MONOTONE LUMINANCE sequence so the brightness ramp
 *   reads as "distance from the source pole" (Ware [ref 8], Ch.4
 *   sequential colour maps).  The pole/body markers must contrast
 *   against the ramp.  Bundling these correlated colours together
 *   forbids accidentally mixing ramp tier 0 of theme A with ramp
 *   tier 1 of theme B during edits.
 *
 * WHY a dual palette (256 + 8):
 *   Modern terminals expose 256 indexed colours; legacy TTYs and
 *   minimal $TERM values ("linux", "dumb") expose only 8.
 *   theme_apply() inspects ncurses COLORS at runtime and binds the
 *   matching pair set — no #ifdef ladder, the same binary works in
 *   both environments.  Both sets describe the SAME theme intent;
 *   the 8-colour set is just a coarser approximation.
 *
 * WHY CP_HUD / CP_HINT are deliberately NOT here:
 *   The CLAUDE.md HUD Standard fixes them at bright yellow (226) +
 *   bright cyan (51) GLOBALLY so they remain legible against any
 *   field-line palette.  Putting them in a per-theme struct would
 *   invite drift toward dim/coloured HUDs that disappear on busy
 *   backgrounds.  color_init() binds them once; theme_apply() never
 *   touches them.
 *
 * Brightness floor:  Every entry must sit in the bright half of the
 * 256-colour space (CLAUDE.md Theme Palette Brightness — colours
 * 16-23 / 232-239 become invisible against default-black + A_DIM).
 */
typedef struct {
  /* ── 256-colour palette (preferred when COLORS >= 256) ────────── *
   * Indices into the xterm-256 cube.  Listed in the order they
   * appear visually on a traced line — dim at the far/null end,
   * brightest at the source-pole end. */
  short line_dim; /* CP_LINE_DIM — tail of line near a null point or
                   *               the opposite pole; LOWEST luminance
                   *               tier of the ramp [ref 8]            */
  short line_mid; /* CP_LINE_MID — middle third of the line;
                   *               medium luminance tier               */
  short line_brt; /* CP_LINE_BRT — first third near the source pole,
                   *               the "freshest" look; BRIGHTEST tier */
  short north;    /* CP_NORTH    — 'N' pole marker, bold accent —
                   *               must POP against the ramp           */
  short south;    /* CP_SOUTH    — 'S' pole marker, contrasting accent
                   *               (typically the colour-wheel opposite
                   *               of `north` for visual polarity)     */
  short body;     /* CP_BODY     — '=' body line drawn between N and S
                   *               poles of the same magnet; subdued
                   *               so it doesn't compete with the ramp */

  /* ── 8-colour ANSI fallback (used when COLORS < 256) ──────────── *
   * Same semantic roles as above, expressed in the 8-colour ANSI
   * basis (COLOR_RED, COLOR_BLUE, …).  Granularity is coarser, so
   * the ramp here uses one accent colour repeated rather than three
   * luminance tiers — readability is preserved by relying on
   * line-density gradients instead of colour-tier separation. */
  short line_dim8, line_mid8, line_brt8;
  short north8, south8, body8;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* Electric — cyan field on black */
    {51, 87, 231, 196, 21, 250, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE, COLOR_RED,
     COLOR_BLUE, COLOR_WHITE},
    /* Plasma — magenta/violet */
    {93, 165, 207, 226, 57, 248, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE,
     COLOR_YELLOW, COLOR_BLUE, COLOR_WHITE},
    /* Fire — red/orange field */
    {124, 208, 226, 231, 21, 250, COLOR_RED, COLOR_RED, COLOR_YELLOW,
     COLOR_WHITE, COLOR_BLUE, COLOR_WHITE},
    /* Ocean — blue/teal */
    {25, 39, 123, 196, 226, 250, COLOR_BLUE, COLOR_CYAN, COLOR_WHITE, COLOR_RED,
     COLOR_YELLOW, COLOR_WHITE},
    /* Matrix — green */
    {22, 46, 118, 196, 21, 250, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE,
     COLOR_RED, COLOR_BLUE, COLOR_WHITE},
};

/* Theme-dependent pairs only — CP_HUD / CP_HINT live in color_init(). */
static void theme_apply(int t) {
  const Theme *th = &k_themes[t];
  if (COLORS >= 256) {
    init_pair(CP_LINE_DIM, th->line_dim, COLOR_BLACK);
    init_pair(CP_LINE_MID, th->line_mid, COLOR_BLACK);
    init_pair(CP_LINE_BRT, th->line_brt, COLOR_BLACK);
    init_pair(CP_NORTH, th->north, COLOR_BLACK);
    init_pair(CP_SOUTH, th->south, COLOR_BLACK);
    init_pair(CP_BODY, th->body, COLOR_BLACK);
  } else {
    init_pair(CP_LINE_DIM, th->line_dim8, COLOR_BLACK);
    init_pair(CP_LINE_MID, th->line_mid8, COLOR_BLACK);
    init_pair(CP_LINE_BRT, th->line_brt8, COLOR_BLACK);
    init_pair(CP_NORTH, th->north8, COLOR_BLACK);
    init_pair(CP_SOUTH, th->south8, COLOR_BLACK);
    init_pair(CP_BODY, th->body8, COLOR_BLACK);
  }
}

/* One-shot at startup — binds the two CANONICAL HUD pairs (bright
 * yellow status, bright cyan hint) then applies the initial theme. */
static void color_init(void) {
  start_color();
  use_default_colors();

  if (COLORS >= 256) {
    init_pair(CP_HUD, 226, -1); /* canonical bright yellow */
    init_pair(CP_HINT, 51, -1); /* canonical bright cyan   */
  } else {
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }

  theme_apply(0);
}

/* ===================================================================== */
/* §4  magnetic physics                                                   */
/* ===================================================================== */

/*
 * Monopole — a single magnetic point charge.
 *
 * ALGORITHM context (Griffiths Ch.5 [ref 1], Jackson §5 [ref 2]):
 *   The demo models each bar magnet as a MONOPOLE PAIR — two
 *   equal-and-opposite point charges separated by a fixed length.
 *   Each pole contributes a Coulomb-like 2-D field at any sample
 *   point r:
 *
 *       B_i(r) = q_i * (r − r_i) / |r − r_i|^3
 *
 *   The total field at r is the LINEAR SUPERPOSITION of every
 *   monopole's contribution (Maxwell's equations are linear in B,
 *   so fields add).  This is the workhorse formula evaluated inside
 *   field_at() — the hot loop of the whole simulation.
 *
 * WHY split a dipole into two monopoles (vs. an analytic dipole formula):
 *   Treating poles INDIVIDUALLY makes superposition trivial — one
 *   uniform loop in field_at() handles N dipoles regardless of
 *   their orientation.  The analytic point-dipole formula (with a
 *   moment vector m) is mathematically tighter but breaks down in
 *   the near field where r ~ ℓ_dipole — exactly where the most
 *   interesting structure of this demo lives (null points between
 *   poles, X-saddles in the quadrupole preset).  Purcell [ref 3]
 *   uses the same pole-pair picture for the same pedagogical reason.
 *
 * Physical caveat:
 *   Isolated magnetic monopoles have never been observed.  But at
 *   scales r >> ℓ_dipole the field of a real bar magnet is
 *   INDISTINGUISHABLE from a pair of opposite charges at its ends
 *   — that's the modelling trick this struct encodes.
 */
typedef struct {
  float cx, cy; /* centre in CELL coords (not pixels) — physics and
                 * rendering share the same grid, so no coordinate
                 * conversion is needed between field_at() and
                 * scene_draw().  Floats because the integrator
                 * advances in sub-cell RK4 steps (RK4_H = 0.35). */
  float q;      /* magnetic "charge":
                 *   +1.0f = NORTH  (field SOURCE — lines emerge),
                 *   -1.0f = SOUTH  (field SINK   — lines converge).
                 * Magnitude is in ARBITRARY UNITS — only the
                 * ratio between poles affects line shape.  |B|'s
                 * absolute scale is normalised away by the
                 * unit-arc-length ODE in trace_line(), which
                 * divides by |B| at every step. */
} Monopole;

/*
 * Dipole — a bar magnet, kept as a RENDER-ONLY twin of two Monopoles.
 *
 * DUAL representation (this is the key design idea):
 *   Each magnet exists TWICE inside Scene —
 *     • PHYSICALLY as two entries in Scene.mp[]   (used by field_at)
 *     • VISUALLY  as one  entry  in Scene.dp[]   (used by scene_draw)
 *
 * WHY the duplication is intentional (not a code smell):
 *   The physics layer wants a FLAT list of point charges to loop
 *   over — adding a "magnet body" concept there would slow the inner
 *   loop with no physical effect (the body itself has no charge).
 *   The render layer wants the OPPOSITE — knowing which N and which
 *   S belong to the same magnet so it can draw the '=' body line and
 *   the matching N/S markers as a single coherent shape.
 *   scene_add_dipole() writes both views atomically so they can't
 *   desync; nothing else touches mp[] / dp[] after that.
 *
 * Memory locality note:
 *   mp[] and dp[] live back-to-back inside Scene (see Scene's locality
 *   regions A and B).  scene_draw() iterates dp[] sequentially in
 *   cache-friendly order; field_at() iterates mp[] the same way.
 *   Neither pass ever touches the other's array.
 */
typedef struct {
  float nx, ny; /* NORTH pole position (cell coords) — matches the
                 * q=+1 monopole that scene_add_dipole() spawned for
                 * this magnet.  Duplicates information in mp[] but
                 * spares scene_draw() a search through mp[]. */
  float sx, sy; /* SOUTH pole position (cell coords) — matches the
                 * q=−1 monopole partner of (nx, ny). */
  int color;    /* colour-pair index for the body glyph (the '='
                 * line between N and S).  Currently always CP_BODY;
                 * field is reserved for per-magnet tinting if a
                 * future preset wants colour-coded magnets (e.g.
                 * red for the "active" one).  Kept as a struct
                 * field rather than a global so the choice is
                 * per-dipole rather than per-frame. */
} Dipole;

/*
 * Compute B field at (px, py) from all monopoles.
 * B += q_i * (r - r_i) / |r - r_i|^3
 * Softening avoids divergence at the pole center.
 */
static void field_at(const Monopole *mp, int nm, float px, float py, float *bx,
                     float *by) {
  *bx = 0.0f;
  *by = 0.0f;
  for (int i = 0; i < nm; i++) {
    float dx = px - mp[i].cx;
    float dy = (py - mp[i].cy) * ASPECT_R; /* aspect correction */
    float r2 = dx * dx + dy * dy + SOFT * SOFT;
    float r = sqrtf(r2);
    float inv3 = mp[i].q / (r2 * r);
    *bx += inv3 * dx;
    *by += inv3 * dy / ASPECT_R;
  }
}

/* ===================================================================== */
/* §5  scene — field lines & dipoles                                      */
/* ===================================================================== */

/*
 * FieldLine — one fully pre-rasterised streamline.
 *
 * ALGORITHM context (Helman & Hesselink [ref 6], Numerical Recipes
 * Ch.17 [ref 5]):
 *   A streamline is a curve everywhere tangent to a vector field —
 *   for a static B, it IS a Faraday field line [ref 4].
 *   trace_line() integrates the UNIT-ARC-LENGTH ODE
 *
 *       dx/ds = B(x) / |B(x)|
 *
 *   with classical 4th-order Runge-Kutta, starting from a seed point
 *   near a north pole.  Integration stops at whichever comes first:
 *     • the curve leaves the visible grid,
 *     • |B| < B_MIN — a NULL POINT (Helman & Hesselink "critical
 *                     point" [ref 6]; integrator divides by 0 there),
 *     • MAX_LINE_STEPS exhausted (hard cap, guarantees bounded work).
 *
 *   Normalising by |B| means the line advances in EQUAL ARC-LENGTH
 *   STEPS regardless of field strength — without this, a strong field
 *   near a pole would overshoot wildly while a weak field far away
 *   would crawl pixel by pixel.
 *
 * WHY pre-rasterise (= store col/row/ch/cp instead of float curves):
 *   Tracing is expensive (up to 600 RK4 evaluations × 4 field_at
 *   calls each); drawing must be cheap (every frame, ~60 Hz).
 *   Rasterising once at trace time means scene_draw() degenerates
 *   to a tight mvaddch loop with no math, no field_at calls, no
 *   character or colour decisions per frame.  A traced line "redraws
 *   for free" every frame until the preset changes.
 *
 * WHY parallel arrays (not an array of step-structs):
 *   The render loop reads col/row to position, then ch + cp to paint.
 *   Parallel arrays put each access onto its own contiguous run of
 *   memory, friendly to the prefetcher.  A struct-of-cells layout
 *   (Step{col,row,ch,cp}[]) would pull all four fields into cache
 *   lines that the render loop doesn't fully use per element.
 *   At MAX_LINE_STEPS=600 the absolute savings are small; the bigger
 *   win is the clearer intent: "this is the column array".
 *
 * Memory cost:  ~4.8 KB / line × MAX_LINES (64) ≈ 310 KB scene-wide.
 * All BSS-resident inside Scene — no malloc on the hot path
 * (CLAUDE.md Memory rule).
 */
typedef struct {
  /* Pre-rasterised cell trail.  ALL four arrays share index i:
   * index 0 = seed cell near the +pole, index len-1 = terminus
   * (off-grid, near a null, or step cap).  Entries past `len` are
   * uninitialised — DO NOT read past len. */
  int col[MAX_LINE_STEPS]; /* terminal COLUMN of step i (cell coords) */
  int row[MAX_LINE_STEPS]; /* terminal ROW    of step i (cell coords) */
  char ch[MAX_LINE_STEPS]; /* glyph chosen at TRACE TIME from the
                            * tangent direction:
                            *   '- | / \'  line segment (line_char,
                            *              8-way angle bucket)
                            *   '> v < ^'  ARROW glyph every
                            *              ARR_STRIDE steps
                            *              (direction_arrow) so the
                            *              reader sees the FLOW
                            *              DIRECTION, not just the
                            *              line geometry            */
  int cp[MAX_LINE_STEPS];  /* CP_LINE_DIM / MID / BRT — distance-
                            * from-source brightness ramp [ref 8].
                            * Filled AFTER the full trace (need to
                            * know `len` to split into three tiers
                            * — see line_color_pair).               */
  int len;                 /* steps actually used (0..MAX_LINE_STEPS).
                            * 0 means the seed was already at a
                            * null/off-grid — caller still treats
                            * the line as done.                     */
  bool done;               /* trace completed marker — set true at
                            * the END of trace_line().  Guards the
                            * render pass from drawing a partially-
                            * traced line if tracing ever becomes
                            * incremental in the future.            */
} FieldLine;

/*
 * Scene — every piece of state for one (preset × theme × terminal-size)
 *         combination.  Rebuilt atomically on any of those changing.
 *
 * LAYOUT — fields are grouped into FOUR locality regions, ordered the
 * way each per-frame pass touches them:
 *
 *   ┌─────────────────────────────────────────────────────────────────┐
 *   │ (A) PHYSICS     — mp[], nm                                       │
 *   │     read by  : field_at()    (hot loop, ~600 calls per line)     │
 *   │     written  : scene_add_dipole() at init only                   │
 *   │                                                                  │
 *   │ (B) RENDER     — dp[], nd, lines[], n_lines, lines_traced        │
 *   │     read by  : scene_draw()  (every frame)                       │
 *   │     written  : scene_seed_lines() at init, scene_tick() per tick │
 *   │                                                                  │
 *   │ (C) ANIMATION — lines_per_tick, paused, preset, theme            │
 *   │     read by  : scene_tick(), screen_draw_hud()                   │
 *   │     written  : input handler in main()                           │
 *   │                                                                  │
 *   │ (D) GEOMETRY  — cols, rows                                       │
 *   │     read by  : scene_draw(), trace_line() bounds check           │
 *   │     written  : scene_init() ONLY (one-shot at init/resize)       │
 *   └─────────────────────────────────────────────────────────────────┘
 *
 * WHY this grouping (locality + clarity):
 *   • Each per-frame pass touches a CONTIGUOUS region — friendly to
 *     the prefetcher and to a reader scanning top-to-bottom (the
 *     access pattern matches the file layout).
 *   • Tick reads (C), writes (C) — no surprise mutations to (A)/(B).
 *   • Draw reads (B) + (D) — no read of physics arrays (mp[]).
 *   • Init writes everything; after that, (A) and (D) are frozen for
 *     the scene's lifetime and (B) is frozen after seeding completes.
 *   This separation is enforced by CONVENTION here, not by separate
 *   structs — see "why one big struct" below.
 *
 * WHY ONE big struct (not split PhysicsScene + RenderScene + UIScene):
 *   The entire scene rebuilds atomically on preset / theme / resize
 *   change.  Keeping it together makes scene_init() = memset + fill,
 *   and makes every field's lifetime obvious (= lifetime of Scene
 *   itself).  Splitting would introduce a pointer hop and a second
 *   allocation site with no behavioural win, and risk drift between
 *   the splits (e.g. a resize that rebuilds physics but forgets
 *   visuals).  CLAUDE.md memory rule also wants ZERO malloc on the
 *   hot path — a single stack/BSS Scene satisfies that trivially.
 *
 * Size:  ~310 KB (dominated by lines[]).  Sits in main()'s stack
 * frame; no dynamic allocation.
 */
typedef struct {
  /* ── (A) PHYSICS DATA ─────────────────────────────────────────── *
   * Flat array of magnetic point charges, the INPUT to field_at().
   * Order is arbitrary — superposition is commutative (refs [1, 2])
   * — so a single uniform loop handles N dipoles of any orientation.
   * Indexed 0 .. nm-1; entries past nm are uninitialised garbage. */
  Monopole mp[MAX_MONOPOLES]; /* the point charges (2 per dipole) */
  int nm;                     /* live count — ALWAYS even (poles
                               * come in N/S pairs)              */

  /* ── (B) RENDER DATA — visual representation, decoupled from (A) ─ *
   * Two sub-pools:
   *   dp[]     — magnet bodies (knowing which N and S pair up)
   *   lines[]  — pre-rasterised streamlines, traced ONCE in
   *              scene_seed_lines() then redrawn every frame.
   * scene_draw() touches ONLY this region (+ region D); the physics
   * arrays are never read during rendering. */
  Dipole dp[MAX_DIPOLES]; /* magnet bodies (N→S geometry pairs) */
  int nd;                 /* live dipole count                  */

  FieldLine lines[MAX_LINES]; /* pre-rasterised streamlines, BSS-
                               * resident — no malloc on hot path  */
  int n_lines;                /* total lines TRACED for this preset
                               * (set once during seeding)         */
  int lines_traced;           /* lines REVEALED to the viewer so far
                               * (≤ n_lines).  Progressive reveal:
                               * scene_draw() paints only
                               * lines[0 .. lines_traced) so the
                               * startup isn't a blocking pause —
                               * the user sees field lines appear
                               * one batch per tick.               */

  /* ── (C) ANIMATION CONTROL — user-tunable per-frame state ────── *
   * These are the keybind targets — every value here is owned by
   * the input handler in main() and read by tick + HUD draw.  No
   * physics ever depends on these (changing preset/theme rebuilds
   * regions A/B from scratch via scene_init).                       */
  int lines_per_tick; /* [/]+ keys — lines revealed per
                       * tick.  Visual PACING only; the
                       * underlying trace work is already
                       * done at seed time.               */
  bool paused;        /* space / p — freeze the reveal
                       * animation.  Physics is static, so
                       * "pause" only halts incrementing
                       * lines_traced.                    */
  int preset;         /* 0 .. N_PRESETS-1 active scenario
                       * (Dipole / Quadrupole / Attract /
                       * Repel).  Changing this triggers a
                       * full scene rebuild.              */
  int theme;          /* 0 .. N_THEMES-1 active palette.
                       * Changing this re-binds colour
                       * pairs but does NOT rebuild lines
                       * — line geometry is theme-free.   */

  /* ── (D) GEOMETRY — render-band dimensions ───────────────────── *
   * The INSET render band size, NOT full terminal dimensions:
   *   rows = term_rows − HUD_TOP_ROWS − HUD_BOT_ROWS
   *   cols = term_cols
   * The HUD rows live OUTSIDE this coordinate system — scene_draw()
   * shifts every mvaddch by HUD_TOP_ROWS so the canvas starts at
   * terminal row 1.  Set once per init/resize and frozen thereafter. */
  int cols, rows;
} Scene;

/* ── Arrow glyph from tangent direction (4-way, picked from 8 sectors) ─ *
 * Bucket the unit tangent into one of 8 angular sectors of width π/4,
 * then map antipodal sectors to the same glyph: → and ← collapse onto
 * '>' / '<', ↓ and ↑ onto 'v' / '^'.  4 distinct outputs is enough — a
 * field line at 90° vs 270° points the SAME way along its trail; only
 * the cardinal direction matters for the reader.                       */
static char direction_arrow(float dx, float dy) {
  float angle = atan2f(dy * ASPECT_R, dx);
  if (angle < 0)
    angle += 2.0f * (float)M_PI; /* → [0, 2π)  */
  int sector = (int)(angle / ((float)M_PI / 4.0f) + 0.5f) % 8;
  static const char sym[4] = {'>', 'v', '<', '^'};
  return sym[sector % 4];
}

/* ── Line character from direction ────────────────────────────────── */

static char line_char(float dx, float dy) {
  float angle = atan2f(dy * ASPECT_R, dx);
  if (angle < 0)
    angle += (float)M_PI;
  if (angle >= (float)M_PI)
    angle -= (float)M_PI;
  /* normalize to [0, pi) and pick character */
  float deg = angle * 180.0f / (float)M_PI;
  if (deg < 22.5f || deg >= 157.5f)
    return '-';
  if (deg < 67.5f)
    return '\\';
  if (deg < 112.5f)
    return '|';
  return '/';
}

/* ── Color pair for a step along a line based on distance from pole ── */

static int line_color_pair(int step, int total) {
  /* bright near source pole, dim farther away */
  if (total <= 0)
    return CP_LINE_DIM;
  int third = total / 3;
  if (step < third)
    return CP_LINE_BRT;
  if (step < 2 * third)
    return CP_LINE_MID;
  return CP_LINE_DIM;
}

/* ── Helpers for trace_line — each one names a single algorithmic step ─ */

/* GRID DOMAIN — is the current integrator position inside the renderable
 * grid?  Tested as float (the integrator advances in sub-cell steps) AND
 * as the rounded cell index (defensive against floor/ceil rounding at
 * the boundary).  Returns the rounded cell via out-params so the caller
 * doesn't repeat the same rounding. */
static bool cell_in_grid(float px, float py, int cols, int rows, int *out_col,
                         int *out_row) {
  if (px < 0 || px >= (float)cols || py < 0 || py >= (float)rows)
    return false;
  int col = (int)(px + 0.5f);
  int row = (int)(py + 0.5f);
  if (col < 0 || col >= cols || row < 0 || row >= rows)
    return false;
  *out_col = col;
  *out_row = row;
  return true;
}

/* CLASSICAL RK4 EVALUATION (Numerical Recipes Ch.17 [ref 5]) —
 * one step of the streamline ODE
 *
 *     dx/ds = B(x)
 *
 * with four field samples at (start, mid₁, mid₂, end) blended via
 *
 *     B̄ = (k₁ + 2k₂ + 2k₃ + k₄) / 6
 *
 * giving local truncation error O(h⁵).  Then the result is NORMALISED
 * to unit length so the caller advances in equal arc-length steps
 * (without normalisation, near-pole fields would overshoot wildly).
 *
 * Returns false when |B̄| < B_MIN — the position is at a NULL POINT
 * [ref 6] where the tangent is undefined and integration must stop. */
static bool rk4_unit_tangent(const Monopole *mp, int nm, float px, float py,
                             float h, float *tx, float *ty) {
  float k1x, k1y, k2x, k2y, k3x, k3y, k4x, k4y;
  field_at(mp, nm, px, py, &k1x, &k1y);
  field_at(mp, nm, px + 0.5f * h * k1x, py + 0.5f * h * k1y, &k2x, &k2y);
  field_at(mp, nm, px + 0.5f * h * k2x, py + 0.5f * h * k2y, &k3x, &k3y);
  field_at(mp, nm, px + h * k3x, py + h * k3y, &k4x, &k4y);

  float bx = (k1x + 2.0f * k2x + 2.0f * k3x + k4x) / 6.0f;
  float by = (k1y + 2.0f * k2y + 2.0f * k3y + k4y) / 6.0f;
  float bmag = sqrtf(bx * bx + by * by);
  if (bmag < B_MIN)
    return false; /* null point — stop */

  *tx = bx / bmag; /* unit-arc-length tangent */
  *ty = by / bmag;
  return true;
}

/* GLYPH SELECTION — every ARR_STRIDE steps we punctuate the trail with
 * a direction arrow so the reader sees FLOW DIRECTION (where does this
 * line point?); the steps in between get the 4-way angle-bucketed line
 * glyph for shape.  Step 0 is always a line glyph — placing an arrow on
 * the seed cell would visually compete with the 'N' pole marker. */
static char glyph_for_step(int step, float tx, float ty) {
  bool is_arrow_step = (step > 0 && step % ARR_STRIDE == 0);
  return is_arrow_step ? direction_arrow(tx, ty) : line_char(tx, ty);
}

/* APPEND ONE TRACED CELL — pure side-effect on fl, no math.  Colour
 * pair is left at 0 (= "unassigned"); the full-trace distance ramp
 * fills it in after the loop, once `len` is known. */
static void record_traced_cell(FieldLine *fl, int col, int row, char ch) {
  fl->col[fl->len] = col;
  fl->row[fl->len] = row;
  fl->ch[fl->len] = ch;
  fl->cp[fl->len] = 0;
  fl->len++;
}

/* DISTANCE-FROM-SOURCE BRIGHTNESS RAMP [Ware ref 8] — split the line
 * into thirds, brightest near the source pole, dimmest near the
 * terminus (null point or off-grid).  Runs AFTER the trace because the
 * three tiers are defined relative to `len`, which is only known when
 * the integrator stops. */
static void apply_distance_brightness_ramp(FieldLine *fl) {
  for (int i = 0; i < fl->len; i++) {
    fl->cp[i] = line_color_pair(i, fl->len);
  }
}

/* ── RK4 streamline trace — Faraday field line from one seed ─────────
 *
 * Pseudocode (Helman & Hesselink streamline integration [ref 6]):
 *
 *     position ← seed
 *     for step = 0 .. MAX_LINE_STEPS:
 *         if position outside grid:           break
 *         tangent ← RK4 unit-direction at position
 *         if no valid tangent (null point):   break
 *         glyph   ← arrow at stride / line otherwise
 *         record cell at position
 *         position ← position + h · tangent
 *     colour the trail by distance from source
 */
static void trace_line(FieldLine *fl, const Monopole *mp, int nm, float sx,
                       float sy, int cols, int rows) {
  fl->len = 0;
  fl->done = true;

  float px = sx, py = sy;

  for (int step = 0; step < MAX_LINE_STEPS; step++) {
    int col, row;
    if (!cell_in_grid(px, py, cols, rows, &col, &row))
      break;

    float tx, ty;
    if (!rk4_unit_tangent(mp, nm, px, py, RK4_H, &tx, &ty))
      break;

    record_traced_cell(fl, col, row, glyph_for_step(step, tx, ty));

    px += RK4_H * tx;
    py += RK4_H * ty;
  }

  apply_distance_brightness_ramp(fl);
}

/* ── Add a dipole: north at (nx,ny), south at (sx,sy) ────────────── */

static void scene_add_dipole(Scene *s, float nx, float ny, float sx, float sy) {
  if (s->nd >= MAX_DIPOLES || s->nm + 2 > MAX_MONOPOLES)
    return;

  s->mp[s->nm] = (Monopole){nx, ny, +1.0f};
  s->nm++;
  s->mp[s->nm] = (Monopole){sx, sy, -1.0f};
  s->nm++;

  s->dp[s->nd] = (Dipole){nx, ny, sx, sy, CP_BODY};
  s->nd++;
}

/* ── Seed field lines from every north pole ──────────────────────────
 *
 * Around each +q monopole, place N_SEEDS seed points on a small circle
 * of radius SEED_R (aspect-corrected in y so the circle LOOKS round
 * on terminals with ~2:1 cell aspect), and trace one streamline from
 * each.  Tracing is done eagerly here — the per-tick "reveal" later
 * only advances the visible count, not the trace itself.
 */
static void scene_seed_lines(Scene *s) {
  s->n_lines = 0;
  s->lines_traced = 0;

  for (int i = 0; i < s->nm && s->n_lines < MAX_LINES; i++) {
    if (s->mp[i].q < 0)
      continue; /* only +q (north) poles seed lines */

    for (int k = 0; k < N_SEEDS && s->n_lines < MAX_LINES; k++) {
      float angle = (float)k / (float)N_SEEDS * 2.0f * (float)M_PI;
      float sx = s->mp[i].cx + SEED_R * cosf(angle);
      float sy = s->mp[i].cy + SEED_R * sinf(angle) / ASPECT_R;

      trace_line(&s->lines[s->n_lines], s->mp, s->nm, sx, sy, s->cols, s->rows);
      s->n_lines++;
    }
  }
}

/* ── Preset layouts — one helper per magnetic configuration ──────────
 *
 * Each helper places magnets around the scene centre (cx, cy) using
 * scaled offsets (dx, dy).  Naming is physics-led: the function name
 * is the configuration name as you'd find it in a magnetostatics text
 * (Purcell Ch.6 [ref 3] — "single dipole", "quadrupole", "facing
 * pairs in attract / repel arrangement").
 */

/* PRESET 0 — single bar magnet, horizontal.  Classic CLOSED-LOOP field
 * lines emerging from N, sweeping around, returning to S.  The most
 * basic dipole pattern (Griffiths §5 [ref 1]). */
static void preset_single_bar_magnet(Scene *s, float cx, float cy, float dx) {
  scene_add_dipole(s, cx - dx, cy, /* north pole — left */
                   cx + dx, cy);   /* south pole — right */
}

/* PRESET 1 — magnetic QUADRUPOLE.  Two crossed dipoles share a centre:
 * one vertical (N on top), one horizontal.  The four poles form a
 * symmetric arrangement whose superposed field has a CRITICAL POINT
 * of saddle topology (X-shaped null) at the centre — the canonical
 * "X-type neutral point" of Helman & Hesselink [ref 6]. */
static void preset_quadrupole(Scene *s, float cx, float cy, float dx,
                              float dy) {
  scene_add_dipole(s, cx, cy - dy * 0.8f, /* vertical dipole: N on top */
                   cx, cy + dy * 0.8f);   /*                  S on bottom */
  scene_add_dipole(s, cx - dx * 1.4f, cy, /* horizontal dipole: N far-left */
                   cx + dx * 1.4f, cy);   /*                    S far-right */
}

/* PRESET 2 — ATTRACT pair (N→S facing N→S).  Two horizontal magnets
 * laid end-to-end with OPPOSITE poles meeting at the centre:
 *
 *     N------S    N------S
 *   ←─ left ─┘    └─ right ─→
 *
 * Equivalent to one long bar magnet — field lines stream continuously
 * from the far-left N to the far-right S through the inner S/N pair. */
static void preset_attract_pair(Scene *s, float cx, float cy, float dx) {
  scene_add_dipole(s, cx - dx * 1.8f,
                   cy, /* LEFT magnet:  N far-left,  S near-centre */
                   cx - dx * 0.4f, cy);
  scene_add_dipole(s, cx + dx * 0.4f,
                   cy, /* RIGHT magnet: N near-centre, S far-right */
                   cx + dx * 1.8f, cy);
}

/* PRESET 3 — REPEL pair (N→N facing N→N).  Two horizontal magnets laid
 * end-to-end with LIKE poles meeting at the centre:
 *
 *     S------N    N------S
 *   ←─ left ─┘    └─ right ─→
 *
 * The two near-centre N poles push each other away — the field has a
 * null point between them (Helman & Hesselink saddle [ref 6]).  Lines
 * curve sharply outward, never crossing the midplane. */
static void preset_repel_pair(Scene *s, float cx, float cy, float dx) {
  scene_add_dipole(s, cx - dx * 0.4f,
                   cy, /* LEFT magnet:  N near-centre, S far-left  */
                   cx - dx * 1.8f, cy);
  scene_add_dipole(s, cx + dx * 0.4f,
                   cy, /* RIGHT magnet: N near-centre, S far-right */
                   cx + dx * 1.8f, cy);
}

/*
 * scene_build_preset — dispatch the preset id to one of the four
 * magnetic configurations above.  Reads as pure preset SELECTION; all
 * placement math lives in the named helpers.
 */
static void scene_build_preset(Scene *s) {
  /* Reference frame for placement: scene centre + scaled half-spans.
   * Layouts express positions as cx ± k·dx, cy ± k·dy so they
   * auto-scale with the terminal size. */
  float cx = (float)s->cols * 0.5f;
  float cy = (float)s->rows * 0.5f;
  float dx = (float)s->cols * 0.18f;
  float dy = (float)s->rows * 0.28f;

  s->nm = 0;
  s->nd = 0;

  switch (s->preset) {
  case 0:
    preset_single_bar_magnet(s, cx, cy, dx);
    break;
  case 1:
    preset_quadrupole(s, cx, cy, dx, dy);
    break;
  case 2:
    preset_attract_pair(s, cx, cy, dx);
    break;
  case 3:
    preset_repel_pair(s, cx, cy, dx);
    break;
  }
}

static void scene_init(Scene *s, int cols, int rows, int preset, int theme) {
  memset(s, 0, sizeof *s);
  s->cols = cols;
  s->rows = rows;
  s->preset = preset;
  s->theme = theme;
  s->lines_per_tick = LINES_PER_TICK_DEF;

  scene_build_preset(s);
  scene_seed_lines(s);
}

/* ── Tick: reveal next batch of lines ────────────────────────────── */

static void scene_tick(Scene *s) {
  if (s->paused)
    return;
  int to_reveal = s->lines_per_tick;
  while (to_reveal-- > 0 && s->lines_traced < s->n_lines) {
    s->lines_traced++;
  }
}

/* ── Render helpers — one per visual element of the scene ────────────
 *
 * The painter's algorithm: field lines paint first (BACKGROUND), then
 * magnet bodies and N/S markers on top (FOREGROUND).  Every helper
 * treats the scene grid as LOGICAL coordinates (s->cols × s->rows)
 * covering only the renderable inset band — paint_cell() shifts row
 * by HUD_TOP_ROWS so the canvas starts at terminal row 1, and the
 * bottom HUD row is excluded implicitly via
 *     s->rows = term_rows − HUD_TOP_ROWS − HUD_BOT_ROWS.
 */

/* Paint one cell with a glyph + attribute, clipped to the scene grid. */
static void paint_cell(int row, int col, char ch, int attr, int grid_cols,
                       int grid_rows) {
  if (row < 0 || row >= grid_rows || col < 0 || col >= grid_cols)
    return;
  mvaddch(row + HUD_TOP_ROWS, col, (chtype)(unsigned char)ch | attr);
}

/* LAYER 1 — paint a single pre-rasterised streamline cell-by-cell.
 * Distance ramp colour is stored per cell at trace time, so this is
 * pure mvaddch — no math, no field_at calls. */
static void draw_field_line(const FieldLine *fl, int grid_cols, int grid_rows) {
  for (int i = 0; i < fl->len; i++) {
    int cp = fl->cp[i] ? fl->cp[i] : CP_LINE_DIM;
    paint_cell(fl->row[i], fl->col[i], fl->ch[i], COLOR_PAIR(cp), grid_cols,
               grid_rows);
  }
}

/* LAYER 1 driver — paint the REVEALED portion of the streamline pool
 * (lines[0 .. lines_traced)).  Lines past lines_traced are still being
 * "revealed" by the progressive-reveal animation and stay hidden. */
static void draw_revealed_field_lines(const Scene *s) {
  for (int li = 0; li < s->lines_traced; li++) {
    draw_field_line(&s->lines[li], s->cols, s->rows);
  }
}

/* LAYER 2a — '=' line connecting the N and S poles of one magnet.
 * Sampled at 2 points per cell of pole-to-pole distance so the line
 * stays continuous at any angle without explicit Bresenham.  A_DIM so
 * the body subdues itself behind the bright N/S markers. */
static void draw_magnet_body_line(const Dipole *dp, int grid_cols,
                                  int grid_rows) {
  float dx = dp->sx - dp->nx;
  float dy = dp->sy - dp->ny;
  float dist = sqrtf(dx * dx + dy * dy);
  if (dist <= 0.5f)
    return; /* coincident poles — skip */

  int steps = (int)(dist * 2);
  int attr = COLOR_PAIR(CP_BODY) | A_DIM;
  for (int k = 0; k <= steps; k++) {
    float t = (float)k / (float)steps; /* parametric in [0,1]    */
    int c = (int)(dp->nx + t * dx + 0.5f);
    int r = (int)(dp->ny + t * dy + 0.5f);
    paint_cell(r, c, '=', attr, grid_cols, grid_rows);
  }
}

/* LAYER 2b — bold 'N' / 'S' marker.  These sit ON TOP of the body line
 * (painter's algorithm) so the pole identity always wins legibility. */
static void draw_pole_marker(float px, float py, char glyph, int cp,
                             int grid_cols, int grid_rows) {
  int r = (int)(py + 0.5f);
  int c = (int)(px + 0.5f);
  paint_cell(r, c, glyph, COLOR_PAIR(cp) | A_BOLD, grid_cols, grid_rows);
}

/* LAYER 2 driver — one full magnet (body line + both pole markers). */
static void draw_magnet(const Dipole *dp, int grid_cols, int grid_rows) {
  draw_magnet_body_line(dp, grid_cols, grid_rows);
  draw_pole_marker(dp->nx, dp->ny, 'N', CP_NORTH, grid_cols, grid_rows);
  draw_pole_marker(dp->sx, dp->sy, 'S', CP_SOUTH, grid_cols, grid_rows);
}

/*
 * scene_draw — painter's-algorithm composite of the magnetostatic scene.
 *
 * Pseudocode:
 *     LAYER 1  draw revealed field lines (background)
 *     LAYER 2  draw each magnet on top   (foreground)
 *
 * Coordinate convention:  scene grid is INSET — every paint helper
 * shifts row by HUD_TOP_ROWS via paint_cell() so the canvas starts at
 * terminal row 1.  The bottom HUD row is excluded by sizing:
 *     s->rows = term_rows − HUD_TOP_ROWS − HUD_BOT_ROWS.
 */
static void scene_draw(const Scene *s) {
  draw_revealed_field_lines(s);

  for (int i = 0; i < s->nd; i++) {
    draw_magnet(&s->dp[i], s->cols, s->rows);
  }
}

/* ===================================================================== */
/* §6  screen / HUD                                                       */
/* ===================================================================== */

static void screen_init(void) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
}

/*
 * screen_draw_hud — canonical CLAUDE.md two-row HUD.
 *
 *   Row 0          : STATUS — title, preset, theme, trace speed,
 *                    progress %, fps, paused state.  CP_HUD (bright
 *                    yellow 226) + A_BOLD.
 *   Row term_rows-1: ACTION keys — q / r / n / N / t / T / ] / [ /
 *                    p.  CP_HINT (bright cyan 51) + A_BOLD.  NEVER
 *                    A_DIM (CLAUDE.md: dim text vanishes against any
 *                    animated palette).
 *
 * Takes terminal dims (cols, term_rows) explicitly — the scene's own
 * rows count is the INSET render band (term_rows − 2), not the full
 * terminal height, so passing term_rows separately keeps the HUD at
 * the screen edges regardless of scene sizing.
 */
static void screen_draw_hud(const Scene *s, int cols, int term_rows, int fps) {
  static const char *preset_names[N_PRESETS] = {"Dipole", "Quadrupole",
                                                "Attract", "Repel"};
  static const char *theme_names[N_THEMES] = {"Electric", "Plasma", "Fire",
                                              "Ocean", "Matrix"};

  int pct = s->n_lines > 0 ? s->lines_traced * 100 / s->n_lines : 100;

  /* ── Top row 0: STATUS (canonical yellow + A_BOLD) ─────────────── */
  move(0, 0);
  clrtoeol();
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvprintw(0, 0,
           " MagField  preset:%-10s  theme:%-8s  speed:%d"
           "  progress:%3d%%  %2dfps  %s",
           preset_names[s->preset], theme_names[s->theme], s->lines_per_tick,
           pct, fps, s->paused ? "PAUSED " : "running");
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* ── Bottom row term_rows-1: ACTION keys (canonical cyan + BOLD) ─ */
  int bot = term_rows - 1;
  move(bot, 0);
  clrtoeol();
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(bot, 0,
           " q:quit  r:reset  n/N:preset  t/T:theme  [/]:speed"
           "  p|spc:pause ");
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);

  (void)cols; /* clrtoeol takes the row; cols reserved for future */
}

/* ===================================================================== */
/* §7  signal handling                                                    */
/* ===================================================================== */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit = 0;

static void handle_sigwinch(int s) {
  (void)s;
  g_resize = 1;
}
static void handle_sigterm(int s) {
  (void)s;
  g_quit = 1;
}

/* ===================================================================== */
/* §8  main loop                                                          */
/* ===================================================================== */

/* ── Bootstrap helpers ──────────────────────────────────────────────── */

/* Wire SIGINT / SIGTERM → graceful quit and SIGWINCH → deferred resize.
 * The handlers themselves only flip volatile flags — all real work
 * happens synchronously from the main loop. */
static void install_signal_handlers(void) {
  signal(SIGWINCH, handle_sigwinch);
  signal(SIGTERM, handle_sigterm);
  signal(SIGINT, handle_sigterm);
}

/* Build (or REBUILD) the scene for the current terminal size.  Hides
 * the HUD inset arithmetic (`rows − HUD_TOP_ROWS − HUD_BOT_ROWS`) at
 * the single call site that needs to know it. */
static void rebuild_scene_for_terminal(Scene *scene, int term_rows,
                                       int term_cols, int preset, int theme) {
  scene_init(scene, term_cols, term_rows - HUD_TOP_ROWS - HUD_BOT_ROWS, preset,
             theme);
}

/* Modular cyclic step (+1 / −1 with wrap-around).  Named so the keybind
 * cases read as "advance preset by +1, wrap mod N_PRESETS" instead of
 * the slightly cryptic `(x + N - 1) % N` reverse-wrap trick. */
static void cycle_index(int *value, int step, int n) {
  *value = (*value + n + step) % n;
}

/* ── Per-frame stage helpers — each one is one phase of the sim loop ── */

/* RESIZE handling — SIGWINCH set the flag asynchronously; we drain it
 * here, ask ncurses for fresh terminal dimensions, and rebuild the
 * scene to match (line geometry depends on grid size). */
static void consume_resize_event(Scene *scene, int *rows, int *cols, int preset,
                                 int theme) {
  if (!g_resize)
    return;
  g_resize = 0;
  endwin();
  refresh();
  getmaxyx(stdscr, *rows, *cols);
  rebuild_scene_for_terminal(scene, *rows, *cols, preset, theme);
}

/* INPUT — drain every queued key and dispatch.  Mutates scene state +
 * the three "session" knobs (preset, theme, sim_fps).  Sets g_quit on
 * q/ESC.  Keeps `rows`/`cols` as plain values since they're only read. */
static void process_input(Scene *scene, int *cur_preset, int *cur_theme,
                          int *sim_fps, int rows, int cols) {
  int ch;
  while ((ch = getch()) != ERR) {
    switch (ch) {
    case 'q':
    case 27: /* 27 = ESC */
      g_quit = 1;
      break;
    case 'r': /* reset current preset */
      rebuild_scene_for_terminal(scene, rows, cols, *cur_preset, *cur_theme);
      break;
    case 'n': /* next preset */
      cycle_index(cur_preset, +1, N_PRESETS);
      rebuild_scene_for_terminal(scene, rows, cols, *cur_preset, *cur_theme);
      break;
    case 'N': /* previous preset */
      cycle_index(cur_preset, -1, N_PRESETS);
      rebuild_scene_for_terminal(scene, rows, cols, *cur_preset, *cur_theme);
      break;
    case 't': /* next theme (no rebuild) */
      cycle_index(cur_theme, +1, N_THEMES);
      scene->theme = *cur_theme;
      theme_apply(*cur_theme);
      break;
    case 'T': /* previous theme */
      cycle_index(cur_theme, -1, N_THEMES);
      scene->theme = *cur_theme;
      theme_apply(*cur_theme);
      break;
    case ']': /* reveal faster */
      if (scene->lines_per_tick < LINES_PER_TICK_MAX)
        scene->lines_per_tick++;
      break;
    case '[': /* reveal slower */
      if (scene->lines_per_tick > LINES_PER_TICK_MIN)
        scene->lines_per_tick--;
      break;
    case 'p':
    case ' ': /* pause toggle */
      scene->paused = !scene->paused;
      break;
    case '+':
    case '=': /* sim Hz up */
      if (*sim_fps < SIM_FPS_MAX)
        *sim_fps += SIM_FPS_STEP;
      break;
    case '-': /* sim Hz down */
      if (*sim_fps > SIM_FPS_MIN)
        *sim_fps -= SIM_FPS_STEP;
      break;
    }
  }
}

/* RENDER one frame: clear → paint scene → paint HUD → flush.
 * Uses `wnoutrefresh + doupdate` (single diff write) instead of
 * plain `refresh()` to avoid flicker on slow terminals — CLAUDE.md
 * ncurses Bug Table. */
static void render_frame(const Scene *scene, int rows, int cols, int fps_disp) {
  erase();
  scene_draw(scene);
  screen_draw_hud(scene, cols, rows, fps_disp);
  wnoutrefresh(stdscr);
  doupdate();
}

/* FRAME PACING — fixed-timestep cap to sim_fps.  Sleeps the leftover of
 * (tick budget − time already spent on this iteration), then re-reads
 * the clock so `t_last` is the start of the NEXT iteration's budget.
 * Writes the measured frame-work time via out-param so the FPS counter
 * can use the same measurement without re-reading the clock. */
static int64_t pace_frame_to_fps(int64_t t_last, int sim_fps,
                                 int64_t *out_work_ns) {
  int64_t t_now = clock_ns();
  int64_t t_work = t_now - t_last;
  int64_t t_tick = TICK_NS(sim_fps);
  clock_sleep_ns(t_tick - t_work); /* clamped to 0 inside sleep */
  *out_work_ns = t_work;
  return clock_ns();
}

/* FPS COUNTER — accumulate per-frame elapsed time; every half second,
 * convert (frames in window × 2) into a displayed fps value.  Uses the
 * fact that `work + max(0, tick − work) = max(work, tick)` to count
 * either a budgeted or a busted frame correctly without a branch on
 * the displayed value. */
static void update_fps_counter(int64_t work_ns, int sim_fps, int64_t *fps_acc,
                               int *fps_cnt, int *fps_disp) {
  int64_t t_tick = TICK_NS(sim_fps);
  int64_t slack = t_tick - work_ns;
  *fps_acc += work_ns + (slack > 0 ? slack : 0);
  (*fps_cnt)++;
  if (*fps_acc >= NS_PER_SEC / 2) {
    *fps_disp = *fps_cnt * 2; /* half-second window → ×2 */
    *fps_acc = 0;
    *fps_cnt = 0;
  }
}

/* AUTO-ADVANCE — once all field lines for the current preset are fully
 * revealed and the user hasn't paused, hold the finished view for
 * ~3 seconds then cycle to the next preset.  Gives the demo a
 * self-running showreel feel; user input can override at any time
 * (process_input mutates cur_preset, which `hold_ticks` doesn't care
 * about — the wraparound stays consistent). */
static void auto_advance_preset_if_complete(Scene *scene, int *cur_preset,
                                            int cur_theme, int sim_fps,
                                            int rows, int cols) {
  if (!(scene->lines_traced >= scene->n_lines && !scene->paused))
    return;

  static int hold_ticks = 0;
  hold_ticks++;
  if (hold_ticks >= sim_fps * 3) {
    hold_ticks = 0;
    cycle_index(cur_preset, +1, N_PRESETS);
    rebuild_scene_for_terminal(scene, rows, cols, *cur_preset, cur_theme);
  }
}

/*
 * main — standard fixed-timestep sim loop.
 *
 * Pseudocode:
 *     install signals; init terminal; init colours
 *     build initial scene at current terminal size
 *     loop until quit:
 *         drain SIGWINCH      (rebuild scene if resized)
 *         drain keyboard      (mutate scene / preset / theme / fps)
 *         tick                (reveal next batch of lines)
 *         render              (scene + HUD)
 *         pace                (sleep to sim_fps deadline)
 *         update fps counter
 *         auto-advance        (cycle preset once current one is done)
 *     teardown terminal
 */
int main(void) {
  install_signal_handlers();
  screen_init();
  color_init();

  int rows, cols;
  getmaxyx(stdscr, rows, cols);

  int sim_fps = SIM_FPS_DEFAULT;
  int cur_preset = 0;
  int cur_theme = 0;

  Scene scene;
  rebuild_scene_for_terminal(&scene, rows, cols, cur_preset, cur_theme);

  int64_t t_last = clock_ns();
  int64_t fps_acc = 0;
  int fps_cnt = 0;
  int fps_disp = 0;

  while (!g_quit) {
    consume_resize_event(&scene, &rows, &cols, cur_preset, cur_theme);
    process_input(&scene, &cur_preset, &cur_theme, &sim_fps, rows, cols);
    scene_tick(&scene);
    render_frame(&scene, rows, cols, fps_disp);

    int64_t work_ns;
    t_last = pace_frame_to_fps(t_last, sim_fps, &work_ns);
    update_fps_counter(work_ns, sim_fps, &fps_acc, &fps_cnt, &fps_disp);

    auto_advance_preset_if_complete(&scene, &cur_preset, cur_theme, sim_fps,
                                    rows, cols);
  }

  endwin();
  return 0;
}
