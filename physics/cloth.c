/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cloth.c — Spring-Mass Cloth Simulation
 *
 * Grid of point masses connected by structural, shear, and bend springs.
 * Integrated with symplectic Euler; spring forces computed explicitly
 * (Hooke's law + relative-velocity damping).  Ten presets vary pin
 * topology and wind mode (sin / const / gust / none).  Edges are
 * coloured by strain so the fabric reads as a live tension map; 14
 * themes cycle with t/T.
 *
 * Framework: follows framework.c §1–§8 skeleton.
 *
 * PHYSICS SUMMARY
 * ─────────────────────────────────────────────────────────────────────
 * Explicit spring force between nodes a, b:
 *   d = b.pos − a.pos
 *   dist = |d|
 *   stretch = dist − rest_len
 *   v_rel = dot(b.vel − a.vel, d̂)          (relative velocity along spring)
 *   F_mag = k * stretch + kd * v_rel
 *   F = F_mag * d̂                            (force on a; −F on b)
 *
 * Symplectic Euler integration (per node):
 *   vel += accel * dt
 *   vel *= DAMP                              (global velocity damping)
 *   pos += vel * dt
 *
 * Spring types:
 *   structural — adjacent nodes (H and V)
 *   shear      — diagonal neighbours
 *   bend       — next-nearest (skip 1 node)
 *
 * Physics in pixel space; only scene_draw calls px_to_cell.
 *
 * Cell spacing:
 *   REST_H = CELL_W × NODE_GAP   (horizontal spacing in pixels)
 *   REST_V = CELL_H × NODE_GAP   (vertical spacing in pixels)
 *
 * Ten presets (preset 0 boots first):
 *   0  Hanging Cloth  — full top row pinned, sin wind
 *   1  Flag           — left column pinned, constant rightward wind
 *   2  Hammock        — top corners only, sin gusts
 *   3  Curtain        — sparse top pins (every 4 cols), pleats
 *   4  Banner         — single top-left pin, strong constant wind
 *   5  Tapestry       — top + bottom rows pinned, gusty middle
 *   6  Tablecloth     — single centre-top pin, radial drape
 *   7  Sail           — 3 corners pinned (TL, TR, BR), constant leftward wind
 *   8  Trampoline     — all 4 corners pinned, wind off — gravity dish
 *   9  Sash           — diagonal corners (TL + BR), twisted drape
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   r         restart current preset
 *   n / p     next / previous preset
 *   t / T     next / previous theme
 *   w         toggle wind on / off
 *   + / -     wind strength up / down (10 px/s² per press, range 0..150)
 *   ] / [     sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra cloth.c -o cloth -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Explicit spring-mass simulation with symplectic Euler.
 *                  "Symplectic" means velocity is updated before position:
 *                    vel_new = vel + F/m · dt  (update velocity first)
 *                    pos_new = pos + vel_new · dt  (use new velocity)
 *                  This preserves the Hamiltonian structure — no long-term
 *                  energy drift (unlike pure explicit Euler where energy
 *                  grows unboundedly without damping).
 *
 * Physics        : Hooke's law spring force + relative-velocity damping.
 *                  Three spring types build in the mechanical response:
 *                  - Structural: resists stretching/compression (strong)
 *                  - Shear:      resists diagonal deformation (medium)
 *                  - Bend:       resists out-of-plane bending (weak)
 *                  Without bend springs, cloth wrinkles freely; with strong
 *                  bend springs it becomes stiff fabric.
 *
 * Stability      : Symplectic Euler stability bound: k·dt² < 2.
 *                  At sub-step dt ≈ 2 ms, max k ≈ 462000 px/s² — all
 *                  spring constants are far below this limit.
 *
 * Wind models    : Five WindModes — sin, const_R, const_L, gust, none.
 *                  All collapse to a single scalar wind_dir applied
 *                  uniformly across the cloth (wind_dir_from_mode), then
 *                  scaled by wind_strength and a y-amplitude curve
 *                  (scalar_wind_force).  The y-curve makes the bottom
 *                  of the cloth bilge harder than the pinned region.
 *
 * Rendering      : Strain-based colour — every horizontal / vertical edge
 *                  takes a colour tier (CP_ROW_0..3) chosen by its current
 *                  stretch ratio (strain_tier).  Each theme's ramp goes
 *                  cool/dim → hot/bright, so taut springs always glow
 *                  brighter and the cloth reads as a live tension map:
 *                  load travels through the fabric as visible waves of
 *                  colour.  Segments are rasterised by DDA (Bresenham-
 *                  style); glyph picked from slope (-, |, /, \).
 *                  Free-node markers are intentionally NOT drawn — only
 *                  pinned anchors render — so the eye reads "fabric"
 *                  instead of "graph paper grid".
 *
 * Performance    : SUB_STEPS=8 divides each frame into 8 mini-steps.
 *                  Cost: O(N_SPRINGS) per sub-step = O(CLOTH_W×CLOTH_H×6)
 *                  per frame.  At 30×18 nodes ≈ 3240 spring force evals/frame.
 *
 * References     :
 *   • Provot, X. (1995) "Deformation Constraints in a Mass-Spring Model
 *     to Describe Rigid Cloth Behaviour", Graphics Interface '95.
 *     — The foundational mass-spring cloth paper.  Introduces the exact
 *       structural / shear / bend spring triad used here.  Start here.
 *
 *   • Baraff, D. & Witkin, A. (1998) "Large Steps in Cloth Simulation",
 *     SIGGRAPH '98.
 *     — The canonical cloth paper.  We use symplectic Euler instead of
 *       their implicit method, but the framework, terminology, and
 *       trade-offs are everyone's starting vocabulary.
 *
 *   • Witkin, A. & Baraff, D. (1997) "Physically Based Modeling:
 *     Principles and Practice", SIGGRAPH Course Notes.
 *     — The clearest pedagogical introduction to particle systems,
 *       constraints, and integration.  Read alongside Baraff-Witkin '98.
 *
 *   • Müller, M., Heidelberger, B., Hennix, M., Ratcliff, J. (2007)
 *     "Position Based Dynamics", J. Visual Communication & Image
 *     Representation, 18(2), 109-118.
 *     — Alternative formulation that bypasses forces entirely.  Useful
 *       for understanding why force-based integration is the harder
 *       road, and to cross-reference physics/chain.c which uses PBD.
 *
 *   • Hairer, E., Lubich, C. & Wanner, G. (2006) "Geometric Numerical
 *     Integration: Structure-Preserving Algorithms for ODEs", Springer.
 *     — Why symplectic / semi-implicit Euler preserves energy.  The
 *       reference for understanding the integrator we picked.
 *
 *   • Bresenham, J. E. (1965) "Algorithm for computer control of a
 *     digital plotter", IBM Systems Journal, 4(1), 25-30.
 *     — Original digital line drawing.  The DDA segment rasteriser in
 *       draw_segment is a descendant.
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

enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,
  FPS_UPDATE_MS = 500,

  CLOTH_W = 30,  /* nodes horizontally                        */
  CLOTH_H = 18,  /* nodes vertically                          */
  NODE_GAP = 2,  /* terminal cells between adjacent nodes     */
  SUB_STEPS = 8, /* physics sub-steps per sim tick            */
  N_PRESETS = 10,
  N_THEMES = 14, /* MATRIX..ECLIPSE + AMBER..VIOLET unicolor  */

  WIND_MIN = 0,   /* px/s² — wind force cap, lower bound       */
  WIND_MAX = 150, /* px/s² — wind force cap, upper bound       */
  WIND_STEP = 10, /* px/s² — increment per +/- key press       */
};

#define CLOTH_N (CLOTH_W * CLOTH_H)

/* Spring counts (upper bounds) */
#define MAX_SPRINGS (CLOTH_W * CLOTH_H * 6)

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* Pixel cell dimensions */
#define CELL_W 8
#define CELL_H 16

/* Node pixel spacing */
#define REST_H (CELL_W * NODE_GAP) /* horizontal rest length (px) */
#define REST_V (CELL_H * NODE_GAP) /* vertical rest length (px)   */

/* GRAVITY: downward acceleration in pixel space (px/s²).
 * Lower than chain.c (380) because cloth has many nodes resisting the
 * fall — too high a value causes the top-row springs to overshoot
 * violently before damping.  200 gives natural drape.                     */
#define GRAVITY 200.0f

/* WIND_FREQ: sinusoidal wind oscillation rate (Hz).
 * 0.40 Hz ≈ 2.5 s per cycle — slow enough to produce cloth flutter
 * rather than rapid shaking.  Combine with preset wind strengths for
 * different feel per preset.                                               */
#define WIND_FREQ 0.40f

/*
 * DAMP — velocity retention per sub-step.
 * SUB_STEPS=8 at 60 Hz → 480 sub-steps/s.
 * 0.9993^480 ≈ 0.718 → cloth settles in a few seconds.
 */
#define DAMP 0.9993f /* velocity retention per sub-step          */

/*
 * Spring stiffness and damping per type.
 * k  (px/s² per px) — restoring force per unit extension
 * kd (s⁻¹)          — damping proportional to relative velocity along spring
 *
 * Stability check (symplectic Euler):  k * dt² < 2
 * dt = 1/(60*8) ≈ 0.00208 s → k < 2/dt² ≈ 462 000.  All values well inside.
 */
#define K_STRUCT 400.0f /* structural spring stiffness                 */
#define K_SHEAR 100.0f  /* shear spring stiffness                      */
#define K_BEND 40.0f    /* bend spring stiffness                       */
#define KD_STRUCT 4.0f  /* structural spring damping                   */
#define KD_SHEAR 2.0f   /* shear spring damping                        */
#define KD_BEND 0.5f    /* bend spring damping                         */

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
/* §3  color — themes + HUD chrome                                        */
/* ===================================================================== */

/*
 * Color pair layout:
 *   1..4  CP_ROW_0..3 — cloth row gradient, top → bottom
 *   5     CP_PIN      — pinned anchor nodes (bright highlight)
 *   6     CP_HUD      — top status bar (bright yellow + bold)
 *   7     CP_HINT     — bottom action bar (bright cyan + bold)
 *
 * CP_HUD / CP_HINT are NOT theme-driven — fixed across all palettes so
 * the HUD stays legible against any cloth color scheme.
 */
enum {
  CP_ROW_0 = 1,
  CP_ROW_1 = 2,
  CP_ROW_2 = 3,
  CP_ROW_3 = 4,
  CP_PIN = 5,
  CP_HUD = 6,
  CP_HINT = 7,
};

/*
 * Theme — four-tier row ramp (top→bottom) plus pin highlight.
 *
 * ramp[0] colours the top row of the cloth; ramp[3] colours the bottom.
 * Direction is theme-dependent: cool→warm (Matrix, Fire) or dark→bright
 * (Mono, Amber).  Each theme keeps a consistent visual gradient.
 *
 * Brightness safety (CLAUDE.md): every entry sits at index ≥ 30, or
 * 24-29 / 240-243 only as the lowest ramp tier.  16-23 / 232-239 are
 * forbidden — they vanish on default-background terminals.
 *
 * The last four entries (Amber, Crimson, Azure, Violet) are *unicolor*:
 * a single hue ramped through four luminance steps.  These give the
 * cloth a classic phosphor-CRT feel without any rainbow.
 */
typedef struct {
  const char *name;
  short ramp[4];
  short pin;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*  name        ramp[0]  ramp[1]  ramp[2]  ramp[3]   pin                */
    {"Matrix", {28, 34, 40, 46}, 82},       /* cyber green   */
    {"Fire", {130, 208, 202, 196}, 226},    /* warm → red    */
    {"Oceanic", {24, 31, 39, 51}, 195},     /* teal → cyan   */
    {"Neon", {129, 165, 201, 213}, 51},     /* purple → pink */
    {"Mono", {240, 247, 250, 255}, 231},    /* grayscale     */
    {"Ice", {153, 117, 159, 195}, 231},     /* light blues   */
    {"Nova", {129, 141, 177, 213}, 226},    /* stellar       */
    {"Forest", {58, 100, 142, 190}, 226},   /* leaves/bark   */
    {"Desert", {130, 178, 214, 220}, 226},  /* sand/gold     */
    {"Eclipse", {240, 244, 124, 196}, 226}, /* gray + red    */
    /* ── unicolor themes (single hue, four luminance tiers) ─────────────── */
    {"Amber", {130, 172, 214, 220}, 226},  /* CRT amber     */
    {"Crimson", {88, 124, 160, 196}, 231}, /* deep red      */
    {"Azure", {25, 32, 39, 45}, 195},      /* solid blue    */
    {"Violet", {54, 91, 129, 165}, 213},   /* solid purple  */
};

static void theme_apply(int t) {
  const Theme *th = &k_themes[t % N_THEMES];
  if (COLORS >= 256) {
    init_pair(CP_ROW_0, th->ramp[0], -1);
    init_pair(CP_ROW_1, th->ramp[1], -1);
    init_pair(CP_ROW_2, th->ramp[2], -1);
    init_pair(CP_ROW_3, th->ramp[3], -1);
    init_pair(CP_PIN, th->pin, -1);
    /* Fixed chrome — same on every theme so the HUD stays legible. */
    init_pair(CP_HUD, 226, -1); /* bright yellow */
    init_pair(CP_HINT, 51, -1); /* bright cyan   */
  } else {
    /* 8-colour fallback — theme-independent. */
    init_pair(CP_ROW_0, COLOR_CYAN, -1);
    init_pair(CP_ROW_1, COLOR_GREEN, -1);
    init_pair(CP_ROW_2, COLOR_YELLOW, -1);
    init_pair(CP_ROW_3, COLOR_RED, -1);
    init_pair(CP_PIN, COLOR_WHITE, -1);
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  theme_apply(0);
}

/* ===================================================================== */
/* §4  coords — pixel ↔ cell                                              */
/* ===================================================================== */

static inline int px_to_cell_x(float px) {
  return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py) {
  return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  entity — Node, Spring, Cloth                                       */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────
 * Node — one point mass in the cloth grid.
 *
 * The cloth is a Lagrangian particle system: the continuous fabric is
 * sampled at a fixed CLOTH_W × CLOTH_H grid of points, and each Node is
 * one such sample.  Between nodes, springs (next struct) interpolate
 * the elastic response — there is no continuum field, no FEM mesh.
 *
 * Each Node carries three SEMANTICALLY DIFFERENT groups of state.  They
 * are grouped here by *access pattern* (who reads, who writes, when),
 * because that's what governs cache locality and correctness of the
 * lerp-on-render trick:
 *
 *   simulation state    : x, y, vx, vy
 *       Read AND written every physics sub-step in cloth_step.  Pixel
 *       coordinates (origin top-left, +y = down to match ncurses).
 *
 *   render snapshot     : rx, ry
 *       Written ONCE per physics tick (in cloth_tick, before the sub-
 *       step loop).  Read every render frame by cloth_draw, which lerps
 *       between (rx,ry) and (x,y) with alpha = sim_accum / TICK_NS to
 *       hide sub-tick discreteness.  Without rx/ry the render snaps to
 *       the latest sub-step and you can see the 8-step granularity.
 *
 *   topology            : pinned
 *       Set ONCE at preset init, never changed during a run.  A pinned
 *       node is a Dirichlet boundary condition — cloth_step skips
 *       integration for it entirely, so the preset's chosen pin pattern
 *       *is* the cloth's boundary condition.
 *
 * Reference: Provot 1995 §2 (mass-spring discretisation of cloth)
 *           Baraff & Witkin 1998 §3 (continuum → particle system)
 * ───────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ─ simulation state (hot path — read+write every sub-step) ─ */
  float x, y;   /* position in pixel space, +y = down              */
  float vx, vy; /* velocity in px/s                                */

  /* ─ render interpolation snapshot (written 1×/tick, read N×/frame) ─ */
  float rx, ry; /* position at start of the latest physics tick.
                   cloth_draw lerps (rx,ry)→(x,y) with sub-tick
                   alpha so render motion stays smooth between
                   SUB_STEPS=8 physics updates per tick.            */

  /* ─ topology (write 1× at preset init, read every sub-step) ─ */
  bool pinned; /* true → Dirichlet BC: this node never integrates,
                  its (x,y) stay at whatever the preset set them
                  to.  The set of pinned nodes IS the cloth's
                  boundary condition.                              */
} Node;

/* ─────────────────────────────────────────────────────────────────────
 * Spring — one Hookean elastic connection with linear damping.
 *
 * Models a damped linear spring between two Nodes a and b.  Per
 * cloth_step:
 *
 *     d   = b.pos − a.pos              (vector along the spring)
 *     d̂   = d / |d|                    (unit axis, undefined if |d|=0)
 *     v_rel = (b.vel − a.vel) · d̂       (signed: + = stretching)
 *
 *     F_mag = k · (|d| − rest)         ← Hookean restoring force
 *           + kd · v_rel               ← relative-velocity damping
 *
 *     F_on_a =  F_mag · d̂   (a is pulled toward b when stretched)
 *     F_on_b = −F_mag · d̂   (Newton's 3rd: equal & opposite)
 *
 * The cloth's three "spring kinds" — structural / shear / bend — are
 * NOT a type field.  They are three POPULATIONS of Spring instances
 * created at cloth_build_springs time with different connectivity and
 * different (k, kd, rest) values:
 *
 *   structural  : grid 4-neighbours (col±1 or row±1)   K=400  KD=4
 *   shear       : grid diagonals    (col±1 AND row±1)  K=100  KD=2
 *   bend        : skip-one along col or row            K= 40  KD=0.5
 *
 * Stiffness drops by ≈10× between tiers (Provot 1995 §3 rationale):
 * structural resists stretching, shear resists in-plane skew, bend
 * resists wrinkling.  Drop bend → cloth wrinkles freely; raise bend →
 * stiff fabric / canvas.
 *
 * Reference: Provot 1995 §3 (structural / shear / bend triad)
 *           Hooke 1660s (F = −kx, the underlying constitutive law)
 *           Baraff & Witkin 1998 §4 (spring force in cloth context)
 * ───────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ─ topology — undirected edge between two nodes ─ */
  int a, b; /* indices into Cloth::nodes[]; convention a < b
               is not enforced and not required.                */

  /* ─ physics — set at cloth_build_springs, constant thereafter ─ */
  float rest; /* rest length in pixels.  Zero force when |d| = rest.
                 Structural: REST_H or REST_V.
                 Shear:      sqrt(REST_H² + REST_V²).
                 Bend:       2·REST_H or 2·REST_V.               */
  float k;    /* stiffness, px/s² per pixel of stretch.  Bounded
                 above by symplectic-Euler stability:  k·dt² < 2,
                 i.e. k < 2/(1/(60·8))² ≈ 462000 — all values
                 used here are well under this.                  */
  float kd;   /* damping coefficient, s⁻¹.  Force component along
                 the spring axis proportional to v_rel.  Required
                 to bleed off oscillation energy that the pure
                 Hookean term alone would conserve forever.       */
} Spring;

/* ── Wind modes ────────────────────────────────────────────────────── */

/*
 * WindMode — per-preset wind behaviour, chosen to suit the cloth's
 * pinning topology and give each preset its own visual signature.
 *
 *   WIND_SIN       ±1 sinusoidal — cloth swings side-to-side smoothly
 *   WIND_CONST_R   constant +1   — cloth blown rightward always (flag-like)
 *   WIND_CONST_L   constant -1   — cloth blown leftward  always (sail-fill)
 *   WIND_GUST      0.6 + 0.4·sin(2φ) — mostly one direction with fast pulses
 *   WIND_NONE      0             — pure gravity, no wind force
 */
typedef enum {
  WIND_SIN = 0,
  WIND_CONST_R,
  WIND_CONST_L,
  WIND_GUST,
  WIND_NONE,
} WindMode;

/* ─────────────────────────────────────────────────────────────────────
 * Cloth — the entire cloth simulation state.
 *
 * One Cloth IS the simulation.  No global mutable state; cloth_step,
 * cloth_tick, cloth_draw all take a Cloth* and do their work on it.
 * That lets the renderer take a const Cloth* even while the physics
 * mutates its own pointer — the boundary between sim and render is
 * exactly one function call deep.
 *
 * Storage is entirely static (per the project's "no malloc in hot path"
 * rule from CLAUDE.md): nodes[] and springs[] are fixed-size arrays
 * sized at compile time from CLOTH_N and MAX_SPRINGS.  Worst-case
 * MAX_SPRINGS = 6·CLOTH_W·CLOTH_H accommodates structural + shear +
 * bend density at full coverage.
 *
 * Members are grouped by what TOUCHES them, not by type — that's the
 * locality grouping CLAUDE.md asks for:
 *
 *   pools (body)        : the cloth itself — nodes + springs.
 *                         Read every sub-step, written by integration.
 *
 *   wind driver state   : per-preset wind dynamics.  Advanced once per
 *                         cloth_tick (wind_phase), read every sub-step
 *                         (wind_strength, wind_on, wind_mode), set at
 *                         preset init.
 *
 *   ui state            : user-controlled run state.  Written by
 *                         app_handle_key only, read by step + render.
 *
 * Reference: Baraff & Witkin 1998 §6 (cloth as a system of particles
 *                                     + constraints — what to store)
 *           Witkin & Baraff 1997 SIGGRAPH notes §3.1 (particle system
 *                                     bookkeeping patterns)
 * ───────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ─ pools — the cloth body ──────────────────────────────────────
   * Allocated once, never resized.  nodes[] is row-major; the helper
   * node_idx(col,row) returns row·CLOTH_W + col.  springs[] is a
   * compact pool: only the first n_springs entries are live, the
   * rest are uninitialised slack (capacity MAX_SPRINGS).            */
  Node nodes[CLOTH_N];
  Spring springs[MAX_SPRINGS];
  int n_springs; /* live count in springs[], 0..MAX_SPRINGS */

  /* ─ wind driver state ───────────────────────────────────────────
   * The cloth is driven by gravity + wind.  Wind has its own little
   * clock (wind_phase, radians) that advances by WIND_FREQ·2π·dt per
   * cloth_tick, wrapping at 2π.  wind_mode picks one of five formulas
   * (sin / const_R / const_L / gust / none) that collapse wind_phase
   * into a single scalar wind_dir; that scalar is then applied
   * uniformly across the cloth with a y-amplitude curve.            */
  float wind_phase;    /* radians; clock for periodic wind motion   */
  float wind_strength; /* px/s²; force magnitude; user adjusts +/-  */
  bool wind_on;        /* user toggle (w); false zeros wind force   */
  WindMode wind_mode;  /* selects the wind_dir formula              */

  /* ─ ui state ────────────────────────────────────────────────────
   * User-controlled.  Written exclusively by app_handle_key, read
   * (but not modified) by cloth_step / cloth_tick / cloth_draw and
   * the HUD.  No render-only fields live on Cloth — render state is
   * either in Node (rx,ry) or in Scene (theme).                       */
  bool paused; /* space; true → cloth_tick skips integration       */
  int preset;  /* current preset index, 0..N_PRESETS-1; the only
                  breadcrumb that says "which scenario am I running" */
} Cloth;

/* ── Helpers ───────────────────────────────────────────────────────── */

static inline int node_idx(int col, int row) { return row * CLOTH_W + col; }

static void cloth_add_spring(Cloth *c, int a, int b, float rest, float k,
                             float kd) {
  if (c->n_springs >= MAX_SPRINGS)
    return;
  Spring *sp = &c->springs[c->n_springs++];
  sp->a = a;
  sp->b = b;
  sp->rest = rest;
  sp->k = k;
  sp->kd = kd;
}

/* ── Preset initialisation ─────────────────────────────────────────── */

/*
 * cloth_reset_positions — place nodes in their rest grid.
 *
 * The top-left node starts at pixel (ox0, oy0).
 * REST_H pixels between horizontal neighbours.
 * REST_V pixels between vertical neighbours.
 */
static void cloth_reset_positions(Cloth *c, float ox0, float oy0) {
  for (int row = 0; row < CLOTH_H; row++) {
    for (int col = 0; col < CLOTH_W; col++) {
      Node *n = &c->nodes[node_idx(col, row)];
      n->x = n->rx = ox0 + (float)col * REST_H;
      n->y = n->ry = oy0 + (float)row * REST_V;
      n->vx = n->vy = 0.0f;
      n->pinned = false;
    }
  }
}

/*
 * add_structural_springs — Provot's first population.
 * Each interior grid node gets two structural springs: one to its right
 * 4-neighbour and one to its bottom 4-neighbour.  Stiff (K_STRUCT) so
 * the cloth resists stretching/compression like real fabric warp+weft.
 */
static void add_structural_springs(Cloth *c) {
  for (int row = 0; row < CLOTH_H; row++) {
    for (int col = 0; col < CLOTH_W; col++) {
      int idx = node_idx(col, row);
      if (col + 1 < CLOTH_W)
        cloth_add_spring(c, idx, node_idx(col + 1, row), (float)REST_H,
                         K_STRUCT, KD_STRUCT);
      if (row + 1 < CLOTH_H)
        cloth_add_spring(c, idx, node_idx(col, row + 1), (float)REST_V,
                         K_STRUCT, KD_STRUCT);
    }
  }
}

/*
 * add_shear_springs — Provot's second population.
 * Each grid quad gets both diagonals (NW-SE and NE-SW), resisting in-
 * plane skew so the cloth keeps its rectangular character instead of
 * collapsing into a rhombus.  Medium stiffness (K_SHEAR).
 */
static void add_shear_springs(Cloth *c) {
  float diag = sqrtf((float)(REST_H * REST_H + REST_V * REST_V));
  for (int row = 0; row + 1 < CLOTH_H; row++) {
    for (int col = 0; col + 1 < CLOTH_W; col++) {
      cloth_add_spring(c, node_idx(col, row), node_idx(col + 1, row + 1), diag,
                       K_SHEAR, KD_SHEAR);
      cloth_add_spring(c, node_idx(col + 1, row), node_idx(col, row + 1), diag,
                       K_SHEAR, KD_SHEAR);
    }
  }
}

/*
 * add_bend_springs — Provot's third population.
 * Skip-one connections along col and row.  Weak (K_BEND) so the cloth
 * can curve naturally but resists the unphysical sharp-fold mode that
 * pure structural+shear permits.  Drop these → cloth wrinkles freely.
 */
static void add_bend_springs(Cloth *c) {
  for (int row = 0; row < CLOTH_H; row++) {
    for (int col = 0; col < CLOTH_W; col++) {
      int idx = node_idx(col, row);
      if (col + 2 < CLOTH_W)
        cloth_add_spring(c, idx, node_idx(col + 2, row), (float)(REST_H * 2),
                         K_BEND, KD_BEND);
      if (row + 2 < CLOTH_H)
        cloth_add_spring(c, idx, node_idx(col, row + 2), (float)(REST_V * 2),
                         K_BEND, KD_BEND);
    }
  }
}

/*
 * cloth_build_springs — wire up Provot 1995's three-population triad.
 * Order doesn't matter for correctness (springs accumulate forces
 * commutatively), but the ordering here mirrors Provot's paper for
 * readability.
 */
static void cloth_build_springs(Cloth *c) {
  c->n_springs = 0;
  add_structural_springs(c);
  add_shear_springs(c);
  add_bend_springs(c);
}

/*
 * Each preset is responsible for:
 *   1. Calling cloth_reset_positions(c, ox0, oy0) with a sensible origin
 *   2. Marking pinned nodes via c->nodes[...].pinned = true
 *   3. Setting wind_strength, wind_on, wind_mode (and optionally wind_phase)
 *
 * All ten preset functions share this shape so cycling through them with
 * n/p produces ten visually distinct cloth scenarios.
 */

/*
 * preset 0 — Hanging Cloth ★ default boot preset
 * Top row fully pinned; gravity drapes the body downward; gentle
 * sinusoidal wind sways the whole sheet side-to-side.
 */
static void preset_hanging(Cloth *c, int cols, int rows) {
  int cloth_px_w = (CLOTH_W - 1) * REST_H;
  float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
  float oy0 = (float)(rows * CELL_H) * 0.05f + (float)CELL_H * 2;

  cloth_reset_positions(c, ox0, oy0);
  for (int col = 0; col < CLOTH_W; col++)
    c->nodes[node_idx(col, 0)].pinned = true;

  /* Start at quarter-period so wind is at full strength immediately. */
  c->wind_phase = (float)M_PI * 0.5f;
  c->wind_strength = 30.0f;
  c->wind_on = true;
  c->wind_mode = WIND_SIN;
}

/*
 * preset 1 — Flag
 * Left column fully pinned (a flagpole); constant rightward wind keeps
 * the flag flying out to the right.
 */
static void preset_flag(Cloth *c, int cols, int rows) {
  float ox0 = (float)(CELL_W * 3);
  float oy0 = (float)(rows * CELL_H) * 0.15f;

  cloth_reset_positions(c, ox0, oy0);
  for (int row = 0; row < CLOTH_H; row++)
    c->nodes[node_idx(0, row)].pinned = true;

  c->wind_strength = 40.0f;
  c->wind_on = true;
  c->wind_mode = WIND_CONST_R;
  (void)cols;
}

/*
 * preset 2 — Hammock
 * Only the two top corners pinned; the body hangs in a deep catenary
 * curve under gravity, with sinusoidal gusts rocking it.
 */
static void preset_hammock(Cloth *c, int cols, int rows) {
  int cloth_px_w = (CLOTH_W - 1) * REST_H;
  float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
  float oy0 = (float)(rows * CELL_H) * 0.08f + (float)CELL_H * 2;

  cloth_reset_positions(c, ox0, oy0);
  c->nodes[node_idx(0, 0)].pinned = true;
  c->nodes[node_idx(CLOTH_W - 1, 0)].pinned = true;

  c->wind_strength = 50.0f;
  c->wind_on = true;
  c->wind_mode = WIND_SIN;
  (void)rows;
}

/*
 * preset 3 — Curtain
 * Top row pinned every 4 columns (plus the rightmost column), creating
 * scalloped pleats that sway between pins.  Gentle sin wind.
 */
static void preset_curtain(Cloth *c, int cols, int rows) {
  int cloth_px_w = (CLOTH_W - 1) * REST_H;
  float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
  float oy0 = (float)(rows * CELL_H) * 0.05f + (float)CELL_H * 2;

  cloth_reset_positions(c, ox0, oy0);
  for (int col = 0; col < CLOTH_W; col += 4)
    c->nodes[node_idx(col, 0)].pinned = true;
  /* Always pin the last column so the right edge is anchored too. */
  c->nodes[node_idx(CLOTH_W - 1, 0)].pinned = true;

  c->wind_phase = (float)M_PI * 0.5f;
  c->wind_strength = 25.0f;
  c->wind_on = true;
  c->wind_mode = WIND_SIN;
}

/*
 * preset 4 — Banner
 * A single pin at the top-left corner; strong constant rightward wind
 * sweeps the entire cloth out into a streaming banner.
 */
static void preset_banner(Cloth *c, int cols, int rows) {
  float ox0 = (float)(CELL_W * 3);
  float oy0 = (float)(rows * CELL_H) * 0.15f;

  cloth_reset_positions(c, ox0, oy0);
  c->nodes[node_idx(0, 0)].pinned = true;

  c->wind_strength = 60.0f;
  c->wind_on = true;
  c->wind_mode = WIND_CONST_R;
  (void)cols;
}

/*
 * preset 5 — Tapestry
 * Both top and bottom rows pinned — the cloth is stretched between two
 * horizontal rails like a wall hanging.  Gusty wind makes the middle
 * billow in and out unpredictably.
 */
static void preset_tapestry(Cloth *c, int cols, int rows) {
  int cloth_px_w = (CLOTH_W - 1) * REST_H;
  float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
  float oy0 = (float)CELL_H * 2;

  cloth_reset_positions(c, ox0, oy0);
  for (int col = 0; col < CLOTH_W; col++) {
    c->nodes[node_idx(col, 0)].pinned = true;
    c->nodes[node_idx(col, CLOTH_H - 1)].pinned = true;
  }

  c->wind_strength = 50.0f;
  c->wind_on = true;
  c->wind_mode = WIND_GUST;
  (void)rows;
}

/*
 * preset 6 — Tablecloth
 * A single pin at the centre of the top row — the cloth drapes radially
 * around one point.  Gentle sin wind sways the entire sheet as one body.
 */
static void preset_tablecloth(Cloth *c, int cols, int rows) {
  int cloth_px_w = (CLOTH_W - 1) * REST_H;
  float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
  float oy0 = (float)(rows * CELL_H) * 0.05f + (float)CELL_H * 2;

  cloth_reset_positions(c, ox0, oy0);
  c->nodes[node_idx(CLOTH_W / 2, 0)].pinned = true;

  c->wind_strength = 15.0f;
  c->wind_on = true;
  c->wind_mode = WIND_SIN;
}

/*
 * preset 7 — Sail
 * Three corners pinned (top-left, top-right, bottom-right) — a
 * triangular sail configuration.  Constant leftward wind fills the sail
 * by pushing the unconstrained bottom-left toward the bottom-right.
 */
static void preset_sail(Cloth *c, int cols, int rows) {
  int cloth_px_w = (CLOTH_W - 1) * REST_H;
  float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
  float oy0 = (float)CELL_H * 2;

  cloth_reset_positions(c, ox0, oy0);
  c->nodes[node_idx(0, 0)].pinned = true;
  c->nodes[node_idx(CLOTH_W - 1, 0)].pinned = true;
  c->nodes[node_idx(CLOTH_W - 1, CLOTH_H - 1)].pinned = true;

  c->wind_strength = 70.0f;
  c->wind_on = true;
  c->wind_mode = WIND_CONST_L;
  (void)rows;
}

/*
 * preset 8 — Trampoline
 * All four corners pinned and wind off — gravity pulls the centre into
 * a clean dish shape so you can see the springs settle.
 */
static void preset_trampoline(Cloth *c, int cols, int rows) {
  int cloth_px_w = (CLOTH_W - 1) * REST_H;
  float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
  float oy0 = (float)CELL_H * 2;

  cloth_reset_positions(c, ox0, oy0);
  c->nodes[node_idx(0, 0)].pinned = true;
  c->nodes[node_idx(CLOTH_W - 1, 0)].pinned = true;
  c->nodes[node_idx(0, CLOTH_H - 1)].pinned = true;
  c->nodes[node_idx(CLOTH_W - 1, CLOTH_H - 1)].pinned = true;

  c->wind_strength = 0.0f;
  c->wind_on = false;
  c->wind_mode = WIND_NONE;
  (void)rows;
}

/*
 * preset 9 — Sash
 * Diagonal corner pins (top-left + bottom-right) — the cloth twists
 * between them, sagging on the unconstrained diagonal.  Sin wind makes
 * the twisted form sway.
 */
static void preset_sash(Cloth *c, int cols, int rows) {
  int cloth_px_w = (CLOTH_W - 1) * REST_H;
  float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
  float oy0 = (float)CELL_H * 2;

  cloth_reset_positions(c, ox0, oy0);
  c->nodes[node_idx(0, 0)].pinned = true;
  c->nodes[node_idx(CLOTH_W - 1, CLOTH_H - 1)].pinned = true;

  c->wind_strength = 30.0f;
  c->wind_on = true;
  c->wind_mode = WIND_SIN;
  (void)rows;
}

static const char *preset_names[N_PRESETS] = {
    "Hanging Cloth", "Flag",       "Hammock", "Curtain",    "Banner",
    "Tapestry",      "Tablecloth", "Sail",    "Trampoline", "Sash",
};

static void cloth_init(Cloth *c, int preset, int cols, int rows) {
  memset(c, 0, sizeof *c);
  c->paused = false;
  c->preset = preset;
  c->wind_phase = 0.0f;

  switch (preset) {
  default:
  case 0:
    preset_hanging(c, cols, rows);
    break;
  case 1:
    preset_flag(c, cols, rows);
    break;
  case 2:
    preset_hammock(c, cols, rows);
    break;
  case 3:
    preset_curtain(c, cols, rows);
    break;
  case 4:
    preset_banner(c, cols, rows);
    break;
  case 5:
    preset_tapestry(c, cols, rows);
    break;
  case 6:
    preset_tablecloth(c, cols, rows);
    break;
  case 7:
    preset_sail(c, cols, rows);
    break;
  case 8:
    preset_trampoline(c, cols, rows);
    break;
  case 9:
    preset_sash(c, cols, rows);
    break;
  }

  cloth_build_springs(c);
}

/* ── Physics tick ──────────────────────────────────────────────────── */

/*
 * wind_dir_from_mode — collapse WindMode + wind_phase into a single
 * scalar direction in [-1, +1].  Applied uniformly across the cloth
 * in scalar_wind_force; the modes differ only in this scalar.
 */
static float wind_dir_from_mode(const Cloth *c) {
  switch (c->wind_mode) {
  case WIND_CONST_R:
    return 1.0f;
  case WIND_CONST_L:
    return -1.0f;
  case WIND_GUST:
    return 0.6f + 0.4f * sinf(c->wind_phase * 2.0f);
  case WIND_NONE:
    return 0.0f;
  case WIND_SIN:
  default:
    return sinf(c->wind_phase);
  }
}

/*
 * scalar_wind_force — uniform horizontal wind with a y-amplitude curve.
 * Returns ax in px/s².  Amplitude grows toward the bottom of the cloth
 * (0.4 + 0.6·y_frac) so the unconstrained edge bilges harder than the
 * pinned region — matches how a real hanging sheet behaves in wind.
 */
static inline float scalar_wind_force(const Cloth *c, float y_frac,
                                      float wind_dir) {
  return c->wind_strength * wind_dir * (0.4f + 0.6f * y_frac);
}

/*
 * accumulate_external_forces — body forces on every free node.
 * Body forces in continuum mechanics = forces acting per particle
 * irrespective of neighbours (here: gravity and wind).  Pinned nodes
 * get ax = ay = 0 since they don't integrate.
 */
static void accumulate_external_forces(const Cloth *c, float *ax, float *ay,
                                       float wind_dir) {
  for (int i = 0; i < CLOTH_N; i++) {
    if (c->nodes[i].pinned) {
      ax[i] = ay[i] = 0.0f;
      continue;
    }
    ax[i] = 0.0f;
    ay[i] = GRAVITY;
    if (!c->wind_on)
      continue;

    int row = i / CLOTH_W;
    float y_frac = (float)row / (float)(CLOTH_H - 1);
    ax[i] = scalar_wind_force(c, y_frac, wind_dir);
  }
}

/*
 * hooke_damped_force — restoring force on endpoint a from one spring.
 *
 *   d   = b.pos − a.pos
 *   d̂   = d / |d|
 *   F_mag = k · (|d| − rest)      ← Hookean restoring term
 *         + kd · v_rel·d̂          ← linear damping along axis
 *   F_on_a =  F_mag · d̂
 *   F_on_b = −F_mag · d̂   (Newton's 3rd)
 *
 * Degenerate case (|d| ≈ 0): force is undefined; we return zero so the
 * caller doesn't divide by zero or apply NaN.
 */
static void hooke_damped_force(const Node *na, const Node *nb, float rest,
                               float k, float kd, float *out_fx,
                               float *out_fy) {
  float dx = nb->x - na->x;
  float dy = nb->y - na->y;
  float dist = sqrtf(dx * dx + dy * dy);
  if (dist < 1e-6f) {
    *out_fx = 0.0f;
    *out_fy = 0.0f;
    return;
  }
  float inv = 1.0f / dist;
  float ex = dx * inv;
  float ey = dy * inv;
  float vrel = (nb->vx - na->vx) * ex + (nb->vy - na->vy) * ey;
  float fmag = k * (dist - rest) + kd * vrel;
  *out_fx = fmag * ex;
  *out_fy = fmag * ey;
}

/*
 * accumulate_spring_forces — internal forces.
 * Each spring contributes equal-and-opposite forces to its two endpoints
 * (Newton's 3rd).  Pinned endpoints simply don't receive the force.
 */
static void accumulate_spring_forces(const Cloth *c, float *ax, float *ay) {
  for (int s = 0; s < c->n_springs; s++) {
    const Spring *sp = &c->springs[s];
    const Node *na = &c->nodes[sp->a];
    const Node *nb = &c->nodes[sp->b];

    float fx, fy;
    hooke_damped_force(na, nb, sp->rest, sp->k, sp->kd, &fx, &fy);

    if (!na->pinned) {
      ax[sp->a] += fx;
      ay[sp->a] += fy;
    }
    if (!nb->pinned) {
      ax[sp->b] -= fx;
      ay[sp->b] -= fy;
    }
  }
}

/*
 * integrate_symplectic_euler — semi-implicit (a.k.a. symplectic) Euler.
 * Velocity is updated FIRST using current acceleration, then position is
 * updated using the NEW velocity.  Order matters: this preserves the
 * Hamiltonian structure and keeps long-term energy bounded, unlike
 * naïve explicit Euler which would blow up.  Global DAMP factor bleeds
 * energy out of every step so the cloth eventually settles.
 *
 * Reference: Hairer, Lubich & Wanner 2006 ch. VI (symplectic methods).
 */
static void integrate_symplectic_euler(Cloth *c, const float *ax,
                                       const float *ay, float dt) {
  for (int i = 0; i < CLOTH_N; i++) {
    Node *n = &c->nodes[i];
    if (n->pinned)
      continue;
    n->vx = (n->vx + ax[i] * dt) * DAMP;
    n->vy = (n->vy + ay[i] * dt) * DAMP;
    n->x += n->vx * dt;
    n->y += n->vy * dt;
  }
}

/*
 * cloth_step — one physics sub-step of the cloth.
 * Reads like the pseudocode of a force-based explicit integrator:
 *   1. resolve wind direction (one scalar from WindMode)
 *   2. body forces      (gravity + uniform wind)
 *   3. internal forces  (springs: Hooke + relative-velocity damping)
 *   4. integrate        (symplectic Euler with global DAMP)
 */
static void cloth_step(Cloth *c, float dt) {
  float ax[CLOTH_N], ay[CLOTH_N]; /* per-node acceleration accumulators */

  float wind_dir = wind_dir_from_mode(c);
  accumulate_external_forces(c, ax, ay, wind_dir);
  accumulate_spring_forces(c, ax, ay);
  integrate_symplectic_euler(c, ax, ay, dt);
}

static void cloth_tick(Cloth *c, float dt) {
  if (c->paused)
    return;

  /*
   * Snapshot node positions BEFORE physics runs.
   * cloth_draw lerps between (rx,ry) and (x,y) using the sub-tick
   * alpha so that every render frame gets a smooth intermediate
   * position rather than the hard physics-tick position.
   */
  for (int i = 0; i < CLOTH_N; i++) {
    c->nodes[i].rx = c->nodes[i].x;
    c->nodes[i].ry = c->nodes[i].y;
  }

  float sub_dt = dt / (float)SUB_STEPS;
  c->wind_phase += WIND_FREQ * 2.0f * (float)M_PI * dt;
  if (c->wind_phase > 2.0f * (float)M_PI)
    c->wind_phase -= 2.0f * (float)M_PI;

  for (int s = 0; s < SUB_STEPS; s++) {
    cloth_step(c, sub_dt);
  }
}

/* ── Drawing ───────────────────────────────────────────────────────── */

/*
 * draw_segment — DDA fill between two cell positions.
 *
 * Chooses the most appropriate ASCII char for the segment direction:
 *   nearly horizontal → '-'
 *   nearly vertical   → '|'
 *   positive slope    → '\'
 *   negative slope    → '/'
 */
static void draw_segment(WINDOW *w, int x0, int y0, int x1, int y1, int cols,
                         int rows, chtype attr) {
  int dx = x1 - x0;
  int dy = y1 - y0;
  int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);
  if (steps == 0) {
    if (x0 >= 0 && x0 < cols && y0 >= 1 && y0 < rows - 1) {
      wattron(w, attr);
      mvwaddch(w, y0, x0, '+');
      wattroff(w, attr);
    }
    return;
  }

  int adx = abs(dx), ady = abs(dy);
  char ch;
  if (adx >= 2 * ady)
    ch = '-';
  else if (ady >= 2 * adx)
    ch = '|';
  else if (dx * dy > 0)
    ch = '\\';
  else
    ch = '/';

  wattron(w, attr);
  for (int k = 0; k <= steps; k++) {
    int cx = x0 + k * dx / steps;
    int cy = y0 + k * dy / steps;
    if (cx >= 0 && cx < cols && cy >= 1 && cy < rows - 1)
      mvwaddch(w, cy, cx, ch);
  }
  wattroff(w, attr);
}

/*
 * node_lerp_cell — lerp a node's pixel position and convert to cell.
 *
 * alpha ∈ [0,1) is the sub-tick interpolation factor:
 *   alpha = sim_accum / tick_ns
 * Lerps between snapshot (rx,ry) and physics position (x,y).
 */
static inline void node_lerp_cell(const Node *n, float alpha, int *out_cx,
                                  int *out_cy) {
  float draw_x = n->rx + (n->x - n->rx) * alpha;
  float draw_y = n->ry + (n->y - n->ry) * alpha;
  *out_cx = px_to_cell_x(draw_x);
  *out_cy = px_to_cell_y(draw_y);
}

/*
 * strain_tier — map a spring's stretch ratio to one of the four ramp tiers.
 *
 * Each theme's ramp is calibrated dim/cool → bright/hot, so taut springs
 * always glow brighter than slack ones regardless of palette.  This is
 * what makes the cloth feel alive: as gravity, wind, and swinging push
 * tension through the fabric, you see waves of colour travel across it.
 *
 *   strain < -2 %    compressed / sagging         → CP_ROW_0 (cool/dim)
 *   strain < +2 %    near rest length             → CP_ROW_1
 *   strain < +8 %    stretched, carrying load     → CP_ROW_2
 *   strain ≥ +8 %    very taut (load-bearing)     → CP_ROW_3 (hot/bright)
 *
 * Vertical structural springs near the top of a hanging cloth carry the
 * weight of everything below them and sit firmly in ROW_2/ROW_3.
 * Horizontal springs in folded regions sit in ROW_0/ROW_1.
 */
static inline int strain_tier(const Node *a, const Node *b, float rest) {
  float dx = b->x - a->x;
  float dy = b->y - a->y;
  float dist = sqrtf(dx * dx + dy * dy);
  float s = (dist - rest) / rest;
  if (s < -0.02f)
    return CP_ROW_0;
  else if (s < 0.02f)
    return CP_ROW_1;
  else if (s < 0.08f)
    return CP_ROW_2;
  else
    return CP_ROW_3;
}

/*
 * draw_strained_edge — render one inter-node edge with strain-tier colour.
 * Compute the spring's current stretch ratio, pick the matching CP_ROW_*
 * tier, lerp both endpoints to cell coordinates, draw a DDA segment.
 */
static void draw_strained_edge(WINDOW *w, const Node *na, const Node *nb,
                               float rest, float alpha, int cols, int rows) {
  int ax, ay, bx, by;
  node_lerp_cell(na, alpha, &ax, &ay);
  node_lerp_cell(nb, alpha, &bx, &by);
  int tier = strain_tier(na, nb, rest);
  draw_segment(w, ax, ay, bx, by, cols, rows, (chtype)COLOR_PAIR(tier));
}

/*
 * draw_strain_weave — the cloth body as a strain-coloured wireframe.
 *
 * For every interior node, draw the structural edge to its right
 * neighbour and the structural edge to its bottom neighbour.  Each
 * edge's colour is driven by its current stretch — so the mesh reads
 * as a live tension map: load-carrying threads glow hot, slack ones
 * sit cool.  When the cloth swings, waves of colour travel through it.
 */
static void draw_strain_weave(const Cloth *c, WINDOW *w, int cols, int rows,
                              float alpha) {
  for (int row = 0; row < CLOTH_H; row++) {
    for (int col = 0; col < CLOTH_W; col++) {
      const Node *n = &c->nodes[node_idx(col, row)];
      if (col + 1 < CLOTH_W) {
        const Node *nr = &c->nodes[node_idx(col + 1, row)];
        draw_strained_edge(w, n, nr, (float)REST_H, alpha, cols, rows);
      }
      if (row + 1 < CLOTH_H) {
        const Node *nd = &c->nodes[node_idx(col, row + 1)];
        draw_strained_edge(w, n, nd, (float)REST_V, alpha, cols, rows);
      }
    }
  }
}

/*
 * draw_pinned_anchors — overlay the cloth's boundary conditions.
 * Pinned nodes are Dirichlet BCs in the simulation; visually they read
 * as the points that hold the cloth up.  Drawn last so they paint over
 * any edge that happens to cross them.  Free nodes are deliberately NOT
 * drawn — only the wireframe defines them, which keeps the cloth
 * surface reading as fabric rather than a graph-paper grid.
 */
static void draw_pinned_anchors(const Cloth *c, WINDOW *w, int cols, int rows,
                                float alpha) {
  for (int i = 0; i < CLOTH_N; i++) {
    const Node *n = &c->nodes[i];
    if (!n->pinned)
      continue;
    int cx, cy;
    node_lerp_cell(n, alpha, &cx, &cy);
    if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1)
      continue;
    wattron(w, COLOR_PAIR(CP_PIN) | A_BOLD);
    mvwaddch(w, cy, cx, '#');
    wattroff(w, COLOR_PAIR(CP_PIN) | A_BOLD);
  }
}

/*
 * cloth_draw — the cloth as the user sees it.
 * Two passes: the wireframe weave (every edge, coloured by strain),
 * then the pinned anchors on top.
 */
static void cloth_draw(const Cloth *c, WINDOW *w, int cols, int rows,
                       float alpha) {
  draw_strain_weave(c, w, cols, rows, alpha);
  draw_pinned_anchors(c, w, cols, rows, alpha);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
  Cloth cloth;
  int theme; /* active index into k_themes[] */
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  cloth_init(&s->cloth, 0, cols, rows);
  s->theme = 0;
  theme_apply(s->theme);
}

static void scene_tick(Scene *s, float dt, int cols, int rows) {
  (void)cols;
  (void)rows;
  cloth_tick(&s->cloth, dt);
}

static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows,
                       float alpha, float dt_sec) {
  (void)dt_sec;
  cloth_draw(&s->cloth, w, cols, rows, alpha);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *s) {
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

static void screen_free(Screen *s) {
  (void)s;
  endwin();
}

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * Two-bar HUD per CLAUDE.md convention:
 *   row 0  right (CP_HUD  bright yellow + bold)  — live status
 *   row -1 left  (CP_HINT bright cyan   + bold)  — actions / keys
 * If the full key list overflows the terminal width, a shorter hint is
 * substituted so the bar always fits on one line.
 */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps,
                        float alpha, float dt_sec) {
  erase();
  scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

  const Cloth *c = &sc->cloth;

  /* ── top: status (row 0, right-aligned) ─────────────────────────── */
  char windbuf[16];
  if (c->wind_on)
    snprintf(windbuf, sizeof windbuf, "%3.0f", c->wind_strength);
  else
    snprintf(windbuf, sizeof windbuf, "OFF");

  char top[180];
  snprintf(top, sizeof top,
           " preset:%s  theme:%s  wind:%s  %s  sim:%3d Hz  %5.1f fps ",
           preset_names[c->preset], k_themes[sc->theme].name, windbuf,
           c->paused ? "PAUSED " : "running", sim_fps, fps);
  int len = (int)strlen(top);
  int hx = s->cols - len;
  if (hx < 0)
    hx = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, hx, top, s->cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* ── bottom: actions (row -1, left-aligned) ─────────────────────── */
  const char *hint_full =
      " q:quit  spc:pause  r:restart  n/p:preset  t/T:theme  "
      "w:wind  +/-:strength  ]/[:Hz ";
  const char *hint_short = " q:quit  spc:pause  n:preset  t:theme  +/-:wind ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= s->cols - 1)
    hint = hint_short;

  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(s->rows - 1, 0, hint, s->cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *scn = &app->scene;
  Cloth *c = &scn->cloth;
  Screen *sc = &app->screen;

  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    c->paused = !c->paused;
    break;

  case 'r':
  case 'R':
    cloth_init(c, c->preset, sc->cols, sc->rows);
    break;

  case 'n':
    cloth_init(c, (c->preset + 1) % N_PRESETS, sc->cols, sc->rows);
    break;

  case 'p': {
    int p = c->preset - 1;
    if (p < 0)
      p = N_PRESETS - 1;
    cloth_init(c, p, sc->cols, sc->rows);
    break;
  }

  case 't':
    scn->theme = (scn->theme + 1) % N_THEMES;
    theme_apply(scn->theme);
    break;
  case 'T':
    scn->theme = (scn->theme + N_THEMES - 1) % N_THEMES;
    theme_apply(scn->theme);
    break;

  case 'w':
  case 'W':
    c->wind_on = !c->wind_on;
    break;

  case '+':
  case '=': /* '=' is shift-less '+' on US layouts */
    c->wind_strength += (float)WIND_STEP;
    if (c->wind_strength > (float)WIND_MAX)
      c->wind_strength = (float)WIND_MAX;
    c->wind_on = true; /* turning up wind implies it's on   */
    break;
  case '-':
  case '_':
    c->wind_strength -= (float)WIND_STEP;
    if (c->wind_strength < (float)WIND_MIN)
      c->wind_strength = (float)WIND_MIN;
    break;

  case ']':
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX)
      app->sim_fps = SIM_FPS_MAX;
    break;
  case '[':
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN)
      app->sim_fps = SIM_FPS_MIN;
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec, app->screen.cols, app->screen.rows);
      sim_accum -= tick_ns;
    }

    float alpha = (float)sim_accum / (float)tick_ns;

    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps, alpha,
                dt_sec);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
