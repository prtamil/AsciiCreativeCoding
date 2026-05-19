/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bubble_chamber.c — Charged Particles in a Magnetic Field
 *
 * Simulates a bubble chamber: charged particles travel through a region of
 * uniform magnetic field (B perpendicular to the screen) and leave curved
 * ionisation tracks as they lose energy.
 *
 * Physics
 *   Lorentz force (2D, B along z-axis):
 *     Rotate velocity by  omega = (q/m_eff) * B  each step.
 *     v' = R(omega) · v   — exact rotation matrix; no Euler spiral drift.
 *   Ionisation drag:
 *     |v| *= (1 − DRAG)  each step  →  orbit spirals inward.
 *   Cyclotron radius:
 *     r = |v| / |omega|   →  light particles curl tight, heavy ones arc gently.
 *
 * Particle types  (q/m_eff tuned for clear visual curvature on a terminal;
 *                  colour is theme-driven — see Themes below)
 *   e⁻  electron   qm = −0.20   tight spiral
 *   e⁺  positron   qm = +0.20   tight spiral, opposite curl to e⁻
 *   μ   muon       qm = −0.07   medium arc
 *   π   pion       qm = +0.045  wide arc
 *   p   proton     qm = +0.022  barely curves
 *
 * Trails are ring buffers drawn with age-faded characters:
 *   O head  * fresh  + medium  . fading
 *
 * Keys
 *   n  burst from centre     e  burst from edge
 *   b/B  field strength      Space  flip field direction
 *   k/K  cycle spawn type    t/T  cycle theme
 *   r  reset                 p  pause                q  quit
 *
 * Parameter-tuning keys (b/B, Space, k/K) auto-respawn a fresh centre
 * burst so the new field strength / direction / spawn type is visible
 * immediately on freshly-launched particles — no manual r needed.
 *
 * HUD: canonical CLAUDE.md two-bar — row 0 right shows live status
 * (field, alive, spawn, theme, paused/running); row rows-1 lists
 * the action keys.
 *
 * Themes (10 palettes; cycle with t / T): Matrix, Fire, Oceanic, Neon,
 * Mono, Ice, Nova, Forest, Desert, Eclipse.  Each theme provides five
 * distinguishable colours, slot-mapped to the five particle types.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/bubble_chamber.c \
 *       -o bubble_chamber -lncurses -lm
 *
 * §1 config  §2 clock  §3 species/themes/color  §4 physics  §5 scene
 * §6 draw    §7 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Exact rotation integration for charged particle motion [4].
 *                  Instead of Euler-approximating the Lorentz force (which
 *                  introduces spiral drift errors), each step applies an exact
 *                  2D rotation matrix R(ω·dt) to the velocity vector.
 *                  This preserves the orbital radius exactly (no energy drift)
 *                  — a key advantage over naive force integration for circular
 * motion.
 *
 * Physics        : Lorentz force in a uniform B-field perpendicular to the
 *                  screen [1]:
 *                    F = q · v × B  → angular velocity ω = (q/m) · B
 *                  The cyclotron (gyro) radius r = |v| / |ω| = m·|v| / (q·B).
 *                  Higher q/m → tighter curve (electrons), lower → gentle arc
 * (protons).
 *
 *                  Ionisation energy loss: |v| multiplied by (1−DRAG) each step
 *                  approximates the Bethe-Bloch slowing of a charged particle
 *                  in a medium [2].  The orbit spirals inward as the particle
 *                  loses energy — exactly the spiral-tightening signature of
 *                  real bubble chamber tracks photographed since Glaser's
 *                  invention of the device [3].
 *
 *                  Particle species (lepton / hadron classifications, q/m
 *                  ratios) follow standard particle-physics texts [2]:
 *                    e⁻ / e⁺ (leptons, tight curl)
 *                    μ       (heavier lepton)
 *                    π       (meson, hadronic)
 *                    p       (baryon, heaviest here → barely curves).
 *
 * Math           : Rotation matrix application:
 *                    v_x' = v_x · cos(ω) − v_y · sin(ω)
 *                    v_y' = v_x · sin(ω) + v_y · cos(ω)
 *                  Each particle stores a ring-buffer of TRAIL_LEN=300
 * positions; the head index advances each step, overwriting the oldest.
 *
 * Performance    : STEPS_PER_FRAME=4 sub-steps smooth the curvature at 30fps.
 *                  Cost: O(MAX_PARTICLES × TRAIL_LEN) drawing +
 * O(MAX_PARTICLES) physics.
 *
 * Rendering      : 10 brightness-safe theme palettes [5] — each maps the
 *                  five particle types to distinguishable colours within
 *                  one theme's mood.  Trail-age glyph ramp (O · * + .)
 *                  fades each track from a bold head to dim ionisation
 *                  history, so the eye reads time-direction along the curl.
 *
 * References (cite inline as [n]):
 *
 *   [1] Griffiths, D. J. — *Introduction to Electrodynamics*, 4th ed.,
 *       Cambridge Univ. Press (2017).  §5.1 Lorentz force; §5.4 cyclotron
 *       motion, gyroradius r = mv / (qB).  Foundational physics for §4
 *       (particle_step).
 *
 *   [2] Griffiths, D. J. — *Introduction to Elementary Particles*,
 *       2nd ed., Wiley-VCH (2008).  Lepton / hadron classifications,
 *       q/m ratios, ionisation energy loss (Bethe-Bloch).  Backs the
 *       choice of species and qm values in §3 (k_types).
 *
 *   [3] Glaser, D. A. (1952) — "Some Effects of Ionizing Radiation on
 *       the Formation of Bubbles in Liquids", *Phys. Rev.* 87, 665.
 *       The original bubble-chamber paper (Nobel Prize, 1960).  Explains
 *       why charged particles leave visible tracks: ionisation along
 *       the path nucleates bubbles in a superheated liquid — the visual
 *       analogue our trail ring buffer models.
 *
 *   [4] Birdsall, C. K. & Langdon, A. B. — *Plasma Physics via Computer
 *       Simulation*, IOP / CRC Press (2004).  §4 covers charged-particle
 *       pushers (Boris, Vay, and the exact-rotation scheme we use in
 *       particle_step).  Argues why naive Euler diverges and rotation-
 *       based schemes preserve energy.
 *
 *   [5] Ware, C. — *Information Visualization: Perception for Design*,
 *       4th ed., Morgan Kaufmann (2020).  Perceptually-ordered colour
 *       and luminance ramps (Ch. 4) back the 10 brightness-safe theme
 *       palettes in §3 (g_themes) and the age-faded trail glyph ramp
 *       in §6 (draw_particle).
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define MAX_PARTICLES 20
#define TRAIL_LEN 300 /* ring-buffer length per particle            */
#define N_TYPES 5
#define N_THEMES 10 /* MATRIX..ECLIPSE (see g_themes in §3)       */

/* HUD: canonical CLAUDE.md two-bar — top row right = live status,
 * bottom row left = action keys.  Particles draw between them. */
#define HUD_TOP 1
#define HUD_BOT 1

/* magnetic field */
#define B_INIT 1.0f /* default field strength                     */
#define B_MIN 0.1f
#define B_MAX 4.0f
#define B_STEP 0.1f

/* particle motion */
#define V_SPAWN                                                                \
  2.2f /* initial speed in cell/step; at STEPS_PER_FRAME=4,                    \
        * electron (qm=0.20, B=1.0) cyclotron radius ≈ 11 cells */
#define V_SPREAD                                                               \
  0.4f /* ±40% speed variation for visual spread of radii        */
#define DRAG                                                                   \
  0.003f /* 0.3% speed loss per step (ionisation); particle covers             \
          * ~1/0.003 ≈ 333 steps before halving — trails ~1000 px */
#define SPEED_DEAD                                                             \
  0.22f /* stop when radius < 1 cell (V/ω < 1); prevents                      \
         * particles spinning invisibly in a single cell         */

/* spawn */
#define BURST_MIN 2 /* particles per burst                        */
#define BURST_MAX 5

/* timing */
#define STEPS_PER_FRAME 4
#define RENDER_NS (1000000000LL / 30)

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void clock_sleep_ns(long long ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  species / themes / color                                           */
/* ===================================================================== */

/*
 * PType — descriptor for one particle species (electron, positron, …).
 *
 * Why a struct (vs three parallel const arrays):
 *   Each species bundles three coupled descriptors: a human-readable
 *   name, a 2-char HUD symbol, and a charge/mass ratio.  Grouping
 *   under one named type makes "for each species, do X" loops read
 *   naturally (k_types[i].qm, k_types[i].symbol) — no parallel-array
 *   indexing gymnastics.
 *
 * Why TUNED q/m (not real Standard-Model values):
 *   Real q/m ratios put electrons at cyclotron radius ~10⁻³ cells and
 *   protons at ~1.8 cells given the demo's V_SPAWN / B_INIT.  Most
 *   species would barely curve, or curve so tightly the curl is one
 *   sub-cell wide.  The qm values here are calibrated so each species
 *   lands at a visually useful radius:
 *
 *      species   qm        r = V_SPAWN / |qm·B|   relative curl
 *      e⁻       −0.200     ≈ 11 cells              tight
 *      e⁺       +0.200     ≈ 11 cells              tight, opposite curl
 *      μ        −0.070     ≈ 31 cells              medium arc
 *      π        +0.045     ≈ 49 cells              wide arc
 *      p        +0.022     ≈ 100 cells             barely curves
 *
 *   Ratios between species (e.g. e/μ ≈ 200/70) are preserved on a log
 *   scale even though absolute values are scaled.
 *
 * Sign convention (with B > 0 = field out of the screen, default):
 *     qm < 0  →  clockwise curl
 *     qm > 0  →  counter-clockwise curl
 *   Flipping B (Space key) negates every curl — the visual signature
 *   of charge symmetry / pair production.
 *
 * Why colour is NOT in this struct:
 *   Themes (§3 g_themes) supply per-species colour at runtime so the
 *   same particle list can render under 10 different palettes without
 *   editing PType.  Colour lives orthogonal to the physics descriptors.
 *
 * Algorithm refs (header REFERENCES):
 *   Lorentz force, cyclotron radius mv/(qB)   — Griffiths E&M [1] §5.4
 *   Lepton / hadron taxonomy, real q/m values — Griffiths Elementary [2]
 */
typedef struct {
  const char *name;   /* full display name ("electron")          */
  const char *symbol; /* 2-char HUD label ("e-", "mu", "p ")     */
  float qm;           /* tuned charge/mass ratio (see table)     */
} PType;

static const PType k_types[N_TYPES] = {
    {"electron", "e-", -0.200f}, {"positron", "e+", +0.200f},
    {"muon", "mu", -0.070f},     {"pion", "pi", +0.045f},
    {"proton", "p ", +0.022f},
};

/*
 * Theme — five 256-colour cube indices, slot-mapped to the five
 * particle types (electron, positron, muon, pion, proton).  Each
 * theme provides distinct colours within the theme's mood so the
 * track types are still visually separable.
 *
 * Brightness safety (CLAUDE.md): every entry ≥ 30, or 24-29 / 240-243
 * only used as the dimmest slot.  Forbidden 16-23 / 232-239 avoided.
 */
typedef struct {
  const char *name;
  short fg[N_TYPES];
} Theme;

static const Theme g_themes[N_THEMES] = {
    /*  name       e-   e+   mu   pi   p    */
    {"Matrix", {46, 82, 118, 154, 226}},   /* cyber green ramp + yellow */
    {"Fire", {196, 226, 220, 208, 130}},   /* white-hot to ember        */
    {"Oceanic", {51, 87, 159, 195, 117}},  /* cyans / light blues       */
    {"Neon", {196, 51, 201, 226, 46}},     /* high-sat neon mix         */
    {"Mono", {255, 252, 248, 244, 240}},   /* grayscale ramp            */
    {"Ice", {195, 159, 153, 117, 87}},     /* light blues to cyan       */
    {"Nova", {231, 213, 177, 141, 105}},   /* stellar white→violet      */
    {"Forest", {190, 154, 100, 130, 64}},  /* leaves to bark            */
    {"Desert", {226, 220, 214, 178, 130}}, /* sand / gold / brown       */
    {"Eclipse", {196, 244, 240, 124, 88}}, /* corona red + grays        */
};

/* 8-colour fallback — fixed mapping (themes can't add variety here). */
static const short g_8fg[N_TYPES] = {COLOR_BLUE, COLOR_RED, COLOR_GREEN,
                                     COLOR_YELLOW, COLOR_CYAN};

/* Color pair layout: 1..N_TYPES are particle types, then HUD + HINT. */
#define CP_HUD (N_TYPES + 1)  /* top status — bright yellow + bold */
#define CP_HINT (N_TYPES + 2) /* bottom hint bar — bright cyan + bold */

static bool g_256color;

static void color_init(int theme_idx) {
  start_color();
  use_default_colors();
  g_256color = (COLORS >= 256);

  const Theme *t = &g_themes[theme_idx % N_THEMES];
  for (int i = 0; i < N_TYPES; i++) {
    short fg = g_256color ? t->fg[i] : g_8fg[i];
    init_pair(1 + i, fg, -1);
  }
  if (g_256color) {
    init_pair(CP_HUD, 226, -1); /* bright yellow */
    init_pair(CP_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
}

static inline int particle_cp(int kind) { return 1 + kind; }

/* ===================================================================== */
/* §4  physics                                                            */
/* ===================================================================== */

/*
 * Particle — one charged-track instance in the chamber.
 *
 * Why a trail ring buffer (not re-derive the path each frame):
 *   The visual signature of a bubble chamber is the lingering trail of
 *   ionisation bubbles along the path.  Re-deriving the path each
 *   frame would need either a long backward time-integration (expensive)
 *   or a closed-form orbit equation (only valid before drag changes the
 *   speed).  A fixed-size ring buffer of TRAIL_LEN past positions
 *   instead lets us draw the recorded history directly, in O(N) per draw.
 *
 * Why a ring buffer (not a growing array):
 *   Bounded memory + O(1) push.  The oldest sample is silently
 *   overwritten when the buffer wraps — the trail naturally has a
 *   maximum age, which matches the visual "ionisation fades" model.
 *
 * Index convention (consumed in §6 draw_particle):
 *   tx[] / ty[]   recorded positions
 *   thead         NEXT write slot — advance after each push
 *   tlen          how many slots are valid (saturates at TRAIL_LEN)
 *
 *   Reading newest-to-oldest at distance i ∈ [0, tlen):
 *     idx = (thead − 1 − i + TRAIL_LEN) mod TRAIL_LEN
 *   Age fraction:
 *     age = i / tlen          (i = 0 newest, i = tlen−1 oldest)
 *
 * Why floats for x/y/vx/vy:
 *   Sub-cell precision is needed for smooth cyclotron orbits at
 *   r ≈ 10–100 cells.  Integer-cell coordinates would quantise the
 *   velocity direction to a handful of steps and the curl would look
 *   stair-stepped.  Quantisation to integer cells happens only at
 *   render time via `(int)(p->x + 0.5f)`.
 *
 * Why an explicit `alive` flag (not implicit via NULL slot):
 *   Slots are reused on respawn via find_dead_slot().  The flag
 *   explicit-marks the logical state; the trail buffer is left intact
 *   so a short-lived particle's track stays visible until overwritten
 *   when the slot is reused.
 *
 * Algorithm refs (header REFERENCES):
 *   Exact rotation integrator     — Birdsall & Langdon [4] §4
 *   Ionisation drag (Bethe-Bloch) — Griffiths Elementary [2]
 *   Bubble-chamber track imaging  — Glaser 1952 [3]
 */
typedef struct {
  /* ── Phase-space state (advanced each step by particle_step) ─── */
  float x, y;   /* current position, cell-space floats     */
  float vx, vy; /* current velocity, cells per sim step    */

  /* ── Identity / lifecycle ─────────────────────────────────────── */
  int kind;   /* index into k_types[] in §3              */
  bool alive; /* false → slot recyclable by find_dead_slot */

  /* ── Trail ring buffer (read newest-to-oldest by §6) ──────────── */
  float tx[TRAIL_LEN]; /* recorded x-positions, oldest-overwritten */
  float ty[TRAIL_LEN]; /* recorded y-positions                     */
  int thead;           /* next write index ∈ [0, TRAIL_LEN)        */
  int tlen;            /* valid sample count, saturates at TRAIL_LEN */
} Particle;

/* ─────────────────────────────────────────────────────────────────────── *
 * Scene — state that spans physics ticks, draw calls, and the main loop.
 *
 * The struct splits into two clearly-labelled groups:
 *
 *   Simulation parameters  — consumed by scene_step and the spawn
 *     helpers.  Anything that affects the PHYSICS (particle state,
 *     field, who-spawns-what) lives here.  Mutated by physics-
 *     affecting keys: b / B (field), Space (flip), k / K (spawn
 *     type), p (pause), r (reset), n / e (bursts).
 *
 *   Rendering parameters   — consumed by scene_draw and the HUD only.
 *     Toggling these while paused must leave particle positions and
 *     velocities byte-identical; only colours may differ.  Mutated by
 *     purely cosmetic keys: t / T (theme).
 *
 * Locality rationale:
 *   The split exists for the READER, not the CPU.  A new flag landing
 *   in the rendering group when it actually steers a particle (or
 *   spawn behaviour) would silently couple display to physics —
 *   exactly the bug the separation prevents.  When adding a field,
 *   ask: does this change what scene_step or the spawn helpers
 *   produce?  If yes, simulation; if no, rendering.
 *
 * Single instance (file-scope `g_scene`):
 *   particles[] dominates the footprint (~1.4 MB at TRAIL_LEN=300,
 *   MAX_PARTICLES=20), so the struct lives in BSS as a file-static
 *   rather than being passed by pointer.  All scene state is accessed
 *   as `g_scene.<field>` from the helpers and the main loop.
 *
 * What stays OUTSIDE this struct (intentionally):
 *   g_quit / g_resize    sig_atomic_t flags read by signal handlers;
 *                        must stay at file scope for async-signal safety.
 *   g_256color           one-shot colour-capability flag set at startup;
 *                        never mutated — no benefit in scene membership.
 *   g_rows / g_cols      screen geometry tracked by the main loop;
 *                        Scene stays geometry-agnostic so resize handling
 *                        is the main loop's concern, not the renderer's.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── Simulation parameters ────────────────────────────────────── */
  Particle particles[MAX_PARTICLES];
  float B;        /* magnetic-field strength, signed.  Space
                   * key flips the sign (reverses every curl).
                   * Magnitude in [B_MIN, B_MAX]; b / B step it. */
  bool paused;    /* true → scene_step is a no-op (p key)       */
  int spawn_kind; /* species index for new bursts:
                   *   −1       = random
                   *   0..N_TYPES−1 = fixed species.
                   * Cycled by k / K keys.                       */

  /* ── Rendering parameters ─────────────────────────────────────── */
  int theme; /* index into g_themes[] (§3); cycled by t/T.
              * Pure cosmetic — never alters physics.       */
} Scene;

static Scene g_scene = {
    .B = B_INIT, .paused = false, .spawn_kind = -1, .theme = 0,
    /* .particles[] is BSS-zeroed; respawn_centre_burst fills it. */
};

/* Screen geometry — main-loop bookkeeping, not scene state
 * (see Scene docstring above). */
static int g_rows, g_cols;

/*
 * Per-step physics helpers — each isolates one operator in the
 * symplectic-Euler-like sequence rotate → drag → drift → record →
 * test-death.  particle_step is then a five-line pseudocode driver.
 *
 * Algorithm refs (header REFERENCES):
 *   Exact rotation for Lorentz force         — Birdsall & Langdon §4 [4]
 *   Bethe-Bloch ionisation drag (approximated) — Griffiths Elem. [2]
 *   Bubble-chamber track imaging             — Glaser 1952 [3]
 */

/*
 * rotate_velocity_exact — closed-form 2-D rotation of the velocity:
 *
 *     v' = R(ω) · v,    R = [[cos ω, −sin ω], [sin ω, cos ω]]
 *
 * Exact solution of dv/dt = ω × v for a constant ω = (q/m)·B over one
 * unit time-step.  Unlike Euler (v += (ω × v)·dt), which inflates |v|
 * by O(ω²dt²) each step, this preserves |v| exactly — no energy drift,
 * perfect circular orbits in the absence of drag.
 */
static void rotate_velocity_exact(Particle *p, float omega) {
  float ca = cosf(omega), sa = sinf(omega);
  float nvx = p->vx * ca - p->vy * sa;
  float nvy = p->vx * sa + p->vy * ca;
  p->vx = nvx;
  p->vy = nvy;
}

/*
 * apply_ionisation_drag — multiplicative speed decay per step.
 *
 *     v ← v · (1 − DRAG)
 *
 * Approximates the Bethe-Bloch dE/dx slowing of a charged particle
 * traversing a medium.  Over many steps the particle decelerates,
 * the cyclotron radius r = |v|/|ω| shrinks, and the orbit spirals
 * inward — the classic bubble-chamber signature.
 */
static void apply_ionisation_drag(Particle *p) {
  p->vx *= (1.f - DRAG);
  p->vy *= (1.f - DRAG);
}

/*
 * advance_position — symplectic-Euler drift step.
 *
 *     x ← x + v_x        (rotated, dragged velocity from this step)
 *     y ← y + v_y
 *
 * Position is updated with the END-of-step velocity, which makes the
 * pair (rotate, drift) symplectic for constant ω.  Sub-cell precision
 * keeps the curl smooth at r ≈ 10–100 cells.
 */
static void advance_position(Particle *p) {
  p->x += p->vx;
  p->y += p->vy;
}

/*
 * record_trail_sample — push the current (x, y) into the ring buffer.
 *
 *   tx[thead] ← x;  ty[thead] ← y
 *   thead     ← (thead + 1) mod TRAIL_LEN
 *   tlen      ← min(tlen + 1, TRAIL_LEN)
 *
 * The oldest sample is silently overwritten when the buffer wraps —
 * the trail therefore has a bounded maximum age, matching the visual
 * "ionisation fades" model.
 */
static void record_trail_sample(Particle *p) {
  p->tx[p->thead] = p->x;
  p->ty[p->thead] = p->y;
  p->thead = (p->thead + 1) % TRAIL_LEN;
  if (p->tlen < TRAIL_LEN)
    p->tlen++;
}

/*
 * is_subgyro_dead — has drag slowed the particle below visible radius?
 *
 *     |v| < SPEED_DEAD  ⇒  cyclotron radius r = |v|/|ω| < 1 cell
 *
 * Below this threshold the particle would orbit invisibly inside a
 * single character cell.  Marking it dead lets find_dead_slot recycle
 * the slot for a fresh particle.
 */
static int is_subgyro_dead(const Particle *p) {
  float spd2 = p->vx * p->vx + p->vy * p->vy;
  return spd2 < SPEED_DEAD * SPEED_DEAD;
}

/*
 * particle_step — one physics step in the Lorentz + drag system.
 *
 * Pseudocode:
 *   if not alive: return
 *   ω ← k_types[kind].qm · g_scene.B
 *   rotate_velocity_exact(p, ω)     // Lorentz turn (exact)
 *   apply_ionisation_drag(p)        // Bethe-Bloch slowing
 *   advance_position(p)             // kinematic drift
 *   record_trail_sample(p)          // ring-buffer push
 *   if is_subgyro_dead(p): p->alive ← false
 */
static void particle_step(Particle *p) {
  if (!p->alive)
    return;

  float omega = k_types[p->kind].qm * g_scene.B;
  rotate_velocity_exact(p, omega);
  apply_ionisation_drag(p);
  advance_position(p);
  record_trail_sample(p);
  if (is_subgyro_dead(p))
    p->alive = false;
}

/* ===================================================================== */
/* §5  scene                                                              */
/* ===================================================================== */

static void scene_reset(void) {
  memset(g_scene.particles, 0, sizeof g_scene.particles);
}

/*
 * init_particle() — fill one particle slot.
 * cx, cy  : spawn centre (physics col/row coordinates)
 * angle   : initial velocity direction (radians)
 * kind    : particle type (−1 = random)
 */
static void init_particle(Particle *p, float cx, float cy, float angle,
                          int kind) {
  memset(p, 0, sizeof *p);
  p->x = cx;
  p->y = cy;
  p->kind = (kind < 0) ? rand() % N_TYPES : kind;

  float speed = V_SPAWN * (1.f - V_SPREAD / 2.f +
                           V_SPREAD * ((float)rand() / (float)RAND_MAX));
  p->vx = cosf(angle) * speed;
  p->vy = sinf(angle) * speed;
  p->alive = true;
}

/*
 * find_dead_slot() — return index of first non-alive particle, or −1 if full.
 */
static int find_dead_slot(void) {
  for (int i = 0; i < MAX_PARTICLES; i++)
    if (!g_scene.particles[i].alive)
      return i;
  return -1;
}

/*
 * spawn_burst_centre() — n particles from screen centre, random directions.
 * Models a head-on collision vertex.
 */
static void spawn_burst_centre(int n) {
  float cx = (float)g_cols * 0.5f;
  float cy = (float)(g_rows - HUD_TOP - HUD_BOT) * 0.5f;
  int k = (g_scene.spawn_kind < 0) ? -1 : g_scene.spawn_kind;

  for (int i = 0; i < n; i++) {
    int slot = find_dead_slot();
    if (slot < 0)
      break;
    float angle = ((float)rand() / (float)RAND_MAX) * 2.f * 3.14159265f;
    init_particle(&g_scene.particles[slot], cx, cy, angle, k);
  }
}

/*
 * edge_entry_geometry — uniformly-random point on a chosen screen edge
 * plus the inward-pointing base velocity angle for that edge.
 *
 *     edge 0 (top)    : cx = U(0,W); cy = 0;     base_angle = +π/2 (down)
 *     edge 1 (bottom) : cx = U(0,W); cy = H−1;   base_angle = −π/2 (up)
 *     edge 2 (left)   : cx = 0;     cy = U(0,H); base_angle =  0   (right)
 *     edge 3 (right)  : cx = W−1;   cy = U(0,H); base_angle =  π   (left)
 *
 * H is the particle-area height (excludes the HUD top/bottom rows).
 * Used by spawn_burst_edge to model a beam entering the chamber from
 * an arbitrary side.
 */
static void edge_entry_geometry(int edge, float W, float H, float *cx,
                                float *cy, float *base_angle) {
  float u = (float)rand() / (float)RAND_MAX;
  switch (edge) {
  case 0:
    *cx = W * u;
    *cy = 0;
    *base_angle = 0.5f * 3.14159265f;
    break;
  case 1:
    *cx = W * u;
    *cy = H - 1;
    *base_angle = -0.5f * 3.14159265f;
    break;
  case 2:
    *cx = 0;
    *cy = H * u;
    *base_angle = 0.0f;
    break;
  default:
    *cx = W - 1;
    *cy = H * u;
    *base_angle = 3.14159265f;
    break;
  }
}

/*
 * inward_velocity_angle — perturb base_angle by ±half_spread (radians).
 *
 *   angle = base_angle + U(−1, 1) · half_spread
 *
 * Models a beam with finite angular dispersion: every spawned particle
 * is launched inward along the edge normal plus a small random spread.
 */
static float inward_velocity_angle(float base_angle, float half_spread) {
  float u = (float)rand() / (float)RAND_MAX;
  return base_angle + (u - 0.5f) * 2.0f * half_spread;
}

/*
 * spawn_burst_edge — n particles entering from a random screen edge,
 * velocity directed inward ± 30°.  Models a beam crashing into the
 * chamber from outside.
 *
 * Pseudocode:
 *   edge ← rand mod 4
 *   edge_entry_geometry(edge, W, H) → (cx, cy, base_angle)
 *   k ← g_scene.spawn_kind            (−1 = random species per particle)
 *   for i in [0, n):
 *       angle ← inward_velocity_angle(base_angle, ±30°)
 *       init_particle at (cx, cy) with angle and kind k
 */
static void spawn_burst_edge(int n) {
  int edge = rand() % 4; /* 0=top 1=bottom 2=left 3=right */
  float W = (float)g_cols;
  float H = (float)(g_rows - HUD_TOP - HUD_BOT);

  float cx, cy, base_angle;
  edge_entry_geometry(edge, W, H, &cx, &cy, &base_angle);

  int k = g_scene.spawn_kind; /* −1 propagates to init_particle as random */
  for (int i = 0; i < n; i++) {
    int slot = find_dead_slot();
    if (slot < 0)
      break;
    float angle = inward_velocity_angle(base_angle, 0.5235988f); /* ±30° */
    init_particle(&g_scene.particles[slot], cx, cy, angle, k);
  }
}

static void scene_step(void) {
  for (int i = 0; i < MAX_PARTICLES; i++)
    particle_step(&g_scene.particles[i]);
}

/*
 * respawn_centre_burst — clear the chamber and spawn a fresh centre burst.
 *
 * Used by 'r' (explicit reset) and by parameter-change keys (b/B/space/
 * k/K) so the user sees the new parameter's effect immediately on fresh
 * particles rather than having to wait for the existing ones (which
 * already locked in their trail/cyclotron-radius before the change).
 */
static void respawn_centre_burst(void) {
  scene_reset();
  int n = BURST_MIN + rand() % (BURST_MAX - BURST_MIN + 1);
  spawn_burst_centre(n);
}

static int alive_count(void) {
  int n = 0;
  for (int i = 0; i < MAX_PARTICLES; i++)
    if (g_scene.particles[i].alive)
      n++;
  return n;
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

/*
 * Trail-rendering helpers — pick the glyph for each age slice, project
 * ring-buffer samples to screen coordinates, and paint the trail and
 * live head.  draw_particle is then the four-line dispatcher.
 *
 * Algorithm ref: perceptual age-fade glyph ramp — Ware [5].
 */

/*
 * trail_age_glyph — choose glyph + extra attribute from age ∈ [0, 1].
 *
 *   age < 0.25 :  '*' bold      — bright fresh ionisation
 *   age < 0.55 :  '+' normal    — settled trail
 *   age < 0.80 :  '.' normal    — fading ionisation
 *   age ≥ 0.80 :  return 0      — too old, skip (saves the inner loop)
 *
 * The four bands approximate an exponential brightness decay using the
 * available ASCII density and bold flag — coarse but readable on a
 * low-resolution terminal.
 */
static int trail_age_glyph(float age, chtype *ch, attr_t *extra_attr) {
  if (age >= 0.80f)
    return 0;
  if (age < 0.25f) {
    *ch = '*';
    *extra_attr = A_BOLD;
  } else if (age < 0.55f) {
    *ch = '+';
    *extra_attr = A_NORMAL;
  } else {
    *ch = '.';
    *extra_attr = A_NORMAL;
  }
  return 1;
}

/*
 * trail_sample_to_screen — ring-buffer sample i (newest = 0) → screen
 * (col, row), accounting for the HUD top-row offset.
 *
 *   idx = (thead − 1 − i + TRAIL_LEN · 2) mod TRAIL_LEN
 *   col = round(tx[idx])
 *   row = round(ty[idx]) + HUD_TOP
 *
 * Returns 1 if on-screen (caller draws), 0 if outside the visible draw
 * area (caller skips).
 */
static int trail_sample_to_screen(const Particle *p, int i, int draw_rows,
                                  int *col, int *row) {
  int idx = (p->thead - 1 - i + TRAIL_LEN * 2) % TRAIL_LEN;
  *col = (int)(p->tx[idx] + 0.5f);
  *row = (int)(p->ty[idx] + 0.5f) + HUD_TOP;
  return *col >= 0 && *col < g_cols && *row >= HUD_TOP && *row < draw_rows;
}

/*
 * paint_trail_history — render every age-eligible ring-buffer sample
 * in the particle's colour, newest-to-oldest.  Breaks out of the loop
 * as soon as trail_age_glyph rejects an age (samples are sorted by
 * age, so all remaining ones are older).
 */
static void paint_trail_history(const Particle *p, int draw_rows, int cp) {
  int denom = p->tlen > 1 ? p->tlen : 1;
  for (int i = 0; i < p->tlen; i++) {
    chtype ch;
    attr_t extra;
    float age = (float)i / (float)denom;
    if (!trail_age_glyph(age, &ch, &extra))
      break;

    int col, row;
    if (!trail_sample_to_screen(p, i, draw_rows, &col, &row))
      continue;

    attr_t attr = (attr_t)COLOR_PAIR(cp) | extra;
    attron(attr);
    mvaddch(row, col, ch);
    attroff(attr);
  }
}

/*
 * paint_live_head — render the live particle as a bold 'O' at its
 * current sub-cell position (snapped to nearest cell), one cell ahead
 * of the freshest trail sample.
 */
static void paint_live_head(const Particle *p, int draw_rows, int cp) {
  int col = (int)(p->x + 0.5f);
  int row = (int)(p->y + 0.5f) + HUD_TOP;
  if (col < 0 || col >= g_cols || row < HUD_TOP || row >= draw_rows)
    return;
  attron(COLOR_PAIR(cp) | A_BOLD);
  mvaddch(row, col, 'O');
  attroff(COLOR_PAIR(cp) | A_BOLD);
}

/*
 * draw_particle — render one particle (trail + live head).
 *
 * Pseudocode:
 *   if dead and trail empty: return
 *   cp ← particle_cp(p->kind)
 *   paint_trail_history(p, draw_rows, cp)
 *   if alive: paint_live_head(p, draw_rows, cp)
 */
static void draw_particle(const Particle *p, int draw_rows) {
  if (!p->alive && p->tlen == 0)
    return;

  int cp = particle_cp(p->kind);
  paint_trail_history(p, draw_rows, cp);
  if (p->alive)
    paint_live_head(p, draw_rows, cp);
}

/*
 * HUD-top helpers — three formatters for the right-aligned status row,
 * one painter for the spawn-type bracket that does the in-place colour
 * swap, and a dispatcher.  Layout:
 *
 *   " B=N.NN[(flipped)]  alive=K/M  spawn=[X]  theme:NAME  STATE "
 *   |_______ before _______________________| |_tag_| |__ after _|
 *
 * The middle segment is recoloured with the particle's own pair so the
 * tag doubles as a colour legend.  CP_HUD wraps before/after so the bar
 * reads as a single bright-yellow line broken only by the tag.
 *
 * Algorithm ref: perceptual-cue colour use — Ware [5].
 */

/*
 * format_hud_status_prefix — left segment: field, alive count, "spawn=".
 */
static void format_hud_status_prefix(char *buf, size_t n) {
  snprintf(buf, n, " B=%.2f%s  alive=%d/%d  spawn=", (double)fabsf(g_scene.B),
           g_scene.B < 0 ? "(flipped)" : "", alive_count(), MAX_PARTICLES);
}

/*
 * format_hud_spawn_tag — middle segment: "[rand]" or "[sym]".
 */
static void format_hud_spawn_tag(char *buf, size_t n) {
  if (g_scene.spawn_kind < 0)
    snprintf(buf, n, "[rand]");
  else
    snprintf(buf, n, "[%s]", k_types[g_scene.spawn_kind].symbol);
}

/*
 * format_hud_status_suffix — right segment: theme + pause/run state.
 */
static void format_hud_status_suffix(char *buf, size_t n) {
  snprintf(buf, n, "  theme:%s  %s ", g_themes[g_scene.theme % N_THEMES].name,
           g_scene.paused ? "PAUSED" : "running");
}

/*
 * paint_spawn_tag_with_particle_colour — write the spawn-type tag at
 * the current cursor with the particle's own colour as a legend cue.
 *
 * Caller must already have CP_HUD + A_BOLD active.  This swap-pair-
 * swap-back leaves the surrounding HUD attribute intact so before/
 * after continue to read as a single yellow line.  Random-species
 * spawns (spawn_kind = −1) fall through to plain HUD styling.
 */
static void paint_spawn_tag_with_particle_colour(const char *tag) {
  if (g_scene.spawn_kind < 0) {
    addstr(tag);
    return;
  }
  int cp = particle_cp(g_scene.spawn_kind);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
  attron(COLOR_PAIR(cp) | A_BOLD);
  addstr(tag);
  attroff(COLOR_PAIR(cp) | A_BOLD);
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/*
 * draw_hud_top — row 0 right-aligned live status (CP_HUD bright yellow + bold).
 *
 * Pseudocode:
 *   format prefix, spawn_tag, suffix into local buffers
 *   col ← g_cols − (len_prefix + len_tag + len_suffix)
 *   move to (0, col)
 *   attron CP_HUD + bold
 *     write prefix
 *     paint_spawn_tag_with_particle_colour(tag)   // in-place colour swap
 *     write suffix
 *   attroff
 */
static void draw_hud_top(void) {
  char before[80], spawn_tag[16], after[64];
  format_hud_status_prefix(before, sizeof before);
  format_hud_spawn_tag(spawn_tag, sizeof spawn_tag);
  format_hud_status_suffix(after, sizeof after);

  int total = (int)(strlen(before) + strlen(spawn_tag) + strlen(after));
  int col = g_cols - total;
  if (col < 0)
    col = 0;

  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddstr(0, col, before);
  paint_spawn_tag_with_particle_colour(spawn_tag);

  addstr(after);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/*
 * draw_hud_bottom — row rows-1 left-aligned action keys (CP_HINT bright cyan +
 * bold). Short fallback fires when the terminal is too narrow for the full
 * list.
 */
static void draw_hud_bottom(void) {
  const char *hint_full =
      " q:quit  p:pause  r:reset  n:burst-centre  e:burst-edge  "
      "b/B:field  spc:flip  k/K:type  t/T:theme ";
  const char *hint_short =
      " q:quit  p:pause  r:reset  n:burst  k:type  t:theme ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= g_cols - 1)
    hint = hint_short;

  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(g_rows - 1, 0, hint, g_cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void scene_draw(void) {
  /* Particle draw area: between top status (row 0) and bottom hint
   * (row rows-1).  draw_rows is the exclusive upper bound. */
  int draw_rows = g_rows - HUD_BOT;

  for (int i = 0; i < MAX_PARTICLES; i++)
    draw_particle(&g_scene.particles[i], draw_rows);

  draw_hud_top();
  draw_hud_bottom();
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s) {
  if (s == SIGINT || s == SIGTERM)
    g_quit = 1;
  if (s == SIGWINCH)
    g_resize = 1;
}

static void cleanup(void) { endwin(); }

int main(void) {
  srand((unsigned)time(NULL));
  atexit(cleanup);
  signal(SIGINT, sig_h);
  signal(SIGTERM, sig_h);
  signal(SIGWINCH, sig_h);

  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);

  /* Theme defaults to g_scene.theme = 0 (Matrix) from the Scene initializer. */
  color_init(g_scene.theme);

  getmaxyx(stdscr, g_rows, g_cols);
  /* initial event: mixed burst from centre */
  respawn_centre_burst();

  long long next_frame = clock_ns();

  while (!g_quit) {

    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, g_rows, g_cols);
    }

    int ch = getch();
    switch (ch) {
    case 'q':
    case 'Q':
    case 27:
      g_quit = 1;
      break;
    case 'p':
    case 'P':
      g_scene.paused = !g_scene.paused;
      break;

    case 'r':
    case 'R':
      respawn_centre_burst();
      break;

    case 'n':
    case 'N': {
      int n = BURST_MIN + rand() % (BURST_MAX - BURST_MIN + 1);
      spawn_burst_centre(n);
      break;
    }
    case 'e':
    case 'E': {
      int n = BURST_MIN + rand() % (BURST_MAX - BURST_MIN + 1);
      spawn_burst_edge(n);
      break;
    }

    /* Parameter-change keys auto-respawn so the new value is
     * visible on fresh particles, not just baked into the next
     * tick of the existing slow / dying ones. */
    case 'b':
      g_scene.B += B_STEP;
      if (g_scene.B > B_MAX)
        g_scene.B = B_MAX;
      respawn_centre_burst();
      break;
    case 'B':
      g_scene.B -= B_STEP;
      if (fabsf(g_scene.B) < B_MIN)
        g_scene.B = (g_scene.B < 0) ? -B_MIN : B_MIN;
      respawn_centre_burst();
      break;

    case ' ':
      g_scene.B = -g_scene.B; /* flip field direction — reverses all curls */
      respawn_centre_burst();
      break;

    case 'k':
      /* cycle spawn-kind forward: rand → e- → e+ → mu → pi → p → rand */
      g_scene.spawn_kind = (g_scene.spawn_kind + 1 + 1) % (N_TYPES + 1) - 1;
      respawn_centre_burst();
      break;
    case 'K':
      g_scene.spawn_kind =
          (g_scene.spawn_kind + N_TYPES + 1) % (N_TYPES + 1) - 1;
      respawn_centre_burst();
      break;

    case 't':
      g_scene.theme = (g_scene.theme + 1) % N_THEMES;
      color_init(g_scene.theme);
      break;
    case 'T':
      g_scene.theme = (g_scene.theme + N_THEMES - 1) % N_THEMES;
      color_init(g_scene.theme);
      break;

    default:
      break;
    }

    long long now = clock_ns();
    if (!g_scene.paused && now >= next_frame) {
      for (int s = 0; s < STEPS_PER_FRAME; s++)
        scene_step();
      next_frame = now + RENDER_NS;
    }

    erase();
    scene_draw();
    wnoutrefresh(stdscr);
    doupdate();
    clock_sleep_ns(next_frame - clock_ns());
  }
  return 0;
}
