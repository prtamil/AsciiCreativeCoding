/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * elastic_collision.c — Hard Sphere Billiards
 *
 * 25 discs with randomised radii and velocities bouncing off walls and
 * each other.  Perfectly elastic collisions preserve kinetic energy and
 * momentum.  After N² pair checks each tick, colliding discs flash with
 * the active theme's accent colour.
 *
 * Physics in pixel space (CELL_W=8, CELL_H=16 px per terminal cell).
 * Disc mass proportional to radius² (equal density).
 *
 * Themes (t / T cycle, 10 total): Matrix, Fire, Oceanic, Neon, Mono,
 *   Ice, Nova, Forest, Desert, Eclipse.  Drive disc speed-tier colours
 *   (slow/med/fast) plus the collision flash accent.  Top yellow / bottom
 *   cyan HUD chrome stays fixed across every theme.
 *
 * Keys:
 *   q / ESC   quit
 *   p         pause / resume
 *   r         reset (new random layout)
 *   t / T     next / previous theme
 *   + / -     sim speed (1x..8x)
 *   space     add random impulse to every disc
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/elastic_collision.c \
 *       -o elastic_collision -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 physics  §5 draw  §6 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Impulse-based elastic collision resolution.
 *                  For each overlapping pair, a single impulse along the
 *                  collision normal simultaneously adjusts both velocities
 *                  so the constraint (non-penetration + elastic restitution)
 *                  is satisfied in one step.
 *
 * Physics        : Conservation laws for elastic collisions:
 *                  (1) Conservation of momentum: m₁v₁ + m₂v₂ = const
 *                  (2) Conservation of kinetic energy: ½m₁v₁² + ½m₂v₂² = const
 *                  Combined, for collision along normal n̂:
 *                    impulse J = 2·m₁·m₂/(m₁+m₂) · Δv·n̂
 *                  Velocities updated: v₁ -= J/m₁·n̂;  v₂ += J/m₂·n̂
 *
 * Performance    : O(N²) pair checks per tick — acceptable for N=25.
 *                  For N>100 broad-phase (spatial hash or sweep-and-prune)
 *                  would reduce to O(N) average checks.
 *
 * Mass model     : mass = r²  (area of 2D disc × uniform density).
 *                  Heavier discs (larger radius) deflect smaller ones
 *                  more, matching intuition about billiard balls.
 *
 * Rendering      : Each disc drawn as an ASCII ellipse — `O` at the
 *                  centre plus rim chars chosen by row (`|` at the
 *                  leftmost/rightmost columns, `-` everywhere else).
 *                  The ellipse equation y = ry · sqrt(1 − (dc/rx)²)
 *                  gives the rim row offset per column.  Speed-based
 *                  colour from theme ramp (slow → ramp[0]; med →
 *                  ramp[2]; fast → ramp[3]); collision tick paints the
 *                  theme's `flash` accent for FLASH_S seconds.
 *
 * References     :
 *   • Alder, B. J. & Wainwright, T. E. (1959) "Studies in Molecular
 *     Dynamics. I. General Method", J. Chem. Phys. 31(2), 459-466.
 *     — The first hard-sphere molecular dynamics simulation.  The
 *       overlap-detect / impulse-resolve loop we use is a direct
 *       descendant of their event-driven scheme (with continuous-time
 *       collision detection traded for discrete fixed-dt overlap
 *       resolution for visual smoothness).
 *
 *   • Goldstein, H., Poole, C. P. & Safko, J. L. (2001) "Classical
 *     Mechanics" (3rd ed.), Addison-Wesley.  Chapter 3 §3.10-3.11.
 *     — Standard textbook derivation of two-body elastic collisions.
 *       The impulse formula J = 2·m₁·m₂/(m₁+m₂) · Δv·n̂ in §4 of this
 *       file is the centre-of-momentum result derived there.
 *
 *   • Mirtich, B. (1996) "Impulse-based Dynamic Simulation of Rigid
 *     Body Systems", PhD thesis, UC Berkeley.
 *     — The canonical reference for impulse-based collision response.
 *       Generalises to arbitrary geometry but reduces to our scalar
 *       impulse formula for spheres along the contact normal.
 *
 *   • Witkin, A. & Baraff, D. (1997) "Physically Based Modeling:
 *     Principles and Practice", SIGGRAPH Course Notes.
 *     — Game-physics canonical course notes.  §F covers rigid-body
 *       contact and impulse forces with worked examples.
 *
 *   • Lubachevsky, B. D. (1991) "How to Simulate Billiards and Similar
 *     Systems", J. Computational Physics 94(2), 255-283.
 *     — Specifically targets large-N billiard simulation with broad-
 *       phase culling.  Start here when N grows past ~100 and the
 *       O(N²) pair loop becomes a bottleneck.
 *
 *   • Sinai, Ya. G. (1970) "Dynamical Systems with Elastic Reflections.
 *     Ergodic Properties of Dispersing Billiards", Russian Math.
 *     Surveys 25(2), 137-189.
 *     — The mathematical foundation of "billiards as dynamical systems"
 *       — including the proof that hard-sphere systems are chaotic,
 *       which is what you watch unfold every reset of this demo.
 *
 *   • Foley, J. D., van Dam, A., Feiner, S. K., Hughes, J. F. (1995)
 *     "Computer Graphics: Principles and Practice" (2nd ed., C ed.),
 *     Addison-Wesley.  §3.3 covers ellipse rasterisation.
 *     — Standard reference for the row-by-row ellipse algorithm
 *       draw_disc uses.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define CELL_W 8
#define CELL_H 16
#define N_DISCS 25

/* R_MIN/R_MAX: disc radius range in pixels.
 * R_MIN = 1.5 cells wide — large enough to be visible on screen.
 * R_MAX = 4 cells wide — at N=25 discs, max radius must not make placement
 * impossible; 4 cells keeps most random layouts placeable within 200 tries. */
#define R_MIN (CELL_W * 1.5f)
#define R_MAX (CELL_W * 4.0f)

/* V_MAX: initial speed cap (px/s).  180 px/s at CELL_W=8 ≈ 22.5 cells/s.
 * At 60 fps that's 0.375 cells per frame — fast enough to look dynamic
 * but slow enough that a disc doesn't skip over another in one tick.      */
#define V_MAX 180.f

/* FLASH_S: impact flash duration (seconds).
 * 0.4 s is just above typical human reaction time (~0.25 s), making
 * collision flashes easy to notice without looking persistent.            */
#define FLASH_S 0.4f

/* SIM_FPS: physics tick rate.  120 Hz gives dt ≈ 8.3 ms — small enough
 * that at V_MAX a disc moves only 1.5 px per tick, preventing tunnelling. */
#define SIM_FPS 120
#define RENDER_NS (1000000000LL / 60) /* 60 fps render period (ns) */

/* Canonical HUD chrome — 1 row top status + 1 row bottom action bar.
 * Discs render in rows TOP_HUD_H .. rows - BOT_HUD_H - 1.            */
#define TOP_HUD_H 1
#define BOT_HUD_H 1
#define HUD_ROWS (TOP_HUD_H + BOT_HUD_H)

#define N_THEMES 10 /* see k_themes[] in §3 */

enum {
  CP_SLOW = 1, /* slow-speed discs   — theme ramp[0]               */
  CP_MED,      /* mid-speed discs    — theme ramp[2]               */
  CP_FAST,     /* fast-speed discs   — theme ramp[3] (brightest)   */
  CP_FLASH,    /* collision flash    — theme `flash` accent        */
  CP_HUD,      /* canonical top status — bright yellow + bold      */
  CP_HINT,     /* canonical bottom action bar — bright cyan + bold */
};

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
/* §3  color — themes + canonical HUD chrome                              */
/* ===================================================================== */

/*
 * Theme — speed ramp + collision flash accent.
 *   ramp[4]    speed tiers dim→bright; CP_SLOW = ramp[0], CP_MED =
 *              ramp[2], CP_FAST = ramp[3] (the brightest tier).
 *   flash      collision-instant highlight (chosen to contrast the
 *              ramp so impacts always pop, even in low-contrast themes).
 *
 * Brightness safety (CLAUDE.md): all entries sit at index ≥ 30, or
 * 24-29 / 240-243 only as the lowest ramp tier.  16-23 / 232-239 are
 * forbidden — they vanish on default-black terminals.
 */
typedef struct {
  const char *name;
  short ramp[4];
  short flash;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*  name        ramp[0..3]                  flash               */
    {"Matrix", {28, 34, 40, 46}, 226},      /* cyber green / yellow pop */
    {"Fire", {130, 208, 202, 196}, 226},    /* warm → red  / gold pop   */
    {"Oceanic", {24, 31, 39, 51}, 196},     /* teal → cyan / red pop    */
    {"Neon", {129, 165, 201, 213}, 51},     /* purple→pink / cyan pop   */
    {"Mono", {240, 247, 250, 255}, 196},    /* grayscale   / red pop    */
    {"Ice", {153, 117, 159, 195}, 196},     /* light blues / red pop    */
    {"Nova", {129, 141, 177, 213}, 226},    /* stellar     / yellow pop */
    {"Forest", {58, 100, 142, 190}, 226},   /* leaves/bark / gold pop   */
    {"Desert", {130, 178, 214, 220}, 51},   /* sand/gold   / cyan pop   */
    {"Eclipse", {240, 244, 124, 196}, 226}, /* gray + red  / yellow pop */
};

/*
 * theme_apply — install the palette for theme index t.
 *
 *   Theme-driven:
 *     CP_SLOW  = ramp[0]   (dim)
 *     CP_MED   = ramp[2]   (skip ramp[1] for more contrast between slow/med)
 *     CP_FAST  = ramp[3]   (brightest tier)
 *     CP_FLASH = flash     (theme-specific accent)
 *
 *   Fixed chrome (theme-independent — always readable):
 *     CP_HUD   bright yellow + bold (top status)
 *     CP_HINT  bright cyan   + bold (bottom action bar)
 */
static void theme_apply(int t) {
  const Theme *th = &k_themes[t % N_THEMES];
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(CP_SLOW, th->ramp[0], -1);
    init_pair(CP_MED, th->ramp[2], -1);
    init_pair(CP_FAST, th->ramp[3], -1);
    init_pair(CP_FLASH, th->flash, -1);
    init_pair(CP_HUD, 226, -1); /* canonical yellow */
    init_pair(CP_HINT, 51, -1); /* canonical cyan   */
  } else {
    /* 8-colour fallback — theme-independent. */
    init_pair(CP_SLOW, COLOR_BLUE, -1);
    init_pair(CP_MED, COLOR_CYAN, -1);
    init_pair(CP_FAST, COLOR_WHITE, -1);
    init_pair(CP_FLASH, COLOR_RED, -1);
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
}

static void color_init(void) {
  start_color();
  theme_apply(0);
}

/* ===================================================================== */
/* §4  physics                                                            */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────
 * Disc — one hard-sphere particle in the billiard pool.
 *
 * The simulation runs in PIXEL space (CELL_W=8, CELL_H=16 px per
 * terminal cell); cell-space coordinates appear only in the renderer.
 * So a disc with r = 24 px occupies 3 cells horizontally and ~1.5 cells
 * vertically (terminal aspect ≈ 2:1 px:px).
 *
 * Members grouped by access pattern, NOT by type — locality + clarity:
 *
 *   kinematic state  : x, y, vx, vy
 *       Hot path.  Written and read every physics step by scene_tick:
 *           1. integrate    x += vx·dt,  y += vy·dt
 *           2. wall bounce  clamp position + sign-flip velocity
 *           3. pair check   resolve overlap + apply elastic impulse
 *
 *   geometry         : r, mass
 *       Set ONCE at scene_init, constant for the disc's lifetime.
 *       mass = r² (area of 2D disc × uniform density), so heavier
 *       discs deflect lighter ones more through the impulse formula
 *           J = 2·m₁·m₂ / (m₁+m₂) · Δv
 *       (Goldstein 2001 §3.10 centre-of-momentum result).
 *
 *   render hook      : flash
 *       The single member that crosses the sim/render boundary.
 *       scene_tick sets it to FLASH_S seconds on collision; each
 *       subsequent tick counts it down.  speed_tier_pair() reads it
 *       to override the speed-band colour with the theme accent.
 *       Sim writes, render reads — kept on Disc instead of a parallel
 *       array so it travels with its owner.
 *
 * References:
 *   • Alder & Wainwright 1959 — the (position, velocity, radius)
 *     hard-sphere representation we use here is identical to theirs.
 *   • Goldstein 2001 §3.10 — physics encoded in the (vx, vy, mass)
 *     fields and the impulse formula in scene_tick.
 * ───────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ─ kinematic state — read+written every scene_tick ─ */
  float x, y;   /* centre position, pixel space (origin top-left) */
  float vx, vy; /* velocity, px/s                                 */

  /* ─ geometry — set once at scene_init, constant after ─ */
  float r;    /* radius, px (range R_MIN..R_MAX)                */
  float mass; /* m = r² (uniform 2D-disc density)               */

  /* ─ render hook — written by sim on collision, read by render ─ */
  float flash; /* seconds remaining of collision flash highlight;
                  > 0 → paint CP_FLASH instead of speed-band tier*/
} Disc;

/* ─────────────────────────────────────────────────────────────────────
 * Scene — the entire demo state.
 *
 * One Scene IS the demo.  Every non-trivial function takes a Scene*
 * (or const Scene* for read-only) and operates on it.  Only the two
 * signal-handler flags in §6 (g_quit, g_resize) remain global, because
 * the C signal API has no context pointer.
 *
 * Members are grouped by what TOUCHES them, separating SIMULATION
 * concerns from RENDERING concerns per the locality principle:
 *
 *   ── SIMULATION ───────────────────────────────────────────────────
 *
 *   particle pool         : discs
 *       Fixed-size pool of N_DISCS hard spheres.  scene_init reseeds
 *       the pool from scratch (radii, velocities, non-overlapping
 *       positions); scene_tick updates every entry each physics step.
 *
 *   wall box              : pw, ph
 *       Pixel-space simulation domain — the inside of the box the
 *       discs bounce against.  Computed from screen dimensions and
 *       the HUD chrome budget at scene_init time.
 *
 *   collision counter     : coll_total
 *       Running tally of pair collisions; incremented by scene_tick.
 *       Read by the HUD as a presentational metric — but lives on the
 *       sim side because the integrator is what writes it.
 *
 *   integrator controls   : paused, speed
 *       User-driven knobs that gate the physics.
 *         paused  → scene_tick early-returns (entire sim frozen).
 *         speed   → sub-divides each frame into 1..8 physics steps;
 *                   higher values give a sharper-looking sim at the
 *                   cost of more O(N²) pair checks per frame.
 *
 *   ── RENDERING ────────────────────────────────────────────────────
 *
 *   screen dimensions     : rows, cols
 *       Cell-space terminal size, refreshed on SIGWINCH.  Used by
 *       scene_init (to compute the wall box from the chrome budget)
 *       AND by every renderer (to clip discs and place the HUD bars).
 *       Read everywhere, written only by scene_init / resize.
 *
 *   palette index         : theme
 *       Active entry in k_themes[].  Pure presentation — t/T cycles
 *       it, theme_apply installs the colour pairs, no simulation
 *       field depends on it.  Lives on Scene only because the HUD
 *       needs to print the name.
 *
 * References:
 *   • Alder & Wainwright 1959 — the particle-pool + pair-check scheme
 *     `discs[]` implements is descended from their hard-sphere MD.
 *   • Goldstein 2001 §3.10 — elastic-collision physics encoded in the
 *     (x, y, vx, vy, r, mass) per-Disc state plus the wall-box bounce.
 *   • Sinai 1970 — the dynamical-systems interpretation of "billiard
 *     in a box" that this Scene implements visually.
 * ───────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── SIMULATION ─────────────────────────────────────────────────
   * Physics state + integrator controls.  Written by scene_tick (and
   * for paused/speed by the key dispatcher); read by scene_tick and
   * by the renderer for presentational purposes only.              */
  Disc discs[N_DISCS];  /* hard-sphere particle pool          */
  float pw, ph;         /* pixel-space wall box (px × px)     */
  long long coll_total; /* running pair-collision tally       */
  bool paused;          /* p   : scene_tick early-returns     */
  int speed;            /* +/- : sub-steps per frame (1..8)   */

  /* ── RENDERING ──────────────────────────────────────────────────
   * Presentation-only state.  Never feeds back into the integrator;
   * mutating these never invalidates physics.                       */
  int rows, cols; /* terminal cell-space size (SIGWINCH)*/
  int theme;      /* t/T : index into k_themes[]        */
} Scene;

/*
 * seed_random_radius_mass — uniform random r ∈ [R_MIN, R_MAX], then
 * derive mass = r² (uniform 2D-disc density).  Heavier discs deflect
 * lighter ones more in the elastic-impulse formula downstream.
 */
static void seed_random_radius_mass(Disc *d) {
  d->r = R_MIN + (float)rand() / RAND_MAX * (R_MAX - R_MIN);
  d->mass = d->r * d->r;
}

/*
 * seed_non_overlapping_position — rejection-sample a position inside
 * the wall box that doesn't intersect any previously-placed disc.
 *
 * For each try: sample a uniform position keeping the full disc inside
 * the walls, then check overlap against discs[0..placed_count-1].
 * Bails after max_tries attempts (placement may fail under dense
 * packing — sim will resolve any residual overlap on the first tick).
 */
static void seed_non_overlapping_position(Disc *d, const Scene *s,
                                          int placed_count, int max_tries) {
  for (int tries = 0; tries < max_tries; tries++) {
    d->x = d->r + (float)rand() / RAND_MAX * (s->pw - 2 * d->r);
    d->y = d->r + (float)rand() / RAND_MAX * (s->ph - 2 * d->r);
    bool ok = true;
    for (int j = 0; j < placed_count && ok; j++) {
      float dx = d->x - s->discs[j].x;
      float dy = d->y - s->discs[j].y;
      float sum = d->r + s->discs[j].r;
      if (dx * dx + dy * dy < sum * sum)
        ok = false;
    }
    if (ok)
      return;
  }
  /* Fall through: keep the last sampled position even if overlapping;
   * scene_tick's positional resolver will separate it next tick.     */
}

/*
 * seed_random_velocity — uniform random direction (0..2π), speed
 * sampled from [60 px/s, V_MAX].  The 60 px/s lower bound prevents
 * "stuck" discs that move imperceptibly on screen.
 */
static void seed_random_velocity(Disc *d) {
  float speed = 60.f + (float)rand() / RAND_MAX * (V_MAX - 60.f);
  float angle = (float)rand() / RAND_MAX * 6.28318f;
  d->vx = speed * cosf(angle);
  d->vy = speed * sinf(angle);
}

/*
 * scene_init — fresh sim state for the given screen size.
 *
 *   1. record screen + recompute the pixel-space wall box
 *   2. reset the collision counter
 *   3. for each disc: geometry → non-overlapping placement → velocity
 *
 * Does NOT touch paused / speed / theme — UI state survives reset and
 * resize so the user's selections stick.
 */
static void scene_init(Scene *s, int cols, int rows) {
  s->rows = rows;
  s->cols = cols;
  s->pw = (float)(cols * CELL_W);
  s->ph = (float)((rows - HUD_ROWS) * CELL_H);
  s->coll_total = 0;

  for (int i = 0; i < N_DISCS; i++) {
    Disc *d = &s->discs[i];
    seed_random_radius_mass(d);
    d->flash = 0.f;
    seed_non_overlapping_position(d, s, i, 200);
    seed_random_velocity(d);
  }
}

/*
 * integrate_disc — explicit-Euler drift step + flash countdown.
 *   x ← x + vx · dt
 *   y ← y + vy · dt
 *   flash ← max(flash − dt, 0)
 * No forces are integrated here — between collisions the discs move in
 * straight lines (Galilean inertia), so a one-step Euler is exact.
 */
static void integrate_disc(Disc *d, float dt) {
  d->x += d->vx * dt;
  d->y += d->vy * dt;
  d->flash -= dt;
  if (d->flash < 0.f)
    d->flash = 0.f;
}

/*
 * reflect_off_walls — elastic wall reflection on the axis-aligned box
 * [0, pw] × [0, ph].  For each side: position-clamp first (so a disc
 * that overshot the wall this tick is pulled back inside), then force
 * the velocity component normal to the violated wall to point inward
 * (sign-flip via fabsf).  Position-clamp + sign-flip is equivalent to
 * a perfectly elastic reflection across the wall plane.
 */
static void reflect_off_walls(Disc *d, float pw, float ph) {
  if (d->x - d->r < 0.f) {
    d->x = d->r;
    d->vx = fabsf(d->vx);
  }
  if (d->x + d->r > pw) {
    d->x = pw - d->r;
    d->vx = -fabsf(d->vx);
  }
  if (d->y - d->r < 0.f) {
    d->y = d->r;
    d->vy = fabsf(d->vy);
  }
  if (d->y + d->r > ph) {
    d->y = ph - d->r;
    d->vy = -fabsf(d->vy);
  }
}

/*
 * pair_overlaps — geometric overlap test for two discs.
 * Returns true iff the discs intersect.  Outputs unit normal n̂ from a
 * to b and the positive penetration depth (min_d − dist).  Skips when
 * centres coincide (dist² < ε) to avoid normal-vector NaN.
 */
static bool pair_overlaps(const Disc *a, const Disc *b, float *out_nx,
                          float *out_ny, float *out_overlap) {
  float dx = b->x - a->x, dy = b->y - a->y;
  float dist2 = dx * dx + dy * dy;
  float min_d = a->r + b->r;
  if (dist2 >= min_d * min_d || dist2 < 1e-6f)
    return false;
  float dist = sqrtf(dist2);
  *out_nx = dx / dist;
  *out_ny = dy / dist;
  *out_overlap = min_d - dist;
  return true;
}

/*
 * resolve_overlap_positional — push the discs apart along n̂ by the
 * penetration depth, mass-weighted so the lighter disc moves more
 * (Baraff-style positional correction).  Conserves the centre of mass.
 *     a ← a − (m_b / (m_a+m_b)) · overlap · n̂
 *     b ← b + (m_a / (m_a+m_b)) · overlap · n̂
 */
static void resolve_overlap_positional(Disc *a, Disc *b, float nx, float ny,
                                       float overlap) {
  float total = a->mass + b->mass;
  a->x -= nx * overlap * b->mass / total;
  a->y -= ny * overlap * b->mass / total;
  b->x += nx * overlap * a->mass / total;
  b->y += ny * overlap * a->mass / total;
}

/*
 * apply_elastic_impulse — exchange momentum along the collision normal.
 *
 *   dv  = (v_a − v_b) · n̂           relative velocity along n̂
 *   J   = 2·m_a·m_b/(m_a+m_b) · dv   impulse magnitude
 *   v_a ← v_a − (J / m_a) · n̂
 *   v_b ← v_b + (J / m_b) · n̂
 *
 * Derived from simultaneous conservation of momentum and kinetic
 * energy (Goldstein 2001 §3.10).  Returns false (no impulse applied)
 * when dv ≤ 0 — the discs are already separating along n̂, which can
 * happen after positional resolution.
 */
static bool apply_elastic_impulse(Disc *a, Disc *b, float nx, float ny) {
  float dv = (a->vx - b->vx) * nx + (a->vy - b->vy) * ny;
  if (dv <= 0.f)
    return false;
  float total = a->mass + b->mass;
  float imp = 2.f * a->mass * b->mass / total * dv;
  a->vx -= imp / a->mass * nx;
  a->vy -= imp / a->mass * ny;
  b->vx += imp / b->mass * nx;
  b->vy += imp / b->mass * ny;
  return true;
}

/*
 * pair_collide — full collision-resolution pass for one disc pair.
 *   1. test overlap
 *   2. positional correction (separate overlapping discs)
 *   3. elastic impulse (skip if already separating)
 *   4. mark both discs as flashing
 *
 * Returns true iff an impulse was actually applied, so the caller can
 * increment the running collision counter.
 */
static bool pair_collide(Disc *a, Disc *b) {
  float nx, ny, overlap;
  if (!pair_overlaps(a, b, &nx, &ny, &overlap))
    return false;
  resolve_overlap_positional(a, b, nx, ny, overlap);
  if (!apply_elastic_impulse(a, b, nx, ny))
    return false;
  a->flash = FLASH_S;
  b->flash = FLASH_S;
  return true;
}

/*
 * scene_tick — one physics step of dt seconds.
 *   1. drift  : integrate position, decay flash
 *   2. walls  : reflect off the box edges (elastic)
 *   3. pairs  : O(N²) collision resolution
 * No-op when s->paused is true.
 */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;

  /* free flight + wall reflections */
  for (int i = 0; i < N_DISCS; i++) {
    integrate_disc(&s->discs[i], dt);
    reflect_off_walls(&s->discs[i], s->pw, s->ph);
  }

  /* O(N²) pair check + collision resolution */
  for (int i = 0; i < N_DISCS - 1; i++) {
    for (int j = i + 1; j < N_DISCS; j++) {
      if (pair_collide(&s->discs[i], &s->discs[j]))
        s->coll_total++;
    }
  }
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static int px_to_cell_x(float px) { return (int)(px / CELL_W + .5f); }
static int px_to_cell_y(float py) {
  return (int)(py / CELL_H + .5f) + TOP_HUD_H;
}

/*
 * disc_to_cell_geometry — pixel-space disc → cell-space render geometry.
 *
 * Centre converted via px_to_cell_*.  Independent rx / ry rounding
 * because terminal cells have a ~2:1 aspect (CELL_H = 2·CELL_W), so
 * the same physical radius gives a wider rx than ry in cells.  Both
 * clamped to a minimum of 1 cell so the disc is always visible.
 */
static void disc_to_cell_geometry(const Disc *d, int *cx, int *cy, int *rx,
                                  int *ry) {
  *cx = px_to_cell_x(d->x);
  *cy = px_to_cell_y(d->y);
  *rx = (int)(d->r / CELL_W + .5f);
  *ry = (int)(d->r / CELL_H + .5f);
  if (*rx < 1)
    *rx = 1;
  if (*ry < 1)
    *ry = 1;
}

/*
 * draw_disc_center — single 'O' at the disc's centre cell, clipped
 * against both top and bottom HUD chrome so discs never paint over
 * the canonical bars.
 */
static void draw_disc_center(int cx, int cy, int cols, int rows) {
  if (cy >= TOP_HUD_H && cy < rows - BOT_HUD_H && cx >= 0 && cx < cols)
    mvaddch(cy, cx, 'O');
}

/*
 * draw_ellipse_rim — ASCII outline of an axis-aligned ellipse via the
 * standard equation
 *
 *     (dc/rx)² + (dr/ry)² = 1     ⇒    dr = ry · √(1 − (dc/rx)²)
 *
 * For each horizontal offset dc ∈ [−rx, rx] we paint two cells (top
 * and bottom) at the corresponding row offsets ±dr.  Glyph picks:
 *   • '|' at the leftmost / rightmost columns and at dc=0 (vertical
 *     tangent points — slope-vertical visually);
 *   • '-' elsewhere (slope-horizontal).
 * All cells clipped against top and bottom HUD chrome.
 *
 * Reference: Foley et al. *Computer Graphics: Principles & Practice*
 * §3.3 — row-by-row ellipse rasterisation.
 */
static void draw_ellipse_rim(int cx, int cy, int rx, int ry, int cols,
                             int rows) {
  for (int dc = -rx; dc <= rx; dc++) {
    int tc = cx + dc;
    if (tc < 0 || tc >= cols)
      continue;
    float frac = 1.f - ((float)(dc * dc)) / ((float)(rx * rx));
    if (frac < 0.f)
      frac = 0.f;
    int dy_cells = (int)(ry * sqrtf(frac));
    for (int sign = -1; sign <= 1; sign += 2) {
      int tr = cy + sign * dy_cells;
      if (tr < TOP_HUD_H || tr >= rows - BOT_HUD_H)
        continue;
      mvaddch(tr, tc, (dc == 0 || dc == -rx || dc == rx) ? '|' : '-');
    }
  }
}

/*
 * draw_disc — paint one disc as `O` at the centre + ASCII ellipse rim.
 * Two passes share the same colour attribute so a single attron/attroff
 * wraps the whole disc.
 */
static void draw_disc(const Disc *d, int cp, int cols, int rows) {
  int cx, cy, rx, ry;
  disc_to_cell_geometry(d, &cx, &cy, &rx, &ry);

  attron(COLOR_PAIR(cp) | A_BOLD);
  draw_disc_center(cx, cy, cols, rows);
  draw_ellipse_rim(cx, cy, rx, ry, cols, rows);
  attroff(COLOR_PAIR(cp) | A_BOLD);
}

/*
 * speed_tier_pair — map a disc's speed (and flash state) to one of the
 * theme-driven colour pairs.  Thresholds at 40 % and 80 % of V_MAX so
 * the three speed bands roughly divide the visible range evenly; the
 * collision flash overrides everything for FLASH_S seconds.
 */
static int speed_tier_pair(const Disc *d) {
  if (d->flash > 0.f)
    return CP_FLASH;
  float speed = sqrtf(d->vx * d->vx + d->vy * d->vy);
  if (speed < V_MAX * 0.4f)
    return CP_SLOW;
  if (speed < V_MAX * 0.8f)
    return CP_MED;
  return CP_FAST;
}

/*
 * draw_hud_top — canonical top status bar (row 0).
 * Right-aligned: disc count, total collisions, sim speed, theme, paused.
 * CP_HUD (bright yellow + bold).
 */
static void draw_hud_top(const Scene *s) {
  char buf[160];
  int n = snprintf(buf, sizeof buf,
                   " N=%d  collisions:%lld  speed:%dx  theme:%s  %s ", N_DISCS,
                   s->coll_total, s->speed, k_themes[s->theme].name,
                   s->paused ? "PAUSED " : "running");
  int col = s->cols - n;
  if (col < 0)
    col = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, col, buf, s->cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/*
 * draw_hud_bottom — canonical bottom action bar (row -1).
 * Left-aligned key list; short fallback if the terminal is narrow.
 * CP_HINT (bright cyan + bold).
 */
static void draw_hud_bottom(const Scene *s) {
  const char *hint_full =
      " q:quit  p:pause  r:reset  t/T:theme  +/-:speed  spc:impulse ";
  const char *hint_short = " q:quit  p:pause  t:theme  spc:impulse ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= s->cols - 1)
    hint = hint_short;

  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(s->rows - 1, 0, hint, s->cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/*
 * scene_draw — paint every disc speed-tinted, then HUD chrome on top.
 */
static void scene_draw(const Scene *s) {
  for (int i = 0; i < N_DISCS; i++) {
    const Disc *d = &s->discs[i];
    draw_disc(d, speed_tier_pair(d), s->cols, s->rows);
  }
  draw_hud_top(s);
  draw_hud_bottom(s);
}

/* ===================================================================== */
/* §6  app                                                                */
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
  color_init();

  /* One Scene owns every piece of mutable state outside the signal flags.
   * UI defaults are set here once; scene_init resets sim state but
   * leaves paused/speed/theme alone, so r/reset and SIGWINCH preserve
   * the user's UI selections.                                          */
  Scene scene = {0};
  scene.speed = 1;
  int rows, cols;
  getmaxyx(stdscr, rows, cols);
  scene_init(&scene, cols, rows);

  long long frame_time = clock_ns();

  while (!g_quit) {

    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, rows, cols);
      scene_init(&scene, cols, rows);
      frame_time = clock_ns();
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
      scene.paused = !scene.paused;
      break;
    case 'r':
    case 'R':
      scene_init(&scene, scene.cols, scene.rows);
      break;
    case '+':
    case '=':
      scene.speed++;
      if (scene.speed > 8)
        scene.speed = 8;
      break;
    case '-':
      scene.speed--;
      if (scene.speed < 1)
        scene.speed = 1;
      break;
    case 't':
      scene.theme = (scene.theme + 1) % N_THEMES;
      theme_apply(scene.theme);
      break;
    case 'T':
      scene.theme = (scene.theme + N_THEMES - 1) % N_THEMES;
      theme_apply(scene.theme);
      break;
    case ' ':
      /* random impulse to every disc */
      for (int i = 0; i < N_DISCS; i++) {
        float a = (float)rand() / RAND_MAX * 6.28318f;
        scene.discs[i].vx += V_MAX * .5f * cosf(a);
        scene.discs[i].vy += V_MAX * .5f * sinf(a);
      }
      break;
    default:
      break;
    }

    long long now = clock_ns();
    long long dt_ns = now - frame_time;
    frame_time = now;
    if (dt_ns > 100000000LL)
      dt_ns = 100000000LL;
    float dt = (float)dt_ns * 1e-9f;

    /* Fixed-rate sub-stepping for stability: each frame's dt is
     * divided into `speed` sub-steps.  scene_tick early-returns when
     * paused, so we can call it unconditionally.                     */
    for (int k = 0; k < scene.speed; k++)
      scene_tick(&scene, dt / scene.speed);

    erase();
    scene_draw(&scene);
    wnoutrefresh(stdscr);
    doupdate();
    clock_sleep_ns(RENDER_NS - (clock_ns() - now));
  }
  return 0;
}
