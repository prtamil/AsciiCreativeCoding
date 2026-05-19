/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * rigid_multiple_bodies.c — 2D Rigid Body Simulation, multi-preset edition
 *                            (translation only, no rotation)
 *
 * Same physics engine as rigid_body.c — AABB-overlap collisions, iterative
 * impulse solver, Baumgarte stabilisation, sleep system — but the initial
 * world layout is one of N_PRESETS canned scenes, cycled with n / N:
 *
 *   1. Brick Wall   — 4×5 grid of small bricks; light bodies that
 *                     scatter dramatically on impact.
 *   2. Beam Stack   — 5 long horizontal beams piled like lumber; heavy
 *                     bodies that topple slowly when the base goes.
 *   3. Tower        — narrow vertical column of cubes; the classic
 *                     "knock it over from the side" demo.
 *   4. Pyramid      — triangular 4-3-2-1 stack; stack-stability test —
 *                     base survives a glancing hit, dead-centre topples.
 *
 * Each preset stages a STRUCTURE on the right side of the world.  The
 * user fires projectile balls in from the left with SPACE — each press
 * spawns one sphere at near-cap horizontal velocity.  Multiple balls
 * can be in flight at once; the only cap is MAX_BODIES.
 *
 * Bodies: cube (AABB rectangle) · sphere (drawn as circle, AABB physics).
 * Implicit floor at the bottom of the screen.  Bodies never penetrate.
 *
 * Why AABB for spheres?
 *   Terminal chars are ~2× taller than wide.  A sphere is drawn as a
 *   circle of visual radius r (cols) × r (rows).  Its physics AABB is
 *   hw=r, hh=2r so the bounding box aligns with the drawing exactly.
 *   A naive circle-distance check using radius r for both axes detected
 *   collision only when bodies were already r rows deep inside each
 *   other — fixed by switching to AABB overlap [4].
 *
 * Collision detection: AABB overlap, minimum-penetration axis [4]
 *   — same function for cube-cube, sphere-sphere, cube-sphere —
 *
 * Resolution — two separate passes per iteration (Box2D pattern [2]):
 *
 *   Pass A — POSITIONAL CORRECTION  (ALWAYS, even when separating)
 *     corr = max(depth - SLOP, 0) * BAUMGARTE / (imA + imB)
 *     Critical: unconditional correction.  An earlier version guarded
 *     this on (vn > 0); bodies that spawned overlapping or shared a
 *     velocity along the contact axis were never separated and fell
 *     through each other.  The Baumgarte feedback term [1] stabilises
 *     the constraint without solving the actual constrained dynamics.
 *
 *   Pass B — VELOCITY IMPULSE  (only when approaching, vn > 0)
 *     j = (1 + e_eff) * vn / (imA + imB)               [3, §3.2]
 *     e_eff = (vn > REST_THRESH) ? e : 0   ← no micro-bounce at rest
 *     Coulomb friction tangent: |jt| ≤ μ·j              [5, §1.5]
 *
 * Floor / walls: full snap + impulse (no fraction — hard boundary).
 *   Runs for ALL bodies (sleeping included) to counter Baumgarte drift.
 *
 * Spawn: 8-attempt random placement, reject if no clear spot found.
 *
 * Sleep: SLEEP_FRAMES quiet frames → frozen.  Woken by impulse > WAKE_IMP
 *        or by positional correction > WAKE_IMP (sleep heuristic [2]).
 *
 * Keys  SPACE fire ball     n/N cycle preset     r reset current preset
 *       c add cube          s add sphere         x remove last
 *       p pause             g gravity            t/T theme
 *       q quit
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra physics/rigid_multiple_bodies.c -o
 * rigid_multiple_bodies -lncurses -lm
 *
 * Sections  §1 config  §2 clock  §3 color  §4 body + scene  §5 framebuf
 *           §6 physics §7 scene  §8 draw   §9 screen §10 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Iterative impulse-based collision resolution [2][3]
 *                  with Baumgarte stabilisation [1].
 *                  SOLVER_ITERS constraint passes per step.  Each pass:
 *                  AABB overlap test [4], compute minimum-penetration
 *                  axis, apply positional correction (Pass A), then
 *                  velocity impulse (Pass B).  More passes → stiffer
 *                  stacks at higher CPU cost; ~10 is a good 2D balance,
 *                  matching Catto's empirical recommendation [2].
 *
 * Physics        : 2D rigid-body mechanics with restitution and friction
 *                  [5, §3.13 + §1.5].
 *                  Restitution (REST_DEF=0.35): e=0 → perfectly inelastic
 *                  (no bounce), e=1 → perfectly elastic (infinite bounce).
 *                  Friction (FRICTION=0.35): Coulomb model |jt| ≤ μ·jn —
 *                  tangential impulse is clamped to μ times the normal
 *                  impulse, so a body resting on the floor cannot be
 *                  slid by a force smaller than μ·m·g.
 *                  REST_THRESH suppresses micro-bounce below 0.20 px/step.
 *
 * Engineering    : SLOP — penetration depth below which no correction
 *                  fires (dead-zone).  Without slop, tiny floating-point
 *                  overlaps cause perpetual jitter [2, "Solver Setup"].
 *                  BAUMGARTE — fraction of penetration corrected each
 *                  iteration [1, eq. 14].  1.0 → sharp but can overshoot;
 *                  0.5 is the value Catto recommends for real-time use [2].
 *                  Sleep system [2, "Sleeping"]: bodies quiet for
 *                  SLEEP_FRAMES frames are frozen; pair-test loop skips
 *                  all-sleeping pairs, saving O(s²) work where s is the
 *                  number of sleeping bodies.
 *
 * Rendering      : 10 brightness-safe theme palettes — 6-stop body ramps
 *                  cycled with t / T.  Bodies are rasterised into a
 *                  cell-grid framebuffer (g_fb / g_fcp in §5) and then
 *                  blitted via fb_flush; this decouples physics-space
 *                  drawing from ncurses I/O and lets fb_put do a single
 *                  bounds check per cell instead of one per primitive.
 *                  HUD chrome (CP_HUD bright yellow, CP_HINT bright cyan)
 *                  is theme-independent so it stays legible everywhere.
 *                  Drawn via ncurses single-buffer + doupdate() diff [6].
 *
 * References (cite inline as [n]):
 *
 *   [1] Baumgarte, J. (1972) — "Stabilization of constraints and
 *       integrals of motion in dynamical systems", *Computer Methods in
 *       Applied Mechanics and Engineering* 1 (1), 1–16.  The original
 *       paper for the feedback term that turns a position constraint
 *       into a stable spring-damper; we use the BAUMGARTE factor in
 *       Pass A.  Eq. 14 gives the standard β / γ split; we use β only
 *       (γ=0) since velocity correction is handled by Pass B's impulse.
 *
 *   [2] Catto, E. (2005, 2006, 2009) — GDC talks "Iterative Dynamics
 *       with Temporal Coherence", "Modeling and Solving Constraints",
 *       and the open-source Box2D engine (https://box2d.org).
 *       Practical reference for the two-pass solver architecture
 *       (positional then velocity), the SLOP / Baumgarte tuning numbers,
 *       the sleep heuristic, and the empirical observation that
 *       ~10 iterations give a good stack-stability vs. cost trade-off
 *       in 2D.  Everything labelled "Box2D pattern" in this file
 *       traces back to these talks.
 *
 *   [3] Mirtich, B. V. (1996) — *Impulse-Based Dynamic Simulation of
 *       Rigid Body Systems*, PhD thesis, U.C. Berkeley.  Foundational
 *       work on impulse-based methods; §3.2 derives the contact impulse
 *       j = (1+e)·v_n / (imA+imB) used in our Pass B and discusses why
 *       impulses are preferable to penalty forces for stiff contacts
 *       (no springs to tune, no integrator stiffness, naturally
 *       single-step convergent for binary contact).
 *
 *   [4] Ericson, C. (2005) — *Real-Time Collision Detection*, Morgan
 *       Kaufmann.  THE reference for game-engine collision math.
 *       §4.2 covers the AABB overlap test, the minimum-translation-
 *       vector concept, and the trick of resolving along the axis of
 *       smaller penetration that col_bodies() uses.  §5.4 covers the
 *       Coulomb friction model we apply in Pass B.
 *
 *   [5] Goldstein, H.; Poole, C.; Safko, J. (2002) — *Classical
 *       Mechanics*, 3rd ed., Addison-Wesley.  §1.5 (constraints,
 *       D'Alembert's principle), §3.13 (elastic / inelastic collisions
 *       and the coefficient of restitution), and Ch. 5 (rigid body
 *       motion).  The textbook backing for "why does e ∈ [0, 1] and
 *       what does e=0.35 mean physically?".
 *
 *   [6] Padala, P. — *NCURSES Programming HOWTO*, The Linux
 *       Documentation Project.  Authoritative free reference for the
 *       rendering layer: init_pair / COLOR_PAIR semantics, the
 *       wnoutrefresh + doupdate diff model that prevents tearing, and
 *       signal-safe SIGWINCH handling.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
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

/* ── world bounds (compile-time) ─────────────────────────────────────── */

/* ROWS_MAX / COLS_MAX — upper bound on terminal geometry we statically
 * pre-allocate the framebuffer for (§5 g_fb / g_fcp).  Sized generously
 * (128×512 cells covers ultrawide monitors at small font sizes); actual
 * drawing is clipped to runtime g_rows / g_cols, so over-provisioning
 * only costs BSS pages, not draw time. */
#define ROWS_MAX 128
#define COLS_MAX 512

/* MAX_BODIES — pool size for Scene.b[].  The pair-wise collision test
 * in scene_step is O(MAX_BODIES²) per iteration × SOLVER_ITERS
 * iterations per step ⇒ ~10 240 col_bodies calls at peak.  Comfortably
 * within a single-frame budget at SIM_FPS=20 Hz; larger values would
 * saturate the screen visually before they saturate CPU. */
#define MAX_BODIES 32

/* N_THEMES — number of palettes in k_themes[] (§3).  Must match the
 * literal array length; the t / T key handlers wrap modulo this. */
#define N_THEMES 10

/* N_PRESETS — number of canned initial layouts (§7).  Must match the
 * length of k_preset_names[] and the Preset enum below; n / N keys
 * wrap modulo this.  Cycling presets calls scene_init(new_preset)
 * which fully rebuilds the body pool. */
#define N_PRESETS 4

/* HUD_TOP_PX — pixel units (one cell-row = 2 px in this sim) reserved
 * at the top of the world for the status bar.  resolve_ceiling (§6)
 * treats this as the ceiling so bodies never draw into row 0, leaving
 * row 0 free for the top HUD overlay.  Row g_rows-1 is similarly
 * reserved for the bottom hint bar, achieved by WH() = (g_rows - 2)*2
 * in §4. */
#define HUD_TOP_PX 2.0f

/* ── physics ─────────────────────────────────────────────────────────── */

/* GRAVITY — downward acceleration per physics step (px / step²).  Acts
 * on vy of awake bodies in scene_step (§6).  Practical terminal
 * velocity is MAX_SPEED (the cap fires before DAMPING-vs-gravity
 * equilibrium); tune higher to make drops snappier, lower for a
 * "lunar" look. */
#define GRAVITY 0.05f

/* REST_DEF — boot default for the restitution coefficient e ∈ [0, 1]
 * (Goldstein [5, §3.13]).  0 = perfectly inelastic (no bounce); 1 =
 * perfectly elastic.  0.35 ≈ rubber ball on concrete — lively but not
 * bouncy.  e > ~0.8 destabilises stacks: each contact injects (1+e)·v_n
 * and the spurious energy accumulates faster than DAMPING removes it. */
#define REST_DEF 0.35f

/* REST_STEP — granularity of the e / E key handlers (one keypress
 * shifts Scene.rest by this).  0.05 gives 19 steps across the legal
 * [0.05, 0.95] range — coarse enough to feel responsive, fine enough
 * to find a sweet spot. */
#define REST_STEP 0.05f

/* FRICTION — Coulomb friction coefficient μ (Ericson [4, §5.4]).
 * apply_coulomb_friction (§6) clamps the tangential impulse to
 * |jt| ≤ μ·jn; resolve_floor uses μ scaled by (1+e_eff) on the slide
 * term so a bouncier collision also leaves more horizontal speed.
 * 0.35 ≈ rubber on wood — bodies coast briefly then stop, not sticky. */
#define FRICTION 0.35f

/* DAMPING — velocity retained per step (dimensionless ∈ [0, 1]).
 * Applied AFTER integrate, BEFORE constraint solve, so it acts on
 * "free flight" velocity only.  0.991^20 ≈ 0.835 / second — bodies
 * slow noticeably across one second, preventing infinite sliding on a
 * frictionless wall.  Lower values look "underwater". */
#define DAMPING 0.991f

/* MAX_SPEED — velocity cap in px / step, applied AFTER damping.
 * Prevents tunnelling: a body moving > ~CUBE_HW per step could cross a
 * thin wall in less than one step and skip the overlap test.  Capping
 * keeps motion below one body-radius per step so SOLVER_ITERS
 * iterations can always converge. */
#define MAX_SPEED 22.0f

/* ── solver  (impulse-based, Baumgarte-stabilised) ──────────────────── */

/* SOLVER_ITERS — constraint solver passes per physics step (Catto
 * [2, "Iteration count"]).  Each pass runs the full O(nb²) col_bodies
 * loop, so wall-clock cost is linear in this constant.  10 is a good
 * 2D balance: stable 3-tall stacks, no visible jitter.  3D rigid-body
 * engines typically use 4–20.  Drop to 4 if profiling shows physics
 * as the hot spot; raise to 20 if stacks feel like jelly. */
#define SOLVER_ITERS 10

/* BAUMGARTE — fraction of penetration resolved per constraint pass
 * (Baumgarte [1, eq. 14]), used in Pass A of col_bodies.  1.0 corrects
 * fully each pass (sharp but can overshoot and pop bodies out of
 * stacks); 0.5 (the value Catto recommends [2]) removes half each
 * pass, so after SOLVER_ITERS=10 passes residual penetration is
 * ~(0.5)^10 ≈ 0.1 % of the original. */
#define BAUMGARTE 0.50f

/* SLOP — penetration depth (px) below which no positional correction
 * fires (dead-zone).  Without slop, tiny float-precision overlaps
 * trigger micro-corrections every frame and bodies visibly jitter.
 * 0.05 px ≈ sub-pixel — absorbs numerical noise without producing
 * visible drift.  Catto [2] uses ~0.005 m in Box2D's SI units; ours
 * is the equivalent in pixel-space. */
#define SLOP 0.05f

/* REST_THRESH — approach speed (px / step) below which e_eff is forced
 * to 0 in Pass B.  Prevents micro-bounce: a body resting on the floor
 * has tiny vy noise from solver residuals; without this gate, every
 * contact would apply a (1+e)·v_n impulse and bodies would vibrate
 * forever.  0.20 px/step ≈ ¼ pixel per frame — well below visible
 * motion but above the solver noise floor. */
#define REST_THRESH 0.20f

/* ── sleep system  (Catto / Box2D [2, "Sleeping"]) ──────────────────── */

/* SLEEP_VEL — speed threshold (px / step) under which Body.sleep_cnt
 * increments each frame.  Above this the body is considered "moving"
 * and sleep_cnt resets to 0.  Chosen LOWER than REST_THRESH so a body
 * that still micro-bounces can't accidentally sleep. */
#define SLEEP_VEL 0.07f

/* SLEEP_FRAMES — consecutive quiet frames before Body.sleeping latches
 * true.  30 frames at SIM_FPS=20 = 1.5 s — long enough that a momen-
 * tarily quiet body in flight doesn't freeze mid-air, short enough
 * that a settled stack actually sleeps within the user's attention
 * span (and frees up the O(nb²) pair loop). */
#define SLEEP_FRAMES 30

/* WAKE_IMP — wake threshold: positional correction or velocity
 * impulse magnitude (px or px/step) above which a sleeping body is
 * forcibly woken via body_wake().  Pegged near SLOP so that any
 * non-noise interaction wakes the sleeper while normal solver drift
 * does not.  Box2D calls this the "wake threshold" in [2]. */
#define WAKE_IMP 0.05f

/* ── body geometry  (px, terminal aspect 1 col : 2 rows) ────────────── */

/* CUBE_HW / CUBE_HH — cube half-extents.  Visual size on screen is
 * (2·HW cols) × (HH cell-rows) because one cell-row = 2 px.  Chosen so
 * the cube reads roughly square on screen (14 cols × 5 rows ≈ visually
 * matched at typical 1:2 font aspect). */
#define CUBE_HW 7.0f
#define CUBE_HH 5.0f

/* SPH_R — sphere visual radius in BOTH cols and screen rows, so the
 * drawn circle is round on screen (not vertically stretched).  Physics
 * AABB uses hw = SPH_R, hh = 2·SPH_R to bound the elliptical pixel-
 * space footprint — see the file header "Why AABB for spheres" and the
 * Body.hw/hh docstring (§4). */
#define SPH_R 4.0f

/* DENSITY — uniform mass-per-AABB-area ρ.  Mass = ρ · (4·hw·hh) at
 * spawn in body_init_mass (§4); Goldstein [5, §1.6] uniform-density
 * rigid-body model.  Value chosen so a typical cube has mass ≈ 1.12
 * and a typical sphere ≈ 1.02 — bodies feel "equivalent" so a sphere
 * doesn't punt a cube across the screen on first contact. */
#define DENSITY 0.008f

/* BRICK_HW / BRICK_HH — half-extents of the small bricks used by the
 * Brick Wall preset (§7).  Sized so a 4×5 wall fits comfortably in
 * 80 cols / 24 rows and so each brick is much lighter than the
 * projectile sphere (DENSITY · 4·3·2 = 0.192 vs sphere's 1.02 — the
 * sphere has ~5× more mass, which is what makes the demo dramatic). */
#define BRICK_HW 3.0f
#define BRICK_HH 2.0f

/* BEAM_HW / BEAM_HH — half-extents of the long beams used by the
 * Beam Stack preset.  Long-and-thin (16 cols × 3 cell-rows visual) so
 * the stack reads as horizontal lumber.  Mass per beam ≈ 0.384, about
 * 2.7× lighter than the projectile — enough that a direct hit topples
 * the bottom beam without the ball stopping dead. */
#define BEAM_HW 8.0f
#define BEAM_HH 1.5f

/* PROJECTILE_VX — horizontal velocity given to the ball when fire_
 * projectile() runs (SPACE key).  Just under MAX_SPEED so the cap
 * doesn't immediately clip it; high enough that DAMPING doesn't
 * drain it before impact (DAMPING^N×PROJECTILE_VX reaches the right
 * edge of an 80-col world in ~5 ticks while still moving fast). */
#define PROJECTILE_VX 18.0f

/* ── timing ──────────────────────────────────────────────────────────── */

/* SIM_FPS — fixed physics rate (Hz).  scene_step runs at most once per
 * (1 s / SIM_FPS) of wall time; the render loop runs uncapped and does
 * NOT interpolate — each frame just draws the latest step's state.
 * 20 Hz is plenty for the visual coarseness of cell-grid rendering. */
#define SIM_FPS 20

/* NS_PER_S — wall-clock conversion factor for clock_ns() / sleep_ns().
 * Long-suffixed (LL) because clock_ns() returns int64_t and 32-bit
 * arithmetic would overflow at ~4.3 seconds. */
#define NS_PER_S 1000000000LL

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_S + t.tv_nsec;
}
static void sleep_ns(int64_t d) {
  if (d <= 0)
    return;
  struct timespec t = {(time_t)(d / NS_PER_S), (long)(d % NS_PER_S)};
  nanosleep(&t, NULL);
}

/* ===================================================================== */
/* §3  color / theme — 10 brightness-safe palettes + fixed HUD chrome    */
/* ===================================================================== */

/*
 * Color-pair layout:
 *   1..6     body slots — theme-driven; cubes/spheres cycle through them
 *            via `1 + (count % 6)`.
 *   CP_FLOOR floor line — theme-independent dim grey
 *   CP_HUD   top status bar (bright yellow + bold) — theme-independent
 *   CP_HINT  bottom action bar (bright cyan + bold) — theme-independent
 *
 * Keeping CP_FLOOR / CP_HUD / CP_HINT outside the theme means the chrome
 * stays legible against every body palette without per-theme tuning.
 */
#define CP_FLOOR 9
#define CP_HUD 10
#define CP_HINT 11

/* ─────────────────────────────────────────────────────────────────────── *
 * Theme — a 6-stop perceptual gradient palette for the body slots.
 *
 * Intent:
 *   A single Theme describes "how the bodies LOOK" — nothing about how
 *   they MOVE.  Cycling themes must be a pure visual change: pause the
 *   demo, press t, and every body's position, velocity, mass, and
 *   sleep state is byte-identical, only the rendered colour shifts.
 *   This is the locality rule that lets us treat the theme index as a
 *   render param on Scene (§4) rather than as physics state.
 *
 * Why 6 stops:
 *   Bodies pick a colour-pair slot via `1 + (count % 6)` so the
 *   palette must contain exactly 6 colours.  Six is enough for visual
 *   distinction without crowding the 8-colour fallback path (which
 *   can only emit COLOR_RED .. COLOR_WHITE anyway).
 *
 * Why ramp[0] → ramp[5] is a monotonic dim→bright gradient:
 *   Adjacent bodies often end up bunched mid-stack; a monotonic
 *   brightness ramp keeps them distinguishable since two consecutive
 *   indices differ in brightness even when their hue is similar.
 *   Arbitrary permutations of the same six colours were tried and
 *   rejected because stacks of "all green" bodies became impossible
 *   to count by eye.
 *
 * Brightness safety (CLAUDE.md, see documentation/COLOR.md):
 *   Every entry sits at 256-cube index ≥ 24.  Indices 16-23 / 232-239
 *   are forbidden — they render as pure background-black on default-bg
 *   terminals and the body simply disappears.
 *
 * Algorithm refs:
 *   256-colour cube structure   — XTerm Control Sequences §
 *                                 "ISO-8613-3 Controls".
 *   Perceptual luminance order  — ITU-R BT.601 luma weighting was
 *                                 the heuristic used when ranking
 *                                 ramp stops by brightness for the
 *                                 audit.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* Human-readable label shown in the top HUD.  ≤ 7 chars so the full
   * status line fits on an 80-column terminal alongside the other
   * fields (cubes:, spheres:, rest:, grav:, paused/running, fps). */
  const char *name;

  /* Six ordered 256-cube indices forming a low→high ramp.
   * ramp[0] = dim end (used by the first-spawned cube/sphere);
   * ramp[5] = bright end (sixth-spawned, then wraps back to ramp[0]
   * on the seventh via modulo).  All entries audited against the
   * brightness-safety bands above. */
  short ramp[6];
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* name        ramp[0..5]                                              */
    {"Matrix", {28, 34, 46, 82, 118, 154}},      /* cyber green       */
    {"Fire", {130, 166, 202, 208, 220, 226}},    /* ember → bright    */
    {"Oceanic", {24, 31, 38, 45, 51, 87}},       /* deep teal → cyan  */
    {"Neon", {93, 129, 165, 201, 207, 213}},     /* purple → hot pink */
    {"Mono", {240, 244, 247, 250, 252, 255}},    /* grayscale         */
    {"Ice", {39, 75, 111, 117, 159, 195}},       /* polar blues       */
    {"Nova", {54, 92, 129, 141, 177, 213}},      /* violet → pink     */
    {"Forest", {58, 64, 100, 106, 142, 184}},    /* leaves            */
    {"Desert", {130, 136, 172, 178, 220, 230}},  /* sand → gold       */
    {"Eclipse", {124, 160, 196, 202, 208, 220}}, /* dark → corona     */
};

/* Fixed chrome — same indices on every theme so the HUD stays legible. */
#define CHROME_FLOOR_256 240 /* dim grey           */
#define CHROME_HUD_256 226   /* bright yellow      */
#define CHROME_HINT_256 51   /* bright cyan        */

/* g_256 is a boot-time TERMINAL CAPABILITY, not scene state — it never
 * changes after screen_init() probes COLORS, so it lives at file scope
 * rather than on g_scene. */
static bool g_256;

/*
 * theme_apply — push the chosen palette into ncurses' colour-pair table.
 * Idempotent; safe to call from t/T handlers and at startup.  ti is
 * wrapped negative-safe so callers don't need to pre-modulo.  Chrome
 * pairs (CP_FLOOR / CP_HUD / CP_HINT) are reinstated every call so a
 * future bug that overwrites them can't outlive one keypress.
 */
static void theme_apply(int ti) {
  const Theme *t = &k_themes[((ti % N_THEMES) + N_THEMES) % N_THEMES];
  for (int i = 0; i < 6; i++) {
    short fg = g_256 ? t->ramp[i] : (short)(COLOR_RED + i % 6);
    init_pair(1 + i, fg, -1);
  }
  init_pair(CP_FLOOR, g_256 ? CHROME_FLOOR_256 : COLOR_WHITE, -1);
  init_pair(CP_HUD, g_256 ? CHROME_HUD_256 : COLOR_YELLOW, -1);
  init_pair(CP_HINT, g_256 ? CHROME_HINT_256 : COLOR_CYAN, -1);
}

/* ===================================================================== */
/* §4  body + scene                                                       */
/* ===================================================================== */

/*
 * Kind — body-shape discriminator.  ONLY consumed by draw_body (§8)
 * to pick rectangle-vs-circle glyphs; the physics step (§6) never
 * reads it because both shapes share the same AABB-overlap path
 * (Ericson [4, §4.2]).  Adding a new shape means: (a) extend this
 * enum, (b) add a draw branch in draw_body, (c) ensure scene_add_*
 * fills hw/hh sensibly — no physics changes needed.
 */
typedef enum { KIND_CUBE = 0, KIND_SPHERE } Kind;

/* ─────────────────────────────────────────────────────────────────────── *
 * Body — one rigid mass-point with AABB extent.
 *
 * Why one struct for cubes AND spheres:
 *   The collision math is AABB-overlap-only [4, §4.2] — one function
 *   handles every pairing (cube-cube, sphere-sphere, cube-sphere).
 *   The visual shape (rectangle vs circle) is purely a §8 draw concern,
 *   gated by `kind`.  Putting cubes and spheres in separate types would
 *   duplicate the entire physics path for no gain.
 *
 * Why no rotation:
 *   This is a translational-rigid-body demo: bodies have no angular
 *   velocity, no angular impulse, no moment of inertia.  Rotation would
 *   need (θ, ω, I⁻¹, tangential cross terms) plus a full polygon
 *   contact solver — see Catto [2] for that path.  Keeping translation-
 *   only keeps the entire physics step under 100 lines while still
 *   showing the iterative impulse + Baumgarte [1] structure.
 *
 * Storage / memory model:
 *   POD struct, ~36 bytes, packed into Scene.b[] in §4.  No pointers,
 *   no ownership — everything in BSS, zero allocation in the hot path.
 *
 * Field groups (in order below):
 *   shape    kind, hw, hh           — geometry, set once at spawn
 *   state    x, y, vx, vy           — integrated every tick (§6)
 *   mass     mass, imass            — derived from area at spawn
 *   render   cp                     — theme-driven colour slot
 *   sleep    sleep_cnt, sleeping    — engine optimisation [2]
 *
 * Algorithm refs:
 *   AABB overlap test               — Ericson [4, §4.2]
 *   Min-translation-vector resolve  — Ericson [4, §4.2]
 *   Impulse-response derivation     — Mirtich [3, §3.2]
 *   Sleep heuristic                 — Catto / Box2D [2, "Sleeping"]
 *   Mass from area (uniform ρ)      — Goldstein [5, §1.6]
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── shape ────────────────────────────────────────────────────────
   * Set once at spawn by scene_add_* (§7); never modified after
   * body_init_mass.  Treat as const after construction.
   */

  /* kind — see Kind docstring above.  Physics treats both values
   * identically; only draw_body (§8) branches on it. */
  Kind kind;

  /* hw, hh — AABB half-extents in pixel-space (one cell-row = 2 px).
   *   cube   : hw = CUBE_HW,  hh = CUBE_HH
   *   sphere : hw = SPH_R,    hh = 2 * SPH_R   ← aspect correction
   * Terminal cells are ~2× taller than wide; a circle drawn with the
   * same visual radius in cols and rows needs twice the y-extent in
   * pixel-space to bound it.  An earlier version used hh = SPH_R and
   * missed every vertical sphere collision until the bodies were
   * already half-overlapping — see the file header "Why AABB for
   * spheres".  Both shapes use the same AABB-overlap test [4]. */
  float hw, hh;

  /* ── kinematic state ──────────────────────────────────────────────
   * Advanced by scene_step's pipeline (§6) in this order:
   *     apply_gravity_to_awake:    vy += GRAVITY     (skipped if sleeping)
   *     integrate_motion:          x += v ; v *= DAMPING ; cap to MAX_SPEED
   *     solve_contact_constraints: col_bodies → Pass A correction + Pass B
   * impulses enforce_world_boundaries:  resolve_floor / _walls / _ceiling
   * snap+reflect
   */

  /* x, y — body centre in pixel-space.  y increases DOWNWARD (screen
   * convention, not maths), so gravity is +y and the floor is the
   * largest y.  prow() / pcol() in §4 collapse this to cell rows /
   * cols for ncurses output. */
  float x, y;

  /* vx, vy — linear velocity in px / tick.  Capped at MAX_SPEED in
   * scene_step to prevent tunnelling — a faster body could cross a
   * 1-cell wall in less than one step and skip the overlap test
   * entirely.  Cap is applied AFTER damping each step so the implicit
   * terminal velocity stays MAX_SPEED · DAMPING / (1 − DAMPING). */
  float vx, vy;

  /* ── mass ─────────────────────────────────────────────────────────
   * imass = 1 / mass is what the solver actually uses; mass itself is
   * kept for any future debug print.  An "infinite mass" body (static
   * wall) would have imass = 0; the solver math already handles it
   * because denom = imA + imB collapses to imA or imB.  Currently
   * unused — every spawned body has finite mass.
   */

  /* mass — derived as (AABB area) × DENSITY in body_init_mass.  Using
   * AABB area means a sphere has the effective mass of its bounding
   * box (≈ 4/π × what a true disc of the same radius would have) — a
   * deliberate simplification: pair-wise behaviour is unaffected
   * since both bodies in any contact compute mass the same way, and
   * the impulse formula j = (1+e)·v_n / (imA+imB) [3, §3.2] is
   * homogeneous in mass under uniform density. */
  float mass;

  /* imass — 1 / mass, the inverse-mass term the solver multiplies by
   * when distributing positional correction (Pass A) and velocity
   * impulse (Pass B).  Stored to avoid a per-tick division in the
   * hot loop, which under SOLVER_ITERS = 10 fires up to
   * 10 · nb · (nb − 1) / 2 times per step. */
  float imass;

  /* ── rendering ────────────────────────────────────────────────────
   * Body has no other render state; the glyph and slope-derived char
   * are computed per frame in draw_body from kind + AABB extents.
   */

  /* cp — ncurses colour-pair ID in [1, 6].  Assigned at spawn from
   * the current theme via `1 + (ncubes % 6)` or `1 + (nsphs % 6)`
   * so a sequence of bodies cycles through the theme ramp.  Survives
   * theme changes: theme_apply() rewrites the SAME pair indices with
   * new RGB, so bodies don't need to be re-tagged on cycle. */
  int cp;

  /* ── sleep system ─────────────────────────────────────────────────
   * Box2D-style sleep heuristic [2, "Sleeping"].  Bodies whose speed
   * stays under SLEEP_VEL for SLEEP_FRAMES consecutive frames are
   * frozen.  The pipeline stages apply_gravity_to_awake, integrate_
   * motion, and update_sleep_state all skip sleeping bodies, and
   * solve_contact_constraints short-circuits any pair where BOTH
   * bodies are sleeping — that's where the real CPU win lives,
   * since the inner loop is O(nb²).
   */

  /* sleep_cnt — quiet-frame counter.  Incremented by update_sleep_state
   * when speed < SLEEP_VEL, reset to 0 on a noisy frame, latched at
   * SLEEP_FRAMES when `sleeping` is set true.  Lives on the body
   * (not in Scene) so each body sleeps independently. */
  int sleep_cnt;

  /* sleeping — when true: apply_gravity_to_awake and integrate_motion
   * skip the body, and solve_contact_constraints skips any pair where
   * both bodies are sleeping.  Cleared by body_wake() whenever a
   * positional correction (baumgarte_correct_pair) or velocity
   * impulse (apply_contact_impulse) exceeds WAKE_IMP — the "wake
   * threshold" of Catto's Box2D [2]. */
  bool sleeping;
} Body;

/* ─────────────────────────────────────────────────────────────────────── *
 * Scene — top-level state spanning physics ticks, draw calls, and the
 * main loop.  Bundles every file-scope mutable that survives across
 * iterations.  Single instance `g_scene`; lives in BSS because the
 * body array (~3 KB) is the only large field and passing the struct
 * by pointer everywhere would just clutter helper signatures without
 * buying any encapsulation.
 *
 * The struct splits into two clearly-labelled groups:
 *
 *   Simulation parameters — consumed by scene_step's pipeline stages
 *     (§6: apply_gravity_to_awake, integrate_motion, solve_contact_
 *     constraints, enforce_world_boundaries, update_sleep_state) and by
 *     scene_add_*() / scene_init() (§7).  Anything that changes WHERE
 *     THE BODIES ARE or HOW THEY MOVE belongs here.  Mutated by physics-
 *     affecting keys: c / s / x (spawn / remove), r (reset),
 *     p / SPACE (pause), g (gravity), e / E (restitution).
 *
 *   Rendering parameters — consumed by theme_apply (§3) and draw_hud_*()
 *     (§8).  Toggling these must leave the body array byte-identical;
 *     only colours change.  Mutated by purely cosmetic keys: t / T.
 *
 * Locality rationale (this contract matters, not the bytes):
 *   The split exists for the READER, not the CPU.  A new flag landing
 *   in the rendering group when it actually changes how scene_step()
 *   advances state would silently couple display to physics — exactly
 *   the bug the separation prevents.  When adding a field, ask: does
 *   this change what any scene_step pipeline stage produces?  If yes,
 *   simulation; if no, rendering.
 *
 * What stays OUTSIDE this struct (intentionally):
 *
 *   g_rows / g_cols       screen geometry tracked by the main loop's
 *                         SIGWINCH handler; Scene stays geometry-
 *                         agnostic so resize logic is the main loop's
 *                         concern, not the simulator's.
 *
 *   g_quit / g_resize     volatile sig_atomic_t flags written by the
 *                         signal handler; must stay at file scope for
 *                         async-signal-safety.
 *
 *   g_256                 boot-time terminal capability set once in
 *                         screen_init(); not state that evolves.
 *
 *   fps_disp / fps_frames / fps_window_start
 *                         rolling-window FPS bookkeeping; lives as
 *                         locals in main() since it has no readers
 *                         outside the loop body.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── Simulation parameters ────────────────────────────────────────
   * Everything below feeds the physics step.  The pipeline stages
   * (§6) read/write b[] and tick; apply_contact_impulse and the
   * resolve_*() helpers read rest; spawners (§7) read rng and the
   * ncubes/nsphs counters.
   */

  /* b[0..nb) — packed array of active bodies.  Insertion appends,
   * deletion by scene_remove_last() just decrements nb (LIFO, no
   * compaction needed).  Capacity is MAX_BODIES; once full, spawn
   * helpers reject new bodies.  The AABB-pair test in scene_step
   * is O(nb²) so MAX_BODIES is set with that quadratic in mind. */
  Body b[MAX_BODIES];
  int nb;

  /* ncubes / nsphs — cumulative spawn counts, used solely to pick a
   * colour-pair slot via `1 + (count % 6)` in scene_add_*.  Decoupled
   * from nb so deleting and respawning still cycles through the
   * palette predictably.  Reset by scene_init. */
  int ncubes, nsphs;

  /* rest — coefficient of restitution e ∈ [0.05, 0.95].  Scales the
   * (1+e_eff) factor in the velocity impulse (Pass B in col_bodies),
   * the floor bounce, and the wall bounce.  Capped below 1.0 because
   * higher values inject energy each contact and destabilise stacks. */
  float rest;

  /* grav — gravity gate.  When false, vy is no longer biased downward
   * in scene_step (§6); existing motion still decays via DAMPING so
   * the bodies coast to rest. */
  bool grav;

  /* paused — when true, main() skips scene_step() entirely.  All body
   * positions / velocities stay frozen, so unpause continues exactly
   * from the last simulated tick — no discontinuity. */
  bool paused;

  /* tick — monotonically increasing physics-step counter, reset by
   * scene_init.  Available for any future "every N ticks" effect; not
   * read by the physics math itself. */
  long tick;

  /* rng — xorshift32 state used by rng_f() for spawn-position jitter
   * and initial-vx randomness.  Re-seeded from time() at boot and at
   * every reset so two runs of the demo differ. */
  uint32_t rng;

  /* preset — which canned initial layout is currently loaded.  Index
   * into k_preset_names[] (§7); also drives the scene_init() switch.
   * Changing it rebuilds the world from scratch — physics state DOES
   * change, hence the field lives in the simulation group rather
   * than the rendering one.  Cycled by n / N. */
  int preset;

  /* ── Rendering parameters ─────────────────────────────────────────
   * Pure cosmetic state.  Mutating these MUST be a no-op for the
   * physics step — that contract is what keeps "press t mid-bounce"
   * from disturbing the simulation.
   */

  /* theme — index into k_themes[] (§3).  Cycled forward by 't',
   * backward by 'T'; both call theme_apply() to push the new ramp
   * into the ncurses colour-pair table.  Always normalised to
   * [0, N_THEMES) by the t / T handlers; theme_apply() still wraps
   * defensively as a safety net for any future caller. */
  int theme;
} Scene;

/*
 * g_scene — the one Scene instance.  Default-initialised here so the
 * file compiles even before scene_init() runs (b[] BSS-zeroed → nb=0
 * → all readers see an empty world).  Non-zero defaults set explicitly:
 *   rest = REST_DEF       sensible bounciness on first frame
 *   grav = true           bodies fall by default
 *   rng  = 0xDEAD1234u    valid seed in case rng_f() is called before
 *                         the time()-based reseed in main / scene_init
 */
static Scene g_scene = {
    .rest = REST_DEF,
    .grav = true,
    .rng = 0xDEAD1234u,
};

/* Screen geometry — main-loop bookkeeping, not scene state
 * (see Scene docstring above). */
static int g_rows, g_cols;

static inline float WW(void) { return (float)g_cols; }

/* World height in pixel units.  Reserves:
 *   row 0           — top HUD (status)            ← via HUD_TOP_PX ceiling
 *   row g_rows - 2  — floor line                  ← prow(WH()) = g_rows-2
 *   row g_rows - 1  — bottom HUD (actions)
 * Game area is therefore rows 1 .. g_rows-3, plus the floor row g_rows-2. */
static inline float WH(void) { return (float)((g_rows - 2) * 2); }

static inline int pcol(float x) { return (int)(x + 0.5f); }
static inline int prow(float y) { return (int)(y * 0.5f + 0.5f); }

/*
 * rng_f — xorshift32 → uniform float in [0, 1).  Stateful via g_scene.rng.
 * Cheap (3 shifts + 3 xors) and good enough for spawn jitter; not a
 * cryptographic RNG.
 */
static float rng_f(void) {
  uint32_t r = g_scene.rng;
  r ^= r << 13;
  r ^= r >> 17;
  r ^= r << 5;
  g_scene.rng = r;
  return (float)(r >> 8) / (float)(1u << 24);
}

static void body_init_mass(Body *b) {
  float area = 4.f * b->hw * b->hh; /* AABB area */
  b->mass = area * DENSITY;
  b->imass = 1.f / b->mass;
}

static void body_wake(Body *b) {
  b->sleeping = false;
  b->sleep_cnt = 0;
}

/* ===================================================================== */
/* §5  framebuffer                                                        */
/* ===================================================================== */

static char g_fb[ROWS_MAX][COLS_MAX];
static int g_fcp[ROWS_MAX][COLS_MAX];

static void fb_clear(void) {
  memset(g_fb, 0, sizeof g_fb);
  memset(g_fcp, 0, sizeof g_fcp);
}
static void fb_put(int r, int c, char ch, int cp) {
  /* Clip against the chrome rows: row 0 (top HUD) and row g_rows-2..-1
   * (floor line + bottom HUD).  Floor and HUDs are painted separately. */
  if (r < 1 || r >= g_rows - 2 || c < 0 || c >= g_cols)
    return;
  g_fb[r][c] = ch;
  g_fcp[r][c] = cp;
}
static void fb_hline(int r, int x0, int x1, char ch, int cp) {
  for (int x = x0; x <= x1; x++)
    fb_put(r, x, ch, cp);
}
static void fb_vline(int c, int y0, int y1, char ch, int cp) {
  for (int y = y0; y <= y1; y++)
    fb_put(y, c, ch, cp);
}

static void fb_flush(void) {
  /* Walk the game area only — rows 1 .. g_rows-3 (floor + HUDs handled
   * separately by the caller).  Range matches the fb_put clip above. */
  for (int r = 1; r < g_rows - 2; r++)
    for (int c = 0; c < g_cols; c++) {
      if (!g_fb[r][c])
        continue;
      attron(COLOR_PAIR(g_fcp[r][c]) | A_BOLD);
      mvaddch(r, c, (chtype)g_fb[r][c]);
      attroff(COLOR_PAIR(g_fcp[r][c]) | A_BOLD);
    }
}

/* ===================================================================== */
/* §6  physics                                                            */
/*                                                                       */
/* scene_step() is a 5-stage pipeline:                                    */
/*                                                                       */
/*     apply_gravity_to_awake();     // a_y += g     for awake bodies     */
/*     integrate_motion();           // x += v ; v *= DAMPING ; |v|≤MAX   */
/*     solve_contact_constraints();  // SOLVER_ITERS × all pairs          */
/*     enforce_world_boundaries();   // floor + 3 walls, hard half-spaces */
/*     update_sleep_state();         // sleep counter + freeze            */
/*                                                                       */
/* Each stage is its own helper named for the physics / algorithm        */
/* concept it implements.  Pair resolution (col_bodies) and per-body     */
/* boundary handling (resolve_*) decompose further into smaller          */
/* primitives — aabb_contact, baumgarte_correct_pair, apply_contact_     */
/* impulse, apply_coulomb_friction — so the math is readable top-down.   */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Contact — outcome of an AABB-pair overlap test.
 *
 * (nx, ny) is the contact NORMAL, conventionally pointing from a TOWARD
 * b — equivalently the direction of the Minimum Translation Vector
 * (MTV).  For axis-aligned boxes the normal is always one of ±x̂ or ±ŷ,
 * so exactly one of nx / ny is non-zero.  `depth` is the positive
 * penetration to resolve along that normal.  `overlapping` distinguishes
 * "no contact" from "contact with zero depth" (defensive — the latter
 * never happens in this solver but keeps callers honest).
 *
 * Inspired by the "Contact Manifold" of Catto's Box2D [2]; we don't
 * store contact POINTS because translation-only physics needs only the
 * normal + depth — there are no torques about a contact point.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  float nx, ny; /* MTV direction, points a → b (unit vector)        */
  float depth;  /* penetration along (nx, ny), ≥ 0 when overlapping */
  bool overlapping;
} Contact;

/*
 * aabb_contact — test two AABBs for overlap and, if overlapping, return
 * the Minimum Translation Vector (Ericson [4, §4.2]).
 *
 * Algorithm:
 *   1. Per-axis overlap = (sum of half-widths) − |Δcenter|.
 *      A non-positive value on either axis ⇒ no contact, return early.
 *   2. If both axes overlap, the MTV runs along the axis of SMALLER
 *      overlap (that's the shortest push out of the contact).
 *   3. Normal sign points from a's centre toward b's centre on that
 *      axis — the convention every helper below relies on.
 */
static Contact aabb_contact(const Body *a, const Body *b) {
  Contact c = {0};
  float ox = (a->hw + b->hw) - fabsf(b->x - a->x);
  float oy = (a->hh + b->hh) - fabsf(b->y - a->y);
  if (ox <= 0.f || oy <= 0.f)
    return c; /* no overlap */

  c.overlapping = true;
  if (ox < oy) { /* x axis is the shorter MTV */
    c.nx = (b->x > a->x) ? 1.f : -1.f;
    c.ny = 0.f;
    c.depth = ox;
  } else { /* y axis is shorter */
    c.nx = 0.f;
    c.ny = (b->y > a->y) ? 1.f : -1.f;
    c.depth = oy;
  }
  return c;
}

/* Relative velocity of a w.r.t. b projected onto the contact normal:
 *   v_n = (v_a − v_b) · n
 * Positive ⇒ a is approaching b; Pass B applies a bounce only then. */
static inline float relative_normal_velocity(const Body *a, const Body *b,
                                             float nx, float ny) {
  return (a->vx - b->vx) * nx + (a->vy - b->vy) * ny;
}

/*
 * baumgarte_correct_pair — Pass A: positional stabilisation
 * (Baumgarte [1, eq. 14]).
 *
 *   corr = max(depth − SLOP, 0) · BAUMGARTE / (imA + imB)
 *
 * Distributes the push by inverse mass so a heavy body barely moves
 * while a light one is shoved out.  Runs ALWAYS — even when bodies are
 * separating — so a pair that spawned overlapping is pushed apart
 * regardless of relative velocity.  SLOP is the dead-zone that absorbs
 * solver noise [2, "Solver Setup"]; a correction exceeding WAKE_IMP
 * wakes the affected body so a nudged sleeping pile comes back to life.
 */
static void baumgarte_correct_pair(Body *a, Body *b, const Contact *c) {
  float denom = a->imass + b->imass;
  if (denom < 1e-12f)
    return; /* both infinite-mass */

  float corr = fmaxf(c->depth - SLOP, 0.f) * BAUMGARTE / denom;
  if (corr <= 0.f)
    return; /* inside the slop dead-zone */

  float ca = corr * a->imass;
  float cb = corr * b->imass;
  a->x -= c->nx * ca;
  a->y -= c->ny * ca;
  b->x += c->nx * cb;
  b->y += c->ny * cb;
  if (ca > WAKE_IMP)
    body_wake(a);
  if (cb > WAKE_IMP)
    body_wake(b);
}

/*
 * apply_contact_impulse — Pass B normal: Newton's contact impulse
 * (Mirtich [3, §3.2]).
 *
 *   j_n = (1 + e_eff) · v_n / (imA + imB)
 *   e_eff = (v_n > REST_THRESH) ? g_scene.rest : 0
 *
 * The adaptive restitution gate suppresses micro-bounce: a body
 * resting on another has tiny solver residual vn; without the gate it
 * would apply a (1+e)·vn impulse every step and vibrate forever.
 *
 * Caller has already screened for v_n > 0; this helper does not.
 * Returns j_n so the friction helper can clamp |j_t| ≤ μ · j_n.
 */
static float apply_contact_impulse(Body *a, Body *b, float nx, float ny,
                                   float vn) {
  float denom = a->imass + b->imass;
  float e_eff = (vn > REST_THRESH) ? g_scene.rest : 0.f;
  float jn = (1.f + e_eff) * vn / denom;
  if (jn > WAKE_IMP) {
    body_wake(a);
    body_wake(b);
  }

  a->vx -= nx * jn * a->imass;
  a->vy -= ny * jn * a->imass;
  b->vx += nx * jn * b->imass;
  b->vy += ny * jn * b->imass;
  return jn;
}

/*
 * apply_coulomb_friction — Pass B tangent: Coulomb's law of dry
 * friction (Ericson [4, §5.4]).
 *
 *   t        = (−ny, nx)             // 90° CCW from contact normal
 *   v_t      = (v_a − v_b) · t       // relative tangential velocity
 *   j_t      = −v_t / (imA + imB)    // impulse to ZERO tangential vel
 *   |j_t|   ≤ μ · j_n                // Coulomb clamp
 *
 * v_t is re-derived from the POST-normal-impulse velocities (the
 * "sequential impulse" pattern of Box2D [2]); friction acts on the
 * already-corrected velocity rather than the pre-impulse one.
 */
static void apply_coulomb_friction(Body *a, Body *b, float nx, float ny,
                                   float jn) {
  float denom = a->imass + b->imass;
  float tx = -ny, ty = nx;
  float vt = (a->vx - b->vx) * tx + (a->vy - b->vy) * ty;
  float jt = -vt / denom;
  float jt_max = FRICTION * jn;
  if (jt > jt_max)
    jt = jt_max;
  if (jt < -jt_max)
    jt = -jt_max;

  a->vx += tx * jt * a->imass;
  a->vy += ty * jt * a->imass;
  b->vx -= tx * jt * b->imass;
  b->vy -= ty * jt * b->imass;
}

/*
 * col_bodies — resolve one body pair (the per-pair primitive called
 * O(nb²) × SOLVER_ITERS times per step).  Reads as pseudocode:
 *
 *   1. AABB overlap test                                          [4]
 *   2. Pass A — Baumgarte positional correction (always)          [1]
 *   3. relative normal velocity
 *   4. if separating (v_n ≤ 0) → done (Pass B is bounce-only)
 *   5. Pass B normal  — contact impulse along n                   [3]
 *   6. Pass B tangent — Coulomb friction along t                  [4]
 *
 * Pass A is intentionally unconditional — see baumgarte_correct_pair.
 */
static void col_bodies(Body *a, Body *b) {
  Contact c = aabb_contact(a, b);
  if (!c.overlapping)
    return;

  baumgarte_correct_pair(a, b, &c);

  float vn = relative_normal_velocity(a, b, c.nx, c.ny);
  if (vn <= 0.f)
    return; /* separating — skip Pass B */

  float jn = apply_contact_impulse(a, b, c.nx, c.ny, vn);
  apply_coulomb_friction(a, b, c.nx, c.ny, jn);
}

/* ─────────────────────────────────────────────────────────────────────── *
 * World-boundary constraints (floor + 3 walls).
 *
 * Each surface is a hard half-space: position is SNAPPED (no Baumgarte
 * fraction), then the normal velocity component is reflected with
 * adaptive restitution.  Hard rather than iterative because the
 * boundaries are static infinite-mass surfaces — there is no other
 * side to push back on, so an exact clamp is both stable and cheaper.
 *
 * Only resolve_floor adds dynamic friction (vx · (1 − μ(1+e_eff))) —
 * the floor is the only surface bodies actually slide along long-term;
 * adding friction to the walls / ceiling would make the demo feel
 * sticky on brief side contacts.
 * ─────────────────────────────────────────────────────────────────────── */

/* resolve_floor — half-space y + hh ≤ WH().  Snap + reflect + friction. */
static void resolve_floor(Body *b) {
  float wh = WH();
  if (b->y + b->hh <= wh)
    return;
  b->y = wh - b->hh; /* exact snap to floor */

  if (b->vy <= 0.f)
    return; /* already moving upward */
  float e_eff = (b->vy > REST_THRESH) ? g_scene.rest : 0.f;
  b->vy = -b->vy * e_eff;
  b->vx *= (1.f - FRICTION * (1.f + e_eff)); /* dynamic slide friction */
  if (fabsf(b->vx) < SLEEP_VEL)
    b->vx = 0.f; /* kill near-zero drift */
}

/* resolve_left_wall — half-space x − hw ≥ 0. */
static void resolve_left_wall(Body *b) {
  if (b->x - b->hw >= 0.f)
    return;
  b->x = b->hw;
  if (b->vx >= 0.f)
    return; /* already moving right */
  float e_eff = (fabsf(b->vx) > REST_THRESH) ? g_scene.rest : 0.f;
  b->vx = -b->vx * e_eff;
}

/* resolve_right_wall — half-space x + hw ≤ WW().  Mirror of the left. */
static void resolve_right_wall(Body *b) {
  float ww = WW();
  if (b->x + b->hw <= ww)
    return;
  b->x = ww - b->hw;
  if (b->vx <= 0.f)
    return;
  float e_eff = (fabsf(b->vx) > REST_THRESH) ? g_scene.rest : 0.f;
  b->vx = -b->vx * e_eff;
}

/* resolve_ceiling — half-space y − hh ≥ HUD_TOP_PX.  The ceiling sits
 * at HUD_TOP_PX (not 0) so row 0 stays clear for the top HUD overlay. */
static void resolve_ceiling(Body *b) {
  if (b->y - b->hh >= HUD_TOP_PX)
    return;
  b->y = b->hh + HUD_TOP_PX;
  if (b->vy >= 0.f)
    return;
  float e_eff = (fabsf(b->vy) > REST_THRESH) ? g_scene.rest : 0.f;
  b->vy = -b->vy * e_eff;
}

/* ─────────────────────────────────────────────────────────────────────── *
 * scene_step pipeline — five named stages.
 * ─────────────────────────────────────────────────────────────────────── */

/*
 * apply_gravity_to_awake — bias vy by GRAVITY for every awake body.
 * Sleeping bodies are skipped: they have vy = 0 and adding gravity
 * would immediately re-wake them on the next sleep check.  Gated by
 * g_scene.grav so the user can suspend gravity at runtime (g key).
 */
static void apply_gravity_to_awake(void) {
  if (!g_scene.grav)
    return;
  for (int i = 0; i < g_scene.nb; i++)
    if (!g_scene.b[i].sleeping)
      g_scene.b[i].vy += GRAVITY;
}

/*
 * integrate_motion — explicit Euler position update, followed by linear
 * damping and the MAX_SPEED tunnelling-prevention cap.
 *
 *   x += vx ; y += vy            (explicit Euler position step)
 *   v  *= DAMPING                (linear drag analogue)
 *   |v|  clamped to MAX_SPEED    (so |v|·dt < CUBE_HW)
 *
 * Order matters: damping AFTER position means v is the "free flight"
 * velocity before any contact impulses; capping AFTER damping pins
 * terminal velocity at MAX_SPEED rather than MAX_SPEED · DAMPING.
 * Sleeping bodies are skipped.
 */
static void integrate_motion(void) {
  for (int i = 0; i < g_scene.nb; i++) {
    Body *b = &g_scene.b[i];
    if (b->sleeping)
      continue;

    b->x += b->vx;
    b->y += b->vy;
    b->vx *= DAMPING;
    b->vy *= DAMPING;

    float spd = sqrtf(b->vx * b->vx + b->vy * b->vy);
    if (spd > MAX_SPEED) {
      float s = MAX_SPEED / spd;
      b->vx *= s;
      b->vy *= s;
    }
  }
}

/*
 * solve_contact_constraints — SOLVER_ITERS iterations of the all-pairs
 * collision loop, each iteration calling col_bodies on every (i, j)
 * pair with i < j.
 *
 * The outer iteration is what gives the solver its "iterative" name
 * (Catto [2]): a single pass would resolve one contact at a time, but
 * stacks need each contact to RE-resolve as bodies above shift the
 * bodies below.  Sleeping pairs are skipped — the entire CPU win of
 * the sleep system, since the inner loop is O(nb²).
 */
static void solve_contact_constraints(void) {
  for (int iter = 0; iter < SOLVER_ITERS; iter++) {
    for (int i = 0; i < g_scene.nb; i++) {
      for (int j = i + 1; j < g_scene.nb; j++) {
        Body *a = &g_scene.b[i];
        Body *bj = &g_scene.b[j];
        if (a->sleeping && bj->sleeping)
          continue;
        col_bodies(a, bj);
      }
    }
  }
}

/*
 * enforce_world_boundaries — apply the four static half-space
 * constraints (floor, ceiling, two walls) to every body, sleeping
 * INCLUDED.  We process sleeping bodies because Baumgarte drift in a
 * tall stack can otherwise nudge a sleeper through the floor across
 * many ticks; the snap keeps geometry honest.
 */
static void enforce_world_boundaries(void) {
  for (int i = 0; i < g_scene.nb; i++) {
    Body *b = &g_scene.b[i];
    resolve_floor(b);
    resolve_left_wall(b);
    resolve_right_wall(b);
    resolve_ceiling(b);
  }
}

/*
 * update_sleep_state — Box2D-style sleep heuristic [2, "Sleeping"].
 *
 *   speed < SLEEP_VEL      →  sleep_cnt++
 *   sleep_cnt ≥ SLEEP_FRAMES  →  body is frozen (v zeroed, sleeping=true)
 *   any noisier frame      →  sleep_cnt reset to 0
 *
 * Wake-up is handled by body_wake() called from baumgarte_correct_pair,
 * apply_contact_impulse, and (implicitly) by external resets.
 */
static void update_sleep_state(void) {
  for (int i = 0; i < g_scene.nb; i++) {
    Body *b = &g_scene.b[i];
    if (b->sleeping)
      continue;

    float spd = sqrtf(b->vx * b->vx + b->vy * b->vy);
    if (spd < SLEEP_VEL) {
      if (++b->sleep_cnt >= SLEEP_FRAMES) {
        b->vx = b->vy = 0.f;
        b->sleeping = true;
      }
    } else {
      b->sleep_cnt = 0;
    }
  }
}

/*
 * scene_step — one simulation tick.  Reads as the pipeline described
 * at the top of §6: each line is an independently-named stage whose
 * helper above implements one well-defined physics / algorithm concept.
 */
static void scene_step(void) {
  apply_gravity_to_awake();
  integrate_motion();
  solve_contact_constraints();
  enforce_world_boundaries();
  update_sleep_state();
  g_scene.tick++;
}

/* ===================================================================== */
/* §7  scene management                                                   */
/* ===================================================================== */

/*
 * aabb_overlaps_any — spawn-time overlap test: does candidate `c`
 * collide with any body already in the pool?  Used by try_place_at_top
 * to reject occupied spawn positions.  O(nb), runs ≤ 8 times per
 * spawn so the worst case is negligible compared to the physics step.
 */
static bool aabb_overlaps_any(const Body *c) {
  for (int i = 0; i < g_scene.nb; i++) {
    const Body *b = &g_scene.b[i];
    if ((c->hw + b->hw) > fabsf(c->x - b->x) &&
        (c->hh + b->hh) > fabsf(c->y - b->y))
      return true;
  }
  return false;
}

/*
 * body_init_shape — set the immutable shape / colour fields of a Body
 * and derive its mass from AABB area.  After this call b has valid
 * kind / hw / hh / cp / mass / imass but x / y / vx / vy are still
 * zero — those are placed by try_place_at_top.
 */
static void body_init_shape(Body *b, Kind kind, float hw, float hh, int cp) {
  b->kind = kind;
  b->hw = hw;
  b->hh = hh;
  b->cp = cp;
  body_init_mass(b);
}

/*
 * try_place_at_top — find a free spawn position along the ceiling row
 * (y = hh + HUD_TOP_PX) using rejection sampling.  Up to `attempts`
 * uniformly-random x values are tried in the middle 80 % of the world
 * width with a 1 px wall margin; first non-overlapping candidate wins.
 * Returns true on success (b->x, b->y written) or false if every
 * attempt overlapped — caller drops the spawn in that case.
 */
static bool try_place_at_top(Body *b, int attempts) {
  float ww = WW();
  for (int i = 0; i < attempts; i++) {
    float x = ww * (0.10f + rng_f() * 0.80f);
    if (x < b->hw + 1.f)
      x = b->hw + 1.f;
    if (x > ww - b->hw - 1.f)
      x = ww - b->hw - 1.f;
    b->x = x;
    b->y = b->hh + HUD_TOP_PX;
    if (!aabb_overlaps_any(b))
      return true;
  }
  return false;
}

/*
 * scene_add_cube — drop a new cube at the ceiling with mild horizontal
 * jitter.  Reads as pseudocode:
 *
 *   1. reject if pool is full
 *   2. build a fresh cube body (shape + mass + colour slot)
 *   3. find a free spawn position (≤ 8 attempts)
 *   4. give it gentle horizontal velocity
 *   5. push it onto Scene.b[]
 */
static bool scene_add_cube(void) {
  if (g_scene.nb >= MAX_BODIES)
    return false;

  Body b = {0};
  body_init_shape(&b, KIND_CUBE, CUBE_HW, CUBE_HH, 1 + (g_scene.ncubes % 6));
  if (!try_place_at_top(&b, 8))
    return false;

  b.vx = (rng_f() - 0.5f) * 2.f; /* ±1 px/step jitter */
  g_scene.b[g_scene.nb++] = b;
  g_scene.ncubes++;
  return true;
}

/*
 * scene_add_sphere — mirror of scene_add_cube; the only differences are
 * the shape kind, the aspect-corrected AABB extent (hh = 2 · SPH_R),
 * and the colour-pair counter (g_scene.nsphs).
 */
static bool scene_add_sphere(void) {
  if (g_scene.nb >= MAX_BODIES)
    return false;

  Body b = {0};
  body_init_shape(&b, KIND_SPHERE, SPH_R, 2.f * SPH_R, 1 + (g_scene.nsphs % 6));
  if (!try_place_at_top(&b, 8))
    return false;

  b.vx = (rng_f() - 0.5f) * 2.f;
  g_scene.b[g_scene.nb++] = b;
  g_scene.nsphs++;
  return true;
}

static void scene_remove_last(void) {
  if (g_scene.nb > 0)
    g_scene.nb--;
}

/* ─────────────────────────────────────────────────────────────────────── *
 * Preset infrastructure.
 *
 * A "preset" is a deterministic initial body layout.  Each preset_*()
 * builder appends bodies to a freshly-cleared g_scene.b[] using the
 * scene_place_body() helper below; no preset is supposed to mutate
 * solver constants (rest, grav) — those stay user-tunable across
 * preset cycles so the demo state survives n / N.
 *
 * Adding a new preset:
 *   1. extend the Preset enum + bump N_PRESETS in §1
 *   2. add an entry to k_preset_names[]
 *   3. write preset_<name>() that calls scene_place_body() in a loop
 *   4. add a switch arm in scene_init()
 * ─────────────────────────────────────────────────────────────────────── */
typedef enum {
  PRESET_BRICK_WALL = 0, /* 4×5 grid of small light bricks      */
  PRESET_BEAMS,          /* 5-tall pile of long horizontal beams */
  PRESET_TOWER,          /* narrow vertical column of cubes      */
  PRESET_PYRAMID,        /* 4-3-2-1 triangular stack             */
} Preset;

static const char *k_preset_names[N_PRESETS] = {
    "Brick Wall",
    "Beam Stack",
    "Tower",
    "Pyramid",
};

/*
 * scene_place_body — append a fully-specified body to g_scene.b[].
 * Returns silently on a full pool (caller should size presets to fit
 * MAX_BODIES, but truncation is preferable to OOB writes).  Also bumps
 * ncubes / nsphs so the colour-pair cycle in scene_add_*() stays in
 * sync after a preset reload.
 */
static void scene_place_body(Kind kind, float x, float y, float hw, float hh,
                             int cp, float vx, float vy, bool start_sleeping) {
  if (g_scene.nb >= MAX_BODIES)
    return;
  Body b = {0};
  body_init_shape(&b, kind, hw, hh, cp);
  b.x = x;
  b.y = y;
  b.vx = vx;
  b.vy = vy;
  b.sleeping = start_sleeping;
  g_scene.b[g_scene.nb++] = b;
  if (kind == KIND_CUBE)
    g_scene.ncubes++;
  else
    g_scene.nsphs++;
}

/*
 * fire_projectile — spawn a ball at the left edge of the world with
 * PROJECTILE_VX horizontal velocity, aimed across the world toward
 * whatever structure the current preset has staged on the right.
 * Bound to SPACE in main; pressing repeatedly stacks balls in flight
 * (capped only by MAX_BODIES).  Returns silently on a full pool.
 *
 * Spawn position — CRITICAL for the demo to work:
 *
 *   x = 2·SPH_R           one sphere-diameter inside the left wall
 *                          (clear of resolve_left_wall's snap)
 *   y = WH() − 4·SPH_R    chest height — one ball-DIAMETER above the
 *                          floor, NOT one ball-radius
 *
 * Why chest height and not floor level:
 *   resolve_floor (§6) applies a slide-friction multiplier of
 *   (1 − μ(1+e_eff)) = (1 − 0.35·1) = 0.65 to vx every time the ball
 *   touches the floor.  A ball spawned at y = wh − 2·SPH_R has its
 *   bottom exactly at the floor — gravity immediately pulls it into
 *   the friction band and vx decays by ~35 %/tick (compounding), so
 *   after only ~5 ticks (≈ 250 ms) the ball is essentially stopped
 *   and never reaches the structure ≈ 70 % of the world away.
 *
 *   Spawning at y = wh − 4·SPH_R puts the ball one full diameter above
 *   the floor.  Gravity makes it arc downward, but it takes ~18 ticks
 *   to fall to the floor, by which time the projectile has already
 *   crossed the world (it covers ~80 px in ~5 ticks at PROJECTILE_VX
 *   with DAMPING).  Impact lands at the structure's middle/upper rows
 *   — visually more dramatic than a floor-grazing shot.
 */
static void fire_projectile(void) {
  float wh = WH();
  scene_place_body(KIND_SPHERE, 2.f * SPH_R, wh - 4.f * SPH_R, SPH_R,
                   2.f * SPH_R, 1 + (g_scene.nsphs % 6), PROJECTILE_VX, 0.f,
                   false);
}

/*
 * preset_brick_wall — 4 × 5 wall of small bricks on the right.  No
 * projectile is auto-fired; the user is expected to press SPACE.
 *
 * Layout:
 *   - bricks on a cell-grid centred on wall_cx, stacked from floor up
 *   - small inter-brick gap (0.2 px) so AABB overlap doesn't trigger
 *     on tick 0 — Pass A would correct it but a clean start reads
 *     better visually
 *   - all bricks start sleeping; the first projectile wakes them
 *
 * Mass ratio sphere : brick ≈ 5 : 1 — what makes the wall scatter.
 */
static void preset_brick_wall(void) {
  float ww = WW(), wh = WH();

  const int wall_cols = 4;
  const int wall_rows = 5;
  const float gap = 0.2f;
  const float pitch_x = 2.f * BRICK_HW + gap;
  const float pitch_y = 2.f * BRICK_HH + gap;
  const float wall_cx = ww * 0.78f;
  const float wall_bot_y = wh - BRICK_HH;

  for (int r = 0; r < wall_rows; r++) {
    for (int c = 0; c < wall_cols; c++) {
      float x = wall_cx + (c - (wall_cols - 1) / 2.f) * pitch_x;
      float y = wall_bot_y - r * pitch_y;
      int cp = 1 + ((r * wall_cols + c) % 6);
      scene_place_body(KIND_CUBE, x, y, BRICK_HW, BRICK_HH, cp, 0.f, 0.f, true);
    }
  }
}

/*
 * preset_beams — 5 long horizontal beams stacked on top of each other,
 * lumber-pile style.  Beams are wider (BEAM_HW=8) but thinner (BEAM_HH
 * =1.5) than bricks; the heavy beams topple together as a unit rather
 * than scatter individually, giving a very different visual to the
 * Brick Wall preset.
 *
 * Stack sits at x = 75 % of the world width.  Aligned (not staggered
 * jenga-style) — a single deep hit at the base sends the whole pile
 * sideways.
 */
static void preset_beams(void) {
  float ww = WW(), wh = WH();
  const int beam_count = 5;
  const float gap = 0.2f;
  const float pitch_y = 2.f * BEAM_HH + gap;
  const float stack_cx = ww * 0.75f;

  for (int r = 0; r < beam_count; r++) {
    scene_place_body(KIND_CUBE, stack_cx, wh - BEAM_HH - r * pitch_y, BEAM_HW,
                     BEAM_HH, 1 + (r % 6), 0.f, 0.f, true);
  }
}

/*
 * preset_tower — narrow vertical column of full-size cubes at 75 % of
 * the world width.  A SPACE-fired ball travelling along the floor hits
 * the bottom cube and the whole tower topples sideways (the classic
 * "kick the foundation" test).  Height capped at 6 cubes so the top
 * stays clear of the ceiling on a 24-row terminal.
 */
static void preset_tower(void) {
  float ww = WW(), wh = WH();
  const int rows = 6;
  const float gap = 0.1f;
  const float pitch_y = 2.f * CUBE_HH + gap;

  for (int r = 0; r < rows; r++) {
    scene_place_body(KIND_CUBE, ww * 0.75f, wh - CUBE_HH - r * pitch_y, CUBE_HW,
                     CUBE_HH, 1 + (r % 6), 0.f, 0.f, true);
  }
}

/*
 * preset_pyramid — triangular 4-3-2-1 stack (10 cubes) sitting at 70 %
 * of world width on the floor.  Stack-stability test: a well-tuned
 * solver keeps the pyramid standing under gravity; a SPACE-fired ball
 * hitting the base scatters the bottom row first, the upper layers
 * tumble after as their supports disappear.
 */
static void preset_pyramid(void) {
  float ww = WW(), wh = WH();
  const int base_rows = 4; /* 4 + 3 + 2 + 1 = 10 cubes */
  const float gap = 0.2f;
  const float pitch_x = 2.f * CUBE_HW + gap;
  const float pitch_y = 2.f * CUBE_HH + gap;
  const float pyramid_cx = ww * 0.70f;

  int idx = 0;
  for (int r = 0; r < base_rows; r++) {
    int n = base_rows - r;
    for (int c = 0; c < n; c++) {
      float x = pyramid_cx + (c - (n - 1) / 2.f) * pitch_x;
      float y = wh - CUBE_HH - r * pitch_y;
      scene_place_body(KIND_CUBE, x, y, CUBE_HW, CUBE_HH, 1 + (idx++ % 6), 0.f,
                       0.f, true);
    }
  }
}

/*
 * scene_init — dispatcher.  Clears the body pool, reseeds the RNG,
 * normalises the preset index, then calls the matching preset_*()
 * builder.  Called at boot, on r/R reset, on SIGWINCH (rebuilds the
 * world for the new geometry), and on every n / N press.
 */
static void scene_init(int preset) {
  g_scene.nb = g_scene.ncubes = g_scene.nsphs = 0;
  g_scene.tick = 0;
  g_scene.rng = (uint32_t)time(NULL) ^ 0xDEAD1234u;
  g_scene.preset = ((preset % N_PRESETS) + N_PRESETS) % N_PRESETS;

  switch (g_scene.preset) {
  case PRESET_BRICK_WALL:
    preset_brick_wall();
    break;
  case PRESET_BEAMS:
    preset_beams();
    break;
  case PRESET_TOWER:
    preset_tower();
    break;
  case PRESET_PYRAMID:
    preset_pyramid();
    break;
  }
}

/* ===================================================================== */
/* §8  draw                                                               */
/* ===================================================================== */

/*
 * Sphere is drawn using hw (x) and hh (y) so the ellipse matches the
 * AABB exactly.  prow(y + hh*sin) = (y + 2r*sin) / 2 → r rows from
 * center on screen.  pcol(x + hw*cos) = x + r cols from center.
 * The result is a circle of radius r on screen, matching the physics box.
 */
static void draw_body(const Body *b) {
  int cp = b->cp;
  if (b->kind == KIND_SPHERE) {
    float rx = b->hw, ry = b->hh;
    int steps = (int)(2.f * 3.14159265f * rx) + 8;
    for (int i = 0; i < steps; i++) {
      float a = i * 2.f * 3.14159265f / steps;
      fb_put(prow(b->y + ry * sinf(a)), pcol(b->x + rx * cosf(a)), 'O', cp);
    }
    fb_put(prow(b->y), pcol(b->x), '+', cp);
  } else {
    int x0 = pcol(b->x - b->hw), x1 = pcol(b->x + b->hw);
    int y0 = prow(b->y - b->hh), y1 = prow(b->y + b->hh);
    fb_hline(y0, x0, x1, '#', cp);
    fb_hline(y1, x0, x1, '#', cp);
    fb_vline(x0, y0, y1, '#', cp);
    fb_vline(x1, y0, y1, '#', cp);
    fb_put(prow(b->y), pcol(b->x), '+', cp);
  }
}

/*
 * Two-bar HUD per CLAUDE.md convention:
 *   row 0       right (CP_HUD  bright yellow + bold) — live status
 *   row rows-1  left  (CP_HINT bright cyan   + bold) — actions / keys
 */
static void draw_hud_top(int fps) {
  char buf[220];
  snprintf(buf, sizeof buf,
           " preset:%s  bodies:%d/%d  grav:%s  theme:%s  %s  %d fps ",
           k_preset_names[g_scene.preset], g_scene.nb, MAX_BODIES,
           g_scene.grav ? "on" : "off", k_themes[g_scene.theme].name,
           g_scene.paused ? "PAUSED " : "running", fps);
  int len = (int)strlen(buf);
  int col = g_cols - len;
  if (col < 0)
    col = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, col, buf, g_cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

static void draw_hud_bottom(void) {
  const char *full = " q:quit  SPACE:fire ball  n/N:preset  r:reset  "
                     "p:pause  c/s:cube/sph  x:del  g:grav  t/T:theme ";
  const char *shrt =
      " q:quit  SPACE:fire  n:preset  r:reset  p:pause  t:theme ";
  const char *h = full;
  if ((int)strlen(full) >= g_cols - 1)
    h = shrt;
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(g_rows - 1, 0, h, g_cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void scene_draw(int fps) {
  erase();
  fb_clear();

  /* floor line — drawn first so body chars overlap it where they sit. */
  int floor_row = prow(WH());
  attron(COLOR_PAIR(CP_FLOOR));
  for (int c = 0; c < g_cols; c++)
    mvaddch(floor_row, c, '=');
  attroff(COLOR_PAIR(CP_FLOOR));

  /* bodies via framebuffer (clipped to rows 1..g_rows-3 by fb_put). */
  for (int i = 0; i < g_scene.nb; i++)
    draw_body(&g_scene.b[i]);
  fb_flush();

  /* HUD overlays — painted after the world so they always win row 0
   * and row g_rows-1 even if a body would have drawn there. */
  draw_hud_top(fps);
  draw_hud_bottom();
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

static volatile sig_atomic_t g_resize = 0, g_quit = 0;
static void on_sigwinch(int s) {
  (void)s;
  g_resize = 1;
}
static void on_sigterm(int s) {
  (void)s;
  g_quit = 1;
}

static void screen_init(void) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  start_color();
  use_default_colors(); /* enable -1 (transparent) bg in init_pair */
  g_256 = (COLORS >= 256);
  theme_apply(g_scene.theme);
}
static void screen_resize(void) {
  endwin();
  refresh();
  int r, c;
  getmaxyx(stdscr, r, c);
  g_rows = (r < ROWS_MAX) ? r : ROWS_MAX;
  g_cols = (c < COLS_MAX) ? c : COLS_MAX;
  g_resize = 0;
}

/* ===================================================================== */
/* §10  app                                                               */
/* ===================================================================== */

int main(void) {
  signal(SIGWINCH, on_sigwinch);
  signal(SIGTERM, on_sigterm);
  signal(SIGINT, on_sigterm);

  g_scene.rng = (uint32_t)time(NULL) ^ 0xDEAD1234u;
  screen_init();
  {
    int r, c;
    getmaxyx(stdscr, r, c);
    g_rows = (r < ROWS_MAX) ? r : ROWS_MAX;
    g_cols = (c < COLS_MAX) ? c : COLS_MAX;
  }
  scene_init(PRESET_BRICK_WALL);

  int64_t next = clock_ns();
  int64_t fps_window_start = next;
  int fps_frames = 0;
  int fps_disp = 0;

  while (!g_quit) {
    int ch;
    while ((ch = getch()) != ERR) {
      switch (ch) {
      case 'q':
      case 27:
        g_quit = 1;
        break;
      case ' ':
        fire_projectile();
        break;
      case 'p':
      case 'P':
        g_scene.paused = !g_scene.paused;
        break;
      case 'r':
        scene_init(g_scene.preset);
        break;
      case 'c':
        scene_add_cube();
        break;
      case 's':
        scene_add_sphere();
        break;
      case 'x':
        scene_remove_last();
        break;
      case 'g':
        g_scene.grav = !g_scene.grav;
        break;
      case 'n':
        scene_init(g_scene.preset + 1);
        break;
      case 'N':
        scene_init(g_scene.preset - 1 + N_PRESETS);
        break;
      case 't':
        g_scene.theme = (g_scene.theme + 1) % N_THEMES;
        theme_apply(g_scene.theme);
        break;
      case 'T':
        g_scene.theme = (g_scene.theme + N_THEMES - 1) % N_THEMES;
        theme_apply(g_scene.theme);
        break;
      }
    }

    if (g_resize) {
      screen_resize();
      scene_init(g_scene.preset);
    }

    int64_t now = clock_ns();
    if (!g_scene.paused && now >= next) {
      scene_step();
      next = now + NS_PER_S / SIM_FPS;
    }

    /* rolling 1-second fps window */
    fps_frames++;
    if (now - fps_window_start >= NS_PER_S) {
      fps_disp = fps_frames;
      fps_frames = 0;
      fps_window_start = now;
    }

    scene_draw(fps_disp);
    wnoutrefresh(stdscr);
    doupdate();
    sleep_ns(next - clock_ns() - 1000000LL);
  }

  endwin();
  return 0;
}
