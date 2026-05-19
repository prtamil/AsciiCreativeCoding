/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * soft_multiple_bodies.c — Destructible Jelly Demos
 *                          (Position-Based Dynamics, anchored + tearable)
 *
 * Each preset stages ONE big soft-body structure standing on the floor,
 * its bottom row pinned to the ground.  The user fires a soft projectile
 * ball with SPACE.  When the ball impacts the structure, distance
 * constraints near the impact get stretched past their break threshold
 * and SNAP — the upper portion tears off, falls under gravity (it's still
 * a soft body, just no longer connected to the anchored base), and
 * jiggles as it lands.  The anchored base remains standing, ripples from
 * the shock, settles.
 *
 * Design from first principles:
 *
 *   1. Each STRUCTURE is one big monolithic body (or, for the multi-
 *      structure presets, a few independent big bodies — Twin Towers
 *      has two, Big + Small has one of each).  We never STACK bodies
 *      vertically: multi-body stacks have inter-body contact pressure
 *      that compounds upward and liquefies under PBD [1, §3].  Each
 *      independent big body has zero internal body-vs-body contacts,
 *      so the stack-pressure problem doesn't exist.
 *
 *   2. Bottom row pinned (kinematic anchor).  Pinned nodes ignore
 *      gravity, integration, and PBD position changes — they are
 *      infinite-mass anchors [1, eq. 7].  The body is rooted to the
 *      floor and cannot slide or fall over.
 *
 *   3. Breakable distance constraints.  Each Con has a break_thresh;
 *      when |x_b − x_a| > rest · break_thresh the constraint is
 *      permanently removed.  Ball impact stress concentrates near
 *      the contact, those constraints break first, fracture propagates
 *      outward — exactly how real brittle material cracks.
 *
 *   4. PBD + collision INTERLEAVED.  Each substep runs pbd_iters
 *      alternations of (project distance constraints) and (resolve
 *      pair penetrations).  Every collision push is followed by an
 *      immediate constraint-relaxation pass before the next collision,
 *      so the body's shape stays close to its rest configuration
 *      between contacts.
 *
 * Presets (cycled with n / N):
 *
 *   1. Big Tower    — one massive 6×18 tower, dominating the right
 *                     two-thirds of the world.  Ball at chest height
 *                     tears the middle and the top 60 % falls.
 *   2. Twin Towers  — two medium 5×14 towers in line.  Ball passes
 *                     through (or breaks) the first, then strikes
 *                     the second with whatever energy remains.
 *   3. Mini Tower   — one short 4×10 tower.  Easy single-hit topple;
 *                     useful for fine-tuning break_thresh with b/B.
 *   4. Big + Small  — a small 4×10 + a big 6×18, side by side.
 *                     Ball hits small first, loses energy, then
 *                     reaches big with reduced impact.
 *
 * Keys:
 *   SPACE   fire soft ball at the structure
 *   n/N     next / previous preset             r     reset current preset
 *   p       pause                               q/ESC quit
 *   g       toggle gravity
 *   i / I   PBD iteration count ± 1            t/T   theme cycle
 *   b / B   break threshold ± 0.1              x     remove last spawned ball
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/soft_multiple_bodies.c -o
 * soft_multiple_bodies -lncurses -lm
 *
 * Sections:
 *   §1  config           §2  clock            §3  color / theme
 *   §4  state            — Node, Con, Blob, Scene structs + g_scene
 *   §5  physics          — Verlet predict + PBD relaxation + boundary
 *   §6  collision        — point-in-polygon + edge-resolve + Newton-3
 *   §7  scene            — builders + presets + projectile + scene_step
 *   §8  draw             — body + HUD
 *   §9  screen           §10 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Position-Based Dynamics with breakable constraints
 *                  and kinematic pinning [1][2].  Each body is a
 *                  network of mass-points connected by distance
 *                  constraints.  Verlet predicts free-flight positions
 *                  [3]; PBD then relaxes the constraints by projecting
 *                  particles toward their rest-distance neighbours.
 *
 * Pinning        : A node with `pinned = true` has effective INVERSE
 *                  MASS = 0.  In Müller's mass-weighted projection
 *                  [1, eq. 7] the corrections distribute by inverse
 *                  mass, so a 0-mass node never moves.  All forces
 *                  (gravity, collision push, distance projection) are
 *                  ignored for pinned nodes.
 *
 * Tearing        : Each constraint carries a break threshold T.  If
 *                  current_length > rest_length · T at any iteration,
 *                  the constraint sets its k = 0 and is permanently
 *                  ignored thereafter.  Connectivity DEGRADES rather
 *                  than the body splitting into separate Blobs — we
 *                  keep a single Blob struct but its constraint graph
 *                  becomes disconnected.  Disconnected sub-regions
 *                  evolve independently because their constraints no
 *                  longer reach across the fracture line.
 *
 * Collision      : Point-in-polygon (Jordan curve [5]) detection,
 *                  symmetric position-split resolution [1, §5].  Pinned
 *                  edge nodes don't get pushed (they're anchors).
 *                  Interleaved with PBD passes so visible jelly squish
 *                  recovers between contacts.
 *
 * Rendering      : Body wireframe (constraint segments) + scan-fill
 *                  interior + boundary nodes (bold 'O' for free, '*'
 *                  for pinned).  Broken constraints are skipped — the
 *                  fracture line becomes visually obvious as the gap
 *                  between still-connected wire segments.
 *
 * References (cite inline as [n]):
 *
 *   [1] Müller, M.; Heidelberger, B.; Hennix, M.; Ratcliff, J. (2007) —
 *       "Position Based Dynamics", J. Visual Communication and Image
 *       Representation 18 (2), 109–118.  The foundational PBD paper.
 *       Specifically used:
 *         §3  the predict / project / update outer loop in scene_step
 *         eq.  6  per-particle distance correction (un-weighted form)
 *         eq.  7  inverse-mass-weighted distance projection — what
 *                 project_all_distance_constraints implements; setting
 *                 w_i = 0 recovers the "pinned anchor" behaviour with
 *                 no special-case code
 *         §5  position-space collision constraints (the symmetric
 *                 push pattern in resolve_penetration)
 *
 *   [2] Bender, J.; Müller, M.; Macklin, M. (2017) — "A Survey on
 *       Position-Based Simulation Methods in Computer Graphics",
 *       Computer Graphics Forum 36 (6).  Modern PBD overview covering
 *       attachment constraints (the pinning we use, §3.1), tearable
 *       constraints (§4.3), and the standard "interleave constraint
 *       projection across all constraint types per iteration" recipe
 *       (§2.2) — which is exactly what scene_step does with distance
 *       and collision constraints.
 *
 *   [3] Jakobsen, T. (2001) — "Advanced Character Physics", GDC.
 *       Verlet two-position storage: velocity is implicit in
 *       (x − x_prev), no separate velocity array needed.  Used by
 *       verlet_predict_all in §5 and by fire_projectile in §7 to
 *       inject the ball's initial horizontal velocity (px = x − v0).
 *
 *   [4] Ericson, C. (2005) — Real-Time Collision Detection, Morgan
 *       Kaufmann.  §5.1.5 "closest point on segment" — the parametric
 *       t = clamp((P − A)·(B − A) / |B − A|², 0, 1) formula used by
 *       nearest_polygon_edge in §6.
 *
 *   [5] Sunday, D. (2001) — "Inclusion of a Point in a Polygon",
 *       geomalgorithms.com.  Even-odd ray-cast test (Jordan curve
 *       theorem).  Implemented in point_in_polygon_jordan in §6 with
 *       Sunday's asymmetric (ay ≤ py < by) endpoint convention that
 *       avoids the vertex-grazing double-count.
 *
 *   [6] Provot, X. (1995) — "Deformation Constraints in a Mass-Spring
 *       Model to Describe Rigid Cloth Behaviour", Graphics Interface.
 *       The classic source for "spring/constraint with break length":
 *       Provot defined cloth tearing as snipping a spring once it
 *       stretches past a length threshold.  Our `break_thresh` field
 *       on Con and the break-check in project_all_distance_constraints
 *       are a direct port of Provot's flag to the PBD framework.
 *
 *   [7] Padala, P. — NCURSES Programming HOWTO, The Linux
 *       Documentation Project.  Reference for the render layer in §8:
 *       init_pair / COLOR_PAIR semantics, the wnoutrefresh + doupdate
 *       diff model, signal-safe SIGWINCH handling.
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

#define ROWS_MAX 128
#define COLS_MAX 512

/* ── Body geometry capacities  (per Blob, statically allocated) ────────── */

/* BLOB_NODES_MAX — max mass points per body.  256 covers the biggest
 * preset tower (6×18 = 108 nodes) with comfortable headroom; the
 * Big + Small preset's two towers (108 + 40 = 148 nodes COMBINED
 * across two Blobs) easily fit since each Blob has its own array. */
#define BLOB_NODES_MAX 256

/* BLOB_CONS_MAX — max distance constraints per body.  A 6×18 big-tower
 * grid has 6·17 + 18·5 = 192 structural + 2·17·5 = 170 shear = 362
 * cons.  900 leaves comfortable headroom for the biggest preset. */
#define BLOB_CONS_MAX 900

/* BLOB_BND_MAX — max boundary polygon length.  Perimeter of a 6×18
 * grid is 2·(5+17) = 44 nodes; 128 is generous. */
#define BLOB_BND_MAX 128

/* MAX_BLOBS — pool size for Scene.blobs[].  Need up to 2 preset bodies
 * (Twin Towers / Big + Small) + ~14 fired soft balls before old balls
 * roll off screen.  Could go higher but the pair-collision loop is
 * O(MAX_BLOBS²) so 16 is the sweet spot for our ~20 Hz simulation. */
#define MAX_BLOBS 16

/* ── Sphere (projectile ball)  ───────────────────────────────────────── */
#define SPH_RING 12 /* ring nodes (node 0 is centre)        */
#define SPH_NODES (SPH_RING + 1)
#define SPH_R 4.0f /* visual col-radius                    */

/* ── Stiffness factors  (PBD k values) ───────────────────────────────── */
/* k = 1.0 means "full distance correction every iteration"; k = 0.5
 * means "half correction" → softer.  These are STIFFNESS, not break
 * thresholds — they don't determine when a constraint snaps. */
#define STRUCT_K 1.0f /* edges of the grid: full stiffness   */
#define SHEAR_K 0.7f  /* diagonals: slightly softer = jelly  */

/* ── Break thresholds  (stretch ratio before constraint snaps) ───────── */
/* A constraint with rest length L breaks when current length > L · T.
 * T = 1.0 means "break immediately on any stretch" (catastrophic).
 * T = ∞  means "never break" (rubber).
 * Sweet spot for visible tearing on ball impact is 1.4 - 2.0. */
#define BREAK_THRESH_DEF 1.6f
#define BREAK_THRESH_MIN 1.1f
#define BREAK_THRESH_MAX 3.0f
#define BREAK_THRESH_STEP 0.1f

/* ── Physics knobs  (per sub-step) ───────────────────────────────────── */
#define GRAVITY_DEF 0.06f  /* downward accel (px / sub-step²)     */
#define DAMPING_DEF 0.985f /* velocity retention each sub-step    */
#define FLOOR_REST 0.10f   /* bounce restitution on floor         */
#define WALL_REST 0.10f    /* bounce restitution on walls         */
#define FLOOR_FRIC 0.85f   /* horizontal velocity retained on floor */

/* PBD_ITERS controls effective body stiffness AND collision tightness
 * (because PBD + collision are interleaved — see scene_step in §7).
 * More iters = stiffer body + tighter contacts.  10 is enough for an
 * anchored jelly slab to look rigid until the ball hits it. */
#define PBD_ITERS_DEF 10
#define PBD_ITERS_MIN 2
#define PBD_ITERS_MAX 30

/* STEPS_DEF — physics sub-steps per render frame.  3 keeps fast-moving
 * projectile balls from tunneling through thin parts of the wall. */
#define STEPS_DEF 3

/* PROJECTILE_VX — initial Verlet velocity of a SPACE-fired ball.
 * Applied as px = x − v0 on every node so the whole ball translates at
 * v0 from frame 1.  Tuned high enough to deform AND tear the wall. */
#define PROJECTILE_VX 3.5f

/* ── World mapping  (physics units → terminal cells) ─────────────────── */
/* One cell-column = 1 px in x; one cell-row = 2 px in y (terminal cells
 * are ~2× taller than wide).  HUD_TOP_PX = 2 reserves the top cell-row
 * (row 0) for the status bar so bodies never draw into it. */
#define HUD_TOP_PX 2.0f
#define PHY_TO_ROW(y) ((int)((y) * 0.5f + 0.5f))
#define PHY_TO_COL(x) ((int)((x) + 0.5f))

/* ── Timing  ─────────────────────────────────────────────────────────── */
#define SIM_FPS 20
#define NS_PER_SEC 1000000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* ── Themes (5 palettes; theme cycles theme indices, not k_themes size) */
#define N_THEMES 5

/* ── Presets ─────────────────────────────────────────────────────────── */
#define N_PRESETS 4

typedef enum {
  PRESET_BIG_TOWER = 0, /* one big single tower (6×18)             */
  PRESET_TWIN_TOWERS,   /* two medium towers (each 5×15)           */
  PRESET_MINI_TOWER,    /* one short tower (4×10)                  */
  PRESET_BIG_AND_SMALL, /* one mini + one big tower side by side   */
} Preset;

/* ── Color slots (3 cubes + 3 spheres + chrome) ──────────────────────── */
#define N_BSLOTS 6
#define CP_BSURF(i) (1 + (i))            /* pairs 1-6  */
#define CP_BFILL(i) (1 + N_BSLOTS + (i)) /* pairs 7-12 */
#define CP_FLOOR (1 + 2 * N_BSLOTS)      /* 13                 */
#define CP_PIN (2 + 2 * N_BSLOTS)        /* 14 — pinned nodes  */
#define CP_HUD (3 + 2 * N_BSLOTS)        /* 15 — top status    */
#define CP_HINT (4 + 2 * N_BSLOTS)       /* 16 — bottom hints  */

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec r = {(time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC)};
  nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  color / theme — 5 brightness-safe palettes + fixed HUD chrome     */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Theme — palette for one rendering of the destructible-jelly scene.
 *
 * Intent:
 *   Pure visual change.  Cycling themes (t / T keys) MUST leave the
 *   simulation byte-identical: every blob, every node position, every
 *   constraint stays exactly where it was — only the colour-pair RGB
 *   shifts.  This is what justifies `theme` living in the Rendering
 *   group on Scene (§4) and never on the Blob / Node themselves.
 *
 * Why TWO arrays (surf + fill) per slot:
 *   Each body is rendered in three layers (see draw_blob in §8):
 *     1. scan-fill INTERIOR — dim, fill_cp colour, ':' character
 *     2. constraint WIREFRAME — bright, surf_cp colour, '|/-\' chars
 *     3. boundary NODES — bold, surf_cp colour, 'O' (free) or '*' (pinned)
 *   Splitting surf vs fill gives the body a 3-D "shaded" look — the
 *   silhouette pops against the dimmer interior.  A single-colour
 *   theme would still work but loses the depth cue.
 *
 * Why 6 slots:
 *   Bodies are tagged with one of 6 colour-pair indices at spawn:
 *     slots 0..2  — cool family, used for slabs / towers
 *     slots 3..5  — warm family, used for projectile spheres
 *   Cycling is round-robin (`nblobs % 3` for slabs, `nballs % 3` for
 *   balls) so each new body gets a fresh distinguishable colour
 *   without needing more than 3 hues per family.
 *
 * Brightness safety (CLAUDE.md, see documentation/COLOR.md):
 *   All palette entries sit at 256-cube index ≥ 24.  Indices in
 *   16–23 / 232–239 are forbidden — they render as background-black
 *   on default-bg terminals and the body disappears.
 *
 * Refs:
 *   256-colour cube structure   — XTerm Control Sequences §
 *                                 "ISO-8613-3 Controls".
 *   Colour-pair semantics       — Padala NCURSES HOWTO [7].
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* Human-readable label shown in the top HUD's `theme:` field.
   * ≤ 7 chars so the full status line fits an 80-col terminal
   * alongside the other HUD fields (preset:, bodies:, iters:, …). */
  const char *name;

  /* surf[i] — surface colour for slot i.  Used for the boundary
   * wireframe (constraint segments) AND the bold 'O' node glyphs.
   * Chosen as the BRIGHTER end of the palette so the body's
   * silhouette reads clearly against the dimmer interior fill. */
  short surf[N_BSLOTS];

  /* fill[i] — interior scan-fill colour for slot i.  Dimmer / darker
   * than surf[i] so the wireframe pops on top.  Typically a tonal
   * variant of the matching surf colour (e.g. 51 surf → 24 fill in
   * the Ocean theme).  Cells inside the body are stippled with ':'
   * in this colour. */
  short fill[N_BSLOTS];

  /* pin — accent colour for anchored / pinned nodes ('*' glyphs).
   * Independent of the 6 body slots so all bodies show their
   * pinned base in the same eye-catching colour.  Typically a
   * bright accent (yellow / white) that contrasts with every
   * surf[] colour in the theme. */
  short pin;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* name       surf[0..5]                       fill[0..5] pin */
    {"Ocean", {51, 45, 87, 214, 196, 206}, {24, 24, 55, 130, 88, 125}, 226},
    {"Matrix", {82, 46, 118, 196, 160, 124}, {29, 29, 35, 52, 52, 52}, 226},
    {"Nebula", {213, 177, 147, 87, 123, 129}, {93, 61, 55, 55, 89, 93}, 220},
    {"Fire", {226, 220, 214, 171, 135, 99}, {100, 94, 88, 93, 57, 57}, 231},
    {"Mono",
     {255, 245, 235, 230, 220, 210},
     {240, 230, 220, 215, 205, 195},
     255},
};

#define CHROME_HUD 226 /* bright yellow (theme-independent) */
#define CHROME_HINT 51 /* bright cyan   (theme-independent) */

static bool g_has_256;

static void theme_apply(int ti) {
  const Theme *t = &k_themes[((ti % N_THEMES) + N_THEMES) % N_THEMES];
  for (int i = 0; i < N_BSLOTS; i++) {
    short cs = g_has_256 ? t->surf[i] : (short)(COLOR_RED + i % 6);
    short cf = g_has_256 ? t->fill[i] : (short)(COLOR_RED + i % 6);
    init_pair(CP_BSURF(i), cs, -1);
    init_pair(CP_BFILL(i), cf, -1);
  }
  init_pair(CP_FLOOR, g_has_256 ? 244 : COLOR_WHITE, -1);
  init_pair(CP_PIN, g_has_256 ? t->pin : COLOR_YELLOW, -1);
  init_pair(CP_HUD, g_has_256 ? CHROME_HUD : COLOR_YELLOW, -1);
  init_pair(CP_HINT, g_has_256 ? CHROME_HINT : COLOR_CYAN, -1);
}

/* ===================================================================== */
/* §4  state — Node, Con, Blob, Scene                                    */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Node — one mass point of a soft body.
 *
 * Intent:
 *   The atomic element of the simulation.  A soft body is just a cloud
 *   of Nodes connected by Cons (distance constraints).  The body's
 *   "shape" and "stiffness" are entirely emergent from the constraint
 *   network; the Node itself carries only POSITION (Jakobsen [3] Verlet
 *   form) and an OPTIONAL pinning flag.
 *
 * Verlet two-position storage (Jakobsen [3]):
 *   We don't store velocity explicitly.  Verlet integration says
 *
 *       x_{n+1} = x_n + (x_n − x_{n-1}) · damping + a · dt²
 *
 *   so velocity is recoverable as (x − x_prev) at any time.  Saving
 *   one float pair per node by NOT having a velocity field is the
 *   classical Verlet trick — and it has a subtle PBD bonus: any
 *   position constraint that snaps the node to a new x AUTOMATICALLY
 *   gets absorbed into the next step's implicit velocity, with no
 *   separate velocity-update needed (Müller [1, §3]).
 *
 * Pinning (Müller [1, eq. 7]):
 *   When `pinned == true` the node has effective inverse mass w = 0.
 *   Every physics pass checks this and skips the node:
 *     - verlet_predict_all:           no gravity, no integration
 *     - project_all_distance_constraints:  w = 0 ⇒ zero correction
 *     - clamp_all_nodes_to_world:     skipped (already at valid pos)
 *     - apply_boundary_velocity_response:  skipped
 *     - resolve_penetration:          can't be pushed (anchor)
 *   Pinned nodes are placed once at scene_init and never move.  They
 *   are the FOUNDATION the rest of the body hangs from — without them
 *   gravity would pull the entire body to the floor and disintegrate
 *   it against the bottom wall.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* x, y — current world position in pixel space.
   * Mutated EVERY substep by:
   *   - verlet_predict_all (gravity + free flight)
   *   - project_all_distance_constraints (constraint relaxation)
   *   - clamp_all_nodes_to_world (arena clamp)
   *   - resolve_penetration (collision push out / inward squish)
   * Read by every rendering pass (PHY_TO_COL/PHY_TO_ROW in §8). */
  float x, y;

  /* px, py — position from the PREVIOUS substep.  Saved at the
   * start of verlet_predict_all (right before x/y change) so the
   * implicit velocity v = (x − px) survives across the step.
   * Also tweaked by apply_boundary_velocity_response to reflect
   * velocity off walls/floor without touching x/y directly. */
  float px, py;

  /* pinned — kinematic-anchor flag.  Set ONCE at preset build time
   * (pin_grid_bottom_row, §7) and never toggled at runtime.  When
   * true, the node behaves as if it had infinite mass: nothing
   * moves it.  Read by all five physics passes in §5 + by §6
   * resolve_penetration, and rendered with a distinct glyph ('*'
   * in CP_PIN colour) so the foundation is visually obvious. */
  bool pinned;
} Node;

/* ─────────────────────────────────────────────────────────────────────── *
 * Con — one breakable distance constraint between two Nodes.
 *
 * Intent:
 *   The "spring" of the soft body — except in PBD there are no springs,
 *   only POSITION INVARIANTS.  Each Con says: "node `a` and node `b`
 *   should be exactly `rest` apart."  Each PBD iteration looks at the
 *   constraint, computes how badly the invariant is violated, and
 *   geometrically moves the two endpoints to satisfy it.
 *
 * The PBD distance correction (Müller [1, eq. 7]):
 *
 *       d   = |x_b − x_a|
 *       err = d − rest
 *       w_a = pinned(a) ? 0 : 1
 *       w_b = pinned(b) ? 0 : 1
 *       Δa  = + (w_a / (w_a + w_b)) · (err / d) · k · (x_b − x_a)
 *       Δb  = − (w_b / (w_a + w_b)) · (err / d) · k · (x_b − x_a)
 *       x_a += Δa ;  x_b += Δb
 *
 *   When both nodes are free, the correction splits half-and-half.
 *   When one is pinned (w = 0), the free node takes the FULL correction
 *   — this is how anchoring works without special-case code.
 *
 * Breakability (Provot [6]):
 *   At the top of every projection iteration we test
 *
 *       if (break_thresh > 0 && d > rest · break_thresh)
 *           c.k = 0  →  permanently broken
 *
 *   Once a constraint's k is zero, all subsequent iterations skip it
 *   AND the wireframe renderer skips it (so the fracture line becomes
 *   visually obvious).  Fracture is one-way: no constraint ever
 *   reactivates.  This matches Provot's cloth-tearing flag from his
 *   1995 paper — porting the idea from springs to PBD distance
 *   constraints is direct (Bender survey [2, §4.3] discusses the
 *   modern variants).
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* a, b — endpoint node indices into the OWNING Blob's nodes[] array.
   * Convention: blob_add_con (§7) is called with the lower index
   * first but the projection math is symmetric so swapping a/b
   * makes no functional difference.  Indices must be valid
   * (0 ≤ a, b < n_nodes) at the time blob_build_grid runs — they
   * are NEVER re-validated thereafter, so don't shrink n_nodes
   * after constraints have been added. */
  int a, b;

  /* rest — target distance between the two nodes.  Set ONCE at build
   * time as the initial |x_b − x_a| from blob_add_con.  Never
   * mutated afterwards — there's no support for "plastic" deformation
   * (where the rest length adapts to the current configuration); a
   * stretched-then-released constraint pulls back toward its
   * original length, giving the body its elastic memory. */
  float rest;

  /* k — stiffness factor in [0, 1].  Multiplies the correction each
   * iteration, so larger k = stiffer.  0 is a SENTINEL meaning
   * "constraint is broken" — both the projector and the wireframe
   * draw skip it.  Practical values are STRUCT_K = 1.0 for edges
   * and SHEAR_K = 0.7 for diagonals; lower k anywhere gives a
   * "rubbery" softer feel. */
  float k;

  /* break_thresh — stretch ratio at which the constraint snaps.
   * Concretely: when current_d > rest · break_thresh, k is set to 0.
   *   0       → constraint NEVER breaks (used for the projectile
   *             ball — we don't want the ball to fall apart).
   *   1.0     → breaks on any stretch (impossibly fragile).
   *   1.4–2.0 → "destructible material" feel: deforms visibly,
   *             snaps under heavy impact.  Default 1.6.
   * Tuned interactively at runtime with the b / B keys. */
  float break_thresh;
} Con;

/* ─────────────────────────────────────────────────────────────────────── *
 * BKind — body shape discriminator.
 *
 * Currently only used by blob_color cycling in §7 to pick a colour
 * family: SLAB → cool palette (slots 0..2), SPHERE → warm (slots 3..5).
 * Physics, collision, and rendering paths are UNIFORM across kinds
 * (both shapes use the same generic point-in-polygon collision and
 * scan-fill render).  Adding a new shape means: extend this enum,
 * write a builder (like blob_build_sphere), pick a colour slot.
 * ─────────────────────────────────────────────────────────────────────── */
typedef enum { BKIND_SLAB, BKIND_SPHERE } BKind;

/* ─────────────────────────────────────────────────────────────────────── *
 * Blob — one soft body (one preset structure OR one projectile ball).
 *
 * Intent:
 *   A complete self-contained simulation entity.  A Blob bundles every
 *   piece of state needed to integrate its physics, collide against
 *   other Blobs, and render itself.  No pointers, no dynamic alloc —
 *   the whole thing lives in BSS via the Scene struct (§ below).
 *
 * Architectural note — "big monolithic bodies, never stacked":
 *   Multi-body soft stacks (the previous architecture) liquefy under
 *   PBD because inter-body contact pressure compounds upward faster
 *   than constraint relaxation can recover.  This file's design
 *   avoids that entirely: each STRUCTURE in a preset is one big Blob
 *   with its bottom row of nodes PINNED to the floor.  Multi-structure
 *   presets (Twin Towers, Big + Small) spawn several such Blobs side
 *   by side but NEVER vertically stacked, so the only inter-Blob
 *   contact is between the projectile ball and a structure — a single
 *   transient impact, not a sustained stack-load.
 *
 * Capacities (BLOB_NODES_MAX = 256, BLOB_CONS_MAX = 900,
 *   BLOB_BND_MAX = 128):
 *   Sized so the largest preset (6×18 = 108-node big tower with 362
 *   constraints) fits as one body.  Static allocation; no malloc.
 *
 * Boundary contract (load-bearing for §6 collision):
 *   bnd[0..n_bnd) holds the indices of the outer polygon, ordered
 *   COUNTER-CLOCKWISE.  Consumed by:
 *     - point_in_polygon_jordan (§6): Jordan curve test [5]
 *     - nearest_polygon_edge    (§6): closest-point-on-segment [4]
 *     - draw_blob               (§8): scan-fill interior
 *   The boundary INDICES don't change when constraints break — but
 *   the COORDINATES of those nodes evolve with deformation, so the
 *   polygon literally distorts.  Post-fracture this means the
 *   point-in-polygon test is approximate (the polygon may not match
 *   the true visible silhouette of the fractured body).  We accept
 *   this imperfection because the alternative — splitting a Blob
 *   into connected components after each fracture — is ~200 lines
 *   of code for marginal visual gain in this demo.
 *
 * Refs:
 *   PBD constraint network    — Müller [1, §3]
 *   Cloth + pinning analogy   — Bender survey [2, §3.1]
 *   Verlet (no velocity field) — Jakobsen [3]
 *   Tearable cloth flag       — Provot [6]
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── Mesh: nodes + constraints + outer ring ───────────────────────
   * The three arrays together define the body's mass distribution
   * (nodes), elastic structure (cons), and visible / collidable
   * silhouette (bnd).  All three are filled at build time and the
   * sizes (n_nodes, n_cons, n_bnd) are set once — only the contents
   * mutate thereafter (nodes move, cons may break, bnd indices stay).
   */

  /* nodes[0..n_nodes) — the body's mass points.  Indexed by Con.a/b
   * and bnd[].  Indices are STABLE for the lifetime of the body;
   * nothing ever shifts the array. */
  Node nodes[BLOB_NODES_MAX];
  int n_nodes;

  /* cons[0..n_cons) — distance constraints.  PROJECTION ORDER
   * (Gauss-Seidel-style) is the order constraints were appended:
   * blob_build_grid adds structural cons row-major, then shear cons
   * row-major.  Reordering would subtly change convergence behaviour
   * at low pbd_iters — keep build order deterministic. */
  Con cons[BLOB_CONS_MAX];
  int n_cons;

  /* bnd[0..n_bnd) — outer-polygon node indices, CCW ordered.  See
   * the boundary contract in the top docstring.  Read-only after
   * build; mutating these breaks point_in_polygon_jordan and
   * draw_blob's scan-fill simultaneously. */
  int bnd[BLOB_BND_MAX];
  int n_bnd;

  /* ── Render metadata ─────────────────────────────────────────────
   * Assigned at spawn by blob_build_*; survives theme cycles because
   * theme_apply rewrites the SAME pair indices with new RGB rather
   * than re-tagging bodies.
   */

  /* surf_cp — ncurses colour-pair index for the boundary wireframe
   * AND the bold 'O' node glyphs.  Brighter end of the theme. */
  int surf_cp;

  /* fill_cp — ncurses colour-pair index for the scan-fill ':' chars
   * inside the polygon.  Dimmer than surf so the silhouette pops. */
  int fill_cp;

  /* kind — BKIND_SLAB or BKIND_SPHERE.  Only consulted by colour-
   * slot cycling (blob_color in §7) and as a possible future hook
   * for kind-specific rendering.  Physics + collision treat all
   * kinds uniformly. */
  BKind kind;
} Blob;

/* ─────────────────────────────────────────────────────────────────────── *
 * Scene — top-level demo state spanning physics ticks, draw calls, and
 * the main loop.  Single instance `g_scene` in BSS; passing by pointer
 * everywhere would just clutter helper signatures without buying any
 * encapsulation.
 *
 * The struct splits into two clearly-labelled groups:
 *
 *   Simulation parameters — consumed by scene_step (§7), the §5 physics
 *     helpers, scene_init / fire_projectile / preset_*() (§7).  Anything
 *     that changes WHAT THE BODIES DO belongs here.  Mutated by physics-
 *     affecting keys: SPACE, n/N (preset), r (reset), p (pause), g
 *     (gravity), i/I (iter count), b/B (break threshold), x (delete).
 *
 *   Rendering parameters — consumed only by §8 draw_*() helpers and
 *     theme_apply (§3).  Mutating these must leave every Blob byte-
 *     identical: positions, constraints, broken flags, pinned flags
 *     all stay exactly the same; only colour-pair RGB / HUD text change.
 *     Mutated by purely cosmetic keys: t / T (theme).
 *
 * Locality rationale (this contract matters, not the bytes):
 *   The split exists for the READER, not the CPU.  A new flag landing
 *   in the rendering group when it actually changes how scene_step
 *   advances state would silently couple display to physics — exactly
 *   the bug the separation prevents.  When adding a field, ask: does
 *   this change what scene_step produces given the same body pool and
 *   the same dt?  If yes, simulation; if no, rendering.
 *
 * What stays OUTSIDE Scene (intentionally):
 *
 *   g_rows / g_cols       screen geometry tracked by the main loop's
 *                         SIGWINCH handler; Scene stays geometry-
 *                         agnostic so resize logic is the main loop's
 *                         concern, not the simulator's.
 *
 *   g_quit / g_resize     volatile sig_atomic_t flags written by the
 *                         signal handler; must stay file-scope for
 *                         async-signal-safety (the C standard
 *                         guarantees atomic load/store only for
 *                         objects of that type at file scope).
 *
 *   g_has_256             boot-time terminal capability probed once
 *                         in screen_init; not state that evolves.
 *
 *   g_mincol[] / g_maxcol[]   per-row scan-fill scratch arrays used
 *                         only inside draw_blob; not persistent state.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── Simulation parameters ──────────────────────────────────────── */

  /* blobs[0..nblobs) — the body pool.  Index 0 is typically a
   * preset structure (one big tower, or first of two towers);
   * subsequent indices are projectile balls or additional preset
   * bodies.  Insertion appends; deletion (scene_remove_last)
   * decrements nblobs (LIFO, no compaction).  All blobs share the
   * same physics treatment in scene_step — no special-casing for
   * "structure" vs "projectile". */
  Blob blobs[MAX_BLOBS];
  int nblobs;

  /* nballs — cumulative count of balls EVER fired since boot (NOT
   * the current number alive).  Drives the colour-slot cycle in
   * fire_projectile so consecutive balls get distinguishable
   * colours; decoupled from nblobs so removing a ball with `x`
   * still leaves the next-fired ball with a fresh colour. */
  int nballs;

  /* preset — which canned tower-layout is currently loaded.  Index
   * in [0, N_PRESETS).  Cycling preset via n/N triggers a full
   * scene_init which tears down the body pool and rebuilds.
   * Stored normalised to [0, N_PRESETS) by the n/N key handlers;
   * scene_init still wraps defensively as a safety net. */
  int preset;

  /* paused — when true, scene_step is skipped entirely each frame.
   * Bodies stay frozen at their current positions; unpause continues
   * from the exact same state with no discontinuity.  Mirrored to
   * the HUD via the "PAUSED " badge. */
  bool paused;

  /* steps — physics sub-steps per render frame.  At STEPS_DEF=3 the
   * solver runs 3× per scene_draw, giving the PBD iteration loop
   * three chances to relax constraints per displayed frame.  More
   * sub-steps = smaller effective dt = less projectile tunneling
   * through thin walls. */
  int steps;

  /* tick — monotonically increasing physics-step counter, reset by
   * scene_init.  Surface for future "every N ticks" hooks; not read
   * by physics math itself. */
  long tick;

  /* rng — xorshift / LCG state for any future spawn jitter.  Re-seeded
   * only at boot; currently unused (all spawn positions are
   * deterministic preset coordinates) but kept on Scene so future
   * features that need RNG don't pollute file scope. */
  uint32_t rng;

  /* gravity_on — global gravity gate.  When false, verlet_predict_all
   * adds 0.f to vy instead of `gravity`; existing motion still
   * decays via `damping` so bodies coast to rest. */
  bool gravity_on;

  /* gravity — downward acceleration per sub-step in px / sub-step².
   * Sized for SIM_FPS=20 with 3 sub-steps: GRAVITY_DEF=0.06
   * produces a comfortable terminal velocity without bodies
   * tunneling through thin walls. */
  float gravity;

  /* damping — velocity retained per sub-step ∈ [0, 1].  Applied at
   * the start of verlet_predict_all on the implicit velocity:
   *   v = (x − x_prev) · damping
   * 0.985 ⇒ retain ~73 %/sec at 20 Hz × 3 sub-steps = 60 Hz —
   * bodies coast visibly but settle within a few seconds. */
  float damping;

  /* pbd_iters — number of constraint-projection passes per sub-step.
   * In our interleaved scene_step this ALSO controls collision-pass
   * count (one collision pass per PBD iter), so it has DOUBLE the
   * impact on stiffness compared to a non-interleaved solver.
   * Default 10 makes the anchored slabs feel rigid until ball
   * impact tears constraints; user nudges with i/I at runtime. */
  int pbd_iters;

  /* break_thresh — global stretch-ratio threshold copied to every
   * new constraint via blob_add_con.  Mutating this DOESN'T affect
   * already-built bodies (their cons captured the old value at
   * build time); to apply a new threshold you re-build the preset.
   * The r key does exactly this, so the b/B → r workflow is
   * "lower threshold, reset, observe more fragile behaviour". */
  float break_thresh;

  /* ── Rendering parameters ───────────────────────────────────────── */

  /* theme — index into k_themes[] (§3); cycled forward by 't',
   * backward by 'T'.  Both handlers call theme_apply() which
   * overwrites the colour-pair RGB.  Body positions, broken
   * constraints, pinned flags, and rng state are all UNTOUCHED —
   * this is what makes theme purely cosmetic. */
  int theme;

  /* fps_disp — rolling 1-second frame-rate readout displayed on the
   * top HUD.  Updated by main()'s frame counter once per second;
   * the renderer just reads it.  Decoupled from physics tick rate
   * — measures pure render-loop throughput. */
  int fps_disp;
} Scene;

static Scene g_scene = {
    .preset = PRESET_BIG_TOWER,
    .steps = STEPS_DEF,
    .rng = 0xdeadbeef,
    .gravity_on = true,
    .gravity = GRAVITY_DEF,
    .damping = DAMPING_DEF,
    .pbd_iters = PBD_ITERS_DEF,
    .break_thresh = BREAK_THRESH_DEF,
};

/* Screen geometry — main-loop bookkeeping, not Scene state. */
static int g_rows, g_cols;

/* ===================================================================== */
/* §5  physics — Verlet predict + PBD relaxation + boundary response     */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * The textbook PBD step (Müller [1, §3]):
 *
 *     for each body:  verlet_predict_all
 *     for k in 0..pbd_iters:
 *         for each body:  project_distance_constraints + clamp_world
 *         for each pair:  resolve_penetration(A,B) + resolve_penetration(B,A)
 *     for each body:  apply_boundary_velocity_response
 *
 * scene_step (§7) is this loop verbatim.  Helpers below implement each
 * pipeline stage with explicit handling of pinned nodes (skip them) and
 * broken constraints (skip k=0).
 * ─────────────────────────────────────────────────────────────────────── */

/*
 * verlet_predict_all — advance free-flight positions using Verlet:
 *
 *     v ≈ (x − x_prev) * damping
 *     x_prev = x                    (save for next step's velocity)
 *     x = x + v + gravity            (move under inertia + gravity)
 *
 * Pinned nodes skip entirely — they don't have velocity, gravity, or
 * inertia.  We still update px = x so a future un-pin doesn't inherit
 * a stale velocity from before pinning.
 */
static void verlet_predict_all(Blob *bl) {
  float g = g_scene.gravity_on ? g_scene.gravity : 0.f;
  for (int i = 0; i < bl->n_nodes; i++) {
    Node *n = &bl->nodes[i];
    if (n->pinned) {
      n->px = n->x;
      n->py = n->y;
      continue;
    }

    float vx = (n->x - n->px) * g_scene.damping;
    float vy = (n->y - n->py) * g_scene.damping;
    n->px = n->x;
    n->py = n->y;
    n->x += vx;
    n->y += vy + g;
  }
}

/* ─────────────────────────────────────────────────────────────────────── *
 * project_all_distance_constraints pipeline — one PBD iteration.
 *
 * For each constraint we run a 4-stage pipeline:
 *
 *     1. constraint_is_alive(c)          — broken-constraint early-out
 *     2. measure_constraint_distance(a, b) → (dx, dy, d)
 *     3. check_and_break_if_overstretched(c, d)  → maybe set k = 0
 *     4. project_pbd_distance(a, b, c, dx, dy, d)
 *
 * Stage (3) before stage (4) is essential — Provot's flag trick [6]:
 * a constraint exceeding the break threshold gets removed BEFORE the
 * solver applies a correction that might mask the stretch.
 *
 * Stage (4) implements Müller's mass-weighted distance projection
 * [1, eq. 7], which handles pinning uniformly: a pinned node has
 * inverse mass 0 so it absorbs none of the correction; the free
 * counterpart takes the full correction.  No special-case code for
 * pinned vs free pairs — the math handles both.
 * ─────────────────────────────────────────────────────────────────────── */

/* constraint_is_alive — sentinel check: k = 0 means "broken
 * permanently" (Provot's flag).  Skips ~half of constraints on tall
 * fractured presets, so the early-out is worth the branch. */
static inline bool constraint_is_alive(const Con *c) { return c->k != 0.f; }

/* measure_constraint_distance — Euclidean distance between two nodes,
 * with the component-wise vector returned too because both the break
 * check (uses d alone) and the projection (uses dx, dy, d) need it. */
static inline void measure_constraint_distance(const Node *a, const Node *b,
                                               float *dx, float *dy, float *d) {
  *dx = b->x - a->x;
  *dy = b->y - a->y;
  *d = sqrtf((*dx) * (*dx) + (*dy) * (*dy));
}

/* check_and_break_if_overstretched — Provot [6] flag: if the current
 * length exceeds rest · break_thresh, set k = 0 (permanent fracture)
 * and signal the caller to skip this iteration.  break_thresh == 0
 * means "never breaks" (used by projectile sphere). */
static inline bool check_and_break_if_overstretched(Con *c, float d) {
  if (c->break_thresh > 0.f && d > c->rest * c->break_thresh) {
    c->k = 0.f;
    return true;
  }
  return false;
}

/* project_pbd_distance — Müller [1, eq. 7] mass-weighted distance
 * correction.  Pinned nodes have w = 0 and absorb zero correction; if
 * BOTH endpoints are pinned (sum_w = 0) the constraint is rigid
 * between two anchors and we skip it.  Otherwise the correction
 * fraction (err / d) · k splits between the endpoints inversely with
 * their masses.
 *
 * For two free nodes (w_a = w_b = 1), the correction halves between
 * them — the classical Jakobsen [3] half-half split.  For one pinned
 * and one free, the free node takes the full correction. */
static inline void project_pbd_distance(Node *a, Node *b, const Con *c,
                                        float dx, float dy, float d) {
  float wa = a->pinned ? 0.f : 1.f;
  float wb = b->pinned ? 0.f : 1.f;
  float sum_w = wa + wb;
  if (sum_w == 0.f)
    return; /* both pinned */

  float corr = (d - c->rest) / d * c->k / sum_w;
  a->x += dx * corr * wa;
  a->y += dy * corr * wa;
  b->x -= dx * corr * wb;
  b->y -= dy * corr * wb;
}

/*
 * project_all_distance_constraints — top-level orchestrator over the
 * pipeline above.  Reads as pseudocode.
 */
static void project_all_distance_constraints(Blob *bl) {
  for (int i = 0; i < bl->n_cons; i++) {
    Con *c = &bl->cons[i];
    if (!constraint_is_alive(c))
      continue;

    Node *a = &bl->nodes[c->a];
    Node *b = &bl->nodes[c->b];

    float dx, dy, d;
    measure_constraint_distance(a, b, &dx, &dy, &d);
    if (d < 1e-6f)
      continue; /* degenerate */

    if (check_and_break_if_overstretched(c, d))
      continue;

    project_pbd_distance(a, b, c, dx, dy, d);
  }
}

/*
 * clamp_all_nodes_to_world — keep free nodes inside the rectangular
 * arena.  Pinned nodes are presumed to be already in valid positions
 * (the preset builder sets them).
 */
static void clamp_all_nodes_to_world(Blob *bl, float ww, float wh) {
  for (int i = 0; i < bl->n_nodes; i++) {
    Node *n = &bl->nodes[i];
    if (n->pinned)
      continue;
    if (n->x < 0.f)
      n->x = 0.f;
    if (n->x > ww)
      n->x = ww;
    if (n->y < HUD_TOP_PX)
      n->y = HUD_TOP_PX;
    if (n->y > wh)
      n->y = wh;
  }
}

/*
 * apply_boundary_velocity_response — wall / floor bounce, applied to
 * free nodes only.  Reflects the implicit Verlet velocity by tweaking
 * px/py so that (x − px) flips sign with a restitution factor.  Floor
 * additionally damps horizontal velocity (Coulomb-ish friction).
 */
static void apply_boundary_velocity_response(Blob *bl, float ww, float wh) {
  for (int i = 0; i < bl->n_nodes; i++) {
    Node *n = &bl->nodes[i];
    if (n->pinned)
      continue;
    float vx = n->x - n->px;
    float vy = n->y - n->py;

    /* Floor: reflect vy, friction on vx. */
    if (n->y >= wh - 0.01f && vy > 0.f) {
      n->py = n->y + vy * FLOOR_REST;
      n->px = n->x - vx * FLOOR_FRIC;
    }
    /* Ceiling. */
    if (n->y <= HUD_TOP_PX + 0.01f && vy < 0.f)
      n->py = n->y + vy * WALL_REST;
    /* Left wall. */
    if (n->x <= 0.01f && vx < 0.f)
      n->px = n->x + vx * WALL_REST;
    /* Right wall. */
    if (n->x >= ww - 0.01f && vx > 0.f)
      n->px = n->x - vx * WALL_REST;
  }
}

/* ===================================================================== */
/* §6  collision — body-vs-body                                          */
/* ===================================================================== */

/*
 * point_in_polygon_jordan — Jordan curve theorem ray-cast (Sunday [5]).
 * Counts horizontal-ray crossings against the boundary polygon edges;
 * odd = inside, even = outside.  Edge endpoint handling avoids the
 * vertex-grazing degeneracy via the asymmetric (ay <= py < by) test.
 */
static bool point_in_polygon_jordan(const Blob *bl, float px, float py) {
  int crossings = 0, nb = bl->n_bnd;
  for (int i = 0; i < nb; i++) {
    int j = (i + 1) % nb;
    float ax = bl->nodes[bl->bnd[i]].x, ay = bl->nodes[bl->bnd[i]].y;
    float bx = bl->nodes[bl->bnd[j]].x, by = bl->nodes[bl->bnd[j]].y;
    if ((ay <= py && by > py) || (by <= py && ay > py)) {
      float t = (py - ay) / (by - ay);
      if (px < ax + t * (bx - ax))
        crossings++;
    }
  }
  return (crossings & 1) != 0;
}

/* ─────────────────────────────────────────────────────────────────────── *
 * nearest_polygon_edge pipeline — closest-point-on-polygon search.
 *
 * For each boundary edge (A, B) of the polygon:
 *
 *     1. parametric_projection_clamped(P, A, B)    → t ∈ [0, 1]
 *     2. point_on_segment_at(A, B, t)              → closest cell point
 *     3. euclidean_distance(P, closest)            → distance d
 *     4. if d < best: keep_as_new_best
 *
 * The clamped-parametric form (Ericson [4, §5.1.5]) handles edges
 * where the foot-of-perpendicular falls outside the segment: instead
 * of returning the perpendicular projection (which would be on the
 * infinite line), we clamp t to [0, 1] so the closest point is one
 * of the two endpoints when the perpendicular misses.
 * ─────────────────────────────────────────────────────────────────────── */

/* parametric_projection_clamped — foot-of-perpendicular t for P onto
 * segment A→B, clamped to [0, 1] so the result stays on the segment.
 * Returns the canonical parametric coordinate where t=0 is A, t=1 is B
 * (Ericson [4, §5.1.5]). */
static inline float parametric_projection_clamped(float px, float py, float ax,
                                                  float ay, float edx,
                                                  float edy, float len2) {
  float t = ((px - ax) * edx + (py - ay) * edy) / len2;
  if (t < 0.f)
    t = 0.f;
  if (t > 1.f)
    t = 1.f;
  return t;
}

/* point_on_segment_at — linear interpolation along edge: c = A + t·edge.
 * Writes the cell coordinates into the out-params. */
static inline void point_on_segment_at(float ax, float ay, float edx, float edy,
                                       float t, float *cx, float *cy) {
  *cx = ax + t * edx;
  *cy = ay + t * edy;
}

/* euclidean_distance — √(Δx² + Δy²) in 2D. */
static inline float euclidean_distance(float px, float py, float qx, float qy) {
  float dx = px - qx, dy = py - qy;
  return sqrtf(dx * dx + dy * dy);
}

/* outward_unit_normal — from a closest-point on the polygon edge, the
 * unit vector pointing TOWARD the query point (i.e. "outward" from
 * the polygon if the query is outside, "the direction we'd push OUT"
 * if inside).  Degenerate d ≈ 0 falls back to (0, 1) so callers always
 * get a unit-length result. */
static inline void outward_unit_normal(float px, float py, float cx, float cy,
                                       float d, float *nx, float *ny) {
  if (d > 1e-6f) {
    *nx = (px - cx) / d;
    *ny = (py - cy) / d;
  } else {
    *nx = 0.f;
    *ny = 1.f;
  }
}

/*
 * nearest_polygon_edge — orchestrator.  Reads as the pipeline at the
 * top of this block: clamp t, interpolate, distance, keep-best.
 * Returns through out-params (nx, ny, depth, ea, eb) so resolve_
 * penetration can use the same query result for both the push
 * direction AND identifying the two B nodes to deform inward.
 */
static void nearest_polygon_edge(const Blob *bl, float px, float py, float *nx,
                                 float *ny, float *depth, int *ea, int *eb) {
  float best = 1e9f;
  *ea = bl->bnd[0];
  *eb = bl->bnd[1];
  *nx = 1.f;
  *ny = 0.f;
  *depth = 0.f;
  int nb = bl->n_bnd;

  for (int i = 0; i < nb; i++) {
    int j = (i + 1) % nb;
    float ax = bl->nodes[bl->bnd[i]].x, ay = bl->nodes[bl->bnd[i]].y;
    float bx = bl->nodes[bl->bnd[j]].x, by = bl->nodes[bl->bnd[j]].y;
    float edx = bx - ax, edy = by - ay;
    float len2 = edx * edx + edy * edy;
    if (len2 < 1e-8f)
      continue; /* degenerate edge */

    float t = parametric_projection_clamped(px, py, ax, ay, edx, edy, len2);
    float cx, cy;
    point_on_segment_at(ax, ay, edx, edy, t, &cx, &cy);
    float d = euclidean_distance(px, py, cx, cy);

    if (d < best) {
      best = d;
      *depth = d;
      outward_unit_normal(px, py, cx, cy, d, nx, ny);
      *ea = bl->bnd[i];
      *eb = bl->bnd[j];
    }
  }
}

/* ─────────────────────────────────────────────────────────────────────── *
 * resolve_penetration pipeline — A→B directional pass.
 *
 * For each free (un-pinned) node of A:
 *
 *     1. point_in_polygon_jordan(B, node)        — detect overlap
 *     2. nearest_polygon_edge(B, node)           — outward normal + depth
 *     3. compute_newton3_push_split(depth)       — (push_out, push_in)
 *     4. push_penetrator_outward(A.node, n, mag)
 *     5. squish_receiver_polygon_inward(B, ea, eb, n, mag)
 *
 * The Newton's-3rd-law split (Müller [1, §5]) gives 50 % of the
 * separation to A and 25 % to each of B's two nearest edge nodes —
 * total displacement equals the depth, no momentum injection.  Pinned
 * nodes on either side skip; they're infinite-mass anchors and Newton's
 * 3rd partner has nothing to grip on the receiving side.
 *
 * Interleaved with PBD passes in scene_step (§7): each inward-squish
 * of B is immediately relaxed by B's own distance constraints before
 * the next collision push — that's what makes the receiving body look
 * "jelly-elastic" without devolving into liquid.
 * ─────────────────────────────────────────────────────────────────────── */

/* compute_newton3_push_split — given a penetration depth, decide how
 * much to push the penetrator OUT vs how much to squish each of the
 * receiver's two nearest edge nodes IN.
 *
 *   push_out = (depth + skin) / 2     half goes to the penetrator
 *   push_in  = depth / 4              quarter to each of B's two nodes
 *
 * Total displacement: push_out + 2·push_in = depth + skin/2 ≈ depth,
 * so the bodies separate cleanly with a small skin gap that prevents
 * re-overlap on the next sub-step's Verlet velocity. */
static inline void compute_newton3_push_split(float depth, float *push_out,
                                              float *push_in) {
  *push_out = (depth + 0.5f) * 0.5f;
  *push_in = depth * 0.25f;
}

/* push_penetrator_outward — move a node along the outward unit normal
 * by `mag` pixels.  Caller has already verified the node is not pinned. */
static inline void push_penetrator_outward(Node *node, float nx, float ny,
                                           float mag) {
  node->x -= nx * mag;
  node->y -= ny * mag;
}

/* squish_receiver_polygon_inward — push both endpoints of the receiver's
 * closest edge INWARD along the outward normal direction (i.e. in the
 * +n direction, which moves the polygon boundary toward the polygon
 * interior).  Pinned endpoints are skipped — anchors don't deform. */
static inline void squish_receiver_polygon_inward(Blob *b, int ea, int eb,
                                                  float nx, float ny,
                                                  float mag) {
  Node *nea = &b->nodes[ea];
  Node *neb = &b->nodes[eb];
  if (!nea->pinned) {
    nea->x += nx * mag;
    nea->y += ny * mag;
  }
  if (!neb->pinned) {
    neb->x += nx * mag;
    neb->y += ny * mag;
  }
}

/*
 * resolve_penetration — orchestrator for one directional pass A→B.
 * The caller (scene_step) alternates resolve_penetration(A, B) and
 * resolve_penetration(B, A) so both bodies get a chance to push
 * symmetrically — Newton's 3rd across the pair, not just per call.
 */
static void resolve_penetration(Blob *a, Blob *b) {
  for (int i = 0; i < a->n_nodes; i++) {
    Node *ni = &a->nodes[i];
    if (ni->pinned)
      continue; /* anchors immovable */

    if (!point_in_polygon_jordan(b, ni->x, ni->y))
      continue;

    float nx, ny, depth;
    int ea, eb;
    nearest_polygon_edge(b, ni->x, ni->y, &nx, &ny, &depth, &ea, &eb);

    float push_out, push_in;
    compute_newton3_push_split(depth, &push_out, &push_in);

    push_penetrator_outward(ni, nx, ny, push_out);
    squish_receiver_polygon_inward(b, ea, eb, nx, ny, push_in);
  }
}

/* ===================================================================== */
/* §7  scene — preset builders + projectile + scene_step                 */
/* ===================================================================== */

static const char *k_preset_names[N_PRESETS] = {
    "Big Tower",
    "Twin Towers",
    "Mini Tower",
    "Big + Small",
};

/*
 * blob_add_con — append one distance constraint with rest length
 * computed from the current node positions.  break_thresh = 0 makes
 * it permanent (never snaps); positive values activate tearing.
 */
static void blob_add_con(Blob *bl, int a, int b, float k, float break_thresh) {
  if (bl->n_cons >= BLOB_CONS_MAX)
    return;
  float dx = bl->nodes[b].x - bl->nodes[a].x;
  float dy = bl->nodes[b].y - bl->nodes[a].y;
  bl->cons[bl->n_cons++] = (Con){.a = a,
                                 .b = b,
                                 .rest = sqrtf(dx * dx + dy * dy),
                                 .k = k,
                                 .break_thresh = break_thresh};
}

/* ─────────────────────────────────────────────────────────────────────── *
 * blob_build_grid pipeline — generic w×h grid soft-body builder.
 *
 *     1. blob_clear_and_metadata          — wipe + assign render slots
 *     2. lay_out_grid_node_positions      — w·h mass points on a lattice
 *     3. wire_grid_axis_aligned_edges     — STRUCT_K horizontal+vertical pairs
 *     4. wire_grid_cell_diagonals         — SHEAR_K cross-bracing
 *     5. trace_grid_boundary_ccw          — outer ring for collision/render
 *
 * Steps 3-4 define the body's MECHANICAL STIFFNESS: axis-aligned cons
 * resist stretch, diagonal cons resist shear (parallelogram squish).
 * Together they make the grid behave like a quad-mesh continuum.
 * Step 5 defines its COLLISION SILHOUETTE and SCAN-FILL CONTOUR.
 *
 * All cons inherit the same break_thresh so fracture is uniform.
 * Caller pins specific rows after build via pin_grid_bottom_row.
 * ─────────────────────────────────────────────────────────────────────── */

/* blob_clear_and_metadata — wipe the struct and stamp the render-side
 * tags that the renderer will pick up for colouring. */
static void blob_clear_and_metadata(Blob *bl, int scp, int fcp, BKind kind,
                                    int n_nodes) {
  memset(bl, 0, sizeof *bl);
  bl->surf_cp = scp;
  bl->fill_cp = fcp;
  bl->kind = kind;
  bl->n_nodes = n_nodes;
}

/* lay_out_grid_node_positions — w×h mass points on a regular lattice
 * with cell spacing `sp`.  Origin (ox, oy) is the top-left node;
 * column c, row r is placed at (ox + c·sp, oy + r·sp).  Verlet's
 * previous-position is initialised equal to current, so the initial
 * implicit velocity is zero (body at rest). */
static void lay_out_grid_node_positions(Blob *bl, float ox, float oy, int w,
                                        int h, float sp) {
  for (int r = 0; r < h; r++)
    for (int c = 0; c < w; c++) {
      int i = r * w + c;
      bl->nodes[i].x = bl->nodes[i].px = ox + c * sp;
      bl->nodes[i].y = bl->nodes[i].py = oy + r * sp;
    }
}

/* wire_grid_axis_aligned_edges — STRUCTURAL cons connecting each node
 * to its right and bottom neighbour.  These resist STRETCH along the
 * grid axes; together they form the visible outline of every cell. */
static void wire_grid_axis_aligned_edges(Blob *bl, int w, int h,
                                         float break_thresh) {
  for (int r = 0; r < h; r++)
    for (int c = 0; c < w; c++) {
      int i = r * w + c;
      if (c + 1 < w)
        blob_add_con(bl, i, i + 1, STRUCT_K, break_thresh);
      if (r + 1 < h)
        blob_add_con(bl, i, i + w, STRUCT_K, break_thresh);
    }
}

/* wire_grid_cell_diagonals — SHEAR cons across both diagonals of every
 * (r, c)–(r+1, c+1) cell.  These resist parallelogram-squish: without
 * them the grid would collapse into a diamond under load, even with
 * structural edges holding lengths constant.  Cell count = (w-1)·(h-1),
 * cons per cell = 2 → total shear cons = 2·(w-1)·(h-1). */
static void wire_grid_cell_diagonals(Blob *bl, int w, int h,
                                     float break_thresh) {
  for (int r = 0; r < h - 1; r++)
    for (int c = 0; c < w - 1; c++) {
      int i = r * w + c;
      blob_add_con(bl, i, i + w + 1, SHEAR_K, break_thresh); /* \ */
      blob_add_con(bl, i + 1, i + w, SHEAR_K, break_thresh); /* / */
    }
}

/* trace_grid_boundary_ccw — write the outer-perimeter node indices
 * into bl->bnd[] in counter-clockwise order:
 *
 *     top edge      left → right          (row 0)
 *     right edge    top → bottom          (col w-1)
 *     bottom edge   right → left          (row h-1, reversed)
 *     left edge     bottom → top          (col 0, reversed)
 *
 * Used by §6 collision (Jordan curve + closest-point) AND §8 render
 * (scan-fill walks edges to find row-extent bounds).  Both functions
 * tolerate any consistent winding; CCW is the convention here. */
static void trace_grid_boundary_ccw(Blob *bl, int w, int h) {
  bl->n_bnd = 0;
  for (int c = 0; c < w; c++)
    bl->bnd[bl->n_bnd++] = c;
  for (int r = 1; r < h; r++)
    bl->bnd[bl->n_bnd++] = r * w + (w - 1);
  for (int c = w - 2; c >= 0; c--)
    bl->bnd[bl->n_bnd++] = (h - 1) * w + c;
  for (int r = h - 2; r > 0; r--)
    bl->bnd[bl->n_bnd++] = r * w;
}

/*
 * blob_build_grid — orchestrator.  Reads as the 5-stage pseudocode
 * pinned at the top of this block.  All five helpers operate on the
 * same Blob* — no cross-stage data dependencies beyond what's already
 * captured in `bl->nodes[]`.
 */
static void blob_build_grid(Blob *bl, float ox, float oy, int w, int h,
                            float sp, int scp, int fcp, float break_thresh) {
  blob_clear_and_metadata(bl, scp, fcp, BKIND_SLAB, w * h);
  lay_out_grid_node_positions(bl, ox, oy, w, h, sp);
  wire_grid_axis_aligned_edges(bl, w, h, break_thresh);
  wire_grid_cell_diagonals(bl, w, h, break_thresh);
  trace_grid_boundary_ccw(bl, w, h);
}

/*
 * pin_grid_bottom_row — mark every node in the last row (r = h-1) as
 * pinned.  Those nodes become kinematic anchors: gravity, integration,
 * constraint projection, world clamp, and collision push all skip them.
 * Net effect: the body cannot slide horizontally or fall over.
 */
static void pin_grid_bottom_row(Blob *bl, int w, int h) {
  for (int c = 0; c < w; c++) {
    bl->nodes[(h - 1) * w + c].pinned = true;
  }
}

/* ─────────────────────────────────────────────────────────────────────── *
 * blob_build_sphere pipeline — wheel-and-spoke soft sphere builder.
 *
 *     1. blob_clear_and_metadata          — wipe + render slot
 *     2. place_sphere_center_node         — node 0 at (cx, cy)
 *     3. place_sphere_ring_nodes          — SPH_RING nodes on the rim
 *     4. wire_sphere_hoop_cons            — adjacent ring nodes
 *     5. wire_sphere_spoke_cons           — every ring node ↔ centre
 *     6. wire_sphere_diameter_cons        — opposite ring pairs (shear)
 *     7. trace_sphere_ring_boundary       — boundary = ring (centre is
 * interior)
 *
 * Geometry trick (step 3): the ring is an ELLIPSE in pixel space with
 * y-radius = 2·SPH_R because terminal cells are ~2× taller than wide.
 * PHY_TO_ROW halves y, so the ellipse renders as a visual CIRCLE.
 *
 * The projectile ball uses break_thresh = 0 on every constraint — the
 * ball must survive impact and bounce off, not disintegrate.  This is
 * the only kind of body in the demo with non-breakable cons.
 * ─────────────────────────────────────────────────────────────────────── */

/* place_sphere_center_node — node 0 sits at the geometric centre.  It
 * has no rendering glyph of its own (the boundary is the ring, centre
 * is interior to the polygon), but it anchors the spokes that keep
 * the body from collapsing radially under load. */
static void place_sphere_center_node(Blob *bl, float cx, float cy) {
  bl->nodes[0].x = bl->nodes[0].px = cx;
  bl->nodes[0].y = bl->nodes[0].py = cy;
}

/* place_sphere_ring_nodes — SPH_RING nodes evenly spaced around an
 * ellipse with x-radius = SPH_R, y-radius = 2·SPH_R.  Indexed 1..N
 * (node 0 is the centre).  Angular parameter goes 0 → 2π counter-
 * clockwise in math-y; with y-down screen coords this paints CCW on
 * screen too (sin flips sign, so + sin advances y downward which is
 * forward-CCW under y-down). */
static void place_sphere_ring_nodes(Blob *bl, float cx, float cy) {
  for (int i = 0; i < SPH_RING; i++) {
    float a = 6.2831853f * i / SPH_RING;
    int ni = i + 1;
    bl->nodes[ni].x = bl->nodes[ni].px = cx + SPH_R * cosf(a);
    bl->nodes[ni].y = bl->nodes[ni].py = cy + SPH_R * 2.f * sinf(a);
  }
}

/* wire_sphere_hoop_cons — STRUCT_K distance cons between adjacent ring
 * nodes, closing the hoop with the (i, (i+1) mod N) wrap.  These hold
 * the rim's circumference roughly constant — without them the ring
 * would bunch up. */
static void wire_sphere_hoop_cons(Blob *bl) {
  for (int i = 0; i < SPH_RING; i++)
    blob_add_con(bl, 1 + i, 1 + (i + 1) % SPH_RING, STRUCT_K, 0.f);
}

/* wire_sphere_spoke_cons — STRUCT_K cons from EVERY ring node to the
 * centre.  These resist RADIAL deformation: without them, the centre
 * is unconstrained and the body would collapse into the rim. */
static void wire_sphere_spoke_cons(Blob *bl) {
  for (int i = 0; i < SPH_RING; i++)
    blob_add_con(bl, 0, 1 + i, STRUCT_K, 0.f);
}

/* wire_sphere_diameter_cons — SHEAR_K cons between OPPOSITE ring nodes
 * (i, i + N/2).  These resist asymmetric squish (oval collapse) by
 * coupling the two sides of the ring directly across the centre.
 * Only N/2 diameters are added — adding more would double-count
 * the same pair. */
static void wire_sphere_diameter_cons(Blob *bl) {
  for (int i = 0; i < SPH_RING / 2; i++)
    blob_add_con(bl, 1 + i, 1 + i + SPH_RING / 2, SHEAR_K, 0.f);
}

/* trace_sphere_ring_boundary — the visible / collidable silhouette is
 * exactly the ring; the centre node is interior to the polygon and is
 * never on the boundary.  Same CCW orientation as place_sphere_ring_nodes
 * produces, so collision and scan-fill agree on inside vs outside. */
static void trace_sphere_ring_boundary(Blob *bl) {
  for (int i = 0; i < SPH_RING; i++)
    bl->bnd[i] = 1 + i;
  bl->n_bnd = SPH_RING;
}

/*
 * blob_build_sphere — orchestrator.  Reads as the 7-stage pseudocode
 * pinned at the top of this block.
 */
static void blob_build_sphere(Blob *bl, float cx, float cy, int scp, int fcp) {
  blob_clear_and_metadata(bl, scp, fcp, BKIND_SPHERE, SPH_NODES);
  place_sphere_center_node(bl, cx, cy);
  place_sphere_ring_nodes(bl, cx, cy);
  wire_sphere_hoop_cons(bl);
  wire_sphere_spoke_cons(bl);
  wire_sphere_diameter_cons(bl);
  trace_sphere_ring_boundary(bl);
}

/* ── Preset builders  (each spawns 1 or 2 anchored towers) ───────────── */

/* ─────────────────────────────────────────────────────────────────────── *
 * TowerSpec — declarative description of one anchored tower.
 *
 * Intent:
 *   The preset_*() builders each compose 1 OR 2 of these and hand them
 *   to place_tower().  Decoupling the SHAPE description (TowerSpec)
 *   from the PLACEMENT logic (place_tower) means adding a preset is
 *   a 4-line operation: declare the TowerSpec(s), call place_tower().
 *
 * Coordinates are PIXEL space (one cell-row = 2 px); place_tower
 * converts the centre to a top-left origin internally.  The vertical
 * coordinate is implicit — every tower's base sits 0.5 px above the
 * floor regardless of height — because the preset concept is "anchored
 * to the ground".  Floating towers would need a more general spec.
 *
 * Why a struct (vs four positional arguments to place_tower):
 *   - Named fields make preset builders self-documenting:
 *       TowerSpec t = { .w = 6, .h = 18, .sp = 2.5f, .cx = ww * 0.65f };
 *     reads better than place_tower(6, 18, 2.5f, ww * 0.65f).
 *   - Future extensions (e.g. .pin_top, .colour_slot_override) can be
 *     added without touching every preset call-site.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* w — grid COLUMNS of nodes.  Determines horizontal density of the
   * mesh; a 6-column tower has 5 horizontal cells across, 5 columns
   * of structural cons.  Must satisfy w · h ≤ BLOB_NODES_MAX. */
  int w;

  /* h — grid ROWS of nodes.  Determines tower height; bottom row
   * (r = h - 1) is automatically pinned by place_tower so the
   * tower is anchored to the floor. */
  int h;

  /* sp — px between adjacent nodes (horizontal AND vertical
   * spacing — square cells).  Bigger sp → taller / wider visible
   * tower without changing the node count, but constraint count
   * stays the same.  2.0-2.5 is the sweet spot for the destructible
   * jelly aesthetic. */
  float sp;

  /* cx — tower's horizontal CENTRE in world pixel space.  place_tower
   * subtracts half the tower's width to compute the actual ox.  Use
   * `ww * fraction` to make presets terminal-width-agnostic. */
  float cx;
} TowerSpec;

/*
 * place_tower — append one anchored tower to g_scene.blobs[].
 *   1. build a w × h soft grid at the requested horizontal centre,
 *      base just above floor
 *   2. pin its bottom row to the ground
 *   3. assign a color slot cycling through the cool family (0..2)
 *
 * Returns silently on a full pool so preset builders can stay free
 * of capacity checks.
 */
static void place_tower(const TowerSpec *t) {
  if (g_scene.nblobs >= MAX_BLOBS)
    return;

  float wh = (float)(g_rows - 2) * 2.f;
  float bw = (t->w - 1) * t->sp;
  float bh = (t->h - 1) * t->sp;
  float ox = t->cx - bw * 0.5f;
  float oy = wh - bh - 0.5f; /* base just above floor */

  Blob *bl = &g_scene.blobs[g_scene.nblobs];
  int slot = g_scene.nblobs % 3; /* cycle through cool color family */
  blob_build_grid(bl, ox, oy, t->w, t->h, t->sp, CP_BSURF(slot), CP_BFILL(slot),
                  g_scene.break_thresh);
  pin_grid_bottom_row(bl, t->w, t->h);
  g_scene.nblobs++;
}

/*
 * preset_big_tower — one massive tower dominating the right two-thirds
 * of the world.  Ball-at-chest-height hits its lower-middle and the top
 * 60 % tears off.  Most dramatic single-target demo.
 */
static void preset_big_tower(void) {
  float ww = (float)g_cols;
  TowerSpec t = {.w = 6, .h = 18, .sp = 2.5f, .cx = ww * 0.65f};
  place_tower(&t);
}

/*
 * preset_twin_towers — two medium towers in line.  Ball passes the
 * first (may shatter through it), continues to the second.  Tests
 * pair-collision propagation under fracture.
 */
static void preset_twin_towers(void) {
  float ww = (float)g_cols;
  TowerSpec left = {.w = 5, .h = 14, .sp = 2.5f, .cx = ww * 0.45f};
  TowerSpec right = {.w = 5, .h = 14, .sp = 2.5f, .cx = ww * 0.78f};
  place_tower(&left);
  place_tower(&right);
}

/*
 * preset_mini_tower — one short tower, easy to topple in a single hit.
 * Good for fine-tuning break_thresh: lower it with B until even the
 * mini tower disintegrates, raise with b to make it bouncy.
 */
static void preset_mini_tower(void) {
  float ww = (float)g_cols;
  TowerSpec t = {.w = 4, .h = 10, .sp = 2.0f, .cx = ww * 0.70f};
  place_tower(&t);
}

/*
 * preset_big_and_small — David and Goliath: a small tower on the left
 * and a big one on the right.  Ball hits the small tower first
 * (probably knocks it over completely), loses energy, then hits the
 * big tower with a weaker impact (tears off less).  Demonstrates how
 * the projectile's accumulated damage modulates downstream destruction.
 */
static void preset_big_and_small(void) {
  float ww = (float)g_cols;
  TowerSpec small_t = {.w = 4, .h = 10, .sp = 2.0f, .cx = ww * 0.40f};
  TowerSpec big_t = {.w = 6, .h = 18, .sp = 2.5f, .cx = ww * 0.78f};
  place_tower(&small_t);
  place_tower(&big_t);
}

/*
 * fire_projectile — SPACE-bound action.  Builds a soft sphere just
 * inside the left wall at chest height (cy = wh − 3·SPH_R − 1 so the
 * ball doesn't immediately touch the floor and get friction-killed),
 * then injects horizontal Verlet velocity by setting every node's
 * px = x − PROJECTILE_VX (Jakobsen [3]: vel ≈ x − px so this is a
 * uniform translation velocity).
 *
 * The ball is NOT pinned and its constraints have break_thresh = 0,
 * so it survives impact and bounces around as a soft ball should.
 */
static void fire_projectile(void) {
  if (g_scene.nblobs >= MAX_BLOBS)
    return;
  float wh = (float)(g_rows - 2) * 2.f;
  float cx = SPH_R + 1.f;            /* just inside left wall */
  float cy = wh - SPH_R * 3.f - 1.f; /* chest height          */

  /* Color slot 3..5 for spheres (warm family). */
  int slot = 3 + (g_scene.nballs % 3);
  Blob *bl = &g_scene.blobs[g_scene.nblobs];
  blob_build_sphere(bl, cx, cy, CP_BSURF(slot), CP_BFILL(slot));

  for (int i = 0; i < bl->n_nodes; i++)
    bl->nodes[i].px = bl->nodes[i].x - PROJECTILE_VX;

  g_scene.nblobs++;
  g_scene.nballs++;
}

/*
 * scene_remove_last — drop the last spawned body.  Useful for clearing
 * old projectile balls without resetting the preset.
 */
static void scene_remove_last(void) {
  if (g_scene.nblobs > 0)
    g_scene.nblobs--;
}

/*
 * scene_init — dispatch: clear the body pool, build the preset structure,
 * reset clocks.  Called at boot, on r/R reset, on every n/N preset cycle,
 * and after SIGWINCH resize (so the new structure scales to the viewport).
 */
static void scene_init(int preset) {
  g_scene.nblobs = 0;
  g_scene.nballs = 0;
  g_scene.tick = 0;
  g_scene.preset = ((preset % N_PRESETS) + N_PRESETS) % N_PRESETS;

  switch ((Preset)g_scene.preset) {
  case PRESET_BIG_TOWER:
    preset_big_tower();
    break;
  case PRESET_TWIN_TOWERS:
    preset_twin_towers();
    break;
  case PRESET_MINI_TOWER:
    preset_mini_tower();
    break;
  case PRESET_BIG_AND_SMALL:
    preset_big_and_small();
    break;
  }
}

/* ─────────────────────────────────────────────────────────────────────── *
 * scene_step pipeline — INTERLEAVED PBD + collision (Müller [1, §3]).
 *
 *     for substep in 1..steps:
 *         predict_all_free_flight()              // Verlet predict per body
 *         for k in 1..pbd_iters:
 *             relax_all_distance_constraints()   // PBD per body
 *             resolve_all_pairwise_contacts()    // collision per pair (×2)
 *         apply_arena_velocity_response()         // walls/floor reflection
 *     tick++
 *
 * Interleaving (steps 2-3 alternating inside the k loop) is the entire
 * reason multi-body stacks don't liquefy — every collision push gets
 * an immediate PBD pass before the next collision happens, so receiving
 * bodies can recover shape between contacts.  The OUTER substep loop
 * exists separately for fast projectiles: at high vx the per-substep
 * displacement must stay smaller than the receiver's wall thickness
 * to avoid tunneling.
 * ─────────────────────────────────────────────────────────────────────── */

/* predict_all_free_flight — apply Verlet predict to every body.  Skips
 * pinned nodes internally (verlet_predict_all checks). */
static void predict_all_free_flight(void) {
  for (int i = 0; i < g_scene.nblobs; i++)
    verlet_predict_all(&g_scene.blobs[i]);
}

/* relax_all_distance_constraints — one PBD iteration over every body's
 * constraint network, followed by an arena-clamp so post-projection
 * positions stay inside the world.  Clamping AFTER projection ensures
 * the projection's correction isn't undone by a stale clamp from a
 * previous iter. */
static void relax_all_distance_constraints(float ww, float wh) {
  for (int i = 0; i < g_scene.nblobs; i++) {
    project_all_distance_constraints(&g_scene.blobs[i]);
    clamp_all_nodes_to_world(&g_scene.blobs[i], ww, wh);
  }
}

/* resolve_all_pairwise_contacts — for each unordered body pair, do both
 * directional passes A→B and B→A so Newton's 3rd is enforced across
 * the pair.  O(nblobs²) inner loop; small N (≤ MAX_BLOBS = 16) so this
 * doesn't dominate. */
static void resolve_all_pairwise_contacts(void) {
  for (int i = 0; i < g_scene.nblobs; i++) {
    for (int j = i + 1; j < g_scene.nblobs; j++) {
      resolve_penetration(&g_scene.blobs[i], &g_scene.blobs[j]);
      resolve_penetration(&g_scene.blobs[j], &g_scene.blobs[i]);
    }
  }
}

/* apply_arena_velocity_response — after the inner PBD+collision loop
 * has converged, reflect velocities off the arena walls/floor for
 * every body (skipping pinned nodes internally).  Runs ONCE per
 * substep at the END so reflection sees the final post-projection
 * velocity, not an intermediate. */
static void apply_arena_velocity_response(float ww, float wh) {
  for (int i = 0; i < g_scene.nblobs; i++)
    apply_boundary_velocity_response(&g_scene.blobs[i], ww, wh);
}

/*
 * scene_step — orchestrator.  Reads as the pseudocode pinned at the
 * top of this block.  3 substeps × 10 PBD-iters = 30 distance-and-
 * collision relaxation passes per render frame at defaults.
 */
static void scene_step(void) {
  float ww = (float)g_cols - 1.f;
  float wh = (float)(g_rows - 2) * 2.f;

  for (int s = 0; s < g_scene.steps; s++) {
    predict_all_free_flight();

    for (int k = 0; k < g_scene.pbd_iters; k++) {
      relax_all_distance_constraints(ww, wh);
      resolve_all_pairwise_contacts();
    }

    apply_arena_velocity_response(ww, wh);
  }
  g_scene.tick++;
}

/* ===================================================================== */
/* §8  draw — body + HUD                                                  */
/*                                                                       */
/* Pure ncurses output (Padala [7]) — init_pair already wired in §3,    */
/* this section just calls attron/mvaddch/clrtoeol against stdscr.       */
/* ===================================================================== */

static int g_mincol[ROWS_MAX], g_maxcol[ROWS_MAX];

/* Bresenham-style edge walk that updates the scan-fill bounds [g_mincol[r],
 * g_maxcol[r]] for each row the edge crosses.  Used to scan-fill the
 * polygon interior after walking all boundary edges. */
static void scanfill_edge(int x0, int y0, int x1, int y1, int fr, int cols) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy, cx = x0, cy = y0;
  for (;;) {
    if (cy >= 0 && cy < fr && cx >= 0 && cx < cols) {
      if (cx < g_mincol[cy])
        g_mincol[cy] = cx;
      if (cx > g_maxcol[cy])
        g_maxcol[cy] = cx;
    }
    if (cx == x1 && cy == y1)
      break;
    int e2 = 2 * err;
    if (e2 >= dy) {
      if (cx == x1)
        break;
      err += dy;
      cx += sx;
    }
    if (e2 <= dx) {
      if (cy == y1)
        break;
      err += dx;
      cy += sy;
    }
  }
}

/* Standard Bresenham line draw — used for the constraint wireframe. */
static void bresenham(int x0, int y0, int x1, int y1, int fr, int cols,
                      chtype ch) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy, cx = x0, cy = y0;
  for (;;) {
    if (cy >= 0 && cy < fr && cx >= 0 && cx < cols)
      mvaddch(cy, cx, ch);
    if (cx == x1 && cy == y1)
      break;
    int e2 = 2 * err;
    if (e2 >= dy) {
      if (cx == x1)
        break;
      err += dy;
      cx += sx;
    }
    if (e2 <= dx) {
      if (cy == y1)
        break;
      err += dx;
      cy += sy;
    }
  }
}

/* ─────────────────────────────────────────────────────────────────────── *
 * draw_blob pipeline — 3-layer body render.
 *
 *     layer 1: paint_scanfill_interior      (fill_cp, ':' inside polygon)
 *     layer 2: stroke_constraint_wireframe  (surf_cp, '|/-\' on each con)
 *     layer 3: dot_boundary_nodes           ('*' pinned / 'O' free, bold)
 *
 * Order matters because ncurses overwrites cells:
 *   - Interior first (bottom layer, lightest)
 *   - Wireframe second (mid layer, brighter)
 *   - Boundary nodes last (top layer, bold accents on top)
 *
 * Broken constraints (k = 0) are skipped at layer 2 — fracture lines
 * become visible as gaps in the wire mesh.
 * ─────────────────────────────────────────────────────────────────────── */

/* compute_polygon_row_extents — walk every boundary edge with a
 * scan-conversion Bresenham, tracking the leftmost and rightmost
 * column visited on each screen row.  Populates the file-scope
 * g_mincol[r] / g_maxcol[r] arrays so the interior fill knows the
 * horizontal extent of the polygon at every row. */
static void compute_polygon_row_extents(const Blob *bl, int floor_row,
                                        int cols) {
  for (int r = 0; r < floor_row; r++) {
    g_mincol[r] = cols;
    g_maxcol[r] = -1;
  }
  for (int i = 0; i < bl->n_bnd; i++) {
    int j = (i + 1) % bl->n_bnd;
    const Node *a = &bl->nodes[bl->bnd[i]];
    const Node *b = &bl->nodes[bl->bnd[j]];
    scanfill_edge(PHY_TO_COL(a->x), PHY_TO_ROW(a->y), PHY_TO_COL(b->x),
                  PHY_TO_ROW(b->y), floor_row, cols);
  }
}

/* paint_scanfill_interior — for every row that the polygon covers,
 * fill cells strictly INSIDE the boundary (mincol+1 .. maxcol-1) with
 * ':' in the fill colour.  The boundary cells themselves are left
 * blank so the wireframe in layer 2 paints them. */
static void paint_scanfill_interior(const Blob *bl, int floor_row, int cols) {
  compute_polygon_row_extents(bl, floor_row, cols);

  attron(COLOR_PAIR(bl->fill_cp));
  for (int r = 0; r < floor_row; r++) {
    if (g_maxcol[r] <= g_mincol[r])
      continue;
    for (int c = g_mincol[r] + 1; c < g_maxcol[r]; c++)
      mvaddch(r, c, ':');
  }
  attroff(COLOR_PAIR(bl->fill_cp));
}

/* slope_glyph_for_edge — pick the ASCII slash character that best
 * matches a line's direction:
 *
 *     vertical          → '|'
 *     horizontal        → '-'
 *     down-right slope  → '\\'  (Δx and Δy same sign)
 *     down-left slope   → '/'   (Δx and Δy opposite sign)
 *
 * The classic "diagonal arrows in ASCII" mapping. */
static inline chtype slope_glyph_for_edge(int x0, int y0, int x1, int y1) {
  int adx = abs(x1 - x0), ady = abs(y1 - y0);
  if (adx == 0)
    return '|';
  if (ady == 0)
    return '-';
  return ((x1 - x0) * (y1 - y0) > 0) ? '\\' : '/';
}

/* stroke_constraint_wireframe — for every INTACT constraint (k > 0),
 * Bresenham a line between its two endpoints using the slope-derived
 * glyph.  Broken constraints (k = 0) are skipped so torn regions
 * appear as gaps in the wire mesh — the visual signature of fracture. */
static void stroke_constraint_wireframe(const Blob *bl, int floor_row,
                                        int cols) {
  attron(COLOR_PAIR(bl->surf_cp));
  for (int ci = 0; ci < bl->n_cons; ci++) {
    if (bl->cons[ci].k == 0.f)
      continue; /* broken */
    const Node *a = &bl->nodes[bl->cons[ci].a];
    const Node *b = &bl->nodes[bl->cons[ci].b];
    int x0 = PHY_TO_COL(a->x), y0 = PHY_TO_ROW(a->y);
    int x1 = PHY_TO_COL(b->x), y1 = PHY_TO_ROW(b->y);
    bresenham(x0, y0, x1, y1, floor_row, cols,
              slope_glyph_for_edge(x0, y0, x1, y1));
  }
  attroff(COLOR_PAIR(bl->surf_cp));
}

/* draw_one_boundary_node — bold 'O' for free nodes, bold '*' in the
 * theme's pin accent for pinned (anchored) nodes.  Distinct glyph
 * makes the foundation immediately visible. */
static inline void draw_one_boundary_node(const Node *nd, int row, int col,
                                          int surf_cp) {
  if (nd->pinned) {
    attron(COLOR_PAIR(CP_PIN) | A_BOLD);
    mvaddch(row, col, '*');
    attroff(COLOR_PAIR(CP_PIN) | A_BOLD);
  } else {
    attron(COLOR_PAIR(surf_cp) | A_BOLD);
    mvaddch(row, col, 'O');
    attroff(COLOR_PAIR(surf_cp) | A_BOLD);
  }
}

/* dot_boundary_nodes — render every boundary node on top of the
 * preceding two layers.  Bound-checked so off-screen nodes (which can
 * happen for projectile balls partially in the HUD or floor row) are
 * silently dropped. */
static void dot_boundary_nodes(const Blob *bl, int floor_row, int cols) {
  for (int i = 0; i < bl->n_bnd; i++) {
    const Node *nd = &bl->nodes[bl->bnd[i]];
    int cr = PHY_TO_ROW(nd->y);
    int cc = PHY_TO_COL(nd->x);
    if (cr < 0 || cr >= floor_row || cc < 0 || cc >= cols)
      continue;
    draw_one_boundary_node(nd, cr, cc, bl->surf_cp);
  }
}

/*
 * draw_blob — orchestrator.  Three layers in deliberate bottom-to-top
 * order so the boundary nodes land on top of the wireframe which
 * lands on top of the scan-fill.
 */
static void draw_blob(const Blob *bl, int floor_row, int cols) {
  paint_scanfill_interior(bl, floor_row, cols);
  stroke_constraint_wireframe(bl, floor_row, cols);
  dot_boundary_nodes(bl, floor_row, cols);
}

/* ─────────────────────────────────────────────────────────────────────── *
 * HUD — top status (row 0, right-aligned, yellow + bold) and bottom
 * action bar (row rows-1, left, cyan + bold).  CLAUDE.md convention.
 * ─────────────────────────────────────────────────────────────────────── */

static int count_intact_cons(const Blob *bl) {
  int n = 0;
  for (int i = 0; i < bl->n_cons; i++)
    if (bl->cons[i].k > 0.f)
      n++;
  return n;
}

static void draw_hud_top(int fps) {
  /* Count broken constraints across all bodies to give the user
   * direct feedback on "how much damage has happened". */
  int total = 0, broken = 0;
  for (int i = 0; i < g_scene.nblobs; i++) {
    total += g_scene.blobs[i].n_cons;
    broken += g_scene.blobs[i].n_cons - count_intact_cons(&g_scene.blobs[i]);
  }
  char buf[200];
  snprintf(buf, sizeof buf,
           " preset:%s  bodies:%d/%d  broken:%d/%d  iters:%d  break:%.2f  "
           "grav:%s  theme:%s  %s  %d fps ",
           k_preset_names[g_scene.preset], g_scene.nblobs, MAX_BLOBS, broken,
           total, g_scene.pbd_iters, g_scene.break_thresh,
           g_scene.gravity_on ? "on" : "off", k_themes[g_scene.theme].name,
           g_scene.paused ? "PAUSED " : "running", fps);
  int col = g_cols - (int)strlen(buf);
  if (col < 0)
    col = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, col, buf, g_cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

static void draw_hud_bottom(void) {
  const char *full = " q:quit  SPACE:fire  n/N:preset  r:reset  p:pause  "
                     "x:del  g:grav  i/I:iters  b/B:break  t/T:theme ";
  const char *shrt =
      " q:quit  SPACE:fire  n:preset  r:reset  b/B:break  t:theme ";
  const char *h = (int)strlen(full) >= g_cols - 1 ? shrt : full;
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(g_rows - 1, 0, h, g_cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void scene_draw(void) {
  erase();
  int rows = g_rows, cols = g_cols, floor_row = rows - 2;

  /* Floor line. */
  attron(COLOR_PAIR(CP_FLOOR));
  for (int c = 0; c < cols; c++)
    mvaddch(floor_row, c, '=');
  attroff(COLOR_PAIR(CP_FLOOR));

  /* Bodies. */
  for (int i = 0; i < g_scene.nblobs; i++)
    draw_blob(&g_scene.blobs[i], floor_row, cols);

  /* HUD chrome. */
  draw_hud_top(g_scene.fps_disp);
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
  use_default_colors();
  g_has_256 = (COLORS >= 256);
  theme_apply(g_scene.theme);
}

static void screen_resize(void) {
  endwin();
  refresh();
  getmaxyx(stdscr, g_rows, g_cols);
  if (g_rows > ROWS_MAX)
    g_rows = ROWS_MAX;
  if (g_cols > COLS_MAX)
    g_cols = COLS_MAX;
  g_resize = 0;
}

/* ===================================================================== */
/* §10  app                                                               */
/* ===================================================================== */

int main(void) {
  signal(SIGWINCH, on_sigwinch);
  signal(SIGTERM, on_sigterm);
  signal(SIGINT, on_sigterm);

  screen_init();
  getmaxyx(stdscr, g_rows, g_cols);
  if (g_rows > ROWS_MAX)
    g_rows = ROWS_MAX;
  if (g_cols > COLS_MAX)
    g_cols = COLS_MAX;

  scene_init(g_scene.preset);

  int64_t next_tick = clock_ns();
  int64_t fps_window_start = next_tick;
  int fps_frames = 0;

  while (!g_quit) {

    if (g_resize) {
      screen_resize();
      scene_init(g_scene.preset);
    }

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
      case 'n':
        scene_init(g_scene.preset + 1);
        break;
      case 'N':
        scene_init(g_scene.preset - 1 + N_PRESETS);
        break;
      case 'x':
        scene_remove_last();
        break;
      case 'g':
        g_scene.gravity_on = !g_scene.gravity_on;
        break;
      case 'i':
        if (g_scene.pbd_iters < PBD_ITERS_MAX)
          g_scene.pbd_iters++;
        break;
      case 'I':
        if (g_scene.pbd_iters > PBD_ITERS_MIN)
          g_scene.pbd_iters--;
        break;
      case 'b':
        if (g_scene.break_thresh < BREAK_THRESH_MAX)
          g_scene.break_thresh += BREAK_THRESH_STEP;
        break;
      case 'B':
        if (g_scene.break_thresh > BREAK_THRESH_MIN)
          g_scene.break_thresh -= BREAK_THRESH_STEP;
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

    int64_t now = clock_ns();
    if (!g_scene.paused && now >= next_tick) {
      scene_step();
      next_tick = now + TICK_NS(SIM_FPS);
    }

    /* rolling 1-second fps window */
    fps_frames++;
    if (now - fps_window_start >= NS_PER_SEC) {
      g_scene.fps_disp = fps_frames;
      fps_frames = 0;
      fps_window_start = now;
    }

    scene_draw();
    wnoutrefresh(stdscr);
    doupdate();
    clock_sleep_ns(next_tick - clock_ns() - 1000000LL);
  }
  endwin();
  return 0;
}
