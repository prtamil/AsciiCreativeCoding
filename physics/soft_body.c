/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * soft_body.c — Jelly Bodies  (Position-Based Dynamics)
 *
 * Multiple soft cubes and spheres with full pairwise collision.
 * All collision types handled by the same generic boundary-polygon test:
 *   cube-cube, cube-sphere, sphere-cube, sphere-sphere,
 *   cube-floor, sphere-floor (floor/wall: per node clamp in blob_step).
 *
 * Physics: Position-Based Dynamics
 *   Verlet predict → project distance constraints × N → clamp walls
 *   Unconditionally stable. Stiffness = iteration count.
 *
 * Collision response (PBD, per penetrating node):
 *   node of A pushed outward by depth/2 along nearest-edge normal
 *   two boundary nodes of B pushed inward by depth/4 each (Newton's 3rd)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/soft_body.c -o soft_body -lncurses
 * -lm
 *
 * Keys:
 *   c   add cube     s   add sphere    x   remove last body
 *   q/ESC quit       p   pause         r   reset (1 cube + 1 sphere)
 *   g   gravity      i/I iterations±   t/T theme
 *
 * Sections: §1 config       §2 clock       §3 color/theme
 *           §4 state         — Node, Con, Blob, Scene + cube/sphere builders
 *           §5 physics       — Verlet predict + PBD constraint solve + walls
 *           §6 collision     — Jordan-curve point-in-polygon + edge resolve
 *           §7 scene         — spawn / remove / init / step
 *           §8 draw          — scan-fill + wireframe + HUD bars
 *           §9 screen        §10 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Position-Based Dynamics (PBD) [1][2] — applied to a 2D
 *                  mesh of nodes forming a soft body.  Per-body constraints:
 *                  structural (keep nodes at rest distance) and shear
 *                  (diagonal links resisting parallelogram-shear).  The
 *                  area-preservation effect emerges from these two without
 *                  an explicit volume constraint.
 *
 * Physics        : Soft-body deformation via Verlet [3] + constraint
 *                  projection.  Unlike explicit spring forces (which need
 *                  per-stiffness dt tuning for stability), PBD constraints
 *                  are projected GEOMETRICALLY — stable for any dt.
 *                  Stiffness is controlled by ITERATION COUNT (more iters
 *                  → stiffer), not by a spring constant.  STRUCT_K=1.0
 *                  applies full structural correction each iteration;
 *                  SHEAR_K=0.8 produces a slightly softer diagonal feel.
 *
 * Collision      : PBD inter-body collision via boundary-polygon test [4].
 *                  Point-in-polygon: even-odd ray-cast (Jordan curve
 *                  theorem) [5].  For each node of body A that penetrates
 *                  body B's polygon, push A's node outward by depth/2
 *                  along the nearest-edge outward normal, and push B's
 *                  two nearest edge nodes inward by depth/4 each —
 *                  Newton's third law in position space.  COLL_ITERS=2
 *                  passes per physics step resolve mutual contacts.
 *
 * Rendering      : Each blob renders in three layers: scan-fill interior
 *                  (':'), Bresenham-rasterised constraint wireframe
 *                  ('|', '-', '\\', '/' chosen by edge slope), and bold
 *                  'O' boundary nodes on top.  All three layers share the
 *                  same theme-driven colour-pair slot.  See [8] for
 *                  ncurses idioms used in the render path.
 *
 * Performance    : COLL_ITERS=2 collision iterations × PBD_ITERS=6
 *                  constraint iterations per physics step.  Total cost
 *                  O(NB² × COLL_ITERS × PBD_ITERS × N) — at 16 blobs ×
 *                  50 nodes this is ~150 k node-ops per step at SIM_FPS=20.
 *
 * References (cite inline as [n]):
 *
 *   [1] Müller, M.; Heidelberger, B.; Hennix, M.; Ratcliff, J. (2007) —
 *       "Position Based Dynamics", *J. Visual Communication and Image
 *       Representation* 18 (2), 109–118.  The seminal PBD paper.
 *       §3 introduces constraint projection; §4 derives the half-half
 *       correction split used by our distance constraints; §5 covers
 *       collision constraints in position space (Newton's 3rd via
 *       symmetric position pushes, exactly what resolve_penetration does).
 *
 *   [2] Bender, J.; Müller, M.; Macklin, M. (2017) — "A Survey on
 *       Position-Based Simulation Methods in Computer Graphics",
 *       Computer Graphics Forum 36 (6).  Modern overview: extended PBD
 *       (XPBD), strain-based dynamics, recent collision-handling
 *       techniques.  Read this for the "where PBD has gone since 2007"
 *       context.
 *
 *   [3] Jakobsen, T. (2001) — "Advanced Character Physics", GDC.
 *       The practical Verlet + iterative-relaxation paper from
 *       Hitman: Codename 47.  Spells out the (x, x_prev) storage
 *       used by Node here: v ≈ x − x_prev is IMPLICIT, no separate
 *       velocity array needed.  Highly readable game-dev primer.
 *
 *   [4] Ericson, C. (2005) — *Real-Time Collision Detection*, Morgan
 *       Kaufmann.  §4.6 covers polygon-polygon overlap; §5.1.5 gives
 *       the "closest point on segment" formula our nearest_polygon_edge uses
 *       (parametric clamp t ∈ [0, 1]); §5.4 covers the friction model
 *       analogous to our FLOOR_FRIC.  THE collision-detection bible.
 *
 *   [5] Sunday, D. (2001) — "Inclusion of a Point in a Polygon",
 *       *Geomalgorithms.com* tutorial.  Open-access write-up of the
 *       Jordan-curve ray-casting test (also known as the even-odd
 *       rule) that point_in_polygon_jordan implements.  Originally formalised
 *       in Hacker (1962) and Shimrat (1962) algorithms; Sunday's
 *       page has the cleanest modern derivation.
 *
 *   [6] Goldstein, H.; Poole, C.; Safko, J. (2002) — *Classical
 *       Mechanics*, 3rd ed., Addison-Wesley.  Ch. 1 covers Newton's
 *       third law and impulsive contact response — the conceptual
 *       backing for "push A out, push B's two edge nodes in" being a
 *       valid position-space analogue of equal-and-opposite impulses.
 *
 *   [7] Witkin, A.; Baraff, D. (2001) — "Physically Based Modeling"
 *       SIGGRAPH 2001 course notes (online).  §5 on rigid-body
 *       collision and §6 on constraints provide the broader context
 *       in which PBD sits as a "lazy" alternative to Lagrange-
 *       multiplier solvers.
 *
 *   [8] Padala, P. — *NCURSES Programming HOWTO*, The Linux
 *       Documentation Project.  Authoritative free reference for
 *       init_pair / COLOR_PAIR semantics, the wnoutrefresh + doupdate
 *       diff model that prevents tearing, and signal-safe SIGWINCH
 *       handling — all used in the render path here.
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

/* Cube */
#define CUBE_W 6
#define CUBE_H 6
#define CUBE_SP 3.0f
#define CUBE_NODES (CUBE_W * CUBE_H) /* 36 */

/* Sphere */
#define SPH_RING 12 /* ring nodes; node 0 = centre */
#define SPH_NODES (SPH_RING + 1)
#define SPH_R 5.0f /* visual col-radius; y-extent = 2×SPH_R */

/* Generic blob limits */
#define BLOB_NODES_MAX 50
#define BLOB_CONS_MAX 250
#define BLOB_BND_MAX 50

/* Scene */
#define MAX_BLOBS 16

/* Color slots: 0-2 = cube family (cool), 3-5 = sphere family (warm) */
#define N_BSLOTS 6
#define CP_BSURF(i) (1 + (i))            /* pairs 1-6  */
#define CP_BFILL(i) (1 + N_BSLOTS + (i)) /* pairs 7-12 */
#define CP_FLOOR (1 + 2 * N_BSLOTS)      /* 13                */
#define CP_HUD (2 + 2 * N_BSLOTS)        /* 14 — top status   */
#define CP_HINT (3 + 2 * N_BSLOTS)       /* 15 — bottom hints */

/* HUD_TOP_PX — pixel units (one cell-row = 2 px) reserved at top of
 * the world for the status bar.  blob_step's ceiling clamp uses this
 * value so soft-body nodes never draw into row 0.  Row g_rows-1 is
 * the bottom action-key bar; floor stays at g_rows-2 with WH unchanged. */
#define HUD_TOP_PX 2.0f

/* PBD */
#define PBD_ITERS_DEF 6
#define PBD_ITERS_MIN 1
#define PBD_ITERS_MAX 20
#define STRUCT_K 1.0f
#define SHEAR_K 0.8f
#define COLL_ITERS 2

/* Physics (per substep) */
#define GRAVITY_DEF 0.06f
#define DAMPING_DEF 0.985f
#define FLOOR_REST 0.12f
#define WALL_REST 0.10f
#define FLOOR_FRIC 0.82f

#define PHY_TO_ROW(y) ((int)((y) * 0.5f + 0.5f))
#define PHY_TO_COL(x) ((int)((x) + 0.5f))

#define STEPS_DEF 3
#define SIM_FPS 20
#define NS_PER_SEC 1000000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

#define N_THEMES 5

/* rng state lives on g_scene.rng (Scene struct, §4) so it sits with
 * the rest of the simulation parameters under the locality contract.
 * Body of rng_f is in §4, right after the Scene typedef. */

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
/* §3  color / theme                                                      */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Theme — palette for one rendering of the soft-body scene.
 *
 * Intent:
 *   Pure visual change.  Cycling themes must leave every Blob byte-
 *   identical (positions, constraints, kind, the works) — only the
 *   colour-pair RGB shifts.  This is the locality rule that lets us
 *   put g_scene.theme in the rendering group on Scene (§4).
 *
 * Why TWO arrays (surf + fill):
 *   Each body is drawn in two passes — the boundary wireframe + nodes
 *   render in the SURFACE colour (bright, A_BOLD), and the scan-fill
 *   interior renders in the FILL colour (dimmer, no bold).  Splitting
 *   gives the bodies a 3D "shaded" look: the outline pops, the body
 *   has visible mass.  A single-colour theme would still work but
 *   would lose the depth cue.
 *
 * Why 6 slots:
 *   Bodies are tagged at spawn with one of 6 colour-pair slots:
 *     slots 0..2  → cube family (cool palette)
 *     slots 3..5  → sphere family (warm palette)
 *   blob_color (§7) cycles within the family by spawn-count modulo 3,
 *   so the first three cubes get distinct colours and the fourth cube
 *   wraps back to slot 0.  Same for spheres.
 *
 * Brightness safety (CLAUDE.md, see documentation/COLOR.md):
 *   Existing palettes use indices 51-255 — all safely above the
 *   forbidden 16-23 / 232-239 bands.  Note that the older background
 *   colour is COLOR_BLACK (not -1) for body pairs — that's deliberate
 *   so the bodies have a solid look on terminals with non-black bg.
 *
 * Algorithm refs:
 *   Colour-pair semantics in ncurses — Padala [8]
 *   ITU-R BT.601 luma ordering — informal heuristic used when ranking
 *                                slot brightness for the audit.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* surf[i] — surface (boundary + wireframe) colour for slot i.
   * Brighter end of the palette so the body's silhouette pops.
   * Slots 0..2 are the cube family, 3..5 are the sphere family. */
  short surf[N_BSLOTS];

  /* fill[i] — interior scan-fill colour for slot i.  Dimmer than
   * surf[i] so the wireframe reads clearly on top.  Typically a
   * shadow / darker variant of the matching surf colour. */
  short fill[N_BSLOTS];

  /* name — short label shown in the top HUD.  ≤ 7 chars so the full
   * status line fits on an 80-column terminal alongside cubes:,
   * spheres:, iters:, grav:, paused/running, fps. */
  const char *name;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* Ocean */
    {{51, 45, 87, 214, 196, 206}, {17, 17, 55, 130, 88, 125}, "Ocean"},
    /* Matrix */
    {{82, 46, 118, 196, 160, 124}, {22, 22, 28, 52, 52, 52}, "Matrix"},
    /* Nebula */
    {{213, 177, 147, 87, 123, 129}, {93, 61, 55, 55, 89, 93}, "Nebula"},
    /* Fire */
    {{226, 220, 214, 171, 135, 99}, {100, 94, 88, 93, 57, 57}, "Fire"},
    /* Mono */
    {{255, 245, 235, 230, 220, 210}, {240, 230, 220, 215, 205, 195}, "Mono"},
};

/* g_has_256 is a boot-time TERMINAL CAPABILITY, set once in screen_init;
 * not scene state.  theme lives on g_scene.theme (§4). */
static bool g_has_256;

static void theme_apply(int ti) {
  const Theme *t = &k_themes[ti];
  for (int i = 0; i < N_BSLOTS; i++) {
    if (g_has_256) {
      init_pair(CP_BSURF(i), t->surf[i], COLOR_BLACK);
      init_pair(CP_BFILL(i), t->fill[i], COLOR_BLACK);
    } else {
      short cs = (i < 3) ? COLOR_CYAN : COLOR_RED;
      short cf = (i < 3) ? COLOR_CYAN : COLOR_RED;
      if (i == 1)
        cs = COLOR_BLUE;
      if (i == 4)
        cs = COLOR_YELLOW;
      init_pair(CP_BSURF(i), cs, COLOR_BLACK);
      init_pair(CP_BFILL(i), cf, COLOR_BLACK);
    }
  }
  init_pair(CP_FLOOR, g_has_256 ? 244 : COLOR_WHITE, COLOR_BLACK);
  /* HUD chrome — fixed across themes per CLAUDE.md HUD convention.
   * Bright yellow + bold for status (row 0); bright cyan + bold for
   * action hints (row rows-1).  Default (-1) bg so the chrome floats
   * over any terminal palette without a coloured band. */
  init_pair(CP_HUD, g_has_256 ? 226 : COLOR_YELLOW, -1);
  init_pair(CP_HINT, g_has_256 ? 51 : COLOR_CYAN, -1);
}

/* ===================================================================== */
/* §4  state — Node, Con, Blob, Scene + cube/sphere builders             */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Node — one point-mass of a soft body, stored in Verlet two-position
 * form (Jakobsen [3]).
 *
 * Why two POSITIONS instead of (position, velocity):
 *   Velocity is IMPLICIT in the difference between consecutive positions:
 *
 *       v_n ≈ (x_n − x_{n-1}) / dt
 *
 *   Verlet integration uses this implicit velocity to advance:
 *
 *       x_{n+1} = x_n + (x_n − x_{n-1}) · damping + a · dt²
 *
 *   Storing (x, x_prev) instead of (x, v) gives us:
 *     - Time-reversible integration (swap x ↔ x_prev → run backwards).
 *     - Constraint-friendly: a position constraint moves x DIRECTLY.
 *       PBD then "absorbs" the displacement into next-step velocity
 *       automatically because v_{n+1} = (x_{n+1} − x_n) / dt picks
 *       up whatever new x_{n+1} the projection produced — no separate
 *       velocity update needed (this is PBD's central trick [1, §3]).
 *     - One float less per node vs (x, v) + separate damping factor.
 *
 * Units: pixel-space, with 1 cell-row = 2 px in y (the PHY_TO_ROW
 * conversion in §1 halves y).  Time step is implicit in the constants
 * GRAVITY_DEF, DAMPING_DEF — these are PER SUB-STEP values.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* x, y — current position in pixel space.  Mutated by:
   *   - Verlet predict (blob_step §5): adds damped velocity + gravity.
   *   - Constraint projection (PBD inner loop, §5): pulled toward
   *     neighbours by distance constraints.
   *   - World clamps (§5): pinned inside [0, ww] × [HUD_TOP_PX, wh].
   *   - Pair collisions (§6): pushed by `nearest_polygon_edge` outward / inward
   *     during the symmetric resolve_penetration passes. */
  float x, y;

  /* px, py — position from the PREVIOUS sub-step.  Saved at the start
   * of Verlet predict (before x/y change) so velocity = x − px is
   * recoverable.  Also mutated by the boundary-velocity-correction
   * step in blob_step (§5) — that step reflects velocity off floor/
   * walls by adjusting px/py rather than x/y, which preserves the
   * snap-to-floor x while flipping the implicit velocity. */
  float px, py;
} Node;

/* ─────────────────────────────────────────────────────────────────────── *
 * Con — one distance constraint between two nodes of a blob.
 *
 * Algorithm: Position-Based Dynamics distance constraint
 * (Müller et al [1, §3]).  Each PBD iteration projects every Con once:
 *
 *     d  = |x_b − x_a|                  (current distance)
 *     n  = (x_b − x_a) / d              (unit vector a → b)
 *     err = d − rest                    (signed error)
 *     correction = err · k · ½          (half-half split)
 *     x_a += n · correction
 *     x_b -= n · correction
 *
 * The ½ split assumes equal masses; constraints with attached infinite-
 * mass pinned nodes would weight by 1 / (im_a + im_b) instead.
 * Iterating over all constraints PBD_ITERS times per step converges
 * to an approximate satisfaction — higher iter count = stiffer body.
 *
 * Two constraint families share this struct:
 *   STRUCTURAL  k = STRUCT_K = 1.0 — keep adjacent nodes at rest length.
 *                  Forms the cube's grid edges and the sphere's hoop
 *                  + spoke wires.  Resists STRETCH / COMPRESSION.
 *   SHEAR       k = SHEAR_K  = 0.8 — diagonal links.  Forms the cube's
 *                  X-pattern within each face and the sphere's
 *                  diameters.  Resists SHEAR (parallelogram squish).
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* a, b — indices into Blob.nodes[].  By convention a < b (added in
   * that order by blob_add_con) but the projection math is symmetric
   * so the ordering doesn't matter functionally. */
  int a, b;

  /* rest — rest length, computed ONCE at construction as the initial
   * |x_b − x_a|.  Stays constant for the lifetime of the constraint;
   * mutating it would mean the constraint slowly forgets where the
   * body was supposed to be (no support for "plastic" deformation
   * in this demo). */
  float rest;

  /* k — stiffness factor ∈ [0, 1].  1.0 = full correction every PBD
   * iter (rigid limit, modulo iter-count truncation); 0.0 = ignore
   * this constraint entirely.  STRUCT_K=1.0 gives a chunky rubber
   * feel; SHEAR_K=0.8 makes the diagonals slightly more compliant
   * than the edges, which is the visual signature of "jelly". */
  float k;
} Con;

/* BKind — body shape discriminator.  Only read by §7 (color slot
 * selection: cube family vs sphere family) and §8 (no path — kind is
 * already encoded in the structural-constraint layout).  Adding a
 * new shape means extending this enum + a builder + a color slot. */
typedef enum { BKIND_CUBE, BKIND_SPHERE } BKind;

/* ─────────────────────────────────────────────────────────────────────── *
 * Blob — one soft body: nodes + constraints + boundary contract.
 *
 * Storage strategy: all arrays are fixed-size with capacity caps
 * (BLOB_NODES_MAX = 50, BLOB_CONS_MAX = 250, BLOB_BND_MAX = 50).  The
 * blob fits in BSS, no malloc.  blob_add_con returns silently on
 * overflow — the builders below stay well under the caps:
 *   cube 6×6: 36 nodes, ~120 constraints, 20 boundary
 *   sphere:   13 nodes,  30 constraints, 12 boundary
 *
 * Boundary contract (load-bearing for §6 collision):
 *   bnd[0..n_bnd) holds the indices of the OUTER POLYGON edge,
 *   ordered COUNTER-CLOCKWISE.  Used by:
 *     - point_in_polygon_jordan (§6) for the Jordan-curve ray-cast test [5]:
 *       crossings parity depends on winding, but only WHETHER it's
 *       monotonic — CCW vs CW would both work, we just need to pick
 *       one and stick to it.
 *     - nearest_polygon_edge (§6) walks bnd[i] → bnd[(i+1) % n_bnd] as edges.
 *     - draw_blob (§8) walks the same loop to scan-fill the interior.
 *   Builders that produce a non-convex or self-intersecting boundary
 *   would silently break all three of these.
 *
 * Algorithm refs:
 *   PBD constraint solver — Müller et al [1, §3]
 *   Verlet position storage — Jakobsen [3]
 *   Jordan-curve point-in-polygon — Sunday [5]
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* nodes[0..n_nodes) — the mass points.  n_nodes is fixed at build
   * time (CUBE_NODES = 36, SPH_NODES = 13) and never grows or shrinks
   * for the body's lifetime. */
  Node nodes[BLOB_NODES_MAX];
  int n_nodes;

  /* cons[0..n_cons) — the distance constraints.  Order of addition
   * determines order of projection in the PBD inner loop, which DOES
   * affect convergence (Gauss-Seidel-style relaxation).  Cube builder
   * adds structural first then shear; sphere adds hoop, spokes, then
   * diameters.  Reordering would subtly change soft-body feel. */
  Con cons[BLOB_CONS_MAX];
  int n_cons;

  /* bnd[0..n_bnd) — CCW outer-polygon node indices (see boundary
   * contract above).  Read-only after build; mutating bnd would
   * silently corrupt all of point_in_polygon_jordan / nearest_polygon_edge /
   * draw_blob. */
  int bnd[BLOB_BND_MAX];
  int n_bnd;

  /* surf_cp / fill_cp — ncurses colour-pair IDs for the boundary
   * wireframe + nodes (surf) and the scan-filled interior (fill).
   * Assigned at spawn by blob_color (§7).  Survives theme cycles
   * because theme_apply rewrites the SAME pair indices with new RGB —
   * the body doesn't need to be re-tagged. */
  int surf_cp, fill_cp;

  /* kind — BKIND_CUBE or BKIND_SPHERE.  Used by §7 to pick a colour
   * family (cool for cube, warm for sphere).  Physics / collision /
   * draw all work uniformly across kinds. */
  BKind kind;
} Blob;

/* ─────────────────────────────────────────────────────────────────────── *
 * Scene — top-level demo state spanning physics ticks, draw calls, and
 * the main loop.  Single instance `g_scene` lives in BSS so the whole
 * simulation is one fixed-size allocation; passing it by pointer
 * everywhere would just clutter helper signatures without buying any
 * encapsulation.
 *
 * The struct splits into two clearly-labelled groups:
 *
 *   Simulation parameters — consumed by blob_step (§5), blob_collide
 *     (§6), scene_step / scene_add_* / scene_init (§7).  Anything that
 *     changes WHAT THE BLOBS DO belongs here.  Mutated by physics-
 *     affecting keys: c / s / x (spawn / remove), r (reset),
 *     p / SPACE (pause), g (gravity), i / I (iter count).
 *
 *   Rendering parameters — consumed by theme_apply (§3) and the
 *     draw_hud_*() helpers (§8).  Mutating these MUST leave every
 *     Blob byte-identical; only colours / chrome change.  Mutated by
 *     purely cosmetic keys: t / T (theme).
 *
 * Locality rationale (this contract matters, not the bytes):
 *   The split exists for the READER, not the CPU.  A new flag landing
 *   in the rendering group when it actually changes how blob_step
 *   advances state would silently couple display to physics — exactly
 *   the bug the separation prevents.  When adding a field, ask: does
 *   this change what blob_step / blob_collide produce?  If yes, it's
 *   a simulation param; if no, rendering.
 *
 * What stays OUTSIDE Scene (intentionally):
 *
 *   g_rows, g_cols       screen geometry tracked by the main loop's
 *                        SIGWINCH handler.  Scene stays geometry-
 *                        agnostic so resize is the main loop's concern.
 *
 *   g_quit, g_resize     volatile sig_atomic_t flags written by the
 *                        signal handler; must stay file-scope for
 *                        async-signal-safety (the standard guarantees
 *                        atomic load/store only for objects of that
 *                        type at file scope).
 *
 *   g_has_256            boot-time terminal capability probed once
 *                        in screen_init; not state that evolves.
 *
 *   g_mincol[], g_maxcol[]  per-row scan-fill scratch — overwritten
 *                        every draw_blob call, not persistent state.
 *                        Lives at file scope so the renderer doesn't
 *                        need a "Renderer" workspace struct just for
 *                        two scratch arrays.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── Simulation parameters ────────────────────────────────────────
   * The body pool, the integrator knobs, and the deterministic RNG
   * that drives spawn placement.  Everything the physics step reads.
   */

  /* blobs[0..nblobs) — packed array of active soft bodies.  Insertion
   * appends (scene_add_cube / scene_add_sphere); deletion by
   * scene_remove_last just decrements nblobs (LIFO, no compaction).
   * Capacity MAX_BLOBS; once full, spawn helpers reject new bodies.
   * The all-pairs collision loop in scene_step is O(nblobs²) so this
   * cap is set with that quadratic in mind. */
  Blob blobs[MAX_BLOBS];
  int nblobs;

  /* ncubes / nsphs — cumulative spawn counts, used solely to pick a
   * colour-pair slot via blob_color() in §7.  Decoupled from nblobs
   * so deleting and respawning still cycles through the palette
   * predictably.  Reset by scene_init. */
  int ncubes, nsphs;

  /* paused — when true, scene_step is skipped entirely each frame.
   * Bodies stay frozen; unpause continues from the exact same state. */
  bool paused;

  /* steps — physics sub-steps per render frame.  At STEPS_DEF=3 the
   * solver runs 3× per scene_draw, giving smoother motion than a
   * single larger step would (PBD's stiffness scales with iteration
   * count [1], so sub-stepping multiplies effective resolution). */
  int steps;

  /* tick — monotonically increasing physics-step counter, reset by
   * scene_init.  Surface for "every N ticks" effects (none used
   * currently); displayed nowhere by default. */
  long tick;

  /* rng — xorshift-LCG state for spawn-position jitter (rng_f in §1
   * advances it).  Re-seeded only at boot; subsequent runs produce
   * identical spawn patterns for given keypress sequence — useful
   * for reproducible bug demos. */
  uint32_t rng;

  /* gravity_on — global gravity gate.  When false, blob_step adds
   * 0.f to vy instead of `gravity`; existing motion still decays
   * via `damping`. */
  bool gravity_on;

  /* gravity — downward acceleration per sub-step (px / step²).
   * Tuned for SIM_FPS=20: GRAVITY_DEF=0.06 reaches a comfortable
   * terminal velocity given damping=0.985.  Not currently exposed
   * to UI but trivially reachable from a future +/- key. */
  float gravity;

  /* damping — velocity retained per sub-step (∈ [0, 1]).  Applied
   * before the gravity add inside blob_step: v *= damping → v += g.
   * DAMPING_DEF=0.985 ⇒ retain 73 %/s at 20 Hz — bodies coast
   * visibly but settle within a few seconds. */
  float damping;

  /* pbd_iters — number of constraint-projection passes per physics
   * step (Müller et al [1, §3] solver-iter count).  Higher = stiffer
   * (closer to rigid); lower = softer / jellier.  i / I keys nudge
   * this in [PBD_ITERS_MIN, PBD_ITERS_MAX]. */
  int pbd_iters;

  /* ── Rendering parameters ─────────────────────────────────────────
   * Pure cosmetic state.  Mutating these must be a no-op for the
   * physics step.
   */

  /* theme — index into k_themes[] (§3); cycled by t / T.  Pure
   * visual change — body positions, constraints, RNG, etc. are
   * untouched. */
  int theme;

  /* fps_disp — rolling 1-second frame-rate readout displayed on the
   * top HUD.  Updated by main()'s frame counter once per second;
   * the renderer just reads it. */
  int fps_disp;
} Scene;

/* g_scene — the one Scene instance.  Non-zero defaults:
 *   gravity_on = true       bodies fall by default
 *   gravity    = 0.06       default downward accel
 *   damping    = 0.985      default velocity retention
 *   pbd_iters  = 6          default constraint solver passes
 *   steps      = 3          physics sub-steps per frame
 *   rng        = 0xdeadbeef seed for deterministic boot behaviour
 * Everything else (blobs, nblobs, ncubes, nsphs, paused, tick,
 * theme, fps_disp) is BSS-zeroed; scene_init() populates blobs[]
 * before the first physics step. */
static Scene g_scene = {
    .gravity_on = true,
    .gravity = GRAVITY_DEF,
    .damping = DAMPING_DEF,
    .pbd_iters = PBD_ITERS_DEF,
    .steps = STEPS_DEF,
    .rng = 0xdeadbeef,
};

/*
 * rng_f — xorshift-LCG → uniform float in [0, 1).  Stateful via
 * g_scene.rng.  Used by scene_add_* (§7) to place new bodies at a
 * deterministic-yet-jittered x.  Knuth-style multiplier (1664525,
 * 1013904223); good enough for spawn jitter, not cryptographic.
 */
static float rng_f(void) {
  g_scene.rng = g_scene.rng * 1664525u + 1013904223u;
  return (g_scene.rng >> 8) * (1.f / 16777216.f);
}

static void blob_add_con(Blob *bl, int a, int b, float k) {
  if (bl->n_cons >= BLOB_CONS_MAX)
    return;
  float dx = bl->nodes[b].x - bl->nodes[a].x;
  float dy = bl->nodes[b].y - bl->nodes[a].y;
  bl->cons[bl->n_cons++] = (Con){a, b, sqrtf(dx * dx + dy * dy), k};
}

/* ─────────────────────────────────────────────────────────────────────── *
 * Blob builders (cube + sphere)
 *
 * Each builder is a 4-step pipeline:
 *
 *     install_*_grid_nodes        — lay out the mass points
 *     install_*_structural_cons   — resist stretch / compression
 *     install_*_shear_cons        — resist parallelogram shear
 *     install_*_boundary_ccw      — write the CCW bnd[] index ring
 *
 * Order is load-bearing in two ways:
 *   1. Constraint order (structural before shear) sets the Gauss-Seidel
 *      iteration order in project_all_distance_constraints (§5), which
 *      DOES affect convergence rate at low PBD_ITERS.
 *   2. Boundary ring must be CCW for the Jordan-curve test [5] in §6 —
 *      builders walk the outer edge in the same direction.
 * ─────────────────────────────────────────────────────────────────────── */

/* ── cube primitives ───────────────────────────────────────────────── */

/* install_cube_grid_nodes — CUBE_W × CUBE_H grid of nodes anchored at
 * (ox, oy), spaced by CUBE_SP px.  px/py initialised equal to x/y so
 * the initial Verlet velocity is zero. */
static void install_cube_grid_nodes(Blob *bl, float ox, float oy) {
  for (int r = 0; r < CUBE_H; r++)
    for (int c = 0; c < CUBE_W; c++) {
      int i = r * CUBE_W + c;
      bl->nodes[i].x = bl->nodes[i].px = ox + c * CUBE_SP;
      bl->nodes[i].y = bl->nodes[i].py = oy + r * CUBE_SP;
    }
}

/* install_cube_structural_cons — horizontal + vertical neighbour links.
 * These resist STRETCH along the grid axes.  Forms the outline of every
 * cell in the cube grid (cube has 2·CUBE_W·CUBE_H − CUBE_W − CUBE_H
 * structural cons; for 6×6 grid that's 60). */
static void install_cube_structural_cons(Blob *bl) {
  for (int r = 0; r < CUBE_H; r++)
    for (int c = 0; c < CUBE_W; c++) {
      int i = r * CUBE_W + c;
      if (c + 1 < CUBE_W)
        blob_add_con(bl, i, i + 1, STRUCT_K);
      if (r + 1 < CUBE_H)
        blob_add_con(bl, i, i + CUBE_W, STRUCT_K);
    }
}

/* install_cube_shear_cons — both diagonals of every (r, c)→(r+1, c+1)
 * cell.  These resist SHEAR (parallelogram squish).  Without them the
 * cube would collapse into a rhombus under gravity since structural
 * cons only constrain edge lengths, not internal angles.  2·(CUBE_W−1)·
 * (CUBE_H−1) shear cons = 50 for a 6×6 grid. */
static void install_cube_shear_cons(Blob *bl) {
  for (int r = 0; r < CUBE_H - 1; r++)
    for (int c = 0; c < CUBE_W - 1; c++) {
      int i = r * CUBE_W + c;
      blob_add_con(bl, i, i + CUBE_W + 1, SHEAR_K); /* \ diag */
      blob_add_con(bl, i + 1, i + CUBE_W, SHEAR_K); /* / diag */
    }
}

/* install_cube_boundary_ccw — walk the outer edge of the grid in
 * counter-clockwise order: top → right → bottom (reversed) → left
 * (reversed).  Result is the closed polygon §6 collision needs. */
static void install_cube_boundary_ccw(Blob *bl) {
  int *bnd = bl->bnd;
  bl->n_bnd = 0;
  /* top edge (left → right) */
  for (int c = 0; c < CUBE_W; c++)
    bnd[bl->n_bnd++] = c;
  /* right edge (top → bottom, skipping corner already added) */
  for (int r = 1; r < CUBE_H; r++)
    bnd[bl->n_bnd++] = r * CUBE_W + (CUBE_W - 1);
  /* bottom edge (right → left, skipping corner) */
  for (int c = CUBE_W - 2; c >= 0; c--)
    bnd[bl->n_bnd++] = (CUBE_H - 1) * CUBE_W + c;
  /* left edge (bottom → top, skipping both corners) */
  for (int r = CUBE_H - 2; r > 0; r--)
    bnd[bl->n_bnd++] = r * CUBE_W;
}

/* blob_build_cube — orchestrator.  Reads as pseudocode of the four
 * install_*() helpers above. */
static void blob_build_cube(Blob *bl, float ox, float oy, int scp, int fcp) {
  memset(bl, 0, sizeof *bl);
  bl->surf_cp = scp;
  bl->fill_cp = fcp;
  bl->kind = BKIND_CUBE;
  bl->n_nodes = CUBE_NODES;

  install_cube_grid_nodes(bl, ox, oy);
  install_cube_structural_cons(bl);
  install_cube_shear_cons(bl);
  install_cube_boundary_ccw(bl);
}

/* ── sphere primitives ─────────────────────────────────────────────── */

/* install_sphere_radial_nodes — node 0 at centre, nodes 1..SPH_RING on
 * an ellipse around it.  y-radius = 2·SPH_R so the body renders as a
 * VISUAL CIRCLE on screen (terminal cells are 1:2 aspect — see SPH_R
 * docstring in §1). */
static void install_sphere_radial_nodes(Blob *bl, float cx, float cy) {
  /* Centre. */
  bl->nodes[0].x = bl->nodes[0].px = cx;
  bl->nodes[0].y = bl->nodes[0].py = cy;

  /* Ring. */
  for (int i = 0; i < SPH_RING; i++) {
    float a = 6.2831853f * i / SPH_RING;
    int ni = i + 1;
    bl->nodes[ni].x = bl->nodes[ni].px = cx + SPH_R * cosf(a);
    bl->nodes[ni].y = bl->nodes[ni].py = cy + SPH_R * 2.f * sinf(a);
  }
}

/* install_sphere_hoop_cons — adjacent ring-to-ring distance constraints
 * forming the closed outer hoop.  These keep neighbouring boundary
 * nodes the same distance apart — analogous to the cube's structural
 * edge constraints. */
static void install_sphere_hoop_cons(Blob *bl) {
  for (int i = 0; i < SPH_RING; i++)
    blob_add_con(bl, 1 + i, 1 + (i + 1) % SPH_RING, STRUCT_K);
}

/* install_sphere_spoke_cons — centre-to-ring distance constraints.
 * Each ring node is tied to the centre, so the body resists radial
 * compression / expansion.  Analogous to the cube's structural grid:
 * structural cons in BOTH families resist stretch. */
static void install_sphere_spoke_cons(Blob *bl) {
  for (int i = 0; i < SPH_RING; i++)
    blob_add_con(bl, 0, 1 + i, STRUCT_K);
}

/* install_sphere_diameter_cons — ring node i ↔ ring node i+SPH_RING/2
 * (opposite side of the hoop).  Resist sphere "pinching" into an oval —
 * the soft-body analogue of the cube's shear diagonals.  SPH_RING/2
 * diameters total (each diameter connects two nodes; we don't want to
 * double-count by iterating the full ring). */
static void install_sphere_diameter_cons(Blob *bl) {
  for (int i = 0; i < SPH_RING / 2; i++)
    blob_add_con(bl, 1 + i, 1 + i + SPH_RING / 2, SHEAR_K);
}

/* install_sphere_boundary_ccw — the ring itself, walked in order.
 * Index 1..SPH_RING (the centre, node 0, is NOT on the boundary).
 * cosf / sinf with positive sine = counter-clockwise in math-y, which
 * matches our screen-y flip since PHY_TO_ROW grows downward. */
static void install_sphere_boundary_ccw(Blob *bl) {
  for (int i = 0; i < SPH_RING; i++)
    bl->bnd[i] = 1 + i;
  bl->n_bnd = SPH_RING;
}

/* blob_build_sphere — orchestrator. */
static void blob_build_sphere(Blob *bl, float cx, float cy, int scp, int fcp) {
  memset(bl, 0, sizeof *bl);
  bl->surf_cp = scp;
  bl->fill_cp = fcp;
  bl->kind = BKIND_SPHERE;
  bl->n_nodes = SPH_NODES;

  install_sphere_radial_nodes(bl, cx, cy);
  install_sphere_hoop_cons(bl);
  install_sphere_spoke_cons(bl);
  install_sphere_diameter_cons(bl);
  install_sphere_boundary_ccw(bl);
}

/* ===================================================================== */
/* §5  physics                                                            */
/* ===================================================================== */

/* Screen geometry tracked by the main loop's SIGWINCH handler;
 * Scene stays geometry-agnostic.  All physics knobs (gravity_on,
 * gravity, damping, pbd_iters) live on g_scene (§4). */
static int g_rows, g_cols;

/* ─────────────────────────────────────────────────────────────────────── *
 * blob_step pipeline — one PBD time-step over one blob.
 *
 * Reads as the textbook PBD algorithm (Müller et al [1, §3]):
 *
 *     verlet_predict_all(bl, ww, wh)           // advance free flight
 *     for k in 0 .. pbd_iters:                 // constraint relaxation
 *         project_all_distance_constraints(bl)
 *         clamp_all_nodes_to_world(bl, ww, wh) // keep inside arena
 *     apply_boundary_velocity_response(bl, ww, wh)  // floor friction + walls
 *
 * Stiffness emerges from pbd_iters (more passes → closer to rigid),
 * NOT from a per-spring stiffness constant.  That's PBD's key trick:
 * geometric projection bypasses the dt-stability problem of explicit
 * spring forces.
 *
 * Helpers below match the pseudocode line-for-line.
 * ─────────────────────────────────────────────────────────────────────── */

/*
 * verlet_predict_all — Jakobsen [3] Verlet predict for every node.
 * Computes v ≈ (x − px) implicitly, applies damping, advances x with
 * gravity.  px/py is updated to the PRE-advance x/y so the next step's
 * velocity sees this step's displacement.
 */
static void verlet_predict_all(Blob *bl) {
  float g = g_scene.gravity_on ? g_scene.gravity : 0.f;
  for (int i = 0; i < bl->n_nodes; i++) {
    Node *nd = &bl->nodes[i];
    float vx = (nd->x - nd->px) * g_scene.damping;
    float vy = (nd->y - nd->py) * g_scene.damping;
    nd->px = nd->x;
    nd->py = nd->y;
    nd->x += vx;
    nd->y += vy + g;
  }
}

/*
 * project_all_distance_constraints — one PBD iteration.  For each Con:
 *
 *     d   = |x_b − x_a|
 *     err = d − rest
 *     correction = (err / d) · k · ½
 *     x_a += (x_b − x_a) · correction
 *     x_b -= (x_b − x_a) · correction
 *
 * Half-half split assumes equal masses (Müller [1, eq. 7] with both
 * inverse masses = 1).  Skipping degenerate d < 1e-6 avoids the
 * division-by-zero when two nodes coincide momentarily after a hard
 * collision push.
 */
static void project_all_distance_constraints(Blob *bl) {
  for (int c = 0; c < bl->n_cons; c++) {
    Node *a = &bl->nodes[bl->cons[c].a];
    Node *b = &bl->nodes[bl->cons[c].b];
    float dx = b->x - a->x, dy = b->y - a->y;
    float d = sqrtf(dx * dx + dy * dy);
    if (d < 1e-6f)
      continue;
    float corr = (d - bl->cons[c].rest) / d * bl->cons[c].k * 0.5f;
    a->x += dx * corr;
    a->y += dy * corr;
    b->x -= dx * corr;
    b->y -= dy * corr;
  }
}

/*
 * clamp_all_nodes_to_world — keep every node inside the rectangular
 * arena defined by [0, ww] × [HUD_TOP_PX, wh].  Position-only clamp
 * (no velocity tweak); the velocity-response helper below handles
 * the bounce / friction after the PBD iterations converge.
 *
 * Ceiling sits at HUD_TOP_PX, not 0, so bodies never draw into the
 * top HUD bar (row 0).
 */
static void clamp_all_nodes_to_world(Blob *bl, float ww, float wh) {
  for (int i = 0; i < bl->n_nodes; i++) {
    Node *nd = &bl->nodes[i];
    if (nd->x < 0.f)
      nd->x = 0.f;
    if (nd->x > ww)
      nd->x = ww;
    if (nd->y < HUD_TOP_PX)
      nd->y = HUD_TOP_PX;
    if (nd->y > wh)
      nd->y = wh;
  }
}

/*
 * apply_boundary_velocity_response — reflection + friction at walls.
 *
 * For each node touching a boundary, we adjust px / py (NOT x / y) so
 * the IMPLICIT Verlet velocity v = x − px flips with restitution.  At
 * the floor we also apply Coulomb-ish friction by scaling px → modify
 * the horizontal velocity component.  Walls are reflection-only (no
 * friction) so a body sliding along a wall doesn't get artificially
 * slowed.
 *
 * Reflection formula (floor example):
 *     vy_old = y - py                  (current implicit velocity)
 *     new py = y + vy * FLOOR_REST     (post-bounce velocity vy' = −vy·rest)
 *
 * The sign flip is implicit: setting py = y + vy makes (y - py) = −vy,
 * scaled by FLOOR_REST < 1 for an inelastic bounce.
 */
static void apply_boundary_velocity_response(Blob *bl, float ww, float wh) {
  for (int i = 0; i < bl->n_nodes; i++) {
    Node *nd = &bl->nodes[i];
    float vx = nd->x - nd->px;
    float vy = nd->y - nd->py;

    /* Floor: reflect vy + apply horizontal friction. */
    if (nd->y >= wh - 0.01f && vy > 0.f) {
      nd->py = nd->y + vy * FLOOR_REST;
      nd->px = nd->x - vx * FLOOR_FRIC;
    }
    /* Ceiling (top HUD line): reflect vy, no friction. */
    if (nd->y <= HUD_TOP_PX + 0.01f && vy < 0.f)
      nd->py = nd->y + vy * WALL_REST;
    /* Left wall. */
    if (nd->x <= 0.01f && vx < 0.f)
      nd->px = nd->x + vx * WALL_REST;
    /* Right wall. */
    if (nd->x >= ww - 0.01f && vx > 0.f)
      nd->px = nd->x - vx * WALL_REST;
  }
}

/*
 * blob_step — one PBD time-step orchestrator.  Reads as the textbook
 * pseudocode pinned at the top of §5.
 */
static void blob_step(Blob *bl) {
  float ww = (float)g_cols - 1.f;
  float wh = (float)(g_rows - 2) * 2.f;

  verlet_predict_all(bl);

  for (int iter = 0; iter < g_scene.pbd_iters; iter++) {
    project_all_distance_constraints(bl);
    clamp_all_nodes_to_world(bl, ww, wh);
  }

  apply_boundary_velocity_response(bl, ww, wh);
}

/* ===================================================================== */
/* §6  collision  (works for any pair: cube-cube, sphere-sphere, mixed)   */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Pair-collision pipeline.
 *
 * For each pair (A, B) of blobs we run COLL_ITERS symmetric passes:
 *
 *     for k in 0 .. COLL_ITERS:
 *         resolve_penetration(A, B)   // push A's nodes out of B
 *         resolve_penetration(B, A)   // push B's nodes out of A
 *
 * resolve_penetration walks A's nodes and for each one that's inside B's
 * polygon, projects it back out and Newton's-3rd-law-pushes the two
 * nearest B-boundary-nodes inward.  The two-way pass plus the symmetric
 * 2× ITER loop is what makes the contact look "mutual" rather than
 * "A always wins".  Algorithm follows Müller et al [1, §5] (PBD
 * collision constraint) layered over Ericson's [4] polygon primitives.
 * ─────────────────────────────────────────────────────────────────────── */

/*
 * point_in_polygon_jordan — even-odd ray-cast point-in-polygon test
 * (Jordan curve theorem; Sunday [5]).  Casts a horizontal ray from
 * (px, py) to the right and counts boundary crossings; odd = inside.
 *
 * Edge case handling:
 *   The (ay <= py && by > py) || (by <= py && ay > py) condition
 *   ensures exactly one endpoint per horizontal-crossing edge counts,
 *   even when the ray passes through a vertex — without this,
 *   degenerate "ray-through-vertex" cases double-count and produce
 *   the wrong parity.
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

/*
 * nearest_polygon_edge — find the boundary edge of blob `bl` closest
 * to the world-space point (px, py).  Returns:
 *   nx, ny    unit vector from the closest edge point TOWARD (px, py)
 *             (i.e. the direction we'd push (px, py) further OUT)
 *   depth     distance from edge to point (always ≥ 0)
 *   ea, eb    node indices of the closest edge's endpoints
 *
 * Algorithm: parametric "closest point on segment" (Ericson [4, §5.1.5]):
 *
 *     edge = b - a
 *     t = ((p - a) · edge) / |edge|²
 *     t = clamp(t, 0, 1)
 *     closest = a + t · edge
 *     d = |p - closest|
 *
 * Walks every boundary edge and keeps the minimum-distance one.  O(n_bnd)
 * per call — acceptable for our small bodies (≤ 20 boundary nodes).
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

    float t = ((px - ax) * edx + (py - ay) * edy) / len2;
    if (t < 0.f)
      t = 0.f;
    if (t > 1.f)
      t = 1.f;

    float cx = ax + t * edx, cy = ay + t * edy;
    float dx = px - cx, dy = py - cy;
    float d = sqrtf(dx * dx + dy * dy);
    if (d < best) {
      best = d;
      *depth = d;
      *nx = d > 1e-6f ? dx / d : 0.f;
      *ny = d > 1e-6f ? dy / d : 1.f;
      *ea = bl->bnd[i];
      *eb = bl->bnd[j];
    }
  }
}

/*
 * push_node_outward — move A's penetrating node `i` along the outward
 * normal by half the penetration depth (plus a small skin offset).
 * The 0.5 fraction is one half of the symmetric depth split with
 * push_edge_inward — Newton's 3rd law in position space (Müller [1,
 * §5], Goldstein [6, §1.1] for the classical-mechanics backing).
 */
static void push_node_outward(Blob *a, int i, float nx, float ny, float depth) {
  float push = (depth + 0.5f) * 0.5f;
  a->nodes[i].x -= nx * push;
  a->nodes[i].y -= ny * push;
}

/*
 * push_edge_inward — move B's two nearest boundary nodes (ea, eb)
 * inward by a quarter of the penetration depth each.  Total inward
 * displacement on B is depth/4 + depth/4 = depth/2, matching the
 * depth/2 outward displacement on A — symmetric, no momentum injection.
 * "Inward" here is OPPOSITE to the (nx, ny) outward normal.
 */
static void push_edge_inward(Blob *b, int ea, int eb, float nx, float ny,
                             float depth) {
  float push = depth * 0.25f;
  b->nodes[ea].x += nx * push;
  b->nodes[ea].y += ny * push;
  b->nodes[eb].x += nx * push;
  b->nodes[eb].y += ny * push;
}

/*
 * resolve_penetration — push every node of A that lies inside B back
 * out, and Newton-3rd-law push B's nearest edge nodes inward.  One
 * directional pass; blob_collide alternates A→B and B→A to make the
 * contact symmetric.
 *
 * Reads as pseudocode:
 *     for each node i of A:
 *         if point_in_polygon_jordan(B, A.nodes[i]):
 *             (n, depth, ea, eb) = nearest_polygon_edge(B, ...)
 *             push_node_outward(A, i, n, depth)
 *             push_edge_inward (B, ea, eb, n, depth)
 */
static void resolve_penetration(Blob *a, Blob *b) {
  for (int i = 0; i < a->n_nodes; i++) {
    float px = a->nodes[i].x;
    float py = a->nodes[i].y;
    if (!point_in_polygon_jordan(b, px, py))
      continue;

    float nx, ny, depth;
    int ea, eb;
    nearest_polygon_edge(b, px, py, &nx, &ny, &depth, &ea, &eb);

    push_node_outward(a, i, nx, ny, depth);
    push_edge_inward(b, ea, eb, nx, ny, depth);
  }
}

/*
 * blob_collide — pair-collision orchestrator.  COLL_ITERS symmetric
 * passes per body pair so neither body dominates the contact.
 */
static void blob_collide(Blob *a, Blob *b) {
  for (int iter = 0; iter < COLL_ITERS; iter++) {
    resolve_penetration(a, b);
    resolve_penetration(b, a);
  }
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/* Body pool, spawn counters, pause/tick — all live on g_scene (§4). */

/* Assign a color slot: cubes cycle through 0-2, spheres through 3-5 */
static void blob_color(BKind kind, int count, int *scp, int *fcp) {
  int slot = (kind == BKIND_CUBE) ? (count % 3) : (3 + count % 3);
  *scp = CP_BSURF(slot);
  *fcp = CP_BFILL(slot);
}

static bool scene_add_cube(void) {
  if (g_scene.nblobs >= MAX_BLOBS)
    return false;
  float ww = (float)g_cols;
  float wh = (float)(g_rows - 2) * 2.f;
  float half_w = (CUBE_W - 1) * CUBE_SP * 0.5f;
  float ox = ww * (0.15f + rng_f() * 0.70f) - half_w;
  /* clamp so cube stays inside screen */
  if (ox < 1.f)
    ox = 1.f;
  if (ox + (CUBE_W - 1) * CUBE_SP > ww - 2.f)
    ox = ww - 2.f - (CUBE_W - 1) * CUBE_SP;
  float oy = HUD_TOP_PX + 0.5f; /* just below the top HUD bar */
  (void)wh;
  int scp, fcp;
  blob_color(BKIND_CUBE, g_scene.ncubes, &scp, &fcp);
  blob_build_cube(&g_scene.blobs[g_scene.nblobs], ox, oy, scp, fcp);
  g_scene.nblobs++;
  g_scene.ncubes++;
  return true;
}

static bool scene_add_sphere(void) {
  if (g_scene.nblobs >= MAX_BLOBS)
    return false;
  float ww = (float)g_cols;
  float cx = ww * (0.15f + rng_f() * 0.70f);
  if (cx < SPH_R + 1.f)
    cx = SPH_R + 1.f;
  if (cx > ww - SPH_R - 1.f)
    cx = ww - SPH_R - 1.f;
  float cy = SPH_R * 2.f + HUD_TOP_PX; /* just below top HUD bar */
  int scp, fcp;
  blob_color(BKIND_SPHERE, g_scene.nsphs, &scp, &fcp);
  blob_build_sphere(&g_scene.blobs[g_scene.nblobs], cx, cy, scp, fcp);
  g_scene.nblobs++;
  g_scene.nsphs++;
  return true;
}

static void scene_remove_last(void) {
  if (g_scene.nblobs > 0)
    g_scene.nblobs--;
}

static void scene_init(void) {
  g_scene.tick = 0;
  g_scene.nblobs = 0;
  g_scene.ncubes = 0;
  g_scene.nsphs = 0;
  float ww = (float)g_cols;
  float wh = (float)(g_rows - 2) * 2.f;
  float cx = ww * 0.5f;

  /* Cube sitting on floor, centred */
  float cube_w = (CUBE_W - 1) * CUBE_SP;
  float cube_h = (CUBE_H - 1) * CUBE_SP;
  int scp, fcp;
  blob_color(BKIND_CUBE, g_scene.ncubes, &scp, &fcp);
  blob_build_cube(&g_scene.blobs[g_scene.nblobs], cx - cube_w * 0.5f,
                  wh - cube_h - 0.5f, scp, fcp);
  g_scene.nblobs++;
  g_scene.ncubes++;

  /* Sphere falling from top-centre, just below the top HUD bar. */
  blob_color(BKIND_SPHERE, g_scene.nsphs, &scp, &fcp);
  blob_build_sphere(&g_scene.blobs[g_scene.nblobs], cx,
                    SPH_R * 2.f + HUD_TOP_PX, scp, fcp);
  g_scene.nblobs++;
  g_scene.nsphs++;
}

static void scene_step(void) {
  for (int s = 0; s < g_scene.steps; s++) {
    /* Integrate each blob (includes floor/wall constraints) */
    for (int i = 0; i < g_scene.nblobs; i++)
      blob_step(&g_scene.blobs[i]);
    /* All pairwise collisions */
    for (int i = 0; i < g_scene.nblobs; i++)
      for (int j = i + 1; j < g_scene.nblobs; j++)
        blob_collide(&g_scene.blobs[i], &g_scene.blobs[j]);
  }
  g_scene.tick++;
}

/* ===================================================================== */
/* §8  draw                                                               */
/* ===================================================================== */

static int g_mincol[ROWS_MAX], g_maxcol[ROWS_MAX];

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

static void draw_blob(const Blob *bl, int floor_row, int cols) {
  /* Scan-fill interior */
  for (int r = 0; r < floor_row; r++) {
    g_mincol[r] = cols;
    g_maxcol[r] = -1;
  }
  for (int i = 0; i < bl->n_bnd; i++) {
    int j = (i + 1) % bl->n_bnd;
    const Node *a = &bl->nodes[bl->bnd[i]], *b = &bl->nodes[bl->bnd[j]];
    scanfill_edge(PHY_TO_COL(a->x), PHY_TO_ROW(a->y), PHY_TO_COL(b->x),
                  PHY_TO_ROW(b->y), floor_row, cols);
  }
  attron(COLOR_PAIR(bl->fill_cp));
  for (int r = 0; r < floor_row; r++) {
    if (g_maxcol[r] <= g_mincol[r])
      continue;
    for (int c = g_mincol[r] + 1; c < g_maxcol[r]; c++)
      mvaddch(r, c, ':');
  }
  attroff(COLOR_PAIR(bl->fill_cp));

  /* Constraint wireframe */
  attron(COLOR_PAIR(bl->surf_cp));
  for (int ci = 0; ci < bl->n_cons; ci++) {
    const Node *a = &bl->nodes[bl->cons[ci].a];
    const Node *b = &bl->nodes[bl->cons[ci].b];
    int x0 = PHY_TO_COL(a->x), y0 = PHY_TO_ROW(a->y);
    int x1 = PHY_TO_COL(b->x), y1 = PHY_TO_ROW(b->y);
    int adx = abs(x1 - x0), ady = abs(y1 - y0);
    chtype ch;
    if (adx == 0)
      ch = '|';
    else if (ady == 0)
      ch = '-';
    else if ((x1 - x0) * (y1 - y0) > 0)
      ch = '\\';
    else
      ch = '/';
    bresenham(x0, y0, x1, y1, floor_row, cols, ch);
  }
  attroff(COLOR_PAIR(bl->surf_cp));

  /* Boundary nodes */
  attron(COLOR_PAIR(bl->surf_cp) | A_BOLD);
  for (int i = 0; i < bl->n_bnd; i++) {
    const Node *nd = &bl->nodes[bl->bnd[i]];
    int cr = PHY_TO_ROW(nd->y), cc = PHY_TO_COL(nd->x);
    if (cr >= 0 && cr < floor_row && cc >= 0 && cc < cols)
      mvaddch(cr, cc, 'O');
  }
  attroff(COLOR_PAIR(bl->surf_cp) | A_BOLD);
}

/* fps_disp lives on g_scene.fps_disp (§4). */

/*
 * Two-bar HUD per CLAUDE.md convention:
 *   row 0      right (CP_HUD,  bright yellow + bold) — live status
 *   row rows-1 left  (CP_HINT, bright cyan   + bold) — action keys
 *
 * The chrome colours are theme-INDEPENDENT (yellow / cyan, set in
 * theme_apply) so the bars stay legible against every body palette.
 */
static void draw_hud_top(int fps) {
  int ncubes = 0, nsphs = 0;
  for (int i = 0; i < g_scene.nblobs; i++) {
    if (g_scene.blobs[i].kind == BKIND_CUBE)
      ncubes++;
    else
      nsphs++;
  }

  char buf[200];
  snprintf(buf, sizeof buf,
           " cubes:%d  spheres:%d/%d  iters:%d  grav:%s  theme:%s  %s  %d fps ",
           ncubes, nsphs, MAX_BLOBS, g_scene.pbd_iters,
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
  const char *full = " q:quit  spc:pause  r:reset  c:cube  s:sphere  x:del  "
                     "i/I:iters  g:gravity  t/T:theme ";
  const char *shrt = " q:quit  spc:pause  c/s:cube/sph  t:theme ";
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

  /* Bodies — clipped by draw_blob to rows 0 .. floor_row-1. */
  for (int i = 0; i < g_scene.nblobs; i++)
    draw_blob(&g_scene.blobs[i], floor_row, cols);

  /* HUD chrome — top status (row 0) + bottom hints (row rows-1). */
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
  use_default_colors(); /* enable -1 (transparent) bg for HUD chrome */
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

  scene_init();
  int64_t next_tick = clock_ns();
  int64_t fps_window_start = next_tick;
  int fps_frames = 0;

  while (!g_quit) {
    int ch;
    while ((ch = getch()) != ERR) {
      switch (ch) {
      case 'q':
      case 27:
        g_quit = 1;
        break;
      case 'p':
      case ' ':
        g_scene.paused = !g_scene.paused;
        break;
      case 'r':
        scene_init();
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
      scene_init();
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
