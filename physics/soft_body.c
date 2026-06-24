/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * soft_body.c — wobbly jelly cubes and spheres that fall, squish, and bump
 * into each other, drawn in the terminal.
 *
 * The trick: instead of springs and forces, we just move points and then
 * gently tug them back toward where they should be. Do that a few times a
 * frame and the body holds its shape but jiggles like jelly. This is called
 * Position-Based Dynamics.
 *
 * References for the ideas the code can't show you:
 *   Müller et al. (2007), "Position Based Dynamics"        — the core method [1]
 *   Jakobsen (2001), "Advanced Character Physics"          — the move-points trick [3]
 *   Sunday (2001), "Inclusion of a Point in a Polygon"     — the inside/outside test [5]
 *   Ericson (2005), Real-Time Collision Detection §5.1.5   — closest point on an edge [4]
 *   Padala, NCURSES Programming HOWTO                       — the ncurses calls used here [8]
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1  config ── */

#define ROWS_MAX 128
#define COLS_MAX 512

/* A cube is a CUBE_W × CUBE_H grid of points spaced CUBE_SP apart. */
#define CUBE_W 6
#define CUBE_H 6
#define CUBE_SP 3.0f
#define CUBE_NODES (CUBE_W * CUBE_H) /* 36 */

/* A sphere is a centre point plus a ring of points around it. The y-size
 * is doubled because terminal cells are tall, so this draws as a round
 * circle rather than a squashed one. */
#define SPH_RING 12 /* points on the ring; node 0 is the centre */
#define SPH_NODES (SPH_RING + 1)
#define SPH_R 5.0f

/* Most points/links any one body can have, so everything fits in fixed
 * arrays with no malloc. */
#define BLOB_NODES_MAX 50
#define BLOB_CONS_MAX 250
#define BLOB_BND_MAX 50

/* How many bodies can be on screen at once. */
#define MAX_BLOBS 16

/* Each body gets one of 6 colour slots. Slots 0-2 are the cool cube
 * colours, 3-5 the warm sphere colours. These macros just turn a slot
 * number into the matching ncurses colour-pair id. */
#define N_BSLOTS 6
#define CP_BSURF(i) (1 + (i))            /* outline + nodes */
#define CP_BFILL(i) (1 + N_BSLOTS + (i)) /* interior fill   */
#define CP_FLOOR (1 + 2 * N_BSLOTS)      /* the floor line  */
#define CP_HUD (2 + 2 * N_BSLOTS)        /* top status bar  */
#define CP_HINT (3 + 2 * N_BSLOTS)       /* bottom key bar  */

/* Space we keep clear at the very top for the status bar, measured in
 * physics units (one screen row = 2 of these). Bodies are stopped from
 * floating up into it so they never overwrite row 0. */
#define HUD_TOP_PX 2.0f

/* How many times we tug points back toward their resting shape each step.
 * More passes = stiffer body; STRUCT_K is full strength, SHEAR_K a touch
 * softer so the diagonals give a little. */
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

/* The random-number state lives on the Scene struct (§4), kept with the
 * other simulation values rather than as a loose global. */

/* ── §2  clock ── */

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

/* ── §3  color / theme ── */

/*
 * Theme — one colour scheme for the whole scene.
 *
 * Switching themes is purely cosmetic: it must not nudge any body even
 * slightly, only swap the colours. Each body is drawn in two shades so it
 * looks like it has some depth — a bright outline and a darker inside — so
 * a theme carries one colour for each. There are 6 colour slots: the first
 * three are the cool cube colours, the last three the warm sphere colours.
 */
typedef struct {
  /* surf[i] — the brighter "outline" colour for slot i (the wireframe and
   * the O nodes). Slots 0..2 are cubes, 3..5 are spheres. */
  short surf[N_BSLOTS];

  /* fill[i] — the darker "inside" colour for slot i, so the outline drawn
   * on top of it still reads clearly. */
  short fill[N_BSLOTS];

  /* name — short label for the status bar. Kept to 7 chars or fewer so the
   * whole top line still fits on an 80-column terminal. */
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

/* Whether the terminal supports 256 colours. Checked once at startup. */
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
  /* The status and key bars stay the same colours in every theme (yellow
   * and cyan) so they're always easy to read. The -1 background lets them
   * sit on whatever the terminal's own background is. */
  init_pair(CP_HUD, g_has_256 ? 226 : COLOR_YELLOW, -1);
  init_pair(CP_HINT, g_has_256 ? 51 : COLOR_CYAN, -1);
}

/* ── §4  state — Node, Con, Blob, Scene + cube/sphere builders ── */

/*
 * Node — one of the points that make up a body.
 *
 * The clever part: we never store how fast a point is moving. Instead we
 * keep where it is now AND where it was last step. The difference between
 * those two is its speed and direction, for free. So to keep something
 * moving we just nudge "now" a bit further past "last time."
 *
 * This pays off for two reasons. First, when we shove a point to fix the
 * body's shape or to push it out of a collision, the speed updates by
 * itself — no separate bookkeeping. Second, it's stable: the body never
 * blows up no matter how big the time step. (Jakobsen [3]; this storage is
 * the heart of the whole method.)
 *
 * Positions are in physics units, where the y direction is twice as fine as
 * x to make up for tall terminal cells.
 */
typedef struct {
  /* x, y — where the point is right now. Moved by the falling step, by the
   * shape-fixing tugs, by being clamped inside the arena, and by being
   * pushed out of other bodies during collisions (all in §5/§6). */
  float x, y;

  /* px, py — where the point was last step. We compare it with x, y to know
   * the point's speed. Bouncing off the floor and walls works by editing
   * these (not x, y), which flips the speed while leaving the point parked
   * on the surface. */
  float px, py;
} Node;

/*
 * Con — a "link" that wants to keep two points a fixed distance apart, like
 * a tiny rubber band with a preferred length.
 *
 * Each pass, every link checks how far apart its two points actually are. If
 * they've drifted, it pulls them back toward the right distance, moving each
 * point half the gap so neither is favoured. Run all the links a few times
 * per step and the whole body settles into its shape. (Müller et al [1].)
 *
 * Two kinds of link share this struct:
 *   structural (full strength) — joins neighbouring points; keeps the body
 *                 from stretching or squashing. These are the cube's grid
 *                 edges and the sphere's ring and spokes.
 *   shear (a little softer) — the diagonals; stops the body from leaning
 *                 over into a slanted shape. These are the cube's X's and
 *                 the sphere's across-the-middle lines.
 */
typedef struct {
  /* a, b — the two points this link connects, given as positions in
   * Blob.nodes[]. */
  int a, b;

  /* rest — the distance the link wants to keep, measured once when the body
   * is built and never changed (so the body always remembers its original
   * shape; it can't be permanently dented). */
  float rest;

  /* k — how hard this link pulls, from 0 (ignored) to 1 (full strength).
   * The diagonals use a slightly lower value so they give a bit, which is
   * what makes the body feel like jelly rather than a rigid box. */
  float k;
} Con;

/* BKind — which shape a body is, cube or sphere. We only check this to
 * pick its colour (cool shades for cubes, warm for spheres); the physics
 * and collision code treats both the same. Adding a new shape means a new
 * value here, a new builder, and a new colour. */
typedef enum { BKIND_CUBE, BKIND_SPHERE } BKind;

/*
 * Blob — one soft body: its points, the links that hold its shape, and a
 * list of which points trace its outline.
 *
 * Everything is a fixed-size array (no malloc), with caps big enough for
 * the shapes we build:
 *   cube 6×6: 36 points, ~110 links, 20 outline points
 *   sphere:   13 points,  30 links, 12 outline points
 * If a builder ever overflowed, blob_add_con just drops the extra links
 * quietly — but our shapes stay well under the caps.
 *
 * The outline list (bnd) matters a lot for collisions. It names the points
 * around the body's edge, always walked the same way around (one fixed
 * direction). Three places trust that ordering:
 *   - the inside/outside test in §6 (which way around doesn't matter, only
 *     that we pick one direction and never change it)
 *   - the nearest-edge search in §6, which treats consecutive outline
 *     points as edges
 *   - the fill step in draw_blob (§8), which walks the same loop
 * A builder that produced a tangled or dented outline would quietly break
 * all three.
 */
typedef struct {
  /* nodes — every point in the body. n_nodes is set when the body is built
   * (36 for a cube, 13 for a sphere) and never changes after that. */
  Node nodes[BLOB_NODES_MAX];
  int n_nodes;

  /* cons — the shape-holding links. The order they're added is the order
   * they're applied each step, and that order does affect how the body
   * settles, so the builders add them in a deliberate sequence (cube: edges
   * then diagonals; sphere: hoop, spokes, then across-the-middle). Don't
   * shuffle them or the jelly feel changes. */
  Con cons[BLOB_CONS_MAX];
  int n_cons;

  /* bnd — the outline: which points form the body's edge, in order around
   * it (see the note above). Fixed once built; changing it would corrupt
   * collision and drawing. */
  int bnd[BLOB_BND_MAX];
  int n_bnd;

  /* surf_cp / fill_cp — the two colours this body draws in: the bright
   * outline (surf) and the darker inside (fill). Set once when the body is
   * spawned. Switching themes keeps working because a theme just re-paints
   * these same colour slots with new shades — the body keeps its slot. */
  int surf_cp, fill_cp;

  /* kind — cube or sphere. Only used to choose the colour family. */
  BKind kind;
} Blob;

/*
 * Scene — everything the demo needs to remember: the bodies on screen, the
 * dials that shape how they move, and a few display settings. There's just
 * one of these (g_scene below), so helpers reach for it directly instead of
 * passing it around.
 *
 * The fields fall into two groups, and the split is the important part:
 *   - Simulation settings change what the bodies DO (how fast they fall, how
 *     stiff they are). The physics and collision code reads these.
 *   - Rendering settings are pure looks (the colour theme, the fps readout).
 *     Changing one of these must never nudge a body, only repaint it.
 * When you add a field, ask which group it belongs to: if it changes how a
 * body moves, it's a simulation setting; if it's only about appearance, it's
 * a rendering one. Putting a movement field in the looks group would quietly
 * tie the picture to the physics — the exact mix-up this split prevents.
 *
 * A few things deliberately live outside this struct, as file-scope globals:
 *   - g_rows, g_cols: the screen size. The main loop's resize handler owns
 *     these so the physics never has to think about the terminal's shape.
 *   - g_quit, g_resize: flags the signal handler sets. They have to be
 *     file-scope sig_atomic_t — that's the only kind of variable C promises
 *     a signal handler can touch safely.
 *   - g_has_256: whether the terminal has 256 colours, checked once at boot.
 *   - g_mincol[], g_maxcol[]: scratch space the fill step rewrites every
 *     frame, not real state worth keeping on the Scene.
 */
typedef struct {
  /* ── Simulation settings: these decide how the bodies behave ── */

  /* blobs / nblobs — the bodies currently on screen, packed at the front
   * of the array. Spawning adds one on the end; deleting just drops the
   * last one. Capacity is MAX_BLOBS, and that cap matters: collisions
   * check every pair, so the work grows with the square of how many there
   * are. Once the array is full, the spawn helpers refuse to add more. */
  Blob blobs[MAX_BLOBS];
  int nblobs;

  /* ncubes / nsphs — how many cubes and spheres have ever been spawned.
   * Only used to pick the next colour, so each new body of a kind looks a
   * little different from the last. Kept separate from nblobs so the
   * colour cycle keeps going even as bodies come and go. */
  int ncubes, nsphs;

  /* paused — when true the world freezes; nothing moves until you unpause,
   * and it picks up exactly where it left off. */
  bool paused;

  /* steps — how many physics updates we do per drawn frame. Several small
   * steps look smoother than one big one. */
  int steps;

  /* tick — counts physics steps since the last reset. Handy hook for
   * "every N steps" effects; nothing uses it right now. */
  long tick;

  /* rng — the random-number state used to scatter where new bodies appear.
   * Seeded once at boot, so the same sequence of keypresses gives the same
   * placements every run — convenient for reproducing a bug. */
  uint32_t rng;

  /* gravity_on — master switch for falling. Turn it off and bodies stop
   * being pulled down, though whatever motion they already have still
   * fades away. */
  bool gravity_on;

  /* gravity — how hard each step pulls a point downward. Tuned so bodies
   * reach a comfortable falling speed rather than rocketing off-screen. */
  float gravity;

  /* damping — the fraction of speed a point keeps each step (0 to 1).
   * A touch under 1 so bodies drift a moment, then settle instead of
   * jiggling forever. */
  float damping;

  /* pbd_iters — how many times per step we tug the points back toward
   * their resting shape. More passes = stiffer body; fewer = softer and
   * more jelly-like. The i / I keys raise and lower it. */
  int pbd_iters;

  /* ── Rendering settings: pure looks, never touch the physics ── */

  /* theme — which colour scheme is active (an index into k_themes). The
   * t / T keys cycle it; bodies don't move when it changes. */
  int theme;

  /* fps_disp — the frame rate shown in the top bar, refreshed once a
   * second by the main loop. The drawing code just reads it. */
  int fps_disp;
} Scene;

/* The one and only Scene. The fields listed here start with sensible
 * non-zero values (gravity on, a gentle pull, light damping, a medium
 * stiffness, a fixed random seed); everything else starts at zero, and
 * scene_init fills in the opening bodies before the first step. */
static Scene g_scene = {
    .gravity_on = true,
    .gravity = GRAVITY_DEF,
    .damping = DAMPING_DEF,
    .pbd_iters = PBD_ITERS_DEF,
    .steps = STEPS_DEF,
    .rng = 0xdeadbeef,
};

/* Hands back a random number between 0 and 1, used to scatter where new
 * bodies appear. Good enough for spreading bodies out — not for anything
 * that needs real randomness. */
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

/*
 * Building a body. Each builder runs the same four steps in order:
 *   1. drop the points into place,
 *   2. add the firm links that hold its size,
 *   3. add the softer diagonal links that stop it leaning over,
 *   4. record which points trace the outline, walked one way around.
 * The order matters. Adding firm links before diagonal ones nudges the
 * shape-fixing pass to settle a little faster, and walking the outline
 * consistently one direction is what the inside/outside test in §6 relies
 * on.
 */

/* ── cube primitives ── */

/* Lay out the cube's points as a grid. We park each point's "last position"
 * right on top of its current one so the body starts perfectly still. */
static void install_cube_grid_nodes(Blob *bl, float ox, float oy) {
  for (int r = 0; r < CUBE_H; r++)
    for (int c = 0; c < CUBE_W; c++) {
      int i = r * CUBE_W + c;
      bl->nodes[i].x = bl->nodes[i].px = ox + c * CUBE_SP;
      bl->nodes[i].y = bl->nodes[i].py = oy + r * CUBE_SP;
    }
}

/* Link each point to its right and below neighbour. These firm links keep
 * the cube from stretching or squashing along its rows and columns. */
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

/* Add both diagonals across every little square of the grid. Without these
 * the cube would just fold over into a slanted shape under gravity — the
 * row/column links keep the edges the right length but do nothing to stop
 * the whole thing leaning. */
static void install_cube_shear_cons(Blob *bl) {
  for (int r = 0; r < CUBE_H - 1; r++)
    for (int c = 0; c < CUBE_W - 1; c++) {
      int i = r * CUBE_W + c;
      blob_add_con(bl, i, i + CUBE_W + 1, SHEAR_K); /* \ diag */
      blob_add_con(bl, i + 1, i + CUBE_W, SHEAR_K); /* / diag */
    }
}

/* Record the points around the cube's outer edge, going around in one
 * steady direction: across the top, down the right, back across the bottom,
 * up the left. This gives §6 a clean closed outline to test against. */
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

/* Build a cube: clear it out, give it its colours, then run the four steps. */
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

/* ── sphere primitives ── */

/* Place the sphere's points: one in the middle, the rest evenly spaced
 * around it in a ring. The ring is stretched twice as tall as it is wide so
 * that, on tall terminal cells, it actually looks round. */
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

/* Link each ring point to the next one around, forming the closed outer
 * loop. These firm links hold the rim's shape, like the cube's edge links. */
static void install_sphere_hoop_cons(Blob *bl) {
  for (int i = 0; i < SPH_RING; i++)
    blob_add_con(bl, 1 + i, 1 + (i + 1) % SPH_RING, STRUCT_K);
}

/* Tie every ring point to the centre, like spokes on a wheel. These keep
 * the sphere from being squeezed in or stretched out from the middle. */
static void install_sphere_spoke_cons(Blob *bl) {
  for (int i = 0; i < SPH_RING; i++)
    blob_add_con(bl, 0, 1 + i, STRUCT_K);
}

/* Link each ring point to the one straight across from it. These across-
 * the-middle links stop the sphere from being pinched into an oval — the
 * sphere's version of the cube's diagonals. */
static void install_sphere_diameter_cons(Blob *bl) {
  for (int i = 0; i < SPH_RING / 2; i++)
    blob_add_con(bl, 1 + i, 1 + i + SPH_RING / 2, SHEAR_K);
}

/* The outline is just the ring points in order (the centre isn't on the
 * edge). They were placed going one way around, which is what §6 wants. */
static void install_sphere_boundary_ccw(Blob *bl) {
  for (int i = 0; i < SPH_RING; i++)
    bl->bnd[i] = 1 + i;
  bl->n_bnd = SPH_RING;
}

/* Build a sphere: clear it out, give it its colours, then run the steps. */
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

/* ── §5  physics ── */

/* The current screen size, kept up to date by the resize handler in the
 * main loop. It lives out here (not on the Scene) so the physics never
 * has to care about screen size. */
static int g_rows, g_cols;

/*
 * One physics step for a single body, in three parts:
 *   1. let every point drift forward on its own (fall, coast),
 *   2. tug the points back toward the body's resting shape a few times,
 *      keeping them inside the screen as we go,
 *   3. handle bouncing off the floor and walls.
 * Notice there's no stiffness number anywhere. How rigid the body feels
 * comes purely from how many times we run the tug pass — more passes means
 * stiffer. That's the whole appeal of this approach: no springs to blow up,
 * just move points and pull them back.
 *
 * The helpers below match these three parts in order.
 */

/* Move every point forward on its own. Its speed is just how far it moved
 * last step; we trim that a touch (so motion fades), then carry it the same
 * way again and add a nudge of gravity. We stash the old spot first so next
 * step can read the speed off the difference again. */
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

/* One shape-fixing pass over every link. For each, check how far apart its
 * two points have drifted from the distance it wants, then move both points
 * toward fixing it — each one half the gap, so neither is favoured. If two
 * points have landed right on top of each other (which can happen right
 * after a hard collision shove), we skip that link to avoid dividing by a
 * near-zero distance. */
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

/* Pin every point back inside the screen if it has wandered out. This only
 * moves points; the bounce and friction come afterward. The ceiling sits a
 * little below the very top so bodies never climb into the status bar. */
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

/* Bounce points off the floor and walls. The trick: we leave the point
 * where it is and instead edit where it *was*, which flips its direction so
 * it heads back the other way, a bit slower (bounces lose energy). On the
 * floor we also drag its sideways motion a little, so a body landing hard
 * doesn't keep skating forever. The walls only bounce, no drag, so a body
 * sliding along one isn't slowed for no reason. */
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

/* Run the three parts of one physics step for a body, in order. */
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

/* ── §6  collision (any pair: cube-cube, sphere-sphere, or mixed) ── */

/*
 * When two bodies overlap, we untangle them. For each pair we push A's
 * points out of B, then B's points out of A, and repeat that a couple of
 * times. Doing both directions, both times, is what makes a collision look
 * like a real mutual shove rather than one body always winning and the
 * other always giving way.
 */

/* Is the point (px, py) inside this body's outline? We shoot an imaginary
 * line straight to the right and count how many times it crosses the
 * outline: an odd number means we started inside, even means outside. The
 * slightly fussy condition on which edges count is there so a line that
 * grazes exactly through a corner is counted once, not twice. */
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

/* Find the spot on the body's outline nearest to a given point, and report
 * back: which direction points from the outline out toward that point (the
 * way we'd shove the point to get it clear), how far off the outline it is,
 * and which two outline points form that nearest edge. We just try every
 * edge and keep the closest — the bodies are small, so that's plenty fast. */
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
      continue; /* skip an edge whose two ends sit on the same spot */

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

/* Shove A's stuck point back out, away from B's surface. It moves half the
 * overlap; push_edge_inward moves B the other half, so the two share the
 * fix evenly and neither gets a free push. */
static void push_node_outward(Blob *a, int i, float nx, float ny, float depth) {
  float push = (depth + 0.5f) * 0.5f;
  a->nodes[i].x -= nx * push;
  a->nodes[i].y -= ny * push;
}

/* The other half of the fix: dent B's surface inward at the two outline
 * points nearest the contact, a quarter of the overlap each. That adds up
 * to B giving as much as A was pushed out — a fair, even trade. */
static void push_edge_inward(Blob *b, int ea, int eb, float nx, float ny,
                             float depth) {
  float push = depth * 0.25f;
  b->nodes[ea].x += nx * push;
  b->nodes[ea].y += ny * push;
  b->nodes[eb].x += nx * push;
  b->nodes[eb].y += ny * push;
}

/* Untangle A from B in one direction: for each of A's points that's poked
 * inside B, shove it out and dent B inward to match. blob_collide runs this
 * both ways so the contact comes out even. */
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

/* Untangle a pair of bodies, pushing both ways a couple of times so neither
 * one wins the contact. */
static void blob_collide(Blob *a, Blob *b) {
  for (int iter = 0; iter < COLL_ITERS; iter++) {
    resolve_penetration(a, b);
    resolve_penetration(b, a);
  }
}

/* ── §7  scene ── */

/* Give each new body a colour. Cubes cycle through slots 0-2, spheres
 * through 3-5, so bodies of the same kind don't all look identical. */
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

/* ── §8  draw ── */

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
  /* Fill the inside. For each screen row we find the leftmost and rightmost
   * edge of the outline, then colour the gap between them. */
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

  /* Draw the links as lines, picking - | / or \ to match each one's slant. */
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

  /* Mark each outline point with a bright O. */
  attron(COLOR_PAIR(bl->surf_cp) | A_BOLD);
  for (int i = 0; i < bl->n_bnd; i++) {
    const Node *nd = &bl->nodes[bl->bnd[i]];
    int cr = PHY_TO_ROW(nd->y), cc = PHY_TO_COL(nd->x);
    if (cr >= 0 && cr < floor_row && cc >= 0 && cc < cols)
      mvaddch(cr, cc, 'O');
  }
  attroff(COLOR_PAIR(bl->surf_cp) | A_BOLD);
}

/* The status bar across the top: counts, settings, and the frame rate. Its
 * colours stay the same in every theme so it's always easy to read against
 * whatever's on screen. */
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

/* ── §9  screen ── */

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

/* ── §10  app ── */

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
