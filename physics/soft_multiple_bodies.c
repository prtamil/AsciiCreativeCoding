/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * soft_multiple_bodies.c — destructible jelly towers you can knock over.
 *
 * A jelly-like tower stands on the floor with its feet glued down.  Press
 * SPACE to fling a soft ball at it.  Where the ball hits, the links holding
 * the tower together stretch too far and snap, so the top breaks off and
 * flops to the ground while the glued base wobbles and settles.
 *
 * The trick that keeps it from melting into goo: each tower is ONE big body,
 * never a stack of separate bodies, and its bottom row is pinned in place so
 * it can't slide or topple.  The "springs" between points can tear once
 * stretched too far, which is what gives the breaking effect.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/soft_multiple_bodies.c \
 *       -o soft_multiple_bodies -lncurses -lm
 *
 * The ideas behind the physics (cited inline as [n]):
 *   [1] Müller et al. (2007) "Position Based Dynamics" — the move-the-points-
 *       to-satisfy-distances solver, and the pinning trick (a glued point has
 *       zero weight so it never moves).
 *   [2] Bender, Müller, Macklin (2017) "A Survey on Position-Based Simulation
 *       Methods in Computer Graphics" — modern overview of pinning + tearing.
 *   [3] Jakobsen (2001) "Advanced Character Physics" — store last position
 *       instead of velocity; speed is just (now − last).
 *   [4] Ericson (2005) Real-Time Collision Detection §5.1.5 — closest point
 *       on a line segment.
 *   [5] Sunday (2001) "Inclusion of a Point in a Polygon" — the ray-cast
 *       inside/outside test.
 *   [6] Provot (1995) cloth-tearing paper — snip a link once it stretches
 *       past a length threshold; that's our break threshold.
 *   [7] Padala, NCURSES Programming HOWTO — the terminal drawing layer.
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

/* ── §1 config ── */

#define ROWS_MAX 128
#define COLS_MAX 512

/* How big one body can get.  Everything is sized statically (no malloc), so
 * these are the hard ceilings.  The biggest tower is 6 wide by 18 tall, which
 * is 108 points and 362 links, and these caps leave room to spare. */
#define BLOB_NODES_MAX 256 /* mass points per body   */
#define BLOB_CONS_MAX 900  /* links per body         */
#define BLOB_BND_MAX 128   /* points on the outline  */

/* How many bodies can exist at once: a preset can spawn up to 2 towers, plus
 * the balls you fire before old ones drift off.  The collision check compares
 * every pair of bodies, so keeping this small keeps it fast. */
#define MAX_BLOBS 16

/* The projectile ball — a ring of points around a centre. */
#define SPH_RING 12 /* points around the rim (point 0 is the centre) */
#define SPH_NODES (SPH_RING + 1)
#define SPH_R 4.0f /* radius in columns */

/* How firmly a link pulls its two points back to the right distance each
 * pass.  1.0 = pull all the way (stiff); lower = springier.  This is about
 * stiffness only; it has nothing to do with when a link snaps. */
#define STRUCT_K 1.0f /* the grid's edges: stiff   */
#define SHEAR_K 0.7f  /* the diagonals: a bit soft */

/* How far a link can stretch before it snaps, as a multiple of its rest
 * length.  1.0 means it breaks the instant it stretches at all; bigger means
 * it can stretch more first.  1.4–2.0 looks like brittle material that
 * shatters on a hard hit.  Tunable live with b / B. */
#define BREAK_THRESH_DEF 1.6f
#define BREAK_THRESH_MIN 1.1f
#define BREAK_THRESH_MAX 3.0f
#define BREAK_THRESH_STEP 0.1f

/* The feel of the physics, applied each sub-step. */
#define GRAVITY_DEF 0.06f  /* pull downward                       */
#define DAMPING_DEF 0.985f /* fraction of speed kept (drag)       */
#define FLOOR_REST 0.10f   /* bounciness off the floor            */
#define WALL_REST 0.10f    /* bounciness off the walls            */
#define FLOOR_FRIC 0.85f   /* sideways speed kept on the floor    */

/* How many times per sub-step we nudge all the links back toward their rest
 * lengths.  More passes = a stiffer, more solid-feeling body (and, since
 * collisions are interleaved with these passes, tighter contacts too).  10
 * makes the tower feel rigid right up until the ball hits it. */
#define PBD_ITERS_DEF 10
#define PBD_ITERS_MIN 2
#define PBD_ITERS_MAX 30

/* Physics sub-steps per drawn frame.  Splitting one frame into smaller chunks
 * stops a fast ball from skipping straight through a thin wall. */
#define STEPS_DEF 3

/* The ball's starting sideways speed when you fire it.  Fast enough to dent
 * and tear the tower it hits. */
#define PROJECTILE_VX 3.5f

/* Turning physics positions into terminal cells.  Each column is 1 unit
 * across; each row is 2 units tall, because terminal characters are about
 * twice as tall as they are wide.  We keep the top row clear for the status
 * bar so bodies never draw over it. */
#define HUD_TOP_PX 2.0f
#define PHY_TO_ROW(y) ((int)((y) * 0.5f + 0.5f))
#define PHY_TO_COL(x) ((int)((x) + 0.5f))

/* Timing. */
#define SIM_FPS 20
#define NS_PER_SEC 1000000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* Colour themes you can cycle through with t / T. */
#define N_THEMES 5

/* Tower layouts you can cycle through with n / N. */
#define N_PRESETS 4

typedef enum {
  PRESET_BIG_TOWER = 0, /* one big tower            */
  PRESET_TWIN_TOWERS,   /* two medium towers        */
  PRESET_MINI_TOWER,    /* one short tower          */
  PRESET_BIG_AND_SMALL, /* a small and a big tower  */
} Preset;

/* Which numbered colour-pair each thing draws with.  6 slots for body
 * colours, then fixed slots for the floor, pinned feet, and the two HUD
 * bars. */
#define N_BSLOTS 6
#define CP_BSURF(i) (1 + (i))            /* pairs 1-6  */
#define CP_BFILL(i) (1 + N_BSLOTS + (i)) /* pairs 7-12 */
#define CP_FLOOR (1 + 2 * N_BSLOTS)      /* 13                 */
#define CP_PIN (2 + 2 * N_BSLOTS)        /* 14 — pinned nodes  */
#define CP_HUD (3 + 2 * N_BSLOTS)        /* 15 — top status    */
#define CP_HINT (4 + 2 * N_BSLOTS)       /* 16 — bottom hints  */

/* ── §2 clock ── */

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

/* ── §3 color / theme ── */

/* Theme — one colour scheme for the whole scene.
 *
 * Switching themes (t / T) only changes colours; nothing about the physics
 * moves.  That's why the chosen theme lives with the scene's drawing settings
 * and never on a body or a point.
 *
 * Two colours per slot give bodies a shaded, 3-D look:
 *   - surf[i]  the brighter outline colour, used for the wire mesh and the
 *              bold 'O'/'*' point markers.
 *   - fill[i]  a dimmer colour for the ':' speckle filling the inside, so the
 *              outline pops against it.
 * There are 6 slots: 0–2 are a cool family for the towers, 3–5 a warm family
 * for the balls, so each new body gets a different colour as you spawn them.
 *
 * Every colour sits in the bright half of the 256-colour set (index 24 and
 * up).  The darkest entries would otherwise blend into a black background and
 * the body would vanish (see CLAUDE.md / documentation/COLOR.md). */
typedef struct {
  /* Short name shown in the HUD's theme field.  Keep it under ~7 characters
   * so the whole status line still fits an 80-column terminal. */
  const char *name;

  /* Outline colour for each of the 6 slots: the wire mesh and the bold point
   * markers.  The brighter of the two so the silhouette reads clearly. */
  short surf[N_BSLOTS];

  /* Inside-fill colour for each slot: the dim ':' speckle within the body.
   * Darker than surf so the outline stands out on top of it. */
  short fill[N_BSLOTS];

  /* Colour for the pinned (glued-down) feet, drawn as '*'.  Kept separate
   * from the body slots so every tower shows its anchored base in the same
   * eye-catching accent that contrasts with all the body colours. */
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

/* ── §4 state — Node, Con, Blob, Scene ── */

/* Node — one tiny point of mass in a soft body.
 *
 * A whole body is just a crowd of these points held together by links
 * (Con, below).  The body's shape and how stiff it feels both come from
 * the web of links — a single point doesn't know any of that.  All it
 * carries is where it is now, where it was a moment ago, and whether it's
 * glued in place.
 *
 * Why keep the old position instead of a speed?  If you know where a point
 * was and where it is now, the gap between them already tells you its speed
 * and direction — no separate speed field needed.  That's the Verlet trick
 * (Jakobsen [3]).  It has a nice bonus: whenever a link yanks a point to a
 * new spot, the next step automatically treats that jump as motion, so we
 * never have to patch up a speed by hand (Müller [1]).
 *
 * Glued points (pinned == true) simply never move — every physics step
 * checks the flag and leaves them alone.  They're the anchor the rest of
 * the body hangs from; without them gravity would drag the whole tower to
 * the floor and pile it up against the bottom (Müller [1]). */
typedef struct {
  /* Where the point is right now, in physics units.  Almost every step
   * nudges this: gravity, the links pulling it back into shape, the
   * walls, and collisions.  Drawing reads it to find the screen cell. */
  float x, y;

  /* Where the point was on the previous step.  We save it just before
   * moving, so the gap (x - px) tells us this point's speed.  The wall
   * and floor bounce works by tweaking this old position to flip the
   * speed, rather than touching x/y directly. */
  float px, py;

  /* True if this point is glued in place and must never move.  Set once
   * when the tower is built (its bottom row) and never changed after.
   * A glued point acts as if it weighed infinitely much — no force budges
   * it.  Drawn with a different mark ('*') so the anchored base stands out. */
  bool pinned;
} Node;

/* Con — one stretchy link between two points, the kind that can snap.
 *
 * Links are what hold a body together.  A link just remembers two points
 * and how far apart they're supposed to be; every physics pass measures
 * the real gap and slides the points to fix it.  Both points free? Each
 * moves halfway.  One point glued? The free one does all the moving — so
 * "glued in place" falls out of the math for free, no special case
 * (Müller [1]).
 *
 * Snapping: if a link gets stretched past its breaking ratio, we set its
 * stiffness to 0 to mark it dead.  After that the physics and the drawing
 * both ignore it, so a torn link shows up as a gap in the mesh.  Once
 * broken it stays broken — links never heal (Provot's cloth-tearing
 * trick [6]). */
typedef struct {
  /* a, b — which two points this link connects, given as their slots in
   * the body's nodes[] array.  Set once when the body is built and never
   * checked again, so don't shrink the node count after links exist. */
  int a, b;

  /* rest — the distance the two points "want" to be apart, measured once
   * when the link is created.  Never changes, which is what gives the
   * body its springy memory: stretch it, let go, it pulls back to this. */
  float rest;

  /* k — how hard the link pulls, from 0 to 1.  Higher = stiffer.  0 is
   * special: it means "this link is broken" — the physics and the
   * drawing both skip it.  We use 1.0 for the grid edges and 0.7 for the
   * softer diagonals. */
  float k;

  /* break_thresh — how far the link can stretch before it snaps, as a
   * multiple of its rest length.  0 means "never snaps" (the ball uses
   * this so it survives impacts).  1.0 snaps the moment it stretches;
   * 1.4–2.0 feels like brittle stuff that holds, then shatters on a hard
   * hit.  Live-tunable with the b / B keys. */
  float break_thresh;
} Con;

/* BKind — what shape a body is: a grid (the towers) or a ball.
 *
 * This only steers which colour family the body gets — towers get cool
 * colours, balls get warm ones.  The physics, collisions, and drawing
 * treat both shapes exactly the same.  To add a new shape: add it here,
 * write a builder, pick a colour. */
typedef enum { BKIND_SLAB, BKIND_SPHERE } BKind;

/* Blob — one whole soft body: either a tower or a fired ball.
 *
 * A Blob carries everything one body needs to move, bump into other
 * bodies, and draw itself.  No pointers and no malloc — every array is a
 * fixed size, so a body is just a chunk of memory.
 *
 * The key design choice is "one big body, never a stack of small ones".
 * The old approach stacked separate bodies and they melted into goo: the
 * weight piling up from above pushes harder than the links can recover
 * each frame.  So instead each tower is a SINGLE body with its bottom row
 * glued to the floor.  Presets with two towers just place two such bodies
 * side by side, never one on top of another — the only time two bodies
 * touch is when the ball hits a tower, a quick one-off bump rather than a
 * constant load.
 *
 * The array sizes are set so the biggest tower (6×18 points, 362 links)
 * fits in one body with room to spare.
 *
 * About the outline (bnd): it's the list of points around the body's edge,
 * walked counter-clockwise.  Collisions and the inside-fill both use it.
 * One wrinkle — when links snap, the outline's point list stays the same,
 * but those points have drifted, so the outline can stop matching the real
 * jagged shape of a shattered body.  We live with that: tracking the true
 * shape of every broken-off piece would be a lot of code for little gain
 * in a demo.
 *
 * The ideas behind it: links + gluing from Müller [1], the no-velocity
 * point trick from Jakobsen [3], snappable links from Provot [6]. */
typedef struct {
  /* The three arrays below ARE the body: where its weight sits (nodes),
   * what holds it together (cons), and its outline (bnd).  All three are
   * filled in once when the body is built; afterward the points move and
   * links may snap, but the counts never change. */

  /* nodes — the body's points.  Slots never move around, so the indices
   * in cons[] and bnd[] stay valid for the body's whole life. */
  Node nodes[BLOB_NODES_MAX];
  int n_nodes;

  /* cons — the links.  They're fixed each pass in the order they were
   * added (grid edges first, then diagonals); keep that order stable
   * because changing it slightly changes how the body settles. */
  Con cons[BLOB_CONS_MAX];
  int n_cons;

  /* bnd — the outline: which points sit on the body's edge, in counter-
   * clockwise order.  Set once at build time; don't touch it afterward
   * or both collision and inside-fill break. */
  int bnd[BLOB_BND_MAX];
  int n_bnd;

  /* The two colour slots below are picked when the body is built.  They
   * survive a theme change because switching themes just repaints the
   * same slot numbers with new colours rather than re-tagging bodies. */

  /* surf_cp — colour for the outline mesh and the bold 'O' point marks.
   * The brighter of the body's two colours. */
  int surf_cp;

  /* fill_cp — colour for the ':' speckle inside the body.  Dimmer than
   * surf_cp so the outline stands out on top of it. */
  int fill_cp;

  /* kind — tower or ball.  Only used to pick a colour family; the
   * physics and collisions don't care which it is. */
  BKind kind;
} Blob;

/* Scene — all the demo's state in one place: the bodies plus the knobs
 * that control them.  There's exactly one of these (g_scene), so helpers
 * just reach for it directly instead of passing it around.
 *
 * The fields fall into two groups, and the split is for the reader.  The
 * simulation group decides what the bodies actually do; the rendering
 * group only changes how things look.  The test for which group a new
 * field belongs in: would changing it move a single point differently?
 * If yes it's simulation, if no it's rendering.  Keeping a colour-only
 * flag out of the simulation group is what guarantees that switching
 * themes can never accidentally nudge the physics.
 *
 * A few things deliberately live outside Scene: the screen size
 * (g_rows/g_cols) belongs to the resize logic in the main loop; the
 * quit/resize flags must be file-scope to be safe to set from a signal
 * handler; and the scan-fill scratch arrays are just throwaway workspace
 * for drawing. */
typedef struct {
  /* ── simulation: what the bodies do ── */

  /* blobs — the live bodies.  Slot 0 is usually the tower(s) from the
   * current preset; later slots are the balls you fire.  New bodies get
   * added on the end, and the x key drops the most recent one.  Every
   * body gets the same physics — towers aren't special-cased. */
  Blob blobs[MAX_BLOBS];
  int nblobs;

  /* nballs — how many balls have been fired since boot, ever (not how
   * many are alive now).  Used only to give each new ball a fresh
   * colour, even after you've deleted some. */
  int nballs;

  /* preset — which tower layout is loaded, a number in [0, N_PRESETS).
   * Switching it (n/N keys) rebuilds the whole scene from scratch. */
  int preset;

  /* paused — when true, the physics is skipped each frame so everything
   * freezes; unpausing picks up exactly where it left off. */
  bool paused;

  /* steps — how many physics sub-steps run per drawn frame.  Splitting a
   * frame into smaller chunks stops a fast ball from skipping through a
   * thin wall. */
  int steps;

  /* tick — a simple step counter, reset on rebuild.  Nothing reads it
   * yet; it's here for future "do something every N steps" hooks. */
  long tick;

  /* rng — random-number state, seeded at boot.  Currently unused (every
   * tower sits at a fixed spot); kept here so future random features
   * don't need a new global. */
  uint32_t rng;

  /* gravity_on — the g key's on/off switch for gravity.  Turned off,
   * bodies keep whatever motion they had and gently coast to a stop. */
  bool gravity_on;

  /* gravity — how hard things are pulled down each sub-step.  Tuned so
   * bodies fall at a nice pace without punching through thin walls. */
  float gravity;

  /* damping — the fraction of speed a point keeps each sub-step (a touch
   * of drag).  0.985 lets bodies coast a bit but settle within a few
   * seconds. */
  float damping;

  /* pbd_iters — how many times per sub-step we nudge all the links back
   * toward their rest lengths.  More passes = a stiffer, more solid body
   * (and, since collisions happen between these passes, firmer contacts
   * too).  Nudge it live with i/I. */
  int pbd_iters;

  /* break_thresh — the breaking ratio handed to every NEW link.
   * Changing it doesn't affect bodies already built — they captured the
   * old value — so the workflow is: change it with b/B, then press r to
   * rebuild and feel the difference. */
  float break_thresh;

  /* ── rendering: how things look ── */

  /* theme — which colour scheme is active; cycled with t/T.  Changing it
   * only repaints colours, leaving every body's positions and links
   * exactly as they were. */
  int theme;

  /* fps_disp — the frame rate shown in the HUD, refreshed once a second
   * by the main loop.  The drawing code just reads it. */
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

/* ── §5 physics — move points, fix links, bounce off walls ── */

/* One physics pass goes: let every point drift under gravity, then nudge
 * all the links back to length a bunch of times (fixing collisions in
 * between), then bounce anything that hit a wall.  scene_step (§7) runs
 * exactly this.  The helpers below handle the two special cases: glued
 * points never move, and broken links (stiffness 0) are skipped. */

/* verlet_predict_all — let each free point coast for one step.
 *
 * We never store a speed.  Instead, the gap between where a point is and
 * where it was last step already tells us how fast it's going, so we just
 * carry it forward by that same gap and add gravity (Jakobsen [3]).
 * Glued points don't move; we still copy their old spot forward so they'd
 * start from rest if they were ever un-glued. */
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

/* Fixing the links, once over the whole body.  For each link: skip it if
 * it's already broken, measure how far apart its two points are, snap it
 * if it's overstretched, otherwise slide the points back toward the right
 * distance.  We check for breaking BEFORE fixing, so a link that's about
 * to snap doesn't get quietly pulled back into shape first (Provot [6]). */

/* constraint_is_alive — false once a link has snapped (stiffness 0).
 * Worth the check: on a shattered tower it skips about half the links. */
static inline bool constraint_is_alive(const Con *c) { return c->k != 0.f; }

/* measure_constraint_distance — how far apart the two points are, handing
 * back both the straight-line gap and its x/y parts (the break test needs
 * the gap, the fix-up needs the parts). */
static inline void measure_constraint_distance(const Node *a, const Node *b,
                                               float *dx, float *dy, float *d) {
  *dx = b->x - a->x;
  *dy = b->y - a->y;
  *d = sqrtf((*dx) * (*dx) + (*dy) * (*dy));
}

/* check_and_break_if_overstretched — if a link is stretched past its
 * breaking ratio, kill it (stiffness 0, permanent) and tell the caller to
 * skip it this pass.  A breaking ratio of 0 means "never snaps", which is
 * what the ball uses (Provot [6]). */
static inline bool check_and_break_if_overstretched(Con *c, float d) {
  if (c->break_thresh > 0.f && d > c->rest * c->break_thresh) {
    c->k = 0.f;
    return true;
  }
  return false;
}

/* project_pbd_distance — slide the two points back toward the link's rest
 * distance.  A glued point counts as weightless so it never moves: if
 * both ends are glued there's nothing to do, if one end is glued the free
 * end does all the moving, and if both are free they each move halfway.
 * That single weight trick handles all three cases with no branching
 * (Müller [1]). */
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
      continue; /* points on top of each other — no direction to push */

    if (check_and_break_if_overstretched(c, d))
      continue;

    project_pbd_distance(a, b, c, dx, dy, d);
  }
}

/* clamp_all_nodes_to_world — shove any point that has wandered off-screen
 * back inside the play area.  Glued points are left alone; the builder
 * already placed them somewhere valid. */
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

/* apply_boundary_velocity_response — bounce free points off the walls and
 * floor.  Since a point's speed is just the gap to its old spot, we bounce
 * it by moving that old spot to the other side, which flips the direction
 * it's heading.  The floor also bleeds off sideways speed, so things slow
 * down and stop instead of sliding forever. */
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

/* ── §6 collision — when one body overlaps another ── */

/* point_in_polygon_jordan — is a point inside a body's outline?  Shoot an
 * imaginary ray sideways from the point and count how many times it
 * crosses the outline: an odd number means inside, even means outside.
 * The slightly lopsided edge test (touch the bottom end, miss the top)
 * keeps a ray that grazes a corner from being counted twice (Sunday [5]). */
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

/* Finding the closest spot on a body's outline to a given point.  We walk
 * every edge of the outline, find the nearest spot on that edge, and keep
 * the closest one overall.  That spot tells us which way to push the
 * overlapping point back out, and how far (Ericson [4]). */

/* parametric_projection_clamped — for a point and one edge, how far along
 * the edge its closest spot sits, as a fraction from 0 (start) to 1 (end).
 * If the point is off the end of the edge we clamp to 0 or 1, so the
 * answer always lands on the edge itself, not off into space. */
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

/* point_on_segment_at — turn that fraction back into an actual spot:
 * start, plus the fraction of the way along the edge. */
static inline void point_on_segment_at(float ax, float ay, float edx, float edy,
                                       float t, float *cx, float *cy) {
  *cx = ax + t * edx;
  *cy = ay + t * edy;
}

/* euclidean_distance — straight-line distance between two points. */
static inline float euclidean_distance(float px, float py, float qx, float qy) {
  float dx = px - qx, dy = py - qy;
  return sqrtf(dx * dx + dy * dy);
}

/* outward_unit_normal — which way to push the overlapping point to get it
 * out: the direction from the closest spot on the edge back to the point.
 * If they're right on top of each other we just pick straight down, so the
 * caller always gets a usable direction. */
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

/* nearest_polygon_edge — finds the closest spot on the outline and hands
 * back the push direction, how deep the overlap is, and which two edge
 * points to dent inward.  The caller reuses all of that, so it's one
 * search rather than two. */
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
      continue; /* zero-length edge — skip it */

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

/* Pushing two overlapping bodies apart.  For each of body A's points that
 * has poked inside body B, we find how deep it went, shove that point back
 * out, and dent B's nearest edge inward a little.  Splitting the push
 * between both bodies (rather than moving just one) keeps the shove fair
 * and stops energy appearing from nowhere (Müller [1]).  Glued points on
 * either side don't budge.
 *
 * scene_step weaves these collision pushes in between the link-fixing
 * passes, so right after B gets dented its own links pull it back toward
 * shape.  That back-and-forth is what makes a hit body wobble like jelly
 * instead of smearing into liquid. */

/* compute_newton3_push_split — divide the overlap depth into how far to
 * push the poking point back out versus how far to dent each of the two
 * edge points inward.  The point gets half, each edge point a quarter,
 * with a tiny extra nudge so they separate with a hair of breathing room
 * and don't immediately re-overlap next step. */
static inline void compute_newton3_push_split(float depth, float *push_out,
                                              float *push_in) {
  *push_out = (depth + 0.5f) * 0.5f;
  *push_in = depth * 0.25f;
}

/* push_penetrator_outward — slide the poking point straight back out by
 * the given amount.  The caller already checked it isn't glued. */
static inline void push_penetrator_outward(Node *node, float nx, float ny,
                                           float mag) {
  node->x -= nx * mag;
  node->y -= ny * mag;
}

/* squish_receiver_polygon_inward — dent the hit body by pushing both ends
 * of its nearest edge inward, which is what makes a struck tower visibly
 * cave in where the ball lands.  Glued ends don't move. */
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

/* resolve_penetration — handles A poking into B.  The caller runs it both
 * ways (A into B, then B into A) so neither body gets pushed around while
 * the other sits still. */
static void resolve_penetration(Blob *a, Blob *b) {
  for (int i = 0; i < a->n_nodes; i++) {
    Node *ni = &a->nodes[i];
    if (ni->pinned)
      continue; /* glued points don't move */

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

/* ── §7 scene — build the towers, fire the ball, run one frame ── */

static const char *k_preset_names[N_PRESETS] = {
    "Big Tower",
    "Twin Towers",
    "Mini Tower",
    "Big + Small",
};

/* blob_add_con — add one link between two points, taking its rest length
 * from where the points sit right now.  A breaking ratio of 0 makes the
 * link permanent (never snaps). */
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

/* Building a tower: lay out a grid of points, link each point to its
 * neighbours so the grid keeps its shape, add diagonal links so it can't
 * shear into a squashed diamond, then trace its outline for collisions and
 * drawing.  Every link gets the same breaking ratio, so the whole tower is
 * equally fragile.  The caller glues the bottom row afterward. */

/* blob_clear_and_metadata — start a fresh body: zero it out and stamp on
 * its colours and shape. */
static void blob_clear_and_metadata(Blob *bl, int scp, int fcp, BKind kind,
                                    int n_nodes) {
  memset(bl, 0, sizeof *bl);
  bl->surf_cp = scp;
  bl->fill_cp = fcp;
  bl->kind = kind;
  bl->n_nodes = n_nodes;
}

/* lay_out_grid_node_positions — drop the points onto an evenly-spaced grid
 * starting from the top-left corner.  Each point's old spot starts equal
 * to its current spot, so the tower begins at rest. */
static void lay_out_grid_node_positions(Blob *bl, float ox, float oy, int w,
                                        int h, float sp) {
  for (int r = 0; r < h; r++)
    for (int c = 0; c < w; c++) {
      int i = r * w + c;
      bl->nodes[i].x = bl->nodes[i].px = ox + c * sp;
      bl->nodes[i].y = bl->nodes[i].py = oy + r * sp;
    }
}

/* wire_grid_axis_aligned_edges — link each point to its right and bottom
 * neighbour.  These are the stiff edges that keep the grid from stretching
 * and form the outline of every cell. */
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

/* wire_grid_cell_diagonals — link both diagonals across every grid cell.
 * Without these the grid would flop over into a leaning diamond even with
 * the edges holding their lengths; the diagonals are what keep the cells
 * square. */
static void wire_grid_cell_diagonals(Blob *bl, int w, int h,
                                     float break_thresh) {
  for (int r = 0; r < h - 1; r++)
    for (int c = 0; c < w - 1; c++) {
      int i = r * w + c;
      blob_add_con(bl, i, i + w + 1, SHEAR_K, break_thresh); /* \ */
      blob_add_con(bl, i + 1, i + w, SHEAR_K, break_thresh); /* / */
    }
}

/* trace_grid_boundary_ccw — list the points around the grid's edge,
 * walking the rim counter-clockwise: across the top, down the right side,
 * back along the bottom, up the left.  Collisions and the inside-fill both
 * walk this list. */
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

static void blob_build_grid(Blob *bl, float ox, float oy, int w, int h,
                            float sp, int scp, int fcp, float break_thresh) {
  blob_clear_and_metadata(bl, scp, fcp, BKIND_SLAB, w * h);
  lay_out_grid_node_positions(bl, ox, oy, w, h, sp);
  wire_grid_axis_aligned_edges(bl, w, h, break_thresh);
  wire_grid_cell_diagonals(bl, w, h, break_thresh);
  trace_grid_boundary_ccw(bl, w, h);
}

/* pin_grid_bottom_row — glue the tower's bottom row to the floor so it
 * can't slide or topple.  Glued points are skipped by every physics step,
 * which is exactly what makes them act like a fixed foundation. */
static void pin_grid_bottom_row(Blob *bl, int w, int h) {
  for (int c = 0; c < w; c++) {
    bl->nodes[(h - 1) * w + c].pinned = true;
  }
}

/* Building the ball, like a wagon wheel: one point at the centre, a ring
 * of points around the rim, then links along the rim, spokes to the
 * centre, and a few straight across.  The rim is drawn as a tall oval
 * because terminal cells are about twice as tall as they are wide — once
 * squashed onto the screen it reads as a round circle.  Every link on the
 * ball is unbreakable so it survives a hit and bounces; it's the only body
 * in the demo whose links can't snap. */

/* place_sphere_center_node — the hub at the middle.  It isn't drawn, but
 * the spokes hang off it so the ball can't collapse inward. */
static void place_sphere_center_node(Blob *bl, float cx, float cy) {
  bl->nodes[0].x = bl->nodes[0].px = cx;
  bl->nodes[0].y = bl->nodes[0].py = cy;
}

/* place_sphere_ring_nodes — the rim points, spaced evenly around a tall
 * oval so the ball looks round once it's drawn on screen. */
static void place_sphere_ring_nodes(Blob *bl, float cx, float cy) {
  for (int i = 0; i < SPH_RING; i++) {
    float a = 6.2831853f * i / SPH_RING;
    int ni = i + 1;
    bl->nodes[ni].x = bl->nodes[ni].px = cx + SPH_R * cosf(a);
    bl->nodes[ni].y = bl->nodes[ni].py = cy + SPH_R * 2.f * sinf(a);
  }
}

/* wire_sphere_hoop_cons — link each rim point to the next one around,
 * closing the loop.  These hold the rim's size steady so it doesn't bunch
 * up. */
static void wire_sphere_hoop_cons(Blob *bl) {
  for (int i = 0; i < SPH_RING; i++)
    blob_add_con(bl, 1 + i, 1 + (i + 1) % SPH_RING, STRUCT_K, 0.f);
}

/* wire_sphere_spoke_cons — link every rim point to the centre, like
 * spokes.  These keep the ball from caving in toward the middle. */
static void wire_sphere_spoke_cons(Blob *bl) {
  for (int i = 0; i < SPH_RING; i++)
    blob_add_con(bl, 0, 1 + i, STRUCT_K, 0.f);
}

/* wire_sphere_diameter_cons — link points on opposite sides of the rim,
 * straight across.  These stop the ball from squashing lopsided.  We only
 * need half as many as there are rim points, since each line covers two. */
static void wire_sphere_diameter_cons(Blob *bl) {
  for (int i = 0; i < SPH_RING / 2; i++)
    blob_add_con(bl, 1 + i, 1 + i + SPH_RING / 2, SHEAR_K, 0.f);
}

/* trace_sphere_ring_boundary — the ball's outline is just its rim; the
 * centre point sits inside and never counts as part of the edge. */
static void trace_sphere_ring_boundary(Blob *bl) {
  for (int i = 0; i < SPH_RING; i++)
    bl->bnd[i] = 1 + i;
  bl->n_bnd = SPH_RING;
}

static void blob_build_sphere(Blob *bl, float cx, float cy, int scp, int fcp) {
  blob_clear_and_metadata(bl, scp, fcp, BKIND_SPHERE, SPH_NODES);
  place_sphere_center_node(bl, cx, cy);
  place_sphere_ring_nodes(bl, cx, cy);
  wire_sphere_hoop_cons(bl);
  wire_sphere_spoke_cons(bl);
  wire_sphere_diameter_cons(bl);
  trace_sphere_ring_boundary(bl);
}

/* ── preset builders — each stands up one or two towers ── */

/* TowerSpec — a little recipe for one tower: how wide, how tall, how
 * spread out, and where to stand it.
 *
 * The preset builders fill in one or two of these and hand them to
 * place_tower.  Keeping the "what it looks like" recipe separate from the
 * "go build it" code means adding a new preset is just a couple of lines.
 * Towers always sit on the floor — there's no field for height off the
 * ground, because every tower in this demo is anchored to it. */
typedef struct {
  /* w — how many points wide the grid is. */
  int w;

  /* h — how many points tall the grid is.  place_tower glues the bottom
   * row to the floor. */
  int h;

  /* sp — the gap between neighbouring points.  Bigger = a physically
   * larger tower from the same number of points.  2.0–2.5 looks best. */
  float sp;

  /* cx — where to centre the tower across the screen.  Use a fraction of
   * the screen width (like ww * 0.65) so it lands in the right place on
   * any terminal size. */
  float cx;
} TowerSpec;

/* place_tower — build one tower from a recipe and stand it on the floor:
 * make the grid, glue its bottom row down, give it a colour.  Quietly does
 * nothing if there's no room left, so the preset builders don't each have
 * to check. */
static void place_tower(const TowerSpec *t) {
  if (g_scene.nblobs >= MAX_BLOBS)
    return;

  float wh = (float)(g_rows - 2) * 2.f;
  float bw = (t->w - 1) * t->sp;
  float bh = (t->h - 1) * t->sp;
  float ox = t->cx - bw * 0.5f;
  float oy = wh - bh - 0.5f; /* base just above floor */

  Blob *bl = &g_scene.blobs[g_scene.nblobs];
  int slot = g_scene.nblobs % 3; /* rotate through the cool colours */
  blob_build_grid(bl, ox, oy, t->w, t->h, t->sp, CP_BSURF(slot), CP_BFILL(slot),
                  g_scene.break_thresh);
  pin_grid_bottom_row(bl, t->w, t->h);
  g_scene.nblobs++;
}

/* preset_big_tower — one tall tower on the right.  The ball hits it low
 * and the whole top half tears off — the most dramatic single target. */
static void preset_big_tower(void) {
  float ww = (float)g_cols;
  TowerSpec t = {.w = 6, .h = 18, .sp = 2.5f, .cx = ww * 0.65f};
  place_tower(&t);
}

/* preset_twin_towers — two medium towers in a row.  The ball smashes
 * through the first and carries on into the second. */
static void preset_twin_towers(void) {
  float ww = (float)g_cols;
  TowerSpec left = {.w = 5, .h = 14, .sp = 2.5f, .cx = ww * 0.45f};
  TowerSpec right = {.w = 5, .h = 14, .sp = 2.5f, .cx = ww * 0.78f};
  place_tower(&left);
  place_tower(&right);
}

/* preset_mini_tower — one short tower that goes down in a single hit.
 * Handy for tuning how fragile things are with the b / B keys. */
static void preset_mini_tower(void) {
  float ww = (float)g_cols;
  TowerSpec t = {.w = 4, .h = 10, .sp = 2.0f, .cx = ww * 0.70f};
  place_tower(&t);
}

/* preset_big_and_small — David and Goliath: a small tower on the left, a
 * big one on the right.  The ball flattens the small one, loses some
 * steam, and only dents the big one. */
static void preset_big_and_small(void) {
  float ww = (float)g_cols;
  TowerSpec small_t = {.w = 4, .h = 10, .sp = 2.0f, .cx = ww * 0.40f};
  TowerSpec big_t = {.w = 6, .h = 18, .sp = 2.5f, .cx = ww * 0.78f};
  place_tower(&small_t);
  place_tower(&big_t);
}

/* fire_projectile — what SPACE does: drop a ball just inside the left
 * wall, up off the floor so it doesn't immediately stick, then give it a
 * rightward shove.  The shove is done by placing each point's "old spot"
 * to its left, so the body reads as already moving right.  The ball can't
 * snap or stick, so it survives the hit and bounces. */
static void fire_projectile(void) {
  if (g_scene.nblobs >= MAX_BLOBS)
    return;
  float wh = (float)(g_rows - 2) * 2.f;
  float cx = SPH_R + 1.f;            /* just inside the left wall */
  float cy = wh - SPH_R * 3.f - 1.f; /* up off the floor         */

  /* the warm colours, for balls */
  int slot = 3 + (g_scene.nballs % 3);
  Blob *bl = &g_scene.blobs[g_scene.nblobs];
  blob_build_sphere(bl, cx, cy, CP_BSURF(slot), CP_BFILL(slot));

  for (int i = 0; i < bl->n_nodes; i++)
    bl->nodes[i].px = bl->nodes[i].x - PROJECTILE_VX;

  g_scene.nblobs++;
  g_scene.nballs++;
}

/* scene_remove_last — delete the most recent body, handy for clearing old
 * balls without rebuilding the towers. */
static void scene_remove_last(void) {
  if (g_scene.nblobs > 0)
    g_scene.nblobs--;
}

/* scene_init — wipe everything and build the chosen preset from scratch.
 * Runs at startup, on reset, when you switch presets, and after a window
 * resize so the towers re-fit the new size. */
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

/* One frame of physics, split into a few sub-steps.  Each sub-step lets
 * every body drift, then loops several times over "fix the links, fix the
 * collisions", and finally bounces anything off the walls.  Fixing
 * collisions and links over and over in the same loop is the whole reason
 * the towers stay solid instead of melting: every time a body gets dented,
 * its links pull it back before the next dent lands.  The sub-steps exist
 * so a fast ball moves in small hops and can't skip through a thin wall. */

/* predict_all_free_flight — let every body coast one step (glued points
 * are skipped inside). */
static void predict_all_free_flight(void) {
  for (int i = 0; i < g_scene.nblobs; i++)
    verlet_predict_all(&g_scene.blobs[i]);
}

/* relax_all_distance_constraints — fix every body's links once, then shove
 * any point that ended up off-screen back inside.  Clamping after the fix
 * (not before) means the fix's work isn't immediately undone. */
static void relax_all_distance_constraints(float ww, float wh) {
  for (int i = 0; i < g_scene.nblobs; i++) {
    project_all_distance_constraints(&g_scene.blobs[i]);
    clamp_all_nodes_to_world(&g_scene.blobs[i], ww, wh);
  }
}

/* resolve_all_pairwise_contacts — check every pair of bodies for overlap,
 * pushing both ways so neither one gets shoved around unfairly.  It
 * compares all pairs, which is fine since there are never many bodies. */
static void resolve_all_pairwise_contacts(void) {
  for (int i = 0; i < g_scene.nblobs; i++) {
    for (int j = i + 1; j < g_scene.nblobs; j++) {
      resolve_penetration(&g_scene.blobs[i], &g_scene.blobs[j]);
      resolve_penetration(&g_scene.blobs[j], &g_scene.blobs[i]);
    }
  }
}

/* apply_arena_velocity_response — bounce every body off the walls and
 * floor.  Runs once at the end of each sub-step, after the links and
 * collisions have settled, so the bounce reacts to the body's real final
 * motion. */
static void apply_arena_velocity_response(float ww, float wh) {
  for (int i = 0; i < g_scene.nblobs; i++)
    apply_boundary_velocity_response(&g_scene.blobs[i], ww, wh);
}

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

/* ── §8 draw — paint the bodies and the status bars ── */

static int g_mincol[ROWS_MAX], g_maxcol[ROWS_MAX];

/* scanfill_edge — walk one outline edge and, for each screen row it
 * crosses, remember the leftmost and rightmost column it touches.  Do this
 * for every edge and you know how wide the body is on each row, which is
 * what the inside-fill needs. */
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

/* bresenham — draw a straight line of characters between two cells.  Used
 * to draw each link of the mesh. */
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

/* A body is drawn in three passes, bottom to top, because each pass paints
 * over the one before: first the ':' speckle inside, then the link mesh,
 * then the bold 'O' and '*' point marks on top.  Broken links are left out
 * of the mesh, so a torn spot shows up as a hole. */

/* compute_polygon_row_extents — figure out how wide the body is on every
 * row by walking its whole outline, so the fill pass knows where to paint. */
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

/* paint_scanfill_interior — fill the inside of the body with ':',
 * stopping just short of the edges so the mesh pass can draw the outline
 * on top. */
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

/* slope_glyph_for_edge — pick the character that best matches a line's
 * tilt: '|' for vertical, '-' for horizontal, '/' or '\' for the two
 * diagonals. */
static inline chtype slope_glyph_for_edge(int x0, int y0, int x1, int y1) {
  int adx = abs(x1 - x0), ady = abs(y1 - y0);
  if (adx == 0)
    return '|';
  if (ady == 0)
    return '-';
  return ((x1 - x0) * (y1 - y0) > 0) ? '\\' : '/';
}

/* stroke_constraint_wireframe — draw a line for every link that's still
 * intact.  Snapped links are skipped, so a torn region shows up as a gap
 * in the mesh — the visible sign of damage. */
static void stroke_constraint_wireframe(const Blob *bl, int floor_row,
                                        int cols) {
  attron(COLOR_PAIR(bl->surf_cp));
  for (int ci = 0; ci < bl->n_cons; ci++) {
    if (bl->cons[ci].k == 0.f)
      continue; /* snapped — leave a gap */
    const Node *a = &bl->nodes[bl->cons[ci].a];
    const Node *b = &bl->nodes[bl->cons[ci].b];
    int x0 = PHY_TO_COL(a->x), y0 = PHY_TO_ROW(a->y);
    int x1 = PHY_TO_COL(b->x), y1 = PHY_TO_ROW(b->y);
    bresenham(x0, y0, x1, y1, floor_row, cols,
              slope_glyph_for_edge(x0, y0, x1, y1));
  }
  attroff(COLOR_PAIR(bl->surf_cp));
}

/* draw_one_boundary_node — a bold 'O' for a normal point, a bold '*' for a
 * glued one, so the anchored base stands out at a glance. */
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

/* dot_boundary_nodes — draw the point marks on top of everything else.
 * Anything that's drifted off-screen (a ball poking into the status bar or
 * floor) is just skipped. */
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

static void draw_blob(const Blob *bl, int floor_row, int cols) {
  paint_scanfill_interior(bl, floor_row, cols);
  stroke_constraint_wireframe(bl, floor_row, cols);
  dot_boundary_nodes(bl, floor_row, cols);
}

/* The two status bars: parameters along the top row, key hints along the
 * bottom. */

static int count_intact_cons(const Blob *bl) {
  int n = 0;
  for (int i = 0; i < bl->n_cons; i++)
    if (bl->cons[i].k > 0.f)
      n++;
  return n;
}

static void draw_hud_top(int fps) {
  /* tally up broken links across every body so the bar can show how much
   * damage there's been */
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

  /* the floor */
  attron(COLOR_PAIR(CP_FLOOR));
  for (int c = 0; c < cols; c++)
    mvaddch(floor_row, c, '=');
  attroff(COLOR_PAIR(CP_FLOOR));

  /* the bodies */
  for (int i = 0; i < g_scene.nblobs; i++)
    draw_blob(&g_scene.blobs[i], floor_row, cols);

  /* the status bars */
  draw_hud_top(g_scene.fps_disp);
  draw_hud_bottom();
}

/* ── §9 screen — set up ncurses and handle resizing ── */

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

/* ── §10 app — the main loop ── */

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

    /* count frames; once a second, publish the rate to the HUD */
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
