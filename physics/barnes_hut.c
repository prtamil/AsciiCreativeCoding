/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * barnes_hut.c — a galaxy of up to 800 stars pulling on each other.
 *
 * The honest way to compute gravity is to check every star against every
 * other one, which gets slow fast. The Barnes-Hut trick groups far-away
 * stars into one blob and treats the whole blob as a single pull, so the
 * work grows gently instead of exploding. Three scenes to watch: a
 * spinning galaxy, a cloud collapsing in on itself, and two clusters
 * crashing together.
 *
 * The grouping trick: Barnes & Hut, "A hierarchical O(N log N)
 * force-calculation algorithm", Nature 324 (1986). Softening, orbits, and
 * the galaxy/cluster physics: Aarseth, "Gravitational N-Body Simulations"
 * (2003) and Binney & Tremaine, "Galactic Dynamics", 2nd ed. (2008). The
 * glowing trails use Bresenham line-drawing (Foley et al., "Computer
 * Graphics", 3rd ed., 2013, ch. 2). Brightness/colour ramps follow Ware,
 * "Information Visualization", 4th ed. (2020).
 */

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

/* ── §1  config ── */

enum {
  RENDER_FPS = 30,
  SIM_HZ = 60,
  FPS_UPDATE_MS = 500,

  N_BODIES_MAX = 800,
  N_BODIES_DEF = 400,
  N_BODIES_STEP = 50,

  NODE_POOL_MAX = 16000,
  QT_MAX_DEPTH = 32,

  GRID_ROWS_MAX = 120,
  GRID_COLS_MAX = 400,

  N_PRESETS = 3,
  N_THEMES = 7,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

#define CELL_W 8
#define CELL_H 16

/* How strong gravity is. Tuned so galaxy stars circle in ~10-20 seconds. */
#define G_DEF 12.0f
#define G_STEP 2.0f
#define G_MIN 1.0f
#define G_MAX 200.0f

/* A little fudge distance added to every gap so two stars that get very
 * close don't fling each other away with a near-infinite kick. */
#define SOFTENING 10.0f
#define SOFT2 (SOFTENING * SOFTENING)

/* How loose the "treat a far group as one blob" rule is. Smaller is more
 * accurate but slower; 0.5 is the value the original paper recommends. */
#define THETA_DEF 0.5f

/* How much the glowing trails fade each frame. Just under 1, so paths
 * linger for a while instead of vanishing instantly. */
#define DECAY 0.93f

/* The central black hole is this many times heavier than the star count,
 * so the disk has something firm to orbit around. */
#define BH_MASS_FACTOR 3.0f

/* ── §2  clock ── */

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

/* ── §3  color / themes ── */

/* Named slots for ncurses colour pairs. The five L1..L5 slots are a
 * dim-to-bright ramp: slow/faint stars use L1, fast/dense ones use L5. */
enum {
  CP_HUD = 1, /* top status bar (bright yellow) */
  CP_L1 = 2,  /* faintest glow */
  CP_L2 = 3,
  CP_L3 = 4,
  CP_L4 = 5,
  CP_L5 = 6, /* brightest glow */
  CP_TREE = 7,
  CP_BH = 8,   /* the central black hole */
  CP_HINT = 9, /* bottom hint bar (bright cyan) */
};

/*
 * Theme — one named colour scheme for the whole scene. Switching themes
 * only repaints; it never touches the physics. Each theme carries two
 * sets of numbers: one for terminals with 256 colours, one fallback for
 * old 8-colour terminals.
 */
typedef struct {
  const char *name;
  int hi256[5]; /* the five-step glow ramp, 256-colour terminals */
  int hi8[5];   /* same ramp for 8-colour terminals */
  int tree256;  /* quadtree overlay lines, 256-colour */
  int tree8;    /* quadtree overlay lines, 8-colour */
  int bh256;    /* the black hole's colour, 256-colour */
  int bh8;      /* the black hole's colour, 8-colour */
} Theme;

static const Theme k_themes[N_THEMES] = {
    {
        /* Galaxy — warm yellows/whites */
        "Galaxy",
        {58, 136, 178, 220, 231},
        {COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
        240,
        COLOR_WHITE,
        196,
        COLOR_RED,
    },
    {
        /* Nebula — purples */
        "Nebula",
        {54, 92, 129, 171, 213},
        {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE},
        237,
        COLOR_WHITE,
        226,
        COLOR_YELLOW,
    },
    {
        /* Fire — reds/oranges */
        "Fire",
        {52, 88, 160, 202, 226},
        {COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE},
        236,
        COLOR_RED,
        51,
        COLOR_CYAN,
    },
    {
        /* Ice — blues/cyans */
        "Ice",
        {24, 31, 39, 75, 159},
        {COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE},
        244,
        COLOR_CYAN,
        201,
        COLOR_MAGENTA,
    },
    {
        /* Mono — grays */
        "Mono",
        {240, 244, 247, 250, 254},
        {COLOR_BLACK, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
        242,
        COLOR_WHITE,
        231,
        COLOR_WHITE,
    },
    {
        /* Aurora — green -> cyan -> white, magenta anchor */
        "Aurora",
        {28, 36, 49, 87, 159},
        {COLOR_GREEN, COLOR_GREEN, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE},
        244,
        COLOR_GREEN,
        201,
        COLOR_MAGENTA,
    },
    {
        /* Plasma — purple -> pink -> yellow, cyan anchor */
        "Plasma",
        {53, 92, 165, 213, 226},
        {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_YELLOW,
         COLOR_WHITE},
        240,
        COLOR_MAGENTA,
        51,
        COLOR_CYAN,
    },
};

static void color_init(int theme) {
  start_color();
  use_default_colors();
  const Theme *th = &k_themes[theme];
  if (COLORS >= 256) {
    init_pair(CP_HUD, 226, -1); /* bright yellow */
    init_pair(CP_HINT, 51, -1); /* bright cyan   */
    init_pair(CP_TREE, th->tree256, -1);
    init_pair(CP_BH, th->bh256, -1);
    for (int i = 0; i < 5; i++)
      init_pair(CP_L1 + i, th->hi256[i], -1);
  } else {
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
    init_pair(CP_TREE, th->tree8, -1);
    init_pair(CP_BH, th->bh8, -1);
    for (int i = 0; i < 5; i++)
      init_pair(CP_L1 + i, th->hi8[i], -1);
  }
}

/* ── §4  coords — pixel <-> cell ── *
 *
 * Stars move in fine-grained "pixel" space; the terminal can only draw on
 * a coarse grid of character cells. These helpers convert between the two:
 * pw/ph give a screen's size in pixels, and px_to_cell_* round a pixel
 * position to the character cell it lands in. */

static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

static inline int px_to_cell_x(float px) {
  return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py) {
  return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5  entity — Body ── */

/*
 * Body — one star (or the central black hole) in the simulation.
 *
 * A body is born in one of the presets, nudged once per simulation step
 * by the gravity it feels, and drawn each frame. If it drifts way off
 * screen we quietly switch it off to save work.
 *
 * Positions are kept in fine "pixel" units, not whole character cells, so
 * motion looks smooth — we only round to a cell when it's time to draw.
 * If we stored cell positions instead, slow stars would sit frozen for
 * many frames and fast ones would jump in chunks.
 *
 * prev_px/prev_py remember where the body was one step ago. The trail
 * renderer draws a line from there to the new spot, so a fast star leaves
 * a smooth glowing streak instead of a dotted line of skipped cells.
 *
 * The two flags do different jobs. 'active' off means "ignore me
 * completely" — that's how we drop runaways without shuffling arrays.
 * 'anchor' means "stay put no matter what" — gravity never moves an
 * anchor. The galaxy's central black hole is an anchor; the cluster and
 * binary scenes have none.
 *
 * How a body moves each step (symplectic Euler — a stable little
 * two-liner): first nudge its speed by the pull it feels, then slide it
 * along at that new speed. This particular order keeps the total energy
 * from drifting over the thousands of steps a long galaxy run takes, so
 * orbits stay tidy instead of slowly spiralling apart.
 *
 * Physics references: softened gravity and the symplectic step, Aarseth
 * (2003) §2.2 and §2.4; the galaxy's starting orbits, Binney & Tremaine
 * (2008) §3.1; the cluster's collapse-and-settle, the same book §8.5.4.
 */
typedef struct {
  float px, py;           /* where it is now, in pixel units */
  float prev_px, prev_py; /* where it was one step ago (for the trail) */
  float vx, vy;           /* how fast it's moving, pixel units per sim second */
  float mass;             /* its heft; 1.0 for an ordinary star,
                           * much larger for the central black hole */
  bool active;            /* false = switched off (ran away, or unused slot) */
  bool anchor;            /* true = pinned in place, gravity can't move it */
} Body;

/* The bodies themselves live inside the Scene struct down in §6 (it has
 * to be defined early so the quadtree code can reach it). The random-
 * number state below stays a plain file-scope variable so the little
 * rng helpers don't have to be handed a Scene every call. */

/* A tiny, fast, repeatable random-number generator (a linear congruential
 * generator). Same seed gives the same scene every time, which makes a
 * reset reproducible. */
static uint32_t g_rng = 12345u;
static uint32_t rng_next(void) {
  g_rng = g_rng * 1664525u + 1013904223u;
  return g_rng;
}
static float rng_f(void) {
  return (float)(rng_next() & 0x7FFFFFFFu) / (float)0x80000000u;
}
static float rng_range(float lo, float hi) { return lo + rng_f() * (hi - lo); }

/* ── §6  quadtree ── */

/*
 * QNode — one box in the tree that organises the stars by where they are.
 *
 * The whole point of the tree is speed. Checking every star against every
 * other is fine for a handful but brutal for hundreds. So we carve the
 * screen into nested boxes; each box remembers the combined weight and
 * the balance point of all the stars inside it. When a star is far from a
 * box, we can treat that whole box as one distant lump instead of looking
 * at its stars one by one. That's the Barnes & Hut (1986) idea, and it
 * turns a punishing amount of work into a manageable amount.
 *
 * Because the screen is 2-D, each box splits into four smaller boxes (a
 * quadtree). The same idea in 3-D would split into eight.
 *
 * What each field holds:
 *   x0, y0, x1, y1   The box's edges, in pixel units. Its width
 *                    (x1 - x0) is the "how big is this box" number used
 *                    when deciding whether it's far enough to lump.
 *   total_mass       Combined weight of every star inside this box and
 *                    its sub-boxes. Tallied up as stars are added.
 *   cx, cy           The balance point (centre of mass) of all those
 *                    stars — the spot where the lumped weight sits.
 *   child[4]         The four sub-boxes: NW=0, NE=1, SW=2, SE=3. A slot
 *                    of -1 means that quarter is empty. A box with all
 *                    four empty is a leaf (holds at most one star).
 *   body_idx         If this is a leaf holding exactly one star, this is
 *                    that star's index; otherwise -1. When a second star
 *                    lands in a one-star leaf, the box splits and the
 *                    resident star moves down into a sub-box.
 *   depth            How deep this box sits below the top (top = 0).
 *                    Capped at QT_MAX_DEPTH so two stars at the exact
 *                    same spot can't make the tree split forever.
 *
 * The "is this box far enough to lump?" test lives in qt_force: compare
 * the box's width to how far the star is from the box's balance point. If
 * the box looks small from where the star stands, lump it; otherwise look
 * inside at its four sub-boxes. The threshold THETA_DEF = 0.5 is the
 * paper's recommended balance of accuracy and speed.
 *
 * All the boxes are handed out from one fixed array (scene.pool); we never
 * call malloc while the simulation runs. The tree is thrown away and
 * rebuilt from scratch every step, which is simpler and faster than
 * patching an old tree as stars move.
 *
 * References: the original tree-code, Barnes & Hut (1986); the practical
 * accuracy/speed trade-off and the running-tally balance point, Aarseth
 * (2003) §6.2.
 */
typedef struct {
  float x0, y0, x1, y1; /* the box's edges, in pixel units */
  float total_mass;     /* combined weight of all stars inside this box */
  float cx, cy;         /* balance point of those stars */
  int child[4];         /* the four sub-boxes; NW=0 NE=1 SW=2 SE=3, -1=empty */
  int body_idx;         /* one-star leaf: that star's index; else -1 */
  int depth;            /* how deep below the top, capped at QT_MAX_DEPTH */
} QNode;

/*
 * Scene — everything the simulation needs to remember between one step and
 * the next, and between one drawn frame and the next.
 *
 * It's defined up here (not down in §7 with the scene_* functions) because
 * the quadtree code above already reaches into it. There is exactly one of
 * these, a single big file-scope variable; it's around 760 KB (the star
 * list, the box pool, and the glow grid), too large to pass around by
 * value, so everything just refers to the one `scene`.
 *
 * The fields fall into two groups, and the split is for the reader's sake:
 *
 *   Simulation fields are the real state of the physics. Only physics keys
 *   (pause, reset, pick a scene, add/remove stars, change gravity,
 *   fast-forward) change them.
 *
 *   Rendering fields only affect what's drawn. Changing the theme or
 *   toggling the overlay must leave every star in exactly the same place —
 *   if it didn't, display and physics would be tangled, which is the bug
 *   the split is meant to prevent. New flags should land on the right side.
 *
 * Two pieces of state deliberately live OUTSIDE this struct: g_rng (the
 * random-number state, kept separate so the small rng helpers stay simple)
 * and g_resize (the resize-signal flag, which has to be a special
 * signal-safe variable).
 */
typedef struct {
  /* ── simulation state ── */
  Body bodies[N_BODIES_MAX]; /* every star slot, switched on or off */
  int n_bodies;              /* how many slots are in use (+/- keys) */
  float G;                   /* strength of gravity (g/G keys) */
  bool paused;               /* true = freeze the physics */
  int preset;                /* which scene is loaded (0..2) */
  bool fastfwd;              /* true = run physics 4x faster */
  float sim_dt;              /* length of one physics step, in seconds */

  /* The box tree, thrown away and rebuilt from scratch every step. */
  QNode pool[NODE_POOL_MAX]; /* the fixed pile of boxes to hand out */
  int pool_top;              /* how many boxes are handed out so far */
  int qt_root;               /* which box is the top of the tree */

  /* ── rendering state ── */
  float bright[GRID_ROWS_MAX][GRID_COLS_MAX]; /* glow on each cell; stars and
                                               * their trails add to it, and
                                               * it fades a little each frame */
  float bright_max; /* a smoothed "brightest cell" used to scale colours */
  float v_max;      /* a smoothed "fastest star" used to pick speed colours */
  int theme;        /* which colour scheme is active (index into k_themes) */
  bool overlay;     /* true = draw the box tree on top */
  int frame_tick;   /* frames drawn so far; drives the black hole's pulse */
} Scene;

static Scene scene = {
    .n_bodies = N_BODIES_DEF,
    .G = G_DEF,
    .sim_dt = 1.0f / (float)SIM_HZ,
    .bright_max = 1.0f,
    .v_max = 1.0f,
};

static int qt_alloc(float x0, float y0, float x1, float y1, int depth) {
  if (scene.pool_top >= NODE_POOL_MAX)
    return -1;
  int idx = scene.pool_top++;
  QNode *n = &scene.pool[idx];
  n->x0 = x0;
  n->y0 = y0;
  n->x1 = x1;
  n->y1 = y1;
  n->total_mass = 0.0f;
  n->cx = 0.0f;
  n->cy = 0.0f;
  n->child[0] = n->child[1] = n->child[2] = n->child[3] = -1;
  n->body_idx = -1;
  n->depth = depth;
  return idx;
}

static int qt_quadrant(const QNode *n, float px, float py) {
  float mx = (n->x0 + n->x1) * 0.5f;
  float my = (n->y0 + n->y1) * 0.5f;
  return (px >= mx ? 1 : 0) | (py >= my ? 2 : 0);
}

static void qt_subdivide(QNode *n) {
  float mx = (n->x0 + n->x1) * 0.5f;
  float my = (n->y0 + n->y1) * 0.5f;
  n->child[0] = qt_alloc(n->x0, n->y0, mx, my, n->depth + 1);
  n->child[1] = qt_alloc(mx, n->y0, n->x1, my, n->depth + 1);
  n->child[2] = qt_alloc(n->x0, my, mx, n->y1, n->depth + 1);
  n->child[3] = qt_alloc(mx, my, n->x1, n->y1, n->depth + 1);
}

static void qt_insert(int ni, int bi) {
  if (ni < 0)
    return;
  QNode *n = &scene.pool[ni];
  Body *b = &scene.bodies[bi];

  /* Fold this star into the box's running weight and balance point. */
  float new_mass = n->total_mass + b->mass;
  n->cx = (n->cx * n->total_mass + b->px * b->mass) / new_mass;
  n->cy = (n->cy * n->total_mass + b->py * b->mass) / new_mass;
  n->total_mass = new_mass;

  if (n->body_idx < 0 && n->child[0] < 0) {
    /* Empty leaf: just drop the star here. */
    n->body_idx = bi;
    return;
  }
  if (n->depth >= QT_MAX_DEPTH)
    return;

  if (n->body_idx >= 0) {
    /* Leaf already has a star: split the box and push that star down. */
    int existing = n->body_idx;
    n->body_idx = -1;
    qt_subdivide(n);
    int q =
        qt_quadrant(n, scene.bodies[existing].px, scene.bodies[existing].py);
    qt_insert(n->child[q], existing);
  } else if (n->child[0] < 0) {
    qt_subdivide(n);
  }
  int q = qt_quadrant(n, b->px, b->py);
  qt_insert(n->child[q], bi);
}

static int qt_build(int cols, int rows) {
  scene.pool_top = 0;
  int root = qt_alloc(0.0f, 0.0f, (float)pw(cols), (float)ph(rows), 0);
  for (int i = 0; i < scene.n_bodies; i++)
    if (scene.bodies[i].active)
      qt_insert(root, i);
  return root;
}

static void qt_force(int ni, int bi, float *fx, float *fy) {
  if (ni < 0)
    return;
  QNode *n = &scene.pool[ni];
  if (n->total_mass == 0.0f)
    return;
  if (n->body_idx == bi)
    return; /* a star doesn't pull on itself */

  Body *b = &scene.bodies[bi];
  float dx = n->cx - b->px;
  float dy = n->cy - b->py;
  float d2 = dx * dx + dy * dy + SOFT2;
  float d = sqrtf(d2);
  float s = n->x1 - n->x0;

  if ((s / d) < THETA_DEF || n->child[0] < 0) {
    float inv = scene.G * n->total_mass / (d2 * d);
    *fx += inv * dx * b->mass;
    *fy += inv * dy * b->mass;
    return;
  }
  for (int c = 0; c < 4; c++)
    qt_force(n->child[c], bi, fx, fy);
}

/* Sketch the top few levels of the box tree so you can see how the
 * stars are being grouped. Only the first few depths, or it's clutter. */
static void qt_draw_overlay(int ni, int rows, int cols) {
  if (ni < 0)
    return;
  QNode *n = &scene.pool[ni];
  if (n->total_mass == 0.0f || n->depth > 3)
    return;

  int cx0 = px_to_cell_x(n->x0), cx1 = px_to_cell_x(n->x1);
  int cy0 = px_to_cell_y(n->y0), cy1 = px_to_cell_y(n->y1);
  int mxc = px_to_cell_x((n->x0 + n->x1) * 0.5f);
  int myc = px_to_cell_y((n->y0 + n->y1) * 0.5f);

  if ((cx1 - cx0) < 2 || (cy1 - cy0) < 2)
    return;

  attron(COLOR_PAIR(CP_TREE) | A_DIM);
  for (int c = cx0 + 1; c < cx1 && c < cols; c++)
    if (myc >= 0 && myc < rows)
      mvaddch(myc, c, '-');
  for (int r = cy0 + 1; r < cy1 && r < rows; r++)
    if (mxc >= 0 && mxc < cols)
      mvaddch(r, mxc, '|');
  if (myc >= 0 && myc < rows && mxc >= 0 && mxc < cols)
    mvaddch(myc, mxc, '+');
  attroff(COLOR_PAIR(CP_TREE) | A_DIM);

  for (int c = 0; c < 4; c++)
    qt_draw_overlay(n->child[c], rows, cols);
}

/* ── §7  scene — state that spans across ticks and frames ── *
 *
 * The Scene struct itself is up in §6 (the quadtree code needed it early).
 * Everything below — the three scene setups and the per-step / per-frame
 * work — operates on that one `scene`. */

/* ── the three scenes ── */

/*
 * Galaxy. A heavy black hole sits pinned at the centre, and every other
 * star starts on a circular orbit around it — inner stars circle faster
 * than outer ones, and that uneven spin naturally winds the disk into
 * spiral arms after a few turns.
 */

/* Drop a pinned, heavy body at (cx, cy). "Pinned" means gravity won't
 * budge it — see the anchor flag on Body. */
static void spawn_central_bh(int idx, float cx, float cy, float M) {
  scene.bodies[idx].px = cx;
  scene.bodies[idx].py = cy;
  scene.bodies[idx].vx = 0.0f;
  scene.bodies[idx].vy = 0.0f;
  scene.bodies[idx].mass = M;
  scene.bodies[idx].active = true;
  scene.bodies[idx].anchor = true;
}

/*
 * Pick a distance from the centre to drop a star, so the disk ends up
 * evenly speckled instead of bunched up in the middle. The catch: a thin
 * ring far out covers far more area than a ring of the same width near
 * the centre, so simply picking the distance at random would crowd the
 * core. Taking a square root spreads the stars out so every patch of the
 * disk gets its fair share. We also keep them a bit out from dead centre
 * so none spawn inside the black hole.
 */
static float sample_disk_radius_uniform_area(float R) {
  return R * (0.08f + 0.92f * sqrtf(rng_f()));
}

/*
 * Drop one star somewhere on the disk and give it just the right speed and
 * direction to circle the central mass — fast enough not to fall in, slow
 * enough not to fly off. The speed for a steady circle depends on how far
 * out the star is; it points sideways (across the line to the centre) so
 * the star goes round. A tiny ~6% nudge to each speed makes the orbits
 * slightly oval, so the disk looks alive rather than mechanically perfect.
 *
 * Starting orbits follow Binney & Tremaine (2008) §3.1.
 */
static void spawn_keplerian_body(int idx, float cx, float cy, float R,
                                 float M_bh) {
  float r = sample_disk_radius_uniform_area(R);
  float theta = rng_f() * 2.0f * (float)M_PI;

  float bx = cx + cosf(theta) * r;
  float by = cy + sinf(theta) * r;

  float v_kep = sqrtf(scene.G * M_bh / r);
  float v = v_kep * (1.0f + rng_range(-0.06f, 0.06f));

  /* Aim the velocity sideways to the centre line, so the star circles. */
  float tx = -(by - cy) / r;
  float ty = (bx - cx) / r;

  scene.bodies[idx].px = bx;
  scene.bodies[idx].py = by;
  scene.bodies[idx].vx = tx * v;
  scene.bodies[idx].vy = ty * v;
  scene.bodies[idx].mass = 1.0f;
  scene.bodies[idx].active = true;
  scene.bodies[idx].anchor = false;
}

static void preset_galaxy(int cols, int rows) {
  /* Centre the disk; size it to the shorter side of the screen. */
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;
  float half = (float)pw(cols);
  if (ph(rows) < pw(cols))
    half = (float)ph(rows);
  half *= 0.5f;
  float R = half * 0.75f;

  /* Make the black hole heavier when there are more stars, so the orbits
   * keep their pace as you add or remove stars with +/-. */
  float M_bh = (float)scene.n_bodies * BH_MASS_FACTOR;

  spawn_central_bh(0, cx, cy, M_bh);

  for (int i = 1; i < scene.n_bodies; i++)
    spawn_keplerian_body(i, cx, cy, R, M_bh);
}

/*
 * Cold collapse. A round cloud of stars sitting almost still. With nothing
 * holding it up, its own gravity yanks it inward into a dense knot, which
 * then overshoots and bounces, settling into a wobble. It all happens in a
 * few seconds, so it's fun to watch. Settling behaviour: Binney & Tremaine
 * (2008) §8.5.4.
 */
static void preset_cluster(int cols, int rows) {
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;
  /* Keep it compact: tighter cloud means stronger pull, faster collapse. */
  float R = (float)(pw(cols) < ph(rows) ? pw(cols) : ph(rows)) * 0.28f;

  /* A touch of spin so it doesn't collapse to a single dot. */
  float M_tot = (float)scene.n_bodies;
  float v_spin = sqrtf(scene.G * M_tot / R) * 0.12f;

  for (int i = 0; i < scene.n_bodies; i++) {
    float r = R * sqrtf(rng_f());
    float theta = rng_f() * 2.0f * (float)M_PI;
    float bx = cx + cosf(theta) * r;
    float by = cy + sinf(theta) * r;

    /* Point the spin sideways so the cloud rotates. */
    float nx = -(by - cy) / (r + 1.0f);
    float ny = (bx - cx) / (r + 1.0f);

    scene.bodies[i].px = bx;
    scene.bodies[i].py = by;
    scene.bodies[i].vx = nx * v_spin;
    scene.bodies[i].vy = ny * v_spin;
    scene.bodies[i].mass = 1.0f;
    scene.bodies[i].active = true;
    scene.bodies[i].anchor = false;
  }
}

/*
 * Two clusters, spinning opposite ways, drifting toward each other. As
 * they pass, each one's gravity tears long streamers off the other's
 * edges, and the two dense centres finally crash together in a burst.
 */
static void preset_binary(int cols, int rows) {
  float cx = (float)pw(cols) * 0.5f;
  float cy = (float)ph(rows) * 0.5f;
  float R = (float)(pw(cols) < ph(rows) ? pw(cols) : ph(rows)) * 0.18f;
  float sep = R * 2.4f;

  int half = scene.n_bodies / 2;

  /* Approach velocity: clusters meet in ~5 seconds at default SIM_HZ */
  float approach = sep / (5.0f * (float)SIM_HZ);

  /* Internal spin speed */
  float M_half = (float)half;
  float v_spin = sqrtf(scene.G * M_half / R) * 0.35f;

  for (int k = 0; k < 2; k++) {
    float ox = cx + (k == 0 ? -sep : sep);
    float oy = cy;
    float vx_drift = (k == 0 ? approach : -approach);
    float spin_dir = (k == 0 ? 1.0f : -1.0f); /* counter-rotate */

    int start = k * half;
    int end = (k == 1) ? scene.n_bodies : half;

    for (int i = start; i < end; i++) {
      float r = R * sqrtf(rng_f());
      float theta = rng_f() * 2.0f * (float)M_PI;
      float bx = ox + cosf(theta) * r;
      float by = oy + sinf(theta) * r;
      float nx = -(by - oy) / (r + 1.0f);
      float ny = (bx - ox) / (r + 1.0f);

      scene.bodies[i].px = bx;
      scene.bodies[i].py = by;
      scene.bodies[i].vx = vx_drift + spin_dir * nx * v_spin;
      scene.bodies[i].vy = spin_dir * ny * v_spin;
      scene.bodies[i].mass = 1.0f;
      scene.bodies[i].active = true;
      scene.bodies[i].anchor = false;
    }
  }
}

static void scene_reset(int cols, int rows) {
  memset(scene.bright, 0, sizeof(scene.bright));
  memset(scene.bodies, 0, sizeof(Body) * N_BODIES_MAX);
  scene.v_max = 1.0f;
  g_rng = 99991u;

  switch (scene.preset) {
  case 0:
    preset_galaxy(cols, rows);
    break;
  case 1:
    preset_cluster(cols, rows);
    break;
  case 2:
    preset_binary(cols, rows);
    break;
  }

  /* Initialise prev position to current so the first frame draws a
   * single point per body, not a streak from (0,0). */
  for (int i = 0; i < scene.n_bodies; i++) {
    scene.bodies[i].prev_px = scene.bodies[i].px;
    scene.bodies[i].prev_py = scene.bodies[i].py;
  }
}

/* ── scene_tick ── */

/* Build the grouping tree fresh each step. Starting over every time is
 * simpler and faster than nudging an old tree as the stars move. */
static void rebuild_force_tree(int cols, int rows) {
  scene.qt_root = qt_build(cols, rows);
}

/* Switch off any star that has wandered far off-screen, so we stop
 * spending work on it. We allow generous slack past the edges first,
 * since a star can swing way out and still loop back into view. */
static void mark_inactive_if_escaped(Body *b, float W, float H) {
  if (b->px < -W || b->px > 2.0f * W)
    b->active = false;
  if (b->py < -H * 2.0f || b->py > 3.0f * H)
    b->active = false;
}

/*
 * Move one star forward by a single step: first nudge its speed by the
 * pull it feels, then slide it along at that new speed. Doing it in that
 * order (speed first, then position) is the trick that keeps long galaxy
 * runs from slowly spiralling apart — see the Body note in §5. Along the
 * way we remember where it was (for the trail) and switch it off if it
 * has escaped. Aarseth (2003) §2.4.
 */
static void integrate_body_symplectic_euler(int i, float W, float H) {
  Body *b = &scene.bodies[i];
  if (!b->active || b->anchor)
    return;

  /* Remember where it was, so the trail can draw a line from there. */
  b->prev_px = b->px;
  b->prev_py = b->py;

  /* Total pull on this star, gathered from the grouping tree. */
  float fx = 0.0f, fy = 0.0f;
  qt_force(scene.qt_root, i, &fx, &fy);

  /* First nudge the speed by that pull... */
  float ax = fx / b->mass;
  float ay = fy / b->mass;
  b->vx += ax * scene.sim_dt;
  b->vy += ay * scene.sim_dt;

  /* ...then slide it along at the new speed. */
  b->px += b->vx * scene.sim_dt;
  b->py += b->vy * scene.sim_dt;

  mark_inactive_if_escaped(b, W, H);

  /* Keep track of the fastest star, which sets the colour scale. */
  float spd = sqrtf(b->vx * b->vx + b->vy * b->vy);
  if (spd > scene.v_max)
    scene.v_max = spd;
}

/* Let the "fastest star" figure ease back down over time, so colours
 * follow how things move right now instead of staying stuck on one brief
 * burst of speed that may never happen again. */
static void relax_speed_normalisation(void) {
  scene.v_max *= 0.9995f;
  if (scene.v_max < 0.1f)
    scene.v_max = 0.1f;
}

/* One step of the whole simulation. */
static void scene_tick(int cols, int rows) {
  /* 1. Rebuild the box tree that groups the stars. */
  rebuild_force_tree(cols, rows);

  /* 2. Move every live, unpinned star forward one step. */
  float W = (float)pw(cols);
  float H = (float)ph(rows);
  for (int i = 0; i < scene.n_bodies; i++)
    integrate_body_symplectic_euler(i, W, H);

  /* 3. Ease the colour scale back toward how things move now. */
  relax_speed_normalisation();
}

/* ── scene_draw ── */

/*
 * Smear glow along the line a star travelled this step, so a fast star
 * leaves a smooth streak instead of a dotted trail of skipped cells. It
 * walks the line cell by cell (the classic Bresenham line-stepping trick)
 * and adds a little brightness to each one. We stop short of the end cell
 * — the star itself lights that one — and cap the number of steps so a
 * wild jump (say, after a resize) can't paint a streak across the screen.
 */
static void streak_glow(float x0, float y0, float x1, float y1, float intensity,
                        int rows, int cols) {
  int r0 = px_to_cell_y(y0), c0 = px_to_cell_x(x0);
  int r1 = px_to_cell_y(y1), c1 = px_to_cell_x(x1);
  int dr = abs(r1 - r0), dc = abs(c1 - c0);
  int sr = -1;
  if (r0 < r1)
    sr = 1;
  int sc = -1;
  if (c0 < c1)
    sc = 1;
  int err = dr - dc;
  for (int i = 0; i < 32; i++) {
    if (r0 == r1 && c0 == c1)
      break; /* endpoint = body draws there */
    if (r0 >= 0 && r0 < rows && c0 >= 0 && c0 < cols)
      scene.bright[r0][c0] += intensity;
    int e2 = 2 * err;
    if (e2 > -dc) {
      err -= dc;
      r0 += sr;
    }
    if (e2 < dr) {
      err += dr;
      c0 += sc;
    }
  }
}

/*
 * Spread a soft glow around the black hole so it visibly pulls in light.
 * The glow is brightest dead centre and fades with distance. Terminal
 * cells are about twice as tall as they are wide, so we count sideways
 * distance less than up-down distance; otherwise the glow would look
 * squashed into an oval instead of a round halo. The pulse value makes
 * the whole halo breathe in and out.
 */
static void deposit_accretion_halo(int cr, int cc, int rows, int cols,
                                   float pulse) {
  for (int dr = -2; dr <= 2; dr++) {
    for (int dc = -4; dc <= 4; dc++) {
      if (dr == 0 && dc == 0)
        continue;
      int rr = cr + dr;
      int ccc = cc + dc;
      if (rr < 0 || rr >= rows || ccc < 0 || ccc >= cols)
        continue;
      float d2 = (float)(dr * dr) + (float)(dc * dc) * 0.25f;
      scene.bright[rr][ccc] += 3.0f * pulse / (1.0f + d2);
    }
  }
}

/*
 * Lay down this frame's glow. Every star lights up its own cell and
 * leaves a faint streak behind it; the black hole gets its round halo
 * instead. When this is done, the brightness grid holds the full picture,
 * ready to be scaled and painted.
 */
static void accumulate_glow_field(int cols, int rows) {
  float halo_pulse = 1.0f + 0.4f * sinf((float)scene.frame_tick * 0.18f);

  for (int i = 0; i < scene.n_bodies; i++) {
    Body *b = &scene.bodies[i];
    if (!b->active)
      continue;

    int cr = px_to_cell_y(b->py);
    int cc = px_to_cell_x(b->px);
    float deposit = 1.0f;
    if (b->mass > 1.0f)
      deposit = 4.0f; /* anchor brighter */

    if (cr >= 0 && cr < rows && cc >= 0 && cc < cols)
      scene.bright[cr][cc] += deposit;

    if (b->anchor) {
      deposit_accretion_halo(cr, cc, rows, cols, halo_pulse);
    } else {
      streak_glow(b->prev_px, b->prev_py, b->px, b->py, deposit * 0.5f, rows,
                  cols);
    }
  }
}

/*
 * Track the brightest cell so we can scale colours against it. We blend
 * the new reading gently into the old one rather than jumping to it, so
 * one fleeting hot spot — a near miss between two stars — doesn't wash
 * out the whole picture for the next second.
 */
static void update_brightness_norm(int cols, int rows) {
  float frame_max = 1.0f;
  for (int r = 0; r < rows; r++)
    for (int c = 0; c < cols; c++)
      if (scene.bright[r][c] > frame_max)
        frame_max = scene.bright[r][c];
  scene.bright_max = scene.bright_max * 0.85f + frame_max * 0.15f;
  if (scene.bright_max < 1.0f)
    scene.bright_max = 1.0f;
}

/*
 * Draw the glow grid to the screen, picking a denser character for
 * brighter cells, then fade every cell a touch so old trails slowly
 * dim away over the next few frames instead of vanishing at once.
 */
static void paint_glow_layer(int cols, int rows) {
  static const char k_glow[] = ".:*o@";
  float b_max = scene.bright_max;

  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      float b = scene.bright[r][c];
      scene.bright[r][c] *= DECAY;
      if (b < 0.4f)
        continue;

      float norm = b / b_max;
      int lvl = (int)(norm * 4.9f);
      if (lvl > 4)
        lvl = 4;

      attr_t attr = COLOR_PAIR(CP_L1 + lvl);
      if (lvl >= 3)
        attr |= A_BOLD;
      mvaddch(r, c, (chtype)k_glow[lvl] | attr);
    }
  }
}

/*
 * Draw the black hole as a little pulsing symbol. It uses the same beat
 * as its halo, so the core and the brackets around it swell and dim in
 * time with the glow breathing in and out.
 */
static void paint_anchor_pulse(int cr, int cc, int cols) {
  float ph = sinf((float)scene.frame_tick * 0.18f);

  chtype core_attr = COLOR_PAIR(CP_BH) | A_BOLD;
  chtype edge_attr = COLOR_PAIR(CP_BH);
  if (ph > 0.6f)
    edge_attr |= A_BOLD;
  else if (ph < -0.4f)
    edge_attr |= A_DIM;

  char core = '*';
  if (ph > 0.0f)
    core = '@';

  mvaddch(cr, cc, (chtype)core | core_attr);
  if (cc > 0)
    mvaddch(cr, cc - 1, (chtype)'(' | edge_attr);
  if (cc < cols - 1)
    mvaddch(cr, cc + 1, (chtype)')' | edge_attr);
}

/*
 * Draw one ordinary star: faster stars get hotter colours and bolder
 * symbols. We also alternate between two look-alike characters so a
 * crowd of stars moving at the same speed reads as a textured cloud
 * rather than a flat, repeating pattern.
 */
static void paint_field_body(int i, const Body *b, int cr, int cc) {
  float spd = sqrtf(b->vx * b->vx + b->vy * b->vy);
  float norm = spd / scene.v_max;

  int pair;
  chtype ch;
  int variant = i & 1;
  if (norm > 0.80f) {
    pair = CP_L5;
    ch = '*';
    if (variant)
      ch = '#';
  } else if (norm > 0.55f) {
    pair = CP_L4;
    ch = '+';
    if (variant)
      ch = 'x';
  } else if (norm > 0.30f) {
    pair = CP_L3;
    ch = 'o';
    if (variant)
      ch = '0';
  } else if (norm > 0.10f) {
    pair = CP_L2;
    ch = '.';
    if (variant)
      ch = ',';
  } else {
    pair = CP_L1;
    ch = ',';
    if (variant)
      ch = '`';
  }
  mvaddch(cr, cc, ch | COLOR_PAIR(pair) | A_BOLD);
}

/*
 * Draw every star on top of the glow. The black hole gets its own
 * pulsing symbol; all the others are coloured by how fast they move.
 */
static void paint_bodies(int cols, int rows) {
  for (int i = 0; i < scene.n_bodies; i++) {
    const Body *b = &scene.bodies[i];
    if (!b->active)
      continue;

    int cr = px_to_cell_y(b->py);
    int cc = px_to_cell_x(b->px);
    if (cr < 1 || cr >= rows - 1 || cc < 0 || cc >= cols)
      continue;

    if (b->anchor)
      paint_anchor_pulse(cr, cc, cols);
    else
      paint_field_body(i, b, cr, cc);
  }
}

/* One drawn frame, built up in five passes. */
static void scene_draw(int cols, int rows, float alpha) {
  (void)alpha;
  scene.frame_tick++;

  /* 1. Lay down this frame's glow: stars, trails, and the halo. */
  accumulate_glow_field(cols, rows);

  /* 2. Find the brightest cell to scale colours against. */
  update_brightness_norm(cols, rows);

  /* 3. Paint the glow, fading old trails as we go. */
  paint_glow_layer(cols, rows);

  /* 4. Paint the stars on top of the glow. */
  paint_bodies(cols, rows);

  /* 5. If asked, sketch the box tree over everything. */
  if (scene.overlay)
    qt_draw_overlay(scene.qt_root, rows, cols);
}

/* ── §8  screen / HUD ── */

/*
 * Two-bar HUD per CLAUDE.md convention:
 *   row 0 right    — bright yellow + bold: live status (N, G, theme, fps, ...)
 *   row 1 left     — bright yellow:        preset name + description
 *   row rows-1     — bright cyan + bold:   key hints
 *
 * Paused indicator is folded into the top status string so it sits with
 * the other live state (rather than a separate inline tag).
 */
static void hud_draw(int cols, int rows, double fps) {
  /* Count active (non-anchor) bodies. */
  int active = 0;
  for (int i = 0; i < scene.n_bodies; i++)
    if (scene.bodies[i].active && !scene.bodies[i].anchor)
      active++;

  /* Row 0 — right-aligned live status. */
  char top[160];
  const char *state = "running";
  if (scene.paused)
    state = "PAUSED ";
  else if (scene.fastfwd)
    state = "4x     ";
  snprintf(
      top, sizeof top, " Barnes-Hut  N=%d(%d)  G=%.0f  theme:%s  %s  %.0f fps ",
      scene.n_bodies, active, scene.G, k_themes[scene.theme].name, state, fps);
  int top_len = (int)strlen(top);
  int top_col = cols - top_len;
  if (top_col < 0)
    top_col = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, top_col, top, cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* Row 1 — left-aligned preset name and tagline. */
  static const char *k_desc[N_PRESETS] = {
      "Galaxy: BH disk — watch spiral arms form",
      "Cluster: cold collapse — core bounce incoming",
      "Binary merger — tidal streams + chaotic ejections",
  };
  char sub[120];
  snprintf(sub, sizeof sub, " [%d] %s ", scene.preset + 1,
           k_desc[scene.preset]);
  attron(COLOR_PAIR(CP_HUD));
  mvaddnstr(1, 0, sub, cols);
  attroff(COLOR_PAIR(CP_HUD));

  /* Row rows-1 — bottom hint bar. */
  const char *hint_full = " q:quit  p:pause  r:reset  1-3:preset  t/T:theme  "
                          "o:overlay  f:4x  +/-:bodies  g/G:grav ";
  const char *hint_short =
      " q:quit  p:pause  r:reset  1-3:preset  f:4x  t:theme ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= cols - 1)
    hint = hint_short;
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(rows - 1, 0, hint, cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §9  app ── */

static volatile sig_atomic_t g_resize = 0;
static void on_sigwinch(int sig) {
  (void)sig;
  g_resize = 1;
}

int main(void) {
  signal(SIGWINCH, on_sigwinch);

  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);

  color_init(scene.theme);

  int rows, cols;
  getmaxyx(stdscr, rows, cols);
  scene_reset(cols, rows);

  int64_t render_ns = NS_PER_SEC / RENDER_FPS;
  int64_t sim_ns = NS_PER_SEC / SIM_HZ;
  int64_t t_last_sim = clock_ns();
  int64_t sim_acc = 0;
  int64_t fps_t0 = clock_ns();
  int fps_frames = 0;
  double fps_disp = 0.0;

  for (;;) {
    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, rows, cols);
      scene_reset(cols, rows);
      color_init(scene.theme);
    }

    int ch;
    while ((ch = getch()) != ERR) {
      switch (ch) {
      case 'q':
      case 27:
        endwin();
        return 0;

      case 'p':
      case ' ':
        scene.paused = !scene.paused;
        break;

      case 'r':
        scene_reset(cols, rows);
        break;

      case '1':
      case '2':
      case '3':
        scene.preset = ch - '1';
        scene_reset(cols, rows);
        break;

      case 't':
        scene.theme = (scene.theme + 1) % N_THEMES;
        color_init(scene.theme);
        break;

      case 'T':
        scene.theme = (scene.theme + N_THEMES - 1) % N_THEMES;
        color_init(scene.theme);
        break;

      case 'o':
        scene.overlay = !scene.overlay;
        break;

      case 'f':
        scene.fastfwd = !scene.fastfwd;
        break;

      case '+':
      case '=':
        scene.n_bodies += N_BODIES_STEP;
        if (scene.n_bodies > N_BODIES_MAX)
          scene.n_bodies = N_BODIES_MAX;
        scene_reset(cols, rows);
        break;

      case '-':
        scene.n_bodies -= N_BODIES_STEP;
        if (scene.n_bodies < N_BODIES_STEP)
          scene.n_bodies = N_BODIES_STEP;
        scene_reset(cols, rows);
        break;

      case 'g':
        scene.G += G_STEP;
        if (scene.G > G_MAX)
          scene.G = G_MAX;
        break;

      case 'G':
        scene.G -= G_STEP;
        if (scene.G < G_MIN)
          scene.G = G_MIN;
        break;
      }
    }

    /* Physics accumulator */
    int64_t now = clock_ns();
    sim_acc += now - t_last_sim;
    t_last_sim = now;

    if (!scene.paused) {
      /* Fast-forward: allow 4× the normal budget */
      int64_t cap = scene.fastfwd ? sim_ns * 16 : sim_ns * 4;
      if (sim_acc > cap)
        sim_acc = cap;
      while (sim_acc >= sim_ns) {
        scene_tick(cols, rows);
        sim_acc -= sim_ns;
      }
    }

    float alpha = (float)sim_acc / (float)sim_ns;

    erase();
    scene_draw(cols, rows, alpha);
    hud_draw(cols, rows, fps_disp);
    wnoutrefresh(stdscr);
    doupdate();

    fps_frames++;
    int64_t fps_elapsed = clock_ns() - fps_t0;
    if (fps_elapsed >= (int64_t)FPS_UPDATE_MS * NS_PER_MS) {
      fps_disp = (double)fps_frames / ((double)fps_elapsed / 1e9);
      fps_frames = 0;
      fps_t0 = clock_ns();
    }

    int64_t t_next = now + render_ns;
    clock_sleep_ns(t_next - clock_ns());
  }
}
