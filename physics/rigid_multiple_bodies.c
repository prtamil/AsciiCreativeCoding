/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * rigid_multiple_bodies.c — boxes and balls fall, stack, and bump into each
 * other in 2D. Pick one of four ready-made scenes (brick wall, beam pile,
 * tower, pyramid) and knock it down by firing balls at it with SPACE. The
 * bodies only slide around — they never spin. Sister file: rigid_body.c.
 *
 * Sources for the physics and rendering ideas, cited inline as [n]:
 *   [1] Baumgarte (1972) — the gentle push that separates overlapping bodies.
 *   [2] Catto / Box2D (https://box2d.org) — the solver, the tuning numbers,
 *       and the sleep system.
 *   [3] Mirtich (1996) — the bounce-impulse formula.
 *   [4] Ericson, Real-Time Collision Detection (2005) — the box-overlap test
 *       and the friction model.
 *   [5] Goldstein et al., Classical Mechanics (2002) — what bounce and
 *       friction mean physically.
 *   [6] Padala, NCURSES Programming HOWTO — colour pairs, flicker-free draw.
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

/* ── world bounds ── */

/* Biggest terminal we pre-size the framebuffer for. We just reserve room for
 * this many cells up front; real drawing is clipped to the actual window, so
 * a generous size costs memory, not speed. */
#define ROWS_MAX 128
#define COLS_MAX 512

/* How many bodies can exist at once. Collision-checking compares every body
 * against every other, several times per tick, so this grows cost fast — 32
 * fills the screen visually long before it stresses the CPU. */
#define MAX_BODIES 32

/* How many colour themes exist (§3) and how many ready-made scenes exist
 * (§7). Each must match its array length; the cycle keys wrap around it. */
#define N_THEMES 10
#define N_PRESETS 4

/* Strip of space kept clear at the top of the world for the status bar, so
 * no body ever draws into the top row. The bottom row is reserved the same
 * way (see WH() in §4). Measured in physics pixels — one screen row = 2. */
#define HUD_TOP_PX 2.0f

/* ── physics ── */

/* How hard gravity pulls bodies down each tick. Bigger = snappier drops,
 * smaller = a floaty, lunar feel. */
#define GRAVITY 0.05f

/* Bounciness, 0 to 1: 0 is a dead thud, 1 bounces forever. 0.35 feels like a
 * rubber ball on concrete. Set it too high and stacks blow apart, because
 * each bounce adds more energy than drag can soak up. REST_DEF is where we
 * start; REST_STEP is the step size for nudging it. (Goldstein [5].) */
#define REST_DEF 0.35f
#define REST_STEP 0.05f

/* How grippy surfaces are, 0 to 1. Higher = bodies stop sliding sooner. 0.35
 * is about rubber on wood — coasts a moment, then stops. (Ericson [4].) */
#define FRICTION 0.35f
/* REST_STEP is here for tuning bounciness, but no key is wired to it yet. */

/* Fraction of speed a body keeps each tick — a gentle air-drag so things
 * don't slide forever. 0.991 a tick works out to losing about 16% of speed
 * over a second. Lower values feel underwater. */
#define DAMPING 0.991f

/* Speed limit (pixels per tick). Without it, a very fast body could jump
 * clean through a thin wall in one tick and miss the collision check
 * entirely. Capping speed keeps every move small enough for the solver to
 * catch. */
#define MAX_SPEED 22.0f

/* ── solver ── */

/* How many times per tick we re-run the whole collision pass. Stacks need
 * repeated passes: fixing the top contact shifts the bodies below, which
 * then need fixing again. About 10 gives stable stacks without wasted work.
 * Each pass is the expensive every-pair loop, so cost scales directly with
 * this number. (Catto [2].) */
#define SOLVER_ITERS 10

/* How much of an overlap we push out per pass — half of it. Pushing it all
 * out at once tends to overshoot and pop bodies out of stacks; nibbling away
 * half each pass settles smoothly. (Baumgarte [1].) */
#define BAUMGARTE 0.50f

/* A tiny overlap we just ignore. Floating-point math always leaves
 * hair-thin overlaps; without this dead-zone the solver would twitch at them
 * every frame. 0.05 px is well under one screen pixel. (Catto [2].) */
#define SLOP 0.05f

/* Below this closing speed we treat a contact as "resting" and skip the
 * bounce, so a body sitting on the floor doesn't jitter forever from tiny
 * leftover wobble. About a quarter-pixel per frame — invisible, but above
 * the solver's noise. */
#define REST_THRESH 0.20f

/* ── sleep system (Catto / Box2D [2]) ── */
/* Bodies that sit still for a while are "put to sleep" — frozen and skipped
 * by the physics, which is the big speed win since the every-pair loop is
 * the costly part. A nudge wakes them again. */

/* A body slower than this counts as still and starts banking quiet frames.
 * Set below REST_THRESH so a body still micro-bouncing can't accidentally
 * fall asleep. */
#define SLEEP_VEL 0.07f

/* How many quiet frames in a row before a body actually sleeps. 30 frames is
 * about 1.5 seconds — long enough that a briefly-still body in flight won't
 * freeze mid-air, short enough that a settled stack sleeps while you watch. */
#define SLEEP_FRAMES 30

/* How big a nudge it takes to wake a sleeping body. Pegged near SLOP, so any
 * real interaction wakes it but normal solver jitter doesn't. */
#define WAKE_IMP 0.05f

/* ── body sizes (pixels; screen cells are about 1 wide : 2 tall) ── */

/* Cube half-width and half-height. Picked so a cube looks roughly square on
 * screen given that cells are about twice as tall as they are wide. */
#define CUBE_HW 7.0f
#define CUBE_HH 5.0f

/* Sphere radius — the same in width and height so the drawn circle looks
 * round, not squashed. The collision box is taller than wide (hh = 2·SPH_R)
 * to wrap that round shape in pixel space; see the Body hw/hh docs in §4. */
#define SPH_R 4.0f

/* How heavy bodies are for their size (mass = this × box area). Tuned so a
 * cube and a sphere weigh about the same, so a sphere doesn't launch a cube
 * across the screen on first touch. (Goldstein [5].) */
#define DENSITY 0.008f

/* Brick size for the Brick Wall scene. Small and light — about 5× lighter
 * than the projectile ball, which is what makes the wall scatter so
 * dramatically on a hit. */
#define BRICK_HW 3.0f
#define BRICK_HH 2.0f

/* Beam size for the Beam Stack scene. Long and thin, like lumber. Heavier
 * than bricks, so the pile topples as a unit instead of scattering. */
#define BEAM_HW 8.0f
#define BEAM_HH 1.5f

/* How fast a fired ball moves sideways. Just under the speed cap so it isn't
 * immediately clipped, and fast enough that drag doesn't drain it before it
 * crosses the world and hits the structure. */
#define PROJECTILE_VX 18.0f

/* ── timing ── */

/* How many physics ticks per second. The drawing runs as fast as it can and
 * just shows the latest tick; 20 ticks a second is plenty for this chunky
 * cell-grid look. */
#define SIM_FPS 20

/* Nanoseconds in a second, for the clock helpers. The LL suffix keeps the
 * 64-bit math from overflowing. */
#define NS_PER_S 1000000000LL

/* ── §2 clock ── */

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

/* ── §3 color / theme ── */

/* ncurses paints with numbered "colour pairs". We hand out the slots like so:
 *   1..6     body colours — set by the current theme; bodies cycle through them
 *   CP_FLOOR the floor line — a fixed dim grey
 *   CP_HUD   top status bar — fixed bright yellow
 *   CP_HINT  bottom key bar — fixed bright cyan
 * The floor and the two bars keep fixed colours so they stay readable no
 * matter which theme the bodies are using. */
#define CP_FLOOR 9
#define CP_HUD 10
#define CP_HINT 11

/* Theme — one colour scheme for the bodies. Six colours, going from dim to
 * bright. Bodies cycle through them as they spawn, so a stack of bodies shows
 * a brightness gradient you can count by eye even when the hues are close.
 *
 * A theme is purely how things LOOK, never how they move. Pressing t to swap
 * themes mid-bounce changes nothing about the physics — that's why the theme
 * lives as a render setting, not part of the simulation.
 *
 * The six colours are 256-colour terminal indices, kept bright enough to stay
 * visible on a black background (see documentation/COLOR.md for which indices
 * vanish). On an 8-colour terminal we fall back to the basic colours. */
typedef struct {
  /* Short label shown in the status bar. Keep it to about 7 characters so the
   * whole status line still fits on an 80-column terminal. */
  const char *name;

  /* The six body colours, dimmest first, brightest last. The first body
   * spawned uses ramp[0], the sixth uses ramp[5], the seventh wraps back to
   * ramp[0], and so on. */
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

/* True if the terminal supports 256 colours. Decided once at startup and never
 * changes, so it sits here rather than inside the scene. */
static bool g_256;

/* Loads a theme's colours into ncurses. Safe to call any time; it also resets
 * the floor and status-bar colours each call so they can never get clobbered.
 * The theme index can be any integer — it's wrapped into range here. */
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

/* ── §4 body + scene ── */

/* Kind — is this body a box or a ball? Only the drawing code cares; the physics
 * treats both the same way (everything is a rectangle to the collision math, so
 * a ball collides as its bounding box). To add a new shape: add it here, add a
 * drawing branch in draw_body, and give it sensible size fields when spawned. */
typedef enum { KIND_CUBE = 0, KIND_SPHERE } Kind;

/* Body — one falling thing: a box or a ball. Both shapes share this one struct
 * because the collision math only ever works with rectangles; the round look of
 * a ball is just how it's drawn. One struct also means one physics path instead
 * of two near-identical copies.
 *
 * Bodies slide but never spin — there's no rotation here. Skipping spin keeps
 * the whole physics step short while still showing how stacking and bouncing
 * work. Adding real spin would need a much heavier solver (see Catto [2]).
 *
 * It's a plain value type with no pointers and no memory to free; the whole
 * pool of bodies lives in one fixed array in the scene.
 *
 * The fields come in five groups: the shape (set once when it spawns), where it
 * is and how fast it's moving (updated every tick), its weight, its colour, and
 * its sleep bookkeeping. */
typedef struct {
  /* ── shape: set once when the body spawns, never changed afterward ── */

  /* Box or ball — only the drawing code looks at this. */
  Kind kind;

  /* Half-width and half-height of the body's box, in physics pixels (one screen
   * row is 2 pixels tall). A box uses CUBE_HW / CUBE_HH. A ball uses its radius
   * for the width but TWICE the radius for the height — because screen cells are
   * about twice as tall as they are wide, so a round-looking ball needs a taller
   * box to wrap it. An early version forgot this and balls passed through each
   * other vertically until they were half-overlapping. */
  float hw, hh;

  /* ── position and motion: updated every tick by the physics step (§6) ── */

  /* Centre of the body, in physics pixels. y grows DOWNWARD like screen rows do,
   * so gravity pushes toward larger y and the floor is the bottom. */
  float x, y;

  /* Velocity, in pixels per tick. Capped at MAX_SPEED each step so a fast body
   * can't leap clean through a thin wall between two collision checks. */
  float vx, vy;

  /* ── weight ── */

  /* mass — how heavy the body is, worked out from its size when it spawns. Kept
   * around mainly for the inverse below; not read by the physics directly. */
  float mass;

  /* imass — one divided by mass. The solver multiplies by this constantly, so
   * we store it once instead of dividing every time. A heavier body has a
   * smaller imass and so gets pushed around less in a collision. */
  float imass;

  /* ── colour ── */

  /* Which of the six theme colour slots this body uses (1 to 6). Chosen when it
   * spawns; swapping themes keeps the same slot number but repaints it, so the
   * body just changes colour without any extra work. */
  int cp;

  /* ── sleep system: stop simulating bodies that have settled ──
   * A body that barely moves for a while gets "put to sleep" — frozen and
   * skipped by the physics. Since checking every pair of bodies is the slow
   * part, sleeping settled stacks is the big speed win. A nudge wakes it. */

  /* How many frames in a row this body has been nearly still. Counts up while
   * it's quiet, resets on any real movement, and locks once it's asleep. Each
   * body tracks its own count so they sleep independently. */
  int sleep_cnt;

  /* True once the body is asleep: gravity and movement skip it, and a pair of
   * two sleeping bodies is skipped entirely. Cleared the moment a collision
   * pushes or shoves it hard enough (see body_wake). */
  bool sleeping;
} Body;

/* Scene — everything about the world that has to survive from one frame to the
 * next: the bodies, the physics knobs, and the current look. There's a single
 * shared instance, g_scene.
 *
 * The fields fall into two groups, kept apart on purpose:
 *
 *   Simulation — anything that changes where the bodies are or how they move.
 *   The physics step reads and writes these.
 *
 *   Rendering — colour only. Changing these must never disturb the bodies, so
 *   that swapping themes mid-bounce is guaranteed not to nudge the simulation.
 *
 * The split is for whoever reads this next: when you add a field, ask "does
 * this change how the bodies move?" If yes it's simulation, if no it's
 * rendering. Putting a physics flag in the rendering group would quietly let
 * the display change the simulation — exactly the bug this separation avoids.
 *
 * A few things deliberately live OUTSIDE here: the screen size (g_rows/g_cols,
 * which the resize handler owns), the quit/resize signal flags (which must be
 * file-scope for signal safety), the 256-colour capability (g_256, decided
 * once), and the fps counters (locals in main, nobody else needs them). */
typedef struct {
  /* ── Simulation: feeds the physics step ── */

  /* The active bodies and how many there are. New bodies are appended; deleting
   * just drops the last one. Capacity is MAX_BODIES; once full, no more spawn.
   * Checking every pair of bodies is the slow part, which is why MAX_BODIES is
   * kept modest. */
  Body b[MAX_BODIES];
  int nb;

  /* Running totals of how many cubes and spheres have ever spawned. Used only
   * to keep the body colours cycling through the theme even as bodies come and
   * go. Reset on a fresh scene. */
  int ncubes, nsphs;

  /* Bounciness, 0 to 1. Higher bounces more. Kept below 1 because anything that
   * bouncy adds energy on every hit and blows stacks apart. */
  float rest;

  /* Gravity on/off. When off, bodies stop being pulled down but keep coasting
   * and gradually slow to a stop from drag. */
  bool grav;

  /* When paused, the physics step is skipped entirely, so everything freezes in
   * place and resumes exactly where it left off. */
  bool paused;

  /* How many physics steps have run since the last reset. Spare counter for any
   * future "every so often" effect; the physics itself doesn't use it. */
  long tick;

  /* Random-number state for spawn jitter. Re-seeded from the clock at startup
   * and on every reset, so each run looks a little different. */
  uint32_t rng;

  /* Which ready-made scene is loaded (brick wall, beams, tower, pyramid).
   * Switching it rebuilds the world, which changes the bodies — that's why it's
   * a simulation field, not a rendering one. */
  int preset;

  /* ── Rendering: colour only, never touches the physics ── */

  /* Which colour theme is active. Cycled with t / T, which also repaint the
   * body colours. */
  int theme;
} Scene;

/* The one and only scene. We set the few values that shouldn't start at zero:
 * a sensible bounciness, gravity on, and a non-zero random seed in case the
 * random helper runs before the clock-based reseed. Everything else starts
 * zeroed, which means an empty world until a scene is built. */
static Scene g_scene = {
    .rest = REST_DEF,
    .grav = true,
    .rng = 0xDEAD1234u,
};

/* Current terminal size in cells. Owned by the main loop's resize handling. */
static int g_rows, g_cols;

static inline float WW(void) { return (float)g_cols; }

/* Height of the play area in physics pixels. We hold back three screen rows:
 * the top status bar, the floor line, and the bottom key bar — so bodies only
 * live in the rows between. */
static inline float WH(void) { return (float)((g_rows - 2) * 2); }

static inline int pcol(float x) { return (int)(x + 0.5f); }
static inline int prow(float y) { return (int)(y * 0.5f + 0.5f); }

/* A cheap random number between 0 and 1 for spawn jitter. Good enough to make
 * runs differ, not meant for anything that needs real randomness. */
static float rng_f(void) {
  uint32_t r = g_scene.rng;
  r ^= r << 13;
  r ^= r >> 17;
  r ^= r << 5;
  g_scene.rng = r;
  return (float)(r >> 8) / (float)(1u << 24);
}

static void body_init_mass(Body *b) {
  float area = 4.f * b->hw * b->hh; /* bigger bodies weigh more */
  b->mass = area * DENSITY;
  b->imass = 1.f / b->mass;
}

static void body_wake(Body *b) {
  b->sleeping = false;
  b->sleep_cnt = 0;
}

/* ── §5 framebuffer ── */

/* We draw bodies into this off-screen grid of characters first, then copy it to
 * the terminal in one pass. Drawing here keeps the physics-to-screen mapping in
 * one place and avoids flicker. */
static char g_fb[ROWS_MAX][COLS_MAX];
static int g_fcp[ROWS_MAX][COLS_MAX];

static void fb_clear(void) {
  memset(g_fb, 0, sizeof g_fb);
  memset(g_fcp, 0, sizeof g_fcp);
}
static void fb_put(int r, int c, char ch, int cp) {
  /* Skip the reserved rows — the top status bar, the floor, the bottom key bar
   * are all painted on their own, so bodies never overwrite them. */
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
  /* Copy the off-screen grid to the terminal, play area only — the same rows
   * fb_put allows. */
  for (int r = 1; r < g_rows - 2; r++)
    for (int c = 0; c < g_cols; c++) {
      if (!g_fb[r][c])
        continue;
      attron(COLOR_PAIR(g_fcp[r][c]) | A_BOLD);
      mvaddch(r, c, (chtype)g_fb[r][c]);
      attroff(COLOR_PAIR(g_fcp[r][c]) | A_BOLD);
    }
}

/* ── §6 physics ── */

/* One physics tick runs five steps in order, each its own function below:
 *   1. apply gravity      — pull awake bodies downward
 *   2. integrate motion   — move bodies, apply drag, cap the top speed
 *   3. solve contacts      — push apart and bounce every overlapping pair,
 *                            repeated a few times so stacks settle
 *   4. enforce boundaries — keep bodies inside the floor, walls, and ceiling
 *   5. update sleep        — freeze bodies that have settled
 * The contact and boundary steps break down further into the small helpers
 * just below, so each piece of the math reads on its own. */

/* Contact — the result of checking whether two boxes overlap. It answers two
 * questions: which way to push them apart, and by how much.
 *
 * Because the boxes are axis-aligned, the shortest way out is always straight
 * left, right, up, or down — so (nx, ny) points purely along one axis, from the
 * first body toward the second. `depth` is how far they overlap along that
 * direction. We only need this push direction and distance, not the exact touch
 * point, because nothing here spins. */
typedef struct {
  float nx, ny; /* which way to push them apart, points from a toward b */
  float depth;  /* how deeply they overlap along that direction          */
  bool overlapping;
} Contact;

/* Do two boxes overlap, and if so, which way do we shove them apart? We measure
 * the overlap on each axis; if either axis has a gap, they miss. When they do
 * overlap, the cheapest way out is along whichever axis they overlap LESS, so
 * we push along that one. (Ericson [4, §4.2].) */
static Contact aabb_contact(const Body *a, const Body *b) {
  Contact c = {0};
  float ox = (a->hw + b->hw) - fabsf(b->x - a->x);
  float oy = (a->hh + b->hh) - fabsf(b->y - a->y);
  if (ox <= 0.f || oy <= 0.f)
    return c; /* a gap on one axis means no contact */

  c.overlapping = true;
  if (ox < oy) { /* less overlap sideways, so push sideways */
    c.nx = (b->x > a->x) ? 1.f : -1.f;
    c.ny = 0.f;
    c.depth = ox;
  } else { /* less overlap vertically, so push up or down */
    c.nx = 0.f;
    c.ny = (b->y > a->y) ? 1.f : -1.f;
    c.depth = oy;
  }
  return c;
}

/* How fast the two bodies are closing in along the push direction. Positive
 * means they're moving into each other, which is the only time we bounce. */
static inline float relative_normal_velocity(const Body *a, const Body *b,
                                             float nx, float ny) {
  return (a->vx - b->vx) * nx + (a->vy - b->vy) * ny;
}

/* Gently nudge two overlapping bodies apart. We only fix part of the overlap
 * each time (and ignore a hair-thin one) so stacks settle smoothly instead of
 * popping. The heavier body moves less. This runs whether or not they're moving
 * toward each other, so two bodies that spawn overlapping still separate. A big
 * enough nudge also wakes a sleeping body. (Baumgarte [1].) */
static void baumgarte_correct_pair(Body *a, Body *b, const Contact *c) {
  float denom = a->imass + b->imass;
  if (denom < 1e-12f)
    return; /* nothing can move (both effectively immovable) */

  float corr = fmaxf(c->depth - SLOP, 0.f) * BAUMGARTE / denom;
  if (corr <= 0.f)
    return; /* overlap too tiny to bother with */

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

/* The bounce: turn the speed two bodies are closing at into a kick that pushes
 * them apart, sharing it by weight. We only bounce above a small speed; below
 * it we treat the contact as resting, so a body sitting on another doesn't
 * buzz forever from tiny leftover wobble. Returns the kick strength so friction
 * can be scaled to it. The caller already checked they're approaching.
 * (Mirtich [3].) */
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

/* Friction: slow down any sliding the two bodies are doing across each other.
 * It tries to stop the sliding entirely, but it can only grip so hard — a
 * fraction of how firmly they're pressed together (the bounce strength). We
 * measure the sliding AFTER the bounce above so friction acts on the corrected
 * motion. (Ericson [4].) */
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

/* Handle one pair of bodies: check if they overlap, push them apart, and if
 * they're moving into each other, bounce them and apply friction. This is the
 * inner loop run for every pair, several times per tick. */
static void col_bodies(Body *a, Body *b) {
  Contact c = aabb_contact(a, b);
  if (!c.overlapping)
    return;

  baumgarte_correct_pair(a, b, &c);

  float vn = relative_normal_velocity(a, b, c.nx, c.ny);
  if (vn <= 0.f)
    return; /* drifting apart — nothing to bounce */

  float jn = apply_contact_impulse(a, b, c.nx, c.ny, vn);
  apply_coulomb_friction(a, b, c.nx, c.ny, jn);
}

/* The floor and the three walls. These are immovable, so we just snap a body
 * back inside the edge and flip the part of its speed heading into the wall.
 * Only the floor adds sliding friction — it's the one surface bodies rest and
 * slide on; gripping the walls too would make brief side bumps feel sticky. */

/* Keep a body above the floor and bounce it off, then slow any sliding. */
static void resolve_floor(Body *b) {
  float wh = WH();
  if (b->y + b->hh <= wh)
    return;
  b->y = wh - b->hh; /* sit exactly on the floor */

  if (b->vy <= 0.f)
    return; /* already heading up, leave it */
  float e_eff = (b->vy > REST_THRESH) ? g_scene.rest : 0.f;
  b->vy = -b->vy * e_eff;
  b->vx *= (1.f - FRICTION * (1.f + e_eff)); /* drag from sliding on the floor */
  if (fabsf(b->vx) < SLEEP_VEL)
    b->vx = 0.f; /* stop barely-there drift so it can settle */
}

/* Keep a body right of the left wall and bounce it off. */
static void resolve_left_wall(Body *b) {
  if (b->x - b->hw >= 0.f)
    return;
  b->x = b->hw;
  if (b->vx >= 0.f)
    return; /* already heading right, leave it */
  float e_eff = (fabsf(b->vx) > REST_THRESH) ? g_scene.rest : 0.f;
  b->vx = -b->vx * e_eff;
}

/* Keep a body left of the right wall and bounce it off. */
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

/* Keep a body below the ceiling and bounce it off. The ceiling sits a couple
 * pixels down, not at the very top, so the status bar's row stays clear. */
static void resolve_ceiling(Body *b) {
  if (b->y - b->hh >= HUD_TOP_PX)
    return;
  b->y = b->hh + HUD_TOP_PX;
  if (b->vy >= 0.f)
    return;
  float e_eff = (fabsf(b->vy) > REST_THRESH) ? g_scene.rest : 0.f;
  b->vy = -b->vy * e_eff;
}

/* ── the five steps of one physics tick ── */

/* Pull every awake body downward a little. Sleeping bodies are left alone so
 * they don't immediately wake back up, and the whole thing is skippable when
 * the user turns gravity off. */
static void apply_gravity_to_awake(void) {
  if (!g_scene.grav)
    return;
  for (int i = 0; i < g_scene.nb; i++)
    if (!g_scene.b[i].sleeping)
      g_scene.b[i].vy += GRAVITY;
}

/* Move every awake body by its speed, bleed off a touch of speed as drag, then
 * cap it so nothing ever moves so fast it tunnels through a wall in one step.
 * The cap goes last so it's the true top speed. */
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

/* Check every pair of bodies for collisions and resolve them, then do it again
 * a few times. The repeats matter for stacks: fixing the top contact shifts the
 * bodies below it, which then need fixing too. Pairs where both bodies are
 * asleep are skipped, which is where the sleep system pays off. (Catto [2].) */
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

/* Clamp every body inside the floor, ceiling, and walls — sleeping ones too.
 * We include sleepers because the gentle separating pushes in a tall stack can
 * otherwise creep a sleeper through the floor over many ticks; snapping it back
 * keeps things honest. */
static void enforce_world_boundaries(void) {
  for (int i = 0; i < g_scene.nb; i++) {
    Body *b = &g_scene.b[i];
    resolve_floor(b);
    resolve_left_wall(b);
    resolve_right_wall(b);
    resolve_ceiling(b);
  }
}

/* Track how long each body has been nearly still, and once it's been quiet long
 * enough, freeze it. Any real movement resets the count. Waking back up is
 * handled elsewhere, by the collision code when something bumps it. (Catto [2].) */
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

/* One full physics tick: run the five steps in order. */
static void scene_step(void) {
  apply_gravity_to_awake();
  integrate_motion();
  solve_contact_constraints();
  enforce_world_boundaries();
  update_sleep_state();
  g_scene.tick++;
}

/* ── §7 scene management ── */

/* Would this would-be body land on top of one that's already there? Used when
 * spawning to avoid dropping a new body inside an existing one. */
static bool aabb_overlaps_any(const Body *c) {
  for (int i = 0; i < g_scene.nb; i++) {
    const Body *b = &g_scene.b[i];
    if ((c->hw + b->hw) > fabsf(c->x - b->x) &&
        (c->hh + b->hh) > fabsf(c->y - b->y))
      return true;
  }
  return false;
}

/* Fill in a body's shape, size, colour, and weight. Its position and speed are
 * still zero afterward — those get set when it's placed. */
static void body_init_shape(Body *b, Kind kind, float hw, float hh, int cp) {
  b->kind = kind;
  b->hw = hw;
  b->hh = hh;
  b->cp = cp;
  body_init_mass(b);
}

/* Find an open spot near the top to drop a body into. It tries a few random x
 * positions across the middle of the screen and takes the first that doesn't
 * land on another body. Returns false if every try was blocked, in which case
 * the caller skips the spawn. */
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

/* Drop a new cube in near the top with a little sideways drift. Does nothing if
 * the world is already full or there's no open spot. */
static bool scene_add_cube(void) {
  if (g_scene.nb >= MAX_BODIES)
    return false;

  Body b = {0};
  body_init_shape(&b, KIND_CUBE, CUBE_HW, CUBE_HH, 1 + (g_scene.ncubes % 6));
  if (!try_place_at_top(&b, 8))
    return false;

  b.vx = (rng_f() - 0.5f) * 2.f; /* small random sideways nudge */
  g_scene.b[g_scene.nb++] = b;
  g_scene.ncubes++;
  return true;
}

/* Same as scene_add_cube but spawns a ball. */
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

/* The ready-made starting layouts. Each one builds a structure for you to knock
 * down. To add one: list it in this enum, bump N_PRESETS in §1, add its name
 * below, write a preset_<name>() that places the bodies, and add it to the
 * switch in scene_init. The physics knobs (bounciness, gravity) stay where the
 * user left them across layout changes. */
typedef enum {
  PRESET_BRICK_WALL = 0, /* grid of small light bricks          */
  PRESET_BEAMS,          /* pile of long horizontal beams        */
  PRESET_TOWER,          /* narrow vertical column of cubes      */
  PRESET_PYRAMID,        /* triangular stack of cubes            */
} Preset;

static const char *k_preset_names[N_PRESETS] = {
    "Brick Wall",
    "Beam Stack",
    "Tower",
    "Pyramid",
};

/* Place one fully-specified body into the world. Used by the layout builders.
 * Does nothing if the world is full (better to skip a body than overrun the
 * array). Keeps the spawn counters ticking so colours stay in step. */
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

/* Launch a ball from the left, flying right toward the structure. Bound to
 * SPACE; press it again and again to send more.
 *
 * The spawn height matters: we start the ball a full diameter ABOVE the floor,
 * not skimming it. A ball that starts on the floor hits floor-friction right
 * away and loses about a third of its speed every tick, so it stops within a
 * few ticks and never reaches the target. Starting it higher lets it sail
 * across the screen before gravity pulls it down — and it lands partway up the
 * structure, which looks better too. */
static void fire_projectile(void) {
  float wh = WH();
  scene_place_body(KIND_SPHERE, 2.f * SPH_R, wh - 4.f * SPH_R, SPH_R,
                   2.f * SPH_R, 1 + (g_scene.nsphs % 6), PROJECTILE_VX, 0.f,
                   false);
}

/* A wall of small light bricks on the right, stacked up from the floor. They
 * start asleep; your first ball wakes them and scatters the wall, because the
 * ball is about five times heavier than a brick. The tiny gap between bricks
 * just keeps the start looking clean. */
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

/* A pile of long, thin beams stacked like lumber. They're heavier than bricks,
 * so instead of scattering they topple over together when you hit the bottom of
 * the stack. */
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

/* A tall, narrow column of cubes. Hit the bottom cube with a ball and the whole
 * tower topples over — the classic kick-the-foundation test. Kept to six cubes
 * so it fits under the ceiling on a short terminal. */
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

/* A pyramid of cubes, four wide at the base up to one on top. It should sit
 * still on its own — a good test that stacking is stable. Knock out the base
 * row with a ball and the upper layers tumble as their supports vanish. */
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

/* Clear the world and build the chosen layout fresh. Called at startup, on
 * reset, when switching layouts, and after a resize. */
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

/* ── §8 draw ── */

/* Draw one body: a ring of 'O' for a ball, an outlined rectangle for a box,
 * with a '+' at its centre. The ball uses its taller collision box for the
 * vertical size, which cancels out the cells being taller than wide and makes
 * it look round on screen. */
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

/* Two status bars: live info along the top row, the key list along the bottom.
 * Both stay bright and bold so they read over any animation. */
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

  /* draw the floor first so bodies sitting on it show on top */
  int floor_row = prow(WH());
  attron(COLOR_PAIR(CP_FLOOR));
  for (int c = 0; c < g_cols; c++)
    mvaddch(floor_row, c, '=');
  attroff(COLOR_PAIR(CP_FLOOR));

  /* draw all the bodies into the off-screen grid, then copy it out */
  for (int i = 0; i < g_scene.nb; i++)
    draw_body(&g_scene.b[i]);
  fb_flush();

  /* the status bars go last so they always sit on top of everything else */
  draw_hud_top(fps);
  draw_hud_bottom();
}

/* ── §9 screen ── */

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
  use_default_colors(); /* lets -1 mean "the terminal's own background" */
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

/* ── §10 app ── */

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

    /* count frames and refresh the fps reading once a second */
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
