/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * rigid_body.c — falling cubes and spheres that bounce, stack, and settle
 * in the terminal.  No spin: bodies only slide and fall, never rotate.
 *
 * The physics is the classic game-engine recipe: every body is wrapped in
 * a box, overlapping boxes get pushed apart and bounced, and bodies that
 * go quiet long enough are put to sleep to save work.
 *
 * Worth reading alongside the code: Box2D and Erin Catto's GDC talks
 * (box2d.org) for the two-pass solver and the sleep trick; Baumgarte 1972
 * for the gentle "nudge apart" position fix; Ericson, *Real-Time Collision
 * Detection* §4.2/§5.4 for the box-overlap and friction math.
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

/* All distances are in "pixels": each terminal cell is 1 px wide and 2 px
 * tall, since cells are about twice as tall as they are wide. */

/* How big a terminal we plan for.  We reserve fixed-size buffers up front;
 * anything bigger than the real window is simply never drawn. */
#define ROWS_MAX 128
#define COLS_MAX 512

/* Most bodies allowed on screen at once.  Collision checking compares every
 * body against every other, so this stays modest. */
#define MAX_BODIES 32

/* How many colour themes live in k_themes[] below.  Must match the array. */
#define N_THEMES 10

/* Empty space kept at the very top so the status bar has a clear row. */
#define HUD_TOP_PX 2.0f

/* ── physics ── */

/* Downward pull added to a body's speed each step.  Higher = snappier
 * drops, lower = a floaty, lunar feel. */
#define GRAVITY 0.05f

/* Bounciness when the demo starts.  0 = no bounce (lands dead), 1 = perfect
 * bounce.  0.35 is roughly a rubber ball on concrete.  Above ~0.8, stacks
 * fall apart because each hit adds energy faster than damping removes it. */
#define REST_DEF 0.35f

/* How much one e/E keypress changes the bounciness. */
#define REST_STEP 0.05f

/* How grippy surfaces are (friction).  0.35 is roughly rubber on wood:
 * bodies coast a little, then stop, instead of sliding forever or sticking. */
#define FRICTION 0.35f

/* Speed kept each step — a gentle drag so nothing slides on forever.
 * Over one second this slows a body to about 83% of its speed. */
#define DAMPING 0.991f

/* Top speed.  Capped so a fast body can't jump clean through a thin wall
 * in a single step and miss the collision check entirely. */
#define MAX_SPEED 22.0f

/* ── solver (how hard it works to keep bodies apart) ── */

/* How many times per step we re-check and re-fix every overlap.  More
 * passes = firmer stacks but more CPU; 10 keeps a few-tall stack steady. */
#define SOLVER_ITERS 10

/* How much of an overlap we nudge away each pass.  Half each pass settles
 * smoothly; correcting the full amount tends to overshoot and pop bodies
 * out of stacks. */
#define BAUMGARTE 0.50f

/* Tiny overlaps we ignore.  Without this slack, sub-pixel rounding noise
 * would make bodies jitter forever. */
#define SLOP 0.05f

/* Below this approach speed we turn bounce off, so a resting body doesn't
 * buzz from the solver's tiny leftover wobble. */
#define REST_THRESH 0.20f

/* ── sleep (freeze bodies that have gone still, to save work) ── */

/* A body slower than this counts as "still" for one frame.  Kept below
 * REST_THRESH so a body that's still micro-bouncing can't fall asleep. */
#define SLEEP_VEL 0.07f

/* How many still frames in a row before a body freezes.  At 20 steps/sec
 * that's 1.5 s — long enough not to freeze something mid-flight, short
 * enough that a settled pile actually goes to sleep. */
#define SLEEP_FRAMES 30

/* A push or bounce bigger than this wakes a sleeping body back up; smaller
 * nudges (normal solver noise) leave it asleep. */
#define WAKE_IMP 0.05f

/* ── body sizes (pixels; remember a cell is 1 wide, 2 tall) ── */

/* Half-width and half-height of a cube, picked so it looks roughly square
 * on screen given the tall cells. */
#define CUBE_HW 7.0f
#define CUBE_HH 5.0f

/* Sphere radius, equal in width and height so the drawn circle looks round.
 * Its collision box uses hh = 2*SPH_R to match — see the Body hw/hh note. */
#define SPH_R 4.0f

/* How heavy bodies are for their size.  Tuned so a cube and a sphere weigh
 * about the same, so a sphere can't punt a cube across the screen. */
#define DENSITY 0.008f

/* ── timing ── */

/* Physics steps per second.  Drawing runs as fast as it can on top of this;
 * 20 steps/sec looks smooth at this chunky resolution. */
#define SIM_FPS 20

/* Nanoseconds in a second.  LL-suffixed so the 64-bit clock math doesn't
 * overflow. */
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

/* Colour-pair slots 1..6 hold the current theme's body colours; bodies
 * pick one as they spawn.  These three are fixed chrome, the same on every
 * theme, so the floor line and HUD stay readable no matter the palette. */
#define CP_FLOOR 9
#define CP_HUD 10
#define CP_HINT 11

/* A Theme is just the look of the bodies — six colours, nothing about how
 * they move.  Switching theme is purely cosmetic: positions, speeds, and
 * sleep state stay exactly the same, only the colour changes.  That's why
 * the theme index lives in the rendering half of Scene, not the physics.
 *
 * The six colours run dim to bright on purpose: bodies bunched in a stack
 * stay countable by eye because neighbours differ in brightness even when
 * their hue is close.  Every colour is kept bright enough to show up on a
 * black terminal (see the brightness rules in documentation/COLOR.md). */
typedef struct {
  /* Short label shown in the status bar.  Keep it under ~7 chars so the
   * whole status line still fits on an 80-column terminal. */
  const char *name;

  /* Six body colours, ordered dim (ramp[0]) to bright (ramp[5]).  The
   * first body spawned uses ramp[0], the next ramp[1], and so on, wrapping
   * back to ramp[0] after six. */
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

/* Fixed chrome colours — the same on every theme. */
#define CHROME_FLOOR_256 240 /* dim grey      */
#define CHROME_HUD_256 226   /* bright yellow */
#define CHROME_HINT_256 51   /* bright cyan   */

/* True when the terminal supports 256 colours.  Set once at startup and
 * never changes, so it lives here rather than in the scene. */
static bool g_256;

/* Loads a theme's colours into ncurses.  Safe to call any time; it also
 * re-sets the chrome colours each call so a stray bug can't leave them
 * wrong for more than one frame.  ti can be any int — it wraps safely. */
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

/* Which shape a body is.  Only the drawing code cares — the physics treats
 * cubes and spheres identically, since both collide as boxes.  To add a new
 * shape: add it here, add a draw case, and set sensible hw/hh when spawning. */
typedef enum { KIND_CUBE = 0, KIND_SPHERE } Kind;

/* A Body is one falling thing — a cube or a sphere.  Both share this one
 * struct because collisions only ever use the body's bounding box, so the
 * physics code never needs to know which shape it is.
 *
 * Bodies don't spin in this demo: they only slide and fall.  Adding spin
 * would mean tracking rotation and a much heavier contact solver, so we
 * leave it out and keep the physics small.
 *
 * It's a plain value type with no pointers — bodies just sit in the scene's
 * array, and nothing is allocated while the demo runs.
 *
 * The fields come in groups: shape (set once when spawned), motion (updated
 * every step), mass, colour, and sleep bookkeeping. */
typedef struct {
  /* ── shape: set once when the body spawns, then left alone ── */

  /* Cube or sphere.  Only the drawing code reads this. */
  Kind kind;

  /* Half the collision box's width and height, in pixels.  A cube uses its
   * real half-sizes.  A sphere uses hw = radius but hh = 2*radius, because
   * cells are twice as tall as wide: a circle that looks round on screen is
   * actually twice as tall in pixels, and the box has to match or vertical
   * sphere hits get missed until the bodies are already half inside. */
  float hw, hh;

  /* ── motion: updated every step ── */

  /* Centre of the body, in pixels.  y grows downward (screen-style), so
   * gravity pulls toward larger y and the floor is at the bottom. */
  float x, y;

  /* Velocity, in pixels per step.  Capped at MAX_SPEED so a body can't skip
   * through a wall between steps. */
  float vx, vy;

  /* ── mass ── */

  /* Mass, from box area times DENSITY.  Using the box area (not the true
   * disc area for a sphere) is a deliberate simplification; since every
   * body measures mass the same way, collisions still behave sensibly. */
  float mass;

  /* One over the mass.  The collision math always multiplies by this, so we
   * store it once to skip a divide in the inner loop.  A value of 0 would
   * mean "immovable" — the math already handles that, though nothing here
   * spawns such a body. */
  float imass;

  /* ── colour ── */

  /* Which body colour-pair slot (1..6) this body draws in, chosen at spawn
   * so successive bodies cycle through the theme.  Survives theme changes
   * because theme_apply just rewrites these same slots with new colours. */
  int cp;

  /* ── sleep ── */

  /* How many steps in a row this body has been nearly still.  Once it
   * reaches SLEEP_FRAMES the body freezes.  Per-body so each sleeps on its
   * own schedule. */
  int sleep_cnt;

  /* True once the body has frozen.  Frozen bodies are skipped by gravity,
   * movement, and any collision where both bodies are asleep — that skip is
   * the whole point, since checking is the expensive part.  A big enough
   * push or bounce wakes it again (see body_wake). */
  bool sleeping;
} Body;

/* Scene holds everything that lives between frames: the bodies and the
 * knobs the user can turn.  There's a single instance, g_scene.
 *
 * The fields fall into two groups, and the split is a promise: the physics
 * group is anything that changes where bodies are or how they move; the
 * rendering group is purely how things look.  Pressing a theme key must
 * never touch the physics group — that's what lets you recolour mid-bounce
 * without disturbing the simulation.  When adding a field, ask which group
 * it belongs to: does it change what the bodies do, or only how they look? */
typedef struct {
  /* ── physics state ── */

  /* The active bodies, b[0..nb-1].  Spawning appends; deleting just drops
   * the last one.  Capacity is MAX_BODIES; spawns are refused when full. */
  Body b[MAX_BODIES];
  int nb;

  /* How many cubes / spheres have ever spawned, used only to pick the next
   * body's colour so successive bodies cycle through the theme.  Kept apart
   * from nb so deleting and respawning still steps through the palette. */
  int ncubes, nsphs;

  /* Bounciness, 0.05 to 0.95.  Scales every bounce (body-body, floor,
   * walls).  Held below 1 because higher adds energy and blows up stacks. */
  float rest;

  /* Gravity on/off.  When off, bodies keep their motion but stop being
   * pulled down and slowly coast to a stop. */
  bool grav;

  /* When true, the simulation is frozen and unpausing resumes exactly where
   * it left off. */
  bool paused;

  /* Step counter, available for any "every N steps" effect.  Not used by
   * the physics itself. */
  long tick;

  /* Random-number state for spawn jitter.  Re-seeded from the clock at boot
   * and on reset so each run differs. */
  uint32_t rng;

  /* ── rendering state (cosmetic only) ── */

  /* Which theme is active, an index into k_themes[].  Cycled by t / T. */
  int theme;
} Scene;

/* The one and only scene.  Sensible starting values: a bit bouncy, gravity
 * on, and a fixed RNG seed so it's valid even before scene_init reseeds it
 * from the clock.  Everything else starts zeroed (empty world). */
static Scene g_scene = {
    .rest = REST_DEF,
    .grav = true,
    .rng = 0xDEAD1234u,
};

/* Current terminal size.  Tracked by the main loop on resize, kept out of
 * Scene so the simulation doesn't need to care about the window. */
static int g_rows, g_cols;

static inline float WW(void) { return (float)g_cols; }

/* Height of the play area, in pixels.  The top row is the status bar, the
 * bottom two rows are the floor line and the key hints, so bodies live in
 * the rows between. */
static inline float WH(void) { return (float)((g_rows - 2) * 2); }

static inline int pcol(float x) { return (int)(x + 0.5f); }
static inline int prow(float y) { return (int)(y * 0.5f + 0.5f); }

/* A fast, cheap random float in [0, 1).  Fine for scattering spawns; not for
 * anything that needs real randomness. */
static float rng_f(void) {
  uint32_t r = g_scene.rng;
  r ^= r << 13;
  r ^= r >> 17;
  r ^= r << 5;
  g_scene.rng = r;
  return (float)(r >> 8) / (float)(1u << 24);
}

static void body_init_mass(Body *b) {
  float area = 4.f * b->hw * b->hh; /* box area */
  b->mass = area * DENSITY;
  b->imass = 1.f / b->mass;
}

static void body_wake(Body *b) {
  b->sleeping = false;
  b->sleep_cnt = 0;
}

/* ── §5 framebuffer ── */

/* We draw bodies into these character + colour grids first, then copy the
 * grids to the screen in one pass.  This keeps the drawing math separate
 * from ncurses and lets every cell share one bounds check. */

static char g_fb[ROWS_MAX][COLS_MAX];
static int g_fcp[ROWS_MAX][COLS_MAX];

static void fb_clear(void) {
  memset(g_fb, 0, sizeof g_fb);
  memset(g_fcp, 0, sizeof g_fcp);
}
static void fb_put(int r, int c, char ch, int cp) {
  /* Skip the reserved rows: the top status bar, the floor line, and the
   * bottom hints are all drawn separately. */
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
  /* Copy only the play-area rows to the screen; the floor and HUDs are the
   * caller's job.  Same row range fb_put allows. */
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

/* One simulation step does five things in order: add gravity, move the
 * bodies, push apart and bounce any that overlap, keep everyone inside the
 * floor and walls, and put still bodies to sleep.  Each gets its own helper
 * below, and the overlap-and-bounce step breaks down further into the small
 * pieces that follow. */

/* The result of checking two bodies for overlap.  When they overlap,
 * (nx, ny) points the shortest way to push them apart — for boxes that's
 * always straight up/down or left/right, so one of the two is zero — and
 * depth is how far they're overlapping along that direction.  We only need
 * direction and depth, not the exact touch point, since bodies don't spin. */
typedef struct {
  float nx, ny; /* shortest push-apart direction, points from a to b */
  float depth;  /* how deep the overlap is along that direction      */
  bool overlapping;
} Contact;

/* Checks whether two boxes overlap and, if so, finds the shortest way to
 * push them apart.  On each axis the overlap is "how close their centres are
 * vs. how wide they both are"; if either axis isn't overlapping they're not
 * touching.  When both overlap, the smaller of the two is the shortest way
 * out, and we point the push from a toward b. */
static Contact aabb_contact(const Body *a, const Body *b) {
  Contact c = {0};
  float ox = (a->hw + b->hw) - fabsf(b->x - a->x);
  float oy = (a->hh + b->hh) - fabsf(b->y - a->y);
  if (ox <= 0.f || oy <= 0.f)
    return c; /* not touching */

  c.overlapping = true;
  if (ox < oy) { /* shorter to push sideways */
    c.nx = (b->x > a->x) ? 1.f : -1.f;
    c.ny = 0.f;
    c.depth = ox;
  } else { /* shorter to push up/down */
    c.nx = 0.f;
    c.ny = (b->y > a->y) ? 1.f : -1.f;
    c.depth = oy;
  }
  return c;
}

/* How fast the two bodies are closing along the push direction.  Positive
 * means they're moving into each other, which is the only time we bounce. */
static inline float relative_normal_velocity(const Body *a, const Body *b,
                                             float nx, float ny) {
  return (a->vx - b->vx) * nx + (a->vy - b->vy) * ny;
}

/* Step one of fixing an overlap: gently slide the two bodies apart.  We only
 * move a fraction of the overlap each call (the rest gets fixed on later
 * passes), and a heavier body moves less than a lighter one.  This runs even
 * when bodies are drifting apart, so a pair that spawns overlapping still
 * separates.  Small overlaps (within SLOP) are left alone to avoid jitter,
 * and a big enough shove wakes a sleeping body. */
static void baumgarte_correct_pair(Body *a, Body *b, const Contact *c) {
  float denom = a->imass + b->imass;
  if (denom < 1e-12f)
    return; /* both immovable */

  float corr = fmaxf(c->depth - SLOP, 0.f) * BAUMGARTE / denom;
  if (corr <= 0.f)
    return; /* overlap too small to bother with */

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

/* Step two: the bounce.  Knocks the closing speed back out, scaled by the
 * bounciness, and shares it between the two bodies by weight.  When the
 * approach is barely a wobble we drop the bounce entirely so a resting body
 * doesn't buzz forever.  Returns the bounce strength so friction can size
 * itself against it.  The caller has already checked the bodies are closing. */
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

/* Step three: friction.  Slows the sideways sliding along the contact, but
 * never by more than the bounce strength times the friction setting — so a
 * gentle contact grips a little, a hard one grips more.  Works on the speed
 * left after the bounce, so the two effects stack cleanly. */
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

/* Handles one pair of bodies: if they overlap, push them apart, then (only
 * if they're moving into each other) bounce and apply friction.  The
 * push-apart always runs; the bounce and friction don't if the bodies are
 * already drifting apart. */
static void col_bodies(Body *a, Body *b) {
  Contact c = aabb_contact(a, b);
  if (!c.overlapping)
    return;

  baumgarte_correct_pair(a, b, &c);

  float vn = relative_normal_velocity(a, b, c.nx, c.ny);
  if (vn <= 0.f)
    return; /* drifting apart — no bounce needed */

  float jn = apply_contact_impulse(a, b, c.nx, c.ny, vn);
  apply_coulomb_friction(a, b, c.nx, c.ny, jn);
}

/* The floor and three walls.  Unlike body-body contacts, these are solid and
 * immovable, so a body that crosses one is snapped right back to the edge and
 * its speed into the surface is reflected — no gradual nudging needed.  Only
 * the floor adds sliding friction, since that's the only surface bodies rest
 * and slide on; friction on the walls would feel sticky on a glancing touch. */

/* Floor: keep the body above it, bounce, and rub off some sideways speed. */
static void resolve_floor(Body *b) {
  float wh = WH();
  if (b->y + b->hh <= wh)
    return;
  b->y = wh - b->hh; /* snap to the floor */

  if (b->vy <= 0.f)
    return; /* already heading up */
  float e_eff = (b->vy > REST_THRESH) ? g_scene.rest : 0.f;
  b->vy = -b->vy * e_eff;
  b->vx *= (1.f - FRICTION * (1.f + e_eff)); /* sliding friction */
  if (fabsf(b->vx) < SLEEP_VEL)
    b->vx = 0.f; /* stop the last bit of crawl */
}

/* Left wall: keep the body to its right and bounce. */
static void resolve_left_wall(Body *b) {
  if (b->x - b->hw >= 0.f)
    return;
  b->x = b->hw;
  if (b->vx >= 0.f)
    return; /* already heading right */
  float e_eff = (fabsf(b->vx) > REST_THRESH) ? g_scene.rest : 0.f;
  b->vx = -b->vx * e_eff;
}

/* Right wall: mirror image of the left. */
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

/* Ceiling: sits a little below the very top so the status bar's row stays
 * clear; bounces a body that hits it back down. */
static void resolve_ceiling(Body *b) {
  if (b->y - b->hh >= HUD_TOP_PX)
    return;
  b->y = b->hh + HUD_TOP_PX;
  if (b->vy >= 0.f)
    return;
  float e_eff = (fabsf(b->vy) > REST_THRESH) ? g_scene.rest : 0.f;
  b->vy = -b->vy * e_eff;
}

/* ── the five steps of one tick ── */

/* Pull every awake body downward.  Sleeping bodies are skipped so they stay
 * frozen, and the whole thing is off when the user toggles gravity. */
static void apply_gravity_to_awake(void) {
  if (!g_scene.grav)
    return;
  for (int i = 0; i < g_scene.nb; i++)
    if (!g_scene.b[i].sleeping)
      g_scene.b[i].vy += GRAVITY;
}

/* Move every awake body by its speed, bleed off a little to drag, then cap
 * the speed so nothing moves far enough in one step to skip through a wall. */
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

/* Check and fix every pair of bodies, several times over.  Repeating is what
 * lets stacks settle: fixing one contact shifts the bodies around it, so they
 * all need re-checking.  Pairs where both bodies are asleep are skipped, and
 * that's where the sleep system pays off, since this is the costliest loop. */
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

/* Keep every body inside the floor and walls — including sleeping ones,
 * because the gentle push-apart in a tall stack can slowly creep a sleeper
 * through the floor otherwise, and the snap keeps things honest. */
static void enforce_world_boundaries(void) {
  for (int i = 0; i < g_scene.nb; i++) {
    Body *b = &g_scene.b[i];
    resolve_floor(b);
    resolve_left_wall(b);
    resolve_right_wall(b);
    resolve_ceiling(b);
  }
}

/* Count how long each body has been nearly still, and freeze it once it's
 * been still long enough.  Any real motion resets the count.  Waking back up
 * happens elsewhere, whenever a body gets a solid push or bounce. */
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

/* One simulation tick: the five steps, in order. */
static void scene_step(void) {
  apply_gravity_to_awake();
  integrate_motion();
  solve_contact_constraints();
  enforce_world_boundaries();
  update_sleep_state();
  g_scene.tick++;
}

/* ── §7 scene management ── */

/* Would a body placed here land on top of an existing one?  Used to find a
 * clear spot when spawning. */
static bool aabb_overlaps_any(const Body *c) {
  for (int i = 0; i < g_scene.nb; i++) {
    const Body *b = &g_scene.b[i];
    if ((c->hw + b->hw) > fabsf(c->x - b->x) &&
        (c->hh + b->hh) > fabsf(c->y - b->y))
      return true;
  }
  return false;
}

/* Fill in a body's shape, colour, and mass.  Its position and speed are set
 * separately, when it's placed. */
static void body_init_shape(Body *b, Kind kind, float hw, float hh, int cp) {
  b->kind = kind;
  b->hw = hw;
  b->hh = hh;
  b->cp = cp;
  body_init_mass(b);
}

/* Try to drop the body in near the top at a random x, retrying a few times
 * until it finds a spot that doesn't land on another body.  Returns false if
 * every try was blocked, so the caller can skip the spawn. */
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

/* Drop a new cube in near the top with a small random sideways nudge.  Does
 * nothing if the scene is full or no clear spot is found. */
static bool scene_add_cube(void) {
  if (g_scene.nb >= MAX_BODIES)
    return false;

  Body b = {0};
  body_init_shape(&b, KIND_CUBE, CUBE_HW, CUBE_HH, 1 + (g_scene.ncubes % 6));
  if (!try_place_at_top(&b, 8))
    return false;

  b.vx = (rng_f() - 0.5f) * 2.f; /* small sideways nudge */
  g_scene.b[g_scene.nb++] = b;
  g_scene.ncubes++;
  return true;
}

/* Same as scene_add_cube, but spawns a sphere (with its taller collision box
 * and its own colour counter). */
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

/* Set up the opening scene (and reset back to it): a resting cube on the
 * floor with a sphere falling toward it.  Called at startup and on reset. */
static void scene_init(void) {
  g_scene.nb = g_scene.ncubes = g_scene.nsphs = 0;
  g_scene.tick = 0;
  g_scene.rng = (uint32_t)time(NULL) ^ 0xDEAD1234u;

  float ww = WW(), wh = WH();

  /* a cube parked on the floor, asleep so it stays put */
  Body c = {0};
  body_init_shape(&c, KIND_CUBE, CUBE_HW, CUBE_HH, 1);
  c.x = ww * 0.50f;
  c.y = wh - CUBE_HH;
  c.sleeping = true;
  g_scene.b[g_scene.nb++] = c;
  g_scene.ncubes++;

  /* a sphere up top, about to drop onto the cube */
  Body s = {0};
  body_init_shape(&s, KIND_SPHERE, SPH_R, 2.f * SPH_R, 4);
  s.x = ww * 0.50f;
  s.y = s.hh + HUD_TOP_PX;
  g_scene.b[g_scene.nb++] = s;
  g_scene.nsphs++;
}

/* ── §8 draw ── */

/* Draws one body into the framebuffer: a sphere as a ring of 'O' around its
 * centre, a cube as a box of '#', each with a '+' marking the middle.  The
 * sphere uses its (wider-than-tall) collision box for the radii, which is
 * exactly what makes it come out round on screen. */
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

/* The status line across the top: body counts and the current settings. */
static void draw_hud_top(int fps) {
  int nc = 0, ns = 0;
  for (int i = 0; i < g_scene.nb; i++) {
    if (g_scene.b[i].kind == KIND_CUBE)
      nc++;
    else
      ns++;
  }
  char buf[200];
  snprintf(
      buf, sizeof buf,
      " cubes:%d  spheres:%d/%d  rest:%.2f  grav:%s  theme:%s  %s  %d fps ", nc,
      ns, MAX_BODIES, g_scene.rest, g_scene.grav ? "on" : "off",
      k_themes[g_scene.theme].name, g_scene.paused ? "PAUSED " : "running",
      fps);
  int len = (int)strlen(buf);
  int col = g_cols - len;
  if (col < 0)
    col = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, col, buf, g_cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* The key hints across the bottom, with a shorter version for narrow
 * windows. */
static void draw_hud_bottom(void) {
  const char *full = " q:quit  p:pause  r:reset  c:cube  s:sphere  x:del  "
                     "e/E:rest  g:gravity  t/T:theme ";
  const char *shrt = " q:quit  p:pause  r:reset  c:cube  s:sphere  t:theme ";
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

  /* floor line first, so bodies sitting on it draw over it */
  int floor_row = prow(WH());
  attron(COLOR_PAIR(CP_FLOOR));
  for (int c = 0; c < g_cols; c++)
    mvaddch(floor_row, c, '=');
  attroff(COLOR_PAIR(CP_FLOOR));

  /* then the bodies */
  for (int i = 0; i < g_scene.nb; i++)
    draw_body(&g_scene.b[i]);
  fb_flush();

  /* HUD last, so the top and bottom bars always stay on top */
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
  use_default_colors(); /* lets -1 mean the terminal's own background */
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
  scene_init();

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
        g_scene.grav = !g_scene.grav;
        break;
      case 'e':
        if (g_scene.rest < 0.95f)
          g_scene.rest += REST_STEP;
        break;
      case 'E':
        if (g_scene.rest > 0.05f)
          g_scene.rest -= REST_STEP;
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
    if (!g_scene.paused && now >= next) {
      scene_step();
      next = now + NS_PER_S / SIM_FPS;
    }

    /* count frames over each one-second window for the fps readout */
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
