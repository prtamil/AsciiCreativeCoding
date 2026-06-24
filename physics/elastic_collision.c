/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * elastic_collision.c — bouncing billiard discs
 *
 * 25 discs of random size and speed bounce off the walls and off each
 * other. Hits are "perfectly elastic": no energy is lost, so the discs
 * keep rattling around forever. When two collide they flash for a moment.
 *
 * The two-body collision math is the standard textbook result
 * (Goldstein, Classical Mechanics, ch. 3). For pushing this past ~100
 * discs without the pair-checking getting slow, see Lubachevsky 1991,
 * "How to Simulate Billiards and Similar Systems".
 *
 * §1 config  §2 clock  §3 color  §4 physics  §5 draw  §6 app
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ── */

/* Physics runs in "pixels", with each terminal cell worth 8 px wide and
 * 16 px tall. Cells are about twice as tall as they are wide, so this
 * keeps round things looking round. */
#define CELL_W 8
#define CELL_H 16
#define N_DISCS 25

/* How big a disc can be. The smallest is still a cell and a half wide so
 * you can see it; the biggest is capped at 4 cells so that 25 of them
 * still fit when we drop them in at random (see the placement code). */
#define R_MIN (CELL_W * 1.5f)
#define R_MAX (CELL_W * 4.0f)

/* Top starting speed. Fast enough to look lively, slow enough that a disc
 * never jumps clean past another between two physics steps. */
#define V_MAX 180.f

/* How long a disc glows after a hit, in seconds. About a blink — long
 * enough to catch your eye, short enough not to linger. */
#define FLASH_S 0.4f

/* Physics runs 120 times a second. The small time step keeps fast discs
 * from skipping through each other. */
#define SIM_FPS 120
#define RENDER_NS (1000000000LL / 60) /* redraw 60 times a second */

/* Two reserved rows: a status line up top and a key-hint line at the
 * bottom. Discs only get the space in between. */
#define TOP_HUD_H 1
#define BOT_HUD_H 1
#define HUD_ROWS (TOP_HUD_H + BOT_HUD_H)

#define N_THEMES 10 /* see k_themes[] in §3 */

/* Color slots. Discs are tinted by how fast they're going, with a special
 * flash color for the moment of impact; the last two keep the HUD readable
 * no matter which theme is on. */
enum {
  CP_SLOW = 1, /* slowest discs                    */
  CP_MED,      /* mid-speed discs                  */
  CP_FAST,     /* fastest discs (brightest)        */
  CP_FLASH,    /* the flash when two discs collide */
  CP_HUD,      /* top status line                  */
  CP_HINT,     /* bottom key-hint line             */
};

/* ── §2 clock ── */

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

/* ── §3 color — themes + HUD colors ── */

/*
 * A look for the discs. Each theme has four colors going from dim to
 * bright (used to shade discs by speed) plus one bold "flash" color for
 * the instant two discs hit. The flash is picked to stand out from the
 * four so impacts always pop, even on a low-contrast theme.
 *
 *   name   what shows in the HUD
 *   ramp   four shades, dim to bright (slow discs get ramp[0], the
 *          fastest get ramp[3])
 *   flash  the collision-flash color
 *
 * All color numbers stay in the bright half of the palette on purpose;
 * the dim end of the terminal palette disappears against a black
 * background.
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
 * Switch to theme t and wire up the color slots. The disc shades come
 * from the theme, but the two HUD colors stay fixed so the text is always
 * readable whichever theme you pick. (We skip ramp[1] for the mid tier to
 * leave a clearer gap between slow and medium.)
 */
static void theme_apply(int t) {
  const Theme *th = &k_themes[t % N_THEMES];
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(CP_SLOW, th->ramp[0], -1);
    init_pair(CP_MED, th->ramp[2], -1);
    init_pair(CP_FAST, th->ramp[3], -1);
    init_pair(CP_FLASH, th->flash, -1);
    init_pair(CP_HUD, 226, -1); /* yellow */
    init_pair(CP_HINT, 51, -1); /* cyan   */
  } else {
    /* Older terminals with only 8 colors get a fixed set. */
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

/* ── §4 physics ── */

/*
 * Disc — one bouncing ball.
 *
 * Everything here lives in pixels (the drawing code is the only place
 * that converts to terminal cells). The fields are grouped by how often
 * they change:
 *
 *   x, y      where the disc's center is right now
 *   vx, vy    how fast it's moving, in pixels per second
 *             (these four change every physics step)
 *
 *   r         radius in pixels, somewhere between R_MIN and R_MAX
 *   mass      how heavy it is. We use mass = r², so a disc the size of
 *             a coin weighs as much as its area — bigger discs are
 *             heavier and shove smaller ones around more. Set once and
 *             never changes.
 *
 *   flash     seconds of glow left after a collision. The physics sets
 *             it to FLASH_S on a hit and ticks it down; while it's above
 *             zero the disc is drawn in the flash color instead of its
 *             usual speed color. This is the one field the drawing code
 *             reads — it rides along on the disc so it stays paired with
 *             the right ball.
 */
typedef struct {
  float x, y;   /* center position (top-left is the origin) */
  float vx, vy; /* velocity, pixels per second              */

  float r;    /* radius in pixels (R_MIN..R_MAX)            */
  float mass; /* weight, equal to r²                        */

  float flash; /* seconds of post-hit glow left; >0 means flashing */
} Disc;

/*
 * Scene — all of the demo's state in one place.
 *
 * Passing one Scene around is how everything talks to everything else;
 * the only things that live outside it are the two flags the signal
 * handler sets, because C signal handlers can't be handed a pointer.
 *
 * The fields split into two halves — the physics, and the presentation:
 *
 *   discs       the 25 balls themselves
 *   pw, ph      the size of the box they bounce inside, in pixels.
 *               Worked out from the screen size minus the two HUD rows.
 *   coll_total  how many collisions have happened so far (shown in the
 *               HUD; lives here because the physics is what counts them)
 *   paused      when true, the physics simply stops
 *   speed       how many physics steps to run per frame, 1 to 8. Higher
 *               looks crisper but does more work, since each step
 *               re-checks every pair of discs.
 *
 *   rows, cols  the terminal's size in cells, kept fresh when the window
 *               is resized. Used both to lay out the box and to clip the
 *               discs and place the HUD.
 *   theme       which look is active (an index into k_themes). Purely
 *               cosmetic — nothing in the physics depends on it.
 */
typedef struct {
  /* The physics side. */
  Disc discs[N_DISCS];  /* the 25 balls                       */
  float pw, ph;         /* box size in pixels                 */
  long long coll_total; /* total collisions so far            */
  bool paused;          /* physics frozen when true           */
  int speed;            /* physics steps per frame (1..8)     */

  /* The display side — changing these never disturbs the physics. */
  int rows, cols; /* terminal size in cells              */
  int theme;      /* which look is active                */
} Scene;

/* Give a disc a random size, then set its weight to match (r²). */
static void seed_random_radius_mass(Disc *d) {
  d->r = R_MIN + (float)rand() / RAND_MAX * (R_MAX - R_MIN);
  d->mass = d->r * d->r;
}

/*
 * Drop a disc somewhere inside the box where it doesn't land on top of a
 * disc already placed. We just keep picking random spots until one is
 * clear, giving up after max_tries. If we run out of tries we leave it
 * where it last landed — the very first physics step will nudge any
 * leftover overlap apart.
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
  /* Out of tries: leave it where it is and let the physics sort it out. */
}

/*
 * Send a disc off in a random direction at a random speed. We never let
 * the speed drop below 60 px/s, so no disc just sits there barely moving.
 */
static void seed_random_velocity(Disc *d) {
  float speed = 60.f + (float)rand() / RAND_MAX * (V_MAX - 60.f);
  float angle = (float)rand() / RAND_MAX * 6.28318f;
  d->vx = speed * cosf(angle);
  d->vy = speed * sinf(angle);
}

/*
 * Start (or restart) the demo at the given screen size: size the box,
 * zero the collision count, and lay out fresh random discs. It leaves
 * pause/speed/theme alone on purpose, so hitting reset or resizing the
 * window keeps the choices you made.
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
 * Move a disc forward by dt seconds and let its flash fade a little.
 * Between collisions a disc just coasts in a straight line, so moving it
 * is as simple as position += velocity × time — no forces to worry about.
 */
static void integrate_disc(Disc *d, float dt) {
  d->x += d->vx * dt;
  d->y += d->vy * dt;
  d->flash -= dt;
  if (d->flash < 0.f)
    d->flash = 0.f;
}

/*
 * Bounce a disc off the four walls. For any wall it has crossed, we pull
 * it back inside and flip the matching part of its velocity to point
 * back inward — exactly like a ball rebounding off a hard edge.
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
 * Are these two discs touching? If so, also report the direction from a
 * to b and how deeply they're overlapping (the caller uses both to push
 * them apart and bounce them). If their centers sit exactly on top of
 * each other we bail out, since there'd be no clear direction to use.
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
 * Two overlapping discs shouldn't share space, so shove them apart until
 * they just touch. The lighter one gives way more than the heavier one,
 * and they split the gap so their shared center of gravity doesn't move.
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
 * The actual bounce: swap momentum between two discs along the line that
 * joins their centers, so they ricochet the way real balls do (no energy
 * lost). The amount each one's velocity changes depends on the weights —
 * this is the standard two-body elastic-collision result (Goldstein,
 * ch. 3). If the discs are already drifting apart we do nothing and
 * return false, so a glancing touch doesn't get a phantom kick.
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
 * Handle one pair of discs end to end: if they're touching, pull them
 * apart, bounce them, and light them both up. Returns true only when a
 * real bounce happened, so the caller knows when to bump the counter.
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
 * Advance the whole world by dt seconds: move every disc and bounce it
 * off the walls, then check every pair for collisions. Does nothing while
 * paused.
 */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;

  /* Move each disc and bounce it off the walls. */
  for (int i = 0; i < N_DISCS; i++) {
    integrate_disc(&s->discs[i], dt);
    reflect_off_walls(&s->discs[i], s->pw, s->ph);
  }

  /* Check every disc against every other one. Fine for 25 discs; for a
   * lot more you'd want a smarter scheme so you don't test all pairs. */
  for (int i = 0; i < N_DISCS - 1; i++) {
    for (int j = i + 1; j < N_DISCS; j++) {
      if (pair_collide(&s->discs[i], &s->discs[j]))
        s->coll_total++;
    }
  }
}

/* ── §5 draw ── */

static int px_to_cell_x(float px) { return (int)(px / CELL_W + .5f); }
static int px_to_cell_y(float py) {
  return (int)(py / CELL_H + .5f) + TOP_HUD_H;
}

/*
 * Work out where a disc lands on the character grid and how many cells
 * wide and tall it should be drawn. We measure the radius separately for
 * width and height because cells are about twice as tall as they are wide,
 * so the same disc spans more columns than rows. Tiny discs are bumped up
 * to at least one cell each way so they never vanish.
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
 * Put a single 'O' at the disc's center. We skip it if it would land on
 * the top or bottom HUD rows, so a disc never paints over the status text.
 */
static void draw_disc_center(int cx, int cy, int cols, int rows) {
  if (cy >= TOP_HUD_H && cy < rows - BOT_HUD_H && cx >= 0 && cx < cols)
    mvaddch(cy, cx, 'O');
}

/*
 * Trace the outline of the disc one column at a time. For each column we
 * figure out how far up and down the rim reaches and drop a character
 * there — a top half and a bottom half. We use '|' at the far left, far
 * right, and the very top/bottom where the edge runs steeply, and '-'
 * everywhere else where it runs flat. Anything landing on a HUD row is
 * skipped. (Standard column-by-column ellipse drawing; Foley et al.,
 * Computer Graphics, §3.3.)
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
 * Draw one whole disc: the 'O' at its center and the outline around it,
 * both in the given color. We set the color once for both so the disc is
 * a single shade.
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
 * Pick a disc's color from how fast it's going: slow, medium, or fast,
 * split at 40% and 80% of the top speed so the three bands are roughly
 * even. A disc that just collided wins out and shows the flash color
 * until its glow fades.
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
 * The status line along the top: how many discs, how many collisions so
 * far, the speed setting, the theme name, and whether we're paused. It's
 * right-aligned so it tucks into the corner out of the way.
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
 * The key reminders along the bottom. If the window is too narrow to fit
 * the full list we show a shorter one instead so it never gets cut off.
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
 * Draw a frame: every disc first (colored by speed), then the HUD on top.
 */
static void scene_draw(const Scene *s) {
  for (int i = 0; i < N_DISCS; i++) {
    const Disc *d = &s->discs[i];
    draw_disc(d, speed_tier_pair(d), s->cols, s->rows);
  }
  draw_hud_top(s);
  draw_hud_bottom(s);
}

/* ── §6 app ── */

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
