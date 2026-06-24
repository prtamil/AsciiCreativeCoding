/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * chain.c — a hanging chain / swinging rope you can watch in the terminal.
 *
 * The rope is a row of beads joined by links that don't stretch. Instead of
 * tracking forces and velocities, we just nudge bead positions until every
 * link is back to its proper length — repeat that nudge a few times and the
 * rope holds together. This trick is Position-Based Dynamics; it never blows
 * up no matter how stiff you make it. Ten presets show it off: a plain hang,
 * a pendulum, a sagging bridge, a driven wave, and so on (see §6).
 *
 * Background, if you want the real names and the papers behind them:
 *   PBD position-nudging idea       — Müller et al., "Position Based Dynamics" (2007)
 *   the Verlet + relaxation recipe  — Jakobsen, "Advanced Character Physics" (GDC 2001)
 *   pendulum / sag / wave behaviour — Goldstein, Classical Mechanics; Crawford, Waves
 *   safe theme colour ramps         — Ware, Information Visualization
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
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ── */

enum {
  SIM_FPS_DEFAULT = 60, /* how many frames per second we aim for       */
  SIM_FPS_MIN = 10,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,

  N_NODES_MAX = 32, /* most beads a rope can have                   */
  N_NODES_DEF = 24, /* beads in a normal rope; more = smoother but  */
                    /* slower, since every bead is work each step   */
  N_ITER_DEF = 20,  /* how many times we re-nudge the links each    */
                    /* step; more passes = stiffer rope, always safe*/
  N_ITER_MIN = 5,
  N_ITER_MAX = 60,
  SUB_STEPS = 8, /* we split each frame into this many tiny physics */
                 /* steps; smaller steps keep a stiff rope steady   */
                 /* without speeding up or slowing down the display */

  TRAIL_LEN = 90, /* how many past positions of the swinging tip we  */
                  /* remember for the fading trail (~1.5 s at 60 fps)*/
  N_PRESETS = 10, /* number of built-in scenes (see §6)              */
  N_THEMES = 10,  /* number of colour palettes (see k_themes in §3)  */
};

#define NS_PER_SEC 1000000000LL       /* nanoseconds in one second      */
#define TICK_NS(f) (NS_PER_SEC / (f)) /* nanoseconds per frame at fps f */

/* We do all the physics in tiny "pixels", then round to character cells only
 * when drawing. One cell is 8 pixels wide and 16 tall — about twice as tall
 * as wide, which matches how monospace fonts look. */
#define CELL_W 8
#define CELL_H 16

/* How hard gravity pulls the beads down (in pixels per second, per second).
 * Real-world gravity feels too slow at this scale, so this number was just
 * tuned by eye until the rope sways at a nice pace on a normal terminal. */
#define GRAVITY 380.f

/* Tiny bit of drag so the rope eventually settles instead of swinging
 * forever. Each step we keep 99.7% of a bead's motion and shed the rest. */
#define DAMP 0.997f

/* How fast the wind swings left and right. 0.35 Hz is one slow gust every
 * ~3 seconds — feels like a breeze, not a buzzing motor. */
#define WIND_FREQ 0.35f

/* The "Wave" and "Snake" presets grab the top bead and shake it like a hand
 * holding a jump rope. These set how fast and how far that shake goes. */

/* How fast the shaking hand moves. Tuned so the ripples line up into a clean
 * standing pattern on a normal-length rope. */
#define WAVE_FREQ 1.8f

/* How far the shaking hand moves (in pixels) — about 3 cells, big enough to
 * see the wave clearly but not so big the rope folds over itself. */
#define WAVE_AMPL 22.f

/* ── §2 clock ── */

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

/* ── §3 color / theme ── */

/*
 * Each thing we draw gets its own colour slot. The first six change with the
 * chosen theme; the last two (the status bar and the key hints) stay the same
 * on every theme so they're always readable.
 *   1   link — relaxed (close to its proper length)
 *   2   link — stretched
 *   3   link — really stretched / about to snap-taut
 *   4   bead — free to swing
 *   5   bead — pinned in place, or the bead being shaken
 *   6   the fading trail behind the swinging tip
 *   7   top status bar (fixed: bright yellow)
 *   8   bottom key hints (fixed: cyan)
 */
enum {
  CP_LINK_LO = 1,
  CP_LINK_MID = 2,
  CP_LINK_HI = 3,
  CP_NODE = 4,
  CP_ANCHOR = 5,
  CP_TRAIL = 6,
  CP_HUD = 7,
  CP_HINT = 8,
};

/*
 * Theme — one colour palette for the rope. Each field is a terminal colour
 * number (from the 256-colour set) picked for one part of the picture.
 *
 *   name              what to show in the HUD (e.g. "Matrix")
 *   link_lo/mid/hi    three shades for the rope links, from relaxed to taut.
 *                     They go light-to-hot on purpose, so a stretched link
 *                     "heats up" in colour and you can see where the strain is.
 *   node              colour of a free, swinging bead
 *   anchor            colour of a pinned bead, or the bead being shaken
 *   trail             colour of the fading trail behind the tip
 *
 * The status bar and key-hint colours are NOT here — they're fixed across
 * every theme so the text on screen is always readable.
 *
 * One rule on the numbers: keep every colour bright enough to show up on a
 * default (often black) background. Very dark indices (16-23, 232-239) vanish,
 * so we never use those; the darkest we allow is the 24-29 / 240-243 range,
 * and only as a link_lo.
 */
typedef struct {
  const char *name;
  short link_lo, link_mid, link_hi;
  short node, anchor, trail;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*  name        link_lo link_mid link_hi  node  anchor  trail */
    {"Matrix", 28, 40, 46, 231, 226, 82},      /* cyber green */
    {"Fire", 130, 208, 196, 231, 226, 220},    /* warm → red  */
    {"Oceanic", 24, 39, 51, 195, 231, 87},     /* teal → cyan */
    {"Neon", 129, 201, 213, 231, 226, 51},     /* purple/pink */
    {"Mono", 240, 247, 255, 255, 255, 244},    /* grayscale   */
    {"Ice", 153, 159, 195, 231, 255, 117},     /* light blues */
    {"Nova", 129, 141, 213, 231, 226, 177},    /* stellar     */
    {"Forest", 58, 100, 190, 178, 226, 130},   /* leaves/bark */
    {"Desert", 130, 178, 220, 230, 226, 178},  /* sand/gold   */
    {"Eclipse", 240, 244, 196, 255, 226, 124}, /* gray + red  */
};

/* Takes the theme number as an argument (rather than reaching into the Scene
 * struct, which isn't defined until §6) so this can live up here near the
 * palette table it uses. */
static void theme_apply(int t) {
  const Theme *th = &k_themes[t % N_THEMES];
  if (COLORS >= 256) {
    init_pair(CP_LINK_LO, th->link_lo, -1);
    init_pair(CP_LINK_MID, th->link_mid, -1);
    init_pair(CP_LINK_HI, th->link_hi, -1);
    init_pair(CP_NODE, th->node, -1);
    init_pair(CP_ANCHOR, th->anchor, -1);
    init_pair(CP_TRAIL, th->trail, -1);
    /* The two HUD bars stay the same on every theme so they're always legible. */
    init_pair(CP_HUD, 226, -1); /* bright yellow */
    init_pair(CP_HINT, 51, -1); /* bright cyan   */
  } else {
    /* Plain 8-colour terminals: a simple fallback, ignores the theme. */
    init_pair(CP_LINK_LO, COLOR_CYAN, -1);
    init_pair(CP_LINK_MID, COLOR_YELLOW, -1);
    init_pair(CP_LINK_HI, COLOR_RED, -1);
    init_pair(CP_NODE, COLOR_WHITE, -1);
    init_pair(CP_ANCHOR, COLOR_YELLOW, -1);
    init_pair(CP_TRAIL, COLOR_WHITE, -1);
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  theme_apply(0);
}

/* ── §4 coords ── */

/* Turn a fine-grained pixel position into a character-cell row or column.
 * Adding 0.5 before chopping to an int rounds to the nearest cell. */
static inline int px_to_cx(float px) { return (int)(px / CELL_W + 0.5f); }
static inline int px_to_cy(float py) { return (int)(py / CELL_H + 0.5f); }

/* ── §5 physics ── */

/*
 * ChainNode — one bead of the rope.
 *
 *   x, y      where the bead is right now, in pixels
 *   ox, oy    where it was on the previous step, also in pixels
 *   pinned    true if this bead is held in place (an anchor, or the bead
 *             being shaken) instead of swinging freely
 *
 * Why store the old position instead of a velocity? Speed is just "how far
 * the bead moved last step" — so the gap between (x,y) and (ox,oy) already
 * tells us that, and we never store a separate velocity. This keeps the
 * update to one line, makes drag a simple multiply, and means our position
 * nudges automatically carry over as next step's motion. (This is Verlet
 * integration; the trick comes from Jakobsen.)
 *
 * Positions are floats, not whole cells, so the rope moves smoothly instead
 * of jumping a full character at a time; we round to cells only when drawing.
 *
 * A pinned bead is left where some other code puts it each step and is
 * skipped by the link-fixing loop — think of it as infinitely heavy, so it
 * never gets pushed around. When a link has one pinned end, the free end
 * takes the whole correction instead of splitting it.
 */
typedef struct {
  float x, y;   /* where the bead is now (pixels)                  */
  float ox, oy; /* where it was last step (pixels)                 */
  bool pinned;  /* held in place? if so, the link loop skips it    */
} ChainNode;

/*
 * Chain — everything about one rope, in a single bundle: the beads
 * themselves, the wind, the live settings you change with the keyboard, which
 * preset is running, and the trail history we draw behind the tip. It's all
 * here together because resetting a scene and stepping the physics both touch
 * all of it at once.
 *
 * The fields fall into a few groups, matching the layout below:
 *
 *   The rope          nodes[], n_nodes, link_rest — the beads, how many
 *                     there are, and how long each link should be.
 *
 *   Wind              wind_phase, wind_str, wind_on — a side-to-side breeze.
 *                     phase is where the gust is in its swing, str is how
 *                     hard it blows (different per preset — gentle for a
 *                     bridge, fierce for a storm), on/off is the 'w' key.
 *
 *   Live settings     paused, n_iter — paused freezes the rope; n_iter is how
 *                     many link-fixing passes we do, i.e. how stiff the rope
 *                     feels (more passes = stiffer, and always stable).
 *
 *   Which scene       preset — picks which built-in scene is running, and lets
 *                     the step code turn on the special shaking for the Wave
 *                     and Snake scenes.
 *
 *   Shaking state     wave_phase, wave_ax, wave_ay — for the two scenes that
 *                     shake the top bead: where the hand started and where it
 *                     is in its motion. Ignored by the other scenes.
 *
 *   Trail history     trail_px/py plus trail_head/trail_cnt — a fixed-size
 *                     circular log of the swinging tip's recent positions.
 *                     head is where the next sample goes; cnt is how many
 *                     we've collected (it stops growing once the log is full).
 *                     We reuse the oldest slot instead of shifting everything.
 *
 * The bead array has a fixed maximum size and lives inside the struct (no
 * heap allocation) — a rope never gets longer than that, and keeping it all
 * in one block is fast for the tight per-frame loop.
 */
typedef struct {
  /* ── the rope ── */
  ChainNode nodes[N_NODES_MAX];
  int n_nodes;
  float link_rest; /* how long each link wants to be, in pixels    */

  /* ── wind ── */
  float wind_phase; /* where the gust is in its left-right swing    */
  float wind_str;   /* how hard it blows (set per preset)           */
  bool wind_on;     /* 'w' key turns it on/off                      */

  /* ── live settings ── */
  bool paused; /* freeze the rope (space / 'p')                */
  int n_iter;  /* link-fixing passes per step = how stiff the
                * rope feels; '+'/'-' keys, clamped to a range */

  /* ── which scene ── */
  int preset; /* which built-in scene is running             */

  /* ── shaking state (Wave & Snake scenes only) ── */
  float wave_phase; /* where the shaking hand is in its motion      */
  float wave_ax;    /* where the shaken bead started (x)            */
  float wave_ay;    /* where the shaken bead started (y)            */

  /* ── trail history (used only for drawing) ── */
  float trail_px[TRAIL_LEN];
  float trail_py[TRAIL_LEN];
  int trail_head; /* slot the next sample goes into               */
  int trail_cnt;  /* how many samples we have (caps at TRAIL_LEN) */
} Chain;

/* ── drawing a link ── */

/* Pick the character that best looks like the tilt of a link: '-' if it's
 * mostly flat, '|' if it's mostly upright, and '\' or '/' for the two
 * diagonals. We use one character for the whole link so it reads as a single
 * stroke. */
static char slope_glyph(int dx, int dy) {
  int adx = abs(dx), ady = abs(dy);
  if (adx >= 2 * ady)
    return '-';
  else if (ady >= 2 * adx)
    return '|';
  else if (dx * dy > 0)
    return '\\';
  else
    return '/';
}

/* Draws a straight line of characters from one bead to the next. Anything off
 * the sides, or on the top and bottom rows (kept clear for the HUD), is
 * skipped. A link with both ends in the same cell just gets a single 'o'. */
static void seg_draw(int x0, int y0, int x1, int y1, int cols, int rows,
                     chtype attr) {
  int dx = x1 - x0;
  int dy = y1 - y0;
  int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);

  if (steps == 0) {
    if (x0 >= 0 && x0 < cols && y0 >= 1 && y0 < rows - 1) {
      attron(attr);
      mvaddch(y0, x0, 'o');
      attroff(attr);
    }
    return;
  }

  char ch = slope_glyph(dx, dy);

  attron(attr);
  for (int k = 0; k <= steps; k++) {
    int cx = x0 + k * dx / steps;
    int cy = y0 + k * dy / steps;
    if (cx >= 0 && cx < cols && cy >= 1 && cy < rows - 1)
      mvaddch(cy, cx, ch);
  }
  attroff(attr);
}

/* ── one physics step ── */

/* How hard the wind is blowing right now — it swings smoothly left and right.
 * Returns zero when the wind is switched off. */
static float compute_wind_force(const Chain *c) {
  return c->wind_on ? c->wind_str * sinf(c->wind_phase) : 0.f;
}

/* Move one free bead forward by a step: it keeps drifting the way it was
 * already going (a little slower, thanks to drag), plus gravity pulls it down
 * and the wind nudges it sideways. Then today's spot becomes "last step's".
 * Pinned beads don't move here — something else places them. */
static void verlet_predict_node(ChainNode *n, float dt2, float wind_force) {
  if (n->pinned)
    return;
  float vx = (n->x - n->ox) * DAMP;
  float vy = (n->y - n->oy) * DAMP;
  float new_x = n->x + vx + wind_force * dt2;
  float new_y = n->y + vy + GRAVITY * dt2;
  n->ox = n->x;
  n->oy = n->y;
  n->x = new_x;
  n->y = new_y;
}

/* Fix one link back toward its proper length by sliding its two beads along
 * the line between them — apart if it's too short, together if it's too long.
 * How the fix is shared:
 *   both ends pinned   → leave it; the pins win
 *   one end pinned     → the free end does all the moving
 *   neither pinned     → the two beads split the move evenly
 * If the two beads sit on top of each other we skip it (no direction to push
 * in, and it would divide by zero). This is the heart of PBD (Müller). */
static void project_link_constraint(ChainNode *a, ChainNode *b, float rest) {
  float dx = b->x - a->x;
  float dy = b->y - a->y;
  float dist = sqrtf(dx * dx + dy * dy);
  if (dist < 1e-6f)
    return;

  float inv = (dist - rest) / dist;
  float cx = inv * dx;
  float cy = inv * dy;

  if (a->pinned && b->pinned) {
    return;
  } else if (a->pinned) {
    b->x -= cx;
    b->y -= cy;
  } else if (b->pinned) {
    a->x += cx;
    a->y += cy;
  } else {
    a->x += 0.5f * cx;
    a->y += 0.5f * cy;
    b->x -= 0.5f * cx;
    b->y -= 0.5f * cy;
  }
}

/* Fix every link, several times over. One pass alone leaves the rope a bit
 * loose, because fixing one link tugs its neighbours out of shape; doing it
 * again and again settles the whole rope. More passes = a stiffer rope, and
 * it never becomes unstable. */
static void relax_link_constraints(Chain *c) {
  for (int iter = 0; iter < c->n_iter; iter++) {
    for (int i = 0; i < c->n_nodes - 1; i++)
      project_link_constraint(&c->nodes[i], &c->nodes[i + 1], c->link_rest);
  }
}

/* One tiny physics step for the whole rope: work out the wind, let every bead
 * drift forward under gravity and wind, then fix the links back to length a
 * bunch of times so the rope holds together. */
static void chain_pbd_step(Chain *c, float dt) {
  float dt2 = dt * dt;
  float wind = compute_wind_force(c);

  for (int i = 0; i < c->n_nodes; i++)
    verlet_predict_node(&c->nodes[i], dt2, wind);

  relax_link_constraints(c);
}

/* ── one frame of physics ── */

/* Nudges a wave's "where are we in the swing" counter forward by one tick,
 * then folds it back to the start once it passes a full circle. Folding it
 * back stops the number from growing forever, which over a long run would
 * slowly make the sin/cos wobble lose accuracy. */
static void advance_phase_wrapped(float *phase, float freq, float dt) {
  *phase += freq * 2.f * (float)M_PI * dt;
  if (*phase > 2.f * (float)M_PI)
    *phase -= 2.f * (float)M_PI;
}

/* The Wave preset: shake the top bead side to side, and the ripple travels
 * down the rope on its own as the links pull each bead along. We set both
 * where the bead is and where it was a moment ago — setting both is what tells
 * the rope the bead is already moving when the link-fixing starts, so the
 * shake carries through smoothly. */
static void drive_wave_anchor_horizontal(Chain *c, int sub_step, float dt,
                                         float sub_dt) {
  float sub_t = (float)sub_step / (float)SUB_STEPS;
  float phase_now =
      c->wave_phase - WAVE_FREQ * 2.f * (float)M_PI * dt * (1.f - sub_t);
  float phase_prev = phase_now - WAVE_FREQ * 2.f * (float)M_PI * sub_dt;
  c->nodes[0].x = c->wave_ax + WAVE_AMPL * sinf(phase_now);
  c->nodes[0].ox = c->wave_ax + WAVE_AMPL * sinf(phase_prev);
  c->nodes[0].y = c->wave_ay;
  c->nodes[0].oy = c->wave_ay;
}

/* The Snake preset: same shake as the Wave one, but up-and-down instead of
 * side to side, on a rope strung out flat. The wiggle runs along the rope,
 * bounces off the pinned far end, and overlaps itself into a slithering
 * pattern (Crawford, Waves). We shake it half as hard here because up-and-down
 * motion fights gravity — a full-size shake would just fling the bead off the
 * top of the screen. */
static void drive_wave_anchor_vertical(Chain *c, int sub_step, float dt,
                                       float sub_dt) {
  float sub_t = (float)sub_step / (float)SUB_STEPS;
  float phase_now =
      c->wave_phase - WAVE_FREQ * 2.f * (float)M_PI * dt * (1.f - sub_t);
  float phase_prev = phase_now - WAVE_FREQ * 2.f * (float)M_PI * sub_dt;
  c->nodes[0].x = c->wave_ax;
  c->nodes[0].ox = c->wave_ax;
  c->nodes[0].y = c->wave_ay + WAVE_AMPL * 0.5f * sinf(phase_now);
  c->nodes[0].oy = c->wave_ay + WAVE_AMPL * 0.5f * sinf(phase_prev);
}

/* Only two scenes shake a bead by hand — Wave and Snake. The rest just let the
 * rope fall and swing on its own, so this does nothing for them. */
static void apply_preset_driver(Chain *c, int sub_step, float dt,
                                float sub_dt) {
  if (c->preset == 3)
    drive_wave_anchor_horizontal(c, sub_step, dt, sub_dt);
  else if (c->preset == 5)
    drive_wave_anchor_vertical(c, sub_step, dt, sub_dt);
}

/* Remember where the bottom bead is right now, so we can draw its fading trail.
 * Skipped when the bottom bead is pinned (like Bridge or Festoon) — a bead
 * that never moves leaves no trail worth showing. */
static void record_bottom_trail_sample(Chain *c) {
  int last = c->n_nodes - 1;
  if (c->nodes[last].pinned)
    return;
  c->trail_px[c->trail_head] = c->nodes[last].x;
  c->trail_py[c->trail_head] = c->nodes[last].y;
  c->trail_head = (c->trail_head + 1) % TRAIL_LEN;
  if (c->trail_cnt < TRAIL_LEN)
    c->trail_cnt++;
}

/* One frame of the rope's life: nudge the wind and shake along, then run the
 * physics in several tiny steps so a stiff rope stays calm, and finally jot
 * down where the tip is for the trail. */
static void chain_tick(Chain *c, float dt) {
  if (c->paused)
    return;

  float sub_dt = dt / (float)SUB_STEPS;

  advance_phase_wrapped(&c->wind_phase, WIND_FREQ, dt);
  advance_phase_wrapped(&c->wave_phase, WAVE_FREQ, dt);

  for (int s = 0; s < SUB_STEPS; s++) {
    apply_preset_driver(c, s, dt, sub_dt);
    chain_pbd_step(c, sub_dt);
  }

  record_bottom_trail_sample(c);
}

/* ── drawing the rope ── */

/* How far this link is stretched, as a fraction of its proper length: 0 means
 * relaxed, 1 means stretched to double. The tiny +0.001 just guards against
 * dividing by zero if a link's proper length is ever zero. */
static float link_strain(const ChainNode *a, const ChainNode *b, float rest) {
  float dx = b->x - a->x;
  float dy = b->y - a->y;
  float dist = sqrtf(dx * dx + dy * dy);
  return fabsf(dist - rest) / (rest + 0.001f);
}

/* Pick a colour for a link based on how stretched it is: relaxed, stretched,
 * or really taut. The two cut-offs were tuned by eye so the colour shifts at
 * about the strain where a real rope would visibly tighten. With the §3
 * palettes this makes the rope look like it "heats up" where it's pulled. */
static int strain_color_pair(float strain) {
  if (strain < 0.04f)
    return CP_LINK_LO;
  if (strain < 0.12f)
    return CP_LINK_MID;
  return CP_LINK_HI;
}

/* Draw the fading trail behind the tip, oldest dots first so newer ones land
 * on top. The freshest part of the path shows as bright '*'; older history
 * fades to a dim '.'. We don't store a brightness per dot — just walking the
 * list from old to new is enough to know how faded each one should be. */
static void paint_trail(const Chain *c, int cols, int rows) {
  int start = (c->trail_head - c->trail_cnt + TRAIL_LEN) % TRAIL_LEN;
  for (int i = 0; i < c->trail_cnt; i++) {
    int idx = (start + i) % TRAIL_LEN;
    int cx = px_to_cx(c->trail_px[idx]);
    int cy = px_to_cy(c->trail_py[idx]);
    if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1)
      continue;

    float age = (float)i / (float)(c->trail_cnt > 1 ? c->trail_cnt : 1);
    bool fresh = (age > 0.6f);
    attron(COLOR_PAIR(CP_TRAIL) | (fresh ? A_BOLD : 0));
    mvaddch(cy, cx, fresh ? '*' : '.');
    attroff(COLOR_PAIR(CP_TRAIL) | (fresh ? A_BOLD : 0));
  }
}

/* Draw every link as a short line, coloured by how stretched it is. */
static void paint_links(const Chain *c, int cols, int rows) {
  for (int i = 0; i < c->n_nodes - 1; i++) {
    const ChainNode *a = &c->nodes[i];
    const ChainNode *b = &c->nodes[i + 1];

    int ax = px_to_cx(a->x), ay = px_to_cy(a->y);
    int bx = px_to_cx(b->x), by = px_to_cy(b->y);

    int cp = strain_color_pair(link_strain(a, b, c->link_rest));
    seg_draw(ax, ay, bx, by, cols, rows, COLOR_PAIR(cp) | A_BOLD);
  }
}

/* Draw every bead, with its look hinting at its job: a pinned bead at either
 * end is a '#' (a mount on the wall), a pinned bead in the middle is a '*' (a
 * hook the rope hangs from), and a free, swinging bead is a plain 'o'. The
 * different shapes just make it easy to tell anchors apart at a glance. */
static void paint_nodes(const Chain *c, int cols, int rows) {
  int N = c->n_nodes;
  for (int i = 0; i < N; i++) {
    const ChainNode *n = &c->nodes[i];
    int cx = px_to_cx(n->x), cy = px_to_cy(n->y);
    if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1)
      continue;

    if (n->pinned) {
      attron(COLOR_PAIR(CP_ANCHOR) | A_BOLD);
      mvaddch(cy, cx, (i == 0 || i == N - 1) ? '#' : '*');
      attroff(COLOR_PAIR(CP_ANCHOR) | A_BOLD);
    } else {
      attron(COLOR_PAIR(CP_NODE));
      mvaddch(cy, cx, 'o');
      attroff(COLOR_PAIR(CP_NODE));
    }
  }
}

/* Draw the rope back to front: the faint trail first, then the links over it,
 * then the beads on top so they're never hidden. */
static void chain_draw(const Chain *c, int cols, int rows) {
  paint_trail(c, cols, rows);
  paint_links(c, cols, rows);
  paint_nodes(c, cols, rows);
}

/* ── §6 scene ── */

/* Each preset lays out the beads in a starting pose and pins whichever ones
 * stay fixed; the physics in §5 takes it from there. Positions are given as
 * fractions of the screen so every scene looks right at any terminal size. */

/*
 * Scene — all the live state for one run, gathered in one place: the rope
 * itself, plus the handful of settings you change from the keyboard. It's
 * split into two groups, and the split is for the reader, not the computer:
 *
 *   Simulation  — anything that changes how the rope MOVES (the rope state,
 *                 the sim speed). Touched by the keys that affect physics:
 *                 n / N (next/prev scene), r (reset), p / space (pause),
 *                 +/- (stiffness), w (wind), ] / [ (sim speed).
 *
 *   Rendering   — anything that only changes how it LOOKS (the colour theme).
 *                 Touched by the cosmetic keys t / T. Flipping a theme while
 *                 paused must not budge the rope by a hair — only colours.
 *
 * Why keep them apart? So a new setting doesn't quietly let a "looks" change
 * leak into "moves". When you add a field, ask: does this change how the rope
 * moves? If yes it's simulation; if no it's rendering.
 *
 *   fields
 *     chain     the rope and its physics (see §5)
 *     sim_fps   how many physics/draw frames per second, 10–120, ] / [ keys
 *     theme     which colour palette, an index into k_themes[] in §3, t / T keys
 *
 * There's just one of these (the file-wide g_scene). The rope's bead array is
 * by far the biggest part (~25 KB), so the whole thing lives in static memory
 * instead of being passed around by pointer. A couple of things stay OUTSIDE
 * on purpose: the quit/resize flags (they're poked by signal handlers and must
 * stay at file scope to be signal-safe), and the screen size (the main loop
 * owns that, so the rope code never has to care about resizing).
 */
typedef struct {
  /* ── simulation ── */
  Chain chain; /* the rope and its physics (§5)               */
  int sim_fps; /* physics/draw frames per second, ] / [ keys (10–120) */

  /* ── rendering ── */
  int theme; /* which palette, an index into k_themes[] in §3; t / T keys */
} Scene;

static Scene g_scene = {
    .sim_fps = SIM_FPS_DEFAULT, .theme = 0,
    /* .chain is BSS-zeroed; scene_init() populates it before any read. */
};

/* Current screen size. The main loop owns this, not the rope (see Scene above). */
static int g_rows, g_cols;

static const char *k_preset_names[N_PRESETS] = {
    "Hanging", "Pendulum", "Bridge",    "Wave", "Festoon",
    "Snake",   "Storm",    "Slingshot", "Flag", "Double"};

/* Shared setup every preset starts from: wipe the rope clean and fill in the
 * basics — how many beads, how long each link should be, default stiffness. */
static void chain_init_common(Chain *c, int n_nodes, float link_rest) {
  memset(c, 0, sizeof *c);
  c->n_nodes = n_nodes;
  c->link_rest = link_rest;
  c->n_iter = N_ITER_DEF;
  c->wind_on = true;
  c->trail_head = 0;
  c->trail_cnt = 0;
}

/* ── Preset 0: Hanging ── */
/* A plain rope pinned at the top-centre, hanging straight down. A gentle wind
 * swings it slowly side to side. */
static void preset_hanging(Chain *c) {
  float anchor_x = (float)g_cols * CELL_W * 0.5f;
  float anchor_y = (float)g_rows * CELL_H * 0.07f;
  float total_len = (float)g_rows * CELL_H * 0.75f;
  int N = N_NODES_DEF;
  float rest = total_len / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_str = 140.f;
  c->preset = 0;

  for (int i = 0; i < N; i++) {
    c->nodes[i].x = c->nodes[i].ox = anchor_x;
    c->nodes[i].y = c->nodes[i].oy = anchor_y + (float)i * rest;
    c->nodes[i].pinned = (i == 0);
  }
}

/* ── Preset 1: Pendulum ── */
/* Pinned at the top and started leaning well off to one side, then let go. It
 * swings back and forth, with little ripples running along the rope. */
static void preset_pendulum(Chain *c) {
  float anchor_x = (float)g_cols * CELL_W * 0.5f;
  float anchor_y = (float)g_rows * CELL_H * 0.07f;
  float total_len = (float)g_rows * CELL_H * 0.70f;
  int N = N_NODES_DEF;
  float rest = total_len / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_on = false;
  c->preset = 1;

  float angle = (float)M_PI / 3.f; /* lean 60° off straight-down — far enough
                                    * for a dramatic swing, not so far it flips
                                    * past sideways */
  float sin_a = sinf(angle), cos_a = cosf(angle);

  for (int i = 0; i < N; i++) {
    float t = (float)i * rest;
    c->nodes[i].x = c->nodes[i].ox = anchor_x + sin_a * t;
    c->nodes[i].y = c->nodes[i].oy = anchor_y + cos_a * t;
    c->nodes[i].pinned = (i == 0);
  }
}

/* ── Preset 2: Bridge ── */
/* Pinned at both ends, same height, with some slack rope between them so it
 * sags into a hammock shape. A breeze makes it billow. */
static void preset_bridge(Chain *c) {
  float span = (float)g_cols * CELL_W * 0.65f;
  float ax = (float)g_cols * CELL_W * 0.175f;
  float ay = (float)g_rows * CELL_H * 0.28f;
  int N = N_NODES_DEF;
  /* Make the rope 25% longer than the gap between the two pins. The extra
   * length has nowhere to go but down, so the rope droops into a sag — the
   * more slack, the deeper the droop. */
  float rest = span * 1.25f / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_str = 110.f;
  c->wind_on = true;
  c->preset = 2;

  /* lay the beads out in a straight line between the two pins */
  for (int i = 0; i < N; i++) {
    float t = (float)i / (float)(N - 1);
    c->nodes[i].x = c->nodes[i].ox = ax + t * span;
    c->nodes[i].y = c->nodes[i].oy = ay;
    c->nodes[i].pinned = (i == 0 || i == N - 1);
  }
}

/* ── Preset 3: Wave ── */
/* The top bead is shaken side to side. The ripple runs down the rope, bounces
 * off the loose bottom end, and overlaps itself — at the right shake speed the
 * overlaps line up into a clean standing pattern that seems to hold still. */
static void preset_wave(Chain *c) {
  float anchor_x = (float)g_cols * CELL_W * 0.5f;
  float anchor_y = (float)g_rows * CELL_H * 0.06f;
  float total_len = (float)g_rows * CELL_H * 0.80f;
  int N = N_NODES_DEF;
  float rest = total_len / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_on = false;
  c->preset = 3;
  c->wave_ax = anchor_x;
  c->wave_ay = anchor_y;
  c->wave_phase = 0.f;

  /* start hanging straight down */
  for (int i = 0; i < N; i++) {
    c->nodes[i].x = c->nodes[i].ox = anchor_x;
    c->nodes[i].y = c->nodes[i].oy = anchor_y + (float)i * rest;
    c->nodes[i].pinned = (i == 0);
  }
}

/* ── Preset 4: Festoon ── */
/* Pinned in three spots across the top — both ends and the middle — with extra
 * slack so the two halves droop into matching swags, like a string of garland.
 * A soft breeze rocks both swags together. */
static void preset_festoon(Chain *c) {
  int N = N_NODES_DEF;
  float W = (float)g_cols * CELL_W;
  float top_y = (float)g_rows * CELL_H * 0.12f;
  float px0 = W * 0.10f;
  float px1 = W * 0.50f;
  float px2 = W * 0.90f;
  /* Make the rope half again as long as the full pin-to-pin width, so each
   * half has slack to droop. */
  float total = (px2 - px0) * 1.50f;
  float rest = total / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_str = 60.f; /* soft breeze, so the swags rock slowly */
  c->wind_on = true;
  c->preset = 4;

  int p_mid = N / 2;
  for (int i = 0; i < N; i++) {
    float t = (float)i / (float)(N - 1);
    float x = (t <= 0.5f) ? px0 + (t * 2.f) * (px1 - px0)
                          : px1 + ((t - 0.5f) * 2.f) * (px2 - px1);
    c->nodes[i].x = c->nodes[i].ox = x;
    c->nodes[i].y = c->nodes[i].oy = top_y;
    c->nodes[i].pinned = (i == 0 || i == p_mid || i == N - 1);
  }
}

/* ── Preset 5: Snake ── */
/* A rope strung out flat, pinned at both ends. The top bead is shaken up and
 * down, so a wiggle runs along the rope, bounces off the far pin, and overlaps
 * itself — the rope slithers like a snake. (The up-and-down shaking is set up
 * in chain_tick for this preset.) */
static void preset_snake(Chain *c) {
  int N = N_NODES_DEF;
  float span = (float)g_cols * CELL_W * 0.70f;
  float ax = (float)g_cols * CELL_W * 0.15f;
  float ay = (float)g_rows * CELL_H * 0.45f;
  float rest = span * 1.05f / (float)(N - 1); /* a touch of slack (5%) */

  chain_init_common(c, N, rest);
  c->wind_on = false;
  c->preset = 5;
  c->wave_ax = ax;
  c->wave_ay = ay;
  c->wave_phase = 0.f;

  /* Start flat and straight; the shaking in chain_tick makes the wave. */
  for (int i = 0; i < N; i++) {
    float t = (float)i / (float)(N - 1);
    c->nodes[i].x = c->nodes[i].ox = ax + t * span;
    c->nodes[i].y = c->nodes[i].oy = ay;
    c->nodes[i].pinned = (i == 0 || i == N - 1);
  }
}

/* ── Preset 6: Storm ── */
/* Same hanging rope as preset 0, but the wind is cranked way up — strong
 * enough to beat gravity. At each gust the rope whips out almost sideways. */
static void preset_storm(Chain *c) {
  int N = N_NODES_DEF;
  float ax = (float)g_cols * CELL_W * 0.5f;
  float ay = (float)g_rows * CELL_H * 0.07f;
  float total = (float)g_rows * CELL_H * 0.75f;
  float rest = total / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_str = 580.f; /* at peak gust, stronger than gravity  */
  c->wind_on = true;
  c->preset = 6;

  for (int i = 0; i < N; i++) {
    c->nodes[i].x = c->nodes[i].ox = ax;
    c->nodes[i].y = c->nodes[i].oy = ay + (float)i * rest;
    c->nodes[i].pinned = (i == 0);
  }
}

/* ── Preset 7: Slingshot ── */
/* Like the pendulum, but hauled almost all the way up to one side before
 * release, so it swings through a huge arc. The pin sits low on screen to
 * leave room for the whole sweep. No wind — the big swing is the whole show. */
static void preset_slingshot(Chain *c) {
  int N = N_NODES_DEF;
  float ax = (float)g_cols * CELL_W * 0.5f;
  float ay = (float)g_rows * CELL_H * 0.60f; /* low, to leave room to swing */
  float total = (float)g_rows * CELL_H * 0.40f;
  float rest = total / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_on = false;
  c->preset = 7;

  float angle = 150.f * (float)M_PI / 180.f; /* hauled 150° round from straight-down — nearly straight up */
  float sin_a = sinf(angle), cos_a = cosf(angle);
  for (int i = 0; i < N; i++) {
    float t = (float)i * rest;
    c->nodes[i].x = c->nodes[i].ox = ax + sin_a * t;
    c->nodes[i].y = c->nodes[i].oy = ay + cos_a * t; /* this tilt points the rope upward */
    c->nodes[i].pinned = (i == 0);
  }
}

/* ── Preset 8: Flag ── */
/* Pinned at the top-left and laid out flat to the right, with a steady strong
 * wind. Gravity pulls it down while the wind pushes it sideways, and the rope
 * settles the tug-of-war into a curling, flapping banner. */
static void preset_flag(Chain *c) {
  int N = N_NODES_DEF;
  float pin_x = (float)g_cols * CELL_W * 0.10f;
  float pin_y = (float)g_rows * CELL_H * 0.40f;
  float total = (float)g_cols * CELL_W * 0.65f;
  float rest = total / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_str = 320.f; /* strong steady sideways wind          */
  c->wind_on = true;
  c->preset = 8;

  for (int i = 0; i < N; i++) {
    c->nodes[i].x = c->nodes[i].ox = pin_x + (float)i * rest;
    c->nodes[i].y = c->nodes[i].oy = pin_y;
    c->nodes[i].pinned = (i == 0);
  }
}

/* ── Preset 9: Double ── */
/* Pinned in two spots: the very top, and a hook a third of the way down. The
 * top third is a taut diagonal between them; the rest hangs free from the
 * hook, tilted to one side and released, and swings around it like a clock
 * arm. */
static void preset_double(Chain *c) {
  int N = N_NODES_DEF;
  int p_mid = N / 3;
  float anchor_x = (float)g_cols * CELL_W * 0.30f;
  float anchor_y = (float)g_rows * CELL_H * 0.18f;
  float hook_x = (float)g_cols * CELL_W * 0.55f;
  float hook_y = (float)g_rows * CELL_H * 0.35f;
  float upper_dx = hook_x - anchor_x;
  float upper_dy = hook_y - anchor_y;
  float upper_L = sqrtf(upper_dx * upper_dx + upper_dy * upper_dy);
  float lower_L = (float)g_rows * CELL_H * 0.45f;
  float rest = (upper_L + lower_L) / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_on = false;
  c->preset = 9;

  float lo_ang = (float)M_PI / 4.f; /* lean the hanging part 45° off straight-down */
  float lo_sx = sinf(lo_ang), lo_cx = cosf(lo_ang);
  for (int i = 0; i < N; i++) {
    if (i <= p_mid) {
      float t = (float)i / (float)p_mid;
      c->nodes[i].x = c->nodes[i].ox = anchor_x + t * upper_dx;
      c->nodes[i].y = c->nodes[i].oy = anchor_y + t * upper_dy;
    } else {
      float s = (float)(i - p_mid) * rest;
      c->nodes[i].x = c->nodes[i].ox = hook_x + lo_sx * s;
      c->nodes[i].y = c->nodes[i].oy = hook_y + lo_cx * s;
    }
    c->nodes[i].pinned = (i == 0 || i == p_mid);
  }
}

static void scene_init(int preset) {
  g_scene.chain.preset = preset;
  switch (preset) {
  case 0:
    preset_hanging(&g_scene.chain);
    break;
  case 1:
    preset_pendulum(&g_scene.chain);
    break;
  case 2:
    preset_bridge(&g_scene.chain);
    break;
  case 3:
    preset_wave(&g_scene.chain);
    break;
  case 4:
    preset_festoon(&g_scene.chain);
    break;
  case 5:
    preset_snake(&g_scene.chain);
    break;
  case 6:
    preset_storm(&g_scene.chain);
    break;
  case 7:
    preset_slingshot(&g_scene.chain);
    break;
  case 8:
    preset_flag(&g_scene.chain);
    break;
  case 9:
    preset_double(&g_scene.chain);
    break;
  }
}

/* ── §7 screen / HUD ── */

static void screen_init(void) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  color_init();
}

/* The top status bar: shows the current scene, theme, settings, and frame rate
 * in bright yellow along the top-right. */
static void draw_hud_top(const Chain *c, int fps) {
  char top[180];
  snprintf(top, sizeof top,
           " preset:%s  theme:%s  iter:%d  wind:%s  %s  %d fps ",
           k_preset_names[c->preset], k_themes[g_scene.theme].name, c->n_iter,
           c->wind_on ? "ON" : "OFF", c->paused ? "PAUSED " : "running", fps);
  int len = (int)strlen(top);
  int col = g_cols - len;
  if (col < 0)
    col = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, col, top, g_cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* The bottom key hints. On a narrow terminal the full list won't fit, so we
 * fall back to a shorter one rather than let it spill and wrap. */
static void draw_hud_bottom(void) {
  const char *hint_full = " q:quit  p:pause  r:reset  n/N:preset  t/T:theme  "
                          "+/-:iter  w:wind  ]/[:fps ";
  const char *hint_short = " q:quit  p:pause  r:reset  n:preset  t:theme ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= g_cols - 1)
    hint = hint_short;

  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(g_rows - 1, 0, hint, g_cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §8 app ── */

static volatile sig_atomic_t g_quit_flag = 0;
static volatile sig_atomic_t g_resize_flag = 0;

static void sig_h(int s) {
  if (s == SIGINT || s == SIGTERM)
    g_quit_flag = 1;
  if (s == SIGWINCH)
    g_resize_flag = 1;
}
static void do_cleanup(void) { endwin(); }

int main(void) {
  atexit(do_cleanup);
  signal(SIGINT, sig_h);
  signal(SIGTERM, sig_h);
  signal(SIGWINCH, sig_h);

  screen_init();
  getmaxyx(stdscr, g_rows, g_cols);

  /* Start on scene 0, the plain hanging rope. */
  scene_init(0);

  int64_t t_last = clock_ns();

  /* for measuring and showing the real frame rate */
  int64_t fps_acc = 0;
  int fps_cnt = 0;
  int fps_disp = 0;

  while (!g_quit_flag) {

    /* ── resize ── */
    if (g_resize_flag) {
      g_resize_flag = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, g_rows, g_cols);
      scene_init(g_scene.chain.preset);
      t_last = clock_ns();
      continue;
    }

    /* ── input ── */
    int ch;
    while ((ch = getch()) != ERR) {
      switch (ch) {
      case 'q':
      case 'Q':
      case 27:
        g_quit_flag = 1;
        break;

      case ' ':
      case 'p':
      case 'P':
        g_scene.chain.paused = !g_scene.chain.paused;
        break;

      case 'r':
      case 'R':
        scene_init(g_scene.chain.preset);
        break;

      case 'n':
        scene_init((g_scene.chain.preset + 1) % N_PRESETS);
        break;

      case 'N':
        scene_init((g_scene.chain.preset + N_PRESETS - 1) % N_PRESETS);
        break;

      case 't':
        g_scene.theme = (g_scene.theme + 1) % N_THEMES;
        theme_apply(g_scene.theme);
        break;

      case 'T':
        g_scene.theme = (g_scene.theme + N_THEMES - 1) % N_THEMES;
        theme_apply(g_scene.theme);
        break;

      case '+':
      case '=':
        if (g_scene.chain.n_iter < N_ITER_MAX)
          g_scene.chain.n_iter += 5;
        break;

      case '-':
        if (g_scene.chain.n_iter > N_ITER_MIN)
          g_scene.chain.n_iter -= 5;
        break;

      case 'w':
      case 'W':
        g_scene.chain.wind_on = !g_scene.chain.wind_on;
        break;

      case ']':
        if (g_scene.sim_fps < SIM_FPS_MAX)
          g_scene.sim_fps += SIM_FPS_STEP;
        break;

      case '[':
        if (g_scene.sim_fps > SIM_FPS_MIN)
          g_scene.sim_fps -= SIM_FPS_STEP;
        break;
      }
    }

    /* ── tick ── */
    int64_t t_now = clock_ns();
    int64_t t_used = t_now - t_last;
    t_last = t_now;
    /* Never let one frame count as more than 100 ms. If the program was paused
     * by the OS (Ctrl-Z, a flurry of resizes), the real gap could be seconds;
     * handing that to the physics would fling the beads off-screen in one jump. */
    if (t_used > 100000000LL)
      t_used = 100000000LL;

    float dt = (float)t_used * 1e-9f;
    chain_tick(&g_scene.chain, dt);

    /* ── draw ── */
    erase();
    chain_draw(&g_scene.chain, g_cols, g_rows);
    draw_hud_top(&g_scene.chain, fps_disp);
    draw_hud_bottom();
    wnoutrefresh(stdscr);
    doupdate();

    /* ── FPS cap ── */
    int64_t t_frame = TICK_NS(g_scene.sim_fps);
    int64_t t_sleep = t_frame - (clock_ns() - t_now);
    clock_sleep_ns(t_sleep);

    /* ── FPS counter ── */
    fps_acc += t_used;
    fps_cnt++;
    if (fps_acc >= NS_PER_SEC / 2) {
      fps_disp = fps_cnt * 2;
      fps_acc = 0;
      fps_cnt = 0;
    }
  }

  return 0;
}
