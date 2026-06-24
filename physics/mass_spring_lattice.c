/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * physics/mass_spring_lattice.c — a grid of masses joined by springs,
 * drawn so you only see the parts that are under strain.
 *
 * A spring sitting at its natural length is invisible; only springs that
 * are squashed or stretched glow.  At rest the screen is just a faint dot
 * grid (the masses sitting still).  Tap it and ripples of bright glyphs
 * spread out, bounce off the edges, cross each other, and slowly die down
 * as friction drains the motion.  It runs itself: four taps (center,
 * corner, two-point, and a line) take turns on about a 12-second loop.
 *
 * Keys:
 *   q / ESC  quit            space / p  pause          r  re-tap
 *   n  next pattern          t / T  cycle theme
 *   k / K  softer / stiffer springs    d / D  less / more friction
 *
 * The physics is the standard mass-spring setup from Witkin & Baraff,
 * "Physically Based Modeling" (SIGGRAPH notes, 2001); the only-H/V spring
 * layout and what you give up by skipping diagonals is Provot,
 * "Deformation Constraints in a Mass-Spring Model" (Graphics Interface
 * '95).  Why the velocity-first time step doesn't drift over thousands of
 * frames: Hairer/Lubich/Wanner, Geometric Numerical Integration (2006).
 * The wave patterns (rings, edge reflections, two-source interference)
 * are the membrane modes from Rayleigh, The Theory of Sound (1877) and
 * Crawford, Waves (Berkeley Physics Vol. 3, 1968).
 */

/* ── How it works ───────────────────────────────────────────────────
 *
 * Each dot is a little mass.  Springs join neighbours left-right and
 * up-down (no diagonals, which keeps the ripples clean and easy to
 * read).  Every frame we add up the pull of each spring on its two
 * masses, add a bit of friction that fights motion, then nudge each
 * mass: speed up first using the force we just found, then move using
 * the new speed.  Doing speed-first is the trick that keeps the whole
 * thing from slowly gaining fake energy and flying apart over time.
 *
 * How stretched or squashed a spring is decides how it's drawn.  We
 * measure "strain" = how far its length is from its resting length, as
 * a fraction.  Nearly-resting springs aren't drawn at all, so only the
 * ripples show:
 *
 *     strain (size)   glyph     look
 *     near 0          (none)    invisible — at rest
 *     small           - or |    dim
 *     medium          = or H    bright
 *     large           #         bright + bold
 *
 * Colour says which way: a cool colour means squashed, a warm colour
 * means stretched, so you can tell compression from tension at a glance.
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config ── */

enum {
  TARGET_FPS = 60,
  SUBSTEPS = 2, /* physics steps per drawn frame */

  NODE_DX = 4, /* gap between neighbours, in screen columns */
  NODE_DY = 2, /* gap between neighbours, in screen rows */
  MAX_NX = 40,
  MAX_NY = 18,
  MAX_NODES = MAX_NX * MAX_NY,
  MAX_SPRINGS = MAX_NX * MAX_NY * 2,

  SCENARIO_COUNT = 4,
  SCENARIO_TICKS = 12 * TARGET_FPS, /* hold each pattern ~12 s */

  N_THEMES = 3,
};

#define MASS_DEF 1.0f
#define K_DEF 60.0f
#define DAMPING_DEF 1.5f
#define IMPULSE_VEL 14.0f
#define HAMMER_VEL 30.0f

#define K_STEP 5.0f
#define K_MIN 10.0f
#define K_MAX 300.0f
#define DAMPING_STEP 0.25f
#define DAMPING_MIN 0.0f
#define DAMPING_MAX 8.0f

/* How much strain a spring needs before it's drawn, and the cutoffs
 * between dim / bright / bold. */
#define STRAIN_NULL 0.04f /* under this the spring is left invisible */
#define STRAIN_MID 0.15f  /* dim below, bright above                 */
#define STRAIN_HIGH 0.35f /* bright below, bold above                */

/* How fast a mass must move to show as 'o' (moving) then 'O' (just hit). */
#define SPEED_REST 0.6f
#define SPEED_FAST 5.0f

#define HUD_TOP_ROWS 1
#define HUD_BOT_ROWS 1

/* ── §2 clock ── */

#define NS_PER_SEC 1000000000LL
#define TICK_NS(fps) (NS_PER_SEC / (fps))

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

/* ── §3 color / theme ──
 *
 * Two colours are fixed (the bright-yellow status line and bright-cyan
 * key hints).  Everything else comes from the active theme: a cool pair
 * for squashed springs, a warm pair for stretched ones, and three
 * brightnesses for the moving masses. */

enum {
  CP_HUD = 1,
  CP_HINT = 2,
  CP_COMPRESS_LO = 3,
  CP_COMPRESS_HI = 4,
  CP_TENSION_LO = 5,
  CP_TENSION_HI = 6,
  CP_NODE_REST = 7,
  CP_NODE_SLOW = 8,
  CP_NODE_FAST = 9,
  CP_NODE_PIN = 10,
};

/*
 * Theme — one named look: all the colours that change together when you
 * cycle the palette, kept in a single struct.
 *
 * The point of bundling them is that they have to agree.  Squashed and
 * stretched springs each get a two-step ramp (mild then strong), and the
 * moving masses get three brightnesses (still / moving / just-hit).
 * Within each ramp the brighter colour must mean "more": if a theme
 * accidentally paired the strong colour of one theme with the mild of
 * another, brightness would stop meaning "more strain".  Holding them
 * side by side makes a mismatch obvious when you edit the table.
 *
 * Each theme carries two copies of its palette.  Most terminals can show
 * 256 colours and get the nice version; very old or minimal ones can only
 * show 8, so there's a coarse fallback that keeps cool-vs-warm even if it
 * can't keep mild-vs-strong.  theme_apply() picks which copy to use by
 * asking ncurses how many colours it has.
 *
 * Two colours are deliberately NOT in here: the status line and key
 * hints (fixed bright yellow + cyan so they stay readable over any
 * theme), and the pinned-mass marker (always white-on-blue so anchors
 * look the same everywhere).  Those are set once and never re-themed.
 *
 * Every 256-colour value is kept in the bright half of the palette;
 * the dark end vanishes against a black background.
 */
typedef struct {
  /* The 256-colour palette (used when the terminal supports it).
   * Cool pair = squashed spring, warm pair = stretched; each goes
   * mild then strong. */
  short compress_lo; /* squashed, mild   */
  short compress_hi; /* squashed, strong */
  short tension_lo;  /* stretched, mild   */
  short tension_hi;  /* stretched, strong */

  /* Brightness of a mass by how fast it's moving — must climb still ->
   * moving -> just-hit so a moving mass simply looks brighter. */
  short node_rest; /* sitting still (the dot grid) */
  short node_slow; /* moving                       */
  short node_fast; /* just struck                  */

  /* The same colours for 8-colour terminals: same meanings, less detail
   * (mild and strong often collapse to one colour). */
  short compress_lo8, compress_hi8;
  short tension_lo8, tension_hi8;
  short node_rest8, node_slow8, node_fast8;

  const char *name; /* shown after "theme:" in the status line */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* Classic — cyan squashed, red stretched */
    {51, 195, 220, 196, 245, 46, 207, COLOR_CYAN, COLOR_WHITE, COLOR_YELLOW,
     COLOR_RED, COLOR_WHITE, COLOR_GREEN, COLOR_MAGENTA, "Classic"},
    /* Cold — blue/cyan squashed, white-hot stretched */
    {39, 87, 255, 226, 244, 51, 213, COLOR_BLUE, COLOR_CYAN, COLOR_WHITE,
     COLOR_YELLOW, COLOR_WHITE, COLOR_CYAN, COLOR_MAGENTA, "Cold"},
    /* Plasma — violet squashed, gold stretched */
    {99, 207, 214, 196, 246, 156, 213, COLOR_MAGENTA, COLOR_WHITE, COLOR_YELLOW,
     COLOR_RED, COLOR_WHITE, COLOR_GREEN, COLOR_MAGENTA, "Plasma"},
};

/* The status line and key hints never re-theme, so set them once here. */
static void color_init(void) {
  start_color();
  use_default_colors();

  if (COLORS >= 256) {
    init_pair(CP_HUD, 226, -1);
    init_pair(CP_HINT, 51, -1);
  } else {
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
}

static void theme_apply(int t) {
  const Theme *th = &k_themes[t];
  if (COLORS >= 256) {
    init_pair(CP_COMPRESS_LO, th->compress_lo, -1);
    init_pair(CP_COMPRESS_HI, th->compress_hi, -1);
    init_pair(CP_TENSION_LO, th->tension_lo, -1);
    init_pair(CP_TENSION_HI, th->tension_hi, -1);
    init_pair(CP_NODE_REST, th->node_rest, -1);
    init_pair(CP_NODE_SLOW, th->node_slow, -1);
    init_pair(CP_NODE_FAST, th->node_fast, -1);
  } else {
    init_pair(CP_COMPRESS_LO, th->compress_lo8, -1);
    init_pair(CP_COMPRESS_HI, th->compress_hi8, -1);
    init_pair(CP_TENSION_LO, th->tension_lo8, -1);
    init_pair(CP_TENSION_HI, th->tension_hi8, -1);
    init_pair(CP_NODE_REST, th->node_rest8, -1);
    init_pair(CP_NODE_SLOW, th->node_slow8, -1);
    init_pair(CP_NODE_FAST, th->node_fast8, -1);
  }
  init_pair(CP_NODE_PIN, COLOR_WHITE, COLOR_BLUE);
}

/* ── §4 lattice — the masses, the springs, and the box that holds them ──
 *
 * Three types here: a Node (one mass), a Spring (one connector), and a
 * tag for which way a spring points.  The big Scene struct lower down
 * owns all of them.  This is the textbook layout: each mass carries its
 * own position, speed, and running force total; each spring just stores
 * the two masses it links. */

/*
 * SpringKind — which way a spring runs: across (horizontal) or down
 * (vertical).
 *
 * We store this even though you could work it out from the two ends.
 * The drawing code reads it every frame to pick the glyph ('-' for
 * across, '|' for down), and reading a stored tag is cheaper than
 * re-checking both endpoints each time.  The physics ignores it — a
 * spring is a spring.
 *
 * Only across and down springs exist; there are no diagonals.  Diagonals
 * would make the grid stiffer (the way real cloth resists shearing), but
 * leaving them out keeps the ripples sharp and easy to read, which is the
 * whole point.  Adding them back would be a one-line change in
 * springs_build().
 */
typedef enum {
  SPRING_H = 0, /* across: drawn as '-' */
  SPRING_V = 1, /* down:   drawn as '|' */
} SpringKind;

/*
 * Node — one little mass in the grid.
 *
 * It keeps two positions: where it is right now, and where it belongs
 * when everything's quiet (its home spot).  We keep the home spot so we
 * can snap the grid back to calm between taps, and so the faint dot grid
 * is drawn in the right place even while a mass is off riding a ripple.
 *
 * It also keeps a running force total that gets rebuilt every step: we
 * zero it, add up the spring pulls, then subtract a bit for friction.
 * Keeping it on the mass lets those two passes pile into the same spot
 * instead of needing a scratch buffer.
 *
 * The fields are laid out in the order the inner loops touch them, so one
 * mass sits in about one cache line.
 */
typedef struct {
  float x, y;   /* where it is now (fractional so motion is smooth) */
  float rx, ry; /* its home spot; set once at build, never changed   */
  float vx, vy; /* how fast it's moving                              */
  float fx, fy; /* force piling up this step (springs, then friction) */
  bool pinned;  /* if set, this mass never moves — a fixed anchor.
                 * The built-in patterns don't use it. */
} Node;

/*
 * Spring — one connector joining two masses.
 *
 * Each step it pulls its two ends back toward its resting length: pull
 * harder the further it's off, and in opposite directions on the two
 * ends.  That's Hooke's law (the force in the famous spring equation).
 * A pinned end feels the pull but doesn't move; it still counts toward
 * how stretched the spring looks, so anchors still glow.
 *
 * It remembers its current strain (how far off its resting length it is,
 * as a fraction) because the force math has to work that out anyway, and
 * the drawing code wants exactly that number to pick a colour and glyph —
 * so caching it saves the drawing pass from redoing a square root every
 * frame.
 *
 * Each spring also carries its own resting length rather than looking it
 * up from its direction. They're equal today (across springs rest at the
 * column gap, down springs at the row gap), but per-spring is the obvious
 * home if a future tweak ever wants varied lengths.
 *
 * The two ends are stored as positions in the masses array, not pointers,
 * because that array gets rebuilt on resize and pointers wouldn't survive
 * it. By the way they're built, the smaller index always comes first.
 */
typedef struct {
  int a, b;        /* the two masses it links (positions in the array) */
  float rest_len;  /* the length it wants to be                        */
  float strain;    /* how far off that length, as a fraction:
                    * negative = squashed, positive = stretched.
                    * Set by the physics, read by the drawing code. */
  SpringKind kind; /* across or down — picks the glyph                 */
} Spring;

/*
 * Scene — all the state for one running session, in one place.
 *
 * Everything the program can change lives here, so any helper just takes
 * a Scene* and there's no hidden global it secretly reads.  On a resize
 * or reset the whole thing is wiped and rebuilt at once, so no stale
 * leftovers can sneak between rebuilds.
 *
 * The fields fall into four groups, listed in roughly the order each
 * frame touches them:
 *   (A) the actual masses and springs, plus how many are in use
 *   (B) the grid's size and where it sits on screen (only changes on
 *       resize)
 *   (C) the physics knobs you can turn at runtime (stiffness, friction…)
 *   (D) which pattern is playing, the theme, paused, the shown fps
 * Keeping physics (A, C) apart from the on-screen stuff (D) means a bug
 * in one can't quietly corrupt the other.
 *
 * It's about 80 KB, almost all the masses and springs, and lives as one
 * file-scope variable so nothing is malloc'd while running.
 */
typedef struct {
  /* (A) the masses and springs themselves. nn / ns say how many of each
   * are actually in use; anything past those is leftover junk. */
  Node nodes[MAX_NODES];
  Spring springs[MAX_SPRINGS];
  int nn; /* masses in use (= nx * ny) */
  int ns; /* springs in use            */

  /* (B) the grid's shape and where it sits. Set when the grid is built
   * and left alone until the next resize. */
  int nx, ny;               /* masses across, masses down            */
  int x0, y0;               /* screen spot of the top-left mass       */
  int term_cols, term_rows; /* current terminal size                  */

  /* (C) physics knobs. The keyboard changes stiffness and friction;
   * turning them only changes the forces, not the grid layout. */
  float k;       /* spring stiffness        */
  float damping; /* how strongly friction fights motion */
  float mass;    /* weight of each mass      */
  float dt;      /* length of one physics step, in seconds */
  int sim_fps;   /* frames per second target */
  int substeps;  /* physics steps per frame  */

  /* (D) what's on screen. None of the physics reads these. */
  int scenario; /* which tap pattern is playing */
  int sc_tick;  /* frames since it started      */
  int theme;    /* which palette                */
  bool paused;  /* frozen but still drawing     */
  int fps_disp; /* fps shown in the status line */
} Scene;

/* The one and only scene. It's file-scope (rather than a local in main)
 * only because the signal handlers need to reach it too. */
static Scene g_scene;

/* ── §5 physics ── */

/* Work out the screen spot for the top-left mass so the whole grid lands
 * centered between the two HUD rows. Won't let it slide off a tiny
 * terminal. */
static void compute_lattice_anchor(Scene *s, int nx, int ny) {
  int lat_w = (nx - 1) * NODE_DX;
  int lat_h = (ny - 1) * NODE_DY;
  s->x0 = (s->term_cols - lat_w) / 2;
  s->y0 =
      HUD_TOP_ROWS + ((s->term_rows - HUD_TOP_ROWS - HUD_BOT_ROWS - lat_h) / 2);
  if (s->x0 < 0)
    s->x0 = 0;
  if (s->y0 < HUD_TOP_ROWS)
    s->y0 = HUD_TOP_ROWS;
}

/* Place one mass at its home spot, sitting perfectly still. */
static void stamp_rest_node(Node *n, int x0, int y0, int c, int r) {
  n->rx = (float)(x0 + c * NODE_DX);
  n->ry = (float)(y0 + r * NODE_DY);
  n->x = n->rx;
  n->y = n->ry;
  n->vx = 0.0f;
  n->vy = 0.0f;
  n->fx = 0.0f;
  n->fy = 0.0f;
  n->pinned = false;
}

/* Build the grid of masses at rest. The springs are wired up separately
 * (springs_build) so a variant could change the wiring without touching
 * this. */
static void lattice_init(Scene *s, int nx, int ny) {
  s->nx = nx;
  s->ny = ny;
  s->nn = nx * ny;
  s->ns = 0;

  compute_lattice_anchor(s, nx, ny);

  for (int r = 0; r < ny; r++) {
    for (int c = 0; c < nx; c++) {
      stamp_rest_node(&s->nodes[r * nx + c], s->x0, s->y0, c, r);
    }
  }
}

/* Join each mass to its right and down neighbour (the ones that exist),
 * which covers every left-right and up-down link exactly once. */
static void springs_build(Scene *s) {
  s->ns = 0;
  float h_rest = (float)NODE_DX;
  float v_rest = (float)NODE_DY;
  for (int r = 0; r < s->ny; r++) {
    for (int c = 0; c < s->nx; c++) {
      int i = r * s->nx + c;
      if (c + 1 < s->nx) {
        s->springs[s->ns++] = (Spring){i, i + 1, h_rest, 0.0f, SPRING_H};
      }
      if (r + 1 < s->ny) {
        s->springs[s->ns++] = (Spring){i, i + s->nx, v_rest, 0.0f, SPRING_V};
      }
    }
  }
}

/* Wipe each mass's force back to zero so this step's pulls add up fresh. */
static void clear_force_accumulators(Scene *s) {
  for (int i = 0; i < s->nn; i++) {
    s->nodes[i].fx = 0.0f;
    s->nodes[i].fy = 0.0f;
  }
}

/* For one spring: measure how far off its resting length it is, push its
 * two ends back toward that length (harder the further off, opposite ways
 * on the two ends), and remember the strain so the drawing code can read
 * it. If the two ends sit exactly on top of each other there's no "which
 * way to push", so we skip the push for that frame. */
static void accumulate_hooke_pair(Scene *s, Spring *sp) {
  Node *a = &s->nodes[sp->a];
  Node *b = &s->nodes[sp->b];
  float dx = b->x - a->x;
  float dy = b->y - a->y;
  float dist = sqrtf(dx * dx + dy * dy);
  if (dist < 1e-4f) {
    sp->strain = 0.0f;
    return;
  }

  sp->strain = (dist - sp->rest_len) / sp->rest_len;

  float inv_d = 1.0f / dist;
  float stretch = dist - sp->rest_len;
  float fx = s->k * stretch * dx * inv_d;
  float fy = s->k * stretch * dy * inv_d;

  if (!a->pinned) {
    a->fx += fx;
    a->fy += fy;
  }
  if (!b->pinned) {
    b->fx -= fx;
    b->fy -= fy;
  }
}

/* Apply every spring's pull. This is the bulk of the work each step. */
static void apply_hooke_spring_forces(Scene *s) {
  for (int i = 0; i < s->ns; i++) {
    accumulate_hooke_pair(s, &s->springs[i]);
  }
}

/* Add a little friction: a drag that always opposes motion, scaled by how
 * fast the mass is going. It's what makes the ripples fade out and go
 * quiet. There's no gravity here, so this is the only thing draining
 * energy. */
static void apply_velocity_damping(Scene *s) {
  for (int i = 0; i < s->nn; i++) {
    if (s->nodes[i].pinned)
      continue;
    s->nodes[i].fx -= s->damping * s->nodes[i].vx;
    s->nodes[i].fy -= s->damping * s->nodes[i].vy;
  }
}

/* The full force tally for one step: start at zero, add the springs, add
 * the friction. The move step then uses these totals. */
static void compute_forces(Scene *s) {
  clear_force_accumulators(s);
  apply_hooke_spring_forces(s);
  apply_velocity_damping(s);
}

/* Move everything one step. Speed up first (from the force we just
 * tallied), then move using the new speed. Doing it in that order is what
 * keeps the grid from slowly gaining fake energy and flying apart. */
static void integrate_step(Scene *s) {
  float inv_m = 1.0f / s->mass;
  float dt = s->dt;
  for (int i = 0; i < s->nn; i++) {
    if (s->nodes[i].pinned)
      continue;
    s->nodes[i].vx += s->nodes[i].fx * inv_m * dt;
    s->nodes[i].vy += s->nodes[i].fy * inv_m * dt;
    s->nodes[i].x += s->nodes[i].vx * dt;
    s->nodes[i].y += s->nodes[i].vy * dt;
  }
}

/* Check whether the numbers have gone haywire (any position or speed not
 * a real value). Happens if springs get too stiff for the step size; the
 * caller resets when this returns true. */
static bool lattice_blown_up(const Scene *s) {
  for (int i = 0; i < s->nn; i++) {
    if (!isfinite(s->nodes[i].x) || !isfinite(s->nodes[i].y) ||
        !isfinite(s->nodes[i].vx) || !isfinite(s->nodes[i].vy))
      return true;
  }
  return false;
}

/* ── §6 scenarios — the four taps that take turns ──
 *
 * Each one calms the grid first, then gives some masses a shove. Three of
 * them are a round shove (a "ring" outward); the fourth shakes a whole
 * column. They look like:
 *   center    — a clean ring spreading from the middle
 *   corner    — a quarter-ring that bounces off the two near edges
 *   two-point — two rings that cross and form bright/dark bands
 *   line      — a flat wave marching sideways
 */

static const char *const k_scenario_names[SCENARIO_COUNT] = {
    "Center", "Corner", "TwoPoint", "Line"};

/* Settle the whole grid back to rest. Run before each tap so leftover
 * motion from the last one doesn't bleed in. */
static void scenario_quiet(Scene *s) {
  for (int i = 0; i < s->nn; i++) {
    s->nodes[i].x = s->nodes[i].rx;
    s->nodes[i].y = s->nodes[i].ry;
    s->nodes[i].vx = 0.0f;
    s->nodes[i].vy = 0.0f;
    s->nodes[i].fx = 0.0f;
    s->nodes[i].fy = 0.0f;
    s->nodes[i].pinned = false;
  }
  for (int idx = 0; idx < s->ns; idx++)
    s->springs[idx].strain = 0.0f;
}

/* Shove every mass within `radius` of a center point outward, gently
 * nearer the edge of the circle and hardest at the middle. That outward
 * shove is what becomes an expanding ring. */
static void scenario_strike_radial(Scene *s, int cr, int cc, float speed,
                                   int radius) {
  if (cr < 0 || cr >= s->ny || cc < 0 || cc >= s->nx)
    return;
  for (int r = cr - radius; r <= cr + radius; r++) {
    for (int c = cc - radius; c <= cc + radius; c++) {
      if (r < 0 || r >= s->ny || c < 0 || c >= s->nx)
        continue;
      float dr = (float)(r - cr), dc = (float)(c - cc);
      float d2 = dr * dr + dc * dc;
      if (d2 > (float)(radius * radius))
        continue;
      float fall = 1.0f - sqrtf(d2) / (float)(radius + 1);
      float angle = (d2 < 1e-4f) ? 0.0f : atan2f(dr, dc);
      s->nodes[r * s->nx + c].vx = speed * fall * cosf(angle);
      s->nodes[r * s->nx + c].vy = speed * fall * sinf(angle);
    }
  }
}

/* The shorter side of the grid. Strike sizes scale off this so a tap
 * looks the same whether the grid is wide or tall. */
static int lattice_minor_axis(const Scene *s) {
  return (s->nx < s->ny ? s->nx : s->ny);
}

/* One shove in the middle: a clean ring spreads outward. */
static void strike_center_pulse(Scene *s) {
  int radius = lattice_minor_axis(s) / 5;
  scenario_strike_radial(s, s->ny / 2, s->nx / 2, HAMMER_VEL, radius);
}

/* A shove up in a corner: the ring is only a quarter and immediately
 * bounces off the two nearby edges, so the reflections cross. */
static void strike_corner_pulse(Scene *s) {
  int radius = lattice_minor_axis(s) / 6;
  scenario_strike_radial(s, 1, 1, HAMMER_VEL * 1.2f, radius);
}

/* Two shoves, left and right. Where the two rings meet they reinforce
 * into bright bands and cancel into dark gaps. */
static void strike_twopoint_interference(Scene *s) {
  int radius = lattice_minor_axis(s) / 7;
  int mid_r = s->ny / 2;
  scenario_strike_radial(s, mid_r, s->nx / 4, HAMMER_VEL * 0.9f, radius);
  scenario_strike_radial(s, mid_r, s->nx - s->nx / 4, HAMMER_VEL * 0.9f,
                         radius);
}

/* Shake one whole column up and down in a smooth wave shape, so a flat
 * wave marches sideways across the grid. */
static void strike_line_planar(Scene *s) {
  int col = s->nx / 3;
  for (int r = 0; r < s->ny; r++) {
    float phase = (float)r / (float)(s->ny - 1) * 2.0f * (float)M_PI;
    s->nodes[r * s->nx + col].vy = IMPULSE_VEL * sinf(phase);
  }
}

/* Calm the grid, then run the tap for the given pattern number. */
static void scenario_apply(Scene *s, int id) {
  scenario_quiet(s);
  s->scenario = id;
  s->sc_tick = 0;

  switch (id) {
  case 0:
    strike_center_pulse(s);
    break;
  case 1:
    strike_corner_pulse(s);
    break;
  case 2:
    strike_twopoint_interference(s);
    break;
  case 3:
    strike_line_planar(s);
    break;
  }
}

/* ── §7 render ── */

/* Turn a spring's strain into a colour, glyph, and bold flag. Returns
 * false for a near-resting spring, which is the "leave it invisible" case
 * that makes the ripples stand out. */
static bool strain_visual(float strain, SpringKind kind, int *out_cp,
                          char *out_glyph, bool *out_bold) {
  float a = fabsf(strain);
  if (a < STRAIN_NULL)
    return false;

  bool compress = (strain < 0.0f);
  if (a < STRAIN_MID) {
    *out_cp = compress ? CP_COMPRESS_LO : CP_TENSION_LO;
    *out_glyph = (kind == SPRING_H) ? '-' : '|';
    *out_bold = false;
  } else if (a < STRAIN_HIGH) {
    *out_cp = compress ? CP_COMPRESS_HI : CP_TENSION_HI;
    *out_glyph = (kind == SPRING_H) ? '=' : 'H';
    *out_bold = false;
  } else {
    *out_cp = compress ? CP_COMPRESS_HI : CP_TENSION_HI;
    *out_glyph = '#';
    *out_bold = true;
  }
  return true;
}

/* Draw one character, but only if it lands inside the play area (not off
 * screen, not on the HUD rows). Everything draws through here so the
 * "stay in bounds" check lives in one spot. */
static void paint_cell_clipped(const Scene *s, int row, int col, char glyph,
                               int attr) {
  if (col < 0 || col >= s->term_cols)
    return;
  if (row < HUD_TOP_ROWS || row >= s->term_rows - HUD_BOT_ROWS)
    return;
  mvaddch(row, col, (chtype)(unsigned char)glyph | attr);
}

/* Fill the cells between two masses with the spring's glyph, skipping the
 * two ends (those belong to the masses). We step evenly along the line
 * rather than using the usual line-drawing trick, because these springs
 * are very short and the even spacing keeps them from clumping. */
static void paint_spring_interior(const Scene *s, int x0, int y0, int x1,
                                  int y1, int steps, char glyph, int attr) {
  for (int k = 1; k < steps; k++) {
    float t = (float)k / (float)steps;
    int cx = (int)(x0 + (x1 - x0) * t + 0.5f);
    int cy = (int)(y0 + (y1 - y0) * t + 0.5f);
    paint_cell_clipped(s, cy, cx, glyph, attr);
  }
}

/* Draw one spring as a glowing line, but only if it's stressed enough to
 * show. (The bad-number check guards the frame right after a blow-up.) */
static void render_spring(const Scene *s, const Spring *sp) {
  int cp;
  char glyph;
  bool bold;
  if (!strain_visual(sp->strain, sp->kind, &cp, &glyph, &bold))
    return;

  const Node *a = &s->nodes[sp->a];
  const Node *b = &s->nodes[sp->b];
  if (!isfinite(a->x) || !isfinite(b->x))
    return;

  int x0 = (int)(a->x + 0.5f), y0 = (int)(a->y + 0.5f);
  int x1 = (int)(b->x + 0.5f), y1 = (int)(b->y + 0.5f);

  int steps = (sp->kind == SPRING_H) ? NODE_DX : NODE_DY;
  int attr = COLOR_PAIR(cp) | (bold ? A_BOLD : 0);

  paint_spring_interior(s, x0, y0, x1, y1, steps, glyph, attr);
}

/* Pick how a mass looks based on how fast it's moving:
 *   pinned        '@'  a fixed anchor
 *   barely moving '.'  the resting dot grid
 *   moving        'o'
 *   just struck   'O'  (bold)
 */
static void node_visual_from_speed(const Node *n, char *out_glyph, int *out_cp,
                                   int *out_attr) {
  if (n->pinned) {
    *out_glyph = '@';
    *out_cp = CP_NODE_PIN;
    *out_attr = A_BOLD;
    return;
  }
  float speed = sqrtf(n->vx * n->vx + n->vy * n->vy);
  if (speed < SPEED_REST) {
    *out_glyph = '.';
    *out_cp = CP_NODE_REST;
    *out_attr = 0;
  } else if (speed < SPEED_FAST) {
    *out_glyph = 'o';
    *out_cp = CP_NODE_SLOW;
    *out_attr = 0;
  } else {
    *out_glyph = 'O';
    *out_cp = CP_NODE_FAST;
    *out_attr = A_BOLD;
  }
}

/* Draw one mass at where it is now. (Bad-number check guards post-blow-up
 * frames.) */
static void render_node(const Scene *s, const Node *n) {
  if (!isfinite(n->x) || !isfinite(n->y))
    return;
  int cx = (int)(n->x + 0.5f);
  int cy = (int)(n->y + 0.5f);

  char glyph;
  int cp;
  int attr;
  node_visual_from_speed(n, &glyph, &cp, &attr);

  paint_cell_clipped(s, cy, cx, glyph, COLOR_PAIR(cp) | attr);
}

/* Compose: springs (background, only the stressed ones glow) then
 * nodes (foreground, always visible — the lattice's skeleton). */
static void render_lattice(const Scene *s) {
  for (int i = 0; i < s->ns; i++)
    render_spring(s, &s->springs[i]);
  for (int i = 0; i < s->nn; i++)
    render_node(s, &s->nodes[i]);
}

/* A single number for "how much is the grid still ringing" — the motion
 * of the masses plus the strain stored in the springs. Shown in the HUD
 * so you can watch it fall as things settle. */
static float total_energy(const Scene *s) {
  float ke = 0.0f, pe = 0.0f;
  for (int i = 0; i < s->nn; i++) {
    float v2 =
        s->nodes[i].vx * s->nodes[i].vx + s->nodes[i].vy * s->nodes[i].vy;
    ke += 0.5f * s->mass * v2;
  }
  for (int i = 0; i < s->ns; i++) {
    float stretch = s->springs[i].strain * s->springs[i].rest_len;
    pe += 0.5f * s->k * stretch * stretch;
  }
  return ke + pe;
}

static void render_hud(const Scene *s) {
  /* Top row: status line, bright yellow. */
  char buf[160];
  snprintf(buf, sizeof buf,
           " MassSpring  scenario:%-8s  theme:%-7s  k=%-4.0f  damp=%-3.1f"
           "  E=%-7.1f  %2dfps  %s",
           k_scenario_names[s->scenario], k_themes[s->theme].name, s->k,
           s->damping, total_energy(s), s->fps_disp,
           s->paused ? "PAUSED " : "running");
  move(0, 0);
  clrtoeol();
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvprintw(0, 0, "%s", buf);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* Bottom row: key hints, bright cyan. */
  int bot = s->term_rows - 1;
  move(bot, 0);
  clrtoeol();
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(bot, 0,
           " q:quit  spc:pause  r:reset  n:next  t/T:theme"
           "  k/K:stiff  d/D:damp ");
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §8 main — setup, the per-frame steps, and the loop ── */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit = 0;

static void on_sigwinch(int s) {
  (void)s;
  g_resize = 1;
}
static void on_sigterm(int s) {
  (void)s;
  g_quit = 1;
}

/* Pick how many masses across and down fit the current terminal, within
 * sane minimums and the array's maximum. */
static void fit_lattice(Scene *s) {
  int avail_w = s->term_cols - 4;
  int avail_h = s->term_rows - HUD_TOP_ROWS - HUD_BOT_ROWS - 2;
  int nx = avail_w / NODE_DX + 1;
  int ny = avail_h / NODE_DY + 1;
  if (nx > MAX_NX)
    nx = MAX_NX;
  if (ny > MAX_NY)
    ny = MAX_NY;
  if (nx < 4)
    nx = 4;
  if (ny < 3)
    ny = 3;
  s->nx = nx;
  s->ny = ny;
}

/* Rebuild everything from scratch — after a resize or a blow-up. Resize
 * the grid, place the masses, re-wire the springs, replay the current
 * tap. */
static void rebuild_for_terminal(Scene *s) {
  fit_lattice(s);
  lattice_init(s, s->nx, s->ny);
  springs_build(s);
  scenario_apply(s, s->scenario);
}

/* Set the starting knobs and UI state. The grid itself gets filled in by
 * rebuild_for_terminal right after. */
static void scene_defaults(Scene *s) {
  s->k = K_DEF;
  s->damping = DAMPING_DEF;
  s->mass = MASS_DEF;
  s->sim_fps = TARGET_FPS;
  s->substeps = SUBSTEPS;
  s->dt = 1.0f / (float)(s->sim_fps * s->substeps);

  s->scenario = 0;
  s->sc_tick = 0;
  s->theme = 0;
  s->paused = false;
  s->fps_disp = s->sim_fps;
}

/* ── startup helpers ── */

/* Catch Ctrl-C / kill (quit) and terminal-resize. The handlers just set a
 * flag; the loop does the real work when it's safe. */
static void install_signal_handlers(void) {
  signal(SIGWINCH, on_sigwinch);
  signal(SIGTERM, on_sigterm);
  signal(SIGINT, on_sigterm);
}

/* Put the terminal into game mode: keys come straight through, nothing
 * echoes, no cursor, input never blocks. typeahead(-1) stops ncurses from
 * pausing the screen update to peek at the keyboard. */
static void terminal_setup(void) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
}

/* Step a value forward or back by one and wrap around the ends. The +n
 * keeps it positive so stepping back from 0 lands on the last entry. */
static void cycle_index(int *value, int step, int n) {
  *value = (*value + n + step) % n;
}

/* Keep a value from straying outside its min/max. */
static void clamp_float(float *v, float lo, float hi) {
  if (*v < lo)
    *v = lo;
  if (*v > hi)
    *v = hi;
}

/* ── one frame, step by step ── */

/* If the terminal was resized, grab the new size and rebuild the grid to
 * fit (its size depends on the terminal). */
static void consume_resize_event(Scene *s) {
  if (!g_resize)
    return;
  g_resize = 0;
  endwin();
  refresh();
  getmaxyx(stdscr, s->term_rows, s->term_cols);
  rebuild_for_terminal(s);
}

/* Handle every key waiting in the buffer. */
static void process_input(Scene *s) {
  int ch;
  while ((ch = getch()) != ERR) {
    switch (ch) {
    case 'q':
    case 27: /* 27 = ESC */
      g_quit = 1;
      break;
    case ' ':
    case 'p': /* pause toggle */
      s->paused = !s->paused;
      break;
    case 'r': /* re-trigger current scenario */
      scenario_apply(s, s->scenario);
      break;
    case 'n': /* next scenario */
      cycle_index(&s->scenario, +1, SCENARIO_COUNT);
      scenario_apply(s, s->scenario);
      break;
    case 't': /* next theme */
      cycle_index(&s->theme, +1, N_THEMES);
      theme_apply(s->theme);
      break;
    case 'T': /* previous theme */
      cycle_index(&s->theme, -1, N_THEMES);
      theme_apply(s->theme);
      break;
    case 'k': /* softer springs */
      s->k -= K_STEP;
      clamp_float(&s->k, K_MIN, K_MAX);
      break;
    case 'K': /* stiffer springs */
      s->k += K_STEP;
      clamp_float(&s->k, K_MIN, K_MAX);
      break;
    case 'd': /* less damping (rings longer) */
      s->damping -= DAMPING_STEP;
      clamp_float(&s->damping, DAMPING_MIN, DAMPING_MAX);
      break;
    case 'D': /* more damping (decays faster) */
      s->damping += DAMPING_STEP;
      clamp_float(&s->damping, DAMPING_MIN, DAMPING_MAX);
      break;
    }
  }
}

/* Once a pattern has had its turn, move on to the next so the show keeps
 * changing on its own. */
static void auto_advance_scenario(Scene *s) {
  s->sc_tick++;
  if (s->sc_tick < SCENARIO_TICKS)
    return;
  cycle_index(&s->scenario, +1, SCENARIO_COUNT);
  scenario_apply(s, s->scenario);
}

/* Advance the physics a few small steps, recover if it blew up, and let
 * the show auto-advance. Does nothing while paused. */
static void step_physics(Scene *s) {
  if (s->paused)
    return;

  for (int sub = 0; sub < s->substeps; sub++) {
    compute_forces(s);
    integrate_step(s);
  }
  if (lattice_blown_up(s)) {
    /* springs got too stiff and the numbers went wild — start over */
    rebuild_for_terminal(s);
  }
  auto_advance_scenario(s);
}

/* Draw one frame: wipe, then the grid, then the HUD on top. The two-step
 * flush sends only what changed, which avoids flicker on slow terminals. */
static void render_frame(const Scene *s) {
  erase();
  render_lattice(s);
  render_hud(s);
  wnoutrefresh(stdscr);
  doupdate();
}

/* Hold a steady frame rate: sleep off whatever time is left in this
 * frame's budget, then read the clock again as the start of the next one.
 * Also hands back how long the work took so the fps counter needn't re-read
 * the clock. */
static int64_t pace_frame_to_fps(int64_t t_last, int sim_fps,
                                 int64_t *out_work_ns) {
  int64_t t_now = clock_ns();
  int64_t t_work = t_now - t_last;
  int64_t t_tick = TICK_NS(sim_fps);
  clock_sleep_ns(t_tick - t_work); /* a negative sleep just does nothing */
  *out_work_ns = t_work;
  return clock_ns();
}

/* Count frames and, twice a second, update the fps number shown in the
 * HUD. Counts a frame as a whole budget even if the work overran, so the
 * reading stays steady. */
static void update_fps_counter(Scene *s, int64_t work_ns, int64_t *fps_acc,
                               int *fps_cnt) {
  int64_t t_tick = TICK_NS(s->sim_fps);
  int64_t slack = t_tick - work_ns;
  *fps_acc += work_ns + (slack > 0 ? slack : 0);
  (*fps_cnt)++;
  if (*fps_acc >= NS_PER_SEC / 2) {
    s->fps_disp = *fps_cnt * 2; /* counted over half a second, so double */
    *fps_acc = 0;
    *fps_cnt = 0;
  }
}

/*
 * The whole program: set up, then loop until quit — handle a resize,
 * handle keys, step the physics, draw, hold the frame rate, update fps.
 */
int main(void) {
  install_signal_handlers();
  terminal_setup();
  color_init();

  Scene *s = &g_scene;
  memset(s, 0, sizeof *s);
  scene_defaults(s);
  theme_apply(s->theme);

  getmaxyx(stdscr, s->term_rows, s->term_cols);
  rebuild_for_terminal(s);

  int64_t t_last = clock_ns();
  int64_t fps_acc = 0;
  int fps_cnt = 0;

  while (!g_quit) {
    consume_resize_event(s);
    process_input(s);
    step_physics(s);
    render_frame(s);

    int64_t work_ns;
    t_last = pace_frame_to_fps(t_last, s->sim_fps, &work_ns);
    update_fps_counter(s, work_ns, &fps_acc, &fps_cnt);
  }

  endwin();
  return 0;
}
