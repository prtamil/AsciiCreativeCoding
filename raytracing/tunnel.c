/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */

/* tunnel.c — a first-person flight down an endless textured pipe, in the
 * terminal. For each character cell we shoot a ray and find where it hits the
 * pipe wall; the camera actually stays put while the texture scrolls past, which
 * the eye reads as flying forward. Patterns (n), colour themes (t), speed,
 * sway and sim rate are all live keys, shown along the bottom while it runs.
 * Sister files: sphere_raytrace.c / cube_raytrace.c / torus_raytrace.c (same
 * one-ray-per-cell idea on solid shapes); raymarcher/raymarcher.c for the
 * opposite approach when a shape has no neat hit formula. */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* §1 — every tunable number in one place, so none of them show up as a
 * mystery value down in the code. */

enum {
  /* How fast the simulation steps (the `[` / `]` keys), and the render cap. */
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,
  RENDER_FPS_CAP = 60, /* drawing never runs faster than this; sim rate is separate */
  FPS_UPDATE_MS = 500, /* how often the on-screen fps number refreshes */

  /* ncurses colour-slot numbers. The depth ramp uses 8 in a row from the base. */
  PAIR_HUD = 1,        /* top status row (yellow) */
  PAIR_HINT = 2,       /* bottom hint row (cyan) */
  PAIR_DEPTH_BASE = 3, /* +0..+7: far/dim → near/bright */
  PAIR_VANISH = 11,    /* the centre crosshair */
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))
#define DT_CAP_NS (100 * NS_PER_MS) /* ignore frame stalls longer than 0.1s */

/* Terminal cells are about twice as tall as wide; we squash by this so the
 * round pipe looks round instead of stretched. */
#define CELL_ASPECT 2.0f

/* The pipe and the lens. */
#define TUNNEL_RADIUS 2.0f /* the camera flies inside a pipe of this radius */
#define FOV_DEG 70.0f      /* how wide a view the lens takes in */

/* Forward speed (texture scroll), set by + / -. */
#define SPEED_DEFAULT 8.0f
#define SPEED_MIN 0.5f
#define SPEED_MAX 40.0f
#define SPEED_STEP_FACTOR 1.30f /* each press multiplies/divides by this */

/* Side-to-side wobble as you fly, set by s / S. */
#define SWAY_AMP_DEFAULT 1.30f /* how far the camera drifts off-centre (cells) */
#define SWAY_AMP_MIN 0.0f
#define SWAY_AMP_MAX 1.65f /* kept under the pipe radius so we never clip the wall */
#define SWAY_AMP_STEP 0.10f
#define SWAY_FREQ_X 0.55f      /* how quickly it drifts left/right (rad per sec) */
#define SWAY_FREQ_Y 0.31f      /* and up/down */
#define SWAY_AMP_Y_RATIO 0.55f /* vertical drift is this fraction of horizontal */
#define ROLL_RATE 0.18f        /* how fast the view slowly spins (rad per sec) */

#define EPS_PARALLEL 1.5e-3f /* a ray flatter than this counts as "straight down
                                the pipe" — it never hits a wall */
#define T_MAX 80.0f /* stop looking past here; treat it as the far vanishing point */

/* Haze: walls fade with distance so the tube has depth. */
#define FOG_DENSITY 0.030f /* bigger = fades out sooner */
#define FOG_FLOOR 0.18f    /* but never all the way to black, so structure stays */

/* We bucket brightness into 8 levels — one colour + one character per level. */
#define N_DEPTH_TIERS 8
#define DEPTH_TIER_SCALE 7.999f /* 0..1 brightness → level 0..7 (just under 8) */
#define TIER_BOLD_MIN 6         /* nearest levels drawn bold */
#define TIER_DIM_MAX 1          /* farthest levels drawn dim */

/* Which texture is painted on the pipe wall, cycled with n / N. Every pattern
 * is just a function of two coordinates on the wall — the angle around the pipe
 * and the distance along it (like longitude and how far down a tube). The shape
 * of the pipe never changes; only this function does. */
typedef enum {
  PATTERN_RINGS = 0,   /* bands wrapped around the tube */
  PATTERN_CHECKER = 1, /* a checkerboard */
  PATTERN_SPOKES = 2,  /* stripes running down the length */
  PATTERN_GRID = 3,    /* thin cross-hatch lines */
  N_PATTERNS = 4,      /* how many there are, so n/N can wrap around */
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_RINGS:
    return "RINGS  ";
  case PATTERN_CHECKER:
    return "CHECKER";
  case PATTERN_SPOKES:
    return "SPOKES ";
  case PATTERN_GRID:
    return "GRID   ";
  default:
    return "?      ";
  }
}

/* The dials for one pattern — one row of the table below, picked by the active
 * pattern and handed to the pattern functions. They're all measured on the wall
 * surface, so a pattern looks the same size whether it's near or far.
 *   stripes_around : how many cells make one trip around the pipe (even, so the
 *                    seam where the angle wraps lines up)
 *   stripes_long   : how many cells per unit of distance down the pipe
 *   ring_freq      : how tightly the RINGS bands are spaced
 *   line_thick     : how wide a GRID line is, as a fraction of a cell */
typedef struct {
  int stripes_around;
  float stripes_long;
  float ring_freq;
  float line_thick;
} PatternParams;

static const PatternParams patterns[N_PATTERNS] = {
    /*                  around  long   ring_freq  line_thick */
    /* RINGS   */ {16, 0.55f, 1.55f, 0.30f},
    /* CHECKER */ {16, 0.55f, 0.0f, 0.0f},
    /* SPOKES  */ {24, 0.0f, 0.0f, 0.0f},
    /* GRID    */ {16, 0.55f, 0.0f, 0.20f},
};

/* A colour scheme, cycled with t / T. `depth` is 8 colours from farthest (dim)
 * to nearest (bright) — that gradient is what makes the tube feel deep. `vanish`
 * tints the centre crosshair. All are kept in the bright half of the palette so
 * even the dim end stays visible. */
typedef struct {
  const char *name;
  short depth[8]; /* far → near */
  short vanish;
} Theme;

#define N_THEMES 6

static const Theme themes[N_THEMES] = {
    /* WARP — magenta/violet hyperdrive */
    {"WARP    ", {53, 91, 134, 165, 207, 213, 219, 225}, 231},

    /* INFERNO — red/orange/yellow lava tube */
    {"INFERNO ", {88, 124, 160, 202, 208, 214, 220, 226}, 231},

    /* ELECTRIC — cyan/blue/white energy conduit */
    {"ELECTRIC", {24, 26, 31, 39, 45, 51, 123, 195}, 231},

    /* VOID — deep teal-blue cold space */
    {"VOID    ", {24, 25, 31, 38, 45, 51, 117, 195}, 195},

    /* MATRIX — neon-green cyber */
    {"MATRIX  ", {28, 34, 76, 82, 118, 154, 190, 226}, 231},

    /* GOLD — amber/bone treasure passage */
    {"GOLD    ", {130, 137, 173, 179, 215, 222, 229, 230}, 231},
};

/* The characters we draw with, faintest to densest, so a brighter cell picks a
 * denser one. */
static const char LUMA_GLYPHS[8] = {' ', '.', ',', ':', '+', '*', '#', '@'};

/* A little 5-cell crosshair drawn at screen centre, where the pipe's far end
 * always sits — a steady thing for the eye to hold onto. */
#define VANISH_CENTER_GLYPH '+'
#define VANISH_ARM_GLYPH 'o'

/* §2 — a steady clock (only ever counts forward) and a plain sleep, used to
 * pace the loop. */

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

/* §3 — load colours into ncurses' slots. Loads the active theme's 8 depth
 * colours + the crosshair tint; falls back to basic colours on a 16-colour
 * terminal. The two HUD colours stay fixed regardless of theme. */

/* program the 8 depth colours + crosshair tint for one theme (on a 't' press) */
static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  if (COLORS >= 256) {
    const Theme *t = &themes[idx];
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_DEPTH_BASE + i), t->depth[i], -1);
    init_pair(PAIR_VANISH, t->vanish, -1);
  } else {
    static const short fb[8] = {
        COLOR_BLUE, COLOR_BLUE, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,   COLOR_WHITE,
    };
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_DEPTH_BASE + i), fb[i], -1);
    init_pair(PAIR_VANISH, COLOR_WHITE, -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
  theme_apply(0);
}

/* §4 — bare-bones 3-D vector maths. */

/* Three floats used as either a point in space (the camera, a hit point) or a
 * direction (a ray, an axis). Directions are kept length-1 by v3norm where the
 * maths needs it. */
typedef struct {
  float x, y, z;
} V3;

static inline V3 v3(float x, float y, float z) { return (V3){x, y, z}; }
static inline V3 v3add(V3 a, V3 b) {
  return (V3){a.x + b.x, a.y + b.y, a.z + b.z};
}
static inline V3 v3scale(float s, V3 a) {
  return (V3){s * a.x, s * a.y, s * a.z};
}
static inline float v3len(V3 a) {
  return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
}
static inline V3 v3norm(V3 a) {
  float L = v3len(a);
  return L > 1e-12f ? v3scale(1.0f / L, a) : v3(0, 0, 1);
}

/* §5 — how far along a ray it hits the pipe wall. The pipe is the set of points
 * a fixed distance from the central axis, which boils the question down to a
 * little quadratic equation in the distance t — so one square root gives the
 * exact answer, no marching along the ray. Since the camera is inside the pipe,
 * there's always a hit ahead; we take it. A ray pointing almost straight down
 * the pipe never meets the wall — we flag it (`parallel`) and the caller paints
 * it as the dark far end. Only the x/y part matters because the pipe runs along
 * z. */
static inline float cylinder_hit(V3 O, V3 D, float R, bool *parallel) {
  float a = D.x * D.x + D.y * D.y;
  if (a < EPS_PARALLEL) {
    *parallel = true;
    return -1.0f;
  }
  *parallel = false;
  float b = O.x * D.x + O.y * D.y;
  float c = O.x * O.x + O.y * O.y - R * R;
  float disc = b * b - a * c;
  if (disc < 0.0f)
    return -1.0f; /* can't happen from inside; guard anyway */
  float t = (-b + sqrtf(disc)) / a;
  return t;
}

/* §6 — RINGS: bright/dark bands wrapped around the tube, as a smooth wave along
 * its length. It ignores the angle, so spinning the view doesn't change it — you
 * only see the forward flow. Returns 0..1. */
static inline float pattern_rings(float v_tex, const PatternParams *pp) {
  return sinf(v_tex * pp->ring_freq) * 0.5f + 0.5f;
}

/* §7 — CHECKER: a checkerboard. Chop the angle and the length into whole cells;
 * a cell is bright when the two cell-counts have opposite odd/even-ness. Both
 * directions matter, so flow, sway and spin all show. Returns 0 or 1. */
static inline float pattern_checker(float u_tex, float v_tex,
                                    const PatternParams *pp) {
  int gu = (int)floorf(u_tex * (float)pp->stripes_around);
  int gv = (int)floorf(v_tex * pp->stripes_long);
  return ((gu + gv) & 1) ? 1.0f : 0.0f;
}

/* §8 — SPOKES: stripes running the length of the tube — bright on every other
 * wedge of angle. No dependence on length, so the forward flow is invisible but
 * the slow spin shows. Returns 0 or 1. */
static inline float pattern_spokes(float u_tex, const PatternParams *pp) {
  int gu = (int)floorf(u_tex * (float)pp->stripes_around);
  return (gu & 1) ? 1.0f : 0.0f;
}

/* §9 — GRID: thin cross-hatch lines on a dark wall. A cell is bright when it's
 * close to a grid line in either direction — "close" meaning within line_thick
 * of a cell boundary. Returns 0 or 1. */
static inline float pattern_grid(float u_tex, float v_tex,
                                 const PatternParams *pp) {
  float u_pos = u_tex * (float)pp->stripes_around;
  float v_pos = v_tex * pp->stripes_long;
  float u_frac = u_pos - floorf(u_pos);
  float v_frac = v_pos - floorf(v_pos);
  float u_dist = fminf(u_frac, 1.0f - u_frac);
  float v_dist = fminf(v_frac, 1.0f - v_frac);
  float thick = pp->line_thick;
  if (u_dist < thick || v_dist < thick)
    return 1.0f;
  return 0.0f;
}

/* §10 — pick the active pattern and ask it how bright this spot on the wall is
 * (0..1). */
static float pattern_sample(float u_tex, float v_tex, int pattern_idx) {
  const PatternParams *pp = &patterns[pattern_idx];
  switch (pattern_idx) {
  case PATTERN_RINGS:
    return pattern_rings(v_tex, pp);
  case PATTERN_CHECKER:
    return pattern_checker(u_tex, v_tex, pp);
  case PATTERN_SPOKES:
    return pattern_spokes(u_tex, pp);
  case PATTERN_GRID:
  default:
    return pattern_grid(u_tex, v_tex, pp);
  }
}

/* §11 — distance haze and bucketing.
 *
 * fog_of: how much a wall hit at distance t is dimmed by haze — 1 right in
 * front of the camera, fading toward 0 far down the tube. It drives the COLOUR
 * (so colour reads as distance), while the pattern folded in with the fog drives
 * the GLYPH. FOG_FLOOR keeps even dark cells from going fully black, so the tube
 * stays visible all the way to the far end. */
static inline float fog_of(float t) { return expf(-t * FOG_DENSITY); }

/* squash a 0..1 brightness onto one of the 8 ramp levels (0..7) */
static inline int tier_of(float x) {
  int s = (int)(x * DEPTH_TIER_SCALE);
  if (s < 0)
    s = 0;
  if (s > N_DEPTH_TIERS - 1)
    s = N_DEPTH_TIERS - 1;
  return s;
}

/* §12 — the flight's state, all in one place. The trick: the only thing that
 * actually changes over time is the clock; everything you see moving (the
 * scroll, the wobble, the slow spin) is computed fresh each frame from that one
 * number, so there's no animation state to keep in sync. The rest are knobs the
 * keys turn. */
typedef struct {
  float time; /* seconds since the last reset — the one evolving value */

  /* flight knobs */
  float speed;    /* how fast the texture scrolls (forward-motion feel) */
  float sway_amp; /* how far the camera wobbles off-centre */
  bool paused;    /* freeze the clock */

  /* look knobs */
  int current_pattern; /* which wall texture (a Pattern) */
  int current_theme;   /* which colour scheme (index into themes) */

  /* current window size, refreshed on resize */
  int cols, rows;
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->current_pattern = PATTERN_RINGS;
  s->current_theme = 0;
  s->cols = cols;
  s->rows = rows;
  s->time = 0.0f;
  s->speed = SPEED_DEFAULT;
  s->sway_amp = SWAY_AMP_DEFAULT;
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
}

static void scene_reset(Scene *s) { s->time = 0.0f; }

/* the only thing that advances each tick: move the clock forward (unless paused) */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->time += dt;
}

/* §13 — where the camera is and which way it's pointing this frame, all worked
 * out from the clock. Two motions layered on: a gentle drift off-centre (the
 * wobble) and a slow roll of the view. `fwd` always points down the pipe; the
 * camera never actually moves forward — the scrolling texture fakes that. Built
 * fresh every frame, nothing kept between frames. */
typedef struct {
  V3 O;              /* where the camera is (off-centre by the wobble) */
  V3 fwd, right, up; /* which way it points (fwd = down the pipe, then rolled) */
  float fov_t;       /* how wide the lens spreads rays (from the FOV) */
  float phys_aspect; /* corrects for tall terminal cells so the pipe stays round */
} Camera;

static Camera build_camera(const Scene *s, int rows_eff) {
  Camera c;
  c.O = (V3){s->sway_amp * sinf(s->time * SWAY_FREQ_X),
             s->sway_amp * cosf(s->time * SWAY_FREQ_Y) * SWAY_AMP_Y_RATIO,
             0.0f};

  float r = s->time * ROLL_RATE;
  float cr = cosf(r);
  float sr = sinf(r);
  c.fwd = v3(0.0f, 0.0f, 1.0f);
  c.right = v3(cr, sr, 0.0f);
  c.up = v3(-sr, cr, 0.0f);

  c.fov_t = tanf(FOV_DEG * (float)M_PI / 180.0f * 0.5f);
  c.phys_aspect = ((float)rows_eff * CELL_ASPECT) / (float)s->cols;
  return c;
}

/* §14 — turning the scene into a screen of characters: one ray per cell, plus
 * the centre crosshair on top. The top and bottom rows are left for the HUD, so
 * the tunnel draws into the rows between (shifted down by y_offset). */

/* the direction the ray for one screen cell points: start down the pipe, then
 * lean right/up by how far this cell is from centre (scaled by the lens) */
static inline V3 primary_ray_dir(const Camera *cam, float u, float v) {
  return v3norm(v3add(
      cam->fwd, v3add(v3scale(u * cam->fov_t, cam->right),
                      v3scale(v * cam->fov_t * cam->phys_aspect, cam->up))));
}

/* where on the wall a hit lands, as two texture coords: the angle around the
 * pipe (0..1) and the distance along it, minus the scroll offset so the texture
 * streams toward you (the forward-motion illusion) */
static inline void cylinder_uv(V3 P, float v_flow, float *u_tex, float *v_tex) {
  *u_tex = (atan2f(P.y, P.x) + (float)M_PI) * (1.0f / (2.0f * (float)M_PI));
  *v_tex = P.z - v_flow;
}

/* work out one wall cell: hit the pipe, then turn the hit into a character +
 * colour + bold/dim. A ray that goes straight down the pipe (or absurdly far)
 * has no wall to show, so it becomes the dark far end. */
static void shade_tunnel_cell(V3 O, V3 D, float v_flow, int pattern_idx,
                              char *glyph, int *pair, attr_t *attr) {
  bool parallel = false;
  float t = cylinder_hit(O, D, TUNNEL_RADIUS, &parallel);

  if (parallel || t > T_MAX || t < 0.0f) {
    *pair = PAIR_DEPTH_BASE + 0;
    *attr = A_DIM;
    *glyph = LUMA_GLYPHS[0];
    return;
  }

  V3 P = v3add(O, v3scale(t, D));
  float u_tex, v_tex;
  cylinder_uv(P, v_flow, &u_tex, &v_tex);

  float fog = fog_of(t);
  float bright = pattern_sample(u_tex, v_tex, pattern_idx);

  /* Two independent channels — far clearer on a coarse character grid:
   *   COLOUR depends on DISTANCE only (fog), so a wall at a given depth is one
   *   colour and the tube reads as a clean near-bright → far-dim gradient.
   *   GLYPH depends on distance AND the surface pattern, so the texture shows
   *   up in the character density instead of masquerading as a depth change in
   *   the colour (which made the checker/grid walls look buckled).
   *   Brightness/dimness follows depth too, reinforcing the near/far cue. */
  int depth_slot = tier_of(fog);
  float intensity = fog * (FOG_FLOOR + bright * (1.0f - FOG_FLOOR));
  int glyph_slot = tier_of(intensity);

  *glyph = LUMA_GLYPHS[glyph_slot];
  *pair = PAIR_DEPTH_BASE + depth_slot;
  *attr = (depth_slot >= TIER_BOLD_MIN)  ? A_BOLD
          : (depth_slot <= TIER_DIM_MAX) ? A_DIM
                                         : A_NORMAL;
}

/* §15 — the little crosshair at screen centre, drawn last so it sits on top.
 * The pipe's far end always lands dead centre (no matter how the camera wobbles),
 * so this is a fixed spot for the eye to rest on. */
static void draw_vanishing_crosshair(int cols, int rows_eff, int y_offset) {
  int cx = cols / 2;
  int cy = rows_eff / 2;
  if (cx >= 0 && cx < cols && cy >= 0 && cy < rows_eff) {
    attron(COLOR_PAIR(PAIR_VANISH) | A_BOLD);
    mvaddch(cy + y_offset, cx, (chtype)(unsigned char)VANISH_CENTER_GLYPH);
    attroff(COLOR_PAIR(PAIR_VANISH) | A_BOLD);

    attron(COLOR_PAIR(PAIR_VANISH));
    if (cy - 1 >= 0)
      mvaddch(cy - 1 + y_offset, cx, (chtype)(unsigned char)VANISH_ARM_GLYPH);
    if (cy + 1 < rows_eff)
      mvaddch(cy + 1 + y_offset, cx, (chtype)(unsigned char)VANISH_ARM_GLYPH);
    if (cx - 1 >= 0)
      mvaddch(cy + y_offset, cx - 1, (chtype)(unsigned char)VANISH_ARM_GLYPH);
    if (cx + 1 < cols)
      mvaddch(cy + y_offset, cx + 1, (chtype)(unsigned char)VANISH_ARM_GLYPH);
    attroff(COLOR_PAIR(PAIR_VANISH));
  }
}

/* cell_to_ndc — map a cell centre (col,row) to normalised device coords in
 * [-1,1], with v flipped so screen-row-down becomes world-up. */
static inline void cell_to_ndc(int col, int row, int cols, int rows, float *u,
                               float *v) {
  *u = ((float)col + 0.5f) / (float)cols * 2.0f - 1.0f;
  *v = -(((float)row + 0.5f) / (float)rows * 2.0f - 1.0f);
}

/* set_active_attr — make (pair, attr) the active ncurses style, but only emit
 * the attron/attroff when it differs from the previous cell, so a run of
 * same-depth cells shares one switch (fewer escape codes). cur_pair and
 * cur_attr carry the active style across the scan; start them at (-1, 0). */
static inline void set_active_attr(int pair, attr_t attr, int *cur_pair,
                                   attr_t *cur_attr) {
  if (pair == *cur_pair && attr == *cur_attr)
    return;
  if (*cur_pair >= 0)
    attroff(COLOR_PAIR(*cur_pair) | *cur_attr);
  attron(COLOR_PAIR(pair) | attr);
  *cur_pair = pair;
  *cur_attr = attr;
}

static void scene_render(const Scene *s) {
  /* leave the top and bottom rows for the HUD; draw the tunnel between them */
  int rows_eff = s->rows - 2;
  int y_offset = 1;
  if (rows_eff < 1)
    return;

  Camera cam = build_camera(s, rows_eff);
  float v_flow = s->speed * s->time; /* how far the texture has scrolled by now */

  /* one cell at a time: cell -> ray -> shade -> draw */
  int last_pair = -1;
  attr_t last_attr = 0;
  for (int row = 0; row < rows_eff; row++) {
    for (int col = 0; col < s->cols; col++) {
      float u, v;
      cell_to_ndc(col, row, s->cols, rows_eff, &u, &v);
      V3 D = primary_ray_dir(&cam, u, v);

      char glyph;
      int pair;
      attr_t attr;
      shade_tunnel_cell(cam.O, D, v_flow, s->current_pattern, &glyph, &pair,
                        &attr);

      set_active_attr(pair, attr, &last_pair, &last_attr);
      mvaddch(row + y_offset, col, (chtype)(unsigned char)glyph);
    }
  }
  if (last_pair >= 0)
    attroff(COLOR_PAIR(last_pair) | last_attr);

  draw_vanishing_crosshair(s->cols, rows_eff, y_offset);
}

/* §16 — talking to the terminal: set it up, draw the HUD, push the frame out. */

/* The terminal we draw to — just its size, re-read whenever the window resizes
 * so the render always matches the real window. */
typedef struct {
  int cols, rows; /* window size in characters (refreshed on resize) */
} Screen;

static void screen_init(Screen *sc) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) {
  (void)sc;
  endwin();
}
static void screen_resize_curses(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

/* Draw one full frame: the tunnel, then the two HUD rows on top. The fps sits
 * in the left label so it stays on screen even if the long status line on the
 * right gets cut off on a narrow window. */
static void screen_draw(const Screen *sc, const Scene *s, double fps,
                        int sim_fps) {
  erase();
  scene_render(s);

  /* top row: title + fps on the left, settings on the right (trimmed to fit) */
  char left[48];
  snprintf(left, sizeof left, " TUNNEL  %5.1f fps ", fps);
  int llen = (int)strlen(left);

  char status[200];
  snprintf(status, sizeof status,
           " %s  pat:%s  theme:%s  speed:%4.1f  sway:%4.2f  sim:%3dHz ",
           s->paused ? "PAUSED" : "FLYING", pattern_name(s->current_pattern),
           themes[s->current_theme].name, (double)s->speed, (double)s->sway_amp,
           sim_fps);
  int slen = (int)strlen(status);
  int max_slen = sc->cols - llen;
  if (max_slen < 0)
    max_slen = 0;
  if (slen > max_slen)
    slen = max_slen;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, 0, "%s", left);
  if (slen > 0)
    mvprintw(0, sc->cols - slen, "%.*s", slen, status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* bottom row: the key hints */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(sc->rows - 1, 0,
           " q:quit  spc:pause  r:reset  n/N:pat  t/T:theme  "
           "+/-:speed  s/S:sway  ]/[:fps ");
  clrtoeol();
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* §17 — the whole program: the main loop, signal handling, and keys. */

/* Everything the running program holds onto: the flight, the screen, the sim
 * rate, and two flags the signal handlers flip (one to quit, one to note the
 * window resized). Only main and the app_* helpers see the whole thing; every
 * other function takes just the piece it needs. */
typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;                       /* simulation steps per second ([ / ]) */
  volatile sig_atomic_t running;     /* set to 0 to quit (by a signal or 'q') */
  volatile sig_atomic_t need_resize; /* set to 1 when the window resized */
} App;

static App g_app;

static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void app_do_resize(App *app) {
  screen_resize_curses(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;
  case 'r':
  case 'R':
    scene_reset(s);
    break;

  case 'n':
    s->current_pattern = (s->current_pattern + 1) % N_PATTERNS;
    break;
  case 'N':
    s->current_pattern = (s->current_pattern + N_PATTERNS - 1) % N_PATTERNS;
    break;

  case 't':
    s->current_theme = (s->current_theme + 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;
  case 'T':
    s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;

  case '=':
  case '+':
    s->speed *= SPEED_STEP_FACTOR;
    if (s->speed > SPEED_MAX)
      s->speed = SPEED_MAX;
    break;
  case '-':
    s->speed /= SPEED_STEP_FACTOR;
    if (s->speed < SPEED_MIN)
      s->speed = SPEED_MIN;
    break;

  case 's':
    s->sway_amp += SWAY_AMP_STEP;
    if (s->sway_amp > SWAY_AMP_MAX)
      s->sway_amp = SWAY_AMP_MAX;
    break;
  case 'S':
    s->sway_amp -= SWAY_AMP_STEP;
    if (s->sway_amp < SWAY_AMP_MIN)
      s->sway_amp = SWAY_AMP_MIN;
    break;

  case ']':
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX)
      app->sim_fps = SIM_FPS_MAX;
    break;
  case '[':
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN)
      app->sim_fps = SIM_FPS_MIN;
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {
    /* one pass = one frame */

    /* handle a window resize that happened since last frame */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* how much real time passed, ignoring any huge stall */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    /* step the clock in fixed-size ticks, so the motion runs at the same rate
     * no matter the frame rate (paused freezes it) */
    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }

    /* update the fps number shown in the HUD */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* don't run hotter than the render cap — sleep off the rest of the frame */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / RENDER_FPS_CAP - elapsed);

    /* the one place state becomes pixels: draw the tunnel + HUD, then show it */
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();

    /* read one keypress (a knob or quit) — not part of the tick */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}

