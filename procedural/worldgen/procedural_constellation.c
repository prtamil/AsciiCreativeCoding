/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * procedural_constellation.c — makes up constellations and reveals them.
 *
 * Scatters a handful of bright "anchor" stars, connects them with one of
 * fifteen line patterns (tree, chain, loop, ...), gives the figure a made-up
 * Latin-ish name, and animates the whole "discovery" star by star then line by
 * line. A faint twinkling backdrop of dots fills the rest of the sky.
 *
 * Sister files: ../worldgen/procedural_galaxy.c (a dense FIELD of stars, the
 * opposite of this sparse named GRAPH); ../fractal_random/lightning.c (same
 * line-drawing trick, applied to an animated lightning bolt).
 *
 * Build: gcc -std=c11 -O2 -Wall -Wextra
 *        procedural/worldgen/procedural_constellation.c -o constellation
 *        -lncurses -lm
 */

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

/* The file is split into eight sections by what they touch. §1 config and the
 * data tables. §2 timing helpers. §3 pure helpers that only read and decide.
 * §4 the only code that changes the simulation (generate a sky, advance the
 * reveal). §5/§6 are placeholders kept for layout symmetry — they hold no real
 * code here. §7 drawing. §8 the main loop. Only scene_tick() (§4) advances the
 * animation; keypresses and resizes change things too but happen outside the
 * tick loop, once per frame. */

/* §1  config — constants, data tables, core types */

enum {
  /* At most 12 bright stars per figure — more starts to look like noise. */
  N_STARS_MAX = 12,
  N_EDGES_MAX = 24,

  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 240,
  SIM_FPS_STEP = 10,

  SPEED_MIN = 1,
  SPEED_DEF = 8,
  SPEED_MAX = 64,

  HUD_COLS = 80,
  FPS_UPDATE_MS = 500,

  /* How thinly the faint backdrop dots are sprinkled: 1 cell in 60. */
  BG_STAR_DENSITY = 60,

  /* Color-pair slots. HUD/HINT are reserved by the project HUD standard. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_BG_STAR = 3,   /* faint backdrop dots             */
  PAIR_LINE = 4,      /* the constellation lines         */
  PAIR_STAR_BASE = 5, /* +0..+3 = the 4 anchor-star tints*/
  PAIR_NAME = 9,      /* the name caption                */
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* How long each step of the animation takes, in seconds. The "speed" knob
 * scales these: higher speed = faster reveal. */
#define PHASE_STARS_TOTAL 1.6f /* time to light up all the stars  */
#define PHASE_EDGE_TIME 0.30f  /* time to draw one line           */
#define PHASE_HOLD_TIME 6.0f   /* how long to sit and admire it   */
#define PHASE_FADE_TIME 0.6f   /* brief pause before the next sky */

/* How fast a fully-lit star pulses, in pulses per second. */
#define TWINKLE_HZ 0.6f

/* How far a star is nudged off its grid-cell centre, as a fraction of the cell.
 * 0.40 looks scattered without letting neighbouring stars overlap. */
#define JITTER_FRAC 0.40f

/* Pattern — which of fifteen ways to connect the stars with lines.
 *
 * The stars are placed the same way every time; the PATTERN is only about which
 * lines get drawn between them. So a constellation's "shape" really is just a
 * choice of how to wire up the same dots. Each value picks one make_* builder
 * in §4. The first four echo the four families real constellations fall into
 * (branchy, straight-line, closed loop, spokes from a hub); the rest add more
 * shapes for variety. */
typedef enum {
  PATTERN_TREE = 0,    /* sparse organic branching, no loops     */
  PATTERN_CHAIN,       /* a left-to-right line                   */
  PATTERN_LOOP,        /* a closed ring                          */
  PATTERN_SPOKE,       /* a central hub with spokes to all       */
  PATTERN_WHEEL,       /* a ring plus spokes from the centre      */
  PATTERN_FAN,         /* rays fanning out from one corner star    */
  PATTERN_STARPOLY,    /* a star/pentagram whose lines cross      */
  PATTERN_NEAREST,     /* each star linked to its 1 nearest        */
  PATTERN_MESH,        /* each star linked to its 2 nearest (web)  */
  PATTERN_ARCH,        /* an open sweeping arc                     */
  PATTERN_BINARY,      /* two separate little clusters             */
  PATTERN_CROSS,       /* two chains woven across each other       */
  PATTERN_TRIANGLES,   /* a fan plus an arc, tiled into triangles  */
  PATTERN_SPIRAL,      /* one arm winding inner to outer           */
  PATTERN_CATERPILLAR, /* a spine with leaves hanging off it       */
  N_PATTERNS,
} Pattern;

/* Theme — one named colour scheme (10 of them; t/T cycles).
 *
 * Each part of the picture gets its own colour slot so they can be tinted
 * independently. theme_apply() loads these into the ncurses colour pairs. Every
 * colour is kept in the bright half of the 256-colour space so nothing
 * disappears against a black terminal. See documentation/COLOR.md. */
typedef struct {
  const char *name; /* shown in the HUD                             */
  short bg_star;    /* the faint backdrop dots                      */
  short line;       /* the constellation lines                      */
  short star[4];    /* four star tints; each star picks one by hash */
  short name_color; /* the name caption                             */
} Theme;

#define N_THEMES 10

/* All colours chosen bright so they stay readable on a dark terminal. The
 * backdrop dots are tinted per theme (green dust for MATRIX, purple for NOVA)
 * instead of plain gray, so each sky has its own mood. */
static const Theme themes[N_THEMES] = {
    /* name        bg   line  star{0,1,2,3}              name_col */
    {"DEFAULT", 245, 252, {226, 230, 159, 117}, 226},
    {"MATRIX", 65, 118, {230, 226, 154, 118}, 154},
    {"NOVA", 97, 213, {231, 219, 207, 177}, 219},
    {"MONO", 247, 253, {255, 252, 248, 244}, 254},
    {"OCEAN", 67, 117, {231, 195, 159, 87}, 159},
    {"FIRE", 131, 214, {231, 226, 215, 209}, 226},
    {"EARTH", 137, 223, {231, 230, 222, 179}, 230},
    {"FOREST", 66, 114, {231, 192, 156, 120}, 156},
    {"DESERT", 180, 223, {231, 230, 223, 215}, 230},
    {"ARCTIC", 153, 195, {231, 219, 195, 159}, 195},
};

/* Name pieces. A name is one prefix + one suffix + an optional modifier word.
 * The prefixes are clipped so prefix+suffix reads as one Latin-ish word
 * ("Lyr"+"ax" -> "Lyrax"). With ~33 x 16 x 10 = ~5000 combinations, two skies
 * in a row almost never get the same name. */
static const char *PREFIXES[] = {
    "Auri",   "Lyr",    "Dracon", "Cygn",  "Pegas", "Hydr",  "Casso",
    "Cepheu", "Andro",  "Boote",  "Virg",  "Aquil", "Sagit", "Capric",
    "Corv",   "Lupin",  "Perse",  "Trian", "Erid",  "Phen",  "Cete",
    "Tauri",  "Gemin",  "Scorpi", "Libr",  "Indus", "Phoen", "Lacert",
    "Vulpe",  "Centau", "Lepor",  "Ursar", "Vegan",
};

static const char *SUFFIXES[] = {
    "a",   "us", "is", "es",  "ax",  "or",   "on",  "ula",
    "ina", "as", "ea", "ona", "ena", "alia", "ius", "yra",
};

static const char *MODIFIERS[] = {
    "",           "",        "",
    "",           "", /* blanks: most names get no modifier */
    " Major",     " Minor",  " Borealis",
    " Australis", " Magnus",
};

static const int N_PREFIXES = (int)(sizeof PREFIXES / sizeof PREFIXES[0]);
static const int N_SUFFIXES = (int)(sizeof SUFFIXES / sizeof SUFFIXES[0]);
static const int N_MODIFIERS = (int)(sizeof MODIFIERS / sizeof MODIFIERS[0]);

/* Star — one bright anchor star, a dot the lines connect.
 *
 * Placed once and never moved. Some patterns sort the stars[] array in place,
 * so a star's index only means something relative to the current order.
 *   x,y           where it sits inside the constellation region (0..region_w/h);
 *                 drawn on screen at gx0+x, gy0+y.
 *   twinkle_phase 0..255, a starting point for its pulse so neighbours twinkle
 *                 out of step instead of all blinking together.
 *   color_idx     0..3, which of the theme's four star tints to use.
 *   reveal_t      0..1, how far this star has faded in. scene_tick writes it. */
typedef struct {
  int x, y;
  uint8_t twinkle_phase;
  uint8_t color_idx;
  float reveal_t;
} Star;

/* Edge — one line between two stars (by their index in stars[]). The set of
 * edges IS the pattern; the fifteen patterns differ only in which lines they
 * choose to draw.
 *   reveal_t  0..1, how much of the line has been drawn so far. At 0.5 only the
 *            first half is on screen, so the line looks like it's growing from
 *            one end. scene_tick writes it. */
typedef struct {
  int from, to;
  float reveal_t;
} Edge;

/* Constellation — one whole figure: the stars, the lines between them, and the
 * name. This is the thing the drawing code reads. It is rebuilt from scratch
 * for every new sky; §4 is the only place that writes it.
 *
 * The arrays are fixed-size and never malloc'd — a rebuild just overwrites them
 * in place.
 *   n_stars/n_edges  how many of the arrays are actually in use.
 *   name             the made-up name.
 *   seed             the number this figure was grown from (also reused to seed
 *                    the backdrop sprinkle).
 *   region_w/h       the area the stars were placed in, so a resize knows the
 *                    layout is stale and must be regenerated. */
typedef struct {
  Star stars[N_STARS_MAX];
  int n_stars;
  Edge edges[N_EDGES_MAX];
  int n_edges;
  char name[40];
  int seed;
  int region_w, region_h;
} Constellation;

/* Phase — which step of the reveal animation we're in, advanced by scene_tick.
 * They run in this order: stars fade in, then lines draw in, then the figure
 * holds while the name appears, then a brief fade before the next sky. phase_t
 * (in Scene) tracks the time spent in the current phase. */
typedef enum {
  PHASE_DRAW_STARS = 0,
  PHASE_DRAW_EDGES = 1,
  PHASE_HOLD = 2,
  PHASE_FADE = 3,
} Phase;

/* §2  timing helpers */

/* Just a clock and a sleep. The actual frame-rate pacing lives in main(). */

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

/* §3  pure helpers — only read and decide, never change anything */

/* Everything here just computes an answer from its inputs: no state is changed
 * and nothing is drawn. The code that DOES change things (sorting stars,
 * adding lines) lives in §4. */

/* Names padded to 5 chars so the HUD column stays a fixed width. */
static const char *pattern_name(Pattern p) {
  static const char *names[N_PATTERNS] = {
      "TREE ", "CHAIN", "LOOP ", "SPOKE", "WHEEL", "FAN  ", "POLY*", "NEAR ",
      "MESH ", "ARCH ", "TWINS", "CROSS", "TRIAD", "SPIRL", "SPINE",
  };
  return ((int)p >= 0 && (int)p < N_PATTERNS) ? names[p] : "?    ";
}

/* Turns three integers into one well-scrambled number — same inputs always give
 * the same result, but tiny input changes scatter the output. Used to drive the
 * star jitter, the name choices, and the backdrop sprinkle. */
static inline uint32_t hash3(int wx, int wy, int wz) {
  uint32_t h = (uint32_t)wx * 73856093u ^ (uint32_t)wy * 19349663u ^
               (uint32_t)wz * 83492791u;
  h ^= h >> 16;
  h *= 0x85ebca6bu;
  h ^= h >> 13;
  h *= 0xc2b2ae35u;
  h ^= h >> 16;
  return h;
}

/* The average position of all the stars — the middle of the cluster. */
static void star_centroid(const Star *stars, int n, int *cx, int *cy) {
  long sx = 0, sy = 0;
  for (int i = 0; i < n; i++) {
    sx += stars[i].x;
    sy += stars[i].y;
  }
  *cx = (int)(sx / n);
  *cy = (int)(sy / n);
}

/* Which star is closest to point (px,py); pass `except` to skip a star (-1 to
 * skip none, e.g. so a star doesn't match itself). */
static int nearest_to(const Star *stars, int n, int px, int py, int except) {
  int best = -1;
  long bd2 = (long)1 << 60;
  for (int i = 0; i < n; i++) {
    if (i == except)
      continue;
    long dx = stars[i].x - px, dy = stars[i].y - py;
    long d2 = dx * dx + dy * dy;
    if (d2 < bd2) {
      bd2 = d2;
      best = i;
    }
  }
  return best;
}

/* Is there already a line between stars a and b? (order doesn't matter). */
static bool has_edge(const Edge *edges, int n_edges, int a, int b) {
  for (int i = 0; i < n_edges; i++)
    if ((edges[i].from == a && edges[i].to == b) ||
        (edges[i].from == b && edges[i].to == a))
      return true;
  return false;
}

/* How many cells a line from (x0,y0) to (x1,y1) will fill — its length in
 * character cells, counting both endpoints. */
static inline int bres_steps(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  return (dx > dy ? dx : dy) + 1;
}

/* Picks the character that best matches the line's slant: '-' if mostly flat,
 * '|' if mostly upright, otherwise '/' or '\' for the two diagonals. */
static inline char line_glyph_for(int dx, int dy) {
  int adx = abs(dx), ady = abs(dy);
  if (adx >= 2 * ady)
    return '-';
  if (ady >= 2 * adx)
    return '|';
  if ((dx > 0) == (dy > 0))
    return '\\';
  return '/';
}

static const char *phase_name(Phase p) {
  switch (p) {
  case PHASE_DRAW_STARS:
    return "DISCOVER ";
  case PHASE_DRAW_EDGES:
    return "TRACE    ";
  case PHASE_HOLD:
    return "NAMED    ";
  case PHASE_FADE:
    return "FADING   ";
  default:
    return "?        ";
  }
}

/* Fade-in progress for something that starts at time `start` and takes `slice`
 * seconds: 0 before it starts, climbing smoothly to 1 by the end, 1 after.
 * Give each star/line a later start and they light up one after another, like a
 * wave sweeping across the figure. */
static inline float reveal_at(float now, float start, float slice) {
  if (now >= start + slice)
    return 1.0f;
  if (now > start)
    return (now - start) / slice;
  return 0.0f;
}

/* A small random nudge for placing a star: takes some hash bits and turns them
 * into an offset somewhere within +/-JITTER_FRAC of a cell's size. Done in
 * integer math (scale up by 1024, then back down) to stay off floats. */
static inline int jitter_offset(uint32_t hbits, int cell_size) {
  return ((int)(hbits & 0x3FFu) - 512) * cell_size * (int)(JITTER_FRAC * 1024) /
         (512 * 1024);
}

/* §4  simulation — the only code that changes anything */

/* Two jobs: build a whole new sky (constellation_gen, run once when the sky
 * changes), and nudge the reveal animation forward (scene_tick, run every
 * tick). Nothing else writes the simulation. Keypresses and resizes also change
 * state, but they happen in §8, outside the per-tick loop. */

/* Scatters n stars across the region. It lays a rough grid over the area, drops
 * one star per cell, then nudges each star off its cell centre by a small
 * random amount. The grid keeps them spread out (no clumps or gaps); the nudge
 * keeps it from looking like a grid. */
static void place_stars(Star *stars, int n, int region_w, int region_h,
                        int seed) {
  /* A roughly square grid with about n cells. */
  int cols = (int)ceilf(sqrtf((float)n));
  if (cols < 1)
    cols = 1;
  int rows = (n + cols - 1) / cols;

  int cell_w = region_w / cols;
  int cell_h = region_h / rows;
  if (cell_w < 1)
    cell_w = 1;
  if (cell_h < 1)
    cell_h = 1;

  /* One star per cell: start at the cell centre, nudge it, then clamp it inside
   * the region and pick its twinkle phase and tint from the same hash. */
  int idx = 0;
  for (int r = 0; r < rows && idx < n; r++) {
    for (int c = 0; c < cols && idx < n; c++) {
      uint32_t h = hash3(c, r, seed);
      int cx = c * cell_w + cell_w / 2;
      int cy = r * cell_h + cell_h / 2;
      stars[idx].x = cx + jitter_offset(h, cell_w);
      stars[idx].y = cy + jitter_offset(h >> 10, cell_h);
      if (stars[idx].x < 1)
        stars[idx].x = 1;
      if (stars[idx].x > region_w - 2)
        stars[idx].x = region_w - 2;
      if (stars[idx].y < 1)
        stars[idx].y = 1;
      if (stars[idx].y > region_h - 2)
        stars[idx].y = region_h - 2;
      stars[idx].twinkle_phase = (uint8_t)((h >> 24) & 0xFFu);
      stars[idx].color_idx = (uint8_t)((h >> 20) & 3u);
      stars[idx].reveal_t = 0.0f;
      idx++;
    }
  }
}

/* The fifteen line patterns. Each one takes the same stars and decides which
 * pairs to connect. Several sort the stars[] array in place first. */

/* Connects all the stars with the shortest total length of line and no loops (a
 * "minimum spanning tree", Prim 1957). It grows the tree one star at a time,
 * each step reaching out to whichever unconnected star is closest to the tree
 * so far. Result: an organic branching shape. */
static void make_tree(const Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 2)
    return;

  bool in_tree[N_STARS_MAX] = {false};
  in_tree[0] = true;

  for (int iter = 1; iter < n; iter++) {
    int best_from = -1, best_to = -1;
    long best_d2 = (long)1 << 60;
    for (int i = 0; i < n; i++) {
      if (!in_tree[i])
        continue;
      for (int j = 0; j < n; j++) {
        if (in_tree[j])
          continue;
        long dx = stars[i].x - stars[j].x;
        long dy = stars[i].y - stars[j].y;
        long d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
          best_d2 = d2;
          best_from = i;
          best_to = j;
        }
      }
    }
    if (best_to < 0)
      break;
    in_tree[best_to] = true;
    edges[*n_edges].from = best_from;
    edges[*n_edges].to = best_to;
    edges[*n_edges].reveal_t = 0.0f;
    (*n_edges)++;
    if (*n_edges >= N_EDGES_MAX)
      break;
  }
}

/* Reorders the stars left to right by their x position. A plain bubble sort:
 * with at most 12 stars it's plenty fast and saves the fuss of a qsort
 * comparator. */
static void sort_by_x(Star *stars, int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - 1 - i; j++) {
      if (stars[j].x > stars[j + 1].x) {
        Star t = stars[j];
        stars[j] = stars[j + 1];
        stars[j + 1] = t;
      }
    }
  }
}

/* Sorts the stars left to right and joins them in that order, like connect-the-
 * dots. Gives a single line that never crosses itself. */
static void make_chain(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 2)
    return;
  sort_by_x(stars, n);
  for (int i = 0; i < n - 1; i++) {
    edges[i].from = i;
    edges[i].to = i + 1;
    edges[i].reveal_t = 0.0f;
  }
  *n_edges = n - 1;
}

/* Reorders the stars by the angle each one sits at around the centre (cx,cy) —
 * going around like a clock. Afterwards, stepping through the array walks the
 * stars in a ring, so neighbours in the array are neighbours on the loop. */
static void sort_by_angle(Star *stars, int n, int cx, int cy) {
  float ang[N_STARS_MAX];
  for (int i = 0; i < n; i++)
    ang[i] = atan2f((float)(stars[i].y - cy), (float)(stars[i].x - cx));
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - 1 - i; j++) {
      if (ang[j] > ang[j + 1]) {
        float ta = ang[j];
        ang[j] = ang[j + 1];
        ang[j + 1] = ta;
        Star ts = stars[j];
        stars[j] = stars[j + 1];
        stars[j + 1] = ts;
      }
    }
  }
}

/* Orders the stars around the centre, then joins them into a closed ring (the
 * last star links back to the first). */
static void make_loop(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 3)
    return;
  int cx = 0, cy = 0;
  for (int i = 0; i < n; i++) {
    cx += stars[i].x;
    cy += stars[i].y;
  }
  cx /= n;
  cy /= n;
  sort_by_angle(stars, n, cx, cy);
  for (int i = 0; i < n; i++) {
    edges[i].from = i;
    edges[i].to = (i + 1) % n;
    edges[i].reveal_t = 0.0f;
  }
  *n_edges = n;
}

/* Picks the star nearest the middle as the hub, then draws a line from it to
 * every other star — like spokes on a wheel hub. */
static void make_spoke(const Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 2)
    return;
  int cx = 0, cy = 0;
  for (int i = 0; i < n; i++) {
    cx += stars[i].x;
    cy += stars[i].y;
  }
  cx /= n;
  cy /= n;
  int hub = 0;
  long bd2 = (long)1 << 60;
  for (int i = 0; i < n; i++) {
    long dx = stars[i].x - cx, dy = stars[i].y - cy;
    long d2 = dx * dx + dy * dy;
    if (d2 < bd2) {
      bd2 = d2;
      hub = i;
    }
  }
  int eidx = 0;
  for (int i = 0; i < n; i++) {
    if (i == hub)
      continue;
    edges[eidx].from = hub;
    edges[eidx].to = i;
    edges[eidx].reveal_t = 0.0f;
    eidx++;
    if (eidx >= N_EDGES_MAX)
      break;
  }
  *n_edges = eidx;
}

/* Adds one line between two stars (ignores a star-to-itself line and stops once
 * the array is full). */
static void add_edge(Edge *edges, int *n_edges, int from, int to) {
  if (*n_edges >= N_EDGES_MAX || from == to)
    return;
  edges[*n_edges].from = from;
  edges[*n_edges].to = to;
  edges[*n_edges].reveal_t = 0.0f;
  (*n_edges)++;
}

/* A loop around the outside plus spokes from the most central star: a wheel. */
static void make_wheel(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 4) {
    make_loop(stars, n, edges, n_edges);
    return;
  }
  int cx, cy;
  star_centroid(stars, n, &cx, &cy);
  sort_by_angle(stars, n, cx, cy);
  int hub = nearest_to(stars, n, cx, cy, -1);
  for (int i = 0; i < n; i++)
    add_edge(edges, n_edges, i, (i + 1) % n); /* rim */
  for (int i = 0; i < n; i++)
    if (i != hub && !has_edge(edges, *n_edges, hub, i))
      add_edge(edges, n_edges, hub, i); /* spokes */
}

/* The lowest star is the corner; lines fan out from it to all the rest — like a
 * hand fan. (Unlike SPOKE, the corner sits at the edge, not the middle.) */
static void make_fan(const Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 2)
    return;
  int apex = 0;
  for (int i = 1; i < n; i++)
    if (stars[i].y > stars[apex].y)
      apex = i;
  for (int i = 0; i < n; i++)
    add_edge(edges, n_edges, apex, i);
}

/* Goes around the ring but skips every other star (connect 0-2-4-...). With an
 * odd number of stars this traces one path whose lines cross through the middle
 * — a five- or seven-pointed star. (gen always feeds this an odd count.) */
static void make_starpoly(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 5) {
    make_loop(stars, n, edges, n_edges);
    return;
  }
  int cx, cy;
  star_centroid(stars, n, &cx, &cy);
  sort_by_angle(stars, n, cx, cy);
  for (int i = 0; i < n; i++)
    add_edge(edges, n_edges, i, (i + 2) % n);
}

/* Links each star to its single closest neighbour: a loose scatter of short
 * lines. */
static void make_nearest(const Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 2)
    return;
  for (int i = 0; i < n; i++) {
    int j = nearest_to(stars, n, stars[i].x, stars[i].y, i);
    if (j >= 0 && !has_edge(edges, *n_edges, i, j))
      add_edge(edges, n_edges, i, j);
  }
}

/* Links each star to its TWO closest neighbours: a fuller web than NEAREST. */
static void make_mesh(const Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 2)
    return;
  for (int i = 0; i < n; i++) {
    int n1 = -1, n2 = -1;
    long d1 = (long)1 << 60, d2 = (long)1 << 60;
    for (int j = 0; j < n; j++) {
      if (j == i)
        continue;
      long dx = stars[i].x - stars[j].x, dy = stars[i].y - stars[j].y;
      long dd = dx * dx + dy * dy;
      if (dd < d1) {
        d2 = d1;
        n2 = n1;
        d1 = dd;
        n1 = j;
      } else if (dd < d2) {
        d2 = dd;
        n2 = j;
      }
    }
    if (n1 >= 0 && !has_edge(edges, *n_edges, i, n1))
      add_edge(edges, n_edges, i, n1);
    if (n2 >= 0 && !has_edge(edges, *n_edges, i, n2))
      add_edge(edges, n_edges, i, n2);
  }
}

/* make_arch — angular order but an OPEN path (no closing edge): a graceful arc
 * or crown across the sky. */
static void make_arch(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 2)
    return;
  int cx, cy;
  star_centroid(stars, n, &cx, &cy);
  sort_by_angle(stars, n, cx, cy);
  for (int i = 0; i < n - 1; i++)
    add_edge(edges, n_edges, i, i + 1);
}

/* make_binary — split the field left/right and loop each half: two distinct
 * sub-constellations (a binary system). */
static void make_binary(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 4) {
    make_loop(stars, n, edges, n_edges);
    return;
  }
  sort_by_x(stars, n);
  int mid = n / 2;
  for (int i = 0; i < mid - 1; i++)
    add_edge(edges, n_edges, i, i + 1); /* left chain  */
  for (int i = mid; i < n - 1; i++)
    add_edge(edges, n_edges, i, i + 1); /* right chain */
  if (mid >= 3)
    add_edge(edges, n_edges, 0, mid - 1); /* close left  */
  if (n - mid >= 3)
    add_edge(edges, n_edges, mid, n - 1); /* close right */
}

/* make_cross — two interleaved chains (even- vs odd-ranked after x-sort) that
 * weave across each other, joined at the ends: a cross-hatch / lattice. */
static void make_cross(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 4) {
    make_chain(stars, n, edges, n_edges);
    return;
  }
  sort_by_x(stars, n);
  for (int i = 0; i + 2 < n; i += 2)
    add_edge(edges, n_edges, i, i + 2);
  for (int i = 1; i + 2 < n; i += 2)
    add_edge(edges, n_edges, i, i + 2);
  add_edge(edges, n_edges, 0, 1);
  add_edge(edges, n_edges, n - 2, n - 1);
}

/* make_triangles — apex = lowest star; angular-sort all, fan apex→each and
 * chain the rest, tiling the figure with triangles between adjacent rays. */
static void make_triangles(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 3) {
    make_chain(stars, n, edges, n_edges);
    return;
  }
  int cx, cy;
  star_centroid(stars, n, &cx, &cy);
  sort_by_angle(stars, n, cx, cy);
  int apex = 0;
  for (int i = 1; i < n; i++)
    if (stars[i].y > stars[apex].y)
      apex = i;
  for (int i = 0; i < n; i++)
    add_edge(edges, n_edges, apex, i); /* fan */
  for (int i = 0; i < n - 1; i++)
    if (i != apex && i + 1 != apex)
      add_edge(edges, n_edges, i, i + 1); /* arc → triangles */
}

/* make_spiral — order stars by radius from the centroid and connect inner →
 * outer: a single arm winding outward. */
static void make_spiral(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 2)
    return;
  int cx, cy;
  star_centroid(stars, n, &cx, &cy);
  for (int i = 1; i < n; i++) { /* insertion-sort by radius² */
    Star key = stars[i];
    long kr =
        (long)(key.x - cx) * (key.x - cx) + (long)(key.y - cy) * (key.y - cy);
    int j = i - 1;
    while (j >= 0) {
      long jr = (long)(stars[j].x - cx) * (stars[j].x - cx) +
                (long)(stars[j].y - cy) * (stars[j].y - cy);
      if (jr <= kr)
        break;
      stars[j + 1] = stars[j];
      j--;
    }
    stars[j + 1] = key;
  }
  for (int i = 0; i < n - 1; i++)
    add_edge(edges, n_edges, i, i + 1);
}

/* make_caterpillar — x-sorted spine of the even-ranked stars, with each
 * odd-ranked star hung off its nearest spine star as a leaf. */
static void make_caterpillar(Star *stars, int n, Edge *edges, int *n_edges) {
  *n_edges = 0;
  if (n < 3) {
    make_chain(stars, n, edges, n_edges);
    return;
  }
  sort_by_x(stars, n);
  int prev = -1;
  for (int i = 0; i < n; i += 2) { /* spine */
    if (prev >= 0)
      add_edge(edges, n_edges, prev, i);
    prev = i;
  }
  for (int i = 1; i < n; i += 2) { /* leaves → nearest spine star */
    int best = -1;
    long bd2 = (long)1 << 60;
    for (int j = 0; j < n; j += 2) {
      long dx = stars[i].x - stars[j].x, dy = stars[i].y - stars[j].y;
      long d2 = dx * dx + dy * dy;
      if (d2 < bd2) {
        bd2 = d2;
        best = j;
      }
    }
    if (best >= 0)
      add_edge(edges, n_edges, i, best);
  }
}

/* ----------------------------------------------------------------------- *
 * Procedural naming.                                                      *
 * ----------------------------------------------------------------------- */

/*
 * gen_name — concatenate three random fragments. The hash bits used
 * are far apart so prefix / suffix / modifier choices are
 * uncorrelated.
 */
static void gen_name(uint32_t hash, char *out, size_t len) {
  int p = (int)(hash % (uint32_t)N_PREFIXES);
  int s = (int)((hash >> 8u) % (uint32_t)N_SUFFIXES);
  int m = (int)((hash >> 16u) % (uint32_t)N_MODIFIERS);
  snprintf(out, len, "%s%s%s", PREFIXES[p], SUFFIXES[s], MODIFIERS[m]);
}

/* ----------------------------------------------------------------------- *
 * Top-level: regenerate everything for a new constellation.               *
 * ----------------------------------------------------------------------- */

/*
 * constellation_gen — regenerate the entire constellation from a
 * (pattern, seed) pair. The caller passes the desired star-region
 * dimensions; star positions land in [0, region_w) × [0, region_h)
 * and are rendered at gx0+x, gy0+y in the screen layout step.
 */
static void constellation_gen(Constellation *con, Pattern p, int seed,
                              int region_w, int region_h) {
  /* Pick N — slightly different per pattern so each topology
   * displays with the count it looks best at. */
  uint32_t h = hash3(seed, (int)p, 0);
  int n;
  switch (p) {
  case PATTERN_CHAIN:
  case PATTERN_LOOP:
    n = 5 + (int)(h % 4u);
    break; /* 5..8  */
  case PATTERN_SPOKE:
    n = 5 + (int)(h % 5u);
    break; /* 5..9  */
  case PATTERN_STARPOLY:
    n = 5 + 2 * (int)(h % 3u);
    break; /* 5,7,9 (odd) */
  case PATTERN_WHEEL:
  case PATTERN_MESH:
  case PATTERN_TRIANGLES:
    n = 6 + (int)(h % 3u);
    break; /* 6..8 (denser) */
  case PATTERN_BINARY:
  case PATTERN_CROSS:
    n = 8 + (int)(h % 4u);
    break; /* 8..11 (two parts) */
  default:
    n = 7 + (int)(h % 4u); /* 7..10 */
  }
  if (n > N_STARS_MAX)
    n = N_STARS_MAX;

  place_stars(con->stars, n, region_w, region_h, seed);
  con->n_stars = n;
  con->region_w = region_w;
  con->region_h = region_h;
  con->seed = seed;

  switch (p) {
  case PATTERN_TREE:
    make_tree(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_CHAIN:
    make_chain(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_LOOP:
    make_loop(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_SPOKE:
    make_spoke(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_WHEEL:
    make_wheel(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_FAN:
    make_fan(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_STARPOLY:
    make_starpoly(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_NEAREST:
    make_nearest(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_MESH:
    make_mesh(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_ARCH:
    make_arch(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_BINARY:
    make_binary(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_CROSS:
    make_cross(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_TRIANGLES:
    make_triangles(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_SPIRAL:
    make_spiral(con->stars, n, con->edges, &con->n_edges);
    break;
  case PATTERN_CATERPILLAR:
    make_caterpillar(con->stars, n, con->edges, &con->n_edges);
    break;
  default:
    con->n_edges = 0;
  }

  gen_name(hash3(seed, 42, 17), con->name, sizeof con->name);
}

/* Scene — the whole simulated-and-shown sky, as a table of contents: the
 * figure being revealed plus the knobs, view selections and animation clocks
 * that drive it. scene_tick() (§4) is its only per-tick writer; user events set
 * the knobs/selections. Fields group by concept, not by which key changes them.
 */
typedef struct {
  /* WHAT is simulated / shown */
  Constellation con; /* the figure: stars + edges + name          */
  int sky_seed;      /* seed for the faint backdrop sprinkle      */
  /* HOW the user drives it — the tunable simulation knob */
  int speed; /* reveal-animation pace, 1..SPEED_MAX        */
  /* WHAT we are looking at — RENDER selections, not simulation */
  int current_theme;       /* index into themes[] (t/T)                 */
  Pattern current_pattern; /* active topology (n/p); a switch rebuilds  */
  /* WHEN we are — the phase-machine clock + run-state */
  Phase phase;   /* DISCOVER / TRACE / HOLD / FADE            */
  float phase_t; /* seconds into the current phase            */
  float total_t; /* wall-clock accumulator (drives twinkle)   */
  bool paused;   /* freeze the reveal; rendering continues    */
} Scene;

/*
 * scene_rebuild — pick a fresh seed and build a new constellation in
 * the current pattern. Resets the phase machine to DRAW_STARS.
 */
static void scene_rebuild(Scene *s, int region_w, int region_h) {
  int seed = (int)hash3((int)(s->total_t * 1000.0f), region_w,
                        region_h ^ ((int)s->current_pattern << 8));
  constellation_gen(&s->con, s->current_pattern, seed, region_w, region_h);
  s->phase = PHASE_DRAW_STARS;
  s->phase_t = 0.0f;
  s->sky_seed = seed ^ 0x5A5A5A5A;
}

static void scene_init(Scene *s, int region_w, int region_h) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->current_theme = 0;
  s->current_pattern = PATTERN_TREE;
  scene_rebuild(s, region_w, region_h);
}

/*
 * scene_resize_to — region size has changed; throw away the current
 * layout and regenerate so positions are valid for the new region.
 */
static void scene_resize_to(Scene *s, int region_w, int region_h) {
  scene_rebuild(s, region_w, region_h);
}

/*
 * scene_tick — phase-machine update. The reveal_t fields on stars and
 * edges are computed from phase_t / phase_total so pause + resume keep
 * the animation perfectly aligned.
 */

/* Transition the phase machine to `next`, restarting the per-phase clock. */
static void enter_phase(Scene *s, Phase next) {
  s->phase = next;
  s->phase_t = 0.0f;
}

static void scene_tick(Scene *s, float dt) {
  s->total_t += dt; /* wall clock runs even when paused (twinkle) */
  if (s->paused)
    return;

  float speed_mul = (float)s->speed / (float)SPEED_DEF; /* speed knob → rate */
  s->phase_t += dt * speed_mul;

  Constellation *c = &s->con;

  switch (s->phase) {
  case PHASE_DRAW_STARS: { /* stars fade in, one staggered slice each */
    float slice = (c->n_stars > 0) ? (PHASE_STARS_TOTAL / (float)c->n_stars)
                                   : PHASE_STARS_TOTAL;
    for (int i = 0; i < c->n_stars; i++)
      c->stars[i].reveal_t = reveal_at(s->phase_t, i * slice, slice);
    if (s->phase_t >= PHASE_STARS_TOTAL)
      enter_phase(s, PHASE_DRAW_EDGES);
    break;
  }

  case PHASE_DRAW_EDGES: /* edges trace in, one after another */
    for (int i = 0; i < c->n_edges; i++)
      c->edges[i].reveal_t =
          reveal_at(s->phase_t, i * PHASE_EDGE_TIME, PHASE_EDGE_TIME);
    if (s->phase_t >= c->n_edges * PHASE_EDGE_TIME)
      enter_phase(s, PHASE_HOLD);
    break;

  case PHASE_HOLD: /* dwell with the name caption, then fade */
    if (s->phase_t >= PHASE_HOLD_TIME)
      enter_phase(s, PHASE_FADE);
    break;

  case PHASE_FADE: /* brief dwell, then a whole new sky */
    if (s->phase_t >= PHASE_FADE_TIME)
      scene_rebuild(s, c->region_w, c->region_h);
    break;
  }
}

/* ===================================================================== */
/* §5  EFFECTS  -- cosmetic-only state                                   */
/* ===================================================================== */

/* No EFFECTS layer. There is no stored cosmetic buffer: the star twinkle is
 * derived at render time from sinf(twinkle_phase + total_t) inside
 * draw_anchor_star (§7), and reveal_t (the fade-in / Bresenham trace progress)
 * is SIMULATION state — it decides WHAT is drawn, not a glow/trail on top. */

/* ===================================================================== */
/* §6  DELAYS  -- pauses, holds, timers                                  */
/* ===================================================================== */

/* No separate layer. The post-trace HOLD (the name dwell) and the brief FADE
 * before regenerating are two phases of the scene phase machine, timed by
 * phase_t against PHASE_HOLD_TIME / PHASE_FADE_TIME inside scene_tick() (§4).
 * The pause toggle (Scene.paused) early-returns scene_tick(). Both are woven
 * into the simulation tick. */

/* ===================================================================== */
/* §7  RENDER  -- state -> screen (reads only, never mutates sim)        */
/* ===================================================================== */

/* state -> screen. Reads Scene / Constellation / Screen and the §3 deciders;
 * writes ONLY the ncurses back buffer and the colour-pair table (theme_apply
 * / color_init at init). Never touches simulation state, so a frame can be
 * dropped or re-ordered with no effect on the sim. */

static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  if (COLORS >= 256) {
    const Theme *t = &themes[idx];
    init_pair(PAIR_BG_STAR, t->bg_star, -1);
    init_pair(PAIR_LINE, t->line, -1);
    init_pair(PAIR_NAME, t->name_color, -1);
    for (int i = 0; i < 4; i++)
      init_pair((short)(PAIR_STAR_BASE + i), t->star[i], -1);
  } else {
    static const short fb_star[4] = {COLOR_WHITE, COLOR_YELLOW, COLOR_CYAN,
                                     COLOR_BLUE};
    init_pair(PAIR_BG_STAR, COLOR_WHITE, -1);
    init_pair(PAIR_LINE, COLOR_WHITE, -1);
    init_pair(PAIR_NAME, COLOR_YELLOW, -1);
    for (int i = 0; i < 4; i++)
      init_pair((short)(PAIR_STAR_BASE + i), fb_star[i], -1);
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

/* ── Screen ────────────────────────────────────────────────────────────── *
 * The terminal viewport: its full size, the constellation region carved out of
 * it, and the offset where that region sits. Pure presentation geometry — holds
 * NO simulation state — recomputed by screen_layout() at startup and on every
 * SIGWINCH. A star at region cell (x,y) draws at terminal (gy0+y, gx0+x). The
 * region is ~80%×75% of the usable rows (rows 0-1 are the HUD, the last row the
 * hint line) so the name caption has room below it. */
typedef struct {
  int cols, rows;         /* full terminal size (getmaxyx)            */
  int region_w, region_h; /* constellation area carved out of it      */
  int gx0, gy0;           /* its top-left origin on screen            */
} Screen;

static void screen_layout(Screen *s) {
  int top = 2, bottom = s->rows - 1;
  int avail_h = bottom - top;
  int avail_w = s->cols;

  /* Constellation region — 80% wide × 75% tall, centred. Leaves
   * room for the name caption below. */
  int rw = (avail_w * 80) / 100;
  int rh = (avail_h * 75) / 100;
  if (rw < 16)
    rw = (avail_w < 16) ? avail_w : 16;
  if (rh < 8)
    rh = (avail_h < 8) ? avail_h : 8;
  if (rw > avail_w)
    rw = avail_w;
  if (rh > avail_h - 2)
    rh = avail_h - 2;
  if (rh < 4)
    rh = 4;

  s->region_w = rw;
  s->region_h = rh;
  s->gx0 = (avail_w - rw) / 2;
  s->gy0 = top + (avail_h - rh) / 2 - 1;
  if (s->gy0 < top)
    s->gy0 = top;
}

static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, s->rows, s->cols);
  screen_layout(s);
}

static void screen_free(Screen *s) {
  (void)s;
  endwin();
}

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
  screen_layout(s);
}

/*
 * draw_backdrop_sky — faint, twinkling sprinkle of dots throughout
 * the renderable region (excluding HUD rows). A per-cell hash gates
 * placement; a slow-time hash modulates intensity for the shimmer.
 */
static void draw_backdrop_sky(const Screen *sc, int sky_seed, float total_t) {
  int top = 2, bottom = sc->rows - 1;
  int twinkle_t = (int)(total_t * 1.5f);
  for (int y = top; y < bottom; y++) {
    for (int x = 0; x < sc->cols; x++) {
      uint32_t h = hash3(x, y, sky_seed);
      if ((h % BG_STAR_DENSITY) != 0)
        continue;
      char glyph = ((h >> 17) & 1u) ? '.' : '`';
      uint32_t h2 = hash3(x, y, sky_seed ^ twinkle_t);
      int attr = ((h2 % 7u) == 0u) ? A_NORMAL : A_DIM;
      attron(COLOR_PAIR(PAIR_BG_STAR) | attr);
      mvaddch(y, x, (chtype)(unsigned char)glyph);
      attroff(COLOR_PAIR(PAIR_BG_STAR) | attr);
    }
  }
}

/*
 * draw_partial_line — partial Bresenham from (x0,y0) to (x1,y1).
 * Plots the first ⌈reveal_t · total_steps⌉ cells; skips the two
 * endpoints (anchor stars draw themselves and look better undimmed).
 *
 * Coordinates are in REGION space; we add gx0/gy0 to land on screen.
 */
static void draw_partial_line(int x0, int y0, int x1, int y1, float reveal_t,
                              int gx0, int gy0, int rows, int cols, int pair,
                              int attr, char glyph) {
  int dx_abs = abs(x1 - x0);
  int dy_abs = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int dx = dx_abs;
  int dy = -dy_abs;
  int err = dx + dy;

  /* Of the line's `total_steps` cells, plot only the first `reveal_cells` —
   * the leading fraction reveal_t — so the line appears to grow from (x0,y0).
   */
  int total_steps = (dx_abs > dy_abs ? dx_abs : dy_abs) + 1;
  int reveal_cells = (reveal_t > 0.999f)
                         ? total_steps
                         : (int)ceilf(reveal_t * (float)total_steps);
  if (reveal_cells > total_steps)
    reveal_cells = total_steps;

  int x = x0, y = y0;
  int step = 0;
  while (step < reveal_cells) {
    bool is_endpoint = (x == x0 && y == y0) || (x == x1 && y == y1);
    if (!is_endpoint) {
      int sy_screen = gy0 + y;
      int sx_screen = gx0 + x;
      if (sy_screen >= 2 && sy_screen < rows - 1 && sx_screen >= 0 &&
          sx_screen < cols) {
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(sy_screen, sx_screen, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | attr);
      }
    }
    if (x == x1 && y == y1)
      break;
    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
    step++;
  }
}

/*
 * draw_anchor_star — bright '*' or 'O' at the star's position. The
 * star reveal_t fades the brightness in: 0 = invisible, 0.5 = dim,
 * 1.0 = full brightness with twinkle.
 */
static void draw_anchor_star(const Screen *sc, const Star *star,
                             float total_t) {
  if (star->reveal_t <= 0.05f)
    return;
  int sy = sc->gy0 + star->y;
  int sx = sc->gx0 + star->x;
  if (sy < 2 || sy >= sc->rows - 1)
    return;
  if (sx < 0 || sx >= sc->cols)
    return;

  int pair = PAIR_STAR_BASE + (star->color_idx & 3);
  int attr;
  char glyph;

  if (star->reveal_t < 0.40f) {
    attr = A_DIM;
    glyph = '.';
  } else if (star->reveal_t < 0.85f) {
    attr = A_NORMAL;
    glyph = '*';
  } else {
    /* Fully revealed — twinkle. The star pulses with a
     * sinusoid keyed to its individual phase so neighbours
     * twinkle out of sync. */
    float phase = (float)star->twinkle_phase / 255.0f * 2.0f * (float)M_PI;
    float b =
        0.5f + 0.5f * sinf(2.0f * (float)M_PI * TWINKLE_HZ * total_t + phase);
    attr = A_BOLD;
    glyph = (b > 0.65f) ? 'O' : '*';
  }

  attron(COLOR_PAIR(pair) | attr);
  mvaddch(sy, sx, (chtype)(unsigned char)glyph);
  attroff(COLOR_PAIR(pair) | attr);
}

/*
 * draw_name_label — render the constellation name centred below the
 * region, with a fade-in keyed to the phase. A tiny underline made
 * of '-' grows under it during the HOLD phase.
 */
static void draw_name_label(const Screen *sc, const Constellation *con,
                            Phase phase, float phase_t) {
  if (phase != PHASE_HOLD && phase != PHASE_FADE)
    return;

  int len = (int)strlen(con->name);
  int row = sc->gy0 + sc->region_h + 0;
  if (row >= sc->rows - 1)
    row = sc->rows - 2;
  int col = sc->gx0 + (sc->region_w - len) / 2;
  if (col < 0)
    col = 0;

  /* Fade in over first second of HOLD; fade out during FADE. */
  float intensity;
  if (phase == PHASE_HOLD) {
    intensity = (phase_t < 1.0f) ? phase_t : 1.0f;
  } else {
    intensity = 1.0f - (phase_t / PHASE_FADE_TIME);
    if (intensity < 0.0f)
      intensity = 0.0f;
  }
  if (intensity < 0.05f)
    return;

  int attr = (intensity > 0.7f) ? A_BOLD : A_NORMAL;
  attron(COLOR_PAIR(PAIR_NAME) | attr);
  mvprintw(row, col, "%s", con->name);
  attroff(COLOR_PAIR(PAIR_NAME) | attr);

  /* Underline that grows in during HOLD. */
  if (phase == PHASE_HOLD) {
    float ut = phase_t / 1.5f;
    if (ut > 1.0f)
      ut = 1.0f;
    int u_len = (int)(ut * (float)len);
    int u_row = row + 1;
    if (u_row < sc->rows - 1) {
      attron(COLOR_PAIR(PAIR_NAME) | A_DIM);
      for (int i = 0; i < u_len; i++) {
        int u_col = col + i;
        if (u_col >= 0 && u_col < sc->cols)
          mvaddch(u_row, u_col, '-');
      }
      attroff(COLOR_PAIR(PAIR_NAME) | A_DIM);
    }
  }
}

/* Render leaf, NOT a tick orchestrator: takes only what it draws — the figure
 * (Constellation), the sky seed, and the animation clocks/phase — never the
 * whole Scene. */
static void scene_draw(const Screen *sc, const Constellation *con, int sky_seed,
                       float total_t, Phase phase, float phase_t) {
  /* Backdrop first — faint sky everywhere. */
  draw_backdrop_sky(sc, sky_seed, total_t);

  /* Edges first (so anchor stars overdraw at vertices). */
  for (int i = 0; i < con->n_edges; i++) {
    const Edge *e = &con->edges[i];
    if (e->reveal_t <= 0.001f)
      continue;
    const Star *a = &con->stars[e->from];
    const Star *b = &con->stars[e->to];
    char glyph = line_glyph_for(b->x - a->x, b->y - a->y);
    int attr = (e->reveal_t > 0.99f) ? A_DIM : A_NORMAL;
    draw_partial_line(a->x, a->y, b->x, b->y, e->reveal_t, sc->gx0, sc->gy0,
                      sc->rows, sc->cols, PAIR_LINE, attr, glyph);
  }

  /* Anchor stars (overdrawing edge tails at vertices). */
  for (int i = 0; i < con->n_stars; i++) {
    draw_anchor_star(sc, &con->stars[i], total_t);
  }

  /* Name caption during HOLD / FADE. */
  draw_name_label(sc, con, phase, phase_t);
}

/* Row 0 right — primary status: fps, sim Hz, phase/pause, speed. Right-aligned.
 */
static void draw_status_line(const Screen *sc, const Scene *s, double fps,
                             int sim_fps) {
  const char *phase = s->paused ? "PAUSED   " : phase_name(s->phase);
  char buf[HUD_COLS + 1];
  snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz  %s  speed:%-3d ", fps, sim_fps,
           phase, s->speed);
  int hx = sc->cols - (int)strlen(buf);
  if (hx < 0)
    hx = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, hx, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Row 1 — pattern (with n/N counter), theme, the 4 anchor-star tint swatch, and
 * the constellation stats (N stars, E edges, name). Fixed left-aligned layout.
 */
static void draw_param_line(const Scene *s) {
  const Constellation *c = &s->con;
  int x = 1;

  char pbuf[40];
  snprintf(pbuf, sizeof pbuf, " pattern:%s %2d/%-2d ",
           pattern_name(s->current_pattern), (int)s->current_pattern + 1,
           (int)N_PATTERNS);
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(1, x, "%s", pbuf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  x += (int)strlen(pbuf);

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  x += 17;

  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(1, x, " stars:");
  attroff(COLOR_PAIR(PAIR_HUD));
  x += 7;
  for (int i = 0; i < 4; i++) { /* anchor-star tint swatch */
    attron(COLOR_PAIR(PAIR_STAR_BASE + i) | A_BOLD);
    mvaddch(1, x, '*');
    attroff(COLOR_PAIR(PAIR_STAR_BASE + i) | A_BOLD);
    x++;
  }
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(1, x, "  N=%d  E=%d  name:\"%s\" ", c->n_stars, c->n_edges, c->name);
  attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row — the key legend. Lists every interactive key (HUD standard). */
static void draw_hint(const Screen *sc) {
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(sc->rows - 1, 0,
           " n/p:pattern  t/T:theme  +/-:speed  ]/[:tickHz  spc:pause  r:regen "
           " q:quit ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* The one render function that takes the whole Scene (read-only): the HUD's
 * concept IS whole-scene status — pattern, theme, speed, phase, run-state — so
 * narrowing it would mean passing half a dozen scalars. A const read can't
 * re-couple the layers; scene_draw and the leaf draws stay narrow.
 * Reads as: draw the sky, then lay the HUD over it. */
static void screen_draw(const Screen *sc, const Scene *s, double fps,
                        int sim_fps) {
  erase();
  scene_draw(sc, &s->con, s->sky_seed, s->total_t, s->phase, s->phase_t);

  draw_status_line(sc, s, fps, sim_fps);

  /* Row 0 left — title. */
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, 1, " PROCEDURAL CONSTELLATION ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  draw_param_line(s);
  draw_hint(sc);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §8  APP  -- events + per-tick combine + main loop                     */
/* ===================================================================== */

/* Owns the App aggregate, signal flags, user-event handlers and the main
 * loop. main() is the ONE place that combines the layers per tick, in fixed
 * order:  scene_tick (SIM + DELAYS; twinkle is render-derived) -> screen_draw
 * (RENDER) -> screen_present -> input. app_handle_key() / app_do_resize()
 * mutate state on USER EVENTS and are deliberately OUTSIDE the tick. */

/* ── App ───────────────────────────────────────────────────────────────── *
 * Top-level harness binding the simulation (scene) to the terminal (screen),
 * plus the loop's PERFORMANCE knob and the async signal flags. A single static
 * instance (g_app) exists ONLY so the signal handlers — which may fire between
 * any two instructions — can reach the flags; everything else passes App
 * explicitly. Only init + the main loop touch it whole. The fixed-timestep rate
 * (sim_fps) decouples animation speed from frame rate (Fiedler, "Fix Your
 * Timestep!"; §8 main()). */
typedef struct {
  /* the two worlds it binds */
  Scene scene;   /* WHAT is simulated + shown        */
  Screen screen; /* WHERE it is drawn                */
  /* loop control */
  int sim_fps; /* fixed-timestep rate, SIM_FPS_* Hz*/
  /* volatile sig_atomic_t: written from signal handlers, so the compiler
   * must re-read them each loop and the write is atomic w.r.t. the
   * interrupted code. */
  volatile sig_atomic_t running;     /* cleared by SIGINT/SIGTERM → exit */
  volatile sig_atomic_t need_resize; /* set by SIGWINCH, served next loop*/
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
  screen_resize(&app->screen);
  scene_resize_to(&app->scene, app->screen.region_w, app->screen.region_h);
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
    scene_rebuild(s, app->screen.region_w, app->screen.region_h);
    break;

  case '=':
  case '+':
    if (s->speed < SPEED_MAX)
      s->speed *= 2;
    if (s->speed > SPEED_MAX)
      s->speed = SPEED_MAX;
    break;
  case '-':
    s->speed /= 2;
    if (s->speed < SPEED_MIN)
      s->speed = SPEED_MIN;
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

  case 't':
    s->current_theme = (s->current_theme + 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;
  case 'T':
    s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;

  /* Pattern switch — regenerate so the user sees the new topology
   * applied to a fresh constellation. */
  case 'n':
  case 'N':
    s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
    scene_rebuild(s, app->screen.region_w, app->screen.region_h);
    break;
  case 'p':
  case 'P':
    s->current_pattern =
        (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
    scene_rebuild(s, app->screen.region_w, app->screen.region_h);
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
  scene_init(&app->scene, app->screen.region_w, app->screen.region_h);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }

    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
