/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wireframe.c — spinning 3-D wireframe shapes in the terminal (cube, sphere,
 * pyramid, torus; Tab cycles them). No filled surfaces and no depth sorting — we
 * just rotate each shape, project its corners onto the screen, and draw a line
 * along every edge, with near edges bright and far ones dim for a sense of depth.
 *
 * The filled-surface cousins are the cube_raster / sphere_raster / torus_raster
 * files in this folder. Lines use Bresenham's algorithm (1962).
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

/* ── §1 settings — every number you can tweak ── */

enum {
  SIM_FPS_MIN = 5,
  SIM_FPS_DEFAULT = 30,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 5,

  HUD_COLS = 46,
  FPS_UPDATE_MS = 500,

  MAX_VERTS = 400,
  MAX_EDGES = 800,
  SHAPE_COUNT = 4,
};

/* CAM_DIST — how far back the camera sits (the shape is at the origin).
 * FILL     — how much of the smaller screen side the shape spans (0.82 ≈ full,
 *            with a small margin).
 * CELL_AR  — a terminal cell is about twice as tall as it is wide; feeding that
 *            in keeps circles round instead of squashed. */
#define CAM_DIST 5.0f
#define FILL 0.82f
#define CELL_AR 2.0f /* cell height ÷ width */

/* spin speed (radians/sec) and the limits the ] / [ keys move between */
#define ROT_X_DEF 0.50f
#define ROT_Y_DEF 0.85f
#define ROT_STEP 1.35f
#define ROT_MIN 0.01f
#define ROT_MAX 8.0f

#define ZOOM_DEFAULT 0.43f
#define ZOOM_STEP 1.15f
#define ZOOM_MIN 0.4f
#define ZOOM_MAX 3.5f

/* How many lines the round shapes are built from — deliberately few. A wireframe
 * needs just enough lines to read as the shape; too many overlap on the coarse
 * grid and it looks solid instead of see-through. */
#define SPHERE_STACKS 6 /* 5 visible latitude rings                */
#define SPHERE_SLICES 8 /* 8 longitude lines                       */
#define TORUS_MAJOR 12  /* 12 ring circles                         */
#define TORUS_MINOR 6   /* 6 tube circles                          */
#define TORUS_RING_R 0.65f /* ring radius — donut centre to tube centre */
#define TORUS_TUBE_R 0.28f /* tube radius — how fat the tube is         */

/* Depth cue — there's no lighting in a wireframe, so instead we vary edge
 * brightness by how near the camera each edge is: near edges bold, far edges
 * dim. On the coarse terminal grid this is what makes the shape read as solid
 * 3-D rather than a flat tangle of same-brightness lines. The fractions are
 * positions along the shape's own near→far depth range, recomputed per frame. */
#define DEPTH_NEAR_FRAC 0.66f /* nearer than this → bold          */
#define DEPTH_FAR_FRAC 0.33f  /* farther than this → dim; else normal */

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* ── §2 clock — a steady timer and a sleep, to pace the frames ── */

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

/* ── §3 colour — one colour per shape, plus switchable themes ── */

/* The ncurses colour-pair numbers: one per shape (1..4) plus the two HUD pairs.
 * Used as the `col` value stored in the Canvas and passed to canvas_set. */
typedef enum {
  COL_CUBE = 1,
  COL_SPHERE = 2,
  COL_PYRAMID = 3,
  COL_TORUS = 4,
  PAIR_HUD = 5,
  PAIR_HINT = 6,
} ShapeColor;

#define THEME_COUNT 6

/* One colour scheme, chosen with t/T — a RENDER concept. Holds the HUD label and
 * one 256-colour code per shape (cube, sphere, pyramid, torus); theme_apply
 * loads these into the four shape colour pairs. */
typedef struct {
  const char *display_name;
  short shape_256[4];
} Theme;

static const Theme THEMES[THEME_COUNT] = {
    /* CLASSIC — original four hues (cyan / green / yellow / magenta). */
    {"CLASSIC ", {51, 46, 226, 201}},
    /* AMBER   — warm phosphor monitor: bronze → gold → orange → amber. */
    {"AMBER   ", {130, 178, 208, 220}},
    /* MATRIX  — green data-stream: moss → emerald → lime → highlight. */
    {"MATRIX  ", {28, 46, 82, 154}},
    /* NEON    — synthwave magenta/pink ramp. */
    {"NEON    ", {165, 201, 207, 213}},
    /* ICE     — blues from teal → cyan → sky. */
    {"ICE     ", {39, 51, 87, 159}},
    /* COPPER  — bronze → copper-orange → gold. */
    {"COPPER  ", {130, 166, 208, 214}},
};

static void theme_apply(int theme_index) {
  if (theme_index < 0 || theme_index >= THEME_COUNT)
    theme_index = 0;
  const Theme *t = &THEMES[theme_index];

  if (COLORS >= 256) {
    init_pair(COL_CUBE, t->shape_256[0], COLOR_BLACK);
    init_pair(COL_SPHERE, t->shape_256[1], COLOR_BLACK);
    init_pair(COL_PYRAMID, t->shape_256[2], COLOR_BLACK);
    init_pair(COL_TORUS, t->shape_256[3], COLOR_BLACK);
  } else {
    /* 8-colour fallback — basic hues, theme-independent. */
    init_pair(COL_CUBE, COLOR_CYAN, COLOR_BLACK);
    init_pair(COL_SPHERE, COLOR_GREEN, COLOR_BLACK);
    init_pair(COL_PYRAMID, COLOR_YELLOW, COLOR_BLACK);
    init_pair(COL_TORUS, COLOR_MAGENTA, COLOR_BLACK);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();

  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }

  theme_apply(0); /* default to CLASSIC */
}

/* ── §4 vec3 — a 3-D point/vector ── */

typedef struct {
  float x, y, z;
} Vec3;

static inline Vec3 v3(float x, float y, float z) { return (Vec3){x, y, z}; }
static inline Vec3 v3mul(Vec3 a, float s) {
  return v3(a.x * s, a.y * s, a.z * s);
}

/* ── §5 project — rotate a point, then flatten it onto the screen ── */

/* Spin a point: first around the vertical axis (ry), then tip it around the
 * horizontal axis (rx). */
static Vec3 rot_yx(Vec3 p, float rx, float ry) {
  float cy = cosf(ry), sy = sinf(ry);
  float x1 = p.x * cy + p.z * sy;
  float z1 = -p.x * sy + p.z * cy;
  p.x = x1;
  p.z = z1;

  float cx = cosf(rx), sx = sinf(rx);
  float y2 = p.y * cx - p.z * sx;
  float z2 = p.y * sx + p.z * cx;
  p.y = y2;
  p.z = z2;

  return p;
}

/* A vertex after projection: where it lands on screen (col, row, in cells) plus
 * its world depth z. Depth is carried along so the renderer can skip points
 * behind the camera and shade near edges brighter than far ones. */
typedef struct {
  float col, row, z;
} ScreenPoint;

/* Flatten one 3-D point onto the screen: divide x and y by how far away the
 * point is, so nearer things spread out and farther ones bunch toward the
 * centre. ox/oy are the screen centre; fov_px sets the size. */
static ScreenPoint project_to_screen(Vec3 p, float fov_px, float ox, float oy) {
  float denom = CAM_DIST - p.z;
  if (denom < 0.01f)
    return (ScreenPoint){-1, -1, -9999}; /* at/behind the eye — caller skips it */

  float scale = fov_px / denom;

  float col = ox + p.x * scale;
  float row = oy - p.y * scale / CELL_AR; /* squeeze vertically so it's not oval */

  return (ScreenPoint){col, row, p.z};
}

/* Work out how big to draw so the shape fills about FILL of the screen — picking
 * whichever of width/height is the tighter fit, and re-doing it each frame so
 * resize and zoom both just work. */
static float fov_from_screen(int cols, int rows, float zoom) {
  /* These must match project_to_screen, which squeezes the row offset by CELL_AR.
   * So the row limit carries the ×CELL_AR and the column limit doesn't — get
   * this wrong and the shape fills only about half the screen. */
  float fov_rows = FILL * (float)rows * 0.5f * CAM_DIST * CELL_AR;
  float fov_cols = FILL * (float)cols * 0.5f * CAM_DIST;
  float fov = fov_rows < fov_cols ? fov_rows : fov_cols;
  return fov * zoom;
}

/* ── §6 canvas — the off-screen grid we draw into, then blit ── */

/*
 * Canvas — the off-screen drawing grid we render into before touching the
 * screen, sized to the exact terminal so the shape can use the whole thing.
 * Three parallel cell arrays: the glyph, its colour pair, and its depth
 * brightness (bold/normal/dim). canvas_draw blits the non-empty cells out.
 */
typedef struct {
  char *ch;        /* [rows * cols]  glyph, 0 = empty                  */
  ShapeColor *col; /* [rows * cols]  which colour pair                 */
  attr_t *attr;    /* [rows * cols]  depth brightness: bold/normal/dim */
  int cols;
  int rows;
} Canvas;

static void canvas_alloc(Canvas *c, int cols, int rows) {
  c->cols = cols;
  c->rows = rows;
  c->ch = calloc((size_t)(cols * rows), sizeof(char));
  c->col = calloc((size_t)(cols * rows), sizeof(ShapeColor));
  c->attr = calloc((size_t)(cols * rows), sizeof(attr_t));
}

static void canvas_free(Canvas *c) {
  free(c->ch);
  free(c->col);
  free(c->attr);
  *c = (Canvas){0};
}

static void canvas_clear(Canvas *c) {
  memset(c->ch, 0, sizeof(char) * (size_t)(c->cols * c->rows));
  memset(c->col, 0, sizeof(ShapeColor) * (size_t)(c->cols * c->rows));
  memset(c->attr, 0, sizeof(attr_t) * (size_t)(c->cols * c->rows));
}

static void canvas_set(Canvas *c, int x, int y, char ch, ShapeColor col,
                       attr_t attr) {
  if (x < 0 || x >= c->cols || y < 0 || y >= c->rows)
    return;
  int i = y * c->cols + x;
  c->ch[i] = ch;
  c->col[i] = col;
  c->attr[i] = attr;
}

/* Pick the line glyph that best matches an edge's direction: '-' near-flat,
 * '|' near-vertical, '/' or '\' for the two diagonals. Used for the flat shapes
 * (cube, pyramid); curved shapes pass a uniform 'o' instead so the slope
 * changing along each arc doesn't make the line look noisy. */
static char slope_glyph(int x0, int y0, int x1, int y1) {
  int adx = abs(x1 - x0), ady = abs(y1 - y0);
  if (adx == 0)
    return '|';
  if (ady == 0)
    return '-';
  float slope = (float)ady / (float)adx;
  if (slope < 0.5f)
    return '-';
  if (slope < 2.0f)
    return ((x0 < x1) == (y0 < y1)) ? '\\' : '/';
  return '|';
}

/* Draw a line between two cells (Bresenham, all directions). ch_override 0 means
 * pick the glyph by slope; non-zero (e.g. 'o') draws that glyph everywhere. */
static void canvas_line(Canvas *c, int x0, int y0, int x1, int y1,
                        char ch_override, ShapeColor col, attr_t attr) {
  char glyph = (ch_override != 0) ? ch_override : slope_glyph(x0, y0, x1, y1);

  int dx = abs(x1 - x0), dy = -abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  for (;;) {
    canvas_set(c, x0, y0, glyph, col, attr);
    if (x0 == x1 && y0 == y1)
      break;
    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

static void canvas_draw(const Canvas *c) {
  int total = c->cols * c->rows;
  for (int i = 0; i < total; i++) {
    char ch = c->ch[i];
    if (!ch)
      continue;

    int y = i / c->cols;
    int x = i % c->cols;

    attr_t attr = COLOR_PAIR(c->col[i]) | c->attr[i];
    attron(attr);
    mvaddch(y, x, (chtype)(unsigned char)ch);
    attroff(attr);
  }
}

/* ── §7 shapes — the corner + edge tables for each shape, built once ── */

/* A line to draw, as the two vertex-array indices it connects. */
typedef struct {
  int a, b;
} Edge;

/* A wireframe model: a list of corner points and a list of edges joining them,
 * plus how to draw it. That's the whole of a shape here — the pipeline never
 * asks "is this a sphere?", it just walks the edge list, so a new shape is only
 * a new builder that fills these tables. `curved` picks dots over line glyphs. */
typedef struct {
  const char *name;
  int nv; /* number of vertices in use */
  int ne; /* number of edges in use    */
  Vec3 verts[MAX_VERTS];
  Edge edges[MAX_EDGES];
  ShapeColor color;
  bool curved; /* true = dot rendering; false = slope-line chars */
} Shape;

/* ---- cube ---- */
static void shape_build_cube(Shape *s) {
  s->name = "cube";
  s->color = COL_CUBE;
  s->curved = false;
  s->nv = 8;
  s->ne = 12;

  for (int i = 0; i < 8; i++)
    s->verts[i] =
        v3((i & 1) ? 1.f : -1.f, (i & 2) ? 1.f : -1.f, (i & 4) ? 1.f : -1.f);

  Edge e[] = {
      {0, 1}, {1, 3}, {3, 2}, {2, 0}, /* front face  */
      {4, 5}, {5, 7}, {7, 6}, {6, 4}, /* back face   */
      {0, 4}, {1, 5}, {2, 6}, {3, 7}  /* pillars     */
  };
  memcpy(s->edges, e, sizeof e);
}

/* ---- sphere ---- */

/* One point on the unit sphere, like latitude/longitude on a globe: st runs
 * pole to pole, sl runs around. */
static Vec3 sphere_vertex(int st, int sl) {
  float phi = (float)M_PI * st / SPHERE_STACKS;      /* latitude  */
  float th = 2.f * (float)M_PI * sl / SPHERE_SLICES; /* longitude */
  return v3(sinf(phi) * cosf(th), cosf(phi), sinf(phi) * sinf(th));
}

static void shape_build_sphere(Shape *s) {
  s->name = "sphere";
  s->color = COL_SPHERE;
  s->curved = true;
  s->nv = 0;
  s->ne = 0;

  int ST = SPHERE_STACKS, SL = SPHERE_SLICES;

  /* a vertex at every (latitude, longitude) grid point */
  for (int st = 0; st <= ST; st++)
    for (int sl = 0; sl < SL; sl++)
      s->verts[s->nv++] = sphere_vertex(st, sl);

  /* longitude lines (pole to pole) */
  for (int sl = 0; sl < SL; sl++)
    for (int st = 0; st < ST; st++)
      s->edges[s->ne++] = (Edge){st * SL + sl, (st + 1) * SL + sl};
  /* latitude rings (around) */
  for (int st = 1; st < ST; st++)
    for (int sl = 0; sl < SL; sl++)
      s->edges[s->ne++] = (Edge){st * SL + sl, st * SL + (sl + 1) % SL};
}

/* ---- pyramid ---- */
static void shape_build_pyramid(Shape *s) {
  s->name = "pyramid";
  s->color = COL_PYRAMID;
  s->curved = false;
  s->nv = 5;
  s->ne = 8;

  s->verts[0] = v3(-1, -1, -1);
  s->verts[1] = v3(1, -1, -1);
  s->verts[2] = v3(1, -1, 1);
  s->verts[3] = v3(-1, -1, 1);
  s->verts[4] = v3(0, 1, 0);

  Edge e[] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {0, 4}, {1, 4}, {2, 4}, {3, 4}};
  memcpy(s->edges, e, sizeof e);
}

/* ---- torus ---- */

/* One point on the torus surface from its grid coords: i goes around the ring
 * (the donut hole), j around the tube. `ring` is how far this tube point sits
 * from the centre axis. */
static Vec3 torus_vertex(int i, int j) {
  float phi = 2.f * (float)M_PI * i / TORUS_MAJOR; /* around the ring */
  float th = 2.f * (float)M_PI * j / TORUS_MINOR;  /* around the tube */
  float ring = TORUS_RING_R + TORUS_TUBE_R * cosf(th);
  return v3(ring * cosf(phi), TORUS_TUBE_R * sinf(th), ring * sinf(phi));
}

static void shape_build_torus(Shape *s) {
  s->name = "torus";
  s->color = COL_TORUS;
  s->curved = true;
  s->nv = 0;
  s->ne = 0;

  int M = TORUS_MAJOR, m = TORUS_MINOR;

  /* a vertex at every (ring, tube) grid point */
  for (int i = 0; i < M; i++)
    for (int j = 0; j < m; j++)
      s->verts[s->nv++] = torus_vertex(i, j);

  /* join each vertex to its neighbour around the tube and around the ring */
  for (int i = 0; i < M; i++)
    for (int j = 0; j < m; j++) {
      s->edges[s->ne++] = (Edge){i * m + j, i * m + (j + 1) % m};   /* tube  */
      s->edges[s->ne++] = (Edge){i * m + j, ((i + 1) % M) * m + j}; /* ring  */
    }
}

static const char *const k_names[SHAPE_COUNT] = {"cube", "sphere", "pyramid",
                                                 "torus"};

/* ── §8 scene — the whole world, the per-frame spin, and the draw ── */

/* Everything the program tracks, in one place:
 *   WHAT   — the four shapes, which one is showing, and how far it has turned.
 *   HOW    — the knobs the user dials: spin speed, zoom, and pause.
 *   RENDER — the off-screen canvas and the chosen colour theme (key-driven, but
 *            rendering choices, not simulation knobs — so grouped apart).
 * Only the scene_* orchestrators take a whole Scene*; the math, canvas, and
 * shape functions take just the piece they need. */
typedef struct {
  /* WHAT is shown */
  Shape shapes[SHAPE_COUNT];
  int active;         /* which shape (Tab cycles)       */
  float rx, ry;       /* how far it has turned, radians  */

  /* HOW the user drives it */
  float rot_x, rot_y; /* spin speed, radians/sec ([ ] keys) */
  float zoom;         /* shape size (z/Z keys)              */
  bool paused;        /* freeze the spin (space)            */

  /* RENDER state */
  Canvas canvas;      /* the off-screen drawing grid     */
  int theme_index;    /* which colour theme (t/T keys)   */
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  shape_build_cube(&s->shapes[0]);
  shape_build_sphere(&s->shapes[1]);
  shape_build_pyramid(&s->shapes[2]);
  shape_build_torus(&s->shapes[3]);

  s->active = 0;
  s->rx = 0.4f;
  s->ry = 0.6f;
  s->rot_x = ROT_X_DEF;
  s->rot_y = ROT_Y_DEF;
  s->zoom = ZOOM_DEFAULT;
  s->theme_index = 0; /* CLASSIC */
  s->paused = false;

  canvas_alloc(&s->canvas, cols, rows);
}

static void scene_free(Scene *s) { canvas_free(&s->canvas); }

static void scene_resize(Scene *s, int cols, int rows) {
  canvas_free(&s->canvas);
  canvas_alloc(&s->canvas, cols, rows);
}

/* The one thing that changes over time: turn the shape a little. */
static void scene_tick(Scene *s, float dt_sec) {
  if (s->paused)
    return;
  s->rx += s->rot_x * dt_sec;
  s->ry += s->rot_y * dt_sec;
}

/* Map an edge's nearness (0 = farthest back, 1 = nearest) to a brightness, so
 * near edges read bold and far ones dim — the wireframe's stand-in for lighting. */
static attr_t depth_to_attr(float near_frac) {
  return near_frac > DEPTH_NEAR_FRAC  ? A_BOLD
         : near_frac < DEPTH_FAR_FRAC ? A_DIM
                                      : A_NORMAL;
}

/*
 * Draw the current shape into the off-screen canvas: size it to the screen,
 * rotate and project every corner (noting how near/far they spread), then draw
 * a line along each edge — dimmer the farther back it sits.
 */
static void scene_render(Scene *s) {
  canvas_clear(&s->canvas);

  int cols = s->canvas.cols;
  int rows = s->canvas.rows;
  float ox = (float)cols * 0.5f;
  float oy = (float)rows * 0.5f;
  float fov = fov_from_screen(cols, rows, s->zoom);

  const Shape *sh = &s->shapes[s->active];

  /* rotate + project every corner, and note the nearest/farthest depth so the
   * brightness cue can stretch across whatever range this shape spans */
  ScreenPoint proj[MAX_VERTS];
  float zmin = 1e9f, zmax = -1e9f;
  for (int i = 0; i < sh->nv; i++) {
    Vec3 v = rot_yx(sh->verts[i], s->rx, s->ry);
    proj[i] = project_to_screen(v, fov, ox, oy);
    if (proj[i].z > -9000.f) { /* ignore the off-screen marker (-9999) */
      if (proj[i].z < zmin)
        zmin = proj[i].z;
      if (proj[i].z > zmax)
        zmax = proj[i].z;
    }
  }
  float zspan = (zmax > zmin) ? (zmax - zmin) : 1.f;

  /* flat shapes get slope-matched line glyphs; curved shapes get plain dots */
  char ch_override = sh->curved ? 'o' : 0;

  for (int e = 0; e < sh->ne; e++) {
    ScreenPoint pa = proj[sh->edges[e].a];
    ScreenPoint pb = proj[sh->edges[e].b];

    if (pa.z < -CAM_DIST + 0.1f || pb.z < -CAM_DIST + 0.1f)
      continue; /* an endpoint is at/behind the camera — skip this edge */

    float near_frac = ((pa.z + pb.z) * 0.5f - zmin) / zspan; /* 0 far … 1 near */

    canvas_line(&s->canvas, (int)(pa.col + 0.5f), (int)(pa.row + 0.5f),
                (int)(pb.col + 0.5f), (int)(pb.row + 0.5f), ch_override,
                sh->color, depth_to_attr(near_frac));
  }
}

static void scene_draw(const Scene *s) { canvas_draw(&s->canvas); }

/* ── §9 screen — set up the terminal, draw the HUD, show the frame ── */

/* The terminal we draw on — just its size in cells, re-queried on resize. Kept
 * apart from Scene so the canvas can be re-sized from it without the render code
 * reaching into scene state. */
typedef struct {
  int cols;
  int rows;
} Screen;

static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1); /* don't let waiting keypresses interrupt drawing (avoids tearing) */
  color_init();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) {
  (void)s;
  endwin();
}

/* The endwin()+refresh() pair is what makes ncurses notice the new window size. */
static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(Screen *s, const Scene *sc, double fps) {
  erase();
  scene_draw(sc);

  /* Top row — yellow status (right-aligned) + title (left). */
  char status[160];
  snprintf(status, sizeof status,
           " %5.1f fps  %-7s  spd:%.2f  zoom:%.2f  theme:%s  %s ", fps,
           k_names[sc->active], sc->rot_y, sc->zoom,
           THEMES[sc->theme_index].display_name,
           sc->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status);
  if (slen > s->cols)
    slen = s->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, s->cols - slen, "%s", status);
  mvprintw(0, 0, " WIREFRAME ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* Bottom row — cyan key hint. */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q:quit  spc:pause  Tab:shape  ]/[:spin  z/Z:zoom  t/T:theme ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §10 app — wire it together: set up, run the loop, handle keys ── */

/* The top-level program: the scene, the terminal it's drawn on, the frame-rate
 * throttle, and two flags the signal handlers flip (time to quit, window
 * resized). Harness glue the main loop drives — not part of the shapes. */
typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
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

/* Handle a window resize — happens between frames, not part of the animation. */
static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

/* Handle one key press — each is a one-off change, separate from the animation. */
static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {

  case 'q':
  case 'Q':
  case 27:
    return false;

  case '\t':
    s->active = (s->active + 1) % SHAPE_COUNT;
    s->rx = 0.4f;
    s->ry = 0.6f;
    break;

  case ' ':
    s->paused = !s->paused;
    break;

  case ']':
    s->rot_x *= ROT_STEP;
    s->rot_y *= ROT_STEP;
    if (s->rot_x > ROT_MAX)
      s->rot_x = ROT_MAX;
    if (s->rot_y > ROT_MAX)
      s->rot_y = ROT_MAX;
    break;
  case '[':
    s->rot_x /= ROT_STEP;
    s->rot_y /= ROT_STEP;
    if (s->rot_x < ROT_MIN)
      s->rot_x = ROT_MIN;
    if (s->rot_y < ROT_MIN)
      s->rot_y = ROT_MIN;
    break;

  case 'z':
    /* zoom in — make the shape bigger */
    s->zoom *= ZOOM_STEP;
    if (s->zoom > ZOOM_MAX)
      s->zoom = ZOOM_MAX;
    break;
  case 'Z':
    /* zoom out — make the shape smaller */
    s->zoom /= ZOOM_STEP;
    if (s->zoom < ZOOM_MIN)
      s->zoom = ZOOM_MIN;
    break;

  case 't':
    s->theme_index = (s->theme_index + 1) % THEME_COUNT;
    theme_apply(s->theme_index);
    break;
  case 'T':
    s->theme_index = (s->theme_index + THEME_COUNT - 1) % THEME_COUNT;
    theme_apply(s->theme_index);
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
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

    /* window resized? rebuild for the new size before anything else */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* One frame, in order: measure time, spin, draw, pace, show, read input. */

    /* how long since the last frame? cap it so a stall can't make the spin jump */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    /* advance the spin in fixed steps, catching up however many fit in dt */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }
    float alpha = (float)sim_accum / (float)tick_ns;
    (void)alpha;

    /* draw the shape into the off-screen canvas */
    scene_render(&app->scene);

    /* update the fps reading, then sleep off the rest of the frame */
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

    /* put the canvas + HUD on screen */
    screen_draw(&app->screen, &app->scene, fps_display);
    screen_present();

    /* read one keypress, if any, and act on it */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  scene_free(&app->scene);
  screen_free(&app->screen);
  return 0;
}
