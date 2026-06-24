/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * kaboom.c — an ASCII explosion that plays in your terminal.
 *
 * Each blast is two layers stacked on top of each other: a flat
 * shockwave ring that ripples outward, and a cloud of little 3-D
 * debris chunks that fly toward you and shrink into the distance.
 * Cycle the look with t/n keys; everything is drawn with plain
 * letters so it renders the same on any terminal.
 *
 * The 3-D debris idea comes from particle systems (Reeves 1983,
 * "Particle Systems," ACM TOG 2(2)). The "make far things smaller"
 * trick is textbook pinhole-camera projection (Foley et al.,
 * Computer Graphics: Principles and Practice). Sister files in this
 * project: fireworks, matrix_rain.
 */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_1_PI
#define M_1_PI (1.0 / M_PI)
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

/* ── §1 config ── */

enum {
  SIM_FPS_MIN = 5,
  SIM_FPS_DEFAULT = 30,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 5,

  NUM_FRAMES = 150,
  NUM_BLOBS = 800,

  HUD_COLS = 28,
  FPS_UPDATE_MS = 500,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(fps) (NS_PER_SEC / (fps))

#define PERSPECTIVE 50.0

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

/* ── §3 color ── */

typedef enum {
  COL_FLASH = 1,
  COL_INNER = 2,
  COL_WAVE = 3,
  COL_BLOB_F = 4,
  COL_BLOB_M = 5,
  COL_BLOB_N = 6,
  COL_HUD = 7,  /* top status bar — bright yellow */
  COL_HINT = 8, /* bottom key hints — bright cyan */
} ColorID;

/*
 * BlastTheme — the set of colours one explosion is painted in.
 *
 * A theme picks a colour for each part of the blast: the bright
 * fireball, the bright core, the fading wave, and the near/mid/far
 * debris. The main numbers are 256-colour terminal codes; the f8_*
 * fields are a backup set of 8 basic colours for older terminals that
 * can't do 256 (color_theme_apply chooses which set to use). The
 * background is always black so the glyphs stand out.
 *
 * The parts, hottest to coolest:
 *   FLASH   the very first fireball burst — usually pure white so it punches.
 *   INNER   the bright core just behind the wave's leading edge.
 *   WAVE    the fading trail behind the shockwave.
 *   BLOB_F  far debris (drawn as '.') — usually pale and washed out.
 *   BLOB_M  mid-distance debris (drawn as 'o').
 *   BLOB_N  close debris (drawn as '@') — usually the boldest colour.
 *
 * Fields:
 *   name      short label shown in the status bar (kept short to fit).
 *   flash     colour of the fireball; nearly always white so the eye
 *             jumps to the centre no matter the theme.
 *   inner     the bright body of the wave — the colour you'd name the
 *             theme by (green for MATRIX, orange for FIRE, and so on).
 *   wave      the trailing edge; usually a darker, cooler version of
 *             inner so the wave looks like it's fading as it spreads.
 *   blob_f    far debris colour; often pale so distant bits look like
 *             tiny bright specks.
 *   blob_m    mid-distance debris colour.
 *   blob_n    close debris colour; usually deep and saturated so near
 *             chunks feel heavy and up-front.
 *   f8_flash  8-colour backup for flash (usually white).
 *   f8_inner  8-colour backup for inner.
 *   f8_wave   8-colour backup for wave.
 *   f8_bm     8-colour backup for mid debris.
 *   f8_bn     8-colour backup for near debris. (Far debris just reuses
 *             f8_flash — the 8-colour set has no pale option.)
 *
 * Themes, in the order t cycles through them (T goes backwards):
 *   0 MATRIX   — digital green rain: white flash, lime inner, dark green wave
 *   1 FIRE     — classic orange/red: white flash, orange inner, amber wave
 *   2 OCEANIC  — deep sea: white flash, pale aqua inner, teal wave
 *   3 NEON     — retro arcade: white flash, hot pink inner, purple wave
 *   4 MONO     — grayscale: white flash through gray ramp
 *   5 ICE      — frozen: white flash, bright cyan inner, blue wave
 *   6 NOVA     — supernova: white flash, yellow inner, orange-red wave
 *   7 FOREST   — woodland: cream flash, lime inner, dark olive wave
 *   8 DESERT   — sand storm: cream flash, sandy peach inner, brown wave
 *   9 ECLIPSE  — bloodmoon: white flash, orange inner, dark red wave
 */
typedef struct {
  const char *name;
  int flash, inner, wave, blob_f, blob_m, blob_n; /* 256-color codes */
  int f8_flash, f8_inner, f8_wave, f8_bm, f8_bn;  /* 8-color backups */
} BlastTheme;

static const BlastTheme k_themes[] = {
    /* name       flash inner wave  blob_f blob_m blob_n   8-color: flash inner
       wave           bm             bn */
    {"MATRIX", 231, 118, 40, 250, 154, 46, COLOR_WHITE, COLOR_GREEN,
     COLOR_GREEN, COLOR_GREEN, COLOR_GREEN},
    {"FIRE", 231, 214, 94, 250, 220, 196, COLOR_WHITE, COLOR_YELLOW, COLOR_RED,
     COLOR_YELLOW, COLOR_RED},
    {"OCEANIC", 231, 159, 31, 195, 87, 39, COLOR_WHITE, COLOR_CYAN, COLOR_BLUE,
     COLOR_CYAN, COLOR_BLUE},
    {"NEON", 231, 201, 93, 219, 207, 165, COLOR_WHITE, COLOR_MAGENTA,
     COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA},
    {"MONO", 231, 253, 245, 251, 247, 244, COLOR_WHITE, COLOR_WHITE,
     COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
    {"ICE", 231, 51, 27, 195, 123, 39, COLOR_WHITE, COLOR_CYAN, COLOR_BLUE,
     COLOR_CYAN, COLOR_BLUE},
    {"NOVA", 231, 226, 202, 255, 220, 208, COLOR_WHITE, COLOR_YELLOW,
     COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW},
    {"FOREST", 230, 154, 64, 230, 184, 70, COLOR_WHITE, COLOR_GREEN,
     COLOR_GREEN, COLOR_YELLOW, COLOR_GREEN},
    {"DESERT", 230, 223, 130, 230, 215, 172, COLOR_WHITE, COLOR_YELLOW,
     COLOR_RED, COLOR_YELLOW, COLOR_RED},
    {"ECLIPSE", 231, 208, 52, 250, 202, 88, COLOR_WHITE, COLOR_RED, COLOR_RED,
     COLOR_RED, COLOR_RED},
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

/* Switch to a new theme's colours. Safe to call any time — the change
 * shows up on the next frame. The status bar stays yellow in every
 * theme so it's always easy to read. */
static void color_theme_apply(int t) {
  const BlastTheme *th = &k_themes[t];
  if (COLORS >= 256) {
    init_pair(COL_FLASH, th->flash, COLOR_BLACK);
    init_pair(COL_INNER, th->inner, COLOR_BLACK);
    init_pair(COL_WAVE, th->wave, COLOR_BLACK);
    init_pair(COL_BLOB_F, th->blob_f, COLOR_BLACK);
    init_pair(COL_BLOB_M, th->blob_m, COLOR_BLACK);
    init_pair(COL_BLOB_N, th->blob_n, COLOR_BLACK);
    init_pair(COL_HUD, 226, COLOR_BLACK);
    init_pair(COL_HINT, 51, COLOR_BLACK);
  } else {
    init_pair(COL_FLASH, th->f8_flash, COLOR_BLACK);
    init_pair(COL_INNER, th->f8_inner, COLOR_BLACK);
    init_pair(COL_WAVE, th->f8_wave, COLOR_BLACK);
    init_pair(COL_BLOB_F, th->f8_flash, COLOR_BLACK);
    init_pair(COL_BLOB_M, th->f8_bm, COLOR_BLACK);
    init_pair(COL_BLOB_N, th->f8_bn, COLOR_BLACK);
    init_pair(COL_HUD, COLOR_YELLOW, COLOR_BLACK);
    init_pair(COL_HINT, COLOR_CYAN, COLOR_BLACK);
  }
}

static void color_init(int theme) {
  start_color();
  color_theme_apply(theme);
}

/* ── §4 blob ── */

/*
 * Blob — one chunk of flying 3-D debris.
 *
 * (x, y, z) is just a direction pointing out from the blast centre,
 * picked once at the start of each explosion (we scatter points
 * roughly evenly over a sphere). A blob never moves on its own — each
 * frame we work out where it should be by stretching its direction
 * outward over time, so the whole cloud is read-only after setup and
 * costs nothing to keep around.
 *
 * Why store all three of x, y, z when a blob is just one character on
 * screen? Because z (depth) is what makes the explosion look 3-D:
 *   - it decides how big the blob looks — closer blobs spread out
 *     wider, far ones bunch toward the centre (the "make far things
 *     smaller" effect).
 *   - it picks the character: far chunks show as '.', mid as 'o',
 *     near as '@'.
 * Drop z and every blob would fly out at the same rate and the burst
 * would look like flat 2-D fireworks. The y value also gets squashed
 * or stretched per shape at draw time, so the very same sphere of
 * blobs can read as a flat disc, a round ball, or a tall column.
 */
typedef struct {
  double x, y, z;
} Blob;

/* A tiny, fast random-number generator giving values in about
 * [-1, 1). It's only good enough for visual jitter — don't use it for
 * anything that needs real randomness. It keeps its own counter, so it
 * never resets, which is why each replayed blast looks a little
 * different from the last. */
static double prng(void) {
  static long long s = 1;
  s = s * 1488248101LL + 981577151LL;
  return ((s % 65536) - 32768) / 32768.0;
}

/* Pick one random outward direction for a debris chunk.
 * Grab a random point inside a cube, then pull it onto the surface of
 * a ball so it becomes a pure direction. We halve its height so the
 * cloud is a bit squashed (terminal characters are taller than they
 * are wide, so a true ball would look stretched), and give each chunk
 * a slightly different length so they don't all fly out at the exact
 * same speed. */
static void blob_sample_unit_direction(Blob *out) {
  double bx = prng();
  double by = prng();
  double bz = prng();
  double br = sqrt(bx * bx + by * by + bz * bz);
  out->x = (bx / br) * (1.3 + 0.2 * prng());
  out->y = (0.5 * by / br) * (1.3 + 0.2 * prng());
  out->z = (bz / br) * (1.3 + 0.2 * prng());
}

/* Give every debris chunk its own random direction. */
static void blob_init_pool(Blob *blobs) {
  for (int i = 0; i < NUM_BLOBS; i++)
    blob_sample_unit_direction(&blobs[i]);
}

/*
 * BlastShape — the dials that make one explosion look different from
 * the next. The drawing code reads these each frame but never has any
 * "if shape is X" branches — same code, six very different blasts.
 * Switch shapes with the n/N keys.
 *
 *   name        short label shown in the status bar.
 *   petal_n     how many points/lobes the ring has. 0 = a smooth
 *               round ring; 4 = a cross; 6 = a six-pointed star;
 *               higher = spikier. (It's a double only because the cos
 *               function wants one.)
 *   ripple      how deep those points cut in. 0 = perfectly smooth;
 *               around 0.3 = gentle lobes; 0.6 = jagged and pulsing.
 *   disc_speed  how fast the opening fireball grows before the wave
 *               takes over. Small = a slow puff; large = an instant
 *               flash.
 *   y_squash    how much the debris cloud is squashed or stretched
 *               vertically. 0.3 = a flat puck; 1.0 = a round ball;
 *               1.6 = a tall column. This is what lets the one shared
 *               cloud of blobs read as totally different silhouettes.
 *   persp       how strong the 3-D depth effect is. Small = flat,
 *               every chunk looks about the same size; large = lots of
 *               depth, near chunks balloon and far ones shrink to dots.
 *   blob_speed  how fast the debris flies outward. Small = a slow
 *               drift; large = a hard blast. Tune alongside persp so
 *               the chunks don't all shoot off-screen before the wave
 *               catches up.
 *   flash_chars the characters used for the bright core right behind
 *               the wave, brightest-looking first. A longer string
 *               makes the bright centre linger longer.
 *   wave_chars  the characters used for the spreading shockwave,
 *               faint leading edge first and dense trail last. As the
 *               wave sweeps a cell it steps through this whole string.
 *               A longer string makes a thicker wave.
 */
typedef struct {
  const char *name;
  double petal_n;
  double ripple;
  double disc_speed;
  double y_squash;
  double persp;
  double blob_speed;
  const char *flash_chars;
  const char *wave_chars;
} BlastShape;

static const BlastShape k_shapes[] = {
    {/* 0  classic — the default all-rounder */
     "classic", 16.0, 0.3, 2.0, 0.5, 50.0, 1.0, "T%@W#H=+~-:.",
     " .:!HIOMW#%$&@08O=+-"},
    {/* 1  star — 6-pointed, slow thick wave, tall blob column */
     "star", 6.0, 0.45, 1.5, 1.6, 35.0, 0.8, "*+oO0@#%&$!^~",
     " `.-:=+*oO0#@%$"},
    {/* 2  ring — smooth sphere, fast thin ring, flat disc blobs */
     "ring", 0.0, 0.0, 3.0, 0.3, 70.0, 1.4, "o0OQ@#%&$()[]{}",
     " .,:;!|/\\-=+~*oO"},
    {/* 3  cross — 4 lobes, medium speed, medium depth */
     "cross", 4.0, 0.5, 2.5, 0.8, 45.0, 1.1, "#@WMH+|=~-:.", " :-=+|H#@WM0O%$"},
    {/* 4  nova — 12 lobes, very fast, deep 3D blobs */
     "nova", 12.0, 0.35, 3.5, 1.0, 80.0, 1.6, "%$&#@!*+~-:.",
     " .`'^-~=+*#@$%&!"},
    {/* 5  pulse — asymmetric teardrop, slow, very flat blobs */
     "pulse", 3.0, 0.6, 1.2, 0.25, 25.0, 0.7, "~-:.+=#@*oO0Q",
     " ..,::==++##@@%%"},
};

#define SHAPE_COUNT (int)(sizeof k_shapes / sizeof k_shapes[0])

/* ── §5 blast ── */

/*
 * Cell — one character slot in the off-screen picture we build up
 * before showing anything.
 *
 * Every layer (fireball, wave, debris) draws into this grid first;
 * only once it's all done do we copy it to the terminal. Working off-
 * screen keeps the "who's on top" rules simple — later layers just
 * overwrite earlier ones, no special blending. A cell with ch == 0
 * means "nobody drew here," and those are skipped when copying, which
 * is how the black background stays black for free.
 *
 *   ch     the character to show, or 0 for an empty cell.
 *   color  which theme colour to use; the fireball colour also gets
 *          drawn bold so it's the brightest thing on screen.
 */
typedef struct {
  char ch;
  ColorID color;
} Cell;

/*
 * Blast — everything about one explosion playing right now.
 * Built once at startup, then thrown away and rebuilt whenever the
 * window resizes or the theme/shape changes (see app_reset_blast).
 *
 *   blobs   the cloud of debris chunks, given random directions once
 *           per explosion. Because the random generator never resets,
 *           each new explosion gets a fresh scatter and looks a little
 *           different from the last.
 *
 *   cells   the off-screen picture we draw into (one Cell per
 *           character on screen). Allocated per explosion and freed
 *           when the explosion ends — one of the few places this
 *           program asks for memory at all. Owned by the Blast;
 *           blast_free releases it.
 *
 *   cols    width of the terminal, remembered at setup. The explosion
 *           thinks in coordinates centred on the middle of the screen,
 *           and this is the basis for that. Cached here so the tight
 *           drawing loop never has to ask ncurses for the size.
 *
 *   rows    height of the terminal, same idea as cols. Used in the
 *           radius math to correct for terminal characters being
 *           taller than wide, so a circle actually looks round.
 *
 *   frame   how far into the explosion we are, counting up from 0.
 *           It drives the three stages of the show:
 *             frame 0      a single spark at the centre
 *             frames 1..7  a growing solid fireball
 *             frame 8 on   the spreading wave plus the 3-D debris
 *           It also sets how far the debris has flown. When it reaches
 *           the end, the explosion finishes and the next one starts.
 *
 *   theme   which colour set (index into k_themes) we're painting in.
 *           Changed by t/T or automatically when an explosion ends.
 *           Also shown in the status bar.
 *
 *   shape   which dial set (index into k_shapes) we're using — ring,
 *           star, nova, and so on. Changed by n/N or automatically.
 *           Keeping shape separate from theme means you can watch the
 *           same silhouette in every colour, or every silhouette in
 *           one colour.
 *
 *   done    set to true the moment the explosion finishes. The tick
 *           function sees it and stops, which tells the main loop to
 *           start a fresh explosion.
 */
typedef struct {
  Blob blobs[NUM_BLOBS];
  Cell *cells;
  int cols;
  int rows;
  int frame;
  int theme;
  int shape;
  bool done;
} Blast;

static void blast_alloc_cells(Blast *b) {
  b->cells = calloc((size_t)(b->cols * b->rows), sizeof(Cell));
}

static void blast_init(Blast *b, int cols, int rows, int theme, int shape) {
  b->cols = cols;
  b->rows = rows;
  b->frame = 0;
  b->theme = theme;
  b->shape = shape;
  b->done = false;
  blast_alloc_cells(b);
  blob_init_pool(b->blobs);
  color_theme_apply(theme);
}

static void blast_free(Blast *b) {
  free(b->cells);
  *b = (Blast){0};
}

/* ── Pure-math helpers ── */

/* How far a cell is from the centre. Terminal characters are about
 * twice as tall as they are wide, so we weight the vertical part
 * extra; that keeps a "circle" actually looking round on screen. */
static inline double aspect_radius(int x, int y) {
  return sqrt((double)(x * x) + 4.0 * (double)(y * y));
}

/* Gives the ring its points. Returns a number near 1 that wiggles up
 * and down as you go around the circle, so the wave bulges out in
 * some directions and pulls in elsewhere — that's what makes a star or
 * cross instead of a plain ring. The tiny +0.01 nudges just avoid an
 * undefined angle exactly at the centre. If a shape asked for no
 * points, we return a flat 1 so the ring stays perfectly round. */
static inline double petal_lobe(int x, int y, double petal_n, double ripple) {
  if (petal_n <= 0.0)
    return 1.0;
  double angle = atan2((double)y * 2.0 + 0.01, (double)x + 0.01);
  return 1.0 + ripple * cos(petal_n * angle);
}

/* The "make far things smaller" step: takes a point's position and
 * its depth and returns where it lands on the flat screen. Things
 * deeper away (bigger bz) get pulled toward the centre. Called once
 * for the across direction and once for the up/down direction. */
static inline double perspective_project(double b, double bz, double persp) {
  return b * persp / (bz + persp);
}

/* ── Per-cell painters (each handles one cell for one stage) ── */

/* Wipe a cell back to empty before we redraw it. Empty (ch == 0) means
 * nothing gets shown there, which keeps the background black for free. */
static inline void cell_clear(Cell *c) {
  c->ch = 0;
  c->color = COL_WAVE;
}

/* First frame: a single bright spark at dead centre, so the eye is
 * already looking there when the fireball and wave kick off. */
static inline void cell_paint_origin_flash(Cell *c, int x, int y) {
  if (x == 0 && y == 0) {
    c->ch = '*';
    c->color = COL_FLASH;
  }
}

/* Early frames: a solid round fireball that grows a little each frame.
 * One flat colour, no points yet — the wave stage takes over from
 * frame 8 on. */
static inline void cell_paint_disc(Cell *c, int x, int y, int frame,
                                   double disc_speed) {
  if (aspect_radius(x, y) < (double)frame * disc_speed) {
    c->ch = '@';
    c->color = COL_FLASH;
  }
}

/* Later frames: the spreading shockwave. For this cell we work out how
 * far it is from the centre (with the points mixed in and a little
 * random jitter so the edge looks ragged, not machine-perfect), then
 * compare that against how far the wave has travelled:
 *   - cell is still inside the wave  -> draw the bright core character
 *   - cell is within the wave band   -> draw a wave character (brighter
 *                                       near the front, fading behind)
 *   - the wave has already passed     -> leave it empty */
static inline void cell_paint_shockwave(Cell *c, int x, int y, int frame,
                                        const BlastShape *sh, int flash_len,
                                        int wave_len) {
  double lobe = petal_lobe(x, y, sh->petal_n, sh->ripple);
  double r = aspect_radius(x, y) * (0.5 + (prng() / 3.0) * lobe * 0.3);
  int v = frame - (int)r - 7;

  if (v < 0) {
    int fi = frame - 8;
    if (fi >= 0 && fi < flash_len) {
      c->ch = sh->flash_chars[fi];
      c->color = COL_INNER;
    }
  } else if (v < wave_len) {
    c->ch = sh->wave_chars[v];
    c->color = (v < wave_len / 2) ? COL_INNER : COL_WAVE;
  }
}

/* ── Wave-layer driver (the loop over every cell) ── */

/* Walk every cell on screen and, depending on how far into the
 * explosion we are, paint the spark, the fireball, or the wave. */
static void wave_layer_render(Blast *b) {
  const int cols = b->cols;
  const int rows = b->rows;
  const int frame = b->frame;
  const BlastShape *sh = &k_shapes[b->shape];

  const int minx = -(cols / 2);
  const int maxx = cols + minx - 1;
  const int miny = -(rows / 2);
  const int maxy = rows + miny - 1;

  const int flash_len = (int)strlen(sh->flash_chars);
  const int wave_len = (int)strlen(sh->wave_chars);

  Cell *p = b->cells;
  for (int y = miny; y <= maxy; y++) {
    for (int x = minx; x <= maxx; x++) {
      cell_clear(p);
      if (frame == 0)
        cell_paint_origin_flash(p, x, y);
      else if (frame < 8)
        cell_paint_disc(p, x, y, frame, sh->disc_speed);
      else
        cell_paint_shockwave(p, x, y, frame, sh, flash_len, wave_len);
      p++;
    }
  }
}

/* ── Per-blob painter and blob-layer driver ── */

/* Pick a character and colour for one debris chunk based on how far
 * away it is: far chunks show as a faint '.', mid ones as 'o', and
 * close ones as a bold '@'. */
static inline void blob_depth_bucket(double bz, double persp, char *out_glyph,
                                     ColorID *out_color) {
  if (bz > persp * 0.8) {
    *out_glyph = '.';
    *out_color = COL_BLOB_F;
  } else if (bz > -persp * 0.4) {
    *out_glyph = 'o';
    *out_color = COL_BLOB_M;
  } else {
    *out_glyph = '@';
    *out_color = COL_BLOB_N;
  }
}

/* Figure out where one debris chunk lands on screen and draw it, or
 * drop it if it's behind the camera or off the edge. We push the chunk
 * outward based on how much time has passed, squash it per the current
 * shape, turn its 3-D spot into a flat screen spot, then write its
 * character. That write lands on top of whatever the wave drew there,
 * which is why debris always shows in front of the wave. */
static void blob_paint_projected(const Blob *blob, Cell *cells, int cols,
                                 int rows, int frame, const BlastShape *sh) {
  const double persp = sh->persp;
  const double bspeed = sh->blob_speed;
  const int t = frame - 6;

  double bx = blob->x * t * bspeed;
  double by = blob->y * t * bspeed * sh->y_squash;
  double bz = blob->z * t * bspeed;

  if (bz < 5.0 - persp || bz > persp)
    return;

  int cx = cols / 2 + (int)perspective_project(bx, bz, persp);
  int cy = rows / 2 + (int)perspective_project(by, bz, persp);
  if (cx < 0 || cx >= cols || cy < 0 || cy >= rows)
    return;

  Cell *c = &cells[cy * cols + cx];
  blob_depth_bucket(bz, persp, &c->ch, &c->color);
}

/* Draw the whole debris cloud. Runs after the wave so the chunks sit
 * in front of it. */
static void blob_layer_render(Blast *b) {
  const BlastShape *sh = &k_shapes[b->shape];
  for (int j = 0; j < NUM_BLOBS; j++) {
    blob_paint_projected(&b->blobs[j], b->cells, b->cols, b->rows, b->frame,
                         sh);
  }
}

/* ── Driver — one frame ── */

/* Draw a whole frame: first the wave (spark, fireball, or shockwave),
 * then the debris on top once the explosion has gotten going. */
static void blast_render_frame(Blast *b) {
  wave_layer_render(b);
  if (b->frame > 6)
    blob_layer_render(b);
}

static bool blast_tick(Blast *b) {
  if (b->done)
    return false;

  blast_render_frame(b);
  b->frame++;

  if (b->frame >= NUM_FRAMES) {
    b->done = true;
    return false;
  }

  return true;
}

/* Copy one cell to the screen, but only if something was drawn there.
 * The fireball colour also gets drawn bold so it's the brightest. */
static inline void cell_blit(WINDOW *w, int x, int y, Cell c) {
  if (!c.ch)
    return;
  attr_t attr = COLOR_PAIR(c.color);
  if (c.color == COL_FLASH)
    attr |= A_BOLD;
  wattron(w, attr);
  mvwaddch(w, y, x, (chtype)(unsigned char)c.ch);
  wattroff(w, attr);
}

/* Copy the whole off-screen picture out to the terminal. */
static void blast_draw(const Blast *b, WINDOW *w) {
  const int cols = b->cols;
  const int rows = b->rows;
  for (int i = 0; i < cols * rows; i++)
    cell_blit(w, i % cols, i / cols, b->cells[i]);
}

/* ── §6 screen ── */

/*
 * Screen — just remembers the terminal's size.
 *
 * Drawing is done with ncurses, which keeps its own hidden copy of the
 * screen: we clear it, draw the blast and the status bars into it, then
 * tell ncurses to flush only what actually changed in one go. That one
 * combined write is what keeps the animation flicker-free.
 *
 *   cols, rows  the terminal's current width and height in characters.
 */
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
  typeahead(-1); /* stop ncurses pausing to check for keys mid-draw — avoids tearing */
  color_init(0);
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) {
  (void)s;
  endwin();
}

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw_blast(Screen *s, const Blast *b) {
  erase();
  blast_draw(b, stdscr);
}

/* Draw the two info bars over the blast: a status line along the top
 * (theme, shape, frame, speed) and a key-hint line along the bottom.
 * Each bar gets its whole row filled with colour first so it reads as
 * a solid strip. Drawn last so the blast can't poke through. */
static void screen_draw_hud(Screen *s, double fps, int sim_fps, int frame,
                            int theme, int shape) {
  /* ── Top row: status ── */
  char status[200];
  snprintf(status, sizeof status,
           " KABOOM   theme:%-7s   shape:%-8s   frame:%3d/%3d   "
           "%4.1f fps  %2d Hz ",
           k_themes[theme].name, k_shapes[shape].name, frame, NUM_FRAMES, fps,
           sim_fps);

  attron(COLOR_PAIR(COL_HUD) | A_BOLD);
  for (int x = 0; x < s->cols; x++)
    mvaddch(0, x, ' ');
  mvprintw(0, 0, "%s", status);
  attroff(COLOR_PAIR(COL_HUD) | A_BOLD);

  /* ── Bottom row: key hints ── */
  const char *hints = " q:quit  r:replay  t/T:theme  n/N:shape  ]/[:speed ";

  int hint_row = s->rows - 1;
  attron(COLOR_PAIR(COL_HINT) | A_BOLD);
  for (int x = 0; x < s->cols; x++)
    mvaddch(hint_row, x, ' ');
  mvprintw(hint_row, 0, "%s", hints);
  attroff(COLOR_PAIR(COL_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §7 app ── */

typedef struct {
  Blast blast;
  Screen screen;
  int sim_fps;
  int theme_idx; /* current theme; changed by t/T or when a blast ends */
  int shape_idx; /* current shape; changed by n/N or when a blast ends */
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

/* Start a fresh explosion from the beginning with the current theme
 * and shape. Used by replay, the theme/shape keys, and the automatic
 * restart when an explosion finishes. */
static void app_reset_blast(App *app) {
  blast_free(&app->blast);
  blast_init(&app->blast, app->screen.cols, app->screen.rows, app->theme_idx,
             app->shape_idx);
}

static void app_do_resize(App *app) {
  blast_free(&app->blast);
  screen_resize(&app->screen);
  blast_init(&app->blast, app->screen.cols, app->screen.rows, app->theme_idx,
             app->shape_idx);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

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

  case 'r':
  case 'R':
    app_reset_blast(app);
    break;

  case 't':
    app->theme_idx = (app->theme_idx + 1) % THEME_COUNT;
    app_reset_blast(app);
    break;
  case 'T':
    app->theme_idx = (app->theme_idx + THEME_COUNT - 1) % THEME_COUNT;
    app_reset_blast(app);
    break;

  case 'n':
    app->shape_idx = (app->shape_idx + 1) % SHAPE_COUNT;
    app_reset_blast(app);
    break;
  case 'N':
    app->shape_idx = (app->shape_idx + SHAPE_COUNT - 1) % SHAPE_COUNT;
    app_reset_blast(app);
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned int)clock_ns());

  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;
  app->theme_idx = 0;
  app->shape_idx = 0;

  screen_init(&app->screen);
  blast_init(&app->blast, app->screen.cols, app->screen.rows, app->theme_idx,
             app->shape_idx);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    /* ── handle a window resize ── */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* ── measure how long since the last loop ── */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS; /* cap it so a stall doesn't fast-forward the blast */

    /* ── step the explosion forward at the chosen speed ── */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      if (!blast_tick(&app->blast)) {
        app->theme_idx = (app->theme_idx + 1) % THEME_COUNT;
        app->shape_idx = (app->shape_idx + 1) % SHAPE_COUNT;
        app_reset_blast(app);
      }
      sim_accum -= tick_ns;
    }
    float alpha = (float)sim_accum / (float)tick_ns;
    (void)alpha;

    /* ── update the frames-per-second number shown in the bar ── */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* ── wait so we don't draw faster than 60 times a second ── */
    int64_t elapsed = clock_ns() - frame_time + dt;
    int64_t budget = NS_PER_SEC / 60;
    clock_sleep_ns(budget - elapsed);

    /* ── draw this frame ── */
    screen_draw_blast(&app->screen, &app->blast);
    screen_draw_hud(&app->screen, fps_display, app->sim_fps, app->blast.frame,
                    app->blast.theme, app->blast.shape);
    screen_present();

    /* ── react to a keypress ── */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  blast_free(&app->blast);
  screen_free(&app->screen);
  return 0;
}
