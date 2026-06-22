/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * matrix_snowflake.c — matrix rain falls; each stream's head freezes
 * where it lands, piling up white "snow" one column at a time until
 * the screen fills, flashes, and resets.
 *
 * Sister files: matrix_rain/matrix_rain.c is the same falling rain
 * without the pile (read it first); fireworks_rain.c and pulsar_rain.c
 * reuse the same glyph-shimmer trick on other shapes.
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

/* §1  config — every tunable lives here */

/* grid bounds + frame rate */
enum {
  ROWS_MAX = 100,
  COLS_MAX = 400,
  TARGET_FPS = 30,
};

/* stream length + how lively the head looks */
enum {
  RAIN_TRAIL_MIN = 4,
  RAIN_TRAIL_MAX = 14, /* also the size of each stream's glyphs[] */

  /* How many cells near the head keep swapping characters. Bigger =
   * busier, flickerier head. */
  RAIN_HEAD_FLICKER = 3,
};

/* How fast streams fall, in rows per second. Each stream picks its own
 * speed somewhere in this range, so columns fall at different rates. */
#define RAIN_SPEED_MIN 6.0f
#define RAIN_SPEED_MAX 20.0f

/* Chance per frame that a head-zone cell swaps to a new character. */
#define RAIN_HEAD_REROLL_PROB 0.67f

/* How long a stream waits after freezing before it falls again, in
 * seconds — a random pause somewhere in [MIN, MIN+VAR] so streams don't
 * all restart together. */
#define STREAM_RESTART_MIN 0.30f
#define STREAM_RESTART_VAR 1.20f

/* How far above the screen a fresh stream starts (as a fraction of the
 * screen height), so they enter staggered instead of all at once. */
#define STREAM_INIT_OFFSCREEN_FRAC 0.4f

/* User-controlled overall speed multiplier ( [ and ] keys ) */
#define RAIN_SPEED_SCALE_DEF 1.0f
#define RAIN_SPEED_SCALE_MIN 0.25f
#define RAIN_SPEED_SCALE_MAX 4.0f
#define RAIN_SPEED_SCALE_STEP 1.25f

/* The top few rows of each pile glow as "fresh snow"; everything below
 * is "packed snow". */
#define SNOW_FRESH_DEPTH 3

/* How many frames the screen-full flash lasts. 28 @ 30 fps ≈ 0.93 s. */
enum {
  FLASH_FRAMES = 28,
};

/* Biggest time step we'll accept, so one slow frame can't make
 * everything lurch. */
#define DT_CAP_SEC 0.10f

/* ncurses colour-pair slots. 1..5 get re-tinted by the theme; the two
 * HUD pairs never change. */
enum {
  CP_RAIN_HEAD = 1,
  CP_RAIN_MID,
  CP_RAIN_FADE,

  CP_SNOW_FRESH,
  CP_SNOW_PACKED,

  PAIR_HUD,
  PAIR_HINT,
};

/* timing + HUD */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define HUD_BUF_LEN 96

/* §2  clock — monotonic time + sleep */

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

/* §3  color */

/*
 * Theme — one named colour preset for the whole screen: a 3-shade
 * green-to-dim ramp for the falling rain, plus a 2-shade ramp for the
 * snow pile.
 *
 * Each theme deliberately pairs a rain colour with a CONTRASTING snow
 * colour, so the pile never blends into the rain landing on it. The
 * `t` key cycles through the presets; theme_apply re-tints the colour
 * pairs in place, so existing streams and pile cells just change hue.
 * The two HUD pairs are kept out of here on purpose — they stay a fixed
 * bright yellow + cyan no matter the theme.
 *
 * Members
 *   name      Short label shown in the HUD ("Classic", "Inferno", …).
 *   rain[3]   Rain colours, brightest head first, dimmest tail last
 *             (256-colour palette indices).
 *   rain_8[3] Same three, for terminals that only have 8 colours.
 *   snow[2]   Snow colours: fresh (bright) then packed (duller).
 *   snow_8[2] Same two, 8-colour fallback.
 *
 * Every colour is chosen from the bright half of the palette so even
 * the dimmest shade stays visible on a black background, and the rain
 * shades run bright -> dim in order (the head is drawn brightest).
 */
typedef struct {
  const char *name;
  int rain[3];   /* head, mid, fade */
  int rain_8[3];
  int snow[2];   /* fresh, packed */
  int snow_8[2];
} Theme;

static const Theme k_themes[] = {
    {"Classic", /* green rain + white snow  */
     {46, 40, 28},
     {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN},
     {231, 195},
     {COLOR_WHITE, COLOR_WHITE}},

    {"Inferno", /* red rain + gold snow     */
     {196, 124, 88},
     {COLOR_RED, COLOR_RED, COLOR_RED},
     {226, 220},
     {COLOR_YELLOW, COLOR_YELLOW}},

    {"Nebula", /* purple rain + cyan snow  */
     {201, 165, 93},
     {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA},
     {159, 87},
     {COLOR_CYAN, COLOR_CYAN}},

    {"Toxic", /* cyan rain + pink snow    */
     {51, 39, 30},
     {COLOR_CYAN, COLOR_CYAN, COLOR_CYAN},
     {219, 207},
     {COLOR_MAGENTA, COLOR_MAGENTA}},

    {"Gold", /* yellow rain + lavender   */
     {226, 220, 178},
     {COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW},
     {183, 141},
     {COLOR_MAGENTA, COLOR_MAGENTA}},
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static bool g_has_256 = false;

/* Re-tint the rain and snow colours to the chosen theme. Leaves the HUD
 * pairs alone so they stay readable in every theme. */
static void theme_apply(int idx) {
  const Theme *t = &k_themes[idx];
  const int *rain = g_has_256 ? t->rain : t->rain_8;
  const int *snow = g_has_256 ? t->snow : t->snow_8;

  init_pair(CP_RAIN_HEAD, rain[0], -1);
  init_pair(CP_RAIN_MID, rain[1], -1);
  init_pair(CP_RAIN_FADE, rain[2], -1);
  init_pair(CP_SNOW_FRESH, snow[0], -1);
  init_pair(CP_SNOW_PACKED, snow[1], -1);
}

/* Set the HUD colours once at startup. They sit on the terminal's own
 * background (-1) instead of forcing a black box. */
static void hud_pairs_init(void) {
  init_pair(PAIR_HUD, g_has_256 ? 226 : COLOR_YELLOW, -1);
  init_pair(PAIR_HINT, g_has_256 ? 51 : COLOR_CYAN, -1);
}

/* §4  stream — the falling rain */

/* the characters streams are made of, plus two tiny random helpers */

static const char k_glyphs[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
    "0123456789!#$%&*+-<>=?@^~|";

#define GLYPHS_LEN (int)(sizeof k_glyphs - 1)

static char rand_glyph(void) { return k_glyphs[rand() % GLYPHS_LEN]; }
static float urand01(void) { return (float)rand() / (float)RAND_MAX; }

/*
 * RainStream — one falling stream of characters in a single column.
 *
 * A stream is a bright head followed by a fading tail. Its life cycle:
 * it spawns above the screen, falls, and when its head reaches the top
 * of the snow pile in that column it FREEZES — the head character drops
 * into the pile and the stream goes quiet for a moment before falling
 * again. That freeze is what feeds the pile, one character at a time.
 *
 * The shimmer trick: instead of drawing a fresh random character at
 * every cell each frame (which would be a blur), each stream remembers
 * its characters in glyphs[]. Only the few cells nearest the head keep
 * changing, so just the leading edge shimmers. Same idea as
 * matrix_rain.c — read that first if this is new.
 *
 * Members
 *   col            Which column this stream lives in. Fixed at spawn.
 *   head           Current head position, as a row number. Kept
 *                  fractional so the stream can move smoothly at any
 *                  speed instead of jumping a whole row at a time.
 *   trail_len      How many cells long the tail is. Random per stream.
 *   speed          Fall speed in rows per second. Random per stream, so
 *                  columns drop at different rates.
 *   glyphs[]       The characters down the stream. glyphs[0] is the head
 *                  (drawn brightest); higher indices are the older,
 *                  dimmer tail. The first few keep re-rolling for the
 *                  shimmer; the rest stay put for the stream's life.
 *   active         true while falling. Goes false on a freeze or when
 *                  the column is full.
 *   restart_delay  Seconds left before an inactive stream falls again.
 *                  Only meaningful while !active.
 */
typedef struct {
  int col;
  float head;
  int trail_len;
  float speed;
  char glyphs[RAIN_TRAIL_MAX];
  bool active;
  float restart_delay;
} RainStream;

/* Start a fresh stream just above the top of the screen. */
static void stream_spawn(RainStream *s, int col, int rows) {
  s->col = col;
  s->head = -urand01() * STREAM_INIT_OFFSCREEN_FRAC * (float)rows;
  s->trail_len =
      RAIN_TRAIL_MIN + rand() % (RAIN_TRAIL_MAX - RAIN_TRAIL_MIN + 1);
  s->speed = RAIN_SPEED_MIN + urand01() * (RAIN_SPEED_MAX - RAIN_SPEED_MIN);
  s->active = true;
  s->restart_delay = 0.0f;
  for (int i = 0; i < s->trail_len; i++)
    s->glyphs[i] = rand_glyph();
}

/*
 * Move the stream down one frame and shimmer its head. Returns true
 * once the head reaches land_row, telling the caller it's time to freeze.
 */
static bool stream_advance(RainStream *s, float dt, float scale, int land_row) {
  s->head += s->speed * dt * scale;

  /* Roll new characters for the cells near the head — the shimmer. The
   * rest of the tail stays put so the stream still reads as one piece. */
  int n = s->trail_len < RAIN_HEAD_FLICKER ? s->trail_len : RAIN_HEAD_FLICKER;
  for (int i = 0; i < n; i++)
    if (urand01() < RAIN_HEAD_REROLL_PROB)
      s->glyphs[i] = rand_glyph();

  /* Round the fractional head to a whole row. We round halves up by hand
   * because roundf() rounds .5 to the nearest even number, which would
   * make the head twitch between two rows. */
  int head_row = (int)floorf(s->head + 0.5f);
  return head_row >= land_row;
}

/* Pick the colour for a tail cell by how far it is from the head: the
 * head is brightest, the deep tail dimmest. */
static attr_t rain_band_attr(int dist, int trail_len) {
  if (dist == 0)
    return COLOR_PAIR(CP_RAIN_HEAD) | A_BOLD;
  if (dist <= trail_len / 2)
    return COLOR_PAIR(CP_RAIN_MID);
  return COLOR_PAIR(CP_RAIN_FADE) | A_DIM;
}

/* §5  snow — the pile that builds up, one column at a time */

/*
 * Snow — the growing pile of frozen characters, tracked per column.
 *
 * Each column has its own little stack of snow that grows upward. When
 * a stream freezes, exactly one character drops onto its column's pile
 * and the pile gets one row taller. Columns fill independently — there's
 * no sideways sliding or avalanching like real sand; a falling-snow look
 * just wants characters to stack up where they land.
 *
 * The pile is stored as a plain 2-D grid of characters. Once columns
 * start filling it's mostly full anyway, so a flat array is simplest and
 * gives instant lookup when drawing.
 *
 * Members
 *   pile_chars[r][c]  The character frozen at this cell, or 0 if the cell
 *                     is still empty. Written once when it freezes.
 *   pile_top[c]       The topmost frozen row of column c. Everything from
 *                     here down to the bottom is pile; everything above
 *                     is empty. It starts just below the screen and moves
 *                     UP (its value shrinks toward 0) as snow stacks up;
 *                     0 means the column is full.
 *   cols, rows        The grid size. Snow keeps its own copy because its
 *                     helpers are handed a Snow*, not the whole Scene.
 *                     The bottom row is reserved for the HUD, so the pile
 *                     only ever fills rows 0..rows-2.
 *   frozen_count      How many characters have frozen since the last
 *                     reset — the HUD shows this as a progress count.
 */
typedef struct {
  char pile_chars[ROWS_MAX][COLS_MAX];
  int pile_top[COLS_MAX];
  int cols, rows;
  int frozen_count;
} Snow;

/* Start with every column empty. */
static void snow_init(Snow *snow, int cols, int rows) {
  if (cols > COLS_MAX)
    cols = COLS_MAX;
  if (rows > ROWS_MAX)
    rows = ROWS_MAX;
  snow->cols = cols;
  snow->rows = rows;
  snow->frozen_count = 0;

  memset(snow->pile_chars, 0, sizeof snow->pile_chars);
  for (int c = 0; c < cols; c++)
    snow->pile_top[c] = rows - 1; /* just below the last usable row */
}

/* Drop one character onto the top of a column's pile, making it one row
 * taller. Does nothing if the column is out of range or already full. */
static void snow_freeze(Snow *snow, int col, char glyph) {
  if (col < 0 || col >= snow->cols)
    return;
  int land_row = snow->pile_top[col] - 1;
  if (land_row < 0)
    return;

  snow->pile_chars[land_row][col] = glyph;
  snow->pile_top[col] = land_row;
  snow->frozen_count++;
}

/* Are all columns full? */
static bool snow_is_full(const Snow *snow) {
  for (int c = 0; c < snow->cols; c++)
    if (snow->pile_top[c] > 0)
      return false;
  return true;
}

/* Colour for one pile cell. The top few rows glow as fresh snow;
 * deeper cells look packed. While the screen-full flash is on, every
 * cell glows. */
static inline attr_t snow_cell_attr(int depth_from_top, bool flashing) {
  bool is_fresh_band = flashing || (depth_from_top < SNOW_FRESH_DEPTH);
  return is_fresh_band
      ? (COLOR_PAIR(CP_SNOW_FRESH)  | A_BOLD)
      :  COLOR_PAIR(CP_SNOW_PACKED);
}

static inline void paint_snow_glyph(int r, int c, char glyph, attr_t attrs) {
  attron(attrs);
  mvaddch(r, c, (chtype)(unsigned char)glyph);
  attroff(attrs);
}

/* Draw one column's pile, top down to the last usable row. */
static inline void draw_snow_column(const Snow *snow, int c,
                                     int last_sim_row, bool flashing) {
  int pile_top_row = snow->pile_top[c];
  for (int r = pile_top_row; r <= last_sim_row; r++) {
    char glyph = snow->pile_chars[r][c];
    if (glyph == 0) continue;
    int depth_from_top = r - pile_top_row;
    paint_snow_glyph(r, c, glyph, snow_cell_attr(depth_from_top, flashing));
  }
}

/* Draw the whole pile. This runs before the rain so the rain can paint
 * over the empty space above each pile. */
static void snow_draw(const Snow *snow, bool flashing) {
  const int LAST_SIM_ROW = snow->rows - 2;   /* keep the HUD row clear */
  for (int c = 0; c < snow->cols; c++)
    draw_snow_column(snow, c, LAST_SIM_ROW, flashing);
}

/* §6  scene — ties the rain and snow together, runs the FALL/FLASH cycle */

/*
 * SceneState — the demo's two big moods.
 *
 *   STATE_FALL   The normal show: rain falls and feeds the pile. Once
 *                every column is full, we switch to FLASH.
 *   STATE_FLASH  A brief celebratory whiteout — the whole pile glows
 *                bright for a moment so you see the screen "fill up".
 *                Then everything clears and we go back to FALL.
 *
 * FALL is 0 on purpose, so a zeroed-out scene starts in the right mood.
 */
typedef enum { STATE_FALL = 0, STATE_FLASH = 1 } SceneState;

/*
 * StreamPool — one stream per column. The array index IS the column
 * number (streams[c] is column c), so there's never any lookup. Only the
 * first `cols` slots are used; the rest sit idle.
 */
typedef struct {
  RainStream streams[COLS_MAX];
} StreamPool;

/*
 * World — the current size of the playing field in cells.
 *   cols   width  (terminal width)
 *   rows   height (terminal height; the bottom row is the HUD)
 * Snow keeps its own copy of these; this one is the source of truth.
 */
typedef struct {
  int cols;
  int rows;
} World;

/*
 * SimControls — the playback knobs the keyboard controls.
 *   paused            When true, the simulation holds still (SPACE / p).
 *   rain_speed_scale  Overall speed multiplier, nudged by [ and ] and
 *                     kept within the configured min/max.
 */
typedef struct {
  bool  paused;
  float rain_speed_scale;
} SimControls;

/*
 * Mode — which mood we're in, plus the flash countdown. Kept together so
 * the state and its timer can never drift out of sync.
 *   state       FALL or FLASH.
 *   flash_tick  Frames left in the flash. 0 while falling.
 */
typedef struct {
  SceneState state;
  int        flash_tick;
} Mode;

/*
 * Scene — everything that makes up one run, reachable from a single
 * pointer: the streams, the snow pile, the field size, the current mood,
 * the playback knobs, and which colour theme is active.
 */
typedef struct {
  StreamPool  pool;
  Snow        snow;
  World       world;
  Mode        mode;
  SimControls sim;
  int         theme_idx;
} Scene;

/* Clear the pile and restart every stream. Theme and speed setting stay
 * as they were — pressing `r` shouldn't undo your tweaks. */
static void scene_reset(Scene *s) {
  snow_init(&s->snow, s->world.cols, s->world.rows);
  for (int c = 0; c < s->world.cols; c++) {
    stream_spawn(&s->pool.streams[c], c, s->world.rows);
    /* Scatter the heads down the whole screen so the first frame is
     * already full of rain instead of empty at the top. */
    s->pool.streams[c].head = urand01() * (float)s->world.rows;
  }
  s->mode.state = STATE_FALL;
  s->mode.flash_tick = 0;
}

static void scene_init(Scene *s, int cols, int rows) {
  s->world.cols = cols;
  s->world.rows = rows;
  s->theme_idx = 0;
  s->sim.paused = false;
  s->sim.rain_speed_scale = RAIN_SPEED_SCALE_DEF;
  scene_reset(s);
}

/* A "wait basically forever" delay, used to park a stream once its
 * column is full so it never tries to fall again before the next reset. */
enum { STREAM_RESTART_NEVER_SEC = 1000000000 };

/* A resting stream: count down its wait, and let it fall again once the
 * wait is up — unless its column already filled up. */
static inline void tick_inactive_stream(Scene *s, RainStream *st,
                                         int c, float dt) {
  st->restart_delay -= dt;
  bool cooldown_done   = (st->restart_delay <= 0.0f);
  bool column_has_room = (s->snow.pile_top[c] > 0);
  if (cooldown_done && column_has_room)
    stream_spawn(st, c, s->world.rows);
}

/* The column is full: shut the stream down until the next reset. */
static inline void park_stream_until_reset(RainStream *st) {
  st->active        = false;
  st->restart_delay = (float)STREAM_RESTART_NEVER_SEC;
}

/* A random rest time before a frozen stream falls again, so they don't
 * all come back at once. */
static inline float random_restart_delay_sec(void) {
  return STREAM_RESTART_MIN + urand01() * STREAM_RESTART_VAR;
}

/* The stream just landed: drop its head into the pile, stop it, and set
 * its rest timer. */
static inline void freeze_head_into_pile(Scene *s, RainStream *st, int c) {
  snow_freeze(&s->snow, c, st->glyphs[0]);
  st->active        = false;
  st->restart_delay = random_restart_delay_sec();
}

/* A falling stream: move it down, and freeze it if it reached the pile. */
static inline void tick_active_stream(Scene *s, RainStream *st,
                                       int c, int land_row, float dt) {
  bool hit_land = stream_advance(st, dt, s->sim.rain_speed_scale, land_row);
  if (hit_land)
    freeze_head_into_pile(s, st, c);
}

/* Advance one column for this frame: it's either resting, full, or
 * actively falling. */
static void scene_tick_one_stream(Scene *s, int c, float dt) {
  RainStream *st = &s->pool.streams[c];

  if (!st->active) {
    tick_inactive_stream(s, st, c, dt);
    return;
  }

  int land_row = s->snow.pile_top[c] - 1;
  if (land_row < 0) {
    park_stream_until_reset(st);
    return;
  }

  tick_active_stream(s, st, c, land_row, dt);
}

/* Advance the whole scene one frame, including the FALL/FLASH switch. */
static void scene_tick(Scene *s, float dt) {
  if (s->sim.paused)
    return;

  if (s->mode.state == STATE_FLASH) {
    if (--s->mode.flash_tick <= 0)
      scene_reset(s);
    return;
  }

  /* falling: step every column, then flash if the screen just filled */
  for (int c = 0; c < s->world.cols; c++)
    scene_tick_one_stream(s, c, dt);

  if (snow_is_full(&s->snow)) {
    s->mode.state = STATE_FLASH;
    s->mode.flash_tick = FLASH_FRAMES;
  }
}

/* The (chtype)(unsigned char) cast keeps characters above 127 from being
 * mangled into garbage by ncurses. */
static inline void paint_rain_cell(int r, int c, char glyph, attr_t attrs) {
  attron(attrs);
  mvaddch(r, c, (chtype)(unsigned char)glyph);
  attroff(attrs);
}

/* Which row the head sits on right now (same round-halves-up as
 * stream_advance, so they always agree). */
static inline int head_row_for_stream(const RainStream *st) {
  return (int)floorf(st->head + 0.5f);
}

/* Draw one stream's visible tail, from the head upward, skipping cells
 * that are off the top, inside the pile, or on the HUD row. */
static inline void draw_rain_stream(const RainStream *st, int c,
                                     int pile_top_for_column,
                                     int hud_row) {
  int head_row = head_row_for_stream(st);
  for (int dist = 0; dist < st->trail_len; dist++) {
    int r = head_row - dist;
    bool off_screen_top   = (r < 0);
    bool inside_pile      = (r >= pile_top_for_column);
    bool on_hud_row       = (r >= hud_row);
    if (off_screen_top || inside_pile || on_hud_row) continue;
    paint_rain_cell(r, c, st->glyphs[dist],
                    rain_band_attr(dist, st->trail_len));
  }
}

/* Draw every falling stream. */
static void draw_rain_pass(const Scene *s) {
  const int HUD_ROW = s->world.rows - 1;
  for (int c = 0; c < s->world.cols; c++) {
    const RainStream *st = &s->pool.streams[c];
    if (!st->active) continue;
    draw_rain_stream(st, c, s->snow.pile_top[c], HUD_ROW);
  }
}

/* Draw the frame: pile first, rain on top of the empty space above it.
 * They never clash because the rain skips any cell inside the pile. */
static void scene_draw(const Scene *s) {
  bool flashing = (s->mode.state == STATE_FLASH);
  snow_draw    (&s->snow, flashing);
  draw_rain_pass(s);
}

/* §6 helpers driven by the keyboard */

static void scene_scale_rain_speed(Scene *s, float factor) {
  s->sim.rain_speed_scale *= factor;
  if (s->sim.rain_speed_scale < RAIN_SPEED_SCALE_MIN)
    s->sim.rain_speed_scale = RAIN_SPEED_SCALE_MIN;
  if (s->sim.rain_speed_scale > RAIN_SPEED_SCALE_MAX)
    s->sim.rain_speed_scale = RAIN_SPEED_SCALE_MAX;
}

static void scene_cycle_theme(Scene *s) {
  s->theme_idx = (s->theme_idx + 1) % THEME_COUNT;
  theme_apply(s->theme_idx);
}

/* §7  screen — ncurses setup, the HUD, and putting frames on screen */

/*
 * Screen — the terminal's current size in cells. ncurses holds the actual
 * pixels; this is just our record of how wide and tall the window is,
 * re-read whenever it changes. The bottom row (rows-1) is the HUD strip,
 * so the simulation uses rows 0..rows-2.
 */
typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1); /* don't let typing interrupt screen writes */
  start_color();
  use_default_colors(); /* lets us draw on the terminal's own background */
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

/* Print a HUD string in the given colour. Shared by both rows. */
static inline void hud_paint_text(int row, int col, int pair, const char *text) {
  attron(COLOR_PAIR(pair) | A_BOLD);
  mvprintw(row, col, "%s", text);
  attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* The short word the HUD shows for what the demo is doing right now. */
static inline const char *state_label(const Scene *s) {
  if (s->mode.state == STATE_FLASH) return "FLASH ";
  if (s->sim.paused)                return "PAUSED";
  return                                  "FALL  ";
}

/* Build the top-row status text: fps, state, speed, count, theme. */
static void format_hud_status(const Scene *s, double fps,
                              char *buf, size_t buflen) {
  snprintf(buf, buflen,
           " %5.1f fps  %s  rain:%.2fx  frozen:%d  [%s] ",
           fps, state_label(s),
           s->sim.rain_speed_scale, s->snow.frozen_count,
           k_themes[s->theme_idx].name);
}

/* Top-row status line, pushed to the right edge. */
static void draw_hud_status(const Screen *sc, const Scene *s, double fps) {
  enum { HUD_TOP_ROW = 0 };
  char buf[HUD_BUF_LEN];
  format_hud_status(s, fps, buf, sizeof buf);
  int right_col = sc->cols - (int)strlen(buf);
  if (right_col < 0) right_col = 0;
  hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, buf);
}

/* Bottom-row strip listing the keys. */
static void draw_hud_hint(const Screen *sc) {
  static const char *KEY_HINT =
      " q:quit  spc:pause  r:reset  []:rain speed  t:theme ";
  hud_paint_text(sc->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

/* The two HUD lines: status across the top, key list along the bottom. */
static void screen_draw_hud(const Screen *sc, double fps, const Scene *s) {
  draw_hud_status(sc, s, fps);
  draw_hud_hint  (sc);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* §8  app — signals, resize, and the main loop */

/*
 * FpsCounter — a steady frame-rate read-out. Measuring each frame on its
 * own would jump around, so this adds frames up over a half-second window
 * and reports one smoothed number.
 */
typedef struct {
  int     frame_count;
  int64_t window_ns;
  double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f) {
  f->frame_count = 0;
  f->window_ns   = 0;
  f->display     = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt_ns) {
  const int64_t FPS_WINDOW_NS = NS_PER_SEC / 2;       /* 500 ms */
  f->frame_count++;
  f->window_ns += dt_ns;
  if (f->window_ns < FPS_WINDOW_NS) return;
  f->display     = (double)f->frame_count
                 * (double)NS_PER_SEC / (double)f->window_ns;
  f->frame_count = 0;
  f->window_ns   = 0;
}

/*
 * App — holds everything that has to live for the whole run.
 *   scene        the simulation and how it's drawn
 *   screen       terminal size + ncurses
 *   fps          the frame-rate read-out
 *   running      cleared to stop the loop (by Ctrl-C, a signal, or `q`)
 *   need_resize  set when the window changes size; handled next frame
 * The two flags are touched by signal handlers, so they get the special
 * volatile sig_atomic_t type. g_app is the one global; everything else is
 * passed around by pointer.
 */
typedef struct {
  Scene                 scene;
  Screen                screen;
  FpsCounter            fps;
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

/* Rebuild the scene for the new window size, but keep the user's chosen
 * theme and speed (scene_init would otherwise reset them). */
static void app_do_resize(App *app) {
  int   saved_theme = app->scene.theme_idx;
  float saved_speed = app->scene.sim.rain_speed_scale;

  screen_resize(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  app->scene.theme_idx          = saved_theme;
  app->scene.sim.rain_speed_scale = saved_speed;
  app->need_resize              = 0;
}

/* Act on one keypress. Returns false to quit. */
static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {

  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;

  case ' ':
  case 'p':
  case 'P':
    s->sim.paused = !s->sim.paused;
    break;

  case 'r':
  case 'R':
    scene_reset(s);
    break;

  case ']':
    scene_scale_rain_speed(s, RAIN_SPEED_SCALE_STEP);
    break;

  case '[':
    scene_scale_rain_speed(s, 1.0f / RAIN_SPEED_SCALE_STEP);
    break;

  case 't':
  case 'T':
    scene_cycle_theme(s);
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
  fps_counter_init(&app->fps);

  screen_init(&app->screen);
  g_has_256 = (COLORS >= 256);
  theme_apply(0);
  hud_pairs_init();
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;
  int64_t last_ns = clock_ns();

  while (app->running) {

    /* (1) handle a resize first so the rest of the frame uses the new size */
    if (app->need_resize) {
      app_do_resize(app);
      last_ns = clock_ns();
    }

    /* (2) how long since the last frame, capped so one slow frame can't lurch */
    int64_t now_ns = clock_ns();
    int64_t dt_ns  = now_ns - last_ns;
    last_ns        = now_ns;
    float   dt     = (float)dt_ns / (float)NS_PER_SEC;
    if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

    /* (3) read every key waiting */
    for (int ch; (ch = getch()) != ERR;) {
      if (!app_handle_key(app, ch)) {
        app->running = 0;
        break;
      }
    }

    /* (4) advance the simulation */
    scene_tick(&app->scene, dt);

    /* (5) update the fps read-out */
    fps_counter_tick(&app->fps, dt_ns);

    /* (6) draw the frame */
    erase();
    scene_draw(&app->scene);
    screen_draw_hud(&app->screen, app->fps.display, &app->scene);
    screen_present();

    /* (7) sleep out the rest of the frame to hold a steady rate */
    int64_t elapsed = clock_ns() - now_ns;
    clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
  }

  screen_free(&app->screen);
  return 0;
}
