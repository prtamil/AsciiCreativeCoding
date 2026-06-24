/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fireworks.c — ASCII fireworks in the terminal.
 *
 * Rockets climb from the bottom, slow to a stop, then burst into a spray of
 * colored sparks that drift and fade. Keys: q/ESC quit, ]/[ speed, +/- rocket
 * count, t/T cycle color theme.
 *
 * The rocket-and-burst idea comes from Reeves' classic particle-system paper
 * "Particle Systems — A Technique for Modeling a Class of Fuzzy Objects"
 * (ACM TOG 2(2), 1983) — the Genesis fire demo from Star Trek II.
 */

#define _POSIX_C_SOURCE 200809L

/* Some compilers don't define M_PI, so make sure it exists. */
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

/* ── §1 config ── */

/* All the knobs in one place. Settings the user can change while it runs
 * (speed, rocket count, theme) live in the App struct down in §8; the fixed
 * limits and constants live here. */
enum {
  /* How fast the simulation steps, in ticks per second. */
  SIM_FPS_MIN = 10,     /* slowest the ] / [ keys will let you go */
  SIM_FPS_DEFAULT = 30, /* speed at startup                       */
  SIM_FPS_MAX = 60,     /* fastest the ] / [ keys will let you go */
  SIM_FPS_STEP = 5,     /* how much each ] or [ press changes it  */

  /* How many rockets can be in the air. */
  ROCKETS_MIN = 1,     /* fewest the + / - keys will let you go  */
  ROCKETS_DEFAULT = 6, /* count at startup                       */
  ROCKETS_MAX = 20,    /* most the + / - keys will let you go    */
  ROCKETS_STEP = 1,    /* how much each + or - press changes it  */

  /* How many sparks each rocket throws off when it bursts. */
  PARTICLES_PER_BURST = 80,

  /* How fast a rocket leaves the ground, in rows per second. */
  LAUNCH_SPEED_MIN = 3,
  LAUNCH_SPEED_MAX = 8,

  HUD_COLS = 30,       /* width of the status bar              */
  FPS_UPDATE_MS = 500, /* refresh the on-screen fps reading twice a second */

  /* Number of color themes you can cycle through with t / T. */
  N_THEMES = 10,

  /* Pool sizes, fixed at compile time so nothing is allocated at runtime. */
  MAX_ROCKETS = ROCKETS_MAX,
  MAX_PARTICLES = MAX_ROCKETS * PARTICLES_PER_BURST,
};

/* Time is measured in nanoseconds throughout. */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(fps) (NS_PER_SEC / (fps))

/* ROCKET_DRAG is how hard an upward rocket is slowed each second; a bigger
 * value means it stops (and bursts) lower down. GRAVITY is the gentler pull
 * on the sparks after they're born — kept smaller so they fan out in every
 * direction before falling. */
#define ROCKET_DRAG 9.8f
#define GRAVITY 4.0f

/* ── §2 clock ── */

/* A steadily-rising clock that never jumps backward, even if the system time
 * is adjusted — exactly what we need for measuring how long a frame took. */
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

/* ncurses identifies a foreground/background combo by a small number called a
 * "pair." We reserve pairs 1..7 for the seven spark colors of the current
 * theme — these get reassigned whenever the user switches themes. Pairs 8 and
 * 9 are the status-bar colors; they never change so the text stays readable
 * over any animation.
 *
 * Every theme color is picked from the brighter half of the 256-color range
 * (index 30 and up) so that even a dimmed, dying spark is still visible
 * against a black terminal. */
typedef enum {
  COL_RED = 1,
  COL_ORANGE = 2,
  COL_YELLOW = 3,
  COL_GREEN = 4,
  COL_CYAN = 5,
  COL_BLUE = 6,
  COL_MAGENTA = 7,
  COL_COUNT = 7,

  PAIR_HUD = 8,  /* status-bar text — bright yellow, never themed */
  PAIR_HINT = 9, /* key-hints text  — bright cyan,   never themed */
} ColorID;

/* One color theme: a name plus seven spark colors.
 *
 * The slot names (RED/ORANGE/...) are just labels for positions 1..7 — a
 * theme is free to put any color in any slot. What matters is that the seven
 * are distinct and bright, so a burst reads as a colorful spray rather than a
 * single-color ring.
 *   name : shown in the status bar
 *   fg   : seven color indices in the 256-color palette, one per slot */
typedef struct {
  const char *name;
  short fg[COL_COUNT];
} Theme;

static const Theme themes[N_THEMES] = {
    {"matrix", {28, 34, 40, 46, 82, 118, 154}},    /* greens          */
    {"neon", {201, 207, 165, 51, 87, 45, 213}},    /* magenta + cyan  */
    {"nova", {196, 202, 208, 220, 226, 231, 255}}, /* red to white-hot */
    {"ocean", {33, 39, 45, 51, 87, 123, 195}},     /* blue to white   */
    {"fire", {196, 202, 208, 214, 220, 226, 230}}, /* red to yellow   */
    {"toxic", {46, 82, 118, 154, 190, 226, 220}},  /* acid green to yellow */
    {"gold", {130, 136, 172, 178, 214, 220, 230}}, /* warm browns     */
    {"ice", {33, 39, 45, 51, 87, 123, 159}},       /* dark to pale cyan */
    {"aurora", {46, 82, 51, 87, 165, 201, 207}},   /* green/cyan/magenta */
    {"plasma", {93, 99, 165, 201, 207, 51, 87}},   /* purple to pink to cyan */
};

/* Plain colors used on basic terminals that can't do 256 colors; themes are
 * ignored there. */
static const short k_fallback_fg[COL_COUNT] = {
    COLOR_RED,  COLOR_YELLOW, COLOR_YELLOW,  COLOR_GREEN,
    COLOR_CYAN, COLOR_BLUE,   COLOR_MAGENTA,
};

/* Switch the seven spark colors over to a chosen theme; status-bar colors are
 * left alone. */
static void theme_apply(int idx) {
  if (COLORS < 256)
    return;
  const Theme *t = &themes[idx];
  for (int i = 0; i < COL_COUNT; i++)
    init_pair((short)(i + 1), t->fg[i], -1);
}

static void color_init(void) {
  start_color();
  use_default_colors();

  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
    theme_apply(0);               /* start on the "matrix" theme */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
    for (int i = 0; i < COL_COUNT; i++)
      init_pair((short)(i + 1), k_fallback_fg[i], -1);
  }
}

/* Pick one of the seven spark colors at random. */
static ColorID color_rand(void) { return (ColorID)(1 + rand() % COL_COUNT); }

/* ── §4 particle ── */

/* One spark thrown off when a rocket bursts.
 *
 * The position and velocity are floats, not whole cells, so tiny movements
 * add up correctly over many ticks before we round to a screen cell to draw.
 *   x, y     : where the spark is right now
 *   vx, vy   : how fast it's moving (columns and rows per second)
 *   life     : a fuel gauge from 1.0 (just born) down to 0.0 (gone); also
 *              controls how bright it's drawn
 *   decay    : how much life it loses each tick — sets how long it lasts
 *   symbol   : the character used to draw it; picked at birth, never changes
 *   color    : its color; picked at birth, never changes
 *   active   : false once it has died, so we can skip it */
typedef struct {
  float x, y;
  float vx, vy;
  float life;
  float decay;
  char symbol;
  ColorID color;
  bool active;
} Particle;

/* The characters a spark might be drawn as — plain ASCII so it looks the same
 * on every terminal. */
static const char k_particle_symbols[] = "*+.,`'^-~=o#@%&$!|\\/:;";
#define PARTICLE_SYM_COUNT (int)(sizeof k_particle_symbols - 1)

/* Fill in a whole burst of sparks all starting at one point.
 *
 * The sparks are spread evenly around a full circle but each gets a slightly
 * random speed and direction, so the burst looks like a round puff rather
 * than a perfect ring. Each spark also gets its own color and its own
 * lifetime, so the explosion is multicolored and fades out raggedly. */
static void particle_burst(Particle *p, int count, float x, float y) {
  for (int i = 0; i < count; i++) {
    float angle = ((float)i / count) * 2.0f * (float)M_PI +
                  ((float)rand() / RAND_MAX) * 0.3f;
    float speed = 1.5f + ((float)rand() / RAND_MAX) * 3.5f;

    p[i].x = x;
    p[i].y = y;
    p[i].vx = cosf(angle) * speed;
    p[i].vy = sinf(angle) * speed * 0.5f; /* flatten the spread vertically because
                                             terminal cells are taller than wide */
    p[i].life = 0.6f + ((float)rand() / RAND_MAX) * 0.4f;
    p[i].decay = 0.03f + ((float)rand() / RAND_MAX) * 0.04f;
    p[i].symbol = k_particle_symbols[rand() % PARTICLE_SYM_COUNT];
    p[i].color = color_rand();
    p[i].active = true;
  }
}

/* Move one spark forward by a single time step (dt_sec seconds): drift along
 * its velocity, let gravity tug it down, and burn off a little life. */
static void particle_tick(Particle *p, float dt_sec) {
  if (!p->active)
    return;

  p->x += p->vx * dt_sec * 8.0f; /* the 8.0 just sets how fast sparks travel */
  p->y += p->vy * dt_sec * 8.0f;
  p->vy += GRAVITY * dt_sec;
  p->life -= p->decay;

  if (p->life <= 0.0f)
    p->active = false;
}

/* Draw one spark. A fresher spark (more life) is drawn brighter and a dying
 * one dimmer, so the burst visibly fades. */
static void particle_draw(const Particle *p, WINDOW *w, int cols, int rows) {
  if (!p->active)
    return;

  int x = (int)p->x;
  int y = (int)p->y;
  if (x < 0 || x >= cols || y < 0 || y >= rows)
    return;

  attr_t attr = COLOR_PAIR(p->color);
  if (p->life > 0.6f)
    attr |= A_BOLD;
  else if (p->life < 0.2f)
    attr |= A_DIM;

  wattron(w, attr);
  mvwaddch(w, y, x, (chtype)(unsigned char)p->symbol);
  wattroff(w, attr);
}

/* ── §5 rocket ── */

/* A rocket lives in one of three stages, and moves through them in order
 * before looping back to the start:
 *   IDLE     — sitting on the ground, counting down before the next launch
 *   RISING   — climbing; bursts once it slows to a stop at the top of its arc
 *   EXPLODED — gone, but its sparks are still on screen and being animated */
typedef enum {
  RS_IDLE = 0,
  RS_RISING = 1,
  RS_EXPLODED = 2,
} RocketState;

/* One rocket plus the burst of sparks it owns.
 *
 * Each rocket carries its own fixed array of sparks, so nothing is ever
 * allocated at runtime — it all lives inside the Show struct.
 *   x, y      : where the rocket is
 *   vy        : how fast it's rising; negative means upward
 *   color     : the streak's color while climbing
 *   state     : which of the three stages above it's in
 *   fuse      : while IDLE, ticks left before it launches again
 *   particles : the sparks for this rocket's most recent burst */
typedef struct {
  float x, y;
  float vy;
  ColorID color;
  RocketState state;
  int fuse;
  Particle particles[PARTICLES_PER_BURST];
} Rocket;

/* Send a rocket up again: place it at the bottom in a random column, give it
 * a random upward speed and color, and clear out any leftover sparks from its
 * previous burst. */
static void rocket_launch(Rocket *r, int cols, int rows) {
  r->x = (float)(rand() % cols);
  r->y = (float)(rows - 1);
  r->vy = -(float)(LAUNCH_SPEED_MIN +
                   rand() % (LAUNCH_SPEED_MAX - LAUNCH_SPEED_MIN + 1));
  r->color = color_rand();
  r->state = RS_RISING;

  /* Wipe out any sparks still hanging around from the last burst. */
  for (int i = 0; i < PARTICLES_PER_BURST; i++)
    r->particles[i].active = false;
}

/* Move a rocket forward by one time step. What happens depends on its stage:
 * count down a waiting rocket, fly a rising one until it bursts, or animate
 * the sparks of one that has already exploded. */
static void rocket_tick(Rocket *r, float dt_sec, int cols, int rows) {
  switch (r->state) {

  case RS_IDLE:
    if (--r->fuse <= 0)
      rocket_launch(r, cols, rows);
    break;

  case RS_RISING:
    r->y += r->vy * dt_sec * 6.0f;
    r->vy += ROCKET_DRAG * dt_sec * 0.5f; /* slow it down as it climbs */

    /* Burst once it has stopped rising (vy reached zero) or run off the top. */
    if (r->vy >= 0.0f || r->y < 2.0f) {
      particle_burst(r->particles, PARTICLES_PER_BURST, r->x, r->y);
      r->state = RS_EXPLODED;
    }
    break;

  case RS_EXPLODED: {
    /* Keep the sparks moving; once they've all died, go back to waiting. */
    bool any_alive = false;
    for (int i = 0; i < PARTICLES_PER_BURST; i++) {
      particle_tick(&r->particles[i], dt_sec);
      if (r->particles[i].active)
        any_alive = true;
    }
    if (!any_alive) {
      /* Wait a random half-second to two-and-a-half seconds before relaunch. */
      int ticks_per_sec = (int)(1.0f / dt_sec);
      r->fuse = ticks_per_sec / 2 + rand() % (ticks_per_sec * 2);
      r->state = RS_IDLE;
    }
    break;
  }
  }
}

/* Draw a rocket: while it's climbing, draw its streak and a little exhaust
 * mark; after it bursts, draw all of its sparks instead. */
static void rocket_draw(const Rocket *r, WINDOW *w, int cols, int rows) {
  if (r->state == RS_RISING) {
    int x = (int)r->x;
    int y = (int)r->y;
    if (x >= 0 && x < cols && y >= 0 && y < rows) {
      wattron(w, COLOR_PAIR(r->color) | A_BOLD);
      mvwaddch(w, y, x, '|');
      wattroff(w, COLOR_PAIR(r->color) | A_BOLD);

      /* A small exhaust mark one row below the rocket. */
      if (y + 1 < rows) {
        wattron(w, COLOR_PAIR(r->color));
        mvwaddch(w, y + 1, x, '\'');
        wattroff(w, COLOR_PAIR(r->color));
      }
    }
  }

  if (r->state == RS_EXPLODED) {
    for (int i = 0; i < PARTICLES_PER_BURST; i++)
      particle_draw(&r->particles[i], w, cols, rows);
  }
}

/* ── §6 show ── */

/* The whole fireworks display: a fixed pool of rockets and how many of them
 * are currently in use.
 *
 * We always keep the full pool around but only run the first active_rockets
 * of them. The unused ones are parked with a huge fuse so they never fire on
 * their own; raising the count just wakes the next parked slot.
 *   rockets        : the pool of rocket slots
 *   active_rockets : how many are live right now (the + / - keys change this) */
typedef struct {
  Rocket rockets[MAX_ROCKETS];
  int active_rockets;
} Show;

static void show_init(Show *s, int cols, int rows, int rocket_count) {
  s->active_rockets = rocket_count;

  for (int i = 0; i < MAX_ROCKETS; i++) {
    if (i < rocket_count) {
      /* Give each live rocket a slightly later start so they don't all fire
       * on the very first frame. */
      rocket_launch(&s->rockets[i], cols, rows);
      s->rockets[i].fuse = i * 8;
      s->rockets[i].state = RS_IDLE;
    } else {
      /* Park the spares with a fuse so long they'll never go off by themselves. */
      s->rockets[i].state = RS_IDLE;
      s->rockets[i].fuse = INT32_MAX / 2;
      for (int j = 0; j < PARTICLES_PER_BURST; j++)
        s->rockets[i].particles[j].active = false;
    }
  }
}

static void show_free(Show *s) {
  /* Nothing was allocated, so there's nothing to free — just clear it out. */
  memset(s, 0, sizeof *s);
}

/* Step the whole display forward by one tick, running only the live rockets. */
static void show_tick(Show *s, float dt_sec, int cols, int rows) {
  for (int i = 0; i < s->active_rockets; i++)
    rocket_tick(&s->rockets[i], dt_sec, cols, rows);
}

/* Draw every live rocket and spark. The screen has already been cleared by
 * the caller, so we just paint on top. */
static void show_draw(const Show *s, WINDOW *w, int cols, int rows) {
  for (int i = 0; i < s->active_rockets; i++)
    rocket_draw(&s->rockets[i], w, cols, rows);
}

/* ── §7 screen ── */

/* Just the current terminal size. ncurses keeps two copies of the screen
 * behind the scenes — one we draw into and one showing what's on the terminal
 * now — and only sends the differences. That's what keeps the picture from
 * flickering.
 *   cols, rows : terminal width and height in characters */
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
  typeahead(-1); /* stop ncurses from pausing drawing to check for keys, which
                    would tear the picture on fast terminals */
  color_init();
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

/* Clear the screen, then paint all the rockets. Nothing actually shows up
 * until screen_present() is called. */
static void screen_draw_show(Screen *s, const Show *show) {
  erase();
  show_draw(show, stdscr, s->cols, s->rows);
}

/* Draw the status bar. This runs after the fireworks so the text always sits
 * on top of them: an fps/speed/count line in the top-right corner, the theme
 * name just below it, and the key hints along the bottom. */
static void screen_draw_hud(Screen *s, double fps, int sim_fps, int rockets,
                            const char *theme_name) {
  char top[64];
  snprintf(top, sizeof top, " %5.1f fps  sim:%2d Hz  rkt:%2d ", fps, sim_fps,
           rockets);
  int top_x = s->cols - (int)strlen(top);
  if (top_x < 0)
    top_x = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, top_x, "%s", top);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  char mid[64];
  snprintf(mid, sizeof mid, " theme:%s ", theme_name);
  int mid_x = s->cols - (int)strlen(mid);
  if (mid_x < 0)
    mid_x = 0;
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(1, mid_x, "%s", mid);
  attroff(COLOR_PAIR(PAIR_HUD));

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0, " q:quit  ]/[:speed  +/-:rockets  t/T:theme ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* Push everything we've drawn out to the terminal in one go, sending only the
 * cells that actually changed since last frame. */
static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §8 app ── */

/* Everything the program owns, in one place.
 *
 *   show          : the fireworks display
 *   screen        : the terminal
 *   sim_fps       : current sim speed in ticks/sec (the ] / [ keys)
 *   rockets       : how many rockets are live (the + / - keys)
 *   current_theme : which color theme is showing (index into themes[])
 *   running       : set to 0 to ask the main loop to stop
 *   need_resize   : set to 1 to ask the main loop to rebuild after a resize
 *
 * running and need_resize are written from signal handlers, so they're marked
 * volatile sig_atomic_t — the type that's safe to touch from a signal. */
typedef struct {
  Show show;
  Screen screen;
  int sim_fps;
  int rockets;
  int current_theme;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

/* A global so the signal handlers below can reach the app state. */
static App g_app;

static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}

static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}

static void cleanup(void) {
  /* Safe to call even if ncurses was never started, and even if we already
   * called it — so it's a reliable last line of defense on exit. */
  endwin();
}

/* Rebuild the screen and restart the fireworks at the terminal's new size.
 * The chosen speed and rocket count carry over. */
static void app_do_resize(App *app) {
  show_free(&app->show);
  screen_resize(&app->screen);
  show_init(&app->show, app->screen.cols, app->screen.rows, app->rockets);
  app->need_resize = 0;
}

/* Handle one keypress; return false if the key means "quit." Speed and rocket
 * count changes take effect right away without restarting anything. */
static bool app_handle_key(App *app, int ch) {
  switch (ch) {

  case 'q':
  case 'Q':
  case 27 /* ESC */:
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

  case '=':
  case '+':
    if (app->rockets < ROCKETS_MAX) {
      /* Wake up the next parked rocket and give it a short fuse so it fires soon. */
      int i = app->rockets;
      rocket_launch(&app->show.rockets[i], app->screen.cols, app->screen.rows);
      app->show.rockets[i].fuse = 5;
      app->show.rockets[i].state = RS_IDLE;
      app->rockets++;
      app->show.active_rockets = app->rockets;
    }
    break;

  case '-':
    if (app->rockets > ROCKETS_MIN) {
      app->rockets--;
      app->show.active_rockets = app->rockets;
    }
    break;

  case 't':
    app->current_theme = (app->current_theme + 1) % N_THEMES;
    theme_apply(app->current_theme);
    break;

  case 'T':
    app->current_theme = (app->current_theme + N_THEMES - 1) % N_THEMES;
    theme_apply(app->current_theme);
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
  app->rockets = ROCKETS_DEFAULT;
  app->current_theme = 0;

  screen_init(&app->screen);

  int cols = app->screen.cols;
  int rows = app->screen.rows;

  show_init(&app->show, cols, rows, app->rockets);

  /* Bookkeeping for the main loop:
   *   frame_time  : when the last frame started
   *   sim_accum   : leftover time we've collected but not yet stepped through
   *   fps_accum   : time gathered toward the next fps reading
   *   frame_count : frames drawn since the last fps reading
   *   fps_display : the fps number currently shown in the status bar */
  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    /* If the terminal was resized, rebuild for the new size before going on. */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns(); /* start timing fresh after the pause */
      sim_accum = 0;
    }

    /* Measure how long since the last frame. */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;

    /* If we were frozen for a while (debugger, swapped-out, etc.), don't try
     * to replay all the missed time at once — cap it at 100 ms. */
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    /* Add the elapsed time to the bank and step the simulation in fixed-size
     * ticks for as many whole ticks as we've saved up. Reading sim_fps here
     * every loop is what lets ] / [ change the speed instantly. */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      show_tick(&app->show, dt_sec, app->screen.cols, app->screen.rows);
      sim_accum -= tick_ns;
    }

    float alpha = (float)sim_accum / (float)tick_ns;
    (void)alpha;

    /* Once every half second, work out the real frame rate to show. */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* Sleep off the rest of our ~60-fps frame budget. Doing it here, before
     * drawing, keeps the frame rate steady no matter how long drawing takes. */
    int64_t elapsed = clock_ns() - frame_time + dt;
    int64_t budget = NS_PER_SEC / 60;
    clock_sleep_ns(budget - elapsed);

    /* Draw the fireworks, then the status bar on top, then push it all out. */
    screen_draw_show(&app->screen, &app->show);
    screen_draw_hud(&app->screen, fps_display, app->sim_fps, app->rockets,
                    themes[app->current_theme].name);
    screen_present();

    /* Read one keypress, if any, and act on it. */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  show_free(&app->show);
  screen_free(&app->screen); /* restores the terminal */

  return 0;
}
