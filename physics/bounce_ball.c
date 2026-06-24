/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bounce_ball.c — balls bouncing around the terminal with smooth, even motion.
 *
 * The trick: terminal cells are about twice as tall as they are wide, so a
 * naive ball looks faster sideways than up-down and circles come out as
 * ovals. We dodge that by running all the physics in a square "pixel" space
 * and only converting to terminal cells at the last moment, when we draw.
 * We also nudge each ball a fraction ahead of its last update so the motion
 * looks silky instead of stuttery.
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

/* ── §1 config ── */

enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 5,

  HUD_COLS = 40,
  FPS_UPDATE_MS = 500,

  BALLS_DEFAULT = 5,
  BALLS_MIN = 1,
  BALLS_MAX = 20,
  N_COLORS = 7,
};

/*
 * How many invisible "pixels" we pretend each terminal cell is made of.
 * A cell is roughly twice as tall as it is wide, so we use 8 across and 16
 * down — that ratio is what makes diagonal motion come out straight. Bigger
 * numbers just give finer sub-cell precision; the ratio is the thing that
 * matters.
 */
#define CELL_W 8
#define CELL_H 16

/*
 * Ball speed range, in pixels per second. The floor is kept fairly high on
 * purpose: a ball that creeps along jumps cell-to-cell too rarely and the
 * motion looks like a staircase. 300 keeps even the slowest ball moving
 * smoothly; 600 is a brisk fast ball.
 */
#define SPEED_MIN 300.0f
#define SPEED_MAX 600.0f

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

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

static void color_init(void) {
  start_color();
  if (COLORS >= 256) {
    init_pair(1, 196, COLOR_BLACK);
    init_pair(2, 208, COLOR_BLACK);
    init_pair(3, 226, COLOR_BLACK);
    init_pair(4, 46, COLOR_BLACK);
    init_pair(5, 51, COLOR_BLACK);
    init_pair(6, 33, COLOR_BLACK);
    init_pair(7, 201, COLOR_BLACK);
  } else {
    init_pair(1, COLOR_RED, COLOR_BLACK);
    init_pair(2, COLOR_RED, COLOR_BLACK);
    init_pair(3, COLOR_YELLOW, COLOR_BLACK);
    init_pair(4, COLOR_GREEN, COLOR_BLACK);
    init_pair(5, COLOR_CYAN, COLOR_BLACK);
    init_pair(6, COLOR_BLUE, COLOR_BLACK);
    init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
  }
}

/* ── §4 coords ── */

/*
 * This little section is the whole reason the motion looks right. Everything
 * else works in square pixel space; only here do we turn a pixel position
 * into the terminal column/row we actually draw at. pw/ph give the pixel
 * size of the whole screen for a given number of columns and rows.
 */

static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

/*
 * Turn a pixel position into the terminal cell it lands in.
 *
 * We round to the nearest cell by hand (add a half, then floor) rather than
 * using roundf(). roundf() breaks exact ties toward even numbers, so a ball
 * parked right on a cell edge would flip between two cells frame after frame
 * and flicker. Plain truncation has the opposite problem — it always rounds
 * down, which makes the ball linger unevenly and look like a staircase.
 * Adding a half and flooring always rounds the same way, so it's steady.
 */
static inline int px_to_cell_x(float px) {
  return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py) {
  return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5 ball ── */

/*
 * One bouncing ball. Everything here lives in pixel space — the square
 * coordinate world the physics uses — never in terminal cells. We only
 * convert to a cell at draw time, and nowhere else.
 *
 *   px, py   where the ball is, in pixels (float so it can sit between cells)
 *   vx, vy   how fast it's moving, in pixels per second
 *   color    which color pair to draw it with
 *   ch       which character to draw (o, *, O, @, +)
 */
typedef struct {
  float px, py;
  float vx, vy;
  int color;
  char ch;
} Ball;

static const char k_chars[] = "o*O@+";
static const int k_nchars = (int)(sizeof k_chars - 1);

/* Drop a ball at a random spot heading in a random, evenly-spread direction. */
static void ball_spawn(Ball *b, int i, int cols, int rows) {
  int pxw = pw(cols);
  int pxh = ph(rows);

  b->px = (float)(CELL_W + rand() % (pxw - 2 * CELL_W));
  b->py = (float)(CELL_H + rand() % (pxh - 2 * CELL_H));

  /*
   * Pick a random direction that's fair to every angle. Picking vx and vy
   * separately would favor the diagonals, so instead we keep throwing darts
   * at a square and only accept the ones that land inside a circle — those
   * are spread evenly around. (Doing it this way also avoids needing an
   * angle function from math.h.)
   */
  float dx, dy, len;
  do {
    dx = (float)(rand() % 2001 - 1000) / 1000.0f;
    dy = (float)(rand() % 2001 - 1000) / 1000.0f;
    len = dx * dx + dy * dy;
  } while (len < 0.01f || len > 1.0f);

  /* Shrink that direction to length 1, then scale it up to a random speed. */
  float mag = sqrtf(len);
  float speed = SPEED_MIN + (float)(rand() % (int)(SPEED_MAX - SPEED_MIN + 1));
  b->vx = (dx / mag) * speed;
  b->vy = (dy / mag) * speed;

  b->color = (i % N_COLORS) + 1;
  b->ch = k_chars[i % k_nchars];
}

/*
 * Move one ball forward by dt seconds and bounce it off the walls. It's
 * handed the edges in pixels, so it never has to know anything about the
 * terminal. Hitting a wall just flips the matching speed and nudges the ball
 * back inside.
 */
static void ball_tick(Ball *b, float dt, float max_px, float max_py) {
  b->px += b->vx * dt;
  b->py += b->vy * dt;

  if (b->px < 0.0f) {
    b->px = 0.0f;
    b->vx = -b->vx;
  }
  if (b->px > max_px) {
    b->px = max_px;
    b->vx = -b->vx;
  }
  if (b->py < 0.0f) {
    b->py = 0.0f;
    b->vy = -b->vy;
  }
  if (b->py > max_py) {
    b->py = max_py;
    b->vy = -b->vy;
  }
}

/* ── §6 scene ── */

/*
 * The whole collection of balls and the sim's running state.
 *
 *   balls   fixed pool, room for BALLS_MAX of them (no allocation at runtime)
 *   n       how many of those are actually live right now
 *   paused  true when the user has frozen the simulation
 */
typedef struct {
  Ball balls[BALLS_MAX];
  int n;
  bool paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->n = BALLS_DEFAULT;
  s->paused = false;
  for (int i = 0; i < s->n; i++)
    ball_spawn(&s->balls[i], i, cols, rows);
}

/*
 * Step every ball forward once. This works out where the walls are in pixels
 * (the one spot that turns screen size into pixel size) and then lets each
 * ball move and bounce without caring about the terminal at all.
 */
static void scene_tick(Scene *s, float dt, int cols, int rows) {
  if (s->paused)
    return;

  float max_px = (float)(pw(cols) - 1); /* right edge, in pixels  */
  float max_py = (float)(ph(rows) - 1); /* bottom edge, in pixels */

  for (int i = 0; i < s->n; i++)
    ball_tick(&s->balls[i], dt, max_px, max_py);
}

/*
 * Draw the balls, nudged a little ahead so the motion looks smooth.
 *
 * The physics only updates in fixed ticks, but a frame usually lands partway
 * between two ticks. alpha (between 0 and 1) says how far along we are, and
 * we slide each ball that fraction of a step forward before drawing it — so
 * it shows up where it really is "now" instead of lagging behind the last
 * tick. The fraction is always less than a full step, so a ball can't slip
 * past a wall it just bounced off; the clamp below mops up any rounding.
 *
 * This is the only place pixel positions get turned into terminal cells.
 */
static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows,
                       float alpha, float dt_sec) {
  float max_px = (float)(pw(cols) - 1);
  float max_py = (float)(ph(rows) - 1);

  for (int i = 0; i < s->n; i++) {
    const Ball *b = &s->balls[i];

    /* Slide the ball forward by the leftover fraction of a step. */
    float draw_px = b->px + b->vx * alpha * dt_sec;
    float draw_py = b->py + b->vy * alpha * dt_sec;

    /* Keep that nudge from poking past the edges, just in case. */
    if (draw_px < 0.0f)
      draw_px = 0.0f;
    if (draw_px > max_px)
      draw_px = max_px;
    if (draw_py < 0.0f)
      draw_py = 0.0f;
    if (draw_py > max_py)
      draw_py = max_py;

    int cx = px_to_cell_x(draw_px);
    int cy = px_to_cell_y(draw_py);

    /* And keep the cell on screen. */
    if (cx < 0)
      cx = 0;
    if (cx >= cols)
      cx = cols - 1;
    if (cy < 0)
      cy = 0;
    if (cy >= rows)
      cy = rows - 1;

    wattron(w, COLOR_PAIR(b->color) | A_BOLD);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)b->ch);
    wattroff(w, COLOR_PAIR(b->color) | A_BOLD);
  }
}

/* ── §7 screen ── */

/*
 * Current terminal size, refreshed whenever the window changes.
 *
 *   cols, rows   width and height of the terminal in cells
 *
 * We draw everything onto the one screen ncurses gives us (stdscr) and let it
 * do its own behind-the-scenes double-buffering: it remembers what's already
 * on the terminal and, when we flush, only sends the cells that changed. So
 * there's no flicker and no need to juggle our own pair of buffers — and we
 * deliberately don't, since a homemade buffer ncurses doesn't know about
 * would just leave ghost trails. We draw the balls, then the status line on
 * top, then flush once.
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
  typeahead(-1); /* don't pause drawing to peek at the keyboard — avoids tearing */
  color_init();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) {
  (void)s;
  endwin();
}

static void screen_resize(Screen *s) {
  endwin();
  refresh(); /* makes ncurses pick up the new window size */
  getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * Build one full frame: wipe the screen, draw the balls, then lay the status
 * line on top. Drawing the HUD last means it always wins over any ball
 * sitting on the same row. Nothing actually shows until screen_present runs.
 */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps,
                        float alpha, float dt_sec) {
  erase();

  scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

  /* Status line, on top of the balls. */
  char buf[HUD_COLS + 1];
  snprintf(buf, sizeof buf, "%5.1f fps  balls:%-2d  %s  spd:%d", fps, sc->n,
           sc->paused ? "PAUSED " : "running", sim_fps);
  int hud_x = s->cols - HUD_COLS;
  if (hud_x < 0)
    hud_x = 0;
  attron(COLOR_PAIR(3) | A_BOLD);
  mvprintw(0, hud_x, "%s", buf);
  attroff(COLOR_PAIR(3) | A_BOLD);
}

/* Push the finished frame to the terminal in one go. */
static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §8 app ── */

/*
 * Everything the program holds onto while running.
 *
 *   scene        the balls and pause state
 *   screen       current terminal size
 *   sim_fps      how many physics steps per second
 *   running      cleared by Ctrl-C / quit to break the main loop
 *   need_resize  set when the terminal changes size; handled next frame
 *
 * The two flags are touched from signal handlers, so they're sig_atomic_t —
 * safe to read and write from inside a signal.
 */
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

static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  int cols = app->screen.cols;
  int rows = app->screen.rows;
  for (int i = 0; i < app->scene.n; i++) {
    Ball *b = &app->scene.balls[i];
    if (b->px >= (float)pw(cols))
      b->px = (float)(pw(cols) - 1);
    if (b->py >= (float)ph(rows))
      b->py = (float)(ph(rows) - 1);
  }
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  int cols = app->screen.cols;
  int rows = app->screen.rows;

  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    s->paused = !s->paused;
    break;

  case 'r':
  case 'R':
    for (int i = 0; i < s->n; i++)
      ball_spawn(&s->balls[i], i, cols, rows);
    break;

  case '=':
  case '+':
    if (s->n < BALLS_MAX) {
      ball_spawn(&s->balls[s->n], s->n, cols, rows);
      s->n++;
    }
    break;

  case '-':
    if (s->n > BALLS_MIN)
      s->n--;
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
  srand((unsigned int)clock_ns());

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

    /* Window changed size? Re-measure and pull stray balls back inside. */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* How long since the last frame? Cap it so a hiccup can't snowball. */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    /* Run as many fixed physics steps as the elapsed time has earned. */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec, app->screen.cols, app->screen.rows);
      sim_accum -= tick_ns;
    }

    /*
     * How far we are into the next physics step that hasn't run yet, as a
     * fraction from 0 up to 1. The draw code uses it to slide each ball that
     * much further along so the motion looks smooth between steps. (While
     * paused this can make a ball drift a hair, but it's well under one cell
     * and snaps back on the next step.)
     */
    float alpha = (float)sim_accum / (float)tick_ns;

    /* Keep a running frames-per-second figure for the status line. */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* Hold steady at 60 fps. Sleep before drawing so write time can't drift. */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    /* Draw the frame and send it to the terminal. */
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps, alpha,
                dt_sec);
    screen_present();

    /* Read one keypress, if any. */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
