/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * pendulum_wave.c — the classic "pendulum wave" lecture demo.
 *
 * Fifteen pendulums hang side by side, each swinging a little faster than
 * its neighbour. They all start together, drift into rippling wave patterns,
 * then snap back into a single line every T_SYNC seconds. Nothing couples
 * them — the show is pure arithmetic in the swing speeds.
 *
 * Original apparatus: Berg & Marshall, "Pendulum waves", Am. J. Phys. 59 (2),
 * 1991, 186-187. Rendering notes live in documentation/Visual.md and
 * documentation/COLOR.md.
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ── */

#define N_PEND                                                                 \
  15 /* how many pendulums: enough for rich waves,                             \
      * few enough that they all fit on screen */

/* How many full swings the slowest pendulum makes before everything lines up
 * again. Pendulum n makes (N_BASE+n) swings in T_SYNC seconds, so a bigger
 * N_BASE packs the swing speeds closer together and the wave patterns unfold
 * more slowly and gracefully. 40 over 60 s works out to about 4.19 rad/s. */
#define N_BASE 40

/* How long (seconds) until every pendulum is back in a single line. Picked so
 * each swing speed is a whole-number multiple of the others' base rate — that
 * shared deadline is what makes them all "clap" back together at once. 60 s
 * gives plenty of wave shapes before the reset. */
#define T_SYNC 60.0f

/* Starting swing width, as a fraction of one pendulum's column. 0.70 means
 * each bob swings about 70% of the way to its column edge. Push it higher and
 * neighbouring bobs may overlap. */
#define AMP_INIT 0.70f
#define AMP_STEP 0.05f

#define RENDER_NS (1000000000LL / 60)

#define HUD_TOP 1 /* top row is the status bar */
#define HUD_BOT 1 /* bottom row is the key hints */

#define N_THEMES 10
#define N_RAMP                                                                 \
  8 /* colour stops per theme; the pendulums                                   \
     * spread out across this gradient */

/* One colour-pair slot per ramp stop, plus two fixed ones for the HUD. */
enum {
  CP_R0 = 1,
  CP_R1,
  CP_R2,
  CP_R3,
  CP_R4,
  CP_R5,
  CP_R6,
  CP_R7,
  CP_HUD,
  CP_HINT,
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

/* ── §3 color ── */

/* A Theme is one colour scheme for the whole pendulum array — purely how it
 * looks, never how it moves. Switching themes is a cosmetic change only: pause
 * the demo, press t, and every bob sits in the exact same spot, just tinted
 * differently. That clean separation is why the theme index lives on the Scene
 * struct (§4) as a render setting, not as physics.
 *
 * Each theme is eight colours that go dim-to-bright. The fifteen pendulums
 * spread out across those eight stops, so the colour you see runs in step with
 * swing speed: the slow pendulum on the left is dim, the fast one on the right
 * is bright. Eight stops is the sweet spot — fewer makes visible steps in the
 * gradient, more is just extra colours to hand-tune for no real gain.
 *
 * Every colour is kept in the bright half of the 256-colour space so the bobs
 * stay visible on a default black background (see documentation/COLOR.md). */
typedef struct {
  /* Short name shown in the status bar. Keep it to about 7 characters so the
   * whole status line still fits on an 80-column terminal. */
  const char *name;

  /* The eight colours, ordered dim to bright. ramp[0] is the dimmest (goes to
   * the slowest pendulum), ramp[N_RAMP-1] the brightest (the fastest). All
   * picked to stay visible on a black background. */
  short ramp[N_RAMP];
} Theme;

/* The ten built-in colour schemes. Matrix (index 0) is the startup default
 * because green-on-black shows up almost anywhere. The names appear in the
 * status bar, so change the colours if you like, but leave the names. */
static const Theme k_themes[N_THEMES] = {
    /*  name        ramp[0..7]                                              */
    {"Matrix", {28, 34, 40, 46, 82, 118, 154, 190}},       /* greens      */
    {"Fire", {130, 166, 202, 208, 214, 220, 226, 231}},    /* embers→white*/
    {"Oceanic", {24, 31, 38, 45, 51, 87, 123, 195}},       /* deep→cyan   */
    {"Neon", {93, 129, 165, 171, 207, 213, 219, 231}},     /* purple→pink */
    {"Mono", {240, 244, 247, 250, 252, 253, 254, 255}},    /* grayscale   */
    {"Ice", {39, 75, 111, 117, 153, 159, 195, 231}},       /* polar blues */
    {"Nova", {54, 92, 129, 135, 141, 177, 213, 231}},      /* violet/pink */
    {"Forest", {58, 64, 100, 106, 142, 148, 184, 190}},    /* leaves      */
    {"Desert", {130, 136, 172, 178, 214, 220, 222, 230}},  /* sand/gold   */
    {"Eclipse", {240, 124, 160, 196, 202, 208, 220, 231}}, /* dark→corona */
};

/* HUD colours, the same on every theme so the status bar and key hints stay
 * readable no matter what scheme the pendulums are using. */
#define CHROME_HUD_256 226 /* bright yellow */
#define CHROME_HINT_256 51 /* bright cyan   */

/* Load a colour scheme into ncurses. Safe to call any time; idx wraps on its
 * own (negatives too) so the t/T keys don't have to. The HUD colours get
 * re-set every call as a safety net in case anything stomped them. */
static void theme_apply(int idx) {
  const Theme *t = &k_themes[((idx % N_THEMES) + N_THEMES) % N_THEMES];
  if (COLORS >= 256) {
    for (int k = 0; k < N_RAMP; k++)
      init_pair(CP_R0 + k, t->ramp[k], -1);
    init_pair(CP_HUD, CHROME_HUD_256, -1);
    init_pair(CP_HINT, CHROME_HINT_256, -1);
  } else {
    /* Old 8-colour terminal: one fixed rainbow-ish ramp for every theme. */
    static const short fallback[N_RAMP] = {
        COLOR_BLUE,   COLOR_BLUE,   COLOR_CYAN, COLOR_GREEN,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,  COLOR_MAGENTA,
    };
    for (int k = 0; k < N_RAMP; k++)
      init_pair(CP_R0 + k, fallback[k], -1);
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
}

/* Turn on ncurses colour once at startup, then load the first theme. Kept
 * separate from theme_apply so switching themes later is cheap. */
static void color_init(int boot_theme) {
  start_color();
  use_default_colors();
  theme_apply(boot_theme);
}

/* Which colour pendulum i gets: spread the 15 bobs across the 8 theme stops. */
static inline int pend_cp(int i) { return CP_R0 + (i * N_RAMP) / N_PEND; }

/* ── §4 physics + scene ── */

/* Scene holds everything that changes while the demo runs. There's one global
 * copy (g_scene), so helpers just read and write it instead of passing a
 * pointer around.
 *
 * The fields fall into two groups, and the split matters: the simulation
 * fields decide where the bobs are, the rendering fields only decide what
 * colour they're painted. Touching a rendering field must never move a bob.
 * That's the whole reason switching themes mid-wave doesn't throw off the
 * timing. When you add a field, ask: does it change where the bobs go? If yes
 * it's simulation, if no it's rendering.
 *
 * A few related things deliberately live outside Scene: g_quit / g_resize are
 * set from inside a signal handler and must stay plain file-scope flags to be
 * safe there; g_rows / g_cols are screen size, owned by the main loop; the fps
 * counters are just local bookkeeping in main(). */
typedef struct {
  /* ── Simulation fields: these decide where every bob is ── */

  /* Swing speed (radians per second) for each pendulum, worked out once at
   * startup. Each one is a whole-number multiple of a shared base rate, and
   * that's the trick behind the synchronised reset: after T_SYNC seconds every
   * pendulum has completed a whole number of swings and they all line up. */
  float omega[N_PEND];

  /* Seconds elapsed since the last reset — the only thing that actually
   * changes as the demo plays. It only counts up while not paused, and each
   * step is capped at 100 ms so a frozen terminal can't make the bobs
   * teleport. Kept as a plain running total (never wrapped) so the HUD can
   * work out how long until the next reset. */
  float time;

  /* How wide the bobs swing, from 0.05 to 0.95. It just scales the swing
   * width — it never touches the swing speeds, so changing it doesn't shift
   * the reset timing. Capped below 1.0 because the simple-pendulum model this
   * uses stops being accurate at large swings. */
  float amp;

  /* When true, time stops advancing. Everything else is left untouched, so
   * unpausing picks up exactly where it left off. Shown as PAUSED/running in
   * the status bar. */
  bool paused;

  /* ── Rendering field: colour only, never moves a bob ── */

  /* Which colour scheme from k_themes[] (§3) is active. The t and T keys step
   * it forward/back and reload the colours. */
  int theme;
} Scene;

/* The one and only Scene. Starts with sensible values; the swing speeds start
 * at zero and get filled in by physics_init() before the first frame. */
static Scene g_scene = {
    .amp = AMP_INIT, .paused = false, .time = 0.0f, .theme = 0,
    /* .omega[] starts zeroed; physics_init() fills it before the loop. */
};

/* Work out each pendulum's swing speed once at startup. */
static void physics_init(void) {
  for (int i = 0; i < N_PEND; i++)
    g_scene.omega[i] = (float)(2.0 * M_PI * (N_BASE + i) / T_SYNC);
}

/* Where pendulum n is in its swing at time t, as an angle. Just a sine wave:
 * swing width times the sine of (speed times time). That closed-form answer is
 * why there's no step-by-step physics anywhere. */
static float theta(int n, float t) {
  return g_scene.amp * sinf(g_scene.omega[n] * t);
}

/* ── §5 draw ── */

static int g_rows, g_cols;

/* A Pendulum is a throwaway snapshot of where one strand sits on screen this
 * frame: where to draw its anchor, its string, and its swinging bob. We build
 * one, hand it to the three drawing helpers, and toss it — it's never kept
 * between frames. That's on purpose, and it's what makes everything painless:
 * resize the terminal, switch themes, or change the swing width, and the next
 * frame just builds fresh snapshots from the current state. Nothing to migrate.
 * Don't move any of these fields into Scene — the whole point is that they're
 * short-lived. We bundle them into a struct so the three drawing helpers can
 * each take one pointer instead of repeating the same handful of numbers.
 *
 * Only pendulum_build talks to the physics; the drawing helpers just paint. */
typedef struct {
  /* The fixed '^' anchor at the top of the strand. pivot_row is the same for
   * every pendulum (one row below the top status bar); pivot_col is the centre
   * of this pendulum's slice of the screen width. */
  int pivot_row, pivot_col;

  /* The 'O' bob's cell right now. bob_row depends on which pendulum this is —
   * the longer ones hang lower; bob_col swings left and right over time. */
  int bob_row, bob_col;

  /* This pendulum's colour (an ncurses colour-pair id), worked out once so the
   * string and the bob always match. */
  int cp;

  /* The character used for every cell of the string this frame — '|', '/', or
   * '\\' — picked from how slanted the string is, so the whole string looks
   * like one continuous line. */
  chtype string_glyph;
} Pendulum;

/* Figures out where pendulum i should be drawn this frame. Each pendulum owns
 * an equal vertical slice of the screen; its bob sits in the middle of that
 * slice and swings sideways. Longer pendulums hang lower, and since the faster
 * ones are the shorter ones, swing speed shows up as height — see Berg &
 * Marshall. The slanted-vs-upright string character is just picked from how far
 * sideways the bob currently is. This is the only place that asks the physics
 * (theta) where the bob is; the drawing helpers below just paint. */
static Pendulum pendulum_build(int i, int pivot_row, int pend_area_rows) {
  Pendulum p;

  /* Anchor: centre of this pendulum's slice of the screen width. */
  p.pivot_row = pivot_row;
  p.pivot_col = (int)((i + 0.5f) * g_cols / N_PEND);

  /* Bob left/right: start at the centre and slide over by the current swing. */
  float th = theta(i, g_scene.time);
  p.bob_col = p.pivot_col + (int)(th * ((float)g_cols / (float)N_PEND));

  /* Bob height: longer pendulums hang lower. */
  int string_len = pend_area_rows - 2;
  float L_frac = (float)(N_BASE + i) / (float)(N_BASE + N_PEND - 1);
  p.bob_row = pivot_row + 1 + (int)(string_len * L_frac);
  if (p.bob_row >= g_rows - HUD_BOT)
    p.bob_row = g_rows - HUD_BOT - 1;

  /* Colour from the theme, string character from how slanted the string is. */
  p.cp = pend_cp(i);

  int dr = p.bob_row - p.pivot_row;
  int dc = p.bob_col - p.pivot_col;
  float slope = (dr > 0) ? (float)dc / (float)dr : 0.f;
  if (fabsf(slope) < 0.35f)
    p.string_glyph = '|';
  else if (slope > 0)
    p.string_glyph = '\\';
  else
    p.string_glyph = '/';

  return p;
}

/* Draws the '^' anchor at the top of the strand. */
static void pendulum_draw_pivot(const Pendulum *p) {
  if (p->pivot_row < g_rows && p->pivot_col >= 0 && p->pivot_col < g_cols)
    mvaddch(p->pivot_row, p->pivot_col, '^');
}

/* Draws the line from anchor to bob, one character per row in between. The
 * anchor and the bob themselves are drawn by the other two helpers, so this
 * skips both ends. */
static void pendulum_draw_string(const Pendulum *p) {
  int dr = p->bob_row - p->pivot_row;
  if (dr <= 0)
    return;
  int dc = p->bob_col - p->pivot_col;
  float slope = (float)dc / (float)dr;

  attron(COLOR_PAIR(p->cp));
  for (int s = 1; s < dr; s++) {
    int r = p->pivot_row + s;
    int c = p->pivot_col + (int)(slope * s + 0.5f);
    if (r < g_rows - HUD_BOT && c >= 0 && c < g_cols)
      mvaddch(r, c, p->string_glyph);
  }
  attroff(COLOR_PAIR(p->cp));
}

/* pendulum_draw_bob — the bold 'O' at the displaced position. */
static void pendulum_draw_bob(const Pendulum *p) {
  if (p->bob_row < g_rows - HUD_BOT && p->bob_col >= 0 && p->bob_col < g_cols) {
    attron(COLOR_PAIR(p->cp) | A_BOLD);
    mvaddch(p->bob_row, p->bob_col, 'O');
    attroff(COLOR_PAIR(p->cp) | A_BOLD);
  }
}

/* Paints all the pendulums: for each one, work out where it is this frame and
 * draw its anchor, string, and bob. The math and the painting all live in the
 * helpers below — this just walks the list and calls them in order. */
static void draw_pendulums(void) {
  int pend_area_rows = g_rows - HUD_TOP - HUD_BOT;
  if (pend_area_rows < 4)
    return; /* terminal too short to draw anything legible */

  int pivot_row = HUD_TOP; /* anchor row, shared by all strands */

  for (int i = 0; i < N_PEND; i++) {
    Pendulum p = pendulum_build(i, pivot_row, pend_area_rows);
    pendulum_draw_pivot(&p);
    pendulum_draw_string(&p);
    pendulum_draw_bob(&p);
  }
}

/* The status bar across the top-right corner: theme name, swing width, the
 * clock, how long until the next sync, paused/running, and the frame rate. */
static void draw_hud_top(int fps) {
  float t_rem = T_SYNC - fmodf(g_scene.time, T_SYNC);
  char buf[180];
  snprintf(buf, sizeof buf,
           " theme:%s  amp:%.2f  t:%5.2f/%.0fs  sync_in:%5.2fs  %s  %d fps ",
           k_themes[g_scene.theme].name, g_scene.amp, g_scene.time, T_SYNC,
           t_rem, g_scene.paused ? "PAUSED " : "running", fps);
  int len = (int)strlen(buf);
  int col = g_cols - len;
  if (col < 0)
    col = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, col, buf, g_cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

static void draw_hud_bottom(void) {
  const char *hint_full =
      " q:quit  p:pause  r:reset  +/-:amp  spc:sync  t/T:theme ";
  const char *hint_short = " q:quit  p:pause  r:reset  t:theme ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= g_cols - 1)
    hint = hint_short;

  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(g_rows - 1, 0, hint, g_cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void scene_draw(int fps) {
  draw_pendulums();
  draw_hud_top(fps);
  draw_hud_bottom();
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
  color_init(g_scene.theme);
  getmaxyx(stdscr, g_rows, g_cols);
  physics_init();

  long long last = clock_ns();
  long long fps_acc = 0;
  int fps_cnt = 0;
  int fps_disp = 0;

  while (!g_quit) {

    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, g_rows, g_cols);
      last = clock_ns();
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
      g_scene.paused = !g_scene.paused;
      break;
    case 'r':
    case 'R':
      g_scene.time = 0.f;
      break;
    case '+':
    case '=':
      g_scene.amp += AMP_STEP;
      if (g_scene.amp > 0.95f)
        g_scene.amp = 0.95f;
      break;
    case '-':
      g_scene.amp -= AMP_STEP;
      if (g_scene.amp < 0.05f)
        g_scene.amp = 0.05f;
      break;
    case ' ':
      /* jump to next sync point */
      g_scene.time = ceilf(g_scene.time / T_SYNC) * T_SYNC;
      break;
    case 't':
      g_scene.theme = (g_scene.theme + 1) % N_THEMES;
      theme_apply(g_scene.theme);
      break;
    case 'T':
      g_scene.theme = (g_scene.theme + N_THEMES - 1) % N_THEMES;
      theme_apply(g_scene.theme);
      break;
    default:
      break;
    }

    long long now = clock_ns();
    long long dt_ns = now - last;
    last = now;
    if (dt_ns > 100000000LL)
      dt_ns = 100000000LL;

    if (!g_scene.paused)
      g_scene.time += (float)dt_ns * 1e-9f;

    /* fps counter — rolling 1-second window */
    fps_acc += dt_ns;
    fps_cnt += 1;
    if (fps_acc >= 1000000000LL) {
      fps_disp = (int)(fps_cnt * 1000000000LL / fps_acc);
      fps_acc = 0;
      fps_cnt = 0;
    }

    erase();
    scene_draw(fps_disp);
    wnoutrefresh(stdscr);
    doupdate();
    clock_sleep_ns(RENDER_NS - (clock_ns() - now));
  }
  return 0;
}
