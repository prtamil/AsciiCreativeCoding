/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ising.c — a screenful of tiny magnets that line up when you cool them.
 *
 * Every character is one tiny magnet that points either up or down.  We
 * keep poking random magnets and let them flip; when it's hot they stay
 * a random mess, and as you cool things down they start agreeing with
 * their neighbours and big single-colour blobs grow.  That sudden switch
 * from mess to order is a real physics result, exact for this 2D grid
 * (Onsager 1944).
 *
 * The poke-and-flip rule is the classic Metropolis method (Metropolis et
 * al. 1953).  Background reading: Newman & Barkema, Monte Carlo Methods
 * in Statistical Physics (1999), ch. 3.
 *
 * §1 config  §2 clock  §3 scene  §4 color  §5 grid  §6 draw  §7 app
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ── */

#define GRID_W_MAX 200
#define GRID_H_MAX 60
#define HUD_ROWS 2
#define N_THEMES 10
#define N_PATTERNS 10
#define STRIPE_W 6 /* how wide each stripe is, in cells */

/* Temperature is a plain number here, not degrees — physicists scale the
 * problem so the interesting point lands near 2.27.  We start hot (above
 * that point) so the grid begins as a random mess and you can cool it
 * down with the ↓ key to watch order appear. */
#define T_INIT 3.0f

/* The exact tipping point for this 2D grid: above it the magnets stay
 * random, below it they line up into big blobs.  This value is Onsager's
 * exact 1944 answer, 2 / ln(1 + √2). */
#define T_CRIT 2.269f

#define T_MIN 0.1f /* keep T off zero — dividing by it would blow up */
#define T_MAX 5.0f
#define T_STEP 0.05f /* how much one ↑/↓ press changes the temperature */

/* How busy the grid is each frame, before the /1000 trim in
 * metropolis_attempts_per_frame().  Turn it up to settle faster (at the
 * cost of more work per frame). */
#define FLIPS_PER_CELL 50
#define RENDER_NS (1000000000LL / 30)

enum {
  CP_UP = 1,   /* colour for up magnets   — theme's up colour          */
  CP_DN = 2,   /* colour for down magnets — theme's down colour        */
  CP_HUD = 3,  /* bright yellow for the status bar                     */
  CP_HINT = 4, /* bright cyan for the key hints                        */
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

/* ── §3 scene ── */

/* How many recent magnetisation samples the little HUD graph keeps.
 * 24 frames at 30 fps is about 0.8 seconds of history — enough to watch
 * the line climb as you cool, and still short enough to fit on an
 * 80-column screen.  Defined here because the Scene struct sizes an
 * array with it. */
#define SPARK_LEN 24

/*
 * Scene — the whole world in one box: everything that changes while the
 * demo runs, both the physics and the on-screen bits.  There's exactly
 * one of these (g_scene), so any function touching the world reaches it
 * as g_scene.<field>, and you can tell at a glance which writes change
 * the world versus a plain local.
 *
 * The fields are split into two halves with a divider: the simulation
 * half first, the drawing half second.  The rule is one-way — the
 * physics must never peek at a drawing field, but the renderer may read
 * physics fields freely (that's its whole job).
 *
 * Two control flags live OUTSIDE this struct on purpose: g_quit and
 * g_resize.  They're set from signal handlers, so they need the special
 * sig_atomic_t type, and they're really program-control switches rather
 * than part of the world.  The fixed lookup tables (themes, patterns)
 * are left out too — they never change.
 */
typedef struct {

  /* ── simulation half ──
   * Everything the flip-the-magnets step needs to advance the grid by
   * one frame.  Pure physics, no terminal. */

  signed char spin[GRID_H_MAX][GRID_W_MAX];
  /* The grid of magnets — each cell is +1 (up) or −1 (down).  Stored as
   * signed char (one byte) instead of int so the whole grid is tiny
   * (200×60 = 12 KB) and stays in fast CPU cache; the inner loop reads
   * a cell and its four neighbours millions of times a second, so
   * keeping it small is what makes that fast.  The grid wraps around at
   * the edges (top joins bottom, left joins right), and that wrap is
   * done when reading, not stored. */

  int gh, gw;
  /* The grid's live height and width in cells.  Recomputed whenever the
   * terminal resizes (screen size minus the two HUD rows), capped at
   * the array's maximum.  The spin array is always full-size, so a
   * resize never reallocates — these two just say how far the loops
   * actually run. */

  float temp;
  /* Temperature — a plain number, not degrees.  Physicists scale things
   * so the interesting tipping point lands near 2.269 (Onsager).  This
   * is the one input the acceptance table depends on. */

  long long sweeps;
  /* Counts how many simulation steps have run since this pattern
   * started — shown in the HUD as a rough "how long has it been
   * settling" number.  Made wide (long long) so a days-long session
   * can't overflow it. */

  float boltz[3];
  /* The "should I accept an uphill flip?" odds, worked out once each
   * time the temperature changes so the hot loop never has to call the
   * slow exp() function.
   *
   * Flipping a magnet can only change the energy by one of five amounts
   * (−8, −4, 0, +4, +8).  The downhill and flat ones are always
   * accepted, so only the two uphill cases need a stored probability:
   *   boltz[0] is the odds for a +4 change,
   *   boltz[1] is the odds for a +8 change.
   * Looking these up instead of recomputing exp() every time is the
   * single biggest speed win in the file (Newman & Barkema, ch. 3).
   * Slot [2] is leftover from an older scheme and unused. */

  int pattern;
  /* Which starting shape is active (an index into the PATTERNS table).
   * This is physics state, not just looks: changing it (n/p keys) wipes
   * the grid and redraws the new shape, unlike a theme change which
   * only repaints. */

  /* ── drawing half ──
   * Everything the painter needs to draw a frame.  Theme cycling,
   * pause, and resize all touch this half and none of them disturb the
   * physics. */

  int rows, cols;
  /* The terminal's current size in characters.  Don't confuse with
   * gh/gw above: those are the GRID size, these are the SCREEN size.
   * The top and bottom rows are the HUD; the grid fills everything in
   * between, and shares the full width. */

  bool paused;
  /* The pause switch.  When on, the screen keeps repainting (so you see
   * a frozen picture) but the main loop skips the physics step — handy
   * for studying a pattern without it shifting under your eyes.  The
   * HUD graph still gets fed the same frozen value, so its tail goes
   * flat. */

  float mhist[SPARK_LEN];
  int mhead;
  /* The recent history feeding the little HUD graph — a ring buffer of
   * the last few magnetisation readings, one added per frame.  It lives
   * on the drawing side because the physics never looks at it.  mhead
   * marks where the next reading goes; it wraps around the buffer.
   * Watching this line climb from floor to ceiling as you cool down IS
   * the order-appears moment, drawn live. */

  int theme;
  /* Which colour theme is active (an index into the themes table),
   * cycled with t/T.  Only changes colours and glyphs — the physics
   * never sees it. */
} Scene;

/* The one and only world.  We only set the starting temperature here
 * (hot, above the tipping point, so the grid begins as a mess you can
 * cool); everything else starts at zero and gets filled in just before
 * the main loop — the grid by apply_current_pattern(), the acceptance
 * table by boltz_update(), the sizes from the terminal.  Pattern 0 is
 * Random and theme 0 is Matrix (green-on-black). */
static Scene g_scene = {
    .temp = T_INIT,
};

/* ── §4 color ── */

/*
 * Theme — one named look for the grid: the colours and characters used
 * for up magnets and down magnets, bundled together.
 *
 * They're bundled so a look always stays coherent — picking the colours
 * and the glyphs as a set means you can't accidentally mix, say, a
 * starfield colour with a ripple character.  One t/T press swaps the
 * whole look at once.
 *
 * Each theme carries two colour choices for each state: a rich 256-colour
 * one and a plain fallback for old terminals that only have 8 colours.
 * The painter picks whichever the terminal supports.  Every colour is
 * deliberately on the bright side so dimmed cells stay visible.
 */
typedef struct {
  const char *name;     /* name shown in the HUD: "Matrix", "Ocean", … */
  short up_fg, dn_fg;   /* rich colours (0..255) for up and down       */
  short up_fg8, dn_fg8; /* fallback colours for 8-colour terminals     */
  chtype up_ch, dn_ch;  /* the character drawn for up and for down     */
} Theme;

/* The ten looks, just names and colours — none of this touches the
 * physics.  Matrix is green-on-black, Fire is flame over embers, Ocean
 * is cyan ripples, and so on. */
static const Theme k_themes[N_THEMES] = {
    /*  name      up256  dn256  up8            dn8           up_ch  dn_ch */
    {"Matrix", 46, 28, COLOR_GREEN, COLOR_GREEN, '#', '.'},
    {"Nova", 231, 57, COLOR_WHITE, COLOR_BLUE, '*', ' '},
    {"Mono", 231, 244, COLOR_WHITE, COLOR_BLACK, '#', '.'},
    {"Fire", 214, 88, COLOR_YELLOW, COLOR_RED, '^', '.'},
    {"Ocean", 51, 25, COLOR_CYAN, COLOR_BLUE, '~', '-'},
    {"Void", 201, 24, COLOR_MAGENTA, COLOR_BLACK, '@', ' '},
    {"Amber", 226, 94, COLOR_YELLOW, COLOR_RED, '#', '.'},
    {"Neon", 199, 54, COLOR_MAGENTA, COLOR_BLUE, '+', '-'},
    {"Ice", 159, 25, COLOR_CYAN, COLOR_BLUE, '*', '.'},
    {"Plasma", 196, 33, COLOR_RED, COLOR_BLUE, '#', '.'},
};

static void theme_apply(int ti) {
  const Theme *t = &k_themes[ti];
  if (COLORS >= 256) {
    init_pair(CP_UP, t->up_fg, -1);
    init_pair(CP_DN, t->dn_fg, -1);
  } else {
    init_pair(CP_UP, t->up_fg8, -1);
    init_pair(CP_DN, t->dn_fg8, -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(CP_HUD, 226, -1); /* bright yellow */
    init_pair(CP_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
  theme_apply(g_scene.theme);
}

/* ── §5 grid ── */

/* This section moves the physics forward: refreshing the acceptance
 * odds when the temperature changes, filling the grid with a starting
 * shape, and doing the actual poke-a-magnet-and-maybe-flip step.  These
 * functions only write the simulation half of the world. */

/* Work out the odds of taking an energy-costing flip at the current
 * temperature.  Call this whenever the temperature changes. */
static void boltz_update(void) {
  g_scene.boltz[0] = expf(-4.f / g_scene.temp);
  g_scene.boltz[1] = expf(-8.f / g_scene.temp);
}

/* ── starting shapes ──
 *
 * Each pat_* function fills the grid with one starting layout; the
 * physics then takes over and settles it.  The fun part is that
 * different starts settle in very different ways — a round blob shrinks
 * smoothly, nested rings vanish from the inside out, a field of small
 * and large blobs has the small ones disappear first.  The list runs
 * roughly from plain to interesting. */

static void pat_random(void) {
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      g_scene.spin[r][c] = (rand() & 1) ? 1 : -1;
}

static void pat_all_up(void) {
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      g_scene.spin[r][c] = 1;
}

static void pat_all_down(void) {
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      g_scene.spin[r][c] = -1;
}

static void pat_checker(void) {
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      g_scene.spin[r][c] = ((r + c) & 1) ? 1 : -1;
}

static void pat_stripes_h(void) {
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      g_scene.spin[r][c] = ((r / STRIPE_W) & 1) ? 1 : -1;
}

static void pat_stripes_v(void) {
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      g_scene.spin[r][c] = ((c / STRIPE_W) & 1) ? 1 : -1;
}

static void pat_halves(void) {
  int mid = g_scene.gw / 2;
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      g_scene.spin[r][c] = (c < mid) ? 1 : -1;
}

static void pat_bubble(void) {
  /* One round blob of up-magnets in a sea of down-magnets.  Once it's
   * running the blob steadily shrinks, like a drop pulling itself in. */
  int cr = g_scene.gh / 2, cc = g_scene.gw / 2;
  int rad = (g_scene.gh < g_scene.gw ? g_scene.gh : g_scene.gw) / 3;
  int rad2 = rad * rad;
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++) {
      int dr = r - cr, dc = c - cc;
      g_scene.spin[r][c] = (dr * dr + dc * dc < rad2) ? 1 : -1;
    }
}

static void pat_rings(void) {
  /* Nested rings, alternating up and down like a bullseye.  The tighter
   * inner rings give way first, so you can watch the rings disappear one
   * at a time from the centre out. */
  int cr = g_scene.gh / 2, cc = g_scene.gw / 2;
  int small_dim = g_scene.gh < g_scene.gw ? g_scene.gh : g_scene.gw;
  int ring_w = small_dim / 10;
  if (ring_w < 2)
    ring_w = 2;
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++) {
      int dr = r - cr, dc = c - cc;
      int dist = (int)sqrtf((float)(dr * dr + dc * dc));
      g_scene.spin[r][c] = ((dist / ring_w) & 1) ? 1 : -1;
    }
}

static void pat_cluster(void) {
  /* A down-magnet background scattered with about a dozen up-magnet
   * blobs of random size.  Watch the cascade: the small blobs vanish in
   * seconds while the big ones hang around far longer. */
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      g_scene.spin[r][c] = -1;

  int n_bubbles = 12;
  int max_rad = (g_scene.gh < g_scene.gw ? g_scene.gh : g_scene.gw) / 5;
  if (max_rad < 3)
    max_rad = 3;
  for (int b = 0; b < n_bubbles; b++) {
    int cr = rand() % g_scene.gh;
    int cc = rand() % g_scene.gw;
    int rad = 2 + rand() % max_rad;
    int rad2 = rad * rad;
    for (int r = 0; r < g_scene.gh; r++)
      for (int c = 0; c < g_scene.gw; c++) {
        int dr = r - cr, dc = c - cc;
        if (dr * dr + dc * dc < rad2)
          g_scene.spin[r][c] = 1;
      }
  }
}

/*
 * Pattern — one entry in the list of starting shapes you cycle with n/p:
 * a human-readable name for the HUD plus the function that paints that
 * shape onto the grid.
 *
 * Pairing the name with the function keeps the whole list in one table
 * (PATTERNS[] below) — adding a new starting shape is just one more row,
 * no switch statements or key-handler changes.
 *
 * Why bother with several starts?  The poke-and-flip rule is always the
 * same, but where you start from changes what you get to watch as it
 * settles: a round blob shrinks smoothly, nested rings vanish from the
 * inside out, a field of blobs loses the small ones first.  Same physics,
 * very different shows.
 *
 * The init function takes no arguments on purpose — it reads the grid's
 * current size when it runs, so a resize just re-paints the shape at the
 * new size with nothing extra to wire up.
 *
 *   name  — what shows in the HUD: "Random", "Bubble", …
 *   init  — fills the grid from scratch with this shape; reads the grid
 *           size, changes nothing else.
 */
typedef struct {
  const char *name;
  void (*init)(void);
} Pattern;

static const Pattern PATTERNS[N_PATTERNS] = {
    /* roughly plain to interesting */
    {"Random", pat_random},       /* a random mess to start from         */
    {"All Up", pat_all_up},       /* everything pointing one way         */
    {"All Down", pat_all_down},   /* everything pointing the other way   */
    {"Checker", pat_checker},     /* a chequerboard, every cell at odds  */
    {"Stripes-H", pat_stripes_h}, /* horizontal stripes                  */
    {"Stripes-V", pat_stripes_v}, /* vertical stripes                    */
    {"Halves", pat_halves},       /* one flat wall down the middle       */
    {"Bubble", pat_bubble},       /* a round blob that shrinks away      */
    {"Rings", pat_rings},         /* a bullseye that peels inward        */
    {"Cluster", pat_cluster},     /* scattered blobs, small ones go first*/
};

/* Re-apply the active pattern (and reset the sweep counter).  Called
 * on startup, on r-reset, on n / p pattern switch, and on resize
 * (the grid dimensions just changed). */
static void apply_current_pattern(void) {
  PATTERNS[g_scene.pattern].init();
  g_scene.sweeps = 0;
}

/* ── the poke-a-magnet step ──
 *
 * Every attempt is the same little recipe (the Metropolis method):
 *   1. pick a random magnet
 *   2. look at which way it points
 *   3. add up which way its four neighbours point
 *   4. work out whether flipping it would cost or save energy
 *   5. decide whether to flip — always if it saves energy, sometimes if
 *      it costs (a weighted coin toss, hotter = more likely)
 *   6. flip it if we decided to
 *
 * Each step is its own little function below so grid_step() reads like
 * the recipe instead of a mess of array math. */

/* Pick a random magnet.  We poke one cell at a time at random rather
 * than marching through them in order; both are valid, random is the
 * simplest to reason about. */
static void lattice_pick_random_site(int *r, int *c) {
  *r = rand() % g_scene.gh;
  *c = rand() % g_scene.gw;
}

/* Add up which way the four neighbours point.  The grid wraps around at
 * the edges (top meets bottom, left meets right), so there are no special
 * edge cases and a small grid behaves like a patch cut from a big one.
 * The answer is one of −4, −2, 0, +2, +4. */
static int lattice_sum_neighbors_toroidal(int r, int c) {
  return g_scene.spin[r > 0 ? r - 1 : g_scene.gh - 1][c] +
         g_scene.spin[r < g_scene.gh - 1 ? r + 1 : 0][c] +
         g_scene.spin[r][c > 0 ? c - 1 : g_scene.gw - 1] +
         g_scene.spin[r][c < g_scene.gw - 1 ? c + 1 : 0];
}

/* Would flipping this magnet cost energy or save it?  A magnet is happy
 * when it agrees with its neighbours, so flipping one that mostly agrees
 * costs energy and flipping one that mostly disagrees saves it.  The
 * answer is one of −8, −4, 0, +4, +8; negative means it saves energy. */
static int metropolis_delta_energy(int s, int sum_nbr) {
  return 2 * s * sum_nbr;
}

/* Decide whether to actually flip.  If the flip saves energy (or breaks
 * even) we always take it.  If it costs energy we take it only sometimes —
 * a weighted coin toss, more likely when it's hot.  The odds were worked
 * out once in boltz_update(), so this is just a table lookup, no slow
 * exp() in the hot loop. */
static bool metropolis_accept_flip(int dE) {
  if (dE <= 0)
    return true;
  float prob = (dE == 4) ? g_scene.boltz[0] : g_scene.boltz[1];
  return ((float)rand() / (float)RAND_MAX) < prob;
}

/* Flip the magnet.  The cast is just to keep the compiler quiet about
 * squeezing the result back into a one-byte cell. */
static void lattice_flip_spin(int r, int c) {
  g_scene.spin[r][c] = (signed char)(-g_scene.spin[r][c]);
}

/* Run the poke-a-magnet step n_attempts times, then bump the counter
 * that the HUD shows.  The body is the six-step recipe from above. */
static void grid_step(int n_attempts) {
  for (int f = 0; f < n_attempts; f++) {
    int r, c;
    lattice_pick_random_site(&r, &c);                   /* (1) */
    int s = g_scene.spin[r][c];                         /* (2) */
    int sum_nbr = lattice_sum_neighbors_toroidal(r, c); /* (3) */
    int dE = metropolis_delta_energy(s, sum_nbr);       /* (4) */
    if (metropolis_accept_flip(dE))                     /* (5) */
      lattice_flip_spin(r, c);                          /* (6) */
  }
  g_scene.sweeps++;
}

/* ── §6 draw ── */

/* This section only reads the world and paints it; it never changes the
 * physics.  All the drawing-side state (screen size, pause flag, the HUD
 * graph history, the active theme) lives in the Scene struct back in §3 —
 * here we just have the functions that use it. */

static float magnetisation(void) {
  long long sum = 0;
  for (int r = 0; r < g_scene.gh; r++)
    for (int c = 0; c < g_scene.gw; c++)
      sum += g_scene.spin[r][c];
  return fabsf((float)sum / (float)(g_scene.gh * g_scene.gw));
}

static void mhist_push(float m) {
  g_scene.mhist[g_scene.mhead] = m;
  g_scene.mhead = (g_scene.mhead + 1) % SPARK_LEN;
}

/* Five bar heights for the little graph, shortest to tallest. */
static const char k_spark_glyph[5] = {'_', '.', '-', '*', '#'};

/* Draw the recent history as a row of little bars, left to right.
 * Coloured like the up-magnets since it tracks how lined-up they are.
 * Stops early so it never spills past the edge on a narrow terminal. */
static void hud_sparkline(int row, int *col, int max_col) {
  int avail = max_col - *col;
  if (avail <= 0)
    return;
  int n = SPARK_LEN < avail ? SPARK_LEN : avail;
  attron(COLOR_PAIR(CP_UP) | A_BOLD);
  for (int i = 0; i < n; i++) {
    int idx = (g_scene.mhead + i) % SPARK_LEN;
    int lvl = (int)(g_scene.mhist[idx] * 4.0f + 0.5f);
    if (lvl < 0)
      lvl = 0;
    if (lvl > 4)
      lvl = 4;
    mvaddch(row, *col + i, (chtype)(unsigned char)k_spark_glyph[lvl]);
  }
  attroff(COLOR_PAIR(CP_UP) | A_BOLD);
  *col += n;
}

/* How many of the four neighbours point the same way as this magnet,
 * 0 to 4.  A 4 means we're deep inside a solid-colour blob; a 2 means
 * we're sitting right on the edge between two blobs.  cell_attr() uses
 * this to dim the boring insides and brighten the edges. */
static int neighbor_agreement(int r, int c, int s) {
  int u = g_scene.spin[r > 0 ? r - 1 : g_scene.gh - 1][c];
  int d = g_scene.spin[r < g_scene.gh - 1 ? r + 1 : 0][c];
  int l = g_scene.spin[r][c > 0 ? c - 1 : g_scene.gw - 1];
  int rt = g_scene.spin[r][c < g_scene.gw - 1 ? c + 1 : 0];
  return (u == s) + (d == s) + (l == s) + (rt == s);
}

/* Pick how bright to draw one cell so the blob edges stand out.  Cells
 * deep inside a blob are dimmed (they're calm and boring); cells on an
 * edge are made bold (that's where the interesting action is).  Cool the
 * grid and you get big dim blobs with bright outlines; heat it up and
 * almost everything is an edge, so the whole screen looks like bright
 * noise. */
static attr_t cell_attr(int r, int c, int s) {
  int agree = neighbor_agreement(r, c, s);
  if (agree >= 4)
    return A_DIM;
  if (agree == 3)
    return A_NORMAL;
  return A_BOLD;
}

/* Print one piece of the status bar in its own colour, then move the
 * cursor along so the next piece lands right after it.  This lets the
 * status bar be built up chunk by chunk, each chunk coloured to match
 * what it's describing.  Stops early on a narrow terminal so the bar
 * never wraps onto the next line. */
static void hud_seg(int row, int *col, int max_col, int pair, attr_t attr,
                    const char *fmt, ...) {
  if (*col >= max_col)
    return;
  char buf[96];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (n < 0)
    n = 0;
  attron(COLOR_PAIR(pair) | attr);
  mvprintw(row, *col, "%.*s", max_col - *col, buf);
  attroff(COLOR_PAIR(pair) | attr);
  *col += n;
}

/* Draw the whole grid.  Each cell's colour and character come from the
 * current theme (one look for up-magnets, one for down), and how bright
 * it is comes from cell_attr() so the blob edges pop out.  The grid fills
 * the screen between the top and bottom rows, which are the HUD. */
static void lattice_paint(void) {
  const Theme *th = &k_themes[g_scene.theme];
  for (int r = 0; r < g_scene.gh && r + 1 < g_scene.rows - 1; r++) {
    for (int c = 0; c < g_scene.gw && c < g_scene.cols; c++) {
      int s = g_scene.spin[r][c];
      int cp;
      chtype ch;
      if (s > 0) {
        cp = CP_UP;
        ch = th->up_ch;
      } else {
        cp = CP_DN;
        ch = th->dn_ch;
      }
      attr_t at = cell_attr(r, c, s);
      attron(COLOR_PAIR(cp) | at);
      mvaddch(r + 1, c, ch);
      attroff(COLOR_PAIR(cp) | at);
    }
  }
}

/* ── the top status bar, built one label at a time ──
 *
 * Each helper below paints one chunk of the top status bar and slides the
 * cursor along.  The colours aren't random: a number is drawn in the same
 * colour as the thing it measures, so "ordered" and the magnetisation
 * reading share the up-magnet colour, and "disordered" takes the
 * down-magnet colour. */

static void hud_label_temperature_and_phase(int *col) {
  hud_seg(0, col, g_scene.cols, CP_HUD, A_BOLD, " Ising ");
  hud_seg(0, col, g_scene.cols, CP_HUD, A_NORMAL, " T=");
  hud_seg(0, col, g_scene.cols, CP_HUD, A_BOLD, "%.3f", g_scene.temp);
  hud_seg(0, col, g_scene.cols, CP_HUD, A_DIM, " (Tc=%.3f", T_CRIT);

  /* Tag the current temperature as ordered, disordered, or right at the
   * tipping point, judged by how close it is to that point.  The tag's
   * colour matches what you should be seeing on screen. */
  float tc_dist = fabsf(g_scene.temp - T_CRIT);
  if (tc_dist < 0.1f)
    hud_seg(0, col, g_scene.cols, CP_HUD, A_BOLD, " ≈Tc");
  else if (g_scene.temp < T_CRIT)
    hud_seg(0, col, g_scene.cols, CP_UP, A_BOLD, " ordered");
  else
    hud_seg(0, col, g_scene.cols, CP_DN, A_BOLD, " disordered");

  hud_seg(0, col, g_scene.cols, CP_HUD, A_DIM, ")");
}

/* Show how lined-up the magnets are right now (0 = random mess, 1 = all
 * agree), plus the little history graph beside it.  Watching this number
 * jump from near 0 to near 1 as you cool down IS the order-appears moment. */
static void hud_label_order_parameter(int *col, float m) {
  hud_seg(0, col, g_scene.cols, CP_HUD, A_NORMAL, "  |M|=");
  hud_seg(0, col, g_scene.cols, CP_UP, A_BOLD, "%.4f ", m);
  hud_sparkline(0, col, g_scene.cols);
}

static void hud_label_run_metadata(int *col) {
  hud_seg(0, col, g_scene.cols, CP_HUD, A_DIM, "  sweeps:%lld", g_scene.sweeps);
  hud_seg(0, col, g_scene.cols, CP_HUD, A_NORMAL, "  pat:[%d] ",
          g_scene.pattern);
  hud_seg(0, col, g_scene.cols, CP_DN, A_BOLD, "%s",
          PATTERNS[g_scene.pattern].name);
  hud_seg(0, col, g_scene.cols, CP_HUD, A_NORMAL, "  [");
  hud_seg(0, col, g_scene.cols, CP_UP, A_BOLD, "%s",
          k_themes[g_scene.theme].name);
  hud_seg(0, col, g_scene.cols, CP_HUD, A_NORMAL, "] ");
}

static void hud_label_pause_state(int *col) {
  hud_seg(0, col, g_scene.cols, CP_HUD, A_BOLD, "%s",
          g_scene.paused ? "PAUSED" : "running");
}

/* The top row — the live readout, built left to right from the four
 * labels above. */
static void hud_render_status_bar(float m) {
  int col = 0;
  hud_label_temperature_and_phase(&col);
  hud_label_order_parameter(&col, m);
  hud_label_run_metadata(&col);
  hud_label_pause_state(&col);
}

/* The bottom row — the list of keys you can press, grouped by what they
 * do.  The hot/cold keys borrow the disorder/order colours as a hint. */
static void hud_render_key_legend(void) {
  int b = 0;
  hud_seg(g_scene.rows - 1, &b, g_scene.cols, CP_HINT, A_BOLD, " q:quit ");
  hud_seg(g_scene.rows - 1, &b, g_scene.cols, CP_HINT, A_BOLD, " spc:pause ");
  hud_seg(g_scene.rows - 1, &b, g_scene.cols, CP_DN, A_BOLD, " n/p:pattern ");
  hud_seg(g_scene.rows - 1, &b, g_scene.cols, CP_HINT, A_BOLD, " r:reset ");
  hud_seg(g_scene.rows - 1, &b, g_scene.cols, CP_HUD, A_BOLD, " up/dn:temp ");
  hud_seg(g_scene.rows - 1, &b, g_scene.cols, CP_DN, A_BOLD, " h:hot ");
  hud_seg(g_scene.rows - 1, &b, g_scene.cols, CP_UP, A_BOLD, " c:cold ");
  hud_seg(g_scene.rows - 1, &b, g_scene.cols, CP_HUD, A_BOLD, " t/T:theme ");
}

/* Draw one full frame: paint the grid, measure how lined-up it is, add
 * that reading to the history graph, then draw the two HUD rows. */
static void scene_draw(void) {
  lattice_paint();

  float m = magnetisation();
  mhist_push(m);

  hud_render_status_bar(m);
  hud_render_key_legend();
}

/* ── §7 app ── */

static volatile sig_atomic_t g_quit = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s) {
  if (s == SIGINT || s == SIGTERM)
    g_quit = 1;
  if (s == SIGWINCH)
    g_resize = 1;
}

static void cleanup(void) { endwin(); }

/* ── startup helpers ──
 *
 * main() is split into these named steps so the startup and the main loop
 * each read as a short list of plain verbs. */

static void app_install_signal_handlers(void) {
  signal(SIGINT, sig_h);
  signal(SIGTERM, sig_h);
  signal(SIGWINCH, sig_h);
}

static void app_init_terminal(void) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE); /* getch is non-blocking — UI never stalls */
  curs_set(0);           /* hide hardware cursor                    */
  typeahead(-1);         /* ncurses won't peek stdin mid-diff-write */
}

/* Resize the grid to match the current terminal.  Called once at startup
 * and again whenever the window changes size.  The grid array is always
 * full-size, so this just changes how far the loops run — it never
 * reallocates. */
static void scene_fit_to_terminal(void) {
  int rows, cols;
  getmaxyx(stdscr, rows, cols);
  g_scene.rows = rows;
  g_scene.cols = cols;
  g_scene.gh = (rows - HUD_ROWS) < GRID_H_MAX ? (rows - HUD_ROWS) : GRID_H_MAX;
  g_scene.gw = cols < GRID_W_MAX ? cols : GRID_W_MAX;
}

/* Handle a window resize.  This runs from the main loop, not from the
 * signal handler — the handler only flips a flag, because doing real work
 * inside a signal handler isn't safe.  It rebuilds ncurses, refits the
 * grid, and re-draws the current pattern at the new size. */
static void handle_resize_event(void) {
  g_resize = 0;
  endwin();
  refresh();
  scene_fit_to_terminal();
  apply_current_pattern();
}

/* ── what the keys do ── */

/* Step to the next or previous starting shape (dir is +1 or −1) and draw
 * it.  We add N_PATTERNS before the wraparound because C's remainder can
 * go negative otherwise, which would index off the front of the list. */
static void pattern_advance(int dir) {
  g_scene.pattern = (g_scene.pattern + N_PATTERNS + dir) % N_PATTERNS;
  apply_current_pattern();
}

/* Jump straight to a given temperature, then refresh the flip odds.
 * Used by the h and c keys for instant hot/cold jumps. */
static void temperature_set(float t) {
  g_scene.temp = t;
  boltz_update();
}

/* Nudge the temperature up or down a little (kept inside the allowed
 * range), then refresh the flip odds.  Used by the ↑/↓ keys. */
static void temperature_nudge(float delta) {
  g_scene.temp += delta;
  if (g_scene.temp < T_MIN)
    g_scene.temp = T_MIN;
  if (g_scene.temp > T_MAX)
    g_scene.temp = T_MAX;
  boltz_update();
}

/* Step to the next or previous colour theme and apply it.  Same
 * wraparound trick as pattern_advance to avoid a negative index. */
static void theme_advance(int dir) {
  g_scene.theme = (g_scene.theme + N_THEMES + dir) % N_THEMES;
  theme_apply(g_scene.theme);
}

/* Act on one keypress.  Cases are grouped by what they do. */
static void handle_key_input(int ch) {
  switch (ch) {
  /* lifecycle */
  case 'q':
  case 'Q':
  case 27:
    g_quit = 1;
    break;
  case ' ':
    g_scene.paused = !g_scene.paused;
    break;

  /* simulation reset / pattern */
  case 'r':
  case 'R':
    apply_current_pattern();
    break;
  case 'n':
    pattern_advance(+1);
    break;
  case 'p':
    pattern_advance(-1);
    break;

  /* temperature control */
  case 'h':
  case 'H':
    temperature_set(T_MAX);
    break;
  case 'c':
  case 'C':
    temperature_set(0.5f);
    break;
  case KEY_UP:
    temperature_nudge(+T_STEP);
    break;
  case KEY_DOWN:
    temperature_nudge(-T_STEP);
    break;

  /* visual theme */
  case 't':
    theme_advance(+1);
    break;
  case 'T':
    theme_advance(-1);
    break;

  default:
    break;
  }
}

/* ── per-frame work ── */

/* How many magnets to poke each frame.  Scales with grid size so the
 * blobs move at a watchable pace whether the terminal is big or small,
 * without pinning the CPU on a large one. */
static int metropolis_attempts_per_frame(void) {
  return g_scene.gh * g_scene.gw * FLIPS_PER_CELL / 1000;
}

/* Clear the screen, draw the frame, and push it out in one go. */
static void app_render_frame(void) {
  erase();
  scene_draw();
  wnoutrefresh(stdscr);
  doupdate();
}

/* The whole program: set everything up, then loop forever (until you
 * quit) reading a key, stepping the magnets, drawing, and pausing just
 * long enough to hold a steady frame rate. */
int main(void) {
  /* set everything up */
  srand((unsigned)time(NULL));
  atexit(cleanup);
  app_install_signal_handlers();
  app_init_terminal();
  color_init();
  scene_fit_to_terminal();
  boltz_update();
  apply_current_pattern();

  /* main loop */
  while (!g_quit) {
    if (g_resize)
      handle_resize_event();
    handle_key_input(getch());

    long long now = clock_ns();
    if (!g_scene.paused)
      grid_step(metropolis_attempts_per_frame());
    app_render_frame();
    clock_sleep_ns(RENDER_NS - (clock_ns() - now));
  }
  return 0;
}
