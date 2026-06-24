/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * schrodinger.c — a quantum wave moving along a line, drawn in the terminal.
 *
 * We track a quantum "wave packet" as it drifts, spreads, bounces off walls,
 * and leaks through barriers. Each terminal column is a slot of space: bar
 * height is how likely the particle is to be found there, and bar colour is
 * the wave's phase. Because the colour cycles faster than the packet moves,
 * the bars look like a rolling barber-pole rainbow — that motion is the part
 * of "quantum" you can't see in a plain probability plot.
 *
 * Four scenes (keys 1-4): a free packet that just spreads, a barrier it
 * tunnels through, a bowl-shaped well it sloshes back and forth in, and a
 * double-slit wall that makes an interference pattern.
 *
 * References the code can't tell you:
 *   Crank & Nicolson (1947) — the time-stepping recipe used here.
 *   Thomas (1949) / Numerical Recipes §2.4 — the fast tridiagonal solver.
 *   Griffiths, Intro to Quantum Mechanics — tunnelling (Ch. 8),
 *     the double slit (Ch. 1), wave-packet spreading (Ch. 2).
 *   Thaller, Visual Quantum Mechanics — why colouring by phase makes the
 *     wave's structure readable at a glance.
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

/* How many slots we chop the line of space into. More slots = smoother
 * wave but more work per step. 512 is plenty to draw a clean packet. */
#define N_GRID 512
#define X_MIN 0.f
#define X_MAX 1.f
#define DX ((X_MAX - X_MIN) / (float)(N_GRID - 1)) /* width of one slot */

/* How big a time jump each solver step takes. Smaller = more accurate but
 * slower. This value keeps the wave's shape honest over thousands of steps. */
#define DT 0.0002f

/* How many time steps we run between drawn frames. Bumping this makes the
 * wave move faster on screen without changing the physics. */
#define STEPS_PER_FRAME 20

/* We work in "natural units" where Planck's constant and the particle mass
 * are both 1. This just clears the clutter of constants out of the formulas;
 * every speed and energy below is measured in these simplified units. */
#define HBAR 1.f
#define MASS 1.f

/* Height of the finite barrier in scene 2. It's taller than the packet's
 * energy, so a classical particle would bounce off — but a quantum one
 * leaks through. That leak is what the scene shows off. */
#define V_BARRIER 2000.f

/* A "hard wall": so much taller than any packet that nothing gets through.
 * Used for the edges of the screen and the double-slit wall. */
#define V_WALL 50000.f

#define RENDER_NS (1000000000LL / 30)
#define HUD_TOP_ROWS 1 /* row 0 is the status bar */
#define HUD_BOT_ROWS 2 /* bottom two rows: potential strip, then key hints */
#define N_PHASE 8      /* number of colours in the phase wheel */
#define N_THEMES 10

typedef struct {
  const char *name;
  int id;
} Preset;
static const Preset PRESETS[] = {
    {"Free", 0},
    {"Barrier", 1},
    {"Harmonic", 2},
    {"D-Slit", 3},
};
#define N_PRESETS 4

/* The colour-pair slots ncurses draws with. The first eight are the phase
 * wheel for the wave bars; the next three colour the potential (wall,
 * barrier, free baseline); the last two are the status and hint bars, which
 * stay the same colour no matter which theme is picked so they're always
 * readable. */
enum {
  CP_PHASE_0 = 1,
  CP_PHASE_1,
  CP_PHASE_2,
  CP_PHASE_3,
  CP_PHASE_4,
  CP_PHASE_5,
  CP_PHASE_6,
  CP_PHASE_7,
  CP_WALL,
  CP_BARRIER,
  CP_AXIS,
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

/* ── §3 color / theme ── */

/*
 * Theme — one colour scheme for the demo.
 *
 * The eight `phase` colours are the wheel the wave bars spin through as the
 * wave's phase turns full circle. Themes that vary hue across those eight
 * (Nova, Neon, Oceanic) read as a moving rainbow; themes that mostly vary
 * brightness (Matrix, Fire, Mono) shimmer instead. Both look good.
 *
 * The other three are fixed roles for the potential layer, not part of the
 * wheel: `wall` is the brightest so even a one-column wall is visible,
 * `barrier` is medium so it reads as "you can leak through this", and `axis`
 * is dim so the empty-space baseline doesn't fight the wave for attention.
 *
 * Every colour index stays in the bright half of the 256-colour space, since
 * the dark end shows up as invisible black on a default terminal.
 */
typedef struct {
  const char *name;
  short phase[N_PHASE]; /* the wheel the bars cycle through with the phase */
  short wall;           /* hard wall marker (brightest)   */
  short barrier;        /* finite barrier marker (medium) */
  short axis;           /* empty-space baseline (dim)     */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* name        phase wheel (8 colours)                       wall  barrier
       axis */
    {"Matrix", {28, 34, 40, 46, 82, 118, 154, 190}, 46, 100, 240},
    {"Fire", {124, 160, 196, 202, 208, 214, 220, 226}, 226, 166, 240},
    {"Oceanic", {24, 31, 38, 45, 51, 87, 123, 195}, 51, 38, 240},
    {"Neon", {93, 129, 165, 201, 51, 87, 207, 213}, 201, 129, 240},
    {"Mono", {240, 244, 247, 250, 252, 253, 254, 255}, 255, 247, 240},
    {"Ice", {39, 75, 111, 117, 153, 159, 195, 231}, 231, 117, 240},
    {"Nova", {196, 208, 220, 226, 46, 51, 93, 201}, 231, 208, 240},
    {"Forest", {58, 64, 100, 106, 142, 148, 178, 184}, 154, 142, 240},
    {"Desert", {130, 166, 172, 178, 214, 220, 222, 230}, 230, 178, 240},
    {"Eclipse", {240, 124, 160, 196, 202, 208, 215, 220}, 220, 160, 240},
};

/* Status and hint bar colours — the same on every theme so they stay legible. */
#define CHROME_HUD_256 226 /* bright yellow */
#define CHROME_HINT_256 51 /* bright cyan   */

static bool g_256; /* does the terminal have 256 colours? checked once at boot */

/*
 * theme_apply — load a theme's colours into ncurses. Safe to call any time;
 * the index wraps both ways so the t/T keys don't have to range-check first.
 * The status/hint colours are re-set every call so nothing can leave them
 * stuck on a wrong colour.
 */
static void theme_apply(int idx) {
  const Theme *t = &k_themes[((idx % N_THEMES) + N_THEMES) % N_THEMES];
  if (g_256) {
    for (int i = 0; i < N_PHASE; i++)
      init_pair(CP_PHASE_0 + i, t->phase[i], -1);
    init_pair(CP_WALL, t->wall, -1);
    init_pair(CP_BARRIER, t->barrier, -1);
    init_pair(CP_AXIS, t->axis, -1);
    init_pair(CP_HUD, CHROME_HUD_256, -1);
    init_pair(CP_HINT, CHROME_HINT_256, -1);
  } else {
    /* Old 8-colour terminal: themes don't apply, just use a basic rainbow. */
    static const short fallback[N_PHASE] = {
        COLOR_RED,  COLOR_YELLOW,  COLOR_GREEN, COLOR_CYAN,
        COLOR_BLUE, COLOR_MAGENTA, COLOR_RED,   COLOR_YELLOW,
    };
    for (int i = 0; i < N_PHASE; i++)
      init_pair(CP_PHASE_0 + i, fallback[i], -1);
    init_pair(CP_WALL, COLOR_WHITE, -1);
    init_pair(CP_BARRIER, COLOR_YELLOW, -1);
    init_pair(CP_AXIS, COLOR_WHITE, -1);
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
}

static void color_init(int boot_theme) {
  start_color();
  use_default_colors();
  g_256 = (COLORS >= 256);
  theme_apply(boot_theme);
}

/*
 * phase_color_pair — pick a wheel colour from the wave's phase at one point.
 * The phase is the angle of the (real, imaginary) pair; we turn that full
 * circle into one of the eight colours. The wheel loops, so there's no seam
 * where the angle wraps around.
 */
static inline int phase_color_pair(float re, float im) {
  float angle = atan2f(im, re) + (float)M_PI; /* [0, 2π] */
  int idx = (int)(angle * (float)N_PHASE / (2.f * (float)M_PI));
  if (idx < 0)
    idx = 0;
  if (idx >= N_PHASE)
    idx = N_PHASE - 1;
  return CP_PHASE_0 + idx;
}

/* ── §4 state ── */

/*
 * Cx — a complex number: a value with a real part and an imaginary part.
 * Quantum waves are complex-valued, so this is the basic unit of arithmetic
 * the solver works in. We carry it as two plain floats and write the four
 * operations (+ − × ÷) by hand below, instead of using C's built-in complex
 * type, so the file compiles the same everywhere.
 */
typedef struct {
  float re; /* real part */
  float im; /* imaginary part */
} Cx;

/* cx_sub — subtract two complex numbers, part by part. */
static inline Cx cx_sub(Cx a, Cx b) { return (Cx){a.re - b.re, a.im - b.im}; }

/* cx_mul — multiply two complex numbers (the usual (a+ib)(c+id) rule). */
static inline Cx cx_mul(Cx a, Cx b) {
  return (Cx){a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re};
}

/* cx_div — divide two complex numbers. Safe without a divide-by-zero guard:
 * the only thing we ever divide by here is a solver diagonal, which is always
 * at least 1 in size. */
static inline Cx cx_div(Cx a, Cx b) {
  float mod_sq = b.re * b.re + b.im * b.im;
  return (Cx){(a.re * b.re + a.im * b.im) / mod_sq,
              (a.im * b.re - a.re * b.im) / mod_sq};
}

/*
 * Wavefunction — everything about the quantum wave laid out along the line of
 * space: the wave itself, plus the landscape it moves through. If the physics
 * step needs it to advance one tick, it lives here; anything that's only about
 * how things look on screen lives on Scene instead.
 *
 * The line is chopped into N_GRID evenly-spaced slots, fixed at compile time.
 * Slot i sits at position X_MIN + i·DX. With 512 slots the wave is drawn
 * smoothly enough that the numerical error stays tiny.
 *
 * Two things stay true between every physics tick:
 *   - the wave is zero at both ends (the endpoints are pinned to 0)
 *   - the total probability of finding the particle somewhere stays at 1
 *     (set once at startup; the solver keeps it there on its own)
 * V[] doesn't change once a scene is loaded — only build_potential (§5)
 * writes it.
 *
 * The whole thing is about 8 KB and lives inside the single Scene instance
 * below; nothing here is ever malloc'd.
 *
 * (Born rule |ψ|² = probability, Griffiths §1.3. Gaussian packet spreading,
 * Cohen-Tannoudji complement G_I.)
 */
typedef struct {
  /* The wave is a complex number at every slot, but instead of storing one
   * array of Cx we keep the real and imaginary parts in two separate arrays.
   * That's friendlier to the CPU's memory cache — the solver reads a run of
   * neighbouring slots of one part at a time — and it matches how the solver
   * does its arithmetic (plain real numbers, see cn_rhs_at in §7). */

  /* re[i] — the real part of the wave at slot i. Together with im[i] it gives
   * the full wave value; the two of them set the bar's colour (its phase) in
   * §9. The colour part of the pattern rotates faster than the lump drifts,
   * which is what makes the bars barber-pole. */
  float re[N_GRID];

  /* im[i] — the imaginary part of the wave at slot i. Squared and added to
   * re[i]² it gives how likely the particle is to be found at that slot —
   * which is the bar height drawn in §9. */
  float im[N_GRID];

  /* V[i] — the landscape value at slot i: how hard it is for the wave to be
   * there. 0 is open space, V_BARRIER (2000) is a leak-through bump, and
   * V_WALL (50000) is a solid wall. Each scene's helper in §5 paints a
   * different shape into this array — same wave, same solver, different
   * landscape gives the four completely different behaviours. */
  float V[N_GRID];

  /* k0 — the starting momentum (think: launch speed) of the wave. Bigger k0
   * means a faster-moving, higher-energy lump. It's a single number for the
   * whole wave, not a per-slot value: the HUD shows it, the +/- keys change
   * it, and reloading a scene re-launches the wave at this speed. Once the
   * wave is moving we don't keep this in sync with its true momentum (which
   * flips when it bounces); it's just the launch setting. */
  float k0;
} Wavefunction;

/*
 * TridiagonalSolver — scratch space for the fast solve done every tick.
 *
 * Each step has to solve a chain of equations where every slot is linked only
 * to its two immediate neighbours. That "only-neighbours" shape lets us solve
 * the whole chain with two quick passes (the Thomas algorithm, 1949) instead
 * of grinding through a full matrix. These two arrays are the working notes:
 * the forward pass fills them, the backward pass reads them back out.
 *
 * They keep their values across ticks (never re-zeroed) so there's no
 * allocation per step — every step overwrites them from scratch before
 * reading, so leftover values do no harm. About 8 KB, living inside Scene.
 * (Numerical Recipes §2.4.)
 */
typedef struct {
  /* cp[i] — the folded "talks to itself" number for slot i, worked out on the
   * forward pass and used as a divisor on the backward pass. cp[0] is unused;
   * the passes only touch the interior slots. */
  Cx cp[N_GRID];

  /* tmp[i] — the folded right-hand-side value for slot i: the forward pass
   * computes it, the backward pass divides by cp[i] to get the new wave
   * value. Named tmp on purpose so it reads as throwaway per-step scratch. */
  Cx tmp[N_GRID];
} TridiagonalSolver;

/*
 * Scene — all the demo's state in one place, shared by the physics step, the
 * drawing code, and the main loop. There's a single global instance, so we
 * read it directly rather than passing a pointer through every helper.
 *
 * The fields fall into two groups, and the split is for the reader's sake: the
 * simulation group is everything that changes what the wave does, and the
 * rendering group is purely cosmetic. The rule of thumb when adding a field:
 * if it changes the next wave value, it's simulation; if it only changes how
 * things look, it's rendering. Keeping them apart is what lets you switch
 * themes mid-wave without nudging the physics.
 *
 * A few things deliberately live OUTSIDE this struct: the quit/resize flags
 * (they're written from a signal handler and have to be plain file-scope
 * variables to be safe), the screen size, and the one-time "does this terminal
 * have 256 colours" check.
 */
typedef struct {
  /* ── simulation: everything that drives the physics ── */

  /* wave — the whole quantum state: the wave, the landscape, and the launch
   * momentum. The physics step updates it every tick; loading a scene resets
   * it; the drawing code reads it to paint the bars. See the Wavefunction
   * docstring above for the details. */
  Wavefunction wave;

  /* solver — scratch space the physics step borrows each tick. Recomputed from
   * scratch every step, so its starting value doesn't matter. */
  TridiagonalSolver solver;

  /* preset — which of the four scenes is loaded (0 Free, 1 Barrier,
   * 2 Harmonic, 3 Double-slit). The number keys pick one; the reset key
   * reloads the current one; the energy keys change the launch speed and
   * reload. */
  int preset;

  /* paused — when true, the main loop skips the physics step. The wave freezes
   * exactly where it is and picks up cleanly when unpaused. Shown as a PAUSED
   * badge in the status bar. */
  bool paused;

  /* ── rendering: cosmetic only, never touches the physics ── */

  /* theme — which colour scheme is active (an index into k_themes). The t / T
   * keys step it forward and back. Starts on Nova, the punchiest rainbow. */
  int theme;

  /* fps_disp — the frame rate shown in the status bar, refreshed once a
   * second. Just a display number; it doesn't affect anything. */
  int fps_disp;
} Scene;

/*
 * The one and only Scene. We boot straight into the barrier scene (the best
 * tunnelling show) with the Nova theme and a launch speed of 200. Everything
 * else starts zeroed; main() loads the scene properly before the first frame,
 * so that blank starting state never reaches the screen.
 */
static Scene g_scene = {
    .preset = 1,
    .theme = 6,
    .wave = {.k0 = 200.f},
};

/* ── §5 potential V(x) ── */

/*
 * The "potential" V(x) is the landscape the wave moves through: a number
 * at every slot saying how hard it is to be there. Walls are huge numbers
 * the wave bounces off; a barrier is a medium bump it can leak through;
 * empty space is zero. Each scene just paints a different V landscape, and
 * the solver below treats them all the same way. Each helper paints one
 * feature; build_potential() stacks them.
 */

/*
 * Build hard walls at both ends so the wave can't run off the screen — it
 * bounces back instead. The walls are made thick and very tall on purpose:
 * a thin or short wall would let a little of the wave leak past, which would
 * look like a glitch.
 */
static void install_absorbing_walls(Wavefunction *w) {
  int wall = N_GRID / 20;
  for (int i = 0; i < wall; i++) {
    w->V[i] = V_WALL;
    w->V[N_GRID - 1 - i] = V_WALL;
  }
}

/*
 * Put a single bump in the middle that's taller than the packet's energy.
 * A normal ball would just bounce off it. A quantum wave leaks a bit
 * through to the far side instead — that leak is what this scene shows off.
 */
static void install_finite_barrier(Wavefunction *w) {
  int c = N_GRID / 2;
  int half_width = N_GRID / 60;
  for (int i = c - half_width; i <= c + half_width; i++)
    w->V[i] = V_BARRIER;
}

/*
 * Make a bowl-shaped landscape — low in the middle, rising on both sides,
 * like a valley. Drop the wave off-centre and it slides back and forth,
 * the same way a marble sloshes around a bowl. The steeper the bowl, the
 * faster it sloshes.
 */
static void install_harmonic_potential(Wavefunction *w) {
  const float k_spring = 8000.f;
  for (int i = 0; i < N_GRID; i++) {
    float x = (float)i / (float)(N_GRID - 1) - 0.5f;
    w->V[i] += 0.5f * k_spring * x * x;
  }
}

/*
 * The famous double-slit: a wall across the middle with two narrow gaps.
 * The wave goes through both gaps at once, and the two pieces overlap on
 * the far side and add up into a striped pattern — bright where they
 * reinforce, dark where they cancel. Feynman called this the one true
 * mystery of quantum mechanics. We build the solid wall first, then punch
 * two gaps in it. (Young's experiment; Feynman Lectures Vol. III Ch. 1.)
 */
static void install_double_slit(Wavefunction *w) {
  int c = N_GRID / 2;
  int slit_half = N_GRID / 100 + 1;
  int sep_half = N_GRID / 24;

  /* Solid wall across the middle. */
  for (int i = c - 2; i <= c + 2; i++)
    w->V[i] = V_WALL;

  /* Punch two gaps, one each side of centre. */
  int s1 = c - sep_half;
  int s2 = c + sep_half;
  for (int i = s1 - slit_half; i <= s1 + slit_half; i++)
    w->V[i] = 0.f;
  for (int i = s2 - slit_half; i <= s2 + slit_half; i++)
    w->V[i] = 0.f;
}

/*
 * Paint the landscape for one scene: wipe it flat, add the walls that every
 * scene needs at the edges, then drop in whatever feature this scene calls
 * for.
 */
static void build_potential(Wavefunction *w, int preset) {
  for (int i = 0; i < N_GRID; i++)
    w->V[i] = 0.f;
  install_absorbing_walls(w);

  switch (preset) {
  case 0: /* open space — nothing extra to add */
    break;
  case 1:
    install_finite_barrier(w);
    break;
  case 2:
    install_harmonic_potential(w);
    break;
  case 3:
    install_double_slit(w);
    break;
  }
}

/* ── §6 initial conditions — Gaussian wave packet ── */

/*
 * We start the wave as a bell-shaped lump (a "Gaussian"). It's the tidiest
 * possible starting shape — as compact as quantum rules allow — so you can
 * clearly see it move as one blob. Left to itself it slowly fattens and
 * blurs over time, which is exactly what the free-particle scene shows.
 * (Cohen-Tannoudji, complement G_I.)
 */

/*
 * Lay down the starting wave: a bell-shaped lump centred at x0, wrapped in a
 * fast wiggle whose speed is set by k0 (more wiggle = more momentum). The
 * wiggle splits into a cosine part (the real array) and a sine part (the
 * imaginary array) — that's where the rolling colour rainbow comes from.
 * The values aren't scaled yet; normalize_wavefunction does that next.
 */
static void make_gaussian_wavepacket(Wavefunction *w, float x0, float sigma,
                                     float k0) {
  w->k0 = k0;
  for (int i = 0; i < N_GRID; i++) {
    float x = (float)i / (float)(N_GRID - 1);
    float envelope = expf(-(x - x0) * (x - x0) / (2.f * sigma * sigma));
    w->re[i] = envelope * cosf(k0 * x);
    w->im[i] = envelope * sinf(k0 * x);
  }
}

/*
 * Scale the whole wave so the total probability of finding the particle
 * somewhere adds up to exactly 1 — otherwise the bar heights would be
 * arbitrary. We only need this once at the start: the solver below keeps the
 * total at 1 on its own as the wave evolves. (Griffiths §1.4.)
 */
static void normalize_wavefunction(Wavefunction *w) {
  float norm_sq = 0.f;
  for (int i = 0; i < N_GRID; i++)
    norm_sq += w->re[i] * w->re[i] + w->im[i] * w->im[i];

  float scale = 1.f / sqrtf(norm_sq * DX);
  for (int i = 0; i < N_GRID; i++) {
    w->re[i] *= scale;
    w->im[i] *= scale;
  }
}

/*
 * Force the wave to be exactly zero at the two far ends of the line. The
 * walls already keep the wave away from the edges, but nailing the endpoints
 * to zero keeps them out of the solver's bookkeeping entirely, which is
 * tidier and a touch faster. (Griffiths §2.2.)
 */
static void apply_dirichlet_boundary(Wavefunction *w) {
  w->re[0] = w->re[N_GRID - 1] = 0.f;
  w->im[0] = w->im[N_GRID - 1] = 0.f;
}

/*
 * Set up the starting wave for whichever scene is loaded. Each scene gets its
 * own starting position, width, and momentum, hand-tuned so the effect reads
 * clearly: e.g. the barrier scene starts slow enough that it can't simply
 * blast over the bump, so you actually see it leak through.
 */
static void init_wavepacket(Wavefunction *w, int preset) {
  float x0, sigma, k0;
  switch (preset) {
  case 0:
    x0 = 0.2f;
    sigma = 0.05f;
    k0 = 150.f;
    break;
  case 1:
    x0 = 0.2f;
    sigma = 0.05f;
    k0 = 200.f;
    break;
  case 2:
    x0 = 0.3f;
    sigma = 0.07f;
    k0 = 50.f;
    break;
  case 3:
    x0 = 0.2f;
    sigma = 0.04f;
    k0 = 200.f;
    break;
  default:
    x0 = 0.2f;
    sigma = 0.05f;
    k0 = 150.f;
    break;
  }
  make_gaussian_wavepacket(w, x0, sigma, k0);
  normalize_wavefunction(w);
  apply_dirichlet_boundary(w);
}

/* ── §7 Crank-Nicolson time evolution + Thomas algorithm ── */

/*
 * This is the heart of the demo: the recipe that nudges the whole wave
 * forward by one small slice of time. The trick (Crank & Nicolson, 1947) is
 * to half-trust the wave's current shape and half-trust where it's heading,
 * which keeps the answer stable over thousands of steps where a naive method
 * would blow up.
 *
 * Doing it means solving a linked chain of equations — each slot's new value
 * depends on its two neighbours. That chain has a special "only-touch-your-
 * neighbours" shape, so we can solve it in one quick forward pass and one
 * quick backward pass (the Thomas algorithm) instead of grinding through a
 * full matrix. The three helpers below do exactly that, and
 * crank_nicolson_step ties them together.
 */

/* A small constant the formulas below lean on, worked out once. */
static inline float cn_offdiagonal_factor(void) { return DT / (4.f * DX * DX); }

/* The "this slot talks to itself" coefficient for one row of the chain. */
static inline Cx cn_diagonal_at(int i, const Wavefunction *w, float r) {
  return (Cx){1.f, 2.f * r + 0.5f * DT * w->V[i]};
}

/*
 * Build the known right-hand side for one slot — what the wave looks like now,
 * tilted a little toward where its neighbours are pulling it. We write the
 * real and imaginary parts out by hand here rather than calling the complex-
 * number helpers, since this runs once per slot every step and the longhand
 * is plain enough to read.
 */
static inline Cx cn_rhs_at(int i, const Wavefunction *w, float r) {
  float V_term = 0.5f * DT * w->V[i];
  float re_b = w->re[i] + r * (w->im[i - 1] - 2.f * w->im[i] + w->im[i + 1]) -
               V_term * w->im[i];
  float im_b = w->im[i] - r * (w->re[i - 1] - 2.f * w->re[i] + w->re[i + 1]) +
               V_term * w->re[i];
  return (Cx){re_b, im_b};
}

/*
 * First half of the solve: walk left-to-right and fold each slot's dependence
 * on its left neighbour into the next slot, leaving a simpler chain that only
 * looks rightward. We stash the folded numbers in cp[] and tmp[] for the
 * backward pass to read.
 */
static void thomas_forward_sweep(const Wavefunction *w, TridiagonalSolver *s,
                                 float r) {
  Cx sub = {0.f, -r};
  Cx sup = {0.f, -r};

  for (int i = 1; i < N_GRID - 1; i++) {
    Cx d_i = cn_diagonal_at(i, w, r);
    Cx b_i = cn_rhs_at(i, w, r);

    if (i == 1) {
      s->cp[i] = d_i;
      s->tmp[i] = b_i;
    } else {
      Cx pivot = cx_div(sub, s->cp[i - 1]);
      s->cp[i] = cx_sub(d_i, cx_mul(pivot, sup));
      s->tmp[i] = cx_sub(b_i, cx_mul(pivot, s->tmp[i - 1]));
    }
  }
}

/*
 * Second half of the solve: now that each slot only looks rightward, start
 * from the far end (where the answer is immediate) and walk back left, filling
 * in each new wave value from the one to its right. This overwrites the wave
 * arrays with the next time-step's wave. The two endpoints are left alone here
 * — the boundary step pins them to zero right after.
 */
static void thomas_back_substitute(Wavefunction *w,
                                   const TridiagonalSolver *s) {
  Cx sup = {0.f, -DT / (4.f * DX * DX)};
  int last = N_GRID - 2;

  Cx psi = cx_div(s->tmp[last], s->cp[last]);
  w->re[last] = psi.re;
  w->im[last] = psi.im;

  for (int i = last - 1; i >= 1; i--) {
    Cx psi_next = {w->re[i + 1], w->im[i + 1]};
    psi = cx_div(cx_sub(s->tmp[i], cx_mul(sup, psi_next)), s->cp[i]);
    w->re[i] = psi.re;
    w->im[i] = psi.im;
  }
}

/*
 * Advance the whole wave by one time slice: forward pass, backward pass, then
 * pin the ends to zero. main() runs this many times between drawn frames so
 * the wave moves at a watchable speed.
 */
static void crank_nicolson_step(Wavefunction *w, TridiagonalSolver *s) {
  float r = cn_offdiagonal_factor();
  thomas_forward_sweep(w, s, r);
  thomas_back_substitute(w, s);
  apply_dirichlet_boundary(w);
}

/* ── §8 driver + diagnostics ── */

/*
 * Switch to a scene from scratch: paint its landscape, drop in a fresh
 * starting wave, and remember which scene we're on. This is what the number
 * keys, the reset key, and the energy keys all call.
 */
static void load_preset(Scene *sc, int preset) {
  sc->preset = preset;
  build_potential(&sc->wave, preset);
  init_wavepacket(&sc->wave, preset);
}

/*
 * Measure how much of the wave sits on the left half of the screen versus the
 * right half. In the barrier scene that's the headline number: R is the
 * fraction that bounced back, T is the fraction that leaked through, and they
 * add up to 1. In the other scenes it's just a handy "does it still add up to
 * 1?" sanity check that the wave hasn't lost or gained probability.
 */
static void compute_reflection_transmission(const Wavefunction *w, float *R_out,
                                            float *T_out) {
  float left = 0.f, right = 0.f;
  int mid = N_GRID / 2;
  for (int i = 1; i < mid; i++)
    left += (w->re[i] * w->re[i] + w->im[i] * w->im[i]) * DX;
  for (int i = mid; i < N_GRID - 1; i++)
    right += (w->re[i] * w->re[i] + w->im[i] * w->im[i]) * DX;

  float total = left + right + 1e-10f;
  *R_out = left / total;
  *T_out = right / total;
}

/*
 * Give the wave a shove — add momentum so it speeds up in one direction
 * without moving where it currently sits. It's the "punch extra energy in"
 * button (the spacebar): the lump stays put but starts drifting faster.
 */
static void apply_momentum_kick(Wavefunction *w, float k) {
  for (int i = 0; i < N_GRID; i++) {
    float x = (float)i / (float)(N_GRID - 1);
    float kre = cosf(k * x);
    float kim = sinf(k * x);
    float new_re = w->re[i] * kre - w->im[i] * kim;
    float new_im = w->re[i] * kim + w->im[i] * kre;
    w->re[i] = new_re;
    w->im[i] = new_im;
  }
}

/* ── §9 draw — phase-coloured bars + potential strip + HUD ── */

static int g_rows, g_cols; /* current terminal size, updated on resize */

/*
 * The main picture: one vertical bar per column. A bar's height is how likely
 * the particle is to be found there, and its colour is the wave's phase at
 * that spot — and since the phase spins faster than the lump moves, the
 * colours roll through the bars like a barber pole. Heights are rescaled each
 * frame to the tallest bar so the wave always fills the screen even as it
 * spreads thin.
 */
static void draw_phase_bars(int plot_top, int plot_rows) {
  int baseline = plot_top + plot_rows - 1;

  /* Find the tallest bar so we can scale everything to fit. */
  float max_prob = 1e-12f;
  for (int i = 0; i < N_GRID; i++) {
    float p = g_scene.wave.re[i] * g_scene.wave.re[i] +
              g_scene.wave.im[i] * g_scene.wave.im[i];
    if (p > max_prob)
      max_prob = p;
  }

  float pts_per_col = (float)N_GRID / (float)g_cols;
  int bar_room = plot_rows - 1;

  for (int col = 0; col < g_cols; col++) {
    int gi = (int)((float)col * pts_per_col);
    if (gi >= N_GRID)
      gi = N_GRID - 1;

    float re = g_scene.wave.re[gi];
    float im = g_scene.wave.im[gi];
    float prob = re * re + im * im;

    int h = (int)(prob / max_prob * (float)bar_room + 0.5f);
    if (h <= 0)
      continue;
    if (h > bar_room)
      h = bar_room;

    int cp = phase_color_pair(re, im);
    attron(COLOR_PAIR(cp) | A_BOLD);
    /* The shaft of the bar. */
    for (int dy = 0; dy < h - 1; dy++)
      mvaddch(baseline - dy, col, '|');
    /* A different glyph at the very top marks the tip. */
    mvaddch(baseline - (h - 1), col, ':');
    attroff(COLOR_PAIR(cp) | A_BOLD);
  }
}

/*
 * A one-row map of the landscape the wave moves through, drawn just below the
 * bars: a bright '|' marks a solid wall, a '#' marks a leak-through barrier,
 * and a dim '.' marks empty space. Lets you see at a glance where the walls
 * and bumps are.
 */
static void draw_potential_strip(int row) {
  float pts_per_col = (float)N_GRID / (float)g_cols;
  for (int col = 0; col < g_cols; col++) {
    int gi = (int)((float)col * pts_per_col);
    if (gi >= N_GRID)
      gi = N_GRID - 1;
    float V = g_scene.wave.V[gi];

    if (V >= V_WALL * 0.5f) {
      attron(COLOR_PAIR(CP_WALL) | A_BOLD);
      mvaddch(row, col, '|');
      attroff(COLOR_PAIR(CP_WALL) | A_BOLD);
    } else if (V > 0.f) {
      attron(COLOR_PAIR(CP_BARRIER) | A_BOLD);
      mvaddch(row, col, '#');
      attroff(COLOR_PAIR(CP_BARRIER) | A_BOLD);
    } else {
      attron(COLOR_PAIR(CP_AXIS));
      mvaddch(row, col, '.');
      attroff(COLOR_PAIR(CP_AXIS));
    }
  }
}

/*
 * The status bar across the top: which scene, the momentum, how much of the
 * wave is on each side (R and T), the theme name, whether it's paused, and the
 * frame rate.
 */
static void draw_hud_top(int fps) {
  float R = 0.f, T = 0.f;
  compute_reflection_transmission(&g_scene.wave, &R, &T);

  char buf[200];
  snprintf(buf, sizeof buf,
           " %s  k0:%5.0f  R:%.2f T:%.2f  theme:%s  %s  %d fps ",
           PRESETS[g_scene.preset].name, (double)g_scene.wave.k0, (double)R,
           (double)T, k_themes[g_scene.theme].name,
           g_scene.paused ? "PAUSED " : "running", fps);
  int col = g_cols - (int)strlen(buf);
  if (col < 0)
    col = 0;

  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, col, buf, g_cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* The key-hint bar along the bottom; shrinks to a short version when the
 * terminal is too narrow for the full list. */
static void draw_hud_bottom(void) {
  const char *full = " q:quit  p:pause  r:reset  1-4:preset  +/-:energy  "
                     "spc:kick  t/T:theme ";
  const char *shrt = " q:quit  p:pause  1-4:preset  t:theme ";
  const char *h = (int)strlen(full) >= g_cols - 1 ? shrt : full;

  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(g_rows - 1, 0, h, g_cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/*
 * Draw one whole frame, top to bottom: the wave bars, the landscape strip
 * under them, and the two status bars.
 */
static void scene_draw(void) {
  int plot_top = HUD_TOP_ROWS;
  int plot_rows = g_rows - HUD_TOP_ROWS - HUD_BOT_ROWS;
  if (plot_rows < 3)
    plot_rows = 3;

  draw_phase_bars(plot_top, plot_rows);
  draw_potential_strip(g_rows - 2);
  draw_hud_top(g_scene.fps_disp);
  draw_hud_bottom();
}

/* ── §10 app — signal handlers + main loop ── */

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
  srand((unsigned)time(NULL));
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
  load_preset(&g_scene, g_scene.preset);

  long long fps_window_start = clock_ns();
  int fps_frames = 0;

  while (!g_quit) {

    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, g_rows, g_cols);
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
      load_preset(&g_scene, g_scene.preset);
      break;
    case '1':
      load_preset(&g_scene, 0);
      break;
    case '2':
      load_preset(&g_scene, 1);
      break;
    case '3':
      load_preset(&g_scene, 2);
      break;
    case '4':
      load_preset(&g_scene, 3);
      break;
    case '+':
    case '=':
      g_scene.wave.k0 *= 1.2f;
      if (g_scene.wave.k0 > 600.f)
        g_scene.wave.k0 = 600.f;
      load_preset(&g_scene, g_scene.preset);
      break;
    case '-':
      g_scene.wave.k0 *= 0.833f;
      if (g_scene.wave.k0 < 30.f)
        g_scene.wave.k0 = 30.f;
      load_preset(&g_scene, g_scene.preset);
      break;
    case 't':
      g_scene.theme = (g_scene.theme + 1) % N_THEMES;
      theme_apply(g_scene.theme);
      break;
    case 'T':
      g_scene.theme = (g_scene.theme + N_THEMES - 1) % N_THEMES;
      theme_apply(g_scene.theme);
      break;
    case ' ':
      apply_momentum_kick(&g_scene.wave, 100.f);
      break;
    default:
      break;
    }

    long long now = clock_ns();
    if (!g_scene.paused)
      for (int s = 0; s < STEPS_PER_FRAME; s++)
        crank_nicolson_step(&g_scene.wave, &g_scene.solver);

    /* Count frames and report the rate once a second. */
    fps_frames++;
    if (now - fps_window_start >= 1000000000LL) {
      g_scene.fps_disp = fps_frames;
      fps_frames = 0;
      fps_window_start = now;
    }

    erase();
    scene_draw();
    wnoutrefresh(stdscr);
    doupdate();
    clock_sleep_ns(RENDER_NS - (clock_ns() - now));
  }
  return 0;
}
