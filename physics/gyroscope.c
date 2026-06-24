/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * gyroscope.c — a spinning top you can watch tumble.
 *
 * Simulates how a rigid body spins and wobbles in 3-D, then draws it as a
 * wireframe in the terminal.  Eight presets show off the classic behaviours
 * of tops and tumbling objects (steady precession, gravity wobble, the
 * tennis-racket flip).  Cycle them with n / p; full key list is in §8.
 *
 * The interesting physics it captures: a spinning object's axis traces a
 * cone instead of pointing in one place (precession), it bobs up and down
 * while it does (nutation), and an asymmetric object spun about its middle
 * axis flips over unpredictably (the Dzhanibekov effect).
 *
 * Built on the project's framework.c §1–§8 layout.
 *
 * The physics and rendering ideas, and where they come from, are written
 * up in the CONCEPTS block just below.
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * How it moves   : We track the body's spin rate and which way it's facing,
 *                  then step both forward a tiny bit at a time using RK4 — a
 *                  well-known recipe for advancing physics accurately [6].
 *                  We need the accurate recipe because the spin equations
 *                  feed back on themselves; the cheap method (plain Euler
 *                  steps) visibly drifts within seconds.  We take 8 small
 *                  sub-steps per frame so even the fastest wobble is captured.
 *
 * The physics    : Euler's equations — the rules for how a rigid body's
 *                  spin changes over time [1, 2].  Each object has three
 *                  "how hard to spin about this axis" numbers (I1, I2, I3).
 *                  The 8 presets show three famous situations:
 *                    • A symmetric top spinning freely — its axis sweeps
 *                          out a steady cone [1]            (presets 0, 3)
 *                    • A top under gravity — it wobbles and bobs; spin it
 *                          fast enough and it "sleeps" nearly upright [1, 2]
 *                                                           (presets 1, 4, 5)
 *                    • The tennis-racket flip [3] — spin an asymmetric
 *                          object about its MIDDLE axis and it keeps
 *                          flipping over; about the biggest or smallest
 *                          axis it's perfectly steady (preset 2 vs 6, 7).
 *
 * Which way it   : We store orientation as a quaternion — four numbers that
 * faces          :  encode a 3-D rotation cleanly, with none of the jams or
 *                  blind spots that plague the simpler angle methods [4].
 *                  quat_to_axes (§5) turns it back into the three body axes.
 *                  Stepping it forward nudges it slightly off being a valid
 *                  rotation, so after each step project_quaternion_to_unit
 *                  rescales it and refresh_body_axes squares the axes back up
 *                  (Gram-Schmidt) — cheap fixes that keep things honest [5].
 *
 * How it's drawn : A flat 3-D-to-2-D projection of a wireframe [8], painted
 *                  back-to-front in six layers (ground ring → upright marker
 *                  → body disc → trail → body axes → momentum arrow) so
 *                  nearer parts cover farther ones.  All the camera numbers
 *                  ride together in one Viewport struct.  Terminal cells are
 *                  about twice as tall as wide, so ASPECT ≈ 0.5 squashes the
 *                  vertical to keep circles round.  Each line picks a glyph
 *                  (- / | \) from its on-screen angle [7], and nearer parts
 *                  are bold while farther parts dim — a depth effect for free.
 *
 * The colours    : 12 diverging palettes [9] — each runs cool → bright →
 *                  warm.  The body's X and Y axes get the cool and warm mid
 *                  tones so you can tell them apart; the spin axis (Z) and
 *                  the momentum arrow (L) get the brightest extremes so the
 *                  two stars of the show stand out in any theme.
 *
 * References (cite inline as [n]):
 *
 *   [1] Goldstein, H.; Poole, C.; Safko, J. — *Classical Mechanics*,
 *       3rd ed., Addison-Wesley (2002).  The canonical English-language
 *       reference for rigid-body rotation: Ch. 4 (kinematics, Euler
 *       angles), Ch. 5 (Euler's equations, symmetric top, heavy
 *       symmetric top, sleeping-top stability threshold, Lagrange
 *       top).  Backs presets 0, 1, 3, 4, 5.
 *
 *   [2] Landau, L. D.; Lifshitz, E. M. — *Mechanics*, Vol. 1 of the
 *       Course of Theoretical Physics, 3rd ed., Butterworth-Heinemann
 *       (1976).  §§32-37 cover the rigid body — concise, elegant
 *       derivations of Euler's equations and the symmetric top with
 *       and without gravity.  Cross-check / second source for [1].
 *
 *   [3] Ashbaugh, M. S.; Chicone, C. C.; Cushman, R. H. (1991) —
 *       "The Twisting Tennis Racket", *J. Dynamics and Differential
 *       Equations* 3 (1), 67–85.  Rigorous analysis of the
 *       Dzhanibekov / intermediate-axis instability: proves rotation
 *       about the middle principal axis is unstable and derives the
 *       flip period from the inertia eigenvalues.  Backs preset 2
 *       and the comparison-stability presets 6 / 7.
 *
 *   [4] Diebel, J. (2006) — "Representing Attitude: Euler Angles,
 *       Unit Quaternions, and Rotation Vectors", Stanford tech report.
 *       Practical compendium of every formula needed to manipulate
 *       attitude representations: q ↔ R conversion, kinematics
 *       q̇ = ½ q ⊗ (0, ω), composition rules.  gyro_deriv and
 *       quat_to_axes in §5 follow this report's conventions verbatim.
 *
 *   [5] Hairer, E.; Lubich, C.; Wanner, G. — *Geometric Numerical
 *       Integration*, 2nd ed., Springer (2006).  Structure-preserving
 *       integrators for mechanical systems on manifolds.  Explains
 *       why naive RK4 drifts off SO(3) / S³ and why a simple
 *       projection step (q /= |q| + Gram-Schmidt on the extracted
 *       axes) is sufficient for short-horizon integration — the
 *       rationale for the post-step cleanup in gyro_step.
 *
 *   [6] Press, W. H.; Teukolsky, S. A.; Vetterling, W. T.;
 *       Flannery, B. P. — *Numerical Recipes*, 3rd ed., Cambridge
 *       University Press (2007).  Ch. 17 on ODE integration — the
 *       classical RK4 coefficients (k1..k4,
 *       y_{n+1} = y_n + (k1 + 2k2 + 2k3 + k4) · dt/6) implemented
 *       verbatim in gyro_step.
 *
 *   [7] Bresenham, J. E. (1965) — "Algorithm for Computer Control of
 *       a Digital Plotter", *IBM Systems Journal* 4 (1), 25–30.
 *       8-direction line discretisation — 360° quantised into eight
 *       45° sectors, each mapped to one of {-, /, |, \}.  Used in
 *       dir_char (§4) and provides the DDA stepping inside
 *       draw_seg3d.
 *
 *   [8] Foley, J. D.; van Dam, A.; Feiner, S. K.; Hughes, J. F. —
 *       *Computer Graphics: Principles and Practice*, 3rd ed.,
 *       Addison-Wesley (2013).  Reference for 3-D viewing transforms,
 *       orthographic projection, and depth cueing.  Backs the
 *       project() function in §4 (azimuth + elevation rotation, then
 *       drop the depth coordinate for the screen, retain it for the
 *       bold/dim shading).
 *
 *   [9] Ware, C. — *Information Visualization: Perception for Design*,
 *       4th ed., Morgan Kaufmann (2020).  Ch. 4 on colour and
 *       perception: why DIVERGING palettes (cool family ↔ warm family
 *       with a bright neutral midpoint) are the canonical choice for
 *       signed / bipolar data.  Backs the 12-theme palette in §3
 *       (shared verbatim with charged_particles.c).
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

/* ── §1  config ───────────────────────────────────────────────────────── */

enum {
  SIM_FPS_MIN = 20,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,

  SUB_STEPS = 8,   /* small physics steps per frame; more = steadier */
  TRAIL_LEN = 300, /* how many past trail dots we remember            */
  DISC_PTS = 32,   /* dots drawn around the spinning disc             */
  GROUND_PTS = 48, /* dots drawn around the ground ring               */
  FPS_UPDATE_MS = 500,
  N_COLORS = 7,
  N_PRESETS = 8,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* ── Preset table ────────────────────────────────────────────────────── */

/*
 * GPreset — the starting setup for one of the eight demos.
 *
 * Each preset is everything we need to recreate a named classic case
 * (Euler's top, Dzhanibekov flip, sleeping top, ...): the body's shape,
 * how fast and which way it starts spinning, how far it's tilted, and
 * whether gravity is acting.  PRESETS[] holds all eight; gyro_set_preset()
 * loads one into the live simulation at startup and whenever you reset.
 * It's a plain record because every preset answers the same few questions —
 * to add one, append to PRESETS[] and bump N_PRESETS.
 *
 * The fields, plainly:
 *
 *   Body shape — I1, I2, I3:
 *     How hard the body is to spin about each of its three axes (kg·m²).
 *     Two equal → a symmetric, top-like body; all three different → an
 *     asymmetric body that can do the tennis-racket flip (presets 2, 6, 7).
 *
 *   How it starts:
 *     omega[3]   — starting spin rate about each body axis (rad/s).  Where
 *                  you put the spin chooses the behaviour: all on Z → a
 *                  clean spinning top; mostly on the middle axis of an
 *                  asymmetric body → the unstable flip [3].
 *     tilt_deg   — how far to lean the body over at the start (degrees).
 *                  Gravity can only act on a leaning top, so the gravity
 *                  presets need a nonzero tilt to get going.
 *     tilt_axis  — which direction to lean it in (auto-normalised on load).
 *
 *   Gravity:
 *     gravity    — false: nothing pulls on it (free spin).
 *                  true:  gravity tugs the leaning top, making it precess.
 *     mgl        — how strong that gravity tug is (N·m).  Bigger means a
 *                  faster, wider precession wobble.
 */
typedef struct {
  const char *name;   /* shown in the HUD                             */
  float I1, I2, I3;   /* how hard to spin about each body axis        */
  float omega[3];     /* starting spin rate per body axis (rad/s)     */
  float tilt_deg;     /* starting lean of the body (degrees)          */
  float tilt_axis[3]; /* direction to lean in (auto-normalised)       */
  bool gravity;       /* does gravity act on it?                      */
  float mgl;          /* strength of the gravity tug (N·m)            */
} GPreset;

/*
 * The eight presets, in plain terms (numbers picked to make each effect
 * obvious on screen):
 *
 * 0 Euler's Top — a saucer-shaped top spinning freely.  Its axis sweeps
 *     out a steady cone; the momentum arrow L stays fixed while the body
 *     circles around it.
 *
 * 1 Gravity Top — same body, now under gravity.  It precesses (a slow
 *     circle) while bobbing (a fast small wobble) — the textbook top.
 *
 * 2 Dzhanibekov — an asymmetric body spun about its middle axis.  It
 *     keeps flipping a full half-turn, roughly once a second.  The
 *     famous tennis-racket / T-handle effect.
 *
 * 3 Oblate Top — like preset 0 but with the shape ratio flipped, so the
 *     axis sweeps its cone the OTHER way.  Set side by side with 0 to see
 *     the difference.
 *
 * 4 Sleeping Top — gravity plus a very fast spin.  It barely moves and
 *     stays nearly upright; spin a top hard enough and it "sleeps".
 *
 * 5 Slow Heavy Top — gravity plus a slow spin.  Wide lazy precession
 *     circles with a clear dip-and-recover bob each lap.  The opposite
 *     extreme of preset 4.
 *
 * 6 Stable Major — the asymmetric body from preset 2, but spun about its
 *     biggest axis.  Rock steady — small nudges stay small.
 *
 * 7 Stable Minor — same body spun about its smallest axis.  Also rock
 *     steady.  Presets 2, 6, 7 together make the point: only the MIDDLE
 *     axis is unstable; the two extremes are fine.
 */
static const GPreset PRESETS[N_PRESETS] = {
    {/* Euler's Top */
     "Euler\\'s Top   (torque-free, symmetric)",
     2.0f,
     2.0f,
     1.0f,
     {0.8f, 0.0f, 8.0f},
     18.0f,
     {1.0f, 0.0f, 0.0f},
     false,
     0.0f},
    {/* Gravity Top */
     "Gravity Top    (precession + nutation)",
     2.0f,
     2.0f,
     1.0f,
     {0.0f, 0.0f, 12.0f},
     25.0f,
     {1.0f, 0.0f, 0.0f},
     true,
     1.5f},
    {/* Dzhanibekov effect */
     "Dzhanibekov    (intermediate-axis flip)",
     1.0f,
     2.5f,
     3.5f,
     {0.05f, 8.0f, 0.05f},
     10.0f,
     {0.0f, 0.0f, 1.0f},
     false,
     0.0f},
    {/* Oblate Top — opposite-sign precession from preset 0 */
     "Oblate Top     (opposite precession)",
     2.0f,
     2.0f,
     3.0f,
     {0.5f, 0.0f, 5.0f},
     18.0f,
     {1.0f, 0.0f, 0.0f},
     false,
     0.0f},
    {/* Sleeping Top — fast spin, almost upright */
     "Sleeping Top   (fast spin under gravity)",
     2.0f,
     2.0f,
     1.0f,
     {0.0f, 0.0f, 25.0f},
     3.0f,
     {1.0f, 0.0f, 0.0f},
     true,
     1.5f},
    {/* Slow Heavy Top — wide precession, cusped nutation */
     "Slow Heavy Top (wide precession circles)",
     2.0f,
     2.0f,
     1.0f,
     {0.0f, 0.0f, 4.0f},
     40.0f,
     {1.0f, 0.0f, 0.0f},
     true,
     2.0f},
    {/* Stable Major — asymmetric, spin on max-I axis */
     "Stable Major   (asym, spin on max-I axis)",
     1.0f,
     2.5f,
     3.5f,
     {0.05f, 0.05f, 6.0f},
     15.0f,
     {1.0f, 0.0f, 0.0f},
     false,
     0.0f},
    {/* Stable Minor — asymmetric, spin on min-I axis */
     "Stable Minor   (asym, spin on min-I axis)",
     1.0f,
     2.5f,
     3.5f,
     {6.0f, 0.05f, 0.05f},
     15.0f,
     {0.0f, 0.0f, 1.0f},
     false,
     0.0f},
};

/* ── §2  clock ────────────────────────────────────────────────────────── */

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

/* ── §3  themes + color ───────────────────────────────────────────────── */

/*
 * Color-pair slots — the drawing code always asks for these by name, and
 * the actual colour behind each one is swapped when you cycle themes (t/T).
 * Keeping the slot numbers fixed means no drawing code has to change.
 */
enum {
  CP_AXIS_X = 1, /* body X axis                                     */
  CP_AXIS_Y = 2, /* body Y axis                                     */
  CP_AXIS_Z = 3, /* body Z axis (spin axis — most prominent)        */
  CP_MOM = 4,    /* angular momentum L                              */
  CP_GROUND = 5, /* ground ring + world-Z reference (rendered A_DIM)*/
  CP_TRAIL = 6,  /* polhode trail                                   */
  CP_DISC = 7,   /* body-disc equator                               */
  PAIR_HUD = 8,  /* HUD parameter line (yellow, canonical)          */
  PAIR_HINT = 9, /* key-legend line (cyan, canonical)               */
};

/*
 * Theme — one colour palette, shared with charged_particles.c.
 *
 * Each theme is an 8-step gradient that runs cool → bright middle → warm.
 * The drawing code pulls specific slots out of that gradient for each
 * element (see color_apply_theme below for which slot goes where).  The
 * gradient tells the elements apart by HUE, not by brightness — every slot
 * is deliberately kept in the bright half of the colour range so nothing
 * vanishes against a dark terminal even when drawn dim.
 *
 *   name  — what the theme is called (shown in the HUD)
 *   ramp  — the 8-step cool-to-warm gradient
 *   trail — the colour used for the spin-axis trail
 *   sky   — not used here; kept so this table matches charged_particles.c
 */
typedef struct {
  const char *name;
  short ramp[8];
  short trail;
  short sky;
} Theme;

#define N_THEMES 12

static const Theme THEMES[N_THEMES] = {
    /*  name        ramp[0..7]  (cool → neutral → warm)   trail  sky */
    {"VOLT",
     {81, 117, 123, 159, 230, 220, 214, 203},
     250,
     246}, /* electric blue → yellow → red (tri)     */
    {"COPPER",
     {110, 117, 153, 195, 230, 222, 215, 209},
     250,
     246}, /* steel blue → cream → copper red (di)   */
    {"NEON",
     {75, 117, 159, 195, 230, 218, 213, 205},
     250,
     246}, /* blue → cyan → pink (tri)               */
    {"ICE_FIRE",
     {81, 117, 159, 195, 230, 215, 209, 196},
     250,
     246}, /* ice blue → flame red (di, classic)     */
    {"AURORA",
     {84, 120, 156, 195, 230, 218, 213, 205},
     250,
     246}, /* green → pink (di, aurora-like)         */
    {"VIOLET",
     {99, 141, 177, 219, 230, 224, 217, 203},
     250,
     246}, /* violet → red (di)                      */
    {"CYBER",
     {82, 120, 158, 195, 230, 220, 215, 197},
     250,
     246}, /* green → cyan → red (tri)               */
    {"PASTEL",
     {117, 153, 195, 219, 230, 224, 218, 211},
     250,
     246}, /* soft blue → soft pink (di, all light)  */
    {"TWILIGHT",
     {99, 141, 177, 219, 230, 217, 215, 203},
     250,
     246}, /* indigo → coral (di)                    */
    {"SODIUM",
     {215, 222, 215, 230, 195, 159, 123, 117},
     250,
     246}, /* sodium amber → mercury cyan (di, rev)  */
    {"ECLIPSE",
     {247, 250, 252, 253, 203, 209, 215, 220},
     250,
     246}, /* light gray → corona gold (di)          */
    {"MONO",
     {247, 248, 250, 251, 252, 253, 254, 255},
     250,
     246}, /* bright grayscale reference             */
};

/*
 * color_apply_theme — point each colour slot at the chosen theme's colours.
 *
 * Run once at startup and again on every theme change, so the next frame
 * just repaints in the new colours with no other code involved.  Terminals
 * that can't do 256 colours fall back to the basic eight, where cycling
 * themes has no visible effect.
 */
static void color_apply_theme(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  const Theme *th = &THEMES[idx];
  if (COLORS >= 256) {
    /* Pull each element's colour from a different spot in the gradient
     * so the spin axis (Z) and momentum arrow (L) get the boldest extremes
     * and the X / Y axes get distinct cool / warm mid-tones. */
    init_pair(CP_AXIS_X, th->ramp[1], -1); /* cool mid               */
    init_pair(CP_AXIS_Y, th->ramp[6], -1); /* warm mid               */
    init_pair(CP_AXIS_Z, th->ramp[7], -1); /* warmest — spin axis    */
    init_pair(CP_MOM, th->ramp[0], -1);    /* coolest — momentum L   */
    init_pair(CP_GROUND, th->ramp[3], -1); /* cool neutral           */
    init_pair(CP_TRAIL, th->trail, -1);    /* trail colour           */
    init_pair(CP_DISC, th->ramp[4], -1);   /* warm neutral           */
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
  } else {
    /* Basic eight-colour terminals: just keep the elements telling apart */
    init_pair(CP_AXIS_X, COLOR_BLUE, -1);
    init_pair(CP_AXIS_Y, COLOR_YELLOW, -1);
    init_pair(CP_AXIS_Z, COLOR_RED, -1);
    init_pair(CP_MOM, COLOR_CYAN, -1);
    init_pair(CP_GROUND, COLOR_WHITE, -1);
    init_pair(CP_TRAIL, COLOR_MAGENTA, -1);
    init_pair(CP_DISC, COLOR_GREEN, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

static void color_init(int theme) {
  start_color();
  use_default_colors();
  color_apply_theme(theme);
}

/* ── §4  coords — flatten 3-D points onto the terminal grid ───────────── */

/*
 * Two jobs here.  First, terminal cells are about twice as tall as they are
 * wide, so circles would look like ovals; ASPECT ≈ 0.5 squashes the vertical
 * at draw time to fix that.  Second, project() turns a 3-D world point into
 * a screen cell: it spins the scene left/right (azimuth) and tilts it up/down
 * (elevation) to set the camera angle, then drops the depth onto the page.
 * It also hands back how near/far the point is, which the drawing code uses
 * to make close things bold and far things dim.
 */
#define CELL_W 8
#define CELL_H 16
#define ASPECT ((float)CELL_W / (float)CELL_H) /* ≈ 0.5 */

static void project(float wx, float wy, float wz, float phi, float theta,
                    float scale, int cx, int cy, int *col, int *row,
                    float *depth) {
  float rx = wx * cosf(phi) + wy * sinf(phi);
  float ry = -wx * sinf(phi) + wy * cosf(phi);

  float sx = rx;
  float sy = ry * cosf(theta) + wz * sinf(theta);
  *depth = -ry * sinf(theta) + wz * cosf(theta);

  *col = cx + (int)roundf(sx * scale);
  *row = cy - (int)roundf(sy * scale * ASPECT);
}

/* Direction-aware character for a 2-D segment angle */
static char dir_char(float angle) {
  float a = fmodf(angle, (float)M_PI);
  if (a < 0.0f)
    a += (float)M_PI;
  if (a < (float)M_PI / 8.0f || a >= 7.0f * (float)M_PI / 8.0f)
    return '-';
  if (a < 3.0f * (float)M_PI / 8.0f)
    return '/';
  if (a < 5.0f * (float)M_PI / 8.0f)
    return '|';
  return '\\';
}

/* Draw a 3-D line segment (from origin to endpoint) using DDA */
static void draw_seg3d(WINDOW *w, float ox3, float oy3, float oz3, float ex3,
                       float ey3, float ez3, float phi, float theta,
                       float scale, int cx, int cy, int cols, int rows,
                       chtype attr) {
  int c0, r0;
  float d0;
  int c1, r1;
  float d1;
  project(ox3, oy3, oz3, phi, theta, scale, cx, cy, &c0, &r0, &d0);
  project(ex3, ey3, ez3, phi, theta, scale, cx, cy, &c1, &r1, &d1);

  float ang = atan2f((float)(r1 - r0), (float)(c1 - c0));
  char ch = dir_char(ang);

  int dx = c1 - c0, dy = r1 - r0;
  int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);
  if (steps < 1)
    steps = 1;

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / (float)steps;
    int c = c0 + (int)roundf(dx * t);
    int r = r0 + (int)roundf(dy * t);
    if (c >= 0 && c < cols && r >= 1 && r < rows - 1) {
      wattron(w, attr);
      mvwaddch(w, r, c, (chtype)(unsigned char)ch);
      wattroff(w, attr);
    }
  }
}

/* ── §5  entity — the gyro ────────────────────────────────────────────── */

/* ── the moving state, packed for the solver ──────────────────────────── */

/*
 * State7 — the 7 numbers that describe the spinning body right now,
 * laid flat in one array so the solver can step them all the same way.
 *
 * The slots are positional — the order has to match between gyro_deriv
 * and gyro_step:
 *   v[0..2]  =  the spin rate about each body axis (rad/s)
 *   v[3..6]  =  the orientation, stored as a quaternion (kept length-1)
 *
 * Why one flat array instead of a separate spin vector and orientation:
 *   The solver does the same simple arithmetic to every number (add two
 *   states, scale a state). A flat array makes s7_add / s7_scale tiny
 *   7-element loops. The price is that two functions have to remember
 *   which slot is which — and only those two do.
 *
 * Why spin and orientation travel together in one state:
 *   They feed each other. How the spin changes depends on the spin (and
 *   any gravity tug); how the orientation changes depends on BOTH the
 *   orientation and the spin. The solver needs a single, consistent
 *   snapshot to take its trial steps, so they can't be advanced in two
 *   separate passes. References: spin rules [1], orientation rules [4].
 */
typedef struct {
  float v[7];
} State7;

static State7 s7_add(State7 a, State7 b) {
  State7 r;
  for (int i = 0; i < 7; i++)
    r.v[i] = a.v[i] + b.v[i];
  return r;
}
static State7 s7_scale(State7 a, float s) {
  State7 r;
  for (int i = 0; i < 7; i++)
    r.v[i] = a.v[i] * s;
  return r;
}

/* ── physics building blocks (used by gyro_deriv below) ───────────────── */

/*
 * gravity_torque_body_frame — how hard gravity twists a leaning top [1].
 *
 * Gravity pulls straight down on the top's centre of mass. When the top
 * leans, that pull acts off to one side and tries to twist it — the
 * sideways twist is what makes a top precess instead of just falling.
 * We work out which way "down" points in the body's own frame, then turn
 * that into the twist. Gravity can't twist the top about the very axis it
 * pulls along, so that part is always zero and only tau_x / tau_y come
 * back. Quaternion-to-direction formula from [4].
 */
static void gravity_torque_body_frame(float qw, float qx, float qy, float qz,
                                      float mgl, float *tau_x, float *tau_y) {
  float gz_bx = 2.0f * (qx * qz - qw * qy);
  float gz_by = 2.0f * (qy * qz + qw * qx);
  *tau_x = mgl * gz_by;
  *tau_y = -mgl * gz_bx;
}

/*
 * euler_equations_omega_dot — how the spin changes from moment to moment.
 *
 * These are Euler's equations [1, 2]: the rule that says how a rigid
 * body's spin speeds up or slows down about each axis, given its current
 * spin and any twist acting on it. The key part is that spinning about
 * one axis bleeds into the others when the body isn't equally hard to
 * spin all ways. That cross-talk is the whole show — it's where the
 * wobble, the precession, and the tennis-racket flip [3] come from. The
 * formulas are kept below for reference.
 *
 *   I₁·ω̇x = (I₂ − I₃)·ωy·ωz + τx
 *   I₂·ω̇y = (I₃ − I₁)·ωz·ωx + τy
 *   I₃·ω̇z = (I₁ − I₂)·ωx·ωy + τz       (τz is always 0 in this demo)
 */
static void euler_equations_omega_dot(float I1, float I2, float I3, float wx,
                                      float wy, float wz, float tau_x,
                                      float tau_y, float *dwx, float *dwy,
                                      float *dwz) {
  *dwx = ((I2 - I3) * wy * wz + tau_x) / I1;
  *dwy = ((I3 - I1) * wz * wx + tau_y) / I2;
  *dwz = ((I1 - I2) * wx * wy) / I3;
}

/*
 * quaternion_kinematics — how the orientation changes as the body spins.
 *
 * Given which way the body currently faces and how fast it's spinning,
 * this gives the rate at which its facing turns. Stepping it forward a
 * little nudges the orientation slightly off "valid" each time, so
 * project_quaternion_to_unit tidies it back up after every step. The
 * exact formulas (the standard quaternion rate from [4]) are below.
 *
 *   q̇w = ½ · (−qx·ωx − qy·ωy − qz·ωz)
 *   q̇x = ½ · ( qw·ωx + qy·ωz − qz·ωy)
 *   q̇y = ½ · ( qw·ωy − qx·ωz + qz·ωx)
 *   q̇z = ½ · ( qw·ωz + qx·ωy − qy·ωx)
 */
static void quaternion_kinematics(float qw, float qx, float qy, float qz,
                                  float wx, float wy, float wz, float *dqw,
                                  float *dqx, float *dqy, float *dqz) {
  *dqw = 0.5f * (-qx * wx - qy * wy - qz * wz);
  *dqx = 0.5f * (qw * wx + qy * wz - qz * wy);
  *dqy = 0.5f * (qw * wy - qx * wz + qz * wx);
  *dqz = 0.5f * (qw * wz + qx * wy - qy * wx);
}

/*
 * gyro_deriv — given the body's state right now, hand back how fast every
 * part of it is changing. This is the one function the solver calls over
 * and over to feel out its next step. It bundles the three rules above:
 * the gravity twist (if any), how the spin changes, and how the facing
 * changes — then packs all seven rates into one State7.
 */
static State7 gyro_deriv(State7 s, float I1, float I2, float I3, float mgl,
                         bool gravity) {
  float wx = s.v[0], wy = s.v[1], wz = s.v[2];
  float qw = s.v[3], qx = s.v[4], qy = s.v[5], qz = s.v[6];

  /* the twist from gravity (zero unless this preset has gravity on) */
  float tau_x = 0.0f, tau_y = 0.0f;
  if (gravity)
    gravity_torque_body_frame(qw, qx, qy, qz, mgl, &tau_x, &tau_y);

  /* how the spin is changing right now */
  float dwx, dwy, dwz;
  euler_equations_omega_dot(I1, I2, I3, wx, wy, wz, tau_x, tau_y, &dwx, &dwy,
                            &dwz);

  /* how the facing is changing right now */
  float dqw, dqx, dqy, dqz;
  quaternion_kinematics(qw, qx, qy, qz, wx, wy, wz, &dqw, &dqx, &dqy, &dqz);

  /* pack the seven rates back in the same slot order as the input */
  State7 d;
  d.v[0] = dwx;
  d.v[1] = dwy;
  d.v[2] = dwz;
  d.v[3] = dqw;
  d.v[4] = dqx;
  d.v[5] = dqy;
  d.v[6] = dqz;
  return d;
}

/* ── turn the orientation back into three pointing directions ─────────── */

/*
 * quat_to_axes — read the orientation and hand back the three directions
 * the body's own axes are pointing in world space: ex and ey lie in the
 * disc, ez is the spin axis. The drawing code needs these to know which
 * way to draw the body. Standard quaternion-to-axes formula.
 */
static void quat_to_axes(const float q[4], float ex[3], float ey[3],
                         float ez[3]) {
  float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
  ex[0] = 1.0f - 2.0f * (qy * qy + qz * qz);
  ex[1] = 2.0f * (qx * qy + qw * qz);
  ex[2] = 2.0f * (qx * qz - qw * qy);

  ey[0] = 2.0f * (qx * qy - qw * qz);
  ey[1] = 1.0f - 2.0f * (qx * qx + qz * qz);
  ey[2] = 2.0f * (qy * qz + qw * qx);

  ez[0] = 2.0f * (qx * qz + qw * qy);
  ez[1] = 2.0f * (qy * qz - qw * qx);
  ez[2] = 1.0f - 2.0f * (qx * qx + qy * qy);
}

/*
 * gram_schmidt — straighten the three axis directions back up [5].
 *
 * Tiny rounding errors creep in each step, so the three axes slowly stop
 * being unit-length and at right angles to each other. This nudges them
 * back to clean, square, length-1 directions so the body never looks
 * skewed or stretched. Only refresh_body_axes calls it.
 */
static void gram_schmidt(float e1[3], float e2[3], float e3[3]) {
  /* make e1 length 1 */
  float n = sqrtf(e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2]);
  if (n > 1e-9f) {
    e1[0] /= n;
    e1[1] /= n;
    e1[2] /= n;
  }

  /* lean e2 so it's square to e1, then make it length 1 */
  float d = e2[0] * e1[0] + e2[1] * e1[1] + e2[2] * e1[2];
  e2[0] -= d * e1[0];
  e2[1] -= d * e1[1];
  e2[2] -= d * e1[2];
  n = sqrtf(e2[0] * e2[0] + e2[1] * e2[1] + e2[2] * e2[2]);
  if (n > 1e-9f) {
    e2[0] /= n;
    e2[1] /= n;
    e2[2] /= n;
  }

  /* e3 is whatever direction is square to both — the cross product */
  e3[0] = e1[1] * e2[2] - e1[2] * e2[1];
  e3[1] = e1[2] * e2[0] - e1[0] * e2[2];
  e3[2] = e1[0] * e2[1] - e1[1] * e2[0];
}

/* ── trail ring-buffer ───────────────────────────────────────────────── */

/*
 * TrailPt — one dot in the fading trail the spin axis leaves behind.
 *
 * We save where the tip of the spin axis landed ON SCREEN (a cell row and
 * column), not where it was in the 3-D world. The trail is just eye candy
 * showing the path the axis has swept. Saving the screen spot directly
 * means gyro_draw can plot the whole trail with no extra math; the catch
 * is the saved spots only make sense for the current view, so we throw
 * the trail away whenever that changes: on resize, when the user turns
 * the trail off, and on a preset switch or reset (the old path belonged
 * to a different motion).
 */
typedef struct {
  int col, row;
} TrailPt;

/*
 * Gyro — everything about one spinning body, in one place.
 *
 * It holds the live motion (spin + facing), a few values worked out from
 * that motion for the drawing code, the body's shape and gravity setting,
 * the fading trail, the camera angle, and the per-body control flags. One
 * Gyro lives inside Scene (§6), which lives inside App (§8).
 *
 * Why keep it all together: the worked-out values and the trail are
 * meaningless without the body that produced them, so they sit right next
 * to it — and "reset" (r) just zeroes the whole struct in one go.
 *
 * A fun thing to watch for: spin a top fast enough under gravity and it
 * stands almost still and upright — it "sleeps". Preset 4 spins fast
 * enough to sleep; preset 5 is deliberately too slow, so it wobbles
 * wide instead. The exact spin-fast-enough threshold is in [1].
 *
 * Field groups (mirrored by the layout below):
 *
 *   Live motion      — omega, quat. The spin rate and the facing: the
 *                      seven numbers the solver steps forward each tick.
 *
 *   Worked out       — ex, ey, ez, L. The three body-axis directions and
 *                      the angular momentum arrow, recomputed each step.
 *                      Kept here so draw and the HUD can just read them
 *                      every frame instead of redoing the math.
 *
 *   Body parameters  — I1, I2, I3, gravity, mgl. The body's shape and
 *                      whether gravity acts. Loaded from the active
 *                      preset; survive a reset unless you switch preset
 *                      (n / p) or toggle gravity (g).
 *
 *   Trail            — trail[], trail_head, trail_fill, show_trail. A
 *                      fixed ring of past tip positions: trail_head walks
 *                      around overwriting the oldest once it fills up. No
 *                      allocation, no shuffling.
 *
 *   View camera      — view_phi (turn left/right), view_theta (tilt up/
 *                      down). Drifts on its own for a cinematic spin and
 *                      nudges with the arrow keys. Lives on Gyro so a
 *                      reset keeps the angle you were looking from.
 *
 *   Control flags    — preset (which demo is active), paused. Here so the
 *                      HUD and key handler reach them through one pointer.
 */
typedef struct {
  /* ── Live motion (stepped forward each tick) ──────────────────── */
  float omega[3]; /* spin rate about each body axis (rad/s)    */
  float quat[4];  /* which way the body faces, as a quaternion;
                   * kept length-1 after every step            */

  /* ── Worked out from the motion, for drawing ──────────────────── */
  float ex[3]; /* where body X points, in world space      */
  float ey[3]; /* where body Y points, in world space      */
  float ez[3]; /* where the spin axis points, in world     */
  float L[3];  /* the angular-momentum arrow, in world     */

  /* ── Body parameters (from the active preset) ─────────────────── */
  float I1, I2, I3; /* how hard to spin about each axis (kg·m²)  */

  bool gravity; /* false: free spin; true: gravity acts     */
  float mgl;    /* strength of the gravity tug (N·m)        */

  /* ── Fading trail (ring of past screen positions) ─────────────── */
  TrailPt trail[TRAIL_LEN]; /* spin-axis tip spots, oldest first        */
  int trail_head;           /* where the next spot gets written         */
  int trail_fill;           /* how many spots are live (caps at full)   */
  bool show_trail;          /* l / L turn it on and off                 */

  /* ── View camera (kept across a reset) ────────────────────────── */
  float view_phi;   /* turn left/right; drifts on its own       */
  float view_theta; /* tilt up/down; held between 0.1 and 1.4   */

  /* ── Control flags ────────────────────────────────────────────── */
  int preset;  /* which demo is active; n / p cycle        */
  bool paused; /* space toggles; frozen while true         */
} Gyro;

/* ── setup helpers ────────────────────────────────────────────────────── */

/*
 * axis_angle_to_quat — "lean this far around this direction" turned into
 * an orientation. Each preset starts the top tilted over by some angle so
 * gravity has something to act on; this builds that starting tilt.
 * Axis-angle to quaternion formula from [4].
 */
static void axis_angle_to_quat(float angle_rad, const float axis[3],
                               float q[4]) {
  float half = angle_rad * 0.5f;
  float sh = sinf(half);
  float ax = axis[0], ay = axis[1], az = axis[2];
  float n = sqrtf(ax * ax + ay * ay + az * az);
  if (n < 1e-9f)
    n = 1.0f;
  q[0] = cosf(half);
  q[1] = sh * ax / n;
  q[2] = sh * ay / n;
  q[3] = sh * az / n;
}

/*
 * refresh_body_axes — re-read the three body-axis directions from the
 * current orientation and straighten them back up. Called every time the
 * orientation changes so the directions draw and the HUD read always
 * match where the body actually faces.
 */
static void refresh_body_axes(Gyro *g) {
  quat_to_axes(g->quat, g->ex, g->ey, g->ez);
  gram_schmidt(g->ex, g->ey, g->ez);
}

/*
 * clear_polhode_trail — throw the whole trail away. Done on a preset
 * switch, reset, trail-off, and resize — in every one of those cases the
 * saved screen spots no longer mean anything.
 */
static void clear_polhode_trail(Gyro *g) {
  g->trail_head = 0;
  g->trail_fill = 0;
}

/*
 * load_body_parameters — copy the body's shape, starting spin, and
 * gravity setting from a preset into the live Gyro. Leaves the facing and
 * the trail alone — gyro_set_preset resets those separately.
 */
static void load_body_parameters(Gyro *g, const GPreset *pr) {
  g->gravity = pr->gravity;
  g->mgl = pr->mgl;
  g->I1 = pr->I1;
  g->I2 = pr->I2;
  g->I3 = pr->I3;
  g->omega[0] = pr->omega[0];
  g->omega[1] = pr->omega[1];
  g->omega[2] = pr->omega[2];
}

/*
 * gyro_set_preset — start one of the eight demos from scratch: load its
 * body and spin, tilt it to its starting lean, work out the body axes,
 * and clear the old trail (it belonged to the previous demo).
 */
static void gyro_set_preset(Gyro *g, int p) {
  const GPreset *pr = &PRESETS[p];
  g->preset = p;

  load_body_parameters(g, pr);

  float tilt_rad = pr->tilt_deg * (float)M_PI / 180.0f;
  axis_angle_to_quat(tilt_rad, pr->tilt_axis, g->quat);

  refresh_body_axes(g);
  clear_polhode_trail(g);
}

static void gyro_init(Gyro *g) {
  memset(g, 0, sizeof *g);
  g->view_phi = 0.4f;
  g->view_theta = 0.6f;
  g->show_trail = true;
  gyro_set_preset(g, 0);
}

/* ── stepping the motion forward (used by gyro_step below) ────────────── */

/* Copy the live spin + facing into the flat array the solver works on. */
static State7 pack_gyro_into_state7(const Gyro *g) {
  State7 s;
  s.v[0] = g->omega[0];
  s.v[1] = g->omega[1];
  s.v[2] = g->omega[2];
  s.v[3] = g->quat[0];
  s.v[4] = g->quat[1];
  s.v[5] = g->quat[2];
  s.v[6] = g->quat[3];
  return s;
}

/* Copy the stepped values back out of the flat array into the Gyro. */
static void unpack_state7_into_gyro(State7 s, Gyro *g) {
  g->omega[0] = s.v[0];
  g->omega[1] = s.v[1];
  g->omega[2] = s.v[2];
  g->quat[0] = s.v[3];
  g->quat[1] = s.v[4];
  g->quat[2] = s.v[5];
  g->quat[3] = s.v[6];
}

/*
 * rk4_classical_step — take one accurate step forward in time [6].
 *
 * Instead of trusting a single guess of how things are changing, it takes
 * four trial peeks across the step and blends them, which cancels out
 * most of the error. We need the accuracy because the spin equations feed
 * back on themselves and the cheap one-peek method visibly drifts within
 * seconds. The four-peek recipe (k₁..k₄, weighted ⅙·(k₁+2k₂+2k₃+k₄)) is
 * the standard one, in the code below.
 */
static State7 rk4_classical_step(State7 s, float dt, float I1, float I2,
                                 float I3, float mgl, bool gravity) {
  State7 k1 = gyro_deriv(s, I1, I2, I3, mgl, gravity);
  State7 k2 =
      gyro_deriv(s7_add(s, s7_scale(k1, dt * 0.5f)), I1, I2, I3, mgl, gravity);
  State7 k3 =
      gyro_deriv(s7_add(s, s7_scale(k2, dt * 0.5f)), I1, I2, I3, mgl, gravity);
  State7 k4 = gyro_deriv(s7_add(s, s7_scale(k3, dt)), I1, I2, I3, mgl, gravity);

  State7 sum =
      s7_add(s7_add(k1, s7_scale(k2, 2.0f)), s7_add(s7_scale(k3, 2.0f), k4));
  return s7_add(s, s7_scale(sum, dt / 6.0f));
}

/*
 * project_quaternion_to_unit — rescale the orientation back to length 1.
 * Each step nudges it slightly off being a valid rotation; dividing by
 * its own length snaps it back, which is all that's needed here [5].
 */
static void project_quaternion_to_unit(float q[4]) {
  float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n > 1e-9f) {
    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;
  }
}

/*
 * recompute_angular_momentum_world — work out the angular-momentum arrow.
 *
 * Add up each axis's share — how hard it is to spin times how fast it's
 * spinning, pointed along that axis — to get one arrow in world space.
 * With no gravity this arrow never moves, which is a nice thing to watch:
 * in preset 0 the L arrow sits dead still while the body circles around
 * it. Each axis's share is Ii·ωi along ei; the sum is below.
 */
static void recompute_angular_momentum_world(Gyro *g) {
  for (int k = 0; k < 3; k++) {
    g->L[k] = g->I1 * g->omega[0] * g->ex[k] + g->I2 * g->omega[1] * g->ey[k] +
              g->I3 * g->omega[2] * g->ez[k];
  }
}

/*
 * gyro_step — move the whole simulation forward by one small slice of
 * time. Steps the spin and facing ahead, then cleans up the rounding
 * (rescale the facing, straighten the axes) and re-derives the
 * angular-momentum arrow for the drawing code.
 */
static void gyro_step(Gyro *g, float dt) {
  State7 s = pack_gyro_into_state7(g);
  State7 ns =
      rk4_classical_step(s, dt, g->I1, g->I2, g->I3, g->mgl, g->gravity);
  unpack_state7_into_gyro(ns, g);

  project_quaternion_to_unit(g->quat);
  refresh_body_axes(g);
  recompute_angular_momentum_world(g);
}

/* ── drawing — the camera bundle and the per-layer painters ───────────── */

/*
 * Viewport — all the camera settings the drawing code needs, in one
 * bundle. Worked out once at the top of gyro_draw and handed to each
 * painter, so they don't each have to carry seven separate arguments.
 *
 *   phi, theta — which way the camera looks (turn, then tilt)
 *   scale      — how many screen cells one world unit spans
 *   cx, cy     — the centre cell everything is drawn around
 *   cols, rows — the screen size, used to clip lines at the edges
 */
typedef struct {
  float phi, theta; /* camera azimuth + elevation (radians)        */
  float scale;      /* pixels per world unit (see compute_*)       */
  int cx, cy;       /* terminal centre cell                        */
  int cols, rows;   /* viewport extent (for clipping)              */
} Viewport;

/*
 * Pick a zoom so a unit-length axis fills about a seventh of the smaller
 * screen side. Height counts double because cells are about twice as tall
 * as wide (project() squashes the vertical back later).
 */
static float compute_render_scale(int cols, int rows) {
  float sc_h = (float)cols / 7.0f;
  float sc_v = (float)(rows - 2) * 2.0f / 7.0f;
  float s = sc_h < sc_v ? sc_h : sc_v;
  return s < 1.0f ? 1.0f : s;
}

static Viewport make_viewport(const Gyro *g, int cols, int rows) {
  Viewport v;
  v.phi = g->view_phi;
  v.theta = g->view_theta;
  v.cx = cols / 2;
  v.cy = (rows + 1) / 2;
  v.cols = cols;
  v.rows = rows;
  v.scale = compute_render_scale(cols, rows);
  return v;
}

/*
 * draw_ground_ring — dotted ellipse in the world XY plane.
 * Gives the viewer a fixed horizon to read body tilt against;
 * without it the gyroscope would float in a featureless void.
 */
static void draw_ground_ring(WINDOW *w, const Viewport *v) {
  const float gr = 1.2f; /* ring radius (world units) */
  wattron(w, COLOR_PAIR(CP_GROUND) | A_DIM);
  for (int i = 0; i < GROUND_PTS; i++) {
    float t = 2.0f * (float)M_PI * i / GROUND_PTS;
    float wx = gr * cosf(t), wy = gr * sinf(t);
    int c, r;
    float d;
    project(wx, wy, 0.0f, v->phi, v->theta, v->scale, v->cx, v->cy, &c, &r, &d);
    if (c >= 0 && c < v->cols && r >= 1 && r < v->rows - 1)
      mvwaddch(w, r, c, '.');
  }
  wattroff(w, COLOR_PAIR(CP_GROUND) | A_DIM);
}

/*
 * draw_world_z_reference — a straight-up line marking "upright". With the
 * ground ring it gives the eye a true vertical, so you can see how far the
 * top's own axis is leaning away from it.
 */
static void draw_world_z_reference(WINDOW *w, const Viewport *v) {
  draw_seg3d(w, 0, 0, 0, 0, 0, 1.3f, v->phi, v->theta, v->scale, v->cx, v->cy,
             v->cols, v->rows, (chtype)(COLOR_PAIR(CP_GROUND) | A_DIM));
}

/*
 * draw_body_disc_equator — a ring of dots lying flat in the body, so the
 * body has a visible face you can watch wobble. It stands in for the
 * shape without committing to any particular one.
 */
static void draw_body_disc_equator(WINDOW *w, const Gyro *g,
                                   const Viewport *v) {
  const float dr = 0.45f;
  chtype attr = (chtype)(COLOR_PAIR(CP_DISC) | A_DIM);
  for (int i = 0; i < DISC_PTS; i++) {
    float t = 2.0f * (float)M_PI * i / DISC_PTS;
    float ct = cosf(t), st = sinf(t);
    float wx = dr * (ct * g->ex[0] + st * g->ey[0]);
    float wy = dr * (ct * g->ex[1] + st * g->ey[1]);
    float wz = dr * (ct * g->ex[2] + st * g->ey[2]);
    int c, r;
    float d;
    project(wx, wy, wz, v->phi, v->theta, v->scale, v->cx, v->cy, &c, &r, &d);
    if (c >= 0 && c < v->cols && r >= 1 && r < v->rows - 1) {
      wattron(w, attr);
      mvwaddch(w, r, c, 'o');
      wattroff(w, attr);
    }
  }
}

/*
 * draw_polhode_trail — the path the spin axis has swept, fading as it
 * ages (newest dots bold, oldest dim). The shape is the tell: clean
 * circles for a steady top, dipped scallops for a wobbling one, and wild
 * loops for the flipping ones. The dots are already in screen positions
 * (see TrailPt), so this just paints them.
 */
static void draw_polhode_trail(WINDOW *w, const Gyro *g, const Viewport *v) {
  if (!g->show_trail || g->trail_fill == 0)
    return;

  int n = g->trail_fill;
  for (int i = 0; i < n; i++) {
    int idx = (g->trail_head - n + i + TRAIL_LEN) % TRAIL_LEN;
    int c = g->trail[idx].col, r = g->trail[idx].row;
    if (c < 0 || c >= v->cols || r < 1 || r >= v->rows - 1)
      continue;

    chtype attr;
    if (i > n * 3 / 4)
      attr = (chtype)(COLOR_PAIR(CP_TRAIL) | A_BOLD);
    else if (i > n / 2)
      attr = (chtype)(COLOR_PAIR(CP_TRAIL));
    else
      attr = (chtype)(COLOR_PAIR(CP_TRAIL) | A_DIM);
    wattron(w, attr);
    mvwaddch(w, r, c, '.');
    wattroff(w, attr);
  }
}

/*
 * draw_body_axes_depth_sorted — the body's three axes, labelled at the
 * tips. We draw the farthest one first so the nearer ones paint over it,
 * which is what makes them look layered in 3-D. Nearer axes are drawn
 * bold so they read as closer. The label sits a little past the tip so it
 * doesn't land on top of the line.
 */
static void draw_body_axes_depth_sorted(WINDOW *w, const Gyro *g,
                                        const Viewport *v) {
  struct {
    const float *axis;
    const char *label;
    int pair;
  } axes[3] = {
      {g->ex, "X", CP_AXIS_X},
      {g->ey, "Y", CP_AXIS_Y},
      {g->ez, "Z", CP_AXIS_Z},
  };

  /* how near/far each axis tip is from the camera */
  float depths[3];
  for (int i = 0; i < 3; i++) {
    int c, r;
    float d;
    project(axes[i].axis[0], axes[i].axis[1], axes[i].axis[2], v->phi, v->theta,
            v->scale, v->cx, v->cy, &c, &r, &d);
    depths[i] = d;
  }

  /* order them farthest-first so nearer axes paint last */
  int order[3] = {0, 1, 2};
  for (int i = 0; i < 2; i++)
    for (int j = i + 1; j < 3; j++)
      if (depths[order[i]] < depths[order[j]]) {
        int t = order[i];
        order[i] = order[j];
        order[j] = t;
      }

  for (int oi = 0; oi < 3; oi++) {
    int i = order[oi];
    const float *ax = axes[i].axis;
    chtype attr =
        (chtype)(COLOR_PAIR(axes[i].pair) | (depths[i] < 0.0f ? A_BOLD : 0u));

    draw_seg3d(w, 0, 0, 0, ax[0], ax[1], ax[2], v->phi, v->theta, v->scale,
               v->cx, v->cy, v->cols, v->rows, attr);

    /* drop the label just past the tip so it clears the line */
    int c, r;
    float d;
    project(ax[0] * 1.15f, ax[1] * 1.15f, ax[2] * 1.15f, v->phi, v->theta,
            v->scale, v->cx, v->cy, &c, &r, &d);
    if (c >= 0 && c < v->cols && r >= 1 && r < v->rows - 1) {
      wattron(w, attr);
      mvwaddch(w, r, c, (chtype)(unsigned char)axes[i].label[0]);
      wattroff(w, attr);
    }
  }
}

/*
 * draw_angular_momentum_vector — the L arrow, shrunk to a tidy length.
 *
 * With no gravity this arrow is pinned in space and the body circles
 * around it — the clearest way to see precession. Under gravity the arrow
 * itself swings around the upright direction. Labelled 'L' just past the
 * tip, like the body axes.
 */
static void draw_angular_momentum_vector(WINDOW *w, const Gyro *g,
                                         const Viewport *v) {
  float Lmag = sqrtf(g->L[0] * g->L[0] + g->L[1] * g->L[1] + g->L[2] * g->L[2]);
  if (Lmag < 1e-9f)
    return;

  float Ldx = g->L[0] / Lmag, Ldy = g->L[1] / Lmag, Ldz = g->L[2] / Lmag;
  draw_seg3d(w, 0, 0, 0, Ldx, Ldy, Ldz, v->phi, v->theta, v->scale, v->cx,
             v->cy, v->cols, v->rows, (chtype)(COLOR_PAIR(CP_MOM) | A_BOLD));

  int c, r;
  float d;
  project(Ldx * 1.15f, Ldy * 1.15f, Ldz * 1.15f, v->phi, v->theta, v->scale,
          v->cx, v->cy, &c, &r, &d);
  if (c >= 0 && c < v->cols && r >= 1 && r < v->rows - 1) {
    wattron(w, COLOR_PAIR(CP_MOM) | A_BOLD);
    mvwaddch(w, r, c, 'L');
    wattroff(w, COLOR_PAIR(CP_MOM) | A_BOLD);
  }
}

/*
 * gyro_draw — paint the whole scene back-to-front, so nearer parts cover
 * farther ones: ground ring, the upright marker, the body's disc, the
 * trail, the body axes, and finally the L arrow on top. Closer things are
 * drawn bold throughout, which reads as depth.
 */
static void gyro_draw(const Gyro *g, WINDOW *w, int cols, int rows) {
  Viewport vp = make_viewport(g, cols, rows);

  draw_ground_ring(w, &vp);
  draw_world_z_reference(w, &vp);
  draw_body_disc_equator(w, g, &vp);
  draw_polhode_trail(w, g, &vp);
  draw_body_axes_depth_sorted(w, g, &vp);
  draw_angular_momentum_vector(w, g, &vp);
}

/* ── trail update ────────────────────────────────────────────────────── */

/* Save where the spin-axis tip lands on screen this tick, for the trail. */
static void gyro_update_trail(Gyro *g, int cx, int cy, float scale, int cols,
                              int rows) {
  int c, r;
  float d;
  project(g->ez[0], g->ez[1], g->ez[2], g->view_phi, g->view_theta, scale, cx,
          cy, &c, &r, &d);
  g->trail[g->trail_head] = (TrailPt){c, r};
  g->trail_head = (g->trail_head + 1) % TRAIL_LEN;
  if (g->trail_fill < TRAIL_LEN)
    g->trail_fill++;

  (void)cols;
  (void)rows;
}

/* ── §6  scene ────────────────────────────────────────────────────────── */

/*
 * Scene — the gyro simulation plus the one cosmetic choice (the theme).
 *
 * The two parts are split on purpose, for whoever reads this next: one
 * group is the actual physics, the other is purely how it looks. When you
 * add a field, ask "does this change how the body moves?" If yes it's
 * physics; if it only changes colours it's rendering. Putting a physics
 * knob in the rendering group by mistake would quietly tie the look to the
 * motion — which is exactly the bug this split guards against. Switching
 * themes while paused must leave the motion untouched; only the colours
 * swap.
 *
 * The tick rate, the quit/resize flags, and the terminal size live on App
 * (§8) instead, since the main loop owns the clock and the window. One
 * Scene lives inside the single file-scope App (§8); the trail alone is a
 * couple of kilobytes, so nothing here is ever copied by value.
 */
typedef struct {
  /* ── the physics (read & written by scene_tick) ───────────────── */
  Gyro gyro; /* the spinning body — see Gyro in §5            */

  /* ── how it looks (no effect on the physics) ──────────────────── */
  int theme; /* which palette; t / T cycle. Cosmetic only —
              * the motion is identical across all 12        */
} Scene;

static void scene_init(Scene *s) {
  memset(s, 0, sizeof *s);
  /* memset → theme = 0 (VOLT); keep current theme on r-reset */
  gyro_init(&s->gyro);
}

/*
 * scene_tick — move everything forward one tick. It splits the tick into
 * several smaller physics steps so even the fastest wobble is captured,
 * drifts the camera a touch for a cinematic feel, and adds one dot to the
 * trail.
 */
static void scene_tick(Scene *s, float dt, int cols, int rows) {
  Gyro *g = &s->gyro;
  if (g->paused)
    return;

  float sub_dt = dt / (float)SUB_STEPS;
  for (int i = 0; i < SUB_STEPS; i++)
    gyro_step(g, sub_dt);

  /* let the camera drift slowly on its own */
  g->view_phi += 0.15f * dt;

  /* add one trail dot per tick (not per sub-step) */
  if (g->show_trail) {
    float sc_h = (float)cols / 7.0f;
    float sc_v = (float)(rows - 2) * 2.0f / 7.0f;
    float scale = sc_h < sc_v ? sc_h : sc_v;
    gyro_update_trail(g, cols / 2, (rows + 1) / 2, scale, cols, rows);
  }
}

/*
 * scene_draw — draw the gyroscope. The alpha argument is ignored: this
 * demo draws the body right where the physics put it, with no smoothing
 * between frames, but the argument stays so the signature matches the
 * shared framework.
 */
static void scene_draw(Scene *s, WINDOW *w, int cols, int rows, float alpha,
                       float dt_sec) {
  (void)alpha;
  (void)dt_sec;
  gyro_draw(&s->gyro, w, cols, rows);
}

/* ── §7  screen ───────────────────────────────────────────────────────── */

/*
 * Screen — the terminal's current size, re-read whenever the window
 * resizes. Kept apart from Scene so resizing never touches the physics.
 * The HUD reads it to place text against the edges, and the drawing code
 * reads it to clip lines that run off-screen.
 */
typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *s, int theme) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init(theme);
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

/*
 * screen_draw — paint the frame and the two info bars: top-right shows
 * speed and pause state, the line under it shows which demo and its
 * settings, and the bottom row lists the keys.
 */
static void screen_draw(Screen *s, Scene *sc, double fps, int sim_fps,
                        float alpha, float dt_sec) {
  erase();
  scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

  const Gyro *g = &sc->gyro;
  const GPreset *pr = &PRESETS[g->preset];

  float omag = sqrtf(g->omega[0] * g->omega[0] + g->omega[1] * g->omega[1] +
                     g->omega[2] * g->omega[2]);

  /* ── Row 0 right: canonical fps / sim / paused ── */
  char top[80];
  snprintf(top, sizeof top, " %5.1f fps  sim:%3d Hz  %s ", fps, sim_fps,
           g->paused ? "PAUSED " : "running");
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, s->cols - (int)strlen(top), "%s", top);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* ── Row 1 left: parameter readouts ── */
  char params[200];
  snprintf(params, sizeof params,
           " preset:%d %s  I=(%.1f,%.1f,%.1f)  |ω|=%.1f rad/s"
           "  g:%s  theme:[%d] %s ",
           g->preset, pr->name, g->I1, g->I2, g->I3, omag,
           g->gravity ? "ON " : "OFF", sc->theme, THEMES[sc->theme].name);
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(1, 0, "%.*s", s->cols, params);
  attroff(COLOR_PAIR(PAIR_HUD));

  /* ── Bottom row: key legend ── */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q:quit  spc:pause  n/p:preset  r:restart  g:gravity"
           "  l:trail  t/T:theme  arrows:view  ]/[:simHz ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §8  app ──────────────────────────────────────────────────────────── */

/*
 * App — one box holding everything the main loop touches. There's a
 * single one, at file scope, because the signal handlers have to reach
 * the quit and resize flags and a signal handler can't be handed a
 * pointer. Bundling it all together also lets the main loop pass one
 * pointer around instead of juggling several loose globals.
 *
 * Field groups (mirrored by the layout below):
 *
 *   Owned parts        — scene, screen. The big state, passed by pointer.
 *   Tick-rate knob     — sim_fps. Lives here, not on Scene, because the
 *                        main loop owns the clock; the simulation itself
 *                        doesn't care how fast it's ticked.
 *   Signal flags       — running, need_resize. The only things the signal
 *                        handlers write. They're the special volatile
 *                        kind so it's safe to share them between a handler
 *                        and the main loop.
 */
typedef struct {
  /* ── owned parts ──────────────────────────────────────────────── */
  Scene scene;   /* the simulation + its look (§6)  */
  Screen screen; /* the terminal size (§7)          */

  /* ── tick-rate knob ───────────────────────────────────────────── */
  int sim_fps; /* how many physics ticks per second;
                * ] / [ adjust it within limits  */

  /* ── signal flags (touched by handlers only) ──────────────────── */
  volatile sig_atomic_t running;     /* goes 0 to quit the main loop    */
  volatile sig_atomic_t need_resize; /* set to 1 when the window resizes */
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
  app->scene.gyro.trail_head = 0;
  app->scene.gyro.trail_fill = 0;
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *sc = &app->scene;
  Gyro *g = &sc->gyro;

  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    g->paused = !g->paused;
    break;

  case 'n':
    gyro_set_preset(g, (g->preset + 1) % N_PRESETS);
    break;

  case 'p':
    gyro_set_preset(g, (g->preset + N_PRESETS - 1) % N_PRESETS);
    break;

  case 'r':
  case 'R':
    gyro_set_preset(g, g->preset);
    break;

  case 'g':
  case 'G':
    g->gravity = !g->gravity;
    break;

  case 'l':
  case 'L':
    g->show_trail = !g->show_trail;
    if (!g->show_trail) {
      g->trail_head = 0;
      g->trail_fill = 0;
    }
    break;

  case 't':
    sc->theme = (sc->theme + 1) % N_THEMES;
    color_apply_theme(sc->theme);
    break;
  case 'T':
    sc->theme = (sc->theme + N_THEMES - 1) % N_THEMES;
    color_apply_theme(sc->theme);
    break;

  case KEY_LEFT:
    g->view_phi -= 0.1f;
    break;
  case KEY_RIGHT:
    g->view_phi += 0.1f;
    break;
  case KEY_UP:
    g->view_theta += 0.05f;
    if (g->view_theta > 1.4f)
      g->view_theta = 1.4f;
    break;
  case KEY_DOWN:
    g->view_theta -= 0.05f;
    if (g->view_theta < 0.1f)
      g->view_theta = 0.1f;
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

/* ── main-loop pieces ─────────────────────────────────────────────────── */

/*
 * install_signal_handlers — wire up clean shutdown. Ctrl-C / kill ask the
 * loop to quit, a window resize asks it to re-measure, and an exit handler
 * restores the terminal. The handlers only flip a flag, which is all
 * that's safe to do from inside a signal.
 */
static void install_signal_handlers(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);
}

/*
 * handle_resize_pending — if the window just resized, re-measure it, drop
 * the now-meaningless trail, and reset the frame clock so the next frame's
 * time gap isn't a huge jump from before the resize.
 */
static void handle_resize_pending(App *app, int64_t *frame_time,
                                  int64_t *sim_accum) {
  if (!app->need_resize)
    return;
  app_do_resize(app);
  *frame_time = clock_ns();
  *sim_accum = 0;
}

/*
 * step_simulation_fixed_dt — keep the physics on its own steady clock,
 * separate from the drawing. It banks the real time that passed and spends
 * it in fixed-size ticks, so each physics step is always the same width no
 * matter how fast or slow the screen is updating. Any leftover time under
 * one tick is carried into the next frame.
 */
static void step_simulation_fixed_dt(Scene *scene, int64_t *sim_accum,
                                     int64_t dt_ns, int64_t tick_ns,
                                     float dt_sec, int cols, int rows) {
  *sim_accum += dt_ns;
  while (*sim_accum >= tick_ns) {
    scene_tick(scene, dt_sec, cols, rows);
    *sim_accum -= tick_ns;
  }
}

/*
 * update_fps_counter — work out the frames-per-second figure as an
 * average over the last half-second. Averaging keeps the number steady
 * enough to read; a raw per-frame figure would flicker too fast to see.
 */
static void update_fps_counter(int64_t dt_ns, int64_t *fps_accum,
                               int *frame_count, double *fps_display) {
  (*frame_count)++;
  *fps_accum += dt_ns;
  if (*fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
    *fps_display =
        (double)(*frame_count) / ((double)(*fps_accum) / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum = 0;
  }
}

/*
 * cap_to_render_framerate — wait a moment before drawing so each frame
 * takes about a sixtieth of a second, however quickly the physics and
 * drawing finished. This is separate from the physics tick rate: the
 * physics runs at its own pace, while the screen always refreshes at a
 * steady 60 frames a second so the motion looks smooth on any terminal.
 */
static void cap_to_render_framerate(int64_t frame_start_ns,
                                    int64_t prev_frame_dt_ns) {
  int64_t elapsed = clock_ns() - frame_start_ns + prev_frame_dt_ns;
  clock_sleep_ns(NS_PER_SEC / 60 - elapsed);
}

/*
 * main — the demo's top-level loop.
 *
 * Pseudocode:
 *   srand + install signal handlers
 *   initialise scene (memset → VOLT theme, gyro_set_preset(0))
 *   initialise screen (ncurses + colour pairs from current theme)
 *
 *   loop until app->running goes false:
 *     handle pending resize (if SIGWINCH fired)
 *     measure wall-clock dt since last frame
 *     step_simulation_fixed_dt  // run scene_tick N times to drain accum
 *     update_fps_counter        // every FPS_UPDATE_MS ms
 *     cap_to_render_framerate   // hold ~60 fps render
 *     screen_draw + screen_present
 *     poll input → app_handle_key (returns false ⇒ quit)
 */
int main(void) {
  srand((unsigned int)(clock_ns() & 0xFFFFFFFFu));
  install_signal_handlers();

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  scene_init(&app->scene);                     /* theme = 0 (VOLT) */
  screen_init(&app->screen, app->scene.theme); /* color_init wants theme */

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {
    handle_resize_pending(app, &frame_time, &sim_accum);

    /* wall-clock dt since previous frame (clamped to 100 ms to
     * prevent spiral-of-death after a hiccup or breakpoint pause) */
    int64_t now = clock_ns();
    int64_t dt_ns = now - frame_time;
    frame_time = now;
    if (dt_ns > 100 * NS_PER_MS)
      dt_ns = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    step_simulation_fixed_dt(&app->scene, &sim_accum, dt_ns, tick_ns, dt_sec,
                             app->screen.cols, app->screen.rows);

    /* how far we are between two physics ticks (0 to just under 1).
     * Other demos use it to smooth motion between ticks; this one draws
     * the body right where the physics put it, so it's unused here and
     * only kept so the function signatures match the shared framework. */
    float alpha = (float)sim_accum / (float)tick_ns;

    update_fps_counter(dt_ns, &fps_accum, &frame_count, &fps_display);
    cap_to_render_framerate(now, dt_ns);

    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps, alpha,
                dt_sec);
    screen_present();

    int key = getch();
    if (key != ERR && !app_handle_key(app, key))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
