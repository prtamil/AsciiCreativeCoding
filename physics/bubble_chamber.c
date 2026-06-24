/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bubble_chamber.c — charged particles curling in a magnetic field.
 *
 * A bubble chamber lets you "see" subatomic particles: a charged particle
 * flying through a magnetic field curves, and leaves a fading trail behind
 * it. Lighter particles curl tightly, heavier ones barely bend, and every
 * particle slowly spirals inward as it loses speed.
 *
 * The physics ideas (Lorentz turning, drag, cyclotron radius) and where they
 * come from are explained in the CONCEPTS block below; the per-particle
 * settings live in §3 (k_types).
 */

/* ── CONCEPTS ── *
 *
 * Turning the velocity:
 *   A magnetic field doesn't speed a particle up or slow it down — it just
 *   bends its path, the way a string bends a ball you swing around. Each
 *   step we turn the velocity arrow by a small angle (faster for light,
 *   strongly-charged particles). We do the turn with an exact rotation
 *   rather than nudging it with a force, so the circle stays a perfect
 *   circle instead of slowly drifting bigger or smaller over time [4].
 *
 * Drag makes it spiral in:
 *   As a particle plows through the chamber's liquid it loses a little speed
 *   every step. Slower speed means a tighter circle, so the path spirals
 *   inward — the signature look of a real bubble-chamber photo [2][3].
 *
 * Why these particles curl differently:
 *   How tightly something curls depends on its charge-to-mass ratio. Light
 *   electrons curl tightly; heavy protons almost go straight. We use tuned
 *   ratios (not the real physical ones) so every track is visible at terminal
 *   scale — details and the radius table are in §3 (k_types) [1][2].
 *
 * The fading trail:
 *   Each particle remembers its recent positions and draws them as a trail
 *   that fades from a bright head to faint dots, so you can read which way it
 *   was going. This mimics the bubbles a real particle leaves behind [3][5].
 *
 * References:
 *
 *   [1] Griffiths, D. J. — Introduction to Electrodynamics, 4th ed.,
 *       Cambridge Univ. Press (2017).  Lorentz force and cyclotron motion;
 *       the radius formula r = mv / (qB) used to pick the qm values in §3.
 *
 *   [2] Griffiths, D. J. — Introduction to Elementary Particles, 2nd ed.,
 *       Wiley-VCH (2008).  The particle families and energy-loss-by-drag
 *       (Bethe-Bloch) idea behind §3's species and qm values.
 *
 *   [3] Glaser, D. A. (1952) — "Some Effects of Ionizing Radiation on the
 *       Formation of Bubbles in Liquids", Phys. Rev. 87, 665.  The original
 *       bubble-chamber paper: charged particles leave visible trails of
 *       bubbles — what the trail ring buffer imitates.
 *
 *   [4] Birdsall, C. K. & Langdon, A. B. — Plasma Physics via Computer
 *       Simulation, IOP / CRC Press (2004).  Why the naive force-nudge drifts
 *       and the exact-rotation scheme (used in particle_step) keeps energy.
 *
 *   [5] Ware, C. — Information Visualization: Perception for Design, 4th ed.,
 *       Morgan Kaufmann (2020).  How to order colours and brightness so the
 *       eye reads them easily — behind the themes and the fading trail.
 * ── */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ── */

#define MAX_PARTICLES 20
#define TRAIL_LEN 300 /* how many past positions each trail remembers */
#define N_TYPES 5
#define N_THEMES 10

/* One status line at the top, one key-hint line at the bottom; particles
 * draw in the rows between. */
#define HUD_TOP 1
#define HUD_BOT 1

/* magnetic field — higher strength means tighter curls */
#define B_INIT 1.0f
#define B_MIN 0.1f
#define B_MAX 4.0f
#define B_STEP 0.1f

/* particle motion */
#define V_SPAWN 2.2f /* starting speed; gives an electron a curl roughly 11 \
                      * cells wide at the default field strength             */
#define V_SPREAD 0.4f /* spread the starting speeds ±20% so radii vary       */
#define DRAG 0.003f   /* speed lost per step; ~330 steps to halve, so trails \
                       * run about a thousand cells long before fading out   */
#define SPEED_DEAD 0.22f /* once this slow the curl is under one cell wide — \
                          * retire the particle so it doesn't spin in place  */

/* how many particles a single burst makes */
#define BURST_MIN 2
#define BURST_MAX 5

/* timing — four physics steps per drawn frame, redrawn 30 times a second */
#define STEPS_PER_FRAME 4
#define RENDER_NS (1000000000LL / 30)

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

/* ── §3 species / themes / color ── */

/*
 * PType — one kind of particle (electron, positron, muon, pion, proton).
 *
 * Each particle type carries three things bundled together so the rest of
 * the code can say k_types[i].qm and k_types[i].symbol instead of juggling
 * three separate look-up arrays:
 *   name    full word shown nowhere on screen, handy for reference
 *   symbol  the 2-character tag shown in the HUD ("e-", "mu", "p ")
 *   qm      charge-to-mass ratio — the one number that decides how tightly
 *           this type curls. Bigger size = tighter curl. Its sign decides
 *           which way it turns: negative curls clockwise, positive the other
 *           way (with the default field). Flipping the field reverses both.
 *
 * The qm values are tuned for the screen, not real physics. Real ratios would
 * make electrons curl in a speck and protons go almost straight — nothing you
 * could see. These values keep the same ordering (electrons tightest, protons
 * loosest) but spread the curls out to readable sizes:
 *
 *      type      qm        curl radius      look
 *      e-       -0.200     ~11 cells         tight
 *      e+       +0.200     ~11 cells         tight, turns the other way
 *      mu       -0.070     ~31 cells         medium arc
 *      pi       +0.045     ~49 cells         wide arc
 *      p        +0.022     ~100 cells        barely bends
 *
 * Colour isn't stored here on purpose — it comes from the current theme
 * (g_themes below), so the same particles can be recoloured without touching
 * their physics. See [1][2] for the real Lorentz/cyclotron and particle-family
 * ideas these values are loosely based on.
 */
typedef struct {
  const char *name;   /* full name, e.g. "electron"              */
  const char *symbol; /* 2-char HUD tag, e.g. "e-", "mu", "p "   */
  float qm;           /* charge/mass ratio — sets curl tightness and direction */
} PType;

static const PType k_types[N_TYPES] = {
    {"electron", "e-", -0.200f}, {"positron", "e+", +0.200f},
    {"muon", "mu", -0.070f},     {"pion", "pi", +0.045f},
    {"proton", "p ", +0.022f},
};

/*
 * Theme — one named colour palette: five colours, one per particle type
 * in the same order as k_types (electron, positron, muon, pion, proton).
 *   name  what shows in the HUD ("Matrix", "Fire", ...)
 *   fg    the five colours, as 256-colour codes
 * Every colour is kept bright enough to stay visible on a dark terminal
 * (project rule), and the five within a theme stay distinct so you can still
 * tell the track types apart.
 */
typedef struct {
  const char *name;
  short fg[N_TYPES];
} Theme;

static const Theme g_themes[N_THEMES] = {
    /*  name       e-   e+   mu   pi   p    */
    {"Matrix", {46, 82, 118, 154, 226}},   /* cyber green ramp + yellow */
    {"Fire", {196, 226, 220, 208, 130}},   /* white-hot to ember        */
    {"Oceanic", {51, 87, 159, 195, 117}},  /* cyans / light blues       */
    {"Neon", {196, 51, 201, 226, 46}},     /* high-sat neon mix         */
    {"Mono", {255, 252, 248, 244, 240}},   /* grayscale ramp            */
    {"Ice", {195, 159, 153, 117, 87}},     /* light blues to cyan       */
    {"Nova", {231, 213, 177, 141, 105}},   /* stellar white→violet      */
    {"Forest", {190, 154, 100, 130, 64}},  /* leaves to bark            */
    {"Desert", {226, 220, 214, 178, 130}}, /* sand / gold / brown       */
    {"Eclipse", {196, 244, 240, 124, 88}}, /* corona red + grays        */
};

/* Fixed colours for the few terminals that only have 8 — themes don't apply. */
static const short g_8fg[N_TYPES] = {COLOR_BLUE, COLOR_RED, COLOR_GREEN,
                                     COLOR_YELLOW, COLOR_CYAN};

/* Colour slots: 1..N_TYPES are the particle types, then the two HUD bars. */
#define CP_HUD (N_TYPES + 1)
#define CP_HINT (N_TYPES + 2)

static bool g_256color;

static void color_init(int theme_idx) {
  start_color();
  use_default_colors();
  g_256color = (COLORS >= 256);

  const Theme *t = &g_themes[theme_idx % N_THEMES];
  for (int i = 0; i < N_TYPES; i++) {
    short fg = g_256color ? t->fg[i] : g_8fg[i];
    init_pair(1 + i, fg, -1);
  }
  if (g_256color) {
    init_pair(CP_HUD, 226, -1);
    init_pair(CP_HINT, 51, -1);
  } else {
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
}

static inline int particle_cp(int kind) { return 1 + kind; }

/* ── §4 physics ── */

/*
 * Particle — one moving track in the chamber, with the fading trail behind it.
 *
 * Position and velocity (x, y, vx, vy) are kept as floats, in cell units. We
 * need the in-between fractions because a curl spanning 10–100 cells would look
 * jagged if we tracked whole cells only; we round to a whole cell just before
 * drawing.
 *
 * kind picks which row of k_types this particle is (and so how it curls).
 * alive marks whether the particle is still moving; when it stops, the slot
 * can be handed to a new particle (see find_dead_slot), but the trail is left
 * untouched so the old track lingers until that slot is actually reused.
 *
 * The trail is a fixed-size circular log of recent positions (tx/ty). Rather
 * than re-computing the whole past path every frame, we just remember the last
 * TRAIL_LEN spots and draw them. "Circular" means when it fills up, the newest
 * write lands on top of the oldest spot, so the trail always shows the most
 * recent stretch and naturally forgets the rest — which is exactly the
 * fade-out we want.
 *   thead  where the NEXT position will be written
 *   tlen   how many spots are filled so far (stops growing at TRAIL_LEN)
 * §6 walks this newest-first to fade the trail; see [3][4] for the physics.
 */
typedef struct {
  /* where it is and how it's moving (updated every step) */
  float x, y;   /* position, in cells */
  float vx, vy; /* velocity, in cells per step */

  /* which kind it is, and whether it's still going */
  int kind;   /* row in k_types (§3) */
  bool alive; /* false = this slot can be reused */

  /* the fading trail: a circular log read newest-first by §6 */
  float tx[TRAIL_LEN]; /* past x positions, oldest gets overwritten */
  float ty[TRAIL_LEN]; /* past y positions */
  int thead;           /* next write slot */
  int tlen;            /* how many slots are filled (caps at TRAIL_LEN) */
} Particle;

/*
 * Scene — all the world state in one place: the particles plus the few knobs
 * the keys turn. There's a single copy of it (g_scene) because the particle
 * array is large (over a megabyte), so it lives as one file-wide variable
 * instead of being copied around.
 *
 * The fields are split into two groups on purpose, to keep "what the particles
 * do" separate from "how they look":
 *   simulation  things that affect the physics — the particles, the field,
 *               whether we're paused, and what new bursts spawn as.
 *   rendering   things that only affect appearance — currently just the theme.
 *               Changing these never moves a particle.
 * The rule of thumb when adding a field: if it changes where particles go, it
 * belongs in the simulation group; if it only changes colour, the rendering
 * group.
 *
 * A few globals deliberately live outside this struct: the quit/resize flags
 * (they're touched by signal handlers and must stay simple and separate), the
 * one-time colour-support flag, and the screen size (which the main loop owns
 * so the scene needn't care about resizing).
 */
typedef struct {
  /* simulation — affects the physics */
  Particle particles[MAX_PARTICLES];
  float B;        /* field strength, with a sign. Space flips the sign (curls
                   * reverse). Magnitude stays within [B_MIN, B_MAX]; b/B step it. */
  bool paused;    /* true freezes the simulation (p key) */
  int spawn_kind; /* what new bursts are: -1 = random, otherwise a fixed type
                   * (0..N_TYPES-1). The k/K keys cycle it. */

  /* rendering — affects only appearance */
  int theme; /* which palette in g_themes (§3); t/T cycle it. Cosmetic only. */
} Scene;

static Scene g_scene = {
    .B = B_INIT, .paused = false, .spawn_kind = -1, .theme = 0,
    /* particles[] starts all-zero; respawn_centre_burst fills it. */
};

/* Current terminal size, owned by the main loop (not part of the scene). */
static int g_rows, g_cols;

/*
 * The work of one physics step is split into small helpers, run in order:
 * turn the velocity, slow it a touch, move, log the position, then check if
 * it's too slow to bother with. particle_step just calls them in sequence.
 */

/*
 * Turn the velocity arrow by a fixed angle, keeping its length exactly the
 * same. Doing the turn as a true rotation (instead of nudging it with a force)
 * is what keeps the circle from slowly growing or shrinking over many steps —
 * a magnetic field bends a path but never changes its speed [4].
 */
static void rotate_velocity_exact(Particle *p, float omega) {
  float ca = cosf(omega), sa = sinf(omega);
  float nvx = p->vx * ca - p->vy * sa;
  float nvy = p->vx * sa + p->vy * ca;
  p->vx = nvx;
  p->vy = nvy;
}

/*
 * Shave a tiny bit off the speed each step, as if the particle were rubbing
 * through the chamber's liquid. Slower speed makes a tighter curl, so over
 * time the path spirals inward — the look of a real bubble-chamber track [2].
 */
static void apply_ionisation_drag(Particle *p) {
  p->vx *= (1.f - DRAG);
  p->vy *= (1.f - DRAG);
}

static void advance_position(Particle *p) {
  p->x += p->vx;
  p->y += p->vy;
}

/* Append the current spot to the trail; once full it overwrites the oldest. */
static void record_trail_sample(Particle *p) {
  p->tx[p->thead] = p->x;
  p->ty[p->thead] = p->y;
  p->thead = (p->thead + 1) % TRAIL_LEN;
  if (p->tlen < TRAIL_LEN)
    p->tlen++;
}

/*
 * True once the particle is so slow its curl would fit inside one character
 * cell — at that point it'd just spin in place, so we retire it and let its
 * slot be reused.
 */
static int is_subgyro_dead(const Particle *p) {
  float spd2 = p->vx * p->vx + p->vy * p->vy;
  return spd2 < SPEED_DEAD * SPEED_DEAD;
}

/* One full step for a particle: turn, slow, move, log, then maybe retire. */
static void particle_step(Particle *p) {
  if (!p->alive)
    return;

  /* turn angle this step: bigger for strongly-charged types and stronger fields */
  float omega = k_types[p->kind].qm * g_scene.B;
  rotate_velocity_exact(p, omega);
  apply_ionisation_drag(p);
  advance_position(p);
  record_trail_sample(p);
  if (is_subgyro_dead(p))
    p->alive = false;
}

/* ── §5 scene ── */

static void scene_reset(void) {
  memset(g_scene.particles, 0, sizeof g_scene.particles);
}

/*
 * Set up one particle slot: place it at (cx, cy), launch it in direction
 * `angle`, as type `kind` (pass -1 to pick a random type). Speed is randomised
 * a little so a burst fans out into different radii.
 */
static void init_particle(Particle *p, float cx, float cy, float angle,
                          int kind) {
  memset(p, 0, sizeof *p);
  p->x = cx;
  p->y = cy;
  p->kind = (kind < 0) ? rand() % N_TYPES : kind;

  float speed = V_SPAWN * (1.f - V_SPREAD / 2.f +
                           V_SPREAD * ((float)rand() / (float)RAND_MAX));
  p->vx = cosf(angle) * speed;
  p->vy = sinf(angle) * speed;
  p->alive = true;
}

/* Find a free particle slot to reuse, or -1 if all are in use. */
static int find_dead_slot(void) {
  for (int i = 0; i < MAX_PARTICLES; i++)
    if (!g_scene.particles[i].alive)
      return i;
  return -1;
}

/* Spray n particles outward from the middle of the screen, like a collision
 * happening right in the centre. */
static void spawn_burst_centre(int n) {
  float cx = (float)g_cols * 0.5f;
  float cy = (float)(g_rows - HUD_TOP - HUD_BOT) * 0.5f;
  int k = (g_scene.spawn_kind < 0) ? -1 : g_scene.spawn_kind;

  for (int i = 0; i < n; i++) {
    int slot = find_dead_slot();
    if (slot < 0)
      break;
    float angle = ((float)rand() / (float)RAND_MAX) * 2.f * 3.14159265f;
    init_particle(&g_scene.particles[slot], cx, cy, angle, k);
  }
}

/*
 * Pick a random spot along one of the four screen edges, and the direction a
 * particle should head to travel inward from there (down from the top, up from
 * the bottom, and so on). H is the height of the particle area, not counting
 * the HUD rows. Used by spawn_burst_edge to make particles enter from a side.
 */
static void edge_entry_geometry(int edge, float W, float H, float *cx,
                                float *cy, float *base_angle) {
  float u = (float)rand() / (float)RAND_MAX;
  switch (edge) {
  case 0:
    *cx = W * u;
    *cy = 0;
    *base_angle = 0.5f * 3.14159265f;
    break;
  case 1:
    *cx = W * u;
    *cy = H - 1;
    *base_angle = -0.5f * 3.14159265f;
    break;
  case 2:
    *cx = 0;
    *cy = H * u;
    *base_angle = 0.0f;
    break;
  default:
    *cx = W - 1;
    *cy = H * u;
    *base_angle = 3.14159265f;
    break;
  }
}

/*
 * Nudge the inward direction by a small random amount, so a burst fans out a
 * little instead of every particle flying in along the exact same line.
 */
static float inward_velocity_angle(float base_angle, float half_spread) {
  float u = (float)rand() / (float)RAND_MAX;
  return base_angle + (u - 0.5f) * 2.0f * half_spread;
}

/*
 * Send n particles in from a random screen edge, each aimed inward with a
 * little spread — like a beam crashing into the chamber from outside.
 */
static void spawn_burst_edge(int n) {
  int edge = rand() % 4; /* 0=top 1=bottom 2=left 3=right */
  float W = (float)g_cols;
  float H = (float)(g_rows - HUD_TOP - HUD_BOT);

  float cx, cy, base_angle;
  edge_entry_geometry(edge, W, H, &cx, &cy, &base_angle);

  int k = g_scene.spawn_kind; /* -1 means let each particle pick a random type */
  for (int i = 0; i < n; i++) {
    int slot = find_dead_slot();
    if (slot < 0)
      break;
    float angle = inward_velocity_angle(base_angle, 0.5235988f); /* ±30° */
    init_particle(&g_scene.particles[slot], cx, cy, angle, k);
  }
}

static void scene_step(void) {
  for (int i = 0; i < MAX_PARTICLES; i++)
    particle_step(&g_scene.particles[i]);
}

/*
 * Wipe the chamber and start a fresh burst from the centre. The reset key and
 * the parameter keys (field strength, flip, type) call this so you see the new
 * setting take effect right away on new particles — the ones already flying
 * locked in their curl before the change and wouldn't show it.
 */
static void respawn_centre_burst(void) {
  scene_reset();
  int n = BURST_MIN + rand() % (BURST_MAX - BURST_MIN + 1);
  spawn_burst_centre(n);
}

static int alive_count(void) {
  int n = 0;
  for (int i = 0; i < MAX_PARTICLES; i++)
    if (g_scene.particles[i].alive)
      n++;
  return n;
}

/* ── §6 draw ── */

/*
 * The trail is drawn oldest-to-newest in a few brightness bands. These small
 * helpers pick the character for a given age, turn a remembered position into
 * a screen spot, and paint the trail and the bright head; draw_particle ties
 * them together. The fade idea — bright fresh, fainter as it ages — follows
 * Ware [5].
 */

/*
 * Pick the character and boldness for a trail dot, given how old it is (0 =
 * just laid down, 1 = oldest we keep). Fresh dots are a bold '*', middle-aged
 * ones a plain '+', old ones a faint '.', and anything past 0.8 we skip — that
 * lets the caller stop early since dots only get older from there. Just three
 * coarse bands, but they read as a clean bright-to-faint fade on a terminal.
 */
static int trail_age_glyph(float age, chtype *ch, attr_t *extra_attr) {
  if (age >= 0.80f)
    return 0;
  if (age < 0.25f) {
    *ch = '*';
    *extra_attr = A_BOLD;
  } else if (age < 0.55f) {
    *ch = '+';
    *extra_attr = A_NORMAL;
  } else {
    *ch = '.';
    *extra_attr = A_NORMAL;
  }
  return 1;
}

/*
 * Turn the i-th remembered position (i = 0 is the newest) into a screen
 * column and row, nudging down by one for the top status line. Returns 1 if
 * that spot is on screen so the caller draws it, 0 if it's off the edge so the
 * caller skips it.
 */
static int trail_sample_to_screen(const Particle *p, int i, int draw_rows,
                                  int *col, int *row) {
  int idx = (p->thead - 1 - i + TRAIL_LEN * 2) % TRAIL_LEN;
  *col = (int)(p->tx[idx] + 0.5f);
  *row = (int)(p->ty[idx] + 0.5f) + HUD_TOP;
  return *col >= 0 && *col < g_cols && *row >= HUD_TOP && *row < draw_rows;
}

/*
 * Draw the whole trail in the particle's colour, newest dot first. We stop the
 * moment a dot is too old to draw — since they only get older after that,
 * there's no point checking the rest.
 */
static void paint_trail_history(const Particle *p, int draw_rows, int cp) {
  int denom = p->tlen > 1 ? p->tlen : 1;
  for (int i = 0; i < p->tlen; i++) {
    chtype ch;
    attr_t extra;
    float age = (float)i / (float)denom;
    if (!trail_age_glyph(age, &ch, &extra))
      break;

    int col, row;
    if (!trail_sample_to_screen(p, i, draw_rows, &col, &row))
      continue;

    attr_t attr = (attr_t)COLOR_PAIR(cp) | extra;
    attron(attr);
    mvaddch(row, col, ch);
    attroff(attr);
  }
}

/*
 * Draw the particle itself as a bright 'O' at its current spot — the bright
 * head leading the fading trail.
 */
static void paint_live_head(const Particle *p, int draw_rows, int cp) {
  int col = (int)(p->x + 0.5f);
  int row = (int)(p->y + 0.5f) + HUD_TOP;
  if (col < 0 || col >= g_cols || row < HUD_TOP || row >= draw_rows)
    return;
  attron(COLOR_PAIR(cp) | A_BOLD);
  mvaddch(row, col, 'O');
  attroff(COLOR_PAIR(cp) | A_BOLD);
}

/* Draw one particle: its fading trail, plus the bright head if it's alive. */
static void draw_particle(const Particle *p, int draw_rows) {
  if (!p->alive && p->tlen == 0)
    return;

  int cp = particle_cp(p->kind);
  paint_trail_history(p, draw_rows, cp);
  if (p->alive)
    paint_live_head(p, draw_rows, cp);
}

/*
 * The top status line is built in three pieces so its middle "[type]" tag can
 * be coloured to match the particle it names — a tiny legend. The rest of the
 * line stays bright yellow, so the bar reads as one yellow stripe broken only
 * by that coloured tag. Layout:
 *
 *   " B=N.NN[(flipped)]  alive=K/M  spawn=[X]  theme:NAME  STATE "
 *   |________ prefix ______________________| |_tag_| |__ suffix _|
 *
 * Colour-as-legend idea from Ware [5].
 */

/* Left piece: field strength, how many are alive, and "spawn=". */
static void format_hud_status_prefix(char *buf, size_t n) {
  snprintf(buf, n, " B=%.2f%s  alive=%d/%d  spawn=", (double)fabsf(g_scene.B),
           g_scene.B < 0 ? "(flipped)" : "", alive_count(), MAX_PARTICLES);
}

/* Middle piece: "[rand]", or the type tag like "[e-]" when a type is pinned. */
static void format_hud_spawn_tag(char *buf, size_t n) {
  if (g_scene.spawn_kind < 0)
    snprintf(buf, n, "[rand]");
  else
    snprintf(buf, n, "[%s]", k_types[g_scene.spawn_kind].symbol);
}

/* Right piece: current theme name and whether we're paused or running. */
static void format_hud_status_suffix(char *buf, size_t n) {
  snprintf(buf, n, "  theme:%s  %s ", g_themes[g_scene.theme % N_THEMES].name,
           g_scene.paused ? "PAUSED" : "running");
}

/*
 * Print the spawn-type tag in that type's own colour, then switch back to the
 * yellow HUD colour so the rest of the line is unaffected. The caller has the
 * yellow style on already; we only borrow it for the tag. Random spawns have
 * no single type, so they just print in plain yellow.
 */
static void paint_spawn_tag_with_particle_colour(const char *tag) {
  if (g_scene.spawn_kind < 0) {
    addstr(tag);
    return;
  }
  int cp = particle_cp(g_scene.spawn_kind);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
  attron(COLOR_PAIR(cp) | A_BOLD);
  addstr(tag);
  attroff(COLOR_PAIR(cp) | A_BOLD);
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Draw the status line, right-aligned along the top row in bright yellow. */
static void draw_hud_top(void) {
  char before[80], spawn_tag[16], after[64];
  format_hud_status_prefix(before, sizeof before);
  format_hud_spawn_tag(spawn_tag, sizeof spawn_tag);
  format_hud_status_suffix(after, sizeof after);

  int total = (int)(strlen(before) + strlen(spawn_tag) + strlen(after));
  int col = g_cols - total;
  if (col < 0)
    col = 0;

  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddstr(0, col, before);
  paint_spawn_tag_with_particle_colour(spawn_tag);

  addstr(after);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/*
 * Draw the key-hint line along the bottom row. Falls back to a shorter list
 * when the terminal is too narrow to fit the full one.
 */
static void draw_hud_bottom(void) {
  const char *hint_full =
      " q:quit  p:pause  r:reset  n:burst-centre  e:burst-edge  "
      "b/B:field  spc:flip  k/K:type  t/T:theme ";
  const char *hint_short =
      " q:quit  p:pause  r:reset  n:burst  k:type  t:theme ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= g_cols - 1)
    hint = hint_short;

  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(g_rows - 1, 0, hint, g_cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void scene_draw(void) {
  /* Particles fill the rows between the top status line and the bottom hint
   * line; draw_rows is the first row past the area, where we stop. */
  int draw_rows = g_rows - HUD_BOT;

  for (int i = 0; i < MAX_PARTICLES; i++)
    draw_particle(&g_scene.particles[i], draw_rows);

  draw_hud_top();
  draw_hud_bottom();
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

  /* Theme defaults to g_scene.theme = 0 (Matrix) from the Scene initializer. */
  color_init(g_scene.theme);

  getmaxyx(stdscr, g_rows, g_cols);
  /* initial event: mixed burst from centre */
  respawn_centre_burst();

  long long next_frame = clock_ns();

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
      respawn_centre_burst();
      break;

    case 'n':
    case 'N': {
      int n = BURST_MIN + rand() % (BURST_MAX - BURST_MIN + 1);
      spawn_burst_centre(n);
      break;
    }
    case 'e':
    case 'E': {
      int n = BURST_MIN + rand() % (BURST_MAX - BURST_MIN + 1);
      spawn_burst_edge(n);
      break;
    }

    /* Parameter-change keys auto-respawn so the new value is
     * visible on fresh particles, not just baked into the next
     * tick of the existing slow / dying ones. */
    case 'b':
      g_scene.B += B_STEP;
      if (g_scene.B > B_MAX)
        g_scene.B = B_MAX;
      respawn_centre_burst();
      break;
    case 'B':
      g_scene.B -= B_STEP;
      if (fabsf(g_scene.B) < B_MIN)
        g_scene.B = (g_scene.B < 0) ? -B_MIN : B_MIN;
      respawn_centre_burst();
      break;

    case ' ':
      g_scene.B = -g_scene.B; /* flip field direction — reverses all curls */
      respawn_centre_burst();
      break;

    case 'k':
      /* cycle spawn-kind forward: rand → e- → e+ → mu → pi → p → rand */
      g_scene.spawn_kind = (g_scene.spawn_kind + 1 + 1) % (N_TYPES + 1) - 1;
      respawn_centre_burst();
      break;
    case 'K':
      g_scene.spawn_kind =
          (g_scene.spawn_kind + N_TYPES + 1) % (N_TYPES + 1) - 1;
      respawn_centre_burst();
      break;

    case 't':
      g_scene.theme = (g_scene.theme + 1) % N_THEMES;
      color_init(g_scene.theme);
      break;
    case 'T':
      g_scene.theme = (g_scene.theme + N_THEMES - 1) % N_THEMES;
      color_init(g_scene.theme);
      break;

    default:
      break;
    }

    long long now = clock_ns();
    if (!g_scene.paused && now >= next_frame) {
      for (int s = 0; s < STEPS_PER_FRAME; s++)
        scene_step();
      next_frame = now + RENDER_NS;
    }

    erase();
    scene_draw();
    wnoutrefresh(stdscr);
    doupdate();
    clock_sleep_ns(next_frame - clock_ns());
  }
  return 0;
}
