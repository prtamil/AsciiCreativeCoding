/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * beam_bending.c — watch a beam bend and wobble under load, in the terminal.
 *
 * We pick a beam (pinned, clamped one end, or clamped both ends), drop a
 * load on it, and draw how far it sags. A second mode lets the beam ring
 * like a struck tuning fork instead of sitting still. The classic
 * small-deflection beam theory (Euler-Bernoulli) gives us exact formulas
 * for the shape, so there's no step-by-step solver — we just evaluate the
 * formulas and the vibration as a sum of natural ringing patterns.
 *
 * Sources the code can't tell you on its own:
 *   [1] Gere & Goodno, "Mechanics of Materials", 9th ed. — the beam theory
 *       behind §4-§5.
 *   [2] Roark's "Formulas for Stress and Strain", 8th ed., Tables 8.1-8.10
 *       — the exact sag/moment formulas §5 transcribes.
 *   [3] Rao, "Mechanical Vibrations", 6th ed., Ch. 8 — the vibration math §6.
 *   [4] Blevins, "Formulas for Natural Frequency and Mode Shape", Table 8-1
 *       — the magic ringing-pattern numbers in §6.
 *   [5] Ware, "Information Visualization", 4th ed. — why the colour and
 *       character-density ramps read the way they do.
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

#define N_NODES 200       /* how many points we sample along the beam   */
#define N_TRAIL 12        /* past beam shapes kept for the blur trail   */
#define BEAM_HEIGHT 3     /* how tall the beam looks, in rows (odd)     */
#define PANEL_W 22        /* width of the moment chart on the right     */
#define BEAM_X_MARGIN 4   /* blank columns left and right of the beam   */
#define LOAD_ARROW_H 4    /* how tall the load arrows are, in rows      */
#define TARGET_FPS 30     /* frames per second we aim for               */
#define RAMP_SPEED 0.7f   /* a new load fades in over ~1.4 s            */
#define N_MODES 4         /* how many ringing patterns we add up        */
#define MODAL_DAMP 0.025f /* how fast the wobble dies down (2.5%)       */

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL

/*
 * Colour slots, named by what they show so colour choices stay in one place.
 * The theme table in §3 fills in the data slots (stress levels, moment bars,
 * load, supports, panel header). The HUD, hint, and dim-grey slots stay the
 * same across every theme so the on-screen text is always readable.
 */
#define CP_LO 1    /* low-stress part of the beam   — per theme */
#define CP_MED 2   /* medium-stress part           — per theme */
#define CP_HI 3    /* high-stress part             — per theme */
#define CP_MOM_P 4 /* downward-bowing moment bar    — per theme */
#define CP_MOM_N 5 /* upward-bowing moment bar      — per theme */
#define CP_LOAD 6  /* load arrows and label        — per theme */
#define CP_SUPP 7  /* support symbols              — per theme */
#define CP_HUD 8   /* top status line — bright yellow + bold    */
#define CP_DIM 9   /* faint reference lines        — grey       */
#define CP_HDR 10  /* moment-panel header          — per theme */
#define CP_HINT 11 /* bottom key-hint line — bright cyan + bold */

#define N_THEMES 10

typedef enum { BC_SS = 0, BC_CANT, BC_FF, BC_COUNT } BCType;
typedef enum { LD_CENTER = 0, LD_UDL, LD_OFFSET, LD_COUNT } LDType;

static const char *bc_names[BC_COUNT] = {
    "Simply Supported",
    "Cantilever (Fixed-Free)",
    "Fixed-Fixed",
};
static const char *ld_names[LD_COUNT] = {
    "Center Point Load",
    "Uniform Distributed Load",
    "Offset Point (3/4 span)",
};

/* ── §2 clock ── */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec r = {(time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC)};
  nanosleep(&r, NULL);
}

/* ── §3 color / themes ── */

/*
 * Theme — one colour scheme: eight colour numbers, one for each thing we draw.
 *
 * Each field is a 256-colour terminal palette number. The fields:
 *   name          what to show in the HUD when this theme is active.
 *   lo / med / hi  three shades for the beam, from least to most stressed,
 *                  so a glance tells you where the beam is working hardest.
 *   mom_p / mom_n  colours for the two moment bars (beam bowing down vs up).
 *                  The '>' / '<' glyphs already show which way, so these two
 *                  just need to look different, not be opposites.
 *   load          colour of the load arrows and the 'P' label.
 *   supp          colour of the support symbols (pin 'A', wall '#', "free").
 *   hdr           colour of the moment panel's header text.
 *
 * Every colour is picked bright enough to stay visible on a default-dark
 * terminal — see the brightness rule in CLAUDE.md.
 */
typedef struct {
  const char *name;
  int lo, med, hi;
  int mom_p, mom_n;
  int load;
  int supp;
  int hdr;
} Theme;

static const Theme themes[N_THEMES] = {
    /*  name        lo   med  hi   mp   mn   ld   sp   hd  */
    {"Matrix", 28, 40, 46, 46, 118, 226, 82, 82},        /* cyber green */
    {"Fire", 94, 208, 196, 220, 124, 226, 255, 208},     /* hot reds    */
    {"Oceanic", 24, 39, 159, 51, 129, 226, 195, 45},     /* blues/teal  */
    {"Neon", 93, 201, 51, 51, 201, 226, 255, 213},       /* magenta/cyn */
    {"Mono", 240, 247, 255, 255, 244, 250, 252, 250},    /* grayscale   */
    {"Ice", 153, 159, 195, 195, 141, 117, 255, 159},     /* light blues */
    {"Nova", 129, 213, 226, 226, 201, 220, 255, 213},    /* purple-gold */
    {"Forest", 58, 100, 190, 82, 130, 220, 144, 178},    /* greens/tan  */
    {"Desert", 180, 215, 220, 220, 124, 226, 252, 215},  /* sand/gold   */
    {"Eclipse", 240, 244, 196, 255, 196, 226, 244, 196}, /* dim + crim. */
};

static void color_init(int theme) {
  start_color();
  use_default_colors();
  const Theme *th = &themes[theme];
  if (COLORS >= 256) {
    /* HUD colours stay the same in every theme so the text stays readable. */
    init_pair(CP_HUD, 226, -1); /* bright yellow */
    init_pair(CP_HINT, 51, -1); /* bright cyan   */
    init_pair(CP_DIM, 240, -1); /* dark grey     */

    /* The rest come from the chosen theme. */
    init_pair(CP_LO, th->lo, -1);
    init_pair(CP_MED, th->med, -1);
    init_pair(CP_HI, th->hi, -1);
    init_pair(CP_MOM_P, th->mom_p, -1);
    init_pair(CP_MOM_N, th->mom_n, -1);
    init_pair(CP_LOAD, th->load, -1);
    init_pair(CP_SUPP, th->supp, -1);
    init_pair(CP_HDR, th->hdr, -1);
  } else {
    /* Old 8-colour terminal: ignore the theme, use these basic colours. */
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
    init_pair(CP_DIM, COLOR_WHITE, -1);
    init_pair(CP_LO, COLOR_GREEN, -1);
    init_pair(CP_MED, COLOR_YELLOW, -1);
    init_pair(CP_HI, COLOR_RED, -1);
    init_pair(CP_MOM_P, COLOR_GREEN, -1);
    init_pair(CP_MOM_N, COLOR_MAGENTA, -1);
    init_pair(CP_LOAD, COLOR_YELLOW, -1);
    init_pair(CP_SUPP, COLOR_WHITE, -1);
    init_pair(CP_HDR, COLOR_YELLOW, -1);
  }
}

/* ── §4 beam entity ── */

/*
 * DynBeam — everything we need to make the beam wobble (§6).
 *
 * The idea: a vibrating beam doesn't move in some random way. It rings in a
 * few fixed patterns at the same time, like the harmonics of a guitar string.
 * Each pattern is a fixed shape along the beam (a "mode shape") that always
 * looks the same, paired with an amount that swings up and down over time.
 * Add the patterns up and you get the live shape of the beam. We track four
 * patterns; more would barely change the picture.
 *
 * We keep this in its own struct because it only matters while the beam is
 * vibrating: dyn_activate fills it in, dyn_tick_modes moves it forward each
 * frame, dyn_impulse kicks it, and the 'd' key flips `active` on and off.
 *
 * The fixed numbers that define each ringing pattern (one set per beam type)
 * are tabulated constants — see dyn_setup and Blevins Table 8-1 [4].
 *
 * Why dyn_setup does its math in double, not float: the higher patterns for
 * clamped beams involve subtracting two big, nearly equal numbers, which
 * eats up precision. Double has enough digits to survive it; the final shape
 * is stored back as float since the renderer just reads it.
 *
 * Background math: Rao Ch. 8 [3] (the patterns and how they add up),
 * Blevins Table 8-1 [4] (the tabulated numbers).
 */
typedef struct {
  /* ── Set once when vibration starts, then never changes ── */
  float phi[N_MODES][N_NODES]; /* the four ringing patterns: for each one,
                                * its fixed shape sampled at every point
                                * along the beam. Computed once so the
                                * per-frame loop is just multiply-and-add. */
  float omega[N_MODES];        /* how fast each pattern swings (its natural
                                * frequency) — higher patterns ring faster. */
  float modal_mass[N_MODES];   /* a normalising weight per pattern, so the
                                * load pushes each one by the right amount.
                                * Computed by integrating the shape squared
                                * along the beam (see dyn_setup).           */
  float Fn[N_MODES];           /* how hard the current load pushes each
                                * pattern. Already divided by modal_mass so
                                * the per-frame step needs no division.
                                * Refreshed by dyn_load when the load changes.*/

  /* ── Updated every simulation step ── */
  float q[N_MODES];    /* how much of each pattern is in play right now —
                        * this is what swings up and down over time. */
  float qdot[N_MODES]; /* how fast each of those amounts is changing —
                        * needed to step the swing forward.          */

  /* ── Display scale and on/off state ── */
  float ref_max_w; /* the vertical zoom level for drawing, locked in when
                    * vibration starts. Only ever grows, never shrinks, so
                    * the beam doesn't appear to zoom in and out as it
                    * swings through the flat resting position.            */
  bool active;     /* true  → the beam is vibrating (dyn_tick_modes drives it)
                    * false → the beam sits still (the static solver drives it)
                    * Flipped by the 'd' key.                              */

  /* ── Fading after-image trail (drawn by §7 render_trails) ── */
  float trail_w[N_TRAIL][N_NODES]; /* a rolling record of the last few beam
                                    * shapes, kept so we can draw faint
                                    * after-images behind the live beam.   */
  int trail_head;                  /* where the next snapshot goes; wraps
                                    * around the ring of N_TRAIL slots.    */
} DynBeam;

/*
 * Beam — everything about one beam: its shape, its load, and how it's wobbling.
 *
 * To keep the formulas tidy we measure the beam in its own units: length 1,
 * stiffness 1. That strips the units out of the §5 formulas (a cantilever tip
 * sags by "P/3" instead of "PL³/(3EI)") and costs nothing, since the screen
 * only shows a visually-exaggerated picture, never real metres.
 *
 * We don't store one continuous curve — we store the beam's height at 200
 * evenly-spaced points along it. That makes drawing easy: each point maps
 * straight to a screen column. The static and vibrating modes both just
 * overwrite the same height array, so the rest of the code doesn't care which
 * one produced it.
 *
 * Heights are stored with "down" being positive. The renderer multiplies by
 * `exag` to make tiny real sags big enough to see; the physics never touches
 * `exag`.
 *
 * Background math: Gere [1] (the beam theory), Roark [2] (the §5 formulas),
 * Rao [3] (the vibration sub-state).
 */
typedef struct {
  /* ── The beam sampled at 200 points along its length ── */
  float w[N_NODES]; /* how far the beam has sagged at this point (down = +) */
  float M[N_NODES]; /* the bending effort at this point (bowing down = +)   */
  float x[N_NODES]; /* this point's position along the beam, 0 to 1.
                     * Computed once at startup so §5 doesn't redo it.    */

  /* ── Fixed beam properties, plus one drawing knob ── */
  float L;    /* beam length — always 1 in our units */
  float EI;   /* beam stiffness — always 1 in our units */
  float P;    /* how heavy the load is (+/- keys, 0.05 to 100).
               * Multiplied by load_anim before it bends the beam. */
  float exag; /* drawing-only exaggeration of the sag (e / E keys,
               * 0.5x to 20x). Never affects the physics.          */

  /* ── Things that change while running ── */
  float load_anim;     /* how much of the load is "on" right now, 0 to 1.
                        * When sitting still, a new load fades in over a
                        * fraction of a second instead of popping on.
                        * When vibrating, it's snapped straight to 1.   */
  float damping_ratio; /* how quickly the wobble dies down (z / Z keys).
                        * 0.001 = rings for ages; 0.5 = stops almost at
                        * once. Default 0.025 is about right for steel. */
  bool paused;         /* spacebar — freezes the whole simulation.       */

  /* ── Which beam and which load (picks the §5 / §6 formula) ── */
  BCType bc;   /* how the ends are held — n / p keys */
  LDType load; /* where the load sits — l key        */

  /* ── Peak values, recomputed each step, used to scale the drawing ── */
  float max_deflection; /* the biggest sag anywhere on the beam — sets the
                         * vertical zoom. While vibrating this is frozen so
                         * the beam doesn't appear to zoom in and out.   */
  float max_moment;     /* the biggest bending effort anywhere — sets the
                         * stress colours and the length of the moment bars.*/

  /* ── Vibration state (only meaningful while dyn.active) ── */
  DynBeam dyn;
} Beam;

/* Sets which beam, which load, and how heavy. Deliberately leaves the
 * fade-in alone — callers reset it themselves when they want the new load
 * to fade in from nothing. */
static void apply_load(Beam *b, BCType bc, LDType ld, float P) {
  b->bc = bc;
  b->load = ld;
  b->P = P;
}

static void init_beam(Beam *b) {
  memset(b, 0, sizeof *b);
  b->L = 1.0f;
  b->EI = 1.0f;
  b->exag = 1.0f;
  b->load_anim = 0.0f;
  b->damping_ratio = MODAL_DAMP; /* 2.5%, like structural steel */
  b->paused = false;
  for (int i = 0; i < N_NODES; i++)
    b->x[i] = (float)i / (float)(N_NODES - 1);
  /* Start with the most fun setup: a diving-board beam with a weight on the
   * tip. It sags the most and wobbles the slowest, so it's the easiest to
   * watch. */
  apply_load(b, BC_CANT, LD_CENTER, 1.0f);
}

/* ── §5 solver — exact beam-shape formulas ── */

/*
 * For a beam sitting still, we don't simulate anything — there's a known
 * formula for the exact sag and bending at every point, for each combination
 * of beam type and load. The helpers below are those formulas, copied from
 * Roark's tables [2]. Each takes a position and the load and hands back the
 * sag and bending there; none of them touch any shared state.
 *
 * solve_beam picks the right helper for the current beam/load, runs it at
 * every point, and notes the biggest sag and bending for the drawing code.
 *
 * Throughout: sag is positive downward, and bending is positive when the
 * beam bows downward (smiling) and negative when it bows up (frowning).
 */

/*
 * Pinned at both ends, with a single weight somewhere along it (at x = a).
 * Both ends rest on supports they can pivot on. Roark Table 8.1 [2].
 */
static void simply_supported_point_load(float x, float L, float EI, float P,
                                        float a, float *M, float *w) {
  float b = L - a;
  if (x <= a) {
    *M = P * b * x / L;
    *w = P * b * x * (L * L - b * b - x * x) / (6.0f * EI * L);
  } else {
    float xs = L - x;
    *M = P * a * xs / L;
    *w = P * a * xs * (L * L - a * a - xs * xs) / (6.0f * EI * L);
  }
}

/*
 * Pinned at both ends, with the load spread evenly along the whole beam
 * (like its own weight). Sags most in the middle. Roark Table 8.1 [2].
 */
static void simply_supported_udl(float x, float L, float EI, float q, float *M,
                                 float *w) {
  *M = q * x * (L - x) / 2.0f;
  *w = q * x * (L * L * L - 2.0f * L * x * x + x * x * x) / (24.0f * EI);
}

/*
 * A diving board: clamped into a wall at one end, free at the other, with a
 * weight hung on the free tip. Sags the most of any setup here, and all the
 * bending effort piles up at the wall. Roark Table 8.2 [2].
 */
static void cantilever_tip_point_load(float x, float L, float EI, float P,
                                      float *M, float *w) {
  *M = P * (L - x);
  *w = P * x * x * (3.0f * L - x) / (6.0f * EI);
}

/*
 * Same diving board, but the weight sits partway out (at x = a), not on the
 * tip. Past the weight the beam carries no bending, so the leftover stub just
 * sticks out straight at whatever angle it had reached. Roark Table 8.2 [2].
 */
static void cantilever_intermediate_point_load(float x, float L, float EI,
                                               float P, float a, float *M,
                                               float *w) {
  (void)L; /* the formula only needs the load position, not the length */
  if (x <= a) {
    *M = P * (a - x);
    *w = P * x * x * (3.0f * a - x) / (6.0f * EI);
  } else {
    *M = 0.0f;
    *w = P * a * a * (3.0f * x - a) / (6.0f * EI);
  }
}

/*
 * Diving board again, but loaded evenly along its whole length instead of at
 * one spot. The wall takes the most strain of any diving-board case here.
 * Roark Table 8.2 [2].
 */
static void cantilever_udl(float x, float L, float EI, float q, float *M,
                           float *w) {
  float Lx = L - x;
  *M = q * Lx * Lx / 2.0f;
  *w = q * x * x * (6.0f * L * L - 4.0f * L * x + x * x) / (24.0f * EI);
}

/*
 * Both ends clamped into walls, with a single weight right in the middle.
 * This is the stiffest setup here — it sags only a quarter as much as the
 * pinned beam, because the walls fight the bending. The beam bows up near
 * the walls and down in the middle. Roark Table 8.3 [2].
 */
static void fixed_fixed_center_point_load(float x, float L, float EI, float P,
                                          float *M, float *w) {
  if (x <= 0.5f * L) {
    *M = P * x / 2.0f - P * L / 8.0f;
    *w = P * x * x * (3.0f * L - 4.0f * x) / (48.0f * EI);
  } else {
    float xs = L - x;
    *M = P * xs / 2.0f - P * L / 8.0f;
    *w = P * xs * xs * (3.0f * L - 4.0f * xs) / (48.0f * EI);
  }
}

/*
 * Both ends clamped, load spread evenly along the whole beam. The walls
 * take the most strain here; the middle sags but bows the other way near
 * each wall. Sags only a fifth as much as the same load on a pinned beam.
 * Roark Table 8.3 [2].
 */
static void fixed_fixed_udl(float x, float L, float EI, float q, float *M,
                            float *w) {
  float Lx = L - x;
  *M = q * x * Lx / 2.0f - q * L * L / 12.0f;
  *w = q * x * x * Lx * Lx / (24.0f * EI);
}

/*
 * Both ends clamped, but the weight sits off to one side (at x = a) instead
 * of the middle. The lopsided position means each wall pulls a different
 * amount. If the weight were centred this would match the center-load case
 * above. Roark Table 8.3 (intermediate point load) [2].
 */
static void fixed_fixed_offset_point_load(float x, float L, float EI, float P,
                                          float a, float *M, float *w) {
  float b = L - a;
  float L2 = L * L;
  float L3 = L * L2;
  float MA = P * a * b * b / L2;
  float RA = P * b * b * (3.0f * a + b) / L3;

  *M = -MA + RA * x - (x > a ? P * (x - a) : 0.0f);
  if (x <= a) {
    *w = P * b * b * x * x * (3.0f * a * L - (3.0f * a + b) * x) /
         (6.0f * EI * L3);
  } else {
    float xs = L - x;
    *w = P * a * a * xs * xs * (3.0f * b * L - (3.0f * b + a) * xs) /
         (6.0f * EI * L3);
  }
}

/*
 * Fills in the whole beam shape for whatever's set up right now: picks the
 * matching formula above, runs it at every point, and notes the biggest sag
 * and bending so the drawing code knows how far to zoom and how to colour.
 */
static void solve_beam(Beam *b) {
  float L = b->L;
  float EI = b->EI;
  float P = b->P * b->load_anim;

  for (int i = 0; i < N_NODES; i++) {
    float x = b->x[i] * L;
    float w = 0.0f, M = 0.0f;

    switch (b->bc) {
    case BC_SS:
      switch (b->load) {
      case LD_CENTER:
        simply_supported_point_load(x, L, EI, P, 0.5f * L, &M, &w);
        break;
      case LD_UDL:
        simply_supported_udl(x, L, EI, P, &M, &w);
        break;
      case LD_OFFSET:
        simply_supported_point_load(x, L, EI, P, 0.75f * L, &M, &w);
        break;
      default:
        break;
      }
      break;

    case BC_CANT:
      switch (b->load) {
      case LD_CENTER:
        cantilever_tip_point_load(x, L, EI, P, &M, &w);
        break;
      case LD_UDL:
        cantilever_udl(x, L, EI, P, &M, &w);
        break;
      case LD_OFFSET:
        cantilever_intermediate_point_load(x, L, EI, P, 0.5f * L, &M, &w);
        break;
      default:
        break;
      }
      break;

    case BC_FF:
      switch (b->load) {
      case LD_CENTER:
        fixed_fixed_center_point_load(x, L, EI, P, &M, &w);
        break;
      case LD_UDL:
        fixed_fixed_udl(x, L, EI, P, &M, &w);
        break;
      case LD_OFFSET:
        fixed_fixed_offset_point_load(x, L, EI, P, 0.75f * L, &M, &w);
        break;
      default:
        break;
      }
      break;

    default:
      break;
    }

    b->w[i] = w;
    b->M[i] = M;
  }

  /* Note the biggest sag and bending — drives the zoom and the colours. */
  float max_w = 0.0f, max_M = 0.0f;
  for (int i = 0; i < N_NODES; i++) {
    if (fabsf(b->w[i]) > max_w)
      max_w = fabsf(b->w[i]);
    if (fabsf(b->M[i]) > max_M)
      max_M = fabsf(b->M[i]);
  }
  b->max_deflection = max_w;
  b->max_moment = max_M;
}

/* ── §6 dynamics — making the beam wobble ── */

/*
 * Each helper below works out the ringing patterns for one kind of beam:
 * the fixed shape of each pattern at every point, plus how fast it swings.
 * There's one helper per beam type because the wall-vs-pin ends change the
 * shapes. They all do their math in double precision for accuracy (see the
 * DynBeam note in §4), then store the shape as float for the drawing code.
 *
 * The pinned beam's patterns are simple sine waves. The clamped ones need
 * a curvier shape that flattens out hard against each wall.
 *
 * Background math: Rao §8 [3], Blevins Table 8-1 [4].
 */

/*
 * Pinned-at-both-ends beam: its ringing patterns are just plain sine waves,
 * like the shapes a guitar string makes. The nth pattern fits n humps
 * between the ends, and the more humps, the faster it swings.
 */
static void setup_simply_supported_modes(DynBeam *d, const Beam *b) {
  double L = (double)b->L;
  for (int n = 0; n < N_MODES; n++) {
    double beta = (double)(n + 1) * 3.14159265358979323846 / L;
    d->omega[n] = (float)(beta * beta);
    for (int i = 0; i < N_NODES; i++)
      d->phi[n][i] = (float)sin(beta * b->x[i] * L);
  }
}

/*
 * Diving-board beam (clamped one end, free at the other): its ringing
 * patterns are flat against the wall and free to flap at the tip. The four
 * `lam` numbers are tabulated constants that pin down each pattern's shape
 * — there's no neat formula for them, so they're looked up (Blevins [4]).
 * Done in double because the higher patterns subtract two huge nearly-equal
 * numbers, which would wreck the accuracy in plain float.
 */
static void setup_cantilever_modes(DynBeam *d, const Beam *b) {
  static const double lam[N_MODES] = {1.87510, 4.69409, 7.85476, 10.99554};
  double L = (double)b->L;
  for (int n = 0; n < N_MODES; n++) {
    double bn = lam[n] / L;
    double sig = (cos(lam[n]) + cosh(lam[n])) / (sin(lam[n]) + sinh(lam[n]));
    d->omega[n] = (float)(bn * bn);
    for (int i = 0; i < N_NODES; i++) {
      double x = b->x[i] * L;
      d->phi[n][i] = (float)(cosh(bn * x) - cos(bn * x) -
                             sig * (sinh(bn * x) - sin(bn * x)));
    }
  }
}

/*
 * Both-ends-clamped beam: its ringing patterns are flat against both walls
 * and bulge in between. Like the diving board, the four `lam` numbers are
 * looked-up constants (Blevins [4]) and the math runs in double to survive
 * the same big-number cancellation. The `sig` guard falls back to -1 if a
 * divide would blow up — shouldn't happen with these numbers, just safety.
 */
static void setup_fixed_fixed_modes(DynBeam *d, const Beam *b) {
  static const double lam[N_MODES] = {4.73004, 7.85321, 10.99561, 14.13717};
  double L = (double)b->L;
  for (int n = 0; n < N_MODES; n++) {
    double bn = lam[n] / L;
    double den = sin(lam[n]) - sinh(lam[n]);
    double sig =
        (fabs(den) > 1e-10) ? (cos(lam[n]) - cosh(lam[n])) / den : -1.0;
    d->omega[n] = (float)(bn * bn);
    for (int i = 0; i < N_NODES; i++) {
      double x = b->x[i] * L;
      d->phi[n][i] = (float)(cosh(bn * x) - cos(bn * x) -
                             sig * (sinh(bn * x) - sin(bn * x)));
    }
  }
}

/*
 * Works out a "weight" for each ringing pattern by adding up its shape
 * squared along the beam. This is what lets the load push each pattern by
 * the right amount later. If the sum comes out tiny (which shouldn't
 * happen), we clamp it to 1 so nothing divides by zero downstream.
 *
 * Background math: Rao §8 (modal mass) [3].
 */
static void integrate_modal_masses_trapezoidal(DynBeam *d, const Beam *b) {
  double dx = (double)b->L / (double)(N_NODES - 1);
  for (int n = 0; n < N_MODES; n++) {
    double mm = 0.0;
    for (int i = 0; i < N_NODES; i++)
      mm += (double)d->phi[n][i] * d->phi[n][i] * dx;
    d->modal_mass[n] = (float)(mm > 1e-9 ? mm : 1.0);
  }
}

/*
 * Sets up the ringing patterns for whichever beam is selected: picks the
 * right pattern-builder, then works out each pattern's weight. Run once when
 * vibration starts, and again if you switch beam type or load mid-wobble.
 * The patterns don't change after this — only how much of each is in play
 * changes from frame to frame.
 */
static void dyn_setup(Beam *b) {
  DynBeam *d = &b->dyn;

  switch (b->bc) {
  case BC_SS:
    setup_simply_supported_modes(d, b);
    break;
  case BC_CANT:
    setup_cantilever_modes(d, b);
    break;
  case BC_FF:
    setup_fixed_fixed_modes(d, b);
    break;
  default:
    break;
  }

  integrate_modal_masses_trapezoidal(d, b);
}

/*
 * Works out how hard the current load pushes on each ringing pattern. A
 * pattern that's tall where the load sits gets pushed a lot; one that's flat
 * there barely feels it. For a spread-out load we add up the push along the
 * whole beam; for a single weight we just sample the pattern where it lands.
 */
static void dyn_load(Beam *b) {
  DynBeam *d = &b->dyn;
  float L = b->L;
  float P = b->P;
  float dx = L / (float)(N_NODES - 1);

  for (int n = 0; n < N_MODES; n++) {
    float Fn = 0.0f;
    if (b->load == LD_UDL) {
      for (int i = 0; i < N_NODES; i++)
        Fn += P * d->phi[n][i] * dx;
    } else {
      float xf = (b->load == LD_CENTER) ? 0.5f : 0.75f;
      int ni = (int)(xf * (float)(N_NODES - 1) + 0.5f);
      Fn = P * d->phi[n][ni];
    }
    d->Fn[n] = Fn / d->modal_mass[n];
  }
}

/*
 * Flips the beam from sitting-still mode into wobbling mode. The beam starts
 * flat and at rest, then the load drops on all at once — which is why it
 * overshoots and swings past where it would settle, before damping reins it
 * in (a suddenly-dropped load makes a beam dip about twice as far on the
 * first swing as it eventually rests). It locks in the zoom level first so
 * the beam doesn't appear to grow and shrink as it swings through flat, then
 * builds the ringing patterns and clears the wobble state.
 */
static void dyn_activate(Beam *b) {
  b->load_anim = 1.0f;
  solve_beam(b); /* get the resting sag so we can lock the zoom */

  DynBeam *d = &b->dyn;
  d->ref_max_w = b->max_deflection; /* lock the vertical zoom */

  dyn_setup(b); /* build the ringing patterns     */
  dyn_load(b);  /* work out the per-pattern push  */

  for (int n = 0; n < N_MODES; n++) {
    d->q[n] = 0.0f;
    d->qdot[n] = 0.0f;
  }
  memset(d->trail_w, 0, sizeof d->trail_w);
  d->trail_head = 0;
  d->active = true;
}

/*
 * Moves one ringing pattern forward a small step in time. Think of each
 * pattern as a damped spring-and-weight that swings, slows, and settles.
 *
 * Instead of nudging it forward in tiny numerical steps, we use the exact
 * formula for where a damped swing lands after a given amount of time. The
 * reason matters: the simple step-it-forward methods blow up when the
 * pattern swings fast relative to the frame rate — and the fastest patterns
 * here do exactly that. The exact formula always stays stable and always
 * decays, no matter how fast the swing or how big the step.
 *
 * Background math: Rao Ch. 2 (damped vibration of one spring-mass) [3].
 */
static void advance_damped_oscillator_exact(float omega, float zeta, float F,
                                            float dt, float *q, float *qdot) {
  float q_s = (omega > 1e-6f) ? F / (omega * omega) : 0.0f;
  float wd = omega * sqrtf(1.0f - zeta * zeta);
  if (wd < 1e-6f)
    wd = 1e-6f; /* keep the swing rate above zero so nothing divides by it */

  float eta = *q - q_s;
  float etad = *qdot;
  float A = eta;
  float B = (etad + zeta * omega * A) / wd;
  float e = expf(-zeta * omega * dt);
  float c = cosf(wd * dt);
  float s = sinf(wd * dt);

  *q = q_s + e * (A * c + B * s);
  *qdot =
      e * ((-zeta * omega * A + wd * B) * c + (-zeta * omega * B - wd * A) * s);
}

/*
 * Builds the live beam shape by adding the ringing patterns together: at
 * each point, take how much of each pattern is in play right now and stack
 * them up. Also hands back the biggest sag so the caller can adjust the zoom
 * without a second pass over the beam.
 *
 * Background math: Rao §8 (adding patterns up) [3].
 */
static float synthesise_deflection_from_modes(Beam *b) {
  const DynBeam *d = &b->dyn;
  float max_w = 0.0f;
  for (int i = 0; i < N_NODES; i++) {
    float w = 0.0f;
    for (int n = 0; n < N_MODES; n++)
      w += d->q[n] * d->phi[n][i];
    b->w[i] = w;
    if (fabsf(w) > max_w)
      max_w = fabsf(w);
  }
  return max_w;
}

/*
 * Saves a snapshot of the current beam shape into the rolling record, so the
 * renderer can draw faint after-images of where the beam just was (§7).
 */
static void push_trail_snapshot(DynBeam *d, const float *w) {
  memcpy(d->trail_w[d->trail_head], w, sizeof(float) * N_NODES);
  d->trail_head = (d->trail_head + 1) % N_TRAIL;
}

/*
 * Lets the zoom level grow but never shrink. As the beam swings through the
 * flat position its sag briefly drops near zero; if the zoom tracked that,
 * the beam would seem to balloon and shrink every swing, which is both
 * dizzying and wrong — the swing itself hasn't gotten any smaller. The 5%
 * margin stops tiny rounding wobble from creeping the zoom up frame by frame.
 */
static void grow_visual_scale_with_hysteresis(DynBeam *d, float current_max) {
  if (current_max > d->ref_max_w * 1.05f)
    d->ref_max_w = current_max;
}

/*
 * One frame of wobble: swing every ringing pattern forward a step, add them
 * up into the new beam shape, save a trail snapshot, and nudge the zoom if
 * the beam swung wider than before. This is the whole vibrating mode in one
 * place — the static formulas in §5 don't run while this is driving.
 */
static void dyn_tick_modes(Beam *b, float dt) {
  DynBeam *d = &b->dyn;
  float zeta = b->damping_ratio; /* z / Z keys */

  for (int n = 0; n < N_MODES; n++)
    advance_damped_oscillator_exact(d->omega[n], zeta, d->Fn[n], dt, &d->q[n],
                                    &d->qdot[n]);

  float max_w = synthesise_deflection_from_modes(b);
  push_trail_snapshot(d, b->w);
  grow_visual_scale_with_hysteresis(d, max_w);
  b->max_deflection = d->ref_max_w;
}

/*
 * Whacks the beam like a tuning fork — a sudden tap at the load spot that
 * sets every ringing pattern going again. Handy for re-energising a wobble
 * that's almost died out, without changing the beam or its load.
 */
static void dyn_impulse(Beam *b, float impulse) {
  DynBeam *d = &b->dyn;
  float xf = 0.5f;
  if (b->bc == BC_CANT)
    xf = 1.0f; /* tap the free tip of the diving board */
  else if (b->load == LD_OFFSET)
    xf = 0.75f;
  int ni = (int)(xf * (float)(N_NODES - 1) + 0.5f);
  if (ni >= N_NODES)
    ni = N_NODES - 1;
  for (int n = 0; n < N_MODES; n++)
    d->qdot[n] += impulse * d->phi[n][ni] / d->modal_mass[n];
}

/* One frame: either keep the beam wobbling, or (sitting still) fade the
 * load in and recompute the resting shape from the formulas. */
static void beam_tick(Beam *b, float dt) {
  if (b->paused)
    return;

  if (b->dyn.active) {
    dyn_tick_modes(b, dt);
  } else {
    /* Sitting still: ease the load in, then recompute the exact sag. */
    if (b->load_anim < 1.0f) {
      b->load_anim += dt * RAMP_SPEED;
      if (b->load_anim > 1.0f)
        b->load_anim = 1.0f;
    }
    solve_beam(b);
  }
}

/* ── §7 render ── */

/* Which screen column a given beam point lands in. */
static int node_col(int i, int x0, int x1) {
  return x0 + i * (x1 - x0) / (N_NODES - 1);
}

/*
 * Turns a sag amount into how many rows below (or above) the centre line to
 * draw it. Real sags are tiny, so we blow them up by the exaggeration knob,
 * then cap the result so the beam can never slide off the screen.
 */
static int defl_row(float w, float max_w, int base_rows, float exag) {
  if (max_w < 1e-9f)
    return 0;
  int dr = (int)(w / max_w * (float)base_rows * exag + 0.5f);
  int cap = base_rows * 3;
  return (dr > cap) ? cap : (dr < -cap) ? -cap : dr;
}

/*
 * Draws one vertical slice of the beam. How busy the character looks and its
 * colour both show how hard the beam is working at this point: sparse green
 * where it's loafing, dense red where it's strained. The top and bottom
 * edges are always '=' to suggest the beam's I-shaped cross-section. The
 * idea is that a packed character just looks more stressed than a faint dot,
 * with the green-yellow-red colours backing it up like a traffic light.
 */
static void draw_curvature_column(int col, int mid_row, float kn, int half_h,
                                  int rows) {
  static const char pal[] = " .-~=*#@";
  int npal = (int)(sizeof pal - 1);

  int pi = (int)(kn * (float)(npal - 1) + 0.5f);
  if (pi < 0)
    pi = 0;
  if (pi >= npal)
    pi = npal - 1;
  chtype fill = (chtype)pal[pi];

  int cp = (kn < 0.35f) ? CP_LO : (kn < 0.70f) ? CP_MED : CP_HI;

  for (int rr = mid_row - half_h; rr <= mid_row + half_h; rr++) {
    if (rr < 2 || rr >= rows - 2)
      continue;
    bool is_flange = (rr == mid_row - half_h || rr == mid_row + half_h);
    chtype ch = is_flange ? '=' : fill;
    attr_t attr = (chtype)COLOR_PAIR(cp) | (rr == mid_row ? A_BOLD : 0);
    attron(attr);
    mvaddch(rr, col, ch);
    attroff(attr);
  }
}

/*
 * The next few helpers draw the load symbols sitting above the beam. The
 * wrapper draw_load_arrows turns the colour on once and picks the right one;
 * the helpers themselves don't fuss with colour.
 */

/*
 * Draws one downward arrow (a 'P' label, a stem, an arrowhead) at a chosen
 * spot along the beam — say the middle, or three-quarters out. The arrow
 * rides up and down with the beam because we re-read the sag at that spot
 * every frame, so during a wobble it bobs along too.
 */
static void draw_point_load_glyph(const Beam *b, float xfrac, int x0, int x1,
                                  int neutral_row, int base_rows, int half_h,
                                  int rows) {
  int lc = x0 + (int)(xfrac * (float)(x1 - x0) + 0.5f);
  int ni = (int)(xfrac * (float)(N_NODES - 1));
  int dr = defl_row(b->w[ni], b->max_deflection, base_rows, b->exag);
  int top = neutral_row + dr - half_h;

  for (int rr = top - LOAD_ARROW_H; rr < top - 1; rr++)
    if (rr >= 2 && rr < rows - 2)
      mvaddch(rr, lc, '|');
  if (top - LOAD_ARROW_H >= 2)
    mvprintw(top - LOAD_ARROW_H, lc - 1, "P");
  if (top - 1 >= 2)
    mvaddch(top - 1, lc, 'v');
}

/*
 * Draws a row of little arrows along the whole beam for an evenly-spread
 * load. Each arrow sits at the local sag, so the line of arrowheads traces
 * the beam's bent shape. A 'q=...' label floats above the middle.
 */
static void draw_distributed_load_glyphs(const Beam *b, int x0, int x1,
                                         int neutral_row, int base_rows,
                                         int half_h, int rows) {
  for (int c = x0; c <= x1; c += 3) {
    int ni = (c - x0) * (N_NODES - 1) / (x1 - x0);
    if (ni < 0)
      ni = 0;
    if (ni >= N_NODES)
      ni = N_NODES - 1;
    int dr = defl_row(b->w[ni], b->max_deflection, base_rows, b->exag);
    int top = neutral_row + dr - half_h;
    if (top - 1 >= 2 && top - 1 < rows - 2)
      mvaddch(top - 1, c, 'v');
    if (top - 2 >= 2 && top - 2 < rows - 2)
      mvaddch(top - 2, c, '|');
  }
  int mc = (x0 + x1) / 2;
  if (neutral_row - half_h - 3 >= 2)
    mvprintw(neutral_row - half_h - 3, mc - 3, "q=%.2f", b->P * b->load_anim);
}

/* Picks the right load symbol for the current load and draws it, turning the
 * load colour on once around the whole thing. */
static void draw_load_arrows(const Beam *b, int x0, int x1, int neutral_row,
                             int base_rows, int half_h, int cols, int rows) {
  (void)cols;
  attron(COLOR_PAIR(CP_LOAD) | A_BOLD);

  switch (b->load) {
  case LD_CENTER:
    draw_point_load_glyph(b, 0.5f, x0, x1, neutral_row, base_rows, half_h,
                          rows);
    break;
  case LD_OFFSET:
    draw_point_load_glyph(b, 0.75f, x0, x1, neutral_row, base_rows, half_h,
                          rows);
    break;
  case LD_UDL:
    draw_distributed_load_glyphs(b, x0, x1, neutral_row, base_rows, half_h,
                                 rows);
    break;
  default:
    break;
  }

  attroff(COLOR_PAIR(CP_LOAD) | A_BOLD);
}

/*
 * Draws the symbols showing how the ends are held: little triangle supports
 * for a pinned beam, a hatched wall for a clamped end, and a "free" label
 * where the diving board sticks out into nothing.
 */
static void draw_supports(BCType bc, int x0, int x1, int neutral_row,
                          int half_h, int cols, int rows) {
  attron(COLOR_PAIR(CP_SUPP) | A_BOLD);

  switch (bc) {

  case BC_SS: {
    int sr = neutral_row + half_h + 1;
    if (sr < rows - 2) {
      mvaddch(sr, x0, 'A');
      mvaddch(sr, x1, 'A');
    }
    if (sr + 1 < rows - 2) {
      mvprintw(sr + 1, x0 - 1, "/_%c", '\\');
      mvprintw(sr + 1, x1 - 1, "/_%c", '\\');
    }
    break;
  }

  case BC_CANT: {
    for (int rr = neutral_row - half_h - 1; rr <= neutral_row + half_h + 1;
         rr++) {
      if (rr < 1 || rr >= rows - 2)
        continue;
      if (x0 - 1 >= 0)
        mvaddch(rr, x0 - 1, '#');
      if (x0 - 2 >= 0)
        mvaddch(rr, x0 - 2, '#');
    }
    if (neutral_row >= 1 && neutral_row < rows - 2 && x1 + 1 < cols)
      mvprintw(neutral_row, x1 + 1, "free");
    break;
  }

  case BC_FF: {
    for (int rr = neutral_row - half_h - 1; rr <= neutral_row + half_h + 1;
         rr++) {
      if (rr < 1 || rr >= rows - 2)
        continue;
      if (x0 - 1 >= 0)
        mvaddch(rr, x0 - 1, '#');
      if (x1 + 1 < cols)
        mvaddch(rr, x1 + 1, '#');
    }
    break;
  }

  default:
    break;
  }

  attroff(COLOR_PAIR(CP_SUPP) | A_BOLD);
}

/*
 * Draws faint after-images of where the beam just was, like motion blur.
 * Older snapshots are drawn with sparser, fainter dots so they fade out
 * instead of smearing into a solid band. They borrow the theme's lowest
 * colour so they stay visible without using dim (which would make them
 * vanish on some themes). Does nothing while the beam is sitting still —
 * there's no motion to blur.
 */
static void render_trails(const Beam *b, int cols, int rows) {
  if (!b->dyn.active)
    return;

  int x0 = BEAM_X_MARGIN;
  int x1 = cols - PANEL_W - BEAM_X_MARGIN - 1;
  if (x1 <= x0 + 10)
    return;

  int neutral_row = rows / 2;
  int base_rows = rows / 6;
  if (base_rows < 3)
    base_rows = 3;

  const DynBeam *d = &b->dyn;

  /* Draw oldest first so the newer, brighter trails land on top. */
  for (int k = N_TRAIL; k >= 1; k--) {
    int idx = (d->trail_head - k + N_TRAIL) % N_TRAIL;
    int stride = (k <= 3) ? 3 : (k <= 7) ? 5 : 7; /* older = fewer dots */
    char gl = (k <= 3) ? '.' : (k <= 7) ? ',' : '`';

    attron(COLOR_PAIR(CP_LO));
    for (int i = 0; i < N_NODES; i += stride) {
      int c = node_col(i, x0, x1);
      if (c < 0 || c >= cols)
        continue;
      int dr =
          defl_row(d->trail_w[idx][i], b->max_deflection, base_rows, b->exag);
      int rr = neutral_row + dr;
      if (rr < 2 || rr >= rows - 2)
        continue;
      mvaddch(rr, c, gl);
    }
    attroff(COLOR_PAIR(CP_LO));
  }
}

/*
 * Draws the whole beam, back to front: first the faint trails, then a thin
 * centre line marking the unbent position, then the beam itself column by
 * column, then the load arrows and the end supports on top.
 */
static void render_beam(const Beam *b, bool show_trails, int cols, int rows) {
  int x0 = BEAM_X_MARGIN;
  int x1 = cols - PANEL_W - BEAM_X_MARGIN - 1;
  if (x1 <= x0 + 10)
    return;

  int neutral_row = rows / 2;
  int base_rows = rows / 6;
  if (base_rows < 3)
    base_rows = 3;
  int half_h = BEAM_HEIGHT / 2;

  /* Thin line marking where the beam would sit if it weren't bent. */
  attron(COLOR_PAIR(CP_DIM) | A_DIM);
  for (int c = x0; c <= x1; c++)
    mvaddch(neutral_row, c, '-');
  attroff(COLOR_PAIR(CP_DIM) | A_DIM);

  if (show_trails)
    render_trails(b, cols, rows);

  /* The beam itself, one vertical slice per point. */
  for (int i = 0; i < N_NODES; i++) {
    int c = node_col(i, x0, x1);
    if (c < 0 || c >= cols)
      continue;

    /* How hard the beam is working here, as 0..1 of the worst spot. */
    float kn = (b->max_moment > 1e-9f) ? fabsf(b->M[i]) / b->max_moment : 0.0f;

    int dr = defl_row(b->w[i], b->max_deflection, base_rows, b->exag);
    draw_curvature_column(c, neutral_row + dr, kn, half_h, rows);
  }

  draw_load_arrows(b, x0, x1, neutral_row, base_rows, half_h, cols, rows);
  draw_supports(b->bc, x0, x1, neutral_row, half_h, cols, rows);
}

/*
 * Draws the bending chart down the right side. Each row is one spot along
 * the beam, and a horizontal bar shows how much it's bending there. Bars for
 * downward bowing (smiling) point right with '>'; upward bowing (frowning)
 * points left with '<'. The two colours separate them because the two kinds
 * of bending stress different parts of the beam — and one of them is the
 * dangerous one for materials like concrete that crack easily on top.
 */
static void render_moment_panel(const Beam *b, int cols, int rows) {
  int px = cols - PANEL_W;
  int half = PANEL_W / 2 - 1;
  int zero = px + half; /* column of the zero-moment axis */
  if (rows - 4 < 3 || px < 1)
    return;

  /* Divider line between the beam and this panel. */
  attron(COLOR_PAIR(CP_DIM) | A_DIM);
  for (int r = 1; r < rows - 1; r++)
    mvaddch(r, px - 1, '|');
  attroff(COLOR_PAIR(CP_DIM) | A_DIM);

  attron(COLOR_PAIR(CP_HDR) | A_BOLD);
  mvprintw(1, px, " MOMENT  +/- ");
  attroff(COLOR_PAIR(CP_HDR) | A_BOLD);

  /* Centre line the bars grow out from — no bending means no bar. */
  attron(COLOR_PAIR(CP_DIM) | A_DIM);
  for (int r = 2; r < rows - 2; r++)
    mvaddch(r, zero, '|');
  attroff(COLOR_PAIR(CP_DIM) | A_DIM);

  if (b->max_moment < 1e-9f)
    return;

  for (int r = 2; r < rows - 2; r++) {
    /* Which point along the beam this chart row stands for. */
    int ni = (r - 2) * (N_NODES - 1) / (rows - 4);
    if (ni < 0)
      ni = 0;
    if (ni >= N_NODES)
      ni = N_NODES - 1;

    float mn = b->M[ni] / b->max_moment; /* -1..1, share of the worst spot */
    int len = (int)(fabsf(mn) * (float)(half - 1) + 0.5f);
    if (len > half - 1)
      len = half - 1;
    if (len == 0)
      continue;

    int cp = (mn >= 0.0f) ? CP_MOM_P : CP_MOM_N;
    char bch = (mn >= 0.0f) ? '>' : '<';

    attron(COLOR_PAIR(cp));
    int dir = (mn >= 0.0f) ? 1 : -1;
    for (int k = 1; k <= len; k++) {
      int bc2 = zero + dir * k;
      if (bc2 >= px && bc2 < cols)
        mvaddch(r, bc2, (chtype)bch);
    }
    attroff(COLOR_PAIR(cp));
  }

  attron(COLOR_PAIR(CP_DIM) | A_DIM);
  mvprintw(rows - 2, px, " sag+ hog- ");
  attroff(COLOR_PAIR(CP_DIM) | A_DIM);
}

/*
 * Draws the on-screen text: live status in the top-right corner, the current
 * beam and load with its numbers along the second row, and the key list
 * across the bottom. The second row is cut short so it never bleeds over the
 * bending panel's header on the right.
 */
static void render_overlay(const Beam *b, int theme, float time_scale,
                           double fps, int cols, int rows) {
  /* Top-right: load, zoom, damping, speed, theme, state, fps. */
  char top[224];
  const char *state = "running";
  if (b->paused)
    state = "PAUSED ";
  else if (b->dyn.active)
    state = "dynamic";
  snprintf(
      top, sizeof top,
      " P=%.2f  Exag=%.1fx  zeta=%.1f%%  speed=%.2fx  theme:%s  %s  %.0f fps ",
      b->P * b->load_anim, b->exag, b->damping_ratio * 100.0f, time_scale,
      themes[theme].name, state, fps);
  int top_len = (int)strlen(top);
  int top_col = cols - top_len;
  if (top_col < 0)
    top_col = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, top_col, top, cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* Second row: beam and load names plus their numbers; cut before panel. */
  char sub[160];
  snprintf(sub, sizeof sub, " %s  |  %s  |  Defl=%7.5f  M=%7.5f%s",
           bc_names[b->bc], ld_names[b->load], b->max_deflection, b->max_moment,
           b->dyn.active ? "  [VIBRATING]" : "");
  int row1_max = cols - PANEL_W - 1;
  if (row1_max < 1)
    row1_max = cols;
  attron(COLOR_PAIR(CP_HUD));
  mvaddnstr(1, 0, sub, row1_max);
  attroff(COLOR_PAIR(CP_HUD));

  /* Bottom: the key list (short version if the window is too narrow). */
  const char *hint_full =
      " q:quit  spc:pause  r:reset  n/p:BC  l:load  +/-:P  e/E:exag  "
      "d:vibrate  i:kick  z/Z:zeta  s/S:speed  g:trails  t/T:theme ";
  const char *hint_short =
      " q:quit  spc:pause  r:reset  n/p:BC  l:load  i:kick  t/T:theme ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= cols - 1)
    hint = hint_short;
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(rows - 1, 0, hint, cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §8 scene ── */

/*
 * Scene — everything the running program holds onto between frames.
 *
 * The fields fall into two groups on purpose, and the split is for the
 * reader, not the machine: things that change the beam's motion, and things
 * that only change how it looks. The test for which group a new field goes
 * in: "does it change what the beam actually does?" If yes it's a simulation
 * field; if it only affects colours or overlays it's a rendering field.
 * Keeping them apart stops a cosmetic toggle from quietly altering the
 * physics — so flipping a theme or a trail while paused leaves the beam
 * exactly where it was.
 *
 * One copy lives inside the App struct, so the signal handlers can reach the
 * whole program through a single global. Resetting with 'r' rebuilds only
 * the beam; your theme, speed, and trail choice carry over.
 */
typedef struct {
  /* ── Simulation: these change the beam's motion ── */
  Beam beam;        /* the beam itself (§4): shape, load, and wobble state */
  float time_scale; /* speed knob, 0.1x to 4x (s / S keys). Counts as
                     * physics because it stretches how fast the wobble
                     * and the load fade-in advance per real second.    */

  /* ── Rendering: these only change how it looks ── */
  int theme;        /* which colour scheme from §3 (t / T keys) */
  bool show_trails; /* draw the motion-blur trails or not (g / G). The
                     * trail snapshots are always recorded while wobbling;
                     * this flag just decides whether we draw them.      */
} Scene;

static void scene_init(Scene *s) {
  /* Only the beam is rebuilt; theme, speed, and trail choice stay put. */
  init_beam(&s->beam);
  dyn_activate(&s->beam); /* start out already wobbling */
}

static void scene_tick(Scene *s, float dt) { beam_tick(&s->beam, dt); }

static void scene_draw(const Scene *s, int cols, int rows, double fps) {
  erase();
  render_beam(&s->beam, s->show_trails, cols, rows);
  render_moment_panel(&s->beam, cols, rows);
  render_overlay(&s->beam, s->theme, s->time_scale, fps, cols, rows);
}

/* ── §9 screen ── */

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
  typeahead(-1);
  /* color_init() is called from main() once the scene's theme exists. */
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

/* ── §10 app ── */

typedef struct {
  Scene scene;
  Screen screen;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int s) {
  (void)s;
  g_app.running = 0;
}
static void on_resize_signal(int s) {
  (void)s;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static bool app_handle_key(App *app, int ch) {
  Beam *b = &app->scene.beam;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    b->paused = !b->paused;
    break;

  case 'n':
  case 'N':
    b->bc = (BCType)((b->bc + 1) % BC_COUNT);
    if (b->dyn.active) {
      dyn_activate(b); /* rebuild the patterns for the new beam, keep wobbling */
    } else {
      b->load_anim = 0.0f;
      solve_beam(b);
    }
    break;

  case 'p':
  case 'P':
    b->bc = (BCType)((b->bc + BC_COUNT - 1) % BC_COUNT);
    if (b->dyn.active) {
      dyn_activate(b);
    } else {
      b->load_anim = 0.0f;
      solve_beam(b);
    }
    break;

  case 'l':
  case 'L':
    b->load = (LDType)((b->load + 1) % LD_COUNT);
    if (b->dyn.active) {
      dyn_activate(b);
    } else {
      b->load_anim = 0.0f;
      solve_beam(b);
    }
    break;

  case 'd':
  case 'D':
    if (b->dyn.active) {
      /* Stop wobbling and settle into the resting shape. */
      b->dyn.active = false;
      b->load_anim = 1.0f;
      solve_beam(b);
    } else {
      dyn_activate(b);
    }
    break;

  case '+':
  case '=':
    b->P = (b->P * 1.25f > 100.0f) ? 100.0f : b->P * 1.25f;
    if (b->dyn.active)
      dyn_load(b);
    else
      solve_beam(b);
    break;

  case '-':
    b->P = (b->P / 1.25f < 0.05f) ? 0.05f : b->P / 1.25f;
    if (b->dyn.active)
      dyn_load(b);
    else
      solve_beam(b);
    break;

  case 'e':
    b->exag = (b->exag * 1.5f > 20.0f) ? 20.0f : b->exag * 1.5f;
    break;

  case 'E':
    b->exag = (b->exag / 1.5f < 0.5f) ? 0.5f : b->exag / 1.5f;
    break;

  case 'z':
    b->damping_ratio *= 1.4f;
    if (b->damping_ratio > 0.5f)
      b->damping_ratio = 0.5f;
    break;

  case 'Z':
    b->damping_ratio /= 1.4f;
    if (b->damping_ratio < 0.001f)
      b->damping_ratio = 0.001f;
    break;

  case 's':
    app->scene.time_scale *= 1.25f;
    if (app->scene.time_scale > 4.0f)
      app->scene.time_scale = 4.0f;
    break;

  case 'S':
    app->scene.time_scale /= 1.25f;
    if (app->scene.time_scale < 0.1f)
      app->scene.time_scale = 0.1f;
    break;

  case 'i':
  case 'I':
    if (b->dyn.active)
      dyn_impulse(b, 1.0f);
    break;

  case 'g':
  case 'G':
    app->scene.show_trails = !app->scene.show_trails;
    break;

  case 'r':
  case 'R':
    scene_init(&app->scene);
    break;

  case 't':
    app->scene.theme = (app->scene.theme + 1) % N_THEMES;
    color_init(app->scene.theme);
    break;

  case 'T':
    app->scene.theme = (app->scene.theme + N_THEMES - 1) % N_THEMES;
    color_init(app->scene.theme);
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

  screen_init(&app->screen);
  app->scene.time_scale = 1.0f;
  app->scene.show_trails = true;
  scene_init(&app->scene);
  color_init(app->scene.theme); /* depends on scene.theme being set */

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t tick_ns = NS_PER_SEC / 30;
  float dt_sec = 1.0f / 30.0f;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    if (app->need_resize) {
      screen_resize(&app->screen);
      app->need_resize = 0;
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* Measure how long the last frame took, capped so a hiccup can't
     * make the simulation try to catch up in one giant lurch. */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    /* Step the simulation in fixed-size ticks so it runs the same on any
     * machine. The speed knob stretches each tick's worth of sim time. */
    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec * app->scene.time_scale);
      sim_accum -= tick_ns;
    }

    /* Smooth the on-screen fps over half a second so it doesn't jitter. */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= 500 * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* Sleep off the rest of the frame's budget before drawing, so we hit
     * our target rate without pinning a CPU core. */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

    scene_draw(&app->scene, app->screen.cols, app->screen.rows, fps_display);

    wnoutrefresh(stdscr);
    doupdate();

    int key = getch();
    if (key != ERR && !app_handle_key(app, key))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
