/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * blackhole.c — a black hole ("Gargantua") drawn in the terminal.
 *
 * For every cell on screen we trace one ray of light BACKWARD from the
 * camera and let gravity bend its path. Where the ray ends up decides the
 * pixel: swallowed by the hole (black), through the glowing disk of gas
 * around it (bright, colour-shifted by motion), or off into the star
 * field behind. Tracing every ray is slow, so we do it once at startup
 * and remember the answer for each pixel; after that each frame is just a
 * fast table lookup plus colour.
 *
 * References (the code can't tell you these):
 *   [1] Misner, Thorne & Wheeler, *Gravitation* (1973), §25 — the bent
 *       light-path equation we use, written in plain (x,y,z) form.
 *   [2] Thorne, *The Science of Interstellar* (2014), Ch. 8–9 — the look
 *       we're imitating, and why one side of the disk glows brighter.
 *   [3] James et al. (2015), Class. Quantum Grav. 32 065001 — the same
 *       backward ray-tracing trick, scaled up for the Interstellar VFX.
 *   [4] Hamilton, *Inside Black Holes* (jila.colorado.edu/~ajsh/insidebh/)
 *       — interactive reference for the bright ring's brightness profile.
 *   [5] Ware, *Information Visualization* (2020), Ch. 4 — how to order a
 *       brightness ramp of colours/characters so the eye reads it as smooth.
 */

#define _POSIX_C_SOURCE 199309L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1  config ─────────────────────────────────────────────────────────── */

#define SIM_FPS 20
#define RENDER_FPS 60
#define ASPECT 0.47f /* how tall a terminal cell is vs wide; keeps the hole round */

/* Sizes of the black hole, measured in "hole radii" (we just call that 1).
 * Everything else is a multiple of that one unit.                         */
#define BH_R 1.0f /* the edge of no return (the event horizon)             */
#define PHOTON_R                                                               \
  1.5f /* where light can briefly circle the hole before spiralling in or   \
        * out — this is what makes the bright ring around the shadow.       */
#define DISK_IN                                                                \
  3.0f                 /* inner edge of the glowing disk: gas any closer can't \
                        * hold a stable orbit and just falls in.            */
#define DISK_OUT 12.0f /* outer edge of the disk (picked to look good).     */

/* Camera — both the tilt and the distance can be changed while running.   */
#define TILT_DEG_DEF                                                           \
  5.0f /* starting tilt above the disk; a/A change it (rebuilds the table)  */
#define TILT_DEG_MIN                                                           \
  0.0f /* looking straight along the disk: it's a thin line, but the        \
        * brightness difference between the two sides is strongest          */
#define TILT_DEG_MAX                                                           \
  85.0f                    /* almost looking down on the disk; stopped short  \
                            * of 90° because the camera math breaks there.   */
#define TILT_DEG_STEP 5.0f /* how much a/A change the tilt per press         */
#define FOV_DEG 72.0f      /* how wide the camera sees, left to right        */
#define CAM_DIST_DEF                                                           \
  24.0f /* starting camera distance — frames the whole disk and shadow with  \
         * room to spare.                                                     */
#define CAM_DIST_MIN                                                           \
  4.0f                     /* closest you can get (biggest on screen); kept   \
                            * outside the disk so the view doesn't break.    */
#define CAM_DIST_MAX 72.0f /* farthest you can get (smallest on screen)       */
#define CAM_DIST_STEP 1.5f /* how much +/- change the distance per press      */

/* disk rotation                                                            */
#define SPIN_DEF 0.04f /* how far the disk texture turns each sim step       */

/* ray tracing                                                              */
#define MAX_STEPS 900   /* give up on a ray after this many integration steps */
#define ESCAPE_R 130.0f /* once a ray gets this far out, call it escaped      */
#define DS_BASE 0.10f   /* largest step we take along a ray                   */

/* size of the precomputed lookup table (one entry per screen cell)         */
#define MAX_COLS 512
#define MAX_ROWS 256

/* Clumps are bright knots of gas drifting around in the disk. The scale
 * is tuned so a clump near the middle of the disk drifts at about the same
 * speed as the spinning texture, so the two motions look like one disk.   */
#define N_CLUMPS 10
#define CLUMP_RATE_SCALE 0.85f

/* ── §1.1 physics constants ────── */

/* The hole's "mass" in our units. Shows up wherever orbital speed does. */
#define SCHWARZSCHILD_M           0.5f

/* When a ray gets this close to the edge we know it's going to fall in,
 * so we stop a hair early — getting any closer would divide by zero. */
#define HORIZON_FUDGE             0.92f

/* How strongly motion brightens the approaching side of the disk. The
 * real physics value is bigger (3 or 4), but that washes everything out
 * to the brightest character; 1.5 keeps a readable range of brightness. */
#define DOPPLER_EXPONENT          1.5f

/* Never let the disk speed reach the speed of light in the formula —
 * it would blow up, so cap it just short. */
#define DOPPLER_BETA_CLAMP        0.95f

/* ── §1.2 render cutoffs ─────────────────────────────────────────── */

/* Dimmer than this and we just don't draw the disk pixel. */
#define DISK_MIN_VISIBLE          0.07f

/* Only rays that passed at least this close to the hole can show the
 * bright ring; farther ones never glow, so we skip the check. */
#define BLOOM_NEAR_RANGE          4.0f

/* Below this the ring glow is too faint to draw; fall back to stars
 * instead so the two never land on the same cell. */
#define BLOOM_MIN_VISIBLE         0.06f

/* ── §1.3 star field ─────────────────────────────────────────────── */

/* How finely we chop the sky into star "cells" — about one every 3°,
 * roughly 5800 cells over the whole sky. */
#define STAR_GRID_PER_RAD         30.0f

/* A sky cell gets a star only if its random value lands in this slice —
 * 4 of 256, so about 1.6% of cells have one. */
#define STAR_DENSITY_BUCKET       4u

/* Cutoffs that sort the surviving stars into bright / medium / faint. */
#define STAR_BRIGHT_THRESH      220u
#define STAR_MID_THRESH         100u

/* ── §1.4 main-loop timing ───────────────────────────────────────── */

/* If the program was paused or stalled, pretend at most this much time
 * passed, so it doesn't try to fast-forward in one huge jump. */
#define DT_CAP_NS         100000000LL

/* How often we recompute the fps number shown in the HUD (every 0.5 s). */
#define FPS_WINDOW_NS     500000000LL

/* ── §2  clock ──────────────────────────────────────────────────────────── */

static long long clock_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* ── §3  color / themes ─────────────────────────────────────────────────── */

/* Our colour slots. The first six fade from brightest to faintest and
 * change with the chosen theme; the last three never change so the
 * stars and the on-screen text stay readable no matter the theme. */
enum {
  CP_RING = 1, /* brightest: the bright ring and the white-hot inner gas */
  CP_HOT,      /* inner disk          */
  CP_WARM,     /* a bit further out   */
  CP_MID,      /* middle of the disk  */
  CP_COOL,     /* outer disk          */
  CP_DIM,      /* the faint far edge  */
  CP_STAR,     /* background stars — always light grey            */
  CP_HUD,      /* status line up top — always bright yellow, bold */
  CP_HINT,     /* key hints at the bottom — always bright cyan, bold */
  CP_COUNT
};

/* The big bundle of program state lives in §6. We name it here so the
 * colour code below can ask "does this terminal do 256 colours?" without
 * waiting for §6 to be defined first. */
struct Scene_;
typedef struct Scene_ Scene;
extern Scene g_scene;
static inline int scene_is_256_color(void);   /* defined in §6 */

/*
 * Theme — one colour scheme for the disk: a ladder of six colours from
 * brightest to faintest. Each row of g_themes[] below is one theme; the
 * 't' key steps through them and recolours the disk.
 *
 * Six colours is the sweet spot: fewer makes the fade look chunky, more
 * and the terminal can't tell neighbouring colours apart. Every colour
 * is from the bright half of the palette so even the faintest one stays
 * visible on a black terminal. They're ordered so the eye reads the fade
 * as smooth (Ware [5]). The "Blackbody" theme even matches real physics:
 * hotter inner gas glows blue-white, cooler outer gas glows red.
 */
typedef struct {
  const char *name; /* short label shown in the status line              */
  short ring;       /* brightest — the ring and the hottest gas          */
  short hot;        /* inner disk                                        */
  short warm;       /* a bit further out                                 */
  short mid;        /* middle of the disk                                */
  short cool;       /* outer disk                                        */
  short dim;        /* faintest — the disk's far edge                    */
} Theme;

static const Theme g_themes[] = {
    /*  name        ring hot  warm mid  cool dim                       */
    {"Matrix", 46, 118, 82, 40, 34, 28},         /* cyber green       */
    {"Fire", 231, 226, 220, 208, 166, 130},      /* white-hot → ember */
    {"Oceanic", 51, 87, 45, 44, 37, 31},         /* bioluminescent    */
    {"Neon", 231, 213, 177, 165, 129, 93},       /* pink / magenta    */
    {"Mono", 255, 252, 248, 244, 242, 240},      /* grayscale         */
    {"Ice", 231, 195, 159, 123, 117, 111},       /* polar white-blue  */
    {"Nova", 231, 195, 153, 111, 105, 99},       /* stellar violet    */
    {"Forest", 156, 142, 100, 94, 64, 58},       /* leaves to bark    */
    {"Desert", 230, 220, 214, 180, 136, 94},     /* sand / gold       */
    {"Eclipse", 196, 160, 124, 88, 52, 240},     /* red corona / dark */
    {"Blackbody", 153, 195, 231, 226, 214, 196}, /* physical T(r):
                                                    inner=blue-white
                                                    hot, outer=red
                                                    cool (Wien's law) */
};
#define THEME_N ((int)(sizeof g_themes / sizeof g_themes[0]))
#define THEME_DEF 10 /* start on Blackbody (the physically real colours) */

/* Star + HUD colours that never change with the theme. */
#define CHROME_STAR_256 253 /* light grey         */
#define CHROME_HUD_256 226  /* bright yellow      */
#define CHROME_HINT_256 51  /* bright cyan        */

/*
 * Backup colours for terminals stuck with only 8 colours. The six disk
 * shades collapse to a plain white-yellow-red fade.
 */
static const struct { short pair; short fg; } FALLBACK_PALETTE[] = {
    {CP_RING,  COLOR_WHITE },
    {CP_HOT,   COLOR_YELLOW},
    {CP_WARM,  COLOR_YELLOW},
    {CP_MID,   COLOR_RED   },
    {CP_COOL,  COLOR_RED   },
    {CP_DIM,   COLOR_RED   },
    {CP_STAR,  COLOR_WHITE },
    {CP_HUD,   COLOR_YELLOW},
    {CP_HINT,  COLOR_CYAN  },
};
#define FALLBACK_PALETTE_LEN \
    (int)(sizeof FALLBACK_PALETTE / sizeof FALLBACK_PALETTE[0])

/* Load the six disk colours from a theme, plus the fixed star/HUD colours. */
static void theme_apply_256(const Theme *t) {
  init_pair(CP_RING, t->ring, -1);
  init_pair(CP_HOT,  t->hot,  -1);
  init_pair(CP_WARM, t->warm, -1);
  init_pair(CP_MID,  t->mid,  -1);
  init_pair(CP_COOL, t->cool, -1);
  init_pair(CP_DIM,  t->dim,  -1);
  init_pair(CP_STAR, CHROME_STAR_256, -1);
  init_pair(CP_HUD,  CHROME_HUD_256,  -1);
  init_pair(CP_HINT, CHROME_HINT_256, -1);
}

/* Load the backup colours for low-colour terminals. */
static void theme_apply_8(void) {
  for (int i = 0; i < FALLBACK_PALETTE_LEN; i++)
    init_pair(FALLBACK_PALETTE[i].pair, FALLBACK_PALETTE[i].fg, -1);
}

static void theme_apply(int idx) {
  const Theme *t = &g_themes[idx % THEME_N];
  if (scene_is_256_color()) theme_apply_256(t);
  else                      theme_apply_8();
}

/* ── §4  V3 math ─────────────────────────────────────────────────────────── */

/*
 * V3 — an ordinary point or direction in 3-D space (x, y, z). Used for
 * everything out there: where a light ray is, which way it's pointed,
 * where the camera sits. The black hole sits at the origin.
 *
 * Worth knowing: y points "up", out of the disk. The disk lies flat like
 * a record on a turntable, in the y = 0 plane, between DISK_IN and
 * DISK_OUT. With the camera level it lies off along −z; tilting lifts it
 * up toward +y.
 */
typedef struct {
  float x; /* sideways (camera-right when level)  */
  float y; /* up, out of the disk                 */
  float z; /* depth (camera-backward when level)  */
} V3;

static inline float v3dot(V3 a, V3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3len2(V3 a) { return v3dot(a, a); }
static inline float v3len(V3 a) { return sqrtf(v3len2(a)); }
static inline V3 v3add(V3 a, V3 b) {
  return (V3){a.x + b.x, a.y + b.y, a.z + b.z};
}
static inline V3 v3sub(V3 a, V3 b) {
  return (V3){a.x - b.x, a.y - b.y, a.z - b.z};
}
static inline V3 v3scale(float s, V3 a) {
  return (V3){s * a.x, s * a.y, s * a.z};
}
static inline V3 v3cross(V3 a, V3 b) {
  return (V3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
              a.x * b.y - a.y * b.x};
}
static inline V3 v3norm(V3 a) {
  float l = v3len(a);
  return l > 1e-12f ? v3scale(1.0f / l, a) : (V3){0, 1, 0};
}

/* ── §5  bending a light ray near the hole ── */

/*
 * One tiny step of the physics: given where a ray is and where it's
 * heading, say how both change a moment later. The position just drifts
 * along the current heading; the heading gets tugged toward the hole,
 * harder the closer the ray is — and that tug is what bends the light.
 * The exact bending formula is the standard one from MTW §25 [1].
 */
static void geo_deriv(V3 pos, V3 vel, V3 *dpos, V3 *dvel) {
  *dpos = vel;
  V3 h = v3cross(pos, vel);
  float h2 = v3len2(h);
  float r2 = v3len2(pos);
  float r = sqrtf(r2);
  float coef = -1.5f * h2 / (r2 * r2 * r); /* strength of the inward tug */
  *dvel = v3scale(coef, pos);
}

/* Blend the four sampled slopes into one final slope, counting the two
 * middle ones double — the standard Runge-Kutta recipe. Pulled out so
 * the step below reads simply as "average the four slopes". */
static V3 rk4_weighted_avg(V3 k1, V3 k2, V3 k3, V3 k4) {
  V3 sum2 = v3add(v3scale(2.0f, k2), v3scale(2.0f, k3));
  V3 total = v3add(v3add(k1, k4), sum2);
  return v3scale(1.0f / 6.0f, total);
}

/*
 * Move the ray forward a short distance ds along its bent path.
 *
 * Instead of trusting the slope just at the start, we check it four
 * times — the start, twice in the middle, and the end — and move by the
 * blend of all four. That four-sample trick (Runge-Kutta) keeps a curved
 * path accurate without forcing tiny steps everywhere.
 */
static void geo_step(V3 *pos, V3 *vel, float ds) {
  V3 dp1, dv1, dp2, dv2, dp3, dv3, dp4, dv4;

  /* slope at the start */
  geo_deriv(*pos, *vel, &dp1, &dv1);

  /* slope at the midpoint, using the start's slope to guess our way there */
  V3 pa = v3add(*pos, v3scale(0.5f * ds, dp1));
  V3 va = v3add(*vel, v3scale(0.5f * ds, dv1));
  geo_deriv(pa, va, &dp2, &dv2);

  /* slope at the midpoint again, this time refined by the previous guess */
  V3 pb = v3add(*pos, v3scale(0.5f * ds, dp2));
  V3 vb = v3add(*vel, v3scale(0.5f * ds, dv2));
  geo_deriv(pb, vb, &dp3, &dv3);

  /* slope at the far end */
  V3 pc = v3add(*pos, v3scale(ds, dp3));
  V3 vc = v3add(*vel, v3scale(ds, dv3));
  geo_deriv(pc, vc, &dp4, &dv4);

  /* take the actual step along the blended slope */
  *pos = v3add(*pos, v3scale(ds, rk4_weighted_avg(dp1, dp2, dp3, dp4)));
  *vel = v3add(*vel, v3scale(ds, rk4_weighted_avg(dv1, dv2, dv3, dv4)));
}

/* ── §6  ray table / scene ───────────────────────────────────────────────── */

/*
 * RayKind — what happened to a traced ray. Every screen cell is one of
 * these three, and that's what decides how the cell is drawn.
 *
 *   R_HORIZON — fell into the hole. Drawn pure black (the shadow).
 *   R_DISK    — hit the glowing disk. We remember where on the disk it
 *               landed so we can colour it.
 *   R_ESCAPED — sailed off into the distance without hitting either. We
 *               remember which way it went (for stars behind) and how
 *               close it skimmed the hole (for the bright ring).
 */
typedef enum { R_HORIZON = 0, R_DISK, R_ESCAPED } RayKind;

/*
 * Cell — the saved answer for one screen pixel: where its ray ended up.
 * There's one Cell per pixel, and the whole table is about 3 MB.
 *
 * Why save these instead of re-tracing every frame: each ray takes up to
 * 900 little steps, and across all the visible pixels at 60 frames a
 * second that's billions of steps a second — way too slow. But the
 * bending only depends on where the camera is. So we trace every ray
 * once when the camera moves and just look the answers up after that.
 * Each frame then only does a quick lookup plus a little colour math.
 *
 * Which fields matter depends on what happened to the ray:
 *   R_HORIZON  — uses none of these; it's just black.
 *   R_DISK     — uses disk_r and disk_phi to know where on the disk it
 *                hit, so the renderer can colour it.
 *   R_ESCAPED  — uses esc_th and esc_ph (which way it flew off, for
 *                drawing a star behind it) and min_r (how close it
 *                skimmed the hole, which drives the bright ring glow).
 *
 * It's a plain struct, not a tagged union — a few wasted bytes per cell,
 * but the whole table fits comfortably in cache, so there's no reason to
 * pack it tighter.
 *
 * Related ideas (see the references up top): bending a light ray near a
 * black hole [1], tracing rays backward from the camera [3], and the
 * bright ring from rays that skim close [4].
 */
typedef struct {
  RayKind kind;
  float disk_r;   /* R_DISK: how far out on the disk it hit (hole radii) */
  float disk_phi; /* R_DISK: angle around the disk, -PI..PI              */
  float esc_th;   /* R_ESCAPED: up/down angle it flew off at, 0..PI      */
  float esc_ph;   /* R_ESCAPED: around angle it flew off at, -PI..PI     */
  float min_r;    /* closest the ray ever got to the hole — R_ESCAPED
                   * uses it to decide the bright-ring glow              */
} Cell;

/*
 * Clump — one bright knot of gas orbiting in the disk.
 *
 * Think of it as a hotter, denser blob circling the hole at a fixed
 * distance. Each tick it slides a little further around, and closer-in
 * clumps go faster (just like planets — inner ones orbit quicker). So
 * the inner clumps keep lapping the outer ones, and the disk churns and
 * shifts instead of turning as one rigid sheet.
 *
 * Grouping the three values into a struct lets the code read as plain
 * loops: "for each clump, move it around a bit" when ticking, and "for
 * each clump, add a soft glow where it is" when drawing.
 *
 * The bending comes for free: when we draw, we look up the disk spot the
 * pixel maps to, and that's already a bent position. So a clump hiding
 * behind the hole still lights up the arc of disk we see curving over the
 * top of the shadow — just like the real Interstellar imagery.
 *
 * Related ideas (see the references up top): inner gas orbits faster [1],
 * the side coming toward us looks brighter [2], and the disk flickers as
 * hot spots move [3].
 */
typedef struct {
  float r;         /* how far out it orbits (hole radii); fixed per clump,
                    * and it sets how fast the clump goes around.        */
  float phi;       /* where it currently is around the ring, 0..2PI;
                    * nudged forward a little each tick.                 */
  float intensity; /* how bright its centre glows, roughly 0..1; given a
                    * random value at start so clumps look different.    */
} Clump;

/* ── §6.1 the smaller pieces of Scene ── */

/*
 * The big Scene below is built from a few small grouped structs (here)
 * plus the two big arrays (the ray table and the clumps) and a handful
 * of flags. Each small struct gathers fields that change together, so
 * "where does this field live?" also tells you "what kind of thing is
 * it?".
 */

/*
 * Camera — where we're watching the black hole from. These two numbers
 * fully decide the view, and every ray we trace is shot from here.
 *
 * The camera always looks straight at the hole. cam_dist is how far away
 * it sits; tilt_deg is how far up off the disk's level we've lifted it.
 * build_camera_basis() in §7 turns these two numbers into the actual
 * position and the left/right/up directions of the view.
 *
 * IMPORTANT: changing either number means every ray flies differently,
 * so the whole saved table is now wrong. Whoever changes these must set
 * g_scene.need_rebuild = 1, and the next loop re-traces everything.
 *
 * We stop the tilt short of 90 degrees: looking straight down, the math
 * for "which way is right" has nothing to work with and falls apart. And
 * we keep a minimum distance so the camera never ends up sitting inside
 * the disk, where the picture would break up.
 *
 * Standard pinhole-camera setup (Foley/van Dam/Feiner/Hughes, "Computer
 * Graphics: Principles and Practice", §6.4); applied to a black hole by
 * James et al. [3].
 */
typedef struct {
  float cam_dist; /* how far the camera sits from the hole (hole radii).*
                   * Kept between 4 and 72; +/- nudge it. Smaller means *
                   * closer, so the hole fills more of the screen.      */
  float tilt_deg; /* how far the camera is lifted above the disk's      *
                   * level, in degrees. Kept between 0 and 85; a/A      *
                   * nudge it.                                          *
                   * 0  = looking along the edge (disk is a thin line,  *
                   *      but the two sides differ in brightness most). *
                   * 85 = looking almost straight down (disk is a ring  *
                   *      around the shadow — the iconic look).         */
} Camera;

/*
 * DiskSim — how the spinning disk is animated, nudged forward each tick.
 *
 * The disk moves in two ways at once. First, the whole spiral pattern
 * turns as one piece — angle creeps forward by spin each tick, like a
 * record on a turntable. Second, the bright clumps (over in Scene.clumps)
 * each drift around at their own speed, inner ones faster. The rigid spin
 * gives a steady turning look; the clumps break it up so it reads as real
 * churning gas, not a flat printed disk.
 *
 * spin is its own knob rather than something we calculate, because how
 * fast the disk "feels" is a taste call. The default gives a comfortable
 * pace. It's counted per tick, not per second, because the simulation
 * always takes the same fixed-size steps (see FrameTimer).
 *
 * angle and paused are free to change anytime — no re-tracing needed.
 * spin is too, but nothing currently changes it at runtime; the field is
 * here so adding a "speed up / slow down" key would be a one-liner.
 *
 * Related ideas (see the references up top): inner gas orbits faster [1],
 * the disk flickers as hot spots move [3], and the spiral pattern is
 * standard accretion-disk astrophysics (Pringle 1981).
 */
typedef struct {
  float spin;   /* how far the whole disk pattern turns each tick.      *
                 * Also doubles as the reference the clump speeds are   *
                 * scaled against, so the two motions roughly agree at  *
                 * the middle of the disk.                              */
  float angle;  /* how far the disk has turned so far, wrapping at one  *
                 * full turn. Advanced by spin each tick unless paused; *
                 * the 'r' key snaps it back to 0.                      */
  int   paused; /* 1 freezes everything — the disk stops turning and    *
                 * the clumps stop moving. The status line shows PAUSED.*
                 * Space or 'p' toggles it.                             */
} DiskSim;

/*
 * RenderConfig — the look-only knobs. These only change colours and
 * what's drawn on top; they never change where the rays went or where
 * the clumps are. That's the whole point of keeping them separate from
 * Camera and DiskSim: nothing here can ever force a re-trace.
 *
 * Rule of thumb if you add a flag: if it would change what the rays do
 * or how the gas moves, it belongs in Camera or DiskSim — not here. Only
 * things that change colour or what's overlaid go here.
 *
 * Stars start off because they clutter the wide edges of the view and
 * pull the eye away from the ring; press 'k' to turn them on and see how
 * the hole magnifies the sky behind it.
 */
typedef struct {
  int theme;      /* which colour scheme is active (row of g_themes[]). *
                   * 't' cycles through them. Starts on Blackbody.      */
  int show_stars; /* 1 draws the bent background starfield, 0 leaves it *
                   * off (the default). 'k' toggles it.                 */
} RenderConfig;

/*
 * Screen — how big the terminal is right now, in character cells. This
 * is the one place that answer lives; the trace and the renderer both
 * read it, and they never look past MAX_ROWS/MAX_COLS so the table is
 * always indexed safely even on a huge terminal.
 *
 * It only changes at startup and when the window is resized — never in
 * the middle of a frame. A resize means every pixel now points a
 * different way, so it kicks off a full re-trace of the table.
 */
typedef struct {
  int cols; /* terminal width in cells  */
  int rows; /* terminal height in cells */
} Screen;

/*
 * FrameTimer — all the timing bookkeeping for the main loop, kept in one
 * place so the loop reads simply as "measure time → step the sim → draw →
 * sleep".
 *
 * There are two clocks running. The drawing clock paints one frame, then
 * sleeps off whatever's left of the frame's time budget so we hit a steady
 * frame rate. The simulation clock takes fixed-size steps: we pile up the
 * real time that passed in sim_accum, and every time it adds up to one
 * step's worth we advance the disk once. Keeping the two separate means
 * the disk turns at the same real-world pace whether the drawing is fast
 * or slow — on a slow machine it just does a few catch-up steps. This is
 * Glenn Fiedler's "Fix Your Timestep!" pattern
 * (https://gafferongames.com/post/fix_your_timestep/).
 *
 * The fps number is averaged over the last half-second; a raw per-frame
 * number jumps around too much to read.
 */
typedef struct {
  long long tick_ns;    /* time budget for one sim step (nanoseconds),  *
                         * set once at startup.                         */
  long long frame_ns;   /* time budget for one drawn frame, set once.   */
  long long sim_accum;  /* leftover time piling up toward the next sim  *
                         * step; reset on resize / re-trace.            */
  long long frame_time; /* when the previous frame started, so we can   *
                         * tell how long it's been since.               */
  long long fps_acc;    /* time counted so far in the current           *
                         * half-second fps window.                      */
  int       fps_cnt;    /* frames counted in that window.               */
  float     fps;        /* the averaged fps shown in the status line.   */
} FrameTimer;

/* ── §6.2 Scene — the one big bundle of state ── */

/*
 * Scene — everything the program keeps around for its whole run, in one
 * place: the camera, the disk, the look settings, the screen size, the
 * timer, the big ray table, the clumps, and a few flags. There's exactly
 * one of these (g_scene), and everyone reaches into it directly.
 *
 * Why one global and not a pointer passed everywhere: the ray table alone
 * is about 3 MB, and a single global also gives the signal handlers an
 * easy way to flip the "quit" and "resize" flags. Everything else is just
 * small numbers.
 *
 * A good reading order, outside-in: Camera (what the table was traced
 * from) → DiskSim (what moves each tick) → RenderConfig (look only) →
 * Screen → FrameTimer → the table → the clumps → the flags.
 *
 * The key thing to remember: changing the Camera (with +/- or a/A) makes
 * the saved table wrong, so those keys ask for a re-trace. The look
 * settings never do. At startup main() fills this in a fixed order:
 * ncurses up, colour check, theme, clumps, then trace the table.
 */
struct Scene_ {
  /* The grouped sub-pieces, documented in §6.1 above. */
  Camera       camera;       /* where we're watching from — changing it  *
                              * forces a re-trace (sets need_rebuild).    */
  DiskSim      disk;         /* the disk's animation and pause state.     */
  RenderConfig render;       /* look-only settings — never a re-trace.    */
  Screen       screen;       /* current terminal size.                    */
  FrameTimer   timer;        /* loop pacing and the fps number.           */

  /* The two big arrays sit right here at the top level so indexing a
   * pixel reads as a plain g_scene.table[r][c]. */
  Cell  table[MAX_ROWS][MAX_COLS];   /* the saved answer for every pixel; *
                                      * filled by precompute(), read each *
                                      * frame. About 3 MB.                */
  Clump clumps[N_CLUMPS];            /* the orbiting bright knots; seeded *
                                      * once and nudged each tick.        */

  /* A few simple flags. */
  int  color256;     /* 1 if the terminal does 256 colours (checked once).*/
  int  need_rebuild; /* set when the camera changed, so the next loop     *
                      * pass re-traces the table.                         */
  volatile sig_atomic_t running;     /* the loop runs while this is 1;    *
                                      * 'q'/ESC and the quit signals clear*
                                      * it. Marked special so a signal    *
                                      * handler can safely touch it.      */
  volatile sig_atomic_t need_resize; /* the resize signal sets this; the  *
                                      * loop notices, re-reads the size,  *
                                      * and re-traces. Same signal-safety.*/
};

Scene g_scene = {
    .camera  = { .cam_dist = CAM_DIST_DEF, .tilt_deg = TILT_DEG_DEF },
    .disk    = { .spin     = SPIN_DEF,     .angle    = 0.0f,
                 .paused   = 0 },
    .render  = { .theme    = THEME_DEF,    .show_stars = 0 },
    .running = 1,
    /* everything else starts at zero and gets filled in before it's read. */
};

static inline int scene_is_256_color(void) { return g_scene.color256; }

/*
 * The next few small helpers are the questions ray_trace asks at each
 * step — has the ray fallen in? has it escaped? did it cross the disk? —
 * so the trace loop itself reads as that plain checklist.
 */

/* How far to step next: take small steps near the hole where the path
 * bends sharply, bigger steps far away where it's nearly straight. Never
 * smaller than a floor or bigger than DS_BASE. */
static float adaptive_geodesic_step_size(float r) {
  return fmaxf(0.003f, fminf(DS_BASE, r * 0.05f));
}

/* Has the ray fallen into the hole? We stop a hair before the true edge:
 * once it's this close it's doomed anyway, and going closer divides by
 * zero in the math. */
static int fell_through_horizon(float r) { return r < BH_R * HORIZON_FUDGE; }

/* Has the ray gotten far enough away that gravity barely touches it? Out
 * here its direction won't change anymore, so we can stop and note which
 * way it's headed. */
static int escaped_to_far_field(float r) { return r > ESCAPE_R; }

/* Did the ray just pass through the disk's flat level? We notice it by
 * its up/down position flipping sign between two steps. This catches the
 * disk both in front of the hole and the far side bent into view over the
 * top of the shadow. */
static int crossed_equatorial_plane(V3 prev, V3 pos) {
  return prev.y * pos.y < 0.0f;
}

/* The two steps straddle the disk's level; find the exact spot in between
 * where it crossed, by blending the two by how close each was to the
 * level. That gives us where on the disk it hit. */
static V3 interpolate_disk_plane_hit(V3 prev, V3 pos) {
  float t = fabsf(prev.y) / (fabsf(prev.y) + fabsf(pos.y));
  return v3add(v3scale(1.f - t, prev), v3scale(t, pos));
}

/* The ray escaped — turn its final heading into a direction on the sky
 * (an up/down angle and an around angle) and save it. Used both for rays
 * that flew off cleanly and for the few that just ran out of steps while
 * circling. */
static Cell record_far_field_direction(V3 vel, float min_r) {
  V3 d = v3norm(vel);
  float th = acosf(fmaxf(-1.f, fminf(1.f, d.y)));
  float ph = atan2f(d.z, d.x);
  return (Cell){R_ESCAPED, 0, 0, th, ph, min_r};
}

/*
 * ray_trace — follow one ray backward from the camera, step by step, until
 * it either falls into the hole, gets far enough away to call it escaped,
 * or crosses the glowing disk. Returns the saved answer for that pixel.
 *
 * The loop is just the checklist: how far am I, did I fall in, did I get
 * away, take a step, did I cross the disk. If it somehow keeps circling
 * without ending, we give up after a step cap and note its heading anyway.
 */
static Cell ray_trace(V3 origin, V3 dir) {
  V3 pos = origin;
  V3 vel = dir;
  V3 prev = pos;
  float min_r = v3len(origin); /* remember how close it ever got, for the glow */

  for (int step = 0; step < MAX_STEPS; step++) {
    float r = v3len(pos);
    if (r < min_r)
      min_r = r;

    if (fell_through_horizon(r))
      return (Cell){R_HORIZON, 0, 0, 0, 0, min_r};
    if (escaped_to_far_field(r))
      return record_far_field_direction(vel, min_r);

    float ds = adaptive_geodesic_step_size(r);
    prev = pos;
    geo_step(&pos, &vel, ds);

    if (crossed_equatorial_plane(prev, pos)) {
      V3 hit = interpolate_disk_plane_hit(prev, pos);
      float cr = sqrtf(hit.x * hit.x + hit.z * hit.z);
      if (cr >= DISK_IN && cr <= DISK_OUT) {
        float ph = atan2f(hit.z, hit.x);
        return (Cell){R_DISK, cr, ph, 0, 0, min_r};
      }
    }
  }

  return record_far_field_direction(vel, min_r);
}

/* ── §7  precompute lensing table ────────────────────────────────────────── */

/*
 * The next few helpers set up the camera, turn each pixel into a ray to
 * shoot, and draw the little progress message while the table is built
 * (it takes about half a second).
 */

/*
 * build_camera_basis — work out the camera's position and the three
 * directions of its view (forward, right, up) from just how far away it is
 * and how much it's tilted. It always looks straight at the hole.
 *
 * "Right" is found by combining forward with straight-up. That's why we
 * never tilt all the way to looking straight down: there, forward and
 * straight-up line up and there's no "right" to find.
 */
static void build_camera_basis(float cam_dist, float tilt_rad, V3 *cam, V3 *fwd,
                               V3 *rgt, V3 *up) {
  *cam = (V3){0.f, cam_dist * sinf(tilt_rad), -cam_dist * cosf(tilt_rad)};
  *fwd = v3norm(v3scale(-1.f, *cam));
  V3 world_up = {0.f, 1.f, 0.f};
  *rgt = v3norm(v3cross(*fwd, world_up));
  *up = v3cross(*rgt, *fwd);
}

/*
 * pinhole_pixel_to_ray — turn one screen pixel into the direction of the
 * ray that should be shot through it. Pixels left of centre aim a little
 * left, above centre aim a little up, and so on, fanning out from the
 * forward direction by how wide the camera sees.
 *
 * We measure both axes against the screen's half-width so the picture
 * isn't squashed, then ASPECT corrects for terminal cells being about
 * twice as tall as they are wide. Standard pinhole-camera projection.
 */
static V3 pinhole_pixel_to_ray(int col, int row, float cx, float cy, float hw,
                               V3 fwd, V3 rgt, V3 up) {
  float u = (col - cx) / cx * hw;
  float v = -(row - cy) / cx * hw / ASPECT;
  return v3norm(v3add(fwd, v3add(v3scale(u, rgt), v3scale(v, up))));
}

/* Shows a "building…" message before the trace starts, so the brief
 * freeze (about half a second) clearly looks intentional, not a hang. */
static void draw_progress_header(int cols, int rows) {
  attron(A_BOLD);
  mvprintw(rows / 2, cols / 2 - 18, "  Building lensing table …          ");
  mvprintw(rows / 2 + 1, cols / 2 - 18, "  (exact Schwarzschild geodesics)   ");
  attroff(A_BOLD);
  wnoutrefresh(stdscr);
  doupdate();
}

/* Updates the "[NN%]" counter every few rows so the user sees it ticking
 * up instead of staring at a frozen message. */
static void draw_progress_percent(int rows_done, int rows_total, int cols,
                                  int rows) {
  int pct = rows_done * 100 / rows_total;
  mvprintw(rows / 2 + 2, cols / 2 - 10, "  [%3d%%] ", pct);
  wnoutrefresh(stdscr);
  doupdate();
}

/*
 * precompute — fill the whole table: set up the camera, then for every
 * visible pixel shoot one ray and save where it ended up. This is the
 * slow part, done only when the camera moves or the window resizes.
 */
static void precompute(int cols, int rows, float cam_dist, float tilt_deg) {
  float tilt_rad = tilt_deg * (float)M_PI / 180.0f;
  float hw = tanf((FOV_DEG * (float)M_PI / 180.0f) * 0.5f);
  float cx = cols * 0.5f;
  float cy = rows * 0.5f;

  V3 cam, fwd, rgt, up;
  build_camera_basis(cam_dist, tilt_rad, &cam, &fwd, &rgt, &up);

  draw_progress_header(cols, rows);

  int rows_lim = rows < MAX_ROWS ? rows : MAX_ROWS;
  int cols_lim = cols < MAX_COLS ? cols : MAX_COLS;

  for (int row = 0; row < rows_lim; row++) {
    for (int col = 0; col < cols_lim; col++) {
      V3 dir = pinhole_pixel_to_ray(col, row, cx, cy, hw, fwd, rgt, up);
      g_scene.table[row][col] = ray_trace(cam, dir);
    }
    if (row % 6 == 0)
      draw_progress_percent(row, rows_lim, cols, rows);
  }
}

/* ── §8  frame render ────────────────────────────────────────────────────── */

/* The Clump type and the storage for them live back in §6 with the rest
 * of the data. The two helpers here set them up and move them. */

static float clumps_frand(void) { return (float)rand() / (float)RAND_MAX; }

/* Scatter the clumps randomly across the disk — random distance, random
 * angle, slightly random brightness so they don't all look the same. Run
 * at startup and again on the 'r' key. */
static void clumps_init(void) {
  for (int i = 0; i < N_CLUMPS; i++) {
    g_scene.clumps[i].r = DISK_IN + (DISK_OUT - DISK_IN) * clumps_frand();
    g_scene.clumps[i].phi = clumps_frand() * 2.0f * (float)M_PI;
    g_scene.clumps[i].intensity =
        0.22f + 0.20f * clumps_frand(); /* 0.22–0.42 */
  }
}

/*
 * clumps_tick — move each clump a little further around the ring. The
 * speed comes from how far out it orbits: just like planets, the inner
 * clumps go faster. The innermost ones lap the outer ones several times
 * over, which is what makes the disk look like it's churning.
 */
static void clumps_tick(void) {
  for (int i = 0; i < N_CLUMPS; i++) {
    float r = g_scene.clumps[i].r;
    float omega = sqrtf(SCHWARZSCHILD_M / (r * r * r));  /* orbital speed at this radius */
    g_scene.clumps[i].phi += omega * CLUMP_RATE_SCALE;
    if (g_scene.clumps[i].phi >= (float)(2.0 * M_PI))
      g_scene.clumps[i].phi -= (float)(2.0 * M_PI);
  }
}

static float fclamp(float v, float lo, float hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

/*
 * The next few helpers paint the background stars. An escaped ray knows
 * which way it flew off — and traced backward, that's the patch of sky
 * behind the hole that the pixel is looking at. So we just ask "is there
 * a star in that direction?".
 *
 * We chop the whole sky into a grid of small patches (about 3 degrees
 * each), and a fixed scramble of each patch's grid position decides two
 * things, the same way every frame: whether that patch has a star at all
 * (only about one in sixty does), and if so how bright it is.
 *
 * The magnifying comes for free: rays that skim the hole fly off in wildly
 * spread-out directions, so a small patch of sky — even stars right behind
 * the hole — gets smeared into a bright ring just outside the shadow.
 */

/* Scramble a sky patch's grid position into a fixed random-looking number.
 * Same patch always gives the same number, so each star stays put frame to
 * frame instead of flickering around. */
static unsigned int sky_cell_hash(unsigned int q_th, unsigned int q_ph) {
  unsigned int h = q_th * 0x9E3779B9u + q_ph * 0xCC9E2D51u;
  h ^= h >> 16;
  h *= 0x85EBCA6Bu;
  h ^= h >> 13;
  h *= 0xC2B2AE35u;
  h ^= h >> 16;
  return h;
}

/* Snap a sky direction to which grid patch it falls in. That patch is both
 * what we scramble and where a star is pinned. */
static void sky_quantise_direction(float th, float ph,
                                   unsigned int *q_th, unsigned int *q_ph) {
  *q_th = (unsigned int)(th * STAR_GRID_PER_RAD);
  *q_ph = (unsigned int)((ph + (float)M_PI) * STAR_GRID_PER_RAD);
}

/* Pick how bright this star looks, from part of its scramble number.
 * Stars come in three sizes — bright '*', medium '+', faint '.' — with the
 * brightest the rarest. */
static void classify_star_brightness(unsigned int hash,
                                     char *glyph, attr_t *attr) {
  unsigned int b = (hash >> 8) & 0xFFu;
  if (b > STAR_BRIGHT_THRESH) { *glyph = '*'; *attr = A_BOLD;   }
  else if (b > STAR_MID_THRESH) { *glyph = '+'; *attr = A_NORMAL; }
  else                          { *glyph = '.'; *attr = A_NORMAL; }
}

/* Is there a star in this direction? If so, fills in its glyph and look
 * and returns 1; otherwise returns 0. */
static int star_lookup(float th, float ph, char *glyph, attr_t *attr) {
  unsigned int q_th, q_ph;
  sky_quantise_direction(th, ph, &q_th, &q_ph);

  unsigned int h = sky_cell_hash(q_th, q_ph);

  /* most patches are empty sky */
  if ((h & 0xFFu) > STAR_DENSITY_BUCKET) return 0;

  classify_star_brightness(h, glyph, attr);
  return 1;
}

/*
 * Brightness-to-character ladder for the disk. The brighter the spot, the
 * heavier the character we use — from solid '@' down to a faint '.'. This
 * lets the disk show a smooth gradient even on terminals with few colours.
 * The order follows Ware (2020) so the eye reads it as steadily fading.
 */
static const struct { float thresh; char glyph; } DISK_GLYPH_RAMP[] = {
    {0.92f, '@'}, {0.82f, '#'}, {0.70f, '8'},
    {0.57f, '0'}, {0.45f, 'O'}, {0.33f, 'o'},
    {0.21f, '+'}, {0.12f, ':'},
};
#define DISK_GLYPH_TIERS  (int)(sizeof DISK_GLYPH_RAMP / sizeof DISK_GLYPH_RAMP[0])

static char disk_char(float b) {
  for (int i = 0; i < DISK_GLYPH_TIERS; i++)
    if (b > DISK_GLYPH_RAMP[i].thresh) return DISK_GLYPH_RAMP[i].glyph;
  return '.';
}

/*
 * Brightness-to-colour ladder for the disk, matching the six theme
 * colours from brightest to faintest. The very brightest tier is bolded,
 * and the next tier is bolded too but only in the inner quarter of the
 * disk, so the hot ring right around the shadow stands out from the gas
 * further out.
 */
#define INNER_DISK_BOLD_FRAC  0.25f

static const struct { float thresh; int cp; } DISK_PAIR_RAMP[] = {
    {0.85f, CP_RING}, {0.67f, CP_HOT},  {0.50f, CP_WARM},
    {0.33f, CP_MID},  {0.17f, CP_COOL},
};
#define DISK_PAIR_TIERS  (int)(sizeof DISK_PAIR_RAMP / sizeof DISK_PAIR_RAMP[0])

static void disk_pair(float b, float r_norm, int *cp, attr_t *a) {
  for (int i = 0; i < DISK_PAIR_TIERS; i++) {
    if (b > DISK_PAIR_RAMP[i].thresh) {
      *cp = DISK_PAIR_RAMP[i].cp;
      *a  = (i == 0) ? A_BOLD
                     : (i == 1 && r_norm < INNER_DISK_BOLD_FRAC) ? A_BOLD
                                                                  : A_NORMAL;
      return;
    }
  }
  *cp = CP_DIM;
  *a  = A_NORMAL;  /* no A_DIM — would vanish on default background */
}

/*
 * The next handful of helpers each work out one thing that makes a disk
 * spot brighter or dimmer; render_disk_cell multiplies them together. The
 * ideas behind them are in the references up top: the side coming toward
 * us looks brighter and light climbing out of the hole looks dimmer [1,2],
 * the inner disk is hotter [2], the bright ring [4], moving hot spots [3],
 * and ordering brightness so the eye reads it smoothly [5].
 */

/*
 * keplerian_doppler_factor — the side of the disk spinning toward us looks
 * brighter, the side spinning away looks dimmer (like a siren rising in
 * pitch as it comes at you, but for brightness). Returns how much to
 * brighten or dim this spot. We dial the effect down a bit from the true
 * physics so the brightest side doesn't wash out to a flat wall of '@'.
 */
static float keplerian_doppler_factor(float disk_r, float phi_rotating,
                                      float cos_tilt) {
  float v_orb = sqrtf(SCHWARZSCHILD_M / disk_r);
  float beta  = -v_orb * cosf(phi_rotating) * cos_tilt;
  beta = fclamp(beta, -DOPPLER_BETA_CLAMP, DOPPLER_BETA_CLAMP);
  return powf((1.f + beta) / (1.f - beta), DOPPLER_EXPONENT);
}

/*
 * gravitational_redshift — light has to climb out of the hole's gravity to
 * reach us, and that climb saps its energy, so it arrives dimmer. Gas right
 * at the edge dims to nothing; gas far out is barely affected. Returns the
 * dimming factor (between 0 and 1).
 */
static float gravitational_redshift(float disk_r) {
  /* floor guards against a square root of a negative if a spot ever sneaks in too close */
  return sqrtf(fmaxf(0.01f, 1.f - 1.f / disk_r));
}

/*
 * disk_radial_temperature — the disk is hottest (brightest) near the
 * inside and cools toward the outside. On top of that steady fade there's
 * an extra-bright lip right at the inner edge, where gas crowds together
 * just before it falls in — the brightest part of the whole disk. Returns
 * a brightness factor.
 */
static float disk_radial_temperature(float disk_r) {
  float r_n = fclamp((disk_r - DISK_IN) / (DISK_OUT - DISK_IN), 0.f, 1.f);
  float dr = disk_r - DISK_IN;
  float isco = expf(-dr * dr * 0.65f);
  return powf(1.f - 0.86f * r_n, 2.2f) + 0.65f * isco;
}

/*
 * spiral_density_texture — lays a faint spiral pattern over the disk that
 * turns slowly with it, so the disk reads as having swirling structure
 * rather than being a flat smooth ring. Just brightens and dims spots by a
 * small amount. */
static float spiral_density_texture(float disk_phi, float disk_angle) {
  return 1.f + 0.18f * sinf(disk_phi * 5.f - disk_angle * 4.f);
}

/*
 * accumulate_clump_bumps — for this disk spot, add up the glow from any
 * nearby clumps. Each clump is a soft blob of extra brightness that fades
 * out with distance, so a spot right on a clump gets a big boost and one
 * far from all of them gets none. The early skip means spots away from
 * every clump cost almost nothing to check.
 */
static float accumulate_clump_bumps(float disk_r, float disk_phi) {
  float bump = 0.0f;
  for (int k = 0; k < N_CLUMPS; k++) {
    float dphi = disk_phi - g_scene.clumps[k].phi;
    if (dphi > (float)M_PI)
      dphi -= (float)(2.0 * M_PI);
    else if (dphi < -(float)M_PI)
      dphi += (float)(2.0 * M_PI);
    float dr = disk_r - g_scene.clumps[k].r;
    float a = dphi * dphi * 30.0f + dr * dr * 1.6f;
    if (a < 6.0f)
      bump += g_scene.clumps[k].intensity * expf(-a);
  }
  return bump;
}

/* Draw one disk spot: turn its brightness into a character and a colour
 * and put it on screen. */
static void paint_disk_pixel(int row, int col, float brightness, float r_norm) {
  int cp;
  attr_t a;
  disk_pair(brightness, r_norm, &cp, &a);
  attron(COLOR_PAIR(cp) | a);
  mvaddch(row, col, (chtype)(unsigned char)disk_char(brightness));
  attroff(COLOR_PAIR(cp) | a);
}

/*
 * photon_ring_bloom — how bright the glowing ring is for a ray that skimmed
 * the hole, based on how close it got. It's three glows added together: a
 * broad halo, a sharp spike for rays that nearly grazed the edge, and a
 * couple of faint echo-rings from light that looped around once or twice
 * before getting out. Returns brightness between 0 and 1.
 */
static float photon_ring_bloom(float min_r) {
  float main = expf(-(min_r - PHOTON_R) * 1.4f);              /* broad halo */

  float rim = (min_r < 1.20f) ? 0.55f * expf(-(min_r - 1.0f) * 10.0f) : 0.0f;
                                                              /* edge-grazing spike */

  float e1 = min_r - (PHOTON_R + 0.06f);                      /* faint echo rings */
  float e2 = min_r - (PHOTON_R + 0.02f);
  float echo =
      0.40f * expf(-e1 * e1 * 250.0f) + 0.30f * expf(-e2 * e2 * 700.0f);

  return fclamp(main + rim + echo, 0.0f, 1.0f);
}

/*
 * Brightness-to-look ladder for the glowing ring. Each row says: if the
 * glow is at least this bright, draw it with this character and colour.
 * Brightest is a bold '#', fading down to a plain '.' at the edge.
 */
static const struct { float thresh; char ch; int cp; attr_t a; }
    RING_RAMP[] = {
        {0.85f, '#', CP_RING, A_BOLD  },
        {0.55f, '*', CP_RING, A_BOLD  },
        {0.30f, '+', CP_HOT,  A_NORMAL},
    };
#define RING_RAMP_TIERS  (int)(sizeof RING_RAMP / sizeof RING_RAMP[0])

static void paint_photon_ring_pixel(int row, int col, float brightness) {
  char   ch = '.';
  int    cp = CP_WARM;
  attr_t a  = A_NORMAL;
  for (int i = 0; i < RING_RAMP_TIERS; i++) {
    if (brightness > RING_RAMP[i].thresh) {
      ch = RING_RAMP[i].ch; cp = RING_RAMP[i].cp; a = RING_RAMP[i].a;
      break;
    }
  }
  attron (COLOR_PAIR(cp) | a);
  mvaddch(row, col, (chtype)(unsigned char)ch);
  attroff(COLOR_PAIR(cp) | a);
}

/* If there's a star in the direction this pixel sees, draw it. Only called
 * when the ring glow didn't already paint here, so a star and the ring
 * never land on the same cell. */
static void paint_lensed_star(int row, int col, float esc_th, float esc_ph) {
  char glyph;
  attr_t a;
  if (star_lookup(esc_th, esc_ph, &glyph, &a)) {
    attron(COLOR_PAIR(CP_STAR) | a);
    mvaddch(row, col, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(CP_STAR) | a);
  }
}

/*
 * render_disk_cell — colour one pixel that hit the disk. Combine all the
 * brightness effects — the approaching-side glow, the climb-out dimming,
 * the hotter-inside fade, the spiral pattern, and any nearby clumps — into
 * one final brightness, and draw it (skipping spots too faint to bother).
 */
static void render_disk_cell(const Cell *c, float disk_angle, float cos_tilt,
                             int row, int col) {
  float phi_rot = c->disk_phi + disk_angle;
  float D = keplerian_doppler_factor(c->disk_r, phi_rot, cos_tilt);
  float g = gravitational_redshift(c->disk_r);
  float rad = disk_radial_temperature(c->disk_r);
  float tex = spiral_density_texture(c->disk_phi, disk_angle);
  float bright = fclamp(D * g * rad * tex, 0.f, 1.f);

  bright =
      fclamp(bright + accumulate_clump_bumps(c->disk_r, c->disk_phi), 0.f, 1.f);
  if (bright < DISK_MIN_VISIBLE)
    return; /* trim dim outer-disk scatter */

  float r_norm = fclamp((c->disk_r - DISK_IN) / (DISK_OUT - DISK_IN), 0.f, 1.f);
  paint_disk_pixel(row, col, bright, r_norm);
}

/*
 * render_escaped_cell — colour one pixel whose ray flew off into the
 * distance. If the ray skimmed the hole closely enough to glow, draw the
 * ring here. Otherwise, if stars are turned on, draw whatever star sits in
 * the direction this pixel sees.
 */
static void render_escaped_cell(const Cell *c, int row, int col,
                                int show_stars) {
  if (c->min_r < BLOOM_NEAR_RANGE) {
    float rb = photon_ring_bloom(c->min_r);
    if (rb > BLOOM_MIN_VISIBLE) {
      paint_photon_ring_pixel(row, col, rb);
      return;
    }
  }
  if (show_stars)
    paint_lensed_star(row, col, c->esc_th, c->esc_ph);
}

/*
 * render — draw one whole frame. Walk every pixel inside the visible
 * region and colour it based on what its ray did: leave it black if it fell
 * in, colour the disk if it hit the disk, or do the ring/star if it escaped.
 */
static void render(float disk_angle, int cols, int rows, float cam_dist,
                   float tilt_deg, int show_stars) {
  float cos_tilt = cosf(tilt_deg * (float)M_PI / 180.0f);

  int rows_lim = rows < MAX_ROWS ? rows : MAX_ROWS;
  int cols_lim = cols < MAX_COLS ? cols : MAX_COLS;

  /*
   * Skip pixels too far from the centre to ever show anything — there's no
   * point checking the empty corners. We work out how big a circle the disk
   * could fill (bigger when the camera is closer) and only look inside it.
   * The 1.24 nudges the edge out to match the actual disk; the 0.96 cap
   * keeps the circle on screen.
   */
  float cx = (float)cols * 0.5f;
  float cy = (float)rows * 0.5f;
  float fov_h_tan = tanf(FOV_DEG * (float)M_PI / 360.0f);
  float clip_frac = fminf((DISK_OUT / cam_dist) / fov_h_tan * 1.24f, 0.96f);
  float clip_r2 = (cx * clip_frac) * (cx * clip_frac);

  for (int row = 0; row < rows_lim - 1; row++) {
    for (int col = 0; col < cols_lim; col++) {
      float sdx = (float)col - cx;
      float sdy = ((float)row - cy) / ASPECT;
      if (sdx * sdx + sdy * sdy > clip_r2)
        continue;

      const Cell *c = &g_scene.table[row][col];
      switch (c->kind) {
      case R_HORIZON:
        /* shadow — black, erase() already cleared the cell */
        break;
      case R_DISK:
        render_disk_cell(c, disk_angle, cos_tilt, row, col);
        break;
      case R_ESCAPED:
        render_escaped_cell(c, row, col, show_stars);
        break;
      }
    }
  }
}

/* ── §9  screen / HUD ───────────────────────────────────────────────────── */

/* These run when the OS interrupts us — a quit request or a window resize.
 * They only flip one simple flag, which is all that's safe to touch from
 * inside a signal handler. */
static void on_sigint(int s) {
  (void)s;
  g_scene.running = 0;
}
static void on_sigwinch(int s) {
  (void)s;
  g_scene.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void screen_init(Screen *s) {
  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  typeahead(-1);
  getmaxyx(stdscr, s->rows, s->cols);
}

/* Draws the two text bars: live status across the top-right (distance,
 * tilt, theme, paused, fps) and the list of keys along the bottom. */
static void screen_hud(int cols, int rows, float fps, float cam_dist,
                       float tilt_deg, int theme, int paused) {
  /* top-right status */
  char top[180];
  snprintf(top, sizeof top, " dist:%.0f  tilt:%.0f  theme:%s  %s  %.0f fps ",
           (double)cam_dist, (double)tilt_deg, g_themes[theme % THEME_N].name,
           paused ? "PAUSED " : "running", (double)fps);
  int top_len = (int)strlen(top);
  int top_col = cols - top_len;
  if (top_col < 0)
    top_col = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, top_col, top, cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* bottom row — the key list */
  const char *hint =
      " q:quit  p:pause  r:reset  t:theme  +/-:dist  a/A:tilt  k:stars ";
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(rows - 1, 0, hint, cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §10  main ───────────────────────────────────────────────────────────── */

/* ── §10.1 setup and the per-frame loop helpers ── */

/* Gets everything ready once before the loop starts: signal handlers,
 * ncurses, the colour palette, the clumps, the first traced table, and the
 * timer. */
static void scene_setup(void) {
  srand((unsigned)time(NULL));
  atexit(cleanup);
  signal(SIGINT,   on_sigint);
  signal(SIGWINCH, on_sigwinch);

  /* this order matters: each step needs the one before it */
  screen_init(&g_scene.screen);
  start_color();
  use_default_colors();
  g_scene.color256 = (COLORS >= 256);

  theme_apply(g_scene.render.theme);
  clumps_init();
  erase();
  precompute(g_scene.screen.cols, g_scene.screen.rows,
             g_scene.camera.cam_dist, g_scene.camera.tilt_deg);

  /* seed the timer budgets (the rest start at zero) */
  g_scene.timer.tick_ns    = 1000000000LL / SIM_FPS;
  g_scene.timer.frame_ns   = 1000000000LL / RENDER_FPS;
  g_scene.timer.frame_time = clock_ns();
}

/* A window resize and a camera change both need the same thing: re-read the
 * terminal size and trace the whole table again. Only happens when the user
 * does something, and takes under a second. */
static void scene_handle_resize_or_rebuild(void) {
  g_scene.need_resize  = 0;
  g_scene.need_rebuild = 0;
  endwin();
  refresh();
  getmaxyx(stdscr, g_scene.screen.rows, g_scene.screen.cols);
  erase();
  precompute(g_scene.screen.cols, g_scene.screen.rows,
             g_scene.camera.cam_dist, g_scene.camera.tilt_deg);
  g_scene.timer.sim_accum  = 0;
  g_scene.timer.frame_time = clock_ns();
}

/* How much real time passed since the last frame, capped so that if the
 * program was paused or stalled it doesn't try to catch up all at once. */
static long long frame_measure_dt(void) {
  long long now = clock_ns();
  long long dt  = now - g_scene.timer.frame_time;
  if (dt > DT_CAP_NS) dt = DT_CAP_NS;
  g_scene.timer.frame_time = now;
  return dt;
}

/* Move the disk forward in fixed-size steps: pile up the time that passed
 * and take one step for each step's worth, so the disk keeps a steady pace
 * no matter how fast the drawing runs. */
static void scene_advance_physics(long long dt) {
  if (g_scene.disk.paused) return;
  FrameTimer *tm = &g_scene.timer;
  tm->sim_accum += dt;
  while (tm->sim_accum >= tm->tick_ns) {
    g_scene.disk.angle += g_scene.disk.spin;
    if (g_scene.disk.angle >= (float)(2.0 * M_PI))
      g_scene.disk.angle -= (float)(2.0 * M_PI);
    clumps_tick();          /* move the bright clumps along too */
    tm->sim_accum -= tm->tick_ns;
  }
}

/* Keep the averaged fps number up to date, refreshing it about twice a
 * second. */
static void frame_tick_fps(long long dt) {
  FrameTimer *tm = &g_scene.timer;
  tm->fps_acc += dt;
  tm->fps_cnt++;
  if (tm->fps_acc >= FPS_WINDOW_NS) {
    tm->fps     = (float)tm->fps_cnt * 1e9f / (float)tm->fps_acc;
    tm->fps_acc = 0;
    tm->fps_cnt = 0;
  }
}

/* Draw one frame: clear the screen, paint the scene, add the text bars,
 * and push it all to the terminal. */
static void scene_draw_one_frame(void) {
  erase();
  render(g_scene.disk.angle,
         g_scene.screen.cols,  g_scene.screen.rows,
         g_scene.camera.cam_dist, g_scene.camera.tilt_deg,
         g_scene.render.show_stars);
  screen_hud(g_scene.screen.cols, g_scene.screen.rows, g_scene.timer.fps,
             g_scene.camera.cam_dist, g_scene.camera.tilt_deg,
             g_scene.render.theme,    g_scene.disk.paused);
  wnoutrefresh(stdscr);
  doupdate();
}

/* Wait out the rest of this frame's time budget so the frame rate stays
 * steady no matter how quickly the work finished. */
static void frame_cap_to_target_fps(long long frame_start_ns) {
  clock_sleep_ns(g_scene.timer.frame_ns - (clock_ns() - frame_start_ns));
}

/* ── §10.2 keyboard actions ── */

/* pause or unpause (the drawing keeps going either way) */
static void key_pause_toggle(void) {
  g_scene.disk.paused = !g_scene.disk.paused;
}

/* reset the disk's rotation and shuffle the clumps to fresh spots */
static void key_reset_disk(void) {
  g_scene.disk.angle = 0.0f;
  clumps_init();
}

/* switch to the next colour scheme */
static void key_cycle_theme(void) {
  g_scene.render.theme = (g_scene.render.theme + 1) % THEME_N;
  theme_apply(g_scene.render.theme);
}

/* move the camera closer or further (closer = bigger hole). Moving it means
 * the whole table has to be traced again. */
static void key_camera_zoom(float delta) {
  g_scene.camera.cam_dist = fclamp(g_scene.camera.cam_dist + delta,
                                   CAM_DIST_MIN, CAM_DIST_MAX);
  g_scene.need_rebuild = 1;
}

/* tilt the camera up or down (more tilt = more of a face-on ring view).
 * Also needs the table re-traced. */
static void key_camera_tilt(float delta) {
  g_scene.camera.tilt_deg = fclamp(g_scene.camera.tilt_deg + delta,
                                   TILT_DEG_MIN, TILT_DEG_MAX);
  g_scene.need_rebuild = 1;
}

/* turn the background stars on or off */
static void key_toggle_stars(void) {
  g_scene.render.show_stars = !g_scene.render.show_stars;
}

/* Grab one waiting keystroke (or none) and run the matching action. The
 * switch below reads as the key map. */
static void scene_handle_one_keystroke(void) {
  switch (getch()) {
  case 'q': case 'Q': case 27 /* ESC */: g_scene.running = 0;       break;
  case 'p': case 'P':                    key_pause_toggle();        break;
  case 'r': case 'R':                    key_reset_disk();          break;
  case 't': case 'T':                    key_cycle_theme();         break;
  case '+': case '=':                    key_camera_zoom(-CAM_DIST_STEP); break;
  case '-':                              key_camera_zoom(+CAM_DIST_STEP); break;
  case 'a':                              key_camera_tilt(+TILT_DEG_STEP); break;
  case 'A':                              key_camera_tilt(-TILT_DEG_STEP); break;
  case 'k': case 'K':                    key_toggle_stars();        break;
  default:                                                          break;
  }
}

/* ── §10.3 main ── */

/*
 * The program in one screen: get set up, then loop every frame —
 * handle any pending resize or camera change, see how much time passed,
 * move the disk, update the fps reading, draw, take one keypress, and
 * sleep off the rest of the frame. Quit when the running flag clears.
 */
int main(void) {
  scene_setup();

  while (g_scene.running) {
    /* re-trace the table if the window resized or the camera moved */
    if (g_scene.need_resize || g_scene.need_rebuild)
      scene_handle_resize_or_rebuild();

    long long dt = frame_measure_dt();

    scene_advance_physics(dt);   /* does nothing while paused */

    frame_tick_fps(dt);

    long long frame_start = clock_ns();
    scene_draw_one_frame();

    scene_handle_one_keystroke();

    frame_cap_to_target_fps(frame_start);
  }

  endwin();
  return 0;
}
