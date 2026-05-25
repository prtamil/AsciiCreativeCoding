/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * strange_attractor.c — Strange-Attractor Zoo (rotating 3-D trail visual)
 *
 * Ten chaotic attractors rendered in the same visual language as the
 * physics/lorenz.c demo: a live RING-BUFFER trail per trajectory,
 * projected through a rotating 3-D camera (azimuth φ + elevation θ),
 * with age-based head/mid/tail colour ramp [4], a comet-head '+' bloom
 * halo, depth-cued brightness [5], a slow-parallax starfield, and N
 * ε-offset GHOST trajectories that diverge on the Lyapunov timescale
 * (visible only for the ODE preset, but cosmetically present for all).
 *
 * Ten named presets, ordered SIMPLE → VISUALLY COMPLEX:
 *
 *   1  "Henon"        a= 1.40  b=0.30                  -- 2-param polynomial
 *   2  "Hopalong"     a= 7.70  b=0.13   c=8.15         -- 3-param Barry Martin
 *   3  "Clifford"     a=-1.40  b=1.60   c=1.00  d=0.70
 *   4  "de Jong"      a=-1.70  b=1.30   c=-0.10 d=-1.20
 *   5  "Rampe"        a= 1.00  b=-1.20  c=-0.50 d=0.50
 *   6  "Tinkerbell"   a= 0.90  b=-0.6013 c=2.00 d=0.50 -- butterfly polynomial
 *   7  "Svensson"     a= 1.50  b=-1.80  c= 1.60 d=0.90
 *   8  "Bedhead"      a=-0.81  b=-0.92                  -- 2-param custom
 *   9  "Marek"        a=-2.00  b=-2.00  c=-1.20 d=2.00 -- extreme Clifford
 *  10  "Lorenz"       σ=10  ρ=28  β=8/3  h=0.005       -- 3-D ODE (RK4)
 *
 * The first 9 are DISCRETE 2-D maps: each iterate jumps far across the
 * attractor, so the trail looks like a HOPPING CONSTELLATION of recent
 * visits that traces the attractor over time.  They are LIFTED to
 * z = 0 and pushed through the same 3-D projection as the Lorenz ODE
 * -- the figure appears as a flat sheet that rotates and tilts in
 * space, revealing the attractor from different angles.
 *
 * The 10th preset (Lorenz) is a CONTINUOUS-TIME ODE integrated with
 * classical four-stage Runge-Kutta [3]; consecutive samples are
 * infinitesimally close, so the trail forms a SMOOTH SPIRAL CURVE --
 * the famous butterfly attractor [1].
 *
 * ═════════════════════════════════════════════════════════════════════
 *  WHAT YOU SEE ON SCREEN
 * ═════════════════════════════════════════════════════════════════════
 *
 *  Each frame paints BACK-TO-FRONT four layers:
 *
 *    1. STARFIELD   -- 60 ambient '.' stars in a 3-D box around the
 *                      attractor, rotating with the same φ/θ for cheap
 *                      parallax.  Far stars A_DIM, near stars A_NORMAL.
 *    2. GHOSTS      -- 5 ε-offset shadow trajectories painted in the
 *                      theme's ghost colour + A_DIM with ',' glyph.
 *                      For Lorenz they fan out exponentially -- Lyapunov
 *                      divergence [6] made visceral.  For 2-D maps they
 *                      look like extra hopping dots (still pretty).
 *    3. MAIN TRAIL  -- ring-buffer of TRAIL_LEN samples drawn
 *                      newest → oldest:
 *                        oldest 20%   '.'   tier-tail   A_DIM
 *                        next   60%   '.'   tier-mid    A_NORMAL
 *                        newest 25%   '.'   tier-head   A_BOLD
 *                      plus DEPTH CUEING -- points closer to the camera
 *                      paint BOLD, ones behind fade to A_DIM, so the
 *                      trail visibly pulses in and out of the page as
 *                      the view rotates [5].  Newest sample is 'O' BOLD.
 *    4. BLOOM HALO  -- 4-cross '+' painted in tier-head AROUND the
 *                      newest two main-trail samples; gives the head a
 *                      wide glowing comet footprint.
 *
 *  THEMES (10, 4-anchor each):  NEON / MATRIX / SUNSET / OCEAN /
 *    PLASMA / INFERNO / MINT / AURORA / SYNTHWAVE / RAINBOW.  Each
 *    theme picks four ANSI-256 codes for head / mid / tail / ghost;
 *    cycle with t / T.
 *
 *  PROJECTION  (the 5-step pipeline in §4)
 *  -----------------------------------------------------------------
 *  Orthographic with azimuth φ and elevation θ [5].  Per preset, the
 *  attractor is centred on its calibrated bounding box's midpoint;
 *  the rotated z-component sz is kept as camera-axis depth and used
 *  by the renderer for depth-cued shading.
 *
 *      (px, py, pz) = (lx − cx, ly − cy, lz − cz)        ; centre
 *      (rx, ry)     = (px·cosφ + py·sinφ,                ; rotate z
 *                      −px·sinφ + py·cosφ)
 *      sx           = rx                                  ; tilt x
 *      sy           = ry·cosθ + pz·sinθ
 *      sz           = −ry·sinθ + pz·cosθ                  ; depth
 *      col          = cx_screen + sx·scale
 *      row          = cy_screen − sy·scale·ASPECT
 *
 *  For 2-D maps lz = cz = pz = 0, so the figure stays in its plane;
 *  θ tilts it like a sheet of paper, φ spins it around its normal.
 *
 * Keys:
 *   q / Q / ESC     quit
 *   space           pause / resume
 *   r / R           reset (re-seed orbit + clear trails + recalibrate)
 *   n / N           next preset
 *   p / P           previous preset
 *   1 .. 9          jump directly to preset 1..9
 *   0               jump to preset 10 (Lorenz)
 *   t / T           next / previous theme
 *   g / G           toggle ghost trajectories
 *   a / A           toggle auto-rotate
 *   ← / →           azimuth φ (manual, disables auto-rotate)
 *   ↑ / ↓           elevation θ
 *   + / =           iteration speed × 2  (capped at SPEED_MAX)
 *   - / _           iteration speed ÷ 2  (capped at SPEED_MIN)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/chaos/strange_attractor.c \
 *       -o strange_attractor -lncurses -lm
 *
 * ═════════════════════════════════════════════════════════════════════
 *  REFERENCES
 * ═════════════════════════════════════════════════════════════════════
 *
 *   [1] Lorenz, E. N. (1963) -- "Deterministic Nonperiodic Flow",
 *       *J. Atmos. Sci.* 20 (2), 130-141.  The original Lorenz ODE
 *       paper; basis for preset 10 and the entire chaos pedigree.
 *
 *   [2] Strogatz, S. H. (2014) -- *Nonlinear Dynamics and Chaos*,
 *       2nd ed., Westview.  Ch.9 on strange attractors and Lyapunov
 *       exponents; the textbook lens through which all 10 presets
 *       become "the same kind of object viewed differently".
 *
 *   [3] Press, W. H.; Teukolsky, S. A.; Vetterling, W. T.; Flannery,
 *       B. P. (2007) -- *Numerical Recipes*, 3rd ed., Cambridge.
 *       Ch.17 covers classical RK4 -- the integrator used for Lorenz.
 *       Backs h = 0.005 (RK4 global error ≈ h⁴ ≈ 6×10⁻¹⁰ per step).
 *
 *   [4] Quilez, I. (2015) -- "Palettes", iquilezles.org/articles/
 *       palettes.  Reference for 4-anchor palette design and the
 *       head→mid→tail brightness logic this file's themes follow
 *       (the codes themselves are hand-picked from the ANSI-256 cube,
 *       not procedurally generated).
 *
 *   [5] Foley, J. D.; van Dam, A.; Feiner, S. K.; Hughes, J. F. (1995)
 *       -- *Computer Graphics: Principles and Practice*, 2nd ed.,
 *       Addison-Wesley.  Ch.6 / 12 cover orthographic projection
 *       (the azimuth/elevation pipeline in §4) and depth-cueing
 *       brightness as a low-cost monocular depth cue.
 *
 *   [6] Wolf, A.; Swift, J. B.; Swinney, H. L.; Vastano, J. A. (1985)
 *       -- "Determining Lyapunov exponents from a time series",
 *       *Physica D* 16 (3), 285-317.  Foundational paper on numerical
 *       Lyapunov computation; sets the timescale on which the ghost
 *       trajectories fan out from the main orbit.
 *
 *   [7] Sprott, J. C. (1993) -- *Strange Attractors: Creating Patterns
 *       in Chaos*, M&T Books.  Catalog of 2-D and 3-D attractor maps
 *       (de Jong, Clifford, Hopalong, Tinkerbell, ...); sources the
 *       parameter sets for presets 2-9.
 *
 *   [8] Henon, M. (1976) -- "A two-dimensional mapping with a strange
 *       attractor", *Commun. Math. Phys.* 50 (1), 69-77.  The original
 *       Henon-map paper; basis for preset 1 and a worked example of a
 *       discrete map with fractal Hausdorff dimension ≈ 1.26.
 *
 *   [9] Bourke, P. -- "Strange Attractors" at paulbourke.net/fractals
 *       (clifford/, peterdejong/, hopalong/, tinkerbell/, ...).  The
 *       practical online catalog this file's 2-D preset parameters
 *       were tuned against; each page renders the attractor + lists
 *       canonical (a, b, c, d) tuples.
 *
 *  [10] Ward, M.; Grinstein, G.; Keim, D. (2015) -- *Interactive Data
 *       Visualization: Foundations, Techniques, and Applications*, 2nd
 *       ed., A K Peters.  Ch.6 covers age-based colour encoding (the
 *       head→mid→tail ramp here) and motion-cued depth perception
 *       (the parallax starfield + auto-rotate combination).
 *
 * Sections:
 *   §1 config        sizes, sampling, sub-stepping, colour pairs
 *   §2 clock         monotonic ns clock + sleep
 *   §3 color         themes (10 × {head, mid, tail, ghost})
 *   §4 coords        3-D → 2-D orthographic projection pipeline
 *   §5 attractor     step formulas + AttrDef + ATTRS + bbox calibration
 *   §6 orbits        Point3 + Trail (ring buffer) + Orbit (pos + Trail)
 *   §7 scene         Scene struct + per-frame tick + draw layers
 *   §8 hud           top data bar + bottom action bar
 *   §9 screen        Screen + initscr + resize
 *   §10 app          App + signals + key actions + main loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Ring-buffer TRAIL of recent orbit samples, drawn
 *                 newest → oldest each frame with an age-based colour
 *                 ramp.  Discrete-map presets push 1 sample per
 *                 attractor_step; the Lorenz ODE pushes 1 sample per
 *                 RK4 step.  Both share the same Trail, projection,
 *                 and rendering pipeline -- the only difference is
 *                 whether consecutive samples are infinitesimally
 *                 close (ODE → smooth curve) or far apart (map →
 *                 hopping constellation).
 *
 * Physics/Math  : 2-D maps -- Clifford / de Jong / Marek / Rampe share
 *                   x' = sin(a·y) + c·cos(a·x);
 *                   y' = sin(b·x) + d·cos(b·y).
 *                 Henon, Tinkerbell, Hopalong, Bedhead, Svensson each
 *                 have their own polynomial / radical formula.  All
 *                 nine are 2-D autonomous maps that exhibit chaos:
 *                 sensitive dependence on initial conditions [2],
 *                 fractal Hausdorff dimension, zero Lebesgue measure.
 *                 The 10th preset is the 3-D Lorenz ODE [1] integrated
 *                 with classical four-stage Runge-Kutta [3]; Lyapunov
 *                 exponent λ ≈ 0.9 sets the ghost-divergence timescale
 *                 [6].
 *
 * Rendering     : Orthographic projection [5] with azimuth φ + elevation
 *                 θ; depth-cued shading from the camera-axis depth sz;
 *                 4-cross '+' bloom halo around the newest 2 main-trail
 *                 samples; back-to-front layer stack (starfield →
 *                 ghosts → main trail).
 *
 * Palette       : 10 hand-picked 4-anchor themes in the xterm-256 cube.
 *                 Each theme provides {head, mid, tail, ghost} codes;
 *                 the age-tier mapping (newest 25% → head, next 60% →
 *                 mid, oldest 20% → tail) is theme-independent [10].
 *
 * Data-structure: Per-trajectory ring buffer of TRAIL_LEN points (1
 *                 main + N_GHOSTS = 5 ghosts = 6 trails total).  At 60
 *                 fps × 10 iter/tick = 600 samples/sec, a 2500-point
 *                 ring covers ~4 s of orbit history -- long enough for
 *                 the attractor outline to be visible at any moment.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
  TRAIL_LEN          = 2500, /* per-trajectory ring length             */
  SUB_STEPS_ODE      = 8,    /* RK4 sub-steps per tick (ODE presets)   */
  ITERS_PER_TICK_MAP = 10,   /* discrete iterations per tick (maps)    */
  N_GHOSTS           = 5,    /* ε-offset shadow trajectories per preset */
  WARMUP_ITERS       = 5000, /* discard initial transient (calibration) */
  BBOX_SAMPLES       = 50000,/* bbox probe size                         */
  N_STARS            = 60,   /* ambient starfield density               */
  SPEED_MIN          = 1,
  SPEED_MAX          = 8,
  FPS_UPDATE_MS      = 500,
};

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS      (NS_PER_SEC / 60)        /* 60 fps target          */

/* Main-loop timing -- both expressed against the 60 Hz fixed-step rate. */
#define MAX_FRAME_DT_NS      (100 * NS_PER_MS) /* dt cap -- spiral-of-death guard */
#define FIXED_TICK_DT_SEC    (1.0f / 60.0f)    /* sim-step wall time; matches TICK_NS */

/* Bbox calibration -- guards for degenerate (single-point) attractors. */
#define BBOX_MARGIN_FRAC      0.05f       /* bbox padding fraction       */
#define BBOX_RADIUS_EPS       1e-6f       /* radius below this = collapse */
#define BBOX_RADIUS_FALLBACK  1.0f        /* unit radius when bbox collapses */

#define ORBIT_SEED            0.1f        /* common (x, y, z) seed value */

/* Renderer sentinel -- "no previous cell painted yet" for the trail
 * dedupe check (a real cell can never be at (-999, -999)). */
#define CELL_NONE            (-999)

/* Lorenz parameters (classic chaotic regime).  Live in the AttrDef
 * (a, b, c) for SIGMA, RHO, BETA; d carries the RK4 step h. */
#define LORENZ_SIGMA  10.0f
#define LORENZ_RHO    28.0f
#define LORENZ_BETA   (8.0f / 3.0f)
#define LORENZ_H      0.005f

/* HUD layout. */
#define HUD_TOP_ROWS     2
#define HUD_BOTTOM_ROWS  1

/* View / projection. */
#define CELL_W 8
#define CELL_H 16
#define ASPECT ((float)CELL_W / (float)CELL_H)   /* ≈ 0.5 */

#define VIEW_PHI_DEFAULT    0.5f      /* initial azimuth (rad)            */
#define VIEW_THETA_DEFAULT  0.55f     /* initial elevation (rad)          */
#define VIEW_PHI_SPEED      0.08f     /* auto-rotation speed (rad/s)      */
#define VIEW_PHI_STEP       0.10f     /* per-keypress azimuth nudge (rad) */
#define VIEW_THETA_STEP     0.05f     /* per-keypress elevation nudge     */
#define VIEW_THETA_MIN      0.10f     /* avoid top-down singularity       */
#define VIEW_THETA_MAX      1.40f     /* avoid side-on singularity        */

#define VIEW_FILL_FRAC      0.80f     /* scale so bbox-diameter fills screen */

/* Depth thresholds for the BOLD / NORMAL / DIM ramp.  Expressed as a
 * fraction of attractor radius so the cue auto-rescales for the wide
 * range of attractor sizes (Lorenz ≈ 43 vs Henon ≈ 1.7). */
#define DEPTH_CLOSE_FRAC   -0.25f
#define DEPTH_FAR_FRAC      0.25f

/* Trail age tier breakpoints (0 = newest, 1 = oldest). */
#define AGE_HEAD_LIMIT      0.25f
#define AGE_MID_LIMIT       0.60f
#define AGE_TAIL_LIMIT      0.80f

/* Comet-head bloom -- glow halo painted around the newest 2 samples. */
#define BLOOM_HEAD_COUNT    2

#define KEY_ESC  27

/*
 * Colour-pair allocation (mirrors physics/lorenz.c so the visual
 * vocabulary is identical):
 *
 *   CP_TRAIL_HEAD  newest trail tier   -- A_BOLD,   theme `head`
 *   CP_TRAIL_MID   mid-age trail tier  -- A_NORMAL, theme `mid`
 *   CP_TRAIL_TAIL  oldest trail tier   -- A_DIM,    theme `tail`
 *   CP_GHOST       ghost trajectories  -- A_DIM,    theme `ghost`
 *   CP_HUD         status bar          -- canonical bright yellow 226
 *   CP_HINT        action keys         -- canonical bright cyan  51
 */
enum {
  CP_TRAIL_HEAD = 1,
  CP_TRAIL_MID  = 2,
  CP_TRAIL_TAIL = 3,
  CP_GHOST      = 4,
  CP_HUD        = 5,
  CP_HINT       = 6,
};

#define HUD_DATA_YELLOW_256   226
#define HUD_TITLE_CYAN_256     51

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
  if (ns <= 0) return;
  struct timespec req = {
    .tv_sec  = (time_t)(ns / NS_PER_SEC),
    .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color / theme                                                      */
/* ===================================================================== */

/* ── Theme ───────────────────────────────────────────────────────────── *
 *
 * One palette = one Theme row.  Encodes a complete visual identity as
 * FOUR ANSI-256 colour codes -- one per ncurses pair the trail renderer
 * touches.  At runtime the four codes are bound into CP_TRAIL_HEAD /
 * CP_TRAIL_MID / CP_TRAIL_TAIL / CP_GHOST by theme_apply(idx).
 *
 * WHY 4 anchors (and not a continuous gradient): ncurses has no
 *   per-cell colour interpolation -- every mvaddch lands on ONE colour
 *   pair.  We map four age tiers (head / mid / tail / ghost) onto
 *   four pairs and pick a code per tier; everything else is dithering
 *   via A_BOLD / A_DIM in draw_trail.
 *
 * WHY hand-picked (and not procedural cosine palettes): cosine
 *   gradients sweep through the muddy "in-between" zone of the cube at
 *   their sample points, producing chalky pastels for mid/tail.  Direct
 *   ANSI codes let each theme stake a vivid identity.  The brightness
 *   floor (every code has ≥1 RGB channel at max 5/5) keeps every tier
 *   legible even when painted A_DIM -- the recipe Inferno crystallised.
 *
 * WHY `short` for the codes: init_pair() takes a `short` for the colour
 *   code; matching the API type avoids a cast at every call.
 *
 * Fixed HUD pairs (CP_HUD = yellow 226, CP_HINT = cyan 51) are
 * theme-independent and bound once in color_init.
 *
 * References:
 *   [4]  Quilez 2015 -- 4-anchor head/mid/tail/ghost brightness logic
 *   [10] Ward et al  -- age-based colour encoding for trail data
 */
typedef struct {
  const char *name;     /* HUD label, e.g. "Inferno"                   */
  short       head;     /* newest 25% of trail -- comet leading edge   */
  short       mid;      /* next   60%          -- comet body           */
  short       tail;     /* oldest 20%          -- fading wake (A_DIM)  */
  short       ghost;    /* ε-offset shadows    -- contrast accent (DIM)*/
} Theme;

/*
 * 10 vivid 4-anchor themes.  INFERNO sets the brightness standard:
 * every code has AT LEAST ONE RGB channel at max (5/5 on the cube),
 * which keeps it saturated even when ncurses applies A_DIM to the tail
 * or ghost tier.  Codes like 99 / 105 / 171 (no channel at max) go
 * chalky under A_DIM and are avoided across the board.
 *
 * Each theme walks head → mid → tail through a coherent hue family
 * (the age ramp reads as a luminance/saturation fade), with the ghost
 * picked from a complementary saturated hue so the shadow trajectories
 * sit visually behind the main trail rather than blending in.
 */
static const Theme THEMES[] = {
  /*  name           head  mid  tail  ghost   palette character                  */
  { "Neon",          213,  207, 201,   51 }, /* hot pink ramp + cyan accent     */
  { "Matrix",         82,   46,  41,   51 }, /* lime → pure green + cyan accent */
  { "Sunset",        220,  214, 209,  213 }, /* gold → orange → salmon + pink   */
  { "Ocean",          51,   45,  39,  213 }, /* cyan → cyan-blue → blue + pink  */
  { "Plasma",        201,  165, 129,   51 }, /* magenta → purple → indigo + cyan*/
  { "Inferno",       226,  220, 208,  196 }, /* yellow → amber → orange → red   */
  { "Mint",          122,   86,  50,  213 }, /* mint → bright mint → cyan-mint  */
  { "Aurora",        122,   87,  51,  207 }, /* mint → light cyan → cyan + pink */
  { "Synthwave",     207,  165,  51,  213 }, /* hot pink → magenta → cyan + pink*/
  { "Rainbow",       196,  226,  46,   51 }, /* red → yellow → green + cyan     */
};
#define N_THEMES ((int)(sizeof THEMES / sizeof THEMES[0]))

/*
 * theme_apply -- rebind the THEME-DEPENDENT colour pairs for theme
 * `idx`.  Fixed HUD pairs (CP_HUD, CP_HINT) are NOT touched -- they
 * live in color_init and stay constant across themes.
 *
 * Stateless: takes the desired index, applies it, returns.  The
 * "current theme" is tracked by Scene.theme_idx (§7); no globals.
 */
static void theme_apply(int idx)
{
  if (idx < 0 || idx >= N_THEMES) return;
  const Theme *th = &THEMES[idx];

  if (COLORS >= 256) {
    init_pair(CP_TRAIL_HEAD, th->head,  -1);
    init_pair(CP_TRAIL_MID,  th->mid,   -1);
    init_pair(CP_TRAIL_TAIL, th->tail,  -1);
    init_pair(CP_GHOST,      th->ghost, -1);
  } else {
    /* 8-colour ANSI fallback -- themes collapse to one palette. */
    init_pair(CP_TRAIL_HEAD, COLOR_RED,     -1);
    init_pair(CP_TRAIL_MID,  COLOR_YELLOW,  -1);
    init_pair(CP_TRAIL_TAIL, COLOR_GREEN,   -1);
    init_pair(CP_GHOST,      COLOR_MAGENTA, -1);
  }
}

/*
 * color_init -- one-shot at startup.  Binds the two FIXED HUD pairs
 * (yellow status, cyan hints) and applies theme 0 as a sensible
 * default; scene_init then keeps Scene.theme_idx in sync.
 */
static void color_init(void)
{
  start_color();
  use_default_colors();

  if (COLORS >= 256) {
    init_pair(CP_HUD,  HUD_DATA_YELLOW_256, -1);
    init_pair(CP_HINT, HUD_TITLE_CYAN_256,  -1);
  } else {
    init_pair(CP_HUD,  COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN,   -1);
  }
  theme_apply(0);
}

/* ===================================================================== */
/* §4  coords -- orthographic 3-D → 2-D projection                       */
/* ===================================================================== */

/* ── project() math primitives ──────────────────────────────────────── *
 *
 * The full 3-D → 2-D pipeline is four named transformations applied in
 * order -- translate the world so the attractor is centred at the
 * origin, rotate around z by azimuth φ, tilt around x by elevation θ,
 * then map (sx, sy) to a terminal cell.  Each step gets a tiny helper
 * so the orchestrator project() reads as the textbook recipe [5].
 */

/* (1) TRANSLATE -- recentre (lx, ly, lz) so the attractor sits on the
 *     origin.  Pure shift; the bbox centre comes from §5 calibration. */
static inline void world_center_to_origin(float lx, float ly, float lz,
                                          float cx, float cy, float cz,
                                          float *px, float *py, float *pz)
{
  *px = lx - cx;
  *py = ly - cy;
  *pz = lz - cz;
}

/* (2) ROTATE AROUND Z by azimuth φ.  Standard 2-D rotation in the
 *     (x, y) plane; convention puts the camera implicitly along +y
 *     after rotation, so ry becomes the "into-screen" axis. [5] */
static inline void rotate_around_z_axis(float px, float py, float phi,
                                        float *rx, float *ry)
{
  float c = cosf(phi), s = sinf(phi);
  *rx =  px * c + py * s;
  *ry = -px * s + py * c;
}

/* (3) TILT AROUND X by elevation θ.  Rotates the (ry, pz) plane.
 *     After tilt, sy is the vertical screen coord and sz is the
 *     camera-axis depth (positive = behind centre / farther from
 *     camera; negative = in front / closer). [5] */
static inline void tilt_around_x_axis(float ry, float pz, float theta,
                                      float *sy, float *sz)
{
  float c = cosf(theta), s = sinf(theta);
  *sy =  ry * c + pz * s;
  *sz = -ry * s + pz * c;
}

/*
 * project() -- map an attractor-space point to terminal (col, row,
 * depth).  Pseudocode pipeline:
 *
 *   (1) translate world so attractor centre → origin
 *   (2) rotate around z by azimuth φ              → (rx, ry, pz)
 *   (3) tilt   around x by elevation θ            → (sx, sy) + depth sz
 *   (4) scale + offset to cell coords             → (col, row)
 *   (5) clip to the renderable band               → return false if off
 *
 * out_depth: pass NULL when the caller doesn't need depth shading
 * (e.g. starfield -- still uses depth, but a separate path).
 *
 * Returns false if (col, row) falls outside the renderable band.
 */
static bool project(float lx, float ly, float lz,
                    float cx, float cy, float cz,
                    float phi, float theta, float scale,
                    int screen_cx, int screen_cy, int cols, int rows,
                    int *out_col, int *out_row, float *out_depth)
{
  float px, py, pz;
  world_center_to_origin(lx, ly, lz, cx, cy, cz,
                         &px, &py, &pz);                          /* (1) */

  float rx, ry;
  rotate_around_z_axis(px, py, phi, &rx, &ry);                    /* (2) */

  float sx = rx;
  float sy, sz;
  tilt_around_x_axis(ry, pz, theta, &sy, &sz);                    /* (3) */

  int col = screen_cx + (int)(sx * scale);                        /* (4) */
  int row = screen_cy - (int)(sy * scale * ASPECT);

  if (out_col)   *out_col   = col;
  if (out_row)   *out_row   = row;
  if (out_depth) *out_depth = sz;

  return (col >= 0 && col < cols
       && row >= HUD_TOP_ROWS && row < rows - HUD_BOTTOM_ROWS);   /* (5) */
}

/* ===================================================================== */
/* §5  attractor definitions                                              */
/* ===================================================================== */

/*
 * AttrDef -- one preset.
 *
 * The `step` function pointer encodes WHICH formula this preset uses;
 * the (a, b, c, d) floats provide its parameters.  Function-pointer
 * dispatch (vs an int type-tag + switch) keeps `attractor_step` a
 * one-liner regardless of how many distinct formulas we add.
 *
 * The step signature carries THREE state pointers (x, y, z) because
 * the Lorenz ODE has 3-D state.  2-D maps ignore z (they take
 * `(void)*z` to silence the unused-parameter warning).
 *
 * is_continuous = true for ODE presets that integrate a vector field
 * (Lorenz).  scene_tick uses this to switch from per-tick discrete
 * iteration (ITERS_PER_TICK_MAP iters/tick) to RK4 sub-stepping
 * (SUB_STEPS_ODE × h Lorenz-time per tick).
 */
struct AttrDef;
typedef void (*attr_step_fn)(const struct AttrDef *at,
                             float *x, float *y, float *z);

typedef struct AttrDef {
  const char  *name;
  float        a, b, c, d;
  attr_step_fn step;
  bool         is_continuous;
} AttrDef;

/* ---- discrete-map step formulas (one per family) -------------------- */

/* Henon (1976) [8]: x' = 1 − a·x² + y,   y' = b·x  (canonical chaotic map). */
static void henon_step(const AttrDef *at, float *x, float *y, float *z)
{
  (void)z;
  float nx = 1.0f - at->a * (*x) * (*x) + (*y);
  float ny = at->b * (*x);
  *x = nx; *y = ny;
}

/* Hopalong (Barry Martin):
 *   x' = y − sign(x) · √|b·x − c|,   y' = a − x. */
static void hopalong_step(const AttrDef *at, float *x, float *y, float *z)
{
  (void)z;
  float sign_x = (*x > 0.0f) ? 1.0f : ((*x < 0.0f) ? -1.0f : 0.0f);
  float nx = *y - sign_x * sqrtf(fabsf(at->b * (*x) - at->c));
  float ny = at->a - *x;
  *x = nx; *y = ny;
}

/* Clifford / de Jong / Marek / Rampe shared form:
 *   x' = sin(a·y) + c·cos(a·x),   y' = sin(b·x) + d·cos(b·y). */
static void clifford_step(const AttrDef *at, float *x, float *y, float *z)
{
  (void)z;
  float nx = sinf(at->a * (*y)) + at->c * cosf(at->a * (*x));
  float ny = sinf(at->b * (*x)) + at->d * cosf(at->b * (*y));
  *x = nx; *y = ny;
}

/* Tinkerbell:
 *   x' = x² − y² + a·x + b·y,   y' = 2·x·y + c·x + d·y. */
static void tinkerbell_step(const AttrDef *at, float *x, float *y, float *z)
{
  (void)z;
  float xx = (*x) * (*x);
  float yy = (*y) * (*y);
  float nx = xx - yy + at->a * (*x) + at->b * (*y);
  float ny = 2.0f * (*x) * (*y) + at->c * (*x) + at->d * (*y);
  *x = nx; *y = ny;
}

/* Svensson:
 *   x' = d·sin(a·x) − sin(b·y),   y' = c·cos(a·x) + cos(b·y). */
static void svensson_step(const AttrDef *at, float *x, float *y, float *z)
{
  (void)z;
  float nx = at->d * sinf(at->a * (*x)) - sinf(at->b * (*y));
  float ny = at->c * cosf(at->a * (*x)) + cosf(at->b * (*y));
  *x = nx; *y = ny;
}

/* Bedhead:
 *   x' = sin(x·y/b)·y + cos(a·x − y),   y' = x + sin(y)/b. */
static void bedhead_step(const AttrDef *at, float *x, float *y, float *z)
{
  (void)z;
  float b_safe = (fabsf(at->b) < 1e-4f) ? 1e-4f : at->b;
  float nx = sinf((*x) * (*y) / b_safe) * (*y) + cosf(at->a * (*x) - (*y));
  float ny = (*x) + sinf(*y) / b_safe;
  *x = nx; *y = ny;
}

/* ---- Lorenz ODE + RK4 ----------------------------------------------- */

/*
 * lorenz_deriv -- evaluate the Lorenz vector field at (x, y, z).
 *
 *   dx/dt = σ·(y − x)
 *   dy/dt = x·(ρ − z) − y
 *   dz/dt = x·y − β·z
 *
 * Autonomous (no explicit t), so each call only depends on the
 * current state.  Inlined for the RK4 stages below.
 */
static inline void lorenz_deriv(float x, float y, float z,
                                float *dx, float *dy, float *dz)
{
  *dx = LORENZ_SIGMA * (y - x);
  *dy = x * (LORENZ_RHO - z) - y;
  *dz = x * y - LORENZ_BETA * z;
}

/*
 * lorenz_step -- one classical four-stage Runge-Kutta step [3].
 *
 *   k1 = f(y_n)
 *   k2 = f(y_n + h/2 · k1)
 *   k3 = f(y_n + h/2 · k2)
 *   k4 = f(y_n + h   · k3)
 *   y_{n+1} = y_n + h/6 · (k1 + 2 k2 + 2 k3 + k4)
 *
 * Global error O(h⁴).  At h = 0.005 the per-step error ≈ 6×10⁻¹⁰,
 * well below float precision over the first ten seconds of simulated
 * time -- the orbit stays on the true attractor visually forever.
 */
static void lorenz_step(const AttrDef *at, float *x, float *y, float *z)
{
  float h = at->d;
  float k1x, k1y, k1z, k2x, k2y, k2z, k3x, k3y, k3z, k4x, k4y, k4z;

  lorenz_deriv(*x, *y, *z, &k1x, &k1y, &k1z);
  lorenz_deriv(*x + 0.5f * h * k1x,
               *y + 0.5f * h * k1y,
               *z + 0.5f * h * k1z, &k2x, &k2y, &k2z);
  lorenz_deriv(*x + 0.5f * h * k2x,
               *y + 0.5f * h * k2y,
               *z + 0.5f * h * k2z, &k3x, &k3y, &k3z);
  lorenz_deriv(*x + h * k3x,
               *y + h * k3y,
               *z + h * k3z, &k4x, &k4y, &k4z);

  *x += (h / 6.0f) * (k1x + 2.0f * k2x + 2.0f * k3x + k4x);
  *y += (h / 6.0f) * (k1y + 2.0f * k2y + 2.0f * k3y + k4y);
  *z += (h / 6.0f) * (k1z + 2.0f * k2z + 2.0f * k3z + k4z);
}

/* ---- the preset table (simple → complex) ---------------------------- *
 *
 * The (a, b, c, d) tuples below come from Sprott [7] and Bourke [9]
 * (paulbourke.net/fractals).  Each row is one preset in the order the
 * `n`/`p`/`1..0` keys jump through them.
 */

static const AttrDef ATTRS[] = {
  /*  name           a              b              c             d           step             continuous */
  { "Henon",       1.40f,         0.30f,         0.00f,         0.00f,    henon_step,      false },
  { "Hopalong",    7.70f,         0.13f,         8.15f,         0.00f,    hopalong_step,   false },
  { "Clifford",   -1.40f,         1.60f,         1.00f,         0.70f,    clifford_step,   false },
  { "de Jong",    -1.70f,         1.30f,        -0.10f,        -1.20f,    clifford_step,   false },
  { "Rampe",       1.00f,        -1.20f,        -0.50f,         0.50f,    clifford_step,   false },
  { "Tinkerbell",  0.90f,        -0.6013f,       2.00f,         0.50f,    tinkerbell_step, false },
  { "Svensson",    1.50f,        -1.80f,         1.60f,         0.90f,    svensson_step,   false },
  { "Bedhead",    -0.81f,        -0.92f,         0.00f,         0.00f,    bedhead_step,    false },
  { "Marek",      -2.00f,        -2.00f,        -1.20f,         2.00f,    clifford_step,   false },
  { "Lorenz",      LORENZ_SIGMA,  LORENZ_RHO,    LORENZ_BETA,   LORENZ_H, lorenz_step,     true  },
};
#define N_ATTRS  ((int)(sizeof ATTRS / sizeof ATTRS[0]))

/* Polymorphic dispatcher: one step of whichever map / ODE this is. */
static inline void attractor_step(const AttrDef *at,
                                  float *x, float *y, float *z)
{
  at->step(at, x, y, z);
}

/* ===================================================================== */
/* §6  orbits  --  Point3, Trail (ring buffer), Orbit (pos + Trail)      */
/* ===================================================================== */

/*
 * §6 builds the abstractions for tracking ONE moving particle through
 * attractor-space.  Three types in a clean composition stack:
 *
 *   Point3   atomic   -- one (x, y, z) sample in attractor-space
 *   Trail    bounded  -- ring buffer of recent Point3 samples
 *   Orbit    composite-- current Point3 + a Trail (the two pieces
 *                        that always travel together for one particle)
 *
 * Scene (§7) holds 1 main Orbit + N_GHOSTS ghost Orbits.
 */

/* ── Point3 ──────────────────────────────────────────────────────────── *
 *
 * Bare 3-coord point in attractor-space.  Used as the atomic element
 * inside Trail (a ring of Point3) and as the "current position" field
 * of an Orbit.
 *
 * WHY a struct (and not three loose floats): lets a single point pass
 *   by value through one parameter slot.  Before this type existed,
 *   every helper that handled "a position" had to take three floats
 *   AND every Trail field had to be parallel arrays just to avoid
 *   defining this type.  Adding the 12-byte struct simplifies the
 *   whole stack.
 *
 * WHY not Vec3 / vec3: in dynamical-systems literature a trajectory is
 *   a sequence of POINTS (not vectors).  "Point3" reads correctly
 *   whether the value is a current position or a recorded sample.
 */
typedef struct {
  float x, y, z;     /* attractor-space coordinates                   */
} Point3;

/* ── Trail ───────────────────────────────────────────────────────────── *
 *
 * Bounded history of recent orbit samples for one trajectory --
 * implemented as a circular (ring) buffer of Point3.
 *
 * WHY a ring buffer (and not a linear append-only array):
 *   the orbit runs forever; a linear buffer would either grow
 *   unbounded (memory leak) or stop recording at a cap (info loss).
 *   A ring of TRAIL_LEN slots overwrites the oldest sample on each
 *   push -- bounded memory, O(1) push, and the last TRAIL_LEN samples
 *   are always available.  TRAIL_LEN = 2500 covers ≈ 4 s of orbit
 *   history at the 60 fps × 10 iter/tick cadence (and ≈ 5 s for Lorenz
 *   at 60 fps × 8 RK4 sub-steps).
 *
 * WHY {head, count} (and not {head, tail}):
 *   the buffer fills monotonically (no element is ever removed except
 *   by overwrite), so a tail pointer would just be head − count + 1
 *   mod TRAIL_LEN -- redundant.  `count` also gives the renderer a
 *   free progress signal: count < TRAIL_LEN means the ring isn't full
 *   yet, so the oldest end shows what's actually there rather than
 *   uninitialised noise.
 *
 * WHY array-of-Point3 (AoS) and not three parallel float arrays (SoA):
 *   earlier revisions used parallel arrays to avoid needing a 3-coord
 *   type.  Now that Point3 exists, AoS reads cleaner -- the ring is
 *   visibly "a ring of points", not "three coincidental arrays".
 *   Cache behaviour is identical: the renderer reads all three coords
 *   per sample inside one loop iteration anyway.
 *
 * Memory: TRAIL_LEN × sizeof(Point3) = 2500 × 12 = 30 KB per trail.
 * With 1 main + N_GHOSTS = 5 ghosts = 6 trails per scene → ≈ 180 KB.
 *
 * Reference: classic circular-buffer pattern; see Sedgewick &
 * Wayne, *Algorithms* 4th ed., §1.3 (queue with bounded capacity).
 */
typedef struct {
  Point3 data[TRAIL_LEN]; /* ring storage -- index 0..TRAIL_LEN-1     */
  int    head;            /* index of NEWEST sample; written-into     */
  int    count;           /* valid samples in ring, ≤ TRAIL_LEN       */
} Trail;

/* trail_clear -- reset to empty ring (no samples). */
static void trail_clear(Trail *t)
{
  t->head  = 0;
  t->count = 0;
}

/* trail_push -- record one sample.  Advances head, overwrites oldest
 * slot when the ring is full (count == TRAIL_LEN). */
static void trail_push(Trail *t, Point3 p)
{
  t->head = (t->head + 1) % TRAIL_LEN;
  t->data[t->head] = p;
  if (t->count < TRAIL_LEN) t->count++;
}

/* trail_at -- read the k-th NEWEST sample (k = 0 is the most recent,
 * k = t->count - 1 is the oldest).  Hides the ring's modular index
 * arithmetic from callers so consumer loops read as "for each sample
 * from newest to oldest" instead of computing indices by hand.
 *
 * Undefined for k < 0 or k >= t->count -- caller must respect count. */
static inline Point3 trail_at(const Trail *t, int k)
{
  int idx = (t->head - k + TRAIL_LEN) % TRAIL_LEN;
  return t->data[idx];
}

/* ── Orbit ───────────────────────────────────────────────────────────── *
 *
 * One trajectory through attractor-space: the CURRENT position + a
 * TRAIL of recent samples.  Bundles the two pieces of state that
 * always travel together -- you never want to advance the position
 * without recording it, or read the trail without knowing the live
 * head's position.
 *
 * WHY a struct (and not loose pos floats + a separate Trail):
 *   the previous revision had main = (mx, my, mz, mt) and PARALLEL
 *   ghost arrays = (gx[], gy[], gz[], gt[]) -- seven Scene fields for
 *   one conceptual object, repeated for N_GHOSTS more.  Wrapping
 *   them into Orbit makes the abstraction visible: "Scene has one
 *   main Orbit and N_GHOSTS ghost Orbits".  Ghosts become an array
 *   of Orbits, not four coincidental parallel arrays.
 *
 * WHY pos AND trail live in the same struct (vs trail-only with
 *   pos = trail_at(t, 0)):
 *   the position is an INPUT to the next attractor_step (needs to be
 *   mutated in place); reading it back from the trail every step
 *   would mean head-index math on the hot path.  Keeping `pos` as a
 *   live mutable field + pushing a copy into the trail keeps both
 *   roles explicit and avoids hidden coupling.
 */
typedef struct {
  Point3 pos;          /* current attractor-space position             */
  Trail  trail;        /* ring of past positions (newest at head)      */
} Orbit;

/* orbit_seed -- set initial conditions: place the particle at p and
 * clear its trail.  Used by scene_seed_trajectories at reset. */
static void orbit_seed(Orbit *o, Point3 p)
{
  o->pos = p;
  trail_clear(&o->trail);
}

/* orbit_step -- one iterate: advance pos via the attractor's step
 * function, then record the new pos in the trail.  All trajectories
 * (main + ghosts) advance through this single function so they stay
 * in LOCKSTEP -- same iteration index across the scene, which is what
 * lets ε-divergence between ghosts be visually meaningful. */
static void orbit_step(Orbit *o, const AttrDef *at)
{
  attractor_step(at, &o->pos.x, &o->pos.y, &o->pos.z);
  trail_push(&o->trail, o->pos);
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * §7 organises the SCENE -- everything that lives in attractor space
 * and gets projected onto the terminal each frame.  Four nested types
 * carve up that scope so each has a clean single job:
 *
 *   Camera    -- viewer pose: azimuth φ, elevation θ, auto-rotate flag.
 *                Written by handle_key (or drifted by scene_tick when
 *                auto-rotate is on); read by every renderer.
 *
 *   Bounds    -- attractor spatial extent: axis-aligned bbox PLUS the
 *                derived centre (translation origin) and diagonal
 *                radius (drives view scale + depth normalisation).
 *                Written by scene_calibrate_bounds at reset; read by
 *                project() / compute_view_scale / depth shading.
 *
 *   Starfield -- N_STARS ambient backdrop points in attractor-space.
 *                Self-initialising; projected with the same Camera as
 *                the trail so the parallax matches.
 *
 *   RenderCtx -- per-frame derived view state: cols/rows, screen
 *                centre, projection scale, depth normaliser.  Built
 *                once at the top of scene_draw and threaded through
 *                every draw helper -- replaces a 9-parameter signature
 *                with a single pointer.
 *
 *   Scene     -- the umbrella struct.  Owns trajectories + Trails,
 *                Camera, Bounds, Starfield, UI flags, speed.  ONE
 *                Scene per program, inside g_app (§10).
 */

/* ── Camera ──────────────────────────────────────────────────────────── *
 *
 * The viewer pose for an ORTHOGRAPHIC projection.  Just two angles --
 * azimuth φ + elevation θ -- because orthographic has no eye position
 * (rays are parallel) and no perspective foreshortening; what matters
 * is the rotation that maps world-space to camera-space.
 *
 *   phi      ROTATION around the world z-axis (spin the world around
 *            its vertical axis -- like rotating a globe).  φ = 0
 *            looks down the +y axis; +φ rotates the world clockwise
 *            as seen from above.  Wraps freely (any real value).
 *
 *   theta    TILT around the camera-local x-axis after the spin
 *            (lean the camera up or down).  θ near 0 looks edge-on
 *            (degenerate flat view); θ near π/2 looks straight down.
 *            Clamped to [VIEW_THETA_MIN, VIEW_THETA_MAX] = [0.10, 1.40]
 *            so the view stays oblique and the attractor doesn't
 *            collapse into a line.
 *
 *   auto_rotate
 *            Camera MODE flag.  When true, scene_tick drifts phi by
 *            VIEW_PHI_SPEED · dt each tick (hands-free demo).  When
 *            false, phi only changes via ← / → keys.  Lives inside
 *            Camera (not Scene's UI block) because it's about how the
 *            camera moves itself, not a renderer toggle.
 *
 * The full 3-D → 2-D pipeline that consumes (φ, θ) is the 4-step
 * orthographic recipe in §4 -- translate → rotate-z → tilt-x → scale.
 *
 * Reference: [5] Foley et al. Ch.6 -- orthographic camera with
 * azimuth/elevation parameterisation (avoids gimbal lock for any
 * (φ, θ) the input clamps allow).
 */
typedef struct {
  float phi;           /* azimuth (rad) -- spin around world z-axis    */
  float theta;         /* elevation (rad) -- tilt around camera x-axis */
  bool  auto_rotate;   /* if true, scene_tick advances phi each tick   */
} Camera;

/* ── Bounds ──────────────────────────────────────────────────────────── *
 *
 * The attractor's spatial extent, in three forms that the renderer
 * consumes at different stages:
 *
 *   xmin..zmax  RAW axis-aligned bounding box (AABB).  Computed once
 *               per preset by scene_calibrate_bounds: walk WARMUP_ITERS
 *               iterates to land on the attractor, then walk
 *               BBOX_SAMPLES more and track per-axis min/max.  Padded
 *               by BBOX_MARGIN_FRAC so the extremes don't sit on the
 *               projection clip plane.
 *
 *   cx, cy, cz  TRANSLATION ORIGIN -- the bbox midpoint.  Subtracted
 *               from every world point in project()'s step (1) so the
 *               attractor's centre lands at the screen centre instead
 *               of in some corner.  Cached here (not recomputed per
 *               sample) because the renderer reads it ~1500 times per
 *               frame and a bbox doesn't change between resets.
 *
 *   radius      ATTRACTOR-SCALE NUMBER -- half the bbox diagonal.
 *               Drives two unrelated things downstream:
 *                 (a) compute_view_scale picks scale = screen_dim /
 *                     (2·radius·fill_frac) so the orbit fills the
 *                     terminal regardless of preset size (Lorenz ≈ 43
 *                     vs Hénon ≈ 1.7).
 *                 (b) depth-cue thresholds in DEPTH_CLOSE_FRAC /
 *                     DEPTH_FAR_FRAC are expressed as fractions of
 *                     radius so they auto-rescale per preset.
 *
 * WHY store raw bbox AND derived centre/radius (instead of just one):
 *   cx/cy/cz/radius are pure functions of xmin..zmax, but they're hot-
 *   path reads.  Computing them per sample would burn cycles inside
 *   project(); computing them once per scene_reset and caching them
 *   here is the classic "precompute derived values" trade.
 *
 * For 2-D map presets, zmin = zmax = 0 (the orbit lives in z = 0) and
 * the projection degenerates to a rotation of a flat figure -- still
 * a valid AABB, just one whose z extent is exactly zero.
 *
 * Reference: [5] Foley et al. Ch.12 -- axis-aligned bounding volumes;
 * the "diagonal half-length" is the bounding sphere radius.
 */
typedef struct {
  float xmin, xmax;    /* per-axis extremes from BBOX_SAMPLES probe    */
  float ymin, ymax;
  float zmin, zmax;
  float cx, cy, cz;    /* bbox midpoint -- projection translation      */
  float radius;        /* 0.5 · sqrt(bw² + bh² + bd²) -- scale driver  */
} Bounds;

/* ── Starfield ───────────────────────────────────────────────────────── *
 *
 * N_STARS = 60 fixed 3-D points scattered in a box around the attractor,
 * acting as an ambient backdrop.  Projected with the SAME Camera as the
 * orbit each frame, so the stars rotate WITH the view -- creating
 * motion parallax: distant stars sweep behind the attractor as the
 * camera spins, a strong monocular depth cue without needing a real
 * z-buffer.
 *
 * BOX GEOMETRY (in attractor-space units):
 *   x, y  ∈ [-40, +40]      isotropic horizontal extent
 *   z     ∈ [-20, +40]      asymmetric vertical box -- more depth
 *                           BEHIND the camera-elevation plane than in
 *                           front, so most stars are far away
 *
 * The ±40 sizing was picked to comfortably enclose the LARGEST preset
 * (Lorenz ≈ ±25) and extend well past the SMALLEST (Hénon ≈ ±1.5).
 * For small attractors the star field looks like a wide sky around a
 * tiny painting; for large ones the stars sit just outside the orbit
 * envelope, like satellites.
 *
 * WHY a struct (and not 3 globals): puts the lifetime + ownership
 *   inside Scene where it belongs.  Earlier revisions had g_star_x[]
 *   / g_star_y[] / g_star_z[] / g_stars_initialised as file-scope
 *   globals -- that worked but obscured the fact that the starfield
 *   IS scene state.
 *
 * WHY parallel x[]/y[]/z[] arrays (SoA, not AoS): exact same trade-off
 *   as Trail -- the renderer reads all three coords per star anyway, so
 *   layout doesn't matter for cache.  SoA matches the LCG-driven
 *   per-coord seeding loop in starfield_init.
 *
 * WHY `initialised` flag (lazy-init pattern): starfield_init is
 *   idempotent -- callable multiple times safely.  scene_init triggers
 *   the FIRST init; future scene_resets skip it (the stars don't need
 *   to change when the attractor preset changes -- the backdrop is
 *   preset-independent).
 *
 * Reference: [5] Foley et al. Ch.14 -- motion parallax as a depth cue
 * (objects at different depths displace by amounts proportional to
 * their depth, when the camera moves).
 */
#define STAR_BOX_HALF_EXTENT  40.0f   /* x,y box half-width (attr units) */
#define STAR_BOX_Z_MIN       -20.0f   /* z box near plane                */
#define STAR_BOX_Z_RANGE      60.0f   /* z box depth (so zmax = +40)     */

typedef struct {
  float x[N_STARS];    /* per-star world coords, attractor-space units  */
  float y[N_STARS];
  float z[N_STARS];
  bool  initialised;   /* lazy-init guard -- set true on first init     */
} Starfield;

/* ── RenderCtx ───────────────────────────────────────────────────────── *
 *
 * A FRAME-LOCAL CACHE: derived view state that is constant for one
 * frame but computed FROM (Screen, Scene) state that can change
 * between frames.  Built once at the top of scene_draw by
 * render_ctx_make() and threaded through every draw helper, so each
 * one can take (scene, ctx) instead of 9 loose floats.
 *
 * WHY a struct (and not pass these as parameters): draw_trail wants
 *   six values to project a point and clip to the renderable band.
 *   Passing them individually meant an 11-parameter signature that
 *   was hostile to read.  Bundling them into one pointer turns every
 *   draw helper into a 2-parameter function -- the textbook
 *   "parameter object" refactor.
 *
 * WHY rebuild every frame (vs cache on Scene): the inputs (cols, rows
 *   from Screen; bounds.radius from Scene) can change at any time --
 *   SIGWINCH resizes, scene_reset recalibrates bounds.  Recomputing 6
 *   floats once per frame is free; mutating Scene to invalidate a
 *   cache would couple draw to event handling.
 *
 * FIELD PROVENANCE (where each value comes from):
 *
 *   cols, rows           ← Screen (terminal geometry from getmaxyx)
 *   screen_cx            ← cols / 2
 *   screen_cy            ← centred between HUD top + bottom bars
 *   scale                ← compute_view_scale(bounds, cols, rows)
 *                          picks min(scale_x, scale_y) so the
 *                          attractor diagonal fills VIEW_FILL_FRAC of
 *                          the smaller dimension after aspect ratio
 *   depth_unit           ← bounds.radius (attractor's diagonal half-
 *                          length, used as the unit for normalising
 *                          camera-axis depth into [-1, +1] before
 *                          comparing against DEPTH_CLOSE/FAR_FRAC)
 *
 * Nothing inside RenderCtx is persistent; pointers to it never escape
 * scene_draw's stack frame.
 */
typedef struct {
  int   cols, rows;       /* terminal extent (from Screen.cols/rows)  */
  int   screen_cx;        /* projection origin col (= cols / 2)       */
  int   screen_cy;        /* projection origin row (between HUD bars) */
  float scale;            /* attractor-units → cell units             */
  float depth_unit;       /* normaliser for depth-cue thresholds      */
} RenderCtx;

/*
 * GHOST_EPS_TABLE -- per-ghost initial x-offset (attractor units).
 * Spans four orders of magnitude (0.001 → 0.1) so the divergence
 * timescale is visible across one viewing session.  For Lorenz
 * (λ ≈ 0.9 [6]) the smallest ε reaches attractor scale in ~10 s;
 * for discrete maps each ghost is just another iterate cloud
 * exploring the attractor -- still visually pleasing.
 */
static const float GHOST_EPS_TABLE[N_GHOSTS] = { 0.001f, 0.005f, 0.01f,
                                                 0.05f,  0.1f };

/*
 * Scene -- the entire visible world in one struct.
 *
 * WHY one struct (and not loose globals): every field shares the
 *   same lifetime -- born at scene_init, dies at scene_free (none
 *   today, but the API supports it).  Bundling lets `Scene *s`
 *   carry the world through scene_tick / scene_draw / app_handle_key
 *   without 15-argument function signatures.
 *
 * LOCALITY INTENT (fields grouped top-to-bottom by ROLE):
 *
 *   ORBITS         physics state -- 1 main Orbit + N_GHOSTS ghost
 *                  Orbits, all advanced by scene_tick.  Each Orbit
 *                  bundles a Point3 pos + a Trail of past positions
 *                  (§6).  Replacing the earlier 7 loose fields
 *                  (mx,my,mz,mt + gx[],gy[],gz[],gt[]) with two Orbit
 *                  arrays cut the field count from 7 to 2 and made
 *                  "there are N_GHOSTS+1 trajectories" explicit.
 *   ATTRACTOR      which preset + its spatial Bounds
 *   CAMERA         viewer pose
 *   STARFIELD      ambient backdrop
 *   UI LATCHES     key-handler toggles
 *   COUNTERS       HUD readouts
 *
 * Memory: trails dominate -- (1 + N_GHOSTS) × Trail × 30 KB = 6 ×
 * 30 KB ≈ 180 KB.  Scene lives inside the static g_app (§10); never
 * on the stack, never allocated.
 */
typedef struct {
  /* ── ORBITS: 1 main + N_GHOSTS ε-offset shadow trajectories ─────── */
  Orbit main;            /* the headline trajectory                    */
  Orbit ghosts[N_GHOSTS];/* ε-offset shadows; diverge at Lyapunov rate */

  /* ── ATTRACTOR: which preset + its spatial extent ───────────────── */
  int    attr_idx;
  Bounds bounds;

  /* ── CAMERA + BACKDROP ──────────────────────────────────────────── */
  Camera    camera;
  Starfield stars;

  /* ── UI LATCHES (written by key handler) ────────────────────────── */
  bool show_ghost;       /* g -- toggle ε-offset shadow trajectories  */
  bool paused;           /* space -- scene_tick early-returns         */

  /* ── COUNTERS (HUD readouts) ────────────────────────────────────── */
  int       speed;       /* iter-rate multiplier (+/- via keys)       */
  int       theme_idx;   /* active theme (index into THEMES)          */
  long long total_pts;   /* accumulated iterates this run             */
} Scene;

static inline const AttrDef *scene_current_attr(const Scene *s)
{
  return &ATTRS[s->attr_idx];
}

/* ── Starfield helpers ──────────────────────────────────────────────── *
 *
 * Self-contained backdrop: a tiny LCG seeds N_STARS positions in a
 * fixed box; starfield_draw projects them with the Scene's Camera.
 *
 * The LCG (vs srand/rand) keeps star placement reproducible without
 * touching libc's global RNG state.
 */
static unsigned int stars_lcg_next(unsigned int *s)
{
  *s = (*s) * 1103515245u + 12345u;
  return (*s) >> 16;
}

static inline float stars_lcg_unit(unsigned int *s)
{
  return (float)(stars_lcg_next(s) & 0xFFFF) / 65535.0f;
}

static void starfield_init(Starfield *sf)
{
  if (sf->initialised) return;
  unsigned int s = 987654321u;
  for (int i = 0; i < N_STARS; i++) {
    sf->x[i] = (stars_lcg_unit(&s) - 0.5f) * 2.0f * STAR_BOX_HALF_EXTENT;
    sf->y[i] = (stars_lcg_unit(&s) - 0.5f) * 2.0f * STAR_BOX_HALF_EXTENT;
    sf->z[i] = STAR_BOX_Z_MIN + stars_lcg_unit(&s) * STAR_BOX_Z_RANGE;
  }
  sf->initialised = true;
}

/* Depth attribute for a starfield point: closer stars get A_NORMAL,
 * stars behind the depth-cue plane fade to A_DIM.  Uses the SAME
 * DEPTH_CLOSE_FRAC threshold as the trail renderer so the cue is
 * consistent between layers. */
static inline attr_t star_attr_by_depth(float depth, float depth_unit)
{
  return (depth < depth_unit * DEPTH_CLOSE_FRAC) ? A_NORMAL : A_DIM;
}

/*
 * starfield_draw -- back-most layer.  For each star:
 *   (1) project with the SAME Camera as the orbit (so the field rotates
 *       in lockstep -- the parallax depth cue);
 *   (2) pick A_NORMAL/A_DIM by camera-axis depth;
 *   (3) paint a '.' in the tail (faintest) colour pair.
 */
static void starfield_draw(const Starfield *sf, const Camera *cam,
                           const RenderCtx *ctx)
{
  for (int i = 0; i < N_STARS; i++) {
    /* Step 1: project star with the Scene's Camera (parallax matches orbit). */
    int   col, row;
    float depth;
    if (!project(sf->x[i], sf->y[i], sf->z[i],
                 0.0f, 0.0f, 0.0f,    /* stars live in absolute space */
                 cam->phi, cam->theta, ctx->scale,
                 ctx->screen_cx, ctx->screen_cy, ctx->cols, ctx->rows,
                 &col, &row, &depth))
      continue;

    /* Step 2: pick attribute by camera-axis depth (close=NORMAL, far=DIM). */
    attr_t at = star_attr_by_depth(depth, ctx->depth_unit);

    /* Step 3: paint star glyph in the tail colour (faintest tier). */
    attron(COLOR_PAIR(CP_TRAIL_TAIL) | at);
    mvaddch(row, col, '.');
    attroff(COLOR_PAIR(CP_TRAIL_TAIL) | at);
  }
}

/* ── scene lifecycle: seed → calibrate → reset → init ───────────────── */

/*
 * scene_seed_trajectories -- place the main orbit + N_GHOSTS at their
 * initial conditions and clear every trail.
 *
 * Main at (ORBIT_SEED, ORBIT_SEED, ORBIT_SEED).  Ghosts at
 * (ORBIT_SEED + ε_g, ORBIT_SEED, ORBIT_SEED) where ε_g spans four
 * decades from GHOST_EPS_TABLE.  By construction the ghosts differ
 * ONLY in initial conditions, so any divergence is pure deterministic
 * chaos (sensitive dependence on initial conditions, [2]).
 */
static void scene_seed_trajectories(Scene *s)
{
  const Point3 main_seed = { ORBIT_SEED, ORBIT_SEED, ORBIT_SEED };
  orbit_seed(&s->main, main_seed);

  for (int g = 0; g < N_GHOSTS; g++) {
    Point3 ghost_seed = { ORBIT_SEED + GHOST_EPS_TABLE[g],
                          ORBIT_SEED,
                          ORBIT_SEED };
    orbit_seed(&s->ghosts[g], ghost_seed);
  }
}

/* ── scene_calibrate_bounds step-helpers ────────────────────────────── *
 *
 * Bounding-box calibration is a four-step recipe:
 *
 *   (1) BURN-IN     -- walk WARMUP_ITERS to escape the initial transient
 *                      so subsequent samples actually live on the
 *                      attractor (not on the trajectory ramping in).
 *   (2) PROBE       -- walk BBOX_SAMPLES more and track per-axis min/max.
 *                      Classic single-pass extremes scan.
 *   (3) INFLATE     -- expand by BBOX_MARGIN_FRAC so the outermost orbit
 *                      points don't land on the projection clip plane.
 *   (4) DERIVE      -- compute the bbox MIDPOINT (translation origin in
 *                      project()) and the DIAGONAL HALF-LENGTH
 *                      (= bounding-sphere radius; drives view scale +
 *                      depth-cue normaliser).
 *
 * Each step is one helper so scene_calibrate_bounds reads as the recipe.
 */

/* (1) Walk the orbit forward WARMUP_ITERS times and DISCARD those
 *     iterates -- they're transient (the trajectory hasn't settled on
 *     the attractor yet).  After this call (px, py, pz) sit on it. */
static void bbox_burn_in_transient(const AttrDef *at,
                                    float *px, float *py, float *pz)
{
  for (int i = 0; i < WARMUP_ITERS; i++)
    attractor_step(at, px, py, pz);
}

/* (2) Walk BBOX_SAMPLES more iterates while tracking per-axis extremes.
 *     Initialises mins/maxes from the starting sample so an attractor
 *     of any size works (no sentinel ±INF needed). */
static void bbox_probe_axis_extremes(const AttrDef *at,
                                      float *px, float *py, float *pz,
                                      float *xmin, float *xmax,
                                      float *ymin, float *ymax,
                                      float *zmin, float *zmax)
{
  *xmin = *xmax = *px;
  *ymin = *ymax = *py;
  *zmin = *zmax = *pz;
  for (int i = 0; i < BBOX_SAMPLES; i++) {
    attractor_step(at, px, py, pz);
    if (*px < *xmin) *xmin = *px;
    if (*px > *xmax) *xmax = *px;
    if (*py < *ymin) *ymin = *py;
    if (*py > *ymax) *ymax = *py;
    if (*pz < *zmin) *zmin = *pz;
    if (*pz > *zmax) *zmax = *pz;
  }
}

/* (3) Inflate raw min/max by BBOX_MARGIN_FRAC on each side and write
 *     the padded box into Bounds.  Padding is per-axis (proportional to
 *     that axis's extent) so a thin axis pads less in absolute units. */
static void bbox_inflate_by_margin(Bounds *b,
                                    float xmin, float xmax,
                                    float ymin, float ymax,
                                    float zmin, float zmax)
{
  float x_pad = (xmax - xmin) * BBOX_MARGIN_FRAC;
  float y_pad = (ymax - ymin) * BBOX_MARGIN_FRAC;
  float z_pad = (zmax - zmin) * BBOX_MARGIN_FRAC;
  b->xmin = xmin - x_pad;  b->xmax = xmax + x_pad;
  b->ymin = ymin - y_pad;  b->ymax = ymax + y_pad;
  b->zmin = zmin - z_pad;  b->zmax = zmax + z_pad;
}

/* (4) Derive bbox MIDPOINT (cx,cy,cz) and DIAGONAL HALF-LENGTH (radius)
 *     from the padded box.  Midpoint = projection translation origin.
 *     Radius = bounding-sphere radius -- used as the unit for both the
 *     view-fill scale and the depth-cue thresholds, so the renderer
 *     auto-rescales across the 25× preset size range (Lorenz vs Hénon).
 *
 *     Degenerate guard: a single-point bbox would yield radius = 0
 *     (division-by-zero downstream); BBOX_RADIUS_FALLBACK substitutes
 *     a sane unit when that happens. */
static void bbox_derive_centre_and_radius(Bounds *b)
{
  b->cx = 0.5f * (b->xmin + b->xmax);
  b->cy = 0.5f * (b->ymin + b->ymax);
  b->cz = 0.5f * (b->zmin + b->zmax);

  float bw = b->xmax - b->xmin;
  float bh = b->ymax - b->ymin;
  float bd = b->zmax - b->zmin;
  b->radius = 0.5f * sqrtf(bw * bw + bh * bh + bd * bd);
  if (b->radius < BBOX_RADIUS_EPS) b->radius = BBOX_RADIUS_FALLBACK;
}

/*
 * scene_calibrate_bounds -- driver for the four-step recipe declared
 * above.  Walks a TEMP orbit (the scene's actual orbits are untouched)
 * to produce s->bounds (raw bbox + centre + radius).  Called by
 * scene_reset on preset change / 'r' / SIGWINCH.
 */
static void scene_calibrate_bounds(Scene *s)
{
  const AttrDef *at = scene_current_attr(s);
  float px = ORBIT_SEED, py = ORBIT_SEED, pz = ORBIT_SEED;

  /* Step 1: burn off transient so subsequent samples live on the attractor. */
  bbox_burn_in_transient(at, &px, &py, &pz);

  /* Step 2: probe BBOX_SAMPLES iterates for per-axis min/max. */
  float xmin, xmax, ymin, ymax, zmin, zmax;
  bbox_probe_axis_extremes(at, &px, &py, &pz,
                           &xmin, &xmax, &ymin, &ymax, &zmin, &zmax);

  /* Step 3: pad extremes so the outermost orbit point isn't on the clip plane. */
  bbox_inflate_by_margin(&s->bounds, xmin, xmax, ymin, ymax, zmin, zmax);

  /* Step 4: derive bbox midpoint (= projection origin) and diagonal
   *         half-length (= bounding-sphere radius, drives view scale). */
  bbox_derive_centre_and_radius(&s->bounds);
}

/*
 * scene_reset -- re-seed trajectories, recalibrate bounds.  Called
 * after preset change / 'r' key / SIGWINCH.
 */
static void scene_reset(Scene *s)
{
  scene_seed_trajectories(s);
  scene_calibrate_bounds(s);
  s->total_pts = 0;
}

static void scene_init(Scene *s)
{
  memset(s, 0, sizeof *s);
  s->attr_idx           = 0;
  s->theme_idx          = 0;
  s->camera.phi         = VIEW_PHI_DEFAULT;
  s->camera.theta       = VIEW_THETA_DEFAULT;
  s->camera.auto_rotate = true;
  s->show_ghost         = true;
  s->paused             = false;
  s->speed              = 1;
  starfield_init(&s->stars);
  scene_reset(s);
}

/* ── scene_tick step-helpers ─────────────────────────────────────────── *
 *
 * One simulation tick is three operations: drift the camera (if auto-
 * rotate is on), pick how many iterates to run this tick, then advance
 * every trajectory in lockstep that many times.
 */

/* Hands-free azimuth drift: when auto_rotate is on, advance φ by a
 * constant angular rate (VIEW_PHI_SPEED · dt).  No-op otherwise -- the
 * user is driving φ via ← / → keys. */
static inline void camera_drift_auto_rotate(Camera *cam, float dt)
{
  if (cam->auto_rotate) cam->phi += VIEW_PHI_SPEED * dt;
}

/* Per-tick iteration count, branching on attractor family:
 *   continuous (Lorenz ODE):  SUB_STEPS_ODE × speed  RK4 sub-steps/tick
 *   discrete   (2-D maps):    ITERS_PER_TICK_MAP × speed  map iterates/tick
 * The `speed` multiplier is the user's +/- key control. */
static inline int iters_per_tick_for_family(const AttrDef *at, int speed)
{
  return at->is_continuous
       ? SUB_STEPS_ODE       * speed
       : ITERS_PER_TICK_MAP  * speed;
}

/* Advance ALL trajectories (main + N_GHOSTS ghosts) by ONE iterate.
 * Stepping in lockstep is what gives ε-divergence its meaning -- after
 * k ticks every trajectory has been stepped k times, so the gap between
 * main and any ghost is exactly k iterates of Lyapunov stretching [6]. */
static inline void step_all_trajectories_once(Scene *s, const AttrDef *at)
{
  orbit_step(&s->main, at);
  for (int g = 0; g < N_GHOSTS; g++)
    orbit_step(&s->ghosts[g], at);
}

/*
 * scene_tick -- driver for the three-step recipe declared above.
 * `dt` is wall-clock seconds; used ONLY by camera_drift_auto_rotate.
 * The orbit integrators work in attractor-step units, not wall-clock.
 */
static void scene_tick(Scene *s, float dt)
{
  if (s->paused) return;

  /* Step 1: optional hands-free azimuth drift (auto-rotate mode). */
  camera_drift_auto_rotate(&s->camera, dt);

  /* Step 2: pick per-tick iterate count by attractor family + speed. */
  const AttrDef *at = scene_current_attr(s);
  int iters = iters_per_tick_for_family(at, s->speed);

  /* Step 3: advance main + ghosts in lockstep so ε-divergence stays meaningful. */
  for (int i = 0; i < iters; i++) {
    step_all_trajectories_once(s, at);
    s->total_pts++;
  }
}

/* ── RenderCtx build + view scale ───────────────────────────────────── *
 *
 * compute_view_scale picks the projection scale so the attractor's
 * diagonal diameter fills VIEW_FILL_FRAC of the smaller screen
 * dimension after accounting for the non-square cell aspect.
 * Conservative on the worst-case diagonal so rotation never overflows.
 *
 * render_ctx_make bundles the per-frame derived view state once so
 * every draw helper can be (scene, ctx) instead of carrying 9 loose
 * floats.  Rebuilt every frame; never persisted.
 */
static inline float compute_view_scale(const Bounds *b, int cols, int rows)
{
  int usable_rows = rows - HUD_TOP_ROWS - HUD_BOTTOM_ROWS;
  if (usable_rows < 1) usable_rows = 1;

  float diameter = 2.0f * b->radius;
  float scale_x = (float)cols        * VIEW_FILL_FRAC / diameter;
  float scale_y = (float)usable_rows * VIEW_FILL_FRAC / (diameter * ASPECT);
  return fminf(scale_x, scale_y);
}

static RenderCtx render_ctx_make(const Scene *s, int cols, int rows)
{
  RenderCtx ctx;
  ctx.cols       = cols;
  ctx.rows       = rows;
  ctx.screen_cx  = cols / 2;
  ctx.screen_cy  = HUD_TOP_ROWS
                 + (rows - HUD_TOP_ROWS - HUD_BOTTOM_ROWS) / 2;
  ctx.scale      = compute_view_scale(&s->bounds, cols, rows);
  ctx.depth_unit = s->bounds.radius;
  return ctx;
}

/* ── trail-sample encoding primitives ───────────────────────────────── *
 *
 * Four pure functions that EACH map a sample's role/age/depth → ONE
 * presentation choice.  Composed by compose_main_trail_attr (below)
 * into the chtype draw_trail paints; paint_bloom_halo is invoked
 * separately for the comet-head samples.
 *
 *   trail_sample_color_pair    age          → CP_TRAIL_HEAD/MID/TAIL
 *   trail_sample_depth_attr    depth, k, age→ A_BOLD/A_NORMAL/A_DIM
 *   trail_sample_glyph         role, k      → 'O'/'.'/'x'/','
 *   paint_bloom_halo           row, col     → 4-cross '+' stamp
 *
 * Concrete thresholds (AGE_HEAD_LIMIT etc., DEPTH_CLOSE_FRAC etc.) and
 * the override logic (comet-head BOLD, oldest 20 % DIM) live inside the
 * helpers themselves.
 */

/* COLOUR tier from age. */
static inline short trail_sample_color_pair(float age)
{
  if (age < AGE_HEAD_LIMIT) return CP_TRAIL_HEAD;
  if (age < AGE_MID_LIMIT)  return CP_TRAIL_MID;
  return CP_TRAIL_TAIL;
}

/* ATTRIBUTE from normalised depth, with two age-extreme overrides. */
static inline attr_t trail_sample_depth_attr(float depth_norm, int k,
                                             float age)
{
  attr_t at;
  if      (depth_norm < DEPTH_CLOSE_FRAC) at = A_BOLD;
  else if (depth_norm > DEPTH_FAR_FRAC)   at = A_DIM;
  else                                    at = A_NORMAL;

  if      (k < BLOOM_HEAD_COUNT)          at = A_BOLD;  /* comet head */
  else if (age > AGE_TAIL_LIMIT)          at = A_DIM;   /* fading tail */
  return at;
}

/* GLYPH -- head marker (k=0) or trail dot.  Ghosts use ',' / 'x'. */
static inline char trail_sample_glyph(bool is_ghost, int k)
{
  if (k == 0) return is_ghost ? 'x' : 'O';
  return is_ghost ? ',' : '.';
}

/* 4-cross BLOOM HALO around the comet head -- '+' marks in tier-head. */
static inline void paint_bloom_halo(int row, int col, const RenderCtx *ctx)
{
  static const int hdr[4] = { -1,  1,  0,  0 };
  static const int hdc[4] = {  0,  0, -1,  1 };
  attron(COLOR_PAIR(CP_TRAIL_HEAD));
  for (int hi = 0; hi < 4; hi++) {
    int br = row + hdr[hi];
    int bc = col + hdc[hi];
    if (br >= HUD_TOP_ROWS && br < ctx->rows - HUD_BOTTOM_ROWS
        && bc >= 0 && bc < ctx->cols)
      mvaddch(br, bc, '+');
  }
  attroff(COLOR_PAIR(CP_TRAIL_HEAD));
}

/* ── draw_trail step-helpers ─────────────────────────────────────────── *
 *
 * Painting one trail point is a six-step recipe per sample:
 *
 *   (1) read sample, compute its AGE FRACTION (0 = newest, 1 = oldest)
 *   (2) PROJECT to (col, row, depth); skip if outside the renderable band
 *   (3) skip if it lands on the SAME CELL as the previous painted sample
 *       (dense ODE curves hit the same cell many times)
 *   (4) choose GLYPH (head marker vs trail dot, main vs ghost role)
 *   (5) compose the chtype ATTRIBUTE (colour pair + bold/normal/dim)
 *       -- two flavours: main (age tier + depth + comet head override)
 *                      / ghost (constant: CP_GHOST + A_DIM)
 *   (6) PAINT the cell; for main-trail comet-head samples, also stamp a
 *       4-cross BLOOM HALO around it
 */

/* (1) AGE FRACTION: 0.0 = newest sample (k=0), 1.0 = oldest (k=count-1).
 *     Guards count == 0/1 so the divisor is never zero. */
static inline float trail_sample_age_fraction(int k, int count)
{
  return (float)k / (float)(count > 1 ? count - 1 : 1);
}

/* (3) Duplicate-cell test: is this sample landing on the same terminal
 *     cell as the previous one we painted?  Cheap dedupe -- avoids
 *     stacking up redundant mvaddch calls along dense curve segments. */
static inline bool sample_overlaps_previous_cell(int col, int row,
                                                  int last_col, int last_row)
{
  return col == last_col && row == last_row;
}

/* (6) Comet-head test: is this sample one of the newest BLOOM_HEAD_COUNT
 *     (i.e. should it carry the 4-cross '+' halo)? */
static inline bool is_comet_head_sample(int k)
{
  return k < BLOOM_HEAD_COUNT;
}

/* (5a) Compose the chtype for one MAIN-trail sample.
 *      Normalise depth against the bounding-sphere radius (so the same
 *      DEPTH_CLOSE/FAR_FRAC thresholds work across all preset sizes),
 *      then combine age-tier colour pair with depth/age attribute. */
static inline chtype compose_main_trail_attr(float depth, float depth_unit,
                                              int k, float age)
{
  float  depth_norm = depth / depth_unit;
  short  cp = trail_sample_color_pair(age);
  attr_t at = trail_sample_depth_attr(depth_norm, k, age);
  return COLOR_PAIR(cp) | at;
}

/* (5b) Ghost samples are constant: ghost colour pair + A_DIM (so the
 *      shadow trajectories sit visually behind the main trail). */
static inline chtype compose_ghost_trail_attr(void)
{
  return COLOR_PAIR(CP_GHOST) | A_DIM;
}

/* (6) Bracketed cell paint: attron → mvaddch → attroff in one helper so
 *     no caller can leak an attribute by forgetting the off side.  The
 *     `(chtype)(unsigned char)` cast prevents sign-extension on chars
 *     with the high bit set (see CLAUDE.md "Common ncurses Bugs"). */
static inline void paint_trail_cell(int row, int col, chtype attr, char ch)
{
  attron(attr);
  mvaddch(row, col, (chtype)(unsigned char)ch);
  attroff(attr);
}

/*
 * draw_trail -- walk one trail newest → oldest and paint each sample.
 * `is_ghost` selects the ghost flavour (CP_GHOST + A_DIM + ',' glyph,
 * no bloom halo); main trail gets full age/depth encoding + bloom halo
 * around the comet head.  Body reads as the six-step recipe above.
 */
static void draw_trail(const Scene *s, const Trail *t, bool is_ghost,
                       const RenderCtx *ctx)
{
  const Camera *cam = &s->camera;
  const Bounds *b   = &s->bounds;
  int last_col = CELL_NONE, last_row = CELL_NONE;

  for (int k = 0; k < t->count; k++) {
    /* Step 1: read k-th newest sample + compute its age fraction. */
    Point3 p   = trail_at(t, k);
    float  age = trail_sample_age_fraction(k, t->count);

    /* Step 2: project into cell coords; skip if outside the renderable band. */
    int   col, row;
    float depth;
    if (!project(p.x, p.y, p.z,
                 b->cx, b->cy, b->cz,
                 cam->phi, cam->theta, ctx->scale,
                 ctx->screen_cx, ctx->screen_cy, ctx->cols, ctx->rows,
                 &col, &row, &depth))
      continue;

    /* Step 3: dedupe consecutive samples that land on the same cell. */
    if (sample_overlaps_previous_cell(col, row, last_col, last_row))
      continue;
    last_col = col;
    last_row = row;

    /* Step 4: pick glyph by role (main vs ghost) and position (head vs tail). */
    char ch = trail_sample_glyph(is_ghost, k);

    /* Step 5: compose chtype (age tier + depth attr for main; constant for ghost). */
    chtype attr = is_ghost
                ? compose_ghost_trail_attr()
                : compose_main_trail_attr(depth, ctx->depth_unit, k, age);

    /* Step 6: paint cell; comet-head samples also get the 4-cross bloom halo. */
    paint_trail_cell(row, col, attr, ch);
    if (!is_ghost && is_comet_head_sample(k))
      paint_bloom_halo(row, col, ctx);
  }
}

/*
 * scene_draw -- back-to-front layer stack, body reads as the recipe:
 *
 *   1  starfield               ambient background with parallax
 *   2  ghost trails            widest-ε first so closer ghosts overlay
 *   3  main trail              full depth / age / bloom encoding
 *
 * The per-frame RenderCtx is built once at the top and passed to
 * every draw helper -- nothing else recomputes scale or screen
 * centre.
 */
static void scene_draw(const Scene *s, int cols, int rows)
{
  RenderCtx ctx = render_ctx_make(s, cols, rows);

  /* Layer 1: ambient starfield backdrop. */
  starfield_draw(&s->stars, &s->camera, &ctx);

  /* Layer 2: ghost trails (widest-ε first → closer overlay them). */
  if (s->show_ghost) {
    for (int g = N_GHOSTS - 1; g >= 0; g--)
      draw_trail(s, &s->ghosts[g].trail, true, &ctx);
  }

  /* Layer 3: main trail (with depth + age + bloom). */
  draw_trail(s, &s->main.trail, false, &ctx);
}

/* ===================================================================== */
/* §8  hud                                                                */
/* ===================================================================== */

/*
 * Top HUD (rows 0..HUD_TOP_ROWS-1) -- live data only.
 *   Row 0:  title (cyan) + preset + theme + ghost + rot + status (yellow)
 *   Row 1:  attractor parameters + speed + fps + total points (yellow)
 *
 * Bottom HUD (row rows-1) -- action keys, bright cyan + A_BOLD.
 */

#define HUD_DATA_COL  24

/* Row 0 left -- title in cyan + bold. */
static void hud_draw_title(void)
{
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(0, 0, " [STRANGE ATTRACTOR] ");
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* Row 0 middle -- preset, theme, ghost, rotate, status. */
static void hud_draw_state(const Scene *s)
{
  const AttrDef *at = scene_current_attr(s);
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvprintw(0, HUD_DATA_COL,
           " [%2d] %-11s  theme:%-7s  ghost:%-3s  rot:%-6s  %-7s ",
           s->attr_idx + 1, at->name,
           THEMES[s->theme_idx].name,
           s->show_ghost          ? "on"   : "off",
           s->camera.auto_rotate  ? "auto" : "manual",
           s->paused              ? "PAUSED " : "running");
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Row 1 -- attractor parameters + iter speed + fps + accumulated count. */
static void hud_draw_params(const Scene *s, double fps)
{
  const AttrDef *at = scene_current_attr(s);
  attron(COLOR_PAIR(CP_HUD));
  mvprintw(1, 0,
           " params: a=%6.2f  b=%6.2f  c=%6.2f  d=%6.2f   speed:%dx   fps:%5.1f   pts:%lld ",
           (double)at->a, (double)at->b, (double)at->c, (double)at->d,
           s->speed, fps, s->total_pts);
  attroff(COLOR_PAIR(CP_HUD));
}

/* Row rows-1 -- action keys, bright cyan + A_BOLD. */
static void hud_draw_action_bar(int rows)
{
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(rows - 1, 0,
           " q:quit  spc:pause  r:reset  n/p:preset  1-9/0:jump  "
           "t/T:theme  g:ghost  a:rot  arrows:view  +/-:speed ");
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* Compose all HUD regions for a frame. */
static void hud_draw(const Scene *s, int rows, double fps)
{
  hud_draw_title();
  hud_draw_state(s);
  hud_draw_params(s, fps);
  hud_draw_action_bar(rows);
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

/*
 * Screen -- terminal-geometry container.
 *
 * DELIBERATELY DISTINCT from §7 Scene so the two domains stay
 * decoupled:
 *
 *   Scene  domain = SIMULATION (attractor units, world coords, orbit
 *                   state -- written by physics).
 *   Screen domain = TERMINAL   (cell coords, cols × rows -- written
 *                   by ncurses + SIGWINCH).
 *
 * The two meet ONLY in RenderCtx -- which reads from both to compute
 * the per-frame projection scale.  Keeping them apart means a terminal
 * resize doesn't accidentally invalidate physics state, and a preset
 * change doesn't trigger an ncurses repaint cycle.
 *
 * LIFECYCLE:
 *   screen_init    one-shot at startup; calls initscr() + color_init()
 *                  + getmaxyx().  Sets ncurses modes (noecho, cbreak,
 *                  curs_set(0), nodelay, keypad, typeahead(-1)).
 *   screen_resize  on SIGWINCH; calls endwin() + refresh() + getmaxyx()
 *                  to re-query the new terminal size.
 *   screen_free    on shutdown; calls endwin() to restore the terminal.
 *
 * NO ALLOCATION: Screen is just an (int, int) pair.  All ncurses
 * buffering lives inside ncurses itself (stdscr is global); we never
 * own a heap-allocated WINDOW.  This keeps cleanup trivial (endwin
 * suffices) and avoids the "who frees what on SIGINT" question.
 */
typedef struct {
  int cols;    /* terminal column count (getmaxyx) -- 0-indexed extent */
  int rows;    /* terminal row count    (getmaxyx) -- 0-indexed extent */
} Screen;

static void screen_init(Screen *sc)
{
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_free(Screen *sc) { (void)sc; endwin(); }

static void screen_resize(Screen *sc)
{
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

/* Compose one frame: paint scene + HUD, present. */
static void screen_draw(Screen *sc, const Scene *s, double fps)
{
  erase();
  scene_draw(s, sc->cols, sc->rows);
  hud_draw(s, sc->rows, fps);
}

static void screen_present(void)
{
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §10  app                                                               */
/* ===================================================================== */

/*
 * App -- top-level lifecycle container.
 *
 * WHY a struct (and not loose globals): the App lifecycle is well-
 * defined -- born at main() init, dies at main() return.  Bundling
 * lets a single App* propagate through the loop helpers (app_do_resize,
 * app_handle_key) without a forest of unrelated globals.
 *
 * WHY signal flags live inside App (with the volatile sig_atomic_t
 * dance):
 *   The C signal-handler signature is `void handler(int)` -- no place
 *   to pass an App* into it.  So we keep a SINGLE static App g_app
 *   instance at file scope; on_exit_signal / on_resize_signal write
 *   to its flags.
 *
 *   The flags themselves MUST be `volatile sig_atomic_t` because:
 *     - volatile      -- the main-loop read can't be optimised away
 *     - sig_atomic_t  -- guaranteed atomic w.r.t. signal interruption
 *
 *   Signal handlers ONLY SET FLAGS; the main loop reads them and
 *   does the real work.  Doing the work IN the handler is unsafe --
 *   endwin / ncurses calls / malloc / free are not signal-safe.
 *
 *     running       clear to 0 to break the main loop and exit
 *                   cleanly through atexit(cleanup).
 *     need_resize   set on SIGWINCH; main loop services it by
 *                   re-fitting Screen + recalibrating bbox on the
 *                   next iteration.
 */
typedef struct {
  Scene                 scene;
  Screen                screen;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
  screen_resize(&app->screen);
  app->need_resize = 0;
}

/* ---- key actions (each one a named helper for clarity) -------------- */

static void action_pause            (Scene *s) { s->paused = !s->paused; }
static void action_reset            (Scene *s) { scene_reset(s); }
static void action_toggle_ghost     (Scene *s) { s->show_ghost         = !s->show_ghost;         }
static void action_toggle_auto_rot  (Scene *s) { s->camera.auto_rotate = !s->camera.auto_rotate; }

static void action_preset_next(Scene *s)
{
  s->attr_idx = (s->attr_idx + 1) % N_ATTRS;
  scene_reset(s);
}

static void action_preset_prev(Scene *s)
{
  s->attr_idx = (s->attr_idx + N_ATTRS - 1) % N_ATTRS;
  scene_reset(s);
}

static void action_preset_jump(Scene *s, int idx)
{
  if (idx < 0)        idx = 0;
  if (idx >= N_ATTRS) idx = N_ATTRS - 1;
  if (idx == s->attr_idx) return;
  s->attr_idx = idx;
  scene_reset(s);
}

static void action_theme_next(Scene *s)
{
  s->theme_idx = (s->theme_idx + 1) % N_THEMES;
  theme_apply(s->theme_idx);
}

static void action_theme_prev(Scene *s)
{
  s->theme_idx = (s->theme_idx + N_THEMES - 1) % N_THEMES;
  theme_apply(s->theme_idx);
}

static void action_phi_left(Scene *s)
{
  s->camera.auto_rotate = false;
  s->camera.phi -= VIEW_PHI_STEP;
}

static void action_phi_right(Scene *s)
{
  s->camera.auto_rotate = false;
  s->camera.phi += VIEW_PHI_STEP;
}

static void action_theta_up(Scene *s)
{
  s->camera.theta += VIEW_THETA_STEP;
  if (s->camera.theta > VIEW_THETA_MAX) s->camera.theta = VIEW_THETA_MAX;
}

static void action_theta_down(Scene *s)
{
  s->camera.theta -= VIEW_THETA_STEP;
  if (s->camera.theta < VIEW_THETA_MIN) s->camera.theta = VIEW_THETA_MIN;
}

static void action_speed_faster(Scene *s)
{
  s->speed *= 2;
  if (s->speed > SPEED_MAX) s->speed = SPEED_MAX;
}

static void action_speed_slower(Scene *s)
{
  s->speed /= 2;
  if (s->speed < SPEED_MIN) s->speed = SPEED_MIN;
}

/* Returns false if the user asked to quit. */
static bool app_handle_key(App *app, int ch)
{
  Scene *s = &app->scene;
  switch (ch) {
  case 'q': case 'Q': case KEY_ESC:  return false;

  case ' ':           action_pause(s);           break;
  case 'r': case 'R': action_reset(s);           break;
  case 'n': case 'N': action_preset_next(s);     break;
  case 'p': case 'P': action_preset_prev(s);     break;
  case 't':           action_theme_next(s);      break;
  case 'T':           action_theme_prev(s);      break;
  case 'g': case 'G': action_toggle_ghost(s);    break;
  case 'a': case 'A': action_toggle_auto_rot(s); break;

  case KEY_LEFT:      action_phi_left(s);        break;
  case KEY_RIGHT:     action_phi_right(s);       break;
  case KEY_UP:        action_theta_up(s);        break;
  case KEY_DOWN:      action_theta_down(s);      break;

  case '+': case '=': action_speed_faster(s);    break;
  case '-': case '_': action_speed_slower(s);    break;

  default:
    /* '1'..'9' jump to presets 1..9; '0' jumps to the 10th preset. */
    if (ch >= '1' && ch <= '9' && (ch - '1') < N_ATTRS)
      action_preset_jump(s, ch - '1');
    else if (ch == '0' && N_ATTRS >= 10)
      action_preset_jump(s, 9);
    break;
  }
  return true;
}

/* ── main-loop step-helpers ──────────────────────────────────────────── *
 *
 * The frame loop is a seven-step recipe.  Pulling each step into a
 * named helper lets main() read as the recipe and pushes the noisy
 * timing arithmetic out of the orchestrator.
 */

/* Bind atexit + the three signal handlers we care about.  Done once
 * before the loop -- handlers only flip volatile sig_atomic_t flags;
 * the main loop services them between frames (signal-safe). */
static void install_signal_handlers(void)
{
  atexit(cleanup);
  signal(SIGINT,   on_exit_signal);
  signal(SIGTERM,  on_exit_signal);
  signal(SIGWINCH, on_resize_signal);
}

/* Initial app state: clear flags, build screen, seed scene. */
static void app_init(App *app)
{
  app->running     = 1;
  app->need_resize = 0;
  screen_init(&app->screen);
  scene_init(&app->scene);
}

/* Step 1: react to a pending SIGWINCH (set by on_resize_signal).
 * Re-queries terminal size, then resets the frame clock + sim accumulator
 * so the dt arithmetic doesn't see the resize gap as accumulated time. */
static inline void app_service_pending_resize(App *app, int64_t *frame_time,
                                               int64_t *sim_accum)
{
  if (!app->need_resize) return;
  app_do_resize(app);
  *frame_time = clock_ns();
  *sim_accum  = 0;
}

/* Step 2: measure wall-clock dt since previous frame start.  Caps the
 * result at MAX_FRAME_DT_NS so a long pause (debugger break, terminal
 * suspend) can't push sim_accum so high that the next frame tries to
 * run thousands of catch-up ticks (the classic "spiral of death"). */
static inline int64_t frame_dt_capped(int64_t now, int64_t prev)
{
  int64_t dt = now - prev;
  return (dt > MAX_FRAME_DT_NS) ? MAX_FRAME_DT_NS : dt;
}

/* Step 3: drain wall-clock time through the fixed-step simulator.
 * Each TICK_NS chunk → one scene_tick at FIXED_TICK_DT_SEC.  This is
 * what gives the demo timing-independent physics: regardless of frame
 * jitter, scene_tick fires at exactly 60 Hz worth of sim time. */
static inline void pump_fixed_simulation(Scene *scene, int64_t *sim_accum,
                                          int64_t dt)
{
  *sim_accum += dt;
  while (*sim_accum >= TICK_NS) {
    scene_tick(scene, FIXED_TICK_DT_SEC);
    *sim_accum -= TICK_NS;
  }
}

/* Step 4: refresh the smoothed HUD fps readout once every
 * FPS_UPDATE_MS of wall-clock time.  Smoothing prevents the per-frame
 * jitter from spamming the HUD with noise. */
static inline void fps_counter_update(int64_t dt, int64_t *fps_accum,
                                       int *frame_count, double *fps_display)
{
  (*frame_count)++;
  *fps_accum += dt;
  if (*fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
    *fps_display = (double)(*frame_count) /
                   ((double)(*fps_accum) / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
  }
}

/* Step 5: sleep so the frame matches TICK_NS (60 fps wall-clock budget).
 *
 * `elapsed` counts BOTH the work done in this frame so far (clock_ns -
 * frame_start) AND the dt that the previous frame ran over budget
 * (`+ dt`); subtracting from TICK_NS gives the remaining sleep budget.
 * If we ran over, clock_sleep_ns sees a non-positive value and returns
 * immediately (it guards `ns <= 0`). */
static inline void throttle_to_target_fps(int64_t frame_start, int64_t dt)
{
  int64_t elapsed = clock_ns() - frame_start + dt;
  clock_sleep_ns(TICK_NS - elapsed);
}

/* Step 6: paint scene + HUD onto stdscr and flush the diff to the
 * terminal.  Single I/O boundary -- nothing else in the loop calls
 * ncurses output. */
static inline void present_frame(App *app, double fps_display)
{
  screen_draw(&app->screen, &app->scene, fps_display);
  screen_present();
}

/* Step 7: non-blocking read of one keystroke; clears running on quit.
 * One key per frame is enough at 60 fps -- the key handler dispatches
 * the action and the next frame's read picks up the following key. */
static inline void pump_one_keystroke(App *app)
{
  int ch = getch();
  if (ch != ERR && !app_handle_key(app, ch))
    app->running = 0;
}

int main(void)
{
  install_signal_handlers();

  App *app = &g_app;
  app_init(app);

  int64_t frame_time  = clock_ns();
  int64_t sim_accum   = 0;
  int64_t fps_accum   = 0;
  int     frame_count = 0;
  double  fps_display = 0.0;

  while (app->running) {
    /* Step 1: service any pending terminal-resize signal. */
    app_service_pending_resize(app, &frame_time, &sim_accum);

    /* Step 2: measure (and cap) wall-clock dt since previous frame. */
    int64_t now = clock_ns();
    int64_t dt  = frame_dt_capped(now, frame_time);
    frame_time  = now;

    /* Step 3: drain dt through the 60 Hz fixed-step simulator. */
    pump_fixed_simulation(&app->scene, &sim_accum, dt);

    /* Step 4: update the smoothed fps readout (HUD only). */
    fps_counter_update(dt, &fps_accum, &frame_count, &fps_display);

    /* Step 5: sleep to honour the 60 fps wall-clock frame budget. */
    throttle_to_target_fps(frame_time, dt);

    /* Step 6: paint the frame and present it to the terminal. */
    present_frame(app, fps_display);

    /* Step 7: poll one keystroke (non-blocking); quit if requested. */
    pump_one_keystroke(app);
  }

  screen_free(&app->screen);
  return 0;
}
