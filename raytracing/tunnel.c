/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */

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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 [DATA] config ─────────────────────────────────────────────────────────
 */

enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,

  FPS_UPDATE_MS = 500,

  /* Color pair indices. */
  PAIR_HUD = 1,        /* yellow + bold — top status row     */
  PAIR_HINT = 2,       /* cyan   + bold — bottom hint row    */
  PAIR_DEPTH_BASE = 3, /* +0..+7 — depth ramp (far → near)   */
  PAIR_VANISH = 11,    /* vanishing-point crosshair          */
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

#define CELL_ASPECT 2.0f /* terminal cell h / w                */

/* Tunnel geometry. */
#define TUNNEL_RADIUS 2.0f
#define FOV_DEG 70.0f

/* Camera flow + sway. */
#define SPEED_DEFAULT 8.0f
#define SPEED_MIN 0.5f
#define SPEED_MAX 40.0f
#define SPEED_STEP_FACTOR 1.30f

#define SWAY_AMP_DEFAULT 1.30f /* max horizontal offset (cells)      */
#define SWAY_AMP_MIN 0.0f
#define SWAY_AMP_MAX 1.65f /* < TUNNEL_RADIUS · 0.85             */
#define SWAY_AMP_STEP 0.10f
#define SWAY_FREQ_X 0.55f /* rad / sec                          */
#define SWAY_FREQ_Y 0.31f
#define ROLL_RATE 0.18f /* rad / sec — texture spin           */

/* Numerical tolerances. */
#define EPS_PARALLEL 1.5e-3f /* ray-axis-parallel threshold        */
#define T_MAX 80.0f          /* far clip — vanishing point         */

/* Distance fog (T8). */
#define FOG_DENSITY 0.030f /* exp(−t · FOG_DENSITY)              */
#define FOG_FLOOR 0.18f    /* never fade below this — keep walls */

/* Depth-tier quantisation (T10): intensity → one of 8 ramp tiers. */
#define N_DEPTH_TIERS 8         /* depth ramp pairs PAIR_DEPTH_BASE+0..7      */
#define DEPTH_TIER_SCALE 7.999f /* intensity∈[0,1] → tier 0..7 (= 8 − ε) */
#define TIER_BOLD_MIN 6 /* tiers ≥ this render A_BOLD (nearest walls) */
#define TIER_DIM_MAX 1  /* tiers ≤ this render A_DIM  (farthest)      */

/*
 * Pattern — which procedural texture is painted on the cylinder wall. All four
 * are evaluated in cylindrical UV space (u = angle around the axis, v =
 * distance along it) by pattern_sample (§10): the geometry is identical, only
 * the texture function differs. Cycled live with n/N (wraps via % N_PATTERNS).
 * Ref: procedural texturing — Ebert et al., "Texturing & Modeling".
 */
typedef enum {
  PATTERN_RINGS = 0,   /* sinusoidal axial bands (rings down the tube)    */
  PATTERN_CHECKER = 1, /* alternating squares (floor(u)·floor(v) parity)  */
  PATTERN_SPOKES = 2,  /* radial stripes (parity of the angle only)       */
  PATTERN_GRID = 3,    /* cross-hatch lines (thin-u + thin-v threshold)   */
  N_PATTERNS = 4,      /* count — enables `% N_PATTERNS` cycling          */
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_RINGS:
    return "RINGS  ";
  case PATTERN_CHECKER:
    return "CHECKER";
  case PATTERN_SPOKES:
    return "SPOKES ";
  case PATTERN_GRID:
    return "GRID   ";
  default:
    return "?      ";
  }
}

/*
 * PatternParams — the per-pattern texture tunables: one row of the patterns[]
 * lookup table (§1), chosen by Scene.current_pattern and passed (const) to
 * every pattern_* function. All quantities live in UV space, so the same
 * numbers read identically at any tunnel depth. Ref: procedural texturing
 * (Ebert et al.).
 *
 *   stripes_around : how many bright/dark cells per full revolution around the
 *                    cylinder axis (must be EVEN so the seam at u = 0/1 lines
 * up) stripes_long   : cells per world-space unit along the axis ring_freq :
 * rad / unit for the sinusoidal RINGS pattern line_thick     : fraction of a
 * cell occupied by a GRID line
 */
typedef struct {
  int stripes_around; /* cells per revolution (EVEN) — seam alignment    */
  float stripes_long; /* cells per axial world unit                      */
  float ring_freq;    /* rad / unit — RINGS sinusoid frequency           */
  float line_thick;   /* fraction of a cell that is a GRID line          */
} PatternParams;

static const PatternParams patterns[N_PATTERNS] = {
    /*                  around  long   ring_freq  line_thick */
    /* RINGS   */ {16, 0.55f, 1.55f, 0.30f},
    /* CHECKER */ {16, 0.55f, 0.0f, 0.0f},
    /* SPOKES  */ {24, 0.0f, 0.0f, 0.0f},
    /* GRID    */ {16, 0.55f, 0.0f, 0.20f},
};

/*
 * Themes — 8-tier depth ramp (far/dim → near/bright) + a vanishing-point
 * accent colour.  All entries sit in the bright half of
 * the 256-colour cube per the CLAUDE.md theme-brightness rule.
 */
typedef struct {
  const char *name;
  short depth[8];
  short vanish;
} Theme;

#define N_THEMES 6

static const Theme themes[N_THEMES] = {
    /* WARP — magenta/violet hyperdrive */
    {"WARP    ", {53, 91, 134, 165, 207, 213, 219, 225}, 231},

    /* INFERNO — red/orange/yellow lava tube */
    {"INFERNO ", {88, 124, 160, 202, 208, 214, 220, 226}, 231},

    /* ELECTRIC — cyan/blue/white energy conduit */
    {"ELECTRIC", {24, 26, 31, 39, 45, 51, 123, 195}, 231},

    /* VOID — deep teal-blue cold space */
    {"VOID    ", {24, 25, 31, 38, 45, 51, 117, 195}, 195},

    /* MATRIX — neon-green cyber */
    {"MATRIX  ", {28, 34, 76, 82, 118, 154, 190, 226}, 231},

    /* GOLD — amber/bone treasure passage */
    {"GOLD    ", {130, 137, 173, 179, 215, 222, 229, 230}, 231},
};

/* Glyph ramp (dim → bright). */
static const char LUMA_GLYPHS[8] = {' ', '.', ',', ':', '+', '*', '#', '@'};

/*
 * Vanishing-point cursor — a small 5-cell crosshair (`+` centre, `o`
 * at N/E/S/W) drawn LAST over the rendered tunnel.  Non-intrusive
 * marker that anchors the eye on the tunnel axis (T9).
 */
#define VANISH_CENTER_GLYPH '+'
#define VANISH_ARM_GLYPH 'o'

/* ── §2 [PERF] clock — monotonic timer + sleep ────────────────────────────────
 */

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

/* ── §3 [RENDER+DATA] color — themes + 2-pair HUD spec
 * ─────────────────────────────── *
 *
 * 8 depth-ramp pairs (PAIR_DEPTH_BASE..+7) hold the active theme's
 * colours.  Two HUD pairs (PAIR_HUD yellow, PAIR_HINT cyan) are
 * theme-independent per the CLAUDE.md HUD spec.  One extra pair
 * (PAIR_VANISH) is a theme-specific colour used by the crosshair.
 */
static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  if (COLORS >= 256) {
    const Theme *t = &themes[idx];
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_DEPTH_BASE + i), t->depth[i], -1);
    init_pair(PAIR_VANISH, t->vanish, -1);
  } else {
    static const short fb[8] = {
        COLOR_BLUE, COLOR_BLUE, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,   COLOR_WHITE,
    };
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_DEPTH_BASE + i), fb[i], -1);
    init_pair(PAIR_VANISH, COLOR_WHITE, -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
  theme_apply(0);
}

/* ── §4 [LOGIC] vec3 — value-type 3-D math
 * ───────────────────────────────────── */

/*
 * V3 — a 3-component single-precision vector, the one math type the renderer is
 * built on. Here it is a POSITION (camera origin O, hit point) or a DIRECTION
 * (ray D, the camera basis fwd/right/up, surface normal) in world space;
 * directions are kept unit length by v3norm where the math assumes |v| = 1.
 * Ref: Shirley, "Ray Tracing in One Weekend" (vec3); Foley & van Dam (vectors).
 */
typedef struct {
  float x, y, z; /* position or direction in world space */
} V3;

static inline V3 v3(float x, float y, float z) { return (V3){x, y, z}; }
static inline V3 v3add(V3 a, V3 b) {
  return (V3){a.x + b.x, a.y + b.y, a.z + b.z};
}
static inline V3 v3scale(float s, V3 a) {
  return (V3){s * a.x, s * a.y, s * a.z};
}
static inline float v3len(V3 a) {
  return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
}
static inline V3 v3norm(V3 a) {
  float L = v3len(a);
  return L > 1e-12f ? v3scale(1.0f / L, a) : v3(0, 0, 1);
}

/* ── §5 [LOGIC] ray-cylinder intersection — closed-form (T1, T2)
 * ─────────────── *
 *
 * Cylinder along z-axis, radius R, infinite extent.  Substitute
 * r(t) = O + t·D into x² + y² = R²:
 *
 *      (O.x + t·D.x)² + (O.y + t·D.y)² = R²
 *      a·t² + 2b·t + c = 0
 *      a = D.x² + D.y²        (radial direction speed²)
 *      b = O.x·D.x + O.y·D.y  (radial inner-product)
 *      c = O.x² + O.y² − R²   (radial offset² − R² — negative inside)
 *
 *      t = (−b + sqrt(b² − a·c)) / a    (camera inside → +sqrt root)
 *
 * Returns t (positive forward distance) or -1 if the ray is too
 * close to the cylinder axis (D.x² + D.y² < EPS_PARALLEL).  The
 * parallel flag tells the caller to render that pixel as the
 * vanishing-point fog tier.
 */
static inline float cylinder_hit(V3 O, V3 D, float R, bool *parallel) {
  float a = D.x * D.x + D.y * D.y;
  if (a < EPS_PARALLEL) {
    *parallel = true;
    return -1.0f;
  }
  *parallel = false;
  float b = O.x * D.x + O.y * D.y;
  float c = O.x * O.x + O.y * O.y - R * R;
  float disc = b * b - a * c;
  if (disc < 0.0f)
    return -1.0f; /* impossible inside, defensive */
  float t = (-b + sqrtf(disc)) / a;
  return t;
}

/* ── §6 [LOGIC] pattern: RINGS — sinusoidal axial bands (T7)
 * ─────────────────── *
 *
 *      brightness = sin(v_tex · ring_freq) · 0.5 + 0.5
 *
 * No u dependence at all — camera roll is INVISIBLE against this
 * pattern, since rolling rotates u_tex but RINGS ignores u.
 * Visible motion = axial flow + sway.
 */
static inline float pattern_rings(float v_tex, const PatternParams *pp) {
  return sinf(v_tex * pp->ring_freq) * 0.5f + 0.5f;
}

/* ── §7 [LOGIC] pattern: CHECKER — alternating squares (T7)
 * ──────────────────── *
 *
 *      brightness = (floor(u·N) + floor(v·M)) & 1
 *
 * The XOR of two integer-floor parities produces alternating
 * bright/dark squares.  Both u and v matter, so roll, flow, and
 * sway are all visible.
 */
static inline float pattern_checker(float u_tex, float v_tex,
                                    const PatternParams *pp) {
  int gu = (int)floorf(u_tex * (float)pp->stripes_around);
  int gv = (int)floorf(v_tex * pp->stripes_long);
  return ((gu + gv) & 1) ? 1.0f : 0.0f;
}

/* ── §8 [LOGIC] pattern: SPOKES — radial stripe parity (T7)
 * ──────────────────── *
 *
 *      brightness = floor(u·N) & 1
 *
 * Pure azimuthal stripes; no v dependence → axial flow and sway
 * are INVISIBLE against this pattern.  Roll, however, IS visible.
 */
static inline float pattern_spokes(float u_tex, const PatternParams *pp) {
  int gu = (int)floorf(u_tex * (float)pp->stripes_around);
  return (gu & 1) ? 1.0f : 0.0f;
}

/* ── §9 [LOGIC] pattern: GRID — line-thickness threshold cross-hatch (T7)
 * ────── *
 *
 *      u_dist = min(frac(u·N), 1 − frac(u·N))           proximity to u line
 *      v_dist = min(frac(v·M), 1 − frac(v·M))           proximity to v line
 *      brightness = (u_dist < thick OR v_dist < thick) ? 1 : 0
 *
 * Bright cross-hatch lines on a dark base.  thick = 0.20 means each
 * line is 20% of one cell wide.  Both u and v matter.
 */
static inline float pattern_grid(float u_tex, float v_tex,
                                 const PatternParams *pp) {
  float u_pos = u_tex * (float)pp->stripes_around;
  float v_pos = v_tex * pp->stripes_long;
  float u_frac = u_pos - floorf(u_pos);
  float v_frac = v_pos - floorf(v_pos);
  float u_dist = fminf(u_frac, 1.0f - u_frac);
  float v_dist = fminf(v_frac, 1.0f - v_frac);
  float thick = pp->line_thick;
  if (u_dist < thick || v_dist < thick)
    return 1.0f;
  return 0.0f;
}

/* ── §10 [LOGIC] pattern dispatch + cylindrical UV mapping (T3)
 * ──────────────── *
 *
 * pattern_sample dispatches by pattern_idx.  Returns brightness in
 * [0, 1] (not 0/1 binary): the lerp into FOG_FLOOR..1 at the call
 * site gives smooth depth shading.  RINGS returns a sinusoid for an
 * extra-smooth visual; the others return 0 or 1 plus the fog
 * smoothing.
 */
static float pattern_sample(float u_tex, float v_tex, int pattern_idx) {
  const PatternParams *pp = &patterns[pattern_idx];
  switch (pattern_idx) {
  case PATTERN_RINGS:
    return pattern_rings(v_tex, pp);
  case PATTERN_CHECKER:
    return pattern_checker(u_tex, v_tex, pp);
  case PATTERN_SPOKES:
    return pattern_spokes(u_tex, pp);
  case PATTERN_GRID:
  default:
    return pattern_grid(u_tex, v_tex, pp);
  }
}

/* ── §11 [LOGIC] distance fog — exp(−t · density) with floor (T8)
 * ────────────── *
 *
 *      fog       = exp(−t · FOG_DENSITY)
 *      intensity = fog · (FOG_FLOOR + brightness · (1 − FOG_FLOOR))
 *
 * fog → 0 at far distances; FOG_FLOOR keeps the result from going
 * fully black so we still see the tunnel structure at the
 * vanishing point.
 */
static inline float fog_intensity(float t, float brightness) {
  float fog = expf(-t * FOG_DENSITY);
  return fog * (FOG_FLOOR + brightness * (1.0f - FOG_FLOOR));
}

/* ── §12 [SIM] scene state — struct + init + tick + reset ────────────────────
 */

/*
 * Scene — the tunnel flight's persistent state, the table of contents the loop
 * in §17 drives. The flight has ONE evolving value (time); every motion the
 * viewer sees — sway, roll, texture flow — is derived from it at render time
 * (§13 build_camera, §14 scene_render), so nothing cosmetic is stored here.
 *   WHAT  : time                         — the simulation clock
 *   HOW   : speed / sway_amp / paused     — flight knobs (SIMULATION)
 *           current_pattern / current_theme — look knobs (RENDER)
 *   WHERE : cols, rows                    — the viewport this frame renders
 * into
 */
typedef struct {
  /* WHAT — the flight: one evolving value; sway/roll/flow derive from it. */
  float time; /* accumulated seconds since reset            */

  /* HOW — flight knobs (SIMULATION). */
  float speed;    /* texture flow speed (units / sec)           */
  float sway_amp; /* horizontal sway amplitude (cells)          */
  bool paused;    /* freeze the time advance (the DELAYS gate)  */

  /* HOW — look knobs (RENDER). */
  int current_pattern; /* active procedural pattern (a Pattern)      */
  int current_theme;   /* active colour theme (index into themes)    */

  /* WHERE — viewport, refreshed on resize. */
  int cols, rows;
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->current_pattern = PATTERN_RINGS;
  s->current_theme = 0;
  s->cols = cols;
  s->rows = rows;
  s->time = 0.0f;
  s->speed = SPEED_DEFAULT;
  s->sway_amp = SWAY_AMP_DEFAULT;
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
}

static void scene_reset(Scene *s) { s->time = 0.0f; }

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->time += dt;
}

/* ── §13 [LOGIC] camera basis — sway position + roll basis (T5, T6)
 * ──────────── *
 *
 * Two separate motions composed into the camera state:
 *
 *   POSITION (sway) — Lissajous orbit within the cylinder:
 *       O.x = sway_amp · sin(t · SWAY_FREQ_X)
 *       O.y = sway_amp · cos(t · SWAY_FREQ_Y) · 0.55
 *       O.z = 0                       (T4 — speed faked via texture flow)
 *
 *   BASIS (roll) — rotate (right, up) around z:
 *       r = ROLL_RATE · t
 *       forward = (0, 0, 1)
 *       right   = ( cos r,  sin r, 0)
 *       up      = (−sin r,  cos r, 0)
 *
 * Built fresh each frame by build_camera (§13) from Scene.time — a pure
 * Scene→Camera mapping with no stored state. Ref: pinhole camera + look-at
 * basis (Shirley, "Fundamentals of Computer Graphics"); Lissajous curve (sway).
 */
typedef struct {
  V3 O;              /* camera origin — sway offset within the tube   */
  V3 fwd, right, up; /* orthonormal view basis (fwd = +Z, then rolled)*/
  float fov_t;       /* tan(FOV/2) — half-FOV ray-spread factor       */
  float phys_aspect; /* (rows·CELL_ASPECT)/cols — non-square-cell fix */
} Camera;

static Camera build_camera(const Scene *s, int rows_eff) {
  Camera c;
  c.O = (V3){s->sway_amp * sinf(s->time * SWAY_FREQ_X),
             s->sway_amp * cosf(s->time * SWAY_FREQ_Y) * 0.55f, 0.0f};

  float r = s->time * ROLL_RATE;
  float cr = cosf(r);
  float sr = sinf(r);
  c.fwd = v3(0.0f, 0.0f, 1.0f);
  c.right = v3(cr, sr, 0.0f);
  c.up = v3(-sr, cr, 0.0f);

  c.fov_t = tanf(FOV_DEG * (float)M_PI / 180.0f * 0.5f);
  c.phys_aspect = ((float)rows_eff * CELL_ASPECT) / (float)s->cols;
  return c;
}

/* ── §14 [RENDER] scene_render — per-pixel orchestrator
 * ───────────────────────── *
 *
 * Tutorials T1-T10 are realised by this function and its helpers
 * (primary_ray_dir, shade_tunnel_cell, cylinder_uv, draw_vanishing_crosshair):
 *   T1, T2  cylinder_hit (closed form, +sqrt root)
 *   T3      cylindrical UV mapping
 *   T4      v_flow = speed · time (fake forward motion)
 *   T5, T6  build_camera (sway + roll)
 *   T7      pattern_sample
 *   T8      fog_intensity
 *   T10     glyph + theme depth quantisation
 *
 * Two HUD rows are reserved (row 0 yellow, row rows-1 cyan); the
 * tunnel renders into rows 1..rows-2 via y_offset = 1.
 *
 * The vanishing-point crosshair (T9, §15) is drawn LAST, on top of
 * the rendered tunnel.
 */
/* primary_ray_dir — unit ray direction through screen-cell NDC (u, v) for the
 * camera basis: norm(fwd + u·fov·right + v·fov·aspect·up). One pinhole ray. */
static inline V3 primary_ray_dir(const Camera *cam, float u, float v) {
  return v3norm(v3add(
      cam->fwd, v3add(v3scale(u * cam->fov_t, cam->right),
                      v3scale(v * cam->fov_t * cam->phys_aspect, cam->up))));
}

/* cylinder_uv — cylindrical texture coords at a wall hit point P (T3): u_tex is
 * the angle around the axis mapped to [0,1) (atan2 + π, /2π); v_tex is the
 * axial distance minus the flow offset so the pattern scrolls toward the camera
 * (T4). */
static inline void cylinder_uv(V3 P, float v_flow, float *u_tex, float *v_tex) {
  *u_tex = (atan2f(P.y, P.x) + (float)M_PI) * (1.0f / (2.0f * (float)M_PI));
  *v_tex = P.z - v_flow;
}

/* shade_tunnel_cell — appearance of one wall cell along ray O+t·D: cast at the
 * cylinder, and from the hit derive glyph + colour pair + attr via pattern ×
 * fog × depth-tier quantisation (T1-T8, T10). A miss / parallel / too-far ray
 * renders as the deepest fog tier — the dark tunnel exit at the vanishing
 * point. */
static void shade_tunnel_cell(V3 O, V3 D, float v_flow, int pattern_idx,
                              char *glyph, int *pair, attr_t *attr) {
  bool parallel = false;
  float t = cylinder_hit(O, D, TUNNEL_RADIUS, &parallel);

  if (parallel || t > T_MAX || t < 0.0f) {
    *pair = PAIR_DEPTH_BASE + 0;
    *attr = A_DIM;
    *glyph = LUMA_GLYPHS[0];
    return;
  }

  V3 P = v3add(O, v3scale(t, D));
  float u_tex, v_tex;
  cylinder_uv(P, v_flow, &u_tex, &v_tex);

  float bright = pattern_sample(u_tex, v_tex, pattern_idx);
  float intensity = fog_intensity(t, bright);

  /* Quantise intensity to a depth tier (far tier 0 → near tier 7). */
  int slot = (int)(intensity * DEPTH_TIER_SCALE);
  if (slot < 0)
    slot = 0;
  if (slot > N_DEPTH_TIERS - 1)
    slot = N_DEPTH_TIERS - 1;

  *glyph = LUMA_GLYPHS[slot];
  *pair = PAIR_DEPTH_BASE + slot;
  *attr = (slot >= TIER_BOLD_MIN)  ? A_BOLD
          : (slot <= TIER_DIM_MAX) ? A_DIM
                                   : A_NORMAL;
}

/* §15 [RENDER] draw_vanishing_crosshair — the eye-anchor at screen centre (T9),
 * drawn LAST by scene_render on top of the tunnel. forward = +Z projects the
 * axis vanishing point to the centre regardless of sway, so the bold '+' and
 * four 'o' arms sit there. */
static void draw_vanishing_crosshair(int cols, int rows_eff, int y_offset) {
  int cx = cols / 2;
  int cy = rows_eff / 2;
  if (cx >= 0 && cx < cols && cy >= 0 && cy < rows_eff) {
    attron(COLOR_PAIR(PAIR_VANISH) | A_BOLD);
    mvaddch(cy + y_offset, cx, (chtype)(unsigned char)VANISH_CENTER_GLYPH);
    attroff(COLOR_PAIR(PAIR_VANISH) | A_BOLD);

    attron(COLOR_PAIR(PAIR_VANISH));
    if (cy - 1 >= 0)
      mvaddch(cy - 1 + y_offset, cx, (chtype)(unsigned char)VANISH_ARM_GLYPH);
    if (cy + 1 < rows_eff)
      mvaddch(cy + 1 + y_offset, cx, (chtype)(unsigned char)VANISH_ARM_GLYPH);
    if (cx - 1 >= 0)
      mvaddch(cy + y_offset, cx - 1, (chtype)(unsigned char)VANISH_ARM_GLYPH);
    if (cx + 1 < cols)
      mvaddch(cy + y_offset, cx + 1, (chtype)(unsigned char)VANISH_ARM_GLYPH);
    attroff(COLOR_PAIR(PAIR_VANISH));
  }
}

static void scene_render(const Scene *s) {
  /* Reserve top + bottom HUD rows.  Tunnel renders into 1..rows-2. */
  int rows_eff = s->rows - 2;
  int y_offset = 1;
  if (rows_eff < 1)
    return;

  Camera cam = build_camera(s, rows_eff);
  float v_flow =
      s->speed * s->time; /* texture flow → fake forward motion (T4) */

  /* Paint the wall, batching attr changes across runs of equal cells. */
  int last_pair = -1;
  attr_t last_attr = 0;
  for (int row = 0; row < rows_eff; row++) {
    for (int col = 0; col < s->cols; col++) {
      /* NDC for this cell (y flipped: screen-down → world-up). */
      float u = ((float)col + 0.5f) / (float)s->cols * 2.0f - 1.0f;
      float v = -(((float)row + 0.5f) / (float)rows_eff * 2.0f - 1.0f);

      V3 D = primary_ray_dir(&cam, u, v);

      char glyph;
      int pair;
      attr_t attr;
      shade_tunnel_cell(cam.O, D, v_flow, s->current_pattern, &glyph, &pair,
                        &attr);

      /* Batched attron/attroff: switch only when (pair, attr) changes, so
       * adjacent same-depth cells share one attron call. */
      if (pair != last_pair || attr != last_attr) {
        if (last_pair >= 0)
          attroff(COLOR_PAIR(last_pair) | last_attr);
        attron(COLOR_PAIR(pair) | attr);
        last_pair = pair;
        last_attr = attr;
      }
      mvaddch(row + y_offset, col, (chtype)(unsigned char)glyph);
    }
  }
  if (last_pair >= 0)
    attroff(COLOR_PAIR(last_pair) | last_attr);

  draw_vanishing_crosshair(s->cols, rows_eff, y_offset);
}

/* ── §16 [RENDER] screen — ncurses init / 2-row HUD / present
 * ─────────────────── */

/*
 * Screen — the terminal output surface: its current size, re-read on SIGWINCH
 * so render and HUD always match the real window. Owns the ncurses lifecycle
 * (init / resize / free / present) in §16.
 */
typedef struct {
  int cols, rows; /* terminal size in character cells (refreshed on resize) */
} Screen;

static void screen_init(Screen *sc) {
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
static void screen_free(Screen *sc) {
  (void)sc;
  endwin();
}
static void screen_resize_curses(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

/*
 * screen_draw — CLAUDE.md HUD spec.
 *   row 0          PAIR_HUD  (yellow + bold) — title + fps left, status right
 *   row rows-1     PAIR_HINT (cyan   + bold) — key hint
 *
 * FPS lives in the LEFT label so it stays visible even when the
 * settings status overflows the terminal width.
 */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_render(s);

  /* Top row — title + fps left, settings status right (truncated). */
  char left[48];
  snprintf(left, sizeof left, " TUNNEL  %5.1f fps ", fps);
  int llen = (int)strlen(left);

  char status[200];
  snprintf(status, sizeof status,
           " %s  pat:%s  theme:%s  speed:%4.1f  sway:%4.2f  sim:%3dHz ",
           s->paused ? "PAUSED" : "FLYING", pattern_name(s->current_pattern),
           themes[s->current_theme].name, (double)s->speed, (double)s->sway_amp,
           sim_fps);
  int slen = (int)strlen(status);
  int max_slen = sc->cols - llen;
  if (max_slen < 0)
    max_slen = 0;
  if (slen > max_slen)
    slen = max_slen;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, 0, "%s", left);
  if (slen > 0)
    mvprintw(0, sc->cols - slen, "%.*s", slen, status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* Bottom row — cyan key hint. */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(sc->rows - 1, 0,
           " q:quit  spc:pause  r:reset  n/N:pat  t/T:theme  "
           "+/-:speed  s/S:sway  ]/[:fps ");
  clrtoeol();
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §17 [APP] app — main loop, signals, key handling ────────────────────────
 */

/*
 * App — the running program in one place: the simulation (Scene), the output
 * surface (Screen), the fixed-timestep rate, and the signal-driven run flags.
 * main / app_handle_key / app_do_resize take App*; everything below takes a
 * sub-type, so the layers never re-couple through this aggregate.
 */
typedef struct {
  Scene scene;                       /* the flight simulation (WHAT + HOW)   */
  Screen screen;                     /* the output surface                   */
  int sim_fps;                       /* fixed-timestep rate ([ / ])          */
  volatile sig_atomic_t running;     /* 0 = quit (set by signal or `q`)      */
  volatile sig_atomic_t need_resize; /* 1 = SIGWINCH pending                 */
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
  screen_resize_curses(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;
  case 'r':
  case 'R':
    scene_reset(s);
    break;

  case 'n':
    s->current_pattern = (s->current_pattern + 1) % N_PATTERNS;
    break;
  case 'N':
    s->current_pattern = (s->current_pattern + N_PATTERNS - 1) % N_PATTERNS;
    break;

  case 't':
    s->current_theme = (s->current_theme + 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;
  case 'T':
    s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;

  case '=':
  case '+':
    s->speed *= SPEED_STEP_FACTOR;
    if (s->speed > SPEED_MAX)
      s->speed = SPEED_MAX;
    break;
  case '-':
    s->speed /= SPEED_STEP_FACTOR;
    if (s->speed < SPEED_MIN)
      s->speed = SPEED_MIN;
    break;

  case 's':
    s->sway_amp += SWAY_AMP_STEP;
    if (s->sway_amp > SWAY_AMP_MAX)
      s->sway_amp = SWAY_AMP_MAX;
    break;
  case 'S':
    s->sway_amp -= SWAY_AMP_STEP;
    if (s->sway_amp < SWAY_AMP_MIN)
      s->sway_amp = SWAY_AMP_MIN;
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
  srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
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
    /* One iteration = one frame. Layers combine in the order documented in
     * ARCHITECTURE; only the §12 scene_tick accumulator advances simulation. */

    /* USER EVENT — apply pending SIGWINCH resize (NOT part of the tick). */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* PERFORMANCE — wall-clock dt with spiral-of-death cap. */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    /* SIMULATION (the tick) — fixed-timestep accumulator: scene_tick advances
     * scene.time; the paused gate inside scene_tick acts as DELAYS. */
    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }

    /* PERFORMANCE — rolling fps measure (display only). */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* PERFORMANCE — frame cap (sleep the remainder of the ~60 fps budget). */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    /* RENDER combine — the ONE place state → screen: scene_render + HUD,
     * then present. Reads scene, never writes it. */
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();

    /* USER EVENTS — drain one key (knobs / reset / quit); NOT part of the tick.
     */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}

/*
 * tunnel.c — first-person flight through a textured cylinder
 *
 * DEMO: A camera sits at the origin inside an infinite cylinder of
 *       radius TUNNEL_RADIUS, looking forward (+Z).  For every
 *       terminal cell we cast a ray, intersect it with the
 *       cylinder ANALYTICALLY (closed-form quadratic — no sphere
 *       tracing), recover cylindrical UV at the hit point, sample
 *       a procedural pattern (RINGS / CHECKER / SPOKES / GRID),
 *       and apply distance fog.  Texture v-coordinate offsets by
 *       speed · time so the pattern flows toward the camera —
 *       gives the illusion of forward motion while the camera
 *       itself stays at z = 0.  Optional sway (translates camera
 *       within the can) and roll (rotates camera basis) layer
 *       extra motion on top.
 *
 *       Six colour themes, four pattern types, live speed / sway
 *       controls, vanishing-point crosshair anchored at screen
 *       centre.
 *
 * Study alongside: raytracing/sphere_raytrace.c, raytracing/cube_
 *       raytrace.c, raytracing/torus_raytrace.c, raytracing/capsule_
 *       raytrace.c — same family of analytical primary-ray
 *       intersections, each with one primitive that has a closed-
 *       form quadratic (or quartic, for torus).  Then read
 *       raymarcher/raymarcher.c for the OPPOSITE pole: when a
 *       primitive's surface has no closed-form intersection, you
 *       sphere-trace it iteratively instead.  Tutorial T1 in this
 *       file derives why a cylinder belongs in the analytical
 *       camp — the geometry has one algebraic equation, so four
 *       multiplies and one sqrt give the exact hit distance.
 *
 * Section map (see ARCHITECTURE for the layer model):
 *   §1   config         [DATA]   every tunable named, no magic numbers later
 *   §2   clock          [PERF]   monotonic timer + sleep
 *   §3   color          [RENDER] themes + 2-pair HUD spec
 *   §4   vec3           [LOGIC]  value-type 3-D math (pure)
 *   §5   ray-cylinder   [LOGIC]  analytical intersection (the central trick)
 *   §6   pattern: RINGS [LOGIC]  sinusoidal axial bands
 *   §7   pattern: CHECKER [LOGIC] floor(u)·floor(v) XOR
 *   §8   pattern: SPOKES [LOGIC] floor(u) parity (radial only)
 *   §9   pattern: GRID  [LOGIC]  hatch lines via line-thickness threshold
 *   §10  pattern dispatch [LOGIC] + cylindrical UV mapping
 *   §11  distance fog   [LOGIC]  exp(−t · density) with floor
 *   §12  scene state    [SIM]    struct + init + tick + reset (advances time)
 *   §13  camera basis   [LOGIC]  sway position + roll basis (pure:
 * Scene→Camera) §14  scene_render   [RENDER] per-pixel orchestrator §15
 * vanishing crosshair [RENDER] eye-anchor at screen centre §16  screen [RENDER]
 * ncurses init + 2-row HUD + present §17  app            [APP]    main loop
 * (tick combine), signals, key handling
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume forward motion + sway
 *   r            reset camera time
 *   n / N        next / previous pattern
 *                  (RINGS / CHECKER / SPOKES / GRID)
 *   t / T        next / previous theme
 *                  (WARP / INFERNO / ELECTRIC / VOID / MATRIX / GOLD)
 *   + / =        speed up
 *   -            speed down
 *   s / S        sway amplitude up / down (curving feel)
 *   ] / [        sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/tunnel.c \
 *       -o tunnel -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * The file is its own textbook.  Read top-to-bottom.
 *
 *   • CONCEPTS         names the algorithm + lists references.
 *   • MENTAL MODEL     intuition + ASCII diagram of the ray-cylinder
 *                      intersection in 2-D cross-section.
 *   • GUIDED TUTORIAL  ten short answers — why analytical (not
 *                      iterative), the +sqrt root choice, cylindrical
 *                      UV mapping, "speed lives in the texture",
 *                      camera basis with roll, sway, the four
 *                      patterns, distance fog, vanishing-point
 *                      crosshair, glyph quantisation.
 *   • §1..§17          the actual code, each section short and focused.
 *
 * Ten-minute version: read the GUIDED TUTORIAL.  By the end the
 * §-sections feel like reviewing notes.
 *
 * Math notation used in code:
 *      O          — ray origin (camera position; varies with sway)
 *      D          — ray direction (unit vector)
 *      R          — cylinder radius
 *      t          — ray parameter at hit (forward distance)
 *      P          — hit point = O + t · D
 *      u_tex      — angular UV in [0, 1) from atan2(P.y, P.x)
 *      v_tex      — axial UV (P.z minus the per-frame "flow" offset)
 *
 * Background you need:
 *   • basic vector arithmetic (add, scale, length, normalise)
 *   • the QUADRATIC FORMULA — the central trick (T1, T2)
 *   • read raymarcher.c first if surface raymarching is unfamiliar;
 *     this file is the closed-form-intersection counterpoint to it
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ────────────────────────────────────────────────────────── *
 *
 * Algorithm    : Per-pixel ANALYTICAL RAY-CYLINDER INTERSECTION.
 *                Cylinder is infinite, axis-aligned along +Z, radius
 *                R.  The intersection equation reduces to a quadratic
 *                in t with closed-form roots:
 *
 *                    a·t² + 2b·t + c = 0
 *                    a = D.x² + D.y²              (radial speed²)
 *                    b = O.x·D.x + O.y·D.y        (radial momentum)
 *                    c = O.x² + O.y² − R²         (radial offset²−R²)
 *                    t = (−b + sqrt(b² − a·c)) / a
 *
 *                The camera is INSIDE the cylinder, so the
 *                discriminant is always ≥ 0 and we always take the
 *                larger root (smaller would exit through the camera).
 *                See T1 for why this is the right tool here.
 *
 *                Per pixel:
 *                  1. build ray from camera + screen-space coords
 *                  2. cylinder_hit → exact t (closed form)
 *                  3. cylindrical UV from hit point
 *                  4. sample one of 4 procedural patterns
 *                  5. apply distance fog with floor
 *                  6. (intensity, theme) → glyph + colour pair
 *
 *                After the per-pixel loop, a 5-cell crosshair is
 *                drawn at screen centre as a vanishing-point anchor.
 *
 * Data         : Stateless math (Vec3 + cylinder_hit + 4 patterns +
 *                pattern dispatch).  ONE Scene struct holds time,
 *                speed, sway amplitude, current pattern + theme,
 *                paused flag.  No frame buffer — emit cells directly.
 *
 * Rendering    : One pixel per terminal cell.  Glyph from an 8-char
 *                ramp (' ', '.', ',', ':', '+', '*', '#', '@')
 *                indexed by intensity; colour from the active
 *                theme's 8-tier depth ramp (far → dim, near →
 *                bright).  attron/attroff batched on (pair, attr)
 *                changes — adjacent cells in the same depth tier
 *                share one attron call.
 *
 * Performance  : Per pixel: 4 mul + 1 sqrt for cylinder_hit, 1
 *                atan2 + a few multiplies for UV, 1 small switch
 *                for pattern, 1 expf for fog.  ~25 floating-point
 *                ops per cell.  At 80×24 = 1920 cells × 60 fps ≈
 *                3 M ops/sec — comfortable on any CPU made this
 *                century.  No iteration, no MAX_STEPS budget —
 *                this is what closed-form geometry gives you.
 *
 * References   :
 *   • Quílez, I. — "Intersectors"
 *     https://iquilezles.org/articles/intersectors/
 *     Closed-form ray-cylinder, ray-box, ray-sphere intersection
 *     formulas in usable form.
 *   • Hart, J. C. (1996) — "Sphere Tracing: A Geometric Method for
 *     the Antialiased Ray Tracing of Implicit Surfaces", *Visual
 *     Computer* 12(10):527-545.  The contrast: when shape is simple
 *     enough for closed-form, you skip Hart entirely.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The camera sits inside a long pipe and looks down its length.  For
 * every cell on the screen, fire a ray from the camera in that cell's
 * direction.  All rays eventually hit the pipe wall (or the far
 * vanishing point if they're nearly parallel to the axis).  A ray's
 * hit DISTANCE plus the WORLD POSITION of the hit determines what we
 * paint: deeper hits get dimmer (fog), and the angular + axial
 * position of the hit indexes a procedural texture wrapped around the
 * pipe's interior.  Forward motion is faked: the camera stays at z=0
 * forever, but the texture's axial coordinate is offset by speed ·
 * time so the pattern STREAMS toward us.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine standing inside a printed cardboard mailing tube.  The
 * pattern on the inside surface is fixed.  To make it look like
 * you're flying down the tube, don't move your head — instead,
 * SHIFT THE PATTERN backward.  The visual effect is identical: dots
 * on the wall stream past your eyes, and your brain reads it as
 * forward motion.  Add a slight side-to-side wobble (sway) and a
 * gentle rotation (roll) and the illusion completes.  The actual
 * geometry — an infinite cylinder centred on you — is so simple that
 * we never have to march along the ray.  One quadratic per pixel
 * gives the exact wall position.
 *
 * One ray, in cross-section through the X-Y plane (looking down the Z
 * axis from above):
 *
 *           ●─────────●  cylinder wall (radius R)
 *          ╱           ╲
 *         ╱             ╲
 *        ╱      ●─→ D    ╲    ray fired from camera O,
 *       ●       O        ●    direction D = (D.x, D.y, D.z)
 *        ╲     [inside]  ╱
 *         ╲             ╱      analytical hit:
 *          ╲           ╱         a = D.x² + D.y²
 *           ●─────────●          b = O.x·D.x + O.y·D.y
 *                                c = O.x² + O.y² − R²
 *                                t = (−b + sqrt(b²−ac)) / a
 *           ●                    P = O + t · D
 *           ↑                    u_tex = (atan2(P.y, P.x) + π) / 2π
 *           hit P                v_tex = P.z − speed · time
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 * Once per tick:
 *   1. animate     scene_t accumulates dt; pause freezes everything
 *
 * Per render frame:
 *   2. camera      O = sway position; basis = roll-rotated (right, up)
 *   3. for each cell:
 *      3a. ray         build D from screen-space (col, row) + camera basis
 *      3b. intersect   t = cylinder_hit(O, D, R)        closed form
 *      3c. axis-parallel branch: render as deepest fog tier
 *      3d. far-clip branch:      render as deepest fog tier
 *      3e. otherwise:  P = O + t·D
 *                      u_tex = atan2-azimuth, v_tex = P.z − speed·time
 *                      bright = pattern_sample(u_tex, v_tex)
 *                      intensity = fog(t) · (FLOOR + bright·(1−FLOOR))
 *                      slot = floor(intensity · 7.999)
 *                      emit glyph + theme-depth pair, attribute by slot
 *   4. crosshair   draw 5-cell vanishing-point cursor at (cx, cy)
 *
 * Per draw frame:
 *   5. HUD overlay yellow row 0 (title + fps + status), cyan key hint
 *                  at row rows-1
 *
 * KEY FORMULAS
 * ────────────
 * Ray-cylinder intersection (axis along +Z, radius R, camera inside):
 *      a = D.x² + D.y²
 *      b = O.x · D.x + O.y · D.y
 *      c = O.x² + O.y² − R²
 *      Δ = b² − a·c
 *      t = (−b + sqrt(Δ)) / a              (always the larger root)
 *
 * Cylindrical UV at hit P:
 *      u_tex = (atan2(P.y, P.x) + π) / (2π)   ∈ [0, 1)   azimuth
 *      v_tex = P.z − speed · time                          axial
 *
 * Camera basis with roll:
 *      r = ROLL_RATE · time
 *      forward = (0, 0, 1)
 *      right   = (cos r,  sin r, 0)
 *      up      = (−sin r, cos r, 0)
 *
 * Sway (camera position within the can):
 *      O.x = sway_amp · sin(time · SWAY_FREQ_X)
 *      O.y = sway_amp · cos(time · SWAY_FREQ_Y) · 0.55
 *      O.z = 0                                  (forward motion is FAKE — see
 * T4)
 *
 * Per-pixel ray direction (NDC in [−1, +1]):
 *      u =  (col + 0.5) / cols · 2 − 1
 *      v = −(row + 0.5) / rows · 2 + 1
 *      D = normalise(forward + u·F·right + v·F·aspect·up)
 *      F = tan(FOV/2)
 *
 * Distance fog with floor:
 *      fog       = exp(−t · FOG_DENSITY)
 *      intensity = fog · (FOG_FLOOR + brightness · (1 − FOG_FLOOR))
 *
 * Pattern brightness (per pattern):
 *      RINGS    = sin(v · ring_freq) · 0.5 + 0.5
 *      CHECKER  = (floor(u·N) + floor(v·M)) & 1
 *      SPOKES   = floor(u·N) & 1
 *      GRID     = (u_dist < thick || v_dist < thick) ? 1 : 0
 *
 * Glyph quantisation:
 *      slot   = floor(intensity · 7.999)              ∈ {0..7}
 *      glyph  = LUMA_GLYPHS[slot]
 *      pair   = PAIR_DEPTH_BASE + slot
 *      attr   = A_BOLD (slot ≥ 6) | A_DIM (slot ≤ 1) | A_NORMAL else
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Axis-parallel ray (D.x² + D.y² < EPS_PARALLEL): the quadratic
 *     coefficient `a` is essentially zero, dividing by it gives
 *     spurious huge t.  Guard with the parallel flag and render the
 *     cell as deepest-fog "vanishing point".  The crosshair cursor
 *     in §15 paints over this region.
 *
 *   • Camera close to the wall: sway_amp is clamped to <
 *     TUNNEL_RADIUS · 0.85.  Push past that and rays toward the wall
 *     hit at very small t → the wall fills nearly the whole screen.
 *     Defensive but not strictly required.
 *
 *   • Discriminant Δ should always be ≥ 0 since the camera is INSIDE
 *     the cylinder (every direction must hit the wall eventually).
 *     Returning −1 on Δ < 0 is purely defensive; in normal operation
 *     this branch is never taken.
 *
 *   • atan2(0, 0) is implementation-defined.  Guarded by the
 *     parallel-ray check upstream — if D.x² + D.y² is tiny we never
 *     reach the UV calculation.
 *
 *   • The Z roll basis swaps right and up but keeps forward = +z.
 *     Composed with forward ray-cylinder intersection, this just
 *     rotates the rendered TEXTURE in place — geometry is invariant
 *     under z-axis rotation by symmetry.  Reads as "camera is
 *     rolling".
 *
 *   • The crosshair at screen centre projects to the axis vanishing
 *     point.  When sway translates the camera within the can, the
 *     vanishing point's screen position is INVARIANT — it depends
 *     only on the forward direction, not on camera translation
 *     parallel to the screen plane.  See T9.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • At default settings, walls of the tunnel STREAM toward the
 *     camera at speed 8.  The HUD shows "speed:8.0".  Press '−'
 *     repeatedly until speed = 0.5 — pattern motion slows to a
 *     crawl.
 *
 *   • Pause (space): stops sway + roll + texture flow, all at once.
 *     Confirms that animation is purely time-driven; pause kills all.
 *
 *   • Press n through all 4 patterns.  RINGS shows axial sinusoidal
 *     bands.  CHECKER shows alternating squares.  SPOKES shows
 *     stripes radiating outward (no axial variation).  GRID shows
 *     bright crosshatch lines.
 *
 *   • Press t through all 6 themes.  Geometry stays identical, only
 *     the depth-ramp colour changes.
 *
 *   • Press s several times to add sway: camera slowly orbits within
 *     the cylinder, but the vanishing-point crosshair stays put at
 *     screen centre (T9).
 *
 *   • Press r to reset time → texture flow snaps back to a fresh
 *     phase; camera position resets.
 *
 *   • Resize the terminal → tunnel re-aspects, vanishing point stays
 *     at the new screen centre, HUD reflows.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL — ten short answers ─────────────────────────────── *
 *
 *
 * T1: Why ANALYTICAL intersection (not sphere tracing)?
 * ─────────────────────────────────────────────────────
 * Most files in raymarcher/ use SPHERE TRACING — iterative march
 * along the ray, asking the SDF "how far is the nearest surface?"
 * at each step until convergence.  That's the right tool when the
 * surface has 3-D structure that VARIES along the ray (a fractal,
 * a Boolean of primitives, a metaball field).
 *
 * For an INFINITE CYLINDER, the surface satisfies a single algebraic
 * equation:
 *
 *      x² + y² = R²
 *
 * Substitute the ray r(t) = O + t · D into that equation:
 *
 *      (O.x + t · D.x)² + (O.y + t · D.y)² = R²
 *
 * Expand → quadratic in t → quadratic formula → exact closed-form t.
 * No iteration, no MAX_STEPS, no convergence epsilon.  Four multiplies
 * and one sqrt give the answer to floating-point precision.
 *
 * Trade-off: closed-form intersection only works when the surface
 * has a simple algebraic equation.  For a torus you'd need a quartic
 * (still solvable in closed form, but messy); for a Bezier patch,
 * forget it.  Sphere tracing handles ALL of those uniformly at the
 * cost of iteration.  Pick the right tool for the shape.
 *
 *
 * T2: The +sqrt root — camera inside the can
 * ──────────────────────────────────────────
 * The quadratic a·t² + 2b·t + c = 0 has two roots:
 *
 *      t± = (−b ± sqrt(b² − a·c)) / a
 *
 * Geometrically, these are the TWO points where the ray crosses the
 * cylinder wall (entering and exiting if camera is outside; or the
 * one behind and one ahead if camera is inside).  Our camera is
 * inside, so:
 *
 *      t− is NEGATIVE — the ray would exit "backward" through the
 *           wall behind the camera (we don't render backward rays).
 *      t+ is POSITIVE — the ray exits "forward" through the wall
 *           ahead.  This is the hit we want.
 *
 * Always take t+.  Code does `(−b + sqrt(...)) / a` — the + is
 * non-negotiable.  Use t− and you'd render the cylinder INSIDE OUT.
 *
 *
 * T3: Cylindrical UV mapping
 * ──────────────────────────
 * Once we have hit point P = O + t · D, recover its position in
 * cylindrical (u, v) coordinates:
 *
 *      u_tex   = (atan2(P.y, P.x) + π) / (2π)         azimuth ∈ [0, 1)
 *      v_tex   = P.z                                    axial position
 *
 * The atan2(P.y, P.x) returns the angle from +X axis to the vector
 * (P.x, P.y), in (−π, π].  Adding π shifts to [0, 2π); dividing by
 * 2π normalises to [0, 1).  The wraparound at u = 0/1 is a SEAM —
 * patterns that depend on integer floor(u·N) cleanly tile across
 * it as long as N divides evenly.
 *
 * v_tex is just the axial position.  A pattern pinned at integer
 * v values stays pinned in world space; subtract a flow offset to
 * make it move (T4).
 *
 *
 * T4: "Speed lives in the texture, not the geometry"
 * ──────────────────────────────────────────────────
 * Naïve approach: advance the camera forward each frame.  Update
 * O.z += speed · dt.  Re-render.
 *
 * Smarter approach for an INFINITE cylinder: the camera doesn't
 * NEED to move.  An infinite cylinder is translation-invariant
 * along its axis — the geometry at z = 1000 is identical to the
 * geometry at z = 0.  So: leave the camera at z = 0 forever, and
 * SUBTRACT a flow offset from v_tex when sampling the texture:
 *
 *      v_tex = P.z − speed · time
 *
 * Visually identical to moving the camera forward at the given
 * speed.  Computationally cheaper (no per-frame state update; no
 * concern about z growing without bound); and the camera-basis
 * code stays trivial.  Speed control becomes "tune one parameter
 * inside the SDF", not "tune the camera animation".
 *
 *
 * T5: Camera basis with roll — rotate right and up around z
 * ─────────────────────────────────────────────────────────
 * For a basic camera, the basis vectors are:
 *
 *      forward = (0, 0, 1)
 *      right   = (1, 0, 0)
 *      up      = (0, 1, 0)
 *
 * To make the camera ROLL (rotate around its forward axis), apply
 * a 2-D rotation to right and up:
 *
 *      r = ROLL_RATE · time
 *      right = ( cos r,  sin r, 0)
 *      up    = (−sin r,  cos r, 0)
 *
 * forward stays (0, 0, 1).  This is a pure z-axis rotation of the
 * camera frame.  Geometrically: the cylinder is symmetric under
 * z-rotation, so rolling the camera is equivalent to rotating the
 * texture sample point's u_tex coordinate.  Visual effect: the
 * pattern appears to rotate in place as you fly.  Different
 * patterns (RINGS) ignore u entirely → they're invisible to roll.
 * CHECKER, SPOKES, GRID all show the rotation.
 *
 *
 * T6: Sway — Lissajous translation of the camera
 * ──────────────────────────────────────────────
 *      O.x = sway_amp · sin(time · SWAY_FREQ_X)
 *      O.y = sway_amp · cos(time · SWAY_FREQ_Y) · 0.55
 *
 * Two different frequencies (SWAY_FREQ_X = 0.55, SWAY_FREQ_Y =
 * 0.31) → quasi-periodic Lissajous orbit; the camera doesn't
 * repeat the same path exactly.  Reads as "the tunnel curves
 * gently" even though the geometry is straight.  Amplitude is
 * clamped to <0.85 · R so we don't slam into the wall.
 *
 * Sway is a TRANSLATION of camera origin, not a rotation of
 * direction.  Combined with the analytical intersection, sway just
 * shifts which part of the cylinder wall each ray hits — no extra
 * cost in the renderer.
 *
 *
 * T7: Four patterns — different ways of dividing (u, v)
 * ─────────────────────────────────────────────────────
 *      RINGS      sin(v · ring_freq) · 0.5 + 0.5
 *                 axial sinusoid; ignores u entirely
 *                 (so camera roll is invisible against this pattern)
 *
 *      CHECKER    floor(u · stripes_around) XOR floor(v · stripes_long)
 *                 alternating squares; both u and v matter, so roll
 *                 + flow + sway are all visible
 *
 *      SPOKES     floor(u · stripes_around) parity
 *                 ignores v entirely; pure radial stripes
 *                 (so flow and sway are invisible against this pattern)
 *
 *      GRID       hatch lines at integer u and v boundaries
 *                 max(u_distance, v_distance) < line_thick → bright
 *                 cross-hatch with thin bright lines on a dark base
 *
 * Each pattern exposes a different aspect of the camera motion.
 * Switch between them (n / N keys) to feel which animation is
 * driving the visual at any given instant.
 *
 *
 * T8: Distance fog — exp(−t · density) with floor
 * ───────────────────────────────────────────────
 * Walls farther down the tunnel should be DIMMER (fog hides
 * distance).  Apply an exponential decay:
 *
 *      fog = exp(−t · FOG_DENSITY)
 *
 * t is the ray-parameter at hit (= distance to the wall).  Far
 * hits (t large) → fog ≈ 0; near hits (t small) → fog ≈ 1.
 *
 * Combine with pattern brightness:
 *
 *      intensity = fog · (FOG_FLOOR + brightness · (1 − FOG_FLOOR))
 *
 * The FOG_FLOOR keeps even very-far walls from going PURE BLACK —
 * we want to SEE the tunnel structure even at the vanishing point,
 * not just see void.  At fog = 0, intensity = 0.  At fog = 1 and
 * full brightness, intensity = 1.  In between, the floor adds a
 * minimum visibility that fades out smoothly with fog.
 *
 *
 * T9: Vanishing-point crosshair — invariant to sway
 * ─────────────────────────────────────────────────
 * For our camera with forward = (0, 0, 1), the AXIS DIRECTION's
 * projection on the screen is at exactly (cols/2, rows_eff/2).
 * Why: the projection of a direction vector (not a point) ignores
 * camera translation entirely — only camera ROTATION affects it.
 * Our roll is around the SAME axis the cylinder lives on, so it
 * also doesn't move the projected vanishing point.
 *
 * Net result: as the camera sways within the can, the cylinder
 * walls visibly shift around on the screen, but the vanishing
 * point — the screen position toward which all "down-the-tunnel"
 * rays point — stays nailed at the centre.  The crosshair is a
 * 5-cell `+` and four `o` arms drawn LAST (over the rendered
 * tunnel) as a visual anchor for the eye.
 *
 *
 * T10: Quantisation — intensity → glyph + theme depth
 * ───────────────────────────────────────────────────
 *      slot   = floor(intensity · 7.999)              ∈ {0..7}
 *      glyph  = LUMA_GLYPHS[slot]                     ' ' through '@'
 *      pair   = PAIR_DEPTH_BASE + slot                theme's tier-N colour
 *      attr   = A_BOLD (slot ≥ 6) | A_DIM (slot ≤ 1) | A_NORMAL else
 *
 * Three signals into one cell: GLYPH from the intensity (perceptual
 * brightness via the ink-density ramp), COLOUR PAIR from the same
 * slot (theme-specific depth ramp), ATTRIBUTE for an extra bold/
 * dim push at the extremes of the range.  All three reinforce each
 * other — the brightest cells are bright glyph + bright colour +
 * bold attribute; the dimmest are space + dim colour + DIM
 * attribute.
 *
 * Six themes (WARP / INFERNO / ELECTRIC / VOID / MATRIX / GOLD)
 * give different depth-ramp hues; every theme keeps the same
 * 8-tier slot mapping so glyph + attr stay coherent across theme
 * changes.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE — concern layers & the per-tick combine ────────────── *
 *
 * This demo grew by accretion; the concerns below were interleaved and are
 * now isolated into labelled sections. The persistent state lives in two
 * structs (Scene §12, App §17) — listed under MUTATES.
 *
 *   LAYER        SECTION(S)        MUTATES
 *   ----------   ---------------   -----------------------------------------
 *   SIMULATION   §12 scene_tick    scene.time  (accumulated seconds — the ONE
 *                                  evolving value; sway/roll/flow derive from
 * it) LOGIC        §4 vec3, §5 ray-  nothing — pure functions; same inputs give
 *                cylinder, §6-§11  the same result, so render/effects ordering
 *                patterns/UV/fog,  cannot corrupt them. build_camera (§13) is a
 *                §13 build_camera  pure Scene→Camera mapping.
 *   EFFECTS      — none —          the sway, roll, fog and texture-flow "look"
 *                                  are all derived at render time from
 *                                  scene.time; nothing cosmetic is stored.
 *   RENDER       §3 color, §14     the terminal only (ncurses screen); reads
 *                scene_render,     scene + knob state, never writes it.
 * theme_apply §15, §16 screen   reprograms colour pairs on a 't' press. DELAYS
 * §12 scene_tick    none — `scene.paused` gates the time advance PERFORMANCE §2
 * clock, §17     frame_time, dt, sim_accum, fps_accum, main loop frame_count,
 * fps_display; sleep — timing only
 *
 * PER-TICK COMBINE — one loop iteration in main (§17); ONLY step 3 advances
 * simulation state, and it does so in a fixed-timestep accumulator:
 *   1. (event)  apply pending resize             — app_do_resize  — NOT the
 * tick
 *   2. perf     measure dt, cap to 100 ms
 *   3. SIM      while (accum >= tick): scene_tick(dt_sec)         ← the tick
 *   4. perf     roll fps average
 *   5. perf     sleep to frame-cap (~60 fps)
 *   6. RENDER   screen_draw (scene_render + HUD) → present  ← the one
 * state→screen
 *   7. (event)  drain one key via app_handle_key            — NOT the tick
 *
 * User EVENTS (resize, keys §17) are NOT the tick — they set inputs the next
 * tick and render read. Keys mutate Scene knobs
 * (pattern/theme/speed/sway/pause) and app.sim_fps; RESET (`r`) → scene_reset
 * zeroes scene.time.
 * ─────────────────────────────────────────────────────────────────── */
