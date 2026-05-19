/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * chain.c — Hanging Chain & Swinging Rope
 *
 * A chain of point masses connected by inextensible links, simulated with
 * Position-Based Dynamics (PBD):
 *
 *   1. Verlet predict:  x_pred = x + (x - x_old)*DAMP + g*dt²
 *   2. Project constraints (N_ITER times):
 *        for each link (a, b):
 *          d = x_pred[b] - x_pred[a]
 *          corr = (|d| - rest) / |d|
 *          x_pred[a] += 0.5 * corr * d   (split equally; skip if pinned)
 *          x_pred[b] -= 0.5 * corr * d
 *   3. Derive velocity: v ≈ (x_pred - x_old) / dt  (implicit in Verlet)
 *   4. x_old = x; x = x_pred
 *
 * PBD is unconditionally stable for any stiffness — no spring constant to
 * tune for stability. More iterations = stiffer chain.
 *
 * Presets:
 *   0  Hanging    — pinned at top, gravity + sinusoidal wind
 *   1  Pendulum   — pinned at top, released from 60°; swings freely
 *   2  Bridge     — pinned at both ends, catenary sag; gust from below
 *   3  Wave       — top node driven horizontally; standing waves emerge
 *   4  Festoon    — three pins at top, double-catenary garland with breeze
 *   5  Snake      — bridge geometry + vertical drive on one end; slithers
 *   6  Storm      — hanging chain with 4× violent wind; whips around
 *   7  Slingshot  — pendulum from 150° (chain inverted); dramatic full swing
 *   8  Flag       — top-left pin, horizontal start, sustained strong wind
 *   9  Double     — two pins (anchor + side-hook), lower pendulum at 45°
 *
 * Keys:
 *   q / ESC    quit
 *   space / p  pause / resume
 *   r          reset current preset
 *   n / N      next / previous preset
 *   t / T      next / previous theme  (10 palettes:
 *                Matrix  Fire    Oceanic  Neon    Mono
 *                Ice     Nova    Forest   Desert  Eclipse)
 *   + / -      more / fewer constraint iterations (stiffness)
 *   w          toggle wind / gust
 *   ] / [      simulation FPS up / down
 *
 * HUD: canonical CLAUDE.md two-bar — row 0 right shows live status
 * (preset, theme, iter, wind, fps, paused/running); row rows-1 lists
 * the action keys.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/chain.c -o chain -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 coords  §5 physics
 *           §6 scene   §7 screen §8 app
 */

/* ── CONCEPTS ────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Position-Based Dynamics (PBD / XPBD) [1].
 *                 Instead of integrating forces to get acceleration and then
 *                 velocity, PBD works directly on positions.  Each constraint
 *                 (link length) is projected to satisfaction by moving the
 *                 endpoints.  Repeating N_ITER times → stiffer chain.  The
 *                 practical Verlet + iterative-relaxation pattern used here
 *                 is the one popularised for game ragdolls in [2].
 *
 * Physics        : Verlet integration + constraint projection [1, 2].
 *                 Velocity is implicit: v ≈ (x_current − x_prev) / dt.
 *                 There is no explicit velocity variable — damping multiplies
 *                 the positional delta before the predict step.
 *
 *                 Visible physics across the 10 presets is standard
 *                 classical mechanics [3]: catenary (Bridge, Festoon),
 *                 simple/large-angle pendulum (Pendulum, Slingshot,
 *                 Double), forced oscillation, beat patterns.  Wave-on-a-
 *                 string behaviour (Wave, Snake) — reflections off fixed
 *                 ends, standing modes at resonance — follows the
 *                 transverse-wave equation derived in [4].
 *
 * Math           : Constraint projection formula (per link), from [1]:
 *                 correction = (|d| − rest) / |d|  ·  d_vector
 *                 Each free endpoint absorbs half the correction (equal
 *                 mass assumption).  A pinned node absorbs none (infinite
 *                 mass).
 *
 * Performance    : Sub-stepping (SUB_STEPS=8) divides dt into 8 smaller
 *                 steps.  Each sub-step runs the full constraint loop.
 *                 This improves stability for stiff constraints without
 *                 shrinking the render frame rate or using an implicit
 *                 (matrix-solving) integrator.  Cost: O(N · I · S) per
 *                 frame where N=nodes, I=iterations, S=sub-steps.
 *
 * Stability      : PBD is unconditionally stable [1] — no spring constant to
 *                 blow up.  Stiffness is purely iteration count, not a
 *                 numerical parameter.  More iterations → stiffer rope,
 *                 but each iteration is just a few multiplies.
 *
 * Data-structure : Ring-buffer trail (TRAIL_LEN entries) for the last
 *                 free node.  Oldest entry is trail_head wrapped by modulo
 *                 arithmetic — no shifts needed.
 *
 * Rendering      : 10 brightness-safe theme palettes [5] — link-tier ramp
 *                 (link_lo / link_mid / link_hi) is a perceptually-ordered
 *                 gradient mapped to strain (relaxed / stretched / taut),
 *                 so the rope visibly "heats up" wherever it is being
 *                 pulled.  Canonical two-bar HUD: row 0 right shows live
 *                 status; row rows-1 lists the action keys.
 *
 * References (cite inline as [n]):
 *
 *   [1] Müller, M.; Heidelberger, B.; Hennix, M.; Ratcliff, J. (2007) —
 *       "Position Based Dynamics", *J. Visual Communication and Image
 *       Representation* 18 (2), 109–118.  The seminal PBD paper:
 *       constraint-projection iteration, unconditional stability, and
 *       the half-half mass-split formula used in chain_pbd_step (§5).
 *
 *   [2] Jakobsen, T. (2001) — "Advanced Character Physics", GDC.
 *       The practical Verlet + iterative-relaxation paper from
 *       Hitman: Codename 47.  Concrete recipe almost identical to ours:
 *       Verlet predict, then N_ITER passes of pairwise distance
 *       constraints, pinned nodes via infinite mass.  Highly readable
 *       game-dev primer.
 *
 *   [3] Goldstein, H.; Poole, C.; Safko, J. — *Classical Mechanics*,
 *       3rd ed., Addison-Wesley (2002).  Pendulum dynamics (small- and
 *       large-angle), catenary curve (variational form), constrained
 *       systems, coupled oscillators.  Backs the qualitative behaviour
 *       of every preset in §6.
 *
 *   [4] Crawford, F. S. — *Waves*, Berkeley Physics Course vol. 3,
 *       McGraw-Hill (1968).  Transverse waves on a string: wave
 *       equation ∂²y/∂t² = c² ∂²y/∂x², reflection at fixed boundaries,
 *       standing-wave modes.  Backs the Wave and Snake presets and
 *       the resonance pattern formation in chain_pbd_step.
 *
 *   [5] Ware, C. — *Information Visualization: Perception for Design*,
 *       4th ed., Morgan Kaufmann (2020).  Perceptually-ordered colour
 *       and luminance ramps (Ch. 4) back the link-strain ramp and the
 *       10 brightness-safe theme palettes in §3 (k_themes).
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
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
  SIM_FPS_DEFAULT = 60, /* target render/sim rate (frames per second)  */
  SIM_FPS_MIN = 10,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,

  N_NODES_MAX = 32, /* max chain nodes (including anchor nodes)    */
  N_NODES_DEF = 24, /* default node count; more → smoother but     */
                    /* O(N) constraint passes per sub-step         */
  N_ITER_DEF = 20,  /* constraint iterations per sub-step:         */
                    /* more → stiffer rope, no instability risk    */
  N_ITER_MIN = 5,
  N_ITER_MAX = 60,
  SUB_STEPS = 8, /* physics sub-steps per rendered frame:       */
                 /* effective dt = frame_dt / SUB_STEPS         */
                 /* smaller sub-dt → more accurate constraint   */
                 /* projection without changing display fps     */

  TRAIL_LEN = 90, /* ring-buffer trail depth for bottom node;    */
                  /* at 60 fps ≈ 1.5 s of history               */
  N_PRESETS = 10, /* 0..3 original + 4..9 complex (see §6)        */
  N_THEMES = 10,  /* MATRIX..ECLIPSE — see k_themes in §3 */
};

#define NS_PER_SEC 1000000000LL       /* nanoseconds in one second      */
#define TICK_NS(f) (NS_PER_SEC / (f)) /* nanoseconds per frame at fps f */

/* Terminal cell dimensions in pixels (ncurses coordinate space).
 * CELL_W × CELL_H is the logical pixel size of one character cell.
 * CELL_H / CELL_W ≈ 2 because typical monospace fonts are taller than wide.
 * All physics runs in pixel-space; px_to_cx / px_to_cy convert to cells.*/
#define CELL_W 8
#define CELL_H 16

/* GRAVITY: downward acceleration in terminal pixel-space (px/s²).
 * Real gravity = 9.81 m/s².  If we define 1 m ≡ CELL_H px (≈ one cell
 * height), then a screen of 40 cells ≈ 40 m — too slow visually.
 * Chosen empirically so the chain sways at a pleasing rate on a typical
 * 80-row terminal.  Scale: ~38.7 px per "metre", giving 9.81×38.7 ≈ 380. */
#define GRAVITY 380.f

/* DAMP: fraction of velocity kept after each sub-step (dimensionless 0–1).
 * Applied as: velocity *= DAMP each sub-step (via positional delta).
 * At SUB_STEPS=8 and 60 fps: 0.997^(8×60) ≈ 0.30 per second — gentle
 * energy drain that prevents perpetual oscillation without overdamping. */
#define DAMP 0.997f

/* WIND_FREQ: frequency of the sinusoidal wind force (Hz).
 * 0.35 Hz ≈ one full left-right cycle every 2.9 s — slow enough to look
 * like natural wind gusts rather than rapid mechanical oscillation.       */
#define WIND_FREQ 0.35f

/* Driven-anchor parameters — shared by the Wave (preset 3, horizontal)
 * and Snake (preset 5, vertical, 0.5× amplitude to fight gravity) drivers. */

/* WAVE_FREQ: driver frequency for the moving anchor (Hz).
 * 1.8 Hz was tuned to fall near the second normal mode of a 24-node chain
 * at the default tension, producing visible standing-wave patterns.       */
#define WAVE_FREQ 1.8f

/* WAVE_AMPL: peak displacement of the driven anchor (pixels).
 * At CELL_W=8 this is ~2.75 character columns — large enough to see the
 * wave clearly without the chain folding over itself.                     */
#define WAVE_AMPL 22.f

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

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

/* ===================================================================== */
/* §3  color / theme                                                      */
/* ===================================================================== */

/*
 * Color pair assignments:
 *   1   link — relaxed  (near rest length)        — per theme
 *   2   link — stretched                          — per theme
 *   3   link — very stretched / taut              — per theme
 *   4   node — free                               — per theme
 *   5   node — pinned / driven anchor             — per theme
 *   6   trail (last free node, fading)            — per theme
 *   7   HUD — top status, bright yellow + bold    — theme-independent chrome
 *   8   HINT — bottom action bar, cyan + bold     — theme-independent chrome
 */
enum {
  CP_LINK_LO = 1,
  CP_LINK_MID = 2,
  CP_LINK_HI = 3,
  CP_NODE = 4,
  CP_ANCHOR = 5,
  CP_TRAIL = 6,
  CP_HUD = 7,
  CP_HINT = 8,
};

/*
 * Theme — six 256-colour cube indices, one per data role on the chain.
 * The link-tier ramp (link_lo → link_mid → link_hi) is a perceptual
 * gradient mapped to constraint strain (relaxed → taut), so the rope
 * visibly "heats up" where it's stretched.  CP_HUD / CP_HINT are NOT
 * in the theme — they stay fixed across all palettes so the HUD chrome
 * is always legible.
 *
 * Brightness safety (CLAUDE.md): every entry sits at index ≥ 30, or
 * 24-29 / 240-243 only as the lowest ramp tier.  16-23 / 232-239 are
 * forbidden — they vanish on default-background terminals.
 */
typedef struct {
  const char *name;
  short link_lo, link_mid, link_hi;
  short node, anchor, trail;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*  name        link_lo link_mid link_hi  node  anchor  trail */
    {"Matrix", 28, 40, 46, 231, 226, 82},      /* cyber green */
    {"Fire", 130, 208, 196, 231, 226, 220},    /* warm → red  */
    {"Oceanic", 24, 39, 51, 195, 231, 87},     /* teal → cyan */
    {"Neon", 129, 201, 213, 231, 226, 51},     /* purple/pink */
    {"Mono", 240, 247, 255, 255, 255, 244},    /* grayscale   */
    {"Ice", 153, 159, 195, 231, 255, 117},     /* light blues */
    {"Nova", 129, 141, 213, 231, 226, 177},    /* stellar     */
    {"Forest", 58, 100, 190, 178, 226, 130},   /* leaves/bark */
    {"Desert", 130, 178, 220, 230, 226, 178},  /* sand/gold   */
    {"Eclipse", 240, 244, 196, 255, 226, 124}, /* gray + red  */
};

/* Active theme index is owned by Scene (g_scene.theme, defined in §6).
 * theme_apply takes the index as a parameter so it doesn't need to
 * forward-reference the Scene struct. */

static void theme_apply(int t) {
  const Theme *th = &k_themes[t % N_THEMES];
  if (COLORS >= 256) {
    init_pair(CP_LINK_LO, th->link_lo, -1);
    init_pair(CP_LINK_MID, th->link_mid, -1);
    init_pair(CP_LINK_HI, th->link_hi, -1);
    init_pair(CP_NODE, th->node, -1);
    init_pair(CP_ANCHOR, th->anchor, -1);
    init_pair(CP_TRAIL, th->trail, -1);
    /* Fixed chrome — same on every theme so the HUD stays legible. */
    init_pair(CP_HUD, 226, -1); /* bright yellow */
    init_pair(CP_HINT, 51, -1); /* bright cyan   */
  } else {
    /* 8-colour fallback — theme-independent. */
    init_pair(CP_LINK_LO, COLOR_CYAN, -1);
    init_pair(CP_LINK_MID, COLOR_YELLOW, -1);
    init_pair(CP_LINK_HI, COLOR_RED, -1);
    init_pair(CP_NODE, COLOR_WHITE, -1);
    init_pair(CP_ANCHOR, COLOR_YELLOW, -1);
    init_pair(CP_TRAIL, COLOR_WHITE, -1);
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  theme_apply(0);
}

/* ===================================================================== */
/* §4  coords                                                             */
/* ===================================================================== */

/* Convert pixel-space coordinates to terminal cell coordinates.
 * + 0.5f performs nearest-cell rounding (equivalent to floorf(x + 0.5)). */
static inline int px_to_cx(float px) { return (int)(px / CELL_W + 0.5f); }
static inline int px_to_cy(float py) { return (int)(py / CELL_H + 0.5f); }

/* ===================================================================== */
/* §5  physics — Position-Based Dynamics (PBD) chain                     */
/*                                                                       */
/* Data structures (ChainNode, Chain) plus the named-step PBD pipeline.  */
/* The integration loop is laid out across compute_wind_force →          */
/* verlet_predict_node → project_link_constraint → relax_link_constraints*/
/* → chain_pbd_step, each with its own docstring.  The driver chain_tick */
/* sequences sub-steps and applies preset-specific anchor drivers.       */
/* ===================================================================== */

/*
 * ChainNode — one mass point in the PBD chain.
 *
 * Why two positions (x, ox) instead of a (position, velocity) pair:
 *   The integrator is Verlet [2] — velocity is IMPLICIT in the
 *   difference between two consecutive positions:
 *
 *       v ≈ (x − ox) / dt
 *
 *   Storing (x, ox) instead of (x, v) gives us:
 *     • a one-line predict step:  x_new = x + (x − ox) + a·dt²
 *     • damping is just a multiplier on (x − ox) before reuse, so we
 *       never need to compute or store v explicitly
 *     • constraint projection moves x directly without converting
 *       back into velocity space — the next tick's "velocity" is
 *       exactly the projected position delta, automatically.
 *
 *   Cost: one extra (ox, oy) per node, ~16 bytes — negligible.  Payoff:
 *   stable integrator with zero coupling between numerical timestep
 *   and constraint stiffness (PBD's headline property [1]).
 *
 * Why floats (sub-pixel precision):
 *   Cells are 8×16 pixels (CELL_W × CELL_H).  Quantising to integer
 *   cells would jitter the chain like a low-resolution Bresenham line
 *   every frame.  The renderer quantises only at draw time via
 *   px_to_cx / px_to_cy (§4).
 *
 * The `pinned` flag:
 *   pinned = true → position is set externally each tick (a constant
 *   anchor for static pins, or a sinusoidal driver for the Wave and
 *   Snake presets).  The PBD constraint loop SKIPS pinned nodes —
 *   they model an infinite-mass endpoint.  When only one end of a
 *   link is pinned, the other absorbs the FULL correction (not half).
 *
 * Algorithm refs (header REFERENCES):
 *   Verlet two-position storage      — Jakobsen [2]
 *   Pinned / infinite-mass handling  — Müller PBD [1]
 */
typedef struct {
  float x, y;   /* current position (pixels)                       */
  float ox, oy; /* previous-step position — Verlet (v = (x-ox)/dt) */
  bool pinned;  /* true → position set externally; PBD skips it    */
} ChainNode;

/*
 * Chain — the full PBD simulation state for one chain instance.
 *
 * Why one struct holding everything (vs. separate arrays):
 *   Chain bundles the physics (nodes, link_rest), the active control
 *   knobs (pause, iter count, wind), the preset identity, and the
 *   renderer's trail ring buffer.  Everything scene_init() resets
 *   together belongs together; everything chain_tick() reads/writes
 *   is one pointer dereference away.
 *
 * Field groups (mirrored by the layout below):
 *
 *   Geometry         nodes[], n_nodes, link_rest — the bonded
 *                    mass-point graph the PBD solver iterates over.
 *
 *   Wind forcing     wind_phase, wind_str, wind_on — sinusoidal
 *                    horizontal driver added in the Verlet predict
 *                    step.  Strength varies per preset (140 default,
 *                    580 in Storm, 320 in Flag, 110 in Bridge, etc.).
 *
 *   Control state    paused, n_iter — interactive knobs.  n_iter
 *                    controls effective stiffness: more constraint
 *                    projection passes per sub-step → stiffer rope.
 *                    No instability risk; PBD is unconditionally stable.
 *
 *   Preset identity  preset (∈ [0, N_PRESETS)) — chain_tick reads it
 *                    to enable per-preset drivers (preset 3 = Wave's
 *                    horizontal drive on node 0; preset 5 = Snake's
 *                    vertical drive on node 0).
 *
 *   Driver state     wave_phase, wave_ax, wave_ay — anchor position
 *                    and phase for the driven-endpoint presets (3, 5).
 *                    Unused for static-pin presets but always present.
 *
 *   Trail (renderer) trail_px / trail_py ring buffer for the bottom
 *                    free node.  Drawn newest-to-oldest with fading
 *                    glyphs in chain_draw — visualises the recent path.
 *
 * Why a fixed N_NODES_MAX array (not heap-allocated):
 *   Chain length never exceeds N_NODES_MAX, and an embedded array keeps
 *   the whole Chain in one contiguous block — better cache behaviour
 *   for the per-frame O(N · I · S) constraint loop and zero allocation
 *   in the hot path.
 *
 * Algorithm refs (header REFERENCES):
 *   PBD constraint-projection loop  — Müller [1]
 *   Verlet + iterative relaxation   — Jakobsen [2]
 *   Catenary / pendulum behaviour   — Goldstein [3]
 *   Wave reflection / standing modes — Crawford [4]
 */
typedef struct {
  /* ── Geometry (PBD operates on these) ─────────────────────────── */
  ChainNode nodes[N_NODES_MAX];
  int n_nodes;
  float link_rest; /* rest length of each link, pixels             */

  /* ── Wind forcing (added in the predict step) ─────────────────── */
  float wind_phase; /* radians; advanced at WIND_FREQ Hz            */
  float wind_str;   /* peak amplitude, pixels/s² (set per preset)   */
  bool wind_on;     /* w / W key toggles                            */

  /* ── Interactive control state ────────────────────────────────── */
  bool paused; /* p / space — chain_tick is a no-op when true  */
  int n_iter;  /* constraint iterations per sub-step (+/- keys)
                * acts as stiffness, clamped [N_ITER_MIN, MAX] */

  /* ── Preset identity (drives per-preset code paths in tick) ───── */
  int preset; /* ∈ [0, N_PRESETS); selects driver branch     */

  /* ── Wave/Snake driver state (presets 3 & 5 only) ─────────────── */
  float wave_phase; /* radians; advanced at WAVE_FREQ Hz            */
  float wave_ax;    /* anchor x for the driven endpoint             */
  float wave_ay;    /* anchor y for the driven endpoint             */

  /* ── Trail ring buffer (read by chain_draw in §5) ─────────────── */
  float trail_px[TRAIL_LEN];
  float trail_py[TRAIL_LEN];
  int trail_head; /* next write index ∈ [0, TRAIL_LEN)            */
  int trail_cnt;  /* valid sample count, saturates at TRAIL_LEN   */
} Chain;

/* ── Draw helper: DDA segment ─────────────────────────────────────── */

/*
 * slope_glyph — pick the ASCII glyph that visually approximates a
 * segment's slope, quantised into four orientation bands:
 *
 *   |dx| ≥ 2 · |dy|   →  '-'    near-horizontal
 *   |dy| ≥ 2 · |dx|   →  '|'    near-vertical
 *   dx·dy > 0         →  '\\'   down-right / up-left diagonal
 *   otherwise         →  '/'    down-left / up-right diagonal
 *
 * The same glyph fills every cell of the segment so the eye reads it
 * as one continuous stroke rather than a stippled line.
 */
static char slope_glyph(int dx, int dy) {
  int adx = abs(dx), ady = abs(dy);
  if (adx >= 2 * ady)
    return '-';
  else if (ady >= 2 * adx)
    return '|';
  else if (dx * dy > 0)
    return '\\';
  else
    return '/';
}

/*
 * seg_draw — DDA line from (x0, y0) to (x1, y1) filling every cell
 * with one slope-picked glyph.  Cells outside [0, cols) horizontally
 * and [1, rows-1) vertically are clipped (the row 0 / rows-1 HUD
 * bands are protected).  Degenerate zero-length segments paint a
 * single 'o' at the start.
 *
 * Pseudocode:
 *   dx, dy ← (x1, y1) − (x0, y0)
 *   steps  ← max(|dx|, |dy|)
 *   if steps == 0: paint 'o' at (x0, y0); return
 *   ch ← slope_glyph(dx, dy)
 *   for k in [0, steps]:
 *       (cx, cy) ← (x0, y0) + (k/steps) · (dx, dy)
 *       if in-chamber: mvaddch(cy, cx, ch)
 */
static void seg_draw(int x0, int y0, int x1, int y1, int cols, int rows,
                     chtype attr) {
  int dx = x1 - x0;
  int dy = y1 - y0;
  int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);

  if (steps == 0) {
    if (x0 >= 0 && x0 < cols && y0 >= 1 && y0 < rows - 1) {
      attron(attr);
      mvaddch(y0, x0, 'o');
      attroff(attr);
    }
    return;
  }

  char ch = slope_glyph(dx, dy);

  attron(attr);
  for (int k = 0; k <= steps; k++) {
    int cx = x0 + k * dx / steps;
    int cy = y0 + k * dy / steps;
    if (cx >= 0 && cx < cols && cy >= 1 && cy < rows - 1)
      mvaddch(cy, cx, ch);
  }
  attroff(attr);
}

/* ── PBD sub-step ─────────────────────────────────────────────────── */

/*
 * compute_wind_force — sinusoidal horizontal forcing at the current
 * wind_phase.  Returns 0 when wind is toggled off.
 *
 *     F_wind = wind_str · sin(wind_phase)
 *
 * Added to each free node's predict step as an external acceleration.
 */
static float compute_wind_force(const Chain *c) {
  return c->wind_on ? c->wind_str * sinf(c->wind_phase) : 0.f;
}

/*
 * verlet_predict_node — advance one mass point by one Verlet step.
 *
 *     v_x   = (x − ox) · DAMP           // implicit damped velocity x
 *     v_y   = (y − oy) · DAMP           // implicit damped velocity y
 *     x_new = x + v_x + wind · dt²      // wind = horizontal force
 *     y_new = y + v_y + GRAVITY · dt²   // gravity is +y (down)
 *     (ox, oy) ← (x, y)                 // save current as "old"
 *     (x, y)   ← (x_new, y_new)
 *
 * Pinned nodes are no-ops (their position is set externally each tick).
 *
 * Algorithm ref: Verlet integration with damping — Jakobsen [2].
 */
static void verlet_predict_node(ChainNode *n, float dt2, float wind_force) {
  if (n->pinned)
    return;
  float vx = (n->x - n->ox) * DAMP;
  float vy = (n->y - n->oy) * DAMP;
  float new_x = n->x + vx + wind_force * dt2;
  float new_y = n->y + vy + GRAVITY * dt2;
  n->ox = n->x;
  n->oy = n->y;
  n->x = new_x;
  n->y = new_y;
}

/*
 * project_link_constraint — one Gauss-Seidel relaxation pass on a
 * single link (a, b) with target rest length.
 *
 *     d    = b.pos − a.pos
 *     dist = |d|                                (current length)
 *     inv  = (dist − rest) / dist               (normalised violation)
 *     corr = inv · d                            (full correction vector)
 *
 * Mass distribution (equal mass = 1 per free node):
 *   • both pinned    → no correction (whatever the pins enforce)
 *   • only a pinned  → b absorbs the FULL correction (infinite mass a)
 *   • only b pinned  → a absorbs the FULL correction
 *   • neither pinned → both absorb HALF (split equally)
 *
 * Degenerate zero-length links are skipped to avoid the 1/0 in `inv`.
 *
 * Algorithm ref: PBD constraint projection — Müller [1].
 */
static void project_link_constraint(ChainNode *a, ChainNode *b, float rest) {
  float dx = b->x - a->x;
  float dy = b->y - a->y;
  float dist = sqrtf(dx * dx + dy * dy);
  if (dist < 1e-6f)
    return;

  float inv = (dist - rest) / dist;
  float cx = inv * dx;
  float cy = inv * dy;

  if (a->pinned && b->pinned) {
    return; /* both fixed — no correction */
  } else if (a->pinned) {
    b->x -= cx;
    b->y -= cy; /* b absorbs full correction */
  } else if (b->pinned) {
    a->x += cx;
    a->y += cy; /* a absorbs full correction */
  } else {
    a->x += 0.5f * cx;
    a->y += 0.5f * cy; /* split 50/50 */
    b->x -= 0.5f * cx;
    b->y -= 0.5f * cy;
  }
}

/*
 * relax_link_constraints — N_ITER Gauss-Seidel passes across all links.
 *
 * Each outer iteration walks the chain end-to-end, calling
 * project_link_constraint on every consecutive pair.  More passes →
 * stiffer rope (the chain converges closer to its rest-length
 * configuration each iteration).  Unconditionally stable — no spring
 * constant to blow up, no implicit matrix solve.
 *
 * Algorithm refs: iterative constraint relaxation — Müller [1], Jakobsen [2].
 */
static void relax_link_constraints(Chain *c) {
  for (int iter = 0; iter < c->n_iter; iter++) {
    for (int i = 0; i < c->n_nodes - 1; i++)
      project_link_constraint(&c->nodes[i], &c->nodes[i + 1], c->link_rest);
  }
}

/*
 * chain_pbd_step — one PBD sub-step on the whole chain.
 *
 * Pseudocode:
 *   wind ← compute_wind_force(c)
 *   for each node:  verlet_predict_node(n, dt², wind)
 *   relax_link_constraints(c)                   // N_ITER projection passes
 */
static void chain_pbd_step(Chain *c, float dt) {
  float dt2 = dt * dt;
  float wind = compute_wind_force(c);

  for (int i = 0; i < c->n_nodes; i++)
    verlet_predict_node(&c->nodes[i], dt2, wind);

  relax_link_constraints(c);
}

/* ── Chain tick ───────────────────────────────────────────────────── */

/*
 * advance_phase_wrapped — increment a sinusoidal driver's phase by
 * one tick's worth of angular motion and wrap it back into [0, 2π).
 *
 *   phase ← phase + freq · 2π · dt
 *   if phase ≥ 2π: phase ← phase − 2π
 *
 * Wrapping keeps the phase from growing without bound over a long
 * run (which would erode float precision in sin/cos).
 */
static void advance_phase_wrapped(float *phase, float freq, float dt) {
  *phase += freq * 2.f * (float)M_PI * dt;
  if (*phase > 2.f * (float)M_PI)
    *phase -= 2.f * (float)M_PI;
}

/*
 * drive_wave_anchor_horizontal — Wave preset's driver: rewrite node 0's
 * (x, ox) pair so the chain sees a sinusoidal HORIZONTAL motion at the
 * top.  The wave then propagates downward via PBD constraint projection.
 *
 *   node[0].x  = wave_ax + WAVE_AMPL · sin(phase_now)
 *   node[0].ox = wave_ax + WAVE_AMPL · sin(phase_prev)
 *
 * Setting BOTH x and ox preserves the implicit Verlet velocity
 * v = (x − ox)/dt for the next predict step — the driven node arrives
 * at the constraint loop already moving.
 */
static void drive_wave_anchor_horizontal(Chain *c, int sub_step, float dt,
                                         float sub_dt) {
  float sub_t = (float)sub_step / (float)SUB_STEPS;
  float phase_now =
      c->wave_phase - WAVE_FREQ * 2.f * (float)M_PI * dt * (1.f - sub_t);
  float phase_prev = phase_now - WAVE_FREQ * 2.f * (float)M_PI * sub_dt;
  c->nodes[0].x = c->wave_ax + WAVE_AMPL * sinf(phase_now);
  c->nodes[0].ox = c->wave_ax + WAVE_AMPL * sinf(phase_prev);
  c->nodes[0].y = c->wave_ay;
  c->nodes[0].oy = c->wave_ay;
}

/*
 * drive_wave_anchor_vertical — Snake preset's driver: same closed-form
 * sinusoid as the horizontal driver but applied to Y instead of X.
 * Launches a TRANSVERSE wave down a horizontal chain; the standing-
 * wave pattern emerges from reflection at the far fixed end (Crawford [4]).
 *
 * Amplitude is reduced to 0.5 × WAVE_AMPL because vertical motion
 * fights against gravity — full amplitude would simply yank node 0 out
 * of the screen.
 */
static void drive_wave_anchor_vertical(Chain *c, int sub_step, float dt,
                                       float sub_dt) {
  float sub_t = (float)sub_step / (float)SUB_STEPS;
  float phase_now =
      c->wave_phase - WAVE_FREQ * 2.f * (float)M_PI * dt * (1.f - sub_t);
  float phase_prev = phase_now - WAVE_FREQ * 2.f * (float)M_PI * sub_dt;
  c->nodes[0].x = c->wave_ax;
  c->nodes[0].ox = c->wave_ax;
  c->nodes[0].y = c->wave_ay + WAVE_AMPL * 0.5f * sinf(phase_now);
  c->nodes[0].oy = c->wave_ay + WAVE_AMPL * 0.5f * sinf(phase_prev);
}

/*
 * apply_preset_driver — dispatch to the active preset's driven-anchor
 * routine.  Only presets 3 (Wave) and 5 (Snake) have one; all other
 * presets leave node positions to PBD's constraint loop.
 */
static void apply_preset_driver(Chain *c, int sub_step, float dt,
                                float sub_dt) {
  if (c->preset == 3)
    drive_wave_anchor_horizontal(c, sub_step, dt, sub_dt);
  else if (c->preset == 5)
    drive_wave_anchor_vertical(c, sub_step, dt, sub_dt);
}

/*
 * record_bottom_trail_sample — push the LAST FREE node's position into
 * the ring-buffer trail.  Skipped when the last node is pinned (e.g.
 * Bridge / Festoon where both ends are anchored — no motion to trail).
 */
static void record_bottom_trail_sample(Chain *c) {
  int last = c->n_nodes - 1;
  if (c->nodes[last].pinned)
    return;
  c->trail_px[c->trail_head] = c->nodes[last].x;
  c->trail_py[c->trail_head] = c->nodes[last].y;
  c->trail_head = (c->trail_head + 1) % TRAIL_LEN;
  if (c->trail_cnt < TRAIL_LEN)
    c->trail_cnt++;
}

/*
 * chain_tick — one frame of physics.
 *
 * Pseudocode:
 *   if paused: return
 *   sub_dt ← dt / SUB_STEPS
 *   advance_phase_wrapped(&wind_phase, WIND_FREQ, dt)
 *   advance_phase_wrapped(&wave_phase, WAVE_FREQ, dt)
 *   for s in [0, SUB_STEPS):
 *       apply_preset_driver(c, s, dt, sub_dt)
 *       chain_pbd_step(c, sub_dt)
 *   record_bottom_trail_sample(c)
 */
static void chain_tick(Chain *c, float dt) {
  if (c->paused)
    return;

  float sub_dt = dt / (float)SUB_STEPS;

  advance_phase_wrapped(&c->wind_phase, WIND_FREQ, dt);
  advance_phase_wrapped(&c->wave_phase, WAVE_FREQ, dt);

  for (int s = 0; s < SUB_STEPS; s++) {
    apply_preset_driver(c, s, dt, sub_dt);
    chain_pbd_step(c, sub_dt);
  }

  record_bottom_trail_sample(c);
}

/* ── Chain draw ───────────────────────────────────────────────────── */

/*
 * link_strain — dimensionless strain ε of one link vs its rest length.
 *
 *     ε = |dist − rest| / rest          0 ≈ relaxed, 1 ≈ 100% stretched
 *
 * +0.001f in the denominator is defensive against any zero-rest links.
 */
static float link_strain(const ChainNode *a, const ChainNode *b, float rest) {
  float dx = b->x - a->x;
  float dy = b->y - a->y;
  float dist = sqrtf(dx * dx + dy * dy);
  return fabsf(dist - rest) / (rest + 0.001f);
}

/*
 * strain_color_pair — three-tier strain → CP_LINK_* selector.
 *
 *     ε < 0.04          →  CP_LINK_LO    relaxed (cool theme tier)
 *     0.04 ≤ ε < 0.12   →  CP_LINK_MID   stretched
 *     ε ≥ 0.12          →  CP_LINK_HI    taut / overstretched (warning)
 *
 * Thresholds tuned so a real rope's visible colour-change-under-load
 * shows at roughly the same strain values.  The ramp is theme-mapped
 * in §3 so it reads as a perceptual "rope heats up where stretched".
 */
static int strain_color_pair(float strain) {
  if (strain < 0.04f)
    return CP_LINK_LO;
  if (strain < 0.12f)
    return CP_LINK_MID;
  return CP_LINK_HI;
}

/*
 * paint_trail — render the ring-buffer trail oldest-to-newest with a
 * fading glyph age ramp:
 *
 *     newest 40% (age > 0.6)  →  '*' bold      (bright fresh path)
 *     older 60%               →  '.' normal    (faded history)
 *
 * No per-entry alpha is stored — read order alone encodes the fade.
 */
static void paint_trail(const Chain *c, int cols, int rows) {
  int start = (c->trail_head - c->trail_cnt + TRAIL_LEN) % TRAIL_LEN;
  for (int i = 0; i < c->trail_cnt; i++) {
    int idx = (start + i) % TRAIL_LEN;
    int cx = px_to_cx(c->trail_px[idx]);
    int cy = px_to_cy(c->trail_py[idx]);
    if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1)
      continue;

    float age = (float)i / (float)(c->trail_cnt > 1 ? c->trail_cnt : 1);
    bool fresh = (age > 0.6f);
    attron(COLOR_PAIR(CP_TRAIL) | (fresh ? A_BOLD : 0));
    mvaddch(cy, cx, fresh ? '*' : '.');
    attroff(COLOR_PAIR(CP_TRAIL) | (fresh ? A_BOLD : 0));
  }
}

/*
 * paint_links — render every link as a strain-coloured DDA segment.
 *
 * Pseudocode:
 *   for each link (a, b):
 *       ε  ← link_strain(a, b, rest)
 *       cp ← strain_color_pair(ε)
 *       seg_draw(a → b, COLOR_PAIR(cp) | A_BOLD)
 */
static void paint_links(const Chain *c, int cols, int rows) {
  for (int i = 0; i < c->n_nodes - 1; i++) {
    const ChainNode *a = &c->nodes[i];
    const ChainNode *b = &c->nodes[i + 1];

    int ax = px_to_cx(a->x), ay = px_to_cy(a->y);
    int bx = px_to_cx(b->x), by = px_to_cy(b->y);

    int cp = strain_color_pair(link_strain(a, b, c->link_rest));
    seg_draw(ax, ay, bx, by, cols, rows, COLOR_PAIR(cp) | A_BOLD);
  }
}

/*
 * paint_nodes — render every node as a role-coded glyph:
 *
 *     pinned + endpoint (i = 0 or N-1)   →  '#'  bold   (wall anchor)
 *     pinned + interior                  →  '*'  bold   (hook / brace)
 *     free                               →  'o'         (mass point)
 *
 * Wall anchors get '#' to read as fixed mounts; interior pins (Festoon
 * middle, Double hook) get '*' so they're distinguishable from end
 * walls.  Free nodes are plain 'o' to keep them visually receding.
 */
static void paint_nodes(const Chain *c, int cols, int rows) {
  int N = c->n_nodes;
  for (int i = 0; i < N; i++) {
    const ChainNode *n = &c->nodes[i];
    int cx = px_to_cx(n->x), cy = px_to_cy(n->y);
    if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1)
      continue;

    if (n->pinned) {
      attron(COLOR_PAIR(CP_ANCHOR) | A_BOLD);
      mvaddch(cy, cx, (i == 0 || i == N - 1) ? '#' : '*');
      attroff(COLOR_PAIR(CP_ANCHOR) | A_BOLD);
    } else {
      attron(COLOR_PAIR(CP_NODE));
      mvaddch(cy, cx, 'o');
      attroff(COLOR_PAIR(CP_NODE));
    }
  }
}

/*
 * chain_draw — render the chain in three layered passes.
 *
 * Pseudocode:
 *   paint_trail(c)   // bottom layer: motion-history dots
 *   paint_links(c)   // middle layer: strain-coloured DDA segments
 *   paint_nodes(c)   // top layer: anchors and free nodes
 */
static void chain_draw(const Chain *c, int cols, int rows) {
  paint_trail(c, cols, rows);
  paint_links(c, cols, rows);
  paint_nodes(c, cols, rows);
}

/* ===================================================================== */
/* §6  scene — preset initialisation & lifecycle                         */
/*                                                                       */
/* Each preset places the chain nodes at specific initial positions and  */
/* pins selected nodes.  The physics §5 then evolves the system forward. */
/*                                                                       */
/* Preset geometry is expressed as fractions of screen dimensions so the */
/* simulation looks proportional regardless of terminal size.            */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Scene — top-level state spanning physics ticks, draw calls, and the
 * main loop.  Groups the file-scope simulation / render globals under
 * one named container.
 *
 * The struct splits into two clearly-labelled groups:
 *
 *   Simulation parameters  — consumed by chain_tick and the preset_*()
 *     init functions.  Anything that affects the PHYSICS lives here.
 *     Mutated by physics-affecting keys: n / N (preset cycle), r
 *     (reset), p / space (pause), +/- (stiffness), w (wind toggle),
 *     ] / [ (sim fps).
 *
 *   Rendering parameters   — consumed by theme_apply and draw_hud_top.
 *     Toggling these while paused must leave the chain positions and
 *     velocities byte-identical; only colours may differ.  Mutated by
 *     purely cosmetic keys: t / T (theme).
 *
 * Locality rationale (this contract matters, not the bytes):
 *   The split exists for the READER, not the CPU.  A new flag landing
 *   in the rendering group when it actually changes how chain_tick
 *   advances state would silently couple display to physics — exactly
 *   the bug the separation prevents.  When adding a field, ask: does
 *   this change what chain_tick or any preset_*() produces?  If yes,
 *   simulation; if no, rendering.
 *
 * Single instance (file-scope `g_scene`):
 *   Chain.nodes[] is the bulk of the storage (~25 KB), so the struct
 *   lives in BSS as a file-static rather than being passed by pointer.
 *   All scene state is accessed as `g_scene.<field>` from the helpers
 *   and the main loop.
 *
 * What stays OUTSIDE this struct (intentionally):
 *   g_quit_flag / g_resize_flag    sig_atomic_t flags read by signal
 *                                  handlers; must stay at file scope
 *                                  for async-signal safety.
 *   g_rows / g_cols                screen geometry tracked by the main
 *                                  loop; Scene stays geometry-agnostic
 *                                  so resize handling is the main
 *                                  loop's concern, not the renderer's.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
  /* ── Simulation parameters ────────────────────────────────────── */
  Chain chain; /* the physics state itself (§5)              */
  int sim_fps; /* sim/render rate, ] / [ keys (10–120)       */

  /* ── Rendering parameters ─────────────────────────────────────── */
  int theme; /* index into k_themes[] in §3; t / T keys   */
} Scene;

static Scene g_scene = {
    .sim_fps = SIM_FPS_DEFAULT, .theme = 0,
    /* .chain is BSS-zeroed; scene_init() populates it before any read. */
};

/* Screen geometry — main-loop bookkeeping, not scene state
 * (see Scene docstring above). */
static int g_rows, g_cols;

static const char *k_preset_names[N_PRESETS] = {
    "Hanging", "Pendulum", "Bridge",    "Wave", "Festoon",
    "Snake",   "Storm",    "Slingshot", "Flag", "Double"};

/*
 * Common initialisation: zero the chain, set n_nodes, compute link_rest
 * so the full chain is ~55% of screen height.
 */
static void chain_init_common(Chain *c, int n_nodes, float link_rest) {
  memset(c, 0, sizeof *c);
  c->n_nodes = n_nodes;
  c->link_rest = link_rest;
  c->n_iter = N_ITER_DEF;
  c->wind_on = true;
  c->trail_head = 0;
  c->trail_cnt = 0;
}

/* ── Preset 0: Hanging ────────────────────────────────────────────── */
/*
 * Top node pinned at screen centre-top.
 * Nodes hang straight down initially.
 * Wind oscillates left-right; produces sway.
 */
static void preset_hanging(Chain *c) {
  float anchor_x = (float)g_cols * CELL_W * 0.5f;
  float anchor_y = (float)g_rows * CELL_H * 0.07f;
  float total_len = (float)g_rows * CELL_H * 0.75f;
  int N = N_NODES_DEF;
  float rest = total_len / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_str = 140.f;
  c->preset = 0;

  for (int i = 0; i < N; i++) {
    c->nodes[i].x = c->nodes[i].ox = anchor_x;
    c->nodes[i].y = c->nodes[i].oy = anchor_y + (float)i * rest;
    c->nodes[i].pinned = (i == 0);
  }
}

/* ── Preset 1: Pendulum ───────────────────────────────────────────── */
/*
 * Top node pinned. Chain starts at 60° from vertical.
 * Released from rest — swings back and forth, wave patterns emerge.
 */
static void preset_pendulum(Chain *c) {
  float anchor_x = (float)g_cols * CELL_W * 0.5f;
  float anchor_y = (float)g_rows * CELL_H * 0.07f;
  float total_len = (float)g_rows * CELL_H * 0.70f;
  int N = N_NODES_DEF;
  float rest = total_len / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_on = false;
  c->preset = 1;

  float angle = (float)M_PI / 3.f; /* 60° from vertical (π/3 radians)
                                    * Large enough angle for dramatic
                                    * swing without going past horizontal */
  float sin_a = sinf(angle), cos_a = cosf(angle);

  for (int i = 0; i < N; i++) {
    float t = (float)i * rest;
    c->nodes[i].x = c->nodes[i].ox = anchor_x + sin_a * t;
    c->nodes[i].y = c->nodes[i].oy = anchor_y + cos_a * t;
    c->nodes[i].pinned = (i == 0);
  }
}

/* ── Preset 2: Bridge ─────────────────────────────────────────────── */
/*
 * Two anchor nodes pinned at equal height, separated by ~65% screen width.
 * Link rest length is 1.25× the straight distance / (N-1) → natural sag.
 * Wind gusts upward (like a flag in the wind from below).
 */
static void preset_bridge(Chain *c) {
  float span = (float)g_cols * CELL_W * 0.65f;
  float ax = (float)g_cols * CELL_W * 0.175f;
  float ay = (float)g_rows * CELL_H * 0.28f;
  int N = N_NODES_DEF;
  /* rest 25% longer than the straight-line distance between anchors:
   * Catenary physics: a chain longer than the span must sag.  The
   * extra 25% of length has to go somewhere — it forms a parabolic
   * (approximately catenary) droop.  More excess → deeper sag.        */
  float rest = span * 1.25f / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_str = 110.f;
  c->wind_on = true;
  c->preset = 2;

  /* initialise along straight line between the two anchors */
  for (int i = 0; i < N; i++) {
    float t = (float)i / (float)(N - 1);
    c->nodes[i].x = c->nodes[i].ox = ax + t * span;
    c->nodes[i].y = c->nodes[i].oy = ay;
    c->nodes[i].pinned = (i == 0 || i == N - 1);
  }
}

/* ── Preset 3: Wave ───────────────────────────────────────────────── */
/*
 * Top node driven with sinusoidal horizontal displacement.
 * Wave travels down the chain, reflects at the free bottom end, and
 * interferes with the incident wave — standing waves emerge at resonant
 * frequencies. The wave speed varies with tension (higher near the top).
 */
static void preset_wave(Chain *c) {
  float anchor_x = (float)g_cols * CELL_W * 0.5f;
  float anchor_y = (float)g_rows * CELL_H * 0.06f;
  float total_len = (float)g_rows * CELL_H * 0.80f;
  int N = N_NODES_DEF;
  float rest = total_len / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_on = false;
  c->preset = 3;
  c->wave_ax = anchor_x;
  c->wave_ay = anchor_y;
  c->wave_phase = 0.f;

  /* start hanging straight down */
  for (int i = 0; i < N; i++) {
    c->nodes[i].x = c->nodes[i].ox = anchor_x;
    c->nodes[i].y = c->nodes[i].oy = anchor_y + (float)i * rest;
    c->nodes[i].pinned = (i == 0);
  }
}

/* ── Preset 4: Festoon ────────────────────────────────────────────── */
/*
 * Three pins evenly placed along the top edge (at indices 0, N/2, N-1).
 * Total chain length is 1.5× the pin-to-pin distance so each of the two
 * segments forms a relaxed catenary sag — a Christmas-garland silhouette.
 * Gentle wind makes the festoons sway as a coupled pair.
 */
static void preset_festoon(Chain *c) {
  int N = N_NODES_DEF;
  float W = (float)g_cols * CELL_W;
  float top_y = (float)g_rows * CELL_H * 0.12f;
  float px0 = W * 0.10f;
  float px1 = W * 0.50f;
  float px2 = W * 0.90f;
  /* Total chain length = 1.5 × full span (each segment gets ~1.5× sag). */
  float total = (px2 - px0) * 1.50f;
  float rest = total / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_str = 60.f; /* gentle gust so festoons rock slowly   */
  c->wind_on = true;
  c->preset = 4;

  int p_mid = N / 2;
  for (int i = 0; i < N; i++) {
    float t = (float)i / (float)(N - 1);
    float x = (t <= 0.5f) ? px0 + (t * 2.f) * (px1 - px0)
                          : px1 + ((t - 0.5f) * 2.f) * (px2 - px1);
    c->nodes[i].x = c->nodes[i].ox = x;
    c->nodes[i].y = c->nodes[i].oy = top_y;
    c->nodes[i].pinned = (i == 0 || i == p_mid || i == N - 1);
  }
}

/* ── Preset 5: Snake ──────────────────────────────────────────────── */
/*
 * Bridge geometry (both ends pinned at equal height), slight slack.
 * chain_tick drives node 0 sinusoidally in Y (vertical), so a transverse
 * wave propagates along the chain, reflects off the far fixed end, and
 * interferes — the chain slithers like a serpent.
 *
 * Drive is rewired in chain_tick under `if (c->preset == 5)`.
 */
static void preset_snake(Chain *c) {
  int N = N_NODES_DEF;
  float span = (float)g_cols * CELL_W * 0.70f;
  float ax = (float)g_cols * CELL_W * 0.15f;
  float ay = (float)g_rows * CELL_H * 0.45f;
  float rest = span * 1.05f / (float)(N - 1); /* 5% slack             */

  chain_init_common(c, N, rest);
  c->wind_on = false;
  c->preset = 5;
  c->wave_ax = ax;
  c->wave_ay = ay;
  c->wave_phase = 0.f;

  /* Initial: straight horizontal — drive in tick produces the wave. */
  for (int i = 0; i < N; i++) {
    float t = (float)i / (float)(N - 1);
    c->nodes[i].x = c->nodes[i].ox = ax + t * span;
    c->nodes[i].y = c->nodes[i].oy = ay;
    c->nodes[i].pinned = (i == 0 || i == N - 1);
  }
}

/* ── Preset 6: Storm ──────────────────────────────────────────────── */
/*
 * Same geometry as Hanging but the wind is dialed up by ~4× — strong
 * enough that gravity is no longer the dominant force.  The chain
 * whips into near-horizontal arcs at each gust peak.
 */
static void preset_storm(Chain *c) {
  int N = N_NODES_DEF;
  float ax = (float)g_cols * CELL_W * 0.5f;
  float ay = (float)g_rows * CELL_H * 0.07f;
  float total = (float)g_rows * CELL_H * 0.75f;
  float rest = total / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_str = 580.f; /* ≈ 1.5× gravity at peak gust          */
  c->wind_on = true;
  c->preset = 6;

  for (int i = 0; i < N; i++) {
    c->nodes[i].x = c->nodes[i].ox = ax;
    c->nodes[i].y = c->nodes[i].oy = ay + (float)i * rest;
    c->nodes[i].pinned = (i == 0);
  }
}

/* ── Preset 7: Slingshot ──────────────────────────────────────────── */
/*
 * Pendulum geometry, but released from 150° (chain points up-and-right,
 * 30° from vertical-up).  Anchor sits low on screen so the full ~330°
 * sweep fits inside the frame.  No wind, no drag damping beyond DAMP;
 * the swing is the entire show.
 */
static void preset_slingshot(Chain *c) {
  int N = N_NODES_DEF;
  float ax = (float)g_cols * CELL_W * 0.5f;
  float ay = (float)g_rows * CELL_H * 0.60f; /* low — leave swing room */
  float total = (float)g_rows * CELL_H * 0.40f;
  float rest = total / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_on = false;
  c->preset = 7;

  float angle = 150.f * (float)M_PI / 180.f; /* 150° from +y (down) */
  float sin_a = sinf(angle), cos_a = cosf(angle);
  for (int i = 0; i < N; i++) {
    float t = (float)i * rest;
    c->nodes[i].x = c->nodes[i].ox = ax + sin_a * t;
    c->nodes[i].y = c->nodes[i].oy = ay + cos_a * t; /* cos(150°) < 0 → up */
    c->nodes[i].pinned = (i == 0);
  }
}

/* ── Preset 8: Flag ───────────────────────────────────────────────── */
/*
 * Single pin at the top-left, chain initialised stretched horizontally
 * to the right.  Strong sustained wind drives the chain into a flapping
 * banner motion — gravity wants to fold it down, wind wants to push it
 * right, the constraint network mediates a curling, flag-like waveform.
 */
static void preset_flag(Chain *c) {
  int N = N_NODES_DEF;
  float pin_x = (float)g_cols * CELL_W * 0.10f;
  float pin_y = (float)g_rows * CELL_H * 0.40f;
  float total = (float)g_cols * CELL_W * 0.65f;
  float rest = total / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_str = 320.f; /* strong horizontal gust               */
  c->wind_on = true;
  c->preset = 8;

  for (int i = 0; i < N; i++) {
    c->nodes[i].x = c->nodes[i].ox = pin_x + (float)i * rest;
    c->nodes[i].y = c->nodes[i].oy = pin_y;
    c->nodes[i].pinned = (i == 0);
  }
}

/* ── Preset 9: Double ─────────────────────────────────────────────── */
/*
 * Two pins: i = 0 (anchor) and i = N/3 (side-hook).  The upper third is
 * a straight diagonal between the pins; the lower two-thirds hangs as a
 * free pendulum from the hook, released at 45° from vertical.  Visually
 * a coupled-pendulum / clock-arm setup that swings around the hook.
 */
static void preset_double(Chain *c) {
  int N = N_NODES_DEF;
  int p_mid = N / 3;
  float anchor_x = (float)g_cols * CELL_W * 0.30f;
  float anchor_y = (float)g_rows * CELL_H * 0.18f;
  float hook_x = (float)g_cols * CELL_W * 0.55f;
  float hook_y = (float)g_rows * CELL_H * 0.35f;
  float upper_dx = hook_x - anchor_x;
  float upper_dy = hook_y - anchor_y;
  float upper_L = sqrtf(upper_dx * upper_dx + upper_dy * upper_dy);
  float lower_L = (float)g_rows * CELL_H * 0.45f;
  float rest = (upper_L + lower_L) / (float)(N - 1);

  chain_init_common(c, N, rest);
  c->wind_on = false;
  c->preset = 9;

  float lo_ang = (float)M_PI / 4.f; /* 45° from vertical */
  float lo_sx = sinf(lo_ang), lo_cx = cosf(lo_ang);
  for (int i = 0; i < N; i++) {
    if (i <= p_mid) {
      float t = (float)i / (float)p_mid;
      c->nodes[i].x = c->nodes[i].ox = anchor_x + t * upper_dx;
      c->nodes[i].y = c->nodes[i].oy = anchor_y + t * upper_dy;
    } else {
      float s = (float)(i - p_mid) * rest;
      c->nodes[i].x = c->nodes[i].ox = hook_x + lo_sx * s;
      c->nodes[i].y = c->nodes[i].oy = hook_y + lo_cx * s;
    }
    c->nodes[i].pinned = (i == 0 || i == p_mid);
  }
}

static void scene_init(int preset) {
  g_scene.chain.preset = preset;
  switch (preset) {
  case 0:
    preset_hanging(&g_scene.chain);
    break;
  case 1:
    preset_pendulum(&g_scene.chain);
    break;
  case 2:
    preset_bridge(&g_scene.chain);
    break;
  case 3:
    preset_wave(&g_scene.chain);
    break;
  case 4:
    preset_festoon(&g_scene.chain);
    break;
  case 5:
    preset_snake(&g_scene.chain);
    break;
  case 6:
    preset_storm(&g_scene.chain);
    break;
  case 7:
    preset_slingshot(&g_scene.chain);
    break;
  case 8:
    preset_flag(&g_scene.chain);
    break;
  case 9:
    preset_double(&g_scene.chain);
    break;
  }
}

/* ===================================================================== */
/* §7  screen / HUD                                                       */
/* ===================================================================== */

static void screen_init(void) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  color_init();
}

/*
 * Two-bar HUD per CLAUDE.md convention:
 *   row 0  right (CP_HUD  bright yellow + bold)  — live status
 *   row -1 left  (CP_HINT bright cyan   + bold)  — actions / keys
 */
static void draw_hud_top(const Chain *c, int fps) {
  char top[180];
  snprintf(top, sizeof top,
           " preset:%s  theme:%s  iter:%d  wind:%s  %s  %d fps ",
           k_preset_names[c->preset], k_themes[g_scene.theme].name, c->n_iter,
           c->wind_on ? "ON" : "OFF", c->paused ? "PAUSED " : "running", fps);
  int len = (int)strlen(top);
  int col = g_cols - len;
  if (col < 0)
    col = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, col, top, g_cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

static void draw_hud_bottom(void) {
  const char *hint_full = " q:quit  p:pause  r:reset  n/N:preset  t/T:theme  "
                          "+/-:iter  w:wind  ]/[:fps ";
  const char *hint_short = " q:quit  p:pause  r:reset  n:preset  t:theme ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= g_cols - 1)
    hint = hint_short;

  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(g_rows - 1, 0, hint, g_cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit_flag = 0;
static volatile sig_atomic_t g_resize_flag = 0;

static void sig_h(int s) {
  if (s == SIGINT || s == SIGTERM)
    g_quit_flag = 1;
  if (s == SIGWINCH)
    g_resize_flag = 1;
}
static void do_cleanup(void) { endwin(); }

int main(void) {
  atexit(do_cleanup);
  signal(SIGINT, sig_h);
  signal(SIGTERM, sig_h);
  signal(SIGWINCH, sig_h);

  screen_init();
  getmaxyx(stdscr, g_rows, g_cols);

  /* Initial preset = 0 (Hanging); scene_init writes g_scene.chain.preset
   * so subsequent code reads the active preset from there. */
  scene_init(0);

  int64_t t_last = clock_ns();

  /* FPS counter */
  int64_t fps_acc = 0;
  int fps_cnt = 0;
  int fps_disp = 0;

  while (!g_quit_flag) {

    /* ── resize ── */
    if (g_resize_flag) {
      g_resize_flag = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, g_rows, g_cols);
      scene_init(g_scene.chain.preset);
      t_last = clock_ns();
      continue;
    }

    /* ── input ── */
    int ch;
    while ((ch = getch()) != ERR) {
      switch (ch) {
      case 'q':
      case 'Q':
      case 27:
        g_quit_flag = 1;
        break;

      case ' ':
      case 'p':
      case 'P':
        g_scene.chain.paused = !g_scene.chain.paused;
        break;

      case 'r':
      case 'R':
        scene_init(g_scene.chain.preset);
        break;

      case 'n':
        scene_init((g_scene.chain.preset + 1) % N_PRESETS);
        break;

      case 'N':
        scene_init((g_scene.chain.preset + N_PRESETS - 1) % N_PRESETS);
        break;

      case 't':
        g_scene.theme = (g_scene.theme + 1) % N_THEMES;
        theme_apply(g_scene.theme);
        break;

      case 'T':
        g_scene.theme = (g_scene.theme + N_THEMES - 1) % N_THEMES;
        theme_apply(g_scene.theme);
        break;

      case '+':
      case '=':
        if (g_scene.chain.n_iter < N_ITER_MAX)
          g_scene.chain.n_iter += 5;
        break;

      case '-':
        if (g_scene.chain.n_iter > N_ITER_MIN)
          g_scene.chain.n_iter -= 5;
        break;

      case 'w':
      case 'W':
        g_scene.chain.wind_on = !g_scene.chain.wind_on;
        break;

      case ']':
        if (g_scene.sim_fps < SIM_FPS_MAX)
          g_scene.sim_fps += SIM_FPS_STEP;
        break;

      case '[':
        if (g_scene.sim_fps > SIM_FPS_MIN)
          g_scene.sim_fps -= SIM_FPS_STEP;
        break;
      }
    }

    /* ── tick ── */
    int64_t t_now = clock_ns();
    int64_t t_used = t_now - t_last;
    t_last = t_now;
    /* Cap dt at 100 ms (= 10 fps equivalent).
     * If the process was suspended (e.g., Ctrl-Z, terminal resize storm),
     * t_used could be seconds.  Feeding that into the physics would
     * launch nodes off-screen.  100 ms is a safe upper bound.          */
    if (t_used > 100000000LL)
      t_used = 100000000LL; /* 100 ms cap */

    float dt = (float)t_used * 1e-9f;
    chain_tick(&g_scene.chain, dt);

    /* ── draw ── */
    erase();
    chain_draw(&g_scene.chain, g_cols, g_rows);
    draw_hud_top(&g_scene.chain, fps_disp);
    draw_hud_bottom();
    wnoutrefresh(stdscr);
    doupdate();

    /* ── FPS cap ── */
    int64_t t_frame = TICK_NS(g_scene.sim_fps);
    int64_t t_sleep = t_frame - (clock_ns() - t_now);
    clock_sleep_ns(t_sleep);

    /* ── FPS counter ── */
    fps_acc += t_used;
    fps_cnt++;
    if (fps_acc >= NS_PER_SEC / 2) {
      fps_disp = fps_cnt * 2;
      fps_acc = 0;
      fps_cnt = 0;
    }
  }

  return 0;
}
