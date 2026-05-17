/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * gear.c — wireframe rotating gear with themed spark effects
 *
 * 10 spark/gear themes cycled with 't'.
 * Every tooth tip emits sparks carrying the gear's tangential surface
 * velocity. Faster rotation = faster sparks + more of them.
 *
 * Keys:
 *   q / ESC   quit
 *   spc       pause / resume
 *   r         reset
 *   + / -     spin faster / slower
 *   ] / [     more / fewer sparks
 *   t / T     next / prev theme
 *   1–5       speed presets  (very slow → screaming fast)
 *
 * Themes:
 *   0 FIRE    1 MATRIX   2 PLASMA   3 NOVA     4 POISON
 *   5 OCEAN   6 GOLD     7 NEON     8 ARCTIC   9 LAVA
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/gear.c -o gear -lncurses -lm
 *
 * §1 config  §2 clock  §3 theme  §4 color  §5 coords
 * §6 entity  §7 draw   §8 screen §9 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic polar gear geometry [4] + particle-system
 *                  sparks [1].  Gear outline computed each frame from
 *                  N_TEETH tooth definitions (outer radius, inner radius,
 *                  hub) using polar angle stepping.  No mesh storage —
 *                  geometry is regenerated every frame from the current
 *                  rotation angle.  Particle pipeline (emit → age →
 *                  integrate → render) is the classical Reeves loop [1]
 *                  with per-stage glyph/colour variants in the spirit of
 *                  Sims [5].
 *
 * Math           : Involute gear tooth approximated as trapezoid in polar
 *                  coordinates [2].  Tooth tip position at angle α:
 *                    (x,y) = GEAR_R_OUTER × (cos α, sin α)
 *                  Tangential surface velocity at tooth tip:
 *                    v_tip = GEAR_R_OUTER × ω  (ω = angular speed rad/s)
 *                  Spark initial velocity = tangential v_tip
 *                                         + radial outward kick
 *                                         + uniform scatter.
 *
 * Physics        : Per-spark semi-implicit Euler [3] in integrate_spark:
 *                    vy += g · dt          (gravity)
 *                    v  *= exp(−k · dt)    (exponential drag, closed-form)
 *                    pos += v · dt
 *                  Closed-form drag stays stable for any dt — a poor-man's
 *                  symplectic step that's fine for short-lived particles.
 *
 * Rendering      : Cell-aspect correction: CELL_W=8, CELL_H=16 → pixels
 *                  are taller than wide; using identical r=√(dx²+dy²) on
 *                  screen-pixel space keeps the gear visually circular.
 *                  Edge glyphs picked by 8-sector quantisation of the
 *                  local angle [6] (one of {-, \, |, /} per 45° sector).
 *                  Fresh sparks (stages 0..2) render as fading comet
 *                  streaks (bright head + dim tail along velocity);
 *                  cooler sparks (3..6) fall back to the theme dot glyph.
 *                  Theme palettes [7] are perceptually-ordered cooling
 *                  ramps (white-hot → ember) in the 256-colour cube.
 *
 * Data-structure : Spark pool — fixed array of MAX_SPARKS structs with
 *                  active flag (life > 0).  O(N) per frame; emission rate
 *                  scales with ω and is hard-capped at 1200 sparks/s.
 *
 * References (cite inline as [n]):
 *
 *   [1] Reeves, W. T. (1983) — "Particle Systems — A Technique for
 *       Modeling a Class of Fuzzy Objects", *ACM Transactions on
 *       Graphics* 2 (2), 91–108.  The foundational particle-system
 *       paper: emit at a source, age each particle, integrate
 *       position/velocity, kill when life ≤ 0, render per-frame.
 *       Our gear_tick / spark_emit / draw_sparks follow this loop
 *       verbatim — and the original Genesis-effect demo is the
 *       direct ancestor of every "spray of glowing sparks" since.
 *
 *   [2] Buckingham, E. (1949) — *Analytical Mechanics of Gears*,
 *       McGraw-Hill (reprinted Dover, 1988).  Classic engineering
 *       reference for involute tooth geometry: addendum/dedendum,
 *       pitch circle, tooth thickness as a fraction of the wedge
 *       (our TOOTH_DUTY = 0.42).  Backs the trapezoidal tooth
 *       approximation used in draw_gear (§7).
 *
 *   [3] Witkin, A.; Baraff, D. (2001) — "Physically Based Modeling:
 *       Principles and Practice", SIGGRAPH course notes.  Practical
 *       recipes for Euler / semi-implicit integration of particle
 *       motion under gravity and drag, including the closed-form
 *       exp(−k·dt) damping factor used in integrate_spark.
 *
 *   [4] Bloomenthal, J. (ed.) (1997) — *Introduction to Implicit
 *       Surfaces*, Morgan Kaufmann.  Per-cell evaluation of analytic
 *       predicates on (r, θ) is exactly what draw_gear does: instead
 *       of rasterising stored geometry, every cell asks "am I on the
 *       hub ring / a spoke / a tooth side / an arc?" against thin
 *       thresholds around each implicit curve.
 *
 *   [5] Sims, K. (1990) — "Particle Animation and Rendering Using
 *       Data Parallel Computation", *SIGGRAPH '90 Proceedings*,
 *       vol. 24 (4), 405–413.  Per-particle attributes (life,
 *       colour, size) varying with age — the source of our 7-stage
 *       cooling ladder of glyphs, colours and attributes per theme.
 *
 *   [6] Bresenham, J. E. (1965) — "Algorithm for Computer Control
 *       of a Digital Plotter", *IBM Systems Journal* 4 (1), 25–30.
 *       Two uses in this file: (a) 8-direction line discretisation
 *       — 360° quantised into eight 45° sectors mapped to {-, \, |,
 *       /}, used in line_char and direction_glyph; (b) the DDA
 *       integer-from-real-rate accumulator pattern used by
 *       emit_via_dda_accumulator to emit floor(rate·dt) sparks per
 *       frame on average, carrying the fractional remainder.
 *
 *   [7] Ware, C. (2020) — *Information Visualization: Perception
 *       for Design*, 4th ed., Morgan Kaufmann.  Perceptually-ordered
 *       luminance ramps (Ch. 4) back the 10 theme palettes in §3:
 *       each spark_fg[0..6] sequence is a monotonic cooling ramp in
 *       the 256-colour cube, so the spray reads as fire fading even
 *       when individual hues differ wildly between themes.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The gear is an IMPLICIT FIELD, not a mesh.  For every cell on screen
 * within the gear's bounding box, ask: "given my polar (r, θ) relative
 * to the gear centre and rotation, do I lie on the hub ring, on a spoke,
 * on a tooth side, or on the inner/outer arc?"  Each test is a thin
 * threshold around an analytic curve.  Sparks are the OPPOSITE: explicit
 * particles emitted from tooth tips, carrying the gear's tangential
 * surface velocity, then integrating gravity + drag + turbulence until
 * their life timer expires through 7 cooling stages.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * The gear is like checking whether you're standing on a painted line
 * by looking at your GPS coordinates — no need to walk the line.  For
 * every screen cell, convert to (r, θ), subtract the gear's rotation,
 * and run a 7-tier priority chain (5 bright wireframe outlines + 2 dim
 * interior fills) — see classify_gear_cell.  The teeth appear because
 * the outer-arc test only fires when the local θ falls inside a tooth
 * slice (TOOTH_DUTY = 42 % of each 1/N_TEETH wedge).  Sparks then are
 * tiny welding chips flying off the spinning rim — they inherit the
 * rim's tangential speed plus an outward radial kick, then gravity
 * wins and they arc downward as they cool through 7 colours.
 *
 * ALGORITHM IN STEPS  (per frame — helper names track the code)
 * ──────────────────
 *  gear_tick(g, dt):
 *    1. gear_advance_rotation     :  angle += ω·dt; wrap mod TAU.
 *    2. emit_via_dda_accumulator  :  emit_acc += emission_rate_hz·dt;
 *                                    while (acc ≥ 1) pick a random
 *                                    tooth and spark_emit at its
 *                                    tooth_tip_angle.
 *    3. spark_emit                :  acquire_free_spark_slot,
 *                                    spawn_at_tooth_tip (position
 *                                    + zero velocity),
 *                                    v += rim_tangential_velocity
 *                                       + radial_outward_kick
 *                                       + uniform_jitter,
 *                                    seed_lifetime ∈ [0.85, 1.00].
 *    4. integrate_spark (each)    :  age → gravity → turb → drag → pos;
 *                                    kill if off-screen.
 *
 *  scene_draw(s, win):
 *    5. draw_sparks               :  stage = first STAGE_THRESH index
 *                                    where life > thresh; stages 0..2
 *                                    render as streaks, 3..6 as theme
 *                                    dot glyphs.
 *    6. draw_gear                 :  for each cell in clipped_gear_bbox:
 *                                      cell_to_gear_polar → (rad, ang_g,
 *                                                            ang_l);
 *                                      classify_gear_cell → CellPaint;
 *                                      paint_cell (skip if transparent).
 *
 * KEY FORMULAS
 * ────────────
 *  Tooth tip angle    :  α_t = gear.angle + (t + duty/2) · TAU / N_TEETH
 *  Tip world position :  (cx + R_outer·cos α, cy + R_outer·sin α)
 *  Tangential velocity:  v_tang = ω · R_outer · TANG_SCALE,
 *                        direction = (−sin α, cos α)
 *  Radial kick        :  v_kick  = (cos α, sin α) · U(KICK_MIN, KICK_MAX)
 *  Local angle        :  ang_l   = atan2(dy, dx) − gear.angle
 *  Tooth phase        :  phase   = frac(ang_l · N_TEETH / TAU)  ∈ [0,1)
 *  In-tooth test      :  phase < TOOTH_DUTY  (42 %)
 *  Spark life decay   :  life -= dt / SPARK_LIFE  (1.9 s total)
 *  Spark drag         :  v *= exp(−DRAG · dt)
 *  Stage              :  smallest i in 0..6 with life > STAGE_THRESH[i]
 *  Cell aspect fix    :  CELL_W=8, CELL_H=16 — pixel space already y-tall,
 *                        so the same r=√(dx²+dy²) gives a circular gear.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Spark pool exhaustion — acquire_free_spark_slot returns NULL when
 *    all 1500 are alive; spark_emit silently no-ops (fine for visual;
 *    rate self-throttles via emit_via_dda_accumulator).
 *  • emission_rate_hz is hard-capped at 1200 sparks/s to prevent
 *    runaway when ω is high AND density is high.
 *  • Theme switch must call color_apply_theme() to re-init pairs CP_S0..
 *    CP_S6; sparks already in flight immediately swap palette.
 *  • Inside classify_gear_cell, FIRST match wins — change the order
 *    and the gear loses parts.  Hub ring before spokes, spokes before
 *    tooth sides, etc.  See its priority-order doc for the full list.
 *  • Sparks dying off-screen: integrate_spark zeros life when px/py
 *    leave [0, max_px] × [0, max_py]; resize updates these before the
 *    next tick.
 *  • Tooth count and TOOTH_DUTY are tuned together — at 10 teeth and
 *    duty 0.42, gap is 0.58 of each wedge so tips read clearly.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press `1` (slow): rotation visible, spark count low (~50–100).
 *    Press `5` (fast): emission climbs near 1200/s cap, sparks form a
 *    bright halo, gear nearly blurs.
 *  • Spark stream ALWAYS leaves a tooth tip tangent-first — at any
 *    instant the freshest sparks lie on the tangent line through the
 *    spinning tooth they just left.
 *  • Sparks fall — vy increases monotonically until drag balances
 *    gravity, so the spray bows downward, never upward, regardless of
 *    rotation direction.
 *  • Theme cycle: press `t` 10 times → returns to FIRE; spark glyphs
 *    and colour ramp change but spark physics is identical.
 *  • Pause (space): gear freezes mid-rotation; sparks freeze in place.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
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

#define CELL_W   8
#define CELL_H  16

/* Gear geometry (pixels) */
#define GEAR_R_OUTER   88.0f
#define GEAR_R_INNER   64.0f
#define GEAR_R_HUB     22.0f
#define N_TEETH        10
#define TOOTH_DUTY     0.42f

/* Wireframe thresholds (pixels) */
#define THRESH_CIRC    7.5f
#define THRESH_SIDE    4.0f
#define THRESH_SPOKE   3.8f

/* Gear rotation */
#define GEAR_ROT_BASE   1.3f
#define GEAR_ROT_STEP   0.6f
#define GEAR_ROT_MAX   20.0f

/* Spark physics */
#define MAX_SPARKS      1500
#define SPARK_BASE_RATE  80.0f
#define TANG_SCALE        0.5f
#define SPARK_KICK_MIN   35.0f
#define SPARK_KICK_MAX  120.0f
#define SPARK_SCATTER    25.0f   /* keep low so streaks stay directional */
#define SPARK_TURB       15.0f   /* keep low so trails stay clean        */
#define SPARK_GRAVITY    28.0f
#define SPARK_DRAG        0.4f
#define SPARK_LIFE        1.9f

#define DENSITY_DEFAULT   1.0f
#define DENSITY_STEP      0.3f
#define DENSITY_MAX       6.0f

#define TAU  6.28318530f
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

enum { TARGET_FPS = 60, FPS_UPDATE_MS = 500 };

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
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  themes                                                             */
/* ===================================================================== */

/*
 * Each theme describes:
 *   gear_fg      — 256-color index for bright gear wireframe
 *   gear_dim_fg  — 256-color index for dim gear lines
 *   spark_fg[7]  — color per cooling stage (freshest → dead)
 *   spark_ch[7]  — character per stage
 *   spark_at[7]  — attribute per stage: 1=A_BOLD 0=A_NORMAL 2=A_DIM
 */
typedef struct {
    const char *name;
    int  gear_fg;
    int  gear_dim_fg;
    int  spark_fg[7];
    char spark_ch[7];
    int  spark_at[7];
} Theme;

#define N_THEMES 10

/* spark_at encoding: 0=A_NORMAL  1=A_BOLD  2=A_DIM */

static const Theme THEMES[N_THEMES] = {

    /* ── 0  FIRE ─────────────────────────────────────────────────────── */
    /* white-hot → yellow → amber → orange → red-orange → red → ember    */
    { "FIRE",
      153,  67,
      { 231, 226, 220, 214, 202, 196, 160 },
      { '*', '*', '+', '+', '.', '.', ',' },
      {   1,   1,   1,   0,   0,   2,   2 } },

    /* ── 1  MATRIX ───────────────────────────────────────────────────── */
    /* white flash → lime → green → mid-green → dark; digital rain vibe  */
    { "MATRIX",
       34,  22,
      { 231, 118,  82,  46,  40,  34,  22 },
      { '@', '#', '*', '+', ';', ':', '.' },
      {   1,   1,   1,   0,   0,   2,   2 } },

    /* ── 2  PLASMA ───────────────────────────────────────────────────── */
    /* white → pink → hot-magenta → purple → deep violet; electric arc   */
    { "PLASMA",
       99,  57,
      { 231, 207, 201, 165, 129,  93,  57 },
      { '*', '*', '+', '+', '.', '.', ',' },
      {   1,   1,   1,   0,   0,   2,   2 } },

    /* ── 3  NOVA ─────────────────────────────────────────────────────── */
    /* white → bright-cyan → cyan → sky-blue → blue → deep-blue; stellar */
    { "NOVA",
       69,  27,
      { 231, 159, 123,  87,  51,  39,  27 },
      { '*', '*', '+', '+', '.', '.', ',' },
      {   1,   1,   1,   0,   0,   2,   2 } },

    /* ── 4  POISON ───────────────────────────────────────────────────── */
    /* white → bright-yellow → yellow-green → lime → green; toxic waste  */
    { "POISON",
       64,  22,
      { 231, 226, 190, 154, 118,  82,  64 },
      { '*', '*', '+', '+', '.', '.', ',' },
      {   1,   1,   1,   0,   0,   2,   2 } },

    /* ── 5  OCEAN ────────────────────────────────────────────────────── */
    /* white → ice → cyan → ocean → deep-blue; bioluminescent            */
    { "OCEAN",
       31,  23,
      { 231, 159, 123,  81,  45,  33,  24 },
      { '~', 'o', '~', '+', '.', ',', '.' },
      {   1,   1,   1,   0,   0,   2,   2 } },

    /* ── 6  GOLD ─────────────────────────────────────────────────────── */
    /* white → pale-gold → gold → ochre → dark-gold → bronze → copper    */
    { "GOLD",
      136,  94,
      { 231, 228, 220, 178, 136, 130,  94 },
      { '*', '*', '+', '+', '.', '.', ',' },
      {   1,   1,   1,   0,   0,   2,   2 } },

    /* ── 7  NEON ─────────────────────────────────────────────────────── */
    /* white → light-pink → hot-pink → magenta → deep-pink; synthwave    */
    { "NEON",
      201, 164,
      { 231, 219, 213, 207, 201, 200, 164 },
      { '*', '*', '+', '+', '.', '.', ',' },
      {   1,   1,   1,   0,   0,   2,   2 } },

    /* ── 8  ARCTIC ───────────────────────────────────────────────────── */
    /* white → pale-blue → ice → steel → periwinkle → cornflower → grey  */
    { "ARCTIC",
      153,  67,
      { 231, 195, 159, 153, 117,  75,  67 },
      { '*', '*', '+', '.', '.', ',', '`' },
      {   1,   1,   1,   0,   2,   2,   2 } },

    /* ── 9  LAVA ─────────────────────────────────────────────────────── */
    /* white → amber → deep-orange → red → dark-red → crimson → black-red*/
    { "LAVA",
       88,  52,
      { 231, 220, 208, 202, 196, 124,  52 },
      { '*', '*', '+', '+', '.', '.', ',' },
      {   1,   1,   1,   0,   0,   2,   2 } },
};

/* life thresholds for the 7 spark stages (inclusive lower bound)        */
static const float STAGE_THRESH[7] = {
    0.85f, 0.70f, 0.55f, 0.38f, 0.22f, 0.10f, 0.00f
};

static inline int spark_stage(float life)
{
    for (int i = 0; i < 6; i++)
        if (life > STAGE_THRESH[i]) return i;
    return 6;
}

/* ===================================================================== */
/* §4  color                                                              */
/* ===================================================================== */

enum {
    CP_GEAR     = 1,   /* bright gear wireframe                           */
    CP_GEAR_DIM = 2,   /* dim gear lines                                  */
    CP_S0       = 3,   /* spark stage 0 (freshest)                        */
    /* CP_S1 = 4 … CP_S6 = 9 implicit */
    CP_HUD      = 10,
};

/* Decode attr int (0/1/2) → ncurses attr_t */
static const attr_t ATTR_DEC[3] = { A_NORMAL, A_BOLD, A_DIM };

static void color_apply_theme(int idx)
{
    const Theme *th = &THEMES[idx];
    if (COLORS >= 256) {
        init_pair(CP_GEAR,     th->gear_fg,     -1);
        init_pair(CP_GEAR_DIM, th->gear_dim_fg, -1);
        for (int i = 0; i < 7; i++)
            init_pair(CP_S0 + i, th->spark_fg[i], -1);
        init_pair(CP_HUD, 226, -1);
    } else {
        /* 8-color fallback: white gear, yellow/red sparks                */
        init_pair(CP_GEAR,     COLOR_WHITE,  -1);
        init_pair(CP_GEAR_DIM, COLOR_WHITE,  -1);
        int fb[7] = {
            COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
            COLOR_RED,   COLOR_RED,    COLOR_RED
        };
        for (int i = 0; i < 7; i++)
            init_pair(CP_S0 + i, fb[i], -1);
        init_pair(CP_HUD, COLOR_WHITE, -1);
    }
}

static void color_init(int theme)
{
    start_color();
    use_default_colors();
    color_apply_theme(theme);
}

/* ===================================================================== */
/* §5  coords                                                             */
/* ===================================================================== */

static inline int px_col(float px) { return (int)floorf(px / CELL_W); }
static inline int px_row(float py) { return (int)floorf(py / CELL_H); }

/* ===================================================================== */
/* §6  entity                                                             */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Spark — one particle in the Reeves emit/age/integrate/render pipeline [1].
 *
 * Why a fixed flat struct (no linked list, no absolute birth time):
 *   The pool (Gear.sparks[MAX_SPARKS]) is scanned linearly every frame.
 *   Storing absolute birth time would force every stage lookup to
 *   recompute age = now − birth; storing a *countdown* collapses that to
 *   "first STAGE_THRESH index crossed" — a single linear probe in
 *   spark_stage().  Cache line is already hot from integration.
 *
 * Lifetime contract:
 *   life > 0   → slot is alive; gear_tick integrates and ages it.
 *   life ≤ 0   → slot is free; spark_emit may overwrite it.
 *   spark_emit seeds life ∈ [0.85, 1.00] so freshly born sparks all land
 *   in stage 0 (the brightest tier in STAGE_THRESH).  Total life span
 *   is SPARK_LIFE (1.9 s) since decay = dt / SPARK_LIFE per frame.
 *
 * Why px/py are pixel-space sub-cell floats (not cell-row/col ints):
 *   Sparks move at well under 1 cell per frame; quantising at the
 *   integration step would freeze slow sparks in place.  Cell mapping
 *   happens only at draw time via px_col / px_row.  Pixel space is the
 *   project-wide physics convention (CELL_W=8, CELL_H=16; see CLAUDE.md).
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
    /* ── Kinematic state (pixel space; integrated by gear_tick) ───── */
    float px, py;   /* position in pixels; clamped to framebuffer rect */
    float vx, vy;   /* velocity in pixels/sec; gravity + drag + turb   */

    /* ── Age / colour selector (Reeves-style attribute aging [5]) ──── */
    float life;     /* 1.0 = just born, 0.0 = dead; drives stage 0..6
                     * via spark_stage() → STAGE_THRESH[] → glyph/colr */
} Spark;

/* ─────────────────────────────────────────────────────────────────────── *
 * Gear — the spinning ring + its particle pool, in one struct.
 *
 * Why a single owner (gear + sparks together):
 *   Sparks are emitted from teeth and inherit the rim's tangential
 *   velocity — their entire existence is bound to the gear.  Reset (r)
 *   zeros the whole struct in one call; sub-step integration touches
 *   every field through one pointer.  Splitting them into a Gear and a
 *   ParticleSystem would force every emit to take both pointers and
 *   re-derive the rim velocity each call — pure ceremony.
 *
 * Coupling via rot_speed (the ONE knob the user spins, +/- and 1..5):
 *     (a) visible rotation:  angle      += rot_speed · dt        (gear_tick)
 *     (b) emission rate:     total_rate ∝ rot_speed / GEAR_ROT_BASE
 *                                                                (gear_tick)
 *     (c) tangential v_tip:  rot_speed · GEAR_R_OUTER · TANG_SCALE
 *                                                                (spark_emit)
 *   A faster gear visibly throws faster sparks AND more of them — the
 *   perceptual coupling between speed and intensity is what makes the
 *   demo read as "machine working harder", not "knob turned".
 *
 * Why a flat pool (not linked free-list) — Reeves 1983 [1]:
 *   MAX_SPARKS=1500.  Linear scan in spark_emit is O(N) worst case but
 *   typically finds a free slot in the first few entries (sparks die
 *   in roughly emission order under uniform life).  A free-list would
 *   save scans but cost a pointer per slot and cache-thrash on every
 *   emit / death.  No malloc on the hot path — strict project rule.
 *
 * Why emit_acc is a float (not an integer counter) — DDA/Bresenham [6]:
 *   Emission rate is real-valued (e.g. 19.2 sparks/frame at 60 fps with
 *   rot_speed=2.0).  emit_acc += rate · dt then `while (acc ≥ 1) emit()`
 *   is the classical integer-from-real-rate pattern: emits exactly
 *   floor(rate · dt) on average each frame, carrying the fractional
 *   remainder forward.  Equivalent to a leaky bucket of size 1.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
    /* ── Gear pose (pixel space; polar parameters from §1) ────────── */
    float cx, cy;             /* gear centre in pixels (mid-screen)    */
    float angle;              /* current rotation in radians; wraps
                               * mod TAU each tick to avoid float drift */
    float rot_speed;          /* ω in rad/s; +/- keys, range [0.2, 20.0]
                               * couples to emission rate AND v_tip    */

    /* ── User-controlled emission knob ────────────────────────────── */
    float spark_density;      /* multiplier on emission rate; ] / [ keys
                               * range [0.2, 6.0], independent of ω so
                               * the user can dial intensity without
                               * also changing visible spin            */

    /* ── Particle pool (Reeves-style fixed allocation [1]) ────────── */
    Spark sparks[MAX_SPARKS]; /* life > 0 → active; life ≤ 0 → free    */
    float emit_acc;           /* fractional sparks owed to emission;
                               * DDA accumulator (see docstring above) */
} Gear;

static float randf(void) { return (float)rand() / (float)RAND_MAX; }

static void gear_init(Gear *g, float max_px, float max_py)
{
    g->cx            = max_px * 0.5f;
    g->cy            = max_py * 0.5f;
    g->angle         = 0.0f;
    g->rot_speed     = GEAR_ROT_BASE;
    g->spark_density = DENSITY_DEFAULT;
    g->emit_acc      = 0.0f;
    for (int i = 0; i < MAX_SPARKS; i++) g->sparks[i].life = 0.0f;
}

/* ── particle pool [1] ─────────────────────────────────────────────── */

/* Linear scan for the first dead slot.  Returns NULL if the pool is
 * full — caller should silently skip (rate self-throttles via the
 * DDA accumulator in emit_via_dda_accumulator). */
static Spark *acquire_free_spark_slot(Spark *pool, int n)
{
    for (int i = 0; i < n; i++)
        if (pool[i].life <= 0.0f) return &pool[i];
    return NULL;
}

/* ── velocity composition (each helper ADDS one term to s->v) ─────── *
 *
 * Reeves-style multi-source velocity composition [1]: spawn places the
 * spark and zeroes its velocity; each apply_*() then adds one physical
 * contribution.  Reading spark_emit top-to-bottom = reading the
 * velocity equation term-by-term.
 * ─────────────────────────────────────────────────────────────────── */

/* Place the spark at the tip of the tooth currently at tip_ang and
 * zero its velocity — ready for apply_*() to add velocity terms. */
static void spawn_spark_at_tooth_tip(Spark *s, float cx, float cy,
                                     float tip_ang)
{
    s->px = cx + GEAR_R_OUTER * cosf(tip_ang);
    s->py = cy + GEAR_R_OUTER * sinf(tip_ang);
    s->vx = 0.0f;
    s->vy = 0.0f;
}

/* Add the rim's tangential surface velocity at tip_ang.
 *   |v| = rot_speed · R_outer · TANG_SCALE
 *   dir = (−sin α, cos α)  — perpendicular to the radial, in the
 *                            spin direction.
 * This is what makes sparks fly off the rim tangent-first. */
static void apply_rim_tangential_velocity(Spark *s, float rot_speed,
                                          float tip_ang)
{
    float v = rot_speed * GEAR_R_OUTER * TANG_SCALE;
    s->vx += -sinf(tip_ang) * v;
    s->vy +=  cosf(tip_ang) * v;
}

/* Add a radial outward kick along (cos α, sin α) with random magnitude
 * in [KICK_MIN, KICK_MAX].  Gives the spark its initial "thrown off
 * the rim" character before tangential carry takes over. */
static void apply_radial_outward_kick(Spark *s, float tip_ang)
{
    float kick = SPARK_KICK_MIN
               + randf() * (SPARK_KICK_MAX - SPARK_KICK_MIN);
    s->vx += cosf(tip_ang) * kick;
    s->vy += sinf(tip_ang) * kick;
}

/* Add a symmetric uniform jitter in [−spread/2, +spread/2] per axis.
 * Without it, every spark from a given tooth would lie on the same
 * tangent line; with it, they spread into a visible spray. */
static void apply_uniform_jitter(Spark *s, float spread)
{
    s->vx += (randf() - 0.5f) * spread;
    s->vy += (randf() - 0.5f) * spread;
}

/* Seed the life timer.  Range [0.85, 1.00] places every newborn in
 * stage 0 (the brightest tier in STAGE_THRESH).  decay = dt/SPARK_LIFE
 * per frame → total lifespan ≈ SPARK_LIFE seconds. */
static void spark_seed_lifetime(Spark *s)
{
    s->life = 0.85f + randf() * 0.15f;
}

/*
 * spark_emit — the Reeves emit step [1].  Claim a slot, position it,
 * compose its velocity from three physical sources, seed its life.
 *
 * Pseudocode:
 *   slot = acquire_free_spark_slot(pool)
 *   if pool full: return            (emission self-throttles)
 *   spawn_at_tooth_tip(slot)        // position, zero velocity
 *   v += rim_tangential_velocity    // spin carry
 *   v += radial_outward_kick        // "thrown off"
 *   v += uniform_jitter             // spread into a spray
 *   seed_lifetime(slot)
 */
static void spark_emit(Gear *g, float tip_ang)
{
    Spark *s = acquire_free_spark_slot(g->sparks, MAX_SPARKS);
    if (!s) return;

    spawn_spark_at_tooth_tip      (s, g->cx, g->cy, tip_ang);
    apply_rim_tangential_velocity (s, g->rot_speed, tip_ang);
    apply_radial_outward_kick     (s, tip_ang);
    apply_uniform_jitter          (s, SPARK_SCATTER);
    spark_seed_lifetime           (s);
}

/* ── per-tick gear update ─────────────────────────────────────────── */

/* Advance gear rotation: angle += ω·dt, wrapped mod TAU to bound
 * float precision under indefinite spinning. */
static void gear_advance_rotation(Gear *g, float dt)
{
    g->angle += g->rot_speed * dt;
    if (g->angle > TAU) g->angle -= TAU;
}

/* Current spark emission rate (sparks/sec).
 *   rate = SPARK_BASE_RATE · (ω / ω_base) · density
 * Hard-capped at 1200/s so "screaming fast × max density" doesn't
 * starve the frame budget. */
static float emission_rate_hz(const Gear *g)
{
    float speed_norm = g->rot_speed / GEAR_ROT_BASE;
    float rate = SPARK_BASE_RATE * speed_norm * g->spark_density;
    return rate > 1200.0f ? 1200.0f : rate;
}

/* World-space tip angle of tooth t for the gear's current rotation.
 *   α_t = gear.angle + (t + TOOTH_DUTY/2) · TAU / N_TEETH
 * The +duty/2 centres emission inside the tooth wedge, not on its
 * leading edge. */
static float tooth_tip_angle(const Gear *g, int t)
{
    return g->angle + (t + TOOTH_DUTY * 0.5f) * TAU / N_TEETH;
}

/* DDA-style emission step — integer-from-real-rate [6]:
 *   emit_acc += rate·dt   (accumulate fractional sparks "owed")
 *   while (acc ≥ 1) emit one, subtract 1 (carry remainder forward)
 * Average emission rate matches emission_rate_hz over many frames. */
static void emit_via_dda_accumulator(Gear *g, float dt)
{
    g->emit_acc += emission_rate_hz(g) * dt;
    while (g->emit_acc >= 1.0f) {
        g->emit_acc -= 1.0f;
        int t = rand() % N_TEETH;
        spark_emit(g, tooth_tip_angle(g, t));
    }
}

/*
 * integrate_spark — per-spark physics: semi-implicit Euler with
 * closed-form exponential drag [3].  Order is significant — age first
 * (kills before integrating dead particles), then gravity → turbulence
 * → drag → position.
 *
 *   age      :  life -= dt / SPARK_LIFE          (linear countdown)
 *   gravity  :  vy   += g · dt                    (constant downward)
 *   turb     :  v    += U(−s/2, +s/2) · dt        (stochastic forcing)
 *   drag     :  v    *= exp(−k · dt)              (closed-form damping)
 *   advance  :  pos  += v · dt                    (explicit step)
 *   kill     :  if off-screen, life = 0
 */
static void integrate_spark(Spark *s, float dt, float max_px, float max_py)
{
    s->life -= dt / SPARK_LIFE;
    if (s->life <= 0.0f) { s->life = 0.0f; return; }

    /* gravity — constant vertical acceleration */
    s->vy += SPARK_GRAVITY * dt;

    /* turbulence — anisotropic (vertical 40 % of horizontal) so trails
     * stay readable as streaks rather than dissolving into noise */
    s->vx += (randf() - 0.5f) * SPARK_TURB * dt;
    s->vy += (randf() - 0.5f) * SPARK_TURB * 0.4f * dt;

    /* drag — exp(−k·dt) is the closed-form solution to dv/dt = −k·v,
     * stable for any dt unlike the explicit form v -= k·v·dt */
    float damp = expf(-SPARK_DRAG * dt);
    s->vx *= damp;
    s->vy *= damp;

    /* advance position */
    s->px += s->vx * dt;
    s->py += s->vy * dt;

    /* off-screen sparks die so they free pool slots */
    if (s->px < 0 || s->px > max_px || s->py < 0 || s->py > max_py)
        s->life = 0.0f;
}

/*
 * gear_tick — one fixed-dt step of the gear simulation.
 *
 * Pseudocode:
 *   1. gear_advance_rotation     // angle += ω·dt
 *   2. emit_via_dda_accumulator  // emit pending sparks
 *   3. integrate_spark for each live slot
 */
static void gear_tick(Gear *g, float dt, float max_px, float max_py)
{
    gear_advance_rotation     (g, dt);
    emit_via_dda_accumulator  (g, dt);
    for (int i = 0; i < MAX_SPARKS; i++) {
        if (g->sparks[i].life <= 0.0f) continue;
        integrate_spark(&g->sparks[i], dt, max_px, max_py);
    }
}

/* ===================================================================== */
/* §7  draw                                                               */
/* ===================================================================== */

/* paint_cell — bounds-checked single-glyph put with given attr.
 * Shared by every draw_* helper to write one cell into the framebuffer.
 * (Defined first so draw_gear below — and the spark drawers further
 * down — can both call it.) */
static void paint_cell(WINDOW *win, int r, int c, chtype ch,
                       int cp, attr_t at, int cols, int rows)
{
    if (c < 0 || c >= cols || r < 0 || r >= rows) return;
    wattron(win, COLOR_PAIR(cp) | at);
    mvwaddch(win, r, c, ch);
    wattroff(win, COLOR_PAIR(cp) | at);
}

/*
 * line_char — wireframe edge glyph picked by 8-sector quantisation of
 * ang (Bresenham-style 8-direction line discretisation [6]).
 *   ang=0   → '-'   ang=π/4 → '\'   ang=π/2 → '|'   ang=3π/4 → '/'
 * The same four glyphs repeat in the opposite half — direction doesn't
 * matter for an unoriented line, only its slope.
 */
static chtype line_char(float ang)
{
    float a = fmodf(ang + TAU, TAU);
    if (a < TAU/ 16 || a >= TAU*15/16) return '-';
    if (a < TAU* 3/16) return '\\';
    if (a < TAU* 5/16) return '|';
    if (a < TAU* 7/16) return '/';
    if (a < TAU* 9/16) return '-';
    if (a < TAU*11/16) return '\\';
    if (a < TAU*13/16) return '|';
    return '/';
}

/* ── gear cell sampler & implicit-field tests [4] ─────────────────── */

/*
 * Rectangle (in cells) covering the gear, clipped to the visible
 * window.  Lets draw_gear iterate only cells that could possibly hit
 * any part of the gear (anything beyond R_OUTER + THRESH_CIRC is
 * culled by the per-cell rad test anyway, but the bbox saves the
 * per-cell sqrt for the vast empty space around the gear).
 */
static void clipped_gear_bbox(const Gear *g, int cols, int rows,
                              int *r_lo, int *r_hi, int *c_lo, int *c_hi)
{
    *r_lo = px_row(g->cy - GEAR_R_OUTER) - 1;  if (*r_lo < 0)     *r_lo = 0;
    *r_hi = px_row(g->cy + GEAR_R_OUTER) + 1;  if (*r_hi >= rows) *r_hi = rows - 1;
    *c_lo = px_col(g->cx - GEAR_R_OUTER) - 1;  if (*c_lo < 0)     *c_lo = 0;
    *c_hi = px_col(g->cx + GEAR_R_OUTER) + 1;  if (*c_hi >= cols) *c_hi = cols - 1;
}

/*
 * Convert cell (r, c) into polar coordinates relative to the gear
 * centre.  Sample point is the cell's PIXEL CENTRE for sub-cell
 * accuracy.
 *   rad   = √(dx² + dy²)                        (radius in pixels)
 *   ang_g = atan2(dy, dx)                       (world-frame angle)
 *   ang_l = ang_g − gear.angle                  (gear's rotating frame)
 *
 * ang_g picks the line glyph (geometry is rotation-invariant in the
 * viewer's frame); ang_l drives the tooth/spoke tests (geometry is
 * fixed in the gear's frame).
 */
static void cell_to_gear_polar(int r, int c, const Gear *g,
                               float *rad, float *ang_g, float *ang_l)
{
    float dx = c * CELL_W + CELL_W * 0.5f - g->cx;
    float dy = r * CELL_H + CELL_H * 0.5f - g->cy;
    *rad   = sqrtf(dx * dx + dy * dy);
    *ang_g = atan2f(dy, dx);
    *ang_l = *ang_g - g->angle;
}

/*
 * tooth_phase — fractional position within the current tooth period
 * (wedge of width TAU/N_TEETH).  phase ∈ [0, 1):
 *   [0, TOOTH_DUTY)         → tooth zone
 *   [TOOTH_DUTY, 1.0)       → gap zone
 */
static float tooth_phase(float ang_l)
{
    float phase = fmodf(ang_l * N_TEETH / TAU, 1.0f);
    if (phase < 0.0f) phase += 1.0f;
    return phase;
}

/*
 * Arc-length (in pixels) from the sample point to the nearest tooth-
 * side boundary at this radius.  Tooth sides sit at phase=0 and
 * phase=TOOTH_DUTY; take whichever is nearer (wrap-around at phase>0.5
 * for the 0-side), then convert phase distance → angle → arc length.
 */
static float arclen_to_tooth_side(float phase, float rad)
{
    float d0 = phase;                          /* dist to phase = 0    */
    float dT = fabsf(phase - TOOTH_DUTY);      /* dist to phase = duty */
    if (d0 > 0.5f) d0 = 1.0f - d0;             /* wrap-around          */
    float nearest = (d0 < dT) ? d0 : dT;       /* whichever is closer  */
    return nearest * (TAU / N_TEETH) * rad;    /* phase → arc length   */
}

/*
 * Arc-length (in pixels) to nearest spoke axis.  Spokes sit at the
 * wedge midpoints (one per tooth period).  modulo ang_l by the period
 * → angular distance to spoke; arc-length = ang_dist · rad.
 */
static float arclen_to_spoke(float ang_l, float rad)
{
    float period = TAU / N_TEETH;
    float m      = fmodf(ang_l, period);
    if (m < 0.0f) m += period;
    float ang_d  = (m < period * 0.5f) ? m : period - m;
    return ang_d * rad;
}

/* ─────────────────────────────────────────────────────────────────────── *
 * CellPaint — what to paint at one gear cell.
 *
 * Output of the implicit-field classifier [4]: instead of having
 * classify_gear_cell mutate caller-side ch/at/cp locals (or return
 * those via three out-pointers), it returns one CellPaint value per
 * sampled cell.  Compound-literal init `(CellPaint){...}` lets each
 * priority-chain branch return its result on a single line — the
 * classifier reads as a flat decision table, not a nested if-then.
 *
 * Why a struct (not three separate variables / out-pointers):
 *   The three fields are inseparable — a glyph without its attribute
 *   and colour-pair index is meaningless to ncurses.  Bundling them
 *   keeps "classify" (decide what's there) and "paint" (write the
 *   cell) syntactically separate: classify_gear_cell returns the
 *   bundle, draw_gear forwards it straight to paint_cell.  Classic
 *   separation of decision from effect.
 *
 * Transparency sentinel:
 *   ch == 0 means "nothing to paint here" — the sample point sits in
 *   empty space (outside every gear region).  Zero is safe because no
 *   printable ASCII glyph (0x20..0x7E) has value 0.  draw_gear checks
 *   `if (!p.ch) continue;` instead of carrying a separate bool.
 *
 * Brightness ladder encoded by (at, cp) — used in priority order by
 * classify_gear_cell to give the gear visual depth:
 *
 *   A_BOLD   + CP_GEAR      → brightest  hub ring, spokes, tooth sides,
 *                                         outer tooth arc (the outline)
 *   A_NORMAL + CP_GEAR_DIM  → medium     inner arc in gap zone (fades
 *                                         the body→gap transition)
 *   A_DIM    + CP_GEAR      → dim        tooth interior ':' fill
 *                                         (theme bright colour, low attr)
 *   A_DIM    + CP_GEAR_DIM  → dimmest    body disc '.' fill
 *
 * Outlines paint OVER fills because classify_gear_cell tests outline
 * regions first (see its priority-order doc); the brightness ladder
 * is therefore also a z-order — bright always wins over dim below it.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
    chtype ch;     /* ncurses glyph — printable ASCII (0x20..0x7E), or
                    * 0 as the "transparent" sentinel.  Picked by
                    * line_char() for wireframe edges (one of {-, \, |,
                    * /} based on local angle [6]) or a fixed fill char
                    * (':' for tooth interior, '.' for body disc).     */

    attr_t at;     /* ncurses attribute mask — controls perceptual
                    * intensity at the same underlying colour.  One of:
                    *   A_BOLD    bright wireframe outline
                    *   A_NORMAL  medium tier (inner arc in gap zone)
                    *   A_DIM     interior fills (tooth body, disc)    */

    int    cp;     /* ncurses colour-pair index — selects the fg/bg
                    * mapping registered in color_apply_theme().  One of:
                    *   CP_GEAR      theme's bright gear colour
                    *   CP_GEAR_DIM  theme's muted gear colour
                    * Theme [7] swaps the underlying RGB; this index
                    * never changes between themes — only what it
                    * resolves to does.                                 */
} CellPaint;

/*
 * classify_gear_cell — the implicit-field test [4].
 *
 * Given polar coordinates of one sample point, decide which gear
 * region (if any) it belongs to and return the glyph/attr/color pair
 * to paint there.
 *
 * Tests run in PRIORITY ORDER — FIRST hit wins:
 *
 *   bright wireframe outline (in this exact order):
 *     1. hub ring          band of width 2·THRESH_CIRC around R_HUB
 *     2. spokes            radial spokes, hub → inner
 *     3. tooth sides       radial cuts at the tooth/gap boundaries
 *     4. inner arc (gaps)  R_INNER, only where phase ∈ gap zone
 *     5. outer arc (teeth) R_OUTER, only where phase ∈ tooth zone
 *
 *   dim interior fill (only when no outline matched):
 *     6. tooth interior    ':' between R_INNER and R_OUTER in teeth
 *     7. body disc         '.' between hub ring and inner ring
 *
 * Reordering breaks the gear — fills would paint over outlines, or
 * the tooth-side cuts would land inside the tooth-interior fill.
 */
static CellPaint classify_gear_cell(float rad, float ang_g, float ang_l)
{
    float phase     = tooth_phase(ang_l);
    bool  in_tooth  = (phase < TOOTH_DUTY);
    float arc_side  = arclen_to_tooth_side(phase, rad);
    float arc_spoke = arclen_to_spoke(ang_l, rad);

    /* ── bright wireframe outline ─────────────────────────────────── */
    if (fabsf(rad - GEAR_R_HUB) < THRESH_CIRC)                     /* hub ring */
        return (CellPaint){ line_char(ang_g + TAU * 0.25f), A_BOLD, CP_GEAR };

    if (rad > GEAR_R_HUB && rad < GEAR_R_INNER                     /* spoke */
        && arc_spoke < THRESH_SPOKE)
        return (CellPaint){ line_char(ang_g), A_BOLD, CP_GEAR };

    if (rad > GEAR_R_INNER - THRESH_SIDE * 0.5f                    /* tooth side */
        && rad < GEAR_R_OUTER && arc_side < THRESH_SIDE)
        return (CellPaint){ line_char(ang_g), A_BOLD, CP_GEAR };

    if (!in_tooth && fabsf(rad - GEAR_R_INNER) < THRESH_CIRC)      /* inner arc */
        return (CellPaint){ line_char(ang_g + TAU * 0.25f), A_NORMAL, CP_GEAR_DIM };

    if (in_tooth && fabsf(rad - GEAR_R_OUTER) < THRESH_CIRC)       /* outer arc */
        return (CellPaint){ line_char(ang_g + TAU * 0.25f), A_BOLD, CP_GEAR };

    /* ── dim interior fill (only reached when no outline matched) ── */
    if (in_tooth && rad > GEAR_R_INNER && rad < GEAR_R_OUTER)      /* tooth fill */
        return (CellPaint){ ':', A_DIM, CP_GEAR };

    if (rad > GEAR_R_HUB    + THRESH_CIRC * 0.5f                   /* body disc */
        && rad < GEAR_R_INNER - THRESH_CIRC * 0.5f)
        return (CellPaint){ '.', A_DIM, CP_GEAR_DIM };

    return (CellPaint){ 0, A_NORMAL, CP_GEAR };                    /* transparent */
}

/*
 * draw_gear — implicit-field rasteriser [4].
 *
 * Pseudocode:
 *   bbox = clipped_gear_bbox(g, viewport)
 *   for each cell in bbox:
 *     (rad, ang_g, ang_l) = cell_to_gear_polar(cell, g)
 *     if rad outside the gear's outermost band: skip
 *     paint = classify_gear_cell(rad, ang_g, ang_l)
 *     if paint not transparent: paint_cell(paint)
 */
static void draw_gear(WINDOW *win, const Gear *g, int cols, int rows)
{
    int r_lo, r_hi, c_lo, c_hi;
    clipped_gear_bbox(g, cols, rows, &r_lo, &r_hi, &c_lo, &c_hi);

    for (int r = r_lo; r <= r_hi; r++) {
        for (int c = c_lo; c <= c_hi; c++) {
            float rad, ang_g, ang_l;
            cell_to_gear_polar(r, c, g, &rad, &ang_g, &ang_l);
            if (rad > GEAR_R_OUTER + THRESH_CIRC) continue;

            CellPaint p = classify_gear_cell(rad, ang_g, ang_l);
            if (!p.ch) continue;
            paint_cell(win, r, c, p.ch, p.cp, p.at, cols, rows);
        }
    }
}

/*
 * direction_glyph — line glyph picked from the velocity's angle.
 *   ~horizontal   → '-'
 *   ~vertical     → '|'
 *   diagonal NE/SW→ '/'
 *   diagonal NW/SE→ '\'
 * Eight 45°-sectors covering the full circle.
 */
static chtype direction_glyph(float vx, float vy)
{
    float a = atan2f(vy, vx);
    if (a < 0.0f) a += TAU;
    if (a < TAU/16.0f || a >= TAU*15.0f/16.0f) return '-';
    if (a < TAU* 3.0f/16.0f) return '\\';
    if (a < TAU* 5.0f/16.0f) return '|';
    if (a < TAU* 7.0f/16.0f) return '/';
    if (a < TAU* 9.0f/16.0f) return '-';
    if (a < TAU*11.0f/16.0f) return '\\';
    if (a < TAU*13.0f/16.0f) return '|';
    return '/';
}

/*
 * draw_spark_streak — fresh spark rendered as a directional streak.
 *
 * One bright "head" at the spark's current position painted with a
 * line-shaped glyph picked from the velocity angle, followed by
 * `n_tail` dim cells stepping BACKWARD along velocity.  Tail step is
 * 1 cell wide / 1 cell tall (anisotropic — terminal cells are
 * CELL_W × CELL_H px), so each tail cell sits roughly one cell behind
 * its predecessor regardless of direction.
 *
 * The cascading head→tail brightness reads as a comet (bright leading
 * tip, fading trail) — much more "spray of streaks" than scattered
 * dots.
 */
static void draw_spark_streak(WINDOW *win, const Spark *s, int st,
                              int cp, attr_t head_attr, int n_tail,
                              int cols, int rows)
{
    chtype glyph = direction_glyph(s->vx, s->vy);
    paint_cell(win, px_row(s->py), px_col(s->px), glyph, cp, head_attr,
               cols, rows);

    /* Tail — only meaningful if the spark is actually moving. */
    float speed = sqrtf(s->vx * s->vx + s->vy * s->vy);
    if (speed < 1.0f) return;
    float ux = s->vx / speed;
    float uy = s->vy / speed;
    for (int k = 1; k <= n_tail; k++) {
        float tx = s->px - ux * CELL_W * (float)k;
        float ty = s->py - uy * CELL_H * (float)k;
        paint_cell(win, px_row(ty), px_col(tx), glyph, cp, A_DIM,
                   cols, rows);
    }
    (void)st;   /* kept for future per-stage tail tuning */
}

/* draw_spark_dot — cooled spark rendered as the theme's dot glyph. */
static void draw_spark_dot(WINDOW *win, const Spark *s, const Theme *th,
                           int st, int cols, int rows)
{
    chtype ch = (chtype)(unsigned char)th->spark_ch[st];
    attr_t at = ATTR_DEC[th->spark_at[st]];
    int    cp = CP_S0 + st;
    paint_cell(win, px_row(s->py), px_col(s->px), ch, cp, at,
               cols, rows);
}

/*
 * Draw sparks.  Fresh sparks (stages 0..2) render as directional
 * streaks with fading tails; cooler sparks (stages 3..6) fall back
 * to the theme's per-stage dot glyph so they fade into the background.
 *
 *   stage 0  head + 2 dim tails    (comet — longest streak)
 *   stage 1  head + 1 dim tail
 *   stage 2  head only (still directional)
 *   stage 3+ theme dot glyph (current behaviour)
 */
static void draw_sparks(WINDOW *win, const Gear *g,
                        int cols, int rows, int theme_idx)
{
    const Theme *th = &THEMES[theme_idx];
    static const int TAIL_PER_STAGE[3] = { 2, 1, 0 };

    for (int i = 0; i < MAX_SPARKS; i++) {
        const Spark *s = &g->sparks[i];
        if (s->life <= 0.0f) continue;

        int st = spark_stage(s->life);
        if (st <= 2) {
            int    cp        = CP_S0 + st;
            attr_t head_attr = ATTR_DEC[th->spark_at[st]];
            draw_spark_streak(win, s, st, cp, head_attr,
                              TAIL_PER_STAGE[st], cols, rows);
        } else {
            draw_spark_dot(win, s, th, st, cols, rows);
        }
    }
}

/* ===================================================================== */
/* §8  screen / scene                                                     */
/* ===================================================================== */

/* ─────────────────────────────────────────────────────────────────────── *
 * Scene — top-level state spanning physics ticks, draw calls, and the
 * main loop.  Wraps the gear simulation together with the per-frame
 * control knobs the user can toggle.
 *
 * The struct splits into two clearly-labelled groups:
 *
 *   Simulation parameters  — consumed by scene_tick / gear_tick.
 *     Anything that affects the PHYSICS lives here.  Mutated by
 *     physics-affecting keys: r (reset), space (pause), +/- and
 *     1..5 (rot_speed), ] / [ (spark_density), and SIGWINCH (which
 *     rewrites max_px / max_py from the new terminal size).
 *
 *   Rendering parameters   — consumed by draw_sparks / draw_gear and
 *     by color_apply_theme.  Toggling these while paused must leave
 *     gear angle and every spark byte-identical; only colours/glyphs
 *     change.  Mutated by purely cosmetic keys: t / T (theme).
 *
 * Locality rationale (the contract that matters, not the byte layout):
 *   The split is for the READER, not the CPU.  max_px / max_py LOOK
 *   like screen geometry but they live with the simulation because
 *   gear_tick KILLS sparks that drift out of [0, max_px] × [0, max_py]
 *   — changing them silently changes which particles survive.  A new
 *   knob landing in the rendering group when it actually steers
 *   gear_tick would couple display to physics — exactly the bug this
 *   separation is here to prevent.  When adding a field, ask: does
 *   this change what gear_tick produces?  If yes → simulation; if
 *   purely visual → rendering.
 *
 * What stays OUTSIDE this struct (intentionally):
 *   App.running / App.need_resize     volatile sig_atomic_t flags
 *                                     written from signal handlers;
 *                                     must stay on the file-scope
 *                                     App for async-signal safety.
 *   App.screen (cols, rows)           terminal geometry tracked by
 *                                     the main loop; Scene stays
 *                                     window-agnostic so resize is
 *                                     the main loop's concern.
 *
 * Single instance: lives inside file-scope `g_app` (§9).  Gear.sparks[]
 * alone is ~30 KB (1500 × 20 B), so the App / Scene sits in BSS rather
 * than being passed by value anywhere.
 * ─────────────────────────────────────────────────────────────────────── */
typedef struct {
    /* ── Simulation parameters (read & written by gear_tick) ──────── */
    Gear  gear;            /* physics state — see Gear docstring §6   */
    bool  paused;          /* space toggles; scene_tick is a no-op
                            * when true — sparks freeze in place      */
    float max_px, max_py;  /* simulation arena bounds in pixels; also
                            * the framebuffer extent.  Sparks outside
                            * this rect die in gear_tick — so this is
                            * simulation state, not rendering state.  */

    /* ── Rendering parameters (no physics side-effects) ───────────── */
    int   theme;           /* index into THEMES[] in §3; t / T cycles.
                            * Purely cosmetic — physics is identical
                            * across all 10 themes                    */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->max_px = (float)(cols * CELL_W);
    s->max_py = (float)(rows * CELL_H);
    s->paused = false;
    /* keep current theme on reset */
    gear_init(&s->gear, s->max_px, s->max_py);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    gear_tick(&s->gear, dt, s->max_px, s->max_py);
}

/* Census of active sparks — used in the HUD readout. */
static int count_live_sparks(const Gear *g)
{
    int n = 0;
    for (int i = 0; i < MAX_SPARKS; i++)
        if (g->sparks[i].life > 0.0f) n++;
    return n;
}

/* Layered composite: bottom layer first (sparks), gear painted on top
 * so its bright wireframe is never obscured by spray. */
static void compose_scene_layers(WINDOW *win, const Scene *s,
                                 int cols, int rows)
{
    draw_sparks(win, &s->gear, cols, rows, s->theme);
    draw_gear  (win, &s->gear, cols, rows);
}

/* Top-row HUD: live simulation metrics (fps, ω, density, spark count,
 * theme).  Per the project HUD standard — PAIR_HUD + A_BOLD. */
static void hud_draw_top(WINDOW *win, const Scene *s, double fps, int live)
{
    wattron(win, COLOR_PAIR(CP_HUD) | A_BOLD);
    mvwprintw(win, 0, 0,
        " %.0f fps  speed:%.1f rad/s  density:%.1fx  sparks:%d"
        "  theme:[%d] %s ",
        fps, s->gear.rot_speed, s->gear.spark_density,
        live, s->theme, THEMES[s->theme].name);
    wattroff(win, COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Bottom-row HUD: static key legend (lists every interactive key). */
static void hud_draw_bottom(WINDOW *win, int rows)
{
    wattron(win, COLOR_PAIR(CP_HUD) | A_BOLD);
    mvwprintw(win, rows - 1, 0,
        " q:quit  spc:pause  r:reset  +/-:speed  ]/[:density"
        "  t/T:theme  1-5:speed presets ");
    wattroff(win, COLOR_PAIR(CP_HUD) | A_BOLD);
}

/*
 * scene_draw — one full frame.
 *
 * Pseudocode:
 *   compose_scene_layers   // sparks (bottom) + gear (top)
 *   hud_draw_top           // live metrics
 *   hud_draw_bottom        // key legend
 */
static void scene_draw(const Scene *s, WINDOW *win,
                       int cols, int rows, double fps)
{
    compose_scene_layers(win, s, cols, rows);
    hud_draw_top        (win, s, fps, count_live_sparks(&s->gear));
    hud_draw_bottom     (win, rows);
}

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *sc, int theme)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(theme);
    getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) { (void)sc; endwin(); }

static void screen_resize(Screen *sc)
{
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_render(Screen *sc, const Scene *s, double fps)
{
    erase();
    scene_draw(s, stdscr, sc->cols, sc->rows, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}
static void cleanup(void) { endwin(); }

static bool app_key(App *app, int ch)
{
    Scene *sc = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': sc->paused = !sc->paused; break;
    case 'r': case 'R':
        scene_init(sc, app->screen.cols, app->screen.rows);
        break;
    case '+': case '=':
        sc->gear.rot_speed += GEAR_ROT_STEP;
        if (sc->gear.rot_speed > GEAR_ROT_MAX) sc->gear.rot_speed = GEAR_ROT_MAX;
        break;
    case '-':
        sc->gear.rot_speed -= GEAR_ROT_STEP;
        if (sc->gear.rot_speed < 0.2f) sc->gear.rot_speed = 0.2f;
        break;
    case ']':
        sc->gear.spark_density += DENSITY_STEP;
        if (sc->gear.spark_density > DENSITY_MAX) sc->gear.spark_density = DENSITY_MAX;
        break;
    case '[':
        sc->gear.spark_density -= DENSITY_STEP;
        if (sc->gear.spark_density < 0.2f) sc->gear.spark_density = 0.2f;
        break;
    case 't': case 'T': {
        int dir = (ch == 't') ? 1 : -1;
        sc->theme = (sc->theme + dir + N_THEMES) % N_THEMES;
        color_apply_theme(sc->theme);
        break;
    }
    case '1': sc->gear.rot_speed =  0.4f; break;
    case '2': sc->gear.rot_speed =  2.0f; break;
    case '3': sc->gear.rot_speed =  6.0f; break;
    case '4': sc->gear.rot_speed = 12.0f; break;
    case '5': sc->gear.rot_speed = 20.0f; break;
    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    App *app     = &g_app;
    app->running = 1;
    app->scene.theme = 0;   /* start with FIRE */

    screen_init(&app->screen, app->scene.theme);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t prev = clock_ns(), fps_acc = 0;
    int     fps_count = 0;
    double  fps_disp  = 0.0;

    while (app->running) {
        if (app->need_resize) {
            screen_resize(&app->screen);
            app->scene.max_px    = (float)(app->screen.cols * CELL_W);
            app->scene.max_py    = (float)(app->screen.rows * CELL_H);
            app->scene.gear.cx   = app->scene.max_px * 0.5f;
            app->scene.gear.cy   = app->scene.max_py * 0.5f;
            app->need_resize     = 0;
            prev = clock_ns();
        }

        int64_t now   = clock_ns();
        int64_t dt_ns = now - prev;
        prev = now;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;
        float dt = (float)dt_ns / (float)NS_PER_SEC;

        scene_tick(&app->scene, dt);

        fps_count++;
        fps_acc += dt_ns;
        if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_disp  = (double)fps_count / ((double)fps_acc / NS_PER_SEC);
            fps_count = 0; fps_acc = 0;
        }

        screen_render(&app->screen, &app->scene, fps_disp);

        int64_t elapsed = clock_ns() - now;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        int key = getch();
        if (key != ERR && !app_key(app, key))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
