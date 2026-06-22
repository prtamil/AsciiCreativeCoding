/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sensitive_dependence.c — the "butterfly effect" you can watch.
 * Two (or three) Lorenz trajectories start almost identical, a hair
 * apart in their starting point. They track each other for a while,
 * then drift apart faster and faster until they're on opposite wings
 * of the attractor. Ten presets show this from different viewpoints
 * and at different starting gaps.
 *
 * Original idea: Lorenz, "Deterministic Nonperiodic Flow" (1963).
 * Sister files: ./strange_attractor.c (same Lorenz, more variations),
 *               ../fractals/lyapunov.c (the divergence rate as a map).
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

/* §1  config */

enum {
    SIM_FPS_MIN      =  10, SIM_FPS_DEFAULT = 60, SIM_FPS_MAX = 240, SIM_FPS_STEP = 10,
    HUD_COLS         =  80, FPS_UPDATE_MS = 500,
    PAIR_HUD         =   1, PAIR_HINT = 2,
    PAIR_A_BASE      =   3,    /* trail A: 4 shades, faded to bright */
    PAIR_B_BASE      =   7,    /* trail B: 4 shades, faded to bright */
    PAIR_LIVE_A      =  11,
    PAIR_LIVE_B      =  12,
    PAIR_PLOT        =  13,    /* the divergence curve */
    PAIR_AXIS        =  14,
    PAIR_LIVE_C      =  15,    /* third trajectory (TRIO presets) */
};

#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1
#define NS_PER_SEC               1000000000LL
#define NS_PER_MS                   1000000LL
#define TICK_NS(f)               (NS_PER_SEC / (f))
#define RENDER_FPS_TARGET        60
#define RENDER_FRAME_BUDGET_NS   (NS_PER_SEC / RENDER_FPS_TARGET)
#define SIM_MAX_FRAME_DT_MS      100

#define LORENZ_DT                 0.01f
#define INT_STEPS_PER_TICK         5
#define TRAIL_MAX               2500
#define TRAIL_BAND_COUNT           4
#define PLOT_MAX                2000
#define TRAJ_MAX                   3    /* up to 3 trajectories (TRIO presets) */

/* The Lorenz settings that make it chaotic — the famous ones. */
#define LZ_SIGMA                 10.0f
#define LZ_RHO                   28.0f
#define LZ_BETA                  (8.0f / 3.0f)

/* Where every run starts. The tiny per-preset gap is added on top of this. */
#define IC_X                      1.0f
#define IC_Y                      1.0f
#define IC_Z                      1.0f

/* How far the camera can see along each axis. We pick a separate
 * window per view so the attractor fills the screen no matter which
 * two axes we're showing. Wide enough to hold the whole shape. */
#define X_AXIS_MIN              -25.0f
#define X_AXIS_MAX               25.0f
#define Y_AXIS_MIN              -30.0f
#define Y_AXIS_MAX               30.0f
#define Z_AXIS_MIN                0.0f
#define Z_AXIS_MAX               50.0f

/* The divergence plot's up/down range, in log units. LOG_MAX is the
 * log of how big the attractor is, so the curve flattens once the two
 * paths are as far apart as they can get. The plot's bottom edge is
 * worked out per preset (see log_floor_for_eps) so the curve always
 * starts near the bottom whatever the starting gap is. */
#define LOG_PLOT_FLOOR_PAD        2.0f   /* a little headroom below log(gap) */
#define LOG_MAX                   4.5f   /* log of how wide the attractor is */

/* The known divergence rate for this Lorenz setup (Sparrow 1982).
 * Shown in the HUD so you can eyeball it against the slope of the
 * LOG_ONLY curve. */
#define LORENZ_LAMBDA              0.906f

/* Sizes for the on-screen dividers and the smallest plot worth drawing. */
#define PAINT_GUTTER_WIDTH           1     /* SPLIT: gap between the two halves */
#define PAINT_DIVIDER_ROWS           1     /* PLOT_BOTTOM: line between panels  */
#define LOG_PLOT_BAND_NUMERATOR      2     /* plot gets about 2/5 of the height */
#define LOG_PLOT_BAND_DENOMINATOR    5
#define PLOT_MIN_WIDTH_CELLS         4     /* smaller than this and it's a blur */
#define PLOT_MIN_HEIGHT_CELLS        3

/* Fixed column widths for the HUD's second row, so the three labels
 * line up the same across every preset and theme. */
#define HUD_PARAM_PRESET_WIDTH      19
#define HUD_PARAM_THEME_WIDTH       17

/*
 * Projection — which two of the three axes we draw.
 *
 * The attractor lives in 3-D but the screen is flat, so we pick two
 * axes and drop the third. Each choice shows a different face of the
 * same motion — handy reminder that the famous butterfly shape is
 * just one camera angle.
 *
 *   PROJ_XZ : the classic butterfly outline. The two wings come from
 *             the path looping up on one side, flipping, looping up
 *             on the other.
 *   PROJ_XY : looking down from above.
 *   PROJ_YZ : looking from the side; shows how the two wings differ.
 */
typedef enum { PROJ_XZ = 0, PROJ_XY, PROJ_YZ } Projection;

/*
 * Layout — how a preset carves up the screen.
 *
 * Some presets want the whole screen for the attractor; some want the
 * two paths side by side so they don't overlap; some stack the
 * attractor on top and the divergence plot underneath so they don't
 * fight for the same cells; one hands the whole screen to the plot.
 *
 *   LAYOUT_FULL        : paths fill the whole drawing area.
 *   LAYOUT_SPLIT       : screen cut in half, one path per side, with a
 *                        thin divider down the middle.
 *   LAYOUT_PLOT_BOTTOM : attractor on top (~60%), divergence plot on
 *                        the bottom (~40%), a line between them.
 *   LAYOUT_PLOT_ONLY   : the divergence plot takes the whole screen.
 */
typedef enum {
    LAYOUT_FULL = 0,
    LAYOUT_SPLIT,
    LAYOUT_PLOT_BOTTOM,
    LAYOUT_PLOT_ONLY,
} Layout;

/*
 * Preset — names for the rows of the presets[] table below.
 *
 * Ordered as a little tour: the classic view first, then the other
 * camera angles, then the starting-gap ladder (big gap diverges fast,
 * tiny gap stays together for ages), then three paths at once, then
 * the layouts that show the divergence plot itself. PRESET_CLASSIC is
 * what you see on startup.
 */
typedef enum {
    PRESET_CLASSIC = 0,   /* butterfly view, 2 paths, gap 1e-6      */
    PRESET_TOP_DOWN,      /* from above, 2 paths                    */
    PRESET_SIDE,          /* from the side, 2 paths                 */
    PRESET_EPS_BIG,       /* big gap (1e-3) — splits in seconds     */
    PRESET_EPS_TINY,      /* tiny gap (1e-12) — stays together long */
    PRESET_TRIO,          /* 3 paths fanning out from the middle    */
    PRESET_SPLIT,         /* the two paths side by side             */
    PRESET_DELTA,         /* attractor on top, divergence plot below*/
    PRESET_LOG_ONLY,      /* divergence plot fills the screen       */
    PRESET_TRIO_LOG,      /* three paths + divergence plot          */
    N_PRESETS,
} Preset;

/*
 * SDPreset — one row of the demo menu.
 *
 * Each row says everything about one entry: its name, which two axes
 * to draw, how to lay out the screen, how many paths to run, and how
 * far apart the first two start. Stepping through this table (n/p) is
 * the only way to change what's on screen.
 *
 *   name    : the label shown in the HUD (padded so columns line up).
 *   proj    : which two axes to draw (Projection).
 *   layout  : how to split the screen (Layout).
 *   n_traj  : how many paths to run — 2 or 3 (3 is the hard cap).
 *   eps     : how far apart the first two paths start, measured along
 *             the x axis. Ranges from 1e-3 down to 1e-12 so you can
 *             feel how a smaller gap buys more time before they split.
 */
typedef struct {
    const char *name;
    Projection  proj;
    Layout      layout;
    int         n_traj;
    float       eps;
} SDPreset;

static const SDPreset presets[N_PRESETS] = {
    /*  name        proj      layout              n  gap     */
    { "CLASSIC ", PROJ_XZ, LAYOUT_FULL,        2, 1e-6f  },
    { "TOP_DOWN", PROJ_XY, LAYOUT_FULL,        2, 1e-6f  },
    { "SIDE    ", PROJ_YZ, LAYOUT_FULL,        2, 1e-6f  },
    { "EPS_BIG ", PROJ_XZ, LAYOUT_FULL,        2, 1e-3f  },
    { "EPS_TINY", PROJ_XZ, LAYOUT_FULL,        2, 1e-12f },
    { "TRIO    ", PROJ_XZ, LAYOUT_FULL,        3, 1e-6f  },
    { "SPLIT   ", PROJ_XZ, LAYOUT_SPLIT,       2, 1e-6f  },
    { "DELTA   ", PROJ_XZ, LAYOUT_PLOT_BOTTOM, 2, 1e-6f  },
    { "LOG_ONLY", PROJ_XZ, LAYOUT_PLOT_ONLY,   2, 1e-6f  },
    { "TRIO_LOG", PROJ_XZ, LAYOUT_PLOT_BOTTOM, 3, 1e-6f  },
};

/*
 * Theme — one named colour set for the whole demo.
 *
 * All the colours the drawing code needs, packed into one row, so the
 * theme key (t/T) just steps through a table. This demo is fussier
 * than most because it draws two paths at once: each theme gives path
 * A and path B colours from the same family (so OCEAN really is all
 * blues) but different enough that you can still tell them apart.
 * Every colour sits in the bright half of the palette so the dots
 * stay readable on a black background (see CLAUDE.md brightness rule).
 *
 *   name    : the label shown in the HUD.
 *   a[]     : four colours for path A, oldest/faintest to newest/brightest.
 *   b[]     : same idea for path B; same family as a[] but distinct.
 *   live_a  : colour of path A's current-position '@' marker.
 *   live_b  : colour of path B's '@' marker.
 *   live_c  : one flat colour for the third path (TRIO presets). No
 *             fade — keeps the three-way overlay from turning to mush.
 *   plot    : colour of the divergence curve.
 *   axis    : faint colour for dividers and the plot's frame.
 */
typedef struct {
    const char *name;
    short       a[TRAIL_BAND_COUNT];
    short       b[TRAIL_BAND_COUNT];
    short       live_a, live_b, live_c, plot, axis;
} Theme;

/* Each theme picks two readable shades from one colour family so the
 * two paths stay tellable apart while still matching the theme name:
 *
 *   DEFAULT : blue vs warm red→yellow — easiest to tell apart
 *   MATRIX  : deep green vs bright lime
 *   NOVA    : magenta vs bright yellow
 *   MONO    : light gray vs medium gray
 *   OCEAN   : deep cyan vs sky blue
 *   FIRE    : red→orange vs yellow→white
 *   EARTH   : brown vs olive
 *   FOREST  : dark green vs leaf green
 *   DESERT  : sand vs rust
 *   ARCTIC  : ice blue vs near-white cyan
 */
#define N_THEMES 10
static const Theme themes[N_THEMES] = {
    /* name        A: oldest→newest    B: oldest→newest    lv_a lv_b lv_c plot axis */
    { "DEFAULT",   {  75, 117, 153, 195 }, { 196, 202, 208, 220 },  51, 220, 213, 226, 244 },
    { "MATRIX",    {  34,  40,  46,  82 }, { 154, 190, 226, 228 },  46, 226,  87, 220, 244 },
    { "NOVA",      { 165, 171, 207, 219 }, { 208, 214, 220, 227 }, 213, 226,  51, 219, 244 },
    { "MONO",      { 244, 247, 250, 253 }, { 240, 242, 245, 248 }, 255, 244, 226, 226, 240 },
    { "OCEAN",     {  31,  38,  44,  51 }, { 117, 159, 195, 231 },  51, 195, 117, 226, 244 },
    { "FIRE",      { 196, 202, 208, 214 }, { 220, 226, 228, 229 }, 196, 226, 208, 226, 244 },
    { "EARTH",     {  94, 130, 137, 143 }, { 100, 142, 178, 215 }, 130, 178, 215, 220, 244 },
    { "FOREST",    {  34,  40,  46,  82 }, { 113, 149, 185, 191 },  46, 154, 220, 226, 244 },
    { "DESERT",    { 137, 179, 215, 222 }, { 130, 166, 208, 214 }, 215, 208, 226, 220, 244 },
    { "ARCTIC",    { 117, 153, 195, 231 }, {  51,  87, 123, 159 }, 231,  51, 220, 226, 244 },
};

/* §2 clock + §3 color */

static int64_t clock_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec; }
static void clock_sleep_ns(int64_t ns) { if (ns <= 0) return;
    struct timespec req = {ns/NS_PER_SEC, ns%NS_PER_SEC}; nanosleep(&req, NULL); }

static inline void theme_install_256(const Theme *t)
{
    for (int i = 0; i < TRAIL_BAND_COUNT; i++) {
        init_pair(PAIR_A_BASE + i, t->a[i], -1);
        init_pair(PAIR_B_BASE + i, t->b[i], -1);
    }
    init_pair(PAIR_LIVE_A, t->live_a, -1);
    init_pair(PAIR_LIVE_B, t->live_b, -1);
    init_pair(PAIR_LIVE_C, t->live_c, -1);
    init_pair(PAIR_PLOT,   t->plot,   -1);
    init_pair(PAIR_AXIS,   t->axis,   -1);
}

/* Fallback for old terminals that only have 8 colours: path A goes
 * cyan, path B red, the third path magenta, plot yellow, axis white. */
static inline void theme_install_8color_fallback(void)
{
    for (int i = 0; i < TRAIL_BAND_COUNT; i++) {
        init_pair(PAIR_A_BASE + i, COLOR_CYAN, -1);
        init_pair(PAIR_B_BASE + i, COLOR_RED,  -1);
    }
    init_pair(PAIR_LIVE_A, COLOR_CYAN,    -1);
    init_pair(PAIR_LIVE_B, COLOR_RED,     -1);
    init_pair(PAIR_LIVE_C, COLOR_MAGENTA, -1);
    init_pair(PAIR_PLOT,   COLOR_YELLOW,  -1);
    init_pair(PAIR_AXIS,   COLOR_WHITE,   -1);
}

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_install_256(&themes[idx]);
    else               theme_install_8color_fallback();
}
static void color_init(void) { start_color(); use_default_colors();
    if (COLORS >= 256) { init_pair(PAIR_HUD, 226, -1); init_pair(PAIR_HINT, 51, -1); }
    else { init_pair(PAIR_HUD, COLOR_YELLOW, -1); init_pair(PAIR_HINT, COLOR_CYAN, -1); }
    theme_apply(0); }

/* §5  Lorenz equations — the system, its state, and the RK4 stepper */

/*
 * Vec3 — a plain triple of floats.
 *
 * The one little 3-number type the whole file passes around. It plays
 * three roles: a point on a path, the rate-of-change at that point,
 * and a flattened 2-D screen sample. Kept as a flat by-value struct so
 * the RK4 math can add and scale copies cheaply without any malloc.
 *
 *   x, y, z : in Lorenz terms, roughly: how fast the fluid is rolling,
 *             and two measures of the temperature pattern.
 */
typedef struct { float x, y, z; } Vec3;

/* A Lorenz state is just a Vec3 — the new name only says "this is a
 * point on the path right now," so the math below reads as physics. */
typedef Vec3 LorenzState;

/*
 * LorenzSystem — the three knobs that define the equations.
 *
 * Kept separate from the moving state so the math function below can
 * be a clean "given where you are, here's which way you're heading."
 * Every path in this demo uses the exact same knobs — that's the whole
 * point: same physics, only the starting point differs. We keep this
 * as its own struct (instead of bare constants) to match the other
 * chaos demos in the project.
 *
 *   sigma : how fast heat moves through the fluid (the famous 10).
 *   rho   : how hard the system is driven (28 — well into chaos).
 *   beta  : a shape factor for the convection cells (8/3).
 */
typedef struct { float sigma, rho, beta; } LorenzSystem;

/*
 * Lorenz — one path: its knobs plus where it is right now.
 *
 * Everything needed to push a single path forward one step, bundled so
 * a step is one tidy call instead of juggling loose floats. Scene keeps
 * an array of these, one per path.
 *
 *   system : the three knobs — never change while stepping; the same
 *            for every path here.
 *   state  : the current point — moves every step. This is the only
 *            thing that starts off different between paths (by the gap).
 */
typedef struct {
    LorenzSystem system;
    LorenzState  state;
} Lorenz;

/* The standard chaotic knob values (10, 28, 8/3). Trying other values
 * is what ./strange_attractor.c is for. */
static inline LorenzSystem lorenz_system_canonical(void)
{
    return (LorenzSystem){ LZ_SIGMA, LZ_RHO, LZ_BETA };
}

/* Given a point, which way is the path heading from here? These are
 * the three Lorenz equations straight from the 1963 paper; each line
 * is one of them. */
static inline LorenzState lorenz_deriv(const LorenzState *s,
                                       const LorenzSystem *sys)
{
    LorenzState dy;
    dy.x = sys->sigma * (s->y - s->x);              /* sigma * (y - x)     */
    dy.y = s->x * (sys->rho - s->z) - s->y;         /* x * (rho - z) - y   */
    dy.z = s->x * s->y - sys->beta * s->z;          /* x*y - beta*z        */
    return dy;
}

/* Take a step h in direction k from point a, hand back the new point.
 * The little building block the step below uses to peek ahead. */
static inline LorenzState state_add(const LorenzState *a,
                                    float h, const LorenzState *k)
{
    LorenzState r;
    r.x = a->x + h * k->x;
    r.y = a->y + h * k->y;
    r.z = a->z + h * k->z;
    return r;
}

/* Mixing weights for the four direction samples below. */
#define RK4_BUTCHER_WEIGHT_SUM   6.0f
#define RK4_MIDPOINT_FRACTION    0.5f

/* Blend the four sampled directions into one, weighting the two
 * middle ones twice as heavily: (k1 + 2k2 + 2k3 + k4) / 6. */
static inline LorenzState rk4_butcher_weighted_average(
    const LorenzState *k1, const LorenzState *k2,
    const LorenzState *k3, const LorenzState *k4)
{
    LorenzState avg;
    avg.x = (k1->x + 2.0f*k2->x + 2.0f*k3->x + k4->x) / RK4_BUTCHER_WEIGHT_SUM;
    avg.y = (k1->y + 2.0f*k2->y + 2.0f*k3->y + k4->y) / RK4_BUTCHER_WEIGHT_SUM;
    avg.z = (k1->z + 2.0f*k2->z + 2.0f*k3->z + k4->z) / RK4_BUTCHER_WEIGHT_SUM;
    return avg;
}

/* Move the path forward by one time step, accurately. The trick
 * (RK4): instead of trusting the direction at the start, sample it
 * four times — at the start, twice around the midpoint, once at the
 * far end — then take a weighted average of those four directions and
 * step that way. Far steadier than a single naive step. */
static void lorenz_rk4_step(Lorenz *L, float dt)
{
    const LorenzSystem *sys = &L->system;
    float half_dt = RK4_MIDPOINT_FRACTION * dt;

    LorenzState slope_start    = lorenz_deriv(&L->state, sys);
    LorenzState midpoint_1     = state_add(&L->state, half_dt, &slope_start);
    LorenzState slope_mid_1    = lorenz_deriv(&midpoint_1, sys);
    LorenzState midpoint_2     = state_add(&L->state, half_dt, &slope_mid_1);
    LorenzState slope_mid_2    = lorenz_deriv(&midpoint_2, sys);
    LorenzState endpoint       = state_add(&L->state, dt, &slope_mid_2);
    LorenzState slope_end      = lorenz_deriv(&endpoint, sys);

    LorenzState effective_slope = rk4_butcher_weighted_average(
        &slope_start, &slope_mid_1, &slope_mid_2, &slope_end);
    L->state = state_add(&L->state, dt, &effective_slope);
}

/* §6  trail + plot — the fading streak behind each path, and the
 *     running record of how far apart two paths have drifted */

/*
 * Trail — the last few thousand screen positions of one path.
 *
 * A round buffer (oldest gets overwritten) so we can draw a fading
 * streak without caring how fast the simulation is running. The
 * stepper drops one position in each step; the drawing code reads them
 * back. No malloc — fixed size. Each path gets its own so two or three
 * streaks don't get tangled together.
 *
 *   x[], y[] : already-flattened screen coords, not the raw 3-D point,
 *              so drawing doesn't have to redo the flattening.
 *   head     : where the newest position sits.
 *   count    : how many are valid; tops out at TRAIL_MAX.
 */
typedef struct {
    float x[TRAIL_MAX], y[TRAIL_MAX];
    int   head, count;
} Trail;

static void trail_reset(Trail *t) { t->head = 0; t->count = 0; }
static void trail_push(Trail *t, float px, float py)
{
    t->head = (t->head + 1) % TRAIL_MAX;
    t->x[t->head] = px;
    t->y[t->head] = py;
    if (t->count < TRAIL_MAX) t->count++;
}

/*
 * LogPlot — the history of how far apart two paths have drifted,
 * stored on a log scale, for the presets that draw the divergence
 * plot.
 *
 * We store the log of the gap rather than the gap itself, computed
 * once when each sample arrives. Two reasons: it's cheaper to do the
 * log here than for every screen cell at draw time, and on a log scale
 * the steady doubling-apart phase shows up as a nice straight slanted
 * line — and that slope is exactly the divergence rate.
 *
 *   v[]   : log of the gap at each moment. The drawing code keeps these
 *           from dropping off the bottom of the chart.
 *   head  : where the newest value sits.
 *   count : how many are valid; tops out at PLOT_MAX.
 */
typedef struct {
    float v[PLOT_MAX];
    int   head, count;
} LogPlot;

static void plot_reset(LogPlot *p) { p->head = 0; p->count = 0; }
static void plot_push(LogPlot *p, float v)
{
    p->head = (p->head + 1) % PLOT_MAX;
    p->v[p->head] = v;
    if (p->count < PLOT_MAX) p->count++;
}

/* §7  state — small wrappers tracking which preset and theme are on */

/*
 * PresetState — remembers which preset is showing.
 *
 * Just an index, but wrapped in its own type so the next/prev helpers
 * read as intentions and so a theme keystroke can't accidentally poke
 * the preset (the types won't let it).
 *
 *   current : which row of presets[] — always 0..N_PRESETS-1.
 */
typedef struct { int current; } PresetState;

static void preset_state_init      (PresetState *p, int initial) { p->current = initial; }
static void preset_state_cycle_next(PresetState *p)              { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)              { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const SDPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }

/*
 * PaletteState — remembers which colour theme is active.
 *
 * Same idea as PresetState, kept a separate type so the theme keys
 * can't reach into the preset table by mistake. After changing it you
 * call scene_apply_theme to push the new colours into ncurses.
 *
 *   current : which row of themes[] — always 0..N_THEMES-1.
 */
typedef struct { int current; } PaletteState;

static void palette_state_init      (PaletteState *p, int initial) { p->current = initial; }
static void palette_state_cycle_next(PaletteState *p)              { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p)              { p->current = (p->current + N_THEMES - 1) % N_THEMES; }
static const Theme *palette_state_active(const PaletteState *p)    { return &themes[p->current]; }
static void palette_state_apply     (const PaletteState *p)        { theme_apply(p->current); }

/* §8  scene — everything that changes as the simulation runs */

/*
 * Scene — all the moving parts in one place.
 *
 * Everything that changes while running, bundled so the main loop can
 * drive it through a handful of named calls (init, reset, tick, apply
 * theme). Each field is its own little type so the physics, the
 * drawing data, and the menu choices stay clearly separate.
 *
 *   traj[]    : the (up to 3) paths. Same physics, but they start a
 *               hair apart:
 *                 traj[0]: the base starting point
 *                 traj[1]: base nudged a tiny gap along x
 *                 traj[2]: base nudged the same gap the other way (TRIO)
 *   trail[]   : the fading streak behind each path. Cleared on r/n/p.
 *   log_delta : the running record of the gap between paths 0 and 1,
 *               for the presets that draw the divergence plot.
 *   t_sim     : seconds simulated since the last reset (shown in HUD).
 *   preset    : which preset is showing.
 *   palette   : which colour theme is on.
 *   paused    : when true, tick does nothing.
 */
typedef struct {
    Lorenz       traj [TRAJ_MAX];
    Trail        trail[TRAJ_MAX];
    LogPlot      log_delta;
    float        t_sim;
    PresetState  preset;
    PaletteState palette;
    bool         paused;
} Scene;

static inline const SDPreset *scene_active_preset(const Scene *s)
{
    return preset_state_active(&s->preset);
}

static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

/* Place the paths' starting points the tiny gap apart: path 0 at the
 * base, path 1 nudged one way, path 2 (TRIO only) nudged the other.
 * Everything else is identical — that near-identical start, drifting
 * apart, is the whole experiment Lorenz ran. */
static void scene_seed_perturbations(Scene *s)
{
    float eps = scene_active_preset(s)->eps;
    LorenzSystem sys = lorenz_system_canonical();

    s->traj[0] = (Lorenz){ sys, (LorenzState){ IC_X,        IC_Y, IC_Z } };
    s->traj[1] = (Lorenz){ sys, (LorenzState){ IC_X + eps,  IC_Y, IC_Z } };
    s->traj[2] = (Lorenz){ sys, (LorenzState){ IC_X - eps,  IC_Y, IC_Z } };
}

static void scene_reset(Scene *s)
{
    scene_seed_perturbations(s);
    for (int i = 0; i < TRAJ_MAX; i++) trail_reset(&s->trail[i]);
    plot_reset(&s->log_delta);
    s->t_sim = 0.0f;
}
static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    preset_state_init (&s->preset,  PRESET_CLASSIC);
    palette_state_init(&s->palette, 0);
    scene_reset(s);
}

/* Flatten a 3-D point to the two axes the current view shows. One
 * place does this so the drawing code never picks axes by hand. */
static inline void trajectory_to_projection(const LorenzState *p, Projection proj,
                                            float *u, float *v)
{
    switch (proj) {
        default:
        case PROJ_XZ: *u = p->x; *v = p->z; break;
        case PROJ_XY: *u = p->x; *v = p->y; break;
        case PROJ_YZ: *u = p->y; *v = p->z; break;
    }
}

/* Straight-line distance between two paths' current points — how far
 * apart they've drifted. Shown in the HUD, and its log feeds the plot. */
static inline float delta_norm(const LorenzState *a, const LorenzState *b)
{
    float dx = a->x - b->x, dy = a->y - b->y, dz = a->z - b->z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/* The bottom edge of the divergence plot, set from the starting gap so
 * the curve always begins just above the floor whatever gap is in use
 * (a fixed floor would leave tiny-gap runs starting way off-screen). */
static inline float log_floor_for_eps(float eps)
{
    return logf(eps) - LOG_PLOT_FLOOR_PAD;
}

/* Move one path forward a step and remember where it landed on screen. */
static inline void trajectory_advance_and_record_sample(
    Lorenz *traj, Trail *trail, Projection proj)
{
    lorenz_rk4_step(traj, LORENZ_DT);
    float u, v;
    trajectory_to_projection(&traj->state, proj, &u, &v);
    trail_push(trail, u, v);
}

/* Record this moment's gap between paths 0 and 1 onto the plot. Guards
 * against a zero gap (can't happen after the first step, but log of 0
 * would blow up) by pinning it to the plot floor. */
static inline void scene_record_divergence(Scene *s, float log_floor)
{
    float d = delta_norm(&s->traj[0].state, &s->traj[1].state);
    plot_push(&s->log_delta, (d > 0.0f) ? logf(d) : log_floor);
}

/* One smallest step of the whole simulation: every path moves the same
 * tiny amount, then we note how far apart they are. They MUST step by
 * the exact same amount, or we'd be watching rounding noise drift them
 * apart instead of real chaos. */
static inline void scene_advance_one_substep(Scene *s,
                                             const SDPreset *active,
                                             float log_floor)
{
    for (int t = 0; t < active->n_traj; t++)
        trajectory_advance_and_record_sample(
            &s->traj[t], &s->trail[t], active->proj);
    s->t_sim += LORENZ_DT;
    scene_record_divergence(s, log_floor);
}

/* Advance the simulation a bit. We take several small steps per tick
 * so the motion stays smooth and accurate. */
static void scene_tick(Scene *s, float dt)
{
    (void)dt;
    if (s->paused) return;

    const SDPreset *active = scene_active_preset(s);
    float log_floor = log_floor_for_eps(active->eps);

    for (int sub = 0; sub < INT_STEPS_PER_TICK; sub++)
        scene_advance_one_substep(s, active, log_floor);
}

/* §9  screen — turning paths into characters on the terminal */

/*
 * Screen — the current terminal size.
 *
 * ncurses tracks the size in globals; we copy it here so the layout
 * code reads one tidy handle and a resize updates one place. Drawing
 * functions take this (or cols/rows) directly rather than peeking at
 * the globals.
 *
 *   cols : width  in characters.
 *   rows : height in characters.
 */
typedef struct { int cols, rows; } Screen;
static void screen_init(Screen *s) { initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(); getmaxyx(stdscr, s->rows, s->cols); }
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols); }
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* The world-space window (left/right, bottom/top) for each view, wide
 * enough to hold the whole attractor without clipping its edges. */
static inline void projection_axis_bounds(Projection proj,
                                          float *u_min, float *u_max,
                                          float *v_min, float *v_max)
{
    switch (proj) {
        default:
        case PROJ_XZ: *u_min = X_AXIS_MIN; *u_max = X_AXIS_MAX;
                      *v_min = Z_AXIS_MIN; *v_max = Z_AXIS_MAX; break;
        case PROJ_XY: *u_min = X_AXIS_MIN; *u_max = X_AXIS_MAX;
                      *v_min = Y_AXIS_MIN; *v_max = Y_AXIS_MAX; break;
        case PROJ_YZ: *u_min = Y_AXIS_MIN; *u_max = Y_AXIS_MAX;
                      *v_min = Z_AXIS_MIN; *v_max = Z_AXIS_MAX; break;
    }
}

/* Turn a world coordinate into a screen column/row inside a panel.
 * The vertical one is flipped so bigger values sit higher up, the way
 * we read a graph. */
static inline int world_u_to_cell_x(float u, int gx0, int w,
                                    float u_min, float u_max)
{
    int c = gx0 + (int)((u - u_min) / (u_max - u_min) * (float)w);
    if (c < gx0)        c = gx0;
    if (c >= gx0 + w)   c = gx0 + w - 1;
    return c;
}
static inline int world_v_to_cell_y(float v, int gy0, int h,
                                    float v_min, float v_max)
{
    int c = gy0 + (h - 1) - (int)((v - v_min) / (v_max - v_min) * (float)h);
    if (c < gy0)        c = gy0;
    if (c >= gy0 + h)   c = gy0 + h - 1;
    return c;
}

/* Where the oldest still-valid position sits in the round buffer. The
 * extra TRAIL_MAX keeps the wrap-around math from going negative. */
static inline int trail_oldest_index(const Trail *t)
{
    return (t->head - t->count + 1 + TRAIL_MAX) % TRAIL_MAX;
}

/* Pick which of the four shades a streak dot gets from its age, so the
 * streak fades from bright at the head to dim at the tail. */
static inline int trail_age_band(int age_from_newest, int n)
{
    int band = (TRAIL_BAND_COUNT - 1) - (age_from_newest * TRAIL_BAND_COUNT) / n;
    if (band < 0)                    band = 0;
    if (band > TRAIL_BAND_COUNT - 1) band = TRAIL_BAND_COUNT - 1;
    return band;
}

/* The colour for one streak dot. Paths A and B fade through four
 * shades; the third path (pair_base < 0) just uses one flat colour. */
static inline short trail_pair_for_age(int pair_base, int live_pair,
                                       int age_from_newest, int n)
{
    if (pair_base < 0) return (short)live_pair;
    return (short)(pair_base + trail_age_band(age_from_newest, n));
}

/* Drop one coloured character on screen in a single call. The cast
 * dodges an ncurses gotcha where bytes over 127 get garbled. */
static inline void paint_cell(int sy, int sx, char glyph, short pair_id)
{
    attron(COLOR_PAIR(pair_id) | A_BOLD);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair_id) | A_BOLD);
}

/* Draw one path's fading streak inside a panel: walk oldest to newest,
 * placing a '.' for each remembered position, dimmer the older it is. */
static void paint_trail_in_rect(const Trail *tr, int pair_base, int live_pair,
                                int gx0, int gy0, int w, int h,
                                float u_min, float u_max,
                                float v_min, float v_max)
{
    if (tr->count == 0) return;
    int n      = tr->count;
    int oldest = trail_oldest_index(tr);

    for (int i = 0; i < n; i++) {
        int idx              = (oldest + i) % TRAIL_MAX;
        int age_from_newest  = n - 1 - i;
        int sx               = world_u_to_cell_x(tr->x[idx], gx0, w, u_min, u_max);
        int sy               = world_v_to_cell_y(tr->y[idx], gy0, h, v_min, v_max);
        short pair           = trail_pair_for_age(pair_base, live_pair,
                                                  age_from_newest, n);
        paint_cell(sy, sx, '.', pair);
    }
}

/* Draw the plot's faint L-shaped frame — just the left and bottom
 * edges. The top and right are left open so the curve can run right up
 * to the latest point without bumping a border. */
static void paint_log_plot_axis_frame(int gx0, int gy0, int w, int h)
{
    attron(COLOR_PAIR(PAIR_AXIS));
    for (int y = 0; y < h; y++) mvaddch(gy0 + y, gx0, (chtype)'|');
    for (int x = 0; x < w; x++) mvaddch(gy0 + h - 1, gx0 + x, (chtype)'-');
    attroff(COLOR_PAIR(PAIR_AXIS));
}

/* Find the i-th most recent value in the round buffer (i=0 is the
 * newest). Lets the plot walk newest-to-oldest without wrap-around
 * fuss at the call site. */
static inline int log_plot_newest_first_index(const LogPlot *p, int i_from_newest)
{
    int oldest = (p->head - p->count + 1 + PLOT_MAX) % PLOT_MAX;
    return (oldest + p->count - 1 - i_from_newest) % PLOT_MAX;
}

/* Turn one log-gap value into a screen row in the plot, clamped to the
 * chart's range. Because the rows are evenly spaced in log units, the
 * steady drift-apart phase comes out as a straight slanted line whose
 * slope is the divergence rate. */
static inline int log_value_to_cell_y(float lg, float log_floor,
                                      int gy0, int h)
{
    if (lg < log_floor) lg = log_floor;
    if (lg > LOG_MAX)   lg = LOG_MAX;
    float fraction = (lg - log_floor) / (LOG_MAX - log_floor);
    return gy0 + (h - 1) - (int)(fraction * (float)(h - 1));
}

/* Draw the divergence curve: one '*' per column, newest at the right.
 * Stops short by one column so it doesn't sit on the axis line. */
static void paint_log_plot_curve(const LogPlot *p, int gx0, int gy0,
                                 int w, int h, float log_floor)
{
    int n = p->count;
    int max_samples = (n < w - 1) ? n : (w - 1);

    attron(COLOR_PAIR(PAIR_PLOT) | A_BOLD);
    for (int i = 0; i < max_samples; i++) {
        int   idx = log_plot_newest_first_index(p, i);
        float lg  = p->v[idx];
        int   sy  = log_value_to_cell_y(lg, log_floor, gy0, h);
        int   sx  = gx0 + (w - 1) - i;
        if (sx > gx0 && sy >= gy0 && sy < gy0 + h)
            mvaddch(sy, sx, (chtype)'*');
    }
    attroff(COLOR_PAIR(PAIR_PLOT) | A_BOLD);
}

/* Draw the whole divergence plot: frame plus curve. Skips drawing if
 * there's nothing yet or the panel is too small to read. */
static void paint_log_plot(const LogPlot *p, int gx0, int gy0,
                           int w, int h, float log_floor)
{
    if (p->count == 0
     || w < PLOT_MIN_WIDTH_CELLS
     || h < PLOT_MIN_HEIGHT_CELLS) return;

    paint_log_plot_axis_frame(gx0, gy0, w, h);
    paint_log_plot_curve     (p, gx0, gy0, w, h, log_floor);
}

/*
 * Rect — one panel of the screen.
 *
 * The layout code carves the screen into one or two of these and hands
 * them to the painters. Passing a named box instead of four loose ints
 * keeps anyone from swapping the corner and the size by accident.
 *
 *   gx0 : leftmost column.
 *   gy0 : top row.
 *   w   : width in characters.
 *   h   : height in characters.
 */
typedef struct { int gx0, gy0, w, h; } Rect;

/* Draw the '@' that marks where a path is right now. */
static void paint_live_marker(const Vec3 *p, Projection proj, short pair,
                              const Rect *r,
                              float u_min, float u_max,
                              float v_min, float v_max)
{
    float u, v;
    trajectory_to_projection(p, proj, &u, &v);
    int sx = world_u_to_cell_x(u, r->gx0, r->w, u_min, u_max);
    int sy = world_v_to_cell_y(v, r->gy0, r->h, v_min, v_max);
    paint_cell(sy, sx, '@', pair);
}

/* Colours per path: index 0 = A, 1 = B, 2 = C. The -1 means "no fade,
 * use one flat colour" — that's the third path in TRIO presets. */
static const int trail_pair_table[TRAJ_MAX] = { PAIR_A_BASE, PAIR_B_BASE, -1 };
static const int live_pair_table [TRAJ_MAX] = { PAIR_LIVE_A, PAIR_LIVE_B, PAIR_LIVE_C };

/* Draw some paths (streaks + current-position markers) into one panel.
 * Markers go in a second pass so every '@' sits on top of the streaks
 * rather than getting buried under a later path's dots. */
static void paint_attractor_panel(const Scene *s, const Rect *r,
                                  int trail_start, int trail_count)
{
    const SDPreset *active = scene_active_preset(s);
    Projection proj = active->proj;

    float u_min, u_max, v_min, v_max;
    projection_axis_bounds(proj, &u_min, &u_max, &v_min, &v_max);

    for (int t = trail_start; t < trail_start + trail_count; t++) {
        if (t >= active->n_traj) break;
        paint_trail_in_rect(&s->trail[t], trail_pair_table[t], live_pair_table[t],
                            r->gx0, r->gy0, r->w, r->h,
                            u_min, u_max, v_min, v_max);
    }
    for (int t = trail_start; t < trail_start + trail_count; t++) {
        if (t >= active->n_traj) break;
        paint_live_marker(&s->traj[t].state, proj, (short)live_pair_table[t], r,
                          u_min, u_max, v_min, v_max);
    }
}

/* The whole drawing area (everything but the HUD rows) as one panel. */
static inline Rect drawable_band_full(int cols, int rows)
{
    return (Rect){ 0, HUD_TOP_ROWS, cols, rows - HUD_BAND_RESERVED_ROWS };
}

/* The thin dividing lines: down the middle for SPLIT, across for the
 * stacked plot layout. */
static void paint_vertical_divider(int col, int gy0, int h)
{
    attron(COLOR_PAIR(PAIR_AXIS));
    for (int y = 0; y < h; y++) mvaddch(gy0 + y, col, (chtype)'|');
    attroff(COLOR_PAIR(PAIR_AXIS));
}
static void paint_horizontal_divider(int row, int cols)
{
    attron(COLOR_PAIR(PAIR_AXIS));
    for (int x = 0; x < cols; x++) mvaddch(row, x, (chtype)'-');
    attroff(COLOR_PAIR(PAIR_AXIS));
}

/* FULL: all paths overlaid on the whole screen. */
static void paint_layout_full(const Scene *s, int cols, int rows)
{
    Rect band = drawable_band_full(cols, rows);
    paint_attractor_panel(s, &band, 0, TRAJ_MAX);
}

/* SPLIT: path A fills the left half, path B the right, divider between. */
static void paint_layout_split(const Scene *s, int cols, int rows)
{
    int drawable_h = rows - HUD_BAND_RESERVED_ROWS;
    int half_w     = (cols - PAINT_GUTTER_WIDTH) / 2;

    Rect left  = { 0,                          HUD_TOP_ROWS, half_w, drawable_h };
    Rect right = { half_w + PAINT_GUTTER_WIDTH, HUD_TOP_ROWS, half_w, drawable_h };

    paint_attractor_panel(s, &left,  0, 1);    /* path A only */
    paint_attractor_panel(s, &right, 1, 1);    /* path B only */
    paint_vertical_divider(half_w, HUD_TOP_ROWS, drawable_h);
}

/* How tall the bottom plot panel is — about 2/5 of the drawing area. */
static inline int log_plot_band_height(int drawable_h)
{
    return drawable_h * LOG_PLOT_BAND_NUMERATOR / LOG_PLOT_BAND_DENOMINATOR;
}

/* PLOT_BOTTOM: attractor on top (~60%), divergence plot below (~40%),
 * a line between them. */
static void paint_layout_plot_bottom(const Scene *s, int cols, int rows)
{
    const SDPreset *active = scene_active_preset(s);
    int drawable_h = rows - HUD_BAND_RESERVED_ROWS;
    int plot_h     = log_plot_band_height(drawable_h);
    int attr_h     = drawable_h - plot_h - PAINT_DIVIDER_ROWS;

    Rect attr_rect = { 0, HUD_TOP_ROWS,
                       cols, attr_h };
    Rect plot_rect = { 0, HUD_TOP_ROWS + attr_h + PAINT_DIVIDER_ROWS,
                       cols, plot_h };

    paint_attractor_panel(s, &attr_rect, 0, TRAJ_MAX);
    paint_log_plot(&s->log_delta,
                   plot_rect.gx0, plot_rect.gy0, plot_rect.w, plot_rect.h,
                   log_floor_for_eps(active->eps));
    paint_horizontal_divider(HUD_TOP_ROWS + attr_h, cols);
}

/* PLOT_ONLY: the divergence plot takes the whole screen. */
static void paint_layout_plot_only(const Scene *s, int cols, int rows)
{
    const SDPreset *active = scene_active_preset(s);
    Rect r = drawable_band_full(cols, rows);
    paint_log_plot(&s->log_delta, r.gx0, r.gy0, r.w, r.h,
                   log_floor_for_eps(active->eps));
}

/* Draw the scene using whichever layout the current preset asks for. */
static void scene_paint(const Scene *s, int cols, int rows)
{
    switch (scene_active_preset(s)->layout) {
        case LAYOUT_FULL:        paint_layout_full       (s, cols, rows); break;
        case LAYOUT_SPLIT:       paint_layout_split      (s, cols, rows); break;
        case LAYOUT_PLOT_BOTTOM: paint_layout_plot_bottom(s, cols, rows); break;
        case LAYOUT_PLOT_ONLY:   paint_layout_plot_only  (s, cols, rows); break;
    }
}

static inline void hud_write_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " SENSITIVE DEPENDENCE (Lorenz) ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* The right-hand status on the top row: frame rate, sim rate, current
 * preset, elapsed time, and how far apart the two paths are now. */
static inline void hud_write_status_right(int cols, double fps, int sim_fps,
                                          const Scene *s)
{
    const SDPreset *active = scene_active_preset(s);
    float d = delta_norm(&s->traj[0].state, &s->traj[1].state);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  t:%.1fs  |δ|:%.2e ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : active->name,
             s->preset.current + 1, (int)N_PRESETS,
             (double)s->t_sim, (double)d);

    int hx = cols - (int)strlen(buf); if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_top(int cols, double fps, int sim_fps, const Scene *s)
{
    hud_write_title();
    hud_write_status_right(cols, fps, sim_fps, s);
}

/* The three labelled chunks of the HUD's second row. */
static inline void hud_write_preset_label(int x, const SDPreset *active)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " preset:%-8s ", active->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}
static inline void hud_write_theme_label(int x, const Scene *s)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", palette_state_active(&s->palette)->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}
static inline void hud_write_eps_traj_lambda(int x, const SDPreset *active)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " ε:%.0e  n:%d  Lorenz λ≈%.3f ",
             (double)active->eps, active->n_traj, (double)LORENZ_LAMBDA);
    attroff(COLOR_PAIR(PAIR_HUD));
}

static void hud_param(const Scene *s)
{
    const SDPreset *active = scene_active_preset(s);
    int x = HUD_LEFT_MARGIN;

    hud_write_preset_label   (x, active); x += HUD_PARAM_PRESET_WIDTH;
    hud_write_theme_label    (x, s);      x += HUD_PARAM_THEME_WIDTH;
    hud_write_eps_traj_lambda(x, active);
}

static void hud_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:preset  t/T:theme  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* Paint one whole frame: clear, draw the scene, then the HUD on top. */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_paint(s, sc->cols, sc->rows);
    hud_top   (sc->cols, fps, sim_fps, s);
    hud_param (s);
    hud_hint  (sc->rows);
}

/* §10 app — signals, resize, keys, timing, and the main loop */

/*
 * App — the whole program in one box.
 *
 * Holds the simulation, the screen size, the speed knob, and two flags
 * the signal handlers flip. It lives as a single global so the signal
 * handlers can reach it without being handed a pointer.
 *
 *   scene       : the simulation.
 *   screen      : the cached terminal size.
 *   sim_fps     : how many times a second the sim steps; clamped to
 *                 10..240.
 *   running     : the main loop runs while this is set; cleared by 'q'
 *                 or a kill signal.
 *   need_resize : set when the terminal is resized; cleared once handled.
 *                 (Both flags are sig_atomic_t because signal handlers
 *                 touch them.)
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Hook up the signals: Ctrl-C / kill ask us to quit, a resize asks us
 * to relayout. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* First-time setup before the loop starts. */
static void app_bootstrap(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    scene_init(&app->scene);
}

/* If the terminal was resized, re-read its size and restart the run. */
static void app_handle_pending_resize(App *app)
{
    if (!app->need_resize) return;
    screen_resize(&app->screen);
    scene_reset(&app->scene);
    app->need_resize = 0;
}

/* How long since the last frame, capped so that if the program stalls
 * (window dragged, laptop slept) it doesn't try to catch up all at
 * once and lock up. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > SIM_MAX_FRAME_DT_MS * NS_PER_MS)
        dt = SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    return dt;
}

/* Step the simulation in fixed-size chunks, however many fit in the
 * time that's passed. Keeping the step size fixed (not tied to frame
 * rate) matters here: every path must step identically, or the split
 * we're showing would be rounding error rather than real chaos. */
static void app_drain_fixed_timestep(App *app, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* Recompute the displayed frame rate twice a second (smoother than
 * updating it every single frame). */
static void app_update_fps_meter(int64_t *fps_accum, int *frame_count,
                                 double *fps_display)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_display = (double)*frame_count
                 / ((double)*fps_accum / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
}

/* Sleep off whatever time is left in this frame so we draw at ~60 fps
 * instead of spinning the CPU flat out. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* Draw the frame and push it to the terminal. */
static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* Speed the sim up / slow it down, staying within the allowed range. */
static void app_sim_rate_faster(App *app)
{
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
}
static void app_sim_rate_slower(App *app)
{
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

/* One-line actions, named so the key table below reads like a list of
 * intentions. */
static void app_toggle_pause     (App *app) { app->scene.paused = !app->scene.paused; }
static void app_reset_attractor  (App *app) { scene_reset(&app->scene); }
static void app_cycle_theme_next (App *app) { palette_state_cycle_next(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_theme_prev (App *app) { palette_state_cycle_prev(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_preset_next(App *app)
{
    preset_state_cycle_next(&app->scene.preset);
    scene_reset(&app->scene);   /* new preset, new starting gap — start over */
}
static void app_cycle_preset_prev(App *app)
{
    preset_state_cycle_prev(&app->scene.preset);
    scene_reset(&app->scene);
}

static bool app_handle_key(App *app, int ch);

/* Check for a keypress without blocking and act on it. Returns false
 * when the key means "quit". */
static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* Which key does what. */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            app_toggle_pause     (app); break;
    case 'r': case 'R':  app_reset_attractor  (app); break;
    case ']':            app_sim_rate_faster  (app); break;
    case '[':            app_sim_rate_slower  (app); break;
    case 't':            app_cycle_theme_next (app); break;
    case 'T':            app_cycle_theme_prev (app); break;
    case 'n': case 'N':  app_cycle_preset_next(app); break;
    case 'p': case 'P':  app_cycle_preset_prev(app); break;
    default: break;
    }
    return true;
}

/*
 * FrameClock — the timekeeping the main loop carries around.
 *
 * Bundles the five running timers so main() can read like a short
 * recipe instead of juggling five loose locals everywhere.
 *
 *   frame_time  : when the previous frame started.
 *   sim_accum   : time owed to the simulation, paid off in fixed steps.
 *   fps_accum   : time piled up since we last recomputed the fps number.
 *   frame_count : frames drawn since then.
 *   fps_display : the frame rate currently shown in the HUD.
 */
typedef struct {
    int64_t frame_time;
    int64_t sim_accum;
    int64_t fps_accum;
    int     frame_count;
    double  fps_display;
} FrameClock;

static void frame_clock_init(FrameClock *c)
{
    c->frame_time  = clock_ns();
    c->sim_accum   = 0;
    c->fps_accum   = 0;
    c->frame_count = 0;
    c->fps_display = 0.0;
}
static void frame_clock_reset_after_resize(FrameClock *c)
{
    c->frame_time = clock_ns();
    c->sim_accum  = 0;
}
static void frame_clock_advance(FrameClock *c, int64_t dt)
{
    c->sim_accum += dt;
    c->fps_accum += dt;
    c->frame_count++;
}

/* The whole program in one loop: handle resize, advance time, step the
 * sim, draw, check for a key, repeat until quit. */
int main(void)
{
    main_install_signal_handlers();

    App *app = &g_app;
    app_bootstrap(app);

    FrameClock clk;
    frame_clock_init(&clk);

    while (app->running) {
        if (app->need_resize) {
            app_handle_pending_resize(app);
            frame_clock_reset_after_resize(&clk);
        }

        int64_t dt = app_compute_frame_dt(&clk.frame_time);
        frame_clock_advance(&clk, dt);
        app_drain_fixed_timestep(app, &clk.sim_accum);
        app_update_fps_meter(&clk.fps_accum, &clk.frame_count, &clk.fps_display);

        app_throttle_to_render_target(clk.frame_time, dt);
        app_present_frame(app, clk.fps_display);

        if (!app_poll_keyboard(app)) app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
