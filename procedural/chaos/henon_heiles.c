/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * henon_heiles.c — a star orbiting a galaxy, drawn as a Poincare section.
 * We fly many orbits at the same energy E and dot the screen each time one
 * crosses a chosen line; low E gives neat loops, high E a chaotic spray.
 *
 * Original problem and the low-E/high-E pictures: Henon & Heiles, Astron. J.
 * 69 (1964), pp. 73-79.  Sister demos: standard_map.c, poincare_section.c,
 * double_pendulum.c.
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
    MAP_W_MAX        = 200, MAP_H_MAX = 56, CELLS_MAX = MAP_W_MAX * MAP_H_MAX,
    SIM_FPS_MIN      =  10, SIM_FPS_DEFAULT = 60, SIM_FPS_MAX = 240, SIM_FPS_STEP = 10,
    HUD_COLS         =  80, FPS_UPDATE_MS = 500,
    PAIR_HUD         =   1, PAIR_HINT = 2,
    PAIR_DENS_BASE   =   3,    /* first of 8 colours, dim to bright */
    PAIR_BOUNDARY    =  11,
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

#define HH_DT                     0.05f
#define INT_STEPS_PER_TICK         60
#define N_TRAJ                     32

/* The window we draw the (y, py) section in.  Orbits stay inside this box
 * as long as their energy is below the escape value 1/6. */
#define Y_MIN                    -0.5f
#define Y_MAX                     0.7f
#define PY_MIN                   -0.5f
#define PY_MAX                    0.5f

#define DENS_BAND_COUNT           8
#define DENS_SAT_LOG              7.0f

/* The seven energies we let you flip through, ordered calm to chaotic.
 * Each one looks clearly different from its neighbour, not just a tweak. */
typedef enum {
    PRESET_QUIET = 0,    /* E = 0.02   tiny central tori only           */
    PRESET_LOW_E,        /* E = 0.07   classic regular-regime plot      */
    PRESET_BUDDING,      /* E = 0.10   chain-of-three islands appear    */
    PRESET_MIXED,        /* E = 0.12   islands + thin chaotic ring      */
    PRESET_BROKEN,       /* E = 0.14   most tori dissolved              */
    PRESET_HIGH_E,       /* E = 0.16   classic chaotic-regime plot      */
    PRESET_BRINK,        /* E = 0.165  near-escape, attractor bounded   */
    N_PRESETS,
} Preset;

/*
 * HHPreset — one row of the presets[] menu: a name plus an energy.
 *
 * The galaxy equation itself never changes; the only dial in this whole
 * demo is the total energy E the orbits start with.  Turning that dial is
 * what takes the picture from calm loops to chaos, so we pair each chosen
 * energy with a short label and let you browse them.
 *
 *   name : the short tag shown in the status bar (padded to 8 chars).
 *   E    : the starting energy.  Keep it above 0 and below 1/6 (about
 *          0.1667) -- past that the orbits fly off the edge and never come
 *          back to draw a dot.
 */
typedef struct {
    const char *name;
    float       E;
} HHPreset;

/* Calm to chaotic.  Nothing above 1/6 (about 0.1667): past there the orbits
 * escape to infinity and never cross the line again. */
static const HHPreset presets[N_PRESETS] = {
    { "QUIET   ", 0.020f },
    { "LOW_E   ", 0.070f },
    { "BUDDING ", 0.100f },
    { "MIXED   ", 0.120f },
    { "BROKEN  ", 0.140f },
    { "HIGH_E  ", 0.160f },
    { "BRINK   ", 0.165f },
};

/*
 * Theme — one colour scheme for the drawing.  Pressing t/T swaps the whole
 * look without touching any physics or drawing code.
 *
 *   name      : the short tag shown in the status bar (padded to 7 chars).
 *   band[8]   : the colour ramp for dot density, dimmest first, brightest
 *               last.  Order matters: thinly-visited spots get band[0],
 *               heavily-visited ones get band[7], so the ramp shows how
 *               often each spot was hit.  Keep every colour at xterm index
 *               24 or higher or the dim end vanishes on a black terminal.
 *   bnd       : one colour for the frame, the centre guide lines, and the
 *               energy edge -- a plain grey so it frames the data without
 *               stealing attention.
 */
typedef struct {
    const char *name;
    short       band[DENS_BAND_COUNT];
    short       bnd;
} Theme;
#define N_THEMES 10
static const Theme themes[N_THEMES] = {
    { "DEFAULT", {  39,  75, 117, 153, 189, 220, 226, 231 }, 244 },
    { "MATRIX",  {  46,  82, 118, 154, 156, 191, 192, 194 }, 244 },
    { "NOVA",    {  99, 135, 171, 207, 213, 219, 225, 231 }, 244 },
    { "MONO",    { 244, 247, 250, 252, 253, 254, 255, 231 }, 240 },
    { "OCEAN",   {  39,  45,  81, 117, 153, 159, 195, 231 }, 244 },
    { "FIRE",    {  88, 124, 160, 196, 202, 208, 220, 227 }, 244 },
    { "EARTH",   { 100, 137, 173, 179, 215, 222, 228, 231 }, 244 },
    { "FOREST",  {  64, 107, 114, 144, 150, 156, 194, 231 }, 244 },
    { "DESERT",  { 130, 173, 179, 215, 222, 228, 230, 231 }, 244 },
    { "ARCTIC",  {  81, 117, 153, 159, 195, 225, 230, 231 }, 244 },
};

/* §2 clock + §3 color */

static int64_t clock_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec; }
static void clock_sleep_ns(int64_t ns) { if (ns <= 0) return;
    struct timespec req = {ns/NS_PER_SEC, ns%NS_PER_SEC}; nanosleep(&req, NULL); }

/* The HUD's bright yellow and cyan, as xterm-256 numbers. */
#define PAIR_HUD_FG_256    226
#define PAIR_HINT_FG_256    51

/* Load a theme's colours on a full 256-colour terminal. */
static void theme_apply_pairs_256color(const Theme *t)
{
    for (int i = 0; i < DENS_BAND_COUNT; i++)
        init_pair(PAIR_DENS_BASE + i, t->band[i], -1);
    init_pair(PAIR_BOUNDARY, t->bnd, -1);
}

/* On an old 8-colour terminal we can't show the fancy themes, so every
 * theme falls back to the same plain set -- we trade variety for staying
 * readable everywhere. */
static void theme_apply_pairs_8color_fallback(void)
{
    for (int i = 0; i < DENS_BAND_COUNT; i++)
        init_pair(PAIR_DENS_BASE + i, COLOR_CYAN, -1);
    init_pair(PAIR_BOUNDARY, COLOR_WHITE, -1);
}

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_apply_pairs_256color(&themes[idx]);
    else               theme_apply_pairs_8color_fallback();
}

/* The HUD's yellow and cyan stay fixed no matter which theme is on, so the
 * status text never loses contrast against the animation. */
static void color_init_hud_pairs(void)
{
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  PAIR_HUD_FG_256,  -1);
        init_pair(PAIR_HINT, PAIR_HINT_FG_256, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    color_init_hud_pairs();
    theme_apply(0);
}

/* §5  physics — the Henon-Heiles galaxy orbit */

/*
 * HHState — where one orbiting star is right now: its place (x, y) and its
 * momentum (px, py).  These four numbers are all you need to step the orbit
 * forward; there's no clock to track because the rules never change in time.
 *
 * It's a small plain struct passed by value so the RK4 stepper can make and
 * mix temporary copies cheaply.  The motion comes from the galaxy's energy:
 *     total energy = motion energy 1/2(px^2 + py^2)
 *                  + position energy V(x, y) = 1/2(x^2 + y^2) + x^2 y - y^3/3
 * and the rates of change work out to:
 *     x grows at px,  y grows at py,
 *     px grows at -x - 2xy,  py grows at -y - x^2 + y^2.
 *
 *   x, y   : position.  The well is a bowl with three lips at distance 1
 *            forming a triangle; bound stars stay inside, energy above 1/6
 *            lets them slip over a lip and escape.
 *   px, py : momentum (same as the speeds here, since we take mass 1).
 */
typedef struct {
    float x, y;
    float px, py;
} HHState;

/* Position energy V(x, y): a triangular bowl with three lips around the
 * centre.  Once the energy clears 1/6 the star can reach a lip and escape. */
static inline float hh_potential(float x, float y)
{
    return 0.5f * (x*x + y*y) + x*x*y - y*y*y / 3.0f;
}

/* Motion energy: how much energy is in the star's speed. */
static inline float hh_kinetic(float px, float py)
{
    return 0.5f * (px*px + py*py);
}

/* Total energy.  It stays fixed along an orbit, so we use it to launch every
 * star at the same chosen energy. */
static inline float hh_total_energy(const HHState *s)
{
    return hh_kinetic(s->px, s->py) + hh_potential(s->x, s->y);
}

/* How fast each of the four numbers is changing right now. */
static inline HHState hh_deriv(const HHState *s)
{
    HHState out;
    out.x  =  s->px;
    out.y  =  s->py;
    out.px = -s->x - 2.0f * s->x * s->y;
    out.py = -s->y -        s->x * s->x + s->y * s->y;
    return out;
}

/* Take a step: a + h*k.  A small building block for the RK4 stepper. */
static inline HHState state_add(const HHState *a, float h, const HHState *k)
{
    HHState r;
    r.x  = a->x  + h * k->x;
    r.y  = a->y  + h * k->y;
    r.px = a->px + h * k->px;
    r.py = a->py + h * k->py;
    return r;
}

#define RK4_BUTCHER_WEIGHT_SUM   6.0f
#define RK4_MIDPOINT_FRACTION    0.5f

/* Blend the four trial slopes into one good average, weighting the two
 * middle ones double: (k1 + 2*k2 + 2*k3 + k4) / 6. */
static inline HHState rk4_butcher_weighted_average(
    const HHState *k1, const HHState *k2,
    const HHState *k3, const HHState *k4)
{
    HHState avg;
    avg.x  = (k1->x  + 2.0f*k2->x  + 2.0f*k3->x  + k4->x ) / RK4_BUTCHER_WEIGHT_SUM;
    avg.y  = (k1->y  + 2.0f*k2->y  + 2.0f*k3->y  + k4->y ) / RK4_BUTCHER_WEIGHT_SUM;
    avg.px = (k1->px + 2.0f*k2->px + 2.0f*k3->px + k4->px) / RK4_BUTCHER_WEIGHT_SUM;
    avg.py = (k1->py + 2.0f*k2->py + 2.0f*k3->py + k4->py) / RK4_BUTCHER_WEIGHT_SUM;
    return avg;
}

/* Move one orbit forward by a small time step, using the classic RK4 method:
 * sample the slope at the start, twice in the middle, and at the end, then
 * step by their weighted average.  Far more accurate than a single guess.
 *
 * Why RK4 and not something fancier: a "symplectic" stepper would hold the
 * energy steadier over very long runs, but at this step size and run length
 * RK4 drifts under 0.1% -- you won't see it.  Switch only if you ever run for
 * hours and notice the picture slowly distorting. */
static void hh_rk4(HHState *s, float dt)
{
    float half_dt = RK4_MIDPOINT_FRACTION * dt;

    HHState slope_start    = hh_deriv(s);
    HHState midpoint_1     = state_add(s, half_dt, &slope_start);
    HHState slope_mid_1    = hh_deriv(&midpoint_1);
    HHState midpoint_2     = state_add(s, half_dt, &slope_mid_1);
    HHState slope_mid_2    = hh_deriv(&midpoint_2);
    HHState endpoint       = state_add(s, dt, &slope_mid_2);
    HHState slope_end      = hh_deriv(&endpoint);

    HHState effective_slope = rk4_butcher_weighted_average(
        &slope_start, &slope_mid_1, &slope_mid_2, &slope_end);
    *s = state_add(s, dt, &effective_slope);
}

/* Place a star on the launch line (x = 0) at height y, sitting still in y,
 * and give it just enough x-speed so its total energy hits the target.  If
 * that height already costs more than the budget, there's no speed left, so
 * we park it at zero and it simply never crosses the line. */
static inline HHState hh_make_state_on_x0_at_energy(float y, float E_target)
{
    float V              = hh_potential(0.0f, y);
    float kinetic_budget = 2.0f * (E_target - V);
    if (kinetic_budget < 0.0f) kinetic_budget = 0.0f;
    HHState s = { 0.0f, y, sqrtf(kinetic_budget), 0.0f };
    return s;
}

/* §6  section grid */

/*
 * SectionGrid — a tally board.  One snapshot of the orbits tells you almost
 * nothing; the picture only appears after thousands of dots pile up.  So we
 * lay a grid over the (y, py) window and, for every dot, bump the count in
 * the cell it lands in.  Cells hit often become the bright loops; cells hit
 * rarely become the faint chaotic haze.
 *
 * A grid of counts beats keeping a list of every dot: it never grows and
 * never allocates, and recording a dot is just one bump no matter how long
 * the demo has run.  A cell tops out at 65535 hits, which takes hours to
 * reach, so we never worry about overflow.
 *
 *   w, h  : grid size in cells, set when we (re)start; matches the screen
 *           window we draw into.
 *   count : w*h, kept around so a reset can clear only the part we use.
 *   hits  : the counts, stored row by row.  Row 0 is the TOP of the picture
 *           (highest py).  We flip the row when recording so the drawing code
 *           can just sweep top to bottom without flipping again.
 */
typedef struct {
    int      w, h, count;
    uint16_t hits[CELLS_MAX];
} SectionGrid;

static void section_reset(SectionGrid *g, int w, int h)
{ g->w = w; g->h = h; g->count = w * h;
  memset(g->hits, 0, sizeof(uint16_t) * (size_t)g->count); }

static bool yp_to_cell(const SectionGrid *g, float y, float py, int *cx, int *cy)
{
    if (y < Y_MIN || y > Y_MAX || py < PY_MIN || py > PY_MAX) return false;
    *cx = (int)((y  - Y_MIN ) / (Y_MAX  - Y_MIN ) * (float)g->w);
    *cy = (int)((py - PY_MIN) / (PY_MAX - PY_MIN) * (float)g->h);
    if (*cx < 0)        *cx = 0;
    if (*cx >= g->w)    *cx = g->w - 1;
    if (*cy < 0)        *cy = 0;
    if (*cy >= g->h)    *cy = g->h - 1;
    return true;
}

static void section_record(SectionGrid *g, float y, float py)
{
    int cx, cy; if (!yp_to_cell(g, y, py, &cx, &cy)) return;
    int idx = (g->h - 1 - cy) * g->w + cx;
    if (g->hits[idx] < UINT16_MAX) g->hits[idx]++;
}

/* Added before rounding a fraction down to a whole band, so we round to the
 * nearest band instead of always rounding down. */
#define DENSITY_BAND_ROUND_OFFSET   0.5f

/* Turn a hit-count into a brightness level 0..7 (-1 if the cell is empty).
 * We compare counts on a log scale, not directly: a hot loop might be hit
 * thousands of times and a faint spot only a handful, and on a plain scale
 * the faint spots would all collapse to invisible.  Log keeps both ends
 * readable. */
static inline int density_band(uint16_t hits)
{
    if (hits == 0) return -1;

    float saturated_log = logf((float)hits) / DENS_SAT_LOG;
    if (saturated_log < 0.0f) saturated_log = 0.0f;
    if (saturated_log > 1.0f) saturated_log = 1.0f;

    int band = (int)(saturated_log * (float)(DENS_BAND_COUNT - 1)
                     + DENSITY_BAND_ROUND_OFFSET);
    if (band > DENS_BAND_COUNT - 1) band = DENS_BAND_COUNT - 1;
    return band;
}

/* §7  state — small wrappers + the flock of orbits */

/*
 * TrajectoryEnsemble — a flock of N_TRAJ stars flown at once, all at the
 * same energy but launched from different heights.  One star alone draws one
 * loop or one chaotic smear; flying many at once paints the whole picture --
 * every loop and the chaos around it -- in a single run.  This is exactly how
 * Henon & Heiles made their original plots.
 *
 * Alongside each star's current state we keep its previous state, so when it
 * crosses the launch line we can pin down exactly where the crossing was.
 *
 *   state [i] : star i's current place and speed.
 *   prev  [i] : star i's place and speed one step ago, used to find the
 *               exact crossing point.
 *   prev_x[i] : just the previous x, kept separate as a fast way to spot a
 *               sign change before doing the full crossing maths.
 *   count     : how many stars (always N_TRAJ here).
 */
typedef struct {
    HHState state [N_TRAJ];
    HHState prev  [N_TRAJ];
    float   prev_x[N_TRAJ];
    int     count;
} TrajectoryEnsemble;

/* The range of launch heights we spread the stars over.  Kept inside the
 * reachable region for every preset so each star has a valid launch speed. */
#define TRAJECTORY_SEED_Y_MIN  -0.4f
#define TRAJECTORY_SEED_Y_MAX   0.6f

/* Launch all the stars on the line x = 0, spread evenly across the height
 * range, each given the right speed to hit the chosen energy. */
static void trajectory_ensemble_init_at_energy(TrajectoryEnsemble *ens,
                                               float E_target)
{
    ens->count = N_TRAJ;
    for (int i = 0; i < N_TRAJ; i++) {
        float fraction = (float)(i + 1) / (float)(N_TRAJ + 1);
        float y_seed   = TRAJECTORY_SEED_Y_MIN
                       + fraction * (TRAJECTORY_SEED_Y_MAX - TRAJECTORY_SEED_Y_MIN);
        HHState seed   = hh_make_state_on_x0_at_energy(y_seed, E_target);
        ens->state [i] = seed;
        ens->prev  [i] = seed;
        ens->prev_x[i] = seed.x;
    }
}

/* Did the star just cross the line x = 0 moving in the +x direction?  We
 * only count crossings going one way (px > 0), the usual convention, so each
 * loop leaves one dot per pass rather than two. */
static inline bool trajectory_is_upward_x_crossing(float prev_x,
                                                   const HHState *current)
{
    return prev_x < 0.0f
        && current->x >= 0.0f
        && current->px > 0.0f;
}

/* The star jumps a little past the line each step, so we estimate where it
 * actually was when it touched x = 0 by sliding back between the before and
 * after points in proportion.  Accurate because the steps are tiny. */
static inline void trajectory_interpolate_section_point(
    const HHState *prev, float prev_x, const HHState *current,
    float *y_at_x0, float *py_at_x0)
{
    float dx    = current->x - prev_x;
    float alpha = (dx != 0.0f) ? -prev_x / dx : 0.0f;
    *y_at_x0    = prev->y  + alpha * (current->y  - prev->y );
    *py_at_x0   = prev->py + alpha * (current->py - prev->py);
}

/* Move one star one step, and if it just crossed the launch line, drop a dot
 * on the tally board at the exact crossing point.  This "only look at the
 * star when it crosses a chosen line" trick is the Poincare section: it turns
 * a tangled 3-D orbit into a flat pattern of dots that's far easier to read --
 * neat loops mean order, a filled smear means chaos. */
static void trajectory_advance_one(TrajectoryEnsemble *ens, int i,
                                   SectionGrid *section, float dt)
{
    /* remember where it was, step it, then check for a crossing */
    ens->prev  [i] = ens->state[i];
    ens->prev_x[i] = ens->state[i].x;

    hh_rk4(&ens->state[i], dt);

    if (!trajectory_is_upward_x_crossing(ens->prev_x[i], &ens->state[i]))
        return;
    float y_at_x0, py_at_x0;
    trajectory_interpolate_section_point(
        &ens->prev[i], ens->prev_x[i], &ens->state[i],
        &y_at_x0, &py_at_x0);
    section_record(section, y_at_x0, py_at_x0);
}

static void trajectory_ensemble_advance(TrajectoryEnsemble *ens,
                                        SectionGrid *section, float dt)
{
    for (int i = 0; i < ens->count; i++)
        trajectory_advance_one(ens, i, section, dt);
}

/* Which preset (energy) is currently chosen.  It's just an index into the
 * presets[] menu, but wrapping it with the next/prev/lookup helpers below
 * lets the key handler read like plain English instead of doing wrap-around
 * arithmetic by hand.  Stays in range 0..N_PRESETS-1; starts on MIXED, the
 * most interesting opener. */
typedef struct {
    int current;
} PresetState;

static void preset_state_init(PresetState *p, int initial)        { p->current = initial; }
static void preset_state_cycle_next(PresetState *p)               { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)               { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const HHPreset *preset_state_active(const PresetState *p)  { return &presets[p->current]; }
static float           preset_state_active_energy(const PresetState *p)
                                                                  { return presets[p->current].E; }

/* Which colour theme is currently chosen.  Same one-index shape as
 * PresetState, but kept as its own type on purpose: that way you can't
 * accidentally feed a theme number where a preset number was wanted, or run
 * one off the end of the wrong list.  Stays in range; starts on theme 0. */
typedef struct {
    int current;
} PaletteState;

static void palette_state_init(PaletteState *p, int initial)      { p->current = initial; }
static void palette_state_cycle_next(PaletteState *p)             { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p)             { p->current = (p->current + N_THEMES - 1) % N_THEMES; }
static const Theme *palette_state_active(const PaletteState *p)   { return &themes[p->current]; }
static void palette_state_apply(const PaletteState *p)            { theme_apply(p->current); }

/* §8  scene */

/*
 * Scene — everything the simulation can change, gathered in one place so
 * there are no loose globals and the main loop can drive it through a few
 * tidy calls.  Think of it as the whole experiment: the stars, the picture
 * they're painting, which energy is loaded, which colours, and whether it's
 * paused.
 *
 *   ensemble : the flock of orbiting stars.
 *   section  : the tally board the dots accumulate on.
 *   preset   : which energy is loaded; n/p change it.
 *   palette  : which theme is on; t/T change it.
 *   paused   : when set, the stars freeze but the picture keeps showing, so
 *              you can study the pattern that's built up.
 */
typedef struct {
    TrajectoryEnsemble ensemble;
    SectionGrid        section;
    PresetState        preset;
    PaletteState       palette;
    bool               paused;
} Scene;

/* Relaunch all the stars at the currently chosen energy.  Energy is the only
 * thing a preset changes -- the galaxy equation itself is fixed. */
static void scene_load_active_energy(Scene *s)
{
    trajectory_ensemble_init_at_energy(&s->ensemble,
                                       preset_state_active_energy(&s->preset));
}

static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

static void scene_reset(Scene *s, int section_w, int section_h)
{
    section_reset(&s->section, section_w, section_h);
    scene_load_active_energy(s);
}

static void scene_init(Scene *s, int section_w, int section_h)
{
    memset(s, 0, sizeof *s);
    s->paused = false;

    preset_state_init (&s->preset,  PRESET_MIXED);  /* the opening view */
    palette_state_init(&s->palette, 0);             /* first theme */

    scene_reset(s, section_w, section_h);
}

/* One round of simulation: nudge every star forward several small steps and
 * collect any dots.  Several little steps per frame keep the loops crisp. */
static void scene_tick(Scene *s, float dt)
{
    (void)dt;
    if (s->paused) return;
    for (int substep = 0; substep < INT_STEPS_PER_TICK; substep++)
        trajectory_ensemble_advance(&s->ensemble, &s->section, HH_DT);
}

/* §9  screen */

/*
 * Screen — our handle on the terminal: how wide and tall it is, plus the
 * one spot that turns ncurses on and off.  Carrying the size here means every
 * drawing helper takes it as one argument instead of asking the terminal
 * each time, and keeping all the setup/teardown in one place means nothing
 * else in the file has to deal with ncurses directly.
 *
 *   cols : width in characters.
 *   rows : height in characters.  Row 0 is the top; a couple of rows top and
 *          bottom are reserved for the status bars.
 */
typedef struct {
    int cols;
    int rows;
} Screen;
static void screen_init(Screen *s) { initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(); getmaxyx(stdscr, s->rows, s->cols); }
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols); }
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* The characters for the eight brightness levels, light to heavy.  They get
 * inkier as you go, so the picture reads even with no colour at all. */
static char glyph_for_band(int b)
{
    static const char ramp[DENS_BAND_COUNT] = {
        '.',  /* lightest, rarely hit */
        ':',
        ';',
        '+',
        'o',
        '*',
        '#',
        '@',  /* heaviest, hit constantly */
    };
    if (b < 0)                    b = 0;
    if (b > DENS_BAND_COUNT - 1)  b = DENS_BAND_COUNT - 1;
    return ramp[b];
}

/* The box drawn around the picture so the data sits in a clear window. */
#define SECTION_FRAME_CORNER     '+'
#define SECTION_FRAME_HORIZ      '-'
#define SECTION_FRAME_VERT       '|'

/* The faint centre cross-hairs, so you can find the middle and judge
 * symmetry.  Drawn first, then dots paint over them where they overlap. */
#define SECTION_AXIS_VERT        '|'
#define SECTION_AXIS_HORIZ       '-'
#define SECTION_AXIS_ORIGIN      '+'

/* Marks for the energy edge -- the farthest a star at this energy can reach.
 * The high mark sits at the top of its cell, the low mark at the bottom, so
 * they read as the rim of the allowed region rather than as data. */
#define SECTION_BOUNDARY_TOP     '\''
#define SECTION_BOUNDARY_BOTTOM  ','

/* PoincareViewport — the on-screen rectangle the picture is drawn in.  It
 * holds where the box starts and how big it is, worked out once per frame, so
 * every drawing helper takes this one thing instead of four loose numbers.
 *
 *   gx0, gy0 : top-left corner of the data area, in screen cells.
 *   w, h     : its width and height, matching the tally board.
 */
typedef struct {
    int gx0, gy0;
    int w, h;
} PoincareViewport;

/* Left edge that centres a width-w box on screen, leaving a cell for the
 * frame. */
static inline int viewport_centered_origin_x(int outer_cols, int w)
{
    int gx0 = (outer_cols - w) / 2;
    return (gx0 < 1) ? 1 : gx0;
}

/* Top edge that centres a height-h box in the area left between the two
 * status bars, leaving a cell for the frame. */
static inline int viewport_centered_origin_y_in_drawable(int outer_rows, int h)
{
    int gy0 = ((outer_rows - HUD_BAND_RESERVED_ROWS) - h) / 2 + HUD_TOP_ROWS;
    return (gy0 < HUD_TOP_ROWS + 1) ? HUD_TOP_ROWS + 1 : gy0;
}

/* Work out this frame's drawing box from the grid size and terminal size. */
static PoincareViewport poincare_viewport_build(const SectionGrid *section,
                                                int cols, int rows)
{
    PoincareViewport vp;
    vp.w   = section->w;
    vp.h   = section->h;
    vp.gx0 = viewport_centered_origin_x(cols, vp.w);
    vp.gy0 = viewport_centered_origin_y_in_drawable(rows, vp.h);
    return vp;
}

/* Turn a real (y, py) value into a column/row inside the box.  The actual
 * flip that puts higher momentum higher up happens in poincare_plot. */
static inline int phase_to_cell_x(const PoincareViewport *vp, float y_phys)
{
    int cx = (int)((y_phys - Y_MIN) / (Y_MAX - Y_MIN) * (float)vp->w);
    if (cx < 0)         cx = 0;
    if (cx >= vp->w)    cx = vp->w - 1;
    return cx;
}
static inline int phase_to_cell_y(const PoincareViewport *vp, float py_phys)
{
    int cy = (int)((py_phys - PY_MIN) / (PY_MAX - PY_MIN) * (float)vp->h);
    if (cy < 0)         cy = 0;
    if (cy >= vp->h)    cy = vp->h - 1;
    return cy;
}

/* Put one character at the screen spot for a real (y, py) value.  The only
 * place that flips the row, since the screen counts down but momentum counts
 * up, so higher momentum lands higher on screen. */
static inline void poincare_plot(const PoincareViewport *vp,
                                 float y_phys, float py_phys, chtype glyph)
{
    int cx = phase_to_cell_x(vp, y_phys);
    int cy = phase_to_cell_y(vp, py_phys);
    mvaddch(vp->gy0 + (vp->h - 1 - cy), vp->gx0 + cx, glyph);
}

/* Draw a full-height vertical line down one column of the box.  Caller picks
 * the colour first. */
static void viewport_paint_vertical_line(const PoincareViewport *vp,
                                         int cx_local, chtype glyph)
{
    for (int y = 0; y < vp->h; y++)
        mvaddch(vp->gy0 + (vp->h - 1 - y), vp->gx0 + cx_local, glyph);
}

/* Draw a full-width horizontal line across one row of the box. */
static void viewport_paint_horizontal_line(const PoincareViewport *vp,
                                           int cy_local, chtype glyph)
{
    for (int x = 0; x < vp->w; x++)
        mvaddch(vp->gy0 + (vp->h - 1 - cy_local), vp->gx0 + x, glyph);
}

/* The '-' edges just above and below the data. */
static void paint_frame_top_and_bottom_edges(const PoincareViewport *vp)
{
    int top_row    = vp->gy0 - 1;
    int bottom_row = vp->gy0 + vp->h;
    for (int x = 0; x < vp->w; x++) {
        mvaddch(top_row,    vp->gx0 + x, SECTION_FRAME_HORIZ);
        mvaddch(bottom_row, vp->gx0 + x, SECTION_FRAME_HORIZ);
    }
}

/* The '|' edges just left and right of the data. */
static void paint_frame_left_and_right_edges(const PoincareViewport *vp)
{
    int left_col  = vp->gx0 - 1;
    int right_col = vp->gx0 + vp->w;
    for (int y = 0; y < vp->h; y++) {
        mvaddch(vp->gy0 + y, left_col,  SECTION_FRAME_VERT);
        mvaddch(vp->gy0 + y, right_col, SECTION_FRAME_VERT);
    }
}

/* The '+' corners, drawn last so they win where the edges meet. */
static void paint_frame_corners(const PoincareViewport *vp)
{
    int top_row    = vp->gy0 - 1,    bottom_row = vp->gy0 + vp->h;
    int left_col   = vp->gx0 - 1,    right_col  = vp->gx0 + vp->w;
    mvaddch(top_row,    left_col,  SECTION_FRAME_CORNER);
    mvaddch(top_row,    right_col, SECTION_FRAME_CORNER);
    mvaddch(bottom_row, left_col,  SECTION_FRAME_CORNER);
    mvaddch(bottom_row, right_col, SECTION_FRAME_CORNER);
}

/* The box around the data: edges then corners, in a bright colour so it
 * frames the picture clearly. */
static void paint_section_frame(const PoincareViewport *vp)
{
    attron(COLOR_PAIR(PAIR_BOUNDARY) | A_BOLD);
    paint_frame_top_and_bottom_edges(vp);
    paint_frame_left_and_right_edges(vp);
    paint_frame_corners             (vp);
    attroff(COLOR_PAIR(PAIR_BOUNDARY) | A_BOLD);
}

/* The faint centre cross-hairs.  Drawn before the dots so the data sits on
 * top of them. */
static void paint_section_axes(const PoincareViewport *vp)
{
    int y_zero_col   = phase_to_cell_x(vp, 0.0f);
    int py_zero_row  = phase_to_cell_y(vp, 0.0f);
    int origin_screen_row = vp->gy0 + (vp->h - 1 - py_zero_row);

    attron(COLOR_PAIR(PAIR_BOUNDARY));
    viewport_paint_vertical_line  (vp, y_zero_col,  SECTION_AXIS_VERT);
    viewport_paint_horizontal_line(vp, py_zero_row, SECTION_AXIS_HORIZ);
    mvaddch(origin_screen_row, vp->gx0 + y_zero_col, SECTION_AXIS_ORIGIN);
    attroff(COLOR_PAIR(PAIR_BOUNDARY));
}

/* The biggest momentum a star at this height can have without breaking the
 * energy budget -- the rim of the allowed region.  Returns -1 if this height
 * is out of reach at all. */
static inline float energy_max_py_at_y(float y, float E)
{
    float V = hh_potential(0.0f, y);
    if (V > E) return -1.0f;
    return sqrtf(2.0f * (E - V));
}

/* 0.5 aims at the middle of a cell, not its left edge, so the energy rim
 * lines up with the dots instead of sitting half a cell to the side. */
#define CELL_CENTER_SAMPLE_FRACTION  0.5f

/* The real y value at the middle of a given column -- the reverse of turning
 * a y value into a column. */
static inline float viewport_cell_x_to_y_phys(const PoincareViewport *vp, int cx)
{
    return Y_MIN + ((float)cx + CELL_CENTER_SAMPLE_FRACTION) / (float)vp->w
                 * (Y_MAX - Y_MIN);
}

/* Trace the energy rim: for each column, mark the highest and lowest momentum
 * a star can reach there.  Columns out of reach get nothing. */
static void paint_energy_boundary(const PoincareViewport *vp, float E)
{
    attron(COLOR_PAIR(PAIR_BOUNDARY) | A_BOLD);
    for (int cx = 0; cx < vp->w; cx++) {
        float y_phys = viewport_cell_x_to_y_phys(vp, cx);

        float py_max = energy_max_py_at_y(y_phys, E);
        if (py_max < 0.0f) continue;

        poincare_plot(vp, y_phys, +py_max, SECTION_BOUNDARY_TOP);
        poincare_plot(vp, y_phys, -py_max, SECTION_BOUNDARY_BOTTOM);
    }
    attroff(COLOR_PAIR(PAIR_BOUNDARY) | A_BOLD);
}

/* Is this screen row in the drawing area, clear of the two status bars? */
static inline bool screen_row_inside_drawable(int sy, int rows)
{
    return sy >= HUD_TOP_ROWS && sy < rows - HUD_BOTTOM_ROWS;
}

/* Is this screen column actually on the terminal? */
static inline bool screen_col_inside_terminal(int sx, int cols)
{
    return sx >= 0 && sx < cols;
}

/* Draw one cell's dot, picking the character and colour from how often it was
 * hit.  Empty cells draw nothing, so the lines underneath show through. */
static void paint_density_cell(int sy, int sx, uint16_t hits)
{
    int band = density_band(hits);
    if (band < 0) return;
    int pair = PAIR_DENS_BASE + band;
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph_for_band(band));
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Draw the whole picture: walk every grid cell, skip any that fall off the
 * drawing area, and dot the rest by how often they were hit. */
static void paint_density_cells(const PoincareViewport *vp,
                                const SectionGrid *g,
                                int cols, int rows)
{
    for (int y = 0; y < vp->h; y++) {
        int sy = vp->gy0 + y;
        if (!screen_row_inside_drawable(sy, rows)) continue;

        for (int x = 0; x < vp->w; x++) {
            int sx = vp->gx0 + x;
            if (!screen_col_inside_terminal(sx, cols)) continue;

            paint_density_cell(sy, sx, g->hits[y * g->w + x]);
        }
    }
}

/* Draw the full picture, back to front: cross-hairs, then the energy rim,
 * then the dots on top, then the frame around it all.  Order matters so the
 * data isn't hidden by the guide marks. */
static void section_paint(const SectionGrid *section, int cols, int rows, float E)
{
    PoincareViewport vp = poincare_viewport_build(section, cols, rows);
    paint_section_axes   (&vp);
    paint_energy_boundary(&vp, E);
    paint_density_cells  (&vp, section, cols, rows);
    paint_section_frame  (&vp);
}

static void hud_paint_top_left_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " HÉNON-HEILES (Poincaré) ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Build the right-hand status text (fps, rate, preset, energy) into a buffer;
 * no drawing here, just the string. */
static void hud_format_top_right_status(char *buf, size_t bufsz,
                                        double fps, int sim_fps,
                                        const Scene *s)
{
    const HHPreset *active = preset_state_active(&s->preset);
    snprintf(buf, bufsz,
             " %5.1f fps  %3d Hz  %s [%d/%d]  E:%.3f ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : active->name,
             s->preset.current + 1, N_PRESETS,
             (double)active->E);
}

/* Draw text flush against the right edge of row 0, clamped so a narrow
 * terminal can't push it off the left side. */
static void hud_paint_top_right(int cols, const char *text)
{
    int anchor_col = cols - (int)strlen(text);
    if (anchor_col < 0) anchor_col = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, anchor_col, "%s", text);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Top status bar: title on the left, live status on the right. */
static void hud_top(int cols, double fps, int sim_fps, const Scene *s)
{
    hud_paint_top_left_title();

    char buf[HUD_COLS + 1];
    hud_format_top_right_status(buf, sizeof buf, fps, sim_fps, s);
    hud_paint_top_right(cols, buf);
}

/* Fixed widths of the second-row labels, so the layout doesn't jump around as
 * the preset and theme names change length. */
#define HUD_PARAM_CELL_WIDTH_PRESET   19   /* " preset:XXXXXXXX " */
#define HUD_PARAM_CELL_WIDTH_THEME    17   /* " theme:XXXXXXXX "  */

/* Draw one labelled status item on row 1.  fmt has one %s for the value. */
static void hud_paint_param_cell_bold(int cursor_x, const char *fmt, const char *value)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, cursor_x, fmt, value);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Second status bar: the current preset, theme, and a few fixed facts. */
static void hud_param(const Scene *s)
{
    int cursor_x = HUD_LEFT_MARGIN;

    hud_paint_param_cell_bold(cursor_x, " preset:%-8s ",
                              preset_state_active(&s->preset)->name);
    cursor_x += HUD_PARAM_CELL_WIDTH_PRESET;

    hud_paint_param_cell_bold(cursor_x, " theme:%-8s ",
                              palette_state_active(&s->palette)->name);
    cursor_x += HUD_PARAM_CELL_WIDTH_THEME;

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cursor_x, " section:x=0 (ẋ>0)   trajs:%d   E_escape≈0.1667 ",
             s->ensemble.count);
    attroff(COLOR_PAIR(PAIR_HUD));
}

static void hud_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:preset  t/T:theme  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    section_paint(&s->section, sc->cols, sc->rows,
                  preset_state_active_energy(&s->preset));
    hud_top  (sc->cols, fps, sim_fps, s);
    hud_param(s);
    hud_hint (sc->rows);
}

/* §10 app */

/*
 * App — the one box holding everything the program owns: the simulation, the
 * terminal handle, the tick rate, the drawing size, and two flags the signal
 * handlers set.  Nothing else is global, so the program reads top-down:
 * main() drives the App, and each helper does one named thing to it.
 *
 *   scene       : the simulation -- what you see.
 *   screen      : the terminal size + ncurses handle.
 *   sim_fps     : how many physics steps per second; the ]/[ keys change it.
 *                 Separate from the 60-fps drawing rate, so the maths can run
 *                 faster or slower than the screen refreshes.
 *   map_w,map_h : drawing size in cells, recomputed when the window resizes.
 *   running     : clear it and the main loop exits (set by Ctrl-C or q).
 *   need_resize : set when the window changes size, handled next frame.
 *
 * running and need_resize are written from signal handlers, so they're the
 * special volatile sig_atomic_t type that's safe to touch there. */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   map_w, map_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

/* The one global, only because signal handlers can't be handed an argument
 * and so need something file-wide to reach. */
static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* The frame eats one cell on each side, so two off each dimension. */
#define SECTION_FRAME_RESERVED  2

static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* Pick the drawing size: terminal minus the status bars and frame, kept
 * within sane min and max bounds. */
static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols - SECTION_FRAME_RESERVED;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS - SECTION_FRAME_RESERVED;
    if (mw < 16)        mw = 16;
    if (mh < 8)         mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

static void app_bootstrap(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);
}

/* Rebuild the screen and restart the picture after the window resized. */
static void app_handle_pending_resize(App *app)
{
    if (!app->need_resize) return;
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* Time since the last frame, capped so a long pause can't dump a huge
 * backlog of physics steps all at once when we resume. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > SIM_MAX_FRAME_DT_MS * NS_PER_MS)
        dt = SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    return dt;
}

/* Run the physics in fixed-size steps, one per tick of saved-up time, until
 * less than a full step is left over.  Using a constant step (not whatever
 * gap the frame happened to take) keeps the picture identical on a fast or
 * slow machine. */
static void app_drain_fixed_timestep(App *app, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* Refresh the displayed fps number a couple of times a second, not every
 * frame, so it stays readable instead of flickering. */
static void app_update_fps_meter(int64_t *fps_accum, int *frame_count,
                                 double *fps_display)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_display = (double)*frame_count
                 / ((double)*fps_accum / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
}

/* Sleep off the rest of this frame's time budget so we hold a steady rate.
 * Done before drawing so the time spent writing to the terminal doesn't add
 * jitter. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* Speed the physics up or down, staying within the allowed range. */
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

/* Tiny named actions so the key table below reads like plain commands. */
static void app_toggle_pause   (App *app) { app->scene.paused = !app->scene.paused; }
static void app_reset_ensemble (App *app) { scene_reset(&app->scene, app->map_w, app->map_h); }

static void app_cycle_preset_next(App *app)
{
    preset_state_cycle_next(&app->scene.preset);
    scene_reset(&app->scene, app->map_w, app->map_h);
}
static void app_cycle_preset_prev(App *app)
{
    preset_state_cycle_prev(&app->scene.preset);
    scene_reset(&app->scene, app->map_w, app->map_h);
}

static void app_cycle_theme_next(App *app)
{
    palette_state_cycle_next(&app->scene.palette);
    scene_apply_theme(&app->scene);
}
static void app_cycle_theme_prev(App *app)
{
    palette_state_cycle_prev(&app->scene.palette);
    scene_apply_theme(&app->scene);
}

static bool app_handle_key(App *app, int ch);

/* Check for a keypress without waiting; returns false only on quit. */
static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* The keymap: each key runs one action.  Returns false to quit. */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            app_toggle_pause     (app); break;
    case 'r': case 'R':  app_reset_ensemble   (app); break;
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
 * FrameClock — the timekeeping the main loop carries, gathered in one place.
 * It tracks two separate clocks: a wall clock for what the screen shows, and
 * a running tally of time the physics still owes.
 *
 *   frame_time  : when the last frame happened, to measure the gap since.
 *   sim_accum   : leftover time not yet spent on a physics step.
 *   fps_accum   : time piled up since the last fps readout.
 *   frame_count : frames counted in the current fps window.
 *   fps_display : the fps number currently shown.
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

/* The whole program at a glance: set up, then each frame catch up the
 * physics, hold the frame rate, draw, and check for a keypress, until we're
 * told to quit. */
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
