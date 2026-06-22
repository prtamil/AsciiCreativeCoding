/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * rossler_attractor.c — the Rössler attractor, Lorenz's simpler cousin: a
 * 3-D system of equations with just one curved term that still traces out a
 * thin, folded "strange attractor" you can watch evolve. We solve the
 * equations step by step and draw the moving point with a fading trail, with
 * 16 presets that walk from a still dot up to full chaos and a few odd shapes.
 *
 * Original equations: Rössler 1976, "An equation for continuous chaos".
 * Sister demos: ./strange_attractor.c (Lorenz + 9 others),
 *               ./poincare_section.c (Lorenz seen as a 2-D return map).
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* §1  config — constants, the 16-preset table, and the colour themes */

enum {
    SIM_FPS_MIN      =  10, SIM_FPS_DEFAULT = 60, SIM_FPS_MAX = 240, SIM_FPS_STEP = 10,
    HUD_COLS         =  80, FPS_UPDATE_MS = 500,
    PAIR_HUD         =   1, PAIR_HINT = 2,
    PAIR_TRAIL_BASE  =   3,    /* first of the 4 trail-fade colours */
    PAIR_LIVE        =   7,
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

#define ROSS_DT                   0.01f
#define INT_STEPS_PER_TICK         15
#define TRAIL_MAX                3000
#define TRAIL_BAND_COUNT            4

/* Camera */
#define CAM_PITCH                 0.45f      /* fixed tilt above the scene (rad) */
#define CAM_YAW_RATE              0.12f      /* how fast the camera orbits (rad/s) */

/* How big to draw the attractor: derived from the terminal size and the
 * current preset's rough radius so it fills the screen at any window size. */
#define ROSS_FIT_MARGIN            0.85f    /* leave a little border */
#define ROSS_SCALE_MIN             0.45f    /* don't shrink below this on tiny windows */
#define CAM_FIT_DRAWABLE_HEIGHT_MIN  4      /* floor so the fit math survives tiny windows */
#define CAM_FIT_EXTENT_MIN         1.0f    /* keep the radius non-zero so we never divide by 0 */
#define CAM_HALF_FRACTION          0.5f    /* half of the viewport, i.e. the centre */

/* Squash the up-down (z) axis before drawing. In the screw presets z can
 * reach ~80, which would shoot the shape way off the top of the screen;
 * halving it keeps it on screen. Same trick as ./strange_attractor.c. */
#define Z_SQUASH_FACTOR            0.5f

/* For showing the camera angle in degrees on the HUD. */
#define RAD_TO_DEG                (180.0f / (float)M_PI)

/*
 * Preset — names for the 16 rows of the presets[] table below.
 *
 * They're just labelled slot numbers, ordered from boring to wild so pressing
 * 'n' walks you through the whole story: a still point, then loops that double
 * in length again and again, then chaos, then a couple of odd shapes. CYCLE_3
 * is slipped in mid-chaos on purpose — a brief calm island in the middle of
 * the storm. PRESET_CHAOS is the classic Rössler setting and the one we boot
 * into. N_PRESETS is the count, used to wrap around when cycling.
 */
typedef enum {
    /* still point / decaying spiral */
    PRESET_STILL = 0,
    PRESET_SPIRAL,
    /* loops that keep doubling in length */
    PRESET_CYCLE_1,
    PRESET_CYCLE_2,
    PRESET_CYCLE_4,
    PRESET_CYCLE_8,
    /* the edge of chaos, then chaos */
    PRESET_ONSET,
    PRESET_CHAOS_S,
    PRESET_CHAOS,            /* classic Rössler — the one we boot into */
    PRESET_CHAOS_L,
    /* a calm 3-loop island in the middle of chaos */
    PRESET_CYCLE_3,
    /* wider chaos */
    PRESET_CHAOS_XL,
    PRESET_BANDS,
    /* odd shapes: trumpet and screw */
    PRESET_FUNNEL,
    PRESET_SCREW,
    PRESET_SCREW_L,
    N_PRESETS,
} Preset;

/*
 * RossPreset — one row of the preset menu: a name, the three dials that shape
 * the attractor, and a rough size used to fit the camera.
 *
 * Cycling through these rows is the only thing that changes what you see. Each
 * row is read in three places: §5 turns (a, b, c) into the live equations, §8
 * uses `extent` to size the camera, and §9 prints the name and dials on the HUD.
 *
 *   name   : short label for the HUD, padded so columns line up.
 *   a, b, c: the three dials of the Rössler equations. Turning c alone walks
 *            you from a single loop up through chaos; FUNNEL and SCREW also
 *            nudge a and b to get their different shapes.
 *   extent : roughly how far the shape reaches from its centre (in the same
 *            squashed z units the renderer uses). Hand-tuned per row so every
 *            preset ends up about the same size on screen — small loops fill a
 *            few cells, the big screw fills dozens.
 *
 * Equations and classic values: Rössler 1976. Funnel/screw regimes follow
 * Letellier, Dutertre & Maheu 1995.
 */
typedef struct { const char *name; float a, b, c; float extent; } RossPreset;
static const RossPreset presets[N_PRESETS] = {
    { "STILL   ", 0.20f, 0.20f,  1.50f,  3.0f },
    { "SPIRAL  ", 0.20f, 0.20f,  2.50f,  4.0f },
    { "CYCLE_1 ", 0.20f, 0.20f,  3.00f,  5.0f },
    { "CYCLE_2 ", 0.20f, 0.20f,  4.00f,  6.0f },
    { "CYCLE_4 ", 0.20f, 0.20f,  4.55f,  7.0f },
    { "CYCLE_8 ", 0.20f, 0.20f,  4.66f,  8.0f },
    { "ONSET   ", 0.20f, 0.20f,  4.72f,  9.0f },
    { "CHAOS_S ", 0.20f, 0.20f,  5.00f, 11.0f },
    { "CHAOS   ", 0.20f, 0.20f,  5.70f, 14.0f },
    { "CHAOS_L ", 0.20f, 0.20f,  6.00f, 16.0f },
    { "CYCLE_3 ", 0.20f, 0.20f,  6.30f, 16.0f },
    { "CHAOS_XL", 0.20f, 0.20f,  7.00f, 20.0f },
    { "BANDS   ", 0.20f, 0.20f,  8.00f, 24.0f },
    { "FUNNEL  ", 0.32f, 0.30f,  4.50f, 12.0f },   /* lopsided trumpet  */
    { "SCREW   ", 0.10f, 0.10f, 14.00f, 30.0f },   /* tall screw shape  */
    { "SCREW_L ", 0.10f, 0.10f, 18.00f, 38.0f },   /* wider screw       */
};

/*
 * Theme — one colour scheme for the drawing. The t/T keys cycle through them.
 *
 *   name  : short label for the HUD.
 *   band[]: the four trail colours, darkest first. The oldest part of the
 *           trail uses the darkest; the freshest uses the brightest, so the
 *           streak looks like it's fading away behind the point.
 *   live  : colour of the '@' that marks where the point is right now —
 *           usually a hot colour so it stands out against its own trail.
 *
 * All colours sit in the bright half of the palette so the dots stay visible
 * on a black terminal (CLAUDE.md "Theme Palette Brightness").
 */
typedef struct { const char *name; short band[TRAIL_BAND_COUNT]; short live; } Theme;
#define N_THEMES 10
static const Theme themes[N_THEMES] = {
    { "DEFAULT", {  75, 123, 220, 231 }, 196 },
    { "MATRIX",  {  77, 118, 156, 194 }, 226 },
    { "NOVA",    { 135, 171, 207, 219 }, 226 },
    { "MONO",    { 247, 250, 253, 255 }, 226 },
    { "OCEAN",   {  81, 117, 159, 195 }, 226 },
    { "FIRE",    { 208, 214, 220, 227 }, 231 },
    { "EARTH",   { 143, 179, 215, 222 }, 196 },
    { "FOREST",  { 114, 150, 157, 194 }, 226 },
    { "DESERT",  { 179, 215, 222, 229 }, 196 },
    { "ARCTIC",  { 117, 159, 195, 231 }, 196 },
};

/* §2 clock + §3 color */

static int64_t clock_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec; }
static void clock_sleep_ns(int64_t ns) { if (ns <= 0) return;
    struct timespec req = {ns/NS_PER_SEC, ns%NS_PER_SEC}; nanosleep(&req, NULL); }

static inline void theme_install_256(const Theme *t)
{
    for (int i = 0; i < TRAIL_BAND_COUNT; i++)
        init_pair(PAIR_TRAIL_BASE + i, t->band[i], -1);
    init_pair(PAIR_LIVE, t->live, -1);
}

/* fallback for old 8-colour terminals: trail goes cyan, point goes red. */
static inline void theme_install_8color_fallback(void)
{
    for (int i = 0; i < TRAIL_BAND_COUNT; i++)
        init_pair(PAIR_TRAIL_BASE + i, COLOR_CYAN, -1);
    init_pair(PAIR_LIVE, COLOR_RED, -1);
}

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_install_256(&themes[idx]);
    else               theme_install_8color_fallback();
}
static void color_init(void)
{ start_color(); use_default_colors();
  if (COLORS >= 256) { init_pair(PAIR_HUD, 226, -1); init_pair(PAIR_HINT, 51, -1); }
  else { init_pair(PAIR_HUD, COLOR_YELLOW, -1); init_pair(PAIR_HINT, COLOR_CYAN, -1); }
  theme_apply(0); }

/* §4  camera — turn the 3-D point into a screen position */

/*
 * Camera — where we're looking at the 3-D shape from. We only need three
 * numbers, not a full 3-D camera, because the view is deliberately simple.
 *
 *   yaw   : how far the camera has spun around the shape (radians). It keeps
 *           creeping up so you slowly see the shape from every side.
 *   pitch : how high above the shape we're looking down from. Held fixed —
 *           letting it change too made the spin feel dizzying at this
 *           chunky character resolution.
 *   scale : how many screen cells one world unit becomes. Recomputed whenever
 *           the window resizes or the preset changes so the shape stays
 *           nicely sized.
 *
 * The rotation + flatten-to-screen math is the standard textbook approach
 * (Foley & van Dam 1996, ch. 5-6).
 */
typedef struct { float yaw, pitch, scale; } Camera;

#define CELL_ASPECT 0.5f   /* a character cell is about twice as tall as wide */

/* usable rows once the HUD (top + bottom) is taken out, never less than a
 * tiny floor so the sizing math below doesn't break on a squashed window. */
static inline int screen_drawable_height(int rows)
{
    int draw_h = rows - HUD_BAND_RESERVED_ROWS;
    if (draw_h < CAM_FIT_DRAWABLE_HEIGHT_MIN) draw_h = CAM_FIT_DRAWABLE_HEIGHT_MIN;
    return draw_h;
}

/* biggest scale that still fits a shape of this radius across the width. */
static inline float horizontal_fit_scale(int cols, float extent, float margin)
{
    float half_x_cells = (float)cols * CAM_HALF_FRACTION;
    return (half_x_cells * margin) / extent;
}

/* same idea for the height; the aspect term accounts for tall, skinny cells. */
static inline float vertical_fit_scale(int rows, float extent, float margin, float aspect)
{
    int   draw_h       = screen_drawable_height(rows);
    float half_y_cells = (float)draw_h * CAM_HALF_FRACTION;
    return (half_y_cells * margin) / (extent * aspect);
}

/* pick the scale that makes the shape fill the window: try both width and
 * height, keep the smaller (so it fits both ways), but never go below a floor. */
static void camera_fit_to_screen(Camera *cam, int cols, int rows, float extent)
{
    if (extent < CAM_FIT_EXTENT_MIN) extent = CAM_FIT_EXTENT_MIN;

    float sx = horizontal_fit_scale(cols, extent, ROSS_FIT_MARGIN);
    float sy = vertical_fit_scale  (rows, extent, ROSS_FIT_MARGIN, CELL_ASPECT);

    float scale = (sx < sy) ? sx : sy;
    if (scale < ROSS_SCALE_MIN) scale = ROSS_SCALE_MIN;
    cam->scale = scale;
}

/* spin the point around the up-down axis by the camera's yaw — this is what
 * lets us see the shape from different sides. Height (z) is untouched. */
static inline void rotate_about_z_yaw(float x, float y,
                                      float cos_yaw, float sin_yaw,
                                      float *xr, float *yr)
{
    *xr = x * cos_yaw - y * sin_yaw;
    *yr = x * sin_yaw + y * cos_yaw;
}

/* tilt the point so we look down on it from above. Only the vertical part
 * changes, so that's all we return. */
static inline float rotate_about_x_pitch(float yr, float z,
                                         float cos_pitch, float sin_pitch)
{
    return yr * cos_pitch - z * sin_pitch;
}

/* last step: forget depth and land on an actual screen cell. We scale to the
 * right size, flip the y-axis (screen rows count downward, the world counts
 * up), and squeeze vertically so tall cells don't stretch the picture. */
static inline void orthographic_to_cell(float xr_world, float yr2_world,
                                        float scale, float aspect,
                                        int cx, int cy, int *sx, int *sy)
{
    *sx = cx + (int)( xr_world  * scale);
    *sy = cy + (int)(-yr2_world * scale * aspect);
}

/* turn a 3-D world point into a screen cell: spin it, tilt it, flatten it. */
static void project(const Camera *cam,
                    float x, float y, float z,
                    int cx_centre, int cy_centre, float aspect,
                    int *sx, int *sy)
{
    float cos_yaw  = cosf(cam->yaw),   sin_yaw   = sinf(cam->yaw);
    float cos_pitch = cosf(cam->pitch), sin_pitch = sinf(cam->pitch);

    float xr, yr;
    rotate_about_z_yaw(x, y, cos_yaw, sin_yaw, &xr, &yr);
    float yr2 = rotate_about_x_pitch(yr, z, cos_pitch, sin_pitch);
    orthographic_to_cell(xr, yr2, cam->scale, aspect, cx_centre, cy_centre, sx, sy);
}

/* §5  the Rössler equations and the solver that steps them forward */

/*
 * Vec3 — three numbers (x, y, z) bundled together. Used as the point's
 * position, as the direction it's moving (its slope), and as a stored trail
 * sample. Passed around by value so the solver can add them up cheaply.
 */
typedef struct { float x, y, z; } Vec3;

/* Just a Vec3, but the name says "this one is the point's current position"
 * so the solver code reads as physics rather than generic vector math. */
typedef Vec3 RosslerState;

/*
 * RosslerSystem — the three dials that shape the motion. Kept apart from the
 * moving point so the equations only depend on (point, dials); switching
 * presets just swaps these dials and restarts the point.
 *
 *   a : how fast the point spirals outward. Usually 0.2.
 *   b : a steady push on the height so it never just dies down to zero.
 *   c : the trip-wire — once the point's x passes c, the height suddenly
 *       shoots up and yanks it back inward. Slowly raising c is what walks
 *       the shape from a single loop, to a doubled loop, and on into chaos.
 *       The big values (14, 18) give the screw shapes.
 *
 * Equations and dial names: Rössler 1976. How c drives the doubling:
 * Strogatz, Nonlinear Dynamics and Chaos, §10.6.
 */
typedef struct { float a, b, c; } RosslerSystem;

/*
 * Rossler — the dials plus the point's position, bundled so the solver takes
 * one argument. The solver moves `state` and only reads `system`.
 */
typedef struct {
    RosslerSystem system;
    RosslerState  state;
} Rossler;

/* Given where the point is, work out which way (and how fast) it's heading —
 * the three Rössler equations, one per line. The single curved term, z·x in
 * the last line, is the whole trick: it's what makes the shape fold over on
 * itself and turn chaotic. */
static inline RosslerState rossler_deriv(const RosslerState *s,
                                         const RosslerSystem *sys)
{
    RosslerState dy;
    dy.x = -s->y - s->z;
    dy.y =  s->x + sys->a * s->y;
    dy.z =  sys->b + s->z * (s->x - sys->c);
    return dy;
}

/* take a step of size h in a given direction from a point — "where would I
 * be if I moved this way for this long?" */
static inline RosslerState state_add(const RosslerState *a,
                                     float h, const RosslerState *k)
{
    RosslerState r;
    r.x = a->x + h * k->x;
    r.y = a->y + h * k->y;
    r.z = a->z + h * k->z;
    return r;
}

#define RK4_BUTCHER_WEIGHT_SUM   6.0f
#define RK4_MIDPOINT_FRACTION    0.5f

/* blend the four trial directions into one, trusting the two middle ones most
 * — the recipe that makes this solver far more accurate than a naive step. */
static inline RosslerState rk4_butcher_weighted_average(
    const RosslerState *k1, const RosslerState *k2,
    const RosslerState *k3, const RosslerState *k4)
{
    RosslerState avg;
    avg.x = (k1->x + 2.0f*k2->x + 2.0f*k3->x + k4->x) / RK4_BUTCHER_WEIGHT_SUM;
    avg.y = (k1->y + 2.0f*k2->y + 2.0f*k3->y + k4->y) / RK4_BUTCHER_WEIGHT_SUM;
    avg.z = (k1->z + 2.0f*k2->z + 2.0f*k3->z + k4->z) / RK4_BUTCHER_WEIGHT_SUM;
    return avg;
}

/* move the point forward one small time step. Instead of trusting a single
 * direction, it samples the direction four times (start, twice in the middle,
 * and end), then blends them — the classic Runge-Kutta method that keeps the
 * curve accurate (Numerical Recipes §17.1). The named locals below read like
 * the textbook recipe top to bottom. */
static void rossler_rk4_step(Rossler *r, float dt)
{
    const RosslerSystem *sys = &r->system;
    float half_dt = RK4_MIDPOINT_FRACTION * dt;

    RosslerState slope_start    = rossler_deriv(&r->state, sys);
    RosslerState midpoint_1     = state_add(&r->state, half_dt, &slope_start);
    RosslerState slope_mid_1    = rossler_deriv(&midpoint_1, sys);
    RosslerState midpoint_2     = state_add(&r->state, half_dt, &slope_mid_1);
    RosslerState slope_mid_2    = rossler_deriv(&midpoint_2, sys);
    RosslerState endpoint       = state_add(&r->state, dt, &slope_mid_2);
    RosslerState slope_end      = rossler_deriv(&endpoint, sys);

    RosslerState effective_slope = rk4_butcher_weighted_average(
        &slope_start, &slope_mid_1, &slope_mid_2, &slope_end);
    r->state = state_add(&r->state, dt, &effective_slope);
}

/* pull the three dials out of a preset row. */
static inline RosslerSystem rossler_system_from_preset(const RossPreset *p)
{
    return (RosslerSystem){ p->a, p->b, p->c };
}

/* §6  trail — the fading streak of recent positions */

/*
 * Trail — the last few thousand positions, kept so we can draw the streak.
 *
 * It's a fixed-size loop of slots (a ring buffer): once full, the newest
 * position overwrites the oldest, so the streak stays a constant length no
 * matter how long the program runs. No memory is allocated at runtime — it's
 * one big static block. We store the three axes as separate arrays because
 * the drawing code walks all the x's, then all the y's, which is friendlier
 * to the cache that way.
 *
 *   x[], y[], z[] : the stored positions, one array per axis.
 *   head          : slot holding the newest position; steps forward (and
 *                   wraps around) each time we add one.
 *   count         : how many slots are filled, stopping at full. It fills up
 *                   after roughly 3 seconds.
 *
 * Wiped whenever we reset, change preset, or resize so an old shape doesn't
 * linger as a ghost behind the new one.
 */
typedef struct {
    float x[TRAIL_MAX], y[TRAIL_MAX], z[TRAIL_MAX];
    int   head, count;
} Trail;

static void trail_reset(Trail *t) { t->head = 0; t->count = 0; }
static void trail_push(Trail *t, Vec3 p)
{
    t->head = (t->head + 1) % TRAIL_MAX;
    t->x[t->head] = p.x;
    t->y[t->head] = p.y;
    t->z[t->head] = p.z;
    if (t->count < TRAIL_MAX) t->count++;
}

/* slot holding the oldest position still kept. The extra TRAIL_MAX keeps the
 * number positive before the wrap, since C's % can go negative. */
static inline int trail_oldest_index(const Trail *t)
{
    return (t->head - t->count + 1 + TRAIL_MAX) % TRAIL_MAX;
}

/* the i-th position counting from the oldest, so callers can just walk
 * 0..count-1 and not worry about where the ring wraps. */
static inline Vec3 trail_sample_at(const Trail *t, int i_from_oldest)
{
    int idx = (trail_oldest_index(t) + i_from_oldest) % TRAIL_MAX;
    return (Vec3){ t->x[idx], t->y[idx], t->z[idx] };
}

/* pick which of the four fade colours a position gets from its age: fresh
 * ones get the bright end, old ones the dim end. */
static inline int trail_age_band(int age_from_newest, int n)
{
    int band = (TRAIL_BAND_COUNT - 1) - (age_from_newest * TRAIL_BAND_COUNT) / n;
    if (band < 0)                     band = 0;
    if (band > TRAIL_BAND_COUNT - 1)  band = TRAIL_BAND_COUNT - 1;
    return band;
}

/* squash the height before drawing so tall screw shapes don't run off screen.
 * Both the trail and the live point go through here so they squash the same. */
static inline Vec3 world_to_view_space(Vec3 world)
{
    return (Vec3){ world.x, world.y, world.z * Z_SQUASH_FACTOR };
}

/* §7  state — small wrappers around the two things you can cycle */

/*
 * PresetState — remembers which preset is showing. It's just an index, but
 * wrapping it in its own type means the "next theme" key literally can't
 * reach in and change the preset by mistake; each cycler only touches its own.
 * `current` always stays a valid row by wrapping around at the ends.
 */
typedef struct { int current; } PresetState;

static void preset_state_init      (PresetState *p, int initial) { p->current = initial; }
static void preset_state_cycle_next(PresetState *p)              { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)              { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const RossPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }

/* Same idea as PresetState but for the colour theme — a distinct type so the
 * theme and preset keys can never get crossed. */
typedef struct { int current; } PaletteState;

static void palette_state_init      (PaletteState *p, int initial) { p->current = initial; }
static void palette_state_cycle_next(PaletteState *p)              { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p)              { p->current = (p->current + N_THEMES - 1) % N_THEMES; }
static const Theme *palette_state_active(const PaletteState *p)    { return &themes[p->current]; }
static void palette_state_apply     (const PaletteState *p)        { theme_apply(p->current); }

/* §8  scene — everything that changes as the demo runs, in one place */

/*
 * Scene — all the moving parts gathered into one struct, so the rest of the
 * program drives the whole demo through a handful of scene_* calls.
 *
 *   rossler  : the equations and the moving point (§5).
 *   trail    : the fading streak (§6).
 *   cam      : the slowly orbiting view (§4).
 *   preset   : which preset is showing (§7).
 *   palette  : which colour theme is showing (§7).
 *   paused   : when true, the point freezes and the streak holds still.
 *   cols/rows: the window size, kept here so changing preset can re-fit the
 *              camera without having to ask the screen again.
 */
typedef struct {
    Rossler      rossler;
    Trail        trail;
    Camera       cam;
    PresetState  preset;
    PaletteState palette;
    bool         paused;
    int          cols, rows;
} Scene;

/* short-hands so callers don't keep digging out the current preset/theme. */
static inline const RossPreset *scene_active_preset(const Scene *s)
{
    return preset_state_active(&s->preset);
}
static inline RosslerSystem scene_active_system(const Scene *s)
{
    return rossler_system_from_preset(scene_active_preset(s));
}
static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

/* switch to the current preset: load its dials, drop the point back to the
 * (1,1,1) start, and wipe the streak so the old shape doesn't linger. */
static void scene_apply_preset(Scene *s)
{
    s->rossler.system = scene_active_system(s);
    s->rossler.state  = (RosslerState){ 1.0f, 1.0f, 1.0f };
    trail_reset(&s->trail);
}

static void scene_reset(Scene *s) { scene_apply_preset(s); }

/* re-fit the camera to the current window and the current preset's size, so
 * whatever's showing fills the screen. Run on start, resize, and preset change. */
static void scene_compute_geometry(Scene *s)
{
    camera_fit_to_screen(&s->cam, s->cols, s->rows,
                         scene_active_preset(s)->extent);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->cols      = cols;
    s->rows      = rows;
    preset_state_init (&s->preset,  PRESET_CHAOS);   /* boot into classic chaos */
    palette_state_init(&s->palette, 0);              /* first theme */
    s->cam.yaw   = 0.0f;
    s->cam.pitch = CAM_PITCH;
    scene_compute_geometry(s);   /* size the camera to the window */
    scene_reset(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
    scene_compute_geometry(s);   /* re-fit to the new window size */
    scene_reset(s);              /* wipe the streak so it redraws cleanly */
}

/* take several tiny solver steps per frame, saving each into the trail. Doing
 * many small steps keeps the curve smooth even when frames are far apart. */
static inline void rossler_advance_substeps(Rossler *r, Trail *trail)
{
    for (int i = 0; i < INT_STEPS_PER_TICK; i++) {
        rossler_rk4_step(r, ROSS_DT);
        trail_push(trail, r->state);
    }
}

/* nudge the camera a little further around the shape this frame. */
static inline void camera_orbit(Camera *cam, float dt)
{
    cam->yaw += CAM_YAW_RATE * dt;
}

/* one step of the world: move the point along its path and turn the camera. */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    rossler_advance_substeps(&s->rossler, &s->trail);
    camera_orbit(&s->cam, dt);
}

/* §9  screen — draw the attractor and the HUD */

/*
 * Screen — the current window size, kept in one spot. ncurses exposes the
 * size as global macros; copying them here gives the drawing code one tidy
 * handle and one thing to update on a resize.
 */
typedef struct { int cols, rows; } Screen;
static void screen_init(Screen *s) { initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(); getmaxyx(stdscr, s->rows, s->cols); }
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols); }
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* the middle of the area we're allowed to draw in — the centre of the rows
 * between the top and bottom HUD lines, so the picture never sits under them. */
static inline void viewport_center_in_drawable_band(int cols, int rows,
                                                    int *cx, int *cy)
{
    *cx = cols / 2;
    *cy = (rows - HUD_BAND_RESERVED_ROWS) / 2 + HUD_TOP_ROWS;
}

/* true if this cell is on screen and clear of the HUD rows; both painters use
 * it to skip anything they shouldn't draw. */
static inline bool cell_in_drawable_band(int sx, int sy, int cols, int rows)
{
    return sx >= 0 && sx < cols
        && sy >= HUD_TOP_ROWS && sy < rows - HUD_BOTTOM_ROWS;
}

/* draw one coloured character. The cast avoids an ncurses gotcha where a
 * byte above 127 gets misread as a control code (CLAUDE.md ncurses bugs). */
static inline void paint_cell(int sy, int sx, char glyph, short pair_id)
{
    attron(COLOR_PAIR(pair_id) | A_BOLD);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair_id) | A_BOLD);
}

/* project an already-squashed point to a cell, so both painters call it the
 * same short way. */
static inline void project_view_to_cell(const Camera *cam, Vec3 v,
                                        int cx, int cy, int *sx, int *sy)
{
    project(cam, v.x, v.y, v.z, cx, cy, CELL_ASPECT, sx, sy);
}

/* draw the streak: every stored position becomes a '.', coloured by how old
 * it is so the trail looks like it's fading out behind the point. */
static void paint_trail(const Trail *tr, const Camera *cam,
                        int cx, int cy, int cols, int rows)
{
    if (tr->count == 0) return;
    int n = tr->count;

    for (int i = 0; i < n; i++) {
        Vec3 sample_view = world_to_view_space(trail_sample_at(tr, i));
        int  sx, sy;
        project_view_to_cell(cam, sample_view, cx, cy, &sx, &sy);
        if (!cell_in_drawable_band(sx, sy, cols, rows)) continue;

        int  age_from_newest = n - 1 - i;
        int  band            = trail_age_band(age_from_newest, n);
        paint_cell(sy, sx, '.', (short)(PAIR_TRAIL_BASE + band));
    }
}

/* draw the whole picture: the fading streak first, then the '@' on top so you
 * can always see exactly where the point is right now. */
static void scene_paint(const Scene *s, int cols, int rows)
{
    int cx, cy;
    viewport_center_in_drawable_band(cols, rows, &cx, &cy);

    paint_trail(&s->trail, &s->cam, cx, cy, cols, rows);

    Vec3 live_view = world_to_view_space(s->rossler.state);
    int sx, sy;
    project_view_to_cell(&s->cam, live_view, cx, cy, &sx, &sy);
    if (cell_in_drawable_band(sx, sy, cols, rows))
        paint_cell(sy, sx, '@', PAIR_LIVE);
}

/* column widths for the row-1 labels so they stay lined up as names change. */
#define HUD_PARAM_PRESET_WIDTH  19
#define HUD_PARAM_THEME_WIDTH   17

static inline void hud_write_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " RÖSSLER ATTRACTOR ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* right side of the top line: frame rate, step rate, preset name, and c. */
static inline void hud_write_status_right(int cols, double fps, int sim_fps,
                                          const Scene *s)
{
    const RossPreset *active = scene_active_preset(s);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  c:%.2f ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : active->name,
             s->preset.current + 1, (int)N_PRESETS,
             (double)active->c);

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

/* the three labelled columns of row 1, split out so hud_param reads like a
 * layout rather than a stack of printf calls. */
static inline void hud_write_preset_label(int x, const RossPreset *active)
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
static inline void hud_write_abc_yaw(int x, const RossPreset *active,
                                     float yaw_radians)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " a:%.2f b:%.2f  yaw:%.1f° ",
             (double)active->a, (double)active->b,
             (double)(yaw_radians * RAD_TO_DEG));
    attroff(COLOR_PAIR(PAIR_HUD));
}

static void hud_param(const Scene *s)
{
    const RossPreset *active = scene_active_preset(s);
    int x = HUD_LEFT_MARGIN;

    hud_write_preset_label(x, active); x += HUD_PARAM_PRESET_WIDTH;
    hud_write_theme_label (x, s);      x += HUD_PARAM_THEME_WIDTH;
    hud_write_abc_yaw     (x, active, s->cam.yaw);
}
static void hud_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:preset  t/T:theme  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{ erase(); scene_paint(s, sc->cols, sc->rows);
  hud_top(sc->cols, fps, sim_fps, s); hud_param(s); hud_hint(sc->rows); }

/* §10 app — signals, resize, keys, timing, and the main loop */

/*
 * App — the whole program in one struct: the simulation, the window, the
 * speed knob, and two flags the signal handlers set. It's a single global so
 * those handlers can reach it directly.
 *
 *   scene      : everything that animates (§8).
 *   screen     : the window size (§9).
 *   sim_fps    : how many simulation steps per second to run, separate from
 *                how fast we redraw; kept between 10 and 240.
 *   running    : the main loop keeps going while this is set; cleared by 'q',
 *                ESC, or a kill signal.
 *   need_resize: set when the window changes size, handled at the top of the
 *                next loop. Both flags are sig_atomic_t because a signal
 *                handler writes them while the loop reads them.
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

/* ctrl-C / kill ask us to quit; a window resize asks us to re-fit. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

static void app_bootstrap(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
}

/* if the window changed size, re-read it and re-fit the scene to match. */
static void app_handle_pending_resize(App *app)
{
    if (!app->need_resize) return;
    screen_resize(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

/* how much real time passed since the last frame, capped so a long pause
 * (debugger, laptop sleep) can't make the sim try to catch up forever. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > SIM_MAX_FRAME_DT_MS * NS_PER_MS)
        dt = SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    return dt;
}

/* run as many fixed-size sim steps as the elapsed time has earned, so the
 * physics ticks at a steady rate no matter how fast we're drawing. */
static void app_drain_fixed_timestep(App *app, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* recompute the displayed frame rate twice a second so the number isn't jumpy. */
static void app_update_fps_meter(int64_t *fps_accum, int *frame_count,
                                 double *fps_display)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_display = (double)*frame_count
                 / ((double)*fps_accum / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
}

/* sleep off whatever's left of the frame's time budget so we sit at ~60 fps
 * instead of spinning the CPU flat out. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* draw this frame and flip it onto the screen in one go. */
static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* speed up / slow down the sim, staying within the allowed range. */
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

/* tiny one-liners, named so the key table below reads like plain intentions. */
static void app_toggle_pause     (App *app) { app->scene.paused = !app->scene.paused; }
static void app_reset_attractor  (App *app) { scene_reset(&app->scene); }
static void app_cycle_theme_next (App *app) { palette_state_cycle_next(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_theme_prev (App *app) { palette_state_cycle_prev(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_preset_next(App *app)
{
    preset_state_cycle_next(&app->scene.preset);
    scene_compute_geometry(&app->scene);   /* new preset, new size — re-fit */
    scene_reset(&app->scene);
}
static void app_cycle_preset_prev(App *app)
{
    preset_state_cycle_prev(&app->scene.preset);
    scene_compute_geometry(&app->scene);
    scene_reset(&app->scene);
}

static bool app_handle_key(App *app, int ch);

/* check for a keypress without blocking; returns false only if the user quit. */
static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* the keymap: each key just calls one named action above. */
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
 * FrameClock — the loop's timekeeping, gathered so main() reads as a short
 * recipe. It's a local in main(), not part of App, because it's pure loop
 * bookkeeping.
 *
 *   frame_time : when the last frame started, used to measure elapsed time.
 *   sim_accum  : leftover time not yet spent on sim steps. The drain loop eats
 *                it in fixed chunks, which is what keeps the sim rate steady
 *                while the draw rate floats.
 *   fps_accum  : time since we last recomputed the displayed frame rate.
 *   frame_count: frames drawn since that last recompute.
 *   fps_display: the frame rate shown on the HUD, refreshed twice a second.
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

/* the whole program: set up, then each loop measure time, run the sim,
 * draw, wait a bit, and check the keyboard, until the user quits. */
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
