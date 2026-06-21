/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ragdoll_figure.c — a stick-figure puppet (15 dots, 17 bones) falls under
 * gravity and tumbles down slanted shelves. Each dot remembers its last
 * position, so its speed is just where-it-is minus where-it-was; "bones" are
 * fixed-distance rules nudged true many times a frame so the body holds shape
 * and flops like a ragdoll. Sister file: ragdoll_ropes.c (same physics, cloth).
 *
 * Verlet integration; T. Jakobsen, "Advanced Character Physics" (GDC 2001).
 *
 * Build (needs -lm for the trig in bone-direction glyphs and normals):
 *   gcc -std=c11 -O2 -Wall -Wextra ragdoll_figure.c -o ragdoll_figure -lncurses -lm
 */
#define _POSIX_C_SOURCE 200809L

/* M_PI isn't standard C; define it ourselves if the toolchain didn't. */
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
#include <stdio.h>

/* ── §1 config — all tunable numbers, in one place ── */

enum {
    /* Physics tick rate in Hz: how many times per second the puppet steps.
     * Decoupled from on-screen fps by the accumulator loop in main(). */
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,   /* [/] keys nudge by this much */

    HUD_COLS         =  96,   /* byte budget for the status string; fits the widest HUD */
    FPS_UPDATE_MS    = 500,   /* recompute the displayed fps this often */

    /* Color-pair IDs. 1..7 paint the body; 8,9 are the HUD lines. */
    PAIR_HEAD        =   1,   /* white   — head marker             */
    PAIR_BODY        =   2,   /* white   — non-head joints         */
    PAIR_SPINE       =   3,   /* grey    — spine + collarbones     */
    PAIR_ARM         =   4,   /* orange  — arm bones + wrists      */
    PAIR_LEG         =   5,   /* blue    — leg bones + ankles      */
    PAIR_PLATFORM    =   6,   /* cyan    — slanted shelves         */
    PAIR_STRUT       =   7,   /* dim grey — stabiliser struts/floor*/
    PAIR_HUD         =   8,   /* bright yellow — top status        */
    PAIR_HINT        =   9,   /* bright cyan   — bottom key hints  */

    /* The puppet: 15 dots ("particles") joined by 17 bones ("constraints").
     * Dot indices: 0 head, 1 neck, 2/3 shoulders, 4/5 elbows, 6/7 wrists,
     *   8 hip_center, 9/10 hips, 11/12 knees, 13/14 ankles.
     * More constraint passes = stiffer body but more CPU; 8 looks rigid
     * for this many dots. (See satisfy_constraint for the tradeoff.) */
    N_PARTICLES        = 15,
    N_CONSTRAINTS      = 17,
    N_CONSTRAINT_ITERS =  8,
    N_PLATFORMS        =  5,   /* staggered shelves the puppet falls through */

    N_THEMES           = 10,   /* color palettes in THEMES[]   */
    N_PRESETS          =  5,   /* physics scenarios in PRESETS[] */
    N_THEME_SLOTS      =  7,   /* colors per theme = the 7 body pairs */
};

/* Physics tunables. +y points DOWN the screen, so gravity is positive. */
#define GRAVITY       800.0f   /* downward pull, px/s²                       */
#define DAMPING         0.995f /* speed kept per tick; <1 = mild air drag    */
#define BOUNCE_COEFF    0.55f  /* speed kept after a bounce; 0=dead, 1=elastic */
#define WIND_PERIOD     3.5f   /* seconds between random sideways gusts       */
#define WIND_FORCE    120.0f   /* strength of each gust                       */

/* How far inside the screen edges the walls sit, in pixels. */
#define FLOOR_MARGIN    8.0f
#define CEIL_MARGIN    16.0f
#define LEFT_MARGIN    16.0f
#define RIGHT_MARGIN   16.0f

/* Step along a bone in this many pixels when drawing it. Must be < CELL_W
 * (8) or the dashed line would skip whole terminal cells. */
#define DRAW_STEP_PX    2.0f

/* Time in nanoseconds. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* One terminal cell is this many physics pixels wide/tall. Cells aren't
 * square, so physics works in pixels and only converts to cells at draw
 * time (see §4). */
#define CELL_W   8
#define CELL_H  16

/* ── §2 clock — monotonic timer + sleep ── */

/* Current time in nanoseconds. MONOTONIC never jumps backward, so frame-to-
 * frame deltas are always sane. */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/* Sleep ns nanoseconds; return at once if the frame already ran over. */
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ── §3 color — body-part palette + HUD pairs ── */

/*
 * Theme — one named color set for the 7 body pairs (head..strut, in
 * pair order). The two HUD pairs are NOT themed: they stay bright
 * yellow/cyan so the status bar reads against any palette. Every value
 * is from the bright half of the 256-color cube so even the dimmed
 * strut stays visible.
 *
 *   name : shown in the HUD.
 *   col  : 7 xterm-256 foreground indices, one per body pair.
 */
typedef struct {
    const char *name;
    int         col[N_THEME_SLOTS];
} Theme;

/* Ten palettes; head and leg colors are usually the most contrasted so
 * the eye finds the figure fast. */
static const Theme THEMES[N_THEMES] = {
    /*  name         HEAD  BODY  SPN   ARM   LEG   PLAT  STRUT */
    { "CLASSIC",  { 231,  255,  248,  214,   75,   45,  244 } }, /* default */
    { "MATRIX",   { 231,  195,  119,   46,   40,   34,   34 } }, /* green code */
    { "FIRE",     { 226,  214,  208,  202,  196,  166,  130 } }, /* yellow→red */
    { "ICE",      { 231,  195,  159,  123,   87,   51,   45 } }, /* white→cyan */
    { "TOXIC",    { 227,  220,  190,  154,  118,   82,   34 } }, /* yellow→green */
    { "NEON",     { 213,  207,  201,  165,  129,   93,   57 } }, /* pink→purple */
    { "AUTUMN",   { 228,  220,  214,  208,  166,  130,   94 } }, /* gold→brown */
    { "MONO",     { 255,  250,  245,  240,  244,  248,  244 } }, /* greyscale  */
    { "AURORA",   { 159,  120,   87,   51,   39,   99,  135 } }, /* mint→violet */
    { "CARNIVAL", { 226,  196,  202,   51,   46,  201,  244 } }, /* every primary */
};

/* Repaint the 7 body pairs from theme `idx`. HUD pairs untouched. Themes
 * need the 256-color palette, so on 8-color terminals this is a no-op and
 * the fallback pairs from color_init stay. Out-of-range idx falls to 0. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS < 256) return;

    const Theme *t = &THEMES[idx];
    init_pair(PAIR_HEAD,     t->col[0], -1);
    init_pair(PAIR_BODY,     t->col[1], -1);
    init_pair(PAIR_SPINE,    t->col[2], -1);
    init_pair(PAIR_ARM,      t->col[3], -1);
    init_pair(PAIR_LEG,      t->col[4], -1);
    init_pair(PAIR_PLATFORM, t->col[5], -1);
    init_pair(PAIR_STRUT,    t->col[6], -1);
}

/* Turn on color, paint the starting theme, and pin the HUD pairs.
 * Background -1 = the terminal's own background. On terminals with fewer
 * than 256 colors we use a coarse 8-color palette and drop theme cycling. */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        theme_apply(initial_theme);
    } else {
        /* 8-color fallback — coarser but readable, not themeable */
        init_pair(PAIR_HEAD,     COLOR_WHITE,   -1);
        init_pair(PAIR_BODY,     COLOR_WHITE,   -1);
        init_pair(PAIR_SPINE,    COLOR_WHITE,   -1);
        init_pair(PAIR_ARM,      COLOR_YELLOW,  -1);
        init_pair(PAIR_LEG,      COLOR_CYAN,    -1);
        init_pair(PAIR_PLATFORM, COLOR_CYAN,    -1);
        init_pair(PAIR_STRUT,    COLOR_WHITE,   -1);
    }

    /* HUD pairs — same bright yellow/cyan in either color mode. */
    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 coords — turn physics pixels into terminal cells ── */

/*
 * Physics runs in square pixels; the terminal's cells are tall (16 vs 8),
 * so a cell is not a pixel. These two helpers do the only conversion, at
 * draw time. Dividing by the cell size and rounding picks the nearest cell.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5 entity — the puppet: dots, bones, and how it moves ── */

/*
 * Vec2 — a 2-D point (or vector) in pixel space. Used for positions,
 * speeds, gusts, and surface normals alike. Passed around by value;
 * the compiler keeps it in registers.
 *
 *   x : pixels rightward (positive → right edge).
 *   y : pixels DOWNWARD  (positive → bottom). Screen y points down, so
 *       gravity is positive and "up" means a smaller y — this sign
 *       convention shows up all through the physics below.
 */
typedef struct {
    float x;
    float y;
} Vec2;

/*
 * Ragdoll — everything the puppet is and remembers.
 *
 * The trick (Verlet): each dot stores where it is now AND where it was
 * last frame. Its speed is just the gap between the two, so there is no
 * separate velocity array. To shove a dot, you move its old position;
 * that changes the gap, which changes the speed. Every bounce and gust
 * below works by editing old_pos, never by storing a velocity.
 *
 * Bones ("constraints"): a bone just says "keep these two dots a fixed
 * distance apart." Each frame, after gravity moves the dots, we walk
 * every bone and nudge its two dots back to the right distance — many
 * passes, because fixing one bone disturbs the next. Enough passes and
 * the loose dots hold a rigid shape that flops like a real ragdoll.
 *
 *   pos      : where each dot is now. Moves every step.
 *   old_pos  : where each dot was last step. Speed = pos − old_pos.
 *              Bounces and gusts write here to change speed.
 *   prev_pos : snapshot of pos taken at the very start of a step, used
 *              only to draw smooth in-between frames (see lerp_positions).
 *              Kept separate from old_pos because old_pos keeps changing
 *              during a step while prev_pos must stay frozen.
 *
 *   c_a, c_b : the two dot indices each bone connects.
 *   c_len    : the distance that bone wants to hold (its rest length),
 *              measured from the starting pose in scene_init.
 *
 *   gravity    : downward pull, px/s² (tunable with w/s and presets).
 *   wind_force : strength of each gust (tunable with a/d and presets).
 *   wind_timer : seconds since the last gust; fires at WIND_PERIOD.
 *   wind_x     : sideways acceleration applied this step.
 *   paused     : true freezes the physics; drawing keeps running.
 */
typedef struct {
    Vec2 pos     [N_PARTICLES];
    Vec2 old_pos [N_PARTICLES];
    Vec2 prev_pos[N_PARTICLES];

    int   c_a  [N_CONSTRAINTS];
    int   c_b  [N_CONSTRAINTS];
    float c_len[N_CONSTRAINTS];

    float gravity;
    float wind_force;
    float wind_timer;
    float wind_x;

    bool  paused;
} Ragdoll;

/* ── §5a verlet_update — move each dot under gravity + wind ── */

/* Speed of a dot = where it is now minus where it was last step.
 * That's the whole Verlet idea: no stored velocity, just two positions. */
static inline Vec2 implicit_velocity(Vec2 pos, Vec2 old_pos)
{
    return (Vec2){ pos.x - old_pos.x, pos.y - old_pos.y };
}

/* Shrink the speed a hair each step (DAMPING < 1) so the puppet loses
 * energy and eventually settles instead of bouncing forever. */
static inline Vec2 apply_damping(Vec2 v, float damping)
{
    return (Vec2){ v.x * damping, v.y * damping };
}

/* The Verlet step: new position = old position + carried speed + the
 * gravity/wind nudge (acceleration × dt²). v already covers one step of
 * motion, so it isn't multiplied by dt again here. */
static inline Vec2 integrate_acceleration(Vec2 pos, Vec2 v, Vec2 a, float dt2)
{
    return (Vec2){
        pos.x + v.x + a.x * dt2,
        pos.y + v.y + a.y * dt2,
    };
}

/*
 * verlet_update — advance one dot by one step.
 *
 * Read its speed, trim it for drag, then slide old_pos up to the current
 * pos and push pos forward by speed + gravity/wind. Saving the current
 * position into `cur` first is what lets old_pos and pos update without
 * clobbering each other.
 */
static void verlet_update(Ragdoll *r, int i, float dt)
{
    float dt2 = dt * dt;

    Vec2 v = implicit_velocity(r->pos[i], r->old_pos[i]);
    v      = apply_damping(v, DAMPING);
    Vec2 a = (Vec2){ r->wind_x, r->gravity };

    Vec2 cur = r->pos[i];
    r->old_pos[i] = cur;
    r->pos[i]     = integrate_acceleration(cur, v, a, dt2);
}

/* ── §5b apply_boundaries — keep dots inside the screen ── */

/*
 * Bounce a dot off a flat wall on one axis. Pin it onto the wall, then
 * place old_pos on the far side so next step's speed points away from the
 * wall — that reversed gap IS the bounce. BOUNCE_COEFF (0..1) sets how
 * much speed survives: 1 = perfectly bouncy, 0 = dead stop.
 */
static inline void bounce_against_wall_1d(float *pos, float *old_pos,
                                          float wall)
{
    float v_axis = *pos - *old_pos;
    *pos     = wall;
    *old_pos = wall + v_axis * BOUNCE_COEFF;
}

/* Dead stop on one axis: both positions snap to the wall, so the gap (and
 * thus the speed) is zero. Used for the ceiling, where a bounce would look
 * odd and a hard stop also avoids tunnelling. */
static inline void snap_to_wall_1d(float *pos, float *old_pos, float wall)
{
    *pos     = wall;
    *old_pos = wall;
}

/*
 * Keep dot i inside the screen box. Floor and side walls bounce; the
 * ceiling hard-stops. The four tests are independent, so a corner hit
 * bounces correctly on both axes.
 */
static void apply_boundaries(Ragdoll *r, int i, int cols, int rows)
{
    float floor_y = (float)(rows * CELL_H) - FLOOR_MARGIN;
    float ceil_y  = CEIL_MARGIN;
    float left_x  = LEFT_MARGIN;
    float right_x = (float)(cols * CELL_W) - RIGHT_MARGIN;

    if (r->pos[i].y > floor_y)
        bounce_against_wall_1d(&r->pos[i].y, &r->old_pos[i].y, floor_y);
    if (r->pos[i].y < ceil_y)
        snap_to_wall_1d       (&r->pos[i].y, &r->old_pos[i].y, ceil_y);
    if (r->pos[i].x < left_x)
        bounce_against_wall_1d(&r->pos[i].x, &r->old_pos[i].x, left_x);
    if (r->pos[i].x > right_x)
        bounce_against_wall_1d(&r->pos[i].x, &r->old_pos[i].x, right_x);
}

/* ── §5b-2 platform collision — bounce off the slanted shelves ── */

/*
 * Platform — one tilted shelf, just a line segment in pixel space. A dot
 * landing on it bounces; because the shelf is tilted, the bounce also
 * shoves the dot sideways, which is what makes the puppet zig-zag down
 * the stack. Set up once in init_platforms and never changed after.
 *
 *   cx     : center x of the shelf.
 *   y      : surface height at cx (the line pivots here).
 *   half_w : half the shelf's width; it exists for x in [cx-hw, cx+hw].
 *   slope  : rise per pixel rightward. With +y down: positive tilts the
 *            right end down (\), negative tilts it up (/), zero is flat.
 */
typedef struct {
    float cx;
    float y;
    float half_w;
    float slope;
} Platform;

/* Height of the shelf line directly under x. Sampled at both the dot's
 * old and new x so the cross test below works even when the dot also
 * slid sideways. */
static inline float surface_y_at_x(const Platform *pl, float x)
{
    return pl->y + (x - pl->cx) * pl->slope;
}

/* True only when x is within the shelf's width; outside it there's no
 * shelf to land on. */
static inline bool particle_within_platform_extent(const Platform *pl,
                                                   float x)
{
    return fabsf(x - pl->cx) <= pl->half_w;
}

/* Did the dot pass through the shelf going downward this step? True when
 * it was at/above the line last frame and at/below it now. Sampling the
 * line at each end's own x is what handles the tilt. */
static inline bool particle_crossed_surface_from_above(const Platform *pl,
                                                       Vec2 old_pos,
                                                       Vec2 pos)
{
    float surf_now = surface_y_at_x(pl, pos.x);
    float surf_old = surface_y_at_x(pl, old_pos.x);
    return (old_pos.y <= surf_old) && (pos.y >= surf_now);
}

/* The "up" direction perpendicular to a shelf of this slope, as a unit
 * vector. Flat shelf gives straight up (0, -1); a tilt leans it sideways.
 * The bounce reflects across this direction. */
static inline Vec2 surface_upward_normal(float slope)
{
    float length = sqrtf(slope * slope + 1.0f);
    return (Vec2){ slope / length, -1.0f / length };
}

/*
 * Reflect speed v off a surface whose "up" is n, keeping fraction e of it.
 * Strip out the part heading into the surface and reverse it; the part
 * sliding along stays. Because a tilted n points partly sideways, some of
 * the downward speed turns into sideways speed — that's the deflection.
 */
static inline Vec2 reflect_velocity_with_restitution(Vec2 v, Vec2 n, float e)
{
    float dot    = v.x * n.x + v.y * n.y;
    float factor = (1.0f + e) * dot;
    return (Vec2){ v.x - factor * n.x, v.y - factor * n.y };
}

/* Give a dot a chosen speed the Verlet way: put old_pos exactly that far
 * behind pos, so next step's gap (pos − old_pos) equals the speed. */
static inline void write_implicit_velocity(Vec2 *old_pos, Vec2 pos, Vec2 v)
{
    old_pos->x = pos.x - v.x;
    old_pos->y = pos.y - v.y;
}

/*
 * Bounce dot i off shelf pl (already known to have been crossed). Grab its
 * speed first, pin it onto the shelf, then reflect. The "moving away" skip
 * stops a dot already resting on the shelf from being flung off when a
 * bone nudges it sideways.
 */
static void resolve_one_platform_collision(Ragdoll *r, int i,
                                           const Platform *pl)
{
    Vec2 v = implicit_velocity(r->pos[i], r->old_pos[i]);

    r->pos[i].y = surface_y_at_x(pl, r->pos[i].x);
    Vec2 n      = surface_upward_normal(pl->slope);

    float vn = v.x * n.x + v.y * n.y;
    if (vn >= 0.0f) return;            /* moving away — no bounce */

    Vec2 v_reflected = reflect_velocity_with_restitution(v, n, BOUNCE_COEFF);
    write_implicit_velocity(&r->old_pos[i], r->pos[i], v_reflected);
}

/* Check dot i against every shelf: if it's over the shelf's width AND
 * just crossed the surface going down, bounce it. The tilt deflects it
 * sideways, like a stone skipping on water. */
static void apply_platform_collisions(Ragdoll *r, int i,
                                      const Platform *plats, int n)
{
    for (int p = 0; p < n; p++) {
        const Platform *pl = &plats[p];

        if (!particle_within_platform_extent(pl, r->pos[i].x)) continue;
        if (!particle_crossed_surface_from_above(pl,
                                                 r->old_pos[i],
                                                 r->pos[i])) continue;

        resolve_one_platform_collision(r, i, pl);
    }
}

/* ── §5c satisfy_constraint — hold each bone at its length ── */

/* Return the vector from a to b (via out_delta) and its length, computing
 * both in one pass; called a lot, so it shares the work. */
static inline float displacement_and_length(Vec2 a, Vec2 b, Vec2 *out_delta)
{
    out_delta->x = b.x - a.x;
    out_delta->y = b.y - a.y;
    return sqrtf(out_delta->x * out_delta->x +
                 out_delta->y * out_delta->y);
}

/*
 * Fix one bone: nudge its two dots back to the bone's rest length. Measure
 * how far off the current distance is, then move each dot half the error
 * along the line between them — together if stretched, apart if squashed.
 * One nudge isn't enough because fixing this bone disturbs its neighbours;
 * the relax loop calls this over all bones many times until they settle.
 * Both dots share the error equally (they're treated as equal weight).
 */
static void satisfy_constraint(Ragdoll *r, int ci)
{
    int a = r->c_a[ci];
    int b = r->c_b[ci];

    Vec2  delta;
    float length = displacement_and_length(r->pos[a], r->pos[b], &delta);
    if (length < 1e-6f) return;        /* dots on top of each other — avoid /0 */

    float fractional = (length - r->c_len[ci]) / length;
    float cx         = 0.5f * fractional * delta.x;
    float cy         = 0.5f * fractional * delta.y;

    r->pos[a].x += cx;  r->pos[a].y += cy;
    r->pos[b].x -= cx;  r->pos[b].y -= cy;
}

/* ── §5d ragdoll_tick — run one physics step in the right order ── */

/* Save where every dot is right now. The renderer blends from here toward
 * the new positions to draw smooth in-between frames, so this must run
 * before the physics moves anything this step. */
static inline void snapshot_for_render_interpolation(Ragdoll *r)
{
    memcpy(r->prev_pos, r->pos, sizeof r->pos);
}

/*
 * Every WIND_PERIOD seconds, shove every dot sideways by a random gust.
 * It's an instant push, not a steady force, so we apply it the Verlet way:
 * move old_pos sideways, which changes the gap and so the speed.
 * Strength varies 0.5..1.0 of wind_force; direction is a coin flip.
 */
static void apply_wind_gust(Ragdoll *r, float dt)
{
    r->wind_timer += dt;
    if (r->wind_timer < WIND_PERIOD) return;
    r->wind_timer = 0.0f;

    float direction = (rand() % 2 == 0) ? 1.0f : -1.0f;
    float magnitude = r->wind_force * (0.5f + (float)(rand() % 100) / 200.0f);
    float impulse   = direction * magnitude;
    for (int i = 0; i < N_PARTICLES; i++)
        r->old_pos[i].x -= impulse * dt;
}

/*
 * Hold the skeleton together: many passes of fixing every bone, and after
 * each pass re-bounce the dots off the shelves. The re-bounce matters
 * because fixing a bone can drag a foot back through a shelf it had landed
 * on; redoing the collision each pass pins it back on top. More passes =
 * stiffer body for more CPU (N_CONSTRAINT_ITERS sets the tradeoff).
 */
static void relax_constraints_with_collision_repinning(Ragdoll *r,
                                                       const Platform *plats)
{
    for (int iter = 0; iter < N_CONSTRAINT_ITERS; iter++) {
        for (int ci = 0; ci < N_CONSTRAINTS; ci++)
            satisfy_constraint(r, ci);
        for (int i = 0; i < N_PARTICLES; i++)
            apply_platform_collisions(r, i, plats, N_PLATFORMS);
    }
}

/*
 * One full physics step. The order is the whole trick: move the dots
 * first, let them hit the world, THEN hold the bones. Doing collisions
 * before the bone-fixing is what lets a foot land on a shelf and then the
 * leg fold up to it, instead of the bone yanking the foot through the floor.
 */
static void ragdoll_tick(Ragdoll *r, float dt, int cols, int rows,
                         const Platform *plats)
{
    snapshot_for_render_interpolation(r);     /* remember frame start for drawing */

    if (r->paused) return;                     /* frozen, but snapshot already saved */

    apply_wind_gust(r, dt);

    for (int i = 0; i < N_PARTICLES; i++)
        verlet_update(r, i, dt);               /* gravity + wind move every dot */

    for (int i = 0; i < N_PARTICLES; i++)
        apply_boundaries(r, i, cols, rows);    /* walls, floor, ceiling */

    for (int i = 0; i < N_PARTICLES; i++)
        apply_platform_collisions(r, i, plats, N_PLATFORMS);

    relax_constraints_with_collision_repinning(r, plats);   /* hold the bones */
}

/* ── §5e ragdoll_draw — stamp the puppet into the terminal ── */

/* Pick the ASCII char that best matches a bone's slope: -, /, |, or \.
 * dy is flipped first so the angle is measured the usual math way despite
 * screen y pointing down. */
static chtype bone_glyph(float dx, float dy)
{
    float ang = atan2f(-dy, dx);
    float deg = ang * (180.0f / (float)M_PI);
    if (deg <    0.0f) deg += 360.0f;
    if (deg >= 180.0f) deg -= 180.0f;

    if (deg < 22.5f || deg >= 157.5f) return (chtype)'-';
    if (deg < 67.5f)                   return (chtype)'\\';
    if (deg < 112.5f)                  return (chtype)'|';
    return                             (chtype)'/';
}

/* Draw one bone as a dashed line from a to b: walk it in small pixel steps,
 * stamping the direction glyph in each cell touched. prev_cx/prev_cy skip
 * re-stamping the same cell (within a bone and at shared joints).
 * Off-screen cells are dropped. */
static void draw_bone(WINDOW *w,
                      Vec2 a, Vec2 b,
                      int pair, attr_t attr,
                      int cols, int rows,
                      int *prev_cx, int *prev_cy)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    chtype glyph  = bone_glyph(dx, dy);
    int    nsteps = (int)ceilf(len / DRAW_STEP_PX) + 1;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == *prev_cx && cy == *prev_cy) continue;
        *prev_cx = cx;
        *prev_cy = cy;

        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, glyph);
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/* Put one char at cell (cx,cy) with color+attr. Off-screen cells are
 * dropped. The cast keeps chars > 127 from sign-extending into garbage. */
static void mark_cell(WINDOW *w, int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/* Blend each dot from its frame-start position toward its current one by
 * alpha (0..1). Physics steps at a fixed rate that rarely matches the
 * draw rate; this in-between blend keeps motion smooth instead of jerky. */
static void lerp_positions(const Ragdoll *r, float alpha, Vec2 rp[N_PARTICLES])
{
    for (int i = 0; i < N_PARTICLES; i++) {
        rp[i].x = r->prev_pos[i].x + (r->pos[i].x - r->prev_pos[i].x) * alpha;
        rp[i].y = r->prev_pos[i].y + (r->pos[i].y - r->prev_pos[i].y) * alpha;
    }
}

/* Draw each shelf in two passes: first a line of 'o' beads along the
 * surface, then bolder '0' anchors at the ends and midpoint so the shelf
 * has a clear shape. */
static void draw_platforms(WINDOW *w, const Platform *plats,
                           int cols, int rows)
{
    static const float  node_u   [5] = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
    static const char   node_ch  [5] = { '0',  'o',   '0',  'o',   '0'  };
    static const attr_t node_attr[5] = { A_BOLD, A_NORMAL, A_BOLD, A_NORMAL, A_BOLD };

    for (int p = 0; p < N_PLATFORMS; p++) {
        const Platform *pl = &plats[p];
        float x0 = pl->cx - pl->half_w;
        float y0 = pl->y  + (-pl->half_w) * pl->slope;
        float x1 = pl->cx + pl->half_w;
        float y1 = pl->y  + ( pl->half_w) * pl->slope;
        float dx = x1 - x0, dy = y1 - y0;
        float L  = sqrtf(dx*dx + dy*dy);
        if (L < 0.1f) continue;

        /* Pass 1 — bead fill */
        int steps = (int)ceilf(L / 5.0f) + 1;
        int prev_cx = -9999, prev_cy = -9999;
        for (int s = 0; s <= steps; s++) {
            float u  = (float)s / (float)steps;
            int   cx = px_to_cell_x(x0 + dx * u);
            int   cy = px_to_cell_y(y0 + dy * u);
            if (cx == prev_cx && cy == prev_cy) continue;
            prev_cx = cx; prev_cy = cy;
            mark_cell(w, cx, cy, 'o', PAIR_PLATFORM, A_NORMAL, cols, rows);
        }

        /* Pass 2 — quarter-point nodes */
        for (int n = 0; n < 5; n++) {
            int cx = px_to_cell_x(x0 + dx * node_u[n]);
            int cy = px_to_cell_y(y0 + dy * node_u[n]);
            mark_cell(w, cx, cy, node_ch[n],
                      PAIR_PLATFORM, node_attr[n], cols, rows);
        }
    }
}

/* A dashed line across the floor row, just for visual reference. */
static void draw_ground(WINDOW *w, int cols, int rows)
{
    int floor_row = px_to_cell_y((float)(rows * CELL_H) - FLOOR_MARGIN);
    if (floor_row < 0 || floor_row >= rows) return;
    for (int cx = 0; cx < cols; cx++) {
        mark_cell(w, cx, floor_row, '-', PAIR_STRUT, A_DIM, cols, rows);
    }
}

/*
 * Draw all 17 bones, each colored by which body part it is:
 *   0-1 spine, 2-3 collarbones, 8-9 hip cross  → SPINE color
 *   4-7 arms                                   → ARM color
 *   10-13 legs                                 → LEG color
 *   14-16 hidden stabiliser struts             → STRUT color, dimmed
 * The de-dup state is reset per bone so neighbours don't blank each
 * other's first cell.
 */
static void draw_bones(WINDOW *w, const Ragdoll *r,
                       const Vec2 rp[N_PARTICLES], int cols, int rows)
{
    static const int bone_pair[N_CONSTRAINTS] = {
        PAIR_SPINE, PAIR_SPINE,                                   /*  0–1 */
        PAIR_SPINE, PAIR_SPINE,                                   /*  2–3 */
        PAIR_ARM,   PAIR_ARM,   PAIR_ARM,   PAIR_ARM,             /*  4–7 */
        PAIR_SPINE, PAIR_SPINE,                                   /*  8–9 */
        PAIR_LEG,   PAIR_LEG,   PAIR_LEG,   PAIR_LEG,             /* 10–13 */
        PAIR_STRUT, PAIR_STRUT, PAIR_STRUT,                       /* 14–16 */
    };
    /* Body bones (0..13) are bold so the figure looks solid; the three
     * stabiliser struts (14..16) are physics-only, so they stay dim. */
    static const attr_t bone_attr[N_CONSTRAINTS] = {
        A_BOLD, A_BOLD,                            /*  0-1  spine        */
        A_BOLD, A_BOLD,                            /*  2-3  collarbones  */
        A_BOLD, A_BOLD, A_BOLD, A_BOLD,            /*  4-7  arms         */
        A_BOLD, A_BOLD,                            /*  8-9  hip cross    */
        A_BOLD, A_BOLD, A_BOLD, A_BOLD,            /* 10-13 legs         */
        A_DIM,  A_DIM,  A_DIM,                     /* 14-16 stabilisers  */
    };

    for (int ci = 0; ci < N_CONSTRAINTS; ci++) {
        int pcx = -9999, pcy = -9999;
        draw_bone(w,
                  rp[r->c_a[ci]], rp[r->c_b[ci]],
                  bone_pair[ci], bone_attr[ci],
                  cols, rows,
                  &pcx, &pcy);
    }
}

/*
 * Stamp a joint marker over each dot (head is drawn separately). The glyph
 * and brightness pick out the silhouette: shoulders and hips are bold 'o'
 * torso corners, wrists are bold 'o' fists, ankles are bold 'v' feet,
 * elbows and knees are plain 'o' hinges, and neck/hip-center are dim '+'
 * since the bones already cover them.
 */
static void draw_particles(WINDOW *w, const Vec2 rp[N_PARTICLES],
                           int cols, int rows)
{
    for (int i = 1; i < N_PARTICLES; i++) {     /* skip i=0; draw_head handles it */
        int cx = px_to_cell_x(rp[i].x);
        int cy = px_to_cell_y(rp[i].y);

        char   ch;
        int    pair;
        attr_t attr;

        switch (i) {
        case 1:  case 8:                          /* neck, hip_center            */
            ch = '+'; pair = PAIR_SPINE; attr = A_DIM;    break;
        case 2:  case 3:  case 9:  case 10:       /* shoulders + hips            */
            ch = 'o'; pair = PAIR_SPINE; attr = A_BOLD;   break;
        case 4:  case 5:                          /* elbows                      */
            ch = 'o'; pair = PAIR_ARM;   attr = A_NORMAL; break;
        case 6:  case 7:                          /* wrists / fists              */
            ch = 'o'; pair = PAIR_ARM;   attr = A_BOLD;   break;
        case 11: case 12:                         /* knees                       */
            ch = 'o'; pair = PAIR_LEG;   attr = A_NORMAL; break;
        case 13: case 14:                         /* ankles / feet               */
            ch = 'v'; pair = PAIR_LEG;   attr = A_BOLD;   break;
        default:
            ch = '.'; pair = PAIR_BODY;  attr = A_DIM;    break;
        }

        mark_cell(w, cx, cy, ch, pair, attr, cols, rows);
    }
}

/*
 * Draw the head as two cells: an 'O' face, plus a tick of "hair" one cell
 * away in the head-minus-neck direction (always the side facing away from
 * the body, so it stays on top however the puppet tumbles). Recomputed each
 * frame; drawn last so the head wins any shared cell.
 */
static void draw_head(WINDOW *w, const Vec2 rp[N_PARTICLES],
                      int cols, int rows)
{
    Vec2 head = rp[0];
    Vec2 neck = rp[1];

    int fx = px_to_cell_x(head.x);
    int fy = px_to_cell_y(head.y);
    mark_cell(w, fx, fy, 'O', PAIR_HEAD, A_BOLD, cols, rows);

    /* Step one cell along head→neck (scaled per axis since cells aren't
     * square) to place the hair on a neighbouring cell. */
    float dx = head.x - neck.x;
    float dy = head.y - neck.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;          /* head sits on neck — no direction, skip hair */

    float ux = dx / len;
    float uy = dy / len;
    int hx = px_to_cell_x(head.x + ux * (float)CELL_W);
    int hy = px_to_cell_y(head.y + uy * (float)CELL_H);

    /* Skip if halo would overdraw the face cell (e.g. very tight neck). */
    if (hx == fx && hy == fy) return;

    mark_cell(w, hx, hy, '\'', PAIR_HEAD, A_BOLD, cols, rows);
}

/* Draw one frame back-to-front: shelves and ground, then bones, then
 * joints, then the head last. Later layers paint over earlier ones at
 * shared cells, so the head always reads on top. */
static void render_ragdoll(const Ragdoll *r, WINDOW *w,
                           int cols, int rows, float alpha,
                           const Platform *plats)
{
    Vec2 rp[N_PARTICLES];
    lerp_positions(r, alpha, rp);

    draw_platforms (w, plats,    cols, rows);
    draw_ground    (w,           cols, rows);
    draw_bones     (w, r, rp,    cols, rows);
    draw_particles (w, rp,       cols, rows);
    draw_head      (w, rp,       cols, rows);
}

/* ── §6 scene — the world: puppet + shelves + presets ── */

/*
 * Scene — the whole world: the puppet plus the shelves it falls past.
 * The shelves are read by the collision pass but never change during a
 * step; they're rebuilt on resize so the layout fits the new terminal.
 * Settings like theme and preset live in App, not here, so a reset can
 * wipe the scene without losing them.
 */
typedef struct {
    Ragdoll  ragdoll;
    Platform platforms[N_PLATFORMS];
} Scene;

/*
 * Preset — a named bundle of the three settings the user can cycle (n/p):
 * gravity, wind strength, and physics tick rate. Values are tuned for how
 * they look in the terminal, not for real-world accuracy.
 *
 *   name       : shown in the HUD.
 *   gravity    : downward pull, px/s².
 *   wind_force : strength of each gust.
 *   sim_fps    : physics steps per second; higher feels stiffer, costs CPU.
 */
typedef struct {
    const char *name;
    float       gravity;
    float       wind_force;
    int         sim_fps;
} Preset;

/* Five scenarios. EARTH (index 0) is the default and matches §1's GRAVITY
 * and WIND_FORCE; the rest are exaggerated for fun. */
static const Preset PRESETS[N_PRESETS] = {
    /*  name         gravity   wind_force  sim_fps */
    { "EARTH",         800.0f,   120.0f,     60 },
    { "MOON",          130.0f,    20.0f,     60 },
    { "TORNADO",       800.0f,   500.0f,     60 },
    { "JUPITER",      2000.0f,    60.0f,     80 },
    { "ZERO_G",          0.0f,   200.0f,     60 },
};

/* Copy a preset's gravity and wind into the puppet. Leaves the figure's
 * shape alone, so callers wanting a fresh figure call scene_init first.
 * sim_fps lives in App, so the caller sets that separately. Bad idx → 0. */
static void preset_apply(Ragdoll *r, int idx)
{
    if (idx < 0 || idx >= N_PRESETS) idx = 0;
    const Preset *p = &PRESETS[idx];
    r->gravity    = p->gravity;
    r->wind_force = p->wind_force;
}

/*
 * Lay out the shelves: spread down the screen (18%..78% of height), left/
 * center/right across it so the puppet zig-zags, each ~36% wide, with
 * alternating tilt (positive = right end lower \, negative = right end up /).
 */
static void init_platforms(Platform *plats, int cols, int rows)
{
    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);
    float hw   = wpx * 0.18f;   /* half-width, so full width is 36% of screen */

    static const float yfrac [N_PLATFORMS] = { 0.18f,  0.32f,  0.47f,  0.62f,  0.77f };
    static const float xfrac [N_PLATFORMS] = { 0.50f,  0.25f,  0.72f,  0.35f,  0.65f };
    static const float slopes[N_PLATFORMS] = { -0.35f,  0.38f, -0.40f,  0.32f, -0.36f };

    for (int i = 0; i < N_PLATFORMS; i++) {
        plats[i].y      = hpx * yfrac[i];
        plats[i].cx     = wpx * xfrac[i];
        plats[i].half_w = hw;
        plats[i].slope  = slopes[i];
    }
}

/* Register one bone between dots a and b. Its rest length is just their
 * current spacing, so whatever pose the dots are placed in becomes the
 * shape the bones try to hold — no separate length table needed. */
static void add_constraint(Ragdoll *r, int *nc, int a, int b)
{
    if (*nc >= N_CONSTRAINTS) return;
    r->c_a[*nc] = a;
    r->c_b[*nc] = b;
    float dx = r->pos[b].x - r->pos[a].x;
    float dy = r->pos[b].y - r->pos[a].y;
    r->c_len[*nc] = sqrtf(dx * dx + dy * dy);
    (*nc)++;
}

/*
 * Build a fresh puppet: spawn it in a spread-arm "T" pose near the top so
 * it can fall through every shelf, then register the 17 bones (their rest
 * lengths come straight from this pose). The off[] table below gives each
 * dot's pixel offset from the spawn point. old_pos and prev_pos start equal
 * to pos, so the figure begins at rest with no jump on frame 1.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Ragdoll *r = &sc->ragdoll;

    r->gravity    = GRAVITY;
    r->wind_force = WIND_FORCE;
    r->paused     = false;
    r->wind_timer = 0.0f;
    r->wind_x     = 0.0f;

    init_platforms(sc->platforms, cols, rows);

    /* Spawn point: centered, near the top. cy must clear the head (96px up)
     * plus CEIL_MARGIN, so 130 keeps the whole figure on screen. */
    float cx = (float)(cols * CELL_W) * 0.5f;
    float cy = CEIL_MARGIN + 130.0f;

    /* Each dot's (x, y) offset from the spawn point, in pixels. */
    static const float off[N_PARTICLES][2] = {
        /*  0 head          */  {  0.0f, -96.0f },
        /*  1 neck          */  {  0.0f, -64.0f },
        /*  2 left_shoulder */  { -48.0f, -64.0f },
        /*  3 right_shoulder*/  { +48.0f, -64.0f },
        /*  4 left_elbow    */  { -96.0f, -64.0f },
        /*  5 right_elbow   */  { +96.0f, -64.0f },
        /*  6 left_wrist    */  {-144.0f, -64.0f },
        /*  7 right_wrist   */  {+144.0f, -64.0f },
        /*  8 hip_center    */  {  0.0f, -32.0f },
        /*  9 left_hip      */  { -32.0f, -32.0f },
        /* 10 right_hip     */  { +32.0f, -32.0f },
        /* 11 left_knee     */  { -32.0f, +16.0f },
        /* 12 right_knee    */  { +32.0f, +16.0f },
        /* 13 left_ankle    */  { -32.0f, +64.0f },
        /* 14 right_ankle   */  { +32.0f, +64.0f },
    };

    for (int i = 0; i < N_PARTICLES; i++) {
        r->pos[i].x     = cx + off[i][0];
        r->pos[i].y     = cy + off[i][1];
        r->old_pos[i]   = r->pos[i];   /* old == now → starts at rest */
        r->prev_pos[i]  = r->pos[i];
    }

    /* The 17 bones, in the order draw_bones colors them. */
    int nc = 0;
    add_constraint(r, &nc,  0,  1);   /*  0: head → neck (spine top)          */
    add_constraint(r, &nc,  1,  8);   /*  1: neck → hip_center (spine)        */
    add_constraint(r, &nc,  1,  2);   /*  2: neck → left_shoulder             */
    add_constraint(r, &nc,  1,  3);   /*  3: neck → right_shoulder            */
    add_constraint(r, &nc,  2,  4);   /*  4: left_shoulder → left_elbow       */
    add_constraint(r, &nc,  4,  6);   /*  5: left_elbow → left_wrist          */
    add_constraint(r, &nc,  3,  5);   /*  6: right_shoulder → right_elbow     */
    add_constraint(r, &nc,  5,  7);   /*  7: right_elbow → right_wrist        */
    add_constraint(r, &nc,  8,  9);   /*  8: hip_center → left_hip            */
    add_constraint(r, &nc,  8, 10);   /*  9: hip_center → right_hip           */
    add_constraint(r, &nc,  9, 11);   /* 10: left_hip → left_knee             */
    add_constraint(r, &nc, 11, 13);   /* 11: left_knee → left_ankle           */
    add_constraint(r, &nc, 10, 12);   /* 12: right_hip → right_knee           */
    add_constraint(r, &nc, 12, 14);   /* 13: right_knee → right_ankle         */
    add_constraint(r, &nc,  2,  3);   /* 14: shoulder width stabiliser        */
    add_constraint(r, &nc,  9, 10);   /* 15: hip width stabiliser             */
    add_constraint(r, &nc,  0,  2);   /* 16: head → left_shoulder (strut)     */
    /* Note: (0,3) would be constraint 17, but we cap at N_CONSTRAINTS=17.
     * The head-to-right-shoulder strut is omitted; the left-side strut
     * plus the shoulder-width strut provides sufficient head stability. */
}

/* One physics step. dt is the fixed step length in seconds (1 / sim_fps). */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    ragdoll_tick(&sc->ragdoll, dt, cols, rows, sc->platforms);
}

/* Draw one frame. alpha (0..1) is how far between physics steps we are,
 * used to blend for smooth motion. */
static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)dt_sec;
    render_ragdoll(&sc->ragdoll, w, cols, rows, alpha, sc->platforms);
}

/* ── §7 screen — the ncurses display layer ── */

/* Screen — the terminal's size in character cells. Cached so the rest of
 * the code reads cols/rows as plain ints, and refreshed on resize. */
typedef struct {
    int cols;   /* width  in cells */
    int rows;   /* height in cells */
} Screen;

/* Put the terminal into raw, no-echo, hidden-cursor mode for animation. */
static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(0);                  /* CLASSIC; cycled via 't' at runtime */
    getmaxyx(stdscr, s->rows, s->cols);
}

/* screen_free() — restore terminal to pre-animation state. */
static void screen_free(Screen *s) { (void)s; endwin(); }

/*
 * screen_resize() — handle a SIGWINCH terminal resize event.
 * endwin() + refresh() forces ncurses to re-read LINES and COLS.
 */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw() — compose a full frame into stdscr.
 *
 * Order:
 *   1. erase()    — clear newscr (no terminal write yet)
 *   2. scene_draw() — bones + particles + ground line
 *   3. HUD top    — status string right-aligned
 *   4. HUD bottom — key hint bar
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        int theme_idx, int preset_idx,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Ragdoll *r = &sc->ragdoll;

    /* Row 0 (top-right): live status — PAIR_HUD bright yellow, A_BOLD */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%3d Hz  grav:%.0f  wind:%.0f  %s ",
             fps, sim_fps, r->gravity, r->wind_force,
             r->paused ? "PAUSED " : "running");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 (top-left): theme + preset readout — PAIR_HUD, no A_BOLD so
     * row 0 stays the dominant status line per CLAUDE.md HUD spec. */
    char buf2[HUD_COLS + 1];
    snprintf(buf2, sizeof buf2,
             " theme: %-9s  preset: %-8s ",
             THEMES [theme_idx ].name,
             PRESETS[preset_idx].name);
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, 0, "%s", buf2);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom-left key hint — PAIR_HINT bright cyan, A_BOLD */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset  w/s:grav  a/d:wind  [/]:Hz  t:theme  n/p:preset ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/*
 * screen_present() — flush to terminal via ncurses double-buffer.
 * wnoutrefresh() → doupdate() sends only changed cells to the fd.
 */
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §8 app — top-level container + main loop ── */

/*
 * App — top-level container; everything outside the world.
 *
 * Intent
 *   Bundles the simulated world (Scene), the host terminal (Screen),
 *   and the session-level loop-control flags into one record so
 *   main() reads as four-line phases: init / service signals /
 *   step+draw / shutdown. Declared file-scope (g_app) so signal
 *   handlers — which cannot take a user argument — can write
 *   `running` and `need_resize` without globals scattered through
 *   the file.
 *
 * Locality of concern
 *   ── Owned subsystems ── nouns the app composes
 *      scene       — the world being simulated (§6)
 *      screen      — the terminal extent it draws to (§7)
 *
 *   ── Session state ── user-tunable settings that survive a reset
 *      sim_fps     — physics tick rate (cycled with [ / ])
 *      theme_idx   — index into §3 THEMES[]  (cycled with `t`)
 *      preset_idx  — index into §6 PRESETS[] (cycled with `n`/`p`)
 *
 *   ── Loop control ── verbs the loop reads each frame
 *      running     — clear → loop exits; set by SIGINT/SIGTERM
 *      need_resize — set by SIGWINCH; cleared after Screen refresh
 *
 * Why theme_idx / preset_idx live HERE (not in Scene)
 *   Session state, not simulation state. `r` reset rebuilds the
 *   ragdoll from scratch (scene_init wipes Scene memory) but the
 *   user's chosen theme + preset must persist across that reset.
 *   Putting them in App keeps them out of the memset's scope.
 *
 * Why volatile sig_atomic_t (not bool, not int)
 *   `volatile`    : the compiler must not cache the flag across a
 *                   signal-handler write — every loop iteration must
 *                   re-read it from memory.
 *   `sig_atomic_t`: POSIX-guaranteed atomic with respect to async
 *                   signals; a plain `int` could be observed half-
 *                   written on architectures where stores are split.
 *   See [12] Raymond §"Signal handling".
 *
 * Things that DO NOT live here
 *   - Wall-clock timestamps / fps counters — main() locals; no
 *     other code path needs them.
 *   - Ragdoll runtime values (gravity, wind_force) — sim state in
 *     §5 Ragdoll. Presets override these via preset_apply().
 *   - Theme + Preset tables themselves — file-scope `static const`.
 */
typedef struct {
    /* ── Owned subsystems ─────────────────────────────────────── */
    Scene  scene;              /* the world (§6)                       */
    Screen screen;             /* terminal extent (§7)                 */

    /* ── Session state (survives scene_init reset) ────────────── */
    int    sim_fps;            /* physics tick rate (Hz)               */
    int    theme_idx;          /* index into §3 THEMES[]               */
    int    preset_idx;         /* index into §6 PRESETS[]              */

    /* ── Loop control ─────────────────────────────────────────── */
    volatile sig_atomic_t running;      /* main loop predicate            */
    volatile sig_atomic_t need_resize;  /* SIGWINCH pending               */
} App;

static App g_app;

/* Signal handlers — set flags only; no ncurses or malloc calls */
static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/* cleanup() — atexit safety net: endwin() even on unhandled exit */
static void cleanup(void) { endwin(); }

/*
 * app_do_resize() — handle a pending SIGWINCH terminal resize.
 *
 * After resize, clamp all particle positions to the new pixel bounds
 * so no particle is stranded in a now-invisible area.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    int      cols = app->screen.cols;
    int      rows = app->screen.rows;
    Ragdoll *r    = &app->scene.ragdoll;
    float    wpx  = (float)(cols * CELL_W);
    float    hpx  = (float)(rows * CELL_H);
    for (int i = 0; i < N_PARTICLES; i++) {
        if (r->pos[i].x >= wpx) { r->pos[i].x = wpx - 1.0f; r->old_pos[i].x = r->pos[i].x; }
        if (r->pos[i].y >= hpx) { r->pos[i].y = hpx - 1.0f; r->old_pos[i].y = r->pos[i].y; }
        if (r->pos[i].x < 0.0f) { r->pos[i].x = 0.0f;        r->old_pos[i].x = 0.0f; }
        if (r->pos[i].y < 0.0f) { r->pos[i].y = 0.0f;        r->old_pos[i].y = 0.0f; }
    }
    /* Rescale platforms to new terminal dimensions */
    init_platforms(app->scene.platforms, cols, rows);
    app->need_resize = 0;
}

/*
 * cycle_preset() — switch to a different preset and respawn the figure.
 *
 *   Preset cycling resets the scene (so the new physics scenario starts
 *   from a clean T-pose) and applies the preset's sim_fps. Theme is
 *   preserved across the reset so visual identity stays stable.
 *
 *   `dir` is ±1 (next/previous, wraps modulo N_PRESETS).
 */
static void cycle_preset(App *app, int dir)
{
    int next = (app->preset_idx + dir + N_PRESETS) % N_PRESETS;
    app->preset_idx = next;

    scene_init(&app->scene, app->screen.cols, app->screen.rows);
    preset_apply(&app->scene.ragdoll, next);
    app->sim_fps = PRESETS[next].sim_fps;
}

/*
 * cycle_theme() — switch to a different colour theme.
 *
 *   theme_apply rebinds pairs 1..7 via init_pair; the next frame's
 *   draw picks up the new palette automatically. Cheap (7 calls);
 *   no scene reset needed.
 */
static void cycle_theme(App *app, int dir)
{
    int next = (app->theme_idx + dir + N_THEMES) % N_THEMES;
    app->theme_idx = next;
    theme_apply(next);
}

/*
 * app_handle_key() — process one keypress; return false to quit.
 *
 * KEY MAP:
 *   q / Q / ESC   quit
 *   space         toggle pause
 *   r / R         reset simulation (preserves theme + preset)
 *   w / ↑         gravity × 1.3  (faster fall)
 *   s / ↓         gravity ÷ 1.3  (slower fall)
 *   d / →         wind_force + 20
 *   a / ←         wind_force − 20 (min 0)
 *   ] / +         sim_fps + SIM_FPS_STEP
 *   [ / -         sim_fps − SIM_FPS_STEP
 *   t / T         cycle THEME (10 multi-colour palettes)
 *   n / N         next PRESET   (5 named physics scenarios)
 *   p / P         previous PRESET
 */
static bool app_handle_key(App *app, int ch)
{
    Ragdoll *r = &app->scene.ragdoll;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ': r->paused = !r->paused; break;

    case 'r': case 'R':
        /* Reset: preserve sim_fps + theme + preset; re-apply preset
         * physics so the freshly-spawned figure inherits current scenario. */
        scene_init(&app->scene, app->screen.cols, app->screen.rows);
        preset_apply(&app->scene.ragdoll, app->preset_idx);
        break;

    case 'w': case KEY_UP:
        r->gravity *= 1.3f;
        if (r->gravity > 3200.0f) r->gravity = 3200.0f;
        break;
    case 's': case KEY_DOWN:
        r->gravity /= 1.3f;
        if (r->gravity < 100.0f) r->gravity = 100.0f;
        break;

    case 'd': case KEY_RIGHT:
        r->wind_force += 20.0f;
        if (r->wind_force > 600.0f) r->wind_force = 600.0f;
        break;
    case 'a': case KEY_LEFT:
        r->wind_force -= 20.0f;
        if (r->wind_force < 0.0f) r->wind_force = 0.0f;
        break;

    case ']': case '+': case '=':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[': case '-':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't': case 'T':
        cycle_theme(app, +1);
        break;

    case 'n': case 'N':
        cycle_preset(app, +1);
        break;
    case 'p': case 'P':
        cycle_preset(app, -1);
        break;

    default: break;
    }
    return true;
}

/* ─────────────────────────────────────────────────────────────────────
 * main() — the game loop  (structure identical to framework.c §8)
 *
 * Seven steps per frame (identical to snake_forward_kinematics.c):
 *  ① RESIZE CHECK     — handle pending SIGWINCH before touching ncurses
 *  ② MEASURE dt       — nanoseconds since last frame, capped at 100 ms
 *  ③ ACCUMULATOR      — fire scene_tick() at fixed sim_fps Hz
 *  ④ ALPHA            — sub-tick lerp factor for smooth rendering
 *  ⑤ FPS COUNTER      — 500 ms sliding window average
 *  ⑥ FRAME CAP        — sleep before render to hold ~60 fps
 *  ⑦ DRAW + PRESENT   — erase → draw → wnoutrefresh → doupdate
 *  ⑧ DRAIN INPUT      — getch() loop until ERR
 * ───────────────────────────────────────────────────────────────────── */
int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);

    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app        = &g_app;
    app->running    = 1;
    app->sim_fps    = SIM_FPS_DEFAULT;
    app->theme_idx  = 0;                /* CLASSIC palette                */
    app->preset_idx = 0;                /* EARTH physics (matches §1)     */

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
    preset_apply(&app->scene.ragdoll, app->preset_idx);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        int64_t frame_start = clock_ns();

        /* ── ① resize ────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── ② dt ────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── ③ fixed-step accumulator ────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── ④ alpha ─────────────────────────────────────────────── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── ⑤ fps counter ───────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── ⑥ frame cap — sleep before render ──────────────────── *
         * Budget = 1/60 s.  elapsed is wall time spent on physics +
         * accounting since frame_start; sleep the remainder so the
         * render rate sits at 60 fps regardless of sim Hz.           */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── ⑦ draw + present ────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
                    app->theme_idx, app->preset_idx,
                    alpha, dt_sec);
        screen_present();

        /* ── ⑧ drain all pending input ──────────────────────────── */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }
    }

    screen_free(&app->screen);
    return 0;
}
