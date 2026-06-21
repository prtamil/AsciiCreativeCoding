/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ragdoll_ropes.c — seven ropes hung from a ceiling, swaying in wind.
 *
 * Each rope is a string of beads held together by "stay a fixed distance
 * from your neighbour" links; the beads fall under gravity and get blown
 * sideways by a sinusoidal wind. Sister file ragdoll_figure.c uses the
 * same engine to animate a stick figure.
 *
 * Physics: Verlet integration with distance constraints
 * (Thomas Jakobsen, "Advanced Character Physics", GDC 2001).
 * SPDX-License-Identifier: MIT.
 *
 * Sections: §1 config  §2 clock  §3 color  §4 coords  §5 entity (ropes +
 * solver + renderer)  §6 scene  §7 screen  §8 app.
 *
 * Keys: q/ESC quit · space pause · w/s wind amplitude · a/d wind frequency
 *       · r reset · t/T theme · [ / ] sim Hz.
 *
 * Build (needs -lm for sin/cos in the wind force):
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       ragdoll_ropes.c -o ragdoll_ropes -lncurses -lm
 */

#define _POSIX_C_SOURCE 200809L

/* M_PI is not standard C; define it ourselves if the toolchain omits it. */
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

/* ── §1 config — every tunable number lives here ── */

enum {
    /* Physics ticks per second. Higher = steadier ropes but more CPU.
       Adjustable [10,120] with [ and ]; 60 matches the render cap. */
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,   /* step for the [ and ] keys */

    HUD_COLS         =  96,   /* byte size of the status-bar string buffer */
    FPS_UPDATE_MS    = 500,   /* how often the fps reading refreshes */

    /* Colour bookkeeping. Rope r uses pair (r % N_PAIRS) + 1. HUD pairs are
       fixed (theme-independent) so the bars stay readable on any backdrop. */
    N_PAIRS          =   7,
    PAIR_HUD         =   8,   /* bright yellow — top status bar  */
    PAIR_HINT        =   9,   /* bright cyan   — bottom key hint */
    N_THEMES         =  10,   /* palettes cycled with t / T */

    N_ROPES = 7,   /* how many ropes hang from the ceiling */
    N_SEG   = 20,  /* beads per rope, including the fixed anchor at index 0 */
    /* Distance-constraint passes per tick. Each pass roughly halves the
       remaining stretch error; 6 makes a 20-bead rope look inextensible.
       (ragdoll_figure.c uses more because its skeleton branches.) */
    N_ITERS =  6,
};

/* Downward pull on every bead, px/s². +y points down (screen convention). */
#define GRAVITY          800.0f
/* Each tick a bead keeps this fraction of its speed: 0.992 = lose 0.8%/tick.
   This air-resistance fudge stops the ropes whipping wildly at strong wind. */
#define ROPE_DAMPING       0.992f
/* Speed kept after hitting the floor or a wall. 0.5 = half; <0.3 looks dead,
   >0.8 looks rubbery. */
#define BOUNCE_COEFF       0.5f

/* Wind sideways-force amplitude, px/s². Runtime range 0 (ropes hang straight)
   to 1000. The wind oscillates left-right as a sine wave. */
#define WIND_AMP_DEFAULT   250.0f
/* Wind oscillation speed, rad/s. 0.4 → a full left-right cycle every ~16 s,
   a lazy breeze; toward 4.0 it rattles. Runtime minimum 0.05. */
#define WIND_FREQ_DEFAULT    0.4f
/* Terminal row of the ceiling line the anchors hang from (below the HUD). */
#define ANCHOR_ROW_CELLS     2

/* How far beads are kept from each edge, in px (clamped in rope_boundaries). */
#define FLOOR_MARGIN   8.0f
#define LEFT_MARGIN   16.0f
#define RIGHT_MARGIN  16.0f

/* Step the renderer walks along each rope segment when filling in beads, px.
   Must be < CELL_W (8) so no cell on a near-horizontal segment is skipped;
   2 is fine even for a vertical segment. */
#define DRAW_STEP_PX   2.0f

/* Timing. TICK_NS(f) = nanoseconds per physics tick at f Hz; used by the
   fixed-step loop in §8. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* A terminal cell is taller than it is wide. Physics runs in a square pixel
   space scaled by these so x and y measure the same real distance; positions
   become (col,row) only at draw time. See §4. */
#define CELL_W   8    /* physical pixels per terminal column */
#define CELL_H  16    /* physical pixels per terminal row    */

/* ── §2 clock — monotonic timer + sleep ── */

/* Current time in nanoseconds. CLOCK_MONOTONIC never jumps backward, so the
   gap between two readings is always the true elapsed time. */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/* Sleep ns nanoseconds. Called before the terminal write each frame so only
   the physics time counts against the frame budget. ns <= 0 means we are
   already over budget — return at once rather than sleeping a negative time. */
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ── §3 color — ten themes, one colour per rope ── */

/*
 * Theme — a named palette: seven foreground colours, one per rope.
 *   name    — shown in the HUD.
 *   body[r] — xterm-256 colour index for rope r (becomes ncurses pair r+1).
 * Themes are designed as a gradient so the bank of ropes reads as a sweep.
 * Every colour sits in the bright half of the cube (index >= 24); the near-
 * black indices vanish under A_DIM at a rope's dim tip, so they are avoided.
 * The HUD pairs (8,9) are not part of any theme and stay fixed.
 */
typedef struct {
    const char *name;              /* HUD-displayable theme name        */
    int         body[N_PAIRS];     /* xterm-256 fg per rope (0..6)      *
                                    * pair (r+1) = body[r] at apply time */
} Theme;

/* The ten built-in palettes; columns run rope 0 → rope 6. */
static const Theme THEMES[N_THEMES] = {
    /* name       rope0  rope1  rope2  rope3  rope4  rope5  rope6 */
    {"Rainbow", {196,   208,   226,    46,    51,    27,   129}},
    {"Neon",    {201,   226,   118,   159,   213,   208,    15}},
    {"Fire",    {124,   160,   196,   202,   208,   214,   220}},
    {"Ocean",   { 24,    25,    31,    33,    39,    45,    51}},
    {"Aurora",  { 28,    34,    79,   122,   159,   165,   201}},
    {"Lava",    { 52,    88,   124,   160,   196,   202,   208}},
    {"Forest",  { 28,    34,    40,    70,    76,   106,    82}},
    {"Sunset",  { 54,    91,   128,   165,   202,   209,   208}},
    {"Ice",     {195,   159,   123,    87,    51,    45,    39}},
    {"Matrix",  { 28,    34,    40,    46,    76,    82,   118}},
};

/* Load theme idx into ncurses pairs 1..7. Takes effect next frame. Background
   -1 = the user's terminal default. Terminals with < 256 colours fall back to
   a ROYGBIV-ish set of the 8 ANSI colours. Leaves the HUD pairs alone. */
static void theme_apply(int idx)
{
    const Theme *th = &THEMES[idx];
    if (COLORS >= 256) {
        for (int p = 1; p <= N_PAIRS; p++)
            init_pair(p, th->body[p - 1], -1);
    } else {
        /* 8-colour fallback — approximate ROYGBIV order */
        init_pair(1, COLOR_RED,     -1);
        init_pair(2, COLOR_YELLOW,  -1);
        init_pair(3, COLOR_YELLOW,  -1);
        init_pair(4, COLOR_GREEN,   -1);
        init_pair(5, COLOR_CYAN,    -1);
        init_pair(6, COLOR_BLUE,    -1);
        init_pair(7, COLOR_MAGENTA, -1);
    }
}

/* One-time colour setup: turn on colour, allow -1 backgrounds, load the first
   theme, and register the two fixed HUD pairs. */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();
    theme_apply(initial_theme);

    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* ── §4 coords — turn pixel positions into terminal cells ── */

/*
 * Terminal cells are about twice as tall as they are wide. So physics runs in
 * a square pixel space (gravity and rope lengths behave the same in x and y),
 * and only here, at draw time, do we divide by the cell size to get a column
 * or row. Adding 0.5 before floor rounds to the nearest cell symmetrically;
 * plain truncation or banker's rounding would make beads flicker on cell
 * boundaries.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5 entity — the ropes, the solver, and the renderer ── */

/*
 * Vec2 — a point (or direction) in pixel space, the unit all physics uses.
 *   x grows rightward, y grows DOWNWARD (screen convention). Because y is
 *   down, gravity is a positive y, the floor is at large y, and a rope hangs
 *   toward larger y. Small struct passed by value; the compiler inlines it.
 */
typedef struct {
    float x;   /* rightward pixel coordinate */
    float y;   /* downward  pixel coordinate */
} Vec2;

/*
 * Scene — all the state for the whole bank of ropes.
 *
 * Every rope is N_SEG beads. A bead's velocity is never stored: it is just
 * pos minus old_pos (where it is now versus where it was last tick). So to
 * change a bead's motion — a bounce, a pinned anchor — you move old_pos.
 * That is the one idea the whole struct is built around.
 *
 * Three position arrays exist on purpose:
 *   pos      — where each bead is now (changes constantly within a tick).
 *   old_pos  — where it was last tick; pos minus this is the velocity.
 *   prev_pos — a snapshot taken at the start of the tick, used only by the
 *              renderer to interpolate smoothly between ticks. old_pos and
 *              prev_pos can't be merged: old_pos is rewritten many times
 *              per tick, prev_pos is frozen for the whole tick.
 *
 * Geometry per rope (anchor, lengths, wind phase) is computed once in
 * scene_init and again on resize. The wind is a single sine wave; each rope
 * just reads it with its own phase offset. theme_idx is kept across r/R so a
 * reset restarts the physics without losing the user's chosen palette.
 *
 * Stored as parallel arrays (rope index first) rather than a Rope struct
 * because the hot loops sweep all ropes one field at a time.
 */
typedef struct {
    /* Verlet two-position memory. velocity = pos - old_pos. Constraints and
       bounces write old_pos to change a bead's motion. */
    Vec2 pos     [N_ROPES][N_SEG];
    Vec2 old_pos [N_ROPES][N_SEG];

    /* Snapshot of pos at tick start; the renderer blends prev_pos -> pos. */
    Vec2 prev_pos[N_ROPES][N_SEG];

    /* Per-rope shape, fixed at init / resize. */
    Vec2  anchor      [N_ROPES];  /* ceiling point bead 0 is pinned to */
    float rope_len_px [N_ROPES];  /* total length, ~35-75% of screen height */
    float rest_len    [N_ROPES];  /* target gap between neighbours (= len/(N_SEG-1)) */
    float phase_offset[N_ROPES];  /* this rope's wind phase shift, r*2pi/N_ROPES */

    /* The wind: one sine wave, sin(wind_freq*wind_time + phase). */
    float wind_time;  /* sim seconds elapsed (frozen while paused) */
    float wind_amp;   /* peak sideways push, px/s², runtime range [0,1000] */
    float wind_freq;  /* oscillation speed, rad/s, runtime range [0.05,4.0] */

    bool  paused;     /* true = skip physics, keep drawing */
    int   theme_idx;  /* which §3 palette is active */
} Scene;

/* ── §5a rope_verlet_step — move one bead under gravity and wind ── */

/* A bead's velocity is just how far it moved last tick: now minus before. */
static inline Vec2 implicit_velocity(Vec2 pos, Vec2 old_pos)
{
    return (Vec2){ pos.x - old_pos.x, pos.y - old_pos.y };
}

/* Shrink the velocity a little each tick so motion bleeds off (air drag). */
static inline Vec2 apply_damping(Vec2 v, float damping)
{
    return (Vec2){ v.x * damping, v.y * damping };
}

/* New position = old position + velocity + acceleration*dt^2. The textbook
   1/2 factor is folded into the tuned GRAVITY and wind_amp constants. */
static inline Vec2 integrate_acceleration(Vec2 pos, Vec2 v, Vec2 a, float dt2)
{
    return (Vec2){
        pos.x + v.x + a.x * dt2,
        pos.y + v.y + a.y * dt2,
    };
}

/*
 * Advance bead s of rope r by one tick: read its velocity, damp it, add
 * gravity plus the sideways wind, then step the position forward. We must
 * copy pos into old_pos BEFORE overwriting pos, or the velocity for next
 * tick would be wrong. Bead 0 (the anchor) is never passed here; it is
 * re-pinned afterward by enforce_anchors.
 */
static void rope_verlet_step(Scene *sc, int r, int s, float wind_x, float dt)
{
    float dt2 = dt * dt;

    Vec2 v = implicit_velocity(sc->pos[r][s], sc->old_pos[r][s]);
    v      = apply_damping(v, ROPE_DAMPING);
    Vec2 a = (Vec2){ wind_x, GRAVITY };

    Vec2 cur = sc->pos[r][s];
    sc->old_pos[r][s] = cur;
    sc->pos[r][s]     = integrate_acceleration(cur, v, a, dt2);
}

/* ── §5b apply_rope_constraints — keep neighbours a fixed distance apart ── */

/*
 * The whole rope shape comes from one rule, applied to every neighbour pair:
 * "you two should be rest_len apart." Gravity and wind move the beads freely;
 * then we nudge each pair back toward that distance. One sweep is not enough
 * because fixing pair (0,1) disturbs bead 1 and breaks pair (1,2), so we
 * repeat the whole sweep N_ITERS times. More passes = stiffer, less stretchy.
 */

/* Out: the vector from a to b. Returns its length (distance between them). */
static inline float displacement_and_length(Vec2 a, Vec2 b, Vec2 *out_delta)
{
    out_delta->x = b.x - a.x;
    out_delta->y = b.y - a.y;
    return sqrtf(out_delta->x * out_delta->x +
                 out_delta->y * out_delta->y);
}

/*
 * Nudge one neighbour pair back toward rest_length. If they are too far,
 * pull them together; too close, push them apart. Each bead moves half the
 * error (the 0.5), so the pair's midpoint stays put. Skip if the two beads
 * sit on top of each other (length ~ 0) to avoid dividing by zero.
 */
static inline void relax_one_distance_constraint(Vec2 *a, Vec2 *b,
                                                 float rest_length)
{
    Vec2  delta;
    float length = displacement_and_length(*a, *b, &delta);
    if (length < 1e-6f) return;        /* degenerate: coincident */

    float fractional = (length - rest_length) / length;
    float cx         = 0.5f * fractional * delta.x;
    float cy         = 0.5f * fractional * delta.y;

    a->x += cx;  a->y += cy;
    b->x -= cx;  b->y -= cy;
}

/* One pass down rope r: nudge every neighbour pair from anchor to tip.
   Fixing one pair disturbs the next, so this pass alone leaves small errors;
   apply_rope_constraints repeats it N_ITERS times. */
static void relax_rope_chain(Scene *sc, int r)
{
    for (int s = 0; s < N_SEG - 1; s++)
        relax_one_distance_constraint(&sc->pos[r][s],
                                      &sc->pos[r][s + 1],
                                      sc->rest_len[r]);
}

/* Pin bead 0 of rope r to its ceiling point. Write BOTH pos and old_pos so
   the velocity (pos - old_pos) is zero; otherwise the next step would yank
   the anchor as if it were moving. */
static inline void pin_rope_anchor(Scene *sc, int r)
{
    sc->pos    [r][0] = sc->anchor[r];
    sc->old_pos[r][0] = sc->anchor[r];
}

/* Sweep every rope N_ITERS times, re-pinning the anchor after each sweep.
   The re-pin matters: each sweep nudges the anchor slightly, so without it
   the anchor would slowly creep away from the ceiling. */
static void apply_rope_constraints(Scene *sc)
{
    for (int iter = 0; iter < N_ITERS; iter++) {
        for (int r = 0; r < N_ROPES; r++) {
            relax_rope_chain(sc, r);
            pin_rope_anchor (sc, r);
        }
    }
}

/* ── §5c enforce_anchors — re-pin every rope's top bead ── */

/* Pin bead 0 of all ropes to the ceiling. Called after the Verlet step and
   again inside the constraint loop, so gravity/wind and constraint nudges
   never drag the anchors away. */
static void enforce_anchors(Scene *sc)
{
    for (int r = 0; r < N_ROPES; r++)
        pin_rope_anchor(sc, r);
}

/* ── §5d apply_wind — blow each rope sideways and bounce off the edges ── */

/*
 * Sideways push on rope r right now: a sine wave shared by all ropes, but
 * each rope is shifted by its own phase_offset (set evenly around the circle
 * in scene_init). Because no two ropes are at the same point in the cycle,
 * the bank sways like a Mexican wave instead of seven identical pendulums.
 * A sine (not random gusts) stays smooth, so the constraint solver never
 * blows up, and the same settings always look the same.
 */
static inline float sinusoidal_wind_acceleration_at(const Scene *sc, int r)
{
    return sc->wind_amp
         * sinf(sc->wind_time * sc->wind_freq + sc->phase_offset[r]);
}

/* Step every bead of rope r except bead 0 (the anchor, re-pinned later). */
static inline void integrate_rope_segments(Scene *sc, int r,
                                           float wind_acc, float dt)
{
    for (int s = 1; s < N_SEG; s++)
        rope_verlet_step(sc, r, s, wind_acc, dt);
}

/* Bounce a bead off a wall on one axis. Snap it onto the wall, then place
   old_pos on the far side so next tick's velocity (pos - old_pos) points away
   from the wall. BOUNCE_COEFF scales how much speed survives the bounce. */
static inline void bounce_against_wall_1d(float *pos, float *old_pos,
                                          float wall)
{
    float v_axis = *pos - *old_pos;
    *pos     = wall;
    *old_pos = wall + v_axis * BOUNCE_COEFF;
}

/* Keep rope r's beads inside the screen: bounce off the floor and the two
   side walls. No top wall is needed — the anchor already holds the top. */
static inline void bounce_rope_against_screen_bounds(Scene *sc, int r,
                                                     int cols, int rows)
{
    float floor_y = (float)(rows * CELL_H) - FLOOR_MARGIN;
    float left_x  = LEFT_MARGIN;
    float right_x = (float)(cols * CELL_W) - RIGHT_MARGIN;

    for (int s = 1; s < N_SEG; s++) {
        if (sc->pos[r][s].y > floor_y)
            bounce_against_wall_1d(&sc->pos[r][s].y, &sc->old_pos[r][s].y, floor_y);
        if (sc->pos[r][s].x < left_x)
            bounce_against_wall_1d(&sc->pos[r][s].x, &sc->old_pos[r][s].x, left_x);
        if (sc->pos[r][s].x > right_x)
            bounce_against_wall_1d(&sc->pos[r][s].x, &sc->old_pos[r][s].x, right_x);
    }
}

/* One physics tick for rope r: pick this tick's wind, move every bead under
   gravity + that wind, then bounce any bead that left the screen. The same
   wind value is used for the whole rope (one uniform horizontal gust). */
static void apply_wind(Scene *sc, int r, float dt, int cols, int rows)
{
    float wind_acc = sinusoidal_wind_acceleration_at(sc, r);
    integrate_rope_segments(sc, r, wind_acc, dt);
    bounce_rope_against_screen_bounds(sc, r, cols, rows);
}

/* ── §5e rope_node_char / _attr — bead glyph + brightness by position ── */

/*
 * Pick the glyph for bead s by where it sits on the rope: a fat '0' in the
 * top quarter, a faint '.' in the bottom quarter, plain 'o' in between. This
 * mimics tension — the top holds all the weight below it, the tip holds none.
 */
static chtype rope_node_char(int s)
{
    if (s < N_SEG / 4)       return (chtype)'0';
    if (s >= N_SEG * 3 / 4)  return (chtype)'.';
    return (chtype)'o';
}

/* Matching brightness for bead s: bold at the top, dim at the tip. */
static attr_t rope_node_attr(int s)
{
    if (s < N_SEG / 4)       return A_BOLD;
    if (s >= N_SEG * 3 / 4)  return A_DIM;
    return A_NORMAL;
}

/* Stamp one glyph at cell (cx,cy), or skip it silently if off-screen. The
   (chtype)(unsigned char) cast avoids sign-extension on bytes above 127. */
static void mark_cell(WINDOW *w, int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/* ── §5f draw_rope_beads — render one rope ── */

/*
 * Draw rope r in two passes. Pass 1 walks each segment in small steps and
 * fills the cells between beads with 'o' so the rope looks continuous. Pass 2
 * stamps the per-bead glyphs on top, plus a bright 'o' on the tip as a hanging
 * weight. Pass 2 must come last so the beads always win over the fill.
 */
static void draw_rope_beads(const Scene *sc, const Vec2 rp[][N_SEG],
                            int r, WINDOW *w, int cols, int rows)
{
    (void)sc;                         /* colour pair derived from r, not sc  */
    int cpair = (r % N_PAIRS) + 1;    /* cycle rope colours through pairs 1–7 */

    /* Pass 1 — fill the gaps between beads with 'o' */
    for (int s = 0; s < N_SEG - 1; s++) {
        float dx  = rp[r][s+1].x - rp[r][s].x;
        float dy  = rp[r][s+1].y - rp[r][s].y;
        float len = sqrtf(dx * dx + dy * dy);
        if (len < 0.1f) continue;   /* segment too short to fill */

        int nsteps  = (int)ceilf(len / DRAW_STEP_PX) + 1;
        int prev_cx = -9999, prev_cy = -9999;

        for (int t = 0; t <= nsteps; t++) {
            float u  = (float)t / (float)nsteps;
            int   cx = px_to_cell_x(rp[r][s].x + dx * u);
            int   cy = px_to_cell_y(rp[r][s].y + dy * u);

            /* skip if this step lands on the same cell as the last one,
               else the double attr on/off would flicker */
            if (cx == prev_cx && cy == prev_cy) continue;
            prev_cx = cx;  prev_cy = cy;
            mark_cell(w, cx, cy, 'o', cpair, A_NORMAL, cols, rows);
        }
    }

    /* Pass 2 — stamp the per-bead glyph over the fill */
    for (int s = 0; s < N_SEG; s++) {
        int cx = px_to_cell_x(rp[r][s].x);
        int cy = px_to_cell_y(rp[r][s].y);
        char   ch   = (char)rope_node_char(s);
        attr_t attr = rope_node_attr(s);
        mark_cell(w, cx, cy, ch, cpair, attr, cols, rows);
    }

    /* Bright 'o' on the free tip — the hanging weight */
    {
        int cx = px_to_cell_x(rp[r][N_SEG - 1].x);
        int cy = px_to_cell_y(rp[r][N_SEG - 1].y);
        mark_cell(w, cx, cy, 'o', cpair, A_BOLD, cols, rows);
    }
}

/* ── §5g render_scene — paint the whole frame ── */

/*
 * Fill rp[][] with each bead drawn partway between its last and current
 * position. alpha (0..1) is how far the next tick is along; blending by it
 * keeps motion smooth even when the draw rate and physics rate differ.
 */
static void lerp_positions(const Scene *sc, float alpha,
                           Vec2 rp[N_ROPES][N_SEG])
{
    for (int r = 0; r < N_ROPES; r++) {
        for (int s = 0; s < N_SEG; s++) {
            rp[r][s].x = sc->prev_pos[r][s].x
                       + (sc->pos[r][s].x - sc->prev_pos[r][s].x) * alpha;
            rp[r][s].y = sc->prev_pos[r][s].y
                       + (sc->pos[r][s].y - sc->prev_pos[r][s].y) * alpha;
        }
    }
}

/* Draw the '#' ceiling line the ropes hang from. Dim, so it reads as scenery,
   and drawn first so rope cells over-stamp it. */
static void draw_ceiling(WINDOW *w, int cols, int rows)
{
    if (ANCHOR_ROW_CELLS < 0 || ANCHOR_ROW_CELLS >= rows) return;
    for (int cx = 0; cx < cols; cx++) {
        mark_cell(w, cx, ANCHOR_ROW_CELLS, '#',
                  N_PAIRS, A_DIM, cols, rows);
    }
}

/* Paint one frame back-to-front: interpolate positions, draw the ceiling,
   then draw every rope on top. */
static void render_scene(const Scene *sc, WINDOW *w,
                         int cols, int rows, float alpha)
{
    Vec2 rp[N_ROPES][N_SEG];
    lerp_positions(sc, alpha, rp);

    draw_ceiling(w, cols, rows);
    for (int r = 0; r < N_ROPES; r++) {
        draw_rope_beads(sc, rp, r, w, cols, rows);
    }
}

/* ── §6 scene — set up, advance, and draw the simulation ── */

/*
 * Build all seven ropes hanging straight down at rest. Called at startup, on
 * r/R reset, and on resize (it reads the current terminal size). The theme is
 * saved across the memset so a reset keeps the user's colours. Anchors are
 * spread evenly across the width; rope lengths fan from 35% to 75% of the
 * screen height so each rope swings at a slightly different rate (a shorter
 * rope swings faster, like a shorter pendulum) and the bank looks lively.
 * Each bead starts with old_pos = pos, i.e. zero velocity.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    int saved_theme = sc->theme_idx;   /* preserve theme across reset */
    memset(sc, 0, sizeof *sc);
    sc->theme_idx = saved_theme;

    sc->wind_amp  = WIND_AMP_DEFAULT;
    sc->wind_freq = WIND_FREQ_DEFAULT;
    sc->wind_time = 0.0f;
    sc->paused    = false;

    float screen_px_h = (float)(rows * CELL_H);
    float screen_px_w = (float)(cols * CELL_W);
    float anchor_py   = (float)(ANCHOR_ROW_CELLS * CELL_H);

    /* Rope lengths span 35%–75% of screen height */
    float min_len = screen_px_h * 0.35f;
    float max_len = screen_px_h * 0.75f;

    for (int r = 0; r < N_ROPES; r++) {
        /* Evenly spaced ceiling anchor positions */
        sc->anchor[r].x = (float)(r + 1) * screen_px_w / (float)(N_ROPES + 1);
        sc->anchor[r].y = anchor_py;

        /* Rope length linearly interpolated from min to max */
        float len = min_len
                  + (float)r * (max_len - min_len) / (float)(N_ROPES - 1);
        sc->rope_len_px[r] = len;
        sc->rest_len[r]    = len / (float)(N_SEG - 1);

        /* Phase offset: distribute evenly over 2π */
        sc->phase_offset[r] = (float)r * 2.0f * (float)M_PI / (float)N_ROPES;

        /* Particles hang vertically; zero initial velocity */
        for (int s = 0; s < N_SEG; s++) {
            sc->pos[r][s].x     = sc->anchor[r].x;
            sc->pos[r][s].y     = sc->anchor[r].y + (float)s * sc->rest_len[r];
            sc->old_pos[r][s]   = sc->pos[r][s];
            sc->prev_pos[r][s]  = sc->pos[r][s];
        }
    }
}

/*
 * One physics tick: snapshot positions, move everything, then fix the shape.
 * The order matters. Save prev_pos first (the renderer blends from it; if we
 * saved it after moving, the rope would visually overshoot). Then, if not
 * paused, advance the wind, integrate every rope under gravity + wind, pin
 * the anchors, and finally relax the distance constraints. This integrate-
 * then-correct sequence is the standard position-based dynamics loop.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    /* snapshot for the renderer's between-tick blend */
    memcpy(sc->prev_pos, sc->pos, sizeof sc->pos);

    if (sc->paused) return;

    sc->wind_time += dt;

    /* move every rope, then pin the tops back to the ceiling */
    for (int r = 0; r < N_ROPES; r++) {
        apply_wind(sc, r, dt, cols, rows);
    }
    enforce_anchors(sc);

    /* pull the beads back to their rest spacing (re-pins anchors inside) */
    apply_rope_constraints(sc);
}

/* Draw all ropes for this frame. alpha is the between-tick blend factor;
   dt_sec is unused here. */
static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)dt_sec;
    render_scene(sc, w, cols, rows, alpha);
}

/* ── §7 screen — ncurses display layer ── */

/*
 * Screen — the current terminal size in character cells, cached so the draw
 * code reads two plain ints instead of querying ncurses every frame. Updated
 * only on resize, then fed to scene_init so the ropes rescale with the window.
 */
typedef struct {
    int cols;   /* terminal width  in character cells */
    int rows;   /* terminal height in character cells */
} Screen;

/*
 * Put the terminal into animation mode: no echo, no line buffering, no
 * visible cursor, non-blocking key reads, decoded arrow keys. typeahead(-1)
 * is the important one — it stops ncurses peeking at input mid-write, which
 * would tear frames. Then load theme 0 and record the terminal size.
 */
static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(0);
    getmaxyx(stdscr, s->rows, s->cols);
}

/* Restore the terminal (echo, cursor, normal mode). */
static void screen_free(Screen *s) { (void)s; endwin(); }

/* React to a terminal resize. endwin()+refresh() makes ncurses re-read the new
   size; without it, draws at the new coordinates silently fail. The caller
   then re-inits the scene for the new dimensions. */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* Compose one frame in memory: clear, draw the scene, then draw the two HUD
   bars on top so they always win over rope cells on those rows. Nothing
   reaches the terminal until screen_present(). */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    /* Top-right status — PAIR_HUD bright yellow, A_BOLD */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%3d Hz  wind:%.0f  freq:%.2f  [%s]  %s ",
             fps, sim_fps,
             sc->wind_amp, sc->wind_freq,
             THEMES[sc->theme_idx].name,
             sc->paused ? "PAUSED " : "swaying");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom-left key hint — PAIR_HINT bright cyan, A_BOLD */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset  w/s:wind  a/d:freq  t:theme  [/]:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* Push the composed frame to the terminal. Two-step (queue then flush) so
   only the cells that changed since last frame are actually written. */
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §8 app — signals, resize, and the main loop ── */

/*
 * App — everything the program owns: the world, the terminal, the loop flags.
 * It is a file-scope global (g_app) because signal handlers take no argument
 * and must reach `running` and `need_resize`. Those two are volatile
 * sig_atomic_t: volatile so the loop re-reads them after a handler writes,
 * sig_atomic_t so a handler's write is never seen half-done.
 */
typedef struct {
    Scene  scene;              /* the simulated world (§6) */
    Screen screen;             /* terminal extent (§7) */

    int    sim_fps;            /* physics tick rate, Hz, cycled with [ / ] */

    volatile sig_atomic_t running;      /* cleared by SIGINT/SIGTERM → exit */
    volatile sig_atomic_t need_resize;  /* set by SIGWINCH, cleared on resize */
} App;

static App g_app;

/* Signal handlers — set flags only; no ncurses or malloc calls here.
 * Signal handlers have severe restrictions (async-signal safety); setting
 * a volatile sig_atomic_t flag is one of the few safe operations. */
static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/* atexit safety net so the terminal is restored even on an unclean exit. */
static void cleanup(void) { endwin(); }

/* Handle a pending resize: re-read the terminal size, then rebuild the ropes
   for it (simpler and cleaner than relocating scattered beads). The caller
   resets its timers afterward so the resize gap doesn't trigger a tick flood. */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

/*
 * app_handle_key() — process one keypress; return false to quit.
 *
 * WHAT: Maps key codes to parameter mutations.  No physics logic here —
 *   only clamp + assign.  All clamping uses explicit if-guards rather than
 *   fmaxf/fminf so the logic is easy to follow and modify.
 *
 * KEY MAP:
 *   q / Q / ESC   — quit: return false → main loop exits.
 *   space         — toggle pause (freezes physics, rendering continues).
 *   r / R         — full reset: call scene_init() with current dimensions.
 *                   Theme is preserved by scene_init()'s save/restore logic.
 *   w / ↑         — wind_amp += 50 px/s²  [0, 1000]
 *                   More amplitude → wider lateral swing.
 *   s / ↓         — wind_amp -= 50 px/s²  [0, 1000]
 *                   Zero amplitude → gravity only; ropes hang straight.
 *   d / →         — wind_freq += 0.05 rad/s  [0.05, 4.0]
 *                   Higher frequency → faster oscillation tempo.
 *   a / ←         — wind_freq -= 0.05 rad/s  [0.05, 4.0]
 *                   Minimum 0.05 prevents division-by-zero (not used here)
 *                   and ensures a perceptible wind oscillation.
 *   t             — next colour theme (wraps 0 → N_THEMES-1 → 0).
 *   T             — previous colour theme (wraps 0 → N_THEMES-1 via modular
 *                   arithmetic: (idx − 1 + N_THEMES) % N_THEMES avoids
 *                   negative modulo which is implementation-defined in C).
 *   ] / + / =     — sim_fps += SIM_FPS_STEP  [SIM_FPS_MIN, SIM_FPS_MAX]
 *                   More physics ticks per second → tighter constraints.
 *   [ / -         — sim_fps -= SIM_FPS_STEP
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ':
        sc->paused = !sc->paused;
        break;

    case 'r': case 'R':
        scene_init(sc, app->screen.cols, app->screen.rows);
        break;

    /* Wind amplitude — controls the peak lateral force on each particle */
    case 'w': case KEY_UP:
        sc->wind_amp += 50.0f;
        if (sc->wind_amp > 1000.0f) sc->wind_amp = 1000.0f;
        break;
    case 's': case KEY_DOWN:
        sc->wind_amp -= 50.0f;
        if (sc->wind_amp < 0.0f) sc->wind_amp = 0.0f;
        break;

    /* Wind frequency — controls the oscillation tempo */
    case 'd': case KEY_RIGHT:
        sc->wind_freq += 0.05f;
        if (sc->wind_freq > 4.0f) sc->wind_freq = 4.0f;
        break;
    case 'a': case KEY_LEFT:
        sc->wind_freq -= 0.05f;
        if (sc->wind_freq < 0.05f) sc->wind_freq = 0.05f;
        break;

    /* Colour themes — t cycles forward, T cycles backward */
    case 't':
        sc->theme_idx = (sc->theme_idx + 1) % N_THEMES;
        theme_apply(sc->theme_idx);
        break;
    case 'T':
        sc->theme_idx = (sc->theme_idx - 1 + N_THEMES) % N_THEMES;
        theme_apply(sc->theme_idx);
        break;

    /* Simulation Hz — affects constraint convergence and physics accuracy */
    case ']': case '+': case '=':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[': case '-':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    default: break;
    }
    return true;
}

/* ─────────────────────────────────────────────────────────────────────
 * main() — the game loop  (structure identical to framework.c §8)
 *
 * The loop body executes these eight steps every frame:
 *
 *  ① RESIZE CHECK
 *     Test need_resize (set by SIGWINCH handler) before touching ncurses.
 *     app_do_resize() re-reads terminal size and re-inits the scene.
 *     frame_time and sim_accum are reset immediately after to prevent the
 *     large dt that accumulated during the resize from firing a burst of
 *     physics ticks (a "physics avalanche").
 *
 *  ② MEASURE dt
 *     Wall-clock nanoseconds elapsed since the previous frame.
 *     Capped at 100 ms: if the process was suspended (Ctrl-Z, debugger,
 *     sleep) and resumed, an uncapped dt would fire up to 6 physics ticks
 *     in one frame at 60 Hz × 100 ms — a sudden visual jump.
 *
 *  ③ FIXED-STEP ACCUMULATOR
 *     sim_accum accumulates wall-clock dt each frame.
 *     While sim_accum ≥ tick_ns (period of one physics tick):
 *       fire one scene_tick() and drain tick_ns from sim_accum.
 *     Result: physics runs at exactly sim_fps Hz on average, completely
 *     decoupled from the render frame rate.  If the render is slow
 *     (say 30 fps), two physics ticks fire per render frame.  If the
 *     render is fast (120 fps), one physics tick fires every two frames.
 *
 *  ④ ALPHA — sub-tick interpolation factor
 *     After draining, sim_accum holds the fractional leftover — how far
 *     into the next unfired tick we are.
 *       alpha = sim_accum / tick_ns  ∈ [0, 1)
 *     Passed to render_scene() so particle positions are lerped between
 *     the last physics state and the predicted next state, eliminating
 *     the periodic micro-stutter visible when sim Hz < render Hz.
 *
 *  ⑤ FPS COUNTER
 *     Frames counted over a 500 ms sliding window.
 *     fps = frame_count / (fps_accum_s).
 *     The 500 ms window gives a stable display without per-frame jitter.
 *
 *  ⑥ FRAME CAP — sleep BEFORE render
 *     budget  = NS_PER_SEC / 60   (one 60-fps frame in nanoseconds).
 *     elapsed = time spent on physics since frame_time was last sampled.
 *     sleep   = budget − elapsed.
 *     Sleeping BEFORE the ncurses write means terminal I/O cost is not
 *     charged against the next frame's budget.  If sleep ≤ 0 (frame
 *     already over budget), clock_sleep_ns() returns immediately.
 *
 *  ⑦ DRAW + PRESENT
 *     erase() → scene_draw() → HUD → wnoutrefresh() → doupdate().
 *     One atomic diff write to the terminal fd; no partial frames visible.
 *
 *  ⑧ DRAIN INPUT
 *     Loop getch() until ERR, processing every queued keypress.
 *     Looping (not single-call) drains all key-repeat events within the
 *     same frame, keeping parameter adjustments responsive when held.
 * ───────────────────────────────────────────────────────────────────── */
int main(void)
{
    /* Seed RNG from monotonic clock so each run looks different */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));

    /* Safety net: endwin() even if we exit via an unhandled path */
    atexit(cleanup);

    /* SIGINT / SIGTERM — graceful exit from Ctrl-C or kill */
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);

    /* SIGWINCH — terminal resize; handled at the top of the next iteration */
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();   /* timestamp at start of last frame */
    int64_t sim_accum   = 0;            /* nanoseconds in the physics bucket */
    int64_t fps_accum   = 0;            /* ns elapsed in current fps window  */
    int     frame_count = 0;            /* frames rendered in fps window     */
    double  fps_display = 0.0;          /* smoothed fps shown in HUD         */

    while (app->running) {

        int64_t frame_start = clock_ns();

        /* ── ① resize ────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();   /* reset so dt doesn't spike         */
            sim_accum  = 0;
        }

        /* ── ② dt ────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;   /* suspend guard */

        /* ── ③ fixed-step accumulator ────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);   /* ns per physics tick   */
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
         * render rate sits at 60 fps regardless of sim Hz.            */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── ⑦ draw + present ────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
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
