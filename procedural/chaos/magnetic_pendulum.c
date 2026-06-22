/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * magnetic_pendulum.c — a damped bob swings over three magnets and
 * always settles on one of them; with light friction, tiny changes in
 * where you drop it send it to wildly different magnets (chaos).
 * Side-view animation: a bob on a rod hanging from a top pivot.
 *
 * The system and the fractal "which magnet wins" boundary: Moon,
 * "Chaotic and Fractal Dynamics" (1992) §4.7; Aguirre & Sanjuan,
 * Phys. Rev. E 67(5) (2003).  Sister files: ../fractals/newton_fractal.c
 * (same basin idea, different system) and ./double_pendulum.c.
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

/* ── §1 config ── */

enum {
    MAP_W_MAX        = 200, MAP_H_MAX = 56,
    SIM_FPS_MIN      =  10, SIM_FPS_DEFAULT = 60, SIM_FPS_MAX = 240, SIM_FPS_STEP = 10,
    HUD_COLS         =  80, FPS_UPDATE_MS = 500,
    PAIR_HUD         =   1, PAIR_HINT = 2,
    /* The whole apparatus is one accent colour in four brightnesses
     * (dim old trail up to bright bob/magnets).  All three magnets
     * share that colour so nothing competes with the swinging bob. */
    PAIR_TIER_BASE   =   3,
};
#define N_TIERS                   4
#define PAIR_TIER(i)              (PAIR_TIER_BASE + (i))
#define PAIR_TIER_BRIGHTEST       PAIR_TIER(N_TIERS - 1)
#define PAIR_TIER_DIM             PAIR_TIER(1)

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

/* Physics */
#define N_MAGNETS                 3
#define MAGNET_RADIUS             1.0f    /* magnets sit on a unit circle */
#define SPRING_K                  0.50f
#define MAGNET_EPS                0.10f
#define SIM_DT                    0.05f
#define MAX_STEPS              2000        /* give up if it never settles */
#define CONV_V_SQ                 0.001f   /* "barely moving" speed cutoff */
#define CONV_R_SQ                 0.05f    /* "close enough to a magnet" cutoff */

/* The slice of the plane we show; the bob lives in these bounds. */
#define X_MIN                    -1.7f
#define X_MAX                     1.7f
#define Y_MIN                    -1.5f
#define Y_MAX                     1.5f

/* Pendulum animation settings.  Once the bob is caught by a magnet it
 * pauses, then a new one is dropped from a random spot; never stops. */
#define LIVE_TRAIL_MAX              700     /* how many past points to remember */
#define LIVE_TRAIL_BAND_COUNT         4     /* fade in 4 brightness steps */
/* How many physics steps to run per drawn frame.  Two keeps the swing
 * smooth and watchable (no teleporting) while still letting a typical
 * orbit settle in roughly 10-20 real seconds. */
#define LIVE_BOB_SUBSTEPS_PER_TICK    2
#define LIVE_BOB_RESPAWN_FRAMES     120     /* ~2 sec pause so you can see who won */
#define LIVE_BOB_WINNER_NONE         -1     /* nobody yet; still swinging */

/* The visible rod and pivot.  The physics only knows a bob being
 * pulled toward the centre, but it reads better as a pendulum hanging
 * from the ceiling: so we draw a fake pivot at the top-centre and a
 * rod from there down to wherever the bob actually is. */
#define ANCHOR_GLYPH               '+'
#define ANCHOR_TOP_OFFSET          1        /* pivot one cell down so it clears the HUD */
#define ROD_GLYPH_HORIZ            '-'
#define ROD_GLYPH_VERT             '|'
#define ROD_GLYPH_DIAG_DOWNRIGHT   '\\'
#define ROD_GLYPH_DIAG_UPRIGHT     '/'

/* The bob: bright while swinging, an O once it sticks to a magnet. */
#define LIVE_BOB_GLYPH_MOVING      '@'
#define LIVE_BOB_GLYPH_SETTLED     'O'

/* Trail characters from faintest to heaviest; paired with the four
 * brightness steps this gives a smooth-looking fade. */
#define LIVE_TRAIL_GLYPH_TIER0     ','
#define LIVE_TRAIL_GLYPH_TIER1     '.'
#define LIVE_TRAIL_GLYPH_TIER2     ':'
#define LIVE_TRAIL_GLYPH_TIER3     '+'

/* Magnets are drawn as [1] [2] [3] so you can read which one is which,
 * matching the "absorbed -> magnet N" message in the status bar. */
#define MAGNET_GLYPH_LEFT          '['
#define MAGNET_GLYPH_RIGHT         ']'
#define MAGNET_GLYPH_CENTRE_FOR(i) ((char)('1' + (i)))

/* The two friction settings you can toggle with n/p.  WEAK lets the
 * bob loop around for ages before settling, so tiny changes in where
 * you drop it pick different magnets (the chaotic case).  STRONG makes
 * it drop almost straight onto the nearest magnet (boring but clean). */
typedef enum {
    PRESET_WEAK_DAMP = 0,
    PRESET_STRONG_DAMP,
    N_PRESETS
} Preset;

/* One named friction setting in the presets[] table below.  Friction
 * is the only thing the user actually changes; everything else about
 * the apparatus is fixed.  Giving each value a name lets the n/p keys
 * cycle "WEAK / STRONG" instead of bare numbers.
 *
 *   name  : label shown in the status bar (8 chars wide).
 *   gamma : the friction amount.  These two values bracket the range
 *           worth watching: much lower and it never settles, much
 *           higher and there's nothing interesting to see.
 */
typedef struct {
    const char *name;
    float       gamma;
} PendPreset;
static const PendPreset presets[N_PRESETS] = {
    { "WEAK    ", 0.05f },
    { "STRONG  ", 0.30f },
};

/* One colour scheme: four shades of a single colour, dim to bright.
 * Everything on screen uses these four shades and nothing else, so the
 * whole picture reads as one colour at different strengths.  The t/T
 * keys swap which colour family it is (greys, greens, cyan, orange...).
 *
 *   name      : label shown in the status bar.
 *   tier[0..3]: the four shades, faintest first.  The dimmest is for
 *               the oldest trail, the brightest for the bob, magnets
 *               and pivot.  Every value is kept fairly bright (xterm
 *               index >= 24) so even the faintest stays visible on a
 *               black background — see CLAUDE.md "Theme Palette
 *               Brightness".
 */
typedef struct {
    const char *name;
    short       tier[N_TIERS];
} Theme;

#define N_THEMES 10
static const Theme themes[N_THEMES] = {
    /*                  dim →                              → bright */
    { "DEFAULT",   { 240, 244, 248, 252 } },   /* greyscale          */
    { "MATRIX",    {  28,  34,  82, 118 } },   /* greens             */
    { "NOVA",      {  90, 126, 162, 207 } },   /* magenta            */
    { "MONO",      { 240, 245, 250, 255 } },   /* pure greyscale     */
    { "OCEAN",     {  24,  31,  39,  51 } },   /* cyans              */
    { "FIRE",      {  94, 130, 208, 220 } },   /* warm orange ramp   */
    { "EARTH",     {  94, 130, 173, 215 } },   /* terracotta         */
    { "FOREST",    {  22,  28,  64, 156 } },   /* deep green ramp    */
    { "DESERT",    {  94, 130, 180, 222 } },   /* tan / dust          */
    { "ARCTIC",    {  24,  31,  81, 195 } },   /* pale blue          */
};

/* ── §2 clock + §3 color ── */

static int64_t clock_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec; }
static void clock_sleep_ns(int64_t ns) { if (ns <= 0) return;
    struct timespec req = {ns/NS_PER_SEC, ns%NS_PER_SEC}; nanosleep(&req, NULL); }

/* The bright yellow / bright cyan used for the status text. */
#define PAIR_HUD_FG_256    226
#define PAIR_HINT_FG_256    51

static void theme_apply_tiers_256color(const Theme *t)
{
    for (int i = 0; i < N_TIERS; i++)
        init_pair(PAIR_TIER(i), t->tier[i], -1);
}

/* On old 8-colour terminals we lose the fade and just go white. */
static void theme_apply_tiers_8color_fallback(void)
{
    for (int i = 0; i < N_TIERS; i++)
        init_pair(PAIR_TIER(i), COLOR_WHITE, -1);
}

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_apply_tiers_256color(&themes[idx]);
    else               theme_apply_tiers_8color_fallback();
}

/* The status-bar colours stay fixed as themes change, so the bar
 * never blends into the animation. */
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

/* ── §5 physics: damped pendulum + magnet pull ── */

/* The bob feels three pulls added together: a gentle spring toward the
 * centre, friction that opposes its motion, and an attraction to each
 * magnet that gets stronger the closer it is.  pendulum_deriv() below
 * builds the acceleration as exactly those three pieces. */

/* One magnet, at a fixed spot.  It only needs its position; how hard
 * it pulls is baked into the shared force formula.  The three magnets
 * sit at the corners of a triangle around the centre (Moon §4.7). */
typedef struct {
    float x, y;
} Magnet;

/* All the magnets together.  Keeping them in one struct means the
 * force code just loops "for each magnet", and adding or removing a
 * magnet only touches this, not the physics.
 *
 *   count : how many magnets are actually in use (at most N_MAGNETS).
 *   m     : their positions; only the first `count` are live.
 */
typedef struct {
    int    count;
    Magnet m[N_MAGNETS];
} MagnetField;

/* The fixed dials of the apparatus (these don't change as it runs).
 *
 *   spring_k  : how strongly the centre pulls the bob back; keeps its
 *               resting spot at the middle.
 *   damping   : how much friction there is.  The one dial the user
 *               actually changes (WEAK / STRONG via n/p).
 *   softening : a small fudge so the magnet pull doesn't blow up to
 *               infinity if the bob passes right over a magnet.
 *   magnets   : where the magnets are.
 */
typedef struct {
    float       spring_k;
    float       damping;
    float       softening;
    MagnetField magnets;
} PendulumSystem;

/* Everything that changes about the bob from moment to moment: where
 * it is and how fast it's going.  It's a plain by-value struct so the
 * step solver can make and combine cheap copies of it.
 *
 *   x, y   : the bob's position.
 *   vx, vy : its speed and direction.  Friction shrinks this toward
 *            zero; once it's small enough (CONV_V_SQ) and the bob is
 *            near a magnet, we call it settled.
 */
typedef struct {
    float x, y;
    float vx, vy;
} BobState;

/* The whole pendulum: its fixed dials plus its current motion.  Kept
 * together so callers pass one thing.  Stepping the physics changes
 * only .state; changing the friction preset changes only .system. */
typedef struct {
    PendulumSystem system;
    BobState       state;
} Pendulum;

/* The three magnets, evenly spaced on a circle round the centre, one
 * at the top. */
static MagnetField magnet_field_equilateral_triangle(void)
{
    MagnetField mf;
    mf.count = N_MAGNETS;
    for (int i = 0; i < N_MAGNETS; i++) {
        float angle_deg = 90.0f + 120.0f * (float)i;
        float angle_rad = angle_deg * (float)M_PI / 180.0f;
        mf.m[i].x = MAGNET_RADIUS * cosf(angle_rad);
        mf.m[i].y = MAGNET_RADIUS * sinf(angle_rad);
    }
    return mf;
}

/* Friction is left at zero here; the active preset fills it in. */
static PendulumSystem pendulum_system_init_default(void)
{
    PendulumSystem sys;
    sys.spring_k  = SPRING_K;
    sys.damping   = 0.0f;                        /* set per-preset */
    sys.softening = MAGNET_EPS;
    sys.magnets   = magnet_field_equilateral_triangle();
    return sys;
}

/* Adds up the pull from every magnet into (ax, ay): each magnet tugs
 * the bob toward itself, harder the closer it is.  eps_sq is the small
 * softening fudge, passed in already squared. */
static inline void magnet_field_net_attraction(const MagnetField *mf,
                                               float x, float y,
                                               float eps_sq,
                                               float *ax, float *ay)
{
    *ax = 0.0f;
    *ay = 0.0f;
    for (int i = 0; i < mf->count; i++) {
        float dx = x - mf->m[i].x;
        float dy = y - mf->m[i].y;
        float r2_softened = dx * dx + dy * dy + eps_sq;
        float inv_r3      = 1.0f / (r2_softened * sqrtf(r2_softened));
        *ax -= dx * inv_r3;
        *ay -= dy * inv_r3;
    }
}

/* Works out how the bob's state is changing right now: position
 * changes at the current speed, and speed changes by the three pulls
 * (spring + friction + magnets) added together.  This is the heart of
 * the physics; the solver below calls it several times per step. */
static inline BobState pendulum_deriv(const BobState *s,
                                      const PendulumSystem *sys)
{
    /* spring: pulls back toward the centre */
    float spring_ax  = -sys->spring_k * s->x;
    float spring_ay  = -sys->spring_k * s->y;

    /* friction: pushes back against whichever way it's moving */
    float damping_ax = -sys->damping  * s->vx;
    float damping_ay = -sys->damping  * s->vy;

    /* the combined pull of all the magnets */
    float magnet_ax, magnet_ay;
    magnet_field_net_attraction(&sys->magnets, s->x, s->y,
                                sys->softening * sys->softening,
                                &magnet_ax, &magnet_ay);

    BobState dy;
    dy.x  = s->vx;
    dy.y  = s->vy;
    dy.vx = spring_ax + damping_ax + magnet_ax;
    dy.vy = spring_ay + damping_ay + magnet_ay;
    return dy;
}

/* Returns a + h*k as a fresh state; the solver uses this to build its
 * trial points. */
static inline BobState state_add(const BobState *a, float h, const BobState *k)
{
    BobState r;
    r.x  = a->x  + h * k->x;
    r.y  = a->y  + h * k->y;
    r.vx = a->vx + h * k->vx;
    r.vy = a->vy + h * k->vy;
    return r;
}

#define RK4_BUTCHER_WEIGHT_SUM   6.0f
#define RK4_MIDPOINT_FRACTION    0.5f

/* Blends the four trial slopes into one good estimate, weighting the
 * two middle ones double.  This is the standard RK4 recipe (Numerical
 * Recipes 17.1) that makes each step accurate. */
static inline BobState rk4_butcher_weighted_average(
    const BobState *k1, const BobState *k2,
    const BobState *k3, const BobState *k4)
{
    BobState avg;
    avg.x  = (k1->x  + 2.0f*k2->x  + 2.0f*k3->x  + k4->x ) / RK4_BUTCHER_WEIGHT_SUM;
    avg.y  = (k1->y  + 2.0f*k2->y  + 2.0f*k3->y  + k4->y ) / RK4_BUTCHER_WEIGHT_SUM;
    avg.vx = (k1->vx + 2.0f*k2->vx + 2.0f*k3->vx + k4->vx) / RK4_BUTCHER_WEIGHT_SUM;
    avg.vy = (k1->vy + 2.0f*k2->vy + 2.0f*k3->vy + k4->vy) / RK4_BUTCHER_WEIGHT_SUM;
    return avg;
}

/* Moves the bob forward by one small time step.  It samples how things
 * are changing at four points across the step (start, two midpoints,
 * end) and blends them, which is far more accurate than a single
 * guess.  This is the classic RK4 method (Numerical Recipes 17.1). */
static void pendulum_rk4(Pendulum *p, float dt)
{
    const PendulumSystem *sys = &p->system;
    float half_dt = RK4_MIDPOINT_FRACTION * dt;

    BobState slope_start    = pendulum_deriv(&p->state, sys);
    BobState midpoint_1     = state_add(&p->state, half_dt, &slope_start);
    BobState slope_mid_1    = pendulum_deriv(&midpoint_1, sys);
    BobState midpoint_2     = state_add(&p->state, half_dt, &slope_mid_1);
    BobState slope_mid_2    = pendulum_deriv(&midpoint_2, sys);
    BobState endpoint       = state_add(&p->state, dt, &slope_mid_2);
    BobState slope_end      = pendulum_deriv(&endpoint, sys);

    BobState effective_slope = rk4_butcher_weighted_average(
        &slope_start, &slope_mid_1, &slope_mid_2, &slope_end);
    p->state = state_add(&p->state, dt, &effective_slope);
}

/* Places the bob at (x, y) and lets go: starts it at rest.  Which
 * magnet it ends up on is "the answer" for this drop point. */
static inline void pendulum_state_release_at(BobState *s, float x, float y)
{
    s->x  = x;
    s->y  = y;
    s->vx = 0.0f;
    s->vy = 0.0f;
}

/* Has the bob settled? It counts as caught when it's barely moving AND
 * sitting right next to a magnet.  Returns that magnet's number, or
 * LIVE_BOB_WINNER_NONE if it's still going. */
static int pendulum_test_convergence(const BobState *s, const MagnetField *mf)
{
    if (s->vx * s->vx + s->vy * s->vy >= CONV_V_SQ)
        return LIVE_BOB_WINNER_NONE;
    for (int i = 0; i < mf->count; i++) {
        float dx = s->x - mf->m[i].x;
        float dy = s->y - mf->m[i].y;
        if (dx * dx + dy * dy < CONV_R_SQ) return i;
    }
    return LIVE_BOB_WINNER_NONE;
}

/* ── §6 trail: remember where the bob has been ── */

/* A rolling record of the bob's recent positions, drawn behind it as a
 * fading tail.  The tail is what makes the motion readable: a single
 * frame is just one character, but the tail shows the looping path,
 * and with light friction those loops make pretty spirograph shapes.
 *
 * It's a ring buffer: a fixed-size array that wraps around, so it never
 * allocates memory while running and the oldest points just get
 * overwritten once it's full.
 *
 *   x[i], y[i] : a stored position, in the same units as the bob.
 *   head       : where the newest point is; wraps back to the start.
 *   count      : how many points are stored (up to the array size).
 */
typedef struct {
    float x[LIVE_TRAIL_MAX];
    float y[LIVE_TRAIL_MAX];
    int   head;
    int   count;
} Trail;

static void trail_reset(Trail *t) { t->head = 0; t->count = 0; }

static void trail_push(Trail *t, float x, float y)
{
    t->head = (t->head + 1) % LIVE_TRAIL_MAX;
    t->x[t->head] = x;
    t->y[t->head] = y;
    if (t->count < LIVE_TRAIL_MAX) t->count++;
}

/* Where the oldest point lives in the wrapped array. */
static inline int trail_oldest_index(const Trail *t)
{
    return (t->head - t->count + 1 + LIVE_TRAIL_MAX) % LIVE_TRAIL_MAX;
}

/* Picks a brightness step for a trail point from its age: newest gets
 * the brightest, oldest the faintest. */
static inline int trail_tier_for_age(int age, int count)
{
    int tier = (LIVE_TRAIL_BAND_COUNT - 1)
             - (age * LIVE_TRAIL_BAND_COUNT) / count;
    if (tier < 0)                         tier = 0;
    if (tier > LIVE_TRAIL_BAND_COUNT - 1) tier = LIVE_TRAIL_BAND_COUNT - 1;
    return tier;
}

/* ── §7 active orbit + state wrappers ── */

/* Stored as the winner when the bob never settles in time; negative so
 * it can't be mistaken for a real magnet number. */
#define BOB_TIMEOUT  (-2)

/* The one swing currently playing: the pendulum, its fading trail, and
 * a little bookkeeping.  When the bob is caught, the countdown runs,
 * and at zero a fresh drop starts the next swing, so it loops forever.
 *
 *   pendulum          : the body and its current motion.
 *   trail             : the fading tail.
 *   step_count        : steps since the drop, for the give-up check.
 *   winner            : which magnet caught it; NONE while swinging,
 *                       or TIMEOUT if it never settled.
 *   respawn_countdown : frames to wait, once caught, before the next
 *                       drop, so you can see who won.
 *   seed_x, seed_y    : where this swing was dropped from.
 */
typedef struct {
    Pendulum pendulum;
    Trail    trail;
    int      step_count;
    int      winner;
    int      respawn_countdown;
    float    seed_x, seed_y;
} ActiveOrbit;

/* A random spot anywhere in the visible area. */
static void viewport_random_phys_point(float *x, float *y)
{
    float fx = (float)rand() / (float)RAND_MAX;
    float fy = (float)rand() / (float)RAND_MAX;
    *x = X_MIN + fx * (X_MAX - X_MIN);
    *y = Y_MIN + fy * (Y_MAX - Y_MIN);
}

/* Starts a new swing from (x0, y0): bob at rest, empty trail, no
 * winner yet. */
static void active_orbit_release_at(ActiveOrbit *orb, float x0, float y0)
{
    pendulum_state_release_at(&orb->pendulum.state, x0, y0);
    trail_reset(&orb->trail);
    orb->step_count        = 0;
    orb->winner            = LIVE_BOB_WINNER_NONE;
    orb->respawn_countdown = 0;
    orb->seed_x            = x0;
    orb->seed_y            = y0;
}

/* Drops the bob from a random spot.  Used by the r key and by each
 * automatic respawn. */
static void active_orbit_release_random_seed(ActiveOrbit *orb)
{
    float x0, y0;
    viewport_random_phys_point(&x0, &y0);
    active_orbit_release_at(orb, x0, y0);
}

/* Advances the swing by one physics step: move, record the new spot,
 * then check if the bob just got caught or has run out of patience.
 * Returns false the moment it stops swinging. */
static bool active_orbit_step_once(ActiveOrbit *orb)
{
    if (orb->winner != LIVE_BOB_WINNER_NONE) return false;

    pendulum_rk4(&orb->pendulum, SIM_DT);

    trail_push(&orb->trail, orb->pendulum.state.x, orb->pendulum.state.y);
    orb->step_count++;

    int absorbed_magnet = pendulum_test_convergence(&orb->pendulum.state,
                                                    &orb->pendulum.system.magnets);
    if (absorbed_magnet != LIVE_BOB_WINNER_NONE) {
        orb->winner            = absorbed_magnet;
        orb->respawn_countdown = LIVE_BOB_RESPAWN_FRAMES;
        return false;
    }

    /* gave up: never settled in time */
    if (orb->step_count >= MAX_STEPS) {
        orb->winner            = BOB_TIMEOUT;
        orb->respawn_countdown = LIVE_BOB_RESPAWN_FRAMES;
        return false;
    }
    return true;
}

/* One frame's worth of the swing: step it a couple of times if it's
 * still moving, otherwise count down the pause and then drop a fresh
 * one.  This is what keeps the animation going forever. */
static void active_orbit_tick(ActiveOrbit *orb)
{
    if (orb->winner == LIVE_BOB_WINNER_NONE) {
        for (int i = 0; i < LIVE_BOB_SUBSTEPS_PER_TICK; i++)
            if (!active_orbit_step_once(orb)) break;
        return;
    }
    if (orb->respawn_countdown > 0) {
        orb->respawn_countdown--;
        return;
    }
    active_orbit_release_random_seed(orb);
}

/* Just holds which friction preset is selected.  It's wrapped in a
 * struct so the cycling helpers below keep the index in range and the
 * key handler reads cleanly ("cycle to next preset").
 *
 *   current : which row of presets[] is active; starts on WEAK, the
 *             chaotic one that shows the effect best.
 */
typedef struct {
    int current;
} PresetState;

static void preset_state_init(PresetState *p, int initial) { p->current = initial; }
static void preset_state_cycle_next(PresetState *p)        { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)        { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const PendPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }
static float            preset_state_active_gamma(const PresetState *p) { return presets[p->current].gamma; }

/* Which colour theme is active.  Same idea as PresetState but kept a
 * separate type on purpose: it indexes a different table (themes[]),
 * and giving it its own type stops the two from being mixed up.
 *
 *   current : which row of themes[] is active; starts on DEFAULT.
 */
typedef struct {
    int current;
} PaletteState;

static void palette_state_init(PaletteState *p, int initial) { p->current = initial; }
static void palette_state_cycle_next(PaletteState *p)        { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p)        { p->current = (p->current + N_THEMES - 1) % N_THEMES; }
static const Theme *palette_state_active(const PaletteState *p) { return &themes[p->current]; }
static void palette_state_apply(const PaletteState *p)        { theme_apply(p->current); }

/* ── §8 scene ── */

/* All the changing state of the simulation in one place, so there are
 * no loose globals.
 *
 *   orbit    : the swing currently playing.
 *   preset   : which friction setting is chosen.
 *   palette  : which colour theme is chosen.
 *   map_w/h  : size of the drawing area, in cells.
 *   paused   : when true, the physics is frozen.
 */
typedef struct {
    ActiveOrbit  orbit;
    PresetState  preset;
    PaletteState palette;
    int          map_w, map_h;
    bool         paused;
} Scene;

/* Copies the chosen preset's friction into the pendulum.  That's the
 * only thing switching presets changes; magnets and spring stay put. */
static void scene_load_active_preset(Scene *s)
{
    s->orbit.pendulum.system.damping = preset_state_active_gamma(&s->preset);
}

static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

static void scene_reset(Scene *s, int w, int h)
{
    s->map_w = w;
    s->map_h = h;
    scene_load_active_preset(s);
    active_orbit_release_random_seed(&s->orbit);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->paused = false;

    preset_state_init (&s->preset,  PRESET_WEAK_DAMP);
    palette_state_init(&s->palette, 0);

    /* Magnets and spring are set up once; friction comes from the
     * preset, filled in by scene_reset just below. */
    s->orbit.pendulum.system = pendulum_system_init_default();
    scene_reset(s, w, h);
}

/* One step of the world (unless paused): advance the swing, which also
 * handles dropping a new bob once the old one is caught. */
static void scene_tick(Scene *s, float dt)
{
    (void)dt;
    if (s->paused) return;
    active_orbit_tick(&s->orbit);
}

/* ── §9 screen ── */

/* The current terminal size, kept here so painters don't each have to
 * ask for it.  Updated on startup and whenever the window is resized.
 *
 *   cols : width in characters.
 *   rows : height in characters; row 0 is the top.
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

/* The on-screen rectangle the pendulum is drawn into, centred in the
 * window.  Painters share it so the bob's position-to-cell maths
 * happens the same way everywhere.
 *
 *   gx0, gy0 : the rectangle's top-left corner, in screen cells.
 *   w, h     : its size, in cells.
 */
typedef struct {
    int gx0, gy0;
    int w, h;
} Viewport;

static Viewport viewport_build(int map_w, int map_h, int cols, int rows)
{
    Viewport vp;
    vp.w   = map_w;
    vp.h   = map_h;
    vp.gx0 = (cols - vp.w) / 2;
    vp.gy0 = ((rows - HUD_BAND_RESERVED_ROWS) - vp.h) / 2 + HUD_TOP_ROWS;
    if (vp.gx0 < 0)            vp.gx0 = 0;
    if (vp.gy0 < HUD_TOP_ROWS) vp.gy0 = HUD_TOP_ROWS;
    return vp;
}

/* Added before rounding a float down to an int so it rounds to nearest
 * instead, otherwise everything drifts a cell low. */
#define VIEWPORT_ROUND_OFFSET   0.5f

/* Turns a position in the world into a screen cell.  Returns false if
 * the point is off the visible area.  Y is flipped because up in the
 * world is up the screen, but rows count downward. */
static bool viewport_phys_to_cell(const Viewport *vp, float x, float y,
                                  int *sx, int *sy)
{
    if (x < X_MIN || x > X_MAX || y < Y_MIN || y > Y_MAX) return false;

    float fx_norm = (x - X_MIN) / (X_MAX - X_MIN);
    float fy_norm = (y - Y_MIN) / (Y_MAX - Y_MIN);

    int cell_x = (int)(fx_norm * (float)(vp->w - 1) + VIEWPORT_ROUND_OFFSET);
    int cell_y = (int)(fy_norm * (float)(vp->h - 1) + VIEWPORT_ROUND_OFFSET);

    if (cell_x < 0)         cell_x = 0;
    if (cell_x >= vp->w)    cell_x = vp->w - 1;
    if (cell_y < 0)         cell_y = 0;
    if (cell_y >= vp->h)    cell_y = vp->h - 1;

    *sx = vp->gx0 + cell_x;
    *sy = vp->gy0 + (vp->h - 1 - cell_y);
    return true;
}

/* Draws each magnet as [1] [2] [3] at its spot.  The number says which
 * magnet it is; all three are the same colour. */
static void paint_magnet_markers(const MagnetField *mf, const Viewport *vp)
{
    attron(COLOR_PAIR(PAIR_TIER_BRIGHTEST) | A_BOLD);
    for (int i = 0; i < mf->count; i++) {
        int sx, sy;
        if (!viewport_phys_to_cell(vp, mf->m[i].x, mf->m[i].y, &sx, &sy))
            continue;
        if (sx > vp->gx0 && sx < vp->gx0 + vp->w - 1) {
            mvaddch(sy, sx - 1, (chtype)(unsigned char)MAGNET_GLYPH_LEFT);
            mvaddch(sy, sx + 1, (chtype)(unsigned char)MAGNET_GLYPH_RIGHT);
        }
        mvaddch(sy, sx, (chtype)(unsigned char)MAGNET_GLYPH_CENTRE_FOR(i));
    }
    attroff(COLOR_PAIR(PAIR_TIER_BRIGHTEST) | A_BOLD);
}

/* The trail character for a given brightness step, faint to heavy. */
static char trail_tier_glyph(int tier)
{
    switch (tier) {
    case 0:  return LIVE_TRAIL_GLYPH_TIER0;
    case 1:  return LIVE_TRAIL_GLYPH_TIER1;
    case 2:  return LIVE_TRAIL_GLYPH_TIER2;
    default: return LIVE_TRAIL_GLYPH_TIER3;
    }
}

/* Draws the whole tail, each point fading according to its age. */
static void paint_trail(const Trail *t, const Viewport *vp)
{
    if (t->count == 0) return;
    int oldest = trail_oldest_index(t);
    for (int i = 0; i < t->count; i++) {
        int idx  = (oldest + i) % LIVE_TRAIL_MAX;
        int age  = t->count - 1 - i;
        int tier = trail_tier_for_age(age, t->count);
        int sx, sy;
        if (!viewport_phys_to_cell(vp, t->x[idx], t->y[idx], &sx, &sy))
            continue;
        int pair     = PAIR_TIER(tier);
        chtype glyph = (chtype)(unsigned char)trail_tier_glyph(tier);
        attron(COLOR_PAIR(pair) | A_BOLD);
        mvaddch(sy, sx, glyph);
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

/* Draws the bob: @ while swinging, O once stuck to a magnet.  Always
 * the brightest thing on screen. */
static void paint_bob_marker(const ActiveOrbit *orb, const Viewport *vp)
{
    int sx, sy;
    if (!viewport_phys_to_cell(vp, orb->pendulum.state.x,
                                   orb->pendulum.state.y, &sx, &sy)) return;
    chtype glyph = (orb->winner == LIVE_BOB_WINNER_NONE)
                 ? (chtype)(unsigned char)LIVE_BOB_GLYPH_MOVING
                 : (chtype)(unsigned char)LIVE_BOB_GLYPH_SETTLED;
    attron(COLOR_PAIR(PAIR_TIER_BRIGHTEST) | A_BOLD);
    mvaddch(sy, sx, glyph);
    attroff(COLOR_PAIR(PAIR_TIER_BRIGHTEST) | A_BOLD);
}

/* The cell the rod hangs from: top-centre of the viewport.  This is
 * just a visual anchor, not the spring centre in the physics. */
static void viewport_virtual_pivot_cell(const Viewport *vp,
                                        int *pivot_x, int *pivot_y)
{
    *pivot_x = vp->gx0 + vp->w / 2;
    *pivot_y = vp->gy0 + ANCHOR_TOP_OFFSET;
}

/* Draws the + the pendulum hangs from. */
static void paint_anchor_marker(const Viewport *vp)
{
    int pivot_x, pivot_y;
    viewport_virtual_pivot_cell(vp, &pivot_x, &pivot_y);
    attron(COLOR_PAIR(PAIR_TIER_BRIGHTEST) | A_BOLD);
    mvaddch(pivot_y, pivot_x, (chtype)(unsigned char)ANCHOR_GLYPH);
    attroff(COLOR_PAIR(PAIR_TIER_BRIGHTEST) | A_BOLD);
}

/* Picks the rod character that matches the direction of this step, so
 * the rod looks like a real line at any angle rather than dots. */
static chtype rod_glyph_for_step(bool step_x, bool step_y, int sx_dir, int sy_dir)
{
    if (step_x && step_y) return (sx_dir == sy_dir) ? ROD_GLYPH_DIAG_DOWNRIGHT
                                                    : ROD_GLYPH_DIAG_UPRIGHT;
    if (step_x)           return ROD_GLYPH_HORIZ;
    return ROD_GLYPH_VERT;
}

/* Skip the rod character at the pivot and bob cells, since those get
 * their own symbols drawn over them anyway. */
static inline bool rod_cell_should_skip(int x, int y,
                                        int pivot_x, int pivot_y,
                                        int bob_x,   int bob_y)
{
    return (x == pivot_x && y == pivot_y)
        || (x == bob_x   && y == bob_y);
}

/* Draws the rod as a straight line of characters from the pivot down
 * to the bob (skipped if the bob is off-screen).  It uses Bresenham's
 * line algorithm (1965), the standard integer-only way to draw a
 * straight line cell by cell. */
static void paint_pendulum_rod(const ActiveOrbit *orb, const Viewport *vp)
{
    int pivot_x, pivot_y;
    viewport_virtual_pivot_cell(vp, &pivot_x, &pivot_y);
    int bob_x, bob_y;
    if (!viewport_phys_to_cell(vp, orb->pendulum.state.x,
                                   orb->pendulum.state.y, &bob_x, &bob_y))
        return;

    int cursor_x      = pivot_x;
    int cursor_y      = pivot_y;
    int delta_x       = abs(bob_x - cursor_x);
    int delta_y       = abs(bob_y - cursor_y);
    int step_dir_x    = (cursor_x < bob_x) ?  1 : -1;
    int step_dir_y    = (cursor_y < bob_y) ?  1 : -1;
    int error_accum   = delta_x - delta_y;

    attron(COLOR_PAIR(PAIR_TIER_DIM));
    while (1) {
        if (!rod_cell_should_skip(cursor_x, cursor_y,
                                  pivot_x, pivot_y, bob_x, bob_y)) {
            int doubled_error  = 2 * error_accum;
            bool will_step_x   = (doubled_error > -delta_y);
            bool will_step_y   = (doubled_error <  delta_x);
            chtype glyph = rod_glyph_for_step(will_step_x, will_step_y,
                                              step_dir_x, step_dir_y);
            mvaddch(cursor_y, cursor_x, glyph);
        }
        if (cursor_x == bob_x && cursor_y == bob_y) break;

        /* step toward the bob along whichever axis is due next */
        int doubled_error = 2 * error_accum;
        if (doubled_error > -delta_y) { error_accum -= delta_y; cursor_x += step_dir_x; }
        if (doubled_error <  delta_x) { error_accum += delta_x; cursor_y += step_dir_y; }
    }
    attroff(COLOR_PAIR(PAIR_TIER_DIM));
}

/* Draws the whole pendulum back to front: pivot, then the faded tail,
 * then the rod on top, then the bob at the tip. */
static void paint_pendulum_apparatus(const ActiveOrbit *orb, const Viewport *vp)
{
    paint_anchor_marker(vp);
    paint_trail        (&orb->trail, vp);
    paint_pendulum_rod (orb, vp);
    paint_bob_marker   (orb, vp);
}

/* Draws the pendulum, then the magnets on top so the bob visibly lands
 * on one when it's caught. */
static void scene_paint(const Scene *s, int cols, int rows)
{
    Viewport vp = viewport_build(s->map_w, s->map_h, cols, rows);
    paint_pendulum_apparatus(&s->orbit, &vp);
    paint_magnet_markers    (&s->orbit.pendulum.system.magnets, &vp);
}

/* Short line about the bob: still swinging, timed out, or which magnet
 * caught it.  Kept under 32 chars to fit the status slot. */
static void hud_pendulum_status(char *buf, size_t bufsz, const Scene *s)
{
    const ActiveOrbit *orb = &s->orbit;
    if (orb->winner == LIVE_BOB_WINNER_NONE)
        snprintf(buf, bufsz, "swinging %4d/%d", orb->step_count, MAX_STEPS);
    else if (orb->winner == BOB_TIMEOUT)
        snprintf(buf, bufsz, "TIMEOUT @ %d", orb->step_count);
    else
        snprintf(buf, bufsz, "absorbed→magnet %d", orb->winner + 1);
}

static void hud_paint_top_left_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " MAGNETIC PENDULUM ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Builds the right-hand status text (fps, rate, preset, bob state).
 * Just fills a buffer; doesn't draw anything. */
static void hud_format_top_right_status(char *buf, size_t bufsz,
                                        double fps, int sim_fps,
                                        const Scene *s)
{
    char pend_status[32];
    hud_pendulum_status(pend_status, sizeof pend_status, s);
    const PendPreset *active_preset = preset_state_active(&s->preset);
    snprintf(buf, bufsz,
             " %5.1f fps  %3d Hz  %s [%d/%d]  %s ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : active_preset->name,
             s->preset.current + 1, N_PRESETS, pend_status);
}

/* Draws text flush against the right edge of row 0, clamped so it
 * never starts off the left edge on a narrow terminal. */
static void hud_paint_top_right(int cols, const char *text)
{
    int anchor_col = cols - (int)strlen(text);
    if (anchor_col < 0) anchor_col = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, anchor_col, "%s", text);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_top(int cols, double fps, int sim_fps, const Scene *s)
{
    hud_paint_top_left_title();

    char buf[HUD_COLS + 1];
    hud_format_top_right_status(buf, sizeof buf, fps, sim_fps, s);
    hud_paint_top_right(cols, buf);
}
/* Printed widths of the row-1 cells, fixed so the layout doesn't jump
 * as the preset or theme name changes. */
#define HUD_PARAM_CELL_WIDTH_PRESET   19
#define HUD_PARAM_CELL_WIDTH_THEME    17
#define HUD_PARAM_CELL_WIDTH_GAMMA    13

/* Row 1: the current preset, theme, the friction and spring numbers,
 * and a little legend of what the symbols mean. */
static void hud_param(const Scene *s)
{
    int cursor_x = HUD_LEFT_MARGIN;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, cursor_x, " preset:%-8s ", preset_state_active(&s->preset)->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    cursor_x += HUD_PARAM_CELL_WIDTH_PRESET;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, cursor_x, " theme:%-8s ", palette_state_active(&s->palette)->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    cursor_x += HUD_PARAM_CELL_WIDTH_THEME;

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cursor_x, " γ:%.2f  k:%.2f ",
             (double)s->orbit.pendulum.system.damping,
             (double)s->orbit.pendulum.system.spring_k);
    attroff(COLOR_PAIR(PAIR_HUD));
    cursor_x += HUD_PARAM_CELL_WIDTH_GAMMA;

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cursor_x, " [N]=magnet  @=bob  +=pivot ");
    attroff(COLOR_PAIR(PAIR_HUD));
}
/* Bottom row: just the list of keys. */
static void hud_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  r:reseed  n/p:preset  t/T:theme  ]/[:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_paint(s, sc->cols, sc->rows);
    hud_top(sc->cols, fps, sim_fps, s);
    hud_param(s);
    hud_hint(sc->rows);
}

/* ── §10 app ── */

/* The whole program's state in one place.  main() just drives this and
 * each helper does one named thing to it; there are no other globals.
 *
 *   scene       : the simulation itself.
 *   screen      : the terminal size + ncurses handle.
 *   sim_fps     : how fast the physics ticks; the drawing always aims
 *                 for 60 fps regardless.  Changed with the ] [ keys.
 *   map_w/h     : size of the drawing area, recomputed on resize.
 *   running     : main loop runs while this is set; cleared on quit.
 *   need_resize : set when the window changes size, handled next frame.
 *
 * running and need_resize are touched by signal handlers, so they're
 * volatile sig_atomic_t (the only type it's safe to do that with).
 * The single global instance below exists because signal handlers
 * need something they can reach without arguments. */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   map_w, map_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* Sizes the drawing area to the terminal minus the HUD rows, clamped
 * to sane minimums and a cap. */
static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS;
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

/* Rebuilds the screen and scene after the window was resized. */
static void app_handle_pending_resize(App *app)
{
    if (!app->need_resize) return;
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* How long since the last frame, capped so a long stall (e.g. the
 * laptop sleeping) doesn't make the physics try to catch up forever. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > SIM_MAX_FRAME_DT_MS * NS_PER_MS)
        dt = SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    return dt;
}

/* Runs the physics in fixed-size steps, taking as many as the elapsed
 * time allows.  Stepping by a fixed amount (rather than by the real
 * frame time) means the same drop lands on the same magnet on a fast
 * or slow machine.  See Fiedler, "Fix Your Timestep!". */
static void app_drain_fixed_timestep(App *app, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* Recomputes the displayed fps a couple of times a second, so the
 * number is steady enough to read. */
static void app_update_fps_meter(int64_t *fps_accum, int *frame_count,
                                 double *fps_display)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_display = (double)*frame_count
                 / ((double)*fps_accum / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
}

/* Sleeps off the rest of the frame so drawing holds at about 60 fps. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* Draws everything and shows it. */
static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* Speed the physics up / slow it down, within limits. */
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

static void app_toggle_pause      (App *app) { app->scene.paused = !app->scene.paused; }
static void app_reseed_orbit      (App *app) { active_orbit_release_random_seed(&app->scene.orbit); }

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

/* Reads a key if one's waiting and acts on it; returns false to quit. */
static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* Maps each key to one action. */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            app_toggle_pause     (app); break;
    case 'r': case 'R':  app_reseed_orbit     (app); break;
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

/* The timing bookkeeping for the main loop, kept together instead of
 * as loose locals.
 *
 *   frame_time  : when the last frame happened, to measure elapsed time.
 *   sim_accum   : time owed to the physics but not yet stepped off.
 *   fps_accum   : time piled up since the last fps refresh.
 *   frame_count : frames since the last fps refresh.
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

/* Set up, then each frame: handle any resize, step the physics by the
 * elapsed time, draw, wait out the frame, and check the keyboard. */
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
