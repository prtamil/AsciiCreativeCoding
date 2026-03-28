/*
 * flocking.c — ncurses ASCII flocking simulation
 *
 * Three flock groups, each with a wandering leader and followers that steer
 * using one of two switchable algorithms. Boids wrap around screen edges
 * (toroidal topology) instead of bouncing. Colors cycle via a cosine palette
 * so all three flocks stay visually distinct while slowly shifting hue.
 *
 * Physics run in square pixel space (CELL_W × CELL_H sub-pixels per cell)
 * so that motion is isotropic — same technique as bounce.c.
 *
 * Sections
 * --------
 *   §1  config   — every tunable constant with reasoning
 *   §2  clock    — monotonic nanosecond clock + sleep
 *   §3  color    — cosine palette → xterm-256 cube → ncurses pairs
 *   §4  coords   — pixel ↔ cell conversion (the one aspect-ratio fix)
 *   §5  boid     — struct; velocity-direction char; spawn; speed clamp; wrap
 *   §6  flock    — leader wander; boids rules; two algorithm modes; flock init
 *   §7  scene    — pool of flocks; tick; draw with brightness gradient
 *   §8  screen   — ncurses init / draw (color update + HUD) / present
 *   §9  app      — dt loop, input, resize, signal handling
 *
 * Keys
 *   q / ESC   quit
 *   space     pause / resume
 *   1         mode: Classic Boids  (separation + alignment + cohesion)
 *   2         mode: Leader Chase   (followers home directly on leader)
 *   + / -     add / remove one follower per flock
 *   r         reset all flocks to their starting quadrants
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra flocking.c -o flocking -lncurses -lm
 */

/*
 * _GNU_SOURCE exposes M_PI from <math.h>.
 * _POSIX_C_SOURCE alone does not include M_PI (it is a POSIX extension).
 */
#define _GNU_SOURCE

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

/*
 * Terminal cell geometry.
 * Physics lives in pixel space where 1 unit ≈ 1 physical pixel.
 * Terminal cells are roughly twice as tall as wide (CELL_H / CELL_W = 2),
 * so we give each cell 8 horizontal and 16 vertical sub-pixels.
 * All physics uses pixel units; only the final draw step converts to cells.
 * This makes diagonal motion and speed look correct on screen.
 */
#define CELL_W   8    /* sub-pixel columns per terminal cell */
#define CELL_H  16    /* sub-pixel rows    per terminal cell */

/*
 * Simulation and render rates.
 * Fixed-timestep accumulator decouples physics accuracy from wall-clock FPS.
 */
enum {
    SIM_FPS        = 60,    /* physics ticks per second               */
    RENDER_FPS     = 60,    /* display frame cap                      */
    FPS_UPDATE_MS  = 500,   /* HUD fps counter refresh interval (ms)  */
};

/*
 * Flock counts.
 * Three flocks → three cosine hue phases spaced 120° apart (0, 1/3, 2/3),
 * keeping flocks visually distinct as the palette slowly cycles.
 * 12 followers per flock gives ~39 total boids — enough for visible
 * flocking without O(n²) neighbor search becoming noticeable.
 */
enum {
    FLOCKS            =  3,
    FOLLOWERS_DEFAULT = 12,
    FOLLOWERS_MIN     =  3,
    FOLLOWERS_MAX     = 20,
};

/*
 * Boid speeds in pixels per second.
 *
 * Smoothness threshold (from bounce.c):
 *   A boid must cross at least one cell boundary every ~4 ticks to avoid
 *   staircase artifacts. For vertical cells (taller dimension):
 *     speed_min ≥ CELL_H × SIM_FPS / 4 = 16 × 60 / 4 = 240 px/s
 * We set BOID_SPEED above that floor with a comfortable margin.
 * LEADER_SPEED is slightly higher so the leader naturally pulls ahead.
 */
#define BOID_SPEED    280.0f    /* follower cruising speed, px/s */
#define LEADER_SPEED  340.0f    /* leader cruising speed, px/s   */
#define MIN_SPEED      80.0f    /* floor — boids never fully stop */
#define MAX_SPEED     500.0f    /* ceiling — keeps flocks legible */

/*
 * Boid perception radii in pixel space.
 * PERCEPTION_RADIUS: how far a boid "sees" neighbors for all three boids rules.
 * SEPARATION_RADIUS: closer range where boids push apart.
 *   Set to ~1/3 of PERCEPTION_RADIUS so boids have a "personal space" zone
 *   while still responding to the wider group outside it.
 */
#define PERCEPTION_RADIUS  180.0f   /* px — neighbor sensing range   */
#define SEPARATION_RADIUS   60.0f   /* px — minimum comfortable gap  */

/*
 * Boids steering weights — scale each rule's contribution to the force vector.
 *
 * Separation is strongest to prevent crowding.
 * Alignment is moderate so boids match direction without locking in formation.
 * Cohesion is weakest so groups stay loose rather than collapsing to a point.
 * Leader pull is gentle — it keeps the group near its leader without
 * overriding the emergent boids behavior among followers.
 */
#define W_SEPARATION   1.8f    /* repulsion from nearby boids       */
#define W_ALIGNMENT    1.0f    /* match average heading of group    */
#define W_COHESION     0.5f    /* drift toward group center of mass */
#define W_LEADER_PULL  0.4f    /* gentle attraction toward leader   */

/*
 * Maximum steering force applied per tick (px/s added to velocity).
 * Clamping the turn rate makes boids curve smoothly rather than snap.
 */
#define MAX_STEER  130.0f

/*
 * Leader wander — smooth random walk on the heading angle.
 * Each tick the leader's heading shifts by up to ±WANDER_JITTER radians,
 * producing gently curved paths instead of sharp random turns.
 */
#define WANDER_JITTER  0.10f    /* max heading change per tick (radians) */

/*
 * Cosine palette animation speed.
 * color_t advances by COLOR_SPEED each frame. One full hue cycle takes
 * 1 / COLOR_SPEED frames. At 60 fps: 1 / 0.003 = 333 frames ≈ 5.5 seconds.
 */
#define COLOR_SPEED  0.003f

/* Common time constants */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(fps)  (NS_PER_SEC / (fps))

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
/* §3  color — standard 16-color terminal pairs                          */
/* ===================================================================== */

/*
 * We use only the 16 standard ANSI colors — no 256-color or truecolor.
 * This works on every terminal: xterm, rxvt, tmux, Linux console, SSH.
 *
 * Each flock has TWO color pairs:
 *   follower pair — the flock's base color (cyan / yellow / magenta)
 *   leader pair   — a bright contrasting color unique to that leader
 *
 * Leader colors are chosen to contrast clearly against their flock's color:
 *   flock 0 cyan     → leader white   (neutral, pops against cyan)
 *   flock 1 yellow   → leader red     (warm contrast against yellow)
 *   flock 2 magenta  → leader green   (complementary to magenta)
 *
 * A_BOLD on top of any of these makes the leader noticeably brighter
 * than its followers, which use A_BOLD only when close to the leader.
 *
 * Color pair layout:
 *   pairs 1 … FLOCKS          — follower colors (one per flock)
 *   pairs FLOCKS+1 … 2*FLOCKS — leader colors   (one per flock)
 *   pair  2*FLOCKS + 1        — HUD (white on black)
 */
static const int k_follower_colors[FLOCKS] = {
    COLOR_CYAN,      /* flock 0 followers — cyan    */
    COLOR_YELLOW,    /* flock 1 followers — yellow  */
    COLOR_MAGENTA,   /* flock 2 followers — magenta */
};

static const int k_leader_colors[FLOCKS] = {
    COLOR_WHITE,     /* flock 0 leader — white  (contrasts cyan)    */
    COLOR_RED,       /* flock 1 leader — red    (contrasts yellow)  */
    COLOR_GREEN,     /* flock 2 leader — green  (contrasts magenta) */
};

static int follower_color_pair(int flock_idx) { return flock_idx + 1;          }
static int leader_color_pair  (int flock_idx) { return flock_idx + 1 + FLOCKS; }
#define HUD_COLOR_PAIR  (2 * FLOCKS + 1)

static void color_init(void)
{
    start_color();
    for (int i = 0; i < FLOCKS; i++) {
        init_pair(follower_color_pair(i), k_follower_colors[i], COLOR_BLACK);
        init_pair(leader_color_pair(i),   k_leader_colors[i],   COLOR_BLACK);
    }
    init_pair(HUD_COLOR_PAIR, COLOR_WHITE, COLOR_BLACK);
}

/* ===================================================================== */
/* §4  coords — the one place aspect ratio is handled                    */
/* ===================================================================== */

/*
 * pw / ph — pixel space dimensions given terminal cell dimensions.
 * Everything in physics uses pixel units; only scene_draw converts back.
 */
static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

/*
 * px_to_cell_x/y — convert pixel coordinate to terminal cell.
 *
 * Uses floorf(px / CELL + 0.5f) — "round half up" — instead of roundf().
 * roundf uses "round half to even" (banker's rounding): a boid sitting
 * exactly on a cell boundary oscillates between two cells every frame,
 * producing a flickering doubled character. Adding 0.5 before floor breaks
 * the tie in one consistent direction, eliminating the flicker.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  boid                                                               */
/* ===================================================================== */

/* All positions and velocities are in PIXEL SPACE. */
typedef struct {
    float px, py;    /* position (pixels)          */
    float vx, vy;    /* velocity (pixels / second) */
} Boid;

/*
 * velocity_dir_char — pick an ASCII glyph that visually represents the
 * direction of travel, so flocks show coherent heading in the terminal.
 *
 * We quantize the continuous velocity angle into 8 compass sectors (45° each)
 * and return the matching character:
 *
 *   Cardinal:   >  v  <  ^
 *   Diagonal:   \  /       (each char represents the diagonal axis, not direction)
 *
 * Sector layout (clockwise from East, matching +y = down in pixel space):
 *   index: 0   1    2   3    4   5    6   7
 *   dir:   W   NW   N   NE   E   SE   S   SW
 *   char:  <   \    ^   /    >   \    v   /
 *
 * Formula:
 *   atan2(vy, vx) gives angle in [-π, π].
 *   Shift to [0, 2π] by adding π, then shift by half a sector (π/8) to
 *   center the sectors on the cardinal directions rather than edging them.
 *   Divide by sector width (π/4), floor, mod 8 → sector index 0–7.
 */
static char velocity_dir_char(float vx, float vy)
{
    static const char k_chars[8] = {'<', '\\', '^', '/', '>', '\\', 'v', '/'};

    float angle   = atan2f(vy, vx);                   /* [-π, π]            */
    float shifted = angle + (float)M_PI                /* → [0, 2π]          */
                          + (float)M_PI / 8.0f;       /* center on cardinals */
    int   sector  = (int)floorf(shifted / ((float)M_PI / 4.0f)) % 8;
    return k_chars[sector];
}

/*
 * boid_spawn_at — place a boid at (px, py) with a random direction at
 * the given speed. Uses rejection-sample unit vector to get a uniform
 * random angle distribution without calling atan2/sinf/cosf at spawn.
 */
static void boid_spawn_at(Boid *b, float px, float py, float speed)
{
    b->px = px;
    b->py = py;

    float dx, dy, len;
    do {
        dx  = (float)(rand() % 2001 - 1000) / 1000.0f;   /* uniform in [-1, 1] */
        dy  = (float)(rand() % 2001 - 1000) / 1000.0f;
        len = dx*dx + dy*dy;
    } while (len < 0.01f || len > 1.0f);                  /* keep if inside unit circle */

    float mag = sqrtf(len);
    b->vx = (dx / mag) * speed;
    b->vy = (dy / mag) * speed;
}

/*
 * boid_clamp_speed — enforce MIN_SPEED ≤ |velocity| ≤ MAX_SPEED.
 * Without a floor, boids stall when opposing forces cancel out.
 * Without a ceiling, large separation forces cause runaway acceleration.
 */
static void boid_clamp_speed(Boid *b)
{
    float mag = hypotf(b->vx, b->vy);
    if (mag < 0.001f) {
        b->vx = MIN_SPEED;    /* stationary boid: nudge forward */
        b->vy = 0.0f;
        return;
    }
    if (mag < MIN_SPEED) {
        b->vx = (b->vx / mag) * MIN_SPEED;
        b->vy = (b->vy / mag) * MIN_SPEED;
    } else if (mag > MAX_SPEED) {
        b->vx = (b->vx / mag) * MAX_SPEED;
        b->vy = (b->vy / mag) * MAX_SPEED;
    }
}

/*
 * boid_wrap — toroidal boundary: a boid leaving one edge re-enters at
 * the opposite edge. This means no boid is ever "cornered" with no
 * neighbors on one side, keeping flocking behavior uniform everywhere.
 */
static void boid_wrap(Boid *b, float max_px, float max_py)
{
    if (b->px <  0.0f)    b->px += max_px;
    if (b->px >= max_px)  b->px -= max_px;
    if (b->py <  0.0f)    b->py += max_py;
    if (b->py >= max_py)  b->py -= max_py;
}

/* ===================================================================== */
/* §6  flock                                                              */
/* ===================================================================== */

/* Two switchable steering algorithms */
typedef enum {
    MODE_BOIDS = 0,    /* Reynolds 1987: separation + alignment + cohesion */
    MODE_CHASE = 1,    /* followers ignore peers and home on their leader   */
    MODE_COUNT
} FlockMode;

static const char *k_mode_names[MODE_COUNT] = {
    "BOIDS  sep+align+cohesion",
    "CHASE  leader homing",
};

typedef struct {
    Boid  leader;
    Boid  followers[FOLLOWERS_MAX];
    int   n;              /* active follower count             */
    float color_phase;    /* hue offset in cosine palette      */
    float wander_angle;   /* leader heading in radians         */
} Flock;

/* ── leader ──────────────────────────────────────────────────────────── */

/*
 * leader_tick — advance the leader one physics step.
 *
 * The leader uses a smooth random walk on its heading angle (wander_angle).
 * Each tick the heading shifts by a small random amount in
 * [-WANDER_JITTER, +WANDER_JITTER] radians — just enough to curve gently
 * without making sharp unpredictable turns that followers cannot track.
 * Speed is constant; we derive velocity from the angle rather than
 * integrating forces, so the leader never slows or stalls.
 */
static void leader_tick(Flock *f, float dt, float max_px, float max_py)
{
    float jitter = ((float)(rand() % 2001) / 1000.0f - 1.0f) * WANDER_JITTER;
    f->wander_angle += jitter;

    f->leader.vx = LEADER_SPEED * cosf(f->wander_angle);
    f->leader.vy = LEADER_SPEED * sinf(f->wander_angle);

    f->leader.px += f->leader.vx * dt;
    f->leader.py += f->leader.vy * dt;
    boid_wrap(&f->leader, max_px, max_py);
}

/* ── steering helper ─────────────────────────────────────────────────── */

/*
 * clamp2d — clamp a 2D vector (fx, fy) so its magnitude does not exceed
 * max_mag. Preserves direction; only scales down if over the limit.
 * Avoids a sqrt when the vector is already within bounds.
 */
static void clamp2d(float *fx, float *fy, float max_mag)
{
    float mag_sq = (*fx) * (*fx) + (*fy) * (*fy);
    if (mag_sq > max_mag * max_mag) {
        float mag = sqrtf(mag_sq);
        *fx = (*fx / mag) * max_mag;
        *fy = (*fy / mag) * max_mag;
    }
}

/* ── boids steering (MODE_BOIDS) ─────────────────────────────────────── */

/*
 * boids_steer — compute the new velocity for followers[idx] using the
 * three Reynolds boids rules plus a gentle leader attraction.
 *
 * Three rules, each producing a steering increment (px/s added to velocity):
 *
 *   Separation: sum of unit-push vectors away from boids within
 *               SEPARATION_RADIUS, weighted by closeness.
 *               Prevents boids from occupying the same cell.
 *
 *   Alignment:  steer toward the average velocity direction of all boids
 *               within PERCEPTION_RADIUS. Produces coherent heading.
 *
 *   Cohesion:   steer toward the center of mass of visible boids.
 *               Keeps the group loosely together.
 *
 *   Leader pull: a constant gentle attraction toward the flock's own
 *               leader. Stops the group from drifting away indefinitely
 *               without overriding the emergent inter-boid behavior.
 *
 * The total steering force is clamped to MAX_STEER so boids curve smoothly.
 *
 * NOTE: reads only — this function never writes to f->followers[].
 * flock_tick pre-computes all new velocities before applying any, so every
 * boid reacts to the same snapshot of positions ("simultaneous update").
 */
static void boids_steer(const Flock *f, int idx,
                         float *out_vx, float *out_vy)
{
    const Boid *b = &f->followers[idx];

    /* Accumulators for each rule */
    float sep_x = 0.0f, sep_y = 0.0f;    /* push vectors away from close boids */
    float ali_vx = 0.0f, ali_vy = 0.0f;  /* sum of neighbor velocities         */
    float coh_px = 0.0f, coh_py = 0.0f;  /* sum of neighbor positions          */
    int   neighbor_n = 0;
    int   sep_n      = 0;

    for (int j = 0; j < f->n; j++) {
        if (j == idx) continue;

        const Boid *nb   = &f->followers[j];
        float       dx   = nb->px - b->px;
        float       dy   = nb->py - b->py;
        float       dist = hypotf(dx, dy);

        if (dist >= PERCEPTION_RADIUS || dist < 0.001f) continue;

        /* Alignment and cohesion use all visible neighbors */
        ali_vx += nb->vx;  ali_vy += nb->vy;
        coh_px += nb->px;  coh_py += nb->py;
        neighbor_n++;

        /* Separation uses only neighbors inside the personal-space radius */
        if (dist < SEPARATION_RADIUS) {
            /* Repulsion grows as boids get closer (linear: 1 at contact, 0 at edge) */
            float closeness = (SEPARATION_RADIUS - dist) / SEPARATION_RADIUS;
            sep_x -= (dx / dist) * closeness;   /* push away: negate the toward vector */
            sep_y -= (dy / dist) * closeness;
            sep_n++;
        }
    }

    /* Accumulate total steering force */
    float steer_x = 0.0f, steer_y = 0.0f;

    /* Separation: direct velocity nudge scaled by weight */
    if (sep_n > 0) {
        /* Normalize so number of close neighbors doesn't amplify the force */
        float sm = hypotf(sep_x, sep_y);
        if (sm > 0.001f) {
            steer_x += (sep_x / sm) * W_SEPARATION * MAX_STEER;
            steer_y += (sep_y / sm) * W_SEPARATION * MAX_STEER;
        }
    }

    /* Alignment: steer toward average neighbor direction */
    if (neighbor_n > 0) {
        float avg_vx = ali_vx / (float)neighbor_n;
        float avg_vy = ali_vy / (float)neighbor_n;
        /* Desired velocity: average direction at cruising speed */
        float am = hypotf(avg_vx, avg_vy);
        if (am > 0.001f) {
            float desired_vx = (avg_vx / am) * BOID_SPEED;
            float desired_vy = (avg_vy / am) * BOID_SPEED;
            /* Steering increment = desired − current (how much to turn) */
            steer_x += (desired_vx - b->vx) * W_ALIGNMENT;
            steer_y += (desired_vy - b->vy) * W_ALIGNMENT;
        }
    }

    /* Cohesion: steer toward center of mass */
    if (neighbor_n > 0) {
        float center_x = coh_px / (float)neighbor_n;
        float center_y = coh_py / (float)neighbor_n;
        float toward_x = center_x - b->px;
        float toward_y = center_y - b->py;
        float cm = hypotf(toward_x, toward_y);
        if (cm > 0.001f) {
            float desired_vx = (toward_x / cm) * BOID_SPEED;
            float desired_vy = (toward_y / cm) * BOID_SPEED;
            steer_x += (desired_vx - b->vx) * W_COHESION;
            steer_y += (desired_vy - b->vy) * W_COHESION;
        }
    }

    /* Leader pull: attract toward own leader at gentle constant strength */
    {
        float lx = f->leader.px - b->px;
        float ly = f->leader.py - b->py;
        float lm = hypotf(lx, ly);
        if (lm > 0.001f) {
            steer_x += (lx / lm) * W_LEADER_PULL * MAX_STEER;
            steer_y += (ly / lm) * W_LEADER_PULL * MAX_STEER;
        }
    }

    /* Clamp total force so boids curve rather than snap */
    clamp2d(&steer_x, &steer_y, MAX_STEER);

    *out_vx = b->vx + steer_x;
    *out_vy = b->vy + steer_y;
}

/* ── chase steering (MODE_CHASE) ─────────────────────────────────────── */

/*
 * chase_steer — simple direct homing on the leader.
 * Each follower ignores its peers and steers straight toward its leader.
 * Only separation at close range is applied, so boids don't stack on top
 * of each other while chasing. Produces "comet tail" formation shapes.
 */
static void chase_steer(const Flock *f, int idx,
                          float *out_vx, float *out_vy)
{
    const Boid *b  = &f->followers[idx];
    float steer_x  = 0.0f, steer_y = 0.0f;

    /* Steer toward leader */
    float lx = f->leader.px - b->px;
    float ly = f->leader.py - b->py;
    float lm = hypotf(lx, ly);
    if (lm > 0.001f) {
        float desired_vx = (lx / lm) * BOID_SPEED;
        float desired_vy = (ly / lm) * BOID_SPEED;
        steer_x = desired_vx - b->vx;
        steer_y = desired_vy - b->vy;
    }

    /* Separation: prevent boids from piling up while chasing */
    for (int j = 0; j < f->n; j++) {
        if (j == idx) continue;
        float dx   = f->followers[j].px - b->px;
        float dy   = f->followers[j].py - b->py;
        float dist = hypotf(dx, dy);
        if (dist < SEPARATION_RADIUS && dist > 0.001f) {
            float closeness = (SEPARATION_RADIUS - dist) / SEPARATION_RADIUS;
            steer_x -= (dx / dist) * closeness * MAX_STEER;
            steer_y -= (dy / dist) * closeness * MAX_STEER;
        }
    }

    clamp2d(&steer_x, &steer_y, MAX_STEER);

    *out_vx = b->vx + steer_x;
    *out_vy = b->vy + steer_y;
}

/* ── flock tick ──────────────────────────────────────────────────────── */

/*
 * flock_tick — advance one flock by dt seconds.
 *
 * Two-stage update (simultaneous, not sequential):
 *   Stage 1: compute new velocity for every follower (reads old positions only).
 *   Stage 2: write new velocities and integrate positions.
 *
 * Without two stages, boids earlier in the array would react to already-moved
 * boids later in the array — the flock would drift in array order.
 */
static void flock_tick(Flock *f, FlockMode mode, float dt,
                        float max_px, float max_py)
{
    leader_tick(f, dt, max_px, max_py);

    /* Stage 1: read-only — compute each follower's desired new velocity */
    float new_vx[FOLLOWERS_MAX];
    float new_vy[FOLLOWERS_MAX];

    for (int i = 0; i < f->n; i++) {
        if (mode == MODE_BOIDS)
            boids_steer(f, i, &new_vx[i], &new_vy[i]);
        else
            chase_steer(f, i, &new_vx[i], &new_vy[i]);
    }

    /* Stage 2: write — apply velocities and move */
    for (int i = 0; i < f->n; i++) {
        Boid *b = &f->followers[i];
        b->vx   = new_vx[i];
        b->vy   = new_vy[i];
        boid_clamp_speed(b);
        b->px  += b->vx * dt;
        b->py  += b->vy * dt;
        boid_wrap(b, max_px, max_py);
    }
}

/* ── flock init ──────────────────────────────────────────────────────── */

/*
 * SPAWN_SCATTER — radius (px) within which followers are initially spread
 * around the leader's starting position. Large enough that boids don't
 * all start on the same cell, small enough that they start as a group.
 */
#define SPAWN_SCATTER  70.0f

/*
 * flock_init — place a flock in a screen quadrant with scattered followers.
 *
 * Starting quadrants (as fractions of pixel-space dimensions):
 *   flock 0 — top-left  (0.25, 0.25)
 *   flock 1 — top-right (0.75, 0.25)
 *   flock 2 — bottom    (0.50, 0.75)
 *
 * Placing flocks in different quadrants means the first few seconds show
 * three separate groups that gradually merge and interact — more visually
 * interesting than starting with everything in the center.
 */
static void flock_init(Flock *f, int flock_idx, int n_followers,
                        float max_px, float max_py)
{
    f->n           = n_followers;
    f->color_phase = (float)flock_idx;    /* kept in struct; used as draw identity */
    /* Random starting heading in [0, 2π] */
    f->wander_angle = (float)(rand() % 6284) / 1000.0f;

    static const float k_origin_x[FLOCKS] = { 0.25f, 0.75f, 0.50f };
    static const float k_origin_y[FLOCKS] = { 0.25f, 0.25f, 0.75f };

    float ox = max_px * k_origin_x[flock_idx];
    float oy = max_py * k_origin_y[flock_idx];

    boid_spawn_at(&f->leader, ox, oy, LEADER_SPEED);

    for (int i = 0; i < f->n; i++) {
        float off_x = ((float)(rand() % 2001) / 1000.0f - 1.0f) * SPAWN_SCATTER;
        float off_y = ((float)(rand() % 2001) / 1000.0f - 1.0f) * SPAWN_SCATTER;
        float sx    = ox + off_x;
        float sy    = oy + off_y;
        /* Clamp to pixel boundary in case scatter pushed outside the screen */
        if (sx < 0)        sx = 0;
        if (sx >= max_px)  sx = max_px - 1.0f;
        if (sy < 0)        sy = 0;
        if (sy >= max_py)  sy = max_py - 1.0f;
        boid_spawn_at(&f->followers[i], sx, sy, BOID_SPEED);
    }
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

typedef struct {
    Flock     flocks[FLOCKS];
    FlockMode mode;
    bool      paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->mode   = MODE_BOIDS;
    s->paused = false;

    float max_px = (float)pw(cols);
    float max_py = (float)ph(rows);

    for (int i = 0; i < FLOCKS; i++)
        flock_init(&s->flocks[i], i, FOLLOWERS_DEFAULT, max_px, max_py);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    if (s->paused) return;

    float max_px = (float)pw(cols);
    float max_py = (float)ph(rows);

    for (int i = 0; i < FLOCKS; i++)
        flock_tick(&s->flocks[i], s->mode, dt, max_px, max_py);
}

/*
 * follower_brightness — choose A_BOLD or A_NORMAL based on proximity to leader.
 *
 * Followers within the inner 35% of PERCEPTION_RADIUS glow bright (A_BOLD),
 * the rest are normal weight. No A_DIM: using A_DIM caused the whole flock
 * to visually dim whenever the leader wrapped to the opposite edge (toroidal
 * wrapping makes Euclidean distance spike briefly to ~max_px, pushing all
 * followers past the dim threshold simultaneously).
 */
static int follower_brightness(const Boid *follower, const Boid *leader)
{
    float dist  = hypotf(follower->px - leader->px, follower->py - leader->py);
    float ratio = dist / PERCEPTION_RADIUS;
    return (ratio < 0.35f) ? A_BOLD : A_NORMAL;
}

/*
 * draw_boid — draw a single boid into window w using render interpolation.
 *
 * Interpolation: project position forward by (alpha × dt_sec) seconds from
 * the last physics tick using current velocity. This makes drawn positions
 * match wall-clock "now" rather than the last tick boundary, eliminating
 * micro-stutter. Same technique as bounce.c §6 scene_draw.
 */
static void draw_boid(WINDOW *w, const Boid *b, char ch, int color_attr,
                       int cols, int rows, float max_px, float max_py,
                       float alpha, float dt_sec)
{
    float draw_px = b->px + b->vx * alpha * dt_sec;
    float draw_py = b->py + b->vy * alpha * dt_sec;

    /* Clamp interpolated position — alpha < 1 so overshoot is sub-cell */
    if (draw_px < 0)       draw_px = 0;
    if (draw_px > max_px)  draw_px = max_px;
    if (draw_py < 0)       draw_py = 0;
    if (draw_py > max_py)  draw_py = max_py;

    int cx = px_to_cell_x(draw_px);
    int cy = px_to_cell_y(draw_py);

    if (cx >= 0 && cx < cols && cy >= 0 && cy < rows) {
        wattron(w, color_attr);
        mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
        wattroff(w, color_attr);
    }
}

/*
 * scene_draw — draw all flocks using interpolated positions.
 *
 * Draw order per flock: followers first, leader last.
 * Leaders are drawn on top so they're never obscured by their own flock.
 */
static void scene_draw(const Scene *s, WINDOW *w,
                        int cols, int rows,
                        float alpha, float dt_sec)
{
    float max_px = (float)(pw(cols) - 1);
    float max_py = (float)(ph(rows) - 1);

    for (int fi = 0; fi < FLOCKS; fi++) {
        const Flock *f = &s->flocks[fi];

        /* Followers: flock color, brightness by distance to their leader */
        int follower_pair = follower_color_pair(fi);
        for (int i = 0; i < f->n; i++) {
            const Boid *b    = &f->followers[i];
            char        ch   = velocity_dir_char(b->vx, b->vy);
            int         attr = COLOR_PAIR(follower_pair) | follower_brightness(b, &f->leader);
            draw_boid(w, b, ch, attr, cols, rows, max_px, max_py, alpha, dt_sec);
        }

        /* Leader: its own distinct bright color, always bold, drawn on top */
        int leader_pair = leader_color_pair(fi);
        draw_boid(w, &f->leader, '@', COLOR_PAIR(leader_pair) | A_BOLD,
                  cols, rows, max_px, max_py, alpha, dt_sec);
    }
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);      /* never interrupt output to poll stdin */
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();    /* re-reads LINES and COLS after terminal resize */
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw — build one complete frame in stdscr (ncurses' newscr).
 *
 * Order:
 *   1. erase() — clear newscr so stale boid glyphs become spaces.
 *   2. scene_draw() — write all boids at interpolated positions.
 *   3. HUD line — overwrite top-left after boids (always on top).
 *
 * Nothing reaches the terminal until screen_present() is called.
 */
static void screen_draw(Screen *s, Scene *sc,
                         double fps, float alpha, float dt_sec)
{
    erase();

    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    /* HUD */
    int total_boids = 0;
    for (int i = 0; i < FLOCKS; i++) total_boids += sc->flocks[i].n + 1;

    attron(COLOR_PAIR(HUD_COLOR_PAIR));
    mvprintw(0, 0, "FLOCKING  mode:%-26s  boids:%-3d  %s  %.0ffps"
                   "  1-2:mode  +/-:boids  r:reset  spc:pause  q:quit",
             k_mode_names[sc->mode], total_boids,
             sc->paused ? "PAUSED " : "running", fps);
    attroff(COLOR_PAIR(HUD_COLOR_PAIR));
}

static void screen_present(void)
{
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

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);

    float max_px = (float)pw(app->screen.cols);
    float max_py = (float)ph(app->screen.rows);

    /* Clamp any boid that is outside the new (smaller) pixel boundary */
    for (int fi = 0; fi < FLOCKS; fi++) {
        Flock *f = &app->scene.flocks[fi];
        if (f->leader.px >= max_px) f->leader.px = max_px - 1.0f;
        if (f->leader.py >= max_py) f->leader.py = max_py - 1.0f;
        for (int i = 0; i < f->n; i++) {
            if (f->followers[i].px >= max_px) f->followers[i].px = max_px - 1.0f;
            if (f->followers[i].py >= max_py) f->followers[i].py = max_py - 1.0f;
        }
    }
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s    = &app->scene;
    int    cols = app->screen.cols;
    int    rows = app->screen.rows;

    switch (ch) {
    case 'q': case 'Q': case 27:    /* ESC */
        return false;

    case ' ':
        s->paused = !s->paused;
        break;

    case '1':  s->mode = MODE_BOIDS;  break;
    case '2':  s->mode = MODE_CHASE;  break;

    case '=': case '+':
        for (int i = 0; i < FLOCKS; i++)
            if (s->flocks[i].n < FOLLOWERS_MAX)
                s->flocks[i].n++;
        break;

    case '-':
        for (int i = 0; i < FLOCKS; i++)
            if (s->flocks[i].n > FOLLOWERS_MIN)
                s->flocks[i].n--;
        break;

    case 'r': case 'R':
        scene_init(s, cols, rows);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── dt ──────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        /* Cap dt at 100 ms to prevent a physics explosion if the
         * process is paused by the OS (e.g. SIGSTOP or debugger) */
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── fixed-step accumulator ──────────────────────────────── */
        int64_t tick_ns = TICK_NS(SIM_FPS);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec, app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /*
         * ── render interpolation alpha ────────────────────────────
         * sim_accum after draining = leftover time into the next tick.
         * alpha = sim_accum / tick_ns ∈ [0, 1)
         * Passed to scene_draw to project boid positions forward by
         * this fractional tick, matching wall-clock "now".
         */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── FPS counter ─────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap (sleep before render to avoid I/O drift) ─── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / RENDER_FPS - elapsed);

        /* ── draw + present ──────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene, fps_display, alpha, dt_sec);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
