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
 *   1         mode: Classic Boids    (separation + alignment + cohesion)
 *   2         mode: Leader Chase     (followers home directly on leader)
 *   3         mode: Vicsek           (align to neighbors + noise angle)
 *   4         mode: Orbit Formation  (followers spin on ring around leader)
 *   5         mode: Predator-Prey    (flock 0 hunts; flocks 1-2 flee)
 *   n / m     decrease / increase Vicsek noise level
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
 * Vicsek model parameters.
 *
 * In the Vicsek model (1995) each boid aligns to its neighbors' average
 * heading and then adds a random noise angle. This single parameter drives
 * a phase transition between two regimes:
 *   low noise  → all boids stream in one direction (ordered, polarised)
 *   high noise → every boid moves independently (disordered, chaotic)
 *
 * VICSEK_NOISE is the maximum random angle perturbation per tick (radians).
 * 0.30 rad ≈ 17° — enough to see occasional bursts of disorder without
 * making the flock completely random at normal density.
 * Press 'n' / 'm' at runtime to decrease / increase noise interactively.
 */
#define VICSEK_NOISE_DEFAULT  0.30f   /* starting noise level, radians     */
#define VICSEK_NOISE_MIN      0.05f   /* near-perfect order (parallel beams) */
#define VICSEK_NOISE_MAX      1.80f   /* near-random (barely any coherence) */
#define VICSEK_NOISE_STEP     0.10f   /* per keypress increment            */

/*
 * Orbit formation parameters.
 *
 * Followers are assigned evenly-spaced slots on a ring of radius ORBIT_RADIUS
 * centred on their leader. The ring rotates at ORBIT_SPEED rad/s, so each
 * boid's target position moves along the ring — it chases a moving point,
 * producing a spinning halo effect.
 *
 * Tangential ring speed = ORBIT_RADIUS × ORBIT_SPEED = 120 × 1.4 ≈ 168 px/s.
 * BOID_SPEED (280 px/s) > 168 px/s, so followers always keep up with their
 * assigned slot.
 */
#define ORBIT_RADIUS  120.0f    /* px — ring radius around leader         */
#define ORBIT_SPEED     1.4f    /* rad/s — ring rotation speed            */

/*
 * Predator-Prey parameters.
 *
 * Flock 0 is the predator; flocks 1 and 2 are prey.
 *
 * PREDATOR_CHASE_RADIUS: how far the predator leader can "see" prey boids.
 *   When a prey boid enters this range the leader steers toward the nearest
 *   one; outside the range it wanders normally.
 *
 * PREY_FLEE_RADIUS: how close a predator boid must be before prey start
 *   fleeing. Larger than separation radius so prey react before contact.
 *
 * W_FLEE: flee force weight. Must be substantially stronger than cohesion
 *   (0.5) so prey actually break out of their flock rather than merely
 *   jostling and staying put.
 */
#define PREDATOR_CHASE_RADIUS  280.0f   /* px — predator sight range        */
#define PREY_FLEE_RADIUS       160.0f   /* px — prey panic zone             */
#define W_FLEE                   3.0f   /* flee force weight (beats cohesion) */

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
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Each flock has TWO color pairs: one for followers, one for the leader.
 * Leader color contrasts its flock so it always pops visually.
 *
 * We use xterm-256 when available for the three vivid flock colors —
 * orange in particular has no equivalent in the 16-color palette.
 * 16-color fallbacks are provided for basic terminals.
 *
 * Follower colors (xterm-256 indices):
 *   flock 0 — matrix green  : 46   (#00ff00  pure bright green)
 *   flock 1 — fire orange   : 208  (#ff8700  deep amber-orange)
 *   flock 2 — electric blue : 33   (#0087ff  dodger blue)
 *
 * Leader colors — chosen to be complementary / high-contrast:
 *   flock 0 green  leader — bright yellow : 226  (warm, pops on green)
 *   flock 1 orange leader — ice cyan      : 51   (cool complement to orange)
 *   flock 2 blue   leader — bright white  : 231  (neutral, clean on blue)
 *
 * 16-color fallbacks (when COLORS < 256):
 *   followers: COLOR_GREEN / COLOR_RED / COLOR_BLUE
 *   leaders:   COLOR_YELLOW / COLOR_CYAN / COLOR_CYAN
 *
 * Color pair layout:
 *   pairs 1 … FLOCKS          — follower colors
 *   pairs FLOCKS+1 … 2*FLOCKS — leader colors
 *   pair  2*FLOCKS + 1        — HUD
 */

/* xterm-256 follower colors */
#define COLOR_256_MATRIX_GREEN   46    /* #00ff00 — pure bright green         */
#define COLOR_256_FIRE_ORANGE   208    /* #ff8700 — deep amber-orange         */
#define COLOR_256_ELECTRIC_BLUE  33    /* #0087ff — vivid dodger blue;
                                        * violet (93) was too dark on black
                                        * backgrounds — blue-purples lose
                                        * brightness fast on 256-color palettes.
                                        * Dodger blue sits in the middle of the
                                        * blue ramp where luminance is highest. */

/* xterm-256 leader colors — complementary to their flock */
#define COLOR_256_LEADER_YELLOW 226    /* #ffff00 — warm yellow  (vs green) */
#define COLOR_256_LEADER_CYAN    51    /* #00ffff — ice cyan     (vs orange) */
#define COLOR_256_LEADER_WHITE  231    /* #ffffff — bright white (vs blue,
                                        * neutral contrast works well here
                                        * because blue is already saturated) */

static int follower_color_pair(int flock_idx) { return flock_idx + 1;          }
static int leader_color_pair  (int flock_idx) { return flock_idx + 1 + FLOCKS; }
#define HUD_COLOR_PAIR  (2 * FLOCKS + 1)

static void color_init(void)
{
    start_color();

    if (COLORS >= 256) {
        /* xterm-256: vivid named colors */
        init_pair(follower_color_pair(0), COLOR_256_MATRIX_GREEN, COLOR_BLACK);
        init_pair(follower_color_pair(1), COLOR_256_FIRE_ORANGE,  COLOR_BLACK);
        init_pair(follower_color_pair(2), COLOR_256_ELECTRIC_BLUE, COLOR_BLACK);

        init_pair(leader_color_pair(0), COLOR_256_LEADER_YELLOW, COLOR_BLACK);
        init_pair(leader_color_pair(1), COLOR_256_LEADER_CYAN,   COLOR_BLACK);
        init_pair(leader_color_pair(2), COLOR_256_LEADER_WHITE,  COLOR_BLACK);
    } else {
        /* 16-color fallback — closest approximations */
        init_pair(follower_color_pair(0), COLOR_GREEN,   COLOR_BLACK);
        init_pair(follower_color_pair(1), COLOR_RED,     COLOR_BLACK);
        init_pair(follower_color_pair(2), COLOR_BLUE,    COLOR_BLACK);

        init_pair(leader_color_pair(0), COLOR_YELLOW, COLOR_BLACK);
        init_pair(leader_color_pair(1), COLOR_CYAN,   COLOR_BLACK);
        init_pair(leader_color_pair(2), COLOR_CYAN,   COLOR_BLACK);
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
    float px, py;          /* position (pixels)                          */
    float vx, vy;          /* velocity (pixels / second)                 */
    float cruise_speed;    /* personal target speed, px/s; varies ±15%  */
} Boid;

/*
 * velocity_dir_char — pick an ASCII glyph for the boid's direction of travel.
 *
 * Each flock uses its own diagonal characters so flocks stay visually
 * identifiable when they intermix — you can distinguish them by shape,
 * not just color. The four cardinal glyphs (< ^ > v) are shared since
 * they clearly suggest direction; only the diagonals differ:
 *
 *   flock 0: slash/backslash   < \ ^ / > \ v /   (sharp angles)
 *   flock 1: tilde             < ~ ^ ~ > ~ v ~   (wave-like)
 *   flock 2: plus              < + ^ + > + v +   (cross-like)
 *
 * Sector layout (clockwise from West, +y = down in pixel space):
 *   index: 0   1    2   3    4   5    6   7
 *   dir:   W   NW   N   NE   E   SE   S   SW
 *
 * Formula: atan2(vy,vx) → shift to [0,2π] → center sectors by adding π/8
 *          → divide by sector width π/4 → floor → mod 8 = sector 0–7.
 */
static char velocity_dir_char(float vx, float vy, int flock_idx)
{
    static const char k_chars[FLOCKS][8] = {
        {'<', '\\', '^', '/', '>', '\\', 'v', '/'},   /* flock 0: \ and / */
        {'<',  '~', '^', '~', '>', '~',  'v', '~'},   /* flock 1: ~       */
        {'<',  '+', '^', '+', '>', '+',  'v', '+'},   /* flock 2: +       */
    };

    float angle   = atan2f(vy, vx);
    float shifted = angle + (float)M_PI + (float)M_PI / 8.0f;
    int   sector  = (int)floorf(shifted / ((float)M_PI / 4.0f)) % 8;
    return k_chars[flock_idx][sector];
}

/*
 * boid_spawn_at — place a boid at (px, py) with a random direction at
 * the given speed. Uses rejection-sample unit vector to get a uniform
 * random angle distribution without calling atan2/sinf/cosf at spawn.
 */
static void boid_spawn_at(Boid *b, float px, float py, float speed)
{
    b->px          = px;
    b->py          = py;
    b->cruise_speed = speed;

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

/*
 * toroidal_delta — shortest signed displacement from position a to b
 * on a single axis of length `extent`.
 *
 * On a toroidal (wrap-around) axis, there are two paths between any two
 * points: the "direct" path and the "wrap" path. We always return the
 * shorter one, with sign indicating direction.
 *
 * Example (extent=100): a=5, b=95 → direct=+90, wrap=-10 → returns -10.
 * Used by boids rules and brightness so boids near opposite edges are
 * treated as close, matching the wrapping physics.
 */
static float toroidal_delta(float a, float b, float extent)
{
    float d = b - a;
    if (d >  extent * 0.5f) d -= extent;
    if (d < -extent * 0.5f) d += extent;
    return d;
}

/* ===================================================================== */
/* §6  flock                                                              */
/* ===================================================================== */

/* Five switchable steering algorithms */
typedef enum {
    MODE_BOIDS    = 0,  /* Reynolds 1987: separation + alignment + cohesion     */
    MODE_CHASE    = 1,  /* followers ignore peers and home on their leader       */
    MODE_VICSEK   = 2,  /* Vicsek 1995: align to neighbors + noise angle        */
    MODE_ORBIT    = 3,  /* followers circle leader on a rotating ring            */
    MODE_PREDATOR = 4,  /* flock 0 hunts; flocks 1-2 flee                       */
    MODE_COUNT
} FlockMode;

static const char *k_mode_names[MODE_COUNT] = {
    "BOIDS    sep+align+cohesion",
    "CHASE    leader homing",
    "VICSEK   align+noise",
    "ORBIT    spinning ring",
    "PREDATOR hunt/flee",
};

typedef struct {
    Boid  leader;
    Boid  followers[FOLLOWERS_MAX];
    int   n;              /* active follower count             */
    float color_phase;    /* hue offset in cosine palette      */
    float wander_angle;   /* leader heading in radians         */
    float orbit_phase;    /* MODE_ORBIT: ring rotation angle   */
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
static void boids_steer(const Flock *f, int idx, float max_px, float max_py,
                         float *out_vx, float *out_vy)
{
    const Boid *b = &f->followers[idx];

    /* Accumulators for each rule */
    float sep_x  = 0.0f, sep_y  = 0.0f;   /* push vectors away from close boids     */
    float ali_vx = 0.0f, ali_vy = 0.0f;   /* sum of neighbor velocities             */
    float coh_dx = 0.0f, coh_dy = 0.0f;   /* sum of toroidal offsets toward neighbors */
    int   neighbor_n = 0;
    int   sep_n      = 0;

    for (int j = 0; j < f->n; j++) {
        if (j == idx) continue;

        const Boid *nb = &f->followers[j];

        /*
         * Toroidal displacement: use the shortest path on each axis.
         * Without this, boids near opposite edges (e.g. px=5 and px=max-5)
         * appear far apart and ignore each other, causing flocks to thin
         * and behave erratically near screen edges.
         */
        float dx   = toroidal_delta(b->px, nb->px, max_px);
        float dy   = toroidal_delta(b->py, nb->py, max_py);
        float dist = hypotf(dx, dy);

        if (dist >= PERCEPTION_RADIUS || dist < 0.001f) continue;

        /* Alignment: accumulate neighbor velocities */
        ali_vx += nb->vx;
        ali_vy += nb->vy;

        /*
         * Cohesion: accumulate toroidal offset (not absolute position).
         * Naively averaging absolute positions fails on a torus — the
         * average of px=5 and px=95 on a width-100 field gives 50 (center),
         * but the correct center-of-mass is 0 (the wrap point). Storing
         * toroidal offsets relative to b, then averaging, gives the correct
         * direction to steer toward regardless of screen position.
         */
        coh_dx += dx;
        coh_dy += dy;
        neighbor_n++;

        /* Separation: push away from boids within personal-space radius */
        if (dist < SEPARATION_RADIUS) {
            float closeness = (SEPARATION_RADIUS - dist) / SEPARATION_RADIUS;
            sep_x -= (dx / dist) * closeness;
            sep_y -= (dy / dist) * closeness;
            sep_n++;
        }
    }

    /* Accumulate total steering force */
    float steer_x = 0.0f, steer_y = 0.0f;

    /* Separation */
    if (sep_n > 0) {
        float sm = hypotf(sep_x, sep_y);
        if (sm > 0.001f) {
            steer_x += (sep_x / sm) * W_SEPARATION * MAX_STEER;
            steer_y += (sep_y / sm) * W_SEPARATION * MAX_STEER;
        }
    }

    /* Alignment: match average neighbor heading at this boid's own cruise speed */
    if (neighbor_n > 0) {
        float avg_vx = ali_vx / (float)neighbor_n;
        float avg_vy = ali_vy / (float)neighbor_n;
        float am     = hypotf(avg_vx, avg_vy);
        if (am > 0.001f) {
            /* Use cruise_speed (personal target) not global BOID_SPEED.
             * Fast boids naturally push forward; slow ones drift back —
             * producing organic depth variation within the flock. */
            float desired_vx = (avg_vx / am) * b->cruise_speed;
            float desired_vy = (avg_vy / am) * b->cruise_speed;
            steer_x += (desired_vx - b->vx) * W_ALIGNMENT;
            steer_y += (desired_vy - b->vy) * W_ALIGNMENT;
        }
    }

    /* Cohesion: steer toward average toroidal center-of-mass offset */
    if (neighbor_n > 0) {
        float avg_dx = coh_dx / (float)neighbor_n;
        float avg_dy = coh_dy / (float)neighbor_n;
        float cm     = hypotf(avg_dx, avg_dy);
        if (cm > 0.001f) {
            float desired_vx = (avg_dx / cm) * b->cruise_speed;
            float desired_vy = (avg_dy / cm) * b->cruise_speed;
            steer_x += (desired_vx - b->vx) * W_COHESION;
            steer_y += (desired_vy - b->vy) * W_COHESION;
        }
    }

    /* Leader pull: toroidal shortest-path attraction toward own leader */
    {
        float lx = toroidal_delta(b->px, f->leader.px, max_px);
        float ly = toroidal_delta(b->py, f->leader.py, max_py);
        float lm = hypotf(lx, ly);
        if (lm > 0.001f) {
            steer_x += (lx / lm) * W_LEADER_PULL * MAX_STEER;
            steer_y += (ly / lm) * W_LEADER_PULL * MAX_STEER;
        }
    }

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
static void chase_steer(const Flock *f, int idx, float max_px, float max_py,
                          float *out_vx, float *out_vy)
{
    const Boid *b = &f->followers[idx];
    float steer_x = 0.0f, steer_y = 0.0f;

    /* Steer toward leader via shortest toroidal path */
    float lx = toroidal_delta(b->px, f->leader.px, max_px);
    float ly = toroidal_delta(b->py, f->leader.py, max_py);
    float lm = hypotf(lx, ly);
    if (lm > 0.001f) {
        float desired_vx = (lx / lm) * b->cruise_speed;
        float desired_vy = (ly / lm) * b->cruise_speed;
        steer_x = desired_vx - b->vx;
        steer_y = desired_vy - b->vy;
    }

    /* Separation: prevent boids from piling up while chasing */
    for (int j = 0; j < f->n; j++) {
        if (j == idx) continue;
        float dx   = toroidal_delta(b->px, f->followers[j].px, max_px);
        float dy   = toroidal_delta(b->py, f->followers[j].py, max_py);
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

/* ── Vicsek steering (MODE_VICSEK) ───────────────────────────────────── */

/*
 * vicsek_steer — Vicsek (1995) model: each boid aligns to the average
 * heading of all neighbors within PERCEPTION_RADIUS, then adds a random
 * noise angle.
 *
 * This one-parameter model drives a sharp phase transition:
 *   low noise  → entire flock streams in one direction (ordered / polarised)
 *   high noise → every boid moves independently (disordered / chaotic)
 *
 * Unlike Reynolds boids there is no explicit cohesion or separation force;
 * the spatial coherence comes purely from the alignment interaction.
 *
 * The leader is included in the neighbor set so the flock stays oriented
 * around its leader's wandering direction at low noise.
 *
 * noise_scale: max random perturbation in radians (VICSEK_NOISE_DEFAULT).
 *              Adjustable live with 'n' / 'm' keys.
 */
static void vicsek_steer(const Flock *f, int idx, float max_px, float max_py,
                           float noise_scale, float *out_vx, float *out_vy)
{
    const Boid *b = &f->followers[idx];

    /* Sum velocity vectors of all boids in perception radius (include self) */
    float sum_vx = b->vx;
    float sum_vy = b->vy;
    int   count  = 1;

    /* Include leader — anchors the flock direction when noise is low */
    {
        float dx = toroidal_delta(b->px, f->leader.px, max_px);
        float dy = toroidal_delta(b->py, f->leader.py, max_py);
        if (hypotf(dx, dy) < PERCEPTION_RADIUS) {
            sum_vx += f->leader.vx;
            sum_vy += f->leader.vy;
            count++;
        }
    }

    for (int j = 0; j < f->n; j++) {
        if (j == idx) continue;
        float dx = toroidal_delta(b->px, f->followers[j].px, max_px);
        float dy = toroidal_delta(b->py, f->followers[j].py, max_py);
        if (hypotf(dx, dy) < PERCEPTION_RADIUS) {
            sum_vx += f->followers[j].vx;
            sum_vy += f->followers[j].vy;
            count++;
        }
    }

    /* Average heading angle, then add random noise */
    float avg_angle = atan2f(sum_vy / (float)count, sum_vx / (float)count);
    float noise     = ((float)(rand() % 2001) / 1000.0f - 1.0f) * noise_scale;
    float new_angle = avg_angle + noise;

    /* Maintain personal cruise speed — only direction comes from Vicsek */
    *out_vx = cosf(new_angle) * b->cruise_speed;
    *out_vy = sinf(new_angle) * b->cruise_speed;
}

/* ── orbit steering (MODE_ORBIT) ─────────────────────────────────────── */

/*
 * orbit_steer — assign each follower a slot on a ring of radius ORBIT_RADIUS
 * centred on the leader. The ring rotates at ORBIT_SPEED rad/s (stored in
 * f->orbit_phase and advanced each tick). Each follower chases its moving slot.
 *
 * Slot assignment: slot angle = orbit_phase + 2π × idx / n
 * Evenly-spaced slots produce a spinning halo; as the phase advances every
 * boid moves along the ring rather than parking in one place.
 *
 * Separation is still applied so boids don't stack when they close in
 * on the same slot or when the flock is small.
 */
static void orbit_steer(const Flock *f, int idx, float max_px, float max_py,
                          float *out_vx, float *out_vy)
{
    const Boid *b = &f->followers[idx];

    /* Target point on the rotating ring around the leader */
    float slot_angle = f->orbit_phase
                     + 2.0f * (float)M_PI * (float)idx / (float)f->n;
    float target_px  = f->leader.px + ORBIT_RADIUS * cosf(slot_angle);
    float target_py  = f->leader.py + ORBIT_RADIUS * sinf(slot_angle);

    /* Toroidal displacement to target */
    float dx = toroidal_delta(b->px, target_px, max_px);
    float dy = toroidal_delta(b->py, target_py, max_py);
    float dm = hypotf(dx, dy);

    float steer_x = 0.0f, steer_y = 0.0f;
    if (dm > 0.001f) {
        float desired_vx = (dx / dm) * b->cruise_speed;
        float desired_vy = (dy / dm) * b->cruise_speed;
        steer_x = desired_vx - b->vx;
        steer_y = desired_vy - b->vy;
    }

    /* Separation: prevent boids from overlapping en route to their slots */
    for (int j = 0; j < f->n; j++) {
        if (j == idx) continue;
        float sdx  = toroidal_delta(b->px, f->followers[j].px, max_px);
        float sdy  = toroidal_delta(b->py, f->followers[j].py, max_py);
        float dist = hypotf(sdx, sdy);
        if (dist < SEPARATION_RADIUS && dist > 0.001f) {
            float closeness = (SEPARATION_RADIUS - dist) / SEPARATION_RADIUS;
            steer_x -= (sdx / dist) * closeness * MAX_STEER;
            steer_y -= (sdy / dist) * closeness * MAX_STEER;
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
static void flock_tick(Flock *f, FlockMode mode, float vicsek_noise, float dt,
                        float max_px, float max_py)
{
    leader_tick(f, dt, max_px, max_py);

    /* Orbit ring rotates independently of boid positions */
    if (mode == MODE_ORBIT)
        f->orbit_phase += ORBIT_SPEED * dt;

    /* Stage 1: read-only — compute each follower's desired new velocity */
    float new_vx[FOLLOWERS_MAX];
    float new_vy[FOLLOWERS_MAX];

    for (int i = 0; i < f->n; i++) {
        switch (mode) {
        case MODE_BOIDS:
            boids_steer(f, i, max_px, max_py, &new_vx[i], &new_vy[i]);
            break;
        case MODE_VICSEK:
            vicsek_steer(f, i, max_px, max_py, vicsek_noise, &new_vx[i], &new_vy[i]);
            break;
        case MODE_ORBIT:
            orbit_steer(f, i, max_px, max_py, &new_vx[i], &new_vy[i]);
            break;
        default: /* MODE_CHASE and any unhandled: simple homing */
            chase_steer(f, i, max_px, max_py, &new_vx[i], &new_vy[i]);
            break;
        }
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
        /* ±15% speed variation: makes fast boids push forward naturally,
         * slow ones drift back — creates organic depth in the flock */
        float variation = 0.85f + 0.30f * ((float)(rand() % 1001) / 1000.0f);
        boid_spawn_at(&f->followers[i], sx, sy, BOID_SPEED * variation);
    }
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

typedef struct {
    Flock     flocks[FLOCKS];
    FlockMode mode;
    bool      paused;
    float     vicsek_noise;   /* MODE_VICSEK: current noise level (radians) */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->mode         = MODE_BOIDS;
    s->paused       = false;
    s->vicsek_noise = VICSEK_NOISE_DEFAULT;

    float max_px = (float)pw(cols);
    float max_py = (float)ph(rows);

    for (int i = 0; i < FLOCKS; i++)
        flock_init(&s->flocks[i], i, FOLLOWERS_DEFAULT, max_px, max_py);
}

/* ── predator-prey tick (MODE_PREDATOR) ──────────────────────────────── */

/*
 * predator_leader_tick — advance the predator's (flock 0) leader.
 *
 * If any prey boid is within PREDATOR_CHASE_RADIUS the leader steers toward
 * the nearest one; otherwise it wanders normally. The wander_angle is set
 * to the bearing of the target, so the same LEADER_SPEED constant is used.
 *
 * prey_flocks: pointer to flocks[1] (first prey flock).
 * n_prey:      number of prey flocks (FLOCKS - 1 = 2).
 */
static void predator_leader_tick(Flock *predator,
                                   const Flock *prey_flocks, int n_prey,
                                   float dt, float max_px, float max_py)
{
    /* Find nearest prey boid (followers + leader of each prey flock) */
    float nearest_dist = PREDATOR_CHASE_RADIUS + 1.0f;
    float target_px    = 0.0f, target_py = 0.0f;
    bool  found        = false;

    for (int fi = 0; fi < n_prey; fi++) {
        /* Check leader */
        {
            const Boid *prey = &prey_flocks[fi].leader;
            float dx   = toroidal_delta(predator->leader.px, prey->px, max_px);
            float dy   = toroidal_delta(predator->leader.py, prey->py, max_py);
            float dist = hypotf(dx, dy);
            if (dist < nearest_dist) {
                nearest_dist = dist;
                target_px = prey->px;
                target_py = prey->py;
                found = true;
            }
        }
        /* Check followers */
        for (int i = 0; i < prey_flocks[fi].n; i++) {
            const Boid *prey = &prey_flocks[fi].followers[i];
            float dx   = toroidal_delta(predator->leader.px, prey->px, max_px);
            float dy   = toroidal_delta(predator->leader.py, prey->py, max_py);
            float dist = hypotf(dx, dy);
            if (dist < nearest_dist) {
                nearest_dist = dist;
                target_px = prey->px;
                target_py = prey->py;
                found = true;
            }
        }
    }

    if (found) {
        /* Snap wander_angle to bearing of nearest prey */
        float dx = toroidal_delta(predator->leader.px, target_px, max_px);
        float dy = toroidal_delta(predator->leader.py, target_py, max_py);
        predator->wander_angle = atan2f(dy, dx);
    } else {
        /* No prey in range — wander randomly */
        float jitter = ((float)(rand() % 2001) / 1000.0f - 1.0f) * WANDER_JITTER;
        predator->wander_angle += jitter;
    }

    predator->leader.vx = LEADER_SPEED * cosf(predator->wander_angle);
    predator->leader.vy = LEADER_SPEED * sinf(predator->wander_angle);
    predator->leader.px += predator->leader.vx * dt;
    predator->leader.py += predator->leader.vy * dt;
    boid_wrap(&predator->leader, max_px, max_py);
}

/*
 * prey_boids_steer — boids rules (separation + alignment + cohesion) PLUS
 * a flee impulse away from any predator boid within PREY_FLEE_RADIUS.
 *
 * The flee force is added after boids_steer (which writes out_vx/out_vy),
 * so we first call boids_steer and then accumulate the flee vector on top.
 * W_FLEE (3.0) >> W_COHESION (0.5) so prey actually break formation rather
 * than just jostling in place.
 */
static void prey_boids_steer(const Flock *f, int idx,
                               const Flock *predator,
                               float max_px, float max_py,
                               float *out_vx, float *out_vy)
{
    /* Start with normal boids steering */
    boids_steer(f, idx, max_px, max_py, out_vx, out_vy);

    const Boid *b = &f->followers[idx];
    float flee_x  = 0.0f;
    float flee_y  = 0.0f;

    /* Flee from predator leader */
    {
        float dx   = toroidal_delta(b->px, predator->leader.px, max_px);
        float dy   = toroidal_delta(b->py, predator->leader.py, max_py);
        float dist = hypotf(dx, dy);
        if (dist < PREY_FLEE_RADIUS && dist > 0.001f) {
            /* Push in opposite direction (flee_x -= towards-predator) */
            flee_x -= dx / dist;
            flee_y -= dy / dist;
        }
    }

    /* Flee from each predator follower */
    for (int i = 0; i < predator->n; i++) {
        float dx   = toroidal_delta(b->px, predator->followers[i].px, max_px);
        float dy   = toroidal_delta(b->py, predator->followers[i].py, max_py);
        float dist = hypotf(dx, dy);
        if (dist < PREY_FLEE_RADIUS && dist > 0.001f) {
            flee_x -= dx / dist;
            flee_y -= dy / dist;
        }
    }

    /* Apply flee impulse */
    float fm = hypotf(flee_x, flee_y);
    if (fm > 0.001f) {
        *out_vx += (flee_x / fm) * W_FLEE * MAX_STEER;
        *out_vy += (flee_y / fm) * W_FLEE * MAX_STEER;
    }
}

/*
 * scene_tick_predator — one predator-prey physics step.
 *
 * Flock 0 (predator):
 *   - Leader hunts nearest prey boid within PREDATOR_CHASE_RADIUS.
 *   - Followers chase their own leader (same as MODE_CHASE).
 *
 * Flocks 1-2 (prey):
 *   - Leaders wander normally.
 *   - Followers: boids_steer + flee from any predator boid in range.
 *
 * All prey flocks run simultaneously (same two-stage pattern), so no
 * prey flock reacts to the already-moved positions of another.
 */
static void scene_tick_predator(Scene *s, float dt, float max_px, float max_py)
{
    Flock *predator = &s->flocks[0];

    /* ── predator ───────────────────────────────────────────────── */
    predator_leader_tick(predator, &s->flocks[1], FLOCKS - 1, dt, max_px, max_py);

    float new_vx[FOLLOWERS_MAX];
    float new_vy[FOLLOWERS_MAX];

    for (int i = 0; i < predator->n; i++)
        chase_steer(predator, i, max_px, max_py, &new_vx[i], &new_vy[i]);
    for (int i = 0; i < predator->n; i++) {
        Boid *b = &predator->followers[i];
        b->vx   = new_vx[i];
        b->vy   = new_vy[i];
        boid_clamp_speed(b);
        b->px  += b->vx * dt;
        b->py  += b->vy * dt;
        boid_wrap(b, max_px, max_py);
    }

    /* ── prey ───────────────────────────────────────────────────── */
    for (int fi = 1; fi < FLOCKS; fi++) {
        Flock *f = &s->flocks[fi];
        leader_tick(f, dt, max_px, max_py);

        for (int i = 0; i < f->n; i++)
            prey_boids_steer(f, i, predator, max_px, max_py, &new_vx[i], &new_vy[i]);
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
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    if (s->paused) return;

    float max_px = (float)pw(cols);
    float max_py = (float)ph(rows);

    if (s->mode == MODE_PREDATOR) {
        scene_tick_predator(s, dt, max_px, max_py);
        return;
    }

    for (int i = 0; i < FLOCKS; i++)
        flock_tick(&s->flocks[i], s->mode, s->vicsek_noise, dt, max_px, max_py);
}

/*
 * follower_brightness — choose A_BOLD or A_NORMAL based on proximity to leader.
 *
 * Uses toroidal shortest-path distance so the gradient stays correct even
 * when the leader has just wrapped to the opposite edge — followers that are
 * physically close across the wrap boundary still glow bright.
 */
static int follower_brightness(const Boid *follower, const Boid *leader,
                                float max_px, float max_py)
{
    float dx    = toroidal_delta(follower->px, leader->px, max_px);
    float dy    = toroidal_delta(follower->py, leader->py, max_py);
    float ratio = hypotf(dx, dy) / PERCEPTION_RADIUS;
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

        /* Followers: flock color, direction char, brightness by toroidal distance */
        int follower_pair = follower_color_pair(fi);
        for (int i = 0; i < f->n; i++) {
            const Boid *b    = &f->followers[i];
            char        ch   = velocity_dir_char(b->vx, b->vy, fi);
            int         attr = COLOR_PAIR(follower_pair)
                             | follower_brightness(b, &f->leader, max_px, max_py);
            draw_boid(w, b, ch, attr, cols, rows, max_px, max_py, alpha, dt_sec);
        }

        /* Leader: its own color, direction char (shows heading), always bold */
        int  leader_pair = leader_color_pair(fi);
        char leader_ch   = velocity_dir_char(f->leader.vx, f->leader.vy, fi);
        draw_boid(w, &f->leader, leader_ch, COLOR_PAIR(leader_pair) | A_BOLD,
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
    if (sc->mode == MODE_VICSEK) {
        mvprintw(0, 0, "FLOCKING  mode:%-28s  noise:%.2f  boids:%-3d  %s  %.0ffps"
                       "  1-5:mode  n/m:noise  +/-:boids  r:reset  spc:pause  q:quit",
                 k_mode_names[sc->mode], sc->vicsek_noise,
                 total_boids, sc->paused ? "PAUSED " : "running", fps);
    } else {
        mvprintw(0, 0, "FLOCKING  mode:%-28s  boids:%-3d  %s  %.0ffps"
                       "  1-5:mode  +/-:boids  r:reset  spc:pause  q:quit",
                 k_mode_names[sc->mode], total_boids,
                 sc->paused ? "PAUSED " : "running", fps);
    }
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

    case '1':  s->mode = MODE_BOIDS;    break;
    case '2':  s->mode = MODE_CHASE;    break;
    case '3':  s->mode = MODE_VICSEK;   break;
    case '4':  s->mode = MODE_ORBIT;    break;
    case '5':  s->mode = MODE_PREDATOR; break;

    /* Vicsek noise adjustment — only meaningful in MODE_VICSEK */
    case 'n': case 'N':
        s->vicsek_noise -= VICSEK_NOISE_STEP;
        if (s->vicsek_noise < VICSEK_NOISE_MIN)
            s->vicsek_noise = VICSEK_NOISE_MIN;
        break;
    case 'm': case 'M':
        s->vicsek_noise += VICSEK_NOISE_STEP;
        if (s->vicsek_noise > VICSEK_NOISE_MAX)
            s->vicsek_noise = VICSEK_NOISE_MAX;
        break;

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
