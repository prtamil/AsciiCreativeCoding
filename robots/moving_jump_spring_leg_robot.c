/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * moving_jump_spring_leg_robot.c — Spring Leg Robot: left-to-right traversal
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  WHAT YOU SEE
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Robot spawns at the left edge and hops right across the full screen.
 *  Camera stays frozen while the robot traverses the visible area.
 *  Only when the robot approaches the right edge does the camera pan,
 *  revealing fresh terrain — this repeats infinitely.
 *
 *  Visual layers (front to back):
 *    '@'  on ground — spring loading, body sinks
 *    'O'  in flight — parabolic arc, trail traces the path
 *    '*'  on land   — impact flash
 *    coil (|()  chars) — spring leg, shrinks as it compresses
 *    PE bar (right margin) — fills quadratically while loading
 *    terrain — surface glyph + sub-surface fill; scrolls when camera pans
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  SPRING PHYSICS
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Hooke's Law:  F = −k·x
 *  Stored PE:    E = ½·k·x²          (quadratic — 2× compression → 4× energy)
 *  Launch speed: v = x·√(k/m)        (energy conservation; mass m=1 normalized)
 *
 *  With SPRING_K=25, COMPRESS_MAX=48px:
 *    v = 48·√25 = 240 px/s
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  TERRAIN-ADAPTIVE LAUNCH ANGLE
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Slope sampled by central finite difference at foot:
 *    s = atan2(floor_y(x+δ) − floor_y(x−δ),  2δ)
 *
 *  Effective launch angle:
 *    θ = clamp(BASE_ANGLE − s × SLOPE_SCALE,  25°, 85°)
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  CAMERA — RIGHT-EDGE SCROLL
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  Camera is frozen at world origin until the robot crosses CAM_TRIGGER
 *  (80% of screen width).  Past that point the camera exponentially
 *  chases the robot so it stays at 80% from the left.
 *
 *  This gives the full "walk across the screen" experience:
 *    1. Robot hops from left edge rightward — camera still.
 *    2. Robot crosses 80% — camera starts sliding right.
 *    3. New terrain scrolls in from the right — infinite loop.
 *
 *  cam_x = world x of the left screen edge
 *  robot screen x = body_px − cam_x
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  PHASE FSM
 * ═══════════════════════════════════════════════════════════════════════
 *
 *  COMPRESS ──(t ≥ T_COMPRESS)──► FLIGHT
 *   spring: 0 → MAX linearly           vy += g·dt  each tick
 *   body sinks (leg shortens)          trail records arc positions
 *   sample slope → θ_eff               detect floor contact
 *   PE bar fills                           │
 *                          ◄── LAND ◄──────┘
 *                           '*' flash  (T_LAND)
 *                           → COMPRESS
 *
 * Controls:
 *   q/ESC  quit       space/p  pause      r  reset
 *   f      cycle floor (flat ↔ perlin)    n  new terrain seed
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra robots/moving_jump_spring_leg_robot.c \
 *       -o moving_jump_spring_leg_robot -lncurses -lm
 *
 * ─────────────────────────────────────────────────────────────────────
 *  §1 config   §2 clock    §3 color    §4 coords
 *  §5 noise    §6 terrain  §7 trail    §8 robot
 *  §9 render   §10 screen  §11 app
 * ─────────────────────────────────────────────────────────────────────
 */

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

#define SIM_FPS     120
#define TARGET_FPS   60

/* Terminal cell aspect ratio — physics in pixel space, draw in cell space. */
#define CELL_W   8
#define CELL_H  16

/* Spring (pixel units, mass = 1 normalized).
 * v_launch = COMPRESS_MAX × √SPRING_K = 48 × 5 = 240 px/s */
#define SPRING_REST          80.0f
#define SPRING_COMPRESS_MAX  48.0f
#define SPRING_K             25.0f
#define BODY_HALF_H           8.0f

/* Base launch angle from horizontal [rad].
 * 50° balances arc height vs horizontal distance:
 *   vx = 240·cos50° ≈ 154 px/s  →  ~35 cols per jump at 80-col terminal.
 *   vy = 240·sin50° ≈ 184 px/s  →  arc peaks ~8 cells high (very visible).
 * At 50°, 2–3 jumps fill the screen before the camera has to pan. */
#define LAUNCH_ANGLE  (50.0f * (float)M_PI / 180.0f)

/* How much terrain slope deflects the launch angle.
 * 0.5 → 20° slope shifts angle by 10°. */
#define SLOPE_SCALE  0.5f
#define LAUNCH_MIN   (25.0f * (float)M_PI / 180.0f)
#define LAUNCH_MAX   (82.0f * (float)M_PI / 180.0f)

/* Gravity (px/s², downward = +y). */
#define GRAVITY  200.0f

#define T_COMPRESS  0.45f   /* s — spring loading time  */
#define T_LAND      0.14f   /* s — impact flash time    */

/* Arc trail: 1500 entries @ 120 Hz ≈ 12 s.  ~5 arcs visible at once. */
#define TRAIL_CAP   1500

/* Terrain noise parameters.
 * T_AMP = 112 px = 7 cells of vertical swing — large dramatic hills.
 * Three octaves of fBm:
 *   T_FREQ  (0.0028) → big hills every ~360px ≈ 45 cols   weight 0.50
 *   T_FREQ2 (0.014)  → medium bumps every ~70px ≈ 9 cols   weight 0.32
 *   T_FREQ3 (0.055)  → small roughness every ~18px ≈ 2 cols weight 0.18
 * Combined they give tall rolling hills with jagged edges. */
#define T_AMP    112.0f
#define T_FREQ   0.0028f
#define T_FREQ2  0.014f
#define T_FREQ3  0.055f
#define NOISE_N  512        /* value noise table size (power of 2) */

/* Speed levels cycled by 'a'.  Applied as a vx multiplier at launch.
 * Only horizontal speed scales — arc height stays the same so the jump
 * shape stays readable while the robot clearly covers more ground. */
#define SPEED_LEVELS  5
static const float SPEED_MULTS[SPEED_LEVELS] = { 1.0f, 1.5f, 2.0f, 2.5f, 3.0f };

/* Camera: right-edge-only scroll.
 *
 * CAM_TRIGGER: robot screen fraction [0,1] at which camera starts panning.
 *   0.80 → robot walks across 80% of screen before camera moves.
 *   Lower value → camera starts earlier (less traversal visible before pan).
 *
 * CAM_SPEED: exponential catch-up rate [1/s] once robot crosses the trigger.
 *   6.0 → time constant 1/6 ≈ 167 ms.  Camera slides smoothly, not snap.
 *
 * CAM_START_COL: screen column (cells) where robot spawns on reset.
 *   3 → robot starts 3 cells from left edge → full traversal visible. */
#define CAM_TRIGGER    0.80f
#define CAM_SPEED      6.0f
#define CAM_START_COL  3

/* Color pairs */
#define CP_BODY      1   /* '@' grounded         — white bold    */
#define CP_SPRING_L  2   /* spring low           — yellow        */
#define CP_SPRING_M  3   /* spring medium        — orange        */
#define CP_SPRING_H  4   /* spring loaded        — red bold      */
#define CP_FLIGHT    5   /* 'O' airborne         — cyan bold     */
#define CP_LAND      6   /* '*' impact           — magenta bold  */
#define CP_TRAIL     7   /* arc trail '.'        — blue          */
#define CP_TRAIL_O   8   /* old trail ':'        — dark blue dim */
#define CP_SURF      9   /* terrain surface      — green bold    */
#define CP_ROCK     10   /* terrain sub-surface  — green dim     */
#define CP_HUD      11   /* HUD text             — cyan          */
#define CP_ENERGY   12   /* PE bar '|'           — yellow        */
#define CP_FOOT     13   /* foot 'v'             — grey dim      */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

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
    struct timespec r = { (time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_BODY,     255, COLOR_BLACK);
        init_pair(CP_SPRING_L, 226, COLOR_BLACK);
        init_pair(CP_SPRING_M, 208, COLOR_BLACK);
        init_pair(CP_SPRING_H, 196, COLOR_BLACK);
        init_pair(CP_FLIGHT,    51, COLOR_BLACK);
        init_pair(CP_LAND,     201, COLOR_BLACK);
        init_pair(CP_TRAIL,     27, COLOR_BLACK);
        init_pair(CP_TRAIL_O,   17, COLOR_BLACK);
        init_pair(CP_SURF,      46, COLOR_BLACK);
        init_pair(CP_ROCK,      22, COLOR_BLACK);
        init_pair(CP_HUD,       51, COLOR_BLACK);
        init_pair(CP_ENERGY,   226, COLOR_BLACK);
        init_pair(CP_FOOT,     245, COLOR_BLACK);
    } else {
        init_pair(CP_BODY,     COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_SPRING_L, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_SPRING_M, COLOR_RED,     COLOR_BLACK);
        init_pair(CP_SPRING_H, COLOR_RED,     COLOR_BLACK);
        init_pair(CP_FLIGHT,   COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_LAND,     COLOR_MAGENTA, COLOR_BLACK);
        init_pair(CP_TRAIL,    COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_TRAIL_O,  COLOR_BLUE,    COLOR_BLACK);
        init_pair(CP_SURF,     COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_ROCK,     COLOR_GREEN,   COLOR_BLACK);
        init_pair(CP_HUD,      COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_ENERGY,   COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_FOOT,     COLOR_WHITE,   COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords                                                             */
/* ===================================================================== */

static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

/* Round-half-up — prevents flicker at cell boundaries. */
static inline int px_to_cx(float px) { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int px_to_cy(float py) { return (int)floorf(py / (float)CELL_H + 0.5f); }

/* ===================================================================== */
/* §5  noise — 1D value noise, cosine-interpolated                       */
/* ===================================================================== */

/*
 * Cosine interpolation: f(t) = a + (b−a)·(1−cos πt)/2
 * Gives C¹-continuous terrain (smooth hills, no kinks at lattice points).
 * Two-octave fBm: slow large hills (0.70 weight) + fast bumps (0.30 weight).
 */
static float g_noise[NOISE_N];

static void noise_init(unsigned seed)
{
    srand(seed);
    for (int i = 0; i < NOISE_N; i++)
        g_noise[i] = (float)rand() / (float)RAND_MAX;
}

static float noise1d(float x)
{
    int   xi = (int)floorf(x) & (NOISE_N - 1);
    float xf = x - floorf(x);
    float a  = g_noise[xi];
    float b  = g_noise[(xi + 1) & (NOISE_N - 1)];
    float f  = (1.0f - cosf(xf * (float)M_PI)) * 0.5f;
    return a * (1.0f - f) + b * f;
}

static float terrain_noise(float world_x)
{
    return noise1d(world_x * T_FREQ)  * 0.50f
         + noise1d(world_x * T_FREQ2) * 0.32f
         + noise1d(world_x * T_FREQ3) * 0.18f;
}

/* ===================================================================== */
/* §6  terrain — height query, slope, surface glyph                      */
/* ===================================================================== */

typedef enum { FLOOR_FLAT = 0, FLOOR_PERLIN, FLOOR_COUNT } FloorMode;
static const char *FLOOR_NAMES[] = { "FLAT  ", "PERLIN" };

/*
 * floor_y_at — world y of terrain surface at world_x.
 *
 * FLAT:   constant = base_y.
 * PERLIN: noise centred at 0.5, scaled ±T_AMP around base_y.
 */
static float floor_y_at(float world_x, FloorMode mode, float base_y)
{
    if (mode == FLOOR_FLAT) return base_y;
    return base_y + (terrain_noise(world_x) - 0.5f) * 2.0f * T_AMP;
}

/*
 * floor_slope — dy/dx slope at world_x (central finite difference).
 * Returns atan2(dy, dx) in radians.
 */
static float floor_slope(float world_x, FloorMode mode, float base_y)
{
    float dx  = (float)(CELL_W * 2);
    float dy  = floor_y_at(world_x + dx, mode, base_y)
              - floor_y_at(world_x - dx, mode, base_y);
    return atan2f(dy, 2.0f * dx);
}

/*
 * surface_glyph — char for the terrain surface based on slope direction.
 *   dh < −threshold → ascending right  → '/'
 *   dh >  threshold → descending right → '\'
 *   else             → '_'
 */
static chtype surface_glyph(float dh)
{
    if (dh < -CELL_H * 0.20f) return '/';
    if (dh >  CELL_H * 0.20f) return '\\';
    return '_';
}

/* ===================================================================== */
/* §7  trail — arc history ring buffer                                   */
/* ===================================================================== */

typedef struct {
    float wx[TRAIL_CAP];   /* world x */
    float wy[TRAIL_CAP];   /* world y */
    int   head, count;
} Trail;

static void trail_push(Trail *t, float wx, float wy)
{
    t->wx[t->head] = wx;
    t->wy[t->head] = wy;
    t->head = (t->head + 1) % TRAIL_CAP;
    if (t->count < TRAIL_CAP) t->count++;
}

/* ===================================================================== */
/* §8  robot — state, FSM, physics                                       */
/* ===================================================================== */

typedef enum {
    PHASE_COMPRESS = 0,
    PHASE_FLIGHT,
    PHASE_LAND,
} Phase;

static const char *PHASE_NAMES[] = { "LOAD", "FLY ", "LAND" };

/*
 * Robot — full world-space state.
 *
 * body_px/py:      world position of body centre.
 * prev_body_px/py: position one tick ago (sub-tick render interpolation).
 * foot_px/py:      ground contact point (fixed during COMPRESS/LAND).
 * cam_x:           world x of screen left edge — updated every tick.
 * slope_angle:     terrain slope at foot [rad].
 * eff_angle:       clamped slope-adjusted launch angle for this jump.
 * base_y:          flat-floor y and perlin terrain baseline.
 */
typedef struct {
    float body_px, body_py;
    float prev_body_px, prev_body_py;
    float foot_px, foot_py;
    float vx, vy;
    float spring_compress;
    Phase phase;
    float phase_t;
    int   launch_count;
    Trail trail;
    bool  paused;
    FloorMode floor_mode;
    float base_y;
    float cam_x;
    float slope_angle;
    float eff_angle;
    int   speed_level;  /* index into SPEED_MULTS; cycled by 'a' */
} Robot;

static inline float spring_energy(float x)
{
    return 0.5f * SPRING_K * x * x;
}

/*
 * effective_launch_angle — slope-adjusted clamped angle.
 * Uphill  (slope < 0): θ increases → steeper, clears the rise.
 * Downhill (slope > 0): θ decreases → shallower, rides the descent.
 */
static float effective_launch_angle(float slope)
{
    float a = LAUNCH_ANGLE - slope * SLOPE_SCALE;
    if (a < LAUNCH_MIN) a = LAUNCH_MIN;
    if (a > LAUNCH_MAX) a = LAUNCH_MAX;
    return a;
}

/*
 * robot_ground_pose — body position derived from foot + spring compression.
 * leg_len = SPRING_REST − spring_compress
 * body_py = foot_py − leg_len − BODY_HALF_H
 */
static void robot_ground_pose(Robot *r)
{
    r->body_py = r->foot_py - (SPRING_REST - r->spring_compress) - BODY_HALF_H;
    r->body_px = r->foot_px;
}

/*
 * cam_update — right-edge-only camera scroll.
 *
 * Robot screen position = body_px − cam_x.
 * The camera stays frozen at its current world position until the robot
 * crosses CAM_TRIGGER × screen_width_px from the left edge.
 *
 * Once triggered, the camera exponentially chases a target that keeps
 * the robot exactly at the trigger column — this prevents the robot
 * from running off-screen while still giving smooth panning.
 *
 * cam_x is never allowed to go negative (can't scroll left of origin).
 */
static void cam_update(Robot *r, float dt, int cols)
{
    float scr_w   = (float)pw(cols);
    float bot_sx  = r->body_px - r->cam_x;         /* robot x in screen px */
    float trigger = scr_w * CAM_TRIGGER;

    if (bot_sx > trigger) {
        /* Keep robot pinned at the trigger column as camera chases */
        float target = r->body_px - trigger;
        r->cam_x += (target - r->cam_x) * CAM_SPEED * dt;
    }

    if (r->cam_x < 0.0f) r->cam_x = 0.0f;
}

static void robot_init(Robot *r, int cols, int rows)
{
    FloorMode fm = r->floor_mode;
    int       sl = r->speed_level;
    memset(r, 0, sizeof *r);
    r->floor_mode  = fm;
    r->speed_level = sl;
    r->base_y     = (float)ph(rows) * 0.72f;
    r->cam_x      = 0.0f;

    /* Spawn at CAM_START_COL cells from the left — robot traverses full screen
     * before the camera ever needs to pan. */
    r->foot_px     = (float)(CAM_START_COL * CELL_W);
    r->foot_py     = floor_y_at(r->foot_px, r->floor_mode, r->base_y);
    r->slope_angle = floor_slope(r->foot_px, r->floor_mode, r->base_y);
    r->eff_angle   = effective_launch_angle(r->slope_angle);
    r->phase       = PHASE_COMPRESS;
    robot_ground_pose(r);
    r->prev_body_px = r->body_px;
    r->prev_body_py = r->body_py;
    (void)cols;
}

/*
 * robot_tick — one fixed-step physics tick (dt seconds).
 *
 * ── COMPRESS ────────────────────────────────────────────────────────
 *  spring_compress ramps 0 → MAX over T_COMPRESS seconds.
 *  At t ≥ T_COMPRESS: v = compress × √K,  vx = v·cosθ,  vy = −v·sinθ
 *
 * ── FLIGHT ──────────────────────────────────────────────────────────
 *  Explicit Euler (exact for constant gravity).
 *  Landing when foot_proj ≥ floor_y at robot x.
 *
 * ── LAND ────────────────────────────────────────────────────────────
 *  Body locked at landing pose.  phase_t counts to T_LAND → COMPRESS.
 */
static void robot_tick(Robot *r, float dt, int cols)
{
    r->prev_body_px = r->body_px;
    r->prev_body_py = r->body_py;
    r->phase_t += dt;

    cam_update(r, dt, cols);

    switch (r->phase) {

    case PHASE_COMPRESS: {
        float prog = r->phase_t / T_COMPRESS;
        if (prog > 1.0f) prog = 1.0f;
        r->spring_compress = SPRING_COMPRESS_MAX * prog;
        robot_ground_pose(r);

        r->slope_angle = floor_slope(r->foot_px, r->floor_mode, r->base_y);
        r->eff_angle   = effective_launch_angle(r->slope_angle);

        if (r->phase_t >= T_COMPRESS) {
            float v    = r->spring_compress * sqrtf(SPRING_K);
            float smul = SPEED_MULTS[r->speed_level];
            r->vx = v * cosf(r->eff_angle) * smul;
            r->vy = -v * sinf(r->eff_angle);
            r->spring_compress = 0.0f;
            r->phase   = PHASE_FLIGHT;
            r->phase_t = 0.0f;
            r->launch_count++;
        }
        break;
    }

    case PHASE_FLIGHT: {
        r->vy      += GRAVITY * dt;
        r->body_px += r->vx * dt;
        r->body_py += r->vy * dt;

        trail_push(&r->trail, r->body_px, r->body_py);

        float foot_proj  = r->body_py + BODY_HALF_H + SPRING_REST;
        float floor_here = floor_y_at(r->body_px, r->floor_mode, r->base_y);
        if (foot_proj >= floor_here) {
            r->foot_px = r->body_px;
            r->foot_py = floor_here;
            r->spring_compress = 0.0f;
            robot_ground_pose(r);
            r->vx = r->vy = 0.0f;
            r->phase   = PHASE_LAND;
            r->phase_t = 0.0f;
        }
        break;
    }

    case PHASE_LAND: {
        robot_ground_pose(r);
        if (r->phase_t >= T_LAND) {
            r->phase   = PHASE_COMPRESS;
            r->phase_t = 0.0f;
        }
        break;
    }
    }
}

/* ===================================================================== */
/* §9  render                                                             */
/* ===================================================================== */

/* scr_cx — convert world x to screen column via camera offset. */
static inline int scr_cx(float world_px, float cam_x)
{
    return px_to_cx(world_px - cam_x);
}

/*
 * render_terrain — draw scrolling floor with solid sub-surface fill.
 *
 * For each screen column sc (0..cols-1):
 *   world_x = cam_x + sc × CELL_W
 *   surf_row = px_to_cy(floor_y_at(world_x))
 *
 *   Row  surf_row:     surface glyph (_  /  \) — bold green
 *   Row  surf_row+1:   texture row (.: alternating) — dim green
 *   Rows surf_row+2…:  solid fill (#) — dark green dim
 */
static void render_terrain(FloorMode mode, float base_y, float cam_x,
                           int cols, int rows)
{
    for (int sc = 0; sc < cols; sc++) {
        float wx      = cam_x + (float)(sc * CELL_W);
        float fy      = floor_y_at(wx, mode, base_y);
        float fy_next = floor_y_at(wx + (float)CELL_W, mode, base_y);
        int   surf    = px_to_cy(fy);
        if (surf < 1)        surf = 1;
        if (surf > rows - 2) surf = rows - 2;

        chtype sg = (mode == FLOOR_PERLIN)
                  ? surface_glyph(fy_next - fy)
                  : '_';

        attron(COLOR_PAIR(CP_SURF) | A_BOLD);
        mvaddch(surf, sc, sg);
        attroff(COLOR_PAIR(CP_SURF) | A_BOLD);

        if (surf + 1 < rows - 1) {
            attron(COLOR_PAIR(CP_SURF) | A_DIM);
            mvaddch(surf + 1, sc, (sc % 2 == 0) ? ':' : '.');
            attroff(COLOR_PAIR(CP_SURF) | A_DIM);
        }

        attron(COLOR_PAIR(CP_ROCK) | A_DIM);
        for (int r = surf + 2; r < rows - 1; r++)
            mvaddch(r, sc, (sc % 2 == 0) ? '#' : ' ');
        attroff(COLOR_PAIR(CP_ROCK) | A_DIM);
    }
}

/*
 * render_trail — arc history, newest brightest, fading leftward.
 *
 * Trail stores world positions; subtract cam_x to get screen column.
 * Old entries that scroll off the left edge (cx < 0) are skipped silently.
 *
 * Age encoding (k=0 is newest):
 *   0–15%  A_BOLD  '.'  CP_FLIGHT  — freshest
 *   15–40% A_NORMAL '.' CP_TRAIL   — recent
 *   40%+   A_DIM   ':'  CP_TRAIL_O — fading
 */
static void render_trail(const Robot *r, float cam_x, int cols, int rows)
{
    for (int k = 0; k < r->trail.count; k++) {
        int   idx = ((r->trail.head - 1 - k) + TRAIL_CAP) % TRAIL_CAP;
        int   cx  = scr_cx(r->trail.wx[idx], cam_x);
        int   cy  = px_to_cy(r->trail.wy[idx]);
        if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1) continue;

        float age = (r->trail.count > 1)
                  ? (float)k / (float)(r->trail.count - 1) : 0.0f;

        int    cp = (age < 0.40f) ? CP_TRAIL   : CP_TRAIL_O;
        attr_t at = (age < 0.15f) ? A_BOLD
                  : (age < 0.40f) ? A_NORMAL   : A_DIM;
        chtype ch = (age < 0.40f) ? '.'        : ':';

        attron(COLOR_PAIR(cp) | at);
        mvaddch(cy, cx, ch);
        attroff(COLOR_PAIR(cp) | at);
    }
}

/*
 * render_spring — compressed spring leg as a coil column.
 *
 * Rows span from body bottom to foot contact.
 * Shrinking row count IS the compression — no extra indicator needed.
 * Coil pattern { '(' '|' ')' '|' } cycles per row.
 * Color: yellow → orange → red as compression increases.
 */
static void render_spring(const Robot *r, float draw_body_py,
                          float cam_x, int cols, int rows)
{
    int cx       = scr_cx(r->body_px, cam_x);
    int body_bot = px_to_cy(draw_body_py + BODY_HALF_H);
    int foot_cy  = px_to_cy(r->foot_py);

    if (cx < 0 || cx >= cols) return;

    float ratio = r->spring_compress / SPRING_COMPRESS_MAX;
    int   cp    = (ratio < 0.30f) ? CP_SPRING_L
                : (ratio < 0.70f) ? CP_SPRING_M : CP_SPRING_H;
    attr_t at   = (ratio > 0.70f) ? A_BOLD : A_NORMAL;

    static const chtype coil[4] = { '(', '|', ')', '|' };
    for (int row = body_bot; row < foot_cy; row++) {
        if (row < 1 || row >= rows - 1) continue;
        attron(COLOR_PAIR(cp) | at);
        mvaddch(row, cx, coil[(row - body_bot) & 3]);
        attroff(COLOR_PAIR(cp) | at);
    }

    if (foot_cy >= 1 && foot_cy < rows - 1) {
        attron(COLOR_PAIR(CP_FOOT) | A_DIM);
        mvaddch(foot_cy, cx, 'v');
        attroff(COLOR_PAIR(CP_FOOT) | A_DIM);
    }
}

/*
 * render_body — phase-coded body glyph at interpolated position.
 *   '@' planted  — grounded
 *   'O' airborne — in flight
 *   '*' impact   — landing flash
 */
static void render_body(const Robot *r, float dpx, float dpy,
                        float cam_x, int cols, int rows)
{
    int cx = scr_cx(dpx, cam_x);
    int cy = px_to_cy(dpy);
    if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1) return;

    chtype ch; int cp; attr_t at;
    switch (r->phase) {
    case PHASE_COMPRESS: ch = '@'; cp = CP_BODY;   at = A_BOLD; break;
    case PHASE_FLIGHT:   ch = 'O'; cp = CP_FLIGHT; at = A_BOLD; break;
    default:             ch = '*'; cp = CP_LAND;   at = A_BOLD; break;
    }
    attron(COLOR_PAIR(cp) | at);
    mvaddch(cy, cx, ch);
    attroff(COLOR_PAIR(cp) | at);
}

/*
 * render_energy_bar — vertical PE gauge, right margin, COMPRESS only.
 * Height ∝ PE/PE_max = (x/MAX)² — quadratic fill shows non-linear storage.
 */
static void render_energy_bar(const Robot *r, int cols, int rows)
{
    if (r->phase != PHASE_COMPRESS) return;

    int bar_x   = cols - 3;
    int bar_bot = rows - 2;
    int bar_h   = (rows > 12) ? rows - 10 : 4;
    int bar_top = bar_bot - bar_h;
    if (bar_x < 0 || bar_top < 1) return;

    float ratio  = spring_energy(r->spring_compress)
                 / spring_energy(SPRING_COMPRESS_MAX);
    int   filled = (int)(ratio * (float)bar_h + 0.5f);

    if (bar_top - 1 >= 1) {
        attron(COLOR_PAIR(CP_HUD) | A_DIM);
        mvprintw(bar_top - 1, bar_x, "PE");
        attroff(COLOR_PAIR(CP_HUD) | A_DIM);
    }

    for (int i = 0; i < bar_h; i++) {
        int row = bar_bot - i;
        if (row < 1 || row >= rows - 1) continue;
        if (i < filled) {
            int    fcp = (ratio > 0.80f) ? CP_SPRING_H : CP_ENERGY;
            chtype fc  = (ratio > 0.80f && i == filled - 1) ? '!' : '|';
            attron(COLOR_PAIR(fcp) | A_BOLD);
            mvaddch(row, bar_x,     fc);
            mvaddch(row, bar_x + 1, fc);
            attroff(COLOR_PAIR(fcp) | A_BOLD);
        } else {
            attron(COLOR_PAIR(CP_HUD) | A_DIM);
            mvaddch(row, bar_x,     '.');
            mvaddch(row, bar_x + 1, '.');
            attroff(COLOR_PAIR(CP_HUD) | A_DIM);
        }
    }
}

/*
 * render_hud — one-line telemetry + key reference.
 *
 * robot_scr: robot screen column — shows how far robot is from left edge.
 * cam_world: how far the camera has scrolled — distance traversed total.
 */
static void render_hud(const Robot *r, int cols, int rows, double fps)
{
    int robot_scr = scr_cx(r->body_px, r->cam_x);

    char buf[256];
    snprintf(buf, sizeof buf,
        " [%s] %-6s  compress:%3.0f/%3.0fpx"
        "  PE:%5.0f  slope:%+5.1f°  launch:%4.1f°"
        "  spd:%.1fx  pos:col%-3d  cam:%5.0fpx  jumps:%-3d  %4.0ffps ",
        FLOOR_NAMES[r->floor_mode],
        PHASE_NAMES[r->phase],
        r->spring_compress, SPRING_COMPRESS_MAX,
        spring_energy(r->spring_compress),
        r->slope_angle * (180.0f / (float)M_PI),
        r->eff_angle   * (180.0f / (float)M_PI),
        SPEED_MULTS[r->speed_level],
        robot_scr,
        r->cam_x,
        r->launch_count, fps);

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvaddnstr(0, 0, buf, cols);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    attron(COLOR_PAIR(CP_HUD) | A_DIM);
    mvprintw(rows - 1, 0,
        " q:quit  spc:pause  r:reset  f:floor  n:new-terrain  a:speed(%.1fx) ",
        SPEED_MULTS[r->speed_level]);
    attroff(COLOR_PAIR(CP_HUD) | A_DIM);

    if (r->paused) {
        attron(COLOR_PAIR(CP_LAND) | A_BOLD);
        int mx = cols / 2 - 4;
        if (mx >= 0) mvprintw(rows / 2, mx, " PAUSED ");
        attroff(COLOR_PAIR(CP_LAND) | A_BOLD);
    }
}

/*
 * scene_draw — compose complete frame.
 *
 * Order: trail → terrain → spring → body → PE bar → HUD
 * cam_x snapshot: one value used across all layers this frame so no
 * layer gets a different camera offset (prevents sub-pixel tearing).
 *
 * Render interpolation:
 *   dpx/dpy = prev + (cur − prev) × alpha
 *   Removes stutter when sim (120 Hz) and render (60 Hz) rates differ.
 */
static void scene_draw(const Robot *r, int cols, int rows,
                       float alpha, double fps)
{
    erase();

    float cam_x = r->cam_x;

    float dpx = r->prev_body_px + (r->body_px - r->prev_body_px) * alpha;
    float dpy = r->prev_body_py + (r->body_py - r->prev_body_py) * alpha;

    render_trail(r, cam_x, cols, rows);
    render_terrain(r->floor_mode, r->base_y, cam_x, cols, rows);

    if (r->phase == PHASE_COMPRESS || r->phase == PHASE_LAND)
        render_spring(r, dpy, cam_x, cols, rows);

    render_body(r, dpx, dpy, cam_x, cols, rows);
    render_energy_bar(r, cols, rows);
    render_hud(r, cols, rows, fps);
}

/* ===================================================================== */
/* §10  screen                                                            */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak();
    curs_set(0); nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* ===================================================================== */
/* §11  app                                                               */
/* ===================================================================== */

typedef struct {
    Robot                 robot;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit(int s)   { (void)s; g_app.running    = 0; }
static void on_resize(int s) { (void)s; g_app.need_resize = 1; }
static void cleanup(void)    { endwin(); }

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit);
    signal(SIGTERM,  on_exit);
    signal(SIGWINCH, on_resize);

    noise_init((unsigned)(clock_ns() & 0xFFFFFFFF));

    App *app     = &g_app;
    app->running = 1;
    app->robot.floor_mode = FLOOR_PERLIN;

    screen_init(&app->screen);
    robot_init(&app->robot, app->screen.cols, app->screen.rows);

    int64_t tick_ns    = NS_PER_SEC / SIM_FPS;
    float   dt_sec     = 1.0f / (float)SIM_FPS;
    int64_t frame_time = clock_ns();
    int64_t sim_accum  = 0;
    int64_t fps_accum  = 0;
    int     fps_count  = 0;
    double  fps_disp   = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────────────── */
        if (app->need_resize) {
            screen_resize(&app->screen);
            app->robot.base_y = (float)ph(app->screen.rows) * 0.72f;
            app->need_resize  = 0;
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── dt ──────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── input ───────────────────────────────────────── */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27:
                app->running = 0; break;
            case ' ': case 'p': case 'P':
                app->robot.paused = !app->robot.paused; break;
            case 'r': case 'R': {
                FloorMode fm = app->robot.floor_mode;
                float     by = app->robot.base_y;
                memset(&app->robot, 0, sizeof app->robot);
                app->robot.floor_mode = fm;
                app->robot.base_y     = by;
                robot_init(&app->robot, app->screen.cols, app->screen.rows);
                break;
            }
            case 'f': case 'F':
                app->robot.floor_mode =
                    (FloorMode)((app->robot.floor_mode + 1) % FLOOR_COUNT);
                break;
            case 'n': case 'N':
                noise_init((unsigned)clock_ns()); break;
            case 'a': case 'A':
                app->robot.speed_level =
                    (app->robot.speed_level + 1) % SPEED_LEVELS;
                break;
            default: break;
            }
        }

        /* ── fixed-step physics ──────────────────────────── */
        if (!app->robot.paused) {
            sim_accum += dt;
            while (sim_accum >= tick_ns) {
                robot_tick(&app->robot, dt_sec, app->screen.cols);
                sim_accum -= tick_ns;
            }
        }

        /* ── sub-tick render alpha ───────────────────────── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── fps counter (500 ms window) ─────────────────── */
        fps_count++;
        fps_accum += dt;
        if (fps_accum >= 500 * NS_PER_MS) {
            fps_disp  = (double)fps_count
                      / ((double)fps_accum / (double)NS_PER_SEC);
            fps_count = 0;
            fps_accum = 0;
        }

        /* ── sleep to cap at 60 fps ──────────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        /* ── render ──────────────────────────────────────── */
        scene_draw(&app->robot, app->screen.cols, app->screen.rows,
                   alpha, fps_disp);
        wnoutrefresh(stdscr);
        doupdate();
    }

    screen_free(&app->screen);
    return 0;
}
