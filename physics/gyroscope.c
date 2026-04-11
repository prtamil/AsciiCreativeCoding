/*
 * gyroscope.c — Spinning Top / Euler Equations
 *
 * Rigid-body rotation via Euler's equations integrated with RK4.
 * Orientation stored as a unit quaternion — no gimbal lock, no drift.
 * Gram-Schmidt re-orthogonalises the extracted rotation axes each frame
 * as an additional numerical safeguard.
 *
 * Three presets:
 *   0  Euler's Top   — torque-free symmetric top; body Z traces a cone
 *                       around the fixed angular momentum vector
 *   1  Gravity Top   — gravity-driven precession + nutation; the wobble
 *                       tightens as spin rate increases
 *   2  Dzhanibekov   — asymmetric torque-free body; rotation near the
 *                       intermediate inertia axis is unstable → periodic
 *                       180° flips (tennis-racket / T-handle effect)
 *
 * Framework: follows framework.c §1–§8 skeleton.
 *
 * ─────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────
 *   §1  config   — presets, constants
 *   §2  clock    — monotonic ns clock + sleep
 *   §3  color    — pairs per axis / element
 *   §4  coords   — CELL_W/H aspect correction; 3-D→2-D projection
 *   §5  entity   — Gyro: RK4 integrator, quaternion, Gram-Schmidt, draw
 *   §6  scene
 *   §7  screen
 *   §8  app
 * ─────────────────────────────────────────────────────────────────────
 *
 * PHYSICS SUMMARY
 * ─────────────────────────────────────────────────────────────────────
 * State vector (7 floats):  s = [ωx, ωy, ωz, qw, qx, qy, qz]
 *
 * Euler's equations (body frame, with optional gravity torque τ):
 *   I₁ω̇x = (I₂−I₃)ωy·ωz + τx
 *   I₂ω̇y = (I₃−I₁)ωz·ωx + τy
 *   I₃ω̇z = (I₁−I₂)ωx·ωy + τz
 *
 * Gravity torque on a top pivoting at origin, CM at l·ez (body Z):
 *   τ_body = mgl · (gz_by, −gz_bx, 0)
 *   where gz_b = R^T · Ẑworld = world-Z unit vector expressed in
 *   the body frame, read from the rotation matrix derived from q.
 *
 * Quaternion kinematics (ω in body frame):
 *   q̇ = ½ · q ⊗ (0, ωx, ωy, ωz)
 *   → dqw = ½(−qx·ωx − qy·ωy − qz·ωz)
 *      dqx = ½( qw·ωx + qy·ωz − qz·ωy)
 *      dqy = ½( qw·ωy − qx·ωz + qz·ωx)
 *      dqz = ½( qw·ωz + qx·ωy − qy·ωx)
 *
 * After each RK4 step: normalise q to unit length (prevents drift).
 *
 * SO(3) DRIFT PREVENTION
 * ─────────────────────────────────────────────────────────────────────
 * Quaternion normalisation keeps |q|=1 → R stays in SO(3) exactly.
 * Additionally, when extracting the body-axis triad (ex, ey, ez) for
 * rendering, Gram-Schmidt is applied so floating-point errors in the
 * quaternion formula cannot accumulate into non-orthogonal axes.
 *
 * PROJECTION
 * ─────────────────────────────────────────────────────────────────────
 * Orthographic projection with azimuth φ and elevation θ:
 *   rx =  wx·cos φ + wy·sin φ          (horizontal after azimuth rotation)
 *   ry = −wx·sin φ + wy·cos φ          (depth    after azimuth rotation)
 *   screen_x =  rx
 *   screen_y =  ry·cos θ + wz·sin θ    (up on screen)
 *   depth    = −ry·sin θ + wz·cos θ    (depth, used for shading)
 * Terminal column = cx + screen_x · scale
 * Terminal row    = cy − screen_y · scale · ASPECT   (ASPECT≈0.5)
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume
 *   n / p        next / previous preset
 *   g            toggle gravity (presets 0 and 2)
 *   t            toggle polhode trail
 *   ← →          rotate view azimuth
 *   ↑ ↓          tilt view elevation
 *   r            restart preset
 *   ] / [        raise / lower sim Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra gyroscope.c -o gyroscope -lncurses -lm
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
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN      = 20,
    SIM_FPS_DEFAULT  = 60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    SUB_STEPS        =   8,   /* RK4 sub-steps per sim tick (stability) */
    TRAIL_LEN        = 300,   /* polhode trail ring-buffer length        */
    DISC_PTS         =  32,   /* points on the body-disc equator         */
    GROUND_PTS       =  48,   /* points on the ground reference ring     */
    FPS_UPDATE_MS    = 500,
    N_COLORS         =   7,
    N_PRESETS        =   3,
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ── Preset table ────────────────────────────────────────────────────── */

typedef struct {
    const char *name;
    float I1, I2, I3;     /* principal moments of inertia                */
    float omega[3];        /* initial angular velocity, body frame (rad/s)*/
    float tilt_deg;        /* initial nutation: tilt body-Z from world-Z  */
    float tilt_axis[3];    /* world axis to tilt around (normalised)      */
    bool  gravity;         /* apply gravity torque                        */
    float mgl;             /* mg·l (gravity torque scale, N·m)            */
} GPreset;

/*
 * Preset 0 — Euler's Top (symmetric, torque-free)
 *   I1=I2=2 (oblate "saucer"), I3=1.
 *   ωz=8 rad/s (fast spin); small ωx seeds the initial wobble.
 *   L is conserved; body-Z precesses at Euler freq ≈ (I1-I3)/I1·ωz = 4 rad/s.
 *
 * Preset 1 — Gravity Top (symmetric, with gravity)
 *   Same inertia.  ωz=12 rad/s; mgl=1.5.
 *   Precession rate ≈ mgl/(I3·ωz) ≈ 0.125 rad/s (slow circle).
 *   Nutation freq   ≈ I3·ωz/I1 = 6 rad/s (fast wobble).
 *
 * Preset 2 — Dzhanibekov / Tennis-Racket
 *   I1<I2<I3; rotation near INTERMEDIATE axis (I2) is unstable.
 *   Flip period T ≈ 2π / (ωy·√((I2-I1)(I3-I2)/(I1·I3))) ≈ 1.1 s.
 *   ~1 dramatic 180° flip per second.
 */
static const GPreset PRESETS[N_PRESETS] = {
    {   /* Euler's Top */
        "Euler\\'s Top   (torque-free, symmetric)",
        2.0f, 2.0f, 1.0f,
        { 0.8f, 0.0f, 8.0f },
        18.0f, { 1.0f, 0.0f, 0.0f },
        false, 0.0f
    },
    {   /* Gravity Top */
        "Gravity Top    (precession + nutation)",
        2.0f, 2.0f, 1.0f,
        { 0.0f, 0.0f, 12.0f },
        25.0f, { 1.0f, 0.0f, 0.0f },
        true, 1.5f
    },
    {   /* Dzhanibekov effect */
        "Dzhanibekov    (intermediate-axis flip)",
        1.0f, 2.5f, 3.5f,
        { 0.05f, 8.0f, 0.05f },
        10.0f, { 0.0f, 0.0f, 1.0f },
        false, 0.0f
    },
};

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

/* Pair assignments:
 *   1  red     — body X axis
 *   2  green   — body Y axis
 *   3  cyan    — body Z axis (spin axis)
 *   4  yellow  — angular momentum L
 *   5  dim     — ground ring / world-Z reference
 *   6  magenta — polhode trail
 *   7  blue    — disc equator
 *   8  yellow  — HUD text
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(1, 196, COLOR_BLACK);   /* red          body X  */
        init_pair(2,  46, COLOR_BLACK);   /* green        body Y  */
        init_pair(3,  51, COLOR_BLACK);   /* cyan         body Z  */
        init_pair(4, 226, COLOR_BLACK);   /* yellow       L       */
        init_pair(5, 240, COLOR_BLACK);   /* dark grey    ground  */
        init_pair(6, 201, COLOR_BLACK);   /* magenta      trail   */
        init_pair(7,  39, COLOR_BLACK);   /* blue         disc    */
        init_pair(8, 226, COLOR_BLACK);   /* yellow bold  HUD     */
    } else {
        init_pair(1, COLOR_RED,     COLOR_BLACK);
        init_pair(2, COLOR_GREEN,   COLOR_BLACK);
        init_pair(3, COLOR_CYAN,    COLOR_BLACK);
        init_pair(4, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(5, COLOR_WHITE,   COLOR_BLACK);
        init_pair(6, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(7, COLOR_BLUE,    COLOR_BLACK);
        init_pair(8, COLOR_YELLOW,  COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords — aspect correction + 3-D orthographic projection          */
/* ===================================================================== */

/*
 * ASPECT CORRECTION
 * Terminal cells are ~2× taller than wide (CELL_H/CELL_W ≈ 2).
 * When converting a 3-D screen-Y coordinate to a terminal row, we
 * multiply by ASPECT = CELL_W/CELL_H ≈ 0.5 so that unit-length axes
 * appear equal length in all directions.  Only applied at draw time.
 *
 * PROJECTION
 * Two sequential rotations applied to world coordinates:
 *   1. Azimuth φ: rotate around world Z (panning left/right).
 *   2. Elevation θ: tilt the scene so the viewer is above the horizon.
 * Depth (sz) is used for depth-cueing (A_BOLD for near, A_DIM for far).
 */
#define CELL_W   8
#define CELL_H  16
#define ASPECT   ((float)CELL_W / (float)CELL_H)   /* ≈ 0.5 */

static void project(float wx, float wy, float wz,
                    float phi, float theta, float scale,
                    int cx, int cy,
                    int *col, int *row, float *depth)
{
    float rx =  wx * cosf(phi) + wy * sinf(phi);
    float ry = -wx * sinf(phi) + wy * cosf(phi);

    float sx  = rx;
    float sy  = ry * cosf(theta) + wz * sinf(theta);
    *depth    = -ry * sinf(theta) + wz * cosf(theta);

    *col = cx + (int)roundf(sx * scale);
    *row = cy - (int)roundf(sy * scale * ASPECT);
}

/* Direction-aware character for a 2-D segment angle */
static char dir_char(float angle)
{
    float a = fmodf(angle, (float)M_PI);
    if (a < 0.0f) a += (float)M_PI;
    if (a < (float)M_PI / 8.0f || a >= 7.0f * (float)M_PI / 8.0f) return '-';
    if (a < 3.0f * (float)M_PI / 8.0f) return '/';
    if (a < 5.0f * (float)M_PI / 8.0f) return '|';
    return '\\';
}

/* Draw a 3-D line segment (from origin to endpoint) using DDA */
static void draw_seg3d(WINDOW *w,
                       float ox3, float oy3, float oz3,
                       float ex3, float ey3, float ez3,
                       float phi, float theta, float scale,
                       int cx, int cy, int cols, int rows,
                       chtype attr)
{
    int c0, r0; float d0;
    int c1, r1; float d1;
    project(ox3, oy3, oz3, phi, theta, scale, cx, cy, &c0, &r0, &d0);
    project(ex3, ey3, ez3, phi, theta, scale, cx, cy, &c1, &r1, &d1);

    float ang = atan2f((float)(r1 - r0), (float)(c1 - c0));
    char  ch  = dir_char(ang);

    int dx = c1 - c0, dy = r1 - r0;
    int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);
    if (steps < 1) steps = 1;

    for (int i = 0; i <= steps; i++) {
        float t  = (float)i / (float)steps;
        int   c  = c0 + (int)roundf(dx * t);
        int   r  = r0 + (int)roundf(dy * t);
        if (c >= 0 && c < cols && r >= 1 && r < rows - 1) {
            wattron(w, attr);
            mvwaddch(w, r, c, (chtype)(unsigned char)ch);
            wattroff(w, attr);
        }
    }
}

/* ===================================================================== */
/* §5  entity — Gyro                                                      */
/* ===================================================================== */

/* ── RK4 state vector [ωx, ωy, ωz, qw, qx, qy, qz] ─────────────────── */

typedef struct { float v[7]; } State7;

static State7 s7_add(State7 a, State7 b)
{
    State7 r;
    for (int i = 0; i < 7; i++) r.v[i] = a.v[i] + b.v[i];
    return r;
}
static State7 s7_scale(State7 a, float s)
{
    State7 r;
    for (int i = 0; i < 7; i++) r.v[i] = a.v[i] * s;
    return r;
}

/*
 * gyro_deriv() — time derivative of the 7-component state.
 *
 * Input:  s = [ωx, ωy, ωz, qw, qx, qy, qz]
 * Output: ṡ = [ω̇x, ω̇y, ω̇z, q̇w, q̇x, q̇y, q̇z]
 *
 * Gravity torque derivation:
 *   The CM sits at l·ez in world frame (ez = body-Z axis in world).
 *   In body frame, world-Z expressed as: gz_b = R^T · Ẑ = (ex[2], ey[2], ez[2])
 *   where ex[2] etc. are the Z-components of the body axes extracted
 *   from the quaternion (third row of R).
 *   Torque = (0,0,l) × m·g·(−gz_b) = mgl·(gz_by, −gz_bx, 0).
 */
static State7 gyro_deriv(State7 s, float I1, float I2, float I3,
                          float mgl, bool gravity)
{
    float wx = s.v[0], wy = s.v[1], wz = s.v[2];
    float qw = s.v[3], qx = s.v[4], qy = s.v[5], qz = s.v[6];

    /* Gravity torque in body frame (zero if gravity disabled) */
    float tau_x = 0.0f, tau_y = 0.0f;
    if (gravity) {
        /*
         * World-Z in body frame = third row of R = (ex[2], ey[2], ez[2])
         * from the rotation-matrix formula:
         *   ex[2] = 2(qx·qz − qw·qy)
         *   ey[2] = 2(qy·qz + qw·qx)
         * (ez[2] not needed — τz = 0 always)
         */
        float gz_bx = 2.0f * (qx * qz - qw * qy);
        float gz_by = 2.0f * (qy * qz + qw * qx);
        tau_x =  mgl * gz_by;
        tau_y = -mgl * gz_bx;
    }

    /* Euler's equations */
    float dwx = ((I2 - I3) * wy * wz + tau_x) / I1;
    float dwy = ((I3 - I1) * wz * wx + tau_y) / I2;
    float dwz = ((I1 - I2) * wx * wy)          / I3;

    /* Quaternion kinematics: q̇ = ½ q ⊗ (0, ω) */
    float dqw = 0.5f * (-qx * wx - qy * wy - qz * wz);
    float dqx = 0.5f * ( qw * wx + qy * wz - qz * wy);
    float dqy = 0.5f * ( qw * wy - qx * wz + qz * wx);
    float dqz = 0.5f * ( qw * wz + qx * wy - qy * wx);

    State7 d;
    d.v[0]=dwx; d.v[1]=dwy; d.v[2]=dwz;
    d.v[3]=dqw; d.v[4]=dqx; d.v[5]=dqy; d.v[6]=dqz;
    return d;
}

/* ── quaternion → rotation axes ─────────────────────────────────────── */

/*
 * quat_to_axes() — extract body frame axes in world coordinates.
 *
 * The rotation matrix R (body→world) has these columns:
 *   ex = first  column = body X in world
 *   ey = second column = body Y in world
 *   ez = third  column = body Z in world (the spin axis)
 *
 * Standard formula from unit quaternion q = (qw, qx, qy, qz):
 */
static void quat_to_axes(const float q[4],
                          float ex[3], float ey[3], float ez[3])
{
    float qw=q[0], qx=q[1], qy=q[2], qz=q[3];
    ex[0] = 1.0f - 2.0f*(qy*qy + qz*qz);
    ex[1] = 2.0f*(qx*qy + qw*qz);
    ex[2] = 2.0f*(qx*qz - qw*qy);

    ey[0] = 2.0f*(qx*qy - qw*qz);
    ey[1] = 1.0f - 2.0f*(qx*qx + qz*qz);
    ey[2] = 2.0f*(qy*qz + qw*qx);

    ez[0] = 2.0f*(qx*qz + qw*qy);
    ez[1] = 2.0f*(qy*qz - qw*qx);
    ez[2] = 1.0f - 2.0f*(qx*qx + qy*qy);
}

/*
 * gram_schmidt() — re-orthonormalise three axis vectors in place.
 *
 * Prevents floating-point round-off in quat_to_axes() from
 * accumulating into non-orthogonal frame axes.
 *
 *   e1 ← normalize(e1)
 *   e2 ← normalize(e2 − (e2·e1)·e1)
 *   e3 ← e1 × e2                       (always exactly orthogonal)
 */
static void gram_schmidt(float e1[3], float e2[3], float e3[3])
{
    /* Normalise e1 */
    float n = sqrtf(e1[0]*e1[0] + e1[1]*e1[1] + e1[2]*e1[2]);
    if (n > 1e-9f) { e1[0]/=n; e1[1]/=n; e1[2]/=n; }

    /* Remove e1 component from e2, normalise */
    float d = e2[0]*e1[0] + e2[1]*e1[1] + e2[2]*e1[2];
    e2[0] -= d*e1[0]; e2[1] -= d*e1[1]; e2[2] -= d*e1[2];
    n = sqrtf(e2[0]*e2[0] + e2[1]*e2[1] + e2[2]*e2[2]);
    if (n > 1e-9f) { e2[0]/=n; e2[1]/=n; e2[2]/=n; }

    /* e3 = e1 × e2 (guaranteed orthonormal) */
    e3[0] = e1[1]*e2[2] - e1[2]*e2[1];
    e3[1] = e1[2]*e2[0] - e1[0]*e2[2];
    e3[2] = e1[0]*e2[1] - e1[1]*e2[0];
}

/* ── trail ring-buffer ───────────────────────────────────────────────── */

typedef struct { int col, row; } TrailPt;

/* ── Gyro struct ─────────────────────────────────────────────────────── */

typedef struct {
    /* Physics state */
    float omega[3];      /* angular velocity in body frame (rad/s)       */
    float quat[4];       /* orientation quaternion [qw,qx,qy,qz]         */

    /* Derived (updated each tick from quat) */
    float ex[3];         /* body X axis in world frame                   */
    float ey[3];         /* body Y axis in world frame                   */
    float ez[3];         /* body Z axis (spin axis) in world frame        */
    float L[3];          /* angular momentum in world frame               */

    /* Inertia */
    float I1, I2, I3;

    /* Gravity */
    bool  gravity;
    float mgl;

    /* Polhode trail: ring-buffer of body-Z tip screen positions */
    TrailPt trail[TRAIL_LEN];
    int     trail_head;
    int     trail_fill;   /* how many valid entries (saturates at TRAIL_LEN) */
    bool    show_trail;

    /* View */
    float   view_phi;     /* azimuth  (auto-rotates slowly)               */
    float   view_theta;   /* elevation                                     */

    /* Control */
    int     preset;
    bool    paused;
} Gyro;

/* ── initialisation ──────────────────────────────────────────────────── */

/*
 * gyro_set_preset() — load initial conditions from preset table.
 *
 * Initial quaternion: rotate by tilt_deg around tilt_axis.
 *   q = (cos(θ/2), sin(θ/2)·axis)
 * This tilts the body away from the upright (world-Z) position,
 * giving the spin axis a nonzero nutation angle to start from.
 */
static void gyro_set_preset(Gyro *g, int p)
{
    const GPreset *pr = &PRESETS[p];
    g->preset  = p;
    g->gravity = pr->gravity;
    g->mgl     = pr->mgl;
    g->I1 = pr->I1; g->I2 = pr->I2; g->I3 = pr->I3;

    g->omega[0] = pr->omega[0];
    g->omega[1] = pr->omega[1];
    g->omega[2] = pr->omega[2];

    /* Compute initial quaternion from tilt */
    float half  = pr->tilt_deg * (float)M_PI / 360.0f;  /* half-angle */
    float sh    = sinf(half);
    float ax    = pr->tilt_axis[0], ay = pr->tilt_axis[1], az = pr->tilt_axis[2];
    float alen  = sqrtf(ax*ax + ay*ay + az*az);
    if (alen < 1e-9f) alen = 1.0f;
    g->quat[0] = cosf(half);
    g->quat[1] = sh * ax / alen;
    g->quat[2] = sh * ay / alen;
    g->quat[3] = sh * az / alen;

    /* Derive initial axes */
    quat_to_axes(g->quat, g->ex, g->ey, g->ez);
    gram_schmidt(g->ex, g->ey, g->ez);

    /* Clear trail */
    g->trail_head = 0;
    g->trail_fill = 0;
}

static void gyro_init(Gyro *g)
{
    memset(g, 0, sizeof *g);
    g->view_phi   = 0.4f;
    g->view_theta = 0.6f;
    g->show_trail = true;
    gyro_set_preset(g, 0);
}

/* ── RK4 integration step ────────────────────────────────────────────── */

/*
 * gyro_step() — one RK4 sub-step of duration dt.
 *
 * After the step:
 *   1. Normalise the quaternion (prevents magnitude drift).
 *   2. Re-derive body axes via quat_to_axes + Gram-Schmidt.
 *   3. Recompute angular momentum L = Σ Ii·ωi·ei (world frame).
 */
static void gyro_step(Gyro *g, float dt)
{
    State7 s;
    s.v[0]=g->omega[0]; s.v[1]=g->omega[1]; s.v[2]=g->omega[2];
    s.v[3]=g->quat[0];  s.v[4]=g->quat[1];
    s.v[5]=g->quat[2];  s.v[6]=g->quat[3];

    /* RK4 */
    State7 k1 = gyro_deriv(s,                           g->I1, g->I2, g->I3, g->mgl, g->gravity);
    State7 k2 = gyro_deriv(s7_add(s, s7_scale(k1, dt*0.5f)), g->I1, g->I2, g->I3, g->mgl, g->gravity);
    State7 k3 = gyro_deriv(s7_add(s, s7_scale(k2, dt*0.5f)), g->I1, g->I2, g->I3, g->mgl, g->gravity);
    State7 k4 = gyro_deriv(s7_add(s, s7_scale(k3, dt)),      g->I1, g->I2, g->I3, g->mgl, g->gravity);

    State7 ns = s7_add(s, s7_scale(
        s7_add(s7_add(k1, s7_scale(k2, 2.0f)), s7_add(s7_scale(k3, 2.0f), k4)),
        dt / 6.0f));

    g->omega[0]=ns.v[0]; g->omega[1]=ns.v[1]; g->omega[2]=ns.v[2];
    g->quat[0] =ns.v[3]; g->quat[1] =ns.v[4];
    g->quat[2] =ns.v[5]; g->quat[3] =ns.v[6];

    /* Normalise quaternion — keeps R in SO(3) */
    float qn = sqrtf(g->quat[0]*g->quat[0] + g->quat[1]*g->quat[1]
                   + g->quat[2]*g->quat[2] + g->quat[3]*g->quat[3]);
    if (qn > 1e-9f) {
        g->quat[0]/=qn; g->quat[1]/=qn; g->quat[2]/=qn; g->quat[3]/=qn;
    }

    /* Re-derive body axes + Gram-Schmidt re-orthogonalisation */
    quat_to_axes(g->quat, g->ex, g->ey, g->ez);
    gram_schmidt(g->ex, g->ey, g->ez);

    /* Angular momentum in world frame: L = R · diag(I) · ω */
    for (int k = 0; k < 3; k++) {
        g->L[k] = g->I1*g->omega[0]*g->ex[k]
                + g->I2*g->omega[1]*g->ey[k]
                + g->I3*g->omega[2]*g->ez[k];
    }
}

/* ── drawing ─────────────────────────────────────────────────────────── */

/*
 * gyro_draw() — render the complete scene into WINDOW *w.
 *
 * Layers (painter's order: farthest first):
 *   1. Ground reference ring (dotted ellipse in XY plane, dim)
 *   2. World-Z reference axis (vertical line, dim)
 *   3. Body-disc equator (ring around spin axis, blue)
 *   4. Polhode trail (dots tracking body-Z tip, magenta)
 *   5. Body axes X, Y, Z (red, green, cyan — axis letters at tips)
 *   6. Angular momentum L (yellow dashed arrow)
 *
 * Depth-cuing: A_BOLD for elements closer to the viewer (depth < 0),
 * normal or A_DIM for elements receding (depth > 0).
 *
 * Scale: axis_scale ≈ min(cols,rows*2)/7 cells per world unit,
 * so a unit-length axis occupies ~1/7 of the screen.
 */
static void gyro_draw(const Gyro *g, WINDOW *w, int cols, int rows)
{
    float phi   = g->view_phi;
    float theta = g->view_theta;
    int   cx    = cols / 2;
    int   cy    = (rows + 1) / 2;

    float sc_h  = (float)(cols) / 7.0f;
    float sc_v  = (float)(rows - 2) * 2.0f / 7.0f;
    float scale = sc_h < sc_v ? sc_h : sc_v;
    if (scale < 1.0f) scale = 1.0f;

    /* ── 1. Ground ring ── */
    wattron(w, COLOR_PAIR(5) | A_DIM);
    float gr = 1.2f;   /* ring radius in world units */
    for (int i = 0; i < GROUND_PTS; i++) {
        float t  = 2.0f * (float)M_PI * i / GROUND_PTS;
        float wx = gr * cosf(t), wy = gr * sinf(t);
        int c, r; float d;
        project(wx, wy, 0.0f, phi, theta, scale, cx, cy, &c, &r, &d);
        if (c >= 0 && c < cols && r >= 1 && r < rows - 1)
            mvwaddch(w, r, c, '.');
    }
    wattroff(w, COLOR_PAIR(5) | A_DIM);

    /* ── 2. World-Z reference (vertical axis) ── */
    draw_seg3d(w, 0,0,0, 0,0,1.3f, phi,theta,scale,cx,cy,cols,rows,
               (chtype)(COLOR_PAIR(5) | A_DIM));

    /* ── 3. Body-disc equator ── */
    {
        float dr = 0.45f;   /* disc radius */
        chtype da = (chtype)(COLOR_PAIR(7) | A_DIM);
        for (int i = 0; i < DISC_PTS; i++) {
            float t  = 2.0f * (float)M_PI * i / DISC_PTS;
            float ct = cosf(t), st = sinf(t);
            float wx = dr*(ct*g->ex[0] + st*g->ey[0]);
            float wy = dr*(ct*g->ex[1] + st*g->ey[1]);
            float wz = dr*(ct*g->ex[2] + st*g->ey[2]);
            int c, r; float d;
            project(wx, wy, wz, phi, theta, scale, cx, cy, &c, &r, &d);
            if (c >= 0 && c < cols && r >= 1 && r < rows - 1) {
                wattron(w, da);
                mvwaddch(w, r, c, 'o');
                wattroff(w, da);
            }
        }
    }

    /* ── 4. Polhode trail (body-Z tip, screen-space dots) ── */
    if (g->show_trail && g->trail_fill > 0) {
        int n = g->trail_fill;
        for (int i = 0; i < n; i++) {
            int idx = (g->trail_head - n + i + TRAIL_LEN) % TRAIL_LEN;
            int c = g->trail[idx].col, r = g->trail[idx].row;
            if (c < 0 || c >= cols || r < 1 || r >= rows - 1) continue;
            /* Fade: recent = bright, old = dim */
            chtype attr;
            if (i > n * 3 / 4)      attr = (chtype)(COLOR_PAIR(6) | A_BOLD);
            else if (i > n / 2)      attr = (chtype)(COLOR_PAIR(6));
            else                     attr = (chtype)(COLOR_PAIR(6) | A_DIM);
            wattron(w, attr);
            mvwaddch(w, r, c, '.');
            wattroff(w, attr);
        }
    }

    /* ── 5. Body axes ── depth-sort tip vs. origin so nearer axis is brighter ── */
    struct { const float *axis; const char *label; int pair; } axes[3] = {
        { g->ex, "X", 1 },
        { g->ey, "Y", 2 },
        { g->ez, "Z", 3 },
    };
    /* Simple depth sort on tip depth: draw farthest first */
    float depths[3];
    for (int i = 0; i < 3; i++) {
        int c, r; float d;
        project(axes[i].axis[0], axes[i].axis[1], axes[i].axis[2],
                phi, theta, scale, cx, cy, &c, &r, &d);
        depths[i] = d;
    }
    int order[3] = {0, 1, 2};
    /* Bubble-sort by depth descending (draw farthest first) */
    for (int i = 0; i < 2; i++)
        for (int j = i+1; j < 3; j++)
            if (depths[order[i]] < depths[order[j]]) {
                int tmp=order[i]; order[i]=order[j]; order[j]=tmp;
            }

    for (int oi = 0; oi < 3; oi++) {
        int i = order[oi];
        const float *ax = axes[i].axis;
        chtype attr = (chtype)(COLOR_PAIR(axes[i].pair) |
                       (depths[i] < 0.0f ? A_BOLD : 0u));

        draw_seg3d(w, 0,0,0, ax[0],ax[1],ax[2],
                   phi,theta,scale,cx,cy,cols,rows, attr);

        /* Axis label at the tip */
        int c, r; float d;
        project(ax[0]*1.15f, ax[1]*1.15f, ax[2]*1.15f,
                phi,theta,scale,cx,cy,&c,&r,&d);
        if (c >= 0 && c < cols && r >= 1 && r < rows-1) {
            wattron(w, attr);
            mvwaddch(w, r, c, (chtype)(unsigned char)axes[i].label[0]);
            wattroff(w, attr);
        }
    }

    /* ── 6. Angular momentum L (yellow, normalised to unit length for display) ── */
    {
        float Lmag = sqrtf(g->L[0]*g->L[0]+g->L[1]*g->L[1]+g->L[2]*g->L[2]);
        if (Lmag > 1e-9f) {
            float Ldx = g->L[0]/Lmag, Ldy = g->L[1]/Lmag, Ldz = g->L[2]/Lmag;
            draw_seg3d(w, 0,0,0, Ldx,Ldy,Ldz, phi,theta,scale,cx,cy,cols,rows,
                       (chtype)(COLOR_PAIR(4) | A_BOLD));
            int c, r; float d;
            project(Ldx*1.15f, Ldy*1.15f, Ldz*1.15f,
                    phi,theta,scale,cx,cy,&c,&r,&d);
            if (c >= 0 && c < cols && r >= 1 && r < rows-1) {
                wattron(w, COLOR_PAIR(4) | A_BOLD);
                mvwaddch(w, r, c, 'L');
                wattroff(w, COLOR_PAIR(4) | A_BOLD);
            }
        }
    }
}

/* ── trail update ────────────────────────────────────────────────────── */

static void gyro_update_trail(Gyro *g, int cx, int cy,
                               float scale, int cols, int rows)
{
    int c, r; float d;
    project(g->ez[0], g->ez[1], g->ez[2],
            g->view_phi, g->view_theta, scale, cx, cy, &c, &r, &d);
    g->trail[g->trail_head] = (TrailPt){ c, r };
    g->trail_head = (g->trail_head + 1) % TRAIL_LEN;
    if (g->trail_fill < TRAIL_LEN) g->trail_fill++;

    (void)cols; (void)rows;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct { Gyro gyro; } Scene;

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    gyro_init(&s->gyro);
}

/*
 * scene_tick() — advance the simulation by one fixed timestep dt.
 *
 * SUB_STEPS RK4 steps are taken within each sim tick.  This keeps
 * the individual RK4 step small relative to the rotation period,
 * ensuring accurate integration even at the default 60 Hz tick rate.
 *
 * The view azimuth auto-rotates at 0.15 rad/s for a cinematic view.
 */
static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    Gyro *g = &s->gyro;
    if (g->paused) return;

    float sub_dt = dt / (float)SUB_STEPS;
    for (int i = 0; i < SUB_STEPS; i++)
        gyro_step(g, sub_dt);

    /* Slow auto-rotation of view */
    g->view_phi += 0.15f * dt;

    /* Update polhode trail (once per tick, not sub-step) */
    if (g->show_trail) {
        float sc_h  = (float)cols / 7.0f;
        float sc_v  = (float)(rows - 2) * 2.0f / 7.0f;
        float scale = sc_h < sc_v ? sc_h : sc_v;
        gyro_update_trail(g, cols/2, (rows+1)/2, scale, cols, rows);
    }
}

/*
 * scene_draw() — render the gyroscope into WINDOW *w.
 *
 * alpha accepted for framework signature compatibility but unused —
 * the gyroscope uses rigid-body physics where the draw position IS
 * the physics position (no separate interpolation needed at 60 Hz).
 */
static void scene_draw(Scene *s, WINDOW *w, int cols, int rows,
                       float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    gyro_draw(&s->gyro, w, cols, rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s)  { (void)s; endwin(); }
static void screen_resize(Screen *s){ endwin(); refresh(); getmaxyx(stdscr, s->rows, s->cols); }

static void screen_draw(Screen *s, Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Gyro    *g  = &sc->gyro;
    const GPreset *pr = &PRESETS[g->preset];

    float omag = sqrtf(g->omega[0]*g->omega[0]
                      +g->omega[1]*g->omega[1]
                      +g->omega[2]*g->omega[2]);

    char buf[160];
    snprintf(buf, sizeof buf,
             " %-40s  I=(%.1f,%.1f,%.1f)  |ω|=%.1f rad/s  g:%s  %dHz  %.1ffps  %s ",
             pr->name, g->I1, g->I2, g->I3, omag,
             g->gravity ? "ON " : "OFF",
             sim_fps, fps, g->paused ? "PAUSED" : "      ");
    attron(COLOR_PAIR(8) | A_BOLD);
    mvprintw(0, 0, "%.*s", s->cols, buf);
    attroff(COLOR_PAIR(8) | A_BOLD);

    attron(A_DIM);
    mvprintw(s->rows-1, 0,
             " q:quit  spc:pause  n/p:preset  r:restart  g:gravity  t:trail  arrows:view ");
    attroff(A_DIM);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
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
    app->scene.gyro.trail_head = 0;
    app->scene.gyro.trail_fill = 0;
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Gyro *g = &app->scene.gyro;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        g->paused = !g->paused;
        break;

    case 'n':
        gyro_set_preset(g, (g->preset + 1) % N_PRESETS);
        break;

    case 'p':
        gyro_set_preset(g, (g->preset + N_PRESETS - 1) % N_PRESETS);
        break;

    case 'r': case 'R':
        gyro_set_preset(g, g->preset);
        break;

    case 'g': case 'G':
        g->gravity = !g->gravity;
        break;

    case 't': case 'T':
        g->show_trail = !g->show_trail;
        if (!g->show_trail) { g->trail_head = 0; g->trail_fill = 0; }
        break;

    case KEY_LEFT:  g->view_phi   -= 0.1f; break;
    case KEY_RIGHT: g->view_phi   += 0.1f; break;
    case KEY_UP:
        g->view_theta += 0.05f;
        if (g->view_theta > 1.4f) g->view_theta = 1.4f;
        break;
    case KEY_DOWN:
        g->view_theta -= 0.05f;
        if (g->view_theta < 0.1f) g->view_theta = 0.1f;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFFu));

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene);

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
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── sim accumulator ─────────────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── alpha ───────────────────────────────────────────────── */
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

        /* ── frame cap — sleep BEFORE render ─────────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── draw + present ──────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha, dt_sec);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
