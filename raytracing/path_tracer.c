/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */

/*
 * path_tracer.c — a tiny path tracer that draws the classic Cornell box as
 * coloured ASCII. Rays bounce randomly around a closed box; we average
 * thousands of these random walks per pixel, so the picture starts as noise
 * and slowly sharpens — with soft shadows and colour bleeding between walls.
 *
 * Simpler sister files (same scene): sphere_raytrace.c (one bounce, no
 * randomness), cube_raytrace.c, capsule_raytrace.c. The scene is the standard
 * Cornell box (Goral et al. 1984); the famous compact version is Beason's
 * "smallpt".
 */

#define _POSIX_C_SOURCE 199309L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1  Settings you can tweak ── */

/* §1.1 Loop speed. Tracing is slow, so we aim for 30 frames/sec, not 60. */
#define NS_PER_SEC 1000000000LL        /* nanoseconds in a second             */
#define TARGET_FPS 30
#define DT_CAP_NS 200000000LL          /* clamp a long stall to 0.2 s so the
                                          sim can't try to "catch up" forever */
#define FPS_WINDOW_NS (NS_PER_SEC / 2) /* refresh the fps readout twice a sec */

/* §1.2 The camera lens. */
#define ASPECT 0.47f  /* characters are taller than wide; this squares pixels */
#define FOV_DEG 66.0f /* how wide an angle the camera takes in                */

/* §1.3 How rays bounce. */
#define MAX_DEPTH 7    /* give up on a ray after this many bounces            */
#define RR_DEPTH 3     /* after this many bounces, start randomly dropping
                          dim rays to save work (see russian_roulette)        */
#define SPP_DEFAULT 2  /* rays fired per pixel each frame                     */
#define SPP_MIN 1
#define SPP_MAX 8
#define ACCUM_CAP 8192 /* stop adding rays once the image is clean enough     */
#define RAY_EPS 1e-4f  /* nudge a bounced ray off the surface it left, so it
                          doesn't immediately "hit" itself                    */

#define MAX_W 320 /* biggest screen we plan for — the image buffer is this big */
#define MAX_H 100

/*
 * §1.4 Where things sit in the box. x runs left(red)→right(green), y runs
 * floor→ceiling, z runs from the open front toward the back wall. The camera
 * sits just outside the open front looking in.
 */
#define CAM_X 0.00f
#define CAM_Y 0.05f
#define CAM_Z -1.50f

/* §1.5 Characters ordered dark→bright; we pick one by how bright a pixel is.
 * (Paul Bourke's 92-character ramp.) */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* §1.6 Colour slots we hand to ncurses. */
#define PAIR_CUBE_BASE 1   /* start of the 216-colour block (6×6×6 cube) */
#define PAIR_HUD 217       /* yellow status text         */
#define PAIR_HINT 218      /* cyan key-hint strip        */
#define PAIR_BAR_FILL 219  /* filled part of the bar     */
#define PAIR_BAR_EMPTY 220 /* empty part of the bar      */

/* §1.7 What to draw, toggled with the 'd' key. PT is the real renderer; the
 * other three are quick diagnostic views that stop at the first thing a ray
 * hits, so they show up instantly with no bouncing or randomness. Switching
 * views throws away the running image, since the views aren't comparable. */
typedef enum {
  MODE_PT = 0,     /* the real path tracer — bounces light, builds up slowly */
  MODE_NORMAL = 1, /* colour each surface by which way it faces              */
  MODE_ALBEDO = 2, /* each surface's own colour, with no lighting            */
  MODE_DEPTH = 3,  /* grey by distance: near is bright, far is dark          */
  MODE_N = 4,      /* how many views there are, so 'd' can wrap around       */
} ShadeMode;

static const char *shade_mode_name(ShadeMode m) {
  switch (m) {
  case MODE_PT:
    return "PT    ";
  case MODE_NORMAL:
    return "NORMAL";
  case MODE_ALBEDO:
    return "ALBEDO";
  case MODE_DEPTH:
    return "DEPTH ";
  default:
    return "?     ";
  }
}

/* §1.7.1 The light is enormously brighter than the walls, so in the ALBEDO
 * view we shrink it by this much — otherwise it would just blow out to white. */
#define ALBEDO_LIGHT_SCALE 0.06f

/* §1.8 Turning brightness into something a screen can show. Light values have
 * no upper limit, but the screen tops out, so we squash them into 0..1. We pin
 * the squash so the ceiling light lands at pure white and the bright surfaces
 * spread out near the top instead of bunching together. EXPOSURE is an overall
 * brightness dial (1.0 = leave as-is); raise it only if the box looks dim. */
#define TONE_WHITE 15.0f   /* brightness that should come out pure white   */
#define TONE_EXPOSURE 1.0f /* overall brightness dial                      */
#define DISPLAY_GAMMA 2.2f /* matches how screens map values to brightness */

/* ── §2  Clock and sleep ── */

/* CLOCK_MONOTONIC, not the wall clock: it only ever counts forward, so a clock
 * change (NTP, daylight saving) can't make our elapsed-time math go negative. */
static long long clock_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

/* Sleeps off the leftover time in a frame so we don't peg a CPU core at 100%. */
static void clock_sleep_ns(long long ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / NS_PER_SEC, ns % NS_PER_SEC};
  nanosleep(&ts, NULL);
}

/* ── §3  3-D vector math ── */

/* Three floats. We use it for a point in space, a direction, OR a colour
 * (red/green/blue) — all three are just three numbers we add and scale the
 * same way, so one type and one set of helpers covers all of them. */
typedef struct {
  float x, y, z;
} V3;

static inline V3 v3(float x, float y, float z) { return (V3){x, y, z}; }
static inline V3 v3add(V3 a, V3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline V3 v3sub(V3 a, V3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline V3 v3mul(V3 a, V3 b) {
  return v3(a.x * b.x, a.y * b.y, a.z * b.z);
}
static inline V3 v3s(float s, V3 a) { return v3(s * a.x, s * a.y, s * a.z); }
static inline float v3dot(V3 a, V3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3len(V3 a) { return sqrtf(v3dot(a, a)); }
static inline V3 v3norm(V3 a) {
  float length = v3len(a);
  return length > 1e-9f ? v3s(1.f / length, a) : v3(0, 1, 0);
}
static inline V3 v3cross(V3 a, V3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
}

/* Biggest of the three numbers. Russian roulette uses it to judge how much
 * "life" a ray has left — the brightest channel, not the average, so a ray
 * carrying mostly deep red doesn't get thrown away too early. */
static inline float v3maxc(V3 a) {
  return a.x > a.y ? (a.x > a.z ? a.x : a.z) : (a.y > a.z ? a.y : a.z);
}

/* ── §4  Random numbers ── */

/* Path tracing burns through a huge number of random values, so we use a fast,
 * cheap generator (xorshift). Its whole state is one 32-bit number, which we
 * pass around by pointer rather than keeping a hidden global one. */
typedef uint32_t Rng;

/* Advances the generator and returns a random float in [0, 1). */
static float rng_f(Rng *rng_state) {
  *rng_state ^= *rng_state << 13;
  *rng_state ^= *rng_state >> 17;
  *rng_state ^= *rng_state << 5;
  return (float)(*rng_state >> 1) * (1.f / (float)0x7FFFFFFF);
}

/* Gives each pixel, on each frame, its own starting seed. Without this,
 * neighbouring pixels would share the same "random" sequence and the noise
 * would line up into visible streaks instead of looking like static. */
static Rng rng_seed(int pixel_x, int pixel_y, int frame) {
  uint32_t s = (uint32_t)(pixel_x * 1973 + pixel_y * 9277 + frame * 26699 + 1);
  s ^= s << 13;
  s ^= s >> 7;
  s ^= s << 17;
  return s ? s : 1u;
}

/* ── §5  What's in the box ──
 *
 * The whole scene is six flat walls (floor, ceiling, back, left, right, and a
 * small lamp on the ceiling) plus two balls on the floor. That's it.
 */

/* §5.1  A surface's look. `albedo` is how much red/green/blue it bounces back
 * (0..1 each — the red wall bounces lots of red, almost no green or blue), and
 * `emit` is light it gives off (zero for everything but the lamp). Every
 * surface here is matte — it scatters light evenly in all directions — which is
 * exactly why a bounce later boils down to "multiply the ray's colour by
 * albedo" and nothing more. */
typedef struct {
  V3 albedo; /* fraction of each colour it reflects, 0..1 */
  V3 emit;   /* light it gives off; nonzero only for the lamp */
} Mat;

static const Mat k_mats[] = {
    /* 0  white  */ {{0.73f, 0.73f, 0.73f}, {0, 0, 0}},
    /* 1  red    */ {{0.65f, 0.05f, 0.05f}, {0, 0, 0}},
    /* 2  green  */ {{0.12f, 0.45f, 0.15f}, {0, 0, 0}},
    /* 3  light  */ {{0, 0, 0}, {15.f, 14.f, 11.f}}, /* warm */
    /* 4  gold   */ {{0.80f, 0.58f, 0.18f}, {0, 0, 0}},
    /* 5  indigo */ {{0.22f, 0.28f, 0.82f}, {0, 0, 0}},
};

static inline bool mat_is_light(const Mat *m) {
  return m->emit.x > 0.f || m->emit.y > 0.f || m->emit.z > 0.f;
}

/* §5.2  A flat rectangle lined up with the axes — a wall, floor, ceiling, or
 * the lamp. We store it as "pinned on one axis, with a min/max box on the other
 * two," which makes the ray test cheap and exact (one divide, two range
 * checks). `axis` says which direction it's pinned along. */
typedef struct {
  int axis;           /* which axis it lies flat against: 0=x, 1=y, 2=z */
  float pos;          /* where it sits along that axis                  */
  float lo[2], hi[2]; /* min/max along the other two axes               */
  int mat;            /* which material (row in the table above)        */
} Quad;

/* The six walls and the lamp. The lamp hangs a hair below the ceiling so a ray
 * heading straight up reaches the lamp first and picks up its light. */
static const Quad k_quads[] = {
    /* floor    y=-1   x∈[-1,1] z∈[0,2] */ {
        1, -1.0f, {-1.f, 0.f}, {1.f, 2.f}, 0},
    /* ceiling  y=+1   x∈[-1,1] z∈[0,2] */
    {1, 1.0f, {-1.f, 0.f}, {1.f, 2.f}, 0},
    /* back     z=+2   x∈[-1,1] y∈[-1,1]*/
    {2, 2.0f, {-1.f, -1.f}, {1.f, 1.f}, 0},
    /* left     x=-1   y∈[-1,1] z∈[0,2] */
    {0, -1.0f, {-1.f, 0.f}, {1.f, 2.f}, 1},
    /* right    x=+1   y∈[-1,1] z∈[0,2] */
    {0, 1.0f, {-1.f, 0.f}, {1.f, 2.f}, 2},
    /* light    y=0.98 centred overhead */
    {1, 0.98f, {-0.36f, 0.62f}, {0.36f, 1.38f}, 3},
};
#define N_QUADS ((int)(sizeof k_quads / sizeof k_quads[0]))

/* §5.3  One of the two balls on the floor: a centre and a radius. They rest a
 * hair above the floor so the ball and the floor don't fight over which one a
 * ray touches right at the contact point. */
typedef struct {
  V3 c;    /* centre */
  float r; /* radius */
  int mat; /* which material (row in the table above) */
} Sphere;

static const Sphere k_spheres[] = {
    {{-0.46f, -0.60f, 0.82f}, 0.38f, 4}, /* gold  , left  */
    {{0.44f, -0.60f, 1.16f}, 0.38f, 5},  /* indigo, right */
};
#define N_SPHERES ((int)(sizeof k_spheres / sizeof k_spheres[0]))

/* §5.4  Everything to render in one bundle, so each function below takes a
 * single `const Scene *` instead of reaching for these globals by name: the
 * material table, the walls, and the balls (with how many of each). */
typedef struct {
  const Mat *mats;
  const Quad *quads;
  int n_quads;
  const Sphere *spheres;
  int n_spheres;
} Scene;

static const Scene k_scene = {k_mats, k_quads, N_QUADS, k_spheres, N_SPHERES};

/* ── §6  Step 1: from a pixel to a ray ── */

/* The camera is a single point you look out from (a pinhole). `fov_deg` is how
 * wide it sees; `aspect` squashes things sideways so the box doesn't look
 * stretched, since terminal characters are taller than they are wide. */
typedef struct {
  V3 origin;
  float fov_deg;
  float aspect;
} Camera;

static const Camera k_camera = {{CAM_X, CAM_Y, CAM_Z}, FOV_DEG, ASPECT};

/* A ray: a start point and a direction, like a laser pointer. Every point on it
 * is origin + t·dir, and because dir has length 1, t is simply how far along we
 * are. This is the one thing every "what do I hit?" test works on. */
typedef struct {
  V3 origin;
  V3 dir; /* length 1, so t reads straight off as a distance */
} Ray;

/* Aims a ray from the camera out through one pixel. The tiny jitter (a random
 * fraction of a pixel) makes each ray pierce a slightly different spot inside
 * the pixel; averaging those samples smooths the jagged stair-steps off edges.
 * The minus sign on the vertical flips it, because screen rows count downward
 * while up in the world is +y. */
static Ray camera_ray(const Camera *cam, int col, int row, int cols, int rows,
                      float jitter_x, float jitter_y) {
  (void)rows; /* v normalised by cols */
  float centre_x = cols * 0.5f;
  float centre_y = rows * 0.5f;
  float tan_half = tanf(cam->fov_deg * (float)M_PI / 360.f);

  float u = ((col + jitter_x) - centre_x) / centre_x * tan_half;
  float v = -((row + jitter_y) - centre_y) / centre_x * tan_half / cam->aspect;

  V3 forward = v3(0, 0, 1);
  V3 right = v3(1, 0, 0);
  V3 up = v3(0, 1, 0);

  Ray ray;
  ray.origin = cam->origin;
  ray.dir = v3norm(v3add(forward, v3add(v3s(u, right), v3s(v, up))));
  return ray;
}

/* ── §7  Step 2: what does the ray hit first? ── */

/* What we learned at the nearest hit: how far (`t`), the exact point (`P`),
 * which way the surface faces there (`N`), and which material it is. We always
 * flip `N` to point back toward the ray, which keeps the bounce code in §9
 * simple — it never has to wonder which side it's looking at. */
typedef struct {
  float t; /* distance along the ray to the hit         */
  V3 P, N; /* the hit point, and which way the surface faces (toward the ray) */
  int mat; /* which material (row in the table)         */
} Hit;

/* Does the ray cross this rectangle, and if so how far away? Because the
 * rectangle is axis-aligned, this is just: find where the ray crosses its flat
 * plane, then check the two sideways coordinates land inside the box. */
static int ray_quad(Ray ray, const Quad *quad, float t_min, float *out_t,
                    V3 *out_normal) {
  float dir_a, origin_a;
  switch (quad->axis) {
  case 0:
    dir_a = ray.dir.x;
    origin_a = ray.origin.x;
    break;
  case 1:
    dir_a = ray.dir.y;
    origin_a = ray.origin.y;
    break;
  default:
    dir_a = ray.dir.z;
    origin_a = ray.origin.z;
    break;
  }
  if (fabsf(dir_a) < 1e-9f)
    return 0; /* parallel to plane */

  float t = (quad->pos - origin_a) / dir_a;
  if (t < t_min)
    return 0;

  float hit_x = ray.origin.x + t * ray.dir.x;
  float hit_y = ray.origin.y + t * ray.dir.y;
  float hit_z = ray.origin.z + t * ray.dir.z;

  float free_axis_u, free_axis_v;
  switch (quad->axis) {
  case 0:
    free_axis_u = hit_y;
    free_axis_v = hit_z;
    break;
  case 1:
    free_axis_u = hit_x;
    free_axis_v = hit_z;
    break;
  default:
    free_axis_u = hit_x;
    free_axis_v = hit_y;
    break;
  }
  if (free_axis_u < quad->lo[0] || free_axis_u > quad->hi[0] ||
      free_axis_v < quad->lo[1] || free_axis_v > quad->hi[1])
    return 0;

  /* Which way this face points — pick the sign so it faces back at the ray. */
  V3 normal = {0, 0, 0};
  switch (quad->axis) {
  case 0:
    normal.x = (dir_a > 0.f) ? -1.f : 1.f;
    break;
  case 1:
    normal.y = (dir_a > 0.f) ? -1.f : 1.f;
    break;
  default:
    normal.z = (dir_a > 0.f) ? -1.f : 1.f;
    break;
  }
  *out_t = t;
  *out_normal = normal;
  return 1;
}

/* Does the ray hit this ball, and how far away? Solving "where is the ray
 * exactly one radius from the centre?" gives a quadratic; `disc < 0` means the
 * ray sails past and misses. We take the nearer of the two crossings. */
static int ray_sphere(Ray ray, const Sphere *sphere, float t_min,
                      float *out_t) {
  V3 origin_to_centre = v3sub(ray.origin, sphere->c);
  float b = v3dot(ray.dir, origin_to_centre);
  float disc =
      b * b - v3dot(origin_to_centre, origin_to_centre) + sphere->r * sphere->r;
  if (disc < 0.f)
    return 0;

  float sq = sqrtf(disc);
  float t = -b - sq;
  if (t < t_min)
    t = -b + sq;
  if (t < t_min)
    return 0;
  *out_t = t;
  return 1;
}

/* Tries the ray against every wall and ball and keeps the closest hit. Only 8
 * shapes, so plain brute force is fine — no fancy spatial index needed. */
static int scene_hit(const Scene *scene, Ray ray, float t_min, Hit *out_hit) {
  float t_best = 1e30f; /* nearest hit so far; starts effectively infinite */
  int any = 0;

  for (int i = 0; i < scene->n_quads; i++) {
    float t;
    V3 normal;
    if (ray_quad(ray, &scene->quads[i], t_min, &t, &normal) && t < t_best) {
      t_best = t;
      out_hit->t = t;
      out_hit->P = v3add(ray.origin, v3s(t, ray.dir));
      out_hit->N = normal;
      out_hit->mat = scene->quads[i].mat;
      any = 1;
    }
  }
  for (int i = 0; i < scene->n_spheres; i++) {
    float t;
    if (ray_sphere(ray, &scene->spheres[i], t_min, &t) && t < t_best) {
      t_best = t;
      out_hit->t = t;
      out_hit->P = v3add(ray.origin, v3s(t, ray.dir));
      V3 outward_n = v3norm(v3sub(out_hit->P, scene->spheres[i].c));
      /* point the surface direction back toward the ray */
      out_hit->N =
          (v3dot(outward_n, ray.dir) < 0.f) ? outward_n : v3s(-1.f, outward_n);
      out_hit->mat = scene->spheres[i].mat;
      any = 1;
    }
  }
  return any;
}

/* ── §8  Step 3: which material did we hit? ── */

/* Looks up the material for a hit. It's a one-line lookup, but giving it a name
 * keeps the main loop reading as one clean step per stage. */
static const Mat *shade_at_hit(const Scene *scene, const Hit *hit) {
  return &scene->mats[hit->mat];
}

/* ── §9  Step 4: pick a random bounce direction ── */

/* Builds two directions at right angles to `normal`, so together with the
 * normal they form a little 3-axis frame standing on the surface. The bounce
 * sampler works in that frame and then maps the result back into the world.
 * The if-test just avoids picking a starter axis that's nearly parallel to the
 * normal (which would collapse to zero). */
static void onb(V3 normal, V3 *out_u, V3 *out_v) {
  V3 up = (fabsf(normal.x) < 0.9f) ? v3(1, 0, 0) : v3(0, 1, 0);
  *out_u = v3norm(v3cross(up, normal));
  *out_v = v3cross(normal, *out_u);
}

/* Picks a random direction for the ray to bounce off a matte surface. Directions
 * facing straight out are favoured over grazing ones (Malley's trick: scatter
 * points evenly on a flat disc, then lift them onto the dome). Doing it this way
 * — rather than guess-and-retry — is what later lets a bounce be just "multiply
 * by albedo," with no extra angle terms to carry around. */
static V3 sample_bounce(V3 normal, Rng *rng_state) {
  float r1 = rng_f(rng_state);
  float r2 = rng_f(rng_state);
  float phi = 2.f * (float)M_PI * r1;
  float sr2 = sqrtf(r2);
  float local_x = cosf(phi) * sr2;
  float local_y = sinf(phi) * sr2;
  float local_z = sqrtf(1.f - r2);
  V3 basis_u, basis_v;
  onb(normal, &basis_u, &basis_v);
  return v3norm(v3add(v3s(local_x, basis_u),
                      v3add(v3s(local_y, basis_v), v3s(local_z, normal))));
}

/* ── §10  Step 5: track the ray's leftover colour ── */

/* A bounce keeps only the share of light the surface reflects, so we multiply
 * the ray's running colour by the surface's albedo. After a few bounces this
 * product is "how much of whatever light we finally reach will make it back to
 * the eye along this path" — and it's why the red wall tints nearby surfaces
 * red as the picture builds up. */
static inline V3 throughput_chain(V3 throughput, V3 albedo) {
  return v3mul(throughput, albedo);
}

/* ── §11  Step 6: deciding when to stop ── */

/* Once a ray has bounced a few times, this randomly ends the dim ones instead
 * of cutting every ray off at a fixed depth (which would darken the image).
 * Survivors are scaled up by exactly the odds they survived, so the average
 * comes out the same — we just stop spending effort on rays that barely add
 * anything. Returns false when the caller should stop. "Dim" is judged by the
 * brightest channel, so a strongly-red ray isn't dropped over its low green. */
static bool russian_roulette(V3 *throughput, Rng *rng_state) {
  float p = v3maxc(*throughput);
  if (p < 1e-4f)
    return false; /* already basically black */
  if (rng_f(rng_state) > p)
    return false;                          /* unlucky roll — end it here */
  *throughput = v3s(1.f / p, *throughput); /* brighten survivors to stay fair */
  return true;
}

/* ── §12  The heart of it: follow one ray as it bounces ── */

/* Follows a single ray bouncing around the box until it reaches the lamp or
 * fizzles out, and returns the colour that one random walk saw. `throughput` is
 * the share of light still able to reach the eye along this path; `color` is
 * what we collect when we finally land on the lamp. Thousands of these walks,
 * averaged, make one pixel. It's a plain loop rather than recursion so you can
 * watch the colour get dimmer one bounce at a time. */
static V3 path_trace(const Scene *scene, Ray ray, Rng *rng_state) {
  V3 color = v3(0, 0, 0);
  V3 throughput = v3(1, 1, 1);

  for (int depth = 0; depth < MAX_DEPTH; depth++) {

    /* what does the ray hit? nothing → it escaped into the dark */
    Hit hit;
    if (!scene_hit(scene, ray, RAY_EPS, &hit))
      break;

    /* reached the lamp? collect its light and we're done */
    const Mat *mat = shade_at_hit(scene, &hit);
    if (mat_is_light(mat)) {
      color = v3add(color, v3mul(throughput, mat->emit));
      break;
    }

    /* after a few bounces, maybe quit early on a dim ray */
    if (depth >= RR_DEPTH && !russian_roulette(&throughput, rng_state))
      break;

    /* dim the ray by the share of light this surface reflects */
    throughput = throughput_chain(throughput, mat->albedo);

    /* bounce off in a new random direction, nudged clear of the surface */
    ray.dir = sample_bounce(hit.N, rng_state);
    ray.origin = v3add(hit.P, v3s(RAY_EPS, hit.N));
  }
  return color;
}

/* ── §13  Step 7: average the rays and put it on screen ──
 *
 * One ray is pure noise, so we keep adding more and show the running average,
 * which sharpens frame by frame. The pieces below add a frame of rays, then
 * turn the average into characters and colours.
 */

/* §13.1  Where the picture builds up. For every pixel we keep a running total
 * of the colours all its rays have seen, plus how many rays that is; the colour
 * we show is total ÷ count. The two live together because you always need both
 * to get the average. We keep the totals as raw, uncapped brightness and only
 * squash them to screen range at draw time — averaging first, squashing second.
 * It's one fixed-size block (no malloc), big enough for the largest screen we
 * expect; on resize we just clear it and start over. */
typedef struct {
  float sum[MAX_H][MAX_W][3]; /* running colour total per pixel (uncapped) */
  int samples;                /* how many rays are in that total           */
} Accumulator;

static Accumulator g_accum;

/* defined later, in §14 */
static V3 first_hit_viz(const Scene *scene, Ray ray, ShadeMode mode);

/* Fires one ray for a pixel and returns the colour it found: seed its own
 * randomness, jitter inside the pixel, aim the ray, and either trace it fully
 * (the real renderer) or take a quick diagnostic shortcut. */
static V3 sample_pixel(const Scene *scene, const Camera *cam, int col, int row,
                       int cols, int rows, int sample_idx, ShadeMode mode) {
  Rng rng_state = rng_seed(col, row, sample_idx);
  float jitter_x = rng_f(&rng_state) - 0.5f; /* random spot inside the pixel */
  float jitter_y = rng_f(&rng_state) - 0.5f;
  Ray primary = camera_ray(cam, col, row, cols, rows, jitter_x, jitter_y);
  return (mode == MODE_PT) ? path_trace(scene, primary, &rng_state)
                           : first_hit_viz(scene, primary, mode);
}

/* §13.3  The only two functions that change the picture buffer. */

/* Throws the picture away and starts over. We call it when something changes
 * what a ray means (resize, or the user changes a setting), never mid-frame. */
static void accum_reset(Accumulator *a) {
  memset(a->sum, 0, sizeof a->sum);
  a->samples = 0;
}

/* Fires another round of rays — `spp` of them per pixel — and adds what they
 * see into the running totals. We add raw colours here and leave the squash-to-
 * screen step for draw time, because averaging has to happen before squashing,
 * not after. */
static void accum_add_frame(Accumulator *a, const Scene *scene,
                            const Camera *cam, int cols, int rows, int spp,
                            int frame_idx, ShadeMode mode) {
  for (int row = 0; row < rows - 1 && row < MAX_H; row++) {
    for (int col = 0; col < cols && col < MAX_W; col++) {
      float sum_r = 0.f, sum_g = 0.f, sum_b = 0.f;

      for (int s = 0; s < spp; s++) {
        /* a number unique to this pixel-and-frame, so each ray gets fresh
         * randomness and frames don't repeat the same noise */
        int sample_idx = frame_idx * spp + s;
        V3 radiance =
            sample_pixel(scene, cam, col, row, cols, rows, sample_idx, mode);
        sum_r += radiance.x;
        sum_g += radiance.y;
        sum_b += radiance.z;
      }

      a->sum[row][col][0] += sum_r;
      a->sum[row][col][1] += sum_g;
      a->sum[row][col][2] += sum_b;
    }
  }
  a->samples += spp;
}

/* §13.4  Turning a brightness into a character and a colour. */

/* Squashes an unbounded brightness down into 0..1 so it fits on screen, with
 * the pin from §1.8 so the lamp lands at pure white instead of just near it. */
static inline float reinhard(float x) {
  x *= TONE_EXPOSURE;
  return x * (1.f + x / (TONE_WHITE * TONE_WHITE)) / (1.f + x);
}
/* Bends 0..1 to match how screens actually turn numbers into brightness. */
static inline float gamma_enc(float x) {
  return powf(x < 0.f ? 0.f : (x > 1.f ? 1.f : x), 1.f / DISPLAY_GAMMA);
}
/* How bright a colour looks to the eye — green counts most, blue least. */
static inline float rec601_luma(float r, float g, float b) {
  return 0.2126f * r + 0.7152f * g + 0.0722f * b;
}

/* Full screen-ready colour from a raw one: squash, then gamma, each channel. */
static inline V3 tonemap_to_srgb(V3 linear_hdr) {
  return v3(gamma_enc(reinhard(linear_hdr.x)), gamma_enc(reinhard(linear_hdr.y)),
            gamma_enc(reinhard(linear_hdr.z)));
}

/* Picks a character for a brightness: brighter pixel → denser-looking glyph. */
static char luma_to_ramp_char(float luma) {
  int ramp_index = (int)(luma * (float)(RAMP_LEN - 1) + 0.5f);
  if (ramp_index < 0)
    ramp_index = 0;
  if (ramp_index >= RAMP_LEN)
    ramp_index = RAMP_LEN - 1;
  return k_ramp[ramp_index];
}

/* Picks the closest terminal colour. Terminals offer a 6×6×6 grid of colours,
 * so we round each of red/green/blue to one of 6 levels and look up that slot. */
static int rgb_to_cube_pair(float r, float g, float b) {
  int red_5bit = (int)(r * 5.f + 0.5f);
  int green_5bit = (int)(g * 5.f + 0.5f);
  int blue_5bit = (int)(b * 5.f + 0.5f);
  if (red_5bit > 5)
    red_5bit = 5;
  if (red_5bit < 0)
    red_5bit = 0;
  if (green_5bit > 5)
    green_5bit = 5;
  if (green_5bit < 0)
    green_5bit = 0;
  if (blue_5bit > 5)
    blue_5bit = 5;
  if (blue_5bit < 0)
    blue_5bit = 0;
  return PAIR_CUBE_BASE + red_5bit * 36 + green_5bit * 6 + blue_5bit;
}

/* §13.5  Draws the picture buffer to the screen. For each pixel: average its
 * rays, squash that to a screen colour, then pick a character and a colour.
 * Averaging must come before squashing — squash first and the brightness comes
 * out wrong. (Reads the buffer; never changes it.) */
static void accum_draw(const Accumulator *a, int cols, int rows) {
  if (a->samples == 0)
    return;
  float inv_samples = 1.f / (float)a->samples;

  for (int row = 0; row < rows - 1 && row < MAX_H; row++) {
    for (int col = 0; col < cols && col < MAX_W; col++) {
      /* average of every ray that hit this pixel (still raw, uncapped) */
      V3 linear_avg = v3(a->sum[row][col][0] * inv_samples,
                         a->sum[row][col][1] * inv_samples,
                         a->sum[row][col][2] * inv_samples);
      V3 srgb = tonemap_to_srgb(linear_avg);

      char ch = luma_to_ramp_char(rec601_luma(srgb.x, srgb.y, srgb.z));
      int pair_index = rgb_to_cube_pair(srgb.x, srgb.y, srgb.z);
      attron(COLOR_PAIR(pair_index) | A_BOLD);
      mvaddch(row, col, (chtype)(unsigned char)ch);
      attroff(COLOR_PAIR(pair_index) | A_BOLD);
    }
  }
}

/* ── §14  The quick diagnostic views ──
 *
 * The 'd' key swaps the real renderer for one of these. They stop at the first
 * thing a ray hits and just colour it in — no bouncing, no randomness — so they
 * show up instantly and let you sanity-check the earlier steps on their own.
 */

/* Colours a surface by which way it faces, so each wall gets its own flat tint
 * (the usual trick: shift each direction value from -1..1 into 0..1). */
static inline V3 normal_to_rgb(V3 n) {
  return v3(n.x * 0.5f + 0.5f, n.y * 0.5f + 0.5f, n.z * 0.5f + 0.5f);
}

static V3 first_hit_viz(const Scene *scene, Ray ray, ShadeMode mode) {
  Hit hit;
  if (!scene_hit(scene, ray, RAY_EPS, &hit))
    return v3(0, 0, 0); /* ray hit nothing */

  const Mat *mat = shade_at_hit(scene, &hit);

  switch (mode) {
  case MODE_NORMAL:
    return normal_to_rgb(hit.N);

  case MODE_ALBEDO:
    /* each surface's own colour; the lamp is dimmed so it doesn't blow out */
    if (mat_is_light(mat)) {
      return v3(fminf(mat->emit.x * ALBEDO_LIGHT_SCALE, 1.f),
                fminf(mat->emit.y * ALBEDO_LIGHT_SCALE, 1.f),
                fminf(mat->emit.z * ALBEDO_LIGHT_SCALE, 1.f));
    }
    return mat->albedo;

  case MODE_DEPTH: {
    /* grey by distance: near is bright, far is dark */
    float v = 1.f / (1.f + hit.t);
    return v3(v, v, v);
  }

  default:
    return v3(0, 0, 0);
  }
}

/* ── §15  The on-screen extras: colours, progress bar, status line ── */

static int g_have_256 = 0;

/* Sets up the colour slots: the full 6×6×6 grid for the image if the terminal
 * has 256 colours, plus the few we reserve for the status text and the bar.
 * Falls back to plain colours on an old 8-colour terminal. */
static void color_init(void) {
  start_color();
  use_default_colors();
  g_have_256 = (COLORS >= 256);

  if (g_have_256) {
    /* fill the 216 image colours */
    for (int i = 0; i < 216; i++)
      init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
    init_pair(PAIR_HUD, 226, -1);       /* bright yellow         */
    init_pair(PAIR_HINT, 51, -1);       /* bright cyan           */
    init_pair(PAIR_BAR_FILL, 46, -1);   /* bright green (filled) */
    init_pair(PAIR_BAR_EMPTY, 240, -1); /* dim grey (empty)      */
  } else {
    init_pair(PAIR_CUBE_BASE, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
    init_pair(PAIR_BAR_FILL, COLOR_GREEN, -1);
    init_pair(PAIR_BAR_EMPTY, COLOR_WHITE, -1);
  }
}

/* Draws the bar showing how close the image is to "clean enough" — it fills as
 * rays pile up and is full once we stop adding more. Width tracks the terminal
 * but stays within sensible limits. */
static void draw_progress_bar(int row, int cols, int samples) {
  int bar_width = cols / 3;
  if (bar_width < 8)
    bar_width = 8;
  if (bar_width > 60)
    bar_width = 60;
  int filled = (int)((float)samples / (float)ACCUM_CAP * bar_width);
  if (filled > bar_width)
    filled = bar_width;

  int x = 1;
  attron(COLOR_PAIR(PAIR_HUD));
  mvaddch(row, x++, '[');
  for (int i = 0; i < bar_width; i++) {
    bool on = (i < filled);
    int pair = on ? PAIR_BAR_FILL : PAIR_BAR_EMPTY;
    int attr = on ? A_BOLD : A_DIM;
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(row, x++, (chtype)(unsigned char)(on ? '=' : '-'));
    attroff(COLOR_PAIR(pair) | attr);
  }
  attron(COLOR_PAIR(PAIR_HUD));
  mvaddch(row, x++, ']');
  attroff(COLOR_PAIR(PAIR_HUD));
}

/* Draws the overlay: a status line up top (speed, settings, what it's doing),
 * the progress bar, and a strip of key hints along the bottom. */
static void hud_draw(int cols, int rows, float fps, int spp, int samples,
                     ShadeMode mode, bool paused) {
  /* status text, top-right */
  char buf[140];
  snprintf(buf, sizeof buf, " %5.1f fps  spp:%d  samples:%-5d  mode:%s  %s ",
           (double)fps, spp, samples, shade_mode_name(mode),
           paused                 ? "PAUSED   "
           : (mode != MODE_PT)    ? "instant  "
           : samples >= ACCUM_CAP ? "CONVERGED"
                                  : "tracing  ");
  int len = (int)strlen(buf);
  if (len > cols)
    len = cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, cols - len, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* title, top-left */
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(0, 0, " PATH TRACER · CORNELL BOX ");
  attroff(COLOR_PAIR(PAIR_HUD));

  /* progress bar only makes sense for the real renderer; the quick views
   * finish in one frame, so the bar would always look instantly full */
  if (rows > 4 && mode == MODE_PT)
    draw_progress_bar(1, cols, samples);

  /* key hints, bottom row */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(rows - 1, 0, " q:quit  spc/p:pause  r:reset  d:mode  +/-:spp ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §16  The program: setup, the main loop, and keys ── */

/* Signals set these flags and the loop checks them — you can't safely do much
 * else from inside a signal handler. */
static volatile sig_atomic_t g_run = 1;
static volatile sig_atomic_t g_resize = 0;
static void on_sigint(int s) {
  (void)s;
  g_run = 0;
}
static void on_sigwinch(int s) {
  (void)s;
  g_resize = 1;
}
static void cleanup(void) { endwin(); }

/* Throws the picture away and starts fresh. Anything that changes what a ray
 * would see (a resize, or a settings change) makes the old picture wrong. */
static void restart_convergence(Accumulator *a, int *frame_idx) {
  accum_reset(a);
  *frame_idx = 0;
}

/* Should we add more rays this frame? No if paused; the quick views are done
 * after one frame; the real renderer keeps going until it's clean enough. */
static bool should_accumulate(ShadeMode mode, int samples, bool paused) {
  if (paused)
    return false;
  return (mode != MODE_PT) ? (samples == 0) : (samples < ACCUM_CAP);
}

/* Puts the terminal into the mode we need: keys read instantly, no echo, no
 * blinking cursor, getch() returns right away instead of waiting. */
static void screen_init(void) {
  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  typeahead(-1); /* stop ncurses checking for keys mid-draw, which causes tearing */
}

/* Handles the terminal being resized: get the new size and start the picture
 * over, since the old one no longer fits. */
static void apply_resize(int *cols, int *rows, Accumulator *a, int *frame_idx) {
  g_resize = 0;
  endwin();
  refresh();
  getmaxyx(stdscr, *rows, *cols);
  restart_convergence(a, frame_idx);
}

/* Draws one whole frame: clear, paint the picture, add the overlay, flush it
 * all to the terminal in one go. Only reads the picture buffer. */
static void render_frame(const Accumulator *a, int cols, int rows, float fps,
                         int spp, ShadeMode mode, bool paused) {
  erase();
  accum_draw(a, cols, rows);
  hud_draw(cols, rows, fps, spp, a->samples, mode, paused);
  wnoutrefresh(stdscr);
  doupdate();
}

/* Handles whatever key was pressed this frame: quit, pause, or change a setting
 * (which starts the picture over). */
static void process_input(Accumulator *a, int *spp, ShadeMode *mode,
                          bool *paused, int *frame_idx) {
  switch (getch()) {
  case 'q':
  case 'Q':
  case 27 /* Esc */:
    g_run = 0;
    break;
  case ' ':
  case 'p':
  case 'P':
    *paused = !*paused;
    break;
  case 'r':
  case 'R':
    restart_convergence(a, frame_idx);
    break;
  case '+':
  case '=':
    if (*spp < SPP_MAX)
      (*spp)++;
    restart_convergence(a, frame_idx);
    break;
  case '-':
  case '_':
    if (*spp > SPP_MIN)
      (*spp)--;
    restart_convergence(a, frame_idx);
    break;
  case 'd':
  case 'D':
    *mode = (ShadeMode)((*mode + 1) % MODE_N);
    restart_convergence(a, frame_idx);
    break;
  default:
    break;
  }
}

int main(void) {
  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);
  signal(SIGWINCH, on_sigwinch);
  atexit(cleanup);

  screen_init();
  int cols, rows;
  getmaxyx(stdscr, rows, cols);
  color_init();
  accum_reset(&g_accum);

  int spp = SPP_DEFAULT;
  bool paused = false;
  ShadeMode mode = MODE_PT;
  float fps = 0.f;
  long long fps_acc = 0;
  int fps_cnt = 0;
  long long frame_ns = NS_PER_SEC / TARGET_FPS;
  long long last = clock_ns();
  int frame_idx = 0;

  while (g_run) {

    /* deal with a resize if one happened */
    if (g_resize)
      apply_resize(&cols, &rows, &g_accum, &frame_idx);

    /* how long since last frame (clamped, so one slow frame can't snowball) */
    long long now = clock_ns();
    long long dt = now - last;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    last = now;

    /* update the fps readout a couple of times a second */
    fps_acc += dt;
    fps_cnt++;
    if (fps_acc >= FPS_WINDOW_NS) {
      fps = (float)fps_cnt * (float)NS_PER_SEC / (float)fps_acc;
      fps_acc = 0;
      fps_cnt = 0;
    }

    /* add another round of rays into the picture (unless we're holding) */
    if (should_accumulate(mode, g_accum.samples, paused))
      accum_add_frame(&g_accum, &k_scene, &k_camera, cols, rows, spp,
                      frame_idx++, mode);

    /* draw it; time the frame from here so the cap below covers draw + input */
    long long frame_start = clock_ns();
    render_frame(&g_accum, cols, rows, fps, spp, mode, paused);

    /* react to any keypress */
    process_input(&g_accum, &spp, &mode, &paused, &frame_idx);

    /* sleep off the rest of the frame so we don't run hot */
    clock_sleep_ns(frame_ns - (clock_ns() - frame_start));
  }
  return 0;
}
