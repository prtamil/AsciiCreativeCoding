/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * capsule_raytrace.c — analytic ray-traced capsule
 *
 * A capsule is a cylinder capped at both ends with hemispheres.  The ray
 * is transformed into object space (same inverse-rotation trick as the
 * cube and torus), then intersected analytically:
 *
 *   1. Infinite cylinder body  — quadratic in t after removing axial component
 *   2. Hemisphere caps         — sphere quadratic at each endpoint
 *
 * The body normal points radially outward from the axis.
 * The cap normal points away from the endpoint centre.
 *
 * Modes  (s):  phong · normals · fresnel · depth
 * Themes (t):  bronze · frost · ember · pine · dusk · pearl
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/capsule_raytrace.c \
 *       -o capsule_rt -lncurses -lm
 *
 * Keys:
 *   s         cycle shade mode
 *   t         cycle theme
 *   p         pause / resume
 *   + / =     zoom in   (closer)
 *   -         zoom out  (farther)
 *   q / ESC   quit
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic ray-capsule intersection — decomposed into
 *                  two sub-problems:
 *                  1. Infinite cylinder test (quadratic in t after projecting
 *                     out the axial component of the ray/capsule vectors).
 *                  2. Hemisphere cap tests (sphere quadratics at each endpoint).
 *                  The minimum positive t among valid hits is the surface hit.
 *
 * Math           : Cylinder intersection: project ray and cylinder axis onto
 *                  the plane perpendicular to the axis.  The resulting 2D
 *                  ray-circle problem is a standard quadratic.
 *                  Axial bounds check: the body t is only valid if the hit
 *                  point's axial projection falls within [0, height].
 *                  Cap normals: (p − endpoint) / r (sphere normal at each cap).
 *
 * Rendering      : Mode cycling (phong / normals / fresnel / depth) shows how
 *                  the same intersection test feeds different shading algorithms.
 *                  Normal mode renders N as an RGB vector — a diagnostic tool
 *                  for verifying correct normal orientation at body/cap boundaries.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 199309L
#include <ncurses.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ── §1  config ─────────────────────────────────────────────────────────── */

#define TARGET_FPS    60
#define ASPECT        0.47f      /* terminal cell width / height             */
#define FOV_DEG       55.0f

/* Capsule geometry (object space, axis along Y) */
#define CAP_HALF_H    0.65f      /* half-length of cylinder body             */
#define CAP_R         0.35f      /* tube radius (caps share same radius)     */

/* Rotation speeds */
#define ROT_Y         0.45f      /* radians / second — spin around Y axis    */
#define ROT_X         0.22f      /* radians / second — slow tilt             */

/* Camera */
#define CAM_DIST_DEF  3.4f
#define CAM_DIST_MIN  1.8f
#define CAM_DIST_MAX  7.0f
#define CAM_DIST_STEP 0.25f

#define AMBIENT       0.05f
#define SHININESS     48.0f

static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN (int)(sizeof k_ramp - 1)

/* ── §2  clock ──────────────────────────────────────────────────────────── */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3  V3 / Mat3 math ──────────────────────────────────────────────────── */

typedef struct { float x, y, z; } V3;

static inline V3    v3add   (V3 a,V3 b)    { return (V3){a.x+b.x,a.y+b.y,a.z+b.z}; }
static inline V3    v3sub   (V3 a,V3 b)    { return (V3){a.x-b.x,a.y-b.y,a.z-b.z}; }
static inline V3    v3scale (float s,V3 a) { return (V3){s*a.x,s*a.y,s*a.z}; }
static inline V3    v3mul   (V3 a,V3 b)    { return (V3){a.x*b.x,a.y*b.y,a.z*b.z}; }
static inline float v3dot   (V3 a,V3 b)    { return a.x*b.x+a.y*b.y+a.z*b.z; }
static inline float v3len   (V3 a)         { return sqrtf(v3dot(a,a)); }
static inline V3    v3norm  (V3 a)         { float l=v3len(a); return l>1e-9f?v3scale(1.f/l,a):(V3){0,1,0}; }
static inline V3    v3cross (V3 a,V3 b)    { return (V3){a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
static inline V3    v3reflect(V3 v,V3 n)   { return v3sub(v,v3scale(2.f*v3dot(v,n),n)); }
static inline V3    v3clamp1(V3 v)
{ return (V3){v.x<0?0:v.x>1?1:v.x,v.y<0?0:v.y>1?1:v.y,v.z<0?0:v.z>1?1:v.z}; }

/* 3×3 rotation matrix — rows stored as V3 */
typedef struct { V3 r[3]; } Mat3;

/*
 * M = Rx(rx) · Ry(ry): object → world.
 * Transpose M^T transforms world → object (inverse for orthogonal matrix).
 */
static Mat3 mat3_rot(float rx, float ry)
{
    float cx=cosf(rx),sx=sinf(rx),cy=cosf(ry),sy=sinf(ry);
    Mat3 m;
    m.r[0] = (V3){  cy,      0.f,   sy    };
    m.r[1] = (V3){  sx*sy,   cx,   -sx*cy };
    m.r[2] = (V3){ -cx*sy,   sx,    cx*cy };
    return m;
}
/* M · v  (object → world) */
static V3 mat3_mul(Mat3 m, V3 v)
{ return (V3){v3dot(m.r[0],v),v3dot(m.r[1],v),v3dot(m.r[2],v)}; }

/* M^T · v  (world → object) */
static V3 mat3_mulT(Mat3 m, V3 v)
{
    return (V3){
        m.r[0].x*v.x+m.r[1].x*v.y+m.r[2].x*v.z,
        m.r[0].y*v.x+m.r[1].y*v.y+m.r[2].y*v.z,
        m.r[0].z*v.x+m.r[1].z*v.y+m.r[2].z*v.z
    };
}

/* ── §4  color / themes ──────────────────────────────────────────────────── */

typedef struct {
    V3 obj; V3 spec; V3 key_col; V3 fill_col; V3 rim_col;
    const char *name;
} Theme;

static const Theme g_themes[] = {
    /* bronze — warm antique gold-brown */
    { {0.80f,0.50f,0.18f},{1.00f,0.88f,0.55f},
      {1.00f,0.90f,0.60f},{0.30f,0.20f,0.60f},{0.90f,0.45f,0.08f},"bronze" },
    /* frost — cold pale blue-white */
    { {0.72f,0.88f,1.00f},{0.92f,0.96f,1.00f},
      {0.80f,0.90f,1.00f},{0.12f,0.20f,0.70f},{0.55f,0.82f,1.00f},"frost" },
    /* ember — deep orange-red with bright core */
    { {0.92f,0.38f,0.08f},{1.00f,0.82f,0.45f},
      {1.00f,0.78f,0.38f},{0.25f,0.05f,0.45f},{1.00f,0.28f,0.05f},"ember" },
    /* pine — deep forest green */
    { {0.12f,0.58f,0.22f},{0.50f,0.92f,0.55f},
      {0.65f,1.00f,0.50f},{0.05f,0.28f,0.50f},{0.10f,0.80f,0.30f},"pine" },
    /* dusk — soft purple-mauve */
    { {0.55f,0.25f,0.75f},{0.88f,0.72f,1.00f},
      {0.90f,0.80f,1.00f},{0.10f,0.05f,0.55f},{0.70f,0.25f,0.90f},"dusk" },
    /* pearl — warm cream with soft highlights */
    { {0.92f,0.88f,0.80f},{1.00f,0.98f,0.92f},
      {1.00f,0.95f,0.80f},{0.30f,0.35f,0.70f},{0.95f,0.80f,0.55f},"pearl" },
};
#define THEME_N (int)(sizeof g_themes / sizeof g_themes[0])

static int g_256;

static void color_init(void)
{
    start_color();
    use_default_colors();
    g_256 = (COLORS >= 256);
    if (g_256)
        for (int i = 0; i < 216; i++)
            init_pair(i + 1, 16 + i, -1);
    init_pair(217, 15, -1);   /* HUD white */
}

static void draw_color(int row, int col, V3 c, float lum)
{
    lum = lum < 0.f ? 0.f : lum > 1.f ? 1.f : lum;
    char ch = k_ramp[(int)(lum * (RAMP_LEN - 1))];
    if (g_256) {
        int r5=(int)(c.x*5.f+.5f); if(r5>5)r5=5;
        int g5=(int)(c.y*5.f+.5f); if(g5>5)g5=5;
        int b5=(int)(c.z*5.f+.5f); if(b5>5)b5=5;
        int pair = r5*36 + g5*6 + b5 + 1;
        attron(COLOR_PAIR(pair));
        mvaddch(row, col, ch);
        attroff(COLOR_PAIR(pair));
    } else {
        mvaddch(row, col, ch);
    }
}

/* ── §5  ray–capsule intersection ────────────────────────────────────────── */

/*
 * Capsule: cylinder body from ca to cb with hemispherical caps, radius r.
 * Axis direction: ba = cb − ca.
 *
 * Derivation (Íñigo Quílez method):
 *   Project out the axial component of the ray to get a 2D cylinder problem.
 *   Let ba = cb−ca, oa = ro−ca.
 *     a = |ba|² − (ba·rd)²          (rd length² minus axial projection²)
 *     b = |ba|²(rd·oa) − (ba·oa)(ba·rd)
 *     c = |ba|²(|oa|²−r²) − (ba·oa)²
 *   Discriminant h = b²−ac.  If h<0: miss the infinite cylinder entirely.
 *   t_body = (−b−√h)/a
 *   y_body = ba·oa + t_body·(ba·rd)   ← signed distance along axis
 *   Hit body if 0 < y_body < |ba|².
 *   Otherwise test the nearer cap as a sphere centered at ca (y≤0) or cb (y≥|ba|²).
 *
 * Normals:
 *   Body: N = normalize( (oa + t·rd) − (y/|ba|²)·ba )
 *             i.e. P−ca minus its axial projection → radially outward.
 *   Cap:  N = normalize( oc + t·rd )
 *             where oc = ro−ca or ro−cb.  Points away from cap centre.
 *
 * Returns 1 on hit.  Sets *t_hit and *N_os.
 */
static int ray_capsule(V3 ro, V3 rd,
                       V3 ca, V3 cb, float r,
                       float *t_hit, V3 *N_os)
{
    V3    ba   = v3sub(cb, ca);
    V3    oa   = v3sub(ro, ca);
    float baba = v3dot(ba, ba);          /* |ba|²                          */
    float bard = v3dot(ba, rd);          /* axial component of ray dir     */
    float baoa = v3dot(ba, oa);          /* signed axial offset of origin  */
    float rdoa = v3dot(rd, oa);
    float oaoa = v3dot(oa, oa);

    float a = baba - bard * bard;
    float b = baba * rdoa - baoa * bard;
    float c = baba * (oaoa - r * r) - baoa * baoa;
    float h = b * b - a * c;
    if (h < 0.f) return 0;              /* misses infinite cylinder         */

    /* ── cylinder body ── */
    float t = (-b - sqrtf(h)) / a;
    float y = baoa + t * bard;
    if (t > 1e-4f && y > 0.f && y < baba) {
        /* radial outward normal: strip axial component from (P − ca) */
        V3 p_minus_ca = v3add(oa, v3scale(t, rd));
        *N_os  = v3norm(v3sub(p_minus_ca, v3scale(y / baba, ba)));
        *t_hit = t;
        return 1;
    }

    /* ── caps: test the nearer hemisphere ── */
    V3 oc = (y <= 0.f) ? oa : v3sub(oa, ba);   /* ro relative to cap centre */
    b = v3dot(rd, oc);
    c = v3dot(oc, oc) - r * r;
    h = b * b - c;
    if (h < 0.f) return 0;
    t = -b - sqrtf(h);
    if (t < 1e-4f) {
        t = -b + sqrtf(h);
        if (t < 1e-4f) return 0;
    }
    *t_hit = t;
    *N_os  = v3norm(v3add(oc, v3scale(t, rd)));   /* away from cap centre */
    return 1;
}

/* ── §6  shading ─────────────────────────────────────────────────────────── */

typedef enum { MODE_PHONG=0, MODE_NORMAL, MODE_FRESNEL, MODE_DEPTH, MODE_N } ShadeMode;

/* Three fixed world-space lights */
static const V3 L_KEY  = { 3.0f, 4.0f,-2.0f };
static const V3 L_FILL = {-4.0f, 1.0f,-1.0f };
static const V3 L_RIM  = { 0.5f,-1.0f, 5.0f };

static V3 shade_phong(V3 P, V3 N, V3 V_dir, const Theme *th)
{
    V3 col = v3scale(AMBIENT, th->obj);

    /* key — strong diffuse + sharp specular */
    {
        V3    L = v3norm(v3sub(L_KEY, P));
        float d = fmaxf(0.f, v3dot(N, L));
        V3    R = v3reflect(v3scale(-1.f, L), N);
        float s = powf(fmaxf(0.f, v3dot(R, V_dir)), SHININESS);
        col = v3add(col, v3scale(d * 0.65f, v3mul(th->obj, th->key_col)));
        col = v3add(col, v3scale(s * 0.55f, th->spec));
    }
    /* fill — soft diffuse */
    {
        V3    L = v3norm(v3sub(L_FILL, P));
        float d = fmaxf(0.f, v3dot(N, L));
        col = v3add(col, v3scale(d * 0.22f, v3mul(th->obj, th->fill_col)));
    }
    /* rim — wide specular on back silhouette */
    {
        V3    L = v3norm(v3sub(L_RIM, P));
        float d = fmaxf(0.f, v3dot(N, L));
        V3    R = v3reflect(v3scale(-1.f, L), N);
        float s = powf(fmaxf(0.f, v3dot(R, V_dir)), 10.f);
        col = v3add(col, v3scale(d * 0.18f, v3mul(th->obj, th->rim_col)));
        col = v3add(col, v3scale(s * 0.65f, th->rim_col));
    }
    return v3clamp1(col);
}

static V3 shade_normal(V3 N)
{
    return (V3){ N.x*.5f+.5f, N.y*.5f+.5f, N.z*.5f+.5f };
}

/*
 * Fresnel / glass mode.
 * A capsule shows beautiful Fresnel behaviour: the cylinder band is dark at
 * head-on angles, and the rounded caps glow at grazing edges.
 */
static V3 shade_fresnel(V3 N, V3 V_dir, const Theme *th)
{
    float cosA    = fabsf(v3dot(N, V_dir));
    float inv     = 1.f - cosA;
    float fresnel = inv * inv * inv * inv * inv;   /* Schlick (1−cosθ)^5 */
    V3 core = v3scale(0.06f, th->obj);
    V3 edge = v3clamp1(v3add(v3scale(0.7f, th->spec), v3scale(0.5f, th->rim_col)));
    return v3clamp1(v3add(v3scale(1.f-fresnel, core), v3scale(fresnel, edge)));
}

static V3 shade_depth(float t, float t_max, const Theme *th)
{
    float d = 1.f - fminf(t / t_max, 1.f);
    d = d * d;
    return v3clamp1(v3scale(d, th->obj));
}

/* ── §7  render frame ────────────────────────────────────────────────────── */

/*
 * Camera is fixed on the Z axis looking toward the origin.
 * The capsule rotates: its object-space axis is always Y.
 * Ray is transformed into object space (M^T), intersection solved there,
 * normal transformed back to world space (M).
 */
static void render(int cols, int rows,
                   float angle_x, float angle_y, float cam_dist,
                   int theme, ShadeMode mode)
{
    const Theme *th  = &g_themes[theme % THEME_N];
    float fov_tan    = tanf(FOV_DEG * (float)M_PI / 360.f);

    Mat3 M   = mat3_rot(angle_x, angle_y);

    /* Fixed camera on −Z, looking at origin */
    V3 cam   = { 0.f, 0.f, -cam_dist };
    V3 fwd   = { 0.f, 0.f,  1.f };
    V3 rgt   = { 1.f, 0.f,  0.f };
    V3 up    = { 0.f, 1.f,  0.f };

    /* Capsule endpoints in object space */
    V3 ca_os = { 0.f, -CAP_HALF_H, 0.f };
    V3 cb_os = { 0.f,  CAP_HALF_H, 0.f };

    float cx = cols * .5f, cy = rows * .5f;

    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            float pu = (col - cx) / cx * fov_tan;
            float pv = -(row - cy) / cx * fov_tan / ASPECT;

            /* World-space ray */
            V3 rd_ws = v3norm(v3add(fwd, v3add(v3scale(pu, rgt), v3scale(pv, up))));

            /* Transform ray into object space (capsule stays fixed there) */
            V3 ro_os = mat3_mulT(M, cam);
            V3 rd_os = mat3_mulT(M, rd_ws);

            float t_hit;
            V3    N_os;
            if (!ray_capsule(ro_os, rd_os, ca_os, cb_os, CAP_R, &t_hit, &N_os))
                continue;

            /* Hit point in world space; normal back to world space */
            V3 P_ws  = v3add(cam, v3scale(t_hit, rd_ws));
            V3 N_ws  = mat3_mul(M, N_os);
            V3 V_dir = v3norm(v3sub(cam, P_ws));

            V3    color;
            float lum;

            switch (mode) {
            default:
            case MODE_PHONG:
                color = shade_phong(P_ws, N_ws, V_dir, th);
                lum   = 0.299f*color.x + 0.587f*color.y + 0.114f*color.z;
                break;
            case MODE_NORMAL:
                color = shade_normal(N_ws);
                lum   = (N_ws.x*.5f+.5f)*.3f+(N_ws.y*.5f+.5f)*.6f+(N_ws.z*.5f+.5f)*.1f;
                break;
            case MODE_FRESNEL:
                color = shade_fresnel(N_ws, V_dir, th);
                lum   = 0.299f*color.x + 0.587f*color.y + 0.114f*color.z;
                break;
            case MODE_DEPTH:
                color = shade_depth(t_hit, cam_dist * 2.2f, th);
                lum   = 0.299f*color.x + 0.587f*color.y + 0.114f*color.z;
                break;
            }

            draw_color(row, col, color, lum);
        }
    }
}

/* ── §8  screen / HUD ────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_run    = 1;
static volatile sig_atomic_t g_resize = 0;
static void on_sigint  (int s){ (void)s; g_run    = 0; }
static void on_sigwinch(int s){ (void)s; g_resize = 1; }

static void screen_hud(int cols, int rows, float fps,
                       int theme, ShadeMode mode, float cam_dist)
{
    (void)cols;
    static const char *const mnames[] = { "phong","normals","fresnel","depth" };
    attron(COLOR_PAIR(217));
    mvprintw(rows-1, 1,
        " fps:%.0f  dist:%.1f  theme:%s  mode:%s"
        "  [s]mode [t]theme [+/-]zoom [p]pause [q]quit ",
        (double)fps, (double)cam_dist,
        g_themes[theme % THEME_N].name, mnames[mode]);
    attroff(COLOR_PAIR(217));
}

/* ── §9  main ────────────────────────────────────────────────────────────── */

int main(void)
{
    signal(SIGINT, on_sigint); signal(SIGWINCH, on_sigwinch);
    initscr(); cbreak(); noecho(); curs_set(0);
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE); typeahead(-1);

    int cols, rows;
    getmaxyx(stdscr, rows, cols);
    color_init();

    int       theme    = 0;
    ShadeMode mode     = MODE_PHONG;
    float     cam_dist = CAM_DIST_DEF;
    float     angle_x  = 0.f;
    float     angle_y  = 0.f;
    int       paused   = 0;
    float     fps      = 0.f;
    long long fps_acc  = 0;
    int       fps_cnt  = 0;
    long long frame_ns = 1000000000LL / TARGET_FPS;
    long long last     = clock_ns();

    while (g_run) {
        if (g_resize) {
            g_resize = 0; endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
        }
        long long now = clock_ns();
        long long dt  = now - last; if (dt > 100000000LL) dt = 100000000LL;
        last = now;

        if (!paused) {
            angle_y += ROT_Y * (float)dt * 1e-9f;
            angle_x += ROT_X * (float)dt * 1e-9f;
        }

        fps_acc += dt; fps_cnt++;
        if (fps_acc >= 500000000LL) {
            fps = (float)fps_cnt * 1e9f / (float)fps_acc;
            fps_acc = 0; fps_cnt = 0;
        }

        long long t0 = clock_ns();
        erase();
        render(cols, rows, angle_x, angle_y, cam_dist, theme, mode);
        screen_hud(cols, rows, fps, theme, mode, cam_dist);
        wnoutrefresh(stdscr); doupdate();

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_run = 0;                             break;
        case 'p': case 'P': paused = !paused;                               break;
        case 's': case 'S': mode = (ShadeMode)((mode + 1) % MODE_N);       break;
        case 't': case 'T': theme = (theme + 1) % THEME_N;                 break;
        case '+': case '=':
            cam_dist -= CAM_DIST_STEP;
            if (cam_dist < CAM_DIST_MIN) cam_dist = CAM_DIST_MIN;
            break;
        case '-':
            cam_dist += CAM_DIST_STEP;
            if (cam_dist > CAM_DIST_MAX) cam_dist = CAM_DIST_MAX;
            break;
        }
        clock_sleep_ns(frame_ns - (clock_ns() - t0));
    }
    endwin();
    return 0;
}
