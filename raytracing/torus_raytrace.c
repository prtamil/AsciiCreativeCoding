/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * torus_raytrace.c — analytic ray-traced torus (quartic intersection)
 *
 * Torus lies in the XZ plane (ring is horizontal).  The ray-torus equation
 * reduces to a quartic in t; roots are found by sampling + bisection (no
 * Ferrari formula needed).  Torus rotates around Y; camera elevated so the
 * hole and tube are both visible.
 *
 * Physics: (√(x²+z²)−R)² + y² = r²
 *   Quartic: t⁴ + At³ + Bt² + Ct + D = 0
 *   coefficients derived by substituting P = ro + t·rd and squaring.
 *
 * Modes  (s):  phong · normals · fresnel · depth
 * Themes (t):  titanium · solar · cobalt · forest · rose · chrome
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/torus_raytrace.c \
 *       -o torus_rt -lncurses -lm
 *
 * Keys:
 *   s         cycle shade mode
 *   t         cycle theme
 *   p         pause / resume
 *   + / =     zoom in
 *   -         zoom out
 *   r         reset rotation
 *   q / ESC   quit
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic ray-torus intersection (quartic polynomial root-finding).
 *                  Substituting ray ro + t·rd into the torus equation
 *                  (√(x²+z²)−R)² + y² = r² produces a degree-4 polynomial in t.
 *                  The full quartic is found by algebra, then solved numerically.
 *
 * Math           : Quartic: t⁴ + At³ + Bt² + Ct + D = 0.
 *                  Solution: evaluate the polynomial at sample points, then
 *                  bisect to find roots (avoids unstable Ferrari formula).
 *                  Sample count vs. accuracy trade-off: more samples find all
 *                  real roots but at higher cost.
 *                  Surface normal at hit point p on torus with major radius R:
 *                    N = normalise(p − R · normalise(p.xz × (0,1)))
 *
 * Rendering      : Phong + Fresnel (same as sphere_raytrace.c), showing that
 *                  the shading pipeline is decoupled from the intersection test.
 *                  The quartic solver is the only torus-specific code.
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
#define ASPECT        0.47f
#define FOV_DEG       52.0f
#define TORUS_R       0.68f      /* major radius: ring centre               */
#define TORUS_r       0.28f      /* minor radius: tube cross-section        */
#define ROT_Y         0.40f      /* ring spin (rad/s)                       */
#define ROT_X         0.18f      /* slow tilt (rad/s)                       */
#define CAM_DIST_DEF  3.4f
#define CAM_DIST_MIN  1.6f
#define CAM_DIST_MAX  7.0f
#define CAM_DIST_STEP 0.25f
#define CAM_HEIGHT    1.8f       /* elevation: see ring hole from above     */
#define AMBIENT       0.04f
#define SHININESS     60.0f

/* Quartic root finder parameters */
#define Q_SAMPLES     256        /* scan steps over t ∈ [ε, Q_T_MAX]        */
#define Q_BISECT      40         /* bisection iterations (< 1e-6 precision) */
#define Q_T_MAX       18.f

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

typedef struct { V3 r[3]; } Mat3;

static Mat3 mat3_rot(float rx, float ry)
{
    float cx=cosf(rx),sx=sinf(rx),cy=cosf(ry),sy=sinf(ry);
    Mat3 m;
    m.r[0]=(V3){  cy,    0.f,   sy    };
    m.r[1]=(V3){  sx*sy, cx,   -sx*cy };
    m.r[2]=(V3){ -cx*sy, sx,    cx*cy };
    return m;
}
static V3 mat3_mul (Mat3 m,V3 v){ return (V3){v3dot(m.r[0],v),v3dot(m.r[1],v),v3dot(m.r[2],v)}; }
static V3 mat3_mulT(Mat3 m,V3 v)
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
    /* titanium — cool silver */
    { {0.72f,0.72f,0.78f},{1.00f,1.00f,1.00f},
      {1.00f,0.96f,0.88f},{0.22f,0.32f,0.80f},{0.82f,0.88f,1.00f},"titanium" },
    /* solar — molten gold-orange */
    { {0.95f,0.62f,0.08f},{1.00f,0.90f,0.55f},
      {1.00f,0.90f,0.65f},{0.35f,0.12f,0.08f},{1.00f,0.55f,0.10f},"solar" },
    /* cobalt — electric blue */
    { {0.08f,0.32f,0.92f},{0.55f,0.75f,1.00f},
      {0.78f,0.88f,1.00f},{0.04f,0.08f,0.55f},{0.28f,0.58f,1.00f},"cobalt" },
    /* forest — deep organic green */
    { {0.12f,0.58f,0.22f},{0.50f,1.00f,0.58f},
      {0.68f,1.00f,0.50f},{0.04f,0.28f,0.42f},{0.15f,0.80f,0.35f},"forest" },
    /* rose — warm pink-red */
    { {0.90f,0.28f,0.48f},{1.00f,0.78f,0.82f},
      {1.00f,0.85f,0.72f},{0.30f,0.05f,0.25f},{1.00f,0.35f,0.55f},"rose" },
    /* chrome — high-contrast silver */
    { {0.82f,0.82f,0.88f},{1.00f,1.00f,1.00f},
      {1.00f,1.00f,1.00f},{0.15f,0.20f,0.50f},{0.92f,0.92f,1.00f},"chrome" },
};
#define THEME_N (int)(sizeof g_themes / sizeof g_themes[0])

static int g_256;

static void color_init(void)
{
    start_color(); use_default_colors();
    g_256 = (COLORS >= 256);
    if (g_256)
        for (int i=0;i<216;i++) init_pair(i+1,16+i,-1);
    init_pair(217, 15, -1);
}

static void draw_color(int row, int col, V3 c, float lum)
{
    lum = lum<0.f?0.f:lum>1.f?1.f:lum;
    char ch = k_ramp[(int)(lum*(RAMP_LEN-1))];
    if (g_256) {
        int r5=(int)(c.x*5.f+.5f); if(r5>5)r5=5;
        int g5=(int)(c.y*5.f+.5f); if(g5>5)g5=5;
        int b5=(int)(c.z*5.f+.5f); if(b5>5)b5=5;
        int pair=r5*36+g5*6+b5+1;
        attron(COLOR_PAIR(pair)); mvaddch(row,col,ch); attroff(COLOR_PAIR(pair));
    } else {
        mvaddch(row,col,ch);
    }
}

/* ── §5  ray-torus intersection ──────────────────────────────────────────── */

/*
 * Torus in XZ plane centred at origin.  Equation:
 *   (√(x²+z²) − R)² + y² = r²
 *
 * Substituting P(t) = ro + t·rd and squaring to remove the radical yields:
 *   t⁴ + A·t³ + B·t² + C·t + D = 0
 * where (with C0 = |ro|² + R² − r²):
 *   A = 4·(rd·ro)
 *   B = 4·(rd·ro)² + 2·C0 − 4·R²·(rdx²+rdz²)
 *   C = 4·(rd·ro)·C0 − 8·R²·(rdx·rox+rdz·roz)
 *   D = C0² − 4·R²·(rox²+roz²)
 *
 * Roots found by scanning [ε, Q_T_MAX] for sign changes then bisecting.
 * Horner's method for efficient polynomial evaluation.
 */

static inline float q_eval(float t, float A, float B, float C, float D)
{
    /* t⁴ + At³ + Bt² + Ct + D via Horner */
    return t*(t*(t*(t + A) + B) + C) + D;
}

static int ray_torus(V3 ro, V3 rd, float R, float r_minor, float *t_hit)
{
    float po2   = v3dot(ro, ro);
    float rod   = v3dot(rd, ro);
    float rxz2  = ro.x*ro.x + ro.z*ro.z;
    float rdxz_d= rd.x*ro.x + rd.z*ro.z;
    float rdxz2 = rd.x*rd.x + rd.z*rd.z;
    float C0    = po2 + R*R - r_minor*r_minor;

    float A = 4.f * rod;
    float B = 4.f*rod*rod + 2.f*C0 - 4.f*R*R*rdxz2;
    float C = 4.f*rod*C0  - 8.f*R*R*rdxz_d;
    float D = C0*C0        - 4.f*R*R*rxz2;

    float dt  = Q_T_MAX / Q_SAMPLES;
    float t0  = 1e-3f;
    float f0  = q_eval(t0, A, B, C, D);

    for (int i = 1; i <= Q_SAMPLES; i++) {
        float t1 = (float)i * dt;
        float f1 = q_eval(t1, A, B, C, D);

        if (f0 * f1 < 0.f) {
            /* sign change → bisect */
            float lo = t0, hi = t1, flo = f0;
            for (int j = 0; j < Q_BISECT; j++) {
                float mid  = (lo + hi) * .5f;
                float fmid = q_eval(mid, A, B, C, D);
                if (flo * fmid < 0.f) { hi = mid; }
                else                  { lo = mid; flo = fmid; }
            }
            *t_hit = (lo + hi) * .5f;
            return 1;
        }
        t0 = t1; f0 = f1;
    }
    return 0;
}

/*
 * Outward surface normal at hit point P on torus in XZ plane.
 * Nearest point on ring centreline: project P to XZ, normalize, scale by R.
 * Normal = P − ring_point.
 */
static V3 torus_normal(V3 P, float R)
{
    V3    P_xz    = { P.x, 0.f, P.z };
    float rho     = v3len(P_xz);
    V3    ring_pt = rho > 1e-9f ? v3scale(R / rho, P_xz) : (V3){R, 0.f, 0.f};
    return v3norm(v3sub(P, ring_pt));
}

/* ── §6  shading ─────────────────────────────────────────────────────────── */

typedef enum { MODE_PHONG=0, MODE_NORMAL, MODE_FRESNEL, MODE_DEPTH, MODE_N } ShadeMode;

static const V3 L_KEY  = { 3.0f, 4.0f,-2.0f };
static const V3 L_FILL = {-4.0f, 1.5f,-1.0f };
static const V3 L_RIM  = { 0.5f,-1.0f, 5.0f };

static V3 shade_phong(V3 P, V3 N, V3 V_dir, const Theme *th)
{
    V3 col = v3scale(AMBIENT, th->obj);
    {
        V3    L = v3norm(v3sub(L_KEY, P));
        float d = fmaxf(0.f, v3dot(N,L));
        V3    R = v3reflect(v3scale(-1.f,L), N);
        float s = powf(fmaxf(0.f, v3dot(R,V_dir)), SHININESS);
        col = v3add(col, v3scale(d*.65f, v3mul(th->obj,th->key_col)));
        col = v3add(col, v3scale(s*.55f, th->spec));
    }
    {
        V3    L = v3norm(v3sub(L_FILL, P));
        float d = fmaxf(0.f, v3dot(N,L));
        col = v3add(col, v3scale(d*.22f, v3mul(th->obj,th->fill_col)));
    }
    {
        V3    L = v3norm(v3sub(L_RIM, P));
        float d = fmaxf(0.f, v3dot(N,L));
        V3    R = v3reflect(v3scale(-1.f,L), N);
        float s = powf(fmaxf(0.f, v3dot(R,V_dir)), 10.f);
        col = v3add(col, v3scale(d*.18f, v3mul(th->obj,th->rim_col)));
        col = v3add(col, v3scale(s*.65f, th->rim_col));
    }
    return v3clamp1(col);
}

static V3 shade_normal(V3 N)
{ return (V3){ N.x*.5f+.5f, N.y*.5f+.5f, N.z*.5f+.5f }; }

static V3 shade_fresnel(V3 N, V3 V_dir, const Theme *th)
{
    float cosA    = fabsf(v3dot(N, V_dir));
    float inv     = 1.f - cosA;
    float fresnel = inv*inv*inv*inv*inv;
    V3 core = v3scale(0.06f, th->obj);
    V3 edge = v3clamp1(v3add(v3scale(.7f,th->spec), v3scale(.5f,th->rim_col)));
    return v3clamp1(v3add(v3scale(1.f-fresnel,core), v3scale(fresnel,edge)));
}

static V3 shade_depth(float t, float t_max, const Theme *th)
{
    float d = 1.f - fminf(t/t_max, 1.f); d = d*d;
    return v3clamp1(v3scale(d, th->obj));
}

/* ── §7  render frame ────────────────────────────────────────────────────── */

static void render(int cols, int rows, float angle_x, float angle_y,
                   float cam_dist, int theme, ShadeMode mode)
{
    const Theme *th = &g_themes[theme % THEME_N];
    float fov_tan   = tanf(FOV_DEG * (float)M_PI / 360.f);
    Mat3  M         = mat3_rot(angle_x, angle_y);

    /* Camera elevated, looking at origin */
    V3 cam  = { 0.f, CAM_HEIGHT, -cam_dist };
    V3 fwd  = v3norm(v3sub((V3){0,0,0}, cam));
    V3 wup  = { 0.f, 1.f, 0.f };
    V3 rgt  = v3norm(v3cross(fwd, wup));
    V3 up   = v3cross(rgt, fwd);

    float cx = cols * .5f, cy = rows * .5f;

    for (int row = 0; row < rows-1; row++) {
        for (int col = 0; col < cols; col++) {
            float pu = (col - cx) / cx * fov_tan;
            float pv = -(row - cy) / cx * fov_tan / ASPECT;
            V3 rd_ws = v3norm(v3add(fwd, v3add(v3scale(pu,rgt), v3scale(pv,up))));

            /* Ray in object space */
            V3 ro_os = mat3_mulT(M, cam);
            V3 rd_os = mat3_mulT(M, rd_ws);

            float t_hit;
            if (!ray_torus(ro_os, rd_os, TORUS_R, TORUS_r, &t_hit)) continue;

            V3 P_os  = v3add(ro_os, v3scale(t_hit, rd_os));
            V3 P_ws  = v3add(cam,   v3scale(t_hit, rd_ws));
            V3 N_os  = torus_normal(P_os, TORUS_R);
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
                color = shade_depth(t_hit, cam_dist * 2.5f, th);
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
        "  [s]mode [t]theme [+/-]zoom [p]pause [r]reset [q]quit ",
        (double)fps, (double)cam_dist,
        g_themes[theme%THEME_N].name, mnames[mode]);
    attroff(COLOR_PAIR(217));
}

/* ── §9  main ────────────────────────────────────────────────────────────── */

int main(void)
{
    signal(SIGINT, on_sigint); signal(SIGWINCH, on_sigwinch);
    initscr(); cbreak(); noecho(); curs_set(0);
    keypad(stdscr,TRUE); nodelay(stdscr,TRUE); typeahead(-1);

    int cols, rows;
    getmaxyx(stdscr, rows, cols);
    color_init();

    int       theme    = 0;
    ShadeMode mode     = MODE_PHONG;
    float     cam_dist = CAM_DIST_DEF;
    float     angle_x  = 0.f, angle_y = 0.f;
    int       paused   = 0;
    float     fps      = 0.f;
    long long fps_acc  = 0;
    int       fps_cnt  = 0;
    long long frame_ns = 1000000000LL / TARGET_FPS;
    long long last     = clock_ns();

    while (g_run) {
        if (g_resize) {
            g_resize=0; endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
        }
        long long now=clock_ns(), dt=now-last;
        if (dt>100000000LL) dt=100000000LL;
        last = now;

        if (!paused) {
            float sec = (float)dt * 1e-9f;
            angle_y += ROT_Y * sec;
            angle_x += ROT_X * sec;
        }

        fps_acc += dt; fps_cnt++;
        if (fps_acc >= 500000000LL) {
            fps=(float)fps_cnt*1e9f/(float)fps_acc;
            fps_acc=0; fps_cnt=0;
        }

        long long t0 = clock_ns();
        erase();
        render(cols, rows, angle_x, angle_y, cam_dist, theme, mode);
        screen_hud(cols, rows, fps, theme, mode, cam_dist);
        wnoutrefresh(stdscr); doupdate();

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_run=0;                             break;
        case 'p': case 'P': paused=!paused;                               break;
        case 'r': case 'R': angle_x=0.f; angle_y=0.f;                    break;
        case 's': case 'S': mode=(ShadeMode)((mode+1)%MODE_N);           break;
        case 't': case 'T': theme=(theme+1)%THEME_N;                     break;
        case '+': case '=':
            cam_dist-=CAM_DIST_STEP; if(cam_dist<CAM_DIST_MIN)cam_dist=CAM_DIST_MIN; break;
        case '-':
            cam_dist+=CAM_DIST_STEP; if(cam_dist>CAM_DIST_MAX)cam_dist=CAM_DIST_MAX; break;
        }
        clock_sleep_ns(frame_ns - (clock_ns()-t0));
    }
    endwin();
    return 0;
}
