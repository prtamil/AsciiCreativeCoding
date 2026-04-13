/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cube_raytrace.c — analytic ray-traced cube (AABB slab method)
 *
 * Ray transformed to object space; intersects axis-aligned box via slab
 * method.  Hit face determined from which axis gave the entry t.  Object
 * tumbles in world space via Rx·Ry rotation applied as inverse to the ray.
 *
 * Modes  (s):  phong · normals · wireframe · depth
 * Themes (t):  iron · sapphire · copper · jade · obsidian · plasma
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/cube_raytrace.c \
 *       -o cube_rt -lncurses -lm
 *
 * Keys:
 *   s         cycle shade mode
 *   t         cycle theme
 *   p         pause / resume
 *   + / =     zoom in
 *   -         zoom out
 *   q / ESC   quit
 */

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
#define FOV_DEG       55.0f
#define CUBE_S        0.80f      /* half-extent: cube spans [-S,S]^3        */
#define WIRE_THRESH   0.055f     /* edge width as fraction of half-extent    */
#define ROT_Y         0.52f      /* radians / second                         */
#define ROT_X         0.35f
#define CAM_DIST_DEF  3.2f
#define CAM_DIST_MIN  1.5f
#define CAM_DIST_MAX  7.0f
#define CAM_DIST_STEP 0.25f
#define AMBIENT       0.05f
#define SHININESS     40.0f

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
 * Build rotation M = Rx(rx) · Ry(ry): object → world.
 * Applied as-is for M·v, transposed for M^T·v (world → object).
 */
static Mat3 mat3_rot(float rx, float ry)
{
    float cx=cosf(rx),sx=sinf(rx),cy=cosf(ry),sy=sinf(ry);
    Mat3 m;
    m.r[0] = (V3){  cy,       0.f,   sy      };
    m.r[1] = (V3){  sx*sy,    cx,   -sx*cy   };
    m.r[2] = (V3){ -cx*sy,    sx,    cx*cy   };
    return m;
}
/* M · v */
static V3 mat3_mul(Mat3 m, V3 v)
{ return (V3){v3dot(m.r[0],v),v3dot(m.r[1],v),v3dot(m.r[2],v)}; }

/* M^T · v  (world → object for orthogonal M) */
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
    /* iron — cool grey metallic */
    { {0.65f,0.65f,0.70f},{0.95f,0.95f,1.00f},
      {1.00f,0.95f,0.85f},{0.25f,0.35f,0.80f},{0.80f,0.85f,1.00f},"iron" },
    /* sapphire — deep blue */
    { {0.10f,0.25f,0.90f},{0.60f,0.75f,1.00f},
      {0.80f,0.88f,1.00f},{0.05f,0.10f,0.60f},{0.30f,0.55f,1.00f},"sapphire" },
    /* copper — warm orange-bronze */
    { {0.85f,0.42f,0.12f},{1.00f,0.82f,0.55f},
      {1.00f,0.88f,0.65f},{0.40f,0.15f,0.10f},{1.00f,0.50f,0.15f},"copper" },
    /* jade — rich green */
    { {0.12f,0.62f,0.32f},{0.55f,1.00f,0.65f},
      {0.70f,1.00f,0.55f},{0.05f,0.30f,0.45f},{0.18f,0.85f,0.40f},"jade" },
    /* obsidian — near-black, bright specular */
    { {0.08f,0.06f,0.10f},{0.90f,0.82f,1.00f},
      {1.00f,0.95f,0.90f},{0.05f,0.05f,0.20f},{0.75f,0.50f,1.00f},"obsidian" },
    /* plasma — neon violet */
    { {0.55f,0.05f,0.90f},{0.95f,0.70f,1.00f},
      {0.90f,0.75f,1.00f},{0.08f,0.05f,0.55f},{0.85f,0.15f,1.00f},"plasma" },
};
#define THEME_N (int)(sizeof g_themes / sizeof g_themes[0])

static int g_256;

static void color_init(void)
{
    start_color(); use_default_colors();
    g_256 = (COLORS >= 256);
    if (g_256)
        for (int i = 0; i < 216; i++)
            init_pair(i+1, 16+i, -1);
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
        int pair = r5*36+g5*6+b5+1;
        attron(COLOR_PAIR(pair)); mvaddch(row,col,ch); attroff(COLOR_PAIR(pair));
    } else {
        mvaddch(row,col,ch);
    }
}

/* ── §5  ray-AABB intersection (slab method) ─────────────────────────────── */

/*
 * Box: [-s, s]^3 in object space.
 * Sets *t_hit and *N_os (object-space normal of the hit face).
 * Returns 1 on hit.
 */
static int ray_aabb(V3 ro, V3 rd, float s, float *t_hit, V3 *N_os)
{
    float tmin = -1e30f, tmax = 1e30f;
    int   near_ax = -1;
    float near_sg = 0.f;

    float ro_a[3] = {ro.x, ro.y, ro.z};
    float rd_a[3] = {rd.x, rd.y, rd.z};

    for (int i = 0; i < 3; i++) {
        float inv;
        if (fabsf(rd_a[i]) < 1e-9f) {
            if (ro_a[i] < -s || ro_a[i] > s) return 0;
            continue;
        }
        inv = 1.f / rd_a[i];
        float t0 = (-s - ro_a[i]) * inv;
        float t1 = ( s - ro_a[i]) * inv;
        float tn = t0 < t1 ? t0 : t1;
        float tf = t0 < t1 ? t1 : t0;
        if (tn > tmin) {
            tmin    = tn;
            near_ax = i;
            /* outward normal: −sign(rd) on entry face */
            near_sg = (rd_a[i] > 0.f) ? -1.f : 1.f;
        }
        if (tf < tmax) tmax = tf;
        if (tmin > tmax) return 0;
    }

    if (tmax < 1e-4f || near_ax < 0) return 0;
    float t = tmin > 1e-4f ? tmin : tmax;
    if (t < 1e-4f) return 0;

    *t_hit = t;
    *N_os  = (V3){0,0,0};
    if      (near_ax == 0) N_os->x = near_sg;
    else if (near_ax == 1) N_os->y = near_sg;
    else                   N_os->z = near_sg;

    return 1;
}

/*
 * Distance to nearest face edge in object space (for wireframe mode).
 * Returns 0 at edge, 1 at face centre.
 */
static float face_edge_dist(V3 P, V3 N, float s)
{
    float u, v;
    if      (fabsf(N.x) > .5f) { u = P.y; v = P.z; }
    else if (fabsf(N.y) > .5f) { u = P.x; v = P.z; }
    else                        { u = P.x; v = P.y; }
    float du = s - fabsf(u);
    float dv = s - fabsf(v);
    return fminf(du, dv) / s;
}

/* ── §6  shading ─────────────────────────────────────────────────────────── */

typedef enum { MODE_PHONG=0, MODE_NORMAL, MODE_WIRE, MODE_DEPTH, MODE_N } ShadeMode;

static const V3 L_KEY  = { 3.0f, 4.0f,-2.0f };
static const V3 L_FILL = {-4.0f, 1.0f,-1.0f };
static const V3 L_RIM  = { 0.5f,-1.0f, 5.0f };

static V3 shade_phong(V3 P, V3 N, V3 V_dir, const Theme *th)
{
    V3 col = v3scale(AMBIENT, th->obj);
    {
        V3    L = v3norm(v3sub(L_KEY, P));
        float d = fmaxf(0.f, v3dot(N,L));
        V3    R = v3reflect(v3scale(-1.f,L), N);
        float s = powf(fmaxf(0.f, v3dot(R,V_dir)), SHININESS);
        col = v3add(col, v3scale(d*.65f, v3mul(th->obj, th->key_col)));
        col = v3add(col, v3scale(s*.50f, th->spec));
    }
    {
        V3    L = v3norm(v3sub(L_FILL, P));
        float d = fmaxf(0.f, v3dot(N,L));
        col = v3add(col, v3scale(d*.22f, v3mul(th->obj, th->fill_col)));
    }
    {
        V3    L = v3norm(v3sub(L_RIM, P));
        float d = fmaxf(0.f, v3dot(N,L));
        V3    R = v3reflect(v3scale(-1.f,L), N);
        float s = powf(fmaxf(0.f, v3dot(R,V_dir)), 10.f);
        col = v3add(col, v3scale(d*.18f, v3mul(th->obj, th->rim_col)));
        col = v3add(col, v3scale(s*.60f, th->rim_col));
    }
    return v3clamp1(col);
}

static V3 shade_normal(V3 N)
{ return (V3){ N.x*.5f+.5f, N.y*.5f+.5f, N.z*.5f+.5f }; }

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

    /* Fixed camera */
    V3 cam  = { 0.f, 0.f, -cam_dist };
    V3 fwd  = { 0.f, 0.f,  1.f };
    V3 rgt  = { 1.f, 0.f,  0.f };
    V3 up   = { 0.f, 1.f,  0.f };

    float cx = cols * .5f, cy = rows * .5f;

    for (int row = 0; row < rows-1; row++) {
        for (int col = 0; col < cols; col++) {
            float pu = (col - cx) / cx * fov_tan;
            float pv = -(row - cy) / cx * fov_tan / ASPECT;
            V3 rd_ws = v3norm(v3add(fwd, v3add(v3scale(pu,rgt), v3scale(pv,up))));

            /* Transform ray to object space (inverse rotation = transpose) */
            V3 ro_os = mat3_mulT(M, cam);
            V3 rd_os = mat3_mulT(M, rd_ws);

            float t_hit;
            V3    N_os;
            if (!ray_aabb(ro_os, rd_os, CUBE_S, &t_hit, &N_os)) continue;

            V3 P_os  = v3add(ro_os, v3scale(t_hit, rd_os));
            V3 P_ws  = v3add(cam,   v3scale(t_hit, rd_ws));
            V3 N_ws  = mat3_mul(M, N_os);          /* normal → world space */
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
            case MODE_WIRE: {
                float ed = face_edge_dist(P_os, N_os, CUBE_S);
                if (ed > WIRE_THRESH) continue;   /* interior: skip          */
                /* edge: colour by face normal */
                color = shade_normal(N_ws);
                lum   = 0.7f + 0.3f * (1.f - ed / WIRE_THRESH);
                break;
            }
            case MODE_DEPTH:
                color = shade_depth(t_hit, cam_dist * 2.f, th);
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
    static const char *const mnames[] = { "phong","normals","wireframe","depth" };
    attron(COLOR_PAIR(217));
    mvprintw(rows-1, 1,
        " fps:%.0f  dist:%.1f  theme:%s  mode:%s"
        "  [s]mode [t]theme [+/-]zoom [p]pause [q]quit ",
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
