/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sphere_raytrace.c — analytic ray-traced sphere
 *
 * One ray per terminal cell.  Solves the quadratic ray-sphere equation
 * exactly (no marching, no mesh).  Camera orbits the sphere; three
 * coloured lights (warm key · cool fill · bright rim) create depth.
 *
 * Modes  (s):  phong · normals · fresnel · depth
 * Themes (t):  gold · ice · crimson · emerald · amethyst · neon
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/sphere_raytrace.c \
 *       -o sphere_rt -lncurses -lm
 *
 * Keys:
 *   s         cycle shade mode
 *   t         cycle theme
 *   p         pause / resume
 *   + / =     zoom in   (orbit closer)
 *   -         zoom out
 *   q / ESC   quit
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic ray-sphere intersection — exact, no marching steps.
 *                  For ray ro + t·rd and sphere centre c radius R:
 *                    |ro + t·rd − c|² = R²
 *                  Expands to quadratic: a·t² + b·t + c = 0
 *                    a = |rd|² = 1 (unit direction)
 *                    b = 2·(rd · oc)  where oc = ro − c
 *                    c = |oc|² − R²
 *                  discriminant = b²−4ac; if < 0: miss; else t = (−b ± √disc) / 2a.
 *
 * Math           : Surface normal at hit point p: N = (p − c) / R (unit outward).
 *                  Phong model: I = ka + kd·max(N·L, 0) + ks·max(R_v·V, 0)^shininess
 *                  where R_v = 2(N·L)N − L (reflection direction).
 *                  Fresnel approximation (Schlick): F = F0 + (1−F0)·(1−N·V)^5
 *                  models the view-angle-dependent reflectance of dielectrics.
 *
 * Rendering      : Each terminal cell fires exactly one ray (no anti-aliasing).
 *                  Three-point lighting: warm key + cool fill + bright rim.
 *                  Luminance mapped to ASCII ramp ".+*#@" — brighter → denser char.
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
#define ASPECT        0.47f      /* terminal cell width / height            */
#define FOV_DEG       58.0f
#define SPHERE_R      1.0f
#define ORBIT_SPEED   0.32f      /* radians / second                        */
#define CAM_HEIGHT    0.55f      /* camera elevation above equator          */
#define CAM_DIST_DEF  3.6f
#define CAM_DIST_MIN  1.9f
#define CAM_DIST_MAX  7.0f
#define CAM_DIST_STEP 0.25f
#define AMBIENT       0.04f
#define SHININESS     52.0f

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

/* ── §3  V3 math ─────────────────────────────────────────────────────────── */

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
{
    return (V3){ v.x<0?0:v.x>1?1:v.x, v.y<0?0:v.y>1?1:v.y, v.z<0?0:v.z>1?1:v.z };
}

/* ── §4  color / themes ──────────────────────────────────────────────────── */

typedef struct {
    V3 obj;        /* surface diffuse base color                          */
    V3 spec;       /* specular highlight color                            */
    V3 key_col;    /* key light colour  (warm, upper-right)               */
    V3 fill_col;   /* fill light colour (cool, left)                      */
    V3 rim_col;    /* rim light colour  (from behind)                     */
    const char *name;
} Theme;

static const Theme g_themes[] = {
    /* gold */
    { {0.90f,0.72f,0.18f},{1.00f,0.95f,0.65f},
      {1.00f,0.92f,0.70f},{0.25f,0.35f,0.80f},{0.95f,0.55f,0.10f},"gold" },
    /* ice */
    { {0.50f,0.78f,1.00f},{0.85f,0.93f,1.00f},
      {0.75f,0.88f,1.00f},{0.15f,0.25f,0.75f},{0.45f,0.80f,1.00f},"ice" },
    /* crimson */
    { {0.88f,0.12f,0.12f},{1.00f,0.75f,0.60f},
      {1.00f,0.82f,0.65f},{0.15f,0.08f,0.50f},{1.00f,0.35f,0.08f},"crimson" },
    /* emerald */
    { {0.08f,0.78f,0.28f},{0.60f,1.00f,0.70f},
      {0.75f,1.00f,0.60f},{0.08f,0.35f,0.60f},{0.15f,0.90f,0.35f},"emerald" },
    /* amethyst */
    { {0.62f,0.18f,0.88f},{0.85f,0.65f,1.00f},
      {0.88f,0.78f,1.00f},{0.08f,0.08f,0.60f},{0.78f,0.18f,0.88f},"amethyst" },
    /* neon */
    { {0.08f,0.92f,0.88f},{0.70f,1.00f,0.95f},
      {0.88f,0.92f,0.50f},{0.08f,0.38f,0.80f},{0.18f,1.00f,0.75f},"neon" },
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
    init_pair(217, 15, -1);   /* HUD white                                */
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

/* ── §5  ray–sphere intersection ─────────────────────────────────────────── */

/*
 * Ray: P(t) = ro + t·rd,  |rd| = 1
 * Sphere at origin radius r:  |P|² = r²
 *   ⟹  t² + 2(rd·ro)t + (|ro|²−r²) = 0
 * Returns 1 on hit, sets *t_hit to nearest positive t.
 */
static int ray_sphere(V3 ro, V3 rd, float r, float *t_hit)
{
    float b    = v3dot(rd, ro);
    float c    = v3dot(ro, ro) - r * r;
    float disc = b * b - c;
    if (disc < 0.f) return 0;
    float sq = sqrtf(disc);
    float t0 = -b - sq, t1 = -b + sq;
    if (t1 < 1e-4f) return 0;
    *t_hit = (t0 > 1e-4f) ? t0 : t1;
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
    /* fill — soft diffuse only */
    {
        V3    L = v3norm(v3sub(L_FILL, P));
        float d = fmaxf(0.f, v3dot(N, L));
        col = v3add(col, v3scale(d * 0.22f, v3mul(th->obj, th->fill_col)));
    }
    /* rim — narrow specular on silhouette */
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
 * Schlick approximation: glancing angles → bright edge; head-on → dark core.
 * Gives a glass-sphere or crystal-ball look.
 */
static V3 shade_fresnel(V3 N, V3 V_dir, const Theme *th)
{
    float cosA    = fabsf(v3dot(N, V_dir));
    float inv     = 1.f - cosA;
    float fresnel = inv * inv * inv * inv * inv;   /* (1−cosθ)^5 */
    V3 core = v3scale(0.06f, th->obj);
    V3 edge = v3clamp1(v3add(v3scale(0.7f, th->spec), v3scale(0.5f, th->rim_col)));
    return v3clamp1(v3add(v3scale(1.f-fresnel, core), v3scale(fresnel, edge)));
}

static V3 shade_depth(float t, float t_max, const Theme *th)
{
    float d = 1.f - fminf(t / t_max, 1.f);
    d = d * d;   /* square for more dramatic falloff */
    return v3clamp1(v3scale(d, th->obj));
}

/* ── §7  render frame ────────────────────────────────────────────────────── */

static void render(int cols, int rows, float orbit_ang, float cam_dist,
                   int theme, ShadeMode mode)
{
    const Theme *th = &g_themes[theme % THEME_N];
    float fov_tan   = tanf(FOV_DEG * (float)M_PI / 360.f);

    /* Orbiting camera */
    V3 cam  = { cam_dist * sinf(orbit_ang), CAM_HEIGHT, -cam_dist * cosf(orbit_ang) };
    V3 fwd  = v3norm(v3sub((V3){0,0,0}, cam));
    V3 wup  = { 0.f, 1.f, 0.f };
    V3 rgt  = v3norm(v3cross(fwd, wup));
    V3 up   = v3cross(rgt, fwd);

    float cx = cols * .5f, cy = rows * .5f;

    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            float pu = (col - cx) / cx * fov_tan;
            float pv = -(row - cy) / cx * fov_tan / ASPECT;
            V3 rd = v3norm(v3add(fwd, v3add(v3scale(pu, rgt), v3scale(pv, up))));

            float t_hit;
            if (!ray_sphere(cam, rd, SPHERE_R, &t_hit)) continue;

            V3 P     = v3add(cam, v3scale(t_hit, rd));
            V3 N     = v3norm(P);                        /* sphere centred at origin */
            V3 V_dir = v3norm(v3sub(cam, P));

            V3    color;
            float lum;

            switch (mode) {
            default:
            case MODE_PHONG:
                color = shade_phong(P, N, V_dir, th);
                lum   = 0.299f*color.x + 0.587f*color.y + 0.114f*color.z;
                break;
            case MODE_NORMAL:
                color = shade_normal(N);
                lum   = (N.x*.5f+.5f)*.3f + (N.y*.5f+.5f)*.6f + (N.z*.5f+.5f)*.1f;
                break;
            case MODE_FRESNEL:
                color = shade_fresnel(N, V_dir, th);
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

    int       theme     = 0;
    ShadeMode mode      = MODE_PHONG;
    float     cam_dist  = CAM_DIST_DEF;
    float     orbit_ang = 0.f;
    int       paused    = 0;
    float     fps       = 0.f;
    long long fps_acc   = 0;
    int       fps_cnt   = 0;
    long long frame_ns  = 1000000000LL / TARGET_FPS;
    long long last      = clock_ns();

    while (g_run) {
        if (g_resize) {
            g_resize = 0; endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
        }
        long long now = clock_ns();
        long long dt  = now - last; if (dt > 100000000LL) dt = 100000000LL;
        last = now;

        if (!paused) orbit_ang += ORBIT_SPEED * (float)dt * 1e-9f;

        fps_acc += dt; fps_cnt++;
        if (fps_acc >= 500000000LL) {
            fps = (float)fps_cnt * 1e9f / (float)fps_acc;
            fps_acc = 0; fps_cnt = 0;
        }

        long long t0 = clock_ns();
        erase();
        render(cols, rows, orbit_ang, cam_dist, theme, mode);
        screen_hud(cols, rows, fps, theme, mode, cam_dist);
        wnoutrefresh(stdscr); doupdate();

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_run = 0;                                   break;
        case 'p': case 'P': paused = !paused;                                      break;
        case 's': case 'S': mode = (ShadeMode)((mode + 1) % MODE_N);              break;
        case 't': case 'T': theme = (theme + 1) % THEME_N;                        break;
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
