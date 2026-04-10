/*
 * metaballs.c — SDF Metaballs + Smooth-Min Blending
 *
 * 6 metaballs on independent Lissajous orbits.  Polynomial smooth-min
 * blending merges their SDFs so the balls smoothly coalesce into one
 * blob and pull apart again.  Phong shading, optional soft shadows, and
 * surface-curvature coloring: flat merged regions get a cool hue, sharp
 * sphere peaks get a warm hue.
 *
 * Vary k live: small k = barely-touching separate spheres;
 *              large k = fully melted single blob.
 *
 * Keys:
 *   k / j      blend k larger / smaller   (more merged / more separate)
 *   s          toggle soft shadows
 *   c          cycle color themes (Classic / Ocean / Ember / Neon)
 *   space      pause / resume
 *   + / =      faster animation
 *   - / _      slower animation
 *   q / Q      quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/metaballs.c -o metaballs -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 color  §4 vec3
 *           §5 sdf     §6 canvas §7 scene  §8 screen  §9 app
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
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

#define N_BALLS       6

/* Canvas: each canvas pixel is drawn as a CELL_W × CELL_H block of
 * terminal cells.  2×2 quartes the pixel count, keeping the frame rate
 * comfortable despite the heavier SDF math.                             */
#define CELL_W        2
#define CELL_H        2
/* Physical height-to-width ratio of one terminal character cell.
 * Multiplied into the vertical ray direction for round-circle output.  */
#define CELL_ASPECT   2.0f

/* Raymarching */
#define RM_MAX_STEPS  64
#define RM_HIT_EPS    0.005f
#define RM_MAX_DIST   12.0f
#define CAM_Z         5.0f
#define FOV_HALF_TAN  0.55f

/* Smooth-min blend radius (world-space units).
 * Small → balls stay separate; large → merge into one blob.            */
#define K_DEFAULT     0.8f
#define K_MIN         0.05f
#define K_MAX         4.0f
#define K_STEP        1.35f

/* Animation speed multiplier */
#define SPD_DEFAULT   0.35f
#define SPD_MIN       0.02f
#define SPD_MAX       3.0f
#define SPD_STEP      1.35f

/* Phong coefficients */
#define KA            0.08f
#define KD            0.75f
#define KS            0.45f
#define SHIN          32.0f

/* Soft shadow */
#define SHADOW_STEPS  16
#define SHADOW_K      8.0f   /* higher = sharper shadow edge            */

/* Normal estimation (tetrahedron finite differences) */
#define NORM_EPS      0.004f

/* Curvature: Laplacian of SDF, ε and normalisation scale */
#define CURV_EPS      0.06f
#define CURV_SCALE    0.25f  /* maps raw Laplacian → [0,1]             */

/* Color */
#define N_CURV_BANDS  8
#define N_THEMES      4
#define CP_IDX(t,b)   ((t)*N_CURV_BANDS + (b) + 1)
#define CP_HUD        (N_THEMES * N_CURV_BANDS + 1)

/* FPS */
#define TARGET_FPS    24
#define NS_PER_SEC    1000000000LL
#define NS_PER_MS     1000000LL

/* ── Lissajous orbit parameters ──────────────────────────────────────── *
 * Each ball i traces: x = RX*sin(A[i]*t + PX[i])                       *
 *                     y = RY*sin(B[i]*t + PY[i])                       *
 *                     z = RZ*cos(C[i]*t)                                *
 * Different A/B/C frequencies give independent, non-repeating paths.   */
static const float BALL_A[N_BALLS]  = {1.0f, 2.0f, 1.5f, 3.0f, 2.5f, 1.0f};
static const float BALL_B[N_BALLS]  = {2.0f, 1.0f, 3.0f, 1.0f, 1.5f, 2.5f};
static const float BALL_C[N_BALLS]  = {1.5f, 3.0f, 1.0f, 2.0f, 1.0f, 2.0f};
/* Phase offsets: spread balls evenly around the orbit at t=0 */
static const float BALL_PX[N_BALLS] = {0.000f, 1.047f, 2.094f, 3.141f, 4.189f, 5.236f};
static const float BALL_PY[N_BALLS] = {0.785f, 0.000f, 1.571f, 0.524f, 3.927f, 0.262f};
/* Radii: slight variation so balls look distinct when separate */
static const float BALL_R[N_BALLS]  = {0.60f,  0.55f,  0.50f,  0.58f,  0.45f,  0.52f};
#define ORBIT_RX  1.35f
#define ORBIT_RY  0.75f
#define ORBIT_RZ  0.40f

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
    struct timespec req = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Curvature coloring: 4 themes × 8 bands.
 * Band 0 = low curvature (flat merged surface) → cool / dark hue.
 * Band 7 = high curvature (sharp sphere peak)  → warm / bright hue.
 *
 * Color pair layout: theme T, band B → pair CP_IDX(T,B).
 * Total pairs: 4×8 + 1(HUD) = 33.
 */
static const int k_theme_colors[N_THEMES][N_CURV_BANDS] = {
    /* Classic: deep-blue (low) → orange (high) */
    { 27,  33,  38,  44,  130, 166, 202, 214 },
    /* Ocean: navy (low) → bright-cyan (high) */
    { 17,  18,  19,  20,   26,  32,  38,  51 },
    /* Ember: dark-red (low) → bright-yellow (high) */
    { 52,  88, 124, 160,  196, 202, 208, 228 },
    /* Neon: magenta (low) → green (high) */
    {201, 165, 129,  93,   57,  82, 118, 155 },
};
static const char *k_theme_names[N_THEMES] = {
    "Classic", "Ocean", "Ember", "Neon"
};

/* Luminance ramp: index 0 = darkest, RAMP_N-1 = brightest */
static const char k_ramp[] = " .,:;+*oxOX#@";
#define RAMP_N  (int)(sizeof k_ramp - 1)

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        for (int t = 0; t < N_THEMES; t++)
            for (int b = 0; b < N_CURV_BANDS; b++)
                init_pair(CP_IDX(t, b), k_theme_colors[t][b], -1);
        init_pair(CP_HUD, 82, -1);
    } else {
        for (int i = 1; i <= N_THEMES * N_CURV_BANDS; i++)
            init_pair(i, COLOR_CYAN, -1);
        init_pair(CP_HUD, COLOR_GREEN, -1);
    }
}

/* Shade [0,1] → character from luminance ramp */
static char shade_ch(float shade)
{
    int i = (int)(shade * (float)(RAMP_N - 1) + 0.5f);
    if (i < 0)       i = 0;
    if (i >= RAMP_N) i = RAMP_N - 1;
    return k_ramp[i];
}

/* ===================================================================== */
/* §4  vec3                                                               */
/* ===================================================================== */

typedef struct { float x, y, z; } Vec3;

static inline Vec3  v3(float x, float y, float z) { return (Vec3){x,y,z}; }
static inline Vec3  v3add(Vec3 a, Vec3 b)  { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline Vec3  v3sub(Vec3 a, Vec3 b)  { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline Vec3  v3mul(Vec3 a, float s) { return v3(a.x*s,   a.y*s,   a.z*s);   }
static inline float v3dot(Vec3 a, Vec3 b)  { return a.x*b.x + a.y*b.y + a.z*b.z;  }
static inline float v3len(Vec3 a)          { return sqrtf(v3dot(a, a));              }
static inline Vec3  v3norm(Vec3 a)
{
    float l = v3len(a);
    return l > 1e-7f ? v3mul(a, 1.0f / l) : v3(0, 0, 1);
}

/* ===================================================================== */
/* §5  sdf + shading  (pure math, no ncurses)                            */
/* ===================================================================== */

/*
 * Polynomial smooth-min (C¹ continuity).  k = blend radius in world units.
 *
 *   k → 0  : approaches hard min  → balls stay separate
 *   k large : wide blending zone  → balls merge into one blob
 *
 * The blend term h²·k/4 widens the isosurface into the gap between balls,
 * creating the characteristic "skin" that stretches between them.
 */
static float smin(float a, float b, float k)
{
    float h = fmaxf(k - fabsf(a - b), 0.0f) / k;
    return fminf(a, b) - h * h * k * 0.25f;
}

static float sdf_ball(Vec3 p, Vec3 c, float r)
{
    return v3len(v3sub(p, c)) - r;
}

/* Scene SDF: smooth-min of all N_BALLS spheres */
static float sdf_scene(Vec3 p, const Vec3 *centers, const float *radii, float k)
{
    float d = sdf_ball(p, centers[0], radii[0]);
    for (int i = 1; i < N_BALLS; i++)
        d = smin(d, sdf_ball(p, centers[i], radii[i]), k);
    return d;
}

/*
 * Normal via tetrahedron finite differences — 4 SDF evaluations vs 6 for
 * central differences, with no accuracy loss.
 *
 * Vertices: k0=(+,-,-), k1=(-,+,-), k2=(-,-,+), k3=(+,+,+).
 * grad_x = k0.x·f0 + k1.x·f1 + k2.x·f2 + k3.x·f3 = +f0 -f1 -f2 +f3
 */
static Vec3 calc_normal(Vec3 p, const Vec3 *centers, const float *radii, float k)
{
    float e  = NORM_EPS;
    float f0 = sdf_scene(v3(p.x+e, p.y-e, p.z-e), centers, radii, k);
    float f1 = sdf_scene(v3(p.x-e, p.y+e, p.z-e), centers, radii, k);
    float f2 = sdf_scene(v3(p.x-e, p.y-e, p.z+e), centers, radii, k);
    float f3 = sdf_scene(v3(p.x+e, p.y+e, p.z+e), centers, radii, k);
    return v3norm(v3( f0-f1-f2+f3,
                     -f0+f1-f2+f3,
                     -f0-f1+f2+f3));
}

/*
 * Mean curvature ≈ Laplacian of the SDF / 2.
 * Returns a raw value; caller normalises to [0,1] via CURV_SCALE.
 *
 * For a sphere of radius r: Laplacian = 2/r (≈3.6 for r=0.55).
 * For a flat merged saddle: Laplacian ≈ 0.
 * For concave neck regions: Laplacian < 0 (clamped to 0 by caller).
 */
static float calc_curvature(Vec3 p, const Vec3 *centers, const float *radii, float k)
{
    float e  = CURV_EPS;
    float c0 = sdf_scene(p, centers, radii, k);
    float lap =
        sdf_scene(v3(p.x+e, p.y,   p.z  ), centers, radii, k) +
        sdf_scene(v3(p.x-e, p.y,   p.z  ), centers, radii, k) +
        sdf_scene(v3(p.x,   p.y+e, p.z  ), centers, radii, k) +
        sdf_scene(v3(p.x,   p.y-e, p.z  ), centers, radii, k) +
        sdf_scene(v3(p.x,   p.y,   p.z+e), centers, radii, k) +
        sdf_scene(v3(p.x,   p.y,   p.z-e), centers, radii, k)
        - 6.0f * c0;
    return lap / (e * e);
}

/*
 * Soft penumbra shadow: march from the surface toward the light, tracking
 * the closest SDF approach normalised by travel distance.  sk controls
 * shadow sharpness — higher values give harder penumbra edges.
 */
static float soft_shadow(Vec3 ro, Vec3 rd, float tmin, float tmax,
                          float sk, const Vec3 *centers, const float *radii,
                          float k)
{
    float res = 1.0f, t = tmin;
    for (int i = 0; i < SHADOW_STEPS && t < tmax; i++) {
        float h = sdf_scene(v3add(ro, v3mul(rd, t)), centers, radii, k);
        if (h < RM_HIT_EPS * 0.5f) return 0.0f;
        res = fminf(res, sk * h / t);
        t  += fmaxf(h, RM_HIT_EPS);
    }
    return fmaxf(res, 0.0f);
}

/* Phong shading intensity [0,1].  shadow ∈ [0,1] multiplies diffuse+spec. */
static float phong(Vec3 hit, Vec3 N, Vec3 cam, Vec3 light, float shadow)
{
    Vec3  L   = v3norm(v3sub(light, hit));
    Vec3  V   = v3norm(v3sub(cam,   hit));
    float ndl = fmaxf(0.0f, v3dot(N, L));
    Vec3  R   = v3sub(v3mul(N, 2.0f * ndl), L);
    float sp  = powf(fmaxf(0.0f, v3dot(R, V)), SHIN);
    return fminf(1.0f, KA + shadow * (KD * ndl + KS * sp));
}

/* Result stored per canvas pixel */
typedef struct { float shade; float curv; } PixelResult;
#define PIXEL_MISS  ((PixelResult){-1.0f, 0.0f})

/*
 * Cast one ray for canvas pixel (px, py) and return shade + curvature.
 *
 * Aspect correction: terminal chars are CELL_ASPECT× taller than wide,
 * so the vertical ray component is scaled by phys_aspect to produce
 * round circles on screen.
 */
static PixelResult rm_cast(int px, int py, int cw, int ch,
                            const Vec3 *centers, const float *radii,
                            float k_blend, Vec3 light, bool soft_shad)
{
    float u = ((float)px + 0.5f) / (float)cw * 2.0f - 1.0f;
    float v = -((float)py + 0.5f) / (float)ch * 2.0f + 1.0f;
    float phys_aspect = ((float)ch * (float)CELL_H * CELL_ASPECT)
                      / ((float)cw * (float)CELL_W);
    Vec3  ro = v3(0.0f, 0.0f, CAM_Z);
    Vec3  rd = v3norm(v3(u * FOV_HALF_TAN,
                         v * FOV_HALF_TAN * phys_aspect,
                         -1.0f));

    float t = 0.0f;
    for (int i = 0; i < RM_MAX_STEPS; i++) {
        Vec3  p = v3add(ro, v3mul(rd, t));
        float d = sdf_scene(p, centers, radii, k_blend);
        if (d < RM_HIT_EPS) {
            Vec3  hit = v3add(ro, v3mul(rd, t));
            Vec3  N   = calc_normal(hit, centers, radii, k_blend);
            Vec3  L   = v3norm(v3sub(light, hit));

            float shadow = 1.0f;
            if (soft_shad) {
                Vec3  sro   = v3add(hit, v3mul(N, 0.01f));
                float ldist = v3len(v3sub(light, hit));
                shadow = soft_shadow(sro, L, 0.02f, ldist,
                                     SHADOW_K, centers, radii, k_blend);
            }

            float raw_curv = calc_curvature(hit, centers, radii, k_blend);
            float curv     = fmaxf(0.0f, fminf(1.0f, raw_curv * CURV_SCALE));

            return (PixelResult){ phong(hit, N, ro, light, shadow), curv };
        }
        if (t > RM_MAX_DIST) break;
        t += d;
    }
    return PIXEL_MISS;
}

/* ===================================================================== */
/* §6  canvas                                                             */
/* ===================================================================== */

typedef struct {
    int    w, h;
    float *shades;   /* [h*w]; -1 = miss */
    float *curvs;    /* [h*w] */
} Canvas;

static void canvas_alloc(Canvas *c, int cols, int rows)
{
    c->w = cols / CELL_W;
    c->h = rows / CELL_H;
    if (c->w < 1) c->w = 1;
    if (c->h < 1) c->h = 1;
    size_t n  = (size_t)(c->w * c->h);
    c->shades = malloc(n * sizeof(float));
    c->curvs  = malloc(n * sizeof(float));
}

static void canvas_free(Canvas *c)
{
    free(c->shades); c->shades = NULL;
    free(c->curvs);  c->curvs  = NULL;
    c->w = c->h = 0;
}

static void canvas_render(Canvas *c, const Vec3 *centers, const float *radii,
                           float k_blend, Vec3 light, bool soft_shad)
{
    for (int py = 0; py < c->h; py++) {
        for (int px = 0; px < c->w; px++) {
            PixelResult r = rm_cast(px, py, c->w, c->h,
                                    centers, radii, k_blend, light, soft_shad);
            c->shades[py * c->w + px] = r.shade;
            c->curvs [py * c->w + px] = r.curv;
        }
    }
}

static void canvas_draw(const Canvas *c, int term_cols, int term_rows, int theme)
{
    int off_x = (term_cols - c->w * CELL_W) / 2;
    int off_y = (term_rows - c->h * CELL_H) / 2;

    for (int py = 0; py < c->h; py++) {
        for (int px = 0; px < c->w; px++) {
            float sh = c->shades[py * c->w + px];
            if (sh < 0.0f) continue;

            float cv   = c->curvs[py * c->w + px];
            int   band = (int)(cv * (float)(N_CURV_BANDS - 1) + 0.5f);
            if (band < 0)           band = 0;
            if (band >= N_CURV_BANDS) band = N_CURV_BANDS - 1;

            char   ch   = shade_ch(sh);
            attr_t attr = COLOR_PAIR(CP_IDX(theme, band));
            if (sh > 0.72f) attr |= A_BOLD;

            for (int by = 0; by < CELL_H; by++) {
                for (int bx = 0; bx < CELL_W; bx++) {
                    int tx = off_x + px * CELL_W + bx;
                    int ty = off_y + py * CELL_H + by;
                    if (tx < 0 || tx >= term_cols) continue;
                    if (ty < 0 || ty >= term_rows) continue;
                    attron(attr);
                    mvaddch(ty, tx, (chtype)(unsigned char)ch);
                    attroff(attr);
                }
            }
        }
    }
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

typedef struct {
    Canvas canvas;
    Vec3   centers[N_BALLS];
    float  radii[N_BALLS];
    float  time;
    float  k_blend;
    float  speed;
    int    theme;
    bool   paused;
    bool   soft_shadows;
} Scene;

static Vec3 scene_light(const Scene *s)
{
    float t = s->time * 0.6f;
    return v3(cosf(t) * 4.0f, sinf(t * 0.45f) * 2.0f + 2.5f, 3.5f);
}

static void scene_update_balls(Scene *s)
{
    float t = s->time;
    for (int i = 0; i < N_BALLS; i++) {
        s->centers[i] = v3(
            ORBIT_RX * sinf(BALL_A[i] * t + BALL_PX[i]),
            ORBIT_RY * sinf(BALL_B[i] * t + BALL_PY[i]),
            ORBIT_RZ * cosf(BALL_C[i] * t)
        );
        s->radii[i] = BALL_R[i];
    }
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    canvas_alloc(&s->canvas, cols, rows);
    s->k_blend     = K_DEFAULT;
    s->speed       = SPD_DEFAULT;
    s->theme       = 0;
    s->paused      = false;
    s->soft_shadows = true;
    scene_update_balls(s);
}

static void scene_free(Scene *s)   { canvas_free(&s->canvas); }

static void scene_resize(Scene *s, int cols, int rows)
{
    canvas_free(&s->canvas);
    canvas_alloc(&s->canvas, cols, rows);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->time += dt * s->speed;
    scene_update_balls(s);
}

static void scene_render(Scene *s)
{
    canvas_render(&s->canvas, s->centers, s->radii,
                  s->k_blend, scene_light(s), s->soft_shadows);
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    canvas_draw(&s->canvas, cols, rows, s->theme);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s)   { (void)s; endwin(); }

static void screen_resize(Screen *s) { endwin(); refresh(); getmaxyx(stdscr, s->rows, s->cols); }

static void screen_draw(const Screen *s, const Scene *sc, double fps)
{
    erase();
    scene_draw(sc, s->cols, s->rows);
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(s->rows - 1, 0,
             " k=%.2f  %s  spd=%.2f  shad=%-3s  %4.1f fps"
             "  j/k:blend  c:theme  s:shadow  +/-:spd  spc:pause  q:quit",
             sc->k_blend, k_theme_names[sc->theme], sc->speed,
             sc->soft_shadows ? "on" : "off", fps);
    attroff(COLOR_PAIR(CP_HUD));
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

static void on_exit_signal(int s)   { (void)s; g_app.running = 0;     }
static void on_resize_signal(int s) { (void)s; g_app.need_resize = 1; }
static void cleanup(void)           { endwin(); }

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': return false;
    case ' ':   s->paused ^= 1;                                      break;
    case 'k':   if (s->k_blend < K_MAX) s->k_blend *= K_STEP;        break;
    case 'j':   if (s->k_blend > K_MIN) s->k_blend /= K_STEP;        break;
    case 's':   s->soft_shadows ^= 1;                                 break;
    case 'c': case 'C':
        s->theme = (s->theme + 1) % N_THEMES;
        break;
    case '+': case '=':
        if (s->speed < SPD_MAX) s->speed *= SPD_STEP;
        break;
    case '-': case '_':
        if (s->speed > SPD_MIN) s->speed /= SPD_STEP;
        break;
    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {
        if (app->need_resize) {
            app->need_resize = 0;
            screen_resize(&app->screen);
            scene_resize(&app->scene, app->screen.cols, app->screen.rows);
            frame_time = clock_ns();
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 200 * NS_PER_MS) dt = 200 * NS_PER_MS;
        float dt_sec = (float)dt / (float)NS_PER_SEC;

        int ch;
        while ((ch = getch()) != ERR)
            if (!app_handle_key(app, ch)) { app->running = 0; break; }

        scene_tick(&app->scene, dt_sec);
        scene_render(&app->scene);

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= 500 * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        screen_draw(&app->screen, &app->scene, fps_display);

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
