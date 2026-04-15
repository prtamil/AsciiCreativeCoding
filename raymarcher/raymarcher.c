/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sphere.c  —  ncurses raymarched ASCII sphere
 *
 * Renders a 3-D Phong-shaded sphere using raymarching at a deliberately
 * low virtual resolution that is then block-upscaled to the terminal,
 * so the sphere always looks round regardless of terminal dimensions.
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume light orbit
 *   ]  [      light faster / slower
 *   =  -      sphere larger / smaller
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra sphere.c -o sphere -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config      — every tunable constant in one place
 *   §2  clock       — monotonic ns clock + sleep
 *   §3  color       — luminance pairs; 256-color + 8-color fallback
 *   §4  vec3        — inline 3-D vector math (value types, no heap)
 *   §5  raymarch    — SDF, march loop, Phong shading  (pure math, no ncurses)
 *   §6  canvas      — virtual low-res framebuffer + upscale to terminal
 *   §7  scene       — owns canvas + animation state + tick + render + draw
 *   §8  screen      — single stdscr, ncurses internal double buffer
 *   §9  app         — dt loop, input, resize, cleanup
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Sphere tracing (SDF raymarching) — safe stepping.
 *                  A ray is cast from the camera through each pixel.
 *                  At each step: evaluate the scene SDF at the current ray
 *                  position p.  The SDF returns the exact distance to the
 *                  nearest surface.  Advance the ray by this distance.
 *                  This guarantees no surface overshoot (sphere tracing).
 *
 * Math           : For a sphere at origin with radius R:
 *                    SDF(p) = |p| − R
 *                  Normal at surface: N = normalise(∇SDF) = p/|p| (exact for sphere).
 *                  Phong shading: I = kd·(N·L) + ks·(R·V)^n + ka
 *                  The ray terminates when SDF(p) < HIT_EPS (surface hit) or
 *                  total distance exceeds MAX_DIST (miss).
 *
 * Rendering      : Virtual canvas at low resolution (VIRT_W × VIRT_H) is
 *                  block-upscaled to the terminal — each virtual pixel maps
 *                  to a small block of terminal cells.  This keeps the sphere
 *                  round despite non-square terminal cells.
 * ─────────────────────────────────────────────────────────────────────── */

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
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN     =  5,
    SIM_FPS_DEFAULT = 24,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,

    HUD_COLS        = 38,
    FPS_UPDATE_MS   = 500,
};

/*
 * §4b  coords — terminal cell aspect ratio
 * ==========================================
 *
 * For the sphere raymarcher the correct approach is:
 *   canvas pixel = ONE terminal cell  (CELL_W=1, CELL_H=1)
 *
 * This gives maximum resolution — every terminal cell is one ray.
 * The sphere is rendered at native terminal resolution.
 *
 * Aspect ratio is corrected INSIDE the ray direction, not in block size.
 * Terminal cells are ~2× taller than wide in physical pixels.
 * If we fire rays on a square grid (u,v both in [-1,1]) the sphere
 * appears squashed vertically because each row covers twice as much
 * physical height as each column covers width.
 *
 * Fix: scale the vertical ray component by CELL_ASPECT = CELL_H/CELL_W
 * so rays are spaced equally in physical pixels, not in cells.
 *
 *   rd = normalise( u * FOV,  v * FOV * CELL_ASPECT,  -1 )
 *
 * CELL_ASPECT = 2.0  means the vertical ray is stretched 2× so it
 * covers the same physical distance per step as the horizontal ray.
 * The sphere renders as a circle on screen.
 *
 * canvas_w = cols  (one pixel per column — full resolution)
 * canvas_h = rows  (one pixel per row    — full resolution)
 */
#define CELL_W       1      /* canvas pixels per terminal column         */
#define CELL_H       1      /* canvas pixels per terminal row            */
#define CELL_ASPECT  2.0f   /* physical height/width of one terminal cell*/

static inline int canvas_w_from_cols(int cols) { return cols / CELL_W; }
static inline int canvas_h_from_rows(int rows) { return rows / CELL_H; }

/* Raymarching */
#define RM_MAX_STEPS   80
#define RM_HIT_EPS     0.002f
#define RM_MAX_DIST    20.0f

/*
 * Camera sits on the +Z axis, looking toward the origin.
 * The field of view is controlled by FOV_HALF_TAN = tan(fov/2).
 * 0.7 ≈ 70° horizontal fov — wide enough to see the whole sphere.
 */
#define CAM_Z          4.0f
#define FOV_HALF_TAN   0.7f

/* Sphere */
#define SPHERE_R_DEFAULT  1.1f
#define SIZE_STEP         1.15f
#define SIZE_MIN          0.2f
#define SIZE_MAXX          3.0f

/* Light orbit (radians/sec) */
#define LIGHT_SPD_DEFAULT 0.8f
#define LIGHT_SPD_STEP    1.35f
#define LIGHT_SPD_MIN     0.02f
#define LIGHT_SPD_MAX     8.0f

/* Phong coefficients */
#define KA   0.10f   /* ambient                                         */
#define KD   0.78f   /* diffuse                                         */
#define KS   0.55f   /* specular                                        */
#define SHIN 40.0f   /* shininess exponent                              */

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC / (f))

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
 * 8 luminance levels → ncurses color pairs 1..8.
 * 256-color: grey ramp 235,238,241,244,247,250,253,255.
 * 8-color:   A_DIM / normal / A_BOLD on COLOR_WHITE.
 *
 * Pair index = luminance_level + 1  (pair 0 is reserved by ncurses).
 */
enum { LUMI_N = 8 };

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        static const int grey[LUMI_N] = {235,238,241,244,247,250,253,255};
        for (int i = 0; i < LUMI_N; i++)
            init_pair(i + 1, grey[i], COLOR_BLACK);
    } else {
        for (int i = 0; i < LUMI_N; i++)
            init_pair(i + 1, COLOR_WHITE, COLOR_BLACK);
    }
}

/* Map luminance level l in [0, LUMI_N-1] to an ncurses attr_t. */
static attr_t lumi_attr(int l)
{
    if (l < 0)         l = 0;
    if (l > LUMI_N-1)  l = LUMI_N-1;
    attr_t a = COLOR_PAIR(l + 1);
    if (COLORS < 256) {
        if      (l < 3)  a |= A_DIM;
        else if (l >= 6) a |= A_BOLD;
    }
    return a;
}

/* ===================================================================== */
/* §4  vec3                                                               */
/* ===================================================================== */

/*
 * All operations return Vec3 by value — no pointers, no heap.
 * With -O2 the compiler inlines every call here to register operations.
 */
typedef struct { float x, y, z; } Vec3;

static inline Vec3  v3(float x, float y, float z) { return (Vec3){x,y,z}; }
static inline Vec3  v3add(Vec3 a, Vec3 b)  { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline Vec3  v3sub(Vec3 a, Vec3 b)  { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline Vec3  v3mul(Vec3 a, float s) { return v3(a.x*s,   a.y*s,   a.z*s);   }
static inline float v3dot(Vec3 a, Vec3 b)  { return a.x*b.x + a.y*b.y + a.z*b.z;   }
static inline float v3len(Vec3 a)          { return sqrtf(v3dot(a,a));               }
static inline Vec3  v3norm(Vec3 a)
{
    float l = v3len(a);
    return l > 1e-7f ? v3mul(a, 1.0f/l) : v3(0,0,1);
}

/* ===================================================================== */
/* §5  raymarch  (pure math — no ncurses, no global state)               */
/* ===================================================================== */

/*
 * sdf_sphere() — signed distance from point p to a sphere at the origin.
 *
 *   d = |p| - r
 *   d < 0  inside the sphere
 *   d = 0  on the surface
 *   d > 0  outside
 */
static float sdf_sphere(Vec3 p, float r)
{
    return v3len(p) - r;
}

/*
 * rm_march() — advance a ray from `ro` in direction `rd` until it hits
 * the sphere (radius r) or escapes.
 *
 * Returns the hit distance t, or -1 if no hit.
 *
 * This function knows ONLY about the math.  It has no knowledge of the
 * terminal, the canvas, or ncurses.
 */
static float rm_march(Vec3 ro, Vec3 rd, float r)
{
    float t = 0.0f;
    for (int i = 0; i < RM_MAX_STEPS; i++) {
        Vec3  p = v3add(ro, v3mul(rd, t));
        float d = sdf_sphere(p, r);
        if (d < RM_HIT_EPS)  return t;
        if (t > RM_MAX_DIST) return -1.0f;
        t += d;
    }
    return -1.0f;
}

/*
 * rm_shade() — compute Phong intensity [0,1] at a surface hit point.
 *
 *   hit      3-D surface point
 *   cam      camera position (for specular view vector)
 *   light    world-space light position
 *
 * For a sphere at the origin the surface normal is simply normalise(hit).
 * Phong: I = Ka + Kd*(N·L) + Ks*(R·V)^shininess
 */
static float rm_shade(Vec3 hit, Vec3 cam, Vec3 light)
{
    Vec3  N    = v3norm(hit);                        /* sphere normal    */
    Vec3  L    = v3norm(v3sub(light, hit));          /* to light         */
    Vec3  V    = v3norm(v3sub(cam,   hit));          /* to camera        */

    /* Reflect L about N: R = 2*(N·L)*N - L */
    float ndl  = fmaxf(0.0f, v3dot(N, L));
    Vec3  R    = v3sub(v3mul(N, 2.0f * ndl), L);
    float spec = powf(fmaxf(0.0f, v3dot(R, V)), SHIN);

    float I    = KA + KD * ndl + KS * spec;
    return I > 1.0f ? 1.0f : I;
}

/*
 * rm_cast_pixel() — cast one ray for canvas pixel (px, py).
 *
 * NDC coordinates: u in [-1,1] horizontal, v in [1,-1] vertical (Y up).
 *
 *   u =  (px + 0.5) / canvas_w * 2 - 1
 *   v = -(py + 0.5) / canvas_h * 2 + 1
 *
 * ASPECT CORRECTION:
 *   canvas_w = cols, canvas_h = rows (one pixel per terminal cell).
 *   Terminal cells are CELL_ASPECT=2.0× taller than wide in physical pixels.
 *   Without correction, the sphere subtends twice as many rows as columns
 *   for the same world-space radius — it appears squashed vertically.
 *
 *   Fix: multiply v by CELL_ASPECT before building the ray direction.
 *   This stretches the vertical field of view to match physical screen space.
 *   The NDC v coordinate now represents physical height, not row count.
 *   The sphere renders as a circle.
 *
 *   Also adjust FOV: horizontal FOV covers canvas_w cells of width CELL_W,
 *   vertical FOV covers canvas_h cells of height CELL_H. The aspect-adjusted
 *   FOV_HALF_TAN applies to the horizontal axis; vertical gets scaled by
 *   (canvas_h * CELL_H) / (canvas_w * CELL_W) to match physical dimensions.
 */
static float rm_cast_pixel(int px, int py,
                             int canvas_w, int canvas_h,
                             float sphere_r, Vec3 light)
{
    float u =  ((float)px + 0.5f) / (float)canvas_w * 2.0f - 1.0f;
    float v = -((float)py + 0.5f) / (float)canvas_h * 2.0f + 1.0f;

    /*
     * Physical aspect: canvas covers (canvas_w * CELL_W) px wide
     *                              × (canvas_h * CELL_H) px tall.
     * The physical aspect ratio of the canvas in pixels:
     *   phys_aspect = (canvas_h * CELL_H) / (canvas_w * CELL_W)
     * Multiply v by this so the ray spans equal physical distance per unit.
     */
    float phys_aspect = ((float)canvas_h * CELL_ASPECT)
                       / (float)canvas_w;

    Vec3 ro = v3(0.0f, 0.0f, CAM_Z);
    Vec3 rd = v3norm(v3(u * FOV_HALF_TAN,
                        v * FOV_HALF_TAN * phys_aspect,
                        -1.0f));

    float t = rm_march(ro, rd, sphere_r);
    if (t < 0.0f) return -1.0f;

    Vec3 hit = v3add(ro, v3mul(rd, t));
    return rm_shade(hit, ro, light);
}

/* ===================================================================== */
/* §6  canvas                                                             */
/* ===================================================================== */

/*
 * Canvas — the virtual square-pixel framebuffer.
 *
 * w, h     — canvas dimensions in square pixels (computed from terminal)
 * pixels[] — heap-allocated [h × w] array of intensity index or MISS
 *
 * The canvas is square in pixel space — every pixel represents the same
 * physical area on screen.  The raymarcher renders circles as circles.
 *
 * canvas_draw() is the one function that knows about CELL_W and CELL_H.
 * It maps each canvas pixel to a CELL_W × CELL_H block of terminal cells.
 * That block is visually square, so the rendered sphere is round.
 */
#define CANVAS_MISS  -1

typedef struct {
    int  w, h;      /* canvas dimensions in square pixels               */
    int *pixels;    /* [h * w] heap array — intensity index or MISS     */
} Canvas;

static void canvas_alloc(Canvas *c, int cols, int rows)
{
    c->w      = canvas_w_from_cols(cols);
    c->h      = canvas_h_from_rows(rows);
    c->pixels = calloc((size_t)(c->w * c->h), sizeof(int));
}

static void canvas_free(Canvas *c)
{
    free(c->pixels);
    c->pixels = NULL;
    c->w = c->h = 0;
}

/* Character ramp: index 0 = darkest, N-1 = brightest. */
static const char k_ramp[] = " .,:;+*oxOX#@";
#define RAMP_N (int)(sizeof k_ramp - 1)

static void canvas_clear(Canvas *c)
{
    for (int i = 0; i < c->w * c->h; i++)
        c->pixels[i] = CANVAS_MISS;
}

/*
 * canvas_render() — fill every square pixel by casting one ray.
 * Pure math — no terminal knowledge.
 */
static void canvas_render(Canvas *c, float sphere_r, Vec3 light)
{
    canvas_clear(c);
    for (int py = 0; py < c->h; py++) {
        for (int px = 0; px < c->w; px++) {
            float intensity = rm_cast_pixel(px, py, c->w, c->h,
                                             sphere_r, light);
            if (intensity < 0.0f) {
                c->pixels[py * c->w + px] = CANVAS_MISS;
            } else {
                int idx = (int)(intensity * (float)(RAMP_N - 1) + 0.5f);
                if (idx >= RAMP_N) idx = RAMP_N - 1;
                c->pixels[py * c->w + px] = idx;
            }
        }
    }
}

/*
 * canvas_draw() — map square canvas pixels to terminal cells.
 *
 * Each canvas pixel (vx, vy) → a CELL_W × CELL_H block of terminal cells.
 * CELL_H / CELL_W = 2.0 (terminal cells are 2× taller than wide), so
 * a 2-column × 4-row block is visually square on screen.
 *
 * This is the ONLY function in the program that reads CELL_W and CELL_H.
 * All code above is aspect-blind — it renders into square-pixel space.
 * All code below (screen) knows only terminal rows/cols.
 */
static void canvas_draw(const Canvas *c, int term_cols, int term_rows)
{
    /* Centre the canvas in the terminal */
    int total_w = c->w * CELL_W;
    int total_h = c->h * CELL_H;
    int off_x   = (term_cols - total_w) / 2;
    int off_y   = (term_rows - total_h) / 2;

    for (int vy = 0; vy < c->h; vy++) {
        for (int vx = 0; vx < c->w; vx++) {

            int idx = c->pixels[vy * c->w + vx];
            if (idx == CANVAS_MISS) continue;

            char   ch   = k_ramp[idx];
            attr_t attr = lumi_attr((idx * LUMI_N) / RAMP_N);

            /*
             * Fill the CELL_W × CELL_H terminal block for this pixel.
             * Every cell in the block gets the same character and color.
             */
            for (int by = 0; by < CELL_H; by++) {
                for (int bx = 0; bx < CELL_W; bx++) {
                    int tx = off_x + vx * CELL_W + bx;
                    int ty = off_y + vy * CELL_H + by;
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

/*
 * Scene — owns animation state + canvas + tick + render + draw.
 *
 * Responsibilities:
 *   tick()   advance time, compute current light position
 *   render() call canvas_render() with current parameters
 *   draw()   call canvas_draw()
 *
 * scene_render() and scene_draw() are intentionally separate so that
 * the heavy raymarching work (render) happens in the sim accumulator
 * loop and the lightweight drawing (draw) happens in the render step.
 * In practice at ≤60 fps they are called together every frame.
 */
typedef struct {
    Canvas canvas;
    float  time;
    float  light_spd;
    float  sphere_r;
    bool   paused;
} Scene;

static Vec3 scene_light(const Scene *s)
{
    float t = s->time * s->light_spd;
    return v3(cosf(t) * 3.0f, sinf(t * 0.7f) * 1.0f + 1.5f, 2.5f);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    canvas_alloc(&s->canvas, cols, rows);
    s->time      = 0.0f;
    s->light_spd = LIGHT_SPD_DEFAULT;
    s->sphere_r  = SPHERE_R_DEFAULT;
    s->paused    = false;
}

static void scene_free(Scene *s)
{
    canvas_free(&s->canvas);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    canvas_free(&s->canvas);
    canvas_alloc(&s->canvas, cols, rows);
}

static void scene_tick(Scene *s, float dt_sec)
{
    if (!s->paused) s->time += dt_sec;
}

static void scene_render(Scene *s)
{
    canvas_render(&s->canvas, s->sphere_r, scene_light(s));
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    canvas_draw(&s->canvas, cols, rows);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

/*
 * Screen — one stdscr, one doupdate, no manual double buffer.
 *
 * ncurses maintains curscr (physical terminal state) and newscr (target)
 * internally. erase() clears newscr. Drawing writes into newscr.
 * doupdate() sends the diff in one write. No extra WINDOWs needed.
 *
 * HUD written directly into stdscr after scene — always on top.
 */
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
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s)
{
    (void)s;
    endwin();
}

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(Screen *s, const Scene *sc, double fps)
{
    erase();
    scene_draw(sc, s->cols, s->rows);

    /* HUD into stdscr — drawn last, always visible */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps  spd:%.2f  r:%.2f  [%dx%d]",
             fps, sc->light_spd, sc->sphere_r,
             sc->canvas.w, sc->canvas.h);
    int hud_x = s->cols - HUD_COLS;
    if (hud_x < 0) hud_x = 0;
    attron(lumi_attr(5) | A_BOLD);
    mvprintw(0, hud_x, "%s", buf);
    attroff(lumi_attr(5) | A_BOLD);
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
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        s->paused = !s->paused;
        break;

    case ']':
        s->light_spd *= LIGHT_SPD_STEP;
        if (s->light_spd > LIGHT_SPD_MAX) s->light_spd = LIGHT_SPD_MAX;
        break;
    case '[':
        s->light_spd /= LIGHT_SPD_STEP;
        if (s->light_spd < LIGHT_SPD_MIN) s->light_spd = LIGHT_SPD_MIN;
        break;

    case '=': case '+':
        s->sphere_r *= SIZE_STEP;
        if (s->sphere_r > SIZE_MAXX) s->sphere_r = SIZE_MAXX;
        break;
    case '-':
        s->sphere_r /= SIZE_STEP;
        if (s->sphere_r < SIZE_MIN) s->sphere_r = SIZE_MIN;
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

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

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
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── sim accumulator ─────────────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }
        float alpha = (float)sim_accum / (float)tick_ns;
        (void)alpha;

        /* ── render ──────────────────────────────────────────────── */
        scene_render(&app->scene);

        /* ── HUD counter ─────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap (sleep BEFORE render so I/O doesn't drift) ── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── draw + present ──────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene, fps_display);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
