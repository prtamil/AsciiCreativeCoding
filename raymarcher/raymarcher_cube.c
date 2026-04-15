/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cube.c  —  ncurses raymarched ASCII spinning cube
 *
 * Renders a 3-D Phong-shaded cube using raymarching at a deliberately
 * low virtual resolution that is then block-upscaled to the terminal,
 * so the cube always looks correct regardless of terminal dimensions.
 *
 * The cube rotates continuously around two axes.  A point light orbits
 * the cube, painting moving specular highlights across the flat faces.
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume rotation
 *   ]  [      spin faster / slower
 *   =  -      larger / smaller cube
 *   l / L     light faster / slower
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra cube.c -o cube -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config      — every tunable constant
 *   §2  clock       — monotonic ns clock + sleep
 *   §3  color       — luminance pairs; 256-color + 8-color fallback
 *   §4  vec3        — inline 3-D vector math (value types, no heap)
 *   §5  raymarch    — SDF, normal estimator, march loop, Phong shading
 *                     (pure math — no ncurses, no global state)
 *   §6  canvas      — virtual low-res framebuffer + upscale to terminal
 *   §7  scene       — owns canvas + cube rotation + tick + render + draw
 *   §8  screen      — single stdscr, ncurses internal double buffer
 *   §9  app         — dt loop, input, resize, cleanup
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : SDF raymarching with a box SDF (cube primitive).
 *                  Box SDF formula (Quilez): for a box with half-extents b:
 *                    d = |p| − b    (component-wise absolute value)
 *                    sdf = length(max(d, 0)) + min(max(d.x, d.y, d.z), 0)
 *                  The first term handles outside distance; the second handles
 *                  inside (negative = inside the box).
 *
 * Math           : The cube's 6 faces have normals ±(1,0,0), ±(0,1,0), ±(0,0,1).
 *                  Normal estimation via finite-difference gradient is exact here
 *                  because the box SDF is piecewise linear.
 *                  Rotation: 3×3 rotation matrix built from Euler angles A, B.
 *                  Point p in world space → box space: p' = R^T · p.
 *
 * Rendering      : Block-upscaled virtual canvas (VIRT_W × VIRT_H) avoids
 *                  the non-square cell aspect problem.  Each virtual pixel maps
 *                  to (BLOCK_W × BLOCK_H) terminal cells.  Resolution is a
 *                  trade-off: lower VIRT → faster render, larger pixelated blocks.
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

    HUD_COLS        = 42,
    FPS_UPDATE_MS   = 500,
};

/*
 * Terminal cell aspect ratio — same approach as sphere.c.
 *
 * CELL_W = CELL_H = 1: one canvas pixel per terminal cell.
 * CELL_ASPECT = 2.0: physical height/width ratio of a terminal cell.
 *
 * Aspect correction lives inside rm_cast_pixel() — the v ray
 * component is multiplied by phys_aspect so the cube's faces
 * appear square, not rectangular.
 *
 * canvas_w = cols, canvas_h = rows — full terminal resolution.
 */
#define CELL_W       1
#define CELL_H       1
#define CELL_ASPECT  2.0f

static inline int canvas_w_from_cols(int cols) { return cols / CELL_W; }
static inline int canvas_h_from_rows(int rows) { return rows / CELL_H; }

/* ---- raymarching ---- */
#define RM_MAX_STEPS    80
#define RM_HIT_EPS      0.003f
#define RM_MAX_DIST     20.0f

/*
 * Normal estimation epsilon.
 * We use the tetrahedron technique (4 SDF samples, no shared axis samples)
 * which gives a better normal than the central-differences 6-sample method
 * and avoids over-rounding the cube edges.
 */
#define RM_NORM_EPS     0.001f

/* ---- camera ---- */
#define CAM_Z           4.5f    /* camera at (0, 0, CAM_Z) looking -Z  */
#define FOV_HALF_TAN    0.65f   /* tan(fov/2); 0.65 ≈ 66° fov          */

/* ---- cube ---- */
#define CUBE_H_DEFAULT  0.9f    /* cube half-size (world units)         */
#define SIZE_STEP       1.15f
#define SIZE_MIN        0.15f
#define SIZE_MAXX        2.5f

/* ---- rotation ---- */
#define ROT_X_DEFAULT   0.7f    /* radians/sec around X axis            */
#define ROT_Y_DEFAULT   1.1f    /* radians/sec around Y axis            */
#define ROT_STEP        1.3f    /* ] / [ multiplier                     */
#define ROT_MIN         0.02f
#define ROT_MAX         10.0f

/* ---- light ---- */
#define LIGHT_SPD_DEFAULT  0.6f   /* radians/sec orbit                  */
#define LIGHT_SPD_STEP     1.3f
#define LIGHT_SPD_MIN      0.02f
#define LIGHT_SPD_MAX      8.0f

/* ---- Phong ---- */
#define KA    0.08f   /* ambient                                        */
#define KD    0.72f   /* diffuse                                        */
#define KS    0.65f   /* specular (higher than sphere for sharp edges)  */
#define SHIN  50.0f   /* shininess exponent                             */

#define NS_PER_SEC    1000000000LL
#define NS_PER_MS     1000000LL
#define TICK_NS(f)    (NS_PER_SEC / (f))

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
 * Eight luminance levels mapped to ncurses color pairs 1..8.
 * 256-color: xterm grey ramp 235..255 (8 evenly spaced steps).
 * 8-color:   A_DIM / normal / A_BOLD on COLOR_WHITE.
 */
enum { LUMI_N = 8 };

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        static const int g[LUMI_N] = {235, 238, 241, 244, 247, 250, 253, 255};
        for (int i = 0; i < LUMI_N; i++)
            init_pair(i + 1, g[i], COLOR_BLACK);
    } else {
        for (int i = 0; i < LUMI_N; i++)
            init_pair(i + 1, COLOR_WHITE, COLOR_BLACK);
    }
}

static attr_t lumi_attr(int l)
{
    if (l < 0)        l = 0;
    if (l > LUMI_N-1) l = LUMI_N-1;
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

typedef struct { float x, y, z; } Vec3;

static inline Vec3  v3(float x, float y, float z) { return (Vec3){x,y,z}; }
static inline Vec3  v3add(Vec3 a, Vec3 b)  { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline Vec3  v3sub(Vec3 a, Vec3 b)  { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline Vec3  v3mul(Vec3 a, float s) { return v3(a.x*s,   a.y*s,   a.z*s);   }
static inline float v3dot(Vec3 a, Vec3 b)  { return a.x*b.x + a.y*b.y + a.z*b.z;   }
static inline float v3len(Vec3 a)          { return sqrtf(v3dot(a, a));              }
static inline Vec3  v3norm(Vec3 a)
{
    float l = v3len(a);
    return l > 1e-7f ? v3mul(a, 1.0f / l) : v3(0, 0, 1);
}

/* Component-wise absolute value — used in cube SDF. */
static inline Vec3 v3abs(Vec3 a)
{
    return v3(fabsf(a.x), fabsf(a.y), fabsf(a.z));
}

/* Component-wise max(v, 0) — used in cube SDF. */
static inline Vec3 v3max0(Vec3 a)
{
    return v3(fmaxf(a.x, 0.0f), fmaxf(a.y, 0.0f), fmaxf(a.z, 0.0f));
}

/* ===================================================================== */
/* §5  raymarch  (pure math — no ncurses, no global state)               */
/* ===================================================================== */

/*
 * Rotation matrices — applied to the ray-space point before calling the
 * SDF, so the cube rotates while the SDF itself stays at the origin.
 *
 * We rotate the *query point* by the inverse of the cube's orientation.
 * For orthogonal rotation matrices the inverse equals the transpose, so
 * rotating the point by (-rx, -ry) achieves the same result as rotating
 * the cube by (rx, ry).
 *
 * rm_rotate_yx() applies Y rotation then X rotation to point p.
 * (Equivalent to Ry * Rx applied to the cube, inverted.)
 */
static Vec3 rm_rotate_yx(Vec3 p, float rx, float ry)
{
    /* Rotate around Y axis by ry. */
    float cy = cosf(ry), sy = sinf(ry);
    float px = p.x * cy + p.z * sy;
    float pz = -p.x * sy + p.z * cy;
    p.x = px; p.z = pz;

    /* Rotate around X axis by rx. */
    float cx = cosf(rx), sx = sinf(rx);
    float py = p.y * cx - p.z * sx;
    pz       = p.y * sx + p.z * cx;
    p.y = py; p.z = pz;

    return p;
}

/*
 * sdf_box() — signed distance from point p to an axis-aligned box
 * centred at the origin with half-extents (h, h, h).
 *
 * The exact box SDF (Inigo Quilez):
 *   q = |p| - h
 *   d = |max(q,0)| + min(max(q.x, q.y, q.z), 0)
 *
 *   Outside: d > 0  (distance to nearest face/edge/corner)
 *   Inside:  d < 0  (negative distance to nearest face)
 *   Surface: d = 0
 */
static float sdf_box(Vec3 p, float h)
{
    Vec3  q  = v3sub(v3abs(p), v3(h, h, h));
    Vec3  qp = v3max0(q);
    float outside = v3len(qp);
    float inside  = fminf(fmaxf(q.x, fmaxf(q.y, q.z)), 0.0f);
    return outside + inside;
}

/*
 * sdf_scene() — the scene SDF.
 * Rotates the query point by the inverse cube orientation, then
 * evaluates sdf_box.  rx and ry are the current cube rotation angles.
 */
static float sdf_scene(Vec3 p, float rx, float ry, float cube_h)
{
    Vec3 lp = rm_rotate_yx(p, rx, ry);
    return sdf_box(lp, cube_h);
}

/*
 * rm_normal() — estimate the surface normal at point p using the
 * tetrahedron technique (4 SDF evaluations, 3 subtraction chains).
 *
 * More stable than central differences for sharp-edged shapes because
 * it avoids sampling across corners where the SDF is non-smooth.
 *
 * Reference: Inigo Quilez, "normals for an SDF"
 */
static Vec3 rm_normal(Vec3 p, float rx, float ry, float cube_h)
{
    const float e = RM_NORM_EPS;

    /* Four tetrahedral sample offsets. */
    Vec3 k0 = v3( e, -e, -e);
    Vec3 k1 = v3(-e, -e,  e);
    Vec3 k2 = v3(-e,  e, -e);
    Vec3 k3 = v3( e,  e,  e);

    float d0 = sdf_scene(v3add(p, k0), rx, ry, cube_h);
    float d1 = sdf_scene(v3add(p, k1), rx, ry, cube_h);
    float d2 = sdf_scene(v3add(p, k2), rx, ry, cube_h);
    float d3 = sdf_scene(v3add(p, k3), rx, ry, cube_h);

    Vec3 n = v3add(
                v3add(v3mul(k0, d0), v3mul(k1, d1)),
                v3add(v3mul(k2, d2), v3mul(k3, d3))
             );
    return v3norm(n);
}

/*
 * rm_march() — march a ray from ro in direction rd until it hits the
 * scene (cube at rotation rx,ry) or escapes.
 *
 * Returns hit distance t, or -1 on miss.
 * This function knows only about math — no terminal, no ncurses.
 */
static float rm_march(Vec3 ro, Vec3 rd, float rx, float ry, float cube_h)
{
    float t = 0.5f;   /* start slightly away from camera to skip self-hit */
    for (int i = 0; i < RM_MAX_STEPS; i++) {
        Vec3  p = v3add(ro, v3mul(rd, t));
        float d = sdf_scene(p, rx, ry, cube_h);
        if (d < RM_HIT_EPS)  return t;
        if (t > RM_MAX_DIST) return -1.0f;
        t += d;
    }
    return -1.0f;
}

/*
 * rm_shade() — Phong illumination at a surface point.
 *
 *   N   surface normal at hit
 *   L   direction from hit toward light (normalised)
 *   V   direction from hit toward camera (normalised)
 *
 * I = Ka  +  Kd*(N·L)  +  Ks*(R·V)^shininess
 * where R = reflect(-L, N) = 2*(N·L)*N - (-L) = 2*(N·L)*N + L ... wait:
 *   R = 2*(N·L)*N - L  when L points away from surface
 *   We use L pointing toward the light, so reflect the *negated* L:
 *   R = reflect(-L, N) = -L + 2*(N·(-L))*N ... simpler: R = 2*ndl*N - L
 */
static float rm_shade(Vec3 N, Vec3 hit, Vec3 cam, Vec3 light)
{
    Vec3  L   = v3norm(v3sub(light, hit));
    Vec3  V   = v3norm(v3sub(cam,   hit));

    float ndl = fmaxf(0.0f, v3dot(N, L));
    Vec3  R   = v3sub(v3mul(N, 2.0f * ndl), L);
    float spec = powf(fmaxf(0.0f, v3dot(R, V)), SHIN);

    float I = KA + KD * ndl + KS * spec;
    return I > 1.0f ? 1.0f : I;
}

/*
 * rm_cast_pixel() — cast one ray for canvas pixel (px, py).
 *
 * Aspect correction: multiply v by phys_aspect so the cube's faces
 * appear square on a terminal where rows are ~2× taller than columns.
 */
static float rm_cast_pixel(int px, int py,
                             int cw, int ch,
                             float rx, float ry, float cube_h,
                             Vec3  light)
{
    float u =  ((float)px + 0.5f) / (float)cw * 2.0f - 1.0f;
    float v = -((float)py + 0.5f) / (float)ch * 2.0f + 1.0f;

    float phys_aspect = ((float)ch * CELL_ASPECT) / (float)cw;

    Vec3 ro = v3(0.0f, 0.0f, CAM_Z);
    Vec3 rd = v3norm(v3(u * FOV_HALF_TAN,
                        v * FOV_HALF_TAN * phys_aspect,
                        -1.0f));

    float t = rm_march(ro, rd, rx, ry, cube_h);
    if (t < 0.0f) return -1.0f;

    Vec3 hit = v3add(ro, v3mul(rd, t));
    Vec3 N   = rm_normal(hit, rx, ry, cube_h);
    return rm_shade(N, hit, ro, light);
}

/* ===================================================================== */
/* §6  canvas                                                             */
/* ===================================================================== */

#define CANVAS_MISS  -1

typedef struct {
    int  w, h;
    int *pixels;    /* [h * w] heap-allocated                           */
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

static const char k_ramp[] = " .,:;+=oxOX#@";
#define RAMP_N (int)(sizeof k_ramp - 1)

static void canvas_clear(Canvas *c)
{
    for (int i = 0; i < c->w * c->h; i++)
        c->pixels[i] = CANVAS_MISS;
}

static void canvas_render(Canvas *c,
                            float rx, float ry, float cube_h,
                            Vec3  light)
{
    canvas_clear(c);
    for (int py = 0; py < c->h; py++) {
        for (int px = 0; px < c->w; px++) {
            float I = rm_cast_pixel(px, py, c->w, c->h,
                                     rx, ry, cube_h, light);
            if (I < 0.0f) {
                c->pixels[py * c->w + px] = CANVAS_MISS;
            } else {
                int idx = (int)(I * (float)(RAMP_N - 1) + 0.5f);
                if (idx >= RAMP_N) idx = RAMP_N - 1;
                c->pixels[py * c->w + px] = idx;
            }
        }
    }
}

/*
 * canvas_draw() — write pixels directly into stdscr.
 * One canvas pixel = one terminal cell (CELL_W=1, CELL_H=1).
 * Canvas is centred in the terminal.
 */
static void canvas_draw(const Canvas *c, int term_cols, int term_rows)
{
    int off_x = (term_cols - c->w) / 2;
    int off_y = (term_rows - c->h) / 2;

    for (int vy = 0; vy < c->h; vy++) {
        for (int vx = 0; vx < c->w; vx++) {
            int idx = c->pixels[vy * c->w + vx];
            if (idx == CANVAS_MISS) continue;

            int tx = off_x + vx;
            int ty = off_y + vy;
            if (tx < 0 || tx >= term_cols) continue;
            if (ty < 0 || ty >= term_rows) continue;

            char   ch   = k_ramp[idx];
            attr_t attr = lumi_attr((idx * LUMI_N) / RAMP_N);
            attron(attr);
            mvaddch(ty, tx, (chtype)(unsigned char)ch);
            attroff(attr);
        }
    }
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * Scene — animation state + canvas.
 *
 *   rx, ry      current cube rotation angles (radians)
 *   rot_x/y     rotation speed (radians/sec) — changed by ] / [
 *   light_spd   light orbit speed (radians/sec) — changed by l / L
 *   cube_h      cube half-extent — changed by = / -
 *   time        seconds elapsed; drives light position
 *   paused      when true tick() does not advance anything
 */
typedef struct {
    Canvas canvas;
    float  rx, ry;       /* current rotation angles                    */
    float  rot_x;        /* rotation speed X (rad/sec)                 */
    float  rot_y;        /* rotation speed Y (rad/sec)                 */
    float  light_spd;    /* light orbit speed (rad/sec)                */
    float  cube_h;       /* cube half-size                             */
    float  time;         /* total elapsed seconds                      */
    bool   paused;
} Scene;

/*
 * scene_light() — compute the current light position.
 *
 * The light orbits in a tilted ellipse around the cube:
 *   horizontal: radius 3.5 in the XZ plane
 *   vertical:   oscillates ±1 around Y=2 (always above the cube)
 * This gives continuously changing diffuse + specular across all faces.
 */
static Vec3 scene_light(const Scene *s)
{
    float t = s->time * s->light_spd;
    return v3(cosf(t) * 3.5f,
              sinf(t * 0.6f) + 2.0f,
              sinf(t) * 3.5f + 1.0f);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    canvas_alloc(&s->canvas, cols, rows);
    s->rx        = 0.3f;
    s->ry        = 0.5f;
    s->rot_x     = ROT_X_DEFAULT;
    s->rot_y     = ROT_Y_DEFAULT;
    s->light_spd = LIGHT_SPD_DEFAULT;
    s->cube_h    = CUBE_H_DEFAULT;
    s->time      = 0.0f;
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
    if (s->paused) return;
    s->rx   += s->rot_x * dt_sec;
    s->ry   += s->rot_y * dt_sec;
    s->time += dt_sec;
}

static void scene_render(Scene *s)
{
    canvas_render(&s->canvas,
                  s->rx, s->ry, s->cube_h,
                  scene_light(s));
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    canvas_draw(&s->canvas, cols, rows);
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
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

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

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps  rx:%.1f ry:%.1f  spd:%.1f  h:%.2f",
             fps, sc->rx, sc->ry, sc->rot_y, sc->cube_h);
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
        s->rot_x *= ROT_STEP;   s->rot_y *= ROT_STEP;
        if (s->rot_x > ROT_MAX) s->rot_x = ROT_MAX;
        if (s->rot_y > ROT_MAX) s->rot_y = ROT_MAX;
        break;
    case '[':
        s->rot_x /= ROT_STEP;   s->rot_y /= ROT_STEP;
        if (s->rot_x < ROT_MIN) s->rot_x = ROT_MIN;
        if (s->rot_y < ROT_MIN) s->rot_y = ROT_MIN;
        break;

    case '=': case '+':
        s->cube_h *= SIZE_STEP;
        if (s->cube_h > SIZE_MAXX) s->cube_h = SIZE_MAXX;
        break;
    case '-':
        s->cube_h /= SIZE_STEP;
        if (s->cube_h < SIZE_MIN) s->cube_h = SIZE_MIN;
        break;

    case 'l':
        s->light_spd *= LIGHT_SPD_STEP;
        if (s->light_spd > LIGHT_SPD_MAX) s->light_spd = LIGHT_SPD_MAX;
        break;
    case 'L':
        s->light_spd /= LIGHT_SPD_STEP;
        if (s->light_spd < LIGHT_SPD_MIN) s->light_spd = LIGHT_SPD_MIN;
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
