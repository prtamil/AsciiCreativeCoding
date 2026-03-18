/*
 * wireframe.c  —  ncurses full-screen 3-D ASCII wireframe shapes
 *
 * Four shapes cycle with Tab:  cube · sphere · pyramid · torus
 *
 * The shape always fills ~80 % of the terminal regardless of size.
 * No fixed-resolution virtual canvas — everything is projected directly
 * into terminal-cell coordinates at runtime.
 *
 * Aspect-ratio fix:
 *   Terminal cells are ~2× taller than wide (in physical pixels).
 *   project() compensates by halving the Y component of the projected
 *   point so vertical distances in world space map to the same physical
 *   size on screen as horizontal distances.
 *
 * Keys:
 *   q / ESC   quit
 *   Tab       cycle shapes  (cube → sphere → pyramid → torus → …)
 *   space     pause / resume
 *   ]  [      spin faster / slower
 *   =  -      zoom in / out
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra wireframe.c -o wireframe -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config    — tunables
 *   §2  clock     — monotonic ns clock + sleep
 *   §3  color     — one hue per shape
 *   §4  vec3      — 3-D math, value types
 *   §5  project   — rotation + perspective + aspect correction
 *   §6  canvas    — heap-allocated terminal-size framebuffer + Bresenham
 *   §7  shapes    — vertex/edge tables for all four shapes
 *   §8  scene     — active shape, rotation, tick, render, draw
 *   §9  screen    — single stdscr, ncurses internal double buffer
 *   §10 app       — dt loop, input, resize, cleanup
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
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN     =  5,
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,

    HUD_COLS        = 46,
    FPS_UPDATE_MS   = 500,

    MAX_VERTS       = 400,
    MAX_EDGES       = 800,
    SHAPE_COUNT     = 4,
};

/*
 * CAM_DIST  — camera distance along +Z.  The shape sits at the origin.
 * FILL      — fraction of the smaller screen dimension the shape fills.
 *             At 0.82 the shape just fits with a small margin.
 * CELL_AR   — physical cell height / cell width.
 *             Standard terminals: ~2.0.  Adjust if shape looks stretched.
 */
#define CAM_DIST     5.0f
#define FILL         0.82f
#define CELL_AR      2.0f     /* cell aspect ratio: height / width        */

#define ROT_X_DEF    0.50f    /* radians/sec                              */
#define ROT_Y_DEF    0.85f
#define ROT_STEP     1.35f
#define ROT_MIN      0.01f
#define ROT_MAX      8.0f

#define ZOOM_STEP    1.15f
#define ZOOM_MIN     0.4f
#define ZOOM_MAX     3.5f

/* Tessellation resolution — kept LOW intentionally.
 * A wireframe needs just enough lines to read as the shape.
 * Too many edges overlap at terminal resolution → looks filled/shaded.
 * Rule of thumb: circumference_in_chars / edges_per_ring >= 4 chars gap. */
#define SPHERE_STACKS    6    /* 5 visible latitude rings                */
#define SPHERE_SLICES    8    /* 8 longitude lines                       */
#define TORUS_MAJOR      12   /* 12 ring circles                         */
#define TORUS_MINOR       6   /* 6 tube circles                          */

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

typedef enum {
    COL_CUBE    = 1,
    COL_SPHERE  = 2,
    COL_PYRAMID = 3,
    COL_TORUS   = 4,
} ShapeColor;

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(COL_CUBE,     51,  COLOR_BLACK);
        init_pair(COL_SPHERE,   46,  COLOR_BLACK);
        init_pair(COL_PYRAMID,  226, COLOR_BLACK);
        init_pair(COL_TORUS,    201, COLOR_BLACK);
    } else {
        init_pair(COL_CUBE,    COLOR_CYAN,    COLOR_BLACK);
        init_pair(COL_SPHERE,  COLOR_GREEN,   COLOR_BLACK);
        init_pair(COL_PYRAMID, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(COL_TORUS,   COLOR_MAGENTA, COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  vec3                                                               */
/* ===================================================================== */

typedef struct { float x, y, z; } Vec3;

static inline Vec3  v3(float x, float y, float z) { return (Vec3){x,y,z}; }
static inline Vec3  v3mul(Vec3 a, float s) { return v3(a.x*s, a.y*s, a.z*s); }

/* ===================================================================== */
/* §5  project                                                            */
/* ===================================================================== */

/*
 * rot_yx() — rotate p:  first Y by ry, then X by rx.
 */
static Vec3 rot_yx(Vec3 p, float rx, float ry)
{
    float cy = cosf(ry), sy = sinf(ry);
    float x1 =  p.x*cy + p.z*sy;
    float z1 = -p.x*sy + p.z*cy;
    p.x = x1;  p.z = z1;

    float cx = cosf(rx), sx = sinf(rx);
    float y2 = p.y*cx - p.z*sx;
    float z2 = p.y*sx + p.z*cx;
    p.y = y2;  p.z = z2;

    return p;
}

/*
 * project_to_screen() — perspective-project one 3-D world point to
 * terminal cell coordinates (col, row).
 *
 *   fov_px   half-height of the viewport in terminal rows, computed once
 *            per frame from the terminal size and the FILL constant so
 *            the shape always occupies FILL fraction of the screen.
 *
 *   ox, oy   screen origin (centre of terminal in cell coordinates).
 *
 * The shape vertex sits in world space scaled to unit size (radius ≈ 1).
 * Perspective division by (CAM_DIST - p.z) maps world units to pixels.
 * CELL_AR divides the Y result: terminal rows are taller than cols, so
 * without correction the shape would be stretched vertically.
 *
 * Returns (col, row, z_world).  If z_world is negative (behind cam) the
 * caller skips the edge.
 */
typedef struct { float col, row, z; } P2;

static P2 project_to_screen(Vec3 p, float fov_px, float ox, float oy)
{
    float denom = CAM_DIST - p.z;
    if (denom < 0.01f) return (P2){ -1, -1, -9999 };

    float scale = fov_px / denom;

    float col =  ox + p.x * scale;
    float row =  oy - p.y * scale / CELL_AR;   /* /CELL_AR = aspect fix */

    return (P2){ col, row, p.z };
}

/*
 * fov_from_screen() — compute the fov_px that makes the shape fill FILL
 * fraction of the screen.
 *
 * The shape has unit radius 1.  At distance CAM_DIST the projected radius
 * in pixels (rows) is:  fov_px / CAM_DIST * 1.
 * We want that to equal  FILL * (rows/2).
 * So: fov_px = FILL * (rows/2) * CAM_DIST.
 *
 * We also take the column dimension into account (accounting for CELL_AR)
 * and use the smaller of the two so the shape fits in both dimensions.
 */
static float fov_from_screen(int cols, int rows, float zoom)
{
    float fov_rows = FILL * (float)(rows / 2) * CAM_DIST;
    /* columns are CELL_AR times narrower visually than rows */
    float fov_cols = FILL * (float)(cols / 2) * CAM_DIST / CELL_AR;
    float fov = fov_rows < fov_cols ? fov_rows : fov_cols;
    return fov * zoom;
}

/* ===================================================================== */
/* §6  canvas                                                             */
/* ===================================================================== */

/*
 * Canvas — heap-allocated terminal-size character grid.
 * Sized to exact terminal dimensions at runtime so shapes always fill
 * the whole screen with no fixed-resolution limitation.
 *
 * Each cell stores: ch (0 = empty) and ShapeColor.
 * canvas_draw() writes non-empty cells to the ncurses WINDOW.
 */
typedef struct {
    char       *ch;    /* [rows * cols]                                  */
    ShapeColor *col;   /* [rows * cols]                                  */
    int         cols;
    int         rows;
} Canvas;

static void canvas_alloc(Canvas *c, int cols, int rows)
{
    c->cols = cols;
    c->rows = rows;
    c->ch   = calloc((size_t)(cols * rows), sizeof(char));
    c->col  = calloc((size_t)(cols * rows), sizeof(ShapeColor));
}

static void canvas_free(Canvas *c)
{
    free(c->ch);
    free(c->col);
    *c = (Canvas){0};
}

static void canvas_clear(Canvas *c)
{
    memset(c->ch,  0, sizeof(char)       * (size_t)(c->cols * c->rows));
    memset(c->col, 0, sizeof(ShapeColor) * (size_t)(c->cols * c->rows));
}

static void canvas_set(Canvas *c, int x, int y, char ch, ShapeColor col)
{
    if (x < 0 || x >= c->cols || y < 0 || y >= c->rows) return;
    int i = y * c->cols + x;
    c->ch [i] = ch;
    c->col[i] = col;
}

/*
 * canvas_line() — Bresenham integer line, all eight octants.
 *
 * ch_override:
 *   0   — pick char by slope: '-' '|' '/' '\' for straight edges
 *   'o' — draw 'o' at every pixel (for curved shapes like sphere/torus)
 *
 * Straight-edge shapes (cube, pyramid) use slope chars so the edges
 * read as clean geometric lines.  Curved shapes use a uniform dot so
 * the varying slope along each arc doesn't produce visual noise.
 */
static void canvas_line(Canvas *c,
                         int x0, int y0, int x1, int y1,
                         char ch_override, ShapeColor col)
{
    int dx = abs(x1 - x0),  dy = -abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    /* Slope-based character (used only when ch_override == 0). */
    char slope_ch;
    if (ch_override != 0) {
        slope_ch = ch_override;
    } else {
        int adx = abs(x1 - x0), ady = abs(y1 - y0);
        if (adx == 0)
            slope_ch = '|';
        else if (ady == 0)
            slope_ch = '-';
        else {
            float slope = (float)ady / (float)adx;
            if (slope < 0.5f)
                slope_ch = '-';
            else if (slope < 2.0f)
                slope_ch = (sx == sy) ? '\\' : '/';
            else
                slope_ch = '|';
        }
    }

    for (;;) {
        canvas_set(c, x0, y0, slope_ch, col);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy;  x0 += sx; }
        if (e2 <= dx) { err += dx;  y0 += sy; }
    }
}

static void canvas_draw(const Canvas *c)
{
    int total = c->cols * c->rows;
    for (int i = 0; i < total; i++) {
        char ch = c->ch[i];
        if (!ch) continue;

        int y = i / c->cols;
        int x = i % c->cols;

        attr_t attr = COLOR_PAIR(c->col[i]) | A_BOLD;
        attron(attr);
        mvaddch(y, x, (chtype)(unsigned char)ch);
        attroff(attr);
    }
}

/* ===================================================================== */
/* §7  shapes                                                             */
/* ===================================================================== */

typedef struct { int a, b; } Edge;

typedef struct {
    const char *name;
    int         nv;
    int         ne;
    Vec3        verts[MAX_VERTS];
    Edge        edges[MAX_EDGES];
    ShapeColor  color;
    bool        curved;   /* true = dot rendering; false = line chars  */
} Shape;

/* ---- cube ---- */
static void shape_build_cube(Shape *s)
{
    s->name  = "cube";
    s->color = COL_CUBE;
    s->curved = false;
    s->nv = 8;  s->ne = 12;

    for (int i = 0; i < 8; i++)
        s->verts[i] = v3((i&1)?1.f:-1.f, (i&2)?1.f:-1.f, (i&4)?1.f:-1.f);

    Edge e[] = {
        {0,1},{1,3},{3,2},{2,0},   /* front face  */
        {4,5},{5,7},{7,6},{6,4},   /* back face   */
        {0,4},{1,5},{2,6},{3,7}    /* pillars     */
    };
    memcpy(s->edges, e, sizeof e);
}

/* ---- sphere ---- */
static void shape_build_sphere(Shape *s)
{
    s->name  = "sphere";
    s->color = COL_SPHERE;
    s->curved = true;
    s->nv = 0;  s->ne = 0;

    int ST = SPHERE_STACKS, SL = SPHERE_SLICES;

    for (int st = 0; st <= ST; st++) {
        float phi = (float)M_PI * st / ST;
        for (int sl = 0; sl < SL; sl++) {
            float th = 2.f*(float)M_PI*sl/SL;
            s->verts[s->nv++] = v3(
                sinf(phi)*cosf(th), cosf(phi), sinf(phi)*sinf(th));
        }
    }
    /* longitude lines */
    for (int sl = 0; sl < SL; sl++)
        for (int st = 0; st < ST; st++)
            s->edges[s->ne++] = (Edge){st*SL+sl, (st+1)*SL+sl};
    /* latitude rings */
    for (int st = 1; st < ST; st++)
        for (int sl = 0; sl < SL; sl++)
            s->edges[s->ne++] = (Edge){st*SL+sl, st*SL+(sl+1)%SL};
}

/* ---- pyramid ---- */
static void shape_build_pyramid(Shape *s)
{
    s->name  = "pyramid";
    s->color = COL_PYRAMID;
    s->curved = false;
    s->nv = 5;  s->ne = 8;

    s->verts[0] = v3(-1,-1,-1);
    s->verts[1] = v3( 1,-1,-1);
    s->verts[2] = v3( 1,-1, 1);
    s->verts[3] = v3(-1,-1, 1);
    s->verts[4] = v3( 0, 1, 0);

    Edge e[] = {{0,1},{1,2},{2,3},{3,0},{0,4},{1,4},{2,4},{3,4}};
    memcpy(s->edges, e, sizeof e);
}

/* ---- torus ---- */
static void shape_build_torus(Shape *s)
{
    s->name  = "torus";
    s->color = COL_TORUS;
    s->curved = true;
    s->nv = 0;  s->ne = 0;

    int   M = TORUS_MAJOR, m = TORUS_MINOR;
    float R = 0.65f, r = 0.28f;

    for (int i = 0; i < M; i++) {
        float phi = 2.f*(float)M_PI*i/M;
        float cp = cosf(phi), sp = sinf(phi);
        for (int j = 0; j < m; j++) {
            float th = 2.f*(float)M_PI*j/m;
            s->verts[s->nv++] = v3(
                (R+r*cosf(th))*cp, r*sinf(th), (R+r*cosf(th))*sp);
        }
    }
    for (int i = 0; i < M; i++)
        for (int j = 0; j < m; j++) {
            s->edges[s->ne++] = (Edge){i*m+j, i*m+(j+1)%m};      /* tube  */
            s->edges[s->ne++] = (Edge){i*m+j, ((i+1)%M)*m+j};    /* ring  */
        }
}

static const char * const k_names[SHAPE_COUNT] = {
    "cube","sphere","pyramid","torus"
};

/* ===================================================================== */
/* §8  scene                                                              */
/* ===================================================================== */

typedef struct {
    Shape  shapes[SHAPE_COUNT];
    Canvas canvas;
    int    active;
    float  rx, ry;
    float  rot_x, rot_y;
    float  zoom;
    bool   paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    shape_build_cube    (&s->shapes[0]);
    shape_build_sphere  (&s->shapes[1]);
    shape_build_pyramid (&s->shapes[2]);
    shape_build_torus   (&s->shapes[3]);

    s->active = 0;
    s->rx     = 0.4f;
    s->ry     = 0.6f;
    s->rot_x  = ROT_X_DEF;
    s->rot_y  = ROT_Y_DEF;
    s->zoom   = 1.0f;
    s->paused = false;

    canvas_alloc(&s->canvas, cols, rows);
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
    s->rx += s->rot_x * dt_sec;
    s->ry += s->rot_y * dt_sec;
}

/*
 * scene_render() — the core pipeline.
 *
 * 1. Compute fov_px once for this frame so the shape fills the screen.
 * 2. Transform all vertices: scale to unit → rotate → project.
 * 3. Draw each edge as a Bresenham line with slope-based character.
 *
 * The shape vertices are in unit coordinates (max radius ≈ 1).
 * fov_from_screen() maps that unit radius to FILL * screen_half_height
 * terminal rows, so the shape always fills the desired fraction of the
 * screen regardless of terminal size.
 */
static void scene_render(Scene *s)
{
    canvas_clear(&s->canvas);

    int   cols  = s->canvas.cols;
    int   rows  = s->canvas.rows;
    float ox    = (float)cols * 0.5f;
    float oy    = (float)rows * 0.5f;
    float fov   = fov_from_screen(cols, rows, s->zoom);

    const Shape *sh = &s->shapes[s->active];

    /* Pre-project all vertices. */
    P2 proj[MAX_VERTS];
    for (int i = 0; i < sh->nv; i++) {
        Vec3 v = rot_yx(sh->verts[i], s->rx, s->ry);
        proj[i] = project_to_screen(v, fov, ox, oy);
    }

    /* Draw edges — slope chars for flat shapes, dots for curved. */
    char ch_override = sh->curved ? 'o' : 0;

    for (int e = 0; e < sh->ne; e++) {
        P2 pa = proj[sh->edges[e].a];
        P2 pb = proj[sh->edges[e].b];

        if (pa.z < -CAM_DIST+0.1f || pb.z < -CAM_DIST+0.1f) continue;

        canvas_line(&s->canvas,
                    (int)(pa.col + 0.5f), (int)(pa.row + 0.5f),
                    (int)(pb.col + 0.5f), (int)(pb.row + 0.5f),
                    ch_override, sh->color);
    }
}

static void scene_draw(const Scene *s)
{
    canvas_draw(&s->canvas);
}

/* ===================================================================== */
/* §9  screen                                                             */
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
    scene_draw(sc);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps  %-7s  spd:%.1f  zoom:%.1f",
             fps, k_names[sc->active], sc->rot_y, sc->zoom);

    ShapeColor col = sc->shapes[sc->active].color;
    int hud_x = s->cols - HUD_COLS;
    if (hud_x < 0) hud_x = 0;
    attron(COLOR_PAIR(col) | A_BOLD);
    mvprintw(0, hud_x, "%s", buf);
    attroff(COLOR_PAIR(col) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §10 app                                                                */
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

    case '\t':
        s->active = (s->active + 1) % SHAPE_COUNT;
        s->rx = 0.4f;  s->ry = 0.6f;
        break;

    case ' ':
        s->paused = !s->paused;
        break;

    case ']':
        s->rot_x *= ROT_STEP;  s->rot_y *= ROT_STEP;
        if (s->rot_x > ROT_MAX) s->rot_x = ROT_MAX;
        if (s->rot_y > ROT_MAX) s->rot_y = ROT_MAX;
        break;
    case '[':
        s->rot_x /= ROT_STEP;  s->rot_y /= ROT_STEP;
        if (s->rot_x < ROT_MIN) s->rot_x = ROT_MIN;
        if (s->rot_y < ROT_MIN) s->rot_y = ROT_MIN;
        break;

    case '=': case '+':
        s->zoom *= ZOOM_STEP;
        if (s->zoom > ZOOM_MAX) s->zoom = ZOOM_MAX;
        break;
    case '-':
        s->zoom /= ZOOM_STEP;
        if (s->zoom < ZOOM_MIN) s->zoom = ZOOM_MIN;
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
