/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * duo_poly.c — Dual Turtle Polygon Animator
 *
 * Two turtles draw regular polygons step-by-step in the terminal:
 *   Turtle A (cyan)    — left half,  starts as a triangle (3 sides)
 *   Turtle B (magenta) — right half, starts as a pentagon (5 sides)
 *
 * Each sim tick advances one polygon edge per turtle — you watch the pen
 * move vertex to vertex.  After both polygons complete, a 2-second pause
 * fires and both auto-reset, each gaining one side (3 → 4 → … → 12 → 3).
 *
 * The turtle head (@) shows the current pen position.  Line characters
 * (-  |  /  \) reflect each edge's true heading for a natural look.
 *
 * Framework: follows framework.c §1–§8 structure exactly.
 *
 * ─────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  coords   — aspect-ratio correction (CELL_W/CELL_H ≈ 0.5)
 *   §5  entity   — Turtle struct + tick + draw
 *   §6  scene
 *   §7  screen
 *   §8  app
 * ─────────────────────────────────────────────────────────────────────
 *
 * Keys:
 *   q / ESC     quit
 *   space       pause / resume
 *   r           reset both turtles (keep current sides)
 *   a / z       turtle A: +1 / -1 sides  (3–12)
 *   s / x       turtle B: +1 / -1 sides  (3–12)
 *   + / =       draw faster
 *   -           draw slower
 *   ] / [       sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra duo_poly.c -o duo_poly -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Regular polygon via turtle graphics.  A regular n-gon
 *                  inscribed in a circle of radius R has vertices at angles
 *                  θ_k = θ_0 + k·(2π/n).  The turtle starts at vertex 0
 *                  and walks to vertex 1, 2, … n (= vertex 0) in sequence.
 *                  Each edge is rasterised with a DDA line fill so no cell
 *                  gaps appear at any step size.
 *
 * Aspect fix     : Terminal cells are ~2× taller than wide (CELL_H/CELL_W).
 *                  The polygon Y coordinates are scaled by ASPECT=0.5 so the
 *                  shape is visually circular/square rather than vertically
 *                  compressed.  Both turtles use the same correction.
 *
 * Timing         : Fixed-step accumulator (framework.c §8).  Each tick
 *                  decrements a per-turtle edge timer; when it expires the
 *                  edge index advances and the timer resets.  Speed is
 *                  controlled by edges_per_second (eps), adjustable via +/-.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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
    SIM_FPS_MIN     =   5,
    SIM_FPS_DEFAULT =  30,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    =   5,

    N_COLORS        =   7,
    SIDES_MIN       =   3,
    SIDES_MAX       =  12,

    HUD_COLS        =  64,
    FPS_UPDATE_MS   = 500,
};

#define EPS_DEFAULT   1.5f   /* edges per second (default drawing speed) */
#define EPS_MIN       0.3f
#define EPS_MAX      12.0f
#define RESET_DELAY   2.0f   /* seconds to wait after both polygons done */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

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
 * Color pairs:
 *   1 → cyan      (turtle A edges + label)
 *   2 → magenta   (turtle B edges + label)
 *   3 → yellow    (HUD, completion notice)
 *   4 → green     (polygon names)
 *   5 → red       (DONE flash)
 *   6 → blue      (hint bar, divider)
 *   7 → white     (turtle @ head)
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(1,  51, COLOR_BLACK);   /* cyan              */
        init_pair(2, 201, COLOR_BLACK);   /* magenta           */
        init_pair(3, 226, COLOR_BLACK);   /* yellow            */
        init_pair(4,  46, COLOR_BLACK);   /* green             */
        init_pair(5, 196, COLOR_BLACK);   /* red               */
        init_pair(6,  21, COLOR_BLACK);   /* blue              */
        init_pair(7, 255, COLOR_BLACK);   /* bright white      */
    } else {
        init_pair(1, COLOR_CYAN,    COLOR_BLACK);
        init_pair(2, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(5, COLOR_RED,     COLOR_BLACK);
        init_pair(6, COLOR_BLUE,    COLOR_BLACK);
        init_pair(7, COLOR_WHITE,   COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords — aspect-ratio correction for isotropic turtle motion       */
/* ===================================================================== */

/*
 * Terminal cells are ~2× taller than wide.
 * We multiply turtle Y positions by ASPECT = CELL_W/CELL_H ≈ 0.5 so that
 * equal column and row distances represent equal physical pixel distances.
 * A polygon drawn this way looks round/square, not vertically stretched.
 */
#define CELL_W   8
#define CELL_H  16
#define ASPECT   ((float)CELL_W / (float)CELL_H)   /* 0.5 */

/*
 * angle_char() — ASCII line character that best represents edge direction.
 *   '-'  for near-horizontal
 *   '|'  for near-vertical
 *   '/'  for up-right / down-left diagonal
 *   '\'  for down-right / up-left diagonal
 */
static char angle_char(float angle)
{
    float a = fmodf(angle, (float)M_PI);
    if (a < 0.0f) a += (float)M_PI;
    if (a < (float)M_PI / 8.0f || a >= 7.0f * (float)M_PI / 8.0f) return '-';
    if (a < 3.0f * (float)M_PI / 8.0f)                             return '/';
    if (a < 5.0f * (float)M_PI / 8.0f)                             return '|';
    return '\\';
}

/* ===================================================================== */
/* §5  entity — Turtle                                                    */
/* ===================================================================== */

/*
 * Turtle — state for one polygon-drawing turtle.
 *
 *   cx, cy      polygon center in cell coordinates (cols, rows)
 *   radius      circumradius in columns (aspect-corrected for Y)
 *   sides       number of polygon sides
 *   edge        edges drawn so far: 0 = pen at vertex 0, not started
 *                                   k = k edges drawn, pen at vertex k
 *                                   sides = polygon complete
 *   edge_timer  seconds remaining until next edge is drawn
 *   eps         edges per second (drawing rate)
 *   start_angle first vertex angle (radians); −π/2 places it at the top
 *   cpair       ncurses color pair index
 *   done        true when edge == sides
 */
typedef struct {
    float cx, cy;
    float radius;
    int   sides;
    int   edge;
    float edge_timer;
    float eps;
    float start_angle;
    int   cpair;
    bool  done;
} Turtle;

/*
 * turtle_init() — position and reset one turtle from terminal dimensions.
 *
 * half = 0 → left half (center at cols/4)
 * half = 1 → right half (center at 3*cols/4)
 *
 * Radius is chosen so the polygon fits within its half-screen:
 *   horizontal: radius ≤ cols * 0.21  (leave margin from divider and edge)
 *   vertical:   radius * ASPECT ≤ (rows-4) * 0.40  → radius ≤ that / ASPECT
 */
static void turtle_init(Turtle *t, int cols, int rows,
                        int half, int sides, int cpair, float eps)
{
    float cx = (half == 0) ? (float)cols * 0.25f : (float)cols * 0.75f;
    float cy = (float)(rows - 2) * 0.5f + 1.5f;
    float max_r_x = (float)cols * 0.21f;
    float max_r_y = (float)(rows - 4) * 0.40f / ASPECT;

    t->cx          = cx;
    t->cy          = cy;
    t->radius      = fminf(max_r_x, max_r_y) * 0.85f;
    t->sides       = sides;
    t->edge        = 0;
    t->eps         = eps;
    t->edge_timer  = 1.0f / eps;
    t->start_angle = -(float)M_PI / 2.0f;   /* first vertex at top */
    t->cpair       = cpair;
    t->done        = false;
}

/*
 * turtle_tick() — advance edge timer by dt seconds.
 * When timer expires, move pen to next vertex (edge++).
 */
static void turtle_tick(Turtle *t, float dt)
{
    if (t->done) return;
    t->edge_timer -= dt;
    while (t->edge_timer <= 0.0f) {
        t->edge++;
        if (t->edge >= t->sides) {
            t->done       = true;
            t->edge       = t->sides;   /* cap — closing edge is drawn */
            break;
        }
        t->edge_timer += 1.0f / t->eps;
    }
}

/*
 * poly_vertex() — compute cell position of vertex i (0-based, wraps at n).
 *
 * Vertex i is at angle:  start_angle + i * (2π/sides)
 * Y is scaled by ASPECT to correct for non-square cells.
 */
static void poly_vertex(const Turtle *t, int i, float *vx, float *vy)
{
    float a = t->start_angle + (float)i * 2.0f * (float)M_PI / (float)t->sides;
    *vx = t->cx + t->radius * cosf(a);
    *vy = t->cy + t->radius * sinf(a) * ASPECT;
}

/*
 * put_seg() — rasterise a line segment with DDA.
 *
 * Fills all cells from (x0,y0) to (x1,y1) using the character that
 * matches the segment angle.  Clips to (0…cols-1, 1…rows-2) to keep
 * drawing away from the HUD row and hint row.
 */
static void put_seg(WINDOW *w, float x0, float y0, float x1, float y1,
                    chtype attr, int cols, int rows)
{
    float dx    = x1 - x0;
    float dy    = y1 - y0;
    int   steps = (int)(fabsf(dx) > fabsf(dy) ? fabsf(dx) : fabsf(dy));
    if (steps < 1) steps = 1;
    char ch = angle_char(atan2f(dy, dx));

    for (int i = 0; i <= steps; i++) {
        float ft  = (float)i / (float)steps;
        int   col = (int)roundf(x0 + dx * ft);
        int   row = (int)roundf(y0 + dy * ft);
        if (col >= 0 && col < cols && row >= 1 && row < rows - 1) {
            wattron(w, attr);
            mvwaddch(w, row, col, (chtype)(unsigned char)ch);
            wattroff(w, attr);
        }
    }
}

/*
 * turtle_draw() — render the turtle's completed edges and current head.
 *
 * Draws edges 0 … edge-1 (each from vertex e to vertex e+1).
 * The turtle head '@' is placed at vertex `edge` — the pen's current
 * position — and is omitted once the polygon is complete.
 */
static void turtle_draw(const Turtle *t, WINDOW *w, int cols, int rows)
{
    chtype attr = (chtype)(COLOR_PAIR(t->cpair) | A_BOLD);

    /* completed edges */
    for (int e = 0; e < t->edge; e++) {
        float x0, y0, x1, y1;
        poly_vertex(t, e,     &x0, &y0);
        poly_vertex(t, e + 1, &x1, &y1);
        put_seg(w, x0, y0, x1, y1, attr, cols, rows);
    }

    /* turtle head — drawn after edges so it is always on top */
    if (!t->done) {
        float hx, hy;
        poly_vertex(t, t->edge, &hx, &hy);
        int ix = (int)roundf(hx);
        int iy = (int)roundf(hy);
        if (ix >= 0 && ix < cols && iy >= 1 && iy < rows - 1) {
            wattron(w, COLOR_PAIR(7) | A_BOLD);
            mvwaddch(w, iy, ix, '@');
            wattroff(w, COLOR_PAIR(7) | A_BOLD);
        }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene — two turtles, shared drawing speed, and reset countdown.
 *
 * When both turtles are done, reset_timer counts up.  At RESET_DELAY
 * seconds both are re-initialised with sides incremented by 1 each.
 * The cycle runs 3 → 4 → … → 12 → 3 indefinitely.
 */
typedef struct {
    Turtle tA;           /* left  turtle (cyan)    */
    Turtle tB;           /* right turtle (magenta) */
    float  eps;          /* edges per second        */
    float  reset_timer;  /* seconds since both done */
    bool   paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->eps    = EPS_DEFAULT;
    s->paused = false;
    turtle_init(&s->tA, cols, rows, 0, 3, 1, s->eps);
    turtle_init(&s->tB, cols, rows, 1, 5, 2, s->eps);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    if (s->paused) return;

    turtle_tick(&s->tA, dt);
    turtle_tick(&s->tB, dt);

    if (s->tA.done && s->tB.done) {
        s->reset_timer += dt;
        if (s->reset_timer >= RESET_DELAY) {
            int na = (s->tA.sides < SIDES_MAX) ? s->tA.sides + 1 : SIDES_MIN;
            int nb = (s->tB.sides < SIDES_MAX) ? s->tB.sides + 1 : SIDES_MIN;
            turtle_init(&s->tA, cols, rows, 0, na, 1, s->eps);
            turtle_init(&s->tB, cols, rows, 1, nb, 2, s->eps);
            s->reset_timer = 0.0f;
        }
    }
}

/*
 * Polygon name table (index = sides).
 */
static const char *const POLY_NAMES[] = {
    "", "", "",
    "Triangle", "Square",      "Pentagon",   "Hexagon",
    "Heptagon", "Octagon",     "Nonagon",    "Decagon",
    "Undecagon","Dodecagon"
};

static const char *poly_name(int sides)
{
    if (sides >= SIDES_MIN && sides <= SIDES_MAX) return POLY_NAMES[sides];
    return "Polygon";
}

static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows,
                       float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;

    /* vertical divider between the two halves */
    wattron(w, COLOR_PAIR(6) | A_DIM);
    for (int r = 1; r < rows - 1; r++)
        mvwaddch(w, r, cols / 2, '|');
    wattroff(w, COLOR_PAIR(6) | A_DIM);

    /* draw both turtles */
    turtle_draw(&s->tA, w, cols, rows);
    turtle_draw(&s->tB, w, cols, rows);

    /* polygon name labels (row 1, centred in each half) */
    char labA[32], labB[32];
    snprintf(labA, sizeof labA, "%s (%d)", poly_name(s->tA.sides), s->tA.sides);
    snprintf(labB, sizeof labB, "%s (%d)", poly_name(s->tB.sides), s->tB.sides);

    int cx_a = cols / 4;
    int cx_b = 3 * cols / 4;

    wattron(w, COLOR_PAIR(1) | A_BOLD);
    mvwprintw(w, 1, cx_a - (int)strlen(labA) / 2, "%s", labA);
    wattroff(w, COLOR_PAIR(1) | A_BOLD);

    wattron(w, COLOR_PAIR(2) | A_BOLD);
    mvwprintw(w, 1, cx_b - (int)strlen(labB) / 2, "%s", labB);
    wattroff(w, COLOR_PAIR(2) | A_BOLD);

    /* "DONE — next in Xs" banner when both polygons are complete */
    if (s->tA.done && s->tB.done) {
        char msg[48];
        int  sec_left = (int)(RESET_DELAY - s->reset_timer) + 1;
        snprintf(msg, sizeof msg, "DONE — next in %ds", sec_left);
        int mx = (cols - (int)strlen(msg)) / 2;
        if (mx < 0) mx = 0;
        wattron(w, COLOR_PAIR(3) | A_BOLD);
        mvwprintw(w, rows / 2, mx, "%s", msg);
        wattroff(w, COLOR_PAIR(3) | A_BOLD);
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
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

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    /* HUD — top-right */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %.1f fps  sim:%d Hz  %.2f eps  %s ",
             fps, sim_fps, sc->eps,
             sc->paused ? "PAUSED " : "drawing");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    /* hint — bottom-left */
    attron(COLOR_PAIR(6) | A_DIM);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset  a/z:A±  s/x:B±  +/-:speed  [/]:Hz ");
    attroff(COLOR_PAIR(6) | A_DIM);
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
    /* re-init turtles to recompute radius for new terminal size */
    int sa = app->scene.tA.sides;
    int sb = app->scene.tB.sides;
    turtle_init(&app->scene.tA, app->screen.cols, app->screen.rows, 0, sa, 1, app->scene.eps);
    turtle_init(&app->scene.tB, app->screen.cols, app->screen.rows, 1, sb, 2, app->scene.eps);
    app->scene.reset_timer = 0.0f;
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene  *sc = &app->scene;
    Screen *sr = &app->screen;

    int sa = sc->tA.sides;
    int sb = sc->tB.sides;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ':
        sc->paused = !sc->paused;
        break;

    case 'r': case 'R':
        turtle_init(&sc->tA, sr->cols, sr->rows, 0, sa, 1, sc->eps);
        turtle_init(&sc->tB, sr->cols, sr->rows, 1, sb, 2, sc->eps);
        sc->reset_timer = 0.0f;
        break;

    /* Turtle A sides */
    case 'a': case 'A':
        sa = (sa < SIDES_MAX) ? sa + 1 : SIDES_MIN;
        turtle_init(&sc->tA, sr->cols, sr->rows, 0, sa, 1, sc->eps);
        sc->reset_timer = 0.0f;
        break;
    case 'z': case 'Z':
        sa = (sa > SIDES_MIN) ? sa - 1 : SIDES_MAX;
        turtle_init(&sc->tA, sr->cols, sr->rows, 0, sa, 1, sc->eps);
        sc->reset_timer = 0.0f;
        break;

    /* Turtle B sides */
    case 's': case 'S':
        sb = (sb < SIDES_MAX) ? sb + 1 : SIDES_MIN;
        turtle_init(&sc->tB, sr->cols, sr->rows, 1, sb, 2, sc->eps);
        sc->reset_timer = 0.0f;
        break;
    case 'x': case 'X':
        sb = (sb > SIDES_MIN) ? sb - 1 : SIDES_MAX;
        turtle_init(&sc->tB, sr->cols, sr->rows, 1, sb, 2, sc->eps);
        sc->reset_timer = 0.0f;
        break;

    /* Drawing speed */
    case '=': case '+':
        sc->eps *= 1.5f;
        if (sc->eps > EPS_MAX) sc->eps = EPS_MAX;
        sc->tA.eps = sc->tB.eps = sc->eps;
        break;
    case '-':
        sc->eps /= 1.5f;
        if (sc->eps < EPS_MIN) sc->eps = EPS_MIN;
        sc->tA.eps = sc->tB.eps = sc->eps;
        break;

    /* Simulation Hz */
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
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
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

        /* ── sim accumulator (fixed timestep) ────────────────────── */
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

        /* ── FPS counter (500 ms window) ─────────────────────────── */
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
