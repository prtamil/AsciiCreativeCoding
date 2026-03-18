/*
 * spring_pendulum.c  —  ncurses ASCII spring pendulum
 *
 * A mass hangs from a fixed pivot at the top centre via a spring that
 * can both rotate (pendulum) and stretch/compress (spring motion).
 * When ω_spring ≈ 2 × ω_pendulum the two modes exchange energy and
 * the bob traces a complex rosette path before slowly settling.
 *
 * Physics: polar-coordinate spring pendulum (Lagrangian mechanics).
 *
 *   r   — current spring length  (pixels)
 *   θ   — angle from downward vertical  (rad, + = right)
 *
 *   Equations of motion (y-axis positive downward):
 *     r̈  =  r·θ̇²  +  g·cos θ  −  (k/m)·(r − r₀)  −  d·ṙ
 *     θ̈  =  −[g·sin θ + 2·ṙ·θ̇] / r  −  d·θ̇
 *
 *   Integration: semi-implicit (symplectic) Euler.
 *     Velocities are updated first, then positions.
 *     Conserves the symplectic structure — no long-term energy drift.
 *
 * Frequency design (so energy exchange is visible):
 *   ω_pend  = √(g / r₀)    (pendulum angular frequency)
 *   ω_spring = √(k / m)    (spring angular frequency, m = 1)
 *   Tuned so ω_spring ≈ 2 × ω_pend — classic 2:1 resonance.
 *
 * Coordinate spaces (same convention as bounce_ball.c):
 *   Pixel space  — square, CELL_W × CELL_H sub-pixels per cell.
 *                  All physics lives here.
 *   Cell space   — terminal columns and rows.
 *                  Only drawing converts to cell coords.
 *
 * Render interpolation:
 *   (prev_r, prev_θ) saved each tick.  Draw state lerps between prev
 *   and current using alpha = sim_accum / tick_ns — smooth motion
 *   regardless of sim vs render timing.
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   r         reset to initial conditions
 *   ]  [      decrease / increase damping
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra spring_pendulum.c -o spring_pendulum -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config    — every tunable constant
 *   §2  clock     — monotonic ns clock + sleep
 *   §3  color     — color pairs
 *   §4  coords    — pixel↔cell conversion; aspect-ratio fix
 *   §5  pendulum  — physics state, symplectic Euler tick
 *   §6  scene     — owns pendulum; tick; draw bar + spring + bob
 *   §7  screen    — single stdscr, ncurses internal double buffer
 *   §8  app       — dt loop, input, resize, cleanup
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
    SIM_FPS_DEFAULT = 120,   /* high rate — spring ODEs need small steps  */
    SIM_FPS_MIN     =  30,
    SIM_FPS_MAX     = 120,

    HUD_COLS        =  44,
    FPS_UPDATE_MS   = 500,

    N_COILS         =   8,   /* spring coils drawn between pivot and bob  */
};

/*
 * CELL_W / CELL_H — sub-pixel resolution per terminal cell.
 * Must match the terminal cell aspect ratio (~2 tall : 1 wide).
 * Physics uses pixel units; cell coords only appear in the draw step.
 */
#define CELL_W   8
#define CELL_H  16

/*
 * COIL_SPREAD — perpendicular half-width of the spring zigzag, pixels.
 * Two cells wide on each side of the spring axis.
 */
#define COIL_SPREAD  (CELL_W * 2)

/*
 * Physics constants — all lengths in pixels, time in seconds, mass = 1.
 *
 * GRAVITY_PX     gravitational acceleration (px/s²)
 * SPRING_K       spring constant (1/s² for unit mass)
 * DAMPING_DEF    default linear damping applied to both ṙ and θ̇
 * DAMPING_STEP   ] / [ adjustment per keypress
 * REST_LEN_FRAC  natural spring length as fraction of pixel-space height
 *
 * 2:1 frequency ratio sanity check (50-row terminal, CELL_H=16):
 *   r₀ = 0.40 × 50 × 16 = 320 px
 *   ω_pend  = √(2000 / 320) ≈ 2.50 rad/s
 *   ω_spring = √25.0        = 5.00 rad/s   → ratio = 2.0  ✓
 */
#define GRAVITY_PX      2000.0f
#define SPRING_K          25.0f
#define DAMPING_DEF        0.12f
#define DAMPING_STEP       0.02f
#define DAMPING_MIN        0.00f
#define DAMPING_MAX        0.80f
#define REST_LEN_FRAC      0.40f

/* Initial conditions */
#define INIT_THETA_DEG    40.0f   /* starting angle from vertical (deg)   */
#define INIT_R_STRETCH     1.15f  /* r_start = r₀ × this factor           */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
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

enum {
    CP_BAR    = 1,   /* top pivot bar — bright white                     */
    CP_WIRE   = 2,   /* straight wire stubs above/below spring coils     */
    CP_SPRING = 3,   /* spring coil nodes and connecting lines — yellow  */
    CP_BALL   = 4,   /* iron bob — bold white / grey                     */
    CP_HUD    = 5,   /* HUD text                                         */
};

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(CP_BAR,    231, COLOR_BLACK);   /* bright white        */
        init_pair(CP_WIRE,   245, COLOR_BLACK);   /* medium grey         */
        init_pair(CP_SPRING, 220, COLOR_BLACK);   /* golden yellow       */
        init_pair(CP_BALL,   252, COLOR_BLACK);   /* light grey          */
        init_pair(CP_HUD,     51, COLOR_BLACK);   /* bright cyan         */
    } else {
        init_pair(CP_BAR,    COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_WIRE,   COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_SPRING, COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_BALL,   COLOR_WHITE,  COLOR_BLACK);
        init_pair(CP_HUD,    COLOR_CYAN,   COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords — the one place aspect ratio is handled                     */
/* ===================================================================== */

/*
 * Same pixel↔cell convention as bounce_ball.c.
 * Physics operates entirely in pixel space.
 * px_to_cell_x/y are the only calls that cross into cell space.
 */
static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  pendulum                                                           */
/* ===================================================================== */

/*
 * Pendulum — full spring-pendulum state in polar coordinates.
 *
 * Coordinate origin is at the pivot.  Ball pixel position:
 *   ball_px = pivot_px + r · sin(θ)
 *   ball_py = pivot_py + r · cos(θ)     (y increases downward)
 *
 * prev_r / prev_theta hold the state at the end of the previous tick
 * and are used by scene_draw() for render interpolation.
 */
typedef struct {
    float r,        theta;      /* current polar state                   */
    float r_dot,    th_dot;     /* radial and angular velocities         */
    float prev_r,   prev_theta; /* previous-tick state for lerp          */
    float pivot_px, pivot_py;  /* fixed pivot in pixel space            */
    float r0;                   /* natural rest length (px)              */
    float damping;              /* linear damping coefficient            */
} Pendulum;

static void pendulum_init(Pendulum *p, int cols, int rows)
{
    p->pivot_px = (float)pw(cols) * 0.5f;
    p->pivot_py = (float)CELL_H;               /* one cell below top row */

    p->r0      = (float)ph(rows) * REST_LEN_FRAC;
    p->r       = p->r0 * INIT_R_STRETCH;
    p->theta   = (float)(INIT_THETA_DEG * M_PI / 180.0);
    p->r_dot   = 0.0f;
    p->th_dot  = 0.0f;
    p->damping = DAMPING_DEF;

    p->prev_r     = p->r;
    p->prev_theta = p->theta;
}

/*
 * pendulum_tick() — one simulation step, semi-implicit (symplectic) Euler.
 *
 * Equations of motion derived from the Lagrangian of a 2-D spring
 * pendulum with linear damping (y-axis positive downward):
 *
 *   r̈  =  r·θ̇²  +  g·cos θ  −  k·(r − r₀)  −  d·ṙ
 *   θ̈  =  −[g·sin θ  +  2·ṙ·θ̇] / r  −  d·θ̇
 *
 * Symplectic Euler update order (prevents energy drift):
 *   1. Compute r̈, θ̈ from current state.
 *   2. ṙ  += r̈·dt,   θ̇  += θ̈·dt    (velocities updated first)
 *   3. r  += ṙ·dt,    θ  += θ̇·dt    (positions use new velocities)
 *
 * r is clamped to [r₀·0.05, r₀·3.5] to prevent numerical collapse.
 */
static void pendulum_tick(Pendulum *p, float dt)
{
    p->prev_r     = p->r;
    p->prev_theta = p->theta;

    float r   = p->r,   th  = p->theta;
    float rd  = p->r_dot, thd = p->th_dot;
    float d   = p->damping;

    float r_ddot  = r * thd * thd
                  + GRAVITY_PX * cosf(th)
                  - SPRING_K   * (r - p->r0)
                  - d * rd;

    float th_ddot = -(GRAVITY_PX * sinf(th) + 2.0f * rd * thd) / r
                  - d * thd;

    /* velocities first (symplectic) */
    p->r_dot  = rd  + r_ddot  * dt;
    p->th_dot = thd + th_ddot * dt;

    /* then positions */
    p->r     = r  + p->r_dot  * dt;
    p->theta = th + p->th_dot * dt;

    if (p->r < p->r0 * 0.05f) p->r = p->r0 * 0.05f;
    if (p->r > p->r0 * 3.5f)  p->r = p->r0 * 3.5f;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    Pendulum pend;
    int      cols, rows;
    bool     paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->cols   = cols;
    s->rows   = rows;
    s->paused = false;
    pendulum_init(&s->pend, cols, rows);
}

static void scene_tick(Scene *s, float dt)
{
    if (!s->paused)
        pendulum_tick(&s->pend, dt);
}

/* ── drawing helpers ─────────────────────────────────────────────────── */

/*
 * draw_line() — Bresenham line between two cell positions.
 *
 * Character per step is chosen by the local step direction so the line
 * looks connected at any angle:
 *   diagonal step (both x and y advance) → '\' or '/'
 *   horizontal step only                 → '-'
 *   vertical step only                   → '|'
 */
static void draw_line(int x0, int y0, int x1, int y1,
                      int cols, int rows, attr_t attr)
{
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    for (;;) {
        if (x0 >= 0 && x0 < cols && y0 >= 0 && y0 < rows) {
            int   e2     = 2 * err;
            bool  step_x = (e2 > -dy);
            bool  step_y = (e2 <  dx);
            chtype ch;
            if (step_x && step_y)        ch = (sx == sy) ? '\\' : '/';
            else if (step_x)             ch = '-';
            else                         ch = '|';
            attron(attr);
            mvaddch(y0, x0, ch);
            attroff(attr);
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/*
 * scene_draw() — render top bar, spring, and bob for one frame.
 *
 * alpha ∈ [0,1): render interpolation factor.
 *   draw_r     = lerp(prev_r,     r,     alpha)
 *   draw_theta = lerp(prev_theta, theta, alpha)
 *
 * Draw order (back → front so nodes overwrite wire lines):
 *   1. Top bar (row 0, full width) with pivot marker
 *   2. Wire stub: pivot cell → first coil node
 *   3. Spring coil connecting lines between adjacent nodes
 *   4. Wire stub: last coil node → bob cell
 *   5. Spring coil nodes (overwrite connecting lines with '*')
 *   6. Iron bob '(@)' (overwrite any wire at bob position)
 *
 * Spring coil layout:
 *   N_COILS * 2 nodes, evenly spaced along the spring axis.
 *   Odd-indexed nodes offset +COIL_SPREAD px perpendicularly (right).
 *   Even-indexed nodes offset −COIL_SPREAD px (left).
 *   This produces the classic spring zigzag at any pendulum angle.
 */
static void scene_draw(const Scene *s, float alpha)
{
    const Pendulum *p    = &s->pend;
    const int       cols = s->cols;
    const int       rows = s->rows;

    /* ── interpolated draw state ──────────────────────────────────── */
    float draw_r     = p->prev_r     + (p->r     - p->prev_r)     * alpha;
    float draw_theta = p->prev_theta + (p->theta  - p->prev_theta) * alpha;

    /* ── pivot cell ───────────────────────────────────────────────── */
    int pivot_cx = px_to_cell_x(p->pivot_px);
    int pivot_cy = px_to_cell_y(p->pivot_py);
    if (pivot_cy < 1) pivot_cy = 1;

    /* ── bob pixel / cell position ────────────────────────────────── */
    float bob_px = p->pivot_px + draw_r * sinf(draw_theta);
    float bob_py = p->pivot_py + draw_r * cosf(draw_theta);
    int   bob_cx = px_to_cell_x(bob_px);
    int   bob_cy = px_to_cell_y(bob_py);
    if (bob_cx < 1)       bob_cx = 1;
    if (bob_cx > cols-2)  bob_cx = cols - 2;
    if (bob_cy < 1)       bob_cy = 1;
    if (bob_cy > rows-2)  bob_cy = rows - 2;

    /* ── spring axis and perpendicular unit vectors ───────────────── */
    float ax    =  sinf(draw_theta);   /* spring axis (toward bob) */
    float ay    =  cosf(draw_theta);
    float perpx = -ay;                 /* perpendicular (90° CCW)  */
    float perpy =  ax;

    /* ── compute all N_NODES coil node positions ──────────────────── */
    int   N_NODES = N_COILS * 2;
    float node_px[N_COILS * 2];
    float node_py[N_COILS * 2];

    for (int i = 0; i < N_NODES; i++) {
        float t    = (float)(i + 1) / (float)(N_NODES + 1);
        float bx   = p->pivot_px + t * draw_r * ax;
        float by   = p->pivot_py + t * draw_r * ay;
        float sign = (i % 2 == 0) ? 1.0f : -1.0f;
        node_px[i] = bx + sign * COIL_SPREAD * perpx;
        node_py[i] = by + sign * COIL_SPREAD * perpy;
    }

    /* ── 1. top bar ───────────────────────────────────────────────── */
    attron(COLOR_PAIR(CP_BAR) | A_BOLD);
    for (int x = 0; x < cols; x++)
        mvaddch(0, x, '=');
    /* downward-pointing pivot marker */
    if (pivot_cx >= 0 && pivot_cx < cols)
        mvaddch(0, pivot_cx, 'v');
    attroff(COLOR_PAIR(CP_BAR) | A_BOLD);

    /* ── 2. wire stub: pivot → first coil node ────────────────────── */
    {
        int nx = px_to_cell_x(node_px[0]);
        int ny = px_to_cell_y(node_py[0]);
        draw_line(pivot_cx, pivot_cy, nx, ny,
                  cols, rows, COLOR_PAIR(CP_WIRE));
    }

    /* ── 3. connecting lines between adjacent coil nodes ─────────── */
    for (int i = 0; i < N_NODES - 1; i++) {
        int x0 = px_to_cell_x(node_px[i]);
        int y0 = px_to_cell_y(node_py[i]);
        int x1 = px_to_cell_x(node_px[i + 1]);
        int y1 = px_to_cell_y(node_py[i + 1]);
        draw_line(x0, y0, x1, y1,
                  cols, rows, COLOR_PAIR(CP_SPRING));
    }

    /* ── 4. wire stub: last coil node → bob ──────────────────────── */
    {
        int nx = px_to_cell_x(node_px[N_NODES - 1]);
        int ny = px_to_cell_y(node_py[N_NODES - 1]);
        draw_line(nx, ny, bob_cx, bob_cy,
                  cols, rows, COLOR_PAIR(CP_WIRE));
    }

    /* ── 5. spring coil nodes (overwrite connecting-line chars) ───── */
    attron(COLOR_PAIR(CP_SPRING) | A_BOLD);
    for (int i = 0; i < N_NODES; i++) {
        int cx = px_to_cell_x(node_px[i]);
        int cy = px_to_cell_y(node_py[i]);
        if (cx >= 0 && cx < cols && cy > 0 && cy < rows)
            mvaddch(cy, cx, '*');
    }
    attroff(COLOR_PAIR(CP_SPRING) | A_BOLD);

    /* ── 6. iron bob ─────────────────────────────────────────────── */
    attron(COLOR_PAIR(CP_BALL) | A_BOLD);
    if (bob_cx > 0 && bob_cx < cols - 1) {
        mvaddch(bob_cy, bob_cx - 1, '(');
        mvaddch(bob_cy, bob_cx,     '@');
        mvaddch(bob_cy, bob_cx + 1, ')');
    } else {
        mvaddch(bob_cy, bob_cx, '@');
    }
    attroff(COLOR_PAIR(CP_BALL) | A_BOLD);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — single stdscr, ncurses internal double buffer.
 *
 * erase()           — clear newscr (back buffer), no terminal I/O
 * scene_draw()      — write spring + bob into newscr
 * mvprintw / attron — write HUD into newscr after scene (always on top)
 * wnoutrefresh()    — mark newscr ready, still no terminal I/O
 * doupdate()        — ONE atomic write: diff newscr vs curscr → terminal
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

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw() — build the complete frame in stdscr (newscr).
 *
 * erase() first so stale pixels from previous frame are cleared.
 * scene_draw() writes spring at interpolated position.
 * HUD is written last (top-right, on top of the bar) — always visible.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, float alpha)
{
    erase();
    scene_draw(sc, alpha);

    const Pendulum *p       = &sc->pend;
    float           deg     = p->theta * (float)(180.0 / M_PI);
    float           stretch = (p->r - p->r0) / p->r0 * 100.0f;

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps  θ:%+6.1f°  Δr:%+5.1f%%  d:%.2f",
             fps, deg, stretch, p->damping);

    int hx = s->cols - HUD_COLS;
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
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
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Pendulum *p = &app->scene.pend;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        app->scene.paused = !app->scene.paused;
        break;

    case 'r': case 'R':
        pendulum_init(p, app->screen.cols, app->screen.rows);
        break;

    case ']':
        p->damping -= DAMPING_STEP;
        if (p->damping < DAMPING_MIN) p->damping = DAMPING_MIN;
        break;

    case '[':
        p->damping += DAMPING_STEP;
        if (p->damping > DAMPING_MAX) p->damping = DAMPING_MAX;
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

        /* ── FPS counter ─────────────────────────────────────────── */
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
        screen_draw(&app->screen, &app->scene, fps_display, alpha);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
