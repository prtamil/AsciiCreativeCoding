/*
 * double_pendulum.c  —  chaotic double pendulum terminal demo
 *
 * Two rigid rods, two unit-mass bobs, no damping.  Derived from the
 * Lagrangian for equal masses m₁ = m₂ = 1 and equal arm lengths
 * L₁ = L₂ = L (pixel units).  Let δ = θ₁ − θ₂, D = 3 − cos 2δ.
 *
 *   Equations of motion (angles from downward vertical, y-axis down):
 *
 *     θ₁'' = [−3g sin θ₁ − g sin(θ₁−2θ₂)
 *              − 2 sin δ (ω₂²L + ω₁²L cos δ)] / (L · D)
 *
 *     θ₂'' = [2 sin δ (2ω₁²L + 2g cos θ₁ + ω₂²L cos δ)] / (L · D)
 *
 *   D ≥ 2 always (cos 2δ ≤ 1), so no singularities.
 *
 * Integration: 4th-order Runge-Kutta.
 *   RK4 is essential for chaotic systems — lower-order integrators
 *   (e.g. Euler) accumulate phase errors on the Lyapunov time-scale
 *   (~3–5 s) that are visually indistinguishable from real chaos.
 *
 * Chaos demo:
 *   A dim "ghost" pendulum starts with θ₁ + GHOST_EPSILON.  Both
 *   trajectories are identical at first; after ~3–5 s they diverge
 *   completely, demonstrating sensitive dependence on initial conditions.
 *   The HUD shows the angular separation growing exponentially.
 *
 * Trail:
 *   The end-bob traces a colour-faded ring-buffer trail.
 *   Recent positions are bright red/orange; older ones fade to dim grey.
 *   Reveals the complex attractor geometry as the system evolves.
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   r         reset (same initial conditions; ghost re-syncs)
 *   g         toggle ghost pendulum
 *   t         toggle trail
 *   + =       longer trail
 *   -         shorter trail
 *   ] [       faster / slower simulation
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra misc/double_pendulum.c \
 *       -o double_pendulum -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  coords
 *   §5  physics  — State, RK4 integrator, DPend struct
 *   §6  scene    — two pendulums, trail ring-buffer, draw
 *   §7  screen
 *   §8  app
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
    SIM_FPS_DEFAULT =  300,   /* RK4 needs small dt for chaotic accuracy   */
    SIM_FPS_MIN     =   60,
    SIM_FPS_MAX     =  600,
    SIM_FPS_STEP    =   60,

    TRAIL_LEN       =  500,   /* ring-buffer capacity (positions)          */
    TRAIL_DEF       =  360,   /* entries drawn by default                  */
    TRAIL_MIN       =   20,
    TRAIL_STEP      =   20,

    HUD_COLS        =   64,
    FPS_UPDATE_MS   =  500,
};

/*
 * CELL_W / CELL_H — sub-pixel resolution.  Physics in pixel space;
 * terminal cells only appear in the draw step.
 */
#define CELL_W   8
#define CELL_H  16

/*
 * ARM_LEN_FRAC — each arm is this fraction of the pixel-space height.
 * Pivot is at screen centre, so worst-case reach is 2×arm upward or
 * downward.  0.22 × ph keeps the full swing within ~44 % of half-height
 * on each side — visible on any typical terminal.
 */
#define ARM_LEN_FRAC    0.22f
#define GRAVITY_PX      2000.0f

/*
 * Starting angles (degrees from straight-down vertical).
 * 120° / 120° — both arms nearly horizontal with plenty of energy;
 * produces chaotic motion almost immediately.
 */
#define INIT_T1_DEG     120.0f
#define INIT_T2_DEG     120.0f

/*
 * GHOST_EPSILON — initial θ₁ offset for the ghost pendulum (radians).
 * ~0.057° perturbation; divergence becomes visible in ~3–5 s.
 */
#define GHOST_EPSILON   0.001f

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

enum {
    CP_BAR   = 1,   /* top bar                                            */
    CP_ARM1  = 2,   /* arm 1 (pivot → joint)  — cyan                     */
    CP_ARM2  = 3,   /* arm 2 (joint → bob)    — gold                     */
    CP_JOINT = 4,   /* joint bob              — medium grey               */
    CP_BOB   = 5,   /* end bob                — bright white              */
    CP_TR1   = 6,   /* trail tier 1 (newest)  — red                      */
    CP_TR2   = 7,   /* trail tier 2 (mid)     — orange                   */
    CP_TR3   = 8,   /* trail tier 3 (oldest)  — dark grey                */
    CP_GHOST = 9,   /* ghost pendulum         — dim purple                */
    CP_HUD   = 10,  /* HUD text               — cyan                     */
};

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(CP_BAR,   231, COLOR_BLACK);  /* bright white          */
        init_pair(CP_ARM1,   51, COLOR_BLACK);  /* bright cyan           */
        init_pair(CP_ARM2,  220, COLOR_BLACK);  /* golden yellow         */
        init_pair(CP_JOINT, 245, COLOR_BLACK);  /* medium grey           */
        init_pair(CP_BOB,   231, COLOR_BLACK);  /* bright white          */
        init_pair(CP_TR1,   196, COLOR_BLACK);  /* red (newest)          */
        init_pair(CP_TR2,   208, COLOR_BLACK);  /* orange                */
        init_pair(CP_TR3,   238, COLOR_BLACK);  /* dark grey (oldest)    */
        init_pair(CP_GHOST,  91, COLOR_BLACK);  /* dim purple            */
        init_pair(CP_HUD,    51, COLOR_BLACK);  /* bright cyan           */
    } else {
        init_pair(CP_BAR,   COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_ARM1,  COLOR_CYAN,    COLOR_BLACK);
        init_pair(CP_ARM2,  COLOR_YELLOW,  COLOR_BLACK);
        init_pair(CP_JOINT, COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_BOB,   COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_TR1,   COLOR_RED,     COLOR_BLACK);
        init_pair(CP_TR2,   COLOR_RED,     COLOR_BLACK);
        init_pair(CP_TR3,   COLOR_WHITE,   COLOR_BLACK);
        init_pair(CP_GHOST, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(CP_HUD,   COLOR_CYAN,    COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords                                                             */
/* ===================================================================== */

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
/* §5  physics                                                            */
/* ===================================================================== */

/*
 * State — the four degrees of freedom of the double pendulum.
 * All arithmetic on State goes through the helpers below so the RK4
 * step is written once and clearly.
 */
typedef struct { float t1, t2, w1, w2; } State;

/* s + dt * k  (used in RK4 intermediate stages) */
static inline State state_step(State s, float dt, State k)
{
    return (State){
        s.t1 + dt * k.t1,
        s.t2 + dt * k.t2,
        s.w1 + dt * k.w1,
        s.w2 + dt * k.w2,
    };
}

/*
 * state_deriv() — equations of motion.
 *
 * Returns ds/dt = (ω₁, ω₂, α₁, α₂) where α = θ''.
 *
 * Numerically, D = 3 − cos 2δ ≥ 2 since cos 2δ ≤ 1, so no division
 * by zero can occur regardless of the pendulum configuration.
 */
static State state_deriv(State s, float L, float g)
{
    float d  = s.t1 - s.t2;
    float sd = sinf(d),  cd = cosf(d);
    float D  = 3.0f - cosf(2.0f * d);   /* ≥ 2, never zero */
    float LD = L * D;

    float a1 = (-3.0f*g*sinf(s.t1) - g*sinf(s.t1 - 2.0f*s.t2)
                - 2.0f*sd*(s.w2*s.w2*L + s.w1*s.w1*L*cd))
               / LD;

    float a2 = 2.0f*sd*(2.0f*s.w1*s.w1*L + 2.0f*g*cosf(s.t1) + s.w2*s.w2*L*cd)
               / LD;

    return (State){ s.w1, s.w2, a1, a2 };
}

/*
 * rk4_step() — one classical 4th-order Runge-Kutta step.
 *
 * Error is O(dt⁵) per step, O(dt⁴) globally — a much tighter bound
 * than symplectic Euler's O(dt²), which matters here because chaotic
 * sensitivity amplifies phase errors exponentially.
 */
static State rk4_step(State s, float L, float g, float dt)
{
    State k1 = state_deriv(s,                         L, g);
    State k2 = state_deriv(state_step(s, 0.5f*dt, k1), L, g);
    State k3 = state_deriv(state_step(s, 0.5f*dt, k2), L, g);
    State k4 = state_deriv(state_step(s, dt,       k3), L, g);

    return (State){
        s.t1 + dt/6.0f*(k1.t1 + 2.0f*k2.t1 + 2.0f*k3.t1 + k4.t1),
        s.t2 + dt/6.0f*(k1.t2 + 2.0f*k2.t2 + 2.0f*k3.t2 + k4.t2),
        s.w1 + dt/6.0f*(k1.w1 + 2.0f*k2.w1 + 2.0f*k3.w1 + k4.w1),
        s.w2 + dt/6.0f*(k1.w2 + 2.0f*k2.w2 + 2.0f*k3.w2 + k4.w2),
    };
}

/*
 * DPend — complete state of one double pendulum.
 *
 * prev_t1/t2 hold the state at the start of the current tick;
 * scene_draw() lerps between (prev, current) using alpha for smooth
 * sub-tick rendering regardless of sim vs render rate.
 */
typedef struct {
    State s;                      /* current (t1, t2, w1, w2)             */
    float prev_t1, prev_t2;       /* state at tick start, for lerp         */
    float pivot_px, pivot_py;     /* fixed pivot position (pixel space)    */
    float arm_len;                /* L — both arms equal                   */
} DPend;

static void dpend_init(DPend *p, int cols, int rows,
                       float t1_deg, float t2_deg, float w1, float w2)
{
    p->pivot_px = (float)pw(cols) * 0.5f;
    p->pivot_py = (float)ph(rows) * 0.5f;    /* centre of screen         */
    p->arm_len  = (float)ph(rows) * ARM_LEN_FRAC;

    p->s = (State){
        (float)(t1_deg * M_PI / 180.0),
        (float)(t2_deg * M_PI / 180.0),
        w1, w2
    };
    p->prev_t1 = p->s.t1;
    p->prev_t2 = p->s.t2;
}

static void dpend_tick(DPend *p, float dt)
{
    p->prev_t1 = p->s.t1;
    p->prev_t2 = p->s.t2;
    p->s       = rk4_step(p->s, p->arm_len, GRAVITY_PX, dt);
}

/*
 * dpend_bobs() — compute joint and end-bob pixel positions for given
 * angles t1, t2 (interpolated or current).
 */
static void dpend_bobs(const DPend *p, float t1, float t2,
                       float *jx, float *jy,   /* joint */
                       float *bx, float *by)   /* end bob */
{
    *jx = p->pivot_px + p->arm_len * sinf(t1);
    *jy = p->pivot_py + p->arm_len * cosf(t1);
    *bx = *jx + p->arm_len * sinf(t2);
    *by = *jy + p->arm_len * cosf(t2);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Trail — ring buffer of the primary end-bob pixel positions.
 *
 * head points to the slot where the NEXT push will write.
 * To iterate oldest→newest: start at (head - count) mod TRAIL_LEN
 * and advance by 1, wrapping at TRAIL_LEN.
 */
typedef struct {
    float px[TRAIL_LEN];
    float py[TRAIL_LEN];
    int   head;           /* next write index */
    int   count;          /* valid entries (0..TRAIL_LEN) */
} Trail;

static void trail_push(Trail *t, float px, float py)
{
    t->px[t->head] = px;
    t->py[t->head] = py;
    t->head = (t->head + 1) % TRAIL_LEN;
    if (t->count < TRAIL_LEN) t->count++;
}

static void trail_clear(Trail *t) { t->head = 0; t->count = 0; }

typedef struct {
    DPend   primary;
    DPend   ghost;
    Trail   trail;
    int     cols, rows;
    bool    paused;
    bool    show_ghost;
    bool    show_trail;
    int     trail_draw;   /* how many entries to draw (≤ TRAIL_LEN) */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->cols       = cols;
    s->rows       = rows;
    s->paused     = false;
    s->show_ghost = true;
    s->show_trail = true;
    s->trail_draw = TRAIL_DEF;

    dpend_init(&s->primary, cols, rows,
               INIT_T1_DEG, INIT_T2_DEG, 0.0f, 0.0f);

    /* Ghost starts GHOST_EPSILON rad ahead on θ₁ */
    dpend_init(&s->ghost, cols, rows,
               INIT_T1_DEG + (float)(GHOST_EPSILON * 180.0 / M_PI),
               INIT_T2_DEG, 0.0f, 0.0f);

    trail_clear(&s->trail);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    dpend_tick(&s->primary, dt);
    dpend_tick(&s->ghost,   dt);

    /* Record current end-bob position for the trail */
    float jx, jy, bx, by;
    dpend_bobs(&s->primary, s->primary.s.t1, s->primary.s.t2,
               &jx, &jy, &bx, &by);
    trail_push(&s->trail, bx, by);
}

/* ── draw helpers ──────────────────────────────────────────────────── */

/*
 * draw_line() — Bresenham line; character chosen by step direction.
 * Identical to spring_pendulum.c.
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
            int  e2     = 2 * err;
            bool step_x = (e2 > -dy);
            bool step_y = (e2 <  dx);
            chtype ch;
            if      (step_x && step_y) ch = (sx == sy) ? '\\' : '/';
            else if (step_x)           ch = '-';
            else                       ch = '|';
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
 * scene_draw() — render one frame with render-interpolation alpha.
 *
 * Draw order (back → front so key elements overwrite background detail):
 *   1. Top bar + pivot marker
 *   2. Trail  (oldest → newest so brighter overwrites dimmer)
 *   3. Ghost arms and bob (if enabled)
 *   4. Primary arm 1 (pivot → joint)
 *   5. Primary arm 2 (joint → end bob)
 *   6. Joint bob
 *   7. End bob
 */
static void scene_draw(const Scene *s, float alpha)
{
    const DPend *p    = &s->primary;
    const int    cols = s->cols;
    const int    rows = s->rows;

    /* ── interpolated primary angles ─────────────────────────────── */
    float t1 = p->prev_t1 + (p->s.t1 - p->prev_t1) * alpha;
    float t2 = p->prev_t2 + (p->s.t2 - p->prev_t2) * alpha;

    float jx, jy, bx, by;
    dpend_bobs(p, t1, t2, &jx, &jy, &bx, &by);

    int piv_cx = px_to_cell_x(p->pivot_px);
    int piv_cy = px_to_cell_y(p->pivot_py);
    int j_cx   = px_to_cell_x(jx);
    int j_cy   = px_to_cell_y(jy);
    int b_cx   = px_to_cell_x(bx);
    int b_cy   = px_to_cell_y(by);

    /* ── 1. pivot marker (centre of screen) ───────────────────────── */
    if (piv_cx >= 1 && piv_cx < cols - 1 && piv_cy >= 0 && piv_cy < rows) {
        attron(COLOR_PAIR(CP_BAR) | A_BOLD);
        mvaddch(piv_cy, piv_cx - 1, '[');
        mvaddch(piv_cy, piv_cx,     '+');
        mvaddch(piv_cy, piv_cx + 1, ']');
        attroff(COLOR_PAIR(CP_BAR) | A_BOLD);
    }

    /* ── 2. trail ──────────────────────────────────────────────────── */
    if (s->show_trail && s->trail.count > 0) {
        int draw = s->trail_draw;
        if (draw > s->trail.count) draw = s->trail.count;

        /*
         * Iterate oldest → newest within the draw window.
         * Oldest = head - draw (wrapping); newest = head - 1 (wrapping).
         * Three colour tiers by age fraction.
         */
        int start = (s->trail.head - draw + TRAIL_LEN) % TRAIL_LEN;
        for (int i = 0; i < draw; i++) {
            int idx = (start + i) % TRAIL_LEN;
            int cx  = px_to_cell_x(s->trail.px[idx]);
            int cy  = px_to_cell_y(s->trail.py[idx]);
            if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

            float age = (float)i / (float)draw;   /* 0=oldest, 1=newest */
            int   cp;
            chtype ch;
            if      (age > 0.70f) { cp = CP_TR1; ch = 'o'; }
            else if (age > 0.35f) { cp = CP_TR2; ch = '.'; }
            else                  { cp = CP_TR3; ch = ','; }

            attron(COLOR_PAIR(cp));
            mvaddch(cy, cx, ch);
            attroff(COLOR_PAIR(cp));
        }
    }

    /* ── 3. ghost pendulum ─────────────────────────────────────────── */
    if (s->show_ghost) {
        const DPend *g = &s->ghost;
        float gt1 = g->prev_t1 + (g->s.t1 - g->prev_t1) * alpha;
        float gt2 = g->prev_t2 + (g->s.t2 - g->prev_t2) * alpha;

        float gjx, gjy, gbx, gby;
        dpend_bobs(g, gt1, gt2, &gjx, &gjy, &gbx, &gby);

        int gjcx = px_to_cell_x(gjx), gjcy = px_to_cell_y(gjy);
        int gbcx = px_to_cell_x(gbx), gbcy = px_to_cell_y(gby);

        attr_t ghost_attr = COLOR_PAIR(CP_GHOST);
        draw_line(piv_cx, piv_cy, gjcx, gjcy, cols, rows, ghost_attr);
        draw_line(gjcx,   gjcy,   gbcx, gbcy, cols, rows, ghost_attr);

        /* Ghost end bob — simple 'x' marker */
        if (gbcx >= 1 && gbcx < cols-1 && gbcy > 0 && gbcy < rows) {
            attron(ghost_attr);
            mvaddch(gbcy, gbcx, 'x');
            attroff(ghost_attr);
        }
    }

    /* ── 4. primary arm 1 (pivot → joint) ─────────────────────────── */
    draw_line(piv_cx, piv_cy, j_cx, j_cy,
              cols, rows, COLOR_PAIR(CP_ARM1) | A_BOLD);

    /* ── 5. primary arm 2 (joint → end bob) ───────────────────────── */
    draw_line(j_cx, j_cy, b_cx, b_cy,
              cols, rows, COLOR_PAIR(CP_ARM2) | A_BOLD);

    /* ── 6. joint bob ──────────────────────────────────────────────── */
    if (j_cx >= 0 && j_cx < cols && j_cy > 0 && j_cy < rows) {
        attron(COLOR_PAIR(CP_JOINT) | A_BOLD);
        mvaddch(j_cy, j_cx, 'O');
        attroff(COLOR_PAIR(CP_JOINT) | A_BOLD);
    }

    /* ── 7. end bob ────────────────────────────────────────────────── */
    if (b_cy > 0 && b_cy < rows) {
        attron(COLOR_PAIR(CP_BOB) | A_BOLD);
        if (b_cx > 0 && b_cx < cols - 1) {
            mvaddch(b_cy, b_cx - 1, '(');
            mvaddch(b_cy, b_cx,     '@');
            mvaddch(b_cy, b_cx + 1, ')');
        } else {
            mvaddch(b_cy, b_cx, '@');
        }
        attroff(COLOR_PAIR(CP_BOB) | A_BOLD);
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
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

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(Screen *s, const Scene *sc, double fps, float alpha)
{
    erase();
    scene_draw(sc, alpha);

    const DPend *p   = &sc->primary;
    const DPend *g   = &sc->ghost;

    float deg1 = p->s.t1 * (float)(180.0 / M_PI);
    float deg2 = p->s.t2 * (float)(180.0 / M_PI);

    /* Angular divergence from ghost — grows exponentially until saturation */
    float div_deg = fabsf(p->s.t1 - g->s.t1) * (float)(180.0 / M_PI)
                  + fabsf(p->s.t2 - g->s.t2) * (float)(180.0 / M_PI);
    if (div_deg > 9999.0f) div_deg = 9999.0f;

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps  t1:%+7.1f  t2:%+7.1f  div:%6.1f  tr:%d",
             fps, deg1, deg2, div_deg, sc->trail_draw);

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

static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
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
    Scene *sc = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        sc->paused = !sc->paused;
        break;

    case 'r': case 'R':
        scene_init(sc, app->screen.cols, app->screen.rows);
        break;

    case 'g': case 'G':
        sc->show_ghost = !sc->show_ghost;
        break;

    case 't': case 'T':
        sc->show_trail = !sc->show_trail;
        break;

    case '+': case '=':
        sc->trail_draw += TRAIL_STEP;
        if (sc->trail_draw > TRAIL_LEN) sc->trail_draw = TRAIL_LEN;
        break;

    case '-':
        sc->trail_draw -= TRAIL_STEP;
        if (sc->trail_draw < TRAIL_MIN) sc->trail_draw = TRAIL_MIN;
        break;

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
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app      = &g_app;
    app->running  = 1;
    app->sim_fps  = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── resize ────────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── dt ─────────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── sim accumulator ──────────────────────────────────────────
         * Drain in fixed steps so the physics integration step dt_sec
         * is constant regardless of render frame rate.  RK4 accuracy
         * degrades with larger dt, so we always use 1/sim_fps seconds.
         */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── FPS counter ──────────────────────────────────────────────*/
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap (sleep BEFORE render — stable regardless of I/O) */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── draw + present ───────────────────────────────────────────*/
        screen_draw(&app->screen, &app->scene, fps_display, alpha);
        screen_present();

        /* ── input ─────────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
