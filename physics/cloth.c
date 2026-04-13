/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cloth.c — Spring-Mass Cloth Simulation
 *
 * Grid of point masses connected by structural, shear, and bend springs.
 * Integrated with symplectic Euler; spring forces computed explicitly
 * (Hooke's law + relative-velocity damping).  Top row / column pinned;
 * gravity + sinusoidal wind.
 *
 * Framework: follows framework.c §1–§8 skeleton.
 *
 * PHYSICS SUMMARY
 * ─────────────────────────────────────────────────────────────────────
 * Explicit spring force between nodes a, b:
 *   d = b.pos − a.pos
 *   dist = |d|
 *   stretch = dist − rest_len
 *   v_rel = dot(b.vel − a.vel, d̂)          (relative velocity along spring)
 *   F_mag = k * stretch + kd * v_rel
 *   F = F_mag * d̂                            (force on a; −F on b)
 *
 * Symplectic Euler integration (per node):
 *   vel += accel * dt
 *   vel *= DAMP                              (global velocity damping)
 *   pos += vel * dt
 *
 * Spring types:
 *   structural — adjacent nodes (H and V)
 *   shear      — diagonal neighbours
 *   bend       — next-nearest (skip 1 node)
 *
 * Physics in pixel space; only scene_draw calls px_to_cell.
 *
 * Cell spacing:
 *   REST_H = CELL_W × NODE_GAP   (horizontal spacing in pixels)
 *   REST_V = CELL_H × NODE_GAP   (vertical spacing in pixels)
 *
 * Three presets:
 *   0  Hanging cloth — top row pinned, gravity, gentle wind
 *   1  Flag         — left column pinned, strong horizontal wind
 *   2  Hammock      — top corners only pinned, heavy gravity + gusts
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   r         restart current preset
 *   n / p     next / previous preset
 *   w         toggle wind
 *   ] / [     sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra cloth.c -o cloth -lncurses -lm
 */

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
    SIM_FPS_MIN       = 10,
    SIM_FPS_DEFAULT   = 60,
    SIM_FPS_MAX       = 120,
    SIM_FPS_STEP      = 10,
    FPS_UPDATE_MS     = 500,
    N_COLORS          = 7,

    CLOTH_W           = 30,   /* nodes horizontally                        */
    CLOTH_H           = 18,   /* nodes vertically                          */
    NODE_GAP          = 2,    /* terminal cells between adjacent nodes     */
    SUB_STEPS         = 8,    /* physics sub-steps per sim tick            */
    N_PRESETS         = 3,
};

#define CLOTH_N  (CLOTH_W * CLOTH_H)

/* Spring counts (upper bounds) */
#define MAX_SPRINGS  (CLOTH_W * CLOTH_H * 6)

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC / (f))

/* Pixel cell dimensions */
#define CELL_W   8
#define CELL_H   16

/* Node pixel spacing */
#define REST_H   (CELL_W * NODE_GAP)    /* horizontal rest length (px) */
#define REST_V   (CELL_H * NODE_GAP)    /* vertical rest length (px)   */

/* Physics constants */
#define GRAVITY      200.0f    /* downward acceleration (px/s²)            */
#define WIND_FREQ    0.40f     /* wind oscillation frequency (Hz)          */

/*
 * DAMP — velocity retention per sub-step.
 * SUB_STEPS=8 at 60 Hz → 480 sub-steps/s.
 * 0.9993^480 ≈ 0.718 → cloth settles in a few seconds.
 */
#define DAMP         0.9993f   /* velocity retention per sub-step          */

/*
 * Spring stiffness and damping per type.
 * k  (px/s² per px) — restoring force per unit extension
 * kd (s⁻¹)          — damping proportional to relative velocity along spring
 *
 * Stability check (symplectic Euler):  k * dt² < 2
 * dt = 1/(60*8) ≈ 0.00208 s → k < 2/dt² ≈ 462 000.  All values well inside.
 */
#define K_STRUCT   400.0f   /* structural spring stiffness                 */
#define K_SHEAR    100.0f   /* shear spring stiffness                      */
#define K_BEND      40.0f   /* bend spring stiffness                       */
#define KD_STRUCT    4.0f   /* structural spring damping                   */
#define KD_SHEAR     2.0f   /* shear spring damping                        */
#define KD_BEND      0.5f   /* bend spring damping                         */

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
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(1, 196, COLOR_BLACK);
        init_pair(2, 208, COLOR_BLACK);
        init_pair(3, 226, COLOR_BLACK);
        init_pair(4,  46, COLOR_BLACK);
        init_pair(5,  51, COLOR_BLACK);
        init_pair(6,  75, COLOR_BLACK);
        init_pair(7, 201, COLOR_BLACK);
    } else {
        init_pair(1, COLOR_RED,     COLOR_BLACK);
        init_pair(2, COLOR_RED,     COLOR_BLACK);
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(6, COLOR_BLUE,    COLOR_BLACK);
        init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords — pixel ↔ cell                                              */
/* ===================================================================== */

static inline int px_to_cell_x(float px) {
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py) {
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  entity — Node, Spring, Cloth                                       */
/* ===================================================================== */

/* ── Node ──────────────────────────────────────────────────────────── */

typedef struct {
    float x,  y;    /* current position (pixels)                        */
    float vx, vy;   /* velocity (pixels/s)                              */
    float rx, ry;   /* render snapshot: position at start of last tick  */
    bool  pinned;
} Node;

/* ── Spring ────────────────────────────────────────────────────────── */

typedef struct {
    int   a, b;      /* node indices */
    float rest;      /* rest length (pixels) */
    float k;         /* stiffness (px/s² per px) */
    float kd;        /* spring damping (s⁻¹) */
} Spring;

/* ── Cloth ─────────────────────────────────────────────────────────── */

typedef struct {
    Node   nodes[CLOTH_N];
    Spring springs[MAX_SPRINGS];
    int    n_springs;
    float  wind_phase;
    float  wind_strength;
    bool   wind_on;
    bool   paused;
    int    preset;
} Cloth;

/* ── Helpers ───────────────────────────────────────────────────────── */

static inline int node_idx(int col, int row) { return row * CLOTH_W + col; }

static void cloth_add_spring(Cloth *c, int a, int b,
                             float rest, float k, float kd)
{
    if (c->n_springs >= MAX_SPRINGS) return;
    Spring *sp = &c->springs[c->n_springs++];
    sp->a    = a;
    sp->b    = b;
    sp->rest = rest;
    sp->k    = k;
    sp->kd   = kd;
}

/* ── Preset initialisation ─────────────────────────────────────────── */

/*
 * cloth_reset_positions — place nodes in their rest grid.
 *
 * The top-left node starts at pixel (ox0, oy0).
 * REST_H pixels between horizontal neighbours.
 * REST_V pixels between vertical neighbours.
 */
static void cloth_reset_positions(Cloth *c, float ox0, float oy0)
{
    for (int row = 0; row < CLOTH_H; row++) {
        for (int col = 0; col < CLOTH_W; col++) {
            Node *n = &c->nodes[node_idx(col, row)];
            n->x  = n->rx = ox0 + (float)col * REST_H;
            n->y  = n->ry = oy0 + (float)row * REST_V;
            n->vx = n->vy = 0.0f;
            n->pinned = false;
        }
    }
}

static void cloth_build_springs(Cloth *c)
{
    c->n_springs = 0;
    float diag  = sqrtf((float)(REST_H*REST_H + REST_V*REST_V));

    for (int row = 0; row < CLOTH_H; row++) {
        for (int col = 0; col < CLOTH_W; col++) {
            int idx = node_idx(col, row);

            /* structural horizontal */
            if (col + 1 < CLOTH_W)
                cloth_add_spring(c, idx, node_idx(col+1, row),
                                 (float)REST_H, K_STRUCT, KD_STRUCT);

            /* structural vertical */
            if (row + 1 < CLOTH_H)
                cloth_add_spring(c, idx, node_idx(col, row+1),
                                 (float)REST_V, K_STRUCT, KD_STRUCT);

            /* shear */
            if (col + 1 < CLOTH_W && row + 1 < CLOTH_H) {
                cloth_add_spring(c, idx, node_idx(col+1, row+1),
                                 diag, K_SHEAR, KD_SHEAR);
                cloth_add_spring(c, node_idx(col+1, row), node_idx(col, row+1),
                                 diag, K_SHEAR, KD_SHEAR);
            }

            /* bend horizontal */
            if (col + 2 < CLOTH_W)
                cloth_add_spring(c, idx, node_idx(col+2, row),
                                 (float)(REST_H * 2), K_BEND, KD_BEND);

            /* bend vertical */
            if (row + 2 < CLOTH_H)
                cloth_add_spring(c, idx, node_idx(col, row+2),
                                 (float)(REST_V * 2), K_BEND, KD_BEND);
        }
    }
}

/*
 * preset 0 — Hanging cloth
 * Top row pinned; gravity pulls down; gentle oscillating wind.
 */
static void preset_hanging(Cloth *c, int cols, int rows)
{
    int cloth_px_w = (CLOTH_W - 1) * REST_H;
    float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
    float oy0 = (float)(rows * CELL_H) * 0.05f + (float)CELL_H * 2;

    cloth_reset_positions(c, ox0, oy0);
    /* pin top row */
    for (int col = 0; col < CLOTH_W; col++)
        c->nodes[node_idx(col, 0)].pinned = true;

    /* Start at quarter-period so wind is at full strength immediately. */
    c->wind_phase    = (float)M_PI * 0.5f;
    c->wind_strength = 30.0f;
    c->wind_on       = true;
}

/*
 * preset 1 — Flag
 * Left column pinned; strong horizontal wind from the left.
 */
static void preset_flag(Cloth *c, int cols, int rows)
{
    float ox0 = (float)(CELL_W * 3);
    float oy0 = (float)(rows * CELL_H) * 0.15f;

    cloth_reset_positions(c, ox0, oy0);
    /* pin left column */
    for (int row = 0; row < CLOTH_H; row++)
        c->nodes[node_idx(0, row)].pinned = true;

    c->wind_strength = 40.0f;
    c->wind_on       = true;

    (void)cols;
}

/*
 * preset 2 — Hammock
 * Top-left and top-right corners only pinned; heavy gravity.
 */
static void preset_hammock(Cloth *c, int cols, int rows)
{
    int cloth_px_w = (CLOTH_W - 1) * REST_H;
    float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
    float oy0 = (float)(rows * CELL_H) * 0.08f + (float)CELL_H * 2;

    cloth_reset_positions(c, ox0, oy0);
    /* pin only the two top corners */
    c->nodes[node_idx(0, 0)].pinned           = true;
    c->nodes[node_idx(CLOTH_W-1, 0)].pinned   = true;

    c->wind_strength = 50.0f;
    c->wind_on       = true;

    (void)rows;
}

static const char *preset_names[N_PRESETS] = {
    "Hanging Cloth", "Flag", "Hammock"
};

static void cloth_init(Cloth *c, int preset, int cols, int rows)
{
    memset(c, 0, sizeof *c);
    c->paused      = false;
    c->preset      = preset;
    c->wind_phase  = 0.0f;

    switch (preset) {
    default:
    case 0: preset_hanging(c, cols, rows); break;
    case 1: preset_flag(c, cols, rows);    break;
    case 2: preset_hammock(c, cols, rows); break;
    }

    cloth_build_springs(c);
}

/* ── Physics tick ──────────────────────────────────────────────────── */

/*
 * cloth_step — one physics sub-step using explicit spring forces.
 *
 * 1. Accumulate external accelerations (gravity, wind) per free node.
 * 2. For each spring: compute Hooke + relative-velocity damping force;
 *    add to both endpoint accumulators.
 * 3. Symplectic Euler: vel += acc*dt; vel *= DAMP; pos += vel*dt.
 */
static void cloth_step(Cloth *c, float dt)
{
    /* per-node acceleration accumulators */
    float ax[CLOTH_N], ay[CLOTH_N];

    /*
     * External: gravity on all free nodes; wind force on free nodes.
     * Wind direction per preset:
     *   Hanging (0): sinusoidal ±1 — cloth swings side to side
     *   Flag (1):    constant +1  — flag always blown right
     *   Hammock (2): sinusoidal   — gusts from both sides
     */
    float wind_dir = (c->preset == 1) ? 1.0f : sinf(c->wind_phase);

    for (int i = 0; i < CLOTH_N; i++) {
        if (c->nodes[i].pinned) {
            ax[i] = ay[i] = 0.0f;
            continue;
        }
        ay[i] = GRAVITY;
        ax[i] = 0.0f;
        if (c->wind_on) {
            /* amplitude grows toward bottom — lower cloth bilges more */
            float y_frac = (float)(i / CLOTH_W) / (float)(CLOTH_H - 1);
            ax[i] = c->wind_strength * wind_dir * (0.4f + 0.6f * y_frac);
        }
    }

    /* Spring forces */
    for (int s = 0; s < c->n_springs; s++) {
        const Spring *sp = &c->springs[s];
        Node *na = &c->nodes[sp->a];
        Node *nb = &c->nodes[sp->b];

        float dx   = nb->x - na->x;
        float dy   = nb->y - na->y;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist < 1e-6f) continue;

        float inv  = 1.0f / dist;
        float ex   = dx * inv;   /* unit vector a→b */
        float ey   = dy * inv;

        /* relative velocity along spring axis (positive = stretching) */
        float vrel = (nb->vx - na->vx) * ex + (nb->vy - na->vy) * ey;

        float fmag = sp->k * (dist - sp->rest) + sp->kd * vrel;
        float fx   = fmag * ex;
        float fy   = fmag * ey;

        if (!na->pinned) { ax[sp->a] += fx; ay[sp->a] += fy; }
        if (!nb->pinned) { ax[sp->b] -= fx; ay[sp->b] -= fy; }
    }

    /* Symplectic Euler integration */
    for (int i = 0; i < CLOTH_N; i++) {
        Node *n = &c->nodes[i];
        if (n->pinned) continue;

        n->vx = (n->vx + ax[i] * dt) * DAMP;
        n->vy = (n->vy + ay[i] * dt) * DAMP;
        n->x += n->vx * dt;
        n->y += n->vy * dt;
    }
}

static void cloth_tick(Cloth *c, float dt)
{
    if (c->paused) return;

    /*
     * Snapshot node positions BEFORE physics runs.
     * cloth_draw lerps between (rx,ry) and (x,y) using the sub-tick
     * alpha so that every render frame gets a smooth intermediate
     * position rather than the hard physics-tick position.
     */
    for (int i = 0; i < CLOTH_N; i++) {
        c->nodes[i].rx = c->nodes[i].x;
        c->nodes[i].ry = c->nodes[i].y;
    }

    float sub_dt = dt / (float)SUB_STEPS;
    c->wind_phase += WIND_FREQ * 2.0f * (float)M_PI * dt;
    if (c->wind_phase > 2.0f * (float)M_PI) c->wind_phase -= 2.0f * (float)M_PI;

    for (int s = 0; s < SUB_STEPS; s++) {
        cloth_step(c, sub_dt);
    }
}

/* ── Drawing ───────────────────────────────────────────────────────── */

/*
 * draw_segment — DDA fill between two cell positions.
 *
 * Chooses the most appropriate ASCII char for the segment direction:
 *   nearly horizontal → '-'
 *   nearly vertical   → '|'
 *   positive slope    → '\'
 *   negative slope    → '/'
 */
static void draw_segment(WINDOW *w, int x0, int y0, int x1, int y1,
                         int cols, int rows, chtype attr)
{
    int dx    = x1 - x0;
    int dy    = y1 - y0;
    int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);
    if (steps == 0) {
        if (x0 >= 0 && x0 < cols && y0 >= 1 && y0 < rows - 1) {
            wattron(w, attr);
            mvwaddch(w, y0, x0, '+');
            wattroff(w, attr);
        }
        return;
    }

    int adx = abs(dx), ady = abs(dy);
    char ch;
    if (adx >= 2 * ady)       ch = '-';
    else if (ady >= 2 * adx)  ch = '|';
    else if (dx * dy > 0)     ch = '\\';
    else                      ch = '/';

    wattron(w, attr);
    for (int k = 0; k <= steps; k++) {
        int cx = x0 + k * dx / steps;
        int cy = y0 + k * dy / steps;
        if (cx >= 0 && cx < cols && cy >= 1 && cy < rows - 1)
            mvwaddch(w, cy, cx, ch);
    }
    wattroff(w, attr);
}

/*
 * node_lerp_cell — lerp a node's pixel position and convert to cell.
 *
 * alpha ∈ [0,1) is the sub-tick interpolation factor:
 *   alpha = sim_accum / tick_ns
 * Lerps between snapshot (rx,ry) and physics position (x,y).
 */
static inline void node_lerp_cell(const Node *n, float alpha,
                                  int *out_cx, int *out_cy)
{
    float draw_x = n->rx + (n->x - n->rx) * alpha;
    float draw_y = n->ry + (n->y - n->ry) * alpha;
    *out_cx = px_to_cell_x(draw_x);
    *out_cy = px_to_cell_y(draw_y);
}

static void cloth_draw(const Cloth *c, WINDOW *w, int cols, int rows, float alpha)
{
    for (int row = 0; row < CLOTH_H; row++) {

        /* colour pair for this cloth row */
        float frac = (float)row / (float)(CLOTH_H - 1);
        int pair;
        if      (frac < 0.25f) pair = 5;   /* cyan   */
        else if (frac < 0.50f) pair = 4;   /* green  */
        else if (frac < 0.75f) pair = 3;   /* yellow */
        else                   pair = 2;   /* orange */

        for (int col = 0; col < CLOTH_W; col++) {
            int idx = node_idx(col, row);
            const Node *n = &c->nodes[idx];

            /* lerped cell position for this node */
            int cx, cy;
            node_lerp_cell(n, alpha, &cx, &cy);

            chtype attr = (chtype)COLOR_PAIR(pair);

            /* horizontal edge to right neighbour */
            if (col + 1 < CLOTH_W) {
                const Node *nr = &c->nodes[node_idx(col+1, row)];
                int rx, ry;
                node_lerp_cell(nr, alpha, &rx, &ry);
                draw_segment(w, cx, cy, rx, ry, cols, rows, attr);
            }

            /* vertical edge to lower neighbour */
            if (row + 1 < CLOTH_H) {
                const Node *nd = &c->nodes[node_idx(col, row+1)];
                /* use the row+1 colour for the lower edge */
                float frac_d = (float)(row + 1) / (float)(CLOTH_H - 1);
                int pair_d;
                if      (frac_d < 0.25f) pair_d = 5;
                else if (frac_d < 0.50f) pair_d = 4;
                else if (frac_d < 0.75f) pair_d = 3;
                else                     pair_d = 2;
                int dx, dy;
                node_lerp_cell(nd, alpha, &dx, &dy);
                draw_segment(w, cx, cy, dx, dy, cols, rows,
                             (chtype)COLOR_PAIR(pair_d));
            }

            /* node marker — pinned nodes are bold '#' */
            if (cx >= 0 && cx < cols && cy >= 1 && cy < rows - 1) {
                chtype node_attr = n->pinned ? (attr | A_BOLD) : (attr | A_DIM);
                char   node_ch   = n->pinned ? '#' : '+';
                wattron(w, node_attr);
                mvwaddch(w, cy, cx, node_ch);
                wattroff(w, node_attr);
            }
        }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    Cloth cloth;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    cloth_init(&s->cloth, 0, cols, rows);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)cols; (void)rows;
    cloth_tick(&s->cloth, dt);
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)dt_sec;
    cloth_draw(&s->cloth, w, cols, rows, alpha);
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

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Cloth *c = &sc->cloth;

    char buf[80];
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  wind:%s ",
             fps, sim_fps, c->wind_on ? "on " : "off");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    attron(COLOR_PAIR(5) | A_BOLD);
    mvprintw(0, 1, " CLOTH: %s ", preset_names[c->preset]);
    attroff(COLOR_PAIR(5) | A_BOLD);

    attron(COLOR_PAIR(6) | A_DIM);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:restart  n/p:preset  w:wind  [/]:Hz ");
    attroff(COLOR_PAIR(6) | A_DIM);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

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
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Cloth  *c  = &app->scene.cloth;
    Screen *sc = &app->screen;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        c->paused = !c->paused;
        break;

    case 'r': case 'R':
        cloth_init(c, c->preset, sc->cols, sc->rows);
        break;

    case 'n': case 'N':
        cloth_init(c, (c->preset + 1) % N_PRESETS, sc->cols, sc->rows);
        break;

    case 'p': case 'P': {
        int p = c->preset - 1;
        if (p < 0) p = N_PRESETS - 1;
        cloth_init(c, p, sc->cols, sc->rows);
        break;
    }

    case 'w': case 'W':
        c->wind_on = !c->wind_on;
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

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        float alpha = (float)sim_accum / (float)tick_ns;

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha, dt_sec);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
