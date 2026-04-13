/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * jellyfish.c — bioluminescent jellyfish simulation
 *
 * Physics-based pulse locomotion:
 *   IDLE (sinks) → CONTRACT (jet thrust) → GLIDE (coasts) → EXPAND (blooms)
 *
 * Keys:
 *   q / ESC   quit
 *   spc       pause / resume
 *   r         reset
 *   + / -     add / remove jellyfish (1–8)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/jellyfish.c -o jellyfish -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 coords  §5 entity
 * §6 draw    §7 screen §8 app
 */

#define _POSIX_C_SOURCE 200809L
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

#define CELL_W   8
#define CELL_H  16

enum {
    N_JELLIES_DEFAULT = 1,
    N_JELLIES_MAX     = 8,
    N_TENTACLES       = 3,     /* left, center, right                     */
    TENTACLE_SEGS     = 20,
    FPS_UPDATE_MS     = 500,
    TARGET_FPS        = 60,
};

/* Bell geometry (pixels) */
#define BELL_RX_BASE    72.0f
#define BELL_RY_BASE   144.0f

/* Minimum open fraction during contraction — needs room to produce thrust */
#define MIN_OPEN         0.85f  /* crown height minimum (height axis)     */
#define HEAD_MIN_OPEN    0.85f  /* body width minimum — squeezes harder   */

/* ── Physics-based pulse locomotion ─────────────────────────────────── */
#define GRAVITY          30.0f  /* downward accel px/s² while idle (sinking) */
#define JET_THRUST      190.0f  /* upward velocity kick at peak contraction   */
#define DRAG_VY           2.4f  /* exponential drag on vertical velocity      */
#define IDLE_DUR_MIN      0.55f /* min time bell stays open between pulses    */
#define IDLE_DUR_MAX      1.10f /* max time bell stays open between pulses    */
#define DUR_CONTRACT      0.19f /* bell snap-shut  (fast ease-out)            */
#define DUR_GLIDE         0.38f /* coasting phase after thrust                */
#define DUR_EXPAND        0.68f /* bell bloom-open (slow ease-in)             */
#define TENT_LAG          0.09f /* tentacle inertia — trails on fast move     */

/* Tentacle wave dynamics */
#define TENT_SEG_PX     14.0f
#define TENT_WAVE_AMP   19.0f
#define TENT_WAVE_K      0.30f
#define TENT_WAVE_SPD    3.2f
#define TENT_PHASE_OFF   0.80f

/* Horizontal oscillation */
#define JELLY_DRIFT_A   15.0f
#define JELLY_DRIFT_HZ   0.30f

#define TAU  6.28318530f
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

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
    CP_J0 = 1, CP_J1, CP_J2, CP_J3,
    CP_J4,     CP_J5, CP_J6, CP_J7,
    CP_HUD,
};

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_J0,  51, -1);
        init_pair(CP_J1, 105, -1);
        init_pair(CP_J2, 213, -1);
        init_pair(CP_J3,  86, -1);
        init_pair(CP_J4, 159, -1);
        init_pair(CP_J5, 141, -1);
        init_pair(CP_J6, 219, -1);
        init_pair(CP_J7, 123, -1);
        init_pair(CP_HUD, 240, -1);
    } else {
        init_pair(CP_J0, COLOR_CYAN,    -1);
        init_pair(CP_J1, COLOR_BLUE,    -1);
        init_pair(CP_J2, COLOR_MAGENTA, -1);
        init_pair(CP_J3, COLOR_GREEN,   -1);
        init_pair(CP_J4, COLOR_WHITE,   -1);
        init_pair(CP_J5, COLOR_CYAN,    -1);
        init_pair(CP_J6, COLOR_MAGENTA, -1);
        init_pair(CP_J7, COLOR_BLUE,    -1);
        init_pair(CP_HUD, COLOR_WHITE,  -1);
    }
}

static const int jelly_cp[N_JELLIES_MAX] = {
    CP_J0, CP_J1, CP_J2, CP_J3, CP_J4, CP_J5, CP_J6, CP_J7
};

/* ===================================================================== */
/* §4  coords                                                             */
/* ===================================================================== */

static inline int px_col(float px) { return (int)floorf(px / (float)CELL_W); }
static inline int px_row(float py) { return (int)floorf(py / (float)CELL_H); }

/* ===================================================================== */
/* §5  entity                                                             */
/* ===================================================================== */

/*
 * Four-state physics pulse:
 *
 *  IDLE      bell = 1.0 (open)  passive sink via gravity
 *  CONTRACT  bell 1→MIN_OPEN    fast ease-out; JET_THRUST burst at end
 *  GLIDE     bell = MIN_OPEN    coast on momentum, drag decelerates
 *  EXPAND    bell MIN_OPEN→1.0  slow ease-in bloom; drift continues
 *
 * bell_open drives crown_f / head_f directly.
 * tent1_f / tent2_f are always 1.0 — tentacles fully extended, their
 * positions trail with a vy-proportional lag (inertia in draw_tentacles).
 */
typedef enum {
    ANIM_IDLE,
    ANIM_CONTRACT,
    ANIM_GLIDE,
    ANIM_EXPAND,
} AnimState;

typedef struct {
    float cx, cy;
    float drift_cx;
    float drift_phase;
    float tent_phase[N_TENTACLES];
    int   cp;

    float vy;           /* vertical velocity px/s  (negative = upward)  */
    float bell_open;    /* master fraction: 0=closed  1=fully open       */
    float idle_dur;     /* randomised duration for each IDLE phase       */

    AnimState anim;
    float     anim_t;   /* time elapsed in current state (seconds)       */

    /* section fractions consumed by draw_bell / draw_tentacles */
    float crown_f, head_f, tent1_f, tent2_f;

    /* tentacle wave amplitude scale: 1=full sway, 0=streaming straight */
    float wave_scale;
} Jellyfish;

static float randf(void) { return (float)rand() / (float)RAND_MAX; }

static float smoothstep(float t)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

/* Bell open-fraction curves:
 *   contract: ease-out (fast at start, slow at end) — snap shut
 *   expand:   ease-in  (slow at start, fast at end) — gentle bloom    */
static float curve_contract(float t) /* t in [0,1] */
{
    float inv = 1.0f - t;
    return 1.0f - inv * inv;   /* ease-out quadratic */
}
static float curve_expand(float t)
{
    return smoothstep(t);      /* smoothstep: ease-in-out but starts slow */
}

static void jelly_spawn(Jellyfish *j, int idx,
                        float max_px, float max_py, bool spread)
{
    if (idx == 0 && spread) {
        j->drift_cx = max_px * 0.5f;
        j->cx       = j->drift_cx;
        j->cy       = max_py * 0.55f;
    } else {
        float zone_w  = max_px / (float)N_JELLIES_MAX;
        float zone_lo = zone_w * (idx % N_JELLIES_MAX) + zone_w * 0.10f;
        float zone_hi = zone_lo + zone_w * 0.80f;
        j->drift_cx   = zone_lo + randf() * (zone_hi - zone_lo);
        j->cx         = j->drift_cx;
        float slot    = (float)(idx % N_JELLIES_MAX) / (float)(N_JELLIES_MAX - 1);
        float jitter  = (max_py / (float)N_JELLIES_MAX) * 0.25f;
        j->cy = spread
              ? max_py * (0.85f - 0.70f * slot) + (randf() - 0.5f) * jitter
              : max_py + TENTACLE_SEGS * TENT_SEG_PX * 0.4f + BELL_RY_BASE;
    }

    j->drift_phase = randf() * TAU;
    for (int k = 0; k < N_TENTACLES; k++)
        j->tent_phase[k] = randf() * TAU;
    j->cp = jelly_cp[idx % N_JELLIES_MAX];

    j->vy         = 0.0f;
    j->bell_open  = 1.0f;
    j->wave_scale = 1.0f;
    j->anim      = ANIM_IDLE;
    j->anim_t    = randf() * IDLE_DUR_MAX;   /* stagger so not all sync  */
    j->idle_dur  = IDLE_DUR_MIN + randf() * (IDLE_DUR_MAX - IDLE_DUR_MIN);

    j->crown_f = j->head_f = 1.0f;
    j->tent1_f = j->tent2_f = 1.0f;
}

static void jelly_anim_tick(Jellyfish *j, float dt)
{
    j->anim_t += dt;

    switch (j->anim) {

    case ANIM_IDLE:
        /* Bell fully open; passive sinking; tentacles sway freely */
        j->bell_open  = 1.0f;
        j->wave_scale = 1.0f;
        j->vy += GRAVITY * dt;
        if (j->anim_t >= j->idle_dur) {
            j->anim   = ANIM_CONTRACT;
            j->anim_t = 0.0f;
        }
        break;

    case ANIM_CONTRACT: {
        /* Bell snaps shut — ease-out (fast at start).
         * Tentacles pull inward as bell contracts: sway shrinks to ~15%. */
        float frac = j->anim_t / DUR_CONTRACT;
        if (frac > 1.0f) frac = 1.0f;
        j->bell_open  = MIN_OPEN + (1.0f - MIN_OPEN) * (1.0f - curve_contract(frac));
        j->wave_scale = 1.0f - 0.85f * frac;   /* 1.0 → 0.15 */
        if (j->anim_t >= DUR_CONTRACT) {
            j->bell_open  = MIN_OPEN;
            j->wave_scale = 0.15f;
            j->vy         = -JET_THRUST;
            j->anim       = ANIM_GLIDE;
            j->anim_t     = 0.0f;
        }
        break;
    }

    case ANIM_GLIDE:
        /* Coasting upward — tentacles stream nearly straight behind */
        j->bell_open  = MIN_OPEN;
        j->wave_scale = 0.12f;
        if (j->anim_t >= DUR_GLIDE) {
            j->anim   = ANIM_EXPAND;
            j->anim_t = 0.0f;
        }
        break;

    case ANIM_EXPAND: {
        /* Bell blooms open slowly; tentacle sway gradually returns */
        float frac = j->anim_t / DUR_EXPAND;
        if (frac > 1.0f) frac = 1.0f;
        j->bell_open  = MIN_OPEN + (1.0f - MIN_OPEN) * curve_expand(frac);
        j->wave_scale = 0.12f + 0.88f * smoothstep(frac);  /* 0.12 → 1.0 */
        if (j->anim_t >= DUR_EXPAND) {
            j->bell_open  = 1.0f;
            j->wave_scale = 1.0f;
            j->anim       = ANIM_IDLE;
            j->anim_t     = 0.0f;
            j->idle_dur   = IDLE_DUR_MIN
                          + randf() * (IDLE_DUR_MAX - IDLE_DUR_MIN);
        }
        break;
    }
    }

    /* Drive section fractions from the master bell_open.
     * crown_f → height axis (MIN_OPEN range: subtle)
     * head_f  → width axis  (HEAD_MIN_OPEN range: more squeeze) */
    j->crown_f = j->bell_open;
    {   /* remap bell_open [MIN_OPEN..1] → head_f [HEAD_MIN_OPEN..1] */
        float t = (j->bell_open - MIN_OPEN) / (1.0f - MIN_OPEN);
        j->head_f = HEAD_MIN_OPEN + (1.0f - HEAD_MIN_OPEN) * t;
    }
    j->tent1_f = 1.0f;   /* tentacles always fully extended */
    j->tent2_f = 1.0f;
}

static void jelly_tick(Jellyfish *j, float dt, float max_px, float max_py)
{
    jelly_anim_tick(j, dt);

    /* Drag on vertical velocity every frame */
    j->vy *= expf(-DRAG_VY * dt);

    j->drift_phase += TAU * JELLY_DRIFT_HZ * dt;
    for (int k = 0; k < N_TENTACLES; k++)
        j->tent_phase[k] += TENT_WAVE_SPD * j->wave_scale * dt;

    j->cy += j->vy * dt;
    j->cx  = j->drift_cx + JELLY_DRIFT_A * sinf(j->drift_phase);

    /* Clamp horizontal so drift can't walk off screen */
    if (j->cx < BELL_RX_BASE)            j->cx = BELL_RX_BASE;
    if (j->cx > max_px - BELL_RX_BASE)   j->cx = max_px - BELL_RX_BASE;

    /* Wrap: when fully above screen, respawn from below */
    float tent_extent = TENTACLE_SEGS * TENT_SEG_PX;
    if (j->cy + tent_extent < 0.0f) {
        j->cy        = max_py * 0.85f;
        j->drift_cx  = max_px * (0.15f + 0.70f * randf());
        j->drift_phase = randf() * TAU;
        j->vy         = 0.0f;
        j->anim       = ANIM_IDLE;
        j->anim_t     = 0.0f;
        j->idle_dur   = IDLE_DUR_MIN + randf() * (IDLE_DUR_MAX - IDLE_DUR_MIN);
        j->bell_open  = 1.0f;
        j->wave_scale = 1.0f;
        j->crown_f = j->head_f = j->tent1_f = j->tent2_f = 1.0f;
    }
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

/*
 * draw_bell() — scan-line render of the dome.
 * Unchanged — driven purely by j->crown_f and j->head_f.
 */
static void draw_bell(WINDOW *win, const Jellyfish *j, int cols, int rows)
{
    /* Body pulses width-wise:  Rx scales with head_f                    */
    /* Crown pulses height-wise: apex descends as crown_f decreases      */
    float Rx = BELL_RX_BASE * j->head_f;
    float Ry = BELL_RY_BASE;

    /* Crown occupies abs_ny in [0.88, 1.0] (range = 0.12).
     * Contracted crown: apex descends → only draw up to crown_ny_limit. */
    float crown_ny_limit = 0.88f + 0.12f * j->crown_f;

    int row_top = px_row(j->cy - Ry) - 1;
    int row_bot = px_row(j->cy)     + 1;
    if (row_top < 0)      row_top = 0;
    if (row_bot >= rows)  row_bot = rows - 1;

    for (int r = row_top; r <= row_bot; r++) {
        float py  = r * (float)CELL_H + (float)CELL_H * 0.5f;
        float ny  = (py - j->cy) / Ry;
        if (ny > 0.0f || ny < -1.0f) continue;

        float abs_ny = fabsf(ny);

        /* Skip crown rows that are above the contracted apex */
        if (abs_ny > crown_ny_limit) continue;

        /* Width from ellipse — already scaled via Rx above */
        float nx_edge = sqrtf(1.0f - ny * ny);
        float x_left  = j->cx - Rx * nx_edge;
        float x_right = j->cx + Rx * nx_edge;

        int cl = px_col(x_left);
        int cr = px_col(x_right);

        /* closing roof */
        if (abs_ny > 0.94f) {
            wattron(win, COLOR_PAIR(j->cp) | A_BOLD);
            for (int c = cl; c <= cr; c++)
                if (c >= 0 && c < cols) mvwaddch(win, r, c, '_');
            wattroff(win, COLOR_PAIR(j->cp) | A_BOLD);
            continue;
        }

        /* rounded crown */
        if (abs_ny > 0.88f) {
            wattron(win, COLOR_PAIR(j->cp) | A_BOLD);
            if (cl >= 0 && cl < cols) mvwaddch(win, r, cl, '(');
            if (cr >= 0 && cr < cols && cr != cl) mvwaddch(win, r, cr, ')');
            for (int c = cl + 1; c < cr; c++)
                if (c >= 0 && c < cols) mvwaddch(win, r, c, '_');
            wattroff(win, COLOR_PAIR(j->cp) | A_BOLD);
            continue;
        }

        /* edge characters */
        chtype lch, rch;
        attr_t ea;
        if (abs_ny > 0.40f) {
            lch = '/';  rch = '\\'; ea = A_BOLD;
        } else if (abs_ny > 0.14f) {
            lch = '(';  rch = ')';  ea = A_BOLD;
        } else {
            lch = '~';  rch = '~';  ea = A_NORMAL;
        }

        if (cl >= 0 && cl < cols) {
            wattron(win, COLOR_PAIR(j->cp) | ea);
            mvwaddch(win, r, cl, lch);
            wattroff(win, COLOR_PAIR(j->cp) | ea);
        }
        if (cr >= 0 && cr < cols && cr != cl) {
            wattron(win, COLOR_PAIR(j->cp) | ea);
            mvwaddch(win, r, cr, rch);
            wattroff(win, COLOR_PAIR(j->cp) | ea);
        }

        /* rim line */
        if (abs_ny <= 0.14f) {
            wattron(win, COLOR_PAIR(j->cp) | A_BOLD);
            for (int c = cl; c <= cr; c++)
                if (c >= 0 && c < cols) mvwaddch(win, r, c, '-');
            wattroff(win, COLOR_PAIR(j->cp) | A_BOLD);
            continue;
        }

        /* interior translucent dots */
        for (int c = cl + 1; c < cr; c++) {
            if (c < 0 || c >= cols) continue;
            float pcx = c * (float)CELL_W + (float)CELL_W * 0.5f;
            float nx  = (pcx - j->cx) / (BELL_RX_BASE * j->head_f + 0.001f);
            float r2  = nx * nx + ny * ny;
            if (r2 < 0.60f) continue;
            wattron(win, COLOR_PAIR(j->cp) | A_DIM);
            mvwaddch(win, r, c, '.');
            wattroff(win, COLOR_PAIR(j->cp) | A_DIM);
        }
    }
}

/*
 * draw_tentacles() — N_TENTACLES wavy lines.
 *
 * Same rendering as before, plus velocity inertia:
 * when vy < 0 (moving upward fast) tentacles stream downward behind the bell.
 * The lag grows linearly from root (0) to tip (full TENT_LAG * vy offset).
 */
static void draw_tentacles(WINDOW *win, const Jellyfish *j, int cols, int rows)
{
    int half      = TENTACLE_SEGS / 2;
    int tent1_vis = (int)(half * j->tent1_f + 0.5f);
    int tent2_vis = (int)(half * j->tent2_f + 0.5f);

    float Rx = BELL_RX_BASE * j->head_f;

    /* Inertia lag: tentacles trail opposite to motion direction */
    float lag_total = -j->vy * TENT_LAG;   /* negative vy → positive = down */

    for (int k = 0; k < N_TENTACLES; k++) {
        float t_frac = (N_TENTACLES > 1)
                     ? (float)k / (float)(N_TENTACLES - 1) : 0.5f;
        float base_x = j->cx + (t_frac - 0.5f) * 2.0f * Rx;

        for (int seg = 0; seg < TENTACLE_SEGS; seg++) {
            if (seg < half) {
                if (seg >= tent1_vis) continue;
            } else {
                if ((seg - half) >= tent2_vis) continue;
            }

            float depth = (float)seg / (float)(TENTACLE_SEGS - 1);

            /* Lag increases toward the tips */
            float lag = lag_total * depth;

            float lateral = TENT_WAVE_AMP * j->wave_scale
                          * sinf(j->tent_phase[k]
                                 - seg * TENT_WAVE_K
                                 + (float)k * TENT_PHASE_OFF);

            float px = base_x + lateral;
            float py = j->cy  + seg * TENT_SEG_PX + lag;

            int c = px_col(px);
            int r = px_row(py);
            if (c < 0 || c >= cols || r < 0 || r >= rows) continue;

            chtype ch;
            attr_t at;
            if (depth < 0.20f) {
                ch = '|'; at = A_BOLD;
            } else if (depth < 0.45f) {
                ch = '|'; at = A_NORMAL;
            } else if (depth < 0.70f) {
                ch = ':'; at = A_DIM;
            } else {
                ch = '.'; at = A_DIM;
            }

            wattron(win, COLOR_PAIR(j->cp) | at);
            mvwaddch(win, r, c, ch);
            wattroff(win, COLOR_PAIR(j->cp) | at);
        }
    }
}

/* ===================================================================== */
/* §7  screen / scene                                                     */
/* ===================================================================== */

typedef struct {
    Jellyfish jellies[N_JELLIES_MAX];
    int       n_jellies;
    bool      paused;
    float     max_px, max_py;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->max_px    = (float)(cols * CELL_W);
    s->max_py    = (float)(rows * CELL_H);
    s->n_jellies = N_JELLIES_DEFAULT;
    s->paused    = false;
    for (int i = 0; i < N_JELLIES_MAX; i++)
        jelly_spawn(&s->jellies[i], i, s->max_px, s->max_py, /*spread=*/true);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    for (int i = 0; i < s->n_jellies; i++)
        jelly_tick(&s->jellies[i], dt, s->max_px, s->max_py);
}

static void scene_draw(const Scene *s, WINDOW *win,
                       int cols, int rows, double fps)
{
    for (int i = 0; i < s->n_jellies; i++) {
        draw_tentacles(win, &s->jellies[i], cols, rows);
        draw_bell(win, &s->jellies[i], cols, rows);
    }

    char buf[80];
    snprintf(buf, sizeof buf,
             " jellyfish:%-2d  %.1f fps ", s->n_jellies, fps);
    wattron(win, COLOR_PAIR(CP_HUD) | A_DIM);
    mvwprintw(win, 0, 0, "%s", buf);
    mvwprintw(win, rows - 1, 0,
              " q:quit  spc:pause  r:reset  +/-:count ");
    wattroff(win, COLOR_PAIR(CP_HUD) | A_DIM);
}

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

static void screen_render(Screen *s, const Scene *sc, double fps)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}

static void cleanup(void) { endwin(); }

static bool app_key(App *app, int ch)
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
    case '+': case '=':
        if (sc->n_jellies < N_JELLIES_MAX) {
            int i = sc->n_jellies++;
            jelly_spawn(&sc->jellies[i], i,
                        sc->max_px, sc->max_py, /*spread=*/false);
        }
        break;
    case '-':
        if (sc->n_jellies > 1) sc->n_jellies--;
        break;
    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t prev      = clock_ns();
    int64_t fps_acc   = 0;
    int     fps_count = 0;
    double  fps_disp  = 0.0;

    while (app->running) {

        if (app->need_resize) {
            screen_resize(&app->screen);
            app->scene.max_px = (float)(app->screen.cols * CELL_W);
            app->scene.max_py = (float)(app->screen.rows * CELL_H);
            app->need_resize  = 0;
            prev = clock_ns();
        }

        int64_t now   = clock_ns();
        int64_t dt_ns = now - prev;
        prev = now;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;
        float dt = (float)dt_ns / (float)NS_PER_SEC;

        scene_tick(&app->scene, dt);

        fps_count++;
        fps_acc += dt_ns;
        if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_disp  = (double)fps_count
                      / ((double)fps_acc / (double)NS_PER_SEC);
            fps_count = 0;
            fps_acc   = 0;
        }

        screen_render(&app->screen, &app->scene, fps_disp);

        int64_t elapsed = clock_ns() - now;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        int ch = getch();
        if (ch != ERR && !app_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
