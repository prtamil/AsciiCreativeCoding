/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * magnetic_field.c — 2D Magnetic Field Lines from Configurable Dipoles
 *
 * Visualises the static magnetic field of 1–4 bar magnets (dipoles) placed
 * on screen.  Each dipole is a pair of equal-and-opposite magnetic charges
 * (monopoles) separated by a fixed length.  The field at every point is the
 * superposition of Coulomb-like fields from all monopoles:
 *
 *   B(r) = sum_i  q_i * (r − r_i) / |r − r_i|³
 *
 * Field lines are traced by 4th-order Runge-Kutta (RK4) streamline
 * integration along B, seeded at N_SEEDS points around each positive pole.
 * Arrows on each line segment show the field direction.
 *
 * Presets:
 *   0  Dipole      — single bar magnet, classic closed-loop lines
 *   1  Quadrupole  — two magnets head-to-head, X-type neutral point
 *   2  Attract     — two magnets N→S facing each other
 *   3  Repel       — two magnets N→N facing each other
 *
 * Keys:
 *   q / ESC    quit
 *   r          restart / rebuild current preset
 *   n / N      next / previous preset
 *   t / T      next / previous theme
 *   ] / [      trace speed up / down  (lines per tick)
 *   p / space  pause / resume
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra physics/magnetic_field.c \
 *       -o magnetic_field -lncurses -lm
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MIN     = 5,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    = 5,

    MAX_MONOPOLES   = 8,    /* max magnetic charges (2 per dipole)        */
    MAX_DIPOLES     = 4,    /* max bar magnets                            */
    N_SEEDS         = 16,   /* field lines seeded around each + pole      */
    MAX_LINE_STEPS  = 600,  /* max RK4 steps per field line               */
    MAX_LINES       = MAX_DIPOLES * N_SEEDS,  /* total lines to trace     */

    N_PRESETS       = 4,
    N_THEMES        = 5,

    LINES_PER_TICK_DEF = 2, /* field lines traced per tick                */
    LINES_PER_TICK_MIN = 1,
    LINES_PER_TICK_MAX = 8,

    ROWS_MAX        = 128,
    COLS_MAX        = 512,
};

#define NS_PER_SEC      1000000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* RK4 step size in normalised cell units */
#define RK4_H           0.35f
/* Stop tracing if |B| < this (near null points) */
#define B_MIN           1e-6f
/* Monopole softening radius in cell units */
#define SOFT            0.8f
/* Seed circle radius around a pole, in cell units */
#define SEED_R          1.2f
/* Arrow placed every ARR_STRIDE steps along a line */
#define ARR_STRIDE      18
/* Aspect ratio correction: terminal cells are ~2x taller than wide */
#define ASPECT_R        2.0f

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
/* §3  color / theme                                                      */
/* ===================================================================== */

/*
 * Color pair assignments:
 *   1   field lines (positive/dim)
 *   2   field lines (stronger)
 *   3   field lines (brightest)
 *   4   North pole symbol (+)
 *   5   South pole symbol (-)
 *   6   dipole body
 *   7   HUD
 */
enum {
    CP_LINE_DIM = 1,
    CP_LINE_MID = 2,
    CP_LINE_BRT = 3,
    CP_NORTH    = 4,
    CP_SOUTH    = 5,
    CP_BODY     = 6,
    CP_HUD      = 7,
};

typedef struct {
    short line_dim, line_mid, line_brt;  /* 256-color fg */
    short north, south, body, hud;
    short line_dim8, line_mid8, line_brt8;  /* 8-color fallback */
    short north8, south8, body8, hud8;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* Electric — cyan field on black */
    { 51, 87, 231,  196, 21,  250, 244,
      COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,
      COLOR_RED, COLOR_BLUE, COLOR_WHITE, COLOR_WHITE },
    /* Plasma — magenta/violet */
    { 93, 165, 207,  226, 57,  248, 244,
      COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE,
      COLOR_YELLOW, COLOR_BLUE, COLOR_WHITE, COLOR_WHITE },
    /* Fire — red/orange field */
    { 124, 208, 226,  231, 21,  250, 244,
      COLOR_RED, COLOR_RED, COLOR_YELLOW,
      COLOR_WHITE, COLOR_BLUE, COLOR_WHITE, COLOR_WHITE },
    /* Ocean — blue/teal */
    { 25, 39, 123,  196, 226,  250, 244,
      COLOR_BLUE, COLOR_CYAN, COLOR_WHITE,
      COLOR_RED, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE },
    /* Matrix — green */
    { 22, 46, 118,  196, 21,  250, 244,
      COLOR_GREEN, COLOR_GREEN, COLOR_WHITE,
      COLOR_RED, COLOR_BLUE, COLOR_WHITE, COLOR_WHITE },
};

static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    if (COLORS >= 256) {
        init_pair(CP_LINE_DIM, th->line_dim, COLOR_BLACK);
        init_pair(CP_LINE_MID, th->line_mid, COLOR_BLACK);
        init_pair(CP_LINE_BRT, th->line_brt, COLOR_BLACK);
        init_pair(CP_NORTH,    th->north,    COLOR_BLACK);
        init_pair(CP_SOUTH,    th->south,    COLOR_BLACK);
        init_pair(CP_BODY,     th->body,     COLOR_BLACK);
        init_pair(CP_HUD,      th->hud,      COLOR_BLACK);
    } else {
        init_pair(CP_LINE_DIM, th->line_dim8, COLOR_BLACK);
        init_pair(CP_LINE_MID, th->line_mid8, COLOR_BLACK);
        init_pair(CP_LINE_BRT, th->line_brt8, COLOR_BLACK);
        init_pair(CP_NORTH,    th->north8,    COLOR_BLACK);
        init_pair(CP_SOUTH,    th->south8,    COLOR_BLACK);
        init_pair(CP_BODY,     th->body8,     COLOR_BLACK);
        init_pair(CP_HUD,      th->hud8,      COLOR_BLACK);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    theme_apply(0);
}

/* ===================================================================== */
/* §4  magnetic physics                                                   */
/* ===================================================================== */

/*
 * A Monopole is a magnetic charge: charge q (positive = north, negative
 * = south) at position (cx, cy) in cell coordinates.
 * A Dipole is a pair of monopoles (N pole + S pole) connected by a body.
 */

typedef struct {
    float cx, cy;  /* center in cell coords */
    float q;       /* +1 = north, -1 = south */
} Monopole;

typedef struct {
    float nx, ny;  /* north pole position */
    float sx, sy;  /* south pole position */
    int   color;   /* dipole body color pair */
} Dipole;

/*
 * Compute B field at (px, py) from all monopoles.
 * B += q_i * (r - r_i) / |r - r_i|^3
 * Softening avoids divergence at the pole center.
 */
static void field_at(const Monopole *mp, int nm,
                     float px, float py,
                     float *bx, float *by)
{
    *bx = 0.0f; *by = 0.0f;
    for (int i = 0; i < nm; i++) {
        float dx = px - mp[i].cx;
        float dy = (py - mp[i].cy) * ASPECT_R;  /* aspect correction */
        float r2 = dx*dx + dy*dy + SOFT*SOFT;
        float r  = sqrtf(r2);
        float inv3 = mp[i].q / (r2 * r);
        *bx += inv3 * dx;
        *by += inv3 * dy / ASPECT_R;
    }
}

/* ===================================================================== */
/* §5  scene — field lines & dipoles                                      */
/* ===================================================================== */

/*
 * A FieldLine is one traced streamline.  We store only the drawn cells
 * plus an arrow direction flag at ARR_STRIDE intervals.
 */
typedef struct {
    int  col[MAX_LINE_STEPS];
    int  row[MAX_LINE_STEPS];
    char ch[MAX_LINE_STEPS];    /* drawn character for that step */
    int  cp[MAX_LINE_STEPS];    /* color pair index */
    int  len;
    bool done;
} FieldLine;

typedef struct {
    /* monopoles */
    Monopole  mp[MAX_MONOPOLES];
    int       nm;

    /* dipoles (for drawing) */
    Dipole    dp[MAX_DIPOLES];
    int       nd;

    /* traced field lines */
    FieldLine lines[MAX_LINES];
    int       n_lines;
    int       lines_traced;   /* how many lines fully traced so far */

    /* animation state */
    int       lines_per_tick;
    bool      paused;
    int       preset;
    int       theme;

    int       cols, rows;
} Scene;

/* ── Arrow character from direction ───────────────────────────────── */

static char direction_arrow(float dx, float dy)
{
    /* 8-way arrow based on angle */
    float angle = atan2f(dy * ASPECT_R, dx);
    /* normalize angle to [0, 2pi) */
    if (angle < 0) angle += 2.0f * (float)M_PI;
    int sector = (int)(angle / ((float)M_PI / 4.0f) + 0.5f) % 8;
    static const char sym[8] = { '>', 'v', '<', '^', '>', 'v', '<', '^' };
    return sym[sector % 4 + (sector % 2 == 1 ? 0 : 0)];
}

/* ── Line character from direction ────────────────────────────────── */

static char line_char(float dx, float dy)
{
    float angle = atan2f(dy * ASPECT_R, dx);
    if (angle < 0) angle += (float)M_PI;
    if (angle >= (float)M_PI) angle -= (float)M_PI;
    /* normalize to [0, pi) and pick character */
    float deg = angle * 180.0f / (float)M_PI;
    if (deg < 22.5f || deg >= 157.5f) return '-';
    if (deg < 67.5f)  return '\\';
    if (deg < 112.5f) return '|';
    return '/';
}

/* ── Color pair for a step along a line based on distance from pole ── */

static int line_color_pair(int step, int total)
{
    /* bright near source pole, dim farther away */
    if (total <= 0) return CP_LINE_DIM;
    int third = total / 3;
    if (step < third)       return CP_LINE_BRT;
    if (step < 2 * third)   return CP_LINE_MID;
    return CP_LINE_DIM;
}

/* ── RK4 streamline trace for one seed ───────────────────────────── */

static void trace_line(FieldLine *fl, const Monopole *mp, int nm,
                       float sx, float sy, int cols, int rows)
{
    fl->len  = 0;
    fl->done = true;

    float px = sx, py = sy;
    float h  = RK4_H;

    for (int step = 0; step < MAX_LINE_STEPS; step++) {
        /* bounds check */
        if (px < 0 || px >= (float)cols || py < 0 || py >= (float)rows)
            break;

        int col = (int)(px + 0.5f);
        int row = (int)(py + 0.5f);
        if (col < 0 || col >= cols || row < 0 || row >= rows) break;

        /* RK4 */
        float k1x, k1y, k2x, k2y, k3x, k3y, k4x, k4y;
        field_at(mp, nm, px,           py,           &k1x, &k1y);
        field_at(mp, nm, px+0.5f*h*k1x, py+0.5f*h*k1y, &k2x, &k2y);
        field_at(mp, nm, px+0.5f*h*k2x, py+0.5f*h*k2y, &k3x, &k3y);
        field_at(mp, nm, px+h*k3x,     py+h*k3y,     &k4x, &k4y);

        float bx = (k1x + 2*k2x + 2*k3x + k4x) / 6.0f;
        float by = (k1y + 2*k2y + 2*k3y + k4y) / 6.0f;
        float bmag = sqrtf(bx*bx + by*by);
        if (bmag < B_MIN) break;

        /* normalize and step */
        bx /= bmag; by /= bmag;

        /* choose character */
        char ch;
        bool is_arrow = (step > 0 && step % ARR_STRIDE == 0);
        if (is_arrow) {
            ch = direction_arrow(bx, by);
        } else {
            ch = line_char(bx, by);
        }

        fl->col[fl->len] = col;
        fl->row[fl->len] = row;
        fl->ch[fl->len]  = ch;
        fl->cp[fl->len]  = 0;  /* set after full trace in color_pass */
        fl->len++;

        px += h * bx;
        py += h * by;
    }

    /* assign colors based on full length */
    for (int i = 0; i < fl->len; i++) {
        fl->cp[i] = line_color_pair(i, fl->len);
    }
}

/* ── Add a dipole: north at (nx,ny), south at (sx,sy) ────────────── */

static void scene_add_dipole(Scene *s,
                              float nx, float ny,
                              float sx, float sy)
{
    if (s->nd >= MAX_DIPOLES || s->nm + 2 > MAX_MONOPOLES) return;

    s->mp[s->nm] = (Monopole){ nx, ny, +1.0f };
    s->nm++;
    s->mp[s->nm] = (Monopole){ sx, sy, -1.0f };
    s->nm++;

    s->dp[s->nd] = (Dipole){ nx, ny, sx, sy, CP_BODY };
    s->nd++;
}

/* ── Seed field lines from all north poles ────────────────────────── */

static void scene_seed_lines(Scene *s)
{
    s->n_lines = 0;
    s->lines_traced = 0;

    for (int i = 0; i < s->nm && s->n_lines < MAX_LINES; i++) {
        if (s->mp[i].q < 0) continue;  /* only seed from north poles */

        for (int k = 0; k < N_SEEDS && s->n_lines < MAX_LINES; k++) {
            float angle = (float)k / (float)N_SEEDS * 2.0f * (float)M_PI;
            float sx = s->mp[i].cx + SEED_R * cosf(angle);
            float sy = s->mp[i].cy + SEED_R * sinf(angle) / ASPECT_R;
            s->lines[s->n_lines].done = false;
            s->lines[s->n_lines].len  = 0;
            /* store seed in col[0], row[0] temporarily */
            /* We'll trace during tick */
            /* pack seed as float in col/row — just store index */
            (void)sx; (void)sy;
            /* store seed coords for later tracing */
            /* encode: use a side array approach — trace immediately */
            trace_line(&s->lines[s->n_lines], s->mp, s->nm,
                       sx, sy, s->cols, s->rows);
            s->n_lines++;
        }
    }
    /* All lines are now traced; reveal them incrementally via lines_traced */
    s->lines_traced = 0;
}

/* ── Preset builders ──────────────────────────────────────────────── */

static void scene_build_preset(Scene *s)
{
    float cx = (float)s->cols * 0.5f;
    float cy = (float)s->rows * 0.5f;
    float dx = (float)s->cols * 0.18f;  /* half-separation in x */
    float dy = (float)s->rows * 0.28f;  /* half-separation in y */

    s->nm = 0; s->nd = 0;

    switch (s->preset) {
    case 0:
        /* Single dipole — N on left, S on right */
        scene_add_dipole(s,
            cx - dx, cy,   /* north */
            cx + dx, cy);  /* south */
        break;
    case 1:
        /* Quadrupole — two dipoles pointing same direction, side by side
         * N-S / N-S stacked vertically → X-type null point in the middle */
        scene_add_dipole(s,
            cx, cy - dy * 0.8f,   /* north (top) */
            cx, cy + dy * 0.8f);  /* south (bottom) */
        scene_add_dipole(s,
            cx - dx * 1.4f, cy,
            cx + dx * 1.4f, cy);
        break;
    case 2:
        /* Attract — two magnets N→S facing each other:
         *   N S    N S
         *  (left)  (right)
         *  S poles facing inward => strong attraction */
        scene_add_dipole(s,
            cx - dx * 1.8f, cy,   /* left dipole: N far-left, S near-center */
            cx - dx * 0.4f, cy);
        scene_add_dipole(s,
            cx + dx * 0.4f, cy,   /* right dipole: N near-center, S far-right */
            cx + dx * 1.8f, cy);
        break;
    case 3:
        /* Repel — two magnets N→N facing each other */
        scene_add_dipole(s,
            cx - dx * 0.4f, cy,   /* left dipole: N on right side, S far left */
            cx - dx * 1.8f, cy);
        scene_add_dipole(s,
            cx + dx * 0.4f, cy,   /* right dipole: N on left side, S far right */
            cx + dx * 1.8f, cy);
        break;
    }
}

static void scene_init(Scene *s, int cols, int rows, int preset, int theme)
{
    memset(s, 0, sizeof *s);
    s->cols           = cols;
    s->rows           = rows;
    s->preset         = preset;
    s->theme          = theme;
    s->lines_per_tick = LINES_PER_TICK_DEF;

    scene_build_preset(s);
    scene_seed_lines(s);
}

/* ── Tick: reveal next batch of lines ────────────────────────────── */

static void scene_tick(Scene *s)
{
    if (s->paused) return;
    int to_reveal = s->lines_per_tick;
    while (to_reveal-- > 0 && s->lines_traced < s->n_lines) {
        s->lines_traced++;
    }
}

/* ── Draw ─────────────────────────────────────────────────────────── */

static void scene_draw(const Scene *s)
{
    /* draw revealed field lines */
    for (int li = 0; li < s->lines_traced; li++) {
        const FieldLine *fl = &s->lines[li];
        for (int i = 0; i < fl->len; i++) {
            int r = fl->row[i], c = fl->col[i];
            if (r < 0 || r >= s->rows || c < 0 || c >= s->cols) continue;
            int attr = fl->cp[i] ? COLOR_PAIR(fl->cp[i]) : COLOR_PAIR(CP_LINE_DIM);
            mvaddch(r, c, (chtype)fl->ch[i] | attr);
        }
    }

    /* draw dipole bodies */
    for (int i = 0; i < s->nd; i++) {
        const Dipole *dp = &s->dp[i];

        /* body line between poles */
        float dx = dp->sx - dp->nx;
        float dy = dp->sy - dp->ny;
        float dist = sqrtf(dx*dx + dy*dy);
        if (dist > 0.5f) {
            int steps = (int)(dist * 2);
            for (int k = 0; k <= steps; k++) {
                float t = (float)k / (float)steps;
                int c = (int)(dp->nx + t * dx + 0.5f);
                int r = (int)(dp->ny + t * dy + 0.5f);
                if (r >= 0 && r < s->rows && c >= 0 && c < s->cols)
                    mvaddch(r, c, (chtype)'=' | COLOR_PAIR(CP_BODY) | A_DIM);
            }
        }

        /* north pole marker */
        int nr = (int)(dp->ny + 0.5f);
        int nc = (int)(dp->nx + 0.5f);
        if (nr >= 0 && nr < s->rows && nc >= 0 && nc < s->cols) {
            mvaddch(nr, nc, (chtype)'N' | COLOR_PAIR(CP_NORTH) | A_BOLD);
        }
        /* south pole marker */
        int sr = (int)(dp->sy + 0.5f);
        int sc = (int)(dp->sx + 0.5f);
        if (sr >= 0 && sr < s->rows && sc >= 0 && sc < s->cols) {
            mvaddch(sr, sc, (chtype)'S' | COLOR_PAIR(CP_SOUTH) | A_BOLD);
        }
    }
}

/* ===================================================================== */
/* §6  screen / HUD                                                       */
/* ===================================================================== */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
}

static void screen_draw_hud(const Scene *s, int fps)
{
    static const char *preset_names[N_PRESETS] = {
        "Dipole", "Quadrupole", "Attract", "Repel"
    };
    static const char *theme_names[N_THEMES] = {
        "Electric", "Plasma", "Fire", "Ocean", "Matrix"
    };

    int pct = s->n_lines > 0
            ? s->lines_traced * 100 / s->n_lines
            : 100;

    int row = s->rows - 1;
    move(row, 0);
    clrtoeol();
    attron(COLOR_PAIR(CP_HUD));
    printw(" MagField  q:quit  n:preset[%s]  t:theme[%s]  "
           "][: speed(%d)  p:pause  %3d%%  %2dfps%s",
           preset_names[s->preset],
           theme_names[s->theme],
           s->lines_per_tick,
           pct, fps,
           s->paused ? " [PAUSED]" : "");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §7  signal handling                                                    */
/* ===================================================================== */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit   = 0;

static void handle_sigwinch(int s) { (void)s; g_resize = 1; }
static void handle_sigterm(int s)  { (void)s; g_quit   = 1; }

/* ===================================================================== */
/* §8  main loop                                                          */
/* ===================================================================== */

int main(void)
{
    signal(SIGWINCH, handle_sigwinch);
    signal(SIGTERM,  handle_sigterm);
    signal(SIGINT,   handle_sigterm);

    screen_init();
    color_init();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    int   sim_fps    = SIM_FPS_DEFAULT;
    int   cur_preset = 0;
    int   cur_theme  = 0;

    Scene scene;
    scene_init(&scene, cols, rows - 1, cur_preset, cur_theme);

    int64_t t_last = clock_ns();

    /* FPS counter */
    int64_t fps_acc  = 0;
    int     fps_cnt  = 0;
    int     fps_disp = 0;

    while (!g_quit) {
        /* ── resize ── */
        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            scene_init(&scene, cols, rows - 1, cur_preset, cur_theme);
        }

        /* ── input ── */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_quit = 1; break;
            case 'r':
                scene_init(&scene, cols, rows - 1, cur_preset, cur_theme);
                break;
            case 'n':
                cur_preset = (cur_preset + 1) % N_PRESETS;
                scene_init(&scene, cols, rows - 1, cur_preset, cur_theme);
                break;
            case 'N':
                cur_preset = (cur_preset + N_PRESETS - 1) % N_PRESETS;
                scene_init(&scene, cols, rows - 1, cur_preset, cur_theme);
                break;
            case 't':
                cur_theme = (cur_theme + 1) % N_THEMES;
                scene.theme = cur_theme;
                theme_apply(cur_theme);
                break;
            case 'T':
                cur_theme = (cur_theme + N_THEMES - 1) % N_THEMES;
                scene.theme = cur_theme;
                theme_apply(cur_theme);
                break;
            case ']':
                if (scene.lines_per_tick < LINES_PER_TICK_MAX)
                    scene.lines_per_tick++;
                break;
            case '[':
                if (scene.lines_per_tick > LINES_PER_TICK_MIN)
                    scene.lines_per_tick--;
                break;
            case 'p': case ' ':
                scene.paused = !scene.paused;
                break;
            case '+': case '=':
                if (sim_fps < SIM_FPS_MAX) sim_fps += SIM_FPS_STEP;
                break;
            case '-':
                if (sim_fps > SIM_FPS_MIN) sim_fps -= SIM_FPS_STEP;
                break;
            }
        }

        /* ── tick ── */
        scene_tick(&scene);

        /* ── draw ── */
        erase();
        scene_draw(&scene);
        screen_draw_hud(&scene, fps_disp);
        wnoutrefresh(stdscr);
        doupdate();

        /* ── FPS cap ── */
        int64_t t_now  = clock_ns();
        int64_t t_used = t_now - t_last;
        int64_t t_tick = TICK_NS(sim_fps);
        clock_sleep_ns(t_tick - t_used);
        t_last = clock_ns();

        /* ── FPS counter ── */
        fps_acc += t_used + (t_tick - t_used > 0 ? t_tick - t_used : 0);
        fps_cnt++;
        if (fps_acc >= NS_PER_SEC / 2) {
            fps_disp = fps_cnt * 2;
            fps_acc  = 0;
            fps_cnt  = 0;
        }

        /* ── auto-cycle when all lines revealed ── */
        if (scene.lines_traced >= scene.n_lines && !scene.paused) {
            /* pause 2 seconds then move to next preset */
            static int hold_ticks = 0;
            hold_ticks++;
            if (hold_ticks >= sim_fps * 3) {
                hold_ticks   = 0;
                cur_preset = (cur_preset + 1) % N_PRESETS;
                scene_init(&scene, cols, rows - 1, cur_preset, cur_theme);
            }
        }
    }

    endwin();
    return 0;
}
