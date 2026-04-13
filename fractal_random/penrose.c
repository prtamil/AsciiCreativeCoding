/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * penrose.c — Penrose Tiling (P3 Rhombus)
 *
 * Computes a Penrose rhombus tiling per terminal cell using de Bruijn's
 * pentagrid duality.  No tiles are stored; each cell is coloured in O(1)
 * from its pentagrid indices.  The view rotates slowly so the aperiodic
 * structure is clearly visible and no period is ever found.
 *
 * DE BRUIJN PENTAGRID METHOD
 * ──────────────────────────
 * Five families of parallel lines, family j with direction 2πj/5:
 *
 *   k_j(x,y) = ⌊ x·cos(2πj/5) + y·sin(2πj/5) − γ_j ⌋
 *
 * where γ_j are offset parameters (0 here for 5-fold symmetry at origin).
 *
 * The 5-tuple (k_0,…,k_4) uniquely identifies which Penrose rhombus a
 * point lies in.  The parity of S = k_0+k_1+k_2+k_3+k_4 distinguishes
 * the two rhombus types:
 *   S even → thick rhombus  (72° acute angle)
 *   S odd  → thin  rhombus  (36° acute angle)
 *
 * Adjacent cells in the same rhombus share the same k-tuple → same colour.
 * Cells in different rhombuses have different tuples → colour changes at
 * tile boundaries, making the pattern visible without explicit edge drawing.
 *
 * ASPECT RATIO CORRECTION
 * ────────────────────────
 * Terminal cells are CELL_H/CELL_W ≈ 2× taller than wide.
 * Cell (col, row) is converted to pixel offset (px, py) before
 * projecting to the pentagrid.  This keeps rhombus proportions correct.
 *
 * ANIMATION
 * ─────────
 * The pixel coordinate frame rotates at ROTATE_SPEED rad/s.
 * The Penrose tiling has 5-fold symmetry, so period = 2π/5 ≈ 1.26 rad.
 * At 0.04 rad/s the view completes one "distinct cycle" in ~31 s, making
 * the aperiodic nature obvious — no configuration repeats.
 *
 * COLOUR SCHEME
 * ─────────────
 * Thick rhombuses (72° wide): '#' A_BOLD in warm gold/amber/orange tones.
 * Thin  rhombuses (36° narrow): '+' in cool cyan/sky-blue/lavender tones.
 * Hash = abs(k_0·3 + k_1·7 + k_2·11 + k_3·13 + k_4·17) mod 6
 * gives six visually distinct shades per type, so same-type neighbours
 * are distinguishable without explicit edge drawing.
 *
 * Keys:
 *   q/ESC quit   space pause   r reset angle   ] / [  sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra penrose.c -o penrose -lncurses -lm
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
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 30,   /* pure visual; 30 fps is smooth enough       */
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    = 10,
    FPS_UPDATE_MS   = 500,
    N_COLORS        = 12,
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Pixel cell dimensions */
#define CELL_W   8
#define CELL_H   16

/*
 * SCALE_PX — pixels per pentagrid unit.
 * With CELL_W=8 and SCALE_PX=80: 1 unit = 10 terminal columns.
 * Each rhombus side spans ~10 cols so tile shapes are clearly visible.
 */
#define SCALE_PX   80.0f

/*
 * BORDER — distance (in pentagrid units) from a grid line that is
 * rendered as a tile edge character instead of tile interior.
 * 0.15 → ~1-2 cell wide border at SCALE_PX=80.
 */
#define BORDER     0.15f

/*
 * ROTATE_SPEED — angular velocity of the view in radians per second.
 * 0.04 rad/s → one 5-fold period (72°) traversed in ~31 seconds.
 */
#define ROTATE_SPEED  0.04f

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
        init_pair(1,  196, COLOR_BLACK);   /* red          */
        init_pair(2,  208, COLOR_BLACK);   /* orange       */
        init_pair(3,  226, COLOR_BLACK);   /* yellow       */
        init_pair(4,   46, COLOR_BLACK);   /* green        */
        init_pair(5,   51, COLOR_BLACK);   /* cyan         */
        init_pair(6,   75, COLOR_BLACK);   /* light blue   */
        init_pair(7,  201, COLOR_BLACK);   /* magenta      */
        /* penrose-specific warm/cool palette */
        init_pair(8,  220, COLOR_BLACK);   /* gold         */
        init_pair(9,  214, COLOR_BLACK);   /* amber        */
        init_pair(10,  87, COLOR_BLACK);   /* aqua         */
        init_pair(11, 147, COLOR_BLACK);   /* lavender     */
        init_pair(12, 228, COLOR_BLACK);   /* pale yellow  */
        init_pair(13, 226, COLOR_BLACK);   /* yellow — HUD */
    } else {
        init_pair(1,  COLOR_RED,     COLOR_BLACK);
        init_pair(2,  COLOR_RED,     COLOR_BLACK);
        init_pair(3,  COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4,  COLOR_GREEN,   COLOR_BLACK);
        init_pair(5,  COLOR_CYAN,    COLOR_BLACK);
        init_pair(6,  COLOR_BLUE,    COLOR_BLACK);
        init_pair(7,  COLOR_MAGENTA, COLOR_BLACK);
        init_pair(8,  COLOR_YELLOW,  COLOR_BLACK);
        init_pair(9,  COLOR_YELLOW,  COLOR_BLACK);
        init_pair(10, COLOR_CYAN,    COLOR_BLACK);
        init_pair(11, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(12, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(13, COLOR_YELLOW,  COLOR_BLACK);   /* HUD */
    }
}

/* ===================================================================== */
/* §4  coords — penrose works in pixel space for aspect correction         */
/* ===================================================================== */

/* ===================================================================== */
/* §5  entity — Penrose                                                   */
/* ===================================================================== */

/*
 * Precomputed pentagrid direction cosines and sines.
 * cos(2πj/5) and sin(2πj/5) for j = 0..4.
 * Using exact values: cos(72°) = (√5−1)/4, sin(72°) = √(10+2√5)/4, etc.
 */
static const float COS5[5] = {
     1.0f,
     0.30901699f,   /* cos(72°)  */
    -0.80901699f,   /* cos(144°) */
    -0.80901699f,   /* cos(216°) */
     0.30901699f,   /* cos(288°) */
};
static const float SIN5[5] = {
     0.0f,
     0.95105652f,   /* sin(72°)  */
     0.58778525f,   /* sin(144°) */
    -0.58778525f,   /* sin(216°) */
    -0.95105652f,   /* sin(288°) */
};

typedef struct {
    float angle;   /* current view rotation (radians)                    */
    bool  paused;
} Penrose;

static void penrose_init(Penrose *p)
{
    p->angle  = 0.0f;
    p->paused = false;
}

static void penrose_tick(Penrose *p, float dt)
{
    if (p->paused) return;
    p->angle += ROTATE_SPEED * dt;
    /* Keep in [0, 2π) to avoid float drift */
    if (p->angle >= 2.0f * (float)M_PI)
        p->angle -= 2.0f * (float)M_PI;
}

/*
 * penrose_draw — render the Penrose tiling into window w.
 *
 * For each terminal cell:
 *   1. Map to pixel offset, rotate, scale → pentagrid (wx, wy).
 *   2. Compute k[j] = floor(wx·cos_j + wy·sin_j) and frac distance
 *      to the nearest grid line for each family j.
 *   3. If the minimum frac-distance < BORDER → draw a directional
 *      edge character ('|' '/' '\' '-') whose angle matches the
 *      actual grid line direction on screen.  This makes tile outlines
 *      visible regardless of neighbour colours.
 *   4. Otherwise fill the interior: '*' bold warm (thick 72°) or
 *      '.' cool (thin 36°), coloured by a hash of the k-tuple so
 *      same-type neighbours use different shades.
 */
static void penrose_draw(const Penrose *p, WINDOW *w, int cols, int rows)
{
    float cx = (float)cols * 0.5f;
    float cy = (float)rows * 0.5f;
    float ca = cosf(p->angle);
    float sa = sinf(p->angle);

    /* Warm colours for thick-tile interiors: yellow, gold, amber */
    static const int WARM[3] = { 3, 8, 9 };
    /* Cool colours for thin-tile interiors: cyan, light-blue, aqua */
    static const int COOL[3] = { 5, 6, 10 };

    for (int row = 1; row < rows - 1; row++) {
        float py = ((float)row - cy) * (float)CELL_H;

        for (int col = 0; col < cols; col++) {
            float px = ((float)col - cx) * (float)CELL_W;

            /* Rotate view */
            float rx = px * ca - py * sa;
            float ry = px * sa + py * ca;

            /* Scale to pentagrid unit coordinates */
            float wx = rx / SCALE_PX;
            float wy = ry / SCALE_PX;

            /* Compute floor indices and minimum frac-distance to any grid line */
            int   k[5], sum = 0;
            float min_dist = 1.0f;
            int   near_j   = 0;

            for (int j = 0; j < 5; j++) {
                float proj = wx * COS5[j] + wy * SIN5[j];
                k[j] = (int)floorf(proj);
                sum += k[j];
                float frac = proj - (float)k[j];           /* ∈ [0,1) */
                float dist = frac < 0.5f ? frac : 1.0f - frac;
                if (dist < min_dist) { min_dist = dist; near_j = j; }
            }

            bool thick = ((sum & 1) == 0);
            int  h     = abs(k[0]*3 + k[1]*7 + k[2]*11 + k[3]*13 + k[4]*17) % 3;

            if (min_dist < BORDER) {
                /*
                 * Tile edge: pick a line character whose slope matches
                 * the grid line of family near_j as seen on screen.
                 *
                 * Family j's lines are perpendicular to e_j = 2πj/5.
                 * After the view rotation the line itself runs at:
                 *   ang = 2π·j/5 + π/2 − view_angle
                 * Fold into [0, π) for the four ASCII slopes.
                 */
                float ang = (float)(2.0 * M_PI * near_j / 5.0 + M_PI * 0.5)
                            - p->angle;
                ang = fmodf(ang, (float)M_PI);
                if (ang < 0.0f) ang += (float)M_PI;

                char ech;
                if      (ang < 0.26f || ang > 2.88f) ech = '-';
                else if (ang < 1.05f)                 ech = '/';
                else if (ang < 2.09f)                 ech = '|';
                else                                  ech = '\\';

                wattron(w, COLOR_PAIR(7) | A_DIM);
                mvwaddch(w, row, col, (chtype)(unsigned char)ech);
                wattroff(w, COLOR_PAIR(7) | A_DIM);

            } else if (thick) {
                /* Thick rhombus interior — warm bold fill */
                wattron(w, COLOR_PAIR(WARM[h]) | A_BOLD);
                mvwaddch(w, row, col, '*');
                wattroff(w, COLOR_PAIR(WARM[h]) | A_BOLD);

            } else {
                /* Thin rhombus interior — cool dim fill */
                wattron(w, COLOR_PAIR(COOL[h]) | 0);
                mvwaddch(w, row, col, '.');
                wattroff(w, COLOR_PAIR(COOL[h]) | 0);
            }
        }
    }

    /* 5-fold axis marker at screen centre */
    int cc = (int)cx, cr = (int)cy;
    if (cc >= 0 && cc < cols && cr >= 1 && cr < rows - 1) {
        wattron(w, COLOR_PAIR(3) | A_BOLD);
        mvwaddch(w, cr, cc, 'O');
        wattroff(w, COLOR_PAIR(3) | A_BOLD);
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct { Penrose penrose; } Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    (void)cols; (void)rows;
    memset(s, 0, sizeof *s);
    penrose_init(&s->penrose);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)cols; (void)rows;
    penrose_tick(&s->penrose, dt);
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    penrose_draw(&s->penrose, w, cols, rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Penrose *p = &sc->penrose;
    char buf[80];
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%3d Hz  angle:%.1f°  %s ",
             fps, sim_fps,
             (double)(p->angle * 180.0f / (float)M_PI),
             p->paused ? "PAUSED" : "");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    attron(COLOR_PAIR(5) | A_BOLD);
    mvprintw(0, 1, " PENROSE P3 ");
    attroff(COLOR_PAIR(5) | A_BOLD);

    attron(COLOR_PAIR(13) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:reset angle  [/]:Hz "
             "  *=thick(gold)  .=thin(cyan)  |/\\-=edges ");
    attroff(COLOR_PAIR(13) | A_BOLD);
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

static bool app_handle_key(App *app, int ch)
{
    Penrose *p = &app->scene.penrose;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': p->paused = !p->paused; break;
    case 'r': case 'R': p->angle = 0.0f; break;
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
            endwin(); refresh();
            getmaxyx(stdscr, app->screen.rows, app->screen.cols);
            app->need_resize = 0;
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
