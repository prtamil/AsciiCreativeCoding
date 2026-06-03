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
 *   q/ESC quit   space pause   r reset angle   +/- speed   ] / [  sim Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra penrose.c -o penrose -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : De Bruijn dual/pentagrid method for Penrose P3 rhombuses.
 *                  Rather than recursive deflation (splitting tiles), this
 *                  method projects a 5D integer lattice onto 2D via five
 *                  directions e_j = (cos 2πj/5, sin 2πj/5).  Each rhombus
 *                  corresponds to a pair of grid lines from two directions.
 *
 * Math           : A Penrose tiling is quasiperiodic: non-periodic but with
 *                  long-range order (well-defined diffraction peaks).  The
 *                  "inflation" symmetry: each tile can be subdivided into
 *                  φ² smaller tiles of the same two types (φ = golden ratio ≈ 1.618).
 *                  Ratio of thick:thin rhombus counts → φ as tiling grows.
 *                  No translational periodicity, but 5-fold local symmetry.
 *
 * Rendering      : Each terminal cell is classified by its 5-integer k-tuple
 *                  (which pentagrid lines it sits between).  Adjacent cells
 *                  in the same tile share the same tuple → same colour.
 *                  Animation shifts the phase offsets φ_j slowly, morphing
 *                  the tiling continuously without breaking its quasiperiodic structure.
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
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 30,   /* pure visual; 30 fps is smooth enough       */
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    = 10,
    FPS_UPDATE_MS   = 500,

    /* Rotation-speed multiplier level (+/- keys; halve / double around    */
    /* SPEED_DEF, so SPEED_DEF == 1x of ROTATE_SPEED).                     */
    SPEED_MIN       =  1,
    SPEED_DEF       =  8,
    SPEED_MAX       = 64,

    HUD_TOP_ROWS    =  2,   /* rows 0-1 = data band; tiling starts at row 2 */
};

/* HUD colour pairs — reuse the tiling palette (see color_init). */
#define HUD_DATA   8        /* yellow — top data band                      */
#define HUD_LABEL  4        /* cyan   — title + bottom action bar          */

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
        init_pair(1, 226, COLOR_BLACK);   /* yellow     — thick fill / centre   */
        init_pair(2, 220, COLOR_BLACK);   /* gold       — thick fill            */
        init_pair(3, 214, COLOR_BLACK);   /* amber      — thick fill            */
        init_pair(4,  51, COLOR_BLACK);   /* cyan       — thin fill / HUD label  */
        init_pair(5,  75, COLOR_BLACK);   /* light blue — thin fill             */
        init_pair(6,  87, COLOR_BLACK);   /* aqua       — thin fill             */
        init_pair(7, 201, COLOR_BLACK);   /* magenta    — tile edges            */
        init_pair(8, 226, COLOR_BLACK);   /* yellow     — HUD data band         */
    } else {
        init_pair(1, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(2, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_CYAN,    COLOR_BLACK);
        init_pair(5, COLOR_BLUE,    COLOR_BLACK);
        init_pair(6, COLOR_CYAN,    COLOR_BLACK);
        init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(8, COLOR_YELLOW,  COLOR_BLACK);
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
    int   speed;   /* rotation-speed level SPEED_MIN..SPEED_MAX (+/- keys) */
    bool  paused;
} Penrose;

static void penrose_init(Penrose *p)
{
    p->angle  = 0.0f;
    p->speed  = SPEED_DEF;
    p->paused = false;
}

static void penrose_tick(Penrose *p, float dt)
{
    if (p->paused) return;
    float speed_mul = (float)p->speed / (float)SPEED_DEF;
    p->angle += ROTATE_SPEED * speed_mul * dt;
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
    static const int WARM[3] = { 1, 2, 3 };
    /* Cool colours for thin-tile interiors: cyan, light-blue, aqua */
    static const int COOL[3] = { 4, 5, 6 };

    for (int row = HUD_TOP_ROWS; row < rows - 1; row++) {
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
    if (cc >= 0 && cc < cols && cr >= HUD_TOP_ROWS && cr < rows - 1) {
        wattron(w, COLOR_PAIR(1) | A_BOLD);
        mvwaddch(w, cr, cc, 'O');
        wattroff(w, COLOR_PAIR(1) | A_BOLD);
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

/*
 * hud_print — draw a HUD string at (y, x) clipped to the terminal width, so a
 * narrow window can never wrap HUD text down into the tiling. Right-aligned
 * callers pass x = cols - len; a negative x is pinned to 0 and the text is
 * truncated to whatever space remains.
 */
static void hud_print(int y, int x, int cols, int pair, int attr, const char *s)
{
    if (x < 0) x = 0;
    int avail = cols - x;
    if (avail <= 0) return;
    char tmp[128];
    snprintf(tmp, sizeof tmp, "%s", s);
    if ((int)strlen(tmp) > avail) tmp[avail] = '\0';   /* clip to fit */
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(y, x, "%s", tmp);
    attroff(COLOR_PAIR(pair) | attr);
}

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Penrose *p = &sc->penrose;

    /* ── top band: data (rows 0-1) ──────────────────────────────────── */
    hud_print(0, 1, s->cols, HUD_LABEL, A_BOLD, " PENROSE P3 ");

    char status[80];
    snprintf(status, sizeof status, " %5.1f fps  %3d Hz  speed:%-3d  %s ",
             fps, sim_fps, p->speed, p->paused ? "PAUSED " : "running");
    hud_print(0, s->cols - (int)strlen(status), s->cols, HUD_DATA, A_BOLD, status);

    char data[80];
    snprintf(data, sizeof data,
             " angle:%5.1f deg   *=thick(warm)  .=thin(cool)  /|\\-=edges ",
             (double)(p->angle * 180.0f / (float)M_PI));
    hud_print(1, 1, s->cols, HUD_DATA, 0, data);

    /* ── bottom band: actions (last row) ────────────────────────────── */
    hud_print(s->rows - 1, 0, s->cols, HUD_LABEL, A_BOLD,
              " q:quit  spc:pause  r:reset  +/-:speed  [/]:Hz ");
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
    case '=': case '+':
        if (p->speed < SPEED_MAX) p->speed *= 2;
        if (p->speed > SPEED_MAX) p->speed  = SPEED_MAX;
        break;
    case '-':
        p->speed /= 2;
        if (p->speed < SPEED_MIN) p->speed  = SPEED_MIN;
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
