/*
 * apollonian.c — Apollonian Gasket
 *
 * Start with four mutually tangent circles (one large outer circle and
 * three inner tangent circles).  Descartes' Circle Theorem gives the
 * curvature k of the fourth circle tangent to any three:
 *
 *   k₄ = k₁+k₂+k₃ ± 2√(k₁k₂ + k₂k₃ + k₃k₁)
 *
 * The center is found via the complex-number Descartes formula:
 *
 *   z₄ = (k₁z₁+k₂z₂+k₃z₃ ± 2√(k₁k₂z₁z₂+k₂k₃z₂z₃+k₃k₁z₃z₁)) / k₄
 *
 * Starting pack: k=(-1, 2, 2, 3) with known centers, then recurse.
 * Color encodes depth level: deep=dark  shallow=bright.
 *
 * Keys: q quit  p pause  r restart  +/- depth limit  1/2/3 presets
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fractal_random/apollonian.c \
 *       -o apollonian -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 descartes  §5 draw  §6 app
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define MAX_CIRCLES  8000
#define DEPTH_MAX      7
#define RENDER_NS   (1000000000LL / 30)
#define HUD_ROWS      2

/* Circle: curvature k = 1/r (negative for outer circle) */
typedef struct {
    float k;             /* curvature */
    float zx, zy;        /* center (complex: zx + i·zy) times k */
    int   depth;
} Circle;

static Circle g_circles[MAX_CIRCLES];
static int    g_n_circles;
static int    g_depth_max = DEPTH_MAX;

enum { CP_D1=1, CP_D2, CP_D3, CP_D4, CP_D5, CP_D6, CP_D7, CP_HUD };

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  color — fire palette: deep=dark, outer=bright                      */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_D1,  88, -1);   /* dark red     — deepest  */
        init_pair(CP_D2, 124, -1);
        init_pair(CP_D3, 196, -1);
        init_pair(CP_D4, 202, -1);
        init_pair(CP_D5, 208, -1);
        init_pair(CP_D6, 226, -1);
        init_pair(CP_D7, 231, -1);   /* white        — shallowest */
        init_pair(CP_HUD,244, -1);
    } else {
        init_pair(CP_D1, COLOR_RED,    -1);
        init_pair(CP_D2, COLOR_RED,    -1);
        init_pair(CP_D3, COLOR_YELLOW, -1);
        init_pair(CP_D4, COLOR_YELLOW, -1);
        init_pair(CP_D5, COLOR_WHITE,  -1);
        init_pair(CP_D6, COLOR_WHITE,  -1);
        init_pair(CP_D7, COLOR_WHITE,  -1);
        init_pair(CP_HUD,COLOR_WHITE,  -1);
    }
}

/* ===================================================================== */
/* §4  Descartes recursion                                                */
/* ===================================================================== */

/* Complex square root of (ax + i·ay) */
static void csqrt(float ax, float ay, float *rx, float *ry)
{
    float r = sqrtf(sqrtf(ax*ax + ay*ay));
    float theta = atan2f(ay, ax) * 0.5f;
    *rx = r * cosf(theta);
    *ry = r * sinf(theta);
}

/*
 * add_circle — attempt to add the Descartes tangent circle for three
 * input circles (by index).  sign = +1 or -1 selects the two solutions.
 * Returns false if curvature is too small (radius too large) or circle
 * already full.
 */
static bool add_circle(int i1, int i2, int i3, int sign, int depth)
{
    if (g_n_circles >= MAX_CIRCLES) return false;

    float k1 = g_circles[i1].k, k2 = g_circles[i2].k, k3 = g_circles[i3].k;

    /* Descartes curvature formula */
    float rad = k1*k2 + k2*k3 + k3*k1;
    if (rad < 0.f) return false;
    float sq = sqrtf(rad);
    float k4 = k1 + k2 + k3 + sign * 2.f * sq;

    if (k4 < 1.0f) return false;          /* radius > 1 — outside frame  */
    if (depth > g_depth_max) return false;

    /* Descartes center formula in complex numbers */
    /* Each circle stored as (k, kz_x, kz_y) = k*(z_x, z_y) */
    float kz1x = g_circles[i1].zx, kz1y = g_circles[i1].zy;
    float kz2x = g_circles[i2].zx, kz2y = g_circles[i2].zy;
    float kz3x = g_circles[i3].zx, kz3y = g_circles[i3].zy;

    /* sum = kz1 + kz2 + kz3 */
    float sumx = kz1x + kz2x + kz3x;
    float sumy = kz1y + kz2y + kz3y;

    /* product terms: k1k2*z1*z2 + k2k3*z2*z3 + k3k1*z3*z1
     * = (kz1)*(kz2) + (kz2)*(kz3) + (kz3)*(kz1)
     * (where kz_i = k_i * z_i, so kz_i * kz_j = k_i*k_j * z_i*z_j) */
    float p12x = kz1x*kz2x - kz1y*kz2y;
    float p12y = kz1x*kz2y + kz1y*kz2x;
    float p23x = kz2x*kz3x - kz2y*kz3y;
    float p23y = kz2x*kz3y + kz2y*kz3x;
    float p31x = kz3x*kz1x - kz3y*kz1y;
    float p31y = kz3x*kz1y + kz3y*kz1x;

    float prodx = p12x + p23x + p31x;
    float prody = p12y + p23y + p31y;

    float sqx, sqy;
    csqrt(prodx, prody, &sqx, &sqy);

    float kz4x = sumx + sign * 2.f * sqx;
    float kz4y = sumy + sign * 2.f * sqy;

    if (fabsf(k4) < 1e-6f) return false;

    /* Store circle */
    Circle *c = &g_circles[g_n_circles];
    c->k     = k4;
    c->zx    = kz4x;
    c->zy    = kz4y;
    c->depth = depth;
    g_n_circles++;
    return true;
}

/* Stack-based iterative packing */
typedef struct { int i1, i2, i3, depth; } Task;
static Task g_tasks[MAX_CIRCLES * 4];
static int  g_task_top;

static void gasket_init(void)
{
    g_n_circles = 0;
    g_task_top  = 0;

    /* Starting pack: k=(-1,2,2,3), centers in complex coords
     * Stored as (k, kz_x, kz_y) = k*(cx, cy)
     * Outer:  k=-1, z=(0,0)  → kz=(-1*(0),     -1*(0))     = (0,0)
     * Circle1: k=2,  z=(0.5,0) → kz=(2*0.5,    2*0)       = (1,0)
     * Circle2: k=2,  z=(-0.5,0)→ kz=(2*(-0.5), 2*0)       = (-1,0)
     * Circle3: k=3,  z=(0,2/3)  → kz=(3*0,     3*(2/3))   = (0,2)
     */
    g_circles[0] = (Circle){ -1.f, 0.f,  0.f, 0 };  /* outer   */
    g_circles[1] = (Circle){  2.f, 1.f,  0.f, 1 };  /* right   */
    g_circles[2] = (Circle){  2.f,-1.f,  0.f, 1 };  /* left    */
    g_circles[3] = (Circle){  3.f, 0.f,  2.f, 1 };  /* top     */
    g_n_circles  = 4;

    /* The 4th tangent to (outer, right, left) = small inner circle k=3, z=(0,-2) */
    /* But we'll let the recursion find it. Push initial tasks. */

    /* Triplets to fill: (outer,1,2), (outer,1,3), (outer,2,3), (1,2,3) */
    g_tasks[g_task_top++] = (Task){0,1,2, 2};
    g_tasks[g_task_top++] = (Task){0,1,3, 2};
    g_tasks[g_task_top++] = (Task){0,2,3, 2};
    g_tasks[g_task_top++] = (Task){1,2,3, 2};
}

/* Process one task: add a circle and push new sub-tasks */
static bool gasket_step(void)
{
    while (g_task_top > 0) {
        Task t = g_tasks[--g_task_top];
        if (t.depth > g_depth_max) continue;

        int n_before = g_n_circles;
        /* Try both Descartes solutions; one should be the "new" circle */
        for (int sign = -1; sign <= 1; sign += 2) {
            if (add_circle(t.i1, t.i2, t.i3, sign, t.depth)) {
                int ni = g_n_circles - 1;
                /* Push three new tasks */
                if (t.depth + 1 <= g_depth_max && g_task_top + 3 < (int)(sizeof g_tasks / sizeof g_tasks[0])) {
                    g_tasks[g_task_top++] = (Task){t.i1, t.i2, ni, t.depth+1};
                    g_tasks[g_task_top++] = (Task){t.i1, t.i3, ni, t.depth+1};
                    g_tasks[g_task_top++] = (Task){t.i2, t.i3, ni, t.depth+1};
                }
                (void)n_before;
                return true;
            }
        }
        (void)n_before;
    }
    return false; /* done */
}

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static int  g_rows, g_cols;
static bool g_paused;
static bool g_done;

/* Map from gasket coordinate space [-1..1] × [-1..1] to terminal */
static int gasket_to_col(float x)
{
    /* gasket fits in x∈[-1,1], map to [1, cols-1] */
    return (int)((x + 1.f) * 0.5f * (g_cols - 2)) + 1;
}

static int gasket_to_row(float y)
{
    /* y∈[-1,1], but cells are 2x taller: correct by factor 2 */
    return (int)((-y + 1.f) * 0.5f * (g_rows - HUD_ROWS - 1)) + HUD_ROWS;
}

static void draw_circle(const Circle *c)
{
    if (c->k <= 0.f) return;  /* skip outer circle */
    float r_sim  = 1.f / c->k;
    float cx_sim = c->zx / c->k;
    float cy_sim = c->zy / c->k;

    int cx_col = gasket_to_col(cx_sim);
    int cy_row = gasket_to_row(cy_sim);

    /* radius in terminal columns (aspect: 1 col ≈ 2 rows) */
    float r_col = r_sim * (float)(g_cols - 2) * 0.5f;
    float r_row = r_col * 0.5f;   /* terminal cell 2:1 aspect */

    if (r_col < 0.5f) {
        /* Too small to draw a circle: just a dot */
        if (cy_row >= HUD_ROWS && cy_row < g_rows &&
            cx_col >= 0 && cx_col < g_cols) {
            int d = c->depth < 1 ? 1 : c->depth > 7 ? 7 : c->depth;
            attron(COLOR_PAIR(d));
            mvaddch(cy_row, cx_col, '.');
            attroff(COLOR_PAIR(d));
        }
        return;
    }

    int d = c->depth < 1 ? 1 : c->depth > 7 ? 7 : c->depth;
    attron(COLOR_PAIR(d));

    /* Draw ellipse outline (aspect-corrected circle) */
    int n_steps = (int)(2.f * 3.14159f * r_col * 2.f) + 4;
    if (n_steps > 360) n_steps = 360;
    for (int s = 0; s < n_steps; s++) {
        float theta = 2.f * 3.14159f * (float)s / (float)n_steps;
        int col = cx_col + (int)(r_col * cosf(theta) + 0.5f);
        int row = cy_row + (int)(r_row * sinf(theta) + 0.5f);
        if (row >= HUD_ROWS && row < g_rows && col >= 0 && col < g_cols)
            mvaddch(row, col, 'o');
    }

    attroff(COLOR_PAIR(d));
}

static void scene_draw(void)
{
    for (int i = 0; i < g_n_circles; i++)
        draw_circle(&g_circles[i]);

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " Apollonian  q:quit  p:pause  r:restart  +/-:depth  1/2/3:presets");
    mvprintw(1, 0,
        " circles:%d  depth_max:%d  tasks:%d  %s",
        g_n_circles, g_depth_max, g_task_top,
        g_done ? "complete" : (g_paused ? "PAUSED" : "building"));
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §6  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

int main(void)
{
    atexit(cleanup);
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();
    getmaxyx(stdscr, g_rows, g_cols);
    gasket_init();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            gasket_init();
            g_done = false;
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case 'r': case 'R': gasket_init(); g_done = false; break;
        case '+': case '=':
            g_depth_max++; if (g_depth_max > DEPTH_MAX) g_depth_max = DEPTH_MAX;
            gasket_init(); g_done = false;
            break;
        case '-':
            g_depth_max--; if (g_depth_max < 1) g_depth_max = 1;
            gasket_init(); g_done = false;
            break;
        case '1': g_depth_max = 3; gasket_init(); g_done = false; break;
        case '2': g_depth_max = 5; gasket_init(); g_done = false; break;
        case '3': g_depth_max = 7; gasket_init(); g_done = false; break;
        default: break;
        }

        long long now = clock_ns();

        if (!g_paused && !g_done) {
            for (int s = 0; s < 50; s++)
                if (!gasket_step()) { g_done = true; break; }
        }

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
