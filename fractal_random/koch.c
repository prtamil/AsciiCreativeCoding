/*
 * koch.c  —  Koch snowflake fractal, animated level-by-level
 *
 * The Koch snowflake starts as an equilateral triangle.  Each straight
 * edge is repeatedly replaced by four shorter edges: the middle third is
 * removed and replaced with an outward equilateral bump.  Repeating this
 * rule produces the classic snowflake fractal boundary.
 *
 * Animation — five levels of detail cycle automatically:
 *
 *   Level 1 :   12 segments  — triangle with bumps
 *   Level 2 :   48 segments  — bumps on bumps
 *   Level 3 :  192 segments  — fine detail emerging
 *   Level 4 :  768 segments  — very fine detail
 *   Level 5 : 3072 segments  — near-fractal resolution
 *
 * At each level the segments are drawn one by one so you watch the
 * snowflake outline appear stroke-by-stroke.  Color shifts from deep
 * blue → sky blue → white as drawing progresses.  When all segments are
 * drawn the level is held briefly, then the next level begins from a
 * blank screen.  After level 5 the cycle restarts at level 1.
 *
 * Koch subdivision rule — for segment A → B:
 *   P = A + (B-A)/3
 *   Q = A + (B-A)*2/3
 *   M = P + R(+60°)(Q-P)       ← outward equilateral bump peak
 *   Replace with: A→P, P→M, M→Q, Q→B
 *
 * All segment geometry is stored in Euclidean coordinates (circumradius=1)
 * and converted to terminal (col, row) at draw time, applying ASPECT_R to
 * keep the snowflake circular rather than squashed.
 *
 * Keys:
 *   q / ESC   quit
 *   r         restart from level 1
 *   n         skip to next level immediately
 *   ] [       faster / slower simulation
 *   p / spc   pause / resume
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra koch.c -o koch -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  grid
 *   §5  koch
 *   §6  scene
 *   §7  screen
 *   §8  app
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

enum {
    SIM_FPS_MIN     =  10,
    SIM_FPS_DEFAULT =  30,
    SIM_FPS_MAX     =  60,
    SIM_FPS_STEP    =   5,

    MAX_LEVEL       =   5,    /* highest detail level                   */
    HOLD_TICKS      =  45,    /* ticks to hold completed level (~1.5 s) */
    N_KOCH_COLORS   =   5,    /* color bands along the drawn curve      */

    /*
     * MAX_SEGS must hold the largest level: 3 × 4^MAX_LEVEL.
     * Level 5 → 3 × 1024 = 3072.  4096 gives a safe margin.
     */
    MAX_SEGS        = 4096,

    GRID_ROWS_MAX   =  80,
    GRID_COLS_MAX   = 300,

    HUD_COLS        =  46,
    FPS_UPDATE_MS   = 500,
};

/*
 * ASPECT_R — terminal cell height / width ≈ 2.
 * Corrects for non-square cells so the snowflake looks circular.
 *
 * SIN60 / COS60 — precomputed trig constants for the 60° rotation used
 * in the Koch subdivision.
 */
#define ASPECT_R  2.0f
#define SIN60     0.8660254f
#define COS60     0.5f

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

/*
 * Vivid ice gradient — cyan → teal → lime → yellow → white.
 * All colors are clearly visible on a black background.
 * Color shifts as the curve is drawn, creating a rainbow wave effect.
 *
 *   COL_K1   51   #00ffff  bright cyan    — first segments
 *   COL_K2   86   #5fffd7  teal-cyan
 *   COL_K3  118   #87ff00  lime green
 *   COL_K4  226   #ffff00  bright yellow
 *   COL_K5  231   #ffffff  white          — final segments
 */
typedef enum {
    COL_K1  = 1,   /* bright cyan   */
    COL_K2  = 2,   /* teal-cyan     */
    COL_K3  = 3,   /* lime green    */
    COL_K4  = 4,   /* bright yellow */
    COL_K5  = 5,   /* white         */
    COL_HUD = 6,
} ColorID;

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(COL_K1,   51, COLOR_BLACK);   /* bright cyan   */
        init_pair(COL_K2,   86, COLOR_BLACK);   /* teal-cyan     */
        init_pair(COL_K3,  118, COLOR_BLACK);   /* lime green    */
        init_pair(COL_K4,  226, COLOR_BLACK);   /* bright yellow */
        init_pair(COL_K5,  231, COLOR_BLACK);   /* white         */
        init_pair(COL_HUD,  51, COLOR_BLACK);   /* cyan hud      */
    } else {
        init_pair(COL_K1,  COLOR_CYAN,   COLOR_BLACK);
        init_pair(COL_K2,  COLOR_CYAN,   COLOR_BLACK);
        init_pair(COL_K3,  COLOR_GREEN,  COLOR_BLACK);
        init_pair(COL_K4,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_K5,  COLOR_WHITE,  COLOR_BLACK);
        init_pair(COL_HUD, COLOR_CYAN,   COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

/*
 * Seg — one Koch curve segment in Euclidean coordinates.
 * The snowflake is inscribed in a circle of radius 1, centred at origin.
 */
typedef struct { float x1, y1, x2, y2; } Seg;

/*
 * Grid — terminal cell buffer + pre-computed segment list.
 *
 *   cells[row][col] == 0    →  empty
 *   cells[row][col] == 1..5 →  drawn, color = COL_K1..K5
 *
 * scale   — Euclidean radius-1 maps to this many terminal columns.
 *           scale / ASPECT_R is the row equivalent.
 * drawn   — segments drawn so far (0..n_segs).
 * level   — current detail level (1..MAX_LEVEL).
 * segs_per_tick — adapts so each level takes ~2 s to draw.
 */
typedef struct {
    uint8_t cells[GRID_ROWS_MAX][GRID_COLS_MAX];
    Seg     segs[MAX_SEGS];
    int     n_segs;
    int     drawn;
    int     hold_ticks;
    int     segs_per_tick;
    int     level;
    int     rows, cols;
    float   scale;
} Grid;

/*
 * grid_seg_color — map draw index → color band 1..N_KOCH_COLORS.
 */
static uint8_t grid_seg_color(int idx, int n_segs)
{
    int c = 1 + (int)((float)idx / (float)n_segs * (float)N_KOCH_COLORS);
    return (uint8_t)(c > N_KOCH_COLORS ? N_KOCH_COLORS : c);
}

/*
 * grid_draw_seg — rasterise one segment onto the cell grid using
 * Bresenham's algorithm.  Short segments (sub-cell) mark one cell.
 */
static void grid_draw_seg(Grid *g, int idx)
{
    uint8_t color = grid_seg_color(idx, g->n_segs);
    const Seg *s  = &g->segs[idx];

    int x0 = g->cols / 2 + (int)roundf(s->x1 * g->scale);
    int y0 = g->rows / 2 - (int)roundf(s->y1 * g->scale / ASPECT_R);
    int x1 = g->cols / 2 + (int)roundf(s->x2 * g->scale);
    int y1 = g->rows / 2 - (int)roundf(s->y2 * g->scale / ASPECT_R);

    /* Bresenham */
    int dx =  abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        if (x0 >= 0 && x0 < g->cols && y0 >= 0 && y0 < g->rows)
            g->cells[y0][x0] = color;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void grid_draw(const Grid *g, WINDOW *w)
{
    for (int row = 0; row < g->rows; row++) {
        for (int col = 0; col < g->cols; col++) {
            uint8_t c = g->cells[row][col];
            if (c == 0) continue;
            attr_t attr = COLOR_PAIR((int)c);
            if (c >= COL_K4) attr |= A_BOLD;
            wattron(w, attr);
            mvwaddch(w, row, col, (chtype)'*');
            wattroff(w, attr);
        }
    }
}

/* ===================================================================== */
/* §5  koch                                                               */
/* ===================================================================== */

/*
 * Koch subdivision — recursive.
 *
 * For segment A→B at level > 0:
 *   P = A + (B-A)/3
 *   Q = A + (B-A)*2/3
 *   M = P + R(+60°)(Q-P)    ← outward bump peak
 *
 * R(+60°) is standard CCW rotation:
 *   (x, y) → (x·cos60 − y·sin60,  x·sin60 + y·cos60)
 *
 * For a CW-oriented triangle (V1→V2→V3→V1) the outward normal of each
 * edge is to the right of the travel direction.  R(+60°) on (Q−P) gives
 * a peak M that lies outside the original triangle on all three edges.
 *
 * g_segs / g_nseg — static globals used during recursive build so the
 * function signature stays simple.
 */
static Seg *g_seg_buf;
static int  g_seg_n;

static void koch_recurse(float ax, float ay, float bx, float by, int level)
{
    if (level == 0) {
        if (g_seg_n < MAX_SEGS)
            g_seg_buf[g_seg_n++] = (Seg){ ax, ay, bx, by };
        return;
    }

    float px = ax + (bx - ax) / 3.0f,  py = ay + (by - ay) / 3.0f;
    float qx = ax + (bx - ax) * 2.0f / 3.0f, qy = ay + (by - ay) * 2.0f / 3.0f;

    /* dqp = Q − P */
    float dqpx = qx - px, dqpy = qy - py;

    /* M = P + R(+60°)(dqp) */
    float mx = px + COS60 * dqpx - SIN60 * dqpy;
    float my = py + SIN60 * dqpx + COS60 * dqpy;

    koch_recurse(ax, ay, px, py, level - 1);
    koch_recurse(px, py, mx, my, level - 1);
    koch_recurse(mx, my, qx, qy, level - 1);
    koch_recurse(qx, qy, bx, by, level - 1);
}

/*
 * build_koch — populate g->segs[] for the current level.
 *
 * Starting triangle: CW equilateral, circumradius 1.
 *   V1 = (0, 1)           top
 *   V2 = (√3/2, −1/2)     bottom-right
 *   V3 = (−√3/2, −1/2)    bottom-left
 */
static void build_koch(Grid *g)
{
    g_seg_buf = g->segs;
    g_seg_n   = 0;

    int lv = g->level;
    koch_recurse( 0.0f,  1.0f,   SIN60, -0.5f, lv);   /* V1 → V2 */
    koch_recurse( SIN60, -0.5f, -SIN60, -0.5f, lv);   /* V2 → V3 */
    koch_recurse(-SIN60, -0.5f,  0.0f,   1.0f, lv);   /* V3 → V1 */

    g->n_segs = g_seg_n;

    /* Adapt draw speed so each level takes ~60 ticks (~2 s at 30 fps) */
    g->segs_per_tick = g->n_segs / 60 + 1;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    Grid grid;
    bool paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    Grid *g = &s->grid;
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;

    memset(g->cells, 0, sizeof g->cells);
    g->rows       = rows;
    g->cols       = cols;
    g->level      = 1;
    g->drawn      = 0;
    g->hold_ticks = 0;

    /*
     * scale: largest Euclidean extent (radius 1) → terminal coordinates.
     * Constrain both horizontally (cols) and vertically (rows/ASPECT_R).
     */
    float sc_col = (float)cols * 0.45f;
    float sc_row = (float)rows * 0.45f * ASPECT_R;
    g->scale = (sc_col < sc_row) ? sc_col : sc_row;

    build_koch(g);
    s->paused = false;
}

static void scene_tick(Scene *s)
{
    if (s->paused) return;

    Grid *g = &s->grid;

    if (g->drawn >= g->n_segs) {
        /* Level complete — hold briefly then advance */
        g->hold_ticks++;
        if (g->hold_ticks >= HOLD_TICKS) {
            g->level = (g->level % MAX_LEVEL) + 1;
            memset(g->cells, 0, sizeof g->cells);
            build_koch(g);
            g->drawn      = 0;
            g->hold_ticks = 0;
        }
        return;
    }

    int end = g->drawn + g->segs_per_tick;
    if (end > g->n_segs) end = g->n_segs;
    for (int i = g->drawn; i < end; i++)
        grid_draw_seg(g, i);
    g->drawn = end;
}

static void scene_draw(const Scene *s, WINDOW *w)
{
    grid_draw(&s->grid, w);
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

static void screen_free(Screen *s)  { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr);

    const Grid *g = &sc->grid;
    int pct = (g->n_segs > 0) ? g->drawn * 100 / g->n_segs : 100;
    if (pct > 100) pct = 100;

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps  level:%d/%d  %3d%%  spd:%d",
             fps, g->level, MAX_LEVEL, pct, sim_fps);
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(COL_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(COL_HUD) | A_BOLD);
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
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case 'r': case 'R':
        /* Restart from level 1 */
        scene_init(&app->scene, app->screen.cols, app->screen.rows);
        break;

    case 'n': case 'N': {
        /* Skip to next level immediately */
        Grid *g = &app->scene.grid;
        g->level = (g->level % MAX_LEVEL) + 1;
        memset(g->cells, 0, sizeof g->cells);
        build_koch(g);
        g->drawn      = 0;
        g->hold_ticks = 0;
        break;
    }

    case 'p': case 'P': case ' ':
        app->scene.paused = !app->scene.paused;
        break;

    case ']':
        if (app->sim_fps < SIM_FPS_MAX) app->sim_fps += SIM_FPS_STEP;
        break;
    case '[':
        if (app->sim_fps > SIM_FPS_MIN) app->sim_fps -= SIM_FPS_STEP;
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)clock_ns());

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
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene);
            sim_accum -= tick_ns;
        }

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

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
