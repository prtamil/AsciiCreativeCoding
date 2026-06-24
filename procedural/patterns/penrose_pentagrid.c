/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * penrose_pentagrid.c — a Penrose tiling that never repeats, in the terminal.
 *
 * Penrose tilings cover the plane with just two diamond shapes (a fat one and
 * a thin one) but never settle into a repeating pattern. We don't store any
 * tiles: for each screen cell we ask "which diamond does this spot fall in?"
 * and colour it. The view turns slowly so you can watch the pattern never
 * line up with itself.
 *
 * The trick that lets us answer that question per cell is de Bruijn's
 * "pentagrid" method — see N. G. de Bruijn, "Algebraic theory of Penrose's
 * non-periodic tilings of the plane," Indag. Math. 43 (1981), 39-66. For a
 * gentle walkthrough, D. Austin, "Penrose Tiles Talk Across Miles," AMS
 * Feature Column (2005). The shapes themselves are from R. Penrose (1974).
 *
 * Keys: q/ESC quit · space pause · r reset angle · +/- speed · [ ] sim Hz
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
    SIM_FPS_DEFAULT = 30,   /* nothing to compute hard, so 30 looks smooth */
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    = 10,
    FPS_UPDATE_MS   = 500,

    /* How fast the view spins, as a whole-number dial. The +/- keys double
     * or halve it. SPEED_DEF is the "1x" setting (the base ROTATE_SPEED). */
    SPEED_MIN       =  1,
    SPEED_DEF       =  8,
    SPEED_MAX       = 64,

    HUD_TOP_ROWS    =  2,   /* rows 0-1 hold the readout; tiles start at row 2 */

    N_SHADES        =  3,   /* shades per tile type, so touching tiles differ */
};

/* HUD colours, borrowed from the tiling palette set up in color_init. */
#define HUD_DATA   8        /* yellow — the numbers across the top         */
#define HUD_LABEL  4        /* cyan   — the title and the bottom key bar    */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* A terminal cell is taller than it is wide; treat it as 8x16 sub-pixels so
 * the diamonds come out the right shape instead of stretched. */
#define CELL_W   8
#define CELL_H   16

/* How much to zoom: 80 sub-pixels per tiling unit makes one diamond edge
 * about 10 columns wide, big enough to actually see the shapes. */
#define SCALE_PX   80.0f

/* A cell this close to a tile boundary gets drawn as an edge mark instead of
 * filled interior. 0.15 gives a roughly 1-2 cell wide outline at this zoom. */
#define BORDER     0.15f

/* Base turn rate of the view, radians per second. At 0.04 it takes about
 * half a minute to turn through one "fifth" of a full circle. */
#define ROTATE_SPEED  0.04f

/* Two colour slots used by name at draw time (the 1..6 fill colours are
 * reached through WARM/COOL instead). Both are set up in color_init. */
#define PAIR_CENTRE  1      /* yellow  — the dot marking the centre of symmetry */
#define PAIR_EDGE    7      /* magenta — the lines between tiles                 */

/* When a tile boundary crosses a cell we draw a little line slanted to match.
 * These cutoffs sort the boundary's on-screen angle into one of four marks
 * (- / | \), each covering the range of angles it looks most like. */
#define SLOPE_DASH_LO  0.26f   /* under this (or over _DASH_HI): '-' flat   */
#define SLOPE_FSLASH   1.05f   /* under this: '/'                           */
#define SLOPE_PIPE     2.09f   /* under this: '|' upright; otherwise '\'    */
#define SLOPE_DASH_HI  2.88f   /* over this: back to '-' flat               */

/* ===================================================================== */
/* §2  PERFORMANCE — timing primitives                                    */
/* ===================================================================== */
/* Just a clock and a sleep. The actual frame pacing that uses them lives  */
/* in the main loop (§5).                                                  */

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
/* §3  SIMULATION — state + advance                                       */
/* ===================================================================== */
/* All the moving state is the Penrose struct below: one turn angle plus    */
/* its two dials. penrose_tick is the only thing that advances it.          */

/*
 * Penrose — the spinning view of the tiling.
 *
 * The pattern itself never changes; it's infinite and fully decided by the
 * fixed pentagrid. The only thing that moves is the angle we look at it from,
 * which we turn slowly so you can see the pattern slide by and never repeat —
 * which is the whole point of a Penrose tiling. All three fields are about
 * that one rotation, so they live together.
 *
 *   angle  — how far the view has turned, in radians. This is the only thing
 *            that actually changes over time. We wrap it back into one full
 *            circle every tick so the number can't drift after running a long
 *            while (see penrose_tick).
 *   speed  — the spin dial, a whole number from SPEED_MIN to SPEED_MAX that
 *            the +/- keys double or halve. We keep it whole (not a raw rate)
 *            so the steps are clean and SPEED_DEF means exactly normal speed.
 *   paused — when true, penrose_tick bails out early and the angle holds still.
 */
typedef struct {
    float angle;
    int   speed;
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
    /* wrap back to under a full turn so the number stays small and exact */
    if (p->angle >= 2.0f * (float)M_PI)
        p->angle -= 2.0f * (float)M_PI;
}

/*
 * Scene — everything being simulated, bundled in one object the sim and draw
 * code can pass around instead of reaching for globals. Here that's just the
 * one rotating tiling; a busier program would list more pieces, one per field.
 *   penrose — the rotating tiling (see Penrose).
 */
typedef struct {
    Penrose penrose;
} Scene;

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

/* ===================================================================== */
/* §4  RENDER — state → screen (reads only, never mutates sim state)      */
/* ===================================================================== */
/* Turns the current angle into pixels. The one cell→pixel→tiling-space      */
/* conversion is inline in penrose_draw; edge_glyph and rhombus_shade are     */
/* small pure helpers it leans on.                                            */

/*
 * Pentagrid — the five directions de Bruijn's method is built on.
 *
 * Here is the idea that makes this whole demo possible. Imagine five families
 * of evenly-spaced parallel lines, each family rotated 72 degrees from the
 * last, so they fan out at 0, 72, 144, 216, 288 degrees. Overlay all five and
 * the gaps between the crossings ARE the Penrose diamonds — that's de Bruijn's
 * "pentagrid" trick. We store one unit-length arrow per direction.
 *
 * Why five at 72 degrees? That fivefold spread is exactly what no repeating
 * grid can do, so the result can't repeat either — which is what we want. Any
 * other count just gives back a boring periodic grid. The cos/sin values below
 * are the golden-ratio numbers that run through every Penrose pattern.
 *
 * How we use it: to find which diamond a point sits in, slide the point onto
 * each of the five arrows and round down. Those five whole numbers name the
 * diamond, and whether they add up to an even or odd total tells fat from thin.
 * See penrose_draw.
 *
 * We precompute the cos/sin once at file scope and never touch them again,
 * because penrose_draw does this projection for every cell every frame and the
 * grid itself is fixed — only the view that samples it turns.
 *   cos[j] / sin[j] — the x / y parts of direction j (direction 0 points right).
 */
typedef struct {
    float cos[5];
    float sin[5];
} Pentagrid;

static const Pentagrid PENTAGRID = {
    /*        0deg    72deg         144deg        216deg        288deg      */
    .cos = {  1.0f,  0.30901699f, -0.80901699f, -0.80901699f,  0.30901699f },
    .sin = {  0.0f,  0.95105652f,  0.58778525f, -0.58778525f, -0.95105652f },
};

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

/* Fill colours, three shades each, picked by rhombus_shade. WARM is for the
 * fat diamonds (yellow/gold/amber), COOL for the thin ones (cyan/blue/aqua).
 * The numbers are the colour-pair ids set up in color_init. */
static const int WARM[N_SHADES] = { 1, 2, 3 };
static const int COOL[N_SHADES] = { 4, 5, 6 };

/* Draw one character in a given colour, then put the colour state back the way
 * it was. The single spot a coloured tile reaches the screen. */
static void put_cell(WINDOW *w, int row, int col, int pair, int attr, chtype ch)
{
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, row, col, ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/* Pick the slanted mark (- / | \) that matches how one family's boundary lines
 * lie on screen after the view has turned. The lines run crosswise to the
 * family's arrow, so we work out that on-screen angle and read off the glyph. */
static char edge_glyph(int near_j, float view_angle)
{
    float ang = (float)(2.0 * M_PI * near_j / 5.0 + M_PI * 0.5) - view_angle;
    ang = fmodf(ang, (float)M_PI);
    if (ang < 0.0f) ang += (float)M_PI;

    if (ang < SLOPE_DASH_LO || ang > SLOPE_DASH_HI) return '-';
    if (ang < SLOPE_FSLASH)                         return '/';
    if (ang < SLOPE_PIPE)                           return '|';
    return '\\';
}

/* Pick one of the three shades for a diamond from its five index numbers.
 * It's a quick scramble of those numbers: neighbouring diamonds have different
 * indices, so they usually land on different shades and you can see the seam
 * between tiles even without a drawn border. */
static int rhombus_shade(const int k[5])
{
    return abs(k[0]*3 + k[1]*7 + k[2]*11 + k[3]*13 + k[4]*17) % N_SHADES;
}

/*
 * penrose_draw — fill the window with the tiling. For each cell: figure out
 * where it lands in the turned tiling, ask which diamond that is, and colour
 * it (or mark a boundary line if it sits on the seam between two diamonds).
 */
static void penrose_draw(const Penrose *p, WINDOW *w, int cols, int rows)
{
    float cx = (float)cols * 0.5f;          /* middle of the screen, in cells */
    float cy = (float)rows * 0.5f;
    float ca = cosf(p->angle);              /* the turn, ready to apply       */
    float sa = sinf(p->angle);

    for (int row = HUD_TOP_ROWS; row < rows - 1; row++) {
        float py = ((float)row - cy) * (float)CELL_H;

        for (int col = 0; col < cols; col++) {
            float px = ((float)col - cx) * (float)CELL_W;

            /* (1) turn this cell by the view angle and zoom to tiling units */
            float rx = px * ca - py * sa;
            float ry = px * sa + py * ca;
            float wx = rx / SCALE_PX;
            float wy = ry / SCALE_PX;

            /* (2) slide the point onto each of the five arrows; the rounded-down
             *     value is its index in that family. Also track how close it
             *     came to a boundary line and which family that line belongs to. */
            int   k[5], sum = 0;
            float min_dist = 1.0f;          /* closest we got to any seam      */
            int   near_j   = 0;             /* which family that seam came from */

            for (int j = 0; j < 5; j++) {
                float proj = wx * PENTAGRID.cos[j] + wy * PENTAGRID.sin[j];
                k[j] = (int)floorf(proj);
                sum += k[j];
                float frac         = proj - (float)k[j];                 /* leftover part */
                float dist_to_line = frac < 0.5f ? frac : 1.0f - frac;   /* to nearer seam */
                if (dist_to_line < min_dist) { min_dist = dist_to_line; near_j = j; }
            }

            bool thick = ((sum & 1) == 0);  /* even total => fat diamond, odd => thin */

            /* (3) draw it */
            if (min_dist < BORDER)          /* sitting on a seam: draw the line  */
                put_cell(w, row, col, PAIR_EDGE, A_DIM,
                         (chtype)(unsigned char)edge_glyph(near_j, p->angle));
            else if (thick)                 /* inside a fat diamond: warm '*'    */
                put_cell(w, row, col, WARM[rhombus_shade(k)], A_BOLD, '*');
            else                            /* inside a thin diamond: cool '.'   */
                put_cell(w, row, col, COOL[rhombus_shade(k)], 0, '.');
        }
    }

    /* dot marking the centre where the fivefold symmetry pivots */
    int cc = (int)cx, cr = (int)cy;
    if (cc >= 0 && cc < cols && cr >= HUD_TOP_ROWS && cr < rows - 1)
        put_cell(w, cr, cc, PAIR_CENTRE, A_BOLD, 'O');
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    penrose_draw(&s->penrose, w, cols, rows);
}

/*
 * Screen — how big the terminal is right now, in character cells. We remember
 * it instead of asking ncurses on every cell, and only re-check it when it can
 * actually change: at start-up and whenever the window is resized (see main).
 * Keep it current before drawing or the tiling would clip or wrap.
 *   cols / rows — width and height in cells. Tiles fill the middle; the top two
 *                 rows are the readout and the bottom row is the key bar.
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

/* Print a HUD line, cut off at the screen edge so a narrow window can't spill
 * the text down into the tiling. An off-screen x is pulled back to 0. */
static void hud_print(int y, int x, int cols, int pair, int attr, const char *s)
{
    if (x < 0) x = 0;
    int avail = cols - x;
    if (avail <= 0) return;
    char tmp[128];
    snprintf(tmp, sizeof tmp, "%s", s);
    if ((int)strlen(tmp) > avail) tmp[avail] = '\0';   /* cut off at the edge */
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(y, x, "%s", tmp);
    attroff(COLOR_PAIR(pair) | attr);
}

static void screen_draw(const Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Penrose *p = &sc->penrose;

    /* top two rows: title and the live numbers */
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

    /* bottom row: the key bar */
    hud_print(s->rows - 1, 0, s->cols, HUD_LABEL, A_BOLD,
              " q:quit  spc:pause  r:reset  +/-:speed  [/]:Hz ");
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §5  APP — user events + per-tick combine                              */
/* ===================================================================== */
/* main wires it all together: read time, advance the sim, draw. Keypresses */
/* and OS signals change state outside that loop body.                      */

/*
 * App — the whole running program in one place: the scene plus the bits the
 * main loop and the signal handlers both need to see. We keep one copy at file
 * scope (g_app) because a signal handler is handed only an int — it has no way
 * to reach a local, so the flags it flips have to be reachable globally.
 *
 *   scene       — what's being simulated (see Scene).
 *   screen      — remembered terminal size (see Screen).
 *   sim_fps     — how many times a second to step the sim ([ and ] keys). This
 *                 is a loop setting, separate on purpose from Penrose.speed,
 *                 which is the spin dial — two different things.
 *   running     — the loop keeps going while this is set; Ctrl-C, kill, or 'q'
 *                 clears it so we exit and hand the terminal back clean.
 *   need_resize — flipped on when the window is resized, then handled once at
 *                 the top of the loop. Both flags are volatile sig_atomic_t,
 *                 the only kind of variable a signal handler may safely touch.
 */
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
        /* 1. window was resized: re-read its size and restart the clock */
        if (app->need_resize) {
            endwin(); refresh();
            getmaxyx(stdscr, app->screen.rows, app->screen.cols);
            app->need_resize = 0;
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* 2. how long since last frame, capped so that if we were paused or
         *    frozen for a while we don't try to catch up all at once */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        /* 3. step the sim forward in fixed-size chunks until we've used up the
         *    time that passed, so motion stays the same speed at any frame rate */
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        float alpha = (float)sim_accum / (float)tick_ns;

        /* 4. update the fps number shown in the HUD about twice a second */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* 5. wait out the rest of the frame so we don't run faster than 60 fps */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* 6. draw the frame and push it to the terminal */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha, dt_sec);
        screen_present();

        /* 7. deal with one keypress if there's one waiting */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
