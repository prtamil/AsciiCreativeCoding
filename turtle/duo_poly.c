/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * duo_poly.c — two turtles draw regular polygons side by side, one edge at
 * a time, then auto-cycle to a polygon with one more side (3..12, then back).
 *
 * The turtle idea (a pen with a heading) comes from Papert, "Mindstorms"
 * (1980) and Abelson & diSessa, "Turtle Geometry" (1981).
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

/* ── §1 config ── */

enum {
    SIM_FPS_MIN     =   5,
    SIM_FPS_DEFAULT =  30,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    =   5,

    N_COLORS        =   7,
    SIDES_MIN       =   3,
    SIDES_MAX       =  12,

    HUD_COLS        =  64,
    FPS_UPDATE_MS   = 500,
};

#define EPS_DEFAULT   1.5f   /* edges per second (default drawing speed) */
#define EPS_MIN       0.3f
#define EPS_MAX      12.0f
#define EPS_SCALE_FACTOR 1.5f /* + / - keys scale eps geometrically by this */
#define RESET_DELAY   2.0f   /* seconds to wait after both polygons done */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ── §1.1 where each turtle sits on screen ── */

/* Each turtle is centred in its half: a quarter of the way across for the
 * left one, three quarters for the right one. */
#define HALF_LEFT_X_FRAC   0.25f
#define HALF_RIGHT_X_FRAC  0.75f

/* How big to draw a polygon. We pick the smaller of a width limit and a
 * height limit so the shape fits its half, then shrink a little to leave
 * room for the name label on top and the key hints at the bottom. */
#define POLY_MAX_R_X_FRAC  0.21f
#define POLY_MAX_R_Y_FRAC  0.40f
#define POLY_R_FIT_FRAC    0.85f

/* ── §1.2 main-loop pacing ── */

#define RENDER_FPS         60
#define RENDER_FRAME_NS    (NS_PER_SEC / RENDER_FPS)

/* If the program stalls (say it was paused by the OS), don't let one frame
 * report a huge time gap and try to catch up all at once — cap the gap. */
#define DT_CAP_NS          (100 * NS_PER_MS)

/* ── §2 clock ── */

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

/* ── §3 color ── */

/* The colours we draw with. The two HUD bars sit on the terminal's own
 * background so they stay readable over whatever the turtles draw. */
#define PAIR_TURTLE_A      1
#define PAIR_TURTLE_B      2
#define PAIR_DONE          3
#define PAIR_POLY_NAME     4
#define PAIR_FLASH         5
#define PAIR_DIVIDER       6
#define PAIR_HEAD          7
#define PAIR_HUD           8
#define PAIR_HINT          9

/*
 * One row of the colour table below: a role and the colours it gets.
 * Listing both colour choices side by side means there's a single place
 * to read the whole palette, instead of two separate code paths to keep
 * in sync.
 */
typedef struct {
    short pair;     /* which PAIR_* role this row sets up                */
    short fg256;    /* text colour on a 256-colour terminal             */
    short fg8;      /* fallback text colour on an old 8-colour terminal */
    short bg;       /* background: black for the drawing, -1 (the       *
                     * terminal's own) for the HUD bars                 */
} PaletteEntry;

static const PaletteEntry PALETTE[] = {
    /* pair             256   8-fallback        bg          role     */
    { PAIR_TURTLE_A,     51,  COLOR_CYAN,    COLOR_BLACK }, /* cyan        */
    { PAIR_TURTLE_B,    201,  COLOR_MAGENTA, COLOR_BLACK }, /* magenta     */
    { PAIR_DONE,        226,  COLOR_YELLOW,  COLOR_BLACK }, /* yellow      */
    { PAIR_POLY_NAME,    46,  COLOR_GREEN,   COLOR_BLACK }, /* green       */
    { PAIR_FLASH,       196,  COLOR_RED,     COLOR_BLACK }, /* red         */
    { PAIR_DIVIDER,      33,  COLOR_BLUE,    COLOR_BLACK }, /* blue        */
    { PAIR_HEAD,        255,  COLOR_WHITE,   COLOR_BLACK }, /* white       */
    { PAIR_HUD,         226,  COLOR_YELLOW,  -1          }, /* HUD chrome  */
    { PAIR_HINT,         51,  COLOR_CYAN,    -1          }, /* HINT chrome */
};
#define PALETTE_LEN  (int)(sizeof PALETTE / sizeof PALETTE[0])

static void color_init(void)
{
    start_color();
    use_default_colors();
    bool truecolor = (COLORS >= 256);
    for (int i = 0; i < PALETTE_LEN; i++) {
        short fg = truecolor ? PALETTE[i].fg256 : PALETTE[i].fg8;
        init_pair(PALETTE[i].pair, fg, PALETTE[i].bg);
    }
}

/* ── §4 coords ── */

/* A terminal character is about twice as tall as it is wide, so a circle
 * drawn naively comes out as a tall oval. We squash the up/down direction
 * by ASPECT (about 0.5) so squares look square and circles look round. */
#define CELL_W   8
#define CELL_H  16
#define ASPECT   ((float)CELL_W / (float)CELL_H)   /* 0.5 */

/* Pick the line character that best matches the direction an edge runs in,
 * so a slanted edge reads as a single straight stroke rather than a stack
 * of dashes. */
static char angle_char(float angle)
{
    float a = fmodf(angle, (float)M_PI);
    if (a < 0.0f) a += (float)M_PI;
    if (a < (float)M_PI / 8.0f || a >= 7.0f * (float)M_PI / 8.0f) return '-';
    if (a < 3.0f * (float)M_PI / 8.0f)                             return '/';
    if (a < 5.0f * (float)M_PI / 8.0f)                             return '|';
    return '\\';
}

/* ── §5 the turtle ── */

/*
 * The shape a turtle is drawing: a regular polygon, described by where its
 * centre is, how big it is, how many sides, and where the first corner
 * sits. We split this out from the drawing-progress (the Pen below) so it's
 * obvious which fields are "the shape" and which are "how far along we are".
 * Set once when a turtle starts and not touched again until it restarts.
 *
 * The corners come from evenly spacing points around a circle; see
 * poly_vertex() for the actual formula. We store the radius in plain column
 * widths and squash the up/down direction only at draw time, which keeps
 * the maths here simple.
 */
typedef struct {
    float cx;            /* centre, in columns (a quarter or three      *
                          * quarters of the way across the screen)      */
    float cy;            /* centre, in rows (roughly the middle)        */
    float radius;        /* size, in column widths; the up/down         *
                          * squash is applied later in poly_vertex      */
    int   sides;         /* how many sides, 3 to 12                      */
    float start_angle;   /* where the first corner points; always       *
                          * straight up so the shape starts at the top  */
} Polygon;

/*
 * How far along the drawing is for one turtle — the moving part, updated
 * every tick (the shape itself lives in Polygon above and never changes
 * mid-draw). `edge` counts how many sides are finished, `edge_timer` is the
 * little countdown to the next side appearing, and `done` flips on when the
 * shape closes.
 */
typedef struct {
    int   edge;          /* sides finished so far; reaches `sides`      *
                          * when the polygon is complete and stops there*/
    float edge_timer;    /* seconds left until the next side appears    */
    float eps;           /* drawing speed, in sides per second; same    *
                          * for both turtles in this demo               */
    bool  done;          /* true once the polygon is closed             */
} Pen;

/*
 * One whole turtle: the shape it's drawing, how far along it is, and what
 * colour it draws in. The colour lives here (rather than inside the shape
 * or the progress) because it's purely about how things look on screen.
 */
typedef struct {
    Polygon poly;        /* the shape it's drawing                      */
    Pen     pen;         /* how far along the drawing is                */
    int     cpair;       /* which colour to draw it in                  */
} Turtle;

/* Where the centre of a turtle goes: left half (half == 0) or right. */
static inline float half_screen_centre_x(int cols, int half)
{
    return (half == 0) ? (float)cols * HALF_LEFT_X_FRAC
                       : (float)cols * HALF_RIGHT_X_FRAC;
}

/* Pick a size that fits the half it's in, taking the tighter of the width
 * and height limits and shrinking a touch for breathing room. */
static inline float fit_polygon_radius(int cols, int rows)
{
    float max_r_x = (float)cols       * POLY_MAX_R_X_FRAC;
    float max_r_y = (float)(rows - 4) * POLY_MAX_R_Y_FRAC / ASPECT;
    return fminf(max_r_x, max_r_y) * POLY_R_FIT_FRAC;
}

/* Place a turtle and reset it to the start of a fresh polygon. */
static void turtle_init(Turtle *t, int cols, int rows,
                        int half, int sides, int cpair, float eps)
{
    /* the shape */
    t->poly.cx          = half_screen_centre_x(cols, half);
    t->poly.cy          = (float)(rows - 2) * 0.5f + 1.5f;
    t->poly.radius      = fit_polygon_radius(cols, rows);
    t->poly.sides       = sides;
    t->poly.start_angle = -(float)M_PI / 2.0f;   /* first corner straight up */

    /* back to the start, nothing drawn yet */
    t->pen.edge       = 0;
    t->pen.eps        = eps;
    t->pen.edge_timer = 1.0f / eps;
    t->pen.done       = false;

    t->cpair          = cpair;
}

/* Move time forward by dt seconds, finishing more sides as the countdown
 * runs out. The while loop handles a big time gap by finishing several
 * sides at once, so the drawing keeps pace even after a hitch. */
static void turtle_tick(Turtle *t, float dt)
{
    Pen *p = &t->pen;
    if (p->done) return;
    p->edge_timer -= dt;
    while (p->edge_timer <= 0.0f) {
        p->edge++;
        if (p->edge >= t->poly.sides) {
            p->done = true;
            p->edge = t->poly.sides;   /* stop here; the shape is closed */
            break;
        }
        p->edge_timer += 1.0f / p->eps;
    }
}

/* Work out where corner i lands on screen. The corners are spread evenly
 * around a circle; we squash the up/down direction so it isn't a tall oval.
 * i wraps past the last corner back to the first, which closes the shape. */
static void poly_vertex(const Turtle *t, int i, float *vx, float *vy)
{
    const Polygon *g = &t->poly;
    float a = g->start_angle + (float)i * 2.0f * (float)M_PI / (float)g->sides;
    *vx = g->cx + g->radius * cosf(a);
    *vy = g->cy + g->radius * sinf(a) * ASPECT;
}

/* How many dots it takes to draw a line with no gaps: one per cell along
 * whichever direction (across or down) is longer. At least one, so a line
 * of zero length still marks its start. */
static inline int dda_step_count(float dx, float dy)
{
    int steps = (int)(fabsf(dx) > fabsf(dy) ? fabsf(dx) : fabsf(dy));
    return steps < 1 ? 1 : steps;
}

/* Draw one dot of a line, but only if it lands on the canvas — the top row
 * (status bar) and bottom row (key hints) are left clear. */
static inline void dda_stamp_cell(WINDOW *w, float x, float y, char ch,
                                  chtype attr, int cols, int rows)
{
    int col = (int)roundf(x);
    int row = (int)roundf(y);
    if (col < 0 || col >= cols || row < 1 || row >= rows - 1) return;
    wattron (w, attr);
    mvwaddch(w, row, col, (chtype)(unsigned char)ch);
    wattroff(w, attr);
}

/* Draw a straight line between two points by stepping evenly from one end
 * to the other and marking each cell along the way. The line character is
 * picked once from the overall direction so the whole edge looks like one
 * clean stroke. */
static void put_seg(WINDOW *w, float x0, float y0, float x1, float y1,
                    chtype attr, int cols, int rows)
{
    float dx    = x1 - x0;
    float dy    = y1 - y0;
    int   steps = dda_step_count(dx, dy);
    char  ch    = angle_char(atan2f(dy, dx));

    for (int i = 0; i <= steps; i++) {
        float t = (float)i / (float)steps;
        dda_stamp_cell(w, x0 + dx * t, y0 + dy * t,
                       ch, attr, cols, rows);
    }
}

/* Draw every side the turtle has finished so far, in its own colour. */
static void paint_completed_edges(const Turtle *t, WINDOW *w,
                                  int cols, int rows)
{
    chtype attr = (chtype)(COLOR_PAIR(t->cpair) | A_BOLD);
    for (int e = 0; e < t->pen.edge; e++) {
        float x0, y0, x1, y1;
        poly_vertex(t, e,     &x0, &y0);
        poly_vertex(t, e + 1, &x1, &y1);
        put_seg(w, x0, y0, x1, y1, attr, cols, rows);
    }
}

/* Draw the '@' marking where the pen is right now. Hidden once the shape is
 * finished, so the completed polygon looks settled. */
static void paint_pen_head(const Turtle *t, WINDOW *w, int cols, int rows)
{
    if (t->pen.done) return;

    float hx, hy;
    poly_vertex(t, t->pen.edge, &hx, &hy);
    int ix = (int)roundf(hx);
    int iy = (int)roundf(hy);
    if (ix < 0 || ix >= cols || iy < 1 || iy >= rows - 1) return;

    wattron (w, COLOR_PAIR(PAIR_HEAD) | A_BOLD);
    mvwaddch(w, iy, ix, '@');
    wattroff(w, COLOR_PAIR(PAIR_HEAD) | A_BOLD);
}

/* Draw one turtle: its finished sides, then the pen marker on top. */
static void turtle_draw(const Turtle *t, WINDOW *w, int cols, int rows)
{
    paint_completed_edges(t, w, cols, rows);
    paint_pen_head       (t, w, cols, rows);
}

/* ── §6 scene ── */

/* The current size of the terminal window. Everything that places things on
 * screen reads from here; it's refreshed when the window is resized and
 * otherwise stays put for the whole frame. */
typedef struct {
    int cols;            /* width, in characters                     */
    int rows;            /* height, in characters                    */
} Screen;

/*
 * The clock state the main loop keeps between frames. We run the drawing on
 * its own steady beat, separate from how fast the screen actually refreshes,
 * so the polygons draw at the same pace on a fast or a slow machine. The
 * fps shown in the corner is averaged over half a second so it doesn't
 * jitter. (This separate-beat trick is Fiedler's "Fix Your Timestep!".)
 */
typedef struct {
    int64_t frame_time;   /* when the last frame started; used to       *
                           * measure how much time has passed           */
    int64_t sim_accum;    /* leftover time waiting to be turned into     *
                           * drawing steps                               */
    int64_t fps_accum;    /* time piled up since the fps number was      *
                           * last refreshed                              */
    int     frame_count;  /* frames counted in that pile                 */
    double  fps_display;   /* the fps number currently shown             */
} FrameTimer;

/*
 * Everything the program needs to keep around in one place: both turtles,
 * the shared drawing speed, the little countdown between rounds, the pause
 * flag, the window size, the loop clock, the drawing-beat rate, and a couple
 * of flags the signal handlers flip.
 */
typedef struct Scene_ {
    /* the two turtles and their shared state */
    Turtle tA;            /* left turtle (cyan)                       */
    Turtle tB;            /* right turtle (magenta)                   */
    float  eps;           /* drawing speed for both, in sides/second; *
                           * the + / - keys change it                 */
    float  reset_timer;   /* once both are done, counts up to the     *
                           * pause length, then both start over with  *
                           * one more side                            */
    bool   paused;        /* spacebar: freeze the action (the screen  *
                           * still refreshes so you can study a frame)*/

    /* timing */
    Screen     screen;    /* current window size                      */
    FrameTimer timer;     /* the loop clock                           */
    int        sim_fps;   /* how many drawing beats per second; the   *
                           * [ and ] keys change it                   */

    /* set by signal handlers */
    volatile sig_atomic_t running;     /* cleared to quit             */
    volatile sig_atomic_t need_resize; /* set when the window resizes */
} Scene;

static void scene_init(Scene *s)
{
    s->eps         = EPS_DEFAULT;
    s->reset_timer = 0.0f;
    s->paused      = false;
    turtle_init(&s->tA, s->screen.cols, s->screen.rows,
                0, 3, PAIR_TURTLE_A, s->eps);
    turtle_init(&s->tB, s->screen.cols, s->screen.rows,
                1, 5, PAIR_TURTLE_B, s->eps);
}

/* Start a fresh round: both turtles draw a polygon with one more side than
 * last time, wrapping from 12 back to 3. */
static void scene_cycle_polygons(Scene *s)
{
    int na = (s->tA.poly.sides < SIDES_MAX) ? s->tA.poly.sides + 1 : SIDES_MIN;
    int nb = (s->tB.poly.sides < SIDES_MAX) ? s->tB.poly.sides + 1 : SIDES_MIN;
    turtle_init(&s->tA, s->screen.cols, s->screen.rows,
                0, na, PAIR_TURTLE_A, s->eps);
    turtle_init(&s->tB, s->screen.cols, s->screen.rows,
                1, nb, PAIR_TURTLE_B, s->eps);
    s->reset_timer = 0.0f;
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    turtle_tick(&s->tA, dt);
    turtle_tick(&s->tB, dt);

    if (s->tA.pen.done && s->tB.pen.done) {
        s->reset_timer += dt;
        if (s->reset_timer >= RESET_DELAY)
            scene_cycle_polygons(s);
    }
}

/* The name for each polygon, looked up by side count. */
static const char *const POLY_NAMES[] = {
    "", "", "",
    "Triangle", "Square",      "Pentagon",   "Hexagon",
    "Heptagon", "Octagon",     "Nonagon",    "Decagon",
    "Undecagon","Dodecagon"
};

static const char *poly_name(int sides)
{
    if (sides >= SIDES_MIN && sides <= SIDES_MAX) return POLY_NAMES[sides];
    return "Polygon";
}

/* The line down the middle that splits the screen into two halves. */
static void paint_half_divider(WINDOW *w, int cols, int rows)
{
    wattron(w, COLOR_PAIR(PAIR_DIVIDER) | A_DIM);
    for (int r = 1; r < rows - 1; r++)
        mvwaddch(w, r, cols / 2, '|');
    wattroff(w, COLOR_PAIR(PAIR_DIVIDER) | A_DIM);
}

/* The name label above a turtle, like "Triangle (3)". */
static void paint_poly_label(WINDOW *w, const Turtle *t, int cx, int cpair)
{
    char lab[32];
    snprintf(lab, sizeof lab, "%s (%d)",
             poly_name(t->poly.sides), t->poly.sides);
    wattron (w, COLOR_PAIR(cpair) | A_BOLD);
    mvwprintw(w, 1, cx - (int)strlen(lab) / 2, "%s", lab);
    wattroff(w, COLOR_PAIR(cpair) | A_BOLD);
}

/* The "DONE — next in Ns" message shown while both shapes are finished and
 * the countdown to the next round runs. */
static void paint_done_banner(WINDOW *w, const Scene *s, int cols, int rows)
{
    if (!(s->tA.pen.done && s->tB.pen.done)) return;

    int  sec_left = (int)(RESET_DELAY - s->reset_timer) + 1;
    char msg[48];
    snprintf(msg, sizeof msg, "DONE — next in %ds", sec_left);

    int mx = (cols - (int)strlen(msg)) / 2;
    if (mx < 0) mx = 0;

    wattron (w, COLOR_PAIR(PAIR_DONE) | A_BOLD);
    mvwprintw(w, rows / 2, mx, "%s", msg);
    wattroff(w, COLOR_PAIR(PAIR_DONE) | A_BOLD);
}

/* Draw the whole picture (but not the status/hint bars): the divider first,
 * then the turtles, then the labels and any banner on top so text stays
 * readable. */
static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows,
                       float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;

    paint_half_divider(w, cols, rows);

    turtle_draw(&s->tA, w, cols, rows);
    turtle_draw(&s->tB, w, cols, rows);

    paint_poly_label(w, &s->tA, cols / 4,     PAIR_TURTLE_A);
    paint_poly_label(w, &s->tB, 3 * cols / 4, PAIR_TURTLE_B);

    paint_done_banner(w, s, cols, rows);
}

/* ── §7 screen ── */

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

static void screen_free(Screen *s)
{
    (void)s;
    endwin();
}

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* One turtle's little progress readout, like "A:3-gon > 2/3": turtle A, a
 * 3-sided shape, 2 of 3 sides drawn. The '>' turns into '*' when it's done. */
static int hud_write_turtle_status(char *dst, size_t cap,
                                   char letter, const Turtle *t)
{
    return snprintf(dst, cap, " %c:%d-gon %s %d/%d ",
                    letter,
                    t->poly.sides,
                    t->pen.done ? "*"  : ">",
                    t->pen.edge,
                    t->poly.sides);
}

/* Build the top status line: each turtle's progress first, then the speed
 * and timing numbers. */
static void hud_format_status(const Scene *sc, double fps, int sim_fps,
                              char *buf, size_t n)
{
    char a_buf[32], b_buf[32];
    hud_write_turtle_status(a_buf, sizeof a_buf, 'A', &sc->tA);
    hud_write_turtle_status(b_buf, sizeof b_buf, 'B', &sc->tB);

    snprintf(buf, n,
             "%s%s %.1f fps  sim:%d Hz  %.2f eps  %s ",
             a_buf, b_buf,
             fps, sim_fps, (double)sc->eps,
             sc->paused ? "PAUSED " : "drawing");
}

/* Show the status line, pinned to the right of the top row. */
static void hud_paint_status(const char *buf, int cols)
{
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* The bottom row listing what every key does. */
static void hud_paint_hint(int rows)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  r:reset  "
             "a/z:A±sides  s/x:B±sides  +/-:speed  [/]:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    char buf[160];
    hud_format_status(sc, fps, sim_fps, buf, sizeof buf);
    hud_paint_status (buf, s->cols);
    hud_paint_hint   (s->rows);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §8 app ── */

/* The one and only Scene. It's a global so the signal handlers can reach in
 * and flip its flags — handlers don't get to take arguments. */
static Scene g_scene;

static void on_exit_signal(int sig)   { (void)sig; g_scene.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_scene.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* After the window is resized, learn the new size and re-place both turtles
 * to fit it. Their side counts and colours stay the same. */
static void scene_handle_resize(Scene *s)
{
    screen_resize(&s->screen);
    int sa = s->tA.poly.sides;
    int sb = s->tB.poly.sides;
    turtle_init(&s->tA, s->screen.cols, s->screen.rows,
                0, sa, PAIR_TURTLE_A, s->eps);
    turtle_init(&s->tB, s->screen.cols, s->screen.rows,
                1, sb, PAIR_TURTLE_B, s->eps);
    s->reset_timer    = 0.0f;
    s->need_resize    = 0;
    s->timer.frame_time = clock_ns();
    s->timer.sim_accum  = 0;
}

static void key_pause_toggle(Scene *s) { s->paused = !s->paused; }

/* 'r': start both turtles over with the same number of sides. */
static void key_reset_both(Scene *s)
{
    int sa = s->tA.poly.sides, sb = s->tB.poly.sides;
    turtle_init(&s->tA, s->screen.cols, s->screen.rows,
                0, sa, PAIR_TURTLE_A, s->eps);
    turtle_init(&s->tB, s->screen.cols, s->screen.rows,
                1, sb, PAIR_TURTLE_B, s->eps);
    s->reset_timer = 0.0f;
}

/* Give one turtle one more or one fewer side, wrapping between 3 and 12. */
static void key_change_sides(Scene *s, Turtle *t, int delta,
                             int half, int cpair)
{
    int n = t->poly.sides + delta;
    if      (n > SIDES_MAX) n = SIDES_MIN;
    else if (n < SIDES_MIN) n = SIDES_MAX;
    turtle_init(t, s->screen.cols, s->screen.rows,
                half, n, cpair, s->eps);
    s->reset_timer = 0.0f;
}

/* '+' / '-': speed the drawing up or slow it down, for both turtles. */
static void key_speed_scale(Scene *s, float factor)
{
    s->eps *= factor;
    if (s->eps > EPS_MAX) s->eps = EPS_MAX;
    if (s->eps < EPS_MIN) s->eps = EPS_MIN;
    s->tA.pen.eps = s->tB.pen.eps = s->eps;
}

/* '[' / ']': change how many drawing beats happen per second. */
static void key_sim_fps_nudge(Scene *s, int delta)
{
    s->sim_fps += delta;
    if (s->sim_fps > SIM_FPS_MAX) s->sim_fps = SIM_FPS_MAX;
    if (s->sim_fps < SIM_FPS_MIN) s->sim_fps = SIM_FPS_MIN;
}

/* Act on one keystroke; returns false if it was a quit key. */
static bool scene_handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */:  return false;
    case ' ':            key_pause_toggle(s);                              break;
    case 'r': case 'R':  key_reset_both(s);                                break;
    case 'a': case 'A':  key_change_sides(s, &s->tA, +1, 0, PAIR_TURTLE_A); break;
    case 'z': case 'Z':  key_change_sides(s, &s->tA, -1, 0, PAIR_TURTLE_A); break;
    case 's': case 'S':  key_change_sides(s, &s->tB, +1, 1, PAIR_TURTLE_B); break;
    case 'x': case 'X':  key_change_sides(s, &s->tB, -1, 1, PAIR_TURTLE_B); break;
    case '=': case '+':  key_speed_scale(s, EPS_SCALE_FACTOR);             break;
    case '-':            key_speed_scale(s, 1.0f / EPS_SCALE_FACTOR);      break;
    case ']':            key_sim_fps_nudge(s, +SIM_FPS_STEP);              break;
    case '[':            key_sim_fps_nudge(s, -SIM_FPS_STEP);              break;
    default: break;
    }
    return true;
}

/* ── §8.1 setup and main-loop helpers ── */

/* Get everything ready once before the loop runs: signal handlers, default
 * settings, ncurses, both turtles, and the loop clock. */
static void scene_setup(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    g_scene.running = 1;
    g_scene.sim_fps = SIM_FPS_DEFAULT;

    /* ncurses must come up first so we know the window size before placing
     * the turtles. */
    screen_init(&g_scene.screen);
    scene_init (&g_scene);

    g_scene.timer.frame_time  = clock_ns();
    g_scene.timer.fps_display = 0.0;
}

/* How much real time passed since the last frame, capped so a stall can't
 * report a huge jump. */
static int64_t frame_measure_dt(FrameTimer *tm)
{
    int64_t now = clock_ns();
    int64_t dt  = now - tm->frame_time;
    tm->frame_time = now;
    if (dt > DT_CAP_NS) dt = DT_CAP_NS;
    return dt;
}

/* Turn elapsed time into a whole number of steady drawing beats, so the
 * drawing keeps the same pace no matter the frame rate. Returns the length
 * of one beat in seconds. */
static float scene_advance_sim_burst(Scene *s, int64_t dt)
{
    int64_t tick_ns = TICK_NS(s->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

    s->timer.sim_accum += dt;
    while (s->timer.sim_accum >= tick_ns) {
        scene_tick(s, dt_sec);
        s->timer.sim_accum -= tick_ns;
    }
    return dt_sec;
}

/* Refresh the fps number shown in the corner, about twice a second so it
 * reads steadily instead of flickering. */
static void frame_tick_fps_window(FrameTimer *tm, int64_t dt)
{
    tm->frame_count++;
    tm->fps_accum += dt;
    if (tm->fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
        tm->fps_display = (double)tm->frame_count
                        / ((double)tm->fps_accum / (double)NS_PER_SEC);
        tm->frame_count = 0;
        tm->fps_accum   = 0;
    }
}

/* Wait out the rest of the frame so we draw at a steady 60 per second. */
static void frame_cap_to_render_fps(int64_t frame_start, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_start + dt;
    clock_sleep_ns(RENDER_FRAME_NS - elapsed);
}

/* Read one pending keypress (if any) and act on it; false means quit. */
static bool scene_drain_one_key(Scene *s)
{
    int ch = getch();
    if (ch == ERR) return true;
    return scene_handle_key(s, ch);
}

/* ── §8.2 main ── */

int main(void)
{
    scene_setup();

    while (g_scene.running) {
        if (g_scene.need_resize)
            scene_handle_resize(&g_scene);

        int64_t frame_start = g_scene.timer.frame_time;
        int64_t dt          = frame_measure_dt(&g_scene.timer);

        /* advance the drawing, then note how far into the next beat we are */
        float   dt_sec = scene_advance_sim_burst(&g_scene, dt);
        float   alpha  = (float)g_scene.timer.sim_accum
                       / (float)TICK_NS(g_scene.sim_fps);

        frame_tick_fps_window(&g_scene.timer, dt);

        /* wait before drawing so the frame rate stays even */
        frame_cap_to_render_fps(frame_start, dt);

        screen_draw(&g_scene.screen, &g_scene,
                    g_scene.timer.fps_display, g_scene.sim_fps,
                    alpha, dt_sec);
        screen_present();

        if (!scene_drain_one_key(&g_scene))
            g_scene.running = 0;
    }

    screen_free(&g_scene.screen);
    return 0;
}
