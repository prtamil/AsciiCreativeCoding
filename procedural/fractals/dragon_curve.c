/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * dragon_curve.c — the Heighway dragon curve, drawn one segment at a time.
 *
 * Fold a strip of paper in half, again and again, then unfold it so every
 * crease is a right angle: trace the edge and you get this curve. We grow it
 * by computing that left/right turn sequence, walking it into a path, and
 * animating the path onto the terminal.
 *
 * Heighway dragon: Davis & Knuth, "Number Representations and Dragon Curves,"
 *   J. Recreational Mathematics 3 (1970), 66-81 & 133-149.
 * Sister files: l_system.c (same dragon from a rewrite grammar instead of the
 *   fold recipe) and koch.c (another turtle-traced curve, same draw pipeline).
 */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define N_GEN_MIN    4
#define N_GEN_MAX   13
#define N_GEN_INIT  12

/* Each fold roughly doubles the curve. These size the arrays for the biggest
 * generation we allow, so nothing ever has to grow at runtime. */
#define MAX_TURNS   8191       /* turns at gen 13 */
#define MAX_POINTS  8192       /* points at gen 13 */

#define SPEED_MIN      1
#define SPEED_MAX   1024
#define SPEED_DEFAULT  8

#define NS_PER_SEC  1000000000LL
#define RENDER_FPS  30
#define RENDER_NS   (NS_PER_SEC / RENDER_FPS)

/* A terminal character is about twice as tall as it is wide, so we stretch the
 * curve horizontally by this much to keep it from looking squashed. */
#define ASPECT_R   2.0f

/* Rows and columns we leave empty around the edges so the curve never paints
 * over the heads-up display. */
enum {
    MARGIN_TOP    = 2,    /* top rows hold the status text */
    MARGIN_BOTTOM = 2,    /* bottom rows hold the key list */
    MARGIN_SIDE   = 1,    /* a little breathing room left and right */
};

/*
 * The colours we can draw in. ncurses works with numbered "pairs" (a chosen
 * foreground and background), and these names ARE those numbers, so picking a
 * colour is just COLOR_PAIR(CP_...).
 *
 * The seven curve colours tint a segment by its age — which fold first created
 * it. Old parts come out cool (blue), new parts warm (white), which makes the
 * nested folding pattern visible. seg_color (§6) does the picking.
 */
enum {
    CP_G1 = 1,   /* oldest folds — dark blue */
    CP_G2,       /*              — blue       */
    CP_G3,       /*              — cyan       */
    CP_G4,       /*              — green      */
    CP_G5,       /*              — yellow     */
    CP_G6,       /*              — orange     */
    CP_G7,       /* newest folds — white      */
    CP_HUD,      /* status text — yellow */
    CP_HINT,     /* key list    — cyan   */
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&ts, NULL);
}

/* Sleep off whatever time this frame had left over, so frames come out at a
 * steady rate no matter how long the drawing took. */
static void frame_pace(long long frame_start)
{
    clock_sleep_ns(RENDER_NS - (clock_ns() - frame_start));
}

/* ===================================================================== */
/* §3  geometry — the integer grid the curve is drawn on                  */
/* ===================================================================== */

/*
 * One spot on the grid. We use whole numbers, not decimals, because the curve
 * only ever lands on exact grid corners and moves exactly one cell at a time —
 * so there's no rounding to worry about. The same type names both a corner of
 * the curve and, after Fit (§7) scales it, a cell on the screen.
 */
typedef struct {
    int x;   /* column, counted from the left */
    int y;   /* row, counted from the top (screens number rows downward) */
} Point;

/*
 * The four directions the pen can face. The order matters: going clockwise
 * (east, south, west, north, back to east) is just adding 1 each time. That
 * makes a right turn "add 1" and a left turn "subtract 1" — turning becomes
 * plain arithmetic instead of a lookup table.
 */
typedef enum { EAST, SOUTH, WEST, NORTH } Heading;

/* How far x and y move for one step in each direction. Looked up by heading so
 * stepping forward is a single line. */
static const Point HEADING_STEP[4] = {
    [EAST]  = {  1,  0 },   /* right */
    [SOUTH] = {  0,  1 },   /* down  */
    [WEST]  = { -1,  0 },   /* left  */
    [NORTH] = {  0, -1 },   /* up    */
};

/*
 * The smallest box that holds the whole curve. The curve gets bigger with each
 * generation but the terminal stays the same size, so we measure exactly how
 * far it reaches in every direction; Fit (§7) uses that to scale and centre it.
 * We fill it in as we trace: start it at the first point, then widen it to
 * include each new point.
 */
typedef struct {
    int x_min, x_max;   /* leftmost and rightmost columns the curve touches */
    int y_min, y_max;   /* topmost and bottommost rows the curve touches    */
} BBox;

static void bbox_reset(BBox *b, Point p)
{
    b->x_min = b->x_max = p.x;
    b->y_min = b->y_max = p.y;
}

static void bbox_include(BBox *b, Point p)
{
    if (p.x < b->x_min) b->x_min = p.x;
    if (p.x > b->x_max) b->x_max = p.x;
    if (p.y < b->y_min) b->y_min = p.y;
    if (p.y > b->y_max) b->y_max = p.y;
}

static int bbox_width (const BBox *b) { return b->x_max - b->x_min + 1; }
static int bbox_height(const BBox *b) { return b->y_max - b->y_min + 1; }

/* ===================================================================== */
/* §4  turtle — the alphabet, and the pen that walks it                   */
/* ===================================================================== */

/*
 * The whole curve is just a long list of these: at each step you either turn
 * left or turn right. We give left the value 0 and right the value 1 on
 * purpose — flipping one into the other (which the fold recipe in §5 does
 * constantly) is then a trivial swap, done by turn_flip below.
 */
typedef enum { TURN_LEFT, TURN_RIGHT } Turn;

static Turn turn_flip(Turn t) { return t == TURN_RIGHT ? TURN_LEFT : TURN_RIGHT; }

/*
 * An imaginary pen on the grid: where it is and which way it's pointing. The
 * turn list in §5 is just left/right letters — abstract. The pen turns that
 * list into an actual shape: read a letter, turn, take a step, and leave a
 * trail of points behind. (This pen-and-paper idea is called turtle graphics.)
 */
typedef struct {
    Point   pos;       /* where the pen is right now */
    Heading heading;   /* the way it's about to step  */
} Turtle;

/* Move one cell the way the pen is facing; hand back where it landed. */
static Point turtle_step(Turtle *t)
{
    t->pos.x += HEADING_STEP[t->heading].x;
    t->pos.y += HEADING_STEP[t->heading].y;
    return t->pos;
}

/* Turn the pen a quarter turn: right adds one direction, left subtracts one
 * (written as +3 since the four directions wrap around). */
static void turtle_turn(Turtle *t, Turn turn)
{
    int delta = (turn == TURN_RIGHT) ? 1 : 3;
    t->heading = (Heading)((t->heading + delta) % 4);
}

/* ===================================================================== */
/* §5  curve — the DragonCurve itself                                     */
/* ===================================================================== */

/*
 * The dragon curve itself — the heart of the program. It's built in two stages:
 * first the list of left/right turns (the recipe), then the actual path you get
 * by following that recipe (the shape). We keep the recipe, the shape, and the
 * bounding box together because everything else needs one of them — drawing
 * wants the path, colouring wants the turn count, scaling wants the box — and
 * all three come from a single generation number.
 *
 * Each fold roughly doubles things: at generation `gen` there are 2^gen - 1
 * turns and 2^gen points, and one drawable segment per turn.
 */
typedef struct {
    int   gen;                   /* how many times it's been folded, 4..13 */
    int   n_turns;               /* how many turns are filled in = segment count */
    Turn  turns[MAX_TURNS];      /* the left/right recipe (stage 1) */
    int   n_points;              /* how many points = n_turns + 1 */
    Point path[MAX_POINTS];      /* the traced-out corners, in grid units (stage 2) */
    BBox  bounds;                /* how far the path reaches, used to scale it to screen */
} DragonCurve;

/*
 * Stage 1 — build the list of turns by "folding" it. Start with a single right
 * turn. Each generation: keep what you have, drop one right turn in the middle
 * (the new crease), then add a mirror-image copy of the first half with every
 * turn flipped. That mirror-and-flip is exactly what happens to the creases
 * when you fold a real paper strip in half again.
 */
static void dragon_build_turns(DragonCurve *d, int gen)
{
    d->turns[0] = TURN_RIGHT;
    d->n_turns  = 1;

    for (int g = 2; g <= gen; g++) {
        int half = d->n_turns;
        d->turns[half] = TURN_RIGHT;                          /* the new crease in the middle */
        for (int i = 0; i < half; i++)                        /* mirror the first half, flipped */
            d->turns[half + 1 + i] = turn_flip(d->turns[half - 1 - i]);
        d->n_turns = half * 2 + 1;
    }
}

/*
 * Stage 2 — walk the recipe into an actual path. Put the pen at the origin
 * facing right, then for each turn: step forward (recording where it lands and
 * widening the bounding box), then turn ready for the next step.
 */
static void dragon_trace_path(DragonCurve *d)
{
    Turtle t = { .pos = { 0, 0 }, .heading = EAST };

    d->path[0] = t.pos;
    bbox_reset(&d->bounds, t.pos);

    for (int i = 0; i < d->n_turns; i++) {
        d->path[i + 1] = turtle_step(&t);
        bbox_include(&d->bounds, d->path[i + 1]);
        turtle_turn(&t, d->turns[i]);
    }
    d->n_points = d->n_turns + 1;
}

/* Build the whole curve at a given number of folds, kept inside the allowed range. */
static void dragon_build(DragonCurve *d, int gen)
{
    if (gen < N_GEN_MIN) gen = N_GEN_MIN;
    if (gen > N_GEN_MAX) gen = N_GEN_MAX;
    d->gen = gen;
    dragon_build_turns(d, gen);
    dragon_trace_path(d);
}

/* ===================================================================== */
/* §6  color — generation depth → colour band                            */
/* ===================================================================== */

/*
 * Which fold first created this segment. The first segment came from fold 1,
 * the next two from fold 2, the next four from fold 3, and so on — each fold
 * adds twice as many as the last, so we count how many times the segment number
 * can be halved.
 */
static int seg_generation(int seg)
{
    int g = 1, n = seg + 1;
    while (n > 1) { n >>= 1; g++; }
    return g;
}

/* Pick a colour for a segment based on its age: spread the oldest-to-newest
 * folds across the seven colours, cool for old, warm for new. */
static int seg_color(int seg, int gen)
{
    int depth = seg_generation(seg);              /* which fold this segment belongs to */
    int steps = CP_G7 - CP_G1;                    /* how many colour steps we have (6) */
    int span  = gen > 1 ? gen - 1 : 1;            /* how many folds to spread across   */
    int pair  = CP_G1 + (depth - 1) * steps / span;
    if (pair < CP_G1) pair = CP_G1;
    if (pair > CP_G7) pair = CP_G7;
    return pair;
}

/* ===================================================================== */
/* §7  fit — map grid points onto terminal cells                          */
/* ===================================================================== */

/*
 * The recipe for placing the curve on the screen: how much to scale it (one
 * factor for each axis) and where to shift it. fit_compute works it out fresh
 * every frame from the curve's size and the window's size, so the picture stays
 * centred and as large as will fit even as the curve grows or the window
 * changes; fit_apply runs a single point through that recipe.
 *
 * There are two scale factors because terminal characters are about twice as
 * tall as wide — without stretching x by ASPECT_R the curve would look squashed.
 */
typedef struct {
    float scale_x, scale_y;   /* how much to grow the curve on each axis (x is stretched) */
    int   off_x, off_y;       /* where the curve's top-left corner lands (this centres it) */
} Fit;

static Fit fit_compute(const BBox *bounds, int cols, int rows)
{
    /* 1. the area we may draw in: the screen minus the reserved HUD edges */
    int draw_cols = cols - 2 * MARGIN_SIDE;
    int draw_rows = rows - (MARGIN_TOP + MARGIN_BOTTOM);
    if (draw_cols < 1) draw_cols = 1;
    if (draw_rows < 1) draw_rows = 1;

    int grid_w = bbox_width(bounds);
    int grid_h = bbox_height(bounds);

    /* 2. pick the biggest scale that still fits both ways. Width is checked
     *    against the stretched scale since each grid unit is drawn wider; keep
     *    whichever direction runs out of room first. */
    float fit_by_rows = (float)draw_rows / (float)grid_h;
    float fit_by_cols = (float)draw_cols / (float)grid_w;
    float unit = fit_by_rows;
    if (unit * ASPECT_R > fit_by_cols)            /* too wide — shrink to fit the columns */
        unit = fit_by_cols / ASPECT_R;

    /* 3. centre the scaled curve in the drawable area */
    Fit f;
    f.scale_y = unit;
    f.scale_x = unit * ASPECT_R;
    f.off_x = MARGIN_SIDE + (draw_cols - (int)(grid_w * f.scale_x)) / 2;
    f.off_y = MARGIN_TOP  + (draw_rows - (int)(grid_h * f.scale_y)) / 2;
    return f;
}

/* Turn one grid point into the screen cell it should be drawn at. */
static Point fit_apply(const Fit *f, const BBox *bounds, Point grid)
{
    return (Point){
        .x = (int)((grid.x - bounds->x_min) * f->scale_x) + f->off_x,
        .y = (int)((grid.y - bounds->y_min) * f->scale_y) + f->off_y,
    };
}

/* ===================================================================== */
/* §8  reveal — the segment-by-segment animation                          */
/* ===================================================================== */

/*
 * Tracks how much of the curve has been drawn so far. Kept separate from the
 * curve itself, so restarting or changing speed never has to rebuild the
 * (slow-to-compute) shape. The count climbs each frame until the whole curve is
 * showing, then it just holds there — the animation doesn't loop. Restarting
 * ('r') only sets the count back to zero and reuses the same curve.
 */
typedef struct {
    int  drawn;    /* how many segments are showing so far (0 up to the total) */
    int  speed;    /* how many more to add each frame (SPEED_MIN .. SPEED_MAX) */
    bool paused;   /* true means hold still */
} Reveal;

static void reveal_reset (Reveal *r)            { r->drawn = 0; }
static void reveal_faster(Reveal *r)            { r->speed *= 2; if (r->speed > SPEED_MAX) r->speed = SPEED_MAX; }
static void reveal_slower(Reveal *r)            { r->speed /= 2; if (r->speed < SPEED_MIN) r->speed = SPEED_MIN; }

static void reveal_tick(Reveal *r, int total)
{
    if (r->paused) return;
    r->drawn += r->speed;
    if (r->drawn > total) r->drawn = total;
}

/* ===================================================================== */
/* §9  scene — the curve plus its animation = the running picture         */
/* ===================================================================== */

/*
 * Everything that's on screen: the curve, plus how much of it has been drawn so
 * far. That's all you need to describe one frame — the curve is the "what" and
 * the reveal is the "how much." (Window size lives in Screen instead, since
 * that's about the terminal, not the picture.) The whole program boils down to
 * advancing a Scene each frame and drawing it.
 */
typedef struct {
    DragonCurve curve;    /* the curve being shown */
    Reveal      reveal;   /* how far the drawing has gotten */
} Scene;

/* Build a fresh curve at the given number of folds and start its reveal over. */
static void scene_load(Scene *s, int gen)
{
    dragon_build(&s->curve, gen);
    reveal_reset(&s->reveal);
}

static void scene_init(Scene *s, int gen, int speed)
{
    s->reveal.speed  = speed;
    s->reveal.paused = false;
    scene_load(s, gen);
}

static void scene_tick(Scene *s)
{
    reveal_tick(&s->reveal, s->curve.n_turns);
}

/* How many segments are visible right now — the reveal's count, but never more
 * than the curve actually has. Both the drawing code and the HUD ask for this. */
static int scene_shown(const Scene *s)
{
    int total = s->curve.n_turns;
    return s->reveal.drawn < total ? s->reveal.drawn : total;
}

/* Draw one point as a single character, skipping it if it falls in the HUD rows
 * or off the screen. */
static void plot_point(Point cell, int color, int cols, int rows)
{
    if (cell.y >= MARGIN_TOP && cell.y < rows - 1 && cell.x >= 0 && cell.x < cols) {
        attron(COLOR_PAIR(color) | A_BOLD);
        mvaddch(cell.y, cell.x, (chtype)'#');
        attroff(COLOR_PAIR(color) | A_BOLD);
    }
}

/*
 * Paint the part of the curve revealed so far. Work out the placement recipe
 * for the current size, then for each visible segment draw both of its ends.
 * Neighbouring segments share an end, so drawing ends covers the whole path.
 */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const DragonCurve *d = &s->curve;
    int shown = scene_shown(s);

    Fit fit = fit_compute(&d->bounds, cols, rows);

    for (int i = 0; i < shown; i++) {
        int color = seg_color(i, d->gen);
        plot_point(fit_apply(&fit, &d->bounds, d->path[i]),     color, cols, rows);
        plot_point(fit_apply(&fit, &d->bounds, d->path[i + 1]), color, cols, rows);
    }
}

/* ===================================================================== */
/* §10  screen — ncurses lifecycle + HUD                                  */
/* ===================================================================== */

/*
 * The current terminal size in characters. Read at startup and again after
 * every window resize; Fit (§7) uses it to scale the curve and the HUD uses it
 * to place its lines.
 */
typedef struct {
    int cols;   /* width  in characters */
    int rows;   /* height in characters */
} Screen;

/*
 * Set up all nine colours at once — the seven curve colours plus the two HUD
 * ones, each drawn on the terminal's normal background. Takes whichever colour
 * list applies (the rich one or the 8-colour fallback) so this wiring lives in
 * just one place.
 */
static void bind_palette(const int gen_fg[7], int hud_fg, int hint_fg)
{
    for (int i = 0; i < 7; i++)
        init_pair(CP_G1 + i, gen_fg[i], -1);     /* CP_G1..CP_G7, oldest to newest */
    init_pair(CP_HUD,  hud_fg,  -1);
    init_pair(CP_HINT, hint_fg, -1);
}

static void color_init(void)
{
    static const int gen_256[7] = {  26,  27,  51,  46, 226, 208, 231 };
    static const int gen_8[7]   = { COLOR_BLUE,   COLOR_BLUE,  COLOR_CYAN, COLOR_GREEN,
                                    COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE };
    start_color();
    use_default_colors();

    bool truecolor = COLORS >= 256;
    bind_palette(truecolor ? gen_256 : gen_8,
                 truecolor ? 226 : COLOR_YELLOW,
                 truecolor ?  51 : COLOR_CYAN);
}

static void screen_init(Screen *sc)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_resize(Screen *sc)
{
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

/*
 * A playful growth-stage name for each fold count, shown instead of a plain
 * number. The curve grows from a tiny "hatchling" (4 folds) to a full "dragon"
 * (13 folds), so the names give a feel for how grown it is.
 */
static const char *gen_name(int gen)
{
    static const char *k_names[N_GEN_MAX - N_GEN_MIN + 1] = {
        "hatchling",  /* gen 4  */
        "wyrmling",   /* gen 5  */
        "fledgling",  /* gen 6  */
        "juvenile",   /* gen 7  */
        "serpent",    /* gen 8  */
        "wyrm",       /* gen 9  */
        "drake",      /* gen 10 */
        "wyvern",     /* gen 11 */
        "elder",      /* gen 12 */
        "dragon",     /* gen 13 */
    };
    int idx = gen - N_GEN_MIN;
    if (idx < 0) idx = 0;
    if (idx > N_GEN_MAX - N_GEN_MIN) idx = N_GEN_MAX - N_GEN_MIN;
    return k_names[idx];
}

/* Top row, right side: the growth-stage name, where it sits in the range, and
 * whether it's drawing, done, or paused. */
static void hud_data_line(const Screen *sc, const Scene *s)
{
    const DragonCurve *d = &s->curve;
    int shown = scene_shown(s);
    const char *state = s->reveal.paused ? "PAUSED "
                      : (shown >= d->n_turns ? "complete" : "drawing ");
    int stage  = d->gen - N_GEN_MIN + 1;          /* which stage we're on, counting from 1 */
    int stages = N_GEN_MAX - N_GEN_MIN + 1;       /* how many stages there are */

    char buf[80];
    snprintf(buf, sizeof buf, " DragonCurve  %s %d/%d  %s ",
             gen_name(d->gen), stage, stages, state);
    int right_x = sc->cols - (int)strlen(buf);
    if (right_x < 0) right_x = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, right_x, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Second row: how many segments are drawn out of the total, and the speed. */
static void hud_params_line(const Scene *s)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(1, 0, " segs:%d/%d  speed:%d ",
             scene_shown(s), s->curve.n_turns, s->reveal.speed);
    attroff(COLOR_PAIR(CP_HUD));
}

/* Bottom row: the list of keys you can press. */
static void hud_action_line(const Screen *sc)
{
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
        " q:quit  spc:pause  r:restart  +/-:speed  n/p:age ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void hud_draw(const Screen *sc, const Scene *s)
{
    hud_data_line(sc, s);     /* top row    — name, stage, state */
    hud_params_line(s);       /* second row — segments and speed */
    hud_action_line(sc);      /* bottom row — the keys           */
}

static void screen_draw(const Screen *sc, const Scene *s)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);
    hud_draw(sc, s);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §11  app — signals, input, main loop                                   */
/* ===================================================================== */

/*
 * Everything main() owns in one place: the picture and the terminal it's shown
 * on. Bundling them lets the loop pass it all around with a single pointer, and
 * lets the whole thing live in global storage instead of on the stack — handy
 * because the curve's arrays are large.
 */
typedef struct {
    Scene  scene;    /* the curve and its animation */
    Screen screen;   /* current terminal size */
} App;

/* The only globals. When the OS tells us to quit or that the window resized, it
 * runs a handler that can't take arguments — so it just flips a flag here and
 * the main loop checks it. */
static volatile sig_atomic_t g_should_quit   = 0;
static volatile sig_atomic_t g_should_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_should_quit   = 1;
    if (s == SIGWINCH)               g_should_resize = 1;
}

static void cleanup(void) { endwin(); }

static void app_resize(App *a)
{
    g_should_resize = 0;
    screen_resize(&a->screen);
}

/* Turn one keypress into an action. */
static void app_handle_key(App *a, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27:  g_should_quit = 1;                       break;
    case ' ':            a->scene.reveal.paused = !a->scene.reveal.paused; break;
    case 'r': case 'R':  reveal_reset(&a->scene.reveal);                   break;
    case '+': case '=':  reveal_faster(&a->scene.reveal);                  break;
    case '-':            reveal_slower(&a->scene.reveal);                  break;
    case 'n': case 'N':  scene_load(&a->scene, a->scene.curve.gen + 1);    break;  /* fold once more */
    case 'p': case 'P':  scene_load(&a->scene, a->scene.curve.gen - 1);    break;  /* fold once less */
    default: break;
    }
}

static void app_poll_input(App *a)
{
    int ch = getch();
    if (ch != ERR) app_handle_key(a, ch);
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);

    /* static so the large curve arrays inside it stay off the stack */
    static App app;

    screen_init(&app.screen);
    scene_init(&app.scene, N_GEN_INIT, SPEED_DEFAULT);

    /* One pass per frame: handle a resize, read a key, reveal a bit more, draw,
     * and wait out the rest of the frame's time. */
    while (!g_should_quit) {

        if (g_should_resize)
            app_resize(&app);

        long long frame_start = clock_ns();

        app_poll_input(&app);                   /* read a key */
        scene_tick(&app.scene);                 /* reveal a little more */
        screen_draw(&app.screen, &app.scene);   /* draw curve and HUD */
        screen_present();

        frame_pace(frame_start);                /* keep a steady frame rate */
    }
    return 0;
}
