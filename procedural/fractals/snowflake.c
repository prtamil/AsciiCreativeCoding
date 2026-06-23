/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * snowflake.c  —  a snowflake drawn as a fractal.
 *
 * A snow crystal has six arms, and each arm grows smaller branches, which grow
 * smaller branches again — the same shape repeating at every size.  We draw that
 * with one rule that calls itself: draw a line, then sprout two half-size copies
 * of the whole rule off it, angled left and right.  Six arms 60° apart around a
 * centre, and the snowflake's six-way symmetry just appears on its own.
 *
 * The + / - keys change the depth (how many times the rule recurses), which is
 * the only real knob: more depth = a finer, lacier crystal.
 *
 * Sister file koch.c draws the snowflake's OUTLINE as a fractal instead of its arms.
 * Snowflake physics: Nakaya, "Snow Crystals" (1954); Libbrecht, Rep. Prog. Phys.
 * 68 (2005), 855.  Self-similar branching: Mandelbrot, "The Fractal Geometry of
 * Nature" (1982).
 *
 * Build: gcc -std=c11 -O2 -Wall -Wextra snowflake.c -o snowflake -lncurses -lm
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

#define PI  3.14159265358979f

enum {
    DEPTH_MIN     =   1,
    DEPTH_MAX     =   6,
    DEPTH_DEFAULT =   4,

    GRID_ROWS_MAX =  80,
    GRID_COLS_MAX = 300,

    RENDER_FPS    =  30,     /* how often we redraw and check for keys */
    RAMP_LEN      =   6,     /* number of colours in a theme, core to tip */
    N_THEMES      =   6,
};

/* The crystal's shape.  Everything here is in "math space": the centre is at
 * (0,0) and a main arm is 1 unit long.  Cells on screen come later. */
#define N_ARMS         6                  /* a snowflake has six arms        */
#define ARM_LENGTH     1.0f
#define ARM_STEP       (2.0f * PI / N_ARMS)   /* turn between arms = 60°     */
#define BRANCH_ANGLE   (PI / 3.0f)        /* side branches angle off at ±60° */
#define BRANCH_SCALE   0.40f              /* each branch is this fraction of its parent's length */
#define N_BRANCH_PTS   2                  /* how many spots along a line sprout branches */
static const float BRANCH_FRAC[N_BRANCH_PTS] = { 0.40f, 0.72f };   /* where along the line, 0=start 1=tip */

/* How far the crystal reaches from the centre.  The canvas shrinks to fit a
 * circle this big on screen; anything still poking past the edge is clipped. */
#define EXTENT       1.3f
#define MARGIN_CELLS 2

#define ASPECT_R    2.0f       /* terminal cells are about twice as tall as wide; we stretch x to match so the snowflake looks round, not squashed */

/* When a line is much more horizontal than vertical (or the reverse) by this
 * factor, draw it as '-' or '|' instead of a diagonal. */
#define GLYPH_AXIS_RATIO  2.0f

#define NS_PER_SEC  1000000000LL

/*
 * Theme — a run of colours from the crystal's centre to its tips.
 *
 * We colour each line by how deep it is in the fractal, so depth becomes
 * brightness: the big main arms look dark, the fine outer lacework looks
 * bright.  That's what lets your eye tell the layers apart at a glance.
 * Every colour is kept in the brighter half of the palette so even the centre
 * stays visible against a black background (project palette-brightness rule).
 */
typedef struct {
    const char *name;   /* shown in the HUD when you cycle themes with t/T */
    int c [RAMP_LEN];   /* the 256-colour version, ordered centre -> tip */
    int c8[RAMP_LEN];   /* fallback for terminals with only 8 colours, same order */
} Theme;

static const Theme k_themes[N_THEMES] = {
    { "Ice",    {  39,  45,  51, 117, 159, 195 },
        { COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE, COLOR_WHITE } },
    { "Frost",  {  33,  39,  45,  51,  87, 159 },
        { COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE } },
    { "Aurora", {  41,  48,  49,  50,  86, 159 },
        { COLOR_GREEN, COLOR_GREEN, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE } },
    { "Fire",   { 130, 166, 202, 208, 214, 220 },
        { COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW } },
    { "Gold",   { 136, 172, 178, 214, 220, 228 },
        { COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE } },
    { "Mono",   { 245, 248, 250, 252, 254, 231 },
        { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE } },
};

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

/* ncurses colour-pair slots.  Slots 1..RAMP_LEN hold the theme colours (these
 * get reassigned when you switch themes); the two HUD slots never change. */
#define COL_RAMP 1
enum {
    COL_HUD  = 1 + RAMP_LEN,   /* yellow status text */
    COL_HINT,                  /* cyan key hints     */
};

/* Which colour a line gets, given how deep it is (0 = the main arms).  Anything
 * deeper than we have colours for just keeps the brightest tip colour. */
static int level_color(int level)
{
    int idx = level < RAMP_LEN ? level : RAMP_LEN - 1;
    return COL_RAMP + idx;
}

/* Load theme t's colours into the ncurses palette slots. */
static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    bool truecolor = COLORS >= 256;
    for (int i = 0; i < RAMP_LEN; i++)
        init_pair((short)(COL_RAMP + i),
                  (short)(truecolor ? th->c[i] : th->c8[i]), COLOR_BLACK);
}

static void color_init(void)
{
    start_color();
    /* The HUD colours are fixed: yellow status text, cyan key hints */
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);
        init_pair(COL_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    theme_apply(0);   /* start on the first theme (Ice) */
}

/* ===================================================================== */
/* §4  geometry — the math plane the crystal lives in                     */
/* ===================================================================== */

/*
 * Vec2 — a point (or direction) in the math plane: x goes right, y goes up,
 * and the snowflake's centre is at (0,0).  We build the whole crystal here in
 * smooth decimal coordinates and only turn it into screen cells at the last
 * moment (§5).  Staying in floats keeps the angles and midpoints exact no
 * matter how deep the recursion goes.
 */
typedef struct {
    float x;   /* left-right (a main arm is 1.0 long) */
    float y;   /* up-down (bigger y = higher up)      */
} Vec2;

static Vec2 vec_add(Vec2 a, Vec2 b)        { return (Vec2){ a.x + b.x, a.y + b.y }; }
static Vec2 vec_scale(Vec2 v, float s)     { return (Vec2){ v.x * s, v.y * s }; }
static Vec2 vec_from_angle(float radians)  { return (Vec2){ cosf(radians), sinf(radians) }; }

/* Spin a direction by some angle (counter-clockwise). */
static Vec2 vec_rotate(Vec2 v, float radians)
{
    float c = cosf(radians), s = sinf(radians);
    return (Vec2){ v.x * c - v.y * s, v.x * s + v.y * c };
}

/* ===================================================================== */
/* §5  canvas — cell buffer + projection + line stroke                    */
/* ===================================================================== */

/*
 * Pt — a spot on the screen, measured in columns and rows.  It's a separate
 * type from Vec2 on purpose so we can't accidentally mix the math plane up with
 * the screen.  A Vec2 becomes a Pt through canvas_project() below.  We keep
 * fractional precision here and only round to whole cells when actually drawing.
 */
typedef struct {
    float x;   /* column */
    float y;   /* row    */
} Pt;

/*
 * Cell — one character of the finished picture: just its colour and glyph,
 * nothing about the geometry that put it there.  We keep a whole grid of these
 * so the slow drawing happens once and every frame just copies them to screen.
 */
typedef struct {
    uint8_t color;   /* colour slot 1..RAMP_LEN, or 0 for "empty / background" */
    char    glyph;   /* the line character: '-' '|' '/' or '\\' */
} Cell;

/*
 * Canvas — the finished picture plus the numbers used to draw it.  The build
 * step (§6) fills this in once; the renderer (§8) only ever reads it.
 * scale_x/scale_y and (cx, cy) are what place a centred circle of the crystal
 * on screen, with x stretched so it doesn't come out squashed.
 */
typedef struct {
    /* The picture grid, [row][col].  Sized big enough for the largest terminal
       up front so we never allocate while running; only the top-left rows x cols
       corner is actually used at any moment. */
    Cell  cell[GRID_ROWS_MAX][GRID_COLS_MAX];
    int   rows, cols;         /* the part actually in use, capped to the grid size */
    float scale_x, scale_y;   /* how many cells per math unit (x is ASPECT_R wider) */
    int   cx, cy;             /* the cell the centre of the crystal sits on */
} Canvas;

static void canvas_resize(Canvas *cv, int cols, int rows)
{
    cv->cols = cols < 1 ? 1 : (cols > GRID_COLS_MAX ? GRID_COLS_MAX : cols);
    cv->rows = rows < 1 ? 1 : (rows > GRID_ROWS_MAX ? GRID_ROWS_MAX : rows);

    /* Biggest the crystal can be while still fitting both ways (x runs wider). */
    float fit_y = ((float)cv->rows * 0.5f - MARGIN_CELLS) / EXTENT;
    float fit_x = ((float)cv->cols * 0.5f - MARGIN_CELLS) / (EXTENT * ASPECT_R);
    cv->scale_y = fminf(fit_y, fit_x);
    cv->scale_x = cv->scale_y * ASPECT_R;

    cv->cx = cv->cols / 2;
    cv->cy = cv->rows / 2;
}

static void canvas_clear(Canvas *cv)
{
    memset(cv->cell, 0, sizeof cv->cell);
}

/* Turn a math-plane point into a screen spot.  We subtract for y because on
 * screen rows count downward, but in the math plane up is positive. */
static Pt canvas_project(const Canvas *cv, Vec2 v)
{
    return (Pt){ (float)cv->cx + v.x * cv->scale_x,
                 (float)cv->cy - v.y * cv->scale_y };
}

/* Set one cell, but only if it's actually on the grid.  This bounds check is
 * the one place we guard against drawing off the edge. */
static void canvas_mark(Canvas *cv, int row, int col, uint8_t color, char glyph)
{
    if (row >= 0 && row < cv->rows && col >= 0 && col < cv->cols) {
        cv->cell[row][col].color = color;
        cv->cell[row][col].glyph = glyph;
    }
}

/* Pick the character for a line based on which way it runs (remember rows count
 * downward): mostly sideways -> '-', mostly up/down -> '|', else a slash. */
static char stroke_glyph(float dx, float dy)
{
    float run = fabsf(dx), rise = fabsf(dy);
    if (run  > GLYPH_AXIS_RATIO * rise) return '-';
    if (rise > GLYPH_AXIS_RATIO * run)  return '|';
    return ((dx > 0.0f) == (dy > 0.0f)) ? '\\' : '/';
}

/*
 * Draw a straight line of cells from (x0,y0) to (x1,y1).  This is Bresenham's
 * line algorithm: it walks one cell at a time and uses `err` to decide whether
 * to step sideways, downward, or both, using only whole-number math.
 */
static void canvas_draw_line(Canvas *cv, int x0, int y0, int x1, int y1,
                             uint8_t color, char glyph)
{
    int span_x =  abs(x1 - x0), step_x = x0 < x1 ? 1 : -1;
    int span_y = -abs(y1 - y0), step_y = y0 < y1 ? 1 : -1;
    int err = span_x + span_y;

    for (;;) {
        canvas_mark(cv, y0, x0, color, glyph);
        if (x0 == x1 && y0 == y1) break;
        int err2 = 2 * err;
        if (err2 >= span_y) { err += span_y; x0 += step_x; }   /* step a column */
        if (err2 <= span_x) { err += span_x; y0 += step_y; }   /* step a row    */
    }
}

/* Draw one math-plane segment onto the canvas: find where its two ends land on
 * screen, pick a line character for its slope, then draw between them. */
static void canvas_stroke(Canvas *cv, Vec2 a, Vec2 b, uint8_t color)
{
    Pt   pa    = canvas_project(cv, a);
    Pt   pb    = canvas_project(cv, b);
    char glyph = stroke_glyph(pb.x - pa.x, pb.y - pa.y);
    canvas_draw_line(cv, (int)lroundf(pa.x), (int)lroundf(pa.y),
                         (int)lroundf(pb.x), (int)lroundf(pb.y), color, glyph);
}

/* ===================================================================== */
/* §6  snowflake — the recursive 6-fold branching build                   */
/* ===================================================================== */

/*
 * The fractal rule, and the heart of the whole program.  Draw one line starting
 * at `base`, heading in direction `dir`, `length` long.  Then, if there's depth
 * left, sprout the very same rule again at a couple of points along the line,
 * angled left and right and shrunk down — so each call grows smaller copies of
 * itself.  `level` only picks the colour (0 = the main arm, deeper = brighter).
 */
static void snowflake_branch(Canvas *cv, Vec2 base, Vec2 dir, float length,
                             int level, int max_level)
{
    Vec2 tip = vec_add(base, vec_scale(dir, length));
    canvas_stroke(cv, base, tip, (uint8_t)level_color(level));

    if (level >= max_level) return;

    Vec2 left  = vec_rotate(dir,  BRANCH_ANGLE);
    Vec2 right = vec_rotate(dir, -BRANCH_ANGLE);
    for (int i = 0; i < N_BRANCH_PTS; i++) {
        Vec2 node = vec_add(base, vec_scale(dir, length * BRANCH_FRAC[i]));
        snowflake_branch(cv, node, left,  length * BRANCH_SCALE, level + 1, max_level);
        snowflake_branch(cv, node, right, length * BRANCH_SCALE, level + 1, max_level);
    }
}

/*
 * snowflake_build — THE processing: clear the canvas and grow the whole crystal at
 * the given depth.  Six identical arms, rotated 60° apart around the hub; because
 * each arm uses the same mirror-balanced rule, the result carries full D6 symmetry
 * automatically.  A pure function of (depth, canvas size).
 */
static void snowflake_build(Canvas *cv, int depth)
{
    canvas_clear(cv);
    Vec2 hub = { 0.0f, 0.0f };
    for (int arm = 0; arm < N_ARMS; arm++) {
        Vec2 dir = vec_from_angle((float)arm * ARM_STEP);
        snowflake_branch(cv, hub, dir, ARM_LENGTH, 0, depth);
    }
}

/* ===================================================================== */
/* §7  scene — orchestration: depth + theme + canvas + dirty              */
/* ===================================================================== */

/*
 * Scene — the complete logical state of the explorer, independent of the terminal.
 * The orchestration layer: it owns the detail level (depth), the palette (theme),
 * the cached crystal for that depth (canvas), and one flag (dirty) that binds them.
 * Keeping it terminal-agnostic is what lets the build (§6) and the draw (§8) stay
 * separate: anything that changes the geometry just sets dirty, and scene_update()
 * is then the single place that ever pays for a rebuild — and only when one is owed.
 */
typedef struct {
    int    depth;    /* recursion depth — the detail knob (more depth → lacier crystal)  */
    int    theme;    /* index into k_themes[]; changing it is a pure recolor, no rebuild */
    Canvas canvas;   /* §5/§6 cached crystal rendered at `depth`                         */
    bool   dirty;    /* true ⇒ depth or terminal size changed and `canvas` is stale.
                        Set by scene_set_depth / scene_resize; cleared by scene_update()
                        once it rebuilds.  This flag is the entire reason idle frames
                        are cheap. */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->depth = DEPTH_DEFAULT;
    s->theme = 0;
    canvas_resize(&s->canvas, cols, rows);
    s->dirty = true;
}

/* scene_update — the only non-render processing: rebuild the crystal if (and only
 * if) depth or size changed.  Idle frames skip it. */
static void scene_update(Scene *s)
{
    if (!s->dirty) return;
    snowflake_build(&s->canvas, s->depth);
    s->dirty = false;
}

static void scene_resize(Scene *s, int cols, int rows)
{
    canvas_resize(&s->canvas, cols, rows);
    s->dirty = true;
}

/* scene_set_depth — clamp to [DEPTH_MIN, DEPTH_MAX]; mark dirty on a real change. */
static void scene_set_depth(Scene *s, int depth)
{
    if (depth < DEPTH_MIN) depth = DEPTH_MIN;
    if (depth > DEPTH_MAX) depth = DEPTH_MAX;
    if (depth != s->depth) {
        s->depth = depth;
        s->dirty = true;
    }
}

/* scene_cycle_theme — a pure recolor: re-bind the ramp, leave the canvas. */
static void scene_cycle_theme(Scene *s, int dir)
{
    s->theme = (s->theme + dir + N_THEMES) % N_THEMES;
    theme_apply(s->theme);
}

/* ===================================================================== */
/* §8  render — canvas → glyphs + HUD (READ-ONLY)                         */
/* ===================================================================== */

/*
 * Screen — the live terminal size, refreshed from getmaxyx() at start-up and after
 * every SIGWINCH.  Held apart from Scene on purpose: it is a property of the
 * display, not of the crystal.  The HUD clamps its text to these bounds and the
 * canvas carves its drawing area out of them.
 */
typedef struct {
    int rows;   /* terminal height, in character cells */
    int cols;   /* terminal width,  in character cells */
} Screen;

/* render_canvas — blit the cached cells.  Read-only: never writes the Canvas. */
static void render_canvas(const Canvas *cv)
{
    for (int row = 0; row < cv->rows; row++) {
        for (int col = 0; col < cv->cols; col++) {
            Cell c = cv->cell[row][col];
            if (c.color == 0) continue;
            attron(COLOR_PAIR((int)c.color) | A_BOLD);
            mvaddch(row, col, (chtype)(unsigned char)c.glyph);
            attroff(COLOR_PAIR((int)c.color) | A_BOLD);
        }
    }
}

/* hud_line — one HUD line at (row, x), truncated to the screen width so it can
 * never wrap or overrun, however long the string or narrow the terminal. */
static void hud_line(int row, int x, int cols, int pair, attr_t attr, const char *str)
{
    if (x < 0) x = 0;
    if (x >= cols) return;
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(row, x, "%.*s", cols - x, str);
    attroff(COLOR_PAIR(pair) | attr);
}

/* branch_count — total segments drawn at this depth: 6 arms × Σ Bˡ, B = children. */
static int branch_count(int depth)
{
    int children = N_BRANCH_PTS * 2;
    int per_arm = 0, term = 1;
    for (int l = 0; l <= depth; l++) { per_arm += term; term *= children; }
    return N_ARMS * per_arm;
}

/*
 * render_hud — data on top, actions on the bottom:
 *   row 0      (yellow bold)  depth, segment count, theme
 *   row rows-1 (cyan bold)    every interactive key
 */
static void render_hud(const Screen *s, const Scene *sc)
{
    char buf[96];
    snprintf(buf, sizeof buf, " Snowflake  depth:%d  segments:%d  theme:%s ",
             sc->depth, branch_count(sc->depth), k_themes[sc->theme].name);
    hud_line(0, 0, s->cols, COL_HUD, A_BOLD, buf);

    hud_line(s->rows - 1, 0, s->cols, COL_HINT, A_BOLD,
             " q:quit  +/-:depth  r:reset  t:theme ");
}

/* ===================================================================== */
/* §9  app — main loop                                                    */
/* ===================================================================== */

/*
 * App — the whole running program in one object: the Scene being explored, the
 * Screen it is drawn on, and the two flags the OS pokes asynchronously.  Bundling
 * them is what lets a single file-scope instance (g_app) be reached from the signal
 * handler while everything else is passed by pointer.
 */
typedef struct {
    Scene                 scene;         /* §7 depth + theme + canvas + dirty */
    Screen                screen;        /* §8 terminal size                  */
    volatile sig_atomic_t running;       /* cleared by SIGINT/SIGTERM or 'q'  */
    volatile sig_atomic_t need_resize;   /* set by SIGWINCH                   */
} App;

/* The one global: signal handlers receive only an int, so the flags they set must
 * be reachable without a parameter (volatile sig_atomic_t).  Everything else is
 * reached through this object. */
static App g_app;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}

static void cleanup(void) { endwin(); }

static void terminal_init(void)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);     /* getch() returns ERR instead of blocking */
    keypad(stdscr, TRUE);
    typeahead(-1);             /* don't let pending input interrupt the diff write */
    color_init();
}

/* app_apply_resize — after SIGWINCH: rebuild ncurses' screen model, re-read the
 * size, and mark the canvas stale so the next frame rebuilds at the new size. */
static void app_apply_resize(App *app)
{
    endwin();
    refresh();
    getmaxyx(stdscr, app->screen.rows, app->screen.cols);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

/* app_handle_key — translate one keypress into a scene action; false = quit. */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case '+': case '=': scene_set_depth(&app->scene, app->scene.depth + 1); break;
    case '-': case '_': scene_set_depth(&app->scene, app->scene.depth - 1); break;

    case 'r': case 'R': scene_set_depth(&app->scene, DEPTH_DEFAULT); break;

    case 't': scene_cycle_theme(&app->scene, +1); break;
    case 'T': scene_cycle_theme(&app->scene, -1); break;

    default: break;
    }
    return true;
}

static void app_drain_input(App *app)
{
    int ch;
    while ((ch = getch()) != ERR)
        if (!app_handle_key(app, ch)) { app->running = 0; break; }
}

/* app_present — compose one frame: clear, blit the cached canvas, overlay the HUD,
 * and flush as a single terminal diff. */
static void app_present(App *app)
{
    erase();
    render_canvas(&app->scene.canvas);
    render_hud(&app->screen, &app->scene);
    wnoutrefresh(stdscr);
    doupdate();
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    terminal_init();

    App *app     = &g_app;
    app->running = 1;
    getmaxyx(stdscr, app->screen.rows, app->screen.cols);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    while (app->running) {
        if (app->need_resize) app_apply_resize(app);

        int64_t frame_start = clock_ns();

        app_drain_input(app);          /* 1. input   → depth/theme edits (sets dirty) */
        scene_update(&app->scene);     /* 2. process → rebuild the crystal if dirty   */
        app_present(app);              /* 3. render  → blit cached canvas + HUD       */

        clock_sleep_ns(NS_PER_SEC / RENDER_FPS - (clock_ns() - frame_start));
    }
    return 0;
}
