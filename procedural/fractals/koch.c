/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * koch.c — the Koch snowflake, drawn one stroke at a time.
 *
 * Start with a triangle. Take every straight edge, pinch out a little
 * triangular bump in the middle, and you now have four shorter edges where
 * there was one. Do that again to all of them, and again — the outline gets
 * more and more crinkled. That repeated bump-the-middle rule is the whole
 * idea, and it makes the classic snowflake shape.
 *
 * Press 'n' to step up a detail level (the picture holds when it finishes;
 * it won't auto-advance). The drawing animates along the outline so you can
 * watch the colour sweep around as it fills in.
 *
 * References the code can't give you:
 *   - von Koch (1904), the original paper introducing this curve.
 *   - Mandelbrot, "The Fractal Geometry of Nature" (1982), for why it matters.
 *   - Sister files: dragon_curve.c (same stroke animation, different rule),
 *     l_system.c (draws Koch from a rewrite grammar instead).
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

    MAX_LEVEL       =   5,    /* how crinkled the outline can get         */
    DEFAULT_LEVEL   =   3,    /* level shown at start and on reset        */
    N_KOCH_COLORS   =   5,    /* how many colour bands run along the line  */
    N_THEMES        =   5,    /* colour themes to cycle with t/T          */

    LEVEL_DRAW_TICKS = 60,    /* spread the draw over this many ticks (~2 s @30fps) */

    /* Big enough for the busiest level. Each level quadruples the edge
     * count; level 5 is 3072 edges, so 4096 leaves headroom. */
    MAX_SEGS        = 4096,

    CANVAS_ROWS_MAX =  80,
    CANVAS_COLS_MAX = 300,

    HUD_COLS        =  80,
    FPS_UPDATE_MS   = 500,
};

/* Terminal characters are about twice as tall as they are wide, so without a
 * fudge the snowflake comes out squashed. ASPECT_R squeezes it back to round. */
#define ASPECT_R  2.0f
/* The bump in the middle of each edge is an equilateral triangle, so finding
 * its tip means turning a direction by 60 degrees. These are sin/cos of 60. */
#define SIN60     0.8660254f
#define COS60     0.5f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * ColorID — the slots ncurses uses for colour. K1..K5 are the five bands that
 * run along the outline as it draws (K1 the first strokes, K5 the last), so the
 * colour you see tells you how recently a stretch was drawn. The two HUD slots
 * are kept separate from the themes so the on-screen text stays readable no
 * matter which palette is active.
 */
typedef enum {
    COL_K1  = 1,   /* earliest strokes */
    COL_K2  = 2,
    COL_K3  = 3,
    COL_K4  = 4,
    COL_K5  = 5,   /* latest strokes   */
    COL_HUD = 6,   /* top status text — yellow */
    COL_HINT = 7,  /* bottom key bar  — cyan   */
} ColorID;

/*
 * Theme — a named set of five colours that shade the outline from start to end,
 * swapped live with t / T.
 *
 * The snowflake is drawn as one long stroke, so colouring it by draw order
 * (first strokes one colour, last strokes another) turns "when did this get
 * drawn" into something you can see — the colour sweeps around as it fills in.
 * Every colour here sits in the bright half of the palette on purpose: the dark
 * end of the range disappears against a black terminal. To add a theme, just
 * add a row; nothing else changes.
 */
typedef struct {
    const char *name;            /* shown in the HUD, e.g. "Aurora"           */
    int fg256[N_KOCH_COLORS];    /* the colours on a 256-colour terminal      */
    int fg8[N_KOCH_COLORS];      /* fallback for old 8-colour terminals       */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*            name      K1   K2   K3   K4   K5      8-colour fallback (same order) */
    { "Aurora", {  51,  86, 118, 226, 231 }, { COLOR_CYAN,  COLOR_CYAN,  COLOR_GREEN,  COLOR_YELLOW, COLOR_WHITE  } },
    { "Fire",   { 196, 202, 208, 220, 231 }, { COLOR_RED,   COLOR_RED,   COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE  } },
    { "Ice",    {  39,  45,  51, 117, 231 }, { COLOR_BLUE,  COLOR_CYAN,  COLOR_CYAN,   COLOR_CYAN,   COLOR_WHITE  } },
    { "Toxic",  {  46,  82, 118, 154, 226 }, { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,  COLOR_YELLOW, COLOR_YELLOW } },
    { "Mono",   { 245, 248, 250, 252, 231 }, { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE  } },
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
/* §3  geometry — points in the snowflake's plane                         */
/* ===================================================================== */

/*
 * Vec2 — a point (or an arrow pointing somewhere) on the flat plane the
 * snowflake lives on.
 *
 * The shape is described in its own clean coordinates that have nothing to do
 * with the terminal: it sits centred on the origin at size 1, and only gets
 * mapped onto actual character cells when it's time to draw (canvas_project).
 * That keeps the math simple and means the shape never depends on window size.
 */
typedef struct {
    float x;   /* left-right, bigger is right */
    float y;   /* up-down,    bigger is up    */
} Vec2;

/* The arrow from b to a (i.e. a − b): which way you'd go from b to reach a. */
static inline Vec2 vec2_sub(Vec2 a, Vec2 b) { return (Vec2){ a.x - b.x, a.y - b.y }; }

/* ===================================================================== */
/* §4  curve — the Koch geometry                                          */
/* ===================================================================== */

/*
 * Segment — one straight piece of the outline, from point a to point b. This is
 * the building block of everything: the bump rule turns one segment into four,
 * and the finished snowflake is just a list of these in the order you'd walk
 * around the edge — which is also the order they're drawn and coloured.
 */
typedef struct {
    Vec2 a;   /* where this piece starts */
    Vec2 b;   /* where it ends           */
} Segment;

/*
 * KochCurve — the whole snowflake at one detail level, laid out flat as a plain
 * list of segments. koch_build fills it once; drawing then just walks the list.
 *
 * It's built into an array up front rather than figured out fresh every frame,
 * so the drawing loop is a simple sweep with no recursion or allocation while
 * it runs. The walk order also gives the colour shading and the draw animation
 * for free. Each level has four times as many segments as the one below, and
 * seg[] is sized for the busiest level (MAX_SEGS).
 */
typedef struct {
    int     level;            /* current detail level, 1..MAX_LEVEL            */
    int     count;            /* how many segments are actually in seg[]        */
    Segment seg[MAX_SEGS];    /* the outline, in walk-around (= draw) order      */
} KochCurve;

/*
 * koch_subdivide — the bump rule, applied recursively. Take edge a→b, find the
 * two points a third and two-thirds of the way along, push the middle out into a
 * little triangular tip, and you now have four shorter edges a→p→m→q→b. Keep
 * doing that to each of the four until `depth` runs out, then record the edge.
 */
static void koch_subdivide(KochCurve *k, Vec2 a, Vec2 b, int depth)
{
    if (depth == 0) {
        if (k->count < MAX_SEGS) k->seg[k->count++] = (Segment){ a, b };
        return;
    }
    /* p and q split the edge into three equal parts. m is the middle part's tip:
     * take the middle third as a direction, turn it 60 degrees so it points
     * outward, and that lands you on the peak of the bump. */
    Vec2 p = { a.x + (b.x - a.x) / 3.0f,        a.y + (b.y - a.y) / 3.0f };
    Vec2 q = { a.x + (b.x - a.x) * 2.0f / 3.0f, a.y + (b.y - a.y) * 2.0f / 3.0f };
    Vec2 d = vec2_sub(q, p);
    Vec2 m = { p.x + COS60 * d.x - SIN60 * d.y,
               p.y + SIN60 * d.x + COS60 * d.y };

    koch_subdivide(k, a, p, depth - 1);
    koch_subdivide(k, p, m, depth - 1);
    koch_subdivide(k, m, q, depth - 1);
    koch_subdivide(k, q, b, depth - 1);
}

/*
 * koch_build — make the snowflake by starting from a plain triangle and bumping
 * every edge `level` times. The three corners sit at the top, bottom-right, and
 * bottom-left of a triangle centred on the origin.
 */
static void koch_build(KochCurve *k, int level)
{
    if (level < 1)         level = 1;
    if (level > MAX_LEVEL) level = MAX_LEVEL;
    k->level = level;
    k->count = 0;

    Vec2 v1 = {  0.0f,   1.0f };
    Vec2 v2 = {  SIN60, -0.5f };
    Vec2 v3 = { -SIN60, -0.5f };
    koch_subdivide(k, v1, v2, level);
    koch_subdivide(k, v2, v3, level);
    koch_subdivide(k, v3, v1, level);
}

/* ===================================================================== */
/* §5  color — gradient along the curve, bound to the active theme        */
/* ===================================================================== */

/*
 * curve_band — which of the five colour bands a segment belongs to, based on how
 * far along the outline it sits. Early segments get band 1, late ones band 5, so
 * the colour shifts smoothly as the drawing sweeps around.
 */
static uint8_t curve_band(int idx, int count)
{
    int band = 1 + (int)((float)idx / (float)count * (float)N_KOCH_COLORS);
    return (uint8_t)(band > N_KOCH_COLORS ? N_KOCH_COLORS : band);
}

/* Point the five outline colours at whichever theme is currently selected. */
static void theme_apply(int theme)
{
    const Theme *t = &k_themes[theme];
    bool truecolor = COLORS >= 256;
    for (int i = 0; i < N_KOCH_COLORS; i++)
        init_pair(COL_K1 + i, truecolor ? t->fg256[i] : t->fg8[i], COLOR_BLACK);
}

static void color_init(void)
{
    start_color();
    /* The HUD colours don't follow the themes — yellow status, cyan keys, so the
     * text reads clearly whatever palette the outline is using. */
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);
        init_pair(COL_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    theme_apply(0);   /* start on Aurora; Scene.theme tracks it from here on */
}

/* ===================================================================== */
/* §6  canvas — the cell buffer and the Euclid → cell projection          */
/* ===================================================================== */

/*
 * Cell — a spot on the character grid (column, row): where one of the
 * snowflake's points ends up once it's mapped onto the screen. It's the bridge
 * between the smooth-coordinate world (Vec2) and the blocky terminal grid.
 */
typedef struct {
    int col;   /* column, 0 = left */
    int row;   /* row,    0 = top  */
} Cell;

/*
 * Canvas — the picture held in memory: a grid that remembers a colour for every
 * character cell, plus the settings for fitting the snowflake onto that grid.
 *
 * Drawing into a buffer first (instead of straight to the terminal) keeps two
 * jobs apart: working out where the shape lands and what colour it is, versus
 * actually pushing characters to the screen (canvas_paint). The draw animation
 * is just repainting this same grid as it fills up.
 *
 * `scale` is how many columns one unit of the shape stretches to, and rows get
 * divided by ASPECT_R so the snowflake looks round rather than tall and squashed.
 */
typedef struct {
    int     rows, cols;                              /* drawing area, in cells       */
    float   scale;                                   /* columns per unit of the shape */
    uint8_t cell[CANVAS_ROWS_MAX][CANVAS_COLS_MAX];  /* 0 = blank, else a colour band */
} Canvas;

static void canvas_reset(Canvas *cv, int cols, int rows)
{
    if (cols > CANVAS_COLS_MAX) cols = CANVAS_COLS_MAX;
    if (rows > CANVAS_ROWS_MAX) rows = CANVAS_ROWS_MAX;
    cv->cols = cols;
    cv->rows = rows;
    memset(cv->cell, 0, sizeof cv->cell);

    /* Size the shape to fit, picking whichever of width or height is tighter so
     * it never spills off the screen. */
    float by_cols = (float)cols * 0.45f;
    float by_rows = (float)rows * 0.45f * ASPECT_R;
    cv->scale = (by_cols < by_rows) ? by_cols : by_rows;
}

/* Turn one of the snowflake's points into a grid cell: centre it, scale it up,
 * and correct for the tall-character squash. */
static Cell canvas_project(const Canvas *cv, Vec2 p)
{
    return (Cell){
        .col = cv->cols / 2 + (int)roundf(p.x * cv->scale),
        .row = cv->rows / 2 - (int)roundf(p.y * cv->scale / ASPECT_R),
    };
}

/*
 * canvas_stroke — draw one straight segment into the grid, one cell at a time.
 * It uses Bresenham's line algorithm, the classic trick for drawing a straight
 * line on a grid using only whole-number steps (no decimals per cell).
 */
static void canvas_stroke(Canvas *cv, Segment s, uint8_t color)
{
    Cell a = canvas_project(cv, s.a);
    Cell b = canvas_project(cv, s.b);

    int x0 = a.col, y0 = a.row, x1 = b.col, y1 = b.row;
    int dx =  abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        if (x0 >= 0 && x0 < cv->cols && y0 >= 0 && y0 < cv->rows)
            cv->cell[y0][x0] = color;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void canvas_paint(const Canvas *cv, WINDOW *w)
{
    for (int row = 0; row < cv->rows; row++) {
        for (int col = 0; col < cv->cols; col++) {
            uint8_t c = cv->cell[row][col];
            if (c == 0) continue;
            attr_t attr = COLOR_PAIR((int)c);
            if (c >= COL_K4) attr |= A_BOLD;          /* make the newest strokes pop */
            wattron(w, attr);
            mvwaddch(w, row, col, (chtype)'*');
            wattroff(w, attr);
        }
    }
}

/* ===================================================================== */
/* §7  reveal — the stroke-by-stroke draw progress                        */
/* ===================================================================== */

/*
 * Reveal — tracks how much of the outline has been drawn so far and how quickly
 * the rest keeps appearing.
 *
 * Drawing the outline gradually, in order, is what lets you watch it being made
 * and see the colour sweep around — rather than the whole shape just popping
 * into view. `drawn` grows by `per_tick` segments each tick until the outline is
 * finished, then it holds (it won't move to the next level on its own).
 * `per_tick` is set from the segment count so every level takes about the same
 * amount of time on the clock, whether it's a dozen segments or a few thousand.
 */
typedef struct {
    int  drawn;       /* how many segments are on screen so far               */
    int  per_tick;    /* how many more to add each tick (set by reveal_begin)  */
    bool paused;      /* frozen by spc/p; stays frozen across level/resize     */
} Reveal;

static void reveal_begin(Reveal *r, int count)
{
    r->drawn    = 0;
    r->per_tick = count / LEVEL_DRAW_TICKS + 1;
    /* deliberately don't touch `paused` — a pause should survive a level change */
}

static bool reveal_complete(const Reveal *r, int count) { return r->drawn >= count; }

/* ===================================================================== */
/* §8  scene — curve + canvas + reveal = the live picture                 */
/* ===================================================================== */

/*
 * Scene — everything making up the live picture, in one place: the shape, the
 * grid it's painted onto, the draw-in animation, and the current colour theme.
 *
 * These belong together because they change together. Stepping a level,
 * resetting, or resizing all rebuild the shape, refit the grid, and restart the
 * animation as one move, and the input handling in §10 has a single thing to act
 * on.
 */
typedef struct {
    KochCurve curve;     /* the shape at the current detail level        */
    Canvas    canvas;    /* the grid it's drawn onto                      */
    Reveal    reveal;    /* how far the draw-in has got                   */
    int       theme;     /* which colour theme, cycled with t / T         */
} Scene;

/*
 * scene_setup — (re)build the snowflake at a given level for the current window
 * size. It rebuilds the shape, refits the grid, and restarts the draw-in, but
 * leaves the theme and the pause state alone so a resize or level step keeps them.
 */
static void scene_setup(Scene *s, int cols, int rows, int level)
{
    canvas_reset(&s->canvas, cols, rows);
    koch_build(&s->curve, level);
    reveal_begin(&s->reveal, s->curve.count);
}

/* Used for the first start and for reset: go back to the default level. */
static void scene_init(Scene *s, int cols, int rows)
{
    scene_setup(s, cols, rows, DEFAULT_LEVEL);
}

static void scene_next_level(Scene *s, int cols, int rows)
{
    scene_setup(s, cols, rows, s->curve.level % MAX_LEVEL + 1);   /* loops back to 1 after the top */
}

static void scene_cycle_theme(Scene *s)
{
    s->theme = (s->theme + 1) % N_THEMES;
    theme_apply(s->theme);
}

/*
 * scene_tick — draw the next handful of strokes. Once the outline is complete it
 * just sits there; press 'n' to move on to the next level.
 */
static void scene_tick(Scene *s)
{
    if (s->reveal.paused) return;
    if (reveal_complete(&s->reveal, s->curve.count)) return;

    int end = s->reveal.drawn + s->reveal.per_tick;
    if (end > s->curve.count) end = s->curve.count;

    for (int i = s->reveal.drawn; i < end; i++)
        canvas_stroke(&s->canvas, s->curve.seg[i], curve_band(i, s->curve.count));

    s->reveal.drawn = end;
}

static void scene_draw(const Scene *s, WINDOW *w) { canvas_paint(&s->canvas, w); }

/* ===================================================================== */
/* §9  screen — ncurses lifecycle + HUD                                   */
/* ===================================================================== */

/*
 * Screen — the terminal's current width and height, remembered so the layout
 * doesn't have to ask ncurses for it every single frame. It's updated at startup
 * and again whenever the window is resized.
 */
typedef struct {
    int cols;   /* width  in characters */
    int rows;   /* height in characters */
} Screen;

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

/*
 * hud_line — print one line of status text, trimmed to fit the window so a long
 * line can't wrap around and scribble over the snowflake.
 */
static void hud_line(int row, int x, int pair, attr_t bold, int cols, const char *str)
{
    if (x < 0) x = 0;
    if (x >= cols) return;
    attron(COLOR_PAIR(pair) | bold);
    mvprintw(row, x, "%.*s", cols - x, str);
    attroff(COLOR_PAIR(pair) | bold);
}

/*
 * The status display: facts up top, keys along the bottom.
 *   row 0      title, frame rate, and what it's doing
 *   row 1      level, theme, how far along, and speed
 *   last row   every key you can press
 */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr);

    int count = sc->curve.count;
    int pct = (count > 0) ? sc->reveal.drawn * 100 / count : 100;
    if (pct > 100) pct = 100;
    const char *state = sc->reveal.paused ? "PAUSED "
                      : (reveal_complete(&sc->reveal, count) ? "complete" : "drawing ");

    char buf[HUD_COLS + 1];

    /* row 0 — the headline, pushed to the right and bold */
    snprintf(buf, sizeof buf, " KochSnowflake  %5.1f fps  %s ", fps, state);
    hud_line(0, s->cols - (int)strlen(buf), COL_HUD, A_BOLD, s->cols, buf);

    /* row 1 — the details, kept un-bold so row 0 stands out */
    snprintf(buf, sizeof buf, " level %d/%d  theme:%s  %d%%  spd:%d Hz ",
             sc->curve.level, MAX_LEVEL, k_themes[sc->theme].name, pct, sim_fps);
    hud_line(1, 0, COL_HUD, A_NORMAL, s->cols, buf);

    /* bottom row — the keys, in bright cyan */
    hud_line(s->rows - 1, 0, COL_HINT, A_BOLD, s->cols,
             " q:quit  n:next level  t:theme  r:reset  spc:pause  [ / ]:speed ");
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §10  app — signals, input, main loop                                   */
/* ===================================================================== */

/*
 * App — the running program bundled together: the picture (scene), the window
 * size (screen), and how fast the drawing animates (sim_fps). There's one of
 * these, kept in fixed storage rather than on the stack because Scene holds some
 * big arrays. Every key press and the main loop work on this one object.
 */
typedef struct {
    Scene  scene;       /* the live picture                                      */
    Screen screen;      /* the window size                                       */
    int    sim_fps;     /* animation speed, stepped by [ and ]                   */
} App;

/* These two are global because signal handlers can't take arguments — all they
 * can do is set a flag that the main loop checks each time around. */
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
    screen_resize(&a->screen);
    /* stay on the same level after a resize — only the fit-to-window size changes */
    scene_setup(&a->scene, a->screen.cols, a->screen.rows, a->scene.curve.level);
    g_should_resize = 0;
}

static void app_change_speed(App *a, int delta)
{
    int fps = a->sim_fps + delta;
    if (fps < SIM_FPS_MIN) fps = SIM_FPS_MIN;
    if (fps > SIM_FPS_MAX) fps = SIM_FPS_MAX;
    a->sim_fps = fps;
}

static bool app_handle_key(App *a, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case 'n': case 'N':
        scene_next_level(&a->scene, a->screen.cols, a->screen.rows);
        break;

    case 't': case 'T':
        scene_cycle_theme(&a->scene);
        break;

    case 'r': case 'R':
        scene_init(&a->scene, a->screen.cols, a->screen.rows);   /* back to the default level */
        break;

    case 'p': case 'P': case ' ':
        a->scene.reveal.paused = !a->scene.reveal.paused;
        break;

    case ']': app_change_speed(a, +SIM_FPS_STEP); break;
    case '[': app_change_speed(a, -SIM_FPS_STEP); break;

    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    /* not on the stack — App contains big arrays that wouldn't comfortably fit */
    static App app = { .sim_fps = SIM_FPS_DEFAULT };

    screen_init(&app.screen);
    scene_init(&app.scene, app.screen.cols, app.screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (!g_should_quit) {

        if (g_should_resize) {
            app_resize(&app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app.sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app.scene);
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

        screen_draw(&app.screen, &app.scene, fps_display, app.sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(&app, ch))
            g_should_quit = 1;
    }

    screen_free(&app.screen);
    return 0;
}
