/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * mandelbrot.c — draws the Mandelbrot set, building up from scattered dots.
 *
 * The Mandelbrot set is a test you run on each point of the screen: keep
 * squaring a number and adding the point back in; if the result stays small
 * forever the point is "inside" the set, and if it runs off toward infinity
 * it's outside. How fast it runs off picks the colour. See julia.c and
 * burning_ship.c for close cousins of this same idea.
 *
 * Mandelbrot, B. B. (1982). "The Fractal Geometry of Nature." Freeman.
 * Peitgen & Richter (1986). "The Beauty of Fractals." Springer — the
 * escape-time colouring scheme used here.
 */

#define _POSIX_C_SOURCE 200809L

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
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  30,
    SIM_FPS_MAX      =  60,
    SIM_FPS_STEP     =   5,

    PIXELS_PER_TICK  =  60,    /* how many dots to fill in each tick (the build-up speed) */
    MAX_ITER         = 256,    /* give up after this many squarings; if still small, call it "inside" */
    DONE_PAUSE_TICKS =  90,    /* how long to admire a finished picture (~3 s) before moving on */
    N_PRESETS        =   6,
    N_COLORS         =   5,    /* one colour for "inside", plus four for how fast a point escaped */
    N_THEMES         =  10,

    GRID_ROWS_MAX    =  80,
    GRID_COLS_MAX    = 300,

    HUD_TOP_ROWS     =   2,    /* rows 0..1 reserved for the status/info bar at the top */
    HUD_BOT_ROWS     =   1,    /* bottom row reserved for the key-hints bar */
    HUD_COLS         =  96,
    FPS_UPDATE_MS    = 500,
    FRAME_DT_CAP_MS  = 100,    /* never let one slow frame count as more than this, so we don't snowball */
    RENDER_FPS_CAP   =  60,    /* draw at most this many frames per second */
};

/* A point has "escaped" once it gets bigger than 2. We compare the squared
 * size against 4 instead, which means one less square root per pixel. */
#define ESCAPE_R2        4.0f
/* Points that escape almost instantly are far outside the set; we leave those
 * blank rather than colouring them, so only the interesting halo shows. */
#define ESCAPE_FRAC_MIN  0.10f

/* Terminal cells are about twice as tall as they are wide. We stretch the
 * horizontal span by this much so the fractal looks round, not squashed. */
#define ASPECT_R    2.0f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Theme — one colour scheme. Five colours, going from the body of the fractal
 * out to its edge, picked by how fast each point escaped (Peitgen & Richter 1986):
 *   c[0] = the solid interior (points that never escaped)
 *   c[1] = points that escaped quickest — the faint outer halo
 *   c[2..4] = slower and slower escapes, working in toward the body
 * c8[] is a plain-8-colour version for terminals that can't do 256 colours.
 * The HUD keeps its own fixed colours (yellow + cyan) so it stays readable on
 * any theme. To add a palette, add one row to the table below; nothing else changes.
 */
typedef struct {
    const char *name;   /* shown in the info bar */
    int c[5];           /* the five colours, for 256-colour terminals */
    int c8[5];          /* same five, fallback for 8-colour terminals */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*              name       INSIDE  C2    C3    C4    C5     8-colour fallbacks */
    { "Electric", {201, 226,  82,  51, 141},  {COLOR_MAGENTA,COLOR_YELLOW,COLOR_GREEN,COLOR_CYAN,COLOR_CYAN} },
    { "Matrix",   { 46,  28,  34,  40, 118},  {COLOR_GREEN,  COLOR_GREEN, COLOR_GREEN,COLOR_GREEN,COLOR_GREEN} },
    { "Nova",     {231,  25,  33,  39, 117},  {COLOR_WHITE,  COLOR_BLUE,  COLOR_BLUE, COLOR_CYAN, COLOR_CYAN} },
    { "Poison",   { 82,  28, 100, 148, 190},  {COLOR_GREEN,  COLOR_GREEN, COLOR_GREEN,COLOR_YELLOW,COLOR_YELLOW} },
    { "Ocean",    {159,  25,  26,  33,  38},  {COLOR_CYAN,   COLOR_BLUE,  COLOR_BLUE, COLOR_CYAN, COLOR_CYAN} },
    { "Fire",     {231,  52,  88, 196, 214},  {COLOR_WHITE,  COLOR_RED,   COLOR_RED,  COLOR_YELLOW,COLOR_YELLOW} },
    { "Gold",     {231,  52,  94, 136, 220},  {COLOR_WHITE,  COLOR_RED,   COLOR_YELLOW,COLOR_YELLOW,COLOR_YELLOW} },
    { "Ice",      {231,  24,  30,  31, 159},  {COLOR_WHITE,  COLOR_BLUE,  COLOR_BLUE, COLOR_CYAN, COLOR_CYAN} },
    { "Nebula",   {231,  55,  93, 141, 183},  {COLOR_WHITE,  COLOR_MAGENTA,COLOR_CYAN,COLOR_CYAN,COLOR_CYAN} },
    { "Lava",     {226,  52, 124, 208, 220},  {COLOR_YELLOW, COLOR_RED,   COLOR_RED,  COLOR_YELLOW,COLOR_YELLOW} },
};

/* ===================================================================== */
/* §2  clock — monotonic ns clock + fixed-timestep frame pacing           */
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

/*
 * FrameClock — keeps the animation running at a steady pace. Real frames come in
 * at uneven times, but we want the picture to fill in by equal amounts. So each
 * frame we add up the real time that passed (sim_debt) and "pay it back" in
 * fixed-size steps. The fps fields just measure the current frame rate to show it.
 */
typedef struct {
    int64_t prev_ns;     /* the clock reading when this frame started, in nanoseconds */
    int64_t sim_debt;    /* time that's passed but not yet turned into steps */
    int64_t fps_window;  /* time gathered toward the next frame-rate measurement */
    int     fps_frames;  /* frames counted toward the next frame-rate measurement */
    double  fps;         /* the frame rate to display, refreshed twice a second */
} FrameClock;

static void frameclock_init(FrameClock *fc)
{
    fc->prev_ns    = clock_ns();
    fc->sim_debt   = 0;
    fc->fps_window = 0;
    fc->fps_frames = 0;
    fc->fps        = 0.0;
}

static void frameclock_sample_fps(FrameClock *fc, int64_t dt)
{
    fc->fps_frames++;
    fc->fps_window += dt;
    if (fc->fps_window >= FPS_UPDATE_MS * NS_PER_MS) {
        fc->fps        = (double)fc->fps_frames
                       / ((double)fc->fps_window / (double)NS_PER_SEC);
        fc->fps_frames = 0;
        fc->fps_window = 0;
    }
}

/* Start a frame: see how long since the last one, cap any big hiccup, save it. */
static void frameclock_begin_frame(FrameClock *fc)
{
    int64_t now = clock_ns();
    int64_t dt  = now - fc->prev_ns;
    fc->prev_ns = now;

    int64_t stall_cap = FRAME_DT_CAP_MS * NS_PER_MS;
    if (dt > stall_cap) dt = stall_cap;

    fc->sim_debt += dt;
    frameclock_sample_fps(fc, dt);
}

/* Returns true while we still owe another step, subtracting one step each time. */
static bool frameclock_step_due(FrameClock *fc, int64_t tick_ns)
{
    if (fc->sim_debt < tick_ns) return false;
    fc->sim_debt -= tick_ns;
    return true;
}

/* If we finished the frame early, sleep the leftover time so we don't run too fast. */
static void frameclock_throttle(const FrameClock *fc)
{
    int64_t budget  = NS_PER_SEC / RENDER_FPS_CAP;
    int64_t elapsed = clock_ns() - fc->prev_ns;
    clock_sleep_ns(budget - elapsed);
}

/* ===================================================================== */
/* §3  color — themeable content palette + HUD pairs                      */
/* ===================================================================== */

/*
 * Names for ncurses colour slots. INSIDE and C2..C5 are the fractal's five
 * colours and get swapped out whenever the theme changes. HUD and HINT keep
 * their own fixed colours so the on-screen text stays readable on any theme.
 */
typedef enum {
    COL_INSIDE = 1,   /* the solid body of the set   */
    COL_C2     = 2,   /* quickest escape — far halo  */
    COL_C3     = 3,
    COL_C4     = 4,
    COL_C5     = 5,   /* slowest escape — near body  */
    COL_HUD    = 6,   /* info text     — bright yellow */
    COL_HINT   = 7,   /* key-hint bar  — bright cyan   */
} ColorID;

/* Point the five fractal colours at the chosen theme (the HUD colours don't change). */
static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    bool truecolor = COLORS >= 256;
    for (int i = 0; i < 5; i++)
        init_pair((short)(COL_INSIDE + i),
                  (short)(truecolor ? th->c[i] : th->c8[i]), COLOR_BLACK);
}

static void color_init(void)
{
    start_color();
    /* The info text and key hints always use these colours, whatever the theme. */
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);
        init_pair(COL_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    theme_apply(0);   /* start on the first theme; the Scene remembers it from here on */
}

/* ===================================================================== */
/* §4  core — complex iteration + escape time (PURE MATH)                 */
/* ===================================================================== */

/*
 * Complex — a "complex number," which is really just a pair of numbers (a point
 * with a left-right part and an up-down part). The whole fractal is built from
 * one rule for multiplying these pairs. Two of them matter here: the point on the
 * screen we're testing, and the running value we keep squaring. We store the two
 * parts as plain floats so the inner loop stays fast.
 */
typedef struct {
    float re;   /* the left-right part; maps across the screen columns */
    float im;   /* the up-down part; maps along the screen rows */
} Complex;

/* One step of the rule: square z and add c. (Squaring a complex number works out
 * to these two lines.) */
static Complex complex_sqr_add(Complex z, Complex c)
{
    return (Complex){ z.re * z.re - z.im * z.im + c.re,
                      2.0f * z.re * z.im        + c.im };
}

/* How far z is from the centre, squared. We square it to skip a slow square root;
 * the escape test was set up to compare against the squared distance to match. */
static float complex_norm2(Complex z) { return z.re * z.re + z.im * z.im; }

/*
 * The core test for one point. Start at zero and keep squaring-and-adding. Return
 * how many steps it took to run off to infinity, or MAX_ITER if it never did
 * (that means the point is inside the set). Same point in, same count out.
 */
static int mandelbrot_escape(Complex c)
{
    Complex z = { 0.0f, 0.0f };
    int iter = 0;
    for (; iter < MAX_ITER; iter++) {
        z = complex_sqr_add(z, c);
        if (complex_norm2(z) > ESCAPE_R2) break;
    }
    return iter;
}

/*
 * Turn "how slowly it escaped" into one of the four halo colours. Points that
 * took longer to escape (closer to the body) get the higher colours; the quickest
 * escapes shown get the lowest. The math just spreads the slow-to-fast range
 * evenly across the four colours.
 */
static uint8_t escape_band(float frac)
{
    float t = (frac - ESCAPE_FRAC_MIN) / (1.0f - ESCAPE_FRAC_MIN);
    int   band = COL_C2 + (int)(t * (float)(N_COLORS - 1));
    return (uint8_t)(band > COL_C5 ? COL_C5 : band);
}

/*
 * Decide which colour a point gets from its escape count:
 *   never escaped         -> the solid interior colour
 *   escaped, by how slow  -> one of the four halo colours
 *   escaped almost at once -> blank, so distant points stay empty
 */
static uint8_t cell_for_escape(int iter)
{
    if (iter >= MAX_ITER) return (uint8_t)COL_INSIDE;   /* never escaped: it's inside */
    float frac = (float)iter / (float)MAX_ITER;
    if (frac < ESCAPE_FRAC_MIN) return 0;               /* escaped too fast: leave blank */
    return escape_band(frac);
}

/* ===================================================================== */
/* §5  field — cell grid + complex-plane view + cell→c mapping (DATA)     */
/* ===================================================================== */

/*
 * ViewPreset — a saved place to look. It says where to centre the view and how
 * tall a slice to show; the width is worked out later from the terminal shape.
 */
typedef struct {
    float       cr, ci;     /* centre point: left-right (cr) and up-down (ci) */
    float       im_half;    /* half the height of the slice to show */
    const char *name;       /* shown in the info bar */
} ViewPreset;

static const ViewPreset k_presets[N_PRESETS] = {
    { -0.5000f,  0.0000f, 1.300f, "full set"        },
    { -0.7450f,  0.1130f, 0.150f, "seahorse valley" },
    {  0.0000f,  0.6500f, 0.300f, "north antenna"   },
    { -0.7220f,  0.2460f, 0.020f, "deep spiral"     },
    { -1.7500f,  0.0000f, 0.080f, "mini mandelbrot" },
    { -0.6000f,  0.6000f, 0.200f, "antenna tip"     },
};

/*
 * View — the actual patch of the plane we're looking at: its centre and how far
 * it reaches left-right and up-down. A preset becomes one of these once we know
 * the screen size. The width is widened by the terminal's tall-cell shape so
 * round shapes look round instead of stretched.
 */
typedef struct {
    Complex center;
    float   re_half;   /* half the width  (left-right reach from the centre) */
    float   im_half;   /* half the height (up-down reach from the centre) */
} View;

/* Build the View that a preset describes, once we know the grid is cols x rows. */
static View view_from_preset(int preset, int cols, int rows)
{
    const ViewPreset *p = &k_presets[preset % N_PRESETS];
    View v;
    v.center  = (Complex){ p->cr, p->ci };
    v.im_half = p->im_half;
    v.re_half = p->im_half * (float)cols / (float)rows / ASPECT_R;   /* keep it from looking squashed */
    return v;
}

/* Given a spot in the view as fractions from 0 to 1 (0,0 is the top-left), return
 * the actual plane coordinate there. fy = 0 is the top because up-down runs
 * upward but screen rows count downward. */
static Complex view_point(View v, float fx, float fy)
{
    return (Complex){
        v.center.re + (fx - 0.5f) * 2.0f * v.re_half,
        v.center.im + (0.5f - fy) * 2.0f * v.im_half,
    };
}

/*
 * Field — the picture itself: a colour for every cell we've worked out, plus the
 * View those cells come from. Just storage; field_point() below tells you which
 * plane coordinate a given cell stands for.
 */
typedef struct {
    uint8_t cell[GRID_ROWS_MAX][GRID_COLS_MAX];  /* 0 = empty/not done yet, 1..5 = a colour */
    int     rows, cols;                          /* how much of the grid we're actually using */
    View    view;                                /* the patch of plane these cells show */
} Field;

static void field_init(Field *f, int cols, int rows, int preset)
{
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    rows -= HUD_TOP_ROWS + HUD_BOT_ROWS;          /* don't draw over the top and bottom bars */
    if (rows < 1)             rows = 1;
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;

    f->cols = cols;
    f->rows = rows;
    f->view = view_from_preset(preset, cols, rows);
    memset(f->cell, 0, sizeof f->cell);
}

/* The plane coordinate that cell (col, row) stands for. */
static Complex field_point(const Field *f, int col, int row)
{
    float fx = (f->cols > 1) ? (float)col / (float)(f->cols - 1) : 0.5f;
    float fy = (f->rows > 1) ? (float)row / (float)(f->rows - 1) : 0.5f;
    return view_point(f->view, fx, fy);
}

/* ===================================================================== */
/* §6  reveal — shuffled progressive fill + done-hold (EFFECTS & DELAYS)  */
/* ===================================================================== */

/*
 * Reveal — controls the "fill in over time" effect, kept separate so the math
 * and the drawing don't have to know about timing:
 *   - order[]      the cells listed in a shuffled order, so the picture appears
 *                  as scattered dots instead of sweeping line by line;
 *   - progress     how many cells we've filled in so far;
 *   - hold_ticks   how long we've sat on a finished picture before moving on.
 * It only decides WHICH cells to do next and how many per tick — the colours and
 * positions still come from the math and the drawing.
 */
typedef struct {
    int order[GRID_ROWS_MAX * GRID_COLS_MAX];  /* the cells, in shuffled order */
    int n;            /* total number of cells (rows * cols) */
    int progress;     /* how far through order[] we've gotten */
    int hold_ticks;   /* ticks waited since the picture finished */
} Reveal;

/* Shuffle a[0..n-1] into a random order (the standard Fisher-Yates method). */
static void shuffle(int *a, int n)
{
    for (int i = n - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int t = a[i]; a[i] = a[j]; a[j] = t;
    }
}

/* reveal_reset() — rewind, then lay out every pixel index in a fresh random order
 * so the next sweep materialises the image from scattered dots. */
static void reveal_reset(Reveal *r, int n)
{
    r->n          = n;
    r->progress   = 0;
    r->hold_ticks = 0;
    for (int i = 0; i < n; i++) r->order[i] = i;
    shuffle(r->order, n);
}

static bool reveal_complete(const Reveal *r) { return r->progress >= r->n; }

static int reveal_progress_pct(const Reveal *r)
{
    if (r->n <= 0) return 100;
    int pct = r->progress * 100 / r->n;
    return pct > 100 ? 100 : pct;
}

/* reveal_step() — compute the next PIXELS_PER_TICK pixels (in shuffled order),
 * evaluating the pure core at each and storing the band into the field. */
static void reveal_step(Reveal *r, Field *f)
{
    int end = r->progress + PIXELS_PER_TICK;
    if (end > r->n) end = r->n;

    for (int s = r->progress; s < end; s++) {
        int idx = r->order[s];
        int col = idx % f->cols;
        int row = idx / f->cols;
        uint8_t cell = cell_for_escape(mandelbrot_escape(field_point(f, col, row)));
        if (cell) f->cell[row][col] = cell;
    }
    r->progress = end;
}

/* ===================================================================== */
/* §7  scene — orchestration: field + reveal + preset + theme            */
/* ===================================================================== */

/*
 * Scene — the whole picture in one object, composed from the layers above so a
 * reader can see what is data, what is the effect, and what the user controls.
 * scene_tick() drives the build-up; theme/preset/paused are user state (theme and
 * preset persist while the field/reveal are rebuilt on each change).
 */
typedef struct {
    Field  field;     /* §5 data:   the sampled bands                */
    Reveal reveal;    /* §6 effect: shuffled build-up + done-hold    */
    int    preset;    /* active zoom window, 0..N_PRESETS-1 (r/n)    */
    int    theme;     /* active colour theme, 0..N_THEMES-1 (t/T)    */
    bool   paused;    /* space: freezes the build-up (the image holds) */
} Scene;

/* scene_load() — point at a preset, size the field to the screen, reshuffle. */
static void scene_load(Scene *s, int preset, int cols, int rows)
{
    s->preset = (preset % N_PRESETS + N_PRESETS) % N_PRESETS;
    field_init(&s->field, cols, rows, s->preset);
    reveal_reset(&s->reveal, s->field.rows * s->field.cols);
}

static void scene_init(Scene *s, int cols, int rows)
{
    s->theme  = 0;
    s->paused = false;
    scene_load(s, 0, cols, rows);
}

/* scene_tick() — advance the build-up; when the image holds long enough, move on. */
static void scene_tick(Scene *s, int cols, int rows)
{
    if (s->paused) return;

    if (!reveal_complete(&s->reveal)) {
        reveal_step(&s->reveal, &s->field);
        return;
    }
    if (++s->reveal.hold_ticks >= DONE_PAUSE_TICKS)
        scene_load(s, s->preset + 1, cols, rows);     /* auto-advance */
}

static void scene_next_preset(Scene *s, int cols, int rows)
{
    scene_load(s, s->preset + 1, cols, rows);
}

/* Theme persists across preset changes (it lives on the Scene). */
static void scene_cycle_theme(Scene *s, int dir)
{
    s->theme = (s->theme + dir + N_THEMES) % N_THEMES;
    theme_apply(s->theme);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    scene_load(s, s->preset, cols, rows);
}

/* ===================================================================== */
/* §8  render — field → glyphs (READ-ONLY)                                */
/* ===================================================================== */

/*
 * Glyph per cell value (index = the cell's colour band): a density ramp from the
 * far halo to the body.  Index 0 (background) is never drawn.
 */
static const char CELL_GLYPH[6] = {
    /* 0 bg */ ' ',
    /* 1 INSIDE */ '*', /* 2 C2 far */ ',', /* 3 C3 */ '.', /* 4 C4 */ '+', /* 5 C5 near */ '#',
};

/* cell_is_hot() — the interior and the band hugging it are drawn bold so the body
 * glows against the cooler halo. */
static bool cell_is_hot(uint8_t cell) { return cell == COL_INSIDE || cell == COL_C5; }

/* render_field() — paint each computed cell below the top HUD rows.  Read-only. */
static void render_field(const Field *f, WINDOW *w)
{
    for (int row = 0; row < f->rows; row++) {
        for (int col = 0; col < f->cols; col++) {
            uint8_t cell = f->cell[row][col];
            if (cell == 0) continue;

            attr_t attr = COLOR_PAIR((int)cell) | (cell_is_hot(cell) ? A_BOLD : 0);
            wattron(w, attr);
            mvwaddch(w, HUD_TOP_ROWS + row, col, (chtype)(unsigned char)CELL_GLYPH[cell]);
            wattroff(w, attr);
        }
    }
}

/* ===================================================================== */
/* §9  screen — ncurses lifecycle + HUD                                   */
/* ===================================================================== */

/*
 * Screen — the terminal as a drawing surface: just its current size, cached from
 * ncurses (getmaxyx) at startup and after each resize so the hot path never
 * re-queries it.  The ncurses lifecycle this wraps — initscr, colour, resize,
 * teardown — follows Gookin (2007).
 */
typedef struct {
    int cols;   /* terminal width  in character columns */
    int rows;   /* terminal height in character rows     */
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

/* hud_line() — one HUD line at (row, x) in `pair`, clamped to the terminal width
 * so a long line is truncated rather than wrapping onto the set. */
static void hud_line(int row, int x, int pair, attr_t attr, int cols, const char *str)
{
    if (x < 0) x = 0;
    if (x >= cols) return;
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(row, x, "%.*s", cols - x, str);
    attroff(COLOR_PAIR(pair) | attr);
}

/* hud_status() — row 0, right-aligned + bold: title, fps, render state. */
static void hud_status(const Screen *s, const Scene *sc, double fps)
{
    const char *state = sc->paused                  ? "PAUSED "
                      : reveal_complete(&sc->reveal) ? "done"
                      :                                "rendering";
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " Mandelbrot  %5.1f fps  %s ", fps, state);
    hud_line(0, s->cols - (int)strlen(buf), COL_HUD, A_BOLD, s->cols, buf);
}

/* hud_params() — row 1, left: preset, build-up progress, speed, theme. */
static void hud_params(const Screen *s, const Scene *sc, int sim_fps)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %s  %d%%  spd:%d Hz  theme:%s ",
             k_presets[sc->preset].name, reveal_progress_pct(&sc->reveal),
             sim_fps, k_themes[sc->theme].name);
    hud_line(1, 0, COL_HUD, A_NORMAL, s->cols, buf);
}

/* hud_keys() — bottom row: every interactive key. */
static void hud_keys(const Screen *s)
{
    hud_line(s->rows - 1, 0, COL_HINT, A_BOLD, s->cols,
             " q:quit  r/n:next  t/T:theme  [ / ]:speed  spc:pause ");
}

/*
 * HUD — data on top, actions on the bottom:
 *   row 0      (yellow bold,  right)  title, fps, render state
 *   row 1      (yellow plain, left)   preset, progress %, speed, theme
 *   row rows-1 (cyan bold,    left)   every interactive key
 */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps)
{
    erase();
    render_field(&sc->field, stdscr);
    hud_status(s, sc, fps);
    hud_params(s, sc, sim_fps);
    hud_keys(s);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §10  app — main loop                                                   */
/* ===================================================================== */

/*
 * App — the top-level program state: the picture (scene), the surface it draws to
 * (screen), the one tunable outside the Scene (tick rate), and the two signal
 * flags.  Everything user-facing is reached through this one object.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;       /* tick rate in Hz, SIM_FPS_MIN..MAX ([/]) */
    volatile sig_atomic_t running;       /* main-loop flag; cleared by SIGINT/TERM  */
    volatile sig_atomic_t need_resize;   /* set by SIGWINCH, serviced next frame    */
} App;

/*
 * The one global: signal handlers receive only an int, so the flags they touch
 * must be reachable without a parameter (volatile sig_atomic_t — the only kind a
 * handler may portably write and the loop read).  Everything else is by pointer.
 */
static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s   = &app->scene;
    int    cols = app->screen.cols, rows = app->screen.rows;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case 'r': case 'R': case 'n': case 'N':
        scene_next_preset(s, cols, rows);
        break;

    case 't': scene_cycle_theme(s, +1); break;
    case 'T': scene_cycle_theme(s, -1); break;

    case 'p': case 'P': case ' ':
        s->paused = !s->paused;
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
    srand((unsigned int)clock_ns());   /* seeds the reveal shuffle */

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    FrameClock clock;
    frameclock_init(&clock);

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frameclock_init(&clock);     /* fresh timing after the resize */
        }

        frameclock_begin_frame(&clock);

        int64_t tick_ns = TICK_NS(app->sim_fps);
        while (frameclock_step_due(&clock, tick_ns))
            scene_tick(&app->scene, app->screen.cols, app->screen.rows);

        frameclock_throttle(&clock);

        screen_draw(&app->screen, &app->scene, clock.fps, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
