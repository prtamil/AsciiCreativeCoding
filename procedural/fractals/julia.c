/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * julia.c — a Julia set fractal that draws itself in, one scattered dot at a
 * time. Each screen cell is a point on a plane; we repeatedly square it and add
 * a fixed number, and colour it by how fast it flies off to infinity. Six
 * preset shapes cycle on their own; q/ESC quits, n next shape, t theme,
 * spc pause, [ / ] slower/faster.
 *
 * Same engine as mandelbrot.c, but here the added number is fixed and the
 * starting point changes per cell (Mandelbrot is the other way round).
 * julia_explorer.c lets you drag that fixed number around live. Julia (1918);
 * the connected-when-c-is-in-the-Mandelbrot-set fact is Douady & Hubbard's
 * Orsay Notes (1984).
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

    PIXELS_PER_TICK  =  60,    /* how many cells we fill in each step    */
    MAX_ITER         = 128,    /* give up after this many tries per cell  */
    DONE_PAUSE_TICKS =  90,    /* hold the finished picture ~3 s          */
    N_PRESETS        =   6,    /* preset shapes                          */
    N_THEMES         =   5,    /* colour themes, cycled with t/T         */
    N_COLORS         =   5,    /* paintable slots: the body + 4 bands    */

    CANVAS_ROWS_MAX  =  80,
    CANVAS_COLS_MAX  = 300,

    RENDER_FPS       =  60,    /* how often we redraw (not the fill rate) */
    MAX_FRAME_MS     = 100,    /* longest frame we'll believe in          */
    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,    /* how often the fps number refreshes      */
};

/* A point "escapes" once it gets more than 2 away from the centre. We compare
 * the squared distance to 4 so we never need a square root. */
#define ESCAPE_RADIUS_SQ  4.0f

/* Points that fly off almost immediately are far from anything interesting;
 * leave them blank so the screen isn't just a fog of dots. */
#define BACKGROUND_FRAC   0.12f

/* How tall a slice of the plane fills the screen, top to bottom. The width is
 * worked out from this and the terminal shape. */
#define VIEW_HALF_IM      1.3f

/* Terminal characters are about twice as tall as they are wide. We stretch the
 * horizontal span to match so the fractal isn't squashed. */
#define ASPECT_R    2.0f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * ColorID — the colour a cell can be painted.
 *
 * The whole idea: we colour each cell by how long it took to fly off to
 * infinity. Points that never leave are the body of the shape; points that
 * leave slowly are right on the lacy edge (bright); points that leave fast are
 * far from it (dim). Each value here is used two ways at once — as the ncurses
 * colour-pair number, and as the byte we store in a canvas cell. 0 means a
 * blank, unpainted cell.
 */
typedef enum {
    COL_INSIDE = 1,   /* part of the shape — never flew off       */
    COL_C2     = 2,   /* flew off fastest — furthest from the edge */
    COL_C3     = 3,
    COL_C4     = 4,
    COL_C5     = 5,   /* flew off slowest — right on the edge      */
    COL_HUD    = 6,   /* HUD info lines, yellow                    */
    COL_HINT   = 7,   /* HUD key bar, cyan                         */
} ColorID;

/*
 * Theme — a named set of five colours: the body plus the four edge bands,
 * cycled with t / T.
 *
 * The fractal maths only ever decides which of the five slots a cell belongs
 * to. A theme is the lookup that turns those slot numbers into actual colours,
 * so switching themes restyles the whole picture instantly without redoing any
 * maths. Every colour sits in the bright half of the palette on purpose: dark
 * colours vanish against the black background.
 *
 * The arrays run dim to bright: slot 0 is the body, then the four bands from
 * far-and-dim to edge-and-bright.
 */
typedef struct {
    const char *name;            /* shown in the HUD, e.g. "Ocean"           */
    int fg256[N_COLORS];         /* the five colours on a 256-colour terminal */
    int fg8[N_COLORS];           /* same five, for plain 8-colour terminals   */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*            name      INSIDE  C2   C3   C4   C5      8-colour fallback (same order) */
    { "Fire",   {231, 160, 196, 208, 226}, { COLOR_WHITE,   COLOR_RED,     COLOR_RED,     COLOR_YELLOW, COLOR_YELLOW } },
    { "Ocean",  {231,  31,  38,  45,  87}, { COLOR_WHITE,   COLOR_BLUE,    COLOR_CYAN,    COLOR_CYAN,   COLOR_CYAN   } },
    { "Toxic",  {190,  46,  82, 154, 226}, { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW, COLOR_YELLOW } },
    { "Neon",   {201, 165,  51,  87, 231}, { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_CYAN,    COLOR_CYAN,   COLOR_WHITE  } },
    { "Mono",   {231, 243, 246, 249, 252}, { COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE,  COLOR_WHITE  } },
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

/* If the program freezes (laptop sleep, a debugger), the next frame looks
 * enormously long. Cap it so we don't suddenly try to catch up all at once. */
static int64_t clamp_frame_dt(int64_t dt)
{
    int64_t cap = MAX_FRAME_MS * NS_PER_MS;
    return dt > cap ? cap : dt;
}

/* Sleep just long enough that the whole frame lasts 1/target_fps. We already
 * spent some of that budget working, so only nap for what's left over. */
static void frame_pace(int target_fps, int64_t frame_start, int64_t work_done)
{
    int64_t budget = TICK_NS(target_fps);
    int64_t spent  = clock_ns() - frame_start + work_done;
    clock_sleep_ns(budget - spent);
}

/*
 * FpsCounter — the frames-per-second number shown in the HUD.
 *
 * Measuring one frame at a time gives a jumpy number, so we tally up frames and
 * time over half a second and divide once for a steady reading.
 */
typedef struct {
    int64_t accum_ns;   /* time piled up since the last reading  */
    int     frames;     /* frames counted in that time           */
    double  value;      /* the steady number the HUD shows       */
} FpsCounter;

static void fps_count_frame(FpsCounter *f, int64_t dt)
{
    f->frames   += 1;
    f->accum_ns += dt;
    if (f->accum_ns >= FPS_UPDATE_MS * NS_PER_MS) {       /* enough piled up — update */
        f->value    = (double)f->frames / ((double)f->accum_ns / (double)NS_PER_SEC);
        f->frames   = 0;
        f->accum_ns = 0;
    }
}

/* ===================================================================== */
/* §3  complex — the kind of number the fractal is built on               */
/* ===================================================================== */

/*
 * Complex — a single point on a flat 2-D plane, with an x part and a y part.
 *
 * What makes it special is one trick: "squaring" such a point spins it around
 * the centre and scales it. The fractal comes from doing that over and over and
 * adding a fixed point each time — some starting points stay put near the
 * centre, others race off to infinity, and the border between the two fates is
 * the lacy shape we draw.
 *
 * Here, every screen cell is one of these points, and so is the fixed number we
 * add. We use float (not double): the picture only spans a few units, so the
 * extra precision isn't worth the speed it costs over millions of steps.
 */
typedef struct {
    float re;   /* the x part (horizontal) */
    float im;   /* the y part (vertical)   */
} Complex;

static inline Complex complex_add(Complex a, Complex b)
{
    return (Complex){ a.re + b.re, a.im + b.im };
}

/* Squaring a complex point — this is the spin-and-scale move the fractal runs on. */
static inline Complex complex_square(Complex z)
{
    return (Complex){ z.re * z.re - z.im * z.im, 2.0f * z.re * z.im };
}

/* How far the point sits from the centre, squared (so we skip the square root). */
static inline float complex_norm_sq(Complex z)
{
    return z.re * z.re + z.im * z.im;
}

/* ===================================================================== */
/* §4  fractal — the escape-time algorithm                                */
/* ===================================================================== */

/*
 * Count how many "square it, then add c" steps a starting point survives before
 * it races off past distance 2. Returns MAX_ITER if it never does — that point
 * is part of the shape.
 */
static int julia_escape(Complex z, Complex c)
{
    for (int n = 0; n < MAX_ITER; n++) {
        z = complex_add(complex_square(z), c);
        if (complex_norm_sq(z) > ESCAPE_RADIUS_SQ)
            return n;
    }
    return MAX_ITER;
}

/*
 * Turn that escape count into a colour slot: never escaped is the body, the
 * quickest escapers stay blank, and everything between gets one of the four
 * edge bands — the slower it escaped, the closer to the bright edge.
 */
static uint8_t escape_to_band(int escape)
{
    if (escape >= MAX_ITER) return COL_INSIDE;

    float frac = (float)escape / (float)MAX_ITER;   /* 0 = instant, 1 = barely held on */
    if (frac < BACKGROUND_FRAC) return 0;

    /* spread the surviving range across the four bands */
    float halo_pos = (frac - BACKGROUND_FRAC) / (1.0f - BACKGROUND_FRAC);
    int   n_bands  = COL_C5 - COL_C2 + 1;
    int   band     = COL_C2 + (int)(halo_pos * (float)n_bands);
    return (uint8_t)(band > COL_C5 ? COL_C5 : band);
}

/* ===================================================================== */
/* §5  view — the fixed number, and the window onto the plane             */
/* ===================================================================== */

/*
 * Preset — one named shape, defined entirely by the fixed number we add.
 *
 * There isn't a single Julia set — there's a different one for every choice of
 * that fixed number. Pick one, ask of every starting point "does it stay put or
 * race off?", and the points that stay form the shape. Change the number and
 * the whole shape changes, which is why a preset is just a name plus that one
 * number.
 *
 * These six numbers were chosen because they each land near the edge of the
 * Mandelbrot set, where the shapes are connected and at their laciest — the
 * famous rabbit, dendrite, seahorse, and so on.
 */
typedef struct {
    const char *name;   /* shown in the HUD, e.g. "seahorse" */
    Complex     c;      /* the fixed number we add each step */
} Preset;

static const Preset k_presets[N_PRESETS] = {
    { "douady rabbit", { -0.7000f,  0.2702f } },
    { "spiral galaxy", {  0.2850f,  0.0100f } },
    { "dendrite",      { -0.8000f,  0.1560f } },
    { "flame",         { -0.4000f,  0.6000f } },
    { "seahorse",      { -0.7269f,  0.1889f } },
    { "basilica",      { -0.1010f,  0.6510f } },
};

/*
 * ViewWindow — which patch of the endless plane fills the screen.
 *
 * The plane goes on forever; the terminal is a fixed grid. This is the bridge:
 * the point shown at screen centre, plus how far the view reaches each way. The
 * horizontal and vertical reaches differ because terminal characters are about
 * twice as tall as wide — without that stretch the shape would look squashed.
 */
typedef struct {
    Complex center;    /* the point shown at screen centre (always the origin) */
    float   half_re;   /* how far the view reaches left/right (stretched)      */
    float   half_im;   /* how far the view reaches up/down                     */
} ViewWindow;

/* Size the window to the current terminal (same window for every preset). */
static ViewWindow view_for_screen(int cols, int rows)
{
    ViewWindow v;
    v.center  = (Complex){ 0.0f, 0.0f };
    v.half_im = VIEW_HALF_IM;
    v.half_re = VIEW_HALF_IM * (float)cols / (float)rows / ASPECT_R;
    return v;
}

/*
 * Which point on the plane does screen cell (col, row) stand for? This is the
 * one place screen positions become plane points. The vertical axis is flipped
 * because screen rows grow downward but the plane's y grows upward.
 */
static Complex view_sample(const ViewWindow *v, int col, int row, int cols, int rows)
{
    float fx = (float)col / (float)(cols - 1);   /* 0 at left edge, 1 at right */
    float fy = (float)row / (float)(rows - 1);   /* 0 at top,       1 at bottom */

    float off_re =  (fx - 0.5f) * 2.0f;
    float off_im = -(fy - 0.5f) * 2.0f;          /* flip so the top of screen is up */

    return (Complex){
        .re = v->center.re + off_re * v->half_re,
        .im = v->center.im + off_im * v->half_im,
    };
}

/* ===================================================================== */
/* §6  canvas — the grid of coloured cells we paint to the terminal       */
/* ===================================================================== */

/*
 * Canvas — the picture held in memory, one colour slot per terminal cell.
 *
 * Keeping a buffer between the maths and the screen means the fractal code only
 * stores slot numbers and never touches ncurses; one function (canvas_paint)
 * turns those numbers into characters. Either side can change without disturbing
 * the other.
 *
 * A cell holds 0 if it hasn't been worked out yet (or is blank background), or
 * 1..5 for a colour slot. The arrays are fixed-size so we never allocate while
 * running; rows/cols say how much of them this terminal actually uses.
 */
typedef struct {
    int     rows, cols;                              /* part of the grid in use */
    uint8_t band[CANVAS_ROWS_MAX][CANVAS_COLS_MAX];  /* each cell's colour slot */
} Canvas;

static void canvas_reset(Canvas *cv, int cols, int rows)
{
    if (cols > CANVAS_COLS_MAX) cols = CANVAS_COLS_MAX;
    if (rows > CANVAS_ROWS_MAX) rows = CANVAS_ROWS_MAX;
    cv->cols = cols;
    cv->rows = rows;
    memset(cv->band, 0, sizeof cv->band);
}

/*
 * The character drawn for each colour slot. It's a bit uneven (inherited from
 * mandelbrot.c — the body shows as ',' and one band is just blank), but colour
 * does the real work here; the character only adds a little texture.
 */
static chtype band_glyph(uint8_t slot)
{
    static const char glyph[8] = " ,. +#*";
    return (chtype)(unsigned char)glyph[slot < 7 ? slot : 0];
}

/* The colour and styling for a slot: its colour, plus bold for the body and the
 * bright edge band so they pop. */
static attr_t band_attr(uint8_t slot)
{
    attr_t attr = COLOR_PAIR((int)slot);
    if (slot == COL_INSIDE || slot == COL_C5) attr |= A_BOLD;
    return attr;
}

static void canvas_paint(const Canvas *cv, WINDOW *w)
{
    for (int row = 0; row < cv->rows; row++) {
        for (int col = 0; col < cv->cols; col++) {
            uint8_t slot = cv->band[row][col];
            if (slot == 0) continue;                 /* blank — skip it */
            attr_t attr = band_attr(slot);
            wattron(w, attr);
            mvwaddch(w, row, col, band_glyph(slot));
            wattroff(w, attr);
        }
    }
}

/* ===================================================================== */
/* §7  reveal — the random-fill animation                                 */
/* ===================================================================== */

/*
 * Reveal — the order cells get filled in, purely for looks: the picture
 * condenses out of scattered dots instead of wiping in line by line.
 *
 * order[] is a list of every cell, shuffled into random order; `done` marks how
 * far through that list we've gotten, advancing a batch at a time. Each entry is
 * a cell's position folded into a single number (row * cols + col), unfolded
 * again when we need its row and column.
 */
typedef struct {
    int order[CANVAS_ROWS_MAX * CANVAS_COLS_MAX];  /* every cell, in shuffled order */
    int total;   /* how many cells there are to fill */
    int done;    /* how many we've filled so far     */
} Reveal;

/* Shuffle the list into a fair random order, in place (the classic
 * Fisher-Yates shuffle). */
static void fisher_yates_shuffle(int *a, int n)
{
    for (int i = n - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int tmp = a[i]; a[i] = a[j]; a[j] = tmp;
    }
}

static void reveal_shuffle(Reveal *r, int n_pixels)
{
    r->total = n_pixels;
    r->done  = 0;
    for (int i = 0; i < n_pixels; i++)            /* list every cell, in order */
        r->order[i] = i;
    fisher_yates_shuffle(r->order, n_pixels);     /* then jumble it up         */
}

static bool reveal_complete(const Reveal *r) { return r->done >= r->total; }

static int reveal_percent(const Reveal *r)
{
    return r->total ? (int)((long)r->done * 100 / r->total) : 100;
}

/* ===================================================================== */
/* §8  color — hand a Theme's colours to ncurses                          */
/* ===================================================================== */

/* Point the five paintable slots at the chosen theme's colours. */
static void theme_apply(int theme)
{
    const Theme *t = &k_themes[theme];
    bool truecolor = COLORS >= 256;
    for (int i = 0; i < N_COLORS; i++)
        init_pair(COL_INSIDE + i,
                  truecolor ? t->fg256[i] : t->fg8[i], COLOR_BLACK);
}

static void color_init(void)
{
    start_color();
    /* The HUD colours never change with the theme — yellow info, cyan keys. */
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);
        init_pair(COL_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    theme_apply(0);   /* start on Fire */
}

/* ===================================================================== */
/* §9  scene — everything needed to render one Julia view                 */
/* ===================================================================== */

/*
 * Scene — everything that's on screen, in one place: which shape and which
 * theme are showing, the window onto the plane, the painted canvas, and how far
 * the fill has gotten. Everything the program does comes down to: update the
 * scene, then draw it.
 *
 * preset_id and theme are just positions in the preset/theme tables, not
 * pointers — easy to step to the next one, easy to print in the HUD, and they
 * can never go stale.
 */
typedef struct {
    int        preset_id;   /* which preset shape is showing      */
    int        theme;       /* which theme is showing             */
    ViewWindow view;        /* the patch of plane we're drawing   */
    Canvas     canvas;      /* the painted picture                */
    Reveal     reveal;      /* how far the fill has gotten        */
    int        hold_ticks;  /* how long we've held a finished one */
    bool       paused;      /* true freezes the fill              */
} Scene;

/* Switch the scene to a preset: resize the window, wipe the canvas, reshuffle. */
static void scene_load(Scene *s, int preset_id, int cols, int rows)
{
    s->preset_id  = preset_id % N_PRESETS;
    s->view       = view_for_screen(cols, rows);
    s->hold_ticks = 0;
    canvas_reset(&s->canvas, cols, rows);
    reveal_shuffle(&s->reveal, s->canvas.rows * s->canvas.cols);
}

static void scene_init(Scene *s, int cols, int rows)
{
    s->theme  = 0;
    s->paused = false;
    scene_load(s, 0, cols, rows);
}

static void scene_next_preset(Scene *s, int cols, int rows)
{
    scene_load(s, (s->preset_id + 1) % N_PRESETS, cols, rows);
}

static void scene_next_theme(Scene *s)
{
    s->theme = (s->theme + 1) % N_THEMES;
    theme_apply(s->theme);
}

/*
 * Work out and store the colour of one canvas cell: find its row and column,
 * find the plane point it stands for, run the escape count, and record the
 * resulting colour (leaving plain-background cells blank).
 */
static void scene_color_pixel(Scene *s, int flat_index)
{
    int col = flat_index % s->canvas.cols;
    int row = flat_index / s->canvas.cols;

    Complex z0   = view_sample(&s->view, col, row, s->canvas.cols, s->canvas.rows);
    Complex c    = k_presets[s->preset_id].c;
    uint8_t band = escape_to_band(julia_escape(z0, c));

    if (band) s->canvas.band[row][col] = band;
}

/* Fill in the next batch of cells from the shuffled list — the actual work of
 * growing the picture. */
static void scene_reveal_batch(Scene *s)
{
    int batch_end = s->reveal.done + PIXELS_PER_TICK;
    if (batch_end > s->reveal.total) batch_end = s->reveal.total;

    for (int i = s->reveal.done; i < batch_end; i++)
        scene_color_pixel(s, s->reveal.order[i]);

    s->reveal.done = batch_end;
}

/*
 * Move the picture forward one step. If it's still filling, draw the next batch;
 * if it's done, hold it a few seconds and then move on to the next preset.
 */
static void scene_tick(Scene *s)
{
    if (s->paused) return;

    if (reveal_complete(&s->reveal)) {                /* done — hold, then move on */
        if (++s->hold_ticks >= DONE_PAUSE_TICKS)
            scene_next_preset(s, s->canvas.cols, s->canvas.rows);
        return;
    }
    scene_reveal_batch(s);                             /* still filling */
}

static void scene_draw(const Scene *s, WINDOW *w) { canvas_paint(&s->canvas, w); }

/* ===================================================================== */
/* §10  screen — ncurses lifecycle + HUD                                  */
/* ===================================================================== */

/*
 * Screen — the terminal's current size in characters. Read at startup and again
 * whenever the window is resized; the view and the HUD both lay themselves out
 * against it.
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
 * Draw one HUD line, trimmed to fit so a long line can't spill over and wrap
 * onto the fractal.
 */
static void hud_line(int row, int x, int pair, attr_t bold, int cols, const char *str)
{
    if (x < 0) x = 0;
    if (x >= cols) return;
    attron(COLOR_PAIR(pair) | bold);
    mvprintw(row, x, "%.*s", cols - x, str);
    attroff(COLOR_PAIR(pair) | bold);
}

/* Top row, right-aligned: title, fps, and whether it's drawing/done/paused. */
static void hud_data_line(const Screen *s, const Scene *sc, double fps)
{
    const char *state = sc->paused ? "PAUSED "
                      : (reveal_complete(&sc->reveal) ? "complete" : "drawing ");
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " Julia  %5.1f fps  %s ", fps, state);
    hud_line(0, s->cols - (int)strlen(buf), COL_HUD, A_BOLD, s->cols, buf);
}

/* hud_params_line — row 1, plain: preset name+position, theme, %, speed. */
static void hud_params_line(const Screen *s, const Scene *sc, int sim_fps)
{
    char tag[32], buf[HUD_COLS + 1];
    snprintf(tag, sizeof tag, "%s %d/%d",
             k_presets[sc->preset_id].name, sc->preset_id + 1, N_PRESETS);
    snprintf(buf, sizeof buf, " %-18s  theme:%s  %d%%  spd:%d Hz ",
             tag, k_themes[sc->theme].name, reveal_percent(&sc->reveal), sim_fps);
    hud_line(1, 0, COL_HUD, A_NORMAL, s->cols, buf);
}

/* hud_action_line — last row, bright cyan: every interactive key. */
static void hud_action_line(const Screen *s)
{
    hud_line(s->rows - 1, 0, COL_HINT, A_BOLD, s->cols,
             " q:quit  n:next preset  t:theme  spc:pause  [ / ]:speed ");
}

/*
 * screen_draw — compose one frame: the fractal, then the HUD.
 * HUD layout is fixed — top is data, bottom is actions:
 *   row 0      title + fps + state    (hud_data_line)
 *   row 1      preset/theme/%/speed   (hud_params_line)
 *   row rows-1 key actions            (hud_action_line)
 * Every line is clamped to the terminal width by hud_line.
 */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr);          /* the fractal itself     */
    hud_data_line(s, sc, fps);       /* row 0   — live data    */
    hud_params_line(s, sc, sim_fps); /* row 1   — parameters   */
    hud_action_line(s);              /* last row — actions     */
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §11  app — signals, input, main loop                                   */
/* ===================================================================== */

/*
 * App — the top-level container main() owns: the picture (scene), the terminal
 * it lives in (screen), and the one tunable speed (sim_fps).  One struct so the
 * loop passes everything with a single &app, and it sits in BSS — it holds the
 * large canvas + shuffle arrays, which don't belong on the stack.
 */
typedef struct {
    Scene  scene;     /* the fractal picture + all its sub-state          */
    Screen screen;    /* current terminal dimensions                      */
    int    sim_fps;   /* reveal speed, ticks/sec (SIM_FPS_MIN..MAX); ]/[  */
} App;

/* The only globals: POSIX signal handlers take no arguments, so they must
 * write a flag the main loop polls. */
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
    scene_load(&a->scene, a->scene.preset_id, a->screen.cols, a->screen.rows);
    g_should_resize = 0;
}

/* app_change_speed — nudge the reveal rate by delta, clamped to its range. */
static void app_change_speed(App *a, int delta)
{
    int fps = a->sim_fps + delta;
    if (fps < SIM_FPS_MIN) fps = SIM_FPS_MIN;
    if (fps > SIM_FPS_MAX) fps = SIM_FPS_MAX;
    a->sim_fps = fps;
}

/* app_handle_key — map one keypress to an action.  Returns false to quit. */
static bool app_handle_key(App *a, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27:  return false;                          /* quit    */
    case 'r': case 'R':
    case 'n': case 'N':  scene_next_preset(&a->scene,
                                  a->screen.cols, a->screen.rows);   break; /* next set */
    case 't': case 'T':  scene_next_theme(&a->scene);                break; /* palette */
    case 'p': case 'P':
    case ' ':            a->scene.paused = !a->scene.paused;         break; /* pause   */
    case ']':            app_change_speed(a, +SIM_FPS_STEP);         break; /* faster  */
    case '[':            app_change_speed(a, -SIM_FPS_STEP);         break; /* slower  */
    default: break;
    }
    return true;
}

/*
 * app_run_due_ticks — fixed-timestep catch-up.  Add the elapsed time to an
 * accumulator and spend one scene_tick per whole tick's worth, so the reveal
 * advances at sim_fps regardless of frame rate.
 */
static void app_run_due_ticks(App *a, int64_t *sim_accum, int64_t dt)
{
    int64_t tick_ns = TICK_NS(a->sim_fps);
    *sim_accum += dt;
    while (*sim_accum >= tick_ns) {
        scene_tick(&a->scene);
        *sim_accum -= tick_ns;
    }
}

/* app_poll_input — handle at most one pending key this frame. */
static void app_poll_input(App *a)
{
    int ch = getch();
    if (ch != ERR && !app_handle_key(a, ch))
        g_should_quit = 1;
}

int main(void)
{
    srand((unsigned int)clock_ns());

    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    /* static (BSS) — Scene holds large arrays; keep them off the stack */
    static App app = { .sim_fps = SIM_FPS_DEFAULT };

    screen_init(&app.screen);
    scene_init(&app.scene, app.screen.cols, app.screen.rows);

    int64_t    frame_time = clock_ns();
    int64_t    sim_accum  = 0;        /* elapsed time not yet spent on ticks */
    FpsCounter fps        = {0};

    /* Main loop — one pass per frame:
     *   1. apply a pending resize
     *   2. measure elapsed time, clamped against stalls
     *   3. run the simulation ticks that time owes
     *   4. update the fps readout
     *   5. pace the frame, then draw it
     *   6. handle one keypress
     */
    while (!g_should_quit) {

        if (g_should_resize) {
            app_resize(&app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = clamp_frame_dt(now - frame_time);
        frame_time  = now;

        app_run_due_ticks(&app, &sim_accum, dt);            /* advance the reveal     */
        fps_count_frame(&fps, dt);                          /* update the fps readout */

        frame_pace(RENDER_FPS, frame_time, dt);             /* cap the display rate   */
        screen_draw(&app.screen, &app.scene, fps.value, app.sim_fps);
        screen_present();

        app_poll_input(&app);                               /* one key → action/quit  */
    }

    screen_free(&app.screen);
    return 0;
}
