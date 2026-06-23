/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * burning_ship.c — the Burning Ship fractal, drawn in the terminal.
 *
 * Same idea as the Mandelbrot set: for each point on screen, repeat a little
 * math step over and over and watch how fast the value runs off to infinity;
 * colour the point by how long it took. The one twist is that we flip both
 * coordinates positive each step (the "fold"), which bends the Mandelbrot shape
 * into something that looks like a burning ship.
 *
 * The fractal: Michelitsch & Rössler (1992), "The 'Burning Ship' and its
 * quasi-Julia sets," Computers & Graphics 16(4):435-438.
 * Sister file: mandelbrot.c is the exact same engine without the fold — diff
 * the iterate step there against §3/§4 here to see what the ship changes.
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
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  30,
    SIM_FPS_MAX      =  60,
    SIM_FPS_STEP     =   5,

    PIXELS_PER_TICK  =  80,    /* how many points we colour in per step      */
    MAX_ITER         = 256,    /* give up after this many repeats per point  */
    N_PRESETS        =   5,
    N_THEMES         =   7,

    CANVAS_ROWS_MAX  =  80,    /* biggest terminal we make room for          */
    CANVAS_COLS_MAX  = 300,

    RENDER_FPS       =  60,    /* screen redraw rate (not the reveal speed)  */
    MAX_FRAME_MS     = 100,    /* longest a single frame may count as        */
    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,    /* refresh the fps number this often          */
};

/* Once the value's distance from the origin passes 2, it's gone for good. We
 * compare distance-squared to 4 instead, which dodges a slow square root in the
 * inner loop. */
#define ESCAPE_RADIUS_SQ  4.0f

/* Points that run away almost instantly (in the first 8% of the step budget)
 * are way out in empty space — we leave them blank so the picture isn't fuzz. */
#define BACKGROUND_FRAC   0.08f

/* Bright cyan for the key-hints line. It stays readable no matter which colour
 * theme is active. */
#define HINT_CYAN   51

/* Terminal characters are about twice as tall as they are wide. We stretch the
 * math horizontally by this factor so the ship doesn't come out squashed. */
#define ASPECT_R    2.0f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * ColorID — the handful of "colour slots" a screen cell can have.
 *
 * Each number pulls double duty: it's both the id we give ncurses for a colour,
 * and the single byte we store per cell to remember that cell's look. So one
 * small number tells us everything about how to draw a point.
 *
 * The order tracks how fast a point ran away to infinity, which is the whole
 * point of this kind of fractal: fast runaway = far from the shape = low number
 * = dim; slow runaway = hugging the shape's edge = high number = bright; never
 * ran away = inside the shape itself.
 *
 * Slot 0 is special: it means "blank — nothing drawn here yet."
 */
typedef enum {
    COL_INSIDE = 1,   /* inside the shape — never ran away (brightest)         */
    COL_C2     = 2,   /* deep background — ran away instantly (dimmest)        */
    COL_C3     = 3,   /* background      — ran away fast                       */
    COL_C4     = 4,   /* mid glow        — ran away slowly                     */
    COL_C5     = 5,   /* edge glow       — clung to the shape's rim (bright)   */
    COL_HUD    = 6,   /* the info text up top (not a fractal colour)           */
    COL_HINT   = 7,   /* the key-hints line, always cyan                       */
} ColorID;

/*
 * Theme — one named colour scheme (Fire, Ice, …) you flip through with 't'.
 *
 * The fractal math only ever decides which slot a point belongs to (inside,
 * edge glow, background, …). A Theme is just the lookup that turns those slots
 * into real colours, so changing the whole look is a matter of pointing at a
 * different row in the table below — the fractal never has to be recomputed.
 *
 * The colour arrays line up with the slots, dim to bright:
 *   c[0] → inside   c[1] → deep background … c[4] → edge glow.
 * c[] is for modern 256-colour terminals; c8[] is the same idea in the 8 old
 * standard colours, so the program still looks okay on an ancient terminal.
 *
 * Every colour is chosen from the bright half of the palette on purpose — the
 * darkest colours vanish against a black background. (See COLOR.md.)
 */
typedef struct {
    const char *name;   /* the name shown in the info bar, e.g. "Fire"    */
    int c[5];           /* the five fractal colours, inside → edge glow   */
    int c8[5];          /* same five for old 8-colour terminals           */
    int hud;            /* colour of the info text                        */
    int hud8;           /* same, for old 8-colour terminals               */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*              name      INSIDE  C2    C3    C4    C5    hud   (8-color fallback)                            hud8 */
    { "Fire",     {231, 160, 202, 214, 226},  {COLOR_WHITE,  COLOR_RED,    COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW}, 226, COLOR_YELLOW  },
    { "Ember",    {230, 130, 172, 208, 220},  {COLOR_YELLOW, COLOR_RED,    COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW}, 214, COLOR_YELLOW  },
    { "Magma",    {231, 124, 196, 209, 223},  {COLOR_WHITE,  COLOR_RED,    COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW}, 220, COLOR_YELLOW  },
    { "Ice",      {231,  24,  31,  39, 159},  {COLOR_WHITE,  COLOR_BLUE,   COLOR_BLUE,   COLOR_CYAN,   COLOR_CYAN},   123, COLOR_CYAN    },
    { "Toxic",    {190,  28,  64, 154, 226},  {COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_YELLOW, COLOR_YELLOW}, 154, COLOR_YELLOW  },
    { "Nebula",   {231,  93, 129, 171, 213},  {COLOR_WHITE,  COLOR_MAGENTA,COLOR_MAGENTA,COLOR_CYAN,   COLOR_CYAN},    87, COLOR_CYAN    },
    { "Mono",     {231, 243, 246, 249, 252},  {COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE},  255, COLOR_WHITE   },
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

/*
 * If the program freezes for a moment (laptop sleep, a slow terminal), the next
 * frame thinks a huge amount of time passed and tries to catch up all at once,
 * which can lock things up. We pretend no single frame lasted longer than this.
 */
static int64_t clamp_frame_dt(int64_t dt)
{
    int64_t cap = MAX_FRAME_MS * NS_PER_MS;
    return dt > cap ? cap : dt;
}

/*
 * Sleep just long enough that the whole frame lasts about 1/target_fps seconds.
 * We subtract the work already done so we wait only the leftover time, instead
 * of spinning the CPU.
 */
static void frame_pace(int target_fps, int64_t frame_start, int64_t work_done)
{
    int64_t budget = TICK_NS(target_fps);
    int64_t spent  = clock_ns() - frame_start + work_done;
    clock_sleep_ns(budget - spent);
}

/*
 * FpsCounter — works out the frames-per-second number for the info bar.
 *
 * A single frame's rate jumps around too much to read, so we tally up frames
 * and time and only divide them every so often for a steady, readable number.
 */
typedef struct {
    int64_t accum_ns;   /* time piled up since we last updated the number */
    int     frames;     /* frames counted in that same stretch            */
    double  value;      /* the steady fps number the info bar shows        */
} FpsCounter;

static void fps_count_frame(FpsCounter *f, int64_t dt)
{
    f->frames   += 1;
    f->accum_ns += dt;
    if (f->accum_ns >= FPS_UPDATE_MS * NS_PER_MS) {       /* time to recompute? */
        f->value    = (double)f->frames / ((double)f->accum_ns / (double)NS_PER_SEC);
        f->frames   = 0;
        f->accum_ns = 0;
    }
}

/* ===================================================================== */
/* §3  complex — the kind of number this fractal is built on              */
/* ===================================================================== */

/*
 * Complex — a 2-D point (re, im) that follows special add and multiply rules.
 *
 * Think of it as just a point on a flat plane, except it comes with its own
 * arithmetic (see complex_square below). Those rules are exactly what makes
 * fractals like this happen: repeat "square it, then add the starting point"
 * and the intricate edge appears.
 *
 * It's worth its own little type because every screen point IS one of these
 * numbers, and giving it named add/square/fold helpers lets the main loop in §4
 * read just like the math, instead of a mess of loose float variables.
 *
 * We use float, not double: ~7 digits of precision is plenty for how far these
 * preset views zoom in, and it's faster across millions of repeats. Zooming
 * much deeper than the presets would eventually need double.
 */
typedef struct {
    float re;   /* the across part — moves right as it grows */
    float im;   /* the up-down part — moves up as it grows   */
} Complex;

static inline Complex complex_add(Complex a, Complex b)
{
    return (Complex){ a.re + b.re, a.im + b.im };
}

/* Squaring a complex number isn't squaring each part — it has its own rule. */
static inline Complex complex_square(Complex z)
{
    return (Complex){ z.re * z.re - z.im * z.im, 2.0f * z.re * z.im };
}

/* The one move that turns Mandelbrot into the Burning Ship: force both parts
 * positive each step. That fold is the entire difference between the two. */
static inline Complex complex_abs_fold(Complex z)
{
    return (Complex){ fabsf(z.re), fabsf(z.im) };
}

/* How far the point is from the origin, squared (we skip the square root and
 * compare against ESCAPE_RADIUS_SQ instead). */
static inline float complex_norm_sq(Complex z)
{
    return z.re * z.re + z.im * z.im;
}

/* ===================================================================== */
/* §4  fractal — count how fast each point runs away                      */
/* ===================================================================== */

/*
 * Repeat the burning-ship step on point c and count how many times we can do it
 * before the value flies off past distance 2. A small count means c is far from
 * the shape; hitting the cap (MAX_ITER) means c never ran away — it's inside.
 */
static int burning_ship_escape(Complex c)
{
    Complex z = { 0.0f, 0.0f };
    for (int n = 0; n < MAX_ITER; n++) {
        z = complex_add(complex_square(complex_abs_fold(z)), c);
        if (complex_norm_sq(z) > ESCAPE_RADIUS_SQ)
            return n;
    }
    return MAX_ITER;
}

/*
 * Turn a runaway count into a colour slot: never ran away → inside; ran away
 * almost instantly → blank background; everything in between → one of the four
 * glow colours, with slower runaways (closer to the shape) getting brighter.
 */
static uint8_t escape_to_band(int escape)
{
    if (escape >= MAX_ITER) return COL_INSIDE;          /* never ran away — inside */

    float frac = (float)escape / (float)MAX_ITER;       /* 0 = instant … 1 = slow  */
    if (frac < BACKGROUND_FRAC) return 0;               /* way out in space → blank */

    /* Spread the surviving range across the four glow colours, fast → slow. */
    float halo_pos = (frac - BACKGROUND_FRAC) / (1.0f - BACKGROUND_FRAC);
    int   n_bands  = COL_C5 - COL_C2 + 1;
    int   band     = COL_C2 + (int)(halo_pos * (float)n_bands);
    return (uint8_t)(band > COL_C5 ? COL_C5 : band);
}

/* ===================================================================== */
/* §5  view — translate between screen cells and points on the plane      */
/* ===================================================================== */

/*
 * Preset — one saved "place to look": where to aim and how far to zoom in.
 * The k_presets table below is a little guided tour of the fractal.
 *
 * We store a centre and a zoom amount rather than the four corners, because
 * that's how zooming actually feels: keep the centre, shrink the view, and you
 * dive in. Corners would have to be recomputed every time.
 *
 * Only the vertical zoom is stored; the horizontal half is worked out at run
 * time from the live terminal size, so the picture never looks squashed when
 * the window is a different shape or gets resized.
 *
 * The numbers were found by scanning renders ahead of time, not guessed — each
 * one lands on a detailed patch of the shape's edge. The famous ship sits near
 * -1.755 - 0.035i; the flames and distant fleet are too sparse to fill a
 * terminal, so the tour dives into the hull instead.
 */
typedef struct {
    const char *name;       /* the name shown in the info bar, e.g. "the ship" */
    float       center_re;  /* the point sitting at screen centre, across part */
    float       center_im;  /* same point, up-down part                        */
    float       im_half;    /* the zoom: smaller = deeper (1.0 = whole fractal) */
} Preset;

static const Preset k_presets[N_PRESETS] = {
    { "full armada",   -0.65000f, -0.45000f, 1.00000f },
    { "the ship",      -1.75500f, -0.03500f, 0.04500f },
    { "hull & masts",  -1.77000f, -0.02200f, 0.01800f },
    { "mini-armada",   -1.75600f, -0.02650f, 0.01000f },
    { "antenna sweep", -1.75650f, -0.02650f, 0.00500f },
};

/*
 * ViewWindow — the exact rectangle of the plane we're drawing right now.
 *
 * A Preset is the saved recipe; a ViewWindow is what you get once you mix in
 * the current terminal size — its width is already stretched to keep the
 * picture from looking squashed. view_from_preset does that conversion.
 *
 * This is the single bridge between two worlds: screen cells (whole-number row
 * and column) and points on the plane (floats). Keeping that conversion in one
 * place (view_sample) means the fractal math never has to know about terminals.
 */
typedef struct {
    Complex center;    /* the point shown at the middle of the screen        */
    float   half_re;   /* half the view's width  (stretched for cell shape)  */
    float   half_im;   /* half the view's height (the preset's zoom)         */
} ViewWindow;

/*
 * Work out how wide the view should be so it doesn't look squashed. We start
 * from the height and stretch it by the window's shape and by the fact that
 * terminal characters are about twice as tall as wide, so a step across covers
 * the same visual distance as a step down.
 */
static float aspect_correct_half_re(float half_im, int cols, int rows)
{
    return half_im * (float)cols / (float)rows / ASPECT_R;
}

static ViewWindow view_from_preset(const Preset *p, int cols, int rows)
{
    ViewWindow v;
    v.center  = (Complex){ p->center_re, p->center_im };
    v.half_im = p->im_half;
    v.half_re = aspect_correct_half_re(p->im_half, cols, rows);
    return v;
}

/*
 * Which point on the plane does screen cell (col, row) stand for?
 *
 * Turn each axis into a distance from the centre between -1 and +1, scale by the
 * view's half-size, and add the centre. The vertical one is flipped so going
 * down the screen goes down the plane — without that flip the ship would hang
 * upside-down, since screen rows count downward but the plane counts up.
 */
static Complex view_sample(const ViewWindow *v, int col, int row, int cols, int rows)
{
    float fx = (float)col / (float)(cols - 1);   /* 0 → 1 left to right */
    float fy = (float)row / (float)(rows - 1);   /* 0 → 1 top to bottom */

    float off_re =  (fx - 0.5f) * 2.0f;          /* -1 at left edge  … +1 at right  */
    float off_im = -(fy - 0.5f) * 2.0f;          /* +1 at top edge … -1 at bottom (flip) */

    return (Complex){
        .re = v->center.re + off_re * v->half_re,
        .im = v->center.im + off_im * v->half_im,
    };
}

/* ===================================================================== */
/* §6  canvas — the grid of coloured cells we draw to the terminal        */
/* ===================================================================== */

/*
 * Canvas — the picture kept in memory: one colour slot per terminal cell.
 *
 * Keeping a buffer keeps two jobs apart: deciding a colour (the fractal math)
 * and putting it on screen (ncurses). The math only ever stores a number in
 * band[row][col]; canvas_paint is the single place that turns those numbers
 * into characters. Either half can change without touching the other.
 *
 * One byte per cell is plenty (slots only go 0..7), which keeps it compact.
 * 0 means blank / not computed; 1..5 are the fractal colours.
 *
 * The arrays are a fixed size because this program never allocates memory while
 * running (see CLAUDE.md). They're sized to the biggest terminal we handle; the
 * live rows/cols say how much is actually in use.
 */
typedef struct {
    int     rows, cols;                              /* how much is in use   */
    uint8_t band[CANVAS_ROWS_MAX][CANVAS_COLS_MAX];  /* colour slot per cell */
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
 * The character drawn for each colour slot. They get visually heavier from the
 * faint background (',') toward the solid inside ('*'), so the shape of the
 * character and its colour brightness pull in the same direction.
 */
static chtype band_glyph(uint8_t slot)
{
    static const char glyph[8] = {
        [COL_INSIDE] = '*',
        [COL_C2]     = ',',
        [COL_C3]     = '.',
        [COL_C4]     = '+',
        [COL_C5]     = '#',
    };
    return (chtype)(unsigned char)glyph[slot];
}

/*
 * How to colour each slot: its colour pair, plus bold on the inside and the
 * bright edge glow so those stand out against the dimmer parts.
 */
static attr_t band_attr(uint8_t slot)
{
    attr_t attr = COLOR_PAIR((int)slot);
    if (slot == COL_INSIDE || slot == COL_C5) attr |= A_BOLD;
    return attr;
}

/* Draw every cell that isn't blank. */
static void canvas_paint(const Canvas *cv, WINDOW *w)
{
    for (int row = 0; row < cv->rows; row++) {
        for (int col = 0; col < cv->cols; col++) {
            uint8_t slot = cv->band[row][col];
            if (slot == 0) continue;                 /* blank / not computed yet */
            attr_t attr = band_attr(slot);
            wattron(w, attr);
            mvwaddch(w, row, col, band_glyph(slot));
            wattroff(w, attr);
        }
    }
}

/* ===================================================================== */
/* §7  reveal — fill the picture in a scattered, random order             */
/* ===================================================================== */

/*
 * Reveal — picks the ORDER cells get filled in, purely for looks: the fractal
 * speckles into view from scattered dots instead of wiping top to bottom.
 *
 * order[] lists every cell once, then gets shuffled so the dots land randomly.
 * To fit the 2-D grid into one list we flatten each cell into a single number,
 * row*cols + col, and unflatten it later (col = n % cols, row = n / cols; see
 * scene_color_pixel).
 *
 * `done` is how many cells from the front of the list we've coloured so far;
 * each step nudges it forward until it reaches `total`.
 */
typedef struct {
    int order[CANVAS_ROWS_MAX * CANVAS_COLS_MAX];  /* every cell, shuffled  */
    int total;   /* how many cells this view has (rows * cols)              */
    int done;    /* how many we've coloured so far                          */
} Reveal;

/*
 * Shuffle the list so every possible order is equally likely. This is the
 * Fisher-Yates shuffle: walk from the back, and swap each item with a randomly
 * picked item at or before it. (Knuth, TAOCP Vol. 2, §3.4.2, "Algorithm P".)
 */
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
    for (int i = 0; i < n_pixels; i++)
        r->order[i] = i;
    fisher_yates_shuffle(r->order, n_pixels);
}

static bool reveal_complete(const Reveal *r) { return r->done >= r->total; }

static int reveal_percent(const Reveal *r)
{
    return r->total ? (int)((long)r->done * 100 / r->total) : 100;
}

/* ===================================================================== */
/* §8  color — hand a Theme's colours to ncurses                          */
/* ===================================================================== */

/*
 * Tell ncurses what colour each of the five fractal slots and the info text
 * should be, all on a black background. Called with either the modern or the
 * old-terminal colour list, so this wiring only has to be written once.
 */
static void bind_palette(const int band_fg[5], int hud_fg)
{
    init_pair(COL_INSIDE, band_fg[0], COLOR_BLACK);
    init_pair(COL_C2,     band_fg[1], COLOR_BLACK);
    init_pair(COL_C3,     band_fg[2], COLOR_BLACK);
    init_pair(COL_C4,     band_fg[3], COLOR_BLACK);
    init_pair(COL_C5,     band_fg[4], COLOR_BLACK);
    init_pair(COL_HUD,    hud_fg,     COLOR_BLACK);
}

static void theme_apply(int theme_id)
{
    const Theme *th = &k_themes[theme_id];
    bool truecolor  = COLORS >= 256;                       /* modern terminal? */

    bind_palette(truecolor ? th->c   : th->c8,
                 truecolor ? th->hud : th->hud8);
    init_pair(COL_HINT, truecolor ? HINT_CYAN : COLOR_CYAN, COLOR_BLACK);
}

static void color_init(void)
{
    start_color();
    theme_apply(0);   /* start on Fire; Scene.theme_id tracks it after that */
}

/* ===================================================================== */
/* §9  scene — everything that makes up one on-screen picture             */
/* ===================================================================== */

/*
 * Scene — all the "what's on screen" state in one place. The whole program
 * boils down to: update a Scene (scene_tick), then draw it (scene_draw).
 *
 * The view, canvas, and reveal are useless on their own — you need the view to
 * know what point a cell is, the canvas to store its colour, and the reveal to
 * know which cell comes next. Bundling them lets one pointer carry the whole
 * picture between functions.
 *
 * preset_id and theme_id are just positions in the preset/theme tables. A
 * position is easy to step to the next one, reads straight into the info bar as
 * "view 2/5", and can never point at freed memory.
 */
typedef struct {
    int        preset_id;   /* which preset is showing (0..N_PRESETS-1)        */
    int        theme_id;    /* which colour theme is active                    */
    ViewWindow view;        /* the patch of plane this preset maps to          */
    Canvas     canvas;      /* the colours filled in so far                    */
    Reveal     reveal;      /* how far through the random fill we are           */
    bool       paused;      /* true = stop filling in new cells                */
} Scene;

/* Switch to a preset: rebuild the view, clear the canvas, reshuffle the order. */
static void scene_load(Scene *s, int preset_id, int cols, int rows)
{
    s->preset_id = preset_id % N_PRESETS;
    s->view      = view_from_preset(&k_presets[s->preset_id], cols, rows);
    canvas_reset(&s->canvas, cols, rows);
    reveal_shuffle(&s->reveal, s->canvas.rows * s->canvas.cols);
}

static void scene_init(Scene *s, int cols, int rows)
{
    s->theme_id = 0;
    s->paused   = false;
    scene_load(s, 0, cols, rows);
}

/*
 * Work out and store the colour of one cell — the heart of the program: turn
 * the flattened index back into a row and column, ask the view which point that
 * is, run the runaway count, and record the colour (leaving far-background
 * cells blank).
 */
static void scene_color_pixel(Scene *s, int flat_index)
{
    int col = flat_index % s->canvas.cols;          /* unflatten: see Reveal docs */
    int row = flat_index / s->canvas.cols;

    Complex point = view_sample(&s->view, col, row, s->canvas.cols, s->canvas.rows);
    uint8_t band  = escape_to_band(burning_ship_escape(point));

    if (band) s->canvas.band[row][col] = band;
}

/*
 * Colour the next batch of cells. Once the whole picture is filled it just sits
 * there — presets don't move on by themselves.
 */
static void scene_tick(Scene *s)
{
    if (s->paused || reveal_complete(&s->reveal)) return;

    int batch_end = s->reveal.done + PIXELS_PER_TICK;
    if (batch_end > s->reveal.total) batch_end = s->reveal.total;

    for (int i = s->reveal.done; i < batch_end; i++)
        scene_color_pixel(s, s->reveal.order[i]);

    s->reveal.done = batch_end;
}

static void scene_next_preset(Scene *s, int cols, int rows)
{
    scene_load(s, (s->preset_id + 1) % N_PRESETS, cols, rows);
}

static void scene_next_theme(Scene *s)
{
    s->theme_id = (s->theme_id + 1) % N_THEMES;
    theme_apply(s->theme_id);
}

static void scene_draw(const Scene *s, WINDOW *w) { canvas_paint(&s->canvas, w); }

/* ===================================================================== */
/* §10  screen — ncurses lifecycle + HUD                                  */
/* ===================================================================== */

/*
 * Screen — the current terminal size. Re-measured at startup and whenever the
 * window is resized; used to place the info bars and to rebuild the view so the
 * fractal always fills the window.
 */
typedef struct {
    int cols;   /* terminal width  in characters */
    int rows;   /* terminal height in characters */
} Screen;

static void screen_init(Screen *sc)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_free(Screen *sc)  { (void)sc; endwin(); }

static void screen_resize(Screen *sc)
{
    endwin();
    refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

/* Top row, bold, pushed to the right: the live status — fps, which view, paused. */
static void hud_data_line(const Screen *sc, const Scene *s, double fps)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " BurningShip  %5.1f fps  view %d/%d  %s ",
             fps, s->preset_id + 1, N_PRESETS, s->paused ? "PAUSED " : "running");

    int right_x = sc->cols - (int)strlen(buf);          /* line it up at the right edge */
    if (right_x < 0) right_x = 0;

    attron(COLOR_PAIR(COL_HUD) | A_BOLD);
    mvprintw(0, right_x, "%s", buf);
    attroff(COLOR_PAIR(COL_HUD) | A_BOLD);
}

/* Second row: the settings behind the current view — name, progress, speed, theme. */
static void hud_params_line(const Scene *s, int sim_fps)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %-14s  %3d%%  spd:%d Hz  theme:%s ",
             k_presets[s->preset_id].name, reveal_percent(&s->reveal),
             sim_fps, k_themes[s->theme_id].name);

    attron(COLOR_PAIR(COL_HUD));
    mvprintw(1, 0, "%s", buf);
    attroff(COLOR_PAIR(COL_HUD));
}

/* hud_action_line — last row, bright cyan: every interactive key. */
static void hud_action_line(const Screen *sc)
{
    attron(COLOR_PAIR(COL_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  n:next view  spc:pause  t:theme  [ / ]:speed ");
    attroff(COLOR_PAIR(COL_HINT) | A_BOLD);
}

/*
 * screen_draw — compose one frame: the fractal, then the HUD.
 * HUD layout is fixed — top is data, bottom is actions:
 *   row 0      live data       (hud_data_line)
 *   row 1      parameters      (hud_params_line)
 *   row rows-1 key actions     (hud_action_line)
 */
static void screen_draw(const Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_draw(s, stdscr);            /* the fractal itself     */
    hud_data_line(sc, s, fps);        /* row 0   — live data    */
    hud_params_line(s, sim_fps);      /* row 1   — parameters   */
    hud_action_line(sc);              /* last row — key actions */
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §11  app — signals, input, main loop                                   */
/* ===================================================================== */

/*
 * App — the top-level container main() owns: the picture (scene), the terminal
 * it lives in (screen), and the one tunable speed (sim_fps).  Kept as a single
 * struct so the loop passes "everything" with one &app.
 */
typedef struct {
    Scene  scene;     /* the fractal picture + all its sub-state               */
    Screen screen;    /* current terminal dimensions                           */
    int    sim_fps;   /* reveal speed, ticks/sec (SIM_FPS_MIN..MAX); ']' / '[' */
} App;

/* The only globals: POSIX signal handlers take no arguments, so they must
 * write a flag the main loop polls. */
static volatile sig_atomic_t g_should_quit   = 0;
static volatile sig_atomic_t g_should_resize = 0;

static void on_stop (int sig) { (void)sig; g_should_quit   = 1; }
static void on_winch(int sig) { (void)sig; g_should_resize = 1; }
static void cleanup (void)    { endwin(); }

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
    case 'q': case 'Q': case 27:  return false;                    /* quit          */
    case 'r': case 'R':
    case 'n': case 'N':  scene_next_preset(&a->scene,
                                  a->screen.cols, a->screen.rows);  break; /* next view */
    case 't': case 'T':  scene_next_theme(&a->scene);              break; /* palette   */
    case 'p': case 'P':
    case ' ':            a->scene.paused = !a->scene.paused;       break; /* pause     */
    case ']':            app_change_speed(a, +SIM_FPS_STEP);       break; /* faster    */
    case '[':            app_change_speed(a, -SIM_FPS_STEP);       break; /* slower    */
    default: break;
    }
    return true;
}

/*
 * app_run_due_ticks — fixed-timestep catch-up.
 * Add the elapsed time to an accumulator and spend one scene_tick per whole
 * tick's worth, so the reveal advances at sim_fps regardless of frame rate.
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
    signal(SIGINT,   on_stop);
    signal(SIGTERM,  on_stop);
    signal(SIGWINCH, on_winch);

    /* static (BSS) — Scene holds large arrays; keep them off the stack */
    static App app = { .sim_fps = SIM_FPS_DEFAULT };

    screen_init(&app.screen);
    scene_init(&app.scene, app.screen.cols, app.screen.rows);

    int64_t    frame_time = clock_ns();
    int64_t    sim_accum  = 0;        /* elapsed time not yet spent on ticks */
    FpsCounter fps        = {0};

    /* Main loop — one pass per displayed frame:
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
            frame_time = clock_ns();             /* don't bill resize as elapsed */
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
