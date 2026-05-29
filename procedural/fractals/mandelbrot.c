/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * mandelbrot.c — the Mandelbrot set, revealed as scattered dots
 *
 * Each cell maps to a complex number c; starting from z = 0 the iteration
 * z → z² + c is repeated until |z| > 2 (escape) or MAX_ITER (inside the set).
 * The escape count colours the cell; the interior is drawn solid.
 *
 * FOUR LAYERS, deliberately separated so the maths is trustworthy and the things
 * that happen "over time" are obvious:
 *
 *   §4 CORE   — the pure maths: complex iteration → escape count → colour band.
 *               Given a point c it returns a cell value.  It touches no grid, no
 *               clock, no screen, so the fractal definition is safe.
 *   §5 FIELD  — the cell grid plus the complex-plane window it samples and the
 *               cell → c mapping.  Pure data.
 *   §6 REVEAL — the EFFECTS & DELAYS, all in one place: the shuffled pixel order
 *               (so the image materialises from scattered dots), the per-tick
 *               pixel budget (so a frame never stalls), and the hold before the
 *               next preset.  Remove it and the picture would simply appear in
 *               scan order at once; nothing else is applied over time but drawing.
 *   §8 RENDER — reads the field and paints glyphs.  Mutates nothing.
 *   §7 SCENE  — orchestration: field + reveal + preset + theme.
 *
 * Six preset zoom windows auto-advance, each a different region of the set
 * (full set, seahorse valley, north antenna, deep spiral, mini mandelbrot,
 * antenna tip).
 *
 * Keys:
 *   q / ESC   quit
 *   r / n     skip to the next preset
 *   t / T     next / prev colour theme
 *   ] [       faster / slower simulation
 *   p / spc   pause / resume
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra mandelbrot.c -o mandelbrot -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config   — constants, themes
 *   §2  clock    — monotonic ns clock + FrameClock (fixed-timestep pacing)
 *   §3  color    — themeable content palette (t/T) + HUD pairs
 *   §4  core     — complex iteration + escape count + band classify (PURE MATH)
 *   §5  field    — cell grid + complex-plane view + cell→c mapping (DATA)
 *   §6  reveal   — shuffled progressive fill + done-hold (EFFECTS & DELAYS)
 *   §7  scene    — orchestration: field + reveal + preset + theme
 *   §8  render   — field → glyphs (READ-ONLY)
 *   §9  screen   — ncurses lifecycle + HUD
 *   §10 app      — main loop
 */

/* ── REFERENCES — for the concepts and the rendering ──────────────────── *
 *
 *   The Mandelbrot set & escape-time  (§4 core)
 *   ── Brooks, R. & Matelski, J. P. (1981). "The Dynamics of 2-Generator
 *      Subgroups of PSL(2,C)." In Riemann Surfaces & Related Topics, Annals of
 *      Math. Studies 97, 65-71.  The first published picture of the set.
 *   ── Mandelbrot, B. B. (1980). "Fractal Aspects of the Iteration of z → λz(1−z)
 *      for Complex λ and z." Annals NY Acad. Sci. 357, 249-259.  The set and the
 *      escape-time view.
 *   ── Mandelbrot, B. B. (1982). "The Fractal Geometry of Nature." Freeman.
 *      Self-similarity and fractal dimension of the boundary.
 *   ── Douady, A. & Hubbard, J. H. (1982). "Itération des polynômes quadratiques
 *      complexes." C. R. Acad. Sci. Paris 294, 123-126.  Proof that the set is
 *      connected — the structure these zoom presets explore.
 *   ── Peitgen, H.-O. & Richter, P. H. (1986). "The Beauty of Fractals." Springer.
 *      The standard reference for escape-time colouring (cell_for_escape).
 *
 *   Rendering  (§6 reveal, §9 screen)
 *   ── Fisher, R. A. & Yates, F. (1938). "Statistical Tables." (Knuth, TAOCP
 *      Vol. 2, §3.4.2.)  The unbiased shuffle behind the random reveal order.
 *   ── Gookin, D. (2007). "Programmer's Guide to NCURSES." Wiley.  The cell-
 *      drawing API behind §8/§9.
 * ─────────────────────────────────────────────────────────────────────── */

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

    PIXELS_PER_TICK  =  60,    /* pixels revealed per sim tick (the build-up rate) */
    MAX_ITER         = 256,    /* Mandelbrot iteration cap                         */
    DONE_PAUSE_TICKS =  90,    /* ticks to hold a finished view (~3 s) before next */
    N_PRESETS        =   6,
    N_COLORS         =   5,    /* INSIDE + 4 escape bands (C2..C5)                 */
    N_THEMES         =  10,

    GRID_ROWS_MAX    =  80,
    GRID_COLS_MAX    = 300,

    HUD_TOP_ROWS     =   2,    /* rows 0..1 — data HUD (status + parameters) */
    HUD_BOT_ROWS     =   1,    /* last row  — action / key-hint bar          */
    HUD_COLS         =  96,
    FPS_UPDATE_MS    = 500,
    FRAME_DT_CAP_MS  = 100,    /* clamp one frame's dt — stops spiral-of-death */
    RENDER_FPS_CAP   =  60,    /* render-rate ceiling the loop sleeps down to  */
};

/* Escape test: |z| > 2  ⟺  |z|² > 4. */
#define ESCAPE_R2        4.0f
/* Escapes faster than this fraction of MAX_ITER are background (too far out). */
#define ESCAPE_FRAC_MIN  0.10f

/* ASPECT_R — terminal cell height / width ≈ 2; undistorts the complex mapping. */
#define ASPECT_R    2.0f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Theme — the five content colours of one palette: the escape-time ramp that maps
 * how slowly a pixel escaped onto a colour (Peitgen & Richter 1986), reading
 * inside → boundary halo.
 *   c[0] = COL_INSIDE  (fractal body)
 *   c[1] = COL_C2      (fastest shown escape band, the far halo)
 *   c[2] = COL_C3, c[3] = COL_C4, c[4] = COL_C5 (progressively nearer the body)
 * c8[] is the 8-colour fallback.  The HUD is theme-independent (yellow data, cyan
 * actions) per the project standard, so a theme carries no HUD colour.  Adding a
 * palette is one table row; the engine never changes.
 */
typedef struct {
    const char *name;   /* shown in the HUD                   */
    int c[5];           /* 256-colour: INSIDE, C2, C3, C4, C5 */
    int c8[5];          /* 8-colour fallback, same order      */
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
 * FrameClock — the fixed-timestep heartbeat.  Real frames arrive irregularly, but
 * the simulation must advance in equal slices: each frame we bank the elapsed real
 * time as `sim_debt`, then repay it one TICK_NS step at a time.  The fps_* fields
 * measure a smoothed rate for the HUD.
 */
typedef struct {
    int64_t prev_ns;     /* CLOCK_MONOTONIC stamp at this frame's start          */
    int64_t sim_debt;    /* unspent ns owed to the fixed step; left in [0,TICK)  */
    int64_t fps_window;  /* ns elapsed since the last fps sample was emitted     */
    int     fps_frames;  /* frames counted since the last fps sample             */
    double  fps;         /* smoothed frames/sec, refreshed every FPS_UPDATE_MS   */
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

/* Open a frame: measure dt since the last one, clamp a long stall, bank it. */
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

/* True (and pays down the debt) while a fixed sim step is owed. */
static bool frameclock_step_due(FrameClock *fc, int64_t tick_ns)
{
    if (fc->sim_debt < tick_ns) return false;
    fc->sim_debt -= tick_ns;
    return true;
}

/* Sleep off the remainder of this frame's budget to cap the render rate. */
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
 * Colour-pair slots.  COL_INSIDE / COL_C2..C5 are the content ramp, rebound by
 * theme_apply().  COL_HUD (data) and COL_HINT (actions) are theme-independent so
 * the HUD stays legible over any palette.
 */
typedef enum {
    COL_INSIDE = 1,   /* fractal interior            */
    COL_C2     = 2,   /* fastest shown escape (far)  */
    COL_C3     = 3,
    COL_C4     = 4,
    COL_C5     = 5,   /* slowest escape (near body)  */
    COL_HUD    = 6,   /* HUD data       — bright yellow */
    COL_HINT   = 7,   /* HUD action bar — bright cyan   */
} ColorID;

/* theme_apply() — bind the five content pairs to a theme (HUD pairs are fixed). */
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
    /* HUD pairs are theme-independent — yellow data, cyan actions */
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);
        init_pair(COL_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    theme_apply(0);   /* Electric; the Scene tracks the active theme thereafter */
}

/* ===================================================================== */
/* §4  core — complex iteration + escape time (PURE MATH)                 */
/* ===================================================================== */

/*
 * Complex — a point a + b·i in the complex plane (Mandelbrot 1980).  Two roles:
 * `c` is a pixel's fixed coordinate, and `z` is the moving orbit that the
 * iteration z → z² + c walks.  Whether that orbit stays bounded is the entire
 * question the program answers per pixel.  Held as scalars for speed in the hot
 * loop; the compiler inlines the helpers below to plain float arithmetic.
 */
typedef struct {
    float re;   /* real part      (the a in a + b·i; maps along screen columns) */
    float im;   /* imaginary part (the b; maps up the screen rows)              */
} Complex;

/* complex_sqr_add() — one Mandelbrot step: z² + c. */
static Complex complex_sqr_add(Complex z, Complex c)
{
    return (Complex){ z.re * z.re - z.im * z.im + c.re,
                      2.0f * z.re * z.im        + c.im };
}

/* complex_norm2() — |z|², compared against ESCAPE_R2 to avoid a square root. */
static float complex_norm2(Complex z) { return z.re * z.re + z.im * z.im; }

/*
 * mandelbrot_escape() — iterations for the orbit of 0 under z → z² + c to escape
 * |z| > 2, or MAX_ITER if it stays bounded (c is inside the set).  Pure: same c,
 * same count.
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
 * escape_band() — map an escape fraction in [ESCAPE_FRAC_MIN, 1) onto one of the
 * exterior bands COL_C2 (far) .. COL_C5 (near the body).  `t` is the normalized
 * position within the visible escape range; slower escape = higher band.
 */
static uint8_t escape_band(float frac)
{
    float t = (frac - ESCAPE_FRAC_MIN) / (1.0f - ESCAPE_FRAC_MIN);
    int   band = COL_C2 + (int)(t * (float)(N_COLORS - 1));
    return (uint8_t)(band > COL_C5 ? COL_C5 : band);
}

/*
 * cell_for_escape() — the drawn cell value for an escape count:
 *   never escaped (>= MAX_ITER)         → COL_INSIDE
 *   escaped slower, by how slow         → COL_C2 (far) .. COL_C5 (near the body)
 *   escaped faster than ESCAPE_FRAC_MIN → 0 (background, not drawn)
 */
static uint8_t cell_for_escape(int iter)
{
    if (iter >= MAX_ITER) return (uint8_t)COL_INSIDE;   /* inside the set */
    float frac = (float)iter / (float)MAX_ITER;
    if (frac < ESCAPE_FRAC_MIN) return 0;               /* escaped too fast → background */
    return escape_band(frac);
}

/* ===================================================================== */
/* §5  field — cell grid + complex-plane view + cell→c mapping (DATA)     */
/* ===================================================================== */

/*
 * ViewPreset — a zoom window: the complex-plane centre and half its vertical
 * extent (re_half is derived at runtime from the terminal aspect).
 */
typedef struct {
    float       cr, ci, im_half;
    const char *name;
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
 * View — a window into the complex plane: where it is centred and how far it
 * reaches.  This is the mathematical domain the grid samples; a zoom preset is
 * just a named View.  re_half is scaled by the terminal aspect so circles stay
 * round rather than stretching.
 */
typedef struct {
    Complex center;
    float   re_half;   /* half-width  along the real axis      */
    float   im_half;   /* half-height along the imaginary axis */
} View;

/* view_from_preset() — the View a preset defines on a cols x rows grid. */
static View view_from_preset(int preset, int cols, int rows)
{
    const ViewPreset *p = &k_presets[preset % N_PRESETS];
    View v;
    v.center  = (Complex){ p->cr, p->ci };
    v.im_half = p->im_half;
    v.re_half = p->im_half * (float)cols / (float)rows / ASPECT_R;   /* undistort */
    return v;
}

/* view_point() — the complex coordinate at normalized position (fx, fy) in [0,1];
 * fy = 0 is the top, since the imaginary axis points up the screen. */
static Complex view_point(View v, float fx, float fy)
{
    return (Complex){
        v.center.re + (fx - 0.5f) * 2.0f * v.re_half,
        v.center.im + (0.5f - fy) * 2.0f * v.im_half,
    };
}

/*
 * Field — the sampled image: one colour band per cell over an active rows x cols
 * area, plus the View it samples.  Pure data; field_point() turns a cell into the
 * point c that §4 evaluates there.
 */
typedef struct {
    uint8_t cell[GRID_ROWS_MAX][GRID_COLS_MAX];  /* 0 = bg/unsampled, 1..5 = band */
    int     rows, cols;                          /* active extent (canvas band)   */
    View    view;                                /* the complex-plane window       */
} Field;

static void field_init(Field *f, int cols, int rows, int preset)
{
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    rows -= HUD_TOP_ROWS + HUD_BOT_ROWS;          /* leave room for the HUD bands */
    if (rows < 1)             rows = 1;
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;

    f->cols = cols;
    f->rows = rows;
    f->view = view_from_preset(preset, cols, rows);
    memset(f->cell, 0, sizeof f->cell);
}

/* field_point() — the complex coordinate sampled at cell (col, row). */
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
 * Reveal — every time-based behaviour, isolated here so the maths (§4) and the
 * drawing (§8) stay free of it:
 *   - order[]      a shuffled list of pixel indices, so the fractal materialises
 *                  from scattered dots instead of scanning line by line;
 *   - progress     how many of those pixels are computed so far;
 *   - hold_ticks   how long a finished image has been held before auto-advancing.
 * reveal_step() calls the pure core to fill the field; it only chooses the ORDER
 * and the per-tick budget — never a colour or a screen position.
 */
typedef struct {
    int order[GRID_ROWS_MAX * GRID_COLS_MAX];  /* pixel indices in shuffled order */
    int n;            /* total pixels (= field rows*cols)        */
    int progress;     /* pixels computed so far (cursor into order[]) */
    int hold_ticks;   /* ticks held since completion             */
} Reveal;

/* shuffle() — Fisher-Yates: put a[0..n-1] into a uniformly random order. */
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
