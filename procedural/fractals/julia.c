/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * julia.c  —  Julia set fractal, animated random-pixel fill
 *
 * For every pixel (col, row) on the terminal, the position is mapped to
 * a complex number z = re + im·i and iterated under f(z) = z² + c until
 * |z| > 2 (escape) or MAX_ITER is reached (inside the set).
 *
 * Pixels are revealed in a random shuffled order, so the fractal
 * materialises from random scattered dots rather than a scan line.
 * Once all pixels are computed the animation pauses briefly, then
 * cycles to the next Julia parameter preset and redraws.
 *
 * Color — a cycle-able theme (t / T): the set body plus four escape bands
 * fading from near-boundary (bright) to far exterior (dim) to black.
 *
 * Six presets cycle automatically:
 *   douady rabbit · spiral galaxy · dendrite · flame · seahorse · basilica
 *
 * Keys:
 *   q / ESC   quit
 *   r / n     skip to next Julia preset immediately
 *   t / T     cycle color theme (Fire/Ocean/Toxic/Neon/Mono)
 *   ] [       faster / slower simulation
 *   p / spc   pause / resume
 *
 * Build (no -lm — this file uses no math-library functions):
 *   gcc -std=c11 -O2 -Wall -Wextra julia.c -o julia -lncurses
 *
 * Reading order — built bottom-up; each section uses only earlier ones.
 *   §1  config   — constants, palette slots, themes
 *   §2  clock    — monotonic time
 *   §3  complex  — Complex number + arithmetic (the plane z lives in)
 *   §4  fractal  — Julia escape iteration + escape→colour mapping
 *   §5  view     — Preset (the Julia constant c) + ViewWindow lens
 *   §6  canvas   — pixel buffer of colour bands
 *   §7  reveal   — Fisher-Yates random-fill animation
 *   §8  color    — apply a Theme to ncurses colour pairs
 *   §9  scene    — Scene: composes view + canvas + reveal into one picture
 *   §10 screen   — ncurses lifecycle + HUD
 *   §11 app      — signals, input, main loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Escape-time — per-pixel iteration until escape.  Unlike
 *                  Mandelbrot (c varies per pixel, z₀ = 0), a Julia set fixes c
 *                  globally and varies the START point z₀ (= the pixel).  Each
 *                  pixel asks: does the orbit of z₀ under f(z) = z² + c stay
 *                  bounded?  Pixels are revealed in shuffled order.
 *
 * Math           : Julia–Mandelbrot duality: J(c) is connected iff c ∈ M.  Near
 *                  ∂M the Julia set is most intricate; well inside M it's a
 *                  filled blob; outside M it's disconnected dust.  Escape test
 *                  |z|² > 4 (squared, to avoid a sqrt).
 *
 * Data model     : Small structs, one idea each —
 *                    Complex     a point/number in the plane (z, c)
 *                    Preset      the Julia constant c that defines a set
 *                    ViewWindow  the rectangle of the plane on screen
 *                    Canvas      the grid of coloured cells we paint
 *                    Reveal      the random order we fill the canvas in
 *                    Scene       all of the above + which preset/theme is live
 *
 * Rendering      : Escape count → 5-slot palette (body + 4 escape bands), themed
 *                  and cycle-able.  Random pixel order makes the fractal
 *                  materialise everywhere at once instead of scan-line by line.
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 * Concepts & maths
 *   [1] Julia, G. (1918). "Mémoire sur l'itération des fonctions rationnelles."
 *       J. Math. Pures Appl. 8:47-245.  — the founding paper; these sets carry
 *       his name.  (Fatou's parallel 1919-20 work is the other half.)
 *   [2] Mandelbrot, B. B. (1982). "The Fractal Geometry of Nature." Freeman.
 *       — self-similarity and the z²+c family; popularised these images.
 *   [3] Peitgen, H.-O. & Richter, P. H. (1986). "The Beauty of Fractals."
 *       Springer.  — the clearest pictures + complex-dynamics behind Julia and
 *       Mandelbrot sets; start here for intuition.
 *   [4] Douady, A. & Hubbard, J. H. (1984-85). "Étude dynamique des polynômes
 *       complexes" (the Orsay Notes).  — proves the duality used in §5: the
 *       Julia set J(c) is connected iff c lies in the Mandelbrot set.
 *
 * Rendering & escape-time
 *   [5] Peitgen, Jürgens & Saupe (1992). "Chaos and Fractals: New Frontiers of
 *       Science." Springer.  — the escape-time algorithm in full; the §4 loop
 *       is a direct descendant.
 *   [6] Vepstas, L. (2004). "Renormalizing the Mandelbrot Escape."  — the
 *       fractional escape count n + 1 − log₂(log|z|) that removes the visible
 *       banding; consult before swapping in smooth colouring for §4's bands.
 *
 * In-repo study companions
 *   [7] julia_explorer.c — interactive Mandelbrot↔Julia: move c on the
 *       Mandelbrot map and watch J(c) change, SEEING the duality [4] live.
 *   [8] mandelbrot.c — the same z²+c escape engine, but z₀ = 0 and c varies per
 *       pixel; diff it against §4 to see exactly what "Julia vs Mandelbrot" is.
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

    PIXELS_PER_TICK  =  60,    /* pixels computed per simulation tick    */
    MAX_ITER         = 128,    /* Julia iteration cap                    */
    DONE_PAUSE_TICKS =  90,    /* ticks to hold completed fractal (~3 s) */
    N_PRESETS        =   6,    /* Julia parameter presets                */
    N_THEMES         =   5,    /* colour themes to cycle with t/T        */
    N_COLORS         =   5,    /* content slots: INSIDE + C2..C5         */

    CANVAS_ROWS_MAX  =  80,
    CANVAS_COLS_MAX  = 300,

    RENDER_FPS       =  60,    /* display refresh cap (≠ reveal speed)  */
    MAX_FRAME_MS     = 100,    /* clamp on one frame's dt (anti spiral) */
    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,    /* recompute the fps readout this often  */
};

/* |z| > 2 ⇒ the orbit escapes to infinity.  Compare |z|² to 4 to skip a sqrt. */
#define ESCAPE_RADIUS_SQ  4.0f

/* Pixels escaping within the first 12 % of the iteration budget are far
 * exterior — left blank so the picture isn't a wall of noise. */
#define BACKGROUND_FRAC   0.12f

/* Half-height of the complex-plane window, in imaginary units.  Fixed for all
 * presets; the real half-extent is derived from it and the terminal aspect. */
#define VIEW_HALF_IM      1.3f

/*
 * ASPECT_R — terminal cell height / width ≈ 2.  Preserves the aspect ratio of
 * the complex plane when mapping (col, row) → (re, im).
 */
#define ASPECT_R    2.0f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * ColorID — the colour "slots" a pixel can take.
 *
 * THE colouring idea to learn: an escape-time fractal colours each pixel by HOW
 * LONG its orbit took to escape (its "escape time"; ref [5]):
 *   never escaped → COL_INSIDE  (the point belongs to the set)
 *   escaped slowly → COL_C5     (just outside the boundary — bright)
 *   escaped fast   → COL_C2     (far outside — dim)
 * Slow escape ⇒ near the intricate boundary ⇒ vivid; fast escape ⇒ far away.
 *
 * One value does double duty: it's both the ncurses colour-pair id AND the byte
 * stored in each Canvas cell.  HUD slots are theme-independent; 0 = blank.
 */
typedef enum {
    COL_INSIDE = 1,   /* the set itself — orbit never escaped */
    COL_C2     = 2,   /* far exterior   — escaped fastest     */
    COL_C3     = 3,   /* exterior                             */
    COL_C4     = 4,   /* mid-exterior                         */
    COL_C5     = 5,   /* near boundary  — escaped slowest     */
    COL_HUD    = 6,   /* HUD top data lines    — yellow       */
    COL_HINT   = 7,   /* HUD bottom action bar — cyan         */
} ColorID;

/*
 * Theme — a named colour scheme for the body + four escape bands, cycled t / T.
 *
 * WHY it exists: the maths only ever produces a band number per pixel (a
 * ColorID); a Theme is the lookup that turns those abstract bands into real
 * on-screen colours, so the whole picture restyles by swapping one table row —
 * no recompute of the fractal.  (Same table-driven idea as buddhabrot.c's
 * palette table.)
 *
 * HOW to read the arrays, dim → bright: fg[0] = INSIDE (the body), then
 * fg[1..4] = C2..C5 (far/dim escape → near/bright escape).
 *
 * WHY every value is bright: a 256-index from the dark bottom of the cube is
 * invisible on black, so all bands sit in the bright half — even the far halo
 * stays visible; the near-boundary band (C5) is also drawn bold.
 */
typedef struct {
    const char *name;            /* shown in the HUD, e.g. "Ocean"      */
    int fg256[N_COLORS];         /* slots INSIDE, C2, C3, C4, C5 (256)  */
    int fg8[N_COLORS];           /* same five slots, 8-colour fallback  */
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

/*
 * clamp_frame_dt — cap a single frame's elapsed time.  If the program stalls
 * (debugger, laptop sleep), a huge dt would make the fixed-timestep loop run a
 * burst of catch-up ticks at once — the "spiral of death".  Capping trades one
 * slow frame for stability.
 */
static int64_t clamp_frame_dt(int64_t dt)
{
    int64_t cap = MAX_FRAME_MS * NS_PER_MS;
    return dt > cap ? cap : dt;
}

/*
 * frame_pace — sleep so the whole frame lasts about 1 / target_fps.
 * `frame_start` is when the frame began; `work_done` is time already spent, so
 * we sleep only the leftover budget instead of busy-waiting.
 */
static void frame_pace(int target_fps, int64_t frame_start, int64_t work_done)
{
    int64_t budget = TICK_NS(target_fps);
    int64_t spent  = clock_ns() - frame_start + work_done;
    clock_sleep_ns(budget - spent);
}

/*
 * FpsCounter — a small running readout of frames-per-second for the HUD.  Sums
 * frames and elapsed time, then every FPS_UPDATE_MS divides the two for a smooth
 * value (raw per-frame fps jitters too much to read).
 */
typedef struct {
    int64_t accum_ns;   /* real time since the last readout update */
    int     frames;     /* frames counted since then               */
    double  value;      /* last computed fps — what the HUD shows  */
} FpsCounter;

static void fps_count_frame(FpsCounter *f, int64_t dt)
{
    f->frames   += 1;
    f->accum_ns += dt;
    if (f->accum_ns >= FPS_UPDATE_MS * NS_PER_MS) {       /* time to refresh? */
        f->value    = (double)f->frames / ((double)f->accum_ns / (double)NS_PER_SEC);
        f->frames   = 0;
        f->accum_ns = 0;
    }
}

/* ===================================================================== */
/* §3  complex — the number system the fractal iterates in                */
/* ===================================================================== */

/*
 * Complex — one point in the complex plane, written z = re + im·i.
 *
 * WHY fractals are built on this: a complex number is a 2-D point (re, im) that
 * comes with a special multiplication — squaring a point both SCALES and
 * ROTATES it about the origin.  Repeatedly doing "square it, then add a fixed
 * point c" either flings the point off to infinity or traps it near the origin
 * forever; the razor's edge between those two fates is the fractal boundary.
 * (Iteration of complex functions: refs [1][2].)
 *
 * In this program: every screen pixel IS one Complex number — its starting
 * point z₀ — and the Julia constant c is Complex too.  Naming the type and
 * giving it add/square lets the iteration read like its formula:  z ← z² + c.
 *
 * WHY float: the window spans only a few units, so ~7 digits is plenty, and it
 * is faster than double across the millions of iterations per frame.
 */
typedef struct {
    float re;   /* real part      — horizontal axis */
    float im;   /* imaginary part — vertical axis   */
} Complex;

/* z = a + b */
static inline Complex complex_add(Complex a, Complex b)
{
    return (Complex){ a.re + b.re, a.im + b.im };
}

/* z² = (re² − im²) + (2·re·im)·i */
static inline Complex complex_square(Complex z)
{
    return (Complex){ z.re * z.re - z.im * z.im, 2.0f * z.re * z.im };
}

/* |z|² = re² + im²  (squared modulus — compared against ESCAPE_RADIUS_SQ) */
static inline float complex_norm_sq(Complex z)
{
    return z.re * z.re + z.im * z.im;
}

/* ===================================================================== */
/* §4  fractal — the escape-time algorithm                                */
/* ===================================================================== */

/*
 * julia_escape — how many steps the orbit of z₀ survives before it escapes the
 * radius-2 disc, under f(z) = z² + c for this set's fixed c.  Returns MAX_ITER
 * if it never escapes (z₀ is inside the set).
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
 * escape_to_band — turn an escape count into a Canvas colour slot.
 *   never escaped (== MAX_ITER) → COL_INSIDE
 *   escaped in first 12 %       → 0 (blank: far exterior)
 *   otherwise                   → COL_C2..COL_C5, slow escape = brighter
 */
static uint8_t escape_to_band(int escape)
{
    if (escape >= MAX_ITER) return COL_INSIDE;

    float frac = (float)escape / (float)MAX_ITER;   /* 0 = instant … →1 = slow */
    if (frac < BACKGROUND_FRAC) return 0;

    /* position within the coloured halo: rescale [BACKGROUND_FRAC,1) → [0,1) */
    float halo_pos = (frac - BACKGROUND_FRAC) / (1.0f - BACKGROUND_FRAC);
    int   n_bands  = COL_C5 - COL_C2 + 1;           /* 4 escape bands          */
    int   band     = COL_C2 + (int)(halo_pos * (float)n_bands);
    return (uint8_t)(band > COL_C5 ? COL_C5 : band);
}

/* ===================================================================== */
/* §5  view — the Julia constant, and the lens onto the plane             */
/* ===================================================================== */

/*
 * Preset — one Julia set, identified by its constant c.
 *
 * THE key idea to learn: there isn't one Julia set — there are infinitely many,
 * one for every value of c.  Fix c, then ask of every start point z₀ "does
 * z ← z² + c stay bounded?"  The points that DO form the Julia set J(c).  A
 * different c gives a completely different shape, which is why a Preset is just
 * a name + a c; nothing else changes.
 *
 * WHY these particular c's: each sits near the edge of the Mandelbrot set,
 * where J(c) is connected and most lacy — the famous rabbit, dendrite, seahorse.
 * Deep inside M it would be a plain blob; outside M, disconnected dust.  (That
 * link, J(c) connected ⇔ c ∈ M, is the Julia–Mandelbrot duality — ref [4].)
 */
typedef struct {
    const char *name;   /* shown in the HUD, e.g. "seahorse" */
    Complex     c;      /* the constant in f(z) = z² + c     */
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
 * ViewWindow — which rectangle of the (infinite) complex plane we're looking at.
 *
 * The plane is continuous and endless; the terminal is a fixed grid of cells.
 * This struct is the bridge: a centre point (the origin, for every Julia set)
 * plus how far the view reaches each way.  view_sample turns a cell into the
 * complex point it stands for — the ONE place screen coordinates meet the maths.
 *
 * WHY half_re ≠ half_im: terminal cells are ~2× taller than wide, so the real
 * (horizontal) half-extent is widened to match; otherwise the set would look
 * vertically stretched.
 */
typedef struct {
    Complex center;    /* plane point shown at screen centre (origin for Julia) */
    float   half_re;   /* half-width  in real units (aspect-corrected)          */
    float   half_im;   /* half-height in imaginary units                        */
} ViewWindow;

/* Build the view for the current terminal size (same window for all presets). */
static ViewWindow view_for_screen(int cols, int rows)
{
    ViewWindow v;
    v.center  = (Complex){ 0.0f, 0.0f };
    v.half_im = VIEW_HALF_IM;
    v.half_re = VIEW_HALF_IM * (float)cols / (float)rows / ASPECT_R;
    return v;
}

/*
 * view_sample — which complex number is the start point z₀ for screen cell
 * (col, row)?  Centre-relative offset in [-1,+1] per axis, scaled by the
 * half-extent.  The imaginary offset is negated so row 0 (top) is +im.
 */
static Complex view_sample(const ViewWindow *v, int col, int row, int cols, int rows)
{
    float fx = (float)col / (float)(cols - 1);   /* 0 → 1 left to right */
    float fy = (float)row / (float)(rows - 1);   /* 0 → 1 top to bottom */

    float off_re =  (fx - 0.5f) * 2.0f;
    float off_im = -(fy - 0.5f) * 2.0f;          /* flip: top of screen = +im */

    return (Complex){
        .re = v->center.re + off_re * v->half_re,
        .im = v->center.im + off_im * v->half_im,
    };
}

/* ===================================================================== */
/* §6  canvas — the grid of coloured cells we paint to the terminal       */
/* ===================================================================== */

/*
 * Canvas — the picture held in memory: one ColorID per terminal cell.
 *
 * WHY a buffer between the maths and the screen: the escape algorithm only ever
 * writes a band number into band[row][col]; canvas_paint alone turns those
 * numbers into glyphs.  So the fractal maths never mentions ncurses, and either
 * half can change without touching the other.
 *
 * 0 = not computed yet (or fast-escape background); 1..5 = a colour band.
 * Fixed-size (this project never mallocs in the loop); rows/cols = portion used.
 */
typedef struct {
    int     rows, cols;                              /* portion in use       */
    uint8_t band[CANVAS_ROWS_MAX][CANVAS_COLS_MAX];  /* [row][col] → ColorID */
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
 * band_glyph — the character drawn for each colour slot, indexed by ColorID.
 * NOTE: this glyph mapping is inherited from mandelbrot.c and is uneven — the
 * set interior shows as ',' and the C3 band as a blank.  Colour does the real
 * visual work; the glyph just adds a little texture.
 */
static chtype band_glyph(uint8_t slot)
{
    static const char glyph[8] = " ,. +#*";   /* [ColorID] → glyph */
    return (chtype)(unsigned char)glyph[slot < 7 ? slot : 0];
}

/* band_attr — ncurses attributes for a slot: its colour pair, plus A_BOLD on
 * the set body and the bright near-boundary band so they stand out. */
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
            if (slot == 0) continue;                 /* not computed / background */
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
 * Reveal — the ORDER cells get computed in, purely for the look: the fractal
 * condenses out of random dots instead of wiping top-to-bottom.
 *
 * order[] lists every cell's flattened index (row*cols + col) once, then a
 * Fisher-Yates shuffle randomises it (Knuth, TAOCP Vol. 2 §3.4.2); `done` walks
 * through it PIXELS_PER_TICK at a time.  Unflatten with col = idx % cols,
 * row = idx / cols.
 */
typedef struct {
    int order[CANVAS_ROWS_MAX * CANVAS_COLS_MAX];  /* shuffled flat cell indices */
    int total;   /* cells to reveal (rows * cols) */
    int done;    /* cells revealed so far         */
} Reveal;

/* fisher_yates_shuffle — uniformly shuffle a[0..n) in place, O(n): swap each
 * slot (back to front) with a randomly chosen earlier-or-equal one. */
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
    for (int i = 0; i < n_pixels; i++)            /* list every cell once     */
        r->order[i] = i;
    fisher_yates_shuffle(r->order, n_pixels);     /* randomise the visit order */
}

static bool reveal_complete(const Reveal *r) { return r->done >= r->total; }

static int reveal_percent(const Reveal *r)
{
    return r->total ? (int)((long)r->done * 100 / r->total) : 100;
}

/* ===================================================================== */
/* §8  color — bind a Theme to ncurses colour pairs                       */
/* ===================================================================== */

/* Bind the body + four escape bands (COL_INSIDE..COL_C5) to the active theme. */
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
    /* HUD/HINT are theme-independent — yellow data, cyan actions on any palette */
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);
        init_pair(COL_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    theme_apply(0);   /* Fire; Scene.theme tracks the active one thereafter */
}

/* ===================================================================== */
/* §9  scene — everything needed to render one Julia view                 */
/* ===================================================================== */

/*
 * Scene — the whole "what is on screen" in one place: which Julia set (preset →
 * c) and palette are live, the view window, the painted canvas, and how far the
 * random fill has gotten.  Everything the program does reduces to: update a
 * Scene (scene_tick), then draw it (scene_draw).
 *
 * WHY ids, not pointers: preset_id / theme are plain indices into the
 * k_presets / k_themes tables — trivial to cycle ( (id+1) % N ), they read
 * straight into the HUD, and they never dangle.
 */
typedef struct {
    int        preset_id;   /* which k_presets entry — the Julia constant c */
    int        theme;       /* which k_themes palette is active             */
    ViewWindow view;        /* the complex-plane rectangle being drawn      */
    Canvas     canvas;      /* painted cells                                */
    Reveal     reveal;      /* random-fill progress                         */
    int        hold_ticks;  /* ticks held after completion, before cycling  */
    bool       paused;      /* true = freeze the reveal                     */
} Scene;

/* Point the scene at a preset: rebuild the lens, clear the canvas, reshuffle. */
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
 * scene_color_pixel — compute and store the colour of ONE canvas cell:
 *   1. unflatten the shuffled index into (row, col)
 *   2. ask the lens for that cell's start point z₀
 *   3. run the escape algorithm with this preset's constant c
 *   4. record the resulting colour band (skip pure-background cells)
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

/* scene_reveal_batch — compute the next PIXELS_PER_TICK cells from the shuffled
 * order; the actual "grow the picture" work. */
static void scene_reveal_batch(Scene *s)
{
    int batch_end = s->reveal.done + PIXELS_PER_TICK;
    if (batch_end > s->reveal.total) batch_end = s->reveal.total;

    for (int i = s->reveal.done; i < batch_end; i++)
        scene_color_pixel(s, s->reveal.order[i]);

    s->reveal.done = batch_end;
}

/*
 * scene_tick — advance the picture by one tick: draw → hold → next preset.
 * While filling, reveal the next batch.  Once complete, count down the hold and
 * then auto-advance to the next Julia set.
 */
static void scene_tick(Scene *s)
{
    if (s->paused) return;

    if (reveal_complete(&s->reveal)) {                /* finished — hold, then cycle */
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
 * Screen — the live terminal size in character cells.  Measured at startup and
 * after every resize (SIGWINCH); the view scales to it and the HUD lays out
 * against it.
 */
typedef struct {
    int cols;   /* terminal width  in characters */
    int rows;   /* terminal height in characters */
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
 * hud_line — draw one HUD line at (row, x) in colour `pair`, clamped to the
 * terminal width so a long line never wraps onto the fractal.  The "%.*s"
 * precision truncates the text to the columns actually left from x.
 */
static void hud_line(int row, int x, int pair, attr_t bold, int cols, const char *str)
{
    if (x < 0) x = 0;
    if (x >= cols) return;
    attron(COLOR_PAIR(pair) | bold);
    mvprintw(row, x, "%.*s", cols - x, str);
    attroff(COLOR_PAIR(pair) | bold);
}

/* hud_data_line — row 0, right-aligned bold: title, fps, run state. */
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
