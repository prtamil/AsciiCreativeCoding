/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * newton_fractal.c — Newton's method fractal for f(z) = z⁴ − 1
 *
 * Newton's root-finding iteration, run from every point of the complex plane:
 *
 *   z  ←  z − f(z)/f′(z)  =  (3z⁴ + 1) / (4z³)
 *
 * The four roots are 1, −1, i, −i.  Almost every start converges to one of them;
 * the colour says WHICH root (the basin), the brightness/glyph say HOW FAST.  The
 * boundary between basins is a fractal — infinitely intricate near each root.
 *
 * FOUR LAYERS, deliberately separated so the maths is trustworthy and it is clear
 * what runs apart from drawing:
 *
 *   §4 CORE   — the pure maths: complex arithmetic → Newton iteration → which root
 *               + how fast → a classified cell.  Touches no screen, no view state.
 *   §5 VIEW   — the complex-plane window the screen samples, plus pan/zoom of it.
 *   §6 FIELD  — the cached grid of classified cells.  field_compute() is the ONLY
 *               heavy processing, and it is a PURE FUNCTION of the view — so it is
 *               gated by Scene.dirty and runs only when the view actually changes,
 *               never per frame.
 *   §8 RENDER — reads the cached field and paints glyphs.  Cheap; every frame.
 *   §7 SCENE  — orchestration: view + field + theme + the dirty flag.
 *
 * EFFECTS / DELAYS: there are none.  This is a static explorer — no animation, no
 * reveal, no auto-advance, no timer.  Pan/zoom are immediate, and changing the
 * theme is a pure recolour (it rebinds palette pairs; the field is NOT recomputed).
 *
 * Keys: q quit  arrows pan  +/- zoom  r reset  t theme  1-4 zoom to each root
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra newton_fractal.c -o newton_fractal -lncurses -lm
 *
 * Sections
 * --------
 *   §1 config   — constants, root-view presets, theme table
 *   §2 clock    — monotonic ns clock + sleep
 *   §3 color    — themeable basin palette (t/T) + HUD pairs
 *   §4 core     — complex arithmetic + Newton iteration + classify (PURE MATH)
 *   §5 view     — complex-plane window + cell→point mapping + pan/zoom (DOMAIN)
 *   §6 field    — cached classified cells, recomputed only on view change
 *   §7 scene    — orchestration: view + field + theme + dirty flag
 *   §8 render   — field → glyphs + HUD (READ-ONLY)
 *   §9 app      — main loop
 */

/* ── REFERENCES — to understand the concepts and the rendering ───────────── *
 *
 *   The algorithm — Newton's method  (§4 newton_step / newton_root)
 *   ── Cayley, A. (1879). "The Newton-Fourier Imaginary Problem." Amer. J. Math.
 *      2(1), 97.  Posed Newton's method on complex roots and asked which root each
 *      start falls to — the original statement of the Newton fractal problem.
 *   ── Press, W. H., Teukolsky, S. A., Vetterling, W. T. & Flannery, B. P. (2007).
 *      "Numerical Recipes" (3rd ed.). Cambridge.  Ch. 9: Newton–Raphson, roots in
 *      the complex plane, and when/why the iteration converges or stalls (the
 *      DENOM_MIN guard and the "never converged" case).
 *
 *   Why it looks like that — fractals & complex dynamics  (§4 classify, §5 view)
 *   ── Peitgen, H.-O. & Richter, P. H. (1986). "The Beauty of Fractals." Springer.
 *      The visual reference for Newton-method images and iteration-count colouring
 *      (exactly what cell_for_root() does).
 *   ── Peitgen, H.-O., Jürgens, H. & Saupe, D. (2004). "Chaos and Fractals" (2nd
 *      ed.). Springer.  The most pedagogical walk-through of Newton's method as a
 *      dynamical system and the fractal basins it carves — best place to start.
 *   ── Milnor, J. (2006). "Dynamics in One Complex Variable" (3rd ed.). Princeton.
 *      The rigorous theory: the basins are Fatou components and their shared
 *      boundary is a Julia set — why zooming (1-4 / +) never stops revealing detail.
 *   ── Mandelbrot, B. B. (1982). "The Fractal Geometry of Nature." Freeman.
 *      Why that boundary has fractal dimension (self-similarity at every scale).
 *
 *   Rendering  (§8)
 *   ── Gookin, D. (2007). "Programmer's Guide to NCURSES." Wiley.  The cell-
 *      drawing / colour-pair API behind §8.
 * ─────────────────────────────────────────────────────────────────────────── */

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

#define MAX_ITER     64
#define TOL          1e-5f       /* |z − root|² below this = converged       */
#define DENOM_MIN    1e-12f      /* |4z³|² below this = derivative ~0, stuck */
#define BRIGHT_FRAC  0.25f       /* converged in < this fraction of MAX_ITER = vivid */

/* Convergence-speed bands.  A point's iteration fraction t = iters/MAX_ITER picks
 * its density glyph: faster convergence (smaller t) reads as a fainter mark, so the
 * glyphs themselves shade the basins from core (slow, dense) to edge (fast, faint). */
#define SPEED_BAND_FAINT   0.15f   /* t below this → '.'  (converged fastest)   */
#define SPEED_BAND_LIGHT   0.40f   /* t below this → ':'                        */
#define SPEED_BAND_MEDIUM  0.70f   /* t below this → '+'  (at/above → '#', slowest) */

#define RENDER_NS    (1000000000LL / 30)   /* ~30 fps frame cap */
#define ZOOM_FACTOR  1.30f
#define PAN_FRAC     0.15f

#define INIT_X_MIN  (-2.0f)
#define INIT_X_MAX  ( 2.0f)
#define INIT_Y_MIN  (-1.5f)
#define INIT_Y_MAX  ( 1.5f)

#define GRID_ROWS_MAX  80
#define GRID_COLS_MAX  300

#define HUD_TOP_ROWS  2   /* rows 0..1 — data HUD (view + root legend) */
#define HUD_BOT_ROWS  1   /* last row  — action / key-hint bar         */

/* Colour-pair slots.  CP_R1D..CP_R4B are the eight basin pairs (per root: a dim
 * shade for slow convergence, a vivid one for fast), rebound by theme_apply().
 * CP_SET / CP_HUD / CP_HINT are theme-independent. */
enum {
    CP_R1D = 1,  /* root +1  dim  */
    CP_R1B,      /* root +1  vivid */
    CP_R2D,      /* root -1  dim  */
    CP_R2B,      /* root -1  vivid */
    CP_R3D,      /* root +i  dim  */
    CP_R3B,      /* root +i  vivid */
    CP_R4D,      /* root -i  dim  */
    CP_R4B,      /* root -i  vivid */
    CP_SET,      /* did not converge — black        */
    CP_HUD,      /* HUD data       — bright yellow  */
    CP_HINT,     /* HUD action bar — bright cyan    */
};

#define N_THEMES 5

/*
 * Theme — the palette for the four basins of attraction.  A Newton fractal carries
 * two independent signals at every point: WHICH root the orbit fell to, and HOW
 * FAST it got there (the iteration count).  This struct encodes both — hue = which
 * root, brightness tier = how fast — which is the classic iteration-count colouring
 * of Peitgen & Richter (1986).  Eight colours in all: 4 roots × {dim, vivid}.
 *
 * Value logic: every entry sits in the BRIGHT half of the 256-colour cube (>= ~24)
 * on purpose.  The "dim" tier is NOT decoration — it paints the slow-converging
 * points, which cluster exactly on the fractal basin boundaries (the most
 * interesting part of the image).  Pushed too dark, that boundary disappears
 * against the black background.  The four hues in a theme are chosen far apart on
 * the colour wheel so two adjacent basins never read as the same colour.
 *
 * All three arrays are indexed in BASIN ORDER — [0]=+1, [1]=−1, [2]=+i, [3]=−i —
 * the same order as ROOTS[] (§4), so basin r reads its colours straight from [r].
 */
typedef struct {
    const char *name;   /* HUD label; how the user identifies the theme when cycling (t/T) */
    int dim[4];         /* per basin: 256-colour shade for SLOW convergence (boundaries) */
    int viv[4];         /* per basin: 256-colour shade for FAST convergence (basin cores) */
    int fg8[4];         /* per basin: 8-colour fallback — one ANSI hue, the tier collapses */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* name        roots: +1   -1   +i   -i   (dim) / (vivid)          8-colour per root */
    { "Classic",  {124,  27, 178,  34}, {196,  39, 226,  46},
        { COLOR_RED, COLOR_BLUE, COLOR_YELLOW, COLOR_GREEN } },
    { "Neon",     {127,  37,  70, 166}, {201,  51, 118, 208},
        { COLOR_MAGENTA, COLOR_CYAN, COLOR_GREEN, COLOR_YELLOW } },
    { "Pastel",   {175,  74, 180,  79}, {218, 117, 223, 121},
        { COLOR_MAGENTA, COLOR_CYAN, COLOR_YELLOW, COLOR_GREEN } },
    { "Vivid",    {124,  37, 178,  92}, {196,  51, 226, 135},
        { COLOR_RED, COLOR_CYAN, COLOR_YELLOW, COLOR_MAGENTA } },
    { "Spectrum", {166,  34,  27, 127}, {208,  46,  39, 201},
        { COLOR_YELLOW, COLOR_GREEN, COLOR_BLUE, COLOR_MAGENTA } },
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  color — themeable basin palette + HUD pairs                        */
/* ===================================================================== */

/*
 * basin_pair() — the colour-pair slot for basin `root` (0..3) at a speed tier.
 * The eight basin pairs are interleaved per root (CP_R1D = dim, CP_R1B = vivid,
 * CP_R2D, CP_R2B, …): base = CP_R1D + root*2, plus 1 for the vivid pair.  This is
 * the single place that knows the layout, so cell_for_root() (which reads it) and
 * theme_apply() (which fills it) can never disagree.
 */
static int basin_pair(int root, bool fast)
{
    return CP_R1D + root * 2 + (fast ? 1 : 0);
}

/*
 * theme_apply() — bind the eight basin pairs to a theme: each root gets a dim
 * (slow-convergence) shade and a vivid (fast) one, both kept in the bright half.
 */
static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    bool truecolor = COLORS >= 256;
    for (int r = 0; r < 4; r++) {
        init_pair((short)basin_pair(r, false),
                  (short)(truecolor ? th->dim[r] : th->fg8[r]), -1);
        init_pair((short)basin_pair(r, true),
                  (short)(truecolor ? th->viv[r] : th->fg8[r]), -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(CP_SET, COLOR_BLACK, COLOR_BLACK);   /* non-converging — black */
    /* HUD pairs are theme-independent — yellow data, cyan actions */
    if (COLORS >= 256) {
        init_pair(CP_HUD, 226, -1);
        init_pair(CP_HINT, 51, -1);
    } else {
        init_pair(CP_HUD, COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN, -1);
    }
    theme_apply(0);   /* Classic at boot; Scene.theme tracks the active one after */
}

/* ===================================================================== */
/* §4  core — complex arithmetic + Newton iteration (PURE MATH)           */
/* ===================================================================== */

/*
 * Complex — one point a + b·i of the complex plane.  It is the central object of
 * the whole program, in two roles: the running ORBIT that Newton's method walks
 * (z → z − f/f′, one step at a time), and the fixed ROOTS that orbit is pulled
 * toward.  Stored as two floats because every operation below (cmul, cdiv, …) is
 * simply its real/imaginary parts written out by hand.  For the dynamics this sets
 * up — orbits, attractors, basins — see Milnor (2006).
 */
typedef struct {
    float re;   /* real part      (a) — horizontal axis of the plane */
    float im;   /* imaginary part (b) — vertical axis of the plane   */
} Complex;

static Complex cadd  (Complex a, Complex b) { return (Complex){ a.re + b.re, a.im + b.im }; }
static Complex cmul  (Complex a, Complex b) { return (Complex){ a.re*b.re - a.im*b.im,
                                                                a.re*b.im + a.im*b.re }; }
static Complex csqr  (Complex a)            { return cmul(a, a); }
static Complex cscale(Complex a, float s)   { return (Complex){ a.re*s, a.im*s }; }
static float   cnorm2(Complex a)            { return a.re*a.re + a.im*a.im; }

/* cdiv() — a / b.  Caller must ensure |b| is not ~0 (see DENOM_MIN). */
static Complex cdiv(Complex a, Complex b)
{
    float d = cnorm2(b);
    return (Complex){ (a.re*b.re + a.im*b.im) / d,
                      (a.im*b.re - a.re*b.im) / d };
}

/* The four roots of z⁴ − 1 = 0, in basin order (+1, −1, +i, −i). */
static const Complex ROOTS[4] = { {1,0}, {-1,0}, {0,1}, {0,-1} };

/*
 * newton_step() — one Newton iteration for f(z) = z⁴ − 1:
 *   z_new = z − f/f' = (3z⁴ + 1) / (4z³).
 * Returns false (leaving z untouched) if the derivative 4z³ is ~0, i.e. the orbit
 * is stuck and cannot make progress.
 */
static bool newton_step(Complex *z)
{
    Complex z2  = csqr(*z);
    Complex z3  = cmul(z2, *z);
    Complex den = cscale(z3, 4.0f);                 /* f'(z) = 4z³ */
    if (cnorm2(den) < DENOM_MIN) return false;
    Complex z4  = csqr(z2);
    Complex num = cadd(cscale(z4, 3.0f), (Complex){ 1.0f, 0.0f });   /* 3z⁴ + 1 */
    *z = cdiv(num, den);
    return true;
}

/* nearest_root() — the root z has converged to (within TOL), or −1 if none. */
static int nearest_root(Complex z)
{
    for (int r = 0; r < 4; r++) {
        Complex d = { z.re - ROOTS[r].re, z.im - ROOTS[r].im };
        if (cnorm2(d) < TOL) return r;
    }
    return -1;
}

/*
 * newton_root() — run Newton's method from start point z; return the basin (root
 * index 0..3) it converged to and the iteration count via *iters, or −1 if it
 * never converged (cycle, stall, or too slow).  Pure: same z, same result.
 */
static int newton_root(Complex z, int *iters)
{
    for (int n = 0; n < MAX_ITER; n++) {
        if (!newton_step(&z)) break;
        int r = nearest_root(z);
        if (r >= 0) { *iters = n + 1; return r; }
    }
    *iters = MAX_ITER;
    return -1;
}

/* ── classification ──────────────────────────────────────────────────── */

/*
 * Cell — the fully-classified drawing result for one screen point: everything the
 * renderer needs, and nothing about the maths that produced it.  This split is the
 * whole reason the struct exists — the expensive Newton work (§4/§6) is reduced to
 * these two small fields, the Field caches a grid of them, and the per-frame
 * renderer (§8) just blits them.  See Peitgen & Richter (1986) for the
 * (root, iteration-count) → (colour, glyph) classification this captures.
 */
typedef struct {
    uint8_t pair;   /* ncurses colour-pair slot (a CP_* value) = which basin + speed tier.
                       uint8_t: only ~11 pairs exist, so it keeps the cached grid compact. */
    char    glyph;  /* ASCII density char ('.' ':' '+' '#') encoding convergence speed,
                       or ' ' for a non-converging point (then pair == CP_SET). */
} Cell;

/* density_glyph() — convergence speed (t = iters/MAX_ITER) → a density char, faster
 * (smaller t) reading as a fainter dot.  See the SPEED_BAND_* cutoffs in §1. */
static char density_glyph(float t)
{
    if (t < SPEED_BAND_FAINT)  return '.';   /* converged fastest — faintest mark */
    if (t < SPEED_BAND_LIGHT)  return ':';
    if (t < SPEED_BAND_MEDIUM) return '+';
    return '#';                              /* converged slowest — densest mark  */
}

/*
 * cell_for_root() — turn a (root, iters) result into a Cell.  The colour pair
 * encodes which basin and whether convergence was fast (vivid) or slow (dim); the
 * glyph encodes the speed as a density.  Non-converging points are CP_SET (black).
 */
static Cell cell_for_root(int root, int iters)
{
    if (root < 0) return (Cell){ CP_SET, ' ' };
    float   t      = (float)iters / (float)MAX_ITER;
    bool    bright = (t < BRIGHT_FRAC);
    uint8_t pair   = (uint8_t)basin_pair(root, bright);
    return (Cell){ pair, density_glyph(t) };
}

/* ===================================================================== */
/* §5  view — the complex-plane window + pan/zoom (DOMAIN)                 */
/* ===================================================================== */

/*
 * View — the axis-aligned rectangle of the complex plane currently shown on screen;
 * the program's "camera".  The field samples Newton's method across exactly this
 * window, and every navigation action (pan, zoom, reset, root-preset) is just an
 * edit to these four numbers — there is no separate camera/transform state.  The
 * boot window (≈ ±2 real, ±1.5 imag) frames the whole interesting region of z⁴−1,
 * wide enough to show all four basins meeting at the origin.
 */
typedef struct {
    float xmin, xmax;   /* real-axis span: screen column 0 → xmin, last column → xmax */
    float ymin, ymax;   /* imag-axis span: screen row 0 → ymin, last row → ymax       */
} View;

static View view_default(void)
{
    return (View){ INIT_X_MIN, INIT_X_MAX, INIT_Y_MIN, INIT_Y_MAX };
}

/* the four root-zoom presets (a tight window around each root). */
static const View ROOT_VIEW[4] = {
    {  0.6f,  1.4f, -0.4f,  0.4f },   /* +1 */
    { -1.4f, -0.6f, -0.4f,  0.4f },   /* -1 */
    { -0.4f,  0.4f,  0.6f,  1.4f },   /* +i */
    { -0.4f,  0.4f, -1.4f, -0.6f },   /* -i */
};

/* view_point() — the complex coordinate at normalized position (fx, fy) in [0,1]. */
static Complex view_point(View v, float fx, float fy)
{
    return (Complex){ v.xmin + (v.xmax - v.xmin) * fx,
                      v.ymin + (v.ymax - v.ymin) * fy };
}

/* unit_coord() — map a pixel index (0..count-1) onto the unit interval [0,1] so it
 * can address view_point().  With a single cell there is no span to divide across,
 * so it samples the centre (0.5) instead of dividing by zero. */
static float unit_coord(int index, int count)
{
    return (count > 1) ? (float)index / (float)(count - 1) : 0.5f;
}

/* view_pan() — slide the window by a fraction of its own size. */
static void view_pan(View *v, float fx, float fy)
{
    float dx = (v->xmax - v->xmin) * fx;
    float dy = (v->ymax - v->ymin) * fy;
    v->xmin += dx; v->xmax += dx;
    v->ymin += dy; v->ymax += dy;
}

/* view_zoom() — scale the window about its centre (factor < 1 zooms in). */
static void view_zoom(View *v, float factor)
{
    float xc = (v->xmin + v->xmax) * 0.5f, yc = (v->ymin + v->ymax) * 0.5f;
    float hw = (v->xmax - v->xmin) * 0.5f * factor;
    float hh = (v->ymax - v->ymin) * 0.5f * factor;
    v->xmin = xc - hw; v->xmax = xc + hw;
    v->ymin = yc - hh; v->ymax = yc + hh;
}

/* ===================================================================== */
/* §6  field — cached classified cells (the only heavy processing)        */
/* ===================================================================== */

/*
 * Field — the screen's worth of classified cells, plus its active size.  It is a
 * pure cache of a View: field_compute() fills it, and the renderer just reads it.
 * Recomputing it is the one expensive thing the program does — so the Scene only
 * triggers it when the view changed (Scene.dirty), never every frame.
 */
typedef struct {
    /* Row-major grid of classified cells, indexed [row][col].  Statically sized to
       the largest terminal we support so the hot path never calls malloc (project
       rule); only the top-left rows×cols sub-rectangle is live in any given frame. */
    Cell cell[GRID_ROWS_MAX][GRID_COLS_MAX];
    int  rows;   /* live canvas height = terminal rows − HUD_TOP_ROWS − HUD_BOT_ROWS, clamped */
    int  cols;   /* live canvas width  = terminal cols, clamped to GRID_COLS_MAX             */
} Field;

static void field_resize(Field *f, int term_cols, int term_rows)
{
    int cols = term_cols;
    int rows = term_rows - HUD_TOP_ROWS - HUD_BOT_ROWS;   /* leave room for the HUD */
    f->cols = cols < 1 ? 1 : (cols > GRID_COLS_MAX ? GRID_COLS_MAX : cols);
    f->rows = rows < 1 ? 1 : (rows > GRID_ROWS_MAX ? GRID_ROWS_MAX : rows);
}

/*
 * field_compute() — evaluate Newton's method at every cell for view v and cache
 * the classified result.  A pure function of v; runs only when the view changes.
 */
static void field_compute(Field *f, View v)
{
    for (int row = 0; row < f->rows; row++) {
        float fy = unit_coord(row, f->rows);          /* this row → [0,1] on the imag axis */
        for (int col = 0; col < f->cols; col++) {
            float fx = unit_coord(col, f->cols);      /* this col → [0,1] on the real axis */
            Complex start = view_point(v, fx, fy);    /* where on the plane this cell sits  */
            int iters;
            int root = newton_root(start, &iters);    /* run Newton: which basin, how fast  */
            f->cell[row][col] = cell_for_root(root, iters);
        }
    }
}

/* ===================================================================== */
/* §7  scene — orchestration: view + field + theme + dirty                */
/* ===================================================================== */

/*
 * Scene — the complete logical state of the explorer, independent of the terminal.
 * The orchestration layer: it owns the camera (view), the cached image of that
 * camera (field), the active palette (theme), and one bookkeeping flag (dirty) that
 * binds them.  Keeping it terminal-agnostic is what lets the compute (§6) and the
 * draw (§8) stay separate: anything that moves the camera just sets dirty, and
 * scene_update() is then the single place that ever pays for a recompute — and only
 * when one is actually owed.
 */
typedef struct {
    View  view;     /* §5 the complex-plane window being explored            */
    Field field;    /* §6 cached classified cells for `view`                 */
    int   theme;    /* index into k_themes[]; a pure recolour, never a recompute */
    bool  dirty;    /* true ⇒ `view` moved and `field` is stale.  Set by every camera
                       edit (pan/zoom/reset/root/resize); cleared by scene_update()
                       once it has rebuilt the field.  This flag is the entire reason
                       idle frames are cheap. */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    s->view  = view_default();
    s->theme = 0;
    field_resize(&s->field, cols, rows);
    s->dirty = true;
}

/* scene_update() — the only non-render processing: recompute the field if (and
 * only if) the view changed.  Idle frames skip it entirely. */
static void scene_update(Scene *s)
{
    if (!s->dirty) return;
    field_compute(&s->field, s->view);
    s->dirty = false;
}

static void scene_resize(Scene *s, int cols, int rows)
{
    field_resize(&s->field, cols, rows);
    s->dirty = true;
}

/* View-changing actions — each marks the field stale. */
static void scene_pan  (Scene *s, float fx, float fy) { view_pan(&s->view, fx, fy); s->dirty = true; }
static void scene_zoom (Scene *s, float factor)       { view_zoom(&s->view, factor); s->dirty = true; }
static void scene_reset(Scene *s)                     { s->view = view_default(); s->dirty = true; }
static void scene_root (Scene *s, int i)              { s->view = ROOT_VIEW[i & 3]; s->dirty = true; }

/* Theme change is a pure recolour — rebinds palette pairs, leaves the field. */
static void scene_cycle_theme(Scene *s, int dir)
{
    s->theme = (s->theme + dir + N_THEMES) % N_THEMES;
    theme_apply(s->theme);
}

/* ===================================================================== */
/* §8  render — field → glyphs + HUD (READ-ONLY)                          */
/* ===================================================================== */

/*
 * Screen — the live terminal size, refreshed from getmaxyx() at start-up and after
 * every SIGWINCH.  Held apart from Scene on purpose: this is a property of the
 * display, not of the fractal.  The HUD clamps its text to these bounds, and the
 * field carves its drawing canvas out of them (rows minus the two HUD bars).
 */
typedef struct {
    int rows;   /* terminal height, in character cells */
    int cols;   /* terminal width,  in character cells */
} Screen;

/* render_field() — paint the cached cells below the top HUD rows.  Read-only. */
static void render_field(const Field *f)
{
    for (int row = 0; row < f->rows; row++) {
        for (int col = 0; col < f->cols; col++) {
            Cell c = f->cell[row][col];
            if (c.pair == CP_SET) continue;        /* non-converging — leave black */
            attron(COLOR_PAIR((int)c.pair) | A_BOLD);
            mvaddch(HUD_TOP_ROWS + row, col, (chtype)(unsigned char)c.glyph);
            attroff(COLOR_PAIR((int)c.pair) | A_BOLD);
        }
    }
}

/* hud_line() — one HUD line at (row, x), clamped to the terminal width. */
static void hud_line(const Screen *s, int row, int x, int pair, attr_t attr, const char *str)
{
    if (x < 0) x = 0;
    if (x >= s->cols) return;
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(row, x, "%.*s", s->cols - x, str);
    attroff(COLOR_PAIR(pair) | attr);
}

/* hud_span() — a coloured segment at (row, x), clamped; returns the next column. */
static int hud_span(const Screen *s, int row, int x, int pair, attr_t attr, const char *str)
{
    if (x >= 0 && x < s->cols) {
        attron(COLOR_PAIR(pair) | attr);
        mvprintw(row, x, "%.*s", s->cols - x, str);
        attroff(COLOR_PAIR(pair) | attr);
    }
    return x + (int)strlen(str);
}

/* hud_status_line() — row 0: title, the complex-plane window, iteration cap, theme. */
static void hud_status_line(const Screen *s, const Scene *sc)
{
    char buf[256];
    snprintf(buf, sizeof buf,
             " Newton z^4-1   x:[%.4f, %.4f]  y:[%.4f, %.4f]  iter:%d  theme:%s ",
             sc->view.xmin, sc->view.xmax, sc->view.ymin, sc->view.ymax,
             MAX_ITER, k_themes[sc->theme].name);
    hud_line(s, 0, 0, CP_HUD, A_BOLD, buf);
}

/* hud_root_legend() — row 1: the root → colour key, each label painted in its own
 * (vivid) basin colour so the legend re-colours itself with the active theme. */
static void hud_root_legend(const Screen *s)
{
    int x = 0;
    x = hud_span(s, 1, x, CP_HUD, A_NORMAL, " roots:  ");
    x = hud_span(s, 1, x, CP_R1B, A_BOLD, "+1   ");
    x = hud_span(s, 1, x, CP_R2B, A_BOLD, "-1   ");
    x = hud_span(s, 1, x, CP_R3B, A_BOLD, "+i   ");
        hud_span(s, 1, x, CP_R4B, A_BOLD, "-i ");
}

/* hud_action_bar() — bottom row: every interactive key. */
static void hud_action_bar(const Screen *s)
{
    hud_line(s, s->rows - 1, 0, CP_HINT, A_BOLD,
             " q:quit  arrows:pan  +/-:zoom  r:reset  t:theme  1-4:root-zoom ");
}

/* hud_draw — data on top, actions on the bottom; one named step per HUD region. */
static void hud_draw(const Screen *s, const Scene *sc)
{
    hud_status_line(s, sc);   /* row 0    — what you are looking at */
    hud_root_legend(s);       /* row 1    — what the colours mean   */
    hud_action_bar(s);        /* last row — what you can press      */
}

/* ===================================================================== */
/* §9  app — main loop                                                    */
/* ===================================================================== */

/*
 * App — the whole running program in one object: the Scene being explored, the
 * Screen it is drawn on, and the two flags the OS pokes asynchronously.
 */
typedef struct {
    Scene                 scene;        /* §7 view + field + theme + dirty   */
    Screen                screen;       /* §8 terminal size                  */
    volatile sig_atomic_t running;      /* cleared by SIGINT/SIGTERM or 'q'  */
    volatile sig_atomic_t need_resize;  /* set by SIGWINCH                   */
} App;

/*
 * The single global.  A signal handler is handed only an int, so the flags it
 * touches must be reachable without a parameter, and must be volatile
 * sig_atomic_t — the only type a handler may portably write and the loop read.
 * Everything else in the program is reached through this one object.
 */
static App g_app;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}

static void cleanup(void) { endwin(); }

/* install_signals() — route interrupt / terminate / resize to on_signal(). */
static void install_signals(void)
{
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);
}

/* terminal_init() — put ncurses into the raw, non-blocking, cursor-less mode the
 * render loop relies on, then build the colour palette. */
static void terminal_init(void)
{
    initscr();
    cbreak();                  /* deliver keys immediately, no line buffering   */
    noecho();
    keypad(stdscr, TRUE);      /* decode arrow keys into KEY_* codes             */
    nodelay(stdscr, TRUE);     /* getch() returns ERR instead of blocking        */
    curs_set(0);               /* hide the hardware cursor                       */
    typeahead(-1);             /* don't let pending input interrupt the diff write */
    color_init();
}

/* app_handle_key() — translate one keypress into a scene action; false = quit. */
static bool app_handle_key(Scene *sc, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case 'r': case 'R': scene_reset(sc);                 break;
    case '1': case '2': case '3': case '4': scene_root(sc, ch - '1'); break;

    case KEY_LEFT:  scene_pan(sc, -PAN_FRAC, 0); break;
    case KEY_RIGHT: scene_pan(sc, +PAN_FRAC, 0); break;
    case KEY_UP:    scene_pan(sc, 0, -PAN_FRAC); break;
    case KEY_DOWN:  scene_pan(sc, 0, +PAN_FRAC); break;

    case '+': case '=': scene_zoom(sc, 1.0f / ZOOM_FACTOR); break;
    case '-':           scene_zoom(sc, ZOOM_FACTOR);        break;

    case 't': scene_cycle_theme(sc, +1); break;
    case 'T': scene_cycle_theme(sc, -1); break;

    default: break;
    }
    return true;
}

/* app_apply_resize() — after a SIGWINCH: rebuild ncurses' screen model, re-read the
 * terminal size, and mark the field stale so the next frame recomputes at the new
 * resolution. */
static void app_apply_resize(App *app)
{
    endwin();
    refresh();
    getmaxyx(stdscr, app->screen.rows, app->screen.cols);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
}

/* app_drain_input() — apply every key buffered this frame; clears running on quit. */
static void app_drain_input(App *app)
{
    int ch;
    while ((ch = getch()) != ERR)
        if (!app_handle_key(&app->scene, ch)) { app->running = 0; break; }
}

/* app_present() — compose one frame: clear, blit the cached field, overlay the HUD,
 * and flush it all as a single terminal diff. */
static void app_present(App *app)
{
    erase();
    render_field(&app->scene.field);
    hud_draw(&app->screen, &app->scene);
    wnoutrefresh(stdscr);
    doupdate();
}

int main(void)
{
    atexit(cleanup);
    install_signals();
    terminal_init();

    App *app     = &g_app;
    app->running = 1;
    getmaxyx(stdscr, app->screen.rows, app->screen.cols);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    while (app->running) {
        if (app->need_resize) { app->need_resize = 0; app_apply_resize(app); }

        long long frame_start = clock_ns();

        app_drain_input(app);          /* 1. input   → camera edits (sets dirty)     */
        scene_update(&app->scene);     /* 2. process → recompute the field if dirty  */
        app_present(app);              /* 3. render  → blit cached field + HUD        */

        clock_sleep_ns(RENDER_NS - (clock_ns() - frame_start));   /* 4. cap to ~30 fps */
    }
    return 0;
}
