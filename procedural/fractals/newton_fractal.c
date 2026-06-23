/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * newton_fractal.c — an explorer for the Newton fractal of z^4 - 1.
 *
 * The idea: pick a starting point on the plane, then keep stepping toward a root
 * of the polynomial (the spots where it equals zero). z^4 - 1 has four of them:
 * +1, -1, +i, -i. Almost any start eventually lands on one. We colour each point
 * by WHICH root it landed on, and pick a brighter shade and denser character for
 * how FAST it got there. The borders between the four landing zones turn out to be
 * fractal — endlessly detailed no matter how far you zoom in.
 *
 * Sister file: mandelbrot.c (another escape-time fractal explorer in this folder).
 *
 * References the code can't give you:
 *   Cayley (1879), "The Newton-Fourier Imaginary Problem" — first asked which root
 *     each starting point falls to.
 *   Peitgen & Richter (1986), "The Beauty of Fractals" — the classic look, and the
 *     trick of colouring by how many steps it took.
 *   Peitgen, Jurgens & Saupe (2004), "Chaos and Fractals" — gentlest walk-through.
 *   Milnor (2006), "Dynamics in One Complex Variable" — why zooming never bottoms out.
 *
 * Keys: q quit  arrows pan  +/- zoom  r reset  t theme  1-4 zoom to each root
 * Build: gcc -std=c11 -O2 -Wall -Wextra newton_fractal.c -o newton_fractal -lncurses -lm
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

#define MAX_ITER     64          /* give up on a point after this many steps */
#define TOL          1e-5f       /* this close to a root counts as "landed"  */
#define DENOM_MIN    1e-12f      /* if the step's divisor is this tiny, the point is stuck */
#define BRIGHT_FRAC  0.25f       /* landed in under a quarter of the steps = use the bright shade */

/* How fast a point landed picks which character we draw it with: quick landers get
 * a faint dot, slow ones a dense hash. The faint-to-dense gradient is what makes the
 * landing zones look solid in the middle and frayed at their edges. Cutoffs are a
 * fraction of MAX_ITER (0 = instant, 1 = took every step). */
#define SPEED_BAND_FAINT   0.15f   /* under this → '.'  (fastest)        */
#define SPEED_BAND_LIGHT   0.40f   /* under this → ':'                   */
#define SPEED_BAND_MEDIUM  0.70f   /* under this → '+'  (else '#', slowest) */

#define RENDER_NS    (1000000000LL / 30)   /* ~30 fps frame cap */
#define ZOOM_FACTOR  1.30f
#define PAN_FRAC     0.15f

#define INIT_X_MIN  (-2.0f)
#define INIT_X_MAX  ( 2.0f)
#define INIT_Y_MIN  (-1.5f)
#define INIT_Y_MAX  ( 1.5f)

#define GRID_ROWS_MAX  80
#define GRID_COLS_MAX  300

#define HUD_TOP_ROWS  2   /* top two rows hold the info display */
#define HUD_BOT_ROWS  1   /* bottom row holds the key hints     */

/* The colour slots ncurses draws with. The first eight are the fractal colours:
 * each of the four roots gets a dim shade (for points that landed slowly) and a
 * vivid one (for points that landed fast). The theme swaps what colours these
 * point to; the last three never change with the theme. */
enum {
    CP_R1D = 1,  /* root +1  dim  */
    CP_R1B,      /* root +1  vivid */
    CP_R2D,      /* root -1  dim  */
    CP_R2B,      /* root -1  vivid */
    CP_R3D,      /* root +i  dim  */
    CP_R3B,      /* root +i  vivid */
    CP_R4D,      /* root -i  dim  */
    CP_R4B,      /* root -i  vivid */
    CP_SET,      /* never landed anywhere — black   */
    CP_HUD,      /* info text      — bright yellow  */
    CP_HINT,     /* key-hint bar   — bright cyan    */
};

#define N_THEMES 5

/*
 * Theme — one colour scheme for the picture, selectable with t/T.
 *
 * Every point on screen tells us two things: which of the four roots it landed on,
 * and how fast it got there. We show the first with colour (a different hue per
 * root) and the second with brightness (a vivid shade for fast, a dim one for slow).
 * So a theme needs eight colours: four roots, each with a dim and a vivid version.
 * This brightness-by-speed look is the classic one from Peitgen & Richter (1986).
 *
 * Two deliberate choices in the numbers below:
 *   - Even the "dim" colours are kept fairly bright (256-colour codes around 24 and
 *     up). The dim shade paints the slow points, and those cluster right on the
 *     fractal borders — the prettiest part. Too dark and that detail vanishes into
 *     the black background.
 *   - The four hues in a theme are spread far apart so two neighbouring zones never
 *     look like the same colour.
 *
 * All three arrays line up with ROOTS[] (§4): index 0 is +1, 1 is -1, 2 is +i,
 * 3 is -i. So the colour for root r is just dim[r] / viv[r].
 */
typedef struct {
    const char *name;   /* shown in the info bar so you know which theme is active */
    int dim[4];         /* per root: 256-colour code for slow landers (the borders) */
    int viv[4];         /* per root: 256-colour code for fast landers (the cores)   */
    int fg8[4];         /* per root: fallback for old 8-colour terminals — one hue,
                           dim and vivid collapse into the same colour              */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* name        roots: +1   -1   +i   -i   (dim) / (vivid)          8-colour fallback */
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
 * Picks the colour slot for a given root and speed. The eight slots run dim, vivid,
 * dim, vivid... per root, so this is the one spot that knows that order — fill and
 * read both go through here so they can't drift apart.
 */
static int basin_pair(int root, bool fast)
{
    return CP_R1D + root * 2 + (fast ? 1 : 0);
}

/* Point the eight fractal colour slots at the chosen theme's colours. */
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
    init_pair(CP_SET, COLOR_BLACK, COLOR_BLACK);   /* points that never landed: draw nothing */
    /* Info text and key hints keep their colours no matter the theme. */
    if (COLORS >= 256) {
        init_pair(CP_HUD, 226, -1);
        init_pair(CP_HINT, 51, -1);
    } else {
        init_pair(CP_HUD, COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN, -1);
    }
    theme_apply(0);   /* start on the first theme; Scene.theme tracks it from here */
}

/* ===================================================================== */
/* §4  core — complex arithmetic + Newton iteration (PURE MATH)           */
/* ===================================================================== */

/*
 * Complex — a single point on the plane, written a + b*i. This is the program's
 * main currency. It plays two roles: the wandering point Newton's method walks one
 * step at a time, and the four fixed roots it walks toward. We store the two parts
 * as plain floats and spell out the arithmetic by hand in the helpers below.
 */
typedef struct {
    float re;   /* real part      — how far left/right (the horizontal axis) */
    float im;   /* imaginary part — how far up/down    (the vertical axis)   */
} Complex;

static Complex cadd  (Complex a, Complex b) { return (Complex){ a.re + b.re, a.im + b.im }; }
static Complex cmul  (Complex a, Complex b) { return (Complex){ a.re*b.re - a.im*b.im,
                                                                a.re*b.im + a.im*b.re }; }
static Complex csqr  (Complex a)            { return cmul(a, a); }
static Complex cscale(Complex a, float s)   { return (Complex){ a.re*s, a.im*s }; }
static float   cnorm2(Complex a)            { return a.re*a.re + a.im*a.im; }

/* Divides a by b. Caller must make sure b isn't basically zero (see DENOM_MIN). */
static Complex cdiv(Complex a, Complex b)
{
    float d = cnorm2(b);
    return (Complex){ (a.re*b.re + a.im*b.im) / d,
                      (a.im*b.re - a.re*b.im) / d };
}

/* The four points where z^4 - 1 hits zero, in the order everything else uses. */
static const Complex ROOTS[4] = { {1,0}, {-1,0}, {0,1}, {0,-1} };

/*
 * Takes one Newton step: nudges z toward a nearby root using the rule
 * z <- (3z^4 + 1) / (4z^3). Returns false and leaves z alone when the divisor 4z^3
 * is basically zero — that means the point is stuck and can't move.
 */
static bool newton_step(Complex *z)
{
    Complex z2  = csqr(*z);
    Complex z3  = cmul(z2, *z);
    Complex den = cscale(z3, 4.0f);                 /* the 4z^3 part */
    if (cnorm2(den) < DENOM_MIN) return false;
    Complex z4  = csqr(z2);
    Complex num = cadd(cscale(z4, 3.0f), (Complex){ 1.0f, 0.0f });   /* the 3z^4 + 1 part */
    *z = cdiv(num, den);
    return true;
}

/* Which root z has reached (close enough to count), or -1 if none of them yet. */
static int nearest_root(Complex z)
{
    for (int r = 0; r < 4; r++) {
        Complex d = { z.re - ROOTS[r].re, z.im - ROOTS[r].im };
        if (cnorm2(d) < TOL) return r;
    }
    return -1;
}

/*
 * Starts at z and keeps stepping until it reaches a root. Returns which root (0..3)
 * and writes how many steps it took into *iters. Returns -1 if it never settled —
 * it got stuck, looped, or just ran out of steps.
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
 * Cell — what one screen point will look like: just a colour and a character. This
 * is the finished answer for a point, with all the maths boiled away. The whole
 * point of keeping it tiny is that we compute a grid of these once and then redraw
 * them cheaply every frame.
 */
typedef struct {
    uint8_t pair;   /* which colour slot to draw with (a CP_* value) — encodes the root
                       and whether it was a fast or slow lander. Only ~11 slots exist,
                       so a byte keeps the cached grid small. */
    char    glyph;  /* the character to draw ('.' ':' '+' '#' by speed), or a space for
                       a point that never landed (in which case pair is CP_SET). */
} Cell;

/* Faster landers get a fainter character, slower ones a denser one. t is the share
 * of the step budget used up; cutoffs are the SPEED_BAND_* values in §1. */
static char density_glyph(float t)
{
    if (t < SPEED_BAND_FAINT)  return '.';   /* fastest — faintest */
    if (t < SPEED_BAND_LIGHT)  return ':';
    if (t < SPEED_BAND_MEDIUM) return '+';
    return '#';                              /* slowest — densest  */
}

/*
 * Turns a "which root, how many steps" answer into a ready-to-draw Cell: colour by
 * root and speed, character by speed. Points that never landed become a blank black
 * cell.
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
 * View — the patch of the plane currently on screen, i.e. the camera. We sample
 * Newton's method across exactly this rectangle. There's no separate camera object:
 * panning, zooming, resetting, and the root presets all just edit these four edges.
 * The starting window (about -2..2 across, -1.5..1.5 up) frames everything worth
 * seeing in z^4 - 1, including all four zones meeting at the centre.
 */
typedef struct {
    float xmin, xmax;   /* left and right edges: leftmost column is xmin, rightmost is xmax */
    float ymin, ymax;   /* top and bottom edges: top row is ymin, bottom row is ymax        */
} View;

static View view_default(void)
{
    return (View){ INIT_X_MIN, INIT_X_MAX, INIT_Y_MIN, INIT_Y_MAX };
}

/* Pressing 1-4 jumps to one of these — a tight window around each root. */
static const View ROOT_VIEW[4] = {
    {  0.6f,  1.4f, -0.4f,  0.4f },   /* +1 */
    { -1.4f, -0.6f, -0.4f,  0.4f },   /* -1 */
    { -0.4f,  0.4f,  0.6f,  1.4f },   /* +i */
    { -0.4f,  0.4f, -1.4f, -0.6f },   /* -i */
};

/* Where on the plane a point sits, given how far across (fx) and down (fy) it is,
 * each from 0 (one edge) to 1 (the other). */
static Complex view_point(View v, float fx, float fy)
{
    return (Complex){ v.xmin + (v.xmax - v.xmin) * fx,
                      v.ymin + (v.ymax - v.ymin) * fy };
}

/* Turns a row or column number into a 0..1 fraction for view_point(). If there's
 * only one cell there's nothing to spread across, so just take the middle (0.5)
 * rather than dividing by zero. */
static float unit_coord(int index, int count)
{
    return (count > 1) ? (float)index / (float)(count - 1) : 0.5f;
}

/* Slides the window sideways/up by a fraction of its own width/height. */
static void view_pan(View *v, float fx, float fy)
{
    float dx = (v->xmax - v->xmin) * fx;
    float dy = (v->ymax - v->ymin) * fy;
    v->xmin += dx; v->xmax += dx;
    v->ymin += dy; v->ymax += dy;
}

/* Grows or shrinks the window around its centre (factor below 1 zooms in). */
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
 * Field — a full screen's worth of finished Cells, plus how much of it is in use.
 * It's a saved copy of the picture for one View: field_compute() fills it, the
 * renderer reads it. Filling it is the one slow job in the program, so we only redo
 * it when the view actually moved (Scene.dirty) — never on an idle frame.
 */
typedef struct {
    /* The grid, indexed [row][col]. Sized big enough for the largest terminal we
       support so we never allocate memory while running (project rule); only the
       top-left rows-by-cols corner is actually in use each frame. */
    Cell cell[GRID_ROWS_MAX][GRID_COLS_MAX];
    int  rows;   /* rows in use = terminal height minus the HUD bars, kept in range */
    int  cols;   /* columns in use = terminal width, kept within GRID_COLS_MAX      */
} Field;

static void field_resize(Field *f, int term_cols, int term_rows)
{
    int cols = term_cols;
    int rows = term_rows - HUD_TOP_ROWS - HUD_BOT_ROWS;   /* save room for the HUD */
    f->cols = cols < 1 ? 1 : (cols > GRID_COLS_MAX ? GRID_COLS_MAX : cols);
    f->rows = rows < 1 ? 1 : (rows > GRID_ROWS_MAX ? GRID_ROWS_MAX : rows);
}

/* Runs Newton's method for every cell of view v and stores the finished picture.
 * The slow part — only called when the view has changed. */
static void field_compute(Field *f, View v)
{
    for (int row = 0; row < f->rows; row++) {
        float fy = unit_coord(row, f->rows);          /* this row as a 0..1 fraction down  */
        for (int col = 0; col < f->cols; col++) {
            float fx = unit_coord(col, f->cols);      /* this column as a 0..1 fraction across */
            Complex start = view_point(v, fx, fy);    /* the plane point this cell stands for */
            int iters;
            int root = newton_root(start, &iters);    /* which root it lands on, and how fast */
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
