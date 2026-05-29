/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * julia_explorer.c — Interactive Julia Set Explorer
 *
 * Split-screen layout:
 *   LEFT  — Mandelbrot set (precomputed at start/resize)
 *           A crosshair marks the selected c = cr + ci·i
 *   RIGHT — Julia set for current c, redrawn every frame
 *           z → z² + c,  escape radius 2,  MAX_ITER 128
 *
 * Moving the crosshair on the Mandelbrot map with arrow keys instantly
 * redraws the Julia panel.  Every pixel of the Julia set is recomputed
 * analytically each frame, so response to keystrokes is immediate.
 *
 * Auto-wander ('a') orbits c along a slow ellipse through the Mandelbrot
 * boundary, revealing many distinct Julia morphologies automatically.
 *
 * Keys:
 *   q / ESC     quit
 *   Arrow keys  move c  (fine step 0.015)
 *   h j k l     move c  (coarse step 0.08)
 *   t / T       next / prev color theme  (Fire/Ocean/Toxic/Neon/Mono)
 *   r           reset c → −0.7 + 0.27i  (Douady rabbit)
 *   z / Z       Julia view: zoom in / out
 *   a           toggle auto-wander
 *   p / spc     pause / resume auto-wander
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra julia_explorer.c -o julia_explorer -lncurses -lm
 *
 * Reading order — built bottom-up; each section uses only earlier ones.
 *   §1  config   — constants, colour slots
 *   §2  clock    — monotonic time
 *   §3  complex  — Complex number + arithmetic (the plane z lives in)
 *   §4  escape   — escape_time(z,c): the engine BOTH panels share + level map
 *   §5  view     — ViewWindow lens: cell ↔ complex point, both directions
 *   §6  color    — themes + apply
 *   §7  scene    — Scene: cursor c, julia zoom, theme, wander, layout, Mandelbrot cache
 *   §8  render   — draw the two panels, divider, crosshair, HUD
 *   §9  app      — signals, input, main loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two views of the SAME iteration z ← z² + c, side by side.
 *                  Left  (Mandelbrot): for each pixel, c = pixel, z starts at 0.
 *                  Right (Julia):      for each pixel, z = pixel, c is fixed (the
 *                  cursor).  One escape_time() engine drives both — only what you
 *                  hold fixed changes.  The Mandelbrot panel is cached (it never
 *                  changes as you move c); the Julia panel is recomputed live.
 *
 * Math           : Mandelbrot/Julia duality, made interactive.  Each point c of
 *                  the Mandelbrot set "indexes" a whole Julia set J(c); moving
 *                  the crosshair sweeps through them.  Auto-wander walks c around
 *                  an ellipse near ∂M, where the Julia sets are richest.
 *
 * Data model     : Small structs, one idea each —
 *                    Complex      a point/number in the plane (z, c)
 *                    ViewWindow   the rectangle of the plane a panel shows
 *                    Wander       the auto-orbit state of the cursor
 *                    Layout       the split-screen geometry
 *                    MandelCache  the precomputed left panel
 *                    Scene        all of the above = the explorer's state
 *
 * Performance    : Julia panel is recomputed per-frame (W/2 × H × MAX_ITER);
 *                  the Mandelbrot panel is computed once per resize and cached,
 *                  so cursor movement stays instant.
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 * Concepts & maths
 *   [1] Douady, A. & Hubbard, J. H. (1984-85). "Étude dynamique des polynômes
 *       complexes" (the Orsay Notes).  — proves the duality this whole tool
 *       demonstrates: J(c) is connected iff c lies in the Mandelbrot set.
 *   [2] Peitgen, H.-O. & Richter, P. H. (1986). "The Beauty of Fractals."
 *       Springer.  — the clearest pictures of how a c-point "indexes" a Julia
 *       set; start here for the intuition behind the two panels.
 *   [3] Julia, G. (1918). "Mémoire sur l'itération des fonctions rationnelles."
 *       J. Math. Pures Appl. 8:47-245.  — the founding paper on iteration.
 *   [4] Mandelbrot, B. B. (1982). "The Fractal Geometry of Nature." Freeman.
 *
 * Rendering & escape-time
 *   [5] Peitgen, Jürgens & Saupe (1992). "Chaos and Fractals: New Frontiers of
 *       Science." Springer.  — the escape-time algorithm in full; §4 is a
 *       direct descendant, shared here by both panels.
 *   [6] Vepstas, L. (2004). "Renormalizing the Mandelbrot Escape."  — the
 *       fractional escape count that smooths the integer levels of §4.
 *
 * In-repo study companions
 *   [7] mandelbrot.c — the LEFT panel as a standalone (cached escape-time +
 *       preset zoom windows).
 *   [8] julia.c — the RIGHT panel as a standalone (fixed-c Julia presets +
 *       themes).  This explorer is those two married by a shared engine.
 * ─────────────────────────────────────────────────────────────────────── */

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

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define RENDER_FPS   30

enum {
    MAX_ITER        = 128,
    N_LEVELS        = 8,     /* colour bands: 0 = bg (not drawn), 1–7 = gradient */
    N_THEMES        = 5,
    ROWS_MAX        = 80,
    COLS_MAX        = 300,
    MANDEL_COLS_MAX = COLS_MAX / 2 + 2,   /* widest the left panel can get */

    CP_HUD   = 1,    /* top data lines — yellow */
    CP_DIV   = 2,    /* panel divider  — gray   */
    CP_XHAIR = 3,    /* crosshair      — yellow */
    CP_LEV1  = 4,    /* pairs CP_LEV1 .. CP_LEV1+6  →  fractal levels 1..7 */
    CP_HINT  = CP_LEV1 + N_LEVELS - 1,    /* = 11, past the level pairs — cyan actions */
};

/* |z| > 2 ⇒ the orbit escapes; compare |z|² to 4 to skip a sqrt. */
#define ESCAPE_RADIUS_SQ  4.0f

/* Terminal aspect: cell height ≈ 2 × cell width */
#define ASPECT_R    2.0f

/* Default cursor: the Douady rabbit */
#define C_INIT_RE  (-0.7f)
#define C_INIT_IM  ( 0.27f)

/* Julia view defaults */
#define J_IM_HALF_DEF  1.5f
#define J_ZOOM_IN      0.85f
#define J_ZOOM_OUT    (1.0f / J_ZOOM_IN)
#define J_IM_HALF_MIN  0.25f
#define J_IM_HALF_MAX  3.0f

/* Mandelbrot fixed view: classic framing */
#define M_RE_MIN    (-2.5f)
#define M_RE_MAX    ( 1.0f)
#define M_IM_HALF    1.25f   /* ±1.25 imaginary */

/* Interaction step sizes */
#define FINE_STEP   0.015f
#define COARSE_STEP 0.08f

/* Auto-wander: c orbits a squished ellipse (keeps it near the boundary) */
#define WANDER_R      0.72f
#define WANDER_YSHRK  0.65f
#define WANDER_SPEED  0.006f   /* rad/frame  ≈ 35 s per orbit at 30 fps */

/* fps readout: recompute over a rolling window this long */
#define FPS_WINDOW_MS  500

/* escape colouring: pixels escaping in the first 7 % of the budget → background */
#define BACKGROUND_FRAC  0.07f

/* crosshair arm half-lengths, in cells (horizontal a touch longer than vertical
 * because terminal cells are taller than wide) */
#define XHAIR_ARM_H  4
#define XHAIR_ARM_V  3

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + (int64_t)ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* frame_pace — sleep whatever is left of this frame's 1/target_fps budget, so
 * the loop holds a steady frame rate regardless of how long the work took. */
static void frame_pace(int target_fps, int64_t frame_start)
{
    clock_sleep_ns(NS_PER_SEC / target_fps - (clock_ns() - frame_start));
}

/*
 * FpsCounter — a smooth frames-per-second readout for the HUD.  Counts frames
 * over a rolling FPS_WINDOW_MS window, then divides to get the rate (per-frame
 * fps jitters too much to display).
 */
typedef struct {
    int64_t window_start;   /* clock_ns when this counting window began */
    int     frames;         /* frames counted since then                */
    double  value;          /* last computed fps — what the HUD shows   */
} FpsCounter;

static void fps_count_frame(FpsCounter *f, int64_t now)
{
    f->frames += 1;
    int64_t elapsed = now - f->window_start;
    if (elapsed >= FPS_WINDOW_MS * NS_PER_MS) {
        f->value        = (double)f->frames / ((double)elapsed / (double)NS_PER_SEC);
        f->frames       = 0;
        f->window_start = now;
    }
}

/* ===================================================================== */
/* §3  complex — the number system both panels iterate in                 */
/* ===================================================================== */

/*
 * Complex — one point in the complex plane, written z = re + im·i.
 *
 * WHY fractals are built on this: a complex number is a 2-D point (re, im) with
 * a special multiplication that both scales AND rotates.  Both panels are the
 * orbit of z ← z² + c; whether that orbit escapes to infinity or stays bounded
 * is what decides every pixel's colour.  (Iteration of complex maps: refs [3][4].)
 *
 * Naming the type lets §4's loop read like the formula instead of juggling
 * zr/zi floats — and who plays z vs c is the whole Mandelbrot/Julia difference
 * (see §4).  float is plenty for these zoom depths and faster in the per-frame
 * inner loop.
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

/* |z|² = re² + im²  (compared against ESCAPE_RADIUS_SQ) */
static inline float complex_norm_sq(Complex z)
{
    return z.re * z.re + z.im * z.im;
}

/* ===================================================================== */
/* §4  escape — the one engine that draws both panels                     */
/* ===================================================================== */

/*
 * escape_time — iterate z ← z² + c from a given z; return the number of steps
 * until |z| > 2, or MAX_ITER if it never escapes.
 *
 * This is the whole "Mandelbrot vs Julia" story in one function:
 *   Mandelbrot pixel → escape_time(z = 0,     c = pixel)   (vary c, z₀ fixed)
 *   Julia pixel      → escape_time(z = pixel, c = cursor)  (vary z₀, c fixed)
 */
static int escape_time(Complex z, Complex c)
{
    for (int n = 0; n < MAX_ITER; n++) {
        z = complex_add(complex_square(z), c);
        if (complex_norm_sq(z) > ESCAPE_RADIUS_SQ)
            return n;
    }
    return MAX_ITER;
}

/*
 * escape_level — map an escape count to a colour level.
 *   iter == MAX_ITER → 7 (inside the set — brightest, drawn bold)
 *   fast escape      → 0 (background, not drawn)
 *   otherwise        → 1..6 by fraction of MAX_ITER
 */
static uint8_t escape_level(int iter)
{
    if (iter >= MAX_ITER) return (uint8_t)(N_LEVELS - 1);  /* never escaped → inside */

    float frac = (float)iter / (float)MAX_ITER;            /* 0 = fast … →1 = slow   */
    if (frac < BACKGROUND_FRAC) return 0;                  /* far exterior → blank   */

    /* halo_pos: where in the coloured range this pixel sits, 0..1 */
    int   n_bands  = N_LEVELS - 2;                         /* drawable bands 1..6    */
    float halo_pos = (frac - BACKGROUND_FRAC) / (1.0f - BACKGROUND_FRAC);
    int   band     = 1 + (int)(halo_pos * (float)n_bands + 0.5f);
    if (band < 1)       band = 1;
    if (band > n_bands) band = n_bands;
    return (uint8_t)band;
}

/* ===================================================================== */
/* §5  view — the lens between screen cells and the complex plane          */
/* ===================================================================== */

/*
 * ViewWindow — the rectangle of the (infinite) complex plane a panel shows,
 * given by its corners.  Both panels carry one: the Mandelbrot's is fixed
 * framing, the Julia's zooms with z/Z.
 *
 * WHY corners (not centre + half-extent): the panels map a cell with the plain
 * "min + fraction·range" formula, so storing the edges keeps that mapping a
 * one-liner.  view_sample goes cell → point (to compute a pixel); view_locate
 * is the inverse, point → cell (to drop the crosshair at the cursor).
 */
typedef struct {
    float re_min, re_max;   /* left / right edge, real axis              */
    float im_min, im_max;   /* bottom / top edge; im_max maps to screen row 0 */
} ViewWindow;

/* cell (col,row) → the complex point it samples */
static Complex view_sample(const ViewWindow *v, int col, int row, int cols, int rows)
{
    int   cols1 = (cols > 1) ? cols - 1 : 1;
    int   rows1 = (rows > 1) ? rows - 1 : 1;
    float fx = (float)col / (float)cols1;
    float fy = (float)row / (float)rows1;
    return (Complex){
        .re = v->re_min + fx * (v->re_max - v->re_min),
        .im = v->im_max - fy * (v->im_max - v->im_min),   /* row 0 = top = im_max */
    };
}

/* complex point → the cell that lands on (inverse of view_sample) */
static void view_locate(const ViewWindow *v, Complex p, int cols, int rows,
                        int *out_col, int *out_row)
{
    float fx = (p.re - v->re_min) / (v->re_max - v->re_min);
    float fy = (v->im_max - p.im) / (v->im_max - v->im_min);
    *out_col = (int)(fx * (float)(cols - 1) + 0.5f);
    *out_row = (int)(fy * (float)(rows - 1) + 0.5f);
}

/* ===================================================================== */
/* §6  color — themes bound to ncurses pairs                              */
/* ===================================================================== */

/*
 * Theme table — each row is 8 xterm-256 foreground colours for levels 0..7.
 * Both panels colour a cell by its escape level, so one table restyles the
 * whole screen by swapping a row — table-driven, same idea as julia.c [8] and
 * buddhabrot.c.
 *   fg[0]  unused (level 0 = background, not drawn)
 *   fg[1..6]  the escape gradient, dim → bright
 *   fg[7]  inside the set (brightest; also drawn A_BOLD for a glow)
 *
 * Same theme set + hue families as julia.c, expanded to 7 levels.  Every level
 * sits in the BRIGHT half of the cube (no value below ~39, no 232-239 gray),
 * so even the lowest band stays visible on black.
 */
static const struct {
    const char *name;          /* shown in the HUD, e.g. "Ocean" */
    short       fg[N_LEVELS];   /* colour per escape level 0..7   */
} k_themes[N_THEMES] = {
    /*            name     bg  level1 ───────────────────────── level7 (inside) */
    { "Fire",   { -1, 160, 202, 208, 214, 220, 226, 231 } },  /* red → orange → yellow → white */
    { "Ocean",  { -1,  39,  45,  51,  87, 123, 159, 231 } },  /* blue → cyan → aqua → white     */
    { "Toxic",  { -1,  46,  82, 118, 154, 190, 226, 231 } },  /* green → lime → yellow → white  */
    { "Neon",   { -1,  99, 141, 171, 201, 207, 213, 231 } },  /* indigo → magenta → pink → white*/
    { "Mono",   { -1, 244, 246, 248, 250, 252, 254, 231 } },  /* grey ramp → white              */
};

/* Characters assigned to each level (denser → brighter → higher level) */
static const char k_chars[N_LEVELS] = { ' ', '.', ',', ':', '+', '#', '@', '*' };

/* 8-color terminal fallback palette */
static const short k_fb8[N_LEVELS] = {
    -1, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
    COLOR_GREEN, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE,
};

static void theme_apply(int t)
{
    for (int l = 1; l < N_LEVELS; l++) {
        short fg = (COLORS >= 256) ? k_themes[t].fg[l] : k_fb8[l];
        init_pair((short)(CP_LEV1 + l - 1), fg, COLOR_BLACK);
    }
}

static void colors_init(int theme)
{
    start_color();
    init_pair(CP_HUD,   (COLORS >= 256) ? 226 : COLOR_YELLOW, COLOR_BLACK);  /* yellow data  */
    init_pair(CP_HINT,  (COLORS >= 256) ?  51 : COLOR_CYAN,   COLOR_BLACK);  /* cyan actions */
    init_pair(CP_DIV,   (COLORS >= 256) ? 240 : COLOR_WHITE,  COLOR_BLACK);  /* gray divider */
    init_pair(CP_XHAIR, (COLORS >= 256) ? 226 : COLOR_YELLOW, COLOR_BLACK);  /* yellow xhair */
    theme_apply(theme);
}

/* ===================================================================== */
/* §7  scene — the whole explorer state                                   */
/* ===================================================================== */

/*
 * Wander — the cursor's auto-orbit ('a' toggles it).  When active and not
 * paused, scene_wander_step walks c around a squished ellipse (radii WANDER_R
 * across, WANDER_R·WANDER_YSHRK tall) that hugs the Mandelbrot boundary —
 * exactly where the Julia sets are most intricate (refs [1][2]) — so the right
 * panel keeps morphing through interesting shapes hands-free.
 */
typedef struct {
    bool  active;   /* is auto-wander on?                */
    bool  paused;   /* frozen by the user (space)?       */
    float phase;    /* angle around the ellipse, radians */
} Wander;

/*
 * Layout — the split-screen geometry derived from the terminal size: the two
 * panel widths, with a one-column divider between them at x = mandel_w.  Left
 * half is the Mandelbrot map, right half the live Julia set.
 */
typedef struct {
    int rows, cols;   /* terminal size (clamped to ROWS_MAX / COLS_MAX) */
    int mandel_w;     /* left  (Mandelbrot) panel width                 */
    int julia_w;      /* right (Julia) panel width                      */
} Layout;

/*
 * MandelCache — the left panel's escape levels, computed once per resize.
 *
 * WHY cache it: the Mandelbrot picture is the MAP of all c-values and never
 * changes as you move the cursor, so recomputing it every frame would be wasted
 * work.  Filling it once (scene_recompute_mandel) keeps cursor movement instant;
 * only the Julia panel — which DOES depend on c — is redrawn live each frame.
 */
typedef struct {
    ViewWindow view;                              /* fixed Mandelbrot framing      */
    int        rows, cols;                        /* cached dimensions             */
    uint8_t    level[ROWS_MAX][MANDEL_COLS_MAX];  /* escape level (0..7) per cell  */
} MandelCache;

/*
 * Scene — everything the explorer holds, in one place.  The main loop reduces
 * to: read keys into the Scene, scene_wander_step, then draw it.
 *
 * WHY bundle them: the cursor picks which Julia set, the zoom frames it, the
 * cache backs the left panel, the layout sizes both — they only make sense
 * together, and one &scene carries the lot between functions.
 */
typedef struct {
    Complex     cursor;      /* selected c = cr + ci·i — drives the Julia panel */
    float       julia_zoom;  /* Julia view half-height (imaginary units)        */
    int         theme;       /* index into k_themes                             */
    Wander      wander;      /* auto-orbit state                                */
    Layout      layout;      /* split-screen geometry                           */
    MandelCache mandel;      /* precomputed left panel                          */
} Scene;

/* The Julia panel's current view (origin-centred, aspect-corrected, zoomed). */
static ViewWindow scene_julia_view(const Scene *s)
{
    float im_half = s->julia_zoom;
    float re_half = im_half * (float)s->layout.julia_w / (float)s->layout.rows / ASPECT_R;
    return (ViewWindow){ -re_half, re_half, -im_half, im_half };
}

/* Recompute the Mandelbrot cache for the current layout. */
static void scene_recompute_mandel(Scene *s)
{
    MandelCache *m = &s->mandel;
    m->view = (ViewWindow){ M_RE_MIN, M_RE_MAX, -M_IM_HALF, M_IM_HALF };
    m->rows = s->layout.rows;
    m->cols = s->layout.mandel_w;

    for (int row = 0; row < m->rows && row < ROWS_MAX; row++)
        for (int col = 0; col < m->cols && col < MANDEL_COLS_MAX; col++) {
            Complex c = view_sample(&m->view, col, row, m->cols, m->rows);
            m->level[row][col] = escape_level(escape_time((Complex){ 0.0f, 0.0f }, c));
        }
}

/* (Re)compute the split-screen layout from a raw terminal size, then refill
 * the Mandelbrot cache. */
static void scene_layout(Scene *s, int term_rows, int term_cols)
{
    int rows = term_rows, cols = term_cols;
    if (rows < 5)        rows = 5;
    if (rows > ROWS_MAX) rows = ROWS_MAX;
    if (cols < 10)       cols = 10;
    if (cols > COLS_MAX) cols = COLS_MAX;

    s->layout.rows     = rows;
    s->layout.cols     = cols;
    s->layout.mandel_w = cols / 2;
    s->layout.julia_w  = cols - s->layout.mandel_w - 1;   /* -1 for the divider */

    scene_recompute_mandel(s);
}

/* Nudge the cursor (and cancel auto-wander — the user took the wheel). */
static void scene_move_cursor(Scene *s, float dre, float dim)
{
    s->cursor.re += dre;
    s->cursor.im += dim;
    s->wander.active = false;
}

/* Zoom the Julia view, clamped to its range. */
static void scene_zoom_julia(Scene *s, float factor)
{
    s->julia_zoom *= factor;
    if (s->julia_zoom < J_IM_HALF_MIN) s->julia_zoom = J_IM_HALF_MIN;
    if (s->julia_zoom > J_IM_HALF_MAX) s->julia_zoom = J_IM_HALF_MAX;
}

/* Snap the cursor back to the Douady rabbit. */
static void scene_reset(Scene *s)
{
    s->cursor = (Complex){ C_INIT_RE, C_INIT_IM };
    s->wander.active = false;
}

/* Cycle the theme (dir = +1 next, −1 prev). */
static void scene_cycle_theme(Scene *s, int dir)
{
    s->theme = (s->theme + dir + N_THEMES) % N_THEMES;
    theme_apply(s->theme);
}

/* Toggle auto-wander; on enable, sync the phase so the cursor doesn't jump. */
static void scene_toggle_wander(Scene *s)
{
    s->wander.active = !s->wander.active;
    if (s->wander.active)
        s->wander.phase = atan2f(s->cursor.im / WANDER_YSHRK, s->cursor.re / WANDER_R);
}

/* One step of auto-wander: advance the phase and place the cursor on the ellipse. */
static void scene_wander_step(Scene *s)
{
    if (!s->wander.active || s->wander.paused) return;
    s->wander.phase += WANDER_SPEED;
    s->cursor.re = WANDER_R * cosf(s->wander.phase);
    s->cursor.im = WANDER_R * sinf(s->wander.phase) * WANDER_YSHRK;
}

/* ===================================================================== */
/* §8  render — paint the panels, divider, crosshair and HUD              */
/* ===================================================================== */

/* attributes for a level: its colour pair, plus bold on the inside-the-set band */
static attr_t level_attr(uint8_t lev)
{
    attr_t attr = COLOR_PAIR(CP_LEV1 + (int)lev - 1);
    if (lev == N_LEVELS - 1) attr |= A_BOLD;
    return attr;
}

static chtype level_glyph(uint8_t lev)
{
    return (chtype)(unsigned char)k_chars[lev];
}

/* Left panel — straight from the cache (no recompute). */
static void mandelbrot_draw(const Scene *s)
{
    const MandelCache *m = &s->mandel;
    for (int row = 0; row < s->layout.rows; row++) {
        for (int col = 0; col < s->layout.mandel_w; col++) {
            uint8_t lev = m->level[row][col];
            if (lev == 0) continue;
            attr_t attr = level_attr(lev);
            attron(attr);
            mvaddch(row, col, level_glyph(lev));
            attroff(attr);
        }
    }
}

/* Right panel — the Julia set for the current cursor, recomputed live. */
static void julia_draw(const Scene *s)
{
    if (s->layout.julia_w <= 0 || s->layout.rows <= 0) return;

    ViewWindow v = scene_julia_view(s);
    int x0 = s->layout.mandel_w + 1;   /* first column right of the divider */

    for (int row = 0; row < s->layout.rows; row++) {
        for (int col = 0; col < s->layout.julia_w; col++) {
            Complex z   = view_sample(&v, col, row, s->layout.julia_w, s->layout.rows);
            uint8_t lev = escape_level(escape_time(z, s->cursor));
            if (lev == 0) continue;
            attr_t attr = level_attr(lev);
            attron(attr);
            mvaddch(row, x0 + col, level_glyph(lev));
            attroff(attr);
        }
    }
}

static void divider_draw(const Scene *s)
{
    attron(COLOR_PAIR(CP_DIV));
    for (int row = 0; row < s->layout.rows; row++)
        mvaddch(row, s->layout.mandel_w, '|');
    attroff(COLOR_PAIR(CP_DIV));
}

/* Crosshair marking the cursor's position on the Mandelbrot panel. */
static void crosshair_draw(const Scene *s)
{
    int xc, xr;
    view_locate(&s->mandel.view, s->cursor, s->layout.mandel_w, s->layout.rows, &xc, &xr);
    if (xc < 0 || xc >= s->layout.mandel_w || xr < 0 || xr >= s->layout.rows) return;

    attron(COLOR_PAIR(CP_XHAIR) | A_BOLD);
    for (int c = xc - XHAIR_ARM_H; c <= xc + XHAIR_ARM_H; c++)
        if (c >= 0 && c < s->layout.mandel_w && c != xc) mvaddch(xr, c, '-');
    for (int r = xr - XHAIR_ARM_V; r <= xr + XHAIR_ARM_V; r++)
        if (r >= 0 && r < s->layout.rows && r != xr) mvaddch(r, xc, '|');
    mvaddch(xr, xc, '+');
    attroff(COLOR_PAIR(CP_XHAIR) | A_BOLD);
}

/*
 * hud_line — draw one HUD line at (row, x) in colour `pair`, clamped to the
 * terminal width so a long line never wraps onto the panels.  The "%.*s"
 * precision truncates the text to the columns left from x.
 */
static void hud_line(int row, int x, int pair, int cols, const char *str)
{
    if (x < 0) x = 0;
    if (x >= cols) return;
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, x, "%.*s", cols - x, str);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/*
 * hud_draw — top is data, bottom is actions:
 *   row 0 left   MANDELBROT label + global data (theme, fps)
 *   row 0 right  JULIA label + this set's data (c, zoom, run state)
 *   row rows-1   every interactive key
 * Each line is clamped to the terminal width by hud_line.
 */
static void hud_draw(const Scene *s, double fps)
{
    const Layout *L = &s->layout;
    char buf[128];

    /* top-left — Mandelbrot panel + global data */
    snprintf(buf, sizeof buf, " MANDELBROT   theme:%s  %.0f fps ",
             k_themes[s->theme].name, fps);
    hud_line(0, 1, CP_HUD, L->cols, buf);

    /* top-right — Julia panel + the current set's data */
    float zoom = J_IM_HALF_DEF / s->julia_zoom;
    const char *state = s->wander.paused ? "PAUSED"
                      : (s->wander.active ? "wander" : "manual");
    snprintf(buf, sizeof buf, " JULIA  c=%.3f%+.3fi  %.2fx  %s ",
             (double)s->cursor.re, (double)s->cursor.im, (double)zoom, state);
    hud_line(0, L->mandel_w + 2, CP_HUD, L->cols, buf);

    /* bottom — actions */
    int last = (L->rows > 1) ? L->rows - 1 : 0;
    hud_line(last, 0, CP_HINT, L->cols,
        " q:quit  arrows/hjkl:move c  z/Z:zoom  t/T:theme  a:wander  spc:pause  r:reset ");
}

/* ===================================================================== */
/* §9  app — signals, input, main loop                                    */
/* ===================================================================== */

static volatile sig_atomic_t g_should_quit   = 0;
static volatile sig_atomic_t g_should_resize = 0;

static void on_signal(int s)
{
    if (s == SIGWINCH) g_should_resize = 1;
    else               g_should_quit   = 1;
}

static void cleanup(void) { endwin(); }

/* Map one keypress to a scene action.  Returns false to quit. */
static bool app_handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    /* fine c movement — arrow keys */
    case KEY_UP:    scene_move_cursor(s,  0.0f,       +FINE_STEP); break;
    case KEY_DOWN:  scene_move_cursor(s,  0.0f,       -FINE_STEP); break;
    case KEY_LEFT:  scene_move_cursor(s, -FINE_STEP,   0.0f);      break;
    case KEY_RIGHT: scene_move_cursor(s, +FINE_STEP,   0.0f);      break;

    /* coarse c movement — hjkl (vim: h=left j=down k=up l=right) */
    case 'k': scene_move_cursor(s,  0.0f,        +COARSE_STEP); break;
    case 'j': scene_move_cursor(s,  0.0f,        -COARSE_STEP); break;
    case 'h': scene_move_cursor(s, -COARSE_STEP,  0.0f);        break;
    case 'l': scene_move_cursor(s, +COARSE_STEP,  0.0f);        break;

    case 't': scene_cycle_theme(s, +1); break;
    case 'T': scene_cycle_theme(s, -1); break;

    case 'r': scene_reset(s); break;

    case 'z': scene_zoom_julia(s, J_ZOOM_IN);  break;
    case 'Z': scene_zoom_julia(s, J_ZOOM_OUT); break;

    case 'a': case 'A': scene_toggle_wander(s); break;

    case 'p': case 'P': case ' ':
        s->wander.paused = !s->wander.paused;
        break;

    default: break;
    }
    return true;
}

/* app_resize — re-measure the terminal and rebuild the layout + Mandelbrot cache. */
static void app_resize(Scene *s)
{
    endwin();
    refresh();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    scene_layout(s, rows, cols);
    g_should_resize = 0;
}

/* app_poll_input — drain all pending keys this frame; quit if a key says so. */
static void app_poll_input(Scene *s)
{
    int ch;
    while ((ch = getch()) != ERR)
        if (!app_handle_key(s, ch)) { g_should_quit = 1; break; }
}

/* app_draw — one full frame: both panels, divider, crosshair, then the HUD. */
static void app_draw(const Scene *s, double fps)
{
    erase();
    mandelbrot_draw(s);
    divider_draw(s);
    julia_draw(s);
    crosshair_draw(s);
    hud_draw(s, fps);            /* HUD last, overlaying the fractals */
    wnoutrefresh(stdscr);
    doupdate();
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);

    /* static (BSS) — Scene holds the Mandelbrot cache; keep it off the stack */
    static Scene scene = {
        .cursor     = { C_INIT_RE, C_INIT_IM },
        .julia_zoom = J_IM_HALF_DEF,
        .theme      = 0,
    };
    colors_init(scene.theme);

    int term_rows, term_cols;
    getmaxyx(stdscr, term_rows, term_cols);
    scene_layout(&scene, term_rows, term_cols);

    FpsCounter fps = { .window_start = clock_ns() };

    /* Main loop — one pass per frame:
     *   1. apply a pending resize
     *   2. handle pending keys
     *   3. step auto-wander
     *   4. draw the frame
     *   5. update the fps readout, then pace to RENDER_FPS
     */
    while (!g_should_quit) {

        if (g_should_resize) app_resize(&scene);

        int64_t frame_start = clock_ns();

        app_poll_input(&scene);
        scene_wander_step(&scene);
        app_draw(&scene, fps.value);

        fps_count_frame(&fps, clock_ns());
        frame_pace(RENDER_FPS, frame_start);
    }

    return 0;
}
