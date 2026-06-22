/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bifurcation.c — the logistic map's bifurcation diagram, drawn as a heat map.
 * We sweep the parameter r across the screen and, for each column, count how
 * often the orbit visits each row; busy cells glow, rare ones stay faint.
 *
 * The map and its theory: May, Nature 261 (1976); Strogatz, "Nonlinear
 * Dynamics and Chaos" ch. 10.  Palette trick from Quilez, "Palettes" (2015):
 * https://iquilezles.org/articles/palettes/
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
#include <stdio.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ── §1  config ── */

/* How many times we iterate per r-sample.  We throw away the first WARMUP
 * steps so the orbit can settle, then count the next PLOT steps.  More PLOT
 * means brighter lines and smoother chaos, at linear extra cost. */
#define WARMUP             500   /* settling steps, thrown away             */
#define PLOT              1500   /* steps we actually count                 */
#define SUBPIXEL             4   /* r-values sampled per screen column      */

/* Frame pacing. */
#define RENDER_NS  (1000000000LL / 30)

/* How far we can see along the r-axis, and how pan/zoom move that view. */
#define R_DOMAIN_LO        0.0f   /* lowest r the map makes sense for         */
#define R_DOMAIN_HI        4.0f   /* highest r the map makes sense for        */
#define R_DEFAULT_LO       2.50f  /* left edge on startup / reset             */
#define R_DEFAULT_HI       4.00f  /* right edge on startup / reset            */
#define ZOOM_FACTOR        1.25f
#define PAN_FRAC           0.12f  /* one pan step = this slice of the view    */
#define AUTO_ZOOM_SHRINK   0.997f /* view shrinks ~0.3% each frame            */
#define ZOOM_MIN_WIDTH     0.002f /* auto-zoom stops once this narrow         */

/* The famous spot where period-doubling piles up into chaos. */
#define FEIGENBAUM_R   3.5699456718695f

/* Biggest grid we ever store; a larger terminal just gets a margin. */
#define MAX_ROWS   128
#define MAX_COLS   320

/* The HUD eats some rows top and bottom; the diagram fills what's left. */
#define HUD_TOP_ROWS                2
#define HUD_BOTTOM_ROWS             1
#define HUD_DATA_COL               16   /* column where the data readout begins */

/* The r-axis labels strung along HUD row 1. */
#define N_TICKS                     5   /* how many labels                        */
#define TICK_LABEL_RIGHT_MARGIN     6   /* space kept clear on the right          */

/* Brightness levels: tier 0 = coldest/faintest, top tier = hottest. */
#define N_TIERS            8
#define BRIGHT_TIER        (N_TIERS - 1)

/* Which ncurses colour pairs we use.  The first N_TIERS change with the
 * theme; the two HUD pairs stay fixed so the chrome never shifts colour. */
#define CP_TIER_0          1
#define CP_HUD             (N_TIERS + 1)
#define CP_HINT            (N_TIERS + 2)

#define HUD_DATA_YELLOW_256   226
#define HUD_TITLE_CYAN_256     51

#define KEY_ESC               27

#define TAU               (2.0f * (float)M_PI)

/* How an (R,G,B) step maps to an xterm-256 colour code, using the 6x6x6
 * colour cube that lives at codes 16..231. */
#define ANSI_CUBE_BASE       16
#define ANSI_CUBE_R_STRIDE   36
#define ANSI_CUBE_G_STRIDE    6
#define ANSI_CUBE_MAX_STEP    5

/* On 8-colour terminals, a channel counts as "on" once it crosses this. */
#define RGB_BIT_THRESHOLD   0.5f

/* ── §2  clock ── */

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

/* ── §3  color ── */

/*
 * RGB -- one colour, each channel a float in [0, 1].
 *
 * We keep colour as floats the whole time and only round to an integer
 * terminal code at the very last moment.  The palette math (see Theme)
 * naturally works in [0, 1], so this avoids needless round-trips.  We
 * pretend the values are plain/linear; the terminal's own gamma curve
 * would shift them slightly, but at the cube's coarse 6 steps per channel
 * that shift is smaller than the rounding error anyway.
 */
typedef struct {
    float r;    /* red,   0..1 */
    float g;    /* green, 0..1 */
    float b;    /* blue,  0..1 */
} RGB;

/* Pin one channel back into [0, 1]. */
static inline float clamp01(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

/* Round to the nearest whole number (for values that are never negative). */
static inline int round_half_up(float v)
{
    return (int)(v + 0.5f);
}

/*
 * Pick the colour at position t along a theme's gradient (Quilez 2015).
 * Each channel is a cosine wave; the four control colours a,b,c,d shape
 * it.  See Theme below for what each one does.
 */
static RGB cosine_palette(const RGB *a, const RGB *b,
                          const RGB *c, const RGB *d, float t)
{
    RGB out;
    out.r = clamp01(a->r + b->r * cosf(TAU * (c->r * t + d->r)));
    out.g = clamp01(a->g + b->g * cosf(TAU * (c->g * t + d->g)));
    out.b = clamp01(a->b + b->b * cosf(TAU * (c->b * t + d->b)));
    return out;
}

/* Snap one channel to its nearest of the 6 cube steps. */
static int rgb_channel_to_cube_step(float v)
{
    int idx = round_half_up(v * (float)ANSI_CUBE_MAX_STEP);
    if (idx < 0)                   idx = 0;
    if (idx > ANSI_CUBE_MAX_STEP)  idx = ANSI_CUBE_MAX_STEP;
    return idx;
}

/* Find the closest xterm-256 colour code for a float colour. */
static int rgb_to_ansi256(RGB c)
{
    int ri = rgb_channel_to_cube_step(c.r);
    int gi = rgb_channel_to_cube_step(c.g);
    int bi = rgb_channel_to_cube_step(c.b);
    return ANSI_CUBE_BASE
         + ANSI_CUBE_R_STRIDE * ri
         + ANSI_CUBE_G_STRIDE * gi
         + bi;
}

/* Same idea for old 8-colour terminals: round each channel to on/off. */
static int rgb_to_ansi8(RGB c)
{
    int r = (c.r > RGB_BIT_THRESHOLD);
    int g = (c.g > RGB_BIT_THRESHOLD);
    int b = (c.b > RGB_BIT_THRESHOLD);
    if (!r && !g && !b) return COLOR_BLACK;
    if ( r && !g && !b) return COLOR_RED;
    if (!r &&  g && !b) return COLOR_GREEN;
    if (!r && !g &&  b) return COLOR_BLUE;
    if ( r &&  g && !b) return COLOR_YELLOW;
    if ( r && !g &&  b) return COLOR_MAGENTA;
    if (!r &&  g &&  b) return COLOR_CYAN;
    return COLOR_WHITE;
}

/*
 * Theme -- a whole colour gradient packed into four control colours
 * (Quilez 2015).  Instead of hand-picking 8 colours, we describe the
 * gradient as a cosine wave per channel and let cosine_palette sample it
 * wherever we need.  That keeps each theme tiny, lets it restretch if we
 * ever change the number of tiers, and makes tweaking a hue a one-number
 * edit.  Switching themes just recomputes the tier colours; the HUD's own
 * colours are left alone.
 */
typedef struct {
    const char *name;  /* shown on the HUD ("Matrix", "Fire", ...) */
    RGB         a;     /* the middle colour each channel swings around */
    RGB         b;     /* how far it swings (keep a+b<=1, a-b>=0; clamped anyway) */
    RGB         c;     /* how many wiggles across the gradient (0.5 = one sweep) */
    RGB         d;     /* where the wave starts -- shifts the colours along */
} Theme;

#define N_THEMES 10

static const Theme k_themes[N_THEMES] = {
    /*  name              a (middle)             b (swing)                c (wiggles)              d (start)              */
    { "Matrix",     {0.00f, 0.55f, 0.08f}, {0.00f, 0.45f, 0.08f}, {0.00f, 0.50f, 0.50f}, {0.00f, 0.50f, 0.50f} },
    { "Sunset",     {0.70f, 0.50f, 0.30f}, {0.20f, 0.50f, 0.30f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.10f, 0.20f} },
    { "Ocean",      {0.20f, 0.45f, 0.65f}, {0.20f, 0.40f, 0.35f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.30f, 0.50f} },
    { "Plasma",     {0.55f, 0.40f, 0.55f}, {0.45f, 0.40f, 0.45f}, {1.00f, 1.00f, 1.00f}, {0.00f, 0.33f, 0.67f} },
    { "Fire",       {0.55f, 0.30f, 0.15f}, {0.45f, 0.50f, 0.35f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.10f, 0.30f} },
    { "Mint",       {0.20f, 0.55f, 0.50f}, {0.20f, 0.40f, 0.40f}, {0.50f, 0.50f, 0.50f}, {0.30f, 0.50f, 0.50f} },
    { "Lavender",   {0.55f, 0.45f, 0.65f}, {0.40f, 0.35f, 0.30f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.10f, 0.20f} },
    { "Aurora",     {0.45f, 0.55f, 0.55f}, {0.40f, 0.40f, 0.40f}, {1.00f, 1.00f, 0.50f}, {0.00f, 0.50f, 0.75f} },
    { "Sepia",      {0.45f, 0.35f, 0.25f}, {0.40f, 0.35f, 0.20f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.05f, 0.10f} },
    { "Rainbow",    {0.50f, 0.50f, 0.50f}, {0.50f, 0.50f, 0.50f}, {1.00f, 1.00f, 1.00f}, {0.00f, 0.33f, 0.67f} },
};

static inline int theme_next(int cur) { return (cur + 1) % N_THEMES; }
static inline int theme_prev(int cur) { return (cur + N_THEMES - 1) % N_THEMES; }

/* Where tier i sits along the gradient, as a fraction from 0 to 1. */
static inline float tier_to_gradient_t(int i)
{
    return (float)i / (float)(N_TIERS - 1);
}

/* Compute this theme's colours and load them into the tier colour pairs. */
static void theme_apply(int theme_idx)
{
    const Theme *th = &k_themes[theme_idx];
    bool have_256   = (COLORS >= 256);
    for (int i = 0; i < N_TIERS; i++) {
        float t   = tier_to_gradient_t(i);
        RGB   rgb = cosine_palette(&th->a, &th->b, &th->c, &th->d, t);
        int   code = have_256 ? rgb_to_ansi256(rgb) : rgb_to_ansi8(rgb);
        init_pair(CP_TIER_0 + i, code, -1);
    }
}

/* Set up colours once: the fixed HUD colours, then the starting theme. */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_HUD,  HUD_DATA_YELLOW_256, -1);
        init_pair(CP_HINT, HUD_TITLE_CYAN_256,  -1);
    } else {
        init_pair(CP_HUD,  COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ── §4  diagram ── */

/*
 * Density -- a grid of counters, one per character cell, tallying how
 * often the orbit visited that cell.  This counting is the whole trick:
 * if we just drew a dot per visit we'd lose the "how often" information
 * that makes the diagram look the way it does.  Cells the orbit settles
 * on (the steady periodic lines) get hit a lot and glow bright; the
 * scatter of chaos spreads thin and stays faint; narrow windows of order
 * inside the chaos show up as bright vertical streaks.  (This is the
 * cell-grid version of a graphics accumulation buffer; same trick is used
 * by geometry/maurer_rose for drawing curves.)
 *
 * The grid is sized to the largest terminal we support (~160 KB) and
 * lives inside the App, so we never call malloc.
 */
typedef struct {
    int cells[MAX_ROWS][MAX_COLS];  /* visit count per cell, 0 = never visited */
    int peak;                       /* the largest count -- the brightest cell */
} Density;

static void density_clear(Density *d)
{
    memset(d->cells, 0, sizeof d->cells);
    d->peak = 0;
}

/* Count one visit, keeping peak up to date so we don't need a second scan. */
static inline void density_stamp(Density *d, int row, int col)
{
    int v = ++d->cells[row][col];
    if (v > d->peak) d->peak = v;
}

/*
 * Turn a raw visit count into a brightness level 0..N_TIERS-1, or -1 for
 * empty cells (the caller skips those).  We scale by logarithm, not flat
 * proportion, so the rare faint cells still show up next to the very busy
 * ones instead of washing out to nothing.
 */
static inline int density_to_tier(int d, int peak)
{
    if (d <= 0 || peak <= 0) return -1;
    float norm = logf(1.0f + (float)d) / logf(1.0f + (float)peak);
    int   tier = round_half_up(norm * (float)(N_TIERS - 1));
    if (tier < 0)         tier = 0;
    if (tier >= N_TIERS)  tier = N_TIERS - 1;
    return tier;
}

/* The character used for each brightness level, faintest to densest. */
static inline char tier_glyph(int tier)
{
    static const char ramp[N_TIERS] = {
        '.', ',', ':', ';', '+', '*', '#', '@'
    };
    return ramp[tier];
}

/* Make the very brightest level bold so the main lines really pop. */
static inline chtype tier_attr(int tier)
{
    return (tier == BRIGHT_TIER) ? A_BOLD : 0;
}

/* ── §5  scene ── */

/*
 * ViewWindow -- the stretch of the r-axis we're currently looking at,
 * given by its left and right edges.  The screen's leftmost column maps
 * to r_min, the rightmost to r_max; everything in between is spread
 * evenly.  Make the window narrower and you zoom in (the auto-zoom keeps
 * shrinking it toward FEIGENBAUM_R); slide it and you pan.  It's its own
 * little struct so the pan/zoom helpers can take just this and not reach
 * into the rest of the Scene.  Both edges always stay inside the map's
 * legal r-range, and we stop shrinking once the window gets tiny so the
 * floats don't collapse.
 */
typedef struct {
    float r_min;   /* left edge,  in r units */
    float r_max;   /* right edge, in r units */
} ViewWindow;

static void view_window_init(ViewWindow *v)
{
    v->r_min = R_DEFAULT_LO;
    v->r_max = R_DEFAULT_HI;
}

/* Keep both edges inside the map's legal r-range. */
static void view_window_clamp(ViewWindow *v)
{
    if (v->r_min < R_DOMAIN_LO) v->r_min = R_DOMAIN_LO;
    if (v->r_max > R_DOMAIN_HI) v->r_max = R_DOMAIN_HI;
}

static inline float view_window_width(const ViewWindow *v)
{
    return v->r_max - v->r_min;
}

static inline bool view_window_too_narrow(const ViewWindow *v)
{
    return view_window_width(v) <= ZOOM_MIN_WIDTH;
}

/* If a pan ran off the left edge, slide the whole window back in so it
 * keeps its width instead of getting squashed.  Does nothing if it fits. */
static void view_window_reflect_off_left(ViewWindow *v)
{
    if (v->r_min < R_DOMAIN_LO) {
        v->r_max += R_DOMAIN_LO - v->r_min;
        v->r_min  = R_DOMAIN_LO;
    }
}

/* Same fix for the right edge. */
static void view_window_reflect_off_right(ViewWindow *v)
{
    if (v->r_max > R_DOMAIN_HI) {
        v->r_min -= v->r_max - R_DOMAIN_HI;
        v->r_max  = R_DOMAIN_HI;
    }
}

/* Slide the view sideways by a fraction of its width, bouncing back off
 * the edges so panning never accidentally shrinks the view. */
static void view_window_pan(ViewWindow *v, float fraction)
{
    float w = view_window_width(v);
    v->r_min += w * fraction;
    v->r_max += w * fraction;
    view_window_reflect_off_left(v);
    view_window_reflect_off_right(v);
}

/* Zoom about the centre: factor > 1 zooms in, factor < 1 zooms out. */
static void view_window_zoom(ViewWindow *v, float factor)
{
    float c  = (v->r_min + v->r_max) * 0.5f;
    float hw = view_window_width(v) / factor * 0.5f;
    v->r_min = c - hw;
    v->r_max = c + hw;
    view_window_clamp(v);
}

/* The auto-zoom step: shrink a little and re-centre on a target r. */
static void view_window_recenter_to(ViewWindow *v, float target, float shrink)
{
    float hw = view_window_width(v) * 0.5f * shrink;
    v->r_min = target - hw;
    v->r_max = target + hw;
    view_window_clamp(v);
}

/*
 * Scene -- everything we need to draw the next frame, and nothing more.
 * The HUD text and the on-screen characters are all worked out from this.
 * The visit-count grid is the big piece (~160 KB), so the Scene lives
 * inside the one static App rather than on the stack.
 */
typedef struct {
    ViewWindow view;       /* the stretch of r-axis we're looking at      */
    Density    density;    /* visit counts, rebuilt every frame (~160 KB) */
    int        theme_idx;  /* which colour theme is active; t/T cycles it */
    bool       paused;     /* SPACE: freeze the auto-zoom (pan/zoom still work) */
} Scene;

static void scene_init(Scene *s)
{
    view_window_init(&s->view);
    density_clear(&s->density);
    s->theme_idx = 0;
    s->paused    = false;
}

/* One step of "time": just the slow auto-zoom.  The actual map iteration
 * happens at draw time, since it needs the current screen size. */
static void scene_tick(Scene *s)
{
    if (s->paused) return;
    if (view_window_too_narrow(&s->view)) return;
    view_window_recenter_to(&s->view, FEIGENBAUM_R, AUTO_ZOOM_SHRINK);
}

/* The logistic map itself: one step of x -> r*x*(1-x). */
static inline float logistic_step(float r, float x)
{
    return r * x * (1.0f - x);
}

/* Turn an orbit value (0..1) into a screen row.  Flipped so bigger x sits
 * higher up, the way the textbook diagram is drawn. */
static inline int x_to_row(float x, int plot_top, int plot_height)
{
    return plot_top + plot_height - 1
         - round_half_up(x * (float)(plot_height - 1));
}

/* For one r in one column: let the orbit settle, then count where it goes. */
static void column_accumulate(Density *dens, float r, int col,
                              int plot_top, int plot_bottom_excl)
{
    int plot_h = plot_bottom_excl - plot_top;
    float x = 0.5f;
    for (int i = 0; i < WARMUP; i++) x = logistic_step(r, x);
    for (int i = 0; i < PLOT; i++) {
        x = logistic_step(r, x);
        int row = x_to_row(x, plot_top, plot_h);
        if (row < plot_top || row >= plot_bottom_excl) continue;
        if (row >= MAX_ROWS) continue;
        density_stamp(dens, row, col);
    }
}

/* Pick the r for one of the SUBPIXEL slices inside a column.  Sampling a
 * few r-values per column instead of one smooths out the lines. */
static inline float column_to_r(int col, int sx, int n_cols,
                                float r_min, float r_max)
{
    float t = ((float)col + ((float)sx + 0.5f) / (float)SUBPIXEL)
            / (float)n_cols;
    return r_min + (r_max - r_min) * t;
}

/* Build the whole frame's visit counts: for every column, sample a few
 * r-values and tally where each orbit lands. */
static void diagram_build_density(Density *dens, const ViewWindow *v,
                                  int rows, int cols)
{
    density_clear(dens);

    int plot_top         = HUD_TOP_ROWS;
    int plot_bottom_excl = rows - HUD_BOTTOM_ROWS;
    if (plot_bottom_excl - plot_top < 2) return;

    int n_cols = (cols < MAX_COLS) ? cols : MAX_COLS;

    for (int col = 0; col < n_cols; col++) {
        for (int sx = 0; sx < SUBPIXEL; sx++) {
            float r = column_to_r(col, sx, n_cols, v->r_min, v->r_max);
            if (r <= R_DOMAIN_LO || r > R_DOMAIN_HI) continue;
            column_accumulate(dens, r, col, plot_top, plot_bottom_excl);
        }
    }
}

/* Draw one cell: pick its brightness level, then its character and colour. */
static void paint_density_cell(int row, int col, int count, int peak)
{
    int tier = density_to_tier(count, peak);
    if (tier < 0) return;
    chtype attr = COLOR_PAIR(CP_TIER_0 + tier) | tier_attr(tier);
    attron(attr);
    mvaddch(row, col, (chtype)(unsigned char)tier_glyph(tier));
    attroff(attr);
}

/* Draw every visited cell in the diagram area. */
static void render_density(const Density *dens, int rows, int cols)
{
    int row_end = rows - HUD_BOTTOM_ROWS;
    if (row_end > MAX_ROWS) row_end = MAX_ROWS;
    int col_end = (cols < MAX_COLS) ? cols : MAX_COLS;

    for (int y = HUD_TOP_ROWS; y < row_end; y++)
        for (int x = 0; x < col_end; x++)
            paint_density_cell(y, x, dens->cells[y][x], dens->peak);
}

/* Draw the diagram: recount from the current view, then paint it. */
static void scene_draw(Scene *s, int rows, int cols)
{
    diagram_build_density(&s->density, &s->view, rows, cols);
    render_density(&s->density, rows, cols);
}

/* ── §6  hud ── */

static void hud_draw_title(void)
{
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(0, 0, " [BIFURCATION] ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void hud_draw_state(const Scene *s)
{
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, HUD_DATA_COL,
             " r=[%.4f, %.4f]  dr=%.4f  t:%-9s  peak=%4d  %-7s ",
             (double)s->view.r_min, (double)s->view.r_max,
             (double)view_window_width(&s->view),
             k_themes[s->theme_idx].name, s->density.peak,
             s->paused ? "PAUSED " : "zooming");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Print the r-axis number labels evenly across the top. */
static void hud_draw_r_axis_ticks(int cols, const ViewWindow *v)
{
    float w = view_window_width(v);
    for (int i = 0; i < N_TICKS; i++) {
        float t  = (float)i / (float)(N_TICKS - 1);
        int   cx = (int)(t * (float)(cols - 1));
        float r  = v->r_min + w * t;
        if (cx < cols - TICK_LABEL_RIGHT_MARGIN)
            mvprintw(1, cx, "%.3f", r);
    }
}

/* Note the auto-zoom target, the Feigenbaum point, on the right. */
static void hud_draw_feigenbaum_annotation(int cols)
{
    char tag[40];
    snprintf(tag, sizeof tag, " r_inf=%.4f ", (double)FEIGENBAUM_R);
    int len = (int)strlen(tag);
    if (len < cols)
        mvprintw(1, cols - len, "%s", tag);
}

/* The second HUD row: the axis labels plus the right-side note. */
static void hud_draw_tick_labels(int cols, const ViewWindow *v)
{
    attron(COLOR_PAIR(CP_HUD));
    hud_draw_r_axis_ticks(cols, v);
    hud_draw_feigenbaum_annotation(cols);
    attroff(COLOR_PAIR(CP_HUD));
}

static void hud_draw_action_bar(int rows)
{
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit   SPACE:pause   r:reset   t:theme   "
             "left/right:pan   +/-:zoom ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §7  screen ── */

/*
 * Screen -- the terminal's current size, remembered so the drawing code
 * can take plain numbers and never has to talk to ncurses just to learn
 * how big the window is.  The sizes are capped at MAX_ROWS / MAX_COLS so
 * the visit-count grid can never be asked for a cell outside its fixed
 * storage; an oversized terminal just shows a margin instead of crashing.
 * Only startup and resize ever change these.
 */
typedef struct {
    int cols;   /* width  in characters */
    int rows;   /* height in characters */
} Screen;

static void screen_clamp_sizes(Screen *s)
{
    if (s->rows > MAX_ROWS) s->rows = MAX_ROWS;
    if (s->cols > MAX_COLS) s->cols = MAX_COLS;
}

static void screen_init(Screen *s)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
    screen_clamp_sizes(s);
}

static void screen_resize(Screen *s)
{
    endwin(); refresh();   /* ncurses needs this to pick up the new size */
    getmaxyx(stdscr, s->rows, s->cols);
    screen_clamp_sizes(s);
}

/* ── §8  app ── */

/*
 * App -- the whole program in one box.  There's a single static copy
 * (g_app) so the signal handlers can flip its flags without a pile of
 * loose globals.  The two flags live here, not in the Scene, because
 * "should I quit?" and "did the window resize?" are about the run loop,
 * not the math -- keeping them apart lets the Scene stay pure state.
 *
 * The flags are volatile sig_atomic_t because that's the one type a
 * signal handler is allowed to touch safely, and "volatile" stops the
 * compiler from caching the value so the loop actually notices the change.
 */
typedef struct {
    Scene                 scene;        /* the simulation state          */
    Screen                screen;       /* remembered terminal size      */
    volatile sig_atomic_t running;      /* set to 0 to quit (SIGINT/TERM) */
    volatile sig_atomic_t need_resize;  /* set to 1 on window resize      */
} App;

static App g_app;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}

static void cleanup(void) { endwin(); }

static void install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);
}

static void app_init(App *app)
{
    app->running     = 1;
    app->need_resize = 0;
    screen_init(&app->screen);
    scene_init(&app->scene);
}

/* Handle a window resize: just re-read the size; the view and counts hold. */
static void apply_resize(App *app)
{
    screen_resize(&app->screen);
    app->need_resize = 0;
}

/* One small helper per key, so the key table below reads cleanly. */

static void action_pause       (Scene *s) { s->paused = !s->paused; }
static void action_reset       (Scene *s)
{
    view_window_init(&s->view);
    s->paused = false;
}
static void action_theme_next  (Scene *s)
{
    s->theme_idx = theme_next(s->theme_idx);
    theme_apply(s->theme_idx);
}
static void action_theme_prev  (Scene *s)
{
    s->theme_idx = theme_prev(s->theme_idx);
    theme_apply(s->theme_idx);
}
static void action_pan_left    (Scene *s) { view_window_pan(&s->view, -PAN_FRAC); }
static void action_pan_right   (Scene *s) { view_window_pan(&s->view,  PAN_FRAC); }
static void action_zoom_in     (Scene *s) { view_window_zoom(&s->view, ZOOM_FACTOR); }
static void action_zoom_out    (Scene *s) { view_window_zoom(&s->view, 1.0f / ZOOM_FACTOR); }

/* Act on one keypress; returns false if it means "quit". */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case KEY_ESC:  return false;
    case ' ':            action_pause(s);        break;
    case 'r': case 'R':  action_reset(s);        break;
    case 't':            action_theme_next(s);   break;
    case 'T':            action_theme_prev(s);   break;
    case KEY_LEFT:       action_pan_left(s);     break;
    case KEY_RIGHT:      action_pan_right(s);    break;
    case '+': case '=':  action_zoom_in(s);      break;
    case '-': case '_':  action_zoom_out(s);     break;
    default: break;
    }
    return true;
}

static bool drain_input(App *app)
{
    int ch;
    while ((ch = getch()) != ERR) {
        if (!app_handle_key(app, ch)) return false;
    }
    return true;
}

/* Draw one full frame: the diagram, then the HUD on top. */
static void frame_render(App *app)
{
    erase();
    scene_draw(&app->scene, app->screen.rows, app->screen.cols);
    hud_draw_title();
    hud_draw_state(&app->scene);
    hud_draw_tick_labels(app->screen.cols, &app->scene.view);
    hud_draw_action_bar(app->screen.rows);
    wnoutrefresh(stdscr);
    doupdate();
}

/* The loop: handle resize and keys, advance, draw, then sleep to ~30 fps. */
int main(void)
{
    install_signal_handlers();

    App *app = &g_app;
    app_init(app);

    while (app->running) {
        if (app->need_resize)  apply_resize(app);
        if (!drain_input(app)) { app->running = 0; break; }

        scene_tick(&app->scene);

        long long t0 = clock_ns();
        frame_render(app);
        clock_sleep_ns(RENDER_NS - (clock_ns() - t0));
    }

    return 0;
}
