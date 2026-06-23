/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * julia_explorer.c — drag a point around the Mandelbrot set on the left and
 * watch its matching Julia set redraw on the right, side by side.
 *
 * Both pictures come from repeating the same step, z = z*z + c. The trick:
 * the Mandelbrot set is a map of every possible choice of c, and each point
 * on that map "owns" one Julia set. So picking a point here picks a picture
 * there. See julia.c and mandelbrot.c for each panel on its own.
 *
 * Why c near the edge of the Mandelbrot set gives the prettiest Julia sets:
 * Douady & Hubbard, "Étude dynamique des polynômes complexes" (1984-85).
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

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define RENDER_FPS   30

enum {
    MAX_ITER        = 128,   /* give up after this many steps and call it "inside" */
    N_LEVELS        = 8,     /* 0 = background (left blank), 1..7 = the colour ramp */
    N_THEMES        = 5,
    ROWS_MAX        = 80,
    COLS_MAX        = 300,
    MANDEL_COLS_MAX = COLS_MAX / 2 + 2,   /* widest the left panel can ever get */

    /* ncurses colour-pair slot numbers we reserve */
    CP_HUD   = 1,    /* top text lines */
    CP_DIV   = 2,    /* the divider between the two panels */
    CP_XHAIR = 3,    /* the crosshair marker */
    CP_LEV1  = 4,    /* fractal levels 1..7 take slots CP_LEV1 .. CP_LEV1+6 */
    CP_HINT  = CP_LEV1 + N_LEVELS - 1,    /* bottom key-hint line; sits just past the level slots */
};

/* A point counts as "escaped" once it gets farther than 2 from the origin.
 * We compare distance-squared to 4 so we never have to take a square root. */
#define ESCAPE_RADIUS_SQ  4.0f

/* Terminal cells are about twice as tall as they are wide; we stretch the
 * picture horizontally to undo that and keep the fractal from looking squashed. */
#define ASPECT_R    2.0f

/* Where the crosshair starts: the classic "Douady rabbit" Julia set. */
#define C_INIT_RE  (-0.7f)
#define C_INIT_IM  ( 0.27f)

/* How much of the plane the Julia panel shows, and how zoom in/out scales it. */
#define J_IM_HALF_DEF  1.5f
#define J_ZOOM_IN      0.85f
#define J_ZOOM_OUT    (1.0f / J_ZOOM_IN)
#define J_IM_HALF_MIN  0.25f
#define J_IM_HALF_MAX  3.0f

/* The slice of the plane the left (Mandelbrot) panel shows — the usual framing
 * that fits the whole set on screen. */
#define M_RE_MIN    (-2.5f)
#define M_RE_MAX    ( 1.0f)
#define M_IM_HALF    1.25f

/* How far one keypress nudges the crosshair: small for arrows, big for hjkl. */
#define FINE_STEP   0.015f
#define COARSE_STEP 0.08f

/* Auto-wander drives the crosshair around an oval that hugs the set's edge,
 * where the Julia sets are most interesting. R is its width, YSHRK squashes it
 * vertically, SPEED is how fast it travels (one lap takes roughly 35 s). */
#define WANDER_R      0.72f
#define WANDER_YSHRK  0.65f
#define WANDER_SPEED  0.006f

/* The fps number is averaged over this many milliseconds so it stops flickering. */
#define FPS_WINDOW_MS  500

/* Points that escape almost instantly (first 7% of the step budget) are the
 * boring far-away region — we leave them blank instead of colouring them. */
#define BACKGROUND_FRAC  0.07f

/* Half-length of the crosshair arms, in cells. The horizontal arm is a little
 * longer so the cross looks even, since cells are taller than they are wide. */
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

/* If this frame finished early, sleep off the rest of its time slice so the
 * program runs at a steady speed no matter how fast the machine is. */
static void frame_pace(int target_fps, int64_t frame_start)
{
    clock_sleep_ns(NS_PER_SEC / target_fps - (clock_ns() - frame_start));
}

/*
 * Keeps a steady frames-per-second number for the on-screen display. We count
 * how many frames go by over a short window, then divide; measuring frame by
 * frame would make the number jump around too much to read.
 */
typedef struct {
    int64_t window_start;   /* when the current counting window started */
    int     frames;         /* frames seen since then                   */
    double  value;          /* the latest fps, i.e. what we show        */
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
/* §3  complex — the numbers both panels work with                        */
/* ===================================================================== */

/*
 * A "complex number" is really just a point on a 2-D plane: one number for
 * how far across (re) and one for how far up (im). The whole fractal comes from
 * repeating one step on these points, z = z*z + c, and watching whether the
 * point shoots off to infinity or stays put. We give it a name so the step
 * below reads like the math instead of four loose floats. float is accurate
 * enough here and keeps the tight inner loop fast.
 */
typedef struct {
    float re;   /* how far across (left-right) */
    float im;   /* how far up    (down-up)     */
} Complex;

static inline Complex complex_add(Complex a, Complex b)
{
    return (Complex){ a.re + b.re, a.im + b.im };
}

/* Squaring a complex number twists and stretches it; this is what makes the
 * fractal's swirly shapes. */
static inline Complex complex_square(Complex z)
{
    return (Complex){ z.re * z.re - z.im * z.im, 2.0f * z.re * z.im };
}

/* Distance from the origin, but squared (we skip the square root and compare
 * against ESCAPE_RADIUS_SQ instead). */
static inline float complex_norm_sq(Complex z)
{
    return z.re * z.re + z.im * z.im;
}

/* ===================================================================== */
/* §4  escape — the one routine that draws both panels                    */
/* ===================================================================== */

/*
 * Repeat the step z = z*z + c and count how many steps it takes before the
 * point runs off to infinity. If it never does within our budget, return the
 * max — that point is "inside" the set.
 *
 * The only difference between the two panels is what you keep fixed:
 *   Mandelbrot: start z at 0, let c be the pixel  (the map of every c)
 *   Julia:      let z be the pixel, fix c at the crosshair  (one chosen c)
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
 * Turn a step count into a colour band (0..7): never escaped means inside the
 * set (band 7, the brightest), escaped right away means blank background
 * (band 0), and everything in between gets a band along the gradient.
 */
static uint8_t escape_level(int iter)
{
    if (iter >= MAX_ITER) return (uint8_t)(N_LEVELS - 1);  /* stayed put → inside the set */

    float frac = (float)iter / (float)MAX_ITER;            /* 0 = escaped fast, 1 = escaped slow */
    if (frac < BACKGROUND_FRAC) return 0;                  /* escaped almost instantly → blank   */

    /* Spread the rest across the drawable bands (1..6). */
    int   n_bands  = N_LEVELS - 2;
    float halo_pos = (frac - BACKGROUND_FRAC) / (1.0f - BACKGROUND_FRAC);
    int   band     = 1 + (int)(halo_pos * (float)n_bands + 0.5f);
    if (band < 1)       band = 1;
    if (band > n_bands) band = n_bands;
    return (uint8_t)band;
}

/* ===================================================================== */
/* §5  view — converting between screen cells and plane coordinates        */
/* ===================================================================== */

/*
 * Which rectangle of the (endless) plane a panel is looking at, stored as its
 * four edges. Each panel has one: the left panel's never moves, the right one's
 * grows and shrinks when you zoom. We keep the edges (not a centre + size)
 * because that makes the cell-to-point math a single clean line. view_sample
 * goes screen cell -> plane point; view_locate goes the other way, so we can
 * find where on screen the crosshair belongs.
 */
typedef struct {
    float re_min, re_max;   /* left and right edges  */
    float im_min, im_max;   /* bottom and top edges; the top edge is screen row 0 */
} ViewWindow;

/* Which point on the plane does this screen cell land on? */
static Complex view_sample(const ViewWindow *v, int col, int row, int cols, int rows)
{
    int   cols1 = (cols > 1) ? cols - 1 : 1;
    int   rows1 = (rows > 1) ? rows - 1 : 1;
    float fx = (float)col / (float)cols1;
    float fy = (float)row / (float)rows1;
    return (Complex){
        .re = v->re_min + fx * (v->re_max - v->re_min),
        .im = v->im_max - fy * (v->im_max - v->im_min),   /* row 0 is the top, so it maps to the top edge */
    };
}

/* Which screen cell does this point on the plane land on? (the reverse of above) */
static void view_locate(const ViewWindow *v, Complex p, int cols, int rows,
                        int *out_col, int *out_row)
{
    float fx = (p.re - v->re_min) / (v->re_max - v->re_min);
    float fy = (v->im_max - p.im) / (v->im_max - v->im_min);
    *out_col = (int)(fx * (float)(cols - 1) + 0.5f);
    *out_row = (int)(fy * (float)(rows - 1) + 0.5f);
}

/* ===================================================================== */
/* §6  color — the colour themes                                          */
/* ===================================================================== */

/*
 * One colour theme per row: a name plus a colour for each of the 8 bands.
 * Since every cell is coloured by its band, changing the whole look is just a
 * matter of switching to another row.
 *   fg[0]  unused (band 0 is background, never drawn)
 *   fg[1..6]  the gradient, faint up to bright
 *   fg[7]  the inside of the set (brightest, and drawn bold so it glows)
 * Every colour is from the bright half of the palette so even the faintest band
 * still shows up against a black terminal.
 */
static const struct {
    const char *name;          /* what shows in the top bar, e.g. "Ocean" */
    short       fg[N_LEVELS];   /* the colour for each band 0..7           */
} k_themes[N_THEMES] = {
    { "Fire",   { -1, 160, 202, 208, 214, 220, 226, 231 } },  /* red to orange to yellow to white */
    { "Ocean",  { -1,  39,  45,  51,  87, 123, 159, 231 } },  /* blue to cyan to white            */
    { "Toxic",  { -1,  46,  82, 118, 154, 190, 226, 231 } },  /* green to yellow to white         */
    { "Neon",   { -1,  99, 141, 171, 201, 207, 213, 231 } },  /* purple to pink to white          */
    { "Mono",   { -1, 244, 246, 248, 250, 252, 254, 231 } },  /* grey to white                    */
};

/* The character drawn for each band — busier marks for the higher bands. */
static const char k_chars[N_LEVELS] = { ' ', '.', ',', ':', '+', '#', '@', '*' };

/* Stand-in colours for old terminals that only have 8 colours. */
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
    init_pair(CP_HUD,   (COLORS >= 256) ? 226 : COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_HINT,  (COLORS >= 256) ?  51 : COLOR_CYAN,   COLOR_BLACK);
    init_pair(CP_DIV,   (COLORS >= 256) ? 240 : COLOR_WHITE,  COLOR_BLACK);
    init_pair(CP_XHAIR, (COLORS >= 256) ? 226 : COLOR_YELLOW, COLOR_BLACK);
    theme_apply(theme);
}

/* ===================================================================== */
/* §7  scene — everything the explorer keeps track of                     */
/* ===================================================================== */

/*
 * The "hands-free tour" state (press 'a' to turn it on). While it's running it
 * slides the crosshair around an oval that follows the edge of the Mandelbrot
 * set — the spot where the Julia sets are the most interesting — so the right
 * panel keeps changing shape on its own.
 */
typedef struct {
    bool  active;   /* is the tour turned on?              */
    bool  paused;   /* did the user freeze it (spacebar)?  */
    float phase;    /* how far around the oval we are now  */
} Wander;

/*
 * How the screen is split into two panels, worked out from the terminal size.
 * The left panel is the Mandelbrot map, the right is the live Julia set, with a
 * single column between them for the divider line.
 */
typedef struct {
    int rows, cols;   /* terminal size (capped at ROWS_MAX / COLS_MAX) */
    int mandel_w;     /* width of the left  (Mandelbrot) panel         */
    int julia_w;      /* width of the right (Julia) panel              */
} Layout;

/*
 * The finished left panel, saved so we don't redraw it constantly. The
 * Mandelbrot picture is the map of every possible c and doesn't change as you
 * move the crosshair, so we work it out once (and again on resize) and just
 * reuse it. That's what keeps the crosshair feeling instant — only the right
 * panel, which actually depends on c, is recomputed each frame.
 */
typedef struct {
    ViewWindow view;                              /* which part of the plane it shows */
    int        rows, cols;                        /* the size it was computed at      */
    uint8_t    level[ROWS_MAX][MANDEL_COLS_MAX];  /* the colour band (0..7) of every cell */
} MandelCache;

/*
 * All of the explorer's state in one bundle. Each frame the main loop just
 * reads keys into this, takes one tour step, and draws it. They belong together:
 * the crosshair chooses the Julia set, the zoom frames it, the cache is the left
 * panel, the layout sizes both — and passing one &scene hands the lot around.
 */
typedef struct {
    Complex     cursor;      /* the chosen c — this is what picks the Julia set */
    float       julia_zoom;  /* how zoomed in the Julia panel is                */
    int         theme;       /* which colour theme is active                    */
    Wander      wander;      /* the hands-free tour state                       */
    Layout      layout;      /* how the screen is split                         */
    MandelCache mandel;      /* the saved left panel                            */
} Scene;

/* Work out the slice of the plane the Julia panel should show right now,
 * centred on the origin and adjusted for zoom and cell shape. */
static ViewWindow scene_julia_view(const Scene *s)
{
    float im_half = s->julia_zoom;
    float re_half = im_half * (float)s->layout.julia_w / (float)s->layout.rows / ASPECT_R;
    return (ViewWindow){ -re_half, re_half, -im_half, im_half };
}

/* Draw the whole left panel once and save it. */
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

/* Work out the panel sizes from the terminal size, then redraw the left panel. */
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
    s->layout.julia_w  = cols - s->layout.mandel_w - 1;   /* one column goes to the divider */

    scene_recompute_mandel(s);
}

/* Moving the crosshair by hand also stops the hands-free tour. */
static void scene_move_cursor(Scene *s, float dre, float dim)
{
    s->cursor.re += dre;
    s->cursor.im += dim;
    s->wander.active = false;
}

static void scene_zoom_julia(Scene *s, float factor)
{
    s->julia_zoom *= factor;
    if (s->julia_zoom < J_IM_HALF_MIN) s->julia_zoom = J_IM_HALF_MIN;
    if (s->julia_zoom > J_IM_HALF_MAX) s->julia_zoom = J_IM_HALF_MAX;
}

static void scene_reset(Scene *s)
{
    s->cursor = (Complex){ C_INIT_RE, C_INIT_IM };
    s->wander.active = false;
}

/* dir is +1 for the next theme, -1 for the previous one. */
static void scene_cycle_theme(Scene *s, int dir)
{
    s->theme = (s->theme + dir + N_THEMES) % N_THEMES;
    theme_apply(s->theme);
}

/* When the tour starts, line it up with where the crosshair already is so it
 * doesn't suddenly jump. */
static void scene_toggle_wander(Scene *s)
{
    s->wander.active = !s->wander.active;
    if (s->wander.active)
        s->wander.phase = atan2f(s->cursor.im / WANDER_YSHRK, s->cursor.re / WANDER_R);
}

/* Move the crosshair one notch further around the tour's oval. */
static void scene_wander_step(Scene *s)
{
    if (!s->wander.active || s->wander.paused) return;
    s->wander.phase += WANDER_SPEED;
    s->cursor.re = WANDER_R * cosf(s->wander.phase);
    s->cursor.im = WANDER_R * sinf(s->wander.phase) * WANDER_YSHRK;
}

/* ===================================================================== */
/* §8  render — drawing the panels, divider, crosshair and text           */
/* ===================================================================== */

/* The colour for a band, plus bold for the brightest band so the inside glows. */
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

/* Left panel — just copy out the saved picture. */
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

/* Right panel — the Julia set for wherever the crosshair is, drawn fresh. */
static void julia_draw(const Scene *s)
{
    if (s->layout.julia_w <= 0 || s->layout.rows <= 0) return;

    ViewWindow v = scene_julia_view(s);
    int x0 = s->layout.mandel_w + 1;   /* the first column past the divider */

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

/* Draw the crosshair on the left panel to show which c is selected. */
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
 * Print one line of text, cut off at the right edge of the screen so a long
 * line can't wrap down over the fractal. (The "%.*s" trick limits the text to
 * however many columns are left.)
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
 * The status text: a label and info over each panel along the top row, and the
 * list of keys along the bottom row.
 */
static void hud_draw(const Scene *s, double fps)
{
    const Layout *L = &s->layout;
    char buf[128];

    /* over the left panel: theme and frame rate */
    snprintf(buf, sizeof buf, " MANDELBROT   theme:%s  %.0f fps ",
             k_themes[s->theme].name, fps);
    hud_line(0, 1, CP_HUD, L->cols, buf);

    /* over the right panel: the chosen c, zoom, and whether the tour is running */
    float zoom = J_IM_HALF_DEF / s->julia_zoom;
    const char *state = s->wander.paused ? "PAUSED"
                      : (s->wander.active ? "wander" : "manual");
    snprintf(buf, sizeof buf, " JULIA  c=%.3f%+.3fi  %.2fx  %s ",
             (double)s->cursor.re, (double)s->cursor.im, (double)zoom, state);
    hud_line(0, L->mandel_w + 2, CP_HUD, L->cols, buf);

    /* the keys, along the bottom */
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

/* Do whatever a single keypress asks for. Returns false when it's time to quit. */
static bool app_handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    /* arrow keys: nudge the crosshair a little */
    case KEY_UP:    scene_move_cursor(s,  0.0f,       +FINE_STEP); break;
    case KEY_DOWN:  scene_move_cursor(s,  0.0f,       -FINE_STEP); break;
    case KEY_LEFT:  scene_move_cursor(s, -FINE_STEP,   0.0f);      break;
    case KEY_RIGHT: scene_move_cursor(s, +FINE_STEP,   0.0f);      break;

    /* hjkl: nudge the crosshair a lot (vim layout: h left, j down, k up, l right) */
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
