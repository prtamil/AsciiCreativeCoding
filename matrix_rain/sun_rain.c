/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sun_rain.c — a '@' at screen centre, with 180 streams of Matrix-style
 * glyphs shooting outward in every direction like a solar corona.
 *
 * Sister files: matrix_rain.c (the same shimmer trick on plain vertical
 * streams) and pulsar_rain.c (the same polar machinery, but the rays
 * rotate instead of flying outward).
 */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ── */

enum {
    TARGET_FPS = 60,
};

enum {
    /* How many streams fan out from the core. 180 gives 2° between
     * neighbours — dense enough to read as a solid corona. */
    N_RAYS    = 180,

    /* How many glyphs long each stream's tail is. */
    RAY_TRAIL = 16,

    /* Where the brightness bands fall along a tail (see dist_attr):
     * the first few cells are bright, the middle normal, the rest dim. */
    TRAIL_HOT_END   = 2,
    TRAIL_WARM_END  = RAY_TRAIL / 2,
};

/* Terminal cells are about twice as tall as they are wide, so a circle
 * drawn naively comes out as a tall ellipse. We squish the vertical
 * direction by this much to make the corona look round. */
#define ASPECT  0.45f

/* Trails stop drawing this close to the centre so they never paint over
 * the '@' core, which sits at distance 0. */
#define CORE_RESERVED_RADIUS  1.0f

/* Each stream picks a random speed in this range when it spawns, so
 * fast rays race to the edge while slow ones still hug the core. */
#define SPEED_MIN_CPS  30.0f
#define SPEED_MAX_CPS  80.0f

/* How far back each ray starts, as a fraction of max_r. A ray spawns
 * already "below" the core and takes a moment to emerge, so the screen
 * fills in smoothly over the first second or two instead of all at once. */
#define STAGGER_FRAC  0.55f

/* Each frame, every glyph in a tail has a 1-in-4 chance of staying put;
 * the other 75% get rerolled. That flicker is the Matrix-rain shimmer. */
#define SHIMMER_KEEP_ONE_IN  4

/* ncurses colour-pair slots. 1..5 are the theme-tinted tail bands;
 * head and core are always white; HUD pairs never change with theme. */
enum {
    SHADE_FADE     = 1,
    SHADE_DARK,
    SHADE_MID,
    SHADE_BRIGHT,
    SHADE_HOT,
    SHADE_HEAD,
    SHADE_CORE,
    PAIR_HUD,
    PAIR_HINT,
};

/* Longest frame we'll trust. A hiccup (window drag, debugger pause) can
 * make one frame huge; clamping it stops the sim from lurching forward. */
#define DT_CAP_SEC  0.10f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

#define HUD_BUF_LEN  72

/* ── §2 clock ── */

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

/* ── §3 color ── */

/*
 * Theme — one named colour scheme for the ray trails.
 *
 * A tail fades from a bright head to a dim tip across 5 brightness
 * bands. dist_attr (§4) decides which band each cell falls in; this
 * struct just holds the colour for each band, brightest last. The 't'
 * key cycles the presets below and theme_apply re-tints the pairs in
 * place, so rays already on screen adopt the new look instantly.
 *
 * Two palettes because rich terminals get a fine 256-colour ramp while
 * old 8-colour ones fall back to fg_8 (the nearest plain ANSI colour,
 * accepting that some bands collapse together).
 *
 * The head and the core aren't in here — they're always white, on
 * purpose, so they stay sharp against any theme.
 *
 * Members
 *   name    label shown in the HUD ("solar", "green", …); never NULL.
 *   fg[]    5 colours for a 256-colour terminal, dimmest to brightest.
 *           Kept in the bright half of the palette (>= 24) so even the
 *           dimmest band stays visible on a black background — see
 *           CLAUDE.md "Theme Palette Brightness".
 *   fg_8[]  same 5 bands for an 8-colour terminal.
 *
 * Presets: solar (amber/gold), green (Matrix), nova (blue→white),
 *          plasma (purple/magenta), fire (red/orange).
 */
typedef struct {
    const char *name;
    int         fg  [5];
    int         fg_8[5];
} Theme;

static const Theme k_themes[] = {
    { "solar",
      { 130, 166, 202, 214, 220 },
      { COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW  } },
    { "green",
      {  28,  34,  40,  46,  82 },
      { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN   } },
    { "nova",
      {  24,  33,  51, 159, 255 },
      { COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,    COLOR_WHITE   } },
    { "plasma",
      {  53,  57,  93, 129, 201 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA } },
    { "fire",
      {  52,  88, 124, 160, 196 },
      { COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_RED     } },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static bool g_has_256 = false;

/* Re-tints the tail bands plus head/core for the chosen theme. The HUD
 * pairs are left alone on purpose — they must stay the same colour no
 * matter which theme is active. */
static void theme_apply(int idx)
{
    const int *fg = g_has_256 ? k_themes[idx].fg : k_themes[idx].fg_8;
    init_pair(SHADE_FADE,   fg[0],       COLOR_BLACK);
    init_pair(SHADE_DARK,   fg[1],       COLOR_BLACK);
    init_pair(SHADE_MID,    fg[2],       COLOR_BLACK);
    init_pair(SHADE_BRIGHT, fg[3],       COLOR_BLACK);
    init_pair(SHADE_HOT,    fg[4],       COLOR_BLACK);
    init_pair(SHADE_HEAD,   COLOR_WHITE, COLOR_BLACK);
    init_pair(SHADE_CORE,   COLOR_WHITE, COLOR_BLACK);
}

/* Sets up the HUD colours once at startup. The -1 background means the
 * HUD sits on the terminal's real background instead of a black box. */
static void hud_pairs_init(void)
{
    init_pair(PAIR_HUD,  g_has_256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, g_has_256 ?  51 : COLOR_CYAN,   -1);
}

/* Picks the colour + boldness for a tail cell from how far back it is.
 * The head is white and bright; cells get dimmer toward the tip. */
static attr_t dist_attr(int i)
{
    if (i == 0)              return COLOR_PAIR(SHADE_HEAD)   | A_BOLD;
    if (i == 1)              return COLOR_PAIR(SHADE_HOT)    | A_BOLD;
    if (i <= TRAIL_HOT_END)  return COLOR_PAIR(SHADE_BRIGHT) | A_BOLD;
    if (i <= TRAIL_WARM_END) return COLOR_PAIR(SHADE_MID);
    if (i <= RAY_TRAIL - 2)  return COLOR_PAIR(SHADE_DARK);
    return COLOR_PAIR(SHADE_FADE) | A_DIM;
}

/* ── §4 scene ── */

/* The pool of glyphs a stream can show. */
static const char k_glyphs[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789!@#$%^&*()-_=+[]{}|;:,.<>?/~`";

#define GLYPHS_LEN (int)(sizeof k_glyphs - 1)

static char  rand_glyph(void) { return k_glyphs[rand() % GLYPHS_LEN]; }
static float urand01   (void) { return (float)rand() / (float)RAND_MAX; }

/*
 * Ray — one stream of glyphs flying straight out from the core.
 *
 * Think of it as a single dot moving along a fixed line from the
 * centre, trailing a few glyphs behind it. The direction is decided
 * once when the ray is born and never changes; only how far out the
 * head has travelled changes over time. When the whole tail has flown
 * off the edge, the ray is reset and starts over (with a new speed and
 * a new random delay), so the fan keeps flowing without any visible
 * loop. Each ray gets its own speed so they don't all pulse in unison.
 *
 * Members
 *   cos_a, sin_a   Which way this ray points, set once at birth. sin_a
 *                  already has the cell-squish (ASPECT) folded in, so
 *                  the draw loop is just multiply-and-add — no sin/cos
 *                  per cell, which matters at 180 rays × 60 fps.
 *   r_off          How far the head is from the centre, in cells (a
 *                  float, for smooth sub-cell motion). Negative means
 *                  the ray hasn't emerged yet — it's waiting below the
 *                  core. Grows by speed × dt each frame.
 *   speed          Cells per second; fixed for one life, re-rolled on
 *                  reset to keep the fan looking organic.
 *   cache[]        The glyphs this ray currently shows; cache[0] is the
 *                  bright head, the rest trail behind it. About 75% of
 *                  them reroll each frame to give the Matrix shimmer.
 *
 * The whole array of rays lives in fixed storage — no malloc anywhere.
 *
 * Why constant speed: real solar wind speeds up as it climbs, but at
 * this resolution you'd never see it, and constant speed keeps the
 * per-frame update a single line.
 */
typedef struct {
    float cos_a, sin_a;
    float r_off;
    float speed;
    char  cache[RAY_TRAIL];
} Ray;

/* Sends an existing ray back to the start with a fresh delay, speed, and
 * glyphs. Its direction is kept — only its motion is re-randomised. */
static void ray_reset(Ray *r, float max_r)
{
    r->r_off = -urand01() * STAGGER_FRAC * max_r;
    r->speed = SPEED_MIN_CPS + urand01() * (SPEED_MAX_CPS - SPEED_MIN_CPS);
    for (int i = 0; i < RAY_TRAIL; i++)
        r->cache[i] = rand_glyph();
}

/* Aims a fresh ray at the given angle (its direction is then fixed for
 * life), then resets it for a starting position, speed, and glyphs. */
static void ray_init(Ray *r, float angle, float max_r)
{
    r->cos_a = cosf(angle);
    r->sin_a = sinf(angle) * ASPECT;       /* squish the vertical here, once */
    ray_reset(r, max_r);
}

/* Rolls fresh glyphs into roughly 75% of the tail; the rest stay put.
 * That mix of changing and steady cells is what makes it shimmer. */
static inline void shimmer_reroll_ray_cache(Ray *r) {
    for (int i = 0; i < RAY_TRAIL; i++)
        if (rand() % SHIMMER_KEEP_ONE_IN != 0)
            r->cache[i] = rand_glyph();
}

/* True while any part of the ray is still on screen. Once even the tip
 * of the tail has flown past the far edge, it's time to recycle. */
static inline bool tail_tip_inside_recycle_horizon(const Ray *r, float max_r) {
    float tail_tip_distance = r->r_off - (float)RAY_TRAIL;
    return tail_tip_distance < max_r;
}

/* Moves a ray one frame: head flies a bit farther out, glyphs shimmer.
 * Returns false once it has fully left the screen (caller resets it). */
static bool ray_tick(Ray *r, float dt, float max_r)
{
    r->r_off += r->speed * dt;
    shimmer_reroll_ray_cache(r);
    return tail_tip_inside_recycle_horizon(r, max_r);
}

/* How far out tail cell `slot_i` sits: the head's distance, minus one
 * cell per step back toward the centre. */
static inline float trail_cell_radius_from_head(const Ray *r, int slot_i) {
    return r->r_off - (float)slot_i;
}

/* Turns "this far out, in this direction" into a screen cell. No sin/cos
 * here — the direction was baked at birth, so it's just multiply, add,
 * round. We round per point rather than walking a line because the
 * glyphs along a ray aren't a simple straight run of cells. */
static inline void polar_to_screen_cell(int cx, int cy,
                                         float radius,
                                         float cos_a, float sin_a,
                                         int *out_col, int *out_row) {
    *out_col = cx + (int)roundf(radius * cos_a);
    *out_row = cy + (int)roundf(radius * sin_a);
}

/* Draws one tail cell, skipping it quietly if it lands off-screen. */
static inline void paint_ray_trail_cell(const Ray *r, int slot_i,
                                         int col, int row,
                                         int cols, int rows) {
    if (col < 0 || col >= cols || row < 0 || row >= rows) return;
    attr_t attr = dist_attr(slot_i);
    attron(attr);
    mvaddch(row, col, (chtype)(unsigned char)r->cache[slot_i]);
    attroff(attr);
}

/* Draws one ray: the bright head and the glyphs trailing behind it,
 * stopping before it reaches the centre so the '@' core stays clean. */
static void ray_draw(const Ray *r, int cx, int cy, int cols, int rows)
{
    for (int slot_i = 0; slot_i < RAY_TRAIL; slot_i++) {
        float radius = trail_cell_radius_from_head(r, slot_i);
        if (radius < CORE_RESERVED_RADIUS) break;   /* leave the '@' core alone */

        int col, row;
        polar_to_screen_cell(cx, cy, radius, r->cos_a, r->sin_a, &col, &row);
        paint_ray_trail_cell(r, slot_i, col, row, cols, rows);
    }
}

/*
 * RayField — the fixed array holding all N_RAYS streams.
 *
 * Members
 *   rays[N_RAYS]   One slot per stream, all filled at startup. Ray i
 *                  points at angle (i / N_RAYS) of a full turn, so the
 *                  streams fan out evenly all the way around.
 */
typedef struct {
    Ray  rays[N_RAYS];
} RayField;

/*
 * SunGeometry — where the sun sits and how far a ray must fly to retire.
 *
 * Worked out from the screen size at startup (and again on resize).
 *
 * Members
 *   cx, cy   The sun's centre, in screen cells (middle of the screen).
 *   max_r    Once a ray's tail tip gets this far out, it has fully left
 *            the screen and gets recycled. It's set generously past the
 *            real screen corner — overshooting just costs a few extra
 *            harmless ticks and guarantees every ray clears the edge.
 */
typedef struct {
    int   cx, cy;
    float max_r;
} SunGeometry;

/*
 * SimControls — the one playback toggle.
 *
 * Members
 *   paused   When true the rays freeze, but the screen keeps redrawing
 *            so the HUD stays live. Toggled by SPACE or 'p'.
 */
typedef struct {
    bool paused;
} SimControls;

/*
 * Scene — all the live state for one run, in one place:
 *   field      the streams themselves
 *   geom       where the sun is and how far rays fly
 *   sim        the pause toggle
 *   theme_idx  which colour scheme is active
 *
 * Helpers take just the piece they need (a Ray, the geometry) rather
 * than the whole Scene, so it's easy to see what each one touches.
 */
typedef struct {
    RayField    field;
    SunGeometry geom;
    SimControls sim;
    int         theme_idx;
} Scene;

/* Works out the sun's centre and retirement distance from the screen
 * size. Called at startup and on every resize. */
static void compute_sun_geometry(SunGeometry *geom, int cols, int rows) {
    geom->cx    = cols / 2;
    geom->cy    = rows / 2;
    geom->max_r = (float)cols + (float)rows / ASPECT;
}

/* Aims each ray at an evenly-spaced angle around the full circle, so the
 * streams cover every direction with no gaps. */
static void spawn_rays_evenly_spaced(RayField *field, float max_r) {
    const float ANGLE_STEP_PER_RAY = 2.0f * (float)M_PI / (float)N_RAYS;
    for (int i = 0; i < N_RAYS; i++) {
        float angle = (float)i * ANGLE_STEP_PER_RAY;
        ray_init(&field->rays[i], angle, max_r);
    }
}

/* Builds the whole scene from scratch: geometry, all the rays, and the
 * default toggles. Runs at startup and on resize. */
static void scene_init(Scene *s, int cols, int rows)
{
    compute_sun_geometry(&s->geom, cols, rows);
    spawn_rays_evenly_spaced(&s->field, s->geom.max_r);
    s->sim.paused = false;
    s->theme_idx  = 0;
}

/* Restarts every ray (the 'r' key) for a fresh bloom, without touching
 * the geometry or each ray's direction. */
static void scene_reset(Scene *s)
{
    for (int i = 0; i < N_RAYS; i++)
        ray_reset(&s->field.rays[i], s->geom.max_r);
}

/* Moves one ray forward, and if it has flown off the screen, sends it
 * back to the start (same direction, new speed and delay). */
static inline void advance_one_ray_with_recycle(Ray *r, float dt, float max_r) {
    bool still_on_screen = ray_tick(r, dt, max_r);
    if (!still_on_screen)
        ray_reset(r, max_r);
}

/* Advances every ray one frame. While paused it does nothing, but the
 * screen still redraws so the HUD keeps ticking. */
static void scene_tick(Scene *s, float dt)
{
    if (s->sim.paused) return;

    for (int i = 0; i < N_RAYS; i++)
        advance_one_ray_with_recycle(&s->field.rays[i], dt, s->geom.max_r);
}

/* Stamps the '@' at the centre. Drawn after the rays so nothing ever
 * covers it. */
static void sun_draw_core(const Scene *s, int cols, int rows)
{
    int cx = s->geom.cx, cy = s->geom.cy;
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    attron(COLOR_PAIR(SHADE_CORE) | A_BOLD);
    mvaddch(cy, cx, '@');
    attroff(COLOR_PAIR(SHADE_CORE) | A_BOLD);
}

/* Draws all the rays, then the core last so the '@' always shows. */
static void scene_draw(const Scene *s, int cols, int rows)
{
    for (int i = 0; i < N_RAYS; i++)
        ray_draw(&s->field.rays[i], s->geom.cx, s->geom.cy, cols, rows);

    sun_draw_core(s, cols, rows);
}

static void scene_cycle_theme(Scene *s)
{
    s->theme_idx = (s->theme_idx + 1) % THEME_COUNT;
    theme_apply(s->theme_idx);
}

/* ── §5 screen ── */

/*
 * Screen — the terminal's size in cells, plus the ncurses lifecycle.
 *
 * The sun's geometry is derived from this size; after that the sim
 * doesn't care about it, but the drawing code does (to clip glyphs that
 * fall off the edge). Refreshed on every resize.
 *
 * Members
 *   cols   Width in cells.
 *   rows   Height in cells. Row 0 holds the status line; row rows-1
 *          holds the key hints.
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);              /* don't let stdin interrupt frame writes */
    start_color();
    use_default_colors();       /* lets HUD pairs use the terminal's own background */
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* Prints one coloured, bold line — shared by both HUD rows. */
static inline void hud_paint_text(int row, int col, int pair, const char *text) {
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Builds the top-row status text: fps, ray count, theme, paused state. */
static void format_hud_status(const Scene *s, double fps,
                              char *buf, size_t buflen) {
    snprintf(buf, buflen, " %5.1f fps  rays:%d  [%s] %s ",
             fps, N_RAYS, k_themes[s->theme_idx].name,
             s->sim.paused ? "PAUSED " : "running");
}

/* Paints the status text flush against the right edge of row 0. */
static void draw_hud_status(const Screen *sc, const Scene *s, double fps) {
    enum { HUD_TOP_ROW = 0 };
    char buf[HUD_BUF_LEN];
    format_hud_status(s, fps, buf, sizeof buf);
    int right_col = sc->cols - (int)strlen(buf);
    if (right_col < 0) right_col = 0;
    hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, buf);
}

/* Paints the key-list strip along the bottom row. */
static void draw_hud_hint(const Screen *sc) {
    static const char *KEY_HINT =
        " q:quit  spc:pause  r:reset  t:theme ";
    hud_paint_text(sc->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

/* Draws both HUD rows: status up top, key hints along the bottom. */
static void screen_draw_hud(const Screen *sc, double fps, const Scene *s)
{
    draw_hud_status(sc, s, fps);
    draw_hud_hint  (sc);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §6 app ── */

/*
 * FpsCounter — a smoothed frames-per-second readout.
 *
 * Measuring one frame at a time would jump around, so this tallies how
 * many frames happened over a short window and reports the average.
 *
 * Members
 *   frame_count   frames seen so far this window.
 *   window_ns     time elapsed so far this window, in nanoseconds.
 *   display       the last finished average, shown in the HUD.
 */
typedef struct {
    int     frame_count;
    int64_t window_ns;
    double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f) {
    f->frame_count = 0;
    f->window_ns   = 0;
    f->display     = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt_ns) {
    const int64_t FPS_WINDOW_NS = NS_PER_SEC / 2;       /* 500 ms */
    f->frame_count++;
    f->window_ns += dt_ns;
    if (f->window_ns < FPS_WINDOW_NS) return;
    f->display     = (double)f->frame_count
                   * (double)NS_PER_SEC / (double)f->window_ns;
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * App — everything the program keeps alive for one run.
 *
 * Members
 *   scene         the rays and their state.
 *   screen        the terminal size and ncurses handle.
 *   fps           the smoothed fps readout for the HUD.
 *   running       set to 0 to quit (by 'q' or Ctrl-C).
 *   need_resize   set when the window changed; handled next frame.
 *
 * g_app is the only global; the signal handlers need to reach these
 * two flags, and a signal handler can't be handed a pointer. The flags
 * are sig_atomic_t for the same reason — they're poked from a signal. */
typedef struct {
    Scene                 scene;
    Screen                screen;
    FpsCounter            fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Rebuilds the scene at the new size but keeps the user's theme and
 * pause state, which a plain scene_init would have reset. */
static void scene_reinit_preserving_knobs(Scene *s, int new_cols, int new_rows) {
    int  saved_theme  = s->theme_idx;
    bool saved_paused = s->sim.paused;

    scene_init(s, new_cols, new_rows);

    s->theme_idx  = saved_theme;
    s->sim.paused = saved_paused;
}

/* Handles a window resize: re-read the size, rebuild the scene to fit. */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_reinit_preserving_knobs(&app->scene,
                                   app->screen.cols,
                                   app->screen.rows);
    app->need_resize = 0;
}

/* Acts on one keypress. Returns false only when the user wants to quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ': case 'p': case 'P': s->sim.paused = !s->sim.paused;   break;
    case 'r': case 'R':           scene_reset(s);                   break;
    case 't': case 'T':           scene_cycle_theme(s);             break;

    default: break;
    }
    return true;
}

/* The main loop: each pass measures real elapsed time, reads keys,
 * advances the rays by that much, draws, then sleeps to hold ~60 fps. */
int main(void)
{
    srand((unsigned int)clock_ns());
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App   *app   = &g_app;
    Scene *scene = &app->scene;
    app->running = 1;
    fps_counter_init(&app->fps);

    screen_init(&app->screen);
    g_has_256 = (COLORS >= 256);
    scene_init(scene, app->screen.cols, app->screen.rows);
    theme_apply(scene->theme_idx);
    hud_pairs_init();

    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;
    int64_t last_ns = clock_ns();

    while (app->running) {

        /* Handle a resize first so the rest of the frame uses the new size. */
        if (app->need_resize) {
            app_do_resize(app);
            last_ns = clock_ns();
        }

        /* How long the last frame really took (clamped so a long stall
         * doesn't make the rays jump). */
        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - last_ns;
        last_ns        = now_ns;
        float   dt     = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        for (int ch; (ch = getch()) != ERR; ) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }

        scene_tick(scene, dt);
        fps_counter_tick(&app->fps, dt_ns);

        erase();
        scene_draw(scene, app->screen.cols, app->screen.rows);
        screen_draw_hud(&app->screen, app->fps.display, scene);
        screen_present();

        /* Sleep out the rest of this frame's time budget so we don't spin. */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
