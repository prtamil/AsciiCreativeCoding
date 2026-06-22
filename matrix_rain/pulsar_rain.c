/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * pulsar_rain.c — a spinning pulsar in your terminal. Beams sweep
 * around a central '@' like a lighthouse, each trailing a fading wake
 * of random characters that flicker every frame (the Matrix-rain look,
 * bent onto a rotating beam instead of a falling stream).
 *
 * Sister files using the same flickering-character cache:
 *   matrix_rain/matrix_rain.c      — the plain falling-stream original
 *   matrix_rain/fireworks_rain.c   — the same flicker on arc trails
 *   matrix_rain/matrix_snowflake.c — rain piling up into snow
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

/* §1  config — every knob in one place */

enum {
    TARGET_FPS = 60,
};

enum {
    /* How many dots we drop from the core out to the rim along each
     * beam. 80 keeps the beam solid out to the corner; fewer leaves
     * gaps at the edge. */
    N_RADII   = 80,

    /* How many fading cells trail behind each beam's bright tip. The
     * beam has WAKE_LEN+1 cells total (the head plus this many tail
     * cells). */
    WAKE_LEN  = 16,
};

/* How far apart, in angle, the wake cells sit. Pick it so the tail
 * stays solid rather than gappy: at radius r, one gap is about
 * r · WAKE_STEP cells wide, so 0.05 means roughly one cell at r=20.
 * Bigger = chunkier tail, smaller = thinner. */
#define WAKE_STEP   0.05f

/* Terminal cells are about twice as tall as they are wide, so a circle
 * drawn naively comes out stretched. We squash the up/down part of
 * every beam by this factor to make beams look round. */
#define ASPECT   0.45f

/* Push the beams a touch past the screen corner so there's no visible
 * gap right at the edge. */
#define MAX_R_OVERSHOOT  1.05f

/* Spin speed, in whole turns per second (easy to time with a watch). */
#define SPIN_DEFAULT_RPS  0.50f
#define SPIN_MIN_RPS      0.00f
#define SPIN_MAX_RPS      4.00f
#define SPIN_STEP_RPS     0.10f

enum {
    BEAMS_MIN     =  1,
    BEAMS_DEFAULT =  2,
    BEAMS_MAX     = 16,
};

/* The flicker. Each frame every cell has a 1-in-this chance of keeping
 * its character; the rest get a fresh random one. 4 means about 75% of
 * cells change each frame — the classic Matrix shimmer. Bigger = more
 * frozen, smaller = noisier. */
#define SHIMMER_KEEP_ONE_IN  4

/* ncurses colour-pair slots. 1..5 are the wake's brightness bands and
 * change with the theme; the head, the core, and the two HUD colours
 * stay fixed. */
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

/* Longest frame we'll believe. After a stall (e.g. dragging the
 * window) dt would be huge; clamping it stops the beam from jumping. */
#define DT_CAP_SEC  0.10f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

#define HUD_BUF_LEN  96

/* §2  clock — monotonic timer + sleep */

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

/* §3  color — HUD pairs plus five themed brightness ramps */

/*
 * Theme — one named colour scheme for a beam's fading tail.
 *
 * A beam fades from a bright tip back through five brightness steps to
 * an almost-gone tail. This struct just stores the five foreground
 * colours for one look; §4's wake_attr decides which step each tail
 * cell gets. The 't' key swaps themes by re-colouring these five slots
 * in place, so beams already on screen change instantly.
 *
 * The head and the central '@' aren't in here — they stay white in
 * every theme so they don't blur into the coloured tail.
 *
 * Members
 *   name   the label shown in the HUD ("green", "amber", …).
 *   fg[]   five colours for a 256-colour terminal, ordered dimmest
 *          (oldest tail) to brightest. Per the project's palette rule,
 *          all stay in the bright half of the colour space so even the
 *          dimmest tail cell is still visible on a black background.
 *   fg_8[] the same five for an 8-colour terminal, which can't manage
 *          a fine gradient — they collapse onto one ANSI colour.
 */
typedef struct {
    const char *name;
    int         fg  [5];
    int         fg_8[5];
} Theme;

static const Theme k_themes[] = {
    { "green",
      {  28,  34,  40,  46,  82 },
      { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN   } },
    { "amber",
      {  94, 130, 172, 214, 220 },
      { COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW  } },
    { "blue",
      {  24,  33,  39,  45,  51 },
      { COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN    } },
    { "plasma",
      {  53,  57,  93, 129, 201 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA } },
    { "fire",
      {  52,  88, 124, 160, 196 },
      { COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_RED     } },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static bool g_has_256 = false;

/* Recolour the wake bands (and re-set head/core to white) for one
 * theme. Leaves the HUD colours alone so they never shift with theme. */
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

/* Set the two HUD colours once at startup. The -1 background means
 * "whatever the terminal's own background is" so the HUD blends in
 * instead of sitting on a black box. */
static void hud_pairs_init(void)
{
    init_pair(PAIR_HUD,  g_has_256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, g_has_256 ?  51 : COLOR_CYAN,   -1);
}

/* Pick the colour and weight for a tail cell from how far it sits
 * behind the head (k=0). Closer cells are brighter and bold; the
 * farthest fade out dim. */
static attr_t wake_attr(int k)
{
    if (k == 0)              return COLOR_PAIR(SHADE_HEAD)   | A_BOLD;
    if (k == 1)              return COLOR_PAIR(SHADE_HOT)    | A_BOLD;
    if (k == 2)              return COLOR_PAIR(SHADE_BRIGHT) | A_BOLD;
    if (k <= WAKE_LEN / 2)   return COLOR_PAIR(SHADE_MID);
    if (k <= WAKE_LEN - 2)   return COLOR_PAIR(SHADE_DARK);
    return COLOR_PAIR(SHADE_FADE) | A_DIM;
}

/* §4  scene — all the spinning + drawing state and logic */

/* The character pool the flicker draws from. */
static const char k_glyphs[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789!@#$%^&*()-_=+[]{}|;:,.<>?/~`";

#define GLYPHS_LEN (int)(sizeof k_glyphs - 1)

static char rand_glyph(void) { return k_glyphs[rand() % GLYPHS_LEN]; }

/*
 * Rotation — where the beams are pointing and how fast they turn.
 *
 * Think of one hand on a clock: `angle` is where beam zero points
 * right now, and the rest sit evenly around the circle from it.
 *
 * Members
 *   angle     beam-zero's direction in radians, kept in [0, 2π).
 *   spin_rps  turn rate in whole turns per second (not radians — turns
 *             are easy to time by eye). Set with +/-, kept in range.
 *   n_beams   how many beams, spread evenly around the circle. Set
 *             with [ and ], kept in range.
 */
typedef struct {
    float angle;
    float spin_rps;
    int   n_beams;
} Rotation;

/*
 * BeamGeometry — where the centre is and how far out the beams reach.
 * Worked out once from the window size (and again on resize), then
 * left alone until the window changes.
 *
 * The radius lives in "square" units that ignore the cell shape; the
 * tall/wide squash from ASPECT gets applied later, when we actually
 * place a cell, so the reach is symmetric in every direction.
 *
 * Members
 *   cx, cy   the centre cell, where the '@' sits (middle of the screen).
 *   max_r    how far the farthest dot reaches — out to the corner plus
 *            a little, so the beam doesn't stop short of the edge.
 *   r_step   spacing between dots along a beam (max_r / N_RADII).
 */
typedef struct {
    int   cx, cy;
    float max_r;
    float r_step;
} BeamGeometry;

/*
 * GlyphCache — the grid of flickering characters, one per (how-far-out,
 * how-far-back-in-the-tail) spot.
 *
 * One shared grid serves every beam: the eye can't tell which beam owns
 * which character, only the overall sparkle, so a second copy would buy
 * nothing. Each frame most of these characters get rerolled — that
 * reroll is the shimmer; the sense of motion is the beam sweeping over
 * them.
 *
 * Members
 *   glyphs[ri][k]  the character at radius slot ri and tail slot k.
 *                  ri runs 0..N_RADII-1; k runs 0..WAKE_LEN (the head
 *                  plus its trailing cells).
 */
typedef struct {
    char glyphs[N_RADII][WAKE_LEN + 1];
} GlyphCache;

/*
 * SimControls — the playback switches. Just one for now.
 *
 * Members
 *   paused  when true the scene stops advancing: no turning, no
 *           flicker. Toggled by space or p.
 */
typedef struct {
    bool paused;
} SimControls;

/*
 * Scene — everything one run needs, in one place.
 *
 *     Scene
 *       rot        where the beams point + spin speed + how many
 *       geom       centre, reach, dot spacing
 *       cache      the flickering characters
 *       sim        paused?
 *       theme_idx  which colour scheme
 *
 * Helpers that only need one part take just that part, so a function's
 * signature tells you what it can touch.
 */
typedef struct {
    Rotation     rot;
    BeamGeometry geom;
    GlyphCache   cache;
    SimControls  sim;
    int          theme_idx;
} Scene;

/* Give every cell a fresh random character — used at start and reset so
 * the first frame isn't blank. */
static void cache_seed_random(GlyphCache *cache) {
    for (int ri = 0; ri < N_RADII; ri++)
        for (int k = 0; k <= WAKE_LEN; k++)
            cache->glyphs[ri][k] = rand_glyph();
}

/* The shimmer itself: each frame reroll most cells to a new random
 * character, leaving roughly 1 in SHIMMER_KEEP_ONE_IN untouched. */
static void cache_shimmer_reroll(GlyphCache *cache) {
    for (int ri = 0; ri < N_RADII; ri++)
        for (int k = 0; k <= WAKE_LEN; k++)
            if (rand() % SHIMMER_KEEP_ONE_IN != 0)
                cache->glyphs[ri][k] = rand_glyph();
}

/* Put the centre in the middle and set the reach to the distance from
 * there to a corner (plus a little). The height is un-squashed first so
 * the reach is measured in the same "square" units the radius uses. */
static void compute_beam_geometry(BeamGeometry *geom, int cols, int rows) {
    geom->cx = cols / 2;
    geom->cy = rows / 2;

    float screen_half_width_iso  = (float)cols * 0.5f;
    float screen_half_height_iso = (float)rows * 0.5f / ASPECT;
    float corner_distance_iso = sqrtf(screen_half_width_iso  *
                                       screen_half_width_iso
                                     + screen_half_height_iso *
                                       screen_half_height_iso);
    geom->max_r  = corner_distance_iso * MAX_R_OVERSHOOT;
    geom->r_step = geom->max_r / (float)N_RADII;
}

/* Build a fresh scene from scratch — defaults for everything. Used at
 * startup and on resize (the resize path puts the user's knobs back
 * afterwards). */
static void scene_init(Scene *s, int cols, int rows) {
    s->rot.angle    = 0.0f;
    s->rot.spin_rps = SPIN_DEFAULT_RPS;
    s->rot.n_beams  = BEAMS_DEFAULT;

    compute_beam_geometry(&s->geom, cols, rows);

    cache_seed_random(&s->cache);

    s->sim.paused = false;
    s->theme_idx  = 0;
}

/* The 'r' key. Re-centres the spin and beam count and re-scrambles the
 * characters, but keeps the spin speed and theme the user chose. */
static void scene_reset(Scene *s) {
    s->rot.angle   = 0.0f;
    s->rot.n_beams = BEAMS_DEFAULT;
    cache_seed_random(&s->cache);
}

/* Fold an angle back into one turn's worth, [0, 2π). The loop is fine
 * here because dt is capped, so it never runs more than a step or two. */
static inline void wrap_angle_to_2pi(float *angle) {
    while (*angle >= 2.0f * (float)M_PI) *angle -= 2.0f * (float)M_PI;
    while (*angle <  0.0f)               *angle += 2.0f * (float)M_PI;
}

/* Turns-per-second to radians-per-second — just so the tick reads in
 * words instead of a bare 2π multiply. */
static inline float rotations_per_sec_to_rad_per_sec(float rps) {
    return rps * 2.0f * (float)M_PI;
}

/* One step of the world: nudge the angle forward by the spin rate,
 * wrap it, and reroll the flicker. Paused stops all of it. */
static void scene_tick(Scene *s, float dt) {
    if (s->sim.paused) return;

    float omega_rad_per_sec = rotations_per_sec_to_rad_per_sec(s->rot.spin_rps);
    s->rot.angle += omega_rad_per_sec * dt;
    wrap_angle_to_2pi(&s->rot.angle);

    cache_shimmer_reroll(&s->cache);
}

/*
 * One beam fans out into WAKE_LEN+1 slightly-different directions (the
 * head and its trailing cells). Work out the direction of each once,
 * here, so the dot-laying loop can reuse them instead of calling sin/cos
 * for every dot. The up/down squash (ASPECT) is folded into the second
 * number so the rest of the code never has to think about it.
 */
static inline void precompute_wake_direction_table(float base_angle,
                                                    float cw[WAKE_LEN + 1],
                                                    float sw[WAKE_LEN + 1]) {
    for (int k = 0; k <= WAKE_LEN; k++) {
        float wake_angle = base_angle - (float)k * WAKE_STEP;
        cw[k] = cosf(wake_angle);
        sw[k] = sinf(wake_angle) * ASPECT;
    }
}

/* Turn a distance-and-direction into an actual screen cell by rounding
 * to the nearest one. */
static inline void polar_to_screen_cell(const BeamGeometry *geom,
                                         float r, float cw, float sw,
                                         int *out_col, int *out_row) {
    *out_col = geom->cx + (int)roundf(r * cw);
    *out_row = geom->cy + (int)roundf(r * sw);
}

/* Draw one tail character, skipping it if it lands off-screen. */
static inline void paint_wake_slot(const Scene *s, int ri, int k,
                                    int col, int row, int cols, int rows) {
    if (col < 0 || col >= cols || row < 0 || row >= rows) return;
    attr_t attr = wake_attr(k);
    attron(attr);
    mvaddch(row, col, (chtype)(unsigned char)s->cache.glyphs[ri][k]);
    attroff(attr);
}

/*
 * Lay down all of one beam's cells at a single distance out, dimmest
 * first so the bright head is painted last. Near the centre several of
 * them round to the same cell, and painting the head last makes it win
 * that spot.
 */
static inline void paint_one_ring_dim_first(const Scene *s, int ri,
                                             const float cw[WAKE_LEN + 1],
                                             const float sw[WAKE_LEN + 1],
                                             int cols, int rows) {
    float ring_radius = (float)(ri + 1) * s->geom.r_step;
    for (int k = WAKE_LEN; k >= 0; k--) {
        int col, row;
        polar_to_screen_cell(&s->geom, ring_radius, cw[k], sw[k], &col, &row);
        paint_wake_slot(s, ri, k, col, row, cols, rows);
    }
}

/* Draw one whole beam: figure out its spread of directions once, then
 * lay dots from the centre out to the rim. */
static void pulsar_draw_beam(const Scene *s, float base_angle,
                             int cols, int rows)
{
    float cw[WAKE_LEN + 1], sw[WAKE_LEN + 1];
    precompute_wake_direction_table(base_angle, cw, sw);

    for (int ri = 0; ri < N_RADII; ri++)
        paint_one_ring_dim_first(s, ri, cw, sw, cols, rows);
}

/* The bright '@' at the centre. Drawn after the beams so it always sits
 * on top of them. */
static void pulsar_draw_core(const Scene *s, int cols, int rows)
{
    int cx = s->geom.cx, cy = s->geom.cy;
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    attron(COLOR_PAIR(SHADE_CORE) | A_BOLD);
    mvaddch(cy, cx, '@');
    attroff(COLOR_PAIR(SHADE_CORE) | A_BOLD);
}

/* Draw the whole frame: every beam spread evenly around the circle,
 * then the core on top. */
static void scene_draw(const Scene *s, int cols, int rows)
{
    float beam_angle_step = 2.0f * (float)M_PI / (float)s->rot.n_beams;
    for (int b = 0; b < s->rot.n_beams; b++)
        pulsar_draw_beam(s, s->rot.angle + (float)b * beam_angle_step,
                         cols, rows);

    pulsar_draw_core(s, cols, rows);
}

/* §4 key handlers — adjust spin, beam count, theme */

static void scene_change_spin(Scene *s, float delta_rps)
{
    s->rot.spin_rps += delta_rps;
    if (s->rot.spin_rps < SPIN_MIN_RPS) s->rot.spin_rps = SPIN_MIN_RPS;
    if (s->rot.spin_rps > SPIN_MAX_RPS) s->rot.spin_rps = SPIN_MAX_RPS;
}

static void scene_change_beams(Scene *s, int delta)
{
    s->rot.n_beams += delta;
    if (s->rot.n_beams < BEAMS_MIN) s->rot.n_beams = BEAMS_MIN;
    if (s->rot.n_beams > BEAMS_MAX) s->rot.n_beams = BEAMS_MAX;
}

static void scene_cycle_theme(Scene *s)
{
    s->theme_idx = (s->theme_idx + 1) % THEME_COUNT;
    theme_apply(s->theme_idx);
}

/* Rebuild the scene for a new window size but keep the user's settings.
 * Used on resize: re-init wipes everything, so we save the knobs the
 * user picked and put them back. Keeping that list in one spot means a
 * new knob added here survives a resize for free. */
static void scene_reinit_preserving_knobs(Scene *s, int new_cols, int new_rows) {
    float saved_spin   = s->rot.spin_rps;
    int   saved_beams  = s->rot.n_beams;
    int   saved_theme  = s->theme_idx;
    bool  saved_paused = s->sim.paused;

    scene_init(s, new_cols, new_rows);

    s->rot.spin_rps = saved_spin;
    s->rot.n_beams  = saved_beams;
    s->theme_idx    = saved_theme;
    s->sim.paused   = saved_paused;
}

/* §5  screen — ncurses setup, frame flush, and the HUD */

/*
 * Screen — the terminal's current size in cells.
 *
 * Everything geometric (centre, reach, dot spacing) is worked out from
 * these two numbers in §4; after that the drawing code only needs them
 * to skip anything that lands off-screen. Refreshed at startup and on
 * every resize.
 *
 * Members
 *   cols   width in cells.
 *   rows   height in cells. Row 0 holds the status line; the bottom row
 *          (rows-1) holds the key hint.
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
    typeahead(-1);              /* stop keypresses from cutting into our drawing */
    start_color();
    use_default_colors();       /* allow -1 = "the terminal's own background" */
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* Print one bold coloured string — shared by both HUD rows. */
static inline void hud_paint_text(int row, int col, int pair, const char *text) {
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Build the top-row status text: fps, spin, beam count, theme, paused. */
static void format_hud_status(const Scene *s, double fps,
                              char *buf, size_t buflen) {
    const char *beam_suffix = (s->rot.n_beams == 1) ? " " : "s";
    snprintf(buf, buflen,
             " %5.1f fps  %5.2f rps  %d beam%s  [%s] %s ",
             fps, (double)s->rot.spin_rps, s->rot.n_beams,
             beam_suffix,
             k_themes[s->theme_idx].name,
             s->sim.paused ? "PAUSED " : "running");
}

/* The yellow status line, pinned to the right of row 0. */
static void draw_hud_status(const Screen *sc, const Scene *s, double fps) {
    enum { HUD_TOP_ROW = 0 };
    char buf[HUD_BUF_LEN];
    format_hud_status(s, fps, buf, sizeof buf);
    int right_col = sc->cols - (int)strlen(buf);
    if (right_col < 0) right_col = 0;
    hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, buf);
}

/* The cyan key-list strip along the bottom row. */
static void draw_hud_hint(const Screen *sc) {
    static const char *KEY_HINT =
        " q:quit  spc:pause  r:reset  +/-:spin  []:beams  t:theme ";
    hud_paint_text(sc->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

/* The whole HUD: status line up top, key hints along the bottom. */
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

/* §6  app — signals, resize, and the main loop */

/*
 * FpsCounter — a smoothed frame-rate readout for the HUD.
 *
 * A raw per-frame number jumps around too much to read, so this totals
 * up frames and time over a short window and publishes a steady figure
 * each time the window fills.
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
 * App — the whole program's state in one bag.
 *
 *   scene        the pulsar (spin, geometry, characters, theme)
 *   screen       the terminal size
 *   fps          the smoothed frame-rate readout
 *   running      cleared on quit or Ctrl-C; the main loop stops
 *   need_resize  set when the window changes; the loop reacts next time
 *
 * There's one global, g_app, only because the signal handlers need to
 * reach these flags; everything else is passed by pointer.
 */
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

/* Re-read the new window size and rebuild the scene to fit it. */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_reinit_preserving_knobs(&app->scene,
                                   app->screen.cols,
                                   app->screen.rows);
    app->need_resize = 0;
}

/* Handle one keypress. Returns false only when the user wants to quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ': case 'p': case 'P': s->sim.paused = !s->sim.paused;  break;

    case 'r': case 'R':           scene_reset(s);                 break;

    case '=': case '+':           scene_change_spin (s, +SPIN_STEP_RPS); break;
    case '-':                     scene_change_spin (s, -SPIN_STEP_RPS); break;

    case ']':                     scene_change_beams(s, +1);       break;
    case '[':                     scene_change_beams(s, -1);       break;

    case 't': case 'T':           scene_cycle_theme (s);           break;

    default: break;
    }
    return true;
}

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

        /* handle a resize first so the rest of the frame uses the new size */
        if (app->need_resize) {
            app_do_resize(app);
            last_ns = clock_ns();
        }

        /* how long since last frame (clamped so a stall can't jump the beam) */
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

        /* sleep out the rest of the frame so we hold the target rate */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
