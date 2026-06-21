/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */

/*
 * voronoi.c — animated Voronoi diagram in the terminal.
 *
 * Colour every screen cell by whichever drifting seed point is nearest, so
 * the screen splits into coloured territories that reshape as the seeds move.
 * Seeds wander with damped random motion (Langevin / Ornstein-Uhlenbeck) and
 * bounce off the edges. Keys: q/ESC quit, space pause, r reset, [ ] sim Hz.
 *
 * Sister files: procedural/generational/voronoi_region_map.c (static version),
 * delaunay_triangulation.c (the dual graph).
 * Build: gcc -std=c11 -O2 -Wall -Wextra voronoi.c -o voronoi -lncurses -lm
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ── §1 config — tunable constants, all magic numbers named here ── */

enum {
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 60,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    = 10,
    FPS_UPDATE_MS   = 500,
    N_COLORS        = 7,
    N_SEEDS         = 24,
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Each terminal character spans this many "pixels" of sub-cell space, so
 * seeds can move smoothly between characters instead of snapping. */
#define CELL_W  8
#define CELL_H  16

/* Seed drift. Each frame a seed's speed is pulled toward zero (DAMP) and
 * kicked by a random nudge (NOISE); the two balance at a steady wander
 * speed of roughly NOISE/DAMP. */
#define DAMP     2.0f    /* how strongly speed decays toward zero            */
#define NOISE   60.0f    /* strength of the random kick each frame           */

/* A cell is drawn as a border '+' when its two nearest seeds are nearly
 * tied (within BORDER_PX), and as a seed centre 'O' when it sits within
 * SEED_PX of the nearest seed. Both measured in pixels. */
#define BORDER_PX  15.0f
#define SEED_PX    12.0f

/* Spawn/bounce margins, given as a count of cells so they scale with
 * terminal size: seeds spawn inside the screen by SEED_SPAWN_MARGIN_*,
 * and bounce back when they drift within BOUNCE_MARGIN_* of an edge.
 * SEED_INIT_VEL_SCALE only sets the first frame's speed; the drift rule
 * takes over within a few frames. */
#define SEED_SPAWN_MARGIN_COLS   3
#define SEED_SPAWN_MARGIN_ROWS   2
#define BOUNCE_MARGIN_COLS       2
#define BOUNCE_MARGIN_ROWS       2
#define SEED_INIT_VEL_SCALE     20.0f

/* Voronoi-draw sentinel: any real squared distance is far below this. */
#define DIST_SQ_INFINITY  1e18f

/* Measure distance from the middle of each cell (half a cell in), not its
 * corner, so the colour matches what the eye sees at the glyph's centre. */
#define CELL_CENTER_OFFSET_FRAC  0.5f

/* HUD rows reserved at top (status) and bottom (action hints). */
#define HUD_TOP_ROWS     1
#define HUD_BOTTOM_ROWS  1

/* Render-loop frame budget: cap the loop at 60 Hz regardless of sim fps. */
#define RENDER_TARGET_FPS  60
#define RENDER_FRAME_NS    (NS_PER_SEC / RENDER_TARGET_FPS)

/* FPS-meter sampling window: how often the HUD's displayed fps refreshes. */
#define FPS_UPDATE_PERIOD_NS  ((int64_t)FPS_UPDATE_MS * NS_PER_MS)

/* Spiral-of-death guard: if a single frame measures longer than this,
 * clamp dt so the fixed-timestep accumulator can never run away. */
#define MAX_FRAME_DT_NS  (100 * NS_PER_MS)

/* ESC key code — POSIX terminals send 27 for both ESC and the start of
 * many escape sequences; treated as quit here. */
#define KEY_ESC   27

/* ── §2 clock — monotonic nanosecond timer and a sleep helper ── */

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
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ── §3 color — seed palette plus the two reserved HUD colour pairs ── */

/*
 * Named colour-pair slots. Pairs 1..N_COLORS are the seed colours (every
 * territory is painted in its seed's colour). CP_HUD and CP_HINT are fixed
 * IDs the whole project reserves for the top/bottom status strips.
 */
enum {
    CP_HUD  = 8,    /* top status row:  bright yellow + bold */
    CP_HINT = 9,    /* bottom hint row: bright cyan + bold   */
};

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(1,       196, COLOR_BLACK);     /* seed palette */
        init_pair(2,       208, COLOR_BLACK);
        init_pair(3,       226, COLOR_BLACK);
        init_pair(4,        46, COLOR_BLACK);
        init_pair(5,        51, COLOR_BLACK);
        init_pair(6,        75, COLOR_BLACK);
        init_pair(7,       201, COLOR_BLACK);
        init_pair(CP_HUD,  226, -1);              /* bright yellow */
        init_pair(CP_HINT,  51, -1);              /* bright cyan   */
    } else {
        init_pair(1,       COLOR_RED,     COLOR_BLACK);
        init_pair(2,       COLOR_RED,     COLOR_BLACK);
        init_pair(3,       COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4,       COLOR_GREEN,   COLOR_BLACK);
        init_pair(5,       COLOR_CYAN,    COLOR_BLACK);
        init_pair(6,       COLOR_BLUE,    COLOR_BLACK);
        init_pair(7,       COLOR_MAGENTA, COLOR_BLACK);
        init_pair(CP_HUD,  COLOR_YELLOW,  -1);
        init_pair(CP_HINT, COLOR_CYAN,    -1);
    }
}

/* ── §4 coords — convert a cell count to pixel-space width/height ── */

static inline float pw(int cols) { return (float)cols * CELL_W; }
static inline float ph(int rows) { return (float)rows * CELL_H; }

/* ── §5 entity — the seeds, their motion, and the diagram renderer ── */

/*
 * Seed — one of the moving points that the territories form around.
 *
 * Every screen cell asks "which seed am I closest to?" each frame, and the
 * answers carve the screen into one coloured region per seed. Seeds carry a
 * velocity (not just a position) so they can drift smoothly, which is what
 * makes the diagram reshape continuously. Each seed also remembers its own
 * colour so the inner draw loop doesn't have to look it up.
 *
 * Members:
 *   px, py   position in pixel space (sub-cell precision). Distances are
 *            measured here, not in character cells, so regions keep their
 *            true shape and aren't squashed by the tall 8x16 cell aspect.
 *   vx, vy   speed in pixels per second; updated by the drift step.
 *   pair     this seed's colour slot, 1..N_COLORS. Set once at reset and
 *            shuffled so neighbouring seeds rarely share a colour (a shared
 *            colour would hide the border between them).
 *
 * Invariants: the drift step keeps px,py inside the screen; pair never
 * changes after voronoi_reset.
 */
typedef struct {
    float px, py;   /* position in pixel space    */
    float vx, vy;   /* velocity, pixels/second    */
    int   pair;     /* colour slot, 1..N_COLORS   */
} Seed;

/*
 * Voronoi — just the pool of seeds. The diagram itself is never stored:
 * voronoi_draw rebuilds it from scratch every frame by checking, for each
 * cell, which seed is nearest. So this struct is the input to the drawing,
 * and the painted screen is the output.
 *
 * The array is fixed-size (no allocation after startup). N_SEEDS is kept
 * small on purpose so the per-frame "every cell scans every seed" cost stays
 * cheap; far larger seed counts would need a smarter algorithm.
 *
 * Member:
 *   seeds[N_SEEDS]   the moving points. The array index has no meaning to
 *                    the viewer; only where the seeds sit matters. The drift
 *                    step keeps every seed on-screen, so the nearest-seed
 *                    search always has a valid answer.
 */
typedef struct {
    Seed seeds[N_SEEDS];
} Voronoi;

/* ── §5a random helpers — uniform floats for spawn and drift noise ── */

/* Random float in [0, 1]. */
static float rand_unit(void) { return (float)rand() / (float)RAND_MAX; }

/* Random float in [-1, 1] — the symmetric kick used by the drift step. */
static float rand_signed(void) { return rand_unit() * 2.0f - 1.0f; }

/* Random float in [lo, hi]. */
static float rand_in_range(float lo, float hi)
{
    return lo + rand_unit() * (hi - lo);
}

/* ── §5b seed setup — drop seeds at random spots and give them colours ── */

/* Drop one seed at a random spot inside the screen (staying margin away
 * from the edges) and give it a small starting nudge. */
static void spawn_seed_uniform(
    Seed *s, float W, float H, float margin_x, float margin_y
) {
    s->px = rand_in_range(margin_x, W - margin_x);
    s->py = rand_in_range(margin_y, H - margin_y);
    s->vx = rand_signed() * SEED_INIT_VEL_SCALE;
    s->vy = rand_signed() * SEED_INIT_VEL_SCALE;
}

/* Hand out colours in order, wrapping when there are more seeds than
 * colours. The shuffle below then scrambles them. */
static void assign_round_robin_colors(Voronoi *v)
{
    for (int i = 0; i < N_SEEDS; i++)
        v->seeds[i].pair = (i % N_COLORS) + 1;
}

/* Shuffle only the colours (Fisher-Yates), leaving positions alone, so two
 * seeds that ended up next to each other are unlikely to share a colour and
 * blur their shared border. */
static void shuffle_seed_colors(Voronoi *v)
{
    for (int i = N_SEEDS - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int t = v->seeds[i].pair;
        v->seeds[i].pair = v->seeds[j].pair;
        v->seeds[j].pair = t;
    }
}

/* Re-roll every seed's position, speed, and colour for the current screen
 * size (the 'r' key and a resize both call this). */
static void voronoi_reset(Voronoi *v, int cols, int rows)
{
    float W  = pw(cols);
    float H  = ph(rows);
    float mx = (float)CELL_W * SEED_SPAWN_MARGIN_COLS;
    float my = (float)CELL_H * SEED_SPAWN_MARGIN_ROWS;

    for (int i = 0; i < N_SEEDS; i++)
        spawn_seed_uniform(&v->seeds[i], W, H, mx, my);

    assign_round_robin_colors(v);
    shuffle_seed_colors(v);
}

/* Zero the struct, then roll fresh seeds. */
static void voronoi_init(Voronoi *v, int cols, int rows)
{
    memset(v, 0, sizeof *v);
    voronoi_reset(v, cols, rows);
}

/* ── §5c seed motion — drift the speed, move, and bounce off the edges ── */

/* Nudge a seed's speed: pull it toward zero (DAMP) and add a fresh random
 * kick (NOISE). Over a few frames this settles into a steady wander rather
 * than running off to infinity or grinding to a halt. Each axis gets its
 * own independent kick. */
static void langevin_velocity_step(Seed *s, float dt)
{
    s->vx += (-DAMP * s->vx + NOISE * rand_signed()) * dt;
    s->vy += (-DAMP * s->vy + NOISE * rand_signed()) * dt;
}

/* Move the seed by its current speed for one time step. */
static void integrate_position(Seed *s, float dt)
{
    s->px += s->vx * dt;
    s->py += s->vy * dt;
}

/* Keep a seed inside the screen: if it crosses an edge, snap it back onto
 * the edge and flip that axis so it heads inward. Using fabsf to set the
 * sign means it bounces in correctly even if it was already moving outward
 * when we caught it. */
static void bounce_seed_off_walls(
    Seed *s, float mxL, float mxR, float myT, float myB
) {
    if (s->px < mxL) { s->px = mxL; s->vx =  fabsf(s->vx); }
    if (s->px > mxR) { s->px = mxR; s->vx = -fabsf(s->vx); }
    if (s->py < myT) { s->py = myT; s->vy =  fabsf(s->vy); }
    if (s->py > myB) { s->py = myB; s->vy = -fabsf(s->vy); }
}

/* Advance every seed by one time step: drift speed, move, bounce. */
static void voronoi_tick(Voronoi *v, float dt, int cols, int rows)
{
    float W   = pw(cols);
    float H   = ph(rows);
    float mxL = (float)CELL_W * BOUNCE_MARGIN_COLS;
    float myT = (float)CELL_H * BOUNCE_MARGIN_ROWS;
    float mxR = W - mxL;
    float myB = H - myT;

    for (int i = 0; i < N_SEEDS; i++) {
        Seed *s = &v->seeds[i];
        langevin_velocity_step(s, dt);
        integrate_position   (s, dt);
        bounce_seed_off_walls(s, mxL, mxR, myT, myB);
    }
}

/* ── §5d render — for each cell, find its nearest seed and paint it ── */

/* Give the pixel-space point at the middle of a character cell, so distance
 * tests probe where the eye looks rather than the cell's corner. */
static void cell_center_in_pixel_space(
    int col, int row, float *out_cx, float *out_cy
) {
    *out_cx = ((float)col + CELL_CENTER_OFFSET_FRAC) * (float)CELL_W;
    *out_cy = ((float)row + CELL_CENTER_OFFSET_FRAC) * (float)CELL_H;
}

/* Scan every seed and report the nearest one (its index) plus the distances
 * to the nearest and second-nearest. The second-nearest is what lets the
 * caller spot border cells. Distances are kept squared during the scan to
 * skip a square root per seed; the caller takes a single sqrt afterward. */
static void find_two_nearest_seeds(
    const Voronoi *v, float cx, float cy,
    int *out_best, float *out_d1_sq, float *out_d2_sq
) {
    float d1_sq = DIST_SQ_INFINITY;
    float d2_sq = DIST_SQ_INFINITY;
    int   best  = 0;
    for (int k = 0; k < N_SEEDS; k++) {
        float dx  = cx - v->seeds[k].px;
        float dy  = cy - v->seeds[k].py;
        float dsq = dx * dx + dy * dy;
        if (dsq < d1_sq) {
            d2_sq = d1_sq;
            d1_sq = dsq;
            best  = k;
        } else if (dsq < d2_sq) {
            d2_sq = dsq;
        }
    }
    *out_best  = best;
    *out_d1_sq = d1_sq;
    *out_d2_sq = d2_sq;
}

/* Decide what a cell looks like from its two distances:
 *   - right on top of a seed (d1 < SEED_PX)        -> bold 'O'
 *   - two nearest seeds nearly tied (d2-d1 small)  -> '+' on the border
 *   - otherwise                                    -> dim '.' filling a region
 * The "two seeds nearly tied" test is a cheap stand-in for computing the
 * exact dividing line between regions, and looks fine on screen. */
static void classify_voronoi_cell(
    float d1, float d2, char *out_ch, chtype *out_attr
) {
    if (d1 < SEED_PX) {
        *out_ch   = 'O';
        *out_attr = A_BOLD;
    } else if (d2 - d1 < BORDER_PX) {
        *out_ch   = '+';
        *out_attr = 0;
    } else {
        *out_ch   = '.';
        *out_attr = A_DIM;
    }
}

/* Draw one character at (row, col) in the given colour and style. */
static void paint_voronoi_cell(
    WINDOW *w, int row, int col, int pair, char ch, chtype attr
) {
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, row, col, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/* Paint the whole diagram: walk every cell (skipping the HUD rows), find its
 * nearest seed, decide its glyph, and draw it in that seed's colour. This is
 * the brute-force heart of the demo -- every cell checks every seed. */
static void voronoi_draw(const Voronoi *v, WINDOW *w, int cols, int rows)
{
    for (int row = HUD_TOP_ROWS; row < rows - HUD_BOTTOM_ROWS; row++) {
        for (int col = 0; col < cols; col++) {
            float cx, cy;
            cell_center_in_pixel_space(col, row, &cx, &cy);

            int   best;
            float d1_sq, d2_sq;
            find_two_nearest_seeds(v, cx, cy, &best, &d1_sq, &d2_sq);
            float d1 = sqrtf(d1_sq);
            float d2 = sqrtf(d2_sq);

            char   ch;
            chtype attr;
            classify_voronoi_cell(d1, d2, &ch, &attr);
            paint_voronoi_cell(w, row, col, v->seeds[best].pair, ch, attr);
        }
    }
}

/* ── §6 scene — bundles the seeds, the playback knobs, and screen size ── */

/*
 * SimControls — the two playback knobs the viewer can change while it runs.
 * Kept apart from the simulation data so it's clear these are user toggles,
 * not part of the diagram.
 *
 * Members:
 *   paused   when true the simulation freezes and the HUD shows "PAUSED"
 *            (toggled with SPACE).
 *   fps      how many simulation steps per second; '[' and ']' nudge it,
 *            clamped between SIM_FPS_MIN and SIM_FPS_MAX. Lower fps means
 *            bigger jumps per step, so motion is faster but choppier.
 */
typedef struct {
    bool paused;
    int  fps;
} SimControls;

/*
 * Scene — every piece of state for one run, gathered in one place. A single
 * Scene lives on main's stack and is threaded by pointer through every tick,
 * draw, and input call. Functions that only read it take a const pointer;
 * functions that change it take a plain pointer, so the signature tells you
 * which is which. The only state kept outside Scene is the pair of signal
 * flags in §8, because signal handlers can't be handed a pointer.
 *
 * Members:
 *   voronoi      the moving seeds (the simulation data).
 *   sim          the paused/fps playback knobs.
 *   scene_cols   drawable width in characters (= terminal columns).
 *   scene_rows   drawable height in characters. Row 0 is the top HUD and the
 *                last row is the key-hint strip; the diagram fills the rows
 *                between.
 *
 * Both sizes are positive once screen_init runs, and every seed stays inside
 * the screen thanks to the bounce step.
 */
typedef struct {
    Voronoi     voronoi;
    SimControls sim;
    int         scene_cols;
    int         scene_rows;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->scene_cols = cols;
    s->scene_rows = rows;
    s->sim.paused = false;
    s->sim.fps    = SIM_FPS_DEFAULT;
    voronoi_init(&s->voronoi, cols, rows);
}

/* Re-roll the seeds (the 'r' key and a resize both call this). */
static void scene_reset_seeds(Scene *s)
{
    voronoi_reset(&s->voronoi, s->scene_cols, s->scene_rows);
}

/* Move the simulation forward one step, unless paused. */
static void scene_tick(Scene *s, float dt)
{
    if (s->sim.paused) return;
    voronoi_tick(&s->voronoi, dt, s->scene_cols, s->scene_rows);
}

/* Draw the diagram for the current seed positions. */
static void scene_draw(const Scene *s, WINDOW *w)
{
    voronoi_draw(&s->voronoi, w, s->scene_cols, s->scene_rows);
}

/* ── §7 screen — ncurses setup/teardown and the HUD strips ── */

/* Start ncurses and record the terminal size into the Scene. */
static void screen_init(Scene *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->scene_rows, s->scene_cols);
}

static void screen_free(void) { endwin(); }

/* One-word run-state indicator used in the top status string. */
static const char *run_state_label(const Scene *s)
{
    return s->sim.paused ? "PAUSED " : "running";
}

/* Format the right-aligned top status line into `buf`. */
static void format_hud_status(
    const Scene *s, double fps, char *buf, size_t bufsz
) {
    snprintf(buf, bufsz,
        " %5.1f fps  sim:%3d Hz  seeds:%d  %s ",
        fps, s->sim.fps, N_SEEDS, run_state_label(s));
}

/* Draw the status line flush-right on the top row. */
static void draw_top_status(const char *status, int term_cols)
{
    int pad = term_cols - (int)strlen(status);
    if (pad < 0) pad = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, pad, "%s", status);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Draw the bottom strip listing every key the user can press. */
static void draw_bottom_hint(int term_rows)
{
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(term_rows - 1, 0,
             " q:quit  spc:pause  r:reset  [/]:Hz ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* Draw both HUD strips: status on top, key hints on the bottom. */
static void draw_hud(const Scene *s, double fps)
{
    char status[80];
    format_hud_status(s, fps, status, sizeof status);
    draw_top_status(status, s->scene_cols);
    draw_bottom_hint(s->scene_rows);
}

/* Draw one full frame: clear, diagram, HUD, then flush in a single update. */
static void frame_render(const Scene *s, double fps)
{
    erase();
    scene_draw(s, stdscr);
    draw_hud(s, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §8 app — signals, input, and the fixed-timestep main loop ── */

/* These two flags are the only state a signal handler may touch, so they're
 * file-scope and volatile sig_atomic_t. The handlers set them; the main loop
 * checks and clears them. */
static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_exit_signal(int sig)   { (void)sig; g_running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Toggle the paused flag. */
static void sim_toggle_pause(Scene *s) { s->sim.paused = !s->sim.paused; }

/* Bump simulation fps up by SIM_FPS_STEP, capped at SIM_FPS_MAX. */
static void sim_fps_increase(Scene *s)
{
    s->sim.fps += SIM_FPS_STEP;
    if (s->sim.fps > SIM_FPS_MAX) s->sim.fps = SIM_FPS_MAX;
}

/* Bump simulation fps down by SIM_FPS_STEP, floored at SIM_FPS_MIN. */
static void sim_fps_decrease(Scene *s)
{
    s->sim.fps -= SIM_FPS_STEP;
    if (s->sim.fps < SIM_FPS_MIN) s->sim.fps = SIM_FPS_MIN;
}

/* Act on one keypress: q/Q/ESC quit, space pauses, r/R re-rolls the seeds,
 * ] speeds the sim up and [ slows it down. */
static void handle_input(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case KEY_ESC: g_running = 0;            break;
    case ' ':                         sim_toggle_pause(s);      break;
    case 'r': case 'R':               scene_reset_seeds(s);     break;
    case ']':                         sim_fps_increase(s);      break;
    case '[':                         sim_fps_decrease(s);      break;
    default: break;
    }
}

/* After a terminal resize, restart ncurses so it learns the new size, then
 * re-roll the seeds to fit the new screen. */
static void handle_resize_if_pending(Scene *s)
{
    if (!g_need_resize) return;
    g_need_resize = 0;
    endwin(); refresh();
    getmaxyx(stdscr, s->scene_rows, s->scene_cols);
    scene_reset_seeds(s);
}

/* ── §8 main-loop helpers — timing, fps meter, frame pacing ── */

/* Wire up the signal handlers and the atexit cleanup. */
static void signals_install(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* Seed the random generator from the clock so each run looks different
 * (just for variety, not security). */
static void rng_seed_from_clock(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
}

/* Cap how big one frame's elapsed time can be. If the program was paused or
 * stalled for a long moment, this stops the catch-up loop below from trying
 * to run hundreds of steps at once; the sim just slows down instead. */
static int64_t clamp_dt_to_prevent_spiral(int64_t dt_ns)
{
    return (dt_ns > MAX_FRAME_DT_NS) ? MAX_FRAME_DT_NS : dt_ns;
}

/* Run the simulation in fixed-size steps regardless of the actual frame rate:
 * add the elapsed time to a running bank and spend it one whole step at a
 * time. This keeps motion consistent on fast and slow machines alike. The
 * step length is read fresh each frame so [ / ] can change the tempo. */
static void advance_fixed_timestep_sim(
    Scene *s, int64_t *sim_accum_ns, int64_t dt_ns
) {
    int64_t tick_ns = TICK_NS(s->sim.fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    *sim_accum_ns += dt_ns;
    while (*sim_accum_ns >= tick_ns) {
        scene_tick(s, dt_sec);
        *sim_accum_ns -= tick_ns;
    }
}

/* Track the frame rate to show in the HUD: count frames over a short window
 * and recompute the displayed number only once per window, so it doesn't
 * jitter every frame. */
static void sample_fps(
    int64_t dt_ns, int *frame_count, int64_t *accum_ns, double *display
) {
    (*frame_count)++;
    *accum_ns += dt_ns;
    if (*accum_ns >= FPS_UPDATE_PERIOD_NS) {
        *display = (double)(*frame_count)
                 / ((double)(*accum_ns) / (double)NS_PER_SEC);
        *frame_count = 0;
        *accum_ns    = 0;
    }
}

/* Sleep off whatever time is left in this frame's budget to hold the target
 * frame rate. */
static void wait_until_next_frame(int64_t frame_start_ns)
{
    int64_t elapsed = clock_ns() - frame_start_ns;
    clock_sleep_ns(RENDER_FRAME_NS - elapsed);
}

/* Set everything up, then loop until quit: handle any resize, measure how
 * long the last frame took, step the simulation that far, refresh the fps
 * meter, draw, read a key, and sleep to pace the next frame. */
int main(void)
{
    rng_seed_from_clock();
    signals_install();

    Scene scene;
    screen_init(&scene);
    scene_init(&scene, scene.scene_cols, scene.scene_rows);

    int64_t frame_start = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (g_running) {
        if (g_need_resize) {
            handle_resize_if_pending(&scene);
            frame_start = clock_ns();
            sim_accum   = 0;
        }

        int64_t now      = clock_ns();
        int64_t dt_raw   = now - frame_start;
        int64_t dt       = clamp_dt_to_prevent_spiral(dt_raw);
        frame_start      = now;

        advance_fixed_timestep_sim(&scene, &sim_accum, dt);
        sample_fps(dt, &frame_count, &fps_accum, &fps_display);
        frame_render(&scene, fps_display);

        int ch = getch();
        if (ch != ERR) handle_input(&scene, ch);

        wait_until_next_frame(now);
    }

    screen_free();
    return 0;
}
