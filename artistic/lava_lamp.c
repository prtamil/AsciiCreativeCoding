/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lava_lamp.c — a lava-lamp animation built from blobs that merge and split.
 *
 * Each blob is a "metaball": instead of drawing a hard circle, every blob
 * spreads a soft glow over the screen and the glows add up. Where the total
 * glow is strong enough, we draw molten material. Refs: Blinn 1982 (metaballs),
 * Bloomenthal 1997 (why summed glows merge), Bourke 1997 (brightness→ASCII).
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — tunable constants, knob ranges, colour-pair IDs ── */

#define TARGET_FPS         30        /* slow scene; 30 fps is plenty       */

#define N_BLOBS_MAX        10
#define N_BLOBS_DEFAULT     6
#define N_BLOBS_MIN         3

#define BLOB_RADIUS_MIN     2.5f
#define BLOB_RADIUS_MAX     5.0f

/* Terminal cells are about twice as tall as wide, so a blob would look
 * stretched vertically. We squash the field vertically to compensate. */
#define ASPECT_K2           4.0f     /* squash factor (cells are ~2x tall)  */
#define FIELD_EPS2          0.5f     /* keeps the glow finite at a blob's
                                      * exact centre, instead of blowing up
                                      * to infinity (see §4 field_eval)      */

/* Cutoff for "how strong must the glow be to count as molten". A higher
 * cutoff means only the very centre of each blob shows, so blobs look smaller. */
#define THRESHOLD_DEFAULT   0.8f
#define THRESHOLD_MIN       0.3f
#define THRESHOLD_MAX       2.0f
#define THRESHOLD_STEP      0.1f

/* How strongly heat lifts a blob. Hotter-than-surroundings blobs rise. */
#define BUOYANCY_DEFAULT    8.0f
#define BUOYANCY_MIN        2.0f
#define BUOYANCY_MAX        20.0f
#define BUOYANCY_STEP       2.0f
#define DAMPING             1.0f     /* drag: shrinks speed per second      */
#define HORIZ_NOISE         3.0f     /* random sideways wobble amplitude    */
#define WALL_RESTITUTION    0.5f     /* speed fraction kept on a wall bounce */

/* Blobs warm up near the floor and cool down near the ceiling. */
#define HEAT_GAIN           0.30f    /* temp gained/sec in the lower half   */
#define HEAT_LOSS           0.20f    /* temp lost/sec in the upper half     */
#define BOUND_MARGIN        2        /* keep blobs this many cells inside   */

#define DT_CAP_S            0.10f    /* cap on one frame's elapsed time     */
#define N_THEMES            4
#define N_HEAT_STOPS        5        /* number of glyph/colour brightness steps */

/* A cell's brightness mixes two things: how deep inside a blob it is, and
 * how hot the nearby blobs are. These two weights add to 1. */
#define SHADE_INTENSITY_W   0.4f     /* weight of "how deep inside the blob" */
#define SHADE_TEMP_W        0.6f     /* weight of blob temperature           */
#define HOT_BUCKET          3        /* brightness step at which cells glow (bold) */

/* Colour pair IDs */
#define PAIR_HEAT_0  1   /* coolest                                       */
#define PAIR_HEAT_1  2
#define PAIR_HEAT_2  3
#define PAIR_HEAT_3  4
#define PAIR_HEAT_4  5   /* hottest                                       */
#define PAIR_WALL    6
#define PAIR_HUD     7
#define PAIR_HINT    8

/* ── §2 performance — clock and sleep helpers (frame cap lives in §6) ── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec  = (time_t)(ns / 1000000000LL),
                          .tv_nsec = (long)(ns % 1000000000LL) };
    nanosleep(&r, NULL);
}

/* ── §3 simulation — moves the blobs (the only code that changes blob state) ── */

/* Random helpers. frand gives 0..1, frand_signed gives -1..1. */
static float frand(void)        { return (float)rand() / (float)RAND_MAX; }
static float frand_signed(void) { return frand() * 2.0f - 1.0f; }

/* Blob — one lava blob, stored as just six numbers.
 *
 * A blob has no drawn shape of its own. Each frame it spreads a soft glow
 * over the screen (see §4 field_eval), and the on-screen blob is whatever
 * region ends up bright enough. Because glows simply add together, two blobs
 * that drift close merge into one shape and pull apart again on their own —
 * that organic lava merge/split comes for free, no special code. Refs: Blinn
 * 1982, Wyvill 1986.
 *
 * Each tick: the blob's height decides how warm its surroundings are; being
 * warmer than its surroundings pushes it up; that motion moves it; its new
 * height warms or cools it. So height and temperature chase each other, which
 * is what gives the slow rise-and-sink.
 *
 * Units: x/y/vx/vy/r are in terminal cells; temp is a plain 0..1 value. */
typedef struct {
    float x, y;     /* centre, in cells; y grows DOWNWARD (top of screen = 0)  */
    float vx, vy;   /* velocity, cells/sec; vy < 0 means moving up             */
    float r;        /* size of the blob, in cells: bigger = stronger, wider glow */
    float temp;     /* temperature, 0..1: 1 = hot (rises, bright colour),
                     * 0 = cold (sinks, dim colour). Warms near the floor,
                     * cools near the ceiling; also tints the colour (§4/§5).  */
} Blob;

static void blob_spawn(Blob *b, int rows, int cols)
{
    b->x   = (float)BOUND_MARGIN + frand() * (float)(cols - 2 * BOUND_MARGIN);
    b->y   = (float)BOUND_MARGIN + frand() * (float)(rows - 2 - 2 * BOUND_MARGIN);
    b->vx  = 0;
    b->vy  = 0;
    b->r   = BLOB_RADIUS_MIN + frand() * (BLOB_RADIUS_MAX - BLOB_RADIUS_MIN);
    b->temp = frand();
}

/* The temperature of the surroundings at height y: hot (1) at the floor,
 * cold (0) at the ceiling. A blob warmer than this rises, cooler sinks. */
static float ambient_temp(float y, int rows)
{
    float t = 1.0f - (y / (float)(rows - 2));
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return t;
}

/* Nudge the blob upward when it's hotter than its surroundings, and add a
 * little sideways wobble so blobs don't all rise in a straight line. */
static void blob_apply_buoyancy(Blob *b, float dt, float buoyancy, int rows)
{
    float t_ambient = ambient_temp(b->y, rows);
    b->vy += -buoyancy * (b->temp - t_ambient) * dt;   /* negative vy = up */
    b->vx += frand_signed() * HORIZ_NOISE * dt;
}

/* Drag: bleed off speed over time so blobs don't keep accelerating. */
static void blob_apply_damping(Blob *b, float dt)
{
    float k = 1.0f - DAMPING * dt;
    if (k < 0) k = 0;
    b->vx *= k;
    b->vy *= k;
}

/* Move the blob by its velocity for this slice of time. */
static void blob_integrate(Blob *b, float dt)
{
    b->x += b->vx * dt;
    b->y += b->vy * dt;
}

/* Warm the blob up while it's in the lower half, cool it in the upper half. */
static void blob_exchange_heat(Blob *b, float dt, int rows)
{
    if (b->y > (float)rows * 0.5f) b->temp += HEAT_GAIN * dt;
    else                           b->temp -= HEAT_LOSS * dt;
    if (b->temp < 0) b->temp = 0;
    if (b->temp > 1) b->temp = 1;
}

/* Bounce off the chamber walls, losing some speed on each bounce. */
static void blob_bounce_walls(Blob *b, int rows, int cols)
{
    float xmin = (float)BOUND_MARGIN;
    float xmax = (float)(cols - BOUND_MARGIN);
    float ymin = (float)BOUND_MARGIN;
    float ymax = (float)(rows - 2 - BOUND_MARGIN);
    if (b->x < xmin) { b->x = xmin; b->vx = fabsf(b->vx) * WALL_RESTITUTION; }
    if (b->x > xmax) { b->x = xmax; b->vx = -fabsf(b->vx) * WALL_RESTITUTION; }
    if (b->y < ymin) { b->y = ymin; b->vy = fabsf(b->vy) * WALL_RESTITUTION; }
    if (b->y > ymax) { b->y = ymax; b->vy = -fabsf(b->vy) * WALL_RESTITUTION; }
}

/* One motion step for a single blob, in order: lift, drag, move, change
 * temperature, then bounce off walls. */
static void blob_tick(Blob *b, float dt, float buoyancy, int rows, int cols)
{
    blob_apply_buoyancy(b, dt, buoyancy, rows);
    blob_apply_damping(b, dt);
    blob_integrate(b, dt);
    blob_exchange_heat(b, dt, rows);
    blob_bounce_walls(b, rows, cols);
}

/* Lamp — the whole scene: the blobs plus the few settings the user can tweak.
 *
 * The blob array is allocated once at full size and never grows; n_blobs just
 * says how many of those slots are currently in use. That keeps everything
 * allocation-free while running: ']' fills the next slot, '[' lowers the count.
 *
 * Fields are grouped by what they're about. theme is changed from the keyboard
 * like the physics knobs, but it only affects colour, so it sits on its own. */
typedef struct {
    /* The blobs (only the first n_blobs are live) */
    Blob blobs[N_BLOBS_MAX]; /* fixed-size pool, never resized while running    */
    int  n_blobs;            /* how many blobs are live, 3..10 ('[' / ']' keys) */

    /* Physics knobs the user can tune */
    float threshold;        /* how strong the glow must be to draw molten material.
                             * Higher = blobs look smaller. 0.3..2.0, -/+ keys   */
    float buoyancy;         /* how strongly heat lifts a blob. Bigger = livelier
                             * rise and sink. 2..20, , / . keys                  */

    /* Colour choice (not physics) */
    int   theme;            /* which palette, 0..N_THEMES-1; the t key cycles it */

    /* Run state */
    int   paused;           /* nonzero = freeze motion but keep drawing (p key)  */
} Lamp;

/* Scatter all live blobs to fresh random spots (used on reset and resize). */
static void lamp_reseed(Lamp *l, int rows, int cols)
{
    for (int i = 0; i < l->n_blobs; i++) blob_spawn(&l->blobs[i], rows, cols);
}

/* Advance the whole scene by one step: move every live blob. */
static void lamp_tick(Lamp *l, float dt, int rows, int cols)
{
    for (int i = 0; i < l->n_blobs; i++)
        blob_tick(&l->blobs[i], dt, l->buoyancy, rows, cols);
}

/* ── §4 logic — pure math: no state changes, no drawing ── */

/* Total glow at point (x, y): add up each blob's glow there. A blob glows
 * brighter the bigger it is and the closer you are to its centre, and the
 * glow fades smoothly with distance. Returns that total.
 *
 * Also reports, via out_t_avg, the average temperature of the blobs that
 * matter at this point — blobs glowing harder here count more. The renderer
 * uses that to pick the colour. */
static float field_eval(const Blob *blobs, int n_blobs,
                        float x, float y, float *out_t_avg)
{
    float total = 0.0f;
    float wsum  = 0.0f;
    float tw    = 0.0f;
    for (int i = 0; i < n_blobs; i++) {
        float dx = x - blobs[i].x;
        float dy = (y - blobs[i].y);
        float d2 = dx * dx + dy * dy * ASPECT_K2 + FIELD_EPS2;
        float w  = blobs[i].r * blobs[i].r / d2;
        total += w;
        wsum  += w;
        tw    += w * blobs[i].temp;
    }
    if (out_t_avg) *out_t_avg = (wsum > 1e-6f) ? (tw / wsum) : 0.0f;
    return total;
}

/* Turn a 0..1 brightness into one of the discrete brightness steps (0..4),
 * which then picks a glyph and colour. */
static int heat_bucket(float t)
{
    int b = (int)(t * 4.99f);   /* just under 5, so 1.0 still lands on step 4 */
    if (b < 0)                 b = 0;
    if (b > N_HEAT_STOPS - 1)  b = N_HEAT_STOPS - 1;
    return b;
}

/* Final brightness (0..1) for a molten cell: blend how deep inside the blob it
 * is with the nearby blobs' average temperature, so cores look bright and cool
 * blobs look dim. The caller has already confirmed this cell is molten. */
static float cell_shade(float f, float threshold, float t_avg)
{
    float intensity = (f - threshold) / (threshold * 2.0f);
    if (intensity < 0) intensity = 0;
    if (intensity > 1) intensity = 1;
    return SHADE_INTENSITY_W * intensity + SHADE_TEMP_W * t_avg;
}

/* ── §5 render — draws the scene; only reads state, never changes it ── */

/* One colour ramp per theme, dim-to-bright. The dimmest step is kept fairly
 * bright so cool/sinking blobs stay visible against a dark terminal. */
static const short HEAT_256[N_THEMES][N_HEAT_STOPS] = {
    /* 0 lava   — red → orange → yellow → white   */
    { 160, 196, 208, 220, 231 },
    /* 1 ocean  — sky blue → cyan → white         */
    {  39,  45,  51,  87, 231 },
    /* 2 toxic  — medium green → lime → white     */
    {  34,  82, 118, 154, 231 },
    /* 3 royal  — purple → pink → white           */
    {  91, 127, 165, 213, 231 },
};
static const short HEAT_8[N_THEMES][N_HEAT_STOPS] = {
    { COLOR_RED,     COLOR_RED,     COLOR_YELLOW,  COLOR_YELLOW,  COLOR_WHITE },
    { COLOR_BLUE,    COLOR_BLUE,    COLOR_CYAN,    COLOR_CYAN,    COLOR_WHITE },
    { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW,  COLOR_WHITE },
    { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE },
};

static void color_init(int theme)
{
    start_color(); use_default_colors();
    int x256 = (COLORS >= 256);
    for (int i = 0; i < N_HEAT_STOPS; i++) {
        short fg = x256 ? HEAT_256[theme][i] : HEAT_8[theme][i];
        init_pair((short)(PAIR_HEAT_0 + i), fg, -1);
    }
    init_pair(PAIR_WALL, x256 ? 240 : COLOR_WHITE, -1);
    init_pair(PAIR_HUD,  x256 ? 226 : COLOR_YELLOW, -1);  /* top: bright yellow */
    init_pair(PAIR_HINT, x256 ?  51 : COLOR_CYAN,   -1);  /* bottom: bright cyan */
}

/* Characters from faint to solid, one per brightness step. */
static const char HEAT_GLYPH[N_HEAT_STOPS] = { '`', '.', '*', 'o', '#' };

static void draw_walls(int rows, int cols)
{
    attron(COLOR_PAIR(PAIR_WALL) | A_DIM);
    /* Top + bottom */
    for (int c = 0; c < cols; c++) {
        mvaddch(0,        c, (chtype)'_');
        if (rows - 2 >= 0) mvaddch(rows - 2, c, (chtype)'_');
    }
    /* Left + right */
    for (int r = 1; r < rows - 1; r++) {
        if (cols >= 1)       mvaddch(r, 0,        (chtype)'|');
        if (cols - 1 >= 0)   mvaddch(r, cols - 1, (chtype)'|');
    }
    attroff(COLOR_PAIR(PAIR_WALL) | A_DIM);
}

static void draw_field(const Lamp *l, int rows, int cols)
{
    /* Walk every cell inside the chamber walls and draw it if it's molten. */
    for (int sr = 1; sr < rows - 2; sr++) {
        for (int sc = 1; sc < cols - 1; sc++) {
            float t_avg;
            float f = field_eval(l->blobs, l->n_blobs,
                                 (float)sc, (float)sr, &t_avg);
            if (f < l->threshold) continue;          /* not bright enough; skip */

            int bucket = heat_bucket(cell_shade(f, l->threshold, t_avg));

            /* The brightest cells are drawn bold so the cores glow. */
            chtype attr = COLOR_PAIR(PAIR_HEAT_0 + bucket);
            if (bucket >= HOT_BUCKET) attr |= A_BOLD;
            attron(attr);
            mvaddch(sr, sc, (chtype)(unsigned char)HEAT_GLYPH[bucket]);
            attroff(attr);
        }
    }
}

/* Status readout in the top-right corner, key reminders along the bottom. */
static void draw_hud(int rows, int cols, const Lamp *l, double fps)
{
    char buf[160];
    snprintf(buf, sizeof buf,
             " blobs:%d  thresh:%.2f  buoy:%.1f  theme:%d  "
             "%5.1f fps  %s ",
             l->n_blobs, l->threshold, l->buoyancy, l->theme, fps,
             l->paused ? "PAUSED " : "running");
    int x = cols - (int)strlen(buf);
    if (x < 0) x = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, x, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " [/]:blobs  -/+:thresh  ,/.:buoyancy  t:theme  "
             "r:reset  p:pause  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Lamp *l, double fps)
{
    erase();
    draw_walls(rows, cols);
    draw_field(l, rows, cols);
    draw_hud(rows, cols, l, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

static void screen_cleanup(void) { endwin(); }

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init(theme);
    atexit(screen_cleanup);
}

/* ── §6 app — the main loop: handle input, advance, draw, cap the frame rate ── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

static Lamp g_lamp;

/* Act on one keypress (change a setting, reset, pause, etc). Returns false
 * when the user asks to quit. */
static bool lamp_handle_key(Lamp *l, int ch, int rows, int cols)
{
    switch (ch) {
        case 'q': case 27: return false;
        case 'p': l->paused ^= 1; break;
        case 'r': lamp_reseed(l, rows, cols); break;
        case 't': l->theme = (l->theme + 1) % N_THEMES; color_init(l->theme); break;
        case '[':
            if (l->n_blobs > N_BLOBS_MIN) l->n_blobs--;
            break;
        case ']':
            if (l->n_blobs < N_BLOBS_MAX) {
                blob_spawn(&l->blobs[l->n_blobs], rows, cols);
                l->n_blobs++;
            }
            break;
        case '-':
            if (l->threshold + THRESHOLD_STEP <= THRESHOLD_MAX)
                l->threshold += THRESHOLD_STEP;
            break;
        case '+': case '=':
            if (l->threshold - THRESHOLD_STEP >= THRESHOLD_MIN)
                l->threshold -= THRESHOLD_STEP;
            break;
        case ',':
            if (l->buoyancy - BUOYANCY_STEP >= BUOYANCY_MIN)
                l->buoyancy -= BUOYANCY_STEP;
            break;
        case '.':
            if (l->buoyancy + BUOYANCY_STEP <= BUOYANCY_MAX)
                l->buoyancy += BUOYANCY_STEP;
            break;
    }
    return true;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    srand((unsigned)time(NULL));

    g_lamp.n_blobs         = N_BLOBS_DEFAULT;
    g_lamp.threshold = THRESHOLD_DEFAULT;
    g_lamp.buoyancy  = BUOYANCY_DEFAULT;
    g_lamp.theme     = 0;

    screen_init(g_lamp.theme);
    int rows = LINES, cols = COLS;
    lamp_reseed(&g_lamp, rows, cols);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps         = TARGET_FPS;
    int64_t t_fps_prev  = clock_ns();
    int64_t t_tick_prev = t_fps_prev;

    while (g_running) {
        /* Terminal was resized: pick up the new size and re-scatter the blobs. */
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            lamp_reseed(&g_lamp, rows, cols);
        }

        /* Handle every key that's been pressed since last frame. */
        int ch;
        while ((ch = getch()) != ERR)
            if (!lamp_handle_key(&g_lamp, ch, rows, cols)) g_running = 0;

        /* Time since last frame. Capped so a long pause can't make blobs
         * teleport (e.g. when the window was hidden). */
        int64_t now = clock_ns();
        float   dt  = (float)(now - t_tick_prev) / 1e9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;
        t_tick_prev = now;

        /* Advance the blobs, unless paused. */
        if (!g_lamp.paused) lamp_tick(&g_lamp, dt, rows, cols);

        /* Smoothed frame rate for the HUD (mostly the old value plus a little
         * of the new one, so it doesn't jitter). */
        fps = fps * 0.95 + (1e9 / (double)(now - t_fps_prev + 1)) * 0.05;
        t_fps_prev = now;

        scene_draw(rows, cols, &g_lamp, fps);

        /* Sleep off whatever time is left to hold a steady frame rate. */
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
