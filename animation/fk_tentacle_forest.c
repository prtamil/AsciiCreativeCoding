/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fk_tentacle_forest.c — eight seaweed strands swaying in an ocean current.
 *
 * Forward kinematics: set each segment's bend angle and the rest of the
 * chain follows down from the fixed root. Each segment's angle wobbles a
 * bit over time (a sine wave), so the whole tentacle ripples. Nothing is
 * stored between frames — the entire forest is recomputed from one clock.
 *
 * Sister files: fk_centipede.c, snake_forward_kinematics.c (same FK idea,
 * with a moving body on top). Tamilselvan R, MIT licence.
 *
 * Build: gcc -std=c11 -O2 -Wall -Wextra animation/fk_tentacle_forest.c \
 *            -o fk_tentacle_forest -lncurses -lm   (-lm for the trig)
 */

#define _POSIX_C_SOURCE 200809L

/* M_PI is a POSIX extension, not standard C99/C11 — provide a fallback
 * so the build never fails on strict-conformance toolchains. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
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

/* ── §1 config — every tunable number, named in one place ── */

enum {
    /* Frame cap. Sim is variable-timestep, so this only sets how long
     * we sleep at the end of each frame. */
    TARGET_FPS    = 60,

    HUD_COLS      = 96,    /* max bytes in the top status bar    */
    FPS_UPDATE_MS = 500,   /* how often the fps readout refreshes */

    /* Colour-pair IDs. 1..N_PAIRS = the root-to-tip body gradient;
     * PAIR_HUD/PAIR_HINT are the two reserved HUD colours. */
    N_PAIRS   = 7,
    PAIR_HUD  = 8,
    PAIR_HINT = 9,

    N_TENTACLES = 8,   /* strands along the sea floor; 8 spaces evenly  */
    N_SEGS      = 16,  /* segments per strand; 16 gives a smooth S-curve */
};

/* Pixel step for stamping glyphs along a segment. Smaller than CELL_W (8)
 * so a near-horizontal segment can't skip over a whole cell. */
#define DRAW_STEP_PX   5.0f

/* How much the wobble phase shifts from one segment to the next (radians).
 * This is what makes the ripple appear to travel up the strand instead of
 * the whole thing swinging as one stiff rod. 0 = rigid rod; bigger = more
 * bends packed into the chain. 0.45 over 16 segments ≈ one gentle S. */
#define PHASE_PER_SEG  0.45f

/* Sway strength and speed (both adjustable at runtime).
 *   AMP  = peak bend per segment, in radians (~0.28 ≈ 16 degrees).
 *   FREQ = how fast each segment wobbles, in radians/sec (~0.8 → one full
 *          sway every ~8 s, the lazy pace of real seaweed). */
#define AMP_DEFAULT    0.28f
#define AMP_MIN        0.0f
#define AMP_MAX        1.2f   /* just below the point where a strand curls up */
#define FREQ_DEFAULT   0.8f
#define FREQ_MIN       0.1f
#define FREQ_MAX       5.0f

/* Each strand wobbles at a slightly different speed (this much added or
 * subtracted) so the eight never fall into lockstep. */
#define FREQ_OFFSET_MAG  0.04f

/* Simulation-speed multiplier, stepped up/down by the [ and ] keys. */
#define TIME_SCALE_DEFAULT  1.0f
#define TIME_SCALE_MIN      0.25f
#define TIME_SCALE_MAX      4.0f
#define TIME_SCALE_STEP     1.5f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* Sub-pixels per character cell. Physics runs in these square pixels;
 * cells are ~2x taller than wide, so x and y use different divisors. */
#define CELL_W   8
#define CELL_H  16

/* ── §2 clock — monotonic time + sleep ── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);    /* never goes backward */
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;                   /* over-budget frame: skip */
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ── §3 color — deep-blue root to sunlit-green tip ── */

/*
 * The body gradient goes deep blue at the root to bright yellow-green at
 * the tip, like looking up from the dark seafloor toward sunlit water.
 * Every colour is in the bright half of the palette so the dimmed root
 * segments (drawn with A_DIM) stay visible. The two HUD colours never
 * change with any theme.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(1,  24, -1);   /* deep blue       — root, sea floor    */
        init_pair(2,  27, -1);   /* medium blue                          */
        init_pair(3,  33, -1);   /* cyan-blue                            */
        init_pair(4,  51, -1);   /* bright cyan     — mid-body           */
        init_pair(5,  86, -1);   /* cyan-green                           */
        init_pair(6, 118, -1);   /* yellow-green                         */
        init_pair(7, 154, -1);   /* bright yellow-green — tip, sunlit    */
    } else {
        /* 8-color fallback — coarser gradient, still directionally right. */
        init_pair(1, COLOR_BLUE,  -1);
        init_pair(2, COLOR_BLUE,  -1);
        init_pair(3, COLOR_CYAN,  -1);
        init_pair(4, COLOR_CYAN,  -1);
        init_pair(5, COLOR_CYAN,  -1);
        init_pair(6, COLOR_GREEN, -1);
        init_pair(7, COLOR_GREEN, -1);
    }

    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 coords — turn pixel positions into cell positions ── */

/*
 * Physics works in square pixels; the terminal is a grid of cells. These
 * two helpers convert px to the nearest cell, the only place that happens.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5 entity — Tentacle: one swaying strand ── */

/*
 * Vec2 — a point in pixel space (8x16 sub-pixels per cell, so motion looks
 * smooth instead of snapping cell-to-cell).
 *   x : pixels rightward  (positive = toward the right edge)
 *   y : pixels downward   (positive = toward the bottom, screen convention)
 * Angles below use math convention (positive = counter-clockwise from +x);
 * since y grows downward, "up the screen" is the angle -pi/2.
 */
typedef struct {
    float x;   /* pixels rightward (positive = right) */
    float y;   /* pixels downward  (positive = down)  */
} Vec2;

/*
 * Tentacle — one seaweed strand: a chain of stiff segments hinged
 * end-to-end. The first four fields are set once and never change; they
 * pin the strand to the floor and give it its own wobble. joint[] is the
 * output, rebuilt from scratch every frame by tentacle_tick() — the strand
 * keeps no memory between frames, only its fixed personality.
 */
typedef struct {
    /* Fixed at startup (scene_init), never touched again. */
    float root_px;        /* x of the pinned base, pixels                    */
    float root_py;        /* y of the pinned base, pixels                    */
    float root_phase;     /* this strand's starting wobble offset, radians;  *
                           * strands get evenly-spread values so they don't  *
                           * all start bending the same way                  */
    float freq_offset;    /* tiny speed tweak, radians/sec; spread symmetric *
                           * around 0 so the forest's average speed still    *
                           * equals the user-set frequency                   */

    /* Rebuilt every frame. joint[0] = the pinned base, joint[N_SEGS] = the
     * free tip; N segments need N+1 joints. */
    Vec2  joint[N_SEGS + 1];
} Tentacle;

/* ── §5a tentacle_tick — place every joint from the clock ── */

/*
 * Rebuild one strand's joints from the current wave_time. Walk from the
 * base upward, keeping a running angle: start pointing up (-pi/2), then at
 * each segment add a small wobble (a sine of time, position, and this
 * strand's phase) and step one segment length in that direction. Because
 * each segment inherits the running angle, a wobble near the base ripples
 * all the way to the tip — no forces, just summed angles.
 */
static void tentacle_tick(Tentacle *t,
                          float wave_time,
                          float amplitude,
                          float frequency,
                          float seg_len_px)
{
    t->joint[0].x = t->root_px;
    t->joint[0].y = t->root_py;

    float cumulative_angle = -(float)M_PI * 0.5f;   /* start pointing up */

    for (int i = 0; i < N_SEGS; i++) {
        float delta = amplitude
                    * sinf((frequency + t->freq_offset) * wave_time
                           + t->root_phase
                           + (float)i * PHASE_PER_SEG);
        cumulative_angle += delta;

        t->joint[i + 1].x = t->joint[i].x
                          + seg_len_px * cosf(cumulative_angle);
        t->joint[i + 1].y = t->joint[i].y
                          + seg_len_px * sinf(cumulative_angle);
    }
}

/* ── §5b rendering helpers — pick colour, shade, and glyph ── */

/* Colour pair for segment i: pair 1 at the base, N_PAIRS at the tip,
 * stepping evenly through the blue-to-green gradient in between. */
static int seg_pair(int i)
{
    return 1 + (i * (N_PAIRS - 1)) / (N_SEGS - 1);
}

/* Brightness for segment i: dim near the base (deep, dark water), normal
 * in the middle, bold at the tip (sunlit shallows). */
static attr_t seg_attr(int i)
{
    if (i <     N_SEGS / 4) return A_DIM;
    if (i > 3 * N_SEGS / 4) return A_BOLD;
    return A_NORMAL;
}

/* The glyph drawn at joint i, shrinking from base to tip:
 *   '#' base anchor, 'O' fat lower nodes, 'o' mid nodes,
 *   '.' thin upper nodes, '*' the free tip. */
static chtype node_marker(int i)
{
    if (i == 0)                  return (chtype)(unsigned char)'#';
    if (i <=     N_SEGS / 4)     return (chtype)(unsigned char)'O';
    if (i <= 3 * N_SEGS / 4)     return (chtype)(unsigned char)'o';
    if (i <  N_SEGS)             return (chtype)(unsigned char)'.';
    return                              (chtype)(unsigned char)'*';
}

/*
 * Pick the ASCII glyph that best shows the direction (dx, dy): '-' for
 * near-horizontal, '|' for near-vertical, '/' and '\' for the diagonals.
 * The angle is folded into a half-circle because each glyph looks the same
 * upside down. dy is negated first so the glyph matches what the eye sees
 * (screen y grows downward, but the glyphs are drawn for upward math y).
 */
static chtype seg_glyph(float dx, float dy)
{
    float ang = atan2f(-dy, dx);
    float deg = ang * (180.0f / (float)M_PI);
    if (deg <    0.0f) deg += 360.0f;
    if (deg >= 180.0f) deg -= 180.0f;

    if (deg < 22.5f || deg >= 157.5f) return (chtype)(unsigned char)'-';
    if (deg < 67.5f)                   return (chtype)(unsigned char)'\\';
    if (deg < 112.5f)                  return (chtype)(unsigned char)'|';
    return                             (chtype)(unsigned char)'/';
}

/* ── §5c draw_segment_dense — fill one segment with glyphs ── */

/*
 * Stamp a direction glyph in every cell the line from a to b crosses. We
 * walk the line in small steps (DRAW_STEP_PX, smaller than a cell) so no
 * cell is missed. prev_cx/prev_cy is the last cell we drew; the caller
 * keeps it across all segments of one strand so a cell never gets stamped
 * twice where two segments meet. It starts at -9999 so the very first cell
 * always draws. Rows 0 and rows-1 are left alone — they hold the HUD bars.
 */
static void draw_segment_dense(WINDOW *w,
                                Vec2 a, Vec2 b,
                                int pair, attr_t attr,
                                int cols, int rows,
                                int *prev_cx, int *prev_cy)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;       /* degenerate: nothing to draw */

    chtype glyph  = seg_glyph(dx, dy);
    int    nsteps = (int)ceilf(len / DRAW_STEP_PX) + 1;

    for (int s = 0; s <= nsteps; s++) {
        float u  = (float)s / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == *prev_cx && cy == *prev_cy) continue;
        *prev_cx = cx;
        *prev_cy = cy;

        if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1) continue;

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, glyph);
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/* ── §5d render_tentacle — draw one strand in two passes ── */

/* Pass 1: fill in the strand's segments with direction glyphs, base to tip.
 * Going base-to-tip means upper segments paint over lower ones where a
 * strand curls back on itself at high amplitude. */
static void draw_tentacle_lines(const Tentacle *t, WINDOW *w,
                                int cols, int rows)
{
    int prev_cx = -9999, prev_cy = -9999;   /* last cell drawn; shared so joints aren't double-stamped */
    for (int i = 0; i < N_SEGS; i++) {
        draw_segment_dense(w, t->joint[i], t->joint[i + 1],
                           seg_pair(i), seg_attr(i),
                           cols, rows, &prev_cx, &prev_cy);
    }
}

/* Pass 2: drop a bold marker on each joint, over the line fill. The marker
 * shrinks from base to tip, giving the strand its knuckled look. */
static void draw_tentacle_nodes(const Tentacle *t, WINDOW *w,
                                int cols, int rows)
{
    for (int i = 0; i <= N_SEGS; i++) {
        int cx = px_to_cell_x(t->joint[i].x);
        int cy = px_to_cell_y(t->joint[i].y);
        if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1) continue;

        int    pair  = seg_pair(i < N_SEGS ? i : N_SEGS - 1);
        chtype glyph = node_marker(i);

        wattron(w, COLOR_PAIR(pair) | A_BOLD);
        mvwaddch(w, cy, cx, glyph);
        wattroff(w, COLOR_PAIR(pair) | A_BOLD);
    }
}

/* Draw lines first, then nodes on top, so joint markers always win. */
static void render_tentacle(const Tentacle *t, WINDOW *w,
                            int cols, int rows)
{
    draw_tentacle_lines(t, w, cols, rows);
    draw_tentacle_nodes(t, w, cols, rows);
}

/* ── §6 scene — the whole forest of strands ── */

/*
 * Scene — all eight strands plus the parameters they share. wave_time is
 * the one number that carries from frame to frame; everything visible is
 * recomputed from it, so the same wave_time always produces the same
 * forest. amplitude, frequency, and seg_len_px are shared inputs; what
 * makes the strands look different is each one's own root_phase and
 * freq_offset (in Tentacle). seg_len_px lives here because it changes the
 * actual shape of a strand, not just how it's drawn.
 */
typedef struct {
    Tentacle t[N_TENTACLES];   /* the forest                              */
    float wave_time;           /* the master clock, seconds; only state   *
                                * that survives between frames            */
    float amplitude;           /* sway strength, radians (a/d keys)       */
    float frequency;           /* sway speed, radians/sec (w/s keys)      */
    float seg_len_px;          /* length of one segment, pixels; set from *
                                * screen height, redone on resize         */
    bool  paused;              /* true freezes the clock; drawing still   *
                                * runs, so the forest just holds still     */
} Scene;

/*
 * Lay out the forest: space the roots evenly across the floor and give
 * each strand its own phase and speed offset. The +1 in the spacing
 * divisor keeps strands off the screen edges (1/9, 2/9, ... 8/9 across).
 * Roots sit a few pixels into the seabed row, and segment length is sized
 * from screen height so tips land around mid-screen.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    sc->amplitude  = AMP_DEFAULT;
    sc->frequency  = FREQ_DEFAULT;
    sc->wave_time  = 0.0f;
    sc->paused     = false;
    sc->seg_len_px = (float)(rows * CELL_H) * 0.55f / (float)N_SEGS;

    float screen_wpx = (float)(cols * CELL_W);
    float root_py    = (float)(rows * CELL_H) - 4.0f;

    for (int i = 0; i < N_TENTACLES; i++) {
        Tentacle *t = &sc->t[i];

        t->root_px     = (float)(i + 1) * screen_wpx
                       / (float)(N_TENTACLES + 1);
        t->root_py     = root_py;
        t->root_phase  = (float)i * 2.0f * (float)M_PI
                       / (float)N_TENTACLES;
        t->freq_offset = ((float)i - (float)N_TENTACLES * 0.5f)
                       * FREQ_OFFSET_MAG;

        /* Seed all joints straight up — overwritten on first tick. */
        for (int k = 0; k <= N_SEGS; k++) {
            t->joint[k].x = t->root_px;
            t->joint[k].y = t->root_py - (float)k * sc->seg_len_px;
        }
    }
}

/*
 * Advance the forest by dt seconds: move the clock forward, then rebuild
 * every strand from it. While paused we skip everything, so the clock and
 * the shapes stay exactly where they were.
 */
static void scene_tick(Scene *sc, float dt)
{
    if (sc->paused) return;
    sc->wave_time += dt;
    for (int i = 0; i < N_TENTACLES; i++) {
        tentacle_tick(&sc->t[i], sc->wave_time,
                      sc->amplitude, sc->frequency, sc->seg_len_px);
    }
}

/* A dim row of '~' under the strands for a bit of seabed atmosphere. */
static void draw_seabed(WINDOW *w, int cols, int rows)
{
    int seabed_row = rows - 2;     /* rows-1 is the hint bar, so sit one above */
    if (seabed_row < 1) return;

    wattron(w, COLOR_PAIR(1) | A_DIM);
    for (int c = 0; c < cols; c++)
        mvwaddch(w, seabed_row, c, (chtype)(unsigned char)'~');
    wattroff(w, COLOR_PAIR(1) | A_DIM);
}

/* scene_draw — read-only render: tentacles + seabed atmosphere. */
static void scene_draw(const Scene *sc, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_TENTACLES; i++)
        render_tentacle(&sc->t[i], w, cols, rows);
    draw_seabed(w, cols, rows);
}

/* ── §7 screen — ncurses display layer ── */

/*
 * Screen — the terminal size, cached so drawing code can read cols/rows as
 * plain ints instead of asking ncurses every frame. Refreshed only on a
 * resize. ncurses counts in cells, not pixels.
 */
typedef struct {
    int cols;   /* terminal width  in cells */
    int rows;   /* terminal height in cells */
} Screen;

/* The non-obvious call here is typeahead(-1): without it, ncurses peeks
 * at stdin during output writes, which can tear frames mid-update. */
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

static void screen_free(Screen *s) { (void)s; endwin(); }

/* SIGWINCH path: endwin()+refresh() forces ncurses to re-read LINES/COLS
 * from the kernel; we then sample the fresh dimensions. */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * Draw one full frame: clear, draw the forest and seabed, then the status
 * line (top-right) and key hints (bottom). The HUD colours are bright and
 * bold on purpose so they stay readable over any animation behind them;
 * never dim the hint strip or it vanishes on bright frames.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, float time_scale)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %.1ffps  %.2fx  amp:%.2f  freq:%.2f  %s ",
             fps, time_scale, sc->amplitude, sc->frequency,
             sc->paused ? "PAUSED" : "swaying");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  w/s:freq  a/d:amp  [/]:time ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §8 app — signals, resize, main loop ── */

/*
 * App — everything outside the forest: the world (Scene), the terminal
 * (Screen), and the loop's control flags. It lives at file scope (g_app)
 * because signal handlers can't take an argument, so they need a global to
 * flip running and need_resize.
 *
 * running and need_resize are volatile sig_atomic_t because a signal can
 * change them at any moment: volatile forces a fresh read each loop, and
 * sig_atomic_t guarantees the read/write can't be seen half-done.
 */
typedef struct {
    Scene  scene;              /* the forest (§6)        */
    Screen screen;             /* terminal size (§7)     */

    float                 time_scale;   /* speed multiplier; 1.0 = realtime ([ ] keys) */
    volatile sig_atomic_t running;      /* loop runs while set; cleared on SIGINT/TERM */
    volatile sig_atomic_t need_resize;  /* set by SIGWINCH, cleared after we resize    */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/* atexit safety net — endwin() called on every exit path. */
static void cleanup(void) { endwin(); }

/*
 * Handle a resize: rebuild the forest's geometry for the new screen size,
 * but carry over wave_time, amplitude, and frequency so the animation
 * keeps going from where it was instead of jumping back to the start.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);

    float saved_wave_time = app->scene.wave_time;
    float saved_amp       = app->scene.amplitude;
    float saved_freq      = app->scene.frequency;

    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    app->scene.wave_time = saved_wave_time;
    app->scene.amplitude = saved_amp;
    app->scene.frequency = saved_freq;

    app->need_resize = 0;
}

/*
 * Act on one keypress; return false to quit. Letter keys (w/s, a/d) mirror
 * the arrows because some terminals eat arrow-key escape sequences.
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': sc->paused = !sc->paused; break;

    case KEY_UP:   case 'w': case 'W':
        sc->frequency += 0.15f;
        if (sc->frequency > FREQ_MAX) sc->frequency = FREQ_MAX;
        break;
    case KEY_DOWN: case 's': case 'S':
        sc->frequency -= 0.15f;
        if (sc->frequency < FREQ_MIN) sc->frequency = FREQ_MIN;
        break;

    case KEY_RIGHT: case 'd': case 'D':
        sc->amplitude += 0.10f;
        if (sc->amplitude > AMP_MAX) sc->amplitude = AMP_MAX;
        break;
    case KEY_LEFT:  case 'a': case 'A':
        sc->amplitude -= 0.10f;
        if (sc->amplitude < AMP_MIN) sc->amplitude = AMP_MIN;
        break;

    case ']':
        app->time_scale *= TIME_SCALE_STEP;
        if (app->time_scale > TIME_SCALE_MAX) app->time_scale = TIME_SCALE_MAX;
        break;
    case '[':
        app->time_scale /= TIME_SCALE_STEP;
        if (app->time_scale < TIME_SCALE_MIN) app->time_scale = TIME_SCALE_MIN;
        break;

    default: break;
    }
    return true;
}

/*
 * main — the render loop. Each frame: read keys, handle any resize, measure
 * how long the last frame took (capped at 100 ms so a paused/suspended
 * program doesn't lurch), advance the forest by that much, update the fps
 * readout, draw, then sleep to hold the target frame rate.
 */
int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app        = &g_app;
    app->running    = 1;
    app->time_scale = TIME_SCALE_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    const int64_t target_ns = NS_PER_SEC / TARGET_FPS;

    int64_t last_time   = clock_ns();
    int64_t fps_accum   = 0;
    int     fps_frames  = 0;
    double  fps_display = 0.0;

    while (app->running) {

        int64_t frame_start = clock_ns();

        /* ① drain input */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }
        if (!app->running) break;

        /* ② resize */
        if (app->need_resize) {
            app_do_resize(app);
            last_time = clock_ns();
        }

        /* ③ measure dt */
        int64_t dt_ns = frame_start - last_time;
        last_time     = frame_start;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;
        float dt = (float)dt_ns / (float)NS_PER_SEC;

        /* ④ tick */
        scene_tick(&app->scene, dt * app->time_scale);

        /* ⑤ fps counter */
        fps_frames++;
        fps_accum += dt_ns;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)fps_frames
                        / ((double)fps_accum / (double)NS_PER_SEC);
            fps_frames = 0;
            fps_accum  = 0;
        }

        /* ⑥ render */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->time_scale);
        screen_present();

        /* ⑦ frame cap — sleep so total frame ≈ target_ns. The math is
         *    just (target − elapsed); no spurious +dt terms. */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(target_ns - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
