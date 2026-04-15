/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * framework.c — Terminal Animation Framework: Complete Reference Template
 *
 * DEMO: Key Generator — 26 character slots centered on screen, each
 *       independently cycling through random printable ASCII characters
 *       (movie-style cryptographic key generation effect).
 *
 * This file is the canonical framework template for every animation in
 * this project. Study bounce_ball.c alongside this file — that is the
 * motion-physics reference; this is the stationary-entity reference.
 *
 * ─────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────
 *   §1  config   — every tunable constant in one place
 *   §2  clock    — monotonic nanosecond clock + sleep
 *   §3  color    — ncurses color pair setup (256-color / 8-color fallback)
 *   §4  coords   — pixel↔cell conversion; the aspect-ratio fix
 *   §5  entity   — per-animation state and update logic  (KeyGen here)
 *   §6  scene    — entity pool; tick (fixed-step); draw (interpolated)
 *   §7  screen   — ncurses double-buffer display layer
 *   §8  app      — signals, resize, main loop
 * ─────────────────────────────────────────────────────────────────────
 *
 * Main loop order (same in every animation):
 *
 *   ① measure dt (wall-clock elapsed since last frame)
 *   ② drain sim accumulator → fixed-step physics ticks
 *   ③ compute alpha (sub-tick render offset ∈ [0,1))
 *   ④ sleep to cap at 60 fps  ← BEFORE render, not after
 *   ⑤ build frame in newscr  → erase → scene_draw → HUD
 *   ⑥ doupdate()             → one diff write to terminal
 *   ⑦ poll input (non-blocking getch)
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          re-randomise all slots
 *   + / =      faster character cycling
 *   -          slower character cycling
 *   ] / [      raise / lower simulation Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra framework.c -o framework -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Reference framework template demonstrating the canonical
 *                  pattern used by all animations in this project:
 *                  fixed-step physics accumulator + render interpolation.
 *
 * Data-structure : Fixed-step accumulator: sim_accum += dt each frame;
 *                  drain in SIM_TICK_NS steps: while (accum ≥ tick) {
 *                  sim_tick(); accum -= tick; }  This decouples physics
 *                  rate from render rate — physics always runs at the same
 *                  speed regardless of CPU or render load.
 *
 * Rendering      : Sub-tick interpolation: alpha = sim_accum/tick_ns ∈ [0,1).
 *                  Entity draw positions lerp between prev and current
 *                  simulated positions at alpha, giving smooth motion at
 *                  any render rate without modifying physics.
 *
 * Performance    : ncurses double-buffer: erase → draw → wnoutrefresh →
 *                  doupdate().  doupdate() sends only changed cells to the
 *                  terminal (diff), minimising write latency and flicker.
 *                  Render capped at TARGET_FPS using CLOCK_MONOTONIC sleep.
 *
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
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/*
 * All magic numbers live here. Never scatter literals through the code.
 * Change behaviour by editing this block only.
 */
enum {
    SIM_FPS_MIN      = 10,
    SIM_FPS_DEFAULT  = 60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    HUD_COLS         =  48,   /* max width of the HUD status string       */
    FPS_UPDATE_MS    = 500,   /* recalculate displayed fps every 500 ms   */

    N_COLORS         =   7,   /* number of color pairs defined in §3      */

    KEY_LEN          =  26,   /* character slots in the key display       */
};

/*
 * Printable ASCII range used for the cycling characters.
 * 0x21 '!' → 0x7E '~'  =  94 printable characters (space excluded).
 */
#define ASCII_FIRST  0x21
#define ASCII_LAST   0x7E
#define ASCII_RANGE  (ASCII_LAST - ASCII_FIRST + 1)   /* 94 */

/*
 * How fast each slot cycles: changes per second.
 * Each slot gets its own rate in [RATE_MIN, RATE_MAX] so the display
 * looks organic — not all chars flipping at the same beat.
 */
#define RATE_MIN   4.0f    /* slowest slot — 4  changes/sec  */
#define RATE_MAX  28.0f    /* fastest slot — 28 changes/sec  */

/*
 * Timing primitives.
 * TICK_NS(f) converts a frame rate (Hz) to nanoseconds per tick.
 */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/*
 * clock_ns() — monotonic wall-clock in nanoseconds.
 *
 * CLOCK_MONOTONIC never goes backwards (unlike CLOCK_REALTIME which can
 * jump on NTP adjustments).  Subtracting two consecutive clock_ns()
 * calls gives the true elapsed time regardless of system load.
 */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/*
 * clock_sleep_ns() — sleep for exactly ns nanoseconds.
 *
 * Called BEFORE render (see §8) so that the sleep budget covers only
 * physics time — not terminal I/O.  If ns ≤ 0 the frame is already
 * over-budget; skip the sleep rather than sleeping a negative amount.
 */
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * color_init() — define N_COLORS color pairs.
 *
 * Color pairs must be defined before any wattron(COLOR_PAIR(n)) call.
 * init_pair(id, fg, bg) — id 1-based (0 is reserved for default).
 *
 * 256-color path: uses xterm-256 color indices for vivid saturated colors.
 * 8-color  path: falls back to the 8 basic terminal colors.
 *
 * Pairs defined:
 *   1 → red        4 → green
 *   2 → orange     5 → cyan
 *   3 → yellow     6 → blue
 *                  7 → magenta
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(1, 196, COLOR_BLACK);   /* red     */
        init_pair(2, 208, COLOR_BLACK);   /* orange  */
        init_pair(3, 226, COLOR_BLACK);   /* yellow  */
        init_pair(4,  46, COLOR_BLACK);   /* green   */
        init_pair(5,  51, COLOR_BLACK);   /* cyan    */
        init_pair(6,  21, COLOR_BLACK);   /* blue    */
        init_pair(7, 201, COLOR_BLACK);   /* magenta */
    } else {
        init_pair(1, COLOR_RED,     COLOR_BLACK);
        init_pair(2, COLOR_RED,     COLOR_BLACK);   /* no orange in 8-color */
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(6, COLOR_BLUE,    COLOR_BLACK);
        init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell; the one aspect-ratio fix                     */
/* ===================================================================== */

/*
 * WHY TWO COORDINATE SPACES
 * ─────────────────────────
 * Terminal cells are not square. A typical cell is ~2× taller than wide
 * in physical pixels (e.g. 8 px wide × 16 px tall).
 *
 * If you store a moving object's position directly in cell coordinates
 * and move it by dx=1, dy=1 per tick, it travels twice as far
 * horizontally as vertically in physical pixels. Diagonal motion looks
 * skewed. Circles become ellipses. Angles look wrong.
 *
 * THE FIX — two spaces, one conversion point:
 *
 *   PIXEL SPACE  (physics lives here)
 *     Square grid. One unit ≈ one physical pixel.
 *     Width  = cols × CELL_W   (e.g. 200 cols × 8  = 1600 px)
 *     Height = rows × CELL_H   (e.g.  50 rows × 16 =  800 px)
 *     All positions, velocities, forces in pixel units.
 *     Speed is isotropic — 1 px/s is the same distance in X and Y.
 *
 *   CELL SPACE   (drawing happens here)
 *     Terminal columns and rows.
 *     cell_x = px_to_cell_x(pixel_x)
 *     cell_y = px_to_cell_y(pixel_y)
 *     Only scene_draw() ever calls px_to_cell_x/y.
 *     Physics code never sees cell coordinates.
 *
 * WHEN §4 IS NOT NEEDED
 * ─────────────────────
 * Simulations whose "physics grid" IS the cell grid (fire, sand, this
 * demo) can work directly in cell coordinates and skip pixel↔cell
 * conversion.  §4 is retained in this template for completeness — it
 * is the first thing you add when introducing continuous motion.
 *
 * CELL_W, CELL_H
 * ──────────────
 * Logical sub-pixel steps per terminal cell.
 * CELL_H / CELL_W must match the terminal cell aspect ratio (≈ 2.0).
 * With CELL_W=8 CELL_H=16: a 200×50 terminal → pixel space 1600×800.
 */
#define CELL_W   8
#define CELL_H  16

static inline int pw(int cols) { return cols * CELL_W; }   /* pixel width  */
static inline int ph(int rows) { return rows * CELL_H; }   /* pixel height */

/*
 * px_to_cell_x/y — convert pixel coordinate to terminal cell index.
 *
 * We use  floorf(px/CELL_W + 0.5f)  — "round half up" — not roundf().
 *
 * WHY NOT roundf:
 *   C's roundf uses "round half to even" (banker's rounding).
 *   When px/CELL_W lands exactly on 0.5, it can round to 0 on one
 *   call and to 1 on the next depending on FPU state. A slow-moving
 *   object sitting on a cell boundary oscillates every frame → flicker.
 *
 * WHY NOT truncation  (int)(px/CELL_W):
 *   Always rounds down. Creates asymmetric dwell time → staircase.
 *
 * WHY floorf(px/CELL_W + 0.5f):
 *   Adds 0.5 before flooring → "round half up".
 *   Always deterministic, breaks ties in one direction, symmetric dwell.
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  entity — KeyGen                                                    */
/* ===================================================================== */

/*
 * KeyGen — state for the key-generator animation.
 *
 * Replaces Ball / Particle / etc. from other animations.
 * The "physics" here is purely timer-driven character replacement —
 * no position, no velocity, no forces.
 *
 * Fields:
 *   slots[]       current displayed character for each key position
 *   timers[]      seconds remaining until next character change per slot
 *   rates[]       base changes-per-second for each slot (randomised once)
 *   colors[]      ncurses color pair index (1–N_COLORS) per slot
 *   speed_scale   global multiplier applied to all rates (+ / - keys)
 *   paused        when true, keygen_tick() is a no-op
 */
typedef struct {
    char  slots[KEY_LEN];
    float timers[KEY_LEN];
    float rates[KEY_LEN];
    int   colors[KEY_LEN];
    float speed_scale;
    bool  paused;
} KeyGen;

/*
 * keygen_spawn() — initialise (or re-randomise) all KEY_LEN slots.
 *
 * Each slot gets:
 *   • a random printable ASCII starting character
 *   • a random cycling rate in [RATE_MIN, RATE_MAX]
 *   • a color cycling through the 7 defined pairs
 *   • a timer seeded to 1/rate so they don't all flip at frame 1
 *
 * Called on startup and when the user presses 'r'.
 */
static void keygen_spawn(KeyGen *k)
{
    k->speed_scale = 1.0f;
    k->paused      = false;
    for (int i = 0; i < KEY_LEN; i++) {
        k->slots[i]  = (char)(ASCII_FIRST + rand() % ASCII_RANGE);
        k->rates[i]  = RATE_MIN
                     + ((float)(rand() % 10000) / 10000.0f)
                       * (RATE_MAX - RATE_MIN);
        k->timers[i] = 1.0f / k->rates[i];
        k->colors[i] = (i % N_COLORS) + 1;
    }
}

/*
 * keygen_tick() — advance the animation by one fixed timestep dt (seconds).
 *
 * Equivalent to ball_tick() in bounce_ball.c.
 * Operates only on KeyGen state; has no knowledge of screen dimensions.
 *
 * For each slot:
 *   • decrement timer by (dt × speed_scale)
 *   • when timer expires: pick new random char, reset timer
 *
 * The timer reset uses the current speed_scale so that + / - key
 * changes take effect immediately on the next expiry.
 */
static void keygen_tick(KeyGen *k, float dt)
{
    if (k->paused) return;

    float scaled_dt = dt * k->speed_scale;
    for (int i = 0; i < KEY_LEN; i++) {
        k->timers[i] -= scaled_dt;
        if (k->timers[i] <= 0.0f) {
            k->slots[i] = (char)(ASCII_FIRST + rand() % ASCII_RANGE);
            float interval = 1.0f / (k->rates[i] * k->speed_scale);
            k->timers[i] = (interval > 0.001f) ? interval : 0.001f;
        }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene — the collection of all entities for this animation.
 *
 * In bounce_ball.c this holds Ball balls[BALLS_MAX] + count + paused.
 * Here it holds a single KeyGen.  The struct exists so that scene_tick
 * and scene_draw have a stable signature regardless of what is inside.
 */
typedef struct {
    KeyGen kg;
} Scene;

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    keygen_spawn(&s->kg);
}

/*
 * scene_tick() — advance the simulation by one fixed-size step.
 *
 * Called from the accumulator loop in §8. dt is the fixed tick duration
 * in seconds (= 1 / sim_fps).
 *
 * cols, rows are passed here in case the entity needs pixel boundaries
 * (see bounce_ball.c scene_tick which calls pw/ph). KeyGen does not use
 * them — they are accepted but ignored to keep the signature uniform.
 */
static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)cols; (void)rows;   /* unused — no pixel-space physics here */
    keygen_tick(&s->kg, dt);
}

/*
 * scene_draw() — render the current scene into WINDOW *w.
 *
 * alpha ∈ [0.0, 1.0) is the render interpolation factor:
 *   alpha = sim_accum / tick_ns
 *
 * For continuous-motion entities (balls, pendulums) alpha is used to
 * extrapolate the draw position between physics ticks, eliminating
 * micro-stutter.  Example from bounce_ball.c:
 *
 *   float draw_px = b->px + b->vx * alpha * dt_sec;
 *
 * For this demo the key is stationary on screen; alpha is accepted
 * but not applied to any position. It is always part of the signature.
 *
 * Layout centered on screen:
 *
 *   row - 2  :    < GENERATING KEY >            ← label (green bold)
 *   row - 1  :    (blank)
 *   row      :    A 3 $ K 2 p ! Q 8 v ...       ← 26 cycling chars
 *   row + 1  :    - - - - - - - - - - - - -     ← dim separator
 *   row + 2  :    KEY-256  AES/RSA HYBRID        ← descriptor
 *
 * The centering arithmetic is the only place in §6 that computes
 * cell coordinates.  (No px_to_cell call needed — position is derived
 * directly from cols/rows, not from pixel physics.)
 *
 * This is the ONLY function that should call mvwaddch / mvwprintw.
 * Never draw from keygen_tick or any §5 function.
 */
static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows,
                       float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;   /* no interpolation for static entity */

    const KeyGen *k = &s->kg;

    /* Each slot is drawn with a 1-column gap between chars (2 cols/slot).
     * Total width of key row: KEY_LEN*2 - 1  (no trailing space). */
    int key_width = KEY_LEN * 2 - 1;
    int key_row   = rows / 2;
    int key_col   = (cols - key_width) / 2;
    if (key_col < 0) key_col = 0;

    /* ── label ── */
    const char *label = "< GENERATING KEY >";
    int label_len = (int)strlen(label);
    int label_col = (cols - label_len) / 2;
    if (label_col < 0) label_col = 0;

    wattron(w, COLOR_PAIR(4) | A_BOLD);
    mvwprintw(w, key_row - 2, label_col, "%s", label);
    wattroff(w, COLOR_PAIR(4) | A_BOLD);

    /* ── 26 cycling character slots ── */
    for (int i = 0; i < KEY_LEN; i++) {
        int sx = key_col + i * 2;
        if (sx < 0 || sx >= cols) continue;

        wattron(w, COLOR_PAIR(k->colors[i]) | A_BOLD);
        mvwaddch(w, key_row, sx, (chtype)(unsigned char)k->slots[i]);
        wattroff(w, COLOR_PAIR(k->colors[i]) | A_BOLD);
    }

    /* ── separator line ── */
    wattron(w, COLOR_PAIR(6) | A_DIM);
    for (int i = 0; i < key_width; i++) {
        int sx = key_col + i;
        if (sx >= 0 && sx < cols)
            mvwaddch(w, key_row + 1, sx, '-');
    }
    wattroff(w, COLOR_PAIR(6) | A_DIM);

    /* ── descriptor label ── */
    const char *desc = "KEY-256  AES/RSA HYBRID";
    int desc_len = (int)strlen(desc);
    int desc_col = (cols - desc_len) / 2;
    if (desc_col < 0) desc_col = 0;

    wattron(w, COLOR_PAIR(5) | A_DIM);
    mvwprintw(w, key_row + 2, desc_col, "%s", desc);
    wattroff(w, COLOR_PAIR(5) | A_DIM);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the ncurses display layer.
 *
 * ARCHITECTURE: ONE window (stdscr), ONE flush per frame (doupdate).
 *
 * ncurses maintains two virtual screens internally:
 *   curscr — what ncurses believes is currently on the physical terminal
 *   newscr — the target frame you are building right now
 *
 * Every mvwaddch / werase / wattron writes into newscr.
 * doupdate() diffs newscr vs curscr, sends only changed cells to the
 * terminal fd, then sets curscr = newscr.  THIS IS the double buffer —
 * it is always present, managed by ncurses, and is not optional.
 *
 * Common mistake — adding your own back/front WINDOW pair:
 *   Creating a second WINDOW and blitting it to stdscr introduces a
 *   third virtual screen that ncurses does not track.  The diff engine
 *   loses accuracy and you get ghost trails and torn frames.
 *
 * CORRECT FRAME SEQUENCE:
 *   erase()              — clear newscr (write spaces everywhere)
 *   scene_draw(…)        — write scene content into newscr
 *   mvprintw(…) HUD      — write status bar last so it is always on top
 *   wnoutrefresh(stdscr) — mark newscr ready; no terminal I/O yet
 *   doupdate()           — ONE write: send the diff to the terminal fd
 *
 * Never call refresh() (= wrefresh(stdscr) = mark + flush in one step).
 * If you have multiple windows to flush in one frame, call wnoutrefresh
 * on each and then ONE doupdate() at the end.  That way the terminal
 * never sees a partial frame.
 *
 * typeahead(-1): disables ncurses' habit of calling read() on stdin to
 * look for escape sequences mid-output.  Without it, output can be
 * interrupted and incomplete frames are sent.
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();            /* don't echo typed characters               */
    cbreak();            /* pass keys immediately, no line buffering  */
    curs_set(0);         /* hide the hardware cursor                  */
    nodelay(stdscr, TRUE);   /* getch() returns ERR immediately if no key */
    keypad(stdscr, TRUE);    /* enable function/arrow key sequences       */
    typeahead(-1);           /* never interrupt output to peek at stdin   */
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s)
{
    (void)s;
    endwin();   /* restore terminal state: show cursor, re-enable echo, etc. */
}

/*
 * screen_resize() — handle SIGWINCH (terminal resize).
 *
 * endwin() + refresh() forces ncurses to re-read LINES and COLS from
 * the kernel, resizing its internal virtual screens to match the new
 * terminal dimensions.  Without this, stdscr still thinks it is the
 * old size and mvwaddch at large (col, row) silently fails.
 */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw() — build the complete frame in stdscr (newscr).
 *
 * Order matters:
 *   1. erase()      — blank newscr so stale content becomes spaces
 *   2. scene_draw() — write animation content
 *   3. HUD          — written last so it always renders on top
 *
 * Nothing reaches the terminal until screen_present() is called.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();

    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    /* HUD — top-right corner; always drawn after scene so it is on top */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  spd:%.2fx  %s ",
             fps, sim_fps, sc->kg.speed_scale,
             sc->kg.paused ? "PAUSED " : "running");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    /* key hint — bottom-left */
    attron(COLOR_PAIR(6) | A_DIM);
    mvprintw(s->rows - 1, 0, " q:quit  spc:pause  r:reset  +/-:speed  [/]:Hz ");
    attroff(COLOR_PAIR(6) | A_DIM);
}

/*
 * screen_present() — flush newscr to the terminal (one write).
 *
 * wnoutrefresh(stdscr) copies stdscr's content into ncurses' newscr model.
 * doupdate()           diffs newscr vs curscr, sends only changed cells.
 *
 * This is the correct two-step flush. Never just call refresh().
 */
static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level application state.
 *
 * g_app is a global so that signal handlers can reach it without
 * needing a pointer argument (signal handlers have a fixed signature).
 *
 * running and need_resize are volatile sig_atomic_t because they are
 * written by signal handlers and read by the main loop.  sig_atomic_t
 * is the only integer type guaranteed to be read/written atomically
 * from a signal handler on POSIX systems.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }   /* atexit safety net */

/*
 * app_do_resize() — handle a pending SIGWINCH.
 *
 * Re-reads terminal dimensions into Screen.  For animations with
 * physics in pixel space (bounce_ball.c), this also clamps entity
 * positions so they don't escape the new smaller boundary.
 * KeyGen has no positions, so only the screen dims are updated.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app->need_resize = 0;
}

/*
 * app_handle_key() — process a single keypress.
 *
 * Returns false to signal "quit", true to continue.
 * All user-facing controls are handled here in one place.
 */
static bool app_handle_key(App *app, int ch)
{
    KeyGen *k = &app->scene.kg;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ':
        k->paused = !k->paused;
        break;

    case 'r': case 'R':
        keygen_spawn(k);
        break;

    /* Cycle speed: multiply / divide by 1.5 per keypress */
    case '=': case '+':
        k->speed_scale *= 1.5f;
        if (k->speed_scale > 16.0f) k->speed_scale = 16.0f;
        break;

    case '-':
        k->speed_scale /= 1.5f;
        if (k->speed_scale < 0.1f) k->speed_scale = 0.1f;
        break;

    /* Simulation Hz — affects fixed timestep granularity */
    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;

    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    default: break;
    }
    return true;
}

/* ─────────────────────────────────────────────────────────────────────
 * main() — the game loop
 *
 * This loop is identical in structure for every animation in the project.
 * The only things that change per-animation are scene_init/tick/draw.
 *
 * Loop body walk-through:
 *
 *   RESIZE CHECK
 *     Handle a pending SIGWINCH before touching any ncurses state.
 *     Reset frame_time and sim_accum so the accumulated dt doesn't
 *     inject a physics jump after the resize.
 *
 *   DT MEASUREMENT
 *     dt = wall-clock nanoseconds since last frame.
 *     Cap at 100 ms to prevent a physics avalanche if the process was
 *     suspended (debugger, Ctrl-Z) and then resumed.
 *
 *   SIM ACCUMULATOR (fixed timestep)
 *     sim_accum is a nanosecond "bucket".
 *     Each frame, dt is added to the bucket.
 *     While the bucket holds ≥ one tick's worth, fire one physics step
 *     and drain that tick's worth.  The remainder stays for next frame.
 *     Result: physics runs at exactly sim_fps Hz on average, regardless
 *     of render frame rate.
 *
 *   ALPHA (render interpolation)
 *     After draining, sim_accum holds the leftover time — how far we
 *     are into the NEXT tick that has not fired yet.
 *     alpha = sim_accum / tick_ns  ∈ [0, 1)
 *     Passed to scene_draw so positions can be extrapolated to "now"
 *     rather than drawn at "last tick".  Eliminates micro-stutter.
 *
 *   FPS COUNTER
 *     Counts frames over a 500 ms window.  Divide frame count by
 *     elapsed seconds → smoothed fps estimate.  Avoids per-frame
 *     division which would oscillate wildly on fast loops.
 *
 *   FRAME CAP — SLEEP BEFORE RENDER
 *     Sleep the remaining 60fps budget BEFORE terminal I/O.
 *     If slept after, the I/O time is included in "elapsed" and
 *     the loop runs full-speed on slow terminals.
 *
 *   DRAW + PRESENT
 *     erase → scene_draw → HUD → wnoutrefresh → doupdate
 *     One atomic diff write to terminal. No partial frames.
 *
 *   INPUT
 *     Non-blocking getch() after the render.  Returns ERR immediately
 *     if no key is pending (nodelay is TRUE from screen_init).
 * ───────────────────────────────────────────────────────────────────── */
int main(void)
{
    /* Seed the RNG from the monotonic clock so each run looks different */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));

    /* Register atexit handler as a safety net in case endwin() is missed */
    atexit(cleanup);

    /* SIGINT / SIGTERM — set running=0 to exit the loop gracefully      */
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);

    /* SIGWINCH — set need_resize=1; handled at top of next loop iter    */
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene);

    int64_t frame_time  = clock_ns();   /* timestamp of last frame start */
    int64_t sim_accum   = 0;            /* nanoseconds in the bucket     */
    int64_t fps_accum   = 0;            /* ns elapsed in current fps window */
    int     frame_count = 0;            /* frames in current fps window  */
    double  fps_display = 0.0;          /* smoothed fps shown in HUD     */

    while (app->running) {

        /* ── resize ──────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── dt ──────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;   /* pause guard */

        /* ── sim accumulator (fixed timestep) ────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── alpha — render interpolation factor ─────────────────── */
        /*
         * sim_accum is now the leftover ns after all full ticks.
         * alpha ∈ [0, 1) indicates how far we are into the next tick.
         * Pass to scene_draw so entities can be drawn at "now" not
         * "last tick" — eliminates visible stutter between ticks.
         *
         * For a paused scene, zero alpha for pixel-perfect freeze:
         *   float alpha = app->scene.kg.paused ? 0.0f :
         *                 (float)sim_accum / (float)tick_ns;
         */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── FPS counter (500 ms sliding window) ─────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap — sleep BEFORE render ─────────────────────── */
        /*
         * elapsed = time spent on physics since frame_time was updated.
         * Budget  = NS_PER_SEC / 60  (one 60fps frame in ns).
         * Sleep   = budget − elapsed.
         *
         * Sleeping BEFORE render means only physics time is charged
         * against the budget.  Terminal I/O (doupdate, getch) happens
         * after the sleep and does not affect the next frame's timing.
         */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── draw + present ──────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
                    alpha, dt_sec);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
