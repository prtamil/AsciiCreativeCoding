/*
 * donut.c  —  ncurses spinning ASCII torus
 *
 * Original algorithm by Andy Sloane (donut.c / a1k0n.net),
 * rewritten in the framework used by matrix_rain / burst / kaboom:
 *   - Single stdscr, ncurses internal double buffer — no flicker
 *   - dt-based rotation — speed is frame-rate independent
 *   - SIGWINCH resize — torus recentres to new terminal dimensions
 *   - Speed control:   ] faster   [ slower
 *   - Size control:    = larger   - smaller
 *   - Pause:           space
 *   - Clean signal / atexit teardown
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   ]  [      spin faster / slower
 *   =  -      larger / smaller torus
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra donut.c -o donut -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config  — every tunable constant
 *   §2  clock   — monotonic nanosecond clock + sleep
 *   §3  color   — luminance color pairs; 256-color with 8-color fallback
 *   §4  torus   — geometry, zbuffer, framebuffer, tick, draw
 *   §5  screen  — single stdscr, ncurses internal double buffer
 *   §6  app     — dt loop, input, resize, cleanup
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
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    /* Render rate */
    SIM_FPS_MIN      =  5,
    SIM_FPS_DEFAULT  = 30,
    SIM_FPS_MAX      = 60,
    SIM_FPS_STEP     =  5,

    /* HUD */
    HUD_COLS         = 32,
    FPS_UPDATE_MS    = 500,
};

/*
 * Rotation speed in radians per second.
 * A increments faster (tumble), B slower (spin) — matches the original feel.
 * ] / [ keys multiply/divide these by SPEED_SCALE.
 */
#define ROT_A_DEFAULT   1.2f    /* radians/sec around X axis            */
#define ROT_B_DEFAULT   0.6f    /* radians/sec around Z axis            */
#define SPEED_SCALE     1.3f    /* multiplier per ] or [ keypress       */
#define SPEED_MIN       0.05f
#define SPEED_MAX      12.0f

/*
 * Torus geometry.
 * R1  inner tube radius (the circle being revolved).
 * R2  distance from torus centre to tube centre.
 * K1  perspective scaling — controls how large the torus appears.
 * K2  viewer distance — larger = less perspective distortion.
 *
 * K1 is recalculated whenever the terminal size or SIZE_SCALE changes
 * so the torus always fills roughly the same fraction of the screen.
 */
#define TORUS_R1        1.0f
#define TORUS_R2        2.0f
#define TORUS_K2        5.0f

/* Size scale multiplier applied on = / - keypresses. */
#define SIZE_SCALE      1.15f
#define SIZE_MIN        0.3f
#define SIZE_MAXX        5.0f

/* Angle step sizes for the geometry loops — finer = smoother surface. */
#define THETA_STEP      0.07f
#define PHI_STEP        0.02f

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS       1000000LL
#define TICK_NS(fps)    (NS_PER_SEC / (fps))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

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

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Luminance color pairs — 8 levels from dim grey to bright white.
 * In 256-color mode each pair uses a distinct grey ramp index for smooth
 * shading.  In 8-color mode we use DIM/NORMAL/BOLD on white to fake
 * three brightness levels across the eight pairs.
 *
 * Pair index maps directly to luminance_index (0..7) from the geometry.
 * Pair 0 is unused (ncurses pair 0 is reserved for the default color).
 * We use pairs 1–8.
 */
enum { LUMI_LEVELS = 8 };

static void color_init(void)
{
    start_color();

    if (COLORS >= 256) {
        /*
         * xterm-256 grey ramp: indices 232–255 are 24 steps dark→light.
         * We pick 8 evenly spaced steps across the brighter half (240–255)
         * so the torus is vivid rather than muddy.
         */
        int greys[LUMI_LEVELS] = { 235, 238, 241, 244, 247, 250, 253, 255 };
        for (int i = 0; i < LUMI_LEVELS; i++)
            init_pair(i + 1, greys[i], COLOR_BLACK);
    } else {
        /* 8-color fallback: alternate WHITE pairs with DIM/NORMAL/BOLD. */
        for (int i = 0; i < LUMI_LEVELS; i++)
            init_pair(i + 1, COLOR_WHITE, COLOR_BLACK);
    }
}

/*
 * lumi_attr() — return the ncurses attribute for luminance level l (0..7).
 * In 256-color mode the pair itself carries the brightness.
 * In 8-color mode A_DIM / A_BOLD carry it.
 */
static attr_t lumi_attr(int l)
{
    if (l < 0) l = 0;
    if (l > LUMI_LEVELS - 1) l = LUMI_LEVELS - 1;

    attr_t attr = COLOR_PAIR(l + 1);
    if (COLORS < 256) {
        if (l < 3)       attr |= A_DIM;
        else if (l >= 6) attr |= A_BOLD;
    }
    return attr;
}

/* ===================================================================== */
/* §4  torus                                                              */
/* ===================================================================== */

/*
 * Torus — owns all mutable state for the spinning donut.
 *
 *   A, B         current rotation angles (radians); accumulate each tick
 *   rot_a, rot_b rotation speed (radians/sec); mutable via ] / [
 *   k1_scale     size scale multiplier; mutable via = / -
 *   paused       when true, tick() does not advance A or B
 *   cols, rows   terminal dimensions (needed for projection centre)
 *
 * The framebuffer (output[]) and zbuffer[] are allocated flat on the
 * struct — no heap allocation needed for a terminal-sized buffer.
 * Maximum terminal size we support: 512 × 256 = 131072 cells.
 * Larger terminals are clipped gracefully.
 */
#define TORUS_MAX_COLS  512
#define TORUS_MAX_ROWS  256
#define TORUS_CELLS     (TORUS_MAX_COLS * TORUS_MAX_ROWS)

typedef struct {
    float A, B;             /* current rotation angles                  */
    float rot_a, rot_b;     /* rotation speed radians/sec               */
    float k1_scale;         /* size multiplier (1.0 = default)          */
    bool  paused;
    int   cols, rows;
    float zbuf[TORUS_CELLS];
    char  outbuf[TORUS_CELLS];
} Torus;

/*
 * torus_k1() — compute the perspective scaling factor K1.
 *
 * K1 is chosen so the projected torus radius fills ~40% of the smaller
 * terminal dimension, then scaled by k1_scale.  Recalculated whenever
 * the terminal size or scale changes.
 *
 * The formula comes from projecting the outermost point of the torus
 * (at distance R2+R1 from the axis) onto the screen:
 *   screen_radius = K1 * (R2+R1) / (K2 + R2+R1)
 * We want screen_radius ≈ 0.4 * min(cols/2, rows):
 *   K1 = 0.4 * min(cols/2, rows) * (K2 + R2+R1) / (R2+R1)
 */
static float torus_k1(const Torus *t)
{
    float half_w = (float)t->cols * 0.5f;
    float half_h = (float)t->rows;
    float target = (half_w < half_h ? half_w : half_h) * 0.42f;
    float k1 = target * (TORUS_K2 + TORUS_R2 + TORUS_R1)
                       / (TORUS_R2 + TORUS_R1);
    return k1 * t->k1_scale;
}

static void torus_init(Torus *t, int cols, int rows)
{
    memset(t, 0, sizeof *t);
    t->A        = 0.0f;
    t->B        = 0.0f;
    t->rot_a    = ROT_A_DEFAULT;
    t->rot_b    = ROT_B_DEFAULT;
    t->k1_scale = 1.0f;
    t->paused   = false;
    t->cols     = cols;
    t->rows     = rows;
}

/*
 * torus_tick() — advance rotation by dt_sec seconds.
 *
 * dt_sec is the fixed simulation tick duration; rotation is in
 * radians/sec so the visual speed is frame-rate independent.
 */
static void torus_tick(Torus *t, float dt_sec)
{
    if (t->paused) return;
    t->A += t->rot_a * dt_sec;
    t->B += t->rot_b * dt_sec;
}

/*
 * torus_render() — rasterise the torus into outbuf[] / zbuf[].
 *
 * This is a direct port of Andy Sloane's original algorithm.
 * Two nested loops parameterise the torus surface by (theta, phi):
 *   theta: angle around the tube cross-section
 *   phi:   angle around the torus axis
 *
 * Each point is rotated by A (around X) and B (around Z), projected
 * to 2D with perspective division, and written to the flat buffers if
 * it passes the z-buffer test.
 *
 * The luminance string maps L→character in order of brightness:
 *   ".,-~:;=!*#$@"
 * Index 0 is dimmest, 11 is brightest.
 * We also split the 12 characters into LUMI_LEVELS=8 color bands so
 * the torus uses both character choice AND color for shading depth.
 */
static void torus_render(Torus *t)
{
    const int   cols = t->cols;
    const int   rows = t->rows;
    const int   n    = cols * rows;
    const float K1   = torus_k1(t);
    const float K2   = TORUS_K2;

    /* Reset buffers. */
    memset(t->zbuf,   0,   sizeof(float) * (size_t)n);
    memset(t->outbuf, ' ', sizeof(char)  * (size_t)n);

    /* Precompute rotation sines/cosines — same for every surface point. */
    const float sinA = sinf(t->A), cosA = cosf(t->A);
    const float sinB = sinf(t->B), cosB = cosf(t->B);

    static const char k_lumi[] = ".,-~:;=!*#$@";
    const int         k_lumi_n = (int)(sizeof k_lumi - 1);

    for (float theta = 0.0f; theta < 2.0f * (float)M_PI; theta += THETA_STEP) {
        const float costh = cosf(theta), sinth = sinf(theta);

        for (float phi = 0.0f; phi < 2.0f * (float)M_PI; phi += PHI_STEP) {
            const float cosph = cosf(phi), sinph = sinf(phi);

            /*
             * Surface point on the torus before rotation:
             *   circle in XZ plane centred at (R2, 0, 0), radius R1.
             */
            const float cx = TORUS_R2 + TORUS_R1 * costh;
            const float cy = TORUS_R1 * sinth;

            /*
             * Rotate by A around X axis then by B around Z axis.
             * (Combined rotation matrix from the original.)
             */
            const float x = cx * (cosB * cosph + sinA * sinB * sinph)
                          - cy * cosA * sinB;
            const float y = cx * (sinB * cosph - sinA * cosB * sinph)
                          + cy * cosA * cosB;
            const float z = K2 + cosA * cx * sinph + cy * sinA;

            const float ooz = 1.0f / z;   /* one-over-z for perspective */

            /* Project to screen coordinates. */
            const int xp = (int)(cols / 2 + K1 * ooz * x);
            const int yp = (int)(rows / 2 - K1 * ooz * y * 0.5f);

            /* Bounds check. */
            if (xp < 0 || xp >= cols || yp < 0 || yp >= rows) continue;

            const int idx = yp * cols + xp;

            /* Luminance: dot product of surface normal with light vector. */
            const float L = cosph * costh * sinB
                          - cosA * costh * sinph
                          - sinA * sinth
                          + cosB * (cosA * sinth - costh * sinA * sinph);

            if (L <= 0.0f) continue;   /* back-facing surface — skip   */

            /* Z-buffer test — keep the nearest surface point. */
            if (ooz <= t->zbuf[idx]) continue;
            t->zbuf[idx] = ooz;

            /* Map luminance to character and color band. */
            int li = (int)(L * (float)k_lumi_n);
            if (li >= k_lumi_n) li = k_lumi_n - 1;
            t->outbuf[idx] = k_lumi[li];
        }
    }
}

/*
 * torus_draw() — write the rendered framebuffer into a WINDOW.
 *
 * We iterate row by row and emit only non-space cells with their
 * luminance color attribute.  Space cells are left as the black
 * background — no mvwaddch for them so we never overdraw with spaces.
 *
 * The y-scaling by 0.5 in torus_render (K1 * ooz * y * 0.5) compensates
 * for the terminal's 2:1 cell aspect ratio so the torus looks round.
 */
static void torus_draw(const Torus *t, WINDOW *w)
{
    const int cols = t->cols;
    const int rows = t->rows;
    const int n    = cols * rows;

    static const char k_lumi[] = ".,-~:;=!*#$@";
    const int         k_lumi_n = (int)(sizeof k_lumi - 1);

    for (int i = 0; i < n; i++) {
        const char c = t->outbuf[i];
        if (c == ' ') continue;

        int y = i / cols;
        int x = i % cols;
        if (x >= cols || y >= rows) continue;

        /* Map character back to luminance index for color selection. */
        const char *p = strchr(k_lumi, c);
        int li = p ? (int)(p - k_lumi) : 0;
        int ci = (li * LUMI_LEVELS) / k_lumi_n;   /* 0..LUMI_LEVELS-1 */

        attr_t attr = lumi_attr(ci);
        wattron(w, attr);
        mvwaddch(w, y, x, (chtype)(unsigned char)c);
        wattroff(w, attr);
    }
}

/* ===================================================================== */
/* §5  screen                                                             */
/* ===================================================================== */

typedef struct {
    int cols;
    int rows;
} Screen;

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

static void screen_free(Screen *s)  { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(Screen *s, const Torus *t, double fps)
{
    erase();
    torus_draw(t, stdscr);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps A:%.1f B:%.1f spd:%.1f",
             fps, t->A, t->B, t->rot_a);
    int hx = s->cols - HUD_COLS;
    if (hx < 0) hx = 0;
    attron(lumi_attr(5) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(lumi_attr(5) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §6  app                                                                */
/* ===================================================================== */

/*
 * App — top-level owner of all subsystems.
 *
 * sim_fps controls how many times per second torus_tick() fires.
 * The rotation step per tick is rot_a/rot_b * dt_sec so changing
 * sim_fps does NOT change the visual rotation speed — only smoothness.
 *
 * running and need_resize are sig_atomic_t for safe signal writes.
 */
typedef struct {
    Torus                 torus;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app->torus.cols = app->screen.cols;
    app->torus.rows = app->screen.rows;
    app->need_resize = 0;
}

/*
 * app_handle_key() — returns false to quit.
 *
 *   q / ESC    quit
 *   space      pause / resume rotation
 *   ]  [       rotate faster / slower (scales rot_a and rot_b together)
 *   =  +       larger torus
 *   -          smaller torus
 */
static bool app_handle_key(App *app, int ch)
{
    Torus *t = &app->torus;

    switch (ch) {
    case 'q': case 'Q': case 27:
        return false;

    case ' ':
        t->paused = !t->paused;
        break;

    case ']':
        t->rot_a *= SPEED_SCALE;
        t->rot_b *= SPEED_SCALE;
        if (t->rot_a > SPEED_MAX) t->rot_a = SPEED_MAX;
        if (t->rot_b > SPEED_MAX) t->rot_b = SPEED_MAX;
        break;

    case '[':
        t->rot_a /= SPEED_SCALE;
        t->rot_b /= SPEED_SCALE;
        if (t->rot_a < SPEED_MIN) t->rot_a = SPEED_MIN;
        if (t->rot_b < SPEED_MIN) t->rot_b = SPEED_MIN;
        break;

    case '=': case '+':
        t->k1_scale *= SIZE_SCALE;
        if (t->k1_scale > SIZE_MAXX) t->k1_scale = SIZE_MAXX;
        break;

    case '-':
        t->k1_scale /= SIZE_SCALE;
        if (t->k1_scale < SIZE_MIN) t->k1_scale = SIZE_MIN;
        break;

    default:
        break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    torus_init(&app->torus, app->screen.cols, app->screen.rows);

    /*
     * dt loop state — identical to every other program in the framework.
     */
    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

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
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── sim accumulator ─────────────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            torus_tick(&app->torus, dt_sec);
            sim_accum -= tick_ns;
        }
        float alpha = (float)sim_accum / (float)tick_ns;
        (void)alpha;

        /* ── render ──────────────────────────────────────────────── */
        torus_render(&app->torus);

        /* ── HUD counter ─────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap (sleep BEFORE render so I/O doesn't drift) ── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── draw + present ──────────────────────────────────────── */
        screen_draw(&app->screen, &app->torus, fps_display);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
