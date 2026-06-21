/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * galaxy.c — a spiral galaxy you can watch wind up.
 *
 * Each star just glides around a circle at a fixed speed; the spiral shape and
 * the slow winding-up come for free from where the stars start and the fact
 * that inner stars circle faster than outer ones. No gravity is simulated.
 * Refs: Binney & Tremaine, "Galactic Dynamics"; Rubin & Ford, ApJ 1970.
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

/* ── §1 config — tunable constants ── */

#define ROWS_MAX      128
#define COLS_MAX      512
#define N_STARS       3000   /* total stars */
#define ARMS_DEF      2      /* starting arm count */
#define ARMS_MIN      2
#define ARMS_MAX      4
#define N_THEMES      5

#define STEPS_DEF     2      /* physics steps per rendered frame */
#define STEPS_MIN     1
#define STEPS_MAX    16
#define SIM_FPS_DEF  20
#define SIM_FPS_MIN   5
#define SIM_FPS_MAX  60
#define SIM_FPS_STEP  5

/* Every star moves at this same speed no matter how far out it is; that is what
 * makes inner stars circle faster and the arms wind up. Units: radius/step. */
#define V0_DEF        0.006f

/* How fast a cell's brightness fades each frame: lower = shorter trails. */
#define DECAY         0.82f

/* How tightly the arms coil; higher = more turns. */
#define WINDING       1.0f

/* Random angular spread that gives each arm some thickness (radians, ± half). */
#define ARM_SCATTER   0.25f

#define SPEED_DEF     1.0f
#define SPEED_MIN     0.1f
#define SPEED_MAX     5.0f
#define SPEED_STEP    0.1f

#define NS_PER_SEC    1000000000LL
#define TICK_NS(f)    (NS_PER_SEC / (f))

static const float PI = 3.14159265358979f;

/* ── §2 clock — read time and sleep ── */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = { .tv_sec  = (time_t)(ns / NS_PER_SEC),
                            .tv_nsec = (long)(ns % NS_PER_SEC) };
    nanosleep(&req, NULL);
}

/* ── §3 data — types, themes, and the Galaxy state ── */

/* Colour-pair ids. ncurses numbers pairs starting at 1 (0 is reserved). Colour
 * here marks WHICH ZONE of the galaxy a cell sits in; the glyph carries how
 * bright it is. The radius splits are in scene_draw (core < 10%, disk to 65%).
 *   CP_CORE — central bulge
 *   CP_DISK — spiral arms and disc
 *   CP_HALO — faint outer halo
 *   CP_HUD / CP_HINT — the two status bars (yellow data row, cyan key row). */
enum { CP_CORE=1, CP_DISK, CP_HALO, CP_HUD, CP_HINT };

/* Theme — one named colour scheme: a colour for each of the three galaxy zones.
 * Two copies of each colour are stored so we can pick the rich 256-colour value
 * on capable terminals and fall back to a basic colour otherwise. Themes are
 * just data, so t/T swaps the whole look with no code change. All on black (deep
 * space). The currently chosen index lives in Galaxy.theme.
 *   core256/disk256/halo256 : 256-colour value per zone (bulge / arms / halo).
 *   core8/disk8/halo8       : plain 8-colour fallback for the same three zones.
 *   name                    : label shown in the status bar. */
typedef struct {
    short core256, disk256, halo256;
    short core8,   disk8,   halo8;
    const char *name;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* 0 MilkyWay  — white core, cyan arms, grey halo */
    { 231,  39, 240,  COLOR_WHITE, COLOR_CYAN,    COLOR_WHITE,   "MilkyWay" },
    /* 1 Starburst — yellow core, blue arms, dark grey halo */
    { 226,  33, 244,  COLOR_YELLOW, COLOR_BLUE,   COLOR_WHITE,   "Starburst" },
    /* 2 Nebula    — white core, pink arms, purple halo */
    { 231, 207,  92,  COLOR_WHITE,  COLOR_MAGENTA, COLOR_MAGENTA, "Nebula" },
    /* 3 Infrared  — white core, red arms, dark-red halo */
    { 231, 196,  52,  COLOR_WHITE,  COLOR_RED,     COLOR_RED,     "Infrared" },
    /* 4 Aurora    — white core, bright-green arms, dark-green halo */
    { 231,  46,  28,  COLOR_WHITE,  COLOR_GREEN,   COLOR_GREEN,   "Aurora" },
};

static bool g_has_256;   /* true if the terminal supports 256 colours; set once at startup. */

/* Star — one star riding a fixed circle around the centre. It never feels
 * gravity; it just sweeps around at a steady angular rate. Because every star
 * shares the same orbital speed, a star close in turns faster than one far out,
 * and that difference is what slowly winds the arms into a tighter spiral. The
 * spiral pattern isn't computed from forces — it falls out of where each star
 * starts (galaxy_init) plus this faster-when-closer behaviour. The turn rate is
 * fixed at birth; only the angle changes each tick. Refs: Binney & Tremaine;
 * Rubin & Ford, ApJ 1970.
 *   r     : how far out the star orbits, 0..~1 where 1.0 is the screen edge.
 *   theta : current angle around the circle, in radians; the only field a tick
 *           changes.
 *   omega : how fast the angle grows per step (radians/step); set once at birth
 *           to V0/r so smaller r spins faster. */
typedef struct {
    float r;
    float theta;
    float omega;
} Star;

/* Galaxy — every piece of the running simulation in one place. Keeping it
 * together means the main functions can pass a single pointer around while small
 * helpers take only the one value they need.
 *   stars   : the pool of orbiting stars (filled by galaxy_init).
 *   bright  : a brightness value per screen cell. Stars add to it as they pass
 *             over a cell and it fades a little each frame, which is what draws
 *             the glowing trails.
 *   rows,cols : size of the drawable area in cells, set from the terminal and
 *               capped at ROWS_MAX/COLS_MAX.
 *   narms   : number of spiral arms, ARMS_MIN..ARMS_MAX.
 *   v0      : the shared orbital speed (see V0_DEF), radius/step.
 *   speed   : extra multiplier on orbit speed, SPEED_MIN..SPEED_MAX.
 *   steps   : how many physics sub-steps to run per drawn frame,
 *             STEPS_MIN..STEPS_MAX.
 *   sim_fps : how many simulation ticks per second, SIM_FPS_MIN..SIM_FPS_MAX.
 *   paused  : when true the simulation freezes but drawing keeps going.
 *   theme   : which colour scheme in k_themes[] is active, 0..N_THEMES-1; kept
 *             here so a resize doesn't lose the choice. */
typedef struct {
    Star  stars[N_STARS];
    float bright[ROWS_MAX][COLS_MAX];
    int   rows, cols;
    int   narms;
    float v0;
    float speed;
    int   steps;
    int   sim_fps;
    bool  paused;
    int   theme;
} Galaxy;

static Galaxy g_galaxy = {
    .narms   = ARMS_DEF,
    .v0      = V0_DEF,
    .speed   = SPEED_DEF,
    .steps   = STEPS_DEF,
    .sim_fps = SIM_FPS_DEF,
    /* bright/stars/rows/cols/paused/theme → zero-init */
};

/* Random-number state; seeded in main, advanced only by the §4 helpers. */
static uint32_t g_rng = 12345u;

/* ── §4 logic — random numbers and sampling ── */

static inline uint32_t rng_next(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng <<  5;
    return g_rng;
}
static inline float rng_float(void)
{
    return (float)(rng_next() >> 8) / (float)(1u << 24);
}

/* One sample from a bell-curve distribution (most values near 0, few far out);
 * the standard Box-Muller trick. */
static float rng_gauss(void)
{
    float u1 = rng_float() + 1e-6f;
    float u2 = rng_float();
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI * u2);
}

/* ── §5 sim — advance every orbit one step and light up its trail ── */

/* The only place the simulation moves forward; main calls it g->steps times per
 * tick. */
static void galaxy_step(Galaxy *g)
{
    int   cx = g->cols / 2, cy = g->rows / 2;
    /* Squash vertically: terminal cells are about twice as tall as wide, so we
     * scale y by half to keep the galaxy looking round, not stretched. */
    float rx = g->cols * 0.44f;
    float ry = rx * 0.50f;

    /* Split each star's brightness across the sub-steps so the total glow it
     * adds per frame stays the same no matter how many sub-steps we run. */
    float w = 1.0f / (float)g->steps;

    for (int i = 0; i < N_STARS; i++) {
        g->stars[i].theta += g->stars[i].omega * g->speed;

        /* Turn angle+radius into an x,y point, then into a screen cell. */
        float x = g->stars[i].r * cosf(g->stars[i].theta);
        float y = g->stars[i].r * sinf(g->stars[i].theta);
        int   sx = cx + (int)(x * rx + 0.5f);
        int   sy = cy + (int)(y * ry + 0.5f);

        if (sx >= 0 && sx < g->cols && sy >= 0 && sy < g->rows)
            g->bright[sy][sx] += w;
    }
}

/* ── §6 init — place all the stars from scratch ── */

/* Rebuilds the whole star field; called at startup and on reset, arm change, or
 * resize. The spiral look is baked in entirely here, by where stars start. */
static void galaxy_init(Galaxy *g, int narms)
{
    g->narms = narms;
    memset(g->bright, 0, sizeof g->bright);

    int n = 0;

    /* Bulge — about a fifth of the stars, packed tight near the centre to make
     * the bright nucleus. Radii come from a bell curve folded positive so they
     * cluster near zero. */
    int n_bulge = N_STARS / 5;
    for (int i = 0; i < n_bulge; i++) {
        float r = fabsf(rng_gauss() * 0.07f);
        if (r < 0.004f) r = 0.004f;
        if (r > 0.20f)  r = 0.20f;
        g->stars[n++] = (Star){ r, rng_float() * 2.0f * PI, g->v0 / r };
    }

    /* Arms — most of the stars, laid out along the spiral arms. The further out
     * a star sits, the more its starting angle is twisted, which is what bends
     * each arm into a coil; a touch of noise gives the arm width. Stars are
     * dealt out to arms in turn (i % narms). */
    int n_arm_total = N_STARS * 7 / 10;
    for (int i = 0; i < n_arm_total; i++) {
        int   arm   = i % narms;
        float a_off = arm * (2.0f * PI / (float)narms);
        float r     = 0.08f + rng_float() * 0.87f;   /* 0.08 .. 0.95 */
        float theta = a_off + WINDING * logf(r / 0.08f)
                      + (rng_float() - 0.5f) * (2.0f * ARM_SCATTER);
        g->stars[n++] = (Star){ r, theta, g->v0 / r };
    }

    /* Halo — the rest, sprinkled at random far out to suggest stray field stars
     * and the faint outer disc. */
    while (n < N_STARS) {
        float r = 0.35f + rng_float() * 0.70f;
        if (r > 1.05f) r = 1.05f;
        g->stars[n++] = (Star){ r, rng_float() * 2.0f * PI, g->v0 / r };
    }
}

/* ── §7 render — load colours, then draw the galaxy and HUD ── */

static void theme_apply(int ti)
{
    const Theme *t = &k_themes[ti];
    if (g_has_256) {
        init_pair(CP_CORE, t->core256, COLOR_BLACK);
        init_pair(CP_DISK, t->disk256, COLOR_BLACK);
        init_pair(CP_HALO, t->halo256, COLOR_BLACK);
    } else {
        init_pair(CP_CORE, t->core8, COLOR_BLACK);
        init_pair(CP_DISK, t->disk8, COLOR_BLACK);
        init_pair(CP_HALO, t->halo8, COLOR_BLACK);
    }
    init_pair(CP_HUD,  g_has_256 ? 226 : COLOR_YELLOW, -1);  /* bright yellow — top data bar    */
    init_pair(CP_HINT, g_has_256 ?  51 : COLOR_CYAN,   -1);  /* bright cyan   — bottom action bar */
}

static void scene_draw(Galaxy *g)
{
    /* Fade every cell a little. Old star passes dim out over a few frames, which
     * is what makes the trails. This is the one spot where drawing changes
     * state. */
    for (int r = 0; r < g->rows; r++)
        for (int c = 0; c < g->cols; c++)
            g->bright[r][c] *= DECAY;

    erase();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    int draw_rows = (g->rows < rows - 2) ? g->rows : rows - 2;
    int draw_cols = (g->cols < cols)     ? g->cols : cols;

    /* Find the brightest cell so we can scale everything else against it; this
     * keeps the picture looking right at any star count or speed. */
    float b_max = 0.1f;
    for (int r = 0; r < draw_rows; r++)
        for (int c = 0; c < draw_cols; c++)
            if (g->bright[r][c] > b_max) b_max = g->bright[r][c];

    int   cx = g->cols / 2, cy = g->rows / 2;
    float rx = g->cols * 0.44f, ry = rx * 0.50f;
    float r_core = 0.10f;    /* inside this radius → core colour */
    float r_disk = 0.65f;    /* out to here → disk colour; beyond → halo */

    for (int r = 0; r < draw_rows; r++) {
        for (int c = 0; c < draw_cols; c++) {
            float b = g->bright[r][c];
            if (b < 0.02f * b_max) continue;   /* too faint to bother drawing */

            /* Pick a glyph by brightness: faint dots up to a solid blob. */
            float t = b / b_max;
            char  ch;
            int   bold = 0;
            if      (t < 0.12f) ch = '.';
            else if (t < 0.25f) ch = ',';
            else if (t < 0.40f) ch = ':';
            else if (t < 0.55f) ch = 'o';
            else if (t < 0.70f) ch = 'O';
            else if (t < 0.85f) { ch = '0'; bold = A_BOLD; }
            else                { ch = '@'; bold = A_BOLD; }

            /* Pick a colour by how far the cell sits from the centre. */
            float dx = (float)(c - cx) / rx;
            float dy = (float)(r - cy) / ry;
            float rn = sqrtf(dx*dx + dy*dy);
            int cp = (rn < r_core) ? CP_CORE
                   : (rn < r_disk) ? CP_DISK
                                   : CP_HALO;

            attron(COLOR_PAIR(cp) | bold);
            mvaddch(r + 1, c, (chtype)ch);   /* +1 because row 0 is the HUD bar */
            attroff(COLOR_PAIR(cp) | bold);
        }
    }

    /* Always mark the galactic centre, even if no star happens to sit there. */
    if (cy < draw_rows && cx < draw_cols) {
        attron(COLOR_PAIR(CP_CORE) | A_BOLD);
        mvaddch(cy + 1, cx, '*');            /* +1 because row 0 is the HUD bar */
        attroff(COLOR_PAIR(CP_CORE) | A_BOLD);
    }

    /* Two status bars: title plus live stats up top, key reminders along the
     * bottom. Each line is filled, then clipped with "%.*s" so a narrow window
     * can't overflow or wrap. */
    char left[20], right[80];
    snprintf(left,  sizeof left,  " SPIRAL GALAXY ");
    snprintf(right, sizeof right,
             " arms:%d  speed:%.1fx  theme:%s  fps:%d  steps:%d  %s ",
             g->narms, g->speed, k_themes[g->theme].name,
             g->sim_fps, g->steps, g->paused ? "PAUSED" : "running");
    int stat_x = cols - (int)strlen(right);      /* column where the stats begin */
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    for (int c = 0; c < cols; c++) mvaddch(0, c, ' ');
    if (stat_x >= 0) {
        mvprintw(0, 0,      "%.*s", stat_x, left);  /* clip title so it stops before the stats */
        mvprintw(0, stat_x, "%s", right);
    } else {
        mvprintw(0, 0,  "%.*s", cols, right);    /* window too narrow: show stats only */
    }
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    for (int c = 0; c < cols; c++) mvaddch(rows - 1, c, ' ');
    mvprintw(rows - 1, 0, "%.*s", cols,
             " q:quit  p:pause  r:reset  a/A:arms  t/T:theme  +/-:speed  [/]:fps  </>:steps ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §8 events — signal flags and terminal resize ── */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit   = 0;

static void handle_sigwinch(int s) { (void)s; g_resize = 1; }
static void handle_sigterm (int s) { (void)s; g_quit   = 1; }

static void screen_resize(Galaxy *g)
{
    endwin(); refresh();
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g->rows = (rows - 2 < ROWS_MAX) ? rows - 2 : ROWS_MAX;
    g->cols = (cols      < COLS_MAX) ? cols      : COLS_MAX;
    g_resize = 0;
}

/* ── §9 screen — start ncurses and load the palette ── */

static void screen_init(int theme)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    start_color();
    use_default_colors();
    g_has_256 = (COLORS >= 256);
    theme_apply(theme);
}

/* ── §10 app — signals, the frame loop, and key handling ── */

int main(void)
{
    signal(SIGWINCH, handle_sigwinch);
    signal(SIGTERM,  handle_sigterm);
    signal(SIGINT,   handle_sigterm);

    g_rng = (uint32_t)time(NULL) ^ 0xC0DE4A1Au;
    screen_init(g_galaxy.theme);

    { int rows, cols; getmaxyx(stdscr, rows, cols);
      g_galaxy.rows = (rows - 2 < ROWS_MAX) ? rows - 2 : ROWS_MAX;
      g_galaxy.cols = (cols      < COLS_MAX) ? cols      : COLS_MAX; }

    galaxy_init(&g_galaxy, g_galaxy.narms);
    int64_t next_tick = clock_ns();

    while (!g_quit) {

        /* ── input ── */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 27: g_quit = 1; break;
            case 'p': case ' ': g_galaxy.paused = !g_galaxy.paused; break;
            case 'r': galaxy_init(&g_galaxy, g_galaxy.narms); break;

            case 'a':
                g_galaxy.narms = (g_galaxy.narms < ARMS_MAX) ? g_galaxy.narms + 1 : ARMS_MIN;
                galaxy_init(&g_galaxy, g_galaxy.narms);
                break;
            case 'A':
                g_galaxy.narms = (g_galaxy.narms > ARMS_MIN) ? g_galaxy.narms - 1 : ARMS_MAX;
                galaxy_init(&g_galaxy, g_galaxy.narms);
                break;

            case 't':
                g_galaxy.theme = (g_galaxy.theme + 1) % N_THEMES;
                theme_apply(g_galaxy.theme);
                break;
            case 'T':
                g_galaxy.theme = (g_galaxy.theme + N_THEMES - 1) % N_THEMES;
                theme_apply(g_galaxy.theme);
                break;

            case '+': case '=':
                if (g_galaxy.speed < SPEED_MAX - 0.01f) g_galaxy.speed += SPEED_STEP;
                break;
            case '-':
                if (g_galaxy.speed > SPEED_MIN + 0.01f) g_galaxy.speed -= SPEED_STEP;
                break;

            case ']':
                if (g_galaxy.sim_fps < SIM_FPS_MAX) g_galaxy.sim_fps += SIM_FPS_STEP;
                break;
            case '[':
                if (g_galaxy.sim_fps > SIM_FPS_MIN) g_galaxy.sim_fps -= SIM_FPS_STEP;
                break;

            case '>':
                if (g_galaxy.steps < STEPS_MAX) g_galaxy.steps++;
                break;
            case '<':
                if (g_galaxy.steps > STEPS_MIN) g_galaxy.steps--;
                break;
            }
        }

        /* ── resize ── */
        if (g_resize) { screen_resize(&g_galaxy); galaxy_init(&g_galaxy, g_galaxy.narms); }

        /* ── simulate ── */
        int64_t now = clock_ns();
        if (!g_galaxy.paused && now >= next_tick) {
            for (int s = 0; s < g_galaxy.steps; s++) galaxy_step(&g_galaxy);
            next_tick = now + TICK_NS(g_galaxy.sim_fps);
        }

        /* ── render ── */
        scene_draw(&g_galaxy);
        wnoutrefresh(stdscr);
        doupdate();

        /* ── sleep until next frame ── */
        clock_sleep_ns(next_tick - clock_ns() - 1000000LL);
    }

    endwin();
    return 0;
}
