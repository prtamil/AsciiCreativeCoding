/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * aurora.c — aurora borealis seen from the side: glowing curtains hanging in
 * the sky over a dark land silhouette, under a field of stars.
 *
 * No particles, no grid of state. Every cell is computed fresh from just the
 * column, the row, the clock, and a random seed. Refs in §1 (colour themes
 * follow real aurora emission lines) and on the Aurora struct (§4).
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

/* ── §1 config — constants, colour themes, composition fractions ── */

enum {
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    = 10,
    FPS_UPDATE_MS   = 500,
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Where things sit on screen. Top to bottom the rows run: starry sky, the
 * aurora curtains (bright green at the bottom edge fading up to purple tips), a
 * gap of sky, then the dark land along the bottom. Every *_FRAC below is a
 * fraction of the screen height, with 0 at the top.
 */
#define AUR_MAX_COLS      1024  /* most columns we precompute ridge/hem for */
#define HORIZON_FRAC      0.82f /* the low points of the land sit this far down */
#define MTN_AMP1          0.10f /* how tall the land peaks rise (× rows)    */
#define MTN_AMP2          0.05f
#define MTN_FREQ1         0.055f/* how bunched-up the land peaks are        */
#define MTN_FREQ2         0.130f
#define HEM_BASE_FRAC     0.54f /* the curtain's bright bottom edge floats this far down */
#define HEM_RIPPLE_FRAC   0.07f /* how far that edge wobbles up and down (× rows) */
#define HEM_RIPPLE_FREQ   0.10f /* how bunched-up the wobbles are           */
#define HEM_DRIFT_SPEED   0.25f /* how fast the bottom edge slides sideways */
#define CURTAIN_LEN_FRAC  0.34f /* how tall the curtain reaches above its edge (× rows) */
#define FOLD_FREQ         0.42f /* how close together the vertical folds are*/
#define FOLD_SPEED        0.30f /* how fast the folds slide sideways        */
#define CURTAIN_TOP       1.15f /* past this height fraction we're off the top of the curtain → sky */
#define AURORA_FLOOR      0.07f /* dimmer than this and the cell shows sky/stars instead */

/* Each background cell is a star this often. We hash (col,row) instead of
 * storing a map, so the stars never move and never need memory. */
#define STAR_THRESH  5          /* out of 256, so about 2% of cells        */

/*
 * Colour themes. Each row is a gradient the curtain is painted from: the first
 * colour is the bright bottom edge, the last is the faint top. A cell's colour
 * is mostly its height up the curtain, nudged sideways by a slow colour wave.
 * Every colour sits in the bright half of the palette so dim cells stay
 * visible. Cycle with t/T:
 *   EMERALD  — the common oxygen-green aurora, green fading to pale mint.
 *   SPECTRUM — green to cyan to blue to purple to a pink fringe.
 *   CRIMSON  — green base rising to the rare high-altitude red tips.
 *   VIOLET   — nitrogen blue to violet to magenta.
 */
#define PAIR_AURORA_BASE  10
#define N_AURORA_RAMP     18
#define N_AURORA_THEMES    4
static const char *const aurora_theme_names[N_AURORA_THEMES] = {
    "EMERALD ", "SPECTRUM", "CRIMSON ", "VIOLET  ",
};
static const short aurora_themes[N_AURORA_THEMES][N_AURORA_RAMP] = {
    {  40,  46,  47,  48,  49,  50,  83,  84,  85, 120, 121, 122, 156, 157, 158, 159, 194, 195 },
    {  46,  47,  48,  49,  50,  51,  45,  39,  75, 111, 147, 141, 135, 171, 177, 207, 213, 219 },
    {  46,  47,  83,  84, 120, 154, 191, 184, 220, 214, 208, 202, 196, 203, 197, 161, 168, 211 },
    {  51,  45,  39,  75,  81, 117, 147, 111, 105,  99, 141, 135, 171, 177, 183, 219, 213, 225 },
};

/* The colour bands slide sideways over time. */
#define COLOR_WAVE_FREQ   1.10f  /* how bunched-up the colour bands are      */
#define COLOR_WAVE_SPEED  0.35f  /* how fast they roll sideways              */
#define COLOR_WAVE_AMP    4.0f   /* how many colours up/down the wave shifts a cell */

#define PAIR_LAND          8     /* the dark land                           */

/* ── §2 timing — read the clock, sleep ── */

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

/* ── §3 logic — pure questions about a cell, no drawing ── */

/* star_at — is this cell a star? Hashing (col,row) gives the same answer every
 * frame, so stars never flicker and we never store a star map. */
static bool star_at(int col, int row, char *out_ch, int *out_pair)
{
    unsigned h = (unsigned)(col * 1234597u ^ row * 987659u ^ (col + row * 31));
    h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
    if ((h & 0xFF) >= STAR_THRESH) return false;
    *out_ch   = ((h >> 8) & 3) == 0 ? '*' : ((h >> 8) & 3) == 1 ? '+' : '.';
    *out_pair = ((h >> 10) & 1) ? 3 : 6;   /* yellow or blue */
    return true;
}

/* seed_unit — turn one seed plus a channel number into a random value in
 * [0,1). Lets a single seed fan out into many independent random knobs that all
 * change together when the seed changes (r key). */
static float seed_unit(unsigned seed, int chan)
{
    unsigned h = seed + (unsigned)chan * 0x9e3779b9u;
    h ^= h >> 16; h *= 0x7feb352du;
    h ^= h >> 15; h *= 0x846ca68bu;
    h ^= h >> 16;
    return (float)h / 4294967296.0f;
}

/* mountain_top — which screen row the land's top edge is at in this column.
 * Anything at or below this row is dark land. Two stacked waves make the jagged
 * peaks; the seed sets the height and shift, so each regen gets a new skyline. */
static int mountain_top(int col, int rows, float amp1, float ph1, float ph2)
{
    float base  = (float)rows * HORIZON_FRAC;
    float ridge = (float)rows * amp1     * fabsf(sinf((float)col * MTN_FREQ1 + ph1))
                + (float)rows * MTN_AMP2 * fabsf(sinf((float)col * MTN_FREQ2 + ph2));
    return (int)(base - ridge);
}

/* hem_row — which row the curtain's bright bottom edge is at in this column. It
 * sits around `base`, wobbles up and down, and slides sideways over time; `ph`
 * (from the seed) shifts the wobble so regens look different. */
static float hem_row(int col, float t, int rows, float base, float ph)
{
    float ripple  = sinf((float)col * HEM_RIPPLE_FREQ + t * HEM_DRIFT_SPEED + ph);
    ripple += 0.4f * sinf((float)col * HEM_RIPPLE_FREQ * 2.3f
                          - t * HEM_DRIFT_SPEED * 0.7f + ph);
    return base + (float)rows * HEM_RIPPLE_FRAC * ripple;
}

/* curtain_intensity — how bright the curtain is at one point, from 0 to 1.
 * `d` is how many rows above the bottom edge we are; `dn` is the same as a
 * fraction (0 at the bright edge, 1 at the faint tip). Brightness is three
 * things multiplied: a living shimmer, the vertical folds, and a fade that
 * makes the bottom edge the brightest part. x is the column's position around
 * the screen; ph_tex and fold_freq come from the seed. */
static float curtain_intensity(float x, int col, float d, float dn, float t,
                               float ph_tex, float fold_freq)
{
    float n1      = sinf(x * 1.5f + t * 0.20f + ph_tex) * cosf(d * 0.25f + t * 0.50f);
    float n2      = cosf(x * 2.3f - t * 0.15f + ph_tex) * sinf(d * 0.40f + t * 0.80f);
    float shimmer = (n1 * 0.6f + n2 * 0.4f) * 0.5f + 0.5f;                /* [0,1] */
    float fold    = 0.55f + 0.45f * sinf((float)col * fold_freq + t * FOLD_SPEED + ph_tex);
    float fade_up = 1.0f - dn; if (fade_up < 0.0f) fade_up = 0.0f;
    return fade_up * fold * (0.55f + 0.45f * shimmer);
}

/* ── §4 simulation — the only place state changes ── */

/* The whole scene is just these two numbers, because every cell is recomputed
 * from scratch each frame (aurora_draw, §7) — nothing is stored between frames.
 *   time  The animation clock, in seconds, starting at 0 and only going up.
 *         aurora_tick adds the frame's elapsed time. The drift and shimmer
 *         happen because `time` feeds the sine waves' positions, and a sine
 *         scrolls as that position grows. A float is plenty here, and main()
 *         caps each step at 100 ms so a hiccup can't jump the animation.
 *   seed  A random number that picks WHICH scene you see — the skyline, the
 *         curtain shape, the colour shifts. seed_unit() (§3) fans it out into
 *         many random knobs; aurora_reseed() (r key) rolls a new one. Unsigned
 *         so it wraps cleanly when it overflows. The starting value is a
 *         well-mixed constant that gives a nice stable first view.
 * Refs: Ebert et al., *Texturing & Modeling* (computing images from a formula);
 *       Jarzynski & Olano, JCGT 2020 (the seed hash). */
typedef struct {
    float    time;
    unsigned seed;
} Aurora;

static void aurora_init(Aurora *a)
{
    a->time = 0.0f;
    a->seed = 0x9e3779b9u;   /* fixed first view; r rerolls it */
}

static void aurora_tick(Aurora *a, float dt)
{
    a->time += dt;
}

/* aurora_reseed — r key: roll a new seed so the whole scene — mountains,
 * curtain shape, colours — regenerates into a fresh random aurora. */
static void aurora_reseed(Aurora *a)
{
    a->seed = a->seed * 1664525u + 1013904223u;
}

/* The display plus the few knobs the user controls.
 *   aurora  WHAT is shown — the scene above.
 *   paused  When true, scene_tick stops advancing time, so motion freezes but
 *           drawing keeps going (you see a held frame, not a black screen).
 *           Toggled by space. Kept here, not in Aurora, so pausing doesn't touch
 *           the scene's own identity.
 *   theme   WHICH colour palette is active, 0..N_AURORA_THEMES-1 into
 *           aurora_themes[] (§1). Cycled by t/T. Kept off Aurora so recolouring
 *           never disturbs the scene. */
typedef struct {
    Aurora aurora;
    bool   paused;
    int    theme;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    (void)cols; (void)rows;
    memset(s, 0, sizeof *s);   /* paused=false, theme=0 */
    aurora_init(&s->aurora);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)cols; (void)rows;
    if (s->paused) return;
    aurora_tick(&s->aurora, dt);
}

/* ── §5 effects — nothing stored; the shimmer, folds and twinkle are all
 *    recomputed at draw time, so there's no state here ── */

/* ── §6 delays — nothing here but the pause flag, which lives on Scene (§4) ── */

/* ── §7 render — turn the scene into characters on screen ── */

/* apply_aurora_theme — point the curtain's colour slots at theme t. Safe to
 * call while running (t/T key); the next redraw shows the new palette. On
 * terminals with only 8 colours the gradient collapses to green/cyan/blue/magenta. */
static void apply_aurora_theme(int t)
{
    if (t < 0 || t >= N_AURORA_THEMES) t = 0;
    if (COLORS >= 256) {
        for (int i = 0; i < N_AURORA_RAMP; i++)
            init_pair((short)(PAIR_AURORA_BASE + i), aurora_themes[t][i], COLOR_BLACK);
    } else {
        static const short fb[4] = { COLOR_GREEN, COLOR_CYAN, COLOR_BLUE, COLOR_MAGENTA };
        for (int i = 0; i < N_AURORA_RAMP; i++)
            init_pair((short)(PAIR_AURORA_BASE + i), fb[i * 4 / N_AURORA_RAMP], COLOR_BLACK);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    /* HUD + stars use pairs 3/5/6 (yellow/cyan/blue); the land is PAIR_LAND;
     * the aurora curtain uses the PAIR_AURORA_BASE ramp (set by the theme). */
    if (COLORS >= 256) {
        init_pair(3, 226, COLOR_BLACK);   /* yellow — HUD data / star */
        init_pair(5,  51, COLOR_BLACK);   /* cyan   — HUD title       */
        init_pair(6,  33, COLOR_BLACK);   /* blue   — HUD hint / star */
        init_pair(PAIR_LAND, 238, COLOR_BLACK);   /* dark-slate land */
    } else {
        init_pair(3, COLOR_YELLOW, COLOR_BLACK);
        init_pair(5, COLOR_CYAN,   COLOR_BLACK);
        init_pair(6, COLOR_BLUE,   COLOR_BLACK);
        init_pair(PAIR_LAND, COLOR_BLUE, COLOR_BLACK);
    }
    apply_aurora_theme(0);   /* default theme */
}

/* draw_star — draw the star at (col,row) if there is one, with a slow twinkle:
 * mixing the position with a coarse time value makes a few stars flare bright
 * now and then, so the field sparkles instead of sitting frozen. */
static void draw_star(WINDOW *w, int col, int row, float t)
{
    char sch; int spair;
    if (!star_at(col, row, &sch, &spair)) return;
    unsigned th = (unsigned)(col * 7u + row * 131u) ^ (unsigned)(int)(t * 1.7f);
    th ^= th >> 13; th *= 0x5bd1e995u; th ^= th >> 15;
    chtype attr = ((th & 15u) == 0u) ? A_BOLD : A_DIM;   /* occasional flare */
    wattron(w, COLOR_PAIR(spair) | attr);
    mvwaddch(w, row, col, (chtype)(unsigned char)sch);
    wattroff(w, COLOR_PAIR(spair) | attr);
}

static void draw_land(WINDOW *w, int row, int col)
{
    wattron(w, COLOR_PAIR(PAIR_LAND));
    mvwaddch(w, row, col, (chtype)(unsigned char)'#');
    wattroff(w, COLOR_PAIR(PAIR_LAND));
}

/* aurora_pair — pick the colour for a curtain cell: green at the bottom edge
 * (dn=0) up to purple at the tips (dn=1), nudged sideways by a slow colour wave. */
static int aurora_pair(float dn, float x, float t, float ph_col)
{
    float colwave = sinf(x * COLOR_WAVE_FREQ + t * COLOR_WAVE_SPEED + ph_col);
    int   idx = (int)(dn * (float)(N_AURORA_RAMP - 1) + colwave * COLOR_WAVE_AMP + 0.5f);
    if (idx < 0) idx = 0;
    else if (idx >= N_AURORA_RAMP) idx = N_AURORA_RAMP - 1;
    return PAIR_AURORA_BASE + idx;
}

/* curtain_glyph — pick the character and bold/dim for a brightness: faint dots
 * for dim cells rising to solid strokes for the brightest ones. */
static char curtain_glyph(float bright, chtype *attr)
{
    *attr = (bright > 0.55f) ? A_BOLD : (bright < 0.15f) ? A_DIM : 0;
    if      (bright < 0.14f) return '.';
    else if (bright < 0.28f) return ':';
    else if (bright < 0.46f) return '|';
    else if (bright < 0.68f) return '!';
    else                     return '#';
}

/*
 * aurora_draw — paint the whole scene. For each column we find the land's top
 * edge and the curtain's bottom edge, then walk down the column deciding what
 * each cell is: land below the ridge, the glowing curtain in a band above its
 * bottom edge, and open sky (stars) everywhere else.
 */
static void aurora_draw(const Aurora *a, WINDOW *w, int cols, int rows)
{
    float t  = a->time;
    unsigned sd = a->seed;

    /* Pull a fresh set of random knobs out of the seed: where each wave starts
     * (the look) plus a few sizes — skyline height, curtain reach, edge
     * position, fold spacing (the mood). */
    const float TWO_PI = 2.0f * (float)M_PI;
    float ph_mtn1 = seed_unit(sd, 0) * TWO_PI;
    float ph_mtn2 = seed_unit(sd, 1) * TWO_PI;
    float ph_hem  = seed_unit(sd, 2) * TWO_PI;
    float ph_tex  = seed_unit(sd, 3) * TWO_PI;   /* shimmer + folds */
    float ph_col  = seed_unit(sd, 4) * TWO_PI;

    float mtn_amp1    = MTN_AMP1 * (0.65f + 0.70f * seed_unit(sd, 5));
    float hem_base    = (float)rows * (HEM_BASE_FRAC + (seed_unit(sd, 6) - 0.5f) * 0.08f);
    float curtain_len = (float)rows * CURTAIN_LEN_FRAC * (0.80f + 0.40f * seed_unit(sd, 7));
    float fold_freq   = FOLD_FREQ * (0.70f + 0.60f * seed_unit(sd, 8));
    if (curtain_len < 1.0f) curtain_len = 1.0f;

    /* The land edge and curtain edge are the same all the way down a column, so
     * work them out once per column instead of once per cell. */
    int   ridge[AUR_MAX_COLS];
    float hem[AUR_MAX_COLS];
    int   ncol = cols < AUR_MAX_COLS ? cols : AUR_MAX_COLS;
    for (int c = 0; c < ncol; c++) {
        ridge[c] = mountain_top(c, rows, mtn_amp1, ph_mtn1, ph_mtn2);
        hem[c]   = hem_row(c, t, rows, hem_base, ph_hem);
    }

    for (int row = 1; row < rows - 1; row++) {
        for (int col = 0; col < ncol; col++) {

            /* Work out what this cell is. Below the land edge it's land.
             * Below the curtain's bottom edge, or above its tip, or too faint
             * to see, it's open sky (a star or nothing). */
            if (row >= ridge[col]) { draw_land(w, row, col); continue; }

            float above_hem = hem[col] - (float)row;    /* how far up into the curtain */
            if (above_hem < 0.0f)        { draw_star(w, col, row, t); continue; }
            float dn = above_hem / curtain_len;         /* 0 at the bottom edge, 1 at the tip */
            if (dn > CURTAIN_TOP)        { draw_star(w, col, row, t); continue; }

            float x = (float)col / (float)cols * 2.0f * (float)M_PI;   /* this column's spot around the screen */
            float bright = curtain_intensity(x, col, above_hem, dn, t, ph_tex, fold_freq);
            if (bright < AURORA_FLOOR)   { draw_star(w, col, row, t); continue; }

            /* It's a curtain cell: colour by height, character by brightness. */
            int    pair = aurora_pair(dn, x, t, ph_col);
            chtype attr;
            char   ch = curtain_glyph(bright, &attr);
            wattron(w, COLOR_PAIR(pair) | attr);
            mvwaddch(w, row, col, (chtype)(unsigned char)ch);
            wattroff(w, COLOR_PAIR(pair) | attr);
        }
    }
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    aurora_draw(&s->aurora, w, cols, rows);
}

/* The terminal's current size, in character cells. Holds no scene state, so
 * resizing can't disturb the aurora. Re-read on every resize (main, §8).
 *   cols  width  — valid x is 0..cols-1.
 *   rows  height — valid y is 0..rows-1. */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

/* Paint a row solid with spaces so the HUD bar covers the aurora drawn under it. */
static void hud_clear_row(int row, int cols)
{
    for (int x = 0; x < cols; x++) mvaddch(row, x, ' ');
}

/*
 * screen_draw — draw the aurora, then lay a two-line HUD over it: the top row
 * has the title on the left and fps / Hz / paused on the right; the bottom row
 * lists the keys. The fps block is nudged so it can't overlap the title.
 */
static void screen_draw(const Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    int cols = s->cols;

    /* Top row: title on the left, fps / Hz / paused on the right. */
    hud_clear_row(0, cols);

    char lbuf[64];
    int  th = sc->theme;
    snprintf(lbuf, sizeof lbuf, " AURORA BOREALIS   theme:%s %d/%d ",
             aurora_theme_names[th], th + 1, N_AURORA_THEMES);
    int llen = (int)strlen(lbuf);
    attron(COLOR_PAIR(5) | A_BOLD);
    mvprintw(0, 1, "%s", lbuf);
    attroff(COLOR_PAIR(5) | A_BOLD);

    char buf[64];
    snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz  %s ",
             fps, sim_fps, sc->paused ? "PAUSED" : "");
    int hx = cols - (int)strlen(buf);
    if (hx < llen + 2) hx = llen + 2;               /* keep clear of the title */
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    /* Bottom row: the key legend. */
    int brow = s->rows - 1;
    hud_clear_row(brow, cols);
    attron(COLOR_PAIR(6) | A_BOLD);
    mvprintw(brow, 1, "q:quit   r:regen   t/T:theme   spc:pause   [/]:Hz");
    attroff(COLOR_PAIR(6) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §8 app — input, the main loop, and gluing it all together ── */

/* Everything one running instance owns, bundled so the main loop can carry one
 * handle.
 *   scene        the display being shown + its user knobs.
 *   screen       the terminal size.
 *   sim_fps      how many times a second time advances, clamped between
 *                SIM_FPS_MIN and SIM_FPS_MAX. This is how SMOOTH the motion is,
 *                not how FAST it drifts (the speed is baked into the formulas).
 *                Adjusted by [/].
 *   running      the loop runs while this is non-zero. The SIGINT/SIGTERM
 *                handlers set it to 0 to quit. It's volatile sig_atomic_t
 *                because that's the only type C promises is safe to touch from a
 *                signal handler.
 *   need_resize  the SIGWINCH handler sets this to 1; the loop notices, re-reads
 *                the terminal size, and clears it. Same signal-safety reason. */
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
static void cleanup(void)             { endwin(); }

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': app->scene.paused = !app->scene.paused; break;
    case 'r': case 'R': aurora_reseed(&app->scene.aurora); break;
    case 't':
        app->scene.theme = (app->scene.theme + 1) % N_AURORA_THEMES;
        apply_aurora_theme(app->scene.theme);
        break;
    case 'T':
        app->scene.theme =
            (app->scene.theme + N_AURORA_THEMES - 1) % N_AURORA_THEMES;
        apply_aurora_theme(app->scene.theme);
        break;
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
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {
        if (app->need_resize) {
            endwin(); refresh();
            getmaxyx(stdscr, app->screen.rows, app->screen.cols);
            app->need_resize = 0;
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        float alpha = (float)sim_accum / (float)tick_ns;

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha, dt_sec);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
