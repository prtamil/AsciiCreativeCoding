/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * hurricane.c — top-down view of a swirling cyclone with a calm eye.
 *
 * Cloud particles orbit a central eye following the Rankine vortex wind
 * profile and spiral slowly inward. n/p cycle 15 preset storms.
 * Refs: Rankine, "A Manual of Applied Mechanics" (1858) §625 — the two-zone
 * vortex; Emanuel, "Divine Wind" (2005) — eye/eyewall/bands structure.
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config — tunable constants and colour-pair ids ── */

#define TARGET_FPS         60

#define N_CLOUDS_MAX       800
#define N_CLOUDS_MIN       100
#define N_CLOUDS_STEP      100

/* Limits for the keys that nudge the storm live. Radii are in screen cells. */
#define EYE_RADIUS_MIN        2.0f
#define EYE_RADIUS_MAX        8.0f
#define EYE_RADIUS_STEP       1.0f
#define OUTER_RADIUS_FRAC     0.85f   /* fraction of min(rows, cols/2)    */

#define OMEGA_MAX_MIN         0.5f
#define OMEGA_MAX_MAX         4.0f
#define OMEGA_MAX_STEP        0.25f

/* Terminal cells are about twice as tall as wide, so we widen the storm
 * horizontally by this factor to make it look round instead of squashed. */
#define ASPECT_X              2.0f

#define DT_CAP_S              0.10f
#define N_THEMES              4

/* How heavily each new frame pulls the smoothed fps reading; small = steady. */
#define FPS_EMA_ALPHA       0.05

/* Tiny floor on radius so the wind formula doesn't divide by zero at the centre. */
#define VORTEX_CENTRE_EPS   1e-3f

/* Where one coloured band ends and the next begins (see radial_zone). */
#define ZONE_EYE_FRAC       0.70f  /* inside this fraction of the eye → calm centre */
#define ZONE_EYEWALL_FRAC   1.15f  /* up to here → the eyewall (strongest wind)     */
#define ZONE_MIDBAND_FRAC   0.60f  /* up to this fraction of the rim → mid band     */

/* When a cloud spawns or gets recycled, relative to outer rim / eye. */
#define SPAWN_R2_MIN        0.40f  /* spawn point is picked area-uniformly so       */
#define SPAWN_R2_SPAN       0.60f  /*   clouds bunch toward the wider outer rim     */
#define RECYCLE_INNER_FRAC  0.50f  /* pulled this far inside the eye → recycle      */
#define RECYCLE_OUTER_FRAC  1.05f  /* drifted past the rim → recycle                */

/* Colour pairs */
#define PAIR_OUTER    1
#define PAIR_BAND     2
#define PAIR_EYEWALL  3
#define PAIR_EYE      4
#define PAIR_HUD      5
#define PAIR_HINT     6

/* ── §2 performance — monotonic clock and sleep ── */

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

/* ── §3 data — presets, palettes, and the Cloud / vortex / scene types ── */

/* One named storm written down as a recipe. Cycling presets (n/p) copies these
 * settings onto the live storm. It carries no outer radius on purpose — that is
 * worked out from the window size — so the same recipe looks right at any size. */
typedef struct {
    const char *name;   /* label shown in the HUD                           */
    int   theme;        /* palette index 0..N_THEMES-1                      */
    float omega_max;    /* eyewall spin (rad/sec)                           */
    float r_eye;        /* eye radius (cells)                               */
    int   n;            /* cloud count                                      */
    int   inflow;       /* 1 = spiral inward, 0 = concentric orbit rings    */
    float spin_dir;     /* +1 = counter-clockwise, -1 = clockwise           */
    float inflow_rate;  /* radius units/sec inward (spiral tightness)       */
} HurricanePreset;

#define N_PRESETS 15
static const HurricanePreset PRESETS[N_PRESETS] = {
    /*  name              thm  omega   eye    n  inflow spin   rate */
    { "Eye of Calm",       0,  0.9f,  8.0f, 350,  0,   +1.f,  0.5f },
    { "Category Five",     0,  3.6f,  2.0f, 750,  1,   +1.f,  1.0f },
    { "Blue Whirl",        1,  1.8f,  4.0f, 500,  1,   +1.f,  0.7f },
    { "Pinwheel",          1,  3.2f,  5.0f, 600,  0,   -1.f,  0.5f },
    { "Golden Fury",       2,  3.0f,  3.0f, 680,  1,   +1.f,  0.9f },
    { "Solar Flare",       2,  4.0f,  2.0f, 800,  1,   -1.f,  1.2f },
    { "Rose Cyclone",      3,  1.6f,  5.0f, 520,  1,   +1.f,  0.6f },
    { "Crimson Vortex",    3,  3.4f,  3.0f, 700,  1,   -1.f,  1.0f },
    { "Wisps",             0,  2.2f,  4.0f, 180,  1,   +1.f,  0.8f },
    { "Concentric",        0,  1.2f,  6.0f, 550,  0,   +1.f,  0.5f },
    { "Maelstrom",         1,  3.8f,  2.0f, 760,  1,   +1.f,  1.3f },
    { "Slow Drift",        2,  0.7f,  7.0f, 300,  1,   +1.f,  0.3f },
    { "Twin Bands",        3,  2.0f,  5.0f, 640,  0,   -1.f,  0.5f },
    { "Tempest",           1,  2.8f,  3.0f, 680,  1,   -1.f,  0.9f },
    { "Supercell",         0,  4.0f,  2.0f, 800,  1,   +1.f,  1.4f },
};

/* Four colours per theme, one for each band: outer / mid / eyewall / eye.
 * Brightness rises toward the centre so the eyewall stands out as white; the
 * eye is always the same gray (240) since the calm centre isn't part of the hue. */
static const short PAL_256[N_THEMES][4] = {
    /* 0 satellite white — light gray to bright white */
    { 250, 254, 231, 240 },
    /* 1 blue storm — sky blue to cyan to white */
    {  39,  45,  51, 240 },
    /* 2 gold storm — orange to gold to white */
    { 178, 214, 231, 240 },
    /* 3 magenta storm — pink to light pink to white */
    { 175, 213, 231, 240 },
};
static const short PAL_8[N_THEMES][4] = {
    { COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE },
    { COLOR_BLUE,    COLOR_CYAN,    COLOR_WHITE,   COLOR_WHITE },
    { COLOR_YELLOW,  COLOR_YELLOW,  COLOR_WHITE,   COLOR_WHITE },
    { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE,   COLOR_WHITE },
};

/* One cloud particle in the storm. Its position is stored as distance-from-centre
 * and angle (polar coordinates) rather than x/y, because the swirling motion is
 * naturally a circle: each tick just turns the angle and shrinks the radius. The
 * pool is fixed-size and reused — a cloud that reaches the eye or the rim is
 * marked dead and reborn in place (Reeves, "Particle Systems", ACM TOG 1983). */
typedef struct {
    float r;          /* distance from the eye centre, in screen cells       */
    float theta;      /* angle around the eye, in radians                    */
    int   alive;      /* 0 means the slot is free and will be respawned      */
} Cloud;

/* The wind field the clouds ride in. Wind speed grows from the centre out to the
 * eyewall, then fades with distance beyond it — so the fastest wind sits right at
 * the eyewall, just like a real cyclone. A gentle inward pull makes clouds spiral
 * in. (This two-zone wind shape is the Rankine combined vortex: Rankine, "A Manual
 * of Applied Mechanics", 1858, §625; the math lives in rankine_omega.) */
typedef struct {
    float r_eye;        /* eyewall radius in cells; where the wind peaks      */
    float r_outer;      /* outer edge in cells; clouds past it get recycled   */
    float omega_max;    /* spin rate at the eyewall, in radians/second        */
    float spin_dir;     /* +1 spins counter-clockwise, -1 clockwise          */
    int   inflow;       /* 1 = spiral inward, 0 = orbit at a fixed radius     */
    float inflow_rate;  /* how fast clouds drift inward, in cells/second      */
} RankineVortex;

/* The whole storm in one place: the pool of clouds, the wind field they ride,
 * and which preset/theme is showing plus whether it's paused. */
typedef struct {
    Cloud clouds[N_CLOUDS_MAX];
    int   n;                 /* how many clouds are actually in use, [0..n) */
    RankineVortex vortex;    /* the wind field the clouds move in           */
    int   preset_idx;        /* which preset is showing (n/p cycle it)      */
    int   theme;             /* which colour palette is showing             */
    int   paused;            /* 1 freezes the animation                     */
} Hurricane;

static Hurricane g_h;

/* ── §4 logic — pure helpers that compute, never change state ── */

static float frand(void) { return (float)rand() / (float)RAND_MAX; }

/* How fast a cloud spins at distance r from the centre. The spin ramps up to the
 * eyewall, then eases off further out — so wind is fastest right at the eyewall. */
static float rankine_omega(float r, float r_eye, float omega_max)
{
    if (r < VORTEX_CENTRE_EPS) return omega_max * VORTEX_CENTRE_EPS / r_eye;
    if (r <= r_eye) return omega_max * r / r_eye;
    return omega_max * r_eye / r;
}

/* Keep an angle from growing forever by folding it back near one full turn. */
static float wrapped_angle(float a)
{
    if (a >  2.0f * (float)M_PI) a -= 2.0f * (float)M_PI;
    if (a < -2.0f * (float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/* Which coloured band a cloud falls in, from its distance to the centre:
 * 0 outer rim, 1 mid band, 2 eyewall, 3 inside the calm eye. */
static int radial_zone(float r, float r_eye, float r_outer)
{
    if (r < r_eye * ZONE_EYE_FRAC)        return 3;
    if (r < r_eye * ZONE_EYEWALL_FRAC)    return 2;
    if (r < r_outer * ZONE_MIDBAND_FRAC)  return 1;
    return 0;
}

/* True once a cloud has been sucked into the eye or has drifted past the rim,
 * so the caller knows to recycle it. */
static bool cloud_escaped(const Cloud *c, const RankineVortex *v)
{
    return c->r < v->r_eye   * RECYCLE_INNER_FRAC
        || c->r > v->r_outer * RECYCLE_OUTER_FRAC;
}

/* Picks a slash-like character (- \ | /) that points along the wind direction,
 * so the swirl is readable even in a frozen frame. */
static char wind_glyph(float vx, float vy)
{
    static const char G[8] = { '-', '\\', '|', '/', '-', '\\', '|', '/' };
    float ang = atan2f(vy, vx);
    if (ang < 0) ang += 2.0f * (float)M_PI;
    int sect = (int)(ang / (2.0f * (float)M_PI / 8.0f));
    if (sect < 0) sect = 0;
    if (sect > 7) sect = 7;
    return G[sect];
}

/* The wind glyph for one cloud, pointing along its swirl (sideways to the radius). */
static char cloud_wind_glyph(const Cloud *c, const RankineVortex *v)
{
    float omega = rankine_omega(c->r, v->r_eye, v->omega_max);
    float vx = -sinf(c->theta) * omega * c->r * ASPECT_X;
    float vy =  cosf(c->theta) * omega * c->r;
    return wind_glyph(vx, vy);
}

/* Turns a cloud's distance-and-angle into a screen row/column around the eye
 * centre, widening it sideways so the storm looks round. */
static void cloud_to_cell(const Cloud *c, int cx, int cy, int *sr, int *sc)
{
    *sc = (int)((float)cx + ASPECT_X * c->r * cosf(c->theta));
    *sr = (int)((float)cy + c->r * sinf(c->theta));
}

/* True if a cell is off-screen, counting the bottom row reserved for the HUD. */
static bool off_screen(int sr, int sc, int rows, int cols)
{
    return sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols;
}

/* ── §5 simulation — move the clouds forward one tick ── */

static void cloud_respawn(Cloud *c, float r_outer)
{
    /* The sqrt spreads new clouds evenly over area, so they bunch near the
     * roomier outer rim instead of crowding the centre. */
    c->r     = r_outer * sqrtf(SPAWN_R2_MIN + SPAWN_R2_SPAN * frand());
    c->theta = frand() * 2.0f * (float)M_PI;
    c->alive = 1;
}

/* Move one cloud: spin its angle, pull it inward, and recycle it if it falls
 * into the eye or drifts off the rim. */
static void cloud_tick(Cloud *c, const RankineVortex *v, float dt)
{
    if (!c->alive) {
        cloud_respawn(c, v->r_outer);
        return;
    }
    c->theta += rankine_omega(c->r, v->r_eye, v->omega_max) * v->spin_dir * dt;
    if (v->inflow) c->r -= v->inflow_rate * dt;
    c->theta = wrapped_angle(c->theta);
    if (cloud_escaped(c, v)) cloud_respawn(c, v->r_outer);
}

static void hurricane_tick(Hurricane *h, float dt)
{
    for (int i = 0; i < h->n; i++)
        cloud_tick(&h->clouds[i], &h->vortex, dt);
}

/* ── §6 render — paint the storm and HUD to the screen ── */

static void color_init(int theme)
{
    start_color(); use_default_colors();
    int x256 = (COLORS >= 256);
    init_pair(PAIR_OUTER,   x256 ? PAL_256[theme][0] : PAL_8[theme][0], -1);
    init_pair(PAIR_BAND,    x256 ? PAL_256[theme][1] : PAL_8[theme][1], -1);
    init_pair(PAIR_EYEWALL, x256 ? PAL_256[theme][2] : PAL_8[theme][2], -1);
    init_pair(PAIR_EYE,     x256 ? PAL_256[theme][3] : PAL_8[theme][3], -1);
    init_pair(PAIR_HUD,     x256 ? 226 : COLOR_YELLOW, -1);  /* bright yellow — top data bar    */
    init_pair(PAIR_HINT,    x256 ?  51 : COLOR_CYAN,   -1);  /* bright cyan   — bottom action bar */
}

/* Colour and emphasis for a band: the eyewall is bold and brightest. (The eye
 * interior, zone 3, never reaches here — the caller skips it.) */
static chtype zone_attr(int zone)
{
    short pair  = (zone == 2) ? PAIR_EYEWALL
                : (zone == 1) ? PAIR_BAND
                :                PAIR_OUTER;
    chtype attr = COLOR_PAIR(pair);
    if (zone == 2) attr |= A_BOLD;
    return attr;
}

static void cloud_draw(const Cloud *c, int cx, int cy,
                       const RankineVortex *v,
                       int rows, int cols)
{
    if (!c->alive) return;

    int sr, sc;
    cloud_to_cell(c, cx, cy, &sr, &sc);
    if (off_screen(sr, sc, rows, cols)) return;

    int zone = radial_zone(c->r, v->r_eye, v->r_outer);
    if (zone == 3) return;          /* leave the calm eye empty; draw_eye marks it */

    chtype attr = zone_attr(zone);
    char   ch   = cloud_wind_glyph(c, v);
    attron(attr);
    mvaddch(sr, sc, (chtype)(unsigned char)ch);
    attroff(attr);
}

/* Marks the calm centre with three dots. */
static void draw_eye(int cx, int cy, int rows, int cols)
{
    if (cy < 0 || cy >= rows - 1) return;
    attron(COLOR_PAIR(PAIR_EYE) | A_BOLD);
    if (cx >= 0 && cx < cols)            mvaddch(cy, cx,     (chtype)'.');
    if (cx - 2 >= 0 && cx - 2 < cols)    mvaddch(cy, cx - 2, (chtype)'.');
    if (cx + 2 >= 0 && cx + 2 < cols)    mvaddch(cy, cx + 2, (chtype)'.');
    attroff(COLOR_PAIR(PAIR_EYE) | A_BOLD);
}

/* Draws the top stats bar and bottom key legend. Text is clipped to the window
 * width so a narrow terminal can't make the bars overflow or wrap. */
static void draw_hud(const Hurricane *h, double fps, int rows, int cols)
{
    char left[24], right[96];
    snprintf(left,  sizeof left,  " HURRICANE ");
    snprintf(right, sizeof right,
             " preset %2d/%d: %-14s  spin:%.1f  eye:%.0f  %.0f fps  %s ",
             h->preset_idx + 1, N_PRESETS, PRESETS[h->preset_idx].name,
             h->vortex.omega_max, h->vortex.r_eye, fps,
             h->paused ? "PAUSED" : "running");
    int rx = cols - (int)strlen(right);          /* column where the stats start */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int c = 0; c < cols; c++) mvaddch(0, c, ' ');
    if (rx >= 0) {
        mvprintw(0, 0,  "%.*s", rx, left);       /* title, trimmed to clear the stats */
        mvprintw(0, rx, "%s", right);
    } else {
        mvprintw(0, 0,  "%.*s", cols, right);    /* too narrow for both: stats only */
    }
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    for (int c = 0; c < cols; c++) mvaddch(rows - 1, c, ' ');
    mvprintw(rows - 1, 0, "%.*s", cols,
             " n/p:preset  [/]:clouds  -/+:spin  ,/.:eye  i:inflow  t:theme  r:reseed  spc:pause  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(int rows, int cols, const Hurricane *h, double fps)
{
    erase();
    int cx = cols / 2;
    int cy = (rows - 1) / 2;

    for (int i = 0; i < h->n; i++)
        cloud_draw(&h->clouds[i], cx, cy, &h->vortex, rows, cols);
    draw_eye(cx, cy, rows, cols);
    draw_hud(h, fps, rows, cols);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §7 init/reset — fit to the window, reseed, and load presets ── */

static void hurricane_geometry(Hurricane *h, int rows, int cols)
{
    /* Set the outer radius to the biggest circle that fits the window, allowing
     * for the sideways stretch that keeps the storm round. */
    float max_h = (float)(rows - 2) * 0.5f;
    float max_w = (float)cols * 0.5f / ASPECT_X;
    float bound = (max_h < max_w) ? max_h : max_w;
    h->vortex.r_outer = bound * OUTER_RADIUS_FRAC;
}

static void hurricane_reseed(Hurricane *h)
{
    for (int i = 0; i < h->n; i++) cloud_respawn(&h->clouds[i], h->vortex.r_outer);
}

/* Switch to preset idx: copy its settings, load its palette, and scatter the
 * clouds fresh. Caller must set the geometry first, since reseed needs r_outer. */
static void hurricane_apply_preset(Hurricane *h, int idx)
{
    const HurricanePreset *p = &PRESETS[idx];
    h->preset_idx         = idx;
    h->theme              = p->theme;
    h->n                  = p->n;
    h->vortex.omega_max   = p->omega_max;
    h->vortex.r_eye       = p->r_eye;
    h->vortex.inflow      = p->inflow;
    h->vortex.spin_dir    = p->spin_dir;
    h->vortex.inflow_rate = p->inflow_rate;
    color_init(p->theme);
    hurricane_reseed(h);
}

/* ── §8 events — signals, keys, and screen setup ── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running     = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

/* Handles one keypress: cycle presets, tweak the storm, pause, reseed, or quit. */
static void hurricane_handle_key(Hurricane *h, int ch)
{
    switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case ' ':          h->paused ^= 1; break;
        case 'n': case KEY_RIGHT:
            h->preset_idx = (h->preset_idx + 1) % N_PRESETS;
            hurricane_apply_preset(h, h->preset_idx);
            break;
        case 'p': case KEY_LEFT:
            h->preset_idx = (h->preset_idx + N_PRESETS - 1) % N_PRESETS;
            hurricane_apply_preset(h, h->preset_idx);
            break;
        case 'r':          hurricane_reseed(h); break;
        case 't':          h->theme = (h->theme + 1) % N_THEMES;
                           color_init(h->theme); break;
        case 'i':          h->vortex.inflow ^= 1; break;
        case '[':
            if (h->n - N_CLOUDS_STEP >= N_CLOUDS_MIN)
                h->n -= N_CLOUDS_STEP;
            break;
        case ']':
            if (h->n + N_CLOUDS_STEP <= N_CLOUDS_MAX) {
                int old = h->n;
                h->n += N_CLOUDS_STEP;
                for (int i = old; i < h->n; i++)
                    cloud_respawn(&h->clouds[i], h->vortex.r_outer);
            }
            break;
        case '-':
            if (h->vortex.omega_max - OMEGA_MAX_STEP >= OMEGA_MAX_MIN)
                h->vortex.omega_max -= OMEGA_MAX_STEP;
            break;
        case '+': case '=':
            if (h->vortex.omega_max + OMEGA_MAX_STEP <= OMEGA_MAX_MAX)
                h->vortex.omega_max += OMEGA_MAX_STEP;
            break;
        case ',':
            if (h->vortex.r_eye - EYE_RADIUS_STEP >= EYE_RADIUS_MIN)
                h->vortex.r_eye -= EYE_RADIUS_STEP;
            break;
        case '.':
            if (h->vortex.r_eye + EYE_RADIUS_STEP <= EYE_RADIUS_MAX)
                h->vortex.r_eye += EYE_RADIUS_STEP;
            break;
    }
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

/* ── §9 app — the main frame loop ── */

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);
    srand((unsigned)time(NULL));

    screen_init(PRESETS[0].theme);
    int rows = LINES, cols = COLS;
    hurricane_geometry(&g_h, rows, cols);     /* must run before the preset reseed */
    hurricane_apply_preset(&g_h, 0);

    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;
    double  fps         = TARGET_FPS;
    int64_t t_fps_prev  = clock_ns();
    int64_t t_tick_prev = t_fps_prev;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            rows = LINES; cols = COLS;
            hurricane_geometry(&g_h, rows, cols);
        }

        int ch;
        while ((ch = getch()) != ERR) hurricane_handle_key(&g_h, ch);

        int64_t now = clock_ns();
        float   dt  = (float)(now - t_tick_prev) / 1e9f;
        if (dt > DT_CAP_S) dt = DT_CAP_S;
        t_tick_prev = now;
        if (!g_h.paused) hurricane_tick(&g_h, dt);

        /* Smooth the fps so the readout doesn't jitter frame to frame. */
        double inst_fps = 1e9 / (double)(now - t_fps_prev + 1);
        fps = fps * (1.0 - FPS_EMA_ALPHA) + inst_fps * FPS_EMA_ALPHA;
        t_fps_prev = now;

        scene_draw(rows, cols, &g_h, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
