/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wave_interference.c — Wave Interference Patterns (Analytic)
 *
 * Each of N point sources emits a sinusoidal cylindrical wave:
 *
 *   u_i(x,y,t) = sin( ω_i · t  −  k_i · r_i  +  φ_i )
 *
 * where  r_i = sqrt( (dx·CELL_W)² + (dy·CELL_H)² )  is the pixel-space
 * distance from source i to cell (col, row), corrected for terminal aspect.
 *
 * The total field  u = Σ u_i  is computed analytically every frame —
 * no FDTD, no CFL stability condition, no transient warm-up.  The
 * term  k_i·r_i − φ_i  is precomputed once per source change so the
 * per-frame cost is just N sinf() calls per grid cell.
 *
 * Signed amplitude maps to a 8-level colour ramp:
 *   deep-negative (troughs)  →  cold colours (blue/purple)
 *   near-zero                →  blank (background shows through)
 *   deep-positive (crests)   →  warm colours (white/yellow/orange)
 *
 * 4 presets:
 *   1 DOUBLE SLIT  — 2 coherent in-phase sources → hyperbolic nodal lines
 *   2 RIPPLE TANK  — 4 corner sources → cross-hatched square fringes
 *   3 BEAT         — 2 sources with Δω≠0 → moving beat envelope
 *   4 RADIAL       — 6 sources on circle → 6-fold symmetric star pattern
 *
 * Keys: q quit  p pause  r reset  1-4 preset  t/T theme
 *       Tab / Shift-Tab  select source
 *       Arrow keys  move selected source (recomputes phases on release)
 *       n  add source at centre    x  delete selected source
 *       +/-  wavelength of selected  f/F  frequency of selected
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/wave_interference.c \
 *       -o wave_interference -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 sources  §5 presets  §6 draw  §7 app
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/* Terminal cell pixel dimensions (project convention) */
#define CELL_W   8
#define CELL_H  16

/* Grid caps (the terminal is clamped to these) */
#define GRID_H_MAX  100
#define GRID_W_MAX  300

/* Source defaults */
#define N_SRC_MAX    8
#define OMEGA_DEF    0.15f   /* rad/frame — 1 cycle ≈ 42 frames ≈ 1.4 s  */
#define LAMBDA_DEF  20.0f   /* wavelength in terminal columns              */
#define OMEGA_STEP   0.01f
#define LAMBDA_STEP  2.0f
#define LAMBDA_MIN   6.0f
#define LAMBDA_MAX  60.0f

/* Step: how many frames to advance per real frame (speed control) */
#define RENDER_NS  (1000000000LL / 30)

#define HUD_ROWS    2
#define N_THEMES    5

/* Color pair layout */
enum {
    CP_HUD = 1,
    CP_SRC,          /* selected source marker  */
    CP_SRC2,         /* unselected source marker*/
    /* 4 negative levels, strongest first */
    CP_N3, CP_N2, CP_N1, CP_N0,
    /* 4 positive levels, weakest first */
    CP_P0, CP_P1, CP_P2, CP_P3,
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  color / theme                                                      */
/* ===================================================================== */

/*
 * Each theme defines 4 negative and 4 positive 256-colour levels.
 * neg[0] = strongest negative (deep cold), neg[3] = barely negative.
 * pos[0] = barely positive,               pos[3] = strongest positive.
 */
typedef struct {
    const char *name;
    short neg256[4], pos256[4];
    short neg8,      pos8;
} Theme;

static const Theme k_themes[N_THEMES] = {
    { "Ocean",
      {  17,  21,  27,  33 }, {  51,  87, 123, 231 },
      COLOR_BLUE, COLOR_CYAN },
    { "Fire",
      {  17,  18,  26,  24 }, { 208, 214, 220, 231 },
      COLOR_BLUE, COLOR_YELLOW },
    { "Matrix",
      {  22,  22,  28,  28 }, {  34,  40,  46, 231 },
      COLOR_GREEN, COLOR_GREEN },
    { "Plasma",
      {  17,  53,  89,  91 }, { 165, 201, 207, 231 },
      COLOR_BLUE, COLOR_MAGENTA },
    { "Mono",
      { 232, 234, 236, 238 }, { 244, 248, 251, 231 },
      COLOR_BLACK, COLOR_WHITE },
};

static int g_theme = 0;

static void theme_apply(int ti)
{
    const Theme *t = &k_themes[ti];
    init_pair(CP_HUD,  COLORS >= 256 ? 244 : COLOR_WHITE, -1);
    init_pair(CP_SRC,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(CP_SRC2, COLORS >= 256 ? 244 : COLOR_WHITE,  -1);
    for (int i = 0; i < 4; i++) {
        short fn = COLORS >= 256 ? t->neg256[i] : t->neg8;
        short fp = COLORS >= 256 ? t->pos256[i] : t->pos8;
        init_pair((short)(CP_N3 + i), fn, -1);   /* N3=strongest, N0=weakest */
        init_pair((short)(CP_P0 + i), fp, -1);   /* P0=weakest,  P3=strongest*/
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    theme_apply(g_theme);
}

/* ===================================================================== */
/* §4  sources & analytic field                                           */
/* ===================================================================== */

typedef struct {
    float x, y;      /* position (terminal col, row) */
    float omega;     /* angular frequency (rad/frame) */
    float lambda;    /* wavelength (terminal columns)  */
    float phase;     /* initial phase (rad)            */
    bool  active;
} Source;

static Source g_src[N_SRC_MAX];
static int    g_nactive  = 0;
static int    g_selected = 0;   /* index of selected source */

/*
 * Precomputed table: g_kp[s][r][c] = k_s * pixel_dist(r,c, src_s) − φ_s
 * Updated via recompute_phases() whenever any source changes.
 * Per-frame cost then = g_nactive sinf() calls per cell.
 */
static float g_kp[N_SRC_MAX][GRID_H_MAX][GRID_W_MAX];
static bool  g_dirty = true;

static int   g_gh, g_gw;
static float g_time  = 0.f;
static bool  g_paused = false;

static void recompute_phases(void)
{
    g_dirty = false;
    for (int s = 0; s < N_SRC_MAX; s++) {
        if (!g_src[s].active) continue;
        float k = 6.28318530f / (g_src[s].lambda * (float)CELL_W);
        float sx = g_src[s].x, sy = g_src[s].y, ph = g_src[s].phase;
        for (int r = 0; r < g_gh; r++) {
            float dy = (r - sy) * (float)CELL_H;
            for (int c = 0; c < g_gw; c++) {
                float dx = (c - sx) * (float)CELL_W;
                g_kp[s][r][c] = k * sqrtf(dx*dx + dy*dy) - ph;
            }
        }
    }
}

/* Analytic field sum at cell (r, c) at current time */
static float field(int r, int c)
{
    float u = 0.f;
    for (int s = 0; s < N_SRC_MAX; s++) {
        if (!g_src[s].active) continue;
        u += sinf(g_src[s].omega * g_time - g_kp[s][r][c]);
    }
    return u;
}

static void clear_sources(void)
{
    memset(g_src, 0, sizeof(g_src));
    g_nactive  = 0;
    g_selected = 0;
    g_dirty    = true;
}

static bool add_source(float x, float y, float omega, float lambda, float phase)
{
    for (int s = 0; s < N_SRC_MAX; s++) {
        if (!g_src[s].active) {
            g_src[s] = (Source){ x, y, omega, lambda, phase, true };
            g_nactive++;
            g_selected = s;
            g_dirty    = true;
            return true;
        }
    }
    return false;   /* full */
}

static void delete_source(int s)
{
    if (!g_src[s].active) return;
    g_src[s].active = false;
    g_nactive--;
    /* select nearest remaining source */
    for (int i = 0; i < N_SRC_MAX; i++) {
        if (g_src[i].active) { g_selected = i; return; }
    }
    g_selected = 0;
}

/* ===================================================================== */
/* §5  presets                                                            */
/* ===================================================================== */

static int g_preset = 1;

static void preset_apply(int p)
{
    clear_sources();
    g_time  = 0.f;
    float cx = g_gw * 0.5f, cy = g_gh * 0.5f;
    float o  = OMEGA_DEF, lam = LAMBDA_DEF;

    if (p == 1) {
        /* DOUBLE SLIT — 2 coherent in-phase sources separated vertically.
         * Both sit at the left quarter; the right screen shows hyperbolic
         * nodal lines (destructive) and antinodal lines (constructive).  */
        float x  = g_gw * 0.28f;
        float sep = (float)g_gh * 0.22f;
        add_source(x, cy - sep * 0.5f, o, lam, 0.f);
        add_source(x, cy + sep * 0.5f, o, lam, 0.f);

    } else if (p == 2) {
        /* RIPPLE TANK — 4 corner sources, same frequency and phase.
         * Produces cross-hatched fringes with 4-fold symmetry.        */
        float rx = g_gw * 0.32f, ry = g_gh * 0.30f;
        add_source(cx - rx, cy - ry, o, lam, 0.f);
        add_source(cx + rx, cy - ry, o, lam, 0.f);
        add_source(cx - rx, cy + ry, o, lam, 0.f);
        add_source(cx + rx, cy + ry, o, lam, 0.f);

    } else if (p == 3) {
        /* BEAT — 2 sources with slightly different frequencies.
         * The superposition creates a beat whose envelope drifts slowly.
         * Δω / ω ≈ 15 % gives a beat period of ~7 full cycles.         */
        float x0 = g_gw * 0.30f, x1 = g_gw * 0.70f;
        add_source(x0, cy, o,            lam, 0.f);
        add_source(x1, cy, o * 1.15f,   lam, 0.f);

    } else {
        /* RADIAL — 6 sources evenly spaced on an ellipse (aspect-corrected
         * to look circular) with progressive phase offsets of 2π/6.
         * Produces a 6-fold star-shaped stationary pattern.            */
        float R  = fminf(cx, cy) * 0.55f;
        float Ry = R * 0.5f;   /* halved for terminal aspect ratio */
        int   N  = 6;
        for (int i = 0; i < N; i++) {
            float a = i * 6.28318530f / (float)N;
            add_source(cx + R  * cosf(a),
                       cy + Ry * sinf(a),
                       o, lam,
                       (float)i * 6.28318530f / (float)N);
        }
    }
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

static int g_rows, g_cols;

/*
 * Map normalised u ∈ [−1, +1] to (color_pair, char, attr).
 * Dead-zone ±0.08 left blank so nodal lines clearly show as background.
 * Characters: '.' '+' '#' '@' — density increases with amplitude.
 */
static const char k_ch[4] = { '.', '+', '#', '@' };

static void draw_field(void)
{
    float norm = (g_nactive > 0) ? 1.f / (float)g_nactive : 1.f;

    for (int r = 0; r < g_gh && r + HUD_ROWS < g_rows; r++) {
        for (int c = 0; c < g_gw && c < g_cols; c++) {
            float u  = field(r, c) * norm;
            float au = fabsf(u);
            if (au < 0.08f) continue;

            int lv = (int)(au * 4.f);   /* 0..3 */
            if (lv > 3) lv = 3;

            int cp;
            if (u < 0.f)
                cp = CP_N3 + (3 - lv);   /* N3=strongest … N0=weakest */
            else
                cp = CP_P0 + lv;         /* P0=weakest  … P3=strongest */

            attr_t at = (lv == 3) ? A_BOLD : A_NORMAL;
            attron(COLOR_PAIR(cp) | at);
            mvaddch(r + HUD_ROWS, c, (chtype)k_ch[lv]);
            attroff(COLOR_PAIR(cp) | at);
        }
    }
}

static void draw_sources(void)
{
    for (int s = 0; s < N_SRC_MAX; s++) {
        if (!g_src[s].active) continue;
        int r = (int)g_src[s].y + HUD_ROWS;
        int c = (int)g_src[s].x;
        if (r < HUD_ROWS || r >= g_rows || c < 0 || c >= g_cols) continue;
        int cp = (s == g_selected) ? CP_SRC : CP_SRC2;
        attron(COLOR_PAIR(cp) | A_BOLD);
        mvaddch(r, c, (s == g_selected) ? 'X' : 'o');
        attroff(COLOR_PAIR(cp) | A_BOLD);
    }
}

static void scene_draw(void)
{
    draw_field();
    draw_sources();

    /* HUD */
    Source *sel = (g_nactive > 0 && g_src[g_selected].active)
                  ? &g_src[g_selected] : NULL;

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " WaveInterference  q:quit  p:pause  r:reset  1-4:preset"
        "  t/T:theme  Tab:select  n:add  x:del");
    if (sel) {
        mvprintw(1, 0,
            " src[%d] pos=(%.0f,%.0f)  λ=%.0f cols  f=%.3f rad/fr"
            "  +/-:λ  f/F:freq  arrows:move  [%s]  %s",
            g_selected, sel->x, sel->y,
            sel->lambda, sel->omega,
            k_themes[g_theme].name,
            g_paused ? "PAUSED" : "running");
    } else {
        mvprintw(1, 0, " no active sources — press n to add  [%s]  %s",
            k_themes[g_theme].name,
            g_paused ? "PAUSED" : "running");
    }
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}
static void cleanup(void) { endwin(); }

static void resize_and_reset(void)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    g_rows = rows; g_cols = cols;
    g_gh = (rows - HUD_ROWS) < GRID_H_MAX ? (rows - HUD_ROWS) : GRID_H_MAX;
    g_gw = cols < GRID_W_MAX ? cols : GRID_W_MAX;
    preset_apply(g_preset);
}

/* Move selected source by (dc, dr), clamped to grid */
static void move_src(int dc, int dr)
{
    if (!g_src[g_selected].active) return;
    Source *s = &g_src[g_selected];
    s->x += dc;
    s->y += dr;
    if (s->x < 0.f)          s->x = 0.f;
    if (s->x >= (float)g_gw) s->x = (float)(g_gw - 1);
    if (s->y < 0.f)          s->y = 0.f;
    if (s->y >= (float)g_gh) s->y = (float)(g_gh - 1);
    g_dirty = true;
}

/* Cycle to next active source */
static void next_source(int dir)
{
    if (g_nactive == 0) return;
    int s = g_selected;
    for (int i = 0; i < N_SRC_MAX; i++) {
        s = (s + dir + N_SRC_MAX) % N_SRC_MAX;
        if (g_src[s].active) { g_selected = s; return; }
    }
}

int main(void)
{
    srand((unsigned)time(NULL));
    atexit(cleanup);
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();
    resize_and_reset();

    long long last_ns = clock_ns();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            resize_and_reset();
            last_ns = clock_ns();
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case 'r': case 'R':
            preset_apply(g_preset);
            break;
        case '1': case '2': case '3': case '4':
            g_preset = ch - '0';
            preset_apply(g_preset);
            break;
        case 't':
            g_theme = (g_theme + 1) % N_THEMES;
            theme_apply(g_theme);
            break;
        case 'T':
            g_theme = (g_theme + N_THEMES - 1) % N_THEMES;
            theme_apply(g_theme);
            break;
        case '\t':   /* Tab — next source */
            next_source(+1);
            break;
        case KEY_BTAB:   /* Shift-Tab — prev source */
            next_source(-1);
            break;
        case 'n': case 'N':
            add_source((float)g_gw/2.f, (float)g_gh/2.f,
                       OMEGA_DEF, LAMBDA_DEF, 0.f);
            break;
        case 'x': case 'X':
            if (g_nactive > 0) delete_source(g_selected);
            break;
        case KEY_UP:    move_src( 0, -1); break;
        case KEY_DOWN:  move_src( 0, +1); break;
        case KEY_LEFT:  move_src(-1,  0); break;
        case KEY_RIGHT: move_src(+1,  0); break;
        case '+': case '=':
            if (g_src[g_selected].active) {
                g_src[g_selected].lambda += LAMBDA_STEP;
                if (g_src[g_selected].lambda > LAMBDA_MAX)
                    g_src[g_selected].lambda = LAMBDA_MAX;
                g_dirty = true;
            }
            break;
        case '-':
            if (g_src[g_selected].active) {
                g_src[g_selected].lambda -= LAMBDA_STEP;
                if (g_src[g_selected].lambda < LAMBDA_MIN)
                    g_src[g_selected].lambda = LAMBDA_MIN;
                g_dirty = true;
            }
            break;
        case 'f':
            if (g_src[g_selected].active)
                g_src[g_selected].omega -= OMEGA_STEP;
            break;
        case 'F':
            if (g_src[g_selected].active)
                g_src[g_selected].omega += OMEGA_STEP;
            break;
        default: break;
        }

        /* Recompute distance-phase table if any source moved */
        if (g_dirty) recompute_phases();

        long long now_ns = clock_ns();
        long long dt = now_ns - last_ns;
        if (dt > 100000000LL) dt = 100000000LL;
        last_ns = now_ns;

        if (!g_paused)
            g_time += (float)dt * 1e-9f * (float)(RENDER_NS / 1000000LL) * 0.03f;

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now_ns));
    }
    return 0;
}
