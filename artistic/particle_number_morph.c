/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * particle_number_morph.c — Solid Particle Number Morphing (LERP)
 *
 * Up to 500 particles densely fill every pixel of a large bitmap font,
 * cycling digits 0 → 1 → 2 → … → 9 → 0.
 *
 * On each digit change a greedy nearest-neighbour match assigns every
 * particle the closest target in the new digit.  Positions are then
 * linearly interpolated (smoothstep easing) from their snapshot at the
 * moment of change to the new target — no spring forces, no velocity.
 * The result is a clean, deterministic glide between shapes.
 *
 * Idle particles (excess over the new digit's count) glide back to the
 * digit centre and become invisible, then rejoin the swarm on future
 * digits that need more particles.
 *
 * Font: 9-row × 7-col bitmap, each '#' expanded to a sub-grid of
 * particles scaled to the terminal (up to 3 rows × 4 cols per pixel).
 *
 * Keys:
 *   q / ESC   quit
 *   p / spc   pause / resume
 *   n         next digit immediately
 *   ] / [     hold time longer / shorter
 *   f / F     morph faster / slower
 *   t / T     next / prev theme  (Neon/Fire/Ice/Plasma/Mono)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra particle_number_morph.c \
 *       -o particle_number_morph -lncurses -lm
 *
 * §1  config & font
 * §2  clock
 * §3  color / themes
 * §4  target precomputation
 * §5  particles  (lerp)
 * §6  draw
 * §7  app / main
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config & font                                                      */
/* ===================================================================== */

#define RENDER_FPS  30
#define RENDER_NS  (1000000000LL / RENDER_FPS)
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* Bitmap font — '#' = filled, ' ' = empty.  9 rows × 7 cols. */
#define FONT_R  9
#define FONT_C  7

static const char k_font[10][FONT_R][FONT_C + 1] = {
    { /* 0 */  " ##### ", "##   ##", "##   ##", "##   ##",
               "##   ##", "##   ##", "##   ##", "##   ##", " ##### " },
    { /* 1 */  "   ##  ", "  ###  ", "   ##  ", "   ##  ",
               "   ##  ", "   ##  ", "   ##  ", "   ##  ", " ##### " },
    { /* 2 */  " ##### ", "##   ##", "     ##", "     ##",
               "  #### ", " ###   ", "###    ", "##     ", "#######" },
    { /* 3 */  " ##### ", "##   ##", "     ##", "     ##",
               "  #### ", "     ##", "     ##", "##   ##", " ##### " },
    { /* 4 */  "##   ##", "##   ##", "##   ##", "##   ##",
               "#######", "     ##", "     ##", "     ##", "     ##" },
    { /* 5 */  "#######", "##     ", "##     ", "###### ",
               "     ##", "     ##", "     ##", "##   ##", " ##### " },
    { /* 6 */  " ##### ", "##   ##", "##     ", "##     ",
               "###### ", "##   ##", "##   ##", "##   ##", " ##### " },
    { /* 7 */  "#######", "##   ##", "     ##", "    ## ",
               "    ## ", "   ##  ", "   ##  ", "  ##   ", "  ##   " },
    { /* 8 */  " ##### ", "##   ##", "##   ##", "##   ##",
               " ##### ", "##   ##", "##   ##", "##   ##", " ##### " },
    { /* 9 */  " ##### ", "##   ##", "##   ##", "##   ##",
               " ######", "     ##", "     ##", "##   ##", " ##### " },
};

/* Particle pool — digit 8 has 39 '#' pixels, max sub = 3×4 = 12 → 468 < 500 */
#define N_PARTS     500
#define SUB_Y_MAX   3
#define SUB_X_MAX   4

/* Lerp timing */
#define MORPH_FRAMES_MIN   10   /* ~0.33 s */
#define MORPH_FRAMES_DEF   40   /* ~1.33 s */
#define MORPH_FRAMES_MAX  120   /* ~4 s    */
#define HOLD_MIN    20
#define HOLD_DEF    60    /* frames to hold after morph completes */
#define HOLD_MAX   200

#define N_THEMES  5

/* Color pairs */
enum {
    CP_HUD  = 1,
    CP_D0   = 2,    /* CP_D0 + digit → per-digit color */
    CP_IDLE = 12,   /* in-transit idle particles */
    CP_MOV  = 13,   /* active particles mid-lerp */
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + (int64_t)ts.tv_nsec;
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

static uint32_t g_lcg = 12345u;
static float lcg_f(void)
{
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return (float)(g_lcg >> 8) / (float)(1u << 24);
}

/* ===================================================================== */
/* §3  color / themes                                                     */
/* ===================================================================== */

static const struct {
    const char *name;
    short       dig[10];
    short       idle;
    short       mov;
} k_themes[N_THEMES] = {
    { "Neon",   { 51,231,226,208,46,201,196,117,255,220 }, 237, 244 },
    { "Fire",   { 124,196,202,208,214,220,226,228,231,222 }, 52, 130 },
    { "Ice",    { 17,21,27,33,39,44,51,87,123,159 }, 18, 26 },
    { "Plasma", { 54,93,129,165,201,207,213,219,225,231 }, 53, 91 },
    { "Mono",   { 255,255,255,255,255,255,255,255,255,255 }, 237, 247 },
};

static void theme_apply(int t)
{
    for (int d = 0; d < 10; d++)
        init_pair((short)(CP_D0 + d),
                  (COLORS >= 256) ? k_themes[t].dig[d] : COLOR_WHITE,
                  COLOR_BLACK);
    init_pair(CP_IDLE, (COLORS >= 256) ? k_themes[t].idle : COLOR_BLACK, COLOR_BLACK);
    init_pair(CP_MOV,  (COLORS >= 256) ? k_themes[t].mov  : COLOR_WHITE, COLOR_BLACK);
}

static void colors_init(int theme)
{
    start_color();
    init_pair(CP_HUD, 51, COLOR_BLACK);
    theme_apply(theme);
}

/* ===================================================================== */
/* §4  target precomputation                                              */
/* ===================================================================== */

static float g_dtx[10][N_PARTS];
static float g_dty[10][N_PARTS];
static int   g_dtn[10];

static int g_rows, g_cols;
static int g_sy, g_sx;
static int g_suby, g_subx;

static void scale_compute(void)
{
    g_sy = (int)((float)g_rows * 0.65f / (float)FONT_R);
    if (g_sy < 1) g_sy = 1;
    if (g_sy > 8) g_sy = 8;
    g_sx = g_sy * 2;
    if (g_sx > 16) g_sx = 16;
    g_suby = (g_sy < SUB_Y_MAX) ? g_sy : SUB_Y_MAX;
    g_subx = (g_sx < SUB_X_MAX) ? g_sx : SUB_X_MAX;
}

static void targets_precompute(float cx, float cy)
{
    float orig_x = cx - (float)(FONT_C * g_sx) * 0.5f;
    float orig_y = cy - (float)(FONT_R * g_sy) * 0.5f;
    float step_x = (g_subx > 1) ? (float)g_sx / (float)g_subx : (float)g_sx * 0.5f;
    float step_y = (g_suby > 1) ? (float)g_sy / (float)g_suby : (float)g_sy * 0.5f;
    float off_x  = step_x * 0.5f;
    float off_y  = step_y * 0.5f;

    for (int d = 0; d < 10; d++) {
        int n = 0;
        for (int fr = 0; fr < FONT_R; fr++) {
            for (int fc = 0; fc < FONT_C; fc++) {
                if (k_font[d][fr][fc] != '#') continue;
                float bx = orig_x + (float)(fc * g_sx);
                float by = orig_y + (float)(fr * g_sy);
                for (int py = 0; py < g_suby && n < N_PARTS; py++) {
                    for (int px = 0; px < g_subx && n < N_PARTS; px++) {
                        g_dtx[d][n] = bx + off_x + (float)px * step_x;
                        g_dty[d][n] = by + off_y + (float)py * step_y;
                        n++;
                    }
                }
            }
        }
        g_dtn[d] = n;
    }
}

/* ===================================================================== */
/* §5  particles  (lerp)                                                  */
/* ===================================================================== */

/*
 * Each particle stores:
 *   ox, oy — position snapshot taken the moment a new digit is triggered
 *   tx, ty — target for this digit (or centre for idle particles)
 *   x,  y  — current position = lerp(o, t, smoothstep(morph_t))
 *   active — true = assigned to a digit target; false = gliding to centre
 */
typedef struct {
    float x,  y;    /* current position  */
    float ox, oy;   /* origin at morph start */
    float tx, ty;   /* target */
    bool  active;
} Particle;

static Particle g_parts[N_PARTS];
static float    g_morph_t     = 1.0f;   /* 0 = just triggered, 1 = done */
static int      g_morph_frames = MORPH_FRAMES_DEF;

/* Smoothstep easing: starts and ends gently, fast through the middle */
static float smoothstep(float t)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

static void parts_init(float cx, float cy)
{
    for (int i = 0; i < N_PARTS; i++) {
        g_parts[i].x  = cx + (lcg_f() - 0.5f) * (float)(FONT_C * g_sx) * 1.5f;
        g_parts[i].y  = cy + (lcg_f() - 0.5f) * (float)(FONT_R * g_sy) * 1.5f;
        g_parts[i].ox = g_parts[i].x;
        g_parts[i].oy = g_parts[i].y;
        g_parts[i].tx = cx;
        g_parts[i].ty = cy;
        g_parts[i].active = false;
    }
}

/*
 * digit_assign — snapshot current positions, match particles to new
 * digit targets (greedy nearest-neighbour), send idle ones to centre,
 * then reset morph_t to 0 so the lerp restarts.
 */
static void digit_assign(int digit, float cx, float cy)
{
    int   n    = g_dtn[digit];
    bool  used[N_PARTS];
    memset(used, 0, sizeof used);

    /* Mark all inactive first */
    for (int i = 0; i < N_PARTS; i++) g_parts[i].active = false;

    /* Greedy NN: each target gets the nearest unassigned particle */
    for (int t = 0; t < n; t++) {
        float ttx = g_dtx[digit][t];
        float tty = g_dty[digit][t];
        float best_d2 = 1e18f;
        int   best_p  = -1;
        for (int p = 0; p < N_PARTS; p++) {
            if (used[p]) continue;
            float dx = g_parts[p].x - ttx;
            float dy = g_parts[p].y - tty;
            float d2 = dx*dx + dy*dy;
            if (d2 < best_d2) { best_d2 = d2; best_p = p; }
        }
        if (best_p >= 0) {
            used[best_p] = true;
            g_parts[best_p].active = true;
            g_parts[best_p].tx = ttx;
            g_parts[best_p].ty = tty;
        }
    }

    /* Idle particles: send to centre so they disappear cleanly */
    for (int i = 0; i < N_PARTS; i++) {
        if (!used[i]) {
            g_parts[i].active = false;
            g_parts[i].tx = cx;
            g_parts[i].ty = cy;
        }
        /* Snapshot current position as lerp origin */
        g_parts[i].ox = g_parts[i].x;
        g_parts[i].oy = g_parts[i].y;
    }

    g_morph_t = 0.0f;   /* kick off lerp */
}

/*
 * parts_update — advance lerp by one frame.
 * All particles (active and idle) move together from their origin
 * snapshot toward their target using the same eased t value.
 */
static void parts_update(void)
{
    if (g_morph_t >= 1.0f) return;

    g_morph_t += 1.0f / (float)g_morph_frames;
    if (g_morph_t > 1.0f) g_morph_t = 1.0f;

    float st = smoothstep(g_morph_t);   /* eased t */

    for (int i = 0; i < N_PARTS; i++) {
        g_parts[i].x = g_parts[i].ox + st * (g_parts[i].tx - g_parts[i].ox);
        g_parts[i].y = g_parts[i].oy + st * (g_parts[i].ty - g_parts[i].oy);
    }
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

static void parts_draw(int digit)
{
    attr_t a_bright = (attr_t)COLOR_PAIR(CP_D0 + digit) | A_BOLD;
    attr_t a_idle   = (attr_t)COLOR_PAIR(CP_IDLE);

    /* Character varies with lerp progress; color is always the digit color */
    float st = smoothstep(g_morph_t);

    for (int i = 0; i < N_PARTS; i++) {
        Particle *p = &g_parts[i];
        int col = (int)(p->x + 0.5f);
        int row = (int)(p->y + 0.5f);
        if (row < 0 || row >= g_rows - 1) continue;
        if (col < 0 || col >= g_cols)     continue;

        if (p->active) {
            chtype ch;
            if      (st > 0.92f) ch = '@';   /* settled   */
            else if (st > 0.55f) ch = '#';   /* nearly there */
            else if (st > 0.20f) ch = '+';   /* mid-flight */
            else                 ch = '.';   /* just started */
            attron(a_bright);
            mvaddch(row, col, ch);
            attroff(a_bright);
        } else {
            /* Idle: visible only during the lerp, invisible once at centre */
            if (g_morph_t >= 1.0f) continue;
            attron(a_idle);
            mvaddch(row, col, '.');
            attroff(a_idle);
        }
    }
}

static void hud_draw(int digit, int hold_tick, int hold_max,
                     int theme, bool paused, double fps)
{
    /* Morph/hold progress bar */
    int  bar_w = g_cols / 5;
    if (bar_w < 4) bar_w = 4;

    char prog[64];
    memset(prog, '-', (size_t)bar_w);
    prog[bar_w] = '\0';

    int filled;
    if (g_morph_t < 1.0f) {
        /* During morph: show morph progress */
        filled = (int)(g_morph_t * (float)bar_w);
        for (int i = 0; i < filled && i < bar_w; i++) prog[i] = '~';
    } else {
        /* During hold: show hold progress */
        filled = hold_max > 0 ? hold_tick * bar_w / hold_max : 0;
        for (int i = 0; i < filled && i < bar_w; i++) prog[i] = '=';
    }

    char bar[256];
    snprintf(bar, sizeof bar,
             " %d  [q]quit [n]next [t]%s [p]%s [f/F]spd [%s] %.0ffps ",
             digit, k_themes[theme].name,
             paused ? "RESUME" : "PAUSE",
             prog, fps);

    int blen = (int)strlen(bar);
    int bx   = (g_cols - blen) / 2;
    if (bx < 0) bx = 0;

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(g_rows - 1, bx, "%s", bar);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* ===================================================================== */
/* §7  app / main                                                         */
/* ===================================================================== */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void sig_exit(int s)   { (void)s; g_running     = 0; }
static void sig_resize(int s) { (void)s; g_need_resize = 1; }
static void cleanup(void)     { endwin(); }

static void do_resize(float *cx, float *cy)
{
    endwin(); refresh();
    getmaxyx(stdscr, g_rows, g_cols);
    if (g_rows < 5)  g_rows = 5;
    if (g_cols < 10) g_cols = 10;
    *cx = (float)g_cols * 0.5f;
    *cy = (float)g_rows * 0.5f;
    scale_compute();
    targets_precompute(*cx, *cy);
    g_need_resize = 0;
}

int main(void)
{
    g_lcg = (uint32_t)clock_ns();

    atexit(cleanup);
    signal(SIGINT,   sig_exit);
    signal(SIGTERM,  sig_exit);
    signal(SIGWINCH, sig_resize);

    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);

    int theme = 0;
    colors_init(theme);

    getmaxyx(stdscr, g_rows, g_cols);
    float cx = (float)g_cols * 0.5f;
    float cy = (float)g_rows * 0.5f;
    scale_compute();
    targets_precompute(cx, cy);

    parts_init(cx, cy);

    int  cur_digit    = 0;
    int  hold_tick    = 0;
    int  hold_max     = HOLD_DEF;
    bool paused       = false;

    digit_assign(cur_digit, cx, cy);

    double  fps       = 0.0;
    int     frame_cnt = 0;
    int64_t fps_clock = clock_ns();

    while (g_running) {

        if (g_need_resize) {
            do_resize(&cx, &cy);
            digit_assign(cur_digit, cx, cy);
        }

        int64_t frame_start = clock_ns();

        /* --- input --- */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27: g_running = 0; break;
            case 'p': case 'P': case ' ': paused = !paused; break;
            case 'n': case 'N':
                /* skip to next digit immediately */
                hold_tick = hold_max;
                g_morph_t = 1.0f;
                break;
            case ']':
                hold_max += 10;
                if (hold_max > HOLD_MAX) hold_max = HOLD_MAX;
                break;
            case '[':
                hold_max -= 10;
                if (hold_max < HOLD_MIN) hold_max = HOLD_MIN;
                break;
            case 'f':
                /* faster morph: fewer frames */
                g_morph_frames -= 5;
                if (g_morph_frames < MORPH_FRAMES_MIN) g_morph_frames = MORPH_FRAMES_MIN;
                break;
            case 'F':
                /* slower morph: more frames */
                g_morph_frames += 5;
                if (g_morph_frames > MORPH_FRAMES_MAX) g_morph_frames = MORPH_FRAMES_MAX;
                break;
            case 't':
                theme = (theme + 1) % N_THEMES;
                theme_apply(theme);
                break;
            case 'T':
                theme = (theme + N_THEMES - 1) % N_THEMES;
                theme_apply(theme);
                break;
            default: break;
            }
        }

        if (!paused) {
            /* Advance the lerp every frame */
            parts_update();

            /* Hold timer only runs after morph completes */
            if (g_morph_t >= 1.0f) {
                hold_tick++;
                if (hold_tick >= hold_max) {
                    hold_tick = 0;
                    cur_digit = (cur_digit + 1) % 10;
                    digit_assign(cur_digit, cx, cy);
                }
            }
        }

        /* --- draw --- */
        erase();
        parts_draw(cur_digit);
        hud_draw(cur_digit, hold_tick, hold_max, theme, paused, fps);
        wnoutrefresh(stdscr);
        doupdate();

        /* FPS counter */
        frame_cnt++;
        int64_t now = clock_ns();
        if (now - fps_clock >= 500LL * NS_PER_MS) {
            fps       = (double)frame_cnt
                      / ((double)(now - fps_clock) / (double)NS_PER_SEC);
            frame_cnt = 0;
            fps_clock = now;
        }

        clock_sleep_ns(RENDER_NS - (clock_ns() - frame_start));
    }

    return 0;
}
