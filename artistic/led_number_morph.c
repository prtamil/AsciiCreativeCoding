/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * led_number_morph.c — Particle LED Number Morphing
 *
 * 168 particles form 7-segment-style digits 0 → 1 → 2 → … → 9 → 0.
 * The display is modelled as seven segments (top/mid/bot horizontal,
 * four vertical halves).  Particles belong permanently to one segment;
 * when the segment is active they spring to evenly-spaced targets on it,
 * when it goes dark they drift slowly back toward centre.
 *
 * Formed particles use orientation-aware characters: horizontal segments
 * render as '─' ('-') and vertical segments as '│' ('|'), so the result
 * looks like a large LED display.  Particles in flight show '+' / '.' .
 *
 * Digit size scales with the terminal so it always fills ≈65% of height.
 *
 * Keys:
 *   q / ESC   quit
 *   p / spc   pause / resume
 *   n         next digit immediately
 *   ] / [     morph faster / slower
 *   t / T     next / prev color theme  (Neon / Fire / Ice / Plasma / Mono)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra led_number_morph.c -o led_number_morph -lncurses -lm
 *
 * §1  config
 * §2  clock
 * §3  color / themes
 * §4  7-segment geometry
 * §5  particles
 * §6  draw
 * §7  app / main
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Spring-mass particle morphing — each particle has a
 *                  target position on the active digit's 7-segment layout.
 *                  Per tick: apply spring force F = k·(target − pos),
 *                  integrate velocity with damping, update position.
 *                  Particles assigned to inactive segments drift toward
 *                  segment centre (aggregation behaviour).
 *
 * Math           : 7-segment encoding: each digit 0–9 activates a subset
 *                  of 7 named segments (top, tl, tr, mid, bl, br, bot).
 *                  Segment lookup table maps digit → bitmask.  Particles are
 *                  permanently bound to one segment and distributed evenly
 *                  along its length when that segment is active.
 *
 * Physics        : Overdamped spring: critical damping chosen so particles
 *                  reach their targets smoothly without overshoot.
 *                  Damping coefficient b chosen near 2√(k·m) (critical
 *                  damping condition).
 *
 * Rendering      : Particles in horizontal segments drawn as '─', vertical
 *                  as '│'; particles in flight (off-segment) as '+' or '.'.
 *                  Digit size scales with terminal height so the display fills
 *                  ~65% of available rows — recomputed on each resize.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define RENDER_FPS  30
#define RENDER_NS  (1000000000LL / RENDER_FPS)
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/*
 * Digit size — computed dynamically from terminal height.
 * g_dh = digit height (rows), g_dw = digit width (cols).
 * 7-segment ratio: physical_width / physical_height ≈ 0.55
 * Terminal cell is ~2× taller than wide, so:
 *   g_dw ≈ g_dh × 0.55 × 2  ≈  g_dh × 1.1
 */
static int g_rows, g_cols;
static int g_dh, g_dw;   /* digit bounding box in terminal cells */

static void dig_size(void)
{
    g_dh = (int)(g_rows * 0.65f);
    if (g_dh < 8)  g_dh = 8;
    if (g_dh > 40) g_dh = 40;
    g_dw = (int)(g_dh * 1.1f);
    if (g_dw > g_cols - 4) g_dw = g_cols - 4;
    if (g_dw < 6) g_dw = 6;
}

/* Segments A-G: A=top, B=top-right, C=bot-right, D=bot,
 *              E=bot-left, F=top-left, G=middle */
enum { SA=0, SB, SC, SD, SE, SF, SG, N_SEGS };

/* Segment orientation: 0=horizontal, 1=vertical */
static const int k_seg_horiz[N_SEGS] = { 1,0,0,1,0,0,1 };

/* Characters for each orientation at different phases */
static const char k_ch_h = '-';    /* formed horizontal segment */
static const char k_ch_v = '|';    /* formed vertical segment   */

#define N_PER_SEG  24                      /* particles per segment */
#define N_PARTS   (N_SEGS * N_PER_SEG)    /* 168 total */

/* Spring physics constants */
#define SPRING_K   9.0f   /* stiffness — active particles   */
#define DAMP       5.5f   /* damping   — active             */
#define DRIFT_K    0.3f   /* stiffness — idle drift to centre */
#define DRIFT_DAMP 1.5f   /* damping   — idle               */
#define JITTER     0.06f  /* random nudge for idle particles */

/* Timing */
#define HOLD_MIN   35     /* min ticks before digit advance */
#define HOLD_DEF  100     /* default (~3.3 s at 30 fps)     */
#define HOLD_MAX  270

#define N_THEMES   5

/* Color pair IDs */
enum {
    CP_HUD  = 1,
    CP_D0   = 2,    /* CP_D0 + digit (0-9) → bright particle color */
    CP_IDLE = 12,   /* dim color for inactive particles             */
    CP_MOV  = 13,   /* medium color for in-flight active particles  */
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
    return (float)(g_lcg >> 8) / (float)(1 << 24);
}

/* ===================================================================== */
/* §3  color / themes                                                     */
/* ===================================================================== */

static const struct {
    const char *name;
    short       dig[10];   /* xterm-256 fg per digit 0-9 */
    short       idle;      /* inactive particle color    */
    short       mov;       /* in-flight particle color   */
} k_themes[N_THEMES] = {
    { "Neon",
      { 51,231,226,208,46,201,196,117,255,220 }, 237, 244 },
    { "Fire",
      { 124,196,202,208,214,220,226,228,231,222 }, 52, 130 },
    { "Ice",
      { 17,21,27,33,39,44,51,87,123,159 }, 18, 26 },
    { "Plasma",
      { 54,93,129,165,201,207,213,219,225,231 }, 53, 91 },
    { "Mono",
      { 238,240,243,245,247,249,251,253,255,231 }, 235, 241 },
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
/* §4  7-segment geometry                                                 */
/* ===================================================================== */

/*
 * 7-segment encoding:  bit 0=A 1=B 2=C 3=D 4=E 5=F 6=G
 */
static const uint8_t k_segs[10] = {
    0x3F, /* 0: ABCDEF  */
    0x06, /* 1: BC      */
    0x5B, /* 2: ABDEG   */
    0x4F, /* 3: ABCDG   */
    0x66, /* 4: BCFG    */
    0x6D, /* 5: ACDFG   */
    0x7D, /* 6: ACDEFG  */
    0x07, /* 7: ABC     */
    0x7F, /* 8: ABCDEFG */
    0x6F, /* 9: ABCDFG  */
};

/*
 * seg_targets — fill tx[],ty[] with n evenly-spaced positions
 * along segment s, for a digit centred at (cx, cy).
 */
static void seg_targets(int s, float cx, float cy,
                         float *tx, float *ty, int n)
{
    float hw = g_dw * 0.5f;
    float hh = g_dh * 0.5f;
    /* Leave 1-cell corner gap so segments don't overlap at junctions */
    float x0, y0, x1, y1;
    switch (s) {
    case SA: /* top horizontal */
        x0 = cx-hw+1; y0 = cy-hh;   x1 = cx+hw-1; y1 = cy-hh;   break;
    case SB: /* top-right vertical */
        x0 = cx+hw;   y0 = cy-hh+1; x1 = cx+hw;   y1 = cy-1;    break;
    case SC: /* bottom-right vertical */
        x0 = cx+hw;   y0 = cy+1;    x1 = cx+hw;   y1 = cy+hh-1; break;
    case SD: /* bottom horizontal */
        x0 = cx-hw+1; y0 = cy+hh;   x1 = cx+hw-1; y1 = cy+hh;   break;
    case SE: /* bottom-left vertical */
        x0 = cx-hw;   y0 = cy+1;    x1 = cx-hw;   y1 = cy+hh-1; break;
    case SF: /* top-left vertical */
        x0 = cx-hw;   y0 = cy-hh+1; x1 = cx-hw;   y1 = cy-1;    break;
    case SG: /* middle horizontal */
        x0 = cx-hw+1; y0 = cy;      x1 = cx+hw-1; y1 = cy;      break;
    default:
        x0 = cx; y0 = cy; x1 = cx; y1 = cy; break;
    }

    for (int i = 0; i < n; i++) {
        float t = (n > 1) ? (float)i / (float)(n - 1) : 0.5f;
        tx[i] = x0 + t * (x1 - x0);
        ty[i] = y0 + t * (y1 - y0);
    }
}

/* ===================================================================== */
/* §5  particles                                                          */
/* ===================================================================== */

typedef struct {
    float x, y;    /* position (terminal float coordinates) */
    float vx, vy;  /* velocity  (cells per frame)           */
    float tx, ty;  /* spring target                         */
    bool  active;  /* true = has a lit-segment target       */
} Particle;

static Particle g_parts[N_PARTS];

static void parts_init(float cx, float cy)
{
    for (int i = 0; i < N_PARTS; i++) {
        /* Scatter randomly within the digit bounding box */
        g_parts[i].x  = cx + (lcg_f() - 0.5f) * (float)g_dw * 1.5f;
        g_parts[i].y  = cy + (lcg_f() - 0.5f) * (float)g_dh * 1.5f;
        g_parts[i].vx = (lcg_f() - 0.5f) * 1.5f;
        g_parts[i].vy = (lcg_f() - 0.5f) * 1.5f;
        g_parts[i].tx = cx;
        g_parts[i].ty = cy;
        g_parts[i].active = false;
    }
}

/*
 * digit_assign — update spring targets for a new digit.
 * Particle i always belongs to segment (i / N_PER_SEG) so particles
 * never need to cross between segments; they either go to their home
 * segment or drift to centre.
 */
static void digit_assign(int digit, float cx, float cy)
{
    uint8_t segs = k_segs[digit];
    for (int s = 0; s < N_SEGS; s++) {
        bool on = (segs >> s) & 1;
        float stx[N_PER_SEG], sty[N_PER_SEG];
        if (on) seg_targets(s, cx, cy, stx, sty, N_PER_SEG);
        for (int j = 0; j < N_PER_SEG; j++) {
            int i = s * N_PER_SEG + j;
            g_parts[i].active = on;
            if (on) { g_parts[i].tx = stx[j]; g_parts[i].ty = sty[j]; }
        }
    }
}

static void parts_update(float dt, float cx, float cy)
{
    for (int i = 0; i < N_PARTS; i++) {
        Particle *p = &g_parts[i];
        if (p->active) {
            float dx = p->tx - p->x, dy = p->ty - p->y;
            p->vx += (SPRING_K * dx - DAMP * p->vx) * dt;
            p->vy += (SPRING_K * dy - DAMP * p->vy) * dt;
        } else {
            /* Drift toward centre with gentle random turbulence */
            float dx = cx - p->x, dy = cy - p->y;
            p->vx += (DRIFT_K * dx - DRIFT_DAMP * p->vx) * dt;
            p->vy += (DRIFT_K * dy - DRIFT_DAMP * p->vy) * dt;
            p->vx += (lcg_f() - 0.5f) * JITTER;
            p->vy += (lcg_f() - 0.5f) * JITTER;
        }
        p->x += p->vx * dt;
        p->y += p->vy * dt;
    }
}

/* ===================================================================== */
/* §6  draw                                                               */
/* ===================================================================== */

static void parts_draw(int digit)
{
    int cpair = CP_D0 + digit;

    for (int i = 0; i < N_PARTS; i++) {
        Particle *p = &g_parts[i];
        int col = (int)(p->x + 0.5f);
        int row = (int)(p->y + 0.5f);
        if (row < 0 || row >= g_rows - 1) continue;
        if (col < 0 || col >= g_cols) continue;

        if (p->active) {
            float dx = p->tx - p->x, dy = p->ty - p->y;
            float dist2 = dx*dx + dy*dy;

            chtype   ch;
            attr_t   attr;

            if (dist2 < 0.64f) {
                /* Arrived — use segment-appropriate char */
                int seg = i / N_PER_SEG;
                ch   = (chtype)(unsigned char)(k_seg_horiz[seg] ? k_ch_h : k_ch_v);
                attr = (attr_t)COLOR_PAIR(cpair) | A_BOLD;
            } else if (dist2 < 6.0f) {
                ch   = '+';
                attr = (attr_t)COLOR_PAIR(CP_MOV) | A_BOLD;
            } else {
                ch   = '.';
                attr = (attr_t)COLOR_PAIR(CP_MOV);
            }
            attron(attr);
            mvaddch(row, col, ch);
            attroff(attr);
        } else {
            /* Idle — only visible if moving */
            float spd2 = p->vx*p->vx + p->vy*p->vy;
            if (spd2 < 0.04f) continue;
            chtype ch = (spd2 > 1.5f) ? '+' : '.';
            attron(COLOR_PAIR(CP_IDLE));
            mvaddch(row, col, ch);
            attroff(COLOR_PAIR(CP_IDLE));
        }
    }
}

static void hud_draw(int digit, int hold_tick, int hold_max,
                     int theme, bool paused, double fps)
{
    /* Bottom bar */
    char bar[256];
    int  bar_w  = g_cols / 4;
    int  filled = (hold_max > 0) ? hold_tick * bar_w / hold_max : 0;

    char prog[64];
    memset(prog, ' ', (size_t)bar_w);
    prog[bar_w] = '\0';
    for (int i = 0; i < filled && i < bar_w; i++) prog[i] = '=';

    snprintf(bar, sizeof bar,
             "[q]quit [n]next [t]theme:%s [][spd [p]%s  [%s]  %.0ffps",
             k_themes[theme].name,
             paused ? "RESUME" : "PAUSE",
             prog, fps);

    int blen = (int)strlen(bar);
    int bx   = (g_cols - blen) / 2;
    if (bx < 0) bx = 0;

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(g_rows - 1, bx, "%s", bar);

    /* Current digit label top-left */
    mvprintw(0, 1, " digit: %d ", digit);
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
    dig_size();

    float cx = (float)g_cols * 0.5f;
    float cy = (float)g_rows * 0.5f;

    parts_init(cx, cy);

    int  cur_digit  = 0;
    int  hold_tick  = 0;
    int  hold_max   = HOLD_DEF;
    bool paused     = false;

    digit_assign(cur_digit, cx, cy);

    double  fps       = 0.0;
    int     frame_cnt = 0;
    int64_t fps_clock = clock_ns();

    while (g_running) {

        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            dig_size();
            cx = (float)g_cols * 0.5f;
            cy = (float)g_rows * 0.5f;
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
                hold_tick = hold_max;   /* force advance on next tick */
                break;
            case ']':
                hold_max -= 10;
                if (hold_max < HOLD_MIN) hold_max = HOLD_MIN;
                break;
            case '[':
                hold_max += 10;
                if (hold_max > HOLD_MAX) hold_max = HOLD_MAX;
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
            /* Physics: fixed dt = 1/FPS */
            parts_update(1.0f / (float)RENDER_FPS, cx, cy);

            hold_tick++;
            if (hold_tick >= hold_max) {
                hold_tick  = 0;
                cur_digit  = (cur_digit + 1) % 10;
                digit_assign(cur_digit, cx, cy);
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
