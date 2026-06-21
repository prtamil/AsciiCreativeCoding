/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * led_number_morph.c — particles assemble into LED-style digits 0..9 and back.
 *
 * A cloud of points springs into seven-segment digit shapes, one after the
 * next.  Particle dynamics: Reeves 1983; damped springs: Witkin & Baraff
 * (Physically Based Modeling, SIGGRAPH notes); digit segment bitmasks:
 * Horowitz & Hill, The Art of Electronics.
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — tunable constants and shared scene geometry ── */

#define RENDER_FPS  30
#define RENDER_NS  (1000000000LL / RENDER_FPS)
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* The digit's bounding box, sized to the terminal by dig_size().  The box is
 * a bit wider than it is tall (g_dw ~ g_dh*1.1) because terminal cells are
 * roughly twice as tall as wide, so this gives a square-looking digit. */
static int g_rows, g_cols;
static int g_dh, g_dw;   /* digit box: height and width, in terminal cells */

/* The seven bars of an LED digit, lettered the usual way: top, top-right,
 * bottom-right, bottom, bottom-left, top-left, middle. */
enum { SA=0, SB, SC, SD, SE, SF, SG, N_SEGS };

/* Which bars run sideways vs. up-down: 1 = horizontal, 0 = vertical. */
static const int k_seg_horiz[N_SEGS] = { 1,0,0,1,0,0,1 };

static const char k_ch_h = '-';    /* glyph for a formed horizontal bar */
static const char k_ch_v = '|';    /* glyph for a formed vertical bar   */

#define N_PER_SEG  48                      /* particles per bar — enough that even
                                            * the long bars look solid, not dotted */
#define N_PARTS   (N_SEGS * N_PER_SEG)    /* 336 total */

/* Spring tuning.  Active particles get pulled hard toward their target; idle
 * ones get a gentle pull toward centre plus a tiny random nudge. */
#define SPRING_K   9.0f   /* how hard active particles are pulled to target */
#define DAMP       5.5f   /* friction on active particles (stops overshoot) */
#define DRIFT_K    0.3f   /* gentle pull of idle particles toward centre    */
#define DRIFT_DAMP 1.5f   /* friction on idle particles                     */
#define JITTER     0.06f  /* random wobble added to idle particles          */

/* Where particles start and how fast (parts_init). */
#define PARTS_SCATTER_SPAN 1.5f  /* startup scatter box = this many digit boxes */
#define PARTS_INIT_SPEED   1.5f  /* startup random speed range (cells/frame)    */

/* Glyph-choice cutoffs (parts_draw), all compared against squared values so we
 * skip the square root.  Distances/speeds are in cells. */
#define ARRIVED_D2      0.64f    /* this close to target = counts as "formed" */
#define FLYING_NEAR_D2  6.0f     /* this close = draw the flying '+' bold     */
#define IDLE_VISIBLE_V2 0.04f    /* an idle particle is only drawn if moving faster */

/* How long each digit stays on screen, measured in frames. */
#define HOLD_MIN   35     /* fastest setting                 */
#define HOLD_DEF  100     /* startup default (~3.3 s at 30fps) */
#define HOLD_MAX  270     /* slowest setting                 */

#define N_THEMES   5

/* ncurses color-pair slots.  CP_D0 is a base: CP_D0 + digit gives that
 * digit's color slot (all the same hue, but kept per-digit for simplicity). */
enum {
    CP_HUD  = 1,    /* top readout (yellow)        */
    CP_D0   = 2,    /* CP_D0 + digit → formed particles */
    CP_IDLE = 12,   /* idle drifting particles     */
    CP_MOV  = 13,   /* particles still flying in   */
    CP_HINT = 14,   /* bottom key hints (cyan)     */
};

/* ── §2 performance — monotonic clock helpers for the frame cap ── */

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

/* ── §3 logic — pure 7-segment geometry, no state changes, no drawing ── */

/* Which bars light up for each digit 0..9, packed one bit per bar
 * (bit 0=A, 1=B, ... 6=G).  Standard seven-segment table. */
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

/* Gives the two ends of one bar, for a digit centred at (cx, cy).  The ±1
 * offsets pull each bar in by a cell at the corners so neighbouring bars
 * don't pile up on top of each other where they meet. */
static void seg_endpoints(int s, float cx, float cy,
                          float *x0, float *y0, float *x1, float *y1)
{
    float hw = g_dw * 0.5f;
    float hh = g_dh * 0.5f;
    switch (s) {
    case SA: /* top horizontal */
        *x0 = cx-hw+1; *y0 = cy-hh;   *x1 = cx+hw-1; *y1 = cy-hh;   break;
    case SB: /* top-right vertical */
        *x0 = cx+hw;   *y0 = cy-hh+1; *x1 = cx+hw;   *y1 = cy-1;    break;
    case SC: /* bottom-right vertical */
        *x0 = cx+hw;   *y0 = cy+1;    *x1 = cx+hw;   *y1 = cy+hh-1; break;
    case SD: /* bottom horizontal */
        *x0 = cx-hw+1; *y0 = cy+hh;   *x1 = cx+hw-1; *y1 = cy+hh;   break;
    case SE: /* bottom-left vertical */
        *x0 = cx-hw;   *y0 = cy+1;    *x1 = cx-hw;   *y1 = cy+hh-1; break;
    case SF: /* top-left vertical */
        *x0 = cx-hw;   *y0 = cy-hh+1; *x1 = cx-hw;   *y1 = cy-1;    break;
    case SG: /* middle horizontal */
        *x0 = cx-hw+1; *y0 = cy;      *x1 = cx+hw-1; *y1 = cy;      break;
    default:
        *x0 = cx; *y0 = cy; *x1 = cx; *y1 = cy; break;
    }
}

/* Spreads n target spots evenly along one bar, end to end — these are where
 * that bar's particles want to settle. */
static void seg_targets(int s, float cx, float cy,
                         float *tx, float *ty, int n)
{
    float x0, y0, x1, y1;
    seg_endpoints(s, cx, cy, &x0, &y0, &x1, &y1);
    for (int i = 0; i < n; i++) {
        float t = (n > 1) ? (float)i / (float)(n - 1) : 0.5f;  /* 0..1 along the bar */
        tx[i] = x0 + t * (x1 - x0);
        ty[i] = y0 + t * (y1 - y0);
    }
}

/* ── §4 simulation — moves the particles and sizes the digit box ── */

/* Cheap built-in random number generator (0..1), so the look doesn't depend
 * on the system rand(). */
static uint32_t g_lcg = 12345u;
static float lcg_f(void)
{
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return (float)(g_lcg >> 8) / (float)(1 << 24);
}

/* Picks the digit box size from the current terminal size (called at startup
 * and on resize).  See the width/height note in §1. */
static void dig_size(void)
{
    g_dh = (int)(g_rows * 0.65f);
    if (g_dh < 8)  g_dh = 8;
    if (g_dh > 40) g_dh = 40;
    g_dw = (int)(g_dh * 1.1f);
    if (g_dw > g_cols - 4) g_dw = g_cols - 4;
    if (g_dw < 6) g_dw = 6;
}

/* One moving point in the cloud.  We never draw a digit directly: each
 * particle just chases a spot, and the digit shape appears once they all
 * settle.  Morphing one digit into the next is then trivial — hand every
 * particle a new spot and let the springs do the animating.
 *
 * Each particle is glued for life to one bar: particle i belongs to bar
 * i / N_PER_SEG.  That binding isn't stored (it's just the array index), and
 * it dodges the hard question of "which point goes to which new spot?" — a
 * particle only ever heads for its own bar or drifts to centre, so the motion
 * stays tidy from digit to digit. */
typedef struct {
    float x, y;    /* position, in terminal cells (fractional)              */
    float vx, vy;  /* velocity, in cells per frame                          */
    float tx, ty;  /* the spot this particle wants to reach (used if active) */
    bool  active;  /* true  = this bar is lit, so spring to (tx,ty)
                    * false = bar is dark, so drift slowly toward centre     */
} Particle;

static Particle g_parts[N_PARTS];

static void parts_init(float cx, float cy)
{
    for (int i = 0; i < N_PARTS; i++) {
        /* Start each particle at a random spot around the digit centre. */
        g_parts[i].x  = cx + (lcg_f() - 0.5f) * (float)g_dw * PARTS_SCATTER_SPAN;
        g_parts[i].y  = cy + (lcg_f() - 0.5f) * (float)g_dh * PARTS_SCATTER_SPAN;
        g_parts[i].vx = (lcg_f() - 0.5f) * PARTS_INIT_SPEED;
        g_parts[i].vy = (lcg_f() - 0.5f) * PARTS_INIT_SPEED;
        g_parts[i].tx = cx;
        g_parts[i].ty = cy;
        g_parts[i].active = false;
    }
}

/* Switches the whole cloud over to a new digit: marks each particle's bar as
 * lit or dark and, when lit, gives it a fresh target spot on that bar. */
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

/* Pulls an active particle toward its target like a stiff, well-damped spring,
 * so it arrives quickly without bouncing past. */
static void particle_spring(Particle *p, float dt)
{
    float dx = p->tx - p->x, dy = p->ty - p->y;
    p->vx += (SPRING_K * dx - DAMP * p->vx) * dt;
    p->vy += (SPRING_K * dy - DAMP * p->vy) * dt;
}

/* Drifts an idle particle gently toward centre with a bit of random wobble,
 * so dark areas shimmer faintly instead of freezing. */
static void particle_drift(Particle *p, float cx, float cy, float dt)
{
    float dx = cx - p->x, dy = cy - p->y;
    p->vx += (DRIFT_K * dx - DRIFT_DAMP * p->vx) * dt;
    p->vy += (DRIFT_K * dy - DRIFT_DAMP * p->vy) * dt;
    p->vx += (lcg_f() - 0.5f) * JITTER;
    p->vy += (lcg_f() - 0.5f) * JITTER;
}

/* Moves the particle by its current velocity for one time step. */
static void particle_integrate(Particle *p, float dt)
{
    p->x += p->vx * dt;
    p->y += p->vy * dt;
}

static void parts_update(float dt, float cx, float cy)
{
    for (int i = 0; i < N_PARTS; i++) {
        Particle *p = &g_parts[i];
        if (p->active) particle_spring(p, dt);
        else           particle_drift(p, cx, cy, dt);
        particle_integrate(p, dt);
    }
}

/* ── §5 render — draws the particles and HUD; never changes sim state ── */

/* A color preset for the whole picture.  We deliberately use a single hue for
 * everything: with hundreds of overlapping points, multiple colors made the
 * lit number hard to read.  The three roles (formed / flying / idle) are told
 * apart by glyph and brightness instead — formed bars are bold '-'/'|', flying
 * points are '+', idle ones are a dim '.'.  t/T cycle the presets. */
typedef struct {
    const char *name;      /* name shown in the HUD                       */
    short       color;     /* the one xterm-256 foreground color to use   */
} Theme;

static const Theme k_themes[N_THEMES] = {
    { "Neon",   51  },     /* bright cyan     */
    { "Fire",   208 },     /* bright orange   */
    { "Ice",    117 },     /* bright sky-blue */
    { "Plasma", 201 },     /* bright magenta  */
    { "Mono",   231 },     /* white           */
};

static void theme_apply(int t)
{
    short c = (COLORS >= 256) ? k_themes[t].color : COLOR_WHITE;
    for (int d = 0; d < 10; d++)
        init_pair((short)(CP_D0 + d), c, COLOR_BLACK);
    init_pair(CP_IDLE, c, COLOR_BLACK);
    init_pair(CP_MOV,  c, COLOR_BLACK);
}

static void colors_init(int theme)
{
    start_color();
    init_pair(CP_HUD,  (COLORS >= 256) ? 226 : COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_HINT, (COLORS >= 256) ? 51  : COLOR_CYAN,   COLOR_BLACK);
    theme_apply(theme);
}

/* Draws one particle.  Its glyph and brightness say what it's doing — a formed
 * bar, a flying '+', or a faint drifting '.'.  i picks the bar's glyph. */
static void draw_particle(const Particle *p, int i, int cpair)
{
    int col = (int)(p->x + 0.5f);
    int row = (int)(p->y + 0.5f);
    if (row < 0 || row >= g_rows - 1) return;
    if (col < 0 || col >= g_cols) return;

    if (p->active) {
        float dx = p->tx - p->x, dy = p->ty - p->y;
        float dist2 = dx*dx + dy*dy;          /* (distance to target, squared) */
        chtype ch;
        attr_t attr;
        if (dist2 < ARRIVED_D2) {
            /* Close enough: draw it as part of the bar, '-' or '|'. */
            int seg = i / N_PER_SEG;
            ch   = (chtype)(unsigned char)(k_seg_horiz[seg] ? k_ch_h : k_ch_v);
            attr = (attr_t)COLOR_PAIR(cpair) | A_BOLD;
        } else {
            /* Still flying in: always a '+' so it can't be mistaken for an
             * idle '.'; brighten it as it nears its target. */
            ch   = '+';
            attr = (attr_t)COLOR_PAIR(CP_MOV) | (dist2 < FLYING_NEAR_D2 ? A_BOLD : 0);
        }
        attron(attr);
        mvaddch(row, col, ch);
        attroff(attr);
    } else {
        /* Idle particle: only show it while it's still drifting, and keep it
         * dim so the lit digit stays the star. */
        float spd2 = p->vx*p->vx + p->vy*p->vy;
        if (spd2 < IDLE_VISIBLE_V2) return;
        attron(COLOR_PAIR(CP_IDLE) | A_DIM);
        mvaddch(row, col, '.');
        attroff(COLOR_PAIR(CP_IDLE) | A_DIM);
    }
}

static void parts_draw(int digit)
{
    int cpair = CP_D0 + digit;
    for (int i = 0; i < N_PARTS; i++)
        draw_particle(&g_parts[i], i, cpair);
}

static void hud_draw(int digit, int hold_max, int theme, bool paused, double fps)
{
    /* Top-right: status readout */
    char buf[160];
    snprintf(buf, sizeof buf,
             " digit:%d  theme:%s  hold:%d  %.0f fps  %s ",
             digit, k_themes[theme].name, hold_max, fps,
             paused ? "PAUSED " : "running");
    int x = g_cols - (int)strlen(buf);
    if (x < 0) x = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, x, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Bottom-left: key hints */
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(g_rows - 1, 0, " q:quit  p:pause  n:next  ]/[:speed  t/T:theme ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §6 app / main — input, the per-frame loop, and the frame cap ── */

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

        /* Terminal was resized: recompute the digit box and re-place targets. */
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

        /* Drain pending keypresses. */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27: g_running = 0; break;
            case 'p': case 'P': case ' ': paused = !paused; break;
            case 'n': case 'N':
                hold_tick = hold_max;   /* jump straight to the next digit */
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
            /* Advance the particles one fixed time step. */
            parts_update(1.0f / (float)RENDER_FPS, cx, cy);

            /* Count down the current digit's screen time; swap when it's up. */
            hold_tick++;
            if (hold_tick >= hold_max) {
                hold_tick  = 0;
                cur_digit  = (cur_digit + 1) % 10;
                digit_assign(cur_digit, cx, cy);
            }
        }

        /* Draw the frame. */
        erase();
        parts_draw(cur_digit);
        hud_draw(cur_digit, hold_max, theme, paused, fps);
        wnoutrefresh(stdscr);
        doupdate();

        /* Update the fps reading about twice a second, then sleep to hold ~30fps. */
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
