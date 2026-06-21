/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * particle_number_morph.c — particles morph through the digits 0..9.
 *
 * Up to 500 dots fill in a big bitmap font and glide from one digit's
 * shape to the next. Greedy matching pairs each dot to its nearest pixel
 * in the new digit; smoothstep easing slides them there. No physics.
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config & font — tunable constants and the 9×7 bitmap digits ── */

#define RENDER_FPS  30
#define RENDER_NS  (1000000000LL / RENDER_FPS)
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* Each digit is drawn as a tiny picture: '#' means filled, ' ' means empty. */
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

/* Pool size: the densest digit needs ~468 dots, so 500 leaves headroom. */
#define N_PARTS     500
#define SUB_Y_MAX   3
#define SUB_X_MAX   4

/* How many frames a morph takes — fewer = faster glide. */
#define MORPH_FRAMES_MIN   10   /* ~0.33 s */
#define MORPH_FRAMES_DEF   40   /* ~1.33 s */
#define MORPH_FRAMES_MAX  120   /* ~4 s    */

/* How long a finished digit sits still before morphing to the next. */
#define HOLD_MIN    20
#define HOLD_DEF    60
#define HOLD_MAX   200

#define FONT_HEIGHT_FRAC 0.65f  /* digit fills this fraction of screen height */
#define SCATTER_SPAN     1.5f   /* startup scatter box = this × the digit box */

/* Glyph thresholds: as a dot settles it grows '.' -> '+' -> '#' -> '@'. */
#define GLYPH_SETTLED    0.92f
#define GLYPH_NEAR       0.55f
#define GLYPH_MID        0.20f

#define N_THEMES  5

enum {
    CP_HUD  = 1,    /* top status line (yellow)              */
    CP_D0   = 2,    /* CP_D0 + digit picks that digit's color */
    CP_IDLE = 12,   /* dots gliding to centre and fading out */
    CP_HINT = 14,   /* bottom key hints (cyan)               */
};

/* ── §2 performance — read the clock and sleep to cap the frame rate ── */

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

/* ── §3 logic — the easing curve, no side effects ── */

/* Smooth ramp from 0 to 1 that eases in and out: slow at the ends, quick
 * in the middle. Makes the glide look natural instead of robotic. */
static float smoothstep(float t)
{
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

/* ── §4 simulation — geometry, digit targets, and the morphing dots ── */

/* Cheap random number in [0,1). Used only to scatter dots at startup. */
static uint32_t g_lcg = 12345u;
static float lcg_f(void)
{
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return (float)(g_lcg >> 8) / (float)(1u << 24);
}

/* Screen size and how big to draw the font; recomputed on every resize. */
static int g_rows, g_cols;
static int g_sy, g_sx;
static int g_suby, g_subx;

static void scale_compute(void)
{
    g_sy = (int)((float)g_rows * FONT_HEIGHT_FRAC / (float)FONT_R);
    if (g_sy < 1) g_sy = 1;
    if (g_sy > 8) g_sy = 8;
    g_sx = g_sy * 2;
    if (g_sx > 16) g_sx = 16;
    g_suby = (g_sy < SUB_Y_MAX) ? g_sy : SUB_Y_MAX;
    g_subx = (g_sx < SUB_X_MAX) ? g_sx : SUB_X_MAX;
}

/* Where the dots should land for one digit. We blow the small font picture
 * up to screen size and record a target spot for every filled sub-pixel, so
 * a digit becomes a cloud of points the dots aim for. One of these per digit
 * 0..9, filled in once whenever the scale changes. */
typedef struct {
    float x[N_PARTS];       /* target column of point i                        */
    float y[N_PARTS];       /* target row of point i                           */
    int   n;                /* how many points are valid (rest of arrays junk) */
} DigitTargets;

static DigitTargets g_targets[10];

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
                        g_targets[d].x[n] = bx + off_x + (float)px * step_x;
                        g_targets[d].y[n] = by + off_y + (float)py * step_y;
                        n++;
                    }
                }
            }
        }
        g_targets[d].n = n;
    }
}

/* One dot. Each morph slides it in a straight line from where it was when
 * the digit changed (the origin snapshot) to where it should end up. */
typedef struct {
    float x,  y;    /* current position on screen                            */
    float ox, oy;   /* where it sat when this morph began                    */
    float tx, ty;   /* where it's heading (a digit pixel, or the centre)     */
    bool  active;   /* true = part of the digit; false = surplus, fading out */
} Particle;

static Particle g_parts[N_PARTS];
static float    g_morph_t     = 1.0f;   /* morph progress: 0 = start, 1 = done */
static int      g_morph_frames = MORPH_FRAMES_DEF;

static void parts_init(float cx, float cy)
{
    for (int i = 0; i < N_PARTS; i++) {
        g_parts[i].x  = cx + (lcg_f() - 0.5f) * (float)(FONT_C * g_sx) * SCATTER_SPAN;
        g_parts[i].y  = cy + (lcg_f() - 0.5f) * (float)(FONT_R * g_sy) * SCATTER_SPAN;
        g_parts[i].ox = g_parts[i].x;
        g_parts[i].oy = g_parts[i].y;
        g_parts[i].tx = cx;
        g_parts[i].ty = cy;
        g_parts[i].active = false;
    }
}

/* Find the closest still-free dot to a target spot, or -1 if all are taken.
 * (Compares squared distance to skip the square root — same winner.) */
static int nearest_unclaimed(float tx, float ty, const bool *used)
{
    float best_d2 = 1e18f;
    int   best_p  = -1;
    for (int p = 0; p < N_PARTS; p++) {
        if (used[p]) continue;
        float dx = g_parts[p].x - tx;
        float dy = g_parts[p].y - ty;
        float d2 = dx*dx + dy*dy;
        if (d2 < best_d2) { best_d2 = d2; best_p = p; }
    }
    return best_p;
}

/* Start a morph to a new digit: hand each target pixel its nearest free dot,
 * park the surplus dots at the centre, snapshot where everyone is now, and
 * rewind the progress clock so the glide plays from the beginning. */
static void digit_assign(int digit, float cx, float cy)
{
    const DigitTargets *tg = &g_targets[digit];
    bool  used[N_PARTS];
    memset(used, 0, sizeof used);

    for (int i = 0; i < N_PARTS; i++) g_parts[i].active = false;

    /* Each target spot grabs its nearest unclaimed dot. */
    for (int t = 0; t < tg->n; t++) {
        int p = nearest_unclaimed(tg->x[t], tg->y[t], used);
        if (p < 0) continue;
        used[p] = true;
        g_parts[p].active = true;
        g_parts[p].tx = tg->x[t];
        g_parts[p].ty = tg->y[t];
    }

    /* Leftover dots head for the centre; record everyone's starting point. */
    for (int i = 0; i < N_PARTS; i++) {
        if (!used[i]) {
            g_parts[i].active = false;
            g_parts[i].tx = cx;
            g_parts[i].ty = cy;
        }
        g_parts[i].ox = g_parts[i].x;
        g_parts[i].oy = g_parts[i].y;
    }

    g_morph_t = 0.0f;
}

/* One frame of the glide: nudge progress forward and slide every dot the
 * matching fraction of the way from its start to its target. */
static void parts_update(void)
{
    if (g_morph_t >= 1.0f) return;

    g_morph_t += 1.0f / (float)g_morph_frames;
    if (g_morph_t > 1.0f) g_morph_t = 1.0f;

    float st = smoothstep(g_morph_t);

    for (int i = 0; i < N_PARTS; i++) {
        g_parts[i].x = g_parts[i].ox + st * (g_parts[i].tx - g_parts[i].ox);
        g_parts[i].y = g_parts[i].oy + st * (g_parts[i].ty - g_parts[i].oy);
    }
}

/* ── §5 render — draw the dots and the status lines ── */

/* One colour preset, cycled with t/T. Gives each digit its own colour plus
 * a dim colour for the surplus dots drifting to the centre. */
typedef struct {
    const char *name;       /* shown in the status line                 */
    short       dig[10];    /* colour to use for each digit 0..9         */
    short       idle;       /* colour for the fading surplus dots        */
} Theme;

static const Theme k_themes[N_THEMES] = {
    { "Neon",   { 51,231,226,208,46,201,196,117,255,220 }, 237 },
    { "Fire",   { 124,196,202,208,214,220,226,228,231,222 }, 52 },
    { "Ice",    { 17,21,27,33,39,44,51,87,123,159 }, 18 },
    { "Plasma", { 54,93,129,165,201,207,213,219,225,231 }, 53 },
    { "Mono",   { 255,255,255,255,255,255,255,255,255,255 }, 237 },
};

static void theme_apply(int t)
{
    for (int d = 0; d < 10; d++)
        init_pair((short)(CP_D0 + d),
                  (COLORS >= 256) ? k_themes[t].dig[d] : COLOR_WHITE,
                  COLOR_BLACK);
    init_pair(CP_IDLE, (COLORS >= 256) ? k_themes[t].idle : COLOR_BLACK, COLOR_BLACK);
}

static void colors_init(int theme)
{
    start_color();
    init_pair(CP_HUD,  (COLORS >= 256) ? 226 : COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_HINT, (COLORS >= 256) ? 51  : COLOR_CYAN,   COLOR_BLACK);
    theme_apply(theme);
}

/* Pick the character for a dot by how far along the glide it is. */
static char morph_glyph(float st)
{
    if (st > GLYPH_SETTLED) return '@';
    if (st > GLYPH_NEAR)    return '#';
    if (st > GLYPH_MID)     return '+';
    return '.';
}

static void parts_draw(int digit)
{
    attr_t a_bright = (attr_t)COLOR_PAIR(CP_D0 + digit) | A_BOLD;
    attr_t a_idle   = (attr_t)COLOR_PAIR(CP_IDLE);

    /* Every dot is the same character this frame, just in the digit's colour. */
    char glyph = morph_glyph(smoothstep(g_morph_t));

    for (int i = 0; i < N_PARTS; i++) {
        Particle *p = &g_parts[i];
        int col = (int)(p->x + 0.5f);
        int row = (int)(p->y + 0.5f);
        if (row < 0 || row >= g_rows - 1) continue;   /* keep off the bottom hint line */
        if (col < 0 || col >= g_cols)     continue;

        if (p->active) {
            attron(a_bright);
            mvaddch(row, col, (chtype)(unsigned char)glyph);
            attroff(a_bright);
        } else {
            /* Surplus dots show only while drifting; once parked they vanish. */
            if (g_morph_t >= 1.0f) continue;
            attron(a_idle);
            mvaddch(row, col, '.');
            attroff(a_idle);
        }
    }
}

static void hud_draw(int digit, int hold_max, int theme, bool paused, double fps)
{
    /* Top-right status: which digit, theme, timings, fps. */
    char buf[160];
    snprintf(buf, sizeof buf,
             " digit:%d  theme:%s  morph:%d  hold:%d  %.0f fps  %s ",
             digit, k_themes[theme].name, g_morph_frames, hold_max, fps,
             paused ? "PAUSED " : "running");
    int x = g_cols - (int)strlen(buf);
    if (x < 0) x = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, x, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

    /* Bottom-left: the key hints. */
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(g_rows - 1, 0,
             " q:quit  p:pause  n:next  ]/[:hold  f/F:speed  t/T:theme ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §6 app — setup, the main loop, input, and the frame cap ── */

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

        /* On resize, rebuild the geometry and re-match the dots to the digit. */
        if (g_need_resize) {
            do_resize(&cx, &cy);
            digit_assign(cur_digit, cx, cy);
        }

        int64_t frame_start = clock_ns();

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27: g_running = 0; break;
            case 'p': case 'P': case ' ': paused = !paused; break;
            case 'n': case 'N':
                /* Jump straight to the next digit: finish the morph and the hold. */
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
                /* Faster glide = fewer frames. */
                g_morph_frames -= 5;
                if (g_morph_frames < MORPH_FRAMES_MIN) g_morph_frames = MORPH_FRAMES_MIN;
                break;
            case 'F':
                /* Slower glide = more frames. */
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
            parts_update();

            /* Once the digit has formed, count down the hold then start the next. */
            if (g_morph_t >= 1.0f) {
                hold_tick++;
                if (hold_tick >= hold_max) {
                    hold_tick = 0;
                    cur_digit = (cur_digit + 1) % 10;
                    digit_assign(cur_digit, cx, cy);
                }
            }
        }

        erase();
        parts_draw(cur_digit);
        hud_draw(cur_digit, hold_max, theme, paused, fps);
        wnoutrefresh(stdscr);
        doupdate();

        /* Update the fps reading about twice a second, then sleep to cap fps. */
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
