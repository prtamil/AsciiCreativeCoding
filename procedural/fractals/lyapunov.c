/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lyapunov.c — Lyapunov fractal
 *
 * For each terminal cell (col, row) the logistic map is iterated with a
 * parameter r that alternates between two values a and b according to a
 * chosen symbol sequence (e.g. "AB", "AAB", "AABAB" …).  The Lyapunov
 * exponent λ = (1/N) Σ ln|r·(1−2x)| classifies the pixel:
 *
 *   λ < 0  → stable (blues / cyans)
 *   λ > 0  → chaotic (yellows / reds)
 *   λ outside [−4, 2.5] → blank (black background)
 *
 * Six sequences cycle automatically.  Rendering is progressive: two rows
 * are computed per frame so the image builds top-to-bottom.  On sequence
 * change or terminal resize the buffer is cleared and rendering restarts.
 *
 * Keys:
 *   q / ESC        quit
 *   p / space      pause / resume
 *   s              next sequence
 *   S              prev sequence
 *   r              reset / restart current sequence
 *   t              next theme
 *   T              prev theme
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra lyapunov.c -o lyapunov -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  compute
 *   §5  render
 *   §6  hud
 *   §7  app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Lyapunov exponent measurement via logistic map iteration.
 *                  For each pixel (a, b), iterate the logistic map using a
 *                  sequence string that alternates between r=a and r=b:
 *                    x_{n+1} = r_n · xₙ · (1−xₙ)
 *                  The Lyapunov exponent λ = (1/N) Σ ln|df/dx| = (1/N) Σ ln|r(1−2x)|
 *                  measures the average rate of divergence of nearby trajectories.
 *
 * Math           : λ < 0: stable (orbit converges to a fixed point or cycle).
 *                  λ = 0: bifurcation boundary (marginally stable).
 *                  λ > 0: chaotic (nearby orbits diverge exponentially at rate eˡ).
 *                  The fractal boundary between stable (blue) and chaotic (red)
 *                  regions is the "Markus-Lyapunov fractal" (Markus, 1990).
 *                  Different symbol sequences (AB, AAB, AABAB …) produce different
 *                  fractal patterns because the alternation pattern of r_a and r_b
 *                  changes the effective "average" dynamics.
 *
 * Performance    : Two rows per frame (progressive rendering), so the image builds
 *                  top-to-bottom in ~H/2 frames.  N_TRANSIENT iterations are
 *                  discarded before measuring λ to remove initial-condition effects.
 * ─────────────────────────────────────────────────────────────────────── */

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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define GRID_ROWS_MAX  80
#define GRID_COLS_MAX  300

#define A_MIN  2.5
#define A_MAX  4.0
#define B_MIN  2.5
#define B_MAX  4.0

#define WARMUP_ITERS    100
#define LYAP_ITERS      200
#define ROWS_PER_FRAME    2

/* Lyapunov exponent clamp range */
#define LEV_MIN  (-4.0)
#define LEV_MAX    2.5

/* Auto-advance: hold this many frames when render is complete */
#define DONE_HOLD_FRAMES  150

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define FRAME_NS    (NS_PER_SEC / 30)

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Color pair IDs:
 *   CP_HUD = 1   HUD text
 *   CP_S1  = 2   barely stable   |λ| < 0.5
 *   CP_S2  = 3   stable          |λ| < 1.5
 *   CP_S3  = 4   more stable     |λ| < 3.0
 *   CP_S4  = 5   very stable
 *   CP_C1  = 6   barely chaotic  λ < 0.4
 *   CP_C2  = 7   chaotic         λ < 1.0
 *   CP_C3  = 8   more chaotic    λ < 2.0
 *   CP_C4  = 9   very chaotic
 */
enum {
    CP_HUD = 1,
    CP_S1  = 2,
    CP_S2  = 3,
    CP_S3  = 4,
    CP_S4  = 5,
    CP_C1  = 6,
    CP_C2  = 7,
    CP_C3  = 8,
    CP_C4  = 9,
};

#define N_THEMES  2

typedef struct {
    const char *name;
    /* stable fg [0..3] = S1..S4, chaos fg [4..7] = C1..C4  (256-color) */
    int fg256[8];
    /* 8-color fallbacks */
    int s_fg8[4];
    int c_fg8[4];
} Theme;

static const Theme k_themes[N_THEMES] = {
    {
        "Classic",
        { 51, 33, 21, 17,   226, 208, 196, 124 },
        { COLOR_CYAN,  COLOR_CYAN,  COLOR_BLUE,  COLOR_BLUE  },
        { COLOR_YELLOW,COLOR_YELLOW,COLOR_RED,   COLOR_RED   },
    },
    {
        "Neon",
        { 46, 34, 28, 22,   165, 129, 93, 54 },
        { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN },
        { COLOR_MAGENTA,COLOR_MAGENTA,COLOR_MAGENTA,COLOR_MAGENTA },
    },
};

static int g_theme = 0;

static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    /* HUD always bright cyan */
    if (COLORS >= 256) {
        init_pair(CP_HUD, 51,        COLOR_BLACK);
        init_pair(CP_S1,  th->fg256[0], COLOR_BLACK);
        init_pair(CP_S2,  th->fg256[1], COLOR_BLACK);
        init_pair(CP_S3,  th->fg256[2], COLOR_BLACK);
        init_pair(CP_S4,  th->fg256[3], COLOR_BLACK);
        init_pair(CP_C1,  th->fg256[4], COLOR_BLACK);
        init_pair(CP_C2,  th->fg256[5], COLOR_BLACK);
        init_pair(CP_C3,  th->fg256[6], COLOR_BLACK);
        init_pair(CP_C4,  th->fg256[7], COLOR_BLACK);
    } else {
        init_pair(CP_HUD, COLOR_CYAN,     COLOR_BLACK);
        init_pair(CP_S1,  th->s_fg8[0],  COLOR_BLACK);
        init_pair(CP_S2,  th->s_fg8[1],  COLOR_BLACK);
        init_pair(CP_S3,  th->s_fg8[2],  COLOR_BLACK);
        init_pair(CP_S4,  th->s_fg8[3],  COLOR_BLACK);
        init_pair(CP_C1,  th->c_fg8[0],  COLOR_BLACK);
        init_pair(CP_C2,  th->c_fg8[1],  COLOR_BLACK);
        init_pair(CP_C3,  th->c_fg8[2],  COLOR_BLACK);
        init_pair(CP_C4,  th->c_fg8[3],  COLOR_BLACK);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    theme_apply(g_theme);
}

/* ===================================================================== */
/* §4  compute                                                            */
/* ===================================================================== */

#define N_SEQUENCES  6

static const char *k_sequences[N_SEQUENCES] = {
    "AB",
    "AAB",
    "ABB",
    "AABB",
    "AABAB",
    "BBBAAAB",
};

/*
 * g_lev[row][col]: encoded level for each cell.
 *   INT8_MIN = not yet computed (or clipped / black)
 *   -4 .. -1 = stable level (S4..S1, absolute value gives bucket)
 *   +1 .. +4 = chaotic level (C1..C4)
 *   0        = explicitly skipped (draw nothing)
 */
static int8_t  g_lev[GRID_ROWS_MAX][GRID_COLS_MAX];
static int     g_cur_row = 0;
static int     g_seq_idx = 0;
static int     g_rows    = 24;
static int     g_cols    = 80;
static int     g_done_frames = 0;

/*
 * lyapunov_exponent — compute λ for parameter pair (a, b) and sequence seq.
 * Returns a value in roughly [−4, 2.5] for meaningful pixels, or a large
 * positive/negative number for divergent/trivial points.
 */
static double lyapunov_exponent(double a, double b, const char *seq)
{
    int slen = (int)strlen(seq);
    double x = 0.5;
    /* warmup */
    for (int i = 0; i < WARMUP_ITERS; i++) {
        double r = (seq[i % slen] == 'A') ? a : b;
        x = r * x * (1.0 - x);
        if (x <= 0.0 || x >= 1.0) return 10.0; /* diverged */
    }
    /* accumulate */
    double lam = 0.0;
    for (int i = 0; i < LYAP_ITERS; i++) {
        double r = (seq[i % slen] == 'A') ? a : b;
        x = r * x * (1.0 - x);
        if (x <= 0.0 || x >= 1.0) return 10.0;
        double deriv = r * (1.0 - 2.0 * x);
        double ab = fabs(deriv);
        if (ab < 1e-15) ab = 1e-15;
        lam += log(ab);
    }
    return lam / LYAP_ITERS;
}

/*
 * level_encode — map λ to a signed bucket:
 *   stable  (λ<0): -1..-4  (−1 = barely stable, −4 = very stable)
 *   chaotic (λ>0):  1.. 4  ( 1 = barely chaotic,  4 = very chaotic)
 *   clipped or zero: 0
 */
static int8_t level_encode(double lam)
{
    if (lam >= LEV_MAX || lam <= LEV_MIN) return 0;
    if (lam < 0.0) {
        double al = -lam;
        if (al < 0.5)  return -1;
        if (al < 1.5)  return -2;
        if (al < 3.0)  return -3;
        return -4;
    }
    /* λ > 0 */
    if (lam < 0.4)  return 1;
    if (lam < 1.0)  return 2;
    if (lam < 2.0)  return 3;
    return 4;
}

/* Compute ROWS_PER_FRAME rows of pixels */
static void compute_rows(void)
{
    int draw_rows = g_rows - 1; /* reserve last row for HUD */
    if (draw_rows < 1) draw_rows = 1;
    const char *seq = k_sequences[g_seq_idx];

    for (int rr = 0; rr < ROWS_PER_FRAME; rr++) {
        int row = g_cur_row + rr;
        if (row >= draw_rows) return;

        double b = B_MAX - (double)row / (double)(draw_rows - 1) * (B_MAX - B_MIN);

        for (int col = 0; col < g_cols && col < GRID_COLS_MAX; col++) {
            double a = A_MIN + (double)col / (double)(g_cols - 1) * (A_MAX - A_MIN);
            double lam = lyapunov_exponent(a, b, seq);
            g_lev[row][col] = level_encode(lam);
        }
    }
    g_cur_row += ROWS_PER_FRAME;
}

/* Reset buffer and restart rendering */
static void reset_render(void)
{
    memset(g_lev, (int8_t)(-127), sizeof g_lev);
    g_cur_row    = 0;
    g_done_frames = 0;
}

/* ===================================================================== */
/* §5  render                                                             */
/* ===================================================================== */

/*
 * Stable:  S4='.', S3=':', S2='+', S1='#'
 * Chaotic: C1='@', C2='#', C3='+', C4=':'
 */
static void render_grid(void)
{
    static const int   s_cp[4] = { CP_S4, CP_S3, CP_S2, CP_S1 };
    static const char  s_ch[4] = { '.',   ':',   '+',   '#'    };
    static const int   c_cp[4] = { CP_C1, CP_C2, CP_C3, CP_C4  };
    static const char  c_ch[4] = { '@',   '#',   '+',   ':'    };

    int draw_rows = g_rows - 1;

    for (int row = 0; row < draw_rows && row < GRID_ROWS_MAX; row++) {
        for (int col = 0; col < g_cols && col < GRID_COLS_MAX; col++) {
            int8_t lv = g_lev[row][col];
            if (lv == (int8_t)(-127) || lv == 0) continue;

            int cp;
            chtype ch;
            if (lv < 0) {
                int idx = (-lv) - 1;  /* 0..3 */
                if (idx > 3) idx = 3;
                cp = s_cp[idx];
                ch = (chtype)(unsigned char)s_ch[idx];
            } else {
                int idx = lv - 1;     /* 0..3 */
                if (idx > 3) idx = 3;
                cp = c_cp[idx];
                ch = (chtype)(unsigned char)c_ch[idx];
            }
            attron(COLOR_PAIR(cp));
            mvaddch(row, col, ch);
            attroff(COLOR_PAIR(cp));
        }
    }
}

/* ===================================================================== */
/* §6  hud                                                                */
/* ===================================================================== */

static void draw_hud(double fps)
{
    int draw_rows = g_rows - 1;
    if (draw_rows < 0) draw_rows = 0;
    int hud_row = g_rows - 1;

    /* progress bar — width of terminal minus label space */
    int bar_w = g_cols - 52;
    if (bar_w < 4) bar_w = 4;
    int filled = (draw_rows > 0)
               ? (g_cur_row * bar_w / draw_rows)
               : bar_w;
    if (filled > bar_w) filled = bar_w;

    char bar[GRID_COLS_MAX + 1];
    int bi = 0;
    bar[bi++] = '[';
    for (int i = 0; i < bar_w; i++)
        bar[bi++] = (i < filled) ? '#' : '.';
    bar[bi++] = ']';
    bar[bi]   = '\0';

    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(hud_row, 0,
             " seq:%-8s %s  theme:%-8s  %5.1f fps",
             k_sequences[g_seq_idx],
             bar,
             k_themes[g_theme].name,
             fps);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* ===================================================================== */
/* §7  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_pause  = 0;

static void sig_h(int s)
{
    if (s == SIGINT  || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)                g_resize = 1;
}

static void cleanup(void) { endwin(); }

static void do_resize(void)
{
    endwin();
    refresh();
    getmaxyx(stdscr, g_rows, g_cols);
    if (g_rows > GRID_ROWS_MAX) g_rows = GRID_ROWS_MAX;
    if (g_cols > GRID_COLS_MAX) g_cols = GRID_COLS_MAX;
    reset_render();
    g_resize = 0;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    initscr();
    cbreak();
    noecho();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();

    getmaxyx(stdscr, g_rows, g_cols);
    if (g_rows > GRID_ROWS_MAX) g_rows = GRID_ROWS_MAX;
    if (g_cols > GRID_COLS_MAX) g_cols = GRID_COLS_MAX;
    reset_render();

    long long fps_accum   = 0;
    int       fps_count   = 0;
    double    fps_display = 0.0;

    while (!g_quit) {

        if (g_resize) do_resize();

        long long frame_start = clock_ns();

        /* Input */
        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': case 27:
                g_quit = 1;
                break;
            case 'p': case ' ':
                g_pause = !g_pause;
                break;
            case 's':
                g_seq_idx = (g_seq_idx + 1) % N_SEQUENCES;
                reset_render();
                break;
            case 'S':
                g_seq_idx = (g_seq_idx - 1 + N_SEQUENCES) % N_SEQUENCES;
                reset_render();
                break;
            case 'r': case 'R':
                reset_render();
                break;
            case 't':
                g_theme = (g_theme + 1) % N_THEMES;
                theme_apply(g_theme);
                break;
            case 'T':
                g_theme = (g_theme - 1 + N_THEMES) % N_THEMES;
                theme_apply(g_theme);
                break;
            default:
                break;
            }
        }

        /* Simulation */
        if (!g_pause) {
            int draw_rows = g_rows - 1;
            if (draw_rows < 1) draw_rows = 1;

            if (g_cur_row < draw_rows) {
                compute_rows();
            } else {
                /* Rendering complete — hold then auto-advance */
                g_done_frames++;
                if (g_done_frames >= DONE_HOLD_FRAMES) {
                    g_seq_idx = (g_seq_idx + 1) % N_SEQUENCES;
                    reset_render();
                }
            }
        }

        /* Draw */
        erase();
        render_grid();
        draw_hud(fps_display);
        wnoutrefresh(stdscr);
        doupdate();

        /* FPS accounting */
        long long frame_end = clock_ns();
        long long frame_dur = frame_end - frame_start;
        fps_accum += frame_dur;
        fps_count++;
        if (fps_accum >= 500 * NS_PER_MS) {
            fps_display = (double)fps_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            fps_accum = 0;
            fps_count = 0;
        }

        clock_sleep_ns(FRAME_NS - frame_dur);
    }

    return 0;
}
