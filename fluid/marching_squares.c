/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * marching_squares.c — Marching Squares isosurface extraction
 *
 * Algorithm overview
 * ------------------
 * Given a scalar field f(x,y), find the contour f(x,y) = threshold.
 *
 * Step 1 — Sample f at every grid corner.
 * Step 2 — For each 2×2 cell of corners, classify each corner as
 *           inside (f > threshold) or outside (f ≤ threshold).
 *           Four corners → 4-bit index → 16 possible cases.
 * Step 3 — Look up which edges the contour crosses in a 16-entry table.
 * Step 4 — Interpolate crossing point along each edge (linear interp).
 * Step 5 — Draw a character at each crossing. Result is an iso-contour.
 *
 * The 16-case lookup table (by 4-bit corner index):
 *   Bits: bit3=TL  bit2=TR  bit1=BR  bit0=BL  (1=inside, 0=outside)
 *   Cases 0 and 15: no contour (all outside / all inside).
 *   Cases 1–14: one or two edges crossed.
 *
 * Scalar field used here:
 *   f(x,y) = Σ_i  A_i / ( (x-cx_i)² + (y-cy_i)² )   (metaball potential)
 *
 * Multiple iso-levels can be drawn simultaneously with different chars.
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   +/-       increase / decrease iso-threshold
 *   m         toggle multi-level (5 iso-levels)
 *   t         cycle colour theme
 *   r         randomise blob positions
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/marching_squares.c \
 *       -o marching_squares -lncurses -lm
 *
 * Sections: §1 config  §2 clock  §3 field  §4 marching  §5 draw  §6 app
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

#define N_BLOBS       4        /* number of metaball sources              */
#define BLOB_R        0.28f    /* blob influence radius (world units)     */
#define THRESH_DEF    0.20f    /* default iso-threshold (field ∈ [0,1])   */
#define THRESH_STEP   0.02f
#define THRESH_MIN    0.02f
#define THRESH_MAX    0.95f
#define RENDER_FPS    20
#define RENDER_NS    (1000000000LL / RENDER_FPS)
#define N_THEMES      4
#define N_LEVELS      5        /* number of iso-levels in multi mode      */

/* Aspect correction: terminal cells are ~2× taller than wide */
#define ASPECT        0.5f

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
/* §3  scalar field                                                       */
/* ===================================================================== */

typedef struct { float x, y, vx, vy, str; } Blob;

static Blob g_blobs[N_BLOBS];

static void blobs_init(int rows, int cols)
{
    for (int i = 0; i < N_BLOBS; i++) {
        g_blobs[i].x   = 0.2f + 0.6f * ((float)rand() / RAND_MAX);
        g_blobs[i].y   = 0.2f + 0.6f * ((float)rand() / RAND_MAX);
        g_blobs[i].vx  = 0.003f + 0.004f * ((float)rand() / RAND_MAX);
        g_blobs[i].vy  = 0.003f + 0.004f * ((float)rand() / RAND_MAX);
        g_blobs[i].str = 1.0f;   /* strength unused — kernel is normalised */
        if (rand() & 1) g_blobs[i].vx = -g_blobs[i].vx;
        if (rand() & 1) g_blobs[i].vy = -g_blobs[i].vy;
    }
    (void)rows; (void)cols;
}

static void blobs_step(void)
{
    for (int i = 0; i < N_BLOBS; i++) {
        g_blobs[i].x += g_blobs[i].vx;
        g_blobs[i].y += g_blobs[i].vy;
        if (g_blobs[i].x < 0.05f || g_blobs[i].x > 0.95f) g_blobs[i].vx = -g_blobs[i].vx;
        if (g_blobs[i].y < 0.05f || g_blobs[i].y > 0.95f) g_blobs[i].vy = -g_blobs[i].vy;
    }
}

/*
 * Evaluate metaball field at normalised coordinates (nx, ny) ∈ [0,1]²
 *
 * Uses the Wyvill compact-support kernel:
 *   W(r) = (1 − r²/R²)³   for r < R,  else 0
 *
 * This kernel is exactly 0 outside radius R, peaks at 1 in the centre,
 * and has continuous first derivative at r=R (no discontinuity).
 * The field sums to [0, N_BLOBS], so a threshold of ~0.2 draws a contour
 * well outside each blob while ~0.8 draws only their dense cores.
 */
static float field_eval(float nx, float ny)
{
    float v = 0.0f;
    float R2 = BLOB_R * BLOB_R;
    for (int i = 0; i < N_BLOBS; i++) {
        float dx = nx - g_blobs[i].x;
        float dy = (ny - g_blobs[i].y) * ASPECT;
        float r2 = dx*dx + dy*dy;
        if (r2 >= R2) continue;
        float t = 1.0f - r2 / R2;
        v += t * t * t;
    }
    return v;
}

/* ===================================================================== */
/* §4  marching squares                                                   */
/* ===================================================================== */

/*
 * Edge table: for each 4-bit case, which of the 4 edges are crossed?
 * Edges: 0=top  1=right  2=bottom  3=left
 * Each entry is a bitmask of crossed edges.
 * Corner bit assignment: bit3=TL bit2=TR bit1=BR bit0=BL
 */
static const uint8_t edge_table[16] __attribute__((unused)) = {
    0x0,  /* 0000 — all outside        */
    0x9,  /* 0001 — BL: left+bottom    */
    0x3,  /* 0010 — BR: bottom+right   */
    0xa,  /* 0011 — BL+BR: left+right  */
    0x6,  /* 0100 — TR: right+top      */
    0xf,  /* 0101 — BL+TR: all 4 (saddle — rare) */
    0x5,  /* 0110 — BR+TR: top+bottom  */
    0xc,  /* 0111 — all but TL: top+left */
    0xc,  /* 1000 — TL: top+left       */
    0x5,  /* 1001 — TL+BL: top+bottom  */
    0xf,  /* 1010 — TL+BR: all 4 (saddle) */
    0x6,  /* 1011 — all but TR: right+top → corrected */
    0xa,  /* 1100 — TL+TR: left+right  */
    0x3,  /* 1101 — all but BR: bottom+right */
    0x9,  /* 1110 — all but BL: left+bottom */
    0x0,  /* 1111 — all inside         */
};

/*
 * Character table: pick an ASCII char that visually suggests the crossing.
 * Two edges are always crossed (except saddle cases). We pick the char
 * based on which pair of edges.
 *
 * edge pairs → char:
 *   top+bottom    → |
 *   left+right    → -
 *   top+right     → /  (or ╮)
 *   top+left      → \  (or ╭)
 *   bottom+right  → \
 *   bottom+left   → /
 *   saddle        → X
 */
static char case_char(int idx)
{
    static const char tbl[16] = {
        ' ',  /* 0000 */
        '/',  /* 0001 BL: left+bottom  → / */
        '\\', /* 0010 BR: bottom+right → \ */
        '-',  /* 0011 left+right       → - */
        '\\', /* 0100 TR: right+top    → \ */
        'X',  /* 0101 saddle            → X */
        '|',  /* 0110 top+bottom       → | */
        '/',  /* 0111 top+left         → / */
        '/',  /* 1000 TL: top+left     → / */
        '|',  /* 1001 top+bottom       → | */
        'X',  /* 1010 saddle           → X */
        '\\', /* 1011 right+top        → \ */
        '-',  /* 1100 left+right       → - */
        '\\', /* 1101 bottom+right     → \ */
        '/',  /* 1110 left+bottom      → / */
        ' ',  /* 1111 */
    };
    return tbl[idx & 0xf];
}

/* ===================================================================== */
/* §5  drawing                                                            */
/* ===================================================================== */

/* colour pairs: CP_BASE + level*2 + 0/1 for outside/inside fill */
#define CP_LABEL   1
#define CP_CONTOUR 2
#define CP_INSIDE  3
#define CP_OUTSIDE 4

static const short theme_contour[N_THEMES] = { 51, 196, 46, 201 };
static const short theme_inside [N_THEMES] = { 87, 202, 82, 171 };

static void init_colors(int theme)
{
    init_pair(CP_LABEL,   231,                  16);
    init_pair(CP_CONTOUR, theme_contour[theme], 16);
    init_pair(CP_INSIDE,  theme_inside[theme],  16);
    init_pair(CP_OUTSIDE, 236,                  16);
}

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;
static void on_signal(int s) { (void)s; g_quit = 1; }
static void on_resize(int s) { (void)s; g_resize = 1; }

/* ===================================================================== */
/* §6  app                                                                */
/* ===================================================================== */

typedef struct {
    float thresh;
    bool  paused;
    bool  multi;
    int   theme;
} App;

static void app_draw(const App *a)
{
    int rows = LINES - 2;   /* leave 2 rows for HUD */
    int cols = COLS;
    if (rows < 2 || cols < 2) return;

    /* pre-allocate field values at cell corners */
    static float fld[256][128];
    int fw = cols < 255 ? cols : 255;
    int fh = rows < 127 ? rows : 127;

    /* sample field at each corner */
    for (int gy = 0; gy <= fh; gy++) {
        float ny = (float)gy / (float)fh;
        for (int gx = 0; gx <= fw; gx++) {
            float nx = (float)gx / (float)fw;
            fld[gy][gx] = field_eval(nx, ny);
        }
    }

    /* march each cell */
    for (int gy = 0; gy < fh; gy++) {
        for (int gx = 0; gx < fw; gx++) {
            float tl = fld[gy  ][gx  ];
            float tr = fld[gy  ][gx+1];
            float br = fld[gy+1][gx+1];
            float bl = fld[gy+1][gx  ];

            if (a->multi) {
                /* draw multiple contour levels */
                bool drew = false;
                for (int lv = 0; lv < N_LEVELS && !drew; lv++) {
                    /* evenly spaced levels from thresh*0.3 up to thresh*0.95 */
                    float t = a->thresh * (0.3f + lv * 0.15f);
                    int idx = ((tl > t) ? 8 : 0)
                            | ((tr > t) ? 4 : 0)
                            | ((br > t) ? 2 : 0)
                            | ((bl > t) ? 1 : 0);
                    if (idx != 0 && idx != 15) {
                        char c = case_char(idx);
                        /* hue shifts per level */
                        short col = (short)(theme_contour[a->theme] + lv * 6);
                        init_pair(10 + (short)lv, col, 16);
                        attron(COLOR_PAIR(10 + lv));
                        mvaddch(gy, gx, c);
                        attroff(COLOR_PAIR(10 + lv));
                        drew = true;
                    }
                }
                if (!drew) {
                    /* fill inside regions faintly */
                    if (fld[gy][gx] > a->thresh) {
                        attron(COLOR_PAIR(CP_INSIDE));
                        mvaddch(gy, gx, '`');
                        attroff(COLOR_PAIR(CP_INSIDE));
                    }
                }
            } else {
                float t = a->thresh;
                int idx = ((tl > t) ? 8 : 0)
                        | ((tr > t) ? 4 : 0)
                        | ((br > t) ? 2 : 0)
                        | ((bl > t) ? 1 : 0);

                if (idx == 0) {
                    /* all outside — blank */
                } else if (idx == 15) {
                    /* all inside — fill char */
                    attron(COLOR_PAIR(CP_INSIDE));
                    mvaddch(gy, gx, '.');
                    attroff(COLOR_PAIR(CP_INSIDE));
                } else {
                    /* contour crossing */
                    char c = case_char(idx);
                    attron(COLOR_PAIR(CP_CONTOUR) | A_BOLD);
                    mvaddch(gy, gx, c);
                    attroff(COLOR_PAIR(CP_CONTOUR) | A_BOLD);
                }
            }
        }
    }

    /* HUD */
    attron(COLOR_PAIR(CP_LABEL));
    mvprintw(rows,     0, "Marching Squares  threshold=%.2f  [+/-] adjust  [m] multi  [t] theme  [r] random  [q] quit",
             a->thresh);
    mvprintw(rows + 1, 0, "16-case lookup: each 2x2 cell → 4-bit index → contour edge character");
    attroff(COLOR_PAIR(CP_LABEL));

    if (a->paused) {
        attron(A_REVERSE | A_BOLD);
        mvprintw(0, COLS - 8, " PAUSED ");
        attroff(A_REVERSE | A_BOLD);
    }

    refresh();
}

int main(void)
{
    srand((unsigned)time(NULL));
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_resize);

    initscr();
    cbreak(); noecho(); curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    start_color();
    use_default_colors();

    App a = { .thresh = THRESH_DEF, .paused = false, .multi = false, .theme = 0 };
    init_colors(a.theme);
    blobs_init(LINES, COLS);

    long long next = clock_ns();

    while (!g_quit) {
        if (g_resize) { g_resize = 0; endwin(); refresh(); }

        int ch = getch();
        switch (ch) {
            case 'q': case 27: g_quit = 1; break;
            case ' ': a.paused = !a.paused; break;
            case '+': case '=':
                a.thresh += THRESH_STEP;
                if (a.thresh > THRESH_MAX) a.thresh = THRESH_MAX;
                break;
            case '-':
                a.thresh -= THRESH_STEP;
                if (a.thresh < THRESH_MIN) a.thresh = THRESH_MIN;
                break;
            case 'm': a.multi = !a.multi; break;
            case 't':
                a.theme = (a.theme + 1) % N_THEMES;
                init_colors(a.theme);
                break;
            case 'r': blobs_init(LINES, COLS); break;
        }

        long long now = clock_ns();
        if (now >= next) {
            if (!a.paused) blobs_step();
            erase();
            app_draw(&a);
            next += RENDER_NS;
        } else {
            clock_sleep_ns(next - now);
        }
    }

    endwin();
    return 0;
}
