/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/* leaf_fall.c — ASCII tree with matrix-rain leaf fall
 *
 * States: DISPLAY (2.5 s static tree) → FALLING (matrix-rain leaf drop)
 *         → RESET (brief blank) → new algorithmically varied tree
 *
 * Keys: q/ESC quit   r new tree   spc skip state
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/leaf_fall.c -o leaf_fall -lncurses -lm
 */
#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config ────────────────────────────────────────────────────────────── */
#define RENDER_FPS      30
#define RENDER_NS       (1000000000LL / RENDER_FPS)
#define FALL_NS         55000000LL    /* 55 ms per fall step (~18 fps) */
#define DISPLAY_NS      2500000000LL  /* 2.5 s static display */
#define RESET_NS        700000000LL   /* 0.7 s blank between trees */
#define TRAIL_LEN       7             /* length of green trail behind white head */
#define MAX_LEAVES      4096
#define GRID_ROWS       128
#define GRID_COLS       320
#define MAX_DEPTH       7
#define BSTACK_MAX      512
#define MAX_START_DELAY 80            /* ticks — stagger leaf fall start */

static const float TRUNK_H_MIN  = 0.45f;
static const float TRUNK_H_MAX  = 0.65f;
static const float BRANCH_SPREAD = 0.50f;

/* ── §2 clock ─────────────────────────────────────────────────────────────── */
static long long clock_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}
static void clock_sleep_ns(long long ns) {
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3 color ─────────────────────────────────────────────────────────────── */
#define CP_TRUNK  1
#define CP_BRANCH 2
#define CP_LEAF   3
#define CP_FALL_H 4   /* white head */
#define CP_FALL_G 5   /* green trail body */
#define CP_HUD    6

static void color_init(void) {
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_TRUNK,  130, -1);  /* brown */
        init_pair(CP_BRANCH, 94,  -1);  /* dark brown */
        init_pair(CP_LEAF,   82,  -1);  /* bright green */
        init_pair(CP_FALL_H, 231, -1);  /* white */
        init_pair(CP_FALL_G, 46,  -1);  /* vivid green */
        init_pair(CP_HUD,    244, -1);
    } else {
        init_pair(CP_TRUNK,  COLOR_YELLOW, -1);
        init_pair(CP_BRANCH, COLOR_YELLOW, -1);
        init_pair(CP_LEAF,   COLOR_GREEN,  -1);
        init_pair(CP_FALL_H, COLOR_WHITE,  -1);
        init_pair(CP_FALL_G, COLOR_GREEN,  -1);
        init_pair(CP_HUD,    COLOR_WHITE,  -1);
    }
}

/* ── §4 tree ──────────────────────────────────────────────────────────────── */
static char   g_grid[GRID_ROWS][GRID_COLS];
static int8_t g_gcp [GRID_ROWS][GRID_COLS];  /* color pair per cell */

typedef struct {
    int16_t orig_row, orig_col;
    char    ch;
    int16_t head_row;
    bool    started;
    bool    done;
    int16_t start_delay;   /* ticks before fall begins */
    int16_t fall_period;   /* ticks between row advances: 1=fast 2=med 3=slow */
    int16_t fall_sub;      /* tick counter within current period */
} Leaf;

static Leaf  g_leaves[MAX_LEAVES];
static int   g_leaf_count;
static int   g_rows, g_cols;
static unsigned g_seed;

/* LCG — deterministic from seed */
static unsigned lcg(void)  { g_seed = g_seed * 1664525u + 1013904223u; return g_seed; }
static float    lcgf(void) { return (float)(lcg() & 0x7fffffffu) / (float)0x7fffffffu; }

static void grid_clear(void) {
    memset(g_grid, 0, sizeof(g_grid));
    memset(g_gcp,  0, sizeof(g_gcp));
    g_leaf_count = 0;
}

static bool in_grid(int r, int c) {
    return r >= 0 && r < g_rows && r < GRID_ROWS &&
           c >= 0 && c < g_cols && c < GRID_COLS;
}

static void set_cell(int r, int c, char ch, int8_t cp) {
    if (in_grid(r, c)) { g_grid[r][c] = ch; g_gcp[r][c] = cp; }
}

static void add_leaf(int r, int c, char ch) {
    if (!in_grid(r, c))                                          return;
    if (g_gcp[r][c] == CP_TRUNK || g_gcp[r][c] == CP_BRANCH)   return;
    if (g_gcp[r][c] == CP_LEAF)                                  return;
    if (g_leaf_count >= MAX_LEAVES)                              return;
    set_cell(r, c, ch, CP_LEAF);
    g_leaves[g_leaf_count++] = (Leaf){
        .orig_row    = (int16_t)r,
        .orig_col    = (int16_t)c,
        .ch          = ch,
        .head_row    = (int16_t)r,
        .started     = false,
        .done        = false,
        .start_delay = (int16_t)(lcg() % MAX_START_DELAY),
        .fall_period = (int16_t)(1 + (int)(lcg() % 3)),  /* 1=fast 2=med 3=slow */
        .fall_sub    = 0,
    };
}

static const char k_lch[] = "*@&#%o~";

/* elliptical foliage patch — aspect-correct so it looks round on screen */
static void place_foliage(int r, int c, int rad) {
    if (rad < 1) rad = 1;
    int nlch = (int)(sizeof(k_lch) - 1);
    for (int dr = -rad; dr <= rad; dr++) {
        for (int dc = -(rad * 2); dc <= (rad * 2); dc++) {
            /* ellipse: semi-axes rad (rows) × 2*rad (cols) ≈ circle on terminal */
            float d = (float)(dr * dr) + 0.25f * (float)(dc * dc);
            if (d > (float)(rad * rad)) continue;
            /* density falls off toward edge — keep sparse */
            float thresh = 0.62f + 0.33f * d / (float)(rad * rad + 1);
            if (lcgf() > thresh) {
                char ch = k_lch[lcg() % (unsigned)nlch];
                add_leaf(r + dr, c + dc, ch);
            }
        }
    }
}

/* Bresenham line — char chosen from line direction */
static void draw_branch_line(int r0, int c0, int r1, int c1, int8_t cp) {
    int dr = abs(r1 - r0), dc = abs(c1 - c0);
    int sr = (r1 > r0) ? 1 : -1, sc = (c1 > c0) ? 1 : -1;
    int err = dc - dr;
    int r = r0, c = c0;
    char ch = (dc == 0) ? '|' :
              (dr == 0) ? '-' :
              (sr * sc > 0) ? '\\' : '/';
    for (;;) {
        /* don't overwrite thick trunk chars with thinner branch chars */
        if (in_grid(r, c) && g_gcp[r][c] != CP_TRUNK) {
            g_grid[r][c] = ch;
            g_gcp[r][c]  = cp;
        }
        if (r == r1 && c == c1) break;
        int e2 = 2 * err;
        if (e2 > -dr) { err -= dr; c += sc; }
        if (e2 <  dc) { err += dc; r += sr; }
    }
}

typedef struct { int r, c; float angle; int len; int depth; } BTask;
static BTask g_bstack[BSTACK_MAX];

static void grow_tree(int base_row, int base_col) {
    /* trunk height */
    int trunk_h = (int)((TRUNK_H_MIN + lcgf() * (TRUNK_H_MAX - TRUNK_H_MIN)) * (float)g_rows);
    if (trunk_h < 8) trunk_h = 8;
    int trunk_top = base_row - trunk_h;
    if (trunk_top < 2) { trunk_top = 2; trunk_h = base_row - trunk_top; }

    /* 2-cell wide trunk */
    for (int row = base_row; row >= trunk_top; row--) {
        set_cell(row, base_col,     '|', CP_TRUNK);
        set_cell(row, base_col + 1, '|', CP_TRUNK);
    }
    /* wider flare at base (~25% of trunk height) */
    int flare = trunk_h / 4;
    for (int row = base_row; row >= base_row - flare; row--) {
        set_cell(row, base_col - 1, '|', CP_TRUNK);
        set_cell(row, base_col + 2, '|', CP_TRUNK);
    }
    /* root buttresses at very bottom */
    set_cell(base_row, base_col - 2, '/', CP_TRUNK);
    set_cell(base_row, base_col + 3, '\\', CP_TRUNK);

    int bsp = 0;

    /* primary branch points along trunk */
    int n_pts = 3 + (int)(lcgf() * 3.0f);   /* 3–5 */
    for (int i = 0; i < n_pts && bsp < BSTACK_MAX - 2; i++) {
        float hf     = (float)(i + 1) / (float)(n_pts + 1);
        int   br     = base_row - (int)(hf * (float)trunk_h * 0.95f);
        float spread = BRANCH_SPREAD + lcgf() * 0.35f;
        /* branches are longer at mid-canopy */
        float len_f  = 0.25f + 0.35f * (1.0f - fabsf(hf - 0.5f) * 2.0f) + lcgf() * 0.10f;
        int   blen   = (int)(len_f * (float)trunk_h);
        if (blen < 3) blen = 3;
        g_bstack[bsp++] = (BTask){ br, base_col,     (float)M_PI/2.0f + spread, blen, 1 };
        g_bstack[bsp++] = (BTask){ br, base_col + 1, (float)M_PI/2.0f - spread, blen, 1 };
    }

    /* top crown */
    if (bsp < BSTACK_MAX - 3) {
        int   top_len = trunk_h / 3;
        if (top_len < 3) top_len = 3;
        float ts = 0.55f + lcgf() * 0.30f;
        g_bstack[bsp++] = (BTask){ trunk_top, base_col,     (float)M_PI/2.0f + ts,   top_len,          1 };
        g_bstack[bsp++] = (BTask){ trunk_top, base_col + 1, (float)M_PI/2.0f - ts,   top_len,          1 };
        g_bstack[bsp++] = (BTask){ trunk_top, base_col,     (float)M_PI/2.0f,         (int)(top_len*0.8f), 1 };
    }

    /* process branch stack */
    while (bsp > 0) {
        BTask t = g_bstack[--bsp];

        if (t.len <= 1 || t.depth > MAX_DEPTH) {
            int frad = 2 + (MAX_DEPTH - t.depth) / 2;
            if (frad < 1) frad = 1;
            if (frad > 5) frad = 5;
            place_foliage(t.r, t.c, frad);
            continue;
        }

        /* branch endpoint */
        int er = t.r - (int)(sinf(t.angle) * (float)t.len);
        int ec = t.c + (int)(cosf(t.angle) * (float)t.len);

        int8_t cp = (t.depth <= 2) ? CP_TRUNK : CP_BRANCH;
        draw_branch_line(t.r, t.c, er, ec, cp);

        /* sparse foliage along mid/deep branches */
        if (t.depth >= 4 && lcgf() < 0.40f)
            place_foliage(er, ec, 1);

        if (t.depth >= MAX_DEPTH) {
            place_foliage(er, ec, 3);
            continue;
        }

        float spread = 0.30f + lcgf() * 0.30f;
        int   slen   = (int)((float)t.len * (0.50f + lcgf() * 0.20f));
        if (slen < 2) slen = 2;

        if (bsp < BSTACK_MAX - 3) {
            g_bstack[bsp++] = (BTask){ er, ec, t.angle + spread, slen, t.depth + 1 };
            g_bstack[bsp++] = (BTask){ er, ec, t.angle - spread, slen, t.depth + 1 };
            /* occasional straight-ahead sub-branch */
            if (lcgf() < 0.35f && bsp < BSTACK_MAX - 1) {
                float jitter = (lcgf() - 0.5f) * 0.20f;
                g_bstack[bsp++] = (BTask){ er, ec, t.angle + jitter,
                                            (int)((float)slen * 0.80f), t.depth + 1 };
            }
        }
    }
}

/* ── §5 fall simulation ───────────────────────────────────────────────────── */
static int g_fall_tick;

static void fall_tick(void) {
    g_fall_tick++;
    for (int i = 0; i < g_leaf_count; i++) {
        Leaf *lf = &g_leaves[i];
        if (lf->done) continue;
        if (!lf->started) {
            if (g_fall_tick >= lf->start_delay) lf->started = true;
            else continue;
        }
        if (++lf->fall_sub >= lf->fall_period) {
            lf->fall_sub = 0;
            lf->head_row++;
            if (lf->head_row > (int16_t)(g_rows + TRAIL_LEN)) lf->done = true;
        }
    }
}

static bool all_done(void) {
    if (g_leaf_count == 0) return true;
    for (int i = 0; i < g_leaf_count; i++)
        if (!g_leaves[i].done) return false;
    return true;
}

/* ── §6 scene ─────────────────────────────────────────────────────────────── */
typedef enum { ST_DISPLAY, ST_FALLING, ST_RESET } State;
static State     g_state;
static long long g_state_t;
static unsigned  g_cycle;

static void scene_new_tree(void) {
    /* vary seed per cycle for algorithmic diversity */
    g_seed = (unsigned)clock_ns() ^ (g_cycle * 0x9e3779b9u);
    g_cycle++;
    grid_clear();
    g_fall_tick = 0;
    grow_tree(g_rows - 2, g_cols / 2 - 1);
    g_state   = ST_DISPLAY;
    g_state_t = clock_ns();
}

static void scene_resize(int rows, int cols) {
    g_rows = (rows > 4 && rows < GRID_ROWS) ? rows : GRID_ROWS - 1;
    g_cols = (cols > 8 && cols < GRID_COLS) ? cols : GRID_COLS - 1;
    scene_new_tree();
}

static void scene_draw(void) {
    bool falling = (g_state == ST_FALLING);

    /* trunk and branch cells */
    for (int r = 0; r < g_rows && r < GRID_ROWS; r++) {
        for (int c = 0; c < g_cols && c < GRID_COLS; c++) {
            char   ch = g_grid[r][c];
            int8_t cp = g_gcp[r][c];
            if (!ch || cp == CP_LEAF) continue;   /* leaves handled below */
            attron(COLOR_PAIR(cp) | A_BOLD);
            mvaddch(r, c, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(cp) | A_BOLD);
        }
    }

    /* leaf cells — static or falling */
    for (int i = 0; i < g_leaf_count; i++) {
        Leaf *lf = &g_leaves[i];

        if (!falling || !lf->started) {
            /* static green leaf at original position */
            attron(COLOR_PAIR(CP_LEAF) | A_BOLD);
            mvaddch(lf->orig_row, lf->orig_col, (chtype)(unsigned char)lf->ch);
            attroff(COLOR_PAIR(CP_LEAF) | A_BOLD);
            continue;
        }
        if (lf->done) continue;   /* fallen off screen — erase() cleared it */

        /* matrix-rain trail: green body + white head
         * Trail is clipped to orig_row so rain originates at the leaf position,
         * not above it. */
        for (int j = TRAIL_LEN; j >= 0; j--) {
            int r = lf->head_row - j;
            if (r < lf->orig_row || r < 0 || r >= g_rows) continue;
            if (j == 0) {
                attron(COLOR_PAIR(CP_FALL_H) | A_BOLD);
                mvaddch(r, lf->orig_col, (chtype)(unsigned char)lf->ch);
                attroff(COLOR_PAIR(CP_FALL_H) | A_BOLD);
            } else {
                attron(COLOR_PAIR(CP_FALL_G));
                mvaddch(r, lf->orig_col, (chtype)(unsigned char)lf->ch);
                attroff(COLOR_PAIR(CP_FALL_G));
            }
        }
    }
}

/* ── §7 screen ────────────────────────────────────────────────────────────── */
static void screen_init(void) {
    initscr();
    cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §8 app ───────────────────────────────────────────────────────────────── */
static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s) {
    if (s == SIGINT  || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)                g_resize = 1;
}
static void cleanup(void) { endwin(); }

int main(void) {
    atexit(cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    screen_init();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    scene_resize(rows, cols);

    long long fall_acc = 0;
    long long last_ns  = clock_ns();

    while (!g_quit) {
        if (g_resize) {
            g_resize = 0;
            getmaxyx(stdscr, rows, cols);
            scene_resize(rows, cols);
            last_ns  = clock_ns();
            fall_acc = 0;
            continue;
        }

        int ch = getch();
        if (ch == 'q' || ch == 27) break;
        if (ch == 'r') {
            scene_new_tree(); last_ns = clock_ns(); fall_acc = 0;
        }
        if (ch == ' ') {
            if (g_state == ST_DISPLAY) {
                g_state = ST_FALLING; g_state_t = clock_ns();
            } else if (g_state == ST_FALLING) {
                scene_new_tree(); last_ns = clock_ns(); fall_acc = 0;
            }
        }

        long long now_ns = clock_ns();
        long long dt     = now_ns - last_ns;
        last_ns = now_ns;
        if (dt > 100000000LL) dt = 100000000LL;

        long long elapsed = now_ns - g_state_t;

        /* state transitions */
        if (g_state == ST_DISPLAY && elapsed >= DISPLAY_NS) {
            g_state = ST_FALLING; g_state_t = now_ns;
        }
        if (g_state == ST_FALLING && all_done()) {
            g_state = ST_RESET; g_state_t = now_ns;
        }
        if (g_state == ST_RESET && elapsed >= RESET_NS) {
            scene_new_tree(); last_ns = clock_ns(); fall_acc = 0;
            continue;
        }

        /* fall accumulator */
        if (g_state == ST_FALLING) {
            fall_acc += dt;
            while (fall_acc >= FALL_NS) { fall_tick(); fall_acc -= FALL_NS; }
        }

        /* render */
        erase();
        if (g_state != ST_RESET) scene_draw();

        attron(COLOR_PAIR(CP_HUD));
        mvprintw(0, 0, " Leaf Fall  q quit  r new tree  spc skip  Cycle:%u  Leaves:%d",
                 g_cycle, g_leaf_count);
        attroff(COLOR_PAIR(CP_HUD));

        screen_present();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now_ns));
    }

    return 0;
}
