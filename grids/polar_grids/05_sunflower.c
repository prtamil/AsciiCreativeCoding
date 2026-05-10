/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * 05_sunflower.c — phyllotaxis sunflower pattern (Vogel model)
 *
 * DEMO: Seeds are placed at parametric positions r=√i × SPACING,
 *       θ = i × GOLDEN_ANGLE, producing the spiral dot pattern seen in
 *       sunflower heads, pinecones, and daisy centres.  An '@' cursor
 *       sits on the n-th seed (a single Vogel index) — arrows step n
 *       up/down.  +/- adjusts the seed spacing; [/] changes max seed count.
 *       The 'g' key cycles through slight angle deviations to show why
 *       GOLDEN_ANGLE is special.
 *
 * Study alongside: 04_log_spiral.c (continuous log-spiral arms),
 *                  03_archimedean_spiral.c (constant-pitch spiral arms),
 *                  ../rect_grids/01_uniform_rect.c (the GridCtx template)
 *
 * Section map:
 *   §1 config   — seed count, spacing, golden angle, deviation table, EWMA
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — theme-switchable PAIR_GRID + HUD/HINT/CURSOR
 *   §4 formula  — GridCtx + ctx_init / ctx_to_screen / ctx_draw_bg
 *   §5 cursor   — Cursor (Vogel index n) + cursor_reset / cursor_move / cursor_draw
 *   §6 scene    — hud_draw + scene_draw
 *   §7 screen   — ncurses init / cleanup
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit   p pause   t theme   r reset   g cycle angle deviation
 *        arrows move @   +/- seed spacing   [/] seed count
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra grids/polar_grids/05_sunflower.c \
 *       -o 05_sunflower -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Vogel's sunflower model (1979).  Seed i is placed at:
 *
 *                    r = √i × SPACING       (in pixel units)
 *                    θ = i × GOLDEN_ANGLE   GOLDEN_ANGLE = 2π / φ² ≈ 137.508°
 *
 *                  The √i spacing ensures equal area per seed (uniform packing
 *                  density).  The golden angle ensures no two seeds ever share
 *                  the same spoke — it is the "most irrational" rotation.
 *
 *                  Conversion to terminal cell:
 *                    col = ox + round((r × cos θ) / CELL_W)
 *                    row = oy + round((r × sin θ) / CELL_H)
 *
 *                  A visited[] grid prevents double-writes without a hash set.
 *
 * Data-structure : Two structs — GridCtx (terminal extent, spacing, n_seeds,
 *                  current angle, ox/oy) and Cursor — a single Vogel index n.
 *                  ctx_to_screen (n) returns the screen position of seed n.
 *
 * Math           : The golden angle α = 2π − 2π/φ = 2π(1 − 1/φ) = 2π/φ²
 *                  where φ = (1+√5)/2.  In degrees: ≈ 137.507764°.
 *
 *                  Why √i? The area of the disc up to seed i is π × r² =
 *                  π × i × SPACING².  So each seed adds the same area π×SPACING²
 *                  — exactly uniform density regardless of distance.
 *
 *                  The "angle deviation" experiment: replace GOLDEN_ANGLE with
 *                  360°/n (a rational multiple of 2π) and you get n radial
 *                  spokes with gaps between them — the pattern degenerates.
 *                  Only irrational multiples of 2π fill uniformly.
 *
 *                  visited[] prevents two seeds occupying the same terminal
 *                  cell.  Allocated on the stack (terminal cells are at most
 *                  ~250×80 ≈ 20 000 booleans).
 *
 * Rendering      : Each seed is one character (SEED_CHAR).  The cursor
 *                  highlights seed n with bright '@'.
 *
 * Performance    : O(N_SEEDS) per frame — iterate seeds, convert to cell,
 *                  write character.  N_SEEDS ≤ 4096 at terminal resolution.
 *
 * References     :
 *   Vogel model — Vogel H (1979) "A better way to construct the sunflower
 *     head" Mathematical Biosciences 44(3–4):179–189
 *   Golden angle — en.wikipedia.org/wiki/Golden_angle
 *   Phyllotaxis — en.wikipedia.org/wiki/Phyllotaxis
 *   Prusinkiewicz & Lindenmayer "The Algorithmic Beauty of Plants" (1990) ch.4
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ──────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   Seeds are placed parametrically: seed i goes to polar position
 *   (√i × SPACING, i × GOLDEN_ANGLE).  The √i radius gives uniform density;
 *   the golden angle ensures no two seeds share a spoke — ever.  The Cursor
 *   address is a single integer n — the index of one specific seed.
 *
 * HOW TO THINK ABOUT IT
 *   Imagine a turntable rotating by GOLDEN_ANGLE per tick while a dispenser
 *   drops a seed at the rim at distance √i from centre.  Because GOLDEN_ANGLE
 *   is irrational relative to 2π the seeds never "stack" on a previous spoke.
 *   After many seeds, organised Fibonacci spiral families emerge (8, 13, 21…
 *   spirals visible at different scales).
 *
 *   Press 'g' to cycle rational angles: spokes appear immediately when the
 *   angle is a rational multiple of 2π — a visual proof of irrationality.
 *
 * DRAWING METHOD
 *   1. For seed i in [0, N_SEEDS):
 *      r = √i × SPACING           ← pixel radius (√i for equal-area packing)
 *      θ = i × current_angle      ← angle in radians (unbounded; cos/sin are periodic)
 *   2. Convert to terminal cell:
 *      col = ox + round(r × cos(θ) / CELL_W)
 *      row = oy + round(r × sin(θ) / CELL_H)
 *   3. Skip if out of bounds.
 *   4. Use visited[row][col] to prevent double-writes (first seed wins).
 *   5. Draw SEED_CHAR at (col, row).
 *
 * KEY FORMULAS
 *   Golden angle: α = 2π / φ² ≈ 2.3999 rad ≈ 137.508°
 *     φ = (1+√5)/2 ≈ 1.618.  Equivalently α = 2π(1 − 1/φ).
 *
 *   Equal-area density (why √i):
 *     Area up to seed i = π × r_i² = π × i × SPACING².
 *     Each seed adds area π × SPACING² — uniform density for all i.
 *
 *   Cursor → screen (Vogel seed n):
 *     r_n     = √n × spacing
 *     theta_n = n × current_angle
 *     col_n   = ox + round(r_n × cos theta_n / CELL_W)
 *     row_n   = oy + round(r_n × sin theta_n / CELL_H)
 *
 *   Why golden angle avoids spokes:
 *     A rational angle p×2π/q closes after q seeds (modular arithmetic).
 *     GOLDEN_ANGLE is irrational: seeds never return to a prior direction.
 *     It is the "most irrational" angle (slowest-converging continued fraction).
 *
 * EDGE CASES TO WATCH
 *   • Double-write: two seeds may map to the same terminal cell.  visited[]
 *     prevents both drawing — first seed wins, second is silently skipped.
 *   • VLA stack: bool visited[rows][cols].  At 250×80 ≈ 20 000 bytes — safe.
 *     On very large terminals (>500 rows) consider heap allocation.
 *   • angle_idx=3 (222.5°): mathematically equivalent to the golden angle
 *     mod 2π — pattern looks identical to idx=0.  Included to show equivalence.
 *   • Cursor n is clamped to [0, n_seeds−1]; reset after seed-count change.
 *
 * HOW TO VERIFY
 *   SPACING=3.5, N=800, ox=40, oy=12.
 *
 *   Seed 0: r=0, θ=0  →  col=40, row=12  (centre dot)
 *   Seed 1: r=√1×3.5=3.5, θ=2.3999 rad
 *     cos(2.3999)=−0.748,  sin(2.3999)=0.664
 *     col = 40 + round(3.5×(−0.748)/2) = 40 + round(−1.31) = 39
 *     row = 12 + round(3.5×0.664/4)    = 12 + round(0.58)  = 13
 *   Seed 4: r=√4×3.5=7.0, θ=4×2.3999=9.600 rad → mod 2π ≈ 3.317 rad
 *     cos(3.317)=−0.987,  sin(3.317)=−0.163
 *     col = 40 + round(7×(−0.987)/2) = 40 − 3 = 37
 *     row = 12 + round(7×(−0.163)/4) = 12 + 0 = 12
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ═══════════════════════════════════════════════════════════════════════ */
/* §1  config                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

#define TARGET_FPS         30
#define CELL_W             2
#define CELL_H             4

/* φ and the golden angle in radians (2π/φ²) */
#define PHI                1.61803398874989484820
#define GOLDEN_ANGLE       (2.0 * M_PI / (PHI * PHI))   /* ≈ 2.3999 rad */

/* Pixel distance between successive seeds (controls overall scale) */
#define SPACING_DEFAULT    3.5
#define SPACING_MIN        1.5
#define SPACING_MAX        8.0
#define SPACING_STEP       0.5

/* Number of seeds to render */
#define N_SEEDS_DEFAULT    800
#define N_SEEDS_MIN        100
#define N_SEEDS_MAX       4096
#define N_SEEDS_STEP       100

/* Alternative angles for the 'g' experiment (rational multiples of π) */
static const double ANGLE_TABLE[] = {
    2.0 * M_PI / (PHI * PHI),    /* golden angle — perfect packing */
    2.0 * M_PI / 5.0,            /* 72° — 5 clear spokes           */
    2.0 * M_PI / 8.0,            /* 45° — 8 clear spokes           */
    2.0 * M_PI * (1.0 - 1.0/PHI), /* 222.5° — same as golden, mod 2π */
    2.0 * M_PI * 0.382,          /* near-golden but rational-ish   */
};
#define N_ANGLES  5

/* Character to draw for each seed */
#define SEED_CHAR  'o'

/* Smoothing factor for the displayed FPS readout (exponential moving avg). */
#define FPS_EWMA_ALPHA     0.05

#define PAIR_GRID    1
#define PAIR_CURSOR  2
#define PAIR_HUD     3
#define PAIR_HINT    4

static const short THEME_FG[][2] = {
    {75,  COLOR_CYAN},
    {82,  COLOR_GREEN},
    {69,  COLOR_BLUE},
    {201, COLOR_MAGENTA},
    {226, COLOR_YELLOW},
};
#define N_THEMES 5

/* ═══════════════════════════════════════════════════════════════════════ */
/* §2  clock                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec r = { .tv_sec=(time_t)(ns/1000000000LL),
                          .tv_nsec=(long)(ns%1000000000LL) };
    nanosleep(&r, NULL);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §3  color                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void color_init(int theme)
{
    start_color(); use_default_colors();
    short fg = COLORS >= 256 ? THEME_FG[theme][0] : THEME_FG[theme][1];
    init_pair(PAIR_GRID,   fg,                              -1);
    init_pair(PAIR_CURSOR, COLORS>=256 ? 226 : COLOR_YELLOW,-1);
    init_pair(PAIR_HUD,    COLORS>=256 ? 226 : COLOR_YELLOW,-1);
    init_pair(PAIR_HINT,   COLORS>=256 ?  51 : COLOR_CYAN,  -1);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §4  formula — GridCtx and the Vogel-seed ↔ screen mapping              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * GridCtx — Vogel phyllotaxis parameters plus cursor bounds.
 *
 * spacing, angle, n_seeds drive the seed placement.  max_n = n_seeds − 1.
 */
typedef struct {
    int rows, cols;

    double spacing;        /* pixel distance scale                          */
    double angle;          /* current placement angle (radians)             */
    int    n_seeds;        /* number of seeds to render                     */
    int    cell_w, cell_h;

    int    ox, oy;

    int    max_n;
} GridCtx;

/*
 * ctx_init — derive geometry from terminal size.
 *
 * No max_ring computation needed — Vogel placement self-bounds via the
 * out-of-bounds skip in ctx_draw_bg.  max_n is just n_seeds − 1.
 */
static void ctx_init(GridCtx *g, int rows, int cols)
{
    g->rows   = rows;
    g->cols   = cols;
    g->cell_w = CELL_W;
    g->cell_h = CELL_H;
    g->ox     = cols / 2;
    g->oy     = rows / 2;
    if (g->spacing <= 0.0) g->spacing = SPACING_DEFAULT;
    if (g->angle   <= 0.0) g->angle   = GOLDEN_ANGLE;
    if (g->n_seeds <= 0)   g->n_seeds = N_SEEDS_DEFAULT;
    g->max_n = g->n_seeds - 1;
}

/*
 * ctx_to_screen — terminal cell of Vogel seed n.
 *
 * THE FORMULA:
 *   r_n     = √n × spacing
 *   theta_n = n × angle
 *   col = ox + (int)round(r_n × cos(theta_n) / CELL_W)
 *   row = oy + (int)round(r_n × sin(theta_n) / CELL_H)
 *
 * This is the inverse direction of cell_to_polar from the ring/spiral files.
 * Dividing by CELL_W and CELL_H undoes the aspect-ratio scaling, so a
 * seed at pixel radius r_n lands at the correct circular position on screen.
 */
static void ctx_to_screen(const GridCtx *g, int n, int *sr, int *sc)
{
    double r_n     = sqrt((double)n) * g->spacing;
    double theta_n = (double)n * g->angle;
    *sc = g->ox + (int)round(r_n * cos(theta_n) / (double)g->cell_w);
    *sr = g->oy + (int)round(r_n * sin(theta_n) / (double)g->cell_h);
}

/*
 * ctx_draw_bg — place N seeds using Vogel's phyllotaxis formula.
 *
 * THE PIPELINE:
 *   for i in [0, n_seeds):
 *     r = √i × spacing                 equal-area radial placement
 *     θ = i × angle                    golden/rational angle rotation
 *     (col, row) ← polar→cell          aspect-corrected cell
 *     skip if out of bounds or already visited
 *     draw SEED_CHAR, mark visited[row][col]
 *
 * visited[] is a stack VLA of bool — prevents two seeds writing the same cell.
 */
static void ctx_draw_bg(const GridCtx *g)
{
    int rows = g->rows, cols = g->cols;
    /* Stack-allocated visited grid — prevents double-writing a cell */
    bool visited[rows][cols];
    memset(visited, 0, sizeof(bool) * (size_t)(rows * cols));

    attron(COLOR_PAIR(PAIR_GRID));
    for (int i = 0; i < g->n_seeds; i++) {
        double r   = sqrt((double)i) * g->spacing;
        double th  = (double)i * g->angle;
        int sc = g->ox + (int)round(r * cos(th) / (double)g->cell_w);
        int sr = g->oy + (int)round(r * sin(th) / (double)g->cell_h);

        if (sr < 0 || sr >= rows - 1 || sc < 0 || sc >= cols) continue;
        if (visited[sr][sc]) continue;
        visited[sr][sc] = true;
        mvaddch(sr, sc, (chtype)(unsigned char)SEED_CHAR);
    }
    attroff(COLOR_PAIR(PAIR_GRID));
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §5  cursor                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

/*
 * Cursor — a single Vogel index n.
 *
 * Bounds and geometry live in GridCtx.  cursor_to_screen calls
 * ctx_to_screen(g, cur->n, ...).
 */
typedef struct { int n; } Cursor;

static void cursor_reset(Cursor *cur, const GridCtx *g)
{
    cur->n = g->max_n / 4;     /* an interior seed, neither centre nor rim */
}

/*
 * cursor_move — apply (d_n) and clamp to [0, max_n].
 *
 * Arrow-key dispatch:
 *   UP    → n − CURSOR_RING_STEP (jump several seeds inward)
 *   DOWN  → n + CURSOR_RING_STEP (jump several seeds outward)
 *   LEFT  → n − 1                 (previous seed)
 *   RIGHT → n + 1                 (next seed)
 *
 * The big up/down step exists because consecutive Vogel seeds live very far
 * apart angularly; stepping by 1 looks like teleporting.  Stepping by ≈φ²
 * (the natural Fibonacci jump) follows a visible Fibonacci spiral arm.
 */
#define CURSOR_RING_STEP 8
static void cursor_move(Cursor *cur, const GridCtx *g, int d_n)
{
    int nn = cur->n + d_n;
    if (nn < 0)         nn = 0;
    if (nn > g->max_n)  nn = g->max_n;
    cur->n = nn;
}

static void cursor_draw(const Cursor *cur, const GridCtx *g)
{
    int sr, sc;
    ctx_to_screen(g, cur->n, &sr, &sc);
    if (sc >= 0 && sc < g->cols && sr >= 0 && sr < g->rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(sr, sc, (chtype)'@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §6  scene                                                               */
/* ═══════════════════════════════════════════════════════════════════════ */

static void hud_draw(const GridCtx *g, const Cursor *cur, int angle_idx,
                     int theme, bool paused, double fps)
{
    const char *angle_name = (angle_idx == 0) ? "golden" :
                             (angle_idx == 1) ? "72deg"  :
                             (angle_idx == 2) ? "45deg"  :
                             (angle_idx == 3) ? "222.5d" : "near-g";
    char buf[112];
    snprintf(buf, sizeof buf,
             " seed:%d/%d  sp:%.1f  ang:%s  th:%d  %5.1f fps  %s ",
             cur->n, g->n_seeds, g->spacing, angle_name,
             theme + 1, fps, paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, g->cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g->rows - 1, 0,
             " q:quit  p:pause  t:theme  r:reset  g:cycle-angle  arrows:move  +/-:spacing  [/]:seeds ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const GridCtx *g, const Cursor *cur, int angle_idx,
                       int theme, bool paused, double fps)
{
    erase();
    ctx_draw_bg(g);
    cursor_draw(cur, g);
    hud_draw(g, cur, angle_idx, theme, paused, fps);
    wnoutrefresh(stdscr); doupdate();
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §7  screen                                                              */
/* ═══════════════════════════════════════════════════════════════════════ */

static void screen_cleanup(void) { endwin(); }
static void screen_init(void)
{
    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    atexit(screen_cleanup);
}

/* ═══════════════════════════════════════════════════════════════════════ */
/* §8  app                                                                 */
/* ═══════════════════════════════════════════════════════════════════════ */

static volatile sig_atomic_t g_running = 1, g_need_resize = 0;
static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_running = 0;
    if (s == SIGWINCH)               g_need_resize = 1;
}

int main(void)
{
    signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
    signal(SIGWINCH, on_signal);

    int theme = 0;
    int angle_idx = 0;
    screen_init();
    color_init(theme);

    GridCtx g = {0};
    g.spacing = SPACING_DEFAULT;
    g.angle   = ANGLE_TABLE[angle_idx];
    g.n_seeds = N_SEEDS_DEFAULT;
    ctx_init(&g, LINES, COLS);

    Cursor cur;
    cursor_reset(&cur, &g);

    bool   paused = false;
    double fps    = TARGET_FPS;
    int64_t t0    = clock_ns();
    const int64_t FRAME_NS = 1000000000LL / TARGET_FPS;

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0; endwin(); refresh();
            ctx_init(&g, LINES, COLS);
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 27: g_running = 0; break;
        case 'p': paused = !paused; break;
        case 'r': cursor_reset(&cur, &g); break;
        case 't': theme = (theme + 1) % N_THEMES; color_init(theme); break;
        case 'g':
            angle_idx = (angle_idx + 1) % N_ANGLES;
            g.angle = ANGLE_TABLE[angle_idx];
            break;
        case KEY_UP:    cursor_move(&cur, &g, -CURSOR_RING_STEP); break;
        case KEY_DOWN:  cursor_move(&cur, &g, +CURSOR_RING_STEP); break;
        case KEY_LEFT:  cursor_move(&cur, &g, -1);                break;
        case KEY_RIGHT: cursor_move(&cur, &g, +1);                break;
        case '+': case '=':
            if (g.spacing < SPACING_MAX) g.spacing += SPACING_STEP;
            break;
        case '-':
            if (g.spacing > SPACING_MIN) g.spacing -= SPACING_STEP;
            break;
        case '[':
            if (g.n_seeds > N_SEEDS_MIN) {
                g.n_seeds -= N_SEEDS_STEP;
                ctx_init(&g, LINES, COLS);
                if (cur.n > g.max_n) cur.n = g.max_n;
            }
            break;
        case ']':
            if (g.n_seeds < N_SEEDS_MAX) {
                g.n_seeds += N_SEEDS_STEP;
                ctx_init(&g, LINES, COLS);
            }
            break;
        }

        int64_t now = clock_ns();
        fps = fps * (1.0 - FPS_EWMA_ALPHA) +
              (1e9 / (double)(now - t0 + 1)) * FPS_EWMA_ALPHA;
        t0  = now;
        if (!paused)
            scene_draw(&g, &cur, angle_idx, theme, paused, fps);
        clock_sleep_ns(FRAME_NS - (clock_ns() - now));
    }
    return 0;
}
