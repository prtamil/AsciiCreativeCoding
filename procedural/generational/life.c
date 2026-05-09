/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * life.c — Conway's Game of Life and Rule Variants
 *
 * Toroidal grid; rules expressed as B/S bitmasks (bit N set = birth/survive
 * when neighbour count is N).  Six variants cycle with n/p:
 *
 *   Conway    B3/S23      — the classic
 *   HighLife  B36/S23     — gliders that self-replicate
 *   Day&Night B3678/S34678— symmetric; same rule alive/dead
 *   Seeds     B2/S        — every cell dies; explosions from any pair
 *   Morley    B368/S245   — chaotic; moving structures
 *   2×2       B36/S125    — tiles of 2×2 blocks grow and divide
 *
 * Seeding:  r random · g glider · G Gosper gun · p R-pentomino · a acorn
 *
 * Layout:
 *   Rows 0 … n-5   — grid (toroidal)
 *   Rows n-4 … n-2 — scrolling population histogram (3 rows, bar chart)
 *   Row  n-1       — HUD
 *
 * Keys:
 *   n / p       next / previous rule
 *   r           random seed (~30 % density)
 *   g           single glider at centre
 *   G           Gosper glider gun
 *   p           R-pentomino
 *   a           Acorn
 *   + / =       more steps per frame
 *   - / _       fewer steps per frame
 *   space       pause / resume
 *   q / Q       quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/life.c -o life -lncurses
 *
 * Sections: §1 config  §2 clock  §3 color  §4 grid  §5 scene
 *           §6 screen  §7 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Conway's Game of Life and variants using B/S rule bitmasks.
 *                  Birth/survival encoded as bit N = 1 in a uint16 mask where
 *                  N is the neighbour count (0–8).  This allows checking any
 *                  rule with a single bitshift: born = (birth>>n)&1.
 *                  Double-buffered toroidal grid; update is O(rows × cols).
 *
 * Physics        : Emergent complexity from local rules (Conway 1970):
 *                  despite only two states and 9 neighbour values, Life
 *                  supports stable structures (still-lifes), oscillators,
 *                  spaceships (gliders), and Turing-complete computation
 *                  (Gosper gun produces infinite glider stream).
 *
 * Data-structure : Population histogram ring buffer — last HIST_LEN
 *                  generation counts stored; drawn as a 3-row bar chart
 *                  using a 4-level block character ramp (▁▂▄█).
 *
 * Performance    : Per-step cost: O(rows × cols) neighbour-count evaluation.
 *                  Steps per frame adjustable (1–16) to allow fast-forward
 *                  to interesting evolved states (e.g., Gosper gun period 30).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each cell looks at its 8 neighbours, counts how many are alive, and
 * asks one question — "with this many neighbours, do I live next tick?"
 * The answer is read out of a 9-bit lookup: bit N of the BIRTH mask
 * decides what an empty cell does, bit N of the SURVIVE mask decides
 * what a live cell does.  Different masks = different universes.
 * Conway's B3/S23 is just one of countless rules; flipping bits gets
 * you HighLife, Day&Night, Seeds, Morley, 2x2 — same code, same loop,
 * different cosmologies.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine an infinite chessboard wrapped into a torus.  Each square
 * holds a stone or doesn't.  Once a tick, every square simultaneously
 * peeks at its 8 surrounding squares and decides — based on how
 * crowded it is — whether to keep its stone, drop one, or pick one up.
 * No square ever cheats by reading the new state of a neighbour; the
 * old board is read, the new board is written, and they swap.  Two
 * boards, ping-pong.  That's it.  The astonishing complexity is the
 * universe's bonus, not your code's burden.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Pick rule (Conway, HighLife, …).  Encode birth/survive as 9-bit
 *     masks: bit N set iff cell with N neighbours is born / survives.
 *  2. For each cell (r,c) in the active board: sum the 8 toroidal
 *     neighbours; (r±1, c±1) wrap with `% rows / % cols`.
 *  3. Form bit = 1u << n.  Next state =
 *        cur ? (survive & bit) ? 1 : 0
 *            : (birth   & bit) ? 1 : 0;
 *     Write into the OTHER board.
 *  4. Swap boards (`g_buf = 1 - g_buf`); increment generation; record
 *     population in the ring history buffer.
 *  5. Repeat steps 2-4 `g_steps` times per frame for fast-forward.
 *  6. Render: draw '#' for live cells (top g_ca_rows rows), draw the
 *     scrolling population histogram (3 rows), HUD on bottom row.
 *
 * KEY FORMULAS
 * ────────────
 *  n = sum over 8 neighbours of g_grid[buf][r±1 mod R][c±1 mod C]
 *                                                   neighbour count 0..8
 *  bit = 1u << n                                    9-bit position flag
 *  next = cur ? (S & bit) != 0
 *             : (B & bit) != 0                      Conway's rule
 *  Conway:    B = (1<<3),                           B3 — birth on 3
 *             S = (1<<2)|(1<<3)                     S23 — survive on 2 or 3
 *  toroidal:  rp = (r-1+R) % R, rn = (r+1) % R      no edge cells
 *  hist bar:  level = pop / (R·C) · 2·HIST_ROWS     2 sub-levels per row
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Two-board read/write — if you forget to write into the OTHER
 *    buffer, neighbour counts read partially-updated state and the
 *    simulation collapses (gliders smear, oscillators die).
 *  • Toroidal wrap with C-style %  — `(r-1) % R` is negative for r=0;
 *    must add R before mod, hence `(r-1+R) % R`.
 *  • Seeds rule (B2/S0) — every live cell dies every tick; only
 *    explosions survive.  Looks like everything is broken until you
 *    see the rule name.
 *  • Gosper gun period — the canonical pattern emits one glider every
 *    30 ticks.  At g_steps=3 you see a glider every 10 frames; at
 *    g_steps=30 you see one every frame.
 *  • Steps-per-frame * grid size — at MAX_COLS=320, MAX_ROWS=128, and
 *    STEPS_MAX=30, that's ~1.2M cell updates/frame.  Still well under
 *    16ms on modern CPUs but watch the histogram ring buffer width
 *    matches g_cols, not HIST_LEN.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Blinker test (Conway): place 3 cells in a row.  After 1 tick it
 *    must be vertical; after 2 ticks horizontal again.  Period 2.
 *  • Glider test (Conway): seed 'g'.  The 5-cell glider must move
 *    diagonally one cell every 4 ticks.  Toroidal wrap means it
 *    eventually returns to start after gcd(R, C) · 4 ticks.
 *  • Acorn test (Conway): seed 'a'.  Should stabilise after exactly
 *    5206 generations into a soup of 633 cells (one of Life's most
 *    famous methuselahs).
 *  • R-pentomino test: seed 'e'.  Stabilises at gen 1103 with 116
 *    live cells (period-2 oscillators + still-lifes + 6 gliders).
 *  • Seeds test: seed 'r' under Seeds rule.  Population should
 *    explode then crash; histogram shows characteristic spikes.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

/* ── §1 config ───────────────────────────────────────────────────────── */

#define TICK_NS     33333333LL
#define MAX_ROWS    128
#define MAX_COLS    320
#define HIST_LEN    512    /* population history ring buffer length        */
#define HIST_ROWS   3      /* screen rows used by histogram                */
#define STEPS_DEF   3
#define STEPS_MAX   30
#define LIVE_CHAR   '#'

enum { CP_LIVE=1, CP_HIST, CP_HUD };

/* B/S rule encoded as bitmasks (bit N = "when count equals N") */
#define B(...)  rule_mask((int[]){__VA_ARGS__}, \
                          sizeof((int[]){__VA_ARGS__})/sizeof(int))
static uint16_t rule_mask(const int *ns, int len)
{
    uint16_t m = 0;
    for (int i = 0; i < len; i++) m |= (uint16_t)(1u << ns[i]);
    return m;
}

#define N_RULES 6
typedef struct { uint16_t birth, survive; const char *name; short color256; } Rule;
static Rule RULES[N_RULES];

static void rules_init(void)
{
    RULES[0] = (Rule){ B(3),         B(2,3),         "Conway B3/S23",       51 };
    RULES[1] = (Rule){ B(3,6),       B(2,3),         "HighLife B36/S23",    82 };
    RULES[2] = (Rule){ B(3,6,7,8),   B(3,4,6,7,8),   "Day&Night B3678/S34678", 201 };
    RULES[3] = (Rule){ B(2),         0,               "Seeds B2/S",          226 };
    RULES[4] = (Rule){ B(3,6,8),     B(2,4,5),        "Morley B368/S245",    202 };
    RULES[5] = (Rule){ B(3,6),       B(1,2,5),        "2x2 B36/S125",        45  };
}

/* ── §2 clock ────────────────────────────────────────────────────────── */

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

/* ── §3 color ────────────────────────────────────────────────────────── */

static int g_rule_idx = 0;

static void color_apply(void)
{
    start_color();
    use_default_colors();
    short lc = (COLORS >= 256) ? RULES[g_rule_idx].color256 : COLOR_GREEN;
    init_pair(CP_LIVE, lc, -1);
    init_pair(CP_HIST, (COLORS >= 256) ? 240 : COLOR_WHITE, -1);
    init_pair(CP_HUD,  (COLORS >= 256) ?  82 : COLOR_GREEN, -1);
}

/* ── §4 grid ─────────────────────────────────────────────────────────── */

static uint8_t g_grid[2][MAX_ROWS][MAX_COLS];
static int     g_buf;          /* active buffer index (0 or 1)           */
static int     g_rows, g_cols;
static int     g_ca_rows;      /* grid display rows = g_rows - HIST_ROWS - 1 */
static long    g_gen;
static int     g_steps, g_paused;

/* population history */
static long    g_pop_hist[HIST_LEN];
static int     g_hist_head;

static void grid_clear(void)
{
    memset(g_grid, 0, sizeof(g_grid));
    g_gen = 0;
    memset(g_pop_hist, 0, sizeof(g_pop_hist));
    g_hist_head = 0;
}

static void seed_random(void)
{
    grid_clear();
    for (int r = 0; r < g_ca_rows; r++)
        for (int c = 0; c < g_cols; c++)
            g_grid[g_buf][r][c] = (rand() % 10 < 3) ? 1 : 0;
}

static void place(const int cells[][2], int n, int r0, int c0)
{
    for (int i = 0; i < n; i++) {
        int r = (r0 + cells[i][0] + g_ca_rows) % g_ca_rows;
        int c = (c0 + cells[i][1] + g_cols)    % g_cols;
        g_grid[g_buf][r][c] = 1;
    }
}

static void seed_glider(void)
{
    grid_clear();
    static const int G[][2] = {{0,1},{1,2},{2,0},{2,1},{2,2}};
    place(G, 5, g_ca_rows/2 - 1, g_cols/2 - 1);
}

static void seed_rpentomino(void)
{
    grid_clear();
    static const int P[][2] = {{0,1},{0,2},{1,0},{1,1},{2,1}};
    place(P, 5, g_ca_rows/2 - 1, g_cols/2 - 1);
}

static void seed_acorn(void)
{
    grid_clear();
    static const int A[][2] = {{0,1},{1,3},{2,0},{2,1},{2,4},{2,5},{2,6}};
    place(A, 7, g_ca_rows/2 - 1, g_cols/2 - 3);
}

static void seed_gosper(void)
{
    grid_clear();
    static const int GG[][2] = {
        {0,24},
        {1,22},{1,24},
        {2,12},{2,13},{2,20},{2,21},{2,34},{2,35},
        {3,11},{3,15},{3,20},{3,21},{3,34},{3,35},
        {4, 0},{4, 1},{4,10},{4,16},{4,20},{4,21},
        {5, 0},{5, 1},{5,10},{5,14},{5,16},{5,17},{5,22},{5,24},
        {6,10},{6,16},{6,24},
        {7,11},{7,15},
        {8,12},{8,13},
    };
    int roff = g_ca_rows / 4;
    int coff = (g_cols > 40) ? 5 : 0;
    place(GG, 36, roff, coff);
}

static long grid_step(void)
{
    int next = 1 - g_buf;
    long pop  = 0;
    const Rule *ru = &RULES[g_rule_idx];

    for (int r = 0; r < g_ca_rows; r++) {
        int rn = (r + 1)              % g_ca_rows;
        int rp = (r - 1 + g_ca_rows) % g_ca_rows;
        for (int c = 0; c < g_cols; c++) {
            int cn = (c + 1)            % g_cols;
            int cp = (c - 1 + g_cols)   % g_cols;
            int n = g_grid[g_buf][rp][cp] + g_grid[g_buf][rp][c] +
                    g_grid[g_buf][rp][cn] + g_grid[g_buf][r ][cp] +
                    g_grid[g_buf][r ][cn] + g_grid[g_buf][rn][cp] +
                    g_grid[g_buf][rn][c ] + g_grid[g_buf][rn][cn];
            uint16_t bit = (uint16_t)(1u << n);
            uint8_t  cur = g_grid[g_buf][r][c];
            uint8_t  nxt = cur ? ((ru->survive & bit) ? 1 : 0)
                               : ((ru->birth   & bit) ? 1 : 0);
            g_grid[next][r][c] = nxt;
            pop += nxt;
        }
    }
    g_buf = next;
    g_gen++;
    return pop;
}

static void sim_tick(void)
{
    if (g_paused) return;
    for (int s = 0; s < g_steps; s++) {
        long pop = grid_step();
        g_pop_hist[g_hist_head] = pop;
        g_hist_head = (g_hist_head + 1) % HIST_LEN;
    }
}

/* ── §5 scene ────────────────────────────────────────────────────────── */

static void scene_grid(void)
{
    attron(COLOR_PAIR(CP_LIVE));
    for (int r = 0; r < g_ca_rows; r++) {
        uint8_t *row = g_grid[g_buf][r];
        for (int c = 0; c < g_cols - 1; c++)
            mvaddch(r, c, row[c] ? (chtype)(unsigned char)LIVE_CHAR : ' ');
    }
    attroff(COLOR_PAIR(CP_LIVE));
}

static void scene_histogram(void)
{
    long max_pop = (long)g_ca_rows * g_cols;
    if (max_pop == 0) return;

    attron(COLOR_PAIR(CP_HIST));
    for (int c = 0; c < g_cols - 1; c++) {
        int idx = (g_hist_head - (g_cols - 1 - c) + HIST_LEN * 2) % HIST_LEN;
        long pop = g_pop_hist[idx];
        /* normalize to [0, HIST_ROWS] */
        int level = (int)((float)pop / (float)max_pop * HIST_ROWS * 2.0f);
        if (level > HIST_ROWS) level = HIST_ROWS;
        for (int hr = 0; hr < HIST_ROWS; hr++) {
            /* hr=0 is top row of histogram area */
            int sr = g_ca_rows + hr;
            /* bar grows upward: bottom row fills first */
            char ch = (level >= HIST_ROWS - hr) ? '#' : '.';
            mvaddch(sr, c, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(CP_HIST));
}

static void scene_hud(void)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(g_rows - 1, 0,
             " %-22s  gen=%ld  spd=%d  %s"
             "  n/p:rule  r:rand g:glider G:gun p:pento a:acorn  +/-:spd  q:quit",
             RULES[g_rule_idx].name, g_gen, g_steps,
             g_paused ? "PAUSED " : "       ");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ── §6 screen ───────────────────────────────────────────────────────── */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_apply();
}

static void screen_resize(void)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > MAX_ROWS) rows = MAX_ROWS;
    if (cols > MAX_COLS) cols = MAX_COLS;
    g_rows    = rows;
    g_cols    = cols;
    g_ca_rows = rows - HIST_ROWS - 1;
    if (g_ca_rows < 1) g_ca_rows = 1;
    seed_random();
    erase();
}

/* ── §7 app ──────────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void sig_handler(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running = 0;
}
static void cleanup(void) { endwin(); }

int main(void)
{
    signal(SIGINT,   sig_handler);
    signal(SIGTERM,  sig_handler);
    signal(SIGWINCH, sig_handler);
    atexit(cleanup);

    rules_init();
    screen_init();
    g_steps = STEPS_DEF;
    g_buf   = 0;
    srand((unsigned)(clock_ns() & 0xFFFFFFFFu));
    screen_resize();

    long long next = clock_ns();

    while (g_running) {
        if (g_need_resize) {
            g_need_resize = 0;
            endwin(); refresh();
            screen_resize();
        }

        int ch;
        while ((ch = getch()) != ERR) {
            switch (ch) {
            case 'q': case 'Q': g_running = 0; break;
            case ' ':           g_paused ^= 1; break;
            case 'n':
                g_rule_idx = (g_rule_idx + 1) % N_RULES;
                color_apply(); seed_random(); break;
            case 'p':
                g_rule_idx = (g_rule_idx - 1 + N_RULES) % N_RULES;
                color_apply(); seed_random(); break;
            case 'r':  seed_random();    break;
            case 'g':  seed_glider();    break;
            case 'G':  seed_gosper();    break;
            /* 'p' already used for prev-rule; use 'e' for R-pentomino */
            case 'e':  seed_rpentomino(); break;
            case 'a':  seed_acorn();     break;
            case '+': case '=':
                if (g_steps < STEPS_MAX) g_steps++;
                break;
            case '-': case '_':
                if (g_steps > 1) g_steps--;
                break;
            }
        }

        sim_tick();
        erase();
        scene_grid();
        scene_histogram();
        scene_hud();
        wnoutrefresh(stdscr);
        doupdate();

        next += TICK_NS;
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
