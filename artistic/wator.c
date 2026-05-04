/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wator.c — Wa-Tor Predator-Prey World
 *
 * Fish (o) swim and breed; sharks (X) hunt fish, breed, and starve.
 * A. K. Dewdney's Wa-Tor simulation (Scientific American, December 1984).
 *
 * Rules (each tick, in randomly shuffled cell order):
 *
 *   Fish:
 *     1. age++
 *     2. Move to a random adjacent empty cell (toroidal wrap).
 *        If no empty cell exists: stay put.
 *     3. If age ≥ FISH_BREED: leave a new fish (age=0) at old cell; reset age.
 *
 *   Shark:
 *     1. breed_age++, hunger++
 *     2. If hunger ≥ SHARK_STARVE: die (cell → empty).
 *     3. Look for adjacent fish first.  If found: eat one, move there, hunger=0.
 *        Otherwise: move to a random adjacent empty cell.
 *        If no move possible: stay put.
 *     4. If breed_age ≥ SHARK_BREED: leave a new shark at old cell; reset breed_age.
 *
 * Shuffled update prevents directional bias — all live cells are collected into
 * an array, Fisher-Yates shuffled, then processed in that order.  A moved[]
 * flag prevents any cell being processed twice per tick.
 *
 * Layout:
 *   Rows 0 … g_ocean_rows−1     — ocean grid
 *   Rows g_ocean_rows … −2      — 4-row dual population history graph
 *   Row  g_rows−1               — HUD
 *
 * Population graph:
 *   Upper 2 rows — fish count (cyan bars, grow downward)
 *   Lower 2 rows — shark count (red bars, grow upward)
 *   Each column = one past tick from a HIST_LEN-entry ring buffer.
 *
 * Keys:
 *   r         reseed (fresh random ocean)
 *   space     pause / resume
 *   + / =     more steps per frame  (up to 20)
 *   - / _     fewer steps per frame
 *   q / Q     quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/wator.c -o wator -lncurses
 *
 * Sections: §1 config  §2 clock  §3 color  §4 grid  §5 sim
 *           §6 scene   §7 screen §8 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Wa-Tor predator-prey cellular automaton (Dewdney 1984).
 *                  Fish and sharks are simulated on a toroidal grid with
 *                  age-based breeding and hunger-based shark death.
 *                  Shuffled update (Fisher-Yates on active cell list) removes
 *                  directional bias — equivalent to random-order asynchronous
 *                  update, which better models spatial competition.
 *
 * Physics        : Lotka-Volterra dynamics emerge: fish and shark populations
 *                  oscillate with a phase lag (fish peak before shark peak).
 *                  Parameter space: low FISH_BREED + high SHARK_BREED →
 *                  shark overpopulation and collapse; opposite → shark
 *                  starvation.  Stable cycles exist in a narrow parameter band.
 *
 * Data-structure : Grid of uint16_t cells encoding (type, age, hunger) in
 *                  bitfields.  Active cell array rebuilt each tick by scanning
 *                  the grid O(rows×cols); Fisher-Yates shuffle is O(N_active).
 *                  moved[] flag (parallel uint8 grid) prevents double-processing.
 *
 * Rendering      : Population history ring buffer (HIST_LEN entries); drawn
 *                  as a 4-row dual bar chart: fish above, sharks below.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each cell of the toroidal ocean owns three numbers — type (empty,
 * fish, shark), breed counter, hunger counter.  Each tick, every live
 * cell is awakened in random order and runs a tiny three-rule script:
 * "did I starve?", "is there food / open water adjacent?", "have I bred
 * yet?".  No global step, no neighbour count — just per-cell lottery
 * draws against a randomly-shuffled neighbour list.  Lotka-Volterra
 * oscillations EMERGE from this micro-rule, they aren't programmed in.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a chess board where every fish and shark gets one ticket per
 * tick.  The tickets are drawn from a hat in random order.  When your
 * ticket is called, you look in the four cardinal directions: a shark
 * checks for fish first, then for empty water; a fish checks for empty
 * water; both then check whether they've bred enough times to leave a
 * baby behind.  Sharks that haven't eaten in SHARK_STARVE turns leave
 * the board.  No central referee — the only synchronization is the
 * "moved" flag that prevents re-drawing the same ticket twice.
 *
 * ALGORITHM IN STEPS  (per sim_step)
 * ──────────────────
 *  1. Clear g_moved[][].  Scan ocean, push every live cell index into
 *     g_order[]; Fisher-Yates shuffle g_order.
 *  2. For each cell index in shuffled order:
 *        if g_moved[r][c] continue.  (cell already participated this tick)
 *        if FISH  → fish_step(r,c)
 *        if SHARK → shark_step(r,c)
 *  3. fish_step:
 *        a. age++.
 *        b. shuffle 4 directions; pick first EMPTY neighbour.
 *        c. if breed_age >= FISH_BREED, leave new fish at old cell, reset.
 *        d. write fish to new cell, mark g_moved.
 *  4. shark_step:
 *        a. breed++, hunger++.
 *        b. if hunger >= SHARK_STARVE → die, return.
 *        c. shuffle dirs; first FISH wins (eat, hunger=0); else first
 *           EMPTY; else stay put.
 *        d. if breed >= SHARK_BREED, leave offspring at old cell.
 *        e. write shark to new cell, mark g_moved.
 *  5. Census fish_pop, shark_pop; push into ring buffers indexed by
 *     g_hist_head, advance head modulo HIST_LEN.
 *  6. Render: ocean rows (top), 4-row dual histogram (middle), HUD (bot).
 *
 * KEY FORMULAS
 * ────────────
 *  Toroidal wrap   :  nr = (r + dr + R) mod R,   nc = (c + dc + C) mod C
 *  Histogram index :  idx = (head − (cols−1−col) + 2·HIST_LEN) mod HIST_LEN
 *  Bar height      :  fl = pop · bar_rows / scale,  scale_fish = R·C/2,
 *                     scale_shark = R·C/10  (sharks scaled brighter
 *                     because they're rarer)
 *  Visual age cue  :  fish "old"   when breed_age >= FISH_BREED − 1
 *                     shark "hungry" when hunger >= SHARK_STARVE − 1
 *  Population
 *    oscillation   :  steady-state cycle requires
 *                     fish births ≈ shark predation rate;
 *                     phase lag arises because sharks reproduce slower
 *                     than fish.  At defaults
 *                     (FISH_BREED=3, SHARK_BREED=10, SHARK_STARVE=4)
 *                     the system shows visible Lotka–Volterra waves.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • A shark that just bred AND ate — the offspring at the OLD cell
 *    inherits hunger from after the meal (zero), not the parent's
 *    pre-meal hunger.  This is the classic Wa-Tor convention.
 *  • g_moved guard: when shark eats fish, the eaten fish's slot becomes
 *    SHARK with moved=1, so when the fish's own ticket is later drawn,
 *    the cell is no longer FISH and the if-chain falls through.
 *  • Total extinction is a stable fixed point — when sharks eat all
 *    fish, they then starve out; ocean ends empty.  Press `r` to reseed.
 *  • Histogram scale is fixed; if fish exceed 50 % of grid the bars
 *    saturate (always full); below scale they fall to 0 (empty bar).
 *  • HIST_LEN = 512 is a ring; older history scrolls off the left edge
 *    silently when g_cols > 512 (rare on a terminal).
 *  • g_breed and g_hunger are uint8_t and saturate at 255 — fish that
 *    sit trapped in a corner forever stop counting at 255 instead of
 *    overflowing.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press `r`: ocean reseeds; fish density ≈30 %, sharks ≈5 %.
 *    Count by sampling 100 random cells.
 *  • Watch the histogram: fish bars (cyan, top) and shark bars (red,
 *    bottom) should oscillate roughly out of phase — fish peak comes
 *    several ticks before shark peak.
 *  • Hold `+` to crank steps/frame to 20: oscillation period clearly
 *    visible across the screen width within seconds.
 *  • Pause (space): tick counter freezes; populations shown stay fixed.
 *  • A fully starved ocean (everything gone) should show empty bars
 *    sliding rightward as new zero-pop ticks accumulate.
 *  • Single fish-only ocean (set SHARK_INIT_PCT=0): fish saturate the
 *    ocean within ~10 ticks at FISH_BREED=3.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

/* ── §1 config ──────────────────────────────────────────────────────────── */

#define TICK_NS         33333333LL   /* ~30 fps                              */
#define MAX_ROWS        128
#define MAX_COLS        320
#define HIST_ROWS       4            /* rows reserved for population graph   */
#define HIST_LEN        512          /* ring-buffer length (columns of history)*/

#define FISH_BREED      3            /* fish reproduces every N ticks        */
#define SHARK_BREED     10           /* shark reproduces every N ticks       */
#define SHARK_STARVE    4            /* shark dies after N ticks without food*/

#define FISH_INIT_PCT   30           /* initial fish density (% of cells)    */
#define SHARK_INIT_PCT  5            /* initial shark density (% of cells)   */

#define STEPS_DEF       1
#define STEPS_MAX       20

#define EMPTY  0
#define FISH   1
#define SHARK  2

/* direction vectors: N E S W */
static const int DR[4] = {-1,  0, 1, 0};
static const int DC[4] = { 0,  1, 0,-1};

enum {
    CP_FISH_Y = 1,   /* young fish (bright cyan)   */
    CP_FISH_O,       /* old fish   (dim teal)       */
    CP_SHARK_F,      /* fed shark  (bright red)     */
    CP_SHARK_H,      /* hungry shark (orange)       */
    CP_HIST_F,       /* histogram — fish            */
    CP_HIST_S,       /* histogram — shark           */
    CP_HUD
};

/* ── §2 clock ───────────────────────────────────────────────────────────── */

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

/* ── §3 color ───────────────────────────────────────────────────────────── */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_FISH_Y,  51,  -1);  /* bright cyan — young fish         */
        init_pair(CP_FISH_O,  37,  -1);  /* dim teal    — old fish           */
        init_pair(CP_SHARK_F, 196, -1);  /* bright red  — fed shark          */
        init_pair(CP_SHARK_H, 202, -1);  /* orange      — hungry shark       */
        init_pair(CP_HIST_F,  45,  -1);  /* cyan histogram bar               */
        init_pair(CP_HIST_S,  160, -1);  /* dark red histogram bar           */
        init_pair(CP_HUD,     82,  -1);  /* green HUD                        */
    } else {
        init_pair(CP_FISH_Y,  COLOR_CYAN,   -1);
        init_pair(CP_FISH_O,  COLOR_CYAN,   -1);
        init_pair(CP_SHARK_F, COLOR_RED,    -1);
        init_pair(CP_SHARK_H, COLOR_YELLOW, -1);
        init_pair(CP_HIST_F,  COLOR_CYAN,   -1);
        init_pair(CP_HIST_S,  COLOR_RED,    -1);
        init_pair(CP_HUD,     COLOR_GREEN,  -1);
    }
}

/* ── §4 grid ────────────────────────────────────────────────────────────── */

static uint8_t g_type  [MAX_ROWS][MAX_COLS]; /* EMPTY / FISH / SHARK         */
static uint8_t g_breed [MAX_ROWS][MAX_COLS]; /* fish: age; shark: breed_age  */
static uint8_t g_hunger[MAX_ROWS][MAX_COLS]; /* shark: ticks since last meal */
static uint8_t g_moved [MAX_ROWS][MAX_COLS]; /* set after entity moves/dies  */
static int     g_order [MAX_ROWS * MAX_COLS];/* shuffled processing indices  */

static int  g_rows, g_cols, g_ocean_rows;
static int  g_steps, g_paused;
static long long g_tick;
static long g_fish_pop, g_shark_pop;

/* dual population ring buffers */
static long g_fish_hist [HIST_LEN];
static long g_shark_hist[HIST_LEN];
static int  g_hist_head;

static void grid_seed(void)
{
    memset(g_type,    0, sizeof(g_type));
    memset(g_breed,   0, sizeof(g_breed));
    memset(g_hunger,  0, sizeof(g_hunger));
    memset(g_fish_hist,  0, sizeof(g_fish_hist));
    memset(g_shark_hist, 0, sizeof(g_shark_hist));
    g_hist_head = 0;
    g_tick      = 0;

    for (int r = 0; r < g_ocean_rows; r++) {
        for (int c = 0; c < g_cols; c++) {
            int roll = rand() % 100;
            if (roll < FISH_INIT_PCT) {
                g_type [r][c] = FISH;
                g_breed[r][c] = (uint8_t)(rand() % FISH_BREED);
            } else if (roll < FISH_INIT_PCT + SHARK_INIT_PCT) {
                g_type  [r][c] = SHARK;
                g_breed [r][c] = (uint8_t)(rand() % SHARK_BREED);
                g_hunger[r][c] = (uint8_t)(rand() % SHARK_STARVE);
            }
        }
    }
}

/* ── §5 simulation ──────────────────────────────────────────────────────── */

/* Fisher-Yates shuffle */
static void ishuffle(int *arr, int n)
{
    for (int i = n - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int t = arr[i]; arr[i] = arr[j]; arr[j] = t;
    }
}

static void fish_step(int r, int c)
{
    if (g_breed[r][c] < 255) g_breed[r][c]++;

    /* find a random empty neighbour */
    int dirs[4] = {0, 1, 2, 3};
    ishuffle(dirs, 4);
    int nr = -1, nc = -1;
    for (int i = 0; i < 4; i++) {
        int tr = (r + DR[dirs[i]] + g_ocean_rows) % g_ocean_rows;
        int tc = (c + DC[dirs[i]] + g_cols)       % g_cols;
        if (g_type[tr][tc] == EMPTY) { nr = tr; nc = tc; break; }
    }
    if (nr < 0) return;  /* no empty neighbour — stay put */

    int     breed_now   = (g_breed[r][c] >= FISH_BREED);
    uint8_t carry_breed = breed_now ? 0 : g_breed[r][c];

    /* place fish at new cell */
    g_type  [nr][nc] = FISH;
    g_breed [nr][nc] = carry_breed;
    g_hunger[nr][nc] = 0;
    g_moved [nr][nc] = 1;

    if (breed_now) {
        /* leave offspring at old cell */
        g_type [r][c] = FISH;
        g_breed[r][c] = 0;
    } else {
        g_type  [r][c] = EMPTY;
        g_breed [r][c] = 0;
        g_hunger[r][c] = 0;
    }
}

static void shark_step(int r, int c)
{
    if (g_breed [r][c] < 255) g_breed [r][c]++;
    if (g_hunger[r][c] < 255) g_hunger[r][c]++;

    /* starvation */
    if (g_hunger[r][c] >= SHARK_STARVE) {
        g_type  [r][c] = EMPTY;
        g_breed [r][c] = 0;
        g_hunger[r][c] = 0;
        return;
    }

    /* shuffle directions once — used for both eating and moving passes */
    int dirs[4] = {0, 1, 2, 3};
    ishuffle(dirs, 4);

    int nr = -1, nc = -1, ate = 0;

    /* pass 1: prefer adjacent fish */
    for (int i = 0; i < 4; i++) {
        int tr = (r + DR[dirs[i]] + g_ocean_rows) % g_ocean_rows;
        int tc = (c + DC[dirs[i]] + g_cols)       % g_cols;
        if (g_type[tr][tc] == FISH) { nr = tr; nc = tc; ate = 1; break; }
    }
    /* pass 2: fall back to empty cell */
    if (nr < 0) {
        for (int i = 0; i < 4; i++) {
            int tr = (r + DR[dirs[i]] + g_ocean_rows) % g_ocean_rows;
            int tc = (c + DC[dirs[i]] + g_cols)       % g_cols;
            if (g_type[tr][tc] == EMPTY) { nr = tr; nc = tc; break; }
        }
    }
    if (nr < 0) return;  /* trapped — stay put */

    uint8_t new_hunger  = ate ? 0 : g_hunger[r][c];
    int     breed_now   = (g_breed[r][c] >= SHARK_BREED);
    uint8_t carry_breed = breed_now ? 0 : g_breed[r][c];

    /* place shark at new cell */
    g_type  [nr][nc] = SHARK;
    g_breed [nr][nc] = carry_breed;
    g_hunger[nr][nc] = new_hunger;
    g_moved [nr][nc] = 1;

    if (breed_now) {
        /* leave offspring at old cell */
        g_type  [r][c] = SHARK;
        g_breed [r][c] = 0;
        g_hunger[r][c] = new_hunger;  /* offspring inherits parent's hunger */
    } else {
        g_type  [r][c] = EMPTY;
        g_breed [r][c] = 0;
        g_hunger[r][c] = 0;
    }
}

static void sim_step(void)
{
    memset(g_moved, 0, sizeof(g_moved));

    /* collect all live cells and shuffle processing order */
    int n = 0;
    for (int r = 0; r < g_ocean_rows; r++)
        for (int c = 0; c < g_cols; c++)
            if (g_type[r][c] != EMPTY)
                g_order[n++] = r * MAX_COLS + c;
    ishuffle(g_order, n);

    for (int i = 0; i < n; i++) {
        int r = g_order[i] / MAX_COLS;
        int c = g_order[i] % MAX_COLS;
        if (g_moved[r][c]) continue;
        if      (g_type[r][c] == FISH)  fish_step(r, c);
        else if (g_type[r][c] == SHARK) shark_step(r, c);
    }

    /* population census */
    g_fish_pop = 0; g_shark_pop = 0;
    for (int r = 0; r < g_ocean_rows; r++)
        for (int c = 0; c < g_cols; c++) {
            if      (g_type[r][c] == FISH)  g_fish_pop++;
            else if (g_type[r][c] == SHARK) g_shark_pop++;
        }

    g_fish_hist [g_hist_head] = g_fish_pop;
    g_shark_hist[g_hist_head] = g_shark_pop;
    g_hist_head = (g_hist_head + 1) % HIST_LEN;
    g_tick++;
}

static void sim_tick(void)
{
    if (g_paused) return;
    for (int s = 0; s < g_steps; s++) sim_step();
}

/* ── §6 scene ───────────────────────────────────────────────────────────── */

static void scene_ocean(void)
{
    for (int r = 0; r < g_ocean_rows; r++) {
        for (int c = 0; c < g_cols - 1; c++) {
            uint8_t t = g_type[r][c];
            if (t == EMPTY) { mvaddch(r, c, ' '); continue; }
            if (t == FISH) {
                int old    = (g_breed[r][c] >= FISH_BREED - 1);
                attr_t at  = COLOR_PAIR(old ? CP_FISH_O : CP_FISH_Y);
                if (!old) at |= A_BOLD;
                attron(at);
                mvaddch(r, c, 'o');
                attroff(at);
            } else {
                int hungry = (g_hunger[r][c] >= SHARK_STARVE - 1);
                attr_t at  = COLOR_PAIR(hungry ? CP_SHARK_H : CP_SHARK_F) | A_BOLD;
                attron(at);
                mvaddch(r, c, 'X');
                attroff(at);
            }
        }
    }
}

static void scene_histogram(void)
{
    long max_pop = (long)g_ocean_rows * (g_cols - 1);
    if (max_pop == 0) return;

    /* scale so typical populations fill the bars:
     *   fish  bar full at 50 % of grid
     *   shark bar full at 10 % of grid                                     */
    long fish_scale  = max_pop / 2;  if (fish_scale  < 1) fish_scale  = 1;
    long shark_scale = max_pop / 10; if (shark_scale < 1) shark_scale = 1;

    int fish_rows  = HIST_ROWS / 2;  /* 2 rows for fish  */
    int shark_rows = HIST_ROWS / 2;  /* 2 rows for sharks*/

    for (int c = 0; c < g_cols - 1; c++) {
        int idx = (g_hist_head - (g_cols - 1 - c) + HIST_LEN * 2) % HIST_LEN;

        /* fish bars — upper 2 rows, grow downward from the top */
        int fl = (int)((float)g_fish_hist[idx]  / (float)fish_scale  * fish_rows);
        if (fl > fish_rows) fl = fish_rows;

        attron(COLOR_PAIR(CP_HIST_F));
        for (int hr = 0; hr < fish_rows; hr++) {
            int sr = g_ocean_rows + hr;
            mvaddch(sr, c, (fl > hr) ? '#' : '.');
        }
        attroff(COLOR_PAIR(CP_HIST_F));

        /* shark bars — lower 2 rows, grow upward from the bottom */
        int sl = (int)((float)g_shark_hist[idx] / (float)shark_scale * shark_rows);
        if (sl > shark_rows) sl = shark_rows;

        attron(COLOR_PAIR(CP_HIST_S));
        for (int hr = 0; hr < shark_rows; hr++) {
            int sr = g_ocean_rows + fish_rows + hr;
            /* hr=0 is top of shark area; fill from bottom (hr=shark_rows-1) up */
            mvaddch(sr, c, (sl >= shark_rows - hr) ? '#' : '.');
        }
        attroff(COLOR_PAIR(CP_HIST_S));
    }
}

static void scene_hud(void)
{
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(g_rows - 1, 0,
             " tick=%-7lld  fish=%-6ld  sharks=%-5ld  spd=%d  %s"
             "  r:reseed  +/-:spd  spc:pause  q:quit",
             (long long)g_tick, g_fish_pop, g_shark_pop, g_steps,
             g_paused ? "PAUSED " : "       ");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ── §7 screen ──────────────────────────────────────────────────────────── */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}

static void screen_resize(void)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > MAX_ROWS) rows = MAX_ROWS;
    if (cols > MAX_COLS) cols = MAX_COLS;
    g_rows       = rows;
    g_cols       = cols;
    g_ocean_rows = rows - HIST_ROWS - 1;
    if (g_ocean_rows < 1) g_ocean_rows = 1;
    grid_seed();
    erase();
}

/* ── §8 app ─────────────────────────────────────────────────────────────── */

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

    screen_init();
    g_steps  = STEPS_DEF;
    g_paused = 0;
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
            case 'q': case 'Q': g_running = 0;          break;
            case ' ':           g_paused ^= 1;           break;
            case 'r':           grid_seed(); erase();    break;
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
        scene_ocean();
        scene_histogram();
        scene_hud();
        wnoutrefresh(stdscr);
        doupdate();

        next += TICK_NS;
        clock_sleep_ns(next - clock_ns());
    }
    return 0;
}
