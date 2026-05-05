/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sort_vis.c — Sorting Algorithm Visualiser (5 algorithms)
 *
 * DEMO: A row of 48 bars of different heights stands across the screen.
 *       Each tick the active algorithm performs ONE compare or swap;
 *       the two indices it touches glow gold (compare) or red-bold
 *       (swap), so you watch the algorithm walk the array in real time.
 *       When the array is sorted, every bar turns matrix-green.  TAB
 *       cycles algorithms, +/- doubles or halves operations per frame.
 *
 * Study alongside: misc/maze.c — both turn an iterative algorithm
 *   (carve a maze / sort an array) into a state machine that emits one
 *   visible event per tick, so the animation IS the algorithm.
 *
 * Section map:
 *   §1 config  — N_ELEMS, HUD_ROWS, frame timing, color pair IDs
 *   §2 clock   — monotonic timer + nanosleep
 *   §3 color   — bar/compare/swap/sort + HUD/HINT pairs
 *   §5 sort    — five state-machine sorters, one operation per call
 *   §7 scene   — mark_cell helper + draw_bars + draw_hud
 *   §8 app     — signals, resize, key handler, main loop
 *
 * Keys:  q/ESC quit   TAB next alg   space scramble   p pause   +/- speed
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra misc/sort_vis.c -o sort_vis -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Five classic comparison sorts, each rewritten as an
 *                  iterative coroutine that yields after one compare-or-
 *                  swap so the visualiser can pace the animation.
 *                    Bubble    O(n²) — adjacent swaps, stable.
 *                    Insertion O(n²) — shift-into-place, stable.
 *                    Selection O(n²) compares + O(n) swaps, not stable.
 *                    Quicksort O(n log n) average, Lomuto partition.
 *                    Heapsort  O(n log n) worst-case, max-heap.
 *
 * Data-structure : One global int array g_arr[N_ELEMS] plus a per-
 *                  algorithm bundle of cursor variables (g_bi/g_bj,
 *                  g_qlo/g_qhi/g_qtop, g_hi_h/g_hphase, ...).  Quicksort
 *                  uses an explicit stack so we never recurse — every
 *                  algorithm can be paused and resumed mid-step.
 *
 * Rendering      : Vertical bar chart.  bar_h = arr[i] · bar_max /
 *                  N_ELEMS in '#' characters, drawn from bottom up.
 *                  Cell colour encodes the per-tick operation:
 *                    grey '#'     — untouched element
 *                    gold '#'     — currently being compared
 *                    red bold '#' — just swapped
 *                    green bold   — sorted (final pass)
 *
 * Performance    : One step per frame is plenty visual.  +/- exponentially
 *                  scales steps-per-frame from 1× to 256× so heapsort on
 *                  N=48 finishes in a couple of seconds at high speed but
 *                  bubble sort is still hypnotic at 1×.
 *
 * References     : Sedgewick & Wayne, "Algorithms" 4th ed., chs. 2.1-2.4
 *                    (bubble, insertion, selection, quick).
 *                  Cormen et al., CLRS 3rd ed., ch. 6 (heapsort).
 *                  Wikipedia, "Sorting algorithm",
 *                    https://en.wikipedia.org/wiki/Sorting_algorithm
 *                  Mike Bostock, "Visualizing Algorithms",
 *                    https://bost.ocks.org/mike/algorithms/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Every comparison sort is a sequence of two-element decisions: look at
 * a pair, decide if they're out of order, maybe swap.  The five
 * algorithms differ ONLY in the rule for choosing the next pair.
 * Animating one decision per frame turns each algorithm into a visible
 * personality — bubble crawls, insertion smooths the left, selection
 * drags a marker, quicksort splits, heap rebuilds.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine 48 people of different heights waiting to line up shortest-to-
 * tallest.  A coach barks out one rule and the line follows it:
 *   Bubble    — "compare your right neighbour; if they're shorter, trade."
 *   Insertion — "you're new; walk left until the person beside you is shorter."
 *   Selection — "find the shortest still-unsorted person; bring them to the front."
 *   Quicksort — "pivot is whoever's at the right end; everyone shorter goes left."
 *   Heapsort  — "build a tournament bracket where parent beats child, then peel
 *                off the champion repeatedly."
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 * Each step function returns 1 when finished, 0 otherwise:
 *
 *   bubble_step():
 *     1. Compare arr[j] with arr[j+1]; swap if out of order.
 *     2. j++. When j hits the end of the unsorted prefix, j=0, i++.
 *     3. Done when i == N-1.
 *
 *   insert_step():
 *     1. Compare arr[j-1] with arr[j].
 *     2. If left is bigger, swap and decrement j (slide left).
 *     3. Otherwise advance to next unsorted element (j = ++i).
 *     4. Done when i == N.
 *
 *   select_step():
 *     1. Walk j across the unsorted suffix, tracking running minimum.
 *     2. When j hits N, swap arr[i] with arr[min] and advance i.
 *     3. Done when i == N-1.
 *
 *   quick_step():  Lomuto, iterative.  Maintains stack of (lo, hi).
 *     1. Phase 0: scan j over [lo, hi-1] comparing arr[j] vs pivot=arr[hi];
 *        if arr[j] <= pivot, swap into the "smaller" zone (i++; swap i,j).
 *     2. Phase 1: place pivot at arr[i+1]; push left and right sub-ranges.
 *     3. Pop next valid range; recurse iteratively.  Done when stack empty.
 *
 *   heap_step():
 *     1. Phase 0 (build): sift-down from N/2-1 down to 0.
 *     2. Phase 1 (extract): swap arr[0] with arr[n-1]; n--; sift-down root.
 *     3. Done when n <= 0.
 *
 * KEY FORMULAS
 * ────────────
 * Bar height:  bar_h = arr[i] · bar_max / N_ELEMS   (integer scale).
 * Bar column:  col_start = i · cols / N_ELEMS,
 *              col_end   = (i+1) · cols / N_ELEMS.
 * Heap children of node k:  left = 2k+1,  right = 2k+2.
 * Lomuto partition invariant after the scan: arr[lo..i] ≤ pivot,
 *   arr[i+1..j-1] > pivot, arr[j..hi-1] unscanned, arr[hi] = pivot.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Quicksort stack depth.  QS_STACK = 128 is enough for any random
 *    array of N=48; for adversarial inputs (already sorted) Lomuto can
 *    push N ranges, so the cap matters.  We silently drop ranges past
 *    the cap — fine for the visualiser, would be a bug in production.
 *  • Heap sift atomicity.  heap_sift() walks ALL the way down in one
 *    call — it does NOT yield per swap.  The other four algorithms do.
 *    This is a deliberate compromise: per-swap heapsort is twice as
 *    much state to track and the swap stream still looks distinct.
 *  • Speed loop short-circuit.  When a step returns 1 (done), break
 *    immediately — running more steps clobbers the "g_done = true"
 *    flag and replays from the cursor on the next compare.
 *  • Resize.  We just refresh terminal dimensions; the array keeps its
 *    current sort progress, so a mid-sort resize is harmless.
 *  • +/- speed range [1, 256].  Below 1 means "no progress per frame";
 *    above 256 the animation skips so much state nothing is visible.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • At end of run, every i should satisfy arr[i] = i+1 (we scrambled
 *    1..N).  All bars rise monotonically left-to-right and turn green.
 *  • Compare counts:  bubble & insertion ~ N²/2 = 1152 worst-case;
 *    selection ≈ N²/2; heapsort ≈ N·log₂(N) ≈ 268; quicksort ~ N·log₂(N)
 *    on random input ≈ 268, but up to N²/2 on already-sorted.
 *  • Doubling N from 48 to 96 should roughly quadruple bubble/select
 *    operation counts and merely double heap/quick.
 *  • Press TAB on a near-sorted array.  Bubble finishes almost
 *    instantly (single pass with no swaps); selection still does
 *    N²/2 compares regardless of input — its cost is input-blind.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define N_ELEMS         48
#define HUD_ROWS         3      /* top 3 rows reserved for status         */
#define STEPS_DEFAULT    1
#define STEPS_MAX      256

#define TARGET_FPS      30
#define NS_PER_SEC      1000000000LL
#define FRAME_NS        (NS_PER_SEC / TARGET_FPS)

#define QS_STACK       128      /* Lomuto iterative-stack depth           */

enum Alg {
    ALG_BUBBLE, ALG_INSERT, ALG_SELECT, ALG_QUICK, ALG_HEAP, ALG_COUNT
};
static const char *ALG_NAME[ALG_COUNT] = {
    "Bubble", "Insertion", "Selection", "Quicksort", "Heapsort"
};

/* Color pair IDs.  HUD pairs are reserved 8/9 across all demos. */
enum {
    PAIR_NORM = 1,    /* untouched bars, light grey                  */
    PAIR_CMP,         /* current compare pair, gold                  */
    PAIR_SWP,         /* just-swapped pair, bright red + A_BOLD      */
    PAIR_SORT,        /* sort complete, matrix green + A_BOLD        */
    PAIR_HUD  = 8,    /* top status line, yellow + A_BOLD            */
    PAIR_HINT = 9     /* bottom key hint, cyan + A_BOLD              */
};

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
 * color_init() — bg = -1 (terminal default) for every pair.  Fg's are
 * picked from the bright half of the 256-colour cube so A_BOLD/A_DIM
 * render legibly on any terminal background.  Compare uses gold (220)
 * rather than HUD-yellow (226) so the bar highlight reads distinct from
 * the status line even when both are on screen.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_NORM, 250, -1);   /* light grey  bars            */
        init_pair(PAIR_CMP,  220, -1);   /* gold        comparing pair  */
        init_pair(PAIR_SWP,  196, -1);   /* bright red  swapped pair    */
        init_pair(PAIR_SORT,  46, -1);   /* matrix grn  sorted          */
        init_pair(PAIR_HUD,  226, -1);   /* yellow      status          */
        init_pair(PAIR_HINT,  51, -1);   /* cyan        hint strip      */
    } else {
        init_pair(PAIR_NORM, COLOR_WHITE,  -1);
        init_pair(PAIR_CMP,  COLOR_YELLOW, -1);
        init_pair(PAIR_SWP,  COLOR_RED,    -1);
        init_pair(PAIR_SORT, COLOR_GREEN,  -1);
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* ===================================================================== */
/* §5  sort state machines                                                */
/* ===================================================================== */

static int       g_arr[N_ELEMS];
static int       g_cmp1 = -1, g_cmp2 = -1;     /* indices being compared */
static int       g_swp1 = -1, g_swp2 = -1;     /* indices just swapped   */
static bool      g_done;
static long long g_cmp_count, g_swp_count;

/* Per-algorithm cursors — one block per state machine. */
static int g_bi, g_bj;                          /* Bubble    */
static int g_ii, g_ij;                          /* Insertion */
static int g_si, g_sj, g_smin;                  /* Selection */
static int g_qlo[QS_STACK], g_qhi[QS_STACK], g_qtop;
static int g_ql, g_qh, g_qi, g_qj, g_qphase;   /* Quick: 0=scan 1=pop  */
static int g_hn, g_hi_h, g_hphase;             /* Heap:  0=build 1=ext */

static void arr_swap(int a, int b)
{
    int t = g_arr[a]; g_arr[a] = g_arr[b]; g_arr[b] = t;
    g_swp1 = a; g_swp2 = b;
    g_swp_count++;
}

static void init_alg(enum Alg alg)
{
    g_cmp1 = g_cmp2 = g_swp1 = g_swp2 = -1;
    g_cmp_count = g_swp_count = 0;
    g_done = false;
    switch (alg) {
    case ALG_BUBBLE: g_bi = 0; g_bj = 0;                              break;
    case ALG_INSERT: g_ii = 1; g_ij = 1;                              break;
    case ALG_SELECT: g_si = 0; g_sj = 1; g_smin = 0;                  break;
    case ALG_QUICK:
        g_qtop = 0;
        g_ql   = 0;  g_qh = N_ELEMS - 1;
        g_qi   = -1; g_qj = 0;          /* Lomuto: i = lo - 1         */
        g_qphase = 0;
        break;
    case ALG_HEAP:
        g_hn   = N_ELEMS;
        g_hi_h = N_ELEMS / 2 - 1;
        g_hphase = 0;
        break;
    default: break;
    }
}

/* Returns 0 = step done, 1 = sort complete. */
static int bubble_step(void)
{
    if (g_bi >= N_ELEMS - 1) { g_done = true; return 1; }
    g_cmp1 = g_bj; g_cmp2 = g_bj + 1; g_swp1 = g_swp2 = -1;
    g_cmp_count++;
    if (g_arr[g_bj] > g_arr[g_bj + 1]) arr_swap(g_bj, g_bj + 1);
    if (++g_bj >= N_ELEMS - 1 - g_bi) { g_bj = 0; g_bi++; }
    return 0;
}

static int insert_step(void)
{
    if (g_ii >= N_ELEMS) { g_done = true; return 1; }
    g_cmp1 = g_ij - 1; g_cmp2 = g_ij; g_swp1 = g_swp2 = -1;
    g_cmp_count++;
    if (g_ij > 0 && g_arr[g_ij - 1] > g_arr[g_ij]) {
        arr_swap(g_ij - 1, g_ij);
        g_ij--;
    } else {
        g_ii++;
        g_ij = g_ii;
    }
    return 0;
}

static int select_step(void)
{
    if (g_si >= N_ELEMS - 1) { g_done = true; return 1; }
    g_swp1 = g_swp2 = -1;
    if (g_sj < N_ELEMS) {
        g_cmp1 = g_smin; g_cmp2 = g_sj;
        g_cmp_count++;
        if (g_arr[g_sj] < g_arr[g_smin]) g_smin = g_sj;
        g_sj++;
    } else {
        if (g_smin != g_si) arr_swap(g_si, g_smin);
        g_si++; g_sj = g_si + 1; g_smin = g_si;
    }
    return 0;
}

/*
 * quick_step() — Lomuto partition, iterative.
 *
 * Phase 0 scans g_qj across the current sub-range comparing each
 * element to the pivot at arr[g_qh]; if smaller, the "boundary"
 * cursor g_qi advances and arr[i] swaps with arr[j] to keep the
 * smaller-than-pivot zone on the left.  When the scan finishes we
 * drop into phase 1, place the pivot, and pop the next sub-range
 * from the stack.  See KEY FORMULAS for the partition invariant.
 */
static int quick_step(void)
{
    g_cmp1 = g_cmp2 = g_swp1 = g_swp2 = -1;

    if (g_qphase == 0) {
        if (g_qj < g_qh) {
            g_cmp1 = g_qj; g_cmp2 = g_qh;
            g_cmp_count++;
            if (g_arr[g_qj] <= g_arr[g_qh]) {
                g_qi++;
                arr_swap(g_qi, g_qj);
            }
            g_qj++;
            return 0;
        }
        /* Scan finished — place pivot, push sub-ranges. */
        g_qi++;
        if (g_qi != g_qh) arr_swap(g_qi, g_qh);
        if (g_ql < g_qi - 1 && g_qtop < QS_STACK - 1) {
            g_qlo[g_qtop] = g_ql;     g_qhi[g_qtop] = g_qi - 1; g_qtop++;
        }
        if (g_qi + 1 < g_qh && g_qtop < QS_STACK - 1) {
            g_qlo[g_qtop] = g_qi + 1; g_qhi[g_qtop] = g_qh;     g_qtop++;
        }
        g_qphase = 1;
        return 0;
    }

    /* Phase 1 — pop next valid range. */
    while (g_qtop > 0) {
        g_qtop--;
        int lo = g_qlo[g_qtop], hi = g_qhi[g_qtop];
        if (lo < hi) {
            g_ql = lo; g_qh = hi;
            g_qi = lo - 1; g_qj = lo;
            g_qphase = 0;
            return 0;
        }
    }
    g_done = true;
    return 1;
}

/*
 * heap_sift() — restore the max-heap property at `root` by walking
 * the larger child downward until the heap rule holds or we reach
 * a leaf.  This is the only function in the file that performs
 * multiple swaps per call (see EDGE CASES).
 */
static void heap_sift(int root, int end)
{
    while (1) {
        int largest = root;
        int l = 2 * root + 1, r = 2 * root + 2;
        g_cmp_count++;
        if (l <= end && g_arr[l] > g_arr[largest]) largest = l;
        if (r <= end && g_arr[r] > g_arr[largest]) largest = r;
        g_cmp1 = root; g_cmp2 = largest;
        if (largest == root) break;
        arr_swap(root, largest);
        root = largest;
    }
}

static int heap_step(void)
{
    g_cmp1 = g_cmp2 = g_swp1 = g_swp2 = -1;
    if (g_hphase == 0) {
        if (g_hi_h >= 0) { heap_sift(g_hi_h--, N_ELEMS - 1); return 0; }
        g_hphase = 1;
        g_hn = N_ELEMS - 1;
    }
    if (g_hn <= 0) { g_done = true; return 1; }
    arr_swap(0, g_hn);
    heap_sift(0, --g_hn);
    return 0;
}

typedef int (*StepFn)(void);
static const StepFn STEP_FN[ALG_COUNT] = {
    bubble_step, insert_step, select_step, quick_step, heap_step
};

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

static int      g_rows, g_cols;
static bool     g_paused;
static int      g_speed = STEPS_DEFAULT;
static enum Alg g_alg   = ALG_BUBBLE;

static void scramble(void)
{
    for (int i = 0; i < N_ELEMS; i++) g_arr[i] = i + 1;
    for (int i = N_ELEMS - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int t = g_arr[i]; g_arr[i] = g_arr[j]; g_arr[j] = t;
    }
    init_alg(g_alg);
}

/*
 * mark_cell() — bounds-checked mvaddch with the canonical
 * (chtype)(unsigned char) double-cast that prevents sign-extension
 * corruption on glyphs above 0x7F.  All bar drawing routes through
 * this single helper.
 */
static void mark_cell(int sr, int sc, char ch, int pair, int attr)
{
    if (sr < 0 || sr >= g_rows) return;
    if (sc < 0 || sc >= g_cols) return;
    chtype c = (chtype)(unsigned char)ch;
    if (pair) c |= (chtype)COLOR_PAIR(pair);
    if (attr) c |= (chtype)attr;
    mvaddch(sr, sc, c);
}

/* Pick the colour pair for bar i based on its current operation state. */
static int bar_pair(int i)
{
    if (i == g_swp1 || i == g_swp2) return PAIR_SWP;
    if (i == g_cmp1 || i == g_cmp2) return PAIR_CMP;
    if (g_done)                     return PAIR_SORT;
    return PAIR_NORM;
}

/* Draw a single vertical bar of '#' characters from the bottom up. */
static void draw_bar(int i, int bar_max)
{
    int bar_h = g_arr[i] * bar_max / N_ELEMS;
    int col_s = i       * g_cols / N_ELEMS;
    int col_e = (i + 1) * g_cols / N_ELEMS;
    if (col_e > g_cols) col_e = g_cols;

    int pair = bar_pair(i);
    int attr = (pair == PAIR_SWP || pair == PAIR_SORT) ? A_BOLD : 0;

    for (int row = g_rows - 1; row >= HUD_ROWS; row--) {
        int depth = g_rows - 1 - row;
        if (depth >= bar_h) break;            /* above bar — empty cells */
        for (int c = col_s; c < col_e; c++)
            mark_cell(row, c, '#', pair, attr);
    }
}

static void draw_bars(void)
{
    int bar_area = g_rows - HUD_ROWS;
    int bar_max  = bar_area > 1 ? bar_area - 1 : 1;
    for (int i = 0; i < N_ELEMS; i++) draw_bar(i, bar_max);
}

static void draw_hud(void)
{
    char status[80];
    snprintf(status, sizeof status,
             " %s  speed:%dx  N=%d  cmp:%lld  swp:%lld  %s ",
             ALG_NAME[g_alg], g_speed, N_ELEMS,
             g_cmp_count, g_swp_count,
             g_done   ? "SORTED"  :
             g_paused ? "PAUSED"  : "running");

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    int pad = g_cols - (int)strlen(status);
    if (pad < 0) pad = 0;
    mvprintw(0, pad, "%s", status);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g_rows - 1, 0,
        " q:quit  TAB:next-alg  spc:scramble  p:pause  +/-:speed ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(void)
{
    draw_bars();
    draw_hud();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

static void handle_key(int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: g_quit = 1; break;
    case ' ':                    scramble(); break;
    case 'p': case 'P':          g_paused = !g_paused; break;
    case '\t':
        g_alg = (enum Alg)((g_alg + 1) % ALG_COUNT);
        scramble();
        break;
    case '+': case '=':
        g_speed *= 2;
        if (g_speed > STEPS_MAX) g_speed = STEPS_MAX;
        break;
    case '-':
        g_speed /= 2;
        if (g_speed < 1) g_speed = 1;
        break;
    default: break;
    }
}

static void step_simulation(void)
{
    if (g_paused || g_done) return;
    for (int s = 0; s < g_speed; s++)
        if (STEP_FN[g_alg]()) break;
}

int main(void)
{
    srand((unsigned)time(NULL));
    atexit(cleanup);
    signal(SIGINT,  sig_h);
    signal(SIGTERM, sig_h);
    signal(SIGWINCH, sig_h);

    initscr();
    cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();

    getmaxyx(stdscr, g_rows, g_cols);
    scramble();

    while (!g_quit) {
        long long frame_start = clock_ns();

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
        }

        int ch = getch();
        if (ch != ERR) handle_key(ch);

        step_simulation();

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();

        /* Frame cap — `elapsed = clock_ns() - frame_start` (no +dt!). */
        long long elapsed = clock_ns() - frame_start;
        clock_sleep_ns(FRAME_NS - elapsed);
    }
    return 0;
}
