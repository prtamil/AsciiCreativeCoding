/*
 * sort_vis.c — Sorting Algorithm Visualiser
 *
 * Animated vertical bar chart.  Each element is a column of '#' characters
 * proportional to its value.  One operation (compare or swap) is executed
 * per tick so the algorithm unfolds visibly.
 *
 * Algorithms (cycle with TAB or auto-advance when done):
 *   B  Bubble sort       — O(n²) comparisons, in-place
 *   I  Insertion sort    — O(n²) shifts, stable
 *   S  Selection sort    — O(n²) comparisons, O(n) swaps
 *   Q  Quicksort         — O(n log n) average, Lomuto partition
 *   H  Heapsort          — O(n log n) worst-case, in-place
 *
 * Color: grey=unsorted  yellow=comparing  red=swapped  green=sorted
 *
 * Keys: q quit  TAB next algorithm  SPACE scramble  p pause  +/- speed
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra misc/sort_vis.c \
 *       -o sort_vis -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 sort  §5 draw  §6 app
 */

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

#define N_ELEMS       48
#define HUD_ROWS       3
#define STEPS_DEFAULT  1
#define RENDER_NS      (1000000000LL / 30)

enum Alg { ALG_BUBBLE, ALG_INSERT, ALG_SELECT, ALG_QUICK, ALG_HEAP, ALG_COUNT };
static const char *ALG_NAME[ALG_COUNT] = {
    "Bubble", "Insertion", "Selection", "Quicksort", "Heapsort"
};

enum { CP_NORM=1, CP_CMP, CP_SWP, CP_SORT, CP_HUD };

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
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_NORM, 250, -1);   /* grey         */
        init_pair(CP_CMP,   51, -1);   /* cyan         */
        init_pair(CP_SWP,  196, -1);   /* red          */
        init_pair(CP_SORT,  46, -1);   /* green        */
        init_pair(CP_HUD,  244, -1);
    } else {
        init_pair(CP_NORM, COLOR_WHITE,  -1);
        init_pair(CP_CMP,  COLOR_CYAN,   -1);
        init_pair(CP_SWP,  COLOR_RED,    -1);
        init_pair(CP_SORT, COLOR_GREEN,  -1);
        init_pair(CP_HUD,  COLOR_WHITE,  -1);
    }
}

/* ===================================================================== */
/* §4  sort state machines                                                */
/* ===================================================================== */

static int g_arr[N_ELEMS];
static int g_cmp1 = -1, g_cmp2 = -1;    /* indices being compared   */
static int g_swp1 = -1, g_swp2 = -1;    /* indices just swapped     */
static bool g_done;
static long long g_cmp_count, g_swp_count;

/* Bubble */
static int g_bi, g_bj;

/* Insertion */
static int g_ii, g_ij;

/* Selection */
static int g_si, g_sj, g_smin;

/* Quicksort */
#define QS_STACK 128
static int g_qlo[QS_STACK], g_qhi[QS_STACK], g_qtop;
static int g_ql, g_qh, g_qi, g_qj, g_qphase; /* 0=scan 1=place */

/* Heapsort */
static int g_hn, g_hi_h, g_hphase;  /* 0=build-heap 1=extract */

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
    case ALG_BUBBLE:
        g_bi = 0; g_bj = 0; break;
    case ALG_INSERT:
        g_ii = 1; g_ij = 1; break;
    case ALG_SELECT:
        g_si = 0; g_sj = 1; g_smin = 0; break;
    case ALG_QUICK:
        g_qtop = 0;
        g_ql = 0; g_qh = N_ELEMS - 1;
        g_qi = -1; g_qj = 0;   /* g_qi = g_ql-1 for Lomuto */
        g_qphase = 0;
        break;
    case ALG_HEAP:
        g_hn = N_ELEMS; g_hi_h = N_ELEMS/2 - 1; g_hphase = 0; break;
    default: break;
    }
}

/* Returns: 0=step done, 1=all done */
static int bubble_step(void)
{
    if (g_bi >= N_ELEMS-1) { g_done = true; return 1; }
    g_cmp1 = g_bj; g_cmp2 = g_bj+1; g_swp1 = g_swp2 = -1;
    g_cmp_count++;
    if (g_arr[g_bj] > g_arr[g_bj+1]) arr_swap(g_bj, g_bj+1);
    if (++g_bj >= N_ELEMS-1-g_bi) { g_bj = 0; g_bi++; }
    return 0;
}

static int insert_step(void)
{
    if (g_ii >= N_ELEMS) { g_done = true; return 1; }
    g_cmp1 = g_ij-1; g_cmp2 = g_ij; g_swp1 = g_swp2 = -1;
    g_cmp_count++;
    if (g_ij > 0 && g_arr[g_ij-1] > g_arr[g_ij]) {
        arr_swap(g_ij-1, g_ij);
        g_ij--;
    } else {
        g_ii++;
        g_ij = g_ii;
    }
    return 0;
}

static int select_step(void)
{
    if (g_si >= N_ELEMS-1) { g_done = true; return 1; }
    g_swp1 = g_swp2 = -1;
    if (g_sj < N_ELEMS) {
        g_cmp1 = g_smin; g_cmp2 = g_sj;
        g_cmp_count++;
        if (g_arr[g_sj] < g_arr[g_smin]) g_smin = g_sj;
        g_sj++;
    } else {
        if (g_smin != g_si) arr_swap(g_si, g_smin);
        g_si++; g_sj = g_si+1; g_smin = g_si;
    }
    return 0;
}

/* Lomuto quicksort: one operation per call.
 * Phase 0: scan g_qj from g_ql to g_qh-1 (one compare/swap per call).
 * Phase 1: pivot placed; pop next sub-range from stack. */
static int quick_step(void)
{
    g_cmp1 = g_cmp2 = g_swp1 = g_swp2 = -1;

    if (g_qphase == 0) {
        if (g_qj < g_qh) {
            /* Compare arr[j] with pivot arr[hi] */
            g_cmp1 = g_qj; g_cmp2 = g_qh;
            g_cmp_count++;
            if (g_arr[g_qj] <= g_arr[g_qh]) {
                g_qi++;
                arr_swap(g_qi, g_qj);
            }
            g_qj++;
            return 0;
        }
        /* Scan done — place pivot at g_qi+1 */
        g_qi++;
        if (g_qi != g_qh) arr_swap(g_qi, g_qh);
        /* Push left sub-range [g_ql .. g_qi-1] */
        if (g_ql < g_qi - 1 && g_qtop < QS_STACK - 1)
            { g_qlo[g_qtop] = g_ql;   g_qhi[g_qtop] = g_qi-1; g_qtop++; }
        /* Push right sub-range [g_qi+1 .. g_qh] */
        if (g_qi + 1 < g_qh && g_qtop < QS_STACK - 1)
            { g_qlo[g_qtop] = g_qi+1; g_qhi[g_qtop] = g_qh;   g_qtop++; }
        g_qphase = 1;
        return 0;
    }

    /* Phase 1: pop next valid range */
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
    g_done = true; return 1;
}

/* Heapsort: full sift-down from root to end (max-heap) */
static void heap_sift(int root, int end)
{
    while (1) {
        int largest = root;
        int l = 2*root+1, r = 2*root+2;
        g_cmp_count++;
        if (l <= end && g_arr[l] > g_arr[largest]) largest = l;
        if (r <= end && g_arr[r] > g_arr[largest]) largest = r;
        g_cmp1 = root; g_cmp2 = largest;
        if (largest == root) break;     /* heap property satisfied */
        arr_swap(root, largest);
        root = largest;                 /* follow the swap down */
    }
}

static int heap_step(void)
{
    g_cmp1 = g_cmp2 = g_swp1 = g_swp2 = -1;
    if (g_hphase == 0) {
        /* build max-heap */
        if (g_hi_h >= 0) { heap_sift(g_hi_h--, N_ELEMS-1); return 0; }
        g_hphase = 1; g_hn = N_ELEMS-1;
    }
    if (g_hn <= 0) { g_done = true; return 1; }
    arr_swap(0, g_hn);
    heap_sift(0, --g_hn);
    return 0;
}

typedef int (*StepFn)(void);
static StepFn STEP_FN[ALG_COUNT] = {
    bubble_step, insert_step, select_step, quick_step, heap_step
};

/* ===================================================================== */
/* §5  draw                                                               */
/* ===================================================================== */

static int  g_rows, g_cols;
static bool g_paused;
static int  g_speed = STEPS_DEFAULT;

static enum Alg g_alg = ALG_BUBBLE;

static void scramble(void)
{
    for (int i = 0; i < N_ELEMS; i++) g_arr[i] = i + 1;
    for (int i = N_ELEMS-1; i > 0; i--) {
        int j = rand() % (i+1);
        int t = g_arr[i]; g_arr[i] = g_arr[j]; g_arr[j] = t;
    }
    init_alg(g_alg);
}

static void scene_draw(void)
{
    int bar_area  = g_rows - HUD_ROWS;
    int bar_max   = bar_area > 1 ? bar_area - 1 : 1;
    int col_w     = g_cols / N_ELEMS;
    if (col_w < 1) col_w = 1;

    for (int i = 0; i < N_ELEMS; i++) {
        int bar_h   = g_arr[i] * bar_max / N_ELEMS;
        int col_s   = i * g_cols / N_ELEMS;
        int col_e   = (i+1) * g_cols / N_ELEMS;
        if (col_e > g_cols) col_e = g_cols;

        int cp;
        if      (i == g_swp1 || i == g_swp2) cp = CP_SWP;
        else if (i == g_cmp1 || i == g_cmp2) cp = CP_CMP;
        else if (g_done)                       cp = CP_SORT;
        else                                   cp = CP_NORM;

        for (int row = g_rows - 1; row >= HUD_ROWS; row--) {
            int depth = g_rows - 1 - row;
            chtype ch = (depth < bar_h) ? '#' : ' ';
            attron(COLOR_PAIR(cp) | (cp == CP_SWP ? A_BOLD : 0));
            for (int c = col_s; c < col_e; c++)
                mvaddch(row, c, ch);
            attroff(COLOR_PAIR(cp) | A_BOLD);
        }
    }

    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " SortVis  q:quit  TAB:next alg  SPACE:scramble  p:pause  +/-:speed");
    mvprintw(1, 0,
        " Algorithm: %-12s  comparisons:%lld  swaps:%lld",
        ALG_NAME[g_alg], g_cmp_count, g_swp_count);
    mvprintw(2, 0,
        " speed:%dx  N=%d  %s",
        g_speed, N_ELEMS,
        g_done ? "SORTED" : (g_paused ? "PAUSED" : "sorting"));
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §6  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

int main(void)
{
    srand((unsigned)time(NULL));
    atexit(cleanup);
    signal(SIGINT, sig_h); signal(SIGTERM, sig_h); signal(SIGWINCH, sig_h);

    initscr(); cbreak(); noecho();
    keypad(stdscr, TRUE); nodelay(stdscr, TRUE);
    curs_set(0); typeahead(-1);
    color_init();
    getmaxyx(stdscr, g_rows, g_cols);
    scramble();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1; break;
        case ' ': scramble(); break;
        case 'p': case 'P': g_paused = !g_paused; break;
        case '\t':
            g_alg = (enum Alg)((g_alg + 1) % ALG_COUNT);
            scramble();
            break;
        case '+': case '=': g_speed *= 2; if (g_speed > 256) g_speed = 256; break;
        case '-': g_speed /= 2; if (g_speed < 1) g_speed = 1; break;
        default: break;
        }

        long long now = clock_ns();

        if (!g_paused && !g_done) {
            for (int s = 0; s < g_speed; s++) {
                if (STEP_FN[g_alg]()) break;
            }
        }

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();
        clock_sleep_ns(RENDER_NS - (clock_ns() - now));
    }
    return 0;
}
