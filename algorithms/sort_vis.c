/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sort_vis.c — watch five classic sorts race a row of bars, one
 * compare-or-swap per tick (bubble, insertion, selection, quicksort,
 * heapsort).  Sister demo: procedural/generational/maze.c uses the same
 * "one visible step per frame" trick.
 *
 * Keys: q/ESC quit  TAB next sort  space scramble  p pause  +/- speed
 * Build: gcc -std=c11 -O2 -Wall -Wextra algorithms/sort_vis.c -o sort_vis -lncurses -lm
 */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — tunable constants, the algorithm list, and colour IDs ── */

#define N_ELEMS         48
#define HUD_ROWS         3      /* top 3 rows kept clear for the status bar */
#define STEPS_DEFAULT    1
#define STEPS_MAX      256

#define TARGET_FPS      30
#define NS_PER_SEC      1000000000LL
#define FRAME_NS        (NS_PER_SEC / TARGET_FPS)

#define QS_STACK       128      /* how many pending quicksort ranges we can stack */

enum Alg {
    ALG_BUBBLE, ALG_INSERT, ALG_SELECT, ALG_QUICK, ALG_HEAP, ALG_COUNT
};
static const char *ALG_NAME[ALG_COUNT] = {
    "Bubble", "Insertion", "Selection", "Quicksort", "Heapsort"
};

/* Quicksort runs in two alternating modes; names beat raw 0/1. */
enum { QS_PHASE_SCAN = 0, QS_PHASE_POP = 1 };

/* Heapsort likewise has a build mode then an extract mode. */
enum { HEAP_PHASE_BUILD = 0, HEAP_PHASE_EXTRACT = 1 };

/* Colour-pair IDs.  8/9 are the project-wide HUD slots reused by every demo. */
enum {
    PAIR_NORM = 1,    /* untouched bars, light grey                  */
    PAIR_CMP,         /* current compare pair, gold                  */
    PAIR_SWP,         /* just-swapped pair, bright red + A_BOLD      */
    PAIR_SORT,        /* sort complete, matrix green + A_BOLD        */
    PAIR_HUD  = 8,    /* top status line, yellow + A_BOLD            */
    PAIR_HINT = 9     /* bottom key hint, cyan + A_BOLD              */
};

/* ── §2 clock — monotonic nanosecond timer + a sleep for frame pacing ── */

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

/* ── §3 color — set up the bar / compare / swap / sorted / HUD colours ── */

/*
 * Every pair keeps the terminal's own background (-1) and uses a bright
 * foreground so it stays readable on any theme.  Compare-gold (220) is
 * kept distinct from HUD-yellow (226) so a glowing bar never blends into
 * the status line.  Falls back to the 8 basic colours on old terminals.
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

/* ── §4 data types — the per-tick highlight hints, the five sorters'    */
/*    saved cursors, and the one Scene struct that owns everything ────── */

/*
 * Everything the program remembers hangs off a single Scene: the array
 * being sorted, which sort is running, that sort's saved cursors, the
 * highlight hints for this frame, the HUD counters, and the UI flags.
 * Only the two signal flags (g_quit, g_resize) live outside Scene, because
 * a signal handler has no way to receive a pointer to it.
 */

/*
 * RenderHints — the sort's note to the renderer: "highlight these bars
 * this frame".  Each step fills it in before returning; the next frame
 * reads it back.  cmp1/cmp2 glow gold (a comparison), swp1/swp2 glow red
 * (a swap).  A slot holding -1 means "nothing to highlight there".
 *
 * Members
 *   cmp1, cmp2   the two bars being compared this tick; -1 if none
 *   swp1, swp2   the two bars just swapped this tick;   -1 if none
 *
 * Range/invariants: each value is -1 or a valid arr[] index (0..N-1), and
 * the two indices in a pair are never equal.
 */
typedef struct {
    int cmp1, cmp2;   /* gold pair      (-1 = no compare this frame) */
    int swp1, swp2;   /* red-bold pair  (-1 = no swap    this frame) */
} RenderHints;

/*
 * The five *State structs below are the trick that makes this whole demo
 * work.  Normally a sort runs start to finish in one function call, so you
 * never see the middle.  Here each sort is turned inside-out: we save its
 * loop counters (its "where was I?" bookmark) in a struct, do just ONE
 * compare-or-swap per call, then return so a frame can be drawn.  Next call
 * picks up exactly where it left off.  Each struct holds precisely the
 * variables needed to resume.
 */

/*
 * BubbleState — bubble sort's bookmark.  Bubble sort sweeps left to right
 * over and over, and each sweep floats the next-largest value to the right
 * end, so the sorted region grows from the right.
 *
 * Members
 *   i   which sweep we're on, 0..N-2.  After sweep i, the last i+1 bars
 *       are in their final places and never touched again.
 *   j   position within the current sweep; each tick compares arr[j] with
 *       its right neighbour arr[j+1] and swaps them if out of order.
 */
typedef struct {
    int i, j;
} BubbleState;

/*
 * InsertState — insertion sort's bookmark.  Like sorting a hand of cards:
 * pick up the next card and slide it left until it sits among the ones you
 * already arranged.  The left part of the array stays sorted as it grows.
 *
 * Members
 *   i   the next card to place, 1..N-1.  Everything left of i is already
 *       in order when we pick up a new i.
 *   j   where that card currently sits as it slides left.  Starts at i and
 *       drops by one each tick while the bar to its left is larger; stops
 *       when the left neighbour is no bigger, or when it reaches the front.
 */
typedef struct {
    int i, j;
} InsertState;

/*
 * SelectState — selection sort's bookmark.  Each pass scans the unsorted
 * part for the smallest bar, then drops it into the next sorted slot.  It
 * does lots of comparisons but very few swaps (at most one per pass), which
 * is its claim to fame.
 *
 * Members
 *   i     boundary of the sorted region, 0..N-1.  Everything left of i is
 *         placed and is no larger than anything to its right.
 *   j     scan position sweeping the unsorted part hunting for the minimum.
 *   min   index of the smallest bar seen so far this pass; starts at i and
 *         updates whenever a smaller bar turns up.
 */
typedef struct {
    int i, j, min;
} SelectState;

/*
 * QuickState — quicksort's bookmark.  Quicksort picks a pivot, throws
 * smaller items to its left and bigger to its right, then repeats on each
 * side.  Normally that "repeat on each side" is recursion, but a recursive
 * call would finish the whole sort before we could draw a single frame.  So
 * instead we keep our OWN to-do list of sub-ranges still needing a sort
 * (lo[]/hi[]) and process one tick at a time.  We use the Lomuto scheme,
 * which always parks the pivot at the right end of the current range.
 *
 * Each range is handled in two modes: SCAN walks across it sorting items
 * relative to the pivot; POP drops the pivot in its final spot, files the
 * two leftover sub-ranges on the to-do list, and pulls the next one.
 *
 * Members
 *   lo[QS_STACK], hi[QS_STACK]   the to-do list: low/high ends of ranges
 *                                still waiting to be sorted
 *   top                          how many entries the to-do list holds
 *                                (also the next free slot)
 *   cur_lo, cur_hi               the range being worked right now; the
 *                                pivot sits at cur_hi
 *   i                            right edge of the "<= pivot" group built so far
 *   j                            scan position sweeping cur_lo toward cur_hi
 *   phase                        QS_PHASE_SCAN or QS_PHASE_POP (see above)
 *
 * The to-do list never overflows for N this small; QS_STACK = 128 is plenty
 * for N_ELEMS = 48.
 */
typedef struct {
    int lo[QS_STACK];
    int hi[QS_STACK];
    int top;
    int cur_lo, cur_hi;
    int i, j;
    int phase;
} QuickState;

/*
 * HeapState — heapsort's bookmark.  Heapsort treats the array as a binary
 * tree where every parent is bigger than its children (a "max-heap"), so the
 * biggest value is always at the front.  It works in two stages: first build
 * the heap, then repeatedly pull the front (the max) to the back and shrink
 * the heap by one — which leaves the array sorted, growing from the right.
 *
 * Members
 *   n           how many bars are still part of the heap.  Starts at N_ELEMS
 *               and drops by one each time we pull off the max.
 *   sift_from   which node we sink down next.  Building counts it down toward
 *               the root; extracting always sinks from the root (0).
 *   phase       HEAP_PHASE_BUILD while forming the heap,
 *               HEAP_PHASE_EXTRACT while pulling off maxima.
 */
typedef struct {
    int n;
    int sift_from;
    int phase;
} HeapState;

/*
 * AlgState — holds all five sorters' bookmarks side by side.  Only the one
 * matching the current algorithm is meaningful at any moment; the other four
 * just sit there untouched until you TAB to them (which scrambles and resets
 * the new one).  We keep them as separate fields rather than a union so each
 * stays its own named type — easier to read and to inspect in a debugger,
 * and the few hundred wasted bytes don't matter here.
 *
 * Members
 *   bubble, insert, select, quick, heap — one bookmark per sort
 */
typedef struct {
    BubbleState bubble;
    InsertState insert;
    SelectState select;
    QuickState  quick;
    HeapState   heap;
} AlgState;

/*
 * Scene — the single bag of state for one run, passed by pointer to nearly
 * every function.  A function that only reads it takes `const Scene *`; one
 * that changes it takes `Scene *`, so the signature alone tells you whether
 * it mutates.  Keeping it all in one struct (rather than scattered globals)
 * means there's exactly one place to look for "what state exists".
 *
 * Members
 *   arr[N_ELEMS]      the bars being sorted (a shuffle of the values 1..N)
 *   alg               which sort is running (ALG_BUBBLE..ALG_HEAP)
 *   algs              all five sorters' bookmarks; one is active
 *   hints             which bars to highlight gold/red this frame
 *   cmp_count         running compare tally shown in the HUD
 *   swp_count         running swap tally shown in the HUD
 *   done              true once the active sort reports it finished
 *   paused            true freezes the sort (toggled with 'p')
 *   speed             how many sort steps to run per frame, 1..STEPS_MAX
 *   rows, cols        terminal size grabbed at the start of this frame
 *
 * arr[] always stays a permutation of 1..N (the shuffle never adds or drops
 * a value), and `done` is only true once arr[] is fully in order.
 */
typedef struct {
    int          arr[N_ELEMS];
    enum Alg     alg;
    AlgState     algs;
    RenderHints  hints;
    long long    cmp_count;
    long long    swp_count;
    bool         done;
    bool         paused;
    int          speed;
    int          rows;
    int          cols;
} Scene;

/* ── §5 sort — the five sorters, each doing one step per call ────────── */

/* ── shared helpers used by every sorter ─────────────────────────────── */

/* Forget any "comparing" highlight (for a tick that doesn't compare). */
static void hints_clear_compare_pair(RenderHints *h) { h->cmp1 = h->cmp2 = -1; }

/* Forget any "swapped" highlight (for a tick that doesn't swap). */
static void hints_clear_swap_pair(RenderHints *h)    { h->swp1 = h->swp2 = -1; }

/* Flag the sort as finished.  Returns 1 so a step can `return
 * mark_sort_complete(sc);` — 1 is the agreed "I'm done" signal. */
static int mark_sort_complete(Scene *sc) { sc->done = true; return 1; }

/* Highlight bars a and b as "being compared" and bump the compare tally. */
static void record_compare(Scene *sc, int a, int b)
{
    sc->hints.cmp1 = a; sc->hints.cmp2 = b;
    sc->cmp_count++;
}

/*
 * Swap two bars and mark them red so the swap flashes on screen; also
 * bumps the swap tally.  Every sorter goes through here to swap.
 */
static void arr_swap(Scene *sc, int a, int b)
{
    int t = sc->arr[a]; sc->arr[a] = sc->arr[b]; sc->arr[b] = t;
    sc->hints.swp1 = a; sc->hints.swp2 = b;
    sc->swp_count++;
}

/* ── per-algorithm cursor seeding ────────────────────────────────────── */

/* Bubble sort begins at pass i=0, scan position j=0. */
static void bubble_state_seed(BubbleState *b)
{
    *b = (BubbleState){ .i = 0, .j = 0 };
}

/* Insertion sort treats the first bar as already sorted; the first bar to
 * place is arr[1], sliding from position 1. */
static void insert_state_seed(InsertState *ins)
{
    *ins = (InsertState){ .i = 1, .j = 1 };
}

/* Selection sort begins with nothing placed yet; the scan starts at index
 * 1 and the current minimum is index 0. */
static void select_state_seed(SelectState *sel)
{
    *sel = (SelectState){ .i = 0, .j = 1, .min = 0 };
}

/* Quicksort begins with one to-do range covering the whole array.  i starts
 * one below the range so the first value at-or-below the pivot lands it on
 * the range's first slot. */
static void quick_state_seed(QuickState *q)
{
    *q = (QuickState){
        .top    = 0,
        .cur_lo = 0,    .cur_hi = N_ELEMS - 1,
        .i      = -1,   .j      = 0,            /* start one below the range */
        .phase  = QS_PHASE_SCAN,
    };
}

/* Heapsort begins building the heap from the lowest non-leaf node upward
 * to the root (the cheap, bottom-up way to heapify). */
static void heap_state_seed(HeapState *h)
{
    *h = (HeapState){
        .n         = N_ELEMS,
        .sift_from = N_ELEMS / 2 - 1,
        .phase     = HEAP_PHASE_BUILD,
    };
}

/* Reset the per-run counters + hints + done flag.  Called by init_alg
 * before any algorithm-specific cursor seeding. */
static void run_counters_reset(Scene *sc)
{
    sc->hints     = (RenderHints){ -1, -1, -1, -1 };
    sc->cmp_count = 0;
    sc->swp_count = 0;
    sc->done      = false;
}

/*
 * Reset the counters and rewind the chosen sort to its starting bookmark,
 * so it runs cleanly from the top.  Only that sort's bookmark is touched.
 */
static void init_alg(Scene *sc, enum Alg alg)
{
    run_counters_reset(sc);
    switch (alg) {
    case ALG_BUBBLE: bubble_state_seed(&sc->algs.bubble); break;
    case ALG_INSERT: insert_state_seed(&sc->algs.insert); break;
    case ALG_SELECT: select_state_seed(&sc->algs.select); break;
    case ALG_QUICK:  quick_state_seed (&sc->algs.quick);  break;
    case ALG_HEAP:   heap_state_seed  (&sc->algs.heap);   break;
    default: break;
    }
}

/* ── bubble sort — swap neighbours that are out of order, sweep, repeat ─ */

/* Done once we've made enough sweeps to place every bar. */
static bool bubble_all_passes_done(const BubbleState *b)
{
    return b->i >= N_ELEMS - 1;
}

/* Compare a bar with its right neighbour and swap them if they're backwards. */
static void bubble_compare_adjacent_pair(Scene *sc, const BubbleState *b)
{
    record_compare(sc, b->j, b->j + 1);
    hints_clear_swap_pair(&sc->hints);
    if (sc->arr[b->j] > sc->arr[b->j + 1])
        arr_swap(sc, b->j, b->j + 1);
}

/* Step right; when we reach the end of the still-unsorted part (which shrinks
 * by one each sweep), start a new sweep from the left. */
static void bubble_advance_cursor(BubbleState *b)
{
    if (++b->j >= N_ELEMS - 1 - b->i) { b->j = 0; b->i++; }
}

/* One bubble-sort tick: compare a neighbouring pair, maybe swap, then step. */
static int bubble_step(Scene *sc)
{
    BubbleState *b = &sc->algs.bubble;
    if (bubble_all_passes_done(b)) return mark_sort_complete(sc);

    bubble_compare_adjacent_pair(sc, b);
    bubble_advance_cursor(b);
    return 0;
}

/* ── insertion sort — slide each bar left into the sorted part ────────── */

/* Done once every bar has been picked up and placed. */
static bool insert_all_done(const InsertState *ins)
{
    return ins->i >= N_ELEMS;
}

/* Does the bar still need to slide?  True when the bar to its left is bigger
 * (and there is a left neighbour).  This is the tick's one comparison. */
static bool insert_left_neighbour_is_bigger(Scene *sc, const InsertState *ins)
{
    record_compare(sc, ins->j - 1, ins->j);
    hints_clear_swap_pair(&sc->hints);
    return ins->j > 0 && sc->arr[ins->j - 1] > sc->arr[ins->j];
}

/* Slide the bar one place to the left (swap with its left neighbour). */
static void insert_slide_one_cell_left(Scene *sc, InsertState *ins)
{
    arr_swap(sc, ins->j - 1, ins->j);
    ins->j--;
}

/* This bar is settled; move on to pick up the next one. */
static void insert_pick_next_candidate(InsertState *ins)
{
    ins->i++;
    ins->j = ins->i;
}

/* One insertion-sort tick: either slide the current bar left, or, if it's
 * settled, pick up the next one. */
static int insert_step(Scene *sc)
{
    InsertState *ins = &sc->algs.insert;
    if (insert_all_done(ins)) return mark_sort_complete(sc);

    if (insert_left_neighbour_is_bigger(sc, ins))
        insert_slide_one_cell_left(sc, ins);
    else
        insert_pick_next_candidate(ins);
    return 0;
}

/* ── selection sort — find the smallest of the rest, drop it in place ─── */

/* Done once all but the last slot are filled (the last bar is the biggest). */
static bool select_all_done(const SelectState *sel)
{
    return sel->i >= N_ELEMS - 1;
}

/* Still scanning the unsorted part for this pass's smallest bar? */
static bool select_scan_in_progress(const SelectState *sel)
{
    return sel->j < N_ELEMS;
}

/* Look at the next bar; if it's smaller than the smallest seen so far,
 * remember it.  Then step the scan forward. */
static void select_update_running_min(Scene *sc, SelectState *sel)
{
    record_compare(sc, sel->min, sel->j);
    if (sc->arr[sel->j] < sc->arr[sel->min]) sel->min = sel->j;
    sel->j++;
}

/* Pass finished: swap the smallest bar into the next sorted slot, then start
 * the next pass over what remains. */
static void select_emit_min_and_restart_scan(Scene *sc, SelectState *sel)
{
    if (sel->min != sel->i) arr_swap(sc, sel->i, sel->min);
    sel->i++;
    sel->j   = sel->i + 1;
    sel->min = sel->i;
}

/* One selection-sort tick: either look at one more bar while hunting the
 * minimum, or, if the pass just ended, drop the minimum into place. */
static int select_step(Scene *sc)
{
    SelectState *sel = &sc->algs.select;
    if (select_all_done(sel)) return mark_sort_complete(sc);
    hints_clear_swap_pair(&sc->hints);

    if (select_scan_in_progress(sel))
        select_update_running_min(sc, sel);
    else
        select_emit_min_and_restart_scan(sc, sel);
    return 0;
}

/* ── quicksort: the to-do list of ranges (our stand-in for recursion) ── */

/* File a range on the to-do list.  Single-element (or empty) ranges are
 * already sorted, so we skip them; the list also can't overflow here. */
static void range_stack_push(QuickState *q, int lo, int hi)
{
    if (lo >= hi || q->top >= QS_STACK - 1) return;
    q->lo[q->top] = lo;
    q->hi[q->top] = hi;
    q->top++;
}

/* Take the next range off the to-do list, skipping any down to one element.
 * Returns false (and the sort is finished) when the list is empty. */
static bool range_stack_pop_next_valid(QuickState *q, int *out_lo, int *out_hi)
{
    while (q->top > 0) {
        q->top--;
        int lo = q->lo[q->top], hi = q->hi[q->top];
        if (lo < hi) { *out_lo = lo; *out_hi = hi; return true; }
    }
    return false;
}

/* ── quicksort: the partition step (throw items left/right of the pivot) ─ */

/* Start partitioning a fresh range.  i sits just before it so the first
 * small-enough bar lands on the range's first slot; j starts the scan. */
static void lomuto_prime_partition(QuickState *q, int lo, int hi)
{
    q->cur_lo = lo;  q->cur_hi = hi;
    q->i      = lo - 1;
    q->j      = lo;
    q->phase  = QS_PHASE_SCAN;
}

/* Look at one bar: if it's no bigger than the pivot (which sits at the right
 * end), swap it into the growing "small" group on the left.  Step the scan. */
static void lomuto_partition_tick(Scene *sc, QuickState *q)
{
    record_compare(sc, q->j, q->cur_hi);
    if (sc->arr[q->j] <= sc->arr[q->cur_hi]) {
        q->i++;
        arr_swap(sc, q->i, q->j);
    }
    q->j++;
}

/* Scan done: drop the pivot into the gap between the small and big groups
 * (its final resting place), then file the two leftover sides as new ranges. */
static void lomuto_finalize_pivot(Scene *sc, QuickState *q)
{
    q->i++;
    if (q->i != q->cur_hi) arr_swap(sc, q->i, q->cur_hi);
    range_stack_push(q, q->cur_lo, q->i - 1);    /* left  sub-range */
    range_stack_push(q, q->i + 1,  q->cur_hi);   /* right sub-range */
    q->phase = QS_PHASE_POP;
}

/* Still bars left to scan in the current range? */
static bool quick_partition_in_progress(const QuickState *q)
{
    return q->j < q->cur_hi;
}

/* One quicksort tick: while partitioning, look at the next bar (or, when the
 * scan ends, place the pivot); otherwise pull the next range off the to-do
 * list, or finish if there are none left. */
static int quick_step(Scene *sc)
{
    QuickState *q = &sc->algs.quick;
    hints_clear_compare_pair(&sc->hints);
    hints_clear_swap_pair   (&sc->hints);

    if (q->phase == QS_PHASE_SCAN) {
        if (quick_partition_in_progress(q))
            lomuto_partition_tick(sc, q);
        else
            lomuto_finalize_pivot(sc, q);
        return 0;
    }

    int lo, hi;
    if (range_stack_pop_next_valid(q, &lo, &hi)) {
        lomuto_prime_partition(q, lo, hi);
        return 0;
    }
    return mark_sort_complete(sc);
}

/*
 * Sink a too-small value down the heap until every parent is at least as big
 * as its children again.  We let this run to completion in ONE tick (the only
 * sorter that does several swaps at once) — pausing mid-sink would briefly
 * show an invalid heap on screen.  `end` is the last index still in the heap;
 * anything past it is already-sorted and off-limits.
 */
static void heap_sift(Scene *sc, int root, int end)
{
    while (1) {
        int largest = root;
        int l = 2 * root + 1, r = 2 * root + 2;
        sc->cmp_count++;
        if (l <= end && sc->arr[l] > sc->arr[largest]) largest = l;
        if (r <= end && sc->arr[r] > sc->arr[largest]) largest = r;
        sc->hints.cmp1 = root; sc->hints.cmp2 = largest;
        if (largest == root) break;
        arr_swap(sc, root, largest);
        root = largest;
    }
}

/* ── heapsort: where are we in the build / extract stages? ───────────── */

/* Still building the heap (working node by node up to the root)? */
static bool heap_build_in_progress(const HeapState *h)
{
    return h->sift_from >= 0;
}

/* Extract stage has emptied the heap — the array is fully sorted. */
static bool heap_extract_done(const HeapState *h)
{
    return h->n <= 0;
}

/* ── heapsort: one build step, one extract step ──────────────────────── */

/* Build tick: heapify the subtree at the current node, then move one node
 * closer to the root. */
static void heap_build_sift_next(Scene *sc, HeapState *h)
{
    heap_sift(sc, h->sift_from, N_ELEMS - 1);
    h->sift_from--;
}

/* Heap is fully built; switch to pulling maxima off it. */
static void heap_transition_to_extract(HeapState *h)
{
    h->phase = HEAP_PHASE_EXTRACT;
    h->n     = N_ELEMS - 1;
}

/* Extract tick: move the front bar (the biggest) to the back, shrink the
 * heap by one, then re-sink the new front to restore the heap. */
static void heap_extract_one_max(Scene *sc, HeapState *h)
{
    arr_swap(sc, 0, h->n);
    h->n--;
    heap_sift(sc, 0, h->n);
}

/* One heapsort tick: during the build stage, heapify one more node; once
 * built, pull one max off the heap each tick (the sorted part grows from the
 * right) until nothing is left. */
static int heap_step(Scene *sc)
{
    HeapState *h = &sc->algs.heap;
    hints_clear_compare_pair(&sc->hints);
    hints_clear_swap_pair   (&sc->hints);

    if (h->phase == HEAP_PHASE_BUILD) {
        if (heap_build_in_progress(h)) {
            heap_build_sift_next(sc, h);
            return 0;
        }
        heap_transition_to_extract(h);
    }
    if (heap_extract_done(h)) return mark_sort_complete(sc);
    heap_extract_one_max(sc, h);
    return 0;
}

/* Lookup table from algorithm enum to its step function. */
typedef int (*StepFn)(Scene *);
static const StepFn STEP_FN[ALG_COUNT] = {
    bubble_step, insert_step, select_step, quick_step, heap_step
};

/* ── §7 scene — shuffle the array, then draw the bars and the HUD ────── */

/* Put the values 1..N back in order (the starting point before shuffling). */
static void arr_fill_identity(Scene *sc)
{
    for (int i = 0; i < N_ELEMS; i++) sc->arr[i] = i + 1;
}

/* Fisher-Yates shuffle: walk from the back, swapping each bar with a randomly
 * chosen one at or before it.  Gives every ordering an equal chance. */
static void arr_fisher_yates_shuffle(Scene *sc)
{
    for (int i = N_ELEMS - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int t = sc->arr[i]; sc->arr[i] = sc->arr[j]; sc->arr[j] = t;
    }
}

/* Start a fresh run: refill 1..N, shuffle, and rewind the current sort. */
static void scramble(Scene *sc)
{
    arr_fill_identity(sc);
    arr_fisher_yates_shuffle(sc);
    init_alg(sc, sc->alg);
}

/*
 * Draw one character, clipped to the screen and coloured.  The
 * (chtype)(unsigned char) cast keeps bytes above 0x7F from being read as
 * negative (which ncurses would treat as a control code).  All bar drawing
 * funnels through here, so the clip and the cast live in just one place.
 */
static void mark_cell(const Scene *sc, int sr, int sx, char ch,
                      int pair, int attr)
{
    if (sr < 0 || sr >= sc->rows) return;
    if (sx < 0 || sx >= sc->cols) return;
    chtype c = (chtype)(unsigned char)ch;
    if (pair) c |= (chtype)COLOR_PAIR(pair);
    if (attr) c |= (chtype)attr;
    mvaddch(sr, sx, c);
}

/*
 * Pick a bar's colour.  A just-swapped bar wins (red), then a being-compared
 * bar (gold), then once everything is sorted all bars go green; otherwise
 * plain grey.  Swap beats compare so a swap still flashes red for its one
 * frame even if the sort immediately starts a new comparison.
 */
static int bar_pair(const Scene *sc, int i)
{
    if (i == sc->hints.swp1 || i == sc->hints.swp2) return PAIR_SWP;
    if (i == sc->hints.cmp1 || i == sc->hints.cmp2) return PAIR_CMP;
    if (sc->done)                                   return PAIR_SORT;
    return PAIR_NORM;
}

/* ── bar geometry: turn a value into a height and a column span ──────── */

/* Scale a value (1..N) to a bar height in rows; bar_max is the height the
 * biggest value gets. */
static int bar_value_to_cell_height(int value, int bar_max)
{
    return value * bar_max / N_ELEMS;
}

/* Work out which columns bar i covers, [col_s, col_e).  Bars are spread
 * evenly across the width; the last one is clamped to the right edge. */
static void bar_column_span(int i, int cols, int *out_s, int *out_e)
{
    int s = i       * cols / N_ELEMS;
    int e = (i + 1) * cols / N_ELEMS;
    if (e > cols) e = cols;
    *out_s = s;  *out_e = e;
}

/* Swapped and sorted bars get bold to draw the eye; the rest stay plain. */
static int bar_attr_for_pair(int pair)
{
    return (pair == PAIR_SWP || pair == PAIR_SORT) ? A_BOLD : 0;
}

/* Fill one bar's columns with '#' from the bottom row up, for bar_h rows. */
static void paint_bar_column(const Scene *sc, int col_s, int col_e,
                             int bar_h, int pair, int attr)
{
    for (int row = sc->rows - 1; row >= HUD_ROWS; row--) {
        int depth_from_bottom = sc->rows - 1 - row;
        if (depth_from_bottom >= bar_h) break;   /* reached the bar's top */
        for (int c = col_s; c < col_e; c++)
            mark_cell(sc, row, c, '#', pair, attr);
    }
}

/* Draw one bar: its height, its columns, its colour, then fill it in. */
static void draw_bar(const Scene *sc, int i, int bar_max)
{
    int bar_h = bar_value_to_cell_height(sc->arr[i], bar_max);
    int col_s, col_e;
    bar_column_span(i, sc->cols, &col_s, &col_e);

    int pair = bar_pair(sc, i);
    int attr = bar_attr_for_pair(pair);
    paint_bar_column(sc, col_s, col_e, bar_h, pair, attr);
}

/* Height for the tallest bar, leaving the top HUD_ROWS rows clear.  Guards
 * tiny terminals where there's barely any room. */
static int bar_area_max_height(const Scene *sc)
{
    int bar_area = sc->rows - HUD_ROWS;
    return bar_area > 1 ? bar_area - 1 : 1;
}

/* Draw all the bars (the scale is the same for every one). */
static void draw_bars(const Scene *sc)
{
    int bar_max = bar_area_max_height(sc);
    for (int i = 0; i < N_ELEMS; i++) draw_bar(sc, i, bar_max);
}

/* ── HUD: the top status line and the bottom key hints ───────────────── */

/* The one-word state shown on the status line. */
static const char *run_state_label(const Scene *sc)
{
    if (sc->done)   return "SORTED";
    if (sc->paused) return "PAUSED";
    return "running";
}

/* Build the status string, e.g. "Bubble  speed:4x  N=48  cmp:123  swp:11
 * running".  buf needs room for ~80 chars. */
static void format_hud_status(const Scene *sc, char *buf, size_t bufsz)
{
    snprintf(buf, bufsz,
             " %s  speed:%dx  N=%d  cmp:%lld  swp:%lld  %s ",
             ALG_NAME[sc->alg], sc->speed, N_ELEMS,
             sc->cmp_count, sc->swp_count, run_state_label(sc));
}

/* Draw the status string right-aligned on the top row. */
static void draw_top_status(const Scene *sc, const char *status)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    int pad = sc->cols - (int)strlen(status);
    if (pad < 0) pad = 0;
    mvprintw(0, pad, "%s", status);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Draw the key-hint strip along the bottom row. */
static void draw_bottom_hint(const Scene *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
        " q:quit  TAB:next-alg  spc:scramble  p:pause  +/-:speed ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* Draw the whole HUD: status on top, key hints on the bottom row. */
static void draw_hud(const Scene *sc)
{
    char status[80];
    format_hud_status(sc, status, sizeof status);
    draw_top_status(sc, status);
    draw_bottom_hint(sc);
}

/* Draw a frame: bars first, then the HUD on top so its text always shows. */
static void scene_draw(const Scene *sc)
{
    draw_bars(sc);
    draw_hud(sc);
}

/* ── §8 app — signals, resize, input, and the main frame loop ────────── */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

/*
 * handle_input — translate one key code into a Scene mutation.
 *
 *   q / Q / ESC  → request quit (set g_quit)
 *   SPACE        → re-scramble (resets active algorithm cursors too)
 *   p / P        → toggle pause
 *   TAB          → cycle to next algorithm AND re-scramble (fresh run)
 *   + / =        → double playback speed (capped at STEPS_MAX)
 *   -            → halve playback speed (floor 1)
 */
static void handle_input(Scene *sc, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: g_quit = 1;             break;
    case ' ':                    scramble(sc);           break;
    case 'p': case 'P':          sc->paused = !sc->paused; break;
    case '\t':
        sc->alg = (enum Alg)((sc->alg + 1) % ALG_COUNT);
        scramble(sc);
        break;
    case '+': case '=':
        sc->speed *= 2;
        if (sc->speed > STEPS_MAX) sc->speed = STEPS_MAX;
        break;
    case '-':
        sc->speed /= 2;
        if (sc->speed < 1) sc->speed = 1;
        break;
    default: break;
    }
}

/* Nothing to do this frame when paused or already finished. */
static bool simulation_idle(const Scene *sc)
{
    return sc->paused || sc->done;
}

/*
 * Run the active sort for `speed` steps this frame, stopping early if it
 * finishes.  speed is the playback tempo: 1 shows every single operation;
 * higher values batch more per frame so the fast sorts don't crawl.
 */
static void step_simulation(Scene *sc)
{
    if (simulation_idle(sc)) return;
    StepFn tick = STEP_FN[sc->alg];
    for (int s = 0; s < sc->speed; s++)
        if (tick(sc)) break;
}

/* ── one-time setup helpers for main() ───────────────────────────────── */

/* Hook up the signal handlers and the exit cleanup.  Ctrl-C / kill ask for
 * a clean shutdown; a terminal resize is noted for the loop to handle. */
static void signals_install(void)
{
    atexit(cleanup);
    signal(SIGINT,  sig_h);
    signal(SIGTERM, sig_h);
    signal(SIGWINCH, sig_h);
}

/* Put the terminal into the usual interactive mode: raw keys, no echo, no
 * cursor, non-blocking reads.  typeahead(-1) stops pending input from
 * tearing the screen mid-draw. */
static void terminal_init(void)
{
    initscr();
    cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}

/* Set up the starting Scene: first algorithm, default speed, the current
 * terminal size, and a freshly shuffled array.  Run once at startup, after
 * terminal_init. */
static void scene_init(Scene *sc)
{
    sc->alg   = ALG_BUBBLE;
    sc->speed = STEPS_DEFAULT;
    getmaxyx(stdscr, sc->rows, sc->cols);
    scramble(sc);
}

/* If the terminal was resized, restart ncurses so it matches the new size,
 * then update our cached rows/cols. */
static void handle_resize_if_pending(Scene *sc)
{
    if (!g_resize) return;
    g_resize = 0;
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

/* Grab at most one pending keypress and act on it (getch returns ERR when
 * none is waiting, since input is non-blocking). */
static void drain_input(Scene *sc)
{
    int ch = getch();
    if (ch != ERR) handle_input(sc, ch);
}

/* Draw one frame and flush it to the terminal in a single diffed write. */
static void frame_render(const Scene *sc)
{
    erase();
    scene_draw(sc);
    wnoutrefresh(stdscr);
    doupdate();
}

/* Sleep out the rest of the frame's time budget so we hold TARGET_FPS.  If
 * the frame already ran long, the sleep is skipped. */
static void wait_until_next_frame(long long frame_start_ns)
{
    long long elapsed = clock_ns() - frame_start_ns;
    clock_sleep_ns(FRAME_NS - elapsed);
}

/*
 * Start everything up, then loop until quit: handle any resize, read a key,
 * advance the sort, draw, and wait for the next frame.  The Scene lives on
 * the stack, so the program never calls malloc.
 */
int main(void)
{
    Scene sc = {0};
    srand((unsigned)time(NULL));

    signals_install();
    terminal_init();
    scene_init(&sc);

    while (!g_quit) {
        long long frame_start = clock_ns();
        handle_resize_if_pending(&sc);
        drain_input(&sc);
        step_simulation(&sc);
        frame_render(&sc);
        wait_until_next_frame(frame_start);
    }
    return 0;
}
